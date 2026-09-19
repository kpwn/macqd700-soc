#include <cstdint>
#include <cstdio>
#include <vector>

#include <verilated.h>
#include "Vnet_block_framer.h"

static Vnet_block_framer *dut;
static uint64_t cycles;
static std::vector<uint8_t> reply_bytes;
static unsigned reply_events;
static uint16_t last_reply_tag;
static uint8_t last_reply_status;

static uint16_t ip_checksum(const std::vector<uint8_t> &header) {
    uint32_t sum = 0;
    for (unsigned i = 0; i < header.size(); i += 2)
        sum += (uint16_t(header[i]) << 8) | header[i + 1];
    while (sum >> 16)
        sum = (sum & 0xffff) + (sum >> 16);
    return uint16_t(~sum);
}

static void tick() {
    dut->clk = 0;
    dut->reply_payload_tready = ((cycles % 4) != 1);
    dut->eval();
    const bool metadata_fire = dut->reply_valid && dut->reply_ready;
    const bool payload_fire = dut->reply_payload_tvalid && dut->reply_payload_tready;
    if (metadata_fire) {
        ++reply_events;
        last_reply_tag = dut->reply_tag;
        last_reply_status = dut->reply_status;
    }
    if (payload_fire)
        reply_bytes.push_back(dut->reply_payload_tdata);
    dut->clk = 1;
    dut->eval();
    ++cycles;
}

static bool compare_frame(const std::vector<uint8_t> &got,
                          const std::vector<uint8_t> &expected,
                          const char *name) {
    if (got.size() != expected.size()) {
        std::printf("FAIL %s length: got %zu expected %zu\n", name,
                    got.size(), expected.size());
        return false;
    }
    for (unsigned i = 0; i < got.size(); ++i) {
        if (got[i] != expected[i]) {
            std::printf("FAIL %s frame mismatch at byte %u: got %02x expected %02x\n",
                        name, i, got[i], expected[i]);
            return false;
        }
    }
    return true;
}

static std::vector<uint8_t> transmit_request(uint8_t op, uint32_t lba,
                                              uint16_t count, uint16_t tag,
                                              const std::vector<uint8_t> &payload) {
    dut->req_op = op;
    dut->req_lba = lba;
    dut->req_block_count = count;
    dut->req_tag = tag;
    dut->req_valid = 1;
    unsigned payload_index = 0;
    bool request_pending = true;
    bool saw_last = false;
    std::vector<uint8_t> frame;

    for (unsigned limit = 0; limit < 10000 && !saw_last; ++limit) {
        dut->tx_tready = ((cycles % 5) != 2);
        if (payload_index < payload.size()) {
            dut->req_payload_tvalid = 1;
            dut->req_payload_tdata = payload[payload_index];
            dut->req_payload_tlast = (payload_index + 1 == payload.size());
        } else {
            dut->req_payload_tvalid = 0;
            dut->req_payload_tdata = 0;
            dut->req_payload_tlast = 0;
        }

        dut->clk = 0;
        dut->eval();
        const bool request_fire = dut->req_valid && dut->req_ready;
        const bool payload_fire = dut->req_payload_tvalid && dut->req_payload_tready;
        const bool tx_fire = dut->tx_tvalid && dut->tx_tready;
        if (tx_fire) {
            frame.push_back(dut->tx_tdata);
            saw_last = dut->tx_tlast;
        }
        dut->clk = 1;
        dut->eval();
        ++cycles;
        if (request_fire) {
            request_pending = false;
            dut->req_valid = 0;
        }
        if (payload_fire)
            ++payload_index;
    }
    dut->req_valid = 0;
    dut->req_payload_tvalid = 0;
    dut->tx_tready = 1;
    if (request_pending || !saw_last)
        std::printf("FAIL request transmit timeout op=%u count=%u\n", op, count);
    return frame;
}

static std::vector<uint8_t> make_reply(uint16_t tag, uint8_t status,
                                       const std::vector<uint8_t> &payload) {
    const uint16_t udp_len = uint16_t(8 + 7 + payload.size());
    const uint16_t ip_len = uint16_t(20 + udp_len);
    std::vector<uint8_t> f = {
        0x02,0x00,0x00,0x00,0x00,0x01, 0x02,0x00,0x00,0x00,0x00,0x02,
        0x08,0x00,
        0x45,0x00, uint8_t(ip_len >> 8),uint8_t(ip_len), 0x22,0x22,
        0x00,0x00, 0x40,0x11, 0x00,0x00,
        0xc0,0xa8,0x0a,0x01, 0xc0,0xa8,0x0a,0x02,
        0x51,0x51, 0x42,0x42, uint8_t(udp_len >> 8),uint8_t(udp_len), 0x00,0x00,
        0x4e,0x42,0x48,0x44, uint8_t(tag >> 8),uint8_t(tag), status
    };
    std::vector<uint8_t> ip(f.begin() + 14, f.begin() + 34);
    const uint16_t checksum = ip_checksum(ip);
    f[24] = checksum >> 8;
    f[25] = checksum;
    f.insert(f.end(), payload.begin(), payload.end());
    return f;
}

static void send_frame(const std::vector<uint8_t> &frame) {
    for (unsigned i = 0; i < frame.size(); ++i) {
        dut->rx_tdata = frame[i];
        dut->rx_tvalid = 1;
        dut->rx_tlast = (i + 1 == frame.size());
        unsigned limit = 1000;
        while (limit--) {
            dut->clk = 0;
            dut->eval();
            if (dut->rx_tready)
                break;
            tick();
        }
        if (!dut->rx_tready) {
            std::printf("FAIL RX ready timeout at byte %u\n", i);
            return;
        }
        tick();
    }
    dut->rx_tvalid = 0;
    dut->rx_tlast = 0;
}

static bool reject_and_recover(std::vector<uint8_t> bad, const char *name,
                               const std::vector<uint8_t> &valid) {
    const unsigned before = reply_events;
    send_frame(bad);
    for (unsigned i = 0; i < 8; ++i)
        tick();
    if (reply_events != before || !dut->rx_tready) {
        std::printf("FAIL rejected %s produced a reply or poisoned RX state\n", name);
        return false;
    }
    send_frame(valid);
    for (unsigned i = 0; i < 20 && reply_events == before; ++i)
        tick();
    if (reply_events != before + 1 || last_reply_tag != 0x7654) {
        std::printf("FAIL RX did not recover after rejected %s\n", name);
        return false;
    }
    while (!dut->rx_tready)
        tick();
    return true;
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vnet_block_framer;
    dut->our_mac = 0x020000000001ULL;
    dut->dst_mac = 0x020000000002ULL;
    dut->our_ip = 0xc0a80a02;
    dut->dst_ip = 0xc0a80a01;
    dut->our_port = 0x4242;
    dut->dst_port = 0x5151;
    dut->req_valid = 0;
    dut->req_payload_tvalid = 0;
    dut->req_payload_tlast = 0;
    dut->tx_tready = 1;
    dut->rx_tvalid = 0;
    dut->rx_tlast = 0;
    dut->reply_ready = 1;
    dut->rst = 1;
    for (unsigned i = 0; i < 3; ++i)
        tick();
    dut->rst = 0;

    bool ok = true;
    // Byte-exact golden vector, independently summed on paper as:
    // 4500+0029+beef+4000+4011+c0a8+0a02+c0a8+0a01 => checksum e680.
    const std::vector<uint8_t> expected_read = {
        0x02,0x00,0x00,0x00,0x00,0x02, 0x02,0x00,0x00,0x00,0x00,0x01,
        0x08,0x00,
        0x45,0x00,0x00,0x29,0xbe,0xef,0x40,0x00,0x40,0x11,0xe6,0x80,
        0xc0,0xa8,0x0a,0x02,0xc0,0xa8,0x0a,0x01,
        0x42,0x42,0x51,0x51,0x00,0x15,0x00,0x00,
        0x4e,0x42,0x48,0x44,0x00,0x12,0x34,0x56,0x78,0x00,0x02,0xbe,0xef
    };
    ok &= compare_frame(transmit_request(0x00, 0x12345678, 2, 0xbeef, {}),
                        expected_read, "read request");

    std::vector<uint8_t> write_payload(512);
    for (unsigned i = 0; i < write_payload.size(); ++i)
        write_payload[i] = uint8_t((i * 29 + 7) & 0xff);
    const auto write_frame = transmit_request(0x01, 0x01020304, 1, 0x1234,
                                               write_payload);
    if (write_frame.size() != 55 + write_payload.size()) {
        std::printf("FAIL write request length: got %zu expected 567\n", write_frame.size());
        ok = false;
    } else {
        if (write_frame[16] != 0x02 || write_frame[17] != 0x29 ||
            write_frame[38] != 0x02 || write_frame[39] != 0x15 ||
            write_frame[46] != 0x01) {
            std::printf("FAIL write request header lengths/op\n");
            ok = false;
        }
        std::vector<uint8_t> ip(write_frame.begin() + 14, write_frame.begin() + 34);
        if (ip_checksum(ip) != 0) {
            std::printf("FAIL write request IPv4 checksum does not verify\n");
            ok = false;
        }
        for (unsigned i = 0; i < write_payload.size(); ++i) {
            if (write_frame[55 + i] != write_payload[i]) {
                std::printf("FAIL write request payload byte %u\n", i);
                ok = false;
                break;
            }
        }
    }

    const std::vector<uint8_t> valid_payload = {0xde,0xad,0xbe,0xef,0x31};
    const auto valid = make_reply(0x7654, 0x03, valid_payload);
    const unsigned reply_before = reply_events;
    const size_t bytes_before = reply_bytes.size();
    send_frame(valid);
    for (unsigned i = 0; i < 40 && !dut->rx_tready; ++i)
        tick();
    if (reply_events != reply_before + 1 || last_reply_tag != 0x7654 ||
        last_reply_status != 0x03 ||
        std::vector<uint8_t>(reply_bytes.begin() + bytes_before, reply_bytes.end()) != valid_payload) {
        std::printf("FAIL valid reply metadata or payload\n");
        ok = false;
    }

    auto bad = valid;
    bad[12] = 0x86; bad[13] = 0xdd;
    ok &= reject_and_recover(bad, "ethertype", valid);

    bad = valid;
    bad[23] = 0x06;
    bad[24] = bad[25] = 0;
    {
        std::vector<uint8_t> ip(bad.begin() + 14, bad.begin() + 34);
        const uint16_t checksum = ip_checksum(ip);
        bad[24] = checksum >> 8; bad[25] = checksum;
    }
    ok &= reject_and_recover(bad, "IP protocol", valid);

    bad = valid;
    bad[36] ^= 0x01;
    ok &= reject_and_recover(bad, "destination port", valid);

    bad = valid;
    bad[42] ^= 0x80;
    ok &= reject_and_recover(bad, "magic", valid);

    bad.assign(valid.begin(), valid.begin() + 32);
    ok &= reject_and_recover(bad, "truncated frame", valid);

    // req_ready is the seam that knows the MTU, so an over-large request must
    // be refused HERE rather than put on the wire for the host to reject.  A
    // read carries no payload, so its size used to be waved through -- but its
    // REPLY carries block_count*512 back and three blocks (1536) exceeds the
    // 1465-byte reply budget.  Two blocks must still be accepted in both
    // directions, and a zero-block request is meaningless in either.
    {
        struct { uint8_t op; uint16_t count; bool want; const char *what; } cases[] = {
            {0x00, 1, true,  "read 1 block"},
            {0x00, 2, true,  "read 2 blocks (the MTU limit)"},
            {0x00, 3, false, "read 3 blocks (reply cannot fit)"},
            {0x00, 0, false, "read 0 blocks"},
            {0x01, 2, true,  "write 2 blocks"},
            {0x01, 3, false, "write 3 blocks"},
            {0x01, 0, false, "write 0 blocks"},
            {0x00, 0xffff, false, "read 65535 blocks (READ(10) ceiling)"},
        };
        for (auto &c : cases) {
            dut->req_valid = 0; dut->req_op = c.op; dut->req_lba = 0;
            dut->req_block_count = c.count; dut->req_tag = 0;
            dut->rx_tvalid = 0; dut->tx_tready = 1;
            tick(); dut->eval();
            const bool got = dut->req_ready != 0;
            if (got != c.want) {
                std::printf("FAIL req_ready for %s: got %d want %d\n",
                            c.what, got ? 1 : 0, c.want ? 1 : 0);
                ok = false;
            }
        }
        dut->req_block_count = 1; dut->req_op = 0x00;
    }

    if (ok)
        std::printf("PASS net_block_framer: exact read/write framing, IPv4 checksum, reply parsing, rejection and recovery\n");
    delete dut;
    return ok ? 0 : 1;
}
