#include <cstdint>
#include <cstdio>
#include <vector>

#include <verilated.h>
#include "Vq700_eth_stream_share.h"

static Vq700_eth_stream_share *dut;
static uint64_t cycles;
static bool slow_block_rx;
static bool block_rx_stalled;
static unsigned loopback_completions;
static std::vector<std::vector<uint8_t>> sonic_rx_frames;
static std::vector<std::vector<uint8_t>> block_rx_frames;
static std::vector<std::vector<uint8_t>> mac_tx_frames;
static std::vector<uint8_t> sonic_rx_frame;
static std::vector<uint8_t> block_rx_frame;
static std::vector<uint8_t> mac_tx_frame;

struct Fires {
    bool mac_rx;
    bool sonic_tx;
    bool block_tx;
};

static Fires tick() {
    dut->sonic_rx_tready = 1;
    dut->blk_rx_tready = !block_rx_stalled &&
        (!slow_block_rx || ((cycles % 4) == 0));
    dut->mac_tx_tready = ((cycles % 5) != 2);
    dut->clk = 0;
    dut->eval();

    Fires fire = {
        bool(dut->mac_rx_tvalid && dut->mac_rx_tready),
        bool(dut->sonic_tx_tvalid && dut->sonic_tx_tready),
        bool(dut->blk_tx_tvalid && dut->blk_tx_tready)
    };
    if (dut->sonic_rx_tvalid && dut->sonic_rx_tready) {
        sonic_rx_frame.push_back(dut->sonic_rx_tdata);
        if (dut->sonic_rx_tlast) {
            sonic_rx_frames.push_back(sonic_rx_frame);
            sonic_rx_frame.clear();
        }
    }
    if (dut->blk_rx_tvalid && dut->blk_rx_tready) {
        block_rx_frame.push_back(dut->blk_rx_tdata);
        if (dut->blk_rx_tlast) {
            block_rx_frames.push_back(block_rx_frame);
            block_rx_frame.clear();
        }
    }
    if (dut->mac_tx_tvalid && dut->mac_tx_tready) {
        mac_tx_frame.push_back(dut->mac_tx_tdata);
        if (dut->mac_tx_tlast) {
            mac_tx_frames.push_back(mac_tx_frame);
            mac_tx_frame.clear();
        }
    }
    if (dut->sonic_loopback_cpl)
        ++loopback_completions;

    dut->clk = 1;
    dut->eval();
    ++cycles;
    return fire;
}

static void idle(unsigned count = 20) {
    dut->mac_rx_tvalid = 0;
    dut->sonic_tx_tvalid = 0;
    dut->blk_tx_tvalid = 0;
    for (unsigned i = 0; i < count; ++i)
        tick();
}

static void reset() {
    cycles = 0;
    slow_block_rx = false;
    block_rx_stalled = false;
    loopback_completions = 0;
    sonic_rx_frames.clear();
    block_rx_frames.clear();
    mac_tx_frames.clear();
    sonic_rx_frame.clear();
    block_rx_frame.clear();
    mac_tx_frame.clear();
    dut->loopback = 0;
    dut->rx_drain = 0;
    dut->blk_mac = 0x020000000042ULL;
    dut->mac_rx_tvalid = 0;
    dut->mac_rx_tdata = 0;
    dut->mac_rx_tlast = 0;
    dut->mac_rx_tuser = 0;
    dut->sonic_tx_tvalid = 0;
    dut->sonic_tx_tdata = 0;
    dut->sonic_tx_tlast = 0;
    dut->blk_tx_tvalid = 0;
    dut->blk_tx_tdata = 0;
    dut->blk_tx_tlast = 0;
    dut->rst = 1;
    tick();
    tick();
    dut->rst = 0;
    idle(3);
}

static bool same_frame(const std::vector<uint8_t> &got,
                       const std::vector<uint8_t> &want,
                       const char *name) {
    if (got.size() != want.size()) {
        std::printf("FAIL %s length: got %zu expected %zu\n", name,
                    got.size(), want.size());
        return false;
    }
    for (unsigned i = 0; i < got.size(); ++i) {
        if (got[i] != want[i]) {
            std::printf("FAIL %s byte %u: got %02x expected %02x\n",
                        name, i, got[i], want[i]);
            return false;
        }
    }
    return true;
}

static void send_mac_frame(const std::vector<uint8_t> &frame,
                           int change_drain_at = -1, bool drain_value = false) {
    unsigned index = 0;
    for (unsigned timeout = 0; index < frame.size() && timeout < 10000; ++timeout) {
        if (int(index) == change_drain_at)
            dut->rx_drain = drain_value;
        dut->mac_rx_tvalid = 1;
        dut->mac_rx_tdata = frame[index];
        dut->mac_rx_tlast = (index + 1 == frame.size());
        Fires fire = tick();
        if (fire.mac_rx)
            ++index;
    }
    dut->mac_rx_tvalid = 0;
    dut->mac_rx_tlast = 0;
    if (index != frame.size())
        std::printf("FAIL MAC RX input timeout at byte %u\n", index);
    idle();
}

static void offer_two_tx_frames(const std::vector<uint8_t> &sonic,
                                const std::vector<uint8_t> &block) {
    unsigned si = 0, bi = 0;
    for (unsigned timeout = 0;
         (si < sonic.size() || bi < block.size()) && timeout < 10000; ++timeout) {
        dut->sonic_tx_tvalid = (si < sonic.size());
        dut->sonic_tx_tdata = (si < sonic.size()) ? sonic[si] : 0;
        dut->sonic_tx_tlast = (si + 1 == sonic.size());
        dut->blk_tx_tvalid = (bi < block.size());
        dut->blk_tx_tdata = (bi < block.size()) ? block[bi] : 0;
        dut->blk_tx_tlast = (bi + 1 == block.size());
        Fires fire = tick();
        if (fire.sonic_tx)
            ++si;
        if (fire.block_tx)
            ++bi;
    }
    dut->sonic_tx_tvalid = 0;
    dut->blk_tx_tvalid = 0;
    idle();
    if (si != sonic.size() || bi != block.size())
        std::printf("FAIL simultaneous TX input timeout sonic=%u block=%u\n", si, bi);
}

static bool test_rx_demux() {
    reset();
    const std::vector<uint8_t> to_block =
        {0x02,0x00,0x00,0x00,0x00,0x42,0x10,0x11,0x12,0x13};
    const std::vector<uint8_t> to_sonic =
        {0x02,0x00,0x00,0x00,0x00,0x99,0x20,0x21,0x22,0x23};
    const std::vector<uint8_t> broadcast =
        {0xff,0xff,0xff,0xff,0xff,0xff,0x30,0x31,0x32,0x33};

    send_mac_frame(to_block);
    send_mac_frame(to_sonic);
    slow_block_rx = true;
    send_mac_frame(broadcast);
    slow_block_rx = false;

    bool ok = true;
    if (block_rx_frames.size() != 2 || sonic_rx_frames.size() != 2) {
        std::printf("FAIL RX demux counts: block=%zu sonic=%zu expected 2/2\n",
                    block_rx_frames.size(), sonic_rx_frames.size());
        return false;
    }
    ok &= same_frame(block_rx_frames[0], to_block, "block unicast RX");
    ok &= same_frame(sonic_rx_frames[0], to_sonic, "SONIC unicast RX");
    ok &= same_frame(block_rx_frames[1], broadcast, "block broadcast RX");
    ok &= same_frame(sonic_rx_frames[1], broadcast, "SONIC broadcast RX");
    return ok;
}

static bool test_tx_frame_grant() {
    reset();
    const std::vector<uint8_t> sonic = {0xa0,0xa1,0xa2,0xa3,0xa4,0xa5};
    const std::vector<uint8_t> block = {0xb0,0xb1,0xb2,0xb3,0xb4};
    offer_two_tx_frames(sonic, block);
    if (mac_tx_frames.size() != 2) {
        std::printf("FAIL simultaneous TX frame count: got %zu expected 2\n",
                    mac_tx_frames.size());
        return false;
    }
    bool ok = same_frame(mac_tx_frames[0], sonic, "simultaneous TX frame 0");
    ok &= same_frame(mac_tx_frames[1], block, "simultaneous TX frame 1");
    return ok;
}

static bool test_drain_boundaries() {
    reset();
    const std::vector<uint8_t> first =
        {0x02,0,0,0,0,0x91,0x41,0x42,0x43,0x44};
    const std::vector<uint8_t> second =
        {0x02,0,0,0,0,0x92,0x51,0x52,0x53,0x54};
    const std::vector<uint8_t> third =
        {0x02,0,0,0,0,0x93,0x61,0x62,0x63,0x64};
    const std::vector<uint8_t> fourth =
        {0x02,0,0,0,0,0x94,0x71,0x72,0x73,0x74};

    send_mac_frame(first, 7, true);   // change mid-frame: still delivered
    send_mac_frame(second);           // next whole frame is drained
    send_mac_frame(third, 7, false);  // change mid-frame: still drained
    send_mac_frame(fourth);           // next whole frame is delivered
    if (sonic_rx_frames.size() != 2) {
        std::printf("FAIL drain frame-boundary count: got %zu expected 2\n",
                    sonic_rx_frames.size());
        return false;
    }
    return same_frame(sonic_rx_frames[0], first, "drain current frame") &&
           same_frame(sonic_rx_frames[1], fourth, "drain next enabled frame");
}

static bool test_loopback_is_sonic_local() {
    reset();
    dut->loopback = 1;
    const std::vector<uint8_t> sonic = {0xc0,0xc1,0xc2,0xc3};
    const std::vector<uint8_t> block = {0xd0,0xd1,0xd2,0xd3,0xd4};
    offer_two_tx_frames(sonic, block);
    if (sonic_rx_frames.size() != 1 || mac_tx_frames.size() != 1 ||
        loopback_completions != 1) {
        std::printf("FAIL loopback TX counts: sonic_rx=%zu mac_tx=%zu cpl=%u expected 1/1/1\n",
                    sonic_rx_frames.size(), mac_tx_frames.size(),
                    loopback_completions);
        return false;
    }
    bool ok = same_frame(sonic_rx_frames[0], sonic, "SONIC loopback RX");
    ok &= same_frame(mac_tx_frames[0], block, "block TX during loopback");

    const std::vector<uint8_t> wire_block =
        {0x02,0,0,0,0,0x42,0xe0,0xe1,0xe2,0xe3};
    const std::vector<uint8_t> wire_sonic =
        {0x02,0,0,0,0,0x95,0xf0,0xf1,0xf2,0xf3};
    send_mac_frame(wire_block);
    send_mac_frame(wire_sonic);
    if (block_rx_frames.size() != 1 || sonic_rx_frames.size() != 1) {
        std::printf("FAIL loopback wire RX counts: block=%zu sonic=%zu expected 1/1\n",
                    block_rx_frames.size(), sonic_rx_frames.size());
        return false;
    }
    ok &= same_frame(block_rx_frames[0], wire_block,
                     "block RX during SONIC loopback");
    return ok;
}


// A tied-off block client (blk_mac all zeroes) must be invisible to routing.
// The dangerous case is a strap that also holds blk_rx_tready low: broadcast
// fan-out waits for every routed consumer, so counting an absent client would
// wedge the shared wire stream and starve the SONIC of ALL traffic.
static bool test_block_client_absent() {
    reset();
    dut->blk_mac = 0;
    block_rx_stalled = true;
    const std::vector<uint8_t> unicast =
        {0x02,0x00,0x00,0x00,0x00,0x42,0x10,0x11,0x12,0x13};
    const std::vector<uint8_t> broadcast =
        {0xff,0xff,0xff,0xff,0xff,0xff,0x30,0x31,0x32,0x33};
    // 0x02..42 would match the configured blk_mac in every other test; with no
    // client attached it must fall through to the SONIC like any other unicast.
    send_mac_frame(unicast);
    send_mac_frame(broadcast);
    if (sonic_rx_frames.size() != 2 || !block_rx_frames.empty()) {
        std::printf("FAIL absent block client: sonic=%zu block=%zu expected 2/0\n",
                    sonic_rx_frames.size(), block_rx_frames.size());
        return false;
    }
    bool ok = same_frame(sonic_rx_frames[0], unicast, "absent-client unicast RX");
    ok &= same_frame(sonic_rx_frames[1], broadcast, "absent-client broadcast RX");
    return ok;
}

// loopback asserted part-way through a SONIC frame that is already streaming
// into the MAC.  The frame must finish on the wire (a truncated frame would sit
// in the MAC frame FIFO forever), and the block client must still get the MAC
// afterwards -- the failure mode being guarded is a grant parked on a SONIC
// stream that has stopped presenting wire beats.
static bool test_loopback_asserted_mid_tx_frame() {
    reset();
    const std::vector<uint8_t> sonic = {0xa0,0xa1,0xa2,0xa3,0xa4,0xa5};
    const std::vector<uint8_t> block = {0xb0,0xb1,0xb2,0xb3,0xb4};
    unsigned si = 0, bi = 0;
    for (unsigned timeout = 0;
         (si < sonic.size() || bi < block.size()) && timeout < 10000; ++timeout) {
        dut->sonic_tx_tvalid = (si < sonic.size());
        dut->sonic_tx_tdata = (si < sonic.size()) ? sonic[si] : 0;
        dut->sonic_tx_tlast = (si + 1 == sonic.size());
        dut->blk_tx_tvalid = (bi < block.size());
        dut->blk_tx_tdata = (bi < block.size()) ? block[bi] : 0;
        dut->blk_tx_tlast = (bi + 1 == block.size());
        if (si == 2)
            dut->loopback = 1;      // mid-frame, after real wire beats
        Fires fire = tick();
        if (fire.sonic_tx) ++si;
        if (fire.block_tx) ++bi;
    }
    dut->sonic_tx_tvalid = 0;
    dut->blk_tx_tvalid = 0;
    idle();
    if (si != sonic.size() || bi != block.size()) {
        std::printf("FAIL mid-frame loopback starved TX: sonic=%u block=%u\n", si, bi);
        return false;
    }
    if (mac_tx_frames.size() != 2 || !sonic_rx_frames.empty() ||
        loopback_completions != 0) {
        std::printf("FAIL mid-frame loopback routing: mac_tx=%zu sonic_rx=%zu cpl=%u "
                    "expected 2/0/0\n", mac_tx_frames.size(),
                    sonic_rx_frames.size(), loopback_completions);
        return false;
    }
    bool ok = same_frame(mac_tx_frames[0], sonic, "in-flight SONIC frame on the wire");
    ok &= same_frame(mac_tx_frames[1], block, "block TX after mid-frame loopback");
    if (!ok) return false;

    // The next SONIC frame starts after the boundary, so it loops instead.
    const std::vector<uint8_t> looped = {0xc0,0xc1,0xc2,0xc3};
    offer_two_tx_frames(looped, {});
    if (sonic_rx_frames.size() != 1 || loopback_completions != 1) {
        std::printf("FAIL post-boundary loopback: sonic_rx=%zu cpl=%u expected 1/1\n",
                    sonic_rx_frames.size(), loopback_completions);
        return false;
    }
    return same_frame(sonic_rx_frames[0], looped, "post-boundary looped frame");
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vq700_eth_stream_share;
    bool ok = test_rx_demux();
    ok &= test_tx_frame_grant();
    ok &= test_drain_boundaries();
    ok &= test_loopback_is_sonic_local();
    ok &= test_block_client_absent();
    ok &= test_loopback_asserted_mid_tx_frame();
    delete dut;
    if (!ok)
        return 1;
    std::printf("PASS q700_eth_stream_share: RX demux/broadcast, frame TX arbitration, "
                "drain, loopback, absent block client, mid-frame loopback\n");
    return 0;
}
