#ifdef DMA_L2C_BUILD
#include "Vtb_dma_l2c.h"
using DmaTop = Vtb_dma_l2c;
#else
#include "Vdma_engine.h"
using DmaTop = Vdma_engine;
#endif
#include "verilated.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <map>
#include <vector>

#ifndef DATA_WIDTH_BUILD
#define DATA_WIDTH_BUILD 128
#endif

static constexpr int DW = DATA_WIDTH_BUILD;
static constexpr int BUS_BYTES = DW / 8;
static constexpr int CLIENTS = 4;

struct ReadBurst { uint32_t addr; uint8_t id; int beat; int beats; int size; };
struct WriteBurst { uint32_t addr; uint8_t id; int beat; int beats; int size; };

static uint64_t cycles;
static std::array<uint8_t, 4096> mem;
static std::deque<ReadBurst> reads;
static std::deque<WriteBurst> writes;
static std::deque<uint8_t> bq;
static uint64_t live_ids;

static void fail(const char *why) {
    std::fprintf(stderr, "FAIL [%d-bit] cycle %llu: %s\n", DW,
                 static_cast<unsigned long long>(cycles), why);
    std::exit(1);
}

static void set_bits(WData *v, int base, int width, uint64_t value) {
    for (int n = 0; n < width; ++n) {
        const int bit = base + n;
        const uint32_t mask = uint32_t(1) << (bit & 31);
        if ((value >> n) & 1) v[bit >> 5] |= mask;
        else v[bit >> 5] &= ~mask;
    }
}

static uint64_t get_bits(const WData *v, int base, int width) {
    uint64_t value = 0;
    for (int n = 0; n < width; ++n)
        value |= uint64_t((v[(base + n) >> 5] >> ((base + n) & 31)) & 1) << n;
    return value;
}

// Memory-lane permutation.  With -GBYTE_SWAP32=1 the engine speaks the SoC's
// convention: the lowest-address byte of each 32-bit group rides bits [31:24]
// of its lane, so raw AXI lane L carries the byte at bus_base + (L ^ 3).  The
// permutation is an involution, so one helper covers both directions.  Building
// this model with BYTE_SWAP32_BUILD but WITHOUT -GBYTE_SWAP32=1 must FAIL --
// that pairing is the mutant `tb-dma-engine-swap-mut` checks.
static inline int mem_lane(int lane) {
#ifdef BYTE_SWAP32_BUILD
    return lane ^ 3;
#else
    return lane;
#endif
}

static void drive_slave(DmaTop &d) {
    d.m_awready = 1;
    d.m_wready = 1;
    d.m_arready = 1;
    d.m_bvalid = !bq.empty();
    d.m_bid = bq.empty() ? 0 : bq.front();
    d.m_bresp = 0;

    d.m_rvalid = !reads.empty();
    d.m_rid = reads.empty() ? 0 : reads.front().id;
    d.m_rresp = 0;
    d.m_rlast = !reads.empty() && reads.front().beat == reads.front().beats - 1;
    for (int i = 0; i < DW / 32; ++i) d.m_rdata[i] = 0;
    if (!reads.empty()) {
        const auto &r = reads.front();
        const int beat_bytes = 1 << r.size;
        const uint32_t beat_addr = r.addr + r.beat * beat_bytes;
        const int lane_base = beat_addr & (BUS_BYTES - 1);
        for (int n = 0; n < beat_bytes; ++n) {
            const int lane = lane_base + n;
            if (lane < BUS_BYTES)
                set_bits(d.m_rdata, mem_lane(lane) * 8, 8, mem.at(beat_addr + n));
        }
    }
}

static void tick(DmaTop &d) {
    drive_slave(d);
    d.clk = 0;
    d.eval();

    const bool aw = d.m_awvalid && d.m_awready;
    const bool w = d.m_wvalid && d.m_wready;
    const bool ar = d.m_arvalid && d.m_arready;
    const bool b = d.m_bvalid && d.m_bready;
    const bool r = d.m_rvalid && d.m_rready;
    const bool rlast = r && d.m_rlast;

    const uint8_t awid = d.m_awid & 63;
    const uint8_t arid = d.m_arid & 63;
    const uint8_t bid = d.m_bid & 63;
    const uint8_t rid = d.m_rid & 63;

    if (b) {
#ifndef DMA_L2C_BUILD
        if (!(live_ids & (1ull << bid))) fail("B response used a non-live ID");
        live_ids &= ~(1ull << bid);
#endif
        bq.pop_front();
    }
    if (rlast) {
#ifndef DMA_L2C_BUILD
        if (!(live_ids & (1ull << rid))) fail("R response used a non-live ID");
        live_ids &= ~(1ull << rid);
#endif
    }
    if (r) {
        if (reads.empty()) fail("R handshake without queued read");
        if (d.m_rlast) reads.pop_front();
        else ++reads.front().beat;
    }
    if (aw) {
#ifndef DMA_L2C_BUILD
        if (live_ids & (1ull << awid)) fail("AXI ID reused by AW");
        live_ids |= 1ull << awid;
#endif
        writes.push_back({static_cast<uint32_t>(d.m_awaddr), awid, 0,
                          static_cast<int>(d.m_awlen) + 1, static_cast<int>(d.m_awsize)});
        if (writes.back().addr == 0x140 && writes.back().beats != 64 / BUS_BYTES)
            fail("aligned 64-byte write did not use the minimum beat count");
    }
    if (ar) {
#ifndef DMA_L2C_BUILD
        if (live_ids & (1ull << arid)) fail("AXI ID reused by AR");
        live_ids |= 1ull << arid;
#endif
        reads.push_back({static_cast<uint32_t>(d.m_araddr), arid, 0,
                         static_cast<int>(d.m_arlen) + 1, static_cast<int>(d.m_arsize)});
        if (d.m_araddr == 0x140 && d.m_arlen + 1 != 64 / BUS_BYTES)
            fail("aligned 64-byte read did not use the minimum beat count");
    }
    if (w) {
        if (writes.empty()) fail("W beat without AW");
        auto &write_burst = writes.front();
        const int beat_bytes = 1 << write_burst.size;
        const uint32_t beat_addr = write_burst.addr + write_burst.beat * beat_bytes;
        const uint32_t bus_base = beat_addr & ~(uint32_t(BUS_BYTES) - 1);
        for (int lane = 0; lane < BUS_BYTES; ++lane) {
            if ((d.m_wstrb >> lane) & 1) {
                const uint32_t a = bus_base + mem_lane(lane);
                mem.at(a) = static_cast<uint8_t>(get_bits(d.m_wdata, lane * 8, 8));
            }
        }
        const bool expected_last = write_burst.beat == write_burst.beats - 1;
        if (bool(d.m_wlast) != expected_last) fail("WLAST at wrong beat");
        if (d.m_wlast) {
            bq.push_back(write_burst.id);
            writes.pop_front();
        } else ++write_burst.beat;
    }
    d.clk = 1;
    d.eval();
    ++cycles;
}

static void clear_requests(DmaTop &d) {
    d.req_valid = 0;
    d.req_write = 0;
    for (int i = 0; i < CLIENTS; ++i) d.req_addr[i] = 0;
    d.req_len = 0;
    for (int i = 0; i < CLIENTS * 512 / 32; ++i) d.req_wdata[i] = 0;
    d.req_tag = 0;
}

static void submit(DmaTop &d, int client, bool write, uint32_t addr,
                   int len, uint8_t tag, uint8_t seed) {
    clear_requests(d);
    set_bits(d.req_addr, client * 32, 32, addr);
    set_bits(&d.req_len, client * 7, 7, len);
    set_bits(&d.req_tag, client * 8, 8, tag);
    if (write) {
        d.req_write = 1u << client;
        for (int n = 0; n < len; ++n)
            set_bits(d.req_wdata, client * 512 + n * 8, 8, uint8_t(seed + n));
    }
    d.req_valid = 1u << client;
    drive_slave(d);
    d.clk = 0;
    d.eval();
    if (!(d.req_ready & (1u << client))) fail("request was not accepted at queue ingress");
    tick(d);
    clear_requests(d);
}

static void wait_responses(DmaTop &d, std::map<uint8_t, std::vector<uint8_t>> expected,
                           bool writes) {
    for (int timeout = 0; timeout < 20000 && !expected.empty(); ++timeout) {
        drive_slave(d); d.clk = 0; d.eval();
        for (int c = 0; c < CLIENTS; ++c) if (d.rsp_valid & (1u << c)) {
            const uint8_t tag = (d.rsp_tag >> (c * 8)) & 0xff;
            auto it = expected.find(tag);
            if (it == expected.end()) fail("unexpected or duplicate completion tag");
            if (bool((d.rsp_write >> c) & 1) != writes) fail("wrong completion direction");
            if (get_bits(&d.rsp_len, c * 7, 7) != int(it->second.size())) fail("wrong completion length");
            if (((d.rsp_status >> (c * 2)) & 3) != 0) fail("unexpected AXI error response");
            if (!writes) for (size_t n = 0; n < it->second.size(); ++n)
                if (get_bits(d.rsp_rdata, c * 512 + n * 8, 8) != it->second[n])
                    fail("read payload mismatch");
            expected.erase(it);
        }
        tick(d);
    }
    if (!expected.empty()) fail("completion timeout");
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    DmaTop d;
    clear_requests(d);
    d.rsp_ready = 0xf;
    d.rst = 1;
    for (int i = 0; i < 4; ++i) tick(d);
    d.rst = 0;
    for (size_t i = 0; i < mem.size(); ++i) mem[i] = uint8_t((i * 37 + 11) & 0xff);
#ifdef DMA_L2C_BUILD
    for (int i = 0; i < 4200; ++i) tick(d);
#endif

    const uint64_t first_write_cycle = cycles;
    submit(d, 0, true, 0x103, 5, 0x10, 0xa0);
    submit(d, 1, true, 0x140, 64, 0x11, 0x20);
    submit(d, 2, true, 0x1bf, 2, 0x12, 0xe0);
    if (cycles != first_write_cycle + 3) fail("write requests were not accepted one per cycle");
    wait_responses(d, {{0x10, std::vector<uint8_t>(5)},
                       {0x11, std::vector<uint8_t>(64)},
                       {0x12, std::vector<uint8_t>(2)}}, true);

#ifndef DMA_L2C_BUILD
    for (int n = 0; n < 5; ++n) if (mem[0x103 + n] != uint8_t(0xa0 + n)) fail("short write mismatch");
    for (int n = 0; n < 64; ++n) if (mem[0x140 + n] != uint8_t(0x20 + n)) fail("line write mismatch");
    if (mem[0x1bf] != 0xe0 || mem[0x1c0] != 0xe1) fail("boundary-crossing write mismatch");
#endif

    std::vector<uint8_t> short_data(5), line_data(64), cross_data(2);
    for (int n = 0; n < 5; ++n) short_data[n] = uint8_t(0xa0+n);
    for (int n = 0; n < 64; ++n) line_data[n] = uint8_t(0x20+n);
    cross_data[0] = 0xe0; cross_data[1] = 0xe1;
    const uint64_t first_read_cycle = cycles;
    submit(d, 3, false, 0x103, 5, 0x20, 0);
    submit(d, 0, false, 0x140, 64, 0x21, 0);
    submit(d, 1, false, 0x1bf, 2, 0x22, 0);
    if (cycles != first_read_cycle + 3) fail("read requests were not accepted one per cycle");
    wait_responses(d, {{0x20, short_data}, {0x21, line_data}, {0x22, cross_data}}, false);

    std::map<uint8_t, std::vector<uint8_t>> perf_writes;
    const uint64_t perf_write_start = cycles;
    d.rsp_ready = 0;
    for (int line = 0; line < 8; ++line) {
        const uint8_t tag = uint8_t(0x40 + line);
        submit(d, line & 3, true, 0x400 + line * 64, 64, tag, uint8_t(line * 17));
        perf_writes.emplace(tag, std::vector<uint8_t>(64));
    }
    d.rsp_ready = 0xf;
    wait_responses(d, perf_writes, true);
    const uint64_t perf_write_cycles = cycles - perf_write_start;

    std::map<uint8_t, std::vector<uint8_t>> perf_reads;
    const uint64_t perf_read_start = cycles;
    d.rsp_ready = 0;
    for (int line = 0; line < 8; ++line) {
        const uint8_t tag = uint8_t(0x60 + line);
        std::vector<uint8_t> data(64);
        for (int n = 0; n < 64; ++n) data[n] = uint8_t(line * 17 + n);
        submit(d, line & 3, false, 0x400 + line * 64, 64, tag, 0);
        perf_reads.emplace(tag, std::move(data));
    }
    d.rsp_ready = 0xf;
    wait_responses(d, perf_reads, false);
    const uint64_t perf_read_cycles = cycles - perf_read_start;

    if (!writes.empty() || !reads.empty() || !bq.empty()) fail("AXI transaction leaked");
#ifndef DMA_L2C_BUILD
    if (live_ids) fail("AXI ID leaked");
#endif
#ifdef DMA_L2C_BUILD
    std::printf("PASS dma_engine->L2C: byte-granular R/W, 64B fast path, 1 request/cycle; "
                "8-line write %.2f cyc/128b-word, read %.2f cyc/128b-word\n",
                perf_write_cycles / 32.0, perf_read_cycles / 32.0);
#else
    // ── THE SONIC RX WRITE PATTERN ────────────────────────────────────
    // A received Ethernet frame is written as one 64-byte request per
    // captured chunk, back-to-back, with a SHORT final request for the
    // remainder.  With QUEUE_DEPTH=16 the 17th request is the first to meet
    // back-pressure -- and on hardware the delivered frame is byte-correct
    // through chunk 16 and STALE from chunk 17 onward.  Nothing in this
    // bench previously issued more than three writes in a row, so the
    // stalling path was never exercised.
    {
        const uint32_t base = 0x400;   // model memory is 4 KiB
        const int full = 17, tail = 5;
        // Distinct content per chunk so a stale or misaddressed write shows
        // up as the WRONG BYTES, not merely a missing completion.
        std::map<uint8_t, std::vector<uint8_t>> expect;
        for (int c = 0; c <= full; ++c) {
            const int len = (c == full) ? tail : 64;
            std::vector<uint8_t> chunk(len);
            for (int n = 0; n < len; ++n) chunk[n] = uint8_t(c * 7 + n);
            // Submit, honouring back-pressure the way the RX engine does.
            int guard = 20000;
            while (guard--) {
                clear_requests(d);
                set_bits(d.req_addr, 0 * 32, 32, base + c * 64);
                set_bits(&d.req_len, 0 * 7, 7, len);
                set_bits(&d.req_tag, 0 * 8, 8, uint8_t(0x40 + c));
                d.req_write = 1u;
                for (int n = 0; n < len; ++n)
                    set_bits(d.req_wdata, 0 * 512 + n * 8, 8, chunk[n]);
                d.req_valid = 1u;
                drive_slave(d); d.clk = 0; d.eval(); d.eval();
                const bool accepted = (d.req_ready & 1u) != 0;
                tick(d); clear_requests(d);
                if (accepted) break;
            }
            if (guard <= 0) fail("RX pattern: engine never accepted a chunk write");
            expect[uint8_t(0x40 + c)] = chunk;
        }
        // Do not use wait_responses here: completions fire as pulses DURING
        // the submit loop above and would be missed, timing out on tags that
        // already landed.  What matters is whether the DATA arrives, so let
        // the queue drain and then read memory.
        for (int q = 0; q < 8000; ++q) { drive_slave(d); d.clk = 0; d.eval(); tick(d); }
        (void)expect;
        for (int c = 0; c <= full; ++c) {
            const int len = (c == full) ? tail : 64;
            for (int n = 0; n < len; ++n) {
                const uint8_t want = uint8_t(c * 7 + n);
                const uint8_t got = mem[base + c * 64 + n];
                if (got != want) {
                    std::printf("FAIL RX pattern: chunk %d byte %d = %02x want %02x "
                                "(offset %d)\n", c, n, got, want, c * 64 + n);
                    fail("RX write pattern mismatch");
                }
            }
        }
    }

    std::printf("PASS dma_engine DATA_WIDTH=%d: byte-granular R/W, 64B fast path, 1 request/cycle; "
                "8-line write %.2f cyc/128b-word, read %.2f cyc/128b-word\n",
                DW, perf_write_cycles / 32.0, perf_read_cycles / 32.0);
#endif
    return 0;
}
