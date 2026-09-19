// tb_l2c_sctr.cpp -- SCATTERED / PARTIAL-LINE write measurement for l2c,
// on the same chain tb_l2c_wstream.cpp uses (tb/tb_l2c_wstream.v):
//
//   32-bit master -> axi_narrow_to_wide -> [l2c] -> axi_async_bridge ->
//   axi_ddr4_mig_bridge -> sim_mig_backend
//
// tb-l2c-wstream measures the case commit 00f079c optimised: writes that
// cover WHOLE 64 B lines.  This measures the case it does NOT catch --
// a write that touches only part of a line -- which is what a SECTORED
// dirty/valid scheme (four 128-bit sectors per line, one per 68040 L1D
// line) would change.
//
// WHAT IS REPORTED, and why it is the right discriminator
// -------------------------------------------------------
// Cycles per useful 32-bit word is the headline, but the number that
// actually decides the design is DDR TRAFFIC, so this harness counts
// handshakes on l2c's OWN 128-bit master port (tb_l2c_chain.v's
// dbg_l2m_*_fire taps):
//
//   AR / R beats  -- the read-allocate fill.  One 64 B line fill is
//                    1 AR + 4 R beats today, whatever fraction of the
//                    line the write actually covered.
//   AW / W beats  -- the dirty writeback.  One eviction is 1 AW + 4 W
//                    beats today, because `rdirty` is ONE bit per 64 B
//                    line: a single byte written anywhere marks the whole
//                    line dirty and l2c_victim.v pushes all 512 bits.
//
// The tap is deliberately on l2c's master port and not on the MIG port:
// mig_* is 256 bits wide, so a 16 B and a 32 B writeback are literally
// indistinguishable there, and 16 B is exactly the granularity in
// question.  With l2c removed from the chain the same taps mirror the
// slave port, which is the correct "no cache at all" reference.
//
// THE TRAFFIC SHAPE
// -----------------
// Each data point writes BYTES_PER_LINE bytes at the start of every 64 B
// line, over a working set built to force conflict evictions with a small
// number of lines: address = base + tag * 256 KiB + set * 64, walked
// tag-major.  256 KiB is 4096 sets x 64 B, i.e. exactly the set-index
// stride, so all 16 tags of a given `set` land in the same 8-way set.
// Passes 0..7 fill the eight ways (misses, no evictions); passes 8..15
// are steady state (every line both misses AND evicts a dirty victim).
// Counters are snapshotted at the pass-8 boundary so the reported
// per-line figures are steady-state, not diluted by the cold half.
//
// BYTES_PER_LINE = 64 reproduces tb-l2c-wstream's case and must stay at
// 0 fill beats -- it is the no-regression guard for commit 00f079c.
//
// This is a MEASUREMENT harness.  Cache correctness belongs to tb-l2c /
// tb-l2c-chain; what is checked here is that every burst got an OKAY B,
// that a sample of the written words reads back byte-exact through the
// DUT, and that nothing wedged.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>

#include <verilated.h>
#include "Vtb_l2c_wstream.h"

using DutT = Vtb_l2c_wstream;

static DutT* dut = nullptr;
static uint64_t sim_time = 0;
static int n_fail = 0;

static constexpr double CLK_MHZ = 100.0;

// Set-index stride: SETS(4096) * LINE(64).  Two addresses this far apart
// share a set and differ only in tag.
static constexpr uint64_t SET_STRIDE = 4096ULL * 64ULL;
static constexpr int      WAYS       = 8;
static constexpr int      TAGS       = 2 * WAYS;   // 8 cold + 8 evicting passes

// DDR-traffic counters, sampled on l2c's own master port.
struct Ddr { uint64_t ar = 0, r = 0, aw = 0, w = 0; };
static Ddr g_ddr;

// Eviction-pressure counters (tb_l2c_chain.v's dbg_vb_* taps).  This chain
// is the FAITHFUL one -- writes are posted by sim_mig_backend exactly as a
// real MIG posts them, so the writeback round trip measured here is the one
// the hardware actually sees.  occ[] is a histogram over victim-buffer
// occupancy (the top bucket, occ[SLOTS], is "buffer full" -- the only one
// that can block a miss); stall is "a miss was ready and blocked ONLY by
// that"; drain is "at least one writeback is in flight or pending"; out is
// the per-cycle count of writebacks with an AW accepted and no B yet, i.e.
// how much of the pipelining (2026-08-20) is actually being used.
//
// What that pipelining bought HERE, which is the control that matters
// because tb_l2c.cpp's model delays B by the full read latency where a
// real MIG posts: the 16 B/line steady state -- the shape a 68040
// copyback L1D push presents -- went 3.745 -> 2.000 cycles/word, with
// VB-full 86.6% -> 0.0% and miss-blocked 46.6% -> 0.0%.  The 4 B/line row
// is fill-bound and the 64 B/line row is W-beat-bound, so neither moves.
// docs/l2c_perf.md S13.
static constexpr int EV_MAX_SLOTS = 31;
struct Ev {
    uint64_t cycles = 0, occ[EV_MAX_SLOTS + 1] = {0}, stall = 0, drain = 0;
    uint64_t seq = 0, out_sum = 0, out_max = 0;
};
static Ev g_ev;

static void tick() {
    // clk low: everything combinational has settled to the value the
    // coming rising edge will sample, so this is where handshakes count.
    dut->core_clk = 0; dut->mig_clk = 0; dut->eval();
    if (dut->dbg_l2m_ar_fire) g_ddr.ar++;
    if (dut->dbg_l2m_r_fire)  g_ddr.r++;
    if (dut->dbg_l2m_aw_fire) g_ddr.aw++;
    if (dut->dbg_l2m_w_fire)  g_ddr.w++;
    g_ev.cycles++;
    g_ev.occ[dut->dbg_vb_occ & 0x1F]++;
    g_ev.out_sum += (dut->dbg_vb_out & 0x1F);
    if ((dut->dbg_vb_out & 0x1F) > g_ev.out_max) g_ev.out_max = dut->dbg_vb_out & 0x1F;
    if (dut->dbg_evict_stall)   g_ev.stall++;
    if (dut->dbg_vb_drain_busy) g_ev.drain++;
    if (dut->dbg_vb_seq_busy)   g_ev.seq++;
    dut->core_clk = 1; dut->mig_clk = 1; dut->eval();
    sim_time++;
}

static void reset_dut() {
    dut->core_rst = 1; dut->mig_rst = 1;
    dut->n_awvalid = 0; dut->n_wvalid = 0; dut->n_bready = 1;
    dut->n_arvalid = 0; dut->n_rready = 1;
    dut->n_awaddr = 0; dut->n_awlen = 0;
    dut->n_wdata = 0; dut->n_wstrb = 0xF; dut->n_wlast = 0;
    dut->n_araddr = 0; dut->n_arlen = 0; dut->n_arsize = 2;
    for (int i = 0; i < 32; i++) tick();
    dut->mig_rst = 0;
    for (int i = 0; i < 32; i++) tick();
    dut->core_rst = 0;
    int guard = 0;
    while (!dut->cal_done && guard++ < 100000) tick();
    // l2c's tag-clear walk is ~4096 cycles and holds the front door shut.
    for (int i = 0; i < 6000; i++) tick();
}

// One narrow INCR write burst of `words` 32-bit words at `addr`.
static bool narrow_write(uint64_t addr, const uint32_t* data, int words) {
    bool aw_done = false, b_done = false;
    int beat = 0, guard = 0;
    while (!b_done) {
        dut->n_awvalid = aw_done ? 0 : 1;
        dut->n_awaddr  = static_cast<uint32_t>(addr);
        dut->n_awlen   = static_cast<uint8_t>(words - 1);
        dut->n_wvalid  = (beat < words) ? 1 : 0;
        dut->n_wdata   = (beat < words) ? data[beat] : 0;
        dut->n_wstrb   = 0xF;
        dut->n_wlast   = (beat + 1 == words) ? 1 : 0;
        dut->n_bready  = 1;
        dut->eval();
        const bool aw_hs = !aw_done && dut->n_awready;
        const bool w_hs  = (beat < words) && dut->n_wready;
        const bool b_hs  = dut->n_bvalid != 0;
        const uint8_t resp = dut->n_bresp;
        tick();
        if (aw_hs) aw_done = true;
        if (w_hs)  beat++;
        if (b_hs) {
            if (resp != 0) {
                printf("  BRESP=%u (not OKAY) for write @0x%llx\n", resp,
                       (unsigned long long)addr);
                n_fail++;
            }
            b_done = true;
        }
        if (++guard > 2000000) {
            printf("  HANG in narrow_write @0x%llx (aw_done=%d beat=%d)\n",
                   (unsigned long long)addr, (int)aw_done, beat);
            n_fail++;
            dut->n_awvalid = 0; dut->n_wvalid = 0;
            return false;
        }
    }
    dut->n_awvalid = 0; dut->n_wvalid = 0;
    return true;
}

// One narrow INCR read burst of `words` 32-bit words; results into out[].
static bool narrow_read(uint64_t addr, uint32_t* out, int words) {
    bool ar_done = false;
    int beat = 0, guard = 0;
    while (beat < words) {
        dut->n_arvalid = ar_done ? 0 : 1;
        dut->n_araddr  = static_cast<uint32_t>(addr);
        dut->n_arlen   = static_cast<uint8_t>(words - 1);
        dut->n_arsize  = 2;
        dut->n_rready  = 1;
        dut->eval();
        const bool ar_hs = !ar_done && dut->n_arready;
        const bool r_hs  = dut->n_rvalid != 0;
        const uint32_t rd = dut->n_rdata;
        const uint8_t resp = dut->n_rresp;
        tick();
        if (ar_hs) ar_done = true;
        if (r_hs) {
            if (resp != 0) {
                printf("  RRESP=%u (not OKAY) for read @0x%llx\n", resp,
                       (unsigned long long)addr);
                n_fail++;
            }
            out[beat++] = rd;
        }
        if (++guard > 2000000) {
            printf("  HANG in narrow_read @0x%llx (beat=%d)\n",
                   (unsigned long long)addr, beat);
            n_fail++;
            dut->n_arvalid = 0;
            return false;
        }
    }
    dut->n_arvalid = 0;
    return true;
}

struct Point {
    double   cyc_per_word = 0.0;
    uint64_t lines = 0;
    Ddr      ddr;      // steady-state only (passes 8..15)
    Ev       ev;       // steady-state only, see the Ev comment above
};

// Write `bytes_per_line` bytes at the head of every 64 B line of a
// conflict-shaped working set.  `n_sets` distinct sets x TAGS tags.
static Point run_point(uint64_t base, int bytes_per_line, int n_sets,
                       std::map<uint64_t, uint32_t>* golden) {
    Point p;
    const int words = bytes_per_line / 4;
    uint32_t buf[16];
    uint64_t cycles_steady = 0, t_steady = 0;
    Ddr ddr0;
    Ev  ev0;

    for (int tag = 0; tag < TAGS; tag++) {
        if (tag == WAYS) { ddr0 = g_ddr; ev0 = g_ev; t_steady = sim_time; }
        for (int set = 0; set < n_sets; set++) {
            const uint64_t addr = base + static_cast<uint64_t>(tag) * SET_STRIDE
                                       + static_cast<uint64_t>(set) * 64;
            for (int i = 0; i < words; i++) {
                buf[i] = static_cast<uint32_t>(0xA5000000u
                          + (static_cast<uint32_t>(tag) << 20)
                          + (static_cast<uint32_t>(set) << 4) + i);
                if (golden) (*golden)[addr + 4u * i] = buf[i];
            }
            if (!narrow_write(addr, buf, words)) return p;
            if (tag >= WAYS) p.lines++;
        }
    }
    cycles_steady = sim_time - t_steady;
    p.ddr.ar = g_ddr.ar - ddr0.ar;
    p.ddr.r  = g_ddr.r  - ddr0.r;
    p.ddr.aw = g_ddr.aw - ddr0.aw;
    p.ddr.w  = g_ddr.w  - ddr0.w;
    p.ev.cycles = g_ev.cycles - ev0.cycles;
    p.ev.stall  = g_ev.stall  - ev0.stall;
    p.ev.drain  = g_ev.drain  - ev0.drain;
    p.ev.seq    = g_ev.seq    - ev0.seq;
    p.ev.out_sum = g_ev.out_sum - ev0.out_sum;
    p.ev.out_max = g_ev.out_max;
    for (int q = 0; q <= EV_MAX_SLOTS; q++) p.ev.occ[q] = g_ev.occ[q] - ev0.occ[q];
    const uint64_t steady_words = p.lines * static_cast<uint64_t>(words);
    p.cyc_per_word = steady_words
                   ? static_cast<double>(cycles_steady) / static_cast<double>(steady_words)
                   : 0.0;
    return p;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new DutT;

#ifdef SCTR_L2C_OFF
    const char* label = "l2c REMOVED from the chain";
#else
    const char* label = "l2c IN the chain";
#endif

    // 128 sets x 16 tags = 2048 lines per data point; the last 1024 are
    // steady state.  Small enough that the sweep is seconds of wall clock,
    // big enough that the per-line figures are stable.
    int n_sets = 128;
    if (const char* e = std::getenv("SCTR_SETS")) n_sets = std::atoi(e);

    printf("=== l2c SCATTERED / PARTIAL-LINE write measurement (%s) ===\n", label);
    printf("  chain: 32-bit master -> axi_narrow_to_wide -> [l2c] -> axi_async_bridge\n");
    printf("         -> axi_ddr4_mig_bridge -> sim_mig_backend\n");
    printf("  %d sets x %d tags; passes 0-%d fill the ways, passes %d-%d are steady\n",
           n_sets, TAGS, WAYS - 1, WAYS, TAGS - 1);
    printf("  state (every line both misses AND evicts a dirty victim).\n");
    printf("  DDR counts are handshakes on l2c's own 128-bit master port:\n");
    printf("  one R or W beat is one 16 B quadrant = one 68040 L1D line.\n\n");
    printf("  %-9s %12s %8s %8s %8s %8s %10s\n",
           "B/line", "cycles/word", "AR", "Rbeats", "AW", "Wbeats", "beats/line");

    const int bpl[] = {4, 16, 32, 64};
    const uint64_t base = 0x0010'0000;
    Point pts[4];
    std::map<uint64_t, uint32_t> golden;

    for (int i = 0; i < 4; i++) {
        reset_dut();
        golden.clear();
        g_ddr = Ddr{};
        // Each data point gets its own base so the previous point's dirty
        // lines cannot be counted as this one's writebacks.
        const uint64_t b = base + static_cast<uint64_t>(i) * 0x0400'0000ULL;
        pts[i] = run_point(b, bpl[i], n_sets, &golden);
        const double bpline = pts[i].lines
            ? static_cast<double>(pts[i].ddr.r + pts[i].ddr.w) / static_cast<double>(pts[i].lines)
            : 0.0;
        printf("  %-9d %12.3f %8llu %8llu %8llu %8llu %10.3f\n",
               bpl[i], pts[i].cyc_per_word,
               (unsigned long long)pts[i].ddr.ar, (unsigned long long)pts[i].ddr.r,
               (unsigned long long)pts[i].ddr.aw, (unsigned long long)pts[i].ddr.w,
               bpline);

        // Correctness sample: a scattered write is exactly where a
        // sectored allocate could silently drop the bytes it did not
        // fetch, so read a slice of what was written back through the DUT.
        int checked = 0;
        for (const auto& kv : golden) {
            if ((checked % 37) == 0) {
                uint32_t got = 0;
                if (!narrow_read(kv.first, &got, 1)) break;
                if (got != kv.second) {
                    printf("  DATA MISMATCH @0x%llx: got 0x%08x expected 0x%08x\n",
                           (unsigned long long)kv.first, got, kv.second);
                    n_fail++;
                    break;
                }
            }
            checked++;
        }
        if (pts[i].lines == 0) n_fail++;
    }

    printf("\n  Steady-state per touched line (%llu lines):\n",
           (unsigned long long)pts[0].lines);
    for (int i = 0; i < 4; i++) {
        if (!pts[i].lines) continue;
        printf("    %2d B/line: %.3f fill-R beats, %.3f writeback-W beats\n",
               bpl[i],
               static_cast<double>(pts[i].ddr.r) / static_cast<double>(pts[i].lines),
               static_cast<double>(pts[i].ddr.w) / static_cast<double>(pts[i].lines));
    }
    printf("\n  (64 B/line is commit 00f079c's case and must stay at 0 fill beats.)\n");

    // Eviction pressure.  Every steady-state line here BOTH misses and
    // evicts a dirty victim, so if the 2-entry victim buffer were ever
    // going to be the limiter it would be here.  `blocked` counts cycles
    // in which a miss was resolved and ready to dispatch and the ONLY
    // thing stopping it was `victim_push_ready`; `wb busy` is the fraction
    // of cycles the writeback FSM was draining.  Both high means the drain
    // engine is the limiter; blocked high with wb busy low would mean
    // lines are being cleaned too late.
    const unsigned vb_slots = dut->dbg_vb_slots & 0x1F;
    printf("\n  Steady-state eviction pressure (victim buffer = %u slots):\n", vb_slots);
    for (int i = 0; i < 4; i++) {
        if (!pts[i].ev.cycles) continue;
        const Ev& e = pts[i].ev;
        uint64_t part = 0;
        for (unsigned q = 1; q < vb_slots && q <= EV_MAX_SLOTS; q++) part += e.occ[q];
        const uint64_t full = (vb_slots <= EV_MAX_SLOTS) ? e.occ[vb_slots] : 0;
        printf("    %2d B/line: VB empty/partial/FULL = %5.1f%% /%5.1f%% /%5.1f%%   "
               "miss blocked on full VB %5.1f%%   wb in flight %5.1f%%   "
               "AW/W seq busy %5.1f%%   mean outstanding %.2f (peak %llu)\n",
               bpl[i],
               100.0 * e.occ[0] / e.cycles, 100.0 * part / e.cycles,
               100.0 * full / e.cycles,
               100.0 * e.stall / e.cycles, 100.0 * e.drain / e.cycles,
               100.0 * e.seq / e.cycles,
               static_cast<double>(e.out_sum) / e.cycles,
               (unsigned long long)e.out_max);
    }

    printf("\n%s: %s\n", label, n_fail ? "FAIL" : "OK");
    delete dut;
    return n_fail ? 1 : 0;
}
