// tb_l2c_wstream.cpp -- cycles per 32-bit word for a streaming write
// through the real boot write path (see tb/tb_l2c_wstream.v).
//
// The BFM is deliberately the simplest thing that saturates the chain:
// one narrow INCR write burst at a time (axi_narrow_to_wide is
// single-outstanding, so more is not possible), AW presented immediately,
// every W beat presented back to back, BREADY always high.  That is
// exactly boot_fsm's RAM pre-zero shape after commit 2afe6d1.
//
// Reported: cycles per 32-bit word for a sweep of narrow AWLEN values,
// and the extrapolation to zeroing 256 MiB at the real 100 MHz core_clk.
// Run it twice -- `make tb-l2c-wstream` (l2c in the chain) and
// `make tb-l2c-wstream-off` (l2c not elaborated at all) -- to get the tax
// l2c charges on streaming writes.
//
// This is a MEASUREMENT harness: correctness of the cache itself belongs
// to tb-l2c / tb-l2c-chain.  What is checked here is only that every
// burst got an OKAY B and that nothing wedged.

#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include <verilated.h>
#include "Vtb_l2c_wstream.h"

using DutT = Vtb_l2c_wstream;

static DutT* dut = nullptr;
static uint64_t sim_time = 0;
static int n_fail = 0;

static constexpr double CLK_MHZ = 100.0;

// One full core_clk/mig_clk period, matched 1:1 -- the same clocking
// tb_l2c_chain.cpp uses (CDC ratio diversity is tb_axi_async_bridge's
// job, not this harness's).
static void tick() {
    dut->core_clk = 0; dut->mig_clk = 0; dut->eval();
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
    // l2c's own tag-clear walk is ~4096 cycles and holds the front door
    // shut; measuring across it would price the reset, not the traffic.
    for (int i = 0; i < 6000; i++) tick();
}

struct Result { double cyc_per_word; uint64_t cycles; uint64_t words; };

// Stream `words` 32-bit words as back-to-back bursts of (awlen+1) beats,
// timed from the first AW presentation to the last B.
static Result stream_writes(uint64_t base, int awlen, uint64_t words) {
    const uint64_t beats  = static_cast<uint64_t>(awlen) + 1;
    const uint64_t bursts = words / beats;
    uint64_t addr = base;
    uint32_t pattern = 0;
    const uint64_t t0 = sim_time;

    for (uint64_t b = 0; b < bursts; b++) {
        bool aw_done = false, b_done = false;
        uint64_t beat = 0;
        int guard = 0;
        while (!b_done) {
            dut->n_awvalid = aw_done ? 0 : 1;
            dut->n_awaddr  = static_cast<uint32_t>(addr);
            dut->n_awlen   = static_cast<uint8_t>(awlen);
            dut->n_wvalid  = (beat < beats) ? 1 : 0;
            dut->n_wdata   = pattern;
            dut->n_wstrb   = 0xF;
            dut->n_wlast   = (beat + 1 == beats) ? 1 : 0;
            dut->n_bready  = 1;
            dut->eval();

            const bool aw_hs = !aw_done   && dut->n_awready;
            const bool w_hs  = (beat < beats) && dut->n_wready;
            const bool b_hs  = dut->n_bvalid != 0;
            const uint8_t resp = dut->n_bresp;
            tick();

            if (aw_hs) aw_done = true;
            if (w_hs)  { beat++; pattern++; }
            if (b_hs) {
                if (resp != 0) {
                    printf("  BRESP=%u (not OKAY) on burst %llu\n", resp, (unsigned long long)b);
                    n_fail++;
                }
                b_done = true;
            }
            if (++guard > 4000000) {
                printf("  HANG: awlen=%d burst=%llu aw_done=%d beat=%llu\n",
                       awlen, (unsigned long long)b, (int)aw_done, (unsigned long long)beat);
                n_fail++;
                dut->n_awvalid = 0; dut->n_wvalid = 0;
                return {0, 0, 0};
            }
        }
        addr += beats * 4;
    }
    dut->n_awvalid = 0; dut->n_wvalid = 0;
    const uint64_t cycles = sim_time - t0;
    const uint64_t total_words = bursts * beats;
    return { total_words ? static_cast<double>(cycles) / total_words : 0.0, cycles, total_words };
}

// Same measurement with a PIPELINED narrow master: AW offered as soon as
// the adapter will take it, W beats streamed behind it, B never waited
// for.  stream_writes() above deliberately waits for B, which was the
// only thing a single-outstanding axi_narrow_to_wide could support and
// which therefore priced the fabric rather than the adapter.  Now that
// the adapter carries WR_OUTSTANDING credits, this is what the chain
// actually delivers to a master that can keep it fed — and it is the
// number to compare against when deciding whether a given master (today:
// boot_fsm's ST_ZERO_AW/W/B loop, which still waits for B) is worth
// pipelining.
static Result stream_writes_pipelined(uint64_t base, int awlen, uint64_t words) {
    const uint64_t beats  = static_cast<uint64_t>(awlen) + 1;
    const uint64_t bursts = words / beats;
    const uint64_t t0 = sim_time;

    uint64_t aw_issued = 0, w_burst = 0, b_seen = 0;
    uint64_t beat = 0;
    uint32_t pattern = 0;
    int guard = 0;

    while (b_seen < bursts) {
        const bool aw_off = (aw_issued < bursts);
        const bool w_off  = (w_burst  < bursts);
        dut->n_awvalid = aw_off;
        dut->n_awaddr  = static_cast<uint32_t>(base + aw_issued * beats * 4);
        dut->n_awlen   = static_cast<uint8_t>(awlen);
        dut->n_wvalid  = w_off;
        dut->n_wdata   = pattern;
        dut->n_wstrb   = 0xF;
        dut->n_wlast   = (beat + 1 == beats) ? 1 : 0;
        dut->n_bready  = 1;
        dut->eval();

        const bool aw_hs = aw_off && dut->n_awready;
        const bool w_hs  = w_off  && dut->n_wready;
        const bool b_hs  = dut->n_bvalid != 0;
        const uint8_t resp = dut->n_bresp;
        tick();

        if (aw_hs) aw_issued++;
        if (w_hs) {
            pattern++;
            if (++beat == beats) { beat = 0; w_burst++; }
        }
        if (b_hs) {
            if (resp != 0) {
                printf("  BRESP=%u (not OKAY) on pipelined burst %llu\n",
                       resp, (unsigned long long)b_seen);
                n_fail++;
            }
            b_seen++;
        }
        if (++guard > 40000000) {
            printf("  HANG: pipelined awlen=%d aw_issued=%llu b_seen=%llu\n",
                   awlen, (unsigned long long)aw_issued,
                   (unsigned long long)b_seen);
            n_fail++;
            dut->n_awvalid = 0; dut->n_wvalid = 0;
            return {0, 0, 0};
        }
    }
    dut->n_awvalid = 0; dut->n_wvalid = 0;
    const uint64_t cycles = sim_time - t0;
    const uint64_t total_words = bursts * beats;
    return { total_words ? static_cast<double>(cycles) / total_words : 0.0,
             cycles, total_words };
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new DutT;

#ifdef WSTREAM_L2C_OFF
    const char* label = "l2c REMOVED from the chain";
#else
    const char* label = "l2c IN the chain";
#endif

    // 64 KiB per data point: long enough that reset/calibration is
    // irrelevant to the steady-state rate, short enough that the whole
    // sweep is a few seconds of wall clock.
    uint64_t words = 16 * 1024;
    if (const char* e = std::getenv("WSTREAM_WORDS")) words = std::strtoull(e, nullptr, 0);

    printf("=== l2c streaming-write measurement (%s) ===\n", label);
    printf("  chain: 32-bit master -> axi_narrow_to_wide -> [l2c] -> axi_async_bridge\n");
    printf("         -> axi_ddr4_mig_bridge -> sim_mig_backend\n");
    printf("  %llu x 32-bit words per data point\n\n",
           (unsigned long long)words);

    const int awlens[] = {0, 15, 63, 255};
    const uint64_t base = 0x0010'0000;

    printf("  -- single-outstanding narrow master (waits for B per burst) --\n");
    printf("  %-12s %14s %14s %18s\n", "narrow AWLEN", "cycles/word", "cycles", "256 MiB @100MHz");
    double serial[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
        reset_dut();
        const Result r = stream_writes(base, awlens[i], words);
        const double secs = r.cyc_per_word * (256.0 * 1024 * 1024 / 4.0) / (CLK_MHZ * 1e6);
        printf("  %-12d %14.3f %14llu %15.3f s\n", awlens[i], r.cyc_per_word,
               (unsigned long long)r.cycles, secs);
        serial[i] = r.cyc_per_word;
        if (r.words == 0) n_fail++;
    }

    // axi_narrow_to_wide became multi-outstanding on 2026-08-20.  The
    // sweep above cannot see that, because its master serializes itself.
    // This one keeps the port fed so the chain's real pipelined rate is
    // on the record next to it.
    printf("\n  -- pipelined narrow master (multi-outstanding, never waits for B) --\n");
    printf("  %-12s %14s %14s %18s %10s\n", "narrow AWLEN", "cycles/word", "cycles",
           "256 MiB @100MHz", "speedup");
    for (int i = 0; i < 4; i++) {
        reset_dut();
        const Result r = stream_writes_pipelined(base, awlens[i], words);
        const double secs = r.cyc_per_word * (256.0 * 1024 * 1024 / 4.0) / (CLK_MHZ * 1e6);
        printf("  %-12d %14.3f %14llu %15.3f s %9.2fx\n", awlens[i], r.cyc_per_word,
               (unsigned long long)r.cycles, secs,
               (r.cyc_per_word > 0.0) ? serial[i] / r.cyc_per_word : 0.0);
        if (r.words == 0) n_fail++;
    }

    printf("\n%s: %s\n", label, n_fail ? "FAIL" : "OK");
    delete dut;
    return n_fail ? 1 : 0;
}
