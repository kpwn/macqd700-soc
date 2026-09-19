// tb_reset_debounce.cpp — Verilator harness for rtl/board/reset_debounce.v.
//
// Validates the synchronise + debounce filter that sits at the cpu_resetn
// and btn[3] board pads (default IDLE_OUT_N=0 build), and — when built
// with -GIDLE_OUT_N=1 / -DTB_IDLE_OUT_N=1 — the momentary-button polarity
// (NMI btn[1] / debug-full-reset btn[2]) used in fpga_top_clocks.vh.
//
// Scenarios (default, IDLE_OUT_N=0 build):
//   1. Cold reset: out_n starts low, follows raw_in_n into a stable high
//      state after DEBOUNCE_CYCLES of consecutive high samples.
//   2. Bouncy press: a glitchy raw_in_n (toggling within the debounce
//      window) produces exactly ONE clean low transition, not multiple.
//   3. Glitch rejection: a brief low-going noise pulse on a previously-
//      stable raw_in_n=1 input does NOT propagate to out_n (the counter
//      resets when the input goes back high before reaching threshold).
//
// Scenario 4 (IDLE_OUT_N=1 build only) — real-HW boot-NMI bug regression,
// 2026-07-17: with a momentary button whose true idle level is
// "not pressed" (raw_in_n=1 from power-up, matching a pulled-up,
// untouched board button), out_n must read 1 ("not pressed") from the
// very first cycle and must NEVER show a spurious falling edge — the
// pre-fix module hardwired out_n's power-up value to 0 regardless of
// the caller's polarity, so for the NMI (btn[1]) instance out_n read
// "pressed" for the first ~DEBOUNCE_CYCLES after every FPGA
// configuration even though the physical button was never touched.
// That produced a genuine rising edge into irq_agg.v's NMI edge
// detector on every single power-up, taking a spurious vec-31 (NMI)
// exception mid-boot on 100% of boots (observed on release/no-vipt,
// build_id 0x8b18323b: CPU landed at PC=0x40847abe inside a ROM delay
// loop with SR interrupt mask=3, drifted into a dead ROM SCC-poll
// loop it never returns from).

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include "Vreset_debounce.h"
#include "verilated.h"

#ifndef DEBOUNCE_CYCLES
#define DEBOUNCE_CYCLES 8
#endif

static Vreset_debounce* g_dut = nullptr;
static uint64_t         g_cycles = 0;

static void tick(int n = 1) {
    for (int i = 0; i < n; ++i) {
        g_dut->clk = 0; g_dut->eval();
        g_dut->clk = 1; g_dut->eval();
        ++g_cycles;
        if (g_cycles > 1000000) {
            std::fprintf(stderr, "tb_reset_debounce: runaway sim\n");
            std::exit(2);
        }
    }
}

#define EXPECT(cond, msg) do { \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL[cycle=%llu]: %s (out_n=%d raw=%d)\n", \
                     (unsigned long long)g_cycles, msg, \
                     (int)g_dut->out_n, (int)g_dut->raw_in_n); \
        std::exit(1); \
    } \
} while (0)

#if defined(TB_IDLE_OUT_N) && TB_IDLE_OUT_N
// Momentary-button build (NMI btn[1] / debug-full-reset btn[2] polarity):
// the true idle level is "not pressed", so raw_in_n idles HIGH from
// power-up (matching an untouched, pulled-up board button) and out_n
// must read 1 immediately with no spurious transition ever observed.
static int run_idle_high_scenario() {
    Vreset_debounce dut;
    g_dut = &dut;

    // Power-up idle: button never touched, raw_in_n is high from t=0 —
    // this is exactly what an untouched btn[1]/btn[2] looks like the
    // instant the FPGA finishes configuration.
    dut.clk = 0;
    dut.raw_in_n = 1;
    dut.eval();   // settle the `initial`-block values without any clk edge

    // Sample out_n on every single cycle from the very first tick;
    // any 1->0 transition here is the bug (a false "pressed" edge
    // with the physical button never touched).
    int spurious_falling_edges = 0;
    int prev = -1;
    // Long enough to clear the 3-FF sync chain + a full debounce
    // window with margin, so we'd catch a spurious edge anywhere in
    // that startup transient (the pre-fix bug manifested inside the
    // first few cycles, but scan generously).
    const int OBSERVE_CYCLES = DEBOUNCE_CYCLES * 3 + 32;
    for (int i = 0; i < OBSERVE_CYCLES; ++i) {
        // out_n is a `reg` with an `initial` value — sample it BEFORE
        // the first clock edge too, since that is the true
        // configuration-time value a downstream consumer would see.
        if (i == 0) {
            EXPECT(dut.out_n == 1,
                   "power-up: out_n reads NOT-PRESSED before any clock edge");
            prev = dut.out_n;
        }
        tick(1);
        if (prev == 1 && dut.out_n == 0) spurious_falling_edges++;
        prev = dut.out_n;
    }
    EXPECT(spurious_falling_edges == 0,
           "no spurious pressed-edge while button is untouched (idle-high)");
    EXPECT(dut.out_n == 1, "stays NOT-PRESSED with no press applied");
    std::printf("PASS: IDLE_OUT_N=1 — no spurious edge on untouched button "
                "(%d cycles observed)\n", OBSERVE_CYCLES);

    // Sanity: a REAL press (raw_in_n->0, held stable) still works
    // correctly under this polarity, i.e. the fix didn't just make
    // out_n permanently stuck at 1.
    dut.raw_in_n = 0;
    int cycles_to_press = 0;
    while (dut.out_n == 1 && cycles_to_press < DEBOUNCE_CYCLES + 16) {
        tick(1);
        cycles_to_press++;
    }
    EXPECT(dut.out_n == 0, "genuine press still propagates under IDLE_OUT_N=1");
    EXPECT(cycles_to_press >= DEBOUNCE_CYCLES,
           "genuine press still waits for full DEBOUNCE_CYCLES");
    std::printf("PASS: genuine press still detected after %d cycles\n",
                cycles_to_press);

    std::printf("\ntb_reset_debounce (IDLE_OUT_N=1): ALL PASS (%llu cycles)\n",
                (unsigned long long)g_cycles);
    return 0;
}
#endif

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

#if defined(TB_IDLE_OUT_N) && TB_IDLE_OUT_N
    return run_idle_high_scenario();
#else
    Vreset_debounce dut;
    g_dut = &dut;

    dut.clk = 0;
    dut.raw_in_n = 0;
    tick(2);

    // ---------- Scenario 1: cold release ----------
    EXPECT(dut.out_n == 0, "post-init: out_n is low (asserted)");
    dut.raw_in_n = 1;   // release
    // Need 3 sync FFs + DEBOUNCE_CYCLES of stable-high samples.
    int cycles_to_release = 0;
    while (dut.out_n == 0 && cycles_to_release < DEBOUNCE_CYCLES + 16) {
        tick(1);
        cycles_to_release++;
    }
    EXPECT(dut.out_n == 1, "release propagates after stable-high window");
    EXPECT(cycles_to_release >= DEBOUNCE_CYCLES,
           "release waits for full DEBOUNCE_CYCLES (no early release)");
    std::printf("PASS: cold release after %d cycles (DEBOUNCE_CYCLES=%d)\n",
                cycles_to_release, DEBOUNCE_CYCLES);

    // ---------- Scenario 2: bouncy press ----------
    // Toggle raw_in_n within the debounce window and verify out_n
    // transitions to 0 ONCE only — i.e. count rising/falling edges
    // of out_n during the bouncy phase.
    int out_falling_edges = 0;
    int prev = dut.out_n;
    for (int i = 0; i < DEBOUNCE_CYCLES * 4; ++i) {
        // Toggle every 1-2 cycles (bouncy).
        dut.raw_in_n = (i & 1) ? 0 : 1;
        tick(1);
        if (prev == 1 && dut.out_n == 0) out_falling_edges++;
        prev = dut.out_n;
    }
    // Now hold low long enough to commit the press.
    dut.raw_in_n = 0;
    for (int i = 0; i < DEBOUNCE_CYCLES + 8; ++i) {
        tick(1);
        if (prev == 1 && dut.out_n == 0) out_falling_edges++;
        prev = dut.out_n;
    }
    EXPECT(dut.out_n == 0, "after stable-low: out_n=0 (press took effect)");
    EXPECT(out_falling_edges == 1,
           "bouncy press produces exactly one clean falling edge on out_n");
    std::printf("PASS: bouncy press → one clean falling edge\n");

    // ---------- Scenario 3: glitch rejection on a stable high ----------
    // First go back to a clean stable-high.
    dut.raw_in_n = 1;
    for (int i = 0; i < DEBOUNCE_CYCLES * 2 + 8; ++i) tick(1);
    EXPECT(dut.out_n == 1, "stable-high reached again before glitch test");

    // Inject a short low pulse — shorter than DEBOUNCE_CYCLES — and
    // verify out_n stays high.
    int glitch_len = (DEBOUNCE_CYCLES > 1) ? (DEBOUNCE_CYCLES - 1) : 1;
    dut.raw_in_n = 0;
    for (int i = 0; i < glitch_len; ++i) tick(1);
    dut.raw_in_n = 1;
    for (int i = 0; i < DEBOUNCE_CYCLES * 2 + 8; ++i) tick(1);
    EXPECT(dut.out_n == 1, "short glitch < DEBOUNCE_CYCLES rejected");
    std::printf("PASS: short glitch rejected\n");

    std::printf("\ntb_reset_debounce: ALL PASS (%llu cycles)\n",
                (unsigned long long)g_cycles);
    return 0;
#endif
}
