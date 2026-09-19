// tb_q700_toggle_rx.cpp — unit tb for the toggle-per-event CDC receiver.
//
// WHAT THIS EXISTS TO PIN (race audit, 2026-09-18)
//
// `q700_eth_link`'s transmit-completion seam crosses 125 MHz -> core_clk as
// a toggle.  The two sides' resets are NOT the same event and cannot be
// made so: the source's only reset is `rst_125mhz` = taxi_sync_reset of
// `~mmcm_locked`, on an MMCM whose `.RST` is hardwired to 1'b0, so no
// platform/cold/debug/68040 reset ever clears it; the destination chain
// resets on `core_rst`.
//
// Consequence, before the fix: every core reset taken after any TX traffic
// refills the chain from 0 towards the source's surviving parity, and when
// that parity is 1 the edge detector manufactures a completion nobody
// generated -- permanently, because a toggle carries no absolute value.
//
// The fix suppresses edge detection for SYNC_FF destination clocks after
// reset release.  `tb-q700-toggle-rx-mut` rebuilds the RTL with the priming
// deleted and REQUIRES this test to reject it; a test that survives its own
// mutant proves nothing.
#include "Vq700_toggle_rx.h"
#include "verilated.h"
#include <cstdio>

static Vq700_toggle_rx* dut;
static int n_pass = 0, n_fail = 0;

static void check(const char* what, bool ok) {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (ok) n_pass++; else n_fail++;
}

// One clock: sample the combinational output BEFORE the edge, then tick.
static int pulses_seen = 0;
static void cycle() {
    dut->eval();
    if (dut->pulse) pulses_seen++;
    dut->clk = 1; dut->eval();
    dut->clk = 0; dut->eval();
}

static void hold_reset(int cycles) {
    dut->rst = 1;
    for (int i = 0; i < cycles; i++) cycle();
    dut->rst = 0;
    dut->eval();
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vq700_toggle_rx;
    dut->clk = 0; dut->rst = 1; dut->src_toggle = 0;
    dut->eval();

    // ── 1. THE DEFECT ───────────────────────────────────────────────────
    // The source has toggled to 1 and its reset never fires.  A destination
    // reset must NOT manufacture a pulse out of the refill.
    dut->src_toggle = 1;
    hold_reset(8);
    pulses_seen = 0;
    for (int i = 0; i < 64; i++) cycle();
    char nm[160];
    std::snprintf(nm, sizeof(nm),
                  "1: a destination reset taken with the source toggle HIGH "
                  "manufactures no completion (%d seen)", pulses_seen);
    check(nm, pulses_seen == 0);

    // ── 2. POSITIVE CONTROL ─────────────────────────────────────────────
    // Once primed, a real flip must still produce exactly one pulse -- if
    // this ever fails the priming has eaten real events.
    pulses_seen = 0;
    dut->src_toggle = 0;
    for (int i = 0; i < 32; i++) cycle();
    check("2: [positive control] one flip -> exactly one pulse", pulses_seen == 1);

    pulses_seen = 0;
    for (int f = 0; f < 5; f++) {
        dut->src_toggle = !dut->src_toggle;
        for (int i = 0; i < 16; i++) cycle();
    }
    check("2: [positive control] five flips -> exactly five pulses", pulses_seen == 5);

    // ── 3. THE SAME DEFECT, SOURCE LOW ──────────────────────────────────
    // The benign half: a reset with the source at 0 was always safe.  Kept
    // so a "fix" that simply stopped detecting edges cannot pass check 1.
    dut->src_toggle = 0;
    hold_reset(8);
    pulses_seen = 0;
    for (int i = 0; i < 64; i++) cycle();
    check("3: a destination reset with the source toggle LOW is also quiet",
          pulses_seen == 0);

    // ── 4. A FLIP DURING RESET IS NOT A COMPLETION ──────────────────────
    // The destination was in reset, so it had nothing outstanding for the
    // source to complete; the flip must be absorbed, not replayed.
    dut->rst = 1;
    for (int i = 0; i < 4; i++) cycle();
    dut->src_toggle = 1;              // the MAC transmits while we are held
    for (int i = 0; i < 4; i++) cycle();
    dut->rst = 0; dut->eval();
    pulses_seen = 0;
    for (int i = 0; i < 64; i++) cycle();
    check("4: a source flip that happened DURING reset is absorbed, not replayed",
          pulses_seen == 0);

    // ── 5. AND THE NEXT REAL EVENT STILL LANDS ──────────────────────────
    pulses_seen = 0;
    dut->src_toggle = 0;
    for (int i = 0; i < 32; i++) cycle();
    check("5: the first real event after that reset is still delivered",
          pulses_seen == 1);

    // ── 6. RESET RE-ENTRY MID-STREAM ────────────────────────────────────
    // Reset asserted one cycle after a flip: the in-flight edge is dropped
    // (the consumer is being reset anyway) and nothing is manufactured on
    // the way out.
    dut->src_toggle = 1;
    cycle();
    dut->rst = 1;
    for (int i = 0; i < 6; i++) cycle();
    dut->rst = 0; dut->eval();
    pulses_seen = 0;
    for (int i = 0; i < 64; i++) cycle();
    check("6: reset re-entered right after a flip leaves nothing manufactured",
          pulses_seen == 0);

    std::printf("\n%s — %d passed, %d failed\n",
                n_fail ? "Some checks FAILED" : "All checks PASSED",
                n_pass, n_fail);
    delete dut;
    return n_fail ? 1 : 0;
}
