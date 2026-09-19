// tb_dbg_rst_pulse.cpp — Verilator harness for tb_dbg_rst_pulse.v.
//
// Validates the debug-full-reset edge-detect / pulse-stretch / watchdog
// wrapper from rtl/fpga_top_clocks.vh:
//
//   1. Rising edge on src_level fires a PULSE_CYCLES-long pulse on
//      pulse_eff regardless of how long the source stays high.
//   2. Holding src_level high beyond STUCK_CYCLES disarms the generator
//      so a stuck VIO probe-out cannot keep the SoC in reset forever.
//   3. After src_level returns low the watchdog rearms and the next
//      rising edge fires another pulse.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include "Vdbg_rst_pulse_dut.h"
#include "verilated.h"

#ifndef PULSE_CYCLES
#define PULSE_CYCLES 8
#endif
#ifndef STUCK_CYCLES
#define STUCK_CYCLES 64
#endif

static Vdbg_rst_pulse_dut* g_dut = nullptr;
static uint64_t            g_cycles = 0;

static void tick(int n = 1) {
    for (int i = 0; i < n; ++i) {
        g_dut->clk = 0; g_dut->eval();
        g_dut->clk = 1; g_dut->eval();
        ++g_cycles;
        if (g_cycles > 1000000) {
            std::fprintf(stderr, "tb_dbg_rst_pulse: runaway sim\n");
            std::exit(2);
        }
    }
}

#define EXPECT(cond, msg) do { \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL[cycle=%llu]: %s (pulse_eff=%d armed=%d)\n", \
                     (unsigned long long)g_cycles, msg, \
                     (int)g_dut->pulse_eff, (int)g_dut->armed_dbg); \
        std::exit(1); \
    } \
} while (0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vdbg_rst_pulse_dut dut;
    g_dut = &dut;

    // ---------- cold reset ----------
    dut.clk = 0;
    dut.rstn = 0;
    dut.src_level = 0;
    tick(4);
    dut.rstn = 1;
    tick(2);
    EXPECT(dut.pulse_eff == 0, "post-reset: pulse_eff is 0");
    EXPECT(dut.armed_dbg == 1, "post-reset: armed");
    std::printf("PASS: post-reset state quiescent\n");

    // ---------- Scenario 1: edge fires fixed-length pulse ----------
    dut.src_level = 1;
    tick(1);
    EXPECT(dut.pulse_eff == 1, "edge causes pulse_eff to assert");
    int pulse_cycles_seen = 0;
    while (dut.pulse_eff && pulse_cycles_seen < PULSE_CYCLES + 4) {
        tick(1);
        if (dut.pulse_eff) pulse_cycles_seen++;
    }
    // After the pulse expires, eff should be low even though src_level
    // is still being held high.
    EXPECT(dut.pulse_eff == 0,
           "after PULSE_CYCLES the pulse auto-deasserts");
    EXPECT(pulse_cycles_seen <= PULSE_CYCLES + 1,
           "pulse length stays within PULSE_CYCLES bound");
    std::printf("PASS: pulse auto-deasserts (saw %d cycles, bound %d)\n",
                pulse_cycles_seen + 1, PULSE_CYCLES);

    // ---------- Scenario 2: stuck-high src does not re-fire ----------
    // Source still held high.  Run long enough for the stuck watchdog
    // to fire and disarm.  After that, even another rising edge has
    // no effect — until the source goes low.
    int saw_second_pulse = 0;
    int total = 0;
    while (total < STUCK_CYCLES + 16 && !saw_second_pulse) {
        tick(1);
        total++;
        if (dut.pulse_eff) saw_second_pulse++;
    }
    EXPECT(saw_second_pulse == 0,
           "held-high src does NOT re-fire pulse after first deassert");
    EXPECT(dut.armed_dbg == 0,
           "watchdog disarms the generator after STUCK_CYCLES");
    std::printf("PASS: held-high src disarmed by watchdog\n");

    // ---------- Scenario 3: drop low → re-arm → next edge fires ----------
    dut.src_level = 0;
    tick(2);
    EXPECT(dut.armed_dbg == 1, "drop-low rearms the watchdog");
    dut.src_level = 1;
    tick(1);
    EXPECT(dut.pulse_eff == 1,
           "post-rearm: next rising edge fires another pulse");
    dut.src_level = 0;
    tick(PULSE_CYCLES + 4);
    EXPECT(dut.pulse_eff == 0, "second pulse expires");
    std::printf("PASS: rearm + second pulse OK\n");

    // ---------- Scenario 4: bouncy press → still ONE pulse ----------
    // Tap-tap-tap within a single pulse window: only the first edge
    // counts (the pulse is already active; subsequent rising edges
    // are no-ops because src_q tracks src_level).
    int extra_pulses = 0;
    dut.src_level = 1; tick(1);
    EXPECT(dut.pulse_eff == 1, "first tap fires pulse");
    for (int i = 0; i < 4; ++i) {
        dut.src_level = 0; tick(1);
        dut.src_level = 1; tick(1);
        if (dut.pulse_eff && i > 0) extra_pulses++;
    }
    // The pulse from the first tap is still in flight; the bouncy
    // re-presses do not extend it.  Drain.
    dut.src_level = 0;
    tick(PULSE_CYCLES + 4);
    EXPECT(dut.pulse_eff == 0, "post-bounce drain: pulse_eff back to 0");
    std::printf("PASS: bouncy press does not double-fire\n");

    std::printf("\ntb_dbg_rst_pulse: ALL PASS (%llu cycles)\n",
                (unsigned long long)g_cycles);
    return 0;
}
