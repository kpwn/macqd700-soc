// tb_cpu_rst_stretch.cpp — Verilator harness for tb_cpu_rst_stretch.v.
//
// Validates the cpu_rst minimum-pulse stretcher from
// rtl/fpga_top_clocks.vh.
//
// Scenarios:
//   1. Steady-state release: with all OR inputs deasserted (boot_rom_
//      ready=1, cpu_rst_settle_done=1, soc_full_rst=0, dbg_soft_rst=0)
//      and the SR drained, cpu_rst is low.
//   2. The glitch we are guarding against: a 1-cycle pulse on
//      dbg_soft_rst.  cpu_rst MUST stay asserted for at least 8 cycles
//      after the glitch ends — NOT a 1-cycle reset.
//   3. Long reset: a multi-cycle assertion still releases 8 cycles
//      after the input deasserts (no extra stretching beyond the
//      stretcher itself).
//   4. Repeated glitches inside the stretch window keep extending it
//      (any new assert reloads the SR to all-1s).

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include "Vcpu_rst_stretch_dut.h"
#include "verilated.h"

static Vcpu_rst_stretch_dut* g_dut = nullptr;
static uint64_t              g_cycles = 0;

static void tick(int n = 1) {
    for (int i = 0; i < n; ++i) {
        g_dut->clk = 0; g_dut->eval();
        g_dut->clk = 1; g_dut->eval();
        ++g_cycles;
        if (g_cycles > 10000) {
            std::fprintf(stderr, "tb_cpu_rst_stretch: runaway sim\n");
            std::exit(2);
        }
    }
}

#define EXPECT(cond, msg) do { \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL[cycle=%llu]: %s " \
                     "(cpu_rst=%d stretch=0x%02X)\n", \
                     (unsigned long long)g_cycles, msg, \
                     (int)g_dut->cpu_rst, \
                     (unsigned)g_dut->stretch_dbg); \
        std::exit(1); \
    } \
} while (0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vcpu_rst_stretch_dut dut;
    g_dut = &dut;

    // ---------- Scenario 1: steady-state release ----------
    // Drive a clean post-cold-reset state and let the SR drain.
    dut.clk = 0;
    dut.soc_full_rst = 0;
    dut.dbg_soft_rst = 0;
    dut.boot_rom_ready = 1;
    dut.cpu_rst_settle_done = 1;
    // Initial SR is all-1s; takes 8 cycles to drain.
    for (int i = 0; i < 16; ++i) tick(1);
    EXPECT(dut.cpu_rst == 0, "steady-state with all inputs OK: cpu_rst low");
    EXPECT(dut.stretch_dbg == 0, "stretch SR drained to 0");
    std::printf("PASS: steady-state release\n");

    // ---------- Scenario 2: 1-cycle dbg_soft_rst glitch ----------
    // Pulse dbg_soft_rst high for exactly one cycle.
    dut.dbg_soft_rst = 1;
    tick(1);
    dut.dbg_soft_rst = 0;
    EXPECT(dut.cpu_rst == 1, "during glitch: cpu_rst asserted");
    // After the glitch, cpu_rst MUST stay high for at least 8 cycles
    // because the SR was reloaded.  Walk forward and count.
    int cycles_high = 0;
    while (dut.cpu_rst == 1 && cycles_high < 32) {
        cycles_high++;
        tick(1);
    }
    EXPECT(cycles_high >= 8,
           "1-cycle glitch produces >=8-cycle cpu_rst (no truncation)");
    EXPECT(dut.cpu_rst == 0, "post-stretcher: cpu_rst back low");
    std::printf("PASS: 1-cycle glitch stretched to %d cycles\n", cycles_high);

    // ---------- Scenario 3: long reset, normal release ----------
    // Drive a 4-cycle reset, then deassert.  cpu_rst should stay
    // high for ~ (long pulse + 8) cycles.
    dut.soc_full_rst = 1;
    tick(4);
    dut.soc_full_rst = 0;
    EXPECT(dut.cpu_rst == 1, "during long reset: asserted");
    int extra_cycles = 0;
    while (dut.cpu_rst == 1 && extra_cycles < 32) {
        extra_cycles++;
        tick(1);
    }
    EXPECT(extra_cycles >= 8 && extra_cycles <= 9,
           "post-long-reset stretch is 8-cycle stable");
    std::printf("PASS: long-reset release (extra_cycles=%d)\n", extra_cycles);

    // ---------- Scenario 4: repeated glitches keep extending ----------
    // Tap dbg_soft_rst every 3 cycles within the stretch window;
    // verify cpu_rst stays asserted continuously.
    dut.dbg_soft_rst = 1;
    tick(1);
    dut.dbg_soft_rst = 0;
    bool dropped = false;
    for (int i = 0; i < 30; ++i) {
        tick(1);
        if (i % 3 == 0) {
            dut.dbg_soft_rst = 1;
            tick(1);
            dut.dbg_soft_rst = 0;
        }
        if (dut.cpu_rst == 0) { dropped = true; break; }
    }
    EXPECT(!dropped,
           "repeated glitches keep cpu_rst continuously asserted");
    // Drain.
    for (int i = 0; i < 16; ++i) tick(1);
    EXPECT(dut.cpu_rst == 0, "post-drain: cpu_rst back low");
    std::printf("PASS: repeated glitches keep extending\n");

    std::printf("\ntb_cpu_rst_stretch: ALL PASS (%llu cycles)\n",
                (unsigned long long)g_cycles);
    return 0;
}
