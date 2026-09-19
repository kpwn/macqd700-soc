// tb_boot_release_gate.cpp — Verilator harness for tb_boot_release_gate.v.
//
// Validates the boot_rom_ready / cpu_rst gating from
// rtl/fpga_top_clocks.vh after the Fix-D landing.
//
// Scenarios:
//   1. Cold boot, no JTAG: cpu_rst stays asserted until boot_rom_loaded
//      rises and soc_full_rst drops.
//   2. JTAG bypass + release with ROM unloaded: cpu_rst drops only when
//      bypass=1 AND release=1, AND soc_full_rst=0.
//   3. The race that motivated the fix: with bypass+release set, fire a
//      debug_full_reset (soc_full_rst=1).  cpu_rst MUST stay asserted
//      throughout the pulse, regardless of the bypass/release bits.
//      After soc_full_rst drops, cpu_rst can release.
//   4. After soc_full_rst, boot_fsm_rst MUST also be asserted so the
//      boot FSM re-arms and clears boot_rom_loaded — so on the next
//      cold-boot path (bypass=0) the CPU still waits for the SD→DDR
//      copy to finish.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include "Vboot_release_gate_dut.h"
#include "verilated.h"

static Vboot_release_gate_dut* g_dut = nullptr;
static uint64_t                g_cycles = 0;

static void settle() {
    g_dut->clk = 0; g_dut->eval();
    g_dut->clk = 1; g_dut->eval();
    ++g_cycles;
    if (g_cycles > 10000) {
        std::fprintf(stderr, "tb_boot_release_gate: runaway sim\n");
        std::exit(2);
    }
}

#define EXPECT(cond, msg) do { \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL[cycle=%llu]: %s\n" \
                     "  cpu_rst=%d boot_rom_ready=%d boot_fsm_rst=%d\n" \
                     "  soc_full_rst=%d boot_rom_loaded=%d bypass=%d release=%d\n", \
                     (unsigned long long)g_cycles, msg, \
                     (int)g_dut->cpu_rst, (int)g_dut->boot_rom_ready, \
                     (int)g_dut->boot_fsm_rst, \
                     (int)g_dut->soc_full_rst, (int)g_dut->boot_rom_loaded, \
                     (int)g_dut->jtag_boot_bypass, (int)g_dut->jtag_boot_release); \
        std::exit(1); \
    } \
} while (0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vboot_release_gate_dut dut;
    g_dut = &dut;

    // ---------- Scenario 1: cold boot, no JTAG ----------
    dut.soc_full_rst = 1;
    dut.boot_rom_loaded = 0;
    dut.jtag_boot_bypass = 0;
    dut.jtag_boot_release = 0;
    settle();
    EXPECT(dut.cpu_rst == 1, "cold soc_full_rst: cpu_rst asserted");
    EXPECT(dut.boot_fsm_rst == 1, "cold soc_full_rst: boot_fsm in reset");

    dut.soc_full_rst = 0;   // reset releases
    settle();
    EXPECT(dut.cpu_rst == 1,
           "soc_full_rst dropped but boot_rom not loaded → CPU held");
    EXPECT(dut.boot_fsm_rst == 0,
           "soc_full_rst dropped: boot_fsm out of reset to start ROM copy");

    dut.boot_rom_loaded = 1;
    settle();
    EXPECT(dut.cpu_rst == 0, "ROM loaded: CPU released");
    EXPECT(dut.boot_rom_ready == 1, "boot_rom_ready high");
    std::printf("PASS: cold boot path\n");

    // ---------- Scenario 2: JTAG bypass + release ----------
    // Reset state for clean slate.
    dut.soc_full_rst = 1; dut.boot_rom_loaded = 0;
    dut.jtag_boot_bypass = 0; dut.jtag_boot_release = 0;
    settle();
    dut.soc_full_rst = 0;
    settle();
    EXPECT(dut.cpu_rst == 1, "no JTAG, no ROM: CPU held");

    dut.jtag_boot_bypass = 1;
    settle();
    EXPECT(dut.cpu_rst == 1,
           "bypass=1 alone (release=0) keeps CPU held");
    EXPECT(dut.boot_fsm_rst == 1,
           "bypass=1: boot_fsm held in reset (host owns ROM load)");

    dut.jtag_boot_release = 1;
    settle();
    EXPECT(dut.cpu_rst == 0,
           "bypass+release: CPU released even though boot_rom_loaded=0");
    std::printf("PASS: JTAG bypass+release path\n");

    // ---------- Scenario 3: the race ----------
    // Now fire a debug_full_reset pulse with bypass+release still set.
    // The CPU must stay held until soc_full_rst drops.
    dut.soc_full_rst = 1;
    settle();
    EXPECT(dut.cpu_rst == 1,
           "debug_full_reset pulse: CPU held DESPITE bypass+release set");
    EXPECT(dut.boot_rom_ready == 0,
           "during soc_full_rst: boot_rom_ready is low (not stuck high)");
    EXPECT(dut.boot_fsm_rst == 1,
           "during soc_full_rst: boot_fsm in reset too");

    // pulse continues for a few cycles
    for (int i = 0; i < 4; ++i) {
        settle();
        EXPECT(dut.cpu_rst == 1,
               "pulse[i]: CPU stays held while soc_full_rst is asserted");
    }

    // pulse drops; bypass+release still set (host hasn't cleared them yet)
    dut.soc_full_rst = 0;
    settle();
    EXPECT(dut.cpu_rst == 0,
           "post-pulse: CPU released by bypass+release");
    EXPECT(dut.boot_rom_ready == 1,
           "post-pulse: boot_rom_ready follows the JTAG release path");
    std::printf("PASS: debug_full_reset gates the JTAG release\n");

    // ---------- Scenario 4: post-debug-reset normal-boot path ----------
    // Drop bypass.  Since boot_rom_loaded was reset by boot_fsm_rst
    // during the pulse (not modeled inline; we approximate by
    // clearing the input), boot_rom_ready should drop.
    dut.jtag_boot_bypass = 0;
    dut.jtag_boot_release = 0;
    dut.boot_rom_loaded = 0;  // boot_fsm hasn't finished re-copying yet
    settle();
    EXPECT(dut.cpu_rst == 1,
           "post-pulse normal path: CPU held until ROM reloads");

    dut.boot_rom_loaded = 1;
    settle();
    EXPECT(dut.cpu_rst == 0,
           "post-pulse: ROM reloaded → CPU released cleanly");
    std::printf("PASS: post-pulse normal-boot path\n");

    std::printf("\ntb_boot_release_gate: ALL PASS (%llu cycles)\n",
                (unsigned long long)g_cycles);
    return 0;
}
