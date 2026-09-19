// tb_debug_full_reset.cpp — Verilator harness for tb_debug_full_reset.v
//
// Validates the task #256 debug-full-reset overlay re-arm semantics:
//
//  Scenario 1 (regression that motivated the task):
//      Cold boot -> overlay asserted.  ROM clears overlay (orb3_q<=0
//      via the rom_clears_overlay pulse).  After overlay drops, pulse
//      ONLY jtag_cpu_hold (vio[2]) -- this models the previous bug:
//      overlay should NOT re-arm on cpu-only-hold.
//
//  Scenario 2 (the new path):
//      After scenario 1 leaves overlay cleared, pulse
//      jtag_debug_full_rst (vio[3]) for one cycle.  Verify:
//        - reset_overlay_active_q goes back to 1
//        - via1_overlay_bit (ORB[3] image) returns to 1 (overlay live)
//        - cpu_rst is asserted while bit[3] is held
//        - releasing bit[3] keeps overlay asserted (until next clear)

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include "Vdebug_full_reset_dut.h"
#include "verilated.h"

static Vdebug_full_reset_dut *g_dut = nullptr;
static uint64_t g_cycles = 0;

static void tick(int n = 1) {
    for (int i = 0; i < n; ++i) {
        g_dut->clk = 0;
        g_dut->eval();
        g_dut->clk = 1;
        g_dut->eval();
        ++g_cycles;
        if (g_cycles > 10000) {
            std::fprintf(stderr, "tb_debug_full_reset: runaway sim\n");
            std::exit(2);
        }
    }
}

#define EXPECT(cond, msg) do { \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL[cycle=%llu]: %s\n", \
                     (unsigned long long)g_cycles, msg); \
        std::exit(1); \
    } \
} while (0)

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Vdebug_full_reset_dut dut;
    g_dut = &dut;

    // ---------- cold reset ----------
    dut.core_rst = 1;
    dut.jtag_cpu_hold = 0;
    dut.jtag_debug_full_rst = 0;
    dut.dbg_cold_reset_pulse = 0;
    dut.dbg_cold_reset_hold = 0;
    dut.rom_clears_overlay = 0;
    dut.high_rom_fetch = 0;
    tick(4);
    dut.core_rst = 0;
    tick(2);
    EXPECT(dut.reset_overlay_active_q == 1,
           "post-cold-reset: overlay should be asserted");
    EXPECT(dut.via1_overlay_bit == 1,
           "post-cold-reset: via1_overlay_bit should be 1 (DDRB[3]=0)");
    std::printf("PASS: cold reset asserts overlay\n");

    // ---------- ROM clears overlay (real boot path) ----------
    dut.rom_clears_overlay = 1;
    tick(1);
    dut.rom_clears_overlay = 0;
    tick(2);
    EXPECT(dut.via1_overlay_bit == 0,
           "after ROM-clears-overlay: via1_overlay_bit should be 0");
    EXPECT(dut.reset_overlay_active_q == 0,
           "after ROM-clears-overlay: reset_overlay_active_q should drop");
    std::printf("PASS: ROM clears overlay drops the alias\n");

    // ---------- Scenario 1: jtag_cpu_hold alone does NOT re-arm ----
    // This is the documented BUG we are guarding against — the
    // legacy CPU-only-halt path must NOT bring the overlay back
    // (use case: the user wants to single-step / patch memory while
    // preserving the post-boot architectural state).
    dut.jtag_cpu_hold = 1;
    tick(4);
    EXPECT(dut.cpu_rst == 1, "jtag_cpu_hold should drive cpu_rst");
    EXPECT(dut.reset_overlay_active_q == 0,
           "jtag_cpu_hold alone must NOT re-arm overlay (preserves "
           "halt-time arch state)");
    EXPECT(dut.via1_overlay_bit == 0,
           "jtag_cpu_hold alone must NOT reset VIA1 ORB[3]");
    dut.jtag_cpu_hold = 0;
    tick(2);
    EXPECT(dut.cpu_rst == 0, "release: cpu_rst drops");
    EXPECT(dut.reset_overlay_active_q == 0,
           "release: overlay still cleared (halt was non-cold)");
    std::printf("PASS: jtag_cpu_hold preserves overlay state\n");

    // ---------- Scenario 2: jtag_debug_full_rst RE-ARMS ------------
    dut.jtag_debug_full_rst = 1;
    tick(2);
    EXPECT(dut.cpu_rst == 1,
           "jtag_debug_full_rst should drive cpu_rst (via soc_full_rst)");
    EXPECT(dut.reset_overlay_active_q == 1,
           "jtag_debug_full_rst MUST re-arm reset_overlay_active_q "
           "(task #256 fix)");
    EXPECT(dut.via1_overlay_bit == 1,
           "jtag_debug_full_rst MUST reset VIA1 ORB[3] so "
           "via1_overlay_bit returns to 1 (otherwise overlay would "
           "self-clear next cycle via !via1_overlay_bit)");
    std::printf("PASS: debug-full-reset re-arms overlay + VIA1\n");

    // ---------- Scenario 3: release lets CPU resume cold ----------
    dut.jtag_debug_full_rst = 0;
    tick(4);
    EXPECT(dut.cpu_rst == 0,
           "after debug-full-reset release: cpu_rst drops");
    EXPECT(dut.reset_overlay_active_q == 1,
           "after debug-full-reset release: overlay STILL asserted "
           "(VIA1 holds ORB[3]=1 until ROM re-clears it)");
    EXPECT(dut.via1_overlay_bit == 1,
           "after debug-full-reset release: via1_overlay_bit STILL 1");
    std::printf("PASS: post-release CPU resumes with overlay live\n");

    // ---------- Scenario 4: ROM can clear overlay again -----------
    dut.rom_clears_overlay = 1;
    tick(1);
    dut.rom_clears_overlay = 0;
    tick(2);
    EXPECT(dut.via1_overlay_bit == 0,
           "second-boot ROM clear: via1_overlay_bit should drop to 0");
    EXPECT(dut.reset_overlay_active_q == 0,
           "second-boot ROM clear: reset_overlay_active_q should drop");
    std::printf("PASS: post-debug-full-reset ROM can clear overlay\n");

    // ---------- Scenario 5: simultaneous cpu-hold + full-reset ----
    // Full-reset wins (covers both regardless of layering).
    dut.jtag_cpu_hold = 1;
    dut.jtag_debug_full_rst = 1;
    tick(2);
    EXPECT(dut.cpu_rst == 1, "both holds: cpu_rst asserted");
    EXPECT(dut.reset_overlay_active_q == 1,
           "both holds: full-reset wins, overlay re-armed");
    dut.jtag_cpu_hold = 0;
    dut.jtag_debug_full_rst = 0;
    tick(2);
    std::printf("PASS: cpu-hold + full-reset combined behaves as full reset\n");

    // Setup for unified-reset scenarios: clear overlay via ROM so we can
    // observe re-arm from a non-overlay state.
    dut.rom_clears_overlay = 1;
    tick(1);
    dut.rom_clears_overlay = 0;
    tick(2);
    EXPECT(dut.via1_overlay_bit == 0,
           "pre-unified-reset: overlay cleared after ROM");

    // ---------- Scenario 6: dbg_cold_reset_pulse (DBG_CONTROL bit 5) -
    //
    // Models a JTAG-AXI host write of DBG_CONTROL.cold_reset_pulse=1
    // (one-cycle pulse from debug_ctrl).  The pulse must drive
    // soc_full_rst (re-arming overlay + VIA1 ORB[3]) exactly like the
    // legacy VIO bit 3 / btn[2] paths do.
    dut.dbg_cold_reset_pulse = 1;
    tick(1);
    dut.dbg_cold_reset_pulse = 0;
    tick(2);
    EXPECT(dut.reset_overlay_active_q == 1,
           "cold_reset_pulse MUST re-arm reset_overlay_active_q");
    EXPECT(dut.via1_overlay_bit == 1,
           "cold_reset_pulse MUST reset VIA1 ORB[3] (overlay re-asserts)");
    std::printf("PASS: dbg_cold_reset_pulse (bit 5) drives unified reset\n");

    // ---------- Scenario 7: cold_reset_hold survives the pulse ------
    //
    // The host pattern: set hold, fire pulse, leave held.  After the
    // pulse drops, cold_reset_hold (a sticky bit on `core_rst` in
    // production) keeps cpu_rst asserted.  The CPU stays in reset until
    // the host explicitly clears the hold bit.

    // First clear overlay so we have a clean precondition.
    dut.rom_clears_overlay = 1;
    tick(1);
    dut.rom_clears_overlay = 0;
    tick(2);
    EXPECT(dut.via1_overlay_bit == 0,
           "pre-S7: overlay cleared");

    // 1. Set hold + pulse together (exactly what the new RTL does on a
    //    DBG_CONTROL write of (CTL_COLD_RESET_HOLD | CTL_COLD_RESET_PULSE)).
    dut.dbg_cold_reset_hold = 1;
    dut.dbg_cold_reset_pulse = 1;
    tick(1);
    EXPECT(dut.cpu_rst == 1,
           "S7: pulse+hold cycle: cpu_rst asserted");
    EXPECT(dut.soc_full_rst == 1,
           "S7: pulse+hold cycle: soc_full_rst asserted");
    // 2. Pulse auto-clears (one-cycle in production).
    dut.dbg_cold_reset_pulse = 0;
    tick(4);
    EXPECT(dut.soc_full_rst == 0,
           "S7: post-pulse: soc_full_rst deasserted (pulse consumed)");
    EXPECT(dut.cpu_rst == 1,
           "S7: post-pulse: cpu_rst STILL asserted by cold_reset_hold");
    EXPECT(dut.reset_overlay_active_q == 1,
           "S7: post-pulse: overlay re-armed by the pulse");
    // 3. Host clears hold — CPU finally resumes.
    dut.dbg_cold_reset_hold = 0;
    tick(2);
    EXPECT(dut.cpu_rst == 0,
           "S7: hold cleared: cpu_rst deasserts (CPU resumes)");
    EXPECT(dut.reset_overlay_active_q == 1,
           "S7: hold cleared: overlay still asserted (VIA1 holds ORB[3]=1)");
    std::printf("PASS: cold_reset_hold survives the pulse it triggers\n");

    // ---------- Scenario 8: cold_reset_hold without pulse ------------
    //
    // The host can also set hold WITHOUT firing a pulse — equivalent to
    // an asynchronous CPU stop, but with stronger guarantees than
    // ctrl_halt_req (hold survives any reset whereas halt_req does not).
    // Verifies that hold alone gates cpu_rst but does NOT touch
    // soc_full_rst / overlay.
    dut.dbg_cold_reset_hold = 1;
    tick(2);
    EXPECT(dut.cpu_rst == 1,
           "S8: hold-only: cpu_rst asserted");
    EXPECT(dut.soc_full_rst == 0,
           "S8: hold-only: soc_full_rst NOT asserted (no pulse fired)");
    EXPECT(dut.reset_overlay_active_q == 1,
           "S8: hold-only: overlay state unchanged from previous scenario");
    dut.dbg_cold_reset_hold = 0;
    tick(2);
    EXPECT(dut.cpu_rst == 0, "S8: hold released: cpu_rst deasserts");
    std::printf("PASS: cold_reset_hold alone gates cpu_rst (no pulse path)\n");

    std::printf("\ntb_debug_full_reset: ALL PASS (%llu cycles)\n",
                (unsigned long long)g_cycles);
    return 0;
}
