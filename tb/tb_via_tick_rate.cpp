// tb_via_tick_rate.cpp — measures the Mac 60 Hz tick, end to end.
//
// The chain under test is the one `fpga_top_peripherals.vh` actually
// builds today:
//
//   pb_clk (50 MHz) → phi2 NCO (783 360 Hz) → VIA2 T1 free-run + ACR[7]
//   → via2_pb_out[7] → via1_ca1_in → VIA1 IFR.CA1 → via1_irq → 68k L1
//
// This gate exists because the two pre-existing gates cannot see a wrong
// rate:
//
//   * `tb_via2`'s `test_t1_freerun_pb7` asserts only that PB7 *toggles*.
//     It passes identically at 1.4 Hz and at 60 Hz.
//   * `tb_vbl_rate` measures a chain the SoC no longer has: it hard-wires
//     `via1_ca1_in = dafb_vbl_level` (the HDMI-VTG shortcut), which was
//     replaced by the VIA2-PB7 board wire.
//
// So every check here is a *rate* in Hz measured from simulated time, and
// every one of them is a two-sided band.  A test that would pass at both
// 1.4 Hz and 60 Hz is not verification.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <verilated.h>
#include "Vtb_via_tick_rate.h"

static Vtb_via_tick_rate* dut = nullptr;
static uint64_t cycles = 0;
static int n_pass = 0, n_fail = 0;

// Must match the wrapper's parameters (and fpga_top.v's defaults).
static constexpr double PB_CLK_HZ   = 50e6;
static constexpr double VIA_PHI2_HZ = 783360.0;

// What the Q700 ROM programs into VIA2 T1, read back live over JTAG on
// build 0x0BC53B30 (see the RESOLVED 2026-07-26 note in
// rtl/soc/fpga_top_peripherals.vh):
//     ACR = 0xC0, T1LH:T1LL = 0x196E
static constexpr uint16_t ROM_T1_LATCH = 0x196E;
static constexpr uint8_t  ROM_VIA2_ACR = 0xC0;

// A 6522 free-running T1 wraps every (N+2) phi2 periods (one extra for
// the wrap cycle, one for the reload cycle); PB7 toggles on each wrap, so
// a full square-wave period — one CA1 edge of the programmed polarity —
// is 2*(N+2) phi2 periods.
static double expected_tick_hz(uint16_t latch) {
    return VIA_PHI2_HZ / (2.0 * (double(latch) + 2.0));
}

#define CHECK(cond, fmt, ...) do {                                   \
    if (cond) { n_pass++; std::printf("  [PASS] " fmt "\n", ##__VA_ARGS__); } \
    else      { n_fail++; std::printf("  [FAIL] " fmt "\n", ##__VA_ARGS__); } \
} while (0)

// ── One pb_clk cycle ───────────────────────────────────────────────────
static uint64_t phi2_count = 0;
static uint64_t pb7_edges  = 0;
static uint64_t irq_edges  = 0;
static int prev_pb7 = -1, prev_irq = 0;

static void step() {
    dut->pb_clk = 0; dut->eval();
    dut->pb_clk = 1; dut->eval();
    cycles++;
    if (dut->phi2_tick_o) phi2_count++;
    if (prev_pb7 >= 0 && dut->via2_pb7 != prev_pb7) pb7_edges++;
    prev_pb7 = dut->via2_pb7;
    if (dut->via1_irq && !prev_irq) irq_edges++;
    prev_irq = dut->via1_irq;
}

static void idle(uint64_t n) { for (uint64_t i = 0; i < n; i++) step(); }

static void via2_write(uint8_t addr, uint8_t data) {
    dut->via2_addr = addr & 0xF; dut->via2_wdata = data;
    dut->via2_wr = 1; dut->via2_rd = 0;
    step();
    dut->via2_wr = 0; dut->via2_wdata = 0;
}

static uint8_t via2_read(uint8_t addr) {
    dut->via2_addr = addr & 0xF; dut->via2_rd = 1; dut->via2_wr = 0;
    step();
    dut->via2_rd = 0; dut->eval();
    return dut->via2_rdata & 0xFF;
}

static void via1_write(uint8_t addr, uint8_t data) {
    dut->via1_addr = addr & 0xF; dut->via1_wdata = data;
    dut->via1_wr = 1; dut->via1_rd = 0;
    step();
    dut->via1_wr = 0; dut->via1_wdata = 0;
}

static uint8_t via1_read(uint8_t addr) {
    dut->via1_addr = addr & 0xF; dut->via1_rd = 1; dut->via1_wr = 0;
    step();
    dut->via1_rd = 0; dut->eval();
    return dut->via1_rdata & 0xFF;
}

static void reset_dut() {
    dut->pb_rst = 1;
    dut->via1_addr = dut->via2_addr = 0;
    dut->via1_wdata = dut->via2_wdata = 0;
    dut->via1_wr = dut->via1_rd = 0;
    dut->via2_wr = dut->via2_rd = 0;
    dut->pb_clk = 0; dut->eval();
    for (int i = 0; i < 8; i++) step();
    dut->pb_rst = 0;
    prev_pb7 = -1; prev_irq = 0;
    step(); step();
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_via_tick_rate;

    std::printf("tb_via_tick_rate — Q700 60 Hz tick chain rate gate\n");
    std::printf("  pb_clk       = %.0f Hz\n", PB_CLK_HZ);
    std::printf("  VIA_PHI2_HZ  = %.0f Hz (target)\n", VIA_PHI2_HZ);
    std::printf("  VIA2 T1LH:LL = 0x%04X (ROM value), ACR = 0x%02X\n\n",
                ROM_T1_LATCH, ROM_VIA2_ACR);

    reset_dut();

    // ── Phase A — is phi2_tick itself at the right rate? ──────────────
    // 0.05 simulated seconds is 2.5 M pb_clk cycles: enough for the NCO's
    // average to settle to well under 0.01 %.
    {
        uint64_t c0 = cycles, p0 = phi2_count;
        idle(uint64_t(PB_CLK_HZ * 0.05));
        double secs = double(cycles - c0) / PB_CLK_HZ;
        double hz   = double(phi2_count - p0) / secs;
        std::printf("Phase A — phi2_tick\n");
        std::printf("  measured %.1f Hz over %.4f s (expected %.0f Hz)\n",
                    hz, secs, VIA_PHI2_HZ);
        CHECK(std::fabs(hz - VIA_PHI2_HZ) / VIA_PHI2_HZ < 0.001,
              "phi2_tick within 0.1%% of %.0f Hz (got %.1f Hz)",
              VIA_PHI2_HZ, hz);
    }

    // ── Measure one (latch, window) point ─────────────────────────────
    struct Result { double pb7_hz; double tick_hz; };

    auto measure = [&](uint16_t latch, double window_s) -> Result {
        // VIA2: ACR[7:6] = 11 → T1 free-run, PB7 output.
        via2_write(11, ROM_VIA2_ACR);
        via2_write(4, latch & 0xFF);        // T1CL → latch low
        via2_write(5, (latch >> 8) & 0xFF); // T1CH → load counter + arm
        // VIA1: PCR[0] = 1 → CA1 latches on the low-to-high edge; IER
        // bit 1 (CA1) enabled via the set-form write (bit 7 = 1).
        via1_write(12, 0x01);
        via1_write(14, 0x82);

        prev_pb7 = dut->via2_pb7;
        uint64_t c0 = cycles, e0 = pb7_edges, i0 = irq_edges;
        uint64_t n = uint64_t(PB_CLK_HZ * window_s);
        for (uint64_t i = 0; i < n; i++) {
            if (dut->via1_irq) {
                // The 68k VBL handler acks by reading VIA1 ORA (reg 1),
                // which clears IFR.CA1 per the 6522 spec.
                via1_read(1);
            } else {
                step();
            }
        }
        double secs = double(cycles - c0) / PB_CLK_HZ;
        return Result{ double(pb7_edges - e0) / secs,
                       double(irq_edges  - i0) / secs };
    };

    // ── Phase B — the ROM's own programming ───────────────────────────
    // Window: 2 simulated seconds.  At the correct rate that is ~120 CA1
    // interrupts; at the rate measured on hardware (~1.4 Hz) it is ~3.
    std::printf("\nPhase B — VIA2 T1 → PB7 → VIA1 CA1, ROM latch 0x%04X"
                " (2.0 s window)\n", ROM_T1_LATCH);
    {
        uint8_t acr = via2_read(11);
        Result r = measure(ROM_T1_LATCH, 2.0);
        double expect = expected_tick_hz(ROM_T1_LATCH);
        uint8_t ll = via2_read(6), lh = via2_read(7);

        std::printf("  PB7 toggles      : %.3f Hz\n", r.pb7_hz);
        std::printf("  VIA1 CA1 IRQs    : %.3f Hz\n", r.tick_hz);
        std::printf("  expected tick    : %.3f Hz\n", expect);

        CHECK(via2_read(11) == ROM_VIA2_ACR, "VIA2 ACR reads back 0x%02X",
              via2_read(11));
        (void)acr;
        CHECK(((lh << 8) | ll) == ROM_T1_LATCH,
              "VIA2 T1 latch reads back 0x%04X", (lh << 8) | ll);
        // Two-sided bands.  1.4 Hz fails the lower bound by 40x; a
        // double-speed regression fails the upper bound.
        CHECK(r.pb7_hz > 100.0 && r.pb7_hz < 145.0,
              "PB7 square wave 100..145 toggles/s (got %.3f)", r.pb7_hz);
        CHECK(r.tick_hz > 50.0 && r.tick_hz < 72.0,
              "VIA1 CA1 tick 50..72 Hz (got %.3f)", r.tick_hz);
        CHECK(std::fabs(r.tick_hz - expect) / expect < 0.02,
              "VIA1 CA1 tick within 2%% of %.3f Hz (got %.3f)",
              expect, r.tick_hz);
        // The Mac's Ticks global advances once per CA1 interrupt.
        std::printf("  → Ticks after 220 s would be %.0f\n",
                    r.tick_hz * 220.0);
    }

    // ── Phase C — the measurement must TRACK the programming ──────────
    // A gate that reports 60 Hz no matter what the ROM wrote would be
    // vacuous.  Halve the latch and the tick must double.
    std::printf("\nPhase C — latch tracking, latch 0x%04X (0.5 s window)\n",
                ROM_T1_LATCH / 2);
    {
        uint16_t latch = ROM_T1_LATCH / 2;
        Result r = measure(latch, 0.5);
        double expect = expected_tick_hz(latch);
        std::printf("  VIA1 CA1 IRQs    : %.3f Hz (expected %.3f Hz)\n",
                    r.tick_hz, expect);
        CHECK(std::fabs(r.tick_hz - expect) / expect < 0.02,
              "half latch → %.3f Hz, measured %.3f Hz", expect, r.tick_hz);
    }

    // ── Phase D — the floor: T1 CANNOT produce 1.4 Hz ─────────────────
    // T1's latch is 16 bits, so the slowest tick this chain can generate
    // is at latch 0xFFFF.  Measuring that floor is what rules the timer
    // out as the cause of a ~1.4 Hz hardware tick: no value the ROM or
    // Mac OS could write, and no T1-side RTL defect short of a broken
    // phi2, can get the rate below this number.
    std::printf("\nPhase D — slowest possible T1 (latch 0xFFFF, 2.0 s window)\n");
    {
        Result r = measure(0xFFFF, 2.0);
        double expect = expected_tick_hz(0xFFFF);
        std::printf("  VIA1 CA1 IRQs    : %.3f Hz (arithmetic floor %.3f Hz)\n",
                    r.tick_hz, expect);
        CHECK(std::fabs(r.tick_hz - expect) / expect < 0.05,
              "max latch → %.3f Hz, measured %.3f Hz", expect, r.tick_hz);
        CHECK(r.tick_hz > 4.0,
              "T1 floor is %.3f Hz, so a 1.4 Hz tick cannot come from this "
              "timer at ANY 16-bit latch value", r.tick_hz);
    }

    std::printf("\n%d passed, %d failed\n", n_pass, n_fail);
    delete dut;
    return n_fail ? 1 : 0;
}
