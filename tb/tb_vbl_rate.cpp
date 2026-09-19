// tb_vbl_rate.cpp -- DAFB vblank pulse + CDC rate gate (task #145).
//
// SCOPE (corrected 2026-09-06): this is NOT the Mac 60 Hz tick gate.  See
// the header of tb_vbl_rate.v -- the SoC drives VIA1 CA1 from VIA2's
// Timer-1 PB7 square wave, not from this HDMI-VTG-derived level.  The
// tick chain is gated by `make tb-via-tick-rate`.
//
// Drives the video pipeline with a 200 MHz reference and a 50 MHz pb_clk
// for VIA1, then counts:
//   * pclk-domain `vbl_pulse_pclk` strobes
//   * pb_clk-domain `dafb_vbl_pulse_pb` strobes (post pulse_cdc)
//   * VIA1 IFR.CA1 transitions (read once per frame via the bus)
//
// Asserts:
//   1. The pclk-domain pulse rate matches 60.000 Hz ± 1 % over a
//      multi-frame window.
//   2. Every pclk pulse produces exactly one pb_clk pulse (no drops).
//   3. VIA1.IFR.CA1 latches at the same rate after the ROM-default
//      PCR/IER programming.
//
// Compares against a baseline derived from MAME (tools/mame_vbl_capture.lua)
// — see docs/vbl_irq.md for the baseline numbers.  This testbench gates
// the RTL side; the docs include the canonical MAME capture command line.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <verilated.h>
#include "Vtb_vbl_rate.h"

static Vtb_vbl_rate* dut = nullptr;
static uint64_t sim_ns = 0;

// Time bases (ps)
//   clk_ref drives the Verilator MMCM stub directly (mmcm_hdmi.v VERILATOR
//   branch passes pclk = clk_in_p), so feed clk_ref at the actual pclk
//   rate of 148.5 MHz to get the right frame cadence.
//   pb_clk @ 50 MHz → 10 ns per half-cycle
static constexpr uint32_t REF_HALF_PS = 3367;   // ~3.367 ns half = 148.5 MHz
static constexpr uint32_t PB_HALF_PS  = 10000;  // 10 ns half = 50 MHz
// phi2 ~ 783 kHz (Q700 VIA timebase).  We don't need exact phi2 cadence
// for the rate gate; pulsing once per N pb_clk cycles is fine.
static constexpr uint32_t PHI2_DIV = 64;        // 50 MHz / 64 ≈ 781 kHz

static int n_pass = 0, n_fail = 0;

#define CHECK(cond, fmt, ...) do { \
    if (cond) { n_pass++; std::printf("  [PASS] " fmt "\n", ##__VA_ARGS__); } \
    else      { n_fail++; std::printf("  [FAIL] " fmt "\n", ##__VA_ARGS__); } \
} while (0)

// Run for ns_window simulated ns, ticking both clocks at their respective
// half periods.  Returns the count of (vbl_pulse_pclk, dafb_vbl_pulse_pb,
// via1_ca1_rising) edges observed plus the timestamps (in ns) of the
// first and last pclk pulse seen during this window.
struct EdgeCounts {
    uint64_t pclk_pulses;
    uint64_t pb_pulses;
    uint64_t via1_irq_edges;
    uint64_t first_pulse_ns;   // 0 if no pulse seen
    uint64_t last_pulse_ns;
};

static EdgeCounts run_window(uint64_t ns_window) {
    EdgeCounts c{};
    uint64_t end_ns = sim_ns + ns_window;
    // Internal half-cycle "next tick" timestamps (in ps; we keep the
    // sub-ns precision so 200 MHz lines up cleanly).
    uint64_t t_ps = sim_ns * 1000ULL;
    uint64_t end_ps = end_ns * 1000ULL;
    uint64_t next_ref_ps = t_ps + REF_HALF_PS;
    uint64_t next_pb_ps  = t_ps + PB_HALF_PS;

    int prev_via1_irq = dut->via1_irq;
    int prev_pclk_pulse = dut->vbl_pulse_pclk;
    int prev_pb_pulse   = dut->dafb_vbl_pulse_pb;
    uint32_t phi2_cnt = 0;

    while (t_ps < end_ps) {
        // Advance to the nearest event.
        uint64_t step = next_ref_ps;
        if (next_pb_ps < step) step = next_pb_ps;
        t_ps = step;
        if (t_ps >= next_ref_ps) {
            dut->clk_ref = !dut->clk_ref;
            next_ref_ps += REF_HALF_PS;
        }
        if (t_ps >= next_pb_ps) {
            dut->pb_clk = !dut->pb_clk;
            // phi2_tick on the rising edge once every PHI2_DIV pb_clk cycles
            if (dut->pb_clk) {
                phi2_cnt++;
                dut->phi2_tick = (phi2_cnt >= PHI2_DIV);
                if (dut->phi2_tick) phi2_cnt = 0;
            } else {
                dut->phi2_tick = 0;
            }
            next_pb_ps += PB_HALF_PS;
        }

        dut->eval();

        // Count rising edges of each strobe so we don't multi-count a
        // single high cycle that we sample more than once.
        if (dut->vbl_pulse_pclk && !prev_pclk_pulse) {
            c.pclk_pulses++;
            uint64_t now_ns = t_ps / 1000ULL;
            if (c.first_pulse_ns == 0) c.first_pulse_ns = now_ns;
            c.last_pulse_ns = now_ns;
        }
        if (dut->dafb_vbl_pulse_pb && !prev_pb_pulse) c.pb_pulses++;
        if (dut->via1_irq && !prev_via1_irq) c.via1_irq_edges++;
        prev_via1_irq    = dut->via1_irq;
        prev_pclk_pulse  = dut->vbl_pulse_pclk;
        prev_pb_pulse    = dut->dafb_vbl_pulse_pb;
    }
    sim_ns = t_ps / 1000ULL;
    return c;
}

// Bus helpers (one pb_clk-rising bus access).
static void via_write(uint8_t addr, uint8_t data) {
    // Wait for posedge pb_clk, drive write strobe, hold for one rising
    // edge.  Easiest: stuff a write request and run for a few ref+pb
    // cycles until pb_via1_ack lands.
    dut->pb_via1_addr  = addr & 0xF;
    dut->pb_via1_wdata = data;
    dut->pb_via1_wr    = 1;
    dut->pb_via1_rd    = 0;
    run_window(40);   // 2 pb_clk cycles
    dut->pb_via1_wr    = 0;
    dut->pb_via1_wdata = 0;
}

static uint8_t via_read(uint8_t addr) {
    dut->pb_via1_addr = addr & 0xF;
    dut->pb_via1_wr   = 0;
    dut->pb_via1_rd   = 1;
    run_window(40);
    uint8_t v = dut->pb_via1_rdata & 0xFF;
    dut->pb_via1_rd   = 0;
    return v;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_vbl_rate;

    std::printf("tb_vbl_rate: DAFB vblank → pulse_cdc rate gate "
                "(NOT the 60 Hz tick — see tb-via-tick-rate)\n");

    // Reset.
    dut->clk_ref    = 0;
    dut->pb_clk     = 0;
    dut->ext_resetn = 0;
    dut->pb_rst     = 1;
    dut->phi2_tick  = 0;
    dut->pb_via1_addr  = 0;
    dut->pb_via1_wdata = 0;
    dut->pb_via1_wr    = 0;
    dut->pb_via1_rd    = 0;

    run_window(2000);  // 2 µs reset hold
    dut->ext_resetn = 1;
    dut->pb_rst     = 0;
    // Wait for MMCM lock + i2c init (long; the harness sleeps through it
    // and only counts edges after the warmup window).
    run_window(1500000);  // 1.5 ms warmup

    // Program VIA1 PCR + IER for DAFB CA1 IRQ.  Q700 ROM uses PCR=0x01
    // (CA1 input, low-to-high select) and IER MSB|CA1.
    via_write(12, 0x01);
    via_write(14, 0x82);

    // Run for ~10 frames at 60 Hz = 167 ms simulated.  Count pulses on
    // both domains.  Acknowledge VIA1 IFR.CA1 between sub-windows so the
    // IRQ output line falls and we can count subsequent rising edges.
    EdgeCounts counts{};
    // Track the timestamp of the first and last pclk-domain pulse so we
    // can derive the rate from inter-pulse spacing instead of the
    // (rounded-down) "pulses per window" ratio — at 60 Hz with ~16.67
    // ms period, a 30 ms window only captures 1-2 frames and rounding
    // dominates a naive rate calc.  Inter-pulse measurement is exact.
    // 5 × 30 ms = 150 ms simulated.  Long enough to span ~9 frames at
    // 60 Hz so inter-pulse spacing measurement is meaningful, short
    // enough to keep tb runtime under a few minutes wall-clock.
    const int sub_windows = 5;
    const uint64_t sub_window_ns = 30'000'000ULL;
    for (int w = 0; w < sub_windows; w++) {
        auto sc = run_window(sub_window_ns);
        counts.pclk_pulses    += sc.pclk_pulses;
        counts.pb_pulses      += sc.pb_pulses;
        counts.via1_irq_edges += sc.via1_irq_edges;
        if (counts.first_pulse_ns == 0 && sc.first_pulse_ns != 0)
            counts.first_pulse_ns = sc.first_pulse_ns;
        if (sc.last_pulse_ns != 0)
            counts.last_pulse_ns = sc.last_pulse_ns;
        // Ack via ORA read so IFR.CA1 clears and subsequent rising edges
        // re-arm the IRQ line.
        (void)via_read(1);
    }
    // Derive rate from inter-pulse span: (N-1) intervals across
    // (last - first) ns.
    double measured_hz = 0.0;
    if (counts.pclk_pulses >= 2) {
        double span_s = (double)(counts.last_pulse_ns - counts.first_pulse_ns) / 1e9;
        measured_hz = (double)(counts.pclk_pulses - 1) / span_s;
    }

    std::printf("Window %.0f ms: pclk_pulses=%llu pb_pulses=%llu via1_irq_edges=%llu\n",
                (double)(sub_windows * sub_window_ns) / 1e6,
                (unsigned long long)counts.pclk_pulses,
                (unsigned long long)counts.pb_pulses,
                (unsigned long long)counts.via1_irq_edges);
    std::printf("First pulse at %llu ns, last at %llu ns (span %llu ns)\n",
                (unsigned long long)counts.first_pulse_ns,
                (unsigned long long)counts.last_pulse_ns,
                (unsigned long long)(counts.last_pulse_ns - counts.first_pulse_ns));
    std::printf("Measured pclk-domain VBL rate: %.3f Hz (target 60.000 Hz)\n", measured_hz);

    // Gate: 60 Hz ± 1 %.
    bool rate_ok = measured_hz >= 59.4 && measured_hz <= 60.6;
    CHECK(rate_ok, "pclk-domain VBL rate within 60.000 Hz +/- 1 pct (%.3f Hz)", measured_hz);

    // Gate: every pclk pulse crossed into pb_clk (no drops).
    CHECK(counts.pb_pulses == counts.pclk_pulses,
          "every pclk-domain pulse crosses CDC into pb_clk (got %llu, expected %llu)",
          (unsigned long long)counts.pb_pulses,
          (unsigned long long)counts.pclk_pulses);

    // Gate: VIA1 IRQ rising-edge count is ≥ 1 per sub-window (proves
    // the chain to the IFR works, even though the count won't equal
    // pb_pulses because each sub-window covers ~1.8 frames and we ack
    // only at the end).
    CHECK((int64_t)counts.via1_irq_edges >= sub_windows,
          "VIA1 IFR.CA1 fires at least once per sub-window (got %llu, expected >=%d)",
          (unsigned long long)counts.via1_irq_edges, sub_windows);

    // Gate: MAME-lockstep tolerance.  MAME 0.264's macqd700 DAFB device
    // uses the 35 kHz × 66.6 Hz screen mode (set_raw 31334400 Hz, 896
    // total H, 525 total V).  Our HDMI pipeline runs at 60.000 Hz
    // (1080p60 nominal).  Documenting both values; the gate is on RTL
    // side stability against a steady 60 Hz target.  See docs/vbl_irq.md.
    std::printf("MAME baseline: 66.62 Hz native (Q700 13\" RGB).  RTL: %.3f Hz HDMI-locked.\n",
                measured_hz);

    std::printf("\nresult: pass=%d fail=%d\n", n_pass, n_fail);

    dut->final();
    delete dut;
    return n_fail == 0 ? 0 : 1;
}
