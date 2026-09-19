// tb_audio_pwm.cpp — Verilator unit tb for rtl/sys/audio_pwm.v
//
// Exercises the first-order Σ-Δ modulator that bridges ASC signed-16
// PCM to a 1-bit-per-channel PWM output for the AN9134 speaker amp
// path.  See docs/superpowers/specs/2026-05-04-an9134-pwm-audio-design.md.
//
// What we test (mean-duty-cycle / time-average semantics — Σ-Δ is a
// time-domain noise-shaper, not a serial protocol, so we measure the
// average ON time over a long enough window):
//
//   1. reset_silence       — after reset, with sample_l = sample_r = 0,
//                             the long-time mean of pwm_l/pwm_r is 0.5.
//   2. dc_full_positive    — sample = +max int16 (0x7FFF) → mean → 1.0.
//   3. dc_full_negative    — sample = -max int16 (0x8000) → mean → 0.0.
//   4. dc_quarter_positive — sample = 0x4000 (≈+0.5) → mean ≈ 0.75.
//   5. dc_quarter_negative — sample = 0xC000 (≈-0.5) → mean ≈ 0.25.
//   6. sample_hold_latch   — pulse audio_sample_valid once with a fresh
//                             value; without further pulses the
//                             modulator's mean tracks the LATEST sample.
//   7. channel_independence — drive sample_l = +full, sample_r = -full;
//                             pwm_l mean → 1.0, pwm_r mean → 0.0,
//                             no cross-coupling.
//   8. reset_clears_err    — drive a DC offset, deassert reset, observe
//                             clean settle to the new mean.  Then re-
//                             apply reset and confirm err clears (mean
//                             returns to 0.5 for sample = 0).
//
// Build: `make tb-audio-pwm`.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <verilated.h>
#include "Vaudio_pwm.h"

static Vaudio_pwm* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0, n_fail = 0;

// Long-window count for mean-duty measurements.  At OSR ≈ 2247 (real
// hardware), one sample period is ~2247 cycles.  For DC-input mean
// measurements the integrator wraps fully every (2^16 / x_u) cycles,
// so 65536 cycles is enough to reach steady-state mean for any DC
// input down to 1 LSB.  Use 131072 (= 2^17) for margin.
static const int MEAN_CYCLES = 131072;

static void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    sim_time++;
}

static void idle_inputs() {
    dut->sample_l     = 0;
    dut->sample_r     = 0;
    dut->sample_valid = 0;
}

static void reset() {
    idle_inputs();
    dut->rst = 1;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;
    tick();
}

// Set both channels to a sample value and pulse sample_valid for one
// clock so the holding regs latch.
static void push_sample(int16_t l, int16_t r) {
    dut->sample_l     = (uint16_t)l;
    dut->sample_r     = (uint16_t)r;
    dut->sample_valid = 1;
    tick();
    dut->sample_valid = 0;
    // Re-establish the held inputs on the next cycle's input lines so
    // any subsequent sample_valid pulse picks up the same intent.
    dut->sample_l = (uint16_t)l;
    dut->sample_r = (uint16_t)r;
}

// Tick `n` cycles and return (sum_l, sum_r) of pwm bits over that span.
struct DutyCount { uint64_t sum_l; uint64_t sum_r; };

static DutyCount count_duty(int n_cycles) {
    DutyCount d{0, 0};
    for (int i = 0; i < n_cycles; i++) {
        tick();
        d.sum_l += dut->pwm_l;
        d.sum_r += dut->pwm_r;
    }
    return d;
}

// Skip a settling window (modulator transient after a step) before
// measuring the mean.  At MEAN_CYCLES the transient is well under 1 %.
static const int SETTLE_CYCLES = 8192;

#define CHECK_NEAR(msg, got, exp, tol) do { \
    double _g = (double)(got); \
    double _e = (double)(exp); \
    double _t = (double)(tol); \
    if (!(_g >= _e - _t && _g <= _e + _t)) { \
        std::printf("    FAIL %s: got %.6f, expected %.6f ± %.6f\n", \
                    (msg), _g, _e, _t); \
        return false; \
    } \
} while (0)

#define CHECK_EQ(msg, got, exp) do { \
    if ((int64_t)(got) != (int64_t)(exp)) { \
        std::printf("    FAIL %s: got %lld, expected %lld\n", (msg), \
                    (long long)(got), (long long)(exp)); \
        return false; \
    } \
} while (0)

// Mean duty cycle of a channel over MEAN_CYCLES, after SETTLE_CYCLES of
// burn-in.  Returned in [0.0, 1.0].
struct MeanResult { double mean_l; double mean_r; };

static MeanResult measure_mean() {
    // Burn in.
    count_duty(SETTLE_CYCLES);
    DutyCount d = count_duty(MEAN_CYCLES);
    return MeanResult{
        (double)d.sum_l / (double)MEAN_CYCLES,
        (double)d.sum_r / (double)MEAN_CYCLES
    };
}

// ─── Scenarios ─────────────────────────────────────────────────────────
static bool test_reset_silence() {
    reset();
    // Silence: pwm should average to 0.5.
    MeanResult m = measure_mean();
    // Tolerance: with a 16-bit accumulator at silence (x_u = 0x8000),
    // the modulator alternates 0/1 perfectly, so mean is exactly 0.5
    // up to the half-cycle at the end of the window.
    CHECK_NEAR("silence mean L", m.mean_l, 0.5, 1.0 / MEAN_CYCLES + 1e-6);
    CHECK_NEAR("silence mean R", m.mean_r, 0.5, 1.0 / MEAN_CYCLES + 1e-6);
    return true;
}

static bool test_dc_full_positive() {
    reset();
    push_sample((int16_t)0x7FFF, (int16_t)0x7FFF);
    MeanResult m = measure_mean();
    // x_u = 0xFFFF → modulator should output 1 almost all the time
    // (one 0 per 65536 cycles for the 1-LSB shy of full-scale).
    CHECK_NEAR("full+ mean L", m.mean_l, 65535.0/65536.0, 2.0/65536.0);
    CHECK_NEAR("full+ mean R", m.mean_r, 65535.0/65536.0, 2.0/65536.0);
    return true;
}

static bool test_dc_full_negative() {
    reset();
    // 0x8000 = -32768 signed → x_u = 0x0000 → modulator never fires.
    push_sample((int16_t)0x8000, (int16_t)0x8000);
    MeanResult m = measure_mean();
    CHECK_NEAR("full- mean L", m.mean_l, 0.0, 1.0/MEAN_CYCLES + 1e-6);
    CHECK_NEAR("full- mean R", m.mean_r, 0.0, 1.0/MEAN_CYCLES + 1e-6);
    return true;
}

static bool test_dc_quarter_positive() {
    reset();
    // sample = +0x4000 → x_u = 0xC000 → mean → 0xC000 / 0x10000 = 0.75
    push_sample((int16_t)0x4000, (int16_t)0x4000);
    MeanResult m = measure_mean();
    CHECK_NEAR("0.5+ mean L", m.mean_l, 0.75, 2.0/65536.0);
    CHECK_NEAR("0.5+ mean R", m.mean_r, 0.75, 2.0/65536.0);
    return true;
}

static bool test_dc_quarter_negative() {
    reset();
    // sample = -0x4000 = 0xC000 unsigned (signed -16384) → x_u = 0x4000
    //   → mean → 0x4000 / 0x10000 = 0.25
    push_sample((int16_t)0xC000, (int16_t)0xC000);
    MeanResult m = measure_mean();
    CHECK_NEAR("0.5- mean L", m.mean_l, 0.25, 2.0/65536.0);
    CHECK_NEAR("0.5- mean R", m.mean_r, 0.25, 2.0/65536.0);
    return true;
}

static bool test_sample_hold_latch() {
    reset();
    // Push +full on both, run a window, then switch to -full WITHOUT
    // a new sample_valid.  Without a fresh latch the modulator should
    // keep tracking the latched sample.
    push_sample((int16_t)0x7FFF, (int16_t)0x7FFF);
    MeanResult m1 = measure_mean();
    CHECK_NEAR("hold+ initial mean L", m1.mean_l, 65535.0/65536.0, 2.0/65536.0);

    // Change input lines but DO NOT pulse sample_valid.
    dut->sample_l     = 0x8000;
    dut->sample_r     = 0x8000;
    dut->sample_valid = 0;
    MeanResult m2 = measure_mean();
    CHECK_NEAR("hold+ unchanged after no latch L", m2.mean_l,
               65535.0/65536.0, 2.0/65536.0);
    CHECK_NEAR("hold+ unchanged after no latch R", m2.mean_r,
               65535.0/65536.0, 2.0/65536.0);

    // Now actually latch -full.
    push_sample((int16_t)0x8000, (int16_t)0x8000);
    MeanResult m3 = measure_mean();
    CHECK_NEAR("hold- after latch L", m3.mean_l, 0.0, 1.0/MEAN_CYCLES + 1e-6);
    CHECK_NEAR("hold- after latch R", m3.mean_r, 0.0, 1.0/MEAN_CYCLES + 1e-6);
    return true;
}

static bool test_channel_independence() {
    reset();
    push_sample((int16_t)0x7FFF, (int16_t)0x8000);
    MeanResult m = measure_mean();
    CHECK_NEAR("L=+full mean", m.mean_l, 65535.0/65536.0, 2.0/65536.0);
    CHECK_NEAR("R=-full mean", m.mean_r, 0.0,             1.0/MEAN_CYCLES + 1e-6);
    return true;
}

static bool test_reset_clears_err() {
    reset();
    // Drive a DC offset for a while.
    push_sample((int16_t)0x4000, (int16_t)0x4000);
    count_duty(MEAN_CYCLES);   // accumulate residue in err

    // Re-reset.  err should clear; pwm goes back to silence behaviour.
    dut->rst = 1;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;
    tick();
    // Inputs are zero by default after reset; mean should be 0.5.
    idle_inputs();
    MeanResult m = measure_mean();
    CHECK_NEAR("post-rst mean L", m.mean_l, 0.5, 1.0/MEAN_CYCLES + 1e-6);
    CHECK_NEAR("post-rst mean R", m.mean_r, 0.5, 1.0/MEAN_CYCLES + 1e-6);
    // And explicit-state check: err regs must be 0 (we can't peek at
    // them directly without --public, but post-reset hold is 0 and
    // mean = 0.5 imply err is the canonical silence-state value).
    return true;
}

// ─── main ──────────────────────────────────────────────────────────────
#define RUN(fn) do { \
    bool ok = fn(); \
    if (ok) { std::printf("[PASS] %s\n", #fn); n_pass++; } \
    else    { std::printf("[FAIL] %s\n", #fn); n_fail++; } \
} while (0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vaudio_pwm;

    RUN(test_reset_silence);
    RUN(test_dc_full_positive);
    RUN(test_dc_full_negative);
    RUN(test_dc_quarter_positive);
    RUN(test_dc_quarter_negative);
    RUN(test_sample_hold_latch);
    RUN(test_channel_independence);
    RUN(test_reset_clears_err);

    std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
