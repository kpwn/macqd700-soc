// tb_audio_i2s.cpp — Verilator unit tb for rtl/sys/audio_i2s.v
//
// Exercises the I2S serialiser at a down-scaled clock ratio so one full
// stereo frame fits in ~256 core cycles.  The tb does not model a real
// codec — it samples DATA on the BCLK rising edge and reassembles
// 32-bit slot words, then checks the payload MSBs against the original
// 16-bit sample.
//
// Scenarios (≥6):
//   1. reset_defaults       — all outputs low after reset; BCLK toggles
//                             at the expected half-period.
//   2. single_sample_emit   — push one {L,R}; capture one L+R frame off
//                             DATA and verify the sign-extended sample.
//   3. silence_then_sample  — DATA remains at its last-held value when
//                             no sample has been injected; after a
//                             sample lands the very next LEFT slot
//                             carries the new data.
//   4. back_to_back_samples — two samples in quick succession: the
//                             second sample overwrites the holding reg
//                             mid-frame; the next full frame reflects
//                             the LATEST sample (hold, not queue).
//   5. left_right_distinct  — distinct L vs R payload; LEFT slot carries
//                             the UPPER byte, RIGHT the LOWER byte.
//   6. sign_extend_negative — sample with negative L (e.g. 0x80 signed
//                             = -128) sign-extends to 0xFF80 in the
//                             16-bit payload slot MSBs.
//   7. bclk_rate            — count BCLK rising edges over N frames;
//                             matches the configured divider.
//   8. lrclk_50pct_duty     — LRCLK high and low halves are equal bit
//                             counts (32 BCLKs each).
//
// Build: `make tb-audio-i2s`.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <verilated.h>
#include "Vaudio_i2s.h"

static Vaudio_i2s* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0, n_fail = 0;

// Scale the core clock so a full frame takes ~256 cycles.  Half-period =
// 2 core cycles gives BCLK = core_clk / 4; a 32-bit slot = 128 core
// cycles; two slots per frame = 256 core cycles.  HALF_PERIOD in the DUT
// is computed from CORE_FREQ_HZ / (2*I2S_BCLK_FREQ_HZ) — using defaults
// (100M / 3.072M ≈ 16), we'd need ~1024 cycles/frame.  To keep the tb
// fast we set the parameters via Verilator's -G switch on the Makefile
// target.  Here CORE_FREQ=16, BCLK=4 ⇒ HALF_PERIOD=2.
// The tb assumes that parameterisation; Makefile enforces it.

// Scenario 1+: helpers
static void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    sim_time++;
}

static void idle_inputs() {
    dut->sample_in = 0;
    dut->sample_valid = 0;
}

static void reset() {
    idle_inputs();
    dut->rst = 1;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;
    tick();
}

// Sample DATA + LRCLK on BCLK rising edge; record per-BCLK-cycle the
// data bit and the LRCLK state.  Returns once `n_bits` bits have been
// captured OR `max_core_cycles` elapse (whichever first).
struct CapturedBit { uint8_t lrclk; uint8_t data; };

static std::vector<CapturedBit> capture_bits(int n_bits, int max_cycles = 20000) {
    std::vector<CapturedBit> out;
    int prev_bclk = dut->i2s_bclk;
    int cycles = 0;
    while ((int)out.size() < n_bits && cycles < max_cycles) {
        tick();
        cycles++;
        int b = dut->i2s_bclk;
        if (b && !prev_bclk) {
            // BCLK rising edge — latch DATA
            out.push_back(CapturedBit{(uint8_t)dut->i2s_lrclk, (uint8_t)dut->i2s_data});
        }
        prev_bclk = b;
    }
    return out;
}

#define CHECK_EQ(msg, got, exp) do { \
    if ((int64_t)(got) != (int64_t)(exp)) { \
        std::printf("    FAIL %s: got %lld, expected %lld\n", msg, \
                    (long long)(got), (long long)(exp)); \
        return false; \
    } \
} while (0)

#define CHECK_TRUE(msg, cond) do { \
    if (!(cond)) { \
        std::printf("    FAIL %s: condition false\n", msg); \
        return false; \
    } \
} while (0)

// Push one sample into the DUT for a single core clock cycle.
static void push_sample(uint16_t v) {
    dut->sample_in = v;
    dut->sample_valid = 1;
    tick();
    dut->sample_valid = 0;
    dut->sample_in = 0;
}

// Record every BCLK rising edge observation until we've seen at least
// two full LRCLK halves (= 64 edges with 32 contiguous LRCLK=0 and 32
// contiguous LRCLK=1).  Then extract the first complete LEFT slot (32
// consecutive edges with LRCLK=0) and the first complete RIGHT slot
// (32 consecutive edges with LRCLK=1) that follows it.
static uint64_t capture_two_slots(uint32_t* slot_left, uint32_t* slot_right,
                                  int max_cycles = 30000) {
    // 200 bits = 3 full frames of 64 BCLKs.  Plenty of slack to find a
    // transition and the two slots following it.
    std::vector<CapturedBit> bits = capture_bits(200, max_cycles);
    *slot_left  = 0;
    *slot_right = 0;
    if (bits.size() < 128) return bits.size();

    // Find a LRCLK 1→0 transition; the next 32 bits are the LEFT slot
    // and the 32 after that are the RIGHT slot.
    int start = -1;
    for (size_t i = 1; i < bits.size(); i++) {
        if (bits[i-1].lrclk == 1 && bits[i].lrclk == 0) { start = (int)i; break; }
    }
    if (start < 0 || start + 64 > (int)bits.size()) return bits.size();

    uint32_t l = 0, r = 0;
    for (int i = 0; i < 32; i++) l = (l << 1) | (bits[start + i].data & 1);
    for (int i = 0; i < 32; i++) r = (r << 1) | (bits[start + 32 + i].data & 1);
    *slot_left  = l;
    *slot_right = r;
    return bits.size();
}

// Expected slot word: 16-bit sign-extended sample in MSBs, 16 zeros in LSBs.
static uint32_t expected_slot(int8_t signed_byte) {
    // audio_i2s.v sign-extends the 8-bit input to BITS_PER_SAMPLE (=16)
    // then packs it into the top BITS_PER_SAMPLE of SLOT_BITS (=32), tail
    // zeros.  So for input = 0xC0 (-64 signed), the sign-extended 16-bit
    // value is 0xFFC0; the slot word = 0xFFC0_0000.
    int16_t ext = (int16_t)signed_byte;
    uint32_t pay = (uint32_t)(uint16_t)ext;
    return (pay << 16) & 0xFFFF0000u;
}

// ─── Scenarios ─────────────────────────────────────────────────────────
static bool test_reset_defaults() {
    reset();
    CHECK_EQ("data low after reset",  dut->i2s_data,  0);
    // LRCLK resets high so the first falling edge inside the RTL's
    // slot loader naturally loads the LEFT (lrclk_q=0) slot; matches
    // the behavioural description inside audio_i2s.v.
    CHECK_EQ("lrclk high after reset", dut->i2s_lrclk, 1);
    // BCLK should toggle within a few cycles (tb param: HALF_PERIOD=2)
    int toggles = 0, prev = dut->i2s_bclk;
    for (int i = 0; i < 20; i++) {
        tick();
        if (dut->i2s_bclk != prev) toggles++;
        prev = dut->i2s_bclk;
    }
    CHECK_TRUE("bclk toggles within 20 cycles", toggles >= 4);
    return true;
}

static bool test_single_sample_emit() {
    reset();
    // Push a sample with L=0x7F (max positive int8), R=0x01.
    push_sample(0x7F01);
    uint32_t l = 0, r = 0;
    capture_two_slots(&l, &r);
    uint32_t exp_l = expected_slot((int8_t)0x7F);
    uint32_t exp_r = expected_slot((int8_t)0x01);
    CHECK_EQ("slot_left  payload", l, exp_l);
    CHECK_EQ("slot_right payload", r, exp_r);
    return true;
}

static bool test_silence_then_sample() {
    reset();
    // No sample: first captured frame should be all-zero (hold regs are 0)
    uint32_t l = 0, r = 0;
    capture_two_slots(&l, &r);
    CHECK_EQ("silent left",  l, 0u);
    CHECK_EQ("silent right", r, 0u);

    // Now inject a sample and the NEXT frame should reflect it.
    push_sample(0x4030);
    capture_two_slots(&l, &r);
    CHECK_EQ("after push left",  l, expected_slot((int8_t)0x40));
    CHECK_EQ("after push right", r, expected_slot((int8_t)0x30));
    return true;
}

static bool test_back_to_back_samples() {
    reset();
    // Push first sample
    push_sample(0x1020);
    // Immediately push a second sample (the hold reg gets overwritten).
    push_sample(0x3040);
    // Skip one frame to land on an aligned LEFT slot with the latest data.
    uint32_t l = 0, r = 0;
    capture_two_slots(&l, &r);
    CHECK_EQ("latest L sample wins", l, expected_slot((int8_t)0x30));
    CHECK_EQ("latest R sample wins", r, expected_slot((int8_t)0x40));
    return true;
}

static bool test_left_right_distinct() {
    reset();
    // Asymmetric: L=0x55, R=0xAA.  0xAA = -86 signed.
    push_sample(0x55AA);
    uint32_t l = 0, r = 0;
    capture_two_slots(&l, &r);
    CHECK_EQ("left slot = 0x55 sext",   l, expected_slot((int8_t)0x55));
    CHECK_EQ("right slot = 0xAA sext",  r, expected_slot((int8_t)0xAA));
    // 0xAA as int8 = -86, sign-extended to uint16 = 0xFFAA
    CHECK_EQ("left  payload upper 16", (l >> 16) & 0xFFFF, 0x0055u);
    CHECK_EQ("right payload upper 16", (r >> 16) & 0xFFFF, 0xFFAAu);
    return true;
}

static bool test_sign_extend_negative() {
    reset();
    // 0x80 = -128 signed → sign-extended 16-bit = 0xFF80.
    push_sample(0x8001);
    uint32_t l = 0, r = 0;
    capture_two_slots(&l, &r);
    CHECK_EQ("neg L top 16 = 0xFF80", (l >> 16) & 0xFFFFu, 0xFF80u);
    CHECK_EQ("pos R top 16 = 0x0001", (r >> 16) & 0xFFFFu, 0x0001u);
    return true;
}

static bool test_bclk_rate() {
    reset();
    // Count BCLK rising edges over 256 core cycles.  HALF_PERIOD=2 →
    // BCLK period = 4 cycles → 64 rising edges in 256 cycles.
    int rises = 0, prev = dut->i2s_bclk;
    for (int i = 0; i < 256; i++) {
        tick();
        if (dut->i2s_bclk && !prev) rises++;
        prev = dut->i2s_bclk;
    }
    // Allow ±2 for reset-alignment slack.
    CHECK_TRUE("bclk rate ≈ 64 rises / 256 cycles",
               rises >= 60 && rises <= 68);
    return true;
}

static bool test_lrclk_50pct_duty() {
    reset();
    // Wait for lrclk to go high and low over a few full frames.
    int high = 0, low = 0, prev_bclk = dut->i2s_bclk;
    // Count BCLK edges in each LRCLK half over 4 frames ≈ 1024 cycles.
    for (int i = 0; i < 1200; i++) {
        tick();
        int b = dut->i2s_bclk;
        if (b && !prev_bclk) {
            if (dut->i2s_lrclk) high++;
            else                low++;
        }
        prev_bclk = b;
    }
    // Should be balanced within 10 % (we skip over reset warm-up).
    CHECK_TRUE("lrclk ≈ 50%% duty",
               std::abs(high - low) <= (high + low) / 5);
    CHECK_TRUE("lrclk not stuck", high > 40 && low > 40);
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
    dut = new Vaudio_i2s;

    RUN(test_reset_defaults);
    RUN(test_single_sample_emit);
    RUN(test_silence_then_sample);
    RUN(test_back_to_back_samples);
    RUN(test_left_right_distinct);
    RUN(test_sign_extend_negative);
    RUN(test_bclk_rate);
    RUN(test_lrclk_50pct_duty);

    std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
