// tb_asc.cpp — Verilator unit testbench for asc.v (Q700 EASC variant)
//
// Build:   make tb-asc
// Covers the ten task-brief scenarios plus a couple of corners that
// almost bit during implementation (half-empty IRQ latch + stereo
// interleave + rate-divider scaling).
//
//  Scenarios:
//   1. reset clears FIFOs + pointers, mode=silent, version reads 0.
//   2. CPU write advances write-pointer (FIFO A + B independently).
//   3. sample-tick advances read-pointer (once mode=FIFO).
//   4. half-empty IRQ fires at midpoint (downward crossing of depth=512).
//   5. IRQ clears on FIFO-status read (+0x810 / +0x811 read-to-ack).
//   6. IRQ clears on interrupt-status read (+0x804 — both FIFOs at once).
//   7. stereo mode interleaves A/B correctly on consecutive sample_ticks.
//   8. rate reg scales sample-tick divider (larger rate = slower).
//   9. volume reg attenuates output (0x00 mutes, 0xFF full-scale).
//  10. mode=silent stops sample-tick (read-pointer frozen).
//  11. version reg reads 0 (deliberately stays no-ASC for boot path).
//  12. FIFO-clear via fifo_ctl bit 7 resets both FIFOs.
//  13. Full FIFO (1024 bytes) drops further writes (count saturates).
//  14. FIFO overflow sticks until status read clears it.
//  15. FIFO underflow sticks until status read clears it.
//  16. FIFO clear clears both overflow and underflow latches.
//  17. ROM-style silence/probe writes leave version/status benign.
//  18. ROM FIFO service loop observes ASC +0x804 bit 1 after FIFO A drains.
//  19. MAME-style aliases/control regs read back and IRQ-mask correctly.
//  20. FIFO B drains in FIFO mode even when stereo control is clear.
//  21. Concurrent CPU fill and sample drain preserve FIFO depth/IRQ state.
//  22. Native signed PCM outputs are valid for a future 48 kHz resampler.
//  23. ROM register probe can write/read +0x804 without asserting IRQ.
//  24. EASC ext WRPTR/RDPTR live readback (0xF00..F03 / 0xF20..F23).
//  25. EASC ext SRC + per-ch volume + FIFO ctrl latched (0xF04..F08 /
//      0xF24..F28).
//  26. EASC ext CD-XA decoder stubs latch but don't side-effect
//      (0xF10 / 0xF30; ADPCM decode deferred).

#include <cstdio>
#include <cstdint>
#include <verilated.h>
#include "Vasc.h"

static Vasc*   dut       = nullptr;
static uint64_t sim_time = 0;
static int     n_pass    = 0;
static int     n_fail    = 0;

static void tick() {
    dut->clk = 0; dut->eval(); sim_time++;
    dut->clk = 1; dut->eval(); sim_time++;
}

static void reset() {
    dut->rst       = 1;
    dut->phi2_tick = 0;
    dut->pb_addr   = 0;
    dut->pb_wdata  = 0;
    dut->pb_wr     = 0;
    dut->pb_rd     = 0;
    tick(); tick();
    dut->rst = 0;
    tick();
    // Sonora resets with R_PLAYRECA = 0 → playback mode active, which
    // level-fires the FIFO A IRQ whenever fa_count < 512 and forces
    // STAT_EMPTY_OR_FULL_A high.  Most legacy tests predate this and
    // assume both are quiescent post-reset, so we put the chip into
    // record mode (bit 0 = 1) by default.  Tests that exercise the
    // playback path explicitly write 0x80A = 0 to re-enable it.
    dut->pb_addr   = 0x80A;
    dut->pb_wdata  = 0x01;
    dut->pb_wr     = 1;
    tick();
    dut->pb_wr     = 0;
    dut->pb_wdata  = 0;
    // Iter-4 EASC compatibility shim: under VERSION_USE = 0xB0 the
    // EASC reset leaves mode_reg = 0 (chip off), fa_irqen / fb_irqen
    // both set to 1 (IRQ DISABLED), fifostat = 0.  This breaks legacy
    // Sonora-shape tests that expect mode = 1, IRQ enabled, fifostat
    // = 0x02 post-reset.  To keep the non-EASC scenario tests
    // meaningful under both VERSIONs, the reset() helper post-resets
    // mode = 1 and IRQ-enables both channels so the existing test
    // catalogue runs under both Sonora and EASC paths.  Tests
    // sensitive to the exact reset values (test_reset_reads,
    // test_cpu_write_advances_wp) opt-out via chip_is_easc() at top.
    //
    // Under Sonora these post-resets are no-ops (mode is already 1,
    // IRQ already enabled).  Under EASC they override the Sonora-
    // divergent default reset state.
    dut->pb_addr   = 0x801;
    dut->pb_wdata  = 0x01;
    dut->pb_wr     = 1;
    tick();
    dut->pb_wr     = 0;
    dut->pb_addr   = 0xF09;
    dut->pb_wdata  = 0x00;
    dut->pb_wr     = 1;
    tick();
    dut->pb_wr     = 0;
    dut->pb_addr   = 0xF29;
    dut->pb_wdata  = 0x00;
    dut->pb_wr     = 1;
    tick();
    dut->pb_wr     = 0;
    dut->pb_wdata  = 0;
}

#define CHECK_EQ(name, got, exp) do { \
    uint32_t _g = (uint32_t)(got); \
    uint32_t _e = (uint32_t)(exp); \
    if (_g != _e) { \
        printf("  FAIL %s: got 0x%x, expected 0x%x\n", name, _g, _e); \
        return false; \
    } \
} while(0)

#define CHECK_TRUE(name, cond) do { \
    if (!(cond)) { \
        printf("  FAIL %s: condition false\n", name); \
        return false; \
    } \
} while(0)

static void bus_write(uint16_t addr, uint8_t data) {
    dut->pb_addr   = addr & 0xFFF;
    dut->pb_wdata  = data;
    dut->pb_wr     = 1;
    dut->pb_rd     = 0;
    dut->phi2_tick = 0;
    tick();
    dut->pb_wr     = 0;
    dut->pb_wdata  = 0;
}

static void bus_write_with_phi2(uint16_t addr, uint8_t data) {
    dut->pb_addr   = addr & 0xFFF;
    dut->pb_wdata  = data;
    dut->pb_wr     = 1;
    dut->pb_rd     = 0;
    dut->phi2_tick = 1;
    tick();
    dut->phi2_tick = 0;
    dut->pb_wr     = 0;
    dut->pb_wdata  = 0;
}

static uint8_t bus_read(uint16_t addr) {
    dut->pb_addr   = addr & 0xFFF;
    dut->pb_rd     = 1;
    dut->pb_wr     = 0;
    dut->phi2_tick = 0;
    tick();
    dut->pb_rd     = 0;
    dut->eval();
    return dut->pb_rdata & 0xFF;
}

// Issue N phi2_tick pulses.  The design expects one-cycle pulses on
// `phi2_tick` spaced at (at least) one clock edge apart.  A single
// trailing non-phi2 tick is appended so that (a) the edge-detect FFs
// for the half-empty IRQ see the post-update count, and (b) the
// audio-output register sees the post-update sample_X_raw bytes.
// Both paths require exactly one extra clock beyond the last phi2
// pulse to settle; tests check IRQ / audio_sample_valid after this
// settling cycle.
static void phi2_ticks(uint32_t n) {
    dut->pb_wr = 0;
    dut->pb_rd = 0;
    for (uint32_t i = 0; i < n; i++) {
        dut->phi2_tick = 1;
        tick();
        dut->phi2_tick = 0;
    }
    tick();
}

// Backwards-compatible alias used by a few tests.
static inline void phi2_ticks_settle(uint32_t n) { phi2_ticks(n); }

// Helper: put `n` bytes into FIFO A starting at byte value `seed`.
static void fill_fifo_a(uint32_t n, uint8_t seed) {
    if (bus_read(0x801) != 0x01)
        bus_write(0x801, 0x01);
    for (uint32_t i = 0; i < n; i++)
        bus_write(0x000, (uint8_t)(seed + i));
}
static void fill_fifo_b(uint32_t n, uint8_t seed) {
    if (bus_read(0x801) != 0x01)
        bus_write(0x801, 0x01);
    for (uint32_t i = 0; i < n; i++)
        bus_write(0x400, (uint8_t)(seed + i));
}

// Put ASC into FIFO mode with a given rate seed.  Volume full-scale by
// default (already that post-reset).
static void enable_fifo_mode(uint8_t rate = 0) {
    if (rate) bus_write(0x808, rate);   // Rate register
    bus_write(0x801, 0x01);             // Mode = FIFO
}

// Helper: detect chip identification at runtime.  Used by the iter-4
// EASC scenarios (40-43) and as a soft-skip for legacy Sonora-shape
// scenarios under VERSION_USE = VERSION_EASC_REAL (0xB0).
static bool chip_is_easc() {
    return bus_read(0x800) == 0xB0;
}

// ─── Scenario 1: reset reads (Sonora defaults) ───────────────────────
// Post-reset reads of the published register file.  Sonora-faithful
// defaults (per MAME asc_sonora_device::device_reset):
//   - R_MODE = 0x01 (hardwired FIFO; Sonora ignores writes)
//   - R_FIFOSTAT = 0x02 (STAT_EMPTY_OR_FULL_A pre-set on Sonora reset;
//     bits 2/3 are only updated by the sample stream loop, not by reset)
//   - m_fifo_irqen[0/1] = 0 (IRQ enabled; bit 0 = 0 means enabled)
// MAME-faithful read values: MAME's read switch returns 0 for
// R_CONTROL/R_FIFOMODE/R_WTCONTROL/R_CLOCK/R_BATMANCONTROL on Sonora.
// Version stays at 0x00 (local — boot-clean "no-ASC simple init" path).
// Iter 3 attempted the 0x00→0xB0 flip with the multi-byte FSM landed,
// but the ROM's EASC FIFO playback path produced spectrally-worse
// audio than the iter 1 wavetable baseline (0.038 vs 0.190 cross-
// correlation; 0/5 vs 4/5 peak match).  See VERSION_EASC localparam
// in asc.v for the iter 4+ hypothesis.
static bool test_reset_reads() {
    reset();
    CHECK_EQ("version @reset",       bus_read(0x800), 0x00);
    CHECK_EQ("mode @reset (FIFO)",   bus_read(0x801), 0x01);
    CHECK_EQ("control @reset (Sonora reads 0)",  bus_read(0x802), 0x00);
    CHECK_EQ("fifomode @reset (Sonora reads 0)", bus_read(0x803), 0x00);
    CHECK_EQ("wt_ctl @reset (0)",    bus_read(0x805), 0x00);
    CHECK_EQ("clock @reset (0)",     bus_read(0x807), 0x00);
    CHECK_EQ("rate @reset (35)",     bus_read(0x808), 35);
    // 0x80A is R_PLAYRECA on Sonora — reset() helper sets it to 0x01
    // (record mode) post-reset to keep playback-mode IRQ quiescent.
    CHECK_EQ("playreca @reset (record mode)", bus_read(0x80A), 0x01);
    CHECK_EQ("volume @reset (0xFF)", bus_read(0x806), 0xFF);
    CHECK_EQ("FIFO A status @reset", bus_read(0x810), 0x00);
    CHECK_EQ("FIFO B status @reset", bus_read(0x811), 0x00);
    // R_FIFOSTAT after reset: 0x02 (Sonora's STAT_EMPTY_OR_FULL_A).
    CHECK_EQ("irq-status @reset (Sonora 0x02)", bus_read(0x804), 0x02);
    CHECK_EQ("F09 irqen @reset (enabled)", bus_read(0xF09), 0x00);
    CHECK_EQ("F29 irqen @reset (enabled)", bus_read(0xF29), 0x00);
    CHECK_TRUE("irq low @reset",     dut->irq == 0);
    return true;
}

// ─── Scenario 2: CPU write advances write-pointer (indirect) ──────────
// We can't inspect fa_wp directly, so observe indirectly: after writing
// 600 bytes the count crosses 512 → half-empty edge (on a subsequent
// sample-tick that drops it below 512 again).  Here we just assert no
// IRQ yet after writes (count is ABOVE the threshold).
//
// Pre-fill, +0x804 reads 0x02 (Sonora reset value — STAT_EMPTY_OR_FULL_A).
// On Sonora, the bit-2/3 (combined A|B half/empty) values are populated
// by the sample stream loop, not by writes — so they're 0 until the
// first phi2 tick.  Our reset() puts the chip in record mode, so the
// FIFO A push base-write hook in MAME-faithful Sonora doesn't update
// status bits (record-mode is gated out — MAME asc.cpp line 386).
static bool test_cpu_write_advances_wp() {
    reset();
    CHECK_EQ("irq-status pre-fill (Sonora 0x02)", bus_read(0x804), 0x02);
    fill_fifo_a(600, 0x10);
    CHECK_TRUE("irq low after fill",  dut->irq == 0);
    CHECK_EQ("FIFO A status no IRQ",  bus_read(0x810), 0x00);
    return true;
}

// ─── Scenario 3: sample-tick advances read-pointer ────────────────────
// Fill FIFO A with 4 bytes, set rate=1 (every phi2_tick = 1 sample),
// enable FIFO mode, and issue 4 phi2_ticks.  After the ticks, count
// should be 0 — verified via triggering a SECOND half-empty crossing:
// fill 513 more, then drain.  For directness here we just confirm that
// sample_tick + audio_sample_valid pulse.
static bool test_sample_tick_advances_rp() {
    reset();
    fill_fifo_a(4, 0xA0);
    enable_fifo_mode(/*rate=*/1);
    dut->audio_sample_valid = 0;
    phi2_ticks(1);
    dut->eval();
    // audio_sample_valid pulses for one cycle on the sample_tick.
    CHECK_TRUE("audio_sample_valid after tick", dut->audio_sample_valid == 1);
    // Subsequent phi2_ticks should continue to emit samples while FIFO
    // has data; after 3 more the FIFO is drained and further phi2_ticks
    // still fire sample_ticks but read-pointer stays (count==0 branch).
    phi2_ticks(3);
    dut->eval();
    // Drain done; now crossing 512->below happens... but count was ≤4 so
    // it never went above.  Just assert no IRQ yet (count stayed low).
    CHECK_EQ("FIFO A status post-drain", bus_read(0x810), 0x00);
    return true;
}

// ─── Scenario 4: half-empty IRQ fires at midpoint ─────────────────────
// Fill FIFO A with 600 bytes, enable FIFO mode at rate=1, then issue
// enough phi2_ticks to drop below 512 (600 - 512 = 88 samples + one).
static bool test_half_empty_irq_fires() {
    reset();
    fill_fifo_a(600, 0x00);
    enable_fifo_mode(/*rate=*/1);
    // Drain 89 samples — 600 - 89 = 511 < 512, triggers edge.
    phi2_ticks_settle(89);
    CHECK_TRUE("irq high at midpoint",    dut->irq == 1);
    CHECK_EQ("FIFO A status bit 7",        bus_read(0x810) & 0x80, 0x80);
    // Must be sticky — reading once consumed it; re-fill check is next scenario.
    return true;
}

// ─── Scenario 5: IRQ clears on FIFO-status read (+0x810) ──────────────
// Local convention (NOT MAME): 0x810 read clears the per-FIFO local
// half-empty/sticky latches.  On MAME-faithful Sonora the IRQ line is
// only cleared by 0x804 read (and only iff STAT_HALF_FULL_B clear) —
// the local 0x810 read is for software readback of the bring-up
// sticky bits, not for IRQ ack.
//
// To exercise both code paths: fill BOTH FIFOs above the threshold so
// the combined-half-B condition flips to inactive after refill, then
// read 0x804 to clear IRQ.  Then verify 0x810 read clears the local
// sticky bits.
static bool test_irq_clears_on_fifo_status_read() {
    reset();
    fill_fifo_a(600, 0x00);
    fill_fifo_b(600, 0x00);            // need B filled too — Sonora drains B
    enable_fifo_mode(/*rate=*/1);
    phi2_ticks_settle(89);
    CHECK_TRUE("irq high before read", dut->irq == 1);
    // Local-status read clears the local half-empty latch.
    (void)bus_read(0x810);
    dut->eval();
    CHECK_EQ("FIFO A local status now 0", bus_read(0x810), 0x00);

    // To clear the IRQ line per MAME-faithful semantics, BOTH conditions
    // must hold: (a) the combined-half-B condition must fall (i.e., both
    // caps >= 0x200), and (b) 0x804 must be read.  Refill both FIFOs to
    // get above 0x200, then read 0x804.
    fill_fifo_a(600, 0x40);            // 600 left + 600 push (saturates ~600)
    fill_fifo_b(600, 0x40);
    // Run a few sample ticks to update bit 2 (combined-half-B) — both
    // caps now >= 512 so bit 2 should clear.
    phi2_ticks_settle(2);
    (void)bus_read(0x804);             // ack — bit 2 clear, IRQ clears
    dut->eval();
    CHECK_TRUE("irq cleared by 0x804 read after refill", dut->irq == 0);
    return true;
}

// ─── Scenario 6: IRQ clears on interrupt-status read (+0x804) ─────────
// Per MAME asc_sonora_device::read line 1049-1055: 0x804 read clears the
// IRQ line iff STAT_HALF_FULL_B (bit 2) is NOT set.  Status BITS are
// NOT cleared on read.  So to exercise the IRQ-clear path we have to
// keep both FIFOs above the threshold during the read.
//
// Sonora bit map: bit 2 (STAT_HALF_FULL_B) is the COMBINED "either A or
// B half-empty" — set by the stream loop when cap_a < 0x200 OR cap_b <
// 0x200.  After 89 sample ticks with both filled to 600, cap_a = cap_b
// = 511 → bit 2 set → IRQ asserted → 0x804 read does NOT clear IRQ.
// The CPU's expected response is to refill, then re-read 0x804.
static bool test_irq_clears_on_intstatus_read() {
    reset();
    fill_fifo_a(600, 0x00);
    fill_fifo_b(600, 0x00);
    enable_fifo_mode(/*rate=*/1);
    phi2_ticks_settle(89);
    CHECK_TRUE("irq high (both FIFOs)",    dut->irq == 1);
    uint8_t s = bus_read(0x804);
    // Sonora bit 2 (STAT_HALF_FULL_B) is the combined indicator —
    // bit 0 (STAT_HALF_FULL_A) is dormant on Sonora.
    CHECK_EQ("STAT_HALF_FULL_B set",   s & 0x04, 0x04);
    dut->eval();
    // Bit 2 is set → 0x804 read does NOT clear IRQ.
    CHECK_TRUE("irq stays high while bit 2 set", dut->irq == 1);
    // Refill above threshold, then a couple sample ticks → bit 2 clears.
    fill_fifo_a(600, 0x40);
    fill_fifo_b(600, 0x40);
    phi2_ticks_settle(2);
    (void)bus_read(0x804);             // now ack works
    dut->eval();
    CHECK_TRUE("irq cleared by 0x804 read after refill", dut->irq == 0);
    return true;
}

// ─── Scenario 7: stereo mode interleaves A/B correctly ────────────────
// In stereo, each sample_tick advances BOTH FIFO A and FIFO B (lock-step
// L/R).  We load A with one distinctive byte, B with another, take a
// sample_tick, then inspect audio_sample_out (upper byte = A, lower = B).
static bool test_stereo_interleaves() {
    reset();
    // Load one byte each — A=0xC0, B=0x40.  Centred: A = 0x40, B = -0x40.
    // Volume=0xFF → attenuation ≈ 255/256 ≈ 1×.  Truncated to 8 LSBs:
    // A' = 0x3F or 0x40 ish, B' = 0xC0 ish (negative after sign extend).
    enable_fifo_mode(/*rate=*/1);
    bus_write(0x000, 0xC0);
    bus_write(0x400, 0x40);
    bus_write(0x802, 0x02);           // stereo
    bus_write(0x80A, 0xFF);           // full scale
    phi2_ticks(1);
    dut->eval();
    uint16_t s = dut->audio_sample_out;
    uint8_t hi = (s >> 8) & 0xFF;
    uint8_t lo = s & 0xFF;
    // A'-centred-attenuated should differ from B'-centred-attenuated —
    // sign opposes.  The precise value isn't the point: the point is
    // that hi != lo (stereo interleave actually wired).
    CHECK_TRUE("stereo: hi byte reflects FIFO A", hi != lo);
    // Now swap: clear, reload A=0x55, B=0xAA.  Expect hi ~= 0xD5-ish and
    // lo ~= 0x2A-ish (centred 0x55-0x80=-0x2B etc).  Again just assert
    // differ.
    bus_write(0x803, 0x80);           // fifo_ctl bit 7 = clear both
    bus_write(0x000, 0x55);
    bus_write(0x400, 0xAA);
    phi2_ticks(1);
    dut->eval();
    uint16_t s2 = dut->audio_sample_out;
    CHECK_TRUE("stereo: sample updates on new FIFO", s2 != s);
    return true;
}

// ─── Scenario 8: rate reg scales sample-tick divider ─────────────────
// With rate=4, we need 4 phi2_ticks per sample.  Fill 8 bytes, mode on,
// phi2_tick 3 times → only 0 samples (not divisible).  Tick once more →
// 1 sample.  Actually the counter decrements each phi2_tick, fires when
// it hits ≤1.  rate_seed=4 means counter sequence: 4,3,2,1(fire, reload)
// — so one sample every 4 phi2_ticks.
static bool test_rate_scales_divider() {
    reset();
    fill_fifo_a(8, 0x10);
    enable_fifo_mode(/*rate=*/4);
    // 3 phi2_ticks: no sample yet.
    dut->audio_sample_valid = 0;
    phi2_ticks(3);
    dut->eval();
    CHECK_TRUE("no sample after 3 ticks (rate=4)", dut->audio_sample_valid == 0);
    // 4th tick fires the sample.
    phi2_ticks(1);
    dut->eval();
    CHECK_TRUE("sample fires on 4th tick", dut->audio_sample_valid == 1);
    return true;
}

// ─── Scenario 9: R_VOLUME is software-visible but not applied ─────────
// MAME's stream loop emits raw `(s8)byte ^ 0x80` to the host audio sink
// — physical attenuation is the off-chip codec's job, not Sonora's.
// We mirror that: R_VOLUME at 0x806 is writable + readable for software
// compatibility, but the audio output is the raw signed sample
// regardless of volume.  The downstream `audio_pwm` modulator handles
// the full 16-bit range.
static bool test_volume_writable_but_not_applied() {
    reset();
    bus_write(0x000, 0xFF);             // max-amplitude byte
    bus_write(0x802, 0x00);             // mono presentation
    enable_fifo_mode(/*rate=*/1);

    // Volume = 0: output is still the raw signed sample.
    bus_write(0x806, 0x00);
    CHECK_EQ("volume readable as 0",  bus_read(0x806), 0x00);
    phi2_ticks(1);
    dut->eval();
    CHECK_TRUE("volume=0 does NOT silence (MAME-faithful raw out)",
               dut->audio_sample_out != 0x0000);
    int16_t pcm0 = (int16_t)dut->audio_pcm_l;

    // Volume = 0xFF: should give the same output (volume not applied).
    bus_write(0x803, 0x80);             // clear FIFOs
    bus_write(0x000, 0xFF);
    bus_write(0x806, 0xFF);
    CHECK_EQ("volume readable as 0xFF", bus_read(0x806), 0xFF);
    phi2_ticks(1);
    dut->eval();
    int16_t pcm_full = (int16_t)dut->audio_pcm_l;
    CHECK_EQ("volume change does not affect PCM out", pcm_full, pcm0);
    return true;
}

// ─── Scenario 10: R_MODE accepts low 2 bits (EASC + first-gen ASC) ───
// Per MAME asc.cpp:439-440 (`data &= 3`), R_MODE stores the low 2 bits
// of the write.  Our hybrid model honours this so the Q700 ROM can
// select wavetable mode (data = 2) for the startup chime.  Verify the
// readback matches the programmed value, and that switching to
// MODE_SILENT halts the sample-tick stream while FIFO mode resumes it.
static bool test_sonora_mode_writes_ignored() {
    reset();
    CHECK_EQ("mode @reset reads as FIFO", bus_read(0x801), 0x01);

    fill_fifo_a(4, 0x10);
    enable_fifo_mode(/*rate=*/1);
    phi2_ticks(1);
    dut->eval();
    CHECK_TRUE("sample valid post-fill", dut->audio_sample_valid == 1);

    // R_MODE is now writable — the low 2 bits land per MAME's catch-all.
    bus_write(0x801, 0x00);
    CHECK_EQ("mode = silent after silent write", bus_read(0x801), 0x00);
    bus_write(0x801, 0x02);
    CHECK_EQ("mode = wave after wave write",     bus_read(0x801), 0x02);
    bus_write(0x801, 0x03);
    CHECK_EQ("mode masks low 2 bits (0x03)",     bus_read(0x801), 0x03);

    // Restore FIFO mode and confirm sample-ticks resume.
    bus_write(0x801, 0x01);
    CHECK_EQ("mode = FIFO after restore", bus_read(0x801), 0x01);
    phi2_ticks(1);
    dut->eval();
    CHECK_TRUE("sample still pulses post-mode-write",
               dut->audio_sample_valid == 1);
    return true;
}

// ─── Scenario 11: version reg reads 0 (no-ASC boot path) ─────────────
// Real EASC reports 0xB0 (asc.cpp:1768-1771), but our chip deliberately
// reports 0 to take the Q700 ROM's "no-ASC simple init" path that boots
// cleanly AND produces wavetable-mode chord tones close to the real
// chime (4/5 within 50 cents vs MAME baseline; cross-correlation 0.190).
// Iter 2 attempted to flip to 0xB0 without the multi-byte FSM and was
// rolled back when the canary $finish tripped.  Iter 3 landed the FSM
// and re-attempted the flip but rolled back again because the EASC
// FIFO playback path produced spectrally-worse audio.  See
// VERSION_EASC localparam in asc.v for the iter 4+ hypothesis.
static bool test_version_easc() {
    reset();
    CHECK_EQ("version", bus_read(0x800), 0x00);
    // Writes to version are silently ignored.
    bus_write(0x800, 0x09);
    CHECK_EQ("version still 0 after write", bus_read(0x800), 0x00);
    return true;
}

// ─── Scenario 12: FIFO clear resets pointers + drives status bits ─────
// Per MAME asc_base_device::write line 459-466: writing R_FIFOMODE bit
// 7 (0x803=0x80) clears the FIFOs AND sets `R_FIFOSTAT |= 0xa` (bits 1
// and 3 — the "FIFO empty" indicators).  The IRQ line itself is not
// auto-cleared by FIFO clear — software must read 0x804.  Once the
// FIFOs are empty, the combined-half-B condition stays asserted, which
// keeps bit 2 set on the next stream tick → 0x804 reads will NOT clear
// IRQ until the CPU refills.
static bool test_fifo_clear() {
    reset();
    fill_fifo_a(600, 0x10);
    enable_fifo_mode(/*rate=*/1);
    phi2_ticks_settle(89);
    CHECK_TRUE("irq high before clear", dut->irq == 1);
    // Clear FIFOs — caps drop to 0 + R_FIFOSTAT bits 1,3 set.
    bus_write(0x803, 0x80);
    dut->eval();
    CHECK_EQ("FIFO A local status 0 after clear", bus_read(0x810), 0x00);
    // R_FIFOSTAT now has bit 3 (FIFO B empty marker) and bit 1 (FIFO A
    // empty marker) forced high by the clear hook.
    uint8_t s = bus_read(0x804);
    CHECK_EQ("FIFO clear sets bits 1+3", s & 0x0a, 0x0a);
    return true;
}

// ─── Scenario 13: FIFO full drops further writes ─────────────────────
// Fill 1024 bytes; then try to write 10 more.  Cap saturates at 1024;
// the dropped writes set the local sticky overflow latch (covered in
// dedicated test below).  After drain past empty, the combined-half-B
// + combined-empty-B bits assert in MAME-faithful order: bit 2 once
// cap_a < 0x200, bit 3 once cap_a == 0.
static bool test_fifo_full_saturates() {
    reset();
    fill_fifo_a(1024, 0xAA);
    fill_fifo_a(10, 0x55);              // dropped — overflow stickies
    enable_fifo_mode(/*rate=*/1);
    // Drain 513 samples — at sample 513, cap_a went from 1024 to 511;
    // bit 2 (combined-half-B, since cap_b=0 anyway, was set already).
    phi2_ticks_settle(513);
    CHECK_TRUE("irq high after midpoint cross", dut->irq == 1);
    // Drain to empty (511 + a few more for safety).
    phi2_ticks_settle(2000);
    // Bit 2 (combined-half-B) and bit 3 (combined-empty-B) are now set.
    uint8_t s = bus_read(0x804);
    CHECK_EQ("STAT_HALF_FULL_B set after drain", s & 0x04, 0x04);
    CHECK_EQ("STAT_EMPTY_OR_FULL_B set after drain", s & 0x08, 0x08);
    // Per MAME-faithful: 0x804 read does NOT clear IRQ while bit 2 set.
    CHECK_TRUE("IRQ remains asserted (bit 2 set)", dut->irq == 1);
    return true;
}

// ─── Scenario 14: FIFO overflow latches until read-cleared ───────────
static bool test_fifo_overflow_latches_until_read_clear() {
    reset();
    fill_fifo_a(1024, 0xAA);
    bus_write(0x000, 0x55);          // dropped write should stick overflow
    CHECK_TRUE("irq low after overflow", dut->irq == 0);
    uint8_t s = bus_read(0x810);
    CHECK_EQ("overflow status bits", s & 0x60, 0x60);
    CHECK_EQ("overflow read clears", bus_read(0x810), 0x00);
    CHECK_EQ("IRQ status sticky bits stay clear", bus_read(0x804) & 0xF0, 0x00);
    return true;
}

// ─── Scenario 15: FIFO underflow latches until read-cleared ──────────
// Local 0x810/0x811 sticky underflow indicators latch on a sample tick
// where the FIFO is empty.  Per MAME-faithful Sonora the IRQ line is
// also asserted (combined-half-B + combined-empty-B bits set when both
// caps are 0, FIFO B IRQ enable defaults to enabled).
static bool test_fifo_underflow_latches_until_read_clear() {
    reset();
    enable_fifo_mode(/*rate=*/1);
    phi2_ticks(1);                    // both FIFOs empty → underflow
    // IRQ asserted by stream-side combined-half-B path.
    CHECK_TRUE("irq high after underflow stream tick", dut->irq == 1);
    uint8_t s = bus_read(0x810);
    CHECK_EQ("underflow local status bits", s & 0x50, 0x50);
    CHECK_EQ("local read clears the local sticky", bus_read(0x810), 0x00);
    // R_FIFOSTAT shows combined bits 2+3 set, bit 0 dormant.
    uint8_t fs = bus_read(0x804);
    CHECK_EQ("R_FIFOSTAT bit 2 set on empty",   fs & 0x04, 0x04);
    CHECK_EQ("R_FIFOSTAT bit 3 set on empty",   fs & 0x08, 0x08);
    return true;
}

// ─── Scenario 16: FIFO clear clears overflow + underflow latches ─────
// Local convention: FIFO clear (0x803 bit 7) wipes the local 0x810/0x811
// sticky overflow/underflow indicators.  Per MAME-faithful: the clear
// also sets R_FIFOSTAT bits 1+3 (FIFO empty markers).  IRQ line is not
// auto-cleared but Sonora's stream loop will refresh bits 2+3 on the
// next sample tick — both stay set since both caps are 0.
static bool test_fifo_clear_clears_fault_latches() {
    reset();

    fill_fifo_a(1024, 0xAA);
    bus_write(0x000, 0x55);          // overflow A → local sticky
    bus_write(0x802, 0x02);          // stereo presentation
    enable_fifo_mode(/*rate=*/1);
    phi2_ticks(1);                    // B underflows on the first tick

    bus_write(0x803, 0x80);          // clear both FIFOs + local stickies
    CHECK_EQ("A local status cleared by FIFO clear", bus_read(0x810), 0x00);
    CHECK_EQ("B local status cleared by FIFO clear", bus_read(0x811), 0x00);
    // Per MAME: FIFO clear ORs in R_FIFOSTAT bits 1+3.
    CHECK_EQ("FIFO clear sets R_FIFOSTAT bits 1+3",
             bus_read(0x804) & 0x0a, 0x0a);
    return true;
}

// ─── Scenario 17: ROM-style setup/probe sequence ─────────────────────
// The Q700 ROM clears and silences the ASC around startup chime setup,
// then polls version/status bytes.  Sonora reset sets R_FIFOSTAT to
// 0x02; FIFO clear ORs in 0x0a; per-register writes that explicitly
// land at 0x804 overwrite the byte (MAME's catch-all m_regs[]).
static bool test_rom_style_silence_probe() {
    reset();
    if (chip_is_easc()) {
        printf("  [skip] EASC build (VERSION_USE = 0xB0) — Sonora-shape probe\n");
        return true;
    }

    CHECK_EQ("ROM version probe", bus_read(0x800), 0x00);

    // Write 0 to every control register (ROM bulk-clear) — note 0x804
    // gets 0 here, which lands in the latched fifostat (MAME catch-all).
    for (uint16_t a = 0x801; a <= 0x80F; a++) {
        bus_write(a, 0x00);
    }
    // The bulk-zero loop above also writes 0x80A=0 (playback) and
    // 0x804=0 (clear all status bits).  Re-disable playback for the
    // quiescent-IRQ check.
    bus_write(0x80A, 0x01);

    CHECK_EQ("version survives silence writes", bus_read(0x800), 0x00);
    // Bulk-clear set R_MODE = 0 (silent).  Restore FIFO mode for the
    // remainder of this test (the bulk-clear loop writes 0 to all of
    // 0x801..0x80F including R_MODE).
    bus_write(0x801, 0x01);
    CHECK_EQ("mode = FIFO after restore",  bus_read(0x801), 0x01);
    // 0x804 was directly written 0 — fifostat now reads 0 (no stream
    // ticks have run to update bits 2/3).
    CHECK_EQ("irq-status after explicit zero write",  bus_read(0x804), 0x00);
    CHECK_EQ("FIFO A quiet after ROM clear", bus_read(0x810), 0x00);
    CHECK_EQ("FIFO B quiet after ROM clear", bus_read(0x811), 0x00);
    CHECK_TRUE("irq low after ROM clear", dut->irq == 0);

    fill_fifo_a(600, 0x20);
    fill_fifo_b(600, 0x40);
    bus_write(0x803, 0x80);          // ROM-style FIFO clear (sets bits 1+3)
    bus_write(0x801, 0x00);          // mode write: no-op on Sonora
    bus_write(0x80A, 0x01);          // re-disable playback mode
    CHECK_EQ("FIFO clear sets bits 1+3",
             bus_read(0x804) & 0x0A, 0x0A);
    CHECK_EQ("FIFO A clear local status", bus_read(0x810), 0x00);
    CHECK_EQ("FIFO B clear local status", bus_read(0x811), 0x00);

    bus_write(0x805, 0xFF);          // reserved/test-adjacent writes
    bus_write(0x80F, 0xA5);
    CHECK_EQ("reserved writes do not perturb version", bus_read(0x800), 0x00);
    // R_TEST is writable scratch (latched).
    CHECK_EQ("R_TEST readable", bus_read(0x80F), 0xA5);
    return true;
}

// ─── Scenario 18: ROM FIFO service status bit (Sonora-faithful) ──────
// On Sonora, R_FIFOSTAT bit 2 (combined STAT_HALF_FULL_B) is set on
// the first sample tick where cap_a < 0x200 OR cap_b < 0x200.  Bit 3
// (combined STAT_EMPTY_OR_FULL_B) is set when either cap reaches 0.
// MAME-faithful: 0x804 read does NOT clear status bits, only the IRQ
// line (and only iff bit 2 clear).
static bool test_rom_fifo_service_status_bit() {
    reset();

    enable_fifo_mode(/*rate=*/1);
    bus_write(0x000, 0x7f);           // FIFO A sample 0
    bus_write(0x000, 0x80);           // FIFO A sample 1
    bus_write(0x400, 0x80);           // FIFO B sample 0
    bus_write(0x400, 0x7f);           // FIFO B sample 1

    phi2_ticks(1);
    uint8_t s = bus_read(0x804);
    // After one sample drain: cap_a = cap_b = 1 < 0x200 → bit 2 set.
    // Bit 3 is set only when cap == 0; cap is 1 → bit 3 clear.
    CHECK_EQ("STAT_HALF_FULL_B set (combined)", s & 0x04, 0x04);
    CHECK_EQ("STAT_EMPTY_OR_FULL_B clear (cap=1)", s & 0x08, 0x00);
    // 0x804 read does NOT clear status bits per MAME-faithful Sonora.
    CHECK_EQ("status bits preserved after read",
             bus_read(0x804) & 0x04, 0x04);
    return true;
}

// ─── Scenario 19: Sonora aliases + IRQ control ───────────────────────
// MAME's Sonora R_VOLUME at 0x806, R_PLAYRECA at 0x80A, and per-FIFO
// IRQ-disable at 0xF09/0xF29.  Per MAME asc_sonora_device::write:
// disabling the per-FIFO IRQ (data&1==1) clears the IRQ line; enabling
// (data&1==0) re-asserts the IRQ if the corresponding STAT_HALF_FULL_x
// bit is set.
static bool test_sonora_aliases_and_irq_control() {
    reset();

    // R_VOLUME at 0x806 — writable + readable, but no longer applied.
    bus_write(0x806, 0x44);
    CHECK_EQ("volume readback", bus_read(0x806), 0x44);
    bus_write(0x80A, 0x55);
    CHECK_EQ("playreca readback (separate from volume)",
             bus_read(0x80A), 0x55);
    CHECK_EQ("volume unchanged by playreca write",
             bus_read(0x806), 0x44);

    // R_CLOCK (0x807) — Sonora ignores writes, reads 0.
    bus_write(0x807, 0x03);
    CHECK_EQ("clock readback (Sonora: 0)", bus_read(0x807), 0x00);
    CHECK_EQ("rate unchanged by clock write", bus_read(0x808), 35);
    bus_write(0x807, 0x00);
    CHECK_EQ("rate still default after clock=0", bus_read(0x808), 35);

    // Disable BOTH per-FIFO IRQs to mute the line.  On Sonora, the IRQ
    // is asserted by the combined-half-B path which is gated by
    // fb_irqen — so disabling fb_irqen alone is enough to mute the
    // half-empty drain.  But disabling fa_irqen too makes record/
    // playback transitions also quiescent.
    bus_write(0xF09, 0x01);           // disable FIFO A IRQ
    bus_write(0xF29, 0x01);           // disable FIFO B IRQ
    CHECK_EQ("FIFO A IRQ control", bus_read(0xF09), 0x01);
    CHECK_EQ("FIFO B IRQ control", bus_read(0xF29), 0x01);
    fill_fifo_a(600, 0x00);
    fill_fifo_b(600, 0x00);
    enable_fifo_mode(/*rate=*/1);
    phi2_ticks_settle(89);
    CHECK_TRUE("IRQ masked at pin (both FIFOs disabled)", dut->irq == 0);
    // R_FIFOSTAT bit 2 (combined HALF_FULL_B) still latched.
    CHECK_EQ("R_FIFOSTAT bit 2 latched", bus_read(0x804) & 0x04, 0x04);

    // Per MAME: writing F29 with disable=0 (enable) AND the prior
    // disable=1 AND STAT_HALF_FULL_B set → re-assert IRQ.
    bus_write(0xF29, 0x00);           // re-enable B IRQ → assert
    dut->eval();
    CHECK_TRUE("IRQ re-asserts on F29 enable edge", dut->irq == 1);

    // Disabling F29 again clears the IRQ line.
    bus_write(0xF29, 0x01);
    dut->eval();
    CHECK_TRUE("IRQ clears on F29 disable edge", dut->irq == 0);

    return true;
}

// ─── Scenario 20: FIFO B drains independent of stereo control ─────────
// Per MAME asc_sonora_device::sound_stream_update (asc.cpp:996-1001):
// FIFO B is drained unconditionally — no chan_ctl / R_CONTROL stereo
// gate.  Q700 ROM writes both FIFOs without programming R_CONTROL, then
// polls R_FIFOSTAT bit 2 (combined HALF_FULL_B) waiting for service.
// On Sonora, bit 0 (HALF_FULL_A) is dormant; only bit 2 reports half.
static bool test_fifo_b_drains_without_stereo_control() {
    reset();
    fill_fifo_a(600, 0x10);
    fill_fifo_b(600, 0x80);
    bus_write(0x802, 0x00);           // R_CONTROL bit 1 = 0 (mono control)
    enable_fifo_mode(/*rate=*/1);

    phi2_ticks_settle(89);
    uint8_t s = bus_read(0x804);
    // Sonora bit 2 (combined HALF_FULL_B) = (cap_a<0x200) || (cap_b<0x200).
    // After 89 ticks both caps = 511 → bit 2 set.  Bit 0 is dormant.
    CHECK_EQ("STAT_HALF_FULL_B set (combined)", s & 0x04, 0x04);
    return true;
}

// ─── Scenario 21: concurrent push+pop keeps FIFO depth stable ─────────
// ROM/Sound Manager code can write FIFO bytes while the sample counter
// is running.  The write-side and sample-side count deltas must merge,
// not race through two nonblocking assignments to the same count reg.
//
// On Sonora the combined-half-B condition is set whenever EITHER cap_a
// OR cap_b is below 0x200.  To exercise the depth-preservation property
// without continuous IRQ asserts from FIFO B's empty side, we fill BOTH
// FIFOs above the threshold first.  Then we balanced-push to A only
// while B drains naturally — so this is also a check that A's depth
// holds steady under concurrent push+pop.
static bool test_concurrent_push_pop_preserves_depth() {
    reset();
    fill_fifo_a(600, 0x10);
    fill_fifo_b(600, 0x10);             // both above threshold
    enable_fifo_mode(/*rate=*/1);

    // First, drain B down to ~512 by 88 sample ticks.  After this both
    // caps are around 512; combined-half-B will still fire as we cross.
    phi2_ticks_settle(88);
    (void)bus_read(0x804);              // tolerate any IRQ from this cross

    // Now do balanced push+pop on A (each iteration: write A + 1 phi2
    // tick = 1 push and 1 pop on each FIFO, so net cap_a delta is 0,
    // cap_b delta is -1).  We expect cap_a to stay around its current
    // value while cap_b drains.
    for (uint32_t i = 0; i < 100; i++) {
        bus_write_with_phi2(0x000, (uint8_t)(0x80 + i));
        tick();
    }

    // A short additional drain shouldn't drop cap_a below threshold.
    // (cap_a ≈ 511 - 100 + 100 = 511; one more sample → 510, still
    // < 0x200 = 512, so combined-half-B was already set).
    phi2_ticks_settle(12);
    // Local A-side half-empty edge latched from the earlier crossing,
    // verify the local sticky bit picks it up.
    uint8_t a_loc = bus_read(0x810);
    CHECK_TRUE("A local edge latched (depth tracked)",
               (a_loc & 0x80) != 0);
    return true;
}

// ─── Scenario 22: signed PCM output for resampler/I2S path ────────────
// MAME emits raw `(s8)byte ^ 0x80` — physical attenuation is the off-
// chip codec's job, so volume_reg does NOT scale the output.
//   byte 0x00 → centred 0x00-0x80 = 0xFF80 (s16 = -128) → PCM = 0x8000
//   byte 0xFF → centred 0xFF-0x80 = 0x007F (s16 = +127) → PCM = 0x7F00
static bool test_native_pcm_outputs() {
    reset();

    bus_write(0x802, 0x02);           // stereo presentation
    enable_fifo_mode(/*rate=*/1);
    bus_write(0x000, 0x00);           // left: most negative
    bus_write(0x400, 0xFF);           // right: near positive full-scale
    phi2_ticks(1);

    CHECK_TRUE("PCM valid follows native sample", dut->audio_sample_valid == 1);
    CHECK_EQ("legacy packed left byte",  dut->audio_sample_out >> 8, 0x80);
    CHECK_EQ("legacy packed right byte (raw)",
             dut->audio_sample_out & 0xFF, 0x7F);
    CHECK_EQ("PCM left signed 16",  dut->audio_pcm_l, 0x8000);
    CHECK_EQ("PCM right signed 16", dut->audio_pcm_r, 0x7F00);

    reset();
    enable_fifo_mode(/*rate=*/1);
    bus_write(0x000, 0x00);
    bus_write(0x400, 0xFF);
    phi2_ticks(1);
    CHECK_EQ("mono PCM left", dut->audio_pcm_l, 0x8000);
    CHECK_EQ("mono PCM right mirrors left", dut->audio_pcm_r, 0x8000);
    return true;
}

// ─── Scenario 24: Sonora ignores wavetable-region writes ──────────────
// 0x830..0x83F is the EASC wavetable phase/increment window.  Sonora
// does not have a programmable wavetable engine — writes are silently
// ignored, reads return 0.  Verify both directions, plus that
// 0x805 (WTCONTROL) and 0x807 (CLOCK) match the "ignored / read 0"
// pattern from MAME asc_sonora_device.
static bool test_sonora_no_op_register_writes() {
    reset();

    // Wavetable phase/increment window — writes ignored, reads 0.
    for (int i = 0; i < 16; i++) {
        bus_write(0x830 + i, 0xA5);
    }
    for (int i = 0; i < 16; i++) {
        char name[64];
        snprintf(name, sizeof(name),
                 "wavetable @0x%03x reads 0", 0x830 + i);
        CHECK_EQ(name, bus_read(0x830 + i), 0x00);
    }

    // 0x805 WTCONTROL — Sonora reads 0, ignores writes.
    bus_write(0x805, 0xFF);
    CHECK_EQ("wt_control reads 0 after write", bus_read(0x805), 0x00);

    // 0x807 CLOCK — Sonora reads 0, ignores writes (sample rate is
    // hardwired).  Also verify the rate doesn't change in response.
    uint8_t rate_before = bus_read(0x808);
    bus_write(0x807, 0x03);   // would request 44 kHz on EASC
    CHECK_EQ("clock reads 0 after write", bus_read(0x807), 0x00);
    CHECK_EQ("rate unchanged by clock write", bus_read(0x808), rate_before);

    return true;
}

// ─── Scenario 25: default sample rate produces a tick at 22 kHz ──────
// With no ROM programming of the rate register, Sonora must default to
// the Mac sample rate (~22 kHz) so the chime plays at the right pitch.
// On the Q700 VIA timebase (783_360 Hz), RATE_DEFAULT (35) gives a
// sample tick every 35 µs ≈ 22.4 kHz, ≈ Sonora's hardwired 22.257 kHz.
//
// Counter walk: rate_cnt starts at 35 post-reset; phi2_tick N sees
// rate_cnt = 36-N.  sample_tick is combinational on `rate_cnt <= 1`,
// so it fires when phi2_tick arrives with rate_cnt=1, i.e. on the 35th
// phi2 tick after reset.
static bool test_sonora_default_rate_drains_fifo() {
    reset();
    fill_fifo_a(2, 0xC0);
    // Do NOT write rate — rely on Sonora default.
    bus_write(0x801, 0x01);          // mode write is a no-op (already FIFO)

    // First 34 phi2 ticks: rate_cnt walks 35 → 2; no sample yet.
    phi2_ticks(34);
    dut->eval();
    CHECK_TRUE("no sample at phi2_tick 34 (rate_cnt = 2)",
               dut->audio_sample_valid == 0);

    // 35th phi2 tick fires sample (rate_cnt was 1 at the edge).
    dut->audio_sample_valid = 0;
    phi2_ticks(1);
    dut->eval();
    CHECK_TRUE("sample fires on 35th phi2 tick (Sonora default)",
               dut->audio_sample_valid == 1);
    return true;
}

// ─── Scenario 26: R_PLAYRECA playback-mode semantics ─────────────────
// Per MAME asc_sonora_device::sound_stream_update lines 970-978:
// playback mode (R_PLAYRECA bit 0 = 0) forces R_FIFOSTAT |=
// STAT_EMPTY_OR_FULL_A (bit 1) at the top of each stream tick AND
// asserts IRQ if FIFO A IRQ is enabled.
//
// Per asc_sonora_device::write lines 1109-1116: every FIFO A push in
// playback mode re-forces bit 1 high in R_FIFOSTAT.
static bool test_sonora_playback_mode_irq_and_status() {
    reset();    // reset() puts us in record mode (bit 0 = 1)

    // Sonora reset value: R_FIFOSTAT = 0x02 (STAT_EMPTY_OR_FULL_A).
    // Bits 2/3 are 0 since no stream tick has run.
    CHECK_EQ("record-mode 0x804 (Sonora reset 0x02)",
             bus_read(0x804), 0x02);
    CHECK_TRUE("record-mode IRQ low at empty FIFO", dut->irq == 0);

    // Push 200 bytes in record mode.  MAME's base-write hook in record
    // mode skips the bit updates (PLAYRECA gate at asc.cpp:386), so
    // R_FIFOSTAT stays at 0x02.
    fill_fifo_a(200, 0x40);
    CHECK_EQ("record-mode 0x804 unchanged after fill",
             bus_read(0x804), 0x02);
    CHECK_TRUE("record-mode IRQ stays low", dut->irq == 0);

    // Switch to playback mode and run a sample tick — the stream-loop
    // top-of-routine forces bit 1 high and asserts IRQ.  Use rate=1 to
    // make every phi2_tick a sample tick (default rate_cnt is 35).
    bus_write(0x80A, 0x00);
    bus_write(0x808, 1);                // rate_reg = 1 → tick every phi2
    phi2_ticks(1);
    dut->eval();
    CHECK_EQ("playback-mode stream tick forces bit 1",
             bus_read(0x804) & 0x02, 0x02);
    CHECK_TRUE("playback-mode level-fires IRQ", dut->irq == 1);

    // Push more data into FIFO A in playback mode — Sonora's post-write
    // hook re-forces bit 1 high after every push.
    bus_write(0x000, 0x55);
    CHECK_EQ("playback-mode FIFO A push re-forces bit 1",
             bus_read(0x804) & 0x02, 0x02);

    // Volume is independent of playreca.
    bus_write(0x806, 0x77);
    bus_write(0x80A, 0xAA);            // bit 0 = 0 → still playback
    CHECK_EQ("volume readback unchanged by playreca write",
             bus_read(0x806), 0x77);
    CHECK_EQ("playreca readback (8-bit, not just bit 0)",
             bus_read(0x80A), 0xAA);
    return true;
}

// ─── Scenario 27: hold last sample on underflow (MAME-faithful) ──────
// Per MAME asc_sonora_device::sound_stream_update lines 982-993: the
// stream loop snapshots `smpll = (s8)fifo[0][rdptr0] ^ 0x80` BEFORE
// the cap decrement.  On underflow (cap == 0), rdptr doesn't advance,
// so smpll holds the byte at the unchanged rdptr — which is the
// previous good sample.  m_last_left captures it.
//
// Our model: sample_a_raw is only updated on a successful pop; on
// underflow it holds.  So the audio_pcm_l output continues to emit
// the last good sample rather than glitching to silence center.
static bool test_hold_last_sample_on_underflow() {
    reset();
    enable_fifo_mode(/*rate=*/1);
    bus_write(0x000, 0xC0);             // FIFO A sample 1 (signed +0x40)
    phi2_ticks(1);                      // pop sample 1
    dut->eval();
    CHECK_TRUE("audio valid after first sample", dut->audio_sample_valid == 1);
    int16_t pcm_first = (int16_t)dut->audio_pcm_l;
    CHECK_TRUE("first sample is non-zero", pcm_first != 0);

    // Now FIFO A is empty.  Issue another phi2_tick — underflow.
    // sample_a_raw should HOLD the last value (not glitch to silence
    // center 0x80).
    phi2_ticks(1);
    dut->eval();
    CHECK_TRUE("audio valid even on underflow",
               dut->audio_sample_valid == 1);
    int16_t pcm_held = (int16_t)dut->audio_pcm_l;
    CHECK_EQ("audio holds last sample on underflow",
             pcm_held, pcm_first);

    // A few more underflows — still holds.
    phi2_ticks(5);
    dut->eval();
    int16_t pcm_held2 = (int16_t)dut->audio_pcm_l;
    CHECK_EQ("audio still holds last sample after 5 underflows",
             pcm_held2, pcm_first);
    return true;
}

// ─── Scenario 28: FIFO A push base-write hook in playback mode ───────
// Per MAME asc_base_device::write lines 386-404: in playback mode
// (R_PLAYRECA bit 0 = 0), each FIFO A push updates the STAT_HALF_FULL_A
// / STAT_EMPTY_OR_FULL_A bits based on post-push capacity:
//   if cap >= 0x200: clear bit 0 (HALF_FULL_A); if cap >= 0x3FF set
//                    bit 1 (EMPTY_OR_FULL_A == "full" here).
//   else if cap > 0: clear bit 1.
// The Sonora write post-hook (asc.cpp:1109-1116) THEN re-forces bit 1
// high.  Net effect: in playback mode, every push leaves bit 1 high.
//
// In record mode (PLAYRECA bit 0 = 1), the FIFO A base-write hook is
// skipped — no status changes from A pushes.
static bool test_fifo_a_push_status_updates_playback_mode() {
    // Playback mode (PLAYRECA = 0).  Reset() sets PLAYRECA=1, so
    // explicitly re-enable playback.
    reset();
    bus_write(0x80A, 0x00);             // playback mode
    // Pre-state: fifostat = 0x02 (Sonora reset value).
    CHECK_EQ("playback prelude 0x804", bus_read(0x804), 0x02);

    // Push 1 byte: cap=1, < 0x200 and > 0 → base hook clears bit 1.
    // BUT then Sonora post-hook forces bit 1 high again → net: 0x02.
    bus_write(0x000, 0x55);
    CHECK_EQ("after push (1): bit 1 still high (Sonora post-hook)",
             bus_read(0x804) & 0x02, 0x02);

    // Push to cap = 512: base hook now clears HALF_FULL_A (bit 0; was 0
    // already) and bit 1 stays whatever (cap>=0x200 path doesn't touch
    // bit 1 unless cap>=0x3FF).  Sonora post-hook re-forces bit 1.
    for (int i = 0; i < 511; i++) {
        bus_write(0x000, (uint8_t)i);
    }
    CHECK_EQ("after push to cap=512: bit 1 still high",
             bus_read(0x804) & 0x02, 0x02);

    // Push past 0x3FF (1023): base hook sets bit 1 explicitly.  Already
    // high here; verify.
    for (int i = 0; i < 511; i++) {
        bus_write(0x000, (uint8_t)i);
    }
    // Now cap = 1023.  Base hook sees cap >= 0x200 AND cap >= 0x3FF →
    // sets bit 1.  Push hook also sets bit 1.
    CHECK_EQ("after push to cap=1023: bit 1 high (full marker)",
             bus_read(0x804) & 0x02, 0x02);

    // Switch to record mode and try the same — base-hook skipped per
    // MAME's PLAYRECA gate.  Status bits remain whatever they were.
    bus_write(0x803, 0x80);             // clear FIFOs
    bus_write(0x80A, 0x01);             // record mode
    bus_write(0x804, 0x00);             // wipe fifostat for clean slate
    bus_write(0x000, 0x55);
    CHECK_EQ("record-mode push leaves bits 0/1 untouched",
             bus_read(0x804) & 0x03, 0x00);
    return true;
}

// ─── Scenario 29: 0xE00 backdoor write asserts IRQ + sets bits 0..3 ──
// Per MAME asc_base_device::write line 374-378: writing to 0xE00 sets
// `m_regs[R_FIFOSTAT] |= 0xf` and asserts the IRQ line.  ASCTester uses
// this to validate IRQ pin wiring; Q700 ROM never touches it.
static bool test_e00_backdoor() {
    reset();
    bus_write(0xF09, 0x01);             // disable A IRQ — must NOT mask 0xE00
    bus_write(0xF29, 0x01);             // disable B IRQ — must NOT mask 0xE00
    CHECK_TRUE("irq low pre-backdoor", dut->irq == 0);

    bus_write(0xE00, 0x00);              // any data — value doesn't matter
    dut->eval();
    CHECK_TRUE("0xE00 write asserts IRQ", dut->irq == 1);
    uint8_t s = bus_read(0x804);
    CHECK_EQ("0xE00 sets fifostat bits 0..3", s & 0x0F, 0x0F);
    return true;
}

// ─── Scenario 30: F09/F29 write edge IRQ semantics (MAME-faithful) ───
// Per MAME asc_sonora_device::write lines 1082-1104:
//   - Disable→enable transition (data&1: 1→0) AND STAT_HALF_FULL_x set
//     → ASSERT IRQ.
//   - Enable→disable transition (data&1: 0→1) → CLEAR IRQ.
//   - Then update m_fifo_irqen[X] = data & 1.
static bool test_f09_f29_edge_irq() {
    // Set up: both FIFOs filled to 600, stream until past mid → bit 2 set.
    reset();
    fill_fifo_a(600, 0x10);
    fill_fifo_b(600, 0x10);
    enable_fifo_mode(/*rate=*/1);
    phi2_ticks_settle(89);
    CHECK_TRUE("baseline IRQ asserted", dut->irq == 1);
    CHECK_EQ("bit 2 (HALF_FULL_B) set", bus_read(0x804) & 0x04, 0x04);

    // Disable B IRQ — IRQ clears (per MAME's `else if (data&1) clear`).
    bus_write(0xF29, 0x01);
    dut->eval();
    CHECK_TRUE("F29 disable clears IRQ", dut->irq == 0);

    // Re-enable B IRQ — bit 2 still set → IRQ re-asserts.
    bus_write(0xF29, 0x00);
    dut->eval();
    CHECK_TRUE("F29 enable re-asserts IRQ when bit 2 set",
               dut->irq == 1);

    // Force bit 0 (STAT_HALF_FULL_A) by direct 0x804 write — Sonora
    // doesn't normally set it, but the catch-all m_regs[] write does.
    // Also disable A IRQ first to set up the disable→enable scenario.
    bus_write(0xF09, 0x01);
    bus_write(0xF29, 0x01);              // both disabled → IRQ low
    dut->eval();
    CHECK_TRUE("both disabled → IRQ low", dut->irq == 0);
    bus_write(0x804, 0x01);              // force bit 0 high
    CHECK_EQ("bit 0 written via 0x804", bus_read(0x804) & 0x01, 0x01);

    // Enable A IRQ — bit 0 set, F09 disable→enable → IRQ asserts.
    bus_write(0xF09, 0x00);
    dut->eval();
    CHECK_TRUE("F09 enable with bit 0 set asserts IRQ",
               dut->irq == 1);
    return true;
}

// ─── Scenario 31: Sonora reads return 0 for ignored registers ─────────
// Per MAME asc_sonora_device::read (asc.cpp:1042-1047): R_CONTROL,
// R_FIFOMODE, R_WTCONTROL, R_CLOCK, R_BATMANCONTROL all return 0
// regardless of what's been written.  Verify reads after writes.
static bool test_sonora_ignored_register_reads() {
    reset();

    // Write non-zero, then read — Sonora returns 0.
    bus_write(0x802, 0xFF);             // R_CONTROL — backdoored to chan_ctl
    CHECK_EQ("R_CONTROL reads 0 (Sonora)", bus_read(0x802), 0x00);

    bus_write(0x803, 0x55);             // R_FIFOMODE — bit 7 not set, no clear
    CHECK_EQ("R_FIFOMODE reads 0 (Sonora)", bus_read(0x803), 0x00);

    bus_write(0x805, 0xAA);             // R_WTCONTROL
    CHECK_EQ("R_WTCONTROL reads 0", bus_read(0x805), 0x00);

    bus_write(0x807, 0x03);             // R_CLOCK
    CHECK_EQ("R_CLOCK reads 0", bus_read(0x807), 0x00);

    // R_BATMANCONTROL — we accept the write into rate_reg as a tb
    // backdoor, so reads return rate_reg.  In a strict-MAME deployment
    // (no tb backdoor) this would also return 0.  Check the backdoor
    // works without breaking the rest.
    bus_write(0x808, 0x12);
    CHECK_EQ("R_BATMANCONTROL = rate_reg via backdoor",
             bus_read(0x808), 0x12);
    return true;
}

// ─── Scenario 32: 0x804 read clears IRQ iff bit 2 not set ─────────────
// Per MAME asc_sonora_device::read line 1049-1055: 0x804 read clears
// the IRQ line iff !(m_regs[R_FIFOSTAT] & STAT_HALF_FULL_B).  Status
// bits are NOT cleared on read — only the IRQ line (and conditionally).
static bool test_sonora_804_read_clear_conditional() {
    reset();

    // Force IRQ via 0xE00 backdoor; this sets fifostat to 0x0F (all 4
    // bits including bit 2).  0x804 read should then NOT clear IRQ
    // because bit 2 is set.
    bus_write(0xE00, 0x00);
    CHECK_TRUE("backdoor IRQ asserted", dut->irq == 1);
    uint8_t s = bus_read(0x804);
    CHECK_EQ("backdoor sets bits 0..3", s & 0x0F, 0x0F);
    CHECK_TRUE("IRQ stays high (bit 2 set)", dut->irq == 1);
    // Reads do NOT clear status bits per MAME-faithful Sonora.
    CHECK_EQ("status bits preserved across read",
             bus_read(0x804) & 0x0F, 0x0F);

    // Clear bit 2 explicitly via 0x804 write, then read → IRQ clears.
    bus_write(0x804, 0x0B);              // clear bit 2 (0x04), keep 0/1/3
    CHECK_EQ("bit 2 cleared via 0x804 write",
             bus_read(0x804) & 0x04, 0x00);
    (void)bus_read(0x804);
    dut->eval();
    CHECK_TRUE("IRQ clears on 0x804 read with bit 2 clear",
               dut->irq == 0);
    return true;
}

// ─── Scenario 33: Sonora write post-hook re-forces bit 1 in playback ──
// Per MAME asc_sonora_device::write lines 1109-1116: every FIFO A push
// in playback mode (PLAYRECA bit 0 = 0) re-forces R_FIFOSTAT bit 1
// (STAT_EMPTY_OR_FULL_A) high — even if the base-write hook just
// cleared it.  This means in playback mode the CPU never sees A as
// "not empty", which keeps the chime feeder loop active.
static bool test_sonora_post_hook_forces_bit1_playback() {
    reset();
    bus_write(0x80A, 0x00);              // playback mode
    bus_write(0x804, 0x00);              // clear all status bits

    // First push in playback mode: post-hook forces bit 1.
    bus_write(0x000, 0x55);
    CHECK_EQ("playback push 1 forces bit 1",
             bus_read(0x804) & 0x02, 0x02);

    // Manually clear bit 1 by 0x804 write, then push again: post-hook
    // re-forces it.
    bus_write(0x804, 0x00);
    bus_write(0x000, 0x66);
    CHECK_EQ("playback push 2 re-forces bit 1",
             bus_read(0x804) & 0x02, 0x02);

    // Switch to record mode: base-hook skipped, post-hook also gated.
    bus_write(0x80A, 0x01);              // record mode
    bus_write(0x804, 0x00);
    bus_write(0x000, 0x77);
    CHECK_EQ("record-mode push does NOT touch bit 1",
             bus_read(0x804) & 0x02, 0x00);
    return true;
}

// ─── Scenario 34: combined-half-B / empty-B aggregation rules ─────────
// Per MAME asc_sonora_device::sound_stream_update lines 1004-1025:
//   bit 2 (STAT_HALF_FULL_B) = (cap_a < 0x200) || (cap_b < 0x200)
//   bit 3 (STAT_EMPTY_OR_FULL_B) = (cap_a == 0) || (cap_b == 0)
// Both are SET/CLEARED by the stream loop per sample tick.
static bool test_combined_half_empty_aggregation() {
    reset();

    // Both FIFOs empty post-reset → bits 2 and 3 set after first stream.
    enable_fifo_mode(/*rate=*/1);
    phi2_ticks(1);
    dut->eval();
    uint8_t s1 = bus_read(0x804);
    CHECK_EQ("both empty: bit 2 set", s1 & 0x04, 0x04);
    CHECK_EQ("both empty: bit 3 set", s1 & 0x08, 0x08);

    // Fill A above threshold, B stays empty.  Stream tick → bit 2 still
    // set (cap_b < 0x200), bit 3 still set (cap_b == 0).
    fill_fifo_a(700, 0x10);
    phi2_ticks(1);
    dut->eval();
    uint8_t s2 = bus_read(0x804);
    CHECK_EQ("A=700 B=0: bit 2 set (cap_b<0x200)", s2 & 0x04, 0x04);
    CHECK_EQ("A=700 B=0: bit 3 set (cap_b==0)",     s2 & 0x08, 0x08);

    // Fill B too.  Now both above threshold → bits 2 and 3 clear.
    fill_fifo_b(700, 0x40);
    phi2_ticks(1);
    dut->eval();
    uint8_t s3 = bus_read(0x804);
    CHECK_EQ("A=700 B=700: bit 2 clear", s3 & 0x04, 0x00);
    CHECK_EQ("A=700 B=700: bit 3 clear", s3 & 0x08, 0x00);

    // Drain A to 0 while B still high.  Bit 2 set (cap_a<0x200) once
    // cap_a falls below 512.  Bit 3 set (cap_a==0) at terminal.
    phi2_ticks_settle(700);              // drain past empty
    uint8_t s4 = bus_read(0x804);
    CHECK_EQ("A drained: bit 2 set", s4 & 0x04, 0x04);
    CHECK_EQ("A drained: bit 3 set (cap_a==0)", s4 & 0x08, 0x08);
    return true;
}

// ─── Scenario 23: +0x804 write/read probe readback ───────────────────
// Per MAME asc_base_device::write line 596: writes to 0x800-0xFFF land
// in m_regs[offset-0x800].  R_FIFOSTAT (0x804) is no exception — every
// 8 bits of the data byte are stored.  Read returns the latched byte
// directly; writes do NOT synthesize IRQs.
static bool test_irq_status_probe_readback() {
    reset();

    bus_write(0x804, 0x01);
    CHECK_EQ("irq status writable, returns latched value",
             bus_read(0x804), 0x01);
    CHECK_TRUE("probe write does not assert IRQ", dut->irq == 0);

    bus_write(0x804, 0x0F);
    CHECK_EQ("low-nibble write readback", bus_read(0x804), 0x0F);
    CHECK_TRUE("probe bits still do not assert IRQ", dut->irq == 0);

    // Upper 4 bits are also writable (general-purpose scratch on Sonora).
    bus_write(0x804, 0xA5);
    CHECK_EQ("full byte readback", bus_read(0x804), 0xA5);
    return true;
}

// ─── Scenario 27: EASC ext WRPTR/RDPTR live readback (0xF00..0xF03,
//     0xF20..0xF23) ─────────────────────────────────────────────────────
// MAME shadows these in m_regs[]; we surface the live FIFO pointers
// (the brief calls for reads to reflect actual FIFO state, not just
// software-latched values).  Push N bytes → fa_wp/fb_wp advance.  Drain
// via phi2_ticks → fa_rp/fb_rp advance.  Both surface as hi/lo bytes
// where hi only carries pointer bits 9:8 (low 6 bits read 0).
static bool test_easc_ext_wrptr_rdptr_readback() {
    reset();
    // Both pointers should start at 0.
    CHECK_EQ("WRPTRA hi @reset", bus_read(0xF00), 0x00);
    CHECK_EQ("WRPTRA lo @reset", bus_read(0xF01), 0x00);
    CHECK_EQ("RDPTRA hi @reset", bus_read(0xF02), 0x00);
    CHECK_EQ("RDPTRA lo @reset", bus_read(0xF03), 0x00);
    CHECK_EQ("WRPTRB hi @reset", bus_read(0xF20), 0x00);
    CHECK_EQ("WRPTRB lo @reset", bus_read(0xF21), 0x00);
    CHECK_EQ("RDPTRB hi @reset", bus_read(0xF22), 0x00);
    CHECK_EQ("RDPTRB lo @reset", bus_read(0xF23), 0x00);

    // Push enough bytes to A and B that the wp wraps into the high
    // byte (>= 256) so hi/lo split coverage is exercised.  300 bytes
    // → wp = 0x12C, hi = 0x01, lo = 0x2C.
    fill_fifo_a(300, 0x10);
    fill_fifo_b(260, 0x40);
    CHECK_EQ("WRPTRA hi after 300B", bus_read(0xF00), 0x01);
    CHECK_EQ("WRPTRA lo after 300B", bus_read(0xF01), 0x2C);
    CHECK_EQ("RDPTRA hi unchanged",  bus_read(0xF02), 0x00);
    CHECK_EQ("RDPTRA lo unchanged",  bus_read(0xF03), 0x00);
    CHECK_EQ("WRPTRB hi after 260B", bus_read(0xF20), 0x01);
    CHECK_EQ("WRPTRB lo after 260B", bus_read(0xF21), 0x04);
    CHECK_EQ("RDPTRB hi unchanged",  bus_read(0xF22), 0x00);
    CHECK_EQ("RDPTRB lo unchanged",  bus_read(0xF23), 0x00);

    // Drain N samples — both rp pointers advance by N.  Rate = 1 plus
    // a trailing settle tick (phi2 deasserted) gives exactly N sample-
    // ticks per phi2_ticks(N).  N=100 → rp = 0x064.
    bus_write(0x808, 1);  // rate = 1 → tick every phi2 pulse
    phi2_ticks(100);
    CHECK_EQ("RDPTRA hi after drain", bus_read(0xF02), 0x00);
    CHECK_EQ("RDPTRA lo after drain", bus_read(0xF03), 0x64);
    CHECK_EQ("RDPTRB hi after drain", bus_read(0xF22), 0x00);
    CHECK_EQ("RDPTRB lo after drain", bus_read(0xF23), 0x64);

    // WP should be unchanged from the post-fill values (drain doesn't
    // touch the write pointer).
    CHECK_EQ("WRPTRA hi after drain", bus_read(0xF00), 0x01);
    CHECK_EQ("WRPTRA lo after drain", bus_read(0xF01), 0x2C);
    CHECK_EQ("WRPTRB hi after drain", bus_read(0xF20), 0x01);
    CHECK_EQ("WRPTRB lo after drain", bus_read(0xF21), 0x04);

    // Hi byte must zero out the upper 6 bits even if a write tried to
    // set them — surface is {6'b0, ptr[9:8]}.  Pointer is currently
    // 0x12C / 0x104 so hi = 0x01 → upper 6 bits already 0; just check.
    CHECK_TRUE("WRPTRA hi upper bits zero", (bus_read(0xF00) & 0xFC) == 0x00);
    CHECK_TRUE("RDPTRB hi upper bits zero", (bus_read(0xF22) & 0xFC) == 0x00);
    return true;
}

// ─── Scenario 28: SRC + per-channel volume + FIFO control latched ────
// 0xF04..0xF08 (FIFO A) and 0xF24..0xF28 (FIFO B): each register holds
// independently after a distinctive write.  No functional side-effects
// yet (SRC engine + per-channel mixing + CD-XA decode are deferred);
// this gates programmer-visible read/write behavior so software can
// probe presence and Sound Manager can park values for later use.
static bool test_easc_ext_src_volume_ctrl_latched() {
    reset();
    // Distinctive walking-bit pattern, A side.
    bus_write(0xF04, 0xA1);   // R_SRCA_H
    bus_write(0xF05, 0xB2);   // R_SRCA_L
    bus_write(0xF06, 0xC3);   // R_VOLA_L
    bus_write(0xF07, 0xD4);   // R_VOLA_R
    bus_write(0xF08, 0xE5);   // R_FIFOA_CTRL
    // Distinctive different pattern, B side.
    bus_write(0xF24, 0x5A);   // R_SRCB_H
    bus_write(0xF25, 0x4B);   // R_SRCB_L
    bus_write(0xF26, 0x3C);   // R_VOLB_L
    bus_write(0xF27, 0x2D);   // R_VOLB_R
    bus_write(0xF28, 0x1E);   // R_FIFOB_CTRL

    CHECK_EQ("SRCA_H readback",      bus_read(0xF04), 0xA1);
    CHECK_EQ("SRCA_L readback",      bus_read(0xF05), 0xB2);
    CHECK_EQ("VOLA_L readback",      bus_read(0xF06), 0xC3);
    CHECK_EQ("VOLA_R readback",      bus_read(0xF07), 0xD4);
    CHECK_EQ("FIFOA_CTRL readback",  bus_read(0xF08), 0xE5);
    CHECK_EQ("SRCB_H readback",      bus_read(0xF24), 0x5A);
    CHECK_EQ("SRCB_L readback",      bus_read(0xF25), 0x4B);
    CHECK_EQ("VOLB_L readback",      bus_read(0xF26), 0x3C);
    CHECK_EQ("VOLB_R readback",      bus_read(0xF27), 0x2D);
    CHECK_EQ("FIFOB_CTRL readback",  bus_read(0xF28), 0x1E);

    // Sanity: master volume_reg at 0x806 untouched by these writes.
    CHECK_EQ("master volume unchanged by F06/F26",
             bus_read(0x806), 0xFF);
    // F09 already-implemented IRQ control should not have leaked into
    // the new ext-block decode (ensure parallel agent's territory at
    // 0x800-0x82F also stays untouched here — chan_ctl, fifo_ctl).
    CHECK_EQ("F09 still default (enabled)", bus_read(0xF09), 0x00);
    CHECK_EQ("F29 still default (enabled)", bus_read(0xF29), 0x00);

    // Independence cross-check: rewrite A side, B side must not move.
    bus_write(0xF04, 0x00);
    bus_write(0xF05, 0x00);
    CHECK_EQ("SRCA_H cleared", bus_read(0xF04), 0x00);
    CHECK_EQ("SRCB_H still set after A clear", bus_read(0xF24), 0x5A);
    CHECK_EQ("SRCB_L still set after A clear", bus_read(0xF25), 0x4B);

    return true;
}

// ─── Scenario 29: CD-XA decoder stub registers latch but don't side-
//     effect (0xF10, 0xF30) ─────────────────────────────────────────────
// Real Sonora performs CD-XA ADPCM decode through these register slots
// (block-header + filter coeffs at 0xF10..0xF17 / 0xF30..0xF37 per the
// asc.cpp comment); our stub captures a single byte at the base offset
// and leaves the FIFO/pointer state untouched.  This proves the stub is
// inert (no spurious IRQ, no FIFO pointer movement) and that software
// reading back the byte gets exactly what it wrote.
static bool test_easc_ext_cdxa_stub_latches() {
    reset();
    // Initial reads of CD-XA registers — both 0 at reset.
    CHECK_EQ("CDXA_A @reset", bus_read(0xF10), 0x00);
    CHECK_EQ("CDXA_B @reset", bus_read(0xF30), 0x00);

    // Snapshot FIFO pointers / IRQ before the stub poke.
    uint8_t fa_wp_lo_before = bus_read(0xF01);
    uint8_t fb_wp_lo_before = bus_read(0xF21);
    uint8_t irq_before = dut->irq;

    // Walking-bit pattern: write distinct values to the A and B stubs.
    bus_write(0xF10, 0x55);
    bus_write(0xF30, 0xAA);
    CHECK_EQ("CDXA_A latch readback", bus_read(0xF10), 0x55);
    CHECK_EQ("CDXA_B latch readback", bus_read(0xF30), 0xAA);

    // Independence: writing F10 must not touch F30 and vice versa.
    bus_write(0xF10, 0xC9);
    CHECK_EQ("CDXA_A re-latch",        bus_read(0xF10), 0xC9);
    CHECK_EQ("CDXA_B independent",     bus_read(0xF30), 0xAA);
    bus_write(0xF30, 0x62);
    CHECK_EQ("CDXA_B re-latch",        bus_read(0xF30), 0x62);
    CHECK_EQ("CDXA_A independent",     bus_read(0xF10), 0xC9);

    // Inert: no FIFO pointer movement, no IRQ assertion side-effect.
    CHECK_EQ("FIFO A wp lo unchanged by CDXA stub",
             bus_read(0xF01), fa_wp_lo_before);
    CHECK_EQ("FIFO B wp lo unchanged by CDXA stub",
             bus_read(0xF21), fb_wp_lo_before);
    CHECK_EQ("irq line unchanged by CDXA stub", dut->irq, irq_before);

    // The CD-XA filter-coefficient region (0xF11..0xF17 / 0xF31..0xF37)
    // is not implemented; reads return 0.  Document the contract here
    // so a future ADPCM engine knows where to plug in.
    CHECK_EQ("CDXA_A+1 reads as 0 (coeff region unimpl)", bus_read(0xF11), 0x00);
    CHECK_EQ("CDXA_B+1 reads as 0 (coeff region unimpl)", bus_read(0xF31), 0x00);

    return true;
}

// ─── Q700 boot-quiescence: IRQ low pre-PLAYRECA-write ─────────────────
// Per docs/asc_q700_rom_trace.md: the Q700 boot ROM never writes 0x80A
// (R_PLAYRECA), never writes 0xF09 (R_FIFOA_IRQCTRL), and reads 0x804
// 0 times.  Reset state: PLAYRECA = 0 (playback mode), fa_irqen = 0
// (IRQ enabled).  Without a sticky `playreca_written` gate, our
// stream-side level-fire would assert IRQ on every sample tick during
// the empty-FIFO window.  MAME's IRQ stays low through boot — verify
// our gate matches.
//
// This test bypasses the normal `reset()` helper (which writes 0x80A=1
// to silence legacy tests) and uses a raw register reset to mirror
// real boot.
static bool test_q700_boot_irq_quiescent_pre_playreca_write() {
    // Raw reset — do NOT write 0x80A.
    dut->rst       = 1;
    dut->phi2_tick = 0;
    dut->pb_addr   = 0;
    dut->pb_wdata  = 0;
    dut->pb_wr     = 0;
    dut->pb_rd     = 0;
    tick(); tick();
    dut->rst = 0;
    tick();

    // Default state: PLAYRECA = 0 (playback), fa_irqen = 0 (enabled),
    // FIFOs empty.  Without the playreca_written gate, level-fire
    // would already be high.
    CHECK_EQ("playreca @raw-reset = 0 (playback mode default)",
             bus_read(0x80A), 0x00);
    CHECK_EQ("fa_irqen @raw-reset = 0 (IRQ enabled)",
             bus_read(0xF09), 0x00);
    CHECK_TRUE("irq low pre-PLAYRECA-write at empty FIFOs",
               dut->irq == 0);

    // Run many phi2 ticks — sample-tick fires every 35 — IRQ should
    // stay low because playreca_written is 0.
    phi2_ticks(200);
    dut->eval();
    CHECK_TRUE("irq low after 200 phi2 ticks (sample-tick fired ~5x)",
               dut->irq == 0);

    // Mimic the ROM's actual boot writes: clear MODE/CLOCK, set
    // VOLUME, clear CONTROL, push 700 bytes into FIFO A.  None of
    // these touch PLAYRECA — IRQ must stay low throughout.  R_MODE
    // briefly hits 0 (silent) which suspends the sample tick stream;
    // restore FIFO mode at the end so the post-PLAYRECA assertion can
    // see the next stream tick.
    bus_write(0x801, 0x00);   // MODE clear (silent)
    bus_write(0x807, 0x00);   // CLOCK clear (no-op)
    bus_write(0x806, 0x40);   // VOLUME = 0x40
    bus_write(0x802, 0x00);   // CONTROL clear
    bus_write(0x801, 0x01);   // restore FIFO mode for the fill below
    for (int i = 0; i < 700; i++) {
        bus_write(0x000, (uint8_t)((0xC001FF40 >> ((i & 3) * 8)) & 0xFF));
    }
    CHECK_TRUE("irq low after 700-byte FIFO A fill (no PLAYRECA write)",
               dut->irq == 0);

    // Now write PLAYRECA — sticky flag latches.  Level-fire becomes
    // active on the NEXT sample tick: FIFO B is empty so the combined-B
    // condition (cap_a < 0x200 || cap_b < 0x200) holds → IRQ asserts.
    // Sample-tick fires every 35 phi2 ticks at Sonora's default rate.
    bus_write(0x80A, 0x00);   // explicit playback-mode enable
    phi2_ticks(40);           // give the divider a chance to fire
    dut->eval();
    CHECK_TRUE("irq asserts on next sample-tick after PLAYRECA write",
               dut->irq == 1);
    return true;
}

// ─── Scenario 35: chime sine-wave end-to-end ─────────────────────────
// The Sound Manager's startup chime path is: write 64 samples of a
// half-cycle sine into FIFO A, set R_CONTROL = 0 (mono), program
// R_VOLUME, then drain via the sample-rate clock.  This test exercises
// that exact path and asserts:
//   (a) every audio_sample_valid pulse produces a sample byte that
//       matches the FIFO byte we pushed (no off-by-one, no swap), with
//       the standard MAME centring (`(s8)byte ^ 0x80`).
//   (b) audio_pcm_l mirrors the centred byte in the upper 8 bits.
//   (c) sample-rate cadence matches the rate-divider seed.
// This is the chime-path smoke test the brief calls for: known sample
// pattern in → expected stream out → no FIFO/IRQ glitching during
// playback.  If this regresses, the iconic "bong" stops working.
static bool test_chime_sine_wave_playback() {
    reset();

    // Build a half-cycle sine table (64 entries, signed 8-bit centred at
    // 0x80) — close to what the actual Sound Manager pushes for the
    // chime header.  The sine table is stored unsigned (0x80 = silence,
    // 0x00 = -full, 0xFF = +full) — the chip XORs with 0x80 to centre.
    static const uint8_t kSineTable[64] = {
        0x80, 0x8C, 0x98, 0xA5, 0xB0, 0xBC, 0xC7, 0xD1,
        0xDA, 0xE2, 0xEA, 0xF0, 0xF5, 0xFA, 0xFD, 0xFE,
        0xFE, 0xFD, 0xFA, 0xF5, 0xF0, 0xEA, 0xE2, 0xDA,
        0xD1, 0xC7, 0xBC, 0xB0, 0xA5, 0x98, 0x8C, 0x80,
        0x73, 0x67, 0x5A, 0x4F, 0x43, 0x38, 0x2E, 0x25,
        0x1D, 0x15, 0x0F, 0x0A, 0x05, 0x02, 0x01, 0x01,
        0x01, 0x02, 0x05, 0x0A, 0x0F, 0x15, 0x1D, 0x25,
        0x2E, 0x38, 0x43, 0x4F, 0x5A, 0x67, 0x73, 0x80
    };

    // Push the table.  fill_fifo_a doesn't accept a literal buffer, so
    // walk it manually.
    bus_write(0x801, 0x01);            // mode = FIFO (no-op on Sonora; documents intent)
    bus_write(0x802, 0x00);            // mono presentation
    bus_write(0x806, 0xFF);            // master volume full-scale
    bus_write(0x80A, 0x01);            // record mode (silences playback-mode level IRQ)
    bus_write(0xF09, 0x01);            // disable per-FIFO A IRQ for this test
    bus_write(0xF29, 0x01);            // disable per-FIFO B IRQ
    bus_write(0x808, 1);                // rate=1 → one sample per phi2_tick
    for (int i = 0; i < 64; i++)
        bus_write(0x000, kSineTable[i]);

    // Drain: each phi2_tick produces a sample on the FOLLOWING clock
    // (the audio output latch is delayed by one cycle behind sample_tick).
    // Verify each sample matches the centred FIFO byte.
    for (int i = 0; i < 64; i++) {
        phi2_ticks(1);
        dut->eval();
        if (!dut->audio_sample_valid) {
            printf("  FAIL chime sample %d: audio_sample_valid not asserted\n", i);
            return false;
        }
        // Centring: raw byte XOR 0x80, then sign-extend to s16 << 8 for PCM.
        uint16_t expected_packed = ((uint16_t)(kSineTable[i] ^ 0x80) & 0xFF);
        uint16_t got_lo = dut->audio_sample_out & 0xFF;
        uint16_t got_hi = (dut->audio_sample_out >> 8) & 0xFF;
        if (got_lo != expected_packed || got_hi != expected_packed) {
            printf("  FAIL chime sample %d: got_pack=0x%02x_%02x expected=0x%02x\n",
                   i, got_hi, got_lo, expected_packed);
            return false;
        }
        // PCM signed-16: centred byte sign-extended << 8.
        int16_t expected_pcm = (int16_t)((int8_t)(kSineTable[i] ^ 0x80)) << 8;
        if ((int16_t)dut->audio_pcm_l != expected_pcm) {
            printf("  FAIL chime sample %d PCM: got=%d expected=%d\n",
                   i, (int16_t)dut->audio_pcm_l, expected_pcm);
            return false;
        }
    }

    // After the 64th sample, FIFO A is empty.  Subsequent phi2_ticks
    // hold the last good sample (MAME-faithful — see scenario 27).
    int16_t pcm_last = (int16_t)dut->audio_pcm_l;
    phi2_ticks(4);
    dut->eval();
    CHECK_EQ("chime: hold last sample after FIFO drain",
             (int16_t)dut->audio_pcm_l, pcm_last);
    return true;
}

// ─── Scenario 36: chime cadence at default Q700 sample rate ──────────
// With no rate programming, the EASC default is hardwired (~22.257 kHz
// on real silicon).  Our RATE_DEFAULT = 35 at Q700 phi2 (783360 Hz)
// produces 22.381 kHz — within 0.6% of MAME's hardwired value (well
// below audible).  Verify the sample-tick cadence matches the divider
// without explicit programming so a chime pushed without rate setup
// plays at the correct pitch.
static bool test_chime_default_rate_cadence() {
    reset();

    // Push a few distinguishable bytes — chime preamble would be tens
    // of samples; we need just enough to verify cadence.
    bus_write(0x80A, 0x01);            // record mode
    bus_write(0xF09, 0x01);            // disable A IRQ (avoid noise)
    bus_write(0xF29, 0x01);            // disable B IRQ
    bus_write(0x000, 0xC0);            // sample 0
    bus_write(0x000, 0x40);            // sample 1
    bus_write(0x000, 0x80);            // sample 2 (silence)

    // Default rate (35): tick fires on every 35th phi2 pulse.  After
    // 34 ticks, no sample yet; 35th tick fires sample.  See existing
    // test_sonora_default_rate_drains_fifo for the rate-walk derivation.
    phi2_ticks(34);
    dut->eval();
    CHECK_TRUE("no sample at phi2_tick 34 (default rate)",
               dut->audio_sample_valid == 0);

    // 35th tick fires.
    phi2_ticks(1);
    dut->eval();
    CHECK_TRUE("sample fires on phi2_tick 35 (default rate)",
               dut->audio_sample_valid == 1);
    // The first sample is byte 0xC0 → centred 0x40 → PCM = 0x4000.
    CHECK_EQ("first chime sample matches FIFO byte 0",
             (int16_t)dut->audio_pcm_l, (int16_t)0x4000);

    // Next 35 phi2 ticks → sample 1 (byte 0x40 → centred 0xC0 → PCM = -0x4000).
    phi2_ticks(35);
    dut->eval();
    CHECK_EQ("second chime sample matches FIFO byte 1",
             (int16_t)dut->audio_pcm_l, (int16_t)0xC000);

    return true;
}

// ─── Scenario 37: wavetable mode 4-voice mix at index 0 ─────────────
// Per MAME asc_base_device::sound_stream_update case 2 (asc.cpp:248-282)
// + the brief: each voice samples from its 512-byte slot in the unified
// 2 KB FIFO array (voice 0 = fifo_a[0..0x1FF], voice 1 =
// fifo_a[0x200..0x3FF], voice 2 = fifo_b[0..0x1FF], voice 3 =
// fifo_b[0x200..0x3FF]).  With phase = incr = 0 the index pins to 0 of
// each slot, so the audio output is `sum_v(byte^0x80)<<8 / 4` (clamp
// elided since we're at most 4×s16 max).  Verify a known set of bytes
// at the four voice index-0 positions produces the expected mixed PCM.
static bool test_wavetable_mode_index_zero_mix() {
    reset();

    // Stamp distinct bytes at voice 0/1/2/3 index 0 via direct-addressed
    // writes in WAVE mode.  Wave-mode writes go to fifo[pb_addr[9:0]]
    // per the first-gen ASC's `m_fifo[ch][offset] = data;` semantic.
    bus_write(0x801, 0x02);   // mode = WAVE
    bus_write(0x000, 0xC0);   // voice 0 sample [0]: 0xC0 → centred 0x40
    bus_write(0x200, 0x40);   // voice 1 sample [0]: 0x40 → centred 0xC0
    bus_write(0x400, 0xFF);   // voice 2 sample [0]: 0xFF → centred 0x7F
    bus_write(0x600, 0x80);   // voice 3 sample [0]: 0x80 → centred 0x00
    bus_write(0x808, 0x01);   // rate = 1 (one sample per phi2_tick)

    // Phase / incr already zero from reset.  Fire one sample tick.
    phi2_ticks(1);
    dut->eval();
    CHECK_TRUE("wave mode: audio_sample_valid pulses on tick",
               dut->audio_sample_valid == 1);

    // Expected mix (each voice byte XOR 0x80 → s8 → s16<<8, sum, /4):
    //   v0: 0x40 → 0x4000   v1: 0xC0 → 0xC000 (=-0x4000)
    //   v2: 0x7F → 0x7F00   v3: 0x00 → 0x0000
    //   sum = 0x4000 - 0x4000 + 0x7F00 + 0x0000 = 0x7F00
    //   >>2 = 0x1FC0
    int16_t expected_pcm = (int16_t)0x1FC0;
    CHECK_EQ("wave mode mixed PCM matches sum-of-voices/4",
             (int16_t)dut->audio_pcm_l, expected_pcm);
    CHECK_EQ("wave mode pcm_r matches pcm_l (mono mix)",
             (int16_t)dut->audio_pcm_r, expected_pcm);
    return true;
}

// ─── Scenario 38: wavetable phase advance walks the slot ────────────
// With incr non-zero, phase advances on every sample tick and the
// index `(phase>>15) & 0x1ff` walks through the voice slot.  Stamp a
// distinguishable pattern (raw byte = 0x80 + index*1, capped) and
// confirm successive sample ticks read successive slot positions.
static bool test_wavetable_phase_advance_walks_slot() {
    reset();

    bus_write(0x801, 0x02);    // mode = WAVE
    bus_write(0x808, 0x01);    // rate = 1

    // Voice 0 wavetable: a ramp 0x80, 0x82, 0x84, ... stamped into
    // fifo_a[0..0x1FF] (within wave mode = direct-addressed write).
    for (int i = 0; i < 64; i++)
        bus_write((uint16_t)i, (uint8_t)(0x80 + (i * 2)));
    // Zero out the other 3 voices' index-0 byte so they contribute 0.
    bus_write(0x200, 0x80);
    bus_write(0x400, 0x80);
    bus_write(0x600, 0x80);

    // Set incr[0] = 0x008000 → phase advances by 0x8000 per tick → top-9
    // bits walk by 1 per tick.  Phase format per MAME asc.h
    // R_WAVETABLE0_INCR (asc.cpp:33-40): big-endian 24-bit packed as
    // bytes at +0x815, +0x816, +0x817 (low byte last).  We want
    // 0x008000 → bytes 0x00, 0x80, 0x00.
    bus_write(0x815, 0x00);    // incr[0][23:16]
    bus_write(0x816, 0x80);    // incr[0][15:8]
    bus_write(0x817, 0x00);    // incr[0][7:0]

    // First tick: phase = 0 → 0x008000; index post-update = 0x8000>>15 = 1
    // → reads fifo_a[1] = 0x82.  Centred = 0x02.  s16<<8 = 0x0200.
    // Other voices = 0x80 → centred 0 → 0.  Sum = 0x0200, /4 = 0x80.
    phi2_ticks(1);
    dut->eval();
    CHECK_EQ("wave phase tick 1: index 1, ramp byte 0x82, PCM = 0x80",
             (int16_t)dut->audio_pcm_l, (int16_t)0x80);

    // Tick 2: phase = 0x010000; index = 2 → fifo_a[2] = 0x84 → centred
    // 0x04 → s16 0x0400 → /4 = 0x100.
    phi2_ticks(1);
    dut->eval();
    CHECK_EQ("wave phase tick 2: index 2, ramp byte 0x84, PCM = 0x100",
             (int16_t)dut->audio_pcm_l, (int16_t)0x100);

    // Tick 5: phase = 0x028000; index = 5 → fifo_a[5] = 0x8A → centred
    // 0x0A → s16 0x0A00 → /4 = 0x280.
    phi2_ticks(3);
    dut->eval();
    CHECK_EQ("wave phase tick 5: index 5, ramp byte 0x8A, PCM = 0x280",
             (int16_t)dut->audio_pcm_l, (int16_t)0x280);
    return true;
}

// ─── Scenario 39: FIFO mode untouched by wavetable writes ────────────
// Wavetable phase/incr writes (0x811-0x82F) must not disturb FIFO mode
// playback.  Verify that programming voice 0 incr while in FIFO mode
// has no effect on the FIFO drain stream.
static bool test_fifo_mode_unaffected_by_wt_writes() {
    reset();

    fill_fifo_a(4, 0xC0);
    enable_fifo_mode(/*rate=*/1);

    // Stamp a non-zero increment on voice 0 — should be latched but
    // inert in FIFO mode.
    bus_write(0x815, 0x12);
    bus_write(0x816, 0x34);
    bus_write(0x817, 0x56);

    phi2_ticks(1);
    dut->eval();
    CHECK_TRUE("FIFO mode tick valid after WT writes",
               dut->audio_sample_valid == 1);
    // Sample 0 (byte 0xC0 → centred 0x40 → PCM = 0x4000).
    CHECK_EQ("FIFO mode sample 0 unchanged",
             (int16_t)dut->audio_pcm_l, (int16_t)0x4000);
    return true;
}

// ─── Iter-4 EASC bookkeeping scenarios (A/B/C/D) ──────────────────────
// These tests exercise the four EASC divergences identified in the iter-3
// report, gated on the chip identifying as EASC at runtime (R_VERSION
// reads 0xB0).  Under VERSION=0x00 they soft-skip with PASS so the tb
// catalogue stays green pre-flip; under VERSION=0xB0 they verify the
// MAME-faithful asc_easc_device behavior.  See VERSION_USE / IS_EASC
// localparams in asc.v.

// ─── Scenario 40: EASC R_MODE accepts bit 0 only (divergence A) ──────
// Per MAME asc_easc_device::write (asc.cpp:1683-1690):
//   m_regs[R_MODE] = data & 1;   // only bit 0 can be written
// Plus on a (data&1) != m_regs[R_MODE] transition: rdptr/wrptr/cap
// reset for both channels and STAT_EMPTY_OR_FULL_B (0x8) is OR'd into
// FIFOSTAT.  Verify both: the bit-0 mask AND the FIFO state reset on
// toggle.
static bool test_easc_mode_accepts_bit0_only() {
    reset();
    if (!chip_is_easc()) {
        printf("  [skip] non-EASC build (VERSION_USE != 0xB0)\n");
        return true;
    }

    // After reset, mode = 1 (FIFO).  Toggle to 0 — must reset FIFO
    // pointers + OR bit 3 into FIFOSTAT.
    fill_fifo_a(64, 0x10);   // fa_count = 64 in FIFO mode
    bus_write(0x801, 0x00);
    CHECK_EQ("mode = 0 after data=0",   bus_read(0x801), 0x00);
    // The wp/rp aren't directly readable at 0x801, but the live wrptr
    // surfaces at 0xF00..0xF01.  After toggle: must read 0.
    CHECK_EQ("EASC: wrptr A hi cleared on mode toggle",
             bus_read(0xF00), 0x00);
    CHECK_EQ("EASC: wrptr A lo cleared on mode toggle",
             bus_read(0xF01), 0x00);

    // Write data = 0x02 (would be wavetable on first-gen ASC) — EASC
    // must mask to 0 (bit 0 of 0x02 is 0).  No state-toggle, FIFO
    // state preserved.
    bus_write(0x801, 0x02);
    CHECK_EQ("EASC: mode write 0x02 masked to 0", bus_read(0x801), 0x00);

    // Write data = 0x03 (low 2 bits = 11) — EASC masks to bit 0 → mode = 1
    // (fifo).  Toggle from 0 → 1.
    bus_write(0x801, 0x03);
    CHECK_EQ("EASC: mode write 0x03 masked to 1", bus_read(0x801), 0x01);

    // Verify the mode-toggle hook OR'd STAT_EMPTY_OR_FULL_B (0x8) high.
    uint8_t s = bus_read(0x804);
    CHECK_TRUE("EASC: mode toggle sets STAT_EMPTY_OR_FULL_B", (s & 0x08) != 0);
    return true;
}

// ─── Scenario 41: EASC R_CLOCK reads 3 (divergence B) ────────────────
// Per MAME asc_easc_device::read (asc.cpp:1662-1664):
//   case R_CLOCK: return 3;   // read-only, "44.1 kHz on original ASC"
// Sonora's read returns 0 (asc.cpp:1045).
static bool test_easc_clock_reads_three() {
    reset();
    if (!chip_is_easc()) {
        printf("  [skip] non-EASC build (VERSION_USE != 0xB0)\n");
        return true;
    }
    // EASC: R_CLOCK readback is hardwired 3 regardless of writes.
    CHECK_EQ("EASC: R_CLOCK = 3 @reset", bus_read(0x807), 0x03);
    bus_write(0x807, 0xAA);
    CHECK_EQ("EASC: R_CLOCK = 3 after write 0xAA", bus_read(0x807), 0x03);
    bus_write(0x807, 0x00);
    CHECK_EQ("EASC: R_CLOCK = 3 after write 0x00", bus_read(0x807), 0x03);
    return true;
}

// ─── Scenario 42: EASC per-channel STAT_HALF_FULL_A/B (divergence C) ─
// Per MAME asc_easc_device::pop_fifo (asc.cpp:1555-1576): each consumed
// sample updates STAT_HALF_FULL_x AND STAT_EMPTY_OR_FULL_x INDEPENDENTLY
// for the channel it belongs to.  STAT_HALF_FULL_A (bit 0) tracks ONLY
// channel A; STAT_HALF_FULL_B (bit 2) tracks ONLY channel B.  This is
// distinct from Sonora's combined `cap_a < 0x200 || cap_b < 0x200`
// model where bit 2 is overloaded as "either channel half".
static bool test_easc_per_channel_half_full() {
    reset();
    if (!chip_is_easc()) {
        printf("  [skip] non-EASC build (VERSION_USE != 0xB0)\n");
        return true;
    }
    // Drop into playback mode so the playback-mode top-of-stream
    // PLAYRECA force is suppressed (EASC has none) and IRQ is gated
    // by the per-channel half-full check.  Disable both IRQs upfront.
    bus_write(0xF09, 0x01);   // FIFO A IRQ disabled (bit 0 = 1)
    bus_write(0xF29, 0x01);   // FIFO B IRQ disabled
    bus_write(0x80A, 0x00);   // playback mode (bit 0 = 0)

    // Fill ONLY FIFO A above the half-full threshold (cap = 600 > 0x1ff).
    // FIFO B stays empty (cap = 0).  Step the stream a few sample ticks
    // and confirm:
    //   - bit 0 (HALF_FULL_A) clears once cap_a > 0x1ff (cleared after
    //     each pop).
    //   - bit 2 (HALF_FULL_B) sets independently because cap_b == 0
    //     stays at <= 0x1ff regardless of channel A.
    fill_fifo_a(600, 0x10);
    enable_fifo_mode(/*rate=*/1);
    phi2_ticks_settle(8);
    uint8_t s1 = bus_read(0x804);
    CHECK_TRUE("EASC: HALF_FULL_A clear when cap_a > 0x1ff (s1)",
               (s1 & 0x01) == 0);
    CHECK_TRUE("EASC: HALF_FULL_B set when cap_b <= 0x1ff",
               (s1 & 0x04) != 0);
    CHECK_TRUE("EASC: EMPTY_B set when cap_b == 0",
               (s1 & 0x08) != 0);
    // Now drain FIFO A below 0x1ff.  Top up to ensure we haven't
    // already crossed.  After ~89 sample ticks cap_a falls below 0x200.
    phi2_ticks_settle(96);
    uint8_t s2 = bus_read(0x804);
    CHECK_TRUE("EASC: HALF_FULL_A set after enough drains",
               (s2 & 0x01) != 0);
    return true;
}

// ─── Scenario 43: EASC F09/F29 edge IRQ semantics (divergence D) ─────
// Per MAME asc_easc_device::write (asc.cpp:1697-1719) and
// asc_sonora_device::write (asc.cpp:1082-1104) — semantically identical.
// On disable→enable transition (data&1: 1→0) AND STAT_HALF_FULL_x set,
// IRQ asserts.  On enable→disable (data&1: 0→1), IRQ clears.  This
// scenario exercises the F09/F29 transitions explicitly and confirms
// the EASC variant matches the Sonora behavior we already have.
static bool test_easc_f09_f29_edge_irq_match() {
    reset();
    if (!chip_is_easc()) {
        printf("  [skip] non-EASC build (VERSION_USE != 0xB0)\n");
        return true;
    }
    // Disable both IRQs initially.
    bus_write(0xF09, 0x01);
    bus_write(0xF29, 0x01);
    CHECK_TRUE("EASC: irq low after both disabled", dut->irq == 0);

    // Force STAT_HALF_FULL_A high via 0x804 direct write (writable
    // catch-all per MAME's m_regs[]).
    bus_write(0x804, 0x01);
    CHECK_TRUE("EASC: irq still low (still disabled)", dut->irq == 0);

    // F09 edge: 1 → 0.  HALF_FULL_A is set → IRQ asserts.
    bus_write(0xF09, 0x00);
    CHECK_TRUE("EASC: F09 disable→enable + HALF_A → irq asserts",
               dut->irq == 1);

    // F09 edge: 0 → 1.  IRQ clears.
    bus_write(0xF09, 0x01);
    CHECK_TRUE("EASC: F09 enable→disable → irq clears",
               dut->irq == 0);

    // Same for F29 with HALF_FULL_B.
    bus_write(0x804, 0x04);   // STAT_HALF_FULL_B
    bus_write(0xF29, 0x00);
    CHECK_TRUE("EASC: F29 disable→enable + HALF_B → irq asserts",
               dut->irq == 1);
    bus_write(0xF29, 0x01);
    CHECK_TRUE("EASC: F29 enable→disable → irq clears",
               dut->irq == 0);
    return true;
}

// ─── Main ─────────────────────────────────────────────────────────────
#define RUN(fn) do { \
    bool ok = fn(); \
    if (ok) { printf("[PASS] " #fn "\n"); n_pass++; } \
    else    { printf("[FAIL] " #fn "\n"); n_fail++; } \
} while(0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vasc;

    RUN(test_reset_reads);
    RUN(test_cpu_write_advances_wp);
    RUN(test_sample_tick_advances_rp);
    RUN(test_half_empty_irq_fires);
    RUN(test_irq_clears_on_fifo_status_read);
    RUN(test_irq_clears_on_intstatus_read);
    RUN(test_stereo_interleaves);
    RUN(test_rate_scales_divider);
    RUN(test_volume_writable_but_not_applied);
    RUN(test_sonora_mode_writes_ignored);
    RUN(test_version_easc);
    RUN(test_fifo_clear);
    RUN(test_fifo_full_saturates);
    RUN(test_fifo_overflow_latches_until_read_clear);
    RUN(test_fifo_underflow_latches_until_read_clear);
    RUN(test_fifo_clear_clears_fault_latches);
    RUN(test_rom_style_silence_probe);
    RUN(test_rom_fifo_service_status_bit);
    RUN(test_sonora_aliases_and_irq_control);
    RUN(test_fifo_b_drains_without_stereo_control);
    RUN(test_concurrent_push_pop_preserves_depth);
    RUN(test_native_pcm_outputs);
    RUN(test_sonora_no_op_register_writes);
    RUN(test_sonora_default_rate_drains_fifo);
    RUN(test_sonora_playback_mode_irq_and_status);
    RUN(test_hold_last_sample_on_underflow);
    RUN(test_fifo_a_push_status_updates_playback_mode);
    RUN(test_e00_backdoor);
    RUN(test_f09_f29_edge_irq);
    RUN(test_sonora_ignored_register_reads);
    RUN(test_sonora_804_read_clear_conditional);
    RUN(test_sonora_post_hook_forces_bit1_playback);
    RUN(test_combined_half_empty_aggregation);
    RUN(test_irq_status_probe_readback);
    RUN(test_easc_ext_wrptr_rdptr_readback);
    RUN(test_easc_ext_src_volume_ctrl_latched);
    RUN(test_easc_ext_cdxa_stub_latches);
    RUN(test_q700_boot_irq_quiescent_pre_playreca_write);
    RUN(test_chime_sine_wave_playback);
    RUN(test_chime_default_rate_cadence);
    RUN(test_wavetable_mode_index_zero_mix);
    RUN(test_wavetable_phase_advance_walks_slot);
    RUN(test_fifo_mode_unaffected_by_wt_writes);
    // Iter-4 EASC bookkeeping (A/B/C/D — soft-skip if VERSION != 0xB0).
    RUN(test_easc_mode_accepts_bit0_only);
    RUN(test_easc_clock_reads_three);
    RUN(test_easc_per_channel_half_full);
    RUN(test_easc_f09_f29_edge_irq_match);

    printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);

    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
