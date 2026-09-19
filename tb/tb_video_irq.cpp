// tb_video_irq.cpp — Verilator unit testbench for the DAFB slot-IRQ path
// in rtl/mac/video.v.
//
// Why this exists
// ────────────────────────────────────────────────────────────────────────
// commit 0bc53b3 fixed a real boot-blocking bug: `irq` was being driven
// ONLY from `vblank_pending` gated by an invented enable register at
// +0x1C, and completely ignored `swatch_cursor_pending`.  Per MAME ground
// truth (dafb_base::recalc_ints() in src/mame/apple/dafb.cpp), the DAFB
// asserts its slot-$F IRQ while int_status != 0, where int_status bit0 =
// VBL and bit2 = cursor scanline -- there is no separate enable register,
// each source is armed via SWATCH_CTRL (+0x104) and acked at its own
// address (+0x114 VBL, +0x10C cursor).
//
// Real Mac OS drives this interrupt from the CURSOR source and acks it
// at +0x10C (ROM slot-$F handler at 0x00007574 does `clr.l (0x10C,a0)`
// with a0 = 0xF9800000).  The old RTL could never release the slot line
// because it never even looked at swatch_cursor_pending -- VIA2 PA6
// stayed low forever and the ROM's slot dispatcher re-dispatched without
// end, starving every other interrupt.  tb_dafb.cpp exercises the +0x108
// status register extensively but never once reads `dut->irq` in the
// cursor scenario, so it could not have caught this class of regression.
// This tb closes exactly that gap by asserting on the `irq` OUTPUT PIN
// itself, not just the status mirror.
//
// Scenarios:
//   1. test_cursor_path            — cursor pending -> irq==1 -> ack at
//      +0x10C -> irq==0.  THE load-bearing scenario: this is the exact
//      path real Mac OS uses.
//   2. test_vbl_path               — VBL pending (via frame_tick) ->
//      irq==1 -> ack at +0x114 -> irq==0.
//   3. test_vbl_gating              — SWATCH_CTRL bit0 clear: repeated
//      frame_tick pulses must never raise irq.
//   4. test_cross_ack_independence — cursor pending, THEN a VBL ack at
//      +0x114 must NOT clear it (irq stays 1); only +0x10C clears it.
//      This is the precise regression 0bc53b3 fixed.
//   5. test_disable_clears_pending — VBL pending, then clearing
//      SWATCH_CTRL bit0 must drop irq to 0.
//
// Build via: make tb-video-irq

#include <cstdio>
#include <cstdint>
#include <verilated.h>
#include "Vvideo.h"

static Vvideo*  dut      = nullptr;
static uint64_t sim_time = 0;
static int      n_pass   = 0;
static int      n_fail   = 0;

// ── Clock / reset helpers ───────────────────────────────────────────────
static void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    sim_time++;
}

static void idle_inputs() {
    dut->s_axi_awaddr  = 0;
    dut->s_axi_awvalid = 0;
    dut->s_axi_wdata   = 0;
    dut->s_axi_wstrb   = 0;
    dut->s_axi_wvalid  = 0;
    dut->s_axi_bready  = 0;
    dut->s_axi_araddr  = 0;
    dut->s_axi_arvalid = 0;
    dut->s_axi_rready  = 0;
    dut->frame_tick    = 0;
    dut->scsi0_drq_in  = 0;
}

static void run_cycles(uint32_t cycles) {
    idle_inputs();
    for (uint32_t i = 0; i < cycles; i++)
        tick();
}

// One rising edge on frame_tick (already clk-domain-synchronised, per the
// real instantiation site) -- video.v's vblank_tick fires on this edge
// when SWATCH_CTRL bit0 is set.
static void pulse_frame_tick() {
    idle_inputs();
    dut->frame_tick = 0;
    tick();                  // frame_tick_q settles low
    dut->frame_tick = 1;
    tick();                  // rising edge: vblank_tick fires this cycle
    dut->frame_tick = 0;
    tick();                  // drop back low
}

static void reset() {
    idle_inputs();
    dut->rst = 1;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;
    tick();
}

// ── AXI4-lite host BFM (mirrors tb_dafb.cpp) ────────────────────────────
static int axil_write(uint32_t addr, uint32_t data, uint32_t strb = 0xF) {
    dut->s_axi_awaddr  = addr;
    dut->s_axi_awvalid = 1;
    dut->s_axi_wdata   = data;
    dut->s_axi_wstrb   = strb;
    dut->s_axi_wvalid  = 1;
    dut->s_axi_bready  = 1;

    bool aw_done = false, w_done = false;
    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (!aw_done && dut->s_axi_awready) aw_done = true;
        if (!w_done  && dut->s_axi_wready)  w_done  = true;
        tick();
        if (aw_done) dut->s_axi_awvalid = 0;
        if (w_done)  dut->s_axi_wvalid  = 0;
        if (aw_done && w_done) break;
    }
    if (!aw_done || !w_done) return 1;

    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (dut->s_axi_bvalid) {
            if (dut->s_axi_bresp != 0) {
                std::printf("    FAIL axil_write: BRESP=%d at 0x%03x\n",
                            dut->s_axi_bresp, addr & 0x3FF);
                return 2;
            }
            tick();
            dut->s_axi_bready = 0;
            return 0;
        }
        tick();
    }
    return 3;
}

static int axil_read(uint32_t addr, uint32_t* out) {
    dut->s_axi_araddr  = addr;
    dut->s_axi_arvalid = 1;
    dut->s_axi_rready  = 1;

    bool ar_done = false;
    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (dut->s_axi_arready) { ar_done = true; tick(); break; }
        tick();
    }
    dut->s_axi_arvalid = 0;
    if (!ar_done) return 1;

    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (dut->s_axi_rvalid) {
            if (dut->s_axi_rresp != 0) {
                std::printf("    FAIL axil_read: RRESP=%d at 0x%03x\n",
                            dut->s_axi_rresp, addr & 0x3FF);
                return 2;
            }
            *out = dut->s_axi_rdata;
            tick();
            dut->s_axi_rready = 0;
            return 0;
        }
        tick();
    }
    return 3;
}

// `regs[]` (the 256x32 register-file array) is intentionally NOT cleared
// by a synchronous reset (video.v's reset block: an array-wide reset loop
// would defeat LUTRAM inference -- see the comment there).  That means a
// SWATCH_CTRL enable bit left set by an EARLIER scenario in this same
// process survives `reset()` and, combined with `swatch_cursor_countdown`
// / `vblank_pending` themselves resetting to 0, can immediately re-arm a
// source on the very first post-reset cycle (countdown==0 -> pending<=1,
// or the next frame_tick edge sets vblank_pending, purely off the
// leftover enable bit).  Every scenario below explicitly disarms
// SWATCH_CTRL right after reset() so it starts from a known-clean DAFB
// slate -- exactly what a real debug-reset flow must do too (see the
// video.v reset-block comment on regs[]-backed live state surviving a
// debug-only reset).
static bool disarm_swatch_ctrl() {
    return axil_write(0x104, 0x00000000) == 0;
}

// Program BASE_HI/BASE_LO/STRIDE/PCBR so fb_ready gates true and the
// frame_tick-driven vblank generator can actually arm (mirrors tb_dafb.cpp
// scenario 2's control-latch sequence).  Only needed for VBL scenarios --
// the cursor countdown path does not depend on fb_ready at all.
static bool arm_fb_ready() {
    bool ok = true;
    ok &= (axil_write(0x00, 0x00000000) == 0);   // BASE_HI
    ok &= (axil_write(0x04, 0x00000008) == 0);   // BASE_LO
    ok &= (axil_write(0x08, 0x00000100) == 0);   // STRIDE
    ok &= (axil_write(0x220, 0x00000018) == 0);  // PCBR (8bpp)
    return ok;
}

// The Swatch cursor countdown at cursor_line==0 (the default, unwritten
// REG_SWATCH_CURSOR_LINE state) takes 199521 cycles per
// swatch_cursor_delay_cycles() in video.v.  Run comfortably past that so
// the countdown-driven `swatch_cursor_pending <= 1'b1` transition has
// definitely landed before we sample state.
static const uint32_t CURSOR_COUNTDOWN_CYCLES = 205000;

#define CHECK_EQ(name, got, exp) do { \
    uint32_t _g = (uint32_t)(got); \
    uint32_t _e = (uint32_t)(exp); \
    if (_g != _e) { \
        std::printf("    FAIL %s: got 0x%x, expected 0x%x\n", name, _g, _e); \
        ok = false; \
    } \
} while (0)

#define CHECK_TRUE(name, cond) do { \
    if (!(cond)) { \
        std::printf("    FAIL %s: condition false\n", name); \
        ok = false; \
    } \
} while (0)

// ════════════════════════════════════════════════════════════════════════
// Scenario 1 — cursor path: enable, let countdown expire, irq asserts,
// ack at +0x10C clears it.  THE load-bearing scenario -- the exact path
// real Mac OS uses to drive/ack this interrupt.
// ════════════════════════════════════════════════════════════════════════

// The cursor IRQ must be PERIODIC.  Mac OS arms it once and then relies on it
// firing every frame to run the cursor task; if the countdown fails to reload
// after an ack, the interrupt fires exactly ONCE and the cursor task never
// runs again — the cursor freezes while every other liveness signal (Ticks
// from VIA1, VBL) keeps looking healthy.  test_cursor_path only ever exercised
// a SINGLE fire-and-ack, so it could not catch that.
static bool test_cursor_rearm() {
    bool ok = true;
    reset();
    CHECK_TRUE("disarm leftover SWATCH_CTRL from a prior scenario", disarm_swatch_ctrl());
    CHECK_TRUE("enable SWATCH_CTRL bit2 (cursor) acks",
               axil_write(0x104, 0x00000004) == 0);
    // Three consecutive fire -> ack cycles with no re-arm write in between.
    for (int i = 0; i < 3; i++) {
        run_cycles(CURSOR_COUNTDOWN_CYCLES);
        char msg[96];
        std::snprintf(msg, sizeof(msg), "cursor irq asserts on cycle %d (periodic re-arm)", i + 1);
        CHECK_TRUE(msg, dut->irq == 1);
        std::snprintf(msg, sizeof(msg), "ack cycle %d at +0x10C acks", i + 1);
        CHECK_TRUE(msg, axil_write(0x10C, 0x00000000) == 0);
        std::snprintf(msg, sizeof(msg), "irq drops after ack on cycle %d", i + 1);
        CHECK_TRUE(msg, dut->irq == 0);
    }
    return ok;
}

static bool test_cursor_path() {
    bool ok = true;
    reset();
    CHECK_TRUE("disarm leftover SWATCH_CTRL from a prior scenario", disarm_swatch_ctrl());
    CHECK_TRUE("irq starts low", dut->irq == 0);
    // Enable cursor-scanline interrupt (SWATCH_CTRL bit2), VBL left clear.
    CHECK_TRUE("enable SWATCH_CTRL bit2 (cursor) acks",
               axil_write(0x104, 0x00000004) == 0);
    CHECK_TRUE("irq still low immediately after enable", dut->irq == 0);
    run_cycles(CURSOR_COUNTDOWN_CYCLES);
    uint32_t v = 0;
    CHECK_TRUE("read +0x108 (swatch status) completes", axil_read(0x108, &v) == 0);
    CHECK_EQ("swatch status bit2 (cursor) set after countdown", v, 0x00000004);
    CHECK_TRUE("irq asserts once cursor countdown expires", dut->irq == 1);
    // Ack at +0x10C -- the exact register real Mac OS's slot-$F handler
    // writes (clr.l (0x10C,a0)).
    CHECK_TRUE("ack cursor IRQ at +0x10C acks", axil_write(0x10C, 0x00000000) == 0);
    CHECK_TRUE("irq drops after +0x10C ack", dut->irq == 0);
    CHECK_TRUE("read +0x108 after ack completes", axil_read(0x108, &v) == 0);
    CHECK_EQ("swatch status clears after +0x10C ack", v, 0x00000000);
    return ok;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 2 — VBL path: enable, pulse frame_tick, irq asserts, ack at
// +0x114 clears it.
// ════════════════════════════════════════════════════════════════════════
static bool test_vbl_path() {
    bool ok = true;
    reset();
    CHECK_TRUE("disarm leftover SWATCH_CTRL from a prior scenario", disarm_swatch_ctrl());
    CHECK_TRUE("arm fb_ready (BASE/STRIDE/PCBR)", arm_fb_ready());
    CHECK_TRUE("irq starts low", dut->irq == 0);
    CHECK_TRUE("enable SWATCH_CTRL bit0 (VBL) acks",
               axil_write(0x104, 0x00000001) == 0);
    CHECK_TRUE("irq still low before frame_tick", dut->irq == 0);
    pulse_frame_tick();
    CHECK_TRUE("irq asserts after frame_tick with VBL enabled", dut->irq == 1);
    CHECK_TRUE("ack VBL IRQ at +0x114 acks", axil_write(0x114, 0x00000000) == 0);
    CHECK_TRUE("irq drops after +0x114 ack", dut->irq == 0);
    return ok;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 3 — VBL enable gating: with SWATCH_CTRL bit0 CLEAR, repeated
// frame_tick pulses must never raise irq.
// ════════════════════════════════════════════════════════════════════════
static bool test_vbl_gating() {
    bool ok = true;
    reset();
    // Explicitly force SWATCH_CTRL to a known all-disabled state (rather
    // than relying on "just don't write it") -- `regs[]` is not cleared
    // by reset, so an earlier scenario's leftover enable bit(s) would
    // otherwise leak through and defeat the point of this scenario.
    CHECK_TRUE("disarm SWATCH_CTRL (bit0 clear)", disarm_swatch_ctrl());
    CHECK_TRUE("arm fb_ready (BASE/STRIDE/PCBR)", arm_fb_ready());
    for (int i = 0; i < 5; i++) {
        pulse_frame_tick();
        CHECK_TRUE("irq stays low across frame_tick pulse with VBL disabled",
                   dut->irq == 0);
    }
    uint32_t v = 0;
    CHECK_TRUE("read +0x108 completes", axil_read(0x108, &v) == 0);
    CHECK_EQ("swatch status bit0 never sets while VBL disabled", v & 0x1, 0x0);
    return ok;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 3b — the VBL must survive EVERY PCBR depth code, including the
// ones the AC842 leaves unmapped and the ones our byte-per-pixel scanner
// cannot render.
//
// `fb_ready` in video.v used to include `fb_bpp_reg != 0`, conflating "has
// the ROM programmed a depth" with "is this depth renderable".  Every PCBR
// code that decode_bpp() leaves unmapped (0x04, 0x0C, 0x14) decodes to 0,
// which drove fb_ready low, which killed vblank_tick, which meant
// vblank_pending never set and the DAFB VBL interrupt went permanently DEAD.
// On this platform that is catastrophic: the $0160 bit-6 VBL guard gates ALL
// deferred tasks, so a dead DAFB VBL freezes the cursor and starves level-2
// slot dispatch.
//
// Those codes are reachable, not theoretical: a driver probing for AC842a
// 15bpp support writes a PCBR with bits 2:1 set (0x06 masks to 0x04, 0x16
// masks to 0x14 — see the decode_bpp derivation comment in video.v), so
// merely PROBING for a depth we don't implement would have taken the VBL
// down.  Depth selection and interrupt liveness must be independent.
// ════════════════════════════════════════════════════════════════════════
static bool test_vbl_alive_at_every_depth() {
    bool ok = true;
    // Every distinct value of the PCBR[4:2] depth field, mapped and unmapped,
    // plus the two "probe for 15bpp" writes that set bits 2:1.
    struct DepthCase { uint32_t pcbr; const char* what; };
    const DepthCase cases[] = {
        {0x00, "1bpp  (0x00, mapped)"},
        {0x08, "2bpp  (0x08, mapped)"},
        {0x10, "4bpp  (0x10, mapped)"},
        {0x18, "8bpp  (0x18, mapped)"},
        {0x1c, "24bpp (0x1c, mapped, direct colour)"},
        {0x04, "unmapped code 0x04"},
        {0x0c, "unmapped code 0x0c"},
        {0x14, "unmapped code 0x14"},
        {0x06, "15bpp probe 0x06 (masks to unmapped 0x04)"},
        {0x16, "15bpp probe 0x16 (masks to unmapped 0x14)"},
    };

    for (const auto& c : cases) {
        reset();
        if (!disarm_swatch_ctrl()) { ok = false; break; }
        // Same placement arm as arm_fb_ready(), but with this depth code.
        bool armed = true;
        armed &= (axil_write(0x00,  0x00000000) == 0);  // BASE_HI
        armed &= (axil_write(0x04,  0x00000008) == 0);  // BASE_LO
        armed &= (axil_write(0x08,  0x00000100) == 0);  // STRIDE
        armed &= (axil_write(0x220, c.pcbr)     == 0);  // PCBR (depth)
        CHECK_TRUE("arm placement + depth", armed);

        CHECK_TRUE("enable SWATCH_CTRL bit0 (VBL)",
                   axil_write(0x104, 0x00000001) == 0);
        pulse_frame_tick();
        if (dut->irq != 1) {
            std::printf("    FAIL VBL DEAD at %s: irq=0 after frame_tick "
                        "(fb_ready must not depend on depth renderability)\n",
                        c.what);
            ok = false;
        }
        // And it must still be ackable, i.e. a real pending bit, not a stuck level.
        CHECK_TRUE("ack VBL at +0x114", axil_write(0x114, 0x00000000) == 0);
        if (dut->irq != 0) {
            std::printf("    FAIL VBL not ackable at %s: irq=1 after +0x114\n",
                        c.what);
            ok = false;
        }
    }
    return ok;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 3c — fb_ready must STILL hold the VBL off before the ROM has
// programmed PCBR at all.  This is the gate's actual purpose (hold VBL off
// through early DAFB init); scenario 3b must not have relaxed it into
// "always ready".  Placement is programmed, PCBR deliberately is not.
// ════════════════════════════════════════════════════════════════════════
static bool test_vbl_held_off_before_pcbr() {
    bool ok = true;
    reset();
    CHECK_TRUE("disarm leftover SWATCH_CTRL", disarm_swatch_ctrl());
    // Program BASE/STRIDE but NOT PCBR -- r_pcbr_set stays low.
    CHECK_TRUE("BASE_HI", axil_write(0x00, 0x00000000) == 0);
    CHECK_TRUE("BASE_LO", axil_write(0x04, 0x00000008) == 0);
    CHECK_TRUE("STRIDE",  axil_write(0x08, 0x00000100) == 0);
    CHECK_TRUE("enable SWATCH_CTRL bit0 (VBL)",
               axil_write(0x104, 0x00000001) == 0);
    for (int i = 0; i < 5; i++) {
        pulse_frame_tick();
        CHECK_TRUE("irq stays low before any PCBR write", dut->irq == 0);
    }
    // Now write PCBR and confirm the VBL comes alive -- proves the gate is
    // genuinely tracking r_pcbr_set and not just wedged low.
    CHECK_TRUE("PCBR (8bpp)", axil_write(0x220, 0x00000018) == 0);
    pulse_frame_tick();
    CHECK_TRUE("irq asserts once PCBR has been written", dut->irq == 1);
    return ok;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 4 — cross-ack independence: assert cursor, ack VBL (+0x114)
// must NOT clear it; only +0x10C does.  This is the precise regression
// that caused the real boot hang (fixed by 0bc53b3).
// ════════════════════════════════════════════════════════════════════════
static bool test_cross_ack_independence() {
    bool ok = true;
    reset();
    CHECK_TRUE("disarm leftover SWATCH_CTRL from a prior scenario", disarm_swatch_ctrl());
    CHECK_TRUE("enable SWATCH_CTRL bit2 (cursor) acks",
               axil_write(0x104, 0x00000004) == 0);
    run_cycles(CURSOR_COUNTDOWN_CYCLES);
    CHECK_TRUE("irq asserts once cursor countdown expires", dut->irq == 1);
    // A VBL ack must be a no-op against a pending CURSOR interrupt -- this
    // is exactly the mechanism that wedged real hardware: the old RTL
    // read `irq` purely off vblank_pending, so anything that touched the
    // VBL side (or nothing at all) could never release a cursor-sourced
    // slot line.
    CHECK_TRUE("VBL ack at +0x114 acks", axil_write(0x114, 0x00000000) == 0);
    CHECK_TRUE("irq STILL asserted after unrelated VBL ack (cross-ack independence)",
               dut->irq == 1);
    uint32_t v = 0;
    CHECK_TRUE("read +0x108 after VBL ack completes", axil_read(0x108, &v) == 0);
    CHECK_EQ("cursor status bit2 survives an unrelated VBL ack", v, 0x00000004);
    // Now the correct ack clears it.
    CHECK_TRUE("cursor ack at +0x10C acks", axil_write(0x10C, 0x00000000) == 0);
    CHECK_TRUE("irq clears only once the CORRECT (+0x10C) ack lands", dut->irq == 0);
    return ok;
}

// ════════════════════════════════════════════════════════════════════════
// Scenario 5 — disable-clears-pending: VBL pending, then writing
// SWATCH_CTRL with bit0 clear must drop irq to 0.
// ════════════════════════════════════════════════════════════════════════
static bool test_disable_clears_pending() {
    bool ok = true;
    reset();
    CHECK_TRUE("disarm leftover SWATCH_CTRL from a prior scenario", disarm_swatch_ctrl());
    CHECK_TRUE("arm fb_ready (BASE/STRIDE/PCBR)", arm_fb_ready());
    CHECK_TRUE("enable SWATCH_CTRL bit0 (VBL) acks",
               axil_write(0x104, 0x00000001) == 0);
    pulse_frame_tick();
    CHECK_TRUE("irq asserts after frame_tick with VBL enabled", dut->irq == 1);
    // Clear SWATCH_CTRL bit0 (disable VBL) WITHOUT acking at +0x114 --
    // MAME swatch_w case 0x4: clearing the enable bit also clears
    // int_status bit0 outright.
    CHECK_TRUE("disable SWATCH_CTRL bit0 acks", axil_write(0x104, 0x00000000) == 0);
    CHECK_TRUE("irq drops when SWATCH_CTRL VBL enable clears", dut->irq == 0);
    return ok;
}

// ── Runner ───────────────────────────────────────────────────────────────
struct Scenario {
    const char* name;
    bool (*fn)();
};

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vvideo;

    // video.v's monitor_sense is an input pin (was `parameter
    // MONITOR_TYPE`).  Irrelevant to the IRQ paths this tb exercises, but
    // drive the historical default rather than leaving Verilator's 0 —
    // 0 is a different monitor, and a silently-wrong sense code would be a
    // confusing thing to inherit if this tb ever grows a sense check.
    dut->monitor_sense = 0x06;

    std::printf("── tb_video_irq: DAFB slot-IRQ (VBL | cursor) unit tb ──\n");

    const Scenario scenarios[] = {
        {"test_cursor_path",              test_cursor_path},
        {"test_cursor_rearm",             test_cursor_rearm},
        {"test_vbl_path",                 test_vbl_path},
        {"test_vbl_gating",               test_vbl_gating},
        {"test_vbl_alive_at_every_depth", test_vbl_alive_at_every_depth},
        {"test_vbl_held_off_before_pcbr", test_vbl_held_off_before_pcbr},
        {"test_cross_ack_independence",   test_cross_ack_independence},
        {"test_disable_clears_pending",   test_disable_clears_pending},
    };

    int scenario_pass = 0, scenario_fail = 0;
    for (const auto& s : scenarios) {
        bool ok = s.fn();
        if (ok) {
            n_pass++;
            scenario_pass++;
            std::printf("  [PASS] %s\n", s.name);
        } else {
            n_fail++;
            scenario_fail++;
            std::printf("  [FAIL] %s\n", s.name);
        }
    }

    std::printf("── result: scenarios pass=%d fail=%d (sim_time=%llu) ──\n",
                scenario_pass, scenario_fail,
                (unsigned long long)sim_time);

    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
