// tb_dafb.cpp — Verilator unit testbench for rtl/mac/video.v (DAFB shim)
//
// Exercises the DAFB register-shim in isolation over its AXI4-lite slave
// port.  Scenarios (task #143):
//
//   1. Write-read round-trip at +0x24 (ROM's first DAFB write) hits the
//      last-written register file and survives masked byte updates.
//   2. The ROM's base / stride / BPP latches at +0x08, +0x0C, +0x10
//      round-trip as expected.
//   3. Read of +0x200 (AC842 palette address) returns low-byte state.
//   4. Reads of +0x200/+0x220 return the low byte of the RAMDAC register.
//   5. Test/version at +0x2C reports original DAFB version bits.
//   6. IRQ status behaves like a sticky vblank bit: framebuffer config arms
//      the frame_tick-driven vblank generator (task T8 fix 2 — video.v's
//      `frame_tick` input, pulsed here via pulse_frame_tick() instead of
//      waiting out a free-running counter), unrelated writes do not alias
//      status, the IRQ-enable register gates an observable pending bit,
//      and byte-lane valid write-1-to-clear at +0x20 acknowledges it. The
//      exported IRQ line follows the enabled-pending bit. Base/stride
//      alone do not arm vblank (even across a frame_tick pulse) until the
//      raw depth selector is non-zero.
//   6. DP8531 PLL register-file coverage: the 16 entries at +0x300 + n*0x10
//      all ACK and read back the written value (T9: this window is
//      EXCLUSIVELY the DP8531 pixel-clock PLL nibble file now -- the
//      former "16-entry low-depth CLUT" overlay on the same window was
//      retired; see clut_export scenario below for the real CLUT).
//   6b. clut_export: the AC842 RAMDAC tri-byte write protocol at
//      +0x200/+0x210 drives a one-cycle clut_we/clut_waddr/clut_wdata
//      pulse per completed RGB triple.  Sweeps all 256 entries with
//      distinct RGB values and shadow-models the export in the C++
//      harness (see model_clut / tick()) to prove address + RGB
//      assembly are both correct end to end.  This is the video.v-level
//      counterpart of tb_framebuffer_pixel's pixel-exact 256-entry
//      scanout proof (T9 -- 256-entry RAMDAC CLUT scanout).
//   7. Sweep: walk every 4-byte offset in 0x000..0x3FC and confirm each
//      write + read completes within a finite handshake budget.  Serves
//      as the no-BERR / no-timeout gate.
//   8. Byte-enable: WSTRB=0x2 only updates bits [15:8] of the target.
//   9. Register-window boundary: 0x400 and above ACK deterministically
//      without aliasing into the 1 KB live register file.
//   10. Reset determinism: named control/status registers (fb_base_px,
//      fb_stride_px, fb_bpp_reg, IRQ status, RAMDAC address) clear on
//      reset, and no clut_we pulse fires spuriously across the reset
//      transition.  The two big dynamically-indexed arrays — the raw
//      256x32 register file and the 256x8x3 RAMDAC CLUT — do NOT reset
//      (task T8 fix 1: an array-wide synchronous reset loop defeated
//      LUTRAM inference), so raw last-written values in those two arrays
//      survive a reset by design; this scenario asserts that survival
//      explicitly instead of expecting the old always-zero behavior.
//      (T9: video.v no longer holds a live CLUT array of its own -- the
//      former 16-entry `clut_rgb` flat-bus reset-to-hardcoded-palette
//      check is retired along with that bus; see docs/... T9 brief.)
//
// Build via: make tb-dafb

#include <cstdio>
#include <cstddef>
#include <cstdlib>
#include <cstdint>
#include <verilated.h>
#include "Vvideo.h"

static Vvideo*  dut      = nullptr;
static uint64_t sim_time = 0;
static int      n_pass   = 0;
static int      n_fail   = 0;

#define CHECK(cond, fmt, ...) do { \
    if (cond) { \
        n_pass++; \
        std::printf("  [PASS] " fmt "\n", ##__VA_ARGS__); \
    } else { \
        n_fail++; \
        std::printf("  [FAIL] " fmt "\n", ##__VA_ARGS__); \
    } \
} while (0)

// T9: video.v no longer holds a live 256x24 (or 16x24) CLUT array of its
// own -- the real CLUT BRAM lives downstream in linebuf_scanout.v, fed by
// video.v's clut_we/clut_waddr/clut_wdata export.  This tb builds a
// host-side shadow model of that export (model_clut[]) by sampling the
// pulse every tick() -- see tick() below -- so scenarios can assert what
// the export would have written into the real CLUT without needing the
// scanout RTL in this unit tb.  Zero-initialized to mirror the real
// BRAM's power-on state (bitstream INIT / Verilator zero-init); like the
// RTL's ramdac_clut_r/g/b arrays, this model is intentionally NOT cleared
// by reset() -- see the reset-determinism scenario.
static uint32_t model_clut[256];

static uint32_t live_clut_rgb(int entry) {
    return model_clut[entry & 0xFF];
}

// ── Clock + reset helpers ─────────────────────────────────────────────
static void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    // Shadow-model the CLUT write export: clut_we is a registered
    // single-cycle pulse, valid immediately after the posedge above.
    if (dut->clut_we)
        model_clut[dut->clut_waddr & 0xFF] = dut->clut_wdata & 0xFFFFFFu;
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
}

static void run_cycles(int cycles) {
    idle_inputs();
    for (int i = 0; i < cycles; i++)
        tick();
}

// Drive one rising edge on `frame_tick` — video.v's vblank source is now
// this externally-supplied per-frame tick (task T8 fix 2) instead of a
// free-running 1024-cycle counter.  Mirrors the real instantiation site
// (fpga_top_peripherals.vh), which feeds this port from a core_clk-
// synchronised copy of the DAFB VBL level.
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

// ── AXI4-lite host BFM ─────────────────────────────────────────────────
// Returns 0 on success, 1 on handshake-timeout (guards against livelock
// when testing the no-BERR/no-timeout gate).
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

    // Wait for BVALID; require BRESP == OKAY (0).
    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (dut->s_axi_bvalid) {
            if (dut->s_axi_bresp != 0) {
                std::printf("  [FAIL] axil_write: BRESP=%d at 0x%03x\n",
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

    // Wait for ARREADY.
    bool ar_done = false;
    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (dut->s_axi_arready) { ar_done = true; tick(); break; }
        tick();
    }
    dut->s_axi_arvalid = 0;
    if (!ar_done) return 1;

    // Wait for RVALID.
    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (dut->s_axi_rvalid) {
            if (dut->s_axi_rresp != 0) {
                std::printf("  [FAIL] axil_read: RRESP=%d at 0x%03x\n",
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

// ── Scenarios ─────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vvideo;

    // Monitor sense used to be video.v's `parameter [6:0] MONITOR_TYPE`
    // and is now an input pin (driven on hardware by the CPU debug-CSR
    // OFF_MON_SENSE, over the socket as cpu_mon_sense).  Drive the value
    // the parameter used to default to so every existing expectation in
    // this file — all of which were written against MONITOR_TYPE = 6 —
    // still describes the same DUT.  Deliberately NOT set inside
    // idle_inputs(): it is a standing configuration pin, not a
    // per-transaction one, and the runtime-change scenario below relies on
    // it surviving the idle_inputs() calls that reset()/run_cycles() make.
    dut->monitor_sense = 0x06;

    std::printf("── tb_dafb: DAFB register-shim unit tb ──\n");
    reset();
    CHECK(dut->fb_base_px == 0,   "FB base output resets to 0");
    CHECK(dut->fb_stride_px == 0, "FB stride output resets to 0");
    CHECK(dut->fb_bpp_reg == 0,   "FB BPP output resets to 0");
    CHECK(dut->irq == 0,          "IRQ output resets low");
    // T9: video.v no longer owns a live CLUT array to reset -- confirm no
    // spurious clut_we pulse fires across/after the reset transition
    // instead (the old "resets to hardcoded 16-color palette" check
    // described a mechanism that has been retired -- see file header).
    CHECK(dut->clut_we == 0, "clut_we export stays low immediately after reset");

    uint32_t v;

    // Scenario 1 — +0x24 SCSI bus 1 ctrl (TurboSCSI shim status mirror).
    // Per MAME dafb.cpp:418-419, reads return m_scsi_ctrl[0] | (m_drq[0]
    // << 9), masked to the low 9 bits of the stored ctrl + bit 9.  Writes
    // store the value (DAFB dafb_w masks data to 0xFFF on entry but the
    // ctrl-only effect range is bits[0:8]).  The TurboSCSI shim consumes
    // these bits to gate DTACK on the DMA window.
    CHECK(axil_write(0x24, 0x000001EC) == 0, "write +0x24 <- 0x1EC acks");
    CHECK(axil_read(0x24, &v) == 0,          "read +0x24 completes");
    CHECK(v == 0x000001EC,                    "read +0x24 returns 0x%08x (want 0x1EC)", v);
    // Masked-write covers bits[7:0] (lane 0) + bits[23:16] (lane 2).
    // Lanes 1 and 3 retain prior 0x00 0x00.  After merge, stored low 32
    // bits = 0x00BB01DD, but the shim returns ONLY (& 0x1FF) | (drq<<9):
    // 0x00BB01DD & 0x1FF = 0x1DD; drq=0 → read = 0x000001DD.
    CHECK(axil_write(0x24, 0xAABBCCDD, 0x5) == 0, "masked write +0x24 WSTRB=0x5 acks");
    CHECK(axil_read(0x24, &v) == 0,          "read +0x24 after mask-write completes");
    CHECK(v == 0x000001DD,                   "read +0x24 returns 0x%08x after masked merge (low 9 bits + drq=0)", v);

    // Base/stride can be programmed before depth.  The ROM does this during
    // mode setup; the shim must expose the placement latches but keep the
    // vblank source idle until a non-zero depth selector lands (BPP is
    // derived from the AC842 PCBR register at +0x220, not from the DAFB
    // CONFIG register at +0x10).  Explicitly pulse frame_tick (rather
    // than just letting time pass) to prove the fb_ready gate actually
    // blocks a real per-frame trigger, not just that nothing fires
    // absent one.
    // BASE_HI=0, BASE_LO=8 → m_base = 0x100; STRIDE=0x100 → m_stride = 0x400.
    CHECK(axil_write(0x00, 0x00000000) == 0, "preload BASE_HI before BPP");
    CHECK(axil_write(0x04, 0x00000008) == 0, "preload BASE_LO before BPP");
    CHECK(axil_write(0x08, 0x00000100) == 0, "preload STRIDE before BPP");
    pulse_frame_tick();
    CHECK(axil_read(0x20, &v) == 0,           "read +0x20 with BPP still zero completes");
    CHECK(v == 0x00000000,                    "base/stride alone do not assert vblank status even on frame_tick (0x%08x)", v);

    // Scenario 2 — the ROM's control latches round-trip.  These are the
    // MAME-canonical DAFB register slots the boot trace programs before
    // it moves on to AC842 / Swatch timing.
    struct ControlLatch {
        uint32_t addr;
        uint32_t value;
        const char* label;
    };
    const ControlLatch control_latches[] = {
        {0x00, 0x00000000, "BASE_HI +0x00"},
        {0x04, 0x00000008, "BASE_LO +0x04"},
        {0x08, 0x00000100, "STRIDE +0x08"},
        {0x10, 0x00000030, "CONFIG +0x10"},
        {0x220, 0x00000018, "PCBR +0x220 (8bpp)"},
    };
    for (size_t i = 0; i < sizeof(control_latches) / sizeof(control_latches[0]); i++) {
        CHECK(axil_write(control_latches[i].addr, control_latches[i].value) == 0,
              "write %s <- 0x%08x acks", control_latches[i].label, control_latches[i].value);
        CHECK(axil_read(control_latches[i].addr, &v) == 0,
              "read %s completes", control_latches[i].label);
        CHECK(v == control_latches[i].value,
              "read %s returns 0x%08x", control_latches[i].label, v);
    }
    // m_base = (BASE_HI<<9) | (BASE_LO<<5) = 0|0x100 = 0x100.
    // m_stride = STRIDE<<2 = 0x400.
    // BPP from PCBR bits[4:2] = 0x18 → 8bpp.
    CHECK(dut->fb_base_px == 0x00000100u,
          "FB base output tracks (BASE_HI,BASE_LO) -> 0x%08x", dut->fb_base_px);
    CHECK(dut->fb_stride_px == 0x00000400u,
          "FB stride output tracks STRIDE<<2 -> 0x%08x", dut->fb_stride_px);
    CHECK(dut->fb_bpp_reg == 0x00000008u,
          "FB BPP output tracks PCBR bits[4:2] -> 0x%08x", dut->fb_bpp_reg);

    // Scenario 3 — AC842 palette address at +0x200 is byte-wide.
    CHECK(axil_write(0x200, 0xDEADBEEF) == 0, "write +0x200 <- 0xDEADBEEF acks");
    CHECK(axil_read(0x200, &v) == 0,          "read +0x200 completes");
    CHECK(v == 0x000000EF,                    "read +0x200 returns low byte 0x%08x", v);
    CHECK(axil_write(0x200, 0x00000000, 0x1) == 0, "byte write +0x200 WSTRB=0x1 acks");
    CHECK(axil_read(0x200, &v) == 0,          "read +0x200 after byte write completes");
    CHECK(v == 0x00000000,                    "read +0x200 returns low byte 0x%08x after byte write", v);

    // Scenario 4 — AC842 pixel-bus control at +0x220 is byte-wide.
    CHECK(axil_write(0x220, 0x12345678) == 0, "write +0x220 <- 0x12345678 acks");
    CHECK(axil_read(0x220, &v) == 0,          "read +0x220 completes");
    CHECK(v == 0x00000078,                    "read +0x220 returns low byte 0x%08x", v);

    // Scenario 4b — AC842 RAMDAC tri-byte CLUT read-back protocol.
    // MAME dafb.cpp lines 710-789: write +0x200 latches palette index and
    // resets the R/G/B sub-byte counter; subsequent writes/reads at +0x210
    // walk the R-then-G-then-B sequence; on the third byte the counter
    // wraps to 0 and the palette index auto-increments.
    //
    //   1) Write CLUT[0x42] = (R=0x11, G=0x22, B=0x33) via tri-byte writes.
    //   2) Read it back via tri-byte reads, confirm wrap to next entry.
    //   3) Confirm that reading +0x200 resets the sub-byte counter.
    //   4) Confirm that interleaved writes to +0x200 reset the counter
    //      mid-sequence.
    CHECK(axil_write(0x200, 0x00000042) == 0, "RAMDAC: latch palette idx 0x42");
    CHECK(axil_read(0x200, &v) == 0,          "RAMDAC: read +0x200 returns latched idx");
    CHECK(v == 0x00000042,                    "RAMDAC: +0x200 returns 0x%08x (want 0x42)", v);
    CHECK(axil_write(0x210, 0x00000011) == 0, "RAMDAC: write R=0x11 at idx 0x42");
    CHECK(axil_write(0x210, 0x00000022) == 0, "RAMDAC: write G=0x22 at idx 0x42");
    CHECK(axil_write(0x210, 0x00000033) == 0, "RAMDAC: write B=0x33 at idx 0x42 (wraps to 0x43)");
    // After three writes, ramdac_pal_idx wrapped to 0 and pal_address advanced.
    // Re-latch palette index 0x42 for read-back.
    CHECK(axil_write(0x200, 0x00000042) == 0, "RAMDAC: re-latch palette idx 0x42 for readback");
    CHECK(axil_read(0x210, &v) == 0,          "RAMDAC: read R completes");
    CHECK(v == 0x00000011,                    "RAMDAC: read R=0x%08x (want 0x11)", v);
    CHECK(axil_read(0x210, &v) == 0,          "RAMDAC: read G completes");
    CHECK(v == 0x00000022,                    "RAMDAC: read G=0x%08x (want 0x22)", v);
    CHECK(axil_read(0x210, &v) == 0,          "RAMDAC: read B completes (idx wraps to 0x43)");
    CHECK(v == 0x00000033,                    "RAMDAC: read B=0x%08x (want 0x33)", v);
    // Next read at +0x210 (without re-latching +0x200) auto-walked address
    // to 0x43.  Stage CLUT[0x43] = (R=0xAA, G=0xBB, B=0xCC) via streaming
    // write to verify auto-increment carries across the wrap.
    CHECK(axil_write(0x200, 0x00000043) == 0, "RAMDAC: latch idx 0x43 (reset sub-byte ctr)");
    CHECK(axil_write(0x210, 0x000000AA) == 0, "RAMDAC: stream R=0xAA at idx 0x43");
    CHECK(axil_write(0x210, 0x000000BB) == 0, "RAMDAC: stream G=0xBB at idx 0x43");
    CHECK(axil_write(0x210, 0x000000CC) == 0, "RAMDAC: stream B=0xCC at idx 0x43");
    // Read entries 0x42, 0x43 sequentially without re-latching to confirm
    // address auto-advance on read wrap.
    CHECK(axil_write(0x200, 0x00000042) == 0, "RAMDAC: re-latch 0x42 for sequential read");
    CHECK(axil_read(0x210, &v) == 0 && v == 0x11, "RAMDAC: seq read [0x42].R = 0x%08x", v);
    CHECK(axil_read(0x210, &v) == 0 && v == 0x22, "RAMDAC: seq read [0x42].G = 0x%08x", v);
    CHECK(axil_read(0x210, &v) == 0 && v == 0x33, "RAMDAC: seq read [0x42].B = 0x%08x", v);
    CHECK(axil_read(0x210, &v) == 0 && v == 0xAA, "RAMDAC: seq read [0x43].R after auto-inc = 0x%08x", v);
    CHECK(axil_read(0x210, &v) == 0 && v == 0xBB, "RAMDAC: seq read [0x43].G = 0x%08x", v);
    CHECK(axil_read(0x210, &v) == 0 && v == 0xCC, "RAMDAC: seq read [0x43].B = 0x%08x", v);
    // Confirm reading +0x200 resets the sub-byte counter mid-sequence
    // (MAME dafb.cpp line 717).
    CHECK(axil_write(0x200, 0x00000042) == 0, "RAMDAC: latch 0x42");
    CHECK(axil_read(0x210, &v) == 0 && v == 0x11, "RAMDAC: read R=0x%08x", v);
    // Reading +0x200 here should reset sub-byte counter.
    CHECK(axil_read(0x200, &v) == 0 && v == 0x42,
          "RAMDAC: read +0x200 returns 0x%08x and resets sub-byte ctr", v);
    CHECK(axil_read(0x210, &v) == 0 && v == 0x11,
          "RAMDAC: read after +0x200 read returns R again = 0x%08x (want 0x11)", v);
    // Confirm +0x200 write resets sub-byte counter mid-sequence.
    CHECK(axil_write(0x200, 0x00000042) == 0, "RAMDAC: re-latch 0x42");
    CHECK(axil_read(0x210, &v) == 0 && v == 0x11, "RAMDAC: read R");
    CHECK(axil_read(0x210, &v) == 0 && v == 0x22, "RAMDAC: read G");
    // Now rewrite +0x200 — sub-byte counter must reset to 0.
    CHECK(axil_write(0x200, 0x00000042) == 0, "RAMDAC: rewrite +0x200 mid-seq resets ctr");
    CHECK(axil_read(0x210, &v) == 0 && v == 0x11,
          "RAMDAC: post-rewrite read returns R=0x%08x (want 0x11)", v);
    // Scenario 4c — B&W monitors drive the CLUT from the BLUE byte only.
    //
    // MAME dafb.cpp:760-767: when the monitor code is 1 (Mac Portrait
    // Display, B&W 15") or 3 (Mac Two-Page Display, B&W 21"), ramdac_w
    // DISCARDS the R and G sub-byte writes and, on the third (blue) byte,
    // sets all three pen components to that one value.  Every other
    // monitor code takes the ordinary R/G/B path exercised above.
    //
    // Task #202 context: this is NOT the cause of the 832x624 yellow cast
    // (that value, CLUT[0] = FF/F7/D6, is written by Mac OS itself and MAME
    // writes it byte-for-byte identically — verified against a live MAME
    // 0x6D boot).  It is a separate, real MAME divergence found while
    // auditing this exact write path: before the fix, sense 1/3 produced a
    // colour entry where MAME produces grey.
    {
        const uint8_t saved_sense = 0x06;
        // ---- monitor code 1: Mac Portrait Display (B&W 15") ----
        dut->monitor_sense = 0x01;
        tick();
        CHECK(axil_write(0x200, 0x00000055) == 0, "MONO(1): latch palette idx 0x55");
        CHECK(axil_write(0x210, 0x00000011) == 0, "MONO(1): write R=0x11 (must be DISCARDED)");
        CHECK(axil_write(0x210, 0x00000022) == 0, "MONO(1): write G=0x22 (must be DISCARDED)");
        CHECK(axil_write(0x210, 0x00000077) == 0, "MONO(1): write B=0x77 (replicates to R/G/B)");
        CHECK(axil_write(0x200, 0x00000055) == 0, "MONO(1): re-latch 0x55 for readback");
        CHECK(axil_read(0x210, &v) == 0 && v == 0x77,
              "MONO(1): readback R=0x%08x (want 0x77, the blue byte — not 0x11)", v);
        CHECK(axil_read(0x210, &v) == 0 && v == 0x77,
              "MONO(1): readback G=0x%08x (want 0x77, the blue byte — not 0x22)", v);
        CHECK(axil_read(0x210, &v) == 0 && v == 0x77,
              "MONO(1): readback B=0x%08x (want 0x77)", v);
        CHECK(live_clut_rgb(0x55) == 0x777777u,
              "MONO(1): scanout CLUT export for 0x55 = 0x%06x (want 0x777777)",
              live_clut_rgb(0x55));

        // ---- monitor code 3: Mac Two-Page Display (B&W 21") ----
        dut->monitor_sense = 0x03;
        tick();
        CHECK(axil_write(0x200, 0x00000056) == 0, "MONO(3): latch palette idx 0x56");
        CHECK(axil_write(0x210, 0x000000AA) == 0, "MONO(3): write R=0xAA (must be DISCARDED)");
        CHECK(axil_write(0x210, 0x000000BB) == 0, "MONO(3): write G=0xBB (must be DISCARDED)");
        CHECK(axil_write(0x210, 0x0000002E) == 0, "MONO(3): write B=0x2E (replicates to R/G/B)");
        CHECK(axil_write(0x200, 0x00000056) == 0, "MONO(3): re-latch 0x56 for readback");
        CHECK(axil_read(0x210, &v) == 0 && v == 0x2E,
              "MONO(3): readback R=0x%08x (want 0x2E — not 0xAA)", v);
        CHECK(axil_read(0x210, &v) == 0 && v == 0x2E,
              "MONO(3): readback G=0x%08x (want 0x2E — not 0xBB)", v);
        CHECK(axil_read(0x210, &v) == 0 && v == 0x2E,
              "MONO(3): readback B=0x%08x (want 0x2E)", v);
        CHECK(live_clut_rgb(0x56) == 0x2E2E2Eu,
              "MONO(3): scanout CLUT export for 0x56 = 0x%06x (want 0x2E2E2E)",
              live_clut_rgb(0x56));

        // Address auto-increment must be unaffected by the mono path: the
        // blue write still wraps idx to 0 and advances the entry pointer.
        CHECK(axil_read(0x200, &v) == 0 && v == 0x57,
              "MONO(3): pal_address auto-advanced to 0x%08x (want 0x57)", v);

        // ---- positive control: a COLOUR monitor code still splits R/G/B ----
        // Without this, a fix that greyed out every monitor would pass the
        // two blocks above.  0x6D is the 832x624 16" RGB code from task #202.
        dut->monitor_sense = 0x6D;
        tick();
        CHECK(axil_write(0x200, 0x00000058) == 0, "COLOUR(0x6D): latch palette idx 0x58");
        CHECK(axil_write(0x210, 0x000000FF) == 0, "COLOUR(0x6D): write R=0xFF");
        CHECK(axil_write(0x210, 0x000000F7) == 0, "COLOUR(0x6D): write G=0xF7");
        CHECK(axil_write(0x210, 0x000000D6) == 0, "COLOUR(0x6D): write B=0xD6");
        CHECK(axil_write(0x200, 0x00000058) == 0, "COLOUR(0x6D): re-latch 0x58 for readback");
        CHECK(axil_read(0x210, &v) == 0 && v == 0xFF, "COLOUR(0x6D): readback R=0x%08x (want 0xFF)", v);
        CHECK(axil_read(0x210, &v) == 0 && v == 0xF7, "COLOUR(0x6D): readback G=0x%08x (want 0xF7)", v);
        CHECK(axil_read(0x210, &v) == 0 && v == 0xD6, "COLOUR(0x6D): readback B=0x%08x (want 0xD6)", v);
        // This exact triple is what Mac OS writes for CLUT[0] at sense 0x6D
        // (confirmed against MAME: "PALW addr=00 idx=0/1/2 data=ff/f7/d6").
        // The warm white is AUTHENTIC Apple per-monitor gamma, and the DAFB
        // model must pass it through unmodified.
        CHECK(live_clut_rgb(0x58) == 0xFFF7D6u,
              "COLOUR(0x6D): scanout CLUT export for 0x58 = 0x%06x (want 0xFFF7D6, unmodified)",
              live_clut_rgb(0x58));

        dut->monitor_sense = saved_sense;   // Scenario 6 expects +0x1C == 1
        tick();
    }

    // Leave RAMDAC state in place — downstream IRQ/vblank scenarios do
    // not read or write the RAMDAC, and the FB latches set in Scenario 2
    // must remain so vblank can fire in Scenario 6.

    // Scenario 5 — DAFB test/version keeps test bits [8:0] and reports
    // original discrete DAFB version 1 in bits [10:9].
    CHECK(axil_write(0x2C, 0x00000000) == 0, "write +0x2C <- 0 acks");
    CHECK(axil_read(0x2C, &v) == 0,         "read +0x2C completes");
    CHECK(v == 0x00000200,                  "read +0x2C returns version 0x%08x", v);
    CHECK(axil_write(0x2C, 0x000001ff) == 0, "write +0x2C <- test mask acks");
    CHECK(axil_read(0x2C, &v) == 0,          "read +0x2C test mask completes");
    CHECK(v == 0x000003ff,                   "read +0x2C returns test+version 0x%08x", v);

    // Scenario 6 — IRQ status at +0x20 now behaves like a sticky vblank
    // latch.  Framebuffer base/stride/BPP arm the frame_tick-driven
    // generator, but unrelated DAFB writes must not manufacture status.
    // Bit 1 mirrors the pending latch through IRQ enable bit 0, and a
    // byte-lane-valid write to +0x20 with bit 0 set explicitly
    // acknowledges the latched event.
    CHECK(axil_read(0x20, &v) == 0,           "read +0x20 before vblank completes");
    CHECK(v == 0x00000000,                    "read +0x20 starts at 0x%08x", v);
    CHECK(dut->irq == 0,                       "IRQ output starts low");
    CHECK(axil_write(0x1C, 0x07) == 0,        "write +0x1C <- 0x07 (local IRQ enable) acks");
    CHECK(axil_read(0x1C, &v) == 0,           "read +0x1C (monitor sense) completes");
    CHECK(v == 0x00000001,                    "read +0x1C returns monitor sense 0x%08x", v);
    CHECK(axil_read(0x20, &v) == 0,           "read +0x20 after IRQ-enable write completes");
    CHECK(v == 0x00000000,                    "IRQ-enable write does not set status (0x%08x)", v);
    CHECK(axil_write(0x24, 0x000001ED) == 0,  "write unrelated +0x24 before vblank acks");
    CHECK(axil_read(0x20, &v) == 0,           "read +0x20 (IRQ status) completes");
    CHECK(v == 0x00000000,                    "unrelated +0x24 does not alias status (0x%08x)", v);
    pulse_frame_tick();
    CHECK(axil_read(0x20, &v) == 0,           "read +0x20 after frame_tick-driven vblank completes");
    CHECK(v == 0x00000003,                    "read +0x20 returns 0x%08x after enabled vblank", v);
    CHECK(dut->irq == 1,                       "IRQ output asserts for enabled vblank");
    CHECK(axil_write(0x1C, 0x00000000) == 0,  "disable +0x1C with vblank still pending");
    CHECK(axil_read(0x20, &v) == 0,           "read +0x20 after IRQ disable completes");
    CHECK(v == 0x00000001,                    "IRQ disable drops bit 1 but keeps sticky vblank (0x%08x)", v);
    CHECK(dut->irq == 0,                       "IRQ output drops when enable clears");
    CHECK(axil_write(0x1C, 0x00000001) == 0,  "re-enable +0x1C with vblank pending");
    CHECK(axil_read(0x20, &v) == 0,           "read +0x20 after IRQ re-enable completes");
    CHECK(v == 0x00000003,                    "IRQ re-enable restores observable pending bit (0x%08x)", v);
    CHECK(dut->irq == 1,                       "IRQ output reasserts for still-pending vblank");
    CHECK(axil_write(0x20, 0x00000000) == 0,  "write +0x20 <- 0x00 leaves pending status");
    CHECK(axil_read(0x20, &v) == 0,           "read +0x20 after zero ack completes");
    CHECK(v == 0x00000003,                    "read +0x20 remains 0x%08x after zero ack", v);
    CHECK(axil_write(0x20, 0x00000001, 0xE) == 0,
          "masked write +0x20 with lane 0 disabled does not W1C");
    CHECK(axil_read(0x20, &v) == 0,           "read +0x20 after masked non-W1C completes");
    CHECK(v == 0x00000003,                    "read +0x20 remains 0x%08x after masked non-W1C", v);
    CHECK(axil_write(0x20, 0x00000001) == 0,  "write +0x20 <- 0x01 (status ack) acks");
    CHECK(axil_read(0x20, &v) == 0,           "read +0x20 after ack completes");
    CHECK(v == 0x00000000,                    "read +0x20 returns 0x%08x after ack", v);
    CHECK(dut->irq == 0,                       "IRQ output clears on vblank ack");
    CHECK(axil_write(0x1C, 0x00000000) == 0,  "disable +0x1C before sweep");

    // Scenario 6 — DP8531 PLL nibble register file at +0x300 stride 0x10
    // writes complete cleanly and round-trip.  T9: this window used to
    // double as a "16-entry low-depth CLUT" overlay (low nibble of the
    // write decoded into a hardcoded colour); that overlay is retired --
    // the window is now exclusively the DP8531 pixel-clock PLL register
    // file (see video.v's REG_CLUT_BASE comment).  The real CLUT is
    // exercised by the AC842 RAMDAC sweep directly below.
    for (int i = 0; i < 16; i++) {
        uint32_t addr = 0x300 + (uint32_t)i * 0x10;
        uint32_t want = 0xA5000000u | ((uint32_t)i * 0x00010101u);
        CHECK(axil_write(addr, want) == 0, "write +0x%03x <- 0x%08x (PLL reg %d) acks",
              addr, want, i);
        CHECK(axil_read(addr, &v) == 0, "read +0x%03x (PLL reg %d) completes", addr, i);
        CHECK(v == want, "read +0x%03x returns 0x%08x (PLL reg %d)", addr, v, i);
    }

    // Scenario 6a — clut_export: sweep all 256 AC842 RAMDAC CLUT entries
    // via the real tri-byte write protocol (+0x200 latch, +0x210 x3
    // R/G/B) with distinct RGB values, and confirm the clut_we/waddr/
    // wdata export (shadow-modelled into model_clut[], see tick()) lands
    // the exact entry/RGB for every one of the 256 entries -- including
    // indices above 15, which the retired 16-entry low-depth CLUT could
    // never reach.  This is the video.v-level proof for T9 (256-entry
    // RAMDAC CLUT scanout); tb_framebuffer_pixel.cpp proves the same
    // programming path end-to-end through real scanout pixels.
    for (int i = 0; i < 256; i++) {
        uint8_t r = (uint8_t)(i * 3 + 1);
        uint8_t g = (uint8_t)(i * 5 + 2);
        uint8_t b = (uint8_t)(i * 7 + 3);
        uint32_t want_rgb = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
        CHECK(axil_write(0x200, (uint32_t)i) == 0,
              "RAMDAC: latch palette idx %d", i);
        CHECK(axil_write(0x210, r) == 0, "RAMDAC: write R=0x%02x at idx %d", r, i);
        CHECK(axil_write(0x210, g) == 0, "RAMDAC: write G=0x%02x at idx %d", g, i);
        CHECK(axil_write(0x210, b) == 0, "RAMDAC: write B=0x%02x at idx %d (completes triple)", b, i);
        CHECK(live_clut_rgb(i) == want_rgb,
              "clut_export CLUT[%d] = 0x%06x (want 0x%06x)", i, live_clut_rgb(i), want_rgb);
    }
    // Spot-check a handful of distinct entries survive as a full sweep
    // (not just "last write wins") -- indices 0, 15, 16 (first entry the
    // old 16-entry CLUT could not reach), 128, 255.
    {
        static const int spot[] = {0, 15, 16, 128, 255};
        for (int si = 0; si < 5; si++) {
            int i = spot[si];
            uint8_t r = (uint8_t)(i * 3 + 1);
            uint8_t g = (uint8_t)(i * 5 + 2);
            uint8_t b = (uint8_t)(i * 7 + 3);
            uint32_t want_rgb = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
            CHECK(live_clut_rgb(i) == want_rgb,
                  "clut_export CLUT[%d] retains distinct RGB 0x%06x after full 256-entry sweep",
                  i, live_clut_rgb(i));
        }
    }

    // Scenario 6b - Swatch timing window follows the Q700 DAFB shape:
    // mode/control writes do not read back, while timing/test fields are
    // 12-bit. This matches MAME's model and avoids false ROM probe state.
    CHECK(axil_write(0x100, 0x00000FF2) == 0, "write Swatch mode +0x100 acks");
    CHECK(axil_read(0x100, &v) == 0,          "read Swatch mode +0x100 completes");
    CHECK(v == 0x00000000,                    "Swatch mode read returns 0x%08x", v);
    CHECK(axil_write(0x120, 0xDEADBEEF) == 0, "write Swatch test +0x120 acks");
    CHECK(axil_read(0x120, &v) == 0,          "read Swatch test +0x120 completes");
    CHECK(v == 0x00000EEF,                    "Swatch test read masks to 12 bits: 0x%08x", v);
    CHECK(axil_write(0x124, 0x00001234) == 0, "write Swatch timing +0x124 acks");
    CHECK(axil_read(0x124, &v) == 0,          "read Swatch timing +0x124 completes");
    CHECK(v == 0x00000234,                    "Swatch timing read masks to 12 bits: 0x%08x", v);
    pulse_frame_tick();
    CHECK(axil_read(0x108, &v) == 0,          "read Swatch IRQ status +0x108 completes");
    CHECK(v == 0x00000000,                    "Swatch IRQ status stays 0 while disabled: 0x%08x", v);
    CHECK(axil_write(0x104, 0x00000001) == 0, "enable Swatch VBL at +0x104");
    CHECK(axil_read(0x108, &v) == 0,          "read enabled Swatch IRQ status completes");
    CHECK(v == 0x00000001,                    "enabled Swatch IRQ status sees pending VBL: 0x%08x", v);
    CHECK(axil_write(0x114, 0x00000000) == 0, "clear Swatch VBL at +0x114");
    CHECK(axil_read(0x108, &v) == 0,          "read Swatch IRQ status after clear completes");
    CHECK(v == 0x00000000,                    "Swatch IRQ status clears to 0x%08x", v);
    CHECK(axil_write(0x104, 0x00000000) == 0, "disable Swatch IRQ control");
    CHECK(axil_write(0x118, 0x00000001) == 0, "program Swatch cursor line +0x118");
    CHECK(axil_write(0x104, 0x00000004) == 0, "enable Swatch cursor IRQ at +0x104");
    CHECK(axil_read(0x108, &v) == 0,          "read Swatch cursor status immediately completes");
    CHECK(v == 0x00000000,                    "Swatch cursor status starts clear: 0x%08x", v);
    run_cycles(110000);
    CHECK(axil_read(0x108, &v) == 0,          "read Swatch cursor status after timer completes");
    CHECK(v == 0x00000004,                    "Swatch cursor status sets bit 2: 0x%08x", v);
    CHECK(axil_write(0x10C, 0x00000000) == 0, "clear Swatch cursor IRQ at +0x10C");
    CHECK(axil_read(0x108, &v) == 0,          "read Swatch cursor status after clear completes");
    CHECK(v == 0x00000000,                    "Swatch cursor status clears to 0x%08x", v);

    // Scenario 6c - Swatch programmable timing register storage.
    // Per MAME dafb.cpp:683-706 the DAFB Swatch exposes 10 horizontal +
    // 7 vertical timing registers (12-bit, reset = 0).  Each must hold
    // independent values, mask to 12 bits on read-back, and survive a
    // reset to the all-zero state.  Walking pattern across all 17
    // confirms there's no aliasing into a shared cell.
    struct SwatchTiming {
        uint32_t addr;
        uint32_t value;     // raw write value
        uint32_t expect;    // expected 12-bit-masked read-back
        const char* name;
    };
    const SwatchTiming swatch_timing[] = {
        // Horizontal params (MAME dafb.cpp:683-693, byte 0x24-0x48 ⇒ DAFB byte 0x124-0x148).
        // Distinct walking values across all 17 regs so a misdirected write/read
        // would corrupt the readback of every other register.  Top nibble of each
        // raw write is non-zero so the 12-bit mask is also exercised.
        {0x124, 0xCAFE0A5A, 0x00000A5A, "HSERR"},   // dafb.cpp:683
        {0x128, 0xCAFE0B6B, 0x00000B6B, "HLFLN"},   // dafb.cpp:684
        {0x12C, 0xCAFE0C7C, 0x00000C7C, "HEQ"},     // dafb.cpp:685
        {0x130, 0xCAFE0D8D, 0x00000D8D, "HSP"},     // dafb.cpp:686
        {0x134, 0xCAFE0E9E, 0x00000E9E, "HBWAY"},   // dafb.cpp:687
        {0x138, 0xCAFE0FAF, 0x00000FAF, "HBRST"},   // dafb.cpp:688
        {0x13C, 0xCAFE10C0, 0x000000C0, "HBP"},     // dafb.cpp:689
        {0x140, 0xCAFE11D1, 0x000001D1, "HAL"},     // dafb.cpp:690
        {0x144, 0xCAFE12E2, 0x000002E2, "HFP"},     // dafb.cpp:691
        {0x148, 0xCAFE13F3, 0x000003F3, "HPIX"},    // dafb.cpp:692
        // Vertical params (MAME dafb.cpp:697-703, byte 0x4c-0x64 ⇒ DAFB byte 0x14C-0x164)
        {0x14C, 0xCAFE1404, 0x00000404, "VHLINE"},  // dafb.cpp:697
        {0x150, 0xCAFE1515, 0x00000515, "VSYNC"},   // dafb.cpp:698
        {0x154, 0xCAFE1626, 0x00000626, "VBPEQ"},   // dafb.cpp:699
        {0x158, 0xCAFE1737, 0x00000737, "VBP"},     // dafb.cpp:700
        {0x15C, 0xCAFE1848, 0x00000848, "VAL"},     // dafb.cpp:701
        {0x160, 0xCAFE1959, 0x00000959, "VFP"},     // dafb.cpp:702
        {0x164, 0xCAFE1A6A, 0x00000A6A, "VFPEQ"},   // dafb.cpp:703
    };
    const int n_timing = sizeof(swatch_timing) / sizeof(swatch_timing[0]);
    // Reset first so the prior 6b/sweep writes to 0x124 don't pollute the
    // initial-zero check.  Then each register reads back as 0.
    reset();
    for (int i = 0; i < n_timing; i++) {
        CHECK(axil_read(swatch_timing[i].addr, &v) == 0,
              "read Swatch %s +0x%03x at reset completes",
              swatch_timing[i].name, swatch_timing[i].addr);
        CHECK(v == 0,
              "Swatch %s +0x%03x reads 0 at reset (got 0x%08x)",
              swatch_timing[i].name, swatch_timing[i].addr, v);
    }
    // Walking-pattern write/read round-trip — each value distinct, masked to 12 bits.
    for (int i = 0; i < n_timing; i++) {
        CHECK(axil_write(swatch_timing[i].addr, swatch_timing[i].value) == 0,
              "write Swatch %s +0x%03x <- 0x%08x acks",
              swatch_timing[i].name, swatch_timing[i].addr, swatch_timing[i].value);
    }
    // Read back AFTER all writes to confirm no register aliases another cell.
    for (int i = 0; i < n_timing; i++) {
        CHECK(axil_read(swatch_timing[i].addr, &v) == 0,
              "read Swatch %s +0x%03x after walking write completes",
              swatch_timing[i].name, swatch_timing[i].addr);
        CHECK(v == swatch_timing[i].expect,
              "Swatch %s +0x%03x = 0x%08x (want 12-bit-masked 0x%08x)",
              swatch_timing[i].name, swatch_timing[i].addr, v, swatch_timing[i].expect);
    }
    // Reset must clear every Swatch timing register back to 0.
    reset();
    for (int i = 0; i < n_timing; i++) {
        CHECK(axil_read(swatch_timing[i].addr, &v) == 0,
              "read Swatch %s +0x%03x after reset completes",
              swatch_timing[i].name, swatch_timing[i].addr);
        CHECK(v == 0,
              "Swatch %s +0x%03x clears on reset (got 0x%08x)",
              swatch_timing[i].name, swatch_timing[i].addr, v);
    }

    // Scenario 7 — sweep every word offset 0x000..0x3FC.  Must
    // complete within the handshake budget (no BERR, no timeout).
    int sweep_fails = 0;
    for (uint32_t off = 0; off < 0x400; off += 4) {
        uint32_t pattern = 0xC0DE0000u | off;
        if (axil_write(off, pattern) != 0) { sweep_fails++; continue; }
        uint32_t rb = 0;
        if (axil_read(off, &rb)    != 0) { sweep_fails++; continue; }
        // Named read-overrides: 0x1C (monitor sense), 0x20 (IRQ status),
        // 0x24 (TurboSCSI bus 1 status, low 9 + drq<<9), 0x2C
        // (test/version), 0x100..0x1FF (Swatch), 0x200 (RAMDAC addr,
        // returns m_pal_address), 0x210 (RAMDAC data, tri-byte CLUT
        // protocol — does NOT round-trip last-written), 0x220
        // (RAMDAC PCBR byte).
        // Skip the value check for those (the shim is entitled to
        // shadow the stored value).  Otherwise require round-trip.
        if (off == 0x1C || off == 0x20 || off == 0x24 || off == 0x2C ||
            (off >= 0x100 && off < 0x200) ||
            off == 0x200 || off == 0x210 || off == 0x220) continue;
        if (rb != pattern) {
            std::printf("  [FAIL] sweep +0x%03x: wrote 0x%08x, read 0x%08x\n",
                        off, pattern, rb);
            sweep_fails++;
        }
    }
    CHECK(sweep_fails == 0, "sweep 256 regs: %d mismatches", sweep_fails);

    // Scenario 8 — byte-enable: write WSTRB=0x2 touches bits [15:8] only.
    // Seed the register with 0xDEADBEEF, then masked-write 0xAABBCCDD
    // with WSTRB=0x2 and expect 0xDEADCCEF (lane 1 replaced, rest kept).
    // WSTRB mapping matches the shim: wstrb[i] gates wdata[i*8 +: 8].
    CHECK(axil_write(0x28, 0xDEADBEEF, 0xF) == 0, "seed +0x28 <- 0xDEADBEEF");
    CHECK(axil_write(0x28, 0xAABBCCDD, 0x2) == 0, "masked write WSTRB=0x2 acks");
    CHECK(axil_read(0x28, &v) == 0 && v == 0xDEADCCEFu,
          "byte-enable merge at +0x28 = 0x%08x", v);

    // Scenario 9 - reset determinism.  A board (debug/JTAG) reset clears
    // every NAMED control/status register, but — per task T8 fix 1 — the
    // two big dynamically-indexed arrays (the raw 256x32 register file
    // `regs[]` and the 256x8x3 AC842 RAMDAC CLUT) intentionally do NOT
    // reset, because an array-wide synchronous reset loop defeated
    // LUTRAM inference (see video.v's reset-block comment).  This
    // scenario proves both halves of that contract: derived/named state
    // (fb_base_px/fb_stride_px/fb_bpp_reg, IRQ status, RAMDAC address/idx
    // pointers) clears, while raw last-written bytes in the two big
    // arrays survive.
    CHECK(axil_write(0x24, 0x13579BDFu) == 0, "seed +0x24 before reset");
    CHECK(axil_write(0x08, 0x00001234u) == 0, "seed FB base before reset");
    CHECK(axil_write(0x0C, 0x00005678u) == 0, "seed FB stride before reset");
    CHECK(axil_write(0x10, 0x00000030u) == 0, "seed FB BPP before reset");
    CHECK(axil_write(0x300, 0x0000000Fu) == 0, "seed +0x300 (PLL reg 0) before reset");
    // Seed AC842 RAMDAC CLUT[0].R via the tri-byte protocol so the
    // post-reset RAMDAC-array check below is decisive (entry 0 was never
    // otherwise touched, so it would trivially read 0 either way).
    CHECK(axil_write(0x200, 0x00000000u) == 0, "RAMDAC: latch idx 0 before reset");
    CHECK(axil_write(0x210, 0x000000ABu) == 0, "RAMDAC: seed CLUT[0].R=0xAB before reset");
    reset();
    CHECK(dut->fb_base_px == 0,   "FB base output clears on reset");
    CHECK(dut->fb_stride_px == 0, "FB stride output clears on reset");
    CHECK(dut->fb_bpp_reg == 0,   "FB BPP output clears on reset");
    // T9: no live CLUT array in video.v to check reset-value of; confirm
    // instead that the export stays quiescent across reset (matches
    // scenario 0's check, re-verified here after real traffic).
    CHECK(dut->clut_we == 0, "clut_we export stays low after reset");
    // Raw `regs[]` storage is NOT reset (task T8 fix 1) — these offsets
    // read back the pre-reset seeded value, not 0.  +0x24's read side
    // masks to the low 9 bits (| live scsi0_drq_in<<9, tied 0 here) per
    // the REG_FIRST_HIT override — 0x13579BDF & 0x1FF = 0x1DF.
    CHECK(axil_read(0x24, &v) == 0 && v == 0x000001DFu,
          "read +0x24 after reset retains stale value 0x%08x (LUTRAM not reset by design)", v);
    CHECK(axil_read(0x08, &v) == 0 && v == 0x00001234u,
          "read +0x08 after reset retains stale value 0x%08x (LUTRAM not reset by design)", v);
    CHECK(axil_read(0x0C, &v) == 0 && v == 0x00005678u,
          "read +0x0C after reset retains stale value 0x%08x (LUTRAM not reset by design)", v);
    CHECK(axil_read(0x10, &v) == 0 && v == 0x00000030u,
          "read +0x10 after reset retains stale value 0x%08x (LUTRAM not reset by design)", v);
    CHECK(axil_read(0x300, &v) == 0 && v == 0x0000000Fu,
          "read +0x300 after reset retains stale value 0x%08x (LUTRAM not reset by design)", v);
    // Named IRQ status / RAMDAC address+idx pointers ARE plain scalar
    // registers with an explicit reset, so these still clear to 0.
    CHECK(axil_read(0x20, &v) == 0 && v == 0x00000000u,
          "IRQ status override remains 0 after reset");
    CHECK(axil_read(0x200, &v) == 0 && v == 0x00000000u,
          "RAMDAC address byte resets to 0x%08x", v);
    // But the RAMDAC CLUT array itself (ramdac_clut_r/g/b) is one of the
    // two big arrays that no longer resets — CLUT[0].R retains the
    // pre-reset seed (0xAB), reachable again because ramdac_pal_address
    // itself DID reset back to 0 above.
    CHECK(axil_read(0x210, &v) == 0 && v == 0x000000ABu,
          "RAMDAC data byte (CLUT[0].R) retains stale 0x%08x (LUTRAM not reset by design)", v);

    // Scenario 9b — Monitor sense extended protocol (MAME dafb.cpp:387-415).
    // The chip has a 3-bit sense connector + an extended-drive bit (0x40).
    // Default-code path (no extended drive): reading +0x1C returns
    //   ((MONITOR_TYPE[2:0]) ^ 7).  Default MONITOR_TYPE = 6 (12-14" RGB)
    // → read returns 6^7 = 1.  Already verified above; re-check after a
    // reset to make sure the monitor-id latch returns to zero.
    reset();
    CHECK(axil_read(0x1C, &v) == 0,           "monitor sense: read +0x1C completes after reset");
    CHECK(v == 0x00000001,                    "monitor sense: default code reads 0x%08x (want 0x01 = 6^7)", v);
    // Extended-drive: write bit 6 + a 3-bit drive pattern.  Per MAME
    // dafb.cpp:469-470 m_monitor_id = (data & 0x7) ^ 7.  The default
    // MONITOR_TYPE = 6 has bit 0x40 = 0, so the extended convolution
    // doesn't engage and the read-side falls through to mon[2:0].  In
    // that case ANY drive value should keep the standard response.
    CHECK(axil_write(0x1C, 0x47) == 0,        "monitor sense: write 0x47 (drive ext, low=7)");
    CHECK(axil_read(0x1C, &v) == 0,           "monitor sense: read +0x1C after ext write");
    CHECK(v == 0x00000001,                    "monitor sense: default monitor ignores ext drive (got 0x%08x)", v);
    // Disable extended drive again.  Standard mon[2:0] = 6 → 6^7 = 1.
    CHECK(axil_write(0x1C, 0x07) == 0,        "monitor sense: write 0x07 (no drive)");
    CHECK(axil_read(0x1C, &v) == 0,           "monitor sense: read after no-drive");
    CHECK(v == 0x00000001,                    "monitor sense: standard code returns 0x%08x (want 0x01)", v);
    // Round-trip across the full standard code space: per MAME dafb.cpp
    // line 410, the read-side returns mon[2:0] when bit 6 is clear.
    //
    // Scenario 9b' — monitor sense is RUNTIME-SETTABLE.
    // This used to read "we can't change MONITOR_TYPE at runtime (it's a
    // parameter)".  It is an input pin now, fed on hardware by the CPU
    // debug-CSR at OFF_MON_SENSE (0x0005C) so an operator can sweep sense
    // codes over JTAG instead of paying a ~50-minute bitstream each.  Walk
    // every standard code and require the +0x1C response to track it
    // immediately — no reset, no re-elaboration.
    for (uint32_t code = 0; code < 8; code++) {
        dut->monitor_sense = code;          // bit 6 clear: standard path
        // Purely combinational into the read path; a tick only exists here
        // so the read BFM starts from a settled cycle.
        run_cycles(1);
        CHECK(axil_read(0x1C, &v) == 0,
              "monitor sense: runtime code %u read completes", code);
        CHECK(v == (code ^ 7u),
              "monitor sense: runtime code %u reads 0x%08x (want 0x%02x = %u^7)",
              code, v, code ^ 7u, code);
    }
    // Setting bit 6 (extended monitor) must change the response for the
    // SAME low bits — i.e. all seven bits genuinely reach sense_response(),
    // not just the low three.  With drive pattern m_monitor_id = 4, the
    // ext path ANDs 0b111 with {1, mon[5], mon[4]}; the standard path would
    // just return mon[2:0].  0x5D (tb-dafb-extmon's baseline code) would be
    // the obvious pick, but it is NOT discriminating: bit 6 set gives
    // mon[5:4] = 0b01 → res = 0b101 → read 0b010, and bit 6 clear (0x1D)
    // gives mon[2:0] = 0b101 → read 0b010 as well — the same answer by
    // coincidence.  Use 0x5C / 0x1C instead, where the two paths differ.
    //   0x5C: mon[5:4] = 0b01 → ext res = 0b111 & 0b101 = 0b101 → read 0b010
    //   0x1C: mon[2:0] = 0b100                                  → read 0b011
    CHECK(axil_write(0x1C, 0x03) == 0, "monitor sense: drive m_monitor_id = 4");
    dut->monitor_sense = 0x1C;   // bit 6 clear — standard path
    run_cycles(1);
    CHECK(axil_read(0x1C, &v) == 0, "monitor sense: 0x1C read completes");
    CHECK(v == 0x3u,
          "monitor sense: 0x1C (bit6 clear) reads 0x%08x (want 0x03 = 4^7)", v);
    dut->monitor_sense = 0x5C;   // same low bits, bit 6 SET — ext path
    run_cycles(1);
    CHECK(axil_read(0x1C, &v) == 0, "monitor sense: 0x5C read completes");
    CHECK(v == 0x2u,
          "monitor sense: 0x5C (bit6 set) reads 0x%08x (want 0x02 — bit 6 and "
          "bits[5:4] both reached sense_response)", v);
    // Restore the shipping default for everything downstream.
    dut->monitor_sense = 0x06;
    run_cycles(1);

    // Scenario 9c — DP8531 pixel-clock PLL register write + recompute.
    // MAME dafb.cpp:882-910:
    //   * Writes only commit on offset & 3 == 3 (low byte of the 4-byte
    //     word under big-endian wstrb mapping).
    //   * dp8531_regs[offset>>4] = data & 0xf — 16 nibble registers.
    //   * On reg 15 write, recompute VCO from R / N / P dividers.
    // We expose r_pixel_clock as the pll_pixel_clock output for
    // software / harness introspection.  Reset value = PLL_RESET_HZ
    // = 31_334_400 (MAME dafb.cpp:83 ctor init).
    reset();
    CHECK(dut->pll_pixel_clock == 31334400u,
          "PLL: reset pixel-clock = %u (want 31334400)", dut->pll_pixel_clock);
    // Program a known set of dividers that yield a deterministic VCO.
    // Using R=0x010 (16), N: a=0, b=2, n_modulus = b<<5 | (a^0x1f) = 0x5F.
    // Wait — the equation is: a_pre = (n_modulus & 0x1f) ^ 0x1f;
    // b_pre = (n_modulus & 0xffe0) >> 5; a = min(a_pre, b_pre);
    // b = max(b_pre, 2); N = 32*(b-a) + 31*(1+a).
    // To get a clean N=64: pick a=0, b=2 → N = 32*2 + 31*1 = 95.  Hmm.
    // Pick a=0 b=64 → N = 32*64 + 31*1 = 2079.
    //  R=20: VCO = (20_000_000/20) * 2079 = 1_000_000 * 2079 = 2079_000_000.
    //  P=1 (regs[9]=0): pclk = 2079000000.  Way too large to fit in 32b...
    // Pick a=0 b=2 → N=95 → R=2 → VCO = 10000000*95 = 950_000_000;
    //   P=8 (regs[9]=3) → pclk = 118_750_000. Within range.
    // n_modulus = b<<5 | (a^0x1f) = 2<<5 | 0x1F = 0x5F.
    // regs[0..3] = nibbles of 0x005F: regs[0]=0xF, [1]=0x5, [2]=0x0, [3]=0x0.
    // R = regs[6]<<8 | regs[5]<<4 | regs[4]; want R=2 → regs[4]=0x2,
    //   regs[5]=0, regs[6]=0.
    // P = 1<<regs[9]; want P=8 → regs[9]=3.
    // Reg writes: byte data goes in low nibble of the 32-bit word at
    //   +0x300+n*0x10.
    struct PllReg {
        uint32_t addr;
        uint32_t value;     // raw write — only low nibble matters
        const char* name;
    };
    const PllReg pll_writes[] = {
        // n_modulus low nibble = 0xF (a_pre = 0x00, n_mod[4:0] = 0x1F).
        {0x300, 0x0000000F, "DP8531[0]: n_mod[3:0] = 0xF"},
        {0x310, 0x00000005, "DP8531[1]: n_mod[7:4] = 0x5"},
        {0x320, 0x00000000, "DP8531[2]: n_mod[11:8] = 0x0"},
        {0x330, 0x00000000, "DP8531[3]: n_mod[15:12] = 0x0"},
        {0x340, 0x00000002, "DP8531[4]: R[3:0] = 0x2"},
        {0x350, 0x00000000, "DP8531[5]: R[7:4] = 0x0"},
        {0x360, 0x00000000, "DP8531[6]: R[11:8] = 0x0"},
        {0x370, 0x00000000, "DP8531[7]: spare"},
        {0x380, 0x00000000, "DP8531[8]: spare"},
        {0x390, 0x00000003, "DP8531[9]: P shift = 3 (P=8)"},
        {0x3A0, 0x00000000, "DP8531[10]"},
        {0x3B0, 0x00000000, "DP8531[11]"},
        {0x3C0, 0x00000000, "DP8531[12]"},
        {0x3D0, 0x00000000, "DP8531[13]"},
        {0x3E0, 0x00000000, "DP8531[14]"},
        // Reg 15 triggers recompute.
        {0x3F0, 0x00000000, "DP8531[15]: TRIGGER recompute"},
    };
    const int n_pll = sizeof(pll_writes) / sizeof(pll_writes[0]);
    // Pixel-clock should remain at reset value while registers 0..14 land.
    for (int i = 0; i < n_pll - 1; i++) {
        CHECK(axil_write(pll_writes[i].addr, pll_writes[i].value) == 0,
              "PLL: %s acks", pll_writes[i].name);
        CHECK(dut->pll_pixel_clock == 31334400u,
              "PLL: pclk unchanged before reg-15 write (got %u)",
              dut->pll_pixel_clock);
    }
    // Trigger recompute.
    CHECK(axil_write(pll_writes[n_pll - 1].addr, pll_writes[n_pll - 1].value) == 0,
          "PLL: %s acks", pll_writes[n_pll - 1].name);
    // Verify the recomputed VCO matches the MAME equation literally:
    //   n_modulus = 0x005F.  a_pre = (0x5F & 0x1F) ^ 0x1F = 0x1F ^ 0x1F = 0.
    //                        b_pre = (0x5F & 0xFFE0) >> 5 = 0x40>>5 = 2.
    //                        a = min(0, 2) = 0.  b = max(2, 2) = 2.
    //   N = 32*(2-0) + 31*(1+0) = 64 + 31 = 95.
    //   R = regs[6]<<8 | regs[5]<<4 | regs[4] = 0|0|2 = 2.
    //   P = 1 << regs[9] = 1 << 3 = 8.
    //   VCO = (20000000/2) * 95 = 10000000 * 95 = 950_000_000.
    //   pclk = 950000000 / 8 = 118_750_000.
    CHECK(dut->pll_pixel_clock == 118750000u,
          "PLL: recomputed pclk = %u (want 118750000)",
          dut->pll_pixel_clock);
    // Reset must restore the reset pixel-clock.
    reset();
    CHECK(dut->pll_pixel_clock == 31334400u,
          "PLL: reset clears pclk back to %u",
          dut->pll_pixel_clock);
    // Second program with different dividers — small VCO this time.
    // n_modulus = 0x002F: a_pre = (0x2F & 0x1F) ^ 0x1F = 0x0F ^ 0x1F = 0x10.
    //                     b_pre = (0x2F & 0xFFE0) >> 5 = 0x20>>5 = 1.
    //                     a = min(0x10, 1) = 1.  b = max(1, 2) = 2.
    //                     N = 32*(2-1) + 31*(1+1) = 32 + 62 = 94.
    // R = 4 (regs[4]=4), P = 4 (regs[9]=2).
    //   VCO = (20_000_000/4) * 94 = 5_000_000 * 94 = 470_000_000.
    //   pclk = 470000000 / 4 = 117_500_000.
    const PllReg pll_writes2[] = {
        {0x300, 0x0000000F, "PLL2[0]"},  // n_mod nibble 0
        {0x310, 0x00000002, "PLL2[1]"},  // n_mod nibble 1
        {0x320, 0x00000000, "PLL2[2]"},
        {0x330, 0x00000000, "PLL2[3]"},
        {0x340, 0x00000004, "PLL2[4] R=4"},
        {0x350, 0x00000000, "PLL2[5]"},
        {0x360, 0x00000000, "PLL2[6]"},
        {0x390, 0x00000002, "PLL2[9] P=4"},
        {0x3F0, 0x00000000, "PLL2[15] TRIGGER"},
    };
    const int n_pll2 = sizeof(pll_writes2) / sizeof(pll_writes2[0]);
    for (int i = 0; i < n_pll2; i++) {
        CHECK(axil_write(pll_writes2[i].addr, pll_writes2[i].value) == 0,
              "PLL2: %s acks", pll_writes2[i].name);
    }
    CHECK(dut->pll_pixel_clock == 117500000u,
          "PLL2: recomputed pclk = %u (want 117500000)",
          dut->pll_pixel_clock);

    // Scenario 10 — out-of-window DAFB register accesses above 0x3FF
    // must not alias into the live register file.
    CHECK(axil_write(0x08, 0x00001234u) == 0, "seed +0x08 before boundary probe");
    CHECK(axil_write(0x400, 0xDEADBEEFu) == 0, "write +0x400 (out of range) acks");
    CHECK(axil_write(0xFFC, 0xA5A5A5A5u) == 0, "write +0xFFC (out of range) acks");
    CHECK(axil_read(0x08, &v) == 0 && v == 0x00001234u,
          "low register survives out-of-range writes");
    CHECK(axil_read(0x400, &v) == 0 && v == 0x00000000u,
          "read +0x400 returns zero outside the live window");
    CHECK(axil_read(0xFFC, &v) == 0 && v == 0x00000000u,
          "read +0xFFC returns zero outside the live window");

    // ── Scenario 11 — full AC842 depth matrix ──────────────────────────
    // Every distinct value of the PCBR[4:2] depth field, checked against all
    // three decoded depth outputs at once so they can never drift apart:
    //   fb_bpp_reg      -- bits per pixel (MAME dafb.cpp:791-816)
    //   fb_bytes_per_px -- VRAM bytes per pixel, 0 for the sub-byte depths
    //   depth_supported -- can the scanout datapath render it
    //
    // 24bpp is FOUR VRAM bytes per pixel, not three.  MAME dafb.cpp:340-350
    // (`case 4: // 24 bpp`) walks the framebuffer as an array of u32s, one
    // word per pixel, and uses the word directly as the RGB value; every
    // other mode in that switch reads through the big-endian BYTE cast.  So
    // the layout is xRGB with the pad byte at +0.  A packed-3-byte reading
    // would shear every row.  This row is the regression guard for that.
    //
    // The mapped set is exactly MAME's AC842 switch: 0x00/0x08/0x10/0x18/0x1c.
    // 0x04 / 0x0C / 0x14 are genuinely unmapped in silicon (MAME has no case
    // for them), so they must decode to 0 bpp and be unsupported -- and
    // critically must NOT be "helpfully" filled in as 16bpp.  Real 16bpp
    // (x555) lives on the AC842a behind a PCBR1 condition this chip does not
    // have; see the derivation comment above decode_bpp in rtl/mac/video.v.
    reset();
    {
        struct DepthRow {
            uint32_t pcbr;      // value written to +0x220
            uint32_t bpp;       // expected fb_bpp_reg
            uint32_t bytes;     // expected fb_bytes_per_px
            uint32_t supported; // expected depth_supported
            const char* what;
        };
        const DepthRow rows[] = {
            {0x00, 1,  0, 1, "1bpp  (PCBR 0x00)"},
            {0x08, 2,  0, 1, "2bpp  (PCBR 0x08)"},
            {0x10, 4,  0, 1, "4bpp  (PCBR 0x10)"},
            {0x18, 8,  1, 1, "8bpp  (PCBR 0x18)"},
            {0x1c, 24, 4, 1, "24bpp (PCBR 0x1c, xRGB 4 B/px, direct colour)"},
            {0x04, 0,  0, 0, "unmapped PCBR 0x04"},
            {0x0c, 0,  0, 0, "unmapped PCBR 0x0c"},
            {0x14, 0,  0, 0, "unmapped PCBR 0x14"},
            // 15bpp capability probes: bits 2:1 set.  On an AC842a these
            // would select x555 once PCBR1[7:6]==11; on our AC842 they mask
            // to unmapped codes and must decode to nothing.
            {0x06, 0,  0, 0, "15bpp probe PCBR 0x06 (masks to 0x04)"},
            {0x16, 0,  0, 0, "15bpp probe PCBR 0x16 (masks to 0x14)"},
        };
        for (const auto& r : rows) {
            CHECK(axil_write(0x220, r.pcbr) == 0, "%s: PCBR write acks", r.what);
            CHECK(dut->fb_bpp_reg == r.bpp,
                  "%s: fb_bpp_reg = %u (want %u)", r.what,
                  dut->fb_bpp_reg, r.bpp);
            CHECK(dut->fb_bytes_per_px == r.bytes,
                  "%s: fb_bytes_per_px = %u (want %u)", r.what,
                  (uint32_t)dut->fb_bytes_per_px, r.bytes);
            CHECK((uint32_t)dut->depth_supported == r.supported,
                  "%s: depth_supported = %u (want %u)", r.what,
                  (uint32_t)dut->depth_supported, r.supported);
        }
    }

    // Runtime depth switching: the shim must track PCBR rewrites without any
    // reset in between, in both directions.  Mac OS changes depth live, so a
    // decode that only settled correctly from reset would be useless.
    {
        struct SwitchStep { uint32_t pcbr; uint32_t bpp; uint32_t bytes; };
        const SwitchStep seq[] = {
            {0x18,  8, 1},   // 8bpp
            {0x1c, 24, 4},   // -> 24bpp direct colour
            {0x18,  8, 1},   // -> back to 8bpp
            {0x00,  1, 0},   // -> 1bpp (sub-byte)
            {0x1c, 24, 4},   // -> 24bpp again, from a sub-byte depth
        };
        for (const auto& s : seq) {
            CHECK(axil_write(0x220, s.pcbr) == 0,
                  "runtime switch: PCBR <- 0x%02x acks", s.pcbr);
            CHECK(dut->fb_bpp_reg == s.bpp && dut->fb_bytes_per_px == s.bytes
                      && dut->depth_supported == 1,
                  "runtime switch to PCBR 0x%02x: bpp=%u(want %u) "
                  "bytes_per_px=%u(want %u) supported=%u(want 1)",
                  s.pcbr, dut->fb_bpp_reg, s.bpp,
                  (uint32_t)dut->fb_bytes_per_px, s.bytes,
                  (uint32_t)dut->depth_supported);
        }
    }

    // ── Swatch geometry decode, incl. the AC842 clockdiv term ─────────
    // MAME dafb_base::recalc_mode() (dafb.cpp:817-868): the Swatch
    // horizontal params count in units of `clockdiv` PIXELS, with
    // clockdiv = 1 << ((PCBR & 0x60) >> 5).  Without that term the shim
    // reported 416 for a real 832-wide mode.  Both rows below are register
    // values read off the live Quadra 700 over JTAG.
    {
        struct GeomCase {
            uint32_t hal, hfp, val, vfp, config, pcbr;
            uint32_t hres, vres;
            const char* what;
        };
        const GeomCase cases[] = {
            // mon-sense 0x06, Mac Hi-Res 12-14" 640x480.  clockdiv = 1, so
            // this is the configuration the clockdiv term must not disturb.
            {0x098, 0x318, 0x052, 0x412, 0x030, 0x80, 640, 480,
             "live 640x480 (PCBR 0x80, clockdiv 1)"},
            // mon-sense 0x6D, Apple 16" RGB 832x624.  clockdiv = 2: the raw
            // HFP-HAL is 416 and the visible width is 832.
            {0x08B, 0x22B, 0x052, 0x532, 0x010, 0xA0, 832, 624,
             "live 832x624 (PCBR 0xA0, clockdiv 2)"},
            // Same Swatch params, clockdiv 4 -- proves the term is a real
            // shift and not a hard-coded doubling.
            {0x08B, 0x22B, 0x052, 0x532, 0x010, 0xC0, 1664, 624,
             "832x624 params at clockdiv 4"},
            // Convolution (config bit3): MAME divides instead, then -23.
            {0x08B, 0x22B, 0x052, 0x532, 0x018, 0xA0, 416 / 2 - 23, 624,
             "convolution on: hres /= clockdiv then -23"},
            // Interlace (config bit2) doubles the vertical resolution.
            {0x098, 0x318, 0x052, 0x412, 0x034, 0x80, 640, 960,
             "interlace on: vres doubles"},
        };
        for (const auto& c : cases) {
            reset();
            CHECK(axil_write(0x140, c.hal)    == 0, "%s: HAL write acks", c.what);
            CHECK(axil_write(0x144, c.hfp)    == 0, "%s: HFP write acks", c.what);
            CHECK(axil_write(0x15C, c.val)    == 0, "%s: VAL write acks", c.what);
            CHECK(axil_write(0x160, c.vfp)    == 0, "%s: VFP write acks", c.what);
            CHECK(axil_write(0x010, c.config) == 0, "%s: CONFIG write acks", c.what);
            CHECK(axil_write(0x220, c.pcbr)   == 0, "%s: PCBR write acks", c.what);
            CHECK((uint32_t)dut->hres == c.hres,
                  "%s: hres = %u (want %u)", c.what,
                  (uint32_t)dut->hres, c.hres);
            CHECK((uint32_t)dut->vres == c.vres,
                  "%s: vres = %u (want %u)", c.what,
                  (uint32_t)dut->vres, c.vres);
        }
        reset();
    }

    // Before ANY PCBR write, r_pcbr_set is low: all three depth outputs must
    // read as "nothing programmed", and depth_supported must be low so the
    // scanner holds its elaborated reset placement rather than committing off
    // an undefined depth.
    reset();
    CHECK(dut->fb_bpp_reg == 0,
          "fb_bpp_reg is 0 before any PCBR write");
    CHECK(dut->fb_bytes_per_px == 0,
          "fb_bytes_per_px is 0 before any PCBR write");
    CHECK(dut->depth_supported == 0,
          "depth_supported is low before any PCBR write");

    // ── Summary ────────────────────────────────────────────────────────
    std::printf("── result: pass=%d fail=%d\n", n_pass, n_fail);
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
