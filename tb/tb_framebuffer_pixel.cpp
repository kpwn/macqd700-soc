// tb_framebuffer_pixel.cpp — CPU → VRAM → fb_reader → scaler pixel-exact tb.
//
// Companion to tb_framebuffer_pixel.v.  Programs the real 256-entry AC842
// RAMDAC CLUT (T9 -- see program_clut() below, via the +0x200/+0x210
// tri-byte write protocol) + framebuffer base/stride/BPP via the DAFB
// register shim, writes a pattern into the VRAM slave byte-by-byte, then
// walks one full 64x64 scanout frame capturing scanout_rgb into a 64x64
// RGB888 image.  Asserts each pixel pixel-exact against palette_rgb(pattern
// (x,y)) for the programmed CLUT.  Scenario A/B/C use the original diagonal
// `(x + y) & 0xF` pattern (indices 0-15); scenario D deliberately spans the
// full 0x00-0xFF index range (the retired 16-entry low-depth CLUT could
// only ever reach the low nibble) and scenario E reprograms one entry
// mid-frame.  On FAIL, dumps a PPM of the captured frame and a PPM of the
// expected frame, and prints the first mismatching pixel for bisection.
//
// Why this matters: on HW, only JTAG-poked VRAM has produced a visible
// line so far, and with HDMI_TEST_PATTERN=1 we get rainbow bars (scanout
// chain proven).  With HDMI_TEST_PATTERN=0 we do NOT yet see CPU-written
// pixels.  Sim-side pixel-exact coverage makes any fb_reader steady-state
// regression bisectable before it reaches hardware.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include <verilated.h>
#include "Vtb_framebuffer_pixel.h"

static Vtb_framebuffer_pixel* dut = nullptr;
static uint64_t sim_time = 0;
static bool trace_enabled = false;
static int trace_cycles = 0;
static const int TRACE_MAX = 20000;
static int trace_last_enabled_at_iter = -1000;

static constexpr int FB_W = 64;
static constexpr int FB_H = 64;
static constexpr uint32_t FB_BASE   = 0;
static constexpr uint32_t FB_STRIDE = FB_W;

// DAFB register offsets — MAME-canonical layout, see rtl/mac/video.v's
// "Named register offsets" comment block and the corresponding
// REG_BASE_HI/REG_BASE_LO/REG_STRIDE/REG_CONFIG localparams:
//   +0x00 BASE_HI    dafb_w: m_base bits[20:9]
//   +0x04 BASE_LO    dafb_w: m_base bits[8:5]  (m_base bits[4:0] always 0
//                     — base is 32-pixel aligned)
//   +0x08 STRIDE     dafb_w: m_stride = stored_value << 2
// NOTE: there is NO flat "write base as one 32-bit value" or "write
// stride directly in pixels" register — base/stride must be encoded
// through this split/shifted scheme.  A stray write to +0x10 (CONFIG)
// is NOT how BPP is programmed (BPP comes from the AC842 PCBR register
// at +0x220, entirely separate) — writing here at all is actively
// harmful: CONFIG bit[3] is "convolution", and it FORCES
// fb_stride_px to a hardcoded 1024 regardless of the real STRIDE
// register once set.  This tb hardwires .scanout_hres/.scanout_vres
// (64x64 harness geometry) but takes the DEPTH channel -- bpp_shift /
// fb_bytes_per_px / depth_supported -- live from the DAFB shim, so a PCBR
// write is REQUIRED here: without one r_pcbr_set stays low, depth_supported
// stays low, and scanout_placement_sync refuses to commit any base/stride.
static constexpr uint32_t DAFB_BASE_HI_OFF   = 0x000;
static constexpr uint32_t DAFB_BASE_LO_OFF   = 0x004;
static constexpr uint32_t DAFB_STRIDE_OFF    = 0x008;
// AC842 PCBR (pixel bus control) — bits[4:2] select the pixel depth.
// MAME dafb.cpp:791-816 / rtl/mac/video.v decode_bpp.
static constexpr uint32_t DAFB_RAMDAC_PBCTRL_OFF = 0x220;
static constexpr uint32_t PCBR_1BPP  = 0x00;
static constexpr uint32_t PCBR_8BPP  = 0x18;
static constexpr uint32_t PCBR_24BPP = 0x1C;
// T9: real AC842 RAMDAC CLUT window (256 entries x 24-bit RGB).  Replaces
// the retired 16-entry low-depth CLUT overlay that used to live at
// +0x100 (which was never actually wired to the render path -- see
// program_clut() below).
static constexpr uint32_t DAFB_RAMDAC_ADDR_OFF = 0x200;
static constexpr uint32_t DAFB_RAMDAC_DATA_OFF = 0x210;

// ────────────────────────────────────────────────────────────────────
// Clock plumbing — two independent clocks (pclk / vram_clk) so the
// scanout side stays in its own domain like it does in real HW.
// ────────────────────────────────────────────────────────────────────
static void trace_cycle(const char* tag) {
    if (!trace_enabled) return;
    if (trace_cycles >= TRACE_MAX) return;
    trace_cycles++;
    std::fprintf(stderr,
        "[%s t=%lu] hc=%u vc=%u de=%u "
        "lb:pa=%u lv0=%u dlr=%u ys=%u xs=%u ry=%u rx=%u "
        "pvp1=%u pvp2=%u rst_p=%u fbv2=%u "
        // d= is the full 4-byte read group: [31:24] is the byte at the
        // requested address, the rest only meaningful when 4-byte aligned.
        "fb:en=%u a=%u rdy=%u v=%u d=%08x "
        "vr:en=%u a=%u v=%u d=%08x sde=%u rgb=%06x\n",
        tag, (unsigned long)sim_time,
        (unsigned)dut->hcount, (unsigned)dut->vcount, (unsigned)dut->de_in,
        (unsigned)dut->dbg_prefetch_active, (unsigned)dut->dbg_line_valid0,
        (unsigned)dut->dbg_display_line_ready,
        (unsigned)dut->dbg_y_src, (unsigned)dut->dbg_x_src,
        (unsigned)dut->dbg_rsp_y, (unsigned)dut->dbg_rsp_x,
        (unsigned)dut->dbg_pvp1, (unsigned)dut->dbg_pvp2,
        (unsigned)dut->dbg_restart_pending, (unsigned)dut->dbg_fb_rd_valid2,
        (unsigned)dut->dbg_fb_rd_en, (unsigned)dut->dbg_fb_rd_addr,
        (unsigned)dut->dbg_fb_rd_ready, (unsigned)dut->dbg_fb_rd_valid,
        (unsigned)dut->dbg_fb_rd_data,
        (unsigned)dut->dbg_vram_rd_en, (unsigned)dut->dbg_vram_rd_addr,
        (unsigned)dut->dbg_vram_rd_valid, (unsigned)dut->dbg_vram_rd_data,
        (unsigned)dut->scanout_de, (unsigned)(dut->scanout_rgb & 0xFFFFFF));
}

static void tick_pclk() {
    dut->pclk = 0;
    dut->eval();
    dut->pclk = 1;
    dut->eval();
    sim_time++;
    trace_cycle("PCLK");
}

static void tick_vram() {
    dut->vram_clk = 0;
    dut->eval();
    dut->vram_clk = 1;
    dut->eval();
    sim_time++;
    trace_cycle("VRAM");
}

static void tick_both(int n) {
    for (int i = 0; i < n; i++) {
        tick_pclk();
        tick_vram();
    }
}

// ────────────────────────────────────────────────────────────────────
// Idle / reset.
// ────────────────────────────────────────────────────────────────────
static void idle_inputs() {
    dut->pclk = 0;
    dut->vram_clk = 0;
    dut->dafb_awaddr  = 0;
    dut->dafb_awvalid = 0;
    dut->dafb_wdata   = 0;
    dut->dafb_wstrb   = 0;
    dut->dafb_wvalid  = 0;
    dut->dafb_bready  = 0;
    dut->dafb_araddr  = 0;
    dut->dafb_arvalid = 0;
    dut->dafb_rready  = 0;
    dut->vram_awid    = 0;
    dut->vram_awaddr  = 0;
    dut->vram_awlen   = 0;
    dut->vram_awsize  = 0;
    dut->vram_awburst = 0;
    dut->vram_awvalid = 0;
    for (int i = 0; i < 4; i++) dut->vram_wdata[i] = 0;
    dut->vram_wstrb   = 0;
    dut->vram_wlast   = 0;
    dut->vram_wvalid  = 0;
    dut->vram_bready  = 0;
    dut->vram_arid    = 0;
    dut->vram_araddr  = 0;
    dut->vram_arlen   = 0;
    dut->vram_arsize  = 0;
    dut->vram_arburst = 0;
    dut->vram_arvalid = 0;
    dut->vram_rready  = 1;
    // CPU-side port (scenario-B) — idle until scenario-B drives it.
    dut->vram_cpu_awid    = 0;
    dut->vram_cpu_awaddr  = 0;
    dut->vram_cpu_awlen   = 0;
    dut->vram_cpu_awsize  = 0;
    dut->vram_cpu_awburst = 0;
    dut->vram_cpu_awvalid = 0;
    for (int i = 0; i < 4; i++) dut->vram_cpu_wdata[i] = 0;
    dut->vram_cpu_wstrb   = 0;
    dut->vram_cpu_wlast   = 0;
    dut->vram_cpu_wvalid  = 0;
    dut->vram_cpu_bready  = 0;
    dut->vram_cpu_arid    = 0;
    dut->vram_cpu_araddr  = 0;
    dut->vram_cpu_arlen   = 0;
    dut->vram_cpu_arsize  = 0;
    dut->vram_cpu_arburst = 0;
    dut->vram_cpu_arvalid = 0;
    dut->vram_cpu_rready  = 1;
    dut->hcount       = 0;
    dut->vcount       = 0;
    dut->de_in        = 0;
    dut->hs_in        = 0;
    dut->vs_in        = 0;
}

static void reset_dut() {
    idle_inputs();
    // hcount=1 (NOT 0, idle_inputs()'s default) for the duration of
    // reset_dut()'s own pclk ticks ONLY, so frame_start
    // ((hcount==0)&&(vcount==0)) is LOW while the platform is still
    // resetting.  Immediately after rst releases, video.v's DAFB shim
    // registers sit at THEIR OWN hardware reset defaults (e.g. a
    // stride register whose bits happen to read back as a large,
    // implausible value before the CPU has written anything real) —
    // if frame_start were held high (as a continuous LEVEL, not a
    // once-per-frame pulse) across these pclk edges, that transient
    // default gets a real chance to synchronize through
    // scanout_placement_sync's two-sample-agree filter and COMMIT,
    // a hazard a wide/production-realistic FB_MAX_PIXELS range check
    // (Bug 2 fix) newly exposes (a narrow default range check used to
    // reject that implausible default outright, by coincidence).
    // Restored to hcount=0 before returning: callers rely on hcount=0
    // (frame_start high, matching real HDMI vertical blanking) during
    // their own post-programming tick_both() head-start so
    // linebuf_scanout's sof-triggered prefetch can actually begin
    // before scanout_frame() starts probing pixels.
    dut->hcount = 1;
    dut->rst = 1;
    tick_both(8);
    dut->rst = 0;
    tick_both(4);
    dut->hcount = 0;
}

// ────────────────────────────────────────────────────────────────────
// DAFB AXI-lite write (vram_clk domain).
// ────────────────────────────────────────────────────────────────────
static int dafb_write(uint32_t addr, uint32_t data) {
    dut->dafb_awaddr  = addr;
    dut->dafb_awvalid = 1;
    dut->dafb_wdata   = data;
    dut->dafb_wstrb   = 0xF;
    dut->dafb_wvalid  = 1;
    dut->dafb_bready  = 1;

    bool aw_done = false, w_done = false;
    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (!aw_done && dut->dafb_awready) aw_done = true;
        if (!w_done  && dut->dafb_wready)  w_done  = true;
        tick_vram();
        if (aw_done) dut->dafb_awvalid = 0;
        if (w_done)  dut->dafb_wvalid  = 0;
        if (aw_done && w_done) break;
    }
    if (!aw_done || !w_done) return 1;

    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (dut->dafb_bvalid) {
            int resp = dut->dafb_bresp;
            tick_vram();
            dut->dafb_bready = 0;
            return resp == 0 ? 0 : 2;
        }
        tick_vram();
    }
    return 3;
}

// ────────────────────────────────────────────────────────────────────
// VRAM AXI byte-write: writes one 8-bit pixel at pixel_addr by issuing
// an AXI single-beat word with a single strobe lane lit.  Mirrors the
// wstrb-lane trick from tb_vram_scaler_firstlight.
// ────────────────────────────────────────────────────────────────────
static int vram_write_byte(uint32_t pixel_addr, uint8_t data) {
    uint32_t word_addr = pixel_addr & ~0xFu;
    uint32_t lane      = pixel_addr & 0xFu;

    VlWide<4> wdata;
    for (int i = 0; i < 4; i++) wdata[i] = 0;
    wdata[lane / 4] = uint32_t(data) << ((lane % 4) * 8);

    dut->vram_awid    = 3;
    dut->vram_awaddr  = word_addr;
    dut->vram_awlen   = 0;
    dut->vram_awsize  = 4;
    dut->vram_awburst = 1;
    dut->vram_awvalid = 1;
    dut->vram_wdata   = wdata;
    dut->vram_wstrb   = uint16_t(1u << lane);
    dut->vram_wlast   = 1;
    dut->vram_wvalid  = 1;
    dut->vram_bready  = 1;

    bool aw_done = false, w_done = false;
    for (int i = 0; i < 128; i++) {
        dut->eval();
        if (!aw_done && dut->vram_awready) aw_done = true;
        if (!w_done  && dut->vram_wready)  w_done  = true;
        tick_vram();
        if (aw_done) dut->vram_awvalid = 0;
        if (w_done)  dut->vram_wvalid  = 0;
        if (aw_done && w_done) break;
    }
    if (!aw_done || !w_done) return 1;

    for (int i = 0; i < 128; i++) {
        dut->eval();
        if (dut->vram_bvalid) {
            int resp = dut->vram_bresp;
            tick_vram();
            dut->vram_bready = 0;
            dut->vram_wlast  = 0;
            return resp == 0 ? 0 : 2;
        }
        tick_vram();
    }
    return 3;
}

// ────────────────────────────────────────────────────────────────────
// VRAM AXI 32-bit CPU-style write — emulates exactly what the CPU LSU
// + axi_narrow_to_wide boundary drives onto vram.v's AXI slave for a
// `MOVE.L data, vram_addr` (dcache-bypass path), where the CPU
// simultaneously emits 4 distinct pixel values at pixel positions
// `word_addr + 0..3`.
//
// Ground-truth trace for the big-endian 68k LSU feeding a 128-bit
// AXI slave via axi_narrow_to_wide:
//   LSU:
//     n_awaddr = pixel_word_addr (byte-granular, byte 0 of the 4-pixel
//                 word; 4-byte-aligned by construction)
//     n_wdata  = 0xB0 B1 B2 B3   // byte 0 (lowest addr) in [31:24]
//                                // byte 3 (highest)      in [7:0]
//     n_wstrb  = 0xF
//   narrow_to_wide (aw_lane_q = n_awaddr[3:2]):
//     w_wdata  = {4x n_wdata}             // replicated across all 4 lanes
//     w_wstrb  =  0x000F << (aw_lane_q*4) // only the addressed lane strobed
//     w_awaddr =  aw_addr_aligned         // rounded to 4-byte align (already is)
// This helper reproduces that handshake pattern exactly so the scenario
// is a faithful proxy for a real MOVE.L on the CPU-DC-bypass path.
// (We don't route through the real LSU in this tb — we're verifying
// the VRAM AXI-slave <-> scanout contract, not the LSU.)
// ────────────────────────────────────────────────────────────────────
static int vram_write_long_move_l(uint32_t word_pixel_addr,
                                  uint8_t b0, uint8_t b1,
                                  uint8_t b2, uint8_t b3) {
    // word_pixel_addr MUST be 4-byte-aligned at the pixel-byte level.
    uint32_t base = word_pixel_addr & ~0x3u;
    uint32_t lane = (base >> 2) & 0x3u;    // aw_lane_q = addr[3:2]
    uint32_t word_addr = base & ~0xFu;     // 128-bit beat address
    // Narrow-side 32-bit word as emitted by a big-endian 68k LSU on a
    // MOVE.L: byte 0 (lowest address) in [31:24], byte 3 in [7:0].
    uint32_t n_wdata = (uint32_t(b0) << 24) | (uint32_t(b1) << 16) |
                       (uint32_t(b2) << 8 ) |  uint32_t(b3);

    // After axi_narrow_to_wide: replicate into all 4 lanes.
    VlWide<4> wdata;
    for (int i = 0; i < 4; i++) wdata[i] = n_wdata;

    // After axi_narrow_to_wide: wstrb is 0xF shifted into the selected
    // 32-bit lane.  For lane=0 -> 0x000F, lane=1 -> 0x00F0, etc.
    uint16_t wstrb = uint16_t(0xF) << (lane * 4);

    // Drive the CPU-side AXI port (routes through vram_cpu_byteswap).
    dut->vram_cpu_awid    = 3;
    dut->vram_cpu_awaddr  = word_addr;
    dut->vram_cpu_awlen   = 0;
    dut->vram_cpu_awsize  = 4;  // 128-bit beat (what narrow_to_wide presents)
    dut->vram_cpu_awburst = 1;
    dut->vram_cpu_awvalid = 1;
    dut->vram_cpu_wdata   = wdata;
    dut->vram_cpu_wstrb   = wstrb;
    dut->vram_cpu_wlast   = 1;
    dut->vram_cpu_wvalid  = 1;
    dut->vram_cpu_bready  = 1;

    bool aw_done = false, w_done = false;
    for (int i = 0; i < 128; i++) {
        dut->eval();
        if (!aw_done && dut->vram_cpu_awready) aw_done = true;
        if (!w_done  && dut->vram_cpu_wready)  w_done  = true;
        tick_vram();
        if (aw_done) dut->vram_cpu_awvalid = 0;
        if (w_done)  dut->vram_cpu_wvalid  = 0;
        if (aw_done && w_done) break;
    }
    if (!aw_done || !w_done) return 1;

    for (int i = 0; i < 128; i++) {
        dut->eval();
        if (dut->vram_cpu_bvalid) {
            int resp = dut->vram_cpu_bresp;
            tick_vram();
            dut->vram_cpu_bready = 0;
            dut->vram_cpu_wlast  = 0;
            return resp == 0 ? 0 : 2;
        }
        tick_vram();
    }
    return 3;
}

// ────────────────────────────────────────────────────────────────────
// Expected pattern + palette.
//
// Pattern: `(x + y) & 0xF` — a diagonal that hits every palette index
// uniformly and fails distinctly if any row / column / index mapping
// goes wrong.
// ────────────────────────────────────────────────────────────────────
static uint8_t expected_pixel(int x, int y) {
    return uint8_t((x + y) & 0xF);
}

// T9: full-byte pattern that hits indices ABOVE 15 -- the retired
// 16-entry low-depth CLUT truncated every pixel index to its low nibble
// (pix_data_pipe2[3:0]) before this fix, so this pattern (spanning the
// whole 0x00-0xFF range) renders WRONG on pre-T9 RTL wherever the true
// index differs from its low nibble.  Deliberately avoids (x+y)&0xF-style
// symmetry so it does not accidentally collapse onto a 16-value range.
static uint8_t expected_pixel_full256(int x, int y) {
    return uint8_t((x * 7 + y * 41 + 19) & 0xFF);
}

static uint32_t palette_rgb(uint8_t idx) {
    // Entries 0x0-0xE: the original 16-entry rainbow palette, kept
    // byte-for-byte so scenario A/B/C's (x+y)&0xF pattern renders with
    // the same colours as before.  T9: idx is NO LONGER masked with
    // &0x0F before this switch -- the real 256-entry AC842 RAMDAC CLUT
    // (programmed via program_clut() below) gives every entry 0x00-0xFF
    // a genuinely distinct colour instead of aliasing through a 16-entry
    // low-depth table.  Entry 0x0F and up fall through to the formula in
    // `default` below.
    switch (idx) {
    case 0x0: return 0x000000;
    case 0x1: return 0x800000;
    case 0x2: return 0x008000;
    case 0x3: return 0x808000;
    case 0x4: return 0x000080;
    case 0x5: return 0x800080;
    case 0x6: return 0x008080;
    case 0x7: return 0xC0C0C0;
    case 0x8: return 0x404040;
    case 0x9: return 0xFF0000;
    case 0xA: return 0x00FF00;
    case 0xB: return 0xFFFF00;
    case 0xC: return 0x0000FF;
    case 0xD: return 0xFF00FF;
    case 0xE: return 0x00FFFF;
    default: {
        // 0x0F..0xFF: deterministic, collision-free per-index formula
        // (r == idx alone already guarantees distinctness across the
        // whole range) -- exercises the part of the 256-entry CLUT the
        // retired 16-entry low-depth overlay could never reach.
        uint8_t r = idx;
        uint8_t g = static_cast<uint8_t>(idx ^ 0x5Au);
        uint8_t b = static_cast<uint8_t>(~idx);
        return (static_cast<uint32_t>(r) << 16)
             | (static_cast<uint32_t>(g) << 8)
             | static_cast<uint32_t>(b);
    }
    }
}

// ────────────────────────────────────────────────────────────────────
// MAME-canonical BASE_HI/BASE_LO/STRIDE encoding (see the DAFB register
// offset comment block above).  base_px must be 32-pixel aligned (bits
// [4:0] are not representable); stride_px must be a multiple of 4
// (STRIDE register stores stride>>2).
// ────────────────────────────────────────────────────────────────────
static int dafb_write_base(uint32_t base_px) {
    uint32_t base_hi = (base_px >> 9) & 0xFFF;   // m_base bits [20:9]
    uint32_t base_lo = (base_px >> 5) & 0xF;     // m_base bits [8:5]
    int rc = dafb_write(DAFB_BASE_HI_OFF, base_hi);
    if (rc != 0) return rc;
    return dafb_write(DAFB_BASE_LO_OFF, base_lo);
}

static int dafb_write_stride(uint32_t stride_px) {
    return dafb_write(DAFB_STRIDE_OFF, stride_px >> 2);
}

// ────────────────────────────────────────────────────────────────────
// T9: CLUT programming now goes through the REAL AC842 RAMDAC tri-byte
// write protocol (+0x200 palette-address latch, +0x210 x3 R/G/B), which
// video.v exports as a clut_we/clut_waddr/clut_wdata pulse per completed
// triple straight into linebuf_scanout.v's 256-entry dual-clock BRAM
// CLUT (no more dead writes to a +0x100 Swatch-window offset that never
// reached the render path -- see the retired DAFB_CLUT_BASE_OFF note in
// git history).  Programs the FULL 256-entry table so every possible
// 8bpp pixel value (not just the legacy 16-entry low nibble) renders
// through a real, distinct palette entry.
// ────────────────────────────────────────────────────────────────────
static int program_clut() {
    for (int i = 0; i < 256; i++) {
        uint32_t rgb = palette_rgb(uint8_t(i));
        int rc = dafb_write(DAFB_RAMDAC_ADDR_OFF, uint32_t(i));
        if (rc != 0) return rc;
        rc = dafb_write(DAFB_RAMDAC_DATA_OFF, (rgb >> 16) & 0xFFu);
        if (rc != 0) return rc;
        rc = dafb_write(DAFB_RAMDAC_DATA_OFF, (rgb >> 8) & 0xFFu);
        if (rc != 0) return rc;
        rc = dafb_write(DAFB_RAMDAC_DATA_OFF, rgb & 0xFFu);
        if (rc != 0) return rc;
    }
    return 0;
}

static int dafb_write_pcbr(uint32_t code) {
    return dafb_write(DAFB_RAMDAC_PBCTRL_OFF, code);
}

// ────────────────────────────────────────────────────────────────────
// DAFB programming: 256-entry CLUT (program_clut() above) + FB
// base/stride via the real MAME-canonical BASE_HI/BASE_LO/STRIDE
// registers + the AC842 PCBR depth code.
// ────────────────────────────────────────────────────────────────────
static int program_dafb(uint32_t fb_base = FB_BASE,
                        uint32_t stride  = FB_STRIDE,
                        uint32_t pcbr    = PCBR_8BPP) {
    int rc = program_clut();             if (rc) return rc;
    rc     = dafb_write_pcbr(pcbr);      if (rc) return rc;
    rc     = dafb_write_base(fb_base);   if (rc) return rc;
    rc     = dafb_write_stride(stride);  if (rc) return rc;
    return 0;
}

static int write_pattern(uint32_t fb_base = FB_BASE) {
    for (int y = 0; y < FB_H; y++) {
        for (int x = 0; x < FB_W; x++) {
            uint32_t addr = fb_base + uint32_t(y) * FB_STRIDE + uint32_t(x);
            int rc = vram_write_byte(addr, expected_pixel(x, y));
            if (rc != 0) {
                std::printf("  vram_write_byte(%u,%u) failed: rc=%d\n",
                            x, y, rc);
                return rc;
            }
        }
    }
    return 0;
}

// ────────────────────────────────────────────────────────────────────
// Frame scanout: walk hcount / vcount across the full 64×64 active
// area while also advancing vram_clk so fb_reader can service requests.
// Captures scanout_rgb per DE-high pixel.
//
// ────────────────────────────────────────────────────────────────────
static void scanout_frame(std::vector<uint32_t>& rgb_out) {
    rgb_out.assign(FB_W * FB_H, 0xDEAD);
    // Ramp up a frame: start from vcount=0,hcount=0 with vs asserted,
    // then walk the active region.  Use a tiny blanking interval
    // (2 pclks vs, 2 pclks hs) to exercise frame_start cleanly.
    dut->vs_in = 1;
    dut->hs_in = 0;
    dut->de_in = 0;
    dut->hcount = 0;
    dut->vcount = 0;
    for (int i = 0; i < 4; i++) { tick_pclk(); tick_vram(); }
    dut->vs_in = 0;
    for (int i = 0; i < 2; i++) { tick_pclk(); tick_vram(); }

    // Active region: 64×64 pixels, pclk-synchronous.  After each pclk
    // tick we also drive 2 vram_clks so fb_reader has time to service
    // requests.
    //
    // linebuf_scanout has a 3-pclk output pipeline
    //   x_src/de_in -> line_rd_addr/de_pipe1 -> pix_data_pipe/de_pipe2 ->
    //   rgb/de_out
    // so scanout_de / scanout_rgb sampled on edge N reflect the
    // de_in / pixel asserted on edge N-3.  We sequence captures by
    // counting de_out pulses and storing them back into the (x,y) grid,
    // and tick 3 extra pclks at each row end (with de_in still high)
    // so the last three row pixels drain through before hblank.
    // Mirror firstlight's capture style: walk the full DST_W*DST_H
    // active region with de_in=1 continuously (no tb-side hblank), then
    // flush with de_in=0 to drain the output pipeline.  Captures every
    // de_out pulse sequentially and maps i → (x=i%DST_W, y=i/DST_W).
    //
    // linebuf_scanout's rgb pipeline lags de_out by 1 pclk (x_src FF
    // adds a stage not present in the de_in chain), so the rgb at the
    // i-th de_out pulse holds the pixel for position i-1 (pulse 0 is
    // the priming artifact).  We compensate by assigning pulse i to
    // pix_idx=(i-1) and discarding pulse 0.
    int pix_idx = 0;
    bool first_pulse_seen = false;
    dut->hs_in = 0;
    for (int y = 0; y < FB_H; y++) {
        dut->vcount = uint16_t(y);
        for (int x = 0; x < FB_W; x++) {
            dut->hcount = uint16_t(x);
            dut->de_in  = 1;
            // vram first so the scanout-side read path has fresh data
            tick_vram(); tick_vram();
            tick_pclk();
            if (dut->scanout_de) {
                if (!first_pulse_seen) {
                    first_pulse_seen = true;  // discard priming pulse
                } else {
                    int cx = pix_idx % FB_W;
                    int cy = pix_idx / FB_W;
                    if (cy < FB_H)
                        rgb_out[cy * FB_W + cx] = dut->scanout_rgb & 0xFFFFFFu;
                    pix_idx++;
                }
            }
        }
    }
    // Post-active flush.  Drain the 3-cycle trailer through the output
    // pipeline.  Keep de_in=1 for 2 more pclks with hc=FB_W-1 (in-active)
    // and vc=FB_H-1, so pvp1 doesn't drop until the last row's last
    // pixel has flushed through.  active_line_end may re-fire at hc=63
    // but y_src is already at SRC_H-1 so no advance happens.
    for (int i = 0; i < 2; i++) {
        dut->de_in  = 1;
        dut->hcount = uint16_t(FB_W - 1);
        dut->vcount = uint16_t(FB_H - 1);
        tick_vram(); tick_vram();
        tick_pclk();
        if (dut->scanout_de) {
            if (!first_pulse_seen) { first_pulse_seen = true; }
            else {
                int cx = pix_idx % FB_W;
                int cy = pix_idx / FB_W;
                if (cy < FB_H)
                    rgb_out[cy * FB_W + cx] = dut->scanout_rgb & 0xFFFFFFu;
                pix_idx++;
            }
        }
    }
    dut->de_in = 0;
    for (int i = 0; i < 8; i++) {
        dut->hcount = uint16_t(FB_W + i);
        tick_vram(); tick_vram();
        tick_pclk();
        if (dut->scanout_de) {
            if (!first_pulse_seen) {
                first_pulse_seen = true;
            } else {
                int cx = pix_idx % FB_W;
                int cy = pix_idx / FB_W;
                if (cy < FB_H)
                    rgb_out[cy * FB_W + cx] = dut->scanout_rgb & 0xFFFFFFu;
                pix_idx++;
            }
        }
    }
    dut->de_in = 0;
    dut->vs_in = 1;
    for (int i = 0; i < 8; i++) { tick_pclk(); tick_vram(); }
}

// ────────────────────────────────────────────────────────────────────
// scanout_frame_blanked(): frame walk WITH per-row horizontal blanking.
//
// Why a separate function instead of a parameter on scanout_frame():
// scanout_frame() captures by counting de_out pulses across ONE contiguous
// 4096-pulse burst and exploits the fixed 1-pulse rgb lag (pulse i holds
// pixel i-1) to index the grid.  Introducing per-row DE gaps breaks that
// indexing outright -- each row's final pixel emerges on the cycle AFTER its
// burst ends, i.e. inside the gap where de_out is low.  Rather than perturb
// the capture the pre-existing 8bpp scenarios depend on, this variant
// captures per ROW and drains each row explicitly.
//
// WHY BLANKING IS NEEDED AT ALL: this harness is DST_W == SRC_W == 64 with no
// blanking, so an 8bpp scan gets exactly one fetch slot per displayed pixel --
// exactly enough, and no more.  24bpp needs THREE byte fetches per pixel
// (R/G/B; the +0 pad byte is skipped), so with zero blanking the fetcher falls
// behind 3:1, exhausts its LINE_COUNT=16 line lookahead within ~8 rows and
// latches line_underflow_sticky.  Real 1080p timing carries 2200x1125 pclks
// per frame against a 1024x768 source; `hblank_pclks` is how this tb models
// that slack instead of pretending it does not exist.
//
// Per-row sequence:
//   1. hblank_pclks x (de_in=0, hcount past active) -- fetcher catches up.
//      The row epilogue below has already drained the pipeline, so no de_out
//      pulses are expected here; any that appear are counted and discarded.
//   2. 64 x (de_in=1, hcount=x) -- the active walk.
//   3. ROW_DRAIN x (de_in=1) to flush the row's tail out of the 5-stage
//      output pipeline.  de_in must stay HIGH throughout or those pixels
//      emerge on cycles where de_out is already low and are simply lost.
//      hcount is stepped in two stages, and both are load-bearing:
//
//        tick 0:      hcount = FB_W-2, i.e. still INSIDE the active window.
//          x_src trails hcount by one cycle, so the row's last source pixel
//          (x_src == 63) is only presented to line_mem on the cycle AFTER
//          the final active tick.  rd_req must still be high then or
//          pix_valid_pipe1 is 0 for it and the pixel renders black.
//          FB_W-2 rather than FB_W-1 because at FB_W-1 `active_line_end`
//          re-fires and advances y_src a second time, skipping a source row.
//
//        ticks 1..:   hcount = FB_W, i.e. just PAST the active window, so
//          in_border=1 and rd_req drops.  Required: by then y_src_pipe1 has
//          advanced to row y+1, which the fetcher legitimately has not
//          reached yet, so leaving rd_req high would trip linebuf's
//          `display_primed && rd_req && !display_line_ready` check.  Those
//          are harness-manufactured events, not scanout starvation (the
//          captured frame is pixel-exact either way) -- and this tb asserts
//          on that check, so it must not manufacture them.
//
// That yields 64+ROW_DRAIN de_in-high cycles per row, of which the last few
// pulses land in the following hblank; pulses[1..64] are pixels 0..63.
// ────────────────────────────────────────────────────────────────────
static int scanout_underflow_events = 0;   // counted per scanout_frame_blanked

static void scanout_frame_blanked(std::vector<uint32_t>& rgb_out,
                                  int hblank_pclks) {
    static constexpr int ROW_DRAIN = 8;   // > the 5-stage output pipeline
    rgb_out.assign(FB_W * FB_H, 0xDEAD);
    scanout_underflow_events = 0;

    dut->vs_in = 1;
    dut->hs_in = 0;
    dut->de_in = 0;
    dut->hcount = 0;
    dut->vcount = 0;
    for (int i = 0; i < 4; i++) { tick_pclk(); tick_vram(); }
    dut->vs_in = 0;
    for (int i = 0; i < 2; i++) { tick_pclk(); tick_vram(); }

    std::vector<uint32_t> pulses;
    for (int y = 0; y < FB_H; y++) {
        for (int hb = 0; hb < hblank_pclks; hb++) {
            dut->de_in  = 0;
            dut->hcount = uint16_t(FB_W + (hb & 0x7));
            tick_vram(); tick_vram();
            tick_pclk();
        if (dut->dbg_line_underflow_ev) scanout_underflow_events++;
        }
        pulses.clear();
        dut->vcount = uint16_t(y);
        for (int x = 0; x < FB_W; x++) {
            dut->hcount = uint16_t(x);
            dut->de_in  = 1;
            tick_vram(); tick_vram();
            tick_pclk();
        if (dut->dbg_line_underflow_ev) scanout_underflow_events++;
            if (dut->scanout_de)
                pulses.push_back(dut->scanout_rgb & 0xFFFFFFu);
        }
        for (int d = 0; d < ROW_DRAIN; d++) {
            dut->hcount = uint16_t((d == 0) ? (FB_W - 2) : FB_W);
            dut->de_in  = 1;
            tick_vram(); tick_vram();
            tick_pclk();
        if (dut->dbg_line_underflow_ev) scanout_underflow_events++;
            if (dut->scanout_de)
                pulses.push_back(dut->scanout_rgb & 0xFFFFFFu);
        }
        for (int x = 0; x < FB_W; x++) {
            size_t p = size_t(x) + 1;   // pulse 0 is the 1-cycle rgb lag
            rgb_out[y * FB_W + x] = (p < pulses.size()) ? pulses[p] : 0xDEADu;
        }
    }

    dut->de_in = 0;
    dut->vs_in = 1;
    for (int i = 0; i < 8; i++) { tick_pclk(); tick_vram(); }
}

// ────────────────────────────────────────────────────────────────────
// T9: same capture loop as scanout_frame() above, but issues a real
// AC842 RAMDAC write (video.v's clk is vram_clk in this tb, so this only
// advances vram_clk -- pclk stays frozen mid-scan, exactly like the
// async CDC-free write path in the real design) partway through the
// active region, at the start of row `inject_at_row`.  Used to prove a
// CLUT reprogram lands within the SAME frame with no frame-atomic
// buffering and no scanout wedge (T9 "mid-frame CLUT writes ... allowed
// to take effect immediately" requirement).
// ────────────────────────────────────────────────────────────────────
static void scanout_frame_with_midframe_write(std::vector<uint32_t>& rgb_out,
                                              int inject_at_row,
                                              uint8_t clut_idx,
                                              uint32_t new_rgb) {
    rgb_out.assign(FB_W * FB_H, 0xDEAD);
    dut->vs_in = 1;
    dut->hs_in = 0;
    dut->de_in = 0;
    dut->hcount = 0;
    dut->vcount = 0;
    for (int i = 0; i < 4; i++) { tick_pclk(); tick_vram(); }
    dut->vs_in = 0;
    for (int i = 0; i < 2; i++) { tick_pclk(); tick_vram(); }

    int pix_idx = 0;
    bool first_pulse_seen = false;
    bool injected = false;
    dut->hs_in = 0;
    for (int y = 0; y < FB_H; y++) {
        if (!injected && y == inject_at_row) {
            dafb_write(DAFB_RAMDAC_ADDR_OFF, uint32_t(clut_idx));
            dafb_write(DAFB_RAMDAC_DATA_OFF, (new_rgb >> 16) & 0xFFu);
            dafb_write(DAFB_RAMDAC_DATA_OFF, (new_rgb >> 8) & 0xFFu);
            dafb_write(DAFB_RAMDAC_DATA_OFF, new_rgb & 0xFFu);
            injected = true;
        }
        dut->vcount = uint16_t(y);
        for (int x = 0; x < FB_W; x++) {
            dut->hcount = uint16_t(x);
            dut->de_in  = 1;
            tick_vram(); tick_vram();
            tick_pclk();
            if (dut->scanout_de) {
                if (!first_pulse_seen) {
                    first_pulse_seen = true;
                } else {
                    int cx = pix_idx % FB_W;
                    int cy = pix_idx / FB_W;
                    if (cy < FB_H)
                        rgb_out[cy * FB_W + cx] = dut->scanout_rgb & 0xFFFFFFu;
                    pix_idx++;
                }
            }
        }
    }
    for (int i = 0; i < 2; i++) {
        dut->de_in  = 1;
        dut->hcount = uint16_t(FB_W - 1);
        dut->vcount = uint16_t(FB_H - 1);
        tick_vram(); tick_vram();
        tick_pclk();
        if (dut->scanout_de) {
            if (!first_pulse_seen) { first_pulse_seen = true; }
            else {
                int cx = pix_idx % FB_W;
                int cy = pix_idx / FB_W;
                if (cy < FB_H)
                    rgb_out[cy * FB_W + cx] = dut->scanout_rgb & 0xFFFFFFu;
                pix_idx++;
            }
        }
    }
    dut->de_in = 0;
    for (int i = 0; i < 8; i++) {
        dut->hcount = uint16_t(FB_W + i);
        tick_vram(); tick_vram();
        tick_pclk();
        if (dut->scanout_de) {
            if (!first_pulse_seen) {
                first_pulse_seen = true;
            } else {
                int cx = pix_idx % FB_W;
                int cy = pix_idx / FB_W;
                if (cy < FB_H)
                    rgb_out[cy * FB_W + cx] = dut->scanout_rgb & 0xFFFFFFu;
                pix_idx++;
            }
        }
    }
    dut->de_in = 0;
    dut->vs_in = 1;
    for (int i = 0; i < 8; i++) { tick_pclk(); tick_vram(); }
}

// ────────────────────────────────────────────────────────────────────
// PPM dump (for failure investigation).
// ────────────────────────────────────────────────────────────────────
static void dump_ppm(const char* path, const std::vector<uint32_t>& rgb) {
    FILE* f = std::fopen(path, "w");
    if (!f) return;
    std::fprintf(f, "P3\n%d %d\n255\n", FB_W, FB_H);
    for (int y = 0; y < FB_H; y++) {
        for (int x = 0; x < FB_W; x++) {
            uint32_t v = rgb[y * FB_W + x];
            std::fprintf(f, "%u %u %u ",
                         (v >> 16) & 0xFF,
                         (v >> 8)  & 0xFF,
                         v & 0xFF);
        }
        std::fprintf(f, "\n");
    }
    std::fclose(f);
}

// ────────────────────────────────────────────────────────────────────
// Main.
// ────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_framebuffer_pixel;

    std::printf("tb_framebuffer_pixel: CPU -> VRAM -> fb_reader -> scaler pixel-exact\n");
    std::printf("  geometry: %dx%d @ 8bpp, 1:1, no border\n", FB_W, FB_H);

    reset_dut();

    if (int rc = program_dafb()) {
        std::printf("[FAIL] DAFB programming rc=%d\n", rc);
        return 1;
    }
    std::printf("  DAFB programmed: base=0x%x stride=%d bpp=8 (readback base=0x%x stride=%u)\n",
                FB_BASE, FB_STRIDE, dut->fb_base_px_out, dut->fb_stride_px_out);

    if (int rc = write_pattern()) {
        std::printf("[FAIL] VRAM write rc=%d\n", rc);
        return 1;
    }
    std::printf("  VRAM filled with (x+y)&0xF diagonal pattern\n");

    // Trace disabled — full-frame capture is too large for stderr.
    // Re-enable locally to debug a specific regression.
    trace_enabled = false;
    (void)trace_last_enabled_at_iter;  // silence unused-var

    // Drain enough vram+pclk cycles so the scanout prefetch can complete
    // the first buffered source line before scanout begins.  With a
    // 64-pixel line and ~2 vram_clks per pclk in this harness, give
    // ~90 cycles of head-start (comfortably >64 so line 0 ingress
    // finishes; matches real HDMI vertical blanking which is hundreds
    // of lines long).
    tick_both(90);

    std::vector<uint32_t> captured;
    scanout_frame(captured);

    // Build the expected frame.  We know each palette index's RGB
    // because we programmed the CLUT ourselves via DAFB, and
    // palette_rgb() was the source of those writes.  If the shim
    // swaps entries (endianness in the 384-bit bundle), this assert
    // will fail with a clear diagnostic — don't paper over.
    std::vector<uint32_t> expected(FB_W * FB_H, 0);
    for (int y = 0; y < FB_H; y++) {
        for (int x = 0; x < FB_W; x++) {
            expected[y * FB_W + x] =
                palette_rgb(expected_pixel(x, y)) & 0xFFFFFFu;
        }
    }

    int mismatches = 0;
    int first_x = -1, first_y = -1;
    uint32_t first_got = 0, first_exp = 0;
    for (int y = 0; y < FB_H; y++) {
        for (int x = 0; x < FB_W; x++) {
            uint32_t got = captured[y * FB_W + x];
            uint32_t exp = expected[y * FB_W + x];
            if (got != exp) {
                if (mismatches == 0) {
                    first_x = x; first_y = y;
                    first_got = got; first_exp = exp;
                }
                mismatches++;
            }
        }
    }

    bool underflow = dut->fb_underflow_sticky != 0;

    std::printf("  captured pixels: %d / %d match, %d mismatch\n",
                FB_W * FB_H - mismatches, FB_W * FB_H, mismatches);
    std::printf("  fb_underflow_sticky: %d (fb_reader_ovfl=%d line_underflow=%d)\n",
                underflow, (int)dut->dbg_fb_reader_underflow,
                (int)dut->dbg_line_underflow);

    if (mismatches > 0 || underflow) {
        if (mismatches > 0) {
            std::printf("[FAIL] first mismatch at (%d,%d): got=0x%06X expected=0x%06X\n",
                        first_x, first_y, first_got, first_exp);
        }
        if (underflow) {
            std::printf("[FAIL] fb_reader or linebuf underflowed during scanout\n");
        }
        dump_ppm("build/framebuffer_pixel/captured.ppm", captured);
        dump_ppm("build/framebuffer_pixel/expected.ppm", expected);
        std::printf("  PPMs dumped to build/framebuffer_pixel/{captured,expected}.ppm\n");
        delete dut;
        return 1;
    }

    std::printf("[PASS] scenario-A (byte-granular): %dx%d pixel-exact\n",
                FB_W, FB_H);

    // Drain the scanout pipeline (scenario-A's capture may have left
    // linebuf/fb_reader mid-frame) then full reset of the scanout + VRAM
    // slave + fb_reader + DAFB so scenario-B starts in a clean state.
    // VRAM memory contents survive rst (xpm tdpram / behavioural array
    // don't clear on rst), so the scenario-A pattern stays in URAM until
    // scenario-B overwrites it word-by-word — that's fine since
    // scenario-B writes every VRAM word.
    dut->de_in = 0;
    dut->vs_in = 1;
    dut->hs_in = 0;
    tick_both(64);
    reset_dut();
    // Extra drain after rst to let any residual linebuf state clear
    // before DAFB is re-programmed.
    tick_both(32);
    if (int rc = program_dafb()) {
        std::printf("[FAIL] DAFB re-programming (scenario-B) rc=%d\n", rc);
        delete dut;
        return 1;
    }

    // ══════════════════════════════════════════════════════════════════
    // Scenario B — MOVE.L-style 4-pixels-per-write (audit repro).
    //
    // Overwrites VRAM with a pattern driven as 32-bit AXI writes where
    // each 32-bit word carries four DISTINCT pixel values at
    // `4k + 0..3`.  Per big-endian 68k LSU convention:
    //   byte 0 (lowest address, pixel at 4k+0) -> wdata[31:24]
    //   byte 1 (4k+1)                          -> wdata[23:16]
    //   byte 2 (4k+2)                          -> wdata[15:8]
    //   byte 3 (4k+3)                          -> wdata[7:0]
    // after axi_narrow_to_wide replication + wstrb-lane gating.
    //
    // If the audit's mirror claim is correct, scanout will show each
    // 4-pixel group in reverse order (pixel 4k+0 reads the byte
    // intended for 4k+3, etc.).  If scenario B passes, the mirror was
    // not reproducible — our LSU + narrow_to_wide + VRAM endian chain
    // is self-consistent end-to-end.
    // ══════════════════════════════════════════════════════════════════
    std::printf("\nscenario-B: MOVE.L-style 4-pixels-per-write (audit repro)\n");

    // Fresh pattern: at pixel position p (0..FB_W*FB_H-1), store a byte
    // equal to (p & 0xF) — a simple column-varying sweep so that every
    // 4-pixel group contains 4 DISTINCT distinguishable values at
    // positions (p%4 = 0, 1, 2, 3) = (p, p+1, p+2, p+3) & 0xF which
    // are 4 consecutive palette indices, always distinct.
    auto scenario_b_pixel = [](int x, int y) -> uint8_t {
        int p = y * FB_W + x;
        return uint8_t(p & 0xF);
    };

    // Write one 4-pixel word at a time.  FB_W=64 so each row = 16
    // aligned words.
    int write_failures = 0;
    for (int y = 0; y < FB_H; y++) {
        for (int wx = 0; wx < FB_W; wx += 4) {
            uint8_t b0 = scenario_b_pixel(wx + 0, y);  // lowest addr (4k+0)
            uint8_t b1 = scenario_b_pixel(wx + 1, y);
            uint8_t b2 = scenario_b_pixel(wx + 2, y);
            uint8_t b3 = scenario_b_pixel(wx + 3, y);  // highest addr
            uint32_t word_pixel_addr =
                FB_BASE + uint32_t(y) * FB_STRIDE + uint32_t(wx);
            int rc = vram_write_long_move_l(word_pixel_addr, b0, b1, b2, b3);
            if (rc != 0) {
                write_failures++;
                if (write_failures <= 4) {
                    std::printf("  move_l write (x=%d,y=%d) failed rc=%d\n",
                                wx, y, rc);
                }
            }
        }
    }
    if (write_failures) {
        std::printf("[FAIL] scenario-B: %d MOVE.L writes failed\n",
                    write_failures);
        delete dut;
        return 1;
    }
    std::printf("  VRAM filled with 4-pixels-per-MOVE.L pattern\n");

    // Scanout vertical-blanking head-start: matches the pre-scenario-A
    // tick_both(90) so fb_reader has a full line prefetched before
    // active scanout begins.  Without this, the first few pixels read
    // as zero from an empty linebuf.
    tick_both(90);

    std::vector<uint32_t> captured_b;
    scanout_frame(captured_b);

    std::vector<uint32_t> expected_b(FB_W * FB_H, 0);
    for (int y = 0; y < FB_H; y++) {
        for (int x = 0; x < FB_W; x++) {
            expected_b[y * FB_W + x] =
                palette_rgb(scenario_b_pixel(x, y)) & 0xFFFFFFu;
        }
    }

    int mismatches_b = 0;
    int first_x_b = -1, first_y_b = -1;
    uint32_t first_got_b = 0, first_exp_b = 0;
    for (int y = 0; y < FB_H; y++) {
        for (int x = 0; x < FB_W; x++) {
            uint32_t got = captured_b[y * FB_W + x];
            uint32_t exp = expected_b[y * FB_W + x];
            if (got != exp) {
                if (mismatches_b == 0) {
                    first_x_b = x; first_y_b = y;
                    first_got_b = got; first_exp_b = exp;
                }
                mismatches_b++;
            }
        }
    }

    bool underflow_b = dut->fb_underflow_sticky != 0;

    std::printf("  captured pixels: %d / %d match, %d mismatch\n",
                FB_W * FB_H - mismatches_b, FB_W * FB_H, mismatches_b);
    std::printf("  fb_underflow_sticky: %d\n", underflow_b);

    if (mismatches_b > 0 || underflow_b) {
        if (mismatches_b > 0) {
            std::printf("[FAIL] scenario-B mirror reproduced: first mismatch at "
                        "(%d,%d): got=0x%06X expected=0x%06X\n",
                        first_x_b, first_y_b, first_got_b, first_exp_b);
            // Diagnostic: dump the first 4-pixel group at the
            // mismatch row so the reversal pattern is visible.
            int gx0 = (first_x_b / 4) * 4;
            std::printf("  first-group (y=%d, x=%d..%d): "
                        "expected=[%06X %06X %06X %06X] "
                        "got=[%06X %06X %06X %06X]\n",
                        first_y_b, gx0, gx0 + 3,
                        expected_b[first_y_b * FB_W + gx0 + 0],
                        expected_b[first_y_b * FB_W + gx0 + 1],
                        expected_b[first_y_b * FB_W + gx0 + 2],
                        expected_b[first_y_b * FB_W + gx0 + 3],
                        captured_b[first_y_b * FB_W + gx0 + 0],
                        captured_b[first_y_b * FB_W + gx0 + 1],
                        captured_b[first_y_b * FB_W + gx0 + 2],
                        captured_b[first_y_b * FB_W + gx0 + 3]);
        }
        if (underflow_b) {
            std::printf("[FAIL] fb_reader or linebuf underflowed during "
                        "scenario-B scanout\n");
        }
        dump_ppm("build/framebuffer_pixel/captured_b.ppm", captured_b);
        dump_ppm("build/framebuffer_pixel/expected_b.ppm", expected_b);
        std::printf("  PPMs dumped to build/framebuffer_pixel/"
                    "{captured_b,expected_b}.ppm\n");
        delete dut;
        return 1;
    }

    std::printf("[PASS] scenario-B (MOVE.L 4-pixels): %dx%d pixel-exact\n",
                FB_W, FB_H);

    // ══════════════════════════════════════════════════════════════════
    // Scenario C — upper-half VRAM scanout (vram.v Bug 2 repro).
    //
    // Places the framebuffer at fb_base_px = 0x100000 (bit 20 set — the
    // upper half of vram.v's 2 MiB VRAM_BYTES aperture).  Pre-fix,
    // vram.v's streaming read port sized PX_ADDR_W from FB_WIDTH_PX *
    // FB_HEIGHT_PX alone (this instance's 256x80 = 20,480 pixels, only
    // 15 bits) rather than from the full VRAM_BYTES aperture (2 MiB =
    // 2,097,152 pixels at BPP=8, needing 21 bits) — the top word-index
    // bit was tied to zero inside the rd_word_idx split, so a scan
    // based this high in VRAM would silently alias onto the LOW half
    // instead of faulting, reading back whatever (wrong) pattern sits
    // at the aliased low address.  Also exercises the DAFB BASE_HI/
    // BASE_LO split reaching a base value that needs BASE_HI's upper
    // bits set, and scanout_placement_sync's placement-range gate
    // accepting a base this large (needs the FB_MAX_PIXELS widening).
    // ══════════════════════════════════════════════════════════════════
    std::printf("\nscenario-C: upper-VRAM-half scanout (fb_base_px = 0x100000, Bug 2 repro)\n");

    const uint32_t FB_BASE_C = 0x100000;  // bit 20 set: upper half of the 2 MiB VRAM aperture

    dut->de_in = 0;
    dut->vs_in = 1;
    dut->hs_in = 0;
    tick_both(64);
    reset_dut();
    tick_both(32);
    if (int rc = program_dafb(FB_BASE_C)) {
        std::printf("[FAIL] DAFB programming (scenario-C) rc=%d\n", rc);
        delete dut;
        return 1;
    }
    std::printf("  DAFB programmed: base=0x%x stride=%d bpp=8 (readback base=0x%x stride=%u)\n",
                FB_BASE_C, FB_STRIDE, dut->fb_base_px_out, dut->fb_stride_px_out);
    if (dut->fb_base_px_out != FB_BASE_C) {
        std::printf("[FAIL] scenario-C: DAFB base readback 0x%x != programmed 0x%x\n",
                    dut->fb_base_px_out, FB_BASE_C);
        delete dut;
        return 1;
    }

    if (int rc = write_pattern(FB_BASE_C)) {
        std::printf("[FAIL] VRAM write (scenario-C) rc=%d\n", rc);
        delete dut;
        return 1;
    }
    std::printf("  VRAM filled with (x+y)&0xF diagonal pattern at base 0x%x\n", FB_BASE_C);

    tick_both(300);

    std::vector<uint32_t> captured_c;
    scanout_frame(captured_c);

    std::vector<uint32_t> expected_c(FB_W * FB_H, 0);
    for (int y = 0; y < FB_H; y++) {
        for (int x = 0; x < FB_W; x++) {
            expected_c[y * FB_W + x] =
                palette_rgb(expected_pixel(x, y)) & 0xFFFFFFu;
        }
    }

    int mismatches_c = 0;
    int first_x_c = -1, first_y_c = -1;
    uint32_t first_got_c = 0, first_exp_c = 0;
    for (int y = 0; y < FB_H; y++) {
        for (int x = 0; x < FB_W; x++) {
            uint32_t got = captured_c[y * FB_W + x];
            uint32_t exp = expected_c[y * FB_W + x];
            if (got != exp) {
                if (mismatches_c == 0) {
                    first_x_c = x; first_y_c = y;
                    first_got_c = got; first_exp_c = exp;
                }
                mismatches_c++;
            }
        }
    }

    bool underflow_c = dut->fb_underflow_sticky != 0;

    std::printf("  captured pixels: %d / %d match, %d mismatch\n",
                FB_W * FB_H - mismatches_c, FB_W * FB_H, mismatches_c);
    std::printf("  fb_underflow_sticky: %d\n", underflow_c);

    if (mismatches_c > 0 || underflow_c) {
        if (mismatches_c > 0) {
            std::printf("[FAIL] scenario-C: first mismatch at (%d,%d): got=0x%06X expected=0x%06X "
                        "(upper-VRAM-half scan likely aliased to the low half — vram.v Bug 2)\n",
                        first_x_c, first_y_c, first_got_c, first_exp_c);
        }
        if (underflow_c) {
            std::printf("[FAIL] fb_reader or linebuf underflowed during scenario-C scanout\n");
        }
        dump_ppm("build/framebuffer_pixel/captured_c.ppm", captured_c);
        dump_ppm("build/framebuffer_pixel/expected_c.ppm", expected_c);
        std::printf("  PPMs dumped to build/framebuffer_pixel/{captured_c,expected_c}.ppm\n");
        delete dut;
        return 1;
    }

    std::printf("[PASS] scenario-C (upper-VRAM-half): %dx%d pixel-exact\n", FB_W, FB_H);

    // ══════════════════════════════════════════════════════════════════
    // Scenario D — T9: 256 DISTINCT CLUT entries via the real AC842
    // RAMDAC register write path, rendering an 8bpp pattern that hits
    // indices ABOVE 15.  MUST FAIL on pre-T9 RTL: the retired 16-entry
    // low-depth CLUT indexed scanout with pix_data_pipe2[3:0] (low
    // nibble only), so any pixel whose true byte value differs from its
    // low nibble rendered the WRONG colour.  This is the proof that the
    // nibble hack is gone and the full 256-entry BRAM CLUT is live.
    // ══════════════════════════════════════════════════════════════════
    std::printf("\nscenario-D: 256-entry CLUT, indices above 15 (nibble-hack regression proof)\n");

    // Deliberately NOT FB_BASE (0) -- scenario A/B both left stale content
    // at address 0 whose row-0 bytes happen to equal x&0xF (their pattern
    // functions all reduce to that at y=0).  linebuf_scanout.v's placement-
    // restart trigger only fires on an actual fb_base_px/fb_stride_px
    // VALUE change (see `placement_changed` in linebuf_scanout.v); since 0
    // is also this tb's compiled-in DEFAULT_BASE, reprogramming base=0
    // again after reset does not trip a restart, so a row 0 prefetched
    // before program_dafb()/write_pattern() run (against whatever was
    // previously in VRAM) is never invalidated.  This is a pre-existing
    // characteristic of the restart-detection logic, orthogonal to T9's
    // CLUT change -- every prior scenario using base 0 happened to write
    // an IDENTICAL x&0xF row-0 pattern, so it was never visible.  Using a
    // fresh base here (matching scenario C's existing approach) sidesteps
    // it rather than papering over a result mismatch.
    const uint32_t FB_BASE_D = 0x00080000u;

    dut->de_in = 0;
    dut->vs_in = 1;
    dut->hs_in = 0;
    tick_both(64);
    reset_dut();
    tick_both(32);
    if (int rc = program_dafb(FB_BASE_D)) {
        std::printf("[FAIL] DAFB programming (scenario-D) rc=%d\n", rc);
        delete dut;
        return 1;
    }
    std::printf("  DAFB programmed: base=0x%x stride=%d bpp=8, 256-entry CLUT live\n",
                FB_BASE_D, FB_STRIDE);

    for (int y = 0; y < FB_H; y++) {
        for (int x = 0; x < FB_W; x++) {
            uint32_t addr = FB_BASE_D + uint32_t(y) * FB_STRIDE + uint32_t(x);
            if (int rc = vram_write_byte(addr, expected_pixel_full256(x, y))) {
                std::printf("[FAIL] VRAM write (scenario-D) rc=%d\n", rc);
                delete dut;
                return 1;
            }
        }
    }
    std::printf("  VRAM filled with full-byte-range pattern (indices above 15 present)\n");

    tick_both(300);

    std::vector<uint32_t> captured_d;
    scanout_frame(captured_d);

    std::vector<uint32_t> expected_d(FB_W * FB_H, 0);
    bool saw_index_above_15 = false;
    for (int y = 0; y < FB_H; y++) {
        for (int x = 0; x < FB_W; x++) {
            uint8_t idx = expected_pixel_full256(x, y);
            if (idx > 15) saw_index_above_15 = true;
            expected_d[y * FB_W + x] = palette_rgb(idx) & 0xFFFFFFu;
        }
    }

    int mismatches_d = 0;
    int first_x_d = -1, first_y_d = -1;
    uint32_t first_got_d = 0, first_exp_d = 0;
    for (int y = 0; y < FB_H; y++) {
        for (int x = 0; x < FB_W; x++) {
            uint32_t got = captured_d[y * FB_W + x];
            uint32_t exp = expected_d[y * FB_W + x];
            if (got != exp) {
                if (mismatches_d == 0) {
                    first_x_d = x; first_y_d = y;
                    first_got_d = got; first_exp_d = exp;
                }
                mismatches_d++;
            }
        }
    }

    bool underflow_d = dut->fb_underflow_sticky != 0;

    std::printf("  captured pixels: %d / %d match, %d mismatch (pattern includes indices > 15: %s)\n",
                FB_W * FB_H - mismatches_d, FB_W * FB_H, mismatches_d,
                saw_index_above_15 ? "yes" : "no");
    std::printf("  fb_underflow_sticky: %d\n", underflow_d);

    if (!saw_index_above_15) {
        std::printf("[FAIL] scenario-D: test pattern never exercised an index above 15 -- weakened test\n");
        delete dut;
        return 1;
    }

    if (mismatches_d > 0 || underflow_d) {
        if (mismatches_d > 0) {
            std::printf("[FAIL] scenario-D: first mismatch at (%d,%d): got=0x%06X expected=0x%06X "
                        "(index=%d) -- 256-entry CLUT not reaching scanout\n",
                        first_x_d, first_y_d, first_got_d, first_exp_d,
                        expected_pixel_full256(first_x_d, first_y_d));
        }
        if (underflow_d) {
            std::printf("[FAIL] fb_reader or linebuf underflowed during scenario-D scanout\n");
        }
        dump_ppm("build/framebuffer_pixel/captured_d.ppm", captured_d);
        dump_ppm("build/framebuffer_pixel/expected_d.ppm", expected_d);
        std::printf("  PPMs dumped to build/framebuffer_pixel/{captured_d,expected_d}.ppm\n");
        delete dut;
        return 1;
    }

    std::printf("[PASS] scenario-D (256-entry CLUT, indices > 15): %dx%d pixel-exact\n", FB_W, FB_H);

    // ══════════════════════════════════════════════════════════════════
    // Scenario E — T9: mid-frame CLUT write lands immediately, no
    // frame-atomic buffering, no scanout wedge/desync.  Fills the whole
    // frame with a single palette index, starts a scanout, reprograms
    // that index's RGB via the real RAMDAC path partway through the
    // frame (row FB_H/2), and confirms rows well before the injection
    // point still show the OLD colour while rows well after it show the
    // NEW colour -- all within the SAME frame.  A small buffer zone
    // around the injection row is excluded from the strict check to
    // avoid asserting against the ~1-pixel scanout pipeline lag at the
    // exact injection boundary (not a correctness issue -- see
    // linebuf_scanout.v's pipe3/pipe4 alignment comments).
    // ══════════════════════════════════════════════════════════════════
    std::printf("\nscenario-E: mid-frame CLUT write lands without frame-atomic buffering\n");

    const uint8_t  MIDFRAME_IDX = 200;
    const uint32_t OLD_RGB_E    = 0x102030u;
    const uint32_t NEW_RGB_E    = 0xA0B0C0u;
    const int      INJECT_ROW   = FB_H / 2;
    const int      ROW_BUFFER   = 2;
    // Fresh base, not FB_BASE (0) -- see the FB_BASE_D comment in
    // scenario-D above for why a base equal to this tb's compiled-in
    // DEFAULT_BASE doesn't trip linebuf_scanout.v's placement-restart on
    // its own and can surface stale line_mem content from an earlier
    // scenario at row 0.
    const uint32_t FB_BASE_E = 0x000C0000u;

    dut->de_in = 0;
    dut->vs_in = 1;
    dut->hs_in = 0;
    tick_both(64);
    reset_dut();
    tick_both(32);

    if (int rc = program_clut()) {
        std::printf("[FAIL] DAFB CLUT programming (scenario-E) rc=%d\n", rc);
        delete dut;
        return 1;
    }
    // Deterministically (re)program MIDFRAME_IDX = OLD_RGB_E so the
    // "before" half of the frame starts from a known colour regardless
    // of what program_clut()'s default formula assigned it.
    if (dafb_write(DAFB_RAMDAC_ADDR_OFF, MIDFRAME_IDX) != 0 ||
        dafb_write(DAFB_RAMDAC_DATA_OFF, (OLD_RGB_E >> 16) & 0xFFu) != 0 ||
        dafb_write(DAFB_RAMDAC_DATA_OFF, (OLD_RGB_E >> 8) & 0xFFu) != 0 ||
        dafb_write(DAFB_RAMDAC_DATA_OFF, OLD_RGB_E & 0xFFu) != 0) {
        std::printf("[FAIL] scenario-E: seed OLD_RGB write failed\n");
        delete dut;
        return 1;
    }
    if (dafb_write_pcbr(PCBR_8BPP) != 0) {
        std::printf("[FAIL] scenario-E: PCBR programming failed\n");
        delete dut;
        return 1;
    }
    if (dafb_write_base(FB_BASE_E) != 0 || dafb_write_stride(FB_STRIDE) != 0) {
        std::printf("[FAIL] scenario-E: base/stride programming failed\n");
        delete dut;
        return 1;
    }

    for (int y = 0; y < FB_H; y++) {
        for (int x = 0; x < FB_W; x++) {
            uint32_t addr = FB_BASE_E + uint32_t(y) * FB_STRIDE + uint32_t(x);
            if (int rc = vram_write_byte(addr, MIDFRAME_IDX)) {
                std::printf("[FAIL] VRAM write (scenario-E) rc=%d\n", rc);
                delete dut;
                return 1;
            }
        }
    }

    tick_both(300);

    std::vector<uint32_t> captured_e;
    scanout_frame_with_midframe_write(captured_e, INJECT_ROW, MIDFRAME_IDX, NEW_RGB_E);

    bool underflow_e = dut->fb_underflow_sticky != 0;
    int mismatches_e = 0;
    int checked_e = 0;
    int first_x_e = -1, first_y_e = -1;
    uint32_t first_got_e = 0, first_exp_e = 0;
    for (int y = 0; y < FB_H; y++) {
        if (y >= INJECT_ROW - ROW_BUFFER && y < INJECT_ROW + ROW_BUFFER)
            continue;  // skip the injection-boundary buffer rows
        uint32_t expect = (y < INJECT_ROW) ? OLD_RGB_E : NEW_RGB_E;
        for (int x = 0; x < FB_W; x++) {
            uint32_t got = captured_e[y * FB_W + x];
            checked_e++;
            if (got != expect) {
                if (mismatches_e == 0) {
                    first_x_e = x; first_y_e = y;
                    first_got_e = got; first_exp_e = expect;
                }
                mismatches_e++;
            }
        }
    }

    std::printf("  captured pixels: %d / %d checked match (old/new split at row %d, buffer=%d rows), %d mismatch\n",
                checked_e - mismatches_e, checked_e, INJECT_ROW, ROW_BUFFER, mismatches_e);
    std::printf("  fb_underflow_sticky: %d\n", underflow_e);

    if (mismatches_e > 0 || underflow_e) {
        if (mismatches_e > 0) {
            std::printf("[FAIL] scenario-E: first mismatch at (%d,%d): got=0x%06X expected=0x%06X\n",
                        first_x_e, first_y_e, first_got_e, first_exp_e);
        }
        if (underflow_e) {
            std::printf("[FAIL] fb_reader or linebuf underflowed/wedged during scenario-E mid-frame write\n");
        }
        dump_ppm("build/framebuffer_pixel/captured_e.ppm", captured_e);
        delete dut;
        return 1;
    }

    std::printf("[PASS] scenario-E (mid-frame CLUT write, no frame-atomic buffering): %dx%d split at row %d\n",
                FB_W, FB_H, INJECT_ROW);

    // ══════════════════════════════════════════════════════════════════
    // scenario-F/G/H: RUNTIME 8bpp <-> 24bpp depth switching
    // ══════════════════════════════════════════════════════════════════
    // The depth channel (bpp_shift / fb_bytes_per_px / depth_supported) is
    // taken live from the DAFB shim in this harness, so these scenarios
    // switch depth the way Mac OS does: by writing the AC842 PCBR register
    // and letting scanout_placement_sync commit the new depth alongside a
    // matching base/stride at a frame boundary.
    //
    //   F: 8bpp  renders pixel-exact through the CLUT      (PCBR 0x18)
    //   G: 24bpp renders pixel-exact as direct colour      (PCBR 0x1C)
    //   H: 8 -> 24 -> 8 in one run, each frame pixel-exact, with the
    //      committed stride/depth asserted at every step.
    //
    // A test that only checked 24bpp-at-elaboration would prove nothing
    // about the switch, which is the whole point of the runtime path.
    {
        // 24bpp is 4 VRAM bytes per pixel (xRGB, big-endian: +0 pad, +1 R,
        // +2 G, +3 B -- see rtl/mac/video.v decode_bytes_per_px for the MAME
        // derivation), so a 64-wide row spans 256 bytes.
        const uint32_t STRIDE_24  = FB_W * 4;      // 256
        const uint32_t BASE_8     = 0x00010000u;
        const uint32_t BASE_24    = 0x00020000u;
        // Enough per-row fetch slack for 3 byte-requests per displayed pixel
        // plus fb_reader's CDC round trip.  See scanout_frame()'s comment.
        const int HBLANK_24       = 320;
        const int HBLANK_8        = 320;

        // Distinct, non-symmetric direct-colour pattern.  Deliberately makes
        // R/G/B all differ so a byte-phase mis-ordering (R/B swap, pad byte
        // fetched instead of R, ...) cannot alias into a pass.
        auto direct_rgb = [](int x, int y) -> uint32_t {
            uint8_t r = uint8_t((x * 5 + y * 3 + 11) & 0xFF);
            uint8_t g = uint8_t((x * 11 + y * 7 + 29) & 0xFF);
            uint8_t b = uint8_t((x * 3 + y * 13 + 199) & 0xFF);
            return (uint32_t(r) << 16) | (uint32_t(g) << 8) | uint32_t(b);
        };

        // `stride` is a parameter (not the fixed STRIDE_24) so the
        // unaligned-placement scenarios below can lay the same pattern down at
        // a stride whose low bits defeat the wide fetch path.
        auto write_pattern_24 = [&](uint32_t base, uint32_t stride) -> int {
            for (int y = 0; y < FB_H; y++) {
                for (int x = 0; x < FB_W; x++) {
                    uint32_t rgb = direct_rgb(x, y);
                    uint32_t a = base + uint32_t(y) * stride
                               + uint32_t(x) * 4u;
                    // +0 pad byte written to a recognisable non-zero value:
                    // if the fetcher ever pulled it as part of the pixel the
                    // comparison below would fail loudly instead of silently
                    // matching a zero pad.
                    if (int rc = vram_write_byte(a + 0, 0xA5))          return rc;
                    if (int rc = vram_write_byte(a + 1, (rgb >> 16) & 0xFF)) return rc;
                    if (int rc = vram_write_byte(a + 2, (rgb >> 8) & 0xFF))  return rc;
                    if (int rc = vram_write_byte(a + 3, rgb & 0xFF))         return rc;
                }
            }
            return 0;
        };

        auto write_pattern_8 = [&](uint32_t base) -> int {
            for (int y = 0; y < FB_H; y++) {
                for (int x = 0; x < FB_W; x++) {
                    uint32_t a = base + uint32_t(y) * FB_STRIDE + uint32_t(x);
                    if (int rc = vram_write_byte(a, expected_pixel_full256(x, y)))
                        return rc;
                }
            }
            return 0;
        };

        // Returns mismatch count; fills first_* with the first mismatch.
        auto compare = [&](const std::vector<uint32_t>& cap,
                           bool direct,
                           int& fx, int& fy,
                           uint32_t& fgot, uint32_t& fexp) -> int {
            int bad = 0;
            for (int y = 0; y < FB_H; y++) {
                for (int x = 0; x < FB_W; x++) {
                    uint32_t exp = direct
                        ? direct_rgb(x, y)
                        : palette_rgb(expected_pixel_full256(x, y));
                    uint32_t got = cap[y * FB_W + x];
                    if (got != exp) {
                        if (bad == 0) { fx = x; fy = y; fgot = got; fexp = exp; }
                        bad++;
                    }
                }
            }
            return bad;
        };

        int transition_underflow_events = 0;
        int steady_underflow_events = 0;

        // A depth+placement change is committed at a frame boundary and the
        // fetcher then has to refill the whole line pool, so the FIRST frame
        // after a switch is the one that exercises the transition and the
        // SECOND is the one that is guaranteed steady-state.  Scan two and
        // assert on the second, exactly as the frame-atomic contract allows.
        auto switch_and_scan = [&](uint32_t pcbr, uint32_t base,
                                   uint32_t stride, int hblank,
                                   std::vector<uint32_t>& cap) {
            dafb_write_pcbr(pcbr);
            dafb_write_base(base);
            dafb_write_stride(stride);
            tick_both(300);
            std::vector<uint32_t> throwaway;
            // Transition frame: the whole line pool has to be refilled under
            // the new depth while the display is already scanning, so a burst
            // of underflow events here is expected and matches real hardware
            // flashing on a mode change.  Recorded, not asserted on.
            scanout_frame_blanked(throwaway, hblank);
            transition_underflow_events = scanout_underflow_events;
            // Steady frame: this one must be clean.
            scanout_frame_blanked(cap, hblank);
            steady_underflow_events = scanout_underflow_events;
        };

        // ── scenario-F: 8bpp indexed, depth sourced from a real PCBR ──
        std::printf("\nscenario-F: 8bpp indexed via PCBR 0x%02X (runtime depth channel)\n",
                    PCBR_8BPP);
        dut->de_in = 0; dut->vs_in = 1; dut->hs_in = 0;
        tick_both(64);
        reset_dut();
        tick_both(32);
        if (int rc = program_clut()) {
            std::printf("[FAIL] scenario-F: CLUT programming rc=%d\n", rc);
            delete dut; return 1;
        }
        if (int rc = write_pattern_8(BASE_8)) {
            std::printf("[FAIL] scenario-F: VRAM fill rc=%d\n", rc);
            delete dut; return 1;
        }
        if (int rc = write_pattern_24(BASE_24, STRIDE_24)) {
            std::printf("[FAIL] scenario-F: 24bpp VRAM fill rc=%d\n", rc);
            delete dut; return 1;
        }

        std::vector<uint32_t> cap_f;
        switch_and_scan(PCBR_8BPP, BASE_8, FB_STRIDE, HBLANK_8, cap_f);

        bool ok_f = true;
        if (dut->fb_bytes_per_px_out != 1) {
            std::printf("[FAIL] scenario-F: fb_bytes_per_px = %u (want 1)\n",
                        unsigned(dut->fb_bytes_per_px_out));
            ok_f = false;
        }
        if (!dut->depth_supported_out) {
            std::printf("[FAIL] scenario-F: depth_supported low at 8bpp\n");
            ok_f = false;
        }
        if (dut->dbg_fetch_direct) {
            std::printf("[FAIL] scenario-F: scanner latched direct colour at 8bpp\n");
            ok_f = false;
        }
        if (dut->committed_fb_stride_px != FB_STRIDE) {
            std::printf("[FAIL] scenario-F: committed stride = %u (want %u)\n",
                        unsigned(dut->committed_fb_stride_px), unsigned(FB_STRIDE));
            ok_f = false;
        }
        int fx = -1, fy = -1; uint32_t fgot = 0, fexp = 0;
        int bad_f = compare(cap_f, /*direct=*/false, fx, fy, fgot, fexp);
        std::printf("  captured pixels: %d / %d match, %d mismatch\n",
                    FB_W * FB_H - bad_f, FB_W * FB_H, bad_f);
        std::printf("  underflow events: transition=%d steady=%d\n",
                    transition_underflow_events, steady_underflow_events);
        if (bad_f) {
            std::printf("[FAIL] scenario-F: first mismatch at (%d,%d): got=0x%06X expected=0x%06X\n",
                        fx, fy, fgot, fexp);
            ok_f = false;
        }
        if (steady_underflow_events) {
            std::printf("[FAIL] scenario-F: %d steady-state underflow events at 8bpp\n",
                        steady_underflow_events);
            ok_f = false;
        }
        if (!ok_f) { dump_ppm("build/framebuffer_pixel/captured_f.ppm", cap_f);
                     delete dut; return 1; }
        std::printf("[PASS] scenario-F (8bpp indexed, PCBR-sourced depth): %dx%d pixel-exact\n",
                    FB_W, FB_H);

        // ── scenario-G: 24bpp direct colour ──────────────────────────
        std::printf("\nscenario-G: 24bpp direct colour via PCBR 0x%02X (4 B/px xRGB)\n",
                    PCBR_24BPP);
        std::vector<uint32_t> cap_g;
        switch_and_scan(PCBR_24BPP, BASE_24, STRIDE_24, HBLANK_24, cap_g);

        bool ok_g = true;
        if (dut->fb_bytes_per_px_out != 4) {
            std::printf("[FAIL] scenario-G: fb_bytes_per_px = %u (want 4)\n",
                        unsigned(dut->fb_bytes_per_px_out));
            ok_g = false;
        }
        if (!dut->depth_supported_out) {
            std::printf("[FAIL] scenario-G: depth_supported low at 24bpp -- the "
                        "scanner now renders it, so it must not be gated off\n");
            ok_g = false;
        }
        if (!dut->dbg_fetch_direct) {
            std::printf("[FAIL] scenario-G: scanner did not latch direct colour\n");
            ok_g = false;
        }
        // BASE_24 and STRIDE_24 are both 4-byte aligned, so the wide
        // (1-request-per-pixel) path MUST engage.  This assert exists because
        // wide and narrow are pixel-identical by construction: without it, a
        // wide path that silently never engaged would leave this whole
        // scenario -- and the entire suite -- green while the 24bpp bandwidth
        // fix was inoperative.
        if (!dut->dbg_fetch_wide) {
            std::printf("[FAIL] scenario-G: aligned 24bpp did not take the WIDE "
                        "fetch path (fetch_wide=0)\n");
            ok_g = false;
        }
        if (dut->committed_bytes_per_px != 4) {
            std::printf("[FAIL] scenario-G: committed bytes_per_px = %u (want 4)\n",
                        unsigned(dut->committed_bytes_per_px));
            ok_g = false;
        }
        if (dut->committed_fb_stride_px != STRIDE_24) {
            std::printf("[FAIL] scenario-G: committed stride = %u (want %u)\n",
                        unsigned(dut->committed_fb_stride_px), unsigned(STRIDE_24));
            ok_g = false;
        }
        fx = fy = -1; fgot = fexp = 0;
        int bad_g = compare(cap_g, /*direct=*/true, fx, fy, fgot, fexp);
        std::printf("  captured pixels: %d / %d match, %d mismatch\n",
                    FB_W * FB_H - bad_g, FB_W * FB_H, bad_g);
        std::printf("  underflow events: transition=%d steady=%d\n",
                    transition_underflow_events, steady_underflow_events);
        if (bad_g) {
            std::printf("[FAIL] scenario-G: first mismatch at (%d,%d): got=0x%06X expected=0x%06X\n",
                        fx, fy, fgot, fexp);
            ok_g = false;
        }
        if (steady_underflow_events) {
            std::printf("[FAIL] scenario-G: %d steady-state underflow events at 24bpp -- "
                        "the scanout fetch path cannot sustain 3 B/px at this geometry\n",
                        steady_underflow_events);
            ok_g = false;
        }
        if (dut->dbg_fb_reader_underflow) {
            std::printf("[FAIL] scenario-G: fb_reader CDC overflowed at 24bpp\n");
            ok_g = false;
        }
        if (!ok_g) { dump_ppm("build/framebuffer_pixel/captured_g.ppm", cap_g);
                     delete dut; return 1; }
        std::printf("[PASS] scenario-G (24bpp direct colour): %dx%d pixel-exact\n",
                    FB_W, FB_H);

        // ── scenario-H: runtime 8 -> 24 -> 8 ─────────────────────────
        // Same DUT instance, no reset between steps.  This is the check the
        // elaboration-parameter design could not pass at all.
        std::printf("\nscenario-H: runtime depth switch 8 -> 24 -> 8 (no reset between)\n");
        struct Step { const char* name; uint32_t pcbr; uint32_t base;
                      uint32_t stride; bool direct; int hblank; };
        const Step steps[] = {
            {"8bpp",  PCBR_8BPP,  BASE_8,  FB_STRIDE, false, HBLANK_8 },
            {"24bpp", PCBR_24BPP, BASE_24, STRIDE_24, true,  HBLANK_24},
            {"8bpp",  PCBR_8BPP,  BASE_8,  FB_STRIDE, false, HBLANK_8 },
        };
        bool ok_h = true;
        for (const Step& s : steps) {
            std::vector<uint32_t> cap;
            switch_and_scan(s.pcbr, s.base, s.stride, s.hblank, cap);
            fx = fy = -1; fgot = fexp = 0;
            int bad = compare(cap, s.direct, fx, fy, fgot, fexp);
            unsigned want_bppx = s.direct ? 4u : 1u;
            std::printf("  step %-5s: bppx=%u(want %u) direct=%u(want %u) "
                        "stride=%u(want %u) mismatch=%d underflow(trans/steady)=%d/%d\n",
                        s.name,
                        unsigned(dut->committed_bytes_per_px), want_bppx,
                        unsigned(dut->dbg_fetch_direct), unsigned(s.direct),
                        unsigned(dut->committed_fb_stride_px), unsigned(s.stride),
                        bad, transition_underflow_events, steady_underflow_events);
            if (unsigned(dut->committed_bytes_per_px) != want_bppx) ok_h = false;
            if (bool(dut->dbg_fetch_direct) != s.direct)            ok_h = false;
            if (unsigned(dut->committed_fb_stride_px) != s.stride)  ok_h = false;
            if (bad)                                                ok_h = false;
            if (steady_underflow_events)                            ok_h = false;
            if (bad) {
                std::printf("    first mismatch at (%d,%d): got=0x%06X expected=0x%06X\n",
                            fx, fy, fgot, fexp);
                dump_ppm(s.direct ? "build/framebuffer_pixel/captured_h24.ppm"
                                  : "build/framebuffer_pixel/captured_h8.ppm", cap);
            }
        }
        if (!ok_h) {
            std::printf("[FAIL] scenario-H: runtime depth switch 8 -> 24 -> 8\n");
            delete dut; return 1;
        }
        std::printf("[PASS] scenario-H (runtime depth switch 8 -> 24 -> 8): "
                    "3 frames pixel-exact, stride/depth correct at each step\n");

        // ── scenarios I/J/K: wide 24bpp across awkward ring-line phases ──
        // The wide path issues ONE aligned 4-byte request per pixel, which is
        // safe because the DAFB register encoding makes 4-byte alignment
        // structural, not a hope: rtl/mac/video.v builds fb_base_px from
        // r_base_hi<<9 | r_base_lo<<5 (so the base is 32-BYTE aligned) and
        // fb_stride_px from r_stride_raw<<2 (so the stride is 4-BYTE aligned),
        // matching MAME's DAFB, where the base is in 32-byte units and the
        // stride in longwords.  An unaligned base or stride is therefore NOT
        // PROGRAMMABLE -- a test that tried would just watch its value get
        // truncated.  The scenario-Z assertions below pin that invariant down
        // so the wide path's precondition cannot silently rot.
        //
        // What DOES vary, and what these three stress, is the row-start PHASE
        // relative to the 128 B DDR ring line and the 16 B AXI beat: a legal
        // stride need only be a multiple of 4, so rows routinely start
        // mid-beat and mid-ring-line and end on a PARTIAL line.  That is the
        // real risk surface in the widened fetch, so all three cases stay wide
        // and must be pixel-exact.
        //
        // The 0xA5 pad byte written by write_pattern_24 is load-bearing here:
        // if the lane order or the pad handling were wrong, 0xA5 would surface
        // as a colour channel and fail loudly instead of aliasing into a pass
        // against a zero pad.  (That is exactly how the first draft of these
        // scenarios caught its own bug.)
        struct Place { const char* name; uint32_t base; uint32_t stride;
                       bool want_wide; const char* why; };
        const Place places[] = {
            {"I", 0x00030000u, 260u, true,
             "stride 4-aligned but NOT 16B/128B aligned: every row starts at a "
             "different phase inside a 128B ring line and ends on a PARTIAL "
             "line, and each row start is mid-AXI-beat"},
            {"J", 0x00040000u, 292u, true,
             "another non-16B-multiple stride (292 = 4*73), so the row-start "
             "phase walks differently through the ring than scenario I"},
            {"K", 0x00050000u, 320u, true,
             "stride 16B-aligned but not 128B-aligned: row starts land on beat "
             "boundaries yet still mid-ring-line"},
        };
        bool ok_ijk = true;
        for (const Place& p : places) {
            std::printf("\nscenario-%s: 24bpp base=0x%X stride=%u\n  %s\n",
                        p.name, p.base, p.stride, p.why);
            if (int rc = write_pattern_24(p.base, p.stride)) {
                std::printf("[FAIL] scenario-%s: VRAM pattern write rc=%d\n",
                            p.name, rc);
                ok_ijk = false;
                continue;
            }
            std::vector<uint32_t> cap;
            switch_and_scan(PCBR_24BPP, p.base, p.stride, HBLANK_24, cap);
            fx = fy = -1; fgot = fexp = 0;
            int bad = compare(cap, /*direct=*/true, fx, fy, fgot, fexp);
            std::printf("  direct=%u wide=%u(want %u) mismatch=%d "
                        "underflow(trans/steady)=%d/%d\n",
                        unsigned(dut->dbg_fetch_direct),
                        unsigned(dut->dbg_fetch_wide), unsigned(p.want_wide),
                        bad, transition_underflow_events,
                        steady_underflow_events);
            if (!dut->dbg_fetch_direct) {
                std::printf("[FAIL] scenario-%s: 24bpp did not latch direct colour\n",
                            p.name);
                ok_ijk = false;
            }
            if (bool(dut->dbg_fetch_wide) != p.want_wide) {
                std::printf("[FAIL] scenario-%s: fetch_wide=%u want %u -- %s\n",
                            p.name, unsigned(dut->dbg_fetch_wide),
                            unsigned(p.want_wide), p.why);
                ok_ijk = false;
            }
            if (bad) {
                std::printf("[FAIL] scenario-%s: first mismatch at (%d,%d): "
                            "got=0x%06X expected=0x%06X\n",
                            p.name, fx, fy, fgot, fexp);
                dump_ppm("build/framebuffer_pixel/captured_ijk.ppm", cap);
                ok_ijk = false;
            }
            if (steady_underflow_events) {
                std::printf("[FAIL] scenario-%s: %d steady-state underflow events\n",
                            p.name, steady_underflow_events);
                ok_ijk = false;
            }
        }
        if (!ok_ijk) {
            std::printf("[FAIL] scenarios I/J/K (24bpp placement vs wide fetch)\n");
            delete dut;
            return 1;
        }
        std::printf("[PASS] scenarios I/J/K (awkward ring-line phases, wide 24bpp): "
                    "pixel-exact, fetch mode correct in all three\n");

        // ── scenario-Z: the wide path's alignment precondition is STRUCTURAL ──
        // The wide 24bpp fetch is safe only because fb_base_px / fb_stride_px
        // cannot carry unaligned low bits: rtl/mac/video.v:535-537 builds the
        // base from r_base_hi<<9 | r_base_lo<<5 (32-byte units) and the stride
        // from r_stride_raw<<2 (longwords), exactly as MAME's DAFB does.  Pin
        // that down by programming deliberately unaligned values and confirming
        // the encoding quantises them.  If anyone ever widens r_base_lo /
        // r_stride_raw so the low bits survive, THIS fires -- and at that point
        // linebuf_scanout's wide_ok_in guard (today unreachable by
        // construction) stops being belt-and-braces and starts being required.
        std::printf("\nscenario-Z: DAFB base/stride quantisation makes the wide "
                    "path's 4-byte alignment structural\n");
        const uint32_t Z_BASE_REQ   = 0x00030000u + 3u;  // +3, deliberately unaligned
        const uint32_t Z_STRIDE_REQ = 261u;              // 4*65 + 1, unaligned
        std::vector<uint32_t> cap_z;
        switch_and_scan(PCBR_24BPP, Z_BASE_REQ, Z_STRIDE_REQ, HBLANK_24, cap_z);
        const unsigned z_base   = unsigned(dut->committed_fb_base_px);
        const unsigned z_stride = unsigned(dut->committed_fb_stride_px);
        std::printf("  programmed base=0x%X stride=%u -> committed base=0x%X stride=%u\n",
                    Z_BASE_REQ, Z_STRIDE_REQ, z_base, z_stride);
        bool ok_z = true;
        if (z_base & 3u) {
            std::printf("[FAIL] scenario-Z: committed base 0x%X is NOT 4-byte aligned "
                        "-- the wide 24bpp fetch path's precondition is broken\n", z_base);
            ok_z = false;
        }
        if (z_base & 31u) {
            std::printf("[FAIL] scenario-Z: committed base 0x%X is not 32-byte aligned "
                        "-- DAFB encodes the base in 32-byte units (MAME parity)\n", z_base);
            ok_z = false;
        }
        if (z_stride & 3u) {
            std::printf("[FAIL] scenario-Z: committed stride %u is NOT 4-byte aligned "
                        "-- the wide 24bpp fetch path's precondition is broken\n", z_stride);
            ok_z = false;
        }
        if (!dut->dbg_fetch_wide) {
            std::printf("[FAIL] scenario-Z: wide path disengaged at a legal "
                        "(quantised) 24bpp placement\n");
            ok_z = false;
        }
        if (!ok_z) { delete dut; return 1; }
        std::printf("[PASS] scenario-Z (base 32B-quantised, stride 4B-quantised): "
                    "wide-fetch alignment holds by construction\n");
    }

    std::printf("[PASS] tb_framebuffer_pixel\n");
    delete dut;
    return 0;
}
