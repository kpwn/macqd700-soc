// tb_dafb_mode_matrix.cpp -- walks the WHOLE Quadra 700 DAFB mode x depth
// matrix through the production-geometry scanout path and checks every output
// pixel against an independent software model.
//
// MODE LIST PROVENANCE.  The geometries are not invented here.  Each row's
// Swatch H/V timing parameters, DAFB config/base/stride and AC842 PCBR were
// captured by booting the real Quadra 700 ROM in MAME 0.285 (`macqd700`) once
// per entry of MAME's own `monitor_config` list (src/mame/apple/dafb.cpp:205),
// with the monitor-type ioport forced to that code, and reading the DAFB
// register window back after the ROM had programmed it.  So the matrix is
// exactly the mode set the real machine offers, with the register values the
// real ROM writes -- not a set our RTL was known to accept.
//
// WHAT IS CHECKED, per (mode, depth):
//   1. the DAFB shim decodes hres/vres/base/stride the way MAME recalc_mode()
//      does (this is where a mode that "works because it equals the default"
//      is separated from one that works because the plumbing works);
//   2. scanout_placement_sync ADMITS the placement and `dafb_live` rises --
//      i.e. the boot splash retires;
//   3. no line underflow;
//   4. EVERY captured output pixel equals the software model: MAME
//      screen_update()'s pixel formula over an address-derived VRAM pattern,
//      through the CLUT the harness programmed via the real RAMDAC protocol,
//      placed by scanout_display's documented Bresenham + letterbox.
//
// NEGATIVE CONTROLS (a matrix that reports green while measuring nothing is
// the expensive failure here, so both are wired in and both are run by the
// Makefile target):
//   MATRIX_BREAK=<name>  -- programs a deliberately WRONG HFP for that one
//                           mode, so the DUT renders a different width than
//                           the model expects.  That mode must go RED and
//                           every other mode must stay green.
//   MATRIX_MUTATE=1      -- flips one bit of one captured pixel.  Proves the
//                           pixel comparison is actually reading the capture.
//
// PLUMBING CONTROL: the matrix includes `synthetic-800x600`, a geometry that
// is NOT any real Q700 mode and cannot match a hardcoded default.  If it
// renders correctly, the geometry path is genuinely runtime-driven.

#include <verilated.h>
#include "Vtb_dafb_mode_matrix.h"

#include <cstdio>
#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

// ── Elaborated harness constants (must match tb_dafb_mode_matrix.v) ──
#ifndef MM_SRC_W
#define MM_SRC_W 1024
#endif
#ifndef MM_SRC_H
#define MM_SRC_H 768
#endif
static const int      SRC_W         = MM_SRC_W;
static const int      SRC_H         = MM_SRC_H;
// Aperture + address width.  The DEFAULTS are the 2 MiB / 21-bit URAM
// build; the SHIPPING bitstream is VRAM_IN_DDR, which is 4 MiB / 22-bit
// (rtl/soc/fpga_top_video.vh FB_MAX_PIXELS / FB_ADDR_W).  Those two
// numbers are inputs to the placement admission gate and to every address
// the fetcher forms, so testing only the 2 MiB shape leaves the
// configuration that actually runs on hardware untested.  Overridable so
// `make tb-dafb-mode-matrix-ddr-aperture` can walk the same matrix at the
// shipping numbers.
#ifndef MM_FB_MAX_PIXELS
#define MM_FB_MAX_PIXELS 2097152u
#endif
#ifndef MM_ADDR_W
#define MM_ADDR_W 21
#endif
static const uint32_t FB_MAX_PIXELS = MM_FB_MAX_PIXELS;
static const int      ADDR_W        = MM_ADDR_W;
static const int      DST_W         = 1920;
static const int      DST_H         = 1080;

static const int H_TOTAL = 2200;
static const int V_TOTAL = 1125;

// DAFB register byte offsets inside the 0xF9800000 window.
static const uint32_t REG_BASE_HI = 0x000;
static const uint32_t REG_BASE_LO = 0x004;
static const uint32_t REG_STRIDE  = 0x008;
static const uint32_t REG_CONFIG  = 0x010;
static const uint32_t REG_RDAC_A  = 0x200;
static const uint32_t REG_RDAC_D  = 0x210;
static const uint32_t REG_PCBR    = 0x220;
static const uint32_t REG_HAL     = 0x140;
static const uint32_t REG_HFP     = 0x144;
static const uint32_t REG_VAL     = 0x15C;
static const uint32_t REG_VFP     = 0x160;

static Vtb_dafb_mode_matrix* dut = nullptr;
static int failures = 0;
static int passes   = 0;

static void check(bool cond, const char* fmt, ...) __attribute__((format(printf,2,3)));
static void check(bool cond, const char* fmt, ...) {
    char buf[512];
    va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof buf, fmt, ap); va_end(ap);
    if (cond) { passes++;  printf("    ok  : %s\n", buf); }
    else      { failures++; printf("    FAIL: %s\n", buf); }
}

static void tick() {
    dut->pclk = 0; dut->eval();
    dut->pclk = 1; dut->eval();
}

static void axi_write(uint32_t addr, uint32_t data) {
    dut->dafb_awaddr  = addr;
    dut->dafb_wdata   = data;
    dut->dafb_wstrb   = 0xF;
    dut->dafb_awvalid = 1;
    dut->dafb_wvalid  = 1;
    dut->dafb_bready  = 1;
    bool aw_done = false, w_done = false;
    for (int i = 0; i < 64 && !(aw_done && w_done); i++) {
        bool aw_hit = (!aw_done) && dut->dafb_awready;
        bool w_hit  = (!w_done)  && dut->dafb_wready;
        tick();
        if (aw_hit) { aw_done = true; dut->dafb_awvalid = 0; }
        if (w_hit)  { w_done  = true; dut->dafb_wvalid  = 0; }
        dut->eval();
    }
    dut->dafb_awvalid = 0;
    dut->dafb_wvalid  = 0;
    for (int i = 0; i < 64; i++) { tick(); if (dut->dafb_bvalid) break; }
    tick();
    dut->dafb_bready = 0;
    tick();
}

// ── The VRAM pattern.  Must match tb_dafb_mode_matrix.v `vram_byte`. ──
static inline uint8_t vram_byte(uint32_t a) {
    uint32_t hi = (a >> 16) & ((1u << (ADDR_W - 16)) - 1u);
    return (uint8_t)((a & 0xff) + ((a >> 8) & 0xff) * 3u + hi * 7u + 17u);
}

// ── The CLUT the harness programs, and its mono-monitor image ────────
// A deliberately ASYMMETRIC palette: entry i is (i, 255-i, (i*7)&0xff).  A
// grey ramp would let an R/G/B lane swap pass; this cannot.
static inline uint32_t clut_entry_colour(int i) {
    uint32_t r = (uint32_t)(i & 0xff);
    uint32_t g = (uint32_t)((255 - i) & 0xff);
    uint32_t b = (uint32_t)((i * 7) & 0xff);
    return (r << 16) | (g << 8) | b;
}
// MAME dafb.cpp:760-767: monitor codes 1 and 3 drive the CLUT from the BLUE
// byte alone; R and G writes are discarded and blue is replicated.
static inline uint32_t clut_entry_expected(int i, bool mono) {
    uint32_t c = clut_entry_colour(i);
    if (!mono) return c;
    uint32_t b = c & 0xff;
    return (b << 16) | (b << 8) | b;
}

static void clut_program_via_ramdac() {
    for (int i = 0; i < 256; i++) {
        uint32_t c = clut_entry_colour(i);
        axi_write(REG_RDAC_A, (uint32_t)i);
        axi_write(REG_RDAC_D, (c >> 16) & 0xff);
        axi_write(REG_RDAC_D, (c >> 8) & 0xff);
        axi_write(REG_RDAC_D, c & 0xff);
    }
}

// ── The mode table (MAME macqd700 boot capture, see file header) ─────
struct Mode {
    const char* name;
    int         sense;        // 7-bit monitor code
    uint32_t    hal, hfp;     // Swatch +0x140 / +0x144
    uint32_t    val, vfp;     // Swatch +0x15C / +0x160
    uint32_t    config;       // DAFB +0x010
    uint32_t    base_hi;      // DAFB +0x000
    uint32_t    base_lo;      // DAFB +0x004
    uint32_t    pcbr_keep;    // AC842 PCBR with the depth field (bits 4:2) cleared
    uint32_t    stride_reg;   // DAFB +0x008 as the ROM left it (1bpp)
    int         want_w, want_h;   // the resolution MAME's monitor list names
    bool        real_mode;    // false => synthetic plumbing control
};

static const Mode MODES[] = {
    // name                sense  HAL    HFP    VAL    VFP    cfg   bhi  blo  pcbrK  strd   w     h   real
    {"640x480-hires",      0x06, 0x098, 0x318, 0x052, 0x412, 0x30, 0x08, 0, 0x80, 0x100,  640, 480, true},
    {"640x480-vga",        0x57, 0x088, 0x308, 0x044, 0x404, 0x30, 0x08, 0, 0x80, 0x100,  640, 480, true},
    {"832x624-16in",       0x6D, 0x08b, 0x22b, 0x052, 0x532, 0x10, 0x07, 0, 0xa0, 0x0d0,  832, 624, true},
    // base_hi 0x07 is what the ROM WRITES (MAME write tap), NOT the 0x08 a
    // post-boot readback shows -- MAME's own recalc_mode() fixup has already
    // replaced the register by then.  Using the readback here would have
    // hidden the whole bug.
    {"512x384-12in",       0x02, 0x068, 0x268, 0x028, 0x32a, 0x50, 0x07, 0, 0x80, 0x100,  512, 384, true},
    {"1152x870-21color",   0x00, 0x040, 0x160, 0x052, 0x71e, 0x10, 0x07, 0, 0xc0, 0x090, 1152, 870, true},
    {"1152x870-twopage",   0x03, 0x040, 0x160, 0x052, 0x71e, 0x10, 0x07, 0, 0xc0, 0x090, 1152, 870, true},
    {"640x870-portrait",   0x01, 0x04b, 0x18b, 0x058, 0x724, 0x10, 0x08, 0, 0xa0, 0x080,  640, 870, true},
    {"640x480-ntsc",       0x54, 0x0be, 0x5ec, 0x023, 0x203, 0x7c, 0x08, 0, 0xa1, 0x100,  640, 480, true},
    {"768x576-pal",        0x40, 0x0fe, 0x72c, 0x027, 0x267, 0x7c, 0x08, 0, 0x21, 0x100,  768, 576, true},
    // PLUMBING CONTROL -- not a real Q700 mode.  640x480 working could be a
    // hardcoded default; 800x600 cannot be.  HAL/HFP/VAL/VFP synthesised to
    // decode to exactly 800x600 through the same formula.
    {"synthetic-800x600",  0x06, 0x100, 0x420, 0x040, 0x4f0, 0x30, 0x08, 0, 0x80, 0x100,  800, 600, false},
};
static const int N_MODES = (int)(sizeof(MODES) / sizeof(MODES[0]));

struct Depth { const char* name; int bpp; uint32_t code; int shift; int bytes_per_px; };
static const Depth DEPTHS[] = {
    {"1bpp",   1, 0x00, 3, 0},
    {"2bpp",   2, 0x08, 2, 0},
    {"4bpp",   4, 0x10, 1, 0},
    {"8bpp",   8, 0x18, 0, 1},
    {"24bpp", 24, 0x1c, 0, 4},
};
static const int N_DEPTHS = 5;

// ── MAME dafb_base::recalc_mode(), reimplemented ─────────────────────
struct Decoded { uint32_t hres, vres, base, stride; };
static Decoded decode_mode(const Mode& m, uint32_t pcbr, uint32_t stride_reg) {
    Decoded d;
    uint32_t raw_h = (m.hfp - m.hal) & 0xfff;
    int cd = (int)((pcbr & 0x60) >> 5);
    if (m.config & 0x8) d.hres = ((raw_h >> cd) - 23) & 0xfff;
    else                d.hres = (raw_h << cd) & 0xfff;
    uint32_t raw_v = ((m.vfp >> 1) - (m.val >> 1)) & 0xfff;
    d.base = ((m.base_hi & 0xfff) << 9) | ((m.base_lo & 0xf) << 5);
    // MAME dafb.cpp:832-838, dafb_base::recalc_mode():
    //     "Quadra 700 programs the wrong base for the 512x384 mode and is
    //      off-by-1 on the vertical res."
    //     if ((m_hres == 512) && (m_dafb_version == 1)) { m_base = 0x1000;
    //                                                     m_vres = 384; }
    // Tested against m_hres BEFORE the clockdiv term and against m_vres
    // BEFORE the interlace doubling, which is why it sits here.
    //
    // MEASURED, not inferred: a MAME write tap on 0xF9800000..0x2F over a
    // real macqd700 ROM boot with monitor code 2 shows the ROM writing
    //     W f9800000 = 00000007   -> m_base = 7<<9 = 3584 (0x0E00)
    // while the post-boot readback of the same register is 0x00000008
    // (m_base = 0x1000) -- i.e. the value MAME's fixup substituted.  The
    // ROM's own vres decode is (0x32a>>1)-(0x28>>1) = 385, one more than
    // the 384 the fixup forces.
    if (raw_h == 512) { d.base = 0x1000; raw_v = 384; }
    d.vres = (m.config & 0x4) ? ((raw_v << 1) & 0xfff) : raw_v;
    d.stride = (m.config & 0x8) ? 1024u : (stride_reg << 2);
    return d;
}

// Bytes one visible row of this depth occupies.
static uint32_t row_span_bytes(uint32_t hres, const Depth& dp) {
    if (dp.bytes_per_px == 4) return hres * 4u;
    return (hres * (uint32_t)dp.bpp + 7u) / 8u;
}

// The stride Mac OS would need for this (mode, depth).  Kept >= the value the
// ROM left in the register so the 1bpp cases replay the ROM's own numbers.
static uint32_t stride_for(const Mode& m, const Decoded& geom, const Depth& dp) {
    uint32_t need = row_span_bytes(geom.hres, dp);
    need = (need + 31u) & ~31u;
    uint32_t rom = m.stride_reg << 2;
    return need > rom ? need : rom;
}

// ── Capture ──────────────────────────────────────────────────────────
static std::vector<uint32_t> g_cap;   // DST_W*DST_H, in DE order

static void run_frame(bool capture) {
    if (capture) { g_cap.assign((size_t)DST_W * DST_H, 0xDEADBEEFu); }
    size_t k = 0;
    for (int v = 0; v < V_TOTAL; v++) {
        for (int h = 0; h < H_TOTAL; h++) {
            dut->hcount = h;
            dut->vcount = v;
            dut->de_in  = (h < DST_W && v < DST_H) ? 1 : 0;
            dut->hs_in  = (h >= DST_W + 88 && h < DST_W + 88 + 44) ? 1 : 0;
            dut->vs_in  = (v >= DST_H + 4 && v < DST_H + 4 + 5) ? 1 : 0;
            tick();
            if (capture && dut->scanout_de && k < g_cap.size())
                g_cap[k++] = dut->scanout_rgb & 0xFFFFFFu;
        }
    }
}

// ── The software model of scanout_display's placement + pixel path ───
struct ModelCfg {
    uint32_t hres, vres, base, stride;
    int      shift;          // bpp_shift
    int      bytes_per_px;
    bool     mono;
};

// The settled INTEGER scaling policy (docs/video_path_review.md S3):
//     N = min(floor(DST_W/hres), floor(DST_H/vres)), clamped to [1,4]
// Written from the FORMULA, not from place_plan.v's comparison ladder, so
// the RTL and this model agree only if both are right.  The 3:2 rung this
// function used to carry is gone: rational-without-filtering is the 3:2
// artifact at every ratio.
static uint32_t policy_scale_n(uint32_t hres, uint32_t vres) {
    if (hres == 0 || vres == 0) return 1;
    uint32_t n = (uint32_t)DST_W / hres;
    uint32_t m = (uint32_t)DST_H / vres;
    if (m < n) n = m;
    if (n < 1) n = 1;
    if (n > 4) n = 4;
    return n;
}

static uint32_t model_pixel(const ModelCfg& c, uint32_t x_src, uint32_t y_src,
                            const uint32_t* clut) {
    if (c.bytes_per_px == 4) {
        uint32_t a = c.base + y_src * c.stride + x_src * 4u;
        return ((uint32_t)vram_byte(a + 1) << 16) |
               ((uint32_t)vram_byte(a + 2) << 8)  |
                (uint32_t)vram_byte(a + 3);
    }
    uint32_t a = c.base + y_src * c.stride + (x_src >> c.shift);
    uint8_t  by = vram_byte(a);
    uint32_t xlo = x_src & 7u;
    uint32_t idx;
    switch (c.shift) {
        case 3: idx = (by >> (7 - xlo)) & 1u; break;
        case 2: idx = (by >> (6 - 2 * (xlo & 3u))) & 3u; break;
        case 1: idx = (xlo & 1u) ? (by & 0xfu) : (by >> 4); break;
        default: idx = by; break;
    }
    return clut[idx] & 0xFFFFFFu;
}

// Build the whole expected 1920x1080 frame.
static void build_expected(const ModelCfg& c, const uint32_t* clut,
                           std::vector<uint32_t>& out) {
    out.assign((size_t)DST_W * DST_H, 0u);
    // num is a CONSTANT 1 and den is N: integer replication, by construction.
    uint32_t n   = policy_scale_n(c.hres, c.vres);
    uint32_t num = 1u;
    uint32_t den = n;
    uint32_t aw  = c.hres * n;
    uint32_t ah  = c.vres * n;
    uint32_t bx  = (uint32_t)((DST_W - (int)aw) >> 1) & 0xfff;
    uint32_t by  = (uint32_t)((DST_H - (int)ah) >> 1) & 0xfff;

    uint32_t y_acc = 0, y_src = 0;
    for (int v = 0; v < DST_H; v++) {
        bool in_y = ((uint32_t)v >= by) && ((uint32_t)v < by + ah);
        if (in_y) {
            uint32_t x_acc = 0, x_src = 0;
            for (int h = 0; h < DST_W; h++) {
                bool in_x = ((uint32_t)h >= bx) && ((uint32_t)h < bx + aw);
                if (!in_x) continue;
                out[(size_t)v * DST_W + h] = model_pixel(c, x_src, y_src, clut);
                if (x_acc + num >= den) {
                    x_acc = x_acc + num - den;
                    if (x_src != (uint32_t)(SRC_W - 1)) x_src++;
                } else {
                    x_acc += num;
                }
            }
            if (y_acc + num >= den) {
                y_acc = y_acc + num - den;
                if (y_src != (uint32_t)(SRC_H - 1)) y_src++;
            } else {
                y_acc += num;
            }
        }
    }
}

// ── One (mode, depth) combo ──────────────────────────────────────────
struct Result { bool skipped; bool ok; const char* why; };

static const char* g_break_mode = nullptr;
static bool        g_mutate     = false;

// `from` / `stride_first` drive the DEPTH-TRANSITION variant (task #204's
// distinction: steady state vs. the switch).  When `from` is non-null the
// DUT is first reset and programmed for THAT depth, four frames are run,
// and only then are the two registers a depth change actually touches --
// AC842 PCBR (+0x220) and the DAFB stride (+0x008) -- rewritten, in the
// order `stride_first` selects, with the DUT never reset in between.  That
// is what Mac OS does when the user picks a new depth in Monitors, and it
// is the one thing a per-combo reset can never reproduce: every gate in
// scanout_placement_sync is a HOLD-LAST-GOOD gate, so a transition that
// passes through an inadmissible (depth, stride) pair behaves differently
// from a cold start into the final pair.
//
// `stride_override` replaces the minimal stride this harness would compute
// with a caller-supplied pitch, for the padded pitches Mac OS actually
// programs (task #161 measured 640x480x24bpp as base 4096 / stride 4096 on
// hardware -- 4096, not the 2560 the payload needs).
static Result run_combo(const Mode& m, const Depth& dp, bool verbose,
                        const Depth* from = nullptr, bool stride_first = false,
                        uint32_t stride_override = 0) {
    Result res{false, true, ""};
    uint32_t pcbr = m.pcbr_keep | dp.code;
    Decoded geom0 = decode_mode(m, pcbr, m.stride_reg);
    uint32_t stride = stride_override ? stride_override : stride_for(m, geom0, dp);
    Decoded geom = decode_mode(m, pcbr, stride >> 2);

    // Does this (mode, depth) fit the real Q700's 2 MiB of VRAM?
    uint32_t span  = row_span_bytes(geom.hres, dp);
    uint64_t last  = (uint64_t)geom.base + (uint64_t)geom.stride * (geom.vres - 1) + span - 1;
    bool fits = (last < (uint64_t)FB_MAX_PIXELS);

    printf("  %-20s %-6s%s hres=%u vres=%u base=%u stride=%u span=%u last=%llu %s\n",
           m.name, dp.name,
           from ? (stride_first ? " [<-,strideFirst]" : " [<-,pcbrFirst]") : "",
           geom.hres, geom.vres, geom.base, geom.stride, span,
           (unsigned long long)last, fits ? "FITS-2MB" : "EXCEEDS-2MB(not offered)");
    if (!fits) { res.skipped = true; res.why = "depth does not fit 2 MiB VRAM"; return res; }
    // Convolution (config bit 3) pins the stride at 1024 in MAME's own
    // screen_update() (dafb.cpp:266), so any depth whose visible row span
    // exceeds 1024 bytes cannot be scanned in MAME either.  Not a gap in our
    // RTL -- an unreachable combination of the encoder modes.
    if ((m.config & 0x8) && span > 1024u) {
        printf("    (skipped: convolution pins stride at 1024, row span %u -- "
               "combination unreachable in MAME too)\n", span);
        res.skipped = true; res.why = "convolution stride 1024"; return res;
    }

    // ── Program the DUT ──────────────────────────────────────────────
    dut->rst = 1;
    dut->monitor_sense = m.sense;
    for (int i = 0; i < 64; i++) tick();
    dut->rst = 0;
    for (int i = 0; i < 64; i++) tick();

    clut_program_via_ramdac();

    axi_write(REG_HAL, m.hal);
    uint32_t hfp = m.hfp;
    if (g_break_mode && !strcmp(g_break_mode, m.name)) hfp = m.hfp - 0x20;  // sentinel
    axi_write(REG_HFP, hfp);
    axi_write(REG_VAL, m.val);
    axi_write(REG_VFP, m.vfp);
    axi_write(REG_BASE_HI, m.base_hi);
    axi_write(REG_BASE_LO, m.base_lo);
    axi_write(REG_CONFIG, m.config);

    if (from) {
        // Settle into the STARTING depth first -- full four frames, so the
        // placement is genuinely committed and dafb_live is up.
        const uint32_t from_pcbr = m.pcbr_keep | from->code;
        Decoded fgeom0 = decode_mode(m, from_pcbr, m.stride_reg);
        const uint32_t from_stride = stride_for(m, fgeom0, *from);
        axi_write(REG_STRIDE, from_stride >> 2);
        axi_write(REG_PCBR,   from_pcbr);
        for (int f = 0; f < 4; f++) run_frame(false);

        // Now the switch, with NO reset.  Two frames between the two
        // register writes: long enough that the intermediate (depth,
        // stride) pair is really presented to the placement gate for a
        // frame boundary, which is the whole point.
        if (stride_first) {
            axi_write(REG_STRIDE, stride >> 2);
            for (int f = 0; f < 2; f++) run_frame(false);
            axi_write(REG_PCBR, pcbr);
        } else {
            axi_write(REG_PCBR, pcbr);
            for (int f = 0; f < 2; f++) run_frame(false);
            // ── NON-VACUITY CHECK for this whole pass ────────────────
            // The PCBR-first ordering is only interesting if the DUT
            // really did spend a frame boundary holding the NEW depth
            // against the OLD stride.  When that pair is inadmissible
            // (the new depth needs a wider row than the old stride
            // provides) the placement gate MUST have rejected it and MUST
            // still be showing the old depth.  Without this, a harness
            // whose two writes landed too close together would report a
            // green "transition" that never transitioned.
            const uint32_t need_to = row_span_bytes(geom.hres, dp);
            if (from_stride < need_to) {
                check(dut->dbg_ps_stride_sane == 0,
                      "%s/%s [<-%s,pcbrFirst] intermediate (new depth, old "
                      "stride %u < %u) must be REJECTED by the placement gate",
                      m.name, dp.name, from->name, from_stride, need_to);
                check(dut->dbg_committed_bytes_per_px == (unsigned)from->bytes_per_px,
                      "%s/%s [<-%s,pcbrFirst] intermediate holds the LAST GOOD "
                      "depth (committed bppx=%u, want %d)",
                      m.name, dp.name, from->name,
                      (unsigned)dut->dbg_committed_bytes_per_px, from->bytes_per_px);
            }
            axi_write(REG_STRIDE, stride >> 2);
        }
    } else {
        axi_write(REG_STRIDE, stride >> 2);
        axi_write(REG_PCBR,   pcbr);
    }

    for (int f = 0; f < 4; f++) run_frame(false);

    // ── Gate 1: the shim decoded the mode MAME's way ─────────────────
    check(dut->dbg_shim_hres == geom.hres,
          "%s/%s shim hres=%u (want %u)", m.name, dp.name,
          (unsigned)dut->dbg_shim_hres, geom.hres);
    check(dut->dbg_shim_vres == geom.vres,
          "%s/%s shim vres=%u (want %u)", m.name, dp.name,
          (unsigned)dut->dbg_shim_vres, geom.vres);
    check(dut->dbg_shim_base == geom.base,
          "%s/%s shim base=%u (want %u)", m.name, dp.name,
          (unsigned)dut->dbg_shim_base, geom.base);
    check(dut->dbg_shim_stride == geom.stride,
          "%s/%s shim stride=%u (want %u)", m.name, dp.name,
          (unsigned)dut->dbg_shim_stride, geom.stride);
    check(dut->dbg_shim_depth_supported == 1,
          "%s/%s depth_supported", m.name, dp.name);

    // ── Gate 2: the placement was admitted and the splash retired ────
    check(dut->dbg_ps_stride_sane == 1, "%s/%s placement stride_sane", m.name, dp.name);
    check(dut->dbg_ps_in_range == 1,
          "%s/%s placement in_range (frame_last_addr=%u limit=%u)", m.name, dp.name,
          (unsigned)dut->dbg_ps_frame_last_addr, FB_MAX_PIXELS);
    check(dut->dbg_dafb_live == 1,
          "%s/%s dafb_live (boot splash retired)", m.name, dp.name);
    check(dut->dbg_committed_hres == geom.hres,
          "%s/%s committed hres=%u (want %u)", m.name, dp.name,
          (unsigned)dut->dbg_committed_hres, geom.hres);
    check(dut->dbg_committed_vres == geom.vres,
          "%s/%s committed vres=%u (want %u)", m.name, dp.name,
          (unsigned)dut->dbg_committed_vres, geom.vres);

    // ── Gate 3: the fetch engine agrees on the geometry ──────────────
    uint32_t eff_hres = (geom.hres == 0 || geom.hres >= (uint32_t)SRC_W)
                            ? (uint32_t)SRC_W : geom.hres;
    uint32_t eff_vres = (geom.vres == 0 || geom.vres >= (uint32_t)SRC_H)
                            ? (uint32_t)SRC_H : geom.vres;
    uint32_t want_px_last = (dp.bytes_per_px == 4) ? (eff_hres - 1)
                                                   : ((eff_hres - 1) >> dp.shift);
    check(dut->dbg_fetch_row_px_last == want_px_last,
          "%s/%s fetch row_px_last=%u (want %u)", m.name, dp.name,
          (unsigned)dut->dbg_fetch_row_px_last, want_px_last);
    check(dut->dbg_fetch_row_y_last == eff_vres - 1,
          "%s/%s fetch row_y_last=%u (want %u)", m.name, dp.name,
          (unsigned)dut->dbg_fetch_row_y_last, eff_vres - 1);
    check(dut->dbg_fetch_stride_sane == 1, "%s/%s fetch stride_sane", m.name, dp.name);
    check(dut->dbg_fetch_frame_in_mem == 1, "%s/%s fetch frame_in_mem", m.name, dp.name);

    // ── Gate 3b: the elaborated scanner bound holds the whole mode ────
    // A mode wider/taller than SRC_W/SRC_H is CROPPED silently -- it still
    // renders, it just renders a sub-rectangle of the desktop, which is why
    // the 1024x768 bound survived so long against MAME's two 1152x870
    // displays and the 640x870 Portrait.  Checked separately from the pixel
    // comparison because the pixel model reproduces the crop faithfully and
    // therefore cannot flag it.
    if (m.real_mode)
        check(eff_hres == geom.hres && eff_vres == geom.vres,
              "%s/%s fits the elaborated scanner bound SRC %dx%d "
              "(mode is %ux%u, scanner would render %ux%u)",
              m.name, dp.name, SRC_W, SRC_H, geom.hres, geom.vres,
              eff_hres, eff_vres);

    // ── Gate 4: pixel-exact frame ────────────────────────────────────
    run_frame(true);
    if (g_mutate) g_cap[(size_t)(DST_H / 2) * DST_W + DST_W / 2] ^= 0x000100u;

    check(dut->line_underflow_sticky == 0, "%s/%s no line underflow", m.name, dp.name);

    uint32_t clut[256];
    bool mono = (m.sense == 1) || (m.sense == 3);
    for (int i = 0; i < 256; i++) clut[i] = clut_entry_expected(i, mono);

    ModelCfg cfg{geom.hres, geom.vres, geom.base, geom.stride,
                 dp.shift, dp.bytes_per_px, mono};
    std::vector<uint32_t> exp;
    build_expected(cfg, clut, exp);

    long mism = 0; int first_v = -1, first_h = -1;
    uint32_t got0 = 0, want0 = 0;
    for (int v = 0; v < DST_H; v++) {
        for (int h = 0; h < DST_W; h++) {
            size_t i = (size_t)v * DST_W + h;
            if (g_cap[i] != exp[i]) {
                if (mism == 0) { first_v = v; first_h = h; got0 = g_cap[i]; want0 = exp[i]; }
                mism++;
            }
        }
    }
    if (mism)
        printf("    first mismatch at (%d,%d): got 0x%06x want 0x%06x\n",
               first_h, first_v, got0, want0);
    check(mism == 0, "%s/%s pixel-exact frame (%ld mismatched of %d)",
          m.name, dp.name, mism, DST_W * DST_H);

    (void)verbose;
    res.ok = true;
    return res;
}

// ── The 832x624 "yellow tint" referee ────────────────────────────────
// Re-measured here rather than argued from task #202's conclusion.
//
// MEASUREMENT (MAME 0.285, macqd700, real ROM boot, one run per monitor
// code, CLUT read back through the AC842 ramdac_r protocol at +0x200/+0x210):
//
//     sense 0x06 640x480 Hi-Res      CLUT[0] = ff ff ff
//     sense 0x57 640x480 VGA         CLUT[0] = ff ff ff
//     sense 0x02 512x384 12" RGB     CLUT[0] = ff ff ff
//     sense 0x01 Portrait B&W        CLUT[0] = ff ff ff
//     sense 0x03 Two-Page B&W        CLUT[0] = ff ff ff
//     sense 0x00 1152x870 21" Color  CLUT[0] = ff f7 d6   <-- warm white
//     sense 0x6D 832x624 16" RGB     CLUT[0] = ff f7 d6   <-- warm white
//
// So Mac OS itself picks a warm-white gamma entry for exactly the two large
// colour displays, and the golden model carries the identical value.  The
// question this check answers is the only one that is OUR problem: given
// that write stream, does OUR RAMDAC path publish the same triple to the
// scanner -- and does the sense code accidentally take the mono branch?
//
// It is deliberately run at BOTH sense codes: an implementation that greyed
// or warmed every entry would pass a one-sided check.
static void tint_referee() {
    struct Case { int sense; uint32_t triple; const char* what; };
    const Case cases[] = {
        {0x6D, 0xFFF7D6u, "832x624 16in RGB  -- Mac OS gamma, MAME writes the same"},
        {0x00, 0xFFF7D6u, "1152x870 21in Color -- same warm white"},
        {0x06, 0xFFFFFFu, "640x480 Hi-Res -- neutral white (the control)"},
        // A GREY triple here would make the mono branch indistinguishable from
        // the colour one, so feed the warm-white triple and require the blue
        // byte replicated: 0xD6D6D6, not 0xFFF7D6.
        {0x01, 0xFFF7D6u, "Portrait B&W -- mono branch must replicate the blue byte"},
        {0x03, 0xFFF7D6u, "Two-Page B&W -- same mono branch"},
    };
    printf("\n---- CLUT tint referee (measured against MAME per sense code) ----\n");
    for (const Case& c : cases) {
        dut->rst = 1; dut->monitor_sense = c.sense;
        for (int i = 0; i < 32; i++) tick();
        dut->rst = 0;
        for (int i = 0; i < 32; i++) tick();
        uint32_t r = (c.triple >> 16) & 0xff;
        uint32_t g = (c.triple >> 8) & 0xff;
        uint32_t b = c.triple & 0xff;
        axi_write(REG_RDAC_A, 0);
        axi_write(REG_RDAC_D, r);
        axi_write(REG_RDAC_D, g);
        uint32_t seen = 0;
        // Capture the export pulse that the BLUE byte completes.
        dut->dafb_awaddr = REG_RDAC_D; dut->dafb_wdata = b; dut->dafb_wstrb = 0xF;
        dut->dafb_awvalid = 1; dut->dafb_wvalid = 1; dut->dafb_bready = 1;
        bool awd = false, wd = false;
        for (int i = 0; i < 64; i++) {
            bool ah = (!awd) && dut->dafb_awready;
            bool wh = (!wd)  && dut->dafb_wready;
            tick();
            if (dut->dbg_clut_we && dut->dbg_clut_waddr == 0) seen = dut->dbg_clut_wdata;
            if (ah) { awd = true; dut->dafb_awvalid = 0; }
            if (wh) { wd  = true; dut->dafb_wvalid  = 0; }
            dut->eval();
        }
        dut->dafb_awvalid = 0; dut->dafb_wvalid = 0;
        for (int i = 0; i < 16; i++) {
            tick();
            if (dut->dbg_clut_we && dut->dbg_clut_waddr == 0) seen = dut->dbg_clut_wdata;
        }
        dut->dafb_bready = 0; tick();
        bool mono = (c.sense == 1) || (c.sense == 3);
        uint32_t want = mono ? ((b << 16) | (b << 8) | b) : c.triple;
        check(seen == want,
              "sense 0x%02X: CLUT[0] published to the scanner = 0x%06X (want 0x%06X) -- %s",
              c.sense, seen, want, c.what);
    }
    printf("---- end tint referee ----\n\n");
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_dafb_mode_matrix;

    g_break_mode = getenv("MATRIX_BREAK");
    if (g_break_mode && !*g_break_mode) g_break_mode = nullptr;
    g_mutate = getenv("MATRIX_MUTATE") && atoi(getenv("MATRIX_MUTATE"));
    const char* only_mode  = getenv("MATRIX_MODE");
    const char* only_depth = getenv("MATRIX_DEPTH");

    if (g_break_mode) printf("[negative control] BREAKING mode '%s' (HFP -0x20)\n", g_break_mode);
    if (g_mutate)     printf("[negative control] MUTATING one captured pixel per combo\n");

    tint_referee();

    printf("Q700 DAFB mode x depth matrix -- scanner elaborated at "
           "SRC %dx%d, aperture 0x%06x, DST %dx%d\n",
           SRC_W, SRC_H, FB_MAX_PIXELS, DST_W, DST_H);

    for (int mi = 0; mi < N_MODES; mi++) {
        const Mode& m = MODES[mi];
        if (only_mode && *only_mode && strcmp(only_mode, m.name)) continue;
        for (int di = 0; di < N_DEPTHS; di++) {
            const Depth& dp = DEPTHS[di];
            if (only_depth && *only_depth && strcmp(only_depth, dp.name)) continue;
            run_combo(m, dp, false);
        }
    }

    // ══ PASS 2: DEPTH TRANSITIONS (task #204) ═════════════════════════
    // Everything above resets the DUT per combo, so it measures the
    // STEADY STATE of each (mode, depth) and structurally cannot see a
    // transition defect.  Mac OS never resets the DAFB to change depth --
    // it rewrites PCBR and the stride on a live scanner.  Both orderings
    // are exercised because the intermediate pair differs:
    //   PCBR first  -> new depth with the OLD (too small) stride, which
    //                  the placement gate must REJECT and hold last-good;
    //   stride first-> old depth with the NEW (too large) stride, which
    //                  the gate ACCEPTS, so a frame renders at the wrong
    //                  pitch before the depth catches up.
    // A correct engine lands on the same pixel-exact frame as the cold
    // start either way.
    printf("\n---- PASS 2: depth transitions (no reset between depths) ----\n");
    static const int T_FROM[] = {3, 4, 3, 0};   // 8bpp, 24bpp, 8bpp, 1bpp
    static const int T_TO[]   = {4, 3, 0, 4};   // 24bpp, 8bpp, 1bpp, 24bpp
    for (int mi = 0; mi < N_MODES; mi++) {
        const Mode& m = MODES[mi];
        if (only_mode && *only_mode && strcmp(only_mode, m.name)) continue;
        for (int ti = 0; ti < 4; ti++) {
            const Depth& from = DEPTHS[T_FROM[ti]];
            const Depth& to   = DEPTHS[T_TO[ti]];
            if (only_depth && *only_depth && strcmp(only_depth, to.name)) continue;
            for (int sf = 0; sf < 2; sf++)
                run_combo(m, to, false, &from, sf != 0);
        }
    }

    // ══ PASS 3: PADDED 24bpp PITCH ════════════════════════════════════
    // Task #161 MEASURED Mac OS programming 640x480x24bpp as base 4096 /
    // stride 4096 -- not the 2560 the 640-pixel payload needs.  Pass 1
    // only ever tests the MINIMAL stride, so "pitch > payload meets direct
    // colour" was untested for every mode except the one #204 added by
    // hand for 832x624.  Here it is for the whole mode list: the stride
    // Mac OS plausibly picks, i.e. the 1bpp mode pitch scaled by 4.
    printf("\n---- PASS 3: 24bpp at the padded pitch Mac OS programs ----\n");
    for (int mi = 0; mi < N_MODES; mi++) {
        const Mode& m = MODES[mi];
        if (only_mode && *only_mode && strcmp(only_mode, m.name)) continue;
        if (only_depth && *only_depth && strcmp(only_depth, "24bpp")) continue;
        const Depth& d24 = DEPTHS[4];
        const uint32_t rom_pitch = m.stride_reg << 2;
        const uint32_t padded    = rom_pitch * 4u;
        Decoded g0 = decode_mode(m, m.pcbr_keep | d24.code, m.stride_reg);
        if (padded <= stride_for(m, g0, d24)) continue;   // no padding to test
        run_combo(m, d24, false, nullptr, false, padded);
        // and the same pitch reached by a live transition from 8bpp
        run_combo(m, d24, false, &DEPTHS[3], false, padded);
        run_combo(m, d24, false, &DEPTHS[3], true,  padded);
    }

    printf("\n==== tb-dafb-mode-matrix: pass=%d fail=%d ====\n", passes, failures);
    delete dut;
    return failures ? 1 : 0;
}
