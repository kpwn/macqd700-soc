// tb_scanout_ddr_frames.cpp -- harness for the full-chain multi-frame scanout
// gate.  See tb_scanout_ddr_frames.v for WHY this rig exists.
//
// WHAT IT CHECKS, per scenario:
//
//   A. ABSOLUTE.  Every active output pixel of every checked frame equals the
//      pixel the source framebuffer says it should be.  Computed from an
//      independent C++ re-derivation of scanout_display.v's nearest-neighbour
//      mapping, not from a captured reference frame.
//
//   A2. THE PAINTED WINDOW.  (A) only compares pixels INSIDE the window the
//      checker assumed, which is blind to the SCALE: a frame rendered at the
//      wrong N is internally self-consistent over the region both agree on.
//      So the harness also measures the bounding box of every non-black
//      pixel on the full 1920x1080 raster and requires it to be the centred
//      hres*N x vres*N window.  That is the check that would have caught the
//      3:2 rung, and it is what the deliberate MUT-B ("restore the
//      fractional rung for 832x624") fails against.
//
//   B. FRAME-TO-FRAME IDENTITY.  The framebuffer is painted once, before the
//      first frame, and never written again.  Consecutive frames must
//      therefore be byte-identical.  This is the assertion the pre-existing
//      tb-scanout-frames did not have.  It is weaker than (A) in principle
//      but it is the one that matches the hardware measurement (frames diffed
//      against each other on a static framebuffer), so both are reported.
//
//   C. SLIP IDENTIFICATION.  When (A) fails, the harness searches a small
//      space of rigid source-space offsets -- dy source ROWS and dx source
//      BYTES -- for one that explains the whole frame.  This exists because
//      the two candidate mechanisms for the hardware symptom (a 64-row
//      vertical origin slip = linebuf_scanout's LINE_COUNT, and a 128-byte
//      horizontal slip = scanout_line_fetch's LINE_BYTES) are VISUALLY
//      IDENTICAL under the SMPTE bar pattern the hardware rig paints, because
//      that pattern's bar index is (x>>7 + y>>6) mod 8.  The patterns painted
//      here vary in x and y independently precisely so the two can be told
//      apart, and this search names which one happened.
//
// The pattern is deliberately NOT the SMPTE bar pattern for that reason.
#include <verilated.h>
#include "Vtb_scanout_ddr_frames.h"

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

// ── Shipping geometry ───────────────────────────────────────────────────
static const int DST_W   = 1920;
static const int DST_H   = 1080;
static const int H_TOTAL = 1920 + 88 + 44 + 148;   // 2200
static const int V_TOTAL = 1080 + 4 + 5 + 36;      // 1125
static const int SRC_W   = 1024;
static const int SRC_H   = 768;
static const uint32_t FB_BYTES = 0x0040'0000u;     // 4 MiB aperture
static const uint32_t CARVEOUT_BASE = 0x4600'0000u;
static const int LINE_COUNT = 64;                  // linebuf ring depth
static const int LINE_BYTES = 128;                 // scanout_line_fetch line

static Vtb_scanout_ddr_frames* dut = nullptr;
static int pass_count = 0, fail_count = 0;
// A scenario the elaborated scanner bound cannot represent is SKIPPED, and a
// skip contributes no checks -- so an all-skip run would read as a green
// gate that tested nothing.  Counted, printed, and failed on below.
static int ran_count = 0, skipped_count = 0;

// ── WORST-CASE ARBITRATION PRESSURE (env BULK=1) ────────────────────────
// Underflow is a VISIBLE artifact, and it does not happen in steady state:
// it happens when a CPU miss storm is queued ahead of scanout on the shared
// DDR read path at the same time as the tightest video mode.  With BULK=1
// the harness saturates axi_vram_priority_mux3's l2c_* AR port for the
// whole frame collection, which is the only condition under which
// MAX_BULK_AHEAD is ever reached.  It also reports the bulk side's own
// service rate, so the CPU-visible cost of a given MAX_BULK_AHEAD is
// measured rather than assumed.
// Default: OFF for the geometry sweep (it would multiply an already slow
// gate's runtime by the storm's own traffic), ON for the mode-transition
// scenario, whose entire reason to exist is the worst case.  BULK=1 turns
// it on everywhere; BULK=0 turns it off everywhere.
static bool bulk_enable = false;
static bool bulk_enable_transition = true;

static void check(bool ok, const std::string& what) {
    if (ok) { pass_count++; printf("  [PASS] %s\n", what.c_str()); }
    else    { fail_count++; printf("  [FAIL] %s\n", what.c_str()); }
}

// ── Clocking ────────────────────────────────────────────────────────────
// Three independent nets at the PRODUCTION frequency ratios.  Time unit is
// 5 ps; pclk 148.5 MHz -> 6.734 ns -> 1347 units, core_clk 100 MHz -> 2000,
// mig_clk 200 MHz -> 1000.  The ratio matters (see the .v header): a 1:1 rig
// never lets the pclk request stream outrun the core_clk fetch engine.
static const uint64_t P_PCLK = 1347, P_CORE = 2000, P_MIG = 1000;
static uint64_t t_now = 0;
static uint64_t next_p = 0, next_c = 0, next_m = 0;

static void (*post_pclk_hook)() = nullptr;

// Advance to the next clock edge of any domain.  Returns true if that edge
// was a pclk RISING edge (the harness samples video output there).
static bool step_edge() {
    uint64_t t = next_p; if (next_c < t) t = next_c; if (next_m < t) t = next_m;
    bool pclk_rise = false;
    t_now = t;
    if (next_p == t) { dut->pclk     = !dut->pclk;     next_p += P_PCLK / 2;
                       pclk_rise = dut->pclk; }
    if (next_c == t) { dut->core_clk = !dut->core_clk; next_c += P_CORE / 2; }
    if (next_m == t) { dut->mig_clk  = !dut->mig_clk;  next_m += P_MIG  / 2; }
    dut->eval();
    return pclk_rise;
}

// ── Model framebuffer (golden) ──────────────────────────────────────────
static std::vector<uint8_t> fbmem;

// ── Scenario ────────────────────────────────────────────────────────────
struct Scenario {
    const char* name;
    int      hres, vres;
    // NO per-scenario scale field.  The output scale is DERIVED from the
    // geometry by policy_scale_n() below and driven into the DUT, so a
    // scenario cannot state a ratio the shipping policy would not pick --
    // which is exactly how the 3:2 rung stayed under test for so long after
    // it was known to alias.  See docs/video_path_review.md S3.
    int      bpp_shift;      // 0 = 8bpp / 24bpp, 3 = 1bpp
    int      bytes_per_px;   // 1 = indexed, 4 = 24bpp direct
    uint32_t base;
    uint32_t stride;
    int      frames;         // frames collected
    int      first_checked;  // frames before this are warm-up
    // Second name SCAN_ONLY also accepts.  Exists so Makefile targets that
    // select a scenario by its pre-integer-policy name (the line-start
    // negative control asks for "24bpp-832x624-3_2") keep selecting the same
    // MODE after the ratio disappeared from the name.  Never edit the
    // Makefile to chase a scenario rename: add the old name here.
    const char* alias = nullptr;
};

// ── Destination -> source index map (scanout_display.v's Bresenham) ────
// Same accumulator and same update order as the RTL, kept in the general
// p/q shape rather than collapsed to "divide by N" so that a fractional
// walk -- which emits src 0,0,1,2,2,3,4,4,5,... -- is REPRESENTABLE here and
// therefore detectable.  map_is_integer_replication() below then asserts the
// result is a uniform staircase, which a fractional ratio is not.
static std::vector<int> bresenham_map(int dst_n, int num, int den) {
    std::vector<int> m((size_t)dst_n);
    int acc = 0, s = 0;
    for (int i = 0; i < dst_n; i++) {
        m[(size_t)i] = s;
        if (acc + num >= den) { acc = acc + num - den; s++; }
        else                  { acc = acc + num; }
    }
    return m;
}

// ── The settled INTEGER scaling policy, re-derived independently ────────
// docs/video_path_review.md S3:
//     N = min(floor(DST_W / hres), floor(DST_H / vres)), clamped to >= 1
// Written from that FORMULA, deliberately NOT from place_plan.v's comparison
// ladder, so the RTL and the checker agree only if both are right.  Capped
// at 4 to match the RTL's ladder ceiling.
static int policy_scale_n(int hres, int vres) {
    if (hres <= 0 || vres <= 0) return 1;
    int n = DST_W / hres;
    const int m = DST_H / vres;
    if (m < n) n = m;
    if (n < 1) n = 1;
    if (n > 4) n = 4;
    return n;
}

// num is 1 and den is N -- integer replication, by construction.  Kept in
// the general Bresenham shape the RTL uses so the two are comparable.
static int scale_num_of(int)    { return 1; }
static int scale_den_of(int n)  { return n; }

// Active window edge = hres * N.  There is no floored-halving case any more.
static int active_span(int res, int n) { return res * n; }

// The map an INTEGER ratio must produce: source index i/N for every display
// index i, i.e. runs of exactly N.  A 3:2 (or any p/q) walk produces runs of
// 2,1,2,1..., which this rejects.  Model-side invariant -- it guards the
// checker itself against being quietly re-fractionalised.
static bool map_is_integer_replication(const std::vector<int>& m, int n) {
    if (m.empty() || n < 1) return false;
    for (size_t i = 0; i < m.size(); i++)
        if (m[i] != (int)(i / (size_t)n)) return false;
    return true;
}

static std::vector<int> xmap, ymap;

static uint32_t clut_entry(int i) {
    // Injective over 0..255 and never black, so "blank" and "palette 0" are
    // always distinguishable.
    return (uint32_t)(((i & 0xFF) << 16) | (((255 - i) & 0xFF) << 8)
                      | (((i * 7 + 0x11) & 0xFF)));
}

// Source BYTE at (byte-index bx, row y).  Varies in BOTH axes and has no
// period that divides 64 rows or 128 bytes, so a rigid slip in either axis is
// detectable AND distinguishable.
static uint8_t src_byte(int bx, int y) {
    return (uint8_t)((y * 181 + bx * 7 + 13) & 0xFF);
}

static void paint(const Scenario& s, bool clear = true) {
    if (clear) std::fill(fbmem.begin(), fbmem.end(), 0);
    for (int y = 0; y < s.vres; y++) {
        uint32_t row = s.base + (uint32_t)y * s.stride;
        if (s.bytes_per_px == 4) {
            for (int x = 0; x < s.hres; x++) {
                uint32_t a = row + 4u * (uint32_t)x;
                if (a + 3 >= FB_BYTES) break;
                fbmem[a + 0] = 0x00;                     // pad
                fbmem[a + 1] = src_byte(4 * x + 1, y);   // R
                fbmem[a + 2] = src_byte(4 * x + 2, y);   // G
                fbmem[a + 3] = src_byte(4 * x + 3, y);   // B
            }
        } else if (s.bpp_shift == 3) {
            int bytes = (s.hres + 7) / 8;
            for (int b = 0; b < bytes; b++) {
                uint32_t a = row + (uint32_t)b;
                if (a >= FB_BYTES) break;
                fbmem[a] = src_byte(b, y);
            }
        } else {
            for (int x = 0; x < s.hres; x++) {
                uint32_t a = row + (uint32_t)x;
                if (a >= FB_BYTES) break;
                fbmem[a] = src_byte(x, y);
            }
        }
    }
}

// Expected output RGB for source pixel (x_src, y_src) under scenario s, with
// the whole source image rigidly displaced by (dx source BYTES, dy source
// ROWS).  (0,0) is the correct image.
static uint32_t expect_px(const Scenario& s, int x_src, int y_src,
                          int dx, int dy) {
    int y = y_src + dy;
    if (y < 0) return 0xFFFFFFFFu;
    if (s.bytes_per_px == 4) {
        int bx = 4 * x_src + dx;
        if (bx < 0) return 0xFFFFFFFFu;
        uint32_t a = s.base + (uint32_t)y * s.stride + (uint32_t)bx;
        if (a + 3 >= FB_BYTES) return 0xFFFFFFFFu;
        return ((uint32_t)fbmem[a + 1] << 16) | ((uint32_t)fbmem[a + 2] << 8)
             | (uint32_t)fbmem[a + 3];
    }
    int bx = (x_src >> s.bpp_shift) + dx;
    if (bx < 0) return 0xFFFFFFFFu;
    uint32_t a = s.base + (uint32_t)y * s.stride + (uint32_t)bx;
    if (a >= FB_BYTES) return 0xFFFFFFFFu;
    uint8_t b = fbmem[a];
    int idx;
    if (s.bpp_shift == 3)      idx = (b >> (7 - (x_src & 7))) & 1;
    else if (s.bpp_shift == 2) idx = (b >> (6 - 2 * (x_src & 3))) & 3;
    else if (s.bpp_shift == 1) idx = (x_src & 1) ? (b & 0xF) : (b >> 4);
    else                       idx = b;
    return clut_entry(idx);
}

// ── Frame capture ───────────────────────────────────────────────────────
struct Frame {
    // Full active-window pixel record: [line][col] over the ACTIVE window
    // only.  active_h * active_w <= 1280*960 = 1.2 M entries, 5 MB.
    std::vector<uint32_t> px;
    int  aw = 0, ah = 0;
    long lit = 0;
    bool complete = false;
    // Bounding box of every NON-BLACK pixel over the WHOLE 1920x1080 raster,
    // not just the window the checker assumed.  This is the only measurement
    // in the file that does not presuppose the placement: it is what the
    // DUT actually painted, so it catches a wrong scale N or a wrong border
    // even though every pixel inside the assumed window is correct.
    int  bx0 = 1 << 30, bx1 = -1, by0 = 1 << 30, by1 = -1;
};

static std::vector<Frame> frames;
static Frame cur;
static int cap_border_x = 0, cap_border_y = 0, cap_aw = 0, cap_ah = 0;
static int px_idx = 0;

static void capture_tick() {
    if (dut->de_out) {
        int col = px_idx++;
        int v = dut->vcount;
        if (dut->rgb & 0xFFFFFFu) {
            if (col < cur.bx0) cur.bx0 = col;
            if (col > cur.bx1) cur.bx1 = col;
            if (v   < cur.by0) cur.by0 = v;
            if (v   > cur.by1) cur.by1 = v;
        }
        if (col >= cap_border_x && col < cap_border_x + cap_aw &&
            v >= cap_border_y && v < cap_border_y + cap_ah) {
            uint32_t c = dut->rgb & 0xFFFFFFu;
            cur.px[(size_t)(v - cap_border_y) * cap_aw + (col - cap_border_x)] = c;
            if (c) cur.lit++;
        }
    } else {
        px_idx = 0;
    }
}

// ── CPU-side AXI write driver (paints DRAM before the frames start) ─────
struct PendingW { bool aw_done = false, w_done = false, done = false; };
static PendingW* w_cur = nullptr;
static uint32_t w_next_id = 1;

static void cpu_w_drive_and_latch() {
    if (w_cur) {
        dut->cpu_awvalid = w_cur->aw_done ? 0 : 1;
        dut->cpu_wvalid  = w_cur->w_done  ? 0 : 1;
    } else {
        dut->cpu_awvalid = 0;
        dut->cpu_wvalid  = 0;
    }
    dut->cpu_bready = 1;
    if (w_cur) {
        if (!w_cur->aw_done && dut->cpu_awvalid && dut->cpu_awready) w_cur->aw_done = true;
        // AXI4 W-channel contract: VALID must drop after a completed
        // handshake.  (tb_fb_reader_ddr_chain.cpp's header records the day
        // this rule was learned the hard way.)
        if (!w_cur->w_done && dut->cpu_wvalid && dut->cpu_wready) w_cur->w_done = true;
        if (dut->cpu_bvalid && dut->cpu_bready) w_cur->done = true;
    }
}

// Tick until the core_clk edge count advances, running the write driver on
// each core_clk rising edge.
static void tick_write() {
    bool prev_core = dut->core_clk;
    for (int guard = 0; guard < 64; guard++) {
        step_edge();
        if (!prev_core && dut->core_clk) { cpu_w_drive_and_latch(); dut->eval(); return; }
        prev_core = dut->core_clk;
    }
}

static bool write_beat(uint32_t addr, const uint8_t* b16) {
    PendingW pw;
    w_cur = &pw;
    dut->cpu_awid    = (w_next_id++) & 0x3F;
    dut->cpu_awaddr  = addr;
    dut->cpu_awlen   = 0;
    dut->cpu_awsize  = 4;
    dut->cpu_awburst = 1;
    for (int w = 0; w < 4; w++) {
        dut->cpu_wdata[w] = (uint32_t)b16[w * 4] | ((uint32_t)b16[w * 4 + 1] << 8)
                          | ((uint32_t)b16[w * 4 + 2] << 16) | ((uint32_t)b16[w * 4 + 3] << 24);
    }
    dut->cpu_wstrb = 0xFFFF;
    dut->cpu_wlast = 1;
    int guard = 40000;
    while (!pw.done && guard-- > 0) tick_write();
    w_cur = nullptr;
    dut->cpu_awvalid = 0; dut->cpu_wvalid = 0; dut->eval();
    return pw.done;
}

static bool load_fb_into_dram(const Scenario& s) {
    // Only the bytes the scanner can reach: rows 0..vres-1 of the stride
    // window, rounded out to whole 16 B beats.
    uint32_t lo = s.base;
    uint32_t hi = s.base + (uint32_t)(s.vres - 1) * s.stride + s.stride;
    if (hi > FB_BYTES) hi = FB_BYTES;
    lo &= ~15u;
    hi = (hi + 15u) & ~15u;
    for (uint32_t a = lo; a < hi; a += 16) {
        if (!write_beat(CARVEOUT_BASE + a, &fbmem[a])) return false;
    }
    return true;
}

// ── CLUT programming (pclk domain in this rig) ──────────────────────────
static void tick_pclk_n(int n) {
    for (int i = 0; i < n; i++) {
        bool rise = false;
        while (!rise) rise = step_edge();
    }
}

static void program_clut() {
    for (int i = 0; i < 256; i++) {
        dut->clut_we = 1; dut->clut_waddr = (uint8_t)i; dut->clut_wdata = clut_entry(i);
        tick_pclk_n(1);
    }
    dut->clut_we = 0;
    tick_pclk_n(2);
}

// ── Scenario runner ─────────────────────────────────────────────────────
static void run_scenario(const Scenario& s) {
    const int n = policy_scale_n(s.hres, s.vres);
    printf("\n== scenario %s: %dx%d %dx integer, %s, base=0x%x stride=%u, %d frames ==\n",
           s.name, s.hres, s.vres, n,
           (s.bytes_per_px == 4) ? "24bpp" : (s.bpp_shift == 3 ? "1bpp" : "8bpp"),
           s.base, s.stride, s.frames);

    // A mode wider or taller than the ELABORATED scanner bound would be
    // silently cropped, and the checker would then measure the crop.  Skip
    // loudly rather than report a green number for a mode that never ran.
    if (s.hres > (int)dut->dbg_src_w || s.vres > (int)dut->dbg_src_h) {
        printf("  SKIPPED: needs a scanner elaborated at >= %dx%d; this build is %ux%u\n",
               s.hres, s.vres, (unsigned)dut->dbg_src_w, (unsigned)dut->dbg_src_h);
        skipped_count++;
        return;
    }
    ran_count++;

    paint(s);

    cap_aw = active_span(s.hres, n);
    cap_ah = active_span(s.vres, n);
    xmap = bresenham_map(cap_aw, scale_num_of(n), scale_den_of(n));
    ymap = bresenham_map(cap_ah, scale_num_of(n), scale_den_of(n));
    cap_border_x = (DST_W - cap_aw) / 2;
    cap_border_y = (DST_H - cap_ah) / 2;

    // The checker's own map must be an integer staircase.  If this ever
    // fails, every pixel result below is measuring the wrong contract.
    check(map_is_integer_replication(xmap, n) && map_is_integer_replication(ymap, n),
          std::string(s.name) + ": expected map is integer replication x" +
          std::to_string(n) + " on both axes");

    // ── Reset ──
    dut->rst = 1; dut->mig_rst = 1; dut->scan_hold = 1;
    dut->fb_base_px = s.base; dut->fb_stride_px = s.stride;
    dut->bpp_shift = s.bpp_shift; dut->bytes_per_px = s.bytes_per_px;
    dut->hres = s.hres; dut->vres = s.vres; dut->scale_n = n;
    dut->clut_we = 0;
    dut->cpu_awvalid = 0; dut->cpu_wvalid = 0; dut->cpu_bready = 1;
    dut->cpu_awid = 0; dut->cpu_awaddr = 0; dut->cpu_awlen = 0;
    dut->cpu_awsize = 4; dut->cpu_awburst = 1; dut->cpu_wstrb = 0; dut->cpu_wlast = 0;
    tick_pclk_n(20);
    dut->mig_rst = 0;
    for (int g = 0; g < 20000 && !dut->cal_done; g++) tick_pclk_n(1);
    check(dut->cal_done != 0, std::string(s.name) + ": MIG model calibrated");
    if (!dut->cal_done) return;

    // Release the FABRIC (mux/bridge/MIG) but keep the SCANNER held, so the
    // paint below has the DDR port to itself.
    dut->rst = 0;
    tick_pclk_n(8);

    // Paint DRAM while the scanner is still held in reset -- no scanout
    // traffic competes with the load, and the framebuffer is complete and
    // final before the first frame starts.
    bool loaded = load_fb_into_dram(s);
    check(loaded, std::string(s.name) + ": framebuffer painted into DDR");
    if (!loaded) return;

    dut->scan_hold = 0;
    tick_pclk_n(8);
    program_clut();

    // Bulk pressure starts only AFTER the paint: during the paint the CPU
    // write port owns the lane and a competing read storm would just make
    // the load slow, not the scanout deadline hard.
    dut->bulk_pressure = bulk_enable ? 1 : 0;

    // Confirm the DUT is actually being driven with the geometry this
    // scenario claims, and print the derived window the checker assumes.  A
    // green gate that never drove the geometry under test is the most common
    // way a bug survives here.
    printf("  driven: hres=%u vres=%u base=%u stride=%u bpp_shift=%u "
           "bytes_per_px=%u scale_n=%u -> active %dx%d at (%d,%d), "
           "row payload %d B, pitch %u B\n",
           (unsigned)dut->hres, (unsigned)dut->vres,
           (unsigned)dut->fb_base_px, (unsigned)dut->fb_stride_px,
           (unsigned)dut->bpp_shift, (unsigned)dut->bytes_per_px,
           (unsigned)dut->scale_n, cap_aw, cap_ah, cap_border_x, cap_border_y,
           (s.bytes_per_px == 4) ? s.hres * 4 : ((s.hres >> s.bpp_shift)
                                                 + ((s.hres & ((1 << s.bpp_shift) - 1)) ? 1 : 0)),
           s.stride);

    // ── Collect frames ──
    frames.clear();
    cur = Frame();
    cur.px.assign((size_t)cap_aw * cap_ah, 0);
    cur.aw = cap_aw; cur.ah = cap_ah;
    px_idx = 0;

    int seen_sof = 0;
    long long guard = (long long)(s.frames + 3) * H_TOTAL * V_TOTAL + 4000000;
    while ((int)frames.size() < s.frames && guard-- > 0) {
        if (dut->hcount == 0 && dut->vcount == 0) {
            if (seen_sof > 0) { cur.complete = true; frames.push_back(cur); }
            seen_sof++;
            cur = Frame();
            cur.px.assign((size_t)cap_aw * cap_ah, 0);
            cur.aw = cap_aw; cur.ah = cap_ah;
        }
        tick_pclk_n(1);
        capture_tick();
    }
    dut->bulk_pressure = 0;
    printf("  fetch port: req_accepted=%u rsp_consumed=%u  ddr_reader q_peak=%u/%d\n",
           (unsigned)dut->dbg_req_accepted, (unsigned)dut->dbg_rsp_consumed,
           (unsigned)dut->dbg_q_peak, 1 << 6);
    {
        const double bursts = (double)dut->dbg_scan_bursts;
        const double exc    = (double)dut->dbg_scan_excursions;
        printf("  DDR excursions: scan bursts=%u in %u excursions (%.2f bursts/excursion, "
               "peak %u in flight); drain starved %u core cycles\n",
               (unsigned)dut->dbg_scan_bursts, (unsigned)dut->dbg_scan_excursions,
               exc > 0 ? bursts / exc : 0.0,
               (unsigned)dut->dbg_scan_outst_max, (unsigned)dut->dbg_scan_stall);
        if (dut->dbg_bulk_cycles) {
            const double cyc = (double)dut->dbg_bulk_cycles;
            const double ops = (double)dut->dbg_bulk_bursts;
            printf("  bulk (CPU miss storm): %u bursts in %u core cycles = %.2f cyc/op\n",
                   (unsigned)dut->dbg_bulk_bursts, (unsigned)dut->dbg_bulk_cycles,
                   ops > 0 ? cyc / ops : 0.0);
        }
    }
    check(guard > 0, std::string(s.name) + ": collected " +
          std::to_string(s.frames) + " frames");
    if (guard <= 0) return;

    // ── (A) absolute per-pixel correctness, per frame ──
    for (size_t f = (size_t)s.first_checked; f < frames.size(); f++) {
        const Frame& fr = frames[f];
        long bad = 0, blank = 0;
        int first_bad_line = -1, first_bad_col = -1;
        for (int r = 0; r < fr.ah; r++) {
            int y = ymap[(size_t)r];
            for (int c = 0; c < fr.aw; c++) {
                int x = xmap[(size_t)c];
                uint32_t want = expect_px(s, x, y, 0, 0);
                uint32_t got  = fr.px[(size_t)r * fr.aw + c];
                if (got != want) {
                    bad++;
                    if (got == 0) blank++;
                    if (first_bad_line < 0) { first_bad_line = r; first_bad_col = c; }
                }
            }
        }
        long total = (long)fr.ah * fr.aw;
        printf("  frame %zu: lit=%ld/%ld  mismatched=%ld (%.2f%%)  blank_of_those=%ld",
               f, fr.lit, total, bad, 100.0 * bad / total, blank);
        if (bad) printf("  first_bad=(line %d,col %d) want %06x got %06x",
                        first_bad_line, first_bad_col,
                        expect_px(s, xmap[(size_t)first_bad_col],
                                  ymap[(size_t)first_bad_line], 0, 0),
                        fr.px[(size_t)first_bad_line * fr.aw + first_bad_col]);
        printf("\n");

        // ── (C) slip identification ──
        if (bad) {
            // Vertical candidates: linebuf_scanout's ring depth.
            for (int dy = -2 * LINE_COUNT; dy <= 2 * LINE_COUNT; dy++) {
                if (dy == 0) continue;
                long m = 0;
                for (int r = 0; r < fr.ah && m < 16; r++)
                    for (int c = 0; c < fr.aw && m < 16; c++)
                        if (fr.px[(size_t)r*fr.aw+c] !=
                            expect_px(s, xmap[(size_t)c], ymap[(size_t)r], 0, dy)) m++;
                if (m < 16)
                    printf("        SLIP: frame matches source displaced by dy=%+d SOURCE ROWS "
                           "(= %+.2f ring depths)\n", dy, (double)dy / LINE_COUNT);
            }
            // PER-LINE horizontal displacement histogram.  A queue lap
            // desyncs the response stream progressively, so the frame is not
            // one rigid offset -- it is a set of them, and the SET is the
            // diagnostic: each distinct value is a multiple of however many
            // requests were lost.
            {
                std::map<int, int> hist;
                int last_y = -1;
                for (int r = 0; r < fr.ah; r++) {
                    if (ymap[(size_t)r] == last_y) continue;   // one probe per SOURCE row
                    last_y = ymap[(size_t)r];
                    int found = 0x7FFFFFFF;
                    for (int dx = -1024; dx <= 1024 && found == 0x7FFFFFFF; dx++) {
                        bool ok = true;
                        for (int c = 0; c < fr.aw && ok; c++)
                            if (fr.px[(size_t)r*fr.aw+c] !=
                                expect_px(s, xmap[(size_t)c], ymap[(size_t)r], dx, 0)) ok = false;
                        if (ok) found = dx;
                    }
                    hist[found]++;
                }
                printf("        per-source-row horizontal displacement (dx SOURCE BYTES):\n");
                for (auto& kv : hist) {
                    if (kv.first == 0x7FFFFFFF)
                        printf("           (no single rigid dx explains the row): %d rows\n", kv.second);
                    else
                        printf("           dx=%+5d bytes (%+.2f x QDEPTH 64, %+.2f ring lines): %d rows\n",
                               kv.first, kv.first / 64.0, (double)kv.first / LINE_BYTES, kv.second);
                }
            }
            // Horizontal candidates: any byte offset up to two ring lines,
            // reported with what structural quantity it equals.
            for (int dx = -4 * LINE_BYTES; dx <= 4 * LINE_BYTES; dx++) {
                if (dx == 0) continue;
                long m = 0;
                for (int r = 0; r < fr.ah && m < 16; r++)
                    for (int c = 0; c < fr.aw && m < 16; c++)
                        if (fr.px[(size_t)r*fr.aw+c] !=
                            expect_px(s, xmap[(size_t)c], ymap[(size_t)r], dx, 0)) m++;
                if (m < 16)
                    printf("        SLIP: frame matches source displaced by dx=%+d SOURCE BYTES "
                           "(= %+.2f ring lines of %d B; %+.2f x QDEPTH 64)\n",
                           dx, (double)dx / LINE_BYTES, LINE_BYTES, (double)dx / 64.0);
            }
        }
        check(bad == 0, std::string(s.name) + " frame " + std::to_string(f) +
              ": every active pixel matches its source pixel");
    }

    // ── (A2) THE PAINTED WINDOW, measured off the raster ──────────────
    // Everything above compares pixels INSIDE the window the checker
    // assumed.  That is blind to the scale itself: a frame rendered at the
    // wrong N is internally self-consistent over the region both agree on,
    // which is precisely how a fractional rung survived a pixel-exact gate.
    // So measure what the DUT actually painted -- the bounding box of every
    // non-black pixel on the full 1920x1080 raster -- and require it to be
    // the centred hres*N x vres*N window.
    //
    // Indexed depths only: clut_entry() is never black, so every source
    // pixel lights.  At 24bpp a source pixel CAN be (0,0,0), which would
    // shrink the box for reasons that are not a placement error.
    if (s.bytes_per_px != 4) {
        for (size_t f = (size_t)s.first_checked; f < frames.size(); f++) {
            const Frame& fr = frames[f];
            const int wx0 = cap_border_x, wx1 = cap_border_x + cap_aw - 1;
            const int wy0 = cap_border_y, wy1 = cap_border_y + cap_ah - 1;
            const bool ok = (fr.bx0 == wx0) && (fr.bx1 == wx1)
                         && (fr.by0 == wy0) && (fr.by1 == wy1);
            if (!ok)
                printf("        painted box x[%d..%d] y[%d..%d] (%dx%d), "
                       "expected x[%d..%d] y[%d..%d] (%dx%d) for %dx%d at N=%d\n",
                       fr.bx0, fr.bx1, fr.by0, fr.by1,
                       fr.bx1 - fr.bx0 + 1, fr.by1 - fr.by0 + 1,
                       wx0, wx1, wy0, wy1, cap_aw, cap_ah, s.hres, s.vres, n);
            check(ok, std::string(s.name) + " frame " + std::to_string(f) +
                  ": painted window is exactly " + std::to_string(cap_aw) + "x" +
                  std::to_string(cap_ah) + " centred (integer scale x" +
                  std::to_string(n) + ")");
        }
    }

    // ── (B) frame-to-frame identity ──
    // The framebuffer is static, so consecutive frames must be identical.
    int ident_fail = 0;
    for (size_t f = (size_t)s.first_checked + 1; f < frames.size(); f++) {
        long diff = 0;
        int first_line = -1;
        int last_line = -1;
        for (int r = 0; r < frames[f].ah; r++) {
            bool line_diff = false;
            for (int c = 0; c < frames[f].aw; c++) {
                if (frames[f].px[(size_t)r*frames[f].aw+c] !=
                    frames[f-1].px[(size_t)r*frames[f].aw+c]) { diff++; line_diff = true; }
            }
            if (line_diff) { if (first_line < 0) first_line = r; last_line = r; }
        }
        long total = (long)frames[f].ah * frames[f].aw;
        if (diff) {
            printf("        frame %zu vs %zu: %ld px differ (%.2f%%), display lines %d..%d\n",
                   f - 1, f, diff, 100.0 * diff / total, first_line, last_line);
            ident_fail++;
        }
        check(diff == 0, std::string(s.name) + ": frame " + std::to_string(f-1) +
              " == frame " + std::to_string(f) + " (static framebuffer)");
    }
    (void)ident_fail;

    check(dut->line_underflow_sticky == 0,
          std::string(s.name) + ": no line-buffer underflow");
    check(dut->fb_reader_underflow_sticky == 0,
          std::string(s.name) + ": no fb_reader response-FIFO overflow");
}

// ────────────────────────────────────────────────────────────────────────
// WORST-CASE scenario: a MODE TRANSITION under maximum bulk pressure.
//
// Everything above holds one geometry for the whole run, and the DUT is
// fully reset between scenarios.  Neither is the condition underflow
// actually happens in.  The one that is:
//
//   * the tightest mode we support (1024x768x24bpp -- 189 MB/s of ring
//     fill, the only mode whose demand is a large fraction of anything),
//   * a saturating CPU miss storm on the SAME DDR read path, so
//     axi_vram_priority_mux3's MAX_BULK_AHEAD is actually reached and
//     scanout's reads sit behind the maximum permitted convoy,
//   * a LIVE placement change -- no reset -- which forces
//     scanout_fetch.v's hard resync, empties the 64-row pclk ring, and
//     makes the DDR ring refill a whole frame's worth of lines from cold
//     WHILE the bulk storm continues.  That last part is the interesting
//     one: the ring's buffering, which is what hides DDR latency in
//     steady state, is exactly what a mode change throws away.
//
// The two geometries are placed in DISJOINT regions of the aperture so a
// single paint serves both and the checker can prove the OTHER mode's
// pixels were not being served out of a stale ring line.
// ────────────────────────────────────────────────────────────────────────
static void apply_placement(const Scenario& s) {
    dut->fb_base_px = s.base; dut->fb_stride_px = s.stride;
    dut->bpp_shift = s.bpp_shift; dut->bytes_per_px = s.bytes_per_px;
    dut->hres = s.hres; dut->vres = s.vres;
    dut->scale_n = policy_scale_n(s.hres, s.vres);
}

static void collect_into(const Scenario& s, int nframes) {
    const int n = policy_scale_n(s.hres, s.vres);
    cap_aw = active_span(s.hres, n);
    cap_ah = active_span(s.vres, n);
    xmap = bresenham_map(cap_aw, scale_num_of(n), scale_den_of(n));
    ymap = bresenham_map(cap_ah, scale_num_of(n), scale_den_of(n));
    cap_border_x = (DST_W - cap_aw) / 2;
    cap_border_y = (DST_H - cap_ah) / 2;
    frames.clear();
    cur = Frame();
    cur.px.assign((size_t)cap_aw * cap_ah, 0);
    cur.aw = cap_aw; cur.ah = cap_ah;
    px_idx = 0;
    int seen_sof = 0;
    long long guard = (long long)(nframes + 3) * H_TOTAL * V_TOTAL + 4000000;
    while ((int)frames.size() < nframes && guard-- > 0) {
        if (dut->hcount == 0 && dut->vcount == 0) {
            if (seen_sof > 0) { cur.complete = true; frames.push_back(cur); }
            seen_sof++;
            cur = Frame();
            cur.px.assign((size_t)cap_aw * cap_ah, 0);
            cur.aw = cap_aw; cur.ah = cap_ah;
        }
        tick_pclk_n(1);
        capture_tick();
    }
}

static long frame_bad_count(const Scenario& s, const Frame& fr) {
    long bad = 0;
    for (int r = 0; r < fr.ah; r++) {
        int y = ymap[(size_t)r];
        for (int c = 0; c < fr.aw; c++) {
            if (fr.px[(size_t)r * fr.aw + c] !=
                expect_px(s, xmap[(size_t)c], y, 0, 0)) bad++;
        }
    }
    return bad;
}

static void report_arbitration(const char* tag) {
    const double bursts = (double)dut->dbg_scan_bursts;
    const double exc    = (double)dut->dbg_scan_excursions;
    printf("    [%s] scan bursts=%u in %u excursions (%.2f/excursion, peak %u in flight), "
           "drain starved %u cyc",
           tag, (unsigned)dut->dbg_scan_bursts, (unsigned)dut->dbg_scan_excursions,
           exc > 0 ? bursts / exc : 0.0, (unsigned)dut->dbg_scan_outst_max,
           (unsigned)dut->dbg_scan_stall);
    if (dut->dbg_bulk_bursts)
        printf("; bulk %u bursts / %u cyc = %.2f cyc/op",
               (unsigned)dut->dbg_bulk_bursts, (unsigned)dut->dbg_bulk_cycles,
               (double)dut->dbg_bulk_cycles / (double)dut->dbg_bulk_bursts);
    printf("\n");
}

static void run_mode_transition(int nframes) {
    // A: the DDR-bandwidth worst case, low 3 MB of the aperture.
    // B: an indexed mode in the region A does not touch (base 0x310000).
    const Scenario A = {"xfer-A-24bpp-1024x768", 1024, 768, 0, 4, 0,        4096, 0, 0};
    const Scenario B = {"xfer-B-8bpp-832x624",    832, 624, 0, 1, 0x310000, 1024, 0, 0};

    printf("\n== scenario mode-transition (worst case: tightest mode + live "
           "placement change + %s bulk storm) ==\n",
           bulk_enable_transition ? "SATURATING" : "no");

    paint(A, /*clear=*/true);
    paint(B, /*clear=*/false);

    dut->rst = 1; dut->mig_rst = 1; dut->scan_hold = 1;
    dut->bulk_pressure = 0;
    apply_placement(A);
    dut->clut_we = 0;
    dut->cpu_awvalid = 0; dut->cpu_wvalid = 0; dut->cpu_bready = 1;
    dut->cpu_awid = 0; dut->cpu_awaddr = 0; dut->cpu_awlen = 0;
    dut->cpu_awsize = 4; dut->cpu_awburst = 1; dut->cpu_wstrb = 0; dut->cpu_wlast = 0;
    tick_pclk_n(20);
    dut->mig_rst = 0;
    for (int g = 0; g < 20000 && !dut->cal_done; g++) tick_pclk_n(1);
    check(dut->cal_done != 0, "mode-transition: MIG model calibrated");
    if (!dut->cal_done) return;
    dut->rst = 0;
    tick_pclk_n(8);

    bool loaded = load_fb_into_dram(A) && load_fb_into_dram(B);
    check(loaded, "mode-transition: both framebuffers painted into DDR");
    if (!loaded) return;

    dut->scan_hold = 0;
    tick_pclk_n(8);
    program_clut();
    dut->bulk_pressure = bulk_enable_transition ? 1 : 0;

    struct Leg { const Scenario* s; const char* tag; };
    const Leg legs[] = { {&A, "A(24bpp 1024x768)"},
                         {&B, "B(8bpp 832x624 x1)"},
                         {&A, "A again"} };

    for (const Leg& leg : legs) {
        apply_placement(*leg.s);
        // +2 warm-up frames: the placement change is committed at a frame
        // boundary and the first frame after it is the cold-ring one.
        collect_into(*leg.s, nframes + 2);
        report_arbitration(leg.tag);
        if ((int)frames.size() < nframes + 2) {
            check(false, std::string("mode-transition ") + leg.tag + ": collected frames");
            continue;
        }
        long bad = frame_bad_count(*leg.s, frames.back());
        long total = (long)frames.back().ah * frames.back().aw;
        if (bad) printf("    %s: %ld/%ld px mismatched (%.2f%%)\n",
                        leg.tag, bad, total, 100.0 * bad / total);
        check(bad == 0, std::string("mode-transition ") + leg.tag +
              ": every active pixel matches after the live placement change");
        check(dut->line_underflow_sticky == 0,
              std::string("mode-transition ") + leg.tag + ": no line-buffer underflow");
        check(dut->fb_reader_underflow_sticky == 0,
              std::string("mode-transition ") + leg.tag +
              ": no fb_reader response-FIFO overflow");
    }
    dut->bulk_pressure = 0;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_scanout_ddr_frames;
    // One settle before anything reads a DUT output.  dbg_src_w/dbg_src_h
    // are constant assigns off the elaborated parameters, but Verilator does
    // not evaluate them until the first eval() -- and reading them as 0
    // would make run_scenario SKIP every scenario and still exit 0.  That
    // failure mode was observed while writing this: a fully green run that
    // tested nothing.
    dut->eval();
    fbmem.assign(FB_BYTES, 0);
    dut->bulk_pressure = 0;
    if (const char* e = getenv("BULK")) {
        bulk_enable = (atoi(e) != 0);
        bulk_enable_transition = bulk_enable;
    }

    int nframes = 4;
    if (const char* e = getenv("SCAN_FRAMES")) nframes = atoi(e);
    const char* only = getenv("SCAN_ONLY");

    printf("tb_scanout_ddr_frames: full-chain multi-frame scanout gate "
           "(vtg + linebuf_scanout + fb_reader + scanout_ddr_reader + DDR)\n");
    printf("  bulk pressure (CPU miss storm on the shared DDR read path): "
           "geometry sweep %s, mode-transition %s\n",
           bulk_enable ? "ON" : "off (set BULK=1)",
           bulk_enable_transition ? "ON" : "off");

    Scenario scen[] = {
        // name            hres vres sh bpp base   stride  frames chk [alias]
        // The output scale is NOT in this table -- policy_scale_n() derives
        // it from (hres,vres) and drives the DUT with it.  N per row is in
        // the comments only, as documentation.
        {"8bpp-640x480",   640, 480, 0, 1, 0x1000, 1024, nframes, 0},   // N=2
        {"1bpp-640x480",   640, 480, 3, 1, 0x1000, 1024, nframes, 0},   // N=2
        // ── The Q700 "832x624 16in RGB" (monitor sense 0x6D) register set ──
        // Taken bit-for-bit off MAME's macqd700 at that sense code (and read
        // back identically from our own DAFB on hardware): base 3584, stride
        // 832 BYTES, 1bpp.  Two things make it structurally unlike every
        // scenario above it, and both are the point of having it here:
        //   * the row PITCH (832 B) is only 8x the row PAYLOAD (104 B), and
        //     is NOT a multiple of the 128 B scanout_line_fetch ring line, so
        //     alternate source rows start mid-ring-line;
        //   * hres (832) is not the elaborated SRC_W (1024), so any term that
        //     still measures a row against the build-time width is wrong here
        //     and right everywhere else.
        //
        // N = min(1920/832, 1080/624) = min(2,1) = 1, so this mode renders
        // 832x624 letterboxed and SHARP.  It used to render at the 3:2 rung
        // (1248x936), whose 2,1,2,1 replication aliases its 1bpp 50%-dither
        // desktop into fine striping -- docs/video_path_review.md S3.  The
        // two 832x624 scenarios that stated that rung explicitly are folded
        // into these, because the scale is now derived and there is exactly
        // one answer per geometry.
        {"1bpp-832x624",   832, 624, 3, 1,   3584,  832, nframes, 0,
                                                       "1bpp-832x624-3_2"},
        {"8bpp-832x624",   832, 624, 0, 1,   3584, 1024, nframes, 0,
                                                       "8bpp-832x624-3_2"},
        // "Millions" (24bpp) at 832x624 -- the combination that glitches on
        // hardware.  Stride is the literal byte row-pitch 832*4 = 3328
        // (4-byte aligned, so the WIDE one-request-per-pixel path in
        // scanout_fetch.v is engaged), and the 832*624*4 = 2,076,672 B
        // footprint fits the 4 MB aperture.  The alias keeps the line-start
        // NEGATIVE CONTROL target (Makefile: SCAN_ONLY=24bpp-832x624-3_2)
        // selecting this same mode without a Makefile edit.
        {"24bpp-832x624",  832, 624, 0, 4,  3584, 3328, nframes, 0,
                                                       "24bpp-832x624-3_2"},
        // Same mode with the ROUNDED-UP row pitch.  The DAFB stride register
        // is a literal byte row-pitch and Mac OS does not always program it
        // tight: 640x480x24bpp programs 4096 for a 2560 B payload, and
        // 832x624x8bpp programs 1024 for an 832 B payload.  3328 is only the
        // tight value; 4096 is the plausible rounded one.  It also pushes
        // `fetch_stride_sane` (stride > row_last_off = 3327) well clear of
        // the 1-byte margin the tight stride sits on.
        {"24bpp-832x624-pitch4096",
                           832, 624, 0, 4,  3584, 4096, nframes, 0,
                                                "24bpp-832x624-3_2-pitch4096"},
        {"24bpp-640x480",  640, 480, 0, 4, 0x1000, 4096, nframes, 0},   // N=2
        // ── The Q700 "Mac RGB 12in" 512x384 mode (monitor sense 0x02) ─────
        // USER-REPORTED: works at 8bpp, fails above it.  This table had no
        // 512-wide scenario at all, so the SMALL end of the geometry range
        // had never been through the real DDR chain -- every scenario above
        // is 640 or wider.  That matters independently of the report: a
        // width-derived term that truncates or underflows shows up at the
        // small end, not the large one, and the mode's fixed DAFB base is
        // 0x1000 with a 1024 B pitch that is 2x its own 8bpp payload.
        //
        // Depths 1 / 8 / 24 with the pitches Mac OS plausibly programs:
        // 1024 for the indexed depths (the ROM's own value for this mode --
        // 16x the 1bpp payload and 2x the 8bpp one), and for direct colour
        // BOTH the tight 512*4 = 2048 and the padded 4096.  2048 is the
        // tightest `fetch_stride_sane` margin anywhere in this table: that
        // gate is `stride > row_last_off` and row_last_off is 2047, so the
        // mode passes by exactly one byte.
        {"1bpp-512x384",      512, 384, 3, 1, 0x1000, 1024, nframes, 0,
                                                       "1bpp-512x384-2x"},
        {"8bpp-512x384",      512, 384, 0, 1, 0x1000, 1024, nframes, 0,
                                                       "8bpp-512x384-2x"},
        {"24bpp-512x384",     512, 384, 0, 4, 0x1000, 2048, nframes, 0,
                                                       "24bpp-512x384-2x"},
        {"24bpp-512x384-pitch4096",
                              512, 384, 0, 4, 0x1000, 4096, nframes, 0,
                                                "24bpp-512x384-2x-pitch4096"},
        // ── The Q700 "Apple 21in Color" 1152x870 mode (sense 0x3B) ────────
        // The last row of the Q700 mode set, and the one this table has
        // never carried.  It does NOT fit the default 1024x768 elaboration
        // (it would be silently cropped), so run_scenario SKIPS it loudly
        // there and RUNS it under `make tb-scanout-first-pixel`, which
        // elaborates the same source at the shipping SRC 1152x1024 bound.
        // N = min(1920/1152, 1080/870) = 1: sharp, letterboxed 768x210.
        {"8bpp-1152x870",    1152, 870, 0, 1, 0x1000, 1152, nframes, 0},
        {"1bpp-1152x870",    1152, 870, 3, 1, 0x1000, 1152, nframes, 0},
        {"8bpp-1024x768",    1024, 768, 0, 1, 0x1000, 1024, nframes, 0},   // N=1
        // The DDR-BANDWIDTH worst case, and the reason the wide 4-byte fetch
        // group and the 128 B ring line exist at all: 1024x768x24bpp needs
        // 142% of a one-byte-per-request port and 189 MB/s of ring fill.
        // Any change that bounds outstanding requests has to be shown NOT to
        // starve this one, which is what this scenario is for -- it fills the
        // 4 MB aperture exactly (base 0, stride 4096, 768 rows).
        {"24bpp-1024x768", 1024, 768, 0, 4, 0,      4096, nframes, 0},  // N=1
    };

    int selected = 0;
    for (const Scenario& s : scen) {
        if (only && strcmp(only, s.name) != 0 &&
            !(s.alias && strcmp(only, s.alias) == 0)) continue;
        selected++;
        run_scenario(s);
    }
    // A SCAN_ONLY that matches nothing used to run zero scenarios and exit
    // 0 -- which reads as a green gate, and silently disarms the line-start
    // negative control (whose entire signal is a NON-zero exit).  Fail loud.
    if (only && selected == 0 && strcmp(only, "mode-transition") != 0) {
        printf("\nSCAN_ONLY='%s' matched no scenario (nor any alias).\n", only);
        fail_count++;
    }
    if (!only || strcmp(only, "mode-transition") == 0) run_mode_transition(nframes);

    printf("\n------------------------------------------\n");
    printf("scenarios: %d ran, %d skipped (scanner bound %ux%u)\n",
           ran_count, skipped_count,
           (unsigned)dut->dbg_src_w, (unsigned)dut->dbg_src_h);
    if (ran_count == 0 && !(only && strcmp(only, "mode-transition") == 0)) {
        printf("NO SCENARIO RAN -- a green exit here would be meaningless.\n");
        fail_count++;
    }
    printf("tb_scanout_ddr_frames: %d PASS / %d FAIL\n", pass_count, fail_count);
    dut->final();
    delete dut;
    return fail_count ? 1 : 0;
}
