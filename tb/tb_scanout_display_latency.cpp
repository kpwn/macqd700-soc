// tb_scanout_display_latency.cpp -- THE FIVE-STAGE LOCKSTEP GATE.
// ---------------------------------------------------------------------------
// scanout_display.v's ladder is five registered stages from a combinational
// `x_src` to `rgb`, and six parallel pipes must match it exactly:
//
//     de, hs, vs, border, pix_valid, splash
//
// docs/video_path_review.md S6 lists that alignment under "what is NOT wrong",
// with "Hand-verified" as its entire evidence.  Since the 2026-08-20
// decomposition it spans three files, so a hand verification no longer even
// fits on one screen.
//
// WHAT THIS FILE DOES DIFFERENTLY FROM THE OTHER SCANOUT TBS.
// tb_scanout_ddr_frames compares every active pixel, so it DOES notice a
// slipped pipe -- but it reports "first_bad = (line, col)", which is the same
// message a fetch underrun, a wrong palette entry or a bad address gives.
// This gate instead MEASURES each pipe's depth independently, by sweeping the
// candidate latency L over 0..9 and finding which L makes the output stream a
// pure function of the stage-0 stream.  The failure message is then the name
// of the pipe and the depth it actually has:
//
//     [FAIL] border pipe depth: measured 4, expected 5
//
// VACUITY GUARD.  For each pipe the sweep must find EXACTLY ONE L that
// matches.  If two candidate depths both match, the stimulus does not toggle
// often enough to pin the depth and the check is vacuous -- that is reported
// as a failure too, not silently accepted.  (This matters: a de/hs/vs pipe
// probed with a signal that is constant for the whole run would "pass" at
// every L.)
// ---------------------------------------------------------------------------

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <verilated.h>
#include "Vtb_scanout_display_latency.h"

// Must match the harness's parameter defaults.
static const int SRC_W    = 64;
static const int SRC_H    = 64;
static const int DST_W    = 256;
static const int DST_H    = 160;
static const int H_TOTAL  = 320;
static const int V_TOTAL  = 200;
static const int LINE_CNT = 4;          // 1 << LINE_COUNT_LOG2
static const int SPLASH_SCALE = 4;
static const int SPLASH_PIX   = 32 * SPLASH_SCALE;
static const int SPLASH_SHIFT = 2;      // log2(SPLASH_SCALE)

static const int EXPECTED_LATENCY = 5;
static const int MAX_PROBE_LAT    = 9;

static const uint32_t SPLASH_FG = 0xFFFFFFu;
static const uint32_t SPLASH_BG = 0x000000u;

static Vtb_scanout_display_latency* dut = nullptr;
static int n_pass = 0;
static int n_fail = 0;

#define CHECK(cond, fmt, ...) do {                       \
    if (cond) { n_pass++; std::printf("  [PASS] " fmt "\n", ##__VA_ARGS__); } \
    else      { n_fail++; std::printf("  [FAIL] " fmt "\n", ##__VA_ARGS__); } \
} while (0)

// ---------------------------------------------------------------------------
// The splash bitmap, byte-for-byte the table in compositor.v.  Duplicated on
// purpose: a check that reads its expectation out of the DUT proves nothing.
// ---------------------------------------------------------------------------
static const uint32_t kSplash[32] = {
    0x00000000u, 0x00000a00u, 0x00000400u, 0x00005540u,
    0x00007fc0u, 0x00003f80u, 0x00003f80u, 0x00001f00u,
    0x00001f00u, 0x00001f00u, 0x00003f80u, 0x0000ffe0u,
    0x00003f80u, 0x00003f80u, 0x00003f83u, 0x00103f9fu,
    0x18103ffbu, 0x0e3fffd5u, 0x1beabfabu, 0x480d7fd5u,
    0xf80abfabu, 0x480d7fd5u, 0x1beabfabu, 0x0e3fffd5u,
    0x18107ffbu, 0x00107fdfu, 0x00007fc3u, 0x0000ffe0u,
    0x0000ffe0u, 0x0000ffe0u, 0x0001fff0u, 0x0001fff0u,
};

// ---------------------------------------------------------------------------
// place_plan.v, re-implemented.  Same reason as the splash table.
// ---------------------------------------------------------------------------
struct Placement { int n, active_w, active_h, border_x, border_y; };

static Placement plan(int hres, int vres, int scale_n_in) {
    int n = scale_n_in;
    if (n == 0) {
        if (hres == 0 || vres == 0)                                n = 1;
        else if (hres * 4 <= DST_W && vres * 4 <= DST_H)           n = 4;
        else if (hres * 3 <= DST_W && vres * 3 <= DST_H)           n = 3;
        else if (hres * 2 <= DST_W && vres * 2 <= DST_H)           n = 2;
        else                                                       n = 1;
    } else if (n > 4) {
        n = 4;
    }
    Placement p;
    p.n = n;
    p.active_w = hres * n; if (p.active_w > 0x1FFF) p.active_w = 0x1FFF;
    p.active_h = vres * n; if (p.active_h > 0x1FFF) p.active_h = 0x1FFF;
    p.border_x = (p.active_w >= DST_W) ? 0 : ((DST_W - p.active_w) >> 1);
    p.border_y = (p.active_h >= DST_H) ? 0 : ((DST_H - p.active_h) >> 1);
    return p;
}

// ---------------------------------------------------------------------------
// pixel_unpack.v, re-implemented.
// ---------------------------------------------------------------------------
static uint8_t unpack_index(int bpp_shift, int x_lo, uint8_t b) {
    switch (bpp_shift) {
        case 3: return (b >> (7 - (x_lo & 7))) & 0x1;
        case 2: return (b >> (6 - 2 * (x_lo & 3))) & 0x3;
        case 1: return (x_lo & 1) ? (b & 0xF) : ((b >> 4) & 0xF);
        default: return b;
    }
}

// ---------------------------------------------------------------------------
// Stage-0 state, one entry per cycle.
// ---------------------------------------------------------------------------
struct Stage0 {
    bool de, hs, vs, border, pix_valid, splash;
    int  x_src, slot;
    int  h, v;
};
struct Out {
    uint32_t rgb;
    bool de, hs, vs;
};

// Shadow of the line store: [slot][byte] for each of the three planes.
static uint8_t shadow[LINE_CNT][SRC_W][3];
static uint32_t clut_shadow[256];

// ---------------------------------------------------------------------------
static void tick() {
    dut->pclk = 0; dut->eval();
    dut->pclk = 1; dut->eval();
}

static void write_line_store() {
    dut->clut_we = 0;
    for (int slot = 0; slot < LINE_CNT; slot++) {
        for (int b = 0; b < SRC_W; b++) {
            dut->wr_en   = 1;
            dut->wr_addr = slot * SRC_W + b;
            dut->wr_d0   = shadow[slot][b][0];
            dut->wr_d1   = shadow[slot][b][1];
            dut->wr_d2   = shadow[slot][b][2];
            tick();
        }
    }
    dut->wr_en = 0;
    tick();
}

static void write_clut() {
    dut->wr_en = 0;
    for (int i = 0; i < 256; i++) {
        dut->clut_we    = 1;
        dut->clut_waddr = i;
        dut->clut_wdata = clut_shadow[i];
        tick();
    }
    dut->clut_we = 0;
    tick();
}

static void reset_dut() {
    dut->resetn = 0;
    dut->wr_en = 0; dut->clut_we = 0;
    dut->disp_ready = 1;
    for (int i = 0; i < 8; i++) tick();
    dut->resetn = 1;
}

// Advance to the very start of a frame so every scenario begins at a known
// raster position.
static void align_to_frame_start() {
    for (int i = 0; i < H_TOTAL * V_TOTAL * 2; i++) {
        if (dut->hcount == 0 && dut->vcount == 0) return;
        tick();
    }
}

// ---------------------------------------------------------------------------
// Latency sweep.  `match(t, L)` asks whether the output at cycle t is what the
// stage-0 record at t-L predicts.  Returns the unique matching L, or -1 when
// none or more than one match.
// ---------------------------------------------------------------------------
template <typename F>
static int measure_latency(const std::vector<Stage0>& in,
                           const std::vector<Out>& out,
                           F match,
                           int* n_matching,
                           int* mismatch_at_expected) {
    const size_t n = out.size();
    int found = -1, count = 0;
    *mismatch_at_expected = -1;
    for (int L = 0; L <= MAX_PROBE_LAT; L++) {
        bool ok = true;
        for (size_t t = MAX_PROBE_LAT + 1; t < n; t++) {
            if (!match(in[t - L], out[t])) {
                if (L == EXPECTED_LATENCY && *mismatch_at_expected < 0)
                    *mismatch_at_expected = (int)t;
                ok = false;
                break;
            }
        }
        if (ok) { count++; if (found < 0) found = L; }
    }
    *n_matching = count;
    return (count == 1) ? found : -1;
}

static void report_latency(const char* pipe, int measured, int n_matching,
                           int mismatch_at_expected) {
    if (n_matching == 0) {
        CHECK(false, "%s pipe depth: NO latency in 0..%d explains the output"
                     " (first mismatch at depth %d was cycle %d)",
              pipe, MAX_PROBE_LAT, EXPECTED_LATENCY, mismatch_at_expected);
    } else if (n_matching > 1) {
        CHECK(false, "%s pipe depth: VACUOUS -- %d different depths all match,"
                     " the stimulus does not pin it", pipe, n_matching);
    } else {
        CHECK(measured == EXPECTED_LATENCY,
              "%s pipe depth: measured %d, expected %d",
              pipe, measured, EXPECTED_LATENCY);
    }
}

// ---------------------------------------------------------------------------
// One scenario: run `frames` frames, capturing stage-0 and output streams.
// ---------------------------------------------------------------------------
struct Scenario {
    const char* name;
    int  bpp_shift;
    int  hres, vres, scale_n;
    bool fetch_direct;
    bool dafb_live;
    int  starve_h;    // <0 = never; else drop disp_ready at this hcount
    int  starve_v;
};

static void run_scenario(const Scenario& sc,
                         std::vector<Stage0>& in,
                         std::vector<Out>& out) {
    Placement p = plan(sc.hres, sc.vres, sc.scale_n);

    dut->bpp_shift    = sc.bpp_shift;
    dut->hres         = sc.hres;
    dut->vres         = sc.vres;
    dut->scale_n      = sc.scale_n;
    dut->fetch_direct = sc.fetch_direct;
    dut->dafb_live    = sc.dafb_live;

    align_to_frame_start();

    // C++ mirror of upscale.v's Bresenham + the ring's one-cycle disp_y delay.
    int x_acc = 0, x_src = 0, y_acc = 0, y_src = 0, y_src_d1 = 0;

    const int splash_mid_x = p.border_x + (p.active_w >> 1);
    const int splash_mid_y = p.border_y + (p.active_h >> 1);
    const int splash_x0 = (splash_mid_x > SPLASH_PIX / 2) ? (splash_mid_x - SPLASH_PIX / 2) : 0;
    const int splash_y0 = (splash_mid_y > SPLASH_PIX / 2) ? (splash_mid_y - SPLASH_PIX / 2) : 0;

    const int frames = 2;
    const long total = (long)H_TOTAL * V_TOTAL * frames;

    in.clear(); out.clear();
    in.reserve(total); out.reserve(total);

    for (long i = 0; i < total; i++) {
        const int h = dut->hcount;
        const int v = dut->vcount;

        const bool de  = (h < DST_W) && (v < DST_H);
        const bool ax  = (h >= p.border_x) && (h < p.border_x + p.active_w);
        const bool ay  = (v >= p.border_y) && (v < p.border_y + p.active_h);
        const bool bord = !(ax && ay);
        const bool rd_req = de && !bord;

        bool ready = true;
        if (sc.starve_h >= 0 && h == sc.starve_h && v == sc.starve_v)
            ready = false;
        dut->disp_ready = ready ? 1 : 0;

        // Splash geometry, combinational off h/v -- compositor.v.
        const int dx = h - splash_x0;
        const int dy = v - splash_y0;
        bool splash = false;
        if (dx >= 0 && dx < SPLASH_PIX && dy >= 0 && dy < SPLASH_PIX) {
            const int col  = (dx >> SPLASH_SHIFT) & 31;
            const int rowi = (dy >> SPLASH_SHIFT) & 31;
            splash = ((kSplash[rowi] >> col) & 1u) != 0;
        }

        Stage0 s;
        s.de = de; s.hs = dut->hs_in; s.vs = dut->vs_in;
        s.border = bord; s.pix_valid = rd_req && ready; s.splash = splash;
        s.x_src = x_src;
        s.slot  = y_src_d1 % LINE_CNT;
        s.h = h; s.v = v;
        in.push_back(s);

        Out o;
        o.rgb = dut->rgb & 0xFFFFFFu;
        o.de = dut->de_out; o.hs = dut->hs_out; o.vs = dut->vs_out;
        out.push_back(o);

        // Advance the model exactly as upscale.v does, then the DUT.
        const bool sof = (h == 0) && (v == 0);
        const bool sol = (h == 0);
        const bool line_end = de && (h == DST_W - 1);
        const int  den = p.n;

        const int next_y_src_d1 = y_src;
        if (sof) { y_acc = 0; y_src = 0; }
        else if (line_end && ay) {
            if (y_acc + 1 >= den) { y_acc = y_acc + 1 - den; if (y_src != SRC_H - 1) y_src++; }
            else                  { y_acc = y_acc + 1; }
        }
        if (sol) { x_acc = 0; x_src = 0; }
        else if (ax && !bord) {
            if (x_acc + 1 >= den) { x_acc = x_acc + 1 - den; if (x_src != SRC_W - 1) x_src++; }
            else                  { x_acc = x_acc + 1; }
        }
        y_src_d1 = next_y_src_d1;

        tick();
    }
    dut->disp_ready = 1;
}

// Expected pixel colour from a stage-0 record, ignoring the splash arm.
static uint32_t expect_pixel(const Scenario& sc, const Stage0& s) {
    if (s.border || !s.pix_valid) return 0;
    const int byte_idx = s.x_src >> sc.bpp_shift;
    const uint8_t p0 = shadow[s.slot][byte_idx][0];
    const uint8_t p1 = shadow[s.slot][byte_idx][1];
    const uint8_t p2 = shadow[s.slot][byte_idx][2];
    if (sc.fetch_direct)
        return ((uint32_t)p0 << 16) | ((uint32_t)p1 << 8) | p2;
    return clut_shadow[unpack_index(sc.bpp_shift, s.x_src & 7, p0)];
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_scanout_display_latency;

    // Palette: entry i is 0x400000 | i, so EVERY entry is non-black (a border
    // pixel and a legitimate index-0 pixel are then distinguishable) and the
    // low byte reads back the index the pipeline actually used.
    for (int i = 0; i < 256; i++) clut_shadow[i] = 0x400000u | (uint32_t)i;

    // Line store: plane 0 of slot s, byte b holds b ^ (s * 0x5B) so the
    // content varies along BOTH axes -- a row-independent pattern would let a
    // y-side slip pass.  Planes 1/2 carry distinct values so the 24bpp direct
    // path is not a function of plane 0 alone.
    for (int s = 0; s < LINE_CNT; s++) {
        for (int b = 0; b < SRC_W; b++) {
            shadow[s][b][0] = (uint8_t)(b ^ (s * 0x5B));
            shadow[s][b][1] = (uint8_t)(b * 3 + s * 17 + 1);
            shadow[s][b][2] = (uint8_t)(b * 7 + s * 29 + 2);
        }
    }

    reset_dut();
    write_line_store();
    write_clut();

    std::vector<Stage0> in;
    std::vector<Out> out;

    // ── Scenario 1: 8bpp, N=1 ────────────────────────────────────────
    // N=1 is the depth-discriminating case: x_src advances EVERY destination
    // pixel, so a one-stage data/control skew changes the pixel VALUE, not
    // just its position.  At N>=2 a source pixel is on screen for N columns
    // and a one-column slip inside a run is invisible.
    {
        Scenario sc = {"8bpp N=1", 0, SRC_W, SRC_H, 1, false, true, -1, -1};
        std::printf("\n-- %s --\n", sc.name);
        run_scenario(sc, in, out);

        int nm, mism;
        int L;
        L = measure_latency(in, out, [](const Stage0& s, const Out& o) {
                return o.de == s.de; }, &nm, &mism);
        report_latency("de", L, nm, mism);

        L = measure_latency(in, out, [](const Stage0& s, const Out& o) {
                return o.hs == s.hs; }, &nm, &mism);
        report_latency("hs", L, nm, mism);

        L = measure_latency(in, out, [](const Stage0& s, const Out& o) {
                return o.vs == s.vs; }, &nm, &mism);
        report_latency("vs", L, nm, mism);

        // Border: with disp_ready pinned high, "the pixel is lit" is exactly
        // "stage 0 said we were inside the active window".
        L = measure_latency(in, out, [](const Stage0& s, const Out& o) {
                return (o.rgb != 0) == (!s.border && s.pix_valid); }, &nm, &mism);
        report_latency("border", L, nm, mism);

        // Data: the palette's low byte is the index the pipeline used, so this
        // pins the DATA path independently of where the border landed.
        L = measure_latency(in, out, [&sc](const Stage0& s, const Out& o) {
                return o.rgb == expect_pixel(sc, s); }, &nm, &mism);
        report_latency("data", L, nm, mism);

        // Whole-stream pixel equality at the measured depth -- the same
        // property tb_scanout_ddr_frames asserts, restated here so a
        // regression is caught by this fast gate too.
        long bad = 0;
        for (size_t t = MAX_PROBE_LAT + 1; t < out.size(); t++)
            if (out[t].rgb != expect_pixel(sc, in[t - EXPECTED_LATENCY])) bad++;
        CHECK(bad == 0, "8bpp N=1: every output pixel matches the model (%ld bad)", bad);
    }

    // ── Scenario 2: 1bpp, N=2 ────────────────────────────────────────
    // Exercises pixel_unpack's MSB-first sub-byte walk and the x_src_lo pipe,
    // which is one stage SHALLOWER than the compositor's control pipes and so
    // is the easiest of the lot to get wrong.
    {
        Scenario sc = {"1bpp N=2", 3, SRC_W, SRC_H, 2, false, true, -1, -1};
        std::printf("\n-- %s --\n", sc.name);
        run_scenario(sc, in, out);

        int nm, mism;
        int L = measure_latency(in, out, [&sc](const Stage0& s, const Out& o) {
                return o.rgb == expect_pixel(sc, s); }, &nm, &mism);
        report_latency("data(1bpp)", L, nm, mism);

        long bad = 0;
        for (size_t t = MAX_PROBE_LAT + 1; t < out.size(); t++)
            if (out[t].rgb != expect_pixel(sc, in[t - EXPECTED_LATENCY])) bad++;
        CHECK(bad == 0, "1bpp N=2: every output pixel matches the model (%ld bad)", bad);
    }

    // ── Scenario 3: 4bpp, N=1 ────────────────────────────────────────
    {
        Scenario sc = {"4bpp N=1", 1, SRC_W, SRC_H, 1, false, true, -1, -1};
        std::printf("\n-- %s --\n", sc.name);
        run_scenario(sc, in, out);
        long bad = 0;
        for (size_t t = MAX_PROBE_LAT + 1; t < out.size(); t++)
            if (out[t].rgb != expect_pixel(sc, in[t - EXPECTED_LATENCY])) bad++;
        CHECK(bad == 0, "4bpp N=1: every output pixel matches the model (%ld bad)", bad);
    }

    // ── Scenario 4: 2bpp, N=2 ────────────────────────────────────────
    {
        Scenario sc = {"2bpp N=2", 2, SRC_W, SRC_H, 2, false, true, -1, -1};
        std::printf("\n-- %s --\n", sc.name);
        run_scenario(sc, in, out);
        long bad = 0;
        for (size_t t = MAX_PROBE_LAT + 1; t < out.size(); t++)
            if (out[t].rgb != expect_pixel(sc, in[t - EXPECTED_LATENCY])) bad++;
        CHECK(bad == 0, "2bpp N=2: every output pixel matches the model (%ld bad)", bad);
    }

    // -- Scenarios 4b/4c: N=3 and N=4 ---------------------------------
    // The accumulator in upscale.v is sized from place_plan.v's MAX_SCALE_N=4
    // clamp (ACCW=4, holding 0..N-1 with a bit of headroom), replacing the 16
    // bits the deleted fractional ladder needed.  These exercise that bound at
    // and near its maximum, so a mis-sized accumulator shows up as replication
    // of the wrong width rather than as a silent wrap.
    //
    // Geometry is 56x32 rather than 64x64 so BOTH axes keep a real border at
    // N=4 -- see the full-width corner scenario below for why that matters.
    for (int n = 3; n <= 4; n++) {
        char label[32];
        std::snprintf(label, sizeof(label), "8bpp N=%d", n);
        Scenario sc = {label, 0, 56, 32, n, false, true, -1, -1};
        std::printf("\n-- %s --\n", sc.name);
        run_scenario(sc, in, out);
        long bad = 0;
        for (size_t t = MAX_PROBE_LAT + 1; t < out.size(); t++)
            if (out[t].rgb != expect_pixel(sc, in[t - EXPECTED_LATENCY])) bad++;
        CHECK(bad == 0, "%s: every output pixel matches the model (%ld bad)",
              sc.name, bad);

        // Prove the replication factor really is N: a source pixel must occupy
        // exactly N consecutive destination columns.  Pixel equality alone
        // would also pass if BOTH the DUT and the model used the wrong N,
        // since both take scale_n from the same input.
        int run_len = 0, worst = 0, prev = -1;
        for (size_t t = 0; t < in.size(); t++) {
            if (in[t].border) { prev = -1; run_len = 0; continue; }
            if (in[t].x_src == prev) { run_len++; }
            else {
                if (run_len && run_len != n && run_len > worst) worst = run_len;
                prev = in[t].x_src; run_len = 1;
            }
        }
        CHECK(worst == 0, "%s: every source pixel spans exactly %d columns"
              " (worst offending run %d)", sc.name, n, worst);
    }

    // -- Scenario 4d: the full-width corner (border_x == 0) -----------
    // PINNING A PRE-EXISTING QUIRK, not asserting a desirable one.
    //
    // `x_src` is reset by `x_rst` (start-of-line, hcount == 0) at the clock
    // EDGE, so during hcount == 0 the register still holds the previous line's
    // final value.  Normally that does not matter: border_x > 0, so column 0
    // is border and its pixel is blanked anyway.  When the active window fills
    // the raster exactly -- active_w == DST_W, so place_plan.v computes
    // border_x = 0 -- column 0 is ACTIVE and renders the previous row's LAST
    // source pixel.  upscale.v also gives x_rst priority over x_step_en, so
    // column 0 does not advance the walk either.
    //
    // This is exactly what scanout_display.v did before the split, and it is
    // UNREACHABLE in production: border_x = 0 needs active_w >= 1920 and the
    // widest Q700 mode renders 1280 (640x480 at N=2).  It is pinned here so
    // that (a) the behaviour is recorded rather than rediscovered by someone
    // debugging a first-pixel artifact, and (b) if anyone gives x_step_en
    // priority, or adds a mode that fills the raster exactly, this check
    // changes and the change is visible in the diff.
    {
        Scenario sc = {"8bpp N=4 full-width", 0, DST_W / 4, 32, 4, false, true, -1, -1};
        std::printf("\n-- %s --\n", sc.name);
        Placement p = plan(sc.hres, sc.vres, sc.scale_n);
        CHECK(p.border_x == 0, "full-width corner really has border_x == 0 (got %d)",
              p.border_x);
        run_scenario(sc, in, out);

        long bad = 0;
        for (size_t t = MAX_PROBE_LAT + 1; t < out.size(); t++)
            if (out[t].rgb != expect_pixel(sc, in[t - EXPECTED_LATENCY])) bad++;
        CHECK(bad == 0, "%s: every output pixel matches the model (%ld bad)",
              sc.name, bad);

        // Column 0 of an active line renders the PREVIOUS row's last source
        // pixel, because x_src has not been reset yet.
        int stale = 0, clean = 0;
        for (size_t t = 0; t < in.size(); t++)
            if (in[t].h == 0 && !in[t].border) {
                if (in[t].x_src == SRC_W - 1) stale++; else clean++;
            }
        CHECK(stale > 0,
              "full-width corner: column 0 shows the previous row's last source"
              " pixel on %d of %d active lines (x_rst lands after the sample)",
              stale, stale + clean);

        // Every source pixel after that spans exactly N columns; the run the
        // line end truncates is discarded, as in the bordered scenarios above.
        int run_len = 0, prev = -1, rest_bad = 0;
        bool skip_first = true;
        for (size_t t = 0; t < in.size(); t++) {
            if (in[t].border) { prev = -1; run_len = 0; skip_first = true; continue; }
            if (in[t].x_src == prev) { run_len++; }
            else {
                if (run_len) {
                    if (skip_first) skip_first = false;
                    else if (run_len != 4) rest_bad++;
                }
                prev = in[t].x_src; run_len = 1;
            }
        }
        CHECK(rest_bad == 0, "full-width corner: every source column after the"
              " stale one spans exactly 4 (%d runs did not)", rest_bad);
    }

    // ── Scenario 5: 24bpp direct, N=1 ────────────────────────────────
    // The direct-colour candidate must arrive on the SAME cycle as the palette
    // read.  clut.v registers it for exactly that reason; if that register is
    // dropped the direct path runs one stage early and only this scenario
    // notices.
    {
        Scenario sc = {"24bpp direct N=1", 0, SRC_W, SRC_H, 1, true, true, -1, -1};
        std::printf("\n-- %s --\n", sc.name);
        run_scenario(sc, in, out);

        int nm, mism;
        int L = measure_latency(in, out, [&sc](const Stage0& s, const Out& o) {
                return o.rgb == expect_pixel(sc, s); }, &nm, &mism);
        report_latency("direct-colour bypass", L, nm, mism);

        long bad = 0;
        for (size_t t = MAX_PROBE_LAT + 1; t < out.size(); t++)
            if (out[t].rgb != expect_pixel(sc, in[t - EXPECTED_LATENCY])) bad++;
        CHECK(bad == 0, "24bpp direct: every output pixel matches the model (%ld bad)", bad);
    }

    // ── Scenario 6: single-cycle ring starvation ─────────────────────
    // Drops disp_ready for ONE destination pixel well inside the active
    // window.  The resulting black pixel must land exactly five cycles later,
    // which is the only probe that pins pix_valid_pipe on its own (in every
    // other scenario pix_valid tracks the border exactly).
    {
        Placement p = plan(SRC_W, SRC_H, 1);
        const int starve_h = p.border_x + 20;
        const int starve_v = p.border_y + 10;
        Scenario sc = {"starve 1 pixel", 0, SRC_W, SRC_H, 1, false, true,
                       starve_h, starve_v};
        std::printf("\n-- %s --\n", sc.name);
        run_scenario(sc, in, out);

        int nm, mism;
        int L = measure_latency(in, out, [](const Stage0& s, const Out& o) {
                return (o.rgb != 0) == (!s.border && s.pix_valid); }, &nm, &mism);
        report_latency("pix_valid", L, nm, mism);

        // And it must actually have starved something -- otherwise the probe
        // above degenerates into the border probe.
        long blacked = 0;
        for (size_t t = 0; t < in.size(); t++)
            if (!in[t].border && !in[t].pix_valid) blacked++;
        CHECK(blacked > 0, "starvation probe actually starved %ld pixel(s)", blacked);
    }

    // ── Scenario 7: boot splash ──────────────────────────────────────
    // dafb_live low, so the compositor substitutes the logo.  splash_pipe is
    // the one control pipe with no other consumer, so nothing else in the
    // suite can pin its depth.
    {
        Scenario sc = {"boot splash", 0, SRC_W, SRC_H, 1, false, false, -1, -1};
        std::printf("\n-- %s --\n", sc.name);
        run_scenario(sc, in, out);

        int nm, mism;
        int L = measure_latency(in, out, [](const Stage0& s, const Out& o) {
                const uint32_t want = (s.de && s.splash) ? SPLASH_FG : SPLASH_BG;
                return o.rgb == want; }, &nm, &mism);
        report_latency("splash", L, nm, mism);

        long lit = 0;
        for (size_t t = 0; t < out.size(); t++) if (out[t].rgb == SPLASH_FG) lit++;
        CHECK(lit > 100, "splash actually rendered %ld foreground pixels", lit);
    }

    std::printf("\n%d pass / %d fail\n", n_pass, n_fail);
    delete dut;
    return n_fail == 0 ? 0 : 1;
}
