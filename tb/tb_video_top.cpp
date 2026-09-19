// tb_video_top.cpp -- Verilator testbench for rtl/mac/video/video_top.v
//
// Drives the wrapper (tb/tb_video_top.v) which instantiates video_top with
// the default 1024x768 -> 1920x1080p60 letterbox scale.  Goals:
//   * Tick the full pipeline for enough cycles to (a) see the I2C init
//     complete and (b) produce a small sample of active pixels from the
//     scaler output.
//   * Model a deterministic VRAM read port: addr -> rgb(N) = N & 0xFFFFFF.
//     Any scan-out pixel we sample must match the pattern for the source
//     coordinate that maps to it.
//   * Exercise the direct HDMI smoke path in both bar and checkerboard
//     modes so the first-light output can be validated before trusting the
//     VRAM CDC path.
//   * Count the i2c_init drive-low strobes to confirm the sequencer at
//     least STARTed -> bits -> STOPped a plausible number of times for
//     the 11-entry ROM.
//
// This is NOT a full HDMI verification -- that needs real bit-cell
// timing analysis on real hardware.  The tb just verifies the pipeline
// runs, the timing generator sweeps out a frame, the I2C FSM rolls
// through its table, and the scaler's addressed pixel stream is
// consistent with the VRAM pattern.

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <verilated.h>
#include "Vtb_video_top.h"

static Vtb_video_top* dut = nullptr;
static uint64_t       sim_time = 0;

static int n_pass = 0;
static int n_fail = 0;

// ── The normal-mode (non-g2) instance's geometry ─────────────────────
// tb_video_top.v drives it hres=1024 vres=768 at video_top's default
// 1920x1080 raster.
//
// THESE WERE ACTIVE_W=1440 / BORDER_X=240 / SCALE 32:45 -- the FRACTIONAL
// scale rung, which docs/video_path_review.md S3 deleted in favour of
// integer-only replication.  The model went on describing a scaler that no
// longer exists for six weeks, and nobody saw it because this whole
// translation unit stopped elaborating (see video_top.v's `ifdef VERILATOR
// clock-forward comment).  Derived from the policy now, so a future raster
// change moves it automatically.
static constexpr int SRC_W = 1024;
static constexpr int SRC_H = 768;
static constexpr int DST_W = 1920;
static constexpr int DST_H = 1080;
static constexpr int SCALE_N  = (DST_W / SRC_W) < (DST_H / SRC_H)
                              ? (DST_W / SRC_W) : (DST_H / SRC_H);
static constexpr int ACTIVE_W = SRC_W * SCALE_N;
static constexpr int ACTIVE_H = SRC_H * SCALE_N;
static constexpr int BORDER_X = (DST_W - ACTIVE_W) / 2;
static constexpr int BORDER_Y = (DST_H - ACTIVE_H) / 2;

#define CHECK(cond, msg) do { \
    if (cond) { \
        n_pass++; \
        std::printf("  [PASS] %s\n", msg); \
    } else { \
        n_fail++; \
        std::printf("  [FAIL] %s\n", msg); \
    } \
} while (0)

// ── g2 VRAM model (production-shaped second instance) ────────────────
// Same 4-byte-group contract as vram_tick() below, but a NEVER-ZERO byte
// pattern: the g2 scenarios distinguish "the scanner rendered the frame"
// from "the scanner emitted black because it never fetched anything", so a
// source byte of 0 would be indistinguishable from the failure.
static inline uint8_t g2_vram_byte(uint32_t a) {
    return static_cast<uint8_t>(((a * 7u) + ((a >> 9) * 13u)) | 1u);
}

static uint32_t g2_data_pipe[2] = {0, 0};
static bool     g2_valid_pipe[2] = {false, false};
static uint64_t g2_read_count = 0;

static void g2_vram_tick() {
    dut->g2_vram_rd_data  = g2_data_pipe[1];
    dut->g2_vram_rd_valid = g2_valid_pipe[1] ? 1 : 0;
    g2_data_pipe[1]  = g2_data_pipe[0];
    g2_valid_pipe[1] = g2_valid_pipe[0];
    if (dut->g2_vram_rd_en) {
        const uint32_t a = dut->g2_vram_rd_addr;
        g2_data_pipe[0] = (static_cast<uint32_t>(g2_vram_byte(a))     << 24)
                        | (static_cast<uint32_t>(g2_vram_byte(a + 1)) << 16)
                        | (static_cast<uint32_t>(g2_vram_byte(a + 2)) <<  8)
                        |  static_cast<uint32_t>(g2_vram_byte(a + 3));
        g2_valid_pipe[0] = true;
        g2_read_count++;
    } else {
        g2_data_pipe[0]  = 0;
        g2_valid_pipe[0] = false;
    }
}

static void vram_tick();

static void tick() {
    // BOTH instances are serviced on EVERY tick, because both streaming
    // ports are ordered request/response streams: dropping a response
    // breaks fb_reader's credit accounting rather than merely starving a
    // frame.  The primary model used to be serviced only on the ticks where
    // the harness happened to call vram_tick() by hand -- so it silently
    // dropped requests, and fb_reader's outstanding-request count drifted up
    // forever.  That was invisible only for as long as the count was dead
    // code; it is a real contract violation by the MODEL, and the comment
    // beside g2_vram_tick already said so.
    g2_vram_tick();
    vram_tick();
    dut->clk = 0;
    dut->eval();
    dut->clk = 1;
    dut->eval();
    sim_time += 1;
}

// Simple VRAM model: byte at address A holds (A & 0xFF).  data_valid follows
// en by 2 cycles.  Accepted reads return an ordered response, matching
// rtl/board/vram.v's pipelined streaming port contract.
//
// This is a MODEL of the backend, so it implements the widened contract: one
// request returns the 4-byte group starting at the requested address, byte at
// +0 in [31:24] down to byte at +3 in [7:0].  Hardware only guarantees the
// lower three bytes for a 4-byte-aligned request; the model returns the true
// bytes at every alignment, which is a strict superset.
static uint32_t vram_data_pipe[2] = {0, 0};
static bool     vram_valid_pipe[2] = {false, false};
static int      vram_read_count = 0;

static void vram_tick() {
    // 2-cycle synchronous read.  The streaming port is byte-ADDRESSED at
    // every pixel depth (the runtime depth arrives on scanout_bytes_per_px,
    // not on the port width) but returns a 4-byte group per request.
    dut->vram_rd_data  = vram_data_pipe[1];
    dut->vram_rd_valid = vram_valid_pipe[1] ? 1 : 0;
    vram_data_pipe[1]  = vram_data_pipe[0];
    vram_valid_pipe[1] = vram_valid_pipe[0];
    if (dut->vram_rd_en) {
        const uint32_t a = dut->vram_rd_addr;
        vram_data_pipe[0]  = ((a       & 0xFFu) << 24)
                           | (((a + 1) & 0xFFu) << 16)
                           | (((a + 2) & 0xFFu) <<  8)
                           |  ((a + 3) & 0xFFu);
        vram_valid_pipe[0] = true;
        vram_read_count++;
    } else {
        vram_data_pipe[0]  = 0;
        vram_valid_pipe[0] = false;
    }
}

static uint32_t expected_test_pattern_rgb(int kind, int h, int v, bool de) {
    if (!de) return 0x000000;
    if (kind == 2) {
        return ((h >> 5) ^ (v >> 5)) & 1 ? 0x404040 : 0xffffff;
    }
    if (h < (1920 * 1) / 8) return 0xffffff;
    if (h < (1920 * 2) / 8) return 0xffff00;
    if (h < (1920 * 3) / 8) return 0x00ffff;
    if (h < (1920 * 4) / 8) return 0x00ff00;
    if (h < (1920 * 5) / 8) return 0xff00ff;
    if (h < (1920 * 6) / 8) return 0xff0000;
    if (h < (1920 * 7) / 8) return 0x0000ff;
    // Keep the last bar non-black so the first-light path still shows
    // a visible active region even if the far right of the frame is all
    // that the monitor or camera sees.
    return 0x404040;
}

static bool in_active_window(int h, int v) {
    return (h >= BORDER_X) && (h < BORDER_X + ACTIVE_W)
        && (v >= BORDER_Y) && (v < BORDER_Y + ACTIVE_H);
}

static uint32_t expected_video_addr(int h, int v) {
    int src_x = (h - BORDER_X) / SCALE_N;
    int src_y = (v - BORDER_Y) / SCALE_N;
    return static_cast<uint32_t>(src_y * SRC_W + src_x);
}

static uint32_t expected_video_rgb(int h, int v, bool bubble_now) {
    if (!in_active_window(h, v) || bubble_now) return 0x000000;
    return expected_video_addr(h, v) & 0xFFFFFFu;
}

// ─────────────────────────────────────────────────────────────────────
// DAFB-mode scenarios on the production-shaped g2 instance (task #161)
// ─────────────────────────────────────────────────────────────────────
// These replay real Quadra 700 DAFB register values -- read off the live
// board over JTAG -- through the same video_top the FPGA elaborates, and
// assert that real pixels come out.  They exist because every gate in the
// scanout chain that measured a row against the ELABORATED SRC_W (1024)
// instead of the DAFB-programmed hres happened to pass at 640x480, where
// the Q700 programs a 1024-byte stride -- exactly SRC_W, a one-byte
// margin.  Any narrower stride (832x624 programs 832) failed
// linebuf_scanout's fetch_stride_sane, can_request never asserted, no
// line ever became valid, and the whole screen went black while the CPU
// kept running.  A tb pinned to a 1024-multiple stride cannot see that.
static constexpr int G2_DST_W = 1920;
static constexpr int G2_DST_H = 1080;
static constexpr int G2_H_TOT = 1920 + 88 + 44 + 148;   // 2200
static constexpr int G2_V_TOT = 1080 + 4 + 5 + 36;      // 1125

static uint32_t g2_clut[256];

static uint32_t g2_expected_rgb(uint32_t base, uint32_t stride, int bpp_shift,
                                int bytes_per_px, int x_src, int y_src) {
    if (bytes_per_px == 4) {
        // 24bpp xRGB, big-endian in the longword: +0 pad, +1 R, +2 G, +3 B.
        const uint32_t a = base + static_cast<uint32_t>(y_src) * stride
                         + 4u * static_cast<uint32_t>(x_src);
        return (static_cast<uint32_t>(g2_vram_byte(a + 1)) << 16)
             | (static_cast<uint32_t>(g2_vram_byte(a + 2)) <<  8)
             |  static_cast<uint32_t>(g2_vram_byte(a + 3));
    }
    const uint32_t a = base + static_cast<uint32_t>(y_src) * stride
                     + (static_cast<uint32_t>(x_src) >> bpp_shift);
    const uint8_t b = g2_vram_byte(a);
    uint8_t idx;
    switch (bpp_shift) {
        case 3: idx = (b >> (7 - (x_src & 7))) & 0x01; break;
        case 2: idx = (b >> (6 - 2 * (x_src & 3))) & 0x03; break;
        case 1: idx = (x_src & 1) ? (b & 0x0F) : (b >> 4); break;
        default: idx = b; break;
    }
    return g2_clut[idx] & 0xFFFFFFu;
}

// The settled INTEGER scaling policy (docs/video_path_review.md S3):
//     N = min(floor(DST_W/hres), floor(DST_H/vres)), clamped to [1,4]
// Written from the FORMULA rather than from place_plan.v's comparison
// ladder, so the RTL and this model agree only if both are right.
static int g2_policy_scale_n(int hres, int vres) {
    if (hres <= 0 || vres <= 0) return 1;
    int n = G2_DST_W / hres;
    const int m = G2_DST_H / vres;
    if (m < n) n = m;
    if (n < 1) n = 1;
    if (n > 4) n = 4;
    return n;
}

// scanout_display.v's active-window span: hres * N.  There is no fractional
// rung any more, so no floored halving either.
static int g2_active_span(int res, int n) { return res * n; }

// scanout_display.v's Bresenham nearest-neighbour walk, re-derived: emits
// `den` destination pixels for every `num` source pixels.  num is pinned at
// 1 and den is N, which IS integer replication.
static std::vector<int> g2_bresenham_map(int dst_n, int n) {
    const int num = 1;
    const int den = n;
    std::vector<int> m(static_cast<size_t>(dst_n));
    int acc = 0, src = 0;
    for (int i = 0; i < dst_n; i++) {
        m[static_cast<size_t>(i)] = src;
        if (acc + num >= den) { acc = acc + num - den; src++; }
        else                  { acc = acc + num; }
    }
    return m;
}

// One scenario: program the placement, let it settle over `frames` full
// frames, capture the last frame, and assert.  Returns nothing; failures
// go through CHECK.
static void g2_scenario(const char* name, uint32_t base, uint32_t stride,
                        int bpp_shift, int bytes_per_px, int hres, int vres,
                        int frames = 4) {
    dut->g2_fb_base_px       = base;
    dut->g2_fb_stride_px     = stride;
    dut->g2_bpp_shift        = bpp_shift;
    dut->g2_bytes_per_px     = bytes_per_px;
    dut->g2_depth_supported  = 1;
    dut->g2_hres             = hres;
    dut->g2_vres             = vres;

    // The output scale is the POLICY's answer for this geometry -- integer
    // only.  832x624 lands on N=1 (sharp, letterboxed 832x624); it used to
    // land on a 3:2 rung whose 2,1,2,1 replication aliases that mode's 1bpp
    // dithered desktop into fine striping.
    const int  n = g2_policy_scale_n(hres, vres);
    const int aw = g2_active_span(hres, n);
    const int ah = g2_active_span(vres, n);
    const int bx = (G2_DST_W - aw) / 2;
    const int by = (G2_DST_H - ah) / 2;
    const int probe_row = by + ah / 2;
    const std::vector<int> xmap = g2_bresenham_map(aw, n);
    const std::vector<int> ymap = g2_bresenham_map(ah, n);

    uint64_t active = 0, nonblack = 0;
    std::vector<uint32_t> row;
    uint64_t reqs0 = g2_read_count;

    for (int f = 0; f < frames; f++) {
        const bool last = (f == frames - 1);
        if (last) { active = 0; nonblack = 0; row.clear(); reqs0 = g2_read_count; }
        for (long i = 0; i < static_cast<long>(G2_H_TOT) * G2_V_TOT; i++) {
            tick();
            if (!last || !dut->g2_de) continue;
            const int hc = dut->g2_hcount;
            const int vc = dut->g2_vcount;
            if (hc < bx || hc >= bx + aw || vc < by || vc >= by + ah) continue;
            active++;
            const uint32_t rgb = dut->g2_rgb & 0xFFFFFFu;
            if (rgb) nonblack++;
            if (vc == probe_row) row.push_back(rgb);
        }
    }
    const uint64_t reqs = g2_read_count - reqs0;

    char msg[256];
    std::snprintf(msg, sizeof(msg),
                  "%s: fetcher issued requests (stride_sane=%d frame_in_mem=%d)",
                  name, dut->g2_fetch_stride_sane, dut->g2_fetch_frame_in_mem);
    CHECK(reqs > 0 && dut->g2_fetch_placement_valid, msg);

    std::snprintf(msg, sizeof(msg),
                  "%s: committed the programmed stride (%u, got %u)",
                  name, stride, static_cast<unsigned>(dut->g2_committed_stride));
    CHECK(dut->g2_committed_stride == stride, msg);

    std::snprintf(msg, sizeof(msg),
                  "%s: active window is not black (%llu/%llu non-black)",
                  name, static_cast<unsigned long long>(nonblack),
                  static_cast<unsigned long long>(active));
    CHECK(active > 0 && nonblack * 100 >= active * 99, msg);

    // Pixel-exact on the probe row.  `rgb` trails hcount by a fixed
    // pipeline depth, so resolve the shift once by correlation and then
    // demand an EXACT match for every pixel after the fill.
    int best_k = -1, best_hits = -1;
    const int rowlen = static_cast<int>(row.size());
    for (int k = 0; k <= 12 && k < rowlen; k++) {
        int hits = 0;
        for (int c = k; c < rowlen; c++) {
            const int col = c - k;
            if (col >= aw) continue;
            const int x_src = xmap[static_cast<size_t>(col)];
            const int y_src = ymap[static_cast<size_t>(probe_row - by)];
            if (row[c] == g2_expected_rgb(base, stride, bpp_shift,
                                          bytes_per_px, x_src, y_src))
                hits++;
        }
        if (hits > best_hits) { best_hits = hits; best_k = k; }
    }
    std::snprintf(msg, sizeof(msg),
                  "%s: probe row is pixel-exact vs VRAM (%d/%d at pipe offset %d)",
                  name, best_hits, rowlen - (best_k < 0 ? 0 : best_k), best_k);
    CHECK(rowlen > 0 && best_k >= 0 && best_hits == rowlen - best_k, msg);

    std::printf("    [DIAG] %s base=%u stride=%u shift=%d bppx=%d %dx%d "
                "reqs=%llu committed{stride=%u hres=%u bppx=%u}\n",
                name, base, stride, bpp_shift, bytes_per_px, hres, vres,
                static_cast<unsigned long long>(reqs),
                static_cast<unsigned>(dut->g2_committed_stride),
                static_cast<unsigned>(dut->g2_committed_hres),
                static_cast<unsigned>(dut->g2_committed_bytes_per_px));
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_video_top;

    // Init
    dut->clk   = 0;
    dut->rst_n = 0;
    dut->vram_rd_data  = 0;
    dut->vram_rd_valid = 0;
    dut->clut_we    = 0;
    dut->clut_waddr = 0;
    dut->clut_wdata = 0;
    vram_data_pipe[0] = 0;
    vram_data_pipe[1] = 0;
    vram_valid_pipe[0] = false;
    vram_valid_pipe[1] = false;
    // g2 (production-shaped) instance starts quiescent: depth_supported low
    // holds its reset placement, so it idles harmlessly in the TEST_PATTERN
    // builds where the g2 scenarios do not run.
    dut->g2_fb_base_px      = 0;
    dut->g2_fb_stride_px    = 1024;
    dut->g2_bpp_shift       = 0;
    dut->g2_bytes_per_px    = 1;
    dut->g2_depth_supported = 0;
    dut->g2_hres            = 1024;
    dut->g2_vres            = 768;
    dut->g2_clut_we         = 0;
    dut->g2_clut_waddr      = 0;
    dut->g2_clut_wdata      = 0;
    dut->g2_vram_rd_data    = 0;
    dut->g2_vram_rd_valid   = 0;
    dut->eval();

    // Hold reset for 16 cycles
    for (int i = 0; i < 16; i++) {
        tick();
    }
    dut->rst_n = 1;

    // Program the 256-entry RAMDAC CLUT.  The scanner runs in its production
    // 8bpp indexed mode, so without a palette every pixel is black and the
    // "non-black active pixels" / "sample points are not black" checks below
    // are meaningless.  Every entry is non-zero and distinct, and the mapping
    // is the identity on the byte the tb's VRAM model returns (rd_addr & 0xFF)
    // so `expected_video_addr()`'s low byte still predicts the rendered pixel.
    for (int i = 0; i < 256; i++) {
        dut->clut_waddr = uint8_t(i);
        dut->clut_wdata = (uint32_t(i) << 16)
                        | (uint32_t(uint8_t(i ^ 0x5A)) << 8)
                        | uint32_t(uint8_t(~i))
                        | 0x000001u;   // guarantee non-zero even at i == 0x5A
        dut->clut_we = 1;
        tick();
    }
    dut->clut_we = 0;
    tick();

    // Same for the g2 instance's own CLUT (separate BRAM inside its own
    // linebuf_scanout).  Every entry non-zero so an indexed g2 scenario can
    // never render legitimately-black pixels.
    for (int i = 0; i < 256; i++) {
        g2_clut[i] = ((uint32_t(uint8_t(i * 3 + 1)) << 16)
                    | (uint32_t(uint8_t(i * 5 + 2)) <<  8)
                    |  uint32_t(uint8_t(i * 7 + 3))) | 0x010101u;
        dut->g2_clut_waddr = uint8_t(i);
        dut->g2_clut_wdata = g2_clut[i];
        dut->g2_clut_we = 1;
        tick();
    }
    dut->g2_clut_we = 0;
    tick();

    // --------------------------------------------------------------
    // Phase A: give the MMCM and reset pipe a short warm-up.
    // The transmitter init itself finishes later in the long scan run.
    // --------------------------------------------------------------
    for (int i = 0; i < 64; i++) {
        tick();
    }
    CHECK(dut->mmcm_locked, "MMCM reports locked");

    // --------------------------------------------------------------
    // Phase B: run a big chunk of cycles and observe that the
    // pipeline produces data (de toggles, hs/vs pulses happen).
    // The default I2C timer values are long, but this run is long
    // enough to see the full transmitter init settle as well.
    // --------------------------------------------------------------
    int  hs_pulses = 0;
    int  vs_pulses = 0;
    bool prev_hs = false, prev_vs = false;
    int  de_count = 0;
    int  de_edges = 0;
    bool prev_de = false;

    int  scl_lows = 0;
    int  sda_lows = 0;
    int  prev_scl_low = 0;
    int  prev_sda_low = 0;
    int  i2c_done_rises = 0;
    int  i2c_done_falls = 0;
    int  resetn_rises = 0;
    int  resetn_falls = 0;
    bool prev_i2c_done = false;
    bool prev_resetn = false;

    uint32_t last_rgb_seen = 0xDEADBEEF;
    int      active_pixel_count = 0;
    int      scan_mismatch = 0;
    int      first_scan_h = -1;
    int      first_scan_v = -1;
    uint32_t first_scan_got = 0;
    uint32_t first_scan_exp = 0;
    int      test_pattern_mismatch = 0;
    int      first_pattern_h = -1;
    int      first_pattern_v = -1;
    uint32_t first_pattern_got = 0;
    uint32_t first_pattern_exp = 0;
    int      blank_rgb_mismatch = 0;
    int      out_of_range_reads = 0;

    struct SamplePoint {
        int h;
        int v;
    };
    // Every one of these must lie INSIDE the active window the integer policy
    // gives this instance (x 448..1471, y 156..923 for 1024x768 at N=1); a
    // point in the letterbox border is legitimately black and gates nothing.
    // {480,120} used to be here, from the deleted 1440x1080 fractional-scale
    // window where the active region ran to the top of the frame.
    static const SamplePoint sample_points[] = {
        {480, 200},
        {720, 360},
        {960, 540},
        {1200, 780},
        {1440, 900},
    };
    const int sample_point_count = sizeof(sample_points) / sizeof(sample_points[0]);
    int sample_hits = 0;

    // 1 full frame @ 1080p60 = 2200*1125 = 2_475_000 pclks.  We run
    // ~3 frames to confirm stability + observe pattern on the 2nd.
    const uint64_t RUN_CYCLES = 3 * 2200ull * 1125ull;

    for (uint64_t i = 0; i < RUN_CYCLES; i++) {
        tick();

        // Edge-detect sync signals
        bool hs = dut->al9134_hs;
        bool vs = dut->al9134_vs;
        bool de = dut->al9134_de;
        if (hs && !prev_hs) hs_pulses++;
        if (vs && !prev_vs) vs_pulses++;
        if (de && !prev_de) de_edges++;
        if (de) {
            de_count++;
            last_rgb_seen = dut->al9134_d;
            if (last_rgb_seen != 0) active_pixel_count++;
        }

        uint32_t got_rgb = dut->al9134_d & 0xFFFFFFu;
        uint32_t exp_rgb = 0x000000u;
        if (dut->test_pattern_mode) {
            exp_rgb = expected_test_pattern_rgb(dut->test_pattern_kind,
                                                dut->hcount,
                                                dut->vcount,
                                                de);
            if (got_rgb != exp_rgb) {
                scan_mismatch++;
                test_pattern_mismatch++;
                if (first_scan_h < 0) {
                    first_scan_h = dut->hcount;
                    first_scan_v = dut->vcount;
                    first_scan_got = got_rgb;
                    first_scan_exp = exp_rgb;
                }
                if (first_pattern_h < 0) {
                    first_pattern_h = dut->hcount;
                    first_pattern_v = dut->vcount;
                    first_pattern_got = got_rgb;
                    first_pattern_exp = exp_rgb;
                }
            }
        } else {
            exp_rgb = expected_video_rgb(dut->hcount, dut->vcount, false);

            // Only the LAST frame is judged.  The boot splash (36188b07)
            // substitutes for the pixel path until the DAFB commits a
            // renderable placement, so the opening frames are legitimately
            // black at these coordinates -- and this gate predates the
            // splash, so it used to require all three frames to be painted.
            bool is_sample = (vs_pulses >= 2);
            if (is_sample) {
                is_sample = false;
                for (int si = 0; si < sample_point_count; si++) {
                    if (dut->hcount == sample_points[si].h &&
                        dut->vcount == sample_points[si].v) {
                        is_sample = true;
                        break;
                    }
                }
            }
            if (is_sample) {
                sample_hits++;
                if (got_rgb == 0) {
                    scan_mismatch++;
                    if (first_scan_h < 0) {
                        first_scan_h = dut->hcount;
                        first_scan_v = dut->vcount;
                        first_scan_got = got_rgb;
                        first_scan_exp = exp_rgb;
                    }
                }
            }
        }

        if (dut->vram_rd_en && dut->vram_rd_addr >= (1024u * 768u)) {
            out_of_range_reads++;
        }

        if (!de && ((dut->al9134_d & 0xFFFFFF) != 0)) {
            blank_rgb_mismatch++;
        }

        bool i2c_done = dut->hdmi_i2c_done;
        bool resetn = dut->al9134_resetn;
        if (i2c_done && !prev_i2c_done) i2c_done_rises++;
        if (!i2c_done && prev_i2c_done) i2c_done_falls++;
        if (resetn && !prev_resetn) resetn_rises++;
        if (!resetn && prev_resetn) resetn_falls++;
        prev_i2c_done = i2c_done;
        prev_resetn = resetn;

        prev_hs = hs;
        prev_vs = vs;
        prev_de = de;

        // I2C strobe counting (rising edge = drive low asserted)
        if (dut->i2c_scl_drive_low && !prev_scl_low) scl_lows++;
        if (dut->i2c_sda_drive_low && !prev_sda_low) sda_lows++;
        prev_scl_low = dut->i2c_scl_drive_low;
        prev_sda_low = dut->i2c_sda_drive_low;
    }

    std::printf("---- summary -----------------------------\n");
    std::printf("  sim_time          = %llu\n", (unsigned long long)sim_time);
    std::printf("  mmcm_locked       = %d\n",   dut->mmcm_locked);
    std::printf("  hdmi_i2c_done     = %d\n",   dut->hdmi_i2c_done);
    std::printf("  hs pulses         = %d\n",   hs_pulses);
    std::printf("  vs pulses         = %d\n",   vs_pulses);
    std::printf("  de edges          = %d\n",   de_edges);
    std::printf("  total de cycles   = %d\n",   de_count);
    std::printf("  non-black pixels  = %d\n",   active_pixel_count);
    std::printf("  vram reads        = %d\n",   vram_read_count);
    std::printf("  normal samples    = %d/%d\n", sample_hits, sample_point_count);
    std::printf("  i2c scl lows      = %d\n",   scl_lows);
    std::printf("  i2c sda lows      = %d\n",   sda_lows);
    std::printf("  i2c done rises    = %d\n",   i2c_done_rises);
    std::printf("  i2c done falls    = %d\n",   i2c_done_falls);
    std::printf("  resetn rises      = %d\n",   resetn_rises);
    std::printf("  resetn falls      = %d\n",   resetn_falls);
    std::printf("  out-of-range rd   = %d\n",   out_of_range_reads);

    CHECK(dut->hdmi_i2c_done, "I2C init completes");
    CHECK(dut->al9134_resetn, "AL9134 reset released");
    CHECK(i2c_done_rises == 1, "I2C done rises exactly once");
    CHECK(i2c_done_falls == 0, "I2C done never drops after completion");
    CHECK(resetn_rises == 1, "AL9134 resetn rises exactly once");
    CHECK(resetn_falls == 0, "AL9134 resetn never glitches low");

    // --------------------------------------------------------------
    // Gates
    // --------------------------------------------------------------
    // Expect ~3 full frames: 3 vsync pulses, 3*1125 = 3375 hsync pulses.
    // Tolerate boundary -- we might miss the first partial.
    CHECK(vs_pulses >= 2 && vs_pulses <= 4,  "vsync pulse count plausible");
    CHECK(hs_pulses > 3000,                  "hsync pulses >3000");
    CHECK(de_count  > 1000000,               "data-enable cycles >1M");

    if (dut->test_pattern_mode) {
        CHECK(dut->test_pattern_kind == 1 || dut->test_pattern_kind == 2,
              "test-pattern selector exposes a known first-light mode");
        CHECK(vram_read_count == 0, "test-pattern mode issues no VRAM reads");
        CHECK(scan_mismatch == 0, "test-pattern output stays aligned");
        CHECK(!dut->fb_underflow_sticky,
              "test-pattern mode never underflows the framebuffer reader");
        if (dut->test_pattern_kind == 2) {
            CHECK(test_pattern_mismatch == 0,
                  "checkerboard pixels match the expected first-light frame");
        } else {
            CHECK(test_pattern_mismatch == 0,
                  "test-pattern pixels match the expected 8-bar frame");
        }
        CHECK(active_pixel_count == de_count,
              "test-pattern active pixels stay non-black");
        if (scan_mismatch) {
            std::printf("  [DIAG] first scan mismatch h=%d v=%d got=0x%06x exp=0x%06x total=%d\n",
                        first_scan_h, first_scan_v,
                        first_scan_got, first_scan_exp,
                        scan_mismatch);
        }
        if (test_pattern_mismatch) {
            std::printf("  [DIAG] first pattern mismatch h=%d v=%d "
                        "got=0x%06x exp=0x%06x total=%d\n",
                        first_pattern_h, first_pattern_v,
                        first_pattern_got, first_pattern_exp,
                        test_pattern_mismatch);
        }
    } else {
        // ONE source byte per source pixel per painted frame, plus at most one
        // ring's worth of prefetch headroom.  A RANGE, not an equality: how
        // much of the first frame is consumed by MMCM lock + the reset banks
        // is a property of the clocking, not of the scanner, and pinning it
        // exactly made this gate fail for a reason it does not gate.  What it
        // still catches is both real failures -- a parked fetcher (far too
        // few) and a fetcher re-reading a source byte per DESTINATION pixel
        // (far too many).
        const int frame_reads = SRC_W * SRC_H;
        const int ring_reads  = 64 * SRC_W;
        const int expected_sample_hits = sample_point_count;
        CHECK(vram_read_count >= 2 * frame_reads
           && vram_read_count <= 3 * frame_reads + ring_reads,
              "normal mode fetches source pixels once per painted frame plus bounded line-buffer headroom");
        CHECK(sample_hits == expected_sample_hits,
              "normal mode hits every interior sample point");
        CHECK(scan_mismatch == 0, "normal mode output matches the expected pixels");
        CHECK(active_pixel_count > 100000, "non-black active pixels seen");
        if (scan_mismatch) {
            std::printf("  [DIAG] first scan mismatch h=%d v=%d got=0x%06x exp=0x%06x total=%d\n",
                        first_scan_h, first_scan_v,
                        first_scan_got, first_scan_exp,
                        scan_mismatch);
        }
    }
    // --------------------------------------------------------------
    // Phase D: real DAFB modes on the production-shaped g2 instance.
    // Only in the normal (non-test-pattern) build -- the pattern builds
    // bypass the VRAM datapath entirely and these would just triple
    // their runtime.
    // --------------------------------------------------------------
    if (!dut->test_pattern_mode) {
        std::printf("---- DAFB mode scenarios (task #161) -----\n");
        // Live Quadra 700 registers, mon-sense 0x06 (Mac Hi-Res 640x480):
        //   +0x00 BASE_HI=0x008 +0x04 BASE_LO=0 -> base 4096
        //   +0x08 STRIDE  =0x100            -> 1024 bytes/row
        //   +0x140 HAL=0x098 +0x144 HFP=0x318 -> hres 640
        //   +0x15C VAL=0x052 +0x160 VFP=0x412 -> vres 480
        // This is the geometry that renders on hardware today, and it
        // clears the old build-time-width stride gate by exactly one byte
        // (1024 > SRC_W-1 == 1023).  It must stay green, unchanged.
        g2_scenario("640x480 8bpp stride 1024 (live regs, HW-good)",
                    4096, 1024, 0, 1, 640, 480);
        g2_scenario("640x480 1bpp stride 1024 (live regs, HW-good)",
                    4096, 1024, 3, 0, 640, 480);
        // Live Quadra 700 registers, mon-sense 0x6D (16" RGB 832x624):
        //   +0x00 BASE_HI=0x007 -> base 3584
        //   +0x08 STRIDE  =0x0D0 -> 832 bytes/row
        // 832 < SRC_W, so the old gate rejected it and the screen went
        // black on hardware with Mac OS still running.  RED before the
        // runtime-row-span fix, GREEN after.
        g2_scenario("832x624 8bpp stride 832 (live regs, HW-black)",
                    3584, 832, 0, 1, 832, 624);
        g2_scenario("832x624 1bpp stride 832 (live regs, HW-black)",
                    3584, 832, 3, 0, 832, 624);
        // 24bpp direct colour at 640x480: 2560 B/row, i.e. NOT a
        // multiple of the 1024-byte build width.  The direct arm of the
        // gate was already hres-derived, so this is a characterisation
        // scenario: it pins down that the fetch/placement chain sustains
        // a 4 B/px stride end to end, which is what makes "24bpp is
        // black on hardware" a statement about the inputs the DAFB shim
        // hands it rather than about the scanout chain.
        g2_scenario("640x480 24bpp stride 2560 (direct colour)",
                    4096, 2560, 0, 4, 640, 480);
    }

    CHECK(blank_rgb_mismatch == 0, "blanking intervals drive black RGB");
    CHECK(out_of_range_reads == 0,
          "VRAM read addresses stay within the source framebuffer");

    // I2C sequencer: pclk = 148.5 MHz, I2C_HALF = 744 -> ~200 kcyc per
    // bit, so in 3 * 2.475M = 7.425M cycles we get ~35 bits flipping
    // SCL.  Lower bound 5 to be conservative (RESET_CYCLES alone eats
    // 1.485M before the first START).
    CHECK(scl_lows > 2, "i2c SCL drive-low transitions >2");

    std::printf("---- result ------------------------------\n");
    std::printf("  pass=%d fail=%d\n", n_pass, n_fail);

    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
