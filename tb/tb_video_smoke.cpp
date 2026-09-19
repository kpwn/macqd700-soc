// tb_video_smoke.cpp — Unit tb for VIDEO_SMOKE preload path.
//
// Exercises rtl/mac/video/vram_smoke.v driving rtl/sys/vram.v at reset,
// then walks the streaming read port over the full 128×48×8bpp frame
// and verifies each pixel matches the SMPTE-bar pattern.
//
// Pass criteria:
//   1. smoke_done rises within a bounded number of cycles after reset.
//   2. Every pixel read via the streaming port matches the golden
//      per-pixel value (low-nibble DAFB CLUT index for the bar colour).
//   3. The frame is ALSO dumped to build/video_smoke/frame.ppm —
//      a human can eyeball it (see docs/video_smoke.md).
//
// Run via: `make tb-video-smoke`  (or `make video-smoke` to also dump
// the PPM and print a path to it).

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <verilated.h>
#include "Vtb_video_smoke.h"

static Vtb_video_smoke* dut = nullptr;
static uint64_t sim_time = 0;

static int n_pass = 0;
static int n_fail = 0;
static constexpr int VRAM_READ_LATENCY = 5;

#define CHECK(cond, msg) do { \
    if (cond) { n_pass++; std::printf("  [PASS] %s\n", msg); } \
    else      { n_fail++; std::printf("  [FAIL] %s\n", msg); } \
} while (0)

static void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    sim_time++;
}

static uint8_t smoke_clut_index_for_bar(int bar_idx) {
    static const uint8_t idx[8] = {
        0xF,  // WHITE
        0xB,  // YELLOW
        0xE,  // CYAN
        0xA,  // GREEN
        0xD,  // MAGENTA
        0x9,  // RED
        0xC,  // BLUE
        0x0,  // BLACK
    };
    return idx[bar_idx & 0x7];
}

// Golden model — must match rtl/mac/video/vram_smoke.v pixel_data at BPP=8.
static uint8_t golden_pixel(int x, int /*y*/, int fb_w) {
    int bar_w = fb_w / 8;
    int bar_idx = x / bar_w;
    if (bar_idx > 7) bar_idx = 7;
    return smoke_clut_index_for_bar(bar_idx);
}

static void mkdirp(const std::string& p) {
    ::mkdir(p.c_str(), 0755);
}

static void write_ppm(const std::string& path, int w, int h,
                      const std::vector<uint8_t>& fb) {
    FILE* fp = std::fopen(path.c_str(), "wb");
    if (!fp) {
        std::printf("  [WARN] could not open %s for write (%s)\n",
                    path.c_str(), std::strerror(errno));
        return;
    }
    std::fprintf(fp, "P6\n%d %d\n255\n", w, h);
    // Expand the same low-nibble DAFB CLUT indexes the real scaler consumes.
    static const uint8_t palette[16][3] = {
        {  0,  0,  0},  // 0 black
        {128,  0,  0},
        {  0,128,  0},
        {128,128,  0},
        {  0,  0,128},
        {128,  0,128},
        {  0,128,128},
        {192,192,192},
        { 64, 64, 64},
        {255,  0,  0},
        {  0,255,  0},
        {255,255,  0},
        {  0,  0,255},
        {255,  0,255},
        {  0,255,255},
        {255,255,255},
    };
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint8_t v = fb[y * w + x];
            int idx = v & 0x0f;
            std::fputc(palette[idx][0], fp);
            std::fputc(palette[idx][1], fp);
            std::fputc(palette[idx][2], fp);
        }
    }
    std::fclose(fp);
    std::printf("  [INFO] framebuffer dumped to %s\n", path.c_str());
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_video_smoke;

    // Matches tb_video_smoke.v parameters.
    const int FB_W = 128;
    const int FB_H = 48;
    const int N_PX = FB_W * FB_H;

    // Reset
    dut->clk = 0;
    dut->rst = 1;
    dut->rd_addr = 0;
    dut->rd_en   = 0;
    dut->eval();
    for (int i = 0; i < 8; i++) tick();
    dut->rst = 0;

    // --------------------------------------------------------------
    // Phase A: wait for smoke_done — smoke writer walks every word
    //   N_WORDS = ceil(N_PX * BPP / DATA_WIDTH) = ceil(6144*8/128) = 384
    //   × ~4 cycles/word + 8 cycle warm-up → ≤ 2000 cycles.
    //   Use a generous 10k-cycle budget to absorb any FSM quirks.
    // --------------------------------------------------------------
    uint64_t done_cycle = 0;
    for (uint64_t i = 0; i < 10000 && !dut->smoke_done; i++) {
        tick();
        if (dut->smoke_done) done_cycle = sim_time;
    }
    CHECK(dut->smoke_done, "smoke_done asserted within 10k cycles");
    std::printf("  [INFO] smoke_done at cycle %llu\n",
                (unsigned long long)done_cycle);

    // Give AXI pipeline a few cycles to drain completely.
    for (int i = 0; i < 16; i++) tick();

    // --------------------------------------------------------------
    // Phase B: walk the streaming read port over every pixel and
    //   compare against the golden model.
    // --------------------------------------------------------------
    std::vector<uint8_t> fb(N_PX, 0);
    int mismatch = 0;
    int first_mismatch_idx = -1;
    uint8_t first_mismatch_got = 0, first_mismatch_exp = 0;

    // Back-to-back reads: hold rd_en=1 and drive a rising address sequence.
    // rd_data / rd_valid trail rd_addr / rd_en by the VRAM XPM read latency.
    dut->rd_en = 1;
    for (int p = 0; p < N_PX; p++) {
        dut->rd_addr = p;
        tick();
        int out_p = p - (VRAM_READ_LATENCY - 1);
        if (out_p < 0) continue;

        // CONSUMER: the byte at rd_addr is the always-valid top lane of the
        // 4-byte group the streaming port now returns.
        uint8_t got = (uint8_t)((dut->rd_data >> 24) & 0xFFu);
        bool    val = dut->rd_valid;
        fb[out_p] = got;

        int x = out_p % FB_W;
        int y = out_p / FB_W;
        uint8_t exp = golden_pixel(x, y, FB_W);

        if (!val) {
            mismatch++;
            if (first_mismatch_idx < 0) {
                first_mismatch_idx = out_p;
                first_mismatch_got = got;
                first_mismatch_exp = exp;
            }
        } else if (got != exp) {
            mismatch++;
            if (first_mismatch_idx < 0) {
                first_mismatch_idx = out_p;
                first_mismatch_got = got;
                first_mismatch_exp = exp;
            }
        }
    }
    dut->rd_en = 0;
    dut->rd_addr = 0;
    for (int tail = 0; tail < VRAM_READ_LATENCY - 1; tail++) {
        tick();
        int out_p = N_PX - (VRAM_READ_LATENCY - 1) + tail;

        // CONSUMER: the byte at rd_addr is the always-valid top lane of the
        // 4-byte group the streaming port now returns.
        uint8_t got = (uint8_t)((dut->rd_data >> 24) & 0xFFu);
        bool    val = dut->rd_valid;
        fb[out_p] = got;

        int x = out_p % FB_W;
        int y = out_p / FB_W;
        uint8_t exp = golden_pixel(x, y, FB_W);

        if (!val) {
            mismatch++;
            if (first_mismatch_idx < 0) {
                first_mismatch_idx = out_p;
                first_mismatch_got = got;
                first_mismatch_exp = exp;
            }
        } else if (got != exp) {
            mismatch++;
            if (first_mismatch_idx < 0) {
                first_mismatch_idx = out_p;
                first_mismatch_got = got;
                first_mismatch_exp = exp;
            }
        }
    }

    CHECK(mismatch == 0, "all FB_W*FB_H pixels match the SMPTE-bar pattern");
    if (mismatch) {
        int x = first_mismatch_idx % FB_W;
        int y = first_mismatch_idx / FB_W;
        std::printf("  [DIAG] first mismatch @ (x=%d, y=%d) idx=%d "
                    "got=0x%02x exp=0x%02x (%d total)\n",
                    x, y, first_mismatch_idx,
                    first_mismatch_got, first_mismatch_exp, mismatch);
    }

    // --------------------------------------------------------------
    // Phase C: dump the captured FB as a PPM so a human can visually
    //   inspect it (open with `eog build/video_smoke/frame.ppm` or
    //   equivalent).  Also dump the first 32 raw bytes.
    // --------------------------------------------------------------
    std::printf("  [INFO] first 32 raw bytes of FB:");
    for (int i = 0; i < 32; i++) std::printf(" %02x", fb[i]);
    std::printf("\n");

    mkdirp("build");
    mkdirp("build/video_smoke");
    write_ppm("build/video_smoke/frame.ppm", FB_W, FB_H, fb);

    // --------------------------------------------------------------
    // Summary
    // --------------------------------------------------------------
    std::printf("────────────────────────────────\n");
    std::printf("  pass=%d fail=%d\n", n_pass, n_fail);

    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
