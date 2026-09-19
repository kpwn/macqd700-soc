// tb_scanout_1bpp.cpp -- harness for the pixel-exact 1bpp scanout gate.
// ---------------------------------------------------------------------------
// See tb_scanout_1bpp.v's header for WHY.  This file:
//   * paints a 1bpp source whose every BYTE and every BIT WITHIN the byte
//     differs, so all hres source pixels of every row are individually
//     checked (tb-scanout-frames paints 0x00/0xFF rows and cannot);
//   * reads the destination raster and the scanner's own elaborated window
//     out of the DUT (the cfg_* ports), so one harness covers 1080p, 720p or
//     anything else a -G override elaborates, and cannot drift off them;
//   * re-derives the integer scale N with its own expression and requires
//     place_plan.v to agree, rather than telling the DUT what N to use;
//   * compares EVERY destination pixel of whole frames -- inside the active
//     window against the source bit, outside it against black.
// ---------------------------------------------------------------------------
#include <verilated.h>
#include "Vtb_scanout_1bpp.h"

#include <algorithm>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

// Read out of the DUT (cfg_fb_bytes) so this harness cannot drift off the
// aperture the design is actually elaborated with.
static uint32_t FB_BYTES = 0;

static Vtb_scanout_1bpp* dut = nullptr;
static int pass_count = 0;
static int fail_count = 0;

static int DST_W = 0, DST_H = 0, H_TOTAL = 0, V_TOTAL = 0;

static void check(bool ok, const std::string& what) {
    if (ok) { pass_count++; printf("  [PASS] %s\n", what.c_str()); }
    else    { fail_count++; printf("  [FAIL] %s\n", what.c_str()); }
}

// -- Model framebuffer + fetch port --------------------------------------
static std::vector<uint8_t> fbmem;
struct InFlight { uint32_t addr; long long due; };
static std::deque<InFlight> inflight;
static long long model_tick    = 0;
static int       model_latency = 9;
static size_t    model_depth   = 32;
static bool      m_ready = false, m_valid = false;
static uint32_t  m_data  = 0;

static uint32_t fetch_group(uint32_t a) {
    uint32_t b0 = (a + 0 < FB_BYTES) ? fbmem[a + 0] : 0;
    uint32_t b1 = (a + 1 < FB_BYTES) ? fbmem[a + 1] : 0;
    uint32_t b2 = (a + 2 < FB_BYTES) ? fbmem[a + 2] : 0;
    uint32_t b3 = (a + 3 < FB_BYTES) ? fbmem[a + 3] : 0;
    return (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
}

// -- Captured output frame -----------------------------------------------
static std::vector<uint32_t> img;          // DST_W x DST_H, current frame
static std::vector<int>      line_n;       // de_out pixels seen per line

static void tick_one() {
    bool     en_pre    = dut->fb_rd_en;
    uint32_t addr_pre  = dut->fb_rd_addr;
    bool     ready_pre = m_ready;
    bool     valid_pre = m_valid;

    dut->pclk = 1; dut->eval();
    dut->pclk = 0; dut->eval();
    model_tick++;

    if (valid_pre && !inflight.empty()) inflight.pop_front();
    if (en_pre && ready_pre) inflight.push_back({addr_pre, model_tick + model_latency});

    m_ready = inflight.size() < model_depth;
    m_valid = (!inflight.empty()) && (inflight.front().due <= model_tick);
    m_data  = m_valid ? fetch_group(inflight.front().addr) : 0;
    dut->fb_rd_ready = m_ready;
    dut->fb_rd_valid = m_valid;
    dut->fb_rd_data  = m_data;

    // de_out trails the VTG by the 5-stage output pipeline, which is far
    // shorter than any front porch here, so bucketing by the LIVE vcount is
    // exact and the nth de_out pixel of a line is destination x = n.
    if (dut->de_out) {
        int v = dut->vcount;
        if (v >= 0 && v < DST_H) {
            int x = line_n[v]++;
            if (x < DST_W) img[(size_t)v * DST_W + x] = dut->rgb & 0xFFFFFFu;
        }
    }
}

static uint32_t clut_entry(int i) {
    return (uint32_t)(((i & 0xFF) << 16) | (((255 - i) & 0xFF) << 8)
                      | (((i * 7 + 0x11) & 0xFF)));
}

static void program_clut() {
    dut->clut_we = 0;
    for (int i = 0; i < 256; i++) {
        dut->clut_we    = 1;
        dut->clut_waddr = (uint8_t)i;
        dut->clut_wdata = clut_entry(i);
        tick_one();
    }
    dut->clut_we = 0;
    tick_one();
}

// -- The source image ----------------------------------------------------
// Every byte of every row differs, and the BITS within each byte differ, so
// a wrong sub-byte slice, a wrong byte index, a horizontal shift or a
// row/column swap all land on a different colour.
static uint8_t src_byte(int y, int b) {
    return (uint8_t)((b * 37 + y * 101 + 0x5A) & 0xFF);
}

static void paint(int hres, int vres, uint32_t base, uint32_t stride) {
    std::fill(fbmem.begin(), fbmem.end(), 0);
    int bytes = (hres + 7) / 8;
    for (int y = 0; y < vres; y++) {
        uint32_t row = base + (uint32_t)y * stride;
        for (int b = 0; b < bytes; b++) {
            uint32_t a = row + (uint32_t)b;
            if (a >= FB_BYTES) break;
            fbmem[a] = src_byte(y, b);
        }
    }
}

// Source pixel (x,y) as the DAFB/MAME convention has it: MSB-first within the
// byte, so x=0 is bit 7 of byte 0.
static int src_bit(int x, int y) {
    return (src_byte(y, x >> 3) >> (7 - (x & 7))) & 1;
}

// -- Scenarios -----------------------------------------------------------
// Q700 1bpp geometries that fit the scanner window, plus one stressed fetch
// port.  The scale N is deliberately NOT listed: it is whatever place_plan.v
// gives for the raster this binary was elaborated with, and the harness
// re-derives it.  That is the point -- at 1080p 640x480 and 512x384 are N=2,
// at 720p they are N=1, and the 1bpp path had never been run at N=1 before
// 788cd2b0 shipped that raster.
struct Scenario {
    const char* name;
    int         hres, vres;
    uint32_t    base, stride;
    int         latency;     // fetch-port response latency, pclk cycles
    int         depth;       // fetch-port outstanding-request ceiling
};

static const Scenario SCENARIOS[] = {
    // The depth and geometry Mac OS actually boots in.
    {"640x480 stride 1024",          640, 480, 0,      1024,  9, 32},
    // The same, with a fetch port four times slower and a quarter as deep.
    // At N=1 the display returns ONE credit per destination line instead of
    // one per two, so the ring's refill window halves -- the only thing about
    // the fetch side that the scale factor actually changes.
    {"640x480 slow/shallow port",    640, 480, 0,      1024, 36,  8},
    // A non-zero base and the TIGHT row pitch (80 bytes = exactly one 1bpp
    // row), so the per-row address arithmetic is exercised away from the easy
    // power-of-two case and mode_admit's stride gate sits on its boundary.
    {"640x480 base 0x2000 pitch 80", 640, 480, 0x2000,   80,  9, 32},
    // The Q700's other small mode; N=2 at 1080p, N=1 at 720p.
    {"512x384 stride 1024",          512, 384, 0,      1024,  9, 32},
    // N=1 at BOTH rasters, so it is the control: if the two above go red while
    // this one stays green, the fault is the scale factor and not the depth.
    {"832x624 stride 832",           832, 624, 0,       832,  9, 32},
};

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_scanout_1bpp;
    dut->pclk = 0;
    dut->eval();
    FB_BYTES = dut->cfg_fb_bytes;
    fbmem.assign(FB_BYTES, 0);

    DST_W   = dut->cfg_dst_w;
    DST_H   = dut->cfg_dst_h;
    H_TOTAL = dut->cfg_h_total;
    V_TOTAL = dut->cfg_v_total;
    img.assign((size_t)DST_W * DST_H, 0);
    line_n.assign(DST_H, 0);

    printf("== pixel-exact 1bpp scanout, destination %dx%d "
           "(H_TOTAL %d, V_TOTAL %d) ==\n", DST_W, DST_H, H_TOTAL, V_TOTAL);
    printf("   scanner window %dx%d, aperture 0x%X bytes\n\n",
           (int)dut->cfg_src_w, (int)dut->cfg_src_h, FB_BYTES);

    const int WARMUP_FRAMES = 3;
    const int CHECK_FRAMES  = 2;

    for (const Scenario& s : SCENARIOS) {
        paint(s.hres, s.vres, s.base, s.stride);

        dut->rst          = 1;
        dut->fb_base_px   = s.base;
        dut->fb_stride_px = s.stride;
        dut->bpp_shift    = 3;      // 1bpp
        dut->bytes_per_px = 1;      // indexed
        dut->hres         = s.hres;
        dut->vres         = s.vres;
        dut->scale_n      = 0;      // ASK place_plan for the policy's N
        dut->dafb_live    = 1;
        dut->clut_we      = 0;
        dut->fb_rd_ready  = 0;
        dut->fb_rd_valid  = 0;
        dut->fb_rd_data   = 0;

        inflight.clear();
        model_latency = s.latency;
        model_depth   = (size_t)s.depth;
        m_ready = true; m_valid = false;
        for (int i = 0; i < 20; i++) tick_one();
        dut->rst = 0;
        for (int i = 0; i < 4; i++) tick_one();
        program_clut();

        // Harness-side policy, derived independently of place_plan.v.
        int n = DST_W / s.hres;
        if (DST_H / s.vres < n) n = DST_H / s.vres;
        if (n < 1) n = 1;
        if (n > 4) n = 4;
        const int active_w = s.hres * n, active_h = s.vres * n;
        const int border_x = (DST_W - active_w) / 2;
        const int border_y = (DST_H - active_h) / 2;

        printf("-- %s : N=%d, active %dx%d at (%d,%d)\n",
               s.name, n, active_w, active_h, border_x, border_y);
        check((int)dut->cfg_scale_n == n,
              std::string(s.name) + ": place_plan's N matches the policy "
              "re-derived here (RTL " + std::to_string((int)dut->cfg_scale_n)
              + ", harness " + std::to_string(n) + ")");

        auto run_one_frame = [&]() {
            std::fill(img.begin(), img.end(), 0u);
            std::fill(line_n.begin(), line_n.end(), 0);
            while (!(dut->hcount == 0 && dut->vcount == 0)) tick_one();
            for (long long t = 0; t < (long long)H_TOTAL * V_TOTAL; t++) tick_one();
        };

        for (int f = 0; f < WARMUP_FRAMES; f++) run_one_frame();

        int total_bad = 0, bad_active = 0, bad_border = 0;
        int first_bad_x = -1, first_bad_y = -1;
        uint32_t first_bad_got = 0, first_bad_exp = 0;
        // Histogram of the horizontal displacement that WOULD have explained
        // a wrong active pixel, so a systematic shift names itself instead of
        // only being counted.
        std::vector<int> shift_hist(33, 0);

        for (int f = 0; f < CHECK_FRAMES; f++) {
            run_one_frame();
            for (int v = 0; v < DST_H; v++) {
                for (int x = 0; x < DST_W; x++) {
                    uint32_t got = img[(size_t)v * DST_W + x];
                    bool in_act = (x >= border_x) && (x < border_x + active_w)
                               && (v >= border_y) && (v < border_y + active_h);
                    int xs = in_act ? (x - border_x) / n : 0;
                    int ys = in_act ? (v - border_y) / n : 0;
                    uint32_t exp = in_act ? clut_entry(src_bit(xs, ys)) : 0u;
                    if (got == exp) continue;
                    total_bad++;
                    if (in_act) {
                        bad_active++;
                        for (int d = -16; d <= 16; d++) {
                            int xx = xs + d;
                            if (xx < 0 || xx >= s.hres) continue;
                            if (got == clut_entry(src_bit(xx, ys))) {
                                shift_hist[d + 16]++;
                                break;
                            }
                        }
                    } else {
                        bad_border++;
                    }
                    if (first_bad_x < 0) {
                        first_bad_x = x; first_bad_y = v;
                        first_bad_got = got; first_bad_exp = exp;
                    }
                }
            }
        }

        if (total_bad) {
            printf("   first mismatch at (x=%d, v=%d): got %06X expected %06X\n",
                   first_bad_x, first_bad_y, first_bad_got, first_bad_exp);
            printf("   mismatches: %d in the active window, %d in the border\n",
                   bad_active, bad_border);
            printf("   displacement histogram (source px, +ve = shifted LEFT):\n");
            for (int d = -16; d <= 16; d++)
                if (shift_hist[d + 16])
                    printf("      d=%+3d : %d\n", d, shift_hist[d + 16]);
            int v0 = border_y + (active_h / 2);
            printf("   display line %d, first 48 active pixels:\n     got ", v0);
            for (int i = 0; i < 48; i++) {
                uint32_t got = img[(size_t)v0 * DST_W + border_x + i];
                int gb = (got == clut_entry(1)) ? 1
                       : (got == clut_entry(0)) ? 0 : 9;
                printf("%d", gb);
            }
            printf("\n     exp ");
            for (int i = 0; i < 48; i++)
                printf("%d", src_bit(i / n, (v0 - border_y) / n));
            printf("\n");
        }

        check(total_bad == 0,
              std::string(s.name) + ": every destination pixel of "
              + std::to_string(CHECK_FRAMES) + " consecutive frames matches "
              "the 1bpp source (" + std::to_string(total_bad) + " mismatches)");
        check(!dut->line_underflow_sticky,
              std::string(s.name) + ": line_underflow_sticky never fired");
    }

    printf("\n%d passed, %d failed\n", pass_count, fail_count);
    delete dut;
    return fail_count ? 1 : 0;
}
