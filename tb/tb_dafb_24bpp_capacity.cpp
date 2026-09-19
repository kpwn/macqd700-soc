// tb_dafb_24bpp_capacity.cpp -- harness for tb_dafb_24bpp_capacity.v
//
// Drives the SHIPPING scanout geometry (SRC 1024x768 / 2 MiB aperture /
// 1920x1080 DST) with the DAFB register values read off the live board over
// JTAG at 8bpp and at 24bpp, and asserts that BOTH depths are admitted by the
// placement gates and paint a non-black active window.
//
// The 8bpp pass is the POSITIVE CONTROL: it uses the identical harness, the
// identical VRAM pattern and the identical pixel-counting code as the 24bpp
// pass, so a 24bpp failure cannot be blamed on the harness.
//
// Live-board register values reproduced here (BUILD_ID 0x217A265B):
//     8bpp   BASE 0x008  STRIDE 0x100  CONFIG 0x30  PCBR 0x98
//     24bpp  BASE 0x008  STRIDE 0x400  CONFIG 0x32  PCBR 0x9c

#include <verilated.h>
#include "Vtb_dafb_24bpp_capacity.h"

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

// 1920x1080p60 timing (what video_top's VTG generates).
static const int DST_W  = 1920;
static const int DST_H  = 1080;
static const int H_TOTAL = 2200;
static const int V_TOTAL = 1125;

// DAFB register byte offsets inside the 0xF9800000 window.
static const uint32_t REG_BASE_HI = 0x000;
static const uint32_t REG_STRIDE  = 0x008;
static const uint32_t REG_CONFIG  = 0x010;
static const uint32_t REG_PCBR    = 0x220;
static const uint32_t REG_HAL     = 0x140;
static const uint32_t REG_HFP     = 0x144;
static const uint32_t REG_VAL     = 0x15C;
static const uint32_t REG_VFP     = 0x160;

static Vtb_dafb_24bpp_capacity* dut = nullptr;
static vluint64_t main_time = 0;
static int failures = 0;

// vram_clk is an INDEPENDENT clock, exactly as in production (video_top
// wires fb_reader's vram_clk to core_clk while linebuf runs on the 148.5 MHz
// pixel clock).  g_vclk_div is the vram_clk half-period measured in pclk
// half-periods: 1 => same rate, 3 => vram_clk is pclk/3 (a harsher drain
// rate than the real core_clk, so it stresses the request FIFO harder).
static int g_vclk_div = 1;
static int vclk_phase = 0;

static void step_vclk() {
    if (++vclk_phase >= g_vclk_div) {
        vclk_phase = 0;
        dut->vram_clk = !dut->vram_clk;
    }
}

static void tick() {
    dut->pclk = 0;
    step_vclk();
    dut->eval();
    main_time++;
    dut->pclk = 1;
    step_vclk();
    dut->eval();
    main_time++;
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
        // Sample readys with the current drive, then advance the clock.
        bool aw_hit = (!aw_done) && dut->dafb_awready;
        bool w_hit  = (!w_done)  && dut->dafb_wready;
        tick();
        if (aw_hit) { aw_done = true; dut->dafb_awvalid = 0; }
        if (w_hit)  { w_done  = true; dut->dafb_wvalid  = 0; }
        dut->eval();
    }
    dut->dafb_awvalid = 0;
    dut->dafb_wvalid  = 0;
    for (int i = 0; i < 64; i++) {
        tick();
        if (dut->dafb_bvalid) break;
    }
    tick();
    dut->dafb_bready = 0;
    tick();
}

static void clut_load_gray_ramp() {
    for (int i = 0; i < 256; i++) {
        dut->clut_we    = 1;
        dut->clut_waddr = i;
        dut->clut_wdata = (uint32_t)((i << 16) | (i << 8) | i);
        tick();
    }
    dut->clut_we = 0;
    tick();
}

struct FrameStats {
    long de_pixels    = 0;
    long nonblack     = 0;
    long max_channel  = 0;
    double mean_lum   = 0.0;
    long fetch_reqs   = 0;   // cycles with fb_rd_en asserted (accepted 1/cycle)
    long fetch_rsps   = 0;   // cycles with fb_rd_valid asserted
    // Exact-value tally.  At 24bpp every active pixel must be 0x00FFFFFF
    // (the framebuffer word is 0x00ffffff; R/G/B are bytes +1/+2/+3).  Any
    // other value means the direct-colour unpack picked the wrong lane --
    // notably 0x000000 would be the xRGB PAD byte leaking into all three
    // planes, which is the shape a bad bpp_shift/lane-select would take.
    long exact_white  = 0;
    long exact_black  = 0;
    long other        = 0;
    uint32_t first_other = 0xFFFFFFFFu;
};

// The "lit" pixel value this depth is expected to paint.  24bpp/8bpp both
// reach 0xFFFFFF; at 1bpp the only palette entries reachable are 0 and 1, so
// with the gray-ramp CLUT a lit pixel is 0x010101.  Keeping the tally
// parametric lets the 1bpp scenario reuse the identical counting code.
static uint32_t g_lit_value = 0x00FFFFFFu;

// Walk exactly one 1920x1080p60 frame.  When `capture` is set, tally the
// output pixels the same way the HDMI capture card measures them.
static FrameStats run_frame(bool capture) {
    FrameStats st;
    double lum_acc = 0.0;
    for (int v = 0; v < V_TOTAL; v++) {
        for (int h = 0; h < H_TOTAL; h++) {
            dut->hcount = h;
            dut->vcount = v;
            dut->de_in  = (h < DST_W && v < DST_H) ? 1 : 0;
            dut->hs_in  = (h >= DST_W + 88 && h < DST_W + 88 + 44) ? 1 : 0;
            dut->vs_in  = (v >= DST_H + 4 && v < DST_H + 4 + 5) ? 1 : 0;
            if (capture) {
                if (dut->dbg_fb_rd_en && dut->dbg_fb_rd_ready) st.fetch_reqs++;
                if (dut->dbg_fb_rd_valid) st.fetch_rsps++;
            }
            tick();
            if (capture && dut->scanout_de) {
                uint32_t rgb = dut->scanout_rgb;
                int r = (rgb >> 16) & 0xFF, g = (rgb >> 8) & 0xFF, b = rgb & 0xFF;
                st.de_pixels++;
                if (rgb != 0) st.nonblack++;
                int mx = r > g ? r : g; if (b > mx) mx = b;
                if (mx > st.max_channel) st.max_channel = mx;
                lum_acc += (0.299 * r + 0.587 * g + 0.114 * b);
                if      (rgb == g_lit_value)  st.exact_white++;
                else if (rgb == 0x00000000u) st.exact_black++;
                else {
                    st.other++;
                    if (st.first_other == 0xFFFFFFFFu) st.first_other = rgb;
                }
            }
        }
    }
    if (st.de_pixels) st.mean_lum = lum_acc / (double)st.de_pixels;
    return st;
}

static void dump_gates(const char* tag) {
    printf("  [%s] shim: base=0x%x stride=%u bytes_per_px=%u bpp_shift=%u "
           "depth_ok=%u hres=%u vres=%u\n",
           tag, (unsigned)dut->dbg_shim_base, (unsigned)dut->dbg_shim_stride,
           (unsigned)dut->dbg_shim_bytes_per_px, (unsigned)dut->dbg_shim_bpp_shift,
           (unsigned)dut->dbg_shim_depth_supported,
           (unsigned)dut->dbg_shim_hres, (unsigned)dut->dbg_shim_vres);
    printf("  [%s] placement_sync: depth_ok_stable=%u stride_sane=%u in_range=%u "
           "frame_last_addr=%u min_stride=%u\n",
           tag, (unsigned)dut->dbg_ps_depth_ok, (unsigned)dut->dbg_ps_stride_sane,
           (unsigned)dut->dbg_ps_in_range, (unsigned)dut->dbg_ps_frame_last_addr,
           (unsigned)dut->dbg_ps_min_stride);
    printf("  [%s] committed: base=%u stride=%u bytes_per_px=%u hres=%u vres=%u\n",
           tag, (unsigned)dut->dbg_committed_base, (unsigned)dut->dbg_committed_stride,
           (unsigned)dut->dbg_committed_bytes_per_px,
           (unsigned)dut->dbg_committed_hres, (unsigned)dut->dbg_committed_vres);
    printf("  [%s] linebuf: fetch_direct=%u fetch_wide=%u stride_sane=%u "
           "frame_in_mem=%u prefetch_active=%u req_addr_limit=%u row_last_off=%u\n",
           tag, (unsigned)dut->dbg_fetch_direct, (unsigned)dut->dbg_fetch_wide,
           (unsigned)dut->dbg_fetch_stride_sane, (unsigned)dut->dbg_fetch_frame_in_mem,
           (unsigned)dut->dbg_prefetch_active, (unsigned)dut->dbg_req_addr_limit,
           (unsigned)dut->dbg_row_last_off);
}

// Optional scenario filter for iteration speed: TB_SCEN=5,7 runs only those.
// Unset (the default, and what `make tb-dafb-24bpp-capacity` uses) runs all.
static bool want(int n) {
    const char* e = getenv("TB_SCEN");
    if (!e || !*e) return true;
    std::string s(e);
    size_t p = 0;
    while (p < s.size()) {
        size_t c = s.find(',', p);
        if (c == std::string::npos) c = s.size();
        if (atoi(s.substr(p, c - p).c_str()) == n) return true;
        p = c + 1;
    }
    return false;
}

static void check(bool cond, const std::string& what) {
    if (!cond) {
        printf("  FAIL: %s\n", what.c_str());
        failures++;
    } else {
        printf("  ok  : %s\n", what.c_str());
    }
}

// Full DUT reset + DAFB re-programming.  Used at start-of-day and whenever
// the fetch-port configuration changes (direct <-> fb_reader CDC, vram_clk
// rate), because switching the path mid-flight would strand the responses
// already in the old path and desynchronise linebuf's request/response
// accounting -- a harness artifact, not a DUT bug.
static void reset_and_program() {
    dut->rst = 1;
    for (int i = 0; i < 40; i++) tick();
    dut->rst = 0;
    for (int i = 0; i < 40; i++) tick();

    clut_load_gray_ramp();

    // Swatch timing for the 640x480 mode the board reported.
    //   hres = HFP - HAL              = 0x318 - 0x098 = 640
    //   vres = (VFP>>1) - (VAL>>1)    = 512 - 32      = 480
    axi_write(REG_HAL, 0x098);
    axi_write(REG_HFP, 0x318);
    axi_write(REG_VAL, 0x040);
    axi_write(REG_VFP, 0x400);
    // BASE register 0x008 -> m_base = 8 << 9 = 4096.
    axi_write(REG_BASE_HI, 0x008);
}

// Program the DAFB shim for one depth, settle, and measure a frame.
static FrameStats program_and_measure(const char* tag,
                                      uint32_t stride_reg,
                                      uint32_t config_reg,
                                      uint32_t pcbr_reg) {
    axi_write(REG_STRIDE, stride_reg);
    axi_write(REG_CONFIG, config_reg);
    axi_write(REG_PCBR,   pcbr_reg);
    // Three settle frames: the placement CDC needs two synced samples plus a
    // frame boundary, and linebuf_scanout then needs one restart + one full
    // prefetch pass before the frame it paints is meaningful.
    for (int f = 0; f < 3; f++) run_frame(false);
    dump_gates(tag);
    return run_frame(true);
}

// Why this exists: the c57fc08 walk terminator stops the fetch walk at
// fetch_row_y_last and clears prefetch_active.  Something must RE-ARM it
// every frame.  A single-frame measurement cannot see a failure to re-arm --
// the first frame after a placement restart always fetches.  So measure
// several consecutive frames and require every one of them to fetch.
static std::vector<FrameStats> measure_frames(const char* tag, int n) {
    std::vector<FrameStats> out;
    for (int f = 0; f < n; f++) {
        FrameStats st = run_frame(true);
        printf("  [%s] frame %d: fb_rd_en=%ld fb_rd_valid=%ld nonblack=%ld "
               "lit=%ld black=%ld other=%ld\n",
               tag, f, st.fetch_reqs, st.fetch_rsps, st.nonblack,
               st.exact_white, st.exact_black, st.other);
        printf("      state: prefetch_active=%u can_request=%u in_resync=%u "
               "placement_changed=%u fsm_state=%u credits=%u\n",
               (unsigned)dut->dbg_prefetch_active, (unsigned)dut->dbg_can_request,
               (unsigned)dut->dbg_in_resync,
               (unsigned)dut->dbg_placement_changed,
               (unsigned)dut->dbg_state,
               (unsigned)dut->dbg_credits);
        printf("      walk : req=(y%u,x%u) rsp_y=%u y_src=%u "
               "row_y_last: latched=%u live=%u  row_px_last: latched=%u live=%u\n",
               (unsigned)dut->dbg_req_y, (unsigned)dut->dbg_req_x,
               (unsigned)dut->dbg_rsp_y, (unsigned)dut->dbg_y_src,
               (unsigned)dut->dbg_fetch_row_y_last, (unsigned)dut->dbg_row_y_last_in,
               (unsigned)dut->dbg_fetch_row_px_last, (unsigned)dut->dbg_row_last_idx_in);
        printf("      gates: outstanding_empty=%u credits=%u addr_oob=%u "
               "stale_count=%u fsm_state=%u fb_rd_addr=%u\n",
               (unsigned)dut->dbg_outstanding_empty,
               (unsigned)dut->dbg_credits, (unsigned)dut->dbg_req_addr_oob,
               (unsigned)dut->dbg_stale_count,
               (unsigned)dut->dbg_state, (unsigned)dut->dbg_fb_rd_addr);
        out.push_back(st);
    }
    return out;
}

static void report(const char* tag, const FrameStats& st) {
    long active = 1280L * 960L;   // hres/vres at 2x integer scale
    printf("  [%s] de_pixels=%ld nonblack=%ld (%.1f%% of the %ld-pixel active "
           "window) max_channel=%ld mean_lum=%.2f\n",
           tag, st.de_pixels, st.nonblack,
           100.0 * (double)st.nonblack / (double)active, active,
           st.max_channel, st.mean_lum);
    printf("  [%s] exact 0xFFFFFF=%ld  exact 0x000000=%ld  other=%ld (first 0x%06x)\n",
           tag, st.exact_white, st.exact_black, st.other,
           st.other ? st.first_other : 0u);
    printf("  [%s] fetch: fb_rd_en cycles=%ld  fb_rd_valid cycles=%ld  (per frame)\n",
           tag, st.fetch_reqs, st.fetch_rsps);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_dafb_24bpp_capacity;

    dut->pclk = 0;
    dut->rst = 1;
    dut->dafb_awvalid = 0;
    dut->dafb_wvalid = 0;
    dut->dafb_bready = 0;
    dut->dafb_wstrb = 0xF;
    dut->clut_we = 0;
    dut->hcount = 0;
    dut->vcount = 0;
    dut->de_in = 0;
    dut->hs_in = 0;
    dut->vs_in = 0;
    dut->mem_latency = 1;        // legacy 1-cycle always-ready model
    dut->mem_backpressure = 0;
    dut->mem_via_fb_reader = 0;
    dut->vram_clk = 0;
    dut->mem_drop_rsp = 0;
    reset_and_program();

    printf("=== tb_dafb_24bpp_capacity ===\n");
    printf("geometry: SRC 1024x768, aperture 0x200000 (2 MiB), DST 1920x1080\n");

    int scenarios_run = 0;

    // ── Scenario 1: 8bpp -- POSITIVE CONTROL (works on hardware) ──────
    printf("\n-- scenario 1: 8bpp  (BASE 0x008 STRIDE 0x100 CONFIG 0x30 PCBR 0x98)\n");
    if (want(1)) {
    scenarios_run++;
    {
        FrameStats st = program_and_measure("8bpp", 0x100, 0x30, 0x98);
        report("8bpp", st);
        check(dut->dbg_shim_hres == 640 && dut->dbg_shim_vres == 480,
              "DAFB shim decodes 640x480");
        check(dut->dbg_shim_bytes_per_px == 1, "shim decodes 8bpp = 1 B/px");
        check(dut->dbg_committed_bytes_per_px == 1,
              "placement_sync commits 8bpp depth to the scanner");
        check(dut->dbg_committed_stride == 1024,
              "placement_sync commits stride 1024");
        check(dut->dbg_committed_base == 4096,
              "placement_sync commits base 4096");
        check(st.nonblack > (1280L * 960L) / 2,
              "8bpp active window is more than half non-black");
        check(st.max_channel == 255, "8bpp frame reaches full white");
        // Gray-ramp CLUT: index 0 -> 0x000000, index 255 -> 0xFFFFFF.  The
        // VRAM pattern is one 0x00 byte per three 0xff bytes, so the active
        // window must be exactly 3/4 white and 1/4 black, nothing else.
        check(st.other == 0, "8bpp emits only palette entries 0 and 255");
        check(st.exact_white == 3L * (1280L * 960L) / 4,
              "8bpp white pixel count is exactly 3/4 of the active window");
    }

    }

    // ── Scenario 2: 24bpp -- the reported black screen ────────────────
    printf("\n-- scenario 2: 24bpp (BASE 0x008 STRIDE 0x400 CONFIG 0x32 PCBR 0x9c)\n");
    if (want(2)) {
    scenarios_run++;
    {
        FrameStats st = program_and_measure("24bpp", 0x400, 0x32, 0x9c);
        report("24bpp", st);
        check(dut->dbg_shim_bytes_per_px == 4, "shim decodes 24bpp = 4 B/px");
        check(dut->dbg_shim_depth_supported == 1, "shim reports 24bpp supported");
        // 640x480x24bpp at base 4096 / stride 4096 occupies
        //     4096 + 4096*479 + 2559 = 1,968,639 bytes
        // which fits the 2 MiB aperture.  It MUST be admitted.
        check(dut->dbg_ps_in_range == 1,
              "placement_sync range gate admits 640x480x24bpp in 2 MiB");
        check(dut->dbg_committed_bytes_per_px == 4,
              "placement_sync commits 24bpp depth to the scanner");
        check(dut->dbg_committed_stride == 4096,
              "placement_sync commits stride 4096");
        check(dut->dbg_fetch_direct == 1, "linebuf latched direct-colour mode");
        check(dut->dbg_fetch_wide == 1, "linebuf latched the WIDE 24bpp fetch");
        check(dut->dbg_fetch_frame_in_mem == 1,
              "linebuf frame-in-memory gate admits 640x480x24bpp");
        check(st.nonblack > (1280L * 960L) / 2,
              "24bpp active window is more than half non-black");
        check(st.max_channel == 255, "24bpp frame reaches full white");
        // EXACT-VALUE proof for the direct-colour unpack.  The framebuffer
        // word is 0x00ffffff big-endian: pad 0x00 at +0, R/G/B 0xff at
        // +1/+2/+3.  Every active pixel must therefore be exactly 0xFFFFFF.
        //   - a count of 0x000000 instead would mean the PAD byte reached all
        //     three planes (the "bpp_shift picks the wrong byte" shape);
        //   - any partial value (0xFF0000 / 0x00FF00 / ...) would mean plane 1
        //     or 2 was never written or came from the wrong lane;
        //   - a CLUT-looked-up value would mean the output mux failed to
        //     select the direct path on fetch_direct.
        check(st.exact_white == 1280L * 960L,
              "every 24bpp active pixel is exactly 0xFFFFFF "
              "(R/G/B planes all written from the right lanes, CLUT bypassed)");
        check(st.other == 0, "no partially-assembled 24bpp pixels");
        check(dut->dbg_shim_bpp_shift == 0,
              "bpp_shift is 0 at 24bpp (and that is harmless - see above)");
    }

    }

    // ── Scenario 3: back to 8bpp -- HW says 256 restores instantly ────
    printf("\n-- scenario 3: back to 8bpp (STRIDE 0x100 CONFIG 0x30 PCBR 0x98)\n");
    if (want(3)) {
    scenarios_run++;
    {
        FrameStats st = program_and_measure("8bpp-again", 0x100, 0x30, 0x98);
        report("8bpp-again", st);
        check(dut->dbg_committed_bytes_per_px == 1,
              "returning to 8bpp re-commits the indexed depth");
        check(st.nonblack > (1280L * 960L) / 2,
              "8bpp active window recovers after the 24bpp excursion");
    }

    }

    // ── Scenario 4: 8bpp over MANY consecutive frames ─────────────────
    // POSITIVE CONTROL for scenario 5.  Proves the multi-frame liveness
    // measurement itself works on a depth that is known-good on hardware,
    // so a scenario-5 failure cannot be blamed on the measurement.
    printf("\n-- scenario 4: 8bpp x 4 consecutive frames (fetch must re-arm every frame)\n");
    if (want(4)) {
    scenarios_run++;
    {
        std::vector<FrameStats> fr = measure_frames("8bpp-multi", 4);
        long lo = fr[0].fetch_reqs, hi = fr[0].fetch_reqs;
        for (const FrameStats& s : fr) {
            if (s.fetch_reqs < lo) lo = s.fetch_reqs;
            if (s.fetch_reqs > hi) hi = s.fetch_reqs;
        }
        // 640x480x8bpp = 307,200 byte requests per frame.
        check(lo > 300000L,
              "8bpp: every one of 4 frames issues a full frame of fetches");
        check(fr.back().nonblack > (1280L * 960L) / 2,
              "8bpp: the 4th frame is still more than half non-black");
        printf("  [8bpp-multi] fetch_reqs min=%ld max=%ld\n", lo, hi);
    }

    }

    // ── Scenario 5: 1bpp -- the depth Mac OS actually boots in ────────
    // Live-board DAFB state at the 0x881f9bfd wedge: base_px 4096,
    // stride_px 1024, 640x480, bpp_shift 3.  vram_rd_addr was frozen at
    // 0x78c4f = 4096 + 1024*479 + 79, i.e. the LAST pixel of a 1bpp frame:
    // the walk completed once and never restarted.
    printf("\n-- scenario 5: 1bpp  (BASE 0x008 STRIDE 0x100 CONFIG 0x30 PCBR 0x80)\n");
    if (want(5)) {
    scenarios_run++;
    {
        // At 1bpp the reachable palette entries are 0 and 1; with the gray
        // ramp a lit pixel is 0x010101.
        g_lit_value = 0x00010101u;
        FrameStats st = program_and_measure("1bpp", 0x100, 0x30, 0x80);
        report("1bpp", st);
        check(dut->dbg_shim_bpp_shift == 3, "shim decodes 1bpp = 8 px/byte");
        check(dut->dbg_shim_bytes_per_px == 0, "shim reports sub-byte depth");
        check(dut->dbg_shim_depth_supported == 1, "shim reports 1bpp supported");
        check(dut->dbg_ps_in_range == 1,
              "placement_sync range gate admits 640x480x1bpp in 2 MiB");
        check(dut->dbg_committed_stride == 1024,
              "placement_sync commits stride 1024 at 1bpp");
        check(dut->dbg_fetch_direct == 0, "linebuf latched indexed mode at 1bpp");
        check(dut->dbg_fetch_frame_in_mem == 1,
              "linebuf frame-in-memory gate admits 640x480x1bpp");
        // 4096 + 1024*479 + 79 = 494,671 -- the exact address the live board
        // froze vram_rd_addr at.
        check(dut->dbg_req_addr_limit == 494671u,
              "req_addr_limit is the live board's 494671 (0x78c4f)");
        check(st.nonblack > (1280L * 960L) / 2,
              "1bpp active window is more than half non-black");
        check(st.other == 0, "1bpp emits only palette entries 0 and 1");
        check(st.exact_white == 3L * (1280L * 960L) / 4,
              "1bpp lit pixel count is exactly 3/4 of the active window");

        // THE REGRESSION: the fetch walk must re-arm on every frame.
        printf("  -- 1bpp x 4 consecutive frames --\n");
        std::vector<FrameStats> fr = measure_frames("1bpp-multi", 4);
        long lo = fr[0].fetch_reqs, hi = fr[0].fetch_reqs;
        long minblack = fr[0].nonblack;
        for (const FrameStats& s : fr) {
            if (s.fetch_reqs < lo) lo = s.fetch_reqs;
            if (s.fetch_reqs > hi) hi = s.fetch_reqs;
            if (s.nonblack < minblack) minblack = s.nonblack;
        }
        printf("  [1bpp-multi] fetch_reqs min=%ld max=%ld\n", lo, hi);
        // 640x480x1bpp = 480 rows x 80 bytes = 38,400 byte requests/frame.
        check(lo > 38000L,
              "1bpp: every one of 4 frames issues a full frame of fetches "
              "(the fetch walk re-arms)");
        check(minblack > (1280L * 960L) / 2,
              "1bpp: every one of 4 frames is more than half non-black");
        g_lit_value = 0x00FFFFFFu;
    }

    }

    // ── Scenario 6: memory-latency sweep at 1bpp AND 8bpp ─────────────
    // The production fetch port is fb_reader.v (pclk<->vram_clk CDC, ~31+
    // cycles round trip, request-FIFO backpressure), not a 1-cycle RAM.
    printf("\n-- scenario 6: memory-latency sweep (1bpp and 8bpp)\n");
    if (want(6)) {
    scenarios_run++;
    {
        const int lats[] = {96};
        for (int bp = 0; bp < 2; bp++) {
            for (int depth = 0; depth < 1; depth++) {
                const bool one_bpp = (depth == 0);
                for (int li = 0; li < (int)(sizeof(lats)/sizeof(lats[0])); li++) {
                    dut->mem_latency = lats[li];
                    dut->mem_backpressure = bp;
                    g_lit_value = one_bpp ? 0x00010101u : 0x00FFFFFFu;
                    // Re-program from scratch each time so the scanner starts
                    // from a clean placement commit.
                    reset_and_program();
                    axi_write(REG_STRIDE, 0x100);
                    axi_write(REG_CONFIG, 0x30);
                    axi_write(REG_PCBR, one_bpp ? 0x80 : 0x98);
                    for (int f = 0; f < 2; f++) run_frame(false);
                    long lo = -1, minnb = -1;
                    for (int f = 0; f < 2; f++) {
                        FrameStats s = run_frame(true);
                        if (lo < 0 || s.fetch_reqs < lo) lo = s.fetch_reqs;
                        if (minnb < 0 || s.nonblack < minnb) minnb = s.nonblack;
                    }
                    long expect = one_bpp ? 38400L : 307200L;
                    bool ok = (lo >= expect) && (minnb > (1280L*960L)/2);
                    printf("  %-4s lat=%-4d bp=%d : min fetch/frame=%-7ld "
                           "(expect %ld)  min nonblack=%-8ld  %s\n",
                           one_bpp ? "1bpp" : "8bpp", lats[li], bp, lo, expect,
                           minnb, ok ? "ok" : "WEDGED");
                    if (!ok) {
                        printf("      state: prefetch_active=%u can_request=%u "
                               "in_resync=%u placement_changed=%u "
                               "outstanding_empty=%u req=(y%u,x%u) rsp_y=%u "
                               "fb_rd_addr=%u fb_rd_valid=%u\n",
                               (unsigned)dut->dbg_prefetch_active,
                               (unsigned)dut->dbg_can_request,
                               (unsigned)dut->dbg_in_resync,
                               (unsigned)dut->dbg_placement_changed,
                               (unsigned)dut->dbg_outstanding_empty,
                               (unsigned)dut->dbg_req_y, (unsigned)dut->dbg_req_x,
                               (unsigned)dut->dbg_rsp_y,
                               (unsigned)dut->dbg_fb_rd_addr,
                               (unsigned)dut->dbg_fb_rd_valid);
                    }
                    check(ok, std::string(one_bpp ? "1bpp" : "8bpp") +
                              " lat=" + std::to_string(lats[li]) +
                              " bp=" + std::to_string(bp) +
                              ": fetch re-arms every frame and the frame paints");
                }
            }
        }
        dut->mem_latency = 1;
        dut->mem_backpressure = 0;
        g_lit_value = 0x00FFFFFFu;
    }

    }

    // ── Scenario 7: through the REAL fb_reader CDC bridge ─────────────
    // Production wiring: linebuf_scanout -> fb_reader (pclk<->vram_clk,
    // gray-pointer async FIFOs, credit throttle) -> VRAM.  This is the only
    // part of the fetch path where a response can actually be LOST, and
    // linebuf's re-arm gate is request-count == response-count, so a single
    // lost response wedges the scanner permanently.
    printf("\n-- scenario 7: fetch through the real fb_reader CDC\n");
    if (want(7)) {
    scenarios_run++;
    {
        const int divs[] = {3};
        const int lats[] = {40};
        dut->mem_via_fb_reader = 1;
        for (int di = 0; di < (int)(sizeof(divs)/sizeof(divs[0])); di++) {
            for (int li = 0; li < (int)(sizeof(lats)/sizeof(lats[0])); li++) {
                for (int depth = 0; depth < 2; depth++) {
                    const bool one_bpp = (depth == 0);
                    g_vclk_div = divs[di];
                    dut->mem_latency = lats[li];
                    dut->mem_backpressure = 0;
                    g_lit_value = one_bpp ? 0x00010101u : 0x00FFFFFFu;
                    reset_and_program();
                    axi_write(REG_STRIDE, 0x100);
                    axi_write(REG_CONFIG, 0x30);
                    axi_write(REG_PCBR, one_bpp ? 0x80 : 0x98);
                    for (int f = 0; f < 3; f++) run_frame(false);
                    long lo = -1, minnb = -1;
                    for (int f = 0; f < 3; f++) {
                        FrameStats st2 = run_frame(true);
                        if (lo < 0 || st2.fetch_reqs < lo) lo = st2.fetch_reqs;
                        if (minnb < 0 || st2.nonblack < minnb) minnb = st2.nonblack;
                    }
                    long expect = one_bpp ? 38400L : 307200L;
                    bool ok = (lo >= expect) && (minnb > (1280L*960L)/2);
                    dump_gates(one_bpp ? "1bpp-cdc" : "8bpp-cdc");
                    printf("  %-4s vclk=pclk/%d lat=%-3d : min fetch/frame=%-7ld "
                           "(expect %ld) min nonblack=%-8ld fbr[req=%u rsp=%u "
                           "miss=%u ovf=%u] %s\n",
                           one_bpp ? "1bpp" : "8bpp", divs[di], lats[li], lo,
                           expect, minnb,
                           (unsigned)dut->dbg_fbr_req_count,
                           (unsigned)dut->dbg_fbr_rsp_count,
                           (unsigned)dut->dbg_fbr_miss_count,
                           (unsigned)dut->dbg_fbr_underflow_sticky,
                           ok ? "ok" : "WEDGED");
                    if (!ok) {
                        printf("      state: prefetch_active=%u can_request=%u "
                               "in_resync=%u placement_changed=%u "
                               "outstanding_empty=%u req=(y%u,x%u) rsp=(y%u) "
                               "fb_rd_addr=%u fb_rd_valid=%u credits=%u\n",
                               (unsigned)dut->dbg_prefetch_active,
                               (unsigned)dut->dbg_can_request,
                               (unsigned)dut->dbg_in_resync,
                               (unsigned)dut->dbg_placement_changed,
                               (unsigned)dut->dbg_outstanding_empty,
                               (unsigned)dut->dbg_req_y, (unsigned)dut->dbg_req_x,
                               (unsigned)dut->dbg_rsp_y,
                               (unsigned)dut->dbg_fb_rd_addr,
                               (unsigned)dut->dbg_fb_rd_valid,
                               (unsigned)dut->dbg_credits);
                    }
                    check(ok, std::string(one_bpp ? "1bpp" : "8bpp") +
                              " via fb_reader vclk=pclk/" + std::to_string(divs[di]) +
                              " lat=" + std::to_string(lats[li]) +
                              ": fetch re-arms every frame and the frame paints");
                }
            }
        }
        dut->mem_via_fb_reader = 0;
        g_vclk_div = 1;
        dut->mem_latency = 1;
        g_lit_value = 0x00FFFFFFu;
    }

    }

    // == Scenario 8: ONE lost fetch response must not kill the display ==
    // linebuf_scanout re-arms its per-frame fetch walk only when
    // `outstanding_empty` (request counters == response counters) is true.
    // Nothing bounds how long that can stay false: a single response that
    // never comes back makes it false FOREVER, so the walk runs to the end
    // of the current frame, clears prefetch_active, and is never re-armed.
    // The observable end state is exactly what the live board showed at
    // bitstream 0x881f9bfd -- vram_rd_addr frozen at
    // 4096 + 1024*479 + 79 = 494671 (0x78c4f, the last pixel of a 1bpp
    // frame), zero further VRAM reads, and one thin band of the last 64
    // fetched source rows still painted out of the stale line buffers.
    if (want(8)) {
    printf("\n-- scenario 8: recovery from a single lost fetch response (1bpp)\n");
    scenarios_run++;
    {
        reset_and_program();
        g_lit_value = 0x00010101u;
        axi_write(REG_STRIDE, 0x100);
        axi_write(REG_CONFIG, 0x30);
        axi_write(REG_PCBR, 0x80);
        for (int f = 0; f < 3; f++) run_frame(false);
        FrameStats pre = run_frame(true);
        printf("  before drop: fetch/frame=%ld nonblack=%ld\n",
               pre.fetch_reqs, pre.nonblack);
        check(pre.fetch_reqs >= 38400L && pre.nonblack > (1280L*960L)/2,
              "1bpp baseline before the injected drop");

        // Swallow exactly one response, mid-frame.
        for (int i = 0; i < 40000; i++) tick();
        dut->mem_drop_rsp = 1; tick(); dut->mem_drop_rsp = 0;
        for (int i = 0; i < 200; i++) tick();
        long lo = -1, minnb = -1;
        for (int f = 0; f < 6; f++) {
            FrameStats st3 = run_frame(true);
            printf("  recovery frame %d: fetch=%-7ld nonblack=%-8ld "
                   "prefetch_active=%u can_request=%u in_resync=%u "
                   "outstanding_empty=%u req=(y%u,x%u) rsp_y=%u addr=%u oob=%u\n",
                   f, st3.fetch_reqs, st3.nonblack,
                   (unsigned)dut->dbg_prefetch_active,
                   (unsigned)dut->dbg_can_request,
                   (unsigned)dut->dbg_in_resync,
                   (unsigned)dut->dbg_outstanding_empty,
                   (unsigned)dut->dbg_req_y, (unsigned)dut->dbg_req_x,
                   (unsigned)dut->dbg_rsp_y, (unsigned)dut->dbg_fb_rd_addr,
                   (unsigned)dut->dbg_req_addr_oob);
            // frames 0..2 may be partially painted while the scanner resyncs
            if (f >= 3) {
                if (lo < 0 || st3.fetch_reqs < lo) lo = st3.fetch_reqs;
                if (minnb < 0 || st3.nonblack < minnb) minnb = st3.nonblack;
            }
        }
        // Checked here, not at injection time: the fetcher is idle in
        // vertical blanking right after it finishes a walk, so the armed
        // drop is not consumed until the next frame's requests start.
        printf("  injected drops=%u\n", (unsigned)dut->dbg_mem_drops);
        check(dut->dbg_mem_drops == 1, "exactly one response was dropped");
        check(lo >= 38400L,
              "the fetch walk re-arms again within 3 frames of a lost response");
        check(minnb > (1280L*960L)/2,
              "the active window is repainted again after a lost response");
        g_lit_value = 0x00FFFFFFu;
    }
    }

    // == Scenario 9: MODE TRANSITION -- 640x480 -> 832x624, 8bpp ========
    //
    // Every scenario above holds the DAFB placement constant for its whole
    // run: program it once, then measure.  A real Monitors-control-panel
    // resolution change does not work that way -- it is SEVEN separate CPU
    // register writes (Swatch HAL, HFP, VAL, VFP, then STRIDE, CONFIG, PCBR),
    // and the scanner has to survive every partial state in between.  This
    // scenario walks them one at a time with a full frame in between, so each
    // intermediate configuration is actually displayed.
    //
    // WHY THIS COMBINATION.  Every register value below was READ OFF THE LIVE
    // BOARD; none is invented:
    //     640x480 (mon-sense 0x06): HAL=0x098 HFP=0x318 PCBR=0x80 -> clockdiv 1
    //     832x624 (mon-sense 0x6D): HAL=0x08B HFP=0x22B PCBR=0xA0 -> clockdiv 2
    // (rtl/mac/video.v:596-604 records both, with the JTAG read they came
    // from.)  The clockdiv term is the reason the two Swatch H values are so
    // different -- 832x624 counts in 2-pixel units -- and it is why hres is
    // DERIVED here rather than asserted: the tb writes the registers and reads
    // dbg_shim_hres back out of video.v.
    //
    // Row pitch: 640x480 uses STRIDE reg 0x100 -> 1024 bytes (the live-board
    // value, scenario 1/5).  832x624 uses STRIDE reg 0xD0 -> 832 bytes; that
    // is the pitch the 832x624 mode was running at on the board immediately
    // before the "Millions" JTAG force (`w 0xF9800008 0x340` took it from
    // 832 to 3328).
    //
    // THE INTERMEDIATE STATE THAT BREAKS.  Once the new 832-byte pitch has
    // committed and the PCBR clockdiv has gone to 2 but the Swatch H
    // registers still hold the 640x480 values, video.v derives
    //     hres = (0x318 - 0x098) << 1 = 1280
    // -- above the elaborated SRC_W, so the scanner measures a full 1024-byte
    // row and needs stride > 1023.  832 is not.  scanout_fetch's
    // fetch_stride_sane goes false, can_request goes false, and the frame
    // fetches NOTHING.
    //
    // Both endpoints render.  Only the seam does not.
    if (want(9)) {
    printf("\n-- scenario 9: mode transition 640x480x8 -> 832x624x8 "
           "(8 register writes, one per frame)\n");
    scenarios_run++;
    {
        struct Reg { uint32_t off; uint32_t val; const char* name; };
        // The seven writes a 640x480 -> 832x624 mode set performs.
        // PCBR 0xB8: AC842 bits[6:5] = 01 -> clockdiv 2 (the value the live
        // board reports for 832x624, 0xA0, carries the SAME clockdiv field),
        // bits[4:2] = 110 -> 0x18 -> 8bpp.  So this is the live-board 832x624
        // clockdiv with the live-board 8bpp depth code -- neither invented.
        const Reg mode832[] = {
            { REG_STRIDE,  0x0D0, "STRIDE(832)" },
            { REG_PCBR,    0x0B8, "PCBR(clockdiv 2, 8bpp)" },
            { REG_HAL,     0x08B, "HAL"    },
            { REG_HFP,     0x22B, "HFP"    },
            { REG_VAL,     0x030, "VAL"    },
            { REG_VFP,     0x510, "VFP"    },
            { REG_CONFIG,  0x030, "CONFIG" },
            // ── The closing PCBR write, added 2026-08-20 ─────────────
            // Every mode set ends by rewriting the AC842 PCBR, and that
            // write is what makes the new geometry take effect: MAME's
            // `recalc_mode()` is reachable from EXACTLY ONE call site,
            // `ramdac_w` case 0x20 (dafb.cpp:816).  `swatch_w` and
            // `dafb_w` never recompute, so m_hres/m_vres are a snapshot
            // taken at the PCBR write.
            //
            // This is not a supposition.  A MAME 0.285 write tap over a
            // full macqd700 ROM boot, run once per entry of MAME's own
            // monitor_config list, shows the PCBR at +0x220 as the LAST
            // write of the mode-set block in all ELEVEN captures (the
            // traces are checked in as tb/tb_mode_decode_traces.h).
            //
            // Without it, `ord_a` below asks the DUT to settle at 832x624
            // having written the PCBR while the Swatch still held the
            // 640x480 pair and never written it again.  MAME settles that
            // sequence at 1280x480 and holds it, so the old expectation
            // was asserting something the golden model contradicts.  The
            // seven measurement steps are untouched; this only closes the
            // mode set the way the hardware does.
            { REG_PCBR,    0x0B8, "PCBR(mode-set close)" },
        };
        const int NREG = 8;

        // Write orders.  The documented Mac OS order is stride-before-depth
        // with the Swatch rewritten in between; the second permutation puts
        // the Swatch first so the seam is crossed from both sides.  Only two
        // are run here because each costs ~24 full 1920x1080 frames of
        // simulation -- the exhaustive order x phase sweep (90 runs per
        // transition) lives in tb-scanout-placement-sync, which is seconds
        // rather than minutes.  This rig's job is to show the same defect as
        // VISIBLE corruption through the real chain.
        const int ord_a[NREG] = { 0, 1, 2, 3, 4, 5, 6, 7 };  // stride, pcbr, swatch
        const int ord_b[NREG] = { 2, 3, 4, 5, 0, 1, 6, 7 };  // swatch first
        struct Ord { const char* name; const int* o; };
        const Ord orders[] = {
            { "stride,pcbr,swatch", ord_a },
            { "swatch,stride,pcbr", ord_b },
        };

        // 8bpp: the depth a colour Mac OS desktop runs at, and the one with
        // the least pitch headroom (one byte per pixel, 832-byte pitch for an
        // 832-pixel row).
        int total_black = 0, total_frames = 0;
        int worst_black = 0;
        std::string worst_detail;

        for (const Ord& od : orders) {
            reset_and_program();                 // 640x480 Swatch + base 4096
            g_lit_value = 0x00FFFFFFu;
            axi_write(REG_STRIDE, 0x100);        // 1024-byte pitch
            axi_write(REG_CONFIG, 0x030);
            axi_write(REG_PCBR,   0x098);        // 8bpp, clockdiv 1
            for (int f = 0; f < 4; f++) run_frame(false);

            FrameStats base_st = run_frame(true);
            check(base_st.fetch_reqs > 300000L && base_st.nonblack > 100000L,
                  std::string("[") + od.name +
                  "] 640x480x8 baseline paints before the mode set");

            int black_here = 0;
            std::string detail;
            for (int i = 0; i < NREG; i++) {
                const Reg& rg = mode832[od.o[i]];
                axi_write(rg.off, rg.val);
                // One settle frame for the CDC + frame-boundary commit, then
                // one measured frame in that state.
                run_frame(false);
                FrameStats st = run_frame(true);
                total_frames++;
                bool blank = (st.fetch_reqs == 0);
                printf("  [%s] step %d %-24s shim hres=%-5u vres=%-4u | "
                       "committed hres=%-5u stride=%-5u bppx=%u | "
                       "stride_sane=%u can_request=%u | fetch=%-7ld "
                       "nonblack=%-8ld%s\n",
                       od.name, i, rg.name,
                       (unsigned)dut->dbg_shim_hres, (unsigned)dut->dbg_shim_vres,
                       (unsigned)dut->dbg_committed_hres,
                       (unsigned)dut->dbg_committed_stride,
                       (unsigned)dut->dbg_committed_bytes_per_px,
                       (unsigned)dut->dbg_fetch_stride_sane,
                       (unsigned)dut->dbg_can_request,
                       st.fetch_reqs, st.nonblack, blank ? "  <-- BLANK" : "");
                if (blank) {
                    black_here++;
                    total_black++;
                    char buf[320];
                    snprintf(buf, sizeof buf,
                             "after %s: shim hres=%u vres=%u stride=%u | "
                             "committed hres=%u vres=%u stride=%u bppx=%u | "
                             "fetch_stride_sane=%u can_request=%u "
                             "row_last_off=%u | fetch_reqs=%ld nonblack=%ld",
                             rg.name,
                             (unsigned)dut->dbg_shim_hres,
                             (unsigned)dut->dbg_shim_vres,
                             (unsigned)dut->dbg_shim_stride,
                             (unsigned)dut->dbg_committed_hres,
                             (unsigned)dut->dbg_committed_vres,
                             (unsigned)dut->dbg_committed_stride,
                             (unsigned)dut->dbg_committed_bytes_per_px,
                             (unsigned)dut->dbg_fetch_stride_sane,
                             (unsigned)dut->dbg_can_request,
                             (unsigned)dut->dbg_row_last_off,
                             st.fetch_reqs, st.nonblack);
                    if (detail.empty()) detail = buf;
                    printf("  [%s] BLANK %s\n", od.name, buf);
                }
            }
            if (black_here > worst_black) {
                worst_black = black_here;
                worst_detail = std::string(od.name) + " -- " + detail;
            }

            // Settle and require the destination mode to render.
            for (int f = 0; f < 4; f++) run_frame(false);
            FrameStats fin = run_frame(true);
            printf("  [%s] settled: shim hres=%u vres=%u | committed hres=%u "
                   "vres=%u stride=%u | fetch=%ld nonblack=%ld\n",
                   od.name,
                   (unsigned)dut->dbg_shim_hres, (unsigned)dut->dbg_shim_vres,
                   (unsigned)dut->dbg_committed_hres,
                   (unsigned)dut->dbg_committed_vres,
                   (unsigned)dut->dbg_committed_stride,
                   fin.fetch_reqs, fin.nonblack);
            check(dut->dbg_shim_hres == 832 && dut->dbg_shim_vres == 624,
                  std::string("[") + od.name +
                  "] video.v derives 832x624 from the written registers "
                  "(hres is observed, not assumed)");
            check(fin.fetch_reqs > 400000L && fin.nonblack > 100000L,
                  std::string("[") + od.name +
                  "] 832x624x8 destination mode paints after the mode set");
        }

        printf("  transition summary: %d of %d intermediate frames fetched "
               "NOTHING\n", total_black, total_frames);
        if (!worst_detail.empty())
            printf("  worst: %s\n", worst_detail.c_str());
        check(total_black == 0,
              "no intermediate frame of a 640x480 -> 832x624 mode set is "
              "blank: scanout_placement_sync must never commit a geometry "
              "its retained placement cannot render");
    }
    }

    int expected = 0;
    for (int i = 1; i <= 9; i++) if (want(i)) expected++;
    printf("\nscenarios run: %d (expected %d)\n", scenarios_run, expected);
    if (scenarios_run != expected) {
        printf("FAIL: scenario RUN count does not match the expected list\n");
        failures++;
    }

    dut->final();
    delete dut;

    if (failures) {
        printf("\ntb_dafb_24bpp_capacity: %d FAILURE(S)\n", failures);
        return 1;
    }
    printf("\ntb_dafb_24bpp_capacity: PASS\n");
    return 0;
}
