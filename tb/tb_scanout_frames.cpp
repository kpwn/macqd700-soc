// tb_scanout_frames.cpp -- multi-frame scanout gate at shipping geometry.
// ---------------------------------------------------------------------------
// See tb_scanout_frames.v's header for WHY this rig exists.  This file is the
// harness: it models the fetch port (a byte-addressed request/response stream
// returning the 4-byte group at the requested address, exactly the contract
// vram.v and scanout_ddr_reader.v present), paints row-varying content into
// the model framebuffer, walks several consecutive 1920x1080 frames, and
// asserts per frame that every active display line carries the pixels of the
// source row it is supposed to show.
//
// The three scenario families cover all three shipping depths:
//     1bpp   (bpp_shift=3, bytes_per_px=1)   640x480 @ 2x
//     8bpp   (bpp_shift=0, bytes_per_px=1)   640x480 @ 2x  and 1024x768 @ 1x
//     24bpp  (bpp_shift=0, bytes_per_px=4)   640x480 @ 2x, wide fetch
//
// plus a fault-injection scenario that drops one fetch response mid-frame --
// the fault af8fe56's re-arm watchdog exists to survive -- and requires the
// scanner to be back to full frames afterwards.
// ---------------------------------------------------------------------------
#include <verilated.h>
#include "Vtb_scanout_frames.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <deque>
#include <set>
#include <string>
#include <vector>
#include <cstdlib>

// ── Shipping geometry (must match tb_scanout_frames.v + vtg.v defaults) ──
static const int DST_W   = 1920;
static const int DST_H   = 1080;
static const int H_TOTAL = 1920 + 88 + 44 + 148;   // 2200
static const int V_TOTAL = 1080 + 4 + 5 + 36;      // 1125
static const uint32_t FB_BYTES = 0x0020'0000u;     // 2 MiB scanner aperture
static const int LINE_COUNT = 64;                  // 1 << LINE_COUNT_LOG2

static Vtb_scanout_frames* dut = nullptr;
static int pass_count = 0;
static int fail_count = 0;

// ── Boot-splash reference ───────────────────────────────────────────────
// An INDEPENDENT copy of the 32x32 logo, so the RTL ROM and the expectation
// cannot both be wrong in the same way.  Blanking the RTL ROM (in
// scanout_display.v) must turn the splash checks RED -- that is the
// negative control for this whole scenario family.
//
// Bit x (LSB-first) is COLUMN x: bit 0 is the LEFTMOST pixel.
static const uint32_t SPLASH_ROM[32] = {
    0x00000000u, 0x00000a00u, 0x00000400u, 0x00005540u,
    0x00007fc0u, 0x00003f80u, 0x00003f80u, 0x00001f00u,
    0x00001f00u, 0x00001f00u, 0x00003f80u, 0x0000ffe0u,
    0x00003f80u, 0x00003f80u, 0x00003f83u, 0x00103f9fu,
    0x18103ffbu, 0x0e3fffd5u, 0x1beabfabu, 0x480d7fd5u,
    0xf80abfabu, 0x480d7fd5u, 0x1beabfabu, 0x0e3fffd5u,
    0x18107ffbu, 0x00107fdfu, 0x00007fc3u, 0x0000ffe0u,
    0x0000ffe0u, 0x0000ffe0u, 0x0001fff0u, 0x0001fff0u
};
// Must match scanout_display.v's SPLASH_SCALE at this instantiation.
static const int SPLASH_SCALE = 4;
static const int SPLASH_PIX   = 32 * SPLASH_SCALE;
static const uint32_t SPLASH_FG = 0xFFFFFFu;
static const uint32_t SPLASH_BG = 0x000000u;

// Set only for SPLASH scenarios: capture the whole 1920x1080 output image so
// individual coordinates can be asserted (and a PPM eyeballed).
static bool capture_img = false;

static void check(bool ok, const std::string& what) {
    if (ok) { pass_count++; printf("  [PASS] %s\n", what.c_str()); }
    else    { fail_count++; printf("  [FAIL] %s\n", what.c_str()); }
}

// ── Model framebuffer + fetch port ──────────────────────────────────────
static std::vector<uint8_t> fbmem;

struct InFlight { uint32_t addr; long long due; };
static std::deque<InFlight> inflight;
static long long model_tick      = 0;
static int       model_latency   = 9;      // pclk cycles, request -> response
static size_t    model_max_depth = 32;     // ready throttle
// Fault injection: drop the Nth accepted request's response (never answer it).
static long long drop_request_idx = -1;
// Recurring fault: drop the response to every Nth accepted request (0 = off).
// Models a fetch port that keeps losing responses -- which is what turns a
// one-frame glitch into the persistent band seen on the live board.
static long long drop_period      = 0;
// Armed one-shot: the NEXT accepted request's response is never made.  Armed
// at a named (frame, display line) so the drop lands in a KNOWN frame -- which
// is what lets a test assert one-frame recovery rather than "eventually".
static bool      drop_armed       = false;
static long long accepted_total   = 0;
static long long dropped_total    = 0;

static bool     m_ready = false;
static bool     m_valid = false;
static uint32_t m_data  = 0;

static void model_reset() {
    inflight.clear();
    model_tick     = 0;
    accepted_total = 0;
    dropped_total  = 0;
    m_ready = true;
    m_valid = false;
    m_data  = 0;
}

static uint32_t fetch_group(uint32_t a) {
    uint32_t b0 = (a + 0 < FB_BYTES) ? fbmem[a + 0] : 0;
    uint32_t b1 = (a + 1 < FB_BYTES) ? fbmem[a + 1] : 0;
    uint32_t b2 = (a + 2 < FB_BYTES) ? fbmem[a + 2] : 0;
    uint32_t b3 = (a + 3 < FB_BYTES) ? fbmem[a + 3] : 0;
    return (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
}

// ── Per-frame accounting ────────────────────────────────────────────────
struct LineStat {
    // EVERY de_out pixel of the line, black included.  Doubles as the
    // horizontal index for image capture: counting de_out pixels within the
    // line is exact regardless of the output pipeline's 5-cycle lag, which
    // guessing `hcount - 5` would not be.
    int      n     = 0;
    int      lit   = 0;        // output pixels with de_out high (any colour)
    uint32_t first = 0xFFFFFFFFu;
    bool     mixed = false;
    // FNV-1a over EVERY de_out pixel of the line, in order, black included.
    // `first`/`mixed` only describe a line whose colour is uniform, which is
    // all these scenarios paint -- so on their own they cannot see a
    // WITHIN-ROW displacement at all.  The hash can, and it is what the
    // frame-to-frame identity check below compares.
    uint64_t hash  = 1469598103934665603ull;
};
struct FrameStat {
    std::vector<LineStat> line;
    long long requests      = 0;
    int       max_req_y     = -1;
    int       rows_requested = 0;
    std::set<int> row_set;
    std::set<uint32_t> colours;
    uint32_t  last_rd_addr  = 0;
    // Sampled once, deep inside the visible frame (display line 700).
    //   awaiting_mid  -- awaiting_frame_start, which must be 0 there: the
    //                    frame has demonstrably started.
    //   req_y_mid / y_src_mid -- the fetch walk's position vs the display's.
    //                    A healthy fetcher is AHEAD (req_y > y_src, up to a
    //                    full ring); a parked one has been overtaken.  This
    //                    is the park criterion: it names an observable
    //                    relationship between two counters, not a guess at
    //                    which gate stopped the walk.
    int       credits_mid   = -1;
    int       req_y_mid     = -1;
    int       y_src_mid     = -1;
    int       state_mid     = -1;
    uint32_t  park_addr     = 0;
    // Full output image, DST_W x DST_H, only allocated for SPLASH scenarios.
    std::vector<uint32_t> img;
    FrameStat() : line(V_TOTAL) {
        if (capture_img) img.assign((size_t)DST_W * DST_H, 0);
    }
};

static std::vector<FrameStat> frames;
static FrameStat cur;

static void tick_one() {
    // Inputs for this edge are already staged in dut->fb_rd_*; capture the
    // slave-visible request BEFORE the edge so the handshake is evaluated on
    // the same values both sides saw.
    bool     en_pre    = dut->fb_rd_en;
    uint32_t addr_pre  = dut->fb_rd_addr;
    bool     ready_pre = m_ready;
    bool     valid_pre = m_valid;

    dut->pclk = 1; dut->eval();
    dut->pclk = 0; dut->eval();
    model_tick++;

    // Post-edge sampling of the scanner's own state.
    if (valid_pre && !inflight.empty()) inflight.pop_front();
    if (en_pre && ready_pre) {
        accepted_total++;
        cur.requests++;
        bool drop = (accepted_total - 1 == drop_request_idx)
                 || (drop_period > 0 && (accepted_total % drop_period) == 0)
                 || drop_armed;
        if (drop_armed) drop_armed = false;
        if (drop) {
            dropped_total++;                    // response is simply never made
        } else {
            inflight.push_back({addr_pre, model_tick + model_latency});
        }
        cur.last_rd_addr = addr_pre;
        int ry = (int)dut->dbg_req_y;
        cur.row_set.insert(ry);
        if (ry > cur.max_req_y) cur.max_req_y = ry;
    }

    // Next-cycle model outputs.
    m_ready = inflight.size() < model_max_depth;
    m_valid = (!inflight.empty()) && (inflight.front().due <= model_tick);
    m_data  = m_valid ? fetch_group(inflight.front().addr) : 0;
    dut->fb_rd_ready = m_ready;
    dut->fb_rd_valid = m_valid;
    dut->fb_rd_data  = m_data;

    // Output-pixel capture.  de_out lags the VTG by the scanner's 5-stage
    // output pipeline, but the horizontal border is 320 px wide at every
    // scenario here, so a pixel never spills into the next display line and
    // bucketing by the live vcount is exact for every non-blank pixel.
    if (dut->de_out) {
        int v = dut->vcount;
        if (v >= 0 && v < V_TOTAL) {
            uint32_t c = dut->rgb & 0xFFFFFFu;
            LineStat& ls = cur.line[v];
            int x = ls.n++;
            if (capture_img && !cur.img.empty() && v < DST_H && x < DST_W)
                cur.img[(size_t)v * DST_W + x] = c;
            ls.hash = (ls.hash ^ (uint64_t)c) * 1099511628211ull;
            if (c != 0) {
                if (ls.lit == 0) ls.first = c;
                else if (ls.first != c) ls.mixed = true;
                ls.lit++;
                cur.colours.insert(c);
            }
        }
    }

    if (getenv("SCAN_TRACE") && dut->hcount == 0 && frames.size() == 1) {
        int v = dut->vcount;
        if (v==0||v==10||v==30||v==58||v==60||v==62||v==70||v==100||v==200||
            v==400||v==700||v==890||v==1000)
            printf("    TRACE v=%4d state=%d cred=%3d req_y=%3d rsp_y=%3d "
                   "y_src=%3d stale=%d outst=%d\n", v, (int)dut->dbg_state,
                   (int)dut->dbg_credits, (int)dut->dbg_req_y,
                   (int)dut->dbg_rsp_y, (int)dut->dbg_y_src,
                   (int)dut->dbg_stale_count, (int)dut->dbg_outstanding);
    }
    // Mid-visible-frame probes: sample once per frame, deep inside the
    // active region, where a healthy fetcher must NOT be parked.
    if (dut->vcount == 700 && dut->hcount == 0) {
        cur.credits_mid  = (int)dut->dbg_credits;
        cur.req_y_mid    = (int)dut->dbg_req_y;
        cur.y_src_mid    = (int)dut->dbg_y_src;
        cur.state_mid    = (int)dut->dbg_state;
        cur.park_addr    = cur.last_rd_addr;
    }
}

// ── Scenario description ────────────────────────────────────────────────
struct Scenario {
    const char* name;
    int      hres, vres;
    bool     use_2x;
    int      bpp_shift;      // 0 = 8bpp / 24bpp, 3 = 1bpp
    int      bytes_per_px;   // 1 = indexed, 4 = 24bpp direct
    uint32_t base;
    uint32_t stride;
    int      frames;
    long long drop_at;       // -1 = no one-shot fault injection (by req index)
    long long drop_period;   // 0  = no recurring fault injection
    // One-shot drop armed at a NAMED (frame, display line), so the corrupted
    // frame is known by number and the frames after it can be gated STRICT.
    int      drop_at_frame;  // -1 = off
    int      drop_at_vcount;
    // Fetch-port model shape.  A deep/slow port leaves the walk still
    // draining at start-of-frame, which is exactly the condition the stale-
    // response discard window exists for.
    int      latency;
    int      max_depth;
    // From this collected frame on, the port is crippled (latency x100).
    // Used as the POSITIVE CONTROL for line_underflow_sticky: without one, a
    // "no underflow" check cannot tell a healthy scanner from a dead gate.
    int      starve_at_frame;   // -1 = off
    int      first_checked_frame;
    // STRICT   -- every checked frame must be pixel-complete.  The gate for
    //             a fetch port that is behaving.
    // RECOVERY -- the fetch port is being made to lose responses forever, so
    //             individual frames are EXPECTED to be lost while the
    //             desync watchdog notices (its detection latency is two frame
    //             boundaries by construction).  What must still hold is that
    //             the scanner keeps coming back: no frame may be parked, and
    //             a full-height frame must appear at least once in every
    //             RECOVERY_WINDOW consecutive frames.
    // STARVED -- the port is deliberately made unable to keep up; the gate is
    //            that line_underflow_sticky FIRES.  Positive control.
    // SPLASH  -- dafb_live is held LOW, so scanout_display substitutes the
    //            boot splash.  The per-pixel gate is the logo itself; the
    //            fetch-walk gates above still apply UNCHANGED, which is the
    //            proof that the splash is invisible to the credit ring.
    enum Mode { STRICT, RECOVERY, STARVED, SPLASH } mode;
    // Drives the RTL's dafb_live.  1 = normal video (every pre-existing
    // scenario), 0 = boot splash.  Defaulted so the existing positional
    // initialisers below are untouched.
    int      dafb_live = 1;
};
static const int RECOVERY_WINDOW = 3;

static uint32_t clut_entry(int i) {
    // Injective over 0..255 and never black, so "this line is blank" and
    // "this line shows palette entry 0" are always distinguishable.
    return (uint32_t)(((i & 0xFF) << 16) | (((255 - i) & 0xFF) << 8)
                      | (((i * 7 + 0x11) & 0xFF)));
}

// The colour an active display line MUST show for source row y.
static uint32_t expected_rgb(const Scenario& s, int y) {
    if (s.bytes_per_px == 4) {
        return (uint32_t)(((y & 0xFF) << 16) | ((((y >> 8) & 0x7) | 0x40) << 8)
                          | 0xA5);
    }
    if (s.bpp_shift == 3) {
        // 1bpp: alternate the whole source row between palette index 1 and 0
        // every source row -- the finest row variation two palette entries
        // can express, and the thing a parked fetcher cannot reproduce.
        return clut_entry((y & 1) ? 1 : 0);
    }
    return clut_entry(y & 0xFF);
}

static void paint(const Scenario& s) {
    std::fill(fbmem.begin(), fbmem.end(), 0);
    for (int y = 0; y < s.vres; y++) {
        uint32_t row = s.base + (uint32_t)y * s.stride;
        if (s.bytes_per_px == 4) {
            for (int x = 0; x < s.hres; x++) {
                uint32_t a = row + 4u * (uint32_t)x;
                if (a + 3 >= FB_BYTES) break;
                fbmem[a + 0] = 0x00;                              // pad
                fbmem[a + 1] = (uint8_t)(y & 0xFF);               // R
                fbmem[a + 2] = (uint8_t)(((y >> 8) & 0x7) | 0x40);// G
                fbmem[a + 3] = 0xA5;                              // B
            }
        } else if (s.bpp_shift == 3) {
            int bytes = (s.hres + 7) / 8;
            for (int b = 0; b < bytes; b++) {
                uint32_t a = row + (uint32_t)b;
                if (a >= FB_BYTES) break;
                fbmem[a] = (y & 1) ? 0xFF : 0x00;
            }
        } else {
            for (int x = 0; x < s.hres; x++) {
                uint32_t a = row + (uint32_t)x;
                if (a >= FB_BYTES) break;
                fbmem[a] = (uint8_t)(y & 0xFF);
            }
        }
    }
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

static void run_scenario(const Scenario& s) {
    char fault[128] = "";
    if (s.drop_at_frame >= 0)
        snprintf(fault, sizeof fault,
                 " [1 fetch response dropped at frame %d line %d]",
                 s.drop_at_frame, s.drop_at_vcount);
    else if (s.drop_period > 0)
        snprintf(fault, sizeof fault,
                 " [1 fetch response dropped every %lld requests]",
                 s.drop_period);
    else if (s.drop_at >= 0)
        snprintf(fault, sizeof fault, " [1 fetch response dropped]");
    printf("\n== scenario %s: %dx%d %s, %s, base=0x%x stride=%u, %d frames%s ==\n",
           s.name, s.hres, s.vres, s.use_2x ? "2x" : "1x",
           (s.bytes_per_px == 4) ? "24bpp"
                                 : (s.bpp_shift == 3 ? "1bpp" : "8bpp"),
           s.base, s.stride, s.frames, fault);

    capture_img = (s.mode == Scenario::SPLASH);
    paint(s);
    model_reset();
    model_latency   = s.latency;
    model_max_depth = (size_t)s.max_depth;
    drop_request_idx = -1;      // arm only after the CLUT/reset traffic
    drop_period      = 0;
    drop_armed       = false;

    dut->rst = 1;
    dut->fb_base_px   = s.base;
    dut->fb_stride_px = s.stride;
    dut->bpp_shift    = s.bpp_shift;
    dut->bytes_per_px = s.bytes_per_px;
    dut->hres         = s.hres;
    dut->vres         = s.vres;
    // scale_n is the INTEGER replication factor (place_plan.v's N): N
    // display pixels per source pixel on both axes.  1 or 2 here; the
    // fractional 3:2 rung this used to be able to select no longer exists
    // (docs/video_path_review.md S3).  Every scenario's geometry is one the
    // policy would give this N anyway -- 640x480 -> 2, 1024x768 -> 1.
    dut->scale_n      = s.use_2x ? 2 : 1;
    dut->dafb_live    = s.dafb_live;
    dut->clut_we      = 0;
    dut->fb_rd_ready  = 0;
    dut->fb_rd_valid  = 0;
    dut->fb_rd_data   = 0;
    for (int i = 0; i < 20; i++) tick_one();
    dut->rst = 0;
    for (int i = 0; i < 4; i++) tick_one();

    program_clut();

    accepted_total   = 0;
    drop_request_idx = s.drop_at;
    drop_period      = s.drop_period;

    frames.clear();
    cur = FrameStat();

    int seen_sof = 0;
    bool one_shot_used = false;
    // Run until we have collected s.frames complete frames.  A frame is
    // delimited by the VTG's start-of-frame, so the first partial stretch
    // (between reset release and the first sof) is discarded.
    long long guard = (long long)(s.frames + 3) * H_TOTAL * V_TOTAL + 1000000;
    while ((int)frames.size() < s.frames && guard-- > 0) {
        bool sof_now = (dut->hcount == 0) && (dut->vcount == 0);
        if (sof_now) {
            if (seen_sof > 0) {
                cur.rows_requested = (int)cur.row_set.size();
                frames.push_back(cur);
            }
            seen_sof++;
            cur = FrameStat();
            if (s.starve_at_frame >= 0 &&
                (int)frames.size() >= s.starve_at_frame) {
                model_latency   = s.latency * 100;
                model_max_depth = 2;
            }
        }
        // Arm the one-shot drop at a NAMED point inside a NAMED frame.
        // `seen_sof > 0` is load-bearing: frames.size() is ALSO 0 during the
        // pre-roll stretch between reset release and the first sof, so
        // without it the "drop in frame 0" scenarios armed inside a frame
        // that is then discarded -- the injection fired, the counter proved
        // it fired, and the corrupted frame was never looked at.
        if (!one_shot_used && s.drop_at_frame >= 0 && seen_sof > 0 &&
            (int)frames.size() == s.drop_at_frame &&
            dut->vcount == s.drop_at_vcount && dut->hcount == 0) {
            drop_armed    = true;
            one_shot_used = true;
        }
        tick_one();
    }
    check(guard > 0, std::string(s.name) + ": collected " +
          std::to_string(s.frames) + " frames");
    if (guard <= 0) return;

    // Prove the FAULT WAS ACTUALLY INJECTED.  Without this, a scenario whose
    // arming condition silently never fires reports a clean sweep and looks
    // like proof of robustness -- the single most effective way for a bug to
    // survive a green test suite.
    if (s.drop_at_frame >= 0 || s.drop_at >= 0 || s.drop_period > 0)
        check(dropped_total > 0,
              std::string(s.name) + ": fault injection fired (" +
              std::to_string(dropped_total) + " response(s) dropped)");
    else
        check(dropped_total == 0,
              std::string(s.name) + ": no response dropped (fault-free run)");

    const int active_h = s.vres * (s.use_2x ? 2 : 1);
    const int active_w = s.hres * (s.use_2x ? 2 : 1);
    const int border_y = (DST_H - active_h) / 2;

    for (size_t f = 0; f < frames.size(); f++) {
        const FrameStat& fs = frames[f];
        int lit_lines = 0, first_lit = -1, last_lit = -1;
        long long lit_px = 0;
        for (int v = 0; v < V_TOTAL; v++) {
            if (fs.line[v].lit > 0) {
                lit_lines++;
                if (first_lit < 0) first_lit = v;
                last_lit = v;
                lit_px += fs.line[v].lit;
            }
        }
        bool parked = (fs.req_y_mid >= 0) && (fs.req_y_mid < fs.y_src_mid);
        printf("  frame %zu: reqs=%lld rows_requested=%d max_req_y=%d "
               "lit_lines=%d [%d..%d] lit_px=%lld colours=%zu\n"
               "           @line700: credits=%d state=%d req_y=%d y_src=%d "
               "fb_rd_addr=0x%05x%s\n",
               f, fs.requests, fs.rows_requested, fs.max_req_y,
               lit_lines, first_lit, last_lit, lit_px, fs.colours.size(),
               fs.credits_mid, fs.state_mid, fs.req_y_mid, fs.y_src_mid,
               fs.park_addr,
               parked ? "   <<< FETCH WALK PARKED" : "");
    }

    // ── Per-frame assertions ────────────────────────────────────────
    for (size_t f = (size_t)s.first_checked_frame; f < frames.size(); f++) {
        const FrameStat& fs = frames[f];
        std::string tag = std::string(s.name) + " frame " + std::to_string(f);

        // 1. The fetch walk must cover every source row, every frame.
        //    Asserted under SPLASH too, and deliberately: the splash must
        //    not change the fetch walk by so much as one request.
        if (s.mode == Scenario::STRICT || s.mode == Scenario::SPLASH)
            check(fs.rows_requested >= s.vres,
                  tag + ": fetch walk covered all " + std::to_string(s.vres) +
                  " source rows (got " + std::to_string(fs.rows_requested) + ")");

        // 2. Mid-frame the fetcher must still be AHEAD of the display.  A
        //    walk that parked at the ring wrap gets overtaken and stays
        //    overtaken -- that is the whole failure, expressed as a
        //    relationship between two counters.
        // Not asserted under STARVED: that scenario deliberately makes the
        // port unable to keep up, so being behind is the POINT of it.  The
        // ring-capacity bound below still applies in every mode.
        if (s.mode != Scenario::STARVED)
            check(fs.req_y_mid >= fs.y_src_mid,
                  tag + ": fetch walk ahead of the display at line 700 (req_y=" +
                  std::to_string(fs.req_y_mid) + " y_src=" +
                  std::to_string(fs.y_src_mid) + ")");
        // The fetcher is never more than LINE_COUNT rows ahead: that bound
        // is the credit count, and it is what makes the vblank race (the
        // fetcher pulling a whole frame into the ring during blanking, so
        // only the LAST 64 rows are resident when the display starts)
        // structurally impossible rather than flag-guarded.
        check((fs.req_y_mid - fs.y_src_mid) <= LINE_COUNT,
              tag + ": fetch walk at most LINE_COUNT rows ahead (req_y=" +
              std::to_string(fs.req_y_mid) + " y_src=" +
              std::to_string(fs.y_src_mid) + ")");

        // 3. Every active display line carries its source row's pixels.
        int bad_lines = 0, blank_lines = 0, wrong_colour = 0, mixed_lines = 0;
        int first_bad = -1;
        // Named sub-gates for the TWO states the live board actually showed
        // (see the file header).  They are subsets of `bad_lines`, split out
        // so a regression names the hardware symptom instead of a line count.
        //   TOP_ROWS -- the hardware tear covered roughly the top 80 display
        //               rows; 96 is that rounded up to a comfortable margin.
        const int TOP_ROWS = 96;
        int top_bad = 0, bottom_band_only = 0;
        for (int v = 0; v < V_TOTAL; v++) {
            bool active = (v >= border_y) && (v < border_y + active_h);
            const LineStat& ls = fs.line[v];
            if (!active) {
                if (ls.lit != 0) { bad_lines++; if (first_bad < 0) first_bad = v; }
                continue;
            }
            int y = (v - border_y) / (s.use_2x ? 2 : 1);
            uint32_t want = expected_rgb(s, y);
            if (ls.lit == 0) {
                blank_lines++; bad_lines++;
                if (first_bad < 0) first_bad = v;
            } else {
                if (ls.first != want) {
                    wrong_colour++; bad_lines++;
                    if (first_bad < 0) first_bad = v;
                }
                if (ls.mixed) { mixed_lines++; bad_lines++; }
                if (ls.lit < active_w) {
                    bad_lines++;
                    if (first_bad < 0) first_bad = v;
                }
            }
            if ((v - border_y) < TOP_ROWS &&
                (ls.lit == 0 || ls.first != want || ls.mixed ||
                 ls.lit < active_w))
                top_bad++;
        }
        // "Only the bottom band renders" == the top of the active window is
        // blank while the bottom is lit.  That is the shape a fetcher that
        // raced a whole frame into the ring during vblank produces: only the
        // LAST LINE_COUNT source rows survive.
        {
            int top_lit = 0, bot_lit = 0;
            for (int v = border_y; v < border_y + active_h; v++) {
                if (fs.line[v].lit == 0) continue;
                if ((v - border_y) < active_h / 2) top_lit++; else bot_lit++;
            }
            if (top_lit == 0 && bot_lit > 0) bottom_band_only = 1;
        }
        if (bad_lines) {
            printf("           blank=%d wrong_colour=%d mixed=%d first_bad_line=%d",
                   blank_lines, wrong_colour, mixed_lines, first_bad);
            if (first_bad >= 0) {
                int y = (first_bad - border_y) / (s.use_2x ? 2 : 1);
                printf(" (src row %d: want %06x got %06x lit=%d)",
                       y, expected_rgb(s, y),
                       fs.line[first_bad].lit ? fs.line[first_bad].first : 0,
                       fs.line[first_bad].lit);
            }
            printf("\n");
        }
        if (s.mode == Scenario::STRICT) {
            check(bad_lines == 0,
                  tag + ": every active display line shows its source row");
            // HW state 1: full-height render with the top rows torn.
            check(top_bad == 0,
                  tag + ": top " + std::to_string(TOP_ROWS) +
                  " display rows are not torn (bad=" +
                  std::to_string(top_bad) + ")");
            // HW state 2: only the bottom band(s) render.
            check(bottom_band_only == 0,
                  tag + ": not a bottom-band-only render");
        }
    }

    // ── BOOT SPLASH ─────────────────────────────────────────────────
    // With dafb_live low the display half must SUBSTITUTE the centred logo
    // for the entire pixel path.  Every scenario that reaches here has a
    // painted framebuffer, a programmed CLUT and a healthy fetch ring -- so
    // "the whole visible frame is the logo on black" is a real substitution
    // claim, not a claim about an empty pipeline.
    if (s.mode == Scenario::SPLASH && !frames.empty()) {
        const FrameStat& fs = frames.back();
        // Recomputed here from the geometry, independently of the RTL.
        const int border_x = (DST_W - active_w) / 2;
        const int mid_x    = border_x + active_w / 2;
        const int mid_y    = border_y + active_h / 2;
        const int x0       = mid_x - SPLASH_PIX / 2;
        const int y0       = mid_y - SPLASH_PIX / 2;
        printf("           splash box: %dx%d at (%d,%d), scale %dx\n",
               SPLASH_PIX, SPLASH_PIX, x0, y0, SPLASH_SCALE);

        long long fg_total = 0, fg_bad = 0, bg_bad = 0, outside_lit = 0;
        int first_bad_x = -1, first_bad_y = -1;
        uint32_t first_bad_got = 0, first_bad_want = 0;
        for (int v = 0; v < DST_H; v++) {
            for (int x = 0; x < DST_W; x++) {
                uint32_t got = fs.img[(size_t)v * DST_W + x];
                bool inbox = (x >= x0) && (x < x0 + SPLASH_PIX)
                          && (v >= y0) && (v < y0 + SPLASH_PIX);
                if (!inbox) {
                    if (got != SPLASH_BG) outside_lit++;
                    continue;
                }
                int col = (x - x0) / SPLASH_SCALE;
                int row = (v - y0) / SPLASH_SCALE;
                bool fg = (SPLASH_ROM[row] >> col) & 1u;
                uint32_t want = fg ? SPLASH_FG : SPLASH_BG;
                if (fg) fg_total++;
                if (got != want) {
                    if (fg) fg_bad++; else bg_bad++;
                    if (first_bad_x < 0) {
                        first_bad_x = x; first_bad_y = v;
                        first_bad_got = got; first_bad_want = want;
                    }
                }
            }
        }
        if (first_bad_x >= 0)
            printf("           first splash mismatch at (%d,%d): want %06x "
                   "got %06x\n", first_bad_x, first_bad_y,
                   first_bad_want, first_bad_got);
        // fg_total is a constant of the bitmap (405 set bits x 16 display
        // pixels each at 4x).  Asserted so a scenario whose box landed off
        // the captured image -- and therefore compared nothing -- cannot
        // report a clean sweep.
        check(fg_total > 0,
              std::string(s.name) + ": splash box contains foreground cells (" +
              std::to_string(fg_total) + " display px)");
        check(fg_bad == 0,
              std::string(s.name) + ": every logo FOREGROUND pixel is white (" +
              std::to_string(fg_bad) + " wrong)");
        check(bg_bad == 0,
              std::string(s.name) + ": every logo BACKGROUND pixel is black (" +
              std::to_string(bg_bad) + " wrong)");
        // The strong one: the normal video path is fully SUBSTITUTED, not
        // overlaid.  The same geometry with dafb_live=1 lights 1280x960
        // non-black pixels (see the paired STRICT scenario), so a splash
        // that merely drew on top would fail this by ~1.2 million.
        check(outside_lit == 0,
              std::string(s.name) + ": nothing outside the logo is lit (" +
              std::to_string(outside_lit) + " px)");

        if (const char* p = getenv("SPLASH_PPM")) {
            FILE* fp = fopen(p, "wb");
            if (fp) {
                fprintf(fp, "P6\n%d %d\n255\n", DST_W, DST_H);
                for (size_t i = 0; i < fs.img.size(); i++) {
                    uint32_t c = fs.img[i];
                    unsigned char px[3] = { (unsigned char)(c >> 16),
                                            (unsigned char)(c >> 8),
                                            (unsigned char)c };
                    fwrite(px, 1, 3, fp);
                }
                fclose(fp);
                printf("           wrote %s\n", p);
            }
        }
    }

    // ── Frame-to-frame IDENTITY ─────────────────────────────────────
    // The model framebuffer is painted once, before the run, and never
    // written again.  Consecutive frames must therefore be byte-identical.
    //
    // This is a DIFFERENT assertion from the per-frame checks above, not a
    // weaker restatement of them, because those compare each line against
    // `expected_rgb(y)` -- one colour per source row.  Every scenario here
    // paints rows of uniform colour, so a displacement WITHIN a row is
    // invisible to them by construction.  A hash over the whole line, held
    // equal across frames, is not.
    //
    // Not applied under RECOVERY/STARVED: those scenarios deliberately break
    // the fetch port, so frames are EXPECTED to differ; asserting identity
    // there would be asserting the fault did not happen.
    // NEGATIVE CONTROL (tb-scanout-frames-negctl): a one-bit mutation of one
    // captured line.  An identity check that cannot see that is not checking
    // anything, and "all frames matched" would be proof of nothing.
    if (getenv("SCANOUT_FRAMES_MUTATE") && frames.size() > 1)
        frames.back().line[V_TOTAL / 2].hash ^= 1ull;

    if (s.mode == Scenario::STRICT || s.mode == Scenario::SPLASH) {
        for (size_t f = (size_t)s.first_checked_frame + 1; f < frames.size(); f++) {
            int diff_lines = 0, first_diff = -1;
            for (int v = 0; v < V_TOTAL; v++) {
                if (frames[f].line[v].hash != frames[f-1].line[v].hash ||
                    frames[f].line[v].lit  != frames[f-1].line[v].lit) {
                    diff_lines++;
                    if (first_diff < 0) first_diff = v;
                }
            }
            if (diff_lines)
                printf("           frame %zu vs %zu: %d display lines differ, "
                       "first at line %d (lit %d vs %d)\n",
                       f - 1, f, diff_lines, first_diff,
                       frames[f-1].line[first_diff].lit, frames[f].line[first_diff].lit);
            check(diff_lines == 0,
                  std::string(s.name) + ": frame " + std::to_string(f-1) +
                  " == frame " + std::to_string(f) + " (static framebuffer)");
        }
    }

    if (s.mode == Scenario::RECOVERY) {
        // The scanner must keep re-arming under a permanently faulting fetch
        // port: in every window of RECOVERY_WINDOW consecutive frames at
        // least one frame must walk the whole source height and light
        // essentially the whole active window.
        const long long want_px =
            (long long)active_w * active_h * 95 / 100;
        int worst_gap = 0, gap = 0, good = 0;
        for (size_t f = (size_t)s.first_checked_frame; f < frames.size(); f++) {
            long long lit_px = 0;
            for (int v = 0; v < V_TOTAL; v++) lit_px += frames[f].line[v].lit;
            bool full = (frames[f].rows_requested >= s.vres)
                     && (lit_px >= want_px);
            if (full) { good++; if (gap > worst_gap) worst_gap = gap; gap = 0; }
            else gap++;
        }
        if (gap > worst_gap) worst_gap = gap;
        printf("           recovery: %d full frames, longest run of lost "
               "frames = %d (limit %d)\n", good, worst_gap,
               RECOVERY_WINDOW - 1);
        check(good > 0 && worst_gap <= RECOVERY_WINDOW - 1,
              std::string(s.name) +
              ": scanner keeps re-arming under continuous response loss");
    }

    // `line_underflow_sticky` is STICKY for the whole run, so it can only be
    // asserted clear in a scenario where nothing was ever supposed to make it
    // fire.  In the drop-injection scenarios a response really is lost, the
    // display really does reach a row that never completed, and the flag
    // firing is the signal doing its JOB -- so those scenarios report it
    // rather than assert on it.  (Before the drop-arming fix above they
    // injected into a discarded pre-roll frame and it never fired at all,
    // which is why this distinction did not previously show up.)  Coverage of
    // the "stays clear" direction is retained by every fault-free STRICT
    // scenario, and of the "can fire" direction by the STARVED positive
    // control below.
    bool fault_injected = (s.drop_at_frame >= 0) || (s.drop_at >= 0)
                       || (s.drop_period > 0);
    if ((s.mode == Scenario::STRICT || s.mode == Scenario::SPLASH)
        && !fault_injected)
        check(dut->line_underflow_sticky == 0,
              std::string(s.name) + ": no line-buffer underflow");
    else if (s.mode == Scenario::STRICT)
        printf("           line_underflow_sticky=%d after %lld injected drop(s)"
               " (reported, not asserted -- see comment)\n",
               (int)dut->line_underflow_sticky, dropped_total);
    // POSITIVE CONTROL.  Every other scenario asserts underflow stays CLEAR;
    // that check is worthless unless something proves the signal can fire at
    // all.  A starved port must set it.
    if (s.mode == Scenario::STARVED)
        check(dut->line_underflow_sticky == 1,
              std::string(s.name) +
              ": starved fetch port DOES raise line_underflow_sticky");
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_scanout_frames;
    fbmem.assign(FB_BYTES, 0);

    printf("tb_scanout_frames: multi-frame linebuf_scanout gate "
           "(shipping 1920x1080 VTG, LINE_COUNT=%d)\n", LINE_COUNT);

    // Every scenario runs >= 3 frames (requirement: a re-arm failure is only
    // visible across frames) over a source height far greater than
    // LINE_COUNT (requirement: the failure is at the ring wrap).
    Scenario scen[] = {
        // name           hres vres  2x  sh bpp base stride frm dropAt period
        //                dropFrm dropV  lat depth starve chk mode
        {"8bpp-640x480",  640, 480, true,  0, 1, 0,   1024,   4,  -1,     0,
                          -1,  0,     9, 32,  -1, 0, Scenario::STRICT},
        {"1bpp-640x480",  640, 480, true,  3, 1, 0,   1024,   4,  -1,     0,
                          -1,  0,     9, 32,  -1, 0, Scenario::STRICT},
        {"24bpp-640x480", 640, 480, true,  0, 4, 0,   4096,   4,  -1,     0,
                          -1,  0,     9, 32,  -1, 0, Scenario::STRICT},
        {"8bpp-1024x768", 1024, 768, false, 0, 1, 0,  1024,   3,  -1,     0,
                          -1,  0,     9, 32,  -1, 0, Scenario::STRICT},
        // Fault injection A (legacy shape, kept): ONE fetch response dropped
        // early, by request index.  Frames 0/1 were legitimately lost to the
        // old watchdog's two-frame detection latency; the credit-ring design
        // does not need them, but the gate is left at its historical strength
        // so this scenario can never get WEAKER than it was.
        {"8bpp-drop-rsp", 640, 480, true,  0, 1, 0,   1024,   5, 5000,    0,
                          -1,  0,     9, 32,  -1, 2, Scenario::STRICT},
        // Fault injection B: a response dropped every ~2/3 frame, forever --
        // the live board's condition, and the reason its symptom was a
        // PERSISTENT band rather than a one-frame glitch.
        {"8bpp-drop-loop",640, 480, true,  0, 1, 0,   1024,  10,   -1, 200000,
                          -1,  0,     9, 32,  -1, 0, Scenario::RECOVERY},

        // ── Added with the credit-ring rewrite ───────────────────────────
        // C: ONE dropped response at a NAMED point in frame 0, and every
        //    frame FROM FRAME 1 ON must be pixel-perfect.  This is the
        //    one-frame-recovery gate, and it is the single most important
        //    test here: it is what makes the af8fe56 watchdog unnecessary.
        //    The old design cannot pass it -- its watchdog needs TWO frame
        //    boundaries to even notice the desync.
        {"8bpp-drop-1frame", 640, 480, true,  0, 1, 0, 1024,  5,  -1,     0,
                          0, 300,      9, 32,  -1, 1, Scenario::STRICT},
        // D: the same at 1bpp -- the depth Mac OS actually boots in, and the
        //    depth whose 8-pixels-per-byte packing makes a one-byte response
        //    desync shift EIGHT display pixels instead of one.
        {"1bpp-drop-1frame", 640, 480, true,  3, 1, 0, 1024,  5,  -1,     0,
                          0, 300,      9, 32,  -1, 1, Scenario::STRICT},
        // E: a DEEP, SLOW port (long latency, deep queue) so the walk is still
        //    draining at start-of-frame.  That is precisely the case the hard
        //    resync must survive without waiting: the old design waited on
        //    outstanding_empty here, which is the unbounded wait that wedged
        //    the live board.  Stale stragglers must be discarded, not counted
        //    into the new walk.
        {"8bpp-slow-port", 640, 480, true,  0, 1, 0,  1024,   5,  -1,     0,
                          -1,  0,   400, 200,  -1, 1, Scenario::STRICT},
        // F: deep/slow port AND a dropped response -- both fault modes at
        //    once, so the discard window has to close on its TIMEOUT rather
        //    than on the count.
        {"8bpp-slow-drop", 640, 480, true,  0, 1, 0,  1024,   6,  -1,     0,
                          0, 300,    400, 200,  -1, 2, Scenario::STRICT},
        // G: POSITIVE CONTROL for line_underflow_sticky.  Every STRICT
        //    scenario asserts it stays clear; that is only meaningful if
        //    something proves it can fire.  Two healthy frames first (so the
        //    reporting gate is armed), then the port is crippled.
        {"8bpp-starved",  640, 480, true,  0, 1, 0,   1024,   5,  -1,     0,
                          -1,  0,     9, 32,   2, 0, Scenario::STARVED},

        // ── Boot splash (checkra1n) ──────────────────────────────────────
        // H: dafb_live LOW over an otherwise HEALTHY 8bpp 640x480 pipeline --
        //    painted framebuffer, programmed CLUT, fetch ring running.  That
        //    is deliberate: it makes "the screen shows the logo on black"
        //    a SUBSTITUTION claim.  If the splash were an overlay, or if it
        //    only worked because nothing else was rendering, `outside_lit`
        //    would be ~1.2 M non-black pixels instead of 0.
        //    The fetch-walk gates (rows_requested, req_y ahead of y_src, the
        //    LINE_COUNT bound, line_underflow_sticky, frame-to-frame
        //    identity) all still run here, UNCHANGED -- that is the proof
        //    the splash never touches the credit ring.
        {"splash-8bpp-640x480", 640, 480, true, 0, 1, 0, 1024, 3, -1,    0,
                          -1,  0,     9, 32,  -1, 0, Scenario::SPLASH, 0},
        // I: the SAME geometry with dafb_live HIGH must be ordinary video.
        //    Paired with H so "the splash renders" and "normal video is
        //    unaffected by the splash logic" are two runs of one rig.
        {"splash-retired-8bpp", 640, 480, true, 0, 1, 0, 1024, 3, -1,    0,
                          -1,  0,     9, 32,  -1, 0, Scenario::STRICT, 1},
        // J: 1bpp 640x480, the depth the Mac actually boots in, with the
        //    splash up.  Different bpp_shift, same substitution.
        {"splash-1bpp-640x480", 640, 480, true, 3, 1, 0, 1024, 3, -1,    0,
                          -1,  0,     9, 32,  -1, 0, Scenario::SPLASH, 0},
        // K: 1024x768 at 1:1 -- a DIFFERENT active window, so the centring
        //    is proved to be derived from the runtime geometry rather than
        //    hardcoded for 640x480@2x.  (Here border_x/border_y are
        //    448/156 instead of 320/60; both centre on (960,540), which is
        //    exactly the invariant a hardcoded constant would also satisfy
        //    -- what this scenario really rules out is a centring that
        //    tracked hres/vres WITHOUT the border term.)
        {"splash-8bpp-1024x768", 1024, 768, false, 0, 1, 0, 1024, 3, -1,  0,
                          -1,  0,     9, 32,  -1, 0, Scenario::SPLASH, 0},
    };

    for (const Scenario& s : scen) run_scenario(s);

    printf("\n──────────────────────────────────────────\n");
    printf("tb_scanout_frames: %d PASS / %d FAIL\n", pass_count, fail_count);
    dut->final();
    delete dut;
    return fail_count ? 1 : 0;
}
