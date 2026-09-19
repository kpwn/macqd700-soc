// tb_scanout_placement_sync.cpp -- directed CDC/frame-commit tests.

#include <cstdio>
#include <cstdint>
#include <verilated.h>
#include "Vtb_scanout_placement_sync_wrap.h"

static Vtb_scanout_placement_sync_wrap* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0;
static int n_fail = 0;

#define CHECK(cond, fmt, ...) do { \
    if (cond) { \
        n_pass++; \
        std::printf("  [PASS] " fmt "\n", ##__VA_ARGS__); \
    } else { \
        n_fail++; \
        std::printf("  [FAIL] " fmt "\n", ##__VA_ARGS__); \
    } \
} while (0)

static void tick() {
    dut->pclk = 0;
    dut->eval();
    dut->pclk = 1;
    dut->eval();
    sim_time++;
}

static void settle(int cycles) {
    dut->frame_start = 0;
    for (int i = 0; i < cycles; i++) tick();
}

static void frame_tick() {
    dut->frame_start = 1;
    tick();
    dut->frame_start = 0;
}

static void reset_dut() {
    dut->pclk = 0;
    dut->rst = 1;
    dut->frame_start = 0;
    dut->rom_scanout_fb_base_px = 0;
    dut->rom_scanout_fb_stride_px = 0;
    dut->hw_scanout_fb_base_px = 0;
    dut->hw_scanout_fb_stride_px = 0;
    dut->geom_scanout_bpp_shift = 0;
    // Depth-supported defaults HIGH so every pre-existing scenario keeps
    // exercising the same commit path it always did (they all model 8bpp or
    // shallower).  The depth-gate scenario below drives it low explicitly.
    dut->geom_scanout_depth_supported = 1;
    dut->geom_scanout_fb_base_px = 0;
    dut->geom_scanout_fb_stride_px = 1024;
    // 1 B/px (8bpp) by default so every pre-existing scenario's footprint
    // arithmetic is byte-for-byte what it was before the runtime-depth
    // channel existed.
    dut->geom_scanout_bytes_per_px = 1;
    dut->geom_scanout_hres = 0;
    dut->geom_scanout_vres = 0;
    dut->d24_scanout_bytes_per_px = 1;
    dut->d24_scanout_fb_base_px = 0;
    dut->d24_scanout_fb_stride_px = 1024;
    // The 4 MB instance's geometry used to be tied to these constants inside
    // the wrapper.  Defaulting them here keeps every pre-existing d24
    // scenario byte-for-byte identical.
    dut->d24_scanout_hres = 1024;
    dut->d24_scanout_vres = 768;
    // u_trans (mode-transition instance): reset-shaped defaults, i.e. what
    // the DAFB shim drives before the ROM has programmed anything.
    // depth_supported LOW is video.v's real pre-PCBR state (r_pcbr_set == 0).
    dut->tr_scanout_fb_base_px = 0;
    dut->tr_scanout_fb_stride_px = 1024;
    dut->tr_scanout_bpp_shift = 0;
    dut->tr_scanout_bytes_per_px = 1;
    dut->tr_scanout_depth_supported = 0;
    dut->tr_scanout_hres = 0;
    dut->tr_scanout_vres = 0;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;
}

// ─────────────────────────────────────────────────────────────────────────
// Mode-transition rig
// ─────────────────────────────────────────────────────────────────────────

// u_trans's elaborated shape -- must track tb_scanout_placement_sync_wrap.v,
// which in turn tracks rtl/soc/fpga_top_video.vh under VRAM_IN_DDR.
static const uint32_t TR_FB_LIMIT = 0x00400000;
static const int      TR_SRC_W    = 1024;
static const int      TR_SRC_H    = 768;

struct TrCfg {
    uint32_t base;
    uint32_t stride;
    int      shift;     // bpp_shift  (1bpp=3, 2bpp=2, 4bpp=1, 8bpp=0, 24bpp=0)
    int      bppx;      // bytes_per_px (1 = indexed, 4 = 24bpp direct)
    int      depth_ok;
    int      hres;
    int      vres;
};

// scanout_fetch.v:115-123,177-192, transcribed.  This is the gate that
// decides whether the fetch engine issues ANY request for the frame.
static bool tr_renderable(uint32_t base, uint32_t stride, int bppx, int shift,
                          int hres, int vres) {
    const bool direct = (bppx == 4);
    const int row_px_last = (hres == 0 || hres >= TR_SRC_W) ? (TR_SRC_W - 1)
                                                            : (hres - 1);
    const uint64_t row_last_idx = direct ? uint64_t(row_px_last)
                                         : (uint64_t(row_px_last) >> shift);
    const uint64_t row_last_off = direct ? ((row_last_idx << 2) + 3)
                                         : row_last_idx;
    const int row_y_last = (vres == 0 || vres >= TR_SRC_H) ? (TR_SRC_H - 1)
                                                           : (vres - 1);
    const uint64_t limit = uint64_t(base)
                         + uint64_t(stride) * uint64_t(row_y_last)
                         + row_last_off;
    return (uint64_t(stride) > row_last_off) && (limit < TR_FB_LIMIT);
}

static bool tr_cfg_renderable(const TrCfg& c) {
    return tr_renderable(c.base, c.stride, c.bppx, c.shift, c.hres, c.vres);
}

static void tr_drive(const TrCfg& c) {
    dut->tr_scanout_fb_base_px      = c.base;
    dut->tr_scanout_fb_stride_px    = c.stride;
    dut->tr_scanout_bpp_shift       = uint8_t(c.shift);
    dut->tr_scanout_bytes_per_px    = uint8_t(c.bppx);
    dut->tr_scanout_depth_supported = uint8_t(c.depth_ok);
    dut->tr_scanout_hres            = uint16_t(c.hres);
    dut->tr_scanout_vres            = uint16_t(c.vres);
}

static TrCfg tr_committed() {
    TrCfg c;
    c.base     = dut->tr_fb_base_px;
    c.stride   = dut->tr_fb_stride_px;
    c.shift    = dut->tr_bpp_shift;
    c.bppx     = dut->tr_bytes_per_px;
    c.depth_ok = 1;                    // not an output; irrelevant to the gate
    c.hres     = dut->tr_hres;
    c.vres     = dut->tr_vres;
    return c;
}

// The five register groups a mode set touches, in the order video.v decodes
// them.  BASE   = DAFB +0x00/+0x04, STRIDE = +0x08, HRES = Swatch HAL/HFP
// (+0x124..), VRES = Swatch VAL/VFP, DEPTH = AC842 PCBR +0x220 (which carries
// bpp_shift, bytes_per_px AND depth_supported together -- one register).
enum TrField { TF_BASE = 0, TF_STRIDE, TF_HRES, TF_VRES, TF_DEPTH, TF_N };

static const char* tr_field_name(int f) {
    switch (f) {
        case TF_BASE:   return "base";
        case TF_STRIDE: return "stride";
        case TF_HRES:   return "hres";
        case TF_VRES:   return "vres";
        default:        return "depth";
    }
}

static void tr_apply_field(TrCfg& cur, const TrCfg& to, int f) {
    switch (f) {
        case TF_BASE:   cur.base   = to.base;   break;
        case TF_STRIDE: cur.stride = to.stride; break;
        case TF_HRES:   cur.hres   = to.hres;   break;
        case TF_VRES:   cur.vres   = to.vres;   break;
        default:
            cur.shift    = to.shift;
            cur.bppx     = to.bppx;
            cur.depth_ok = to.depth_ok;
            break;
    }
}

struct TrResult {
    int  frames_checked;
    int  bad_frames;        // committed tuple the scanner cannot fetch at all
    int  first_bad_frame;
    TrCfg first_bad;
    int  first_bad_after;   // the last field write that had landed, or -1
    bool settled;           // committed == `to` once every write has landed
};

// Drive `from` -> `to` one register group at a time, in `order`, with the
// write landing `phase` cycles after a frame_start and `per_frame` groups
// written per frame period.  Sample and check the committed tuple after every
// frame_start.
//
// FRAME_CYCLES is only "long enough that the two-sample-agree synchroniser
// (3 flops) always settles between writes"; the module has no other timing
// relationship to the frame period.
//
// `targets` is a WAYPOINT LIST, not a single destination: a real mode set can
// pass through an intermediate geometry that is neither the old nor the new
// one, because the fields that produce it are separate registers.  hres, for
// instance, is video.v's `(HFP - HAL) << clockdiv_log2` -- three registers --
// so writing the PCBR clockdiv before the Swatch H pair derives an hres that
// was never programmed as such.  tb-dafb-24bpp-capacity scenario 9 OBSERVES
// those values coming out of the real shim; they are reproduced here rather
// than assumed.
static TrResult tr_run(const TrCfg& from, const TrCfg* targets, int n_targets,
                       const int* order, int phase, int per_frame) {
    const int FRAME_CYCLES = 64;
    TrResult r;
    r.frames_checked = 0;
    r.bad_frames = 0;
    r.first_bad_frame = -1;
    r.first_bad_after = -1;
    r.settled = false;

    // ── Prime: land `from` atomically and let it commit. ──────────────
    reset_dut();
    tr_drive(from);
    for (int i = 0; i < 6; i++) { settle(FRAME_CYCLES); frame_tick(); }

    TrCfg cur = from;
    int landed = -1;
    // steps: one per field write per waypoint, then a tail of empty frames so
    // the final configuration has every chance to commit.
    const int n_steps = TF_N * n_targets;
    const int tail_frames = 8;
    const int total_frames = (n_steps + per_frame - 1) / per_frame + tail_frames;

    for (int f = 0; f < total_frames; f++) {
        frame_tick();
        r.frames_checked++;
        TrCfg got = tr_committed();
        if (!tr_cfg_renderable(got)) {
            r.bad_frames++;
            if (r.first_bad_frame < 0) {
                r.first_bad_frame = f;
                r.first_bad = got;
                r.first_bad_after = (landed >= 0) ? order[landed % TF_N] : -1;
            }
        }
        // Register writes land `phase` cycles into the frame.
        settle(phase);
        for (int k = 0; k < per_frame; k++) {
            if (landed + 1 < n_steps) {
                landed++;
                tr_apply_field(cur, targets[landed / TF_N],
                               order[landed % TF_N]);
                tr_drive(cur);
            }
        }
        settle(FRAME_CYCLES - phase);
    }

    const TrCfg& to = targets[n_targets - 1];
    TrCfg got = tr_committed();
    r.settled = (got.base == to.base) && (got.stride == to.stride)
             && (got.shift == to.shift) && (got.bppx == to.bppx)
             && (got.hres == to.hres) && (got.vres == to.vres);
    return r;
}

static void run_transition_scenarios() {
    std::printf("\n  -- mode transitions (u_trans, 1024x768 src / 4 MB aperture) --\n");

    // Q700 modes.  base 4096 is the live board's (BASE reg 0x008 -> 8<<9).
    // Row pitches marked (HW) are the values read off the live board:
    //   640x480  any depth : STRIDE reg 0x100 -> 1024 bytes          (HW)
    //   832x624  1bpp      : STRIDE reg 0x0D0 ->  832 bytes          (HW,
    //                        the value `w 0xF9800008 0x340` replaced)
    //   832x624 24bpp      : STRIDE reg 0x340 -> 3328 bytes          (HW)
    //   640x480 24bpp      : STRIDE reg 0x400 -> 4096 bytes          (HW)
    // The two TIGHT-pitch entries below are not live-board values; they are
    // the minimum legal pitch for their mode, and they are here as a
    // headroom-free robustness case, labelled as such in the scenario names.
    const TrCfg c640x480x8   = { 4096, 1024, 0, 1, 1, 640, 480 };
    const TrCfg c832x624x1   = { 4096,  832, 3, 1, 1, 832, 624 };
    const TrCfg c832x624x8   = { 4096,  832, 0, 1, 1, 832, 624 };
    const TrCfg c832x624x24  = { 4096, 3328, 0, 4, 1, 832, 624 };
    const TrCfg c640x480x24  = { 4096, 4096, 0, 4, 1, 640, 480 };
    const TrCfg c640x480x8t  = { 4096,  640, 0, 1, 1, 640, 480 };  // tight
    // The clockdiv WAYPOINT, transcribed from what rtl/mac/video.v derives in
    // tb-dafb-24bpp-capacity scenario 9 when the AC842 PCBR's clockdiv field
    // has gone to 2 (the live-board 832x624 value) while the Swatch H pair
    // still holds the 640x480 values:  hres = (0x318 - 0x098) << 1 = 1280.
    // This is not an assumed register value -- it is an observed derivation.
    const TrCfg cclkdiv_way  = { 4096,  832, 0, 1, 1, 1280, 480 };

    // Sanity: BOTH endpoints of every transition below are renderable on
    // their own.  Without this the test could "fail" on a mode the scanner
    // was never able to display in the first place.
    struct Endp { const char* n; const TrCfg* c; };
    const Endp endpoints[] = {
        { "640x480x8",  &c640x480x8  }, { "832x624x1",  &c832x624x1  },
        { "832x624x8",  &c832x624x8  }, { "832x624x24", &c832x624x24 },
        { "640x480x24", &c640x480x24 }, { "640x480x8-tight", &c640x480x8t },
    };
    bool all_endpoints_ok = true;
    for (const Endp& e : endpoints)
        if (!tr_cfg_renderable(*e.c)) { all_endpoints_ok = false;
            std::printf("      endpoint %s is NOT renderable\n", e.n); }
    CHECK(all_endpoints_ok,
          "every transition endpoint is independently renderable "
          "(so any bad frame below is a TRANSITION defect, not a bad mode)");

    // Write orders.  The brief-documented Mac OS order is stride-before-depth
    // with the Swatch geometry rewritten in between; the others are here
    // because the exact order is not known from a live register dump, and a
    // test that bets on one order is a test that can be dodged.
    static const int ord_macos[TF_N]   = { TF_BASE, TF_STRIDE, TF_HRES, TF_VRES, TF_DEPTH };
    static const int ord_geom_1st[TF_N]= { TF_HRES, TF_VRES, TF_BASE, TF_STRIDE, TF_DEPTH };
    static const int ord_depth_1st[TF_N]={ TF_DEPTH, TF_BASE, TF_STRIDE, TF_HRES, TF_VRES };
    static const int ord_stride_1st[TF_N]={ TF_STRIDE, TF_DEPTH, TF_BASE, TF_HRES, TF_VRES };
    static const int ord_rev[TF_N]     = { TF_DEPTH, TF_VRES, TF_HRES, TF_STRIDE, TF_BASE };

    struct Order { const char* name; const int* o; };
    const Order orders[] = {
        { "macos(base,stride,hres,vres,depth)", ord_macos     },
        { "geom-first(hres,vres,base,stride,depth)", ord_geom_1st },
        { "depth-first(depth,base,stride,hres,vres)", ord_depth_1st },
        { "stride-first(stride,depth,base,hres,vres)", ord_stride_1st },
        { "reverse(depth,vres,hres,stride,base)", ord_rev     },
    };

    struct Trans {
        const char* name;
        const TrCfg* from;
        const TrCfg* via;     // optional waypoint, or nullptr
        const TrCfg* to;
    };
    const Trans transitions[] = {
        { "832x624x1 -> 832x624x24 (Millions, same geometry, HW pitches)",
          &c832x624x1,  nullptr,       &c832x624x24 },
        { "640x480x8 -> 832x624x8 via the clockdiv waypoint (HW pitches)",
          &c640x480x8,  &cclkdiv_way,  &c832x624x8  },
        { "640x480x8 -> 832x624x24 via the clockdiv waypoint (HW pitches)",
          &c640x480x8,  &cclkdiv_way,  &c832x624x24 },
        { "640x480x8 -> 832x624x24 (resolution change, HW pitches)",
          &c640x480x8,  nullptr,       &c832x624x24 },
        { "832x624x24 -> 640x480x8 (back out of Millions, HW pitches)",
          &c832x624x24, nullptr,       &c640x480x8  },
        { "640x480x8 -> 640x480x24 (Millions at 640x480, HW pitches)",
          &c640x480x8,  nullptr,       &c640x480x24 },
        { "832x624x8 -> 832x624x24 (Millions from 8bpp, HW pitches)",
          &c832x624x8,  nullptr,       &c832x624x24 },
        { "640x480x8 -> 832x624x24 (resolution change, TIGHT 640 pitch)",
          &c640x480x8t, nullptr,       &c832x624x24 },
    };

    // Phase sweep: where in the frame the CPU's register write lands relative
    // to frame_start.  The synchroniser is 3 flops deep and the commit is at
    // frame_start, so a write at phase 0..2 can straddle the commit while one
    // at phase >= 3 cannot -- both must be covered, and a single fixed phase
    // would miss a one-frame window.
    const int phases[]    = { 0, 1, 2, 3, 7, 31 };
    const int per_frames[] = { 1, 2, 5 };

    for (const Trans& t : transitions) {
        int worst_bad = 0;
        int runs = 0, bad_runs = 0, unsettled = 0;
        TrResult worst; worst.bad_frames = -1; worst.first_bad_frame = -1;
        const char* worst_order = "";
        int worst_phase = -1, worst_pf = -1;

        TrCfg way[2];
        int n_way = 0;
        if (t.via) way[n_way++] = *t.via;
        way[n_way++] = *t.to;

        for (const Order& o : orders) {
            for (int ph : phases) {
                for (int pf : per_frames) {
                    TrResult r = tr_run(*t.from, way, n_way, o.o, ph, pf);
                    runs++;
                    if (!r.settled) unsettled++;
                    if (r.bad_frames > 0) {
                        bad_runs++;
                        if (r.bad_frames > worst_bad) {
                            worst_bad = r.bad_frames;
                            worst = r;
                            worst_order = o.name;
                            worst_phase = ph;
                            worst_pf = pf;
                        }
                    }
                }
            }
        }

        if (worst_bad > 0) {
            std::printf("      worst: order=%s phase=%d writes/frame=%d -> "
                        "%d unrenderable frame(s), first at frame %d "
                        "after the '%s' write landed\n",
                        worst_order, worst_phase, worst_pf, worst.bad_frames,
                        worst.first_bad_frame,
                        worst.first_bad_after < 0
                            ? "(none)" : tr_field_name(worst.first_bad_after));
            std::printf("             committed tuple was base=%u stride=%u "
                        "bytes_per_px=%d bpp_shift=%d hres=%d vres=%d "
                        "-- scanout_fetch would issue ZERO requests\n",
                        worst.first_bad.base, worst.first_bad.stride,
                        worst.first_bad.bppx, worst.first_bad.shift,
                        worst.first_bad.hres, worst.first_bad.vres);
        }
        CHECK(bad_runs == 0,
              "%s: %d/%d write orderings x phases commit a tuple the scanner "
              "cannot fetch (black frame)", t.name, bad_runs, runs);
        CHECK(unsettled == 0,
              "%s: final configuration commits in every one of %d runs",
              t.name, runs);
    }
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_scanout_placement_sync_wrap;

    std::printf("-- tb_scanout_placement_sync: frame-boundary placement sync --\n");

    // ══════════════════════════════════════════════════════════════════
    // dafb_live -- the boot-splash switchover signal
    // ══════════════════════════════════════════════════════════════════
    // scanout_display.v shows the checkra1n boot splash while this is low.
    // The three properties it has to have, in order:
    //   1. FALSE out of reset and while the DAFB is unprogrammed -- otherwise
    //      the splash never appears at all;
    //   2. TRUE the first time a renderable placement with a real geometry
    //      commits -- otherwise the splash never goes away;
    //   3. STICKY -- a later unmapped/unsupported depth probe (which really
    //      happens; see the fb_ready comment in rtl/mac/video.v) must not
    //      put the splash back over a running desktop.
    // Run first, then reset again, so the sticky latch cannot leak into the
    // placement scenarios below.
    reset_dut();
    CHECK(dut->geom_dafb_live == 0, "dafb_live is LOW out of reset");
    // Unprogrammed DAFB: depth supported but geometry still 0x0.  A
    // committed placement with an empty active window renders nothing, so
    // this must NOT count as live.
    dut->geom_scanout_depth_supported = 1;
    dut->geom_scanout_hres = 0;
    dut->geom_scanout_vres = 0;
    settle(6);
    for (int i = 0; i < 3; i++) frame_tick();
    CHECK(dut->geom_dafb_live == 0,
          "dafb_live stays LOW with an unprogrammed 0x0 geometry");
    // Geometry programmed but the depth still unrenderable (the window
    // between the ROM's Swatch writes and its AC842 PCBR write).
    dut->geom_scanout_depth_supported = 0;
    dut->geom_scanout_hres = 640;
    dut->geom_scanout_vres = 480;
    settle(6);
    for (int i = 0; i < 3; i++) frame_tick();
    CHECK(dut->geom_dafb_live == 0,
          "dafb_live stays LOW until the depth is renderable");
    // Both now valid -- the first commit must raise it.
    dut->geom_scanout_depth_supported = 1;
    settle(6);
    frame_tick();
    CHECK(dut->geom_dafb_live == 1,
          "dafb_live goes HIGH on the first renderable DAFB placement");
    // A later depth probe the scanner cannot render must NOT bring the
    // splash back mid-session.
    dut->geom_scanout_depth_supported = 0;
    settle(6);
    for (int i = 0; i < 4; i++) frame_tick();
    CHECK(dut->geom_dafb_live == 1,
          "dafb_live is STICKY across a later unsupported-depth probe");
    dut->geom_scanout_depth_supported = 1;

    reset_dut();
    CHECK(dut->geom_dafb_live == 0, "dafb_live clears again on reset");
    CHECK(dut->rom_fb_base_px == 0, "ROM base resets to default 0");
    CHECK(dut->rom_fb_stride_px == 1024, "ROM stride resets to default source width");
    CHECK(dut->hw_fb_base_px == 0, "hardware base resets to default 0");
    CHECK(dut->hw_fb_stride_px == 1024, "hardware stride resets to default source width");

    dut->rom_scanout_fb_base_px = 0x100;
    dut->rom_scanout_fb_stride_px = 0x61e;
    settle(6);
    CHECK(dut->rom_fb_base_px == 0 && dut->rom_fb_stride_px == 1024,
          "stable DAFB update does not commit before frame_start");

    frame_tick();
    CHECK(dut->rom_fb_base_px == 0x100 && dut->rom_fb_stride_px == 0x61e,
          "valid ROM-ish DAFB base/stride commits on frame_start");

    dut->rom_scanout_fb_base_px = 0x200;
    dut->rom_scanout_fb_stride_px = 0x620;
    settle(6);
    CHECK(dut->rom_fb_base_px == 0x100 && dut->rom_fb_stride_px == 0x61e,
          "next DAFB update remains hidden until a later frame");

    frame_tick();
    CHECK(dut->rom_fb_base_px == 0x200 && dut->rom_fb_stride_px == 0x620,
          "frame_start commits the updated pair together");

    dut->rom_scanout_fb_base_px = 0x300;
    dut->rom_scanout_fb_stride_px = 0;
    settle(6);
    frame_tick();
    CHECK(dut->rom_fb_base_px == 0x300 && dut->rom_fb_stride_px == 1024,
          "zero stride is sanitized to default at commit");

    dut->rom_scanout_fb_base_px = 0x320;
    dut->rom_scanout_fb_stride_px = 512;
    settle(6);
    frame_tick();
    CHECK(dut->rom_fb_base_px == 0x320 && dut->rom_fb_stride_px == 512,
          "stride equal to source width is accepted");

    dut->rom_scanout_fb_base_px = 0x330;
    dut->rom_scanout_fb_stride_px = 511;
    settle(6);
    frame_tick();
    CHECK(dut->rom_fb_base_px == 0x320 && dut->rom_fb_stride_px == 512,
          "nonzero stride smaller than source width is rejected");

    dut->rom_scanout_fb_base_px = 0x400;
    dut->rom_scanout_fb_stride_px = 0x700;
    tick();
    dut->rom_scanout_fb_base_px = 0x401;
    dut->rom_scanout_fb_stride_px = 0x701;
    tick();
    frame_tick();
    CHECK(dut->rom_fb_base_px == 0x320 && dut->rom_fb_stride_px == 512,
          "unstable one-cycle DAFB samples are not committed");
    settle(6);
    frame_tick();
    CHECK(dut->rom_fb_base_px == 0x401 && dut->rom_fb_stride_px == 0x701,
          "latest stable DAFB pair commits after filtering");

    dut->hw_scanout_fb_base_px = 0x0;
    dut->hw_scanout_fb_stride_px = 1024;
    settle(6);
    frame_tick();
    CHECK(dut->hw_fb_base_px == 0x0 && dut->hw_fb_stride_px == 1024,
          "valid 1024x768 hardware placement commits");

    // ── The two admission gates, each at its EXACT equality point ──────
    // mode_admit.v owns both inequalities and they use DIFFERENT operators:
    //     stride gate  stride >= row_span_bytes      count vs count
    //     memory gate  frame_last_addr < FB_MAX      inclusive addr vs count
    // Two operators, and each is only distinguishable from its neighbour on
    // ONE tuple -- the one where the two sides are exactly equal.  Every
    // scenario that came before this one clears both gates by hundreds of
    // bytes, so a `<` silently widened to `<=`, or a `>=` narrowed to `>`,
    // changed no observable behaviour anywhere in this file.  Proven, not
    // assumed: mutating mode_admit.v's range gate to `<=` left this tb GREEN
    // until these two vectors existed.
    //
    // u_hw is elaborated with FB_MAX_PIXELS = 1024*768 = 786,432 and
    // SRC 1024x768, i.e. ZERO headroom, which is what makes the boundary
    // reachable with ordinary numbers.
    //
    // MEMORY GATE, at equality.  base 1, stride 1024, 8bpp:
    //     1 + 1024*767 + 1023 = 786,432 == FB_MAX_PIXELS
    // The last byte the scanner would read is one past the last byte that
    // exists.  `<` refuses it; `<=` would admit a one-byte overrun.
    dut->hw_scanout_fb_base_px = 0x1;
    dut->hw_scanout_fb_stride_px = 1024;
    settle(6);
    frame_tick();
    CHECK(dut->hw_fb_base_px == 0x0 && dut->hw_fb_stride_px == 1024,
          "frame whose LAST BYTE lands exactly ON the aperture limit is "
          "rejected (786432 == FB_MAX; separates `<` from `<=`)");

    // STRIDE GATE, at equality minus one.  base 0, stride 1023, 8bpp:
    // one visible row is 1024 bytes, so a 1023-byte pitch is one byte short
    // and the scanner would read into the next row on every line.  The frame
    // still FITS memory (0 + 1023*767 + 1023 = 785,664 < 786,432), so this
    // isolates the stride gate from the range gate -- the pre-existing
    // "too-small stride" vector below uses base 0x180 and fails BOTH, so it
    // could never have told them apart.
    dut->hw_scanout_fb_base_px = 0x0;
    dut->hw_scanout_fb_stride_px = 1023;
    settle(6);
    frame_tick();
    CHECK(dut->hw_fb_base_px == 0x0 && dut->hw_fb_stride_px == 1024,
          "stride one byte short of a visible row is rejected while the "
          "frame still fits memory (isolates the stride gate)");

    dut->hw_scanout_fb_base_px = 0x100;
    dut->hw_scanout_fb_stride_px = 1024;
    settle(6);
    frame_tick();
    CHECK(dut->hw_fb_base_px == 0x0 && dut->hw_fb_stride_px == 1024,
          "placement beyond physical 1024x768 backing is rejected");

    dut->hw_scanout_fb_base_px = 0x180;
    dut->hw_scanout_fb_stride_px = 1023;
    settle(6);
    frame_tick();
    CHECK(dut->hw_fb_base_px == 0x0 && dut->hw_fb_stride_px == 1024,
          "too-small 1024x768 hardware stride is rejected");

    dut->hw_scanout_fb_base_px = 0xf0010;
    dut->hw_scanout_fb_stride_px = 0x40000;
    settle(6);
    frame_tick();
    CHECK(dut->hw_fb_base_px == 0x0 && dut->hw_fb_stride_px == 1024,
          "out-of-range 0xf0010/0x40000 1024x768 hardware placement is rejected");

    // ── Geometry channel (bpp_shift/hres/vres) two-sample-agree filter ──
    // Same torn-value hazard as fb_base/fb_stride: a DAFB register write
    // straddling a frame_start sample can leave bpp_shift/hres/vres
    // torn for one committed frame if they commit unconditionally.
    dut->geom_scanout_bpp_shift = 0;
    dut->geom_scanout_hres = 640;
    dut->geom_scanout_vres = 480;
    settle(6);
    frame_tick();
    CHECK(dut->geom_bpp_shift == 0 && dut->geom_hres == 640 && dut->geom_vres == 480,
          "stable geometry (bpp_shift/hres/vres) commits on frame_start");

    // Continuously-changing (never-settled) sample straddling frame_start:
    // drive a NEW distinct value every pclk for several cycles (never
    // letting two consecutive synchronizer samples agree) right up to
    // the frame boundary.  Without the two-sample-agree filter, the
    // unconditional `stable <= sync` assignment mirrors whatever
    // transient sample happens to be in flight, and frame_start commits
    // it; WITH the filter, `stable` never advances off the last
    // genuinely-settled value because `sync` never repeats, so the
    // commit must still read back the OLD stable geometry.
    for (int i = 0; i < 8; i++) {
        dut->geom_scanout_bpp_shift = uint8_t((i % 3) + 1);      // 1..3, never repeats immediate previous value pattern
        dut->geom_scanout_hres = uint16_t(700 + i * 11);
        dut->geom_scanout_vres = uint16_t(500 + i * 7);
        tick();
    }
    frame_tick();
    CHECK(dut->geom_bpp_shift == 0 && dut->geom_hres == 640 && dut->geom_vres == 480,
          "continuously-changing (never-settled) geometry is not committed mid-tear (torn-value filter)");

    // Now drive one final, distinct value and let it actually settle
    // (held steady) before the next frame_start — must commit cleanly.
    dut->geom_scanout_bpp_shift = 2;   // 2bpp
    dut->geom_scanout_hres = 800;
    dut->geom_scanout_vres = 600;
    settle(6);
    frame_tick();
    CHECK(dut->geom_bpp_shift == 2 && dut->geom_hres == 800 && dut->geom_vres == 600,
          "latest stable geometry commits after filtering");

    // ── Depth gate: an unrenderable depth must NOT commit a placement ──
    // depth_supported is low for the three genuinely-unmapped AC842 depth
    // codes (0x04/0x0C/0x14).  When it is low this module must retain the
    // last good placement -- the same fail-safe it already applies to an
    // out-of-range base/stride.  Committing instead would scan each row at
    // the wrong bytes-per-pixel and push raw bytes through the CLUT as if
    // they were palette indices.
    //
    // First establish a known-good committed placement at a supported depth.
    dut->geom_scanout_depth_supported = 1;
    dut->geom_scanout_fb_base_px = 0x1000;
    dut->geom_scanout_bpp_shift = 0;   // 8bpp
    settle(6);
    frame_tick();
    CHECK(dut->geom_fb_base_px == 0x1000,
          "supported depth commits a new base (0x%x)", dut->geom_fb_base_px);

    // Now select an unrenderable depth AND a new base, as a real 24bpp mode
    // switch would (Mac OS reprograms base and depth together).  The base
    // must NOT move.
    dut->geom_scanout_depth_supported = 0;
    dut->geom_scanout_fb_base_px = 0x2000;
    settle(6);
    frame_tick();
    CHECK(dut->geom_fb_base_px == 0x1000,
          "unsupported depth retains the last good base (got 0x%x, want 0x1000)",
          dut->geom_fb_base_px);

    // Hold it unsupported across several frames -- must stay put, not creep.
    for (int i = 0; i < 4; i++) frame_tick();
    CHECK(dut->geom_fb_base_px == 0x1000,
          "unsupported depth stays latched across repeated frames (0x%x)",
          dut->geom_fb_base_px);

    // Returning to a supported depth must release the gate and commit the
    // pending base -- the gate is a hold, not a permanent lockout.
    dut->geom_scanout_depth_supported = 1;
    settle(6);
    frame_tick();
    CHECK(dut->geom_fb_base_px == 0x2000,
          "returning to a supported depth commits the pending base (0x%x)",
          dut->geom_fb_base_px);

    // ══════════════════════════════════════════════════════════════════
    // Runtime bytes-per-pixel channel (8bpp <-> 24bpp)
    // ══════════════════════════════════════════════════════════════════
    // 24bpp on DAFB is 4 VRAM bytes per pixel (MAME dafb.cpp:340-350 walks
    // the framebuffer as u32s -- see rtl/mac/video.v decode_bytes_per_px), so
    // the footprint this module validates a placement against has to scale
    // with it.  u_geom is 1024x768, hres/vres driven, FB_BYTE_LIMIT = 2 MB.
    std::printf("\n-- runtime bytes-per-pixel (24bpp footprint) --\n");

    // Establish a clean 8bpp baseline: hres=1024, stride=1024, base=0.
    dut->geom_scanout_depth_supported = 1;
    dut->geom_scanout_bytes_per_px = 1;
    dut->geom_scanout_bpp_shift = 0;
    dut->geom_scanout_hres = 1024;
    dut->geom_scanout_vres = 768;
    dut->geom_scanout_fb_stride_px = 1024;
    dut->geom_scanout_fb_base_px = 0x4000;
    settle(6);
    frame_tick();
    CHECK(dut->geom_fb_base_px == 0x4000 && dut->geom_bytes_per_px == 1,
          "8bpp baseline commits (base=0x%x bppx=%u)",
          dut->geom_fb_base_px, dut->geom_bytes_per_px);

    // 24bpp with an 8bpp-sized stride.  1024 bytes cannot hold a 1024-pixel
    // row at 4 B/px (needs 4096), so the stride gate must reject it and the
    // 8bpp placement must stay in force.  Before the runtime channel existed
    // this was invisible: the check measured every depth at 1 B/px.
    dut->geom_scanout_bytes_per_px = 4;
    dut->geom_scanout_fb_base_px = 0x8000;
    settle(6);
    frame_tick();
    CHECK(dut->geom_fb_base_px == 0x4000 && dut->geom_bytes_per_px == 1,
          "24bpp with an 8bpp stride (1024 < 4*1024) is rejected; 8bpp "
          "placement retained (base=0x%x bppx=%u)",
          dut->geom_fb_base_px, dut->geom_bytes_per_px);

    // Same depth, now with a legal 4 B/px stride -- but 1024x768x24bpp needs
    // 4096*767 + 4095 = 3,145,727 bytes, which overruns u_geom's 2 MB
    // FB_BYTE_LIMIT.  Still rejected, now by the range gate rather than the
    // stride gate.  THIS is the check that says a 2 MB aperture cannot serve
    // 1024x768 direct colour, whatever the stride.
    dut->geom_scanout_fb_stride_px = 4096;
    settle(6);
    frame_tick();
    CHECK(dut->geom_fb_base_px == 0x4000 && dut->geom_bytes_per_px == 1,
          "1024x768x24bpp overruns a 2 MB aperture and is rejected "
          "(base=0x%x bppx=%u)",
          dut->geom_fb_base_px, dut->geom_bytes_per_px);

    // Narrow the visible width to 640.  The 24bpp footprint is bounded by
    // hres (the scanner bounds its 24bpp fetch the same way -- see
    // linebuf_scanout.v req_x_last), so a 640-wide row needs only 2560 bytes
    // of stride and 2560*767 + 2559 = 1,965,679 bytes total, which DOES fit
    // 2 MB.  Measuring this against SRC_W=1024 instead would reject every
    // real narrow direct-colour mode.
    dut->geom_scanout_hres = 640;
    dut->geom_scanout_vres = 480;
    dut->geom_scanout_fb_stride_px = 2560;
    settle(6);
    frame_tick();
    CHECK(dut->geom_fb_base_px == 0x8000 && dut->geom_bytes_per_px == 4,
          "640-wide 24bpp (stride 2560) fits 2 MB and commits "
          "(base=0x%x bppx=%u)",
          dut->geom_fb_base_px, dut->geom_bytes_per_px);

    // ... and one byte less of stride is rejected: 640*4 = 2560 exactly, so
    // 2556 (still 4-byte aligned) cannot hold the row.
    dut->geom_scanout_fb_stride_px = 2556;
    dut->geom_scanout_fb_base_px = 0xC000;
    settle(6);
    frame_tick();
    CHECK(dut->geom_fb_base_px == 0x8000,
          "640-wide 24bpp with stride 2556 (< 4*640) is rejected (base=0x%x)",
          dut->geom_fb_base_px);

    // Runtime switch BACK to 8bpp must release the gate immediately.
    dut->geom_scanout_bytes_per_px = 1;
    dut->geom_scanout_fb_stride_px = 1024;
    settle(6);
    frame_tick();
    CHECK(dut->geom_fb_base_px == 0xC000 && dut->geom_bytes_per_px == 1,
          "switching back to 8bpp commits the pending base (base=0x%x bppx=%u)",
          dut->geom_fb_base_px, dut->geom_bytes_per_px);

    // ── 4 MB aperture: what actually unblocks 1024x768x24bpp ──────────
    // u_d24 is the same 1024x768 geometry with FB_BYTE_LIMIT = 4 MB (the
    // VRAM_IN_DDR value) and ADDR_W=22.  The placement rejected above by
    // u_geom's 2 MB limit is accepted here, unchanged.
    std::printf("\n-- 4 MB aperture (VRAM_IN_DDR) 24bpp at 1024x768 --\n");
    dut->d24_scanout_bytes_per_px = 1;
    dut->d24_scanout_fb_stride_px = 1024;
    dut->d24_scanout_fb_base_px = 0;
    settle(6);
    frame_tick();
    CHECK(dut->d24_fb_stride_px == 1024 && dut->d24_bytes_per_px == 1,
          "4 MB instance: 8bpp baseline commits (stride=%u bppx=%u)",
          dut->d24_fb_stride_px, dut->d24_bytes_per_px);

    dut->d24_scanout_bytes_per_px = 4;
    dut->d24_scanout_fb_stride_px = 4096;
    settle(6);
    frame_tick();
    CHECK(dut->d24_fb_stride_px == 4096 && dut->d24_bytes_per_px == 4,
          "4 MB instance: 1024x768x24bpp (stride 4096, 3,145,728 bytes) "
          "commits -- the aperture, not the depth decode, was the blocker "
          "(stride=%u bppx=%u)",
          dut->d24_fb_stride_px, dut->d24_bytes_per_px);

    // ── 4 MB aperture: 832x624x24bpp ("Millions" on the 16in RGB) ─────
    // The combination that glitches on hardware.  Everything about it had
    // been tested SEPARATELY and never together: 24bpp only ever at 640x480
    // and 1024x768, 832x624 only ever at 1bpp/8bpp.  This is the admission
    // half of that gap (the pixel half is tb-scanout-ddr-frames'
    // 24bpp-832x624-3_2 scenario).
    //
    // Both plausible row pitches are checked, because the DAFB stride
    // register is a literal byte pitch and Mac OS does not always program it
    // tight -- 640x480x24bpp uses 4096 for a 2560 B payload:
    //   tight   3328 = 832*4  -> footprint 3584 + 3328*623 + 3327 = 2,080,255
    //   rounded 4096          -> footprint 3584 + 4096*623 + 3327 = 2,559,119
    // Both are inside 4 MB, so both must commit.  (Both would be REJECTED by
    // a 2 MB URAM aperture in the rounded case -- 2,559,119 > 2,097,152 --
    // which is worth knowing if this mode is ever run on a non-VRAM_IN_DDR
    // build: the placement would be retained at its old 8bpp value while
    // hres/vres/scale_n commit anyway, since those commit unconditionally.)
    std::printf("\n-- 4 MB aperture (VRAM_IN_DDR) 24bpp at 832x624 --\n");
    dut->d24_scanout_hres = 832;
    dut->d24_scanout_vres = 624;
    dut->d24_scanout_bytes_per_px = 1;
    dut->d24_scanout_fb_stride_px = 1024;
    dut->d24_scanout_fb_base_px = 3584;
    settle(6);
    frame_tick();
    CHECK(dut->d24_fb_stride_px == 1024 && dut->d24_bytes_per_px == 1
          && dut->d24_fb_base_px == 3584,
          "832x624x8bpp baseline commits (base=%u stride=%u bppx=%u)",
          dut->d24_fb_base_px, dut->d24_fb_stride_px, dut->d24_bytes_per_px);
    CHECK(dut->d24_scale_n == 1,
          "832x624 picks INTEGER N=1 (sharp, letterboxed; scale_n=%u)",
          dut->d24_scale_n);

    dut->d24_scanout_bytes_per_px = 4;
    dut->d24_scanout_fb_stride_px = 3328;
    settle(6);
    frame_tick();
    CHECK(dut->d24_fb_stride_px == 3328 && dut->d24_bytes_per_px == 4,
          "832x624x24bpp at the TIGHT pitch 3328 (footprint 2,080,255) "
          "commits (stride=%u bppx=%u)",
          dut->d24_fb_stride_px, dut->d24_bytes_per_px);

    // Back to 8bpp, then in at the rounded pitch, so the second case is a
    // real transition rather than a stride tweak on an already-24bpp commit.
    dut->d24_scanout_bytes_per_px = 1;
    dut->d24_scanout_fb_stride_px = 1024;
    settle(6);
    frame_tick();
    dut->d24_scanout_bytes_per_px = 4;
    dut->d24_scanout_fb_stride_px = 4096;
    settle(6);
    frame_tick();
    CHECK(dut->d24_fb_stride_px == 4096 && dut->d24_bytes_per_px == 4,
          "832x624x24bpp at the ROUNDED pitch 4096 (footprint 2,559,119) "
          "commits (stride=%u bppx=%u)",
          dut->d24_fb_stride_px, dut->d24_bytes_per_px);

    // NEGATIVE CONTROL: a stride that cannot hold one 832-pixel 24bpp row
    // must be rejected and the last good placement retained.  Without this,
    // the two checks above would pass on a module that admitted everything.
    dut->d24_scanout_bytes_per_px = 1;
    dut->d24_scanout_fb_stride_px = 1024;
    settle(6);
    frame_tick();
    dut->d24_scanout_bytes_per_px = 4;
    dut->d24_scanout_fb_stride_px = 3327;   // one byte short of 832*4
    settle(6);
    frame_tick();
    CHECK(dut->d24_fb_stride_px == 1024 && dut->d24_bytes_per_px == 1,
          "832x624x24bpp with stride 3327 (< 4*832) is rejected; 8bpp "
          "placement retained (stride=%u bppx=%u)",
          dut->d24_fb_stride_px, dut->d24_bytes_per_px);

    // Restore the instance's defaults for anything that runs after this.
    dut->d24_scanout_hres = 1024;
    dut->d24_scanout_vres = 768;
    dut->d24_scanout_bytes_per_px = 1;
    dut->d24_scanout_fb_stride_px = 1024;
    dut->d24_scanout_fb_base_px = 0;
    settle(6);
    frame_tick();

    // ══════════════════════════════════════════════════════════════════
    // Output-scale policy (scale_n) -- place_plan.v's whole contract
    // ══════════════════════════════════════════════════════════════════
    // docs/video_path_review.md S3, settled 2026-08-20:
    //
    //     N = min(floor(DST_W/hres), floor(DST_H/vres)), clamped to >= 1
    //     integer nearest-neighbour only; NO fractional ratios
    //
    // The `want` column below is computed BY HAND from that formula (shown
    // in `why`), not from the RTL, so this is the table place_plan.v has to
    // satisfy rather than a transcription of what it does.
    //
    // The 832x624 row is the one that CHANGED: it used to select a 3:2 rung
    // (1248x936), whose 2,1,2,1 line doubling aliases that mode's 1bpp
    // 50%-dither desktop into fine striping.  Sharp and smaller wins.
    std::printf("\n-- output-scale policy (integer N, no fractional rungs) --\n");
    dut->geom_scanout_depth_supported = 1;
    dut->geom_scanout_bytes_per_px = 1;
    dut->geom_scanout_bpp_shift = 0;
    dut->geom_scanout_fb_stride_px = 1024;
    dut->geom_scanout_fb_base_px = 0;

    struct ScaleCase { int hres, vres, want; const char* why; };
    const ScaleCase scale_cases[] = {
        // ── the Q700 mode set ──
        {  512, 384, 2, "min(1920/512=3, 1080/384=2) = 2 -> 1024x768"     },
        {  640, 480, 2, "min(1920/640=3, 1080/480=2) = 2 -> 1280x960"     },
        {  832, 624, 1, "min(1920/832=2, 1080/624=1) = 1 -> 832x624 sharp"},
        { 1024, 768, 1, "min(1920/1024=1, 1080/768=1) = 1"                },
        { 1152, 870, 1, "min(1920/1152=1, 1080/870=1) = 1"                },
        // ── boundary cases for the fit comparison ──
        // Both axes land EXACTLY on the raster at N=2, so the test must be
        // <=, not <.  This is the row that catches an off-by-one there.
        {  960, 540, 2, "2x960=1920 and 2x540=1080, EXACTLY DST"          },
        // One pixel wider: 2x961 = 1922 > 1920, so N drops to 1.
        {  961, 540, 1, "2x961=1922 > 1920, so N=1"                       },
        // The clamp ceiling.  min(1920/240=8, 1080/180=6) = 6, capped at 4.
        {  240, 180, 4, "policy would give 6; the ladder tops out at 4"    },
        // Nothing fits even at 1x -- N must clamp UP to 1, never to 0.
        { 2048, 1536, 1, "wider than DST on both axes; N clamps to 1"     },
    };
    for (const ScaleCase& sc : scale_cases) {
        dut->geom_scanout_hres = uint16_t(sc.hres);
        dut->geom_scanout_vres = uint16_t(sc.vres);
        settle(6);
        frame_tick();
        CHECK(dut->geom_scale_n == sc.want,
              "%dx%d -> scale_n %u (want %d): %s",
              sc.hres, sc.vres, dut->geom_scale_n, sc.want, sc.why);
    }

    // An unprogrammed geometry (hres/vres still 0) has an EMPTY active
    // window whatever N says, so the ladder's "0 fits every rung" answer is
    // meaningless.  place_plan pins it at 1 -- never 0 (which would be a
    // zero Bresenham denominator) and never the clamp ceiling (which would
    // read as a real decision).  This is the pre-DAFB reset path.
    dut->geom_scanout_hres = 0;
    dut->geom_scanout_vres = 0;
    settle(6);
    frame_tick();
    CHECK(dut->geom_scale_n == 1,
          "unprogrammed geometry (0x0) pins N at 1 (scale_n=%u)",
          dut->geom_scale_n);

    // ══════════════════════════════════════════════════════════════════
    // LOUD FAILURE: reject_reason / sticky / the rejected tuple
    // ══════════════════════════════════════════════════════════════════
    // docs/video_path_review.md S4.2.  Before this channel existed, a
    // refused placement produced rd_en=0 indefinitely with no reason code,
    // no counter and no status bit -- diagnosing the resulting black screen
    // meant hand-evaluating four inequalities against live register values.
    //
    // Each case below provokes ONE gate and checks three separate things:
    // the reason code names the RIGHT gate (not just "something failed"),
    // the sticky bit sets, and the LATCHED tuple is the one that was judged.
    // The last part matters because the DAFB keeps being written after a
    // rejection, so "print the current registers" would report a tuple that
    // was never the one refused.
    //
    // u_geom is the instance used: ADDR_W=20, SRC 1024x768, 2 MB aperture,
    // and every input driveable.
    enum { REJ_NONE = 0, REJ_DEPTH_UNSUP = 1, REJ_STRIDE_SHORT = 2,
           REJ_FRAME_OOM = 3, REJ_GEOMETRY_ZERO = 4 };
    std::printf("\n-- reject_reason / sticky / rejected tuple --\n");

    // Baseline: a placement that is admitted from every angle.  Establishes
    // REJ_NONE and, just as importantly, a CLEAR sticky bit -- so every set
    // below is attributable to the case that set it.
    dut->geom_scanout_depth_supported = 1;
    dut->geom_scanout_bytes_per_px = 1;
    dut->geom_scanout_bpp_shift = 0;
    dut->geom_scanout_hres = 1024;
    dut->geom_scanout_vres = 768;
    dut->geom_scanout_fb_stride_px = 1024;
    dut->geom_scanout_fb_base_px = 0;
    settle(6);
    frame_tick();
    CHECK(dut->geom_reject_reason == REJ_NONE,
          "admitted placement reports REJ_NONE (reason=%u)", dut->geom_reject_reason);
    CHECK(dut->geom_rejected_sticky == 0,
          "a commit clears placement_rejected_sticky (sticky=%u)",
          dut->geom_rejected_sticky);

    // (1) DEPTH.  16/24bpp on this instance: the scanner cannot render it,
    // so the last good placement is retained.
    dut->geom_scanout_depth_supported = 0;
    dut->geom_scanout_fb_base_px = 0x4000;
    settle(6);
    frame_tick();
    CHECK(dut->geom_reject_reason == REJ_DEPTH_UNSUP,
          "unsupported depth -> REJ_DEPTH_UNSUP (reason=%u)", dut->geom_reject_reason);
    CHECK(dut->geom_rejected_sticky == 1, "depth rejection sets the sticky bit");
    CHECK(dut->geom_reject_reason_latched == REJ_DEPTH_UNSUP,
          "latched reason is the depth one (%u)", dut->geom_reject_reason_latched);
    CHECK(dut->geom_reject_base_px == 0x4000,
          "latched tuple carries the REFUSED base (0x%x)",
          (unsigned)dut->geom_reject_base_px);
    CHECK(dut->geom_fb_base_px == 0,
          "the refused base was NOT committed (committed base 0x%x)",
          (unsigned)dut->geom_fb_base_px);

    // (2) STRIDE vs ROW EXTENT.  512-byte pitch cannot hold a 1024-byte
    // 8bpp row.  Depth restored first so this case is the only failing gate.
    dut->geom_scanout_depth_supported = 1;
    dut->geom_scanout_fb_base_px = 0;
    dut->geom_scanout_fb_stride_px = 512;
    settle(6);
    frame_tick();
    CHECK(dut->geom_reject_reason == REJ_STRIDE_SHORT,
          "stride 512 < 1024-byte row -> REJ_STRIDE_SHORT (reason=%u)",
          dut->geom_reject_reason);
    CHECK(dut->geom_reject_stride_px == 512,
          "latched tuple carries the REFUSED stride (%u)",
          (unsigned)dut->geom_reject_stride_px);
    CHECK(dut->geom_reject_hres_px == 1024 && dut->geom_reject_vres_px == 768,
          "latched tuple carries the refused geometry (%ux%u)",
          (unsigned)dut->geom_reject_hres_px, (unsigned)dut->geom_reject_vres_px);

    // (3) FRAME OUT OF MEMORY.  Stride is sane (4096 >= 1024) but
    // 0x30000 + 4096*767 + 1023 = 3,339,263 overruns the 2 MB aperture.
    // Reported as REJ_FRAME_OOM, NOT as a stride problem -- the whole point
    // of a reason code is telling those two apart.
    dut->geom_scanout_fb_stride_px = 4096;
    dut->geom_scanout_fb_base_px = 0x30000;
    settle(6);
    frame_tick();
    CHECK(dut->geom_reject_reason == REJ_FRAME_OOM,
          "frame past the aperture -> REJ_FRAME_OOM (reason=%u)",
          dut->geom_reject_reason);
    CHECK(dut->geom_reject_base_px == 0x30000 && dut->geom_reject_stride_px == 4096,
          "latched tuple is the out-of-memory one (base 0x%x stride %u)",
          (unsigned)dut->geom_reject_base_px, (unsigned)dut->geom_reject_stride_px);

    // (4) GEOMETRY ZERO.  Every inequality passes, so the placement COMMITS
    // (this is the pre-DAFB path and it must keep working) -- but the active
    // window is 0 pixels wide and nothing can render.  That is a black
    // screen with a different cause, and it gets its own code rather than
    // being left for a reader to infer from hres=0 in a hex dump.  The
    // sticky bit must CLEAR here: nothing was refused.
    dut->geom_scanout_fb_stride_px = 1024;
    dut->geom_scanout_fb_base_px = 0;
    dut->geom_scanout_hres = 0;
    dut->geom_scanout_vres = 0;
    settle(6);
    frame_tick();
    CHECK(dut->geom_reject_reason == REJ_GEOMETRY_ZERO,
          "0x0 geometry -> REJ_GEOMETRY_ZERO (reason=%u)", dut->geom_reject_reason);
    CHECK(dut->geom_rejected_sticky == 0,
          "REJ_GEOMETRY_ZERO is not a refusal, so the sticky bit clears (sticky=%u)",
          dut->geom_rejected_sticky);

    // (5) RECOVERY.  A good tuple after a bad one clears the sticky bit and
    // returns REJ_NONE, so the bit really does mean "since the last commit"
    // and not "ever".
    dut->geom_scanout_hres = 1024;
    dut->geom_scanout_vres = 768;
    settle(6);
    frame_tick();
    CHECK(dut->geom_reject_reason == REJ_NONE && dut->geom_rejected_sticky == 0,
          "a good tuple clears the sticky bit and reports REJ_NONE (reason=%u sticky=%u)",
          dut->geom_reject_reason, dut->geom_rejected_sticky);

    // ── MODE-TRANSITION scenarios (u_trans, shipping VRAM_IN_DDR shape) ──
    //
    // Everything above holds the placement tuple constant apart from the one
    // field under test.  A real Mac OS mode switch does not: picking
    // "Millions" at 832x624 moves base, stride, hres, vres AND depth, in that
    // many separate CPU register writes, spread over an unknown number of
    // frames.  The states this module actually has to survive are the
    // INTERMEDIATE ones, where only some of those writes have landed.
    //
    // THE INVARIANT.  scanout_placement_sync exists so the scanner is never
    // handed a placement it cannot render -- that is the entire stated
    // purpose of its stride/range/depth gate, and the reason the header
    // documents a measured hardware black screen from violating it.  So the
    // check is not an invented one: every tuple this module emits must
    // satisfy scanout_fetch.v's OWN admission gate,
    //     fetch_stride_sane  = fetch_stride_px > row_last_off
    //     fetch_frame_in_mem = base + stride*row_y_last + row_last_off < LIMIT
    // (rtl/board/video_phy/scanout_fetch.v:190-192), because that gate is
    // what decides whether the fetch engine issues a single request.  When it
    // is false `can_request` is false for the whole frame: no requests, no
    // ring fill, no valid lines, black screen.
    //
    // tr_renderable() below is that gate, transcribed.
    run_transition_scenarios();

    std::printf("-- tb_scanout_placement_sync: %d PASS / %d FAIL --\n",
                n_pass, n_fail);

    dut->final();
    delete dut;
    return n_fail == 0 ? 0 : 1;
}
