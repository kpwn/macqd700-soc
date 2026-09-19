// video_top.v -- HDMI video pipeline top-level for m68k-ooo.
// ---------------------------------------------------------------------------
// Stitches together mmcm_hdmi + vtg + scaler + fb_reader + i2c_init into
// one self-contained module.  Replaces the BRAM-backed framebuf.v path
// from ~/sd-hdmi-bringup with a streaming read port that the `vram`
// module (agent/vram, URAM-backed) owns.
//
// Pipeline:
//     clk_ref_p/n or external single-ended ref --(mmcm_hdmi)--> pclk (148.5 MHz)
//                                               |
//        .---- vtg (1080p60 timing) ------------+
//        |           |
//        |           +---> hcount/vcount/de/hs/vs
//        v           v
//     i2c_init linebuf_scanout --(fb_reader CDC)--> vram.rd_{en,addr}
//     |              ^                       vram.rd_{data,valid}
//     v              |
//     AL9134 I2C     +-- scaler expands RGB / applies letterbox borders
//                    v
//                 AL9134 RGB / DE / HS / VS  + forwarded pclk
//
// The scaler accepts framebuffer base/stride and CLUT inputs so the scan-out
// path can be pointed at the DAFB-programmed framebuffer with indexed colour.
// Base/stride come from the core clock domain; video_top samples them into
// pclk only after two consecutive synchronized values agree, then commits
// the pair on a frame boundary so live register writes cannot alter the
// address equation mid-frame.  The 256-entry RAMDAC CLUT is different: it
// lives in linebuf_scanout.v as a true dual-clock BRAM fed directly by
// video.v's clut_we/clut_waddr/clut_wdata write export (vram_clk domain,
// no synchroniser, no frame-boundary gating) -- a mid-frame palette write
// takes effect immediately, matching real RAMDAC behaviour.  See
// linebuf_scanout.v's header for the full rationale.
//
// If TEST_PATTERN=1, scan-out bypasses the scaler/fb_reader data path and
// emits pclk-domain colour bars directly.  If TEST_PATTERN=2, it emits a
// checkerboard.  Those modes are intended for first board bring-up where
// HDMI clocking/pinout must be proven before trusting the VRAM-to-pixel
// CDC path.
//
// Scan-out resolution: 1024x768 source -> 1920x1080 output via Bresenham
// nearest-neighbour letterbox (see scaler.v header).
//
// Clock + reset strategy:
//   - mmcm_hdmi owns its own IBUFDS and BUFG; no external clock buffer
//     required.  Single clock domain downstream of BUFG (pclk).
//   - Internal async-assert / sync-release reset pipe: dropped on MMCM
//     unlock, released 4 pclk after lock.
//   - HDMI does not stall on any CPU-side event; it runs free on pclk.
//     The line-buffered scanout path requests source pixels once into rolling
//     pclk-side line buffers, then reuses those pixels for scaled repeats.
//     The VRAM request/data stream crosses through fb_reader's bounded async
//     FIFOs using the explicit vram_clk/vram_rst inputs.  The caller must wire
//     these to the clock/reset that drive the connected VRAM streaming read
//     port.  If the core/VRAM side cannot keep up, visible pixels blank rather
//     than replay stale data; Verilator tests treat FIFO overflow as fatal.
//
// Init_done contract:
//   `hdmi_i2c_done` is exported upstream so clk_rst.v can AND it with
//   the DDR / ROM-loaded gates before releasing CPU reset.  This matches
//   the spec in docs/peripheral_arch.md "init_done sources".
// ---------------------------------------------------------------------------
`default_nettype none

module video_top #(
    // Source framebuffer resolution.  Default 1024x768 (4:3 Mac
    // resolution that fits comfortably in 768 KB of URAM at 8bpp).
    parameter SRC_W           = 1024,
    parameter SRC_H           = 768,
    // Width of the VRAM streaming read port.  That port is byte-ADDRESSED but
    // returns a 4-BYTE GROUP per request at every pixel depth (vram.v and
    // scanout_ddr_reader.v both present that contract); the runtime depth
    // arrives on scanout_bytes_per_px instead.  32 is the only supported
    // value -- linebuf_scanout.v hard-errors otherwise.
    // (Was `PIX_BPP`=8 when the port returned a single byte, and before that
    // 24, a leftover from the pre-runtime-depth "PIX_BPP==24 selects direct
    // colour at elaboration" scheme, which could not also serve the 8bpp
    // screen the ROM programs at boot.)
    parameter FETCH_W         = 32,
    parameter ADDR_W          = 20,   // covers 1024*768
    parameter FB_MAX_PIXELS   = SRC_W * SRC_H,
    // HDMI timing and scaler geometry.  Override these together to retarget
    // the VRAM->HDMI path (for example 720p bring-up or a different Mac mode)
    // without editing scaler.v/video_top.v internals.
    parameter DST_W           = 1920,
    parameter DST_H           = 1080,
    parameter H_FP            = 88,
    parameter H_SYNC          = 44,
    parameter H_BP            = 148,
    parameter V_FP            = 4,
    parameter V_SYNC          = 5,
    parameter V_BP            = 36,
    parameter FB_BASE_PX      = 0,
    parameter FB_STRIDE_PX    = SRC_W,
    parameter real PCLK_DIVIDE_F = 10.000,
    // INTEGER twin of PCLK_DIVIDE_F, for the MMCM's CLKOUT1 (which has no
    // fractional divide).  ⚠️ MUST equal PCLK_DIVIDE_F exactly, or the clock
    // forwarded to the encoder runs at a DIFFERENT FREQUENCY from the data --
    // which presents as no sync at all, not as a subtle artefact.  Both are
    // set together in rtl/soc/fpga_top_video.vh; see the note in mmcm_hdmi.v
    // for why the VCO has to be chosen to make an integer divide possible.
    parameter integer PCLK_DIVIDE_I = 10,
    // Phase, in degrees, of the pixel clock FORWARDED OFF-CHIP relative to the
    // pixel clock that LAUNCHES the RGB data.  90 deg = a quarter period.
    // Why a quarter and not 0 or 180: see the ODDRE1 note at the bottom of
    // this file.  0.0 restores the pre-2026-09-15 behaviour exactly.
    parameter real PCLK_FWD_PHASE_DEG = 90.000,
    parameter real MMCM_CLKIN1_PERIOD = 5.000,
    parameter integer MMCM_DIVCLK_DIVIDE = 5,
    parameter real MMCM_CLKFBOUT_MULT_F = 37.125,
    parameter I2C_HALF        = 10'd744,
    parameter I2C_RESET_CYCLES = 24'd1485000,
    parameter I2C_INIT_WAIT   = 24'd1485000,
    // When 1, caller provides an already-buffered single-ended clock
    // on clk_ref_p; skip mmcm_hdmi's internal IBUFDS.  Needed when the
    // top-level design already buffers the ref-clock pin for another
    // consumer (see fpga_top.v u_sys_clk_ibufds).  clk_ref_n is ignored.
    parameter EXTERNAL_IBUFDS = 0,
    // Direct pclk-domain first-hardware HDMI smoke patterns.
    parameter TEST_PATTERN    = 0,
    // Boot-splash pixel replication factor (power of two): the 32x32
    // checkra1n logo shown centred until the DAFB commits a renderable
    // placement.  4 => 128x128 on screen, 8 => 256x256.
    //
    // 0 (the default) asks scanout_display.v to derive this from DST_H
    // instead of shipping a size tuned for one mode and never revisited --
    // see its auto_splash_scale.  Pass a nonzero value here to pin an
    // explicit size regardless of DST_H.
    parameter SPLASH_SCALE    = 0
) (
    // 200 MHz differential board reference
    input  wire        clk_ref_p,
    input  wire        clk_ref_n,

    // External active-low reset (e.g. board push-button)
    input  wire        ext_resetn,

    // Clock/reset for the connected VRAM streaming read port.  In the FPGA
    // top this is core_clk/core_rst; in simple tests it can equal pclk.
    input  wire        vram_clk,
    input  wire        vram_rst,

    // ── Coupled scan-out reset, vram_clk domain (task #194) ───────────
    // THE WHOLE SCAN-OUT CHAIN MUST RESET AS ONE.  It spans two clock
    // domains and, before this port existed, three different reset nets:
    //   linebuf_scanout  <- pclk  resetn_bank[1]   (HDMI MMCM LOCKED)
    //   fb_reader        <- pclk  resetn_bank[2] AND vram_clk vram_rst
    //   the VRAM read port (scanout_ddr_reader / vram.v) <- vram_rst only
    // fb_reader turns EITHER reset into a reset of BOTH its halves, so a
    // pclk-only event -- an HDMI MMCM relock, which happens on every JTAG
    // bitstream reload and on any ref-clock disturbance -- clears its
    // credit counter and both CDC FIFOs while the read port downstream
    // keeps every request it was already handed.  The read port then
    // answers those stale requests, IN ORDER, to a caller that has
    // forgotten them: from that moment the ordered response stream no
    // longer corresponds to the request stream, permanently.  Measured as
    // fb_reader's own "response FIFO overflow" assertion in
    // tb-fb-reader-ddr-chain's pclk_only_reset_midstream scenario.
    //
    // This output is the vram_clk-domain reset the caller MUST use for
    // whatever it connects to vram_rd_*: it is `vram_rst` OR'd with the
    // pclk-domain reset synchronised into vram_clk, i.e. exactly the term
    // fb_reader's own vram half uses.  Driving the read port from the raw
    // `vram_rst` instead re-opens the skew.
    output wire        vram_side_rst,

    // Exposed for upstream observability / debug
    output wire        pclk_out,
    output wire        mmcm_locked,
    output wire        hdmi_i2c_done,
    output wire        fb_underflow_sticky,
    output wire [15:0] fb_reader_req_count,
    output wire [15:0] fb_reader_rsp_count,
    output wire [15:0] fb_reader_miss_count,
    output wire [11:0] debug_hcount,
    output wire [10:0] debug_vcount,
    output wire [23:0] debug_rgb,
    output wire        debug_de,
    output wire        debug_hs,
    output wire        debug_vs,

    // ── Coherent scan-out debug snapshot (task #243) ──────────────────
    //
    // WHY THIS EXISTS, and why it is ONE bus rather than N probes:
    //
    // debug_hcount / debug_vcount / debug_rgb above are separate VIO
    // probes, and a JTAG VIO read samples each probe in its OWN
    // transaction -- seconds apart when driven from a script.  They can
    // therefore never be correlated: during the 2026-08-05 black-screen
    // hunt an rgb=ffffff sample was paired with an hcount from a
    // different read and the pairing was meaningless.  Everything below
    // is latched from ONE pclk edge into ONE bus, so a single vio-read
    // returns a self-consistent snapshot.
    //
    // It also exposes the two things that were invisible and are the
    // prime suspects for a black screen with a healthy fetch path:
    //   * dafb_live -- while 0, scanout_display substitutes the boot
    //     splash for the pixel path entirely.
    //   * the COMMITTED hres/vres/base/stride (post-resync, post-debounce
    //     placement values the display half actually uses).  vio_dafb_cfg
    //     shows only what the DAFB *register block* holds; if
    //     scanout_placement_sync commits geometry of 0x0 the active
    //     window is empty and nothing renders, and no existing probe
    //     could see that.
    //
    // And it SPLITS the underflow sticky, which fb_underflow_sticky ORs
    // together: bit 67 is the line-buffer (display-side ran dry) and bit
    // 66 is the fb_reader (fetch-side response FIFO).  One bit could
    // never separate a cause from its effect -- see the comment on
    // vio_fb_reader_stats in fpga_top_debug_vio.vh.
    output reg  [95:0] dbg_video_snap,
    // Committed placement addresses + the REJECTED tuple, same pclk edge as
    // dbg_video_snap.  Field map (see the assignment below):
    //   [159:128] rejected fb_base_px      [ 63:32] COMMITTED fb_base_px
    //   [127: 96] rejected fb_stride_px    [ 31: 0] COMMITTED fb_stride_px
    //   [ 95: 84] rejected hres_px
    //   [ 83: 72] rejected vres_px
    //   [ 71: 68] rejected reason (latched at the refusing frame boundary)
    //   [ 67]     placement_rejected_sticky
    //   [ 66: 64] spare
    // The COMMITTED pair keeps its old bit positions on purpose: an older
    // jtag_repl `video-status` decode still reads them correctly off the
    // widened probe.  docs/video_path_review.md S4.2.
    output reg [159:0] dbg_video_place,
    // Frame-start (vertical-blank entry) tick in pclk domain.
    // 1-cycle pulse on the first blanking pixel after the active image.
    // Drives the DAFB VBL → VIA1 CA1 chain after a pclk→pb_clk pulse
    // synchroniser.
    output wire        vbl_pulse_pclk,

    // AL9134 HDMI transmitter pins
    inout  wire        al9134_scl,
    inout  wire        al9134_sda,
    output reg  [23:0] al9134_d,
    output reg         al9134_hs,
    output reg         al9134_vs,
    output reg         al9134_de,
    output wire        al9134_clk,
    output wire        al9134_resetn,
    input  wire        al9134_int,

    // DAFB-programmed framebuffer placement, expressed in pixel addresses.
    // The pclk domain sanitizes a zero stride back to FB_STRIDE_PX.
    input  wire [ADDR_W-1:0] scanout_fb_base_px,
    input  wire [ADDR_W-1:0] scanout_fb_stride_px,
    input  wire [2:0]        scanout_bpp_shift,
    // VRAM bytes per source pixel (video.v fb_bytes_per_px): 0 sub-byte,
    // 1 = 8bpp, 4 = 24bpp direct colour.  Runtime, frame-boundary-committed.
    input  wire [2:0]        scanout_bytes_per_px,
    // 1 iff the scanner can render the DAFB-programmed depth.  Gates
    // placement commits inside scanout_placement_sync so an unmapped AC842
    // depth code retains the last good placement instead of scanning garbage.
    input  wire              scanout_depth_supported,
    // DAFB-decoded source dimensions (12-bit) — drive the runtime
    // active-window + scale select inside linebuf_scanout.
    input  wire [11:0]       scanout_hres,
    input  wire [11:0]       scanout_vres,
    // 256-entry RAMDAC CLUT write export from video.v (DAFB shim).  Lives
    // in the vram_clk domain (core_clk at the real fpga_top instantiation
    // site) -- wired straight into linebuf_scanout's dual-clock CLUT BRAM
    // write port below, no synchroniser (see linebuf_scanout.v header).
    input  wire               scanout_clut_we,
    input  wire [7:0]         scanout_clut_waddr,
    input  wire [23:0]        scanout_clut_wdata,

    // VRAM read port (goes to agent/vram's streaming port)
    output wire              vram_rd_en,
    output wire [ADDR_W-1:0] vram_rd_addr,
    input  wire [FETCH_W-1:0] vram_rd_data,
    input  wire              vram_rd_valid
);

    // ── Clocking ─────────────────────────────────────────────────────
    wire pclk;
    // Same frequency as pclk, shifted by PCLK_FWD_PHASE_DEG.  Drives ONLY the
    // ODDRE1 that forwards the pixel clock to the encoder -- never any logic.
    wire pclk_fwd;
    assign pclk_out = pclk;

    mmcm_hdmi #(
        .CLKIN1_PERIOD     (MMCM_CLKIN1_PERIOD),
        .DIVCLK_DIVIDE     (MMCM_DIVCLK_DIVIDE),
        .CLKFBOUT_MULT_F   (MMCM_CLKFBOUT_MULT_F),
        .CLKOUT0_DIVIDE_F  (PCLK_DIVIDE_F),
        .CLKOUT1_DIVIDE_I  (PCLK_DIVIDE_I),
        .CLKOUT1_PHASE_DEG (PCLK_FWD_PHASE_DEG),
        .EXTERNAL_IBUFDS   (EXTERNAL_IBUFDS)
    ) u_mmcm (
        .clk_in_p (clk_ref_p),
        .clk_in_n (clk_ref_n),
        .resetn   (ext_resetn),
        .pclk     (pclk),
        .pclk_fwd (pclk_fwd),
        .locked   (mmcm_locked)
    );

    // Async-assert / sync-release reset pipe in pclk domain.  Dropped
    // while MMCM isn't locked; released 4 cycles after lock.
    reg [3:0] rst_pipe = 4'hF;
    wire      rst    = rst_pipe[3];
    wire      resetn = ~rst;

    always @(posedge pclk or negedge mmcm_locked) begin
        if (~mmcm_locked) rst_pipe <= 4'hF;
        else              rst_pipe <= {rst_pipe[2:0], 1'b0};
    end

    // vram_clk-domain reset, synchronised into pclk.  Folded into rst_bank
    // below so the whole pclk domain resets with it -- see that block.
    (* ASYNC_REG = "TRUE" *) reg [1:0] vram_rst_pclk_sync;
    always @(posedge pclk or negedge mmcm_locked) begin
        if (~mmcm_locked) vram_rst_pclk_sync <= 2'b11;
        else              vram_rst_pclk_sync <= {vram_rst_pclk_sync[0], vram_rst};
    end

    // ─────────────────────────────────────────────────────────────────────
    // Hand-replicated broadcast bank for `rst` (post-vivado_main_1d2af15
    // BUFG-insertion fix).  The placer log showed `u_video/rst` driving
    // 1660 sinks at 100 MHz, forcing a clock-tree BUFG.  We bank rst into
    // 4 explicit FFs marked `keep` + `dont_touch` + `MAX_FANOUT` so each
    // bit drives ~415 sinks — comfortably below the BUFG threshold.
    // Sim-neutral: every bit carries the identical Q value as `rst` one
    // pclk cycle later.  Internal consumers below take one bit per
    // logical region.  Reset assertion is async via `mmcm_locked` (same
    // event that asserts `rst_pipe`), so consumers see assert in the
    // same cycle as `rst`.
    (* keep = "true" *) (* dont_touch = "true" *)
    (* MAX_FANOUT = 256, ASYNC_REG = "TRUE" *)
    reg [3:0] rst_bank;
    always @(posedge pclk or negedge mmcm_locked) begin
        if (~mmcm_locked) rst_bank <= 4'b1111;
        else              rst_bank <= {4{rst | vram_rst_pclk_sync[1]}};
    end
    wire [3:0] resetn_bank = ~rst_bank;

    // ─────────────────────────────────────────────────────────────────────
    // COUPLED SCAN-OUT RESET (task #194).  ONE event, BOTH domains.
    //
    // `vram_rst` is folded into rst_bank ABOVE rather than OR-ed into
    // individual consumers below.  That is deliberate and it is a FANOUT
    // decision as much as a correctness one: rst_bank exists because the
    // placer measured `u_video/rst` driving 1660 sinks and inserted a BUFG
    // (see its own comment).  OR-ing a second term into each consumer would
    // create a fresh high-fanout combinational net per consumer and undo
    // that.  Folding it in at the source keeps exactly the same four banked
    // FFs, the same MAX_FANOUT, and leaves video_top with ONE pclk reset --
    // which is also the strongest statement of the property: there is no
    // longer a wiring in which part of the pclk domain resets and part does
    // not.
    //
    // In production this changes nothing for the VTG or the placement
    // stage, because the only thing that drives `vram_rst` is
    // soc_full_rst_bank[0], and fpga_top_video.vh feeds that same bit into
    // `video_mmcm_resetn` -- so the MMCM is already being reset, pclk is
    // already stopping, and rst_bank was already going to assert.  What the
    // fold buys is that the property no longer DEPENDS on that coincidence.
    //
    // Each synchroniser is fed from the OTHER domain's RAW reset, never
    // from the coupled result: coupled -> coupled would be a four-FF ring
    // that could hold itself asserted after both sources released.
    //
    //   rst_bank        = pclk rst OR (vram_rst synced into pclk)
    //   vram_side_rst   = vram_rst OR (pclk rst synced into vram_clk)
    //
    // The vram half is EXPORTED because the caller owns the VRAM read port
    // (scanout_ddr_reader under VRAM_IN_DDR, vram.v otherwise) and that port
    // holds the other half of the in-order request/response contract
    // fb_reader's credit counter tracks.  Resetting one without the other
    // desynchronises them permanently -- see this module's vram_side_rst
    // port comment.
    (* ASYNC_REG = "TRUE" *) reg [1:0] pclk_rst_vram_sync;
    always @(posedge vram_clk) begin
        if (vram_rst) pclk_rst_vram_sync <= 2'b11;
        // rst_bank[2] is the fb_reader/CDC bank -- a real FF, and one that
        // is async-SET by ~mmcm_locked, so while pclk is stopped it holds 1
        // and this reset stays asserted.  Sampling the combinational `rst`
        // instead would work too but would put a pclk-domain LUT on a
        // cross-domain path for no gain.
        else          pclk_rst_vram_sync <= {pclk_rst_vram_sync[0], rst_bank[2]};
    end
    assign vram_side_rst = vram_rst | pclk_rst_vram_sync[1];


    // ── Video timing generator (1920x1080p60) ─────────────────────────
    wire [11:0] hcount;
    wire [10:0] vcount;
    wire        de_vtg, hs_vtg, vs_vtg;

    vtg #(
        .H_ACTIVE (DST_W), .H_FP (H_FP), .H_SYNC (H_SYNC), .H_BP (H_BP),
        .V_ACTIVE (DST_H), .V_FP (V_FP), .V_SYNC (V_SYNC), .V_BP (V_BP)
    ) u_vtg (
        .pclk   (pclk),
        // rst-bank [0] (timing-generator + CLUT region).
        .resetn (resetn_bank[0]),
        .hcount (hcount),
        .vcount (vcount),
        .de     (de_vtg),
        .hsync  (hs_vtg),
        .vsync  (vs_vtg)
    );

    // ── VBL pulse (start of vertical-blank period) ─────────────────────
    // MAME's DAFB device asserts VBL on `screen->time_until_pos(480, 0)`:
    // x=0 on the first scanline beyond the active region.  Generate the
    // equivalent event in the HDMI VTG domain instead of waiting for VSYNC,
    // which starts after the vertical front porch.
    localparam [10:0] VBL_START_LINE = DST_H;
    assign vbl_pulse_pclk = !rst_bank[0] &&
                             (hcount == 12'd0) &&
                             (vcount == VBL_START_LINE);

    (* DONT_TOUCH = "true", KEEP = "true" *) reg [11:0] debug_hcount_r;
    (* DONT_TOUCH = "true", KEEP = "true" *) reg [10:0] debug_vcount_r;
    (* DONT_TOUCH = "true", KEEP = "true" *) reg [23:0] debug_rgb_r;
    (* DONT_TOUCH = "true", KEEP = "true" *) reg        debug_de_r;
    (* DONT_TOUCH = "true", KEEP = "true" *) reg        debug_hs_r;
    (* DONT_TOUCH = "true", KEEP = "true" *) reg        debug_vs_r;

    assign debug_hcount = debug_hcount_r;
    assign debug_vcount = debug_vcount_r;
    assign debug_rgb    = debug_rgb_r;
    assign debug_de     = debug_de_r;
    assign debug_hs     = debug_hs_r;
    assign debug_vs     = debug_vs_r;

    // ── Scaler + fb_reader + VRAM port ────────────────────────────────
    wire                 sc_rd_en;
    wire [ADDR_W-1:0]    sc_rd_addr;
    wire [FETCH_W-1:0]   sc_rd_data;
    wire                 sc_rd_ready;
    wire                 sc_rd_valid;
    wire                 sc_underflow_sticky;

    wire [23:0]          sc_rgb;
    wire                 sc_de;
    wire                 sc_hs;
    wire                 sc_vs;

    localparam FB_RETURN_LATENCY = 31;
    wire [ADDR_W-1:0] fb_base_px;
    wire [ADDR_W-1:0] fb_stride_px;
    wire [2:0]        bpp_shift;
    wire [2:0]        bytes_per_px;
    wire [11:0]       hres;
    wire [11:0]       vres;
    wire [2:0]        scale_n;
    // Loud-failure channel (docs/video_path_review.md S4.2): why the last
    // candidate placement was refused, a sticky "refused since the last
    // commit" bit, and the tuple that was refused.  Observation only -- no
    // scan-out behaviour is conditioned on any of it.
    wire [3:0]        place_reject_reason;
    wire              place_rejected_sticky;
    wire [ADDR_W-1:0] place_reject_base_px;
    wire [ADDR_W-1:0] place_reject_stride_px;
    wire [11:0]       place_reject_hres_px;
    wire [11:0]       place_reject_vres_px;
    wire [3:0]        place_reject_reason_latched;
    // "the DAFB has started feeding real video" -- sticky, and only ever
    // changes ON frame_start, so the boot-splash switchover is frame-aligned.
    wire              dafb_live;
    wire frame_start = (hcount == 12'd0) && (vcount == 11'd0);

    scanout_placement_sync #(
        .ADDR_W       (ADDR_W),
        .SRC_W        (SRC_W),
        .SRC_H        (SRC_H),
        .FB_MAX_PIXELS(FB_MAX_PIXELS),
        .FB_BASE_PX   (FB_BASE_PX),
        .FB_STRIDE_PX (FB_STRIDE_PX),
        .DST_W        (DST_W),
        .DST_H        (DST_H)
    ) u_scanout_placement_sync (
        .pclk                 (pclk),
        // rst-bank [0] (timing + CLUT region).
        .rst                  (rst_bank[0]),
        .frame_start          (frame_start),
        .scanout_fb_base_px   (scanout_fb_base_px),
        .scanout_fb_stride_px (scanout_fb_stride_px),
        .scanout_bpp_shift    (scanout_bpp_shift),
        .scanout_bytes_per_px (scanout_bytes_per_px),
        .scanout_depth_supported (scanout_depth_supported),
        .scanout_hres         (scanout_hres),
        .scanout_vres         (scanout_vres),
        .fb_base_px           (fb_base_px),
        .fb_stride_px         (fb_stride_px),
        .bpp_shift            (bpp_shift),
        .bytes_per_px         (bytes_per_px),
        .hres                 (hres),
        .vres                 (vres),
        .scale_n              (scale_n),
        .reject_reason             (place_reject_reason),
        .placement_rejected_sticky (place_rejected_sticky),
        .reject_base_px            (place_reject_base_px),
        .reject_stride_px          (place_reject_stride_px),
        .reject_hres_px            (place_reject_hres_px),
        .reject_vres_px            (place_reject_vres_px),
        .reject_reason_latched     (place_reject_reason_latched),
        .dafb_live            (dafb_live)
    );

    linebuf_scanout #(
        .SRC_W           (SRC_W),
        .SRC_H           (SRC_H),
        .FB_MAX_PIXELS   (FB_MAX_PIXELS),
        .DST_W           (DST_W),
        .DST_H           (DST_H),
        .FETCH_W         (FETCH_W),
        .ADDR_W          (ADDR_W),
        .LINE_COUNT_LOG2 (6),
        .SPLASH_SCALE    (SPLASH_SCALE)
    ) u_scanout (
        .pclk        (pclk),
        // rst-bank [1] (line-buffer / scaler -- largest internal sink).
        // The bank now carries the vram_rst coupling (task #194), so the
        // ring's walk position cannot survive a reset that emptied the
        // fetch path underneath it.
        .resetn      (resetn_bank[1]),
        .hcount      (hcount),
        .vcount      (vcount),
        .de_in       (de_vtg),
        .hs_in       (hs_vtg),
        .vs_in       (vs_vtg),
        .fb_base_px  (fb_base_px),
        .fb_stride_px(fb_stride_px),
        .bpp_shift   (bpp_shift),
        .bytes_per_px(bytes_per_px),
        .hres        (hres),
        .vres        (vres),
        .scale_n     (scale_n),
        .dafb_live   (dafb_live),
        // CLUT write port lives in the vram_clk domain (core_clk at the
        // real fpga_top instantiation site) -- no synchroniser, see
        // linebuf_scanout.v header.
        .clut_wclk   (vram_clk),
        .clut_we     (scanout_clut_we),
        .clut_waddr  (scanout_clut_waddr),
        .clut_wdata  (scanout_clut_wdata),
        .fb_rd_en    (sc_rd_en),
        .fb_rd_addr  (sc_rd_addr),
        .fb_rd_ready ((TEST_PATTERN != 0) ? 1'b0 : sc_rd_ready),
        .fb_rd_data  (sc_rd_data),
        .fb_rd_valid (sc_rd_valid),
        .rgb         (sc_rgb),
        .de_out      (sc_de),
        .hs_out      (sc_hs),
        .vs_out      (sc_vs),
        .line_underflow_sticky(sc_underflow_sticky)
    );

    wire              fb_vram_rd_en;
    wire [ADDR_W-1:0] fb_vram_rd_addr;
    wire              sc_rd_en_cdc = (TEST_PATTERN != 0) ? 1'b0 : sc_rd_en;
    wire              fb_reader_underflow_sticky;
    wire [15:0]       sc_fb_req_count;
    wire [15:0]       sc_fb_rsp_count;
    wire [15:0]       sc_fb_miss_count;

    fb_reader #(
        .ADDR_W        (ADDR_W),
        .DATA_W        (FETCH_W),
        .RETURN_LATENCY(FB_RETURN_LATENCY)
    ) u_fb_reader (
        .pclk       (pclk),
        // rst-bank [2] (CDC / DMA-fetch state machine region), which now
        // carries the vram_rst coupling, plus the coupled vram-side reset
        // (task #194).  fb_reader still derives its own internal coupling
        // from these two; feeding it the already-coupled pair makes that
        // idempotent rather than a second, independent opinion about when
        // the chain is in reset.
        .resetn     (resetn_bank[2]),
        .vram_clk   (vram_clk),
        .vram_rst   (vram_side_rst),
        .s_rd_en    (sc_rd_en_cdc),
        .s_rd_addr  (sc_rd_addr),
        .s_rd_ready (sc_rd_ready),
        .s_rd_data  (sc_rd_data),
        .s_rd_valid (sc_rd_valid),
        .v_rd_en    (fb_vram_rd_en),
        .v_rd_addr  (fb_vram_rd_addr),
        .v_rd_data  (vram_rd_data),
        .v_rd_valid (vram_rd_valid),
        .underflow_sticky(fb_reader_underflow_sticky),
        .req_count  (sc_fb_req_count),
        .rsp_count  (sc_fb_rsp_count),
        .miss_count (sc_fb_miss_count)
    );

    assign fb_underflow_sticky = sc_underflow_sticky | fb_reader_underflow_sticky;
    assign fb_reader_req_count  = sc_fb_req_count;
    assign fb_reader_rsp_count  = sc_fb_rsp_count;
    assign fb_reader_miss_count = sc_fb_miss_count;

    // ── Coherent scan-out debug snapshot (task #243) ──────────────────
    // Every field below is captured on the SAME pclk edge, so one
    // vio-read of dbg_video_snap + dbg_video_place is self-consistent.
    // See the port declaration for why that matters.
    //
    // N-1, clamped to [0,3] so an out-of-range N cannot wrap the field.
    wire [1:0] scale_n_m1 = (scale_n >= 3'd4) ? 2'd3
                          : (scale_n <= 3'd1) ? 2'd0
                                              : (scale_n[1:0] - 2'd1);

    // No reset on the snapshot registers: this is pure observation, and
    // holding them in reset would hide exactly the post-reset window we most
    // want to look at.
    always @(posedge pclk) begin
        dbg_video_snap <= {
            hcount,                      // [95:84] 12
            vcount,                      // [83:73] 11
            sc_de,                       // [72]
            dafb_live,                   // [71]     splash-substitution gate
            sc_rd_en,                    // [70]
            sc_rd_valid,                 // [69]
            sc_rd_ready,                 // [68]
            sc_underflow_sticky,         // [67]     line-buffer ran dry
            fb_reader_underflow_sticky,  // [66]     fetch-side FIFO
            // COMMITTED integer scale, encoded as N-1 so it still fits the
            // two bits the old scale_sel enum occupied.  N is 1..4, so
            // 0=1:1 and 1=2:1 read the same as the old encoding did for the
            // only two rungs that survived the integer-only policy.
            scale_n_m1,                  // [65:64]  2  COMMITTED N-1
            sc_rgb,                      // [63:40] 24
            bpp_shift,                   // [39:37]  3
            bytes_per_px,                // [36:34]  3
            hres,                        // [33:22] 12  COMMITTED
            vres,                        // [21:10] 12  COMMITTED
            place_reject_reason,         // [9:6]    4  LIVE candidate verdict
            6'd0                         // [5:0]   spare
        };
        dbg_video_place <= {
            {(32-ADDR_W){1'b0}}, place_reject_base_px,   // [159:128] REJECTED
            {(32-ADDR_W){1'b0}}, place_reject_stride_px, // [127: 96] REJECTED
            place_reject_hres_px,                        // [ 95: 84] REJECTED
            place_reject_vres_px,                        // [ 83: 72] REJECTED
            place_reject_reason_latched,                 // [ 71: 68]
            place_rejected_sticky,                       // [ 67]
            3'd0,                                        // [ 66: 64] spare
            {(32-ADDR_W){1'b0}}, fb_base_px,    // [63:32] COMMITTED
            {(32-ADDR_W){1'b0}}, fb_stride_px   // [31:0]  COMMITTED
        };
    end

    assign vram_rd_en   = (TEST_PATTERN != 0) ? 1'b0 : fb_vram_rd_en;
    assign vram_rd_addr = (TEST_PATTERN != 0) ? {ADDR_W{1'b0}} : fb_vram_rd_addr;

    // First-board smoke patterns generated entirely in the HDMI pixel
    // clock domain.  They prove pclk, sync, DE, data pins, and AL9134 init
    // independently of framebuffer contents and CDC plumbing.
    reg [23:0] test_rgb;
    always @(*) begin
        if (!de_vtg) begin
            test_rgb = 24'h00_00_00;
        end else if (TEST_PATTERN == 2) begin
            // 32x32 tiles keep the checkerboard large enough to see on
            // first light while still exercising both axes across the frame.
            test_rgb = (hcount[5] ^ vcount[5]) ? 24'h40_40_40
                                               : 24'hff_ff_ff;
        end else if (hcount < (DST_W * 1) / 8) begin
            test_rgb = 24'hff_ff_ff;
        end else if (hcount < (DST_W * 2) / 8) begin
            test_rgb = 24'hff_ff_00;
        end else if (hcount < (DST_W * 3) / 8) begin
            test_rgb = 24'h00_ff_ff;
        end else if (hcount < (DST_W * 4) / 8) begin
            test_rgb = 24'h00_ff_00;
        end else if (hcount < (DST_W * 5) / 8) begin
            test_rgb = 24'hff_00_ff;
        end else if (hcount < (DST_W * 6) / 8) begin
            test_rgb = 24'hff_00_00;
        end else if (hcount < (DST_W * 7) / 8) begin
            test_rgb = 24'h00_00_ff;
        end else begin
            test_rgb = 24'h40_40_40;
        end
    end

    wire [23:0] al9134_d_next  = (TEST_PATTERN != 0) ? test_rgb : sc_rgb;
    wire        al9134_de_next = (TEST_PATTERN != 0) ? de_vtg   : sc_de;
    wire        al9134_hs_next = (TEST_PATTERN != 0) ? hs_vtg   : sc_hs;
    wire        al9134_vs_next = (TEST_PATTERN != 0) ? vs_vtg   : sc_vs;

    reg [23:0] al9134_d_pipe;
    reg        al9134_de_pipe;
    reg        al9134_hs_pipe;
    reg        al9134_vs_pipe;
    reg [11:0] al9134_hcount_pipe;
    reg [10:0] al9134_vcount_pipe;

    always @(posedge pclk) begin
        // rst-bank [3] (HDMI output FFs + i2c region).
        if (rst_bank[3]) begin
            al9134_d_pipe  <= 24'h00_00_00;
            al9134_de_pipe <= 1'b0;
            al9134_hs_pipe <= 1'b0;
            al9134_vs_pipe <= 1'b0;
            al9134_hcount_pipe <= 12'd0;
            al9134_vcount_pipe <= 11'd0;
            al9134_d  <= 24'h00_00_00;
            al9134_de <= 1'b0;
            al9134_hs <= 1'b0;
            al9134_vs <= 1'b0;
            debug_hcount_r <= 12'd0;
            debug_vcount_r <= 11'd0;
            debug_rgb_r <= 24'h00_00_00;
            debug_de_r <= 1'b0;
            debug_hs_r <= 1'b0;
            debug_vs_r <= 1'b0;
        end else begin
            al9134_d_pipe  <= al9134_d_next;
            al9134_de_pipe <= al9134_de_next;
            al9134_hs_pipe <= al9134_hs_next;
            al9134_vs_pipe <= al9134_vs_next;
            al9134_hcount_pipe <= hcount;
            al9134_vcount_pipe <= vcount;

            al9134_d  <= al9134_d_pipe;
            al9134_de <= al9134_de_pipe;
            al9134_hs <= al9134_hs_pipe;
            al9134_vs <= al9134_vs_pipe;
            debug_hcount_r <= al9134_hcount_pipe;
            debug_vcount_r <= al9134_vcount_pipe;
            debug_rgb_r <= al9134_d_pipe;
            debug_de_r <= al9134_de_pipe;
            debug_hs_r <= al9134_hs_pipe;
            debug_vs_r <= al9134_vs_pipe;
        end
    end

    // ── SiI9134 I2C init sequencer ────────────────────────────────────
    // At pclk = 148.5 MHz: I2C_HALF = 744 -> SCL ~ 99.8 kHz.
    // Reset low & post-reset wait = 10 ms each = 1,485,000 cycles.
    wire i2c_scl_oe, i2c_sda_oe, i2c_chip_rstn;

    i2c_init #(
        .I2C_HALF     (I2C_HALF),
        .RESET_CYCLES (I2C_RESET_CYCLES),
        .INIT_WAIT    (I2C_INIT_WAIT)
    ) u_i2c (
        .clk         (pclk),
        // rst-bank [3] (HDMI output FFs + i2c region).
        .resetn      (resetn_bank[3]),
        .scl_oe      (i2c_scl_oe),
        .sda_oe      (i2c_sda_oe),
        .sda_i       (al9134_sda),
        .chip_rstn   (i2c_chip_rstn),
        .done        (hdmi_i2c_done),
        .ack_count   (),
        .nak_count   (),
        .dbg_state   (),
        .dbg_rom_idx ()
    );

    // Open-drain I2C.  On real silicon the pads drive low or float
    // (board pull-ups restore high).  Under Verilator inout + 1'bz is
    // error-prone -- we drive 1'b1 when released so the tb harness can
    // observe the line as an ordinary wire.  The dedicated scl_oe /
    // sda_oe strobes are pulled out hierarchically in tb_video_top.v
    // for functional verification of the init sequence.
`ifdef VERILATOR
    assign al9134_scl    = i2c_scl_oe ? 1'b0 : 1'b1;
    assign al9134_sda    = i2c_sda_oe ? 1'b0 : 1'b1;
`else
    assign al9134_scl    = i2c_scl_oe ? 1'b0 : 1'bz;
    assign al9134_sda    = i2c_sda_oe ? 1'b0 : 1'bz;
`endif
    assign al9134_resetn = i2c_chip_rstn;

    // ── Pixel clock forwarded to the AL9134 via ODDRE1 (2026-09-13) ────
    //
    // WAS: `assign al9134_clk = pclk;` -- a BUFG-distributed clock driving an
    // OBUF directly. That was forced by KU5P HDIOLOGIC's 8 ns min-period
    // (125 MHz), which 148.5 MHz violated. **That objection is gone**: the
    // pixel clock is now 99.0 MHz (10.101 ns), comfortably inside the rating.
    // See the 1280x960@60 note in rtl/soc/fpga_top_video.vh.
    //
    // WHY IT MATTERS. Driving the clock from fabric means the launch flops see
    // the FULL clock-network insertion delay while the forwarded clock does
    // not traverse a matched path, so data-vs-clock skew at the pins is
    // whatever placement happens to give -- and hdmi.xdc records the
    // consequence: "Clock Path Skew -2.299ns ... 7 al9134_d[*] endpoints
    // failing". An ODDRE1 in the IOB launches the clock from the SAME column
    // logic as the data flops (now IOB TRUE, see hdmi.xdc), so clock and data
    // traverse matched I/O paths and the skew is a device constant rather than
    // a placement lottery. This is the actual source-synchronous structure the
    // interface always wanted.
    //
    // ── WHY THE FORWARDED CLOCK IS SHIFTED A QUARTER PERIOD (2026-09-15) ──
    //
    // The ODDR is clocked by `pclk_fwd` (MMCM CLKOUT1, +90 deg), NOT by `pclk`.
    // The RGB flops still launch on `pclk` rising. So the forwarded clock's
    // rising edge lands a quarter period AFTER the data changes, and its
    // falling edge lands three quarters after. BOTH edges are ~3.4 ns clear of
    // every data transition at 74.25 MHz.
    //
    // MEASURED, on build/vivado200_ra route.dcp (report_datasheet, xcku5p-2-i):
    //
    //            pad delay from the same launch edge     SLOW      FAST
    //   al9134_d[*] / de / hs / vs                   8.930-8.973  4.759-4.805
    //   al9134_clk  (rising edge of the forward)        9.629       4.523
    //
    //   => clock_rise - data = +0.656 .. +0.699 ns (SLOW)
    //                          -0.236 .. -0.282 ns (FAST)
    //
    // That is the whole defect in two lines. With the ODDR clocked by `pclk`
    // IN PHASE, the forwarded clock's rising edge sits ON the data transition,
    // within a ~1 ns window that CHANGES SIGN with process corner. Vivado's own
    // bus-skew figure for the 24-bit bus is 0.043 ns, so per-bit skew is 20x
    // smaller than that offset: skew does not cause the slip, it just decides
    // WHICH bits land on the late side of an edge that is already sitting on
    // the transition. Result: a stable, per-channel, one-pixel displacement --
    // exactly what the board shows at 720p60, where N=1 makes a one-pixel slip
    // a whole source pixel instead of a half-masked doubled one.
    //
    // WHY A QUARTER AND NOT A HALF. A half period (the old D1=0/D2=1 inversion,
    // or equivalently +180 deg) moves the RISING edge mid-eye but drops the
    // FALLING edge onto the transition. Which of those two is fatal depends on
    // which edge the encoder captures on -- and that is NOT established; see
    // synth/hdmi.xdc and the register note in i2c_init.v for the three-way
    // contradiction. A quarter period is the only placement that clears BOTH
    // edges, so it is correct under either reading and does not require the
    // board to arbitrate first.
    //
    // MEASURED AFTER THE SHIFT, on build/vivado200_closure route.dcp (the first
    // real build carrying it), in the same pad frame:
    //   data transition at the pad     7.713 - 8.429 ns
    //   forwarded RISING edge at pad  10.702 - 11.787 ns
    //   => the rising edge lands 2.27 - 4.07 ns into the data eye, the falling
    //      edge 9.01 - 10.81 ns in. Against the data sheet's TSIDR/THIDR and
    //      TSIDF/THIDF that is +1.27 ns (rising setup), +8.90 ns (rising hold),
    //      +8.01 ns (falling setup), +1.86 ns (falling hold). All four positive.
    // Before the shift the nearest edge was ZERO ns from the transition.
    //
    // ⚠️ STA checks BOTH pairs, but only with the multicycle in synth/hdmi.xdc.
    // The generated clock models the rising edge at phase zero (the +90 deg is
    // carried as clock path DELAY, not waveform), so without
    // `set_multicycle_path 0 -setup -end -rise_to` the rising hold check
    // degenerates to launch@0/capture@0 and fails by -4.547 ns as a PHANTOM --
    // which is exactly what the first build reported. hdmi.xdc has the autopsy.
    //
    // ⚠️ THIS IS NOT A FLIP OF THE ODDR POLARITY. D1/D2 keep the values that
    // 5c9ac81a settled on from board evidence; only the CLOCK feeding the ODDR
    // moves. Nothing here contradicts that commit's observation -- it sidesteps
    // the question that commit could not settle.
    //
    // The old `assign al9134_clk = pclk` worked BY ACCIDENT for the same reason
    // this fix works on purpose: the clock went BUFG->OBUF while data went
    // flop->OBUF, so the clock led the data by roughly the clock-network
    // insertion delay (hdmi.xdc records "Clock Path Skew -2.299ns") -- about a
    // third of a period at 148.5 MHz, comfortably off the transition. IOB
    // packing plus the ODDR removed that accident and left zero offset; this
    // puts a defined offset back.
    //
    // NOTE: Vivado TRANSFORMS this ODDRE1 into an OSERDESE3 in HDIOLOGIC
    // (routed netlist: REF=OSERDESE3, LOC=HDIOLOGIC_S_X0Y43). Its clock pin is
    // `CLK`, not `C` -- synth/hdmi.xdc's create_generated_clock must match.
`ifdef VERILATOR
    // There is no ODDRE1 primitive in simulation, so forward the clock
    // behaviourally.  This mirrors the D1=1/D2=0 (IN-PHASE) hardware forward
    // below; if that ever goes back to D1=0/D2=1, this becomes ~pclk.
    //
    // >>> DO NOT start this comment with the simulator's name.  A `//` comment
    // >>> whose FIRST word is that name is parsed as a metacomment, and the
    // >>> elaboration error it produces (at this line, about an unknown
    // >>> metacomment) reads nothing like a comment problem -- it took
    // >>> tb-video, tb-video-pattern and tb-video-checkerboard out of the
    // >>> suite entirely until someone traced it back here.
    assign al9134_clk = pclk;
`else
    ODDRE1 #(
        .IS_C_INVERTED  (1'b0),
        .IS_D1_INVERTED (1'b0),
        .IS_D2_INVERTED (1'b0),
        .SRVAL          (1'b0)
    ) u_pclk_fwd (
        .Q  (al9134_clk),
        // pclk_fwd, NOT pclk: the +90 deg copy.  This one substitution is the
        // entire fix -- see the long note above.
        .C  (pclk_fwd),
        // D1=1/D2=0 forwards the ODDR's own clock IN PHASE.  That clock is now
        // pclk_fwd (+90 deg), so the pin rises a quarter period after the data
        // launch.  UNCHANGED from 5c9ac81a -- the polarity decision below stands,
        // the shift is carried entirely by which clock feeds .C above.
        // It was D1=0/D2=1 (inverted, edge at mid-period) from
        // 2026-09-13 until the board said otherwise:
        //   * 1080p60, forwarded NON-inverted by the old `assign al9134_clk = pclk`:
        //     no pixel displacement.
        //   * 1080p30, forwarded INVERTED: rock-solid sync, zero line glitches, but a
        //     STABLE one-pixel displacement on some channels -- worst on the desktop's
        //     checkerboard dither, where adjacent pixels differ maximally.
        // A one-pixel offset is 13.47 ns and per-bit skew measures 0.123 ns, so skew
        // cannot CAUSE the shift -- it only selects which bits fall on the late side of
        // a sampling edge sitting ON the data transition.  Inverting put the edge there;
        // this encoder wants the in-phase clock, which lands its sample mid-eye.
        //
        // ⚠️ 2026-09-15 CORRECTION to the last clause. Measured at the PADS
        // (report_datasheet on route.dcp), the in-phase forward does NOT land
        // mid-eye -- it lands ON the transition, 0.66 ns late at the slow corner
        // and 0.24 ns early at the fast one. Both polarities put SOME edge on the
        // transition; only a quarter-period shift clears both, which is why .C is
        // now pclk_fwd. Also: the per-bit skew is 0.043 ns on this build, not the
        // 0.123 ns quoted above (that figure mixed in the clock-path spread).
        .D1 (1'b1),
        .D2 (1'b0),
        .SR (1'b0)
    );
`endif

    // Silence unused-input warnings
    wire _unused = &{al9134_int, 1'b0};

endmodule
`default_nettype wire
