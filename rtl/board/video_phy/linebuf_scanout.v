// linebuf_scanout.v -- framebuffer scanout with source-line prefetch.
// ---------------------------------------------------------------------------
// This is the production VRAM scanout path for an indexed/direct-colour Mac
// framebuffer scaled to 1080p.  Source pixels are fetched once into a rolling
// ring of source-line buffers; the pclk-side scaler then reuses those buffered
// pixels for horizontal and vertical nearest-neighbour repeats.
//
// This file is now a WRAPPER.  The engine is two modules, split at the one
// clean seam the design has (producer / consumer of the line ring):
//
//   scanout_fetch.v    -- the credit-paced fetch walk, the ring's tag/valid
//                         state, the ordered request/response port and the
//                         line-memory WRITE port.  ONE FSM
//                         (IDLE/RESYNC/FILL/RUN), no "pending" flags.
//   scanout_display.v  -- the Bresenham scaler, letterbox border, line-memory
//                         READ port, dual-clock RAMDAC CLUT and the output
//                         pipeline.
//
// WHY THE REWRITE (2026-08-01).  The previous single 1,260-line module
// coordinated its fetch walk with nine interacting flags -- prefetch_active,
// restart_pending, placement_pending, placement_restart_pending, start_pending,
// awaiting_frame_start, frame_active, display_primed, restart_stall_frames --
// and gated a single request behind TEN terms.  Every bug found on hardware
// was an interaction BETWEEN those flags, never a logic error inside one:
//
//   * awaiting_frame_start suppressed the only term that ever released a ring
//     slot, so once all 64 slots were valid the fetcher parked at source row
//     63 forever (screen shows 64 rows, black below);
//   * the af8fe56 response-desync watchdog force-fired a re-arm one cycle
//     AFTER sof, and that re-arm re-armed awaiting_frame_start -- the very
//     flag it needed clear.  The mitigation fed the condition it was written
//     to break;
//   * fixing that re-arm then exposed the vblank race the flag existed to
//     prevent (the fetcher racing a whole frame into the ring during
//     blanking, leaving only the LAST 64 rows resident).
//
// Measured on hardware with CPU=stub -- a static framebuffer, so all
// frame-to-frame variation was scanout -- that was SIX distinct frames out of
// ten.  The replacement makes both failures structurally impossible instead of
// guarded against:
//
//   CREDITS.  The fetcher holds credits; the display returns one
//   UNCONDITIONALLY as it advances past each source row; a row is fetched iff
//   a credit is available.  At frame start the fetcher holds exactly
//   LINE_COUNT credits, so it fills the ring and stops -- it cannot race ahead
//   during vblank, by arithmetic.  Credits come back with no `line_tag <
//   y_src` comparison, so nothing can suppress them -- it cannot park.
//
//   UNCONDITIONAL HARD RESYNC AT FRAME START.  No wait on the walk draining;
//   that unbounded wait was the root of the original permanent wedge.  Stale
//   responses are counted and discarded (the port is in-order), inside a
//   window that is also time-bounded.  A lost response therefore corrupts at
//   most one frame and can never wedge -- which is why the af8fe56 watchdog is
//   DELETED rather than ported: there is no longer a wedge for it to break.
//
// The multi-frame gate for all of this is tb-scanout-frames.  A re-arm bug is
// invisible inside a single frame walk, which is how the original survived the
// rest of the video tb suite.
//
// FETCH PORT.  An ordered request/response stream.  fb_rd_ready throttles new
// requests so the pclk domain cannot overrun fb_reader's CDC request FIFO.
// fb_rd_en/fb_rd_addr are a registered valid/data pair: once asserted the
// address is held stable until fb_rd_ready accepts it.  fb_rd_data is the
// 4-BYTE GROUP at fb_rd_addr ([31:24] always valid at any alignment; the lower
// three lanes only when fb_rd_addr[1:0]==0).
//
// SCANOUT BANDWIDTH.  The binding resource is REQUEST SLOTS: the port accepts
// at most one request per core_clk (100 MHz).  At 60 Hz, as a fraction of that
// 100 M/s ceiling:
//                        indexed 8bpp    24bpp narrow    24bpp WIDE
//     640x480              18%              55%             18%
//     832x624              31%              94%             31%
//     1024x768             47%             142%             47%
// Narrow 24bpp at 1024x768 cannot be sustained, which is exactly why the wide
// path (one aligned 4-byte request per pixel) exists.  When the budget IS
// exceeded the module fails visibly (line_underflow_sticky), not silently.
// ---------------------------------------------------------------------------
`default_nettype none

module linebuf_scanout #(
    parameter SRC_W          = 1024,
    parameter SRC_H          = 768,
    parameter FB_MAX_PIXELS  = SRC_W * SRC_H,
    parameter DST_W          = 1920,
    parameter DST_H          = 1080,
    // Width of the fetch data port ONLY.  The streaming read port is byte-
    // ADDRESSED but returns a 4-byte group per request at every depth; the
    // runtime depth arrives on `bytes_per_px`.  32 is the only supported
    // value (scanout_fetch.v hard-errors otherwise).
    parameter FETCH_W        = 32,
    parameter ADDR_W         = 20,
    parameter LINE_COUNT_LOG2 = 6,
    // Boot-splash pixel replication factor (power of two).  Pure
    // pass-through to scanout_display.v, which owns the splash and, when
    // this is 0 (the default), derives it from DST_H instead of using a
    // size fixed for whatever mode was shipping when the constant was
    // chosen -- see scanout_display.v's auto_splash_scale.
    parameter SPLASH_SCALE   = 0
) (
    input  wire                 pclk,
    input  wire                 resetn,

    input  wire [11:0]          hcount,
    input  wire [10:0]          vcount,
    input  wire                 de_in,
    input  wire                 hs_in,
    input  wire                 vs_in,

    input  wire [ADDR_W-1:0]    fb_base_px,
    input  wire [ADDR_W-1:0]    fb_stride_px,
    // log2(source_pixels_per_byte): 1bpp=3, 2bpp=2, 4bpp=1, 8bpp=0.
    input  wire [2:0]           bpp_shift,
    // VRAM bytes per source pixel, RUNTIME (video.v's fb_bytes_per_px,
    // frame-boundary-committed by scanout_placement_sync):
    //     0 or 1 -> indexed, one byte per source-pixel step (1/2/4/8bpp)
    //     4      -> 24bpp direct colour, xRGB
    // Any other value is treated as indexed (fail-safe).
    input  wire [2:0]           bytes_per_px,
    // DAFB-driven visible source dimensions.  The active output window is
    // hres*N x vres*N for the committed integer scale N below; outside it,
    // black.
    input  wire [11:0]          hres,
    input  wire [11:0]          vres,
    // Committed INTEGER output scale: N display pixels per source pixel on
    // both axes (scanout_placement_sync commits it with the placement; the
    // policy itself lives in place_plan.v).  1..4, or 0 for "derive it from
    // hres/vres".  Consumed only by scanout_display -- the fetch walk works
    // in SOURCE rows/bytes and is scale-agnostic.
    input  wire [2:0]           scale_n,
    // 1 once the DAFB has committed a renderable placement (sticky,
    // frame-boundary-aligned -- scanout_placement_sync.v).  While 0,
    // scanout_display substitutes the boot splash for the pixel path.  It is
    // consumed ONLY by the display half; the fetch walk never sees it.
    input  wire                 dafb_live,

    // ── 256-entry RAMDAC CLUT write port (clut_wclk domain) ──────────
    input  wire                 clut_wclk,
    input  wire                 clut_we,
    input  wire [7:0]           clut_waddr,
    input  wire [23:0]          clut_wdata,

    output wire                 fb_rd_en,
    output wire [ADDR_W-1:0]    fb_rd_addr,
    input  wire                 fb_rd_ready,
    input  wire [FETCH_W-1:0]   fb_rd_data,
    input  wire                 fb_rd_valid,

    output wire [23:0]          rgb,
    output wire                 de_out,
    output wire                 hs_out,
    output wire                 vs_out,
    output wire                 line_underflow_sticky
);

    localparam LINE_MEM_AW = $clog2((1 << LINE_COUNT_LOG2) * SRC_W);

    // ── Frame boundary ───────────────────────────────────────────────
    // `sof` -- (0,0) -- resets the display's vertical Bresenham state.
    //
    // The RING RESYNC deliberately fires somewhere else: at the END of the
    // visible frame, i.e. the start of vertical blanking.  On the real VTG
    // (0,0) is the FIRST ACTIVE PIXEL, not a blanking cycle, so clearing the
    // ring there wipes it exactly as the display starts reading it.  With a
    // letterbox border that is invisible (the first real source row is not
    // needed for another ~60 display lines); at a geometry with NO border it
    // blacks out the top source row outright -- measured in
    // tb-framebuffer-pixel scenarios F/G/H (64x64 into 64x64), 64 mismatched
    // pixels, all of source row 0.
    //
    // Resyncing at end-of-visible-frame instead gives the fetcher the whole
    // blanking interval to refill before row 0 is needed, and means the ring
    // is only ever cleared while nothing is being displayed.  The two-cycle
    // pipe keeps the clear behind the last row's in-flight line-buffer reads.
    //
    // What makes this a REWRITE and not the old behaviour: it is
    // UNCONDITIONAL.  The old design gated the same event on `restart_ok`
    // (outstanding_empty && pipeline drained), an unbounded wait that one
    // lost fetch response made false forever -- the permanent wedge.
    wire sof = (hcount == 12'd0) && (vcount == 11'd0);
    wire visible_frame_end = de_in && (hcount == DST_W - 1)
                                   && (vcount == DST_H - 1);
    reg [1:0] vfe_pipe;
    always @(posedge pclk) begin
        if (!resetn) vfe_pipe <= 2'b00;
        else         vfe_pipe <= {vfe_pipe[0], visible_frame_end};
    end
    wire resync_pulse = vfe_pipe[1];
    // Top-of-frame WINDOW, used only to service a cold start or a placement
    // change (see scanout_fetch's `cold_start` / `placement_changed` terms;
    // neither can re-fire once serviced, so a held level is harmless).
    //
    // TWO cycles wide, and that matters: scanout_placement_sync commits the
    // new base/stride/depth ON the frame boundary, so the committed value is
    // only visible to this module the cycle AFTER sof.  A one-cycle window
    // latches the OLD placement and then has no boundary left to correct it
    // -- measured as tb-vram-scaler-firstlight's prime pass fetching nothing
    // at all, because its whole priming sequence contains exactly one sof.
    reg  sof_d;
    always @(posedge pclk) begin
        if (!resetn) sof_d <= 1'b0;
        else         sof_d <= sof;
    end
    wire frame_top = sof || sof_d;

    wire                       credit_return;
    wire [11:0]                disp_y;
    wire                       disp_ready;
    wire [LINE_COUNT_LOG2-1:0] disp_slot;
    wire                       fetch_direct;
    wire                       wr_en_p0, wr_en_p1, wr_en_p2;
    wire [LINE_MEM_AW-1:0]     wr_addr;
    wire [7:0]                 wr_d0, wr_d1, wr_d2;

    /* verilator lint_off UNUSEDSIGNAL */
    wire [3:0] fetch_reject_reason;
    /* verilator lint_on UNUSEDSIGNAL */

    scanout_fetch #(
        .SRC_W           (SRC_W),
        .SRC_H           (SRC_H),
        .FB_MAX_PIXELS   (FB_MAX_PIXELS),
        .FETCH_W         (FETCH_W),
        .ADDR_W          (ADDR_W),
        .LINE_COUNT_LOG2 (LINE_COUNT_LOG2)
    ) u_fetch (
        .pclk          (pclk),
        .resetn        (resetn),
        .resync_pulse  (resync_pulse),
        .frame_top     (frame_top),
        .credit_return (credit_return),
        .disp_y        (disp_y),
        .disp_ready    (disp_ready),
        .disp_slot     (disp_slot),
        .fb_base_px    (fb_base_px),
        .fb_stride_px  (fb_stride_px),
        .bytes_per_px  (bytes_per_px),
        .bpp_shift     (bpp_shift),
        .hres          (hres),
        .vres          (vres),
        .fb_rd_en      (fb_rd_en),
        .fb_rd_addr    (fb_rd_addr),
        .fb_rd_ready   (fb_rd_ready),
        .fb_rd_data    (fb_rd_data),
        .fb_rd_valid   (fb_rd_valid),
        .wr_en_p0      (wr_en_p0),
        .wr_en_p1      (wr_en_p1),
        .wr_en_p2      (wr_en_p2),
        .wr_addr       (wr_addr),
        .wr_d0         (wr_d0),
        .wr_d1         (wr_d1),
        .wr_d2         (wr_d2),
        .fetch_direct  (fetch_direct),
        // mode_admit's verdict on the fetcher's own latched placement.  Not
        // yet plumbed to video-status: docs/video_path_review.md S4.2 owns
        // that, and exporting it needs a port on every linebuf_scanout
        // instantiation (8 in-tree tbs plus video_top).  Connected here so the
        // channel exists and the fetcher's refusal has a named cause the
        // moment the status plumbing lands.
        .fetch_reject_reason (fetch_reject_reason)
    );

    scanout_display #(
        .SRC_W           (SRC_W),
        .SRC_H           (SRC_H),
        .DST_W           (DST_W),
        .DST_H           (DST_H),
        .LINE_COUNT_LOG2 (LINE_COUNT_LOG2),
        .SPLASH_SCALE    (SPLASH_SCALE)
    ) u_disp (
        .pclk          (pclk),
        .resetn        (resetn),
        .hcount        (hcount),
        .vcount        (vcount),
        .de_in         (de_in),
        .hs_in         (hs_in),
        .vs_in         (vs_in),
        .sof           (sof),
        .bpp_shift     (bpp_shift),
        .hres          (hres),
        .vres          (vres),
        .scale_n       (scale_n),
        .fetch_direct  (fetch_direct),
        .dafb_live     (dafb_live),
        .disp_y        (disp_y),
        .disp_ready    (disp_ready),
        .disp_slot     (disp_slot),
        .credit_return (credit_return),
        .wr_en_p0      (wr_en_p0),
        .wr_en_p1      (wr_en_p1),
        .wr_en_p2      (wr_en_p2),
        .wr_addr       (wr_addr),
        .wr_d0         (wr_d0),
        .wr_d1         (wr_d1),
        .wr_d2         (wr_d2),
        .clut_wclk     (clut_wclk),
        .clut_we       (clut_we),
        .clut_waddr    (clut_waddr),
        .clut_wdata    (clut_wdata),
        .rgb           (rgb),
        .de_out        (de_out),
        .hs_out        (hs_out),
        .vs_out        (vs_out),
        .line_underflow_sticky(line_underflow_sticky)
    );

    // ── Debug aliases ────────────────────────────────────────────────
    // Testbenches probe these by hierarchical name (u_scanout.<name>).  Kept
    // HERE, at the historical names, so the module split does not churn every
    // tb.  Signals whose underlying CONCEPT the rewrite deleted are gone from
    // the tbs too, rather than silently tied off to a lie.
    /* verilator lint_off UNUSED */
    wire        fetch_placement_valid = u_fetch.fetch_placement_valid;
    // These two used to be inline wires in scanout_fetch.  They are now the
    // mode_admit authority's two gates, reached through the instance that
    // judges the fetcher's latched placement.  Same signals, same meaning,
    // same hierarchical alias names -- tbs that probe them do not change.
    wire        fetch_stride_sane     = u_fetch.u_admit_latched.stride_fits;
    wire        fetch_frame_in_mem    = u_fetch.u_admit_latched.frame_in_range;
    wire        fetch_wide            = u_fetch.fetch_wide;
    wire        can_request           = u_fetch.can_request;
    wire        placement_changed     = u_fetch.placement_changed;
    wire        req_addr_oob          = u_fetch.req_addr_oob;
    wire        display_line_ready    = disp_ready;
    wire [1:0]  fetch_state           = u_fetch.state;
    wire        outstanding_empty     = (u_fetch.outstanding == 12'd0);
    // S_RUN == 2'd3, S_RESYNC == 2'd1, S_FILL == 2'd2 (scanout_fetch.v).
    wire        prefetch_active       = (u_fetch.state == 2'd3);
    wire        in_resync             = (u_fetch.state == 2'd1)
                                     || (u_fetch.state == 2'd2);
    /* verilator lint_on UNUSED */

endmodule

`default_nettype wire
