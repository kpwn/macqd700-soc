// scanout_placement_sync.v -- frame-boundary CDC for DAFB scanout placement
// ---------------------------------------------------------------------------
// Synchronizes the DAFB-programmed framebuffer base/stride values into the
// HDMI pixel clock domain, filters for two consecutive equal synchronized
// samples, and commits the pair only on a caller-supplied frame boundary.
//
// This keeps a live DAFB register rewrite from changing the scaler address
// equation in the middle of a frame.  A zero stride is sanitized back to the
// elaborated default source stride so reset or partially-programmed DAFB state
// cannot collapse all rows onto the same address.
//
// The synchronized base/stride pair is also sanity/range-checked before a
// frame-boundary commit.  If the stride cannot hold one complete source row,
// or if the complete source frame would exceed the addressable framebuffer
// span, the last committed placement is retained.
//
// >>> THE CHECK ITSELF IS NOT HERE.  It is mode_admit.v -- stage 3 of
// >>> docs/video_path_review.md S4.1, the single module in the video path
// >>> that is allowed to own an admission inequality.  This module decides
// >>> WHEN a verdict is latched (a frame boundary) and never what it is.
// >>> The pre-migration inline gates survive at the bottom of this file
// >>> under `ifdef VERILATOR as an equivalence assertion against the
// >>> authority; if the two ever disagree, simulation dies.
//
// The GEOMETRY channel (hres/vres/scale_n) is gated by the same check, on
// whichever placement will actually be in force alongside it -- the candidate
// when the candidate commits, the retained one otherwise.  Geometry used to
// commit unconditionally, which meant a mode set could put the new geometry
// on screen next to the old placement, a pair nothing had validated; see the
// `commit_geometry` comment block for the measured black frames.  Everything
// this module emits is therefore a tuple that satisfies the admission
// authority, which is what "never hand the scanner something it cannot
// render" has to mean -- and since scanout_fetch.v now consults the SAME
// authority rather than its own copy of the test, the two cannot disagree
// about what "renderable" means.
//
// Latency: at least two pclk cycles for synchronization plus one frame boundary
// before a changed pair reaches fb_base_px/fb_stride_px.
// ---------------------------------------------------------------------------
`default_nettype none

module scanout_placement_sync #(
    parameter ADDR_W       = 20,
    parameter SRC_W        = 1024,
    parameter SRC_H        = 768,
    parameter FB_MAX_PIXELS = SRC_W * SRC_H,
    parameter FB_BASE_PX   = 0,
    parameter FB_STRIDE_PX = 1024,
    parameter DST_W        = 1920,
    parameter DST_H        = 1080
) (
    input  wire              pclk,
    input  wire              rst,
    input  wire              frame_start,

    input  wire [ADDR_W-1:0] scanout_fb_base_px,
    input  wire [ADDR_W-1:0] scanout_fb_stride_px,
    // log2(source_pixels_per_byte): 1bpp=3, 2bpp=2, 4bpp=1, 8bpp=0.
    // Used both to gate the placement-range check (the scanner reads
    // SRC_W>>bpp_shift bytes per row, not SRC_W) and to drive
    // linebuf_scanout's line_rd_addr right-shift.
    //
    // Only valid when scanout_depth_supported is high.  bpp_shift cannot
    // express a multi-byte-per-pixel depth (24bpp = 4 B/px of VRAM) -- see
    // the port comment in rtl/mac/video.v.  For those depths the
    // scanout_bytes_per_px channel below carries the real term and bpp_shift
    // is 0.
    input  wire [2:0]        scanout_bpp_shift,
    // VRAM bytes per source pixel (video.v's fb_bytes_per_px):
    //   0 -> sub-byte depth (1/2/4bpp), use bpp_shift
    //   1 -> 8bpp
    //   4 -> 24bpp direct colour (xRGB)
    // Synchronised, torn-value-filtered and frame-boundary-committed exactly
    // like bpp_shift, and consumed by the range/stride validity check below
    // so a 24bpp placement is measured against its real 4 B/px footprint.
    input  wire [2:0]        scanout_bytes_per_px,
    // 1 iff the scanner can render the currently programmed depth (i.e. the
    // depth is 8bpp or less).  A placement is only committed when this is
    // high, so selecting 16/24bpp retains the last good placement instead of
    // scanning a row at the wrong bytes-per-pixel and pushing raw colour
    // bytes through the CLUT as palette indices.  Same fail-safe this module
    // already applies to an out-of-range base/stride.
    input  wire              scanout_depth_supported,
    // Visible source dimensions decoded from DAFB Swatch params.  Both
    // 12-bit so the largest Q700 mode (1280-wide) fits.
    input  wire [11:0]       scanout_hres,
    input  wire [11:0]       scanout_vres,

    output reg  [ADDR_W-1:0] fb_base_px,
    output reg  [ADDR_W-1:0] fb_stride_px,
    output reg  [2:0]        bpp_shift,
    output reg  [2:0]        bytes_per_px,
    output reg  [11:0]       hres,
    output reg  [11:0]       vres,
    // Committed INTEGER output scale, N display pixels per source pixel on
    // both axes.  The POLICY lives in place_plan.v and nowhere else:
    //     N = min(floor(DST_W/hres), floor(DST_H/vres)), clamped to >= 1
    // Nearest-neighbour replication, no fractional ratios.  This module only
    // decides WHEN the answer is latched (a frame boundary), never what it is.
    //
    //     640x480  -> 2      1024x768 -> 1
    //     512x384  -> 2      1152x870 -> 1
    //     832x624  -> 1
    output reg  [2:0]        scale_n,
    // ── Why the last placement was NOT admitted ───────────────────────
    // Evaluated on the CANDIDATE tuple, combinationally, so it is valid at
    // every frame_start and not only after a rejection.  REJ_NONE (0) means
    // the candidate satisfies every gate.  The encoding lives in
    // mode_admit.v's REJ_* localparams -- this module drives the channel but
    // does not own it.  docs/video_path_review.md S4.2 for why it exists: a
    // rejected placement used to produce rd_en=0 indefinitely with no reason
    // code, no counter and no status bit, and diagnosing the resulting black
    // screen meant hand-evaluating four inequalities against live registers.
    output wire [3:0]        reject_reason,
    // STICKY "a placement was rejected since the last one committed".  Set at
    // any frame_start whose candidate is refused; cleared at any frame_start
    // that commits.  This is the bit that says "the screen is black because
    // the scanner was never given anything to scan", which no single live
    // signal could say before.
    output reg               placement_rejected_sticky,
    // The tuple that was refused, latched at the rejecting frame_start.  Held
    // until the next rejection, so it survives long enough to be read over
    // JTAG.  Meaningless while placement_rejected_sticky is 0.
    output reg  [ADDR_W-1:0] reject_base_px,
    output reg  [ADDR_W-1:0] reject_stride_px,
    output reg  [11:0]       reject_hres_px,
    output reg  [11:0]       reject_vres_px,
    output reg  [3:0]        reject_reason_latched,
    // ── "the DAFB has started feeding real video" ────────────────────
    // STICKY, set at the first frame_start where this module actually
    // COMMITS a DAFB-programmed placement with a non-degenerate geometry --
    // i.e. the exact instant the scanner has something renderable to scan.
    // Consumed by scanout_display.v to retire the boot splash.
    //
    // Why this signal and not one of the neighbours:
    //   * it is FALSE from reset by construction, because depth_ok_stable is
    //     (see its reset comment: "before the DAFB has been programmed there
    //     is no renderable depth") and hres/vres are 0;
    //   * it becomes TRUE only when depth_ok + stride-sane + in-range all
    //     hold, which is the same conjunction that gates the placement
    //     commit -- so it cannot claim video is live one frame before the
    //     scanner would agree;
    //   * it is STICKY, so a later 16bpp / unmapped-AC842 depth probe (which
    //     really happens -- see the fb_ready comment in rtl/mac/video.v) or a
    //     transient out-of-range base cannot resurrect the splash over a
    //     running desktop;
    //   * it only ever changes ON frame_start, so the switchover is
    //     frame-aligned by construction and cannot tear mid-frame.
    output reg               dafb_live
);

    localparam [ADDR_W-1:0] FB_BASE_DEFAULT   = FB_BASE_PX;
    localparam [ADDR_W-1:0] SRC_W_ADDR        = SRC_W;
    localparam [ADDR_W-1:0] SRC_H_ADDR        = SRC_H;
    localparam [ADDR_W-1:0] FB_STRIDE_DEFAULT =
        (FB_STRIDE_PX < SRC_W) ? SRC_W_ADDR : FB_STRIDE_PX;
    localparam integer FRAME_ADDR_W = ADDR_W + $clog2(SRC_H) + 1;
    // Byte capacity of the framebuffer backing store.  See the
    // frame_last_addr comment below for why this is a byte count and not the
    // pixel count its old name (FB_MAX_PIXELS) implied.  The parameter name
    // is kept for interface compatibility with existing instantiations.
    localparam [FRAME_ADDR_W-1:0] FB_BYTE_LIMIT = FB_MAX_PIXELS;

    (* ASYNC_REG = "TRUE" *) reg [ADDR_W-1:0] fb_base_meta;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_W-1:0] fb_base_sync;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_W-1:0] fb_stride_meta;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_W-1:0] fb_stride_sync;
    (* ASYNC_REG = "TRUE" *) reg [2:0]        bpp_shift_meta;
    (* ASYNC_REG = "TRUE" *) reg [2:0]        bpp_shift_sync;
    (* ASYNC_REG = "TRUE" *) reg [2:0]        bppx_meta;
    (* ASYNC_REG = "TRUE" *) reg [2:0]        bppx_sync;
    (* ASYNC_REG = "TRUE" *) reg              depth_ok_meta;
    (* ASYNC_REG = "TRUE" *) reg              depth_ok_sync;
    (* ASYNC_REG = "TRUE" *) reg [11:0]       hres_meta, hres_sync;
    (* ASYNC_REG = "TRUE" *) reg [11:0]       vres_meta, vres_sync;

    reg [ADDR_W-1:0] fb_base_prev;
    reg [ADDR_W-1:0] fb_stride_prev;
    reg [2:0]        bpp_shift_prev;
    reg [2:0]        bppx_prev;
    reg              depth_ok_prev;
    reg [11:0]       hres_prev;
    reg [11:0]       vres_prev;
    reg [ADDR_W-1:0] fb_base_stable;
    reg [ADDR_W-1:0] fb_stride_stable;
    reg [2:0]        bpp_shift_stable;
    reg [2:0]        bppx_stable;
    reg              depth_ok_stable;
    reg [11:0]       hres_stable;
    reg [11:0]       vres_stable;
    // ── Stage 4: the scaling policy, as a module ──────────────────────
    // Pre-computed in the pclk domain (combinational off the synced
    // hres/vres) so it commits atomically with the rest of the placement at
    // frame_start.  The policy itself is NOT here -- it is place_plan.v, the
    // single implementation, shared with scanout_display.v.  Feeding it
    // scale_n_in = 0 asks it to apply the policy; the display half feeds the
    // committed answer back in, so the two cannot disagree about where the
    // active window ends.
    wire [2:0] scale_pick;
    place_plan #(
        .DST_W (DST_W),
        .DST_H (DST_H)
    ) u_place_plan (
        .hres_px     (hres_stable),
        .vres_px     (vres_stable),
        .scale_n_in  (3'd0),
        .scale_n     (scale_pick),
        .active_w_px (),
        .active_h_px (),
        .border_x_px (),
        .border_y_px ()
    );

    // ── Stage 3: mode_admit -- THE admission authority ────────────────
    // Every admission inequality that used to live in this file now lives in
    // rtl/board/video_phy/mode_admit.v, and so does the reason-code encoding.
    // This module decides only WHEN a verdict is latched (a frame boundary),
    // never what the verdict is.  docs/video_path_review.md S4.1 stage 3, and
    // read mode_admit.v's header for the >= / < operator derivation before
    // touching anything here.
    //
    // TWO instances, because two different tuples have to be judged:
    //
    //   u_admit_candidate  the CANDIDATE -- the synchronised, torn-value-
    //                      filtered *_stable tuple.  Gates the placement
    //                      commit and drives the reject_reason channel.
    //   u_admit_retained   the RETAINED placement (the committed fb_base_px /
    //                      fb_stride_px / depth already on screen) measured
    //                      against the CANDIDATE geometry.  See the
    //                      commit_geometry block below for why that pair needs
    //                      judging separately: when the candidate is rejected,
    //                      THAT is the pair actually in force.
    //
    // The retained instance is read for `addr_admissible`, not `renderable`:
    // the retained depth was already admitted when it committed, and this
    // module has no fresher information about it.  That is exactly the
    // conjunction the placement_renderable() function it replaces returned.
    //
    // The FB_MAX_PIXELS parameter name is kept for interface compatibility
    // with existing instantiations.  It has always been a BYTE capacity -- see
    // mode_admit.v for the rename that had to happen here once already.
    wire [FRAME_ADDR_W-1:0] frame_last_addr;
    wire [ADDR_W-1:0]       row_px_stable;
    wire [ADDR_W-1:0]       min_stride_bytes;
    wire [ADDR_W-1:0]       row_last_off;
    wire [ADDR_W-1:0]       row_count_stable;
    wire [ADDR_W-1:0]       last_row_stable;
    wire                    stable_stride_sane;
    wire                    stable_placement_in_range;
    wire                    commit_placement;
    wire                    retained_renderable;
    // REJ_NONE, from the authority.  The encoding lives in mode_admit.v and
    // is not repeated here -- that is the whole point of stage 3.
    wire [3:0]              rej_none_code;

    mode_admit #(
        .ADDR_W       (ADDR_W),
        .SRC_W        (SRC_W),
        .SRC_H        (SRC_H),
        .FB_MAX_BYTES (FB_MAX_PIXELS)
    ) u_admit_candidate (
        .hres_px               (hres_stable),
        .vres_px               (vres_stable),
        .bpp_shift             (bpp_shift_stable),
        .bytes_per_px          (bppx_stable),
        .depth_supported       (depth_ok_stable),
        .fb_base_bytes         (fb_base_stable),
        .fb_stride_bytes       (fb_stride_stable),
        .renderable            (commit_placement),
        .addr_admissible       (),
        .stride_fits           (stable_stride_sane),
        .frame_in_range        (stable_placement_in_range),
        .reject_reason         (reject_reason),
        .reason_none_code      (rej_none_code),
        .row_px                (row_px_stable),
        .row_span_bytes        (min_stride_bytes),
        .row_last_off_bytes    (row_last_off),
        .row_last_elem_idx     (),
        .row_count             (row_count_stable),
        .last_row              (last_row_stable),
        .frame_last_addr_bytes (frame_last_addr),
        .direct_colour         (),
        .geometry_zero         ()
    );

    mode_admit #(
        .ADDR_W       (ADDR_W),
        .SRC_W        (SRC_W),
        .SRC_H        (SRC_H),
        .FB_MAX_BYTES (FB_MAX_PIXELS)
    ) u_admit_retained (
        .hres_px               (hres_stable),
        .vres_px               (vres_stable),
        .bpp_shift             (bpp_shift),
        .bytes_per_px          (bytes_per_px),
        .depth_supported       (1'b1),
        .fb_base_bytes         (fb_base_px),
        .fb_stride_bytes       (fb_stride_px),
        .renderable            (),
        .addr_admissible       (retained_renderable),
        .stride_fits           (),
        .frame_in_range        (),
        .reject_reason         (),
        .reason_none_code      (),
        .row_px                (),
        .row_span_bytes        (),
        .row_last_off_bytes    (),
        .row_last_elem_idx     (),
        .row_count             (),
        .last_row              (),
        .frame_last_addr_bytes (),
        .direct_colour         (),
        .geometry_zero         ()
    );

    // ── Geometry-commit gate ──────────────────────────────────────────
    // hres/vres/scale_n used to commit UNCONDITIONALLY while base/stride/
    // bpp_shift/bytes_per_px committed only on validation.  The validation is
    // performed on the CANDIDATE tuple -- candidate base/stride/depth measured
    // against the candidate hres/vres -- so it says nothing at all about the
    // pair that is actually in force when the candidate is rejected: the NEW
    // geometry alongside the RETAINED old placement.  That pair was never
    // checked by anything, and it is trivially reachable, because a mode set
    // is many separate CPU register writes and the geometry ones can land
    // first.
    //
    // Measured, on the real chain (tb-dafb-24bpp-capacity scenario 9), for a
    // 640x480 -> 832x624 mode set at the live-board row pitches:
    //   after STRIDE(832):  hres 640, stride 832  -> 348,160 fetches, painted
    //   after PCBR:         hres 1280, stride 832 -> 0 fetches, BLACK FRAME
    //   after HAL:          hres 1306, stride 832 -> 0 fetches, BLACK FRAME
    //   after HFP:          hres 832,  stride 832 -> 452,608 fetches, painted
    // hres goes to 1280 because video.v derives it as (HFP-HAL) << clockdiv,
    // and the AC842 PCBR carries the clockdiv field: the 832x624 clockdiv (2)
    // lands one write before the Swatch H pair it applies to.  At an 832-byte
    // pitch an hres at/over SRC_W needs stride > 1023, and 832 is not, so
    // scanout_fetch.v's `fetch_stride_sane` goes false, `can_request` goes
    // false, and the frame fetches NOTHING.  Both endpoints render; only the
    // seam does not.
    //
    // Fix: commit geometry only when the placement that will be in force
    // alongside it can actually render it.  Two arms:
    //   commit_placement    -- the candidate was validated against exactly
    //                          this geometry, so they commit ATOMICALLY.
    //   retained_renderable -- the candidate was rejected, but the retained
    //                          placement still satisfies the same gate under
    //                          the new geometry.  This is the arm that keeps
    //                          the original "draw a stale colour frame rather
    //                          than go blank" behaviour alive for the cases
    //                          where it was actually true.
    // When neither holds, geometry is held too -- the last FULLY consistent
    // configuration stays on screen.  It cannot deadlock: the moment a valid
    // candidate arrives, the first arm commits the whole tuple together.
    // commit_placement is u_admit_candidate's `renderable` output: exactly
    // depth_ok_stable && stride_fits && frame_in_range, evaluated in the
    // authority instead of here.

    // ── WHY a placement was not admitted (docs/video_path_review.md S4.2) ──
    // The reason-code ENCODING and the priority ladder that picks between the
    // codes both live in mode_admit.v -- `reject_reason` above is driven
    // straight out of u_admit_candidate.  It is evaluated on the CANDIDATE
    // tuple, combinationally, so it is valid at every frame_start and not only
    // after a rejection.  REJ_NONE (0) means the candidate satisfies every
    // gate.  tools/jtag_repl.tcl decodes the numbers; they are a wire protocol.
    //
    // retained_renderable is u_admit_retained's `addr_admissible`.  See the
    // commit_geometry block above for what it is for.
    wire commit_geometry = commit_placement || retained_renderable;

    always @(posedge pclk) begin
        if (rst) begin
            fb_base_meta     <= FB_BASE_DEFAULT;
            fb_base_sync     <= FB_BASE_DEFAULT;
            fb_base_prev     <= FB_BASE_DEFAULT;
            fb_base_stable   <= FB_BASE_DEFAULT;
            fb_base_px       <= FB_BASE_DEFAULT;
            fb_stride_meta   <= FB_STRIDE_DEFAULT;
            fb_stride_sync   <= FB_STRIDE_DEFAULT;
            fb_stride_prev   <= FB_STRIDE_DEFAULT;
            fb_stride_stable <= FB_STRIDE_DEFAULT;
            fb_stride_px     <= FB_STRIDE_DEFAULT;
            bpp_shift_meta   <= 3'd0;
            bpp_shift_sync   <= 3'd0;
            bpp_shift_prev   <= 3'd0;
            bpp_shift_stable <= 3'd0;
            bpp_shift        <= 3'd0;
            // Reset to the indexed (1 B/px) interpretation -- matches the
            // reset placement, which is an 8bpp-shaped default stride.
            bppx_meta        <= 3'd1;
            bppx_sync        <= 3'd1;
            bppx_prev        <= 3'd1;
            bppx_stable      <= 3'd1;
            bytes_per_px     <= 3'd1;
            // Reset LOW: before the DAFB has been programmed there is no
            // renderable depth, and the reset placement is already in force.
            depth_ok_meta    <= 1'b0;
            depth_ok_sync    <= 1'b0;
            depth_ok_prev    <= 1'b0;
            depth_ok_stable  <= 1'b0;
            hres_meta        <= 12'd0;
            hres_sync        <= 12'd0;
            hres_prev        <= 12'd0;
            hres_stable      <= 12'd0;
            hres             <= 12'd0;
            vres_meta        <= 12'd0;
            vres_sync        <= 12'd0;
            vres_prev        <= 12'd0;
            vres_stable      <= 12'd0;
            vres             <= 12'd0;
            // Reset N to 1 (1:1), the identity ratio -- the reset geometry
            // is 0x0, whose active window is empty either way.
            scale_n          <= 3'd1;
            dafb_live        <= 1'b0;
            placement_rejected_sticky <= 1'b0;
            reject_base_px        <= FB_BASE_DEFAULT;
            reject_stride_px      <= FB_STRIDE_DEFAULT;
            reject_hres_px        <= 12'd0;
            reject_vres_px        <= 12'd0;
            reject_reason_latched <= rej_none_code;
        end else begin
            fb_base_meta   <= scanout_fb_base_px;
            fb_base_sync   <= fb_base_meta;
            fb_base_prev   <= fb_base_sync;
            fb_stride_meta <= scanout_fb_stride_px;
            fb_stride_sync <= fb_stride_meta;
            fb_stride_prev <= fb_stride_sync;
            bpp_shift_meta <= scanout_bpp_shift;
            bpp_shift_sync <= bpp_shift_meta;
            bpp_shift_prev <= bpp_shift_sync;
            bppx_meta      <= scanout_bytes_per_px;
            bppx_sync      <= bppx_meta;
            bppx_prev      <= bppx_sync;
            depth_ok_meta  <= scanout_depth_supported;
            depth_ok_sync  <= depth_ok_meta;
            depth_ok_prev  <= depth_ok_sync;
            hres_meta   <= scanout_hres;
            hres_sync   <= hres_meta;
            hres_prev   <= hres_sync;
            vres_meta   <= scanout_vres;
            vres_sync   <= vres_meta;
            vres_prev   <= vres_sync;

            // Same two-sample-agree filter as base/stride below: a DAFB
            // write can straddle a frame_start commit and leave bpp_shift/
            // hres/vres torn for one sample even though each field is a
            // single narrow register — the CPU can still be mid-write to
            // an adjacent field when the synchronizer samples it.  Only
            // commit to *_stable once two consecutive synced samples
            // agree, exactly like fb_base/fb_stride, so a torn value for
            // one frame is filtered instead of committing.
            if (bpp_shift_sync == bpp_shift_prev)
                bpp_shift_stable <= bpp_shift_sync;
            if (bppx_sync == bppx_prev)
                bppx_stable <= bppx_sync;
            if (depth_ok_sync == depth_ok_prev)
                depth_ok_stable <= depth_ok_sync;
            if (hres_sync == hres_prev)
                hres_stable <= hres_sync;
            if (vres_sync == vres_prev)
                vres_stable <= vres_sync;

            if (fb_base_sync == fb_base_prev)
                fb_base_stable <= fb_base_sync;
            if (fb_stride_sync == fb_stride_prev) begin
                fb_stride_stable <= (fb_stride_sync == {ADDR_W{1'b0}})
                                  ? FB_STRIDE_DEFAULT
                                  : fb_stride_sync;
            end

            if (frame_start) begin
                // depth_ok_stable joins the existing stride/range validity
                // gate: an unrenderable depth (16/24bpp) is treated exactly
                // like an out-of-range base/stride, i.e. the last good
                // placement is retained rather than committing something the
                // scanner would render as garbage.
                if (commit_placement) begin
                    fb_base_px   <= fb_base_stable;
                    fb_stride_px <= fb_stride_stable;
                    bpp_shift    <= bpp_shift_stable;
                    // bytes_per_px commits with the placement it was
                    // validated against, never independently -- the scanner
                    // must never see a 24bpp depth paired with an 8bpp
                    // base/stride (or vice versa) for even one frame.
                    bytes_per_px <= bppx_stable;
                    // A committed placement whose geometry is still 0x0 has
                    // an EMPTY active window (see scanout_display's
                    // in_active_x/in_active_y), so it renders nothing -- that
                    // is not "video is live".  Require both.
                    if ((hres_stable != 12'd0) && (vres_stable != 12'd0))
                        dafb_live <= 1'b1;
                end
                // hres/vres/scale_n are a separate channel, but NOT an
                // independent one: the placement gate above is evaluated
                // against them, so committing them next to a placement that
                // was rejected ships a pair nothing ever validated.  See the
                // commit_geometry comment block above for the measured black
                // frames that came of it.
                if (commit_geometry) begin
                    hres    <= hres_stable;
                    vres    <= vres_stable;
                    scale_n <= scale_pick;
                end

                // ── Loud failure (docs/video_path_review.md S4.2) ──────
                // Sticky "rejected since the last commit", plus the tuple
                // that was refused.  Latched at the frame boundary that
                // made the decision so what is reported is exactly what
                // was judged, not whatever the DAFB has drifted to since.
                if (commit_placement) begin
                    placement_rejected_sticky <= 1'b0;
                end else begin
                    placement_rejected_sticky <= 1'b1;
                    reject_base_px        <= fb_base_stable;
                    reject_stride_px      <= fb_stride_stable;
                    reject_hres_px        <= hres_stable;
                    reject_vres_px        <= vres_stable;
                    reject_reason_latched <= reject_reason;
                end
            end
        end
    end

`ifdef VERILATOR
    // ── THE MIGRATION'S OWN REGRESSION TEST ───────────────────────────
    // What follows is the admission logic this module used to carry inline,
    // preserved VERBATIM (modulo a legacy_ prefix) and compiled only under
    // `ifdef VERILATOR.  It is not part of the design: it exists so that the
    // authority and the code it replaced are checked against each other on
    // every candidate tuple every simulated cycle, at zero synthesis cost.
    //
    // If mode_admit.v is ever edited into disagreement with what shipped
    // before it -- a flipped operator, a lost clamp, a units slip -- sim dies
    // here with the term named.  Delete this block only when the shipped
    // behaviour is deliberately being changed, and say so in the commit.
    wire        legacy_direct = (bppx_stable == 3'd4);
    wire [ADDR_W-1:0] legacy_row_px =
        ((hres_stable != 12'd0) && (hres_stable < SRC_W))
            ? {{(ADDR_W-12){1'b0}}, hres_stable}
            : SRC_W_ADDR;
    // Byte span of one visible row, and the highest byte offset within it.
    //
    // The INDEXED arms used to be built from the elaborated SRC_W rather than
    // legacy_row_px, i.e. from the build-time scanner width instead of the
    // DAFB-programmed one.  That mattered because linebuf_scanout.v's own
    // `fetch_stride_sane` gate was built the same way and is the one that
    // gates `can_request`: a Q700 832x624 mode programs an 832-byte stride,
    // which is a perfectly valid 832-pixel row, but both gates measured it
    // against SRC_W=1024 and rejected it -- the fetcher then never issued a
    // single request and the screen went black with the CPU running.  Both
    // arms are now measured against the VISIBLE row, which is what the
    // scanner actually reads.  When hres is 0 or >= SRC_W, legacy_row_px is
    // SRC_W and every expression below collapses to exactly its old value,
    // so the elaborated-width configurations are bit-for-bit unchanged.
    wire [ADDR_W-1:0] legacy_direct_span = legacy_row_px << 2;
    wire [ADDR_W-1:0] legacy_indexed_last =
        (legacy_row_px - {{(ADDR_W-1){1'b0}}, 1'b1}) >> bpp_shift_stable;
    wire [ADDR_W-1:0] legacy_min_stride =
        legacy_direct ? legacy_direct_span
                      : (legacy_indexed_last + {{(ADDR_W-1){1'b0}}, 1'b1});
    wire [ADDR_W-1:0] legacy_row_last_off =
        legacy_direct ? (legacy_direct_span - {{(ADDR_W-1){1'b0}}, 1'b1})
                      : legacy_indexed_last;
    // ── Rows the scanner actually reads ──────────────────────────────
    // This used to be the ELABORATED SRC_H (768 on the production build),
    // i.e. the build-time scanner height rather than the DAFB-programmed
    // vres -- the exact same build-time-constant-standing-in-for-a-runtime-
    // value mistake `legacy_row_px` above already had to fix for the
    // horizontal axis.  It matters for the same reason: the frame-footprint
    // term below is what admits or rejects a placement, and measuring a
    // 480-row mode as if it were 768 rows overstates the footprint by 60%.
    //
    // Concretely, and this is a measured hardware black-screen: Mac OS
    // selecting "Millions" on a 640x480 Q700 programs base 4096 / stride
    // 4096 / 4 B/px.  The real footprint is
    //     4096 + 4096*479 + 2559 = 1,968,639 bytes,
    // which fits the 2 MiB (0x200000) URAM aperture.  Measured at SRC_H=768
    // it comes out as
    //     4096 + 4096*767 + 2559 = 3,148,287 bytes,
    // which does not -- so the placement was never committed, the scanner
    // stayed in the previously committed 8bpp INDEXED mode, and the direct-
    // colour framebuffer got pushed through the CLUT as palette indices.
    // (The 8bpp modes escaped only because their stride is 4x smaller:
    // 4096 + 1024*767 + 639 = 790,143, comfortably under the aperture.)
    //
    // Clamped to [1, SRC_H]: a zero or oversized vres falls back to the
    // elaborated height, so every configuration that programs vres >= SRC_H
    // (or has not programmed it yet) is bit-for-bit unchanged.
    wire [ADDR_W-1:0] legacy_row_count =
        ((vres_stable != 12'd0) && (vres_stable < SRC_H))
            ? {{(ADDR_W-12){1'b0}}, vres_stable}
            : SRC_H_ADDR;
    wire [ADDR_W-1:0] legacy_last_row =
        legacy_row_count - {{(ADDR_W-1){1'b0}}, 1'b1};
    // `base + stride * row + offset` -- docs/video_path_review.md S4.1 names
    // this and scanout_fetch's identical expression as THE arithmetic that
    // belongs in a DSP48E2: it is an exact (A+D)*B+C fit, and the fabric it
    // otherwise sits in is at 76.9% LUT with congestion 5-6 while 1797 of
    // 1824 DSPs are idle.  The attribute is a synthesis directive only --
    // simulation semantics are unchanged, and the cone stays combinational
    // because the admission answer must be valid at every frame_start.
    wire [FRAME_ADDR_W-1:0] legacy_frame_last_addr =
        {{(FRAME_ADDR_W-ADDR_W){1'b0}}, fb_base_stable}
      + ({{(FRAME_ADDR_W-ADDR_W){1'b0}}, fb_stride_stable}
         * {{(FRAME_ADDR_W-ADDR_W){1'b0}}, legacy_last_row})
      + {{(FRAME_ADDR_W-ADDR_W){1'b0}}, legacy_row_last_off};
    wire legacy_in_range =
        (legacy_frame_last_addr < FB_BYTE_LIMIT);
    wire legacy_stride_sane =
        (fb_stride_stable >= legacy_min_stride);

    always @(*) begin
        if (!rst) begin
            if (legacy_stride_sane !== stable_stride_sane)
                $fatal(1, "[scanout_placement_sync] mode_admit stride_fits=%0d but the pre-migration gate says %0d (stride=%0d span=%0d)",
                       stable_stride_sane, legacy_stride_sane,
                       fb_stride_stable, min_stride_bytes);
            if (legacy_in_range !== stable_placement_in_range)
                $fatal(1, "[scanout_placement_sync] mode_admit frame_in_range=%0d but the pre-migration gate says %0d (last_addr=%0d)",
                       stable_placement_in_range, legacy_in_range, frame_last_addr);
            if (legacy_min_stride !== min_stride_bytes)
                $fatal(1, "[scanout_placement_sync] row_span_bytes=%0d, pre-migration min_stride_bytes=%0d",
                       min_stride_bytes, legacy_min_stride);
            if (legacy_row_last_off !== row_last_off)
                $fatal(1, "[scanout_placement_sync] row_last_off_bytes=%0d, pre-migration row_last_off=%0d",
                       row_last_off, legacy_row_last_off);
            if (legacy_frame_last_addr !== frame_last_addr)
                $fatal(1, "[scanout_placement_sync] frame_last_addr_bytes=%0d, pre-migration frame_last_addr=%0d",
                       frame_last_addr, legacy_frame_last_addr);
            if (legacy_row_px !== row_px_stable)
                $fatal(1, "[scanout_placement_sync] row_px=%0d, pre-migration row_px_stable=%0d",
                       row_px_stable, legacy_row_px);
            if (legacy_last_row !== last_row_stable)
                $fatal(1, "[scanout_placement_sync] last_row=%0d, pre-migration last_row_stable=%0d",
                       last_row_stable, legacy_last_row);
        end
    end
`endif

endmodule

`default_nettype wire
