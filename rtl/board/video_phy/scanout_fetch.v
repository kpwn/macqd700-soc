// scanout_fetch.v -- CREDIT-PACED source-line fetch engine for linebuf_scanout.
// ---------------------------------------------------------------------------
// Owns the LINE_COUNT-entry source-line ring: request walk, (in-order)
// response walk, per-slot tag/valid, and the line-memory WRITE port.
// scanout_display.v owns the read port, the scaler and the CLUT.
//
// >>> DESIGN RATIONALE: docs/scanout_credit_ring.md.  Read it before changing
// >>> anything here: it records which measured hardware symptom each term
// >>> below exists for, and why the af8fe56 watchdog is DELETED not ported.
// >>> This replaced nine interacting flags and a ten-term request gate --
// >>> every bug on hardware was an interaction BETWEEN those flags.
//
// The producer holds credits; the display returns one UNCONDITIONALLY per
// source row it passes; a row is fetched iff a credit is held.  At a frame
// boundary the fetcher holds exactly LINE_COUNT credits, so it fills the ring
// and stops -- it cannot race ahead during vblank, by arithmetic -- and
// credits carry no `line_tag < y_src` term for a flag to suppress, so it
// cannot park either.  The frame boundary is an UNCONDITIONAL hard resync: no
// wait on the walk draining (that unbounded wait was the original permanent
// wedge), just a counted, time-bounded discard of the responses still owed.
//
// fetch_row_y_last / req_addr_limit come from the RUNTIME `vres`, never the
// elaborated SRC_H (c57fc08 -- SRC_H was a measured HW black screen at 24bpp).
//
// >>> ADMISSION IS NOT DECIDED HERE.  This module used to carry its own copy
// >>> of the placement test -- `fetch_stride_sane`, `fetch_frame_in_mem` and a
// >>> `base + stride*row + offset` multiply character-identical to
// >>> scanout_placement_sync's -- written in a DIFFERENT idiom (strict `>`
// >>> over an inclusive last offset, against the other file's `>=` over a
// >>> count).  All of it is gone: mode_admit.v (stage 3) decides, this module
// >>> consumes the verdict and reports the reason on fetch_reject_reason.
// >>> docs/video_path_review.md S4.1.
// ---------------------------------------------------------------------------
`default_nettype none

module scanout_fetch #(
    parameter SRC_W                 = 1024,
    parameter SRC_H                 = 768,
    parameter FB_MAX_PIXELS         = SRC_W * SRC_H,
    parameter FETCH_W               = 32,
    parameter ADDR_W                = 20,
    parameter LINE_COUNT_LOG2       = 6,
    // Discard-window bound (pclk).  Must exceed the deepest fetch-port
    // backlog: fb_reader's FIFOs are 256 entries each, drained 1/clk.
    parameter RESYNC_DISCARD_CYCLES = 4096,
    // Derived widths -- never overridden by the wrapper.
    parameter SRC_X_W               = $clog2(SRC_W),
    parameter SRC_Y_W               = $clog2(SRC_H),
    parameter LINE_MEM_AW           = $clog2((1 << LINE_COUNT_LOG2) * SRC_W)
) (
    input  wire                       pclk,
    input  wire                       resetn,

    // ── Display coupling ─────────────────────────────────────────────
    input  wire                       resync_pulse,  // 1-cycle frame boundary
    input  wire                       frame_top,     // top-of-frame window
    input  wire                       credit_return, // display passed a row
    input  wire [11:0]                disp_y,        // display's source row
    output reg                        disp_ready,
    output reg  [LINE_COUNT_LOG2-1:0] disp_slot,

    // ── Runtime placement / depth (frame-committed upstream) ─────────
    input  wire [ADDR_W-1:0]          fb_base_px,
    input  wire [ADDR_W-1:0]          fb_stride_px,
    input  wire [2:0]                 bytes_per_px,
    input  wire [2:0]                 bpp_shift,
    input  wire [11:0]                hres,
    input  wire [11:0]                vres,

    // ── Ordered request/response fetch port ──────────────────────────
    output reg                        fb_rd_en,
    output reg  [ADDR_W-1:0]          fb_rd_addr,
    input  wire                       fb_rd_ready,
    input  wire [FETCH_W-1:0]         fb_rd_data,
    input  wire                       fb_rd_valid,

    // ── Line-memory write port (three byte planes) ───────────────────
    output wire                       wr_en_p0,
    output wire                       wr_en_p1,
    output wire                       wr_en_p2,
    output wire [LINE_MEM_AW-1:0]     wr_addr,
    output wire [7:0]                 wr_d0,
    output wire [7:0]                 wr_d1,
    output wire [7:0]                 wr_d2,

    // Latched depth for the display's CLUT/direct output mux.  Only ever
    // changes at a resync, where the ring is cleared, so it needs no pipe.
    output reg                        fetch_direct,

    // ── WHY this fetcher is refusing to issue requests ───────────────
    // Straight out of the mode_admit authority, evaluated on the LATCHED
    // placement this walk is actually using.  Before this existed, a fetch-
    // side refusal was `fb_rd_en == 0` forever with no reason code, no
    // counter and no status bit -- the placement half got a reason channel in
    // 7c23fcd and this half did not, which is the same decision owned in two
    // places that mode_admit exists to end.  Encoding: mode_admit.v's REJ_*.
    //
    // The depth codes cannot appear here.  This module is handed an
    // already-committed tuple and has no depth-support channel, so it gates
    // on `addr_admissible` (geometry + memory range) rather than
    // `renderable`; judging the depth would mean inventing an answer that
    // scanout_placement_sync already gave.
    output wire [3:0]                 fetch_reject_reason
);

    localparam LINE_COUNT   = (1 << LINE_COUNT_LOG2);
    localparam CRED_W       = LINE_COUNT_LOG2 + 1;
    localparam OUTST_W      = 12;                 // >> any reachable backlog
    localparam DISC_W       = 13;
    localparam [CRED_W-1:0] CRED_MAX  = LINE_COUNT;
    localparam [DISC_W-1:0] DISC_LOAD = RESYNC_DISCARD_CYCLES;
    localparam integer FRAME_ADDR_W = ADDR_W + $clog2(SRC_H) + 1;
    localparam [FRAME_ADDR_W-1:0] FB_PIXEL_LIMIT = FB_MAX_PIXELS;

    // ONE FSM: a restart is a STATE, not four racing booleans.  S_FILL is the
    // discard window -- stale responses swallowed, NO new request issued.
    localparam [1:0] S_IDLE   = 2'd0;
    localparam [1:0] S_RESYNC = 2'd1;
    localparam [1:0] S_FILL   = 2'd2;
    localparam [1:0] S_RUN    = 2'd3;

    generate
        if (FETCH_W != 32) begin : gen_fetch_w_bad
            initial begin
                $error("[scanout_fetch] FETCH_W=%0d unsupported -- the fetch port returns a 4-byte group at every depth; use the runtime bytes_per_px input for depth.", FETCH_W);
            end
        end
    endgenerate

    // ── Runtime depth decode ─────────────────────────────────────────
    wire        direct_in   = (bytes_per_px == 3'd4);
    wire [1:0]  px_shift_in = direct_in ? 2'd2 : 2'd0;
    // WIDE 24bpp: ONE aligned 4-byte request per pixel instead of three byte
    // requests at +1/+2/+3.  Needs base and stride 4-byte aligned (the DAFB
    // encoding guarantees it); the narrow fallback keeps an encoding change
    // correct-but-3x-hungrier rather than sheared.
    wire        wide_ok_in  = direct_in && (fb_base_px[1:0]   == 2'b00)
                                        && (fb_stride_px[1:0] == 2'b00);
    // ── Stage 3 (LIVE inputs): the ring-walk bounds ──────────────────
    // `row_last_elem_idx` is the last ring INDEX per row -- PIXELS at direct
    // colour, BYTES when indexed -- and `last_row` is the last source row.
    // Both used to be derived here; they are now read off the authority so
    // that the walk terminator and the byte footprint it is checked against
    // cannot drift apart (mode_admit assertion A4 ties them together).
    //
    // This instance sees the LIVE inputs, because `placement_changed` below
    // has to compare what is arriving against what was latched.  The verdict
    // instance further down sees the LATCHED tuple.  Everything this instance
    // computes beyond the two bounds is left unconnected and synthesises away.
    wire [ADDR_W-1:0] live_row_last_elem;
    wire [ADDR_W-1:0] live_last_row;
    mode_admit #(
        .ADDR_W       (ADDR_W),
        .SRC_W        (SRC_W),
        .SRC_H        (SRC_H),
        .FB_MAX_BYTES (FB_MAX_PIXELS)
    ) u_admit_live (
        .hres_px               (hres),
        .vres_px               (vres),
        .bpp_shift             (bpp_shift),
        .bytes_per_px          (bytes_per_px),
        .depth_supported       (1'b1),
        .fb_base_bytes         (fb_base_px),
        .fb_stride_bytes       (fb_stride_px),
        .renderable            (),
        .addr_admissible       (),
        .stride_fits           (),
        .frame_in_range        (),
        .reject_reason         (),
        .reason_none_code      (),
        .row_px                (),
        .row_span_bytes        (),
        .row_last_off_bytes    (),
        .row_last_elem_idx     (live_row_last_elem),
        .row_count             (),
        .last_row              (live_last_row),
        .frame_last_addr_bytes (),
        .direct_colour         (),
        .geometry_zero         ()
    );
    wire [SRC_X_W-1:0] row_last_idx_in = live_row_last_elem[SRC_X_W-1:0];
    wire [SRC_Y_W-1:0] row_y_last_in   = live_last_row[SRC_Y_W-1:0];

    // ── Ring state ───────────────────────────────────────────────────
    reg [SRC_Y_W-1:0]        line_tag [0:LINE_COUNT-1];
    reg [LINE_COUNT-1:0]     line_valid;
    reg [CRED_W-1:0]         credits;

    reg [SRC_Y_W-1:0]        req_y;
    reg [SRC_X_W-1:0]        req_x;
    reg [1:0]                req_phase;
    reg [SRC_Y_W-1:0]        rsp_y;
    reg [SRC_X_W-1:0]        rsp_x;
    reg [1:0]                rsp_phase;

    reg [1:0]                state;
    reg [OUTST_W-1:0]        outstanding;
    reg [OUTST_W-1:0]        stale_count;
    reg [DISC_W-1:0]         discard_timer;

    reg [ADDR_W-1:0]         fetch_base_px;
    reg [ADDR_W-1:0]         fetch_stride_px;
    reg                      fetch_wide;
    reg [1:0]                fetch_px_shift;
    // ── The latched placement: RAW, and latched exactly once ─────────
    // `fetch_row_px_last` and `fetch_row_y_last` used to be REGISTERS holding
    // pre-derived geometry, latched at the resync alongside the raw base and
    // stride.  They are wires now, derived from the raw tuple below through
    // the authority.
    //
    // That is not cosmetic.  Latching a derived value NEXT TO the raw value it
    // came from is two copies of the same fact in two registers, and the two
    // can hold different epochs -- which is precisely the disease this whole
    // change exists to cure, in miniature.  It bit immediately: the migration
    // assertion fired at time 0, where the derived regs still read 0 while the
    // raw tuple already implied a 1151-byte row.  One latched source, derived
    // after the register and never before it, makes the skew unrepresentable.
    reg [11:0]               fetch_hres_px;
    reg [11:0]               fetch_vres_px;
    reg [2:0]                fetch_bpp_shift;
    reg [2:0]                fetch_bytes_per_px;

    wire [LINE_COUNT_LOG2-1:0] req_slot = req_y[LINE_COUNT_LOG2-1:0];
    wire [LINE_COUNT_LOG2-1:0] rsp_slot = rsp_y[LINE_COUNT_LOG2-1:0];

    // ── Ring lookup for the display ──────────────────────────────────
    wire [SRC_Y_W-1:0] disp_y_t = disp_y[SRC_Y_W-1:0];
    integer si;
    always @(*) begin
        disp_ready  = 1'b0;
        disp_slot   = {LINE_COUNT_LOG2{1'b0}};
        for (si = 0; si < LINE_COUNT; si = si + 1) begin
            if (line_valid[si] && (line_tag[si] == disp_y_t)) begin
                disp_ready = 1'b1;
                disp_slot  = si[LINE_COUNT_LOG2-1:0];
            end
        end
    end

    // ── The verdict and the derived bounds, from mode_admit ──────────
    // Declared here, driven by u_admit_latched further down.  `req_x_last`
    // below is the ring-walk terminator and reads fetch_row_px_last, so the
    // nets have to exist before that block.
    wire [ADDR_W-1:0]       row_last_off;
    wire [ADDR_W-1:0]       latched_row_last_elem;
    wire [ADDR_W-1:0]       latched_last_row;
    wire [FRAME_ADDR_W-1:0] req_addr_limit;
    wire                    fetch_placement_valid;
    wire [SRC_X_W-1:0] fetch_row_px_last = latched_row_last_elem[SRC_X_W-1:0];
    wire [SRC_Y_W-1:0] fetch_row_y_last  = latched_last_row[SRC_Y_W-1:0];

    // ── Request address generation (unchanged arithmetic) ────────────
    wire [1:0] last_phase = (fetch_direct && !fetch_wide) ? 2'd2 : 2'd0;
    wire [SRC_X_W-1:0] req_x_last = fetch_row_px_last;
    wire [ADDR_W-1:0] req_x_zx      = {{(ADDR_W-SRC_X_W){1'b0}}, req_x};
    wire [ADDR_W-1:0] req_x_last_zx = {{(ADDR_W-SRC_X_W){1'b0}}, req_x_last};
    // Narrow 24bpp: +phase+1 skips the pad byte.  Wide: aligned (x<<2).
    wire [ADDR_W-1:0] req_row_off =
        (req_x_zx << fetch_px_shift)
      + ((fetch_direct && !fetch_wide)
             ? ({{(ADDR_W-2){1'b0}}, req_phase} + {{(ADDR_W-1){1'b0}}, 1'b1})
             : {ADDR_W{1'b0}});
    // ── Stage 3 (LATCHED tuple): THE VERDICT ─────────────────────────
    // `fetch_stride_sane`, `fetch_frame_in_mem` and the `req_addr_limit`
    // multiply that fed them are GONE from this file.  All three lived here as
    // a second, independently-maintained copy of scanout_placement_sync's
    // admission test -- the same decision owned in two places, in two idioms,
    // which is the structural bug docs/video_path_review.md S4.1 names.  This
    // module now consumes the authority's answer about its own latched tuple.
    //
    // DSP.  Deleting `req_addr_limit` removes one of the design's two
    // character-identical `base + stride*row + offset` multiplies outright;
    // the survivor is mode_admit's, and it carries (* use_dsp = "yes" *).
    // That closes the review's named follow-up better than annotating a
    // duplicate would have.
    mode_admit #(
        .ADDR_W       (ADDR_W),
        .SRC_W        (SRC_W),
        .SRC_H        (SRC_H),
        .FB_MAX_BYTES (FB_MAX_PIXELS)
    ) u_admit_latched (
        .hres_px               (fetch_hres_px),
        .vres_px               (fetch_vres_px),
        .bpp_shift             (fetch_bpp_shift),
        .bytes_per_px          (fetch_bytes_per_px),
        .depth_supported       (1'b1),
        .fb_base_bytes         (fetch_base_px),
        .fb_stride_bytes       (fetch_stride_px),
        .renderable            (),
        .addr_admissible       (fetch_placement_valid),
        .stride_fits           (),
        .frame_in_range        (),
        .reject_reason         (fetch_reject_reason),
        .reason_none_code      (),
        .row_px                (),
        .row_span_bytes        (),
        .row_last_off_bytes    (row_last_off),
        .row_last_elem_idx     (latched_row_last_elem),
        .row_count             (),
        .last_row              (latched_last_row),
        .frame_last_addr_bytes (req_addr_limit),
        .direct_colour         (),
        .geometry_zero         ()
    );

    // The PER-REQUEST address.  This multiply is NOT the one the review names
    // for a DSP: it is not registered -- it feeds `fb_rd_addr` AND, in the
    // same cycle, the `req_addr_oob` comparators that gate `req_take` -- so a
    // DSP here would win no pipeline stage, which is where a DSP's value in
    // this path comes from.  The real fix is stage 6 (row_addrgen): `req_y`
    // increments by one per row, so `base + req_y*stride` is an ACCUMULATOR
    // and the multiplier disappears entirely rather than moving house.
    wire [FRAME_ADDR_W-1:0] req_addr_c =
        {{(FRAME_ADDR_W-ADDR_W){1'b0}}, fetch_base_px}
      + ({{(FRAME_ADDR_W-ADDR_W){1'b0}}, req_y}
         * {{(FRAME_ADDR_W-ADDR_W){1'b0}}, fetch_stride_px})
      + {{(FRAME_ADDR_W-ADDR_W){1'b0}}, req_row_off};
    // WALK BOUNDS, NOT ADMISSION.  These two are the only inequalities left in
    // this file, and they are a different kind of thing from the gates that
    // moved to mode_admit: they ask "has the RUNNING request address wandered
    // outside the window the authority already admitted", not "should this
    // placement be admitted at all".  Both right-hand sides come from stage 3
    // -- `req_addr_limit` is its frame_last_addr_bytes and FB_PIXEL_LIMIT is
    // the same aperture capacity it was handed -- so neither re-derives a
    // limit, and a `$fatal` below turns a violation into a sim failure rather
    // than a silent read past the framebuffer.
    wire req_addr_frame_oob    = fetch_placement_valid && (req_addr_c > req_addr_limit);
    wire req_addr_mem_oob      = (req_addr_c >= FB_PIXEL_LIMIT);
    wire req_addr_oob          = req_addr_frame_oob || req_addr_mem_oob;

    // ── Resync trigger ───────────────────────────────────────────────
    wire placement_changed = (fb_base_px       != fetch_base_px)
                           || (fb_stride_px    != fetch_stride_px)
                           || (direct_in       != fetch_direct)
                           || (row_last_idx_in != fetch_row_px_last)
                           || (row_y_last_in   != fetch_row_y_last);
    // `resync_pulse` (end-of-visible-frame) is the steady-state resync -- see
    // linebuf_scanout.v for why it lives in blanking and not at (0,0).
    // Top-of-frame does two narrow jobs on top of it, and every term below is
    // load-bearing; doc S3/S4 records the measured symptom for each.
    //   COLD START: resync_pulse cannot arm the FIRST walk after reset,
    //   because there has been no end-of-frame yet.  Gated on S_IDLE *and* an
    //   empty ring -- "the ring holds nothing" is NOT "no walk is armed", and
    //   re-arming a walk whose first responses are still in flight throws that
    //   whole batch into the discard window.
    //   PLACEMENT CHANGE: an INPUT to the FSM, serviced at a frame boundary,
    //   never mid-frame; `can_request` is held off in the meantime because the
    //   latched placement is known-stale.
    // Neither can wipe a ring that holds anything, which is the property the
    // blanking-time resync exists to guarantee.
    wire ring_empty = (line_valid == {LINE_COUNT{1'b0}});
    wire cold_start = (state == S_IDLE) && ring_empty;
    wire resync_req = resync_pulse
                   || (frame_top && (cold_start || placement_changed));

    // ── Request gating: FOUR terms, not ten ──────────────────────────
    wire row_start   = (req_x == {SRC_X_W{1'b0}}) && (req_phase == 2'd0);
    wire credit_ok   = !row_start || (credits != {CRED_W{1'b0}});
    wire can_request = (state == S_RUN) && fetch_placement_valid
                    && !placement_changed && !req_addr_oob && credit_ok;
    wire fb_req_accept    = fb_rd_en && fb_rd_ready;
    wire fb_req_slot_open = !fb_rd_en || fb_req_accept;
    wire req_take         = fb_req_slot_open && can_request;

    // Consumed only when the walk owns them; inside the window they are
    // stragglers from the abandoned walk.
    wire in_discard  = (state == S_RESYNC) || (state == S_FILL);
    wire rsp_consume = fb_rd_valid && !in_discard;
    wire rsp_stale   = fb_rd_valid && in_discard;
    // Closes on the COUNT or the TIMEOUT; the timeout is the load-bearing
    // half, since a lost response means the count never reaches zero.
    wire stale_done = (stale_count == {OUTST_W{1'b0}})
                    || (rsp_stale && (stale_count == {{(OUTST_W-1){1'b0}}, 1'b1}))
                    || (discard_timer == {DISC_W{1'b0}});

    // CLAMPED AT ZERO: an unmatched response (fb_reader can emit one across
    // the split pclk/vram_clk reset domains) would otherwise wrap the counter
    // and the next resync would latch THAT as stale_count.
    wire outst_inc = fb_req_accept && !fb_rd_valid;
    wire outst_dec = !fb_req_accept && fb_rd_valid
                     && (outstanding != {OUTST_W{1'b0}});
    wire [OUTST_W-1:0] outstanding_next =
        outst_inc ? (outstanding + {{(OUTST_W-1){1'b0}}, 1'b1}) :
        outst_dec ? (outstanding - {{(OUTST_W-1){1'b0}}, 1'b1}) :
                    outstanding;

    // ── Line-memory write port ───────────────────────────────────────
    wire [LINE_MEM_AW-1:0] rsp_slot_base =
        {{(LINE_MEM_AW-LINE_COUNT_LOG2){1'b0}}, rsp_slot} * SRC_W;
    assign wr_addr  = rsp_slot_base + {{(LINE_MEM_AW-SRC_X_W){1'b0}}, rsp_x};
    assign wr_en_p0 = rsp_consume && (fetch_wide || (rsp_phase == 2'd0));
    assign wr_en_p1 = rsp_consume && (fetch_wide || (rsp_phase == 2'd1));
    assign wr_en_p2 = rsp_consume && (fetch_wide || (rsp_phase == 2'd2));
    // Wide takes R/G/B from [23:16]/[15:8]/[7:0] in one cycle (the pad byte is
    // never consumed); narrow takes the ALWAYS-valid top lane.
    assign wr_d0 = fetch_wide ? fb_rd_data[23:16] : fb_rd_data[31:24];
    assign wr_d1 = fetch_wide ? fb_rd_data[15:8]  : fb_rd_data[31:24];
    assign wr_d2 = fetch_wide ? fb_rd_data[7:0]   : fb_rd_data[31:24];

`ifdef VERILATOR
    always @(*) begin
        if (resetn && (state == S_RUN) && fetch_placement_valid && req_addr_oob) begin
            $fatal(1, "scanout_fetch read address out of range: addr=%0d limit=%0d addr_w=%0d",
                   req_addr_c, req_addr_limit, ADDR_W);
        end
    end

    // ── THE MIGRATION'S OWN REGRESSION TEST ───────────────────────────
    // The admission gates this module used to apply for itself, preserved
    // verbatim behind `ifdef VERILATOR and checked against the authority every
    // simulated cycle.  Free in synthesis; a flipped operator or a units slip
    // in mode_admit.v kills the sim here with the term named.
    //
    // Note `legacy_stride_sane` is the STRICT `>` form over an inclusive last
    // offset while the authority uses `>=` over a count.  They are the same
    // predicate -- see mode_admit.v's operator derivation -- and this
    // assertion is what keeps that true rather than merely asserted.
    // Rebuilt from the LATCHED RAW tuple using the pre-migration expressions
    // verbatim -- NOT from any authority output -- so this really is an
    // independent second opinion and not a restatement of the first.
    wire legacy_direct_l = (fetch_bytes_per_px == 3'd4);
    wire [SRC_X_W-1:0] legacy_row_px_last =
        ((fetch_hres_px == 12'd0) || (fetch_hres_px >= SRC_W))
            ? (SRC_W - 1) : (fetch_hres_px[SRC_X_W-1:0] - 1'b1);
    wire [SRC_X_W-1:0] legacy_elem_idx =
        legacy_direct_l ? legacy_row_px_last
                        : (legacy_row_px_last >> fetch_bpp_shift);
    wire [SRC_Y_W-1:0] legacy_row_y_last =
        ((fetch_vres_px == 12'd0) || (fetch_vres_px >= SRC_H))
            ? (SRC_H - 1) : (fetch_vres_px[SRC_Y_W-1:0] - 1'b1);
    wire [ADDR_W-1:0] legacy_elem_zx =
        {{(ADDR_W-SRC_X_W){1'b0}}, legacy_elem_idx};
    wire [ADDR_W-1:0] legacy_row_last_off =
        legacy_direct_l ? ((legacy_elem_zx << 2) + {{(ADDR_W-2){1'b0}}, 2'd3})
                        : legacy_elem_zx;
    wire [FRAME_ADDR_W-1:0] legacy_req_addr_limit =
        {{(FRAME_ADDR_W-ADDR_W){1'b0}}, fetch_base_px}
      + ({{(FRAME_ADDR_W-ADDR_W){1'b0}}, fetch_stride_px}
         * {{(FRAME_ADDR_W-SRC_Y_W){1'b0}}, legacy_row_y_last})
      + {{(FRAME_ADDR_W-ADDR_W){1'b0}}, legacy_row_last_off};
    wire legacy_stride_sane  = (fetch_stride_px > legacy_row_last_off);
    wire legacy_frame_in_mem = (legacy_req_addr_limit < FB_PIXEL_LIMIT);

    always @(*) begin
        if (resetn) begin
            if (legacy_row_last_off !== row_last_off)
                $fatal(1, "[scanout_fetch] mode_admit row_last_off_bytes=%0d, pre-migration row_last_off=%0d",
                       row_last_off, legacy_row_last_off);
            if (legacy_elem_idx !== fetch_row_px_last)
                $fatal(1, "[scanout_fetch] mode_admit row_last_elem_idx=%0d, pre-migration row_last_idx=%0d",
                       fetch_row_px_last, legacy_elem_idx);
            if (legacy_row_y_last !== fetch_row_y_last)
                $fatal(1, "[scanout_fetch] mode_admit last_row=%0d, pre-migration row_y_last=%0d",
                       fetch_row_y_last, legacy_row_y_last);
            if (legacy_req_addr_limit !== req_addr_limit)
                $fatal(1, "[scanout_fetch] mode_admit frame_last_addr_bytes=%0d, pre-migration req_addr_limit=%0d",
                       req_addr_limit, legacy_req_addr_limit);
            if ((legacy_stride_sane && legacy_frame_in_mem) !== fetch_placement_valid)
                $fatal(1, "[scanout_fetch] mode_admit addr_admissible=%0d but the pre-migration gates say %0d (stride_sane=%0d in_mem=%0d)",
                       fetch_placement_valid,
                       (legacy_stride_sane && legacy_frame_in_mem),
                       legacy_stride_sane, legacy_frame_in_mem);
        end
    end
`endif

    integer li;
    always @(posedge pclk) begin
        if (resetn) begin
            outstanding <= outstanding_next;

            // Credits saturate at LINE_COUNT: the ring cannot hold more, and
            // the saturation is what stops the counter wrapping.
            if (credit_return && !(req_take && row_start)) begin
                if (credits != CRED_MAX)
                    credits <= credits + {{(CRED_W-1){1'b0}}, 1'b1};
            end else if (!credit_return && req_take && row_start) begin
                credits <= credits - {{(CRED_W-1){1'b0}}, 1'b1};
            end

            // ── Request walk ─────────────────────────────────────────
            if (req_take) begin
                fb_rd_en   <= 1'b1;
                fb_rd_addr <= req_addr_c[ADDR_W-1:0];
                if (row_start) begin
                    line_valid[req_slot] <= 1'b0;
                    line_tag[req_slot]   <= req_y;
                end
                if (req_phase != last_phase) begin
                    req_phase <= req_phase + 2'd1;
                end else begin
                    req_phase <= 2'd0;
                    if (req_x == req_x_last) begin
                        req_x <= {SRC_X_W{1'b0}};
                        // `>=`, not `==`: an equality terminator can only stop
                        // a walk that lands exactly on the bound.
                        if (req_y >= fetch_row_y_last) state <= S_IDLE;
                        else req_y <= req_y + {{(SRC_Y_W-1){1'b0}}, 1'b1};
                    end else begin
                        req_x <= req_x + {{(SRC_X_W-1){1'b0}}, 1'b1};
                    end
                end
            end else if (fb_req_slot_open) begin
                fb_rd_en <= 1'b0;
            end

            // ── Response walk ────────────────────────────────────────
            if (rsp_consume) begin
                if (rsp_phase != last_phase) begin
                    rsp_phase <= rsp_phase + 2'd1;
                end else begin
                    rsp_phase <= 2'd0;
                    if (rsp_x == req_x_last) begin
                        line_tag[rsp_slot]   <= rsp_y;
                        line_valid[rsp_slot] <= 1'b1;
                        rsp_x <= {SRC_X_W{1'b0}};
                        if (rsp_y < fetch_row_y_last)
                            rsp_y <= rsp_y + {{(SRC_Y_W-1){1'b0}}, 1'b1};
                    end else begin
                        rsp_x <= rsp_x + {{(SRC_X_W-1){1'b0}}, 1'b1};
                    end
                end
            end

            // ── Discard window ───────────────────────────────────────
            if (in_discard) begin
                if (rsp_stale && (stale_count != {OUTST_W{1'b0}}))
                    stale_count <= stale_count - {{(OUTST_W-1){1'b0}}, 1'b1};
                if (discard_timer != {DISC_W{1'b0}})
                    discard_timer <= discard_timer - {{(DISC_W-1){1'b0}}, 1'b1};
                if (stale_done) begin
                    state <= S_RUN;
                    // Back to truth on exit: nothing is really in flight now.
                    // Stops a lost response leaving a permanent +1 offset.
                    outstanding <= {OUTST_W{1'b0}};
                end else begin
                    state <= S_FILL;
                end
            end
        end else begin
            outstanding <= {OUTST_W{1'b0}};
            fb_rd_addr  <= {ADDR_W{1'b0}};
        end

        // Hard resync.  Reset IS a resync with the accounting zeroed, so the
        // two share ONE block and cannot drift apart.  Last, so it wins.
        if (!resetn || resync_req) begin
            for (li = 0; li < LINE_COUNT; li = li + 1)
                line_valid[li] <= 1'b0;
            req_y             <= {SRC_Y_W{1'b0}};
            req_x             <= {SRC_X_W{1'b0}};
            req_phase         <= 2'd0;
            rsp_y             <= {SRC_Y_W{1'b0}};
            rsp_x             <= {SRC_X_W{1'b0}};
            rsp_phase         <= 2'd0;
            credits           <= CRED_MAX;
            fb_rd_en          <= 1'b0;   // an un-accepted request is dropped
            discard_timer     <= DISC_LOAD;
            stale_count       <= resetn ? outstanding_next : {OUTST_W{1'b0}};
            // Skip the window when nothing is owed (the healthy steady
            // state); those two cycles are real at unit-tb geometries.
            state <= !resetn                                ? S_IDLE
                   : (outstanding_next == {OUTST_W{1'b0}})  ? S_RUN
                                                            : S_RESYNC;
            fetch_base_px     <= fb_base_px;
            fetch_stride_px   <= fb_stride_px;
            fetch_direct      <= direct_in;
            fetch_wide        <= wide_ok_in;
            fetch_px_shift    <= px_shift_in;
            fetch_hres_px      <= hres;
            fetch_vres_px      <= vres;
            fetch_bpp_shift    <= bpp_shift;
            fetch_bytes_per_px <= bytes_per_px;
        end
    end

endmodule

`default_nettype wire
