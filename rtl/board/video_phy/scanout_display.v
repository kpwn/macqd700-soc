// scanout_display.v -- pclk display half of linebuf_scanout: the LINE STORE
// read port, plus the wiring harness for scan-out stages 9-12.
// ---------------------------------------------------------------------------
// This file used to hold FIVE jobs -- scaling, centring, CLUT lookup, pixel
// unpack and boot-logo substitution -- and docs/video_path_review.md S4.1
// names it the canonical jack of all trades.  Those five are now four modules
// with typed, unit-explicit contracts, and what is left here is the one job
// the review assigns to this side of the CDC (stage 8, read side) plus the
// harness that connects them:
//
//     rtl/board/video_phy/upscale.v       stage 11  replication (source walk)
//     rtl/board/video_phy/pixel_unpack.v  stage  9  bit-depth unpacking
//     rtl/board/video_phy/clut.v          stage 10  palette + its write port
//     rtl/board/video_phy/compositor.v    stage 12  centring + the boot logo
//
// place_plan.v (stage 4) is entered ONCE, here, and its answer is handed to
// the stages that need it.  No stage below re-derives a resolution, a scale
// factor or a window origin.
//
// WHAT THIS FILE STILL OWNS
//   * the source-line memory (three byte planes) and its READ port;
//   * the read address generator, `slot_base + (x_src >> bpp_shift)`;
//   * the two read-pipeline registers the BRAM needs;
//   * the ring coupling -- `disp_y`, `credit_return`, and the delays that
//     align them with the read depth;
//   * the underflow health report.
// scanout_fetch.v owns the ring's tag/valid state and the WRITE port.
//
// >>> DESIGN RATIONALE: docs/scanout_credit_ring.md.
//
// The two halves talk through three things only: `disp_y` (the source row
// being read) with the ring's `disp_ready`/`disp_slot` answer, and
// `credit_return` -- ONE credit, returned UNCONDITIONALLY each time the
// display advances past a source row.  That is the only thing pacing the
// fetcher; there is no `line_tag < y_src` comparison left for a flag to
// suppress.  See the credit_return comment below: it is derived from the
// ADVANCE EVENT, not from a change in y_src, and that distinction is
// load-bearing (doc S1).
//
// LINE BUFFER: three byte-wide planes, not one 24-bit array -- a BRAM-cost
// decision.  RAMB36E2 trades width against depth in fixed steps, so at the
// production geometry (64 x 1024 entries) one 24-bit array costs 64 RAMB36
// while three 8-bit planes cost 48.  Plane 0 carries the indexed byte at
// 1/2/4/8bpp and R at 24bpp; planes 1/2 carry G/B and are untouched at the
// indexed depths.  Read side concatenates {p0,p1,p2}; pixel_unpack.v is where
// that convention is interpreted.
//
// ===========================================================================
// >>> THE LATENCY LADDER -- FIVE STAGES, AND EVERY PIPE MUST MATCH
// ===========================================================================
// From a combinational `x_src` to `rgb` there are exactly FIVE registered
// stages, and they now span three files:
//
//   stage 1  line_rd_addr                       (this file)
//   stage 2  pix_data_pipe    <= line_mem[]     (this file, BRAM read)
//   stage 3  pix_data_pipe2                     (this file, BRAM DOREG fold)
//   stage 4  clut.rgb / clut.bypass_rgb         (clut.v)
//   stage 5  compositor.rgb                     (compositor.v)
//
// Everything sampled combinationally at stage 0 and consumed at the output
// mux must therefore be delayed by FOUR before that final register:
// de / hs / vs / border / pix_valid / splash are the compositor's
// CTRL_DELAY = 4 pipes, and `x_src[2:0]` is `x_src_lo_pipe` here -- three
// stages, because it is consumed COMBINATIONALLY by pixel_unpack at stage 3's
// output, one stage earlier than the compositor's mux.
//
// The ring-facing delays match the same ladder from the other side:
// `disp_y` trails `y_src` by ONE (matching line_rd_addr -> pix_data_pipe;
// the live y_src would gate each row's trailing pixels against the NEXT
// row's readiness and lose them), and `credit_return` trails `y_advance` by
// TWO, so a slot is released only after that row's last read is issued.
//
// This ladder was hand-verified during a hardware investigation
// (docs/video_path_review.md S6, "Not display-pipeline misalignment").  An
// off-by-one in ANY of these pipes lands precisely at `border_x` -- the first
// pixels of a line -- which is a symptom that has actually been reported off
// hardware.  If you add or remove a stage, move ALL of them together, and
// re-run `make tb-scanout-display-latency`, which measures each pipe's depth
// independently and fails naming the one that slipped.
// ===========================================================================
//
// DSP: none in this file.  The read address is `slot_base + (x_src >> shift)`
// where slot_base is `disp_slot * SRC_W` -- a shift when SRC_W is a power of
// two and a small constant-coefficient multiply otherwise, both feeding a
// register.  docs/video_path_review.md S4.1 names the ADDRESS path as the
// place DSPs belong, but the node it means is `fb_base + row * stride +
// row_off` in stages 3 and 6 (scanout_placement_sync.v / the fetch side),
// which is a full three-operand `(A+D)*B+C` over live geometry.  This one is
// an index into a fixed-geometry ring with an elaboration-time coefficient;
// it is the mux side of the line, like place_plan.v's.
// ---------------------------------------------------------------------------
`default_nettype none

module scanout_display #(
    parameter SRC_W           = 1024,
    parameter SRC_H           = 768,
    parameter DST_W           = 1920,
    parameter DST_H           = 1080,
    parameter LINE_COUNT_LOG2 = 6,
    // Boot-splash pixel replication factor.  POWER OF TWO ONLY -- the walk
    // divides by it with a right shift.  4 => 128x128 on screen, 8 => 256x256.
    //
    // 0 (the default) means "derive it from DST_H" -- see auto_splash_scale
    // below and SPLASH_SCALE_EFF, which is what actually reaches the
    // compositor.  A nonzero value here is an explicit override that bypasses
    // the derivation entirely, e.g. for a testbench that wants a fixed size
    // regardless of DST_H.
    parameter SPLASH_SCALE    = 0,
    // Derived widths -- never overridden by the wrapper.
    parameter SRC_X_W         = $clog2(SRC_W),
    parameter SRC_Y_W         = $clog2(SRC_H),
    parameter LINE_MEM_AW     = $clog2((1 << LINE_COUNT_LOG2) * SRC_W)
) (
    input  wire                       pclk,
    input  wire                       resetn,

    input  wire [11:0]                hcount,
    input  wire [10:0]                vcount,
    input  wire                       de_in,
    input  wire                       hs_in,
    input  wire                       vs_in,
    input  wire                       sof,

    input  wire [2:0]                 bpp_shift,
    input  wire [11:0]                hres,
    input  wire [11:0]                vres,
    // Committed INTEGER output scale: N display pixels per source pixel on
    // both axes (see place_plan.v for the policy and scanout_placement_sync.v
    // for the frame-boundary commit).  1..4; 0 means "no committed N, derive
    // it from hres/vres" and is what an unconnected/pre-commit driver gives.
    input  wire [2:0]                 scale_n,
    input  wire                       fetch_direct,
    // 1 once the DAFB has committed a renderable placement (sticky, and only
    // ever changes on a frame boundary -- see scanout_placement_sync.v).
    // While 0, the compositor shows the boot splash instead of the pixel
    // path.  Purely a display-side input: it reaches nothing on the fetch
    // side of this module.
    input  wire                       dafb_live,

    // -- Ring coupling ------------------------------------------------
    output wire [11:0]                disp_y,
    input  wire                       disp_ready,
    input  wire [LINE_COUNT_LOG2-1:0] disp_slot,
    output wire                       credit_return,

    // -- Line-memory write port (from scanout_fetch) ------------------
    input  wire                       wr_en_p0,
    input  wire                       wr_en_p1,
    input  wire                       wr_en_p2,
    input  wire [LINE_MEM_AW-1:0]     wr_addr,
    input  wire [7:0]                 wr_d0,
    input  wire [7:0]                 wr_d1,
    input  wire [7:0]                 wr_d2,

    // -- 256-entry RAMDAC CLUT write port (clut_wclk domain) ----------
    input  wire                       clut_wclk,
    input  wire                       clut_we,
    input  wire [7:0]                 clut_waddr,
    input  wire [23:0]                clut_wdata,

    output wire [23:0]                rgb,
    output wire                       de_out,
    output wire                       hs_out,
    output wire                       vs_out,
    output reg                        line_underflow_sticky
);

    localparam LINE_MEM_DEPTH = (1 << LINE_COUNT_LOG2) * SRC_W;
    localparam LINE_DATA_W    = 24;
    // Registered stages the pixel data crosses before the compositor's output
    // register: line_rd_addr, pix_data_pipe, pix_data_pipe2, clut.  The
    // compositor's control pipes are this deep too -- see the LATENCY LADDER
    // note above.  ONE constant, threaded to every consumer.
    localparam CTRL_DELAY     = 4;
    // x_src[2:0] is consumed COMBINATIONALLY by pixel_unpack off the stage-3
    // register, so it needs one stage FEWER than the compositor's pipes.
    localparam XLO_DELAY      = CTRL_DELAY - 1;

    // -- Stage 4: placement, entered ONCE -----------------------------
    // place_plan.v is the single implementation of the scaling policy; this
    // module re-enters the COMMITTED N so it cannot disagree with the stage
    // that committed it (scanout_placement_sync.v drives scale_n_in = 0 to
    // ASK the policy; we drive the answer back in).  Committed geometry and
    // committed N therefore cannot disagree about where the window is.
    wire [2:0]  plan_scale_n;
    wire [12:0] active_w;
    wire [12:0] active_h;
    wire [11:0] border_x;
    wire [11:0] border_y;

    place_plan #(
        .DST_W (DST_W),
        .DST_H (DST_H)
    ) u_place_plan (
        .hres_px     (hres),
        .vres_px     (vres),
        .scale_n_in  (scale_n),
        .scale_n     (plan_scale_n),
        .active_w_px (active_w),
        .active_h_px (active_h),
        .border_x_px (border_x),
        .border_y_px (border_y)
    );

    // -- Raster-timing strobes (VTG-derived, no geometry) -------------
    wire sol             = (hcount == 12'd0);
    wire active_line_end = de_in && (hcount == DST_W - 1);

    // -- Stage 12 window decision, consumed here and by stage 11 ------
    wire in_active_x;
    wire in_active_y;
    wire in_border;

    wire rd_req    = de_in & ~in_border;
    wire pix_valid = rd_req & disp_ready;

    // -- Stage 11: the source-coordinate walk (replication) -----------
    wire [SRC_X_W-1:0] x_src;
    wire [SRC_Y_W-1:0] y_src;
    wire               y_advance;

    upscale #(
        .SRC_W   (SRC_W),
        .SRC_H   (SRC_H),
        .SRC_X_W (SRC_X_W),
        .SRC_Y_W (SRC_Y_W)
    ) u_upscale (
        .clk       (pclk),
        .resetn    (resetn),
        .scale_n   (plan_scale_n),
        .x_rst     (sol),
        // The ONE bit of window knowledge stage 11 gets.  `in_active_x &&
        // ~in_border` is `in_active_x && in_active_y` by construction; kept
        // in the form the pre-split code used so the equivalence is visible.
        .x_step_en (in_active_x && ~in_border),
        .y_rst     (sof),
        .y_step_en (active_line_end && in_active_y),
        .x_src     (x_src),
        .y_src     (y_src),
        .y_advance (y_advance)
    );

    // -- Ring coupling: y_src / y_advance, delayed to the read depth --
    reg [SRC_Y_W-1:0] y_src_pipe1;
    reg               y_advance_d1;
    reg               y_advance_d2;

    assign disp_y        = {{(12-SRC_Y_W){1'b0}}, y_src_pipe1};
    assign credit_return = y_advance_d2;

    // -- Underflow reporting gate -------------------------------------
    // `line_underflow_sticky` is a STICKY health signal, so it must not latch
    // on the unavoidable transient right after a reset or a placement change,
    // when the ring is legitimately empty.  Arming rule: a frame reports
    // underflow iff the PREVIOUS frame proved the ring works by delivering at
    // least one pixel.  That is strictly stronger than the old
    // `display_primed` (which armed on "source row 0 happens to be resident at
    // sof" and, with the ring now cleared AT sof, would never arm at all --
    // silently disabling the very thing it gates).  A wholly blank frame is
    // now reported, which display_primed could not do.
    reg underflow_armed;
    reg frame_delivered;

    // -- Stage 8 (read side): line memory, three byte planes ----------
    (* ram_style = "block" *) reg [7:0] line_mem_p0 [0:LINE_MEM_DEPTH-1];
    (* ram_style = "block" *) reg [7:0] line_mem_p1 [0:LINE_MEM_DEPTH-1];
    (* ram_style = "block" *) reg [7:0] line_mem_p2 [0:LINE_MEM_DEPTH-1];

    wire [LINE_MEM_AW-1:0] display_slot_base =
        {{(LINE_MEM_AW-LINE_COUNT_LOG2){1'b0}}, disp_slot} * SRC_W;
    // At 1bpp x_src/8 selects the source byte (8 x_src values share it,
    // decoded into sub-byte slices by pixel_unpack); at 8bpp the shift is 0.
    wire [SRC_X_W-1:0] x_src_byte = x_src >> bpp_shift;
    wire [LINE_MEM_AW-1:0] display_line_rd_addr =
        display_slot_base + {{(LINE_MEM_AW-SRC_X_W){1'b0}}, x_src_byte};

`ifdef VERILATOR
    always @(*) begin
        if (resetn && ((display_line_rd_addr >= LINE_MEM_DEPTH)
            || (wr_addr >= LINE_MEM_DEPTH))) begin
            $fatal(1, "scanout_display line RAM address out of range: rd=%0d wr=%0d depth=%0d",
                   display_line_rd_addr, wr_addr, LINE_MEM_DEPTH);
        end
    end
`endif

    // pix_data_pipe2 stays its own FF: Vivado folds it INTO the line-buffer
    // BRAM as DOREG ([Synth 8-7052]), off the pclk critical path.
    reg [LINE_MEM_AW-1:0]  line_rd_addr;
    reg [LINE_DATA_W-1:0]  pix_data_pipe;
    reg [LINE_DATA_W-1:0]  pix_data_pipe2;
    // XLO_DELAY stages of 3 bits each, oldest in the top slice.
    reg [3*XLO_DELAY-1:0]  x_src_lo_pipe;

    always @(posedge pclk) begin
        if (!resetn) begin
            y_src_pipe1     <= {SRC_Y_W{1'b0}};
            y_advance_d1    <= 1'b0;
            y_advance_d2    <= 1'b0;
            underflow_armed <= 1'b0;
            frame_delivered <= 1'b0;
            line_rd_addr    <= {LINE_MEM_AW{1'b0}};
            pix_data_pipe   <= {LINE_DATA_W{1'b0}};
            pix_data_pipe2  <= {LINE_DATA_W{1'b0}};
            x_src_lo_pipe   <= {(3*XLO_DELAY){1'b0}};
            line_underflow_sticky <= 1'b0;
        end else begin
            if (sof) begin
                underflow_armed <= frame_delivered;
                frame_delivered <= 1'b0;
            end

            // -- Line-memory write port (fetch side) ------------------
            if (wr_en_p0) line_mem_p0[wr_addr] <= wr_d0;
            if (wr_en_p1) line_mem_p1[wr_addr] <= wr_d1;
            if (wr_en_p2) line_mem_p2[wr_addr] <= wr_d2;

            // -- Read pipeline (ladder stages 1-3) --------------------
            line_rd_addr   <= display_line_rd_addr;
            pix_data_pipe  <= {line_mem_p0[line_rd_addr],
                               line_mem_p1[line_rd_addr],
                               line_mem_p2[line_rd_addr]};
            pix_data_pipe2 <= pix_data_pipe;
            x_src_lo_pipe  <= {x_src_lo_pipe[3*XLO_DELAY-4:0], x_src[2:0]};

            // -- Ring-facing delays -----------------------------------
            y_src_pipe1  <= y_src;
            y_advance_d1 <= y_advance;
            y_advance_d2 <= y_advance_d1;

            if (pix_valid)
                frame_delivered <= 1'b1;
            if (underflow_armed && rd_req && !disp_ready)
                line_underflow_sticky <= 1'b1;
        end
    end

    // -- Stage 9: bit-depth unpacking (pure, combinational) -----------
    wire [7:0]  clut_idx;
    wire [23:0] unpacked_direct_rgb;

    pixel_unpack u_pixel_unpack (
        .bpp_shift   (bpp_shift),
        .x_lo        (x_src_lo_pipe[3*XLO_DELAY-1:3*XLO_DELAY-3]),
        .plane_bytes (pix_data_pipe2),
        .index       (clut_idx),
        .direct_rgb  (unpacked_direct_rgb)
    );

    // -- Stage 10: the palette (ladder stage 4) -----------------------
    wire [23:0] clut_rgb;
    wire [23:0] direct_rgb;

    clut u_clut (
        .rclk       (pclk),
        .resetn     (resetn),
        .index      (clut_idx),
        .bypass_in  (unpacked_direct_rgb),
        .rgb        (clut_rgb),
        .bypass_rgb (direct_rgb),
        .wclk       (clut_wclk),
        .we         (clut_we),
        .waddr      (clut_waddr),
        .wdata      (clut_wdata)
    );

    // -- Boot-splash auto-scale ----------------------------------------
    // SPLASH_SCALE was a bare `4` for the whole life of this feature (see
    // 36188b07), chosen when the shipping mode was 1920x1080: 32*4 = 128 px,
    // 128/1080 = 11.9% of the frame height. Nothing recomputed it when
    // DST_H later moved to 720 (788cd2b0) -- 128/720 = 17.8%, roughly 50%
    // bigger on screen than the original design point, for a reason that has
    // nothing to do with the logo and everything to do with a display mode
    // change six files away. That is exactly the "constant sized for the old
    // resolution" failure mode: it happens to still FIT (720 lines comfortably
    // hold a 128 px box) so nothing failed loudly, it just silently drifted.
    //
    // auto_splash_scale reproduces the ORIGINAL 1080p point exactly
    // (1080/270 = 4) and rescales proportionally for any other DST_H, always
    // landing on a power of two as SPLASH_SCALE requires. SPLASH_SCALE == 0
    // (the default, see the parameter comment) selects this; a nonzero
    // SPLASH_SCALE is used verbatim, unchanged, for any caller that wants a
    // fixed size regardless of mode.
    function integer auto_splash_scale;
        input integer dst_h;
        integer q;
        begin
            q = dst_h / 270;
            if (q >= 8)      auto_splash_scale = 8;
            else if (q >= 4) auto_splash_scale = 4;
            else if (q >= 2) auto_splash_scale = 2;
            else             auto_splash_scale = 1;
        end
    endfunction

    localparam integer SPLASH_SCALE_EFF =
        (SPLASH_SCALE != 0) ? SPLASH_SCALE : auto_splash_scale(DST_H);

    // -- Stage 12: centring + boot logo (ladder stage 5) --------------
    compositor #(
        .SPLASH_SCALE (SPLASH_SCALE_EFF),
        .CTRL_DELAY   (CTRL_DELAY)
    ) u_compositor (
        .clk          (pclk),
        .resetn       (resetn),
        .hcount       (hcount),
        .vcount       (vcount),
        .de_in        (de_in),
        .hs_in        (hs_in),
        .vs_in        (vs_in),
        .sof          (sof),
        .active_w_px  (active_w),
        .active_h_px  (active_h),
        .border_x_px  (border_x),
        .border_y_px  (border_y),
        .dafb_live    (dafb_live),
        .in_active_x  (in_active_x),
        .in_active_y  (in_active_y),
        .in_border    (in_border),
        .pix_valid_in (pix_valid),
        .fetch_direct (fetch_direct),
        .clut_rgb     (clut_rgb),
        .direct_rgb   (direct_rgb),
        .rgb          (rgb),
        .de_out       (de_out),
        .hs_out       (hs_out),
        .vs_out       (vs_out)
    );

endmodule

`default_nettype wire
