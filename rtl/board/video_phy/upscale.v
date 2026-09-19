// upscale.v -- stage 11 of the scan-out pipeline: NEAREST-NEIGHBOUR REPLICATION.
// ---------------------------------------------------------------------------
// Owns pixel replication and NOTHING else.  docs/video_path_review.md S4.1
// stage 11: "rgb + scale_n -> replicated rgb.  Owns replication ONLY; has no
// idea what a border is."  It holds that contract literally -- the only
// geometry that reaches it is `scale_n`, and the only thing it knows about the
// active window is a single-bit STEP ENABLE handed to it by the compositor.
// It cannot compute a border, an active width or an origin; it has none of
// the terms.
//
// WHERE THE REPLICATION ACTUALLY HAPPENS, AND WHY IT IS HERE.
// The review draws stage 11 downstream of the CLUT, replicating finished RGB.
// This implementation replicates by holding the SOURCE COORDINATE still for N
// destination pixels instead, so the same source pixel is re-read and re-looked
// -up N times.  The rendered image is identical -- that is what nearest
// neighbour means -- and the address-side form is strictly better here:
//
//   * it needs no rate change.  An RGB-side replicator consumes source pixels
//     at 1/N the destination rate, so it needs an elastic buffer between
//     stages 10 and 11 and its depth becomes a function of N.  The
//     address-side form runs one read per destination pixel at a FIXED rate,
//     which is the property S4.1 asks for ("keep the pipeline constant, never
//     data-dependent");
//   * it therefore adds ZERO stages to the five-stage ladder.  An RGB-side
//     replicator would add at least one, and every parallel control pipe in
//     the compositor would have to move with it;
//   * it costs one accumulator instead of a 24-bit-wide holding register.
//
// So stage 11 sits UPSTREAM of the line-store read in dataflow order while
// remaining the module that owns replication.  That is a deliberate departure
// from the review's ordering, recorded here rather than silently.
//
// INTEGER ONLY.  This is a Bresenham that emits `scale_den` destination
// pixels per `scale_num` source pixels, with num pinned at a CONSTANT 1 and
// den = N.  That IS pixel replication, exactly and by construction: N=1
// advances every pixel, N=2 gives 0,0,1,1,...  The fractional rung is GONE --
// the old ladder had a 3:2 rung (num=2, den=3) whose 2,1,2,1 replication
// aliases the 1bpp 50%-dither Mac desktop into fine striping.
// docs/video_path_review.md S3 settles the policy as integer-only on
// AESTHETICS, not cost: sharp and smaller beats full-screen and soft.  Do not
// reintroduce a num != 1 rung.  `scale_num` is kept as a named wire so the
// accumulator still reads as the general Bresenham it is, and so anyone
// tempted to make it non-constant meets this paragraph first.
//
// ACCUMULATOR WIDTH, AND THE DSP QUESTION.  place_plan.v clamps N to
// MAX_SCALE_N = 4, so the accumulator only ever holds 0..N-1 <= 3 and
// `acc + 1` only ever reaches 4.  ACCW is sized from that bound rather than
// left at the 16 bits the fractional ladder needed, and an assertion below
// makes a future N > 4 loud instead of silently wrapping.
// docs/video_path_review.md S4.1's rule is that arithmetic wider than a mux
// should justify not being a DSP48E2.  This is a 4-bit modulo-N counter --
// narrower than the 4:1 multiply-mux place_plan.v already argued sits on the
// mux side of that line.  Spending a hard macro to save single-digit LUTs is
// not what the 1797 idle DSPs are for; the genuine `base + stride * row +
// offset` node the review names is in the ADDRESS path, and is marked for DSP
// inference at its own site in scanout_placement_sync.v.
//
// LATENCY: the outputs are registers, updated at most once per destination
// pixel.  This module inserts NO stage into the display data ladder -- `x_src`
// is consumed combinationally by the line-store address generator in
// scanout_display.v, exactly as it was before the split.
// ---------------------------------------------------------------------------
`default_nettype none

module upscale #(
    parameter SRC_W   = 1024,
    parameter SRC_H   = 768,
    parameter SRC_X_W = $clog2(SRC_W),
    parameter SRC_Y_W = $clog2(SRC_H)
) (
    input  wire                clk,
    input  wire                resetn,

    // Committed integer output scale: N destination pixels per source pixel
    // on both axes.  Comes from place_plan.v, which guarantees 1 <= N <= 4.
    input  wire [2:0]          scale_n,

    // Horizontal walk control.  `x_rst` is start-of-line; `x_step_en` is
    // "this destination pixel is inside the active window", the ONE bit of
    // window knowledge this module gets.
    input  wire                x_rst,
    input  wire                x_step_en,

    // Vertical walk control.  `y_rst` is start-of-frame; `y_step_en` is
    // "an active destination line just ended inside the active window".
    input  wire                y_rst,
    input  wire                y_step_en,

    output reg  [SRC_X_W-1:0]  x_src,
    output reg  [SRC_Y_W-1:0]  y_src,
    // Combinational: this line end retires a SOURCE ROW.  The consumer is the
    // credit ring; see scanout_display.v, which delays it to match the
    // line-store read depth.  Derived from the ADVANCE EVENT, not from a
    // change in y_src -- y_src also changes at the frame wrap, and that pulse
    // lands after the resync has reloaded the credit counter, leaving the
    // fetcher exactly ONE ring position too far ahead (doc S1).
    output wire                y_advance
);

    // Sized from place_plan.v's MAX_SCALE_N = 4: acc in [0, N-1] <= 3, and
    // acc + num reaches 4.  Four bits hold 0..15, so there is a full bit of
    // headroom above the reachable maximum.
    localparam ACCW = 4;

    reg [ACCW-1:0] x_acc;
    reg [ACCW-1:0] y_acc;

    // num is a CONSTANT 1 -- see the header.
    wire [ACCW-1:0] scale_num = {{(ACCW-1){1'b0}}, 1'b1};
    wire [ACCW-1:0] scale_den = {{(ACCW-3){1'b0}}, scale_n};

    wire x_wrap = ((x_acc + scale_num) >= scale_den);
    wire y_wrap = ((y_acc + scale_num) >= scale_den);

    assign y_advance = y_step_en && y_wrap && (y_src != SRC_H - 1);

`ifdef VERILATOR
    // The ACCW bound above is only sound while place_plan.v keeps clamping N.
    // Make a regression there loud rather than a silent accumulator wrap.
    always @(*) begin
        if (resetn && ((scale_n == 3'd0) || (scale_n > 3'd4))) begin
            $fatal(1, "upscale: scale_n=%0d outside the 1..4 place_plan clamp that ACCW=%0d is sized from",
                   scale_n, ACCW);
        end
    end
`endif

    always @(posedge clk) begin
        if (!resetn) begin
            x_acc <= {ACCW{1'b0}};
            x_src <= {SRC_X_W{1'b0}};
            y_acc <= {ACCW{1'b0}};
            y_src <= {SRC_Y_W{1'b0}};
        end else begin
            if (x_rst) begin
                x_acc <= {ACCW{1'b0}};
                x_src <= {SRC_X_W{1'b0}};
            end else if (x_step_en) begin
                if (x_wrap) begin
                    x_acc <= x_acc + scale_num - scale_den;
                    if (x_src != SRC_W - 1)
                        x_src <= x_src + {{(SRC_X_W-1){1'b0}}, 1'b1};
                end else begin
                    x_acc <= x_acc + scale_num;
                end
            end

            if (y_rst) begin
                y_acc <= {ACCW{1'b0}};
                y_src <= {SRC_Y_W{1'b0}};
            end else if (y_step_en) begin
                if (y_wrap) begin
                    y_acc <= y_acc + scale_num - scale_den;
                    if (y_src != SRC_H - 1)
                        y_src <= y_src + {{(SRC_Y_W-1){1'b0}}, 1'b1};
                end else begin
                    y_acc <= y_acc + scale_num;
                end
            end
        end
    end

endmodule

`default_nettype wire
