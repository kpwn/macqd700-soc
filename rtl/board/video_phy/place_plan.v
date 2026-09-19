// place_plan.v -- stage 4 of the scan-out pipeline: THE SCALING POLICY.
// ---------------------------------------------------------------------------
// PURE FUNCTION.  No clock, no reset, no state.  Given a source geometry in
// pixels and the destination raster size (DST_W x DST_H, elaboration-time),
// it answers the only two questions the display half has about placement:
//
//     how many display pixels per source pixel      -> scale_n
//     where does the centred active window sit      -> active_*_px, border_*_px
//
// >>> POLICY (settled 2026-08-20, docs/video_path_review.md S3):
// >>>
// >>>     N = min(floor(DST_W / hres), floor(DST_H / vres)), clamped to >= 1
// >>>     nearest-neighbour replication, INTEGER ONLY
// >>>
// >>> "Sharp and smaller beats full-screen and soft."  There is no fractional
// >>> ratio here and there must never be one again.  The deleted SCALE_3_2
// >>> rung existed to make the Q700 832x624 mode fill more of the 1080p frame
// >>> and it is the direct cause of a documented artifact: a 2,1,2,1 line
// >>> doubling aliases the 1bpp 50%-dither Mac desktop into fine striping
// >>> (the solid menu bar carries no dither, which is why row 0 alone looked
// >>> right).  Rational-WITHOUT-filtering is that same artifact at every
// >>> ratio; rational-WITH-filtering (bilinear ~12 DSPs at pixel rate,
// >>> against 1797 free) is affordable but softens pixel-exact content, which
// >>> is the wrong trade for a dithered 1bpp desktop.  The choice is on
// >>> AESTHETICS, not cost -- so "we can afford a scaler now" is not a reason
// >>> to revisit it.
//
// Applied to the Q700 mode set at 1080p:
//
//     source     N   rendered     letterbox
//     512x384    2   1024x768     896x312
//     640x480    2   1280x960     640x120
//     832x624    1    832x624     1088x456
//     1024x768   1   1024x768     896x312
//     1152x870   1   1152x870     768x210
//
// WHY NO DIVIDER, AND WHY NO DSP.  N is bounded above by 4: the smallest
// geometry the DAFB can present is 512x384 (N=2) and even a hypothetical
// 320x240 lands on 4.  So the policy is a three-term comparison ladder over
// {2,3,4} and `active = hres * N` is a 4:1 mux of {h, h<<1, h+(h<<1), h<<2}.
// docs/video_path_review.md S4.1's DSP rule is "anything WIDER THAN A MUX
// should be asking why it is not a DSP48E2"; this is the mux side of that
// line, and a pure combinational stage cannot use the DSP's A/B/M/P pipeline
// registers -- which is where a DSP's value in this path actually comes from.
// The genuine `base + stride * row + offset` multiply, which is what the
// review names as the hot node, is marked for DSP inference at its own site
// in scanout_placement_sync.v.
//
// TWO CALLERS, ONE IMPLEMENTATION.
//   * scanout_placement_sync drives scale_n_in = 0 ("apply the policy") and
//     COMMITS the resulting scale_n on a frame boundary.
//   * scanout_display is handed that COMMITTED scale_n and drives it back in
//     as scale_n_in, so it gets the active/border arithmetic from the same
//     module without re-deriving the policy.  Committed geometry and
//     committed N therefore cannot disagree about where the window is: there
//     is exactly one expression for it in the design.
//
// UNITS.  Every port is named `_px`.  Nothing here is a byte count and
// nothing here knows what a framebuffer stride is.
// ---------------------------------------------------------------------------
`default_nettype none

module place_plan #(
    parameter DST_W = 1920,
    parameter DST_H = 1080
) (
    // Source geometry (DAFB-decoded, in source pixels).
    input  wire [11:0] hres_px,
    input  wire [11:0] vres_px,
    // 0        -> apply the policy and report the answer on scale_n.
    // 1..4     -> use this N verbatim (a committed N being fed back in).
    // >4       -> clamped to 4; N is bounded by 4 by construction.
    input  wire [2:0]  scale_n_in,

    output wire [2:0]  scale_n,
    output wire [12:0] active_w_px,
    output wire [12:0] active_h_px,
    output wire [11:0] border_x_px,
    output wire [11:0] border_y_px
);

    // Largest N the ladder considers.  See the header: the reachable Q700
    // mode set tops out at 2 and nothing plausible reaches 5.
    localparam [2:0] MAX_SCALE_N = 3'd4;

    localparam [13:0] DST_W_PX = DST_W;
    localparam [13:0] DST_H_PX = DST_H;

    // hres * {1,2,3,4} and vres * {1,2,3,4} -- shifts and one add each.
    wire [13:0] h1_px = {2'b00, hres_px};
    wire [13:0] v1_px = {2'b00, vres_px};
    wire [13:0] h2_px = h1_px << 1;
    wire [13:0] v2_px = v1_px << 1;
    wire [13:0] h3_px = h2_px + h1_px;
    wire [13:0] v3_px = v2_px + v1_px;
    wire [13:0] h4_px = h1_px << 2;
    wire [13:0] v4_px = v1_px << 2;

    // "<=", not "<": a ratio that lands EXACTLY on the destination raster
    // fits.  960x540 at N=2 is the integer boundary case.
    wire fits_2x = (h2_px <= DST_W_PX) && (v2_px <= DST_H_PX);
    wire fits_3x = (h3_px <= DST_W_PX) && (v3_px <= DST_H_PX);
    wire fits_4x = (h4_px <= DST_W_PX) && (v4_px <= DST_H_PX);

    // A degenerate geometry (the pre-DAFB reset state, hres/vres still 0)
    // has an EMPTY active window whatever N says, so the ladder's "0 fits
    // every rung" answer is meaningless.  Pin it at 1 rather than let it
    // report the clamp ceiling: N is then never 0 and never surprising, and
    // scanout_display's Bresenham denominator is never zero.
    wire geom_zero_px = (hres_px == 12'd0) || (vres_px == 12'd0);

    wire [2:0] policy_n = geom_zero_px ? 3'd1
                        : fits_4x      ? 3'd4
                        : fits_3x      ? 3'd3
                        : fits_2x      ? 3'd2
                                       : 3'd1;

    wire [2:0] n_sel = (scale_n_in == 3'd0)       ? policy_n
                     : (scale_n_in > MAX_SCALE_N) ? MAX_SCALE_N
                                                  : scale_n_in;
    assign scale_n = n_sel;

    // active = hres * N.  4:1 mux over the multiples computed above.
    wire [13:0] aw_px = (n_sel == 3'd4) ? h4_px
                      : (n_sel == 3'd3) ? h3_px
                      : (n_sel == 3'd2) ? h2_px
                                        : h1_px;
    wire [13:0] ah_px = (n_sel == 3'd4) ? v4_px
                      : (n_sel == 3'd3) ? v3_px
                      : (n_sel == 3'd2) ? v2_px
                                        : v1_px;

    // Saturate rather than wrap.  Only reachable when a caller forces an N
    // the policy would never pick for a geometry wider than the raster; the
    // window is then simply the whole raster with no border, which is what
    // "does not fit" should look like on screen.
    assign active_w_px = aw_px[13] ? 13'h1FFF : aw_px[12:0];
    assign active_h_px = ah_px[13] ? 13'h1FFF : ah_px[12:0];

    // border = (DST - active) / 2, FLOORED.  Bit 0 of the slack is the odd
    // leftover column/row and is deliberately discarded -- it lands on the
    // right/bottom border, exactly as the pre-existing `(DST_W - active_w)
    // >> 1` did.  Bit 13 cannot be set on the arm that uses the result (the
    // ternary guard is the aw_px >= DST_W_PX case), so both are genuinely
    // dead and the lint waiver below names why rather than hiding it.
    /* verilator lint_off UNUSEDSIGNAL */
    wire [13:0] slack_x_px = DST_W_PX - aw_px;
    wire [13:0] slack_y_px = DST_H_PX - ah_px;
    /* verilator lint_on UNUSEDSIGNAL */
    assign border_x_px = (aw_px >= DST_W_PX) ? 12'd0 : slack_x_px[12:1];
    assign border_y_px = (ah_px >= DST_H_PX) ? 12'd0 : slack_y_px[12:1];

endmodule

`default_nettype wire
