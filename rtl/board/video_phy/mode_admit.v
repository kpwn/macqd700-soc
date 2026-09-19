// mode_admit.v -- stage 3 of the scan-out pipeline: THE ADMISSION AUTHORITY.
// ---------------------------------------------------------------------------
// PURE FUNCTION.  No clock, no reset, no state.  Given a canonical mode
// descriptor (stage 2, rtl/mac/mode_decode.v) and the memory limits of the
// framebuffer backing store, it answers exactly one question:
//
//     can the scanner render this, and if not, WHY
//
// >>> THIS IS THE ONLY PLACE IN THE VIDEO PATH WHERE AN ADMISSION INEQUALITY
// >>> IS ALLOWED TO EXIST.  docs/video_path_review.md S4.1 rule 6: "Only
// >>> stage 3 can say no, and it must say why."
//
// Everything downstream CONSUMES the verdict and the derived limits emitted
// here.  scanout_placement_sync.v and scanout_fetch.v used to each re-derive
// the same geometry and apply their own gates; both now instantiate this
// module, and both keep their former inline expressions ONLY under `ifdef
// VERILATOR` as equivalence assertions against it.  If the authority and the
// code it replaced ever disagree, simulation dies.  That is the migration's
// own regression test and it costs nothing in synthesis.
//
// ===========================================================================
// THE OPERATOR DERIVATION -- read this before touching any comparison
// ===========================================================================
//
// The review recorded the two stride gates as a units/operator conflict:
//
//     scanout_fetch.v        fetch_stride_px  >  row_last_off      (strict >)
//     scanout_placement_sync fb_stride_stable >= min_stride_bytes  (>=)
//
// They are NOT in conflict.  They are the SAME predicate written in two
// idioms whose right-hand sides differ by exactly one:
//
//     row_span_bytes == row_last_off_bytes + 1        (asserted below, A1)
//     (stride >= span)  <=>  (stride > span - 1)  <=>  (stride > last_off)
//
// Both were correct.  The hazard was never an off-by-one already in the tree
// -- it was that a reader "harmonising" them had two ways to get it wrong,
// and BOTH ways are a shipped bug:
//
//   * both `>` over the COUNT (stride > row_span_bytes) demands a pad byte on
//     every row.  A tightly packed framebuffer has stride == span EXACTLY, so
//     this rejects the entire Q700 mode set.  Black screen everywhere.
//   * both `>=` over the LAST OFFSET (stride >= row_last_off_bytes) admits
//     stride == span - 1, i.e. a row one byte short of its own pitch.  The
//     scanner then walks one byte into the next row on every line: a
//     cumulative horizontal shear, which is the "rightward drift" symptom.
//
// This is also why the review's alarm at 832x624 passing "with one byte of
// margin (3328 > 3327)" is a false alarm.  832 px of direct colour is
// 832*4 = 3328 bytes and its last byte sits at offset 3327.  The one-byte gap
// IS the count-vs-inclusive-offset difference and nothing else; the mode is an
// EXACT fit, which is the normal case for an unpadded framebuffer, not a
// coincidence.  A gate that did not admit it would be the bug.
//
// >>> CANONICAL FORM, one operator per comparison:
// >>>
// >>>   stride gate:  fb_stride_bytes >= row_span_bytes       COUNT vs COUNT
// >>>   memory gate:  frame_last_addr_bytes < FB_MAX_BYTES    ADDR  vs COUNT
//
// The two gates use DIFFERENT operators and that is correct, because their
// right-hand sides are different kinds of number.  The naming convention is
// what makes the right operator obvious at the call site, so it is a rule:
//
//     *_bytes        a COUNT / span / capacity.  Compare with >= or <=.
//     *_off_bytes    an INCLUSIVE offset within a row.
//     *_addr_bytes   an INCLUSIVE byte address.  Against a capacity: <.
//     *_px           a pixel count or pixel index.  Never a byte quantity.
//
// The stride gate is stated over counts rather than over the inclusive offset
// because the count form is TOTAL: `row_span_bytes` is >= 1 by construction,
// whereas the offset form needs a `- 1` that underflows on a zero-width row.
// The memory gate is stated over an inclusive address because that is what
// `base + stride*last_row + row_last_off` physically is -- the last byte the
// scanner will read.  With FB_MAX_BYTES a capacity, the highest legal address
// is FB_MAX_BYTES-1, so `last < capacity` is the exact statement and `<=`
// would admit one byte past the aperture.
//
// ===========================================================================
// UNITS, AND THE ASSERTION THAT THEY AGREE
// ===========================================================================
//
// S4.1 rule 2 requires this stage to carry a Verilator assertion that its
// byte-domain and pixel-domain derivations agree.  The block at the bottom of
// this file has five, and none is a tautology -- each computes a quantity a
// second, independent way:
//
//   A1  row_span_bytes == row_last_off_bytes + 1
//         the count and the inclusive offset are exactly one apart, which is
//         what makes the two historical operators the same predicate.
//   A2  (stride >= span) == (stride > last_off)
//         the derivation above, as an executable statement.
//   A3  indexed span, two ways: ((row_px-1) >> s) + 1  ==  ceil(row_px / 2^s)
//         "highest byte index, plus one" against a pixel-domain ceiling
//         division.  Different expressions, same number.
//   A4  row_last_off_bytes agrees with the ELEMENT index times its stride
//         (x4+3 for direct colour, x1 for indexed) -- this is precisely the
//         expression scanout_fetch.v used to compute for itself, so A4 is the
//         bridge between the fetch idiom and the placement idiom.
//   A5  exact-depth cross-checks: at direct colour span == row_px*4, and at
//         8bpp (bpp_shift 0, one byte per pixel) span == row_px.
//
// ===========================================================================
// DSP
// ===========================================================================
//
// `base + stride*last_row + row_last_off` is an exact (A+D)*B+C fit for a
// DSP48E2 pre-adder, and the review names it as the arithmetic that should
// not be in LUTs (76.9% LUT / congestion 5-6 against 1797 idle DSPs).  Two
// things to know before changing the attribute below:
//
//   * There used to be TWO of this multiply -- one here (as
//     scanout_placement_sync's `frame_last_addr`) and a character-identical
//     one in scanout_fetch as `req_addr_limit`.  Promoting the authority
//     DELETES the second one outright.  That is a better outcome than
//     annotating it, and it is what closes the review's named follow-up.
//   * A PURE stage cannot use the DSP's A/B/M/P pipeline registers, which is
//     where most of a DSP's value in this path comes from.  The attribute is
//     still worth carrying (it relocates a ~20x10 multiply off congested
//     fabric) but it buys area, not depth.  The verdict must be
//     combinationally valid at every frame_start, so it cannot be pipelined
//     here.  See scanout_fetch.v's `req_addr_c` comment for the one multiply
//     in the path that IS a genuine registered-DSP candidate, and why the
//     real fix for it is stage 6 (an incremental accumulator, no multiplier
//     at all) rather than an attribute.
//
// ---------------------------------------------------------------------------
`default_nettype none

module mode_admit #(
    parameter ADDR_W       = 20,
    // Build-time scanner window.  A programmed geometry wider/taller than
    // this is CLAMPED to it, matching what the scanner will actually read.
    parameter SRC_W        = 1024,
    parameter SRC_H        = 768,
    // Byte capacity of the framebuffer backing store (vram.v's VRAM_BYTES,
    // 0x200000 on the Q700 build).  A CAPACITY, not a pixel count -- see the
    // operator derivation above.
    parameter FB_MAX_BYTES = SRC_W * SRC_H,
    // Derived -- never override at an instantiation.
    parameter FRAME_ADDR_W = ADDR_W + $clog2(SRC_H) + 1
) (
    // ── Canonical mode descriptor (stage 2) ──────────────────────────
    input  wire [11:0]              hres_px,
    input  wire [11:0]              vres_px,
    // log2(source pixels per byte) for the sub-byte indexed depths:
    // 1bpp=3, 2bpp=2, 4bpp=1, 8bpp=0.  Meaningless at direct colour.
    input  wire [2:0]               bpp_shift,
    // VRAM bytes per source pixel: 0 -> sub-byte (use bpp_shift), 1 -> 8bpp,
    // 4 -> 24bpp direct colour (xRGB).
    input  wire [2:0]               bytes_per_px,
    // 1 iff the scanner can render this depth at all.  Not an inequality --
    // it is stage 2's verdict, carried here so that ONE module owns the
    // whole "no", reason code included.
    input  wire                     depth_supported,

    // ── Memory placement, in bytes ───────────────────────────────────
    input  wire [ADDR_W-1:0]        fb_base_bytes,
    input  wire [ADDR_W-1:0]        fb_stride_bytes,

    // ── The verdict ──────────────────────────────────────────────────
    // Full verdict: depth AND geometry AND memory range.  Gate a placement
    // COMMIT on this.
    output wire                     renderable,
    // Geometry + memory range only, depth excluded.  This is what a consumer
    // that has no depth-support channel gates on -- scanout_fetch is handed
    // an already-committed tuple and cannot re-judge the depth, so judging it
    // would mean inventing an answer.
    output wire                     addr_admissible,
    // The two gates, broken out so a consumer's migration assertion can name
    // WHICH one it disagrees about instead of just failing the conjunction.
    output wire                     stride_fits,
    output wire                     frame_in_range,
    output wire [3:0]               reject_reason,
    // The REJ_NONE code as a constant, exported so a consumer can reset a
    // latched-reason register to "nothing was rejected" without re-encoding
    // the wire protocol locally.  Synthesises to a tie-off.
    output wire [3:0]               reason_none_code,

    // ── Derived limits, carried downstream so nothing recomputes them ──
    output wire [ADDR_W-1:0]        row_px,
    output wire [ADDR_W-1:0]        row_span_bytes,
    output wire [ADDR_W-1:0]        row_last_off_bytes,
    // Index of the last FETCH ELEMENT in a row, where an element is the fetch
    // port's addressing granule: one PIXEL at direct colour, one BYTE at the
    // indexed depths.  This is the one output whose unit is depth-dependent,
    // and that is exactly why it is derived here once rather than in the
    // fetcher: assertion A4 ties it to row_last_off_bytes, so the walk bound
    // and the byte footprint cannot drift apart.
    output wire [ADDR_W-1:0]        row_last_elem_idx,
    output wire [ADDR_W-1:0]        row_count,
    output wire [ADDR_W-1:0]        last_row,
    output wire [FRAME_ADDR_W-1:0]  frame_last_addr_bytes,
    output wire                     direct_colour,
    output wire                     geometry_zero
);

    // ── Reject reason encoding ────────────────────────────────────────
    // WIRE PROTOCOL.  tools/jtag_repl.tcl decodes these and
    // tb/tests/host/test_jtag_repl_helpers.tcl parses THIS FILE to prove the
    // decoder has not drifted from the RTL.  Do not renumber; append.
    localparam [3:0] REJ_NONE          = 4'd0;
    localparam [3:0] REJ_DEPTH_UNSUP   = 4'd1;  // 16/24bpp: scanner can't render
    localparam [3:0] REJ_STRIDE_SHORT  = 4'd2;  // stride < one visible row
    localparam [3:0] REJ_FRAME_OOM     = 4'd3;  // last byte past the aperture
    localparam [3:0] REJ_GEOMETRY_ZERO = 4'd4;  // admitted, but 0-pixel window

    localparam [ADDR_W-1:0]       SRC_W_ADDR    = SRC_W;
    localparam [ADDR_W-1:0]       SRC_H_ADDR    = SRC_H;
    localparam [ADDR_W-1:0]       ADDR_ONE      = {{(ADDR_W-1){1'b0}}, 1'b1};
    localparam [FRAME_ADDR_W-1:0] FB_BYTE_LIMIT = FB_MAX_BYTES;

    // ── Visible geometry, clamped to the scanner window ───────────────
    // A programmed hres/vres of 0 (pre-DAFB reset state) or one at/over the
    // build-time window falls back to the window itself, which is what the
    // scanner reads in that case.  Both axes had to learn this the hard way:
    // measuring an 832-px row or a 480-row frame against the ELABORATED
    // SRC_W/SRC_H was a measured hardware black screen in each direction.
    assign direct_colour = (bytes_per_px == 3'd4);

    assign row_px = ((hres_px != 12'd0) && (hres_px < SRC_W))
                        ? {{(ADDR_W-12){1'b0}}, hres_px}
                        : SRC_W_ADDR;
    assign row_count = ((vres_px != 12'd0) && (vres_px < SRC_H))
                        ? {{(ADDR_W-12){1'b0}}, vres_px}
                        : SRC_H_ADDR;
    assign last_row = row_count - ADDR_ONE;

    // ── Byte footprint of one visible row ─────────────────────────────
    // The indexed and direct arms are kept as separate expressions and
    // selected between, NOT folded into one shift-based formula: unifying
    // them silently changes the indexed answer whenever SRC_W is not a power
    // of two, because (SRC_W >> s) and ((SRC_W-1) >> s) + 1 differ there.
    wire [ADDR_W-1:0] indexed_row_last_off = (row_px - ADDR_ONE) >> bpp_shift;
    wire [ADDR_W-1:0] direct_row_span      = row_px << 2;

    assign row_span_bytes = direct_colour ? direct_row_span
                                          : (indexed_row_last_off + ADDR_ONE);
    assign row_last_off_bytes = direct_colour ? (direct_row_span - ADDR_ONE)
                                              : indexed_row_last_off;
    assign row_last_elem_idx = direct_colour ? (row_px - ADDR_ONE)
                                             : indexed_row_last_off;

    // ── The frame footprint: base + stride*last_row + row_last_off ────
    // The (A+D)*B+C node.  See the DSP block in the header.
    // The attribute sits on the WIRE DECLARATION, not on the `assign` -- that
    // is the form 7c23fcd proved through Vivado, and the one XST accepts.
    (* use_dsp = "yes" *)
    wire [FRAME_ADDR_W-1:0] frame_last_addr_int =
        {{(FRAME_ADDR_W-ADDR_W){1'b0}}, fb_base_bytes}
      + ({{(FRAME_ADDR_W-ADDR_W){1'b0}}, fb_stride_bytes}
         * {{(FRAME_ADDR_W-ADDR_W){1'b0}}, last_row})
      + {{(FRAME_ADDR_W-ADDR_W){1'b0}}, row_last_off_bytes};
    assign frame_last_addr_bytes = frame_last_addr_int;

    // ── THE TWO INEQUALITIES.  There are no others in the video path. ──
    assign stride_fits    = (fb_stride_bytes >= row_span_bytes);
    assign frame_in_range = (frame_last_addr_bytes < FB_BYTE_LIMIT);

    assign addr_admissible = stride_fits && frame_in_range;
    assign renderable      = depth_supported && addr_admissible;

    // A degenerate 0-pixel window is ADMITTED (the reset / pre-DAFB path
    // depends on that) but it renders nothing, so it is named rather than
    // left to be inferred from hres==0 in a hex dump.
    assign geometry_zero = (hres_px == 12'd0) || (vres_px == 12'd0);

    // FIRST failing gate, in evaluation order, so a reader gets a cause
    // rather than a bitmask they have to prioritise themselves.
    assign reject_reason = (!depth_supported) ? REJ_DEPTH_UNSUP  :
                           (!stride_fits)     ? REJ_STRIDE_SHORT :
                           (!frame_in_range)  ? REJ_FRAME_OOM    :
                           (geometry_zero)    ? REJ_GEOMETRY_ZERO
                                              : REJ_NONE;
    assign reason_none_code = REJ_NONE;

`ifdef VERILATOR
    // ── Byte-domain / pixel-domain agreement (S4.1 rule 2) ────────────
    // See the header for what each of these independently re-derives.  These
    // are the assertions that make the operator choice above checkable rather
    // than merely argued.
    wire [ADDR_W-1:0] shift_one   = ADDR_ONE << bpp_shift;
    wire [ADDR_W-1:0] ceil_div_px = (row_px + shift_one - ADDR_ONE) >> bpp_shift;

    always @(*) begin
        // A1 -- the count and the inclusive offset are exactly one apart.
        if (row_span_bytes != (row_last_off_bytes + ADDR_ONE))
            $fatal(1, "[mode_admit] A1: row_span_bytes=%0d != row_last_off_bytes+1=%0d",
                   row_span_bytes, row_last_off_bytes + ADDR_ONE);

        // A2 -- the two historical operator idioms decide identically.
        if (stride_fits != (fb_stride_bytes > row_last_off_bytes))
            $fatal(1, "[mode_admit] A2: (stride>=span)=%0d disagrees with (stride>last_off)=%0d (stride=%0d span=%0d last_off=%0d)",
                   stride_fits, (fb_stride_bytes > row_last_off_bytes),
                   fb_stride_bytes, row_span_bytes, row_last_off_bytes);

        if (!direct_colour) begin
            // A3 -- indexed row span, byte-domain vs pixel-domain ceiling.
            if (row_span_bytes != ceil_div_px)
                $fatal(1, "[mode_admit] A3: byte-domain span=%0d != pixel-domain ceil(%0d/2^%0d)=%0d",
                       row_span_bytes, row_px, bpp_shift, ceil_div_px);
            // A4 -- indexed element index IS the byte offset.
            if (row_last_off_bytes != row_last_elem_idx)
                $fatal(1, "[mode_admit] A4(indexed): last_off=%0d != elem_idx=%0d",
                       row_last_off_bytes, row_last_elem_idx);
            // A5 -- at 8bpp one byte is one pixel, exactly.
            if ((bpp_shift == 3'd0) && (row_span_bytes != row_px))
                $fatal(1, "[mode_admit] A5(8bpp): span=%0d != row_px=%0d",
                       row_span_bytes, row_px);
        end else begin
            // A4 -- direct colour: the fetcher's own idiom, ((x<<2)+3).
            if (row_last_off_bytes != ((row_last_elem_idx << 2) + {{(ADDR_W-2){1'b0}}, 2'd3}))
                $fatal(1, "[mode_admit] A4(direct): last_off=%0d != (elem_idx<<2)+3=%0d",
                       row_last_off_bytes,
                       (row_last_elem_idx << 2) + {{(ADDR_W-2){1'b0}}, 2'd3});
            // A5 -- direct colour is exactly four bytes per pixel.
            if (row_span_bytes != (row_px << 2))
                $fatal(1, "[mode_admit] A5(direct): span=%0d != row_px*4=%0d",
                       row_span_bytes, row_px << 2);
        end
    end
`endif

endmodule

`default_nettype wire
