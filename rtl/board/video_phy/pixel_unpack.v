// pixel_unpack.v -- stage 9 of the scan-out pipeline: BIT-DEPTH UNPACKING.
// ---------------------------------------------------------------------------
// PURE FUNCTION.  No clock, no reset, no state.  Given the three line-store
// byte planes for one source-byte address and which sub-byte pixel within it
// is wanted, it produces the two candidate colour sources the downstream
// stages choose between:
//
//     packed bytes + bpp_shift + x_lo  ->  palette index      (indexed depths)
//                                      ->  direct RGB         (24bpp)
//
// It owns bit-depth unpacking and NOTHING else.  It does not know what a
// palette is (stage 10 owns that), what a scale factor is (stage 11), or what
// a border is (stage 12).  Splitting it out is docs/video_path_review.md S4.1
// stage 9; it was previously the `clut_index` function buried in
// scanout_display.v alongside four other jobs.
//
// FIDELITY.  The sub-byte slice order is MSB-FIRST at every indexed depth,
// matching MAME dafb.cpp screen_update(): bit / 2-bit / nibble / byte, all
// indexing the SAME 256-entry palette.  `bpp_shift` is the DAFB-decoded
// pixels-per-byte shift, so 3 => 1bpp (8 px/byte), 2 => 2bpp, 1 => 4bpp,
// 0 => 8bpp.  At 24bpp the caller ignores `index` and takes `direct_rgb`.
//
// PLANE LAYOUT.  The line store is three byte-wide planes, not one 24-bit
// array (a RAMB36E2 width/depth trade -- see scanout_display.v's header).
// The read side concatenates {p0,p1,p2}: plane 0 carries the indexed byte at
// 1/2/4/8bpp and R at 24bpp, planes 1/2 carry G/B and are untouched at the
// indexed depths.  So the palette index comes from [23:16] and direct colour
// is the whole [23:0].  That asymmetry is the reason both outputs exist here
// rather than one being derived from the other downstream.
//
// LATENCY: 0 cycles, combinational.  This stage adds NOTHING to the display
// pipeline's five-stage ladder (see scanout_display.v's LATENCY LADDER note);
// it sits in the combinational cone feeding stage 10's registered read, which
// is exactly where the `clut_index` function sat before the split.
//
// DSP: none, deliberately.  Everything here is a bit-select and a 4:1 mux --
// strictly narrower than the mux that docs/video_path_review.md S4.1 draws as
// the line, and a purely combinational stage cannot use a DSP48E2's A/B/M/P
// pipeline registers, which is where a DSP's value in this path comes from.
// ---------------------------------------------------------------------------
`default_nettype none

module pixel_unpack (
    // DAFB-decoded pixels-per-byte shift: 3=1bpp, 2=2bpp, 1=4bpp, 0=8bpp.
    // At 24bpp (`fetch_direct` upstream) the caller uses `direct_rgb` and
    // this input is don't-care.
    input  wire [2:0]  bpp_shift,
    // Low 3 bits of the SOURCE x coordinate: which sub-byte pixel inside the
    // fetched byte.  Must be the x that produced `plane_bytes`, i.e. already
    // delayed to match the line store's read latency by the caller.
    input  wire [2:0]  x_lo,
    // {plane0, plane1, plane2} straight off the line store read port.
    input  wire [23:0] plane_bytes,

    output wire [7:0]  index,
    output wire [23:0] direct_rgb
);

    // NEGATIVE-CONTROL FAULT INJECTION -- never defined in a production or a
    // normal sim build.  `make tb-scanout-1bpp-negctl` elaborates this file
    // with -DPIXEL_UNPACK_INJECT_1BPP_MIRROR and REQUIRES the pixel-exact
    // 1bpp gate to fail; a green run there is the real failure ("the gate
    // cannot see the thing it gates").  Without it, "every destination pixel
    // matched" would be equally consistent with a harness whose source
    // pattern is constant along a row -- which is exactly the blind spot in
    // tb-scanout-frames that left the 1bpp horizontal path ungated.
    //
    // The injected fault is the one a 1bpp reader is most likely to get
    // wrong: the sub-byte slice taken LSB-first instead of MSB-first, i.e.
    // each group of 8 pixels mirrored in place.
`ifdef PIXEL_UNPACK_INJECT_1BPP_MIRROR
    wire [2:0] x_lo_eff = (bpp_shift == 3'd3) ? ~x_lo : x_lo;
`else
    wire [2:0] x_lo_eff = x_lo;
`endif

    // Per-BPP palette index, MSB-first sub-byte slice.
    function [7:0] clut_index;
        input [2:0] shift;
        input [2:0] xlo;
        input [7:0] byte_in;
        begin
            case (shift)
                3'd3:
                    case (xlo)
                        3'd0: clut_index = {7'd0, byte_in[7]};
                        3'd1: clut_index = {7'd0, byte_in[6]};
                        3'd2: clut_index = {7'd0, byte_in[5]};
                        3'd3: clut_index = {7'd0, byte_in[4]};
                        3'd4: clut_index = {7'd0, byte_in[3]};
                        3'd5: clut_index = {7'd0, byte_in[2]};
                        3'd6: clut_index = {7'd0, byte_in[1]};
                        default: clut_index = {7'd0, byte_in[0]};
                    endcase
                3'd2:
                    case (xlo[1:0])
                        2'd0: clut_index = {6'd0, byte_in[7:6]};
                        2'd1: clut_index = {6'd0, byte_in[5:4]};
                        2'd2: clut_index = {6'd0, byte_in[3:2]};
                        default: clut_index = {6'd0, byte_in[1:0]};
                    endcase
                3'd1:
                    clut_index = xlo[0] ? {4'd0, byte_in[3:0]} : {4'd0, byte_in[7:4]};
                default:
                    clut_index = byte_in[7:0];
            endcase
        end
    endfunction

    assign index      = clut_index(bpp_shift, x_lo_eff, plane_bytes[23:16]);
    assign direct_rgb = plane_bytes[23:0];

endmodule

`default_nettype wire
