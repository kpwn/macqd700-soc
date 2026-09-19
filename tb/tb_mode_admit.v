// tb_mode_admit.v -- harness for the scan-out admission authority (stage 3).
// ---------------------------------------------------------------------------
// mode_admit is a PURE FUNCTION, so this harness has no clock, no reset and no
// stimulus sequencing: it is a fan-out of the input tuple into several
// elaborated configurations, and the C++ side drives tuples and compares every
// output against a model written from the CONTRACT.
//
// THREE configurations, each chosen for a property it is the only one to have:
//
//   A  the PRODUCTION Q700 scanner bound -- SRC_W 1152, SRC_H 1024, a 2 MiB
//      aperture (rtl/soc/fpga_top_video.vh).  SRC_W is NOT a power of two,
//      which is the case mode_admit.v's own comment warns about: the indexed
//      row span (row_px-1)>>s + 1 and the naive row_px>>s differ exactly
//      there, so this configuration is the one that catches a "simplified"
//      span.
//   B  a POWER-OF-TWO bound with a deliberately TINY aperture, so the
//      frame-out-of-memory gate is reachable at ordinary strides instead of
//      only at absurd ones.  A is 2 MiB and almost nothing overruns it.
//   C  a NARROW bound (SRC_W 640) so the "programmed geometry is wider than
//      the scanner window" clamp is exercised by the real Q700 mode list
//      rather than by invented numbers.
// ---------------------------------------------------------------------------
`default_nettype none

module tb_mode_admit (
    input  wire [11:0] hres_px,
    input  wire [11:0] vres_px,
    input  wire [2:0]  bpp_shift,
    input  wire [2:0]  bytes_per_px,
    input  wire        depth_supported,
    input  wire [23:0] fb_base_bytes,
    input  wire [23:0] fb_stride_bytes,

    output wire        a_renderable,
    output wire        a_addr_admissible,
    output wire        a_stride_fits,
    output wire        a_frame_in_range,
    output wire [3:0]  a_reject_reason,
    output wire [3:0]  a_reason_none_code,
    output wire [23:0] a_row_px,
    output wire [23:0] a_row_span_bytes,
    output wire [23:0] a_row_last_off_bytes,
    output wire [23:0] a_row_last_elem_idx,
    output wire [23:0] a_row_count,
    output wire [23:0] a_last_row,
    output wire [39:0] a_frame_last_addr_bytes,
    output wire        a_direct_colour,
    output wire        a_geometry_zero,

    output wire        b_renderable,
    output wire        b_addr_admissible,
    output wire        b_stride_fits,
    output wire        b_frame_in_range,
    output wire [3:0]  b_reject_reason,
    output wire [23:0] b_row_span_bytes,
    output wire [23:0] b_row_last_off_bytes,
    output wire [39:0] b_frame_last_addr_bytes,

    output wire        c_renderable,
    output wire [3:0]  c_reject_reason,
    output wire [23:0] c_row_px,
    output wire [23:0] c_row_count,
    output wire [23:0] c_row_span_bytes,
    output wire [23:0] c_row_last_elem_idx
);

    // ── A: production ────────────────────────────────────────────────
    localparam A_ADDR_W = 24;
    localparam A_SRC_W  = 1152;
    localparam A_SRC_H  = 1024;
    localparam A_FRAME_W = A_ADDR_W + 10 + 1;   // $clog2(1024) == 10

    wire [A_FRAME_W-1:0] a_last_addr;
    assign a_frame_last_addr_bytes = {{(40-A_FRAME_W){1'b0}}, a_last_addr};

    mode_admit #(
        .ADDR_W       (A_ADDR_W),
        .SRC_W        (A_SRC_W),
        .SRC_H        (A_SRC_H),
        .FB_MAX_BYTES (24'h20_0000)
    ) u_a (
        .hres_px               (hres_px),
        .vres_px               (vres_px),
        .bpp_shift             (bpp_shift),
        .bytes_per_px          (bytes_per_px),
        .depth_supported       (depth_supported),
        .fb_base_bytes         (fb_base_bytes),
        .fb_stride_bytes       (fb_stride_bytes),
        .renderable            (a_renderable),
        .addr_admissible       (a_addr_admissible),
        .stride_fits           (a_stride_fits),
        .frame_in_range        (a_frame_in_range),
        .reject_reason         (a_reject_reason),
        .reason_none_code      (a_reason_none_code),
        .row_px                (a_row_px),
        .row_span_bytes        (a_row_span_bytes),
        .row_last_off_bytes    (a_row_last_off_bytes),
        .row_last_elem_idx     (a_row_last_elem_idx),
        .row_count             (a_row_count),
        .last_row              (a_last_row),
        .frame_last_addr_bytes (a_last_addr),
        .direct_colour         (a_direct_colour),
        .geometry_zero         (a_geometry_zero)
    );

    // ── B: power-of-two bound, TINY aperture ─────────────────────────
    localparam B_ADDR_W = 24;
    localparam B_SRC_W  = 1024;
    localparam B_SRC_H  = 512;
    localparam B_FRAME_W = B_ADDR_W + 9 + 1;    // $clog2(512) == 9

    wire [B_FRAME_W-1:0] b_last_addr;
    assign b_frame_last_addr_bytes = {{(40-B_FRAME_W){1'b0}}, b_last_addr};

    mode_admit #(
        .ADDR_W       (B_ADDR_W),
        .SRC_W        (B_SRC_W),
        .SRC_H        (B_SRC_H),
        .FB_MAX_BYTES (24'h01_0000)             // 64 KiB
    ) u_b (
        .hres_px               (hres_px),
        .vres_px               (vres_px),
        .bpp_shift             (bpp_shift),
        .bytes_per_px          (bytes_per_px),
        .depth_supported       (depth_supported),
        .fb_base_bytes         (fb_base_bytes),
        .fb_stride_bytes       (fb_stride_bytes),
        .renderable            (b_renderable),
        .addr_admissible       (b_addr_admissible),
        .stride_fits           (b_stride_fits),
        .frame_in_range        (b_frame_in_range),
        .reject_reason         (b_reject_reason),
        .reason_none_code      (),
        .row_px                (),
        .row_span_bytes        (b_row_span_bytes),
        .row_last_off_bytes    (b_row_last_off_bytes),
        .row_last_elem_idx     (),
        .row_count             (),
        .last_row              (),
        .frame_last_addr_bytes (b_last_addr),
        .direct_colour         (),
        .geometry_zero         ()
    );

    // ── C: narrow bound, to exercise the clamp ───────────────────────
    mode_admit #(
        .ADDR_W       (24),
        .SRC_W        (640),
        .SRC_H        (480),
        .FB_MAX_BYTES (24'h20_0000)
    ) u_c (
        .hres_px               (hres_px),
        .vres_px               (vres_px),
        .bpp_shift             (bpp_shift),
        .bytes_per_px          (bytes_per_px),
        .depth_supported       (depth_supported),
        .fb_base_bytes         (fb_base_bytes),
        .fb_stride_bytes       (fb_stride_bytes),
        .renderable            (c_renderable),
        .addr_admissible       (),
        .stride_fits           (),
        .frame_in_range        (),
        .reject_reason         (c_reject_reason),
        .reason_none_code      (),
        .row_px                (c_row_px),
        .row_span_bytes        (c_row_span_bytes),
        .row_last_off_bytes    (),
        .row_last_elem_idx     (c_row_last_elem_idx),
        .row_count             (c_row_count),
        .last_row              (),
        .frame_last_addr_bytes (),
        .direct_colour         (),
        .geometry_zero         ()
    );

endmodule

`default_nettype wire
