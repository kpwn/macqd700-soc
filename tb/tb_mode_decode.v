// tb_mode_decode.v — harness for the DAFB mode-decode contract.
//
// Instantiates the REAL rtl/mac/video.v (which instantiates rtl/mac/
// mode_decode.v) and promotes the whole canonical mode descriptor plus the
// monitor-sense input to this wrapper's port list.
//
// WHY video.v AND NOT mode_decode.v ALONE.  mode_decode is a pure function
// and could be poked directly, but the shipped `hres` bug was never in the
// formula -- it was in WHEN the formula was evaluated against the register
// file.  A tb that drives mode_decode's inputs directly hands it a single
// self-consistent snapshot and therefore cannot see that bug at all.  Driving
// real AXI register writes in the real order the Quadra 700 ROM performs them
// is the only way the epoch-mixing is reachable.

`default_nettype none

module tb_mode_decode (
    input  wire        clk,
    input  wire        rst,

    input  wire [31:0] s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,
    output wire [1:0]  s_axi_bresp,
    output wire        s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [31:0] s_axi_araddr,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,
    output wire [31:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,
    output wire        s_axi_rvalid,
    input  wire        s_axi_rready,

    // ── The canonical mode descriptor, unit-explicit ─────────────────
    output wire [11:0] src_w_px,
    output wire [11:0] src_h_px,
    output wire [2:0]  bpp_shift,
    output wire [31:0] bytes_per_row,
    output wire [31:0] fb_base_bytes,
    output wire [31:0] lut_depth,
    output wire [2:0]  bytes_per_px,
    output wire        depth_supported,

    input  wire [6:0]  monitor_sense
);
    video #(
        .PLL_REF_HZ   (32'd20_000_000),
        .PLL_RESET_HZ (32'd31_334_400)
    ) u_video (
        .clk             (clk),
        .rst             (rst),
        .s_axi_awaddr    (s_axi_awaddr),
        .s_axi_awvalid   (s_axi_awvalid),
        .s_axi_awready   (s_axi_awready),
        .s_axi_wdata     (s_axi_wdata),
        .s_axi_wstrb     (s_axi_wstrb),
        .s_axi_wvalid    (s_axi_wvalid),
        .s_axi_wready    (s_axi_wready),
        .s_axi_bresp     (s_axi_bresp),
        .s_axi_bvalid    (s_axi_bvalid),
        .s_axi_bready    (s_axi_bready),
        .s_axi_araddr    (s_axi_araddr),
        .s_axi_arvalid   (s_axi_arvalid),
        .s_axi_arready   (s_axi_arready),
        .s_axi_rdata     (s_axi_rdata),
        .s_axi_rresp     (s_axi_rresp),
        .s_axi_rvalid    (s_axi_rvalid),
        .s_axi_rready    (s_axi_rready),
        // video.v still names these two ports fb_base_px / fb_stride_px.
        // Both carry BYTES.  The wrapper renames them at the boundary so the
        // test speaks the descriptor's units; renaming the RTL ports is a
        // separate, wider change (they are wired into fpga_top).
        .fb_base_px      (fb_base_bytes),
        .fb_stride_px    (bytes_per_row),
        .fb_bpp_reg      (lut_depth),
        .bpp_shift       (bpp_shift),
        .fb_bytes_per_px (bytes_per_px),
        .depth_supported (depth_supported),
        .hres            (src_w_px),
        .vres            (src_h_px),
        .clut_we         (),
        .clut_waddr      (),
        .clut_wdata      (),
        .irq             (),
        .pll_pixel_clock (),
        .scsi0_ctrl_out  (),
        .scsi0_drq_in    (1'b0),
        // Geometry decode only; the VBL cadence is out of scope here.
        .frame_tick      (1'b0),
        .monitor_sense   (monitor_sense)
    );
endmodule

`default_nettype wire
