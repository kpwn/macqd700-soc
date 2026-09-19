// tb_dafb_extmon.v — DAFB shim wrapper for extended-monitor sense tests.
//
// Instantiates rtl/mac/video.v and promotes its `monitor_sense` input to
// this wrapper's port list so the C++ host can drive an extended-monitor
// code (bit 6 = 1 + bc/ac/ab nibbles per MAME's ext(bc, ac, ab) macro at
// /tmp/mame_src/dafb.cpp:197-200) and exercise the m_monitor_id ×
// monitor_sense convolution path.
//
// The code used to be a `.MONITOR_TYPE(7'h5D)` parameter override here;
// it is now an input, driven by tb_dafb_extmon.cpp, which additionally
// CHANGES it mid-run to prove the response tracks it at runtime (that
// runtime-settability is the whole point of the debug-CSR that drives
// this pin on hardware).  The baseline code is 0x5D, i.e. bit 6 set with
// bc = mon[5:4] = 1, ac = mon[3:2] = 3, ab = mon[1:0] = 1.  NOTE: an
// earlier revision of this comment labelled 0x5D as "ext(2, 3, 1)", which
// does not add up — 0x40|(2<<4)|(3<<2)|1 is 0x6D.  The tb BODY always used
// the bit fields directly, so only the label was wrong; the new
// tb_dafb_extmon.cpp sweep covers both 0x5D and 0x6D explicitly.

`default_nettype none

module tb_dafb_extmon (
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

    output wire [31:0] fb_base_px,
    output wire [31:0] fb_stride_px,
    output wire [31:0] fb_bpp_reg,
    output wire [2:0]  bpp_shift,
    output wire [11:0] hres,
    output wire [11:0] vres,
    output wire        clut_we,
    output wire [7:0]  clut_waddr,
    output wire [23:0] clut_wdata,
    output wire        irq,
    output wire [31:0] pll_pixel_clock,
    output wire [8:0]  scsi0_ctrl_out,
    input  wire        scsi0_drq_in,

    // Apple Display Sense code — driven by the C++ host, not a parameter.
    input  wire [6:0]  monitor_sense
);
    video #(
        .PLL_REF_HZ    (32'd20_000_000),
        .PLL_RESET_HZ  (32'd31_334_400)
    ) u_video (
        .clk           (clk),
        .rst           (rst),
        .s_axi_awaddr  (s_axi_awaddr),
        .s_axi_awvalid (s_axi_awvalid),
        .s_axi_awready (s_axi_awready),
        .s_axi_wdata   (s_axi_wdata),
        .s_axi_wstrb   (s_axi_wstrb),
        .s_axi_wvalid  (s_axi_wvalid),
        .s_axi_wready  (s_axi_wready),
        .s_axi_bresp   (s_axi_bresp),
        .s_axi_bvalid  (s_axi_bvalid),
        .s_axi_bready  (s_axi_bready),
        .s_axi_araddr  (s_axi_araddr),
        .s_axi_arvalid (s_axi_arvalid),
        .s_axi_arready (s_axi_arready),
        .s_axi_rdata   (s_axi_rdata),
        .s_axi_rresp   (s_axi_rresp),
        .s_axi_rvalid  (s_axi_rvalid),
        .s_axi_rready  (s_axi_rready),
        .fb_base_px    (fb_base_px),
        .fb_stride_px  (fb_stride_px),
        .fb_bpp_reg    (fb_bpp_reg),
        .bpp_shift     (bpp_shift),
        .fb_bytes_per_px (),
        .depth_supported (),
        .hres          (hres),
        .vres          (vres),
        .clut_we       (clut_we),
        .clut_waddr    (clut_waddr),
        .clut_wdata    (clut_wdata),
        .irq           (irq),
        .pll_pixel_clock(pll_pixel_clock),
        .scsi0_ctrl_out(scsi0_ctrl_out),
        .scsi0_drq_in  (scsi0_drq_in),
        // This tb exercises monitor-sense convolution only; vblank
        // cadence is out of scope here, so tie the frame_tick input low.
        .frame_tick    (1'b0),
        .monitor_sense (monitor_sense)
    );
endmodule

`default_nettype wire
