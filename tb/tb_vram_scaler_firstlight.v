// tb_vram_scaler_firstlight.v -- CPU/VRAM -> fb_reader -> linebuf first-light tb.
//
// This wrapper keeps the test focused on the synthesizable scanout chain:
// a DAFB register shim supplies live framebuffer placement in the core/VRAM
// clock domain, CPU-shaped AXI writes fill the real URAM-backed vram module,
// and the video_top scanner subpath (scanout_placement_sync + fb_reader +
// linebuf_scanout) reads it back across independent pclk/vram clocks.

`default_nettype none

module tb_vram_scaler_firstlight (
    input  wire         pclk,
    input  wire         vram_clk,
    input  wire         rst,

    input  wire [31:0]  dafb_awaddr,
    input  wire         dafb_awvalid,
    output wire         dafb_awready,
    input  wire [31:0]  dafb_wdata,
    input  wire [3:0]   dafb_wstrb,
    input  wire         dafb_wvalid,
    output wire         dafb_wready,
    output wire [1:0]   dafb_bresp,
    output wire         dafb_bvalid,
    input  wire         dafb_bready,
    input  wire [31:0]  dafb_araddr,
    input  wire         dafb_arvalid,
    output wire         dafb_arready,
    output wire [31:0]  dafb_rdata,
    output wire [1:0]   dafb_rresp,
    output wire         dafb_rvalid,
    input  wire         dafb_rready,

    input  wire [3:0]   vram_awid,
    input  wire [31:0]  vram_awaddr,
    input  wire [7:0]   vram_awlen,
    input  wire [2:0]   vram_awsize,
    input  wire [1:0]   vram_awburst,
    input  wire         vram_awvalid,
    output wire         vram_awready,
    input  wire [127:0] vram_wdata,
    input  wire [15:0]  vram_wstrb,
    input  wire         vram_wlast,
    input  wire         vram_wvalid,
    output wire         vram_wready,
    output wire [3:0]   vram_bid,
    output wire [1:0]   vram_bresp,
    output wire         vram_bvalid,
    input  wire         vram_bready,

    input  wire [3:0]   vram_arid,
    input  wire [31:0]  vram_araddr,
    input  wire [7:0]   vram_arlen,
    input  wire [2:0]   vram_arsize,
    input  wire [1:0]   vram_arburst,
    input  wire         vram_arvalid,
    output wire         vram_arready,
    output wire [3:0]   vram_rid,
    output wire [127:0] vram_rdata,
    output wire [1:0]   vram_rresp,
    output wire         vram_rlast,
    output wire         vram_rvalid,
    input  wire         vram_rready,

    input  wire [11:0]  hcount,
    input  wire [10:0]  vcount,
    input  wire         de_in,
    input  wire         hs_in,
    input  wire         vs_in,

    output wire [31:0]  fb_base_px,
    output wire [31:0]  fb_stride_px,
    output wire [31:0]  fb_bpp_reg,
    output wire         fb_rd_en,
    output wire [19:0]  fb_rd_addr,
    output wire         fb_rd_ready,
    output wire         vram_rd_en,
    output wire [19:0]  vram_rd_addr,
    output wire [23:0]  scanout_rgb,
    output wire         scanout_de,
    output wire         fb_underflow_sticky
);

    localparam VRAM_ADDR_W = 15;

    wire [31:0] shim_fb_base_px;
    wire [31:0] shim_fb_stride_px;
    wire [31:0] shim_fb_bpp_reg;
    wire        shim_clut_we;
    wire [7:0]  shim_clut_waddr;
    wire [23:0] shim_clut_wdata;

    video u_dafb (
        .clk          (vram_clk),
        .rst          (rst),
        .s_axi_awaddr (dafb_awaddr),
        .s_axi_awvalid(dafb_awvalid),
        .s_axi_awready(dafb_awready),
        .s_axi_wdata  (dafb_wdata),
        .s_axi_wstrb  (dafb_wstrb),
        .s_axi_wvalid (dafb_wvalid),
        .s_axi_wready (dafb_wready),
        .s_axi_bresp  (dafb_bresp),
        .s_axi_bvalid (dafb_bvalid),
        .s_axi_bready (dafb_bready),
        .s_axi_araddr (dafb_araddr),
        .s_axi_arvalid(dafb_arvalid),
        .s_axi_arready(dafb_arready),
        .s_axi_rdata  (dafb_rdata),
        .s_axi_rresp  (dafb_rresp),
        .s_axi_rvalid (dafb_rvalid),
        .s_axi_rready (dafb_rready),
        .fb_base_px   (shim_fb_base_px),
        .fb_stride_px (shim_fb_stride_px),
        .fb_bpp_reg   (shim_fb_bpp_reg),
        .bpp_shift    (),
        .fb_bytes_per_px (),
        .depth_supported (),
        .hres         (),
        .vres         (),
        .clut_we      (shim_clut_we),
        .clut_waddr   (shim_clut_waddr),
        .clut_wdata   (shim_clut_wdata),
        .irq          (),
        .pll_pixel_clock(),
        .scsi0_ctrl_out(),
        .scsi0_drq_in (1'b0),
        // This tb exercises scaler/scanout geometry, not vblank cadence.
        .frame_tick   (1'b0),
        // Monitor sense is irrelevant here; pin the historical default
        // (7'h06 = Mac Hi-Res 12-14" 640x480) so behaviour is unchanged
        // from when this was video.v's MONITOR_TYPE parameter default.
        .monitor_sense(7'h06)
    );

    wire frame_start = (hcount == 12'd0) && (vcount == 11'd0);
    wire [19:0] scanout_fb_base_px;
    wire [19:0] scanout_fb_stride_px;

    scanout_placement_sync #(
        .ADDR_W       (20),
        .SRC_W        (16),
        .SRC_H        (12),
        .FB_MAX_PIXELS(256 * 80),
        .FB_BASE_PX   (0),
        .FB_STRIDE_PX (16),
        .DST_W        (40),
        .DST_H        (28)
    ) u_scanout_placement_sync (
        .pclk                 (pclk),
        .rst                  (rst),
        .frame_start          (frame_start),
        .scanout_fb_base_px   (shim_fb_base_px[19:0]),
        .scanout_fb_stride_px (shim_fb_stride_px[19:0]),
        .scanout_bpp_shift    (3'd0),
        .scanout_bytes_per_px (3'd1),
        .scanout_depth_supported (1'b1),
        .scanout_hres         (12'd16),
        .scanout_vres         (12'd12),
        .fb_base_px           (scanout_fb_base_px),
        .fb_stride_px         (scanout_fb_stride_px),
        .bpp_shift            (),
        .bytes_per_px         (),
        .hres                 (),
        .vres                 (),
        .reject_reason             (),
        .placement_rejected_sticky (),
        .reject_base_px            (),
        .reject_stride_px          (),
        .reject_hres_px            (),
        .reject_vres_px            (),
        .reject_reason_latched     (),
        .scale_n              (),
        // These tbs cover the normal video path only; the boot-splash
        // switchover is exercised in tb_scanout_placement_sync /
        // tb_scanout_frames.
        .dafb_live            ()
    );

    assign fb_base_px   = {12'd0, scanout_fb_base_px};
    assign fb_stride_px = {12'd0, scanout_fb_stride_px};
    assign fb_bpp_reg   = shim_fb_bpp_reg;

    // Streaming read port returns a 4-BYTE GROUP per request (see vram.v /
    // linebuf_scanout.v headers); [31:24] is the byte at the requested
    // address, which is all this 8bpp-indexed tb consumes.
    wire [31:0] scaler_fb_rd_data;
    wire       scaler_fb_rd_valid;
    wire [31:0] real_vram_rd_data;
    wire       real_vram_rd_valid;
    wire       fb_reader_underflow_sticky;
    wire       line_underflow_sticky;

    assign fb_underflow_sticky = fb_reader_underflow_sticky | line_underflow_sticky;

    fb_reader #(
        .ADDR_W        (20),
        .DATA_W        (32),
        .REQ_FIFO_DEPTH_LOG2(3),
        .RSP_FIFO_DEPTH_LOG2(4)
    ) u_fb_reader (
        .pclk       (pclk),
        .resetn     (~rst),
        .vram_clk   (vram_clk),
        .vram_rst   (rst),
        .s_rd_en    (fb_rd_en),
        .s_rd_addr  (fb_rd_addr),
        .s_rd_ready (fb_rd_ready),
        .s_rd_data  (scaler_fb_rd_data),
        .s_rd_valid (scaler_fb_rd_valid),
        .v_rd_en    (vram_rd_en),
        .v_rd_addr  (vram_rd_addr),
        .v_rd_data  (real_vram_rd_data),
        .v_rd_valid (real_vram_rd_valid),
        .underflow_sticky(fb_reader_underflow_sticky),
        .req_count  (),
        .rsp_count  (),
        .miss_count ()
    );

    linebuf_scanout #(
        .SRC_W           (16),
        .SRC_H           (12),
        .FB_MAX_PIXELS   (256 * 80),
        .DST_W           (40),
        .DST_H           (28),
        .FETCH_W         (32),
        .ADDR_W          (20),
        .LINE_COUNT_LOG2 (4)
    ) u_scanout (
        .pclk        (pclk),
        .resetn      (~rst),
        .hcount      (hcount),
        .vcount      (vcount),
        .de_in       (de_in),
        .hs_in       (hs_in),
        .vs_in       (vs_in),
        .fb_base_px  (scanout_fb_base_px),
        .fb_stride_px(scanout_fb_stride_px),
        .bpp_shift   (3'd0),
        .bytes_per_px(3'd1),   // 8bpp indexed
        // 2× scale: 16×12 → 32×24 active in 40×28 DST.
        .hres        (12'd16),
        .vres        (12'd12),
        .scale_n     (3'd2),
        // Existing scanout tbs cover the NORMAL video path, so the boot
        // splash is retired here (dafb_live=1).  tb_scanout_frames.v drives
        // it low to cover the splash itself.
        .dafb_live   (1'b1),
        .clut_wclk   (vram_clk),
        .clut_we     (shim_clut_we),
        .clut_waddr  (shim_clut_waddr),
        .clut_wdata  (shim_clut_wdata),
        .fb_rd_en    (fb_rd_en),
        .fb_rd_addr  (fb_rd_addr),
        .fb_rd_ready (fb_rd_ready),
        .fb_rd_data  (scaler_fb_rd_data),
        .fb_rd_valid (scaler_fb_rd_valid),
        .rgb         (scanout_rgb),
        .de_out      (scanout_de),
        .hs_out      (),
        .vs_out      (),
        .line_underflow_sticky(line_underflow_sticky)
    );

    vram #(
        .FB_WIDTH_PX (256),
        .FB_HEIGHT_PX(80),
        // BPP stays 8: it is the pixel packing / read-port address
        // granularity.  RD_DATA_W is the (widened) read-port DATA width, which
        // vram.v requires to be exactly 4*BPP (a 4-lane gather).
        .BPP         (8),
        .RD_DATA_W   (4 * 8),
        .DATA_WIDTH  (128),
        .ID_WIDTH    (4)
    ) u_vram (
        .clk(vram_clk),
        .rst(rst),
        .clear_req(1'b0),

        .s_awid   (vram_awid),
        .s_awaddr (vram_awaddr),
        .s_awlen  (vram_awlen),
        .s_awsize (vram_awsize),
        .s_awburst(vram_awburst),
        .s_awvalid(vram_awvalid),
        .s_awready(vram_awready),
        .s_wdata  (vram_wdata),
        .s_wstrb  (vram_wstrb),
        .s_wlast  (vram_wlast),
        .s_wvalid (vram_wvalid),
        .s_wready (vram_wready),
        .s_bid    (vram_bid),
        .s_bresp  (vram_bresp),
        .s_bvalid (vram_bvalid),
        .s_bready (vram_bready),
        .s_arid   (vram_arid),
        .s_araddr (vram_araddr),
        .s_arlen  (vram_arlen),
        .s_arsize (vram_arsize),
        .s_arburst(vram_arburst),
        .s_arvalid(vram_arvalid),
        .s_arready(vram_arready),
        .s_rid    (vram_rid),
        .s_rdata  (vram_rdata),
        .s_rresp  (vram_rresp),
        .s_rlast  (vram_rlast),
        .s_rvalid (vram_rvalid),
        .s_rready (vram_rready),
        .rd_clk   (vram_clk),
        .rd_rst   (rst),
        .rd_addr  (vram_rd_addr[VRAM_ADDR_W-1:0]),
        .rd_en    (vram_rd_en),
        .rd_data  (real_vram_rd_data),
        .rd_valid (real_vram_rd_valid)
    );

endmodule

`default_nettype wire
