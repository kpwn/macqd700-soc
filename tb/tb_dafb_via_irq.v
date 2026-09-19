// tb_dafb_via_irq.v -- DAFB vblank IRQ route wrapper
//
// Connects the DAFB register shim to VIA1's external VBL input and then
// through irq_agg so the unit test can prove the Q700 level-1 path.

`default_nettype none

module tb_dafb_via_irq (
    input  wire        clk,
    input  wire        rst,
    input  wire        phi2_tick,
    // Drives video.v's frame_tick input directly (task T8 fix 2 — vblank
    // is now frame_tick-driven, not a free-running 1024-cycle counter).
    input  wire        frame_tick,

    input  wire [31:0] dafb_awaddr,
    input  wire        dafb_awvalid,
    output wire        dafb_awready,
    input  wire [31:0] dafb_wdata,
    input  wire [3:0]  dafb_wstrb,
    input  wire        dafb_wvalid,
    output wire        dafb_wready,
    output wire [1:0]  dafb_bresp,
    output wire        dafb_bvalid,
    input  wire        dafb_bready,

    input  wire [31:0] dafb_araddr,
    input  wire        dafb_arvalid,
    output wire        dafb_arready,
    output wire [31:0] dafb_rdata,
    output wire [1:0]  dafb_rresp,
    output wire        dafb_rvalid,
    input  wire        dafb_rready,

    input  wire [3:0]  via_addr,
    input  wire [7:0]  via_wdata,
    input  wire        via_wr,
    input  wire        via_rd,
    output wire [7:0]  via_rdata,
    output wire        via_ack,

    output wire        dafb_irq,
    output wire        via1_irq,
    output wire [2:0]  ipl
);

    wire [31:0]  fb_base_px;
    wire [31:0]  fb_stride_px;
    wire [31:0]  fb_bpp_reg;
    wire         clut_we;
    wire [7:0]   clut_waddr;
    wire [23:0]  clut_wdata;

    video u_dafb (
        .clk(clk),
        .rst(rst),
        .s_axi_awaddr(dafb_awaddr),
        .s_axi_awvalid(dafb_awvalid),
        .s_axi_awready(dafb_awready),
        .s_axi_wdata(dafb_wdata),
        .s_axi_wstrb(dafb_wstrb),
        .s_axi_wvalid(dafb_wvalid),
        .s_axi_wready(dafb_wready),
        .s_axi_bresp(dafb_bresp),
        .s_axi_bvalid(dafb_bvalid),
        .s_axi_bready(dafb_bready),
        .s_axi_araddr(dafb_araddr),
        .s_axi_arvalid(dafb_arvalid),
        .s_axi_arready(dafb_arready),
        .s_axi_rdata(dafb_rdata),
        .s_axi_rresp(dafb_rresp),
        .s_axi_rvalid(dafb_rvalid),
        .s_axi_rready(dafb_rready),
        .fb_base_px(fb_base_px),
        .fb_stride_px(fb_stride_px),
        .fb_bpp_reg(fb_bpp_reg),
        .bpp_shift(),
        .fb_bytes_per_px (),
        .depth_supported (),
        .hres(),
        .vres(),
        .clut_we(clut_we),
        .clut_waddr(clut_waddr),
        .clut_wdata(clut_wdata),
        .irq(dafb_irq),
        .pll_pixel_clock(),
        // TurboSCSI shim glue — left dangling (output) / tied low (input)
        // for this VIA-IRQ-focused tb.
        .scsi0_ctrl_out(),
        .scsi0_drq_in(1'b0),
        .frame_tick(frame_tick),
        // This tb is about the VIA IRQ route, not monitor sense; pin the
        // historical MONITOR_TYPE default (7'h06).
        .monitor_sense(7'h06)
    );

    wire [7:0] via1_pa_out;
    wire [7:0] via1_pa_mask;
    wire [7:0] via1_pb_out;
    wire [7:0] via1_pb_mask;
    wire       via1_overlay_bit;
    wire       via1_adb_rx_ready;
    wire [7:0] via1_adb_tx_byte;
    wire       via1_adb_tx_valid;
    wire       via1_rtc_enb;
    wire       via1_rtc_clk;
    wire       via1_rtc_data_o;
    wire       via1_rtc_data_oe;

    via1 u_via1 (
        .clk(clk),
        .rst(rst),
        .phi2_tick(phi2_tick),
        .pb_addr(via_addr),
        .pb_wdata(via_wdata),
        .pb_wr(via_wr),
        .pb_rd(via_rd),
        .pb_rdata(via_rdata),
        .pb_ack(via_ack),
        .pa_in(8'h00),
        .pb_in(8'h00),
        // CB1/CB2 are unused by this tb (it drives the DAFB slot IRQ into
        // VIA2, and VIA1 only for CA1), but the pins must be CONNECTED:
        // this target builds -Wall with warnings-as-errors, so an
        // unconnected pin is a BUILD failure -- which is why this target
        // has been dark.  Tying the inputs to 0 matches what they already
        // were under Verilator's --x-initial fast, so nothing changes but
        // the build.
        .cb1_in(1'b0),
        .cb2_in(1'b0),
        .cb2_out(),
        .cb2_oe(),
        .pa_out(via1_pa_out),
        .pa_mask(via1_pa_mask),
        .pb_out(via1_pb_out),
        .pb_mask(via1_pb_mask),
        .overlay_bit(via1_overlay_bit),
        .adb_rx_byte(8'h00),
        .adb_rx_valid(1'b0),
        .adb_rx_ready(via1_adb_rx_ready),
        .adb_tx_byte(via1_adb_tx_byte),
        .adb_tx_valid(via1_adb_tx_valid),
        .rtc_enb(via1_rtc_enb),
        .rtc_clk(via1_rtc_clk),
        .rtc_data_o(via1_rtc_data_o),
        .rtc_data_oe(via1_rtc_data_oe),
        .rtc_data_i(1'b0),
        .rtc_cko(1'b1),
        .vblank_irq_in(dafb_irq),
        .irq(via1_irq)
    );

    irq_agg u_irq_agg (
        .clk(clk),
        .rst(rst),
        .via1_irq(via1_irq),
        .via2_irq(1'b0),
        .scsi_irq(1'b0),
        .scc_irq(1'b0),
        .snd_irq(1'b0),
        .rsvd_irq6(1'b0),
        .nmi_edge(1'b0),
        .ipl_ack(1'b0),
        .ipl(ipl)
    );

    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused = ^{fb_base_px, fb_stride_px, fb_bpp_reg,
                     clut_we, clut_waddr, clut_wdata,
                     via1_pa_out, via1_pa_mask, via1_pb_out, via1_pb_mask,
                     via1_overlay_bit, via1_adb_rx_ready, via1_adb_tx_byte,
                     via1_adb_tx_valid, via1_rtc_enb, via1_rtc_clk,
                     via1_rtc_data_o, via1_rtc_data_oe};
    /* verilator lint_on UNUSEDSIGNAL */

endmodule

`default_nettype wire
