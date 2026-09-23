// mame_axi_periph_top.v - Verilator top for MAME-driven AXI peripheral tests.
//
// This is intentionally not a production SoC top.  It instantiates the real
// xbar decode boundary, peripheral_bus fan-out, plus the currently usable Q700
// pb-style peripherals, and stubs repo-service AXI windows with deterministic
// AXI-Lite responders.

`default_nettype none

module mame_axi_periph_top (
    input  wire         clk,
    input  wire         rst,

    input  wire [5:0]   s_awid,
    input  wire [31:0]  s_awaddr,
    input  wire [7:0]   s_awlen,
    input  wire [2:0]   s_awsize,
    input  wire [1:0]   s_awburst,
    input  wire         s_awvalid,
    output wire         s_awready,

    input  wire [127:0] s_wdata,
    input  wire [15:0]  s_wstrb,
    input  wire         s_wlast,
    input  wire         s_wvalid,
    output wire         s_wready,

    output wire [5:0]   s_bid,
    output wire [1:0]   s_bresp,
    output wire         s_bvalid,
    input  wire         s_bready,

    input  wire [5:0]   s_arid,
    input  wire [31:0]  s_araddr,
    input  wire [7:0]   s_arlen,
    input  wire [2:0]   s_arsize,
    input  wire [1:0]   s_arburst,
    input  wire         s_arvalid,
    output wire         s_arready,

    output wire [5:0]   s_rid,
    output wire [127:0] s_rdata,
    output wire [1:0]   s_rresp,
    output wire         s_rlast,
    output wire         s_rvalid,
    input  wire         s_rready,

    output wire         via1_irq,
    output wire         via2_irq,
    output wire         scc_irq,
    output wire         scsi_irq,
    output wire         scsi_drq,
    output wire         asc_irq,
    output wire         iwm_irq,
    input  wire         scc_rx_a_valid,
    input  wire [7:0]   scc_rx_a_data,
    input  wire         scc_rx_b_valid,
    input  wire [7:0]   scc_rx_b_data,
    input  wire         scc_cts_a_n,
    input  wire         scc_dcd_a_n,
    input  wire         scc_sync_a_n,
    input  wire         scc_cts_b_n,
    input  wire         scc_dcd_b_n,
    input  wire         scc_sync_b_n,
    output wire         scc_tx_a_valid,
    output wire [7:0]   scc_tx_a_data,
    output wire         scc_tx_b_valid,
    output wire [7:0]   scc_tx_b_data,
    output wire         scc_rts_a_n,
    output wire         scc_dtr_a_n,
    output wire         scc_rts_b_n,
    output wire         scc_dtr_b_n
);

    // MAME reports 68040 cycles to the bridge; one bridge clock is one
    // MAME CPU cycle.  The Q700 VIAs run at C7M/10 = 783.36 kHz while the
    // CPU is 25 MHz, so the period is 31.914... CPU cycles.  Use an NCO
    // rather than /32 so long VIA timers land on the same side of ROM
    // polling/clear races as MAME's via_sync() model.
    localparam [31:0] CPU_HZ = 32'd25_000_000;
    localparam [31:0] VIA_HZ = 32'd783_360;
    reg [31:0] phi2_accum;
    reg        phi2_tick_r;
    wire [32:0] phi2_accum_sum = {1'b0, phi2_accum} + {1'b0, VIA_HZ};
    always @(posedge clk) begin
        if (rst) begin
            phi2_accum  <= 32'd0;
            phi2_tick_r <= 1'b0;
        end else if (phi2_accum_sum >= {1'b0, CPU_HZ}) begin
            phi2_accum  <= phi2_accum_sum[31:0] - CPU_HZ;
            phi2_tick_r <= 1'b1;
        end else begin
            phi2_accum  <= phi2_accum_sum[31:0];
            phi2_tick_r <= 1'b0;
        end
    end
    wire phi2_tick = phi2_tick_r;

    wire [19:0] dbg_awaddr;
    wire        dbg_awvalid;
    wire        dbg_awready;
    wire [31:0] dbg_wdata;
    wire [3:0]  dbg_wstrb;
    wire        dbg_wvalid;
    wire        dbg_wready;
    wire [1:0]  dbg_bresp;
    wire        dbg_bvalid;
    wire        dbg_bready;
    wire [19:0] dbg_araddr;
    wire        dbg_arvalid;
    wire        dbg_arready;
    wire [31:0] dbg_rdata;
    wire [1:0]  dbg_rresp;
    wire        dbg_rvalid;
    wire        dbg_rready;

    wire [19:0] prov_awaddr;
    wire        prov_awvalid;
    wire        prov_awready;
    wire [31:0] prov_wdata;
    wire [3:0]  prov_wstrb;
    wire        prov_wvalid;
    wire        prov_wready;
    wire [1:0]  prov_bresp;
    wire        prov_bvalid;
    wire        prov_bready;
    wire [19:0] prov_araddr;
    wire        prov_arvalid;
    wire        prov_arready;
    wire [31:0] prov_rdata;
    wire [1:0]  prov_rresp;
    wire        prov_rvalid;
    wire        prov_rready;

    wire [31:0] dafb_awaddr;
    wire        dafb_awvalid;
    wire        dafb_awready;
    wire [31:0] dafb_wdata;
    wire [3:0]  dafb_wstrb;
    wire        dafb_wvalid;
    wire        dafb_wready;
    wire [1:0]  dafb_bresp;
    wire        dafb_bvalid;
    wire        dafb_bready;
    wire [31:0] dafb_araddr;
    wire        dafb_arvalid;
    wire        dafb_arready;
    wire [31:0] dafb_rdata;
    wire [1:0]  dafb_rresp;
    wire        dafb_rvalid;
    wire        dafb_rready;
    wire [31:0] dafb_fb_base_px;
    wire [31:0] dafb_fb_stride_px;
    wire [31:0] dafb_fb_bpp_reg;
    wire        dafb_clut_we;
    wire [7:0]  dafb_clut_waddr;
    wire [23:0] dafb_clut_wdata;
    wire        dafb_irq;

    wire [3:0]  via1_addr;
    wire [7:0]  via1_wdata;
    wire        via1_wr;
    wire        via1_rd;
    wire [7:0]  via1_rdata;
    wire        via1_ack;
    wire [3:0]  via2_addr;
    wire [7:0]  via2_wdata;
    wire        via2_wr;
    wire        via2_rd;
    wire [7:0]  via2_rdata;
    wire        via2_ack;
    wire [2:0]  enet_addr;
    wire [7:0]  enet_wdata;
    wire        enet_wr;
    wire        enet_rd;
    wire [7:0]  enet_rdata;
    wire        enet_ack;
    wire [5:0]  sonic_addr;
    wire [15:0] sonic_wdata;
    wire [1:0]  sonic_wstrb;
    wire        sonic_wr;
    wire        sonic_rd;
    wire [15:0] sonic_rdata;
    wire        sonic_ack;
    wire [7:0]  orwell_addr;
    wire [7:0]  orwell_wdata;
    wire        orwell_wr;
    wire        orwell_rd;
    wire [7:0]  orwell_rdata;
    wire        orwell_ack;
    wire [3:0]  scc_addr;
    wire [7:0]  scc_wdata;
    wire        scc_wr;
    wire        scc_rd;
    wire [7:0]  scc_rdata;
    wire        scc_ack;
    wire [8:0]  scsi_addr;
    wire [7:0]  scsi_wdata;
    wire        scsi_wr;
    wire        scsi_rd;
    wire [7:0]  scsi_rdata;
    wire        scsi_ack;
    wire        scsi_dma16_lo_beat;
    wire        scsi_dma_rd_ready;
    wire        scsi_dma_wr_ready;
    wire [11:0] asc_addr;
    wire [7:0]  asc_wdata;
    wire        asc_wr;
    wire        asc_rd;
    wire [7:0]  asc_rdata;
    wire        asc_ack;
    wire [3:0]  iwm_addr;
    wire [7:0]  iwm_wdata;
    wire        iwm_wr;
    wire        iwm_rd;
    wire [7:0]  iwm_rdata;
    wire        iwm_ack;

    wire [3:0]  m0_bid;
    wire [3:0]  m0_rid;
    assign s_bid = {2'b00, m0_bid};
    assign s_rid = {2'b00, m0_rid};

    wire [5:0]   x_s0_awid, x_s0_bid, x_s0_arid, x_s0_rid;
    wire [31:0]  x_s0_awaddr, x_s0_araddr;
    wire [7:0]   x_s0_awlen, x_s0_arlen;
    wire [2:0]   x_s0_awsize, x_s0_arsize;
    wire [1:0]   x_s0_awburst, x_s0_arburst, x_s0_bresp, x_s0_rresp;
    wire         x_s0_awvalid, x_s0_awready, x_s0_wvalid, x_s0_wready;
    wire         x_s0_bvalid, x_s0_bready, x_s0_arvalid, x_s0_arready;
    wire         x_s0_rvalid, x_s0_rready, x_s0_wlast, x_s0_rlast;
    wire [127:0] x_s0_wdata, x_s0_rdata;
    wire [15:0]  x_s0_wstrb;

    wire [5:0]   x_s1_awid, x_s1_bid, x_s1_arid, x_s1_rid;
    wire [31:0]  x_s1_awaddr, x_s1_araddr;
    wire [7:0]   x_s1_awlen, x_s1_arlen;
    wire [2:0]   x_s1_awsize, x_s1_arsize;
    wire [1:0]   x_s1_awburst, x_s1_arburst, x_s1_bresp, x_s1_rresp;
    wire         x_s1_awvalid, x_s1_awready, x_s1_wvalid, x_s1_wready;
    wire         x_s1_bvalid, x_s1_bready, x_s1_arvalid, x_s1_arready;
    wire         x_s1_rvalid, x_s1_rready, x_s1_wlast, x_s1_rlast;
    wire [127:0] x_s1_wdata, x_s1_rdata;
    wire [15:0]  x_s1_wstrb;

    wire [5:0]   x_s2_awid, x_s2_bid, x_s2_arid, x_s2_rid;
    wire [31:0]  x_s2_awaddr, x_s2_araddr;
    wire [7:0]   x_s2_awlen, x_s2_arlen;
    wire [2:0]   x_s2_awsize, x_s2_arsize;
    wire [1:0]   x_s2_awburst, x_s2_arburst, x_s2_bresp, x_s2_rresp;
    wire         x_s2_awvalid, x_s2_awready, x_s2_wvalid, x_s2_wready;
    wire         x_s2_bvalid, x_s2_bready, x_s2_arvalid, x_s2_arready;
    wire         x_s2_rvalid, x_s2_rready, x_s2_wlast, x_s2_rlast;
    wire [127:0] x_s2_wdata, x_s2_rdata;
    wire [15:0]  x_s2_wstrb;

    wire [5:0]   x_s3_awid, x_s3_bid, x_s3_arid, x_s3_rid;
    wire [31:0]  x_s3_awaddr, x_s3_araddr;
    wire [7:0]   x_s3_awlen, x_s3_arlen;
    wire [2:0]   x_s3_awsize, x_s3_arsize;
    wire [1:0]   x_s3_awburst, x_s3_arburst, x_s3_bresp, x_s3_rresp;
    wire         x_s3_awvalid, x_s3_awready, x_s3_wvalid, x_s3_wready;
    wire         x_s3_bvalid, x_s3_bready, x_s3_arvalid, x_s3_arready;
    wire         x_s3_rvalid, x_s3_rready, x_s3_wlast, x_s3_rlast;
    wire [127:0] x_s3_wdata, x_s3_rdata;
    wire [15:0]  x_s3_wstrb;

    wire [5:0]   x_s4_awid, x_s4_bid, x_s4_arid, x_s4_rid;
    wire [31:0]  x_s4_awaddr, x_s4_araddr;
    wire [7:0]   x_s4_awlen, x_s4_arlen;
    wire [2:0]   x_s4_awsize, x_s4_arsize;
    wire [1:0]   x_s4_awburst, x_s4_arburst, x_s4_bresp, x_s4_rresp;
    wire         x_s4_awvalid, x_s4_awready, x_s4_wvalid, x_s4_wready;
    wire         x_s4_bvalid, x_s4_bready, x_s4_arvalid, x_s4_arready;
    wire         x_s4_rvalid, x_s4_rready, x_s4_wlast, x_s4_rlast;
    wire [127:0] x_s4_wdata, x_s4_rdata;
    wire [15:0]  x_s4_wstrb;

    wire [19:0]  dafb_axil_awaddr;
    wire [19:0]  dafb_axil_araddr;

    wire [31:0] pb_dafb_awaddr;
    wire        pb_dafb_awvalid;
    wire        pb_dafb_awready;
    wire [31:0] pb_dafb_wdata;
    wire [3:0]  pb_dafb_wstrb;
    wire        pb_dafb_wvalid;
    wire        pb_dafb_wready;
    wire [1:0]  pb_dafb_bresp;
    wire        pb_dafb_bvalid;
    wire        pb_dafb_bready;
    wire [31:0] pb_dafb_araddr;
    wire        pb_dafb_arvalid;
    wire        pb_dafb_arready;
    wire [31:0] pb_dafb_rdata;
    wire [1:0]  pb_dafb_rresp;
    wire        pb_dafb_rvalid;
    wire        pb_dafb_rready;

    mame_axi_xbar_sink #(.ID_WIDTH(6), .DATA_WIDTH(128), .RESP(2'b11)) u_s0_sink (
        .clk(clk), .rst(rst),
        .awid(x_s0_awid), .awaddr(x_s0_awaddr), .awlen(x_s0_awlen),
        .awsize(x_s0_awsize), .awburst(x_s0_awburst),
        .awvalid(x_s0_awvalid), .awready(x_s0_awready),
        .wdata(x_s0_wdata), .wstrb(x_s0_wstrb), .wlast(x_s0_wlast),
        .wvalid(x_s0_wvalid), .wready(x_s0_wready),
        .bid(x_s0_bid), .bresp(x_s0_bresp), .bvalid(x_s0_bvalid), .bready(x_s0_bready),
        .arid(x_s0_arid), .araddr(x_s0_araddr), .arlen(x_s0_arlen),
        .arsize(x_s0_arsize), .arburst(x_s0_arburst),
        .arvalid(x_s0_arvalid), .arready(x_s0_arready),
        .rid(x_s0_rid), .rdata(x_s0_rdata), .rresp(x_s0_rresp),
        .rlast(x_s0_rlast), .rvalid(x_s0_rvalid), .rready(x_s0_rready)
    );

    mame_axi_xbar_sink #(.ID_WIDTH(6), .DATA_WIDTH(128), .RESP(2'b11)) u_s2_sink (
        .clk(clk), .rst(rst),
        .awid(x_s2_awid), .awaddr(x_s2_awaddr), .awlen(x_s2_awlen),
        .awsize(x_s2_awsize), .awburst(x_s2_awburst),
        .awvalid(x_s2_awvalid), .awready(x_s2_awready),
        .wdata(x_s2_wdata), .wstrb(x_s2_wstrb), .wlast(x_s2_wlast),
        .wvalid(x_s2_wvalid), .wready(x_s2_wready),
        .bid(x_s2_bid), .bresp(x_s2_bresp), .bvalid(x_s2_bvalid), .bready(x_s2_bready),
        .arid(x_s2_arid), .araddr(x_s2_araddr), .arlen(x_s2_arlen),
        .arsize(x_s2_arsize), .arburst(x_s2_arburst),
        .arvalid(x_s2_arvalid), .arready(x_s2_arready),
        .rid(x_s2_rid), .rdata(x_s2_rdata), .rresp(x_s2_rresp),
        .rlast(x_s2_rlast), .rvalid(x_s2_rvalid), .rready(x_s2_rready)
    );

    mame_axi_xbar_sink #(.ID_WIDTH(6), .DATA_WIDTH(128), .RESP(2'b11)) u_s3_sink (
        .clk(clk), .rst(rst),
        .awid(x_s3_awid), .awaddr(x_s3_awaddr), .awlen(x_s3_awlen),
        .awsize(x_s3_awsize), .awburst(x_s3_awburst),
        .awvalid(x_s3_awvalid), .awready(x_s3_awready),
        .wdata(x_s3_wdata), .wstrb(x_s3_wstrb), .wlast(x_s3_wlast),
        .wvalid(x_s3_wvalid), .wready(x_s3_wready),
        .bid(x_s3_bid), .bresp(x_s3_bresp), .bvalid(x_s3_bvalid), .bready(x_s3_bready),
        .arid(x_s3_arid), .araddr(x_s3_araddr), .arlen(x_s3_arlen),
        .arsize(x_s3_arsize), .arburst(x_s3_arburst),
        .arvalid(x_s3_arvalid), .arready(x_s3_arready),
        .rid(x_s3_rid), .rdata(x_s3_rdata), .rresp(x_s3_rresp),
        .rlast(x_s3_rlast), .rvalid(x_s3_rvalid), .rready(x_s3_rready)
    );

    axi_xbar #(
        .DATA_WIDTH(128),
        .ID_WIDTH  (4),
        .N_MASTERS (5),
        .N_SLAVES  (5)
    ) u_xbar (
        .clk(clk), .rst(rst),
        .cpu_overlay_active(1'b0),
        .cpu_overlay_reset(rst),
        .dbg_ram_window_lg2(6'd26),
        .m0_awid(s_awid[3:0]), .m0_awaddr(s_awaddr), .m0_awlen(s_awlen),
        .m0_awsize(s_awsize), .m0_awburst(s_awburst),
        .m0_awvalid(s_awvalid), .m0_awready(s_awready),
        .m0_wdata(s_wdata), .m0_wstrb(s_wstrb), .m0_wlast(s_wlast),
        .m0_wvalid(s_wvalid), .m0_wready(s_wready),
        .m0_bid(m0_bid), .m0_bresp(s_bresp), .m0_bvalid(s_bvalid), .m0_bready(s_bready),
        .m0_arid(s_arid[3:0]), .m0_araddr(s_araddr), .m0_arlen(s_arlen),
        .m0_arsize(s_arsize), .m0_arburst(s_arburst),
        .m0_arvalid(s_arvalid), .m0_arready(s_arready),
        .m0_rid(m0_rid), .m0_rdata(s_rdata), .m0_rresp(s_rresp),
        .m0_rlast(s_rlast), .m0_rvalid(s_rvalid), .m0_rready(s_rready),
        .m1_awid(4'd0), .m1_awaddr(32'd0), .m1_awlen(8'd0), .m1_awsize(3'd0),
        .m1_awburst(2'd0), .m1_awvalid(1'b0), .m1_awready(),
        .m1_wdata(128'd0), .m1_wstrb(16'd0), .m1_wlast(1'b0), .m1_wvalid(1'b0),
        .m1_wready(), .m1_bid(), .m1_bresp(), .m1_bvalid(), .m1_bready(1'b1),
        .m1_arid(4'd0), .m1_araddr(32'd0), .m1_arlen(8'd0), .m1_arsize(3'd0),
        .m1_arburst(2'd0), .m1_arvalid(1'b0), .m1_arready(),
        .m1_rid(), .m1_rdata(), .m1_rresp(), .m1_rlast(), .m1_rvalid(), .m1_rready(1'b1),
        .m2_awid(4'd0), .m2_awaddr(32'd0), .m2_awlen(8'd0), .m2_awsize(3'd0),
        .m2_awburst(2'd0), .m2_awvalid(1'b0), .m2_awready(),
        .m2_wdata(128'd0), .m2_wstrb(16'd0), .m2_wlast(1'b0), .m2_wvalid(1'b0),
        .m2_wready(), .m2_bid(), .m2_bresp(), .m2_bvalid(), .m2_bready(1'b1),
        .m3_arid(4'd0), .m3_araddr(32'd0), .m3_arlen(8'd0), .m3_arsize(3'd0),
        .m3_arburst(2'd0), .m3_arvalid(1'b0), .m3_arready(),
        .m3_rid(), .m3_rdata(), .m3_rresp(), .m3_rlast(), .m3_rvalid(), .m3_rready(1'b1),
        .m4_awid(4'd0), .m4_awaddr(32'd0), .m4_awlen(8'd0), .m4_awsize(3'd0),
        .m4_awburst(2'd0), .m4_awvalid(1'b0), .m4_awready(),
        .m4_wdata(128'd0), .m4_wstrb(16'd0), .m4_wlast(1'b0), .m4_wvalid(1'b0),
        .m4_wready(), .m4_bid(), .m4_bresp(), .m4_bvalid(), .m4_bready(1'b1),
        .m4_arid(4'd0), .m4_araddr(32'd0), .m4_arlen(8'd0), .m4_arsize(3'd0),
        .m4_arburst(2'd0), .m4_arvalid(1'b0), .m4_arready(),
        .m4_rid(), .m4_rdata(), .m4_rresp(), .m4_rlast(), .m4_rvalid(), .m4_rready(1'b1),
        .s0_awid(x_s0_awid), .s0_awaddr(x_s0_awaddr), .s0_awlen(x_s0_awlen),
        .s0_awsize(x_s0_awsize), .s0_awburst(x_s0_awburst), .s0_awvalid(x_s0_awvalid),
        .s0_awready(x_s0_awready), .s0_wdata(x_s0_wdata), .s0_wstrb(x_s0_wstrb),
        .s0_wlast(x_s0_wlast), .s0_wvalid(x_s0_wvalid), .s0_wready(x_s0_wready),
        .s0_bid(x_s0_bid), .s0_bresp(x_s0_bresp), .s0_bvalid(x_s0_bvalid),
        .s0_bready(x_s0_bready), .s0_arid(x_s0_arid), .s0_araddr(x_s0_araddr),
        .s0_arlen(x_s0_arlen), .s0_arsize(x_s0_arsize), .s0_arburst(x_s0_arburst),
        .s0_arvalid(x_s0_arvalid), .s0_arready(x_s0_arready), .s0_rid(x_s0_rid),
        .s0_rdata(x_s0_rdata), .s0_rresp(x_s0_rresp), .s0_rlast(x_s0_rlast),
        .s0_rvalid(x_s0_rvalid), .s0_rready(x_s0_rready),
        .s1_awid(x_s1_awid), .s1_awaddr(x_s1_awaddr), .s1_awlen(x_s1_awlen),
        .s1_awsize(x_s1_awsize), .s1_awburst(x_s1_awburst), .s1_awvalid(x_s1_awvalid),
        .s1_awready(x_s1_awready), .s1_wdata(x_s1_wdata), .s1_wstrb(x_s1_wstrb),
        .s1_wlast(x_s1_wlast), .s1_wvalid(x_s1_wvalid), .s1_wready(x_s1_wready),
        .s1_bid(x_s1_bid), .s1_bresp(x_s1_bresp), .s1_bvalid(x_s1_bvalid),
        .s1_bready(x_s1_bready), .s1_arid(x_s1_arid), .s1_araddr(x_s1_araddr),
        .s1_arlen(x_s1_arlen), .s1_arsize(x_s1_arsize), .s1_arburst(x_s1_arburst),
        .s1_arvalid(x_s1_arvalid), .s1_arready(x_s1_arready), .s1_rid(x_s1_rid),
        .s1_rdata(x_s1_rdata), .s1_rresp(x_s1_rresp), .s1_rlast(x_s1_rlast),
        .s1_rvalid(x_s1_rvalid), .s1_rready(x_s1_rready),
        .s2_awid(x_s2_awid), .s2_awaddr(x_s2_awaddr), .s2_awlen(x_s2_awlen),
        .s2_awsize(x_s2_awsize), .s2_awburst(x_s2_awburst), .s2_awvalid(x_s2_awvalid),
        .s2_awready(x_s2_awready), .s2_wdata(x_s2_wdata), .s2_wstrb(x_s2_wstrb),
        .s2_wlast(x_s2_wlast), .s2_wvalid(x_s2_wvalid), .s2_wready(x_s2_wready),
        .s2_bid(x_s2_bid), .s2_bresp(x_s2_bresp), .s2_bvalid(x_s2_bvalid),
        .s2_bready(x_s2_bready), .s2_arid(x_s2_arid), .s2_araddr(x_s2_araddr),
        .s2_arlen(x_s2_arlen), .s2_arsize(x_s2_arsize), .s2_arburst(x_s2_arburst),
        .s2_arvalid(x_s2_arvalid), .s2_arready(x_s2_arready), .s2_rid(x_s2_rid),
        .s2_rdata(x_s2_rdata), .s2_rresp(x_s2_rresp), .s2_rlast(x_s2_rlast),
        .s2_rvalid(x_s2_rvalid), .s2_rready(x_s2_rready),
        .s3_awid(x_s3_awid), .s3_awaddr(x_s3_awaddr), .s3_awlen(x_s3_awlen),
        .s3_awsize(x_s3_awsize), .s3_awburst(x_s3_awburst), .s3_awvalid(x_s3_awvalid),
        .s3_awready(x_s3_awready), .s3_wdata(x_s3_wdata), .s3_wstrb(x_s3_wstrb),
        .s3_wlast(x_s3_wlast), .s3_wvalid(x_s3_wvalid), .s3_wready(x_s3_wready),
        .s3_bid(x_s3_bid), .s3_bresp(x_s3_bresp), .s3_bvalid(x_s3_bvalid),
        .s3_bready(x_s3_bready), .s3_arid(x_s3_arid), .s3_araddr(x_s3_araddr),
        .s3_arlen(x_s3_arlen), .s3_arsize(x_s3_arsize), .s3_arburst(x_s3_arburst),
        .s3_arvalid(x_s3_arvalid), .s3_arready(x_s3_arready), .s3_rid(x_s3_rid),
        .s3_rdata(x_s3_rdata), .s3_rresp(x_s3_rresp), .s3_rlast(x_s3_rlast),
        .s3_rvalid(x_s3_rvalid), .s3_rready(x_s3_rready),
        .s4_awid(x_s4_awid), .s4_awaddr(x_s4_awaddr), .s4_awlen(x_s4_awlen),
        .s4_awsize(x_s4_awsize), .s4_awburst(x_s4_awburst), .s4_awvalid(x_s4_awvalid),
        .s4_awready(x_s4_awready), .s4_wdata(x_s4_wdata), .s4_wstrb(x_s4_wstrb),
        .s4_wlast(x_s4_wlast), .s4_wvalid(x_s4_wvalid), .s4_wready(x_s4_wready),
        .s4_bid(x_s4_bid), .s4_bresp(x_s4_bresp), .s4_bvalid(x_s4_bvalid),
        .s4_bready(x_s4_bready), .s4_arid(x_s4_arid), .s4_araddr(x_s4_araddr),
        .s4_arlen(x_s4_arlen), .s4_arsize(x_s4_arsize), .s4_arburst(x_s4_arburst),
        .s4_arvalid(x_s4_arvalid), .s4_arready(x_s4_arready), .s4_rid(x_s4_rid),
        .s4_rdata(x_s4_rdata), .s4_rresp(x_s4_rresp), .s4_rlast(x_s4_rlast),
        .s4_rvalid(x_s4_rvalid), .s4_rready(x_s4_rready)
    );

    axi_wide_to_axilite #(.ID_WIDTH(6), .DATA_WIDTH(128)) u_dafb_s4_bridge (
        .clk(clk), .rst(rst),
        .s_awid(x_s4_awid), .s_awaddr(x_s4_awaddr), .s_awlen(x_s4_awlen),
        .s_awsize(x_s4_awsize), .s_awburst(x_s4_awburst),
        .s_awvalid(x_s4_awvalid), .s_awready(x_s4_awready),
        .s_wdata(x_s4_wdata), .s_wstrb(x_s4_wstrb), .s_wlast(x_s4_wlast),
        .s_wvalid(x_s4_wvalid), .s_wready(x_s4_wready),
        .s_bid(x_s4_bid), .s_bresp(x_s4_bresp), .s_bvalid(x_s4_bvalid),
        .s_bready(x_s4_bready),
        .s_arid(x_s4_arid), .s_araddr(x_s4_araddr), .s_arlen(x_s4_arlen),
        .s_arsize(x_s4_arsize), .s_arburst(x_s4_arburst),
        .s_arvalid(x_s4_arvalid), .s_arready(x_s4_arready),
        .s_rid(x_s4_rid), .s_rdata(x_s4_rdata), .s_rresp(x_s4_rresp),
        .s_rlast(x_s4_rlast), .s_rvalid(x_s4_rvalid), .s_rready(x_s4_rready),
        .l_awaddr(dafb_axil_awaddr), .l_awvalid(dafb_awvalid), .l_awready(dafb_awready),
        .l_wdata(dafb_wdata), .l_wstrb(dafb_wstrb), .l_wvalid(dafb_wvalid),
        .l_wready(dafb_wready), .l_bresp(dafb_bresp), .l_bvalid(dafb_bvalid),
        .l_bready(dafb_bready),
        .l_araddr(dafb_axil_araddr), .l_arvalid(dafb_arvalid), .l_arready(dafb_arready),
        .l_rdata(dafb_rdata), .l_rresp(dafb_rresp), .l_rvalid(dafb_rvalid),
        .l_rready(dafb_rready)
    );

    assign dafb_awaddr = {12'h000, dafb_axil_awaddr};
    assign dafb_araddr = {12'h000, dafb_axil_araddr};

    peripheral_bus u_bus (
        .clk(clk), .rst(rst),
        // Every peripheral in this harness shares `rst` with the bus, so
        // there is no window in which a pb_* face is in reset while the
        // bus is live -- the barrier has nothing to do here.
        .periph_rst(1'b0),
        .s_awid(x_s1_awid), .s_awaddr(x_s1_awaddr), .s_awlen(x_s1_awlen),
        .s_awsize(x_s1_awsize), .s_awburst(x_s1_awburst),
        .s_awvalid(x_s1_awvalid), .s_awready(x_s1_awready),
        .s_wdata(x_s1_wdata), .s_wstrb(x_s1_wstrb), .s_wlast(x_s1_wlast),
        .s_wvalid(x_s1_wvalid), .s_wready(x_s1_wready),
        .s_bid(x_s1_bid), .s_bresp(x_s1_bresp), .s_bvalid(x_s1_bvalid),
        .s_bready(x_s1_bready),
        .s_arid(x_s1_arid), .s_araddr(x_s1_araddr), .s_arlen(x_s1_arlen),
        .s_arsize(x_s1_arsize), .s_arburst(x_s1_arburst),
        .s_arvalid(x_s1_arvalid), .s_arready(x_s1_arready),
        .s_rid(x_s1_rid), .s_rdata(x_s1_rdata), .s_rresp(x_s1_rresp),
        .s_rlast(x_s1_rlast), .s_rvalid(x_s1_rvalid), .s_rready(x_s1_rready),
        .dbg_awaddr(dbg_awaddr), .dbg_awvalid(dbg_awvalid), .dbg_awready(dbg_awready),
        .dbg_wdata(dbg_wdata), .dbg_wstrb(dbg_wstrb), .dbg_wvalid(dbg_wvalid),
        .dbg_wready(dbg_wready), .dbg_bresp(dbg_bresp), .dbg_bvalid(dbg_bvalid),
        .dbg_bready(dbg_bready), .dbg_araddr(dbg_araddr), .dbg_arvalid(dbg_arvalid),
        .dbg_arready(dbg_arready), .dbg_rdata(dbg_rdata), .dbg_rresp(dbg_rresp),
        .dbg_rvalid(dbg_rvalid), .dbg_rready(dbg_rready),
        .prov_awaddr(prov_awaddr), .prov_awvalid(prov_awvalid), .prov_awready(prov_awready),
        .prov_wdata(prov_wdata), .prov_wstrb(prov_wstrb), .prov_wvalid(prov_wvalid),
        .prov_wready(prov_wready), .prov_bresp(prov_bresp), .prov_bvalid(prov_bvalid),
        .prov_bready(prov_bready), .prov_araddr(prov_araddr), .prov_arvalid(prov_arvalid),
        .prov_arready(prov_arready), .prov_rdata(prov_rdata), .prov_rresp(prov_rresp),
        .prov_rvalid(prov_rvalid), .prov_rready(prov_rready),
        .dafb_awaddr(pb_dafb_awaddr), .dafb_awvalid(pb_dafb_awvalid),
        .dafb_awready(pb_dafb_awready),
        .dafb_wdata(pb_dafb_wdata), .dafb_wstrb(pb_dafb_wstrb),
        .dafb_wvalid(pb_dafb_wvalid), .dafb_wready(pb_dafb_wready),
        .dafb_bresp(pb_dafb_bresp), .dafb_bvalid(pb_dafb_bvalid),
        .dafb_bready(pb_dafb_bready), .dafb_araddr(pb_dafb_araddr),
        .dafb_arvalid(pb_dafb_arvalid), .dafb_arready(pb_dafb_arready),
        .dafb_rdata(pb_dafb_rdata), .dafb_rresp(pb_dafb_rresp),
        .dafb_rvalid(pb_dafb_rvalid), .dafb_rready(pb_dafb_rready),
        .via1_addr(via1_addr), .via1_wdata(via1_wdata), .via1_wr(via1_wr),
        .via1_rd(via1_rd), .via1_rdata(via1_rdata), .via1_ack(via1_ack),
        .via2_addr(via2_addr), .via2_wdata(via2_wdata), .via2_wr(via2_wr),
        .via2_rd(via2_rd), .via2_rdata(via2_rdata), .via2_ack(via2_ack),
        .enet_addr(enet_addr), .enet_wdata(enet_wdata), .enet_wr(enet_wr),
        .enet_rd(enet_rd), .enet_rdata(enet_rdata), .enet_ack(enet_ack),
        .sonic_addr(sonic_addr), .sonic_wdata(sonic_wdata), .sonic_wr(sonic_wr),
        .sonic_wstrb(sonic_wstrb),
        .sonic_rd(sonic_rd), .sonic_rdata(sonic_rdata), .sonic_ack(sonic_ack),
        .orwell_addr(orwell_addr), .orwell_wdata(orwell_wdata), .orwell_wr(orwell_wr),
        .orwell_rd(orwell_rd), .orwell_rdata(orwell_rdata), .orwell_ack(orwell_ack),
        .scc_addr(scc_addr), .scc_wdata(scc_wdata), .scc_wr(scc_wr),
        .scc_rd(scc_rd), .scc_rdata(scc_rdata), .scc_ack(scc_ack),
        .scsi_addr(scsi_addr), .scsi_wdata(scsi_wdata), .scsi_wr(scsi_wr),
        .scsi_rd(scsi_rd), .scsi_rdata(scsi_rdata), .scsi_ack(scsi_ack),
        .scsi_dma_rd_ready(scsi_dma_rd_ready),
        .scsi_dma_wr_ready(scsi_dma_wr_ready),
        .scsi_dma16_lo_beat(scsi_dma16_lo_beat),
        .asc_addr(asc_addr), .asc_wdata(asc_wdata), .asc_wr(asc_wr),
        .asc_rd(asc_rd), .asc_rdata(asc_rdata), .asc_ack(asc_ack),
        .iwm_addr(iwm_addr), .iwm_wdata(iwm_wdata), .iwm_wr(iwm_wr),
        .iwm_rd(iwm_rd), .iwm_rdata(iwm_rdata), .iwm_ack(iwm_ack)
    );

    wire [7:0] via1_pa_out, via1_pa_mask, via1_pb_out, via1_pb_mask;
    wire       via1_overlay_bit;
    wire       via1_adb_rx_ready, via1_adb_tx_valid;
    wire [7:0] via1_adb_tx_byte;
    wire       rtc_enb, rtc_clk, rtc_data_o, rtc_data_oe, rtc_data_i, rtc_cko;
    wire [7:0] via2_pa_out, via2_pa_mask, via2_pb_out, via2_pb_mask;
    wire       via2_ca2_out, via2_cb1_out, via2_cb2_out;
    wire       via2_pb7_ca1 = via2_pb_mask[7] ? via2_pb_out[7] : 1'b1;
    wire       sonic_irq;

    via1 #(
        .ENABLE_INTERNAL_VBL(1'b0)
    ) u_via1 (
        .clk(clk), .rst(rst), .phi2_tick(phi2_tick),
        .pb_addr(via1_addr), .pb_wdata(via1_wdata), .pb_wr(via1_wr),
        .pb_rd(via1_rd), .pb_rdata(via1_rdata), .pb_ack(via1_ack),
        // Match MAME macqd700/Vmac_top reset straps so the Universal ROM
        // accepts the Q700 descriptor during the low-ROM table scan.
        .pa_in(8'hc1), .pb_in({4'b0000, 3'b100, rtc_data_i}),
        .pa_out(via1_pa_out), .pa_mask(via1_pa_mask),
        .pb_out(via1_pb_out), .pb_mask(via1_pb_mask),
        .overlay_bit(via1_overlay_bit),
        .adb_rx_byte(8'h00), .adb_rx_valid(1'b0),
        .adb_rx_ready(via1_adb_rx_ready),
        .adb_tx_byte(via1_adb_tx_byte), .adb_tx_valid(via1_adb_tx_valid),
        .rtc_enb(rtc_enb), .rtc_clk(rtc_clk), .rtc_data_o(rtc_data_o),
        .rtc_data_oe(rtc_data_oe), .rtc_data_i(rtc_data_i), .rtc_cko(rtc_cko),
        .vblank_irq_in(via2_pb7_ca1), .irq(via1_irq)
    );

    rtc #(
        .SEC_DIV(VIA_HZ)
    ) u_rtc (
        .clk(clk), .rst(rst), .phi2_tick(phi2_tick),
        .rtc_enb(rtc_enb), .rtc_clk(rtc_clk), .rtc_data_o(rtc_data_o),
        // PRAM is battery-backed (survives rst); no zap source here.
        .rtc_data_oe(rtc_data_oe), .pram_clear(1'b0),
        .pram_busy(),
        // PRAM snapshot/restore back door (rtl/soc/pram_sd.v) — unused here.
        .pram_ext_addr(8'h00), .pram_ext_we(1'b0),
        .pram_ext_wdata(8'h00), .pram_ext_rdata(),
        .cko(rtc_cko), .rtc_data_i(rtc_data_i)
    );

    via2 u_via2 (
        .clk(clk), .rst(rst), .phi2_tick(phi2_tick),
        .pb_addr(via2_addr), .pb_wdata(via2_wdata), .pb_wr(via2_wr),
        .pb_rd(via2_rd), .pb_rdata(via2_rdata), .pb_ack(via2_ack),
        .pa_in({~scsi_irq, ~scsi_drq, 5'h1f, ~sonic_irq}),
        .pa_out(via2_pa_out), .pa_mask(via2_pa_mask),
        // Match MAME/Q700 reset straps: no slot IRQs, TM0A/TM1A high.
        .pb_in(8'hcf), .pb_out(via2_pb_out), .pb_mask(via2_pb_mask),
        // Q700 routes ASC/EASC IRQ to VIA2 CB1 and SCSI IRQ to VIA2 CB2
        // through active-low board glue, matching MAME's inverted callbacks.
        .ca1_in(1'b1), .ca2_in(1'b1), .cb1_in(~asc_irq), .cb2_in(~scsi_irq),
        .ca2_out(via2_ca2_out), .cb1_out(via2_cb1_out), .cb2_out(via2_cb2_out),
        .irq(via2_irq)
    );

    scc u_scc (
        .clk(clk), .rst(rst), .pb_addr(scc_addr), .pb_wdata(scc_wdata),
        .pb_wr(scc_wr), .pb_rd(scc_rd), .pb_rdata(scc_rdata),
        .pb_ack(scc_ack),
        .rx_a_valid(scc_rx_a_valid), .rx_a_data(scc_rx_a_data),
        .rx_b_valid(scc_rx_b_valid), .rx_b_data(scc_rx_b_data),
        .cts_a_n(scc_cts_a_n), .dcd_a_n(scc_dcd_a_n), .sync_a_n(scc_sync_a_n),
        .cts_b_n(scc_cts_b_n), .dcd_b_n(scc_dcd_b_n), .sync_b_n(scc_sync_b_n),
        .tx_a_valid(scc_tx_a_valid), .tx_a_data(scc_tx_a_data),
        .tx_b_valid(scc_tx_b_valid), .tx_b_data(scc_tx_b_data),
        .rts_a_n(scc_rts_a_n), .dtr_a_n(scc_dtr_a_n),
        .rts_b_n(scc_rts_b_n), .dtr_b_n(scc_dtr_b_n),
        .irq(scc_irq)
    );

    scsi #(.TURBOSCSI_C96(1'b1)) u_scsi (
        // Write-protect is a vhdd_ctrl runtime setting (CTRL bit 2), not a
        // property of the SCSI target; these harnesses predate it and test
        // the writable behaviour, so tie it off.
        .wprot(1'b0),
        .clk(clk), .rst(rst), .pb_addr(scsi_addr), .pb_wdata(scsi_wdata),
        .pb_dma16_lo_beat(scsi_dma16_lo_beat),
        .pb_wr(scsi_wr), .pb_rd(scsi_rd), .pb_rdata(scsi_rdata),
        .pb_ack(scsi_ack), .irq(scsi_irq), .drq(scsi_drq),
        .dma_rd_ready(scsi_dma_rd_ready), .dma_wr_ready(scsi_dma_wr_ready),
        // No backing store in this harness.  vh_chk_ok is tied LOW, which
        // is exactly what the internal LBA mapper used to produce here:
        // disk_num_lbas was never connected, so its capacity check
        // (end_lba <= num_lbas) failed for every request and reads fell
        // through to the "out of range" CHECK CONDITION path.
        .vh_chk_lba(), .vh_chk_blocks(), .vh_chk_ok(1'b0),
        .vh_req_write(), .vh_req_multi(), .vh_req_lba(),
        .vh_req_block_count(), .vh_req_go(),
        .vh_busy(1'b0), .vh_done(1'b0), .vh_error(1'b0),
        .vh_rd_valid(1'b0), .vh_rd_data(8'h00), .vh_rd_ready(),
        .vh_wr_ready(1'b0), .vh_wr_valid(), .vh_wr_data(),
        .vh_wr_avail()
    );

    wire [15:0] asc_sample;
    wire [15:0] asc_pcm_l;
    wire [15:0] asc_pcm_r;
    wire        asc_sample_valid;
    asc u_asc (
        .clk(clk), .rst(rst), .phi2_tick(phi2_tick),
        .pb_addr(asc_addr), .pb_wdata(asc_wdata), .pb_wr(asc_wr),
        .pb_rd(asc_rd), .pb_rdata(asc_rdata), .pb_ack(asc_ack),
        .audio_sample_out(asc_sample),
        .audio_pcm_l(asc_pcm_l), .audio_pcm_r(asc_pcm_r),
        .audio_sample_valid(asc_sample_valid),
        .irq(asc_irq)
    );

    wire [7:0] iwm_mode;
    wire       iwm_dma_req;
    iwm_stub u_iwm (
        .clk(clk), .rst(rst), .cs(iwm_wr | iwm_rd), .rd(iwm_rd), .wr(iwm_wr),
        .reg_sel(iwm_addr), .wdata(iwm_wdata), .drive_present(1'b0),
        .rdata(iwm_rdata), .irq(iwm_irq), .dma_req(iwm_dma_req),
        .mode_o(iwm_mode)
    );
    assign iwm_ack = iwm_wr | iwm_rd;

    q700_eth_sonic u_eth_sonic (
        .clk(clk),
        .rst(rst),
        .enet_cs(enet_wr | enet_rd),
        .enet_rd(enet_rd),
        .enet_wr(enet_wr),
        .enet_addr(enet_addr),
        .enet_wdata(enet_wdata),
        .enet_rdata(enet_rdata),
        .enet_ack(enet_ack),
        .sonic_cs(sonic_wr | sonic_rd),
        .sonic_rd(sonic_rd),
        .sonic_wr(sonic_wr),
        .sonic_addr(sonic_addr),
        .sonic_wdata(sonic_wdata),
        .sonic_wstrb(sonic_wstrb),
        .sonic_rdata(sonic_rdata),
        .sonic_ack(sonic_ack),
        .sonic_irq(sonic_irq)
    );

    orwell_stub u_orwell (
        .cs(orwell_wr | orwell_rd),
        .rd(orwell_rd),
        .wr(orwell_wr),
        .addr(orwell_addr),
        .wdata(orwell_wdata),
        .rdata(orwell_rdata),
        .ack(orwell_ack)
    );

    mame_axi_stub32 #(.RESET_VALUE(32'h0000_0000)) u_dbg_stub (
        .clk(clk), .rst(rst),
        .awaddr({12'h000, dbg_awaddr}), .awvalid(dbg_awvalid), .awready(dbg_awready),
        .wdata(dbg_wdata), .wstrb(dbg_wstrb), .wvalid(dbg_wvalid), .wready(dbg_wready),
        .bresp(dbg_bresp), .bvalid(dbg_bvalid), .bready(dbg_bready),
        .araddr({12'h000, dbg_araddr}), .arvalid(dbg_arvalid), .arready(dbg_arready),
        .rdata(dbg_rdata), .rresp(dbg_rresp), .rvalid(dbg_rvalid), .rready(dbg_rready)
    );

    mame_axi_stub32 #(.RESET_VALUE(32'h0000_0000)) u_prov_stub (
        .clk(clk), .rst(rst),
        .awaddr({12'h000, prov_awaddr}), .awvalid(prov_awvalid), .awready(prov_awready),
        .wdata(prov_wdata), .wstrb(prov_wstrb), .wvalid(prov_wvalid), .wready(prov_wready),
        .bresp(prov_bresp), .bvalid(prov_bvalid), .bready(prov_bready),
        .araddr({12'h000, prov_araddr}), .arvalid(prov_arvalid), .arready(prov_arready),
        .rdata(prov_rdata), .rresp(prov_rresp), .rvalid(prov_rvalid), .rready(prov_rready)
    );

    video u_dafb (
        .clk(clk), .rst(rst),
        .s_axi_awaddr(dafb_awaddr), .s_axi_awvalid(dafb_awvalid),
        .s_axi_awready(dafb_awready),
        .s_axi_wdata(dafb_wdata), .s_axi_wstrb(dafb_wstrb),
        .s_axi_wvalid(dafb_wvalid), .s_axi_wready(dafb_wready),
        .s_axi_bresp(dafb_bresp), .s_axi_bvalid(dafb_bvalid),
        .s_axi_bready(dafb_bready),
        .s_axi_araddr(dafb_araddr), .s_axi_arvalid(dafb_arvalid),
        .s_axi_arready(dafb_arready),
        .s_axi_rdata(dafb_rdata), .s_axi_rresp(dafb_rresp),
        .s_axi_rvalid(dafb_rvalid), .s_axi_rready(dafb_rready),
        .fb_base_px(dafb_fb_base_px), .fb_stride_px(dafb_fb_stride_px),
        .fb_bpp_reg(dafb_fb_bpp_reg),
        .fb_bytes_per_px (),
        .depth_supported (),
        .clut_we(dafb_clut_we), .clut_waddr(dafb_clut_waddr), .clut_wdata(dafb_clut_wdata),
        .irq(dafb_irq),
        // Vblank cadence out of scope for this bridge — tie low.
        .frame_tick(1'b0),
        // Monitor sense: pin the historical MONITOR_TYPE default (7'h06)
        // so the MAME lockstep comparison is unaffected.
        .monitor_sense(7'h06)
    );

    assign pb_dafb_awready = 1'b0;
    assign pb_dafb_wready = 1'b0;
    assign pb_dafb_bresp = 2'b11;
    assign pb_dafb_bvalid = 1'b0;
    assign pb_dafb_arready = 1'b0;
    assign pb_dafb_rdata = 32'h0000_0000;
    assign pb_dafb_rresp = 2'b11;
    assign pb_dafb_rvalid = 1'b0;

    wire _unused = |{enet_addr, enet_wdata, sonic_addr, sonic_wdata,
                     orwell_addr, orwell_wdata, via1_pa_out, via1_pa_mask,
                     via1_pb_out, via1_pb_mask, via1_overlay_bit,
                     via1_adb_rx_ready, via1_adb_tx_byte, via1_adb_tx_valid,
                     rtc_enb, rtc_clk, rtc_data_o, rtc_data_oe, rtc_cko, via2_pa_out,
                     via2_pa_mask, via2_pb_out, via2_pb_mask, via2_ca2_out,
                     via2_cb1_out, via2_cb2_out, asc_sample, asc_pcm_l,
                     asc_pcm_r, asc_sample_valid,
                     iwm_dma_req, iwm_mode, dafb_fb_base_px, dafb_fb_stride_px,
                     dafb_fb_bpp_reg, dafb_clut_we, dafb_clut_waddr, dafb_clut_wdata,
                     pb_dafb_awaddr,
                     pb_dafb_awvalid, pb_dafb_wdata, pb_dafb_wstrb,
                     pb_dafb_wvalid, pb_dafb_bready, pb_dafb_araddr,
                     pb_dafb_arvalid, pb_dafb_rready};
endmodule

module mame_axi_stub32 #(
    parameter [31:0] RESET_VALUE = 32'h0000_0000
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] awaddr,
    input  wire        awvalid,
    output wire        awready,
    input  wire [31:0] wdata,
    input  wire [3:0]  wstrb,
    input  wire        wvalid,
    output wire        wready,
    output wire [1:0]  bresp,
    output reg         bvalid,
    input  wire        bready,
    input  wire [31:0] araddr,
    input  wire        arvalid,
    output wire        arready,
    output reg  [31:0] rdata,
    output wire [1:0]  rresp,
    output reg         rvalid,
    input  wire        rready
);
    reg [31:0] reg0;
    reg        aw_seen;
    reg        w_seen;
    assign awready = !bvalid && !aw_seen;
    assign wready  = !bvalid && !w_seen;
    assign bresp   = 2'b00;
    assign arready = !rvalid;
    assign rresp   = 2'b00;

    always @(posedge clk) begin
        if (rst) begin
            reg0 <= RESET_VALUE;
            bvalid <= 1'b0;
            rvalid <= 1'b0;
            aw_seen <= 1'b0;
            w_seen <= 1'b0;
            rdata <= 32'h0000_0000;
        end else begin
            if (bvalid && bready) begin
                bvalid <= 1'b0;
                aw_seen <= 1'b0;
                w_seen <= 1'b0;
            end
            if (!bvalid && awvalid && awready)
                aw_seen <= 1'b1;
            if (!bvalid && wvalid && wready) begin
                if (wstrb[0]) reg0[7:0]   <= wdata[7:0];
                if (wstrb[1]) reg0[15:8]  <= wdata[15:8];
                if (wstrb[2]) reg0[23:16] <= wdata[23:16];
                if (wstrb[3]) reg0[31:24] <= wdata[31:24];
                w_seen <= 1'b1;
            end
            if (!bvalid && ((aw_seen || (awvalid && awready)) &&
                            (w_seen || (wvalid && wready)))) begin
                bvalid <= 1'b1;
            end

            if (rvalid && rready)
                rvalid <= 1'b0;
            if (!rvalid && arvalid) begin
                rdata <= reg0 ^ {araddr[15:0], araddr[15:0]};
                rvalid <= 1'b1;
            end
        end
    end
endmodule

module mame_axi_xbar_sink #(
    parameter ID_WIDTH = 6,
    parameter DATA_WIDTH = 128,
    parameter STRB_WIDTH = DATA_WIDTH/8,
    parameter [1:0] RESP = 2'b11
) (
    input  wire                  clk,
    input  wire                  rst,
    input  wire [ID_WIDTH-1:0]   awid,
    input  wire [31:0]           awaddr,
    input  wire [7:0]            awlen,
    input  wire [2:0]            awsize,
    input  wire [1:0]            awburst,
    input  wire                  awvalid,
    output wire                  awready,
    input  wire [DATA_WIDTH-1:0] wdata,
    input  wire [STRB_WIDTH-1:0] wstrb,
    input  wire                  wlast,
    input  wire                  wvalid,
    output wire                  wready,
    output wire [ID_WIDTH-1:0]   bid,
    output wire [1:0]            bresp,
    output reg                   bvalid,
    input  wire                  bready,
    input  wire [ID_WIDTH-1:0]   arid,
    input  wire [31:0]           araddr,
    input  wire [7:0]            arlen,
    input  wire [2:0]            arsize,
    input  wire [1:0]            arburst,
    input  wire                  arvalid,
    output wire                  arready,
    output reg [ID_WIDTH-1:0]    rid,
    output wire [DATA_WIDTH-1:0] rdata,
    output wire [1:0]            rresp,
    output wire                  rlast,
    output reg                   rvalid,
    input  wire                  rready
);
    reg [ID_WIDTH-1:0] bid_q;
    reg aw_seen;
    reg w_seen;

    assign awready = !bvalid && !aw_seen;
    assign wready = !bvalid && !w_seen;
    assign bid = bid_q;
    assign bresp = RESP;
    assign arready = !rvalid;
    assign rdata = {DATA_WIDTH{1'b0}};
    assign rresp = RESP;
    assign rlast = 1'b1;

    always @(posedge clk) begin
        if (rst) begin
            bvalid <= 1'b0;
            rvalid <= 1'b0;
            aw_seen <= 1'b0;
            w_seen <= 1'b0;
            bid_q <= {ID_WIDTH{1'b0}};
            rid <= {ID_WIDTH{1'b0}};
        end else begin
            if (bvalid && bready) begin
                bvalid <= 1'b0;
                aw_seen <= 1'b0;
                w_seen <= 1'b0;
            end
            if (!bvalid && awvalid && awready) begin
                aw_seen <= 1'b1;
                bid_q <= awid;
            end
            if (!bvalid && wvalid && wready)
                w_seen <= 1'b1;
            if (!bvalid && (aw_seen || (awvalid && awready)) &&
                (w_seen || (wvalid && wready))) begin
                bvalid <= 1'b1;
            end

            if (rvalid && rready)
                rvalid <= 1'b0;
            if (!rvalid && arvalid && arready) begin
                rid <= arid;
                rvalid <= 1'b1;
            end
        end
    end

    wire _unused = |{awaddr, awlen, awsize, awburst, wdata, wstrb, wlast,
                     araddr, arlen, arsize, arburst};
endmodule

`default_nettype wire
