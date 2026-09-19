// tb_fb_reader_ddr_chain.v -- REAL fb_reader.v driven through the T14
// VRAM-in-DDR chain (T14 follow-up: fb_reader/video_top integration gap).
//
// tb-vram-ddr-chain (tb_vram_ddr_chain.v) drives scanout_ddr_reader.v's
// streaming port directly from a C++ BFM -- it never exercises the REAL
// fb_reader.v (the pclk<->vram_clk CDC bridge + credit machinery that
// video_top.v actually uses in production). This wrapper closes that gap
// with the smallest arrangement that still uses the real module: CPU AXI
// writes (raw, bypassing axi_xbar's S3-swap boundary -- irrelevant here
// since fb_reader/scanout_ddr_reader both operate in VRAM-native byte
// order, matching how video_top.v's own vram_rd_data path already
// bypasses that swap, see fpga_top_video.vh's byte-swap comment) land a
// known pattern in the carveout via axi_async_bridge directly; fb_reader
// then streams it back through vram_clk-domain scanout_ddr_reader and
// pclk-domain s_rd_*, for a pixel-exact comparison. Deliberately skips
// scanout_placement_sync/linebuf_scanout (scan-timing generation, not
// this seam) -- the C++ driver requests pixels directly.
//
// Two clock domains: pclk (fb_reader's scaler-facing side) and core_clk
// (scanout_ddr_reader + the rest of the T13/T14 DDR chain, matching
// fpga_top_video.vh's `.rd_clk(core_clk)` convention -- vram_clk IS
// core_clk in production). mig_clk is a third, independent domain
// matching the real MIG UI clock crossing inside axi_async_bridge.
//
`default_nettype none
`include "axi_defs.vh"

module tb_fb_reader_ddr_chain (
    input  wire         pclk,
    input  wire         core_clk,
    input  wire         core_rst,
    // PCLK-DOMAIN-ONLY reset.  In PRODUCTION fb_reader's `resetn` is
    // video_top's pclk `resetn_bank[2]` -- derived from the HDMI MMCM's
    // LOCKED output -- while its `vram_rst` and scanout_ddr_reader's `rst`
    // are the core_clk-domain `vram_rd_rst`.  Those are DIFFERENT NETS: an
    // MMCM relock (JTAG reload, ref-clock blip, cold boot) asserts the pclk
    // one WITHOUT asserting the core_clk one.  This tb used to tie all three
    // to `core_rst`, which made that skew -- and therefore task #194 --
    // structurally unobservable here.  Driving this input alone reproduces
    // the production event exactly.
    input  wire         pclk_rst,
    input  wire         mig_clk,
    input  wire         mig_rst,

    // ── fb_reader's scaler-facing (pclk) port ───────────────────────────
    input  wire              s_rd_en,
    input  wire [20:0]       s_rd_addr,
    output wire               s_rd_ready,
    // 4-byte group starting at the requested address: [31:24] is the byte at
    // s_rd_addr (valid at any alignment), [23:0] the bytes at +1/+2/+3
    // (architecturally valid only when s_rd_addr[1:0]==0).  fb_reader is
    // width-agnostic; the contract comes from scanout_ddr_reader.v below.
    output wire [31:0]        s_rd_data,
    output wire                s_rd_valid,
    output wire                underflow_sticky,

    // ── Raw CPU-shaped AXI4 write port (bypasses axi_xbar; see header) ──
    input  wire [5:0]   cpu_awid, input wire [31:0] cpu_awaddr,
    input  wire [7:0]   cpu_awlen, input wire [2:0] cpu_awsize, input wire [1:0] cpu_awburst,
    input  wire cpu_awvalid, output wire cpu_awready,
    input  wire [127:0] cpu_wdata, input wire [15:0] cpu_wstrb,
    input  wire cpu_wlast, input wire cpu_wvalid, output wire cpu_wready,
    output wire [5:0] cpu_bid, output wire [1:0] cpu_bresp,
    output wire cpu_bvalid, input wire cpu_bready,

    output wire cal_done,

    // ── Reset-skew observability (task #194) ────────────────────────────
    // The two accountings that MUST stay in step: fb_reader's credit
    // counter (requests issued, responses not yet returned) and
    // scanout_ddr_reader's request-queue occupancy.  If a reset clears one
    // and not the other they desynchronise, and the ordered response stream
    // stops corresponding to the request stream.
    output wire [5:0]  dbg_in_flight_cnt,
    output wire [6:0]  dbg_q_count
);

    // PRODUCTION SHAPE: fb_reader's pclk-side reset is its OWN net, not
    // core_rst.  `core_rst` is kept in the OR only so the existing
    // scenarios (which drive core_rst alone) behave exactly as before.
    wire resetn = !(core_rst || pclk_rst);

    // ── COUPLED SCAN-OUT RESET, mirroring video_top.v (task #194) ───────
    // video_top synchronises its pclk-domain reset into vram_clk and OR-s
    // it with vram_rst, then drives fb_reader's vram half AND the VRAM
    // read port from that one term (its `vram_side_rst` output).  This tb
    // reproduces that wiring so it measures the production topology, not a
    // simplification of it.  Fed from the RAW pclk reset, never from the
    // coupled result -- a coupled->coupled path is a ring that can hold
    // itself asserted.
    (* ASYNC_REG = "TRUE" *) reg [1:0] pclk_rst_core_sync;
    always @(posedge core_clk) begin
        if (core_rst) pclk_rst_core_sync <= 2'b11;
        else          pclk_rst_core_sync <= {pclk_rst_core_sync[0], pclk_rst};
    end
    wire sc_vram_rst = core_rst | pclk_rst_core_sync[1];

    assign dbg_in_flight_cnt = u_fb_reader.in_flight_cnt;
    assign dbg_q_count       = u_scan.q_count;

    // ── scanout_ddr_reader (vram.v-compatible streaming port) ──────────
    wire        v_rd_en;
    wire [20:0] v_rd_addr;
    wire [31:0] v_rd_data;
    wire        v_rd_valid;

    wire [5:0]   scan_arid;   wire [31:0]  scan_araddr; wire [7:0] scan_arlen;
    wire [2:0]   scan_arsize; wire [1:0]   scan_arburst; wire scan_arvalid, scan_arready;
    wire [5:0]   scan_rid;    wire [127:0] scan_rdata;   wire [1:0] scan_rresp;
    wire scan_rlast, scan_rvalid, scan_rready;

    scanout_ddr_reader #(
        // BPP=8 is the read-port ADDRESS granularity (1 byte per index);
        // RD_DATA_W=32 is the (widened) read-port DATA width.
        .ADDR_W(21), .BPP(8), .RD_DATA_W(32),
        .DATA_WIDTH(128), .ID_WIDTH(6), .AXI_ID(6'h00),
        .CARVEOUT_BASE(`AXI_VRAM_DDR_CARVEOUT_BASE),
        .CARVEOUT_SIZE(`AXI_VRAM_DDR_CARVEOUT_SIZE)
    ) u_scan (
        // COUPLED scan-out reset (task #194) -- the same net fb_reader's
        // vram half uses, so the request queue here and fb_reader's credit
        // counter can never be cleared independently.
        .clk(core_clk), .rst(sc_vram_rst),
        .rd_clk(core_clk), .rd_rst(sc_vram_rst),
        .rd_addr(v_rd_addr), .rd_en(v_rd_en),
        .rd_data(v_rd_data), .rd_valid(v_rd_valid),
        .m_arid(scan_arid), .m_araddr(scan_araddr), .m_arlen(scan_arlen),
        .m_arsize(scan_arsize), .m_arburst(scan_arburst),
        .m_arvalid(scan_arvalid), .m_arready(scan_arready),
        .m_rid(scan_rid), .m_rdata(scan_rdata), .m_rresp(scan_rresp),
        .m_rlast(scan_rlast), .m_rvalid(scan_rvalid), .m_rready(scan_rready)
    );

    // ── fb_reader (REAL module) -- retuned INFLIGHT_WIDTH per its own
    //    T14 header note (DDR latency vs. vram.v's fixed ~6 cycles). ────
    fb_reader #(
        .ADDR_W(21), .DATA_W(32), .RETURN_LATENCY(48),
        .REQ_FIFO_DEPTH_LOG2(8), .RSP_FIFO_DEPTH_LOG2(8),
        .INFLIGHT_WIDTH(6)
    ) u_fb_reader (
        .pclk(pclk), .resetn(resetn),
        .vram_clk(core_clk), .vram_rst(sc_vram_rst),
        .s_rd_en(s_rd_en), .s_rd_addr(s_rd_addr),
        .s_rd_ready(s_rd_ready), .s_rd_data(s_rd_data), .s_rd_valid(s_rd_valid),
        .v_rd_en(v_rd_en), .v_rd_addr(v_rd_addr),
        .v_rd_data(v_rd_data), .v_rd_valid(v_rd_valid),
        .underflow_sticky(underflow_sticky),
        .req_count(), .rsp_count(), .miss_count()
    );

    // ── cpu-path (stands in for l2c/S0 -- this tb has no xbar/l2c, see
    //    header) AW/W/B + AR/R (unused, tied 0): the "l2c" input of the
    //    3-way arbiter. S3 lane: tied inactive -- this tb never drives
    //    VRAM-aperture CPU traffic, only scanout reads (that's
    //    tb-vram-ddr-chain's job, via the real xbar S3 port). Since s3_*
    //    never asserts *valid, it never wins any arbitration and the
    //    merged port behaves exactly as the old direct-wire topology did
    //    for this tb (single real writer, cpu-path; single real AR/R
    //    contender pair, cpu-path (inactive) vs scanout). ─────────────
    axi_vram_priority_mux3 #(.ID_WIDTH(6), .ADDR_WIDTH(32), .DATA_WIDTH(128),
                             .EXTERNAL_WRITE_RESET_RECOVERY(1)) u_mux (
        .clk(core_clk), .rst(core_rst),
        .l2c_awid(cpu_awid), .l2c_awaddr(cpu_awaddr), .l2c_awlen(cpu_awlen),
        .l2c_awsize(cpu_awsize), .l2c_awburst(cpu_awburst),
        .l2c_awvalid(cpu_awvalid), .l2c_awready(cpu_awready),
        .l2c_wdata(cpu_wdata), .l2c_wstrb(cpu_wstrb), .l2c_wlast(cpu_wlast),
        .l2c_wvalid(cpu_wvalid), .l2c_wready(cpu_wready),
        .l2c_bid(cpu_bid), .l2c_bresp(cpu_bresp), .l2c_bvalid(cpu_bvalid), .l2c_bready(cpu_bready),
        .l2c_arid(6'd0), .l2c_araddr(32'd0), .l2c_arlen(8'd0),
        .l2c_arsize(3'd0), .l2c_arburst(2'd0),
        .l2c_arvalid(1'b0), .l2c_arready(),
        .l2c_rid(), .l2c_rdata(), .l2c_rresp(), .l2c_rlast(), .l2c_rvalid(), .l2c_rready(1'b1),
        .s3_awid(6'd0), .s3_awaddr(32'd0), .s3_awlen(8'd0),
        .s3_awsize(3'd0), .s3_awburst(2'd0),
        .s3_awvalid(1'b0), .s3_awready(),
        .s3_wdata(128'd0), .s3_wstrb(16'd0), .s3_wlast(1'b0),
        .s3_wvalid(1'b0), .s3_wready(),
        .s3_bid(), .s3_bresp(), .s3_bvalid(), .s3_bready(1'b1),
        .s3_arid(6'd0), .s3_araddr(32'd0), .s3_arlen(8'd0),
        .s3_arsize(3'd0), .s3_arburst(2'd0),
        .s3_arvalid(1'b0), .s3_arready(),
        .s3_rid(), .s3_rdata(), .s3_rresp(), .s3_rlast(), .s3_rvalid(), .s3_rready(1'b1),
        .scan_arid(scan_arid), .scan_araddr(scan_araddr), .scan_arlen(scan_arlen),
        .scan_arsize(scan_arsize), .scan_arburst(scan_arburst),
        .scan_arvalid(scan_arvalid), .scan_arready(scan_arready),
        .scan_rid(scan_rid), .scan_rdata(scan_rdata), .scan_rresp(scan_rresp),
        .scan_rlast(scan_rlast), .scan_rvalid(scan_rvalid), .scan_rready(scan_rready),
        .m_awid(m_awid), .m_awaddr(m_awaddr), .m_awlen(m_awlen),
        .m_awsize(m_awsize), .m_awburst(m_awburst),
        .m_awvalid(m_awvalid), .m_awready(m_awready),
        .m_wdata(m_wdata), .m_wstrb(m_wstrb), .m_wlast(m_wlast),
        .m_wvalid(m_wvalid), .m_wready(m_wready),
        .m_bid(m_bid), .m_bresp(m_bresp), .m_bvalid(m_bvalid), .m_bready(m_bready),
        .m_arid(m_arid), .m_araddr(m_araddr), .m_arlen(m_arlen),
        .m_arsize(m_arsize), .m_arburst(m_arburst),
        .m_arvalid(m_arvalid), .m_arready(m_arready),
        .m_rid(m_rid), .m_rdata(m_rdata), .m_rresp(m_rresp),
        .m_rlast(m_rlast), .m_rvalid(m_rvalid), .m_rready(m_rready)
    );
    wire [5:0]   m_awid;   wire [31:0]  m_awaddr; wire [7:0] m_awlen;
    wire [2:0]   m_awsize; wire [1:0]   m_awburst; wire m_awvalid, m_awready;
    wire [127:0] m_wdata;  wire [15:0]  m_wstrb;   wire m_wlast, m_wvalid, m_wready;
    wire [5:0]   m_bid;    wire [1:0]   m_bresp;   wire m_bvalid, m_bready;
    wire [5:0] m_arid; wire [31:0] m_araddr; wire [7:0] m_arlen;
    wire [2:0] m_arsize; wire [1:0] m_arburst; wire m_arvalid, m_arready;
    wire [5:0] m_rid; wire [127:0] m_rdata; wire [1:0] m_rresp;
    wire m_rlast, m_rvalid, m_rready;

    wire [5:0]   ui_awid;   wire [31:0]  ui_awaddr;  wire [7:0] ui_awlen;
    wire [2:0]   ui_awsize; wire [1:0]   ui_awburst; wire ui_awvalid, ui_awready;
    wire [127:0] ui_wdata;  wire [15:0]  ui_wstrb;   wire ui_wlast, ui_wvalid, ui_wready;
    wire [5:0]   ui_bid;    wire [1:0]   ui_bresp;   wire ui_bvalid, ui_bready;
    wire [5:0]   ui_arid;   wire [31:0]  ui_araddr;  wire [7:0] ui_arlen;
    wire [2:0]   ui_arsize; wire [1:0]   ui_arburst; wire ui_arvalid, ui_arready;
    wire [5:0]   ui_rid;    wire [127:0] ui_rdata;   wire [1:0] ui_rresp;
    wire ui_rlast, ui_rvalid, ui_rready;

    axi_async_bridge #(.DATA_WIDTH(128), .ADDR_WIDTH(32), .ID_WIDTH(6), .USER_WIDTH(1)) u_bridge (
        .s_clk(core_clk), .s_rst(core_rst),
        // AW/W/B come from the merged arbiter output now (T16) -- the
        // arbiter's l2c_* input is what's fed by cpu_* above.
        .s_awid(m_awid), .s_awaddr(m_awaddr), .s_awlen(m_awlen),
        .s_awsize(m_awsize), .s_awburst(m_awburst), .s_awlock(1'b0),
        .s_awcache(4'b0011), .s_awprot(3'b000), .s_awqos(4'b0000),
        .s_awuser(1'b0), .s_awvalid(m_awvalid), .s_awready(m_awready),
        .s_wdata(m_wdata), .s_wstrb(m_wstrb), .s_wlast(m_wlast),
        .s_wuser(1'b0), .s_wvalid(m_wvalid), .s_wready(m_wready),
        .s_bid(m_bid), .s_bresp(m_bresp), .s_buser(), .s_bvalid(m_bvalid),
        .s_bready(m_bready),
        .s_arid(m_arid), .s_araddr(m_araddr), .s_arlen(m_arlen),
        .s_arsize(m_arsize), .s_arburst(m_arburst), .s_arlock(1'b0),
        .s_arcache(4'b0011), .s_arprot(3'b000), .s_arqos(4'b0000),
        .s_aruser(1'b0), .s_arvalid(m_arvalid), .s_arready(m_arready),
        .s_rid(m_rid), .s_rdata(m_rdata), .s_rresp(m_rresp), .s_rlast(m_rlast),
        .s_ruser(), .s_rvalid(m_rvalid), .s_rready(m_rready),

        .m_clk(mig_clk), .m_rst(mig_rst || !cal_done),
        .m_awid(ui_awid), .m_awaddr(ui_awaddr), .m_awlen(ui_awlen),
        .m_awsize(ui_awsize), .m_awburst(ui_awburst), .m_awlock(),
        .m_awcache(), .m_awprot(), .m_awqos(),
        .m_awuser(), .m_awvalid(ui_awvalid), .m_awready(ui_awready),
        .m_wdata(ui_wdata), .m_wstrb(ui_wstrb), .m_wlast(ui_wlast),
        .m_wuser(), .m_wvalid(ui_wvalid), .m_wready(ui_wready),
        .m_bid(ui_bid), .m_bresp(ui_bresp), .m_buser(1'b0),
        .m_bvalid(ui_bvalid), .m_bready(ui_bready),
        .m_arid(ui_arid), .m_araddr(ui_araddr), .m_arlen(ui_arlen),
        .m_arsize(ui_arsize), .m_arburst(ui_arburst), .m_arlock(),
        .m_arcache(), .m_arprot(), .m_arqos(),
        .m_aruser(), .m_arvalid(ui_arvalid), .m_arready(ui_arready),
        .m_rid(ui_rid), .m_rdata(ui_rdata), .m_rresp(ui_rresp),
        .m_rlast(ui_rlast), .m_ruser(1'b0), .m_rvalid(ui_rvalid),
        .m_rready(ui_rready)
    );

    wire [0:0]   mig_awid;  wire [30:0] mig_awaddr; wire [7:0] mig_awlen;
    wire [2:0]   mig_awsize; wire [1:0] mig_awburst; wire mig_awvalid, mig_awready;
    wire [255:0] mig_wdata; wire [31:0] mig_wstrb;   wire mig_wlast, mig_wvalid, mig_wready;
    wire [0:0]   mig_bid;   wire [1:0]  mig_bresp;   wire mig_bvalid, mig_bready;
    wire [0:0]   mig_arid;  wire [30:0] mig_araddr;  wire [7:0] mig_arlen;
    wire [2:0]   mig_arsize; wire [1:0] mig_arburst; wire mig_arvalid, mig_arready;
    wire [0:0]   mig_rid;   wire [255:0] mig_rdata;  wire [1:0] mig_rresp;
    wire mig_rlast, mig_rvalid, mig_rready;

    axi_ddr4_mig_bridge u_repo_to_pcie_mig (
        .clk(mig_clk), .rst(mig_rst || !cal_done),
        .s_awid(ui_awid), .s_awaddr(ui_awaddr), .s_awlen(ui_awlen),
        .s_awsize(ui_awsize), .s_awburst(ui_awburst),
        .s_awvalid(ui_awvalid), .s_awready(ui_awready),
        .s_wdata(ui_wdata), .s_wstrb(ui_wstrb), .s_wlast(ui_wlast),
        .s_wvalid(ui_wvalid), .s_wready(ui_wready),
        .s_bid(ui_bid), .s_bresp(ui_bresp), .s_bvalid(ui_bvalid),
        .s_bready(ui_bready),
        .s_arid(ui_arid), .s_araddr(ui_araddr), .s_arlen(ui_arlen),
        .s_arsize(ui_arsize), .s_arburst(ui_arburst),
        .s_arvalid(ui_arvalid), .s_arready(ui_arready),
        .s_rid(ui_rid), .s_rdata(ui_rdata), .s_rresp(ui_rresp),
        .s_rlast(ui_rlast), .s_rvalid(ui_rvalid), .s_rready(ui_rready),

        .m_awid(mig_awid), .m_awaddr(mig_awaddr), .m_awlen(mig_awlen),
        .m_awsize(mig_awsize), .m_awburst(mig_awburst),
        .m_awvalid(mig_awvalid), .m_awready(mig_awready),
        .m_wdata(mig_wdata), .m_wstrb(mig_wstrb), .m_wlast(mig_wlast),
        .m_wvalid(mig_wvalid), .m_wready(mig_wready),
        .m_bid(mig_bid), .m_bresp(mig_bresp), .m_bvalid(mig_bvalid),
        .m_bready(mig_bready),
        .m_arid(mig_arid), .m_araddr(mig_araddr), .m_arlen(mig_arlen),
        .m_arsize(mig_arsize), .m_arburst(mig_arburst),
        .m_arvalid(mig_arvalid), .m_arready(mig_arready),
        .m_rid(mig_rid), .m_rdata(mig_rdata), .m_rresp(mig_rresp),
        .m_rlast(mig_rlast), .m_rvalid(mig_rvalid), .m_rready(mig_rready)
    );

    sim_mig_backend #(.BEATS_LOG2(20), .STALL_ENABLE(1), .STALL_SEED(32'hFACE_B00C)) u_mig_sim (
        .clk(mig_clk), .rst(mig_rst),
        .cal_done(cal_done),
        .awid(mig_awid), .awaddr(mig_awaddr), .awlen(mig_awlen),
        .awsize(mig_awsize), .awburst(mig_awburst),
        .awvalid(mig_awvalid), .awready(mig_awready),
        .wdata(mig_wdata), .wstrb(mig_wstrb), .wlast(mig_wlast),
        .wvalid(mig_wvalid), .wready(mig_wready),
        .bid(mig_bid), .bresp(mig_bresp), .bvalid(mig_bvalid), .bready(mig_bready),
        .arid(mig_arid), .araddr(mig_araddr), .arlen(mig_arlen),
        .arsize(mig_arsize), .arburst(mig_arburst),
        .arvalid(mig_arvalid), .arready(mig_arready),
        .rid(mig_rid), .rdata(mig_rdata), .rresp(mig_rresp),
        .rlast(mig_rlast), .rvalid(mig_rvalid), .rready(mig_rready)
    );

endmodule

`default_nettype wire
