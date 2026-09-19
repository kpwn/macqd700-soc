// tb_l2c_chain.v -- Verilator-top wrapper instantiating the REAL T13
// integration chain: l2c -> axi_async_bridge -> axi_ddr4_mig_bridge ->
// sim_mig_backend, across two clock domains (core_clk, mig_clk).
//
// This mirrors rtl/soc/fpga_top_ddr.vh's `L2C_ENABLE` wiring exactly:
//   xbar S0 (core_clk) -> l2c (core_clk) -> [u_ddr's internal
//   axi_async_bridge CDC + axi_ddr4_mig_bridge contract shim, both
//   reused verbatim from rtl/board/, not reimplemented here] -> MIG.
// In sim, sim_mig_backend.v stands in for the MIG (the same substitution
// ddr_ctrl.v's own SIM_MIG_BRIDGE path uses -- see rtl/board/ddr_ctrl.v).
//
// `CHAIN_L2C_ENABLE` (default 1) selects whether l2c sits in the chain
// at all:
//   1 -- real chain: s_axi_* (BFM) -> l2c -> axi_async_bridge -> ...
//   0 -- l2c entirely absent from elaboration (not merely bypassed --
//        mirrors fpga_top_ddr.vh's `L2C_ENABLE`-undefined path exactly):
//        s_axi_* (BFM) -> axi_async_bridge -> ... directly.
// Used for the "L2C_ENABLE=off equivalence smoke" scenario (Makefile
// tb-l2c-chain-off): proves the harness/chain wiring itself is correct
// by running the identical BFM traffic through the identical downstream
// stack with l2c only removed, and comparing pass/fail + final DDR
// contents against the CHAIN_L2C_ENABLE=1 run.
//
// `L2_BYPASS_ALL` (only meaningful when CHAIN_L2C_ENABLE=1) forwards to
// l2c's own bring-up escape hatch (docs/l2c_spec.md S8) -- kept at 0 for
// the real-cache scenarios per the T13 brief ("L2_BYPASS_ALL=0 in the
// integration tb -- we want the real cache exercised in sim").
//
// `STALL_ENABLE`/`STALL_SEED` forward to sim_mig_backend's command-side
// backpressure injection (see rtl/board/sim_mig_backend.v header).
//
// Verilog-2005, two independent clk domains, sync active-high resets in
// each domain (core_rst resets core_clk-domain state + the async
// bridge's s-side; mig_rst resets mig_clk-domain state + the async
// bridge's m-side + the mig bridge + the sim backend -- matching
// axi_async_bridge.v's own s_rst/m_rst contract).

`default_nettype none

module tb_l2c_chain #(
    parameter CHAIN_L2C_ENABLE = 1,
    parameter L2_BYPASS_ALL    = 0,
    parameter STALL_ENABLE     = 0,
    parameter [31:0] STALL_SEED = 32'hACE1_1234,
    parameter DDR_READ_LATENCY_BASE = 200,
    parameter DDR_READ_LATENCY_JITTER = 64
) (
    input  wire core_clk, core_rst,
    input  wire mig_clk,  mig_rst,

    // AXI4 slave -- BFM drives this (same shape as l2c.v's s_axi_* / xbar
    // S0: 128b data, 6b ID, 32b addr).
    input  wire [5:0]   s_axi_awid, input wire [31:0] s_axi_awaddr,
    input  wire [7:0]   s_axi_awlen, input wire [2:0] s_axi_awsize, input wire [1:0] s_axi_awburst,
    input  wire s_axi_awvalid, output wire s_axi_awready,
    input  wire [127:0] s_axi_wdata, input wire [15:0] s_axi_wstrb,
    input  wire s_axi_wlast, input wire s_axi_wvalid, output wire s_axi_wready,
    output wire [5:0] s_axi_bid, output wire [1:0] s_axi_bresp,
    output wire s_axi_bvalid, input wire s_axi_bready,
    input  wire [5:0] s_axi_arid, input wire [31:0] s_axi_araddr,
    input  wire [7:0] s_axi_arlen, input wire [2:0] s_axi_arsize, input wire [1:0] s_axi_arburst,
    input  wire s_axi_arvalid, output wire s_axi_arready,
    output wire [5:0] s_axi_rid, output wire [127:0] s_axi_rdata,
    output wire [1:0] s_axi_rresp, output wire s_axi_rlast, output wire s_axi_rvalid, input wire s_axi_rready,

    output wire cal_done,
    output wire [3:0] dbg_mig_read_q_count,
    output wire       dbg_mig_ar_fire,
    output wire       dbg_mig_r_fire,
    // DDR-traffic taps on L2C's OWN master port (`c_*`, 128 b = one 16 B
    // quadrant per beat).  Measuring here rather than at the MIG side is
    // deliberate: mig_* is 256 b wide, so a 16 B and a 32 B writeback are
    // indistinguishable there, and 16 B is exactly the granularity a
    // sectored dirty/valid scheme changes.  With CHAIN_L2C_ENABLE=0 these
    // mirror the slave port, which is the correct "no cache" reference.
    output wire       dbg_l2m_ar_fire,
    output wire       dbg_l2m_r_fire,
    output wire       dbg_l2m_aw_fire,
    output wire       dbg_l2m_w_fire,
    // Eviction-pressure taps (sim-only hierarchical references into l2c).
    // dbg_vb_occ        -- victim-buffer slot occupancy, 0..SLOTS.
    // dbg_vb_slots      -- l2c_victim's SLOTS parameter.
    // dbg_evict_stall   -- a miss sitting in S_LOOKUP that would dispatch
    //                      RIGHT NOW if the victim buffer had a free slot.
    // dbg_vb_drain_busy -- at least one writeback is in flight or pending
    //                      (the pipelined successor to "the FSM is not
    //                      idle", which stopped meaning anything once the
    //                      sequencer handed work off without waiting on B).
    // dbg_vb_out        -- writebacks with an AW accepted and no B yet.
    // Together these say WHY the buffer is full: cleaned-too-late (stall
    // high, drain idle) or drain-throughput-bound (both high).
    output wire [4:0] dbg_vb_occ,
    output wire [4:0] dbg_vb_slots,
    output wire       dbg_evict_stall,
    // dbg_vb_seq_busy   -- the AW/W sequencer has something to send, i.e.
    //                      the master write port's actual occupancy.
    output wire       dbg_vb_drain_busy,
    output wire       dbg_vb_seq_busy,
    output wire [4:0] dbg_vb_out
);

    // -- s-side of the CDC: either l2c's m_axi (CHAIN_L2C_ENABLE=1) or a
    //    direct pass-through of the top-level s_axi_* (CHAIN_L2C_ENABLE=0).
    wire [5:0]   c_awid;   wire [31:0]  c_awaddr;  wire [7:0] c_awlen;
    wire [2:0]   c_awsize; wire [1:0]   c_awburst; wire c_awvalid, c_awready;
    wire [127:0] c_wdata;  wire [15:0]  c_wstrb;   wire c_wlast, c_wvalid, c_wready;
    wire [5:0]   c_bid;    wire [1:0]   c_bresp;   wire c_bvalid, c_bready;
    wire [5:0]   c_arid;   wire [31:0]  c_araddr;  wire [7:0] c_arlen;
    wire [2:0]   c_arsize; wire [1:0]   c_arburst; wire c_arvalid, c_arready;
    wire [5:0]   c_rid;    wire [127:0] c_rdata;   wire [1:0] c_rresp;
    wire c_rlast, c_rvalid, c_rready;

    generate
    if (CHAIN_L2C_ENABLE != 0) begin : g_l2c
        l2c #(
            .ADDR_WIDTH(32), .DATA_WIDTH(128), .ID_WIDTH(6),
            .L2_BYPASS_ALL(L2_BYPASS_ALL),
            .EXTERNAL_WRITE_RESET_RECOVERY(1),
            .NUM_BYPASS_WINDOWS(1),
            .BYP_WIN_BASE(32'h0000_0000), .BYP_WIN_MASK(32'h0000_0000), .BYP_WIN_EN(1'b0),
            .CACHEABLE_BASE(32'h0000_0000), .CACHEABLE_SIZE(32'h4100_0000)
        ) u_l2c (
            .clk(core_clk), .rst(core_rst),
            .s_axi_awid(s_axi_awid), .s_axi_awaddr(s_axi_awaddr), .s_axi_awlen(s_axi_awlen),
            .s_axi_awsize(s_axi_awsize), .s_axi_awburst(s_axi_awburst), .s_axi_awvalid(s_axi_awvalid),
            .s_axi_awready(s_axi_awready),
            .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb), .s_axi_wlast(s_axi_wlast),
            .s_axi_wvalid(s_axi_wvalid), .s_axi_wready(s_axi_wready),
            .s_axi_bid(s_axi_bid), .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid), .s_axi_bready(s_axi_bready),
            .s_axi_arid(s_axi_arid), .s_axi_araddr(s_axi_araddr), .s_axi_arlen(s_axi_arlen),
            .s_axi_arsize(s_axi_arsize), .s_axi_arburst(s_axi_arburst), .s_axi_arvalid(s_axi_arvalid),
            .s_axi_arready(s_axi_arready),
            .s_axi_rid(s_axi_rid), .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp),
            .s_axi_rlast(s_axi_rlast), .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready),
            .m_axi_awid(c_awid), .m_axi_awaddr(c_awaddr), .m_axi_awlen(c_awlen),
            .m_axi_awsize(c_awsize), .m_axi_awburst(c_awburst), .m_axi_awvalid(c_awvalid),
            .m_axi_awready(c_awready),
            .m_axi_wdata(c_wdata), .m_axi_wstrb(c_wstrb), .m_axi_wlast(c_wlast),
            .m_axi_wvalid(c_wvalid), .m_axi_wready(c_wready),
            .m_axi_bid(c_bid), .m_axi_bresp(c_bresp), .m_axi_bvalid(c_bvalid), .m_axi_bready(c_bready),
            .m_axi_arid(c_arid), .m_axi_araddr(c_araddr), .m_axi_arlen(c_arlen),
            .m_axi_arsize(c_arsize), .m_axi_arburst(c_arburst), .m_axi_arvalid(c_arvalid),
            .m_axi_arready(c_arready),
            .m_axi_rid(c_rid), .m_axi_rdata(c_rdata), .m_axi_rresp(c_rresp),
            .m_axi_rlast(c_rlast), .m_axi_rvalid(c_rvalid), .m_axi_rready(c_rready)
        );
        // Hierarchical references into the DUT -- legal Verilog-2005 and
        // testbench-only; nothing in this file is ever synthesised.
        assign dbg_vb_occ   = u_l2c.g_active.u_victim.used_c;
        assign dbg_vb_out   = u_l2c.g_active.u_victim.out_cnt;
        assign dbg_vb_slots = u_l2c.g_active.u_victim.slots_c[4:0];
        assign dbg_evict_stall = u_l2c.g_active.u_ctrl.miss_no_hit_c &&
                                 !u_l2c.g_active.u_ctrl.mshr_lu_hit &&
                                 u_l2c.g_active.u_ctrl.need_evict_c &&
                                 u_l2c.g_active.u_ctrl.victim_sel_ok &&
                                 !u_l2c.g_active.u_ctrl.victim_push_ready;
        assign dbg_vb_drain_busy = u_l2c.g_active.u_victim.drain_busy_c;
        assign dbg_vb_seq_busy   = u_l2c.g_active.u_victim.seq_busy_c;
    end else begin : g_no_l2c
        assign dbg_vb_occ        = 5'd0;
        assign dbg_vb_out        = 5'd0;
        assign dbg_vb_slots      = 5'd0;
        assign dbg_evict_stall   = 1'b0;
        assign dbg_vb_drain_busy = 1'b0;
        assign dbg_vb_seq_busy   = 1'b0;
        // CHAIN_L2C_ENABLE=0: l2c is not instantiated at all -- mirrors
        // fpga_top_ddr.vh's `L2C_ENABLE`-undefined path (xbar S0 wired
        // straight to u_ddr / the async bridge).
        assign c_awid = s_axi_awid; assign c_awaddr = s_axi_awaddr; assign c_awlen = s_axi_awlen;
        assign c_awsize = s_axi_awsize; assign c_awburst = s_axi_awburst; assign c_awvalid = s_axi_awvalid;
        assign s_axi_awready = c_awready;
        assign c_wdata = s_axi_wdata; assign c_wstrb = s_axi_wstrb; assign c_wlast = s_axi_wlast;
        assign c_wvalid = s_axi_wvalid;
        assign s_axi_wready = c_wready;
        assign s_axi_bid = c_bid; assign s_axi_bresp = c_bresp; assign s_axi_bvalid = c_bvalid;
        assign c_bready = s_axi_bready;
        assign c_arid = s_axi_arid; assign c_araddr = s_axi_araddr; assign c_arlen = s_axi_arlen;
        assign c_arsize = s_axi_arsize; assign c_arburst = s_axi_arburst; assign c_arvalid = s_axi_arvalid;
        assign s_axi_arready = c_arready;
        assign s_axi_rid = c_rid; assign s_axi_rdata = c_rdata; assign s_axi_rresp = c_rresp;
        assign s_axi_rlast = c_rlast; assign s_axi_rvalid = c_rvalid;
        assign c_rready = s_axi_rready;
    end
    endgenerate

    // -- CDC: core_clk -> mig_clk, same instance shape as
    //    rtl/board/ddr_ctrl.v's real-hw u_core_to_mig_ui. --
    wire [5:0]   ui_awid;   wire [31:0]  ui_awaddr;  wire [7:0] ui_awlen;
    wire [2:0]   ui_awsize; wire [1:0]   ui_awburst; wire ui_awvalid, ui_awready;
    wire [127:0] ui_wdata;  wire [15:0]  ui_wstrb;   wire ui_wlast, ui_wvalid, ui_wready;
    wire [5:0]   ui_bid;    wire [1:0]   ui_bresp;   wire ui_bvalid, ui_bready;
    wire [5:0]   ui_arid;   wire [31:0]  ui_araddr;  wire [7:0] ui_arlen;
    wire [2:0]   ui_arsize; wire [1:0]   ui_arburst; wire ui_arvalid, ui_arready;
    wire [5:0]   ui_rid;    wire [127:0] ui_rdata;   wire [1:0] ui_rresp;
    wire ui_rlast, ui_rvalid, ui_rready;

    axi_async_bridge #(
        .DATA_WIDTH(128), .ADDR_WIDTH(32), .ID_WIDTH(6), .USER_WIDTH(1)
    ) u_core_to_mig_ui (
        .s_clk(core_clk), .s_rst(core_rst),
        .s_awid(c_awid), .s_awaddr(c_awaddr), .s_awlen(c_awlen),
        .s_awsize(c_awsize), .s_awburst(c_awburst), .s_awlock(1'b0),
        .s_awcache(4'b0011), .s_awprot(3'b000), .s_awqos(4'b0000),
        .s_awuser(1'b0), .s_awvalid(c_awvalid), .s_awready(c_awready),
        .s_wdata(c_wdata), .s_wstrb(c_wstrb), .s_wlast(c_wlast),
        .s_wuser(1'b0), .s_wvalid(c_wvalid), .s_wready(c_wready),
        .s_bid(c_bid), .s_bresp(c_bresp), .s_buser(), .s_bvalid(c_bvalid),
        .s_bready(c_bready),
        .s_arid(c_arid), .s_araddr(c_araddr), .s_arlen(c_arlen),
        .s_arsize(c_arsize), .s_arburst(c_arburst), .s_arlock(1'b0),
        .s_arcache(4'b0011), .s_arprot(3'b000), .s_arqos(4'b0000),
        .s_aruser(1'b0), .s_arvalid(c_arvalid), .s_arready(c_arready),
        .s_rid(c_rid), .s_rdata(c_rdata), .s_rresp(c_rresp), .s_rlast(c_rlast),
        .s_ruser(), .s_rvalid(c_rvalid), .s_rready(c_rready),

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

    // -- repo DDR contract -> pcie_test MIG contract shim, same instance
    //    shape as rtl/board/ddr_ctrl.v's real-hw u_repo_to_pcie_mig. --
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

    // -- behavioural MIG stand-in (sim only), same as ddr_ctrl.v's
    //    SIM_MIG_BRIDGE path.  STALL_ENABLE forwarded for the "mig
    //    stalls enabled" scenario in the T13 brief. --
    sim_mig_backend #(
        .BEATS_LOG2(20),  // 2^20 x 32B = 32 MiB -- ample for the tb's working set
        .STALL_ENABLE(STALL_ENABLE),
        .STALL_SEED(STALL_SEED),
        .READ_LATENCY_BASE(DDR_READ_LATENCY_BASE),
        .READ_LATENCY_JITTER(DDR_READ_LATENCY_JITTER)
    ) u_mig_sim (
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

    assign dbg_mig_read_q_count = u_mig_sim.r_q_count;
    assign dbg_mig_ar_fire = mig_arvalid && mig_arready;
    assign dbg_mig_r_fire  = mig_rvalid && mig_rready;
    assign dbg_l2m_ar_fire = c_arvalid && c_arready;
    assign dbg_l2m_r_fire  = c_rvalid  && c_rready;
    assign dbg_l2m_aw_fire = c_awvalid && c_awready;
    assign dbg_l2m_w_fire  = c_wvalid  && c_wready;

endmodule

`default_nettype wire
