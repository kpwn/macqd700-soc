// tb_l2c_wstream.v -- streaming-write measurement harness for the REAL
// boot write path:
//
//   32-bit master -> axi_narrow_to_wide -> [l2c] -> axi_async_bridge ->
//   axi_ddr4_mig_bridge -> sim_mig_backend
//
// This is the chain the 2026-08-19 measurement in commit 2afe6d1 used to
// find that l2c's read-allocate-on-write-miss costs ~4x on streaming
// writes.  It exists so that number can be re-derived on demand rather
// than living only in a commit message: `make tb-l2c-wstream` measures
// cycles per 32-bit word with l2c in the chain, `make tb-l2c-wstream-off`
// measures the same traffic with l2c not elaborated at all.
//
// Everything downstream of the adapter is tb_l2c_chain.v verbatim -- the
// same instance used by tb-l2c-chain / tb-l2c-chain-off -- so this adds a
// front end and nothing else.  WS_L2C_ENABLE forwards to that module's
// CHAIN_L2C_ENABLE.
//
// The narrow side is deliberately single-outstanding: that is what
// axi_narrow_to_wide.v is (one AW latch, cleared on B), and it is exactly
// the shape boot_fsm's RAM pre-zero pass presents.
//
// Verilog-2005, two clock domains, sync active-high resets.

`default_nettype none

module tb_l2c_wstream #(
    parameter WS_L2C_ENABLE = 1,
    parameter DDR_READ_LATENCY_BASE   = 64,
    parameter DDR_READ_LATENCY_JITTER = 0
) (
    input  wire core_clk, core_rst,
    input  wire mig_clk,  mig_rst,

    // ── Narrow (32-bit) AXI slave -- the BFM drives this ──────────────
    input  wire [31:0] n_awaddr,
    input  wire [7:0]  n_awlen,
    input  wire        n_awvalid,
    output wire        n_awready,
    input  wire [31:0] n_wdata,
    input  wire [3:0]  n_wstrb,
    input  wire        n_wlast,
    input  wire        n_wvalid,
    output wire        n_wready,
    output wire [1:0]  n_bresp,
    output wire        n_bvalid,
    input  wire        n_bready,
    input  wire [31:0] n_araddr,
    input  wire [2:0]  n_arsize,
    input  wire [7:0]  n_arlen,
    input  wire        n_arvalid,
    output wire        n_arready,
    output wire [31:0] n_rdata,
    output wire [1:0]  n_rresp,
    output wire        n_rlast,
    output wire        n_rvalid,
    input  wire        n_rready,

    output wire        cal_done,

    // DDR-traffic taps on l2c's own 128-bit master port -- one beat is one
    // 16 B quadrant.  tb_l2c_sctr.cpp counts these to report DDR read/write
    // beats per touched line; tb_l2c_wstream.cpp ignores them.
    output wire        dbg_l2m_ar_fire,
    output wire        dbg_l2m_r_fire,
    output wire        dbg_l2m_aw_fire,
    output wire        dbg_l2m_w_fire,

    // Eviction-pressure taps -- see tb_l2c_chain.v.
    output wire [4:0]  dbg_vb_occ,
    output wire [4:0]  dbg_vb_slots,
    output wire        dbg_evict_stall,
    output wire        dbg_vb_drain_busy,
    output wire        dbg_vb_seq_busy,
    output wire [4:0]  dbg_vb_out
);

    wire [5:0]   w_awid;   wire [31:0]  w_awaddr;  wire [7:0] w_awlen;
    wire [2:0]   w_awsize; wire [1:0]   w_awburst; wire w_awvalid, w_awready;
    wire [127:0] w_wdata;  wire [15:0]  w_wstrb;   wire w_wlast, w_wvalid, w_wready;
    wire [5:0]   w_bid;    wire [1:0]   w_bresp;   wire w_bvalid, w_bready;
    wire [5:0]   w_arid;   wire [31:0]  w_araddr;  wire [7:0] w_arlen;
    wire [2:0]   w_arsize; wire [1:0]   w_arburst; wire w_arvalid, w_arready;
    wire [5:0]   w_rid;    wire [127:0] w_rdata;   wire [1:0] w_rresp;
    wire w_rlast, w_rvalid, w_rready;

    axi_narrow_to_wide #(
        .ID_WIDTH(6), .ID_TAG(6'd0),
        // Shrunk from the shipping 20 s so a wedge shows up as a tb
        // failure in seconds rather than as an apparently-hung run.
        .TIMEOUT_CYCLES(32'd200000)
    ) u_n2w (
        .clk(core_clk), .rst(core_rst),
        .n_awaddr(n_awaddr), .n_awprot(3'b000), .n_awlen(n_awlen),
        .n_awvalid(n_awvalid), .n_awready(n_awready),
        .n_wdata(n_wdata), .n_wstrb(n_wstrb), .n_wlast(n_wlast),
        .n_wvalid(n_wvalid), .n_wready(n_wready),
        .n_bresp(n_bresp), .n_bvalid(n_bvalid), .n_bready(n_bready),
        .n_araddr(n_araddr), .n_arprot(3'b000), .n_arsize(n_arsize),
        .n_arlen(n_arlen), .n_arvalid(n_arvalid), .n_arready(n_arready),
        .n_rdata(n_rdata), .n_rresp(n_rresp), .n_rlast(n_rlast),
        .n_rvalid(n_rvalid), .n_rready(n_rready),

        .w_awid(w_awid), .w_awaddr(w_awaddr), .w_awlen(w_awlen),
        .w_awsize(w_awsize), .w_awburst(w_awburst),
        .w_awvalid(w_awvalid), .w_awready(w_awready),
        .w_wdata(w_wdata), .w_wstrb(w_wstrb), .w_wlast(w_wlast),
        .w_wvalid(w_wvalid), .w_wready(w_wready),
        .w_bid(w_bid), .w_bresp(w_bresp), .w_bvalid(w_bvalid), .w_bready(w_bready),
        .w_arid(w_arid), .w_araddr(w_araddr), .w_arlen(w_arlen),
        .w_arsize(w_arsize), .w_arburst(w_arburst),
        .w_arvalid(w_arvalid), .w_arready(w_arready),
        .w_rid(w_rid), .w_rdata(w_rdata), .w_rresp(w_rresp),
        .w_rlast(w_rlast), .w_rvalid(w_rvalid), .w_rready(w_rready)
    );

    /* verilator lint_off PINCONNECTEMPTY */
    tb_l2c_chain #(
        .CHAIN_L2C_ENABLE(WS_L2C_ENABLE),
        .L2_BYPASS_ALL(0),
        .STALL_ENABLE(0),
        .DDR_READ_LATENCY_BASE(DDR_READ_LATENCY_BASE),
        .DDR_READ_LATENCY_JITTER(DDR_READ_LATENCY_JITTER)
    ) u_chain (
        .core_clk(core_clk), .core_rst(core_rst),
        .mig_clk(mig_clk),   .mig_rst(mig_rst),
        .s_axi_awid(w_awid), .s_axi_awaddr(w_awaddr), .s_axi_awlen(w_awlen),
        .s_axi_awsize(w_awsize), .s_axi_awburst(w_awburst),
        .s_axi_awvalid(w_awvalid), .s_axi_awready(w_awready),
        .s_axi_wdata(w_wdata), .s_axi_wstrb(w_wstrb), .s_axi_wlast(w_wlast),
        .s_axi_wvalid(w_wvalid), .s_axi_wready(w_wready),
        .s_axi_bid(w_bid), .s_axi_bresp(w_bresp), .s_axi_bvalid(w_bvalid),
        .s_axi_bready(w_bready),
        .s_axi_arid(w_arid), .s_axi_araddr(w_araddr), .s_axi_arlen(w_arlen),
        .s_axi_arsize(w_arsize), .s_axi_arburst(w_arburst),
        .s_axi_arvalid(w_arvalid), .s_axi_arready(w_arready),
        .s_axi_rid(w_rid), .s_axi_rdata(w_rdata), .s_axi_rresp(w_rresp),
        .s_axi_rlast(w_rlast), .s_axi_rvalid(w_rvalid), .s_axi_rready(w_rready),
        .cal_done(cal_done),
        .dbg_mig_read_q_count(), .dbg_mig_ar_fire(), .dbg_mig_r_fire(),
        .dbg_l2m_ar_fire(dbg_l2m_ar_fire), .dbg_l2m_r_fire(dbg_l2m_r_fire),
        .dbg_l2m_aw_fire(dbg_l2m_aw_fire), .dbg_l2m_w_fire(dbg_l2m_w_fire),
        .dbg_vb_occ(dbg_vb_occ), .dbg_vb_slots(dbg_vb_slots),
        .dbg_evict_stall(dbg_evict_stall),
        .dbg_vb_drain_busy(dbg_vb_drain_busy), .dbg_vb_seq_busy(dbg_vb_seq_busy),
        .dbg_vb_out(dbg_vb_out)
    );
    /* verilator lint_on PINCONNECTEMPTY */

endmodule

`default_nettype wire
