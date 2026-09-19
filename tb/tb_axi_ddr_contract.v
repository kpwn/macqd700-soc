// tb_axi_ddr_contract.v -- 32-bit AXI to DDR SIM_MODEL contract wrapper.
//
// Covers the first-light memory path below fpga_top: a 32-bit AXI master
// feeding axi_narrow_to_wide, then ddr_ctrl's 128-bit SIM_MODEL slave.
// This stays clear of core, SD, HDMI, and peripheral models while proving
// the lane, strobe, and alignment contract that those blocks rely on.

`default_nettype none

module tb_axi_ddr_contract (
    input  wire        clk,
    input  wire        rst,

    output wire        ddr_cal_done,

    input  wire [31:0] n_awaddr,
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
    // Read transfer size, mirroring n_wstrb on the write side (see
    // axi_narrow_to_wide.v's "Sub-word load support" header note): 0=1 B,
    // 1=2 B, 2=4 B.  Exposed to the C++ harness so it can cover the
    // byte-granular-araddr contract that side-effecting peripheral
    // registers depend on; the harness defaults it to 2 (4 B), which is
    // the shape this tb exercised before the port existed.
    input  wire [2:0]  n_arsize,
    input  wire        n_arvalid,
    output wire        n_arready,
    output wire [31:0] n_rdata,
    output wire [1:0]  n_rresp,
    output wire        n_rlast,
    output wire        n_rvalid,
    input  wire        n_rready,

    // ── Wide-side direct-drive override ─────────────────────────────
    // When `wide_override` is asserted, the ddr_ctrl AXI inputs are
    // sourced from the dx_* pins instead of the narrow adapter.  Lets
    // the C++ harness inject deliberate AXI protocol violations
    // (misaligned awaddr with a large awsize) to verify that
    // ddr_ctrl's align_error check still fires.
    input  wire        wide_override,
    input  wire [31:0] dx_awaddr,
    input  wire [2:0]  dx_awsize,
    input  wire        dx_awvalid,
    input  wire [127:0] dx_wdata,
    input  wire [15:0] dx_wstrb,
    input  wire        dx_wlast,
    input  wire        dx_wvalid,
    output wire        dx_awready,
    output wire        dx_wready,
    input  wire        dx_bready,
    output wire        dx_bvalid,
    output wire [1:0]  dx_bresp,
    input  wire [31:0] dx_araddr,
    input  wire [2:0]  dx_arsize,
    input  wire        dx_arvalid,
    output wire        dx_arready,
    input  wire        dx_rready,
    output wire        dx_rvalid,
    output wire [1:0]  dx_rresp,
    output wire [127:0] dx_rdata,

    output wire [31:0] dbg_aw_cnt,
    output wire [31:0] dbg_w_cnt,
    output wire [31:0] dbg_b_cnt,
    output wire [31:0] dbg_ar_cnt,
    output wire [31:0] dbg_r_cnt,

    // Wide-side AR shape as the adapter actually presented it, latched on
    // the last wide AR handshake.  Lets the harness prove that n_arsize is
    // carried through to w_arsize/w_araddr rather than force-aligned — the
    // contract byte-addressed, side-effecting peripheral registers depend
    // on (see axi_narrow_to_wide.v "Sub-word load support").  Without an
    // observation point here the pin's value is behaviourally invisible on
    // plain DRAM, so a mis-wired n_arsize would pass silently.
    output wire [31:0] dbg_wide_araddr,
    output wire [2:0]  dbg_wide_arsize,
    output wire [7:0]  dbg_wide_arlen
);

// Adapter-side wide signals (driven by u_n2w).
wire [5:0]   a_awid;
wire [31:0]  a_awaddr;
wire [7:0]   a_awlen;
wire [2:0]   a_awsize;
wire [1:0]   a_awburst;
wire         a_awvalid;
wire [127:0] a_wdata;
wire [15:0]  a_wstrb;
wire         a_wlast;
wire         a_wvalid;
wire         a_bready;
wire [5:0]   a_arid;
wire [31:0]  a_araddr;
wire [7:0]   a_arlen;
wire [2:0]   a_arsize;
wire [1:0]   a_arburst;
wire         a_arvalid;
wire         a_rready;
wire         ddr_clk_unused;

axi_narrow_to_wide #(
    .ID_WIDTH(6),
    .ID_TAG  (6'd18)
) u_n2w (
    .clk(clk),
    .rst(rst),

    .n_awlen  (8'd0),
    .n_arlen  (8'd0),
    .n_awaddr (n_awaddr),
    .n_awprot (3'b000),
    .n_awvalid(n_awvalid),
    .n_awready(n_awready),
    .n_wdata  (n_wdata),
    .n_wstrb  (n_wstrb),
    .n_wlast  (n_wlast),
    .n_wvalid (n_wvalid),
    .n_wready (n_wready),
    .n_bresp  (n_bresp),
    .n_bvalid (n_bvalid),
    .n_bready (n_bready),
    .n_araddr (n_araddr),
    .n_arprot (3'b000),
    .n_arsize (n_arsize),
    .n_arvalid(n_arvalid),
    .n_arready(n_arready),
    .n_rdata  (n_rdata),
    .n_rresp  (n_rresp),
    .n_rlast  (n_rlast),
    .n_rvalid (n_rvalid),
    .n_rready (n_rready),

    .w_awid   (a_awid),
    .w_awaddr (a_awaddr),
    .w_awlen  (a_awlen),
    .w_awsize (a_awsize),
    .w_awburst(a_awburst),
    .w_awvalid(a_awvalid),
    .w_awready(a_awready),
    .w_wdata  (a_wdata),
    .w_wstrb  (a_wstrb),
    .w_wlast  (a_wlast),
    .w_wvalid (a_wvalid),
    .w_wready (a_wready),
    .w_bid    (a_bid),
    .w_bresp  (a_bresp),
    .w_bvalid (a_bvalid),
    .w_bready (a_bready),
    .w_arid   (a_arid),
    .w_araddr (a_araddr),
    .w_arlen  (a_arlen),
    .w_arsize (a_arsize),
    .w_arburst(a_arburst),
    .w_arvalid(a_arvalid),
    .w_arready(a_arready),
    .w_rid    (a_rid),
    .w_rdata  (a_rdata),
    .w_rresp  (a_rresp),
    .w_rlast  (a_rlast),
    .w_rvalid (a_rvalid),
    .w_rready (a_rready)
);

// Slave-response fan-out: the adapter sees slave responses only
// when not overridden.  dx_* pins see them only when overridden.
wire         a_awready;
wire         a_wready;
wire [5:0]   a_bid;
wire [1:0]   a_bresp;
wire         a_bvalid;
wire         a_arready;
wire [5:0]   a_rid;
wire [127:0] a_rdata;
wire [1:0]   a_rresp;
wire         a_rlast;
wire         a_rvalid;

// ── Wide-side mux: narrow adapter or direct drive ──────────────────
// When `wide_override` is asserted, the ddr_ctrl AXI inputs come from
// the dx_* pins so the harness can inject raw (possibly illegal) AXI
// transactions.  When deasserted, the adapter drives the slave.
wire [31:0]  ddr_awaddr  = wide_override ? dx_awaddr  : a_awaddr;
wire [2:0]   ddr_awsize  = wide_override ? dx_awsize  : a_awsize;
wire         ddr_awvalid = wide_override ? dx_awvalid : a_awvalid;
wire [127:0] ddr_wdata   = wide_override ? dx_wdata   : a_wdata;
wire [15:0]  ddr_wstrb   = wide_override ? dx_wstrb   : a_wstrb;
wire         ddr_wlast   = wide_override ? dx_wlast   : a_wlast;
wire         ddr_wvalid  = wide_override ? dx_wvalid  : a_wvalid;
wire         ddr_bready  = wide_override ? dx_bready  : a_bready;
wire [31:0]  ddr_araddr  = wide_override ? dx_araddr  : a_araddr;
wire [2:0]   ddr_arsize  = wide_override ? dx_arsize  : a_arsize;
wire         ddr_arvalid = wide_override ? dx_arvalid : a_arvalid;
wire         ddr_rready  = wide_override ? dx_rready  : a_rready;

// Latch the wide-side AR shape the adapter presented, so the harness can
// assert on it directly (see dbg_wide_ar* in the port list).
reg [31:0] wide_araddr_q;
reg [2:0]  wide_arsize_q;
reg [7:0]  wide_arlen_q;
always @(posedge clk) begin
    if (rst) begin
        wide_araddr_q <= 32'd0;
        wide_arsize_q <= 3'd0;
        wide_arlen_q  <= 8'd0;
    end else if (a_arvalid && a_arready) begin
        wide_araddr_q <= a_araddr;
        wide_arsize_q <= a_arsize;
        wide_arlen_q  <= a_arlen;
    end
end
assign dbg_wide_araddr = wide_araddr_q;
assign dbg_wide_arsize = wide_arsize_q;
assign dbg_wide_arlen  = wide_arlen_q;

wire         ddr_awready;
wire         ddr_wready;
wire         ddr_bvalid;
wire [1:0]   ddr_bresp;
wire         ddr_arready;
wire         ddr_rvalid;
wire [1:0]   ddr_rresp;
wire [127:0] ddr_rdata;
wire         ddr_rlast;

// Adapter sees slave responses only when not overridden.
assign a_awready  = wide_override ? 1'b0 : ddr_awready;
assign a_wready   = wide_override ? 1'b0 : ddr_wready;
assign a_bvalid   = wide_override ? 1'b0 : ddr_bvalid;
assign a_bresp    = ddr_bresp;
assign a_bid      = 6'd0;
assign a_arready  = wide_override ? 1'b0 : ddr_arready;
assign a_rvalid   = wide_override ? 1'b0 : ddr_rvalid;
assign a_rresp    = ddr_rresp;
assign a_rdata    = ddr_rdata;
assign a_rlast    = ddr_rlast;
assign a_rid      = 6'd0;

// dx_* channels see slave responses only when overridden.
assign dx_awready = wide_override ? ddr_awready : 1'b0;
assign dx_wready  = wide_override ? ddr_wready  : 1'b0;
assign dx_bvalid  = wide_override ? ddr_bvalid  : 1'b0;
assign dx_bresp   = ddr_bresp;
assign dx_arready = wide_override ? ddr_arready : 1'b0;
assign dx_rvalid  = wide_override ? ddr_rvalid  : 1'b0;
assign dx_rresp   = ddr_rresp;
assign dx_rdata   = ddr_rdata;

ddr_ctrl #(
    .DATA_WIDTH     (128),
    .XID_WIDTH      (6),
    .BRAM_LOG2_BEATS(10),
    .SIM_CAL_CYCLES (8)
) u_ddr (
    .clk(clk),
    .rst(rst),
    .ddr_clk(ddr_clk_unused),
    .ddr_cal_done(ddr_cal_done),

    .awid   (6'd0),
    .awaddr (ddr_awaddr),
    .awlen  (8'd0),
    .awsize (ddr_awsize),
    .awburst(2'b01),
    .awvalid(ddr_awvalid),
    .awready(ddr_awready),
    .wdata  (ddr_wdata),
    .wstrb  (ddr_wstrb),
    .wlast  (ddr_wlast),
    .wvalid (ddr_wvalid),
    .wready (ddr_wready),
    .bid    (),
    .bresp  (ddr_bresp),
    .bvalid (ddr_bvalid),
    .bready (ddr_bready),
    .arid   (6'd0),
    .araddr (ddr_araddr),
    .arlen  (8'd0),
    .arsize (ddr_arsize),
    .arburst(2'b01),
    .arvalid(ddr_arvalid),
    .arready(ddr_arready),
    .rid    (),
    .rdata  (ddr_rdata),
    .rresp  (ddr_rresp),
    .rlast  (ddr_rlast),
    .rvalid (ddr_rvalid),
    .rready (ddr_rready),

    .dbg_aw_cnt(dbg_aw_cnt),
    .dbg_w_cnt (dbg_w_cnt),
    .dbg_b_cnt (dbg_b_cnt),
    .dbg_ar_cnt(dbg_ar_cnt),
    .dbg_r_cnt (dbg_r_cnt)
);

endmodule

`default_nettype wire
