// axi_pb_s1_cdc.v — drop-in narrow-payload replacement for the S1
// (peripheral bus) core_clk -> pb_clk AXI CDC bridge.
//
// Port-for-port and parameter-for-parameter compatible with
// `axi_async_bridge` so the instantiation in
// `rtl/soc/fpga_top_peripherals.vh` differs only in the module name
// (selected by the `PB_S1_WIDE_CDC` escape hatch — see that file).
//
// Structure
// ─────────
//   s_* (DATA_WIDTH=128, core_clk)
//        -> axi_pb_lane_narrow      (combinational, 128 -> 32)
//        -> axi_async_bridge #(32)  (the real CDC: 5 gray-pointer FIFOs)
//        -> axi_pb_lane_widen       (combinational, 32 -> 128)
//        -> m_* (DATA_WIDTH=128, pb_clk)
//
// The whole point is the middle instance: its W and R channel FIFOs are
// `W_DEPTH_LOG2`/`R_DEPTH_LOG2` deep (16 entries each at the S1
// settings), so carrying 32 payload bits instead of 128 removes ~73% of
// this bridge's FIFO storage.  Everything else — the gray-pointer
// crossing, the T6 coupled-reset handshake, the T14 stale-beat sinks,
// the `REQ_MIN_HOLD` bound, the AW/AR/B depths and the multi-entry
// pre-staging that lets a second master's transaction cross while the
// first is being serviced — is the SAME `axi_async_bridge` as before and
// is not re-implemented here.
//
// Reset behaviour is therefore identical: `s_rst` and `m_rst` go
// straight to the inner bridge.  The two shims are purely combinational
// and have no reset of their own, so the deliberate choice to hold this
// bridge on the *core* reset banks (`core_rst_bank[1]` /
// `pb_core_rst_bank[3]`, keeping the host path to `debug_ctrl` alive
// across a JTAG/button full reset) is preserved exactly.
//
// Burst/lane preconditions and the one intentional behavioural
// difference (unselected read lanes are replicated rather than zeroed)
// are documented in `rtl/soc/axi_pb_lane_shim.v`'s header and asserted
// by `make tb-pb-lane-shim`.
//
// Verilog-2005.

`default_nettype none

module axi_pb_s1_cdc #(
    parameter DATA_WIDTH    = 128,
    parameter ADDR_WIDTH    = 32,
    parameter ID_WIDTH      = 6,
    parameter USER_WIDTH    = 1,
    parameter AW_DEPTH_LOG2 = 2,
    parameter AR_DEPTH_LOG2 = 2,
    parameter W_DEPTH_LOG2  = 4,
    parameter R_DEPTH_LOG2  = 4,
    parameter B_DEPTH_LOG2  = 2,
    // Payload width actually carried across the clock domain.
    parameter NARROW_DW     = 32
) (
    input  wire                       s_clk,
    input  wire                       s_rst,
    input  wire [ID_WIDTH-1:0]        s_awid,
    input  wire [ADDR_WIDTH-1:0]      s_awaddr,
    input  wire [7:0]                 s_awlen,
    input  wire [2:0]                 s_awsize,
    input  wire [1:0]                 s_awburst,
    input  wire                       s_awlock,
    input  wire [3:0]                 s_awcache,
    input  wire [2:0]                 s_awprot,
    input  wire [3:0]                 s_awqos,
    input  wire [USER_WIDTH-1:0]      s_awuser,
    input  wire                       s_awvalid,
    output wire                       s_awready,
    input  wire [DATA_WIDTH-1:0]      s_wdata,
    input  wire [(DATA_WIDTH/8)-1:0]  s_wstrb,
    input  wire                       s_wlast,
    input  wire [USER_WIDTH-1:0]      s_wuser,
    input  wire                       s_wvalid,
    output wire                       s_wready,
    output wire [ID_WIDTH-1:0]        s_bid,
    output wire [1:0]                 s_bresp,
    output wire [USER_WIDTH-1:0]      s_buser,
    output wire                       s_bvalid,
    input  wire                       s_bready,
    input  wire [ID_WIDTH-1:0]        s_arid,
    input  wire [ADDR_WIDTH-1:0]      s_araddr,
    input  wire [7:0]                 s_arlen,
    input  wire [2:0]                 s_arsize,
    input  wire [1:0]                 s_arburst,
    input  wire                       s_arlock,
    input  wire [3:0]                 s_arcache,
    input  wire [2:0]                 s_arprot,
    input  wire [3:0]                 s_arqos,
    input  wire [USER_WIDTH-1:0]      s_aruser,
    input  wire                       s_arvalid,
    output wire                       s_arready,
    output wire [ID_WIDTH-1:0]        s_rid,
    output wire [DATA_WIDTH-1:0]      s_rdata,
    output wire [1:0]                 s_rresp,
    output wire                       s_rlast,
    output wire [USER_WIDTH-1:0]      s_ruser,
    output wire                       s_rvalid,
    input  wire                       s_rready,

    input  wire                       m_clk,
    input  wire                       m_rst,
    output wire [ID_WIDTH-1:0]        m_awid,
    output wire [ADDR_WIDTH-1:0]      m_awaddr,
    output wire [7:0]                 m_awlen,
    output wire [2:0]                 m_awsize,
    output wire [1:0]                 m_awburst,
    output wire                       m_awlock,
    output wire [3:0]                 m_awcache,
    output wire [2:0]                 m_awprot,
    output wire [3:0]                 m_awqos,
    output wire [USER_WIDTH-1:0]      m_awuser,
    output wire                       m_awvalid,
    input  wire                       m_awready,
    output wire [DATA_WIDTH-1:0]      m_wdata,
    output wire [(DATA_WIDTH/8)-1:0]  m_wstrb,
    output wire                       m_wlast,
    output wire [USER_WIDTH-1:0]      m_wuser,
    output wire                       m_wvalid,
    input  wire                       m_wready,
    input  wire [ID_WIDTH-1:0]        m_bid,
    input  wire [1:0]                 m_bresp,
    input  wire [USER_WIDTH-1:0]      m_buser,
    input  wire                       m_bvalid,
    output wire                       m_bready,
    output wire [ID_WIDTH-1:0]        m_arid,
    output wire [ADDR_WIDTH-1:0]      m_araddr,
    output wire [7:0]                 m_arlen,
    output wire [2:0]                 m_arsize,
    output wire [1:0]                 m_arburst,
    output wire                       m_arlock,
    output wire [3:0]                 m_arcache,
    output wire [2:0]                 m_arprot,
    output wire [3:0]                 m_arqos,
    output wire [USER_WIDTH-1:0]      m_aruser,
    output wire                       m_arvalid,
    input  wire                       m_arready,
    input  wire [ID_WIDTH-1:0]        m_rid,
    input  wire [DATA_WIDTH-1:0]      m_rdata,
    input  wire [1:0]                 m_rresp,
    input  wire                       m_rlast,
    input  wire [USER_WIDTH-1:0]      m_ruser,
    input  wire                       m_rvalid,
    output wire                       m_rready
);
    // ── core-side: 128 -> 32 ─────────────────────────────────────────
    wire [ID_WIDTH-1:0]      n_awid;
    wire [ADDR_WIDTH-1:0]    n_awaddr;
    wire [7:0]               n_awlen;
    wire [2:0]               n_awsize;
    wire [1:0]               n_awburst;
    wire                     n_awvalid, n_awready;
    wire [NARROW_DW-1:0]     n_wdata;
    wire [NARROW_DW/8-1:0]   n_wstrb;
    wire                     n_wlast, n_wvalid, n_wready;
    wire [ID_WIDTH-1:0]      n_bid;
    wire [1:0]               n_bresp;
    wire                     n_bvalid, n_bready;
    wire [ID_WIDTH-1:0]      n_arid;
    wire [ADDR_WIDTH-1:0]    n_araddr;
    wire [7:0]               n_arlen;
    wire [2:0]               n_arsize;
    wire [1:0]               n_arburst;
    wire                     n_arvalid, n_arready;
    wire [ID_WIDTH-1:0]      n_rid;
    wire [NARROW_DW-1:0]     n_rdata;
    wire [1:0]               n_rresp;
    wire                     n_rlast, n_rvalid, n_rready;

    axi_pb_lane_narrow #(
        .ID_WIDTH(ID_WIDTH), .ADDR_WIDTH(ADDR_WIDTH),
        .WIDE_DW(DATA_WIDTH), .NARROW_DW(NARROW_DW)
    ) u_narrow (
        .s_awid(s_awid), .s_awaddr(s_awaddr), .s_awlen(s_awlen),
        .s_awsize(s_awsize), .s_awburst(s_awburst),
        .s_awvalid(s_awvalid), .s_awready(s_awready),
        .s_wdata(s_wdata), .s_wstrb(s_wstrb), .s_wlast(s_wlast),
        .s_wvalid(s_wvalid), .s_wready(s_wready),
        .s_bid(s_bid), .s_bresp(s_bresp),
        .s_bvalid(s_bvalid), .s_bready(s_bready),
        .s_arid(s_arid), .s_araddr(s_araddr), .s_arlen(s_arlen),
        .s_arsize(s_arsize), .s_arburst(s_arburst),
        .s_arvalid(s_arvalid), .s_arready(s_arready),
        .s_rid(s_rid), .s_rdata(s_rdata), .s_rresp(s_rresp),
        .s_rlast(s_rlast), .s_rvalid(s_rvalid), .s_rready(s_rready),

        .m_awid(n_awid), .m_awaddr(n_awaddr), .m_awlen(n_awlen),
        .m_awsize(n_awsize), .m_awburst(n_awburst),
        .m_awvalid(n_awvalid), .m_awready(n_awready),
        .m_wdata(n_wdata), .m_wstrb(n_wstrb), .m_wlast(n_wlast),
        .m_wvalid(n_wvalid), .m_wready(n_wready),
        .m_bid(n_bid), .m_bresp(n_bresp),
        .m_bvalid(n_bvalid), .m_bready(n_bready),
        .m_arid(n_arid), .m_araddr(n_araddr), .m_arlen(n_arlen),
        .m_arsize(n_arsize), .m_arburst(n_arburst),
        .m_arvalid(n_arvalid), .m_arready(n_arready),
        .m_rid(n_rid), .m_rdata(n_rdata), .m_rresp(n_rresp),
        .m_rlast(n_rlast), .m_rvalid(n_rvalid), .m_rready(n_rready)
    );

    // ── the real CDC, carrying NARROW_DW payload bits ────────────────
    wire [ID_WIDTH-1:0]      p_awid;
    wire [ADDR_WIDTH-1:0]    p_awaddr;
    wire [7:0]               p_awlen;
    wire [2:0]               p_awsize;
    wire [1:0]               p_awburst;
    wire                     p_awvalid, p_awready;
    wire [NARROW_DW-1:0]     p_wdata;
    wire [NARROW_DW/8-1:0]   p_wstrb;
    wire                     p_wlast, p_wvalid, p_wready;
    wire [ID_WIDTH-1:0]      p_bid;
    wire [1:0]               p_bresp;
    wire                     p_bvalid, p_bready;
    wire [ID_WIDTH-1:0]      p_arid;
    wire [ADDR_WIDTH-1:0]    p_araddr;
    wire [7:0]               p_arlen;
    wire [2:0]               p_arsize;
    wire [1:0]               p_arburst;
    wire                     p_arvalid, p_arready;
    wire [ID_WIDTH-1:0]      p_rid;
    wire [NARROW_DW-1:0]     p_rdata;
    wire [1:0]               p_rresp;
    wire                     p_rlast, p_rvalid, p_rready;

    axi_async_bridge #(
        .DATA_WIDTH(NARROW_DW), .ADDR_WIDTH(ADDR_WIDTH),
        .ID_WIDTH(ID_WIDTH), .USER_WIDTH(USER_WIDTH),
        .AW_DEPTH_LOG2(AW_DEPTH_LOG2), .AR_DEPTH_LOG2(AR_DEPTH_LOG2),
        .W_DEPTH_LOG2(W_DEPTH_LOG2), .R_DEPTH_LOG2(R_DEPTH_LOG2),
        .B_DEPTH_LOG2(B_DEPTH_LOG2)
    ) u_bridge (
        .s_clk(s_clk), .s_rst(s_rst),
        .s_awid(n_awid), .s_awaddr(n_awaddr), .s_awlen(n_awlen),
        .s_awsize(n_awsize), .s_awburst(n_awburst),
        .s_awlock(s_awlock), .s_awcache(s_awcache), .s_awprot(s_awprot),
        .s_awqos(s_awqos), .s_awuser(s_awuser),
        .s_awvalid(n_awvalid), .s_awready(n_awready),
        .s_wdata(n_wdata), .s_wstrb(n_wstrb), .s_wlast(n_wlast),
        .s_wuser(s_wuser), .s_wvalid(n_wvalid), .s_wready(n_wready),
        .s_bid(n_bid), .s_bresp(n_bresp), .s_buser(s_buser),
        .s_bvalid(n_bvalid), .s_bready(n_bready),
        .s_arid(n_arid), .s_araddr(n_araddr), .s_arlen(n_arlen),
        .s_arsize(n_arsize), .s_arburst(n_arburst),
        .s_arlock(s_arlock), .s_arcache(s_arcache), .s_arprot(s_arprot),
        .s_arqos(s_arqos), .s_aruser(s_aruser),
        .s_arvalid(n_arvalid), .s_arready(n_arready),
        .s_rid(n_rid), .s_rdata(n_rdata), .s_rresp(n_rresp),
        .s_rlast(n_rlast), .s_ruser(s_ruser),
        .s_rvalid(n_rvalid), .s_rready(n_rready),

        .m_clk(m_clk), .m_rst(m_rst),
        .m_awid(p_awid), .m_awaddr(p_awaddr), .m_awlen(p_awlen),
        .m_awsize(p_awsize), .m_awburst(p_awburst),
        .m_awlock(m_awlock), .m_awcache(m_awcache), .m_awprot(m_awprot),
        .m_awqos(m_awqos), .m_awuser(m_awuser),
        .m_awvalid(p_awvalid), .m_awready(p_awready),
        .m_wdata(p_wdata), .m_wstrb(p_wstrb), .m_wlast(p_wlast),
        .m_wuser(m_wuser), .m_wvalid(p_wvalid), .m_wready(p_wready),
        .m_bid(p_bid), .m_bresp(p_bresp), .m_buser(m_buser),
        .m_bvalid(p_bvalid), .m_bready(p_bready),
        .m_arid(p_arid), .m_araddr(p_araddr), .m_arlen(p_arlen),
        .m_arsize(p_arsize), .m_arburst(p_arburst),
        .m_arlock(m_arlock), .m_arcache(m_arcache), .m_arprot(m_arprot),
        .m_arqos(m_arqos), .m_aruser(m_aruser),
        .m_arvalid(p_arvalid), .m_arready(p_arready),
        .m_rid(p_rid), .m_rdata(p_rdata), .m_rresp(p_rresp),
        .m_rlast(p_rlast), .m_ruser(m_ruser),
        .m_rvalid(p_rvalid), .m_rready(p_rready)
    );

    // ── pb-side: 32 -> 128 ───────────────────────────────────────────
    axi_pb_lane_widen #(
        .ID_WIDTH(ID_WIDTH), .ADDR_WIDTH(ADDR_WIDTH),
        .WIDE_DW(DATA_WIDTH), .NARROW_DW(NARROW_DW)
    ) u_widen (
        .s_awid(p_awid), .s_awaddr(p_awaddr), .s_awlen(p_awlen),
        .s_awsize(p_awsize), .s_awburst(p_awburst),
        .s_awvalid(p_awvalid), .s_awready(p_awready),
        .s_wdata(p_wdata), .s_wstrb(p_wstrb), .s_wlast(p_wlast),
        .s_wvalid(p_wvalid), .s_wready(p_wready),
        .s_bid(p_bid), .s_bresp(p_bresp),
        .s_bvalid(p_bvalid), .s_bready(p_bready),
        .s_arid(p_arid), .s_araddr(p_araddr), .s_arlen(p_arlen),
        .s_arsize(p_arsize), .s_arburst(p_arburst),
        .s_arvalid(p_arvalid), .s_arready(p_arready),
        .s_rid(p_rid), .s_rdata(p_rdata), .s_rresp(p_rresp),
        .s_rlast(p_rlast), .s_rvalid(p_rvalid), .s_rready(p_rready),

        .m_awid(m_awid), .m_awaddr(m_awaddr), .m_awlen(m_awlen),
        .m_awsize(m_awsize), .m_awburst(m_awburst),
        .m_awvalid(m_awvalid), .m_awready(m_awready),
        .m_wdata(m_wdata), .m_wstrb(m_wstrb), .m_wlast(m_wlast),
        .m_wvalid(m_wvalid), .m_wready(m_wready),
        .m_bid(m_bid), .m_bresp(m_bresp),
        .m_bvalid(m_bvalid), .m_bready(m_bready),
        .m_arid(m_arid), .m_araddr(m_araddr), .m_arlen(m_arlen),
        .m_arsize(m_arsize), .m_arburst(m_arburst),
        .m_arvalid(m_arvalid), .m_arready(m_arready),
        .m_rid(m_rid), .m_rdata(m_rdata), .m_rresp(m_rresp),
        .m_rlast(m_rlast), .m_rvalid(m_rvalid), .m_rready(m_rready)
    );
endmodule

`default_nettype wire
