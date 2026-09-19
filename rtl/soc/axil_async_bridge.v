// axil_async_bridge.v - AXI-Lite clock-domain crossing wrapper.
//
// This is a thin, synthesizable wrapper around axi_async_bridge for the
// single-beat AXI-Lite register windows hanging off peripheral_bus.  The
// full AXI bridge carries the channel payloads through async_fifo; this
// module supplies the constant AXI4 fields and exposes only the AXI-Lite
// handshake used by debug_ctrl, sd_provision, and the DAFB register shim.

`default_nettype none

module axil_async_bridge #(
    parameter integer DATA_WIDTH = 32,
    parameter integer ADDR_WIDTH = 20
) (
    // Upstream AXI-Lite slave side.
    input  wire                       s_clk,
    input  wire                       s_rst,
    input  wire [ADDR_WIDTH-1:0]      s_awaddr,
    input  wire                       s_awvalid,
    output wire                       s_awready,
    input  wire [DATA_WIDTH-1:0]      s_wdata,
    input  wire [(DATA_WIDTH/8)-1:0]  s_wstrb,
    input  wire                       s_wvalid,
    output wire                       s_wready,
    output wire [1:0]                 s_bresp,
    output wire                       s_bvalid,
    input  wire                       s_bready,
    input  wire [ADDR_WIDTH-1:0]      s_araddr,
    input  wire                       s_arvalid,
    output wire                       s_arready,
    output wire [DATA_WIDTH-1:0]      s_rdata,
    output wire [1:0]                 s_rresp,
    output wire                       s_rvalid,
    input  wire                       s_rready,

    // Downstream AXI-Lite master side.
    input  wire                       m_clk,
    input  wire                       m_rst,
    output wire [ADDR_WIDTH-1:0]      m_awaddr,
    output wire                       m_awvalid,
    input  wire                       m_awready,
    output wire [DATA_WIDTH-1:0]      m_wdata,
    output wire [(DATA_WIDTH/8)-1:0]  m_wstrb,
    output wire                       m_wvalid,
    input  wire                       m_wready,
    input  wire [1:0]                 m_bresp,
    input  wire                       m_bvalid,
    output wire                       m_bready,
    output wire [ADDR_WIDTH-1:0]      m_araddr,
    output wire                       m_arvalid,
    input  wire                       m_arready,
    input  wire [DATA_WIDTH-1:0]      m_rdata,
    input  wire [1:0]                 m_rresp,
    input  wire                       m_rvalid,
    output wire                       m_rready
);
    localparam integer STRB_WIDTH = DATA_WIDTH / 8;

    wire        s_bid_unused;
    wire        s_buser_unused;
    wire        s_rid_unused;
    wire        s_rlast_unused;
    wire        s_ruser_unused;

    wire        m_awid_unused;
    wire [7:0]  m_awlen_unused;
    wire [2:0]  m_awsize_unused;
    wire [1:0]  m_awburst_unused;
    wire        m_awlock_unused;
    wire [3:0]  m_awcache_unused;
    wire [2:0]  m_awprot_unused;
    wire [3:0]  m_awqos_unused;
    wire        m_awuser_unused;
    wire        m_wlast_unused;
    wire        m_wuser_unused;
    wire        m_arid_unused;
    wire [7:0]  m_arlen_unused;
    wire [2:0]  m_arsize_unused;
    wire [1:0]  m_arburst_unused;
    wire        m_arlock_unused;
    wire [3:0]  m_arcache_unused;
    wire [2:0]  m_arprot_unused;
    wire [3:0]  m_arqos_unused;
    wire        m_aruser_unused;

    axi_async_bridge #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .ID_WIDTH  (1),
        .USER_WIDTH(1)
    ) u_bridge (
        .s_clk(s_clk),
        .s_rst(s_rst),

        .s_awid   (1'b0),
        .s_awaddr (s_awaddr),
        .s_awlen  (8'd0),
        .s_awsize (3'd2),
        .s_awburst(2'b01),
        .s_awlock (1'b0),
        .s_awcache(4'd0),
        .s_awprot (3'd0),
        .s_awqos  (4'd0),
        .s_awuser (1'b0),
        .s_awvalid(s_awvalid),
        .s_awready(s_awready),

        .s_wdata  (s_wdata),
        .s_wstrb  (s_wstrb),
        .s_wlast  (1'b1),
        .s_wuser  (1'b0),
        .s_wvalid (s_wvalid),
        .s_wready (s_wready),

        .s_bid    (s_bid_unused),
        .s_bresp  (s_bresp),
        .s_buser  (s_buser_unused),
        .s_bvalid (s_bvalid),
        .s_bready (s_bready),

        .s_arid   (1'b0),
        .s_araddr (s_araddr),
        .s_arlen  (8'd0),
        .s_arsize (3'd2),
        .s_arburst(2'b01),
        .s_arlock (1'b0),
        .s_arcache(4'd0),
        .s_arprot (3'd0),
        .s_arqos  (4'd0),
        .s_aruser (1'b0),
        .s_arvalid(s_arvalid),
        .s_arready(s_arready),

        .s_rid    (s_rid_unused),
        .s_rdata  (s_rdata),
        .s_rresp  (s_rresp),
        .s_rlast  (s_rlast_unused),
        .s_ruser  (s_ruser_unused),
        .s_rvalid (s_rvalid),
        .s_rready (s_rready),

        .m_clk(m_clk),
        .m_rst(m_rst),

        .m_awid   (m_awid_unused),
        .m_awaddr (m_awaddr),
        .m_awlen  (m_awlen_unused),
        .m_awsize (m_awsize_unused),
        .m_awburst(m_awburst_unused),
        .m_awlock (m_awlock_unused),
        .m_awcache(m_awcache_unused),
        .m_awprot (m_awprot_unused),
        .m_awqos  (m_awqos_unused),
        .m_awuser (m_awuser_unused),
        .m_awvalid(m_awvalid),
        .m_awready(m_awready),

        .m_wdata  (m_wdata),
        .m_wstrb  (m_wstrb),
        .m_wlast  (m_wlast_unused),
        .m_wuser  (m_wuser_unused),
        .m_wvalid (m_wvalid),
        .m_wready (m_wready),

        .m_bid    (1'b0),
        .m_bresp  (m_bresp),
        .m_buser  (1'b0),
        .m_bvalid (m_bvalid),
        .m_bready (m_bready),

        .m_arid   (m_arid_unused),
        .m_araddr (m_araddr),
        .m_arlen  (m_arlen_unused),
        .m_arsize (m_arsize_unused),
        .m_arburst(m_arburst_unused),
        .m_arlock (m_arlock_unused),
        .m_arcache(m_arcache_unused),
        .m_arprot (m_arprot_unused),
        .m_arqos  (m_arqos_unused),
        .m_aruser (m_aruser_unused),
        .m_arvalid(m_arvalid),
        .m_arready(m_arready),

        .m_rid    (1'b0),
        .m_rdata  (m_rdata),
        .m_rresp  (m_rresp),
        .m_rlast  (1'b1),
        .m_ruser  (1'b0),
        .m_rvalid (m_rvalid),
        .m_rready (m_rready)
    );

    wire unused_bridge_sideband = &{1'b0,
        s_bid_unused, s_buser_unused, s_rid_unused, s_rlast_unused,
        s_ruser_unused, m_awid_unused, m_awlen_unused, m_awsize_unused,
        m_awburst_unused, m_awlock_unused, m_awcache_unused,
        m_awprot_unused, m_awqos_unused, m_awuser_unused,
        m_wlast_unused, m_wuser_unused, m_arid_unused, m_arlen_unused,
        m_arsize_unused, m_arburst_unused, m_arlock_unused,
        m_arcache_unused, m_arprot_unused, m_arqos_unused,
        m_aruser_unused};
endmodule

`default_nettype wire
