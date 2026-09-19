`default_nettype none

module tb_n2w_vram_byte (
    input  wire        clk,
    input  wire        rst,

    input  wire [31:0] n_awaddr,
    input  wire [2:0]  n_awprot,
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
    input  wire [2:0]  n_arprot,
    // Read transfer size (0=1 B, 1=2 B, 2=4 B) — see the "Sub-word load
    // support" note in axi_narrow_to_wide.v.  Driven by the harness; a
    // byte-sized read keeps its byte-granular address instead of being
    // rounded down to the containing 32-bit word.
    input  wire [2:0]  n_arsize,
    input  wire        n_arvalid,
    output wire        n_arready,
    output wire [31:0] n_rdata,
    output wire [1:0]  n_rresp,
    output wire        n_rlast,
    output wire        n_rvalid,
    input  wire        n_rready,

    input  wire [19:0] rd_addr,
    input  wire        rd_en,
    // 4-byte group starting at rd_addr (vram.v RD_DATA_W=32): [31:24] is the
    // byte at rd_addr (valid at any alignment), [23:0] the bytes at +1/+2/+3
    // (valid only when rd_addr[1:0]==0).
    output wire [31:0] rd_data,
    output wire        rd_valid
);

    wire [3:0]   m0_awid;
    wire [31:0]  m0_awaddr;
    wire [7:0]   m0_awlen;
    wire [2:0]   m0_awsize;
    wire [1:0]   m0_awburst;
    wire         m0_awvalid;
    wire         m0_awready;
    wire [127:0] m0_wdata;
    wire [15:0]  m0_wstrb;
    wire         m0_wlast;
    wire         m0_wvalid;
    wire         m0_wready;
    wire [3:0]   m0_bid;
    wire [1:0]   m0_bresp;
    wire         m0_bvalid;
    wire         m0_bready;
    wire [3:0]   m0_arid;
    wire [31:0]  m0_araddr;
    wire [7:0]   m0_arlen;
    wire [2:0]   m0_arsize;
    wire [1:0]   m0_arburst;
    wire         m0_arvalid;
    wire         m0_arready;
    wire [3:0]   m0_rid;
    wire [127:0] m0_rdata;
    wire [1:0]   m0_rresp;
    wire         m0_rlast;
    wire         m0_rvalid;
    wire         m0_rready;

    axi_narrow_to_wide #(
        .ID_WIDTH(4),
        .ID_TAG  (4'd0)
    ) u_n2w (
        .clk      (clk),
        .rst      (rst),
        .n_awlen  (8'd0),
        .n_arlen  (8'd0),
        .n_awaddr (n_awaddr),
        .n_awprot (n_awprot),
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
        .n_arprot (n_arprot),
        .n_arsize (n_arsize),
        .n_arvalid(n_arvalid),
        .n_arready(n_arready),
        .n_rdata  (n_rdata),
        .n_rresp  (n_rresp),
        .n_rlast  (n_rlast),
        .n_rvalid (n_rvalid),
        .n_rready (n_rready),
        .w_awid   (m0_awid),
        .w_awaddr (m0_awaddr),
        .w_awlen  (m0_awlen),
        .w_awsize (m0_awsize),
        .w_awburst(m0_awburst),
        .w_awvalid(m0_awvalid),
        .w_awready(m0_awready),
        .w_wdata  (m0_wdata),
        .w_wstrb  (m0_wstrb),
        .w_wlast  (m0_wlast),
        .w_wvalid (m0_wvalid),
        .w_wready (m0_wready),
        .w_bid    (m0_bid),
        .w_bresp  (m0_bresp),
        .w_bvalid (m0_bvalid),
        .w_bready (m0_bready),
        .w_arid   (m0_arid),
        .w_araddr (m0_araddr),
        .w_arlen  (m0_arlen),
        .w_arsize (m0_arsize),
        .w_arburst(m0_arburst),
        .w_arvalid(m0_arvalid),
        .w_arready(m0_arready),
        .w_rid    (m0_rid),
        .w_rdata  (m0_rdata),
        .w_rresp  (m0_rresp),
        .w_rlast  (m0_rlast),
        .w_rvalid (m0_rvalid),
        .w_rready (m0_rready)
    );

    tb_vram_xbar_e2e #(
        .FB_W  (1024),
        .FB_H  (768),
        .FB_BPP(8)
    ) u_bus (
        .clk      (clk),
        .rst      (rst),
        .m0_awid  (m0_awid),
        .m0_awaddr(m0_awaddr),
        .m0_awlen (m0_awlen),
        .m0_awsize(m0_awsize),
        .m0_awburst(m0_awburst),
        .m0_awvalid(m0_awvalid),
        .m0_awready(m0_awready),
        .m0_wdata (m0_wdata),
        .m0_wstrb (m0_wstrb),
        .m0_wlast (m0_wlast),
        .m0_wvalid(m0_wvalid),
        .m0_wready(m0_wready),
        .m0_bid   (m0_bid),
        .m0_bresp (m0_bresp),
        .m0_bvalid(m0_bvalid),
        .m0_bready(m0_bready),
        .m0_arid  (m0_arid),
        .m0_araddr(m0_araddr),
        .m0_arlen (m0_arlen),
        .m0_arsize(m0_arsize),
        .m0_arburst(m0_arburst),
        .m0_arvalid(m0_arvalid),
        .m0_arready(m0_arready),
        .m0_rid   (m0_rid),
        .m0_rdata (m0_rdata),
        .m0_rresp (m0_rresp),
        .m0_rlast (m0_rlast),
        .m0_rvalid(m0_rvalid),
        .m0_rready(m0_rready),
        .rd_addr  (rd_addr),
        .rd_en    (rd_en),
        .rd_data  (rd_data),
        .rd_valid (rd_valid)
    );

    wire _unused_awprot = &{1'b0, n_awprot};
    wire _unused_arprot = &{1'b0, n_arprot};

endmodule

`default_nettype wire
