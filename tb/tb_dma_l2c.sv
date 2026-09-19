`default_nettype none
module tb_dma_l2c (
    input wire clk, input wire rst,
    input wire [127:0] req_addr, input wire [27:0] req_len,
    input wire [2047:0] req_wdata, input wire [31:0] req_tag,
    input wire [3:0] req_write, input wire [3:0] req_valid,
    output wire [3:0] req_ready,
    output wire [2047:0] rsp_rdata, output wire [31:0] rsp_tag,
    output wire [27:0] rsp_len, output wire [7:0] rsp_status,
    output wire [3:0] rsp_write, output wire [3:0] rsp_valid,
    input wire [3:0] rsp_ready,
    output wire [5:0] m_awid, output wire [31:0] m_awaddr,
    output wire [7:0] m_awlen, output wire [2:0] m_awsize,
    output wire [1:0] m_awburst, output wire m_awvalid, input wire m_awready,
    output wire [127:0] m_wdata, output wire [15:0] m_wstrb,
    output wire m_wlast, output wire m_wvalid, input wire m_wready,
    input wire [5:0] m_bid, input wire [1:0] m_bresp,
    input wire m_bvalid, output wire m_bready,
    output wire [5:0] m_arid, output wire [31:0] m_araddr,
    output wire [7:0] m_arlen, output wire [2:0] m_arsize,
    output wire [1:0] m_arburst, output wire m_arvalid, input wire m_arready,
    input wire [5:0] m_rid, input wire [127:0] m_rdata,
    input wire [1:0] m_rresp, input wire m_rlast,
    input wire m_rvalid, output wire m_rready
);
    wire [5:0] d_awid,d_bid,d_arid,d_rid;
    wire [31:0] d_awaddr,d_araddr;
    wire [7:0] d_awlen,d_arlen;
    wire [2:0] d_awsize,d_arsize;
    wire [1:0] d_awburst,d_bresp,d_arburst,d_rresp;
    wire d_awvalid,d_awready,d_wlast,d_wvalid,d_wready,d_bvalid,d_bready;
    wire d_arvalid,d_arready,d_rlast,d_rvalid,d_rready;
    wire [127:0] d_wdata,d_rdata;
    wire [15:0] d_wstrb;

    dma_engine #(.N_CLIENTS(4),.DATA_WIDTH(128),.ID_WIDTH(6)) u_dma (
        .clk(clk),.rst(rst),.req_addr(req_addr),.req_len(req_len),
        .req_wdata(req_wdata),.req_tag(req_tag),.req_write(req_write),
        .req_valid(req_valid),.req_ready(req_ready),.rsp_rdata(rsp_rdata),
        .rsp_tag(rsp_tag),.rsp_len(rsp_len),.rsp_status(rsp_status),
        .rsp_write(rsp_write),.rsp_valid(rsp_valid),.rsp_ready(rsp_ready),
        .m_awid(d_awid),.m_awaddr(d_awaddr),.m_awlen(d_awlen),.m_awsize(d_awsize),
        .m_awburst(d_awburst),.m_awvalid(d_awvalid),.m_awready(d_awready),
        .m_wdata(d_wdata),.m_wstrb(d_wstrb),.m_wlast(d_wlast),
        .m_wvalid(d_wvalid),.m_wready(d_wready),.m_bid(d_bid),
        .m_bresp(d_bresp),.m_bvalid(d_bvalid),.m_bready(d_bready),
        .m_arid(d_arid),.m_araddr(d_araddr),.m_arlen(d_arlen),.m_arsize(d_arsize),
        .m_arburst(d_arburst),.m_arvalid(d_arvalid),.m_arready(d_arready),
        .m_rid(d_rid),.m_rdata(d_rdata),.m_rresp(d_rresp),.m_rlast(d_rlast),
        .m_rvalid(d_rvalid),.m_rready(d_rready)
    );

    wire [10:0] unused_snap0,unused_snap1;
    wire [31:0] unused_hits,unused_misses;
    wire [3:0] unused_occ;
    l2c u_l2c (
        .clk(clk),.rst(rst),
        .s_axi_awid(d_awid),.s_axi_awaddr(d_awaddr),.s_axi_awlen(d_awlen),
        .s_axi_awsize(d_awsize),.s_axi_awburst(d_awburst),
        .s_axi_awvalid(d_awvalid),.s_axi_awready(d_awready),
        .s_axi_wdata(d_wdata),.s_axi_wstrb(d_wstrb),.s_axi_wlast(d_wlast),
        .s_axi_wvalid(d_wvalid),.s_axi_wready(d_wready),
        .s_axi_bid(d_bid),.s_axi_bresp(d_bresp),.s_axi_bvalid(d_bvalid),.s_axi_bready(d_bready),
        .s_axi_arid(d_arid),.s_axi_araddr(d_araddr),.s_axi_arlen(d_arlen),
        .s_axi_arsize(d_arsize),.s_axi_arburst(d_arburst),
        .s_axi_arvalid(d_arvalid),.s_axi_arready(d_arready),
        .s_axi_rid(d_rid),.s_axi_rdata(d_rdata),.s_axi_rresp(d_rresp),
        .s_axi_rlast(d_rlast),.s_axi_rvalid(d_rvalid),.s_axi_rready(d_rready),
        .m_axi_awid(m_awid),.m_axi_awaddr(m_awaddr),.m_axi_awlen(m_awlen),
        .m_axi_awsize(m_awsize),.m_axi_awburst(m_awburst),
        .m_axi_awvalid(m_awvalid),.m_axi_awready(m_awready),
        .m_axi_wdata(m_wdata),.m_axi_wstrb(m_wstrb),.m_axi_wlast(m_wlast),
        .m_axi_wvalid(m_wvalid),.m_axi_wready(m_wready),
        .m_axi_bid(m_bid),.m_axi_bresp(m_bresp),.m_axi_bvalid(m_bvalid),.m_axi_bready(m_bready),
        .m_axi_arid(m_arid),.m_axi_araddr(m_araddr),.m_axi_arlen(m_arlen),
        .m_axi_arsize(m_arsize),.m_axi_arburst(m_arburst),
        .m_axi_arvalid(m_arvalid),.m_axi_arready(m_arready),
        .m_axi_rid(m_rid),.m_axi_rdata(m_rdata),.m_axi_rresp(m_rresp),
        .m_axi_rlast(m_rlast),.m_axi_rvalid(m_rvalid),.m_axi_rready(m_rready),
        .dbg_ctrl_write_snap(unused_snap0),.dbg_master_snap(unused_snap1),
        .dbg_hit_count(unused_hits),.dbg_miss_count(unused_misses),
        .dbg_mshr_occupancy(unused_occ)
    );
endmodule
`default_nettype wire
