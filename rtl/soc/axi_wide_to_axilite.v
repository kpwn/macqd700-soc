// axi_wide_to_axilite.v — 128-bit AXI4 slave → 32-bit AXI-Lite master.
//
// Bridges a wide AXI4 slave port (as presented by axi_xbar's S2) to a
// narrow AXI-Lite peripheral such as dma_ctrl's `cfg_*` interface.
//
// Scope: single-beat transactions only (awlen=0, arlen=0).  32-bit slice
// is selected by awaddr[3:2] / araddr[3:2] — matches peripheral_bus.v
// lane policy.  One outstanding write and one outstanding read at a time.
//
// Verilog-2005, synchronous active-high rst.

`default_nettype none

module axi_wide_to_axilite #(
    parameter ID_WIDTH   = 6,
    parameter DATA_WIDTH = 128,
    parameter STRB_WIDTH = DATA_WIDTH/8
) (
    input  wire                   clk,
    input  wire                   rst,

    // Wide AXI4 slave face (from xbar S2).
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [ID_WIDTH-1:0]    s_awid,
    input  wire [31:0]            s_awaddr,
    input  wire [7:0]             s_awlen,
    input  wire [2:0]             s_awsize,
    input  wire [1:0]             s_awburst,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire                   s_awvalid,
    output wire                   s_awready,

    input  wire [DATA_WIDTH-1:0]  s_wdata,
    input  wire [STRB_WIDTH-1:0]  s_wstrb,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire                   s_wlast,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire                   s_wvalid,
    output wire                   s_wready,

    output wire [ID_WIDTH-1:0]    s_bid,
    output wire [1:0]             s_bresp,
    output wire                   s_bvalid,
    input  wire                   s_bready,

    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [ID_WIDTH-1:0]    s_arid,
    input  wire [31:0]            s_araddr,
    input  wire [7:0]             s_arlen,
    input  wire [2:0]             s_arsize,
    input  wire [1:0]             s_arburst,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire                   s_arvalid,
    output wire                   s_arready,

    output wire [ID_WIDTH-1:0]    s_rid,
    output wire [DATA_WIDTH-1:0]  s_rdata,
    output wire [1:0]             s_rresp,
    output wire                   s_rlast,
    output wire                   s_rvalid,
    input  wire                   s_rready,

    // Narrow AXI-Lite master face (to peripheral).
    output wire [19:0]            l_awaddr,
    output wire                   l_awvalid,
    input  wire                   l_awready,
    output wire [31:0]            l_wdata,
    output wire [3:0]             l_wstrb,
    output wire                   l_wvalid,
    input  wire                   l_wready,
    input  wire [1:0]             l_bresp,
    input  wire                   l_bvalid,
    output wire                   l_bready,

    output wire [19:0]            l_araddr,
    output wire                   l_arvalid,
    input  wire                   l_arready,
    input  wire [31:0]            l_rdata,
    input  wire [1:0]             l_rresp,
    input  wire                   l_rvalid,
    output wire                   l_rready
);

    // Write path.
    reg                   wr_busy;
    reg [ID_WIDTH-1:0]    wr_id_q;
    reg [19:0]            wr_addr_q;
    reg [1:0]             wr_lane_q;
    reg                   wr_aw_done;
    reg                   wr_w_done;
    reg                   wr_b_done;
    reg [1:0]             wr_resp_q;

    wire [1:0] s_aw_lane = s_awaddr[3:2];
    wire [31:0] wr_lane_data =
        (wr_lane_q == 2'd0) ? s_wdata[31:0]   :
        (wr_lane_q == 2'd1) ? s_wdata[63:32]  :
        (wr_lane_q == 2'd2) ? s_wdata[95:64]  :
                              s_wdata[127:96];
    wire [3:0]  wr_lane_strb =
        (wr_lane_q == 2'd0) ? s_wstrb[3:0]    :
        (wr_lane_q == 2'd1) ? s_wstrb[7:4]    :
        (wr_lane_q == 2'd2) ? s_wstrb[11:8]   :
                              s_wstrb[15:12];

    always @(posedge clk) begin
        if (rst) begin
            wr_busy    <= 1'b0;
            wr_id_q    <= {ID_WIDTH{1'b0}};
            wr_addr_q  <= 20'b0;
            wr_lane_q  <= 2'b0;
            wr_aw_done <= 1'b0;
            wr_w_done  <= 1'b0;
            wr_b_done  <= 1'b0;
            wr_resp_q  <= 2'b0;
        end else begin
            if (!wr_busy) begin
                if (s_awvalid) begin
                    wr_busy    <= 1'b1;
                    wr_id_q    <= s_awid;
                    wr_addr_q  <= s_awaddr[19:0];
                    wr_lane_q  <= s_aw_lane;
                    wr_aw_done <= 1'b0;
                    wr_w_done  <= 1'b0;
                    wr_b_done  <= 1'b0;
                    wr_resp_q  <= 2'b0;
                end
            end else begin
                if (!wr_aw_done && l_awready) wr_aw_done <= 1'b1;
                if (!wr_w_done  && l_wready  && s_wvalid) wr_w_done <= 1'b1;
                if (!wr_b_done  && l_bvalid) begin
                    wr_b_done <= 1'b1;
                    wr_resp_q <= l_bresp;
                end
                if (wr_b_done && s_bready) wr_busy <= 1'b0;
            end
        end
    end

    assign s_awready = !wr_busy;
    assign s_wready  = wr_busy && !wr_w_done && l_wready;
    assign s_bid     = wr_id_q;
    assign s_bresp   = wr_resp_q;
    assign s_bvalid  = wr_busy && wr_b_done;

    assign l_awaddr  = wr_addr_q;
    assign l_awvalid = wr_busy && !wr_aw_done;
    assign l_wdata   = wr_lane_data;
    assign l_wstrb   = wr_lane_strb;
    assign l_wvalid  = wr_busy && !wr_w_done && s_wvalid;
    assign l_bready  = wr_busy && !wr_b_done;

    // Read path.
    reg                   rd_busy;
    reg [ID_WIDTH-1:0]    rd_id_q;
    reg [19:0]            rd_addr_q;
    reg [1:0]             rd_lane_q;
    reg                   rd_ar_done;
    reg                   rd_r_done;
    reg [31:0]            rd_data_q;
    reg [1:0]             rd_resp_q;

    wire [1:0] s_ar_lane = s_araddr[3:2];

    always @(posedge clk) begin
        if (rst) begin
            rd_busy    <= 1'b0;
            rd_id_q    <= {ID_WIDTH{1'b0}};
            rd_addr_q  <= 20'b0;
            rd_lane_q  <= 2'b0;
            rd_ar_done <= 1'b0;
            rd_r_done  <= 1'b0;
            rd_data_q  <= 32'b0;
            rd_resp_q  <= 2'b0;
        end else begin
            if (!rd_busy) begin
                if (s_arvalid) begin
                    rd_busy    <= 1'b1;
                    rd_id_q    <= s_arid;
                    rd_addr_q  <= s_araddr[19:0];
                    rd_lane_q  <= s_ar_lane;
                    rd_ar_done <= 1'b0;
                    rd_r_done  <= 1'b0;
                end
            end else begin
                if (!rd_ar_done && l_arready) rd_ar_done <= 1'b1;
                if (!rd_r_done  && l_rvalid) begin
                    rd_r_done <= 1'b1;
                    rd_data_q <= l_rdata;
                    rd_resp_q <= l_rresp;
                end
                if (rd_r_done && s_rready) rd_busy <= 1'b0;
            end
        end
    end

    assign s_arready = !rd_busy;
    assign s_rid     = rd_id_q;
    assign s_rdata   = (rd_lane_q == 2'd0) ? {96'b0, rd_data_q} :
                       (rd_lane_q == 2'd1) ? {64'b0, rd_data_q, 32'b0} :
                       (rd_lane_q == 2'd2) ? {32'b0, rd_data_q, 64'b0} :
                                             {rd_data_q, 96'b0};
    assign s_rresp   = rd_resp_q;
    assign s_rlast   = 1'b1;
    assign s_rvalid  = rd_busy && rd_r_done;

    assign l_araddr  = rd_addr_q;
    assign l_arvalid = rd_busy && !rd_ar_done;
    assign l_rready  = rd_busy && !rd_r_done;

endmodule

`default_nettype wire
