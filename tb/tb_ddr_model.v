// tb_ddr_model.v — Verilator wrapper around ddr_ctrl.v in SIM_MODEL mode.
//
// Simply instantiates ddr_ctrl with SIM_MODEL defined and with a small
// BRAM depth suitable for unit testing (32 KB instead of 8 MB so Verilator
// builds quickly and memory footprint stays tiny).
//
// The ports mirror ddr_ctrl one-for-one so the C++ tb can exercise the
// slave directly.

`default_nettype none

module tb_ddr_model #(
    parameter DATA_WIDTH      = 128,
    parameter STRB_WIDTH      = DATA_WIDTH/8,
    parameter XID_WIDTH       = 6,
    parameter ADDR_WIDTH      = 32,
    parameter BRAM_LOG2_BEATS = 11,     // 2^11 * 16B = 32 KB
    parameter SIM_CAL_CYCLES  = 8
) (
    input  wire                       clk,
    input  wire                       rst,

    output wire                       ddr_clk,
    output wire                       ddr_cal_done,

    input  wire [XID_WIDTH-1:0]       awid,
    input  wire [ADDR_WIDTH-1:0]      awaddr,
    input  wire [7:0]                 awlen,
    input  wire [2:0]                 awsize,
    input  wire [1:0]                 awburst,
    input  wire                       awvalid,
    output wire                       awready,

    input  wire [DATA_WIDTH-1:0]      wdata,
    input  wire [STRB_WIDTH-1:0]      wstrb,
    input  wire                       wlast,
    input  wire                       wvalid,
    output wire                       wready,

    output wire [XID_WIDTH-1:0]       bid,
    output wire [1:0]                 bresp,
    output wire                       bvalid,
    input  wire                       bready,

    input  wire [XID_WIDTH-1:0]       arid,
    input  wire [ADDR_WIDTH-1:0]      araddr,
    input  wire [7:0]                 arlen,
    input  wire [2:0]                 arsize,
    input  wire [1:0]                 arburst,
    input  wire                       arvalid,
    output wire                       arready,

    output wire [XID_WIDTH-1:0]       rid,
    output wire [DATA_WIDTH-1:0]      rdata,
    output wire [1:0]                 rresp,
    output wire                       rlast,
    output wire                       rvalid,
    input  wire                       rready,

    output wire [31:0]                dbg_aw_cnt,
    output wire [31:0]                dbg_w_cnt,
    output wire [31:0]                dbg_b_cnt,
    output wire [31:0]                dbg_ar_cnt,
    output wire [31:0]                dbg_r_cnt
);

ddr_ctrl #(
    .DATA_WIDTH     (DATA_WIDTH),
    .STRB_WIDTH     (STRB_WIDTH),
    .XID_WIDTH      (XID_WIDTH),
    .ADDR_WIDTH     (ADDR_WIDTH),
    .BRAM_LOG2_BEATS(BRAM_LOG2_BEATS),
    .SIM_CAL_CYCLES (SIM_CAL_CYCLES)
) u_dut (
    .clk          (clk),
    .rst          (rst),
    .ddr_clk      (ddr_clk),
    .ddr_cal_done (ddr_cal_done),

    .awid   (awid),   .awaddr(awaddr), .awlen (awlen),
    .awsize (awsize), .awburst(awburst),
    .awvalid(awvalid),.awready(awready),
    .wdata  (wdata),  .wstrb (wstrb), .wlast (wlast),
    .wvalid (wvalid), .wready(wready),
    .bid    (bid),    .bresp (bresp), .bvalid(bvalid), .bready(bready),

    .arid   (arid),   .araddr(araddr), .arlen (arlen),
    .arsize (arsize), .arburst(arburst),
    .arvalid(arvalid),.arready(arready),
    .rid    (rid),    .rdata (rdata), .rresp (rresp),
    .rlast  (rlast),  .rvalid(rvalid), .rready(rready),

    .dbg_aw_cnt(dbg_aw_cnt),
    .dbg_w_cnt (dbg_w_cnt),
    .dbg_b_cnt (dbg_b_cnt),
    .dbg_ar_cnt(dbg_ar_cnt),
    .dbg_r_cnt (dbg_r_cnt)
);

endmodule

`default_nettype wire
