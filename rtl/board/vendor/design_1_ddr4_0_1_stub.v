// design_1_ddr4_0_1_stub.v -- pcie_test DDR4 MIG black-box declaration.
//
// This port list matches the known-working DDR4 IP from
// ~/FPGA/pcie_test/pcie_test.gen/sources_1/bd/design_1/ip/design_1_ddr4_0_1.
// Vivado stitches the synthesized OOC DCP into this black box in real-MIG
// builds; Verilator only needs the declaration for lint/elaboration.

`default_nettype none

(* X_CORE_INFO = "ddr4_v2_2_19,Vivado 2023.1" *)
module design_1_ddr4_0_1 (
    input  wire         sys_rst,
    input  wire         c0_sys_clk_p,
    input  wire         c0_sys_clk_n,

    output wire         c0_ddr4_act_n,
    output wire [16:0]  c0_ddr4_adr,
    output wire [1:0]   c0_ddr4_ba,
    output wire [0:0]   c0_ddr4_bg,
    output wire [0:0]   c0_ddr4_cke,
    output wire [0:0]   c0_ddr4_odt,
    output wire [0:0]   c0_ddr4_cs_n,
    output wire [0:0]   c0_ddr4_ck_t,
    output wire [0:0]   c0_ddr4_ck_c,
    output wire         c0_ddr4_reset_n,
    inout  wire [3:0]   c0_ddr4_dm_dbi_n,
    inout  wire [31:0]  c0_ddr4_dq,
    inout  wire [3:0]   c0_ddr4_dqs_c,
    inout  wire [3:0]   c0_ddr4_dqs_t,

    output wire         c0_init_calib_complete,
    output wire         c0_ddr4_ui_clk,
    output wire         c0_ddr4_ui_clk_sync_rst,
    output wire         dbg_clk,

    input  wire         c0_ddr4_aresetn,
    input  wire [0:0]   c0_ddr4_s_axi_awid,
    input  wire [30:0]  c0_ddr4_s_axi_awaddr,
    input  wire [7:0]   c0_ddr4_s_axi_awlen,
    input  wire [2:0]   c0_ddr4_s_axi_awsize,
    input  wire [1:0]   c0_ddr4_s_axi_awburst,
    input  wire [0:0]   c0_ddr4_s_axi_awlock,
    input  wire [3:0]   c0_ddr4_s_axi_awcache,
    input  wire [2:0]   c0_ddr4_s_axi_awprot,
    input  wire [3:0]   c0_ddr4_s_axi_awqos,
    input  wire         c0_ddr4_s_axi_awvalid,
    output wire         c0_ddr4_s_axi_awready,

    input  wire [255:0] c0_ddr4_s_axi_wdata,
    input  wire [31:0]  c0_ddr4_s_axi_wstrb,
    input  wire         c0_ddr4_s_axi_wlast,
    input  wire         c0_ddr4_s_axi_wvalid,
    output wire         c0_ddr4_s_axi_wready,

    input  wire         c0_ddr4_s_axi_bready,
    output wire [0:0]   c0_ddr4_s_axi_bid,
    output wire [1:0]   c0_ddr4_s_axi_bresp,
    output wire         c0_ddr4_s_axi_bvalid,

    input  wire [0:0]   c0_ddr4_s_axi_arid,
    input  wire [30:0]  c0_ddr4_s_axi_araddr,
    input  wire [7:0]   c0_ddr4_s_axi_arlen,
    input  wire [2:0]   c0_ddr4_s_axi_arsize,
    input  wire [1:0]   c0_ddr4_s_axi_arburst,
    input  wire [0:0]   c0_ddr4_s_axi_arlock,
    input  wire [3:0]   c0_ddr4_s_axi_arcache,
    input  wire [2:0]   c0_ddr4_s_axi_arprot,
    input  wire [3:0]   c0_ddr4_s_axi_arqos,
    input  wire         c0_ddr4_s_axi_arvalid,
    output wire         c0_ddr4_s_axi_arready,

    input  wire         c0_ddr4_s_axi_rready,
    output wire [0:0]   c0_ddr4_s_axi_rid,
    output wire [255:0] c0_ddr4_s_axi_rdata,
    output wire [1:0]   c0_ddr4_s_axi_rresp,
    output wire         c0_ddr4_s_axi_rlast,
    output wire         c0_ddr4_s_axi_rvalid,

    output wire [511:0] dbg_bus
);
endmodule

`default_nettype wire
