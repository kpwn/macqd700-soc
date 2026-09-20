// Xilinx primitive stubs for fpga_top lint.
//
// These are only for Verilator lint of fpga_top.  Vivado uses the real
// Xilinx primitives from the device libraries.

`default_nettype none

module IBUFDS #(
    parameter DIFF_TERM    = "FALSE",
    parameter IBUF_LOW_PWR = "TRUE",
    parameter IOSTANDARD   = "DEFAULT"
) (
    input  wire I,
    input  wire IB,
    output wire O
);
    assign O = I;

    wire _unused = &{IB, 1'b0};
endmodule

module IBUFDS_GTE4 #(
    parameter REFCLK_EN_TX_PATH  = 1'b0,
    parameter REFCLK_HROW_CK_SEL = 2'b00,
    parameter REFCLK_ICNTL_RX    = 2'b00
) (
    input  wire I,
    input  wire IB,
    input  wire CEB,
    output wire O,
    output wire ODIV2
);
    assign O     = I;
    assign ODIV2 = I;

    wire _unused = &{IB, CEB, REFCLK_EN_TX_PATH,
                     REFCLK_HROW_CK_SEL, REFCLK_ICNTL_RX, 1'b0};
endmodule

module BUFG (
    input  wire I,
    output wire O
);
    assign O = I;
endmodule

module BUFG_GT (
    input  wire       I,
    input  wire       CE,
    input  wire       CEMASK,
    input  wire       CLR,
    input  wire       CLRMASK,
    input  wire [2:0] DIV,
    output wire       O
);
    assign O = I;

    wire _unused = &{CE, CEMASK, CLR, CLRMASK, DIV, 1'b0};
endmodule

// xpm_cdc_async_rst — Xilinx Parameterized Macro for async-assert /
// sync-deassert reset distribution (UG974).
//
// Behavioural stub for Verilator lint + sim.  Mirrors the real macro:
//   - dest_arst asserts asynchronously when src_arst rises
//     (matches RST_ACTIVE_HIGH = 1).
//   - dest_arst deasserts synchronously DEST_SYNC_FF cycles after
//     src_arst falls.
//
// In sim every FF in the chain is initialised to the asserted state so
// the destination clock domain powers up reset (matching the real macro
// when INIT_SYNC_FF = 0 — Vivado SAFE_INIT preserves the asserted reset
// during configuration anyway).
//
// Vivado uses the real device-library macro; this stub never reaches
// synthesis.
module xpm_cdc_async_rst #(
    parameter integer DEST_SYNC_FF    = 4,
    parameter integer INIT_SYNC_FF    = 0,
    parameter integer RST_ACTIVE_HIGH = 1
) (
    input  wire src_arst,
    input  wire dest_clk,
    output wire dest_arst
);
    // Power-up asserted; bit-width sized to DEST_SYNC_FF so the
    // shift-register length matches the real macro.
    reg [DEST_SYNC_FF-1:0] sync_chain = {DEST_SYNC_FF{1'b1}};

    always @(posedge dest_clk or posedge src_arst) begin
        if (src_arst) begin
            sync_chain <= {DEST_SYNC_FF{1'b1}};
        end else begin
            sync_chain <= {sync_chain[DEST_SYNC_FF-2:0], 1'b0};
        end
    end

    assign dest_arst = sync_chain[DEST_SYNC_FF-1];

    wire _unused = &{INIT_SYNC_FF, RST_ACTIVE_HIGH, 1'b0};
endmodule

module BUFGCE_DIV #(
    parameter integer BUFGCE_DIVIDE = 1
) (
    input  wire I,
    input  wire CE,
    input  wire CLR,
    output wire O
);
    generate
        if (BUFGCE_DIVIDE == 1) begin : gen_passthrough
            assign O = I;
        end else begin : gen_div
            reg o_r;
            integer div_count;

            assign O = o_r;

            always @(posedge I or posedge CLR) begin
                if (CLR) begin
                    o_r <= 1'b0;
                    div_count <= 0;
                end else if (CE) begin
                    if (div_count == (BUFGCE_DIVIDE / 2) - 1) begin
                        o_r <= ~o_r;
                        div_count <= 0;
                    end else begin
                        div_count <= div_count + 1;
                    end
                end
            end
        end
    endgenerate
endmodule

module debug_vio (
    input wire clk,
    input wire [31:0] probe_in0,
    input wire [5:0] probe_in1,
    input wire [9:0] probe_in2,
    input wire [7:0] probe_in3,
    input wire [15:0] probe_in4,
    input wire [31:0] probe_in5,
    input wire [31:0] probe_in6,
    input wire [31:0] probe_in7,
    input wire [31:0] probe_in8,
    input wire [67:0] probe_in9,
    input wire [186:0] probe_in10,
    input wire [47:0] probe_in11,
    input wire [95:0] probe_in12,
    input wire [159:0] probe_in13,
    input wire [83:0] probe_in14,
    output wire [4:0] probe_out0,
    output wire probe_out1
);
    assign probe_out0 = 5'b00011;
    assign probe_out1 = 1'b0;
    wire _unused = &{1'b0, clk, probe_in0, probe_in1, probe_in2, probe_in3, probe_in4, probe_in5, probe_in6, probe_in7, probe_in8, probe_in9, probe_in10, probe_in11, probe_in12, probe_in13, probe_in14};
endmodule

// Ethernet-link clock/delay primitives used by q700_eth_link.sv.  Functional
// clock ratios are not modeled here; these shells exist so ETH_ENABLE whole-
// top lint elaborates the same hierarchy that Vivado sees.
module MMCME4_BASE #(
    parameter real CLKIN1_PERIOD = 0.0,
    parameter integer DIVCLK_DIVIDE = 1,
    parameter real CLKFBOUT_MULT_F = 1.0,
    parameter real CLKOUT0_DIVIDE_F = 1.0,
    parameter integer CLKOUT1_DIVIDE = 1,
    parameter real CLKOUT1_PHASE = 0.0,
    parameter integer CLKOUT2_DIVIDE = 1,
    parameter BANDWIDTH = "OPTIMIZED",
    parameter STARTUP_WAIT = "FALSE"
) (
    input wire CLKIN1, CLKFBIN, RST, PWRDWN,
    output wire CLKFBOUT, CLKFBOUTB,
    output wire CLKOUT0, CLKOUT0B, CLKOUT1, CLKOUT1B,
    output wire CLKOUT2, CLKOUT2B, CLKOUT3, CLKOUT3B,
    output wire CLKOUT4, CLKOUT5, CLKOUT6, LOCKED
);
    assign CLKFBOUT=CLKIN1; assign CLKFBOUTB=~CLKIN1;
    assign CLKOUT0=CLKIN1; assign CLKOUT0B=~CLKIN1;
    assign CLKOUT1=CLKIN1; assign CLKOUT1B=~CLKIN1;
    assign CLKOUT2=CLKIN1; assign CLKOUT2B=~CLKIN1;
    assign CLKOUT3=1'b0; assign CLKOUT3B=1'b0; assign CLKOUT4=1'b0;
    assign CLKOUT5=1'b0; assign CLKOUT6=1'b0; assign LOCKED=~RST&~PWRDWN;
    wire _unused_mmcm=&{CLKFBIN,1'b0};
endmodule

module IDELAYCTRL #(parameter SIM_DEVICE="ULTRASCALE") (
    input wire REFCLK, RST, output wire RDY
);
    assign RDY=~RST; wire _unused_idelayctrl=REFCLK;
endmodule

module IDELAYE3 #(
    parameter DELAY_SRC="IDATAIN", parameter CASCADE="NONE",
    parameter DELAY_TYPE="FIXED", parameter integer DELAY_VALUE=0,
    parameter real REFCLK_FREQUENCY=300.0, parameter DELAY_FORMAT="TIME",
    parameter UPDATE_MODE="SYNC", parameter SIM_DEVICE="ULTRASCALE_PLUS"
) (
    input wire CASC_IN,CASC_RETURN,IDATAIN,DATAIN,CLK,EN_VTC,CE,INC,LOAD,RST,
    input wire [8:0] CNTVALUEIN,
    output wire CASC_OUT,DATAOUT,output wire [8:0] CNTVALUEOUT
);
    assign CASC_OUT=1'b0; assign DATAOUT=IDATAIN; assign CNTVALUEOUT=CNTVALUEIN;
    wire _unused_idelaye3=&{CASC_IN,CASC_RETURN,DATAIN,CLK,EN_VTC,CE,INC,LOAD,RST,1'b0};
endmodule

`default_nettype wire

// ---------------------------------------------------------------------
// debug_ila / debug_jtag_axi -- Xilinx IP black boxes, stubbed for lint.
//
// Added 2026-09-05. Without these, ANY lint config that defines
// ILA_ENABLE or JTAG_AXI_ENABLE dies with 'Cannot find file containing
// module', so the entire `ifdef ILA_ENABLE arm of
// fpga_top_debug_ctrl.vh could not be linted at all -- which is how it
// came to connect ~22 .dbg040_* ports that exist on no cpu040 ref and
// stay broken for a week, undetected, because every board bitstream in
// that period was built with ENABLE_ILA=0.
//
// Probe widths below are GENERATED from synth/debug_ila.tcl's
// CONFIG.C_PROBEn_WIDTH (unspecified => 1, the IP default). They must
// track that file, exactly as debug_vio's stub must track
// gen_debug_vio_ip -- otherwise lint and synth disagree.
module debug_ila (
    input  wire        clk,
    input  wire        probe0,
    input  wire [6:0] probe1,
    input  wire [6:0] probe2,
    input  wire        probe3,
    input  wire        probe4,
    input  wire [4:0] probe5,
    input  wire [4:0] probe6,
    input  wire [6:0] probe7,
    input  wire [6:0] probe8,
    input  wire [7:0] probe9,
    input  wire [31:0] probe10,
    input  wire [15:0] probe11,
    input  wire [31:0] probe12,
    input  wire [32:0] probe13,
    input  wire [39:0] probe14,
    input  wire [8:0] probe15,
    input  wire [8:0] probe16,
    input  wire [8:0] probe17,
    input  wire [7:0] probe18,
    input  wire [8:0] probe19,
    input  wire [31:0] probe20,
    input  wire [31:0] probe21,
    input  wire [31:0] probe22,
    input  wire        probe23,
    input  wire [31:0] probe24,
    input  wire [31:0] probe25,
    input  wire [6:0] probe26,
    input  wire        probe27,
    input  wire [31:0] probe28,
    input  wire [31:0] probe29,
    input  wire [23:0] probe30,
    input  wire [6:0] probe31,
    input  wire        probe32,
    input  wire        probe33,
    input  wire [31:0] probe34,
    input  wire [31:0] probe35,
    input  wire [31:0] probe36,
    input  wire        probe37,
    input  wire        probe38,
    input  wire [5:0] probe39,
    input  wire [31:0] probe40,
    input  wire [10:0] probe41,
    input  wire [10:0] probe42,
    input  wire        probe43,
    input  wire [31:0] probe44,
    input  wire [31:0] probe45,
    input  wire [15:0] probe46,
    input  wire [13:0] probe47,
    input  wire [31:0] probe48,
    input  wire [16:0] probe49,
    input  wire [31:0] probe50,
    input  wire [31:0] probe51,
    input  wire [31:0] probe52,
    input  wire [31:0] probe53,
    input  wire [31:0] probe54,
    input  wire [15:0] probe55,
    input  wire [255:0] probe56
);
    wire _unused_ila = &{1'b0, clk, probe0, probe1, probe2, probe3, probe4, probe5, probe6, probe7, probe8, probe9, probe10, probe11, probe12, probe13, probe14, probe15, probe16, probe17, probe18, probe19, probe20, probe21, probe22, probe23, probe24, probe25, probe26, probe27, probe28, probe29, probe30, probe31, probe32, probe33, probe34, probe35, probe36, probe37, probe38, probe39, probe40, probe41, probe42, probe43, probe44, probe45, probe46, probe47, probe48, probe49, probe50, probe51, probe52, probe53, probe54, probe55, probe56};
endmodule


module debug_jtag_axi (
    input  wire        aclk,
    input  wire        aresetn,
    output wire [0:0]  m_axi_awid,
    output wire [3:0]  m_axi_awqos,
    output wire [31:0] m_axi_awaddr,
    output wire [7:0]  m_axi_awlen,
    output wire [2:0]  m_axi_awsize,
    output wire [1:0]  m_axi_awburst,
    output wire [3:0]  m_axi_awcache,
    output wire [2:0]  m_axi_awprot,
    output wire        m_axi_awlock,
    output wire        m_axi_awvalid,
    input  wire        m_axi_awready,
    output wire [31:0] m_axi_wdata,
    output wire [3:0]  m_axi_wstrb,
    output wire        m_axi_wlast,
    output wire        m_axi_wvalid,
    input  wire        m_axi_wready,
    input  wire [0:0]  m_axi_bid,
    input  wire [1:0]  m_axi_bresp,
    input  wire        m_axi_bvalid,
    output wire        m_axi_bready,
    output wire [0:0]  m_axi_arid,
    output wire [3:0]  m_axi_arqos,
    output wire [31:0] m_axi_araddr,
    output wire [7:0]  m_axi_arlen,
    output wire [2:0]  m_axi_arsize,
    output wire [1:0]  m_axi_arburst,
    output wire [3:0]  m_axi_arcache,
    output wire [2:0]  m_axi_arprot,
    output wire        m_axi_arlock,
    output wire        m_axi_arvalid,
    input  wire        m_axi_arready,
    input  wire [0:0]  m_axi_rid,
    input  wire [31:0] m_axi_rdata,
    input  wire [1:0]  m_axi_rresp,
    input  wire        m_axi_rlast,
    input  wire        m_axi_rvalid,
    output wire        m_axi_rready
);
    assign m_axi_awid = 1'd0; assign m_axi_awqos = 4'd0;
    assign m_axi_arid = 1'd0; assign m_axi_arqos = 4'd0;
    assign m_axi_awaddr = 32'd0; assign m_axi_awlen = 8'd0;
    assign m_axi_awsize = 3'd0;  assign m_axi_awburst = 2'd0;
    assign m_axi_awcache = 4'd0; assign m_axi_awprot = 3'd0;
    assign m_axi_awlock = 1'b0;  assign m_axi_awvalid = 1'b0;
    assign m_axi_wdata = 32'd0;  assign m_axi_wstrb = 4'd0;
    assign m_axi_wlast = 1'b0;   assign m_axi_wvalid = 1'b0;
    assign m_axi_bready = 1'b0;
    assign m_axi_araddr = 32'd0; assign m_axi_arlen = 8'd0;
    assign m_axi_arsize = 3'd0;  assign m_axi_arburst = 2'd0;
    assign m_axi_arcache = 4'd0; assign m_axi_arprot = 3'd0;
    assign m_axi_arlock = 1'b0;  assign m_axi_arvalid = 1'b0;
    assign m_axi_rready = 1'b0;
    wire _unused_jtag_axi = &{1'b0, aclk, aresetn, m_axi_awready,
        m_axi_wready, m_axi_bid, m_axi_rid, m_axi_bresp, m_axi_bvalid, m_axi_arready,
        m_axi_rdata, m_axi_rresp, m_axi_rlast, m_axi_rvalid};
endmodule


// STARTUPE3 — the configuration-engine primitive fpga_top_clocks.vh uses
// for EOS (end-of-startup), which gates fabric_gt_clr.  Only instantiated
// in the `ifndef SIM_MODEL (real-hardware) arm, so `make lint-configs` --
// which always passes -DSIM_MODEL -- never needed it.  `make lint-realmig`
// does: see the Makefile's note on why that arm needs elaborating at all.
module STARTUPE3 #(
    parameter PROG_USR           = "FALSE",
    parameter real SIM_CCLK_FREQ = 0.0
) (
    output wire       CFGCLK,
    output wire       CFGMCLK,
    output wire [3:0] DI,
    output wire       EOS,
    output wire       PREQ,
    input  wire [3:0] DO,
    input  wire [3:0] DTS,
    input  wire       FCSBO,
    input  wire       FCSBTS,
    input  wire       GSR,
    input  wire       GTS,
    input  wire       KEYCLEARB,
    input  wire       PACK,
    input  wire       USRCCLKO,
    input  wire       USRCCLKTS,
    input  wire       USRDONEO,
    input  wire       USRDONETS
);
    // Startup is complete as far as lint is concerned.
    assign EOS    = 1'b1;
    assign CFGCLK = 1'b0;
    assign CFGMCLK= 1'b0;
    assign DI     = 4'b0;
    assign PREQ   = 1'b0;
    wire _unused_startupe3 = &{1'b0, DO, DTS, FCSBO, FCSBTS, GSR, GTS,
                               KEYCLEARB, PACK, USRCCLKO, USRCCLKTS,
                               USRDONEO, USRDONETS};
endmodule
