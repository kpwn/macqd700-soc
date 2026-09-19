// tb_clk_rst.v — Verilator wrapper for rtl/sys/clk_rst.v.
//
// Exercises the board-clock divide and reset-release helper in
// isolation.  The default parameter value targets the first-board
// 50 MHz bring-up case (200 MHz input / 4), but the Makefile target
// allows CLK_DIVIDE to be overridden for the 100 MHz / 25 MHz variants
// without touching the RTL.

`default_nettype none

module tb_clk_rst #(
    parameter integer CLK_DIVIDE = 4
) (
    input  wire clk_in,
    input  wire rst_in,
    input  wire init_done,
    output wire clk_out,
    output wire rst_out
);

    // The XPM-based clk_rst (post-d1c9f11) added bank/debug-reset ports.
    // The unit tb only exercises the divide + cold-reset release path,
    // so wire the new inputs to safe defaults and ignore the bank
    // outputs.
    wire        soc_full_rst_unused;
    wire [7:0]  core_rst_bank_unused;
    wire [7:0]  soc_full_rst_bank_unused;

    clk_rst #(
        .CLK_DIVIDE(CLK_DIVIDE),
        .BANK_W(8)
    ) u_clk_rst (
        .clk_in           (clk_in),
        .rst_in           (rst_in),
        .init_done        (init_done),
        .dbg_full_rst_in  (1'b0),
        .clk_out          (clk_out),
        .rst_out          (rst_out),
        .soc_full_rst_out (soc_full_rst_unused),
        .core_rst_bank    (core_rst_bank_unused),
        .soc_full_rst_bank(soc_full_rst_bank_unused)
    );

endmodule

`default_nettype wire
