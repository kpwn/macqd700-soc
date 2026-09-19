// tb_via_tick_rate.v — the REAL Q700 60 Hz tick chain, end to end.
//
// The Mac's `Ticks` global is advanced by the level-1 VBL interrupt, and
// on this SoC that interrupt is produced by:
//
//     pb_clk ──► phi2 NCO ──► VIA2 Timer-1 (free-run, ACR[7]=PB7 out)
//            ──► via2_pb_out[7] ──► VIA1 CA1 ──► VIA1 IFR.CA1 ──► irq
//
// `fpga_top_peripherals.vh` wires `via1_ca1_in = via2_pb_out[7]` and both
// VIAs share the single `phi2_tick` strobe generated in
// `fpga_top_clocks.vh`.  This wrapper reproduces exactly that: the NCO is
// a verbatim copy of the fpga_top generator (same parameters), so a rate
// measured here is the rate the board produces.
//
// The pre-existing `tb_vbl_rate` gate does NOT cover this: it still
// models the superseded `via1_ca1_in = dafb_vbl_level` (HDMI-VTG) chain,
// which the SoC stopped using when the VIA2-PB7 wire went in.  And
// `tb_via2`'s `test_t1_freerun_pb7` only asserts that PB7 *toggles*, not
// how fast — it passes at 1.4 Hz and at 60 Hz alike.

`default_nettype none

module tb_via_tick_rate #(
    // Match rtl/soc/fpga_top.v's defaults so this measures the board.
    parameter integer PB_CLK_HZ   = 50_000_000,
    parameter integer VIA_PHI2_HZ = 783_360
) (
    input  wire        pb_clk,
    input  wire        pb_rst,

    // VIA2 peripheral-bus port (the timer that makes the tick).
    input  wire [3:0]  via2_addr,
    input  wire [7:0]  via2_wdata,
    input  wire        via2_wr,
    input  wire        via2_rd,
    output wire [7:0]  via2_rdata,

    // VIA1 peripheral-bus port (the interrupt the 68k actually takes).
    input  wire [3:0]  via1_addr,
    input  wire [7:0]  via1_wdata,
    input  wire        via1_wr,
    input  wire        via1_rd,
    output wire [7:0]  via1_rdata,

    // Observation points.
    output wire        phi2_tick_o,
    output wire        via2_pb7,
    output wire        via1_ca1_in,
    output wire        via1_irq,
    output wire        via2_irq
);

    // ── phi2_tick NCO — verbatim from rtl/soc/fpga_top_clocks.vh ──────
    localparam [31:0] PB_CLK_HZ_U32   = PB_CLK_HZ;
    localparam [31:0] VIA_PHI2_HZ_U32 = VIA_PHI2_HZ;
    reg [31:0] phi2_accum;
    reg        phi2_tick_r;
    wire [32:0] phi2_accum_sum = {1'b0, phi2_accum} + {1'b0, VIA_PHI2_HZ_U32};
    always @(posedge pb_clk) begin
        if (pb_rst) begin
            phi2_accum  <= 32'd0;
            phi2_tick_r <= 1'b0;
        end else if (phi2_accum_sum >= {1'b0, PB_CLK_HZ_U32}) begin
            phi2_accum  <= phi2_accum_sum[31:0] - PB_CLK_HZ_U32;
            phi2_tick_r <= 1'b1;
        end else begin
            phi2_accum  <= phi2_accum_sum[31:0];
            phi2_tick_r <= 1'b0;
        end
    end
    wire phi2_tick = phi2_tick_r;
    assign phi2_tick_o = phi2_tick;

    // ── VIA2 — Timer 1 drives PB7 ────────────────────────────────────
    /* verilator lint_off UNUSEDSIGNAL */
    wire [7:0] via2_pa_out, via2_pa_mask, via2_pb_mask;
    wire       via2_ca2_out, via2_cb1_out, via2_cb2_out;
    wire       via2_ack;
    /* verilator lint_on UNUSEDSIGNAL */
    wire [7:0] via2_pb_out;

    via2 u_via2 (
        .clk     (pb_clk),
        .rst     (pb_rst),
        .phi2_tick(phi2_tick),
        .pb_addr (via2_addr ),
        .pb_wdata(via2_wdata),
        .pb_wr   (via2_wr   ),
        .pb_rd   (via2_rd   ),
        .pb_rdata(via2_rdata),
        .pb_ack  (via2_ack  ),
        // Q700 reset straps, as fpga_top_peripherals.vh passes them.
        .pa_in   (8'hFF),
        .pb_in   (8'hCF),
        .pa_out  (via2_pa_out ),
        .pa_mask (via2_pa_mask),
        .pb_out  (via2_pb_out ),
        .pb_mask (via2_pb_mask),
        .ca1_in  (1'b1),
        .ca2_in  (1'b1),
        .cb1_in  (1'b1),
        .cb2_in  (1'b1),
        .ca2_out (via2_ca2_out),
        .cb1_out (via2_cb1_out),
        .cb2_out (via2_cb2_out),
        .irq     (via2_irq)
    );

    // The board-level VIA2-PB7 → VIA1-CA1 wire
    // (rtl/soc/fpga_top_peripherals.vh: `wire via1_ca1_in = via2_pb_out[7];`)
    assign via2_pb7    = via2_pb_out[7];
    assign via1_ca1_in = via2_pb_out[7];

    // ── VIA1 — CA1 is the 68k's level-1 VBL interrupt ────────────────
    /* verilator lint_off UNUSEDSIGNAL */
    wire [7:0] via1_pa_out, via1_pa_mask, via1_pb_out, via1_pb_mask;
    wire       via1_overlay_bit, via1_adb_rx_ready;
    wire [7:0] via1_adb_tx_byte;
    wire       via1_adb_tx_valid;
    wire       via1_rtc_enb, via1_rtc_clk, via1_rtc_data_o, via1_rtc_data_oe;
    wire       via1_cb2_out, via1_cb2_oe;
    wire       via1_ack;
    /* verilator lint_on UNUSEDSIGNAL */

    via1 #(
        .ENABLE_INTERNAL_VBL(1'b0)
    ) u_via1 (
        .clk         (pb_clk),
        .rst         (pb_rst),
        .phi2_tick   (phi2_tick),
        .pb_addr     (via1_addr ),
        .pb_wdata    (via1_wdata),
        .pb_wr       (via1_wr   ),
        .pb_rd       (via1_rd   ),
        .pb_rdata    (via1_rdata),
        .pb_ack      (via1_ack  ),
        .pa_in       (8'hC1),
        .pb_in       (8'h00),
        .cb1_in      (1'b0),
        .cb2_in      (1'b0),
        .cb2_out     (via1_cb2_out),
        .cb2_oe      (via1_cb2_oe),
        .pa_out      (via1_pa_out),
        .pa_mask     (via1_pa_mask),
        .pb_out      (via1_pb_out),
        .pb_mask     (via1_pb_mask),
        .overlay_bit (via1_overlay_bit),
        .adb_rx_byte (8'h00),
        .adb_rx_valid(1'b0),
        .adb_rx_ready(via1_adb_rx_ready),
        .adb_tx_byte (via1_adb_tx_byte),
        .adb_tx_valid(via1_adb_tx_valid),
        .rtc_enb     (via1_rtc_enb),
        .rtc_clk     (via1_rtc_clk),
        .rtc_data_o  (via1_rtc_data_o),
        .rtc_data_oe (via1_rtc_data_oe),
        .rtc_data_i  (1'b0),
        .rtc_cko     (1'b1),
        .vblank_irq_in(via1_ca1_in),
        .irq         (via1_irq)
    );

endmodule

`default_nettype wire
