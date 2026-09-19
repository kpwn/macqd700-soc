// tb_dbg_rst_pulse.v — Verilog wrapper for the debug-full-reset edge-
// detect / pulse-stretch / watchdog wrapper that lives inline in
// rtl/fpga_top_clocks.vh.  Extracted into a self-contained module so
// the unit tb can drive scenarios without instantiating fpga_top.
//
// Bug guarded:
//   vio_boot_ctrl[3] is the JTAG VIO probe-out bit that triggers
//   debug_full_reset.  Before this fix it was a level-driven input
//   straight into clk_rst's dbg_full_rst_in port.  If the host set
//   the bit and the JTAG link became unreachable, the bit stayed
//   high until the bitstream was reloaded — leaving the SoC stuck
//   in soc_full_rst forever.  Same risk for btn[2]: a stuck or
//   bouncy mechanical press would re-fire reset multiple times.
//
// Behaviour the tb verifies:
//   1. Rising edge on `src_level` produces a fixed-length pulse on
//      `pulse_eff` (PULSE_CYCLES) regardless of how long the source
//      stays high afterwards.
//   2. Holding the source high beyond STUCK_CYCLES disarms the
//      generator so a second pulse cannot fire — recovers from a
//      wedged probe-out without bitstream reload.
//   3. After the source returns low the watchdog rearms; the next
//      rising edge fires another pulse.

`default_nettype none

module dbg_rst_pulse_dut #(
    parameter integer PULSE_CYCLES = 8,
    parameter integer STUCK_CYCLES = 64
) (
    input  wire clk,
    input  wire rstn,           // active-low platform_resetn equivalent
    input  wire src_level,      // raw OR of (vio[3] || btn2_sync)
    output wire pulse_eff,      // stretched pulse to clk_rst.dbg_full_rst_in
    output wire armed_dbg       // for tb introspection of watchdog state
);
    function integer clog2_local;
        input integer v;
        integer       i;
        begin
            clog2_local = 0;
            for (i = v - 1; i > 0; i = i >> 1) clog2_local = clog2_local + 1;
        end
    endfunction
    localparam integer PCNT_W  = clog2_local(PULSE_CYCLES + 1);
    localparam integer STUCK_W = clog2_local(STUCK_CYCLES + 1);

    reg                 src_q;
    reg                 armed;
    reg [PCNT_W-1:0]    pulse_cnt;
    reg                 pulse_active;
    reg [STUCK_W-1:0]   stuck_cnt;

    wire rising = src_level && !src_q && armed;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            src_q        <= 1'b0;
            armed        <= 1'b1;
            pulse_cnt    <= {PCNT_W{1'b0}};
            pulse_active <= 1'b0;
            stuck_cnt    <= {STUCK_W{1'b0}};
        end else begin
            src_q <= src_level;

            if (rising) begin
                pulse_active <= 1'b1;
                pulse_cnt    <= PULSE_CYCLES[PCNT_W-1:0];
            end else if (pulse_active) begin
                if (pulse_cnt == {{(PCNT_W-1){1'b0}}, 1'b1}) begin
                    pulse_active <= 1'b0;
                    pulse_cnt    <= {PCNT_W{1'b0}};
                end else begin
                    pulse_cnt <= pulse_cnt - 1'b1;
                end
            end

            if (!src_level) begin
                stuck_cnt <= {STUCK_W{1'b0}};
                armed     <= 1'b1;
            end else if (armed && !pulse_active) begin
                if (stuck_cnt == STUCK_CYCLES[STUCK_W-1:0]) begin
                    armed <= 1'b0;
                end else begin
                    stuck_cnt <= stuck_cnt + 1'b1;
                end
            end
        end
    end

    assign pulse_eff = pulse_active;
    assign armed_dbg = armed;

endmodule

`default_nettype wire
