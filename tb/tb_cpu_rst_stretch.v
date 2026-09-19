// tb_cpu_rst_stretch.v — directed regression for the cpu_rst minimum-
// pulse stretcher from rtl/fpga_top_clocks.vh.
//
// Bug guarded:
//   A 1-cycle glitch on any of the cpu_rst OR inputs (especially
//   dbg_soft_rst, which can race a JTAG-AXI write) gives the CPU a
//   single-cycle reset.  The OoO core's PRF/ROB/RAT/CCR-RAT need
//   multiple consecutive clock edges to clear cleanly — a truncated
//   reset pulse leaves them in a partially-reset state.
//
// Fix mirrored here: 8-cycle shift-register stretcher.  Any time the
// OR asserts, reload the SR with all 1s.  cpu_rst stays high for at
// least 8 cycles after the last input deasserts.

`default_nettype none

module cpu_rst_stretch_dut (
    input  wire clk,
    input  wire soc_full_rst,
    input  wire dbg_soft_rst,
    input  wire boot_rom_ready,
    input  wire cpu_rst_settle_done,
    output wire cpu_rst,
    output wire [7:0] stretch_dbg
);
    wire cpu_rst_or = soc_full_rst || dbg_soft_rst || !boot_rom_ready ||
                      !cpu_rst_settle_done;
    reg [7:0] cpu_rst_stretch;
    initial cpu_rst_stretch = 8'hFF;
    always @(posedge clk) begin
        if (cpu_rst_or)
            cpu_rst_stretch <= 8'hFF;
        else
            cpu_rst_stretch <= {cpu_rst_stretch[6:0], 1'b0};
    end

    assign cpu_rst    = cpu_rst_or || (|cpu_rst_stretch);
    assign stretch_dbg = cpu_rst_stretch;
endmodule

`default_nettype wire
