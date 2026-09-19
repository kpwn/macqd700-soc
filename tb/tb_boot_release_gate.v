// tb_boot_release_gate.v — directed regression for the boot_rom_ready
// release gate from rtl/fpga_top_clocks.vh.
//
// Bug guarded:
//   The previous expression was
//     boot_rom_ready = jtag_boot_bypass ? jtag_boot_release : boot_rom_loaded;
//
//   With vio_boot_ctrl=0x3 (bypass=1, release=1), then a debug_full_reset
//   pulse, the [1] release-CPU bit STILL fires the moment soc_full_rst
//   drops — even though the boot FSM has not yet refreshed ROM.
//
//   New expression:
//     boot_rom_ready = !soc_full_rst &&
//                      (boot_rom_loaded ||
//                       (jtag_boot_bypass && jtag_boot_release));
//
//   plus boot_fsm_rst = soc_full_rst || jtag_boot_bypass.
//
// The DUT below mirrors that exact gating in isolation.

`default_nettype none

module boot_release_gate_dut (
    input  wire clk,
    input  wire soc_full_rst,
    input  wire boot_rom_loaded,
    input  wire jtag_boot_bypass,
    input  wire jtag_boot_release,
    output wire boot_rom_ready,
    output wire boot_fsm_rst,
    // CPU-rst aggregator for the tb to observe.  Mirrors the OR from
    // rtl/fpga_top_clocks.vh:
    //   cpu_rst = soc_full_rst || dbg_soft_rst || !boot_rom_ready ||
    //             !cpu_rst_settle_done.
    // We tie dbg_soft_rst off and assume cpu_rst_settle_done is high
    // here — the tb is about boot_rom_ready only.
    output wire cpu_rst
);

    assign boot_rom_ready = !soc_full_rst &&
                            (boot_rom_loaded ||
                             (jtag_boot_bypass && jtag_boot_release));
    assign boot_fsm_rst   = soc_full_rst || jtag_boot_bypass;
    assign cpu_rst        = soc_full_rst || !boot_rom_ready;

    // Quiet verilator about unused clock — the gate is purely
    // combinational here; the real RTL has plenty of FFs feeding the
    // inputs, but the tb drives them directly.
    /* verilator lint_off UNUSED */
    wire _unused_clk = clk;
    /* verilator lint_on UNUSED */
endmodule

`default_nettype wire
