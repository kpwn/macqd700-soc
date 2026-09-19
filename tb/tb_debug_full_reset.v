// tb_debug_full_reset.v — Verilog wrapper for the debug-full-reset
// overlay re-arm DUT (task #256).  Mirrors the cpu-side reset gating
// logic from rtl/fpga_top_clocks.vh + the overlay re-arm FF from
// rtl/fpga_top_cpu.vh in a self-contained module so we can sim it
// without instantiating the entire fpga_top.
//
// Why this exists:
//   The actual re-arm logic lives inline inside `include slices of
//   fpga_top.v.  Verilator builds for the full SoC sim use mac_top.v
//   (no overlay).  This unit tb gives us a directed, Musashi-free
//   regression for "debug-full-reset (vio_boot_ctrl[3]=1) re-arms
//   reset_overlay_active_q AND resets the via1_overlay_bit input
//   path so the overlay stays asserted across the pulse".
//
// Bug it guards against: a previous version re-armed
// reset_overlay_active_q ONLY on core_rst, so the JTAG cpu-only-hold
// path could not restore the cold-boot ROM alias after a prior boot
// had cleared it.  See task #256.

`default_nettype none

module debug_full_reset_dut #(
    parameter [31:0] AXI_ROM_BASE = 32'h4000_0000,
    parameter [31:0] AXI_ROM_SIZE = 32'h0040_0000
) (
    input  wire        clk,
    input  wire        core_rst,           // board cold reset
    input  wire        jtag_cpu_hold,      // vio[2]
    input  wire        jtag_debug_full_rst,// vio[3] (task #256)
    // Phase-2 unified-reset signals (docs/reset_story.md):
    //   dbg_cold_reset_pulse — DBG_CONTROL bit 5 (write-1-to-pulse).
    //                          Fans into dbg_rst_src_level →
    //                          jtag_debug_full_reset_eff alongside
    //                          jtag_debug_full_rst, so the unified
    //                          reset is sourceable from JTAG-AXI in
    //                          addition to VIO/btn.
    //   dbg_cold_reset_hold  — DBG_CONTROL bit 4 (sticky level).
    //                          ORs into cpu_rst so the CPU stays
    //                          held after the pulse deasserts.
    input  wire        dbg_cold_reset_pulse,
    input  wire        dbg_cold_reset_hold,
    // VIA1 reset stub: when high, ORB[3] (the overlay bit) returns
    // to its post-reset value (1).  We don't need a full VIA1 here —
    // just the same reset-rearm semantics.
    output wire        soc_full_rst,
    output wire        cpu_rst,
    output reg         reset_overlay_active_q,
    // VIA1 overlay-bit model: ORB[3] resets to 1 on soc_full_rst, can
    // be cleared by an external "ROM clears overlay" pulse.  Mirrors
    // the relevant subset of rtl/mac/via1.v ORB[3] behaviour.
    input  wire        rom_clears_overlay, // pulse high to model ROM
                                            // writing ORB[3]=0
    output wire        via1_overlay_bit,
    // Sim-only hook: pulse high to model the auto-clear path
    // (sel_src_high_rom_read in rtl/fpga_top_cpu.vh) — clears the
    // overlay flag when CPU fetches from high ROM aperture.
    input  wire        high_rom_fetch
);
    // ---------- aggregate reset (mirrors fpga_top_clocks.vh) ----------
    // dbg_cold_reset_pulse joins jtag_debug_full_rst as a unified-reset
    // source, matching the production wiring where both fan into
    // dbg_rst_src_level → soc_full_rst.
    assign soc_full_rst = core_rst || jtag_debug_full_rst ||
                          dbg_cold_reset_pulse;
    // dbg_cold_reset_hold ORs into cpu_rst alongside jtag_cpu_hold.
    // The hold survives soc_full_rst because in production it lives
    // on debug_ctrl's `core_rst` reset domain (cleared only by board
    // cold reset, not by the unified reset it triggers).
    assign cpu_rst      = soc_full_rst || jtag_cpu_hold ||
                          dbg_cold_reset_hold;

    // ---------- VIA1 ORB[3] model (mirrors rtl/mac/via1.v) ----------
    reg orb3_q;     // ORB[3] storage; ddrb[3] = 1 once driven
    reg ddrb3_q;
    always @(posedge clk) begin
        if (soc_full_rst) begin
            orb3_q  <= 1'b1;   // ORB resets to 0x80; bit3=0, but
            ddrb3_q <= 1'b0;   // overlay_live = ddrb[3] ? orb[3] : 1'b1
                               // so with ddrb[3]=0 the overlay is asserted.
        end else if (rom_clears_overlay) begin
            // ROM writes DDRB[3]=1 then ORB[3]=0 to drop overlay.
            orb3_q  <= 1'b0;
            ddrb3_q <= 1'b1;
        end
    end
    assign via1_overlay_bit = (ddrb3_q ? orb3_q : 1'b1);

    // ---------- overlay re-arm FF (mirrors rtl/fpga_top_cpu.vh) ------
    always @(posedge clk) begin
        if (soc_full_rst)
            reset_overlay_active_q <= 1'b1;
        else if (high_rom_fetch || !via1_overlay_bit)
            reset_overlay_active_q <= 1'b0;
    end

    /* verilator lint_off UNUSEDPARAM */
    wire _unused = &{1'b0, AXI_ROM_BASE[0], AXI_ROM_SIZE[0]};
    /* verilator lint_on UNUSEDPARAM */
endmodule

`default_nettype wire
