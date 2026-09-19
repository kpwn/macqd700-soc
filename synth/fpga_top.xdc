# fpga_top.xdc — board-level constraints for the real-hw Vivado build.
#
# This file consolidates the pin + timing constraints specific to fpga_top.v
# ports that are NOT covered by peripheral-specific .xdc files (ddr4.xdc,
# hdmi.xdc).  Read order in synth/vivado.tcl:
#   read_xdc synth/fpga_top.xdc     (this file — core clock, reset, LEDs,
#                                     buttons, SD SPI)
#   read_xdc synth/ddr4.xdc         (DDR4 — all ddr4_* physical pins)
#   read_xdc synth/hdmi.xdc         (AL9134 — i2c + control + RGB data)
#   read_xdc synth/audio_pwm.xdc    (Σ-Δ PWM audio on AN9134 NC pins J1.35/36)
#
# The overlapping pin constraints historically in ku5p.xdc for a flat `clk`
# port (placeholder) are intentionally dropped by the vivado.tcl flow; only
# ku5p.xdc's configuration / bitstream settings at the bottom still apply.
#
# Part: xcku5p-ffvb676-2-i  (KU5P "mystery Chinese" board)

# ──────────────────────────────────────────────────────────────────────────────
# DDR4 reference / SIM_MODEL system clock (200 MHz differential, bank 66)
#
# In SIM_MODEL builds fpga_top buffers this pair as the fabric clock.
# In real-MIG builds the pair is passed directly to the MIG c0_sys_clk_p/n
# pins only; the fabric clock comes from the separate AB7/AB6 MGTREFCLK pair
# below, matching the known-good pcie_test split.
# ──────────────────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN  T24   [get_ports sys_clk_p]
set_property PACKAGE_PIN  U24   [get_ports sys_clk_n]
set_property IOSTANDARD   DIFF_SSTL12 [get_ports sys_clk_p]
set_property IOSTANDARD   DIFF_SSTL12 [get_ports sys_clk_n]
# Real-MIG builds stitch in the vendor MIG XDC, which already constrains this
# port. XDC does not support Tcl conditionals, so use -add to keep this
# fallback clock legal when a vendor clock is also present.
create_clock -add -period 5.000 -name sysclk200 [get_ports sys_clk_p]

# ──────────────────────────────────────────────────────────────────────────────
# Reset (active-low push-button)
# ──────────────────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN T19       [get_ports cpu_resetn]
set_property IOSTANDARD  LVCMOS12  [get_ports cpu_resetn]
# Active-low button (pressed = GND).  Pullup so a floating / not-pressed
# pin reads as HIGH = released = no reset.
set_property PULLUP      TRUE      [get_ports cpu_resetn]
set_false_path -from [get_ports cpu_resetn]

# ──────────────────────────────────────────────────────────────────────────────
# LEDs (bring-up visibility)
# ──────────────────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN H9  [get_ports {led[0]}]
set_property PACKAGE_PIN J9  [get_ports {led[1]}]
set_property PACKAGE_PIN G11 [get_ports {led[2]}]
set_property PACKAGE_PIN H11 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[*]}]

# ──────────────────────────────────────────────────────────────────────────────
# Board UART routed to selected RTL SCC channel
# ──────────────────────────────────────────────────────────────────────────────
set_property IOSTANDARD LVCMOS33 [get_ports uart_rtl_0_rxd]
set_property IOSTANDARD LVCMOS33 [get_ports uart_rtl_0_txd]
set_property PACKAGE_PIN AD13 [get_ports uart_rtl_0_rxd]
set_property PACKAGE_PIN AC14 [get_ports uart_rtl_0_txd]
set_false_path -from [get_ports uart_rtl_0_rxd]
set_false_path -to   [get_ports uart_rtl_0_txd]

# ──────────────────────────────────────────────────────────────────────────────
# Buttons
# ──────────────────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN K9  [get_ports {btn[0]}]
set_property PACKAGE_PIN K10 [get_ports {btn[1]}]
set_property PACKAGE_PIN J10 [get_ports {btn[2]}]
set_property PACKAGE_PIN J11 [get_ports {btn[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {btn[*]}]
# Active-low buttons (pressed = GND).  Pullup so floating / not-pressed
# pins read as HIGH = released.  RTL inverts btn[1..3] in
# rtl/fpga_top_clocks.vh to convert to active-high "pressed" semantics.
set_property PULLUP     TRUE     [get_ports {btn[*]}]
set_false_path -from [get_ports {btn[*]}]

# ──────────────────────────────────────────────────────────────────────────────
# SD card SPI pins (same as sd-hdmi-bringup)
# ──────────────────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN Y15  [get_ports sd_clk]
set_property PACKAGE_PIN AA15 [get_ports sd_mosi]
set_property PACKAGE_PIN AB14 [get_ports sd_miso]
set_property PACKAGE_PIN AB15 [get_ports sd_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports {sd_clk sd_mosi sd_miso sd_cs_n}]
set_property PULLUP true [get_ports sd_miso]
set_false_path -to   [get_ports {sd_clk sd_mosi sd_cs_n}]
set_false_path -from [get_ports sd_miso]

# ──────────────────────────────────────────────────────────────────────────────
# Configuration / bitstream settings (fallback copy — may already be set
# by ku5p.xdc; Vivado coalesces identical set_property calls harmlessly).
# ──────────────────────────────────────────────────────────────────────────────
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 85.0 [current_design]

# ──────────────────────────────────────────────────────────────────────────────
# Async clock-domain-crossing false paths
# ──────────────────────────────────────────────────────────────────────────────
# Both targets are 2-FF synchronisers carrying 1-bit signals between
# unrelated clock domains.  The destination FFs already carry the
# `(* ASYNC_REG = "TRUE" *)` attribute (placement hint for the timer to
# cluster the meta+sync pair on the same slice for max metastability
# margin), but ASYNC_REG alone does not stop Vivado from timing the
# cross-domain edge — it still reports setup violations like:
#   - u_dafb_vbl_cdc/src_toggle_reg → dst_meta_reg
#       (pclk_unbuf @ ~148 MHz → pb_clk @ 50 MHz; pulse_cdc instance in
#        fpga_top_peripherals.vh, video VBL → VIA1.CA1 wakeup path)
#   - u_mig_ddr4 calDone_gated → ddr_cal_done_pb_meta_reg
#       (mmcm_clkout0 @ 333 MHz → pb_clk; ddr_cal_done sync FFs in
#        fpga_top_clocks.vh)
# Both are TRUE async CDCs by construction (toggle synchroniser pulse +
# slow-changing status flag); the synchroniser FFs handle metastability,
# and the timer should treat the cross-domain edge as a false path.
set_false_path -to [get_pins -hier -filter {NAME =~ *u_dafb_vbl_cdc/dst_meta_reg/D}]
set_false_path -to [get_pins -hier -filter {NAME =~ *ddr_cal_done_pb_meta_reg/D}]

# u_scsi_trace_ring (rtl/soc/scsi_trace_ring.v) spans TWO domains: capture on
# pb_clk (50 MHz, snooping the 53C96 peripheral bus) and readout on core_clk
# (100 MHz, the xbar S2 register window at VHDD_BASE+0x20).  Four crossings,
# all 2-FF synchronised with ASYNC_REG on the destination pairs:
#   core_clk -> pb_clk : rd_freeze -> freeze_meta, rd_clear -> clear_meta
#                        (rd_clear is a TOGGLE, edge-detected after sync)
#   pb_clk -> core_clk : wr_ptr -> wrptr_meta, {frozen,wrapped} -> flags_meta
# wrptr_meta is multi-bit, so a sample taken mid-update can be skewed; the
# module's contract is that a reader takes two samples and only trusts a
# quiesced ring, which is why a plain binary synchroniser is acceptable here
# and gray coding is not required.
# Left unconstrained, these show up as unconstrained path ends -- and Vivado
# 2025.2 SEGFAULTED in visitUnconstrainedPathEnds during post-route phys opt
# on 2026-09-09 with these present and unconstrained.
set_false_path -to [get_pins -hier -filter {NAME =~ *u_scsi_trace_ring/freeze_meta_reg/D}]
set_false_path -to [get_pins -hier -filter {NAME =~ *u_scsi_trace_ring/clear_meta_reg/D}]
set_false_path -to [get_pins -hier -filter {NAME =~ *u_scsi_trace_ring/wrptr_meta_reg[*]/D}]
set_false_path -to [get_pins -hier -filter {NAME =~ *u_scsi_trace_ring/flags_meta_reg[*]/D}]

# ── DEBUG CSR: MULTICYCLE, NOT A FALSE PATH ───────────────────────────────────
# The debug CSR block (DebugCtrlPlugin) sits on core_clk (200 MHz, 5.0 ns) but is
# driven ONLY by JTAG. Measured on build/vivado200_rob32, its paths owned ~80 of
# the 2021 failing setup endpoints INCLUDING the design's worst:
#     -0.213  DebugCtrlPlugin_csr_awAddr_reg -> RobPlugin_exc_activeReg_reg
#     -0.208  DebugCtrlPlugin_csr_arAddr_reg -> DebugCtrlPlugin_csr_rData_reg  (x23)
#     -0.152  DebugCtrlPlugin_csr_awAddr_reg -> DcachePlugin_probeLineLine_reg (x56)
#
# WHY A MULTICYCLE IS SOUND HERE. The AXI4-Lite debug slave uses a pend-flag
# handshake (DebugCtrlPlugin.scala:459):
#     dbgAxi.awready := !awPend && !bPend && !dbgRst
# so `awAddr`/`arAddr` are CAPTURED when the pend flag sets and CANNOT CHANGE
# until that transaction's B/R response retires it. The next transaction cannot
# even be accepted until then. Every request originates in the JTAG TCK domain,
# which is orders of magnitude slower than core_clk, so the true interval between
# successive CSR addresses is thousands of core cycles. 4 is enormously
# conservative against that.
#
# WHY NOT set_false_path. A false path would also excuse the DATA these registers
# feed, and some of it IS functional -- `haltAfterInvalidate` reaches RobPlugin's
# retire gate. A multicycle keeps every path CHECKED, just against a realistic
# requirement. It is a relaxation, not a suppression.
#
# The -hold 3 companion is mandatory, not optional: without it the hold check
# moves to the same relaxed edge and manufactures hold violations that are not
# real. (N setup / N-1 hold is the standard pairing.)
#
# ⚠️ SCOPE. Deliberately anchored to the two CSR ADDRESS registers as SOURCE and
# rData as DESTINATION -- NOT a blanket exception on the DebugCtrl hierarchy. A
# blanket rule would silently cover any future functional signal that happens to
# live in this plugin. If you add a debug output with real-time semantics, it will
# be timed normally unless you consciously add it here.
set_multicycle_path 4 -setup -from [get_cells -quiet -hier -filter {NAME =~ *DebugCtrlPlugin_logic_csr_a*Addr_reg*}]
set_multicycle_path 3 -hold  -from [get_cells -quiet -hier -filter {NAME =~ *DebugCtrlPlugin_logic_csr_a*Addr_reg*}]
set_multicycle_path 4 -setup -to   [get_cells -quiet -hier -filter {NAME =~ *DebugCtrlPlugin_logic_csr_rData_reg*}]
set_multicycle_path 3 -hold  -to   [get_cells -quiet -hier -filter {NAME =~ *DebugCtrlPlugin_logic_csr_rData_reg*}]
