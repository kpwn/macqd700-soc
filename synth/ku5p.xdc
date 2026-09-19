# ku5p.xdc — KU5P timing and pin constraints for m68k-ooo Mac
#
# Part: xcku5p-ffvb676-2-i
# Target: 200 MHz main clock
#
# Pin assignments are PLACEHOLDERS — update with your actual board pinout
# before attempting place-and-route.

# ──────────────────────────────────────────────────────────────────────────────
# Primary clock: 200 MHz system clock
# (If your board provides a different reference, use create_generated_clock
#  for the PLL/MMCM output)
# ──────────────────────────────────────────────────────────────────────────────
create_clock -period 5.000 -name clk_200 -waveform {0.000 2.500} [get_ports clk]

# ──────────────────────────────────────────────────────────────────────────────
# If using an on-board 100 MHz reference and generating 200 MHz via MMCM:
# ──────────────────────────────────────────────────────────────────────────────
# create_clock -period 10.000 -name clk_ref_100 [get_ports clk_ref]
# create_generated_clock -name clk_200 -source [get_pins clk_rst/mmcm/CLKIN1] \
#     -multiply_by 2 [get_pins clk_rst/mmcm/CLKOUT0]

# ──────────────────────────────────────────────────────────────────────────────
# DDR4 clock (if MIG is used — adjust multiplier for your DDR4 speed grade)
# ──────────────────────────────────────────────────────────────────────────────
# create_clock -period 3.750 -name ddr4_sys_clk [get_ports ddr4_sys_clk_p]

# ──────────────────────────────────────────────────────────────────────────────
# Clock domain crossings — mark as false paths if appropriate
# ──────────────────────────────────────────────────────────────────────────────
# NOTE: this file (ku5p.xdc) is NOT loaded by the synth/impl flow — see
# synth/vivado.tcl which reads synth/fpga_top.xdc and friends instead.
# CDC constraints belong in synth/vivado.tcl's apply_*_cdc_constraints
# procs (which can resolve clocks dynamically post-elaboration).
# set_false_path -from [get_clocks clk_200] -to [get_clocks clk_pixel]
# set_false_path -from [get_clocks clk_200] -to [get_clocks clk_audio]

# ──────────────────────────────────────────────────────────────────────────────
# I/O timing (PLACEHOLDER — fill in per board schematic)
# ──────────────────────────────────────────────────────────────────────────────
# set_input_delay  -clock clk_200 -max 2.0 [get_ports {uart_rx}]
# set_output_delay -clock clk_200 -max 2.0 [get_ports {uart_tx}]
# set_input_delay  -clock clk_200 -max 1.5 [get_ports {rst}]

# ──────────────────────────────────────────────────────────────────────────────
# Pin assignments (PLACEHOLDER — update with real board pinout)
# ──────────────────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN   AK17  [get_ports clk]
set_property IOSTANDARD    LVCMOS18 [get_ports clk]

set_property PACKAGE_PIN   AJ16  [get_ports rst]
set_property IOSTANDARD    LVCMOS18 [get_ports rst]

# UART debug
set_property PACKAGE_PIN   AH12  [get_ports uart_tx]
set_property IOSTANDARD    LVCMOS18 [get_ports uart_tx]
set_property PACKAGE_PIN   AJ12  [get_ports uart_rx]
set_property IOSTANDARD    LVCMOS18 [get_ports uart_rx]

# ──────────────────────────────────────────────────────────────────────────────
# Timing exceptions for known multi-cycle paths
# ──────────────────────────────────────────────────────────────────────────────

# FPU is pipelined (4–8 cycles) — multicycle paths for FPU stage outputs
# (uncomment and refine when FPU pipeline depth is determined)
# set_multicycle_path -setup 4 -from [get_cells rtl/core/execute/fpu/*] \
#                              -to   [get_cells rtl/core/commit/*]
# set_multicycle_path -hold  3 -from [get_cells rtl/core/execute/fpu/*] \
#                              -to   [get_cells rtl/core/commit/*]

# Divide unit (multi-cycle, non-pipelined)
# set_multicycle_path -setup 20 -from [get_cells rtl/core/execute/mul_div/div_*]
# set_multicycle_path -hold  19 -from [get_cells rtl/core/execute/mul_div/div_*]

# ──────────────────────────────────────────────────────────────────────────────
# Configuration / bitstream settings
# ──────────────────────────────────────────────────────────────────────────────
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 85.0 [current_design]
