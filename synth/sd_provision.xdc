# sd_provision.xdc — pin + timing constraints for the DEDICATED SD-card
# provisioning bitstream (rtl/soc/sd_provision_top.v).
#
# Pin assignments are copied from the authoritative fpga_top constraints
# (synth/fpga_top.xdc / synth/fpga_top_real_mig.xdc) — same board, tiny
# port subset.  This bitstream is JTAG-loaded transiently for SD-card
# bulk writes and then replaced by the normal fpga_top bitstream.
#
# Part: xcku5p-ffvb676-2-i

# ──────────────────────────────────────────────────────────────────────────────
# Fabric clock — 100 MHz MGTREFCLK0 pair (AB7/AB6), consumed through
# IBUFDS_GTE4 + BUFG_GT exactly like the real-MIG fpga_top build.
# ──────────────────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN AB7 [get_ports fabric_clk_p]
set_property PACKAGE_PIN AB6 [get_ports fabric_clk_n]
set_property IO_BUFFER_TYPE NONE [get_ports {fabric_clk_p fabric_clk_n}]
set_property CLOCK_BUFFER_TYPE NONE [get_ports {fabric_clk_p fabric_clk_n}]
create_clock -period 10.000 -name fabric_clk100 [get_ports fabric_clk_p]

# ──────────────────────────────────────────────────────────────────────────────
# Reset (active-low push-button)
# ──────────────────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN T19       [get_ports cpu_resetn]
set_property IOSTANDARD  LVCMOS12  [get_ports cpu_resetn]
set_property PULLUP      TRUE      [get_ports cpu_resetn]
set_false_path -from [get_ports cpu_resetn]

# ──────────────────────────────────────────────────────────────────────────────
# LEDs — led[0] card_ready, led[1] writer_busy, led[2] error, led[3] heartbeat
# ──────────────────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN H9  [get_ports {led[0]}]
set_property PACKAGE_PIN J9  [get_ports {led[1]}]
set_property PACKAGE_PIN G11 [get_ports {led[2]}]
set_property PACKAGE_PIN H11 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[*]}]
set_false_path -to [get_ports {led[*]}]

# ──────────────────────────────────────────────────────────────────────────────
# SD card SPI pins
# ──────────────────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN Y15  [get_ports sd_clk]
set_property PACKAGE_PIN AA15 [get_ports sd_mosi]
set_property PACKAGE_PIN AB14 [get_ports sd_miso]
set_property PACKAGE_PIN AB15 [get_ports sd_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports {sd_clk sd_mosi sd_miso sd_cs_n}]
set_property PULLUP true [get_ports sd_miso]
# Match the full SoC's FPGA-side SPI latency allocation, not an external
# card timing guarantee. Leave the synchronizer interstage path timed.
set sd_spi_tx_q [get_pins -hier -filter {NAME =~ */spi_clk_reg/Q || NAME =~ */spi_mosi_reg/Q || NAME =~ */spi_cs_n_reg/Q}]
set_max_delay -datapath_only 4.000 -from $sd_spi_tx_q -to [get_ports {sd_clk sd_mosi sd_cs_n}]
set_max_delay -datapath_only 3.000 -from [get_ports sd_miso]

# ──────────────────────────────────────────────────────────────────────────────
# Configuration / bitstream settings (same as fpga_top)
# ──────────────────────────────────────────────────────────────────────────────
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 85.0 [current_design]
