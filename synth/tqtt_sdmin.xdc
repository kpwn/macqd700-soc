# TQTT-KU5P standalone read-only SD/CRC harness.
# Pin locations follow the TQTT XCKU5P schematic (clock/SD pages 4 and 6).
# No adjustable-rail reset, LED, HDMI or Ethernet pads are driven here.

# USER1_CLK: differential 100 MHz clock in bank 65 (1.2 V).
set_property PACKAGE_PIN T24 [get_ports sys_clk_p]
set_property PACKAGE_PIN U24 [get_ports sys_clk_n]
set_property IOSTANDARD DIFF_SSTL12 [get_ports {sys_clk_p sys_clk_n}]
create_clock -period 10.000 -name tqtt_user_clk100 [get_ports sys_clk_p]

# SD J8 in bank 84 (3.3 V).  DAT3 is chip select in SPI mode.
set_property PACKAGE_PIN AE15 [get_ports sd_clk]
set_property PACKAGE_PIN AD15 [get_ports sd_mosi]
set_property PACKAGE_PIN AF13 [get_ports sd_miso]
set_property PACKAGE_PIN AF14 [get_ports sd_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports {sd_clk sd_mosi sd_miso sd_cs_n}]
set_property PULLUP TRUE [get_ports sd_miso]
set_property PULLUP TRUE [get_ports sd_cs_n]

# FPGA-side timing allocations.  Card timing needs a hardware sweep.
set sd_spi_tx_q [get_pins -hier -filter {NAME =~ */spi_clk_reg/Q || NAME =~ */spi_mosi_reg/Q || NAME =~ */spi_cs_n_reg/Q}]
set_max_delay -datapath_only 4.000 -from $sd_spi_tx_q -to [get_ports {sd_clk sd_mosi sd_cs_n}]
set_max_delay -datapath_only 3.000 -from [get_ports sd_miso]

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 51.0 [current_design]
