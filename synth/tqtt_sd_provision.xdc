# TQTT KU5P standalone SD provisioning pins.
# USER1_CLK is a 100 MHz fabric differential clock.
set_property PACKAGE_PIN T24 [get_ports fabric_clk_p]
set_property PACKAGE_PIN U24 [get_ports fabric_clk_n]
set_property IOSTANDARD DIFF_SSTL12 [get_ports {fabric_clk_p fabric_clk_n}]
create_clock -period 10.000 -name tqtt_user_clk100 [get_ports fabric_clk_p]

# P19 is the active-high board reset.
set_property -dict {PACKAGE_PIN P19 IOSTANDARD LVCMOS12} [get_ports cpu_resetn]
set_false_path -from [get_ports cpu_resetn]

set_property -dict {PACKAGE_PIN E16 IOSTANDARD LVCMOS18} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVCMOS18} [get_ports {led[1]}]
set_property -dict {PACKAGE_PIN F15 IOSTANDARD LVCMOS18} [get_ports {led[2]}]
set_property -dict {PACKAGE_PIN D15 IOSTANDARD LVCMOS18} [get_ports {led[3]}]
set_false_path -to [get_ports {led[*]}]

set_property -dict {PACKAGE_PIN AE15 IOSTANDARD LVCMOS33} [get_ports sd_clk]
set_property -dict {PACKAGE_PIN AD15 IOSTANDARD LVCMOS33} [get_ports sd_mosi]
set_property -dict {PACKAGE_PIN AF13 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports sd_miso]
set_property -dict {PACKAGE_PIN AF14 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports sd_cs_n]
set sd_spi_tx_q [get_pins -hier -filter {NAME =~ */spi_clk_reg/Q || NAME =~ */spi_mosi_reg/Q || NAME =~ */spi_cs_n_reg/Q}]
set_max_delay -datapath_only 4.000 -from $sd_spi_tx_q -to [get_ports {sd_clk sd_mosi sd_cs_n}]
set_max_delay -datapath_only 3.000 -from [get_ports sd_miso]

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 51.0 [current_design]
