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

# TQTT J8 SD card wired for four-bit native mode.
set_property PACKAGE_PIN AE15 [get_ports sd_clk]
set_property PACKAGE_PIN AD15 [get_ports sd_cmd]
set_property PACKAGE_PIN AF13 [get_ports {sd_dat[0]}]
set_property PACKAGE_PIN AE13 [get_ports {sd_dat[1]}]
set_property PACKAGE_PIN AF15 [get_ports {sd_dat[2]}]
set_property PACKAGE_PIN AF14 [get_ports {sd_dat[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sd_clk sd_cmd sd_dat[*]}]
set_property PULLUP TRUE [get_ports {sd_cmd sd_dat[*]}]

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 51.0 [current_design]
