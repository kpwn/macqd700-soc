# Standalone native SD probe; TQTT J8 is fixed 3.3 V.
set_property PACKAGE_PIN T24 [get_ports sys_clk_p]
set_property PACKAGE_PIN U24 [get_ports sys_clk_n]
set_property IOSTANDARD DIFF_SSTL12 [get_ports {sys_clk_p sys_clk_n}]
create_clock -period 10.000 -name tqtt_user_clk100 [get_ports sys_clk_p]

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
