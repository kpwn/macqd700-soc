# TQTT J8 SD card wired for four-bit native mode.
set_property PACKAGE_PIN AE15 [get_ports sd_clk]
set_property PACKAGE_PIN AD15 [get_ports sd_cmd]
set_property PACKAGE_PIN AF13 [get_ports {sd_dat[0]}]
set_property PACKAGE_PIN AE13 [get_ports {sd_dat[1]}]
set_property PACKAGE_PIN AF15 [get_ports {sd_dat[2]}]
set_property PACKAGE_PIN AF14 [get_ports {sd_dat[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sd_clk sd_cmd sd_dat[*]}]
set_property PULLUP TRUE [get_ports {sd_cmd sd_dat[*]}]
