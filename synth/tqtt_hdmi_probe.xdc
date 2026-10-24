# TQTT bank 65 is 1.2 V: use DIFF_SSTL12, not the vendor LVDS example.
set_property PACKAGE_PIN T24 [get_ports sys_clk_p]
set_property PACKAGE_PIN U24 [get_ports sys_clk_n]
set_property IOSTANDARD DIFF_SSTL12 [get_ports {sys_clk_p sys_clk_n}]
create_clock -period 10.000 -name tqtt_user_clk100 [get_ports sys_clk_p]

set_property PACKAGE_PIN T25 [get_ports tmds_clk_p]
set_property PACKAGE_PIN U25 [get_ports tmds_clk_n]
set_property PACKAGE_PIN N24 [get_ports {tmds_p[0]}]
set_property PACKAGE_PIN P24 [get_ports {tmds_n[0]}]
set_property PACKAGE_PIN V24 [get_ports {tmds_p[1]}]
set_property PACKAGE_PIN W24 [get_ports {tmds_n[1]}]
set_property PACKAGE_PIN V23 [get_ports {tmds_p[2]}]
set_property PACKAGE_PIN W23 [get_ports {tmds_n[2]}]
set_property IOSTANDARD DIFF_SSTL12 [get_ports {tmds_clk_p tmds_clk_n tmds_p[*] tmds_n[*]}]

set_property PACKAGE_PIN Y16 [get_ports hdmi_out_en]
set_property IOSTANDARD LVCMOS33 [get_ports hdmi_out_en]

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
