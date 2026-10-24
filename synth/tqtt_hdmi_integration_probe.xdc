set_property -dict {PACKAGE_PIN E18 IOSTANDARD LVCMOS18} [get_ports fabric_clk_50]
create_clock -period 20.000 -name tqtt_fabric_clk50 [get_ports fabric_clk_50]
