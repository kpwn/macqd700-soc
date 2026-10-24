# TQTT PMOD0/J1 single-port USB host bench probe.
set_property PACKAGE_PIN T24 [get_ports sys_clk_p]
set_property PACKAGE_PIN U24 [get_ports sys_clk_n]
set_property IOSTANDARD DIFF_SSTL12 [get_ports {sys_clk_p sys_clk_n}]
create_clock -name tqtt_user_clk100 -period 10.000 [get_ports sys_clk_p]

# J1 pin 1 (D10) is D+; J1 pin 3 (D9) is D-.
# The external USB-A front end supplies 5 V VBUS and 15k data pull-downs.
set_property -dict {PACKAGE_PIN D10 IOSTANDARD LVCMOS33 DRIVE 8 SLEW FAST} [get_ports usb_dp]
set_property -dict {PACKAGE_PIN D9 IOSTANDARD LVCMOS33 DRIVE 8 SLEW FAST} [get_ports usb_dm]

set_property -dict {PACKAGE_PIN E16 IOSTANDARD LVCMOS18} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVCMOS18} [get_ports {led[1]}]
set_property -dict {PACKAGE_PIN F15 IOSTANDARD LVCMOS18} [get_ports {led[2]}]
set_property -dict {PACKAGE_PIN D15 IOSTANDARD LVCMOS18} [get_ports {led[3]}]

# D+/D- are asynchronous inputs to the host's oversampling filter. There is
# no external USB clock to constrain against the FPGA's 60 MHz host clock.
set_false_path -from [get_ports {usb_dp usb_dm}]

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 51.0 [current_design]
