# Optional PMOD0/J1 USB HID host, matching the standalone probe.
# J1 pin 1 (D10) is D+; J1 pin 3 (D9) is D-.
# The external USB-A front end supplies current-limited 5 V VBUS, 15k data
# pull-downs, series resistors, and protection.
set_property -dict {PACKAGE_PIN D10 IOSTANDARD LVCMOS33 DRIVE 8 SLEW FAST} [get_ports usb_dp]
set_property -dict {PACKAGE_PIN D9 IOSTANDARD LVCMOS33 DRIVE 8 SLEW FAST} [get_ports usb_dm]

# The USB data inputs have no external clock relationship to the 60 MHz
# oversampling clock. The host samples them asynchronously.
set_false_path -from [get_ports {usb_dp usb_dm}]
