# TQTT J8 SD card wired for the legacy one-bit SPI transport.
set_property -dict {PACKAGE_PIN AE15 IOSTANDARD LVCMOS33} [get_ports sd_clk]
set_property -dict {PACKAGE_PIN AD15 IOSTANDARD LVCMOS33} [get_ports sd_mosi]
set_property -dict {PACKAGE_PIN AF13 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports sd_miso]
set_property -dict {PACKAGE_PIN AF14 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports sd_cs_n]
# FPGA pad latency allocations; card round-trip timing is separate.
set sd_spi_tx_q [get_pins -hier -filter {NAME =~ */spi_clk_reg/Q || NAME =~ */spi_mosi_reg/Q || NAME =~ */spi_cs_n_reg/Q}]
set_max_delay -datapath_only 4.000 -from $sd_spi_tx_q -to [get_ports {sd_clk sd_mosi sd_cs_n}]
set_max_delay -datapath_only 3.000 -from [get_ports sd_miso]
