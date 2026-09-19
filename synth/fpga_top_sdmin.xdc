# fpga_top_sdmin.xdc — pin constraints for the standalone SD/CRC test
# harness (rtl/soc/fpga_top_sdmin.v). Pin assignments copied verbatim from
# synth/fpga_top.xdc (same physical board) — only the subset this minimal
# design actually uses: the DDR4-reference clock pair (repurposed here as
# a plain fabric clock, no MIG), the reset button, LEDs, and SD SPI pins.
#
# Part: xcku5p-ffvb676-2-i

# ── Clock (DDR4-reference pair, reused as plain fabric clock — no MIG) ──
set_property PACKAGE_PIN  T24   [get_ports sys_clk_p]
set_property PACKAGE_PIN  U24   [get_ports sys_clk_n]
set_property IOSTANDARD   DIFF_SSTL12 [get_ports sys_clk_p]
set_property IOSTANDARD   DIFF_SSTL12 [get_ports sys_clk_n]
create_clock -period 5.000 -name sysclk200 [get_ports sys_clk_p]

# ── Reset (active-low push-button) ──────────────────────────────────────
set_property PACKAGE_PIN T19       [get_ports cpu_resetn]
set_property IOSTANDARD  LVCMOS12  [get_ports cpu_resetn]
set_property PULLUP      TRUE      [get_ports cpu_resetn]
set_false_path -from [get_ports cpu_resetn]

# ── LEDs ─────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN H9  [get_ports {led[0]}]
set_property PACKAGE_PIN J9  [get_ports {led[1]}]
set_property PACKAGE_PIN G11 [get_ports {led[2]}]
set_property PACKAGE_PIN H11 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[*]}]

# ── SD card SPI pins ──────────────────────────────────────────────────────
set_property PACKAGE_PIN Y15  [get_ports sd_clk]
set_property PACKAGE_PIN AA15 [get_ports sd_mosi]
set_property PACKAGE_PIN AB14 [get_ports sd_miso]
set_property PACKAGE_PIN AB15 [get_ports sd_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports {sd_clk sd_mosi sd_miso sd_cs_n}]
set_property PULLUP true [get_ports sd_miso]
# Match the full SoC's bounded FPGA-side SPI latency allocation.
# External card/board margin must still be qualified by the read sweep.
set sd_spi_tx_q [get_pins -hier -filter {NAME =~ */spi_clk_reg/Q || NAME =~ */spi_mosi_reg/Q || NAME =~ */spi_cs_n_reg/Q}]
set_max_delay -datapath_only 4.000 -from $sd_spi_tx_q -to [get_ports {sd_clk sd_mosi sd_cs_n}]
set_max_delay -datapath_only 3.000 -from [get_ports sd_miso]

# ── Config / bitstream settings (copied verbatim from synth/ku5p.xdc) ───
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 85.0 [current_design]
