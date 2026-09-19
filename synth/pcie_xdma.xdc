# PCIe/XDMA host-access shell constraints.
#
# This file is read only when ENABLE_PCIE_XDMA=1.  Pin choices match the
# known-good /offlinenas/share/FPGA/pcie_test project:
#   - AB7/AB6 fabric_clk_p/n is the 100 MHz PCIe MGT reference clock.
#   - GTY Quad 224 carries the Gen3 x4 PCIe lanes.
#   - T19 is the active-low PCIe PERST# input.

set_property PACKAGE_PIN AF7 [get_ports {pcie_txp[3]}]
set_property PACKAGE_PIN AE9 [get_ports {pcie_txp[2]}]
set_property PACKAGE_PIN AD7 [get_ports {pcie_txp[1]}]
set_property PACKAGE_PIN AC5 [get_ports {pcie_txp[0]}]

# PERST# (T19) is the same physical pin as cpu_resetn (fpga_top.xdc owns
# the LOC/IOSTANDARD).  fpga_top has no separate pcie_perstn port — the
# XDMA sys_rst_n is driven from cpu_resetn internally.  Constraining a
# second port on T19 loses the LOC (Vivado 12-1411) and then fails
# bitstream DRC UCIO-1 ("Problem ports: pcie_perstn").
