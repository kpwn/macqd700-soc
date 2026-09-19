# fpga_top_real_mig.xdc -- extra board constraints only used by USE_REAL_MIG=1.
#
# This keeps fpga_top.xdc free of Tcl conditionals that `read_xdc` rejects.
# The AB7/AB6 pair matches the known-good ~/FPGA/pcie_test utility refclk:
# AB7 = MGTREFCLK0P_224, AB6 = MGTREFCLK0N_224.  fpga_top consumes it through
# IBUFDS_GTE4 and sends ODIV2 through BUFG_GT before it reaches fabric
# MMCM/clock-divider loads.  ODIV2 is not directly routable to ordinary
# fabric clock loads on KU5P.

set_property PACKAGE_PIN AB7 [get_ports fabric_clk_p]
set_property PACKAGE_PIN AB6 [get_ports fabric_clk_n]
set_property IO_BUFFER_TYPE NONE [get_ports {fabric_clk_p fabric_clk_n}]
set_property CLOCK_BUFFER_TYPE NONE [get_ports {fabric_clk_p fabric_clk_n}]
create_clock -period 10.000 -name fabric_clk100 [get_ports fabric_clk_p]
