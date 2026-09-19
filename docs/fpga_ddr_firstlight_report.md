# FPGA DDR4/MIG First-Light Report

Date: 2026-04-22

Scope: HEAD-based ALINX/RK-XCKU5P-F / AN9134 first-light path with real
DDR4/MIG and HDMI-visible activity.  This report uses only repo-local
artifacts plus these external references:

- `/home/qwertyoruiop/FPGA/pcie_test`
- `/home/qwertyoruiop/FPGA/9.RK-XCKU5P-F/5_FactoryData/image_ku5p`

The prompt path
`/home/qwertyoruiop/FPGA/9.RK-XCKU5P-F/5_FactoryData/image_ku5p/image_ku5p`
does not exist in this checkout; the available Vivado project is one level
higher at the second path above.

## Existing DDR4/MIG Assets

`pcie_test` is the active low-risk DDR parameter reference.  The repo now
generates the matching MIG locally by Tcl and keeps generated output under
`build/ddr4_mig/`; the old pcie_test OOC DCP is retained only as a legacy
comparison/fallback artifact:

- OOC DCP: `pcie_test.runs/design_1_ddr4_0_1_synth_1/design_1_ddr4_0_1.dcp`
- MIG XCI: `pcie_test.srcs/sources_1/bd/design_1/ip/design_1_ddr4_0_1/design_1_ddr4_0_1.xci`
- generated stub: `pcie_test.gen/sources_1/bd/design_1/ip/design_1_ddr4_0_1/design_1_ddr4_0_1_stub.v`
- board XDC: `pcie_test.srcs/constrs_1/new/pcie_test.xdc`

The pcie_test MIG contract is:

- memory part `MT40A512M16LY-075`
- 32-bit DDR4 bus, DM-no-DBI, parity/alert disabled
- 750 ps memory period, 200 MHz input clock, 4:1 PHY ratio
- 256-bit AXI data, 31-bit AXI address, 1-bit AXI ID
- UI/AXI clock named as 333.25 MHz in the BD/XCI contract

The factory `image_ku5p` project corroborates the DDR4 board pin map, but it
is not drop-in compatible with the current repo DCP-stitch path:

- DDR XDC: `image_ku5p.srcs/constrs_1/new/ddr4.xdc`
- MIG XCI: `image_ku5p.srcs/sources_1/bd/design_1/ip/design_1_ddr4_0_0/design_1_ddr4_0_0.xci`
- exported bitstreams under `vitis/.../design_1_wrapper.bit`

The factory MIG differs from pcie_test:

- memory part `MT40A512M16HA-075E`
- 833 ps memory period, 4998 ps input clock period
- 256-bit AXI data, 31-bit AXI address, 5-bit AXI ID

Repo assets already present:

- `rtl/sys/ddr_ctrl.v`: SIM_MODEL RAM path plus real-MIG wrapper path.
- `rtl/vendor/design_1_ddr4_0_1_stub.v`: black-box declaration matching the
  pcie_test generated MIG stub.
- `rtl/sys/axi_ddr4_mig_bridge.v`: 128-bit/6-ID repo DDR AXI to
  256-bit/1-ID pcie_test MIG AXI shim for first-light single-beat traffic.
- `synth/gen_ddr4_mig.tcl`: repo-owned DDR4 IP generator using the pcie_test
  XCI parameters.
- `synth/ddr4.xdc`: repo DDR4 physical pins, intentionally leaving DDR4
  IOSTANDARD ownership to MIG.
- `synth/fpga_top_real_mig.xdc`: AB7/AB6 100 MHz fabric clock for real-MIG
  builds.
- `synth/vivado.tcl`: guarded `USE_REAL_MIG=1` / `REAL_FPGA_BUILD=1` flow,
  repo-generated MIG DCP stitch, and `ddr_pincheck` parse-only mode.
- `tools/check_ddr4_pcie_test.py`: static DDR reference checker.
- `Makefile` targets:
  - `make ddr4-mig-validate`
  - `make ddr4-mig`
  - `make ddr-reference-check`
  - `make fpga-first-hw-offline-preflight`
  - `make fpga-50mhz-preflight`
  - `make fpga-50mhz-jtag-bitstream-dram` or its alias `make fpga-50mhz-jtag-bitstream`

## Open Items

The repo now has a deterministic MIG generator, but generated IP output is
still an operator-produced build artifact. The remaining questions are:

- No board evidence yet for DDR calibration in this top.
- Timing closure for real-MIG/core/HDMI clocks is not signed off.
- Burst policy remains intentionally first-light only; current bridge tests
  focus on single-beat ROM/CPU traffic.
- HDMI-visible VRAM activity should still start with `HDMI_TEST_PATTERN=1`
  bars before switching to `HDMI_TEST_PATTERN=0` scanout.

## Recommended First-Light Path

Run the non-destructive checks first:

```bash
make ddr-reference-check
make ddr4-mig-validate
make fpga-first-hw-offline-preflight
make fpga-50mhz-preflight
```

These do not start a destructive full implementation.  `fpga-50mhz-preflight`
uses Vivado only for parse/elaboration/constraint checks through the mutex.

For the eventual bitstream, use only the real-MIG target:

```bash
make ddr4-mig
make fpga-50mhz-jtag-bitstream-dram
```

This expands to `USE_REAL_MIG=1 REAL_FPGA_BUILD=1` and therefore does not
define `SIM_MODEL`.  The expected isolated outputs are under
`build/vivado/`, including `fpga_top.bit` and, with probes enabled,
`fpga_top.ltx`.

Do not use `SIM_MODEL` for full synth/impl first-light.  The guarded Vivado
flow already refuses `REAL_FPGA_BUILD=1` without `USE_REAL_MIG=1`, and the
full-ROM boot image is larger than the 64 KiB SIM_MODEL DDR window.
