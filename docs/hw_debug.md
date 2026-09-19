# Hardware Debug Controls (VIO + JTAG-AXI)

This document is the canonical reference for the live JTAG/VIO debug
controls exposed by `fpga_top.v` (the synth target).  It complements
`docs/jtag_bringup_tui.md` (which covers the TUI command surface) and
`docs/debug_pcie.md` (PCIe XDMA host-debug master).

> Sim does not exercise these controls — `make sim` builds `mac_top.v`,
> not `fpga_top.v`.  Unit tbs that exercise the relevant inline FFs
> live next to the modules they touch (e.g. `make tb-debug-full-reset`
> for the overlay re-arm gate).

## probe_out0 contract (4 bits, task #256)

`fpga_top` exposes a single VIO output probe `probe_out0` driving a
4-bit boot/debug control bus `vio_boot_ctrl`:

| Bit | Wire                    | Effect                                                     |
|----:|-------------------------|------------------------------------------------------------|
| 0   | `jtag_boot_bypass`      | Hold `boot_fsm` in reset; ROM is not loaded from SD.       |
| 1   | `jtag_boot_release`     | When bit 0 is set, gates `boot_rom_ready` high so the CPU is released after the host has populated DDR via JTAG-AXI. |
| 2   | `jtag_cpu_hold`         | Force `cpu_rst` high (CPU-only halt).  Preserves overlay + VIA1 + DDR state.  Use this for halt-after / single-step / inspect flows. |
| 3   | `jtag_debug_full_reset` | **Cold-boot equivalent** for the CPU-side fabric: drives `soc_full_rst` which re-arms `reset_overlay_active_q` AND resets VIA1 (so `ORB[3]`, the overlay-control bit, returns to its post-reset value).  Releasing this bit lets the CPU resume from the reset PC (vec-0 fetch under `FETCH_RESET_VECTORS=1`) with the low-memory ROM alias live again. |

### Why bit 2 vs bit 3 are distinct

- Bit 2 (`jtag_cpu_hold`) is intentionally minimal: it gates `cpu_rst`
  but leaves the overlay flag, VIA1 ORB, DAFB regs, etc. intact.  This
  is the path used by `debug-run-from-reset-halt-after`, single-step,
  and "halt-and-inspect" flows where we want the CPU's architectural
  state preserved across the halt boundary.
- Bit 3 (`jtag_debug_full_reset`, task #256) is the path you take when
  you actually want a cold-boot semantic without re-programming the
  bitstream.  After a prior boot cleared the ROM overlay (via1 ORB[3]
  was driven 0 by ROM software), the low-mem ROM alias is gone.
  Pulsing bit 2 alone leaves it gone — the CPU resumes fetching from
  whatever low memory contains, which is usually zeros, and the core
  immediately decodes garbage at PC=0.  Pulsing bit 3 instead re-arms
  the alias AND resets VIA1 so ORB[3] returns to its overlay-asserted
  reset value, and the CPU's next fetch lands on the ROM image.

### What bit 3 does **not** touch

- DDR4 / MIG.  In-flight DRAM is preserved.
- AXI xbar masters/slaves other than the CPU side.  HDMI scan-out,
  XDMA, and JTAG-AXI continue to operate.
- All Mac peripherals other than VIA1 (VIA2, SCC, SCSI, ASC, RTC, DAFB,
  DMA controller).

If you want a true board-level cold reset, drive `cpu_resetn` low (or
press the board reset button); that resets everything.

## Invocation

### TUI

```bash
tools/jtag_bringup_tui.py debug-full-reset --go
tools/jtag_bringup_tui.py debug-full-reset --go --hold-ms 50
tools/jtag_bringup_tui.py debug-full-reset --go --no-release   # leave bit3 asserted
```

`debug-full-reset --go` defaults to a 100 ms hold of bit 3 followed by
a release to `vio_boot_ctrl=0`.  Use `--no-release` when you want to
patch ROM/DRAM via JTAG-AXI before the CPU starts; finalize with
`tools/jtag_bringup_tui.py vio-set 0x0 --go` (or `release` / `sd-boot`
as appropriate).

### Raw Vivado TCL

```tcl
# direct
source synth/jtag_bringup.tcl debug-full-reset 100 1
# or via vio-set: assert/deassert manually
source synth/jtag_bringup.tcl vio-set 0x8     ;# bit3 = 1
after 100
source synth/jtag_bringup.tcl vio-set 0x0     ;# release
```

## HW bring-up checklist

After programming a `fpga_top.bit` built with this fix:

1. `tools/jtag_bringup_tui.py status --go` — confirm `dbg_pc=0x0000_002A`
   on cold boot, `reset_overlay_active_q=1`.
2. After ROM has cleared the overlay (a few thousand cycles into
   bring-up), `via1_overlay_bit=0` and `reset_overlay_active_q=0`.
3. `tools/jtag_bringup_tui.py debug-full-reset --go` — expect
   `dbg_pc` to return to `0x0000_002A` and `via1_overlay_bit=1`,
   `reset_overlay_active_q=1` on the next dashboard sample.
4. The CPU should resume normal Quadra-700 ROM execution from there
   without reprogramming the FPGA.

## Regression coverage

`make tb-debug-full-reset` exercises the inline overlay re-arm FF +
`soc_full_rst` aggregate + VIA1 ORB[3] reset path.  Six directed
scenarios:

- cold reset asserts overlay
- ROM clears overlay drops the alias
- `jtag_cpu_hold` alone preserves overlay state (regression)
- `jtag_debug_full_reset` re-arms overlay AND resets VIA1 (the fix)
- post-release the CPU resumes with overlay live
- post-debug-full-reset ROM can re-clear the overlay normally
- cpu-hold + full-reset combined behaves as full reset

## See also

- `synth/vivado.tcl` — `gen_debug_vio_ip` configures the VIO IP
  (`probe_out0` width is 4 bits as of probe-map v8).
- `synth/jtag_bringup.tcl` — `vio-set`, `debug-full-reset`,
  `vio-get` decode bit 3.
- `tools/jtag_bringup_tui.py` — TUI `debug-full-reset` subcommand and
  dashboard `jtag_full_dbg_rst=` field.
- `rtl/fpga_top_clocks.vh` — `vio_boot_ctrl` decode + `soc_full_rst`
  aggregate.
- `rtl/fpga_top_cpu.vh` — `reset_overlay_active_q` re-arm.
- `rtl/mac/via1.v` — ORB[3] reset value (`orb <= 8'h80` on `rst`).
- `tb/tb_debug_full_reset.v` + `tb/tb_debug_full_reset.cpp` — unit tb.
