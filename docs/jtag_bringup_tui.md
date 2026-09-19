# JTAG Bring-Up Terminal Dashboard

`tools/jtag_bringup_tui.py` is a thin terminal wrapper around the existing
Vivado helpers:

- `synth/program_fpga.sh` for safe bitstream programming.
- `synth/jtag_bringup.tcl` for VIO snapshots, VIO control, JTAG AXI reads and
  writes, and ROM loading.

It does not run synthesis or implementation.  It expects a bitstream built with
`ENABLE_VIO=1 ENABLE_JTAG_AXI=1`, for example the output from:

```bash
make fpga-100mhz-jtag-bitstream-dram
```

Current default artifacts:

```text
build/vivado/fpga_top.bit
build/vivado/fpga_top.ltx
```

## Program The Current Build

Dry-run is the default and only prints the Vivado command:

```bash
tools/jtag_bringup_tui.py program
```

Program `build/vivado/fpga_top.bit` and attach
`build/vivado/fpga_top.ltx`:

```bash
tools/jtag_bringup_tui.py program --go
```

If the matching `.ltx` is missing, the program flow now fails instead of
silently dropping back to bitstream-only programming.  Use `--no-vio` only for
an intentional non-debug image.

Program a specific debug build from another output directory:

```bash
tools/jtag_bringup_tui.py program --go \
  --bit build/vivado_100mhz_debug/fpga_top.bit \
  --ltx build/vivado_100mhz_debug/fpga_top.ltx
```

Allow a bitstream older than 24 hours:

```bash
tools/jtag_bringup_tui.py program --go --allow-stale
```

Program a bitstream intentionally built without VIO:

```bash
tools/jtag_bringup_tui.py program --go --no-vio --bit build/vivado/fpga_top.bit
```

Use an explicit Vivado install or select one cable/device:

```bash
tools/jtag_bringup_tui.py --vivado /tools/Vivado/2025.2/Vivado/bin/vivado \
  --target-re 'localhost.*xilinx_tcf' --device-re 'xcku5p' program --go
```

## Dashboard

One decoded VIO snapshot:

```bash
tools/jtag_bringup_tui.py status
```

Alias that makes the no-reprogramming behavior explicit:

```bash
tools/jtag_bringup_tui.py snapshot
```

Attach the LTX from a specific build that is already programmed, without
reprogramming:

```bash
tools/jtag_bringup_tui.py \
  --ltx build/vivado_100mhz_debug/fpga_top.ltx \
  snapshot
```

Refresh-loop dashboard every five seconds:

```bash
tools/jtag_bringup_tui.py dashboard --interval 5
```

Run a bounded dashboard for lab logs:

```bash
tools/jtag_bringup_tui.py dashboard --interval 2 --count 10
```

The dashboard decodes:

- reset and boot state: `cpu_resetn`, `core_rst`, `ddr_cal_done`,
  `boot_rom_ready`, HDMI I2C, framebuffer underflow, and the current
  boot/core hold state.
- core progress: last committed PC and committed instruction counter.
- DDR activity: low read-beat counter, `s0_wready`, and AXI handshake bundle.
- HDMI/VRAM state: MMCM lock, AL9134 reset/I2C, sync/DE/HS/VS, RGB, scan
  mode, VTG counters, VRAM read address, fb-reader request/response/miss
  counters, VRAM/DAFB write counters, smoke status, and DAFB base/stride/BPP.
- AXI / error state: last summary response code plus a sticky source/status
  flag for DDR, IO, DMA, VRAM, or boot-side errors when the fabric reports
  one.
- JTAG boot controls: bypass SD, release CPU, force CPU reset.

The snapshot path is resilient to either positional VIO probes
(`probe_in0`, `probe_in1`, ...) or Vivado-synthesized net names such as
`hdmi_mmcm_locked`, `dbg_pc`, `vio_rst_bundle`, `s0_awvalid`, and
`vio_boot_ctrl`.  Missing probes are reported as unknown rather than failing
the whole dashboard.

If the board reports HDMI lock/I2C activity but `ddr_cal_done=0` and
`boot_rom_ready=0`, the dashboard calls out that HDMI is alive while MIG/DDR
calibration is blocking CPU release.  It also surfaces Vivado
`Labtools 27-3410 Calibration Failed` warnings when they appear in the Tcl
refresh output.

## Control And Memory Access

Mutating commands default to dry-run.  Add `--go` after the subcommand to
perform the write/control action.

Hold SD boot off and keep the CPU reset while loading or patching memory:

```bash
tools/jtag_bringup_tui.py hold
tools/jtag_bringup_tui.py hold --go
```

Release the CPU from a JTAG-loaded ROM image:

```bash
tools/jtag_bringup_tui.py release --go
```

Return control to the SD boot path on the next reset:

```bash
tools/jtag_bringup_tui.py sd-boot --go
```

Raw VIO control:

```bash
tools/jtag_bringup_tui.py vio-get
tools/jtag_bringup_tui.py vio-set 0x5
tools/jtag_bringup_tui.py vio-set 0x5 --go
```

Single-word JTAG AXI transactions:

```bash
tools/jtag_bringup_tui.py axi-read 0x40000000
tools/jtag_bringup_tui.py axi-write 0x40000000 0x4ef9002a
tools/jtag_bringup_tui.py axi-write 0x40000000 0x4ef9002a --go
```

Programmable CPU holds through the JTAG-to-AXI `debug_ctrl` window:

```bash
tools/jtag_bringup_tui.py debug-halt-status
tools/jtag_bringup_tui.py debug-reset-halt --go
tools/jtag_bringup_tui.py debug-halt-after 1000000 --go
tools/jtag_bringup_tui.py debug-break-pc 0x40801234 --go
tools/jtag_bringup_tui.py debug-halt-exc --go
tools/jtag_bringup_tui.py debug-clear-halt --go
tools/jtag_bringup_tui.py debug-step --go
tools/jtag_bringup_tui.py debug-step --halt-first --go
```

`debug-step` expects the core to already be halted.  Add `--halt-first` to
request a halt before pulsing the step bit when the CPU is running.

`debug-halt-after` defaults to `HALT_CTL=0x5` (enable halt-after and clear
any old auto-halt latch). `debug-break-pc` defaults to `HALT_CTL=0x6`
(enable PC breakpoint and clear the latch). `debug-halt-exc` defaults to
exception vector 4, the 68k illegal-instruction vector, and `HALT_CTL=0x44`.
Pass `--halt-ctl 0x47` when all three conditions should remain enabled after
reprogramming one of them.

Paint a VRAM pattern through the current scanout placement without touching
DAFB registers.  This is useful on bitstreams where JTAG can reach the
`0xF9000000` VRAM aperture but DAFB `0xF9800000` is only intercepted on the
CPU path:

```bash
tools/jtag_bringup_tui.py vram-wrap-poke --go --hold-cpu
tools/jtag_bringup_tui.py vram-wrap-poke --go --hold-cpu 0xf0010 0x40000 4 256
```

The command wraps each computed source-row address into the 1 MB VRAM aperture,
so it can paint a visible pattern even when the live DAFB base/stride are
wrong but stable.

Load a ROM image into the DRAM-backed ROM window.  This calls the existing Tcl
`rom-load`, which sets `probe_out0=0x5`, writes 32-bit words, then sets
`probe_out0=0x3`:

```bash
tools/jtag_bringup_tui.py rom-load files/420dbff3.rom 0x40000000
tools/jtag_bringup_tui.py rom-load files/420dbff3.rom 0x40000000 --go
```

## Direct Tcl Snapshot Mode

The Python tool uses this machine-readable Tcl path:

```bash
vivado -nojournal -nolog -mode batch -source synth/jtag_bringup.tcl \
  -tclargs snapshot-machine
```

With an explicit LTX for an already-programmed image:

```bash
LTX_FILE=build/vivado_100mhz_debug/fpga_top.ltx \
vivado -nojournal -nolog -mode batch -source synth/jtag_bringup.tcl \
  -tclargs snapshot-machine
```

The older human dashboard still works:

```bash
vivado -nojournal -nolog -mode batch -source synth/jtag_bringup.tcl \
  -tclargs dashboard
```

## OpenOCD Backend

The Python wrapper can also drive the debug_ctrl and AXI paths through
OpenOCD instead of Vivado batch mode:

```bash
tools/jtag_bringup_tui.py --backend openocd \
  --openocd-bin /usr/bin/openocd \
  --openocd-cfg board/m68k-ooo-debug.cfg \
  debug-halt-status
```

That backend is intentionally narrow in scope for now:

- `status`, `dashboard`, `axi-read`, `axi-write`, `debug-halt-status`,
  `debug-reset-halt`, `debug-halt-after`, `debug-break-pc`,
  `debug-halt-exc`, `debug-clear-halt`, and `debug-step` go through
  OpenOCD.
- VIO-specific commands such as `program`, `vio-set`, `hold`, `release`,
  `sd-boot`, `video-poke`, `vram-wrap-poke`, and `rom-load` remain
  Vivado-only.

The backend expects one or more OpenOCD config files passed with
`--openocd-cfg`, or `OPENOCD_CFG` in the environment.  The config stack
should already define the target chain and leave the desired CPU target
current before the batch commands run.  In practice that usually means:

- an interface cfg for your adapter or cable;
- a board cfg that sources the interface cfg and the relevant target cfg;
- target memory access enabled for the debug_ctrl and AXI windows used by
  `read_memory` / `write_memory`.

For multi-target setups, pass `--openocd-target <name>` to select the
current target explicitly after `init`.

## Limitations And Future Hooks

- The tool launches Vivado once per operation.  That is slow but keeps sessions
  simple and avoids persistent Tcl state during first-board bring-up.
- The OpenOCD backend is transport-focused and assumes the board cfg makes
  the debug_ctrl and AXI windows reachable as ordinary target memory.
- The OpenOCD path is batch-oriented today; a persistent session would be the
  next step if we want to squeeze more latency out of repeated reads.
- The VIO map exposes useful first-light status, but not a full CPU register
  file, exception vector/state, cache/MMU state, or a PC trace FIFO.
- JTAG AXI is single-word oriented.  Bulk ROM load works through Tcl loops, but
  faster burst loading needs the broader DDR/JTAG burst-support work.
- Probe values sampled from the pixel clock domain are asynchronous into the
  VIO clock and should be treated as smoke indicators, not cycle-accurate
  traces.
