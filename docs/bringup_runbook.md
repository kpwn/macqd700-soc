# bringup_runbook.md — take the KU5P from synth-complete → mac logo on HDMI

Single-page runbook. Pre-reqs: working Vivado 2025.2, hw_server able to see the
KU5P JTAG cable, HDMI monitor attached, SD card inserted. Host (10.200.0.11)
is LOCAL to the board — no SSH needed.

## 0. Before you start

1. Bitstream baseline: `ls -la build/vivado/fpga_top.bit` should be minutes-old,
   not days. Stale bitstreams are the #1 cause of "I thought I fixed it in
   RTL but the board still shows the old bug."  `synth/program_fpga.sh`
   refuses anything >24h old unless you pass `--allow-stale`.
2. Confirm `ENABLE_VIO=1` was set when synth ran if you want live probes.
   Check for `build/vivado/fpga_top.ltx` — if missing, VIO was off.
   `synth/program_fpga.sh` now refuses to program in debug mode without that
   matching `.ltx`; use `--no-vio` only for an intentional non-debug image.
3. Full-ROM bitstreams must use real DRAM.  The `ddr_ctrl` SIM_MODEL RAM is
   only 64 KiB, so Vivado now blocks SIM_MODEL synth/impl when
   `BOOT_ROM_SECTORS` would exceed that capacity.  Use
   `make fpga-50mhz-jtag-bitstream-dram` for a DRAM-backed ROM image.  For
   the ALINX AN9134 / KU5P board, `make ddr-pcie-test-check` verifies the
   known-good pcie_test DDR4 facts: `MT40A512M16LY-075`, 32-bit bus, 750 ps
   memory period, 200 MHz input clock, 333.25 MHz UI/AXI clock, and a
   256-bit/31-address/1-ID MIG AXI boundary.  The real-MIG path stitches the
   pcie_test OOC DCP from `PCIE_TEST_DIR`, bridges repo DDR AXI to the MIG
   contract, and crosses core clock to MIG UI clock through `axi_async_bridge`.
4. Canonical 100 MHz preflight: `make fpga-first-hw-preflight` runs FPGA-top
   lint, divider/reset tests, framebuffer first-light simulations, clock-report,
   and DDR pincheck without writing a synthesized netlist, starting place/route,
   or generating a bitstream.  The clock-report step should show the fixed
   100 MHz board fabric clock, the active 100 MHz core clock, and any generated
   clocks that Vivado can resolve.
   `TARGET_FREQ_MHZ` is only a timing budget; it does not create a board
   clock.  `make fpga-50mhz-preflight` remains available for legacy lower-clock
   experiments.
5. Canonical hardware build path: use `make fpga-100mhz-jtag-bitstream-dram`
   for a debug-capable real-MIG bitstream.  Plain `make impl` is a generic
   local flow; `REAL_FPGA_BUILD=1` full impls now fail unless they also enable
   VIO and one host debug path (`ENABLE_JTAG_AXI=1` or `ENABLE_PCIE_XDMA=1`),
   or you explicitly set `ALLOW_UNDEBUGGABLE_FPGA_BUILD=1`.

## 0a. Build the canonical 100 MHz debug bitstream

Use this for current board work.  It uses the repo-owned DDR4 MIG artifact,
enables VIO and JTAG AXI debug by default, and emits a bitstream, matching
`.ltx`, and a small build manifest that the programming flow checks.

```
out="$PWD/build/vivado_100mhz_debug"
mkdir -p "$out"
MAKEFLAGS='-j1' VIVADO_IMPL_DIR="$out" \
  make fpga-100mhz-jtag-bitstream-dram \
  2>&1 | tee "$out/run.log"
```

Important details:

- `VIVADO_IMPL_DIR=...` can now be passed either as an environment variable or
  as a make command-line variable; the Makefile default no longer hides the
  environment override.
- `make impl` uses the machine-wide Vivado mutex at
  `/var/tmp/m68k-ooo-vivado.lock`; do not start a second synth/impl job while
  this is running.
- Expected outputs are:
  `build/vivado_100mhz_debug/fpga_top.bit`,
  `build/vivado_100mhz_debug/fpga_top.ltx`,
  `build/vivado_100mhz_debug/fpga_top.buildinfo`,
  `build/vivado_100mhz_debug/timing_summary.rpt`, and the detailed reports
  under `build/vivado_100mhz_debug/reports/`.
- The canonical target self-checks those debug artifacts after implementation.
- To force the full framebuffer path instead of the direct HDMI test pattern,
  add `FPGA_100MHZ_HDMI_TEST_PATTERN=0` to the command above.

After implementation, check routed timing.  This is the signoff report for the
bitstream:

```
make VIVADO_IMPL_DIR="$PWD/build/vivado_100mhz_debug" timing
grep -A 12 "Design Timing Summary" \
  build/vivado_100mhz_debug/timing_summary.rpt
```

`reports/timing_synth.rpt` is useful while debugging pre-route critical paths,
but it is not the final pass/fail artifact; pre-route video/MIG paths can be
negative and still close after place/route.

Flash that exact build with a dry run first, then program:

```
synth/program_fpga.sh \
  --bit build/vivado_100mhz_debug/fpga_top.bit \
  --ltx build/vivado_100mhz_debug/fpga_top.ltx

synth/program_fpga.sh --go \
  --bit build/vivado_100mhz_debug/fpga_top.bit \
  --ltx build/vivado_100mhz_debug/fpga_top.ltx
```

If VIO was deliberately disabled, add `--no-vio`.  Otherwise keep the `.ltx`
attached so the dashboard and JTAG AXI tools can inspect DDR calibration, CPU
reset/boot state, HDMI counters, VRAM/DAFB write counters, and JTAG ROM-load
controls.

## 1. Provision the SD card (once per Q700 ROM change)

First check the positional raw-block layout:

```
PYTHONPATH=tools python3 -m m68kctl sd check-layout \
  --rom files/420dbff3.rom \
  --hdd /path/to/q700-disk.hdv
```

For an offline SD-card image to write with `dd` or another raw block writer:

```
PYTHONPATH=tools python3 -m m68kctl sd make-image \
  --rom files/420dbff3.rom \
  --hdd /path/to/q700-disk.hdv \
  -o /tmp/q700-firstlight.sdimg
```

This does not create or inspect a filesystem. It writes the ROM at byte 0,
zero-pads through the 4 MiB reserved window, and places raw SCSI HDD byte 0 at
SD LBA 8192 / byte offset `0x00400000`.

```
m68kctl sd upload   files/420dbff3.rom --progress
m68kctl sd verify   files/420dbff3.rom --progress
```

When you want a fast host-side snapshot of RAM/ROM for checkpointing or
offline diffing, use:

```
m68kctl checkpoint dump --output-dir /tmp/checkpoint \
  --region ram 0x00001000 0x2000 \
  --region rom 0x40002000 0x1000
```

Image is a POSITIONAL arg (not `--image`). Default LBA is 0, which is exactly
where `boot_fsm` starts reading. The current FPGA boot copy is the 1 MiB
Q700 ROM: sectors 0..2047 (2048 × 512 B) into DDR at `0x4000_0000`.
The SD image still reserves sectors 0..8191 as the boot/provisioning
window, and the raw SCSI disk still starts at LBA 8192. The 1 MiB CMD18
copy should complete in well under 0.3 s at 25 MHz SPI once SD init is
done.

For a small-ROM/checkerboard hardware smoke image, keep the image at LBA 0
but build the bitstream with `BOOT_ROM_SECTORS=<sector-count>`.  The default
CPU reset PC is still `0x4000_002A`, so the small image must contain reachable
code at that offset or branch through it.

## 2. Program the FPGA

Dry-run first (default):
```
synth/program_fpga.sh
```
Prints the Vivado command and bitstream age. Abort if stale.  If the selected
image is meant to be debug-capable, the script also requires the matching
`.ltx`; it no longer silently falls back to a bitstream-only program path.

Check JTAG visibility without programming:
```
make jtag-discover
```

For real:
```
synth/program_fpga.sh --go
```

If the bitstream was built without VIO probes, use `synth/program_fpga.sh
--go --no-vio`; the script will use the program-only Vivado path instead of
requiring `build/vivado/fpga_top.ltx`.

To attach VIO probes and dump a dashboard snapshot:
```
vivado -mode tcl -source synth/vio_dashboard.tcl \
       -tclargs build/vivado/fpga_top.bit build/vivado/fpga_top.ltx
```

For a decoded refresh-loop terminal dashboard and safer dry-run wrappers around
VIO/JTAG AXI controls, use:
```
tools/jtag_bringup_tui.py status
tools/jtag_bringup_tui.py dashboard --interval 5
```

See [`docs/jtag_bringup_tui.md`](jtag_bringup_tui.md) for programming,
control, ROM-load, and AXI read/write commands.

## 3. Interpret LEDs (rtl/fpga_top.v:1919-1922)

| LED  | Signal              | Expected post-boot                              |
|------|---------------------|-------------------------------------------------|
| 0    | `core_rst`          | 0 (out of reset)                                |
| 1    | `ddr_cal_done`      | 1 after SIM_MODEL delay or real DDR4 calibration |
| 2    | `boot_rom_ready`    | 1 (SD-loaded ROM, or JTAG ROM-load release). Gates CPU reset. |
| 3    | `hdmi_mmcm_locked`  | 1 (pixel clock locked, video alive)             |

If LED[2] stays low ≥2 s: SD provision missing, or boot_fsm is stuck.
If LED[3] stays low: HDMI PLL didn't lock — check cable + monitor.

The normal JTAG bitstream also supports ROM patch iteration without rebuilding.
Set VIO `probe_out0[0]=1` to hold `boot_fsm` in reset and make CPU release
depend on JTAG control.  Write or patch the DRAM-backed ROM window at
`0x40000000` through the Xilinx JTAG-to-AXI master on xbar M1 using single
32-bit transactions.  Then set `probe_out0=0x3`
(`bypass_sd_boot=1`, `release_cpu=1`, `force_cpu_reset=0`) to let the core
fetch from the loaded image.  Set bit 2 to force the CPU back into reset before
another patch.

Batch helpers for those operations:

```
vivado -nojournal -nolog -mode batch -source synth/jtag_bringup.tcl \
  -tclargs vio-set 0x5
vivado -nojournal -nolog -mode batch -source synth/jtag_bringup.tcl \
  -tclargs axi-read 0x40000000
vivado -nojournal -nolog -mode batch -source synth/jtag_bringup.tcl \
  -tclargs axi-write 0x40000000 0x4ef9002a
vivado -nojournal -nolog -mode batch -source synth/jtag_bringup.tcl \
  -tclargs rom-load files/420dbff3.rom 0x40000000
```

`rom-load` sets `probe_out0=0x5` first (`bypass_sd_boot=1`,
`force_cpu_reset=1`), writes 32-bit words through the JTAG-to-AXI master, then
sets `probe_out0=0x3` to release the CPU from the JTAG-loaded ROM image.

## 4. Expect on the monitor

`HDMI_TEST_PATTERN=1` (default) emits pclk-local colour bars directly from
`video_top`. Expect bars within ~10 ms of boot once the AL9134 I2C init and
pixel clock are alive. This proves HDMI clocking, data pins, sync, DE, reset,
and I2C without depending on normal VRAM scan-out.

`make tb-video-pattern` checks the same first-board contract in sim: the
pattern path stays on the pclk-side bars, never asks VRAM for pixels, and the
AL9134 reset/I2C handoff stays monotonic while the test runs.

`make tb-video-checkerboard` runs the same bypass with a coarse checkerboard.
It is the better quick check when you want obvious pixel alignment errors to
stand out.

`VIDEO_SMOKE=1` also preloads VRAM with SMPTE bars at reset, but those bars are
not the displayed source while `HDMI_TEST_PATTERN=1`. Disable the test pattern
only when you want to prove the full VRAM -> CDC -> scaler path.

## 5. Common failures

| Symptom                                 | Suspect                                  |
|-----------------------------------------|------------------------------------------|
| No JTAG target                          | cable unplugged, permissions on /dev     |
| VIO probes unavailable                  | `ENABLE_VIO=1` missing at synth          |
| Monitor blank, LED[3]=1                 | HDMI pinout/I2C/test-pattern path        |
| Monitor blank, LED[3]=0                 | HDMI MMCM / pixel-clock bug              |
| LED[2] stuck low                        | SD not provisioned, or boot_fsm error    |
| Bars forever with `HDMI_TEST_PATTERN=1`  | expected first-board display mode; use LEDs/VIO for CPU progress |
| Sad mac logo                            | ROM hit a POST failure — demo still counts as success |

## 6. Rollback

Keep a known-good bitstream at `build/vivado/fpga_top.good.bit`. If a fresh
bit bricks video, re-run `synth/program_fpga.sh --go --bit build/vivado/fpga_top.good.bit`.

## 7. Escalation (VIO dashboard)

`synth/vio_dashboard.tcl` reads:
- `probe1` / `probe2` = live VTG horizontal/vertical counters
- `probe3` / `probe4` = VRAM read address and registered HDMI RGB debug data
- `probe5` = last-committed CPU PC (shows boot progress)
- `probe7` = `{cpu_resetn, core_rst, ddr_cal_done, boot_rom_ready, hdmi_i2c_done, fb_underflow_sticky}`
- `probe9` = retired-insn counter (catches "CPU alive but slow")
- `probe10` / `probe11` = HDMI control and VRAM read valid/enable bundles
- `probe12` = DDR AXI valid/ready handshakes
- `probe13` / `probe14` = DAFB/VRAM write counters and VRAM write handshakes
- `probe_out0` = JTAG ROM-load controls: bit0 bypasses SD boot, bit1 releases
  the CPU, and bit2 forces CPU reset

If monitor is wrong and LEDs look right, dbg_pc + dbg_committed tells you
whether the core is running, stuck on a specific PC, or looping.
