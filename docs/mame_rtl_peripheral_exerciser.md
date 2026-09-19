# MAME-driven RTL peripheral exerciser feasibility

Scope: feasibility/design/prototype slice only.  This does not claim a
working live co-simulation path.  It describes the smallest architecture that
can use MAME's Quadra 700 driver to exercise our RTL peripheral/AXI path, using
the existing ROM harness and address map as constraints.

## Current repo and host facts

- Local MAME: `/usr/games/mame`, `MAME v0.264 (unknown)`.
- Local `macqd700` driver is present and reports the expected ROM:
  `420dbff3.rom`, size `1048576`, CRC `88ea2081`, SHA1
  `7a8ee468d16e64f2ad10cb8d1a45e6f07cc9e212`.
- `mame macqd700 -listmedia` exposes floppy, CD-ROM, and hard-disk media.
- A bounded local run with only `files/420dbff3.rom` copied into a temporary
  `macqd700` ROM directory does not start on this packaged MAME: it also asks
  for `342s0440-b.bin` from the `adbmodem` ROM set.  That file must be present
  in the chosen `-rompath` before the generated trace command can execute.
- `mame -showusage` in this package supports `-debug`, `-debugscript`,
  `-autoboot_script`, `-console`, `-seconds_to_run`, `-bench`,
  `-video none`, `-nothrottle`, `-rompath`, and `-nvram_directory`.
- `docs/mame_integration.md` recommends a source build pinned to
  `mame0287` for reproducibility.  The public MAME 0.287 docs require a
  C++20-capable compiler/library; GCC 11 is the minimum supported GCC family.
  Ubuntu 24.04's GCC 13 toolchain satisfies that.  The packaged 0.264 binary is
  useful for trace capture and local capability checks, but it is below the
  doc's recommended `0.266+` minimum for Mac source-reference work and is not
  the right base for custom-device work.
- Existing repo support:
  - `tb/tb_rom_boot.cpp` has a Q700 ROM boot harness, capped peripheral logs,
    event filters, data watches, DAFB/VRAM activity summaries, and directed
    selftests for several peripheral stubs.
  - `Makefile` has `tb-rom-boot`, ROM checkpoint/replay targets, Musashi ROM
    boot targets, and `rom-frontier-diff` for trace comparison.
  - `rtl/sys/peripheral_bus.v` fans out the 0x5000_0000 I/O window to pb-style
    byte peripherals plus DAFB AXI-Lite.

## Recommendation

Use a custom MAME build/device for live RTL peripheral exercising.  Keep the
debugger/Lua path as a trace oracle and smoke-probe generator, not as the live
read/write bridge.

Reason: synchronous reads are the hard requirement.  The Mac CPU must receive
the read value during the MAME memory access handler.  MAME debugger
watchpoints and Lua/autoboot scripts can observe accesses and drive an external
Verilator process after the fact, but they do not cleanly replace an already
completed MMIO read with data returned by RTL.  That makes them good for
logging and replay, but not for exercising ROM code against RTL-sourced read
data.

A custom MAME device or Q700 driver variant can install handlers for selected
MMIO windows.  Its `read` handler can synchronously call into an RTL bridge,
tick Verilator until AXI/pb read completion, then return the byte/word/long to
MAME's 68040.  Its `write` handler can synchronously or near-synchronously
forward writes to RTL and wait for write response.  That is the minimal design
that preserves the Mac bus contract.

## Minimal live architecture

Process split:

1. `mame_macrtl macqd700` runs a custom Q700-derived driver.
2. A bridge device replaces selected Q700 MMIO handlers with forwarding
   handlers.
3. The bridge talks to an external Verilator peripheral exerciser process over
   a simple local IPC protocol, or links Verilator directly into the custom MAME
   binary if the build friction is acceptable.
4. The external process instantiates the smallest RTL top that contains
   `peripheral_bus.v` plus the selected peripheral modules.  Start without the
   CPU core.  Drive the AXI4 slave side directly and score read/write responses.

Direct linking has lower latency and simpler synchronous reads, but couples
MAME and Verilator build systems.  IPC is easier to iterate if the protocol is
blocking per transaction:

```text
MAME read handler:
  send READ addr size fc pc
  block waiting for RESP data resp cycles irq_snapshot
  return data to MAME

MAME write handler:
  send WRITE addr size wdata wstrb fc pc
  block waiting for RESP bresp cycles irq_snapshot
  return
```

For the first prototype, use one outstanding transaction.  This matches
`peripheral_bus.v`'s current simplicity and avoids having to model MAME's CPU
access reordering.  Add batch/async only after correctness signals are useful.

The initial wire format and a mock blocking endpoint now live in:

- `tools/mame_rtl_bridge_protocol.py`
- `tools/mame_rtl_bridge_server.py`
- `tools/mame_scsi_bridge.cpp`
- `tools/mame_q700_rtl_overlay.py`
- `tb/mame_axi_periph_top.v`
- `tools/mame_axi_periph_bridge.cpp`

`mame_rtl_bridge_server.py` is still a mock responder for broad protocol
smoke tests.  `mame_scsi_bridge.cpp` is the first Verilated endpoint; it
services the Q700 TurboSCSI register and pseudo-DMA windows through
`rtl/mac/scsi.v`.  `mame_q700_rtl_overlay.py` patches a pinned MAME
`mame0287` checkout so the Quadra 700 VIA1, VIA2, Ethernet-ID, SONIC, SCC,
Orwell-control, SCSI/TurboSCSI, ASC, SWIM/IWM, and DAFB-register handlers
synchronously call the bridge when `MAME_RTL_BRIDGE_SOCKET` is set, while
falling back to the stock MAME handlers where one exists when the environment
variable is unset.

`mame_axi_periph_bridge.cpp` is the broader endpoint.  It instantiates
`tb/mame_axi_periph_top.v`, drives 128-bit AXI into `rtl/sys/peripheral_bus.v`,
and lets that RTL fan out to VIA1, VIA2, SCC, SCSI, ASC, and IWM.  DAFB
registers and debug_ctrl are currently deterministic AXI-Lite
responders inside the wrapper; they prove AXI timing and decode but are not
full behavioral device models yet.  This endpoint is the preferred target for
expanding the MAME overlay beyond the initial SCSI-only handler replacement.

The next display-oriented slice should replace the DAFB register stub with the
real DAFB register shim, add the `0xf9000000..0xf91fffff` VRAM aperture to the
bridge wrapper, and reuse the existing VRAM/scaler screenshot path already
covered by `tb-vram-scaler-firstlight`, `tb-framebuffer-pixel`, and
`tb-video-smoke`.  The minimal proof is a MAME-driven VRAM write sequence that
lands in RTL VRAM and emits a deterministic PPM/PNG frame from the RTL scanout
pipeline.

## Read handling

Reads must be synchronous at the MAME memory handler boundary.

Required behavior:

- Decode the MAME access width and address exactly as Q700 would see it.
- Canonicalize the 0x50xx_xxxx mirror using the existing repo rule:
  clear address bits covered by `0x00fc0000`.
- Drive the Verilated AXI read address channel or pb read pulse.
- Tick until `RVALID`/`pb_ack` or a cycle timeout.
- Return the lane-correct byte/word/long to MAME.
- Treat timeout, DECERR, or unexpected unmapped access as a test failure with
  the MAME PC and RTL cycle in the report.

The debugger/Lua path cannot solve this for real ROM execution because a
watchpoint callback observes the access but is not a replacement for the
device's read handler.

## Write handling

Writes are easier but should still be acknowledged by RTL.

Required behavior:

- Forward MAME writes with address, size, write strobes, data, PC, and function
  code where available.
- Tick Verilator until AXI `BVALID` or pb `ack`.
- Log the resulting state and any IRQ-line deltas.
- Fail on timeout, bad response, or write to an unmapped window that MAME
  thinks is backed.

For high-volume VRAM writes, add a later streaming fast path.  Do not start
there; the first peripheral exerciser should validate register traffic.

## Initial address windows

Start with the windows already backed in `tb/tb_rom_boot.cpp`,
`rtl/mac/glue.v`, and `rtl/sys/peripheral_bus.v`:

| Window | Device | First purpose |
|---|---|---|
| `0x50000000..0x50001fff` and mirrors | VIA1 | Overlay, timers, IFR/IER, ADB/RTC bit-bang side effects |
| `0x50002000..0x50003fff` and mirrors | VIA2 | Slot/SCSI IRQ gating and benign ROM probes |
| `0x5000c000..0x5000dfff` and mirrors | SCC | Serial probe register pointer/read status behavior |
| `0x5000f000..0x5000f0ff` and mirrors | SCSI/TurboSCSI regs | NCR/TurboSCSI command/status polling |
| `0x5000f100..0x5000f101` and mirrors | SCSI DMA shim | Pseudo-DMA handshake smoke only |
| `0x50014000..0x50015fff` and mirrors | ASC | SONORA ASC version/control/FIFO register probes |
| `0x5001e000..0x5001ffff` and mirrors | SWIM/IWM | No-media floppy probe behavior |
| `0xf9800000..0xf98003ff` | DAFB regs | Register setup and first display-write proof |
| `0xf9000000..0xf90fffff` | VRAM pixel aperture | Later framebuffer writes; too high-volume for phase 1 |

Do not alias `0x50004000` to SCC; the existing harness explicitly treats that
as a Q700 map hole.

## Pass/fail signals

Per-transaction pass/fail:

- Every forwarded read/write completes within a fixed RTL cycle budget.
- MAME-visible read data equals RTL returned data after lane extraction.
- AXI `RRESP/BRESP` is OKAY for backed windows and DECERR/SLVERR only where
  expected.
- pb-style peripherals assert `ack` exactly once per request.
- No unexpected writes land in the harness's unmapped-read/write scoreboards.

Behavioral pass/fail by device:

- VIA1: overlay clear sequence is observed; IFR bit 7 equals
  `|(IFR[6:0] & IER[6:0])`; Timer1 poll eventually sees IFR6; ADB/RTC
  serial reads do not wedge ROM polling.
- VIA2: slot/SCSI interrupt status reads are stable and do not create phantom
  pending IRQs.
- SCC: register-pointer writes and RR0/status reads match the ROM probe path;
  no monitor serial input is falsely reported.
- SCSI: command/status polling progresses through the same high-level
  selection/status states as the MAME device for no-disk or provisioned-media
  tests; DMA shim accesses are logged and acknowledged.
- ASC: version/control reads match the Q700 ROM's expected probe behavior;
  FIFO/IRQ bits are stable enough that audio init does not spin.
- SWIM/IWM: no-media status is returned and the ROM exits floppy probes.
- DAFB: first register write at the expected window is seen by RTL; later VRAM
  writes, when enabled, alter the RTL VRAM backing store.

Run-level pass/fail:

- MAME and RTL bridge logs agree on transaction count, order, address,
  direction, size, and write data for the enabled windows.
- The existing `tb-rom-boot` trace and the MAME run reach the same named
  ROM frontier PC for a small bounded run, or the first divergence report names
  a single MMIO transaction and PC.

## Practical next commands

Inventory local MAME and generate a debugger trace script for the same windows:

```sh
python3 tools/mame_mmio_trace_scaffold.py --out-dir /dev/shm/m68k/mame_mmio_probe
```

Run the generated command only as an observational trace, not as live
co-simulation:

```sh
sh /dev/shm/m68k/mame_mmio_probe/run_mame_mmio_trace.sh
```

Exercise the bridge protocol and run the mock endpoint:

```sh
python3 tools/mame_rtl_bridge_protocol.py --selftest --dump-windows
python3 tools/mame_rtl_bridge_server.py --socket /tmp/mame-rtl-bridge.sock
```

Build and run the first RTL-backed endpoint:

```sh
make mame-scsi-bridge
build/mame_scsi_bridge/Vscsi --socket /tmp/mame-scsi-bridge.sock
```

Build and run the AXI peripheral-bus endpoint:

```sh
make mame-axi-periph-bridge
build/mame_axi_periph_bridge/Vmame_axi_periph_top --socket /tmp/mame-axi-periph-bridge.sock
```

Patch a local MAME checkout for the live Q700 AXI peripheral bridge experiment:

```sh
python3 tools/mame_q700_rtl_overlay.py /tmp/mame
cd /tmp/mame
make SUBTARGET=macrtl SOURCES=src/mame/apple REGENIE=1 TOOLS=0 -j4
MAME_RTL_BRIDGE_SOCKET=/tmp/mame-axi-periph-bridge.sock ./mame_macrtl macqd700 ...
```

Build the existing RTL ROM harness and capture comparable peripheral logs:

```sh
make tb-rom-boot ROMBOOT_EXTRA="+periph_event_log=/dev/shm/m68k/rtl_periph_events.log +periph_event_filter=VIA1,VIA2,RTC,ADB,SCSI,SCC,ASC,SWIM,DAFB,VRAM +periph_event_log_limit=4096 +no_waves"
```

Custom MAME work should start from a pinned source tree rather than the
packaged 0.264 binary:

```sh
git clone --depth 1 --branch mame0287 https://github.com/mamedev/mame.git /tmp/mame
cd /tmp/mame
make SUBTARGET=macrtl SOURCES=src/mame/apple REGENIE=1 TOOLS=0 -j4
```

First custom-code milestone: replace only one read-mostly window, VIA2 or ASC,
with a bridge handler.  Prove that MAME blocks for RTL read data and still
reaches the same early ROM frontier as stock MAME.  Then add VIA1, because VIA1
has overlay/IRQ/timer side effects and is the first window where timing bugs
matter.
