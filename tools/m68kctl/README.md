# m68kctl

Host-side Python package and CLI for the `m68k-ooo` FPGA target.
Provides PCIe/XDMA-based access to the CPU debug interface, DDR4 system
bus, and SD-card provisioning engine — with a full mock backend so the
library and CLI are testable and demoable without an FPGA present.

## Install

```bash
# Editable install (recommended for dev):
pip install -e tools/m68kctl

# Or run straight from the source tree:
PYTHONPATH=tools python -m m68kctl --help
```

Python 3.10+ required.  No non-stdlib dependencies.

## Quickstart

```bash
# Identify the FPGA + print link status + perf counters.
m68kctl info

# Halt / resume / single-step the CPU.
m68kctl cpu halt
m68kctl cpu step
m68kctl cpu resume

# Force the PC to a specific address.
m68kctl cpu redirect 0x40800000

# Dump the last 32 retired PCs.
m68kctl cpu trace --tail 32

# Program a latched hardware stop and clear it after inspection.
m68kctl cpu reset-halt
m68kctl cpu halt-after 1000000
m68kctl cpu break-pc 0x40801234
m68kctl cpu halt-exc
m68kctl cpu halt-status
m68kctl cpu clear-halt

# Architectural register snapshot (TIER 2; graceful degrade if bitstream
# only implements TIER 1 — warns in the log and returns zeroes).
m68kctl cpu regs

# Upload a 4 MB ROM image to the SD card at LBA 0.
m68kctl sd upload /data/quadra_rom.bin

# Diff it back.
m68kctl sd verify /data/quadra_rom.bin

# Check the first-light raw SD layout.
m68kctl sd check-layout --rom /data/quadra_rom.bin --hdd /data/q700.hdv

# Build an offline raw SD image: ROM at byte 0, HDD at byte 0x00400000.
m68kctl sd make-image --rom /data/quadra_rom.bin --hdd /data/q700.hdv \
  -o /tmp/q700-firstlight.sdimg

# Read/hex-dump a single SD block.
m68kctl sd read 0

# Dump a region of the CPU's system bus (ROM at 0x40000000).
m68kctl bus dump 0x40000000 0x400000 -o /tmp/rom_dump.bin

# Capture multiple RAM/ROM regions into a checkpoint bundle.
m68kctl checkpoint dump --output-dir /tmp/checkpoint \
  --region ram 0x00001000 0x2000 \
  --region rom 0x40002000 0x1000
```

## Mock mode — no FPGA required

Pass `--mock` to any subcommand to route all operations through a
`MockDevice` (an in-process dict-backed memory model with a fake SD
card).  Lets you exercise the full CLI + library on any workstation:

```bash
# Version + IPC come back with plausible magic values.
m68kctl --mock info

# Round-trip a ROM image through the fake SD card sim.
m68kctl --mock sd upload /tmp/test.bin
# (Note: mock state is per-process; upload + verify on separate CLI
# invocations won't share state.  For the round-trip, run the test
# suite or drive the mock from Python directly.)
```

## Python library

```python
from m68kctl import MockDevice, CpuDebug, SdCard, SystemBus
from m68kctl.provision import upload_rom, verify_rom

dev = MockDevice()
cpu = CpuDebug(dev)
print(f'PC=0x{cpu.pc:08x}, IPC={cpu.ipc:.3f}')

sd = SdCard(dev)
upload_rom(sd, '/data/quadra_rom.bin')
bad = verify_rom(sd, '/data/quadra_rom.bin')
assert not bad

bus = SystemBus(dev)
bus.write_bytes(0x40000000, open('rom.bin', 'rb').read())
```

## Layout

| File            | Purpose                                               |
|-----------------|-------------------------------------------------------|
| `device.py`     | `XdmaDevice` (real) + `MockDevice` (test)             |
| `regs.py`       | Register offsets + bit masks (from `debug_pcie.md`)   |
| `bus.py`        | `SystemBus` over BAR 0 DMA                            |
| `cpu.py`        | `CpuDebug` over BAR 1 debug_ctrl                      |
| `sd.py`         | `SdCard` over BAR 1 sd_provision                      |
| `sd_image.py`   | First-light raw SD-card image layout checker/builder  |
| `provision.py`  | `upload_rom` / `verify_rom` / `dump_region` / bundle   |
| `cli.py`        | argparse CLI — `python -m m68kctl`                    |

## Reference

- `/docs/debug_pcie.md` — authoritative register catalogue (debug_ctrl)
- `/rtl/core/debug/debug_ctrl.v` — TIER 1 RTL
- `/rtl/sys/sd_provision.v` — SD provisioning RTL

## Testing

```bash
make tb-host          # runs python -m unittest on tb/tests/host/
make pcie-checkpoint-dump-check
```

## Known gaps

- `SystemBus` treats the entire BAR 0 window as DMA-reachable — this matches
  the integrator plan (single XDMA AXI-MM master onto the system crossbar)
  but needs re-check when `mac-top-integrator` lands.
- `checkpoint dump` streams RAM/ROM bundle captures directly to disk and
  writes a JSON manifest alongside the binary region files.
- `SdCard` is single-block (CMD17 / CMD24) only.  Multi-block streaming
  will come with task #22 sd-ctrl unification.
- TIER 2 debug_ctrl registers (arch reg snapshot, commit log, watchpoints)
  are wired through but degrade gracefully when the bitstream lacks them.
