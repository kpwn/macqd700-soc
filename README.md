# macqd700-soc

A Macintosh Quadra 700 system-on-chip for Xilinx Kintex UltraScale+. It is
everything around the CPU — DRAM, bus fabric, video, storage, networking and
the Mac peripheral set — so that a 68040 dropped into the AXI socket boots real
Mac OS from a real ROM.

It currently boots System 7.0.1 and System 7.5.3 to the Finder.

== BEGIN HUMAN ==

This is a project that began as an exploration of the capabilities of LLMs to write RTL
and do CPU design; Macintosh Quadra 700 was choosen as a target system as a good 68040 based
platform that wasn't too complex and can run System 7.0.1 -> 8.1 as well as A/UX.

Most of my design focus has been on the CPU side so the SoC is probably jankier; 
it's good enough to have System 7 run, though. 

A fair warning to the purists: MAME was used as a ground-truth, and the goal of
this project is NOT accuracy: we run AXI, have 2MB of L2C synchronous to core clock.
Peripheral/IO bus clk is @ 50MHz, and I-side of the CPU is hooked directly up to L2C.

One consequence of this design decision is that the CPU may ONLY run code in RAM or ROM, and
cannot successfully complete instruction fetches in I/O space. This is also a necessity since
the CPU is also not an accurate CPU, but a compatible and performance oriented one and I-cache is
always enabled and fetching 256 bits at a time (which I/O space couldn't possibly support).

== END HUMAN ==

[![The Quadra 700 FPGA system running Mac OS](https://h.kjc.sh/~qwertyoruiop/macq.JPG)](https://h.kjc.sh/~qwertyoruiop/macq.JPG)

## Target hardware

| | |
|---|---|
| Board | **RK-XCKU5P-F V1.2** |
| FPGA | `xcku5p-ffvb676-2-i` |
| DRAM | on-board DDR4, 32-bit (x8 ×4), via MIG |
| Video out | **Alinx AN9134** HDMI daughterboard |
| HDMI transmitter | AL9134 (SiI9134), fed parallel RGB + I2C control |
| Ethernet PHY | RTL8211F, RGMII, 1000BASE-T |
| Storage | SD card (SPI, 25 MHz) |

There is no internal TMDS — video leaves the FPGA as a parallel RGB bus to the
AN9134's transmitter, which is why `synth/hdmi.xdc` carries explicit bus-skew
and timing constraints for that interface.

The board-tested September 19, 2026 image runs the core at **200 MHz** with
the peripheral bus at **50 MHz**. Its final route meets setup and hold timing
(setup slack +0.001 ns, zero failing endpoints). The conservative build
script preset remains 100 MHz; do not assume a new implementation inherits
the verified image's timing result.

Known video limitation in this image: **832×624 fails at “millions of
colors” on hardware, while 256 colors works**. The cause is not yet
established; the corresponding simulated mode passes, so simulation alone
does not establish hardware support for this combination.

## The CPU is a submodule

The CPU is not part of this repository. It plugs into the socket defined by
`rtl/soc/cpu_socket.vh`:

    cpu040/  ->  https://github.com/kpwn/m68k-core-040-ooo   (branch: master)

an out-of-order, superscalar 68040 written in SpinalHDL. Clone with
submodules:

    git clone --recurse-submodules https://github.com/kpwn/macqd700-soc.git

`CPU=stub` builds the SoC standalone with an idle socket occupant, for platform
work without elaborating a core.

## ROMs you must supply

**No Apple firmware ships in the source tree.** These are Apple's and you
must provide your own dumps, exactly as MAME requires:

| path | what it is |
|---|---|
| `files/420dbff3.rom` | Quadra 700 ROM, 1 MiB — **required to boot** |
| `files/342s0440-b.bin` | ADB modem ROM — FPGA ADB PIC and integration tests, also MAME |
| `files/pmuv2.bin` | PMU firmware — MAME lockstep only |

Releases do not include Apple firmware. Add your own ADB modem ROM before
flashing; see the [firmware setup guide](docs/adb_firmware_bitstream.md).
The Quadra boot ROM and Mac OS disk image are supplied separately on SD.

**The ADB modem dump is required even if you use only injected keyboard/mouse
input.** Its PIC runs inside the FPGA; it is not an optional MAME-only test
asset. For real-firmware **simulation**, put your 1,024-byte `342s0440-b.bin`
in `files/`, then run:

```sh
make prepare-adb-firmware
# Alternatively, keep your dump outside the repository:
make prepare-adb-firmware MAME_ADB_ROM=/absolute/path/to/342s0440-b.bin
```

This creates `rtl/mac/adb_pic_fw.hex` (512 three-digit instruction words).
It is intentionally untracked. Do not substitute an empty file or a dummy
ROM: that will not provide a working ADB modem. This hex file is simulation-
only; synthesis always leaves PIC BRAM blank. The Quadra ROM goes on the SD
card; the ADB firmware is inserted into your bitstream locally, not loaded
from SD at runtime. PMU firmware is not needed for the FPGA build.

## SD card layout and preparation

The SD card is both the boot medium and the hard disk. The physical card
uses a raw layout, **not a host partition table or a filesystem containing
image files**. Each LBA is 512 bytes:

| LBA range | contents |
|---|---|
| `0 .. 2047` | Quadra 700 ROM, 1 MiB. `boot_fsm` copies this into DDR at `0x4000_0000` before releasing the CPU |
| `2048 .. 8175` | reserved; leave unused |
| `8176 .. 8191` | system persistence; PRAM uses LBA `8191` |
| `8192 ..` | the raw SCSI disk image the Mac sees |

At power-on `boot_fsm` reads sectors 0–2047 over SPI and writes them to DDR;
`boot_rom_ready` gates the CPU out of reset until that completes. The CPU's
reset PC is `0x4000_002A`.

### Write with a Linux SD-card reader (`dd`)

Use a **raw, uncompressed whole SCSI disk image** with a bootable Mac OS
installation, including its Macintosh partition map/driver partitions if
present. A partition-only HFS image, `.sit` archive, or container-format
disk image is not interchangeable with a raw whole-disk image. No OS image
is supplied. The image starts at card byte **4,194,304** (`8192 * 512`);
the Mac sees that location as SCSI disk LBA zero.

**These commands overwrite the selected device.** Back up its contents.
Use `lsblk -o NAME,PATH,SIZE,MODEL,TRAN,MOUNTPOINTS` to identify the removable
card, and manually unmount every mounted partition on it. Select the whole
card, such as `/dev/sdX` or `/dev/mmcblkN`, never a partition such as
`/dev/sdX1` or `/dev/mmcblkNp1`. The placeholder below deliberately does not
name a real device. Do not run these against your system disk.

Run the following in Bash after replacing all three paths:

```bash
(
set -eu
sd_device=/dev/disk/by-id/REPLACE_WITH_YOUR_SD_CARD
rom_image=files/420dbff3.rom
hdd_image=/absolute/path/to/your-bootable-mac-disk.hda

test -b "$sd_device"
test -f "$rom_image"
test -f "$hdd_image"
test "$(stat -c %s "$rom_image")" -eq 1048576
hdd_bytes=$(stat -c %s "$hdd_image")
test "$hdd_bytes" -gt 0
test "$((hdd_bytes % 512))" -eq 0
test "$(sudo blockdev --getsize64 "$sd_device")" -ge "$((4194304 + hdd_bytes))"
lsblk -o NAME,PATH,SIZE,MODEL,TRAN,MOUNTPOINTS "$sd_device"
read -r -p "All partitions unmounted? Type the exact device path to overwrite: " confirmed_device
test "$confirmed_device" = "$sd_device"

# ROM: exactly card LBA 0..2047.
sudo dd if="$rom_image" of="$sd_device" bs=512 count=2048 seek=0 conv=notrunc,fsync status=progress
# HDD: card LBA 8192 onward. Never write the HDD image at seek=0.
sudo dd if="$hdd_image" of="$sd_device" bs=512 seek=8192 conv=notrunc,fsync status=progress
sync

# Read back and compare both regions. cmp produces no output on success.
sudo cmp -n 1048576 "$rom_image" "$sd_device"
sudo cmp -n "$hdd_bytes" "$hdd_image" "$sd_device" 0 4194304
)
```

Those two writes **preserve** the reserved region and any existing PRAM.
On a first-use card, invalid PRAM is rejected and defaults are used. To
explicitly factory-reset stored PRAM, zero **only LBA 8191** after checking
the device again: `sudo dd if=/dev/zero of=/dev/YOUR_SD_CARD bs=512 seek=8191
count=1 conv=notrunc,fsync`. Do not zero the first 4 MiB after writing the ROM.
Safely eject the card, install it in the FPGA board, and boot. Flashing SPI
does not write either SD image; see [SPI programming](docs/spi_flash.md).

### Write through the FPGA's JTAG provisioning image

Two tools write the card, deliberately kept separate because their safety
rules are exact inverses — folding them together would make each one's guard
conditional:

    tools/sd_write_rom.sh          # writes the ROM; never writes at or above LBA 2048
    tools/sd_os_swap.sh            # writes the disk; never writes below LBA 8192

Both use a two-bitstream flow: a provisioning image exposes the SD writer over
JTAG (`docs/sd_jtag_writer.md`), then the real image is reloaded.

## Memory and bus fabric

A six-slave AXI crossbar (`rtl/soc/axi_xbar.v`) carries every master:

| slave | target |
|---|---|
| S0 | DDR4 via MIG |
| S1 | peripheral bus |
| S2 | DMA configuration (AXI-Lite) |
| S3 | VRAM pixel aperture |
| S4 | DAFB registers (AXI-Lite) |
| S5 | SD/JTAG writer (AXI-Lite) |

S1 is deliberately **outside** the flush domain: the peripheral bus stays alive
across `soc_full_rst`, so in-flight S1 transactions ride out a reset rather
than being torn down under a live device.

**L2 cache** — 8-way, 4096 sets, 64-byte lines (2 MiB), held in **URAM**,
with a dedicated 256-bit instruction-fetch port so the CPU's native-width fetch
is not truncated through the 128-bit crossbar path.

**Peripheral bus** — the Mac side of the machine hangs off a serialised I/O
port behind GLUE, which bridges AXI to the 50 MHz peripheral bus. Multi-hot
writes are serialised there; see `docs/peripheral_arch.md`.

## Storage: SCSI

A 53C96 SCSI controller (`rtl/mac/scsi.v`) backed by the SD card. The ROM
drives it with blind pseudo-DMA bursts, which is the hard part: the controller
must keep a FIFO supplied faster than the ROM drains it, or a burst silently
returns stale bytes. The supply path is pre-staged to a high-water mark before
each burst is armed for exactly that reason.

There is **no DMA-backed SCSI path**, matching the original Quadra 700:
its SCSI transfers use CPU-driven pseudo-DMA, not a hardware DMA engine.

## Networking

| piece | state |
|---|---|
| RTL8211F PHY + MAC + 1000BASE-T link | proven on hardware |
| SONIC (DP83932) emulation for Mac OS | `rtl/mac/q700_eth_sonic.v` + CDC/RX/TX |
| ICMP echo responder | kept as a physical-link regression image |
| Ethernet-backed vHDD | RTL landed (`rtl/board/net_block_framer.sv`), **not yet instantiated** |

The MAC and PHY layer come from **Taxi** — see Third-party code below. The
source is vendored under `vendor/rk5-eth`; `ETH_RK5_DIR` defaults to that
directory. Retain its license and attribution files in source releases.

## ADB — implemented, but not physically realized

The ADB subsystem is fully implemented in RTL: the PIC-based ADB modem
(`rtl/mac/adb_modem.v`, `adb_pic_modem.v`, `pic16c5x.v`), the line PHY
(`adb_phy.v`), and synthetic keyboard and mouse devices.

**It is not physically realized on this board.** There is no ADB connector,
because the RK-XCKU5P-F has no I/O left to give it — the constraint is the
board, not the design.

It is fully usable anyway: input is injected over JTAG, straight into the same
ADB state machines a real bus would drive. From the REPL:

    adb-mouse <dx> <dy> [down|up]        # relative mouse motion, and clicks
    adb-key <keycode> [down|up|press]    # synthetic keystroke (0..0x7F)
    adb-status                           # keyboard-FIFO / mouse-pending flags

That is enough to drive System 7 — menus, the Finder, applications — from a
host shell or from the control panel described below. What it is not is a real
ADB bus with real devices on it.

The plan is to move to an FPGA development board with enough accessible I/O
to connect physical peripherals. The ADB RTL is already present; exposing a
real connector also requires suitable electrical interfacing.

## Roadmap

The longer-term aim is to **retain the existing in-SoC bus and extend it to
support physical hardware too**. Moving to an FPGA board with more accessible
I/O would let a daughterboard expose genuine NuBus and SCSI signalling, so
period cards and drives could coexist with the peripherals modelled inside
the FPGA.

Electrical level conversion between the FPGA and the original interfaces is
expected to be the biggest challenge.

## Building

The supported workflow is Linux. Install Git, Bash, GNU Make/coreutils,
Python 3.9+, a C++17 compiler, Verilator, Java and `sbt` on `PATH`. CPU
generation was checked with sbt 1.10.11 / OpenJDK 25; platform simulations
with Verilator 5.032. The CPU submodule pins its sbt/Scala/SpinalHDL versions.
Some assembly/ROM tests also require GNU m68k binutils (`m68k-linux-gnu-*`
or `m68k-elf-*`).

For FPGA implementation, install AMD Vivado with Kintex UltraScale+ device
support and source its `settings64.sh`; the working image used Vivado
2025.2. Leave enough RAM for one implementation (roughly 12–15 GiB peak
in the measured flow) and do not run concurrent FPGA builds. Hardware
programming also needs working USB/JTAG permissions and `openFPGALoader`
for SPI flash.

From the repository root:

    git submodule update --init --recursive
    make cpu040-gen
    make lint
    CPU=m68k040 tools/build_bitstream.sh

Then follow [local ADB firmware insertion](docs/adb_firmware_bitstream.md).
**Do not flash `fpga_top.blank.bit`: its ADB modem cannot work until patched.**

Use this script for the guarded build workflow. Both it and the Makefile
currently default to `CPU=m68k040`. The script refuses `CPU=stub`, which
would produce a standalone platform image with no working Mac CPU.

The default script builds at 100 MHz. The downloadable 200 MHz image is a
specific verified implementation, not a guarantee that arbitrary changed
sources will close at 200 MHz. Keep its `.bit`, `.ltx`, and `.buildinfo`
together; follow [SPI programming](docs/spi_flash.md) and the release notes.

## Testing

    make lint          # whole-design lint, every shippable define combination (15)
    make tb-axi-xbar   # crossbar unit testbench
    make test          # the testbench suite

`make lint` is the minimum gate before any synthesis run.

`make test` runs the SoC testbenches serially and reports PASS, XFAIL and
FAIL separately. The Makefile's `TB_KNOWN_BROKEN` list records existing
failures; an XFAIL is not a passing test. Any unexpected failure makes the
command fail. Some full-resolution video tests take several minutes.
Historical v1 CPU simulation/fuzz targets are not the current CPU test gate;
run `make test-fast` inside `cpu040/` for the current core's fast suite.

The publication regression run is **not fully green**: two VRAM-chain
targets remain unresolved, in addition to existing expected failures.
See [verification results](docs/publication_verification.md) for the full
run summary, test-infrastructure fixes and targeted reruns.

## Development setup

The machine is developed headless. Everything below talks to one FPGA over one
JTAG cable, so the pieces are designed to share it.

**The REPL** is the foundation — a Vivado `-mode tcl` process running
`tools/jtag_repl.tcl`, held open by systemd units with a FIFO pair
(`/tmp/jtag_in`, `/tmp/jtag_out`) as its interface. It exposes the JTAG-AXI
debug bridge: halt/step, breakpoints, exception capture, memory and cache
operations, VIO probes, SD provisioning, ADB injection.

    tools/p141_start_repl.sh <bit> <ltx>                        # programs the FPGA
    JTAG_REPL_NO_PROGRAM=1 tools/p141_start_repl.sh <bit> <ltx> # reattach, board state kept

Use the reattach form to recover a REPL that died under a running OS — the
default form reprograms the FPGA and wipes whatever was running.

Because the FIFO has several independent writers, do not echo into it directly.
`tools/jt.sh "<command>"` sends one command and returns exactly its output, and
`tools/jtag_lease.sh` provides mutual exclusion (`acquire`/`release`/`run`) so
two agents cannot interleave commands and mis-attribute each other's replies.

**Video comes back over a real HDMI capture card.** The AN9134's HDMI output
loops into a V4L2 capture device on the development host — the FPGA's actual
display output, not a simulation of it.

    tools/fpga_video_capture.sh list           # enumerate capture devices
    tools/fpga_video_capture.sh view           # live view
    tools/fpga_video_capture.sh still shot.png # timestamped still

Defaults are `/dev/video0`, 1280x720, 60 fps, MJPEG, overridable via
`FPGA_VIDEO_*`.

**The control panel** ties the two together: `tools/fpga_mjpeg_server.py`
serves the capture card as MJPEG-over-HTTP with machine controls beside it, so
the Mac can be watched and driven from a browser — screen, mouse, keyboard,
reset, debug state. It writes into the same JTAG FIFO as everything else.

    python3 tools/fpga_mjpeg_server.py     # default http://10.200.0.12:8080

Override with `FPGA_MJPEG_BIND` / `FPGA_MJPEG_PORT`. This is how the machine is
used day to day: capture card for the screen, panel for input, REPL for
everything underneath.

**GDB** attaches to the same bridge for source-level debugging of code running
on the Mac:

    tools/gdbstub.py --port 1234
    # GDB <-TCP-> gdbstub.py <-FIFO-> jtag_repl.tcl <-JTAG-AXI-> FPGA

`tools/jtag_bringup_tui.py` is a terminal dashboard over the same interface.

## Licensing and third-party code

Original project contributions are **MIT**. Third-party components retain
their own licenses. Ethernet-enabled bitstreams are subject to
CERN-OHL-S-2.0 through Taxi Ethernet.

The VIA implementation drew inspiration from Shachar Shemesh’s CompuSAR/6522 project.

See [LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)
for component attribution and distribution requirements.
