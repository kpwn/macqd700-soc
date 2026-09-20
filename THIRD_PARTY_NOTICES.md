# Third-party notices

Original contributions are MIT under LICENSE. Third-party terms remain in
force, including for derived portions and binary distribution. This document
identifies third-party components, their licenses, and applicable notices.

Ethernet-enabled bitstreams are subject to CERN-OHL-S-2.0.
Original contributions remain available under MIT; component notices,
source obligations, and separate firmware/vendor-IP terms still apply.

## MAME — BSD-3-Clause portions

MAME as a combined program is GPL-2.0-or-later, but the following reference
files were checked at both mame0285 and mame0287 and carry BSD-3-Clause.
Excerpts/adaptations retain the credited authors and the complete conditions
and disclaimer in LICENSES/MAME-BSD-3-Clause.txt. Supply these notices with
binary distributions too. MAME and its contributors do not endorse this project.

Upstream: https://github.com/mamedev/mame
License guidance: https://docs.mamedev.org/license.html
Reference versions: https://github.com/mamedev/mame/tree/mame0285 and
https://github.com/mamedev/mame/tree/mame0287 .

| MAME source | Copyright holders recorded upstream | Local reference/adaptation area |
|---|---|---|
| src/mame/apple/dafb.cpp | R. Belmont | rtl/mac/video.v, mode_decode.v, video pipeline and mode tests/tools |
| src/mame/apple/macquadra700.cpp | R. Belmont | platform wiring and tools/mame_q700_rtl_overlay.py |
| src/mame/apple/macrtc.cpp | R. Belmont | rtl/mac/rtc.v |
| src/mame/apple/adbmodem.cpp; macadb.cpp | R. Belmont | ADB modem, PIC wrapper and synthetic devices |
| src/devices/machine/ncr53c90.cpp | Olivier Galibert | rtl/mac/scsi.v and SCSI tests |
| src/devices/machine/ncr5380.cpp | Patrick Mackinlay | SCSI compatibility behavior |
| src/devices/machine/nscsi_bus.cpp; src/devices/bus/nscsi/hd.cpp | Olivier Galibert | SCSI target behavior |
| src/devices/sound/asc.cpp | R. Belmont | rtl/mac/asc.v and sound tests |
| src/devices/machine/6522via.cpp | Peter Trauner, Mathis Rosenhauer | VIA behavior |
| src/devices/machine/z80scc.cpp | Joakim Larsson Edstrom | rtl/mac/scc.v |
| src/devices/cpu/pic16c5x/pic16c5x.cpp | Tony La Porta | rtl/mac/pic16c5x.v |
| src/devices/machine/iwm.cpp | Olivier Galibert | rtl/mac/iwm_stub.v and tests |
| src/devices/machine/dp83932c.cpp | Patrick Mackinlay | rtl/mac/q700_eth_sonic.v, q700_sonic_tx.sv, q700_sonic_rx.sv |

The DAFB implementation excerpt in mode_decode.v and source fragments in the
MAME overlay tool are included material, not just observations. Local changes
adapt behavior to synchronous RTL, bus interfaces, buffering and verification.
Keep attribution when extracting these files. Preserve the original file
headers when applying the overlay to a MAME checkout. If distributing a
modified MAME executable, meet the licenses of that combined executable;
the BSD status of these individual files is not a blanket BSD grant for MAME.

macrtc.cpp additionally acknowledges previous work by Nathan Woods and
Raphael Nabet; dafb.cpp acknowledges inspiration from Olivier Galibert and
Vas Crabb. Those credits are retained here.

## CompuSAR/6522

The VIA implementation drew inspiration from Shachar Shemesh’s CompuSAR/6522 project.

## Taxi Ethernet — CERN-OHL-S-2.0

Vendored source: vendor/rk5-eth/third_party/taxi .
Retain its LICENSE, AUTHORS and per-file notices. See vendor/rk5-eth/VENDORING.md
for upstream commit and local changes. Taxi's terms apply independently of
the repository's MIT default. All releases and bitstreams with Ethernet
enabled include Taxi and are subject to CERN-OHL-S-2.0. Applicable
complete-source, source-location and modification-notice
obligations must be met, subject to the license's Available Component
exceptions.

Source location for local modifications: https://github.com/kpwn/macqd700-soc
CPU source: https://github.com/kpwn/m68k-core-040-ooo

## CPU and host reference models

The pinned CPU contains MIT contributions and NaxRiscv attribution.
Its SoftFloat-derived FPU portions retain Release 2b's nonstandard terms.
See cpu040/THIRD_PARTY_NOTICES.md and cpu040/LICENSES/SoftFloat-2b.txt.

Musashi is an optional upstream submodule in the pinned CPU revision.
Its PMMU/FPU source is fetched directly upstream, not bundled in our current
source tree or release packages. Our local integration patch retains its
own Musashi/MAME notices. Host models are not synthesized into the FPGA.

## User-supplied firmware

Apple firmware is not included in the source or releases and is not covered
by the project's licenses. Supply your own ROM dumps; see the
[firmware setup guide](docs/adb_firmware_bitstream.md) for the ADB modem and
[README](README.md#roms-you-must-supply) for the Quadra boot ROM.

## FPGA vendor IP

Vivado-generated MIG/debug/clocking IP and FPGA primitives are subject to
their applicable vendor terms. They are not relicensed by this project's MIT
grant. A complete-source/redistribution assessment must account for them.
