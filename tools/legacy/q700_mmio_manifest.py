#!/usr/bin/env python3
"""Report Quadra 700 MMIO coverage against the RTL replication manifest.

The manifest is intentionally explicit: every Q700 device window from the
MAME address map should have an RTL ownership status.  A trace report keeps
boot-driven coverage separate from the longer-term goal of full device
replication.
"""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Window:
    name: str
    start: int
    end: int
    rtl: str
    status: str
    next_validation: str


WINDOWS = [
    Window("ROM_SWITCH", 0x40000000, 0x40FFFFFF, "rom_switch_r / overlay path",
           "external-to-peripheral_bus",
           "Validate ROM overlay disable and unmapped ROM mirror behavior in CPU/platform tb."),
    Window("VIA1", 0x50000000, 0x50001FFF, "rtl/mac/via1.v plus board RTC/ADB pins",
           "partial-rtl",
           "Lockstep 6522 registers, Timer1 VBL, IFR/IER, CA/CB handshakes, RTC/PRAM serial, ADB SR."),
    Window("VIA2", 0x50002000, 0x50003FFF, "rtl/mac/via2.v",
           "partial-rtl",
           "Lockstep slot/SCSI interrupt inputs, Timer1 PB7 accepted divergence, IFR/IER behavior."),
    Window("ENET", 0x50008000, 0x50008007, "rtl/mac/q700_eth_sonic.v",
           "active-rtl",
           "Lockstep Q700 MAC PROM bytes/checksum against MAME and board straps."),
    Window("SONIC", 0x5000A000, 0x5000B0FF, "rtl/mac/q700_eth_sonic.v",
           "partial-rtl",
           "Deepen DP83932 descriptor DMA and connect packet movement through the planned FPGAtaxi AXI-stream path."),
    Window("SCC", 0x5000C000, 0x5000DFFF, "rtl/mac/scc.v",
           "active-rtl",
           "Lockstep modem pins, TX/RX/IRQ timing, and serial cable/no-cable line-state transitions."),
    Window("ORWELL", 0x5000E000, 0x5000E0FF, "rtl/mac/orwell_stub.v",
           "rtl-placeholder",
           "Identify controls from MAME/Apple docs; validate reset/status bits instead of returning zero."),
    Window("SCSI", 0x5000F000, 0x5000F0FF, "rtl/mac/scsi.v TURBOSCSI_C96",
           "active-rtl",
           "Deepen 53C96 front door beyond reset/config/status polling, then validate selection/CDB/status phases, disk backing store, IRQ/DRQ."),
    Window("SCSI_DMA", 0x5000F100, 0x5000F101, "rtl/mac/scsi.v DMA shim",
           "partial-rtl",
           "Implement DAFB TurboSCSI pseudo-DMA handshakes and byte ordering against disk traffic."),
    Window("ASC", 0x50014000, 0x50015FFF, "rtl/mac/asc.v",
           "active-rtl",
           "Lockstep SONORA/EASC registers, FIFO service IRQs, chime FIFO fill, sample output WAV."),
    Window("SWIM", 0x5001E000, 0x5001FFFF, "rtl/mac/iwm_stub.v",
           "rtl-placeholder",
           "Implement SWIM/IWM mode register semantics, drive status, no-media and media-backed floppy flows."),
    Window("VRAM", 0xF9000000, 0xF91FFFFF, "bridge shm / rtl/sys/vram.v in hardware",
           "testbench-shared",
           "Route MAME writes into RTL scanout VRAM and compare dumped frames/CLUT/scaler output."),
    Window("DAFB", 0xF9800000, 0xF98003FF, "rtl/mac/video.v",
           "active-rtl",
           "Lockstep DAFB register map, monitor sense, CLUT, base/stride/BPP, frame timing."),
]

LINE_RE = re.compile(r"\blabel=([^ ]+).*?\baddr=(0x[0-9a-fA-F]+)")


def find_window(addr: int) -> Window | None:
    for window in WINDOWS:
        if window.start <= addr <= window.end:
            return window
    return None


def parse_trace(path: Path) -> dict[str, int]:
    counts: dict[str, int] = {}
    for line in path.read_text(errors="replace").splitlines():
        match = LINE_RE.search(line)
        if not match:
            continue
        label = match.group(1)
        addr = int(match.group(2), 16)
        window = find_window(addr)
        key = window.name if window else f"UNMAPPED:{label}"
        counts[key] = counts.get(key, 0) + 1
    return counts


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", nargs="?", type=Path,
                        help="Optional MAME_RTL_MMIO_TRACE log to count by manifest window.")
    args = parser.parse_args()

    counts = parse_trace(args.trace) if args.trace else {}
    print("name,start,end,status,trace_count,rtl,next_validation")
    for window in WINDOWS:
        print(
            f"{window.name},0x{window.start:08x},0x{window.end:08x},"
            f"{window.status},{counts.get(window.name, 0)},"
            f"{window.rtl},{window.next_validation}"
        )
    for name, count in sorted(counts.items()):
        if name.startswith("UNMAPPED:"):
            print(f"{name},,,unmapped,{count},,Add to manifest or fix address decode")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
