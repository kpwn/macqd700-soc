#!/usr/bin/env python3
"""Compare native-MAME and RTL-bridged Q700 MMIO traces.

The overlay logs the exact handler width used on each side.  Native MAME VIA
handlers often see 16-bit accesses carrying an active high byte, while the RTL
bridge forwards the actual byte lane.  This comparator normalizes those common
byte-lane cases and reports the first behavioral divergence.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Event:
    line: int
    mode: str
    op: str
    label: str
    size: int
    addr: int
    data: int
    mem_mask: int
    pc: int
    cycles: int

    @property
    def key(self) -> tuple[str, str, int]:
        return (self.op, self.label, self.addr)

    @property
    def normalized_data(self) -> int:
        if self.size == 1:
            return self.data & 0xff
        if self.mem_mask == 0x0000_ff00:
            return (self.data >> 8) & 0xff
        if self.mem_mask == 0x0000_00ff:
            return self.data & 0xff
        if self.size == 2 and ((self.data >> 8) & 0xff) == (self.data & 0xff):
            return self.data & 0xff
        if self.size == 2:
            return self.data & 0xffff
        return self.data & 0xffff_ffff


def parse_trace(path: Path, labels: set[str]) -> list[Event]:
    events: list[Event] = []
    with path.open("r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            if not line.startswith("mame-mmio "):
                continue
            fields: dict[str, str] = {}
            for part in line.split()[1:]:
                key, sep, value = part.partition("=")
                if sep:
                    fields[key] = value
            label = fields["label"]
            if labels and label not in labels:
                continue
            events.append(
                Event(
                    line=line_no,
                    mode=fields["mode"],
                    op=fields["op"],
                    label=label,
                    size=int(fields["size"], 0),
                    addr=int(fields["addr"], 0),
                    data=int(fields["data"], 0),
                    mem_mask=int(fields["mem_mask"], 0),
                    pc=int(fields["pc"], 0),
                    cycles=int(fields["cycles"], 0),
                )
            )
    return events


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("native_trace", type=Path)
    ap.add_argument("rtl_trace", type=Path)
    ap.add_argument("--label", action="append", default=[], help="restrict comparison to one label; repeatable")
    ap.add_argument("--limit", type=int, default=100_000, help="maximum events to compare")
    args = ap.parse_args()

    labels = set(args.label)
    native = parse_trace(args.native_trace, labels)
    rtl = parse_trace(args.rtl_trace, labels)
    count = min(len(native), len(rtl), args.limit)

    for index in range(count):
        a = native[index]
        b = rtl[index]
        if a.key != b.key or a.normalized_data != b.normalized_data:
            print(f"divergence index={index}")
            print(
                f"native line={a.line} key={a.key} size={a.size} data=0x{a.data:08x} "
                f"norm=0x{a.normalized_data:08x} mem_mask=0x{a.mem_mask:08x} pc=0x{a.pc:08x}"
            )
            print(
                f"rtl    line={b.line} key={b.key} size={b.size} data=0x{b.data:08x} "
                f"norm=0x{b.normalized_data:08x} mem_mask=0x{b.mem_mask:08x} pc=0x{b.pc:08x}"
            )
            return 1

    if len(native) != len(rtl):
        print(f"matched {count} events; length differs native={len(native)} rtl={len(rtl)}")
        return 2
    print(f"matched {count} events")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
