#!/usr/bin/env python3
"""axi_lockstep_diff.py — diff two CPU-side AXI traces (MAME vs RTL).

CSV format (one event per non-comment line):
    <seq>,<R|W>,<addr_hex>,<size_bytes>,<data_hex>

Both captures cover the same address filter:
  - DROP  0x0000_0000..0x3FFF_FFFF (DDR/RAM)
  - DROP  0x4000_0000..0x40FF_FFFF (ROM)
  - KEEP  everything else (peripherals 0x5000_0000.. , VRAM 0xF900_0000..,
          DAFB 0xF980_0000..0xF980_03FF, NuBus probes, …)

Diff strategy:
  1. Strip comments (#-prefixed) and blanks.  <seq> is informational —
     reindex to 0..N-1 in load order so an out-of-order writer doesn't
     destabilise the alignment.
  2. Walk both streams step-by-step on (rw, addr, size, data) tuples.
  3. Stop on the first divergence and print ±5 events of context plus a
     "DIVERGE @ seq=N" header.

Exit codes:
    0  identical or matching prefix exhausted (one stream truncated)
    1  divergence
    2  argument or file error
"""

import argparse
import os
import sys


def read_events(path):
    """Read an AXI capture CSV.  Returns a list of (rw, addr, size, data) tuples.

    Bad lines abort with sys.exit(2).  Unrecognised <rw> values likewise.
    """
    events = []
    if not os.path.exists(path):
        sys.stderr.write(f"axi_lockstep_diff: missing capture: {path}\n")
        sys.exit(2)
    with open(path) as f:
        for line_num, raw in enumerate(f, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) != 5:
                sys.stderr.write(
                    f"{path}:{line_num}: expected 5 fields, got {len(parts)}: {line}\n")
                sys.exit(2)
            _seq, rw, addr_hex, size_str, data_hex = parts
            rw = rw.strip().upper()
            if rw not in ("R", "W"):
                sys.stderr.write(
                    f"{path}:{line_num}: bad rw='{rw}': {line}\n")
                sys.exit(2)
            try:
                addr = int(addr_hex, 16)
                size = int(size_str)
                data = int(data_hex, 16)
            except ValueError:
                sys.stderr.write(
                    f"{path}:{line_num}: bad numeric field: {line}\n")
                sys.exit(2)
            if size not in (1, 2, 4):
                sys.stderr.write(
                    f"{path}:{line_num}: unexpected size={size}: {line}\n")
                sys.exit(2)
            events.append((rw, addr & 0xFFFFFFFF, size, data & ((1 << (size * 8)) - 1)))
    return events


def fmt_event(ev):
    rw, addr, size, data = ev
    width = size * 2
    return f"{rw} addr=0x{addr:08x} size={size}B data=0x{data:0{width}x}"


def window(events, center, radius=5):
    start = max(0, center - radius)
    end = min(len(events), center + radius + 1)
    out = []
    for i in range(start, end):
        marker = "  <-- here" if i == center else ""
        out.append(f"  [{i:5d}] {fmt_event(events[i])}{marker}")
    return "\n".join(out) if out else "  (empty)"


def main():
    ap = argparse.ArgumentParser(
        description="diff two CPU-side AXI captures (MAME vs RTL)")
    ap.add_argument("mame", help="MAME-side capture (.csv)")
    ap.add_argument("rtl",  help="RTL-side capture (.csv)")
    ap.add_argument("--max-events", type=int, default=10000,
                    help="report match-status for this many initial events")
    ap.add_argument("--quiet", action="store_true",
                    help="only print summary lines on stdout")
    args = ap.parse_args()

    mame = read_events(args.mame)
    rtl  = read_events(args.rtl)

    if not args.quiet:
        print(f"[axi-lockstep] MAME {args.mame}: {len(mame)} events")
        print(f"[axi-lockstep] RTL  {args.rtl}:  {len(rtl)} events")

    if len(mame) == 0 and len(rtl) == 0:
        print("[axi-lockstep] both streams empty -> IDENTICAL")
        return 0

    if len(mame) == 0:
        print("[axi-lockstep] MAME stream empty, RTL is not.")
        print(window(rtl, 0))
        return 1

    if len(rtl) == 0:
        print("[axi-lockstep] RTL stream empty, MAME is not.")
        print(window(mame, 0))
        return 1

    common = min(len(mame), len(rtl))
    first_div = -1
    for i in range(common):
        if mame[i] != rtl[i]:
            first_div = i
            break

    if first_div == -1 and len(mame) == len(rtl):
        print(f"[axi-lockstep] streams IDENTICAL ({len(mame)} events)")
        return 0

    if first_div == -1:
        # Streams agree on the prefix but differ in length — surface that
        # as a divergence at the truncation boundary so the caller sees
        # context on both sides.
        first_div = common
        print(f"[axi-lockstep] streams agree on first {common} events, "
              f"then diverge by length (MAME={len(mame)} RTL={len(rtl)})")
    else:
        print(f"[axi-lockstep] DIVERGE @ seq={first_div}: "
              f"mame={fmt_event(mame[first_div])} "
              f"rtl={fmt_event(rtl[first_div])}")

    print(f"[axi-lockstep] MAME context around #{first_div}:")
    print(window(mame, first_div, 5))
    print(f"[axi-lockstep] RTL  context around #{first_div}:")
    print(window(rtl, first_div, 5))

    n = min(args.max_events, common)
    matched = 0
    for i in range(n):
        if mame[i] != rtl[i]:
            break
        matched += 1
    print(f"[axi-lockstep] matched {matched}/{n} events before divergence")
    return 1


if __name__ == "__main__":
    sys.exit(main())
