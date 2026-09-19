#!/usr/bin/env python3
"""iwm_lockstep_diff.py — diff two IWM/SWIM register-event captures.

Reads MAME's `iwm_q700_<scenario>.csv` (CPU-side memory tap on Q700 SWIM
aperture, emitted by tools/mame_iwm_capture.lua) and our RTL's
`iwm_<scenario>.csv` (driven by tb/tb_iwm_stub.cpp playing back the
MAME write events and capturing the read responses).

The CSV format is `<sim_time_ns>,<R|W>,<reg>,<byte>` per non-comment
line, where:
  - sim_time_ns is informational only (ignored for the diff)
  - R/W is the access direction
  - reg is the SWIM register selector 0..F
  - byte is the byte exchanged on the access

Diff strategy:
  - Strip comment lines (starting with '#') and blanks.
  - Walk both streams in lock-step on (rw, reg, byte) triples.
  - Report the first divergence and a short window around it.

Exit codes:
  0  identical (or both empty)
  1  divergence
  2  argument or file error
"""

import argparse
import os
import sys


def read_events(path):
    """Read a CSV trace, returning a list of (rw, reg, byte) tuples.

    Comment lines (#-prefixed) and blanks are ignored.  Anything else is
    expected to look like `<ns>,<R|W>,<reg_hex>,<byte_hex>`.
    """
    events = []
    if not os.path.exists(path):
        return events
    with open(path) as f:
        for line_num, raw in enumerate(f, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 4:
                # Tolerate older 3-column traces (no time)
                if len(parts) == 3:
                    rw, reg, byte = parts
                else:
                    sys.stderr.write(
                        f"{path}:{line_num}: bad format: {line}\n")
                    sys.exit(2)
            else:
                _ns, rw, reg, byte = parts[0], parts[1], parts[2], parts[3]
            rw = rw.strip().upper()
            if rw not in ("R", "W"):
                sys.stderr.write(
                    f"{path}:{line_num}: bad rw='{rw}': {line}\n")
                sys.exit(2)
            try:
                reg_v = int(reg, 16)
                byte_v = int(byte, 16)
            except ValueError:
                sys.stderr.write(
                    f"{path}:{line_num}: bad hex: {line}\n")
                sys.exit(2)
            events.append((rw, reg_v & 0xF, byte_v & 0xFF))
    return events


def fmt_event(ev):
    rw, reg, byte = ev
    return f"{rw} reg={reg:x} byte=0x{byte:02x}"


def window(events, center, radius=4):
    start = max(0, center - radius)
    end = min(len(events), center + radius + 1)
    out = []
    for i in range(start, end):
        marker = "  <-- here" if i == center else ""
        out.append(f"  [{i:4d}] {fmt_event(events[i])}{marker}")
    return "\n".join(out) if out else "  (empty)"


def main():
    ap = argparse.ArgumentParser(
        description="diff two IWM/SWIM register-event captures (MAME vs RTL)")
    ap.add_argument("mame", help="MAME-side capture (.csv)")
    ap.add_argument("rtl",  help="RTL-side capture (.csv)")
    ap.add_argument("--max-events", type=int, default=512,
                    help="report match-status for this many initial events")
    ap.add_argument("--quiet", action="store_true",
                    help="only print summary lines on stdout")
    args = ap.parse_args()

    mame = read_events(args.mame)
    rtl  = read_events(args.rtl)

    if not args.quiet:
        print(f"[iwm-lockstep] MAME {args.mame}: {len(mame)} events")
        print(f"[iwm-lockstep] RTL  {args.rtl}:  {len(rtl)} events")

    if len(mame) == 0 and len(rtl) == 0:
        print("[iwm-lockstep] both streams empty -> IDENTICAL")
        return 0

    if len(mame) == 0:
        print("[iwm-lockstep] MAME stream empty, RTL is not.")
        print(window(rtl, 0))
        return 1

    if len(rtl) == 0:
        print("[iwm-lockstep] RTL stream empty, MAME is not.")
        print(window(mame, 0))
        return 1

    common = min(len(mame), len(rtl))
    first_div = -1
    for i in range(common):
        if mame[i] != rtl[i]:
            first_div = i
            break

    if first_div == -1 and len(mame) == len(rtl):
        print(f"[iwm-lockstep] streams IDENTICAL ({len(mame)} events)")
        return 0

    if first_div == -1:
        first_div = common
        print(f"[iwm-lockstep] streams agree on first {common} events, "
              f"then diverge by length (MAME={len(mame)} RTL={len(rtl)})")
    else:
        print(f"[iwm-lockstep] FIRST DIVERGENCE at event #{first_div}: "
              f"MAME={fmt_event(mame[first_div])} "
              f"RTL={fmt_event(rtl[first_div])}")

    print(f"[iwm-lockstep] MAME window around #{first_div}:")
    print(window(mame, first_div, 4))
    print(f"[iwm-lockstep] RTL window around #{first_div}:")
    print(window(rtl, first_div, 4))

    n = min(args.max_events, common)
    matched = 0
    for i in range(n):
        if mame[i] != rtl[i]:
            break
        matched += 1
    print(f"[iwm-lockstep] first {matched}/{n} events match")
    return 1


if __name__ == "__main__":
    sys.exit(main())
