#!/usr/bin/env python3
"""Summarize MAME/RTL IRQ snapshots from the Q700 bridge trace.

`tools/mame_q700_rtl_overlay.py` emits `mame-irq` records when
`MAME_RTL_IRQ_TRACE=1` is set.  The records are intentionally snapshots at
MMIO boundaries, not cycle-exact interrupts.  This tool checks event order with
an allowed snapshot lag so timer/audio phase differences do not look like hard
failures.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
import sys


MAME_IRQ_BITS = (
    ("via1", 0x01),
    ("via2", 0x02),
    ("scc", 0x04),
)

RTL_IRQ_BITS = (
    ("via1", 0x01),
    ("via2", 0x02),
    ("scc", 0x04),
    ("scsi_raw", 0x08),
    ("asc_raw", 0x10),
    ("iwm_raw", 0x20),
)


@dataclass(frozen=True)
class Snapshot:
    index: int
    label: str
    addr: int
    mame: int
    rtl: int
    pc: int
    cycles: int


def _parse_kv(line: str) -> dict[str, str]:
    out: dict[str, str] = {}
    for tok in line.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


def load_snapshots(path: Path) -> list[Snapshot]:
    snapshots: list[Snapshot] = []
    with path.open("r", errors="replace") as f:
        for line in f:
            if not line.startswith("mame-irq "):
                continue
            parts = _parse_kv(line)
            snapshots.append(
                Snapshot(
                    index=len(snapshots),
                    label=parts.get("label", "?"),
                    addr=int(parts.get("addr", "0"), 16),
                    mame=int(parts.get("mame", "0"), 16),
                    rtl=int(parts.get("rtl", "0"), 16),
                    pc=int(parts.get("pc", "0"), 16),
                    cycles=int(parts.get("cycles", "0"), 0),
                )
            )
    return snapshots


def transitions(snapshots: list[Snapshot], field: str, bit: int) -> list[tuple[int, bool]]:
    prev = False
    out: list[tuple[int, bool]] = []
    for snap in snapshots:
        value = bool((snap.mame if field == "mame" else snap.rtl) & bit)
        if value != prev:
            out.append((snap.index, value))
            prev = value
    return out


def has_transition_near(events: list[tuple[int, bool]], index: int, value: bool, lag: int) -> bool:
    return any(value == ev_value and index <= ev_index <= index + lag for ev_index, ev_value in events)


def bit_state_at(snapshots: list[Snapshot], field: str, index: int, bit: int) -> bool:
    snap = snapshots[index]
    value = snap.mame if field == "mame" else snap.rtl
    return bool(value & bit)


def validate_order(snapshots: list[Snapshot], lag: int) -> list[str]:
    errors: list[str] = []
    rtl_via2_events = transitions(snapshots, "rtl", 0x02)

    for raw_name, raw_bit in (("asc_raw", 0x10), ("scsi_raw", 0x08)):
        for index, asserted in transitions(snapshots, "rtl", raw_bit):
            if asserted and not (
                bit_state_at(snapshots, "rtl", index, 0x02) or
                has_transition_near(rtl_via2_events, index, True, lag)
            ):
                snap = snapshots[index]
                errors.append(
                    f"{raw_name} asserted at snapshot {index} pc=0x{snap.pc:08x} "
                    f"without RTL VIA2 assertion within {lag} snapshots"
                )

    for name, bit in MAME_IRQ_BITS:
        mame_events = transitions(snapshots, "mame", bit)
        rtl_events = transitions(snapshots, "rtl", bit)
        for index, asserted in mame_events:
            if not (
                bit_state_at(snapshots, "rtl", index, bit) == asserted or
                has_transition_near(rtl_events, index, asserted, lag)
            ):
                snap = snapshots[index]
                errors.append(
                    f"MAME {name} {'assert' if asserted else 'clear'} at snapshot {index} "
                    f"pc=0x{snap.pc:08x} has no RTL class match within {lag} snapshots"
                )

    return errors


def summarize(snapshots: list[Snapshot]) -> str:
    states = Counter((snap.mame, snap.rtl, snap.label) for snap in snapshots)
    lines = [f"snapshots={len(snapshots)}"]
    for name, bit in MAME_IRQ_BITS:
        lines.append(f"mame_{name}_high={sum(1 for s in snapshots if s.mame & bit)}")
    for name, bit in RTL_IRQ_BITS:
        lines.append(f"rtl_{name}_high={sum(1 for s in snapshots if s.rtl & bit)}")
    lines.append("top_states:")
    for (mame, rtl, label), count in states.most_common(12):
        lines.append(f"  {count:8d} label={label:8s} mame=0x{mame:02x} rtl=0x{rtl:02x}")
    return "\n".join(lines)


def selftest() -> None:
    snaps = [
        Snapshot(0, "ASC", 0x50014804, 0x00, 0x00, 0x1000, 1),
        Snapshot(1, "ASC", 0x50014804, 0x00, 0x10, 0x1004, 2),
        Snapshot(2, "VIA2", 0x50003A00, 0x00, 0x12, 0x1008, 3),
        Snapshot(3, "VIA2", 0x50003A00, 0x02, 0x12, 0x100C, 4),
        Snapshot(4, "VIA2", 0x50003A00, 0x00, 0x00, 0x1010, 5),
    ]
    assert not validate_order(snaps, lag=2)
    assert validate_order(snaps[:2], lag=0)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("trace", nargs="?", type=Path)
    ap.add_argument("--lag", type=int, default=64, help="allowed snapshot lag for class matches")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args(argv)

    if args.selftest:
        selftest()
        print("analyze_mame_irq_trace selftest passed")
        return 0

    if args.trace is None:
        ap.error("trace is required unless --selftest is used")
    snapshots = load_snapshots(args.trace)
    print(summarize(snapshots))
    if not snapshots:
        print("no mame-irq snapshots found", file=sys.stderr)
        return 1

    errors = validate_order(snapshots, args.lag)
    if errors:
        print("irq validation failures:", file=sys.stderr)
        for err in errors[:20]:
            print(f"  {err}", file=sys.stderr)
        if len(errors) > 20:
            print(f"  ... {len(errors) - 20} more", file=sys.stderr)
        return 1
    print(f"irq validation passed lag={args.lag}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
