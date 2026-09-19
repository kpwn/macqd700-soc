#!/usr/bin/env python3
"""Summarize MAME Q700 MMIO traces emitted by the RTL overlay."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict, deque
from dataclasses import dataclass
from pathlib import Path


SCSI_REGS = {
    0x0: "TC_LOW/FIFO",
    0x1: "TC_MID",
    0x2: "FIFO",
    0x3: "COMMAND",
    0x4: "STATUS",
    0x5: "INTERRUPT/TIMEOUT",
    0x6: "SEQ_STEP/SYNC",
    0x7: "FLAGS/SYNC_OFFSET",
    0x8: "CONFIG1",
    0x9: "CLOCK_CONV",
    0xA: "TEST",
    0xB: "CONFIG2",
    0xC: "CONFIG3",
    0xD: "CONFIG4",
}


@dataclass(frozen=True)
class Event:
    line: int
    kind: str
    mode: str
    op: str
    label: str
    size: int
    addr: int
    data: int
    mem_mask: int
    pc: int
    cycles: int
    reason: str = ""
    mame: int | None = None
    rtl: int | None = None

    @property
    def normalized_data(self) -> int:
        if self.kind in {"divergence", "accepted-divergence"}:
            if self.size == 1:
                return self.data & 0xFF
            if self.size == 2:
                return self.data & 0xFFFF
            return self.data & 0xFFFF_FFFF
        if self.size == 1:
            return self.data & 0xFF
        if self.size == 2 and self.mem_mask == 0x0000_FF00:
            return (self.data >> 8) & 0xFF
        if self.size == 2 and self.mem_mask == 0x0000_00FF:
            return self.data & 0xFF
        if self.size == 2:
            return self.data & 0xFFFF
        return self.data & 0xFFFF_FFFF

    @property
    def key(self) -> tuple[str, str, int, int, int]:
        return (self.mode, self.op, self.label, self.addr, self.normalized_data)

    @property
    def normalized_mame(self) -> int | None:
        if self.mame is None:
            return None
        if self.size == 1:
            return self.mame & 0xFF
        if self.size == 2 and self.mem_mask == 0x0000_FF00:
            return (self.mame >> 8) & 0xFF
        if self.size == 2 and self.mem_mask == 0x0000_00FF:
            return self.mame & 0xFF
        if self.size == 2:
            return self.mame & 0xFFFF
        return self.mame & 0xFFFF_FFFF

    @property
    def normalized_rtl(self) -> int | None:
        if self.rtl is None:
            return None
        if self.size == 1:
            return self.rtl & 0xFF
        if self.size == 2:
            return self.rtl & 0xFFFF
        return self.rtl & 0xFFFF_FFFF


def parse_trace(path: Path) -> list[Event]:
    events: list[Event] = []
    with path.open("r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            kind = ""
            if line.startswith("mame-mmio "):
                kind = "mmio"
            elif line.startswith("mame-mmio-missing "):
                kind = "missing"
            elif line.startswith("mame-mmio-unmodeled "):
                kind = "unmodeled"
            elif line.startswith("mame-pc-trap "):
                kind = "pc-trap"
            elif line.startswith("mame-mmio-divergence "):
                kind = "divergence"
            elif line.startswith("mame-mmio-accepted-divergence "):
                kind = "accepted-divergence"
            else:
                continue
            fields: dict[str, str] = {}
            for part in line.split()[1:]:
                key, sep, value = part.partition("=")
                if sep:
                    fields[key] = value
            events.append(
                Event(
                    line=line_no,
                    kind=kind,
                    mode=fields.get("mode", kind),
                    op=fields.get("op", "r"),
                    label=fields["label"],
                    size=int(fields["size"], 0),
                    addr=int(fields["addr"], 0),
                    data=int(fields.get("data", fields.get("rtl", "0")), 0),
                    mem_mask=int(fields["mem_mask"], 0),
                    pc=int(fields["pc"], 0),
                    cycles=int(fields["cycles"], 0),
                    reason=fields.get("reason", ""),
                    mame=int(fields["mame"], 0) if "mame" in fields else None,
                    rtl=int(fields["rtl"], 0) if "rtl" in fields else None,
                )
            )
    return events


def scsi_detail(addr: int) -> str:
    off = addr - 0x5000_F000
    if off < 0 or off > 0x101:
        return ""
    if off >= 0x100:
        return " SCSI_DMA"
    reg = (off >> 4) & 0xF
    return f" scsi_reg={reg:x}:{SCSI_REGS.get(reg, 'RESERVED')}"


def fmt_event(ev: Event) -> str:
    reason = f" reason={ev.reason}" if ev.reason else ""
    compare = ""
    if ev.mame is not None or ev.rtl is not None:
        compare = f" mame=0x{(ev.mame or 0):08x} rtl=0x{(ev.rtl or 0):08x}"
    return (
        f"line={ev.line} kind={ev.kind} mode={ev.mode} op={ev.op} label={ev.label}{reason} "
        f"addr=0x{ev.addr:08x} data=0x{ev.data:08x} norm=0x{ev.normalized_data:x} "
        f"mask=0x{ev.mem_mask:08x} pc=0x{ev.pc:08x} cycles={ev.cycles}{compare}"
        f"{scsi_detail(ev.addr)}"
    )


def run_lengths(events: list[Event]) -> list[tuple[int, Event]]:
    runs: list[tuple[int, Event]] = []
    prev: Event | None = None
    count = 0
    for ev in events:
        if prev is not None and ev.key == prev.key:
            count += 1
            continue
        if prev is not None:
            runs.append((count, prev))
        prev = ev
        count = 1
    if prev is not None:
        runs.append((count, prev))
    return runs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("trace", type=Path)
    ap.add_argument("--top", type=int, default=10)
    ap.add_argument("--tail", type=int, default=8)
    args = ap.parse_args()

    events = parse_trace(args.trace)
    print(f"events={len(events)}")
    if not events:
        return 0

    print(f"first: {fmt_event(events[0])}")
    print(f"last:  {fmt_event(events[-1])}")

    by_label = Counter(ev.label for ev in events)
    by_kind = Counter(ev.kind for ev in events)
    by_mode = Counter(ev.mode for ev in events)
    by_label_op = Counter((ev.label, ev.op) for ev in events)
    print("labels:", " ".join(f"{k}={v}" for k, v in sorted(by_label.items())))
    print("kinds:", " ".join(f"{k}={v}" for k, v in sorted(by_kind.items())))
    print("modes:", " ".join(f"{k}={v}" for k, v in sorted(by_mode.items())))
    print("ops:", " ".join(f"{label}/{op}={count}" for (label, op), count in sorted(by_label_op.items())))

    values: dict[tuple[str, int], set[int]] = defaultdict(set)
    pcs: dict[tuple[str, int], set[int]] = defaultdict(set)
    for ev in events:
        values[(ev.label, ev.addr)].add(ev.normalized_data)
        pcs[(ev.label, ev.addr)].add(ev.pc)

    addr_counts = Counter((ev.label, ev.addr) for ev in events)
    print("hot-addresses:")
    for (label, addr), count in addr_counts.most_common(args.top):
        vals = sorted(values[(label, addr)])
        val_text = ",".join(f"0x{v:x}" for v in vals[:8])
        if len(vals) > 8:
            val_text += ",..."
        print(
            f"  {label} addr=0x{addr:08x} count={count} values={val_text} "
            f"pcs={len(pcs[(label, addr)])}{scsi_detail(addr)}"
        )

    runs = sorted(run_lengths(events), key=lambda item: item[0], reverse=True)
    print("longest-runs:")
    for count, ev in runs[: args.top]:
        print(f"  count={count} {fmt_event(ev)}")

    divergence_events = [
        ev for ev in events
        if ev.kind in {"divergence", "accepted-divergence"}
    ]
    if divergence_events:
        div_counts = Counter(
            (
                ev.kind,
                ev.reason,
                ev.label,
                ev.addr,
                ev.pc,
                ev.normalized_mame,
                ev.normalized_rtl,
            )
            for ev in divergence_events
        )
        print("divergences:")
        for (kind, reason, label, addr, pc, mame_norm, rtl_norm), count in div_counts.most_common(args.top):
            reason_text = f" reason={reason}" if reason else ""
            print(
                f"  {kind}{reason_text} {label} addr=0x{addr:08x} pc=0x{pc:08x} "
                f"count={count} mame_norm=0x{mame_norm or 0:x} rtl_norm=0x{rtl_norm or 0:x}"
                f"{scsi_detail(addr)}"
            )

    print("tail:")
    tail = deque(events, maxlen=args.tail)
    for ev in tail:
        print(f"  {fmt_event(ev)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
