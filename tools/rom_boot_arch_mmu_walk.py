#!/usr/bin/env python3
"""Walk current RTL-style 68040 page tables from an arch checkpoint."""

from __future__ import annotations

import argparse
import shlex
from dataclasses import dataclass, field
from pathlib import Path


DEFAULT_VAS = (0x00000000, 0x00000028, 0x38000000, 0x38000028, 0x40800502)


def parse_u32(text: str) -> int:
    return int(text, 0) & 0xFFFFFFFF


def kv_args(parts: list[str]) -> dict[str, str]:
    fields: dict[str, str] = {}
    for part in parts:
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        fields[key] = value
    return fields


def fmt32(value: int) -> str:
    return f"0x{value & 0xFFFFFFFF:08x}"


@dataclass
class Segment:
    name: str
    base: int
    size: int
    encoding: str
    default_value: int | None = None
    bytes_by_addr: dict[int, int] = field(default_factory=dict)

    def contains(self, addr: int, size: int = 1) -> bool:
        return self.base <= addr and addr + size <= self.base + self.size

    def read8(self, addr: int) -> int:
        if not self.contains(addr):
            raise ValueError(f"{self.name}: address {fmt32(addr)} outside segment")
        value = self.bytes_by_addr.get(addr)
        if value is not None:
            return value
        if self.default_value is not None:
            return self.default_value
        raise ValueError(f"{self.name}: address {fmt32(addr)} missing from full segment")

    def read32(self, addr: int) -> int:
        value = 0
        for i in range(4):
            value = (value << 8) | self.read8(addr + i)
        return value


@dataclass
class Checkpoint:
    path: Path
    mmu: dict[str, int] = field(default_factory=dict)
    segments: dict[str, Segment] = field(default_factory=dict)


def parse_checkpoint(path: Path) -> Checkpoint:
    cp = Checkpoint(path=path)
    current: Segment | None = None

    with path.open("r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("mmu "):
                fields = kv_args(shlex.split(line)[1:])
                cp.mmu = {key: parse_u32(value) for key, value in fields.items()}
                continue
            if line.startswith("segment "):
                fields = kv_args(shlex.split(line)[1:])
                if not {"name", "base", "size", "encoding"} <= fields.keys():
                    current = None
                    continue
                current = Segment(
                    name=fields["name"],
                    base=parse_u32(fields["base"]),
                    size=int(fields["size"], 0),
                    encoding=fields["encoding"],
                    default_value=parse_u32(fields["default"])
                    if "default" in fields
                    else None,
                )
                cp.segments[current.name] = current
                continue
            if line == "endsegment" or line.startswith("endsegment "):
                current = None
                continue
            if line.startswith("data ") and current is not None:
                fields = kv_args(shlex.split(line)[1:])
                off = int(fields["off"], 0)
                data = bytes.fromhex(fields["hex"])
                for i, value in enumerate(data):
                    current.bytes_by_addr[current.base + off + i] = value

    if "ram" not in cp.segments:
        raise ValueError(f"{path}: checkpoint has no ram segment")
    return cp


def ttr_match(ttr: int, va: int, supervisor: bool) -> bool:
    enabled = (ttr >> 15) & 1
    if not enabled:
        return False
    base = (ttr >> 24) & 0xFF
    mask = (ttr >> 16) & 0xFF
    s_field = (ttr >> 13) & 0x3
    if s_field == 0:
        s_ok = not supervisor
    elif s_field == 1:
        s_ok = supervisor
    else:
        s_ok = True
    va_hi = (va >> 24) & 0xFF
    return s_ok and ((va_hi & (~mask & 0xFF)) == (base & (~mask & 0xFF)))


def read_pte(ram: Segment, addr: int) -> int:
    return ram.read32(addr)


def walk_one(cp: Checkpoint, va: int, *, instruction: bool, supervisor: bool, write: bool) -> list[str]:
    ram = cp.segments["ram"]
    tc = cp.mmu.get("tc", 0)
    lines: list[str] = []
    kind = "inst" if instruction else "data"
    lines.append(f"va={fmt32(va)} kind={kind} supervisor={int(supervisor)} write={int(write)}")
    lines.append(
        "  mmu "
        f"tc={fmt32(tc)} itt0={fmt32(cp.mmu.get('itt0', 0))} "
        f"itt1={fmt32(cp.mmu.get('itt1', 0))} dtt0={fmt32(cp.mmu.get('dtt0', 0))} "
        f"dtt1={fmt32(cp.mmu.get('dtt1', 0))} urp={fmt32(cp.mmu.get('urp', 0))} "
        f"srp={fmt32(cp.mmu.get('srp', 0))}"
    )

    if not ((tc >> 15) & 1):
        lines.append(f"  result pa={fmt32(va)} source=mmu-disabled")
        return lines

    ttr_names = ("itt0", "itt1") if instruction else ("dtt0", "dtt1")
    for name in ttr_names:
        ttr = cp.mmu.get(name, 0)
        if ttr_match(ttr, va, supervisor):
            wp_fault = (not instruction) and write and bool((ttr >> 2) & 1)
            if wp_fault:
                lines.append(f"  result fault=wp source={name} ttr={fmt32(ttr)}")
            else:
                lines.append(f"  result pa={fmt32(va)} source={name}-transparent ttr={fmt32(ttr)}")
            return lines

    page_size_8k = bool((tc >> 14) & 1)
    page_bits = 13 if page_size_8k else 12
    root = cp.mmu.get("srp" if supervisor else "urp", 0) & 0xFFFFFFFC
    l1_idx = (va >> 25) & 0x7F
    l2_idx = (va >> 18) & 0x7F
    leaf_idx = (va >> page_bits) & (0x1F if page_size_8k else 0x3F)
    lines.append(
        f"  walk root={fmt32(root)} page={'8k' if page_size_8k else '4k'} "
        f"l1_idx=0x{l1_idx:02x} l2_idx=0x{l2_idx:02x} leaf_idx=0x{leaf_idx:02x}"
    )

    acc_wp = False
    try:
        l1_addr = root + l1_idx * 4
        l1 = read_pte(ram, l1_addr)
        lines.append(f"  l1 addr={fmt32(l1_addr)} pte={fmt32(l1)} dt={l1 & 3}")
        if (l1 & 3) != 2:
            lines.append("  result fault=invalid-pointer level=l1")
            return lines
        acc_wp = acc_wp or bool((l1 >> 2) & 1)

        l2_addr = (l1 & 0xFFFFFFF0) + l2_idx * 4
        l2 = read_pte(ram, l2_addr)
        lines.append(f"  l2 addr={fmt32(l2_addr)} pte={fmt32(l2)} dt={l2 & 3}")
        if (l2 & 3) != 2:
            lines.append("  result fault=invalid-pointer level=l2")
            return lines
        acc_wp = acc_wp or bool((l2 >> 2) & 1)

        leaf_addr = (l2 & 0xFFFFFFF0) + leaf_idx * 4
        leaf = read_pte(ram, leaf_addr)
        lines.append(f"  leaf addr={fmt32(leaf_addr)} pte={fmt32(leaf)} dt={leaf & 3}")
        if (leaf & 3) in (2, 3):
            lines.append("  result fault=indirect-unsupported level=leaf")
            return lines
        if (leaf & 3) != 1:
            lines.append("  result fault=invalid-page level=leaf")
            return lines

        acc_wp = acc_wp or bool((leaf >> 2) & 1)
        sup_only = bool((leaf >> 7) & 1)
        if write and acc_wp:
            lines.append("  result fault=write-protect")
            return lines
        if sup_only and not supervisor:
            lines.append("  result fault=supervisor-only")
            return lines

        frame_mask = 0xFFFFE000 if page_size_8k else 0xFFFFF000
        offset_mask = 0x00001FFF if page_size_8k else 0x00000FFF
        pa = (leaf & frame_mask) | (va & offset_mask)
        flags = []
        if acc_wp:
            flags.append("wp")
        if sup_only:
            flags.append("sup")
        if (leaf >> 6) & 1:
            flags.append("ci")
        if (leaf >> 4) & 1:
            flags.append("m")
        if (leaf >> 3) & 1:
            flags.append("u")
        flag_text = ",".join(flags) if flags else "-"
        lines.append(f"  result pa={fmt32(pa)} flags={flag_text}")
        return lines
    except ValueError as exc:
        lines.append(f"  result fault=checkpoint-read-error detail={exc}")
        return lines


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("va", nargs="*", help="virtual address to walk")
    parser.add_argument("--instruction", action="store_true", help="use ITT instead of DTT")
    parser.add_argument("--user", action="store_true", help="walk as user mode")
    parser.add_argument("--write", action="store_true", help="check write permissions")
    args = parser.parse_args()

    cp = parse_checkpoint(args.checkpoint)
    vas = [parse_u32(text) for text in args.va] if args.va else list(DEFAULT_VAS)
    for idx, va in enumerate(vas):
        if idx:
            print()
        print(
            "\n".join(
                walk_one(
                    cp,
                    va,
                    instruction=args.instruction,
                    supervisor=not args.user,
                    write=args.write,
                )
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
