#!/usr/bin/env python3
"""Compare two portable ROM boot architectural checkpoints.

This intentionally compares the replay contract, not exact simulator state.
Cycle counts, stop text, overlay/peripheral model state, traces, and other
transient harness details are ignored.  CPU architectural state and backed
memory images must match.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from rom_boot_arch_checkpoint_summary import (
    fmt_u32,
    parse_checkpoint,
    validate_replayable,
)


REG_NAMES = [f"d{i}" for i in range(8)] + [f"a{i}" for i in range(8)]
CONTROL_NAMES = ("vbr", "cacr", "sfc", "dfc", "usp", "ssp", "isp")
MMU_NAMES = ("tc", "itt0", "itt1", "dtt0", "dtt1", "urp", "srp")
SEGMENT_NAMES = ("q700-rom", "ram", "vram", "magic")


def _fmt(value: int | None) -> str:
    return "-" if value is None else fmt_u32(value)


def compare_values(errors: list[str], label: str, left: int | None, right: int | None) -> None:
    if left != right:
        errors.append(f"{label}: left={_fmt(left)} right={_fmt(right)}")


def compare_dict_key(
    errors: list[str],
    label: str,
    left: dict[str, int],
    right: dict[str, int],
    key: str,
) -> None:
    compare_values(errors, f"{label}.{key}", left.get(key), right.get(key))


def compare_segments(errors: list[str], left_cp, right_cp) -> None:
    for name in SEGMENT_NAMES:
        left = left_cp.segment(name)
        right = right_cp.segment(name)
        if left is None or right is None:
            errors.append(
                f"segment {name}: left={'present' if left else 'missing'} "
                f"right={'present' if right else 'missing'}"
            )
            continue

        if left.base != right.base:
            errors.append(
                f"segment {name} base: left={fmt_u32(left.base)} "
                f"right={fmt_u32(right.base)}"
            )
        if left.size != right.size:
            errors.append(
                f"segment {name} size: left=0x{left.size:x} right=0x{right.size:x}"
            )
        if left.encoding != right.encoding:
            errors.append(
                f"segment {name} encoding: left={left.encoding} right={right.encoding}"
            )
        if left.default_value != right.default_value:
            errors.append(
                f"segment {name} default: left={left.default_value!r} "
                f"right={right.default_value!r}"
            )
        if left.material_bytes != right.material_bytes:
            errors.append(
                f"segment {name} material_bytes: left={left.material_bytes} "
                f"right={right.material_bytes}"
            )
        if left.chunks != right.chunks:
            errors.append(
                f"segment {name} chunks: left={left.chunks} right={right.chunks}"
            )
        if left.data_bytes != right.data_bytes:
            errors.append(
                f"segment {name} data_bytes: left={left.data_bytes} "
                f"right={right.data_bytes}"
            )
        if left.checksum_fnv1a64 != right.checksum_fnv1a64:
            errors.append(
                f"segment {name} checksum_fnv1a64: "
                f"left=0x{(left.checksum_fnv1a64 or 0):016x} "
                f"right=0x{(right.checksum_fnv1a64 or 0):016x}"
            )


def compare_checkpoints(left_path: Path, right_path: Path, *, check_replayable: bool) -> list[str]:
    left = parse_checkpoint(left_path)
    right = parse_checkpoint(right_path)

    errors: list[str] = []
    if check_replayable:
        errors.extend(f"left replayability: {error}" for error in validate_replayable(left))
        errors.extend(f"right replayability: {error}" for error in validate_replayable(right))

    compare_values(errors, "run.committed", left.run_committed, right.run_committed)
    compare_values(errors, "arch.ccr", left.ccr, right.ccr)
    compare_values(errors, "arch.sr", left.sr, right.sr)
    compare_values(errors, "pc.next", left.pc_next, right.pc_next)
    compare_values(errors, "pc.commit_pc", left.pc_commit_pc, right.pc_commit_pc)
    compare_values(errors, "pc.committed", left.pc_committed, right.pc_committed)

    for reg in REG_NAMES:
        compare_dict_key(errors, "reg", left.regs, right.regs, reg)
    for key in CONTROL_NAMES:
        compare_dict_key(errors, "control", left.control, right.control, key)
    for key in MMU_NAMES:
        compare_dict_key(errors, "mmu", left.mmu, right.mmu, key)

    compare_segments(errors, left, right)
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("left", type=Path, help="continuous/reference checkpoint")
    parser.add_argument("right", type=Path, help="replayed checkpoint")
    parser.add_argument(
        "--check-replayable",
        action="store_true",
        help="also validate each checkpoint against the replay contract",
    )
    args = parser.parse_args()

    for path in (args.left, args.right):
        if not path.is_file():
            print(f"[rom-boot-arch-compare] checkpoint not found: {path}", file=sys.stderr)
            return 2

    errors = compare_checkpoints(args.left, args.right, check_replayable=args.check_replayable)
    if errors:
        print(
            f"[rom-boot-arch-compare] FAIL left={args.left} right={args.right}",
            file=sys.stderr,
        )
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1

    left = parse_checkpoint(args.left)
    print(
        f"[rom-boot-arch-compare] PASS left={args.left} right={args.right} "
        f"committed={left.run_committed} pc_next={_fmt(left.pc_next)} "
        f"ccr={_fmt(left.ccr)} sr={_fmt(left.sr)}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
