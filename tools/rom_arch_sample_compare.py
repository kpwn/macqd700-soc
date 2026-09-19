#!/usr/bin/env python3
"""Compare RTL and Musashi ROM architectural sample logs."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


DEFAULT_FIELDS = [
    "pc",
    "sr",
    "ccr",
    "d0",
    "d1",
    "d2",
    "d3",
    "d4",
    "d5",
    "d6",
    "d7",
    "a0",
    "a1",
    "a2",
    "a3",
    "a4",
    "a5",
    "a6",
    "a7",
    "sfc",
    "dfc",
    "vbr",
]


def parse_sample_log(path: Path) -> dict[int, dict[str, str]]:
    columns: list[str] | None = None
    samples: dict[int, dict[str, str]] = {}

    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("# columns:"):
            columns = line.split(":", 1)[1].strip().split()
            continue
        if line.startswith("#"):
            continue
        parts = line.split()
        if columns is None:
            raise ValueError(f"{path}: data row before '# columns:' header")
        if len(parts) != len(columns):
            raise ValueError(
                f"{path}: expected {len(columns)} columns, got {len(parts)}: {line}"
            )
        row = dict(zip(columns, parts, strict=True))
        samples[int(row["committed"], 0)] = row

    return samples


def compare_samples(
    rtl: dict[int, dict[str, str]],
    musashi: dict[int, dict[str, str]],
    fields: list[str],
    *,
    max_diffs: int,
) -> list[str]:
    diffs: list[str] = []
    common = sorted(set(rtl) & set(musashi))
    if not common:
        return ["no shared committed-count samples"]

    for committed in common:
        left = rtl[committed]
        right = musashi[committed]
        for field in fields:
            if field not in left or field not in right:
                diffs.append(f"{committed}: missing field {field}")
            elif normalize_value(left[field]) != normalize_value(right[field]):
                diffs.append(
                    f"{committed}: {field} rtl={left[field]} musashi={right[field]}"
                )
            if len(diffs) >= max_diffs:
                return diffs
    return diffs


def normalize_value(value: str) -> int | str:
    try:
        return int(value, 0)
    except ValueError:
        return value.lower()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("rtl", type=Path)
    ap.add_argument("musashi", type=Path)
    ap.add_argument(
        "--fields",
        default=",".join(DEFAULT_FIELDS),
        help="comma-separated field list to compare",
    )
    ap.add_argument("--max-diffs", type=int, default=20)
    args = ap.parse_args()

    fields = [field.strip() for field in args.fields.split(",") if field.strip()]
    rtl = parse_sample_log(args.rtl)
    musashi = parse_sample_log(args.musashi)
    diffs = compare_samples(rtl, musashi, fields, max_diffs=args.max_diffs)
    common = sorted(set(rtl) & set(musashi))

    if diffs:
        print(
            f"[rom-arch-sample-compare] MISMATCH shared={len(common)} "
            f"rtl={len(rtl)} musashi={len(musashi)}"
        )
        for diff in diffs:
            print(diff)
        return 1

    print(
        f"[rom-arch-sample-compare] PASS shared={len(common)} "
        f"rtl={len(rtl)} musashi={len(musashi)} fields={','.join(fields)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
