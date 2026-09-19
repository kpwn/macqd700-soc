#!/usr/bin/env python3
"""rom_trace_diff.py — diff our ROM-boot trace against a normalised
MAME trace and print the first divergence with context.

Task #101 bring-up tooling.  Both inputs are expected in the format
emitted by tb_rom_boot.cpp / tools/mame_trace_normalize.py:

    <pc-hex-8>  <ir-hex-4>  <ccr-hex-2>

Comment lines (`# ...`) and blank lines are ignored.

Diff logic
──────────

- Walk the two streams in lockstep.
- Divergence = first index where (pc, ir) mismatch OR one stream
  ran out before the other.  CCR mismatches are reported but by
  default do NOT count as a divergence (CCR state is late to the
  party during ROM boot and our model may diverge on flag bits
  without the control flow changing; --strict-ccr flips this).
- With --pc-only, ignore IR mismatches.  This is useful with MAME
  debugger traces that expose PC/SR but not raw opcode words.

Output
──────

Writes a summary to stdout and, if --context N is given, the
±N-line window of each trace around the divergence.  Exits 0 only for
a full match by default.  --allow-prefix-match keeps the old frontier
workflow behavior where a divergence after --min-match lines is a
successful bounded-prefix comparison.

Usage
─────

    ./tools/rom_trace_diff.py \\
        build/sim/rom_boot_trace.log \\
        traces/macqd700_reference.tr \\
        --context 20 --min-match 100
"""

from __future__ import annotations

import argparse
import sys
from typing import Iterable, List, Optional, Tuple


def load_trace(path: str) -> List[Tuple[int, int, int]]:
    out: List[Tuple[int, int, int]] = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for ln in f:
            s = ln.strip()
            if not s or s.startswith("#"):
                continue
            parts = s.split()
            if len(parts) < 2:
                continue
            try:
                pc  = int(parts[0], 16)
                ir  = int(parts[1], 16)
                ccr = int(parts[2], 16) if len(parts) >= 3 else 0
            except ValueError:
                continue
            out.append((pc, ir, ccr))
    return out


def print_context(tag: str, trace: List[Tuple[int, int, int]],
                  idx: int, radius: int) -> None:
    lo = max(0, idx - radius)
    hi = min(len(trace), idx + radius + 1)
    print(f"--- {tag} context [{lo}..{hi-1}] (divergence at index {idx}) ---")
    for i in range(lo, hi):
        marker = " >>" if i == idx else "   "
        pc, ir, ccr = trace[i]
        print(f"{marker} [{i:5d}] pc={pc:08x} ir={ir:04x} ccr={ccr:02x}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("ours",   help="our trace (build/sim/rom_boot_trace.log)")
    ap.add_argument("theirs", help="reference (normalised MAME trace)")
    ap.add_argument("--context", type=int, default=20,
                    help="lines of context around divergence (default 20)")
    ap.add_argument("--min-match", type=int, default=1,
                    help="minimum matching lines for --allow-prefix-match")
    ap.add_argument("--allow-prefix-match", action="store_true",
                    help="return success when at least --min-match lines match before divergence")
    ap.add_argument("--strict-ccr", action="store_true",
                    help="treat CCR mismatches as divergences too")
    ap.add_argument("--pc-only", action="store_true",
                    help="compare only committed PCs, ignoring IR mismatches")
    args = ap.parse_args()

    ours   = load_trace(args.ours)
    theirs = load_trace(args.theirs)
    n = min(len(ours), len(theirs))

    print(f"[rom-trace-diff] ours   = {args.ours} ({len(ours)} lines)")
    print(f"[rom-trace-diff] theirs = {args.theirs} ({len(theirs)} lines)")

    first_div: Optional[int] = None
    for i in range(n):
        opc, oir, occr = ours[i]
        tpc, tir, tccr = theirs[i]
        if opc != tpc or (not args.pc_only and oir != tir):
            first_div = i
            break
        if args.strict_ccr and occr != tccr:
            first_div = i
            break

    if first_div is None and len(ours) == len(theirs):
        print(f"[rom-trace-diff] FULL MATCH across {n} lines")
        return 0
    if first_div is None:
        # One stream is a prefix of the other.
        longer = "ours" if len(ours) > len(theirs) else "theirs"
        print(f"[rom-trace-diff] matched for {n} lines; {longer} has more")
        if args.allow_prefix_match and n >= args.min_match:
            return 0
        return 1

    opc, oir, occr = ours[first_div]
    tpc, tir, tccr = theirs[first_div]
    print(f"[rom-trace-diff] FIRST DIVERGENCE at index {first_div}")
    print(f"  ours   : pc={opc:08x} ir={oir:04x} ccr={occr:02x}")
    print(f"  theirs : pc={tpc:08x} ir={tir:04x} ccr={tccr:02x}")
    print(f"[rom-trace-diff] matched {first_div} lines before divergence")

    if args.context > 0:
        print()
        print_context("OURS",   ours,   first_div, args.context)
        print()
        print_context("THEIRS", theirs, first_div, args.context)

    if args.allow_prefix_match and first_div >= args.min_match:
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
