#!/usr/bin/env python3
"""Sample PC streams from MAME trace + RTL pc_dump, find first divergence.

MAME trace lines look like:
    XXXXXXXX: <opcode> <disasm>
RTL pc_dump lines look like:
    <idx> <committed_uop> 0xXXXXXXXX

We extract the PC sequence from each, sample at every Nth, and diff.
"""
import argparse
import re
import sys

MAME_RE = re.compile(r"^([0-9A-Fa-f]{8}):")
RTL_RE = re.compile(r"^\s*\d+\s+\d+\s+0x([0-9A-Fa-f]+)\s*$")


def load_mame_pcs(path, limit):
    pcs = []
    with open(path, "r", errors="replace") as f:
        for line in f:
            m = MAME_RE.match(line)
            if m:
                pcs.append(int(m.group(1), 16))
                if limit and len(pcs) >= limit:
                    break
    return pcs


def load_rtl_pcs(path, limit):
    pcs = []
    with open(path, "r", errors="replace") as f:
        for line in f:
            m = RTL_RE.match(line)
            if m:
                pcs.append(int(m.group(1), 16))
                if limit and len(pcs) >= limit:
                    break
    return pcs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mame", required=True)
    ap.add_argument("--rtl", required=True)
    ap.add_argument("--every", type=int, default=1000,
                    help="sampling stride")
    ap.add_argument("--limit", type=int, default=0,
                    help="cap raw PCs loaded (0 = all)")
    ap.add_argument("--align-to", type=lambda s: int(s, 0), default=None,
                    help="trim leading PCs from each side until both start at this PC")
    ap.add_argument("--skip-mame", type=int, default=0)
    ap.add_argument("--skip-rtl", type=int, default=0)
    ap.add_argument("--skew-rtl", type=int, default=0,
                    help="extra trim of N entries off RTL after alignment")
    ap.add_argument("--show-context", type=int, default=4)
    args = ap.parse_args()

    print(f"loading mame: {args.mame}")
    mame = load_mame_pcs(args.mame, args.limit)
    print(f"  {len(mame)} PCs")
    print(f"loading rtl: {args.rtl}")
    rtl = load_rtl_pcs(args.rtl, args.limit)
    print(f"  {len(rtl)} PCs")

    if args.skip_mame:
        mame = mame[args.skip_mame:]
        print(f"  skip-mame={args.skip_mame} -> {len(mame)} PCs")
    if args.skip_rtl:
        rtl = rtl[args.skip_rtl:]
        print(f"  skip-rtl={args.skip_rtl} -> {len(rtl)} PCs")

    if args.align_to is not None:
        target = args.align_to
        try:
            m_idx = mame.index(target)
            r_idx = rtl.index(target)
        except ValueError:
            print(f"FATAL: align-to PC 0x{target:08x} not found "
                  f"(mame_has={target in mame}, rtl_has={target in rtl})")
            return 2
        mame = mame[m_idx:]
        rtl = rtl[r_idx:]
        print(f"  aligned at 0x{target:08x}: mame trim {m_idx}, rtl trim {r_idx}")
        print(f"  -> mame={len(mame)} rtl={len(rtl)}")

    if args.skew_rtl:
        # mame[i] vs rtl[i + skew_rtl]
        rtl = rtl[args.skew_rtl:]
        print(f"  skew-rtl={args.skew_rtl} -> rtl now {len(rtl)} PCs")

    n = min(len(mame), len(rtl))
    print(f"comparing {n} PCs in lockstep, stride={args.every}")

    samples = list(range(0, n, args.every))
    if samples and samples[-1] != n - 1:
        samples.append(n - 1)
    diverge_at = None
    for i in samples:
        if mame[i] != rtl[i]:
            diverge_at = i
            break

    if diverge_at is None:
        print(f"OK: no divergence among {len(samples)} samples (stride {args.every})")
        # show last 5 samples to confirm
        for i in samples[-5:]:
            print(f"  [{i:>9}] mame=0x{mame[i]:08x} rtl=0x{rtl[i]:08x}")
        # if streams unequal length
        if len(mame) != len(rtl):
            print(f"  NOTE: lengths differ mame={len(mame)} rtl={len(rtl)}")
        return 0

    # Find prior aligned sample
    prev_sample = samples[samples.index(diverge_at) - 1] if samples.index(diverge_at) > 0 else 0
    print(f"DIVERGE at index {diverge_at} "
          f"(mame=0x{mame[diverge_at]:08x} rtl=0x{rtl[diverge_at]:08x})")
    print(f"prior aligned sample at index {prev_sample} "
          f"(mame=0x{mame[prev_sample]:08x} rtl=0x{rtl[prev_sample]:08x})")
    print(f"=> bisect window [{prev_sample}, {diverge_at}] (size={diverge_at-prev_sample})")

    # Bisect within the window: find exact first divergence
    lo, hi = prev_sample, diverge_at
    # invariant: mame[lo] == rtl[lo], mame[hi] != rtl[hi]
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if mame[mid] == rtl[mid]:
            lo = mid
        else:
            hi = mid
    print(f"first divergent index: {hi}")
    print(f"  last match @ {lo}: 0x{mame[lo]:08x}")
    print(f"  first diff @ {hi}: mame=0x{mame[hi]:08x} rtl=0x{rtl[hi]:08x}")

    # Show context window
    ctx = args.show_context
    print(f"\ncontext (lo-{ctx} ... hi+{ctx}):")
    a = max(0, lo - ctx)
    b = min(n, hi + ctx + 1)
    for i in range(a, b):
        marker = ""
        if i == lo:
            marker = "  <-- last match"
        elif i == hi:
            marker = "  <-- FIRST DIFF"
        same = "==" if mame[i] == rtl[i] else "!="
        print(f"  [{i:>9}] mame=0x{mame[i]:08x} {same} rtl=0x{rtl[i]:08x}{marker}")

    return 1


if __name__ == "__main__":
    sys.exit(main())
