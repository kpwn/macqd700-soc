#!/usr/bin/env python3
"""Alignment-tolerant PC diff for MAME trace vs RTL pc_dump.

Walks both streams together. On mismatch, look ahead K=8 entries on
each side; if mame[i+a] == rtl[j+b] is found, emit the skipped chunks
on each side (RTL missing mame[i..i+a-1], MAME missing rtl[j..j+b-1])
and continue. Caps total reports.
"""
import argparse
import re
import sys

MAME_RE = re.compile(r"^([0-9A-Fa-f]{8}):")
RTL_RE = re.compile(r"^\s*\d+\s+\d+\s+0x([0-9A-Fa-f]+)\s*$")


def load_mame(path, limit):
    pcs = []
    with open(path, "r", errors="replace") as f:
        for line in f:
            m = MAME_RE.match(line)
            if m:
                pcs.append(int(m.group(1), 16))
                if limit and len(pcs) >= limit:
                    break
    return pcs


def load_rtl(path, limit):
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
    ap.add_argument("--align-to", type=lambda s: int(s, 0), required=True)
    ap.add_argument("--lookahead", type=int, default=16)
    ap.add_argument("--max-events", type=int, default=20)
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    print(f"loading mame: {args.mame}")
    mame = load_mame(args.mame, args.limit)
    print(f"  {len(mame)} PCs")
    print(f"loading rtl: {args.rtl}")
    rtl = load_rtl(args.rtl, args.limit)
    print(f"  {len(rtl)} PCs")

    target = args.align_to
    try:
        mi = mame.index(target)
        ri = rtl.index(target)
    except ValueError:
        print("FATAL: align-to PC not found")
        return 2
    print(f"align at 0x{target:08x}: mame skip {mi}, rtl skip {ri}")

    K = args.lookahead
    events = 0
    missing_in_rtl = 0   # PCs MAME executed that RTL never surfaced
    missing_in_mame = 0  # PCs RTL surfaced that MAME didn't execute
    matched = 0
    while mi < len(mame) and ri < len(rtl):
        if mame[mi] == rtl[ri]:
            mi += 1
            ri += 1
            matched += 1
            continue
        # Mismatch — try to re-sync.
        # Find smallest (a + b) such that mame[mi+a] == rtl[ri+b], a,b in [0..K]
        best = None
        for total in range(1, 2 * K + 1):
            for a in range(0, min(K, total) + 1):
                b = total - a
                if b > K:
                    continue
                if mi + a >= len(mame) or ri + b >= len(rtl):
                    continue
                if mame[mi + a] == rtl[ri + b]:
                    best = (a, b)
                    break
            if best:
                break
        if not best:
            print(f"  HARD DIVERGE at mame[{mi}]=0x{mame[mi]:08x} "
                  f"rtl[{ri}]=0x{rtl[ri]:08x} — cannot re-sync within K={K}")
            break
        a, b = best
        events += 1
        missing_in_rtl += a
        missing_in_mame += b
        if events <= args.max_events:
            mame_block = ", ".join(f"0x{mame[mi+x]:08x}" for x in range(a))
            rtl_block = ", ".join(f"0x{rtl[ri+x]:08x}" for x in range(b))
            ctx_pre = (mame[mi-1] if mi > 0 else 0,
                       rtl[ri-1] if ri > 0 else 0)
            print(f"\nEVENT #{events} @ mame_idx={mi} rtl_idx={ri} "
                  f"(prev common 0x{ctx_pre[0]:08x})")
            print(f"  RTL missing (a={a}): [{mame_block}]")
            print(f"  MAME missing (b={b}): [{rtl_block}]")
            print(f"  resync at mame[{mi+a}]=rtl[{ri+b}]=0x{mame[mi+a]:08x}")
        mi += a
        ri += b

    print()
    print(f"summary: matched={matched} events={events}")
    print(f"  PCs RTL missed (executed by MAME, not surfaced by RTL): {missing_in_rtl}")
    print(f"  PCs RTL surfaced extra (not in MAME's stream): {missing_in_mame}")
    print(f"  consumed mame={mi} rtl={ri} of {len(mame)}/{len(rtl)}")


if __name__ == "__main__":
    sys.exit(main())
