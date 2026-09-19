#!/usr/bin/env python3
"""Analyse a p141 ILA capture: find the freeze and show how the machine got there.

    tools/p141_analyse_capture.py /tmp/p141_walker_stall.csv [--window N]

This exists because the CSV is 4096 rows x 5 words and the answer is a handful
of TRANSITIONS buried in it. The specific question it answers -- the one a
frozen CSR read structurally cannot -- is which of these three happened:

  (a) a quiesce term NEVER cleared;
  (b) a term cleared and was RE-ARMED (by a grant hand-over);
  (c) a hand-over landed one cycle LATE relative to drain entry.

All three have identical terminal state and imply different fixes. So the
output below is organised around edges, not levels: the last change of each
quiesce term, every ownership hand-over, and the ExceptionUnit state timeline,
all indexed relative to the freeze sample.

POLARITY: stallDc bits [13:0] block when SET; bits [17:14] are *Done flags and
block when CLEAR. Encoded once, here.
"""
import csv, sys, argparse

DC_HI = ["resetSweepBusy","busy","ldS1Valid","ldS2Valid","loadShadowValid",
         "earlyProbeValid","pendingStoreMiss","pendingWtKickoff","s0Valid",
         "stS1Valid","stS2Valid","stS3Valid","serialStoreInFlight",
         "storeMissBarrier"]
DC_LO = ["stAwDone","stWDone","evictAwDone","evictWDone"]      # block when CLEAR
EXC_ST = ["IDLE","E_DRAIN","E_STORE","E_STWAIT","E_VECREQ","E_VECWAIT","E_REDIR",
          "R_DRAIN","R_SRREQ","R_SRWAIT","R_PCREQ","R_PCWAIT","R_PCREQ2",
          "R_PCWAIT2","R_FMTREQ","R_FMTWAIT","R_REDIR","S_DRAIN","S_APPLY",
          "S_MAINTWAIT","S_REDIR"]
WK_ST = ["IDLE","RD_ROOT","RD_PTR","RD_PAGE","FINISH"]
OWNER = {0:"CORE", 1:"ITLB", 2:"DTLB", 3:"?3"}

def bit(v, i):  return (v >> i) & 1
def fld(v, o, w): return (v >> o) & ((1 << w) - 1)

def exc_state(v):
    for i, n in enumerate(EXC_ST):
        if bit(v, 6 + i): return n
    return "(none)"

def wk_state(v, base):
    for i, n in enumerate(WK_ST):
        if bit(v, base + i): return n
    return "(none)"

def dc_blockers(v):
    out = [n for i, n in enumerate(DC_HI) if bit(v, i)]
    out += ["!" + n for i, n in enumerate(DC_LO) if not bit(v, 14 + i)]
    so = fld(v, 18, 4)
    if so: out.append(f"storeOutstanding={so}")
    return out

def load(path):
    rows = []
    with open(path) as f:
        rd = csv.DictReader(l for l in f if not l.startswith("#"))
        cols = {c.lower().strip(): c for c in (rd.fieldnames or [])}
        def pick(*cands):
            for c in cands:
                for k, orig in cols.items():
                    if c in k: return orig
            return None
        c_dc    = pick("cdb0_data")
        c_grant = pick("cdb1_data")
        c_exc   = pick("a7_writeback_val")
        c_walk  = pick("diag_fault_addr")
        c_trig  = pick("ic_mshr_snap")
        if not all([c_dc, c_grant, c_exc, c_walk, c_trig]):
            sys.exit(f"could not find all 5 probe columns in {rd.fieldnames}")
        # write_hw_ila_data emits a SECOND header row ("Radix - UNSIGNED,HEX,...")
        # after the column names. It is not data and must be skipped, or the
        # first "sample" parses as garbage and shifts every index by one.
        for r in rd:
            first = (r.get(rd.fieldnames[0]) or "").strip()
            if first.lower().startswith("radix"):
                continue
            def iv(c):
                s = (r[c] or "").strip()
                if not s: return 0
                if s.lower().startswith("0x"): return int(s, 16)
                # Probe columns are written with Radix HEX and NO 0x prefix.
                try:    return int(s, 16)
                except ValueError: return int(s)
            rows.append((iv(c_dc), iv(c_grant), iv(c_exc), iv(c_walk), iv(c_trig)))
    return rows

ap = argparse.ArgumentParser()
ap.add_argument("csv"); ap.add_argument("--window", type=int, default=24)
a = ap.parse_args()
rows = load(a.csv)
print(f"{len(rows)} samples")

trig = [r[4] for r in rows]
# The freeze: last index where the retire counter still changed.
freeze = None
for i in range(len(trig) - 1, 0, -1):
    if trig[i] != trig[i-1]:
        freeze = i
        break
if freeze is None:
    print("retire counter NEVER changes in this capture -- either the whole")
    print("window is post-freeze, or this boot did not wedge. Cross-check")
    print("OFF_INST_LO on the board before drawing any conclusion.")
    freeze = 0
else:
    print(f"freeze at sample {freeze} (retire {trig[freeze-1]:#010x} -> {trig[freeze]:#010x}, "
          f"then static for {len(rows)-freeze-1} samples)")

print(f"\n== quiesce terms: LAST edge before the freeze ==")
print("(a term whose last edge is far before the freeze NEVER cleared;")
print(" one that toggles right at the freeze was RE-ARMED -- that is the")
print(" (a) vs (b) discrimination this capture exists to make)")
for i, name in enumerate(DC_HI + DC_LO):
    b = i if i < len(DC_HI) else 14 + (i - len(DC_HI))
    seq = [bit(r[0], b) for r in rows]
    last = None
    for k in range(1, len(seq)):
        if seq[k] != seq[k-1]: last = k
    fin = seq[-1]
    blocking = (fin == 1) if i < len(DC_HI) else (fin == 0)
    mark = " <-- BLOCKING at end" if blocking else ""
    if last is None:
        print(f"  {name:<20} constant {fin}{mark}")
    else:
        print(f"  {name:<20} last edge @{last} (rel {last-freeze:+d}), final {fin}{mark}")

print(f"\n== D-cache port ownership hand-overs ==")
prev = None
n = 0
for i, r in enumerate(rows):
    cur = (fld(r[1], 0, 2), fld(r[1], 2, 2))
    if cur != prev:
        if prev is not None:
            print(f"  @{i} (rel {i-freeze:+d}) ld {OWNER[prev[0]]}->{OWNER[cur[0]]}"
                  f"  st {OWNER[prev[1]]}->{OWNER[cur[1]]}")
            n += 1
        prev = cur
if n == 0:
    print(f"  none in this window; ld={OWNER[prev[0]]} st={OWNER[prev[1]]} throughout")

print(f"\n== ExceptionUnit state timeline ==")
prev = None
for i, r in enumerate(rows):
    s = exc_state(r[2])
    if s != prev:
        print(f"  @{i} (rel {i-freeze:+d}) {s}")
        prev = s

print(f"\n== DTLB walker state timeline ==")
prev = None
for i, r in enumerate(rows):
    s = wk_state(r[3], 0)
    if s != prev:
        print(f"  @{i} (rel {i-freeze:+d}) {s}"
              f"  cmd v/r={bit(r[3],6)}/{bit(r[3],7)} rsp={bit(r[3],8)}")
        prev = s

lo = max(0, freeze - a.window); hi = min(len(rows), freeze + 8)
print(f"\n== per-sample detail, samples {lo}..{hi-1} ==")
for i in range(lo, hi):
    dc, gr, ex, wk, tr = rows[i]
    print(f"  @{i:<5}(rel {i-freeze:+4d}) retire={tr:#010x} exc={exc_state(ex):<11}"
          f" ld={OWNER[fld(gr,0,2)]} st={OWNER[fld(gr,2,2)]}"
          f" wedge={bit(gr,10)} dtlb={wk_state(wk,0):<8}")
    print(f"        blocking: {', '.join(dc_blockers(dc)) or '(none)'}")

final = rows[-1]
print("\n== terminal state (should match the CSR read) ==")
print(f"  dc={final[0]:#010x} grant={final[1]:#010x} exc={final[2]:#010x} walk={final[3]:#010x}")
print(f"  decode with: tools/p141_decode_stall.sh {final[0]:#010x} {final[1]:#010x} "
      f"{final[2]:#010x} {final[3]:#010x}")
