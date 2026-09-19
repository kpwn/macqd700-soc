#!/usr/bin/env python3
"""Diff a hardware 53C96 trace-ring dump against the MAME golden capture.

Inputs
------
  MAME   : tools/mame_scsi96_capture.lua output    (ns,rw,reg,byte)
  HW     : jtag_repl.tcl `scsi-trace dump` output  (idx,rw,reg,byte,tstamp)

Both carry the same (rw, reg, byte) event triple; only the leading column
differs, so both are normalised to that triple.

THE ALIGNMENT PROBLEM, AND WHY THIS TOOL DOES NOT PRETEND TO SOLVE IT
--------------------------------------------------------------------
The HW ring is a 4096-entry LAST-N-WINS window ending at the wedge.  The
MAME trace is a whole 45 s boot.  They are NOT aligned at index 0, and the
HW side has been through the ring's read-side poll filter while MAME's has
not.  Comparing them naively index-by-index would produce a "divergence"
at entry 0 of every run -- a confident, meaningless answer.

So:
  1. The MAME trace is passed through the SAME poll filter the RTL ring
     applies (a read is kept only if its value differs from the last value
     read from that register; a write is always kept and invalidates that
     register's shadow).  --no-filter disables this, for checking what the
     filter itself is costing.
  2. Both sides are cut into TRANSACTIONS at each select command
     (0x41/0x42/0x43/0xc1 written to reg 3).  A transaction is a
     self-contained unit of driver behaviour, which makes it a meaningful
     alignment anchor where a raw index is not.
  3. The HW window's LAST complete transaction is matched against every
     MAME transaction; the report names the best match and the first
     event at which they differ.

Everything printed is derived from the two files.  Where the tool cannot
establish something it says so instead of guessing.
"""

import argparse
import sys
from collections import Counter

SELECT_CMDS = {0x41, 0x42, 0x43, 0xC1, 0xC2, 0xC3}

CMD_NAMES = {
    0x00: "NOP", 0x01: "FLUSH_FIFO", 0x02: "RESET_CHIP", 0x03: "RESET_BUS",
    0x10: "XFER_INFO", 0x11: "INIT_CMD_CMPL", 0x12: "MSG_ACCEPTED",
    0x1A: "XFER_PAD", 0x41: "SEL_noATN", 0x42: "SEL_ATN", 0x43: "SEL_ATN_STOP",
    0x44: "ENA_SEL_RESEL", 0x47: "RESELECT",
    0x90: "XFER_INFO/DMA", 0xC1: "SEL_noATN/DMA", 0xC2: "SEL_ATN/DMA",
}
PHASES = {0: "DATA_OUT", 1: "DATA_IN", 2: "COMMAND", 3: "STATUS",
          6: "MSG_OUT", 7: "MSG_IN"}
ISTAT_BITS = [(0, "SEL"), (1, "SELATN"), (2, "DISC"), (3, "BUS_SVC"),
              (4, "FUNC_CMPL"), (5, "RESEL"), (6, "ILLEGAL"), (7, "SCSI_RST")]


def load(path):
    """Return [(rw, reg, byte)].  Accepts either file format."""
    out = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split(",")
            if len(p) < 4:
                continue
            rw, reg, byte = p[1].strip(), p[2].strip().lower(), p[3].strip()
            if rw not in ("R", "W"):
                continue
            out.append((rw, reg, int(byte, 16)))
    return out


def drop_dma_reads(events):
    """Remove pseudo-DMA PORT READS (reg 'd') — data-in payload beats.

    MEASURED, over a full healthy 45 s MAME 7.5.3 boot: the CPU makes
    ZERO reads anywhere in the 0x50f0f100 pseudo-DMA page, while making
    6743 WRITES to it (3684 at ...100 + 3059 at ...101 — the CDB tail
    bytes of each DMA-form select).  So MAME moves >1.7 MB of data-in
    payload without any CPU-side pseudo-DMA read appearing in the
    TurboSCSI window at all.

    Our RTL takes the other path: the driver/ROM drains each chunk with
    blind CPU reads of the shim (see rtl/mac/scsi.v's pb_dma_shim and the
    16 shim_r() reads per chunk in tb/tb_scsi_c96_read6.cpp), so the
    hardware ring records thousands of R,d events that MAME's trace
    STRUCTURALLY CANNOT contain.

    Comparing the two without dropping these would report a "first
    divergence" at the very first payload beat of every run — a
    confident, meaningless answer.  Writes to the port are kept: both
    sides genuinely have those.

    This asymmetry is itself worth reporting; it is not evidence of the
    bug by itself, but it does mean payload-beat COUNTS cannot be
    compared between the two traces.
    """
    return [e for e in events if not (e[0] == "R" and e[1] == "d")]


def poll_filter(events):
    """The RTL ring's read-side filter, applied in software."""
    kept, shadow = [], {}
    for rw, reg, d in events:
        if rw == "W":
            kept.append((rw, reg, d))
            shadow.pop(reg, None)
        elif shadow.get(reg, None) != d:
            kept.append((rw, reg, d))
            shadow[reg] = d
    return kept


def split_txns(events):
    """Cut at each select command written to reg 3."""
    txns, cur = [], []
    for e in events:
        if e[0] == "W" and e[1] == "3" and e[2] in SELECT_CMDS:
            if cur:
                txns.append(cur)
            cur = [e]
        elif cur:
            cur.append(e)
    if cur:
        txns.append(cur)
    return txns


def describe(ev):
    rw, reg, d = ev
    s = "%s%s=%02x" % (rw, reg, d)
    if rw == "W" and reg == "3":
        s += "{%s}" % CMD_NAMES.get(d, "cmd?")
    elif rw == "R" and reg == "4":
        f = []
        if d & 0x80: f.append("INT")
        if d & 0x20: f.append("PERR")
        if d & 0x10: f.append("TC")
        f.append(PHASES.get(d & 7, "ph%d" % (d & 7)))
        s += "{%s}" % ",".join(f)
    elif rw == "R" and reg == "5":
        f = [n for b, n in ISTAT_BITS if d & (1 << b)]
        s += "{%s}" % ",".join(f)
    elif rw == "R" and reg == "7":
        s += "{fifo=%d}" % (d & 0x1F)
    return s


def cmd_shape(txn):
    """The command sequence of a transaction, run-length encoded."""
    cmds = [e[2] for e in txn if e[0] == "W" and e[1] == "3"]
    out = []
    for c in cmds:
        if out and out[-1][0] == c:
            out[-1][1] += 1
        else:
            out.append([c, 1])
    return " ".join("%02x%s" % (c, "x%d" % n if n > 1 else "") for c, n in out)


def first_divergence(a, b):
    for i, (x, y) in enumerate(zip(a, b)):
        if x != y:
            return i
    return len(a) if len(a) != len(b) else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mame")
    ap.add_argument("hw")
    ap.add_argument("--no-filter", action="store_true",
                    help="do NOT apply the ring's poll filter to the MAME trace")
    ap.add_argument("--context", type=int, default=12)
    ap.add_argument("--keep-dma-reads", action="store_true",
                    help="do NOT drop pseudo-DMA payload reads (see drop_dma_reads)")
    args = ap.parse_args()

    mame_raw, hw_raw = load(args.mame), load(args.hw)

    hw = hw_raw
    mame_pre = mame_raw
    if not args.keep_dma_reads:
        hw = drop_dma_reads(hw_raw)
        mame_pre = drop_dma_reads(mame_raw)
        nm, nh = len(mame_raw) - len(mame_pre), len(hw_raw) - len(hw)
        print("pseudo-DMA payload reads dropped: MAME %d, HW %d" % (nm, nh))
        if nm == 0 and nh > 0:
            print("  (expected: MAME moves data-in without CPU pseudo-DMA reads;\n"
                  "   our RTL drains each chunk with blind CPU reads of the shim.\n"
                  "   Payload-beat COUNTS are therefore not comparable between the two.)")

    mame = mame_pre if args.no_filter else poll_filter(mame_pre)

    print("MAME : %d raw events -> %d after %s" %
          (len(mame_raw), len(mame),
           "NO filter" if args.no_filter else "the ring's poll filter"))
    print("HW   : %d events (already filtered in RTL)" % len(hw))
    if not hw:
        print("\nHW trace is EMPTY.  That is NOT evidence the SCSI bus was idle -- it is\n"
              "equally consistent with a ring that never captured.  Check `scsi-trace\n"
              "status` reports a non-zero wr_ptr before drawing any conclusion.")
        return 1

    mt, ht = split_txns(mame), split_txns(hw)
    print("MAME : %d transactions   HW: %d transactions" % (len(mt), len(ht)))
    if not ht:
        print("\nHW window contains NO select command, so it cannot be cut into\n"
              "transactions.  The whole 4096-entry window is inside a single\n"
              "transaction -- report the raw tail instead of a transaction diff.")
        for e in hw[-args.context:]:
            print("   %s" % describe(e))
        return 1

    print("\n=== HW command shapes, oldest -> newest (last 12 transactions) ===")
    for t in ht[-12:]:
        print("  %4d ev  %s" % (len(t), cmd_shape(t)))

    print("\n=== MAME command-shape census ===")
    for s, n in Counter(cmd_shape(t) for t in mt).most_common(8):
        print("  %6d  %s" % (n, s))

    # The last HW transaction is the one that wedged.
    last = ht[-1]
    print("\n=== HW FINAL (wedged) transaction: %d events ===" % len(last))
    print("  shape: %s" % cmd_shape(last))

    # Best MAME match = longest common prefix.
    best_i, best_len = -1, -1
    for i, t in enumerate(mt):
        n = 0
        for x, y in zip(last, t):
            if x != y:
                break
            n += 1
        if n > best_len:
            best_i, best_len = i, n
    ref = mt[best_i]
    print("  best MAME match: transaction #%d (common prefix %d events)" %
          (best_i, best_len))
    print("  MAME shape:  %s" % cmd_shape(ref))

    d = first_divergence(last, ref)
    if d is None:
        print("\nNO DIVERGENCE: the wedged HW transaction is identical to MAME's.\n"
              "The difference is then NOT in the register sequence -- look at\n"
              "TIMING or at what the data-in supply actually delivered.")
        return 0

    # Distinguish a REAL mismatch from the HW window simply running out.
    # These are completely different findings and they look identical if
    # you only report an index.  (Caught by the tool's own positive
    # control: feeding it a synthetic HW trace built from MAME reported a
    # "divergence" at the window edge, which is not a divergence at all.)
    if d >= len(last):
        print("\n=== HW TRACE STOPS at event %d; MAME's transaction CONTINUES ===" % d)
        print("  This is NOT a value mismatch -- every event HW produced matched MAME.")
        print("  HW simply stopped emitting.  Since the dump is oldest->newest and this")
        print("  is the LAST transaction, this stopping point IS the wedge: the driver")
        print("  got no further response from the chip.")
        print("\n  last %d matching events before the stop:" % min(args.context, d))
        for i in range(max(0, d - args.context), d):
            print("      %4d  %s" % (i, describe(last[i])))
        print("\n  what MAME does NEXT (i.e. what the hardware failed to do):")
        for i in range(d, min(len(ref), d + args.context)):
            print("      %4d  %s" % (i, describe(ref[i])))
        return 0

    print("\n=== FIRST DIVERGENCE at event %d of the wedged transaction ===" % d)
    lo = max(0, d - args.context)
    print("  common prefix (last %d events):" % (d - lo))
    for i in range(lo, d):
        print("      %4d  %s" % (i, describe(last[i])))
    print("  HW  : %s" % (describe(last[d]) if d < len(last) else "<end of HW window>"))
    print("  MAME: %s" % (describe(ref[d]) if d < len(ref) else "<end of MAME txn>"))
    print("\n  MAME continues:")
    for i in range(d, min(len(ref), d + args.context)):
        print("      %4d  %s" % (i, describe(ref[i])))
    print("\n  HW continues:")
    for i in range(d, min(len(last), d + args.context)):
        print("      %4d  %s" % (i, describe(last[i])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
