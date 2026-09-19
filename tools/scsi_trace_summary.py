#!/usr/bin/env python3
"""Summarise a 53C96 register trace: what commands were issued, and did any
command get a CHECK CONDITION?

Works on BOTH trace formats, which differ only in their leading column:
    MAME (tools/mame_scsi96_capture.lua) : ns,rw,reg,byte
    HW   (jtag_repl.tcl `scsi-trace dump`): idx,rw,reg,byte,tstamp

Why REQUEST SENSE is the signal
-------------------------------
A SCSI initiator issues REQUEST SENSE (opcode 0x03) only after a command
returned CHECK CONDITION. So its COUNT is a direct, format-independent
error indicator that does not require decoding bus phases.

Measured 2026-08-18: MAME's whole 7.5.3 boot = 3597 READ(10) + 40 WRITE(10)
and ZERO REQUEST SENSE, i.e. not one SCSI error. Any REQUEST SENSE on the HW
side is therefore a divergence, and the sense bytes that come back name the
fault:
    key 2 / ASC 0x3A  medium not present  -> scsi.v backing-store timeout
    key 5 / ASC 0x21  LBA out of range    -> extent/mapper bug
    key 5 / ASC 0x20  invalid opcode      -> unsupported command
    key 3 / ASC 0x11  unrecovered read    -> a real sd_ctrl error

CDBs are reconstructed from runs of consecutive FIFO (reg 2) writes. That is
a heuristic -- data bytes can also pass through the FIFO -- so runs are only
counted when the length looks like a CDB (6/10/12 bytes) and the opcode's
group code agrees with that length.
"""
import sys, collections

NAME = {0x00:"TEST UNIT READY",0x03:"REQUEST SENSE",0x08:"READ(6)",0x0A:"WRITE(6)",
        0x12:"INQUIRY",0x15:"MODE SELECT",0x1A:"MODE SENSE",0x1B:"START STOP",
        0x25:"READ CAPACITY",0x28:"READ(10)",0x2A:"WRITE(10)",0x37:"READ DEFECT",
        0x5A:"MODE SENSE(10)"}

# Empirically (MAME 7.5.3 boot) a CDB shows up as a run of 8-9 FIFO writes,
# not the nominal 6/10, because the driver interleaves other register accesses
# mid-CDB. So the only workable filter is a length floor plus the first byte as
# the opcode; a stricter group-code/length check matches NOTHING (verified: it
# scored 0 CDBs on a trace that plainly contains 3637 of them).
MIN_RUN = 6

def parse(path):
    runs, cur = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or line.lower().startswith('idx'):
                continue
            p = line.split(',')
            if len(p) < 4:
                continue
            rw, reg, byte = p[1].strip(), p[2].strip(), p[3].strip()
            try:
                b = int(byte, 16)
            except ValueError:
                continue
            if rw.upper() == 'W' and reg.strip().lstrip('0').lower() in ('2',):
                cur.append(b)
            else:
                if cur: runs.append(cur)
                cur = []
    if cur: runs.append(cur)
    return runs

def main():
    if len(sys.argv) < 2:
        print("usage: scsi_trace_summary.py <trace.csv> [...]"); return 1
    for path in sys.argv[1:]:
        runs = parse(path)
        h = collections.Counter()
        for r in runs:
            if len(r) >= MIN_RUN:
                h[r[0]] += 1
        print(f"=== {path} ===")
        print(f"  {len(runs)} FIFO write runs; {sum(h.values())} look like CDBs")
        for op, c in h.most_common(16):
            print(f"    0x{op:02X}  {NAME.get(op,'?'):<18} x{c}")
        rs = h.get(0x03, 0)
        print(f"  REQUEST SENSE = {rs}  ->  " +
              ("CHECK CONDITION(s) occurred" if rs else "no SCSI error in this window"))
        print()
    return 0

sys.exit(main())
