#!/usr/bin/env python3
"""Diff a hardware DP83932C SONIC register trace against the MAME golden capture.

Inputs
------
  MAME : tools/mame_sonic_capture.lua output
  HW   : the RTL SONIC trace-ring dump read out over JTAG

Both files are CSV with a self-describing header line:

    # columns: ns,rw,reg,data,lanes,flags

The differ reads that header, so the two sides do NOT have to agree on
column order or on which extra columns they carry.  Recognised column
names: ns / idx / seq / ts / tstamp (all dropped), rw, reg, data (or
byte), lanes, flags.  A file with no "# columns:" header falls back to a
positional guess documented in `parse_columns`.

WHAT THIS TOOL COMPARES, AND WHAT IT DELIBERATELY DOES NOT
----------------------------------------------------------
An event is the 4-tuple (rw, reg, data, lanes).  `lanes` is the byte-lane
mask of the access, encoded exactly like rtl/soc/peripheral_bus.v's
`sonic_wstrb`: bit1 = D15..D8 (physical byte +2), bit0 = D7..D0 (byte +3).
Half-register accesses are first-class here because the Mac driver makes
them and because MAME's dp83932c CANNOT model them (see MAME NOTES below).

Dropped outright, with a printed count:
  * timestamps  — the two machines run at different speeds; nothing about
                  wall/emulated time is comparable.
  * open-bus-only accesses (lanes == 0) — nothing is wired to D31..D16 in
                  the SONIC slot, so the chip never sees them.

Everything else is compared byte-exactly UNLESS an explicitly named
tolerance rule says otherwise.  Every tolerated difference is COUNTED and
PRINTED in the summary; nothing is ever silently ignored.  `--strict`
turns all of them off.  `--list-rules` prints the rules and their reasons
and exits.

MAME NOTES THAT BEAR ON READING THE DIFF
-----------------------------------------
(from src/devices/machine/dp83932c.{h,cpp} in MAME 0.285)

  * reg_r() is `return m_reg[offset];` — a pure read with NO side effects.
    So on the MAME side, reading a register never clears or pops anything.
    If our RTL has a read side effect anywhere, that is a divergence the
    register stream will show directly.

  * reg_w() takes NO mem_mask.  Combined with .umask32(0x0000ffff), a
    BYTE write from the CPU still lands as a full 16-bit register write in
    MAME, with the untouched half taken as 0.  MAME therefore cannot
    reproduce a half-register write correctly, and a `lanes` mismatch
    between the two traces is more likely to be a MAME limitation than an
    RTL bug.  It is still reported, never hidden.

  * CR is SET-ONLY on write (`m_reg[CR] |= data & mask`); the command
    handler clears the self-clearing bits afterwards.  So a readback of CR
    that still shows a command bit means the command has not run yet.

  * ISR is write-1-to-clear, and clearing ISR_RBE re-reads the RRA.

  * CRCT / FAET / MPT are stored INVERTED on write (`m_reg[x] = ~data`).

  * load_cam() runs on CR_LCAM, walks the CAM descriptor list at
    URRA:CDP for CDC entries, then reads CE from the word after the list,
    clears CR_LCAM and sets ISR_LCD.

  * CR_TXP is cleared by send_complete_cb / halt, not by the write; and a
    TXP write while TXP is already set is masked out of the command so it
    cannot smash a running transmission.

  * update_interrupts() asserts INT on `ISR & IMR`.  IMR reading back as
    0x0000 on hardware while MAME reads back what the driver wrote is a
    register-model divergence, not an interrupt-routing one — the
    readback audit below is aimed straight at that.

EXIT CODES
----------
  0  the two streams agree under the rules in force
  1  a divergence was found, or one side stopped while the other continued
  2  the input is unusable (empty trace, unparsable columns)

USAGE
-----
  tools/sonic_trace_diff.py --list-rules
  tools/sonic_trace_diff.py mame_sonic.csv hw_sonic.csv
  tools/sonic_trace_diff.py mame_sonic.csv hw_sonic.csv --strict
  tools/sonic_trace_diff.py mame_sonic.csv hw_sonic.csv --tolerate rx-pointers

Tests: python3 -m unittest discover -s tools/tests -p 'test_sonic_trace_diff.py'
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter, OrderedDict

# ── Register model (MAME src/devices/machine/dp83932c.h; our RTL's
#    sonic_reg_mask() in rtl/mac/q700_eth_sonic.v is byte-identical) ─────
REG_NAMES = [
    "CR",    "DCR",   "RCR",   "TCR",   "IMR",   "ISR",   "UTDA",  "CTDA",
    "TPS",   "TFC",   "TSA0",  "TSA1",  "TFS",   "URDA",  "CRDA",  "CRBA0",
    "CRBA1", "RBWC0", "RBWC1", "EOBC",  "URRA",  "RSA",   "REA",   "RRP",
    "RWP",   "TRBA0", "TRBA1", "TBWC0", "TBWC1", "ADDR0", "ADDR1", "LLFA",
    "TTDA",  "CEP",   "CAP2",  "CAP1",  "CAP0",  "CE",    "CDP",   "CDC",
    "SR",    "WT0",   "WT1",   "RSC",   "CRCT",  "FAET",  "MPT",   "MDT",
    "r30",   "r31",   "r32",   "r33",   "r34",   "r35",   "r36",   "r37",
    "r38",   "r39",   "r3a",   "r3b",   "r3c",   "r3d",   "r3e",   "DCR2",
]
REGMASK = [
    0x03bf, 0xbfff, 0xfe00, 0xf000, 0x7fff, 0x7fff, 0xffff, 0xffff,
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xfffe, 0xfffe, 0xfffe,
    0xfffe, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
    0xffff, 0x000f, 0x0000, 0x0000, 0x0000, 0xffff, 0xfffe, 0x001f,
    0x0000, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x0000,
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xf017,
]

CR, DCR, RCR, TCR, IMR, ISR = 0x00, 0x01, 0x02, 0x03, 0x04, 0x05
UTDA, CTDA, TPS, TFC = 0x06, 0x07, 0x08, 0x09
URDA, CRDA, CRBA0, CRBA1, RBWC0, RBWC1, EOBC = 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13
URRA, RSA, REA, RRP, RWP = 0x14, 0x15, 0x16, 0x17, 0x18
TRBA0, TRBA1, TBWC0, TBWC1, LLFA = 0x19, 0x1a, 0x1b, 0x1c, 0x1f
TTDA, CEP, CAP2, CAP1, CAP0, CE, CDP, CDC = 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27
SR, WT0, WT1, RSC, CRCT, FAET, MPT, MDT, DCR2 = 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x3f

PROM_LO, PROM_HI = 0x40, 0x47          # Ethernet ID PROM bytes, if captured

CR_BITS = [(0x0001, "HTX"), (0x0002, "TXP"), (0x0004, "RXDIS"), (0x0008, "RXEN"),
           (0x0010, "STP"), (0x0020, "ST"), (0x0080, "RST"), (0x0100, "RRRA"),
           (0x0200, "LCAM")]
ISR_BITS = [(0x0001, "RFO"), (0x0002, "MP"), (0x0004, "FAE"), (0x0008, "CRC"),
            (0x0010, "RBAE"), (0x0020, "RBE"), (0x0040, "RDE"), (0x0080, "TC"),
            (0x0100, "TXER"), (0x0200, "TXDN"), (0x0400, "PKTRX"), (0x0800, "PINT"),
            (0x1000, "LCD"), (0x2000, "HBL"), (0x4000, "BR")]
IMR_BITS = [(0x0001, "RFOEN"), (0x0002, "MPEN"), (0x0004, "FAEEN"), (0x0008, "CRCEN"),
            (0x0010, "RBAEEN"), (0x0020, "RBEEN"), (0x0040, "RDEEN"), (0x0080, "TCEN"),
            (0x0100, "TXEREN"), (0x0200, "PTXEN"), (0x0400, "PRXEN"), (0x0800, "PINTEN"),
            (0x1000, "LCDEN"), (0x2000, "HBLEN"), (0x4000, "BREN")]
RCR_BITS = [(0x0001, "PRX"), (0x0002, "LBK"), (0x0004, "FAER"), (0x0008, "CRCR"),
            (0x0010, "COL"), (0x0020, "CRS"), (0x0040, "LPKT"), (0x0080, "BC"),
            (0x0100, "MC"), (0x0800, "AMC"), (0x1000, "PRO"), (0x2000, "BRD"),
            (0x4000, "RNT"), (0x8000, "ERR")]
TCR_BITS = [(0x0001, "PTX"), (0x0002, "BCM"), (0x0004, "FU"), (0x0008, "PMB"),
            (0x0020, "OWC"), (0x0040, "EXC"), (0x0080, "CRSL"), (0x0100, "NCRS"),
            (0x0200, "DEF"), (0x0400, "EXD"), (0x1000, "EXDIS"), (0x2000, "CRCI"),
            (0x4000, "POWC"), (0x8000, "PINT")]
BITMAPS = {CR: CR_BITS, ISR: ISR_BITS, IMR: IMR_BITS, RCR: RCR_BITS, TCR: TCR_BITS}

CR_RST, CR_TXP, CR_LCAM, CR_RRRA = 0x0080, 0x0002, 0x0200, 0x0100


def reg_name(reg):
    if PROM_LO <= reg <= PROM_HI:
        return "PROM%d" % (reg - PROM_LO)
    if 0 <= reg < 64:
        return REG_NAMES[reg]
    return "r%02x" % reg


def lane_mask(lanes):
    """Data bits the access actually drove/sampled.  Same encoding as sonic_wstrb."""
    return (0xff00 if lanes & 2 else 0) | (0x00ff if lanes & 1 else 0)


# ── Event ────────────────────────────────────────────────────────────────
class Event(tuple):
    """(rw, reg, data, lanes) with a name, so it stays hashable and cheap."""
    __slots__ = ()

    def __new__(cls, rw, reg, data, lanes):
        return tuple.__new__(cls, (rw, reg, data, lanes))

    rw = property(lambda self: self[0])
    reg = property(lambda self: self[1])
    data = property(lambda self: self[2])
    lanes = property(lambda self: self[3])


def describe(ev):
    n = reg_name(ev.reg)
    if ev.lanes == 3:
        val = "%04x" % ev.data
    elif ev.lanes == 2:
        val = "%02x__" % ((ev.data >> 8) & 0xff)
    elif ev.lanes == 1:
        val = "__%02x" % (ev.data & 0xff)
    else:
        val = "----"
    s = "%s %-5s=%s" % (ev.rw, n, val)
    bits = BITMAPS.get(ev.reg)
    if bits:
        m = lane_mask(ev.lanes)
        set_names = [nm for b, nm in bits if (ev.data & b & m)]
        s += " {%s}" % ",".join(set_names) if set_names else " {-}"
    return s


# ── Loading ──────────────────────────────────────────────────────────────
DROP_COLUMNS = {"ns", "idx", "seq", "ts", "tstamp", "time", "cycle", "n"}
DATA_ALIASES = {"data", "byte", "value", "val"}


def parse_columns(header_cols, nfields, path, which="hw"):
    """Map field index -> role.  Roles: rw, reg, data, lanes, flags, None."""
    if header_cols:
        roles = []
        for c in header_cols:
            c = c.strip().lower()
            if c == "rw":
                roles.append("rw")
            elif c == "reg":
                roles.append("reg")
            elif c in DATA_ALIASES:
                roles.append("data")
            elif c == "lanes":
                roles.append("lanes")
            elif c == "flags":
                roles.append("flags")
            elif c in DROP_COLUMNS:
                roles.append(None)
            else:
                roles.append(None)
        if "rw" in roles and "reg" in roles and "data" in roles:
            return roles
        raise SystemExit(
            "%s: '# columns:' header names %r, which lacks one of rw/reg/data.\n"
            "Fix the producer or pass --%s-columns explicitly."
            % (path, header_cols, which))
    # No header.  Positional fallback, documented so it cannot surprise:
    #   4 fields -> seq,rw,reg,data
    #   5 fields -> seq,rw,reg,data,lanes
    #   6+       -> seq,rw,reg,data,lanes,flags,...
    if nfields < 4:
        raise SystemExit("%s: %d fields per line, need at least 4 "
                         "(seq,rw,reg,data)" % (path, nfields))
    roles = [None, "rw", "reg", "data"]
    if nfields >= 5:
        roles.append("lanes")
    if nfields >= 6:
        roles.append("flags")
    roles += [None] * (nfields - len(roles))
    return roles


def load(path, forced_columns=None, which="hw"):
    """Return (events, meta).  meta['has_lanes'] says whether lanes were real."""
    header_cols = None
    if forced_columns:
        header_cols = forced_columns.split(",")
    rows = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                low = line.lstrip("#").strip().lower()
                if low.startswith("columns:") and not forced_columns:
                    header_cols = low.split(":", 1)[1].split(",")
                continue
            rows.append(line.split(","))
    if not rows:
        return [], {"has_lanes": False, "n_raw": 0, "flags": Counter()}

    roles = parse_columns(header_cols, len(rows[0]), path, which)
    i_rw, i_reg, i_data = roles.index("rw"), roles.index("reg"), roles.index("data")
    i_lanes = roles.index("lanes") if "lanes" in roles else None
    i_flags = roles.index("flags") if "flags" in roles else None

    events, flags = [], Counter()
    for p in rows:
        if len(p) <= max(i_rw, i_reg, i_data):
            continue
        rw = p[i_rw].strip().upper()
        if rw not in ("R", "W"):
            continue
        reg = int(p[i_reg].strip(), 16)
        data = int(p[i_data].strip(), 16) & 0xffff
        lanes = 3
        if i_lanes is not None and len(p) > i_lanes:
            tok = p[i_lanes].strip()
            if tok:
                lanes = int(tok, 16) & 3
        if i_flags is not None and len(p) > i_flags:
            for ch in p[i_flags].strip():
                if ch != "-":
                    flags[ch] += 1
        events.append(Event(rw, reg, data, lanes))
    return events, {"has_lanes": i_lanes is not None,
                    "n_raw": len(events), "flags": flags}


# ── Tolerance rules ──────────────────────────────────────────────────────
class Rule:
    def __init__(self, name, regs, mask, kinds, default_on, why):
        self.name, self.regs, self.mask = name, frozenset(regs), mask
        self.kinds, self.default_on, self.why = frozenset(kinds), default_on, why


RULES = [
    Rule("tally", {RSC, CRCT, FAET, MPT}, 0xffff, "R", True,
         "RSC/CRCT/FAET/MPT are tally and sequence counters of events on the WIRE "
         "(frames received, CRC errors, frame-alignment errors, missed packets). "
         "MAME's network backend and the FPGA's PHY are on different networks, so "
         "equal values would be a coincidence, not a match.  None of them can "
         "cause IMR to read back 0, so tolerating them costs no evidence."),
    Rule("rx-pointers", {CRDA, CRBA0, CRBA1, RBWC0, RBWC1, RRP, LLFA,
                         TRBA0, TRBA1, TBWC0, TBWC1}, 0xffff, "R", False,
         "Receive descriptor / buffer pointers.  They advance as the chip consumes "
         "receive resources, so they legitimately depend on how many frames arrived.  "
         "OFF BY DEFAULT: 'the receive ring fills and stops' is the symptom under "
         "investigation, so these are evidence, not noise.  Enable only once the RX "
         "path is already accounted for."),
    Rule("rx-isr", {ISR}, 0x047f, "RW", False,
         "ISR bits RFO|MP|FAE|CRC|RBAE|RBE|RDE|PKTRX -- the receive-side interrupt "
         "sources.  Applies to writes too, because the driver clears ISR by writing "
         "back the bits it just read (write-1-to-clear), so a tolerated read "
         "difference produces a matching write difference.  OFF BY DEFAULT for the "
         "same reason as rx-pointers: this is where the bug is expected to show."),
    Rule("tx-status", {TCR}, 0x07fe, "R", False,
         "TCR transmit-status bits except PTX.  Collision, carrier-sense, deferral "
         "and FIFO-underrun status reflect the real medium.  PTX (0x0001, 'packet "
         "transmitted OK') is deliberately EXCLUDED -- whether a transmit succeeded "
         "is a fact worth comparing.  The TCR config half (0xf000) is loaded from "
         "the transmit descriptor in RAM and must match."),
    Rule("mac-address", {CEP, CAP2, CAP1, CAP0, CE}, 0xffff, "R", False,
         "CAM entry pointer / address ports / enable.  The CAM is loaded from the "
         "Ethernet ID PROM, which holds a different MAC on the FPGA than in MAME "
         "(macquadra700.cpp machine_start() bit-swizzles a fixed Apple OUI).  OFF BY "
         "DEFAULT because a wrong CE is a plausible cause of 'the driver never sees "
         "its own frames'."),
    Rule("prom", set(range(PROM_LO, PROM_HI + 1)), 0xffff, "R", False,
         "Ethernet ID PROM bytes at 0x50008000..7 -- the MAC address itself, "
         "different by construction on the two machines.  OFF BY DEFAULT because a "
         "PROM that reads back garbage (or fails its checksum) makes the driver give "
         "up before it ever programs the SONIC, which would explain everything."),
]
RULES_BY_NAME = {r.name: r for r in RULES}


def build_tolerance(enabled):
    """-> dict[(kind, reg)] -> (mask, rule_name).  Later rules OR in."""
    tol = {}
    for r in RULES:
        if r.name not in enabled:
            continue
        for k in r.kinds:
            for reg in r.regs:
                m, names = tol.get((k, reg), (0, ()))
                tol[(k, reg)] = (m | r.mask, names + (r.name,))
    return tol


def parse_extra_tolerance(specs, tol):
    """--tolerate-reg REG[:MASK] -- ad-hoc, applies to reads only unless :W."""
    for spec in specs or []:
        parts = spec.split(":")
        reg = int(parts[0], 16)
        mask = int(parts[1], 16) if len(parts) > 1 and parts[1] else 0xffff
        kinds = parts[2].upper() if len(parts) > 2 else "R"
        for k in kinds:
            m, names = tol.get((k, reg), (0, ()))
            tol[(k, reg)] = (m | mask, names + ("cli:%s" % spec,))
    return tol


# ── Normalisation ────────────────────────────────────────────────────────
def drop_open_bus(events):
    """lanes==0: the access covered only D31..D16, which is not wired to the SONIC."""
    return [e for e in events if e.lanes != 0]


def collapse_polls(events, window):
    """Fold repeated adjacent READ cycles of length 1..window down to one copy.

    A poll loop reads the same register(s) over and over until a bit changes.
    How MANY times it spins is a pure timing artefact: MAME runs the machine at
    a different speed than the FPGA, and the HW ring may also have been read out
    mid-spin.  Folding an EXACTLY repeated read cycle is therefore safe -- the
    fold requires the values to be identical, so the iteration where the polled
    value finally changes is never folded away, and neither is anything else.

    Writes are barriers: they are never folded and never folded across.
    `window` bounds the cycle length so an interleaved poll (read ISR, read CR,
    read ISR, read CR, ...) folds too.  --poll-window 1 restricts folding to a
    single repeated read.
    """
    out = []
    for e in events:
        out.append(e)
        if e.rw == "W":
            continue
        for k in range(1, window + 1):
            if len(out) < 2 * k:
                break
            tail, prev = out[-k:], out[-2 * k:-k]
            if tail == prev and all(x.rw == "R" for x in tail):
                del out[-k:]
                break
    return out


# ── Comparison ───────────────────────────────────────────────────────────
class Divergence:
    def __init__(self, index, kind, hw, mame, detail=""):
        self.index, self.kind, self.hw, self.mame, self.detail = index, kind, hw, mame, detail


def compare_event(hw, mame, tol, ignore_lanes, tolerated):
    """None if the two events agree (possibly via a rule); else a reason string."""
    if hw.rw != mame.rw:
        return "read/write direction"
    if hw.reg != mame.reg:
        return "register index"
    if not ignore_lanes and hw.lanes != mame.lanes:
        return "byte lanes (half-register access differs)"
    m = lane_mask(hw.lanes) & lane_mask(mame.lanes)
    delta = (hw.data ^ mame.data) & m
    if delta == 0:
        return None
    tmask, names = tol.get((hw.rw, hw.reg), (0, ()))
    residual = delta & ~tmask
    if residual == 0:
        tolerated.append((hw, mame, delta, names))
        return None
    return "data (differing bits %04x)" % residual


def common_prefix(hw, mame, tol, ignore_lanes):
    """Length of the tolerant common prefix, plus the tolerated list for it."""
    tolerated = []
    n = 0
    for a, b in zip(hw, mame):
        if compare_event(a, b, tol, ignore_lanes, tolerated) is not None:
            break
        n += 1
    return n, tolerated


# ── Alignment ────────────────────────────────────────────────────────────
def split_epochs(events):
    """Cut the stream at each software reset: a CR write with RST set.

    The driver's whole init sequence begins with `CR <- CR_RST`, which makes it
    the one anchor in the stream that means the same thing on both sides.  A raw
    index does not: the HW ring is a last-N-wins window and the MAME trace is a
    whole boot, so they are not aligned at index 0.
    """
    epochs, cur = [], []
    for e in events:
        if e.rw == "W" and e.reg == CR and (e.lanes & 1) and (e.data & CR_RST):
            if cur:
                epochs.append(cur)
            cur = [e]
        elif cur:
            cur.append(e)
    if cur:
        epochs.append(cur)
    return epochs


def best_slide(hw, mame, tol, ignore_lanes, cap=200000):
    """Slide the HW stream over MAME, return (offset, prefix_len)."""
    best = (0, -1)
    tried = 0
    for off in range(len(mame)):
        if hw and mame[off].rw != hw[0].rw:
            continue
        if hw and mame[off].reg != hw[0].reg:
            continue
        tried += 1
        if tried > cap:
            break
        n, _ = common_prefix(hw, mame[off:], tol, ignore_lanes)
        if n > best[1]:
            best = (off, n)
        if n == len(hw):
            break
    return best


# ── Readback audit ───────────────────────────────────────────────────────
# Registers software owns outright: nothing in the chip writes them behind the
# driver's back, so a read that disagrees with the last write is unambiguous.
# (CR is set-only, ISR is w1c, CRCT/FAET/MPT invert on write, TCR/CTDA/CRDA/
#  RRP/CDP/CDC/CE/TTDA/RSC and the tallies are chip-updated -- all excluded.)
AUDIT_REGS = OrderedDict([
    (DCR,  "data configuration"),
    (IMR,  "interrupt mask"),
    (UTDA, "upper transmit descriptor address"),
    (URDA, "upper receive descriptor address"),
    (EOBC, "end of buffer word count"),
    (URRA, "upper receive resource address"),
    (RSA,  "resource start address"),
    (REA,  "resource end address"),
    (RWP,  "resource write pointer"),
    (WT0,  "watchdog timer 0"),
    (WT1,  "watchdog timer 1"),
    (DCR2, "data configuration 2"),
])


def readback_audit(events, regs):
    """-> OrderedDict[reg] = dict(wrote, read, ok, bad, first_bad)."""
    shadow, out = {}, OrderedDict()
    for e in events:
        if e.reg not in regs:
            continue
        m = lane_mask(e.lanes) & REGMASK[e.reg] if e.reg < 64 else lane_mask(e.lanes)
        st = out.setdefault(e.reg, {"wrote": None, "read": None,
                                    "ok": 0, "bad": 0, "first_bad": None})
        if e.rw == "W":
            prev = shadow.get(e.reg, 0)
            shadow[e.reg] = (prev & ~m) | (e.data & m)
            st["wrote"] = shadow[e.reg]
        else:
            st["read"] = e.data & m
            if e.reg in shadow:
                if ((shadow[e.reg] ^ e.data) & m) == 0:
                    st["ok"] += 1
                else:
                    st["bad"] += 1
                    if st["first_bad"] is None:
                        st["first_bad"] = (shadow[e.reg] & m, e.data & m, m)
    return out


def print_readback(label, audit):
    if not audit:
        print("  %-5s (no software-owned register was both written and read)" % label)
        return
    for reg, st in audit.items():
        line = "  %-5s %-5s wrote=%s reads_matching=%d reads_mismatching=%d" % (
            label, reg_name(reg),
            "%04x" % st["wrote"] if st["wrote"] is not None else "----",
            st["ok"], st["bad"])
        if st["first_bad"]:
            w, r, m = st["first_bad"]
            line += "   FIRST MISMATCH: wrote %04x, read %04x (lane mask %04x)" % (w, r, m)
        print(line)


# ── Reporting helpers ────────────────────────────────────────────────────
def census(events):
    c = Counter()
    for e in events:
        c[(e.rw, e.reg)] += 1
    return c


def print_census(label, c, limit=24):
    rows = sorted(c.items(), key=lambda kv: -kv[1])[:limit]
    print("  %s: %s" % (label, "  ".join(
        "%s%s x%d" % (rw, reg_name(reg), n) for (rw, reg), n in rows)))


def print_context(stream, lo, hi, marker_at=None):
    for i in range(max(0, lo), min(len(stream), hi)):
        mark = ">>" if i == marker_at else "  "
        print("      %s %4d  %s" % (mark, i, describe(stream[i])))


# ── main ─────────────────────────────────────────────────────────────────
def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Find the first divergence between a MAME SONIC capture "
                    "and a hardware SONIC trace.")
    ap.add_argument("mame", nargs="?", help="tools/mame_sonic_capture.lua output")
    ap.add_argument("hw", nargs="?", help="hardware SONIC trace-ring dump")
    ap.add_argument("--list-rules", action="store_true",
                    help="print the benign-difference rules and why each exists, then exit")
    ap.add_argument("--strict", action="store_true",
                    help="disable EVERY tolerance rule; compare byte-exactly")
    ap.add_argument("--tolerate", action="append", metavar="RULE", default=[],
                    help="enable a tolerance rule (repeatable); see --list-rules")
    ap.add_argument("--no-tolerate", action="append", metavar="RULE", default=[],
                    help="disable a default-on tolerance rule (repeatable)")
    ap.add_argument("--tolerate-reg", action="append", metavar="REG[:MASK[:RW]]",
                    default=[], help="ad-hoc tolerance, hex reg index, e.g. 05:0400 "
                                     "or 03:07ff:RW (default mask ffff, default kind R)")
    ap.add_argument("--poll-window", type=int, default=4, metavar="N",
                    help="max repeated read-cycle length folded as a poll loop "
                         "(default 4; 0 disables poll folding)")
    ap.add_argument("--keep-open-bus", action="store_true",
                    help="keep lanes==0 accesses (nothing is wired there)")
    ap.add_argument("--ignore-lanes", action="store_true",
                    help="do not compare byte lanes (use when one side does not record them)")
    ap.add_argument("--align", choices=("auto", "epoch", "slide", "first"),
                    default="auto",
                    help="alignment strategy (default auto: epoch if the HW trace "
                         "contains a CR<-RST write, else slide)")
    ap.add_argument("--context", type=int, default=12)
    ap.add_argument("--mame-columns", metavar="LIST",
                    help="override the MAME file's column names")
    ap.add_argument("--hw-columns", metavar="LIST",
                    help="override the HW file's column names")
    args = ap.parse_args(argv)

    if args.list_rules:
        for r in RULES:
            print("%-12s  %s  applies to %s  mask %04x  regs %s"
                  % (r.name, "ON by default " if r.default_on else "off by default",
                     "/".join(sorted(r.kinds)), r.mask,
                     ",".join(sorted(reg_name(x) for x in r.regs))))
            for ln in r.why.split(". "):
                if ln.strip():
                    print("              %s" % ln.strip().rstrip(".") + ".")
            print()
        return 0

    if not args.mame or not args.hw:
        ap.error("both MAME and HW trace paths are required (or use --list-rules)")

    enabled = {r.name for r in RULES if r.default_on}
    for n in args.tolerate:
        if n not in RULES_BY_NAME:
            ap.error("unknown rule %r (see --list-rules)" % n)
        enabled.add(n)
    for n in args.no_tolerate:
        if n not in RULES_BY_NAME:
            ap.error("unknown rule %r (see --list-rules)" % n)
        enabled.discard(n)
    if args.strict:
        enabled = set()
    tol = parse_extra_tolerance(args.tolerate_reg, build_tolerance(enabled))

    mame_raw, mame_meta = load(args.mame, args.mame_columns, "mame")
    hw_raw, hw_meta = load(args.hw, args.hw_columns, "hw")

    print("=== inputs ===")
    print("  MAME : %-40s %6d events  lanes=%s"
          % (args.mame, len(mame_raw), "yes" if mame_meta["has_lanes"] else "NOT RECORDED"))
    print("  HW   : %-40s %6d events  lanes=%s"
          % (args.hw, len(hw_raw), "yes" if hw_meta["has_lanes"] else "NOT RECORDED"))
    for label, meta in (("MAME", mame_meta), ("HW", hw_meta)):
        if meta["flags"]:
            print("  %s flags: %s" % (label, dict(meta["flags"])))
            if meta["flags"].get("U"):
                print("    'U' = access above 0x5000a0ff.  Our RTL mirrors the 64 "
                      "registers every 0x100\n"
                      "         across 0x5000a000..0x5000b0ff; MAME maps only the "
                      "first 0x100.\n"
                      "         The two sides are not modelling the same aperture "
                      "for those accesses.")

    ignore_lanes = args.ignore_lanes
    if not (mame_meta["has_lanes"] and hw_meta["has_lanes"]) and not ignore_lanes:
        ignore_lanes = True
        print("\n  NOTE: one side does not record byte lanes, so lane comparison is")
        print("        DISABLED for this run.  Half-register writes are exactly the")
        print("        kind of thing this bug could be, so add the column on that")
        print("        side before drawing a conclusion from a clean diff.")

    if not hw_raw:
        print("\nHW trace is EMPTY.  That is NOT evidence the driver never touched the")
        print("SONIC -- it is equally consistent with a ring that never armed.  Check")
        print("the ring's write pointer before drawing any conclusion.")
        return 2
    if not mame_raw:
        print("\nMAME trace is EMPTY.  Most likely the capture tapped the wrong mirror,")
        print("or the emulated run never brought up AppleTalk/MacTCP.  Re-run with")
        print("MAME_SONIC_TRACE_MIRRORS=all and long enough to reach the driver.")
        return 2

    # ── normalise ────────────────────────────────────────────────────────
    mame, hw = mame_raw, hw_raw
    if not args.keep_open_bus:
        mame, hw = drop_open_bus(mame), drop_open_bus(hw)
        print("\n  open-bus-only accesses dropped (lanes==0): MAME %d, HW %d"
              % (len(mame_raw) - len(mame), len(hw_raw) - len(hw)))
    if args.poll_window > 0:
        pre_m, pre_h = len(mame), len(hw)
        mame = collapse_polls(mame, args.poll_window)
        hw = collapse_polls(hw, args.poll_window)
        print("  poll repetitions folded (window %d): MAME %d -> %d, HW %d -> %d"
              % (args.poll_window, pre_m, len(mame), pre_h, len(hw)))

    print("\n=== tolerance rules in force ===")
    if not enabled and not args.tolerate_reg:
        print("  (none -- strict byte-exact comparison)")
    for n in sorted(enabled):
        r = RULES_BY_NAME[n]
        print("  %-12s %s mask %04x on %s" % (
            n, ",".join(sorted(reg_name(x) for x in r.regs)), r.mask,
            "/".join(sorted(r.kinds))))
    for s in args.tolerate_reg:
        print("  cli:%s" % s)
    off = sorted({r.name for r in RULES} - enabled)
    if off:
        print("  not in force: %s   (--list-rules explains each)" % ", ".join(off))

    print("\n=== register access census ===")
    print_census("MAME", census(mame))
    print_census("HW  ", census(hw))
    m_half = sum(1 for e in mame if e.lanes not in (0, 3))
    h_half = sum(1 for e in hw if e.lanes not in (0, 3))
    print("  half-register accesses: MAME %d, HW %d" % (m_half, h_half))
    if m_half == 0 and h_half > 0:
        print("    MAME's dp83932c::reg_w() takes no mem_mask, so MAME cannot model a")
        print("    half-register write at all.  A HW-only half-register access is a")
        print("    real fidelity gap, but the gap may be on MAME's side.")

    print("\n=== readback audit (software-owned registers only) ===")
    print("  A register the driver wrote and then read back differently is a")
    print("  register-model bug on that side, independent of any alignment.")
    m_audit = readback_audit(mame, AUDIT_REGS)
    h_audit = readback_audit(hw, AUDIT_REGS)
    print_readback("MAME", m_audit)
    print_readback("HW", h_audit)
    audit_disagreement = False
    for reg in AUDIT_REGS:
        m, h = m_audit.get(reg), h_audit.get(reg)
        if h and h["bad"] and not (m and m["bad"]):
            audit_disagreement = True
            print("  *** %s: HW reads back a value it never wrote, MAME does not."
                  % reg_name(reg))
    if not audit_disagreement:
        print("  (no register misbehaves on HW but not on MAME)")

    # ── alignment ────────────────────────────────────────────────────────
    print("\n=== alignment ===")
    mode = args.align
    hw_epochs, mame_epochs = split_epochs(hw), split_epochs(mame)
    if mode == "auto":
        mode = "epoch" if (hw_epochs and mame_epochs) else "slide"
        print("  auto -> %s" % mode)
    print("  software-reset epochs (CR <- RST): MAME %d, HW %d"
          % (len(mame_epochs), len(hw_epochs)))

    tolerated = []
    if mode == "epoch":
        if not hw_epochs or not mame_epochs:
            print("  cannot use epoch alignment: one side has no CR<-RST write.")
            return 2
        hw_s = hw_epochs[-1]
        best_i, best_n, best_tol = -1, -1, []
        for i, ep in enumerate(mame_epochs):
            n, t = common_prefix(hw_s, ep, tol, ignore_lanes)
            if n > best_n:
                best_i, best_n, best_tol = i, n, t
        mame_s = mame_epochs[best_i]
        tolerated = best_tol
        print("  HW: last of %d epochs, %d events" % (len(hw_epochs), len(hw_s)))
        print("  MAME: best-matching epoch #%d of %d, %d events (common prefix %d)"
              % (best_i, len(mame_epochs), len(mame_s), best_n))
    elif mode == "slide":
        hw_s = hw
        off, n = best_slide(hw, mame, tol, ignore_lanes)
        mame_s = mame[off:]
        _, tolerated = common_prefix(hw_s, mame_s, tol, ignore_lanes)
        print("  slid HW over MAME: best start offset %d, common prefix %d" % (off, n))
    else:
        hw_s, mame_s = hw, mame
        _, tolerated = common_prefix(hw_s, mame_s, tol, ignore_lanes)
        print("  comparing from index 0 on both sides (--align first)")

    # ── first divergence ─────────────────────────────────────────────────
    d, reason = None, None
    probe = []
    for i, (a, b) in enumerate(zip(hw_s, mame_s)):
        r = compare_event(a, b, tol, ignore_lanes, probe)
        if r is not None:
            d, reason = i, r
            break
    tolerated = probe

    print("\n=== tolerated differences (up to the divergence) ===")
    if not tolerated:
        print("  none")
    else:
        agg = Counter()
        example = {}
        for a, b, delta, names in tolerated:
            key = (a.rw, a.reg, names)
            agg[key] += 1
            example.setdefault(key, (a, b, delta))
        for (rw, reg, names), n in agg.most_common():
            a, b, delta = example[(rw, reg, names)]
            print("  %s %-5s x%-5d by [%s]  e.g. HW %04x vs MAME %04x (bits %04x)"
                  % (rw, reg_name(reg), n, ",".join(names), a.data, b.data, delta))
        print("  %d event(s) differed but were accepted by a named rule."
              % len(tolerated))
        print("  Re-run with --strict to see them as divergences.")

    ret = 0
    if d is None:
        if len(hw_s) == len(mame_s):
            print("\n=== NO DIVERGENCE ===")
            print("  The two register streams are identical under the rules above.")
            print("  The difference is then NOT in the register access sequence --")
            print("  look at TIMING, at interrupt routing, or at what the DMA engine")
            print("  actually read/wrote in guest memory.")
        else:
            n = min(len(hw_s), len(mame_s))
            shorter = "HW" if len(hw_s) < len(mame_s) else "MAME"
            longer = mame_s if shorter == "HW" else hw_s
            print("\n=== %s TRACE STOPS at event %d; the other side CONTINUES ==="
                  % (shorter, n))
            print("  This is NOT a value mismatch -- every event both sides produced")
            print("  matched.  One side simply stopped emitting.")
            print("\n  last %d matching events before the stop:" % min(args.context, n))
            print_context(hw_s, n - args.context, n)
            print("\n  what the other side does NEXT:")
            print_context(longer, n, n + args.context)
            ret = 1
    else:
        print("\n=== FIRST DIVERGENCE at event %d  (%s) ===" % (d, reason))
        print("  common prefix (last %d events):" % min(args.context, d))
        print_context(hw_s, d - args.context, d)
        print("\n  HW  : %s" % describe(hw_s[d]))
        print("  MAME: %s" % describe(mame_s[d]))
        print("\n  MAME continues:")
        print_context(mame_s, d, d + args.context, marker_at=d)
        print("\n  HW continues:")
        print_context(hw_s, d, d + args.context, marker_at=d)
        ret = 1

    return ret


if __name__ == "__main__":
    sys.exit(main())
