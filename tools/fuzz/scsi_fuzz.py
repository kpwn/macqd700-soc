#!/usr/bin/env python3
# scsi_fuzz.py — differential fuzzer for the NCR 53C96 SCSI controller
# model (rtl/mac/scsi.v, TURBOSCSI_C96=1) against MAME's ncr53c90 as the
# golden reference — the Musashi methodology applied to SCSI.
#
# Both sides execute the SAME generated register-access script:
#   * RTL:  tb/tb_scsi_fuzz.cpp — by default Verilated as
#           build/scsi_fuzz_pb/Vtb_pb_scsi, i.e. REAL peripheral_bus.v +
#           REAL scsi.v + vhdd_sd driven over AXI4, so the pseudo-DMA
#           aperture's word-splitting serializers are inside the DUT.
#           build/scsi_fuzz/Vtb_scsi_vhdd_sd (make fuzz-scsi-direct) is
#           the scsi.v-only shape, kept for attribution.
#   * MAME: tools/mame_scsi96_fuzz.lua inside a macqd700 boot with the CPU
#           parked, driving the chip through the DAFB TurboSCSI aperture
#           (byte AND 16-bit accesses; the DAFB DRQ-check bits are armed
#           through the real control register at 0xf9800024).
# and emit result logs in an identical format; this script generates the
# inputs, runs both sides, and diffs the logs at defined sync points.
# Comparison contract, exclusions and false-positive sources are
# documented in docs/scsi_fuzz.md — read that before triaging a report.
#
# Usage:
#   tools/fuzz/scsi_fuzz.py --n 50                # fuzz 50 seeds from 0
#   tools/fuzz/scsi_fuzz.py --n 50 --start 1000   # seeds 1000..1049
#   tools/fuzz/scsi_fuzz.py --seed 1234           # one seed
#   tools/fuzz/scsi_fuzz.py --replay 1234         # alias of --seed
#   tools/fuzz/scsi_fuzz.py --gen-only --seed 7   # print the script
#   tools/fuzz/scsi_fuzz.py --n 50 --mask-cmd     # drop cmd= from the diff
# Normally invoked via `make fuzz-scsi [N=..] [START=..]`.
#
# Results are reported in two tiers: a CORE rate (aperture payloads,
# FIFO, DRQ, tcount, status/seq/flags/istat) and a stricter rate that
# also compares the cmd= command echo + queue depth.  See
# docs/scsi_fuzz.md; cmd= used to be masked unconditionally and that hid
# a load-bearing wedge mechanism.

import argparse
import os
import random
import re
import shutil
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

DISK_LBAS = 32768          # 16 MiB deterministic disk, must match tb + CHD
TARGET_ID = 6              # live target (RTL -GTARGET_ID=6; MAME :scsi:6:harddisk)
CDROM_ID = 3               # MAME default config ships a CD-ROM here; the
                           # RTL build has no target 3, so the generator
                           # NEVER selects ID 3 (see docs/scsi_fuzz.md)
SELECT_IDS = [0, 1, 2, 4, 5, 6, 7]          # 7 = chip's own ID
WRITE_LBA_LO, WRITE_LBA_HI = 0x300, 0x3ff   # writes stay in this window;
                                            # random reads stay out of it

# ── deterministic disk pattern (must match tb_scsi_fuzz.cpp + CHD) ───────
def disk_byte(lba, i):
    return (lba * 197 + i * 13 + ((lba >> 8) * 59) + 7) & 0xff


def build_disk_image(img_path):
    with open(img_path, "wb") as f:
        for s in range(DISK_LBAS):
            f.write(bytes(disk_byte(s, i) for i in range(512)))


# ═══════════════════════════════════════════════════════════════════════
# Script generation
# ═══════════════════════════════════════════════════════════════════════

class Gen:
    """Script generator.

    profile "clean": only stimulus shapes whose RTL semantics are known to
    match MAME (the regression-gate profile — expected green).
    profile "full": additionally emits the shapes the fuzzer has PROVEN
    divergent (bare-CDB ATN selects, ATN_STOP with CDB in FIFO, junk/
    truncated/over-long CDBs, empty-FIFO selects) — used to characterize
    the known select-path modelling gap and to validate future RTL fixes.
    See docs/scsi_fuzz.md "Findings".
    """

    def __init__(self, seed, num_lbas=DISK_LBAS, profile="clean"):
        self.rng = random.Random(seed)
        self.ops = []
        self.sync_n = 0
        self.profile = profile
        self.num_lbas = num_lbas
        if num_lbas >= 0x800:
            self.wr_lo, self.wr_hi = WRITE_LBA_LO, WRITE_LBA_HI
        else:                        # tiny image: scale the write window
            self.wr_lo, self.wr_hi = num_lbas // 4, num_lbas // 2

    def op(self, s):
        self.ops.append(s)

    def gap(self, p=0.3, lo=0, hi=64):
        # RTL-only sub-frame timing perturbation; MAME ignores GAP.
        if self.rng.random() < p:
            self.op("GAP %d" % self.rng.randint(lo, hi))

    def sync(self):
        self.op("SETTLE")
        self.op("SYNC %d" % self.sync_n)
        self.sync_n += 1

    def reg3_cmd(self, v):
        # 0x46 / 0xC6 (SELECT_ATN3) passes MAME's 53c90a validity check
        # but has NO start_command case: mame0285 ncr53c90.cpp:1057
        # fatalerror()s — the golden model itself dies.  Never emit it.
        while (v & 0x7f) == 0x46:
            v = self.rng.randrange(256)
        return v

    def reg4_val(self):
        # any byte whose &7 != CDROM_ID (MAME masks bus_id to 3 bits)
        while True:
            v = self.rng.randrange(256)
            if (v & 7) != CDROM_ID:
                return v

    def normalize(self):
        # Renormalization preamble: returns both sides to an identical
        # baseline from ANY state (incl. mid-transaction wreckage).
        # W 8 sets config1 (and the chip's own bus ID = low 3 bits);
        # chip reset preserves config's low 3 bits, so writing it makes
        # the baseline independent of earlier junk config writes.
        #
        # CTRL 000 is FIRST and is load-bearing: the DAFB TurboSCSI
        # control word lives OUTSIDE the 53C96, so neither a chip reset
        # nor a bus reset clears it.  Without this a block that armed the
        # DRQ-check would leak its hold-off behaviour into every
        # following block (and, because both sides run a batch of scripts
        # back-to-back against one live chip, into the next seed).
        self.op("CTRL 000")
        self.op("W 3 02")        # chip reset
        self.op("W 8 07")        # config1: own ID 7
        self.op("W 3 03")        # SCSI bus reset (unwedges connected targets)
        self.op("SETTLE")
        # Interrupt drain is UNCOMPARED: when a disruptor interrupted a
        # select mid-arbitration, the istatus wreckage that surfaces here
        # is timing-dependent (measured: 0x18 vs 0x80).  Drained twice;
        # the SYNC below then compares the converged state, istat=0
        # included.
        self.op("RU 5")
        self.op("SETTLE")
        self.op("RU 5")
        self.op("W 3 01")        # flush FIFO
        self.op("W 4 06")
        self.op("W 5 01")        # select timeout (constrained <= 7 globally)
        self.op("W 6 05")
        self.op("W 7 00")
        self.op("W 9 02")
        self.op("W 0 00")
        self.op("W 1 00")
        self.sync()

    # ── CDB builders ────────────────────────────────────────────────────
    def cdb_for(self, kind):
        r = self.rng
        if kind == "tur":
            return [0x00, 0, 0, 0, 0, 0], 0, "none"
        if kind == "reqsense":
            n = r.choice([0, 8, 18, 32])
            return [0x03, 0, 0, 0, n, 0], n or 4, "dev"
        if kind == "inquiry":
            n = r.choice([4, 8, 36, 64])
            return [0x12, 0, 0, 0, n, 0], n, "dev"
        if kind == "modesense":
            n = r.choice([4, 12, 24])
            page = r.choice([0x00, 0x01, 0x03, 0x04, 0x30, 0x3f])
            return [0x1a, 0, page, 0, n, 0], n, "dev"
        if kind == "readcap":
            return [0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0], 8, "disk"
        if kind == "read6":
            lba = self._read_lba()
            nb = r.randint(1, 3)
            return [0x08, (lba >> 16) & 0x1f, (lba >> 8) & 0xff, lba & 0xff,
                    nb, 0], nb * 512, "disk"
        if kind == "read10":
            lba = self._read_lba()
            nb = r.randint(1, 3)
            return [0x28, 0, (lba >> 24) & 0xff, (lba >> 16) & 0xff,
                    (lba >> 8) & 0xff, lba & 0xff, 0, (nb >> 8) & 0xff,
                    nb & 0xff, 0], nb * 512, "disk"
        if kind == "write10":
            lba = self.rng.randint(self.wr_lo, self.wr_hi)
            nb = r.randint(1, 2)
            return [0x2a, 0, (lba >> 24) & 0xff, (lba >> 16) & 0xff,
                    (lba >> 8) & 0xff, lba & 0xff, 0, 0, nb, 0], nb * 512, "wr"
        if kind == "junkop":
            n = r.choice([6, 10, 12])
            cdb = [r.randrange(256) for _ in range(n)]
            # avoid accidentally forming a huge valid read/write
            if cdb[0] in (0x08, 0x0a, 0x28, 0x2a):
                cdb[0] = 0xee
            return cdb, 0, "none"
        raise ValueError(kind)

    def _read_lba(self):
        # random reads stay OUT of the write window (see docs/scsi_fuzz.md)
        while True:
            if self.profile == "full":
                # may cross the end of the disk: finding F9
                tail = self.num_lbas - 4 + self.rng.randint(0, 3)
            else:
                tail = self.num_lbas - 4
            lba = self.rng.choice(
                [self.rng.randint(0, min(255, self.num_lbas - 1)),
                 self.rng.randint(0, self.num_lbas - 4),
                 tail])
            if not (self.wr_lo <= lba <= self.wr_hi + 4):
                return lba

    # ── building blocks ────────────────────────────────────────────────
    def block_valid_txn(self):
        r = self.rng
        full = (self.profile == "full")
        if full:
            kinds = ["tur", "reqsense", "inquiry", "modesense",
                     "readcap", "read6", "read6", "read10", "read10",
                     "write10", "junkop"]
        else:
            # reqsense/modesense: the two TARGET models supply different
            # payload lengths, which leaks into chip-visible tcounter/
            # TC0/command-queue state — excluded from the gate profile
            # (device-level, not 53C96-level; see docs/scsi_fuzz.md)
            kinds = ["tur", "inquiry", "readcap", "read6", "read6",
                     "read10", "read10", "write10"]
        kind = r.choice(kinds)
        cdb, dlen, dclass = self.cdb_for(kind)
        tgt = r.choice([TARGET_ID] * 5 + [i for i in SELECT_IDS if i != TARGET_ID])
        present = (tgt == TARGET_ID)

        sel = r.choice([0x41, 0x42, 0x42, 0x43, 0x43])
        if full and r.random() < 0.2:
            sel |= 0x80          # DMA-form select (tcount pre-loaded or not)
        if full:
            # free-form: IDENTIFY optional and decoupled from the select
            # form — the shape family PROVEN divergent (docs/scsi_fuzz.md)
            identify = r.choice([None, None, 0x80, 0xc0,
                                 0x80 | r.randint(1, 7)])
            trunc = r.random() < 0.10       # truncated CDB
            extra = r.random() < 0.10       # over-long CDB
        else:
            # MAME-compatible coupling: bare SELECT gets a bare CDB; ATN
            # gets IDENTIFY + CDB (the fixed c96_sel_strip contract).
            # ATN_STOP is FULL-profile only: finding F2 (docs/scsi_fuzz.md)
            # — the RTL posts no stop-interrupt after the IDENTIFY.
            if sel == 0x43:
                sel = r.choice([0x41, 0x42])
            identify = None if sel == 0x41 else r.choice([0x80, 0xc0])
            trunc = extra = False

        # Real drivers flush before selecting; without this, a FIFO left
        # dirty by e.g. a timed-out select re-creates finding F1 shapes.
        if not full:
            self.op("W 3 01")

        self.op("W 4 %02x" % tgt)
        self.op("W 5 %02x" % r.choice([0, 1, 1, 2]))
        if identify is not None:
            self.op("W 2 %02x" % identify)
        push = list(cdb)
        if not full and sel == 0x43:
            push = []                       # ATN_STOP: IDENTIFY only
        if trunc and len(push) > 2:
            push = push[: r.randint(1, len(push) - 1)]
        if extra:
            push += [r.randrange(256) for _ in range(r.randint(1, 3))]
        for b in push:
            self.op("W 2 %02x" % b)
            self.gap(0.15)
        self.op("W 3 %02x" % sel)
        self.sync()

        if not present:
            return                        # selection timed out; SYNC read it

        # ATN_STOP stops after the IDENTIFY: send the CDB as a CMD-phase
        # DMA transfer, like the real Mac driver does.
        if sel == 0x43 and (not full or r.random() < 0.7):
            n = len(cdb)
            self.op("W 0 %02x" % (n & 0xff))
            self.op("W 1 00")
            self.op("W 3 90")
            self.op("DW %x %s" % (n, " ".join("%02x" % b for b in cdb)))
            self.sync()

        self._aborted = False
        if dclass in ("disk", "dev") and dlen > 0:
            self._data_in(dlen, dclass)
        elif dclass == "wr":
            self._data_out(dlen)

        if not self._aborted:
            self._complete(dclass)

    def _data_in(self, dlen, dclass):
        r = self.rng
        if self.profile == "full":
            mode = r.choice(["exact", "exact", "partial", "over", "zero"])
        else:
            # partial/zero tcount modes are FULL-profile: findings F4/F5
            mode = r.choice(["exact", "exact", "exact", "over"])
        tc = {"exact": dlen, "partial": dlen, "over": dlen,
              "zero": 0}[mode]
        drain = {"exact": dlen, "partial": max(2, (dlen // 2) & ~1),
                 "over": dlen + r.choice([2, 8]), "zero": min(dlen, 16)}[mode]
        if self.profile != "full" or r.random() < 0.8:
            # DMA form
            self.op("W 0 %02x" % (tc & 0xff))
            self.op("W 1 %02x" % ((tc >> 8) & 0xff))
            self.op("W 3 90")
            self.gap(0.5)
            # Access width is part of the stimulus space now: a 16-bit
            # host read is ncr53c94_device::dma16_swap_r, ONE DRQ check
            # and an atomic two-byte pop — not two byte reads.  Only
            # emitted for even drains (an odd byte count cannot be
            # expressed in whole words).
            # BLIND drains deliberately do NOT appear here: this path
            # arms the whole transfer length, so the target is still
            # streaming while the burst runs, and MAME's frozen clock vs
            # the RTL's running one would make the comparison about
            # refill timing rather than about the chip.  Blind drains
            # live in block_rom_chunk_drain, which arms exactly one FIFO
            # load and settles to TC0 first — the ROM's real condition.
            if (drain % 2) == 0 and drain > 0 and r.random() < 0.4:
                self.op("%s %x" % (
                    "DR16" if dclass == "disk" else "DRU16", drain // 2))
            else:
                self.op("%s %x" % ("DR" if dclass == "disk" else "DRU", drain))
            if dclass == "dev":
                self.op("W 3 01")     # flush: fifo may hold device-
                                      # dependent bytes (excluded class)
            self.sync()
        else:
            # non-DMA: interrupt per byte; static bounded loop
            self.op("W 3 10")
            for _ in range(min(drain, 10)):
                self.op("SETTLE")
                self.op("RC 5")
                self.op("%s 2" % ("RC" if dclass == "disk" else "RU"))
                self.op("W 3 10")
            self.op("W 3 01")
            self.sync()

    def _data_out(self, dlen):
        r = self.rng
        clean = self.profile != "full" or r.random() < 0.6
        n = dlen if clean else max(2, r.randint(2, dlen) & ~1)
        data = [r.randrange(256) for _ in range(n)]
        self.op("W 0 %02x" % (dlen & 0xff))
        self.op("W 1 %02x" % ((dlen >> 8) & 0xff))
        self.op("W 3 90")
        if (n % 2) == 0 and n > 0 and r.random() < 0.35:
            # 16-bit host push: dma16_swap_w, which degrades to a single
            # dma_w when fifo_pos > 14 || tcounter == 1 — a case a byte
            # replay of the same word cannot produce.
            words = [(data[i] << 8) | data[i + 1] for i in range(0, n, 2)]
            if self.profile == "full" and r.random() < 0.4:
                per = self.FIFO_DEPTH // 2
                for i in range(0, len(words), per):
                    self.op("SETTLE")
                    self.op("DWB16 %x %s" % (
                        len(words[i:i + per]),
                        " ".join("%04x" % w for w in words[i:i + per])))
            else:
                self.op("DW16 %x %s" % (
                    n // 2, " ".join("%04x" % w for w in words)))
        else:
            self.op("DW %x %s" % (n, " ".join("%02x" % b for b in data)))
        if not clean:
            # mid-transfer settled state: finding F6 family — compare
            # only after renormalizing
            self.normalize()
            self._aborted = True
            return
        if self.profile != "full":
            self.op("W 3 01")   # finding F6: RTL leaves 16 bytes in the
                                # FIFO after a completed DMA-out; MAME
                                # drains to 0.  Flush before comparing.
        self.sync()

    def _complete(self, dclass):
        r = self.rng
        if r.random() < 0.12:
            # abandon the transaction mid-flight; renormalize
            self.normalize()
            return
        self.op("W 3 11")            # CI_COMPLETE
        self.sync()
        self.op("RC 2")              # status byte (device-independent)
        self.op("RC 2")              # message byte
        self.op("W 3 12")            # CI_MSG_ACCEPT
        self.sync()

    def block_write_readback(self):
        # clean WRITE(10) then READ(10) of the same LBA: catches wrong
        # LBA/direction/data through BOTH backends
        r = self.rng
        lba = r.randint(self.wr_lo, self.wr_hi)
        nb = 1
        data = [r.randrange(256) for _ in range(512)]
        for cdb in ([0x2a, 0, 0, 0, (lba >> 8) & 0xff, lba & 0xff, 0, 0, nb, 0],):
            self.op("W 4 %02x" % TARGET_ID)
            self.op("W 5 01")
            self.op("W 2 80")        # IDENTIFY, then CDB: the fixed
                                     # c96_sel_strip path (the motivating bug)
            for b in cdb:
                self.op("W 2 %02x" % b)
            self.op("W 3 42")
            self.sync()
        self.op("W 0 00")
        self.op("W 1 02")
        self.op("W 3 90")
        self.op("DW 200 %s" % " ".join("%02x" % b for b in data))
        self.op("W 3 01")   # finding F6: flush RTL's post-DMA-out residue
        self.sync()
        self.op("W 3 11")
        self.sync()
        self.op("RC 2")
        self.op("RC 2")
        self.op("W 3 12")
        self.sync()
        # read it back
        cdb = [0x28, 0, 0, 0, (lba >> 8) & 0xff, lba & 0xff, 0, 0, nb, 0]
        self.op("W 4 %02x" % TARGET_ID)
        identify = self.rng.choice([None, 0x80, 0xc0])
        if identify is not None:
            self.op("W 2 %02x" % identify)
        for b in cdb:
            self.op("W 2 %02x" % b)
        self.op("W 3 %02x" % (0x42 if identify is not None else 0x41))
        self.sync()
        # (identify None -> bare SELECT: MAME-compatible in both profiles)
        self.op("W 0 00")
        self.op("W 1 02")
        self.op("W 3 90")
        self.op("DR 200")
        self.sync()
        self.op("W 3 11")
        self.sync()
        self.op("RC 2")
        self.op("RC 2")
        self.op("W 3 12")
        self.sync()

    def block_junk_burst(self):
        r = self.rng
        armed = False                 # a W3 may have started *anything*
        for _ in range(r.randint(3, 10)):
            c = r.random()
            if c < 0.55:
                reg = r.randrange(16)
                val = r.randrange(256)
                if reg == 5:
                    val &= 7          # bound the select timeout (quiesce cap)
                if reg == 4:
                    val = self.reg4_val()
                if reg == 3:
                    armed = True
                    val = self.reg3_cmd(val)
                self.op("W %x %02x" % (reg, val))
            elif c < 0.8:
                # reg 3 (command echo) is finding F8: MAME's 2-deep queue
                # pops on istatus reads, the RTL echo does not — always RU
                reg = r.choice([0, 1, 4, 6, 7, 8, 0xb, 0xc, 9, 0xa, 0xd, 0xe, 0xf])
                self.op("%s %x" % ("RU" if armed else "RC", reg))
                if r.random() < 0.1:
                    self.op("RU 3")
            elif c < 0.92:
                self.op("RU %x" % r.choice([2, 5]))
                armed = True
            else:
                # DAFB control noise: arm/disarm the DRQ checks in the
                # middle of junk.  normalize() below puts it back to 000.
                self.op("CTRL %03x" % r.choice([0x000, 0x080, 0x100, 0x180,
                                                r.randrange(0x200)]))
            self.gap(0.4)
        self.normalize()

    def block_orphans(self):
        r = self.rng
        for _ in range(r.randint(1, 3)):
            cmds = [0x10, 0x90, 0x11, 0x12, 0x1a, 0x13, 0x44, 0x45,
                    0x20, 0x21, 0x27, 0xff]
            if self.profile == "full":
                # empty-FIFO selects and free random commands live here
                # (0x46 excluded everywhere: it fatalerror()s MAME)
                cmds += [0x47, self.reg3_cmd(r.randrange(256))]
            cmd = r.choice(cmds)
            if r.random() < 0.4:
                self.op("W 0 %02x" % r.randrange(256))
                self.op("W 1 %02x" % r.randrange(4))
            self.op("W 3 %02x" % cmd)
            self.op("SETTLE")
            self.op("RC 4")
            self.op("RC 5")
        self.normalize()

    def block_dma_noise(self):
        r = self.rng
        for _ in range(r.randint(1, 3)):
            c = r.random()
            if c < 0.25:
                self.op("DR %x" % r.choice([1, 2, 4]))
            elif c < 0.40:
                # blind byte pops of whatever residue is in the FIFO —
                # MAME's fifo_pop() is a memmove, so an empty-FIFO
                # dma_r() repeats the last-popped byte (finding F15)
                self.op("DRB %x" % r.choice([1, 2, 3, 5]))   # <= FIFO depth
            elif c < 0.60:
                # 16-bit pops: dma16_r's `fifo_pos < 2` underflow case
                # (ncr53c90.cpp:1327-1329) returns dma_r() | 0xff00, i.e.
                # ONE pop with 0xff in the other half — structurally
                # different from two byte pops, which is exactly why an
                # aperture fuzzer that only speaks bytes proves nothing.
                self.op("%s %x" % (r.choice(["DR16", "DRB16", "DRB16"]),
                                   r.choice([1, 2, 3])))
            elif c < 0.75:
                n = r.choice([2, 4])
                self.op("DW %x %s" % (
                    n, " ".join("%02x" % r.randrange(256) for _ in range(n))))
            elif c < 0.90:
                # dma16_w degrades to a SINGLE dma_w of the high byte
                # when fifo_pos > 14 || tcounter == 1 (ncr53c90.cpp:
                # 1352-1358); a byte-replay of the same word pushes both.
                n = r.choice([1, 2, 4])
                self.op("%s %x %s" % (
                    r.choice(["DW16", "DWB16"]), n,
                    " ".join("%04x" % r.randrange(0x10000) for _ in range(n))))
            else:
                n = r.choice([1, 2, 4])
                self.op("DWB %x %s" % (
                    n, " ".join("%02x" % r.randrange(256) for _ in range(n))))
        self.sync()

    # A BLIND burst issues its beats with no DRQ poll and, on the MAME
    # side, with no emulated time in between — so it can only be
    # compared as far as one already-staged FIFO load (16 bytes).  Any
    # longer and the two sides are comparing "who got to advance time",
    # not the chip.  The ROM never blind-drains more than a chunk
    # either, so this is the real shape, not a workaround.
    FIFO_DEPTH = 16

    def _blind_bytes(self, n):
        return max(1, min(n, self.FIFO_DEPTH))

    def _blind_words(self, n_bytes):
        return max(1, min(n_bytes, self.FIFO_DEPTH) // 2)

    def block_rom_chunk_drain(self):
        """The flow the Quadra 700 ROM actually runs, and the one that
        put a Sad Mac 0F02 on the board with `make fuzz-scsi` green.

        SELECT + READ(6), config3 = LBTM (the ROM writes 0x04 at PC
        0x40899120), then the payload is drained in fixed-size chunks:
        per chunk the transfer counter is loaded, DMA|CI_XFER is armed,
        and the host issues eight BLIND `move.w` at the pseudo-DMA
        aperture — no DRQ poll between beats.  Under LBTM the BUSMD_1
        DMA_IN DRQ formula is `fifo_pos > 1`, so a byte-granular paced
        drain stalls at the odd occupancy; only a 16-bit pop, which
        skips that occupancy, reaches fifo_pos == 0.

        The DRQ-check bits are exercised here too (CTRL 080/180), which
        is what turns every hold-off path in scsi.v and peripheral_bus.v
        from dead code into compared behaviour."""
        r = self.rng
        full = (self.profile == "full")
        self.op("CTRL 000")
        self.op("W c %02x" % r.choice([0x04, 0x04, 0x04, 0x00]))  # LBTM
        self.op("W 4 %02x" % TARGET_ID)
        self.op("W 5 01")
        self.op("W 3 01")                     # flush before selecting
        identify = r.choice([None, 0x80, 0x80])
        if identify is not None:
            self.op("W 2 %02x" % identify)
        lba = self._read_lba()
        cdb = [0x08, (lba >> 16) & 0x1f, (lba >> 8) & 0xff, lba & 0xff, 1, 0]
        for b in cdb:
            self.op("W 2 %02x" % b)
        self.op("W 3 %02x" % (0x42 if identify is not None else 0x41))
        self.sync()

        # DAFB control for the drain.  080 = DRQ-check reads (what the
        # ROM sets), 180 = reads AND writes, 000 = the blind aperture.
        self.op("CTRL %03x" % r.choice([0x000, 0x080, 0x080, 0x180]))
        for _ in range(r.randint(1, 4)):
            chunk = r.choice([16, 16, 16, 8] + ([32, 4, 2] if full else []))
            style = r.random()
            if style < 0.65 or style >= 0.92:
                # BLIND styles: the armed count MUST equal what the burst
                # will drain, and must fit one FIFO load, so the chip
                # reaches TC0 with nothing left to stage.  Otherwise the
                # target keeps streaming during the burst — and since
                # MAME's clock is frozen inside a Lua op burst while the
                # RTL's is not, the two sides would be comparing which
                # executor got to refill, not the drain.  This is the
                # ROM's own condition: MAME trace record #1612 shows
                # fifo=16 AND TC0 before the first DMAR of #1613.
                chunk = min(chunk, self.FIFO_DEPTH)
            self.op("W 0 %02x" % (chunk & 0xff))
            self.op("W 1 %02x" % ((chunk >> 8) & 0xff))
            self.op("W 3 90")
            self.gap(0.4)
            if style < 0.65:
                # BLIND drain — the ROM's real one.  The SETTLE is
                # load-bearing and is NOT a fudge: MAME advances zero
                # emulated time inside a Lua op burst, so without it the
                # golden side would still have an empty FIFO while the
                # RTL (which advances a cycle per bus beat) has a full
                # one.  The ROM's flow has exactly this shape — MAME
                # trace record #1612 shows fifo=16 + TC0 BEFORE the first
                # DMAR of record #1613 — so settling first is faithful,
                # and it is what makes the blind drain a comparison of
                # the DRAIN rather than of harness timekeeping.
                self.op("SETTLE")
                if style < 0.45:
                    self.op("DRB16 %x" % self._blind_words(chunk))
                else:
                    self.op("DRB %x" % self._blind_bytes(chunk))
            elif style < 0.80:
                self.op("DR16 %x" % (chunk // 2))
            elif style < 0.92:
                self.op("DR %x" % chunk)
            else:
                # mixed-width drain: some words, then bytes — leaves the
                # odd FIFO occupancy a pure word drain never sees, which
                # is where the split-beat DRQ grant lives
                # A settle between the halves is required, not cosmetic:
                # the first half leaves the transfer mid-stream, and the
                # chip keeps staging while the RTL executor advances
                # cycles and MAME's clock stands still.  Without it the
                # second half compares who refilled first (measured:
                # seed 12 of the 2026-08-19 run took one byte more than
                # MAME on the PB build while matching it on the direct
                # build — pure executor cadence, not a DUT difference).
                self.op("SETTLE")
                self.op("DRB16 %x" % max(1, self._blind_words(chunk) // 2))
                self.op("SETTLE")
                self.op("DRB %x" % max(1, self._blind_bytes(chunk) // 2))
            self.sync()
        self.op("CTRL 000")
        self.normalize()

    def block_pdma_write_chunk(self):
        """Write-side twin of block_rom_chunk_drain.  WRITE(10) staged
        through the aperture with 16-bit host accesses, optionally with
        the DAFB write DRQ-check (bit 8) armed.

        Two mechanisms only this block can reach: MAME's dma16_w
        degradation to a single dma_w at `fifo_pos > 14 || tcounter == 1`
        (ncr53c90.cpp:1352-1358), and the SoC's write-side split-beat
        gating — peripheral_bus.v drives scsi_dma16_lo_beat from
        rd_scsi_dma16_lo ONLY (peripheral_bus.v:1722), so the second byte
        of a split word write re-checks DRQ where MAME never does."""
        r = self.rng
        lba = r.randint(self.wr_lo, self.wr_hi)
        cdb = [0x2a, 0, 0, 0, (lba >> 8) & 0xff, lba & 0xff, 0, 0, 1, 0]
        self.op("CTRL 000")
        self.op("W 3 01")
        self.op("W 4 %02x" % TARGET_ID)
        self.op("W 5 01")
        self.op("W 2 80")                    # IDENTIFY
        for b in cdb:
            self.op("W 2 %02x" % b)
        self.op("W 3 42")
        self.sync()
        self.op("CTRL %03x" % r.choice([0x000, 0x100, 0x100, 0x180]))
        self.op("W 0 00")
        self.op("W 1 02")                    # tcount = 512
        self.op("W 3 90")
        words = [r.randrange(0x10000) for _ in range(256)]
        style = r.random()
        if style < 0.45:
            self.op("DW16 100 %s" % " ".join("%04x" % w for w in words))
        elif style < 0.70:
            # word body, ONE-byte tail: leaves tcounter == 1 in front of
            # a 16-bit push, MAME's single-dma_w degradation case
            self.op("DW16 ff %s" % " ".join("%04x" % w for w in words[:255]))
            self.op("DW 2 %02x %02x" % (words[255] >> 8, words[255] & 0xff))
        else:
            # BLIND staging, chunked to one FIFO load at a time with a
            # settle between chunks — the only shape in which a blind
            # burst is comparable at all (see _blind_bytes).  This is
            # also the only path that can leave `fifo_pos > 14` in front
            # of a 16-bit push, MAME's other dma16_w degradation case.
            per = self.FIFO_DEPTH // 2      # 8 words = one FIFO load
            for i in range(0, 256, per):
                self.op("SETTLE")
                self.op("DWB16 %x %s" % (
                    per, " ".join("%04x" % w for w in words[i:i + per])))
        self.op("CTRL 000")
        self.op("W 3 01")   # finding F6: flush post-DMA-out residue
        self.sync()
        self.op("W 3 11")
        self.sync()
        self.op("RC 2")
        self.op("RC 2")
        self.op("W 3 12")
        self.sync()

    def block_fifo_games(self):
        r = self.rng
        n = r.randint(1, 20)          # over-fill past 16 on purpose
        for _ in range(n):
            self.op("W 2 %02x" % r.randrange(256))
            self.gap(0.2)
        self.op("RC 7")
        if r.random() < 0.5:
            for _ in range(r.randint(1, 4)):
                self.op("RC 2")       # idle pops are settled: comparable
        self.op("W 3 01")
        self.sync()

    def block_midflight_disrupt(self):
        r = self.rng
        # start a select, then disrupt it before it can settle
        kind = r.choice(["read6", "inquiry", "tur"])
        cdb, _dlen, _dclass = self.cdb_for(kind)
        tgt = r.choice([TARGET_ID] * 3 + [1, 5])
        self.op("W 4 %02x" % tgt)
        self.op("W 5 01")
        if r.random() < 0.5:
            self.op("W 2 %02x" % r.choice([0x80, 0xc0]))
        for b in cdb:
            self.op("W 2 %02x" % b)
        self.op("W 3 %02x" % r.choice([0x41, 0x42, 0x43]))
        self.gap(0.9, 0, 40)
        d = r.random()
        if d < 0.25:
            self.op("W 3 02")         # chip reset mid-select
        elif d < 0.5:
            self.op("W 3 03")         # bus reset mid-select
        elif d < 0.65:
            self.op("W 3 01")         # flush mid-select
        elif d < 0.8:
            self.op("W 8 %02x" % r.randrange(256))   # conf write mid-select
        else:
            self.op("RU %x" % r.choice([2, 5]))      # destructive read mid-flight
        # states may legitimately differ now: renormalize before comparing
        self.normalize()

    def generate(self):
        self.normalize()
        self.op("SDGAP %d" % self.rng.choice([0, 0, 4, 16, 24]))
        blocks = [
            (self.block_valid_txn, 5),
            (self.block_write_readback, 1),
            (self.block_junk_burst, 2),
            (self.block_orphans, 2),
            (self.block_dma_noise, 1),
            (self.block_fifo_games, 1),
            (self.block_midflight_disrupt, 2),
            # The pseudo-DMA aperture as the Q700 actually drives it.
            # Weighted heavily: these are the only blocks that reach the
            # 16-bit entry points and the DRQ hold-off paths, and they
            # are the ones a green run has to mean something about.
            (self.block_rom_chunk_drain, 4),
            (self.block_pdma_write_chunk, 2),
        ]
        if self.profile != "full":
            # orphan commands at bus-free are FULL-profile: finding F3
            blocks = [(b, w) for b, w in blocks if b != self.block_orphans]
        weighted = [b for b, w in blocks for _ in range(w)]
        for _ in range(self.rng.randint(2, 5)):
            self._aborted = False
            self.rng.choice(weighted)()
        self.sync()
        self.op("END")
        return "\n".join(self.ops) + "\n"


def generate_script(seed, num_lbas=DISK_LBAS, profile="clean"):
    return ("# scsi_fuzz seed %d profile %s\n" % (seed, profile)
            + Gen(seed, num_lbas, profile).generate())


# ═══════════════════════════════════════════════════════════════════════
# Log diffing (the comparison contract lives here — see docs/scsi_fuzz.md)
# ═══════════════════════════════════════════════════════════════════════

SYNC_RE = re.compile(r"^(SYNC \S+ )(.*)$")


def canon_line(line, mask_cmd=False):
    """Reduce a log line to its compared form."""
    if line.startswith("RU "):
        return line.split("=")[0] + "=--"
    if line.startswith("DRU"):
        # device-dependent payloads: both the bytes AND the length are
        # target-model properties (vhdd vs nscsi_harddisk), not 53C96
        # properties — masked (see docs/scsi_fuzz.md exclusions).
        # Covers DRU / DRU16 / DRUB / DRUB16.  The trailing to= flag is
        # deliberately NOT masked: an aperture beat that never terminates
        # is a defect whatever the payload was going to be.
        return re.sub(r"got=[0-9a-f]+ data=[0-9a-f]*", "got=-- data=--", line)
    m = SYNC_RE.match(line)
    if m and mask_cmd:
        # cmd= (command echo + queue depth).  COMPARED BY DEFAULT since
        # 2026-08-19: MAME's command_w DROPS a command with S_GROSS_ERROR
        # at command_pos == 2 and QUEUES WITHOUT STARTING at 1, so a chip
        # that echoes a command in reg 3 and never runs it is exactly the
        # shape of a board wedge — masking it hid a load-bearing
        # mechanism (docs/scsi_fuzz.md blind spot #5), and ruling it out
        # during the 2026-08-19 hang had to be done by hand.
        # --mask-cmd restores the old behaviour when bisecting an
        # unrelated regression through the known-noisy retirement
        # micro-timing.
        return m.group(1) + re.sub(r"cmd=[0-9a-f]+:\d", "cmd=--", m.group(2))
    return line


def diff_logs(rtl_path, mame_path, mask_cmd=False):
    """Return None if equivalent, else (line_no, rtl_line, mame_line)."""
    with open(rtl_path) as f:
        rtl = [l.rstrip("\n") for l in f]
    with open(mame_path) as f:
        mame = [l.rstrip("\n") for l in f]
    n = max(len(rtl), len(mame))
    for i in range(n):
        a = rtl[i] if i < len(rtl) else "<missing>"
        b = mame[i] if i < len(mame) else "<missing>"
        if canon_line(a, mask_cmd) != canon_line(b, mask_cmd):
            return (i + 1, a, b)
    return None


# ═══════════════════════════════════════════════════════════════════════
# Runner
# ═══════════════════════════════════════════════════════════════════════

def find_mame_assets(args):
    roms = args.mame_roms or os.environ.get(
        "MAME_Q700_ROMS", os.path.expanduser("~/mame_q700_good/roms"))
    if not os.path.isdir(roms):
        sys.exit("scsi_fuzz: MAME rompath %s not found "
                 "(set --mame-roms or MAME_Q700_ROMS)" % roms)
    return roms


def ensure_chd(work, image=None):
    """Return (chd_path, num_lbas).  With --disk-image, MAME's CHD is
    built from the user's image and the RTL side serves the same file, so
    SCSI read/write payloads are compared against real disk content.
    Neither side modifies the image (MAME writes go to the diff dir, the
    RTL side to an in-memory overlay)."""
    if image:
        image = os.path.abspath(image)
        num = os.path.getsize(image) // 512
        if num < 16:
            sys.exit("scsi_fuzz: disk image too small (<16 sectors)")
        chd = os.path.join(work, "userdisk.chd")
        if (not os.path.exists(chd)
                or os.path.getmtime(chd) < os.path.getmtime(image)):
            print("scsi_fuzz: building CHD from %s (%d sectors)..."
                  % (image, num))
            subprocess.run(["chdman", "createhd", "-f", "-i", image,
                            "-o", chd, "-c", "none"],
                           check=True, capture_output=True)
        return chd, num
    chd = os.path.join(work, "fuzzdisk.chd")
    if os.path.exists(chd):
        return chd, DISK_LBAS
    img = os.path.join(work, "fuzzdisk.img")
    print("scsi_fuzz: building deterministic disk (%d sectors)..." % DISK_LBAS)
    build_disk_image(img)
    subprocess.run(["chdman", "createhd", "-f", "-i", img, "-o", chd,
                    "-c", "none"], check=True, capture_output=True)
    os.unlink(img)
    return chd, DISK_LBAS


def run_rtl(binary, batch_dir, image=None):
    cmd = [binary, "--dir", batch_dir]
    if image:
        cmd += ["--img", os.path.abspath(image)]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=1800)
    if r.returncode != 0:
        print(r.stdout[-2000:])
        print(r.stderr[-2000:])
        raise RuntimeError("RTL harness failed (rc=%d)" % r.returncode)


def run_mame(args, batch_dir, nscripts, work, roms, chd):
    for d in ("mame_cfg", "mame_nvram", "mame_diff"):
        p = os.path.join(work, d)
        shutil.rmtree(p, ignore_errors=True)
        os.makedirs(p)
    env = dict(os.environ)
    env["QT_QPA_PLATFORM"] = "offscreen"
    env["SCSI_FUZZ_DIR"] = os.path.abspath(batch_dir)
    secs = 60 + 8 * nscripts
    cmd = [args.mame, "macqd700",
           "-rompath", roms,
           "-cfg_directory", os.path.join(work, "mame_cfg"),
           "-nvram_directory", os.path.join(work, "mame_nvram"),
           "-diff_directory", os.path.join(work, "mame_diff"),
           "-video", "none", "-sound", "none", "-nothrottle",
           "-seconds_to_run", str(secs),
           "-hard", chd, "-skip_gameinfo", "-autoboot_delay", "0",
           "-autoboot_script",
           os.path.join(REPO, "tools", "mame_scsi96_fuzz.lua")]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=900, env=env)
    out = r.stdout + r.stderr
    if "scripts executed" not in out:
        print(out[-3000:])
        raise RuntimeError("MAME executor did not complete")


def fuzz(args):
    work = os.path.abspath(args.work)
    os.makedirs(work, exist_ok=True)
    binary = os.path.abspath(args.rtl)
    if not os.path.exists(binary):
        sys.exit("scsi_fuzz: RTL harness %s missing — run "
                 "`make tb-scsi-fuzz-harness`" % binary)
    roms = find_mame_assets(args)
    chd, num_lbas = ensure_chd(work, args.disk_image)

    def run_batch(batch, tag):
        """Run one batch; returns {seed: divergence-or-None-or-'nolog'}."""
        batch_dir = os.path.join(work, "batch_" + tag)
        shutil.rmtree(batch_dir, ignore_errors=True)
        os.makedirs(batch_dir)
        for s in batch:
            with open(os.path.join(batch_dir, "seed_%08d.txt" % s), "w") as f:
                f.write(generate_script(s, num_lbas, args.profile))
        run_rtl(binary, batch_dir, args.disk_image)
        run_mame(args, batch_dir, len(batch), work, roms, chd)
        res = {}
        for s in batch:
            stem = os.path.join(batch_dir, "seed_%08d" % s)
            rl, ml = stem + ".rtl.log", stem + ".mame.log"
            if not os.path.exists(ml):
                res[s] = "nolog"
                continue
            # TWO-TIER classification (2026-08-19).  `cmd=` used to be
            # masked unconditionally, which hid command-queue retirement
            # — a load-bearing wedge mechanism (docs/scsi_fuzz.md blind
            # spot #5).  It is compared now, but a cmd=-ONLY divergence
            # is reported separately instead of being allowed to shadow
            # the first aperture/FIFO/DRQ divergence further down the
            # log, which is what actually matters for boot.
            core = diff_logs(rl, ml, mask_cmd=True)
            strict = None if args.mask_cmd else diff_logs(rl, ml, mask_cmd=False)
            res[s] = (core, strict)
        return res, batch_dir

    seeds = args.seeds
    fails = []       # (seed, divergence, order_dependent)
    cmd_only = []    # seeds whose ONLY divergence is the cmd= field
    bs = args.batch
    for base in range(0, len(seeds), bs):
        batch = seeds[base:base + bs]
        res, batch_dir = run_batch(batch, "main")
        for s in batch:
            core, strict = res[s] if isinstance(res[s], tuple) else (res[s], None)
            if core is None and strict is None:
                print("seed %d: OK" % s)
                continue
            # Scripts share a live chip within a batch (both sides), so a
            # real divergence in seed k can cascade into k+1.  Re-verify
            # each failure standalone before reporting it.
            sres, sdir = run_batch([s], "solo")
            score, sstrict = (sres[s] if isinstance(sres[s], tuple)
                              else (sres[s], None))
            if score is None and sstrict is None:
                print("seed %d: order-dependent divergence (cascade from an"
                      " earlier seed in the batch; passes standalone)" % s)
                continue
            if score == "nolog" or core == "nolog":
                print("seed %d: FAIL (no MAME log — executor died?)" % s)
                fails.append((s, None, False))
                continue
            if score is None:
                # Aperture / FIFO / DRQ / status state all agree; only the
                # command echo + queue depth differ.
                ln, a, b = sstrict
                print("seed %d: cmd=-ONLY divergence at log line %d" % (s, ln))
                print("   rtl : %s" % a[:200])
                print("   mame: %s" % b[:200])
                cmd_only.append(s)
                keep = os.path.join(work, "fails_cmd", "seed_%08d" % s)
                shutil.rmtree(keep, ignore_errors=True)
                os.makedirs(keep)
                stem = os.path.join(sdir, "seed_%08d" % s)
                for ext in (".txt", ".rtl.log", ".mame.log"):
                    if os.path.exists(stem + ext):
                        shutil.copy(stem + ext, keep)
                continue
            ln, a, b = score
            print("seed %d: DIVERGENCE at log line %d" % (s, ln))
            print("   rtl : %s" % a[:200])
            print("   mame: %s" % b[:200])
            fails.append((s, score, False))
            keep = os.path.join(work, "fails", "seed_%08d" % s)
            shutil.rmtree(keep, ignore_errors=True)
            os.makedirs(keep)
            stem = os.path.join(sdir, "seed_%08d" % s)
            for ext in (".txt", ".rtl.log", ".mame.log"):
                if os.path.exists(stem + ext):
                    shutil.copy(stem + ext, keep)
    print("=" * 60)
    core_pass = len(seeds) - len(fails) - len(cmd_only)
    print("scsi_fuzz: %d/%d seeds passed (core state: aperture payloads, "
          "FIFO, DRQ, tcount, status/seq/flags/istat)"
          % (len(seeds) - len(fails), len(seeds)))
    if not args.mask_cmd:
        print("scsi_fuzz: %d/%d seeds passed with cmd= (command echo + "
              "queue depth) compared too" % (core_pass, len(seeds)))
    if cmd_only:
        print("cmd=-only divergent seeds: %s"
              % " ".join(str(s) for s in cmd_only))
        print("  (artifacts in %s/fails_cmd/ — these are command-queue "
              "retirement micro-timing, NOT aperture state)" % work)
    if fails:
        print("failing seeds: %s" % " ".join(str(s) for s, _, _ in fails))
        print("artifacts in %s/fails/  — replay one with:" % work)
        print("  python3 tools/fuzz/scsi_fuzz.py --seed <N>")
        return 1
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=0, help="number of seeds")
    ap.add_argument("--start", type=int, default=0, help="first seed")
    ap.add_argument("--seed", type=int, help="run a single seed")
    ap.add_argument("--replay", type=int, help="alias of --seed")
    ap.add_argument("--gen-only", action="store_true",
                    help="print the generated script and exit")
    ap.add_argument("--profile", choices=["clean", "full"], default="full",
                    help="full (default): the whole stimulus space — junk, "
                         "mid-flight disrupts, dirty-FIFO selects included; "
                         "clean: the restricted driver-shaped subset (useful "
                         "for bisecting a broad regression)")
    ap.add_argument("--mask-cmd", action="store_true",
                    help="EXCLUDE the cmd= sync field (command echo + "
                         "queue depth) from the comparison.  It is "
                         "compared by DEFAULT since 2026-08-19 — command-"
                         "queue retirement is a load-bearing mechanism "
                         "and masking it hid a class of wedge.  Use this "
                         "only to bisect an unrelated regression through "
                         "the known-noisy retirement micro-timing.")
    ap.add_argument("--strict", action="store_true",
                    help=argparse.SUPPRESS)   # back-compat no-op
    ap.add_argument("--batch", type=int, default=8,
                    help="scripts per MAME invocation")
    ap.add_argument("--work", default=os.path.join(REPO, "build", "fuzz_scsi"))
    ap.add_argument("--rtl", default=os.path.join(
        REPO, "build", "scsi_fuzz_pb", "Vtb_pb_scsi"))
    ap.add_argument("--mame", default=os.environ.get("MAME", "mame"))
    ap.add_argument("--mame-roms", default=None)
    ap.add_argument("--disk-image", default=None,
                    help="serve THIS image as the SCSI disk on both sides "
                         "(MAME gets a CHD built from it, the RTL SD model "
                         "reads the file directly; neither side modifies it)")
    args = ap.parse_args()

    single = args.seed if args.seed is not None else args.replay
    if args.gen_only:
        if single is None:
            sys.exit("--gen-only needs --seed")
        sys.stdout.write(generate_script(single, DISK_LBAS, args.profile))
        return 0
    if single is not None:
        args.seeds = [single]
    elif args.n > 0:
        args.seeds = list(range(args.start, args.start + args.n))
    else:
        sys.exit("give --n <count> or --seed <N>")
    return fuzz(args)


if __name__ == "__main__":
    sys.exit(main())
