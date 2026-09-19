#!/usr/bin/env python3
"""csc21_race_classify.py -- classify the $0D24/csCode-21 hardware-presence
race outcome (WON/LOST, per docs/BUG_calibration_word_misplaced_0d00.md
Part 44 Section 5's own signature) from a `coherent-dump` word table, and
separately bucket a sequence of post-release `pc` polls into this
investigation's known terminal-resting-point families (Part 64 Section 5 /
Part 65).

Usage:
    csc21_race_classify.py race   < coherent_dump_output.txt
    csc21_race_classify.py resting < pc_poll_output.txt

Race classification input format: lines of `> coherent mem 0xAAAAAAAA =
0xDDDDDDDD` as printed by jtag_repl.tcl's `coherent-dump` command (one word
per line, ascending address order).

Signature (Part 44 Section 5 / Part 64 Section 4, reproduced twice on real
hardware byte-for-byte):
  LOST: a 24-byte record whose word+0 low byte of the high half-word is
        0x81 (spID=RBV1) with word+8 == 0x00030001 (exact video-class `cat`
        match) and the validity-flag byte (word+20 & 0xFF, offset+23) has
        bit0 CLEAR (record is un-pruned, un-flagged, sitting in the
        ascending scan path BEFORE DAFB/0xDD ever gets reached).
  WON:  no such un-flagged RBV1 record, AND a spID 0xDD (DAFB) record with
        the same cat match and its own validity flag clear is present
        (accepted candidate, csCode21's actual observed healthy pick).
  UNKNOWN: neither signature is unambiguously present in the dumped window
        (e.g. the dump did not cover the record, or the table shape has
        changed) -- reported honestly rather than guessed.
"""
import re
import sys

WORD_RE = re.compile(r"coherent mem\s+(0x[0-9A-Fa-f]+)\s*=\s*(0x[0-9A-Fa-f]+)")

# Known terminal-resting-point families (docs/BUG_calibration_word_misplaced_0d00.md)
MONITOR_CLUSTER_PREFIX = "0x4084a"      # historical 0x4084axxx operator-monitor-loop cluster
CPUSHL_STALL = 0x40887126               # Part 41 Section 2 CPUSHL/dbf cache-maintenance stall
NEW_STALL_0x40885032 = 0x40885032       # Part 64 Section 5, previously-undocumented stall
CSC21_PROBE = 0x40800bf0


def parse_words(text):
    out = []
    for m in WORD_RE.finditer(text):
        addr = int(m.group(1), 16)
        word = int(m.group(2), 16)
        out.append((addr, word))
    return out


def classify_race(text):
    words = parse_words(text)
    if not words:
        return "UNKNOWN", "no coherent-dump words found in input"

    vals = [w for _, w in words]
    addrs = [a for a, _ in words]

    def find_record(spid):
        """A record is 6 words (24 bytes) starting at word i where
        (vals[i] >> 16) == spid and vals[i+2] == 0x00030001 (cat match)."""
        hits = []
        for i in range(len(vals) - 5):
            hi = (vals[i] >> 16) & 0xFFFF
            if hi == spid and vals[i + 2] == 0x00030001:
                flag_word = vals[i + 5]
                flag_byte = flag_word & 0xFF
                valid = (flag_byte & 1) == 0
                hits.append({
                    "addr": addrs[i],
                    "word0": vals[i],
                    "cat": vals[i + 2],
                    "flag_word": flag_word,
                    "flag_byte": flag_byte,
                    "valid": valid,
                })
        return hits

    rbv1 = find_record(0x81)
    dafb = find_record(0xDD)

    rbv1_unflagged = [r for r in rbv1 if r["valid"]]
    dafb_unflagged = [r for r in dafb if r["valid"]]

    if rbv1_unflagged:
        r = rbv1_unflagged[0]
        detail = (f"RBV1 (spID=0x81) record at 0x{r['addr']:08X}: "
                  f"word0=0x{r['word0']:08X} cat=0x{r['cat']:08X} "
                  f"flag_byte=0x{r['flag_byte']:02X} (bit0 clear=VALID) "
                  f"-- un-pruned, un-flagged, ascending-scan-visible before DAFB")
        return "LOST", detail

    if dafb_unflagged:
        r = dafb_unflagged[0]
        detail = (f"DAFB (spID=0xDD) record at 0x{r['addr']:08X}: "
                  f"word0=0x{r['word0']:08X} cat=0x{r['cat']:08X} "
                  f"flag_byte=0x{r['flag_byte']:02X} (bit0 clear=VALID), "
                  f"no un-flagged RBV1 in dump -- accepted-candidate/pruned-pool shape")
        return "WON", detail

    if rbv1 or dafb:
        detail = (f"RBV1 records found={len(rbv1)} (all flagged-invalid), "
                  f"DAFB records found={len(dafb)} (all flagged-invalid) "
                  f"-- neither signature cleanly matches, needs manual review")
        return "UNKNOWN", detail

    return "UNKNOWN", ("neither an un-flagged RBV1 (0x81) nor an un-flagged "
                       "DAFB (0xDD) cat-matching record found in the dumped window "
                       "-- dump likely did not cover the relevant records")


def classify_resting(text):
    """Extract every `pc = 0x........` sample from a poll transcript and
    bucket the STABLE (repeated, non-transient) final value."""
    pcs = []
    for m in re.finditer(r"pc\s*=\s*(0x[0-9A-Fa-f]+)", text, re.IGNORECASE):
        pcs.append(int(m.group(1), 16))
    if not pcs:
        return "UNKNOWN", "no pc samples found in input", []

    def bucket(pc):
        hexs = f"0x{pc:08x}"
        if hexs.startswith(MONITOR_CLUSTER_PREFIX):
            return "monitor-cluster-0x4084axxx"
        if pc == CPUSHL_STALL:
            return "cpushl-dbf-stall-0x40887126"
        if pc == NEW_STALL_0x40885032:
            return "new-stall-0x40885032"
        if pc == CSC21_PROBE:
            return "still-at-probe-0x40800bf0"
        return f"other-0x{pc:08x}"

    buckets = [bucket(p) for p in pcs]
    # "Settled" = the last N samples (up to all of them) are the SAME bucket.
    tail = buckets[-1]
    stable_run = 0
    for b in reversed(buckets):
        if b == tail:
            stable_run += 1
        else:
            break
    settled = stable_run >= min(3, len(buckets))
    verdict = tail if settled else "NOT-SETTLED/BOUNCING"
    detail = f"{len(pcs)} samples, buckets={buckets}, stable_run={stable_run}"
    return verdict, detail, buckets


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("race", "resting"):
        print(__doc__)
        sys.exit(2)
    text = sys.stdin.read()
    if sys.argv[1] == "race":
        verdict, detail = classify_race(text)
        print(f"RACE_VERDICT={verdict}")
        print(f"DETAIL: {detail}")
    else:
        verdict, detail, buckets = classify_resting(text)
        print(f"RESTING_VERDICT={verdict}")
        print(f"DETAIL: {detail}")


if __name__ == "__main__":
    main()
