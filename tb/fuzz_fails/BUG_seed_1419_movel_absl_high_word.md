# Seed 1419 — MOVE.L imm-Dn-source absolute-long destination address divergence

## Status

FIXED — discovered during Phase B2 deep-fuzz gate landing.  It was
reproducible under `--write-log-policy strict --instr-mix --base-seed 100
--n 1500` or directly via the saved seed list (single seed) at
`--instr 1000`.  At `--instr 40` the same seed passed.  The failing
instruction was isolated to the prefix ending at line 318.

## Symptom

Seed 1419 generates the line:

    move.l  %d4, 0x0010fdb4

at line 318 of the 1000-instruction body (program length matters: 40-
instruction body PASSes for the same seed).  Both models execute the
program to completion; RTL records the AXI write at `0x000ffdb4`,
Musashi records the architectural write at `0x0010fdb4`.  The
addresses differ by exactly `0x10000` — bit 16 of the destination
address is dropped on the RTL side.

Diff-relevant excerpt (full state files saved alongside this BUG_md):

    < mem[0x000ffdb4]=0x00      (RTL)
    < mem[0x000ffdb5]=0x00      (RTL)
    < mem[0x000ffdb6]=0x00      (RTL)
    < mem[0x000ffdb7]=0x00      (RTL)
    > mem[0x0010fdb4]=0x00      (Musashi)
    > mem[0x0010fdb5]=0x00      (Musashi)
    > mem[0x0010fdb6]=0x00      (Musashi)
    > mem[0x0010fdb7]=0x00      (Musashi)

The written value (D4 = 0) is identical on both sides; only the address
diverges.  This is a single divergent instruction in a long program —
classic deep-fuzz finding.

## Investigation update (2026-05-08)

The absolute-long extension word composition is correct at dispatch.  A
DEBUG=1 run of a version that inserts the PASS sentinel immediately after
line 318 shows:

    [DISP cyc=2041] pc=0x4080041c ... imm=0x0010fdb4 ... hsra=0 ... is_st=1
    [LSU] ... base=0xffff0000 disp=0x0010fdb4 ea=0x000ffdb4 ...

So the RTL does decode the store displacement as `0x0010fdb4`; the bad
EA comes from adding a non-zero base of `0xffff0000`.

That base appears to be the supposedly pinned absolute-address zero
source.  Earlier in the same run, the debug stream shows phys 16 traffic:

    [DISP cyc=96]  pc=0x40800042 ... pdst=16 ...   | moveq #0,%d4
    [CDB0 cyc=100] phys=16 data=0x00000000 ...
    [FREE cyc=209] phys=16
    ...
    [CDB0 cyc=1681] phys=16 data=0xffff0000 ...

`PHYS_ZERO_TAG`/phys 16 is therefore not staying permanently zero.  When
the absolute store reaches LSU, the correct displacement is added to the
corrupted phys-16 PRF value, producing `0xffff0000 + 0x0010fdb4 =
0x000ffdb4`.

The local BFEXTS + absolute-store pattern alone does not reproduce:

    /tmp/seed1419_min_bf_abs.bin replay: states match

The repro still needs earlier zero-elim / phys-16 history, but later
instructions are not involved:

    /tmp/seed1419_stop_after_318.bin replay: same address mismatch

This points at the RAT/PRF zero-tag invariant, not at decode immediate
assembly, `agu.v`, or `lsu.v`.

## Reproducer

Replay the saved binary (deterministic, doesn't require gen_program):

    make fuzz-replay FILE=tb/fuzz_fails/seed_00001419.bin

Or with the seed file:

    echo 1419 > /tmp/seeds && \
    python3 tools/fuzz/fuzz.py --seed-file /tmp/seeds \
        --sim build/sim/Vmac_top \
        --musashi tb/models/libmusashi_ref.a \
        --work build/fuzz --instr 1000 \
        --write-log-policy strict

## Saved repro files

- `tb/fuzz_fails/seed_00001419.s`            — generated assembly source
- `tb/fuzz_fails/seed_00001419.bin`          — assembled flat binary
- `tb/fuzz_fails/seed_00001419.rtl.txt`      — RTL final-state dump
- `tb/fuzz_fails/seed_00001419.musashi.txt`  — Musashi golden state dump

## Fix

Fixed in `rtl/core/rename/rat.v` and `rtl/core/m68k_core_execute.vh` by
making the phys-16 zero-source invariant explicit at both boundaries:

- RAT allocation/reclaim masks can no longer free or allocate
  `PHYS_ZERO_TAG` or the reserved USP/SSP/ISP phys slots.
- The integer PRF forces `PHYS_ZERO_TAG` back to zero every cycle, reads
  phys16 as zero, and blocks same-cycle CDB bypass data for phys16.

This matches the architectural contract used by absolute-address memory
ops: phys16 is not a normal physical register, it is a hard zero source.

Verification:

    make sim
    make fuzz-replay FILE=tb/fuzz_fails/seed_00001419.bin
    make lint

The saved replay now reports `states match, no mismatch`.

This bug was caught by:

1. `--write-log-policy strict` (B1g) — auto policy would have classed
   this as WRITELOG and silently dropped it.
2. `--instr-mix` (B0/B1c) — the seed only fails at N=1000, not the
   default N=40.

Both knobs are part of the deep-fuzz gate by design.  The seed has been
removed from `known_failures.txt` so future deep-fuzz runs include it again.
