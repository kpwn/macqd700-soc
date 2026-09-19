# BUG: MOVE.L with memory source/dest doesn't update CCR

**Surfaced by:** fuzz seeds 148, 159, 196 at BASE_SEED=100 (after the
prior MOVE-register-form C-flag fix).

## Symptom

Per the 68k PRM, every MOVE instruction — regardless of addressing
mode — should set NZVC from the moved data (X unchanged, V=C=0).
Our decoder only emits flags_wr for the Dn↔Dn and imm→Dn forms;
the memory-form MOVE.L's (both load and store) are decoded as a
bare UOP_LOAD/UOP_STORE with flags_wr=0.  The LSU doesn't compute
flags, so CCR stays stale across every memory-form MOVE.L.

Concrete repro (seed 148 fragment):

```
    ...
    andi.l  #0xc29fa4e5, %d0   ; CCR: N=0 Z=0 V=0 C=0
    bcc    fwd_2                ; taken
fwd_2:
    moveq   #39, %d6            ; CCR: N=0 Z=0 V=0 C=0
    move.l  %d7, (24,%a0)       ; stores d7.  MUSASHI: updates CCR from d7
                                 ; RTL: leaves CCR unchanged  ← BUG
    ...
    move.l  (248,%a0), %d2      ; loads 0xFFFFFFFF (uninit mem).
                                 ; MUSASHI: CCR.N=1  RTL: CCR.N stays 0
    bge    fwd_3                ; MUSASHI fall-through (N=1); RTL takes branch
                                 ; → divergent control flow
```

## Root cause

rtl/core/decode/decode.v, lines 412-511 (the six MOVE.L memory
addressing modes):
  - MOVE.L (An),Dn
  - MOVE.L (d16,An),Dn
  - MOVE.L Dn,(An)
  - MOVE.L Dn,(d16,An)
  - MOVE.L (xxx).L,Dn
  - MOVE.L Dn,(xxx).L
  - MOVE.L (xxx).W,Dn
  - MOVE.L Dn,(xxx).W

Each emits exactly one UOP_LOAD or UOP_STORE with flags_wr=0.
The LSU has no path to compute or broadcast CCR, so the flag
update is silently dropped.

## Fix

Crack each of the 8 memory-form MOVE.L's into 2 μops so the flag
update rides the normal iq_int+ALU+ccr_rat path:

- **Load forms**: (phase 0) UOP_LOAD into Dn, then (phase 1)
  UOP_INT ALU_TST reading the same Dn, flags_wr=NZVC.  Phase 1
  naturally depends on phase 0's rename of Dn, so the TST's
  flag write waits for the loaded data via the int CDB wake-up —
  no new architectural ports needed.

- **Store forms**: (phase 0) UOP_INT ALU_TST on the source Dn,
  flags_wr=NZVC; (phase 1) UOP_STORE unchanged.  Flag write and
  store run in parallel (both source Dn), ordering via
  commit-time retirement keeps CCR visible in program order.

## Trade-off: bench cycle drift

Bench cycle counts for memory-heavy workloads drift upward (each
MOVE.L memory op is now 2 μops instead of 1):

  bench_fullpipe:   681 → 886 cycles  (+205)
  bench_mixed_mem:  320 → 390 cycles  (+70)

Other benches drift ≤ 2 cycles.  The correctness win is mandatory
for Mac OS boot; the cycle cost can be recovered later by
teaching the LSU to drive the CCR CDB directly on load completion
and by giving stores a parallel flag-only μop path.  For now we
favour correctness.

## Evidence

N=100 BASE_SEED=100:     PASS=97 → 100
N=200 BASE_SEED=100:     PASS=200 / 200
N=200 BASE_SEED=1000:    PASS=200 / 200
N=500 BASE_SEED=10000:   PASS=500 / 500
(total 900 random programs, zero divergences post-fix)

Directed suite: 72 PASS preserved (jmp_jsr_return and
multiply_test are pre-existing failures unrelated).
