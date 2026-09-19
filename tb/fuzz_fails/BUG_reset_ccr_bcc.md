# BUG (suspected): cold-start Bcc may see stale CCR tag

**Surfaced by:** fuzzer seed 106 (also seed 102 tail cascade).

## Symptom

When a program executes a Bcc as one of its first few instructions
(before any CCR-writing op has dispatched), our RTL appears to take
a branch that Musashi does NOT take.

## Minimal evidence

`tb/fuzz_fails/seed_00000106.s` starts with:

```
_start:
    lea     0x0010f500, %a0     ; does not write CCR
    bls    fwd_1                ; BLS = C | Z
    ...
```

At reset, Musashi's CCR = 0 (X=N=Z=V=C=0), so BLS is not taken;
Musashi falls through and runs ~25 more instructions before halting.
Our RTL evidently resolves BLS as taken — its committed count is
11 vs Musashi's ~40-equivalent, and every Dn beyond D0/D4 ends
at 0 rather than the value Musashi computes.

## Hypothesis

`ccr_rat`'s reset is supposed to seed the committed CCR slot to
zero, but the *speculative* crat_tag used by the very first Bcc's
flag read may point at an un-initialised physical CCR slot whose
contents happen to produce a "taken" vote for several of the
cc-codes that BLS/LS decode into.

This is a secondary finding — the CCR C-flag decode bug
(BUG_move_ccr_c_flag.md) is almost certainly more common and
should be fixed first.  This one may dissolve once the reset CCR
plumbing is audited.

## Repro

```
# Save seed 106
make fuzz-replay FILE=tb/fuzz_fails/seed_00000106.bin
```

## Counted as: 1 suspected bug (not isolated to a 3-line repro yet)
