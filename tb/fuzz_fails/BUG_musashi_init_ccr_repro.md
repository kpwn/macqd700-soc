# BUG: Musashi reset leaves CCR with Z=1; RTL has CCR=0

**Surfaced by:** fuzz seeds 106, 144, 176 at BASE_SEED=100.

## Symptom

Short fuzz programs that begin with a Bcc on initial CCR state
diverge between the RTL and Musashi — RTL's ccr_rat resets all
flags to 0 (BEQ not taken), but Musashi's m68k_pulse_reset() does
not touch the per-flag C-style variables (not_z_flag, flag_n,
flag_v, flag_c, flag_x).  The zero-init on Musashi's `not_z_flag`
makes COND_EQ() true → initial CCR Z=1.

Example (seed 106):
```
_start:
    lea     0x0010f500, %a0
    bls    fwd_1       ; Musashi: Z=1 → BLS taken; RTL: Z=0 → fall through
    ...
```

## Root cause

tb/models/m68k_ref.cpp, `MusashiRef::reset()`.  Musashi's
pulse_reset does not zero the CCR bits of SR (only T1/T0/S/M/IM
are explicitly set).  The C globals for the flags happen to start
zero, which maps to "Z=1, all others clear" because FLAG_Z stores
the *inverse* of the Z bit.

## Fix

After `m68k_pulse_reset()`, explicitly clear the CCR bits
(`sr & ~0x001F`) so both models agree on "CCR = 0 at reset".

## Evidence

Before fix: 100 seeds → PASS=94  MISMATCH=6
After fix:  100 seeds → PASS=97  MISMATCH=3

The three fixed seeds (106, 144, 176) all had a Bcc in the first
few instructions before any CCR-writing op.
