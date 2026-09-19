# BUG: MOVE/MOVEQ do not update CCR C flag

**Surfaced by:** musashi-wrapper fuzzer, seeds 103, 104, 108 at N=10 base_seed=100.
**Also reproduces at:** minimal hand-written case in
`BUG_move_ccr_c_flag_repro.s` (3 instructions).

## Symptom

After a subtract that sets the carry flag (C=1), any subsequent
`MOVE.L`, `MOVEQ`, or similar MOVE-family instruction should
overwrite C (specifically, clear it since MOVE semantics are "C is
always cleared").  Our RTL leaves C set; Musashi clears it.

Minimal repro:
```
moveq   #1, %d0          ; d0=1 (CCR: N=0 Z=0 V=0 C=0)
subq.l  #2, %d0           ; d0=-1, sets X=1, N=1, C=1 (borrow)
moveq   #0, %d1           ; should clear N, Z=1, V=0, C=0 (keep X)
...                      ; final CCR diverges: rtl=0x19, musashi=0x18
```

## Root cause

`rtl/core/decode/decode.v` sets `flags_wr = 5'b01110` for every
instruction that should update NZVC but not X.  Affected opcodes
(lines from decode.v):

  - MOVEQ                (line 951)
  - MOVE.L Dn,Dm         (line 384)
  - MOVE.L #imm,Dn       (line 396)
  - MOVE.L (An),Dn       (line ~408)
  - MOVE.L (d16,An),Dn   (similar pattern)
  - ORI.L #imm,Dn        (line 229)
  - ANDI.L #imm,Dn       (line 242)
  - EORI.L #imm,Dn       (line 281)
  - OR.L Dn,Dm           (line ~961)
  - AND.L Dn,Dm          (similar)
  - NEG.L / NOT.L        (via UOP_INT path)
  - BTST / BCHG / …      (single bit ops)
  - SWAP / EXT           (similar)

The `flags_wr` bit layout is `{X, N, Z, V, C}` (see `uop_pkg.v`
FLAG_X=4, FLAG_N=3, FLAG_Z=2, FLAG_V=1, FLAG_C=0).  `5'b01110`
therefore enables writes to N, Z, V — but **not C**.  The ALU's
`ALU_MOV` handler already computes the correct value for C
(`alu_flags[FLAG_C] = 1'b0`), but because the mask bit is 0, the
write gets squashed by the flag merge.

## Fix (out of scope for this commit)

Change every MOVE-family `flags_wr = 5'b01110` in decode.v to
`5'b01111` — enables writes to N, Z, V, C (still excludes X, which
MOVE never touches).  The ALU_MOV handler already masks X out
explicitly (line 247 of alu.v), so this is a pure decode fix.

## Evidence from fuzzer

```
$ make fuzz N=10 FUZZ_N=10 (base_seed=100)
fuzz: 10 seeds  PASS=5  MISMATCH=5  TIMEOUT=0  ERROR=0
  MISMATCH seed=103 ccr: rtl='0x09' musashi='0x08'
  MISMATCH seed=104 ccr: rtl='0x09' musashi='0x08'
  MISMATCH seed=108 ccr: rtl='0x19' musashi='0x18'
```

In all three cases, the only divergence is CCR bit 0 (C).  This
matches exactly the decode.v bug above: a carry-setting op (SUBI /
SUB / ADDX borrow path) is followed by a MOVE-family op that fails
to clear C because its `flags_wr` mask has bit 0 clear.

Seeds 102 and 106 show CCR + register divergences because the
undetected C flag then drives a Bcc to the wrong arm of a branch,
cascading the corruption through subsequent instructions.

## Counted as: 1 bug (with cascade symptoms in 5/10 fuzz seeds)
