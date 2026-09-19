# BUG: dual brief-indexed MOVE.L with src/dst disp=0, scale=1 — RTL drops the STORE

**Status**: OPEN.  fuzz-deep MISMATCH on seeds 999 and 1391 (2026-05-20,
HEAD `91864935`).  Both seeds hit the same diverging instruction.

## Symptom

fuzz-deep replay of `tb/fuzz_fails/seed_00000999.bin` and
`seed_00001391.bin`:

```
mem[0x0010032c..0x0010032f]: rtl=None musashi='0x39 0x26 0xa6 0x7d'
mem[0x00100420..0x00100423]: rtl=None musashi='0x39 0x26 0xa6 0x7d'
```

(Plus downstream cascade — register A2 off by 4 on seed 1391, byte
writes at 0x10002f / 0x1000c2 off because subsequent addressing
computations diverge from the missed store onwards.)

## Root cause (suspected)

Both `.s` files contain the same instruction shape at the divergence:

```
    lea     (808,%a4), %a1            | A1 = base + 808
    lea     (1052,%a4), %a0           | A0 = base + 1052
    moveq   #4, %d2
    move.l  #0x3926a67d, (4,%a1)      | write to A1+4
    move.l  #0x00000000, (4,%a0)      | write to A0+4
    .word   0x21b1, 0x2000, 0x2000    | MOVE.L (0,A1,D2.W*1),(0,A0,D2.W*1)
```

The `.word` encodes `MOVE.L (0,A1,D2.W*1),(0,A0,D2.W*1)` — dual brief-
indexed MOVE.L with both src and dst displacement = 0, scale = 1,
size = LONG.  Musashi's behaviour: read 4 bytes from A1+0+D2.W = A1+4,
write 4 bytes to A0+0+D2.W = A0+4.  RTL: store never lands.

Commit `50fd5abf` added the dual-indexed MOVE crack to
`rtl/core/decode/decode_uop_assemble.v` (the `move_dii_dst_*_phase`
sequence).  The crack's directed test (`tb/tests/asm/move_idx_idx.s`)
verified MOVE.B with src scale=8/dst scale=1, MOVE.B src=1/dst=1, and
MOVE.W src=2/dst=4 — but NOT MOVE.L, and NOT src/dst displacement=0.
fuzz `N=250` at landing also missed this seed range (failures at 999
and 1391).

Bisect: at parent commit `476fcdca`, the same `.word 0x21b1` hits vec-4
illegal-instruction (the crack hadn't landed yet) — also fails, just
differently.  So `50fd5abf` partially handles the instruction but the
crack drops the final STORE under this specific corner — probably an
off-by-one in `move_dii_dst_base_phase` /
`move_dii_dst_disp_phase` when both `src_index_scale=0` and
`dst_index_scale=0` (no doublings on either side), or a missing branch
when src displacement = 0 (the ph0 ALU_ADD becomes a redundant add of
zero).

## How to reproduce

```
make sim musashi-ref musashi
make fuzz-replay FILE=tb/fuzz_fails/seed_00000999.bin
```

Expected fail.  Replay against parent `476fcdca` reproduces the
older vec-4 form.  Replay against `50fd5abf` itself reproduces the
current "store dropped" form.

## Path to a fix

1. Build a minimal directed test:
   `move_idx_idx_long_zero_disp.s` — MOVE.L (0,An,Dm.W*1),(0,Am,Dm.W*1)
   with explicit pre-fill + post-check at the dst byte addresses.
2. Trace the uop crack with `+pic_trace`-equivalent decode logging
   (`+rename_trace`?) to confirm whether the STORE uop is emitted
   and what `arch_src_a/arch_src_b` it carries.
3. Likely fixes to inspect:
   - `move_dii_dst_disp_phase` ordering when `dst_index_scale==0` —
     does the `else if (uop_phase < move_dii_dst_base_phase)` doubling
     loop accidentally swallow ph5 when base == idx+1 (no doublings)?
   - The `else` (STORE) fallthrough — is the dispatcher fetching
     `uop_phase > disp_phase` as the last uop, or stopping at disp_phase
     because `last_uop` got asserted one phase early?
4. Widen `tools/fuzz/gen_program.py emit_move_idx_idx` to randomise
   src/dst disp ∈ {0, ±4, ±8} and size ∈ {B, W, L} so future fuzz
   covers this matrix.

## Track

- Both seeds saved alongside this file:
  `seed_00000999.s/.bin/.rtl.txt/.musashi.txt`,
  `seed_00001391.s/.bin/.rtl.txt/.musashi.txt`.
- Added to `tb/fuzz_fails/known_failures.txt` so `make fuzz-deep`
  excludes them until the fix lands.  Remove the exclusion when fixed
  — the deep-fuzz gate will then re-validate automatically.
