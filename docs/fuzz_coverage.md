# Fuzz corpus coverage

> **Scope**: what the `tools/fuzz/gen_program.py` generator emits today,
> what artificial bounds remain (and why), and what's still missing.

**Current baseline (main @ 05393e3 + fuzz-corpus branch)**:

| metric              | 200-seed run        | notes                      |
|---------------------|---------------------|----------------------------|
| PASS                | 155–171             | 75–85%                     |
| TIMEOUT             | 27–50               | 15–22%                     |
| WRITELOG            | 0–2                 | 0–1% — triaged separately  |
| MISMATCH            | 0                   | architectural diffs only   |
| ERROR               | 0                   |                            |

`make fuzz N=200` passes the regression gate when `MISMATCH=0`.
Pure write-log-only cases are emitted as `WRITELOG`, saved in
`build/fuzz/repro_seeds.txt`, and can be replayed deterministically.

Memory-write parity is useful but still noisy.  The random fuzzer now
classifies pure write-log-only cases separately, records the matching seeds in
`build/fuzz/repro_seeds.txt`, and keeps architectural mismatches distinct from
write-log triage.  Use `--write-log-policy strict` when you explicitly want
the write log itself to fail the run.  `tools/regstate/regstate_compare.py`
still keeps memory comparison opt-in and folds stable memory effects back
into final registers.

---

## Program structure

Every generated `.s` has:

```
_start:
    lea     0x00080000, %a7              | stack top
    lea     0x00100000, %a0              | data pool 0
    lea     0x00104000, %a1              | data pool 1
    lea     0x00108000, %a2              | data pool 2
    lea     0x0010c000, %a3              | data pool 3
    moveq   #3, %d6                      | backward-branch hop counter
    moveq   #0, %d4                      | DBcc counter (zeroed)
    moveq   #0, %d5                      | DBcc counter (zeroed)
    <N body instructions>

_end:
    move.l  #0xC0FFEE00, %d7
    lea     0xFFFF0000, %a1
    move.l  %d7, (%a1)                   | PASS sentinel
_halt:
    bra     _halt

<deferred subroutine bodies; unreachable by fallthrough>
```

`A0..A3` each hold a disjoint 16 KiB safe-memory window.  `A4..A6`
are LEA/MOVEA targets only.  `A7` is the stack.  `D6` is the hop
counter.  `D4/D5` are DBcc counters.  `D7` is scratch for the
PASS-sentinel store.

## Instruction classes emitted

### Tier-0 — baseline integer
- `moveq #imm8q, Dn`                              (weight 6)
- `move.l Dn, Dm`                                 (5)
- `move.l #imm32, Dn`                             (5)
- `{add,sub,and,or}.l Dn, Dm`                     (6)
- `{addi,subi,andi,ori,eori}.l #imm32, Dn`        (5)
- `{addq,subq}.l #q, Dn`                          (5)
- `cmp.l Dn, Dm`                                  (3)
- `cmpi.l #imm32, Dn`                             (3)
- `tst.l Dn`                                      (3)
- `{neg,not,swap,ext}.l Dn`                       (3)
- `{clr,not,neg}.b Dn`                            (3)
- `move.l Dn, (d16,An)` / `move.l (d16,An), Dn`   (3 / 3)
- `nop`                                           (1)

### Tier-0 — long-form multiply / divide
- `{mulu,muls}.l Dn, Dm`                          (2)
- `{divu,divs}.l Dy, Dq`  (Dq==Dr collapsed form, with D7 re-seeded
  before every divide to avoid /0 traps)          (1)

### Tier-1 — shifts, rotates, bit-ops, branches
- `{asl,asr,lsl,lsr,rol,ror,roxl,roxr}.l #imm, Dn` (4)
- `{asl,...,roxr}.l Dn_count, Dm` — full dynamic-count register
  shifts/rotates; ALU consumes the low six count bits               (3)
- `{bchg,bclr,bset,btst} #n, Dn`                  (3)
- `{bchg,bclr,bset,btst} Dn, Dm`                  (2)
- local backward-Bcc pattern:
  `moveq #HOP,%d6; back_N: addq.l #1,Dn; subq.l #1,%d6; Bcc back_N` (2)
- `DBcc Dn, lbl` — counter pre-seeded via MOVE.W   (2)
- `move.l Dn, (An)` / `move.l (An), Dn`           (2 / 2)
- `move.l Dn, 0x<safe>.L` / `move.l 0x<safe>.L, Dn` (2 / 2)
- forward Bcc (existing)                          (unchanged; driven
  by branch_prob, not INSTRS weight)

### Tier-2 — addressing-mode variety + control flow
- `movea.l {Dn,Am,#imm,(An),(d16,An),(xxx).L}, A{4,5,6}` (2)
- `movea.l (An)+, A{4,5,6}`                        (1)
- ADDX/SUBX chain: `add.l Dn,Dm` + 2..3 × `{addx,subx}.l Dn,Dm` (2)
- `jmp (xxx).L`                                    (1)
- `lea (xxx).L, A{4,5,6}`                          (1)
- `link %a6, #disp; addq.l #1, Dn; unlk %a6`       (1) — guaranteed
  unwind inside body

### Tier-2 — directed-heavy emitters
- `movem.l <regs>, -(%a7); clobber; movem.l (%a7)+, <regs>`
  **weight 1** — 4..6 register round-trips are back in fuzz after
  `movem_bitop_followup.s` covered the old 5+ register bit-op hang.
- `bsr sub_N; ... sub_N: rts` **weight 1** — BSR/RTS round-trips are
  back in the random corpus after the old A7 rewrite hang was resolved.

### Tier-3 — NOT EMITTED (by design)
- Supervisor ops (MOVEC, RTE, MOVES, MOVE USP/SSP) — exception
  frame semantics are brittle enough to need directed tests only.
- `/0` divide — intentionally avoided; D7 re-seed pattern.
- `CHK`, `TRAPV`, `TRAP #n`, `A-line`, `F-line` — exception
  frame round-trip is directed-test territory.
- MMU ops (PTEST, PFLUSH, PMOVE) — phase-3 MMU walker only.

---

## Artificial bounds still in place

These remaining bounds exist in `gen_program.py` to keep the random
corpus deterministic.  They are not standing in for known bad RTL in the
same way the removed shift, MOVEM, DBcc, and BSR limits were.

1. **A4/A5/A6 never used as memory base** — these regs are LEA /
   MOVEA destinations only.  If they were used by a random
   `(d16,An)` load-store emitter, the An value would be whatever
   the previous LEA/MOVEA left there — often pointing into an
   untracked region.  Could be lifted once the memory-model
   tracks a "valid-base regs" set.

2. **D6 reserved as local backward-branch hop counter, D7 reserved
   for divide-seed and sentinel** — neither is a random-emitter
   destination.  Lifting these requires a better "save-restore"
   wrapper around the relevant patterns.

---

## What's still missing

Instructions the decoder accepts but `gen_program.py` doesn't emit:

- **TAS**, **CAS**, **CAS2** — decoded; atomic semantics need
  care so Musashi matches RTL on the compare path.
- **Bitfield ops** (BFCHG/BFCLR/BFEXTS/BFEXTU/BFFFO/BFINS/BFSET/
  BFTST) — decoded-ish; emitter would need careful offset/width
  encoding.
- **Scc** — decoded; no emitter.  Would stress CCR-read semantics
  at byte granularity.
- **TRAPcc** — decoded; conditional trap on cc_true.  Harness
  doesn't handle exception frames cleanly.
- **CHK / CHK2 / CMP2** — decoded-ish; bound-check semantics
  need a known-bounds harness.
- **MOVE.B / MOVE.W** arbitrary source-dest pairs — partial Dn
  writes are fixed and covered, but the random emitter still only
  enables a curated subset of addressing-mode pairs.
- **PACK / UNPK** — decoded; BCD semantics.
- **BCD ops** (ABCD / SBCD / NBCD) — decoded; BCD carry.
- **MOVEP.L / MOVEP.W** — decoded; alternate-byte memory layout.

Addressing modes the decoder accepts but `gen_program.py` still doesn't use:

- `-(An)` predecrement (except A7 in MOVEM-push)
- `(d8,An,Xn)` / `(bd,An,Xn)` full index forms other than the MOVEA case
  above; the MOVE.W brief-extension timeout repro is captured in
  `tb/fuzz_fails/BUG_movew_indexed_word_timeout.md`
- `(d16,PC)` / `(d8,PC,Xn)` PC-relative
- `([bd,An,Xn],od)` full extension word
- Alternate-size (.B, .W) variants for most of the .L body

---

## Raising the bar

To grow the fail-rate-aware coverage:

1. Fix a BUG_*.md, drop its workaround, widen the weight of the
   unblocked emitter, re-run `make fuzz N=500`.
2. Repeat for each BUG.
3. When the writeback noise floor is cleaned (either x-init change
   or a cache-line-dirty-byte fix), expect the replay seed list to
   shrink and `MISMATCH=0` with `TIMEOUT=0..5%` as the new baseline.
4. Tighten the sentinel-timeout budget on both models (currently
   200k cycles each); most programs complete in <1 k cycles.
