# Deep-fuzz pre-synth gate (`make fuzz-deep`)

> **Status**: landed Phase B (2026-05-07), follows the Phase A audit
> in [`fuzz_audit.md`](fuzz_audit.md).
>
> **Cadence**: pre-impl gate.  Distinct from the legacy `make fuzz`
> (N=200 baseline, runs nightly, write-log policy `auto`).
>
> **Wall-clock budget**: ≤ 5 min on the dev machine.

## Goal

Catch RTL bugs the legacy 200-seed nightly gate is too narrow to see.
The audit identified seven dimensions where the random corpus was
artificially narrow; this gate turns all of them on.

## What's wider than `make fuzz`

| Dimension                  | `make fuzz` | `make fuzz-deep` |
|----------------------------|-------------|------------------|
| Default N                  | 200         | 1500 (tunable up to ~3000 inside the budget) |
| Parallel workers           | 1           | 12 (`--jobs 12`) |
| Per-seed program length    | fixed 40    | mix {10, 40, 200, 1000} (`--instr-mix`) |
| Initial Dn / An state      | A0..A3 LEA, rest 0 | D0..D3 + A4..A6 randomized per seed |
| MOVEM register list size   | 4..6        | 1..15 (full architectural range) |
| MOVEM register pool        | D0..D5 + A0..A3 (10 regs) | D0..D7 + A0..A6 (15, A7 still excluded) |
| DBcc loop iters            | 0..3        | 0..63 |
| Backward Bcc hop count     | 3 (fixed)   | mix {3, 10, 63} |
| `emit_shift_bw_imm`        | weight 0    | weight 1 (B1a) |
| `emit_moveb_mem_src_disp`  | weight 0    | weight 1 (B1a) |
| Sentinel exclusion window  | 16 bytes    | 4 bytes (the actual sentinel size) |
| `a7` in IGNORE_KEYS        | yes         | NO — A7 is compared |
| `--write-log-policy`       | auto        | strict — WRITELOG counts as MISMATCH |
| Exit-code on TIMEOUT       | ignored     | exit 1 |

## What's NOT widened (yet)

| Dimension                  | Why                              |
|----------------------------|----------------------------------|
| ALU memind src/dst family  | RTL stalls under random + memind, see `BUG_alu_memind_timeout_cluster.md`.  Opt-in via `FUZZ_DEEP_EXTRAS=--memind-heavy` once the underlying RTL bug is fixed. |
| Supervisor / FPU / MMU emit | Audit P2 §10–12.  Each needs a non-trivial preamble (privilege flip, PMOVE handler, FPCR reset).  Filed as `B3 follow-up`. |

## Comparison oracle — what's compared at end-of-program

After B1f (drop `a7` from `IGNORE_KEYS`), the post-execution diff covers
the full architectural register state plus all CPU writes:

- Every Dn (D0..D7) — full 32-bit value
- Every An (A0..A7) — full 32-bit value  (A7 was previously ignored;
  fuzz-deep compares it)
- SR (Status Register, including N/Z/V/C/X CCR + supervisor mode + IPL)
- mem[] write log — every byte the RTL drove on the AXI bus, compared
  byte-for-byte against Musashi's per-write callback log

Ignored (legitimately diverge between RTL OoO retired-PC and Musashi
ISS halt-loop PC):

- `cycles`, `committed`, `committed_macros` — different microarchitecture
- `last_pc`, `pc` — RTL reports last-committed PC; Musashi reports the
  halt-loop PC.  Stop-PC parity is covered separately by
  `make musashi-parity`.

## Random initial DRAM (B3 / audit P2 §13)

Set `FUZZ_DEEP_RANDOM_DRAM=1` to seed the first 128 bytes of each
A0..A3 data pool with deterministic-per-seed random bytes BEFORE the
random body runs.  Both models execute the same `move.l #imm,(off,An)`
preamble, so the random bytes are mirrored — any read divergence is
purely RTL.

Without random-dram, every load from a never-explicitly-written
address returns `0xFF` on both models (the mem-model default).  That
silently masks data-dependent bugs:

- A wrong-byte selector in the LSU (e.g. byte-lane shuffle on misaligned
  word load) only diverges from Musashi when the bytes-being-shuffled
  contain different non-0xFF values.
- A cache-line-fill that reads stale post-eviction bytes is
  invisible if the stale bytes are 0xFF (matches default fill).
- An indexed addressing mode that computes the wrong address
  silently agrees with the right address when both addresses return
  the same default 0xFF.

Cost: ~120 preamble instructions per seed → wall-clock budget drops
from N=1500 (default) to N=1000 with random-dram on, both within the
5-min budget.

```
make fuzz-deep                          # N=1500, no random-dram (~2:30)
make fuzz-deep FUZZ_DEEP_RANDOM_DRAM=1  # N=1000, random-dram on (~2:00-2:50)
```

## Knobs (Makefile variables)

```
FUZZ_DEEP_N         (default 1500)   — # seeds
FUZZ_DEEP_JOBS      (default 12)     — parallel workers
FUZZ_DEEP_BASE_SEED (default 100)    — pinned seed for reproducibility
FUZZ_DEEP_TIMEOUT   (default 400000) — RTL cycle cap (per seed)
FUZZ_DEEP_EXTRAS    (default empty)  — extra fuzz.py flags (e.g. --memind-heavy)
```

The pinned base seed is deliberate — when the deep gate fails, the exact
seed list is reproducible by anyone on the same generator state.

## Output format

```
====================================================================
 fuzz-deep: pre-synth gate
  N=1500 jobs=12 base_seed=100
  policy=strict  instr_mix=on  fail_on=MISMATCH,ERROR,TIMEOUT,WRITELOG
  budget: 5 min wall-clock
====================================================================
fuzz: excluded N known-failing seed(s) from <K>-seed manifest
.....................[per-seed dot/W/M/T/E].........................
fuzz: <NN> seeds  PASS=...  WRITELOG=...  MISMATCH=...  TIMEOUT=...  ERROR=...  base_seed=...
====================================================================
 fuzz-deep: PASS  N=1500 wall=151s
====================================================================
```

On FAIL the banner lists the repro seed file and the saved-fails dir;
re-running `make fuzz-replay FILE=tb/fuzz_fails/seed_<N>.bin` deterministically
re-produces a single failure for waveform debug.

## Known-failure manifest

`tb/fuzz_fails/known_failures.txt` lists seeds we're aware of that
trigger an OPEN RTL bug.  Each entry MUST link to a `BUG_md` so the
underlying issue stays tracked.  The deep gate reads this file via
`--exclude-seeds` so a stuck-red bug doesn't permablock CI; remove
the entry once the RTL fix lands so the gate re-validates the seed
on the next run.

Current entries:
- `seed=1419` → `BUG_seed_1419_movel_absl_high_word.md`
  — found in the inaugural fuzz-deep run (2026-05-07).  MOVE.L Dn,
  (xxx).L drops bit-16 of the destination address on a deep program.
  Only fails at N≥1000 instr; the legacy N=40 random corpus could
  never hit this.

## Workflow

```
# Pre-impl gate (block on RTL bugs):
make fuzz-deep

# Override N or seed range:
make fuzz-deep FUZZ_DEEP_N=2500 FUZZ_DEEP_BASE_SEED=42

# Nightly baseline (legacy, 200 seeds, auto policy):
make fuzz N=200
```

## Why two gates?

`make fuzz` proves the random corpus' baseline shape works against
Musashi at the historical width.  Past ~95% of the bugs that ever
landed were caught here.  It's small enough to run on every PR
without burning the developer's clock.

`make fuzz-deep` is the pre-impl gate: before you spend an hour on
synth/route, this 5-minute gate has already swept ~5x more random
shapes and forced every previously-loose comparison knob to strict.
Anything it surfaces would otherwise become a mid-route surprise.
