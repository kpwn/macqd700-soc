# Fuzz infrastructure audit (Phase A)

> **Date**: 2026-05-07
> **Scope**: bug-hiding behavior in `tools/fuzz/{fuzz.py,gen_program.py}`,
> the comparison oracle in `tb/tb_top.cpp` + `tb/models/musashi_run.cpp`,
> and the regression-gate defaults.
> **Baseline**: `make test` 620/0/0 PASS, `make fuzz N=200` 200/200 PASS
> at write_log_policy=auto.
>
> **Core question**: does the fuzz infra itself have bugs that were
> tolerable when the RTL had matching bugs, but now hide real RTL bugs?

## Executive summary

Yes — the fuzz infra has several real bug-hiding holes that the user's
intuition is correct about, but most of them are **deliberate** (filed
against existing BUG_*.md). Three classes are unambiguous regressions
worth fixing right now in Phase B:

1. **Iteration-count default is too low**. `FUZZ_N=500` (or `N=200` for
   the gate). With 40 instructions/program, that's 8000–20000 random
   instructions per gate run. Modern fuzz cadence (e.g. afl, hypothesis)
   wants 1e5–1e6 inputs per run for any non-trivial state space. We
   should crank N to 10000+ for nightly, while keeping 200 for the
   PR-blocking gate.

2. **Many emitters silently sit at weight 0** while their assertion is
   "we'll re-enable later" — but nobody re-enables. This includes
   `emit_shift_bw_imm`, `emit_moveb_mem_src_disp`, the entire ALU memind
   src/dst family, TRAPcc, MOVES, MOVEC, MOVE USP, TRAP, ILLEGAL. These
   represent decoded-and-believed-tested op shapes that the random
   corpus never touches. (See "Disabled-by-weight" matrix below.)

3. **Architecturally-narrow generator constraints** mask the entire
   classes of bugs by construction:
   - Body length fixed at `N=40` instructions; long-tail interactions
     (ROB-fill plus IRQ during MOVEM, deep stack, tens-of-thousands-of-
     iterations loops) cannot trigger.
   - Backward-branch hop counter capped at `HOP_LIMIT=3` so loop bodies
     execute at most 3 iters — a SUBQ.L #1,Dn / Bcc back loop with
     loop-carried CCR dependency on iteration N>3 is structurally
     unreachable.
   - DBcc counter capped at `r.randint(0, 3)`.
   - All registers initialized **deterministically** in the preamble
     (LEA into A0..A3, MOVEQ #0 into D4/D5/D6) — D0..D3 are reset-
     defined and start identical on both models. There is **zero
     randomization** of initial register state, which masks any
     reset-value-dependent bug.
   - RAM is initialized to 0xFF (mem_model.h) and never randomized
     between seeds. Loads from a never-written address return a
     deterministic 0xFF. Any bug that depends on garbage / random
     payload data in a loaded value is invisible.

4. **Comparison oracle has documented swallow-classes** that are
   sometimes legitimate and sometimes hiding bugs:
   - `IGNORE_KEYS = {cycles, committed, committed_macros, last_pc, pc, a7}`.
     The PC/A7 ignore was a known SR-fix-blocker (commit `c1746a77`
     re-enabled SR after it had been ignoring divergence for months).
     Today PC/last_pc and A7 are still always-ignored. The A7 ignore
     hides any spurious stack-pointer adjustment downstream of the
     PASS-sentinel store on either side.
   - `_is_sentinel_key()` suppresses mem[0xFFFF0000..0xFFFF000F] — fine
     for the 4-byte sentinel write, but the 16-byte window is wider
     than necessary (could accept stray writes adjacent to the
     sentinel).
   - The "WRITELOG" classification (write-log-only diff with matching
     regs+CCR) is **not counted as a failure** under the default
     `write_log_policy=auto`. A real RTL bug that manifests only as a
     spurious memory write will be silently logged but the run will
     report "200/200 PASS". The dcache writeback stale-bytes BUG_md
     literally documents that we routed around it instead of fixing.

5. **No supervisor / FPU / MMU fuzzing whatsoever.** The generator is
   user-mode only. Vec=2 (bus error), vec=3 (address error), vec=8
   (privilege violation), every TRAP frame, every MMU fault, every FPU
   op — all untested by the random corpus. The ROM-boot harness covers
   *some* of this, but the cosim-vs-Musashi differential is restricted
   to the user-integer slice.

The good news: each of these is fixable, and the existing BUG_*.md
catalog shows the team has internalized the "widen as you fix" pattern.
Phase B can demonstrably surface real bugs by lifting any of the above.

## Generator constraints — full matrix

### Disabled-by-weight (weight 0 in INSTRS)

| Emitter | Reason given | Notes / risk |
|---|---|---|
| `emit_shift_bw_imm` | "byte/word shift Dn-form, keep weight 0 until family passes targeted slices" | Sub-comment says "the upper-Dn preservation gap is fixed". Likely safe to enable. |
| `emit_moveb_mem_src_disp` | Same reasoning | Same |
| `emit_alu_memind_src_dn` | "8+ µops per memind op, push past cycle cap" | Real cycle-budget concern; but timeout is configurable |
| `emit_alu_memind_src_preidx` | same | same |
| `emit_alu_memind_src_postidx` | same | same |
| `emit_alu_memind_dst_reg` | same | same |
| `emit_alu_memind_dst_imm` | same | same |

### Disabled-by-INSTRS-omission (no emitter at all today)

| Op family | Per docs/fuzz_coverage.md | Actual reason |
|---|---|---|
| TAS, CAS, CAS2 | "atomic semantics need care" | Decode partially done; semantics-fragile |
| MOVES | "user-mode body, would trap vec-8" | Real (program runs in user mode) |
| MOVEC, MOVE USP | "needs supervisor preamble" | Real |
| RTE, RTR, STOP | Exception/halt frames | Real for most; STOP could in principle be tested |
| CHK / CHK2 | "needs known-bounds harness" | Real but achievable |
| TRAP, TRAPV, ILLEGAL, A-line, F-line | "exception frame round-trip" | Real |
| ALL FPU ops (FADD/FMUL/FDIV/FSQRT/FBcc/FMOVE/FSAVE/FRESTORE) | not mentioned | Major hole — FPU compute path is decoded and tested directionally but **never fuzzed** |
| MMU ops (PMOVE, PFLUSH, PTEST) | "phase-3 MMU walker" | MMU stub+walker+ATC landed; could be fuzzed via supervisor preamble |

### Architecturally-narrow ranges

| Constraint | Current | Recommendation |
|---|---|---|
| Body instruction count | 40 (Makefile), no spread | Mix 5/40/200/1000 across seeds; long programs surface ROB-fill + commit-stall interactions |
| Backward-branch loop iters (`HOP_LIMIT`) | 3 | Mix 3 / 10 / 100 / 1000 (with safe upper cap) |
| DBcc counter | randint(0,3) | Mix 0..3 / 10..15 / 100..200 |
| MOVEM list size | 4..6 | Already widened to 6; consider 1..15 (full range) |
| Initial register state | All deterministic (LEA/MOVEQ #0) | Randomize D0..D5 and A4..A6 in preamble (keep A0..A3=pools, A7=stack, D6=hop, D7=scratch) |
| Initial RAM state | 0xFF everywhere | Optionally seed the safe data pools with random bytes per seed (must mirror in Musashi `bus_->mem`) |
| Branch probability | 0.08 | Mix; high branch density stresses BPU/RAS |
| Address registers reserved (A4–A6 LEA-only) | Documented | Could lift via "valid-base regs" tracking |
| D6/D7 reserved | Documented | Required for hop counter / divisor seed |

### Cycle / timeout defaults

| Knob | Default | Risk |
|---|---|---|
| `--instr` (program length) | 40 | Bug class "thousands of cycles before manifestation" never seen |
| `--timeout` (RTL cycles) | 200000 | Generous enough; real concern is short programs not long ones |
| `--musashi-max` | 200000 | Same |
| `--post-sentinel-drain` | 1024 | Adequate per BUG_dcache_writeback_stale_bytes.md |
| `make fuzz N` | 500 default, **200 gate** | 200 too small; 10000+ for nightly recommended |

## Comparison oracle — silent-swallow paths

`tools/fuzz/fuzz.py:diff_state()` and `_classify_mem_diffs()` define
which mismatches make it through.

### Always-ignored keys
```
IGNORE_KEYS = {"cycles", "committed", "committed_macros",
               "last_pc", "pc", "a7"}
```
- `cycles`, `committed`, `committed_macros`: legitimate — different
  microarchitectures.
- `last_pc`, `pc`: documented as "RTL reports last committed PC;
  Musashi reports the halt-loop PC". **However**: any bug that lands
  PC at the wrong sentinel (e.g. taken-vs-not-taken branch around the
  sentinel store) is invisible. Stop-PC parity tests
  (`make musashi-parity`) cover this elsewhere, but not in `make fuzz`.
- `a7`: stack pointer at the end of the program is ignored. Any spurious
  push/pop drift from an unbalanced LINK/UNLK or a bad MOVEM A7
  bookkeeping bug exits the test silently. (Note: A7's *contents*
  through stores are still compared via the mem-write log.)
- `mem[0xFFFF0000..0xFFFF000F]`: the 16-byte sentinel window. The
  comment says "the PASS sentinel store"; in practice the sentinel is 4
  bytes, so the extra 12-byte exclusion can hide spurious writes near
  the sentinel.

### Write-log classification

`auto` policy (default for `make fuzz`):
- Pure write-log-only diffs (regs + CCR match, only mem[] differs) →
  `WRITELOG`, not `MISMATCH`. Counted in repro_seeds.txt but **does
  not fail the run**.
- Mixed (regs+mem differ) → `MISMATCH`, run fails.
- Reg-only → `MISMATCH`.

Sub-classifications inside WRITELOG:
- `write-log/stale-byte-line` — all-0xFF presence-only diffs around a
  cache line. **Documented as "known artifact"**. Real cause may or may
  not be the cache stale-byte bug; either way it's silently triaged.
- `write-log/presence-only` — one side records the byte, other doesn't.
- `write-log/mixed` — values differ.

### Sentinel-handling subtleties

- Musashi sentinel detection in `cb_write32()` requires
  `last_sentinel_value() == 0xC0FFEE00`. A program that writes any
  non-magic value at 0xFFFF0000 is reported as "Musashi failed"
  (returncode 1) and classified as TIMEOUT in fuzz.py. Fine.
- RTL sentinel: tb_top.cpp marks `test_pass` when the AXI write is
  exactly `0xC0FFEE00`. Any other value → test_fail. Fuzz.py treats
  RTL non-zero rc as "not a TIMEOUT, fall through and try to diff
  state" — so a FAIL-on-RTL with PASS-on-Musashi will cleanly diff (good).
- **However**: the RTL fail path leaves `test_pass=false` but the dump
  still emits `pass=0` while Musashi emits `pass=1` — and `pass` is NOT
  in IGNORE_KEYS, so it should diff. Verify this in Phase B.

### x-init `fast` interacts with sim-vs-Musashi parity

Per `BUG_dcache_writeback_stale_bytes.md`, Verilator's `--x-initial fast`
puts 0xFF in undefined regs/mem-cells. The MemModel is **deliberately**
0xFF-initialized to match Musashi (m68k_ref.cpp:`return 0xFF`). But the
D-cache implements full-line writeback, so dirty evictions emit 0xFF
strobes for untouched bytes — these don't appear in Musashi's per-write
log. The current "fix" is to classify these as
`write-log/stale-byte-line` and ignore. **Real fix options listed in
the BUG md were never landed.**

### Comparison points and timing

- RTL: dump at `dump_final_state()` after `post_sentinel_drain` cycles
  + a `dbg_dcache_flush_req` pulse + 8-cycle B-channel drain. So the
  RTL dump reflects post-flush memory + RAT-committed regs.
- Musashi: dump at `run_until_sentinel_or_pc()` exit point — Musashi's
  in-loop snapshot at the moment the sentinel write completes. This is
  pre-`bra .` halt — Musashi never executes the halt loop at all.
- Snapshot misalignment: RTL dumps at an arbitrary point ≥ sentinel +
  drain; Musashi dumps at sentinel store completion. PC/A7 differ for
  exactly this reason — they have to be in IGNORE_KEYS. But `mem[]` is
  also affected: if RTL executes anything between sentinel and dump
  (it shouldn't past the halt loop, but consider mid-flight uops with
  wrong commit ordering), those writes appear on the RTL side only.

## Recommendations (priority-ordered)

### P0 — definitely lift (low risk, high coverage gain)

1. **Crank N to 10000 for nightly** (keep 200 as gate). Current cadence
   only ever runs 200 seeds against a 1000+ instruction state-space —
   trivially under-sampled.

2. **Randomize initial register state** for D0..D5 and A4..A6 in the
   preamble. Mirror in `MusashiRef::reset()`. Removes the
   "always-deterministic" floor that hides reset-value-dependent bugs.

3. **Add per-seed program-length spread**: distribute body N over
   {10, 40, 200, 1000} per seed. The 1000-instr seeds will catch
   ROB/RAT/store-buffer-fill interactions invisible at N=40.

4. **Remove the obsolete `weight=0` shifts and MOVE.B disp emitters**
   (`emit_shift_bw_imm`, `emit_moveb_mem_src_disp`). Comment says
   "until family passes targeted slices" — they pass now. Set weight=1.

5. **Tighten the sentinel-region exclusion to 4 bytes** (0xFFFF0000..
   0xFFFF0003), not 16. Surface stray writes adjacent to the sentinel.

### P1 — opt-in widening (manual flag, not on by default)

6. **Re-enable ALU memind emitters at weight 1** under a `--memind-heavy`
   flag, with raised --timeout. The "8+ µops per crack" concern is
   real but addressable by raising the timeout, not by gating coverage.

7. **Widen MOVEM list size to 1..15** (full architectural range) and
   widen DBcc counter / hop limit to mix 3 / 30 / 300.

8. **Reconsider IGNORE_KEYS for `a7`**: at minimum, instrument it. If
   the comment "stack pointer init differs; we haven't wired Musashi's
   SSP setup to match" is no longer true (Musashi reset matches RTL
   today per `m68k_ref.cpp:reset()`), drop the a7 ignore entirely.

9. **Switch default `write_log_policy` to `strict`** — or at least make
   the WRITELOG count contribute to a documented noise-floor, not be
   silently dropped. This forces the dcache stale-byte bug to surface
   so it gets a real fix.

### P2 — strategic / future (need infrastructure)

10. **Supervisor / exception fuzzing**: extend gen_program.py to emit a
    supervisor preamble (set vector base, install a handler that just
    RTEs back to the next instruction), then enable TRAP, ILLEGAL,
    CHK, TRAPV, A/F-line.

11. **FPU fuzzing**: gen_fpu_program.py with FADD/FMUL/FDIV/FSQRT/FMOVE
    /FBcc emitters. Compare FP regs + FPCR/FPSR/FPIAR.

12. **MMU fuzzing**: with supervisor preamble, exercise PMOVE / PFLUSH
    / PTEST against random page-table layouts.

13. **Initial-RAM randomization**: per-seed seed both the RTL and
    Musashi safe data pools with the same random byte pattern. Catches
    "wrong byte read" bugs that 0xFF-everywhere can't expose.

## Stop point

Phase A complete. Awaiting user direction on which P0/P1 items to
prioritize for Phase B (the high-N stress run + new-fail catalog).
