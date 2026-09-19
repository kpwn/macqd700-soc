# Testing track

> Read `docs/agent_policy.md` first. This brief adds testing-track scope.

## Scope

Test infrastructure: Verilator harness, directed tests, unit tbs,
Musashi fuzz, benchmarks, regression gating. You own the tools that
validate RTL — not the RTL itself.

## Files you own

```
tb/models/{mem_model.{cpp,h}, mac_rom.{cpp,h}, m68k_ref.*}
tb/tests/asm/*.s                      (directed ISA tests)
tb/tests/deferred.txt                 (failing-but-tracked manifest)
tb/fuzz_fails/*                       (captured BUG_*.md + repros)
tb/tb_*.cpp                           (unit tbs + harnesses)
tools/fuzz/{fuzz.py, gen_program.py}  (fuzz + Musashi co-sim)
tools/{mame_trace_normalize.py, rom_trace_diff.py}
tools/isa_coverage.py, tools/decode_check.py
Makefile                              (test / tb-* / fuzz targets)
docs/{bench_baseline.md, fuzz_coverage.md, mmu_test_plan.md,
      rom_boot_bringup.md, mame_integration.md}
```

**tb_top.cpp is a coordination hotspot** — DO NOT edit without PM
coordination. It's the full-mac_top Verilator harness and multiple
tracks read it. If you need a change there, stop + report.

Do NOT touch `rtl/*` unless your task is test-only (e.g. fixing a
clearly-broken .s assembler syntax). RTL changes belong in other tracks.

## Current test state (2026-04-18 session)

- **Directed**: 188 PASS / 6 DEFER / 0 FAIL / 0 SKIP from a clean
  `/tmp` run at `fea9d34`.
- **Fuzz**: `N=50` at `fea9d34` had 47 PASS / 0 MISMATCH /
  3 TIMEOUT. The current fuzz harness now reports pure write-log-only
  cases as `WRITELOG` and writes the deterministic replay seed list to
  `build/fuzz/repro_seeds.txt`; use `--write-log-policy strict` when you
  explicitly want those to fail the run.
- **Lint**: 0 warnings expected.
- **Unit tbs**: tb-{via1, via2, scsi, scc, asc, glue, irq-agg, rtc,
  video, vram, dcache, icache, lsu, mmu, mmu-walker, rat, rob, alu,
  mul-div, bpu, axi-xbar, async-fifo, axi-async-bridge, dma-ctrl,
  if-stage, peripheral-bus, mac-top-smc, sd-ctrl, sd-boot,
  ddr-model, debug, host, ras, rom-boot}

Missing / pending unit tbs (task #72): tb-iq-int, tb-iq-mem, tb-commit,
tb-exception, tb-predecode.

## Canonical test flow

```bash
rm -rf /tmp/m68k-ooo-build
mkdir -p /tmp/m68k-ooo-build
rsync -a --delete --exclude='.git' --exclude='build' \
      --exclude='.claude/worktrees' ./ /tmp/m68k-ooo-build/
cd /tmp/m68k-ooo-build
MAKEFLAGS="-j1" make sim              # single-threaded avoids PCH race
make test                              # all directed .s tests
make fuzz N=200                        # Musashi co-sim
make fuzz-replay-seeds SEEDS_FILE=build/fuzz/repro_seeds.txt
make lint                              # 0 warnings
make tb-<module>                       # unit tb
make test TEST=<name>                  # single directed test
make test WAVES=1 TEST=<name>          # generates .fst
```

Additional decode-frontier checks:

```bash
make test TEST_GROUP=adversarial-decode
make isa-width-audit
```

`adversarial-decode` clusters ROM-frontier full-format indexed and
memory-indirect forms so they can be run before long ROM boots.  Tests
that expose current decode/addressing holes stay in
`tb/tests/deferred.txt` until the owning RTL track fixes them.

`isa-width-audit` is a lightweight directed-test inventory.  It scans
`tb/tests/asm` for explicit `.b/.w/.l` mnemonic coverage and filename
sibling clusters, then prints missing byte/word/long variants without
failing the build.

## Test-count verification discipline (MUST READ)

**`make test` silently skips tests whose .bin didn't compile.** The
`summary: PASS=X DEFER=Y FAIL=0` line doesn't count skipped tests. An
empty run and a fully-passing run look identical in summary.

**Always verify by test name:**

```bash
make test 2>&1 | grep -E "my_new_test_1|my_new_test_2"
```

When you add N new tests, confirm N new `[PASS ]` lines appear with
those exact names. If fewer, the .s didn't assemble or the harness
skipped it.

When resetting state (new checkout, different build tree), also:

```bash
cd /tmp/m68k-ooo-build && ls tb/tests/asm/*.s | wc -l
# confirm matches main repo
ls /tmp/m68k-ooo-build/build/tests/*.bin | wc -l
# confirm .bin count ≈ .s count − deferred
```

## Fuzz workflow

```bash
python3 tools/fuzz/fuzz.py --n 200 \
    --sim build/sim/Vmac_top \
    --musashi tb/models/libmusashi_ref.a \
    --work build/fuzz \
    --as m68k-linux-gnu-as --ld m68k-linux-gnu-ld --objcopy m68k-linux-gnu-objcopy \
    --save-fails tb/fuzz_fails \
    --save-seed-file build/fuzz/repro_seeds.txt
python3 tools/fuzz/fuzz.py --seed-file build/fuzz/repro_seeds.txt \
    --sim build/sim/Vmac_top \
    --musashi tb/models/libmusashi_ref.a \
    --work build/fuzz \
    --as m68k-linux-gnu-as --ld m68k-linux-gnu-ld --objcopy m68k-linux-gnu-objcopy
```

- **0 MISMATCH is the critical architectural invariant.** TIMEOUTs are
  expected (Musashi budget exhausts on loop-heavy programs); they're
  not regressions. Pure write-log-only cases are summarized separately
  and carried in the replay seed list unless `--write-log-policy strict`
  is set.
- **Widen `gen_program.py` in the same PR as any ISA addition** (per
  `docs/agent_policy.md`). New instruction classes get a new
  `emit_*(r, ctx)` function + an entry in `INSTRS` list.
- When a fuzz run surfaces a new MISMATCH, capture as
  `tb/fuzz_fails/BUG_<name>.md` with the repro `.s` + `.bin` + a
  one-paragraph root-cause hypothesis. Don't try to fix RTL — file as
  a task for the appropriate track.

## Directed tests

- Assembly: `m68k-linux-gnu-as -m68040`, link at `0x40800000`.
- **PASS sentinel**: `move.l #0xC0FFEE00, 0xFFFF0000` then `bra .`.
  The tb watches byte `0xFFFF0000` for that value.
- **FAIL sentinel**: any other 32-bit value written to `0xFFFF0000`.
- Tests that need supervisor mode must install exception vectors
  BEFORE raising an exception.
- Deferred tests go in `tb/tests/deferred.txt` with a comment
  explaining why + blocker task number.

## Benchmarks

```bash
tb/tests/asm/bench_*.s
```

Each is a fixed workload. Run + record cycle count + committed
instruction count to derive IPC:

```
IPC = committed / cycles
```

`docs/bench_baseline.md` pins the authoritative numbers. **Refresh it
after any µarch / cache / memory-system change** — not doing so leaves
a stale baseline that misleads future ipc-roadmap iteration.

## MAME trace replay

```bash
# capture MAME reference (run MAME locally — not this agent)
mame -machine macqd700 -debug -log trace.log,0,noloop

# normalize
python3 tools/mame_trace_normalize.py trace.log > /tmp/mame.norm

# diff vs our ROM boot trace
python3 tools/rom_trace_diff.py \
    /tmp/m68k-ooo-build/build/sim/rom_boot_trace.log \
    /tmp/mame.norm
```

The diff reports first-divergence PC with ±20 lines context. This is
the primary tool for Q700 ROM boot progression.

## Common pitfalls

- **Stale .bin files.** `rsync --delete` forces clean sync. Without
  --delete, removed .s files' .bin remain and confuse the test set.
- **Verilator PCH race.** `MAKEFLAGS="-j1"` for sim builds.
- **Fuzz seed reproducibility.** Same seed + same gen_program version
  = same program. When gen_program changes, old seeds generate new
  programs. Re-capture BUG_*.md repros when widening.
- **Deferred manifest pollution.** New defers get committed with an
  explanation + blocker task reference. Don't defer to silence a
  failing test — file the blocker.
- **Test naming collisions.** `make test TEST=<name>` substring-matches;
  avoid test names that are prefixes of other tests.

## When to write a new unit tb

- You're modifying a module and the existing tb-<module> doesn't
  cover your change.
- You're adding a module and there's no unit tb yet.
- A fuzz-surfaced bug has a hard-to-reproduce integration path;
  isolate into a unit tb for regression tracking.

Keep unit tbs ≤ 50 lines per scenario, ≥8 scenarios per tb target.

## References

- `docs/bench_baseline.md` — cycle scoreboard.
- `docs/fuzz_coverage.md` — what gen_program emits / doesn't.
- `docs/mmu_test_plan.md` — MMU stress categories + 47-scenario plan.
- `docs/mame_integration.md` — MAME trace replay spec.
- `docs/rom_boot_bringup.md` — Q700 ROM bring-up log.
