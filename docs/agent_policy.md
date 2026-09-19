# Agent Policy — shared invariants

> **Every agent reads this file.** It is the shared policy layer — invariants
> that apply regardless of which track (core / peripheral / platform / testing)
> your task sits in. Your track brief (`docs/tracks/<track>.md`) has the
> scope-specific detail. The full `CLAUDE.md` is the PM's map; you do NOT need
> to read it unless your task explicitly crosses tracks (it shouldn't).

## Your role

You are a focused delegate. The main session (PM) keeps the big picture and
manages cross-track coordination. Your job is a surgical change inside one
track, validated by sim, committed on your own worktree branch.

If your task needs to touch files outside your track's scope, **stop and
report** — don't silently expand scope. The PM will coordinate.

## Worktree requirement

Every implementation agent works from a dedicated git worktree and branch,
not the PM's main checkout. The expected location is:

```bash
/home/qwertyoruiop/m68k-ooo-worktrees/<track-or-task-name>
```

Create the branch/worktree before editing, for example:

```bash
git worktree add -b agent/<track-or-task-name> \
  /home/qwertyoruiop/m68k-ooo-worktrees/<track-or-task-name> main
```

Run Verilator and unit tests inside that worktree. Its local `build/`
directory is disposable and cannot corrupt another agent's PCH or generated
objects. Do not build, edit, stage, or commit from `/home/qwertyoruiop/m68k-ooo`
unless the PM explicitly assigns the main checkout to you.

Tool-created `.claude/worktrees/*` directories are local runtime state, not
the project coordination boundary. If you inherit one, treat it as scratch
until the PM assigns a real `/home/qwertyoruiop/m68k-ooo-worktrees/*`
worktree branch.

### The `cpu` submodule: check it initialised, always

**Fixed 2026-08-20** — `/home/qwertyoruiop/m68k-ooo` `main` was re-pointed at
the SoC's `cpu/` line, so `git submodule update --init` works again. The
two repos had diverged (the discarded side is kept on `main-before-soc-sync`
and the `agent/*` branches). Keep reading anyway: the divergence can recur
the moment someone commits to one repo and not the other, and the failure
mode below is quiet enough to cost a whole run.

Historically `git submodule update --init` **failed** in agent worktrees, and
the failure is quiet in a way that costs you a whole run: `rtl/soc/axi_narrow_to_wide.v`
is a symlink into `cpu/`, so a dangling `cpu/` silently prevents ~5 gates
from building. A gate that did not build is not a gate that passed.

Cause when it recurs: `.gitmodules` points `submodule.cpu.url` at
`/home/qwertyoruiop/m68k-ooo`, and the SoC repo's pinned gitlink is **not
reachable from there** whenever the two have drifted — the SoC-side commits
then live only in the parent checkout's own `cpu/` clone.

Recovery, and confirm the symlink resolves before trusting any result:

```bash
git -C cpu fetch /home/qwertyoruiop/macqd700-soc/cpu '+refs/*:refs/remotes/parent/*'
git -C cpu checkout <gitlink-sha>      # from `git ls-tree HEAD cpu` in the SoC repo
readlink -f rtl/soc/axi_narrow_to_wide.v && test -e rtl/soc/axi_narrow_to_wide.v \
  && echo "symlink OK" || echo "STILL DANGLING - stop, do not run gates"
```

Do **not** try to fix a recurrence yourself by pushing `cpu/` to
`/home/qwertyoruiop/m68k-ooo`. Which line wins is the PM's call, and that
repo has `main` checked out, so a push would be refused anyway — the
2026-08-20 sync was done as a `fetch` + `reset --hard` from inside it,
after checking its tree was clean and the superseded commits survived on
other refs.

## Integrating your work back: patches, never file copies

Report your result as a **diff or `patch -p1` patch**, and say where you
saved it. Do not tell the PM to copy files out of your worktree.

Your worktree forked from the HEAD that existed when you were spawned, and
you may run for an hour. Anything that lands on `main` meanwhile is absent
from your copy, so copying a **shared** file wholesale (`Makefile`,
top-level `.vh`, shared tb harnesses) silently reverts it. This has already
happened once: a `Makefile` copy reverted 58 lines of a scanout test gate,
including the negative control that proved the checker was sensitive at all.

The tell is a **symmetric** insertion/deletion count (`58 insertions(+), 58
deletions(-)`) on a file your change should only have added to.

So: avoid editing shared files at all where you can. If you must, say so
prominently in your report so the PM merges rather than copies.

## Sim-first policy

- **Default = Verilator sim.** `make sim`, `make test`, `make fuzz`,
  `make tb-<module>`. Iterate in seconds.
- **No `make synth` / `make impl`.** Vivado runs are phase-boundary events
  the PM schedules. If your task prompt says "no synth", that's binding.
  The Vivado mutex (`/var/tmp/m68k-ooo-vivado.lock`) is enforced — don't
  block-wait on it.
- **Prefer unit tbs.** If a `tb-<module>` exists, extend it. If not and your
  change is non-trivial, add one.

## Musashi is the golden model

For any ALU / decode / ISA semantics question:

```bash
python3 tools/fuzz/fuzz.py --replay <seed>
```

answers it in seconds against Musashi v4.60 in 68040 mode. Don't guess
semantics from the PRM when Musashi disagrees.

## Widening-tbs-as-features-land

Every feature you land MUST include:

1. **Directed scenario** in the closest `tb-<module>` for the corner you
   *almost got wrong* while implementing — not just happy path. A feature
   that lands with green tests but no new corner coverage is a feature
   that will regress silently.
2. **Fuzz widening** in `tools/fuzz/gen_program.py` if the feature is
   ISA-observable. Widen any artificially narrow bounds that would have
   hidden your bug.
3. **Both commits in the same PR** as the feature.

Motivating precedent: MOVEM 5+-reg-list + trailing bit-op hang, caught
only after the fuzz-corpus agent widened the register-list bound. That
bound was small because MOVEM landed before the generator learned to
stress it.

## Build commands

Canonical flow (from your assigned worktree root):

```bash
pwd                           # must be /home/qwertyoruiop/m68k-ooo-worktrees/<name>
git branch --show-current     # must be your agent/* branch
MAKEFLAGS="-j1" make sim      # single-threaded to avoid Verilator PCH race
make test                      # full directed suite
make fuzz N=200                # golden-ref vs Musashi
make lint                      # 0 warnings expected
make tb-<module>               # unit tb
```

Use a clean `/tmp` rsync build only when you need a second independent
sanity-check of the same committed tree:

```bash
rm -rf /tmp/m68k-ooo-build
mkdir -p /tmp/m68k-ooo-build
rsync -a --delete --exclude='.git' --exclude='build' --exclude='.claude/worktrees' \
      ./ /tmp/m68k-ooo-build/
cd /tmp/m68k-ooo-build
MAKEFLAGS="-j1" make sim
```

**`--delete` on rsync is mandatory** when using the `/tmp` fallback. Without
it, deleted files remain stale in the copied tree and silent-skip during
tests. Missing .s files won't error — they just don't run.

**`MAKEFLAGS="-j1"` for `make sim`.** Parallel Verilator builds race on
the PCH (`Vmac_top__pch.h.fast`). Single-threaded builds are reliable.

## Test-count verification discipline

**Don't trust the `make test` summary alone.** Always verify specific
test names appear in the PASS list:

```bash
make test 2>&1 | grep -E "my_new_test_1|my_new_test_2|my_new_test_3"
```

If a test's .bin failed to compile, `make test` silently skips it and the
summary still reads green. Grep by name.

## Current baseline (2026-05-07 session)

- **Directed suite**: 628 PASS / 0 DEFER / 0 FAIL / 0 SKIP on `main`
  HEAD `73f8e865`.  Any FAIL, skipped new test, or DEFER increase is
  a regression unless explicitly documented.
- **Fuzz (per-PR)**: `make fuzz N=200` — 200/200 vs Musashi.
- **Fuzz-deep (pre-impl)**: `make fuzz-deep` — exhaustive ~5-min sweep,
  mandatory before any `make impl` / Vivado bitstream run.  Contract +
  failure runbook in `docs/fuzz_deep_policy.md`.  The
  `tb/fuzz_fails/known_failures.txt` manifest excludes only seeds with
  an OPEN BUG_md.
- **Lint**: 0 warnings expected.
- **Post-route timing**: at phase boundaries only, gated on
  `fuzz-deep` PASS first.

After your landing: must remain ≥ baseline on every axis or you document
the regression + file a follow-up task.

## Toolchain on this host

```
m68k-linux-gnu-as -m68040
m68k-linux-gnu-ld -Ttext 0x40800000
m68k-linux-gnu-objcopy -O binary
```

Verilator 5.x, Vivado 2025.2 (binary at `/tools/Vivado/2025.2/Vivado/bin/vivado`), Python 3 for fuzz/tools.

## Coding conventions

- **Verilog-2005 only.** No SystemVerilog (`logic`, `always_ff`, packed
  structs). No `{concat}[bit]` bit-selects (XST rejects them — use an
  intermediate wire).
- Synchronous, active-high reset.
- `always @(posedge clk)` for FFs. No latches. No `initial` blocks in
  synthesisable RTL.
- 4-space indent, no tabs. Parameters `ALL_CAPS`, signals `snake_case`,
  modules `snake_case`.
- No magic numbers in RTL — use `uop_pkg.v` defines or module
  localparams.
- Every module gets a header comment: purpose, key interfaces, latency.
- Keep modules ≤ 300 lines; split if larger.

## Reset / debug-CSR (cross-cutting)

The unified-reset story (Phase 1-4 landed) routes one debug-CSR bit
family through `debug_stop_manager`:

- **bit 4 (RESET_REQ)** — assert reset to the CPU + peripheral
  fabric.  Self-clearing.
- **bit 5 (HALT_AFTER_RESET)** — when set with bit 4, the CPU stays
  halted on reset release until an explicit `reset release` command.

JTAG REPL canonical commands are `reset` / `reset hold` /
`reset release` / `reset-and-halt-after <N>` (see `tools/jtag_repl.tcl`
docstring + `docs/reset_story.md`).  Legacy `full-reset`,
`full-reset-and-halt`, `reset-halt-after` are kept for one release as
deprecated aliases.

## Reserved architectural state (cross-cutting)

- `ARCH_INT_REGS = 19` (D0–D7, A0–A7, TMP0, TMP1, TMP2).
- **Phys 16 = `PHYS_ZERO_TAG` = `REG_TMP0`** — hidden always-zero scratch.
  Never freed by RAT free-list. Used by decode-time cracks that need a
  constant-zero source.
- Phys 17 = `REG_TMP1`, phys 18 = `REG_TMP2` — hidden writable scratch
  for multi-µop cracks (CHK2/CMP2 `lo`/`hi` stash, MOVEP byte-shuffle,
  BFINS field-stage).
- PRF[0..18] initialised to 0 at reset; free list contains phys 19..47
  (29 speculative regs). Agents bumping `ARCH_INT_REGS` must update
  `rat.v` masks AND `tb/tb_rat.cpp` baselines.

## Shell + git discipline

Before any `git merge`, `git commit`, or `git reset`:

```bash
pwd                              # confirm you're in main repo
git branch --show-current        # confirm target branch
```

Agent worktrees live under `/home/qwertyoruiop/m68k-ooo-worktrees/<name>/`.
Shell cwd can drift there between commands. A `git merge` run from a worktree
targets that worktree's branch, not main. The PM caught this once the hard
way; don't repeat.

## Commit messages

Short (≤72-char title), specific, conventional.

```
<scope>: <imperative summary>

<body — what + why, not how>

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
```

Co-author footer only when instructed. No emoji unless explicitly
requested. No marketing prose.

## Docs

Only create `*.md` files when the task explicitly asks for one. Don't
auto-generate planning/summary docs — the PM manages those. Task briefs
and architectural decisions go into the track docs
(`docs/tracks/<track>.md`) or a dedicated spec doc only on request.

## Reporting back

At task end, report ≤300 words:

- What changed (files + rough LoC).
- Gate results (test counts by name, fuzz PASS count, lint).
- Honest caveats — "should work" isn't a number.
- Follow-up tasks worth filing.

Commit on your own worktree branch. The PM merges.
