# tools/pm/ — PM workflow helpers

Small shell helpers that collapse repetitive PM Bash/Grep queries into
single commands. Target: keep per-tick session state reconstruction
under a single tool call.

All scripts:
- Accept `--help` (one-line usage + description).
- Exit non-zero on failure with a one-line error message.
- Use `/tmp/m68k-pm-cache/` for caching (not repo, never committed).
- Never touch `rtl/*.v`, `tb/*.{cpp,v}`, or synth artifacts.

## `status`

One-paste PM dashboard (<30 lines). Shows:

- Current branch + HEAD SHA + subject.
- `git status` counts (staged / modified / untracked / stashes / worktrees).
- Last `make test` summary line (cached 5 min; use `--refresh` to force).
- Last `make fuzz` PASS / MISMATCH / TIMEOUT counts (cached 5 min).
- Vivado lock state — `HELD` (with holder cmd/PID) or `free`.
- Most recent bitstream under `build/vivado/*.bit` with size + age.
- In-flight agents — `.output` files touched in last 2 min under
  `/tmp/claude-1000/`.
- Peak IPC from `docs/bench_baseline.md` headline table.

Flags:
- `--refresh` — ignore cache, re-run `make test` / `make fuzz`.
- `--no-test` / `--no-fuzz` — skip that cache refresh entirely (fast).

Also available via `make pm-status` from repo root.

## `brief <task-id> <track> <subject>`

Emits a standardized sub-agent brief template to stdout. Prepends the
`docs/agent_policy.md` + `docs/tracks/<track>.md` read directive, bakes
in the "files currently being touched on main" concurrency warning, and
leaves `<SCOPE>` / `<GATES>` / `<REPORT>` placeholders for the PM.

```
tools/pm/brief 147 core "MOVES supervisor addressing"
```

Tracks: `core` | `peripheral` | `platform` | `testing`.

## `verify-merge <branch>`

Pre-merge safety checks run from the main repo:

- Confirms cwd is `/home/qwertyoruiop/m68k-ooo` on branch `main`.
- Shows the commit log of the branch vs its fork-point.
- Lists files touched by the branch.
- Warns on overlap with files touched in the last 3 main commits.
- Exits 0 if safe to merge, 1 on overlap / conflict risk.

## `summarize-fuzz-fails [dir]`

Scans `tb/fuzz_fails/BUG_*.md` (or given dir) and prints a columnar
table: `NAME | SYMPTOM | TASK-ID` one line per bug. Useful for deciding
which deferred bug to pick up next.

## `watch-synth <log-path>`

Streams a Vivado synth/impl log through a narrow signal filter:

- URAM / BRAM / LUT resource summaries
- WNS / worst-slack
- `Writing bitstream`
- Phase completion markers
- `ERROR:` / `CRITICAL WARNING` / `Abnormal program termination`

Exits 0 when bitstream is written, 1 on error. Run in a terminal while
`make impl` runs in another.

## Cache

Scripts write under `/tmp/m68k-pm-cache/`. Wipe with `rm -rf
/tmp/m68k-pm-cache/` if you want a fully cold dashboard.
