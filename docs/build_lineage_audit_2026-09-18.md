# Build lineage audit — 2026-09-18

The last programmed artifact is the 100 MHz diagnostic
`build/vivado100_fetch_capture/fpga_top.bit`, build ID efb2e55d.
This audit reads the saved artifact provenance, not a fresh live-board identity.

## CPU

- Build source: reserved core worktree `codex-continuation/agent-01`, HEAD
  df89712ad8482c1140013b3ac131d165975ece26.
- Parent: Claude's `investigate/ifetch-desync`, 64862487dc0e16be445f361b74a7fb50fa6b7508.
  No commits from that branch are missing from the build HEAD.
- Main checkout (`master`) is f6b17b87861fe89cd7fbbd2123d9b7ab89b10864;
  committed ancestry comparison is 0 commits unique to master / 111 unique
  to the diagnostic HEAD. The main checkout also has unrelated uncommitted work.
- Generated CPU RTL SHA256 matches the build-time provenance exactly:
  `4cffdc978c4f04adb7bc6a6ff53cdf43d2c3fc4b6c35429aee902614020a369f`.
  Bitstream SHA256 remains
  `9caa0a6041918aee2f23634d2271ecb6275aa29eeb397938e0f745d3915f4b1a`.

## SoC: ahead of main is not sufficient

Build HEAD efb2e55d is 19 commits ahead of main ca298a16, with no main-only
commits. It includes closure-200mhz-soc c1293172. Nevertheless, it lacks nine
commits on `audit/soc-race-matrix` f66fc997; `git cherry` marks all nine as
non-equivalent patches, not merely differently named cherry-picks:

| Commit | Missing work |
|---|---|
| baedda52 | Crossbar flush in RS_DRAIN_R: response and S3 quarantine |
| 999e45e7 | Crossbar abandonment in WS_DRAIN_W / WS_SEND_BLOCAL |
| df222f47 | Ethernet phantom transmit completion across core reset |
| 700e9616 | ROM-overlay disarm ledger timing and leakage |
| 77608f63 | Fetch guard: outstanding-read tracking and reset AR acceptance |
| be2b14e9 | Crossbar S1 exclusion during 68040 RESET |
| a027c3fa | Boot-master reset visibility at crossbar/adapter |
| 28a5f06c | L2 array-collision hostile/write-first test modes and rationale |
| f66fc997 | L2 randomized stress gate, stronger sizing, guard documentation |

The final two are verification/model additions rather than evidence of an
omitted normal-hardware L2 data-path fix. Other entries change synthesized RTL.
Their absence is relevant but does not by itself explain the captured A198
wrong-fetch word.

Conversely, our HEAD has five commits not present on the audit branch:
cfee29a6 (peripheral-reset strobe handling), c1293172 (L2 reset debt drain),
d02c3f11 (SONIC RX timing), 71ca5ad2 (presented fetch response stability), and
efb2e55d (DDR padding control). A branch switch would discard those.

`git merge-tree --write-tree HEAD audit/soc-race-matrix` found conflicts only
in `rtl/soc/ifetch_window_guard.v` and `tb/tb_ifetch_window_guard.cpp`.
No worktree merge or RTL changes were performed. Integration must preserve
both outstanding-request tracking from the audit and the current protection
for presented/stalled responses, and run both sets of tests. Do not copy the
audit branch over this checkout wholesale.

The user requested testing the CPU reset-response absorber, not removing it.
No absorber/reset RTL has been changed during this audit. Review integration
and validate the targeted tests before making another 100 MHz diagnostic image.

## User-requested HEAD-tip integration

The user subsequently requested bringing both build trees to their current
tips and queuing a 100 MHz build. CPU HEAD is now 9d3ab41e, adding only the
previously uncommitted simulation-visibility annotations; all four checked
CPU investigation/audit tips remain ancestors, and `src/main` is clean.

The SoC audit branch has been merged into the diagnostic worktree. Conflict
resolution keeps outstanding-read accounting, reset acceptance qualification,
and the current presented-response checks; both test sets are retained.
Validation on the merged sources:

- Fetch guard: 52 passed, 0 failed.
- Crossbar: 522 passed, 0 failed.
- Ethernet toggle receiver: 7 passed, 0 failed.
- L2 stress: five seeds, 100,000 randomized operations per seed, passed.
- L2 hostile-collision and write-first modes: directed and five-seed stress
  checks passed under both models; no collided array read consumed.
- CPU fast gate: 372 passed, 0 failed, 2 ignored.

The queued build script checks committed RTL against HEAD, regenerates CPU RTL
with the SoC build ID, and runs the repository's pre-implementation gate before
invoking 100 MHz implementation under the Vivado lock. No bitstream from this
integration is yet claimed built or deployed.
