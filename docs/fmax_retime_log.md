# fmax_retime_log.md — zero-IPC-risk retime campaign

Started from main @ f85adee (tests: defer 4 adversarial-found bugs).
Target: 200 MHz (5.0 ns period), KU5P xcku5p-ffvb676-2-i.

Per `docs/fmax_analysis.md` the P1 + P2 + P4 + P5 + P6 retimes had
already landed on main; remaining "simple" levers documented there
were max_fanout fine-tuning and the F1 (registered PRF read) retime
whose IPC tax requires a CDB→RS forwarding mux.

**2026-04-18 status overlay:** this log is historical through the retime
campaign.  The F1 retime called out by the original campaign later landed as
`fe9f27d` with bench drift within 1%; see `docs/bench_baseline.md`.  Current
low-IPC Fmax planning should treat F1 as done and focus on the still-open
items: sequential DIVS.L/DIVU.L, residual control fanout (`crosses_line`,
flush distribution, and post-F1 measurement of the existing `pd_valid`
pragma), MUL/ALU flag-path split, reset-tree replication, and the Vivado
clock-period override cleanup.  No synth/impl is scheduled during the
mac-logo sim-first phase.

## Pre-campaign baseline

Sim / bench baseline (copied from `docs/bench_baseline.md` @ 06fdbef):

| Bench                | Cycles | IPC   |
|----------------------|--------|-------|
| bench_alu_parallel   | 684    | 0.368 |
| bench_btb_dbra       | 748    | 0.142 |
| bench_btb_loop       | 849    | 0.243 |
| bench_cmp_branch     | 545    | 0.235 |
| bench_dep_chain      | 98     | 0.276 |
| bench_fullpipe       | 889    | 0.353 |
| bench_ind_adds       | 679    | 0.386 |
| bench_mixed_mem      | 390    | 0.369 |
| bench_move_heavy     | 1914   | 0.343 |

Functional: 119 PASS / 8 DEFER / 0 FAIL.  Fuzz: 200/200.

## Landed retimes (agent/fmax-simple → main)

All six are attribute-only (max_fanout pragmas on register output
ports / driven wires).  Zero functional change, zero IPC change.

| Commit | Retime | File | Pragma | Rationale |
|--------|--------|------|--------|-----------|
| d658a41 | R1 flush_en SRC replica | commit.v:228-234 | `max_fanout=64` on output reg | flush_en fans to 9 sinks in m68k_core with ~1280 FFs at the deepest (rob).  Prior P5 put the pragma at the *consumer* (rob.v:143) — that only helps that one sink.  Source-side licenses the register to be replicated under `-keep_equivalent_registers`. |
| 9ad983b | R2 alu.valid_out (cdb0_en) | alu.v:62-68 | `max_fanout=48` | valid_out → cdb0_en → wake snoop on RAT + iq_int + iq_mem + lsu + ccr_rat, ~164 loads per historical §2 high-fanout table. |
| 7279d29 | R3 lsu.cdb_valid (cdb1_en) | lsu.v:77-82 | `max_fanout=48` | Mirror of R2 for the LSU-side CDB. |
| 1659cce | R4 CCR CDB broadcast | alu.v:76-83 | `max_fanout=32` on ccr_dst_tag_out + ccr_has_dst_out | Drives ccr_rat + iq_int CCR-src snoops. |
| 88519c1 | R5 CDB phys-tag replicas | alu.v:76 / lsu.v:83-89 | `max_fanout=48` on phys_dst_out, cdb_phys, cdb_has_dst | Completes the CDB pragma set — these feed per-entry src-tag comparators across RAT + iq_int + iq_mem. |
| c68510c | R6 if_stage.pd_valid | if_stage.v:83-90 | `max_fanout=32` on intermediate wire | pd_valid gates rn_ready, disp_en, iq inserts, ras pop, BPU predict-redirect — 145 loads per §2. |

Alternatives considered:
- **Manual register replication** for flush_en (vs pragma): pragma is
  simpler under `-keep_equivalent_registers`, doesn't diverge reset/
  set code paths.  Manual replica was viable as backup if Vivado
  ignored the pragma.
- **Pipeline the ALU's PRF-read (F1)**: rejected as violating the
  brief's "no bench >1% slowdown" rule — would need CDB→RS forwarding
  mux to avoid a 20%+ regression on bench_dep_chain.  Handed off to a
  separate IPC-impacting ticket.
- **`pd_buf` max_fanout pragma**: rejected.  pd_buf is a 128-bit
  variable-position barrel mux; replicating takes 128 LUTs per
  replica.  `pd_valid` is the cheaper proxy.

## Measurement constraints encountered

### Vivado resource contention

The task-machine has 62 GiB RAM and 8 GiB swap.  Two concurrent Vivado
synths OOM-kill each other — each uses ~14 GiB peak memory during
Translating / Timing Optimization phases, and the SIM_MODEL DDR in
ddr_ctrl.v defaults to BRAM_LOG2_BEATS=19 (524288 RAM256X1D), which
causes Vivado to spin ~30 min in RAM expansion before it even begins
tech mapping.

Worked around by:
1. Waiting for the hw-impl agent to finish before kicking synth.
2. Locally (in `/tmp/fmax-baseline-synth/rtl/sys/ddr_ctrl.v` only,
   not committed) changing BRAM_LOG2_BEATS=19 → 12 for timing-only
   runs.  (hw-impl's branch 0893329 landed the same change
   permanently as default = 12.)
3. The `flock -n /var/tmp/m68k-ooo-vivado.lock` Vivado mutex was
   added to main (commit 13d8044) while this campaign was running.

### Clock period override bug in vivado.tcl

`vivado.tcl:161` has `set_property -dict {PERIOD …} [get_clocks
sysclk200]`, which Vivado 2023.1 rejects because PERIOD is a
read-only property (it can only be set via `create_clock`).  The
`TARGET_FREQ_MHZ=200` env-var path skips this block and runs at the
fpga_top.xdc default (5.0 ns, 200 MHz) — so running the synth with
`TARGET_FREQ_MHZ=200` bypasses the bug.

A permanent fix would replace the set_property with a re-run of
`create_clock -period $target_period_ns` (under `reset_timing` to
drop the XDC version first).  Not in scope for this campaign.

## Synth results

### Attempt 1: pre-retime baseline @ f85adee

Killed by OOM during Timing Optimization at 20+ min elapsed.  No
report produced.  Required the BRAM_LOG2_BEATS=12 workaround.

### Attempt 2: post-retime (R1…R6) @ agent/fmax-simple HEAD, 100 MHz

Ran with BRAM_LOG2_BEATS=12 workaround.  Vivado tcl period override
hit a "Cannot change read-only property PERIOD" error on
`set_property`, aborting before write_checkpoint.  No report.

### Attempt 3: post-retime (R1…R6) @ agent/fmax-simple HEAD, 200 MHz

Ran with `TARGET_FREQ_MHZ=200` env var so the vivado.tcl override
block is skipped and the fpga_top.xdc 5 ns clock stays in effect.

**WNS = -19.892 ns** at 5.000 ns period (Fmax ≈ 40 MHz).

### Critical-path root cause: task #62 32×32 combinational divide

All top-60 VIOLATED paths in the report route through the flattened
DIVS.L/DIVU.L combinational divider.  Top source pin
`u_cpu/prf_reg[19][2]/C` fans into `u_cpu/u_iq_int/divs_quot_full` /
`divul_quot` / `divsl_rem3` nets → `u_cpu/u_alu/result_reg[*]/D`:

- **Source**: prf_reg[19][2]/C (PRF read from phys 19 bit 2)
- **Destination**: u_alu/result_reg[29]/D (ALU result writeback)
- **Data path delay**: 24.737 ns (logic 12.263 ns / route 12.474 ns)
- **Logic levels**: 209 (161 × CARRY8 + mixed LUTs + MUXF7)
- **Net**: divsl_rem3__[1020..1043+] — a fully-unrolled restoring
  divide.  Each CARRY8 chain of 2 stages (adder) × 32+ iterations
  = ~64 CARRY8 levels, all combinational within a single clock
  edge.

This was NOT visible in the historical `docs/fmax_analysis.md`
because that analysis predates MULS/DIVS.L landing (task #62).  The
fmax_analysis document predicted ~-1.18 ns WNS via the ALU-flag
path post-P1+P2+P4+P5+P6 — the +18.7 ns extra regression is
entirely from the new divide.

No amount of fanout pragma tuning or register duplication can fix
a 209-level combinational chain.  The divide must become
**multi-cycle** — either:
1. Re-use the existing 2-cycle `mul_pipe_*` lane in alu.v and add
   a separate sequential divide state (~16-32 cycles for 32-bit
   non-restoring divide).
2. Re-route DIVS.L / DIVU.L through `mul_div.v` which is currently
   a 5-line stub.

Both options violate the brief's "simple, zero-IPC-risk" scope —
every DIVS/DIVU in a benchmark becomes a 16-32-cycle stall.  The
DIVS-pipeline ticket is handed off as a separate retime.

### What the pragma retimes did and didn't do

The 6 max_fanout pragmas are structurally correct and were
accepted by Vivado (grep for `Replicated .* times` in the physopt
log shows iq_int/iss_phys_src_b_reg replicas as expected from the
iq_int.v:94-97 pre-existing pragma).  But they cannot move the
overall WNS while the divide path dominates at -19.89 ns — any
improvement on CDB-fanout paths (target ~-1.18 ns) is invisible
below the divide.

The retimes will realise their expected ~0.3-0.5 ns WNS recovery
ONLY after the divide is pipelined to remove it from the 1-cycle
envelope.  At that point the CDB fanout cleanup should help push
WNS from the post-divide-fix value (est. -1 to -2 ns from other
residual ALU paths) toward zero.

Merged to main as commit b3ad843.

## Current follow-up after later F1 landing

1. **#1 PRIORITY: pipeline DIVS.L/DIVU.L** — blocks ALL further Fmax
   recovery.  Route DIVS/DIVU through `mul_div.v` as a 16-32-cycle
   sequential divider, mirroring the MULS 2-cycle lane semantics
   (iss_ready back-pressure on iq_int, cmpl channel on retire).
   Expected WNS recovery: **+18 to +19 ns** by removing the 209-
   level combinational divide.
2. **Residual control fanout review** — `pd_valid` already has a source
   pragma from this campaign, but `crosses_line` and flush distribution
   should be reviewed once synth resumes.  Sim validation should prove
   branch/flush tests are cycle-stable before timing is re-measured.
3. **Split MULS into dedicated 2-stage lane** — the ALU flag path
   §1 rows 4–8 folds the DSP48E2 cascade and the flag-merge mux into
   one cycle.  MULS is already 2-cycle for the first stage; the
   flag-merge still happens combinationally in cycle 2 and could be
   split to two stages.
4. **Reset tree replication** (§4.6) — the `rst_IBUF_inst/O` at
   fanout 6795.  BUFGCE on clk is already in place; a dedicated
   reset BUFGCE + per-hierarchy replication could shave ~0.2 ns of
   reset path routing.
5. **Rewrite clock-period override in vivado.tcl** to use
   `reset_timing; create_clock` (fix the set_property read-only
   error encountered in Attempt 2).
