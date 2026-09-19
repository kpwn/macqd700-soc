# Fmax analysis — post-synth @ 6135513 (WNS -4.242 ns, Fmax ≈ 108 MHz)

Diagnostic + prescriptive sibling to `synth/timing_reports/baseline_6135513.md`.

**TL;DR.**  The worst 20 paths in the design are all the SAME logical path
fanning into different bits of `u_if/l0_addr_reg` / `u_if/l0_data_reg`.  Root
cause is a combinational feedback loop that leaves decode → fetch backpressure
un-registered: iq_mem's selection + dispatch-ready logic is an O(N²)
comparator, its output feeds `rn_ready`, which (via the decode-stage
`pd_consumed`) flows into the 32-bit PC adder, the `crosses_line` compare, and
the `l0_addr` / `l0_data` clock-enable — all in one clock edge, 38 logic
levels, 9.001 ns.

Registering the `disp_ready` output from each IQ (one FF per IQ) removes ~5 ns
of this path in a single commit and moves the WNS chapter to the real #2,
which is the DSP-heavy ALU flags output (-1.180 ns, 24 levels).

---

## §1 — Top 10 worst-slack paths

Paths were extracted via `report_timing -unique_pins -max_paths N` on
`build/vivado/checkpoints/synth.dcp`.  Because the tool reports each
endpoint separately, rows 1–8 collapse to **one physical critical path**
differing only in which CE/D bit of `l0_addr_reg` / `l0_data_reg` sinks
it.  I've collapsed them below and used the remaining rows for distinct
logical paths.

| # | Start FF | End FF | Slack | Data-path delay | Logic levels | Cell mix | Category |
|---|----------|--------|-------|-----------------|--------------|----------|----------|
| 1 | `u_iq_mem/e_disp_reg[1][5]` | `u_if/l0_addr_reg[*]` / `l0_data_reg[*]` (~190 bits, fo=156) | **-4.242** | 9.001 ns (logic 2.78 / route 6.22) | **38** | 21×LUT6 + 5×LUT5 + 3×LUT4 + LUT3 + LUT2 + 7×CARRY8 | A (combinational depth) + B (fanout) + F (cross-hierarchy) |
| 2 | `u_iq_mem/e_disp_reg[1][5]` | `u_rob/e_br_target_reg[11..14][*]` | -3.246 | 8.091 ns | 32 | 21×LUT6 + 6×LUT5 + 2×LUT4 + LUT3 + 2×CARRY8 | A + F |
| 3 | `u_iq_mem/e_disp_reg[1][5]` | `u_rat/ratmap_reg[0..N][*]/CE` | -2.829 | 7.588 ns | 30 | 19×LUT6 + 5×LUT5 + 2×LUT4 + 2×LUT3 + 2×CARRY8 | A + F |
| 4 | `prf_reg[3][31]` | `u_alu/flags_out_reg[2]` | **-1.180** | 6.025 ns (logic 3.61 / route 2.41) | 24 | 6×LUT6 + mixed, **1×DSP48E2 chain** (DSP_MULTIPLIER + 2×DSP_ALU + 2×DSP_OUTPUT), 3×CARRY8, MUXF7/MUXF8 | E (unbalanced arithmetic, via ALU+MUL+flag path) + D (DSP staging) |
| 5 | `prf_reg[3][31]` | `u_alu/flags_out_reg[3]` | -0.346 | 5.191 ns | 21 | 5×LUT6 + DSP chain + 3×CARRY8 | E + D |
| 6 | `prf_reg[3][8]` | `u_alu/result_reg[25]` | -0.214 | 5.059 ns | 20 | 2×LUT6 + 4×LUT5 + DSP chain + 2×CARRY8 + MUXF7/8 | E + D |
| 7 | `prf_reg[3][8]` | `u_alu/result_reg[29]` | -0.209 | 5.054 ns | 20 | same shape as #6 | E + D |
| 8 | `prf_reg[3][8]` | `u_alu/result_reg[23..26]` | -0.12 ≈ -0.10 | ~4.97 ns | 19-20 | same shape | E + D |
| 9 | `u_if/pc_reg[0]_rep__0` | `u_if/l0_addr_reg[*]/CE` | -0.641 | 5.400 ns | 23 | 12×LUT6 + 3×LUT4 + 2×LUT2 + 5×CARRY8 | A (the fetch tail alone) + B (fo=156 on `l0_data`) |
| 10 | `u_lsu/FSM_onehot_state_reg[0]` | `u_if/l0_addr_reg[*]/CE` | (would be negative; see note) | 3.380 ns | 16 | 7×LUT6 + LUT4/3/2 + 5×CARRY8 | F (LSU `lsu_ready` → fetch tail) |

Notes on rows 1–3: same source FF (`u_iq_mem/e_disp_reg[1][5]` — this is
bit-5 of the 32-bit displacement on IQ slot #1, i.e. a random data bit that
happens to be the placed "representative" of the whole O(N²) disp-comparator
cloud that feeds `load_blocked`).  The endpoint differs because the long
combinational fan continues into three wide structures (if_stage l0 write-
enables, ROB br_target write, RAT ratmap CE) — same mountain, three
avalanche paths.

Rows 4–8 are the "real" #2 once row 1 is registered: the 1-cycle ALU
including the 32×32 MULS/MULU spanning DSP cascade → flag-gen → FLAG
write.

Row 9 is the fetch tail **alone** (pc adder → crosses_line → l0 CE) with
iq_mem filtered out of the source.  It sits at -0.64 ns on its own — i.e.
even a perfectly-registered dispatch would leave ~0.6 ns of WNS on the
table purely from the PC increment + cross-line compare driving 156-load
CE nets.

Row 10 is another reminder that `if_stage.l0_addr.CE` is over-constrained
— many unrelated control signals (`lsu_ready`, `commit.flush_en_rep`)
feed the same 5-CARRY8 `crosses_line` compare and then fan out to 156
CE pins.

## §2 — Categorisation

Applying the rubric:

| Path rows | Category | Evidence |
|-----------|----------|----------|
| 1 | **A + B + F** | 38 levels, 69% route delay, fo=156 net at the tail, spans iq_mem → m68k_core → decode → if_stage. |
| 2 | A + F | 32 levels, crosses iq_mem → m68k_core → rob. |
| 3 | A + F | 30 levels, crosses iq_mem → m68k_core → rat. |
| 4–8 | **E + D** | 3.6 ns in logic alone; mix includes full DSP48E2 multiply cascade driving a 64→32 mux and then a 5-bit flag OR-tree.  The DSP output is not pipe-registered before feeding the flag path. |
| 9 | A + B | 23-level tail from `pc + pd_consumed` through `crosses_line` into a fo=156 CE net. |
| 10 | F | Cross-module (lsu → if_stage) path into the same `crosses_line` tail. |

High-fanout register hit list (from `report_high_fanout_nets`, thresholded
at 100 loads — these are the nets most likely to dominate route delay on
placed paths):

| Net | Fanout | Why it is big |
|-----|-------:|---------------|
| `rst_IBUF_inst/O` | 6795 | Synchronous-reset fanout to everything; BUFGCE is already used.  Methodology will flag as not replicated; after place will warrant ~4 replicas. |
| `u_rob/e_br_target[29][31]_i_1_n_0` | 1280 | ROB per-entry CE driver — one LUT1 fans out to 40 entries × 32 bits = 1280 FFs.  Should be register-replicated. |
| `u_rob/ADDRD[0..2]` | 591 / 591 / 147 | Distributed-RAM write-address broadcast inside rob.  Normal for a 40-entry × wide-payload distRAM. |
| `u_commit/bpu_update_pc_reg[*]_0[0..5]` | 135–574 | `bpu_update_pc` fans into every BTB entry's tag compare.  Phase-1 BPU trains by PC, so this fanout is unavoidable until we switch to synchronous update (phase 2). |
| `u_iq_int/int_iss_psa/psb[0..2]` | 192–384 | **Suspicious** — the issued phys-src tag goes into the age-priority mask + hazard-match logic across entries.  One LUT replica per 8 would halve route. |
| `u_if/i___0_i_1_n_0` (= `crosses_line`) | 189 | The PC adder's overflow / line-cross predicate fans into every l0_* CE.  Named in row 1 of §1. |
| `u_if/pd_valid` | 145 | Fetches 145 loads — every decode AND iq entry. |
| `u_alu/cdb0_en` | 164 | ALU's valid-out drives 164 wake-up snoop pins across RAT + both IQs + LSU (every renamed entry).  Already CDB-style, no fix. |

The three nets that matter for our top paths: `crosses_line` (row 9),
`int_iss_psa/psb` (potential row 3 contributor), `e_br_target[29][31]_i_1`
(ROB CE driver; candidate for replication independent of slack).

## §3 — Prioritised retiming plan

Each commit is small and independent unless noted; cumulative slack
recovery is approximate.

### P1.  Register `disp_ready` on both issue queues
**Addresses rows 1, 2, 3** (and eliminates ~5 ns in one go).

- **Files**: `rtl/core/issue/iq_mem.v`, `rtl/core/issue/iq_int.v`, `rtl/core/m68k_core.v`.
- **Change**: today `assign disp_ready = has_free;` where `has_free` is the
  tail of an O(N²) comparator tree.  Add `reg disp_ready_r;` and sample
  `disp_ready_r <= has_free;` into the next-cycle dispatch gate.  The
  dispatch path in `m68k_core` already uses `rn_ready` combinationally;
  retime the gate so `rn_ready = … && iq_ok_r && …` reads the registered
  copy.  Since IQ capacity changes by at most ±1 per cycle and we have 8
  entries with 1-cycle latency to the allocator, one-cycle staleness is
  safe as long as we conservatively stall on "was full last cycle".
- **Expected WNS impact**: **+4.5 ns** to the iq_mem path — the adder and
  crosses_line tail alone are 5.4 ns (row 9), so the post-fix WNS will
  likely be gated by row 4 (-1.18 ns).  Post-P1 estimated WNS ≈ **-1.2 ns**.
- **Latency cost**: +1 cycle of dispatch backpressure response.  Mostly
  invisible because the pipeline fills the IQ slowly anyway; expect
  <2% IPC hit on the loop-heavy tests.
- **Correctness risk**: **medium**.  The stale-for-one-cycle `disp_ready_r`
  must be qualified by "also, are we dispatching this cycle?" to avoid
  double-allocating into the same free slot.  Same-cycle conflict check
  lives in m68k_core (already has `alloc_en = rn_ready && d_has_dst`).
- **Bypass consequences**: none on CDB / PRF forwarding.  Does not touch
  the ALU→CDB→IQ wake-up paths at all.
- **Test risk**: high-coverage.  Re-run every iq_mem-touching test:
  `smoke, load_use, waw_hazard, raw_chain, war_hazard, store_load_alias,
  branch_skip_load, two_stores, bsr_rts_basic, dbcc_loop, loop_count`.
  If any show a new stall-induced cycle regression, investigate — but a
  1-cycle IPC hit is acceptable.

### P2.  Replace iq_mem's O(N²) `load_blocked` with an O(N) mask + age grid
**Addresses row 1's actual logic volume, also shrinks LUT count by ~200.**

- **File**: `rtl/core/issue/iq_mem.v`.
- **Change**: today `load_blocked[k]` is an 8×8 double-loop doing
  `e_pbase[j]==e_pbase[k] && e_disp[j]==e_disp[k]` for every (k,j)
  pair.  Precompute once per dispatch:
  `e_alias_sig[k] = {e_pbase[k], e_disp[k]}` (38-bit).  On dispatch,
  set a bitmap `older_store_alias_of[k][j] = (e_is_store[j] &&
  e_alias_sig[j]==e_alias_sig[k] && is_older(j,k))`.  Store it as a
  per-entry [IQ_DEPTH-1:0] register and update incrementally on
  valid-flip / dispatch / fire.  Then `load_blocked[k]` collapses to a
  single 8-wide OR.
- **Expected WNS impact**: on its own (without P1), probably **+2.0 ns**
  on the row-1 path because the comparator cloud is ~half the 2.8 ns
  of pure logic delay.  With P1 already applied, this is quality-of-
  implementation rather than slack-critical.
- **Latency cost**: none (same cycle, cheaper circuit).
- **Correctness risk**: medium.  The bitmap must flush correctly on
  sel-fire / disp-fill / flush.  Unit-testable.
- **Test implications**: same list as P1.  Cross-check with
  `store_load_alias` (the one test whose correctness depends on this
  exact check).

### P3.  Register ALU source reads (`prf[int_iss_psa]` / `[psb]`)
**Addresses rows 4–8.**

- **File**: `rtl/core/m68k_core.v` (maybe `rtl/core/execute/alu.v`).
- **Change**: Today the ALU reads `alu_a_val = prf[int_iss_psa]`
  combinationally and feeds straight into the big case statement incl.
  32×32 MULS.  Add a 1-cycle RS-read pipe stage: latch `alu_a_val_r`,
  `alu_b_val_r` on the cycle the IQ issues; ALU evaluates on cycle +1.
  This splits the 6 ns from (PRF read → ALU case → DSP cascade → flag
  gen) into (PRF read → register) + (register → ALU case → DSP → flag gen).
- **Expected WNS impact**: **+1.5 to +2.0 ns** on the ALU flag path.
  The DSP cascade alone is ~2.4 ns of pure logic in row 4; after the
  cut, the surviving path is "register → DSP → mux → CARRY8 → flag" at
  roughly 3.5 ns, well under budget.
- **Latency cost**: +1 cycle on every INT op.  **This is the main IPC
  tax** — dependent integer chains get an extra bubble.  Measurable
  hit on `raw_chain`, `bench_dep_chain`, tight compute loops.
- **Correctness risk**: low (pure pipelining).
- **Bypass consequences**: **yes** — we need to bypass the registered
  source value from CDB to the added stage.  Today CDB wake-up + PRF
  write happen in the same cycle a dependent uop can issue; after the
  cut, we need a forwarding mux from `cdb0_data` / `cdb1_data` into
  `alu_a_val_r` / `alu_b_val_r` when the issued source matches.  Not
  trivial, but standard RS design.  Without this forwarding, RAW
  chains would double their latency — the 2.0 ns WNS win would cost
  ~10% IPC on ALU-heavy workloads.
- **Test implications**: re-run every ALU-touching test.  Add a focused
  bench for back-to-back ALU dependency to confirm the forwarding mux
  works.

### P4.  Replicate / register `crosses_line` in if_stage
**Addresses row 9 (-0.64 ns) and the F-category cross-module variants (row 10).**

- **File**: `rtl/core/fetch/if_stage.v`.
- **Change**: today `crosses_line = consuming && (nxt_pc[31:4] != pc_line)`
  fans to 189 CE pins (l0_data[127:0], l0_addr[27:0], plus l1_ invs).
  Add a `(* MAX_FANOUT = 32 *)` on `crosses_line` (or manually replicate:
  one copy per l0_data/l0_addr byte group).  Nicer still: register
  `crosses_line_q`, gate the l0 writes one cycle later — but the data-
  flow in if_stage is currently "cycle-perfect" with the bus handshake,
  so a MAX_FANOUT hint is the low-risk choice.
- **Expected WNS impact**: **+0.5 ns** — mostly removes the 3.8 ns of
  pure route on row 9.  Will also lift several of the row-1-flavor
  paths that get through P1 with non-zero residual slack.
- **Latency cost**: zero if MAX_FANOUT; +1 cycle if we pipeline (not
  recommended yet).
- **Correctness risk**: low (synth attribute only) for MAX_FANOUT.
- **Test implications**: re-run smoke + any of the BTB-sensitive tests
  (`loop_count`, `dbcc_loop`, `bcc_test`) to confirm fetch behaviour
  unchanged.

### P5.  Break the ROB `flush_en_rep` → ROB_CE fanout of 1280
**Preemptive; no single top-10 row, but will surface as worst after P1–P4.**

- **File**: `rtl/core/rename/rob.v`.
- **Change**: `e_br_target[29][31]_i_1_n_0` is a synthesised `LUT1`
  that fans into 40 × 32 = 1280 FFs.  Manually replicate the flush CE
  driver per ROB slot (`(* MAX_FANOUT = 64 *) reg flush_en_per_slot[0:39];`
  or rewrite so each `always` block in rob.v uses a local `flush_en_r`
  driven from a small replicate tree).
- **Expected WNS impact**: **+0.3 ns** on whichever path lands on a
  flushed ROB entry's CE.
- **Latency cost**: none.
- **Correctness risk**: low.
- **Test implications**: `mispredict`, `branch_skip_load`, any flush-
  heavy test.

### P6.  Replicate `int_iss_psa/psb` selects on iq_int outputs
**Mirror of P5 for iq_int; both preemptive.**

- **File**: `rtl/core/issue/iq_int.v`.
- **Change**: issued phys-tag broadcasts at fo=192–384.  Add
  `MAX_FANOUT = 48` pragma or manually register-replicate.
- **Expected WNS impact**: **+0.2 ns** on whichever path lands there.
- **Latency cost**: none.
- **Risk**: low.

### Commit sequence summary

| Commit | WNS gain (est) | IPC cost | Risk | Unblocks |
|--------|----------------|----------|------|----------|
| P1 (register disp_ready) | **+4.5 ns** | ~2% | Medium | Everything below — without this, other fixes are invisible. |
| P4 (MAX_FANOUT on crosses_line) | +0.5 ns | 0 | Low | Could land in parallel with P1; they don't touch the same file. |
| P3 (register ALU operands) | +1.8 ns | 5–10% on compute-bound | Low (core) / Medium (forwarding) | Best done after P1 so the baseline stabilises. |
| P5 (ROB flush CE replicate) | +0.3 ns | 0 | Low | Parallel. |
| P6 (iq_int psa/psb replicate) | +0.2 ns | 0 | Low | Parallel. |
| P2 (iq_mem O(N²) → O(N) comparator) | +0 ns (after P1) / quality | 0 | Medium | Standalone LUT-shrink win; do after P1. |

Cumulative expected post-retiming WNS: starting from -4.242 ns,
recovering +4.5 (P1) + 0.5 (P4) + 1.8 (P3) + 0.3 (P5) + 0.2 (P6) =
**+7.3 ns of headroom**, landing somewhere around **WNS +3.0 ns** —
i.e. 5.0 ns target with ~3 ns slack, or ~350 MHz theoretical post-synth
(placed/routed will drop some, plausibly 250–280 MHz achievable).

## §4 — Structural gaps exposed

Three architectural observations that aren't retiming fixes but should
open tickets.

### §4.1  `disp_ready` is a global combinational feedback signal
Every issue queue publishes `disp_ready` combinationally from its
oldest-free-slot scan.  Combined over 3 queues and the RAT's
`alloc_ok`, the dispatch gate in `m68k_core` (`rn_ready`) is a
wide AND of combinational outputs from all 4 modules, none of
which are registered.  This is what makes the iq_mem path so
long.  **Structural fix**: define a protocol where each issue
queue publishes `disp_ready_r` (registered) and pre-reserves a
slot if it signalled ready.  This is a small contract change but
touches every IQ simultaneously.  Best done before P1 as the
framework P1 rides on top of.

### §4.2  CCR RAT sits in the int dispatch path but isn't a top-10 contributor
Interesting negative result: I expected `ccr_alloc_ok` to surface in
the critical path alongside `alloc_ok`.  It doesn't, because `ccr_rat`
is lightweight (96 LUTs, 136 FFs total).  No action needed, but
confirms the CCR-rename landing was Fmax-neutral — a good result
given how invasive it was.

### §4.3  ALU is single-cycle INCLUDING 32×32 MULS
ALU rows 4–8 all traverse the DSP48E2 cascade in the same cycle as
the rest of the ALU logic.  The 68040 had a separate multiply pipe;
we're currently folding it into the general ALU to save routing.
This is structurally questionable at any target >200 MHz — MULS/MULU
takes 3–4 DSP cascade stages on KU5P -2.  **Structural fix**: promote
MULS/MULU to a 2-cycle ALU sub-lane (like real OoO designs).  This
is what the Motorola 88110 / MIPS R10000 / most commercial OoO cores
do.  P3 is a less-invasive near-term mitigation (pipelines the PRF
read so at least the DSP cascade starts from a clean register).

### §4.4  BPU train PC fanout (135–574) is inherent to direct-mapped bimodal
The `bpu_update_pc_reg[31]_0[*]` signals fan wide because every BTB
entry compares against them.  Expected behaviour for direct-mapped,
but as soon as we move to gshare / TAGE (phase 4) we should switch
train-side to *one-hot entry decode* rather than broadcast-and-
compare.  Not a phase-1 ticket.

### §4.5  `u_lsu` at 3010 LUTs is bigger than expected
`u_lsu` is using more LUT than `u_rob` despite simpler state.  This
is a flag for a separate `lsu-audit` pass, not a timing fix.  The
module's 34-state FSM and AXI lane mapping are suspect.

## §5 — Estimate: post-retiming Fmax

Period target: 5.000 ns.  Current data-path delay on worst: 9.001 ns.
Gap to close: **4.001 ns of data path + 0.241 ns of skew/uncertainty =
4.242 ns WNS**.

§3's cumulative recovery projection is +7.3 ns.  That overshoots the
gap by ~3 ns of margin, which is where I'd want to land anyway given
P&R typically erodes ~1.5–2.0 ns more on a top-of-hierarchy design of
this density.

**Sanity check**: the gap is 4.0 ns, P1 alone delivers ~4.5 ns — so on
paper P1 *alone* hits 200 MHz.  I don't fully trust that because row
9 (-0.64 ns, if_stage tail alone with iq_mem filtered) implies even
a perfect iq_mem gives only ~0.64 ns of spare, which P&R would
likely consume.  That's why P4 belongs in the mandatory set — it
removes the hidden slack cliff lurking behind P1.

**Realistic target after §3**: WNS ≈ 0.5 to 1.5 ns post-synth → Fmax
of ~230 MHz with room for the P&R hit.  200 MHz closes comfortably.

**Structural gap above 200 MHz**: for a 250 MHz stretch (4.0 ns), §4.3
— the single-cycle-including-MULS ALU — becomes the blocker.  That's
a phase-4 concern and wants a full "split MULS into mul_div.v" ticket,
not a retiming patch.

## §6 — Open questions for the user

1. **IPC tolerance for P3.**  Registering the PRF read adds a mandatory
   1-cycle bubble to every ALU op unless we also add a CDB→RS
   forwarding mux.  Are we OK trading ~5% IPC on the ALU path for ~1.8
   ns of WNS?  Alternative: leave P3 for later, land P1+P4+P5+P6, land
   somewhere around WNS -0.5 to -1.0 ns, live with ~180 MHz post-synth
   for another phase.

2. **Dispatch backpressure latency for P1.**  P1 turns the dispatch-
   ready line from combinational into 1-cycle-delayed.  This means the
   back-end needs to flag "I was full last cycle" explicitly, and the
   front-end decode must hold the uop for one extra cycle on IQ-full.
   Acceptable?  Or do you want a handshake where the IQ pre-reserves
   a slot combinationally and we register only the downstream
   indicator?  (Second version is more conservative, same timing win.)

3. **P2 scope.**  The load-blocked bitmap refactor is a non-trivial
   iq_mem rewrite.  Given P1 solves the slack problem alone, do you
   want P2 at all, or should it wait until we also need the LUT count
   back for a larger IQ?

4. **LSU size (§4.5)**.  Do you want an `lsu-audit` ticket to figure
   out why 3010 LUTs?  Not timing-critical today but will be once P3
   lands and the ALU stops dominating.

5. **Phase split for this doc**.  Would you like §3 split into a
   "phase-1 retiming" (P1, P4, P5, P6 — none cost IPC) and a
   "phase-2 retiming" (P3 — costs IPC, needs forwarding)?  I can
   author sub-tickets if so.

6. **Reset fanout (6795 loads)**.  Already very high.  We're using a
   BUFGCE on clk but reset is a straight IBUF → global net.  Do you
   want a reset-tree ticket (split into per-hierarchy buffered
   resets), or is it fine to let P&R replicate?  Low priority, but
   it'll show up in placement reports.
