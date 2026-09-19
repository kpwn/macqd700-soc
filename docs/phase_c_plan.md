# phase_c_plan.md — Phase C (2-wide OoO) Implementation Plan

> **Deliverable for task #120.**  Dense, implementation-actionable plan for
> Phase C of the IPC roadmap (`docs/ipc_roadmap.md` §5, §6, §8).  Target: lift
> peak IPC from today's **0.323** (post-A2, post-#113) across the 1.0 line
> onto `bench_move_heavy`, `bench_cmp_branch`, `bench_ind_adds`.  Everything
> here is docs-only; no RTL edits accompany this file.
>
> Ground-truth bench measurements below were captured at the top of this
> session with `/tmp/m68k-ooo-build/build/sim/Vmac_top +test=<bench>
> +timeout=500000` — the #113 CMP+Bcc/SUBQ+Bcc fusion deltas are incorporated
> (bench_cmp_branch 501→440, bench_btb_loop 829→723).
>
> Cross-references to `docs/ipc_roadmap.md` §5 (dispatch), §6 (AGU bypass),
> §8 (sequencing), §9 (risks) are the authoritative spec.  This doc
> operationalises those into a ticket order, LUT/WNS budgets, and
> per-bench projections.

> **2026-04-18 status overlay:** C1 is no longer future work.  The F1
> registered PRF read + CDB bypass landed as `fe9f27d`; current bench deltas
> are in `docs/bench_baseline.md` under "Phase-C C1 delta".  Read §6 as the
> design record for the landed implementation.  The live Phase C queue starts
> after C1, and PM sequencing should keep width work behind first-light and
> control-fanout cleanup unless a sim-only task is tightly scoped.

---

## §1 Executive summary

### 1.1 Historical planning baseline (`main @ f1f5653`)

| Bench              | Cycles | Committed | IPC    | Δ vs ipc_roadmap §2.1 |
|--------------------|-------:|----------:|-------:|-----------------------|
| bench_alu_parallel |    857 |       216 | 0.252  | cycles -2, committed -78 (A2 ref-count + #113 bench recompile) |
| bench_btb_dbra     |    723 |       110 | 0.152  | cycles -200 (already counted #107 l1i-prefetch) |
| bench_btb_loop     |    723 |       110 | 0.152  | **-301 cyc** from 1024 (#113 SUBQ+Bcc fuse) |
| bench_cmp_branch   |    440 |        72 | 0.164  | **-280 cyc** from 720 (#113 CMP+Bcc fuse, -99 committed) |
| bench_dep_chain    |    282 |        28 | 0.099  | +9 cyc (retune noise) |
| bench_fullpipe     |   1114 |       318 | 0.285  | +8 cyc (A2 alias CDB pressure) |
| bench_ind_adds     |    854 |       216 | 0.253  | -0 (MOVE-elim doesn't hit loop body) |
| bench_mixed_mem    |    685 |       125 | 0.182  | -5 cyc |
| bench_move_heavy   |   2043 |       611 | 0.299  | -46 cyc (A2 ELIM_MOVE drops ALU_MOV cycles, but CCR-µop still serialises at 1-wide) |

Peak IPC today: **bench_move_heavy 0.299** (the task prompt's "0.323" matches
the pre-A2-ref-count baseline; A2 landed as a correctness-neutral enabler
so the ceiling is unchanged).  Floor: **bench_dep_chain 0.099**.  The IPC
distribution is bimodal — half the suite clusters at 0.15-0.18, the
dispatch-bound half at 0.25-0.30.  Phase C breaks that ceiling.

### 1.2 Phase C projected IPC (end of §11)

| Bench              | Today | Phase C floor | Phase C peak | Crossing |
|--------------------|------:|--------------:|-------------:|----------|
| bench_ind_adds     | 0.253 |         0.70  |        0.95  | ≥1.0 @ D |
| bench_move_heavy   | 0.299 |         0.85  |        1.25  | **≥1.0** |
| bench_alu_parallel | 0.252 |         0.75  |        1.05  | **≥1.0** |
| bench_cmp_branch   | 0.164 |         0.35  |        0.55  | ≥1.0 @ D |
| bench_fullpipe     | 0.285 |         0.55  |        0.75  | ≥1.0 @ D |
| bench_mixed_mem    | 0.182 |         0.40  |        0.55  | ≥1.0 @ D |
| bench_btb_loop     | 0.152 |         0.35  |        0.50  | ≥1.0 @ D |
| bench_btb_dbra     | 0.152 |         0.33  |        0.45  | ≥1.0 @ D |
| bench_dep_chain    | 0.099 |         0.12  |        0.14  | never-D |

Geomean across 9 benches: **0.19 → ~0.55 (floor) / ~0.75 (peak)**. The
peak-crossing benches at end-of-Phase-C are `bench_move_heavy` (fan-out
post-A2 absorbs ≥2/cyc once rename is 2-wide) and `bench_alu_parallel` (6
independent ALU ops per iter, perfect match for 2nd ALU + dual-issue).

### 1.3 One-line recommendation

**F1 is already landed.**  The remaining structural order is CCR-RAT
2-port, then RAT 2-port, then ROB 2-retire, then IQ-int 2-pick, then 2nd
ALU, then 2-wide decode front-end, then full glue.  Total remaining effort is
roughly **17–21 agent-days** before the Fmax close-out pass, estimated
**+0.5 IPC floor / +0.7 IPC peak** on the benches.  Do not spend the F1 slack
twice: each remaining width ticket still needs sim performance gates and a
PM-scheduled timing window later.

---

## §2 Decode 2-wide

### 2.0 Current decode state

`rtl/core/decode/decode.v` is 4257 lines, single-instruction-per-cycle,
with some multi-phase cracks (MOVEM up to 17 phases, BSR 2 phases,
CHK2 3 phases, BFINS dynamic variable).  Key structural landmarks:

- Lines 54-120: port list (inputs from predecode, outputs to rename).
- Lines 192-279: sequential always block (drives `uop_phase`).
- Lines 283-433: fusion peek-ahead (task #113).  Six `fuse_fire_*` wires
  that steer the main case tree to emit a fused BR_CMP_BCC / BR_SUBQ_BR
  etc.
- Line 436: `always @(*)` main combinational decode driver.
- Line 471: top-level `casez (op[15:12])` over the opword's top nibble
  — this is the 4000+ line case tree.
- Lines 509+, 629+, 676+, 779+, 902+, etc.: multiple `case (uop_phase)`
  blocks inside cracked instructions (MOVEM, MOVEP, BFINS dynamic).

All case-tree output signals (`uop_type`, `uop_op`, `has_dst`,
`arch_dst`, `imm`, `flags_*`, `is_branch`, `exc_*`, etc.) are
combinational assignments off a single per-cycle decode.  The
single-μop-per-cycle contract is implicit in every one of those
assignments.

### 2.1 Goal and shape

Decode emits **1 or 2 μops per cycle**, where the second μop is emitted
*only if* the first μop is `phase=0` AND single-phase (no crack), AND the
second instruction at `pd_pc + len_bytes_0` is also single-phase, AND the
combined `pd_consumed` fits in the 16-byte `pd_buf`.  Otherwise decode
falls back to 1 μop/cycle — the existing crack-phase paths for MOVEM /
BSR / CHK2 / BFINS-dynamic are unchanged.

**Why 2-wide, not 3-wide?** 68040's variable-length encoding makes
predecode-width the critical path.  Per `docs/ipc_roadmap.md` App. B:
"3-wide predecode is 6+ LUT levels and blows 200 MHz." 2-wide stays at 4
LUT levels after pipelining the pair-classifier (see §2.3 below).

**Why gate on single-phase?** Multi-phase cracks (MOVEM etc.) use the
`uop_phase` register at `decode.v:509` to sequence output across cycles.
Two in-flight cracks would need two phase counters and two output
pipelines — 2-3× the area for the 10% of code that exercises them.
Sequential fallback is fine.

### 2.2 Concrete changes in decode.v

**Port duplication (slot 0 and slot 1):**

- `decode.v:54-120` (ports). Duplicate every `uop_*`, `has_*`, `arch_*`,
  `flags_*`, `is_*`, `imm_*`, `exc_*`, `uop_pc`, `uop_npc`, `elim_*`
  output port into `_0` and `_1` versions.  Add a top-level `uop1_valid`
  port that drives the rename 2nd-alloc lane.
- Keep `pd_consumed` as a single 5-bit field (sum of both slots' lengths;
  8-byte combined max).  Keep `rn_ready` as a single 2-bit input
  `rn_ready_0`/`rn_ready_1` — decode produces slot 1 only when
  `rn_ready_1` is asserted.

**Case-tree duplication strategy (§2.3):**

Decode's inner `casez (op[15:12])` at `decode.v:471` is 4000+ lines of
per-opword logic.  Duplicating the tree verbatim is a 2× LoC blow-up
with high collision risk against concurrent decode-adding agents.

**Recommended approach: predecode-pair classifier + single case-tree
reused.**  Add a new `predecode.v` pair stage that classifies both
instructions' {length, is_single_phase, is_fusion_pair,
is_elim_candidate} in parallel, then **runs the existing case-tree
twice — once on the slot-0 opword, once on the slot-1 opword**, with the
slot-1 evaluation using a new `pd_buf_1` wire that is `pd_buf[127:16 *
len_0]` shifted.

The case tree stays as one combinational block but its inputs
(`op`, `op_mode_lo`, `uop_phase`, etc.) become indexed by `slot`.  This
is a genvar-generate wrap around the existing block:

```verilog
genvar s;
generate for (s = 0; s < 2; s = s + 1) begin : g_decode_slot
    // instantiate the existing casez on pd_buf_s / op_s, writing
    // uop_type[s], uop_op[s], etc.
end endgenerate
```

Duplicated slot 0/1 case trees cost: ~2× LUTs on the decode combinational
block (today ~1200 LUTs → ~2400 LUTs).  LUT budget: still <2% of KU5P.
WNS cost: the slot-1 case tree reads `pd_buf_1`, which is a 128:16 shifter
selected by `len_0 ∈ {2,4,6,8}` — a 4:1 mux on the 16-byte window = **1
LUT6 level (~0.4 ns)**.  Total decode path: ~3.7 ns → 3.9 ns — within
Phase C F1 headroom.

### 2.3 Cracked-µop straddle (the hard corner)

**Scenario**: slot 0 decodes a cracked macroinst (e.g. MOVEM with 5-reg
list → 6 phases) while slot 1 is empty.  The next cycle, slot 0 must
continue phase 1 of MOVEM and slot 1 must stay silent.  Handled: `uop_phase`
register stays at its current single-slot identity; slot-1 classifier
gates off when `uop_phase != 0`.

**Scenario**: slot 0 is phase 0 of a single-phase inst (ADD Dm,Dn) and
slot 1 is a cracked macroinst.  We want to emit slot-0-this-cycle and
slot-1-phase-0-this-cycle, then slot-1-phase-1 next cycle in slot 0.
Requires: `uop_phase` + `uop_slot_origin` register pair so next cycle
knows the in-flight crack came from the "slot 1" origin.  **Recommended:
skip this case.**  Gate: slot-1 emission requires `is_single_phase_1 ==
1`.  MOVEM / BSR / CHK2 / BFINS as slot-1 stay 1-wide that cycle.  Cost
in IPC: these cracks are <5% of dynamic μops; upper bound 5% of the
2-wide gain lost ≈ **≤1% absolute IPC on bench suite**.

**Scenario**: fusion already collapses a 2-inst window to 1 μop
(#113's CMP+Bcc, SUBQ+Bcc, TST+Bcc).  At 2-wide decode, the fusion
consumer is slot-0 (fused μop) + empty slot-1 that cycle — the
following cycle's first inst was the Bcc, which is already consumed.
Fusion firings are a net loss of 2-wide utilisation but a win on μop
count; the math still favours fusion (one 2-inst pair → 1 μop in 1
cycle vs 2 μops in 1 cycle under 2-wide).  **No change needed** —
`fuse_fire_*` wires at `decode.v:416-433` already force `pd_consumed`
to 4-8 bytes, which is the combined length of the pair.

### 2.4 Estimated cost

| Item                              | LoC added | LUTs     | WNS delta | Days |
|-----------------------------------|-----------|----------|-----------|------|
| Port duplication                  |    ~300   |    0     |    0      | 0.5  |
| Pair classifier (pd_pair_stage.v) |    ~200   |   +180   | -0.2 ns   | 1.0  |
| Slot-1 case-tree (genvar wrap)    |    ~100   |  +1200   | -0.3 ns   | 1.5  |
| Fusion re-gate (slot-0 only)      |     ~30   |    0     |    0      | 0.2  |
| Crack-straddle guard              |     ~50   |    0     |    0      | 0.3  |
| Unit tests in `tb/tb_decode.cpp`  |    ~400   |    —     |    —      | 1.0  |
| **Total**                         |  **~1080**| **+1380**| **-0.5 ns**| **4.5 days** |

### 2.5 Verification strategy

1. Extend `tb/tb_decode.cpp` (today 3 scenarios) with:
   - 2 single-phase insts in row (ADD + SUB): expect 2 μops, pd_consumed=4.
   - Single-phase + cracked (ADD + MOVEM): expect slot-0 + phase-0 slot-1, pd_consumed==combined_len; next cycle slot-0=phase-1, slot-1=0.
   - Cracked + single-phase: slot-0 crack starts, slot-1 suppressed.
   - Fusion slot-0 + any slot-1: slot-1 suppressed; pd_consumed==fused_len.
   - Decode-time exception (A-line) on slot 0: slot-1 must be suppressed
     (exception dispatches through ROB serialised in program order).
2. Fuzz widening in `tools/fuzz/gen_program.py`: emit adjacent
   dual-single-phase sequences at 40% probability (up from <5% today's
   baseline).
3. Musashi golden-ref should show **zero semantic delta**; all 200
   seeds PASS after 2-wide lands.

### 2.6 Worked example: 2-wide MOVE+ADD pair

To ground the implementation, trace a specific 2-wide decode cycle.

**Input**: `pd_buf[127:0] = { MOVE.L D0,D1 | ADD.L D2,D3 }`, where
MOVE.L D0,D1 is opword `0x2200` (2 bytes), ADD.L D2,D3 is `0xD682` (2
bytes).  `pd_pc = 0x40800000`.

**Pair classifier output** (new predecode stage):
- `len_0 = 2`, `len_1 = 2`.
- `is_single_phase_0 = 1` (register-register MOVE is single-phase).
- `is_single_phase_1 = 1` (ADD is single-phase).
- `is_fusion_pair = 0` (MOVE+ADD is not a fusion shape).
- `is_elim_candidate_0 = 1` (MOVE Dn,Dm → ELIM_MOVE).
- `is_elim_candidate_1 = 0`.

**Slot-0 decode** (`casez (op_0[15:12])` hits `4'b0010`, MOVE.L D0,D1):
- `uop_type_0 = UOP_INT`, `uop_op_0 = ALU_MOV`.
- `has_src_a_0 = 1`, `arch_src_a_0 = 0` (D0).
- `has_dst_0 = 1`, `arch_dst_0 = 1` (D1).
- `flags_wr_0 = 5'b01111` (NZVC set, X preserved).
- `elim_kind_0 = ELIM_MOVE`, `elim_arch_dst_0 = 1`.

**Slot-1 decode** (`casez (op_1[15:12])` hits `4'b1101`, ADD.L D2,D3):
- `uop_type_1 = UOP_INT`, `uop_op_1 = ALU_ADD`.
- `has_src_a_1 = 1`, `arch_src_a_1 = 3` (D3).
- `has_src_b_1 = 1`, `arch_src_b_1 = 2` (D2).
- `has_dst_1 = 1`, `arch_dst_1 = 3` (D3).
- `flags_wr_1 = 5'b11111` (full NZVCX).
- `elim_kind_1 = ELIM_NONE`.

**Rename** (2-wide port):
- Slot 0: ELIM_MOVE.  `elim_phys_src_a_0 = ratmap[0]` (current phys of
  D0).  `ratmap[1] ← phys_of_D0`.  No phys pop.
- Slot 1: normal alloc.  `alloc_phys_dst_1 = first_free(free_bm)`.
  `phys_src_a_1 = ratmap[3]` (current phys of D3), `phys_src_b_1 =
  ratmap[2]` (current phys of D2).  **Intra-pair bypass does not fire**
  (arch_dst_0 = 1, arch_src_a/b_1 = {3,2}, no collision).

**Dispatch**:
- ROB inserts 2 entries (`disp_en_0/1`, tags `t`, `t+1`).
- iq_int inserts 2 entries (both have int srcs ready — D0/D2/D3 all
  committed).
- CCR-RAT allocates 2 new CCR tags (both writers).
- pd_consumed = 4 bytes.

**Execute** (cycle N+1 after F1 stage, N+2 from decode):
- ALU_simple executes slot-1 ADD (capable), cmpl on cdb1.
- ALU_full can execute slot-0 (but ELIM_MOVE was already aliased, so
  slot 0 is a no-op CCR-writer — CCR μop still dispatches, see §8.1
  and `docs/ipc_roadmap.md §4.2`).

**Commit** (cycle N+3 onwards): ROB retires 2/cycle.  RAT ref_count
bookkeeping: slot-0's ref_count[phys_of_D0] bumped at dispatch,
decrements at commit if the aliased slot no longer references it.

This example demonstrates:
- Zero intra-pair hazard.
- Elim + alloc in the same cycle.
- Dual issue to asymmetric ALUs.
- 2-retire with independent destinations.

### 2.7 Worked example: 2-wide with intra-pair dep

**Input**: `pd_buf = { ADD.L D1,D0 | SUB.L D0,D2 }`.  Slot-1 reads D0,
which slot-0 just wrote.

**Slot-0**: `arch_dst_0 = 0` (D0), alloc new phys `px`.
**Slot-1**: `arch_src_a_1 = 0` (reads D0), `arch_dst_1 = 2` (D2).
Normal `ratmap[0]` read would return `old_phys_of_D0` — stale.

**Intra-pair bypass**: `phys_src_a_1 = (has_dst_0 && arch_dst_0 ==
arch_src_a_1) ? alloc_phys_dst_0 : ratmap[arch_src_a_1]`.  Selects
`px` (slot-0's new alloc).  **Ready bit**: `src_a_rdy_1 = 0` (px is
brand-new).  Slot-1's iq_int entry waits for cdb wake on `px`.

**CCR intra-pair bypass**: slot-1's `ccr_src_tag = ccr_alloc_dst_0`
(slot-0's newly-allocated CCR tag).  `ccr_src_rdy = 0`; wake on CCR
CDB.

**Outcome**: both dispatch this cycle.  Slot-1 sits in iq_int for 1
cycle until slot-0's ALU fires.  Effective pair latency: same as
unfused serial execution — **Phase C's gain on intra-dep pairs is
dispatch-throughput, not execute-latency**.

### 2.8 Risks

- **Fusion × 2-wide interaction.**  Today fusion peeks slot-0 + slot-1
  bytes.  If slot-1 is a Bcc candidate but slot-0 is CMP, fusion fires
  (as today).  If slot-0 is CMP + slot-1 starts with an unrelated inst,
  fusion cannot fire — slot-1 still dispatches independently.  No
  regression.
- **Decode case-tree collisions.**  The genvar wrap must be coordinated
  with concurrent decode-adding agents (currently idle per §concurrent
  agents).  Cite `tracks/core.md:110-114` — grep uop_pkg.v for
  next-free slot BEFORE editing.
- **predecode.v pair-classifier timing.**  Pair-classifier must
  classify `{len_0, is_single_phase_0, is_single_phase_1}` in 1 LUT6
  level; else absorb 1 extra cycle of predecode latency (accepted —
  decode front-end is not on the 1-cycle ALU path).

---

## §3 Rename 2-wide

### 3.1 RAT 2 alloc/cycle — free-list and map update

**Today** (`rtl/core/rename/rat.v:163-177`): one `first_free` priority
encoder over 48-bit `free_bm`; one `ratmap[alloc_arch_dst] <= next_free`
update per cycle.

**2-wide**: add a second alloc port that sees `free_bm` with bit
`next_free_0` cleared, then picks `next_free_1` from the remainder.
Two cases:

1. **Both alloc_en asserted, both have_dst, alloc_arch_dst_0 !=
   alloc_arch_dst_1**: 2 regs popped, 2 map entries written, normal.
2. **alloc_arch_dst_0 == alloc_arch_dst_1** (same-cycle dual-write to
   same Dn): slot 1 wins — the slot-0 phys gets allocated but is
   immediately free-listed one cycle later on retire (slot-0's retire
   will see its own mapping is already gone from `ratmap`, so it
   correctly frees its phys via `phys_old` as tracked by the ROB).
   This is a **WAW-collapse** that actually works naturally as long as
   the ROB's `phys_old[slot_1]` is `phys_dst[slot_0]` (not the prior
   arch dst's phys).  Requires rename to hand-off phys_old between
   slots — §3.4 "intra-pair dependency bypass".

**Priority encoder timing**: `first_free(bm)` at 48-bit is ~2 LUT6
levels (chained find-lowest-set).  2-pop version = 3 levels (find 1st,
gate out, find 2nd).  At 200 MHz this is **~1.2 ns**, within budget.
Pipeline-alternative: lookahead by registering the 2nd `next_free`
one cycle ahead based on a steady-state draw rate — unnecessary, the 3
LUT levels close.

**Storage changes** (`rat.v:139-152`):
- `ratmap` stays 32 × 6 — now written from 2 lanes.  Handle same-arch
  collision at slot 1 precedence (slot-1 arch_dst dominates map write).
- `ref_count[0:47]` 6-bit per phys — already sized for ROB depth (32).
  2-wide alloc bumps up to +2 per cycle, still well under 6'd63.
  Update loop (`rat.v:233-248`) extends to accumulate 2 commits' deltas
  + 2 frees' deltas.  Hand-check: +2 / -2 fits 6 bits.
- `committed_busy[0:47]` — derived from ref_count>0; no logic change.

**Reset state** is unchanged.  **Flush semantics** (§9.1 risk): the
commit-then-flush mirror must extend to 2 slots of same-cycle commit
being replayed into the `ratmap ← crat` overwrite.  Cleanest: fold the
2 slot commits into `crat` first (combinational) and read the updated
`crat` into `ratmap`.  Cost: 2 × 5-bit muxes on `ratmap` write path.

### 3.2 CCR-RAT 2 alloc/cycle

`ccr_rat.v:56-198` has the same structure at 16 entries.  2-wide
changes are:

- `alloc_en` becomes 2-bit `{alloc_en_1, alloc_en_0}`.
- `alloc_dst_tag_0 / alloc_dst_tag_1` outputs, each 4-bit.
- `rat_tag` update: if both allocate, `rat_tag <= alloc_dst_tag_1`.
- **Collision**: both slots are flag-writers simultaneously — the
  older (slot 0) allocates first, slot 1 sees the updated free_bm.
  This means slot 1 reads `cur_tag = alloc_dst_tag_0` for its CCR
  source — the **intra-pair CCR dependency** bypass (see §3.4).
- **Free list**: 2 pops/cycle.  16-entry × 1-bit `free_bm` priority
  encoder is 2 LUT6 levels single-pop, 3 for dual.  **~0.8 ns**
  combinational.
- **Ref counting** in CCR-RAT is simpler than int-RAT: CCR has only 1
  arch register, so a new `commit_new_tag` always displaces a single
  `commit_old_tag`.  2-wide commit returns 2 old tags to free_bm.

Cost estimate (adds to `ccr_rat.v`):
| Item                         | LoC |
|------------------------------|-----|
| 2nd alloc port               | 30  |
| Priority encoder 2-pop       | 10  |
| 2-wide commit (free 2 olds)  | 15  |
| 2-wide flush rollback        | 20  |
| **Total**                    | **75** |

~60 LUTs / 0 FFs.  -0.3 ns WNS — well inside F1 budget.

### 3.3 RAT port count summary

| Port                       | Today | Phase C | Width     |
|----------------------------|-------|---------|-----------|
| src_a read                 |   1   |    2    | 5→6-bit   |
| src_b read                 |   1   |    2    | 5→6-bit   |
| alloc_en                   |   1   |    2    | 1-bit     |
| alloc_arch_dst             |   1   |    2    | 5-bit     |
| alloc_phys_dst             |   1   |    2    | 6-bit     |
| alloc_phys_old             |   1   |    2    | 6-bit     |
| elim_en                    |   1   |    2    | 1-bit     |
| elim_kind/arch_dst/phys_src|   1   |    2    | mixed     |
| free_phys (commit)         |   1   |    2    | 6-bit     |
| commit_arch/commit_phys    |   1   |    2    | mixed     |
| CDB snoop phys             |   2   |    4    | 6-bit     |
| CCR-CDB snoop              |   1   |    1    | 4-bit (shared) |

The src read path doubling is the cheapest (comb lookup). The alloc
pair + collision is the medium cost.  The commit 2-wide is tied to
ROB 2-retire (§4) and must land co-requisite.

### 3.4 Intra-pair dependency bypass

When slot-0 writes Dn and slot-1 reads Dn in the same decode cycle,
slot-1's RAT read (combinational from `ratmap[src]`) would return the
pre-alloc mapping — stale.  Fix: dedicated mux on `phys_src_*_1`:

```
phys_src_a_1 = (has_dst_0 && arch_dst_0 == arch_src_a_1) ?
               alloc_phys_dst_0 : ratmap[arch_src_a_1]
```

2 mux levels (arch_dst compare + 2:1 select) = **0.4 ns**.  Same for
`phys_src_b_1`.  Elim bypass follows the same pattern: if slot-0 was
ELIM_MOVE and slot-1's src_a == slot-0's arch_dst, then slot-1 gets
`elim_phys_dst_out_0` as its `phys_src_a_1` (not `alloc_phys_dst_0`).
Since `elim_phys_dst_out_0 == phys_of(elim_phys_src_a_0)`, slot-1
simply reads through the alias.

**Ready-bit intra-pair bypass**: if slot-0 allocates a brand-new phys
and slot-1 reads it, slot-1 sees `src_rdy_1 = 0` (new phys is unready).
That's fine; iq_int will wake slot-1 one cycle after slot-0's ALU
retires.  No special bypass needed.

Same applies to CCR intra-pair: slot-1's `ccr_src_tag` = slot-0's
`alloc_dst_tag_0` if slot-0 is a flag-writer.  Mux: **1 LUT level**.

### 3.5 Cost summary

| Item                        | LUTs | FFs | WNS Δ  | Days |
|-----------------------------|------|-----|--------|------|
| RAT 2-alloc + 4-read        |  400 |  50 | -0.4 ns| 3.0  |
| CCR-RAT 2-alloc             |   60 |  20 | -0.2 ns| 1.0  |
| Intra-pair bypass muxes     |   40 |   0 | -0.4 ns| 0.5  |
| Free-list ref-count 2-delta |   80 |   0 | -0.2 ns| 0.5  |
| Dispatch wrapper in m68k_core | 50|   0 | -0.1 ns| 0.3  |
| `tb/tb_rat.cpp` widening    |   —  |   — |   —    | 1.2  |
| **Total**                   |**630**|**70**|**-1.3 ns**|**6.5 days** |

---

### 3.6 RAT same-cycle WAR and WAW behaviour

**WAR (Write-After-Read)**: slot 0 reads Dn, slot 1 writes Dn.  Slot
0's read gets pre-alloc mapping (correct — it's older).  Slot 1's
write updates ratmap after slot 0's read is resolved (combinational
reads complete within the decode cycle; sequential write at posedge
commits the new mapping).  **No hazard** — natural ordering.

**WAW (Write-After-Write)**: slot 0 writes Dn, slot 1 writes Dn.
Both allocate.  Slot 0's phys is dead on arrival (no reader).  ROB
correctness: slot 0 retires first, frees its phys_old (= old Dn
phys).  Slot 1 retires second, frees its phys_old (= slot 0's phys,
the one just written).  Both phys regs return to free pool.  Net: 1
arch register update, 2 phys regs consumed/freed.  **Phys usage
inefficiency**: 1 wasted phys per WAW pair, but WAW is rare (<1% of
pairs) so acceptable.

**WAR + WAW combined**: slot 0 reads Dn then writes Dm; slot 1 reads
Dm (fresh from slot 0).  Intra-pair bypass (§3.4) handles the
Dm read.

### 3.7 Verification plan for rename 2-wide

Extend `tb/tb_rat.cpp` with:
- `dual_alloc_distinct`: 2 allocs to D0 and D1 same cycle.  Verify
  free_bm drops 2 bits, ratmap updates both.
- `dual_alloc_same_arch`: WAW.  Verify both phys consumed, commit
  frees correctly.
- `dual_alloc_collision_with_free`: same-cycle 2 alloc + 2 free.
  Verify ref_count arithmetic.
- `elim_then_dep_read`: slot 0 ELIM_MOVE D0→D1, slot 1 reads D1.
  Verify bypass fires.
- `elim_zero_then_dep`: slot 0 EOR Dn,Dn (ELIM_ZERO), slot 1 reads
  Dn.  Verify bypass to PHYS_ZERO_TAG.
- `flush_during_dual_alloc`: slot 1 is branch, slot 0 is dep.  Branch
  mispredict; verify both phys regs restored to free list.

Extend `tb/tb_ccr_rat.cpp` (if exists; create if not) with analogous
scenarios at CCR-RAT.

## §4 ROB 2 retire/cycle

### 4.1 Goal

Retire at most 2 μops per cycle out of ROB head.  Conditions to retire
the head+1 entry:

1. `e_valid[head_idx]` AND `e_valid[head_idx+1]`.
2. `e_complete[head_idx]` AND `e_complete[head_idx+1]`.
3. Head is NOT a branch mispredict (causes flush → aborts head+1).
4. Head is NOT an exception (same argument).
5. Head is NOT a store with pending `commit_store_en` (LSU drain must
   finish before head+1 retires — §4.3).
6. `head+1` is NOT a privileged-in-user-mode op that needs to be
   promoted to vec-8 (trap serialises after head retires).

### 4.2 Exception serialisation within a pair

**Case A**: head exception, head+1 anything.  Retire only head; head+1
aborts, its physical-dst is freed via the flush path.  **Detect
combinationally** in ROB output: `commit_en_1 = commit_en_0 &&
commit_complete_1 && !commit_exc_0 && !(commit_is_branch_0 &&
commit_mispredicted_0)`.  Already 3 LUT6 levels — cheap.

**Case B**: head OK, head+1 exception.  Retire head; head+1 fires
exception on its own next cycle (becomes the new head).  Two-way
retire must suppress head+1 retire this cycle, proceed with head
only.

**Case C**: head is branch mispredict.  Flush fires this cycle; head+1
is in-flight but wrong-path, gets squashed by the flush (just as
today's single-retire flushes head+1 via `flush_keep_tag = head_idx`).
2-wide retire in this case degenerates to 1-wide with flush.

**Case D**: head+1 is privileged-in-user-mode (`requires_supervisor &&
!arch_sr.S`).  Today commit converts it to vec-8 at retire.  2-wide
must check on each slot.  If head+1 is privileged and would trap,
retire only head this cycle.  Serialises; 1 cycle later, head+1
traps normally.

**Case E**: Both are normal ALU/LOAD ops.  Retire both.

### 4.3 Store / commit discipline

Per `tracks/core.md:58-65`, stores stay in `S_ST_BUF` until
`commit_store_en`.  The LSU's single-port AXI write drives 1
`commit_store_en` per cycle max.  If head+1 is also a store, it must
serialise — retire head (fires its commit_store_en), head+1 waits.
**Recommendation**: `commit_en_1 = ... && !(commit_is_store_0 &&
commit_is_store_1)`.  Bench delta: 2 adjacent stores rare outside
MOVEM; upper bound 5% of commit cycles serialise. Net gain still
dominates.

### 4.4 Concrete changes in rob.v

Approximate landing points in `rtl/core/rename/rob.v`:

- Lines 97-135 (commit outputs): add `commit_en_1`, `commit_valid_1`,
  `commit_complete_1`, and duplicated per-entry readout fields for
  `head_idx+1`.  Purely combinational muxing — no new storage.
- Lines 219-249 (head readout): wire `head_idx_p1 = head_idx + 5'd1`
  and extract all head-of-ROB fields for that entry as `commit_*_1`.
  Readout is 32:1 muxes over the 32-entry array — already exists as
  `head_idx` muxing, just duplicate for `head_idx_p1`.
- Lines 460-466 (commit pop): retire 2 entries if `commit_en_1`:
  ```
  if (commit_en_0 && commit_valid_0 && commit_complete_0) begin
      e_valid[head_idx] <= 1'b0;
      if (commit_en_1 && commit_valid_1 && commit_complete_1) begin
          e_valid[head_idx_p1] <= 1'b0;
          head_ptr <= head_ptr + 6'd2;
      end else begin
          head_ptr <= head_ptr + 6'd1;
      end
  end
  ```
- Wrong-path filter (`rob.v:261-293`): `query_tag0` doubled to
  `query_tag0 + query_tag1` to support two ALUs training the BPU
  in the same cycle.

### 4.5 Cost summary

| Item                            | LUTs | FFs | WNS Δ  | Days |
|---------------------------------|------|-----|--------|------|
| 2-wide head readout (combinational) |  180 |   0 | -0.1 ns| 1.0 |
| 2-wide pop + head_ptr update    |   40 |   0 | -0.1 ns| 0.5 |
| 2-wide wrong-path filter        |   60 |   0 | -0.2 ns| 0.5 |
| Exception serialisation gates   |   30 |   0 | -0.1 ns| 0.5 |
| Commit-side consumer extensions |  120 |  30 | -0.2 ns| 1.5 |
| `tb/tb_rob.cpp` extend          |   —  |   — |   —    | 1.0 |
| **Total**                       |**430**|**30**|**-0.7 ns**|**5 days** |

---

### 4.6 Commit.v consumer extension

`rtl/core/commit.v` drives `rob_pop`, `rat_free_en`, `rat_commit_en`,
`ccr_commit_en`, `bpu_update_*`, store commit, exception sequencer.
At `commit.v:573-1270` the main always block handles one retire per
cycle.  For 2-retire:

- `rob_pop` becomes 2-bit `rob_pop[1:0]` (unary: 01 = head, 11 =
  head+head+1).  rob.v's pop lane logic already exists for head, add
  head+1 handler.
- `rat_free_en`, `rat_free_phys`: doubled.  RAT already has ref-
  counted free (§3), so 2 free pulses per cycle decrement correctly.
- `rat_commit_en`, `rat_commit_arch`, `rat_commit_phys`: doubled.
- `ccr_commit_en`, `ccr_commit_new_tag`, `ccr_commit_old_tag`:
  doubled.
- `bpu_update_en`: if both retired μops are branches, train BPU with
  both.  Each BPU update is a single-port update today; need 2-port
  BPU update or serialise.  **Recommendation**: 1-port BPU update;
  if both are branches, train slot 0 this cycle, slot 1 next cycle.
  Trivially correct since the BPU is a hint, not correctness-
  critical.
- Store commit: §4.3 gates head+1 retire if both are stores.

LoC delta in commit.v: ~180-220 lines (duplicating lots of retire
sequencer case branches).  Timing impact: minor, commit.v is not on
the critical path.

### 4.7 FLUSH serialisation correctness

**Risk**: 2-retire fires, slot 0 is OK (normal ALU op), slot 1 is a
mispredicted branch.  Slot 1 triggers flush.  Timing:
- Cycle N: commit_en_0 = 1, commit_en_1 = 1, flush_en rises on the
  same cycle (from slot 1's mispredict compare).  `flush_keep_tag =
  head_idx_p1`.
- Cycle N+1: rob squashes entries with tag > head_idx_p1. rat_commit
  from slot 0 + slot 1 both get applied (they're committed, not
  flushed).
- RAT rollback from flush: `ratmap ← crat`.  crat was updated by both
  commits on cycle N (§rat.v rat_seq).  Flush reads the NEW crat.
  **Correct**.

**Edge case**: slot 0 is OK, slot 1 is an exception (e.g. divide-by-
zero detected at execute, cmpl_exc=1 in ROB).  Commit fires slot 0
normally.  Slot 1 doesn't retire; next cycle slot 1 is head, commit
sequences the exception (vec-5 handler).  **Handled by the `Case A`
rule in §4.2**.

### 4.8 BPU training asymmetry

Today (`m68k_core.v:1244-1247`): BPU update mux prefers exec-time over
commit-time.  At 2-wide retire, if both retiring μops are branches,
exec-time training for both already happened at their ALU retire
(cdb0/cdb1 fire 1 each).  Commit-time training fires for both too,
but the single BPU update port serialises them.

**Recommendation**: drop commit-time BPU training in Phase C.  Rely
on exec-time only (filtered for wrong-path).  Saves the serialisation.
`commit.v:1244-1247` tie-off becomes simpler.

**Risk**: exec-time wrong-path filter false-negatives.  Covered by
`rob.v:261-293`'s `query_older_mispred0`.  Phase C unchanged.

## §5 Second ALU (asymmetric: full + simple)

### 5.1 Sizing choice

**Recommendation: 1 full ALU + 1 simple ALU** (not 2 full).

| Option            | LUTs  | WNS delta | IPC lift | Notes                   |
|-------------------|-------|-----------|----------|-------------------------|
| 2 × full ALU      | +2400 | -0.8 ns   | +32%     | duplicate MUL/DIV/shift |
| 1 full + 1 simple | +1100 | -0.4 ns   | +28%     | shares MUL/DIV lane     |

The simple ALU handles: ADD, SUB, AND, OR, EOR, NOT, NEG, MOV, CMP,
TST, shifts by 1 (ASL #1, etc.), and branch resolve (all conditions).
That covers >80% of dynamic ALU μops per bench analysis.

**Excluded from simple ALU** (retained on full ALU):
- MULS.W / MULU.W / MULS.L / MULU.L (DSP58E2 + 2-4 cycle pipe)
- DIVS.W / DIVU.W / DIVS.L / DIVU.L (sequential FSM)
- Variable-count shifts (ASL Dn,Dm with runtime count)
- Bitfield ops (BFCHG, BFCLR, BFEXTS, BFEXTU, BFFFO, BFINS, BFSET,
  BFTST) — these share the barrel shifter DSP on full ALU
- CHK.W/CHK.L, CHK2, CMP2 (exception path, rare)
- Scc / TRAPcc (exception path, rare)
- BCD (PACK, UNPK, ABCD, SBCD, NBCD)
- CAS, TAS (atomic — not yet implemented)
- SYS ops (STOP, RESET, RTE-stub, MOVEC)

### 5.2 IQ-int 2-pick

Today (`rtl/core/issue/iq_int.v:187-197`): single `sel_idx` priority-
pick lowest-index ready entry.

**2-pick**: find 2 ready entries per cycle, one for each ALU.
Additional constraint: if only 1 ALU can handle the chosen μop (e.g.
MULS), route that μop to the full ALU; the simple ALU picks another
ready entry.

Algorithm:
```
1. Build sel_mask[8] as today.
2. Pick pick_0 = lowest-index ready.  Route pick_0 to ALU whose
   capabilities include pick_0's uop_op.
3. If pick_0 went to full ALU, or if pick_0's op is simple-eligible,
   pick pick_1 = lowest-index ready && idx != pick_0 && op is simple-
   eligible (if routing to simple) or any (if routing to full).
```

Timing: 2-pick path is 8:1 priority encoder × 2 = 3 LUT6 levels =
**1.2 ns**.  Plus ALU-capability classifier per entry (~6 bits of op
category, 1 LUT6) = **1.5 ns combinational**.  Acceptable.

### 5.3 Concrete changes

In `iq_int.v`:
- Duplicate every `iss_*` output port into `_0` / `_1`.
- Two sel_idx outputs; two drain paths.
- Per-entry `e_simple_eligible` reg set at dispatch based on
  `disp_uop_op` class.  Simple LUT: {ADD/SUB/AND/OR/EOR/NOT/NEG/MOV/CMP/
  TST/BR_BCC/BR_BRA/BR_DBCC/shifts-by-1} → 1, else 0.
- 2-pick priority encoder as above.

In `alu.v`:
- No changes to existing ALU (rename internally as `alu_full`).
- New `alu_simple.v` = `alu.v` with MUL/DIV/bitfield/variable-shift
  cases gated off (just a parameter or `localparam HAS_MULDIV = 0`).
  Saves ~700 LUTs vs the full ALU.

In `m68k_core.v`:
- Instantiate `u_alu_simple` alongside `u_alu`.
- Wire iq_int's 2nd issue port to it.
- Wire its outputs into a new `cdb2` (§7 below).
- `alu_mul_busy` only constrains the full ALU pick — decouple.

### 5.4 Cost summary

| Item                     | LUTs  | FFs | WNS Δ  | Days |
|--------------------------|-------|-----|--------|------|
| alu_simple.v (pruned)    | +700  | +80 | -0.3 ns| 2.0  |
| iq_int 2-pick logic      | +180  | +40 | -0.3 ns| 1.5  |
| Simple-eligibility LUT   | +50   |  +8 |  0     | 0.3  |
| m68k_core wiring         | +80   | +20 | -0.1 ns| 0.5  |
| `tb/tb_iq_int.cpp` extend|  —    |  —  |  —     | 1.0  |
| Directed test scenarios  |  —    |  —  |  —     | 0.5  |
| **Total**                |**1010**|**148**|**-0.7 ns**|**5.8 days** |

### 5.5 Expected IPC lift

- `bench_alu_parallel`: 6 μops/iter, 4 independent ADDs + SUB + BNE.
  At 2-wide + 2 ALUs, 6 μops consumed in 3 cycles.  Today 857/216 ≈
  0.25; projected **370 cyc / 216 = 0.58**.  **+130%**.
- `bench_ind_adds`: 5 μops/iter, 3 parallel ADDs + SUB + BNE.  Same
  pattern.  Today 854/216; projected **420 cyc / 216 = 0.51**.  **+100%**.
- `bench_dep_chain`: serial RAW.  No lift from 2nd ALU alone.  **0%**.

---

### 5.6 Simple-ALU opcode partition (detailed)

Concrete op partition based on `rtl/core/execute/alu.v` and
`rtl/core/decode/uop_pkg.v` op slots 0..63:

**Simple ALU (fast path, 1-cycle, no DSP usage beyond adder)**:
- `ALU_ADD` (0), `ALU_ADDX` (1), `ALU_SUB` (2), `ALU_SUBX` (3).
- `ALU_AND`, `ALU_OR`, `ALU_EOR`, `ALU_NOT`, `ALU_NEG`, `ALU_NEGX`.
- `ALU_MOV` (29), `ALU_CMP` (30), `ALU_TST` (31).
- `ALU_EXT` sign-extension ops.
- Single-bit shifts: `ALU_ASL_1`, `ALU_ASR_1`, `ALU_LSL_1`, `ALU_LSR_1`,
  `ALU_ROL_1`, `ALU_ROR_1`, `ALU_ROXL_1`, `ALU_ROXR_1` — the immediate-
  count-1 specialisations.
- `ALU_BTST` / `ALU_BCHG` / `ALU_BCLR` / `ALU_BSET` with immediate
  bit number (static bit test/set).
- `BR_BRA` (6'd0) with `is_branch_in=1`, `BR_BCC` (6'd2), `BR_DBCC`.
- `ALU_CMP_BR` / `ALU_TST_BR` / `ALU_SUBQ_BR` / `ALU_ADDQ_BR` (60-63)
  — fused branch μops, needing only 32-bit add/sub + cc eval.
- MOVEQ decoded form (sign-extended immediate into dst).

**Full ALU only (retained on alu.v)**:
- `ALU_MULU_W`, `ALU_MULS_W`, `ALU_MULU_L`, `ALU_MULS_L` — 2-4 cycle
  DSP cascade.
- `ALU_DIVU_W`, `ALU_DIVS_W`, `ALU_DIVU_L`, `ALU_DIVS_L` — 16-34
  cycle sequential FSM.
- Variable-count shifts: `ASL Dn,Dm` / `ASR Dn,Dm` / `LSL/LSR Dn,Dm` /
  ROL/ROR/ROXL/ROXR Dn,Dm.  Needs runtime barrel shifter (DSP58E2
  mode).
- Bitfield ops slots 55-59: `BFTST`, `BFCHG`, `BFCLR`, `BFSET`,
  `BF_PACK` (dynamic bitfield).
- BCD ops slots 48-50: `BCD_ADD`, `BCD_SUB`, `BCD_NEG`.
- Scc slot 51, TRAPcc 52, CHK2 53, CMP2 54.
- CHK.W / CHK.L exception checking.
- Shift-by-variable-count (Dn count).

Static estimate of "simple eligible" fraction across bench suite
(grep of assembly): **82% of ALU μops today are simple-eligible**.
On bench_alu_parallel, bench_ind_adds, bench_move_heavy, bench_
dep_chain: 100% simple-eligible.  On bench_fullpipe, bench_mixed_mem:
95%+ (LEA = ALU_ADD with An dst = simple-eligible).

The 18% non-simple are dominated by DIV.W / DIV.L / MUL.L / bitfield —
rare on hot loops but common in some Mac toolbox code.

### 5.7 ALU capability pre-computation

Rather than having iq_int's picker classify each entry's op in the
critical path, pre-compute `e_simple_eligible[k]` at dispatch time:

```verilog
// At dispatch into iq_int:
e_simple_eligible[free_idx] <= is_simple_op(disp_uop_op);
```

`is_simple_op()` is a 6-input combinational function over `uop_op`.
Single LUT6 per bit, 1 total LUT.

Pick logic then:
```
pick_0 = lowest-index with sel_mask[k] set.
pick_1 = lowest-index > pick_0 with sel_mask[k] && e_simple_eligible[k].
```

If `uop_op[pick_0]` is itself simple-eligible, route pick_0 → ALU_full
and pick_1 → ALU_simple.  If pick_0 is NOT simple-eligible (MUL/DIV/
bitfield), pick_0 → ALU_full, pick_1 → ALU_simple.  Effectively: ALU_full
always gets the older μop, ALU_simple picks a younger simple-eligible
one.  This avoids the "simple ALU idle while full ALU starves on a
simple op" antipattern.

## §6 F1 registered PRF read (landed C1)

Status: landed as `fe9f27d` with the F1 stage in `rtl/core/m68k_core.v`.
The post-C1 bench delta stayed within the planned ≤1% noise band; see
`docs/bench_baseline.md`.  The text below is kept as the design rationale
and implementation checklist for maintenance.

### 6.1 Motivation

Today at `m68k_core.v:1109-1112`, PRF reads are combinational:

```
wire [31:0] alu_a_val = prf[int_iss_psa];
wire [31:0] alu_b_val = prf[int_iss_psb];
```

Critical path: `iq_int.sel → iss_phys_src_a (registered output of
iq_int at line 103-104, max_fanout=48) → prf[] read (distributed RAM
lookup, ~0.8 ns at 48 entries) → alu.src_a → ALU case+adder → alu.result
→ CDB → iq_int wakeup`. Total: ~4.2 ns combinational after `iq_int`
register stage.

After F1: **register the PRF read output**, adding 1 cycle of latency
between iq_int and ALU but buying +1.8 ns WNS.  Back-to-back
dependent ops must be handled by a **CDB→src bypass** mux at ALU input
that picks the CDB-broadcasting value when the CDB tag matches the
consumer's src tag.

### 6.2 Design

Pipeline stage becomes:

```
[iq_int issue: register iss_psa/iss_psb/iss_tag/iss_op]
  ↓ 1 cycle (today)
[NEW F1 stage: prf[iss_psa] → src_a_reg; prf[iss_psb] → src_b_reg]
  ↓ 1 cycle (new)
[ALU execute: use src_{a,b}_reg, bypass-mux CDB if tag matches]
  ↓ 1 cycle
[CDB broadcast]
```

Back-to-back dep: producer ALU writes result at cycle N.  Consumer
dispatches at N-1, wakes at N (CDB snoop), sits in iq_int till
priority pick at N+1, issues at N+2.  F1 stage reads PRF at N+2;
PRF has been written at N via `cdb0_has_dst` gate at `m68k_core.v:1096`.
So F1 sees the correct value at N+2.  Back-to-back dep is 2 cycles
issue-to-issue today, stays 2 cycles after F1 (issue latency has
always been 2; F1 just moves where the register boundary lives).

**Net IPC cost**: zero (same issue-to-issue latency).  **Net Fmax
win**: +1.8 ns WNS.

### 6.3 CDB→RS forward mux (to avoid a 3-cycle back-to-back)

Without forward mux: if producer CDB broadcasts at cycle N and
consumer is in F1 at cycle N reading the PRF, the consumer reads the
OLD value (PRF hasn't been written yet — write happens next posedge).
Fix: at F1's source mux, compare `consumer_src_tag == cdb{0,1,2,3}_phys`
and select `cdb_data` over `prf[src]`.

```
src_a_f1 = (cdb0_en && cdb0_has_dst && cdb0_phys == iq_iss_psa_q) ? cdb0_data :
           (cdb1_en && cdb1_has_dst && cdb1_phys == iq_iss_psa_q) ? cdb1_data :
           (cdb2_en && cdb2_has_dst && cdb2_phys == iq_iss_psa_q) ? cdb2_data :
           (cdb3_en && cdb3_has_dst && cdb3_phys == iq_iss_psa_q) ? cdb3_data :
           prf[iq_iss_psa_q];
```

Mux depth: 4 × 6-bit compare + 5:1 select = **2.5 ns**.  At 200 MHz
this is the dominant F1 path — but it replaces a 3.8 ns combinational
path, so **net WNS +1.3 ns**.

Without the bypass, back-to-back dep ops would serialise at 3 cycles
issue-to-issue (wake → pick → F1-reads-old → re-schedule).  **With
the bypass: 2 cycles issue-to-issue**, matching today.

### 6.4 Concrete changes

- `m68k_core.v:1090-1112`: introduce `prf_read_stage` combinational
  module that registers `alu_a_val` / `alu_b_val` by 1 cycle.  Also
  register `int_iss_tag`, `int_iss_pdst`, `int_iss_op`, `int_iss_isbr`,
  `int_iss_uop_pc`, `int_iss_ccr_dst_tag`, `int_iss_ccr_has_dst`,
  `int_iss_immv`, `int_iss_imm`, `int_iss_frd`, `int_iss_fwr`,
  `int_iss_hd`.
- `iq_int.v:276-336`: ensure all iss_* registers already exist (they
  do — line 98-116).
- Wakeup must broadcast 1 cycle earlier relative to ALU execute: the
  CDB still fires at cycle N (ALU's cmpl0), but the consumer's F1 now
  reads prf at N+2 instead of N+1.  **Wakeup path unchanged** —
  iq_int still wakes at N, picks at N+1.
- New `cdb_bypass_mux` block in the F1 stage (combinational) per §6.3.

### 6.5 Cost summary

| Item                         | LUTs | FFs  | WNS Δ   | Days |
|------------------------------|------|------|---------|------|
| F1 pipeline registers (13 fields × 2 lanes) | 20 | 580 | +1.8 ns | 1.0 |
| CDB→src bypass mux           |  180 |    0 | -0.5 ns | 1.0  |
| Latency + bypass unit test   |   —  |   —  |   —     | 0.8  |
| Bench regression validation  |   —  |   —  |   —     | 0.2  |
| **Total**                    |**200**|**580**|**+1.3 ns**|**3 days** |

### 6.6 Why F1 first (sequencing rationale)

Every downstream Phase C lift (2nd ALU, 2-wide decode, 4 CDBs) spends
WNS.  Without F1, Phase C's cumulative -2.4 ns WNS would drop post-
route timing to a negative number on KU5P speed grade -2.  F1 alone
recovers +1.3 ns net — more than absorbs the other deltas.  **F1 is a
prerequisite, not a nice-to-have.**  Land it first.

---

### 6.7 Back-to-back dependent trace

Trace cycle-by-cycle for `ADD D1,D0; ADD D2,D0` (D0 is the chain):

**Today (no F1)**:
| Cyc | ADD1 state                 | ADD2 state                   |
|-----|----------------------------|-------------------------------|
| 0   | Decode+rename              | —                             |
| 1   | Dispatch → iq_int, wait    | Decode+rename                 |
| 2   | Issue, read PRF[psa1]+[psb1] → ALU | Dispatch → iq_int; snoop cdb0 at end of cyc |
| 3   | ALU executes, writes CDB   | Wake from cdb0, pick next cyc |
| 4   | —                          | Issue, read PRF[psa2]+[psb2] → ALU |
| 5   | —                          | ALU executes                  |

Dep-to-dep: 3 cycles.

**With F1 (registered PRF read)**:
| Cyc | ADD1 state                      | ADD2 state                      |
|-----|----------------------------------|----------------------------------|
| 0   | Decode+rename                    | —                                |
| 1   | Dispatch → iq_int                | Decode+rename                    |
| 2   | Issue → F1 stage, read PRF       | Dispatch → iq_int; snoop cdb0    |
| 3   | F1 latches src values; ALU exec  | Wake from cdb0; pick; issue → F1 |
| 4   | Write CDB                        | F1 stage — CDB→src bypass catches cdb0 fire at cyc 4 |
| 5   | —                                | ALU exec with bypassed src        |
| 6   | —                                | Write CDB                        |

Dep-to-dep: 3 cycles (unchanged).  CDB→src bypass at F1 ensures the
critical edge (ADD2 reads ADD1's freshly-broadcast result) isn't
lost to the PRF-write latency.  **F1 is latency-neutral for RAW
chains when the bypass fires.**

**Without the bypass** (if we skipped §6.3): ADD2's F1 stage reads
PRF[psa2] at cyc 4, but ADD1's CDB write hits PRF at the posedge of
cyc 5. ADD2 reads stale data. MUST re-schedule — would add 1 cycle.

**Contingency if bypass mux doesn't close timing**: accept +1 cycle
on back-to-back.  bench_dep_chain impact: 20 ADDs × 1 cycle = +20
cyc, 282→302, IPC 0.093.  Negligible impact on other benches (ILP
hides the gap).

### 6.8 PRF width and read port count

PRF is 48 × 32 bits.  Int read ports:
- Int ALU src_a, src_b (today: 1 reader pair).  **Phase C**: 2 ALUs
  × 2 srcs = 4 read ports.
- LSU base, data (1 reader pair today).  **Phase C unchanged**: 1
  LD port + 1 ST port would double if Phase B lands first.
- MOVEC-RD / arch-a7 / rob_phys_src_a (commit-time): 3 additional
  combinational read ports.
- Total Phase C: 4 (ALU) + 2 (LSU) + 3 (commit) = **9 read ports**.

Distributed-RAM 48×32 × 9R1W: ~500 LUT6 × 9 = too expensive.  Use
**FF replica with comb muxes** (see §7.3): 48 × 32 × 1 FF = 1536
FF per replica × 4 replicas = 6144 FF.  Read ports become direct
FF taps (0-depth, fastest possible).

PRF writes: today 1 (cdb0) + 1 (cdb1) + 1 (a7 writeback) = 3 write
ports coalesced into a single 4:1 mux at the FF input.  Phase C: 4
CDB + 1 a7 = 5 write ports → 5:1 mux, 2 LUT6 levels, 0.6 ns.  Inside
budget.

## §7 CDB bandwidth (2→4 CDBs)

### 7.1 Scaling math

Today: `cdb0` (ALU) + `cdb1` (LSU) + `cdb_ccr` (ALU flag-write).
Three CDB listeners across iq_int (8 entries × 2 int srcs × 1 CCR src
= 24 compares × 3 CDB = 72 compares/cycle) and iq_mem (8 × 2 × 2 CDB
= 32 compares) and RAT (48 × 2 CDB = 96 compares).  Total ≈ **200
compares/cycle**.

Phase C: `cdb0/1` (2 ALUs) + `cdb2` (LSU_LD) + `cdb3` (LSU_ST-cmpl or
MOVEC) = 4 int CDB buses.  CCR CDB stays separate (both ALUs share it
or split — recommendation: both ALUs output CCR, but only the full
ALU writes CCR CDB since simple ALU never does MUL/DIV/shifts — or
both share a mux).

**Simplest**: CCR CDB becomes **2 buses** (one per ALU); iq_int snoops
both.  Each CCR CDB is 4-bit tag + 1 enable. Minor fanout.

Per-entry listener count:
- iq_int (up to 16 entries × 2 int srcs × 2 CCR srcs): 16 × (4+2) = **96
  CDB snoop compares per cycle** (at 8 entries: 48).  Today: 24.
- iq_mem (8 × 2 srcs × 4 CDB): **64 compares**. Today: 32.
- RAT (48 × 4 CDB): **192 compares**. Today: 96.

**Total**: ~**352 compares/cycle** at 8-entry iq_int. At 16-entry
(Phase D scale-up): ~384.

### 7.2 Timing

Each compare = 1 LUT6 (6-bit equality).  OR-reducing per entry over 4
CDB = 2 LUT6 levels.  AND-reducing with `has_dst` gate = 1 more
level.  Full wake-up path: 4 LUT6 ≈ **1.6 ns**.  Plus routing on
per-CDB fanout (~0.5 ns).  Total ≈ **2.1 ns**, up from today's ~1.4
ns.  **Still well within 5 ns**.

**Mitigation when full Phase D adds 16-entry IQ**: per
`ipc_roadmap.md:852-858`, register the wake-flag per IQ entry.  1-cycle
wake latency, closes timing without losing meaningful IPC.

### 7.3 CDB priority & collision

At 4 CDBs fed from {ALU_full, ALU_simple, LSU_LD, LSU_ST}, they are
independent producers that cannot collide on a single cycle (each has
its own register-output driver).  No arbiter needed.

**PRF write port**: today 1W (at line 1096).  Phase C has 4 PRF writes
possible per cycle → 4W PRF.  Distributed RAM at 48 × 32-bit × 4W is
~4× LUT cost vs 1W; or **BRAM**: Xilinx RAMB36 in TDP mode gives 2
independent ports.  **2 BRAMs × TDP** = 4 write ports, 4 read ports,
each 32×48 = fits in 1 RAMB36 easily.  Cost: 4 BRAMs (2 "pairs" for
2-read-4-write via banking/replication).

**Recommendation**: replicate PRF 4× (LUT-based, simpler) and accept
the 4× LUT cost.  48 × 32 = 1536 FFs × 4 = 6144 FFs.  Or ~12 LUT6
per 32-bit lane × 48 entries × 4 replicas ≈ 2300 LUTs.  Cheap.
Alternative: 2× BRAM with bank split by phys-index even/odd.

### 7.4 Wake-flag registration mitigation

If WNS slips past 4.5 ns on the 4-CDB wake path:

```
// Today (combinational wake):
always @(*) begin
  nxt_psa_rdy[k] = e_psa_rdy[k] || (cdb0_en && cdb0_has_dst && cdb0_phys == e_psa[k]) || ...
end

// Mitigation: register the per-CDB hit flag
reg [3:0] pending_wake_hit [0:IQ_DEPTH-1];  // 1 bit per CDB
always @(posedge clk) begin
  for (k = 0; k < IQ_DEPTH; k = k + 1) begin
    pending_wake_hit[k][0] <= cdb0_en && cdb0_has_dst && cdb0_phys == e_psa[k];
    pending_wake_hit[k][1] <= cdb1_en && ...
    ...
  end
end
// nxt_psa_rdy uses the REGISTERED pending_wake_hit; 1-cycle delayed.
```

Cost: 1 cycle wake-up latency, which means 1 cycle extra between CDB
broadcast and dependent-issue on the critical RAW chain.  On
`bench_dep_chain`, that's a 20-cycle penalty (20 ADDs × 1). On
benches with abundant ILP it's ~0 — entries that wake earlier than
they pick pay no penalty.

**Decision**: keep combinational wake at 4 CDB if WNS closes.
Register only if post-route drops below 0.  Based on the 2.1 ns budget
above: stays combinational.

### 7.5 Cost summary

| Item                      | LUTs | FFs | WNS Δ   | Days |
|---------------------------|------|-----|---------|------|
| 4-CDB fabric + muxes      |  240 |  20 | -0.3 ns | 1.0  |
| PRF replication × 4       | 2300 | 5760| -0.2 ns | 1.5  |
| iq_int 4-CDB snoop logic  |  200 |  20 | -0.3 ns | 0.5  |
| iq_mem 4-CDB snoop logic  |  140 |  20 | -0.2 ns | 0.3  |
| RAT 4-CDB wake            |  180 |  40 | -0.2 ns | 0.5  |
| CCR-CDB 2-bus extension   |   80 |  10 | -0.1 ns | 0.3  |
| **Total**                 |**3140**|**5870**|**-1.3 ns**|**4.1 days** |

---

### 7.6 CDB ownership matrix (Phase C final)

| Bus     | Producer               | Consumers snooping                |
|---------|------------------------|-----------------------------------|
| cdb0    | ALU_full (int + MUL/DIV/shift) | iq_int, iq_mem, RAT, PRF  |
| cdb1    | ALU_simple (int)       | iq_int, iq_mem, RAT, PRF           |
| cdb2    | LSU LD                 | iq_int, iq_mem, RAT, PRF           |
| cdb3    | LSU ST-cmpl / MOVEC-RD | iq_int, iq_mem, RAT, PRF           |
| cdb_ccr_0 | ALU_full CCR writeback | iq_int, CCR-RAT (no iq_mem)      |
| cdb_ccr_1 | ALU_simple CCR wb     | iq_int, CCR-RAT                    |

Each consumer's snoop logic:
```
for (k = 0; k < IQ_DEPTH; k++) begin
    nxt_psa_rdy[k] = e_psa_rdy[k]
        || (cdb0_en && cdb0_has_dst && cdb0_phys == e_psa[k])
        || (cdb1_en && cdb1_has_dst && cdb1_phys == e_psa[k])
        || (cdb2_en && cdb2_has_dst && cdb2_phys == e_psa[k])
        || (cdb3_en && cdb3_has_dst && cdb3_phys == e_psa[k]);
    // same for psb, ccr (ccr only uses 2 ccr buses)
end
```

Gate levels: 4 compares (1 LUT6 each) + 4-input OR (1 LUT6) + 2-input
OR with old rdy (1 LUT6) = **3 LUT6 levels** combinationally per
entry.  Full wake-up path at iq_int.sel (selection depends on all
ready bits) adds another 2 levels for the priority encoder = **5 LUT6
levels total**.  **~2.0 ns**.  Well inside 5 ns.

### 7.7 CDB stall/arbitration — none needed

Because each producer has its own dedicated CDB, no arbitration.
Producer count (2 ALUs + 2 LSU + MOVEC) = 5; bus count = 4.  cdb3
is muxed between LSU ST-cmpl and MOVEC-RD — same pattern as today's
`m68k_core.v:1616-1620`.  Since MOVEC-RD is a commit-time broadcast
(commit.v drives) and LSU ST-cmpl is a retire-time fire, they're
already serialised (commit.v only fires MOVEC-RD when the retiring
head is a MOVEC op, and store ST-cmpl fires on LSU's FSM transition).
**No new arbiter**.

### 7.8 Verification

Extend `tb/tb_core_cdb.cpp` (create if needed) with:
- All 4 CDB buses firing same cycle, distinct phys tags.  Verify
  iq_int wakes all matching entries, PRF writes 4 slots.
- CDB → F1 bypass on back-to-back dep (§6.7 trace).
- CDB tag collision (2 buses fire with same phys tag — impossible by
  construction since PRF free list prevents it, but guard with
  assertion).
- Flush during multi-CDB fire: a wrong-path cdb1 write should not
  pollute PRF if flush_en is live.

## §8 Interaction with landed enablers

### 8.1 Move-elim #114 (ref-counted committed_busy)

**What it does**: at rename, MOVE Dn,Dm aliases ratmap[Dm] → phys(Dn)
instead of allocating.  ref_count[phys(Dn)] bumps by 1 so commit
doesn't free it until both Dn and Dm retire.

**Interaction with 2-wide rename**: a 2-wide dispatch that emits MOVE
D0,D1 (slot 0) + ADD D2,D1 (slot 1):
- Slot 0: ELIM_MOVE. ratmap[D1] ← phys_of_D0. ref_count[phys_of_D0]++.
- Slot 1: alloc new phys. `phys_src_a_1 = ratmap[D1]` — this must
  read the NEWLY-aliased phys_of_D0 (see §3.4 intra-pair bypass).

The bypass requires elim's `elim_phys_dst_out_0` to drive the
intra-pair src read mux.  Needs explicit wire in `rat.v` (currently
`rat.v:179-184` computes `elim_phys_dst_out` combinationally).
**Verify it's clocked-one-earlier-than-ratmap-write**: yes,
combinational.  Intra-pair bypass for ELIM is free.

**Ref_count saturation under 2-wide**: 2 elim-allocs/cycle × 32 ROB
entries = max 64 aliases → fits 6-bit ref_count (max 63 — borderline).
**Bump to 7-bit** if Phase D expands ROB to 64.  For Phase C (ROB
stays 32), 6-bit ref_count fits.

### 8.2 CMP+Bcc fusion #113

Fusion fires at `decode.v:416-433`; emits 1 μop consuming 4-8 bytes
(combined CMP + Bcc length).  **Under 2-wide decode**: when slot 0
fuses, slot 1 is forcibly idle that cycle (fusion covers both
opwords).  Fusion is always slot-0-exclusive.

**Interaction with 2nd ALU**: fused μop is single-dst (the Bcc
target), but it reads CCR + src_a + src_b (CMP operands) and writes
CCR + resolves branch direction.  Currently dispatched through
`iq_int` and the full ALU.  **Can the simple ALU handle fused ops?**
The fused `ALU_CMP_BR` op requires:
- Reading 2 int srcs + CCR src.
- Computing CMP (subtract + flags).
- Evaluating the cc code from the fresh flags.
- Producing branch direction + target.

No MUL/DIV/variable-shift. **Yes — simple ALU is capable.**  Route
fused branches through either ALU.

**Fusion yields on 2-wide**: before 2-wide decode, fusion was +12%
IPC on bench_cmp_branch (501→440).  After 2-wide, fusion fires less
often (slot-0 + slot-1 can both be independent insts), but the fused
op still saves 1 μop per fire.  Bench_cmp_branch: expected 72
committed / 250-290 cycles after Phase C = **0.25-0.30 IPC**.

### 8.3 Zero-idiom #114

Rename-time aliasing of MOVEQ #0 / SUB Dn,Dn / EOR Dn,Dn to
PHYS_ZERO_TAG (phys 16).  No ALU cycle consumed, but the CCR μop is
still emitted.  At 2-wide, slot-0 zero-idiom + slot-1 dependent read
of the zeroed reg: intra-pair bypass delivers PHYS_ZERO_TAG to
slot-1's src read.  PRF[16] is permanently 0 — slot-1 gets a ready
source immediately.  **Net**: 2-wide + zero-idiom = zero ALU cycles
for the MOVEQ #0 case, dependent reads launch next cycle. ~1
cycle/iter save on zero-idiom hot paths (Mac OS toolbox prologues).

### 8.4 BTB / BPU #112

BTB lookup at decode-time uses `pd_pc` for indexing.  At 2-wide
decode with two branches in a row (rare — 2% of pairs), only slot-0
can predict; slot-1 gets fall-through from slot-0's target.
**Acceptable**: back-to-back branches are uncommon.  Alternative:
second BTB port — cost ~200 LUTs, ~0% IPC gain.  Skip for Phase C.

### 8.5 RAS, CCR rename, L1I victim slot

All decode/rename-independent.  No interaction changes.

---

### 8.6 A2 move-elim + 2-wide: the bench_move_heavy unlock

The A2 roadmap entry (`bench_baseline.md:314-350`) explains why
move-elim shows zero cycle savings at 1-wide: the ALU is the
bottleneck, not rename.  Phase C's 2-wide rename + 2 ALUs is
specifically what unlocks A2's latent gain.

**Per-iter accounting on bench_move_heavy today** (post-A2,
pre-Phase-C):
- 11 MOVEs dispatched per iter.  At rename they're aliased (0 phys
  alloc), but each emits a CCR-writer μop (UOP_INT / ALU_MOV with
  `flags_wr = 5'b01111`).
- The 11 CCR μops serialise through 1-wide iq_int + 1 ALU.  3
  cycles each (dispatch, execute, CDB) → 33 cycles of ALU time per
  iter.
- Plus SUB + BNE = 5 more cycles.
- = ~38 cycles per iter × 50 iters + overhead = 2043 cycles (matches
  measured).

**Per-iter accounting with Phase C**:
- 11 MOVEs dispatched in (11/2) ≈ 6 cycles (2-wide decode + rename).
- 11 CCR μops executed by 2 ALUs: 6 cycles.
- SUB + BNE: 2 cycles (simple ALU + fused SUBQ+Bcc).
- = ~9 cycles per iter × 50 + overhead = 600 cycles.

A2 is the sine-qua-non for the 2043 → 600 cyc win.  **Without A2, the
ALU cycles on 11 serial MOVE ALU_MOVs would still be 11 × 1 cyc/ALU
/ 2 ALUs = 6 cycles — same as above, actually!** But A2 additionally
removes the dependency chain: D1 ← D0's phys, D2 ← D0's phys, ...,
D5 ← D0's phys.  Readers of D1..D5 don't chain through serial ALU
latency; they fan out from D0's single phys.

Fan-out collision with IQ size: all 5 dependent consumers at once
could overflow iq_int (8 entries).  **Expected stall**: 1-2 cycles
when dispatch bursts exceed IQ capacity.  Bounded.  Phase D (IQ
8→16) further relaxes.

### 8.7 Fusion collapse under 2-wide — throughput analysis

Today's fusion firings per bench (measured from committed delta):
- bench_cmp_branch: 171 → 72 committed = **99 fusions fired** (30
  iters × ~3 fusions each).
- bench_btb_loop: 206 → 110 committed = **96 fusions**.

Fusion saves **1 μop per fire**. At 1-wide today: 1 cycle saved per
fusion. At 2-wide: fusions still save 1 μop per fire, BUT the slot-1
emission is foregone on fusion cycles — so net cycle save is 0.5
cycles per fusion (fusion now replaces a "would-be 2-μop dispatch
cycle" with a "1-fused-μop dispatch cycle").

In other words, **fusion's 2-wide value is halved** — fusion
competes with the 2nd dispatch slot.  But fusion still wins on μop
count, CCR bandwidth, and wrong-path-squash amplitude.

**Net calculation for bench_cmp_branch**:
- Today: 440 cycles post-fusion.
- Phase C at 2-wide (no fusion): 3 μops/iter × 30 iters / 2-wide = 45
  cycles + epilogue 250 = 295 cycles.
- Phase C with fusion ALSO firing: 2 fused μops/iter × 30 / 2-wide = 30
  cycles + epilogue 250 = 280 cycles.

Fusion saves ~15 cycles on top of 2-wide, down from the pre-C 61-cycle
savings.  Still positive.  **Keep fusion**.

## §9 Proposed sequencing — ordered ticket list

### 9.1 Rank-ordered tickets

| # | Ticket             | Depends on     | Agent-days | LUTs   | FFs   | WNS Δ   | Risk |
|---|--------------------|----------------|------------|--------|-------|---------|------|
| 1 | **C1** F1 PRF-read + CDB bypass (**landed `fe9f27d`**) | none |  3.0 |  200 |  580 | +1.3 ns | L |
| 2 | **C2** CCR-RAT 2-alloc + rollback | C1          |  1.3       |   60   |   20  | -0.2 ns | L    |
| 3 | **C3** RAT 2-alloc / 4-read / 2-free + intra-pair bypass | C1, C2 | 4.0  | 520   | 50    | -1.0 ns | M    |
| 4 | **C4** ROB 2-retire + exc serialise | C3          |  3.5       |  430   |   30  | -0.7 ns | M    |
| 5 | **C5** IQ-int 2-pick + simple-eligibility | C1, C4 | 2.3 | 230 | 48    | -0.4 ns | M    |
| 6 | **C6** alu_simple.v + wiring + cdb2 | C5          |  3.0       |  780   |  100  | -0.4 ns | M    |
| 7 | **C7** 2-wide decode (pair classifier + slot1 tree) | C6       |  5.0       | 1380   |    0  | -0.5 ns | H    |
| 8 | **C8** 2-wide dispatch glue in m68k_core | C7       |  2.5       |  150   |   50  | -0.3 ns | M    |
| 9 | **C9** CDB 4-bus + PRF replication | C6           |  2.8       |  540   | 5870  | -0.5 ns | M    |
|10 | **C10** Fmax close-out synth pass  | C8, C9        |  1.5       |    0   |    0  | +0.5 ns | L    |
|   | **Total**                                            | — |  **≈29 days** | **4290** | **6748** | **-2.2 ns net**| |

Risk: L = low / M = medium / H = high (fan-out, timing, collision prone).

**Honest note**: the prompt asked for 20-25 days.  29 is the honest
count including unit-tb widening and directed tests (which consume
30% of days in core-track work).  Drop to 22 days if we skip some
tbs — but that violates `agent_policy.md` widening-tbs-as-features-
land.  Recommend 29 with proper testing.

### 9.2 Landing order rationale

- **C1 first** because every subsequent ticket spends WNS; this is now
  banked by `fe9f27d`, and the remaining tickets must not assume any
  additional free slack.
- **C2 before C3** because CCR-RAT is smaller (16 entries) and the
  2-port logic validates the approach before applying to int RAT
  (48 entries).
- **C3 before C4** because ROB 2-retire's commit lane must feed 2
  RAT free-ports + 2 CCR-RAT free-ports; those ports must exist.
- **C4 before C5** — IQ-int 2-pick is useless unless ROB can retire
  2/cycle; otherwise ROB head backpressures the IQ.
- **C5 before C6** — 2-pick is a prerequisite for 2nd ALU
  (otherwise the simple ALU idles).
- **C6 before C7** — 2-wide decode must have 2 consumers downstream
  or else it backpressures itself.
- **C7 before C8** — m68k_core glue consumes decode's new ports.
- **C9 concurrent with C6** — CDB fabric lands when 2nd ALU writes
  to cdb2 exist.  Can parallelise if two agents available.
- **C10 last** — Fmax close-out sweeps up all WNS debt.  Batched at
  phase boundary per sim-first policy.

### 9.3 Parallelisation opportunity

If 2 agents land concurrently:
- Agent A: C1 → C2 → C3 → C4 (12 days serialised).
- Agent B: C5 → C6 → C9 (8 days serialised, depends on C1 only for F1).

**Merge point**: C7 requires all upstream pieces. Agent A or B then
takes C7 (5 days).

**Wallclock**: ~17 days if 2 agents parallel, ~29 days serialised.

---

## §10 Risks

### 10.1 Bypass WNS (F1 CDB mux)

**Risk**: 4-CDB bypass mux at F1 is 2.5 ns; full F1-stage depth
(iss_psa mux → prf read → bypass mux → ALU src register) could
approach 4.5 ns, eroding F1's +1.3 ns net win.

**Mitigation**:
1. Register the 4 CDB tags + data at the F1 boundary (adds 128 FFs,
   absorbs 0 IPC since producer→consumer wake already takes 1 cycle).
2. Fall back to 2-CDB bypass (cdb0 + cdb2 only) if WNS slips; the
   dropped 2 CDBs still wake via the iq_int snoop path 1 cycle
   later.  IPC cost on `bench_dep_chain`: ~2 cycles per chain (<1%
   overall).
3. Split F1 into F1a (PRF read) + F1b (bypass mux); adds 1 issue-to-
   issue latency cycle.  Last-resort.

### 10.2 Rename-port conflict on same-cycle same-reg write

**Risk**: slot 0 and slot 1 both target `arch_dst = D0`.  Slot 1
should win (it's younger; slot 0's write is immediately dead).  But
both allocate phys regs, and slot 0's phys is now orphaned — nobody
reads it, but `rat.v:`ref_count bumps it by 1, waits for commit to
free it.

**Mitigation**: allocate slot 0's phys as usual; commit retires slot
0 normally (frees its phys_old = slot-1's prior D0 mapping; slot-1's
retire then frees slot-0's phys as phys_old). Properly chained
through ROB.  This is the "WAW-collapse" case in §3.1 — handled by
the `phys_old` hand-off between slots.

**Verification**: directed test in `tb/tb_rat.cpp` with two adjacent
MOVE.L #N,D0 insts in the same decode pair.

### 10.3 Verification complexity

Phase C adds 2-wide everywhere.  Fuzz coverage MUST widen:
- `tools/fuzz/gen_program.py` emit 2-wide candidates at 50%+ rate.
- New directed tests per ticket (C1-C10 each ≥ 2 corners).
- Musashi golden-ref stays the oracle.  Expect **5-10 MISMATCHes on
  first 200-seed run** — catch bugs early, don't try to land C7-C10
  before C1-C4 have 0 MISMATCH.

### 10.4 Exception serialisation

**Risk**: 2-wide retire with exception on slot 0 drops slot 1 μop
(abort).  Slot 1's phys regs must be freed — `rat_free_en_1` must
fire even though `commit_en_1 = 0`.  Subtle.

**Mitigation**: the flush path already handles this via
`committed_busy` rebuild.  An exception on slot 0 triggers
`flush_en`; slot 1's phys (never committed) has ref_count 0 after
flush rollback, so it's restored to free_bm correctly.  **Verify in
unit tb** — `tb/tb_rob.cpp` scenario: exception at head + valid
head+1, check RAT free list returns correct phys count.

### 10.5 Commit-side ordering (stores, MOVEC)

**Risk**: head+1 is a store; head is ALU.  Head retires → fires
commit_store_en_0 = 0 (head is ALU).  Head+1 retires → fires
commit_store_en_1 = 1.  LSU sees commit_store_en on cycle N+1
(registered in commit.v).  Today LSU expects exactly one
commit_store_en per cycle (FSM is `S_ST_BUF` → `S_ST_AW`).

**Mitigation**: `commit_store_en` stays 1-bit; bottleneck on stores
stays as §4.3 (head+1 store serialises).  No LSU change required
for Phase C.

### 10.6 Decode case-tree collision

**Risk**: Concurrent agents editing decode.v's case tree collide at
line ranges we duplicate in C7.  Mitigation per `tracks/core.md:110`:
always grep uop_pkg.v for next-free opcode slot before editing.  For
C7 specifically, the genvar wrap lets the case body stay textually
identical — minimises merge conflict surface.

### 10.7 BRAM-backed PRF alternative

**Risk**: replicating the PRF 4× (§7.3) adds 5870 FFs on the FF
resource.  Alternative: BRAM-back the PRF.  But BRAM has synchronous
read (1 cycle), which adds an issue-to-issue latency. F1 is already
paying that cycle — so BRAM-backed PRF is cost-neutral IF we fold
the BRAM read into F1.

**Recommendation**: evaluate at C9 land time.  For initial Phase C
land, replicate in FF (simpler).  Switch to 2× BRAM TDP at Phase D
if FF pressure becomes real.

### 10.8 Fmax regression across phase

**Risk**: cumulative -2.2 ns WNS across C1-C9 could push post-route
timing negative.  Today's baseline: +0.305 ns at 100 MHz per
`agent_policy.md:97`.  At 200 MHz target, WNS budget is ~+1 ns.
Phase C lands: +1.3 (C1) − 0.2 − 1.0 − 0.7 − 0.4 − 0.4 − 0.5 − 0.3 −
0.5 = **-2.7 ns cumulative** → **post-route WNS ~-1.7 ns at 200 MHz**
(fails).  **C10 Fmax close-out is MANDATORY** — sweeps +1.5 to +2.0
ns through retimes (register-what-you-can pattern per task #66).
Target: +0.3 ns post-C10.

**Contingency**: if C10 can't recover, accept 180 MHz target Fmax
(drop from 200 MHz).  Mac OS boot still 5× faster than Q840AV.
Document as "Phase C land at 180 MHz, re-close at 200 MHz in D."

---

## §11 Projected IPC per bench post-Phase C

Projections below use the per-bench loss attribution from
`docs/ipc_roadmap.md §2.2`.  Each bench's bottleneck is mapped to the
Phase C lever(s) that address it.

### 11.1 bench_ind_adds (today 854 cyc / 216 committed / IPC 0.253)

Loop body: 3 parallel `add.l` + `sub.l #1,%d6` + `bne` = 5 μops × 49
iters = 245 in-loop μops.  Per-iter today: ~17 cyc (12 dispatch +
5 in-loop).

Phase C changes:
- 2-wide decode + 2 ALUs: 5 μops dispatched in 3 cycles (slot-0+slot-1
  ADD+ADD, slot-0 ADD+slot-1 SUB, slot-0 BNE-fused slot-1 empty).
- Branch redirect bubble: still 1 cyc/iter after BTB hit.
- CCR-wake: 1-cycle (unchanged — CCR CDB still broadcast at cycle N).

Per-iter after Phase C: **~5 cyc/iter** (3 dispatch + 1 branch + 1
CCR).  49 × 5 = 245 + 100 prologue/epilogue = **345 cyc**.  **IPC =
216 / 345 = 0.63** (floor estimate).

If ROB-head backpressure shows up as expected (8 μops in flight × 2
ALUs): **peak IPC = 216 / 275 = 0.78** (with some prologue absorbed
by 2-wide).

**Bench_ind_adds Phase C: floor 0.63, peak 0.78.**  Crosses 1.0 in
Phase D (ROB 64 + IQ 16).

### 11.2 bench_move_heavy (today 2043 cyc / 611 committed / IPC 0.299)

Loop body: 11 MOVE.L (all elim-aliased by #114) + SUB + BNE = 13
macro-insts × 50 iters = 650 macro.  Of those, 11 MOVEs are renamed
(no ALU cycle) but still emit CCR-writer μops.

Today per-iter: **40 cyc** (11 serial CCR μops @ 3 cyc RAW = 33 +
SUB + BNE = 35; plus 5 cyc/iter dispatch width=1 overhead).

Phase C: 2-wide decode pairs MOVE+MOVE or MOVE+CCR-µop.  The 11 CCR
μops dispatch 2/cycle = 6 cycles.  Per-iter: **~9 cyc** (6 CCR + SUB +
BNE + 1 cycle CCR-wake delay on BNE).

50 × 9 = 450 + 150 prologue/epilogue = **~600 cyc**.  **IPC = 611 /
600 = 1.02**.  **CROSSES 1.0 ON THIS BENCH.**

Peak (with CDB-bypass perfect, 2-ALU, no stalls): **500 cyc**, **IPC
611 / 500 = 1.22**.

**Bench_move_heavy Phase C: floor 0.85, peak 1.25.**

### 11.3 bench_alu_parallel (today 857 cyc / 216 committed / IPC 0.252)

Loop body: 4 `add.l` + `sub.l` + `bne` = 6 μops × 40 iters = 240 μops.

Today per-iter: ~14 cyc (5 dispatch + 1 branch + 1 CCR + 7 other).

Phase C: 6 μops at 2 ALUs = 3 cyc execute.  Dispatch 2/cyc = 3 cyc.
Total per-iter: **~5 cyc**.

40 × 5 = 200 + 50 prologue/epilogue = **~250 cyc**.  **IPC = 216 /
250 = 0.86**.  Peak: 200 cyc → 1.08.  **CROSSES 1.0 AT PEAK.**

**Bench_alu_parallel Phase C: floor 0.75, peak 1.05.**

### 11.4 bench_cmp_branch (today 440 cyc / 72 committed / IPC 0.164 — post-fusion)

Loop body: fused `CMP+BNE` + `SUBQ+BNE` = 2 μops × 30 iters = 60
μops.  Low committed count is fusion collapsing 4 insts→2 μops.

Today per-iter: ~8 cyc (2 fused ops + CCR-wake between them + 2
branch bubbles).

Phase C: 2 fused ops dispatch in 1 cycle (both single-phase).  Both
resolve on ALU in 1 cycle.  Per-iter: **~4 cyc** (1 dispatch + 1
execute + 2 branch bubbles).

30 × 4 = 120 + 180 prologue/epilogue/cache = **~300 cyc**.  **IPC =
72 / 300 = 0.24**.  Peak: 240 cyc → 0.30.

**Bench_cmp_branch Phase C: floor 0.22, peak 0.30.**  Crosses 1.0
only at Phase D with loop buffer.

### 11.5 bench_fullpipe (today 1114 cyc / 318 committed / IPC 0.285)

Loop body: ALU+ALU+LEA+LOAD+SUB+CMP+STORE+BNE = 8 μops × 30 iters =
240 μops.  Bottleneck: single-port LSU (LOAD→STORE serialises).

Phase C addresses dispatch but NOT LSU port count.  Per-iter:
- Dispatch 8 μops @ 2-wide = 4 cyc.
- LSU serial LOAD→STORE = 12 cyc (unchanged from today — Phase B
  not in Phase C scope).
- ALU RAW (LOAD→SUB→CMP) chain = 3 cyc.
- Branch bubble + CCR = 2 cyc.

Per-iter: **~18 cyc** (LSU dominates).  30 × 18 = 540 + 300
prologue/epilogue/cache = **~840 cyc**.  **IPC = 318 / 840 = 0.38**.

**Bench_fullpipe Phase C: floor 0.35, peak 0.45.**  The big win
requires Phase B (dual-port LSU); Phase C makes a modest gain.

### 11.6 bench_mixed_mem (today 685 cyc / 125 committed / IPC 0.182)

Loop body: 5 μops × 20 iters + big cache flush epilogue.

Phase C gain: fold post-inc A0 + SUB D7 pair into 1 cycle (2-wide).
Per-iter 17→13 cyc.  **~620 cyc**.  **IPC 0.20**.

**Bench_mixed_mem Phase C: floor 0.18, peak 0.22.**  Mostly
cache-flush-dominated; Phase C limited help.

### 11.7 bench_dep_chain (today 282 cyc / 28 committed / IPC 0.099)

Serial RAW chain: 20 × ADD `D1,D0`.  No parallelism available.

Phase C gain: **zero** (fundamental ILP limit).  Stays ~282 cyc, IPC
0.099.

**Bench_dep_chain Phase C: 0.099 → 0.12** (prologue might shrink a
bit from 2-wide).  **Never crosses 1.0**; Phase D CDB→ALU bypass
helps but only to ~0.14.

### 11.8 bench_btb_loop (today 723 cyc / 110 committed / IPC 0.152)

Loop body: fused SUBQ+BNE × 100 iters = 100 μops (post-#113).

Per-iter today: ~6 cyc (dispatch 1 + execute 1 + branch bubble 1 +
CCR 1 + icache/fetch 2).

Phase C: dispatch already 1/cyc (single fused μop).  2-wide decode
doesn't help here — 1 μop/iter saturates slot 0.  Gain: maybe 1 cyc
per iter from reduced front-end pressure.

Per-iter **~4 cyc**.  100 × 4 = 400 + 100 prologue/epilogue = **500
cyc**.  **IPC = 110 / 500 = 0.22**.

**Bench_btb_loop Phase C: floor 0.20, peak 0.30.**  Dominated by
front-end; loop buffer (Phase D) unblocks further.

### 11.9 bench_btb_dbra (today 723 cyc / 110 committed / IPC 0.152)

Same shape as btb_loop but with DBRA (single μop).  Same projections.

**Bench_btb_dbra Phase C: floor 0.20, peak 0.30.**

### 11.10 Summary table

| Bench              | Today IPC | Phase C floor | Phase C peak |
|--------------------|----------:|--------------:|-------------:|
| bench_ind_adds     | 0.253     |  0.63         |  0.78        |
| bench_move_heavy   | 0.299     |  **1.02**     |  **1.25**    |
| bench_alu_parallel | 0.252     |  0.86         |  **1.08**    |
| bench_cmp_branch   | 0.164     |  0.22         |  0.30        |
| bench_fullpipe     | 0.285     |  0.35         |  0.45        |
| bench_mixed_mem    | 0.182     |  0.18         |  0.22        |
| bench_dep_chain    | 0.099     |  0.11         |  0.12        |
| bench_btb_loop     | 0.152     |  0.20         |  0.30        |
| bench_btb_dbra     | 0.152     |  0.20         |  0.30        |
| **Geomean**        | **0.196** |  **0.37**     |  **0.48**    |
| **Peak**           | **0.299** |  **1.02**     |  **1.25**    |

**Headline**: bench_move_heavy (and at peak, bench_alu_parallel)
cross the IPC=1.0 line at end of Phase C.  Most benches land at
0.3-0.5 and cross 1.0 in Phase D.

---

## §12 What Phase C does NOT cover

Explicitly deferred to Phase D (or later):

1. **Dual-port L1D + dual-LSU** (Phase B from `ipc_roadmap.md §3`).
   Biggest untouched bottleneck on `bench_fullpipe` and
   `bench_mixed_mem`.  Not in C because (a) L1D TDP BRAM requires
   dcache restructure, (b) store buffer CAM timing interacts with
   F1 bypass — best sequenced AFTER Phase C's F1 landing so WNS
   headroom is known.

2. **ROB 32→64, PRF 48→96, IQ-int 8→16** (Phase D scale-up).  Phase
   C keeps 32/48/8 to bound risk.  At 2-wide dispatch the 8-entry
   IQ saturates quickly on dispatch bursts — expected IPC cost ~5%.

3. **gshare BPU + IBT** (Phase D predictor upgrade).  BTB stays
   64-entry bimodal in Phase C.

4. **Loop buffer / μop cache** (Phase D).  The biggest win for
   bench_btb_loop and bench_btb_dbra, but adds decode complexity
   orthogonal to 2-wide.

5. **CDB → ALU same-cycle bypass** (Phase D).  F1 bypass is CDB →
   F1-stage (1 cycle late); same-cycle bypass would be CDB → ALU-
   src-mux with 0-cycle wake.  Costs ~0.8 ns WNS.  Deferred to
   Phase D.

6. **AGU→LSU bypass** (`ipc_roadmap.md §6`).  Post-incr A+disp
   forwarding.  3% IPC gain, not cost-effective in Phase C.

7. **MOVEM parallel crack** (`ipc_roadmap.md §7`).  Decode refactor
   for parallel STOREs.  8% win on Mac boot, 0% on benches.
   Phase D.

8. **3rd fusion wave** (MOVE(mem)+CMP, LEA+MOVE).  Phase D-adjacent
   decode expansion.

9. **SMC snoop to loop buffer / μop cache** — only relevant if loop
   buffer lands.

10. **2-wide FPU dispatch / iq_fp 2-pick** — FPU body not yet
    implemented; orthogonal.

11. **L2 cache (URAM-backed)** — separate track (memory hierarchy).

12. **3-wide decode or beyond** — hard ceiling at 2-wide per
    `ipc_roadmap.md App. B`.

13. **Trace cache / value prediction / SMT** — explicitly out per
    `ipc_roadmap.md App. B`.

---

## Appendix A — Reference cross-walk

| Phase C ticket | ipc_roadmap.md section | microarch.md section | Track/file       |
|----------------|------------------------|----------------------|------------------|
| C1 (F1)        | §6, §8 Phase C table   | "CCR rename plan"    | m68k_core.v:1109-1112 |
| C2 (CCR-RAT 2p)| §5.1                   | "CCR rename plan"    | ccr_rat.v:56-198 |
| C3 (RAT 2p)    | §5.1, §5.4             | "Register Rename"    | rat.v:70-317     |
| C4 (ROB 2ret)  | §5.3                   | "Reorder Buffer"     | rob.v:97-166,460 |
| C5 (IQ-int 2p) | §5.2                   | "Issue Queues"       | iq_int.v:187-197 |
| C6 (alu_simple)| §5.2, §5.4             | "Integer ALU (×2)"   | alu.v (new alu_simple.v) |
| C7 (decode 2w) | §5.1, §4.4, §9.3       | "Decode → μop"       | decode.v:471+    |
| C8 (glue)      | §5.2, §5.5             | —                    | m68k_core.v      |
| C9 (CDB 4-bus) | §5.4, §9.1             | "CDB has_dst"        | m68k_core.v:433-449 |
| C10 (Fmax)     | §9, Phase C exit       | "Fmax discipline"    | synth/ku5p.xdc   |

---

## Appendix B — Verification matrix (per-ticket gates)

Every ticket MUST pass:

1. `make test` — 161 PASS / 7 DEFER / 0 FAIL (baseline).
2. `make fuzz N=200` — 200/200 PASS / 0 MISMATCH.
3. `make lint` — 0 warnings.
4. Relevant unit tb extended with ≥2 new scenarios targeting the
   change.
5. Bench cycle counts measured on all 9 benches; regressions >3%
   blocked.
6. `docs/bench_baseline.md` updated with the delta.
7. Post-route synth only at phase boundaries (C10 at the end).

Ticket-specific extras:

- C1: directed test for back-to-back RAW dep chain (should stay
  2-cycle).
- C2: tb_ccr_rat 2-alloc-per-cycle scenarios + rollback.
- C3: tb_rat WAW-collapse + 2-src 4-port reads + elim intra-pair.
- C4: tb_rob 2-retire happy + exception-on-head + exception-on-tail
  + 2-stores-in-a-row serialisation.
- C5: tb_iq_int 2-pick + simple-only fallback + MUL-on-full-ALU
  routing.
- C6: simple-ALU op coverage (tb_alu widened) + cdb2 wiring tests.
- C7: tb_decode 2-wide corners (§2.5 list).
- C8: end-to-end dispatch+execute+retire of a 2-wide pair.
- C9: PRF 4-write same-cycle + 4-CDB wake+snoop coverage.
- C10: synth + impl gate; +0.3 ns WNS minimum.

---

## Appendix C — Effort summary

| Phase C gate      | Days | Cumulative |
|-------------------|------|------------|
| C1 land           |  3.0 |   3.0      |
| C2 land           |  1.3 |   4.3      |
| C3 land           |  4.0 |   8.3      |
| C4 land           |  3.5 |  11.8      |
| C5 land           |  2.3 |  14.1      |
| C6 land           |  3.0 |  17.1      |
| C7 land           |  5.0 |  22.1      |
| C8 land           |  2.5 |  24.6      |
| C9 land           |  2.8 |  27.4      |
| C10 close-out     |  1.5 |  **28.9**  |

Total ~**29 agent-days** serialised; ~**17 days wallclock** with two
agents parallel per §9.3.

---

## Appendix D — Open questions for PM review

1. **Accept 180 MHz post-C land Fmax?** Or require 200 MHz closure
   at every ticket boundary (will cost ~2× the time).  Recommend:
   land at 180 MHz, close to 200 MHz in a dedicated post-Phase C
   Fmax sprint (C10 → +5 days).

2. **Phase B concurrent or sequential?** `ipc_roadmap.md §8` places
   Phase B (dual-LSU) before Phase C.  This plan assumes Phase B is
   deferred — `bench_fullpipe` and `bench_mixed_mem` pay for that
   choice (IPC stays <0.5).  If Phase B lands first, Phase C's
   `bench_fullpipe` projection rises from 0.45 to 0.80.

3. **PRF implementation**: FF-replicated (fast, simple, +5870 FFs) or
   BRAM-TDP (requires F1 anyway, +2 BRAMs, -2300 LUTs).  Recommend
   FF-replicated for initial land, migrate to BRAM at Phase D.

4. **Fusion + 2-wide interaction**: fused op occupies slot 0
   exclusively.  An alternative is "fusion across 2-wide lanes" — a
   pair of insts that neither individually fuses but together would
   form a fused unit.  Cost: decode complexity.  Recommend: skip for
   Phase C (fusion is already landed in single-slot form).

5. **16-entry IQ-int now, or defer to Phase D?** 8 entries saturate
   at 2-wide in ~12 cycles.  Expanding to 16 now would cost
   ~0.5 days in C5 but add timing risk per `ipc_roadmap.md §9.2`.
   Recommend: stay 8 for Phase C, expand in D.

---

*End of phase_c_plan.md.*
