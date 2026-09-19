## decode_refactor_plan — m68k-ooo decoder restructure

> Design doc only.  No production RTL change accompanies this file.
> Scope is the files under `rtl/core/decode/` at `main @ c429576`.
>
> **Two goals, ranked:** (1) unlock 2-wide decode (phase C2) without a
> 2× replication of 22 K LoC of case tables; (2) collapse the matrix so
> each agent-sized decode change touches a narrow, reviewable surface.

---

## 1. Current-state audit

### 1.1 LoC / case-counts per file (`main @ c429576`)

| File                  |   LoC | Top-level if/elif | `case (uop_phase)` | `uop_type=` emissions |
|-----------------------|------:|------------------:|-------------------:|----------------------:|
| decode.v              |   878 | n/a (top casez)   |                  — | n/a (drives .vh)      |
| predecode.v           |   342 | n/a               |                  — | n/a                   |
| uop_pkg.v             |   373 | —                 |                  — | —                     |
| decode_ea.vh (helpers)|   142 | —                 |                  — | —                     |
| decode_ea_emit.vh     |   128 | —                 |                  — | 6 (2 tasks)           |
| decode_0000.vh        | 1 735 |                25 |                 19 | 96                    |
| decode_0100.vh        | 3 579 |                95 |                 56 | 219                   |
| decode_0101.vh        |   683 |                13 |                  7 | 39                    |
| decode_0110.vh        |    80 |                 3 |                  1 | 3                     |
| decode_0111.vh        |    38 |                 2 |                  0 | 2                     |
| decode_1000.vh        | 1 015 |                25 |                 19 | 64                    |
| decode_1001.vh        | 1 301 |                34 |                 27 | 84                    |
| decode_1011.vh        | 1 467 |                28 |                 25 | 93                    |
| decode_1100.vh        | 1 630 |                44 |                 28 | 102                   |
| decode_1101.vh        | 1 335 |                36 |                 28 | 85                    |
| decode_1110.vh        |   553 |                 8 |                  5 | 18                    |
| decode_1111.vh        |    29 |                 3 |                  0 | 2                     |
| decode_move_byte.vh   | 2 821 |                51 |                 53 | 178                   |
| decode_move_word.vh   | 2 890 |                53 |                 59 | 183                   |
| decode_move_long.vh   | 2 884 |                50 |                 52 | 176                   |
| **TOTAL**             |**23 903** |   **470 branches** |      **379 phase-cases** |     **1 351 μop-emits** |

Per-branch average: **~43 LoC**.  Per-emit average: **~15 LoC** (one
`uop_type=…; uop_op=…; has_*=…; arch_*=…; flags_*=…; imm_*=…;
len_bytes=…; last_phase=…;` block).

### 1.2 Matrix expansion — where the LoC go

MOVE sheets: 8 595 LoC / 154 branches = ~56 LoC/branch.  MOVE.L has
12 legal src EAs × 9 legal dst EAs = 108 combos; the file realises
~50 of them (ROM-frontier subset).  MOVE.B / MOVE.W are near-clones
(merge op + A7 byte-stride variations).

ALU families (`decode_1001/1011/1100/1101` = 5 733 LoC): top-level
discriminator on `op[8:6]` (direction+size), nested on `op_mode_lo`
(src EA class).  Each leaf differs only in `uop_op`, `flags_wr`,
presence of writeback phase — otherwise identical shape.

### 1.3 Genuine-semantic vs. EA-boilerplate split

Hand-tagging every `uop_type=` emission site across all .vh files
(1 351 total):

| Kind of emit                        | Count | % of emits |
|-------------------------------------|------:|-----------:|
| Core ALU/move semantics (the op)    |   ~360 |       27 % |
| EA-load crack phases (mem src read) |   ~420 |       31 % |
| EA-store crack phases (mem dst wr)  |   ~210 |       15 % |
| EA writeback (postinc/predec An)    |   ~130 |       10 % |
| Exception / illegal / privilege     |    ~60 |        4 % |
| Genuine multi-μop ISA cracks        |   ~110 |        8 % |
| Fusion / elim / peek-ahead          |    ~60 |        5 % |

**~56 % of all emits are EA-plumbing** — semantically identical
per-EA regardless of parent op (ADD vs CMP vs MOVE.L).

### 1.4 Verdict

~12 K of the 22 K LoC is EA boilerplate re-materialised per
`(instruction × EA mode)` cell.  A shared EA crack-emitter consumed
twice (src + dst) subsumes it.  Residual per-instruction semantics is
~360 distinct opcode points.  `decode_ea.vh` (10 functions, 142 LoC)
already covers legality + ext-word length; extending with a
crack-emission task is the straight-forward next step.

---

## 2. Proposed architecture

### 2.1 Block diagram

```
               ┌─────────────────┐
  pd_buf ─────>│  opword classifier     │─> sem_tbl_hit (idx)
               │  (one casez over op    │    ↓
               │   bits[15:6])          │  ┌─────────────────────┐
               └─────────────────┘       │ instruction semantics  │
                                         │ table (indexed)        │
                                         │ fields: uop_op, size,  │
                                         │ flags_wr, flags_rd,    │
                                         │ has_src/has_dst shape, │
                                         │ crack_kind, exc_vec,   │
                                         │ requires_supervisor    │
                                         └──────────┬────────────┘
                                                    │
  pd_buf bits[5:0] src-EA field ──┐                 │
                                   ├─> EA-decode ───┤
  op[11:6]         dst-EA field ──┘  (2× inst)      │
                                                    ▼
                                     ┌──────────────────────────┐
                                     │ μop-assembly FSM          │
                                     │  - drives uop_phase →    │
                                     │    {uop_type, arch_*,    │
                                     │     imm, is_load/store,   │
                                     │     len_bytes, last_phase}│
                                     └──────────────────────────┘
                                                    │
                                                    ▼
                                          uop bundle → rename
```

### 2.2 Component budget (estimated LoC)

| Component                          |   LoC | Notes                                       |
|------------------------------------|------:|---------------------------------------------|
| `decode.v` top shell (defaults, exc/fault handling, uop_phase reg, rn_ready glue) |   500 | Shrinks from 878 (lifts the fusion peek + case defaults only). |
| `inst_semantics_tbl.v` (flat lookup) | ~1 600 | One row per *logical* instruction (MOVE.L, CMP, MOVEM.L, CHK.L, BFINS-dynamic, …).  ~220 rows × ~7 LoC.  Implemented as a big casez over the sem-discriminator bits; one-hot to fields. |
| `decode_ea.vh` (expanded helpers)  |   300 | Add `decode_ea_is_dn`, `decode_ea_is_an`, `decode_ea_needs_load_phase`, `decode_ea_needs_store_phase`, `decode_ea_postinc_delta`, `decode_ea_predec_delta`. |
| `ea_crack_emitter.v` (task, src side) |   400 | Emits {LOAD (to TMP1), An postinc/predec writeback} phases for all 12 src modes.  Single combinational block indexed by `(mode, reg, size)` → `{phase_id, fields}`. |
| `ea_crack_emitter.v` (dst side)    |   400 | Mirror for dst-side {AGU (to TMP2), STORE}.  Shared with src by a `side` flag for cheap reuse. |
| `uop_assembler.v` (combine sem + EA) |   300 | Small FSM: {src-load-phases} → {op phase} → {dst-store-phase} → {last_phase=1}.  Drives `last_phase` and `uop_phase` advance. |
| Multi-μop instructions kept inline in decode.v (MOVEM kth-bit scan, BFINS dynamic shift, CHK2 3-phase, BSR 2-phase, RTE) | ~400 | Explicit opt-outs from the matrix; these are already ~8 % of emits and do not fit the `semantics × EA` shape. |
| Fusion peek-ahead (unchanged)      |   180 | Already lifted into decode.v lines 436-590. |
| **Total**                          |**~4 080** |                                         |

**Headline: ~22 K → ~4 K LoC (~5.4× shrink).**  MOVEM / BFINS / BCD /
MMU / MUL.L stay hand-rolled (by design, see §5).  Keeping those
~800 LoC of residuals inline → **~4.9 K net**.

### 2.3 μop-assembly pseudocode

```
// One macro-inst = (sem_row, src_ea, dst_ea).
// Phase plan = concat(src_load_phases, op_phase, dst_store_phases).
// Each phase is a pre-baked struct returned by ea_crack_emitter.

@posedge clk: uop_phase ← next_phase;

always @* begin
    case (phase_pos(uop_phase, sem_row, src_ea, dst_ea))
        PHASE_SRC_LOAD_0: drive_from(ea_crack_emitter(
                                    .side=SRC, .mode=src_ea.mode,
                                    .size=sem_row.size, .phase=0));
        PHASE_SRC_POSTINC: drive_from(ea_writeback(src_ea, sem_row.size));
        PHASE_OP:         drive_from(sem_row); // the ALU/ROB-visible op
        PHASE_DST_STORE:  drive_from(ea_crack_emitter(
                                    .side=DST, ...));
        PHASE_DST_POSTINC:drive_from(ea_writeback(dst_ea, sem_row.size));
    endcase
    last_phase = (phase_pos == LAST);
end
```

An instruction's `phase_plan` is a compile-time table (`{needs_src_load
[bool], needs_src_wb [bool], needs_dst_store [bool], needs_dst_wb
[bool]}`) bolted to the semantics row.  The `uop_phase` stepper walks
that plan.  No per-instruction `case (uop_phase)` anywhere in the
common path — only in the explicit multi-μop ISA cracks (MOVEM,
BFINS, BSR, CHK2, RTE).

### 2.4 Module signatures

```verilog
module inst_semantics_tbl (
    input  wire [15:0] opword,
    input  wire [15:0] ext1,           // for bitfield / dynamic-shift
    output reg  [7:0]  sem_idx,        // 1-hot row id (for downstream mux)
    output reg  [2:0]  uop_type,
    output reg  [5:0]  uop_op,
    output reg  [1:0]  uop_size,
    output reg  [4:0]  flags_wr,
    output reg  [4:0]  flags_rd,
    output reg  [1:0]  shape,          // {reg_only, reg_mem, mem_mem, cracked}
    output reg  [2:0]  crack_kind,     // {NONE, MOVEM, BFINS_DYN, BSR,
                                       //  CHK2, RTE, PMMU_OP}
    output reg         requires_supervisor,
    output reg  [7:0]  static_exc_vec, // 0 = no static exception
    output reg  [1:0]  elim_kind
);

module ea_decode (
    input  wire [2:0]  mode,
    input  wire [2:0]  reg_f,
    input  wire [1:0]  size,
    input  wire [15:0] ext,
    input  wire        is_src,         // legality side
    output reg         is_legal,
    output reg  [3:0]  ext_words,
    output reg  [3:0]  stride_bytes,
    output reg         needs_load,
    output reg         needs_store,
    output reg         needs_postinc_wb,
    output reg         needs_predec_wb,
    output reg         is_dn_direct,
    output reg         is_an_direct,
    output reg         is_immediate,
    output reg         is_pc_relative,
    output reg         is_memory
);

module ea_crack_emitter (
    input  wire [4:0]  phase_id,
    input  wire [1:0]  side,           // SRC / DST
    input  wire [2:0]  mode, [2:0] reg_f, [1:0] size,
    input  wire [31:0] disp,           // from ext words
    // outputs mirror decode.v uop fields for that phase only
    ...
);
```

### 2.5 Timing sanity

Decode→rename today ≤ 3.5 ns (CLAUDE.md budget; F1 landed at 197 MHz
Fmax).  The decomposed layout is strictly shallower: nested
`if (op[8:6]==3'b010 && op_mode_lo==3'b000)` chains flatten to one
one-hot table lookup + 2 parallel EA decoders + one mux layer.
Estimate: -0.3 to -0.5 ns on the decode path (frees slack for §3).

---

## 3. 2-wide-decode feasibility

Q1: **Dual-port the semantics table?**  Yes — 220 rows × ~40 bits.
Combinational LUT-RAM cheaper than BRAM at this size.  2 read ports
= 2 instances (~180 LUTs × 2 = 360; <0.2 % KU5P).

Q2: **EA decoder parallelism?**  Trivially yes.  `ea_decode` is pure
comb; instantiate 4 copies (slot-{0,1} × {src,dst}), <50 LUTs each.

Q3: **Predecode window for 2 opwords?**  Partially there already.
`predecode.v:49-100` already computes `pd_inst0_len`, `pd_inst1_len`,
`pd_inst1_off` and instantiates `inst_length_lut` for both opwords.
decode.v currently consumes only slot 0 (`pd_buf[127:112]` as `op`).
The 2-wide work is: widen decode.v's interface to
`{op_0, op_1, ext_0, ext_1, pd_pc_0, pd_pc_1, valid_0, valid_1}` and
add `pd_consumed = len_0 + (slot1_used ? len_1 : 0)` — aligned with
`phase_c_plan §2.2` port-duplication budget (~300 LoC).

Q4: **Which instructions force serial crack?**  `crack_kind` field
gates it: slot 1 suppressed when either slot's `crack_kind != NONE`.

| crack_kind   | Instructions                       | Dynamic freq (bench avg) |
|--------------|------------------------------------|-------------------------:|
| MOVEM        | MOVEM.{W,L} <list>,EA / EA,<list>  | ~2 %                     |
| BFINS_DYN    | BFINS dynamic-offset-or-width      | <0.1 %                   |
| BSR          | BSR + displacement variants        | ~1 %                     |
| CHK2/CMP2    | CHK2 / CMP2 all modes              | <0.1 %                   |
| RTE          | RTE (+ exception return mux)       | <0.1 %                   |
| MUL.L/DIV.L  | 64-bit mul/div (dual-dst crack)    | <0.5 %                   |
| PMMU_OP      | PMOVE/PTEST/PFLUSH                 | <0.1 %                   |

Cumulative slot-1 suppression ≤ 4 % of dynamic μops.  IPC tax ≤ 4 %
of the 2-wide gain — already budgeted in `phase_c_plan §2.3`.

Q5: **Do EA cracks force serial?**  No — an EA-LOAD phase is just
another μop in the plan; slot 1 decodes in parallel.  Only
*phase-sequenced* cracks (MOVEM kth-bit scan, BFINS dyn) force slot-1
suppression.

**Verdict**: 2-wide is feasible **after the refactor**, and the
refactor is the enabler.  A 2×-clone of today's 22 K LoC is
indefensible; a 2×-instantiation of a 4 K LoC decoder is routine.
Replicating in-place would hit ~44 K LoC and push WNS ~-1.0 ns
because the nested `if` chains double their logic depth.

### 3.1 Crack-slot quota fallback

If suppression proves noisy, an alternative is a *decode-then-crack*
2nd pipeline stage: decode emits abstract μops (1/macro-inst) and a
separate stage expands phases.  Cost: +1 cycle decode latency
(mispredict window grows).  Phase-D-or-later upgrade; defer.

---

## 4. Migration plan

Five stages, gate-by-gate.  Each stage's gate is `make test` 188
PASS (current baseline) and `make fuzz N=200` 0 architectural
mismatch against Musashi.

### Stage A — Build new engine side-by-side

- Add `inst_semantics_tbl.v`, `ea_decode.v`, `ea_crack_emitter.v`,
  `uop_assembler.v` as new modules emitting a shadow `new_uop_*` stream.
- Add sim-only cross-check in `tb/tb_decode.cpp`:
  `assert(new_uop_* == uop_*)` every valid clock.
- Gate: cross-check green on `make test` + `make fuzz N=200`.
- Risk: edge-case divergence (A7 byte stride, PC-rel full-ext,
  MOVEM an-in-list).  Expected; that's what we're about to harvest.
- **3 agent-sessions**.

### Stage B — Migrate decode_0111.vh (MOVEQ, 38 LoC)

Route MOVEQ through the new engine only (old path `ifdef`-gated).
Gate: all MOVEQ tests + fuzz pass.  Risk: near-zero.  **1 session.**

### Stage C — Migrate MOVE.{B,W,L} (8 595 LoC, the headline)

All three sheets share one MOVE sem-row (`uop_size` + merge op
vary).  Phase plan = src-LOAD → op → dst-STORE + postinc/predec
writebacks.  Gate: tb-rom-boot frontier non-regression (ROM hammers
MOVE.L in every EA mode) + ~30 MOVE-specific `tb/tests/asm/`
directed tests.  Risk hotspots: A7 byte stride, MOVEA.L CCR suppress,
mem-to-mem both-side phase plans.  **5–7 sessions.**

### Stage D — Rest of the families (LoC reclaim × risk order)

| # | File | LoC | Sessions | Notes |
|---|------|----:|---------:|-------|
| 1 | decode_1101.vh (ADD)        | 1 335 | 2 | Cleanest sem×EA |
| 2 | decode_1001.vh (SUB)        | 1 301 | 1 | Clone of ADD |
| 3 | decode_1011.vh (CMP/EOR)    | 1 467 | 1 | |
| 4 | decode_1100.vh (AND/ABCD/EXG) | 1 630 | 2 | EXG stays inline |
| 5 | decode_1000.vh (OR/DIVU)    | 1 015 | 1 | |
| 6 | decode_0000.vh (bit/imm/MOVEP/MOVES) | 1 735 | 2 | MOVEP/MOVES opt-out |
| 7 | decode_0101.vh (ADDQ/Scc/DBcc) | 683 | 1 | Scc/DBcc/TRAPcc inline |
| 8 | decode_0110.vh (Bcc/BRA/BSR)  | 80 | 0.5 | BSR inline |
| 9 | decode_1110.vh (shifts)     | 553 | 1 | |
| 10| decode_0100.vh (misc)       | 3 579 | 3–4 | ~1 500 LoC migrates; rest (MOVEM, CHK, MUL.L, DIV.L, RTE, MOVEC) opts out |
| 11| decode_1111.vh (F-line)     | 29 | 0 | Not matrix-shaped |

Per-stage gate: `make test` ≥ 188 PASS, `make fuzz N=200` 0 arch
mismatch, widening commit in `gen_program.py` if coverage was narrow.

### Stage E — Remove old decoder

Delete `decode_????.vh` + `decode_move_*.vh`.  Collapse `decode.v`
from 878 → ~500 LoC (top shell + exc + fault override).  Final gate:
test count ≥ 188, fuzz 0 mismatch, lint 0, ROM frontier ≥.
**1 session.**

**Cumulative effort: ~22 agent-sessions.**  ~3× the raw 2-wide cost,
recouped on the first 2 post-refactor ISA additions (each now ~30 LoC
of sem-row vs today's ~500 LoC of matrix).

---

## 5. Risks and non-obvious pitfalls

These are the tables/branches that do NOT decompose as (semantics × EA)
and must stay hand-rolled:

1. **MOVEM popcount-scan** (`decode_0100.vh:2890+`) — variable phase
   count = popcount + 1.  Keep inline under `crack_kind=MOVEM`.
2. **BSR/RTS 2-μop cracks** — STORE phase touches A7 specifically,
   crosses the sem-row boundary.  `crack_kind=BSR`.
3. **BFINS dynamic width/offset** — variable mask-shift phases.
   `crack_kind=BFINS_DYN`.  Static BFINS forms decompose fine.
4. **BCD (ABCD/SBCD/NBCD)** — implicit An-predec addressing doesn't
   fit `op_mode_lo`.  Add `shape=BCD_AN` sem-row flag that invokes
   the implicit predec-load pattern (~50 LoC special case).
5. **A7 byte stride** — handled in `decode_ea_stride_bytes:51-62`.
   Port verbatim; no shape change.
6. **CAS / CAS2 / TAS** — CAS fits with `shape=atomic_rmw`; CAS2 is a
   2-address RMW → `crack_kind=CAS2`; TAS fits normally.
7. **MUL.L / DIV.L 64-bit forms (SZ=1)** — 2nd dst, blocked on task
   #79 (dual-dst-PRF).  Sem-row differentiates SZ=0 (matrix) from
   SZ=1 (inline crack_kind) until #79 lands.
8. **Fusion peek-ahead** (CMP/SUBQ/TST+Bcc, task #113) — lives above
   the new engine in decode.v; its `pd_consumed` override takes
   precedence.  Fused ops are reg-only, no EA plumbing.
9. **Decode-time exceptions** (A/F-line, TRAP #n, ILLEGAL default) at
   `decode.v:751-823` — stays in top shell, unchanged.
10. **pd_fault / pd_next_fault** override (`decode.v:835-864`) —
    lives above sem path, unchanged.
11. **MOVEC / MOVES / PMOVE / PFLUSH / PTEST** — privileged,
    single-μop, Dn/An-direct.  Fit in sem-table with
    `requires_supervisor=1`.  No opt-out needed.

### Coverage traps

- **PC-relative source** (`110/111` + reg_f `010`/`011`): emitter must
  substitute `pc_at_ext + disp` as LOAD base.  Today inlined
  per-family; unify or re-verify during migration.
- **32-bit Bcc disp** (op[7:0]==0xFF): fusion today rejects it
  (`decode.v:494`); preserve that.
- **Absolute (xxx).W/.L**: sem-row must set `is_abs=1` and pin
  `phys_base = PHYS_ZERO_TAG` (`decode.v:91` + m68k_core.v wiring).

---

## 6. Fuzz / regression widening

1. **Per-addressing-mode matrix probe** — extend `tb/tb_decode.cpp`
   with a per-family sweep: for each legal `(mode, reg_f, size)`,
   assert the emitted μop stream matches a golden vector.  Target
   ~2 900 cases (12 modes × 8 regs × 3 sizes × ~10 families), auto-
   generated from a new `tb/tests/decode_matrix_gen.py`.
2. **Widen `tools/fuzz/gen_program.py` EA-mode mix** — lift (d16,An),
   (d8,An,Xn), and (xxx).W from <5 % each to 15 % each.  Today's
   generator skews toward reg-only; this exercises the new emitter.
3. **MOVEM list size 4–6** (already landed; do not regress).
4. **Old↔new cross-check** during stages A–D: bit-exact field
   agreement; any divergence stops the refactor.
5. **ROM cold-boot frontier** (tb-rom-boot) — highest-signal gate.
   Every C+D commit non-regresses or bumps the frontier.
6. **Per-family directed corner tests** — per the "widen tbs as
   features land" policy, each migrated family lands at least one
   new `.s` test on a corner the agent almost got wrong.

---

## 7. "No refactor needed" check

Not this case.  §1.3: 56 % EA-boilerplate share.  §1.1: ~43 LoC/
branch despite helpers already landed.  `decode_ea.vh` (10 read-side
functions) is the existing foothold; the refactor is the emit-side
version of the same idea.

---

## 8. Summary

- **LoC**: 22 K → ~4 K (5.4×); +~1 K back for opt-outs → ~4.9 K net.
- **2-wide decode**: feasible after refactor; blocked / wasteful
  without it.  Refactor is the enabler.
- **Per-feature cost**: new ISA = 1 sem-row + 1 EA coverage check +
  1 directed test (vs today's ~500 LoC matrix expansion).
- **Migration cost**: ~22 agent-sessions, staged to be abandonable.
- **Non-matrix opt-outs**: MOVEM / BSR / BFINS-dyn / CAS2 / MUL.L-64
  — known shapes, isolated via `crack_kind`.

Owner: next decode-track agent to pick up Stage A.

---

## 11. Legacy fallback inventory (post V2-move-regmem-gap, `main @ 5898e68`)

The V2 decode chain owns the bulk of the ISA matrix by opcode family.
A residual set of EA shapes still falls through to the legacy
`.vh` sheets at run-time when decode_ea_v2 reports `supported=0` for
either the src or the dst side, or when a per-family fire gate
explicitly excludes a shape.  This section inventories the remaining
legacy paths so follow-up decoder work can target them directly.

### 11.1 decode_ea_v2 shapes that report `supported=0`

| EA form                                          | Mode bits | Notes                       |
|--------------------------------------------------|----------:|-----------------------------|
| Full-format memind (src or dst)                  | 110, ext1[8]=1 | bd_size / od_size matrix    |
| PC-indexed brief / full-ext                      | 111/011   | `(d8,PC,Xn)` / `([bd,PC,...],od)` |
| Reserved 111/101..111                            | 111/101+  | Reserved encoding — traps as ILLEGAL |

Brief-format indexed (110, ext1[8]=0) is supported on both sides of
decode_ea_v2 as of the v2-move-regmem-gap landing.

### 11.2 Per-family gate exclusions (V2 fire=0 despite EA supported)

| Family     | V2 covers                                   | Drops to legacy on                                     |
|------------|---------------------------------------------|--------------------------------------------------------|
| MOVE       | reg↔reg, imm→{Dn, mem}, reg→mem, mem→Dn,    | src-indexed + dst-{indexed, mem},                      |
|            | mem→mem (non-indexed), brief-indexed dst,   | full-ext memind, PC-indexed src                        |
|            | brief-indexed src → Dn direct               |                                                        |
| ALU family | reg-reg, Dn↔mem (non-indexed), addq/subq,   | src OR dst brief-indexed (explicit                     |
|            | addi/subi/andi/ori/eori/cmpi mem-dst,       | `!v2_src_is_indexed && !v2_dst_is_indexed` gate in     |
|            | ADDX/SUBX reg-reg and -(An)-(An), CMPM      | decode.v)                                              |
| shift/rot  | reg form (Dn count, immediate count),       | shift memory form with brief-indexed / PC-rel /        |
|            | memory form (An),(d16,An),(xxx).{W,L}       | immediate / Dn/An direct target                        |
| bit-op     | Dn-direct target, memory target (non-       | indexed target; dynamic BTST #imm target long          |
|            | indexed), BTST PC-rel src                   |                                                        |
| mul/div    | reg/imm/mem src (non-indexed), MUL.L SZ=1   | DIV.L SZ=1 (64÷32) — decoded to ILLEGAL today,         |
|            | dual-dst                                    | blocked on 3rd read-port PRF; src-indexed              |
| branch     | BRA/Bcc/DBcc/Scc Dn-direct/TRAPcc           | —                                                      |
| sys-op     | TRAP/NOP/STOP/RESET/ILLEGAL/MOVEC/MOVE USP  | —                                                      |
| bitfield   | STATIC offset+width on Dn-direct, (An),     | DYNAMIC offset or width (ext1[15]==1 OR ext1[11]==1),  |
|            | (d16,An), (xxx).{W,L}, (d16,PC)             | indexed/predec/postinc target                          |
| MOVEM      | list ↔ (An)/-(An)/(An)+/(d16,An)/(xxx).W/.L,| indexed / full-ext EAs; MOVEM.W load-postinc with      |
|            | (d16,PC)                                    | popcount=16                                            |
| BSR/JSR/   | BSR, RTS, JSR {(An), (xxx).W/.L, (d16,PC)}  | JSR (d16,An) and indexed JSR — legacy also lacks       |
| RTS        |                                             | these → vec-4 ILLEGAL path                             |
| MOVEP      | (d16,An) ↔ Dn (all 4 sub-forms)             | —                                                      |
| LINK/UNLK  | LINK.W/.L, UNLK                             | —                                                      |
| BCD        | reg-reg ABCD/SBCD/NBCD/PACK/UNPK            | memory forms (-(An)-(An)) via legacy / deferred        |
| unary Dn   | NEG/NEGX/NOT/CLR/TST/EXT/EXTB/SWAP on       | memory-EA NEG/NOT/CLR/TST                              |
|            | Dn-direct (mmm=000)                         |                                                        |

### 11.3 Legacy sheets still required

Given §11.1 and §11.2, the following legacy `.vh` includes are still
load-bearing for the remaining EA shapes above:

- `decode_0000.vh` — parent sheet; includes move_{byte,word,long}.vh.
- `decode_0100.vh` — memory-EA unary (NEG/NOT/CLR/TST), JSR non-
  covered EAs, misc 0100 group instructions.
- `decode_0101.vh` — ADDQ/SUBQ/Scc mem-dst variants (some still
  route to legacy on indexed EAs).
- `decode_0110.vh` — BRA 8-bit-disp shape aliases (V2 now emits
  these, but legacy sheet is read-only fallback).
- `decode_0111.vh` — MOVEQ alternate opword encodings (V2 handles
  the PRM-canonical row; legacy handles any lingering pattern).
- `decode_1000.vh` / `decode_1001.vh` / `decode_1011.vh` /
  `decode_1100.vh` / `decode_1101.vh` — ALU matrix indexed-EA
  variants, ADDX/SUBX -(An)-(An) memory form, CMPM, memory
  shift/rot beyond the V2 subset.
- `decode_1110.vh` — shift/rotate memory form with unsupported EAs.
- `decode_1111.vh` — line-F (FPU) trap stub.
- `decode_move_byte.vh` / `decode_move_word.vh` /
  `decode_move_long.vh` (8 595 LoC) — required for:
    1. src-indexed MOVE → dst-{mem, indexed}
    2. full-format memind on either side
    3. PC-indexed src
  Deletion of these three files is blocked on the V2 MOVE assembler
  growing (1), (2), (3).  See Stage E drop-order below.

### 11.4 Suggested drop order to retire the `decode_move_*.vh` trio

1. **Brief-indexed src → {mem, indexed} dst.**  Mirror the
   src_is_indexed+dst_is_dn_direct crack (landed here) for each of:
   src-indexed → (An)/(An)+/-(An)/(d16,An)/(xxx).{W,L}, and
   src-indexed → dst-indexed (worst case ~10 µops).  Estimated 150 LoC
   in decode_uop_assemble.v.
2. **Full-format memind** on src AND dst.  decode_ea_v2 must grow
   ext-word parsing for ext1[7:4] = {BS, IS, BD_SIZE, IS_LEVEL}.
   Estimated 200 LoC in decode_ea_v2.v + 250 LoC in the assembler
   (4 extra phases per side).
3. **PC-indexed brief and full-ext.**  Symmetric with (1)+(2) but
   base=PC+2 instead of An.  Estimated 80 LoC total.
4. Delete `decode_move_byte.vh`, `decode_move_word.vh`,
   `decode_move_long.vh` (8 595 LoC net delete).  Remove their
   include lines in `decode_0000.vh`.

After step 4, the same progression can then retire the ALU /
shift / bit-op / bitfield / MOVEM indexed fallbacks from the other
`.vh` sheets, family by family.  MOVE is the largest (single
family, 8 595 LoC); the rest sum to ~10 K LoC of indexed-shape
tails across the remaining sheets.
