# m68k-ooo µarch Audit & Redesign Plan

> Generated 2026-04-27 from a clean-sheet architectural review against `main`.
> Purpose: identify the structural flaws capping IPC × Fmax today and propose
> a phased upgrade that lands an FPGA-realisable design closer to the
> "fastest 68k" goal.  Companion to `microarch.md` (current state),
> `core_gaps.md` (ISA-level gaps), `optimisation_roadmap.md` (phase-4 perf
> bundle), `ipc_roadmap.md`, and `fmax_autopsy_20260425.md`.

---

## TL;DR

The core is correctly OoO, but three structural flaws cap IPC and Fmax:

1. **Single-port, blocking, sequential LSU** (`rtl/core/mem/lsu.v:246-258`):
   `iss_ready = (state==S_IDLE)` — one in-flight memory op at a time, no
   MSHRs, no hit-under-miss, no store-buffer forwarding.  Single biggest cap
   on real Mac OS workloads.
2. **Decode-bound front end with an 83-logic-level critical path**: per
   `docs/fmax_autopsy_20260425.md:22-31`, `pc_reg → q_*` from
   `decode_semantics + decode_ea_v2 + decode_uop_assemble` reaches 19.06 ns
   with two nets at fanouts 316 and 205.  WNS is **−9.119 ns at a 100 MHz
   target** — i.e. the design will not route at 200 MHz today.
3. **Bimodal-only BPU with 64-entry direct-mapped BTB and 8-deep RAS**.
   No global history, no IBT for indirect branches, RAS overflows silently.
   Mac OS Toolbox routinely nests 12+ deep.

Top 3 wins: F1/F2/F3/F4 front-end split + 8-position parallel predecode +
32-entry µop loop cache; real LSU with 2 AGUs / 8 MSHRs / store-buffer
forwarding / 2-banked L1D / speculative load-past-store; gshare(13) +
L-TAGE-5 hybrid + BTB-512 + IBT-128 + RAS-32 with split CCR rename
({NZVC}/{X}).

Headline expected delta: **~4-5× macro-IPC** (peak ~0.39 → target ~1.6-1.8)
and **WNS −9.1 ns @ 100 MHz → +0 ns @ 200 MHz** post-route.

---

## 1. Design flaws (severity-ordered, file:line citations)

### Front-end (highest severity)

**F-1.** Single 83-level path `PC → rotation-buffer-CE` that gates fetch
advancement — `docs/fmax_autopsy_20260425.md:22-31`.  Two nets at fanouts
316 and 205.  Round of attempted retime documented at `if_stage.v:75-111`,
re-grew on tasks #249/#251/#252/#254/#255/#256.  Cost: **no route at 200 MHz**.

**F-2.** Variable-length predecode is a single-cycle flat case-statement on
`opword[15:12]`, computed twice (positions 0 and 2),
`rtl/core/decode/predecode.v:81-127, 244-340`.  No parallel scan beyond pos
2; spec at `docs/microarch.md:36-38` calls for positions 0,2,4,6,8 but only
0,2 are wired.  Lengths 12/14-byte instructions (full extension words +
bd.L+od.L) cause silent degeneration to 1-wide.  Cost: decode width capped
at 2 macro-ops with frequent degenerate 1-wide on full-format EAs (LEA
index, MOVE table, Toolbox JSR/JMP indexed).

**F-3.** 2-wide accept gate is conservative to the point of self-defeat.
`CLAUDE.md:740-755`, `docs/bench_baseline.md:43-53`: lane-1 accepts only
`UOP_INT/UOP_NOP/UOP_LOAD/UOP_STORE`, no branches/SYS/FP/AGU, no elim,
no `has_dst_b`, same-IQ-only.  Measured lane-1 fire rate **10–30%** of
macros.  Macro-IPC peak today is `bench_move_heavy = 0.394` against a
mathematical ceiling of 2.0.  Only 5–18% of theoretical 2-wide headroom
captured.

**F-4.** BTB phase-1 + phase-2 commit-time-train, bimodal-only
(`rtl/core/fetch/bpu.v:74-181`, 64-entry direct-mapped, 2-bit bimodal, no
GHR).  Indirect branches "self-correct via BTB counter decay"
(`CLAUDE.md:608-610`) — every distinct callsite eats ~10-cycle commit
redirects.  RAS 8-deep (`rtl/core/fetch/ras.v`); Mac OS Toolbox nests 12+.
Cost: `bench_btb_dbra IPC=0.142`, `bench_cmp_branch IPC=0.235`
(`docs/bench_baseline.md:434`).

**F-5.** Multi-µop cracks are decode-stage state machines — MOVEM up to
**17 phases** (`CLAUDE.md:653-654`), MOVEP up to 14 (`docs/isa_status.md:13`),
BSR 2, RTS 1, BFINS dyn-both 2 (`tracks/core.md:99-103`).  Each phase = one
decode + one dispatch.  `MOVEM.L D0-D7,(A7)` = 17 cycles before dispatch
advances.  No µop cache or replay buffer.

**F-6.** `q_d_*` register pair is a one-stage flop between decode and
dispatch.  Long combinational path produces `q_*` with no internal
pipelining.  Autopsy at `docs/fmax_autopsy_20260425.md:148-161` proposes
F1/F2/F3 split; not yet done.

### Rename / dispatch

**R-1.** RAT free-list is a 96-wide one-hot priority encoder for **4
simultaneous allocs** (`rtl/core/rename/rat.v` 64-80).  Four allocs/cycle
all scan a 96-bit `free_bm` with cascading-mask priority encoders.
`docs/uarch_proposals.md:96-103` flags +0.3 ns at 48 → +0.8 ns at 96
entries.  The recent PRF widening already cost timing.

**R-2.** CCR-RAT renames per partial-flag-write but bypass is OK only for
full-mask writers (`rtl/core/issue/iq_int.v:271-296`, `e_ccr_bypass_ok`).
ALU_MOV/CMP/TST/AND/OR/EOR (mask=NZVC, X-preserve), BTST/BSET/BCLR/BCHG
(mask=Z) all FAIL bypass — wait for prior CCR producer's tag.  Mixed
CMP-with-ADD serialises every CMP.

**R-3.** RAT rollback is single-cycle 96×7-bit rebuild
(`rtl/core/rename/rat.v:170-191`, `CLAUDE.md:582-591`).  Borderline at 96;
flush rate scales with mispred × ROB depth.

**R-4.** ARCH_INT_REGS = 19 wastes phys regs on REG_PC/REG_CCR conflation
(`rtl/core/decode/uop_pkg.v:57, 76-82`).  TMP1/TMP2 should be rename-only
scratch; today they consume 3 perma-pinned phys regs (16/17/18).

### Issue queues / scheduler

**I-1.** iq_mem 8 entries with O(N²) `blocked_by` bitmap, already a
2-cycle pipelined build (`rtl/core/issue/iq_mem.v:233-336, 437-498`).
Compound retime at `:247-329` is the design's hairiest block.  Scaling to
16 quadruples bitmap.  `disp_d_ready` 0 across cycles N and N+1
(`:391-396`) — 2-wide enqueue back-pressures the front-end one cycle on
every double insert.

**I-2.** iq_mem disambiguation over-conservative (`iq_mem.v:466-485`).
Alias compares `e_disp[31:2]` only — `MOVE.B (A0,2),D0` and
`MOVE.B (A0,3),D1` collide spuriously.  In a write-back D-cache world this
becomes a real ordering stall.

**I-3.** iq_int sel is high-index-first scan with `nxt_*_rdy` combinational
wake (`rtl/core/issue/iq_int.v:412-444`).  Lane-B is a second pass
excluding lane-A's pick — ~9 LUT levels in series.  Second-tier cluster A
critical path per `docs/fmax_autopsy_20260425.md:46-53`.

**I-4.** MUL/DIV serialises iq_int wholesale (`iq_int.v:155-173, 348-362`).
`alu_mul_busy` zeroes `sel_mask` for MUL/DIV lifetime — 4 cycles MUL.L,
~34 cycles DIV.L.  Single DIV.L stops the integer back-end for 34 cycles.

### Execute

**E-1.** Two ALUs but lane-B is a strict subset of lane-A's ops
(`rtl/core/issue/iq_int.v:381-406`).  No branches, no MUL/DIV, no
Scc/TRAPcc/CHK/CHK2/CMP2, no BFEXTU/BFINS/BFFFO/PACK/UNPK/BCD/CAS.  CMP+Bcc
fusion (`docs/uarch_proposals.md:140` I8) lands lane-A only.

**E-2.** ROB head_ptr fanout 5450 with −1.72 ns slack
(`docs/fmax_autopsy_20260425.md:74-78`).  Drives every ROB entry's
"I'm head" comparator.  At ROB=64 this is the new structural problem from
task #255 (ROB 32→64).

**E-3.** No store-buffer forwarding (`rtl/core/mem/lsu.v:1-52, 246-258`).
A store is held in `S_ST_BUF` until commit; younger load to same address
blocks via I-2's alias bit and **cannot forward** from the store buffer.
Every callee-saved register reload from `MOVEM.L (A7)+,<list>` waits for
the prologue's `MOVEM.L <list>,-(A7)` to commit before reloading — 16+
cycles serialised per call/return pair.

**E-4.** LSU is a single-in-flight FSM (`rtl/core/mem/lsu.v:246-258`).
`iss_ready = (state == S_IDLE)`.  With 20-cycle L1D miss latency every
miss serialises the entire memory pipe.  **No MSHRs, no hit-under-miss,
no miss-under-miss.**  Largest IPC cap on real workloads.

**E-5.** Store commit pessimistically holds AXI (`CLAUDE.md:567-575`,
`lsu.v:37-46`).  Full AXI roundtrip post-commit, blocking next memory op
until `bvalid`.

### Memory subsystem

**M-1.** L1I/L1D BRAM-backed but cold-miss costs ~175 cycles per bench
(`docs/bench_baseline.md:629-650`).  No write-coalescing, no way
prediction.

**M-2.** MMU walker exists but is not wired (`tracks/core.md:88-90`):
"Phase-A landed, Phase-B not wired".  ITT/DTT passthrough until #99 wires
fault path.  System 7 doesn't run because of this.

**M-3.** No SMC snoop / I-cache-invalidate-on-D-store (M5 in
`docs/uarch_proposals.md:378`).  Mac OS A-line patching uses self-modifying
code routinely; CPUSH/CINV still ❌ (`docs/isa_status.md:81`).

**M-4.** CPUSH / CINV not implemented (`docs/isa_status.md:81`).  Blocks
Mac OS overlay-exit.

### ROB / commit

**ROB-1.** ROB=64 wrong-path scanner is 64-wide combinational
(`rob.v:276-293` per `docs/uarch_proposals.md:421-428`).  2-cycle pipeline
recommended, not landed.

**ROB-2.** Dual-retire on lane B gated by complex `head0_dual_ok`
(`commit.v:756-780, 1397-1418`).  Excludes branches/stores/SYS/exceptions/
dual-dst/A7.  On Mac OS workloads with frequent BSR/RTS+stores, typically
never satisfied.  Measured win on `bench_ind_adds` only +15%.

**ROB-3.** Retire ≤ 2/cycle (`commit.v:1418`).  If dispatch ever goes
3-wide, retire is the new bottleneck.

### Branch / CCR / partial-flag

**B-1.** RAS 8-deep (`rtl/core/fetch/ras.v`).  Overflow wraps silently
(`CLAUDE.md:300-304`); each wrap costs a commit-redirect on RTS.

**B-2.** CCR-PRF 16 entries (`rtl/core/decode/uop_pkg.v:54-55`); undersized
vs ROB-64 + many CCR writers.

**B-3.** X-flag preservation requires partial-flag readers to wait
(`iq_int.v:288-296`).  ADDX/SUBX/NEGX/ROXL/ROXR/shifts read X; CMP after
ADDX serialises through CCR-RAT.  Splitting CCR PRF into {X-only} and
{NZVC} would unblock — not done.

### FPGA realisation

**FPGA-1.** 94% LUT occupancy at 100 MHz target
(`docs/fmax_autopsy_20260425.md:104`).  At the edge.  PRF→BRAM migration
recommended at `:80-100`, not done.

**FPGA-2.** 12 of 1080 DSP58E2 used (1%) (`:111`).  Wide adders /
comparators / shifters / EA adder / `pc+len` adder / FPU mantissa multiply
all on CARRY8 chains; could move to DSP58E2 macros.

**FPGA-3.** Reset fanout 20355 with WNS −6.79 ns (`:73`).
`core_rst | jtag_debug_full_reset` OR-gate at broadcast point — needs
migration up into clk_rst with extra pipe stages.

**FPGA-4.** Combinational loop in lane-1 dual-dst feedback (`:55-67`).
435-LUT loop through `u_rat/e_pdst_b[14]` and `u_ccr_rat/q_valid_reg`.
Vivado treats as worst-case-delay; inflates 1000s of endpoints.

---

## 2. Proposed design (clean-sheet, ISA-stable)

The 68040 ISA contract is preserved.  Internal µarch is reset to maximise
FPGA IPC × Fmax.

### 2.1 Headline parameters

| Parameter | Today | Proposed | Rationale |
|---|---|---|---|
| Fetch width | 16 B / 1 line | 32 B / 2 lines | parallel L1I, 2 banks |
| Predecode width | 2-pos (deg. to 1) | 8-pos parallel + 16-µop loop cache | KU5P LUT6 is fast for 8-way length lookup |
| Decode width | 1 macro / 2 µops | 4 macro / 6 µops | 4×F1 length-lookup + 4×F2 EA-decode parallel |
| Rename width | 2 | 4 | matches decode |
| ROB | 64 | 128 | covers 200 MHz × 200-cyc DDR roundtrip × 4-wide |
| PRF int | 96 | 160 | 4-wide × 8 cycles in-flight × ~1.4 alloc/macro |
| PRF CCR (NZVC) | 16 | 48 | one per CCR-writer in flight |
| PRF CCR (X-only) | 0 (fused) | 24 | split-flag rename eliminates X serialisation |
| IQ-int | 16 ×2 | 32 ×4 | 4-wide issue, 4 ALUs |
| IQ-mem | 8 | 16-LD + 16-ST | 2 LD-ports, 2 ST-ports |
| IQ-FP | 8 | 16 | FPU goes live |
| ALUs | 2 | 4 (3 simple + 1 complex) | complex has MUL/DIV |
| AGU | 0 (fused) | 2 dedicated | EA-compute parallel with PRF read |
| LSU | 1 single-flight | 2-port banked, 8 MSHRs | hit-under-miss, 8 outstanding |
| L1I | 4 KB / 4-way | 32 KB / 8-way | Mac OS hot-loop footprint |
| L1D | 4 KB / 4-way | 32 KB / 8-way, 2-banked | dual-port via banking |
| L2 | none | 256 KB URAM (4×64 KB) | 6-cycle L1 miss / L2 hit |
| TLB | 64 unified | 64 I + 64 D + 8 L1-uTLB | hit-under-miss |
| BPU | bimodal-512 + BTB-64 | gshare(13)-8K + L-TAGE-5 + BTB-512 + IBT-128 + RAS-32 | matches Toolbox profile |
| Retire width | 2 | 4 | matches dispatch |
| Pipeline depth | ~10 | 14 | 14 × 5 ns = 70 ns mispred; gshare keeps rate <3% |

### 2.2 Block diagram

```
                  ┌─────────────────────────────────────────────────┐
                  │  L1I 32KB 8-way    BPU: gshare(13)+TAGE+BTB512  │
                  │  (2-banked,        +IBT128+RAS32                │
                  │   2 read ports)         │                       │
                  └─────────┬───────────────┘                       │
                            │ 32 B / cycle                          │
                  ┌─────────▼─────────┐                             │
   F1: PREDECODE  │ 8-pos parallel    │  ←── µop loop cache (32×6  │
                  │ length-LUT array  │      µops, hit on L2 line)  │
                  └─────────┬─────────┘                             │
                            │                                      │
   F2: DECODE     ┌─────────▼─────────┐                             │
                  │ 4-wide split:     │                             │
                  │ 4× decode_sem     │                             │
                  │ in parallel       │                             │
                  └─────────┬─────────┘                             │
                            │                                      │
   F3: CRACK      ┌─────────▼─────────┐                             │
                  │ MOVEM/MOVEP/BSR   │                             │
                  │ FSM (BRAM-backed) │                             │
                  └─────────┬─────────┘                             │
                            │ 4-6 µops/cycle                       │
   R1: RENAME     ┌─────────▼─────────┐                             │
                  │ RAT (4 read /     │   CCR-RAT split             │
                  │ 4 alloc), free-   │   {NZVC, X} renamed         │
                  │ list bank-RR      │   independently             │
                  └─────────┬─────────┘                             │
                            │                                      │
   D1: DISPATCH   ┌─────────▼─────────┐                             │
                  │ ROB128 enqueue +  │                             │
                  │ IQ steering       │                             │
                  └────┬────┬────┬────┘                             │
                       │    │    │                                  │
                ┌──────▼┐  ┌▼───┐ ┌─▼──┐                            │
                │IQ-int│  │IQMM│ │IQFP│                             │
                │ 32   │  │ 32 │ │ 16 │                             │
                └──┬───┘  └─┬──┘ └─┬──┘                             │
       ┌──────┬───┴──┬───┐  │      │                                │
       │      │      │   │  │      │                                │
   ┌───▼┐ ┌──▼─┐┌──▼┐┌▼─┐│      ┌─▼───┐                             │
   │ALU0││ALU1│ALU2││Cx│ AGU0/1  │ FPU │                            │
   │simp││simp│simp││MD│ + LSU   │ pipe│                            │
   └─┬──┘ └─┬──┘└──┬┘└─┬┘ ↓      └──┬──┘                            │
     │      │     │   │ L1D 32K     │                               │
     │      │     │   │ 2-bank      │                               │
     │      │     │   │ 8 MSHRs     │                               │
     │      │     │   │  → L2       │                               │
     │      │     │   │  → DDR      │                               │
     └──────┴─────┴───┴───────────┬─┘                               │
            CDB ×6 (4 ALU+2 LSU+CCRx2)                              │
                                  │                                 │
                          ┌───────▼──────┐                          │
                          │ ROB128       │                          │
                          │ retire 4/cyc │ ──── flush ──────────────┘
                          └──────────────┘
```

### 2.3 Pipeline stage map / target combinational depth

| Stage | Cycles | Comb budget | Notes |
|---|---|---|---|
| **F1** I-fetch + uTLB | 1 | 3.5 ns | BRAM read registered |
| **F2** Predecode (8-pos parallel) | 1 | 3.5 ns | flat case → 1 LUT6 stage on a 4-bit length per pos |
| **F3** Decode (4-wide parallel) | 1 | 4.0 ns | 4 independent semantic decoders; no serial chain |
| **F4** Crack/uop-cache lookup | 1 | 3.0 ns | µop-cache hit short-circuits F1-F3 |
| **R1** Rename (RAT 4-port) | 1 | 4.0 ns | free-list bank-RR scanner, 1 LUT level |
| **D1** Dispatch (ROB enqueue + IQ steering) | 1 | 3.5 ns | |
| **S1** Schedule / wakeup (32-entry IQ-int) | 1 | 4.0 ns | 2-level select (group-OR + within-group prio) |
| **X1-Xn** Execute (1 cyc ALU, 4 cyc MUL, 16-34 cyc DIV, 2-3 cyc cache hit) | 1-N | 3.5 ns ALU, 4 ns MUL stage | flag-gen split from data path |
| **W1** CDB broadcast / wake | 1 | 3.5 ns | replicate CDB drivers |
| **C1** ROB head check + retire | 1 | 3.5 ns | retire 4/cyc |
| **C2** RAT commit + free-list push | 1 | 3.0 ns | commit cRAT advance |

Mispredict total: F1+F2+F3+F4+R+D+S = 7 cyc minimum; with gshare at 3%
mispred × 7 = 0.21 cyc/branch overhead.

### 2.4 DSP58E2 + BRAM allocation

DSP58E2 (~120 of 1080 used by proposed design):
- ALU adder/sub: 4 ALUs × 2 = 8 DSPs
- Shifter (ASL/ASR/LSL/LSR/ROL/ROR/ROXL/ROXR) per ALU: 4 DSPs
- MUL.L cascade (32×32→64): 4-DSP58E2 cascade
- DIV non-restoring partial product: 8 DSPs
- AGU `base+disp+index*scale`: 2 AGUs × 2 DSPs
- EA adder for if_stage `pc+len`: 1 DSP
- FPU mantissa multiply: 3 DSPs; adder: 2 DSPs
- BPU gshare XOR + 13-bit history adder: 1 DSP

BRAM (RAMB36 + RAMB18):
- L1I 32 KB 8-way: 8×RAMB36 (data) + 1×RAMB36 (tags + LRU)
- L1D 32 KB 8-way 2-banked: 16×RAMB36 + 2×RAMB36 tags
- L2 256 KB URAM: 4×URAM
- ROB128: 1 RAMB36 (PC/NPC/opword) + 1 RAMB36 (tags)
- PRF int 160×32: 4-port via 4 BRAM18 replicas
- PRF CCR 48×5 + 24×1: LUTRAM
- µop loop cache 32×80b: 1 RAMB18
- gshare 8K×2-bit: 1 RAMB18
- TAGE banks 5×1K: 5 RAMB18

Total: ~50 BRAM36 + 8 BRAM18 + 4 URAM ≪ 480 RAMB36 / 64 URAM available.

CMAC: not used.

### 2.5 Variable-length predecode parallelisation

The hard front-end problem.  Length is determined by `opword[15:12]`
(major group) + EA modes/extension words + optional bd.W/bd.L/od.W/od.L
suffix (`predecode.v:184-241`).  Range 1-11 words (2-22 bytes).

- **8 independent length-LUTs** at fixed byte positions
  {0,2,4,6,8,10,12,14} of the 32-byte fetch buffer.  Each LUT is the
  existing `inst_length_lut` (2 LUT6 levels combinational).  Run in
  parallel in F1.
- **Boundary-chain combinational network**: pos[0] always valid start;
  pos[2].valid = (pos[0].len==1); pos[4].valid = (pos[0].len + (pos[2].
  valid?pos[2].len:0)==2); etc.  Serial dependency, 4 LUT6 levels max for
  8-wide.
- **Decode 4 wide deterministic + 4 spillover**: take first 4 valid starts
  from 8-pos scan; remaining 4 wait for next cycle.
- **µop loop cache** (32 entries × 80-bit µop): backward-branch target
  match → µop replay from cache, bypass F1-F3 entirely.  ~5 cycles saved
  per loop iter on `bench_btb_dbra`, `bench_btb_loop`.  1×RAMB18.

### 2.6 BPU strategy

- **Gshare(13)** main: 8K × 2-bit = 16 Kb (1 RAMB18).  `PC[14:2] ⊕ GHR[12:0]`.
  Target mispred rate 5-7% on Mac OS vs ~15% bimodal.
- **TAGE-5 hybrid**: 5 banks × 1K × tagged 3-bit-counter geometric histories
  {4, 16, 64, 256, 1024}.  Hard-to-predict branches.  Target <3%.
- **BTB-512** 4-way set-associative replacing direct-mapped 64.
- **IBT-128**: indirect-branch-target predictor for JMP(An), JSR(An),
  Toolbox vector dispatch.  Indexed by `PC ⊕ source-reg-value-hash`.
  Cuts the "BTB counter decay self-correct" cost (F-4) to a 1-cycle bubble
  on hit.
- **RAS-32** with overflow-shadow: 32 + 32 shadow.  Spill/refill on
  over/underflow.
- **Decode-time predict, execute-time train, commit-time confirm**:
  predict at F2; train at execute (single-bubble penalty on mispred caught
  at execute); confirm at commit (BPU update only after non-speculative
  path established, prevents wrong-path pollution).

### 2.7 LSU + memory subsystem

- **2 AGUs** (lane 0 + lane 1) — separate from LSU; compute EA in 1 cycle
  parallel with PRF data read for store.
- **8 MSHRs** in LSU; hit-under-miss + miss-under-miss.
- **L1D 2-banked** (even/odd line); 2 LD ports + 2 ST ports issue per cycle
  if no bank conflict.
- **Store buffer 16 entries with byte-mask forwarding** to younger loads on
  full or partial overlap.  Replaces `iq_mem` alias-bitmap (`iq_mem.v:466
  -485`); aliasing becomes a forward, not a stall.
- **Speculative load past unresolved store** with 8-entry alias predictor;
  squash on miss.
- **Hardware MMU walker** wired through (`mmu_walker.v` exists); 64 I-TLB
  + 64 D-TLB + 8 L1 uTLB.  ATC fault drives precise vec 2/3.
- **Stride prefetcher** in front of L1D for linear scans; 8 stride-stream
  slots.
- **L2 256 KB URAM**, 6-cycle hit, write-back; covers L1D miss working
  sets.

### 2.8 CCR rename / partial-flag handling

Split CCR-PRF into two sub-PRFs:
- **NZVC-PRF** (4 bits × 48 entries): allocated by writers of any of N,Z,V,C.
- **X-PRF** (1 bit × 24 entries): allocated by writers of X (ADD, SUB,
  ADDX, SUBX, NEGX, ROXL, ROXR, ASL/ASR/LSL/LSR shifts of count >0).

A reader of {NZVC} (Bcc, DBcc, Scc, TRAPcc) reads only NZVC-PRF tag.  A
reader of {X} (ADDX, SUBX, NEGX, ROXL, ROXR) reads only X-PRF tag.
Eliminates F-2's serialisation: a CMP (writes NZVC, preserves X) doesn't
allocate X-PRF, so a younger ADDX (reads X) waits only for the prior
X-writer.

### 2.9 Multi-µop crack policy

- **MOVEM**: still cracked, but **µop cache stores cracked sequence** (µops
  1..N + last_phase marker).  On second invocation, replay from µop cache:
  0-cycle decode for repeated MOVEM.
- **BSR/RTS**: keep 2-µop / 1-µop crack — already optimal.
- **DIV.L SZ=1 (64÷32)**: 2 µops with TMP1 staging
  (`docs/isa_status.md:22`).
- **MUL.L SZ=1 (32×32→64)**: dual-dst PRF allocation already in place;
  break the comb loop by registering the lane-1 dual-dst feedback
  (`docs/fmax_autopsy_20260425.md:60-67`).
- **Bitfield ops**: BFINS dynamic-both already 2-µop; static cases stay
  1-µop.
- **PACK/UNPK/BCD**: stay sequential, rare path.

---

## 3. Phased migration plan

Each phase is independently sim-validated (`make test` + `make fuzz N=200`),
bench-measured (cycles delta on the 9 bench grid), Fmax-checked at phase
boundary only (per `docs/agent_policy.md:46-50`).  No phase merges into
`main` without bench delta logged in `docs/bench_baseline.md`.

| Phase | Scope | IPC Δ | Fmax Δ | Risk | Prereqs |
|---|---|---|---|---|---|
| **P0** | Stabilise: break combinational loop in `u_rat/e_pdst_b[14]`; fix `rst_pipe[3]` fanout 20355; register `u_boot_fsm/rom_loading_reg_0`; pipeline ROB head_ptr broadcast | ~0% | +6-8 ns WNS @ 100 MHz | low | none |
| **P1** | Front-end retime: F1/F2/F3/F4 split per `fmax_autopsy_20260425.md:139-161` | -3% (extra cold-fetch) | +6 ns WNS, closes 150 MHz | high (touches biggest file) | P0 |
| **P2** | PRF→BRAM 4-replica + DSP58E2 ALU/AGU adders + CARRY8→DSP58 conversion | 0% | +1.5 ns WNS, frees 30K LUTs | med | P1 |
| **P3** | gshare(13) + IBT-128 + RAS-32 (replace bpu.v) | +5-8% on branchy benches | +0 ns | low | P1 |
| **P4** | Real LSU: 2 AGUs + 8 MSHRs + store-buffer forwarding + 2-banked L1D | **+25-40%** on bench_mixed_mem, bench_fullpipe | +1 ns | high (correctness surface) | P2, plus phase-3 real L1D landed |
| **P5** | Wider front-end: 4-pos predecode + 4-wide decode + µop loop cache | **+30-40%** on bench_alu_parallel, bench_move_heavy | -1 ns (recover via DSP58 + retime) | high | P1, P3 |
| **P6** | Wider rename + 4-wide IQ + 4 ALUs + 6 CDBs + split CCR (NZVC/X) | **+60-100%** (peak macro-IPC ~1.6) | -2 ns (then recover) | high | P5 |
| **P7** | ROB-128 + PRF-160 + L2 URAM + speculative load past store + alias predictor | +15-25% on Mac OS | +0 ns | high | P6 |
| **P8** | TAGE hybrid + memory rename on stack + FPU body | +5-15% on hard-to-predict | +0 ns | very high | P3, P7 |
| **P9** | MMU walker wire + SMC snoop + CPUSH/CINV → boot Mac OS | (correctness) | +0 ns | very high | P4 |

Cumulative expected: today's peak 0.39 macro-IPC → ~1.6-1.8.  Today's WNS
−9.1 ns @ 100 MHz → +0 ns @ 200 MHz post-P2.

**P0 is "free" and gates everything else.**  No-IPC-impact, low-risk,
unlocks +6-8 ns WNS at 100 MHz — moves the design from "won't route at
200 MHz" to "has slack to retime upward".  Right next concrete piece of
work: small surface, big leverage, clears the way for P1's F1/F2/F3/F4
split which is the actual fmax fix.

---

## 4. Open questions / experiments

**Q1.** Actual mispred rate on Mac OS ROM cold-boot for bimodal vs
gshare(13) vs TAGE-5?  Run Musashi-style trace under MAME for first 10 M
instructions, capture branch outcomes + PCs, simulate predictors offline.
Without this, the +5-7% claim for P3 is best-guess.  **Cost: ~1 day.**

**Q2.** Working-set fit at L1D=32 KB / L2=256 KB on Mac OS hot loops?  Run
Musashi-trace D-cache miss-rate simulation.  Determines whether L2 is worth
URAM cost or 32 KB L1 alone suffices for INIT chain + Toolbox dispatch.
**Cost: 1 day.**

**Q3.** KU5P 4-wide decode predecode timing: are 8 length-LUTs at LUT6-
depth 2 with a 4-deep dependency chain actually < 3.5 ns?  Synthesise a
standalone `predecode_4wide.v`.  If not, fall back to 3-wide.  **Cost: 1
synth-day.**

**Q4.** Does the `q_d_*` register pair survive the F1/F2/F3/F4 split, or
does it disappear?  4-stage split likely makes the q-stage redundant; need
to confirm against post-P1 wiring.  **Resolved by hand-tracing post-P1.**

**Q5.** Is store-buffer forwarding worth the alias-predictor complexity vs
just speeding up commit?  Analytical: 16-entry SB with byte-mask forwarding
catches MOVEM prologue/epilogue spill-reload (Mac OS ~30% of memory traffic
per SheepShaver traces).  Verify on Mac OS boot trace.  **Cost: 0.5 days
analytic, 2 days RTL.**

**Q6.** Does µop cache hit rate justify 32-entry size?  bench_ind_adds
inner-loop ~6 µops; bench_btb_loop ~4; ROM checksum ~12.  32 covers all
observed loops.  Mac OS QuickDraw inner may exceed 32 — measure on first
L1I trace.  **Cost: 1 day post-P5.**

**Q7.** PRF-160 BRAM read with 4 read ports/cycle: is replicate-by-4
sufficient, or do we need bank conflict resolution?  Replicate-by-4 is the
textbook approach (`docs/fmax_autopsy_20260425.md:96-99`); confirms 4
RAMB36 cost.  **Resolved by initial timing of P2.**

**Q8.** Real DDR4 latency at 200 MHz core clock: is 200-cycle round-trip
estimate accurate?  LSU sees pass-through-ish today
(`docs/bench_baseline.md:49`).  Hardware bring-up will tell.  **Resolved
at first real-bitstream bring-up.**

**Q9.** Is 14-stage pipeline mispred cost (~7 cyc) tolerable on
worst-case Mac OS branch density?  Budget: 3% × 7 = 0.21 cyc/branch.  If
real Mac OS hits 8% (Toolbox-heavy), 0.56 cyc/branch — acceptable but
compounds.  **Measure under P3 after gshare lands.**

**Q10.** Should we serialise CPUSH/CINV through LSU like a fence, or
short-circuit through commit?  Mac OS uses these at overlay-exit and
trap-stub patching; correctness drives latency budget.  Recommend: CPUSH =
drain-store-buffer + L1D writeback; CINV = drain + invalidate.  **Specify
before P9.**

---

## 5. Critical files for implementation

- `rtl/core/fetch/if_stage.v`
- `rtl/core/decode/predecode.v`
- `rtl/core/decode/decode.v`
- `rtl/core/mem/lsu.v`
- `rtl/core/issue/iq_int.v`
- `rtl/core/issue/iq_mem.v`
- `rtl/core/rename/rat.v`
- `rtl/core/rename/rob.v`
- `rtl/core/fetch/bpu.v` → `bpu_gshare.v` → `bpu_tage.v`
- `rtl/core/fetch/ras.v`
- `rtl/core/execute/alu.v`
- `rtl/core/execute/agu.v`
- `rtl/core/mem/dcache.v`, `icache.v`, `mmu.v`
- `m68k_core_*.vh` includes
