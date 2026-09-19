# Pipeline deep rework plan — m68k-ooo at 200 MHz

Status: **strategic plan**, no RTL touched yet. Started 2026-04-26 after the
F2/F3 retime experiment proved that bypass-mux IPC recovery cannot win Fmax
(Vivado STA always scores worst-case = bypass mode = original 83-level chain).

The fundamental conclusion from `docs/fmax_autopsy_20260425.md` and the
F2-consolidate run: **the entire core pipeline is collapsed to ~8 cycles
where each "cycle" is a 30-90 logic-level chain.** Surgical retiming +
bypass mux is insufficient. Reaching 200 MHz requires deep pipelining of
every stage, accepting the IPC cost of higher mispredict / wakeup latency,
and recovering it later via well-known μarch tricks (speculative wakeup,
larger ROB / IQ, more aggressive forwarding).

User has approved:
1. Accept ~30-40% IPC loss on tight dep-chains for 2× clock.
2. Ship 100 MHz first (existing `3c812f1` bitstream is the proven-clean
   baseline; surgical-fix-only main is −7.5 ns short and is NOT 100 MHz
   ready). Then start the deep rework.

This doc is the design contract for the deep rework.

---

## 1. Where we are today

Per the autopsy + bench data:

```
Current pipeline (8 cycles, single-cycle per logical stage):
  cyc 1   F1   PC mux + I-cache tag                           ~10 levels
  cyc 2   F2   I-cache data + predecode → pd_buf              ~15 levels
  cyc 3   D    pd_buf → V2 sem + EA + assemble + always@*
                                       → q_*                  83 LEVELS  ← cluster B
  cyc 4   R    RAT + CCR-RAT + free-list + ROB + IQ enqueue   ~35 levels  ← cluster A subset
  cyc 5   I    IQ wakeup + select + PRF read                  ~25 levels  ← cluster A subset
  cyc 6   E    ALU compute                                    ~15 levels (OK)
  cyc 7   W    CDB broadcast + PRF writeback + IQ wakeup-write ~25 levels  ← cluster A subset
  cyc 8   C    ROB commit + ARF/RAT update                    ~15 levels
```

Peak IPC (bench_ind_adds with bypass): **0.39**. Mac OS workloads expected
0.2-0.3.

Surgical fixes (Stage 0 + Stage 1, on main `53a2bb4`) cleared:
- LUTLP-1 combinational loop in RAT lane-1 dual-dst
- rst_pipe[3] 20K-fanout / −6.79 ns
- boot_rom_loading LUT5 / −7.18 ns
- ROB head_ptr 5.4K-fanout / −1.7 ns
- bpu_update_pc 574-fanout / −0.45 ns

Result: WNS −9.119 → −7.5 ns post-route. **Still does not close 100 MHz.**
The remaining −7.5 ns lives in cluster B (decode 83-level monster) and
cluster A (RAT + IQ wakeup + CDB + commit).

---

## 2. Target: 200 MHz / 5 ns / ≤12 logic levels per stage

The ratio: 83 levels current → 12 levels target ⇒ **need to break the worst
stage into 7 sub-stages.** That bounds the depth growth: front-end alone
goes from 2 to ~7 cycles. Total core depth grows 8 → ~18-20.

Modern OoO chips run at this depth (Sandy Bridge: 14, Zen: 19, P4: 31).
Acceptable.

### Target stage budget

| Block | Current cycles | Target cycles | Per-stage logic budget | Reason for split |
|---|---|---|---|---|
| **Fetch** | 2 | 4 | 12 levels | BTB lookup / I-cache tag / I-cache data / predecode-finish |
| **Decode** | 1 (83 levels) | **5** | 15-17 levels | V2-sem / V2-EA / uop_assemble / fire-gates / dispatch-pack |
| **Rename** | 1 (~35 levels) | 3 | 12 levels | RAT-lookup / free-list-pop / ROB+IQ-write |
| **Issue** | 1 (~25 levels) | 3 | 9 levels | wakeup-CAM / select-priority-encoder / PRF-read |
| **Execute (ALU)** | 1 (~15) | 1 | 12 levels | already ok; tight retime if needed |
| **Writeback** | 1 (~25) | 2 | 12 levels | CDB-broadcast / PRF-write+IQ-wakeup-write |
| **Commit** | 1 (~15) | 2 | 12 levels | commit-window-check / ARF+RAT-update+free-list-push |
| **Total** | 8 | **20** | — | — |

Memory pipeline (LSU) is mostly external-latency dominated (DDR4 + AXI
async bridge); it doesn't need much sub-cycle work and stays at ~4 cycles
internal.

### What this costs (latency penalties)

| Penalty | Current | Target | Workload affected |
|---|---|---|---|
| **Branch mispredict** (front-end refill) | ~6 cyc | ~12 cyc | Branch-heavy: −15% IPC |
| **RAW dep wakeup → re-issue → execute** | 1 cyc | 3 cyc | Tight dep chains: −50% IPC ★ |
| **D-cache miss → LSU restart** | 2-3 + DRAM | +1-2 | negligible (DRAM dominates) |
| **I-cache miss → fetch refill** | 2-3 + DRAM | +2 | small |

★ The dep-chain penalty is the killer. It must be mitigated by speculative
wakeup (issue a dependent op assuming parent will produce, replay if wrong).
Without speculative wakeup, IPC on tight dep chains drops from 0.4 to ~0.13
which at 2× clock is a NET LOSS.

### Net throughput math (post-rework, assuming speculative wakeup recovery)

| Workload | IPC (current) | IPC (rework, no spec) | IPC (rework + spec wakeup) | × 2 clock |
|---|---|---|---|---|
| Branch-heavy | 0.30 | 0.25 | 0.27 | 1.8× ✓ |
| Tight dep chain | 0.40 | 0.13 | 0.32 | 1.6× ✓ |
| ILP-friendly mixed | 0.39 | 0.31 | 0.36 | 1.85× ✓ |
| Cache-miss bound | 0.10 | 0.09 | 0.09 | 1.8× ✓ |

**All workloads net positive at 2× clock**, but only with speculative
wakeup. Without it, dep chains regress.

---

## 3. The 20-stage pipeline

```
                       FRONT END (4 cycles)
F1 PC mux + BTB lookup         → reg
F2 I-cache tag access          → reg
F3 I-cache data + line-fetch   → reg
F4 Predecode + pd_buf assemble → reg

                      DECODE (5 cycles)
D1 pd_buf → V2 semantics       → reg (v2_sem_q)
D2 v2_sem → V2 EA decoder      → reg (v2_ea_q)
D3 v2_ea + v2_sem → uop_assemble → reg (uop_q)
D4 uop_q → fire-gates + always@* → reg (q_*)
D5 q_* → 2-wide pair pack       → reg (dispatch_pkt)

                      RENAME (3 cycles)
R1 RAT lookup (reads arch sources, both lanes) → reg
R2 free-list pop + dual-dst alloc + CCR-RAT    → reg
R3 ROB enqueue + IQ enqueue                    → reg

                      ISSUE (3 cycles)
I1 wakeup CAM (parent broadcasts → ready bits) → reg
I2 select (priority-encode oldest-ready)       → reg
I3 PRF read (already F1)                       → reg

                      EXECUTE
E1 ALU / AGU compute             → reg → CDB

                      WRITEBACK (2 cycles)
W1 CDB broadcast → IQ wakeup-write + PRF-write start → reg
W2 PRF write commits                                  → reg

                      COMMIT (2 cycles)
C1 commit-window check + exception/branch-resolve + flush_en → reg
C2 ARF/RAT update + free-list push + ROB head advance        → reg
```

Total fetch-to-commit: **F1 → C2 = 20 cycles in steady state** (no stalls).

---

## 4. IPC recovery: speculative wakeup (mandatory for the dep-chain case)

The 3-cycle wakeup-→-issue-→-execute penalty is the IPC killer. Standard
fix: **issue dependent ops speculatively, before the parent's PRF-write
commits, and replay on miss.**

### Mechanism

When parent at I1 reads "1-cycle ALU" type:
- Parent expected at E1 in cycle N+3 (I1+3)
- Parent broadcasts on CDB in N+3 (E1 = compute + CDB-broadcast latch)
- Dependent in IQ wakes up in N+4 (W1)
- Dependent selects in N+5 (I2 — but we want to skip this)
- Dependent reads PRF in N+6 (I3 — value not committed until N+5 W2)

Speculative wakeup:
- Parent issues at I2 in cycle N+1
- Parent's expected-ready time is broadcast 2 cycles BEFORE the actual CDB
- Dependent wakes up at W1-2 (= I1-1, i.e. cycle N+1)
- Dependent issues at I2 in cycle N+2 (1 cycle behind parent, NOT 4)
- Dependent reads PRF in N+3 (I3) — but parent value not in PRF yet
- BYPASS network at E1 forwards parent's combinational result directly
- Total dep-chain latency: parent-issue → dep-issue = 1 cycle (matches today)

Cost:
- IQ has to track speculative wakeup state (extra bits per entry)
- Mispredict (parent had a 2-cycle latency op, e.g. mul) → replay dependent.
  Squash 2 cycles of work. Acceptable.
- Adds a wide bypass mux at E1 input (not on critical path because
  pre-staged at I3).

This is **the standard way modern OoO fights pipeline depth IPC loss.**
Required for the rework to win.

### Other latency-hiding tricks (also needed)

| Trick | What it does | Cost |
|---|---|---|
| **2-cycle bypass forwarding** at W1→I3 | Reduces RAW penalty on 1-cycle-latency ALU ops to 1 cycle | extra mux at I3 |
| **Larger BTB** (64 → 256 entries) + 2-bit hysteresis predictor | Reduces mispredict rate by 2-3× | small; BRAM-based |
| **gshare or TAGE branch predictor** | Reduces mispredict rate further | medium; one BRAM |
| **Larger ROB** (64 → 128) | Hides longer latency, bigger OoO window | linear in entries |
| **Larger IQ-int** (16 → 32) | More candidates to issue per cycle | quadratic in wakeup CAM |
| **Larger PRF** (96 → 128) | Avoids RAT stalls on rename | linear |
| **Predecode-side pair-packing** | Two lanes find consecutive macros earlier | small extra logic |

Combined, these recover most of the dep-chain IPC. The **non-negotiables**:
spec wakeup + 2-cycle bypass forwarding. Everything else is tuning.

---

## 5. Implementation roadmap

The rework lands as a series of agent-shaped tasks, each independently
sim-validatable. Order matters: front-end first (cluster B), then back-end
(cluster A), then μarch widening for IPC recovery.

### Phase α: 100 MHz HW bringup (parallel — uses existing bit)

α0. **Use `3c812f1` bitstream** for HW bringup (it closes 100 MHz cleanly).
    Accept: missing #256 debug-reset re-arm (probe-map v7), missing surgical
    fixes. Use for sim-vs-HW correlation only, not for shipping demos.

### Phase β: Front-end deep pipeline (decode 1→5 cycles)

β1. **F4 backpressure** — register q_*, add `q_valid` + `rn_ready` elastic
    handshake. No bypass mux. Sim-validates current behavior with a 1-cycle
    delay on rename accept. (~Stage A scaffold + minimal regs.)

β2. **D4 register insertion** — split fire-gates from q_* pack. Stages between.
    Sim re-validate.

β3. **D3 register insertion** — register uop_assemble outputs. Stages between
    D2 and D4.

β4. **D2 register insertion** — register V2 EA outputs.

β5. **D1 register insertion** — register V2 semantics outputs.

β6. **F4 split** — predecode-finish on its own cycle (already mostly there).

β7. **F3, F2, F1 split** — not necessary for the worst path, just for cluster
    cleanup. Probably free wins.

After β: WNS expected ~−2 to −4 ns post-route. Cluster B gone, cluster A is
now the worst.

### Phase γ: Back-end deep pipeline (rename + issue + writeback + commit)

γ1. **R1/R2/R3 split** — RAT lookup / free-list / IQ enqueue across 3 cycles.

γ2. **I1/I2/I3 split** — wakeup CAM / select / PRF read across 3 cycles.
    PRF read already has F1 register stage from `#126`.

γ3. **W1/W2 split** — CDB broadcast / PRF write across 2 cycles.

γ4. **C1/C2 split** — commit check / state update across 2 cycles.

After γ: WNS expected ~+0.5 to +2 ns at 100 MHz. Push target to 150 MHz,
re-synth, iterate. Dep-chain IPC is HORRIBLE here (no spec wakeup yet) —
benches will regress 30-50% on tight chains.

### Phase δ: IPC recovery

δ1. **Speculative wakeup at IQ-int** — parent's expected-ready broadcast,
    dependent issues speculatively, replay on miss.

δ2. **Bypass network at E1 input** — forward parent's combinational result
    when same-cycle dependent issues.

δ3. **Larger BPU + ROB + IQ + PRF** (per `#229`/`#247` widening from earlier
    waves; revisit + push further).

δ4. **gshare or 2-bit-hysteresis BPU** if mispredict rate is still high.

After δ: IPC should recover to within 5-10% of baseline; net throughput at
2× clock is the win.

### Phase ε: Push toward 200 MHz

ε1. Re-synth at 5 ns target. Find new worst path, retime.

ε2. Likely new bottlenecks: ALU shifter chain, mul/div multi-cycle paths,
    LSU AGU, MMU walker. Each needs sub-cycle pipelining.

ε3. Iterate ε1 until WNS ≥ 0 at 200 MHz.

---

## 6. Open architectural questions (deferred per user)

These are decisions that affect every stage. Defer to mid-rework when we
have actual numbers from sim:

- **Spec-wakeup mispredict policy**: full squash vs partial replay
- **ROB depth**: 64 → 128 → 192? More = more parallelism but more flush cost
- **PRF size**: 96 → 128 → 192? Constrains rename throughput
- **IQ-int depth**: 16 → 32 → 64? Quadratic wakeup-CAM area
- **BTB depth + history**: 64 / 1-bit → 256 / 2-bit → 1024 / gshare
- **2-wide → 3-wide decode** down the line? Only after 2-wide is stable

---

## 7. Validation strategy

Each phase β/γ/δ/ε:

1. Sim-first: `make test` 541 PASS / 0 FAIL, `make fuzz N=300` 0 MISMATCH
2. Bench: 4 benches must remain functional (cycle counts will change).
   Document cycle delta; tolerate up to 50% IPC drop in phases β-γ
   (recovered by δ).
3. Lint clean.
4. Synth + measure WNS before moving to next sub-phase. Each register
   stage must demonstrably reduce WNS (can re-synth post-place to skip
   route + bitgen for fast iteration; only do route at phase-end milestones).

When δ recovers IPC, re-bench and document final numbers.

---

## 8. What this is NOT

- Not a 1-week effort. Realistically: 4-6 weeks of focused work assuming
  agent-driven sim validation per sub-phase.
- Not a guarantee of 200 MHz. The KU5P speed grade -2 may have intrinsic
  limits at certain stages (e.g. the BRAM read path is ~2.2 ns minimum).
  We may land at 150-180 MHz and ship.
- Not a 2× IPC throughput. The honest expectation is 1.4-1.8× net
  throughput vs current baseline, depending on workload.
- Not "rip out and rewrite" — every retime is incremental on top of
  existing RTL. Multi-µop cracks, exception handling, MMU walker, FPU
  all stay; their clients just see new pipeline stages above them.

---

## 9. Decision log

- **2026-04-26**: F2/F3 retime with bypass mux abandoned. Vivado STA
  scores worst-case path through any mode = bypass = original 83-level
  chain. Bypass-mux IPC recovery cannot win Fmax. Pure register staging
  with elastic-buffer backpressure is the only path forward.
- **2026-04-26**: User approved 30-40% IPC loss on dep chains in exchange
  for 2× clock, on the understanding that speculative wakeup will
  recover most of it post-rework.
- **2026-04-26**: 100 MHz HW bringup uses existing `3c812f1` bitstream
  (proven clean close); current main `53a2bb4` is −7.5 ns short of 100 MHz.
- **2026-04-26**: Architectural sizing (ROB / IQ / PRF / BTB depths)
  deferred until mid-rework; current sizes are starting point.
- **2026-04-26**: Speculative wakeup mispredict policy = **full squash**
  (simpler, no partial-replay state machine in IQ). Squash cost = 1-2
  cycles of replay work per mispredict, acceptable.
- **2026-04-26**: Sim-validation cadence = **batched**. Each phase (β / γ
  / δ / ε) sim-validates at the phase boundary, not after each sub-stage.
  Reduces context-switch overhead; sub-stages within a phase ride together.
  Risk: harder to bisect when something breaks; mitigation = each
  sub-stage is a separate commit so `git bisect` still works.
