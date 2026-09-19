# Optimisation roadmap — pushing past the MVP

This is the menu of performance techniques to apply once the core actually
boots Mac OS (end of phase 3). Each entry has a rough cost estimate, the
expected IPC/Fmax win, and notes on FPGA suitability for KU5P.

Companion docs:
- [microarch.md](microarch.md) — current microarch + CCR/BTB plans (phase 1)
- [gameplan.md](gameplan.md) — phase 4 references this doc

Categorisation: the techniques fall into 6 buckets, ordered by impact for
typical Mac OS workloads:

1. Front-end widening (decode/dispatch bandwidth)
2. Branch + indirect prediction
3. Memory subsystem (load/store + cache)
4. Speculative tricks (move elim, zeroing, value pred)
5. Execution unit scaling
6. Far-future ideas (L2, SMT, trace cache)

---

## 1. Front-end widening — the biggest single lever

Single-issue decode is the IPC ceiling at 1.0. To get past 1.0 sustained
on Mac OS workloads (heavily branchy, mixed memory + ALU), the front-end
must widen.

### 1a. 2-way decode

**What:** decode emits up to 2 μops per cycle.

**Cost:**
- Predecoder: scan two instruction starts in parallel from the 16-byte
  window. Critical path doubles in fan-in but stays within 2 LUT levels.
- Decode: instantiate two decode units sharing the pd_buf window.
- pd_consumed widens: 0 / 2 / 4 / 6 / 8 bytes per cycle.
- Multi-uop crack: the decoder phase counter must allow at most one
  cracking instruction per cycle (the second slot is empty if slot 0
  cracks).

**Wins:** straight-line code (no taken branches) hits IPC ~1.8–2.0.

**KU5P risk:** the predecoder LUT depth is the most critical timing path
in the front-end. Already ~3.5ns single-issue. Doubling fan-in may push
above 4ns. Mitigate with a 2-cycle predecode pipeline.

**Estimated effort:** 2 sessions.

### 1b. 2-way RAT + dispatch

**What:** rename two μops simultaneously. Each gets a fresh phys-dst
allocation. Source lookups use 4 read ports on the RAT (2 src per μop).

**Cost:**
- RAT: implement as flat reg array (already is) with 4 read ports + 2
  write ports. KU5P distributed RAM handles this with no contention.
- Free-list: pop 2 entries per cycle. Trivial.
- ROB: dual dispatch. Tail pointer advances by 0/1/2.
- IQ: each IQ takes 1 dispatch / cycle from each rename slot. May get 2
  pushes in 1 cycle if both μops are integer ALU.

**Wins:** combines with 2a. Without 2-way dispatch, 2-way decode is wasted.

**Estimated effort:** 2 sessions.

### 1c. Loop buffer (μop cache for short loops)

**What:** detect short loops (< 16 instructions) at fetch, cache the
already-decoded μops in a small buffer. Subsequent iterations skip
fetch + predecode + decode entirely.

**Cost:** ~1 BRAM (16 entries × 80-bit μop encoding). Loop detection FSM
in fetch.

**Wins:** every IPC-critical inner loop in QuickDraw, file copy, etc.
benefits. ~10-15% real-world IPC.

**Estimated effort:** 1 session.

---

## 2. Branch + indirect prediction

### 2a. gshare (replaces bimodal BPU)

**What:** XOR PC with global history register, index into 4K × 2-bit
predictor table. 95%+ accuracy on Mac OS-class workloads vs ~85% for
bimodal.

**Cost:** 1 BRAM for the 4K × 2 table. 16-bit GHR. Combinational XOR.

**Wins:** every conditional branch. Mispredict rate drops by ~3×.

**Estimated effort:** 1 session.

### 2b. Hybrid predictor (gshare + bimodal)

**What:** two predictors, third meta-predictor chooses between them per
branch. Some branches favor history (loops), others favor local bias
(early-out checks).

**Cost:** another 4K × 2 BRAM + meta-predictor of similar size.

**Wins:** another ~1.5× reduction in mispredict rate.

**Estimated effort:** 1 session.

### 2c. Indirect Branch Predictor (IBT — Indirect Branch Target)

**What:** for JMP (An), JMP ([An]), and other indirect branches: predict
the target address. Mac OS dispatches (jump tables) hit this constantly.

Implementation: tag-indexed cache, ~512 entries × 32-bit target. PC ⊕ GHR
selects. Verify target after JMP resolves.

**Cost:** 4 BRAMs.

**Wins:** every Toolbox dispatch, every C++ vtable lookup, every Mac OS
event loop. Substantial.

**Estimated effort:** 2 sessions.

### 2d. Return Address Stack (already in next_iteration.md)

**What:** 8-entry RAS pushed on BSR commit, popped for RTS lookup. Beats
BTB for return targets (each call site has different return address).

**Cost:** 8 × 32-bit shift register.

**Wins:** every RTS hits the right target without polluting the BTB.

**Estimated effort:** 1 session (mostly already designed).

### 2e. Branch fusion

**What:** detect CMP+Bcc pairs at decode, fuse into single μop. Saves
ROB slot, dispatch slot, and the CCR rename hop.

**Cost:** decode complexity increase; new ALU op (`ALU_CMP_BR`).

**Wins:** ~20% of all branches in benchmark code are CMP+Bcc. Phase 4
move only when basic gshare lands.

**Estimated effort:** 1 session.

---

## 3. Memory subsystem

### 3a. Multi-outstanding loads (already on wip branch)

**What:** N_LD=4 load slots with AXI ARID tagging. Multiple loads in
flight; bus returns out-of-order tagged by RID.

**Status:** designed and partly built on `wip/ddr-peripheral-bringup`.
Needs integration with the OoO-track LSU semantics post-CCR-rename.

**Wins:** load-heavy code (memcpy, struct walks) goes from 1 load every
N cycles (N = round-trip) to N parallel loads. Often 3-4× speedup on
memcpy.

**Estimated effort:** 2 sessions (rebase + reimpl + test).

### 3b. Speculative load past unresolved store (memory disambiguation)

**What:** issue a load even when older stores have unresolved addresses.
If addresses don't conflict, load completes early. If they do, squash
the load (and dependents).

**Cost:** address-based squash logic; alias predictor (~1024 entries)
to suppress squashes for loads that always conflict.

**Wins:** ~15% IPC on memory-heavy code. Mac OS event handlers do this
constantly.

**Estimated effort:** 3 sessions.

### 3c. Store-to-load forwarding

**What:** load that hits a pending store buffer entry returns the
buffered data instead of going to memory.

**Cost:** byte-level address compare on load issue against all SB entries.
Width-mismatch handling (byte store + long load needs special path).

**Wins:** function call sequences (push args, call, pop args) become
free. Substantial in compiled code.

**Estimated effort:** 2 sessions.

### 3d. Hardware prefetcher (stride detect)

**What:** detect sequential or strided load patterns; prefetch the next
N lines into L1 ahead of demand.

**Cost:** ~16-entry stride detection table. Background prefetch path.

**Wins:** ~20% on linear data scans. Mac OS file ops, blits, etc.

**Estimated effort:** 2 sessions.

### 3e. Larger I+D caches

**What:** 16KB or 32KB each (vs 4KB 68040 spec).

**Cost:** linear in BRAM. 16KB = 4× 4KB ≈ 32 BRAMs total. Easy on KU5P.

**Wins:** Mac OS hot working set fits comfortably; ~10% IPC on real
workloads.

**Note:** breaks 68040-spec compatibility for sizing-sensitive code, but
Mac OS doesn't care about the actual size as long as CPUSH/CINV work.

**Estimated effort:** 1 session.

---

## 4. Speculative tricks

### 4a. Move elimination

**What:** at rename, MOVE Dn,Dm becomes a RAT pointer update — Dm now
maps to the same physical reg as Dn. No ALU op issued. Only the CCR
update (if MOVE writes flags) goes to the ALU.

**Cost:** rename complexity increase; "rename freelist" must distinguish
arch-mapping changes from new allocations.

**Wins:** Mac OS prologues / epilogues are MOVE-heavy. ~5-10% IPC.

**Estimated effort:** 1 session.

### 4b. Zeroing idiom recognition

**What:** detect MOVEQ #0,Dn / SUB Dn,Dn / CLR Dn / EOR Dn,Dn at decode.
Rename Dn to a pinned phys-zero register. No ALU op (except CCR if it
writes flags).

**Cost:** trivial decode.

**Wins:** zeroing is ~5% of all integer ops in compiled code. ~2-3% IPC.

**Estimated effort:** 0.5 session.

### 4c. NOP elimination

**What:** detect NOPs (and STOP-translated-to-NOP) at decode; don't
issue a μop at all. Decode just advances PC.

**Cost:** trivial.

**Wins:** small (Mac OS doesn't have many NOPs in hot paths) but free.

**Estimated effort:** 0.2 session.

### 4d. Memory renaming on stack accesses

**What:** detect `MOVE.L (A7,d) → Dn` followed shortly by `MOVE.L Dn → (A7,d)`
(spill/reload). Forward through a small "stack rename buffer" without
hitting D-cache.

**Cost:** 8-entry stack rename buffer. Address compare with offset hash.

**Wins:** compiler-generated spill/reload pairs become free. ~8% IPC on
register-pressure-heavy code.

**Estimated effort:** 2 sessions. Phase 5 territory.

### 4e. Value prediction (controversial)

**What:** predict values returned by frequent loads (e.g. constant
pointers, frequently-zero values). Speculatively use predicted value;
verify when load completes.

**Cost:** value predictor BRAM (~1KB). Squash on misprediction is
expensive.

**Wins:** mixed. Sometimes 5-10%, sometimes nothing. Modern designs
generally don't bother.

**Recommend skipping** unless benchmarks show a specific win.

---

## 5. Execution unit scaling

### 5a. Three ALUs

**What:** add a third ALU pipe. iq_int gets a 3rd issue port.

**Cost:** ALU is small; the issue logic gets tighter (3-port wakeup,
3-port select). RAT/PRF read ports increase to 6 (3 ALUs × 2 srcs).

**Wins:** clears integer-bound stretches faster. ~10% IPC.

**Estimated effort:** 2 sessions.

### 5b. Pipelined FPU

**What:** FADD/FSUB 4-cycle, FMUL 6-cycle, FDIV 12-cycle, FSQRT 16-cycle,
all pipelined (1 result per cycle after fill). Already in the microarch
plan; not yet built.

**Cost:** moderate — 80-bit FPU is real silicon. ~1500 LUTs for adder,
DSP58E2 for mul, iterative for div/sqrt.

**Wins:** unlocks FPU-bound workloads (graphics transforms, audio). Mac
OS doesn't use FPU heavily during boot, so this is phase-3-or-later.

**Estimated effort:** 3-4 sessions.

### 5c. Branch resolution at issue (not commit)

**What:** when branch resolves at ALU output, fire the redirect+flush
immediately (already kind of works via cmpl0_br_*). Saves ~6 cycles per
mispredict vs commit-time redirect.

**Status:** partly done — BPU update fires at resolve, but the redirect
itself goes through ROB commit. Should be fine after CCR rename clarifies
the resolve→commit path.

**Estimated effort:** 1 session, dependent on CCR rename.

---

## 6. Far-future ideas

### 6a. L2 cache (SRAM-backed, 256KB)

> **Superseded 2026-07-16** — see `docs/memhier.md`'s "L2 — shared
> second-level cache / point of coherency" section, which is now
> authoritative for this ticket.  Summary of what changed: L2 moved
> from "between L1 and axi-xbar, CPU-private" to **front-of-MIG**
> (between the axi-xbar's S0 port and the DDR4 MIG bridge), so it
> covers every xbar master (CPU, host debug, boot FSM, future DMA-
> ramdisk/scan-out) as a single point of coherency, with a no-
> allocate-unless-resident policy for streaming masters.  This is
> also now a **platform-repo** (`macqd700-soc/rtl/soc/l2.v`) ticket,
> not `m68k-ooo`.

**What:** on-FPGA SRAM (URAM blocks on KU5P) as a unified last-level
cache in front of DDR4, serving all AXI crossbar masters, not just
the CPU's own L1 misses.

**Cost:** 256 KB logical capacity.  Physical URAM288 primitive count
is an open reconciliation between this doc's original "~16" estimate
and `memhier.md`'s original "4" estimate — this doc's "~16" (one
cascaded pair per way, for parallel 8-way lookup) turned out to be
the more defensible one on the same per-way-parallel-BRAM pattern the
L1D design already uses, but the real number depends on the (already-
specified, 4-cycle) L2 hit latency budget and should be picked at
real Phase-4 design time.  See `docs/memhier.md`'s L2 section for the
full reconciliation. Tag arrays in BRAM (~3 BRAM36, unaffected by the
above).

**Wins:** DDR4 latency is ~80-120ns. L2 hit at ~10ns is 10× faster.
Mac OS working set is small enough that L2 hit rate would be excellent.
Also now doubles as the coherency point for DMA/scan-out traffic,
removing the need for a bespoke bus-snoop port on `dcache.v` once it
lands (see `memhier.md`'s "Coherence story" section).

**New dependency (2026-07-16):** gated on real-hardware DDR4
calibration in addition to Phase-3 L1s — the burst shape L2 needs
already exists in `rtl/board/axi_ddr4_mig_bridge.v`, but it's never
been proven on real silicon.  See `docs/fpga_ddr_firstlight_report.md`.

**Estimated effort:** 4 sessions (unchanged; front-of-MIG re-target is
a design simplification, not more RTL).

### 6b. SMT (Simultaneous Multi-Threading)

**What:** run two threads on the same core. Each thread has its own
rename table and architectural state; they share execution units.
"Run two Mac OSs at once."

**Cost:** very high — RAT, ROB, IQs all need thread tags. Architecturally
elegant but FPGA-resource-expensive.

**Recommend skipping** unless there's a clear "showcase" reason. The
single-thread performance optimisations get more bang for the buck.

### 6c. Trace cache (Pentium 4-style)

**What:** cache decoded μop traces, indexed by entry PC. Bypasses the
front-end entirely for hot traces.

**Cost:** complex. ~8-16 BRAMs.

**Wins:** moderate. Modern designs (Intel post-Sandy Bridge, AMD post-Zen)
went back to instruction-cache + decode because trace-cache management
was expensive.

**Recommend skipping**. Loop buffer (1c) gives most of the benefit.

### 6d. Custom 68k extensions (not 68040 ISA)

**Possible additions** if you're willing to break the strict 68040 contract:

- Branch hint bits in unused opword corners (some bit patterns are
  ILLEGAL on real 68040 — repurpose for "predict taken"/"predict
  not-taken"/"likely loop end").
- MOVE16 (existed on 68060, not 68040): 16-byte block move in single
  uop. Massive memcpy speedup.
- Prefetch instruction (PEA-like but for cache prefetch).
- LOCK prefix for atomic sequences.
- 64-bit register pairs for some operations (e.g. 64-bit multiply
  result accessible as a pair).

These would only help code compiled for the extended ISA. Not useful
for running stock Mac OS, but interesting for "modern m68k" benchmarks.

### 6e. Superscalar superscalar — 4-wide

**What:** decode and dispatch 4 μops per cycle, 4 ALUs, 2 LSU ports,
2 FPU ports. ROB to 128, PRF to 192.

**Cost:** approaches the FPGA's resource limits. ~50% of KU5P LUTs.

**Wins:** approaches IPC 3.0 on parallel workloads. Most Mac OS code
won't sustain this width — the gain may not justify the cost.

**Recommend** as the final stretch goal. By the time we get here we
should have benchmarks indicating where the bottleneck actually is.

### 6f. ASIC tape-out

**What:** take the design to silicon. 28nm process gives ~1-2 GHz
operating frequency vs FPGA's 200 MHz. ~10-50× speedup.

**Cost:** $$$. Requires SkyWater 130nm (free open-source) or similar
shuttle program. ~6 month cycle.

**Recommend** as the "if it really works and people want it" endpoint.
Could become a retrocomputing accelerator product.

---

## Quantitative IPC targets by phase

| Phase | Sustained IPC (Mac OS workload) | How |
|---|---|---|
| End of phase 3 | 0.6-0.8 | Single-issue, basic BTB, in-order LSU |
| End of phase 4 (with 2a-c, 3a, 4a-b) | 1.4-1.6 | 2-wide front, multi-load, move/zero elim |
| End of phase 5 (full opt menu) | 2.0-2.5 | 3-wide front, hybrid pred, MD, prefetch |
| ASIC at 1 GHz, end of phase 5 | wallclock ~50× Quadra | freq × IPC |

The IPC ceiling for Mac OS-class workloads (lots of branches, modest ILP)
is generally ~3.0 even on infinite resources. Past 2.0 the marginal
benefit of additional optimisation drops sharply.

---

## Suggested execution order for phase 4

If you only have time for 4-6 of these post-MVP, do them in this order
for max IPC return per session:

1. **Multi-outstanding loads** (3a) — already 80% built, ~25% wallclock win
2. **2-way front-end** (1a + 1b) — IPC ceiling shifts from 1.0 to 1.8
3. **gshare** (2a) — easy, big mispredict reduction
4. **Move + zero elim** (4a + 4b) — simple, real-world wins
5. **Larger caches** (3e) — 1 session for ~10% win
6. **RAS** (2d) — already designed; clean up indirect predictions

Stop here unless you have benchmarks indicating the next bottleneck.
