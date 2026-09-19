# Memory hierarchy + branch prediction — the story

End-to-end story for the six components that sit between the instruction
stream and DDR4: **BTB**, **RAS**, **TLB/MMU**, **L1I**, **L1D**, **L2**.
Read this before touching any of them.

Companion docs:
- [`gameplan.md`](gameplan.md) — phase gating these components attach to
- [`microarch.md`](microarch.md) — pipeline stages they integrate into
- [`core_gaps.md`](core_gaps.md) — per-instruction gaps that drive scope
- [`optimisation_roadmap.md`](optimisation_roadmap.md) — phase 4+ wishlist
- [`peripheral_arch.md`](peripheral_arch.md) — address map + axi-xbar

---

## Pipeline view

```
fetch PC ─► [BTB] ─► redirect on pred-taken ─┐
                                             │
   [IF ]──[ICache]──[MMU I-side]──►DDR       │ (axi-xbar)
       │                                     │
       ▼                                     │
    decode ─► rename ─► dispatch             │
                                             │
                      ┌─► IQ_INT ─► ALU ─┐   │
                      ├─► IQ_MEM ─► LSU ─┤   │
                      │                  ▼   │
                      │     [DCache]──[MMU D-side]──►DDR
                      │                              │
                      └──► commit / ROB ◄────────────┘
                                │
                                ├─► BTB train
                                └─► RAS push (BSR) / pop (RTS)

     "DDR" above is shorthand for axi-xbar's S0 downstream port.  L2
     (phase 4, **re-targeted 2026-07-16** — see "L2" section below) does
     NOT sit between L1{I,D} and axi-xbar.  It sits INSIDE that "DDR"
     arrow, front-of-MIG:

         axi-xbar S0 ──► [ L2 ] ──► MIG bridge ──► DDR4

     Every xbar master (CPU LSU, CPU IF, host debug/XDMA, boot FSM, and
     any future DMA-ramdisk / video-scanout master) already funnels onto
     that single S0 port — so L2 is a single point of coherency for all
     of them, not just the CPU's own L1 misses.
```

At a glance: BTB + RAS are front-end speculation; L1I + MMU-I are the
fetch pipe; L1D + MMU-D + store buffer are the memory pipe; L2 sits
behind the axi-xbar and in front of the MIG/DDR4 bridge — it is the
single point of coherency for every master the xbar aggregates, not a
CPU-private structure.

---

## BTB — branch target buffer

**What it is.**  Direct-mapped predictor indexed by PC bits.  Stores
{tag, target, 2-bit bimodal counter, valid}.  Predicts direction +
target for direct branches (BRA, Bcc, BSR, DBcc).

**Where it lives.**  `rtl/core/fetch/bpu.v` — wired into `if_stage.v`
at decode-time lookup.  Trained at commit today.

**Phase status.**

| Phase | State | What |
|---|---|---|
| 1 — DONE | `bpu.v` on main | 64-entry DM, 2-bit bimodal, commit-time train.  Wins 8-17 cycles on flags-heavy benches. |
| 1.5 tail | `bpu-phase2` | Execute-time train: thread `uop_pc` through iq_int → alu → cmpl0; drive `bpu_update_*` from cmpl0 instead of commit.  Saves 1 mispredict per branch on cold entries. |
| 4 | `bpu-gshare` | 4K-entry PHT with global-history register XOR.  Captures correlated branches (If-else chains in QuickDraw hot loops). |

**Sizing.**  Phase 1 = 64 entries × (15-bit tag + 32-bit target + 2-bit
ctr + valid) ≈ 50 bits × 64 = 3.2 kbit = 1 BRAM18 trivially.
Phase 4 gshare = 4K × 2 bits = 8 kbit (same BRAM18).

**Key contract.**  Entry N is valid for *all* branches whose PC hashes
to N.  Aliasing is tolerated because:
- taken-aliased-with-not-taken degrades into "sometimes right";
- target-aliased resolves via the mispredict check in commit — the
  `rob_br_target != rob_npc_pred` gate (already in place).

**Ownership conflicts.**  `bpu-phase2` touches `bpu.v`, `if_stage.v`,
`iq_int.v`, `alu.v`, `m68k_core.v`.  That's wide — serialize after any
agent currently in iq_int / alu territory.

---

## RAS — return-address stack

**What it is.**  Dedicated 8-entry circular stack pushed on BSR commit,
popped on RTS lookup.  Target predictor for RTS — more accurate than
BTB because it tracks the actual call stack instead of a per-PC
2-bit counter.

**Where it lives.**  `rtl/core/fetch/ras.v` — already on main as a
standalone module (landed from the earlier RAS agent).  Integration
into `if_stage.v` + `commit.v` is pending.

**Phase status.**

| Phase | State | What |
|---|---|---|
| 1.5 core | module ready | Exists, unit-tested. |
| 1.5 tail | `ras-integrate` | Wire BSR push at commit (BSR is 2-μop, the BRA phase carries `cmpl_is_bsr`); RTS pop at if_stage's RTS-predict path.  Fall back to BTB when RAS is empty or disagrees. |

**Key contract.**  Push on BSR retire, pop on RTS fetch-redirect.  On
flush: roll back `spec_top` to the committed depth (mechanism already
in `ras.v`).  Overflow wraps silently — 8 is deeper than any Mac OS
call stack that matters for hot paths.

**Ownership conflicts.**  `if_stage.v` + `commit.v`.  `commit.v` is
touched by any exception-path agent (phase 2) — schedule RAS before
that or be prepared for a rebase.

---

## TLB / MMU — 68040 address translation

**What it is.**  68040 MMU: 4 KB / 8 KB page size, two transparent-
translation register pairs (ITT0/1 + DTT0/1), page-table walker, 64-
entry ATC (split 32 I + 32 D on real hw; we unify to 64).  Privilege +
R/W/X per page.  PMOVE (control move), PFLUSH (invalidate), PTEST
(query) are the management instructions.

**Where it lives.**  `rtl/core/mem/mmu.v` — today a stub.  Integrated
into LSU on the D-side and `if_stage.v` / icache on the I-side.

**Phase status.**

| Phase | State | What |
|---|---|---|
| 2 | `mmu-stub` | ITT0/1 + DTT0/1 only.  All ROM/IO address ranges use transparent translation (VA==PA).  No walker, no ATC, no page fault.  Gets the Mac ROM checksum through without tripping over nothing. |
| 3 | `mmu-full` | Real walker (multi-cycle FSM, backed by L1D reads of page-table entries); 64-entry ATC with LRU; privilege check; page-fault exception path; PMOVE / PFLUSH / PTEST fully implemented. |
| 4 | opt | 2-level ATC (32-entry L0 + 256-entry L1) if 64 becomes a miss-rate bottleneck.  Likely unnecessary for Mac OS. |

**Key contract (phase-2 stub).**

```
// Transparent translation match:
//   if (VA & itt_mask) == itt_base && itt_enable:
//       PA = VA; permit if mode matches
//   else:
//       // no walker yet — STALL and raise bus error?  Or pass through?
//       PA = VA  // phase-2 "trust me" mode
```

The stub takes the pass-through exit outside ITT/DTT matches so the
early ROM doesn't stall waiting for a walker that doesn't exist.
This is what the Quadra ROM does in practice — it sets up ITT0 to
cover the ROM region and DTT0 for the I/O page, then turns on the
MMU.  We pretend we did the walk.

**Ownership conflicts.**
- Phase 2: `rtl/core/mem/mmu.v` standalone + 1-line hooks into LSU
  (`ea → pa` translation) and if_stage (`pc → pc_phys`).
- Phase 3: invasive.  LSU must handle page-fault stall + retry; ROB
  must carry exception state; commit must dispatch page-fault vector.
  Serialize with exception-path agent.

---

## L1I — instruction cache

**What it is.**  4 KB × 4-way SA, 32 B line, LRU, read-only from the
CPU side.  Self-invalidated by D$ coherence on a write to the same
line (SMC support for Mac OS).

**Where it lives.**  `rtl/core/fetch/icache.v` — new file.  `if_stage.v`
queries it instead of issuing AXI reads directly.  Fill via axi-xbar
burst to DDR (ROM region or RAM region, depending on PC).

**Phase status.**

| Phase | State | What |
|---|---|---|
| 2 | `l1i-stub` | Pass-through: if_stage issues single-beat AXI reads.  Correct but slow — fine for ROM cold-start where we're bottlenecked by decode anyway. |
| 3 | `l1i` | Real 4 KB 4-way.  Fill on miss = AXI burst 8×32b beats.  Invalidated on self-modifying-code line write from the D-side. |

**Sizing (real).**  4 KB data = 1 BRAM36 per way × 4 = 4 BRAM36.
Tag: 4 ways × 128 sets × (18-bit tag + valid + 2 LRU) ≈ 11 kbit =
1 BRAM18.  Trivial.

**Key contract.**  1-cycle hit (registered data out).  3-cycle miss
(AR → R burst → data valid).  Returns 32-bit word per if_stage read.
Line size 32 B matches a 4-beat 64-bit AXI burst or 8-beat 32-bit;
pick based on xbar width decision (axi-xbar is targeting 128-bit
data, so 32 B = 2 beats).

**Ownership conflicts.**  `if_stage.v` — conflicts with `bpu-phase2`
and `ras-integrate` (both touch if_stage).  Serialize.

---

## L1D — data cache

**What it is.**  4 KB × 4-way SA, 32 B line, tree-PLRU, **write-back +
write-allocate**.  Consumes `CPUSH` (flush line to memory) and `CINV`
(invalidate without flush) from commit.  Snooped from XDMA writes to
RAM (phase 3+) so the host's DMA loads of ROM images show up
coherently.

**Where it lives.**  `rtl/core/mem/dcache.v`.  Sits between LSU and the
D-side AXI master.  LSU-facing ports are identical to the phase-2
pass-through (`req` / `is_write` / `addr` / `wdata` / `wstrb` →
`rdata` / `rvalid` / `rresp` / `bvalid` / `bresp`) — no client changes.

**Phase status.**

| Phase | State | What |
|---|---|---|
| 2 | `l1d-stub` | Pass-through.  Cold boot doesn't need the cache. |
| 3 | `l1d-real` | Real 4 KB 4-way write-back (this ticket).  PLRU, fill-on-miss, commit-time write-back, exception-entry flush gate, byte-granular writeback mask, CPUSH/CINV port stubbed, SMC snoop export. |
| 3 tail | `l1d-smc` | Wire the SMC snoop (dcache `snoop_valid` + `snoop_addr` → I-cache line-invalidate port). |
| 3 tail | `cpush-cinv` | Real decode + drive of CPUSH / CINV instructions from commit. |
| 4 | opt | Larger (16 KB), higher associativity, maybe non-blocking on multiple outstanding misses. |

### Phase-3 L1D-real: design rationale (landed this ticket)

**Geometry.**  4 KB / 4-way / 32 B lines → 32 sets.  Address slice:
`addr[9:5]` set, `addr[4:2]` word-in-line, `addr[1:0]` byte-in-word,
`addr[31:10]` tag (22 bits).

**Inference choices (FPGA).**
- **Tags + valid + dirty**: registers.  32 sets × 4 ways × (22 tag + 1 V + 1 D) ≈ 3 kbit.  Fits as LUT-FFs; too small to bother with BRAM.
- **Data RAM**: `(* ram_style = "block" *)` hint on a per-way `[31:0]` reg array of 256 entries (set×word).  4 ways × 256 × 32 bits = 32 kbit → 1 RAMB36 per way (~conservative; could share).
- **Tree-PLRU**: 3 bits per set × 32 sets = 96 FFs.  Way-0 hit = {0,0,x}; way-1 = {0,1,x}; way-2 = {1,x,0}; way-3 = {1,x,1}.  Victim = "not most-recent" side at each level.
- **Byte-write mask (bwr)**: per (way, set, word) 4 bits of "which byte lanes has the CPU actually written".  4 ways × 32 sets × 8 words × 4 lanes = 4 kbit.  LUT-FFs.  Only purpose: so the end-of-test flush-all walks back the exact bytes the CPU wrote, not the whole line — which would overwrite the testbench's per-byte oracle (cpu_writes) with fill-time memory contents.

**Hit latency.**  2 cycles end-to-end (IDLE → LOOKUP → response pulse), matching the pass-through's end-to-end timing against the tb's single-beat 1-cycle-latency AXI slave.  Front-end LSU FSM sees no change in contract.

**Miss latency.**  1 (IDLE) + 1 (LOOKUP) + 8 × ~2 (FILL beats at single-beat AXI) + 1 (COMPLETE) = ~20 cycles worst-case on a clean miss.  Dirty-victim eviction adds another 8 × 2 = 16 cycles on rare multi-way conflicts.

**AXI beats.**  Single-beat (not burst).  The existing testbench AXI slave model is single-beat only, and the real KU5P MIG wrapper isn't wired yet.  Converting to true INCR8 bursts is a low-risk follow-up once the MIG lands — the FSM already tracks `beat_cnt`, so the change is local.

**Exception coherence (exc_flush_req / exc_flush_done).**  The exception sequencer (`rtl/core/exception.v`) drives an independent AXI master that bypasses the cache.  Before `exc_fire` or `rte_fire` reach the sequencer, `m68k_core.v`'s exc-gate FSM asserts `flush_all_req` to the dcache and holds the sequencer pulse until `flush_all_done` comes back.  The dcache walks every (way, set) and writes back dirty lines.  Typical exception entry has zero dirty lines → flush_all_done pulses in ~130 cycles (1 scan/set × 4 ways + overhead).  Worst-case (4 KB fully dirty) → ~1200 cycles — acceptable for phase 3 since exceptions are rare at this stage.  Follow-up can either shorten by a dirty-any bitmap short-circuit or architecturally route the sequencer through the cache.

**LSU squash-in-flight fix.**  With a real miss latency of ~20 cycles (up from 2 in the pass-through), a branch mispredict can now squash + re-rename a load's `phys_dst` before the load's AXI response lands.  Without mitigation, the LSU's stale cdb / cmpl pulse would clobber the re-used phys reg.  `lsu.v` now latches `cur_squashed` when `flush_en` fires during `S_LD_WAIT` / `S_LD_GAP` / `S_LD_WAIT2` and suppresses the cdb / cmpl broadcast on completion.  Was latent pre-cache; only surfaced once miss-latency was long enough to open the window (observed first on `jmp_jsr_return`).

**Cache-cooldown fix.**  After firing `bvalid` / `rvalid` (registered, 1-cycle pulse), the LSU holds `req` high for one extra cycle while it samples the response.  Without a cooldown the cache's `S_IDLE` would mistake that leftover `req` for a fresh second request and re-walk the tags.  A `cooldown` reg gates new-request acceptance until `req` drops.

**End-of-test flush hook (`dbg_dcache_flush_req`).**  The fuzz harness (`tools/fuzz/fuzz.py`) compares per-byte CPU writes against Musashi's memory state.  Write-back hides any dirty line that never evicted.  The testbench (`tb_top.cpp`) pulses `dbg_dcache_flush_req` once the sentinel write lands, waits for `dbg_dcache_flush_done`, then does its per-byte compare.  Implemented at the `mac_top` boundary as input/output ports.

**Non-cacheable bypass.**  Addresses outside RAM (0x0000_0000 – 0x03FF_FFFF) and ROM (0x4080_0000 – 0x40BF_FFFF) are non-cacheable and take a direct-AXI bypass path (states `S_BY_LD_R` / `S_BY_ST_AW` / `S_BY_ST_B`).  This covers I/O space and the testbench sentinel at `0xFFFF_0000`.  Matches real-Mac MMU setup: ITT0 covers ROM+RAM cacheable, DTT0 covers I/O non-cacheable.

**Store commit discipline.**  Unchanged from pass-through.  LSU buffers a store in `S_ST_BUF` until `commit_store_en`, then issues `dc_req=1` / `dc_is_write=1` to the cache.  On cache hit: merge bytes into the line, set dirty, set the per-word `bwr`, pulse bvalid.  On cache miss: write-allocate (8-beat fill), apply the merged store, dirty + bwr as above.

**CPUSH / CINV.**  `flush_req` / `inval_req` / `maint_done` ports exist and ack synchronously (single-cycle NOP) — the real decoder + commit drive are a follow-up (`cpush-cinv` agent).

**SMC snoop export.**  `snoop_valid` / `snoop_addr` pulse on any D-side write that lands in the cache (hit or fill-then-write).  Line-aligned address.  Not yet wired to the I-cache — the receiver is the `l1i-smc-snoop` agent.

**Ownership conflicts.**  `lsu.v` — light touch (added `cur_squashed` state).  `m68k_core.v` — moderate (added exc-gate FSM, rewired `exc_fire` / `rte_fire` through it, added `dbg_dcache_flush_*` ports).  `mac_top.v` + `tb_top.cpp` — port plumbing only.  Any concurrent agent touching these files must rebase against `agent/l1d-real` before merge.

---

## L2 — shared second-level cache / point of coherency

> **Re-targeted 2026-07-16.**  Everything below supersedes the original
> "private, CPU-only L2 between L1{I,D} and axi-xbar" plan.  L2 still
> doesn't exist as RTL anywhere (`rtl/**/l2*.v` is a zero-hit grep in
> both repos as of this rewrite) — this section is groundwork for
> whoever picks up the Phase-4 ticket, not a change in progress.  See
> also the "Coherence story" and "Sequencing + conflict summary"
> sections below, which this rewrite also updates.

**What it is.**  Bonus layer not present in the real 68040.  Our
addition for IPC **and** for solving DMA/scan-out coherence in one
place.  Sits **front-of-MIG**: between the AXI crossbar's single
DDR-bound downstream port (`axi_xbar.v`'s S0) and the DDR4 MIG bridge
(`rtl/board/axi_ddr4_mig_bridge.v`).  UltraRAM-backed (KU5P has 18 Mb /
2.25 MB of URAM total).

**Why front-of-MIG, not between L1s and the xbar (the old plan).**
The xbar already funnels *every* upstream master onto one S0 port via
its internal round-robin arbiter (confirmed by reading the live
`axi_xbar.v` 2026-07-16: post-consolidation it's M0 = CPU LSU + boot
FSM time-mux, M1 = host debug/XDMA, M2 = CPU IF, with a permanently
idle 4th internal fan-in slot reserved for DMA).  Putting L2 at that
one already-arbitrated choke point means:
- L2 needs exactly **one** upstream AXI4 slave port (from xbar S0) and
  **one** downstream AXI4 master port (to the MIG bridge) — no bespoke
  N-way arbitration to invent, no separate L1I-side and L1D-side slave
  ports to reconcile.
- **Every** master is naturally a point-of-coherency member — CPU
  L1 misses, host-debug/XDMA writes (ROM staging, live patching), boot
  FSM's cold-boot RAM zero + SD-sector streaming, and any future
  DMA-ramdisk or video-scanout master that gets wired to the xbar
  later.  The old plan's "XDMA/boot-FSM bypass L2" carve-out existed
  only to avoid two independently-updated cache views disagreeing —
  under front-of-MIG there is only ever one view, so *inclusion*
  becomes the simpler design, not exclusion.  This flips the old
  reasoning, it doesn't just relax it.

**Universal pass-through, bimodal allocate policy.**  All of the above
is "every master passes through L2" — but not all traffic *allocates*
into it:

| Traffic class | Masters (today's 3-master xbar + reserved slots) | L2 policy |
|---|---|---|
| CPU data + instruction | M0 (CPU LSU), M2 (CPU IF) | **allocate** — normal cache fill/writeback |
| Host debug / XDMA | M1 | **allocate** — matches the old doc's own reasoning: debug/provisioning traffic needs cache-coherent visibility, same as CPU traffic |
| Boot FSM (cold boot only, mux'd onto M0's physical port) | M0 (boot phase) | **allocate** — small one-shot write burst at cold boot; no reason to special-case it, and it shares M0's physical port anyway |
| Future SCSI-ramdisk DMA (`dma_ctrl.v`, currently a stubbed xbar master with zero live consumers — see `docs/dma_ctrl.md`) | reserved 4th internal xbar slot | **no-allocate-unless-resident** |
| Future video scan-out (VRAM still lives in dedicated URAM today, see "VRAM placement" section below — this row is forward-looking) | not yet an xbar master at all | **no-allocate-unless-resident** |

"No-allocate-unless-resident" (a standard streaming-bypass-with-hit-
promotion policy): on a miss, do **not** pull the line into L2 — avoid
a bulk streaming workload (disk block I/O, video refresh) evicting the
CPU's actual working set.  On a **hit** (the line is already resident,
e.g. the CPU recently wrote that address and it's still cached),
serve it coherently from L2 rather than bypassing to a stale DRAM
copy underneath it.  This is also *precisely* why the L2 sizing table
below doesn't need to grow to accommodate ramdisk/scan-out working
sets — capacity planning only has to reason about CPU-relevant (L1-
miss) traffic, because streaming traffic structurally can't evict it.

**How a master's policy is selected — per-port static, reusing an
existing signal, not a new sideband.**  The maintainer's framing was
right to flag this as a real design question (AxUSER bit? ID-range
convention? per-port static table?) rather than hand-wave it.
Concretely, from reading `axi_xbar.v` 2026-07-16:
- The xbar has **no AxCACHE or AxUSER plumbing today** (confirmed:
  zero `awcache`/`arcache`/`awuser`/`aruser` hits in the file) — adding
  a new per-transaction sideband bit would mean touching every master
  port on `axi_xbar.v`, which is out of scope here and, more
  importantly, unnecessary: scan-out and the ramdisk DMA are
  **architecturally fixed masters**, not per-transaction choices, so a
  per-transaction hint is the wrong granularity anyway.
- The xbar **already** widens every ID reaching S0 by `SLOT_W=2` bits
  (`XID_WIDTH = ID_WIDTH + 2`) to carry the winning internal fan-in
  slot index, purely so the B/R response can be routed back to the
  right master (`s0_rtgt = s0_rid[XID_WIDTH-1 -: SLOT_W]` and the
  write-side equivalent).  That tag is already present on every single
  transaction that reaches L2's upstream port, at zero extra cost.
- **Chosen mechanism**: L2 decodes the same top `SLOT_W` bits of the
  incoming extended ID as a **static per-slot policy table** — slots
  0-2 (CPU LSU+boot, host debug, CPU IF) = allocate; the reserved 4th
  slot (DMA/ramdisk, and whatever future master ends up sharing or
  extending that reservation for video scan-out) = no-allocate-unless-
  resident.  Zero new AXI signals, zero `axi_xbar.v` edits required —
  the mechanism the crossbar already has for response routing doubles
  as the allocate-policy selector.  **Open item for whoever wires a
  real video-scanout xbar master later**: the internal fan-in arrays
  are hardcoded 4-wide (`NW`/`NR` local params) with all 4 slots now
  spoken for (CPU-LSU/boot, host-debug, CPU-IF, DMA-reserved) — a
  second no-allocate master (scan-out) either shares the DMA slot's
  no-allocate policy bit (fine, since both get the same policy anyway)
  by also routing through `dma_ctrl.v`, or the xbar's fan-in arrays
  need widening to 5.  Flag this for the crossbar owner when that work
  starts; not decided here.

**Allocating traffic is normalized to full-line width at the MIG
boundary; no-allocate traffic is not.**  Because L2 always fills and
evicts whole lines, every DRAM access *for allocating masters*
downstream of L2 becomes a full-line (64 B) burst — CPU LSU/IF, host
debug, and boot FSM never issue anything narrower than a full-line
fetch/writeback once L2 is in the picture, regardless of how narrow
the original upstream request was (a 1-byte CPU load still triggers a
64 B line fill).  This is a real simplification of the MIG-facing
interface: the bridge only has to reason about one shape (aligned
full-line INCR bursts) for the allocating path.  No-allocate-unless-
resident traffic (scan-out reads, ramdisk fetches/stores) is the
**exception** — on a miss it passes through L2 at its own natural,
narrower request width (bypassing the line-fill logic entirely,
per the policy above); it only ever sees the full-line path if it
happens to hit an already-resident line written by an allocating
master.  **Design choice: one downstream AXI4 master port, bimodal
transaction shape** — not two physical ports.  AXI4 already carries
variable burst length per transaction (`AWLEN`/`ARLEN`), so there is
no protocol reason to split allocate/no-allocate traffic onto separate
physical ports; L2's internal FSM picks the burst shape per
transaction based on the same slot-tag policy lookup above, and the
MIG bridge (see dependency below) doesn't need to know or care which
mode produced a given burst.

**New, explicit dependency this creates on the MIG bridge.**  A
64 B line fill/writeback is an aligned INCR burst of 4× 128-bit
repo-side beats (`AWSIZE=4`, `AWADDR[3:0]=0`, `AWLEN=3`).  Checked
against the current `rtl/board/axi_ddr4_mig_bridge.v` 2026-07-16: this
shape is **already implemented**, not missing — the bridge's
`wr_aw_beat_ok` / `rd_ar_beat_ok` paths already accept aligned
128-bit-size INCR bursts of arbitrary length and pack them into the
256-bit MIG-side beats (2 repo beats per MIG beat), independent of the
narrow single-beat path used for non-cacheable/no-allocate traffic.
So this is **not** "add burst support from scratch."  What genuinely
*is* still missing, and is a real prerequisite before L2 can be built
for real:
1. **Real hardware calibration.**  Per `docs/fpga_ddr_firstlight_report.md`
   (2026-04-22, still current as of this rewrite): "No board evidence
   yet for DDR calibration in this top."  The burst path above has
   never been exercised on real silicon, only reasoned about from RTL.
2. **Single-outstanding-per-direction, not pipelined.**  The bridge
   serializes one write and one read transaction at a time (each
   direction has its own tiny state machine, so a read and a write
   *can* overlap, but two independent bursts on the same direction
   cannot).  A first L2 design should keep at most one line-fill and
   one writeback in flight at a time to match this — consistent with
   the existing capability, not a new ask, but worth stating so nobody
   designs a multi-outstanding-miss L2 against a bridge that can't
   serve it yet.
3. **Sustained/contended bandwidth and arbitration latency are
   unmeasured.**  No numbers exist anywhere in either repo's docs for
   what happens when the bridge is driven continuously by cache-line
   traffic instead of occasional single-beat pokes.

Net: L2 is gated on Phase-3 real L1D/L1I landing (unchanged from the
original plan) **and** on DDR4 reaching calibrated status on real
hardware — the burst *shape* L2 needs already exists in the bridge
RTL, but nobody has proven it works on the board.  See
[[project_ddr4_mig_bridge_first_light_only]] (PM agent memory) /
`docs/fpga_ddr_firstlight_report.md` for the calibration gate.

**Repo location — platform (`macqd700-soc`), not CPU (`m68k-ooo`).**
The original doc's file pointer (`rtl/sys/l2.v`) was ambiguous even
before the repo split, and is doubly stale now: (a) `rtl/sys/` is the
*CPU* submodule's own internal directory name (fetch→AXI adapters
etc., see `m68k-ooo/CLAUDE.md`'s "Repo Split" section) — it is not
this repo's convention, which uses `rtl/soc/` for AXI fabric / DMA /
crossbar (`axi_xbar.v`, `dma_ctrl.v` both live there) and `rtl/board/`
for board-specific bridges (`axi_ddr4_mig_bridge.v`).  (b) More
fundamentally, front-of-MIG placement settles the question on its own
merits, not just naming: L2 sits strictly downstream of the
platform-owned crossbar and strictly upstream of the platform-owned
MIG bridge, and every master it needs to reason about (CPU LSU/IF via
the CPU socket, host debug, boot FSM, future DMA/scan-out) is a
platform-level AXI participant, not internal CPU pipeline state.  Per
`m68k-ooo/CLAUDE.md`'s repo split ("AXI fabric, DDR4/MIG... lives in
`macqd700-soc`"), this is unambiguously platform territory.
**Conclusion: `rtl/soc/l2.v` in `macqd700-soc`, analogous in placement
to `axi_xbar.v` and `dma_ctrl.v`.**

**Interface-contract sketch (NOT a real module — comment-only, for
whoever starts Phase-4 L2 design).  Do not create `rtl/soc/l2.v` with
real cache logic yet; this is gated per above.**

```verilog
// rtl/soc/l2.v — sketch only, not implemented.  Front-of-MIG L2 /
// point of coherency.  Gated on: Phase-3 real L1D/L1I landing in
// m68k-ooo, AND real-hardware DDR4 calibration (see
// docs/fpga_ddr_firstlight_report.md).
//
// module l2 #(
//     parameter DATA_WIDTH   = 128,          // matches xbar S0 / MIG bridge repo side
//     parameter ID_WIDTH     = 6,             // XID_WIDTH from axi_xbar.v (ID_WIDTH+SLOT_W)
//     parameter SLOT_W       = 2,             // reused xbar slot-index field width
//     parameter ADDR_WIDTH   = 32,
//     parameter LINE_BYTES   = 64,
//     parameter WAYS         = 8,
//     parameter SETS         = 512,           // 64B * 8 * 512 = 256 KiB
//     // Static per-slot allocate policy, indexed by the top SLOT_W bits
//     // of the incoming AxID.  1 = allocate, 0 = no-allocate-unless-resident.
//     parameter [3:0] SLOT_ALLOCATE_MASK = 4'b0111  // slots 0,1,2 allocate; slot 3 (DMA/scan-out) does not
// )(
//     input  wire clk, rst,
//
//     // Upstream AXI4 slave — from axi_xbar.v S0.  Same shape as today's
//     // S0->ddr_ctrl wiring in fpga_top_ddr.vh; L2 is inserted inline,
//     // no xbar changes needed.
//     input/output ... s_aw*, s_w*, s_b*, s_ar*, s_r*   // AXI4, DATA_WIDTH/ID_WIDTH/ADDR_WIDTH above
//
//     // Downstream AXI4 master — to rtl/board/axi_ddr4_mig_bridge.v's
//     // repo-side slave port.  ONE port, bimodal transaction shape:
//     //   allocate hit/fill/writeback  -> aligned full-LINE_BYTES INCR burst (AWSIZE=4, AWLEN=LINE_BYTES/16-1)
//     //   no-allocate pass-through     -> forwards the original narrower request shape unchanged
//     output/input ... m_aw*, m_w*, m_b*, m_ar*, m_r*
//
//     // Debug/maintenance (mirrors the L1D CPUSH/CINV port shape in
//     // dcache.v, extended to whole-of-L2 scope):
//     input  wire flush_all_req, output wire flush_all_done,
//     input  wire [ADDR_WIDTH-1:0] inval_addr, input wire inval_req, output wire inval_done
// );
```

**Phase status.**

| Phase | State | What |
|---|---|---|
| 4 | `l2` | 256 KB, 8-way SA, 64 B line (2× L1 line), write-back, inclusive of L1s, front-of-MIG, universal-master pass-through with allocate/no-allocate-unless-resident split.  Non-blocking within the single-outstanding-per-direction limit of the MIG bridge (see dependency above). |

**Sizing.**  256 KB total capacity is unchanged and still the right
target — see the "why skip until phase 4" analysis below for the
performance case, and the no-allocate-unless-resident policy above for
why growing it to accommodate ramdisk/scan-out working sets is *not*
needed (they structurally can't evict CPU lines on a miss).  The
**physical URAM288 count is an open, unreconciled number** — flagging
rather than inventing one, since real design work is still gated:
- This doc's old math ("256 KB = 4 × 64 KB URAM blocks = 4 URAM288")
  undercounts: a single URAM288 cascaded to 128-bit width (matching
  the VRAM section's own methodology below) provides 64 KB *per
  cascaded pair*, i.e. **2** physical URAM288 primitives, not 1 — so
  "4 × 64 KB logical blocks" is really 8 physical URAM288s if ways
  don't need independent parallel ports.
- `docs/optimisation_roadmap.md` §6a independently estimated "~16 URAM
  blocks for 256KB" — consistent with giving each of the 8 ways its
  own cascaded-pair (2 URAM288 × 8 ways = 16), which is the standard
  set-associative pattern this doc's own L1D section uses ("1 RAMB36
  per way") for parallel tag+data lookup.
- Which is right depends on the still-open hit-latency micro-design:
  this section already specs **L2 hit = 4 cycles** (not 1, unlike L1),
  so a fully-parallel 8-way-in-1-cycle bank (16 URAM288) is not
  mandatory — a time-multiplexed narrower bank count amortized across
  the 4-cycle hit window could plausibly land closer to 8.  **Not
  resolving this here** — real Phase-4 design should pick a number
  against the actual hit-latency pipeline, and reconcile both docs
  when it does.
- **For URAM-budget planning purposes** (the table in the VRAM section
  below), use the conservative **16 URAM288 (25% of KU5P's 64)**
  until Phase-4 design lands a real number — better to over-budget
  than re-discover a resource cliff late.

Tags = 8 ways × 512 sets × (14-bit tag + valid + dirty + LRU bits) ≈
75 kbit = ~3 BRAM36 (unchanged from the original estimate; tags don't
scale with the URAM288-count question above, which is data-storage
parallelism only).

**Key contract.**  L1 miss issues to L2.  L2 hit = 4 cycles.  L2
miss = ~20 cycles (DDR4 round-trip).  Writeback from L1 to L2 is
filtered: L2 only stores what L1 evicted.  Inclusion policy:
*inclusive* — if L2 evicts a line, it back-invalidates L1 copies
(simplifies snoop, adds small complication to eviction FSM).  This
inclusion directory is also the mechanism that subsumes SCSI-DMA /
future-ramdisk coherence (see "Coherence story" below): any external
write to a resident line invalidates the L1 copy, not just on L2's own
eviction.

**Why skip until phase 4.**  On Mac OS, 4 KB L1D has a ~70% hit
rate for QuickDraw hot paths (measured on SheepShaver traces).  The
remaining 30% miss to DDR at ~20 cycles average.  Adding L2 cuts
that to ~6 cycles average — measurable speedup, but dwarfed by the
wins phase 3 still has on the table (real L1s, gshare BPU, MMU
walker).

**Future motivation, confirmed 2026-07-16 (not just IPC).**  The
maintainer confirmed 16/24bpp QuickDraw ("would be damn nice") as an
**intended, confirmed direction** for v1+ — not speculative, not a
maybe-drop item.  Real Mac 16/24bpp framebuffers don't fit in KU5P
URAM (see "URAM capacity — BPP > 16 DOES NOT FIT" below); the only
realistic path is moving VRAM to DDR4-backed storage with this L2 as
the CPU-write/scan-out-read coherency point (scan-out would join the
no-allocate-unless-resident traffic class above).  This does **not**
change the gating: still blocked on Phase-3 real L1D/L1I, real-
hardware DDR4 calibration, and now also the MIG-bridge burst-support
validation above.  Today's URAM-backed VRAM stays as-is until those
gates clear (see the VRAM section below and
[[project_l2_vram_placement_decision_2026_07_16]] in PM agent memory
for the full investigation).

**Ownership conflicts.**  Green-field on the RTL side.  On the design
side, genuinely couples to two things that used to be independent:
(1) the AXI crossbar's S0 port shape and slot-tag convention (see
"Crossbar interaction" in the sequencing section below), and (2) the
MIG bridge's burst path (see dependency above) — schedule after both
L1s land, after DDR4 calibration, and coordinate with whoever owns
the crossbar at the time.

---

## Coherence story (phase 3+)

> **Updated 2026-07-16** for the front-of-MIG L2 re-target above.  The
> old version of this section had XDMA bypass L1D via a manual
> `DBG_CACHE_INVAL` poke-and-continue protocol.  That workaround
> existed because there was no shared structure between "CPU's cache"
> and "DMA's view of RAM."  Once L2 sits front-of-MIG as a universal
> pass-through with an inclusion directory, that structure exists —
> so item 1 below is simplified, and items 2/3 are unchanged in kind
> but now flow through the same mechanism instead of a bespoke one.

Three things can write to RAM addresses the CPU has cached:

1. **XDMA / host debug DMA**.  For debug + ROM loading.  Frequency:
   during provisioning + occasional debug.  **New solution (L2
   front-of-MIG)**: XDMA is an ordinary allocating master through L2
   (see the L2 section's traffic-class table) — its writes update L2
   directly, and L2's inclusion directory back-invalidates any L1D
   copy of the same line, same as any other external write to a
   resident line.  No more manual `DBG_CACHE_INVAL` poke-and-continue
   protocol needed once L2 lands; `debug_ctrl`'s `DBG_CACHE_INVAL`
   register can stay as a manual escape hatch but stops being load-
   bearing for correctness.  **Until L2 lands**, the old bypass-and-
   flush protocol is still how this works — this is a phase-4-gated
   simplification, not a change to today's behaviour.

2. **SCSI DMA** (phase 3 once scsi.v has a real disk path) / **future
   DDR4-backed SCSI ramdisk DMA** (`dma_ctrl.v`, see
   [[project_ddr4_scsi_ramdisk_design_2026_07_16]] in PM agent memory
   — designed but not yet built).  Higher frequency: whenever Mac OS
   reads a disk block.  **New solution (L2 front-of-MIG)**: ramdisk
   DMA is a **no-allocate-unless-resident** master through L2 (see the
   L2 section) — on a miss it reads/writes DRAM directly without
   disturbing L2's contents; on a hit (the CPU has a dirty/resident
   copy of that block), L2's inclusion directory serves it coherently
   and, symmetrically, a DMA write to a resident line invalidates the
   stale L1D copy the same way an XDMA write would.  This subsumes
   the SD-backed SCSI path's original two options (bus-snoop vs.
   software CPUSH) for free once L2 exists — no separate snoop port
   needed on `dcache.v`.  **Until L2 lands**, SD-backed SCSI DMA still
   needs one of the two original mechanisms (bus-snoop preferred, per
   the 68040 spec and MAME-matched Mac OS behaviour) — do that when
   real SCSI lands in phase 3, independent of the L2 timeline.

3. **Self-modifying code** (Mac OS patches trap stubs).  Frequency:
   every boot.  **Unchanged** — this is an L1I/L1D-only concern (SMC
   snoop, see the L1D section above) and doesn't route through L2 or
   DDR at all in the steady-state hit case.  Solution: on any D-side
   write that hits a line also cached in L1I, invalidate the I-line.

Phase 2 has none of these problems because we have no caches yet.

---

## Sequencing + conflict summary

```
phase 1.5 tail ┃ bpu-phase2 ───► ras-integrate ───┐
                                                  │
phase 2        ┃ mmu-stub ──► l1i-stub ──► l1d-stub ──► (platform agents)
                                                  │
phase 3        ┃ l1i ──► l1d ──► mmu-full ──► l1d-smc
                    │        │          │
                    │        │          └► requires exception-path
                    │        └► conflicts with lsu-disambig (serialize)
                    └► conflicts with bpu-phase2 (serialize)
                                                  │
phase 4        ┃ l2 ──► bpu-gshare ──► (optional: 2-level MMU, prefetch)
```

All of these except `l2` conflict with core files that other agents
currently touch.  Rule of thumb: each memhier agent gets an isolated
worktree, rebases against the moving main, merges when the full test
suite passes.

### `l2`'s dependencies, updated 2026-07-16 (front-of-MIG re-target)

`l2` is now a **platform-repo** (`macqd700-soc`) ticket, not a
`m68k-ooo` one — see the repo-location conclusion in the L2 section
above.  Its dependency set grew from "both L1s land" to:

1. **Phase-3 real L1D/L1I land** (`m68k-ooo`, unchanged).
2. **Real-hardware DDR4 calibration** (`macqd700-soc`) — new, see
   `docs/fpga_ddr_firstlight_report.md`.  Not started as of this
   rewrite.
3. **MIG-bridge full-line-burst validation on real hardware** — the
   burst *shape* L2 needs already exists in
   `rtl/board/axi_ddr4_mig_bridge.v` (confirmed by reading the RTL,
   see the L2 section), so this folds into (2)'s calibration work
   rather than being separate RTL work, but call it out explicitly
   since "DDR4 needs calibration" alone undersells what needs proving.

**Crossbar interaction — confirmed, not just assumed.**  The AXI
master-consolidation work landed in-tree 2026-07-16 (uncommitted at
time of writing, `rtl/soc/axi_xbar.v`: 5 masters → 3, M0 = CPU LSU +
boot FSM time-multiplexed, M1 = host debug, M2 = CPU IF, M4/DMA
master port stubbed out with its internal fan-in slot reserved-but-
idle) **simplifies** L2's design surface, as expected, but the
mechanism matters and is worth recording: L2's allocate/no-allocate
policy selector (see the L2 section) directly reuses the crossbar's
existing `SLOT_W`-bit response-routing tag on the extended ID reaching
S0.  Fewer live masters today means fewer slot values L2 needs a
real policy entry for right now (0/1/2 = allocate; the idle 4th slot
is future no-allocate) — but the mechanism itself (decode the top
`SLOT_W` bits of the incoming ID) is **independent of how many
masters are currently wired**, so L2's design doesn't need to be
revisited if/when a real DMA-ramdisk or video-scanout master gets
wired back onto that reserved slot later.  One real open item this
consolidation surfaces: the internal fan-in arrays are hardcoded
4-wide with all 4 slots now spoken for (3 live + 1 reserved) — a
*second* no-allocate master (e.g. video scan-out, if it ever needs
its own xbar master distinct from DMA-ramdisk) will need the crossbar
owner to widen `NW`/`NR`, which is real scope for that future ticket,
not for L2 itself.

---

## VRAM placement — URAM, not DDR

**Decision** (2026-04-16): the Mac-resolution framebuffer lives in
KU5P UltraRAM, not DDR4.  HDMI scan-out reads URAM directly; the
scaler up-converts to 1920×1080p60 for HDMI output as in the existing
`~/sd-hdmi-bringup` pipeline.

### URAM inference — xpm_memory_tdpram, NOT inferred 2D reg array

**Update (2026-04-18)** — the first full synth of `fpga_top` with
FB_BPP=24 landed Vivado in a degenerate state: **221,184 RAM256X1D
distributed-LUT primitives** were inferred for `u_vram`, blowing the
tech-mapper out of memory.  Root cause: with two INDEPENDENT clocks
on the two ports (core_clk @ 200 MHz for AXI, pclk @ 148.5 MHz for
scan-out), Vivado's URAM-inference rule set does NOT fire on
`(* ram_style = "ultra" *) reg [...] mem [0:N-1]`.  XST quietly fell
back to LUTRAM per byte-lane, which at 18 Mbit detonates at elab.

**Fix (same commit)** — instantiate `xpm_memory_tdpram` directly:

```verilog
xpm_memory_tdpram #(
    .MEMORY_PRIMITIVE   ("ultra"),
    .CLOCKING_MODE      ("independent_clock"),
    .BYTE_WRITE_WIDTH_A (8),         // per-lane wstrb
    .WRITE_MODE_A       ("no_change"),
    .WRITE_MODE_B       ("no_change"),
    .READ_LATENCY_A     (1),
    .READ_LATENCY_B     (1),
    .WRITE_DATA_WIDTH_A (DATA_WIDTH),
    .WRITE_DATA_WIDTH_B (DATA_WIDTH),
    // ... port-A clocking (clk, rst), port-B clocking (rd_clk, rd_rst)
) u_uram ( ... );
```

The XPM primitive explicitly supports asynchronous-clock dual-port
URAM, hits `MEMORY_PRIMITIVE="ultra"` unconditionally, and gives us
native byte-write-enable for AXI wstrb masking.  Verilator can't
compile the XPM SV — so the module carries a bit-identical
behavioural 2D-register model under `ifdef VERILATOR` for sim.

### URAM capacity — BPP > 16 DOES NOT FIT

KU5P has **64 URAM288** primitives.  One URAM288 is 4096 deep × 72
bit wide.  For `vram.v` at `DATA_WIDTH=128`, Vivado always cascades
2 URAMs wide (2×72 ≥ 128) — so the effective primitive is 4096 deep
× 128 bit.  Depth cascades to reach `N_WORDS`.

| FB_WIDTH_PX × FB_HEIGHT_PX | BPP | N_WORDS | Deep × 2-wide | URAM288 count | % KU5P |
|---|---|---|---|---|---|
| 1024 × 768 | **8**  |  49,152 | 12 × 2 | **24** | 37.5 % |
| 1024 × 768 | 16 |  98,304 | 24 × 2 | 48 | 75 % |
| 1024 × 768 | 24 | 153,600\* | 38 × 2 | 76 | 119 % — **NO** |
| 1024 × 768 | 32 | 196,608 | 48 × 2 | 96 | 150 % — **NO** |

\*: at BPP=24, 128/24 = 5.33 pixels per word, so `vram_smoke.v` packs
5 pixels per word and wastes 8 bits per word — pushing N_WORDS even
higher than a linear BPP calculation would suggest.  Either way,
BPP=24 doesn't fit.

`vram.v` hard-errors at elab on `BPP > 16`:

```verilog
generate if (BPP > 16) begin : gen_bpp_oversize
    initial $error("[vram] VRAM URAM capacity exceeded at BPP=%0d ...", BPP);
end endgenerate
```

This kills the "silent fallback to 221k distributed-LUTs" failure
mode.  Any future caller who wants 16/24-bit Mac modes must route the
framebuffer through DDR (outside `vram.v`'s scope).

### Current bitstream default: FB_BPP=8

`fpga_top.v` sets `FB_BPP = 8` (indexed-colour Mac mode).  Real Mac
8bpp compatibility is preserved; the scaler handles BPP=8 by
greyscale-expanding the palette index (`rtl/mac/video/scaler.v`
line 209).  Task #91 ROM (Quadra 700) boots in indexed-8 by default
so this is the correct bring-up mode.

**16/24bpp QuickDraw — confirmed future direction, 2026-07-16.**  The
maintainer confirmed 16/24bpp is wanted ("would be damn nice") — this
is now a **confirmed intended direction for v1+, not a speculative or
maybe-drop item**.  Since URAM structurally cannot hold a 16/24bpp
framebuffer at any Mac resolution ≥1024×768 (see the hard-error above),
the only realistic path is DDR4-backed VRAM with the front-of-MIG L2
(see "L2 — shared second-level cache" above) as the coherency point
between CPU QuickDraw writes and HDMI scan-out reads (scan-out would
be a no-allocate-unless-resident L2 master, same policy class as the
future SCSI ramdisk).  **This does not move up the schedule** — it is
still gated on Phase-3 real L1D/L1I, real-hardware DDR4 calibration,
and MIG-bridge burst validation, exactly like L2 itself.  Today's
URAM-backed 8bpp VRAM is unaffected and stays as the shipping default
until those gates clear.  Full investigation:
[[project_l2_vram_placement_decision_2026_07_16]] (PM agent memory).



**Sizing.**  URAM budget = 64 URAM288 primitives total (18 Mb / 2.25 MB).
Planned allocation, **updated 2026-07-16** to use primitive-count
(the actual allocatable unit) consistently rather than mixing it with
raw-byte-capacity percentages, which is what caused this table and
the capacity table above to disagree slightly in the pre-2026-07-16
version of this doc:

| Consumer | URAM288 primitives | % of KU5P's 64 |
|---|---|---|
| Phase-3 VRAM (1024×768×8bpp) | 24 | 37.5 % |
| Phase-4 L2 (256 KB, 8-way — see L2 section for the 4-vs-16 reconciliation) | 16 (conservative planning figure; real number TBD at Phase-4 design time, could land as low as ~8) | 25 % |
| **Headroom** | 24 | 37.5 % |

Even with both landed, there's comfortable headroom.  Native
1024×768×16bpp or higher BPP does **NOT** fit in URAM at all (see
"URAM capacity — BPP > 16 DOES NOT FIT" above) — that's the load-
bearing reason the confirmed 16/24bpp direction (see L2 section above)
requires moving VRAM to DDR4, not just adding more URAM budget.
Native 1920×1080×24bpp (6.2 MB) does NOT fit in URAM either way — but
Mac OS expects Mac resolutions, so the scaler does the final
conversion on read regardless of where VRAM ends up living.

**Wins.**
- Deterministic 1-cycle VRAM reads → HDMI scan-out needs only a
  trivial line-buffer FIFO, no deep DDR-jitter absorber.
- Zero DDR contention from scan-out.  CPU + boot FSM + XDMA own DDR
  uncontested.
- CPU framebuffer writes (Mac OS QuickDraw pokes VRAM directly) hit
  URAM single-cycle with no D$/L2 coherence concern — writes are
  write-combining, bypass L1D.

**Architecture impact.**
- `axi_xbar.v` gains a third slave (S2 = VRAM URAM write port).  CPU
  + XDMA can write VRAM through the xbar; boot FSM does not touch
  VRAM.
- The HDMI scan-out master (M3 on the current xbar) is **removed** —
  HDMI reads URAM directly via the URAM's second native port,
  bypassing the xbar entirely.  Final topology: **3M × 3S** (CPU,
  XDMA, boot FSM × DDR, peripheral bus, VRAM).
- A `vram` module wraps the URAM block(s), exposing: AXI4 slave
  (writes from xbar), and a read-only streaming port (for HDMI
  scan-out + scaler).  The streaming port is a 2-cycle registered read
  path so Vivado can use the URAM output pipeline before the pixel lane mux.
- First xbar merge is a valid 4M×2S baseline; the follow-up retune
  is an additive feature ticket (`axi-xbar-vram`) — schedule after
  the first `hdmi-video` module exists so the URAM reader port can
  be specified against real consumers.

---

## Decisions (baked in from phase-2 review)

| Question | Decision | Rationale |
|---|---|---|
| L1 cache line size | **32 B** | Matches 68040 spec; 2× 128-bit AXI beats per line. |
| L2 cache line size | **64 B** | 2× L1 line → simple inclusion / refill. |
| L1I / L1D split | **4 KB / 4 KB, 4-way** | 68040 spec.  Revisit in phase 4. |
| TLB layout | **Unified 64-entry ATC** | Simpler PFLUSH snoop.  Mac OS working set small. |
| L2 inclusion | **Inclusive of L1s** | Back-invalidate on L2 eviction — simplest coherence. |
| Phase-4 BPU | **gshare (4K PHT + GHR XOR)** | Simplicity-first.  Upgrade to TAGE only if specific mispredict patterns warrant. |
| VRAM location (today) | **URAM on-chip** | See section above. |
| L2 placement (updated 2026-07-16) | **Front-of-MIG** (between axi-xbar S0 and the DDR4 MIG bridge), not between L1{I,D} and axi-xbar | Single point of coherency for every xbar master with one AXI port each side; see L2 section above. Platform-repo (`macqd700-soc/rtl/soc/l2.v`), not CPU-repo. |
| L2 allocate policy | **Per-slot static, reusing the xbar's existing `SLOT_W`-bit response-routing ID tag** | CPU/debug/boot = allocate; DMA-ramdisk/scan-out = no-allocate-unless-resident. No new AXI signals, no `axi_xbar.v` edits. See L2 section above. |
| VRAM future direction | **16/24bpp via DDR4 + L2, confirmed intended (2026-07-16), not scheduled** | Gated on Phase-3 L1s + DDR4 calibration + MIG burst validation. See "16/24bpp QuickDraw" note above. |
