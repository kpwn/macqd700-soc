# Fabric concurrency contract

**Status:** RATIFIED, v1.0. This document is authoritative for the SoC's
memory-concurrency target in the same way `rtl/soc/cpu_socket.vh` is
authoritative for the socket's AXI widths. It is that file's sibling:
`cpu_socket.vh` fixes the *shape* of the seam, this fixes the *depth*.

**Version:** 1.0 — 2026-09-03.
**Ratified against:** SoC `85ee76d`, `cpu040` submodule `22b16f4`.
**Supersedes as a target statement:** `docs/soc_bus_review.md` §7a and §8
Phase 2, and `docs/fabric_concurrency_and_soc_review_2026-09-02.md` §1/§4.
Those remain the analysis and the costing; this is the ratified target.

---

## 0. Why this document exists, and how to use it

`docs/fabric_concurrency_and_soc_review_2026-09-02.md` §4 names a recurring
failure mode in this repo: two components each defer improving their own side
because the other "isn't ready to benefit". The standoff self-perpetuates, and
its residue is a class of comments that justify a local limit by quoting a
*neighbour's present-day behaviour* — `if_to_axi.v:16-18` ("the core's
if_stage already self-serialises"), `axi_narrow_to_wide.v:72-76` ("both the
core LSU and boot_fsm self-serialise"). Those comments were true when written
and are false now, and nothing made them fail loudly.

The fix is to make the *contract* the thing modules implement against:

> Implement to the clause. Cite the clause. Never cite the neighbour.

A module that is built to C3 and says so stays correct when the crossbar,
the L2C or the core changes underneath it. A module that says "the thing
above me only issues one at a time" is wrong the day that stops being true,
silently, and reads as "this is fine" to whoever is debugging why the machine
is slow.

Two consequences, both load-bearing:

- **Capability is built unilaterally.** Depth, ID space and width are built to
  the clause *before* the neighbour can reward it. This project has run that
  experiment three times and won each time: the Part 88 AXI boundary register
  slice (+107 FF, +1.88ns WNS, coupling dissolved), `AxiIds.scala`'s reserved
  4-MSHR D-side ID range, and `dma_engine.sv`'s 8 unique IDs built against a
  fabric that caps it at 1. See §5 for the equally important exception.
- **Clause numbers are stable.** C-numbers are never reused or renumbered.
  A clause that is retired is marked RETIRED in place, with the reason.

---

## 1. The clauses

| # | Clause | Applies to | Status | Evidence |
|---|---|---|---|---|
| **C1** | A read master that can have N misses outstanding SHALL present all N to the fabric concurrently. Target N ≥ 4. | every read master | **PARTIAL** | I-side MET (5 MSHRs, `IcachePlugin.scala:313` `MSHR_N = 1 + pfSlots`); DMA MET (8 IDs, unmerged); **D-side OPEN** (`AxiIds.scala:39`, `N_MSHR == 1`) |
| **C2** | Concurrent reads from one master SHALL each carry a distinct ARID. No master may hold two reads in flight under one ID. | every read master | **MET** | `AxiIds.scala` (`dRefill(k)`, `iRefill(k)` — ID *is* the MSHR index); `dma_engine.sv:75,101-102` (`id_live`/`free_id`) |
| **C3** | The crossbar SHALL accept ≥ 4 outstanding reads per master port. | `axi_xbar.v` | **OPEN** | scalar `rs_state[mi]`/`RS_IDLE` gates every AR (`axi_xbar.v` read-path block, ~:3095-3235) |
| **C4** | Every L2C header door SHALL accept the next header while the current burst is still decomposing (one-entry pre-latch per channel). | `l2c_ctrl.v` | **MET** | LSU: `ar_pend_v`/`aw_pend_v`, `s_arready = !ar_pend_v` (~:335-352). Fetch: `fetch_ar_pend_v`, commit **`51d2b0e`** |
| **C5** | Same-ID gating SHALL block only the actual hazard, never every live same-ID op. | `l2c_ctrl.v` | **PARTIAL** | `a28d5f7` (fetch/LSU ID-namespace split in `pipe_id_haz_c`); cache-path ordering moved to S2 (`ord_now_block_c`/`ord_merge_block_c`). Residual: C6 |
| **C6** | A 64 B line fill SHALL cost no more than the L2C's own quadrant floor. | `l2c_ctrl.v` + `cpu040` I-side | **OPEN** | measured 4.97 cyc/burst against a 4.0 quadrant floor |
| **C7** | The DDR path below the L2C SHALL sustain ≥ 8 outstanding reads and ≥ 4 outstanding writes. | `axi_ddr4_mig_bridge.v` | **MET** | `axi_ddr4_mig_bridge.v:15-33` (`RMAX_OUTSTANDING = 8`, `WMAX_OUTSTANDING = 4`) |
| **C8** | The VRAM lane mux SHALL not throttle bulk fills below the crossbar's per-master budget. | `axi_vram_priority_mux3.v` | **MET** | `axi_vram_priority_mux3.v:196` `MAX_BULK_AHEAD = 4`, `:203` `MAX_BULK_AHEAD_QUIET = 8` (adaptive, `bulk_cap_c` at `:353`). Already raised from 2 -- review S5's F3 |
| **C9** | **Write-side streaming throughput: MET AND CLOSED.** No new write concurrency is a target of this contract. | whole write path | **MET — CLOSED** | `00f079c` + `980342c`; `tb-l2c-wstream` (see §3) |
| **C10** | A write master MAY reuse one AWID; the fabric guarantees in-order B per ID, and no master may depend on out-of-order B. | every write master | **MET** | `AxiIds.scala` `D_STORE`; `axi_xbar.v` `sw_owner`/`sw_owned` (see §5) |
| **C11** | No door's AXI `*ready` output may be a combinational function of its accept/hazard cone. Depth is bought with a pre-latch, not with a wider `ready` expression. | every door in the fabric | **MET** | the explicit construction rule of both C4 pre-latches (`l2c_ctrl.v`, `s_arready` comment) |
| **C12** | ID space: 16 IDs per socket master (`CPU_SOCKET_AXI_IW = 4`), plus a 2-bit crossbar slot tag, reaching the L2C's `ID_WIDTH = 6`. C3 needs no ID widening. | socket + `axi_xbar.v` | **MET** | `cpu_socket.vh`; `axi_xbar.v:282` `XID_WIDTH = ID_WIDTH + 2`, `:775` `SLOT_W = 2`, `:3408` `s0_rtgt = s0_rid[XID_WIDTH-1 -: SLOT_W]` |
| **C13** | Table-walk memory traffic (ITLB/DTLB descriptor reads AND the U/M-bit writeback) SHALL be served through `DcacheService`, not a private AXI master. The walkers SHALL NOT carry their own `Axi4` port. | `TableWalker`, `ItlbPlugin`, `DtlbPlugin`, `LsEuPlugin` | **OPEN — CONFIRMED BUG** | `dtlbAxi`/`itlbAxi` are separate masters merged only at the bus (`AxiDMerge`), bypassing the L1D array. Fully designed and unbuilt: spec `2026-08-18-walker-dcache-passthrough-design.md` (W1-W30, signed off round 7) + its 19-task plan |

---

## 2. Clause detail

### C1 — N ≥ 4 concurrent reads per read master

**Rationale.** Miss concurrency is the largest single unexploited factor in
the machine. Measured at current HEAD with `tb-l2c-chain`
`concurrent_fill_overlap`: **8 concurrent fills complete in 414 cycles; the
same 8 sequentially take 2019 — 4.88x.** The MIG queue peaked at 8 and 8 ARs
were issued before the first R returned, i.e. the concurrency is real all the
way down, not an artefact of the model.

> This 4.88x supersedes the 4.96x / "13.8 vs 68.3 cyc/miss" pair quoted in
> `soc_bus_review.md:76-83` and repeated in the 2026-09-02 review. Same
> phenomenon, re-measured on the post-`5213100`/`00f079c`/`980342c` L2C.

**Status.** I-side met: `IcachePlugin` runs 5 MSHRs (1 demand + 4
speculative), each with its own AXI ID. D-side **open**: `N_MSHR == 1`, with
reserved ID space for 4 already in `AxiIds.dRefill`. The DMA engine (not yet
merged) is met at 8.

**What is needed.** Flip `N_MSHR` to 4 and build slice D2's enablers (V2a.2
R/B routing by ID, V2a.3 response tagging). Per §4, do this **in the same
campaign as C3**, not after it — deferring it "until the fabric is verified"
just re-creates the standoff one turn later.

### C2 — unique ARID per concurrent read

**Rationale.** The cost of getting this wrong is not gradual. `docs/l2c_perf.md`
records the cliff directly: 8 outstanding misses with unique IDs = 8.38 cyc/op;
the same 8 under one ID = **55.40 cyc/op, a 6.6x cliff**, and no test fails.
`a28d5f7` narrowed the accept gate but did not retire the discipline — C2 is
a permanent requirement on masters, not a workaround for a fabric limitation.

**Status.** Met on both `cpu040` masters and on `dma_engine.sv`. Any new read
master (table-walk fold V2c, a future NVMe engine) is bound by this clause
from the moment it is written.

### C3 — crossbar, ≥ 4 outstanding reads per master port

**Rationale.** This is the binding gate on the D side. `rs_state[mi]` is a
scalar FSM per master port; every `req_rd_*` term is qualified by
`(rs_state[mi] == RS_IDLE)`, so a master gets exactly one AR in flight
regardless of how many it presents.

**What is needed.** Replace the scalar with a small per-master read-tracking
table (4 entries) keyed on the master's ARID. **No ID widening is required**
(C12): R beats already route on the slot bits of `XID_WIDTH`.

**Sequencing note.** `soc_bus_review.md` §8 recommends a separable
prerequisite: register the R-return stage (today a combinational 4×6×128b mux,
`axi_xbar.v` ~:3446-3466). It costs one cycle of read latency and *removes* a
long combinational path from a congested region — measure timing with that
alone before C3 lands. Area estimate for C3 itself: **+3,000 LUT / +1,500 FF**
(estimate, see §6).

### C4 — L2C doors accept while a burst decomposes — **MET**

**Rationale.** With the lookup pipelined (`5213100`) the door's old two-cycle
slack against a three-cycle loop went to zero: at one beat per cycle the next
header can only be latched the cycle *after* the previous burst's last beat.
One-beat bursts measured 2.00 cyc/op with 255 REFUSED-EARLIER bubbles over 256
ops before the pre-latch existed.

**Status: MET on both doors.** The LSU channels (`ar_pend_v`/`aw_pend_v`)
carried it already. The fetch door got the identical treatment this session in
**`51d2b0e`**. Measured A/B on that change: single-ID 1-beat back-to-back fetch
bursts **2.992 → 2.004 cyc/burst (1.49x)**; rotating-ID shapes essentially
unchanged, which is the expected shape — the pre-latch buys back the accept
slot that a single-ID stream was wasting.

**Consequence, and it is the point of writing this down:** *the L2C door is no
longer a D-side gate.* The remaining D-side chain is exactly two links — C1
(core `N_MSHR == 1`) and C3 (crossbar `rs_state`). Any document, comment or
plan still listing the L2C front door as a D-side limiter is stale as of
`51d2b0e`.

### C5 — same-ID gating scoped to the real hazard

`a28d5f7` closed a fetch/LSU ID-namespace starvation (the two ports share an
ID space at the door; `pipe_id_haz_c` now qualifies on `q_is_fetch`), and the
2026-08-19/20 pipeline rework moved cache-path response ordering off the door
to S2 (`ord_now_block_c` / `ord_merge_block_c`), so `id_busy_c` applies at the
door only to bypass-bound beats. What remains is C6.

### C6 — fetch line fills at the quadrant floor — **OPEN, and it is not the door**

**The measurement.** `cpu040` issues instruction fills as `len=1 / size=5` —
two 256-bit beats, one 64 B line — on rotating MSHR IDs
(`IcachePlugin.scala:2313-2318`). The L2C serves each 256-bit beat as **two
internal 128-bit quadrant lookups**, so the floor for a 2-beat burst is 4
lookups = **4.0 cyc/burst**. Measured: **4.97 cyc/burst.** The ~1 cycle of
excess is the same-ID quadrant *pair* being held apart by `pipe_id_haz_c`'s
two pipeline stages — the two halves of one 256-bit beat necessarily share an
ID, and the hazard check does not know they are two halves of one thing.

**Why this clause exists separately from C4.** Because the obvious diagnosis
("the fetch door is 1-deep") is now wrong, and re-deriving it wastes a session.
The door is fixed. This is a datapath-width / hazard-scoping issue behind it.

**What is needed** (either, not both; needs a design pass, not a bolt-on):
widen the L2C's fetch datapath so a 256-bit beat is one lookup, or scope
`pipe_id_haz_c` so the two quadrants of a single accepted beat are not treated
as an ID collision with each other.

### C7 — the DDR path below — **MET**

`axi_ddr4_mig_bridge.v:15-33`: 8 outstanding reads (`m_arid = 0` throughout,
relying on the plain AXI4 same-ID in-order rule rather than any MIG-specific
scheduling assumption), 4 outstanding writes, cut-through on both directions.
Built and paid for; nothing above it currently feeds it. This is the floor the
rest of the contract is aiming at.

### C8 — VRAM lane mux — MET

`axi_vram_priority_mux3.v:196` `MAX_BULK_AHEAD = 4`, and `:203`
`MAX_BULK_AHEAD_QUIET = 8` with `bulk_cap_c` (`:353`) selecting between them on
`scan_quiet_c` — i.e. the cap is not only raised but adaptive to scanout
activity, which is the "better still" the original F3 item asked for. Scanout
keeps its unconditional AR priority and its own `MAX_SCAN_AHEAD` quota.

**Do not re-open this as a target.** `soc_bus_review.md` §8's item 2d and
`fabric_concurrency_and_soc_review_2026-09-02.md` both still describe this as
`MAX_BULK_AHEAD = 2`; that is stale (the raise is listed as done in the latter's
own §5 "already done" list, which the §1 series diagram then contradicts).
Verified against the RTL 2026-09-03. It is met at 4/8 and is no longer a link in
the read series.

### C11 — doors buy depth with a pre-latch, not a wider `ready`

Both C4 pre-latches were built this way deliberately, and the reasoning is
worth promoting to a clause because the tempting shortcut is a one-line edit:
making `s_arready` read `!ar_have || <burst ends this cycle>` pulls
`do_accept_c` — and with it the whole hazard/selection cone — into the AXI
ready path, which is the one output the upstream master's own timing closure
depends on. `s_arready`, `f_arready` and any door added later stay plain
register bits. Cost of doing it right: ~53 FF per channel.

---

### C13 — table walks must go through the D-cache

**This is not a style preference; it is a confirmed lost-update bug, and it is
the reason the TLBs can be shallow.**

`DtlbPlugin`/`ItlbPlugin` each own a private 128-bit `Axi4` master
(`dtlbAxi`/`itlbAxi`). They are merged with the D-cache's port by `AxiDMerge` —
but that is *bus arbitration only*. The walker never consults the L1D array,
and `DcachePlugin` has no snoop port and no external-write invalidate path, so
it is never told the walker wrote.

Two failures, both real:

- **Write side (confirmed, with a failing test).** The U/M-bit descriptor
  update is a raw `AW`+`W`+`B` on the walker's own port. If the descriptor's
  16-byte line is resident **COPYBACK-dirty**, the eventual eviction writes the
  *older* cached image back over the walker's update — **a lost update**, not
  mere staleness. That is the observed shape of
  `mmu_atc_write_hit_sets_modified`, which still fails with the M-bit logic
  itself already correct.
- **Read side (undocumented).** Descriptor reads issue on the same private
  port, so a PTE the supervisor has just written — sitting dirty in L1D — is
  invisible to the walk, which reads the stale copy from memory.

**Why it matters beyond correctness — the shallow-TLB premise.** TLB depth was
chosen on the understanding that a walk is cheap. A walk is three *dependent*
descriptor reads; that is only cheap if they hit in L1D. Served on a private
port they are three full memory round trips, every time, which makes a shallow
TLB the wrong trade rather than a good one. Closing C13 is what makes the
existing TLB geometry the right choice — a prerequisite for the depth decision,
not an optimisation layered on top of it.

**Status: fully designed, entirely unbuilt.** The spec
(`cpu040/docs/superpowers/specs/2026-08-18-walker-dcache-passthrough-design.md`,
30 DECIDED items W1-W30, signed off after 7 review rounds) and its 19-task
implementation plan both exist and have never been executed — no commit touches
a walker→`DcacheService` path. Another instance of §4's pattern: capability
designed, reviewed exhaustively, then not landed. Scope is a **complete kill**:
the walkers lose their `Axi4` ports entirely and become ordinary
`DcacheService` clients (`loadCmd`/`loadRsp`, `store`/`storeAck`).
`AxiIds.WALK_READ`/`WALK_WRITE` stay defined but unreferenced (W20).

Note the plan's own warning: it names two unconditional `ready` leaks in
`LsEuPlugin` that are benign today and become **silent-corruption** bugs once
the walkers share that port — and a later review round found the same shape at
**nine** sites, not the two originally fixed. Those must be closed as part of
this work, not after it.

---

## 3. C9 — the write side is met and closed

**This is the single most important anti-staleness statement in this document.**

The write-side streaming case is **solved**. It was won at the L2C, not by
adding transaction concurrency:

- **`00f079c`** — a write that fully covers a line allocates without fetching
  it first. The read-for-ownership disappears.
- **`980342c`** — sectored valid/dirty (4 bits per way, one per 128-bit
  quadrant). A line touched in one quadrant costs one 16 B writeback beat, not
  four.

**Measurement:** `tb-l2c-wstream` at `AWLEN=255` measures **1.020 cyc/word
single-outstanding** against **1.004 cyc/word pipelined**. The entire remaining
benefit available from write pipelining on that shape is **1.02x.**

Therefore:

1. **Write concurrency is NOT an open target of this contract.** Do not add
   write-side outstanding depth to the crossbar to chase throughput. There is
   no throughput there to chase.
2. **The `boot_fsm` multi-outstanding-writes item is withdrawn.** The claim it
   rested on — "22 cycles per write, ~3.7 s of a ~4.06 s cold boot, purely
   serialization-bound" (`soc_bus_review.md:1043-1062`, repeated in the
   2026-09-02 review §1 and its "first domino" recommendation) — was measured
   before `00f079c`/`980342c` and is **~20x stale**. It is retracted here so
   nobody re-derives it from those documents. It was proposed as the
   zero-contention proving ground; it is now the wrong first move, because it
   is a large change against a ~1.02x ceiling.
3. **`axi_xbar.v`'s write-side objection stands** — see §5.

**Honest scope of the measurement.** `tb-l2c-wstream` measures long bursts
(`AWLEN=255`). A master that emits many short single-beat writes still pays
per-transaction overhead at every hop. That is a **master-side burst-shaping**
question, not a fabric-concurrency one, and this contract does not cover it.
The cold-boot wall-clock figure has **not** been re-measured on hardware since
`00f079c`/`980342c` — the old number is retracted, not replaced (§6).

---

## 4. The in-series read path — "intermediate landings measure zero, by policy"

**Adopted as the plan of record**, not as a footnote. This is
`soc_bus_review.md:826-842`'s own prescription, ratified here.

The read-path limits are **in series**. Fixing any one alone moves the system
by approximately nothing, because the next one downstream binds immediately.
The precise failure mode being guarded against: someone lands one item,
measures correctly, observes no change, and concludes "that wasn't the
bottleneck" — a false conclusion from a correct measurement, and one that
tends not to get revisited.

The series, as it stands at v1.0 of this contract:

```
  C1 core N_MSHR   →   C3 xbar rs_state   →   [C4 L2C door: MET]   →
      (D-side: 1)          (per master: 1)         (51d2b0e)

  →   C6 quadrant pair   →   [C8 VRAM mux: MET]   →   C7 MIG
      (4.97 vs 4.0)              (4 / 8 adaptive)        (8 reads — MET)
```

**Items declared in series — a zero result on any one of these alone is
expected and is NOT evidence the item failed:**

- **C1** (core D-side `N_MSHR`)
- **C3** (crossbar per-master read table)
- **C6** (L2C quadrant/hazard scoping, I-side)

**Items NOT in series — these are independently measurable and a zero result
on one of them IS meaningful:**

- **C4** (both doors) — already landed and independently measured (1.49x on
  the fetch door's worst shape).
- **C9's** L2C write work — already landed and independently measured.
- Instrumentation (`soc_bus_review.md` Phase 0a/0b — L2 counters made
  AXI-readable, DDR round-trip latency counter). ~300 LUT, no timing risk, and
  still not landed; see §6.

**Recommended campaign shape.** Land C3 (with its R-return registration
prerequisite measured separately, for timing) and C1 together, then measure.
C6 is a separate design pass and can follow, since it is I-side only. C8 needs
nothing — it is already met at 4/8 adaptive.

---

## 5. The carve-out: capability is unilateral, policy is not

The "build to the contract, don't wait for the neighbour" rule in §0 applies
to **capability** — depth, ID space, width. It does **not** apply to **policy
that needs a single owner**. Two standing examples, both of which this
contract explicitly preserves rather than bulldozes:

### 5a. The crossbar's write-side objection is correct and stays

`axi_xbar.v:1629-1654` records a 2026-07-21 investigation that analysed
write-side AW pipelining and **deliberately did not implement it**. Both of
its reasons hold:

- Cross-master overlap on one slave (unlocking at `wlast` instead of
  B-consumed) needs B routing switched from `sw_owner`-based to bid-slot-based,
  like the read side's `s?_rtgt`. Mechanical, but every live slave is itself
  single-outstanding on writes, so it would present AW2 one handshake earlier —
  marginal.
- Same-master pipelining — the case that would actually help a store stream —
  needs the per-master single-transaction FSM to become a queue with W-burst
  boundary tracking, and if the two AWs decode to *different* slaves, B
  responses can complete out of order. That violates AXI same-ID B ordering for
  a master reusing one AWID, which the LSU does (`AxiIds.D_STORE`), unless the
  crossbar reorders B. That is a substantial, risk-bearing redesign.

**Contract position:** C10 codifies the in-order-B-per-ID guarantee the current
`sw_owned` scheme provides, and C9 removes the throughput motive for breaking
it. `sw_owned` stays. Revisit only with a dedicated design pass plus
multi-outstanding BFM coverage — never as a bolt-on to the current owner
scheme. **This contract does not authorise anyone to "implement to C3" on the
write side.**

### 5b. The DMA/L1 coherency question is a policy decision, not a capability gap

`docs/fabric_concurrency_and_soc_review_2026-09-02.md` §3b: L2-as-point-of-
coherency does nothing for L1, and neither core has an L1 snoop port. The
resolution (driver-managed cache ops confirmed by disassembly / ROM patch /
non-cacheable MMU mapping) needs **one owner and real evidence**, not two
parallel guesses built unilaterally. Out of scope for this contract; named
here so nobody reads §0 as licence to race it.

---

## 6. What is NOT measured

Stated plainly, because half this document's value is not creating a new class
of confident-but-stale claims:

- **No full-SoC IPC measurement exists** for any of this. The Part 88 register
  slice showed byte-identical retired/cycles in microbenchmarks, which
  validates that the added AXI latency is largely hideable, but a real full-SoC
  IPC run (Part 72/73 methodology, tens of wall-clock hours) is outstanding.
  Nothing in this contract has been shown to move end-to-end machine
  performance yet.
- **All LUT costs are estimates.** C3's +3,000 LUT / +1,500 FF, C6's
  unknown — derived from the existing utilization report in
  `soc_bus_review.md` §8, not from a Vivado run for this contract. The device
  was at **187,011 / 216,960 = 86%** with congestion level 6 at the last full
  route, and `u_l2c/g_active.u_ctrl` shares congestion windows with the CPU's
  ROB and commit logic. Area here is not unconditionally affordable.
- **All concurrency measurements above are Verilator**, on `tb-l2c-chain` /
  `tb-l2c-wstream`. None of C4, C9 or the 4.88x figure has been confirmed on
  hardware.
- **The cold-boot wall-clock figure is retracted, not replaced.** §3 withdraws
  the 3.7 s / 22-cyc-per-write claim; no post-`00f079c` hardware re-measurement
  has been taken.
- **The real DDR round-trip latency on hardware has never been measured.**
  `soc_bus_review.md` Phase 0b's counter has not landed, and Phase 0a (making
  the existing L2 hit/miss/occupancy counters AXI-readable instead of
  ILA-probe-only, ~300 LUT, no timing risk) has not landed either. Every DDR
  latency figure in this repo remains an assumption.
- **Nothing enforces this contract mechanically.** There is no assertion, no
  lint rule and no testbench gate that fails when a clause is violated. The
  cheapest first step is the one `soc_bus_review.md` §9/S1 already proposes: a
  testbench that drives N concurrent misses under one ID and asserts
  `mshr_occupancy` never exceeds 1 — that would have caught the C2 class
  permanently. Until such a gate exists, this document is a discipline, not a
  guarantee.

---

## 7. How to cite this contract in a comment

Retiring the stale-comment class is half the point of this document. A
contract-citing comment does not silently become false when a neighbour
changes; a neighbour-citing comment does.

**Do this:**

```verilog
// Depth: 4 outstanding reads, per docs/fabric_concurrency_contract.md C3
// (>=4 per crossbar master) with C2's unique-ARID discipline assumed of
// the master above.  Sized to the CONTRACT, not to what the L2C door or
// the core's N_MSHR happen to accept today.  C11: `arready` stays a plain
// register bit -- the accept cone must not reach the AXI ready path.
```

```scala
/** D-side refill MSHR count.  Contract C1 (N >= 4 concurrent reads per read
  * master), macqd700-soc/docs/fabric_concurrency_contract.md.  The reserved
  * ID range in `dRefill` is sized to C1 and C2 regardless of what the
  * crossbar admits at any given moment. */
val N_MSHR = 4
```

**Not this:**

```verilog
// One outstanding transaction at a time.  The core's if_stage already
// self-serialises its fetches, so there is no queue here.
```

That comment (`if_to_axi.v:16-18`, and its twin at
`axi_narrow_to_wide.v:72-76`) was accurate the day it was written. It is false
now, it fails silently, and it reads as "this is fine" to the next person
debugging throughput. Both modules are on the cleanup list for exactly this
reason.

**Three rules:**

1. **Cite the clause number and this file's path.** Not a neighbour's
   `file:line`, and never a neighbour's current depth as a justification.
2. **A neighbour's present-day behaviour may be recorded as a dated
   measurement, never as a justification.** "Measured 4.97 cyc/burst against a
   4.0 floor, 2026-09-03" is durable and self-dating. "The L2C only takes one
   at a time" is a landmine.
3. **When a clause is OPEN upstream of you, say so and build to it anyway.**
   "Implements C3; currently throttled by C1 on the D side" is the correct
   form. "Not worth doing until the core is multi-outstanding" is the standoff
   this document exists to end.

---

## 8. Change control

- **Amendment** = a new clause number, or a status change on an existing one
  with the commit or measurement that justifies it. Both require an entry in
  §9 and a version bump.
- **Clause numbers are permanent.** A clause that no longer applies is marked
  RETIRED in place with its reason and date. Numbers are never reused.
- **Status vocabulary:** MET (with the commit that met it), PARTIAL (with what
  remains), OPEN (with what is needed), MET — CLOSED (met, and explicitly not
  a target for further work), RETIRED.
- **Measurements cited here must name the testbench and the scenario.** A bare
  ratio with no reproduction path is not admissible evidence for a status
  change — that is how the 22-cyc-per-write claim outlived its truth by ~20x.

## 9. Revision log

| Version | Date | Change |
|---|---|---|
| 1.0 | 2026-09-03 | Initial ratification. C4 recorded MET on both doors (`51d2b0e` closed the fetch door). C9 recorded MET — CLOSED, and the `boot_fsm` / 22-cyc-per-write / 3.7 s claim explicitly retracted as ~20x stale. C6 opened as a distinct, non-door I-side limiter. §4's "intermediate landings measure zero, by policy" adopted as plan of record. §5a preserves `axi_xbar.v`'s write-side objection unchanged. |
