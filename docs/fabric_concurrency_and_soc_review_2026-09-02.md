# Fabric concurrency & SoC architecture review (2026-09-02)

**Origin**: a critical, wide-lens review of the SoC's design given the CPU swap
from a simple in-order core to `cpu040` (a genuinely strong OoO 68040-compatible
core, ~197MHz demonstrated FMax in isolation, 8-16x real measured advantage
over the actual 25MHz Quadra 700 CPU it replaces — see
`docs/BUG_calibration_word_misplaced_0d00.md` Part 80). Conducted by a Fable
review agent, three passes (initial review, DMA/Ethernet-merge addendum,
mutual-blockers addendum), each with direct RTL evidence (file/line), not
speculation. This doc consolidates all three into one actionable reference so
a future session can pick this up without re-deriving it.

**Headline**: the "the SoC still feels shaped for the old core" instinct is
real but was mislocated as generic rot. Roughly half the migration already
happened and happened well (see "Already done, keep" below). What's left is
one specific, well-diagnosed, cross-repo standoff — not a wholesale
architecture problem, not a case for a rewrite.

---

## 1. The single highest-leverage item: fabric/core memory-level-parallelism is stuck at 1

**The problem, in one sentence**: every memory miss in the machine today —
CPU I-side, CPU D-side, and (once merged) DMA — is a fully serialized round
trip, even though every layer beneath the serialization point (L2C, the MIG
bridge) is already built multi-outstanding and sitting unused.

**Evidence chain** (all read directly, not inferred):
- Core D-side: `cache/AxiIds.scala:21-30,39` — `N_MSHR == 1` on purpose, with
  an explicit comment that multi-outstanding D-loads (slice D2) "deliver
  nothing end-to-end until [the crossbar] is reworked." Reserved ID space for
  4 D-side MSHRs already exists, unused — see §4 below, this matters.
- Core I-side: `IcachePlugin.scala:313` — 5 MSHRs (1 demand + 4 prefetch,
  distinct AXI IDs). Structurally capable of 5 concurrent fills.
- Crossbar: `rtl/soc/axi_xbar.v:3208-3235` — one outstanding read per master
  port (`rs_state[mi] == RS_IDLE` gates every new AR). Writes similarly via
  `sw_owned` (`:1629-1654`), one outstanding write per master.
- L2C front door: `rtl/soc/l2c_ctrl.v:341-352` — `s_arready = !ar_pend_v`
  (LSU: one active burst + one pre-latched header) and
  `f_arready = !fetch_ar_have` (fetch: **explicitly single-burst, no
  pre-latch** — the code's own comment says the pre-latch is "real, valuable
  follow-on work... tracked separately"). Behind this door: 8 MSHRs
  (`l2c_mshr.v`), unused past the 1-deep gate.
- MIG bridge: `rtl/board/axi_ddr4_mig_bridge.v:15-33` — 8 outstanding reads,
  4 writes. Also unused past the gate above it.

**Net**: the fabric has 8-way concurrency built and paid for at multiple
layers, and a single scalar flag per master port throttles all of it to 1.

**Measured cost** (not estimated):
- Miss concurrency worth **4.96x** (8 concurrent fills: 13.8 cyc/miss vs 68.3
  cyc/miss sequential) — `docs/soc_bus_review.md:76-83`.
- The L2C door's same-ID-in-flight cliff is a further **6.6x** —
  `docs/l2c_perf.md:40-50`.
- L2C's 8-way MSHR structure costs **20,953 LUT — 11.2% of the entire
  device** — bought and unfed (`docs/soc_bus_review.md:140-152`).
- Cold-boot RAM-zero pass measured at **22 cycles/write**, purely
  serialization-bound, ~3.7s of a ~4.06s cold boot
  (`docs/soc_bus_review.md:1043-1062`).
- With one DMA/disk stream sharing the L2C door with the CPU, CPU hit cost
  went **3.00 -> 23.21 cycles/op (7.7x)** at DDR-40 —
  `docs/l2c_perf.md:29-35`. This is what a NIC under real traffic will
  recreate permanently once merged (see §3).

**Why this existed**: the old, simple in-order core self-serialized anyway,
so single-outstanding fabric cost nothing. Two adapter modules encode this
assumption explicitly as their own design justification:
`rtl/soc/if_to_axi.v:16-18` and `rtl/soc/axi_narrow_to_wide.v:72-76` ("the
core self-serialises," per `docs/soc_bus_review.md:103-107`). That
justification is now false and, per `soc_bus_review.md` its own §S9, was
predicted in advance to become "misread as 'the new core is slow' rather
than a fabric limitation" the day the core changed. That prediction was
correct.

**The recommended fix** (already designed and costed, not a new proposal):
land `docs/soc_bus_review.md` §8 Phase 2 as **one measured unit**:
- **2a**: crossbar per-master N-outstanding read table (the R channel
  already demuxes on ID at `axi_xbar.v:3408`, no ID widening needed).
- ~~**2b/2c**: L2C front-door work + the fetch-port pre-latch
  (`l2c_ctrl.v:341-348`'s already-deferred item).~~ **The fetch pre-latch
  is DONE** (`51d2b0e`); the LSU door already had one. Measured A/B:
  single-ID back-to-back 1-beat fetch bursts 2.992 → 2.004 cyc/burst
  (1.49x); the rotating-ID shape cpu040 actually drives is unchanged,
  because its limiter is not the door but l2c serving each 256-bit fetch
  beat as two 128-bit quadrant lookups (contract clause C6).
- Then flip the core's `N_MSHR` to enable D-side slice D2 and let the
  I-side's existing 5-deep prefetcher actually prefetch.

**Critical sequencing note — the in-series trap**: `soc_bus_review.md:836-842`
already documents this correctly: F1 (crossbar), F2 (L2C door), F3 (fetch
pre-latch) each measure **zero benefit alone** — landing only one and
concluding "that wasn't the bottleneck" is the exact failure mode that has
likely stalled this before. **Adopt "intermediate landings measure zero, by
policy" as the plan of record** before starting, so whoever lands the first
piece doesn't mistakenly conclude it failed.

~~**Proving ground with zero contention risk**: `boot_fsm` multi-outstanding
writes (2a-boot in `soc_bus_review.md`) ... cold-boot RAM-zero from ~3.7s
down to ~1.0-1.5s ... do this first to bank a win.~~

> **CORRECTION 2026-09-03 — DO NOT START HERE. This item is already won.**
> The "22 cycles/write, ~3.7s of a ~4.06s cold boot" figure above predates
> L2C commits `00f079c` (allocate full-line writes without fetching) and
> `980342c` (sectored valid/dirty), which together took the write-miss path
> off read-allocate. Re-measured at current HEAD with `make tb-l2c-wstream`,
> through the real chain, at `boot_fsm`'s shipping `ZERO_AWLEN=255`:
>
> | | cycles/word | 256 MiB @100MHz |
> |---|---|---|
> | single-outstanding (today) | **1.020** | 0.684 s |
> | pipelined multi-outstanding | **1.004** | 0.674 s |
>
> **1.02x remains** — the write stream is at its 1 word/cycle bandwidth
> floor. The predicted 3.7s→1.0-1.5s win was already banked; only the
> hardware number was never re-measured. Landing 2a-boot would deliver ~2%
> and would burn the campaign's credibility on precisely the
> "intermediate landings measure zero" trap this section warns about.
>
> **The read side is where the prize actually is, and it is still real:**
> `make tb-l2c-chain`'s `concurrent_fill_overlap` at current HEAD measures
> 8 concurrent misses = 414 cyc vs 8 sequential = 2019 cyc — **4.88x**,
> with MIG queue peak 8 and 8 ARs accepted before the first R. See
> `docs/fabric_concurrency_contract.md` for the ratified target, which
> records the write side as MET AND CLOSED so this cannot recur.

---

## 2. The clock-domain question — decide it explicitly, sequenced after §1

Today CPU + crossbar + L2C share one `core_clk`, no CDC
(`fpga_top_xbar.vh:274`, `fpga_top_ddr.vh:245`). Deployed at 100MHz,
**+0.044ns WNS at last full route — zero margin already, at half the core's
demonstrated speed.**

Real Vivado timing analysis this session (`docs/BUG_calibration_word_misplaced_0d00.md`
Part 86) proved whole-SoC-at-200MHz is **not achievable with current RTL**:
synth-only 200MHz target came back at **-3.619ns WNS**, 8,163 failing
endpoints. Worst path was CPU-internal (task #127's ROB fault-capture cone)
spilling straight from the fabric with zero register boundary.

**Part 88 fix, already landed** (core repo commit `22b16f4`, SoC repo commit
`5c852ad`): registered the CPU's `axi_i`/`axi_d` AXI ports (SpinalHDL
`Axi4.pipelined(StreamPipe.FULL)`, all 7 channels, +107 FF). Real re-synth
result: **WNS -3.619ns -> -1.739ns**, and the *specific* fabric-into-CPU
violation is structurally gone (confirmed in both whole-design and per-block
isolation queries — neither xbar's nor L2C's worst path touches the CPU
anymore). IPC cost unmeasurable in microbenchmarks (byte-identical
retired/cycles across configs with/without register slice) — validates that
the added AXI latency is largely hideable, though a real full-SoC IPC
measurement (tens of wall-clock hours, Part 72/73 methodology) is still
outstanding.

**Remaining new bottleneck after the Part 88 fix**: purely CPU-internal —
`RobPlugin_logic_head_reg -> IssueQueuePlugin` via `LsEuPlugin`'s
store-queue forward-stall cone. Distinct from task #127. Belongs to the
core's own FMax closure work, not the SoC.

**Sizing note**: with the -1.739ns WNS against a 5ns/200MHz target, a
**single-domain constraint sweep around ~140-150MHz may already close** —
worth measuring before anything else clock-related, since it could be a
40-50% whole-system speedup for a constraint sweep + closure work alone, no
CDC, no new board hardware.

**Sequencing verdict**: do §1 (fabric concurrency) before any clock-domain
split. A CDC split adds real per-transaction latency (destination-clock
edges per burst); paying that on top of a fabric that still serializes every
miss to MLP=1 makes the machine slower per-miss to run a faster core that's
already stalled. After §1 lands, the split becomes genuinely attractive.

**Hardware note**: real board `core_clk` is hard-constrained at exactly
100MHz in `synth/fpga_top_real_mig.xdc`; `CORE_CLK_DIVIDE` only divides
*down*, never up. A real >100MHz core clock needs new clock-gen (an MMCM/PLL
stage) — a board-level hardware change, separate track, can be commissioned
in parallel with the RTL work above since it's slow and independent.

**Also flagged**: the Part 88 fix's core-repo commit was deliberately not
yet bumped into the SoC's `cpu040` submodule pointer. That's a pending,
explicit decision, not an oversight to just default on.

---

## 3. The incoming DMA engine + SONIC Ethernet NIC merge (from the v1 repo)

**Source**: `/home/qwertyoruiop/macqd700-soc` (the old, in-order-core SoC
repo), `main` branch. Files: `rtl/soc/dma_engine.sv`, `rtl/soc/dma_ctrl.v`,
`rtl/mac/q700_eth_sonic.v`, `rtl/mac/q700_sonic_{rx,tx,cdc}.sv`,
`rtl/mac/q700_sonic_rx_cdc.sv`, `rtl/board/q700_eth_link.sv`,
`rtl/soc/sonic_trace_ring.v`, `rtl/soc/eth_debug_regs.sv`,
`rtl/soc/fpga_top_ethernet.vh`, `rtl/soc/fpga_top_dma.vh`,
`docs/dma_engine_design.md` (in the v1 repo). To be merged into the cpu040
SoC repo (`macqd700-soc-worktrees/m68k040ooo-integration`) at a later date.

### 3a. §1's fix is a hard prerequisite for this merge, not a nice-to-have

`axi_xbar.v` is **byte-identical between the v1 repo and the cpu040
integration repo** (md5-compared) — nothing about v1's fabric handled a
DMA-class master differently. The DMA engine is well-built: 8 unique AXI IDs
(`dma_engine.sv:75,101-102`, `id_live`/`free_id`, never reused while live),
16-deep read/write queues, W-channel look-ahead. Against today's fabric it
gets capped to **1 read + 1 write in flight** — the same `rs_state`/
`sw_owned` gate from §1. ~90% of the engine's own concurrency machinery
would be dead on arrival.

**Not merely slower — already caused a real bug on v1**: commit `76766ef`
root-caused actual RX data loss: `q700_sonic_rx`'s BRAM pipeline free-ran
during a DMA stall and skipped a chunk (memorialized in
`fpga_top_dma.vh:249-256`; the preceding workaround, `dc29f38`, had raised
`QUEUE_DEPTH` to 32 to paper over a related 1088-byte RX ceiling). Stall-
window bugs breed in a fabric that stalls constantly — landing the NIC
before §1 means shipping into the exact regime that already produced this.

**Mitigating factor**: the M3 seat is already free in the integration repo
(freed by RAM-disk removal), and the engine's unique-ID discipline puts it
on the correct side of L2C's 6.6x same-ID cliff — the moment the crossbar
can pass more than one AR, the engine needs zero changes to benefit. The
merge is well-prepared on both ends; only the fabric middle is missing.

### 3b. Real, unresolved L1 coherency gap — needs a decided owner before merge, not discovery after

The design intent is correct and matches the already-correct
I-fetch-through-L2C precedent: `docs/dma_engine_design.md` §0 states
anything CPU-observable "always goes through the main AXI bus and through
L2C... with L2 as the point of coherency"; non-allocating hints are
explicitly "an allocation policy, not a coherency bypass." Raw-DDR mode is
confined to CPU-invisible regions.

**But L2-as-coherency-point does nothing for L1.** `dma_engine_design.md`
§2.1: L2C has "no snoop, no probe port." Neither core has L1 snoop hardware
(v1's only mechanism is internal SMC D-write -> I-cache invalidate;
cpu040's L1D is copyback with CPUSH/CINV as the only push-out path). Two
concrete hazard directions on the SONIC descriptor rings
(`q700_sonic_rx.sv` reads RRA/RDA, writes CRDA/status; TX reads TDA):

1. DMA writes a descriptor in L2C/DDR while the CPU has stale data cached in
   L1D from its last poll -> CPU reads stale data indefinitely.
2. CPU builds a TX descriptor, it sits dirty in copyback L1D -> the engine
   reads pre-write bytes through L2C.

This is structurally the same bug class as the historical, hardware-
confirmed I-cache/D-cache coherency bug
(`docs/BUG_icache_dcache_coherency.md`), with the DMA engine playing the
I-cache's role. On cpu040 the exposure is strictly worse than v1 (bigger
speculative window, copyback L1D vs. v1's simpler scheme).

**Whether this actually bites depends on the real Mac driver.** Updated with
a real, load-bearing data point from the project owner (2026-09-02):
**Ethernet already works correctly on the v1 (in-order) core**, with DMA and
caches both live. That's strong field evidence the driver already performs
correct cache management around the descriptor rings at the software level
— if it were silently relying on real 68040 bus-snooping (which this SoC
does not implement, on either core), it would very likely have already
surfaced as RX/TX corruption on v1. This substantially *de-risks* this item
relative to the original "unverified" framing below, though it doesn't
fully retire the need to confirm the exact mechanism cpu040-side (bigger
speculative window, copyback L1D vs. v1's simpler cache — see above).

**Required action before merge, now scoped down given the above**: confirm
(don't re-derive from scratch) the driver's actual cache-management
instructions around the descriptor rings — likely a quick disassembly
check now that we know what to look for (CPUSH/CINV calls bracketing RRA/
RDA/TDA access, or a non-cacheable mapping already in place), rather than
an open-ended "is there a bug here" investigation. If confirmed
driver-managed: cpu040 should be fine as-is, verify the driver's flush
granularity actually covers cpu040's (larger) speculative/dirty window
before declaring done. If NOT confirmed (e.g. v1 got lucky on timing rather
than doing real cache management): decide between
- (a) driver-side ROM patch to CPUSH the rings (fits the established
  ROM-patch treadmill from the boot-debug campaign), or
- (b) map the descriptor/buffer region non-cacheable via the MMU.
- **Not**: a new L1 snoop port — disproportionate rework for one peripheral.

This decision needs a single owner and real evidence, not a race to
implement two guesses (see §4's caution about policy vs. capability).

### 3c. `dma_ctrl` stub — correction

v1 already **deleted** the 2,024-LUT `dma_ctrl` instance (2026-08-01,
`fpga_top_dma.vh:17-25`), replacing it with a null-slave/`vhdd_ctrl`
terminator; the file itself is retained deliberately for a future
NVMe-as-SCSI engine. **If the cpu040 integration repo still carries the live
stub instance, reconcile at merge time by taking v1's terminator
arrangement — don't re-inherit the dead instance.** Also: the merge brings
back a real consumer for the `dma_irq_w` level-6 line currently tied low.

### 3d. SONIC real-time behavior — lower risk than the VIA/T2 class, two watch items

Network traffic is genuinely asynchronous — it doesn't care how fast the
CPU is — so the calibration-loop race shape mostly doesn't apply here. Two
residual, unverified (no driver source available) watch items for
post-merge if Ethernet misbehaves:
1. Driver code busy-waiting on CR command-completion bits with a fixed
   iteration cap would pass fine (RTL acks instantly) — but code expecting
   a *minimum* latency between command and completion is the same
   saturation-class problem as the boot-debug campaign's Part 7.
2. The RX descriptor-ring lazy-CRDA-reload path
   (`q700_sonic_rx.sv:200-203`, deliberately matches MAME's own lazy-reload
   behavior) is exactly the kind of ordering-dependent timing a much-faster
   consumer can race — same meta-pattern as the rest of this campaign.

Neither justifies pre-emptive work; both belong on the "check first if
Ethernet misbehaves" list post-merge.

---

## 4. The mutual-blocker pattern — and the proven way out

Across this review, the same shape recurs: two components each defer
improving their own side because the other side "isn't ready to benefit,"
and the standoff is self-perpetuating. Four confirmed instances:

1. **Core MLP <-> fabric MLP** (the canonical one, §1). Both directions are
   in writing: `AxiIds.scala:21-26` defers core D-side concurrency because
   the fabric can't take it; `docs/l2c_perf.md:61-66` records an *earlier
   draft actually recommending deleting L2C's own miss concurrency* because
   it looked unreachable by the old core — at the same time the new core
   was declining to grow its own for the mirror-image reason. The two old
   adapter modules (`if_to_axi.v`, `axi_narrow_to_wide.v`) encode the
   standoff directly as a design comment about the *other side's* behavior.
2. **Fetch door <-> prefetcher.** The pre-latch fix
   (`l2c_ctrl.v:341-348`) is deferred pending fetch traffic demonstrating
   need; the core's 5-MSHR prefetcher can't demonstrate anything through a
   1-deep door. Generalizes to the in-series trap already described in §1.
3. **CDC split <-> fabric concurrency** (§2). Each discounts the other's
   urgency.
4. **Prioritization decision <-> instrumentation.**
   `docs/soc_bus_review.md` §10 gates the Phase-2-sequencing decision on a
   real-boot L2 hit-rate number whose counters exist but are ILA-probe-only
   (`fpga_top_debug_vio.vh:169-182`); the ~300-LUT, "no timing risk" change
   that would make them AXI-readable (Phase 0a) has never landed, because
   nothing has prioritized it, because the decision it would inform hasn't
   been made.

### The proven resolution: build capability to a declared contract, unilaterally, don't wait for the other side

This project has already run this experiment three times, and it won every
time:
- **Part 88** (this session): rather than waiting for the fabric's or the
  core's worst-cone owner to move first, the AXI boundary got registered
  unilaterally. Cost: +107 FF. Result: the mutual coupling dissolved
  entirely, +1.88ns WNS, IPC cost unmeasurable. The cheapest change of the
  whole campaign was the one that stopped negotiating and just decoupled
  the interface.
- **`AxiIds.scala`**: the core already reserved ID space for 4 D-side MSHRs
  and implemented I-side unique-ID discipline *before* any fabric could
  reward it. This is why §1's fix is "flip a parameter," not a redesign.
- **`dma_engine.sv`**: built 8-ID multi-outstanding against a v1 fabric that
  caps it at 1r+1w. The moment §1 lands, it benefits with zero changes.

**Contrast, the negative proof**: every module that instead encoded the
*other side's current behavior* as its own justification
(`if_to_axi.v`, `axi_narrow_to_wide.v`, the near-miss MSHR-deletion
recommendation) became dead weight, a stale/misleading comment, or nearly
threw away real capability.

### Recommended methodology going forward

1. **Write the fabric's concurrency target down as a ratified contract
   document**, versioned next to `cpu_socket.vh` (which already does this
   for AXI widths): N>=4 outstanding reads per crossbar master, unique-ID-
   per-transaction discipline, L2C door accepts while bursts decompose,
   fetch door pre-latched. Every module then implements *to the contract*
   and cites it — not the neighbor's present-day limitation. This also
   mechanically retires the stale-comment class (`soc_bus_review.md` §S9)
   since contract-citing comments don't silently become false when a
   neighbor changes.
2. **Adopt "intermediate landings measure zero, by policy"** as the plan of
   record before starting §1 (see the in-series trap note there) — this is
   already `soc_bus_review.md:836-842`'s own prescription, just needs to be
   the agreed plan, not a review footnote.
3. **`boot_fsm` multi-outstanding writes as the first domino** — zero
   contention risk, the one link that measures a real result in isolation.
4. **Flip the core's `N_MSHR` in the same campaign as the fabric fix, not
   after** — leaving it at 1 "until the fabric is verified" just re-creates
   the standoff one turn later.

**One explicit caution, so this doesn't overcorrect**: unilateral-to-contract
is right for *capability* (depth, IDs, width) — it is **not** right for
*policy that needs a single owner*. The crossbar's write-side objection to
naive pipelining (`axi_xbar.v:1632-1653`) is a case where the existing
caution is correct, not a standoff to bulldoze. The DMA/L1-coherency
question in §3b is a policy decision needing one deliberate owner and real
evidence (disassemble the driver first) — not a race to implement two
different guesses and hope they meet.

---

## 5. Already done, keep as-is (don't rediscover these as "problems")

- **L2C front door pipelining** (F2, commit `5213100`) — 533 -> 1588 MB/s.
- **Same-ID accept-gate fix** (F4, `l2c_ctrl.v:36-42`, commit `a28d5f7`).
- **VRAM mux cap raise** (F3, `axi_vram_priority_mux3.v:196-203`).
- **MIG bridge multi-outstanding** (8 read / 4 write,
  `axi_ddr4_mig_bridge.v:15-33`).
- **Dedicated I-fetch L2C port bypassing the crossbar** (task #269,
  `fpga_top_cpu.vh:110-127`, bind at `fpga_top_ddr.vh:273-279`) — this
  directly answered and closed the "should I-fetch go through L2C"
  question; it already does, and it's the right design (shared capacity,
  and it closes the exact stale-I-fetch SMC bug class documented in
  `docs/BUG_icache_dcache_coherency.md` from the v1 era). Residual debt on
  this specific path, lower priority than §1: the fetch AR tracker is
  single-outstanding (subsumed by §1's L2C-door fix); the 256b port
  decomposes into 2x128b quadrants through the shared L2C datapath (fine
  today, worth widening only if/when §1 exposes real I/D contention, not
  before).
- **`peripheral_bus.v`'s multi-byte-write serialization — correct, not
  over-engineering.** Real VIA/SCC/ASC/5380 registers are byte-wide with
  real per-byte side effects; membership was checked against 1.69M
  MAME-classified writes; double-pulse-hazard registers are correctly
  excluded; reads aren't serialized. Leave alone.
- **The CDC bridges** (`axi_pb_s1_cdc.v`, `pulse_cdc.v`, `axi_async_bridge.v`,
  `axil_async_bridge.v`) — all guard real, necessary clock boundaries
  (peripheral island at 50MHz, MIG's own `ui_clk`). Leave alone.
- **The stale-sink/W-pad/reset-drain hardening machinery** — exists because
  of real, hardware-fought JTAG-reset-vs-in-flight-burst wedges. Simplifying
  this away would re-buy those bugs. Leave alone.

## 6. Real, scoped cleanup list (lower priority than §1-3, do opportunistically)

> **CORRECTION 2026-09-03.** Of the three bullets below, TWO were right and
> one was dangerously wrong. The criterion that settles it: **a module is
> dead if it is not reachable from a synthesized top** (`fpga_top`,
> `sd_provision_top`, `fpga_top_sdmin`). Having a testbench does NOT make it
> live — a module whose only consumer is its own test is dead weight plus a
> test that proves nothing about the shipped design.
>
> Reachability was computed mechanically over the full instantiation closure.
> **Watch out for macro-hidden instantiations**: `axi_pb_s1_cdc` is
> instantiated via `` `PB_S1_CDC_MODULE `` and looks dead to any grep — the
> file says so itself. It is live.

- `rtl/soc/axi_n64_to_wide.v` — **CONFIRMED DEAD. REMOVED** (2026-09-03).
  Not reachable from any synthesized top; its only consumer was
  `tb/tb_dma_integration_top.v`, which tested the fully-dead
  `dma_ctrl → axi_n64_to_wide → xbar → DDR` stack. Removed together with
  that testbench, its `ALL_TBS` entry and the `synth/vivado.tcl` read line.
  **It is NOT needed by the incoming DMA merge** (§3): v1 instantiates
  `dma_engine` with `.DATA_WIDTH(128)` (`fpga_top_dma.vh:243-247`) — natively
  128-bit, no widener in the path. The `axi_n64_to_wide` mention in v1's
  `fpga_top_dma.vh:11` is a historical comment about the removed `dma_ctrl`.
- `rtl/soc/vram_cpu_byteswap.v` — **CONFIRMED DEAD. REMOVED** (2026-09-03).
  Its transform moved INTO the crossbar: `axi_xbar.v:1376`'s
  `vram_swap_word32` is byte-identical and is applied to S3 `wdata`
  (`:1841`), `wstrb` (`:1843`) and `rdata` (`:3456`). The shim was left
  instantiated by nothing but `tb/tb_framebuffer_pixel.v`, and
  `tools/check_synth_sources.py` had *allowlisted* it with the reason
  "referenced only from a comment; no live instantiation" — i.e. the repo
  already knew it was dead and kept it anyway. The testbench was NOT
  deleted (it is the only pixel-exact CPU→VRAM→scanout coverage, and
  `video.v`/`vram.v`/`fb_reader`/`linebuf_scanout` are all live): the byte
  reorder is now modelled inline, mirroring `axi_xbar.v`, so scenario-B
  still reproduces the P1 pixel-mirror condition. Passes.
- ~~`rtl/soc/if_to_axi.v` — real name-collision hazard ... Remove
  deliberately.~~
  **DO NOT DELETE — this one really is wrong, and it is NOT dead.**
  `rtl/soc/if_to_axi.v` is a git-tracked **symlink** (mode `120000`) to
  `../../cpu/rtl/sys/if_to_axi.v`, so there is exactly one `module
  if_to_axi` tree-wide and no collision can occur. The duplication hazard
  was real until commit `7b4be17` (2026-07-04), which converted the copy to
  this symlink *precisely to fix it*; the bullet is stale by two months.
  The cited source (`docs/soc_bus_review.md:1137-1140`) says the
  *opposite* — it sits under "Corrections to an earlier count" and exists
  to prevent this deletion. It IS reachable from a synthesized top:
  `cpu/rtl/core/m68k_axi_wrapper.v:1142` instantiates it in every
  `CPU=m68k` bitstream, and `synth/vivado.tcl:628` reads the path
  unconditionally for all three CPU values.
  *Caution*: because it is a symlink, writing through
  `rtl/soc/if_to_axi.v` silently modifies the `cpu/` submodule.

**Still dead, but deliberately retained with a written rationale — left
alone pending an explicit decision:** `rtl/soc/dma_ctrl.v` (kept for a
future NVMe-as-SCSI engine, §3c; superseded by the incoming `dma_engine.sv`),
`rtl/soc/scsi_trace_ring.v` (`fpga_top_peripherals.vh:1981` — "deliberately
LEFT IN THE TREE ... a one-line-per-port re-instantiation away from coming
back", removed from the design for routing margin), and `sd_image_lba_map`
(defined in the live `rtl/board/sd_ctrl.v:1877`, instantiated nowhere).
- ~~`rtl/soc/if_to_axi.v` — real name-collision hazard ... Remove
  deliberately.~~
  **NO COLLISION EXISTS, AND DELETING IT BREAKS EVERY VIVADO BUILD.**
  `rtl/soc/if_to_axi.v` is a git-tracked **symlink** (mode `120000`) to
  `../../cpu/rtl/sys/if_to_axi.v` — there is exactly one `module if_to_axi`
  declaration tree-wide. The duplication hazard was real until commit
  `7b4be17` (2026-07-04), which converted the copy to this symlink
  precisely to fix it; the bullet is stale by two months. Worse, the cited
  source (`docs/soc_bus_review.md:1137-1140`) says the *opposite* — it sits
  under "Corrections to an earlier count" and exists to prevent this
  deletion. `synth/vivado.tcl:628` reads the path unconditionally for
  `CPU=stub`, `m68k` and `m68k040` alike, and `cpu/rtl/core/
  m68k_axi_wrapper.v:1142` instantiates it in every v1 bitstream.
  *Caution for future edits*: because it is a symlink, writing through
  `rtl/soc/if_to_axi.v` silently modifies the `cpu/` submodule.
- ~~`l2c_bypass` (`BYP_WIN_EN=0` in every shipping config) — generate-gate
  it~~ — **rationale stale; done differently.** The quoted 213 LUT / 320 FF
  predates commit `3bf03b3` (which made the engine a depth-8 pipelined
  queue); post-route residual at `BYP_WIN_EN=0` is **109 LUT / 12 FF**.
  Gating it would have to touch `l2c.v`'s victim-vs-bypass AW/W/B arbiter
  and the route queue — real risk in coherency-critical logic for ~109 LUT.
  The genuinely valuable half — the latent fetch-into-bypass response
  misroute — **is now a `$fatal` assertion** in `l2c_ctrl.v` (commit
  `7fab58e`), inside the existing `translate_off` region, netlist
  bit-identical.
- `VIDEO_SMOKE` defaults **on** in `fpga_top.v:210` — **DONE** (`7fab58e`),
  though not for the stated reason: production bitstreams were never at
  risk, because `synth/vivado.tcl` always passes the generic explicitly
  (`:661`) and defaults its own env knob to 0 (`:386`). The real exposure
  was every path that elaborates `fpga_top.v` *without* setting the generic
  — the lint targets and `fpga_top`-based sims.
- **`tools/build_bitstream.sh` hard-pins and verifies `CPU=m68k` (the OLD
  core)** at lines 62, 94, 159-163 — the actual blessed hardware-build
  pipeline currently refuses the new core. Every cpu040 hardware build has
  been an ad-hoc manual command line, not the real pipeline. Fix this
  independent of everything else above — it's a real landmine for whoever
  next tries to build cpu040 for hardware through the "normal" path.
- Delete the rejected `cpuBootThrottleEn` plumbing
  (`fpga_top_cpu.vh:46-53`, `fpga_top_peripherals.vh:638-666`) — a
  mechanism that was proposed, built, and explicitly rejected by the
  project owner in favor of ROM patches; leaving it wired in as dead
  capability invites accidental future use.
- Update the stale "core self-serialises" comments in the two adapter
  modules once §1 lands (or delete the modules per the cleanup above) so
  they don't keep asserting something false.

## 7. The boot-debug race-condition bug class — not SoC design debt, but institutionalize the mitigation

Not itself a SoC architecture problem: `docs/BUG_calibration_word_misplaced_0d00.md`
Part 80's verdict holds — ROM loops that measure CPU speed on purpose, or
cap counts in fixed-width registers, cannot be made to "measure slow" on a
16x-faster CPU without either lying to it (a throttle — tried, explicitly
rejected) or scaling every peripheral's real-world timebase to match (breaks
RTC/audio/video wall-clock behavior — a non-starter). Any RTL "fidelity fix"
attempted here is a throttle wearing a costume.

**The class appears convergent, not endless**: Mac OS's own design expects
variable CPU speed (that's what TimeDBRA-style calibration values are for)
— once the calibration values themselves are ROM-patched correct-for-a-fast-
CPU, downstream timed code self-corrects. Failures so far are confined to
pre-calibration loops and fixed-iteration-count saturation cases, which are
finite and enumerable (5 confirmed instances as of this doc: Parts 6-10's
original calibration-word bug, Part 79's SCC/VIA1 device-init loop, and
others logged in the doc). The newest blocker found in the broader campaign
(SCSI `c96_phase_bits`) is a genuine RTL model-completeness gap, not another
instance of this race pattern — a data point that the class is drying up.

~~**What's actually not great**: the delivery mechanism. Hardware-verified
fixes currently live as hand-edited ROM images ... **Recommendation**:
consolidate into one versioned patch table ...~~

> **CORRECTION 2026-09-03 — the diagnosis is wrong; the consolidation the
> review asks for ALREADY EXISTS. The real gap is one step further on.**
>
> There are no "hand-edited ROM images". The patches are already a single
> versioned table: `cpu/tb/models/rom_patch_sets.h` — 2,223 lines, named
> patch sets (`calibration-fix`, `via-alias-corruption-fix`,
> `machine-descriptor-slot4-fix`, `all-boot-race-fixes-v4`, the
> `bsrw-collision-shim-*` variants, and composed bundles), symlinked into
> the SoC repo as `tb/models/rom_patch_sets.h` (mode `120000`). They are
> applied by `tools/mame_patch_rom.cpp` (`make mame-patch-rom`), which
> emits a patched ROM image.
>
> **What is actually missing is the hardware delivery path.** That tool is
> host-side only: it produces a file that SIMS consume (e.g.
> `build/roms_patched/420dbff3_diagloopskip.rom`). Nothing in the build
> applies a patch set to the ROM that reaches real hardware — the board
> boots whatever `boot_fsm` streams off the SD card, and no make target
> provisions a patched ROM onto that image. So "not yet deployed to the
> canonical ROM file/default build path" is true, but the reason is a
> missing provisioning step, not a missing table.
>
> **A second, structural problem the review did not identify:** the patch
> table lives inside the **v1 `cpu/` submodule**, on the unmerged branch
> `feat/calibration-fix-rom-patch`. The SoC's ROM-patch data — which is a
> property of the *machine*, not of either CPU — is therefore hostage to a
> branch of a submodule that the **cpu040 build does not otherwise use at
> all**. A cpu040-only checkout that never initialises `cpu/` has a dangling
> symlink where its ROM patches should be.
>
> **Revised recommendation**, in order:
> 1. Move `rom_patch_sets.h` out of `cpu/` into the SoC repo proper (it
>    belongs next to the ROM images in `files/`, not inside a CPU), leaving
>    the symlink pointing the other way if v1's tbs still need it.
> 2. Add a make target that emits a patched ROM for PROVISIONING, not just
>    for MAME, and make the SD-image flow consume it.
> 3. Only then is the treadmill actually "deployed"; today it is auditable
>    (the table is good) but not delivered.
Makes the treadmill auditable instead of scattered across branches.

---

## Priority order, if picking this up fresh

1. **§1**: land Phase 2 fabric concurrency (crossbar N-outstanding + L2C
   door + fetch pre-latch) as one measured unit; start with the
   zero-risk `boot_fsm` proving ground. Flip core `N_MSHR` in the same
   campaign.
2. **§2**: run the ~140-150MHz single-domain constraint sweep to bank
   Part 88's already-paid-for timing win; decide the CDC-split question
   explicitly (recommended: yes, but only after §1); commission the MMCM
   board-level work in parallel since it's slow and independent.
3. **§3b**: before the DMA/NIC merge, disassemble the real ROM/driver's
   descriptor-ring cache-management pattern and decide ROM-patch vs.
   non-cacheable mapping. Do this *before* merging, not after first Sad
   Mac.
4. **§3a**: merge the DMA engine + SONIC NIC only after §1 lands (not
   strictly blocking, but landing it first reproduces a bug class v1 has
   already hit once).
5. **§4**: write the fabric concurrency contract doc; bump the `cpu040`
   submodule pointer to include Part 88's fix (currently pending,
   deliberately not yet done); fix `tools/build_bitstream.sh`'s `CPU=m68k`
   hardcoding.
6. **§6**: the cleanup list, opportunistically.
7. **§7**: consolidate the ROM-patch treadmill into one versioned table.

No rewrite is warranted anywhere in this review. The fabric's correctness
hardening (CDC bridges, reset-drain machinery, byte-write serialization) is
a real asset and should not be touched. The one load-bearing assumption left
over from the old core is the single-outstanding memory concurrency model,
and the project already knows, in writing, exactly how to fix it.
