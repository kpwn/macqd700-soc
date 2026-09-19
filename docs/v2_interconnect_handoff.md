# Interconnect / L2C / MIG — principles for hosting v2

Handoff for whoever builds the `m68k-core-040-ooo` integration. Everything
here is measured on this repo unless marked (E) for estimate. Written
2026-08-20, after the serialization sweep that took L2 read hits from
533 MB/s to 1588 MB/s.

Companion documents, all still current:
- `docs/soc_v2_interconnect_shape.md` — the design review this extends; §9a
  is the serialization register, §4.4 the timing-methodology finding.
- `docs/l2c_perf.md` — the perf harness and every number's provenance.
- `docs/l2c_spec.md` — invariants. §1 invariant 3 (dirty lines dropped at
  reset) is load-bearing and non-obvious.

---

## 0. The prime directive

> **"Not the bottleneck today" is not a reason to under-build.**

Almost every deferral in this repo has the form *"X dominates, so Y doesn't
matter."* Each one expires the moment X is fixed — and we are fixing all of
them. If you accept those arguments individually you will arrive at v2 with
a fabric that is uniformly mediocre and no single thing to blame.

This is not hypothetical. It happened twice in one day:

- Pipelining the L2C lookup to 1 beat/cycle **immediately** made the
  single-entry front door the new bottleneck — 2.012 cyc/op with 255
  recoverable bubbles in 256 ops. The door had measured *exactly zero cost*
  three commits earlier (`7d49e1f`), and that result was correct at the
  time: it had two cycles of slack against a three-cycle loop. Remove the
  loop, remove the slack. Shipping the pipeline alone would have delivered
  **1.5× instead of 2.97×** and looked like the pipeline underdelivering.
- Fixing `l2c_bypass`'s single-outstanding engine (`3bf03b3`) removed the
  stated reason `ENABLE_DDR_RAMDISK` is disabled — and immediately promoted
  `vhdd_ddr.v`'s *own* one-at-a-time engine to the ceiling. Neither fix
  alone buys anything.

**The rule that follows:** when you widen or pipeline something, re-measure
everything downstream **in the same change**, and expect to fix the next
thing too. Budget for pairs, not single changes.

**The corollary:** a counterfactual metric beats an occupancy metric. "Was
the door refused?" reads 66.5% and means nothing. "Was it refused *having
already been presented and refused on the previous cycle*?" reads 0 and is
the truth. Build the counterfactual, not the easy counter.

---

## 1. The ceiling stack, measured

Ordered by how hard they are to move. Everything is at `fabric_clk100`
(100 MHz) unless noted.

| # | Ceiling | Value | Notes |
|---|---|---|---|
| 1 | DDR4 device | **~10.7 GB/s** | MT40A512M16LY-075, tCK 750 ps, ×32 |
| 2 | MIG AXI port | 256-bit | already 256; we narrow to 128 ourselves |
| 3 | **Fabric** | **1.6 GB/s** | 128-bit × 100 MHz — **6.7× below the DRAM** |
| 4 | L2 read hits | 1588 MB/s | 99.2% of #3, as of `5213100` |
| 5 | L2 write streams | 1264 MB/s | full-line gather, no fill |
| 6 | Bypass reads | 286 MB/s | `3bf03b3` |
| 7 | Miss stream | 365 MB/s | latency-bound, not width-bound |
| 8 | Random miss, 8 out | 191 MB/s | Little's Law against DDR latency |

**The single most important line is #3.** At 200 MHz it becomes 3.2 GB/s
for free. Everything else in this document is downstream of that number,
and no amount of L2C work moves it.

### The cliffs — discontinuities, not slopes

| Cliff | Cost | Cause |
|---|---|---|
| **Same-ID misses** | **55.40 vs 8.38 cyc/op — 6.6×** | `l2c_ctrl` Critical-3 accept gate |
| MSHR saturation | flat past 8 outstanding | `l2c_mshr.v:110` `N = 8` |
| MIG read outstanding | flat past 8 | `axi_ddr4_mig_bridge` `RMAX_OUTSTANDING = 8` |
| MIG write outstanding | flat past 4 | `WMAX_OUTSTANDING = 4` |

Note everything is 8: MSHRs 8, victim slots 8, bypass slots 8, MIG reads 8.
That is not coincidence — each was independently argued to 8 *because the
next thing downstream was 8*. **If you raise one, raise the set**, or you
will move a queue across a boundary and measure nothing.

---

## 2. The same-ID cliff is the biggest single item on this list

Read `l2c_mshr.v`'s header in full before touching anything. Summary:

Every master reaching xbar S0 today presents **one constant AXI ID** and is
**one-outstanding** — `if_to_axi.v:16`, `axi_narrow_to_wide.v:72`,
`fpga_top_debug_host.vh:489`, `vhdd_ddr.v`. Critical-3 (`id_busy_c`) blocks
a second live op per ID, so the live MSHR table **cannot exceed 3 entries**
and the 8 look dead.

**They are not dead.** v2 is multi-outstanding on both I and D sides, which
makes all 8 reachable — *if and only if* it presents distinct IDs, or the
gate is fixed. Measured:

```
miss_read           8 outstanding, unique IDs :  8.38 cyc/op  (191 MB/s)
miss_read_single_id 8 outstanding, one ID     : 55.40 cyc/op  ( 29 MB/s)
```

An earlier revision of that header recommended shrinking `N` on the "unused"
argument. **That recommendation is withdrawn, in writing, in the file.** Do
not re-derive it. Fix the gate; do not shrink the table.

`3bf03b3` showed the shape of the fix: it qualified `id_busy_c` with
`&& !is_bypass_c` because bypass-to-bypass has no same-ID different-path
hazard. `5213100` moved the cacheable case to the resolve using a per-entry
id-match vector from `l2c_mshr`, which is what lets beats 2..N of a missing
burst reach the MSHR and merge. What remains is the general multi-ID case.

---

## 3. What is already done — do not redo it

| Area | State | Commit |
|---|---|---|
| L2 victim writeback | 8-slot pipeline, no `S_B` | `386ea29` |
| L2 bypass engine | 8-slot ring, 7.7× | `3bf03b3` |
| L2 lookup | **1 beat/cycle**, was 1-per-3 | `5213100` |
| L2 front door | 1-entry pre-latch per channel | `5213100` |
| MSHR replay payload | flops → LUTRAM | `878d002` |
| `axi_narrow_to_wide` | multi-outstanding, depth 4 | `0994eaf8` |
| n2w gather | double-buffered, 1.28→1.04 | `2858029e` |
| Crossbar seat 0 | assessed, **not** a bottleneck | §9a |

Two adapters are **dead code** — read into every build, instantiated
nowhere: `axi_n64_to_wide.v` (234 lines), `axi_pb_lane_shim.v` (344).
Delete them. Do not "consolidate" the rest; the count reflects genuine
width and clock diversity, and merging the CDC bridges or VRAM muxes just
moves complexity. The one real consolidation is parameterizing
`axi_narrow_to_wide` by width so `axi_n64_to_wide` never needs reviving.

---

## 4. The arc, in dependency order

Each item names what must land **with** it. Deviating from the pairing is
how you get the 1.5×-instead-of-2.97× outcome.

### 4.1 L2C ↔ MIG to 256-bit — do this first, it is nearly free

The MIG is **already 256-bit**. `axi_ddr4_mig_bridge` contains packing logic
whose only reason to exist is that L2C speaks 128: *"128-bit repo burst
beats are packed into 256-bit MIG INCR beats."* Widening **deletes** that
path.

Critically, **this path does not touch the crossbar**:

```
L2C master (128b) → axi_vram_priority_mux3 → axi_async_bridge
                  → axi_ddr4_mig_bridge → MIG (256b)
```

So it costs **zero crossbar congestion**, which is the objection that rules
out widening the fabric generally (§6). And `l2c_data.v` stores
`LINE_BITS = 512` — the whole 64 B line — and already reads
`WAYS*LINE_BITS = 4096` bits per access. **Zero additional URAM**, which
matters because URAM is at 100%.

Work: `l2c.v`'s master port (8 hardcoded `[127:0]`, despite having a
`DATA_WIDTH` parameter); parameter changes on the mux and async bridge
(both already parameterized); delete the MIG bridge's packing. Scanout may
widen or stay narrow — narrow transfers on a wide bus are legal AXI4.

Effect: 64 B line fill goes 4 beats → 2. Path ceiling 1.6 → 3.2 GB/s.

**Pair with:** nothing. This one is genuinely standalone.

### 4.2 Fix the same-ID accept gate

§2. The 6.6× cliff. **Pair with** whatever makes v2's masters multi-ID —
fixing the gate while every master still presents one constant ID measures
nothing, and presenting distinct IDs while the gate stands measures nothing.
Land them together or you will conclude both were worthless.

### 4.3 `if_to_axi` multi-outstanding

`if_to_axi.v` is one-outstanding, and its own header explains why that was
fine: *"The core's if_stage already self-serialises its fetches."* True for
v1. **v2 is multi-outstanding on the I side**, so this becomes a 1-deep
adapter in front of a multi-outstanding fetcher on day one, and it will
present as the new core underperforming.

**Pair with:** v2 bring-up. There is nothing to measure before then.

### 4.4 Tag→data serialization and per-way banking

This is the big one, and `docs/soc_v2_interconnect_shape.md` §3.3 has the
full analysis. Summary:

URAM288 has exactly two ports and `l2c_data.v` uses both (UG901 simple
dual-port). There is **no free second port**, and duplicating the array
needs 64 more URAM288 that do not exist. **But the array is already eight
independent banks and the RTL discards that**: writes are already per-way,
only reads broadcast one address to all eight arrays and mux 4096 bits
afterwards.

Serialize tag → data and the banking falls out:

| | today | tag-first |
|---|---|---|
| data bus out of the URAM column | 4096 bits | **512** |
| way-select mux | 1578 LUT | **deleted** |
| URAMs activated per access | 64 | 8 |
| **concurrent data accesses** | 1 | **up to 8** |
| L2 hit latency | 4 cyc | ~6 cyc (E) |

To use the concurrency you need two tag lookups per cycle, and **tags are
BRAM at 24.8% utilisation** — a duplicate tag array costs +17 RAMB36 /
+8 RAMB18 and ~0 LUT, on the one axis with real headroom.

#### 4.4a Go further: bank per (way, quadrant), not per way — 32 banks, same URAM

§3.3 of the design review stops at per-way addressing (8 banks). It does not
have to. **Every bus-width slice of a line can be independently addressed,
and it costs no extra URAM.**

The arithmetic is exact, not approximate:

```
today:  8 ways × [511:0] mem[0:4095], ram_style="ultra"
        512 bits ÷ 72 = 8 URAM288 per way × 8 ways = 64  ← all of them
split:  128-bit quadrant ÷ 72 = 2 URAM288 per quadrant
        2 × 4 quadrants × 8 ways = 64                    ← identical
```

The URAM288s are **already physically there, one per quadrant-pair**. Today
`l2c_data.v` broadcasts a single `raddr` to all of them and muxes 4096 bits
afterwards. Giving each quadrant-pair its own address changes no memory at
all — it changes the address distribution and deletes the mux.

Bank count goes **8 → 32**. For a random pair of sub-line accesses the
conflict probability goes from 1/8 to **1/32**.

**And the metadata is already quadrant-granular**, which is what makes this
natural rather than bolted on: `rvsec`/`rdsec` are 32 bits (8 ways ×
4 sectors), `req_cov`/`req_dty`/`req_want` are per-quadrant `[3:0]`, and
`victim_push_dsec` is `[3:0]`. Sectored valid/dirty already exists and is
load-bearing — `00f079c`'s full-line-write path and `386ea29`'s partial
writebacks both depend on it. **The tags already think in quadrants; only
the data array's addressing does not.**

Three honest limits, so nobody over-sells this:

1. **A full 64 B line request needs all four quadrants of one way**, so
   line-granular traffic gets its parallelism from *ways* (8), not from
   quadrants. L1 refills are line-granular. The quadrant win is for sub-line
   accesses, for a 256-bit port that takes 2 quadrants and leaves 2 free,
   and for decoupling fill/writeback traffic from the hit stream
   per-quadrant instead of per-way.
2. **The cost is address distribution.** One 12-bit address broadcast today
   becomes up to 32. Tag-first (§4.4) mitigates this decisively: once the
   way is resolved you address only the hit way, so it is 4 addresses, not
   32. **Do not attempt quadrant banking without tag-first** — the address
   fan-out lands in exactly the windows already at congestion level 5–6.
3. Read/write already do not conflict (URAM's two ports carry `wr_set` and
   `raddr` independently). The conflict this removes is **two reads to
   different sets**, which today is a hard global serialization because
   `raddr` is shared across all eight ways.

**Sequencing:** this is an extension of §4.4, not an alternative. Land
tag-first addressing first, measure, then split the address per quadrant.
Both together are what make §4.5's 256-bit I-side port cheap — a wide port
is 2 quadrants of one way, drawn from banks the D-side is not using.

**Pair with:** v2's non-blocking L1. The +2 cycles of hit latency is only
free if the L1 can absorb it. v1's L1D is blocking with no hit-under-miss
(`dcache.v:899`: `req_ready = (state == S_IDLE) && !cooldown`), so it would
feel both cycles on **every** hit. **Do not land this before v2**, and
measure it on v2's own IPC, not on a bandwidth harness.

### 4.5 The 256-bit I-side port

v2 has a 256-bit I-side. Do **not** widen the fabric to serve it (§6). Give
L2C a dedicated wide port fed from the core, leaving the crossbar,
peripheral bus and VRAM path at 128. This is the same change as §4.4 — a
second read port is a per-way slice of an array that already emits 4096
bits, plus the duplicated tag array.

**Pair with:** §4.4. They are one piece of work.

### 4.6 The remaining one-at-a-time engines

- **`vhdd_ddr.v`** — `r_outstanding`, `S_RD_AR→S_RD_DATA`, `S_WR_AW→S_WR_B`.
  Live. Now the RAM-disk ceiling since `3bf03b3` removed the L2C one.
- **`dma_ctrl.v`** — `S_DESC_AR→S_DESC_R`, `S_RD_AR→S_RD_R`,
  `S_WR_AW→S_WR_W→S_WR_B`.
- `boot_fsm.v`'s `ST_ZERO_AW→W→B` — measured at **5%** because L2C's
  full-line-write path already absorbs the round trip. **Leave it.** This is
  the one place where "not the bottleneck" survived measurement.

---

## 5. Design principles

1. **Pipeline, do not deepen.** Three of today's four biggest wins were
   turning a one-at-a-time FSM into a pipeline. Adding queue depth in front
   of a serial engine buys a constant; pipelining buys a factor.
2. **Argue depth from the system, not from a knee.** The isolated model
   never saturates (`l2c_bypass` was still improving at 32 slots). Depth 8
   ships because `axi_ddr4_mig_bridge` takes 8 reads. `0994eaf8` shipped
   depth 4 because depth 8 bought nothing on any burst shape real masters
   use. Name the downstream limit you are matching.
3. **Retire on the response, count the rest.** `386ea29` and `3bf03b3` both
   collapse to a counter rather than a table because everything issues under
   one AXI ID, so AXI4 guarantees in-order responses and identity stays
   positional. Preserve that property; it is worth more than it looks.
4. **AXI4 has no WID.** Two sources' W bursts may never interleave. The
   owner-lock in `l2c.v` is the pattern: lock the port to one owner while it
   has writes outstanding, but let that owner run many in flight.
5. **Keep work out of the resolve cone.** `miss_ok_c`, `req_cov`,
   `req_dty`, `req_want` are all decided at accept time specifically so they
   are not in `S_LOOKUP`'s cone. Solving a throughput problem by moving
   logic into that cone is how you convert a perf win into a timing loss.
6. **Prefer LUTRAM/flops to comparators.** Depth in distributed RAM is one
   LUT6/bit at any depth ≤32 and adds nothing to a cone. A CAM feeding
   hit-resolve is expensive at any depth.
7. **A mux's worst input is its delay.** STA takes the worst leg, not the
   live one. `3b021305` had to add a conversion-free `b_ext_gen` purely so
   the *generic* comparator leg was not fed by the converter.

---

## 6. The hard walls — check these before proposing anything

Post-route, from the last completed build:

| resource | used | headroom |
|---|---|---|
| CLB LUTs | 166,895 / 216,960 | **76.9%** |
| **URAM** | **64 / 64** | **0%** |
| CLB Registers | 114,945 / 433,920 | 26.5% |
| Block RAM | 119 / 480 | 24.8% |
| Congestion | level 5 global, **level 6** South Long | router initial |

**URAM is full.** Any proposal needing one more URAM288 is dead. This is
why §4.4 works (reuses the array) and why set-banking does not (2-way
banking needs 128 URAM288; 4-way needs 256).

**Congestion is already 5–6.** This is why widening the *fabric* to 256 is
expensive and widening the *private L2C↔MIG path* is not. Doubling every
AXI data path doubles wires in exactly the congested windows.

**FF and BRAM are where the room is.** Spend there.

### Timing methodology — read `docs/soc_v2_interconnect_shape.md` §4.4

`synth/vivado.tcl:1302` applies `set_max_delay -datapath_only` whenever
`TARGET_FREQ_MHZ != 200`. It excludes clock skew and uncertainty, and it is
active in **every build we do**. Measured cost on the real design:
**0.172 ns**. True pre-today `fabric_clk100` WNS was **−0.154 ns**, not the
+0.018 we quoted for months.

**Post-build measurement, 2026-08-20 (M).** Validation impl on `a51dd5f`,
scoped to `fabric_clk100` on `route.dcp`, exception removed by overriding
the same `set_max_delay -from/-to` **without** `-datapath_only`:

| check | as shipped | honest |
|---|---|---|
| setup WNS | **+0.111** (0 failing of 360,202) | **−0.064** (~25 of 3000 sampled) |
| exception cost | — | **0.175 ns** |

0.175 independently corroborates the 0.172 measured the day before on the
same clock tree. Hold is **+0.012, 0 failing of 80,526**. The only other
violation is **WPWS −0.043**, a `Min Skew` check between `RXTX_BITSLICE`
`D[2]`/`D[1]` **inside the DDR4 MIG PHY** (`BITSLICE_RX_TX_X0Y36`) — vendor
IP, unaffected by our RTL.

For scale: the honest baseline **before** any of today's work was
**−0.154**. It is now **−0.064** — 0.090 ns *better*, while adding FPU I2X,
the full-format FPU EA decode, and five interconnect pipelines. The
interconnect work is timing-neutral-to-positive; the FPU cone is the whole
story, and `3b021305` recovered most but not all of it.

Two consequences for you:

- **The first 200 MHz build removes the exception**, so real setup checks
  run for the first time *and* the period halves. Slack will fall by the
  clock-network term for reasons that have nothing to do with v2. **This
  will look like the new core broke timing.** It is the constraint becoming
  honest.
- Out-of-context synthesis does **not** reproduce this: measured 0.022–0.027
  ns there, because OOC has no real clock tree. OOC is valid as a relative
  A/B only. Never claim closure from it.

---

## 7. Traps that actually bit us

Each of these cost real time. They are cheap to avoid and expensive to
rediscover.

1. **Your first test will pass its own mutant.** This happened **six times
   in one session**, each for a different reason: a park scenario that
   parked a *middle* group so the last-beat branch was never reached; an R+W
   interleave where reads covered for the write side; a same-set relaxation
   that 61 scenarios could not distinguish; six directed tests hand-encoded
   to the bug they were meant to catch. **Assume your first version is
   vacuous and go hunting.** RED-verify every new scenario against a named
   mutation and show the others stay green.
2. **Tests can be written to the bug.** `1dfe3479` found six tests encoding
   `.short 0xF200,0x4000` commented "FMOVE.S" — but `ext1[12:10]` is the
   source *format* and `000` is long integer. `isa_status.md` had recorded
   the resulting behaviour as *deliberate*. When a test and the spec
   disagree, assemble the instruction and look.
3. **`git submodule update --init` fails silently in agent worktrees**, and
   a dangling `cpu/` dangles `rtl/soc/axi_narrow_to_wide.v`, which stops ~5
   gates from **building** — which reads exactly like passing. Verify the
   symlink resolves before trusting any gate. See `docs/agent_policy.md`.
4. **Never copy a shared file out of a worktree.** Worktrees fork from
   spawn-time HEAD; a wholesale `cp` of `Makefile` silently reverted 58
   lines of a test gate. The tell is a *symmetric* insertion/deletion count.
   Integrate by patch.
5. **Unscoped timing reports are contaminated.** An unscoped WNS on this
   design read −8.827 from the DDR domain while `fabric_clk100` was fine.
   Always scope to the clock you mean.
6. **`check-cpu-sync`'s suggested `rsync` runs toward `cpu/`.** If your work
   is newer, following it reverts your own commit. Advance the other repo
   instead, after confirming it is a fast-forward.

---

## 8. Open measurements

- **M2 (unchanged from the design review): real DDR round-trip latency.**
  Still unmeasured; every bandwidth and scanout-margin claim rests on it.
  `docs/soc_bus_review.md` Phase 0 has a JTAG pointer-chase procedure
  needing **no rebuild**. Run it before sizing anything with Little's Law.
- **v2's L1 miss rate and L1 geometry.** §4.4's latency trade and the MSHR
  depth argument both want it. Ask the core team.
- **`inst_hist` 1-deep survives the whole suite** (`5213100`). Pre-existing,
  not introduced — but the second entry is load-bearing and nothing proves
  it. A new `l2c_tags` uniqueness assertion watches for the corruption
  family it causes. Worth closing.
- **`arr_ce_c` fanout.** `5213100` put the resolve decision on a clock
  enable with 8-way URAM fanout. Fallback documented in-file (~256 LUT hold
  mux after the way-select) if post-route says it binds.
