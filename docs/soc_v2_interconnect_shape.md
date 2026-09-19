# Is the interconnect the right *shape* to host `m68k-core-040-ooo`?

Status: **analysis only.** No RTL changed, no Vivado run, no board touched,
nothing committed.  2026-08-20, against the Aug 20 00:33 routed report set.

This is the design-thinking half of a two-part goal.  The mechanical half —
turning every serialisation point into a pipeline — is in flight elsewhere
and is **not re-argued here**; where those fixes matter, this document
reasons about what they *imply* rather than restating them.

## 0. What is new here, and what is deliberately not

Four reviews already exist and are load-bearing.  Read them; this document
builds on them and contradicts them only where new data forces it:

| Document | What it owns | Status of its numbers |
|---|---|---|
| `docs/soc_bus_review.md` | F1-F9 footgun inventory, hop-by-hop latency map, socket contract | Sound. **Its area and timing figures are two builds stale** — see §1.1 |
| `docs/l2c_perf.md` | Measured L2 throughput, the 6.6x same-ID cliff, sectoring, full-line writes | Throughput measurements sound. Area figures superseded — §1.1 |
| `docs/soc_clocking_study.md` | Fabric/CPU/L2 clock choice, time-multiplexing, 64b-at-333 refutation | **Conclusions endorsed.** §4 here adds device evidence its author did not have |
| `docs/dma_engine_design.md` | One shared peripheral master hub, raw + coherent modes | Endorsed. §5 here adds the routing argument for it |

**What this document adds that none of them contain:**

1. The **physical geometry of the L2 data array** — read out of the routed
   report, not the RTL — and the three architectural conclusions that
   follow from it, two of which close off options that have been discussed
   as open (§2, §3).
2. A **demand model expressed as saturation thresholds** rather than as a
   guess at v2's miss rate: "at what L1 miss rate does each structure
   break", which is directly answerable and does not require data we do not
   have (§3.2, §7).
3. An **empirical ns-per-logic-level rule** for this device and speed
   grade, derived from two independent cones in this very design, which
   converts "can the fabric run at X MHz" from an argument into arithmetic
   (§2.3, §4.1).
4. A **timing-signoff finding**: the reported `+0.018 ns` WNS is not a
   conventional setup check, and the reason it is not will disappear the
   moment anyone builds at 200 MHz (§4.4).
5. The **area bound on v2 itself**, which turns out to be the tightest
   constraint in the entire system (§7.1).

### 0.1 Method

Every number is one of:

- **(R)** read from `build/vivado/reports/utilization_route.rpt`,
  `build/vivado/timing_summary.rpt`, or
  `build/vivado/reports/congestion_route.rpt` (Aug 20 route, except
  congestion — Aug 10, flagged wherever used);
- **(S)** read from the RTL with a file:line citation;
- **(M)** a measurement quoted from `docs/l2c_perf.md` /
  `docs/soc_bus_review.md`;
- **(E)** arithmetic derived from the above, labelled ESTIMATE.

Where a question cannot be settled without data that does not exist, §9
says so and states the measurement that would settle it.  Four of the
things asked end there, and that is the correct answer to them.

---

## 1. The five answers, up front

**Q1 — Is a single-ported, single-front-door L2 the right structure for an
OoO core?**  Single **front door**: no, and it is already being fixed.
Single **ported array**: yes, and the question is moot — the device makes
every alternative either impossible or free.  Set-banking the L2 is
**physically impossible** on this part; multi-porting the data array is
**physically impossible**; and the array is **already eight independent
banks** that the RTL currently throws away by broadcasting one read address
to all of them.  §3.

**Q2 — Should the fabric run faster than the core?**  No for the CPU↔L2
path, yes for everything below the L2 — which is what
`docs/soc_clocking_study.md` already concluded.  New evidence strengthens
it: the 333 MHz domain in this design closes with **+0.027 ns**, so it has
no room for the extra logic that "move the fabric to DDR clock" would put
in it.  Time-multiplexing does **not** substitute for real ports, but the
question is moot for the same reason as Q1 — the ports are already there.
§4.

**Q3 — Is the topology right?**  The *shape* is right and under-recognised:
a coherent tier above the L2 and a bandwidth tier below it is the correct
two-tier structure, and the design already has it by accident.  The
implementations are wrong (both arbiters hand-written, fixed-width,
single-outstanding), and the **seat count is the wrong thing to optimise** —
a seat costs routing, not LUTs, which is the strongest available argument
for the user's one-shared-DMA-engine direction.  §5.

**Q4 — Where is the real headroom?**  Not in the fabric.  Ranked: (1) the
concurrency work already in flight, (2) **not decoupling the fabric clock
from the core clock**, (3) serialising the L2's tag→data access, (4)
matching L1 line size to the L2 line, (5) topology.  Items 3 and 4 are new.
§6.

**Q5 — What breaks first?**  **The die.**  Not the L2, not the crossbar, not
DDR.  v1's CPU is 107,289 LUT of a 166,406 LUT design (R); to keep the
design at a routable 85% LUT, **v2 may be at most ~17% larger than v1**.
Congestion is already level 5 (placer) / 6 (router) and every congestion
window reports **URAM 100%**, because the L2's data array is a single
column that runs the **full height of the device** and the CPU is placed
around it.  §7.

### 1.1 Two corrections to the prior reviews, because they change rankings

Both are from the Aug 20 routed report (R) and both are improvements — the
prior documents are pessimistic, not wrong:

| Claim in prior docs | Aug 20 reality | Consequence |
|---|---|---|
| `u_l2c` = 24,222 LUT, `u_mshr` = 20,953 LUT (13.0% / 11.2% of design) | `u_l2c` = **10,000**, `u_mshr` = **6,893** | The l2c area work landed. `u_mshr` is no longer "the largest piece of dead capability in the SoC". Any ranking that put L2 area work near the top should be re-sorted |
| Design at 86-87% LUT, WNS −0.612 ns, three builds failing | **76.7% LUT** (166,406/216,960), **WNS +0.018**, 0 failing endpoints, DRC fully routed | "Area is not affordable" becomes "area is affordable, but only just, and only for a v2 that is not much bigger than v1" |
| l2c accept cone is the binding path (0.077 ns, #1, 21 levels, 82% route) | **No `u_l2c` path appears anywhere in the 652 constrained max-delay paths.** All 50 worst `fabric_clk100` paths are one FPU cone | The l2c cone's current slack is **unknown, and > 0.055 ns**. §9/M1 gets the number in two minutes |

The last row matters most for this review's framing.  The premise handed to
me — "the l2c accept cone is 0.407 ns, #25 of 50, 21 levels, 82% routing" —
is from a superseded report.  **I could not confirm it and I could not
refute it**, because the current report does not contain the path.  Every
conclusion below that depends on that cone is written to be robust to
either answer, and §9/M1 is the two-minute measurement that settles it.

---

## 2. The physical facts that constrain everything

These are read out of the routed reports.  They are not design choices that
can be revisited by writing different RTL; they are properties of
`xcku5p-ffvb676-2-i` and of the geometry already chosen.

### 2.1 The L2 data array is one column, the full height of the die

`rtl/soc/l2c_data.v:52` declares **eight independent per-way arrays**, each
`4096 x 512 bits`, each `(* ram_style = "ultra" *)` (S).  512 bits needs
eight URAM288s in width (8 x 72 = 576, so 64 bits per line are unused);
4096 deep is exactly one URAM288 depth.  Eight ways x eight URAMs = **64
URAM288, and the device has exactly 64** (R).

Where they are is the part that matters.  Every URAM site instantiated in
this design is in **column X0, rows Y0 through Y63** (R, extracted from
site names in `timing_summary.rpt` / `reports/timing_place.rpt`).  There is
one URAM column on this part, it is 64 sites tall, it spans the device, and
`u_l2c` owns 100% of it.

Three consequences, and none of them is avoidable by rewriting `l2c*.v`:

- **The L2 bisects the die.**  All five placer congestion windows report
  **URAM 100%**, spanning CLEM_X43Y99-X59Y130 through CLEL_R_X41Y61-X57Y92
  (R, Aug 10 — stale but structural).  The cells crowded around it are
  `u_cpu/u_cpu` (up to 75% of a window), `u_rob` (up to 39%), `u_commit`
  (19%), `u_iq_mem` (28%), `u_lsu` (11%).  **The CPU is placed around the
  L2's column because the column cannot move.**  That is the mechanism
  behind congestion level 5/6, and it gets worse, not better, when a
  larger core lands.
- **Any signal that reaches all eight ways spans the column.**  The shared
  `raddr`, `wr_set`, `wr_way`, `wr_data[511:0]` and `wr_strb[63:0]`
  (`l2c_data.v:30-39`) all fan out over 64 sites of height.
- **The read bus is 4,096 bits wide.**  `rdata` is `WAYS*LINE_BITS`
  (`l2c_data.v:32`) — all eight ways are read in parallel on one shared
  address and returned to a way-select mux in `l2c_ctrl`.  That mux is
  `u_data`'s entire **1,578 LUT with 0 FF** (R).  **This is the single
  largest routing structure in the fabric**, and it exists purely because
  the read address is broadcast.  §3.3 is about deleting it.

### 2.2 The resource profile is lopsided, and it dictates every trade

| Resource | Used | Available | Util | Implication |
|---|---:|---:|---:|---|
| LUT | 166,406 | 216,960 | **76.7%** | the binding budget |
| FF | 120,961 | 433,920 | 27.9% | free |
| RAMB36 | 98 | 480 | 20.4% | free |
| RAMB18 | 42 | 960 | 4.4% | free |
| URAM288 | **64** | **64** | **100.0%** | exhausted, and immovable |
| DSP48E2 | 27 | 1,824 | 1.5% | free |

(R, `utilization_route.rpt` + `utilization_synth.rpt` for device totals.)

**The rule that follows, and it should be applied to every proposal below:
spend FF, BRAM and DSP; do not spend LUT; do not ask for URAM.**  Pipeline
registers are free.  Duplicated tag arrays are free.  Wider muxes are not.

### 2.3 The timing picture, and the empirical design rule it yields

All 50 worst `fabric_clk100` paths are one cone in the CPU, launched from a
single flop `u_cpu/u_cpu/u_iq_fp/e_valid_reg[2]` and fanning through 34
logic levels of shared `s0_sp_res[*]` logic into both `u_fpu/u_add` (19
endpoints) and `u_fpu/u_div` (16) result registers (R).  Slack +0.018 ns;
9.908 ns of data path, **3.351 ns logic (33.8%) / 6.557 ns route (66.2%)**;
physical span SLICE_X71Y53 -> SLICE_X86Y65.

The second cone is `u_ddr` — the MIG async bridge and the 128->256 width
converter — in the 333.33 MHz `mmcm_clkout0` domain: 15 of the top 50, best
slack **+0.027 ns**, 10 logic levels, 2.888 ns at 71.8% route (R).

Normalising both gives a rule this device has just told us:

| Cone | Domain | ns | levels | **ns/level** | route % |
|---|---|---:|---:|---:|---:|
| FPU add/div | 100 MHz | 9.908 | 34 | **0.291** | 66% |
| MIG bridge (path 2) | 333 MHz | 2.888 | 10 | **0.289** | 72% |
| MIG bridge (path 19) | 333 MHz | 2.795 | 11 | 0.254 | 62% |
| l2c accept (Aug 19, **STALE**) | 100 MHz | 9.948 | 21 | **0.474** | 82% |

Two deep cones in two different clock domains, placed by two different
strategies, land within 1% of each other at **~0.29 ns per logic level**.
The stale l2c number is 63% worse per level, and the reason is stated in
its own report: seven hierarchy transitions and three crossings of the die
width — the URAM anchoring of §2.1.

**The rule (E):** budget **0.29 ns/level for a compactly-placed cone, 0.47
ns/level for one that crosses the die.**  Therefore:

| Target | Period | Compact budget | Die-crossing budget |
|---|---:|---:|---:|
| 166.67 MHz (`mmcm_clkout6`, exists, **+1.509 ns** slack today) | 6.0 ns | ~20 levels | ~12 levels |
| **200 MHz** | 5.0 ns | **~17 levels** | ~10 levels |
| 333.33 MHz (MIG UI) | 3.0 ns | ~10 levels | ~6 levels |

This is the arithmetic §4 uses instead of adjectives.  Note what it says
about the CPU too: the FPU cone is 34 levels with **3.351 ns of pure logic**,
so at 200 MHz it would consume 67% of the period before a single wire.  A
200 MHz core means roughly **≤17 levels everywhere**, and that is a core
constraint, not a fabric one.

---

## 3. Q1 — Is a single-ported, single-front-door L2 right for an OoO core?

Split the question, because the two halves have opposite answers.

### 3.1 The front door: no, and that is the work in flight

Not re-argued.  `docs/l2c_perf.md` §1.3 and `docs/soc_bus_review.md` F2
establish it: the front door is one `st` register, beat-granular, one AR
globally, 3 cycles occupancy per 16 B beat, and it re-serialises I-fetch
against D-side traffic that the crossbar carefully separated.  4 cycles
AR->RVALID on a hit (S, `l2c_ctrl.v:206,368,657,675` + `l2c.v:371`).  That
is being pipelined now.  §3.2 is about how much pipelining is enough.

### 3.2 How much miss parallelism does a 200 MHz OoO 68040 actually need?

**I cannot tell you v2's miss rate, and neither can anyone else without
running it.**  `m68k-core-040-ooo` is a sibling repository; nothing in this
tree describes its L1 geometry, its MSHR count or its issue width.  So the
question is inverted: *at what miss rate does each structure saturate?*
That is answerable from this tree alone.

Parameters and their status:

| Symbol | Meaning | Value | Status |
|---|---|---|---|
| `F` | core clock | 200 MHz | target, given |
| `IPC` | sustained IPC | 1.5 | project goal, `CLAUDE.md` |
| `b_I` | instruction bytes per instruction | 3.5 | ESTIMATE (68k avg 2-6 B) |
| `r_D` | data references per instruction | 0.5 | ESTIMATE — 68k is a memory-operand ISA, so higher than RISC's ~0.35 |
| `d` | fraction of L1D evictions that are dirty | 0.30 | ESTIMATE |
| `m_I`, `m_D` | L1I misses/instruction, L1D misses/data-ref | **unknown** | the thing being solved for |
| `R` | fabric clock / core clock | 1.0 or 0.5 | the Q2 decision |

v1's L1 geometry, for calibration (S): L1D **4 KB, 4-way, 32 B lines, 32
sets**, write-back, blocking, one outstanding (`dcache.v:61,66-70,899`);
L1I **4 KB, 4-way, 16 B lines, 64 sets**, one outstanding
(`icache.v:12,24-27`).  The L2 line is 64 B and the front-door beat is 16 B
(S).

Front-door beats consumed per instruction, at v1's line sizes:

```
  I side:  m_I  x (16 B / 16 B)          =  1.0  m_I
  D side:  r_D  x m_D x (32/16) x (1+d)  =  1.3  m_D
```

Saturation is `IPC x cycles_per_instruction = R`.  With `m_I = m_D = m`:

| Front door | Cycles/beat | `R = 1.0` (fabric = core) | `R = 0.5` (fabric = core/2) |
|---|---:|---:|---:|
| Today (3 cyc/beat, 1 at a time) | 3 | **saturates at m = 9.7%** | **saturates at m = 4.8%** |
| Pipelined (1 beat/cycle) | 1 | saturates at m = **29.0%** | saturates at m = 14.5% |

(E.  Sensitivity: at IPC 1.0 every threshold rises 50%; at `r_D = 0.7` the
D-side term grows 40%, moving the pipelined `R=1` threshold from 29% to
~23%.  The ranking is insensitive to both.)

**Read the table this way.**  A 4 KB L1 pair on a machine running System 7
plausibly sits somewhere in the 5-15% region — that is the range where
today's front door is the constraint and a pipelined one is not.  It is
*not* plausibly at 29%.

Restating the same thing as an absolute demand figure, because §4.3 needs
it: front-door beats per core cycle is `IPC x 2.3 x m = 3.45 m`, so

| `m` | beats/core-cycle | vs a pipelined 1-beat/cycle port |
|---:|---:|---:|
| 5% | 0.17 | 5.9x headroom |
| 10% | 0.35 | 2.9x |
| 15% | 0.52 | 1.9x |

So:

> **A single, fully-pipelined, 128-bit L2 front door at core clock has
> roughly 2-6x headroom over any credible v2 demand.  A second front door
> is not needed.  A banked L2 is not needed for bandwidth.**

The same conclusion from the concurrency side, via Little's Law.  To
sustain `B` bytes/s of fill traffic at line-fill latency `L` you need
`N = B x L / 64 B` outstanding fills:

| Fill bandwidth | L = 100 ns | L = 150 ns | L = 250 ns |
|---|---:|---:|---:|
| 400 MB/s | 0.6 | 0.9 | 1.6 |
| 1 GB/s | 1.6 | 2.3 | 3.9 |
| 3.2 GB/s (the whole 128b@200MHz port) | 5.0 | 7.5 | 12.5 |

(E.  `L` is unmeasured — see §9/M2 — but the conclusion holds across the
whole plausible range.)  `MSHR_N = 8` (S, `l2c_mshr.v:110`) covers the
**entire fabric port** at any latency under ~160 ns.  **Do not deepen the
MSHR.  Do not shrink it either** — 8 is right, and `docs/l2c_perf.md` §4.2
already withdrew the proposal to shrink it, correctly.

And the DDR is not remotely in the picture: DDR4-2666 x32 = **10.67 GB/s**
device-side, MIG AXI 256 b @ 333.25 MHz = 10.66 GB/s (S,
`synth/gen_ddr4_mig.tcl:150,153,246,257`), against a 128 b @ 200 MHz fabric
port of 3.2 GB/s.  **The fabric port is a 3.3x bottleneck on the DRAM, and
the CPU is a 2-6x bottleneck on the fabric port.**  Nothing about DDR needs
to change for v2.

### 3.3 The array: banking is impossible, multi-porting is impossible, and you already have eight banks

This is where §2.1's geometry becomes decisive, and it closes off two
options that have been discussed as open.

**Set-banking is physically impossible.**  Splitting 4096 sets into `B`
banks means each (bank, way) array is `4096/B` deep.  URAM288 is 4096 deep
and is not sub-dividable — you cannot get two independent addresses out of
one primitive.  So at B = 2 each of the 16 (bank, way) arrays still needs 8
URAM288s in width and burns 8 of them at 50% depth utilisation:
**8 x 8 x 2 = 128 URAM288 for a 2-way-banked 2 MB L2.  The device has 64.**
At B = 4 it is 256.  Set-banking is not a cost trade; it does not fit.

**Multi-porting the data array is physically impossible.**
`l2c_data.v:56-65` is the UG901 simple-dual-port template: one synchronous
write port, one synchronous read port, in one always block.  URAM288 has
exactly two ports, and this uses both.  The usual escape — duplicate the
array for a second read port — requires 64 more URAM288 that do not exist.

**But the array is already eight independent banks, and the RTL discards
it.**  Look again at `l2c_data.v`:

```verilog
    wire this_wen = wr_en && (wr_way == w[WAY_BITS-1:0]);   // :55  per-way WRITE
    ...
        dout_r1 <= mem[raddr];                              // :63  shared  READ
```

The **write** side is already per-way — a write touches one way's eight
URAMs and leaves the other 56 idle.  Only the **read** side broadcasts one
address to all eight arrays and returns 4,096 bits into a way-select mux.

That broadcast exists for exactly one reason: the design reads tags and
data *in parallel* and picks the way afterwards.  That is the right choice
for an **L1**, where hit latency is everything.  It is the wrong choice for
an **L2**, and essentially every real L2 does the opposite — serial tag
access, then read only the hit way.

**The proposal: serialise tag -> data.**  Read tags in cycle 1, resolve the
way in cycle 2, present a per-way read address in cycle 3, data out in
cycle 5.  Hit latency goes from 4 cycles to ~6 (E).  What it buys:

| | Today | Tag-first |
|---|---|---|
| Data read bus out of the URAM column | **4,096 bits** | **512 bits** (8x reduction) |
| Way-select mux (`u_data`) | **1,578 LUT** (R) | **deleted** |
| URAMs activated per access | 64 | 8 (power win, unmeasured) |
| Concurrent data accesses | 1 | **up to 8**, conflict probability 1/8 for a random pair |
| L2 hit latency | 4 cycles | ~6 cycles (E) |
| Miss path | reads data it discards | reads no data at all |

The concurrency line is the interesting one and it costs nothing extra: two
lookups that resolve to different ways can read simultaneously, because
they already are different memories.  **You do not need to build a banked
L2.  You need to stop pretending the banked L2 you have is one memory.**

To use that concurrency you also need two tag lookups per cycle — and
**tags are BRAM, which this design has in abundance**.  `u_tags` is 17
RAMB36 + 8 RAMB18 (R) out of 382 + 918 free.  A duplicate tag array for a
second read port costs **+17 RAMB36 / +8 RAMB18 and ~0 LUT**, on the one
resource axis with 80% headroom.

So the honest shape of the answer to Q1:

> The device gives you a **cheaply duplicable tag array** and an
> **inherently 8-banked, un-duplicable data array**.  The structure that
> fits is asymmetric: **N-wide tag/control, 8-bank data.**  Both roads to
> L2 concurrency are open and neither costs URAM.  Neither is needed for
> bandwidth; the tag->data change is worth doing for **routing and area**,
> and the concurrency arrives as a side effect.

**Two honest caveats.**

1. **+2 cycles of L2 hit latency is only free if the L1 is non-blocking.**
   v1's L1D is blocking with no hit-under-miss (`dcache.v:899`), so it
   stalls on every L2 hit and would feel both cycles in full.  This change
   must land **with** v2, not before it, and it needs a measured
   before/after on v2's own IPC.  `docs/soc_clocking_study.md` §7.5 makes
   the same point about the hit path generally, and it applies here.
2. It restructures the front-door pipeline that three agents are currently
   pipelining.  **Sequence it after their work lands**, or the two changes
   collide in the same 200 lines of `l2c_ctrl.v`.

---

## 4. Q2 — Should the fabric run faster than the core?

`docs/soc_clocking_study.md` answered this in depth and its conclusion —
*keep the CPU and all of `l2c` in the CPU clock domain; move the DDR-facing,
CPU-independent traffic to a faster domain; target UI/2 = 166.67 MHz rather
than 333* — is **endorsed without modification**.  I will not re-derive it.
What follows is four things the Aug 20 report adds that its author did not
have.

### 4.1 333 MHz is arithmetically out of reach for the L2 path

From §2.3's rule: 3.0 ns buys **~10 logic levels compact, ~6 die-crossing**.
The L2 front door was 21 levels at its last measurement.  Even granting
that the l2c area work (24,222 -> 10,000 LUT) shortened it, and even
granting perfect compact placement, **it would have to lose more than half
its depth** to run at the MIG UI rate.  That is a redesign of the accept
cone, not a retime — and it would have to survive being anchored to a
64-site-tall URAM column (§2.1), which is what pushed the old cone to 0.474
ns/level in the first place.

For 200 MHz the same arithmetic is demanding but not absurd: **~17 levels
compact**.  For 166.67 MHz: ~20 levels, i.e. roughly where the front door
already is.  That ordering — 166.67 plausible, 200 hard, 333 out — is the
same ordering the clocking study reached by a different route.

### 4.2 The 333 MHz domain has no room in it

This is the finding that most sharpens the clocking study's recommendation.
Its proposal is to move `axi_vram_priority_mux3`, `scanout_ddr_reader` and
the future DMA engine **into** `mig_ui_clk`.

The Aug 20 report says that domain is **already nearly full**: 15 of the 50
worst paths in the design are in `u_ddr`, best slack **+0.027 ns**, on a
module that is nothing more than five async FIFOs and a 128->256 width
converter, at 10-11 logic levels (R).

The candidates for relocation are not that simple.  `axi_vram_priority_mux3`
is 495 lines of priority arbitration with quota counters and a coupled
reset handshake; `scanout_ddr_reader` is 758 LUT of address generation and
credit tracking (R).  Dropping either into a domain with 0.027 ns of margin
is a real risk, not a rewiring.

**Refinement, not reversal:** target the relocation at **`mmcm_clkout6`,
166.67 MHz**, which already exists in this design, is a MIG-generated
integer divisor of the UI clock, and closes today with **+1.509 ns of
margin** (R).  That is 1.67x the current fabric rate, it keeps an integer
clock relationship to the MIG rather than adding an irrational one, and it
has fifty times the timing headroom of the 333 MHz domain.  It is also, not
coincidentally, the clock the clocking study recommends for the fabric
itself (its §5.4).

A related point worth stating explicitly because it cuts against the 200
MHz target: **200 MHz is not a divisor of the MIG UI clock (333.25 MHz);
166.67 MHz is.**  If v2 cannot reach 200 MHz anyway — and §2.3's FPU-cone
arithmetic suggests it will be a fight — then 166.67 MHz is the target that
makes the whole clock tree rational and removes an asynchronous boundary
instead of adding one.  That trade should be made deliberately, not
discovered.

### 4.3 Time-multiplexing does not substitute for real ports — and does not need to

Assessed on its own terms, then dismissed for a better reason.

**On its own terms it is weak, and the arithmetic is worse than it looks.**
`docs/soc_clocking_study.md` §7.3 already corrected the multiplex ratio
from ~3x to ~1.1x and the correction stands: a 3-cycle-occupancy pipeline
at 333 MHz serves 111 M beats/s, which at a 200 MHz core is **0.56 beats
per core cycle**.  Against §3.2's demand table that is:

| `m` | demand (beats/core-cycle) | multiplexed 333 MHz port | pipelined port at 200 MHz core clock |
|---:|---:|---:|---:|
| 5% | 0.17 | 3.3x headroom | **5.9x** |
| 10% | 0.35 | 1.6x | **2.9x** |
| 15% | 0.52 | **1.1x** | **1.9x** |

**A time-multiplexed physical port in the MIG UI domain delivers roughly
half the throughput per core cycle that an ordinary pipelined port at core
clock does** — because three fast cycles per op is worse than one slow one
when the fast clock is only 1.67x.  It is the more expensive option *and*
the slower one.

**The cost is not symmetric with the benefit.**  Time-multiplexing pays for
a logical port with (a) a clock the L2 cannot reach (§4.1), (b) a CDC on
the CPU's hit path, and (c) worse single-access latency on the one path
where latency is what the CPU stalls on.  You would be spending the L2's
hit latency to buy throughput it does not need.

**And it is moot.**  §3.3: the data array is *already* eight physical
banks.  Concurrency at the array is a control-logic change, available at
any clock, costing no URAM and no CDC.  There is no version of this design
in which time-multiplexing a physical port is the cheapest way to get a
second logical one.

The one place the clocking study endorses time-multiplexing — collapsing
`l2c_mshr`'s flat register files into small memories — remains correct, is
already partly landed (`r_pay` is LUTRAM now, S, `l2c_mshr.v:196-199`), and
is **not gated on the clock**.  Its own §7.1 says so.

### 4.4 A finding about the +0.018 ns that changes what "meets timing" means

`synth/vivado.tcl:1302-1314` applies, whenever `TARGET_FREQ_MHZ != 200`:

```tcl
    set_max_delay -from $core_budget_clks -to $core_budget_clks \
        $target_period_ns -datapath_only
```

`Makefile:2348` sets `TARGET_FREQ_MHZ ?= 100`, so **this exception is
active in every shipped build**.  The worst path's own header confirms it:
`Timing Exception: MaxDelay Path 10.000ns -datapath_only` (R).

`-datapath_only` **excludes the clock network entirely** — skew,
uncertainty and jitter.  All 284,274 `fabric_clk100` endpoints are being
analysed this way.  There is no `set_clock_uncertainty` in any XDC, so
Vivado's auto-derived uncertainty is exactly what is being discarded.  On
this device and clock structure the omitted term was predicted to be order
0.1-0.4 ns (E); the reported margin is **+0.018 ns**.

**Measured 2026-08-19 (M).**  Against `build/vivado/checkpoints/route.dcp`,
scoped to `fabric_clk100` (an unscoped report is contaminated by the DDR
domain, which reads -8.827 once `reset_timing` drops the IP clocks):

```
ASSHIPPED_CORE_WNS: 0.018   TRUE_CORE_WNS: -0.154
both: u_cpu/u_cpu/u_iq_fp/e_valid_reg[2]/C
   -> u_cpu/u_cpu/u_fpu/u_add/s0_sp_res_reg[5]/R
```

**Same endpoints under both analyses**, so the 0.172 ns delta is exactly
the excluded clock skew + uncertainty -- inside the predicted range, at the
low end.  Consequence 1 below is therefore confirmed, not merely likely:
the design does **not** close 100 MHz under a conventional setup check.

Two things follow that outrank the rest of this document's rankings:

- **Every timing claim in this repo is +0.172 ns optimistic**, including
  each one made while the interconnect work below was landing.  Budget the
  skew term explicitly in future claims rather than quoting reported WNS.
- **The binding path is the FPU issue cone in the CPU, not the fabric** --
  `u_iq_fp -> u_fpu/u_add`, the same cone that owns 22 of the 50 worst
  paths (S2.3).  Pipelining the interconnect's serialization points is
  right for *throughput*, and the victim-writeback work (386ea29) measured
  6.8-7.8x on eviction traffic, but it is not what is limiting Fmax.

Three consequences worth stating plainly:

1. **The design's 100 MHz closure is probably not real** under a
   conventional setup check.  It is quite likely negative.
2. **This will surface as a step change at 200 MHz and will look like an
   RTL regression.**  At `TARGET_FREQ_MHZ=200` the exception stops being
   applied, and real setup checks with skew and uncertainty run for the
   first time on this design.  Whoever builds v2 at 200 MHz will see slack
   fall by the clock-network term for reasons that have nothing to do with
   v2 — and the natural reading will be "the new core broke timing".
3. It is **not a bug** — the exception is a deliberate, documented
   relaxation for running a 200 MHz-constrained board clock at a 100 MHz
   target.  It is a **methodology hazard**, and it is exactly the class
   `docs/soc_bus_review.md` §9 catalogues: a correct thing whose
   side-effect is invisible.

§9/M1 is the two-minute measurement that quantifies it.

---

## 5. Q3 — Is the topology right?

### 5.1 The shape is right, and nobody has said so

Today: `axi_xbar`, 3 top-level masters x 6 slaves with 4 internal fan-in
slots (S, `axi_xbar.v:288-289,772-774`); then `l2c` on S0; then a *second*
arbiter, `axi_vram_priority_mux3`, below the L2, merging L2's master port
with scanout and the S3 VRAM write lane (S, `fpga_top_ddr.vh:448`).

`docs/soc_bus_review.md` §8a treats the second arbiter as a defect —
"`axi_vram_priority_mux3` exists because the crossbar could not be extended
(F7)... arbitration belongs in the crossbar."  **I disagree, and the
disagreement matters for v2.**  Those two arbiters are doing genuinely
different jobs:

| | Above the L2 | Below the L2 |
|---|---|---|
| What it merges | things that must be **coherent** | things that only need **bandwidth** |
| What binds | **latency** — the CPU stalls on it | **throughput and real-time deadlines** |
| Right clock | core clock, always | as fast as it will close (§4.2) |
| Right arbitration | fairness, low fan-in | priority + quota (scanout is hard real-time) |
| Right placement | compact, adjacent to CPU + L2 tags | anywhere; near the MIG |

They are the **coherent tier** and the **bandwidth tier**, and the split
falls exactly where `docs/l2c_spec.md` §1 puts the point of coherency.
Folding them into one crossbar would put hard-real-time scanout quota logic
and CPU load latency in the same arbiter and the same clock domain.  That is
a step backwards.

**Recommendation: make the two tiers explicit and stop treating the second
arbiter as an accident.**  `docs/dma_engine_design.md` already reaches the
same structure from the other direction — its "sys port on M3, raw port as a
2:1 tap at the `l2c_m -> mux3` seam" is precisely a coherent-tier client and
a bandwidth-tier client.  It should be described as the architecture it is.

### 5.2 Seats cost routing, not LUTs — which is the argument for the shared DMA hub

The stated direction is that peripherals should reach the bus through
**one** shared DMA engine rather than each taking a master seat.  That is
right, and the usual justification (LUT area) is the weak one.  `u_xbar` is
**4,151 LUT, 2.5% of the design** (R) — nobody should care.

The real cost is in §2.3's routing term.  A master seat adds a row to the
fan-in mux at every slave and a column to the response mux at every master;
the R-return path is deliberately **combinational** (`axi_xbar.v:3446-3466`)
to save a cycle, so a 4x6x128b mux is one combinational structure spanning
the whole crossbar.  Every seat added widens the structure that lives in the
congestion window (§2.1) the CPU is already fighting for.

So the ranking is: **seat count is a routing budget, and routing is the
scarce thing in this design.**  Concretely for v2:

| Seat | Who | Verdict |
|---|---|---|
| M0 | CPU LSU (+ boot FSM via a 2:1 reset-hold mux) | keep |
| M1 | host debug / JTAG-AXI | **keep** — `docs/dma_engine_design.md` §7.4's debug-independence argument is correct and should not be traded |
| M2 | CPU instruction fetch | keep — the one genuinely good decoupling in the fabric |
| M3 | freed by the RAM-disk removal (`ENABLE_DDR_RAMDISK` is defined nowhere, S) | **give to the DMA hub**, per `docs/dma_engine_design.md` §5.1 |

That is four, and it is full.  Two things follow that should be written into
the socket contract now, not discovered later:

- **v2 must not ask for a fifth master seat.**  A separate table-walk port,
  a separate store port, a separate prefetch port — none can be
  accommodated: `NW = NR = 4` with hand-written round-robin pickers
  (`axi_xbar.v:772-773,2009-2032,3370-3393`, S) and
  `docs/soc_bus_review.md` F7 explains why widening them is a rewrite.  It
  does not fail slowly; **it fails to elaborate.**  (v1's MMU walker
  already does the right thing: it shares the D-side bus through
  `mmu_walker_dcache_bridge`, S, `m68k_core_memory.vh:178,411`.  v2 should
  keep that.)
- **Bandwidth per seat is the right lever instead.**  One 128 b seat at 200
  MHz is 3.2 GB/s, which §3.2 shows is 2-6x a credible v2's demand.  Seats
  should get *outstanding depth*, not multiplicity.

### 5.3 What should move, for v2

Three boundary changes, in benefit-per-risk order:

1. **Non-cacheable I/O off the latency tier.**  Today a VIA register poll
   travels CPU -> `axi_narrow_to_wide` -> M0 -> S1, sharing `rs_state[0]`
   with DDR reads (`docs/soc_bus_review.md` F5) and parking the blocking
   L1D for ~20-25 cycles.  In a two-tier design, non-cacheable I/O has no
   business on the tier whose job is CPU load latency.  Note honestly:
   **the dominant half of this cost is CPU-side** (`dcache.v:899`) and
   belongs in the core repo; the fabric half is worth doing anyway because
   it is a decode change, not new logic.
2. **Delete `l2c_bypass` and the three arbiters that exist only to serve
   it.**  `BYP_WIN_EN` is 1 only under `ifdef ENABLE_DDR_RAMDISK`, which is
   defined in no Makefile and no tcl (S) — **the bypass window is disabled
   in every current build**, and `u_bypass` is down to 148 LUT (R) from
   213.  `docs/l2c_perf.md` §5.1 costs the full deletion at ~620-950 LUT
   and ~340 FF, plus one file and the priority-inversion hazard
   `docs/l2c_spec.md` spends half a section on.  The DMA hub's raw mode is
   the replacement and it is already designed.  **Do not let a bypass
   window come back through the front door for direct-colour scanout or
   anything else** — `docs/soc_bus_review.md` S3 is the standing warning.
3. **Formalise the bandwidth tier's clock** as §4.2 recommends
   (`mmcm_clkout6`, not the UI clock).

---

## 6. Q4 — Where the real headroom is, ranked by benefit-per-risk

The blunt answer the question invites: **the fabric is not the constraint.
The CPU is** — for area (64.5% of the LUTs), for timing (50 of the 50 worst
paths), and for demand (§3.2 says a pipelined front door has 2-6x
headroom).  The fabric's job is to not get in the way, and after the
in-flight concurrency work, mostly it will not.

| # | Change | Benefit | Risk | Area (E) | Owner |
|---|---|---|---|---|---|
| **1** | **F1+F2+F3+F4 as one change set** (in flight) | Prerequisite. Without it v2 is throttled to 1 in flight and charged 6.6x on top if it reuses an ARID (M) | Medium — four modules, and **any subset measures as zero** | +4.5-6k LUT (prior E) | in flight |
| **2** | **Do not decouple the fabric clock from the core clock** | Avoids a CDC on every L1 miss and keeps §3.2's saturation threshold at 29% instead of 14.5% | **Negative risk** — it is a decision not to do something | 0 | decision |
| **3** | **ID rotation in the socket contract** | Turns a silent 6.6x (M) into a non-event. Costs the fabric nothing | Very low | 0 | `cpu_socket.vh`, before v2 lands |
| **4** | **L2 tag -> data serialisation** (§3.3) | −1,578 LUT, 8x cut in the largest routing structure in the fabric, 8-way data concurrency for free | Medium — restructures the pipeline three agents are editing; **+2 cycles of hit latency that only a non-blocking L1 absorbs** | **−1,578 LUT**, +~40 FF | with v2 |
| **5** | **Match L1 line size to the L2 line (64 B)** (§6.1) | 4x fewer I-side and 2x fewer D-side L2 transactions per byte | Low, but it is a **core-repo** change | ~0 (BRAM-neutral) | core repo |
| **6** | **DMA hub on M3** | Removes PIO from every byte of disk traffic — plausibly the largest user-visible deficit in the machine | Medium | +0.1-1.2k LUT (prior E) | `docs/dma_engine_design.md` |
| **7** | **Two-tier topology made explicit; I/O off the latency tier** (§5.3) | Removes head-of-line blocking of DDR reads by VIA polls | Low-medium | ~0 (decode) | fabric |
| **8** | **Relocate the bandwidth tier to `mmcm_clkout6`** (§4.2) | Headroom for DMA + VRAM, +1.509 ns of margin to work in | Medium — a CDC boundary moves | +~0.7k LUT (one more bridge) | after 1-3 |
| **9** | Delete `l2c_bypass` and its arbiters | −620-950 LUT, −1 file, −1 hazard class | Low (no consumer in any build) | −620-950 LUT | fabric |

Items 4, 5 and 7 are new to this review.  Items 1, 3, 6 and 9 are prior
work re-ranked against the Aug 20 numbers, not re-derived.

Note item 1's area against §7.1's budget: **+4.5-6k LUT of fabric work
consumes 9-12% of the total LUT headroom v2 has.**  That is not a reason
not to do it — it is a reason to land items 4 and 9 (both area-*negative*,
together roughly −2.2 to −2.5k LUT) in the same window.

### 6.1 The change the question list missed: L1 line size

The most leveraged change available is not in the fabric at all, and it is
not "make the fabric faster".  It is **make less traffic reach the fabric**.

v1's L1I line is **16 B** against an L2 line of **64 B** (S).  So:

- One L1I miss requests 16 B.  If it misses in L2, the L2 fetches 64 B from
  DDR (`arlen=3`, S, `l2c_mshr.v:353`) and returns 16.
- The next three sequential I-fetch misses hit in L2 — but each is a
  **separate front-door transaction**, with its own AR, its own MSHR
  same-line check, its own ID, its own accept.
- **Four front-door round trips to consume one L2 line**, on the I side.
  Two on the D side (32 B lines).

Against a requester that is latency-bound (v1: `if_to_axi` is one
outstanding, S, `:16-18`) that is a 4x multiplier on stall time.  Against a
multi-outstanding v2 it is a 4x multiplier on *transaction* pressure — AR
slots, ID space, MSHR same-line merge traffic, and front-door accepts —
which is exactly the resource §3.2 shows is nearest to saturation.

Making both L1 lines 64 B collapses it to one transaction per line, at
identical byte-level bandwidth and identical BRAM cost (4 KB is 4 KB;
fewer sets, wider lines).

**Architectural legality, since this is a 68040:** `CPUSH`/`CINV` LINE
operate on the architectural 16 B line.  A 64 B physical line means those
operations affect four architectural lines at once.  **Over-invalidating
and over-pushing are both safe** — invalidate discards a clean copy that
will be re-fetched; push writes back data that is already correct in the
cache.  The only cost is performance, and only on
cache-maintenance-heavy code.  This is legal; it needs to be written down,
and it needs a directed test, because "we widened the line and CPUSH LINE
now touches 4x the data" is precisely the kind of thing that is correct in
review and wrong in silicon.

I have **not** modelled the miss-rate effect of the wider line — spatial
locality cuts both ways (fewer, larger fetches, but fewer lines resident in
4 KB).  §9/M3 is the measurement.  The transaction-count argument above
does not depend on it.

---

## 7. Q5 — What breaks first, and at what threshold

Ordered by *when*, not by severity.  This is the most useful single answer
in the document, so thresholds are stated even where they are estimates.

### 7.1 First: the die. v2 may be at most ~17% larger than v1.

| | LUT |
|---|---:|
| Design total, Aug 20 route (R) | 166,406 |
| `u_cpu` (v1, being replaced) (R) | 107,289 |
| Everything else | **59,117** |
| Device | 216,960 |

Solving `59,117 + v2 <= X% of 216,960`:

| Target occupancy | Max v2 LUT | vs v1 |
|---|---:|---:|
| 85% (routable, given congestion is already 5/6) | **125,299** | **+16.8%** |
| 90% (painful) | 136,147 | +26.9% |
| 100% (does not route) | 157,843 | +47.1% |

(E from R.)  Congestion is already **level 5 placer / level 6 router** at
76.7% (R, Aug 10 — stale, and the current build is 10 points *better* on
LUT, so treat it as an upper bound on today's congestion).  Every
congestion window reports URAM 100% and is dominated by `u_cpu/u_cpu`,
`u_rob`, `u_commit`, `u_iq_mem`, `u_lsu` — the CPU packed around the L2's
immovable column.

**This is the constraint that governs every other recommendation in this
document.**  A superscalar OoO 68040 with a real FPU, a wider ROB, more
physical registers and multiple L1 MSHRs, at ≤117% of a design that already
has a 32-entry ROB and an FPU, is a demanding target.  If v2 comes in
larger, every fabric proposal that is area-positive — including the
in-flight concurrency work — has to be re-costed against it, and the
area-negative ones (§6 items 4 and 9) stop being nice-to-have.

**It fails loudly** (`route_design` fails), which is the one mercy here.

### 7.2 Second: the timing methodology, on the first 200 MHz build

§4.4.  At `TARGET_FREQ_MHZ=200` the `set_max_delay -datapath_only`
exception stops applying and 284,274 endpoints get a conventional setup
check — including clock skew and uncertainty — for the first time in this
design's history.  Expect reported slack to drop by the clock-network term
(0.1-0.4 ns, E, unmeasured) for reasons unrelated to any RTL change.
**Threshold: the first build at `TARGET_FREQ_MHZ=200`, before v2 exists.**

### 7.3 Third: the crossbar seat count, if v2 asks for one

`NW = NR = 4`, hand-written pickers, all four accounted for once the DMA
hub takes M3 (§5.2).  **Threshold: v2's first request for a third CPU-side
master port.**  Fails to elaborate — loudly, and at the worst possible
moment.  Mitigation is a socket-contract line, not RTL.

### 7.4 Fourth: the L2 front door, and it depends entirely on the clock decision

From §3.2, with `IPC = 1.5` and v1's line sizes, the combined L1 miss rate
`m` at which the front door saturates:

| | fabric = core | fabric = core/2 |
|---|---:|---:|
| Today's 3-cycle beat-serial front door | **9.7%** | **4.8%** |
| Pipelined, 1 beat/cycle | 29.0% | **14.5%** |

**Threshold: `m` above ~5% if the fabric is left at half the core clock and
the pipelining does not land; above ~15% if the fabric is decoupled but
pipelined; above ~29% if neither.**  Only the first of those is likely to
be reached.  **This is the strongest argument in the document for keeping
the fabric at core clock:** the clock decision is worth exactly a factor of
two on this threshold, and it is free.

### 7.5 Fifth and beyond: things that do not break

- **MSHR depth.**  8 entries covers the entire 128 b fabric port at any
  fill latency under ~160 ns (§3.2).  Never binds.
- **DDR bandwidth.**  10.67 GB/s device-side against a 3.2 GB/s fabric port
  and ~1 GB/s of credible aggregate demand (scanout 189 MB/s + L2 fills +
  DMA).  **~9% utilised.**  Never binds.
- **The L2 data array.**  §3.3.  It is eight banks; demand is 0.17-0.52
  accesses per core cycle against a single-bank capacity of 1.  Never
  binds — and it has 8x that in reserve the moment the way-select
  broadcast goes away.
- **The peripheral bus.**  Measured at ~5x faster than real Q700 silicon
  (`docs/soc_bus_review.md` §3c).  Leave it alone.

---

## 8. Not worth doing — and why

Each of these has been raised somewhere; each should be closed.

| Proposal | Verdict | Why, specifically |
|---|---|---|
| **Set-bank the L2 data array** | **Impossible, not expensive** | 2-way set-banking needs 128 URAM288; 4-way needs 256; the device has 64 (§3.3). Not a cost trade — it does not fit |
| **Multi-port the L2 data array** | **Impossible** | URAM288 has two ports and `l2c_data.v:56-65`'s SDP template uses both. Duplication needs 64 URAM that do not exist (§3.3) |
| **Build a second L2 front door** | Unnecessary | One pipelined port at core clock is 2-6x credible demand (§3.2). `docs/l2c_perf.md` §7.4 deferred the 2-stage front door on *area* grounds; §3.2 says it is not needed on *demand* grounds either, which is the stronger reason |
| **Run the fabric at the MIG UI clock (333 MHz)** | No | §4.1: needs the accept cone under ~10 logic levels. §4.2: the 333 MHz domain closes at **+0.027 ns** today and cannot absorb `mux3` + scanout + DMA. Use `mmcm_clkout6` (166.67 MHz, **+1.509 ns**) instead |
| **Time-multiplex one port into N logical ports** | No | Ratio is ~1.1x not 3x (`soc_clocking_study.md` §7.3, endorsed); it buys throughput the design does not need while spending hit latency it cannot afford; and §3.3 shows the ports already exist for free |
| **Widen the CPU socket to 256 bits** (reserved in `cpu_socket.vh:71`) | **No — and it is worse than neutral** | It doubles the column-crossing read bus (§2.1) on a LUT- and routing-bound design, on the exact structure §3.3 wants to shrink 8x. `docs/soc_bus_review.md` §7 row 5 already flags it as "looks like a win, delivers ~0"; the routing argument makes it actively negative |
| **Deepen the MSHR past 8** | No | Little's Law: 8 covers the whole fabric port at any latency under ~160 ns (§3.2). Also **do not shrink it** — `docs/l2c_perf.md` §4.2 withdrew that, correctly |
| **Shrink the L2 to 1 MB to relieve the URAM anchoring** | No | It buys routing at the cost of capacity, when §3.3 buys 8x more routing relief at the cost of 2 cycles. Do the tag->data change and keep the 2 MB |
| **Move the L2 tags into the URAM's spare bits** | No | 64 bits/line are unused (§2.1) and a 22-bit tag entry would fit — but it puts the tag behind the data array's 2-cycle latency and its single port, which is the exact opposite of §3.3. The ~2 Mbit is genuinely free and there is genuinely nothing worth putting in it |
| **Fold `axi_vram_priority_mux3` into the crossbar** (`soc_bus_review.md` §8a item 4) | **Disagree — do not do this** | It merges the coherent tier with the bandwidth tier, putting hard-real-time scanout quota logic and CPU load latency in one arbiter and one clock domain (§5.1). The second arbiter is the right structure badly implemented, not the wrong structure |
| **Give peripherals their own master seats** | No | Seats cost routing in the congestion window the CPU is already fighting for (§5.2). One shared DMA hub on M3 is right, and `docs/dma_engine_design.md` already specifies it |
| **Re-enable an L2 bypass window for anything** | No | Single-op, single-beat, and it head-of-line blocks the whole front door (`docs/l2c_perf.md` §5.1: measured 7.7x-34x CPU stall during a bypass stream). The DMA hub's raw mode is the replacement |
| **Optimise the peripheral bus** | No | Already ~5x faster than real Q700 silicon. The cost is head-of-line blocking (F5), which is a decode change and mostly a CPU-side one |
| **Add L2 back-invalidation to make inclusion an invariant** | Not now | `docs/l2c_perf.md` §8 costs it. There is no incoherent DMA writer into Mac RAM today (its §8.2), and the hash presence filter (its §8.5d) is sound without inclusion for ~1 RAMB18. Revisit when the DMA hub's coherent mode is real |

---

## 9. What I could not determine, and the measurement that would settle it

Four things asked here cannot be answered from this tree.  Listing them is
more useful than guessing.

**M1 — What is the L2 accept cone's actual slack?  (Two minutes. Do this first.)**
No `u_l2c` path appears in any of the 652 constrained max-delay paths in
the Aug 20 report; the only figure anyone has is from a superseded build
(§1.1).  `build/vivado/checkpoints/route.dcp` exists, so this needs
`open_checkpoint` plus a targeted `report_timing` — **not an implementation
run**:

```tcl
open_checkpoint build/vivado/checkpoints/route.dcp

# 1. What the L2 front door actually costs now.
report_timing -through [get_cells -hier -filter {NAME =~ *u_l2c/*}] \
              -max_paths 30 -nworst 30 -path_type full_clock_expanded \
              -file /tmp/l2c_cone.rpt

# 2. ANSWERED 2026-08-19 -- see S4.4.  True fabric_clk100 WNS is
#    -0.154 ns vs the +0.018 ns as shipped, same endpoints, so the
#    excluded clock term is 0.172 ns.  Script retained as the recipe
#    for re-measuring after any clocking change.
# 2. What the +0.018 ns becomes under a conventional setup check.
#    Drop ONLY the max-delay exceptions; leave the two legitimate
#    set_false_path CDC constraints in synth/fpga_top.xdc:117-118 alone.
foreach e [get_timing_exceptions -quiet] {
    if {[get_property -quiet MAX_DELAY $e] ne ""} { delete_timing_constraint $e }
}
report_timing_summary -delay_type max -max_paths 20 \
                      -file /tmp/no_exception_summary.rpt
```

Result (1) tells you whether §3.3's tag->data change is a timing *fix* or
merely an area one.  Result (2) quantifies §4.4/§7.2 — how much of the
+0.018 ns is real.  **Nothing else in this document is as cheap or as
decisive.**

**M2 — What is the real DDR round-trip latency?**  Still unmeasured, still
the assumption every bandwidth and scanout-margin claim in the repo rests
on (`scanout_ddr_reader.v:77-80` is honest about it).
`docs/soc_bus_review.md` Phase 0 gives a complete JTAG pointer-chase
procedure needing **no rebuild**, using the existing 64-bit cycle counter at
`0x5090_1000`.  It has apparently still not been run.  It bounds the
Little's Law table in §3.2 and it is the difference between the
1024x768x24 scanout mode closing and not.

**M3 — What is v2's L1 miss rate, and its L1 geometry?**  Unknowable from
this repository; `m68k-core-040-ooo` is a sibling repo and nothing here
describes it.  §3.2 is written as saturation thresholds precisely so that
it does not need this number — but §6.1's L1-line-size recommendation and
§7.4's threshold both want it.  The measurement is v2's own simulator
running a System 7 boot, reporting L1I and L1D misses per instruction and
per data reference.  **Ask the core team for it before committing to a
fabric clock ratio**, because §7.4 shows the clock decision is worth 2x on
that threshold.

**M4 — Is v2 within the ~17% area budget?**  §7.1.  Nobody can answer this
without synthesising v2.  It is the single most consequential unknown in
the project and it should be established with a **synthesis-only run of v2
standalone**, long before anyone attempts integration.  A standalone
`synth_design` on the core is cheap relative to a full impl, and it
converts §7.1 from a warning into a plan.

---

## 9a. Serialization-point register (2026-08-19/20 sweep)

Written against the goal "tackle all serialization points into actual
pipelines."  Every candidate that was named is listed with its verdict, so
none gets re-litigated from its *shape* instead of its *measurement*.  The
recurring lesson: **four of the six look identical in the RTL -- a
one-at-a-time FSM with a state that waits for a response -- and they have
completely different costs.**  Shape did not predict cost in a single case.
The register also now carries a case where the VERDICT ITSELF expired: #4
was correctly measured as free, and stopped being free the moment #6
removed what was hiding it.

| # | Point | Verdict | Evidence |
|---|---|---|---|
| 1 | `l2c_victim.v` -- writeback held AW/W grant to B | **FIXED** `386ea29` | 6.8-7.8x on eviction; chain 3.745 -> 2.000 cyc/word; VB-full 86.6% -> 0.0% |
| 2 | `axi_narrow_to_wide` -- one txn/direction to B/RLAST | **FIXED** `0994eaf8` | 64-beat 2.20 -> 1.28 cyc/word; depth 4, depth 8 buys nothing |
| 3 | `l2c_bypass.v` -- one DDR round trip per 16 B beat | **FIXED** `3bf03b3` | 1376 -> 179 cyc per 512 B read (37.2 -> 285.6 MB/s); shared CPU hits 23.2 -> 5.23 |
| 4 | `l2c_ctrl.v:206` front door -- one AW + one AR in flight | **WAS NOT, THEN WAS: FIXED** (`7d49e1f` measured it, the lookup pipeline voided that measurement, now 2-deep) | At a 3-cycle lookup: REFUSED-EARLIER = 0 on every cacheable row, AR refused 66.5% of presented cycles at 3.008 cyc/op.  At 1 beat/cycle: 2.012 cyc/op with 255 REFUSED-EARLIER bubbles in 256 ops. One-entry pre-latch per channel -> 1.016 cyc/op, REFUSED-EARLIER back to 0 |
| 5 | `axi_xbar.v:887` -- CPU/boot share crossbar seat 0 | **NOT A BOTTLENECK** | mutually exclusive by reset sequencing; deselected side guaranteed idle, which is why it is a 2:1 mux with no arbiter.  Reads were never muxed |
| 6 | `l2c_ctrl.v` tag lookup -- ONE request in the S_IDLE/S_WAIT/S_LOOKUP loop | **FIXED** (this change) | 3-stage pipeline, 1 beat/cycle: `hit_read` 3.00 -> 1.01 cyc/op (533 -> 1588 MB/s, 99.2% of the 128 b x 100 MHz fabric ceiling); `hit_read_64B_burst` 12.01 -> 4.02; `stream_read_64B` 63.8 -> 17.5 at DDR-40 and 223.8 -> 31.1 at DDR-200 |

Two notes worth carrying forward:

**On #4, and why a "not a bottleneck" verdict can have a shelf life.**
`7d49e1f`'s finding was correct and its reasoning was correct, and the
reasoning is exactly what expired.  The tag pipeline was `S_IDLE ->
S_WAIT -> S_LOOKUP` with accept only in `S_IDLE`, so one beat per 3
cycles was the ceiling regardless; `ar_have`/`aw_have` cleared on the edge
that took a burst's *last* beat, so the next header saw a 2-cycle window
of `s_arready` high.  **Two cycles of slack against a three-cycle loop** --
the door could not cost anything because the loop was always the tighter
constraint.

Row 6 removed the loop, and with it the slack.  A 1-beat burst's header
could then only be latched the cycle *after* the previous burst's beat was
taken, and its own beat the cycle after that: 2.012 cyc/op, with 255 of
256 ops showing a REFUSED-EARLIER bubble.  The door went from free to
being the entire remaining cost, without a line of it changing.  The fix
is a one-entry pre-latch per header channel (`ar_pend_*`/`aw_pend_*`), not
a wider `s_*ready` expression: `s_arready = !ar_pend_v` stays a register
bit, whereas `!ar_have || <burst ends this cycle>` would have pulled the
accept cone into the AXI ready path.

The metric survived intact and is the reason the flip was diagnosable in
one run: not "was it refused" (66.5% before and ~0% after, both times
meaningless) but "was it refused *having already been presented and
refused on the previous cycle*".  Keep measuring that one.  Its
anti-vacuity guard did NOT survive -- "AR refused >50% of presented
cycles", which proved the old test was stressing the door, is false by
construction once a 2-deep door faces a 1/cycle pipeline, because this
master can no longer outrun it.  `test_front_door_throughput_floor` now
guards vacuity with "the master presented a header on >=95% of cycles"
plus "every header presented was accepted" instead.

**Two coupled measurements, one lesson.**  #4 and #6 had to be measured
*together*: pipelining alone delivered 3.00 -> 2.01 (1.5x), and only with
the door deepened does it reach 3.00 -> 1.01 (2.97x).  Shipping either
half alone would have shown a third of the win and looked like the
pipeline had underdelivered.

**Sweep closed 2026-08-20, then reopened and re-closed the same day.**
All six settled: four fixed, two measured and closed.  #4 flipped from
"measured and closed" to "fixed" as a direct consequence of #6 -- see the
note above; a serialization point that is provably free can stop being
free when the thing that was hiding it goes away.  The gather lever below was taken as well (`2858029e`), so the
interconnect has no known un-pipelined serialization point left.  What
remains are *paired* changes, each needing a partner module moved with it:
`BYPASS_SLOTS` past 8 needs `axi_ddr4_mig_bridge`'s `RMAX_OUTSTANDING`;
n2w depth past 4 is now credit-limited on 16-beat bursts (1.32 vs 1.18 at
depth 8); and the read-side `rg_*` second bank is the remaining 1.28 ->
~1.0 on burst reads.

**The lever that was not on this list, now taken.**  `0994eaf8` measured a
1.28 cyc/word floor in `axi_narrow_to_wide` that is **width conversion,
not outstandingness**: four narrow beats pack into one wide beat and the
handover cycle is a cycle `n_wready` is low, so 4 beats cost 5.  Double-
buffering the gather output takes it to 1.0 -- **~20% on every burst write
in the design**, and it is the actual lever on the boot pre-zero pass
(`boot_fsm`'s own AW/B loop was measured at 5%, because L2C's full-line-
write path already absorbs that round trip).  **Taken in `2858029e`**:
1.28 -> 1.04, chain-level -15% to -19.7% depending on burst length.  It
lands at 1.04 rather than 1.00 because the gather path itself reaches 1.0
-- 64 narrow beats accepted on 64 consecutive cycles -- and the residual is
one cycle per *transaction*, the deliberate `aw_release` bubble that keeps
`w_wready` off the AW-ready path.

Ranking caveat: none of this moves Fmax.  S4.4's measurement puts the
worst path in the CPU's FPU issue cone (`u_iq_fp -> u_fpu/u_add`), so
everything here is throughput and latency, not clock.

---

## 10. Assumption register

Everything below is an ESTIMATE that a conclusion leans on.  If one is
wrong, the section named is what changes.

| # | Assumption | Value | Used by | If wrong |
|---|---|---|---|---|
| A1 | v2 sustained IPC | 1.5 | §3.2, §7.4 | Thresholds scale as 1/IPC. At IPC 1.0 all of §7.4 rises 50%; the ranking is unchanged |
| A2 | 68k data references per instruction | 0.5 | §3.2 | At 0.7 the pipelined `R=1` threshold moves 29% -> ~23%. Ranking unchanged |
| A3 | 68k instruction bytes per instruction | 3.5 | §3.2 (I side) | Scales the I-side term linearly |
| A4 | Dirty fraction of L1D evictions | 0.30 | §3.2 | ±0.2 moves the D-side term ±15% |
| A5 | Clock skew + uncertainty omitted by `-datapath_only` | 0.1-0.4 ns | §4.4, §7.2 | M1 measures it directly |
| A6 | ns/logic-level, compact / die-crossing | 0.29 / 0.47 | §2.3, §4.1 | Derived from two independent cones in this design (R); the crossing figure is from a stale report |
| A7 | Tag->data serialisation costs +2 cycles | +2 | §3.3, §6 | Pipeline-depth arithmetic; could be +1 with a way predictor, or +3 with a register on the column crossing |
| A8 | Congestion is still level 5/6 | Aug 10 report | §2.1, §7.1 | The current build is 10 points better on LUT, so this is an **upper** bound. A fresh `report_design_analysis -congestion` on `route.dcp` is free |
| A9 | The routed `route.dcp` matches the Aug 20 report | assumed | M1 | Check the timestamp before trusting M1's output |
