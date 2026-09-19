# SoC clocking study — what the fabric, the L2 and the CPU should run at

Status: 2026-08-19.  **Analysis and costed proposals only.  No RTL was
changed, no Vivado was run, no board was touched.**

Every number below is either (a) read out of an existing report in
`build/vivado/` or `synth/timing_reports/`, (b) quoted from
`docs/l2c_perf.md` / `docs/soc_bus_review.md` / `docs/dma_engine_design.md`,
which measured it, or (c) derived arithmetic from (a) and (b), labelled
**ESTIMATE**.  Nothing here is a new post-route measurement, because the
constraint on this work was explicitly "no Vivado".

Companion documents, read them rather than trusting summaries here:

- `docs/l2c_perf.md` — measured L2 throughput and post-route area
- `docs/soc_bus_review.md` — the F1-F9 footgun inventory and CDC costs
- `docs/dma_engine_design.md` — bandwidth ceilings and the DMA plan
- `docs/clocking.md` — the original domain plan (parts of it are stale;
  §2 below says which)

---

## 0. The decision this document serves

The direction is already set, and this document costs it rather than
re-opens it:

> *"the new core aims for 200 MHz, but we will probably feed it 100 MHz at
> first; the fabric should be ready for faster core and running at ddr clk
> gives headroom for dma and vram activity"*
>
> *"concurrency first though i agree, clock later. and maybe we can have
> the faster clock also provide cheaper deeper concurrency via time
> multiplex"*

So: **concurrency work lands first; re-clocking follows as a migration.**
The job of this document is to say what the fabric should become, what
concurrency work now makes that a migration rather than a rewrite, and
what each step costs.

The analysis endorses the direction and changes three things about it.
Those three are stated up front in §1 because they change what gets built.

---

## 1. Executive summary

### 1.1 The direction is right, and the reason is not the one in the brief

The premise handed to this study was that the 333 -> 100 MHz crossing is
expensive and is throttling DDR.  **It is not.**  `u_core_to_mig_ui` costs
**3.0 core cycles round trip** (measured, `docs/soc_bus_review.md` §5) and
**717 LUT** (`build/vivado/reports/utilization_route.rpt:117`).  The
post-route inter-clock table is **entirely positive** — worst crossing
slack **+2.065 ns** — so no CDC bridge is on the critical path.  All four
CDC bridges together are **2,500 LUT, 1.34% of the design**.  The "57% of a
peripheral read is CDC" figure belongs to a *different* bridge,
`u_pb_s1_cdc` (core -> pb), and to a path that never touches DDR.

Removing the DDR-side CDC therefore saves ~0.4% of the LUTs and ~3 cycles
out of a 72-80 cycle miss.  **That is not the argument for re-clocking.**

The real argument is **bandwidth headroom**, and it is strong:

| | Ceiling | Source |
|---|---:|---|
| DRAM (DDR4-2666, x32) | 10.67 GB/s | `pll_clk[0]` = 2666.666 MHz, timing report `:161` |
| MIG UI (256 b @ 333.333 MHz) | 10.67 GB/s | timing report `:160` |
| **Fabric (128 b @ 100 MHz)** | **1.6 GB/s** | `docs/dma_engine_design.md:128-129` |

The fabric can absorb **15% of what the DRAM can deliver.**  And the demand
side is about to exceed the fabric, not the DRAM:

| Consumer | Demand | Source |
|---|---:|---|
| Scanout, 1024x768x24bpp | 189 MB/s | `rtl/soc/scanout_ddr_reader.v:70-75` |
| L2 fills at MSHR saturation (8.38 cyc/op, 64 B/fill) | ~764 MB/s | derived from `docs/l2c_perf.md` §1.1 |
| DMA engine (planned) | 270-800 MB/s | `docs/dma_engine_design.md:324, 578-579` |
| **Total** | **1.2-1.75 GB/s** | vs a **1.6 GB/s** fabric ceiling |

**Once the concurrency work lands and the DMA engine exists, the 100 MHz /
128-bit fabric ceiling is the binding constraint.**  That is the
justification, and note its shape: it only becomes true *after* the
concurrency work.  Which is exactly why "concurrency first, clock later" is
the correct order and not merely a conservative one.

### 1.2 Three corrections to the plan as briefed

**(a) The fabric's Fmax has never been measured, and the design's timing
failure is not the fabric's fault.**  All 50 worst paths in the current
post-route report are inside `u_cpu` — `u_iq_fp/e_valid_reg[5]` ->
`u_fpu/u_div/rem_reg[*]`, 36-42 logic levels, 64-77% route delay
(`build/vivado/timing_summary.rpt:6334-6892`).  **Zero** violating paths are
in `l2c`, the crossbar, any CDC bridge, the DDR bridge or the video path.
The design is not "barely closing 100 MHz" because of the fabric; it is
barely closing because of the FPU divider in the core that is being
replaced.  We therefore do not know what the fabric would close at.  §11
makes measuring it the top recommendation.

**(b) 64-bit-at-333 is worse than 128-bit-at-166, at identical bandwidth.**
Both deliver 2.67 GB/s.  The narrow/fast option needs a **3.3x** critical-path
reduction instead of **1.67x**, doubles every burst's beat count on a front
door that is *beat-granular* (`docs/soc_bus_review.md` F2) — so it doubles
L2 front-door occupancy per burst — and does **not** shrink the buffers,
because FIFO storage is set by bytes-in-flight, not by width.  §6 works this
through.  Narrowing is a real option, but only *after* F2 is fixed, and it
is not the cheap half of the trade.

**(c) The time-multiplex idea is right and is worth ~5-9k LUT, but it is
not gated on the clock.**  `u_mshr`'s remaining muxing is flat dynamic reads
of a 4,096-bit flip-flop array; replacing that array with a 32 x 128 memory
is available at **today's** clock, because fill traffic already arrives one
beat at a time.  What the faster clock buys is that the added serialisation
becomes invisible in CPU cycles.  §7 gives the arithmetic, and corrects a
3x error in the framing (one 3-cycle pipeline at 3.33x serves ~1.1 ops per
CPU cycle, not ~3).

### 1.3 The recommendation, in one paragraph

Keep the CPU and the whole of `l2c` in the CPU clock domain.  Move the
**DDR-facing, CPU-independent** traffic — `axi_vram_priority_mux3`,
`scanout_ddr_reader`, `vhdd_ddr`, and the future DMA engine — to the MIG UI
domain, which relocates the existing `u_core_to_mig_ui` bridge up exactly
one hop (from below the VRAM lane mux to above it).  That is a migration,
not a rewrite: the boundary already exists, it just sits one hop too low.
Do the concurrency work (F1-F4) first, because until it lands the fabric is
running at 5-33% of the clock it already has.  Target **fabric = MIG UI / 2
= 166.667 MHz** as the first re-clock, not 333.333 — it is an *integer* ratio
to the MIG, it is 1.67x today, and `mmcm_clkout6` already closes at that
exact frequency with **+1.571 ns** of margin on this device.  And if the MIG
is re-generated at DDR4-2400 (UI = 300 MHz), the entire clock tree becomes
integer-related to a 100 or 150 MHz CPU, which removes async CDC from the
hot path altogether — at the cost of 10% of a DRAM peak we use 15% of.

---

## 2. The clock map as built

Sources: `build/vivado/timing_summary.rpt:153-172` (Clock Summary, the
authoritative table), `build/ddr4_mig/design_1_ddr4_0_1.xci`,
`synth/gen_ddr4_mig.tcl`, `rtl/soc/fpga_top_clocks.vh`, `synth/vivado.tcl`.

### 2.1 Domains

| Clock | Freq | Period | Source | Endpoints | Intra-clock WNS |
|---|---:|---:|---|---:|---:|
| `fabric_clk100` (`core_clk`) | 100.000 | 10.000 | `fabric_clk_p` AB7 GTY refclk -> ODIV2 -> BUFG_GT DIV=0 | 294,371 | **-0.612** |
| `pb_clk` | 50.000 | 20.000 | **same** ODIV2 -> BUFG_GT DIV=1 | 37,345 | +8.120 |
| `pclk_unbuf` | 148.500 | 6.734 | video MMCM off the fabric clock | 6,464 | +0.121 |
| `sys_clk_p` / `sysclk200` | 200.000 | 5.000 | `sys_clk_p` T24, dedicated DDR ref pin | 23 | +3.655 |
| `mmcm_clkout0` (**MIG UI**) | **333.333** | **3.000** | MIG MMCM off `sys_clk_p`: M=5, D=1, CLKOUT0_DIVIDE=3 (VCO 1000 MHz) | 30,927 | **0.000** |
| `mmcm_clkout6` (MIG PHY) | **166.667** | 6.000 | same MMCM, CLKOUT6 (VCO/6) | 5,336 | **+1.571** |
| `pll_clk[0..1]` | 2666.666 | 0.375 | MIG PHY PLL — the DDR4 data-rate clock | — | (pulse-width only) |
| `INTERNAL_TCK` | 20.000 | 50.000 | BSCANE2 / dbg_hub | 1,002 | +12.299 |

Four things in that table matter and are not written down anywhere else:

1. **The MIG UI is 333.333 MHz, not 333.25, and the DRAM is DDR4-2666,
   not DDR4-2400.**  `pll_clk[0]` at 2666.666 MHz is the data-rate clock;
   the part is `MT40A512M16LY-075` (0.75 ns tCK).  `rtl/board/ddr_ctrl.v:42`
   ("DDR4-2400, 1200 MHz memory clock") is **wrong**, and the derived
   "~9.6 GB/s" in `docs/dma_engine_design.md:125` and
   `docs/soc_bus_review.md:549` is wrong with it.  True DRAM peak is
   **10.67 GB/s**.  333.25 MHz is an IP metadata float on the
   `C0_DDR4_CLOCK` bus interface (`.xci:436,537`); STA times the real clock
   at exactly 3.000 ns.  `docs/clocking.md:37` says 300 MHz, which is
   stale in a third direction.
2. **`mmcm_clkout6` is 166.667 MHz and closes with +1.571 ns of margin
   (26% of its period) on this device, in this build, with 5,336
   endpoints.**  That is the single most useful datapoint for the
   feasibility question in §10.
3. **The MIG UI clock closes at exactly 0.000 ns** across 30,927
   endpoints.  333 MHz is demonstrably achievable on a `-2` KU5P — for
   hand-floorplanned vendor IP, at literally zero margin.
4. **`core_clk` and `pb_clk` are the same clock divided by two**, from one
   BUFG_GT source, and Vivado times them synchronously
   (`synth/vivado.tcl:1530`, "real, related fabric clocks and meet timing
   fine when checked synchronously").  Their crossing slack is +2.065 ns.
   The 771-LUT async FIFO between them is therefore already redundant —
   `docs/soc_bus_review.md` Phase 3b, not re-argued here.

### 2.2 Crossings, their cost, and their area

`axi_async_bridge` is five independent gray-pointer `async_fifo`s, one per
AXI channel.  It is **not** a handshake, so a burst pays the crossing once,
not per beat (`docs/soc_bus_review.md` §5, verified against an 8-beat
burst).  Latency is **3 destination-clock edges** per channel, not 2,
because `async_fifo.v:656-662` registers `empty` on top of the two
`ASYNC_REG` stages.  AW pays a 4th.

| Instance | Module | Domains | Round trip | LUT | FF | Report line |
|---|---|---|---:|---:|---:|---|
| `u_ddr/u_core_to_mig_ui` | `axi_async_bridge_24` | core -> mig_ui | **3.0 core cyc** | **717** | 420 | `utilization_route.rpt:117` |
| `u_pb_s1_cdc` | `axi_async_bridge` | core -> pb | **8.0 core cyc** | **771** | 409 | `:258` |
| `u_vhdd_ddr_cdc` | `axi_async_bridge__param1` | pb -> core (RAM disk) | — | **672** | 402 | `:282` |
| `u_dbg_pb_to_core` | `axil_async_bridge` | pb -> core | 4.0 pb cyc | **340** | 357 | `:104` |
| `u_pram_cdc` | 4-phase handshake | core <-> pb | **16 core cyc / BYTE** | — | — | JTAG only |
| `u_video/u_fb_reader/u_{req,rsp}_fifo` | `fb_reader_cdc_fifo` | core -> pclk | — | 33 / 282 | 80 / 80 | `:296-297` |
| `u_scanout_placement_sync` | 7x 2-FF + agree filter | core -> pclk | — | 617 | 377 | `:302` |
| `dafb_vbl_cdc` | `pulse_cdc` (2-FF) | pb <-> pclk | — | small | — | `vivado.tcl:1520-1543` |
| MIG cal-done -> `pb_clk` | 2-FF meta sync | mig_ui -> pb | — | small | — | `vivado.tcl:1571-1580` |

**`u_vhdd_ddr_cdc` is present in the shipped netlist at 672 LUT**, contrary
to the "now compiled out" note in the brief — it appears in the post-route
hierarchy report.

### 2.3 The first result: CDC is not a cost centre

- **Area.** All four AXI CDC bridges = **2,500 LUT = 1.34%** of the design's
  187,011.  The DDR-side one specifically is **717 LUT = 0.38%**.
- **Timing.** The post-route **inter-clock table is entirely positive**
  (`timing_summary.rpt:209-230`).  Worst crossing is `fabric_clk100 ->
  pb_clk` at **+2.065 ns** over 21,784 endpoints.  **No CDC bridge appears
  on any of the 50 worst paths.**
- **Latency, DDR path.** 3.0 core cycles against a 72-80 cycle miss
  (`docs/soc_bus_review.md` §3a) = **4%**.

The 57%-of-a-read figure in the brief is real but belongs to `u_pb_s1_cdc`
on the *peripheral* path (`docs/soc_bus_review.md` §3c), which never
touches DDR and which is over-provisioned for an unrelated reason: it is a
fully asynchronous 128-bit FIFO across a **synchronous 2:1 boundary**, to
reach an **8-bit** register.

**Conclusion for §1.1: "remove the 333->100 CDC" is not a saving worth
designing for.  Re-clock for bandwidth headroom, or do not re-clock.**

---

## 3. Where the bandwidth actually goes — Little's Law

This is the frame that makes every subsequent trade decidable.

For a memory system, sustained bandwidth is set by concurrency, not by
clock:

```
    throughput  =  outstanding_transactions  x  bytes_per_transaction
                   -------------------------------------------------
                                 round_trip_latency
```

and it is capped by the fabric's raw carrying capacity:

```
    fabric_ceiling  =  bus_width_bytes  x  fabric_clock
```

For the DDR fill path: `bytes_per_transaction` = **64 B** (the L2 line;
`l2c_mshr.v` issues `arlen=3, arsize=4`), `bus_width_bytes` = **16**.

### 3.1 The DDR latency assumption, stated prominently

**There is no measured real-hardware DDR round-trip latency anywhere in
this repository, and every margin claim in every document depends on one.**
`rtl/soc/scanout_ddr_reader.v:77-80` says so explicitly: *"the ~40-cycle
DDR round trip is an ASSUMPTION inherited from the original T14 writeup,
not a measurement, and every number above scales with it."*

This study uses **L = 40 +/- 8 core cycles at 100 MHz = 400 +/- 80 ns**, as
instructed, and reports the sensitivity everywhere it matters.

**And the assumption is probably pessimistic by 3-4x.**  A DDR4-2666 part
at CL19 has ~14.25 ns of CAS, plus tRCD, plus the MIG UI pipeline (order
20-30 UI cycles = 60-90 ns), plus our 128->256 bridge.  A realistic total
is **100-150 ns**, i.e. **10-15 core cycles**, not 40.

**The direction of that error is the important part**, and it is
counter-intuitive: *a shorter DDR latency makes the fabric clock MORE
urgent, not less.*  Each outstanding transaction completes sooner, so it
demands more bandwidth, so the fabric ceiling binds at fewer outstanding
transactions.  §3.3 quantifies it.

### 3.2 The crossover: at what concurrency does the fabric clock start to bind?

Set demand equal to ceiling and solve for outstanding transactions `n`:

```
    n_crit  =  bus_width_bytes x f_fabric x L / 64 B
            =  f_fabric x L / 4
```

| f_fabric | L = 400 ns (assumed) | L = 200 ns | L = 120 ns (realistic ESTIMATE) |
|---|---:|---:|---:|
| **100 MHz** (today) | **10** | 5 | **3** |
| 166.667 MHz | 16.7 | 8.3 | 5 |
| 333.333 MHz | 33.3 | 16.7 | 10 |

**Today the fabric permits n = 1** (`axi_xbar.v:3208`, F1) **and the VRAM
lane mux permits n = 2** (`axi_vram_priority_mux3.v:100`,
`MAX_BULK_AHEAD = 2`, F3).  Against a crossover of 3-10.

So at today's concurrency, **raising the fabric clock buys nothing on the
DDR path** — we are a factor of 1.5 to 10 below the point at which the
100 MHz clock is what limits us.  This is the quantitative form of
"concurrency first, clock later", and it agrees with the sequencing the
user set.

### 3.3 The same arithmetic, run forward: when the clock DOES bind

The MSHR has **8 entries** (`l2c_mshr.v:42`), costing **20,953 LUT**
(`utilization_route.rpt:169`) — 11% of the entire design, twice the whole
MIG DDR4 controller.  `docs/l2c_perf.md` §4.2 makes the case that this
depth stops being dead capability the moment `m68k-core-040-ooo` lands.

Ask what it takes to *feed* 8 outstanding fills:

| L | Demand at n=8 (8 x 64 B / L) | vs 100 MHz ceiling (1.6 GB/s) | vs 166.667 (2.67) | vs 333.333 (5.33) |
|---|---:|---|---|---|
| 400 ns | 1.28 GB/s | 80% — fits, barely | 48% | 24% |
| 200 ns | 2.56 GB/s | **160% — does not fit** | 96% | 48% |
| 120 ns | 4.27 GB/s | **267% — does not fit** | **160% — does not fit** | 80% |

**This is the strongest single argument for re-clocking, and it is an
argument about area already spent.**  If the true DDR round trip is
anywhere below ~250 ns, a 100 MHz / 128-bit fabric **cannot carry an
8-entry MSHR's worth of concurrent fills**, and roughly 60% of that
20,953 LUT is unreachable for a *fabric bandwidth* reason — on top of the
concurrency-cap reason `docs/l2c_perf.md` already documents.  Neither
existing document makes this point, because neither multiplied the MSHR
depth by the fill size and compared it to the bus.

Re-clocking to 166.667 MHz makes n=8 reachable down to L ~ 190 ns.
Re-clocking to 333.333 makes it reachable down to L ~ 95 ns.

### 3.4 Aggregate demand, and why the DMA engine is the trigger

| Consumer | Demand | Source |
|---|---:|---|
| Scanout, 1024x768x24bpp | **189 MB/s** (71% of its lane's ~267 MB/s) | `scanout_ddr_reader.v:70-75` |
| L2 fills at measured MSHR saturation (8.38 cyc/op, 64 B/fill, L=400 ns) | **~764 MB/s** | derived from `docs/l2c_perf.md` §1.1 |
| DMA engine (planned) | **270-800 MB/s** | `docs/dma_engine_design.md:324, 578-579` |
| CPU peripheral + boot + debug | tens of MB/s | — |
| **Total** | **1.2 - 1.75 GB/s** | **vs 1.6 GB/s fabric ceiling** |

The scanout figure is the one that should worry us: **71% of its lane's
capability at 1024x768x24bpp**, on a machine whose whole point is running
Mac OS at a usable resolution.  The user's stated principle for scanout was
that its cost should be *"sporadic and hidden as much as possible"*.  At
71% lane occupancy it is neither.  A 2x fabric clock takes that to 36%; a
3.33x clock to 21%.

**This is the clean statement of the case: the fabric clock is not needed
to make the CPU faster.  It is needed so that DMA and scanout stop
competing with the CPU for fabric slots.**

---

## 4. Frequency vs concurrency — testing the hypothesis, and refuting it

The hypothesis in the brief was:

> *"on the DDR-facing side ... and on THROUGHPUT rather than hit latency —
> a CDC round trip would eat the gain on hits, but L1 absorbs most hits, so
> what matters is how fast L2 retires concurrent misses."*

**The measurements say this is exactly inverted.**

### 4.1 The model, fitted to measured data

`docs/l2c_perf.md` §1.1 and §1.2 measured the same scenarios at two DDR
latencies.  Fit the 8-outstanding miss row:

```
    L =  40 cyc  ->   8.38 cyc/op
    L = 200 cyc  ->  28.38 cyc/op
    =>  cyc/op = L/8 + 3.38          (exact, both points)
```

The fit is exact to two decimal places at both latencies, which means the
8-entry MSHR's concurrency works *perfectly* and the miss path is **purely
latency-bound**.  The hit rows are **3.00 cyc/op at both latencies** —
purely **pipeline-bound**, and `docs/l2c_perf.md` §1.3 decomposes it as
array read latency (2) + resolve (1).

### 4.2 What frequency does to each

DDR round-trip latency is **physical time**.  Doubling the fabric clock
doubles it *when counted in fabric cycles*.  So:

| Path | Cost model | 100 MHz | 200 MHz | 333.333 MHz | Speedup at 3.33x clock |
|---|---|---:|---:|---:|---:|
| **L2 hit** | 3 cyc, invariant to L | 30.0 ns | 15.0 ns | 9.0 ns | **3.33x — linear** |
| **L2 miss** (n=8, L=400 ns) | L_cyc/8 + 3.38 | 83.8 ns | 66.9 ns | 60.2 ns | **1.39x — sub-linear** |

**Frequency helps the hit path linearly and the miss path barely at all**,
because 5/8 of a miss is physical DRAM time that no fabric clock changes.
The hypothesis had it backwards: the traffic frequency helps *least* is
precisely the concurrent-miss traffic it was expected to help most.

And the corollary is uncomfortable for the "L1 absorbs hits" premise: if
L1 really does absorb most hits, then most of what reaches L2 is misses —
which is the traffic frequency does least for.  **Frequency is the wrong
lever for L2 latency.**  (It remains the right lever for L2 *aggregate
carrying capacity*, which is §3's argument and a different quantity.)

### 4.3 What concurrency does, at zero timing cost

At fixed 100 MHz, from the same fitted model, L = 400 ns:

| Outstanding | cyc/op | ns/op | vs n=1 |
|---:|---:|---:|---:|
| 1 | 54.20 (measured) | 542 | 1.0x |
| 2 | 28.25 (measured) | 283 | 1.9x |
| 4 | 14.88 (measured) | 149 | 3.6x |
| 8 | 8.38 (measured) | 83.8 | **6.5x** |
| 16 | 8.03 (measured) | 80.3 | 6.7x |

**Concurrency delivers 6.5x where frequency delivers 1.39x, and it costs
nothing in timing closure.**  Note also that measurement saturates between
8 and 16 (8.38 -> 8.03, where the model predicts 5.88): something other
than MSHR depth binds at n>8 — most likely the fill data-return occupancy
(4 beats per 64 B line at 1 beat/cycle) or the 1-AR/cycle issue rate at
`l2c_mshr.v:189`.  That saturation point is itself a fabric-bandwidth
symptom, and it is exactly what a faster fabric clock moves.

### 4.4 The order this settles

**Concurrency first, unambiguously, and by a factor of ~5.**  The four
in-series limits (`docs/soc_bus_review.md` F1/F2/F3/F4, restated in
`docs/l2c_perf.md` §4.4) must land as one change set; landing any subset
measures as zero.  Only after they land does the fabric run near its own
clock's capability, and only then does raising that clock buy anything.

Re-clocking is **not** an alternative route to the same win.  It is the
step *after*, and its payoff is the §3.4 headroom argument, not the miss
path.

---

## 5. The clock family — and the ratio result that reshapes the plan

This is the section that changes what should be built.

### 5.1 The MIG UI rate is not a free parameter, but its divisors are

`build/ddr4_mig/design_1_ddr4_0_1.xci`:  M=5, D=1, CLKOUT0_DIVIDE=3 off a
200 MHz `sys_clk_p` -> **VCO 1000 MHz, UI = 333.333 MHz**, PHY ratio 4:1,
memory clock 1333.33 MHz, DDR4-2666.

The rate is pinned in four places, three of which hard-fail if changed:
`synth/gen_ddr4_mig.tcl:223-360` (params), `:368-386` (`required_checks`
assertions), `:153` (manifest literal `333250000`), and
`tools/check_ddr4_pcie_test.py:108` (equality check against the external
known-good `pcie_test` project, run from `Makefile:2680,2690`).  The RTL
port lists in `rtl/board/ddr_ctrl.v:140-176` and the vendor stub are
hard-wired to 256/31/1.  **Treat the UI rate as fixed at 333.333 MHz.**

But 333.333 MHz has divisors, and `BUFGCE_DIV` (the same primitive family
already used for `core_clk`/`pb_clk`) divides by 1-8 with a fixed,
phase-aligned relationship:

```
    333.333 / 1 = 333.333        333.333 / 2 = 166.667
    333.333 / 3 = 111.111        333.333 / 4 =  83.333
```

`mmcm_clkout6` = **166.667 MHz already exists in this design** as the MIG
PHY clock, and closes with **+1.571 ns** of margin.

### 5.2 The integer-ratio result: rational is not good enough

The brief asked whether an integer ratio could replace async CDC with a
cheap clock-enable crossing.  It can — but the qualifying condition is
stricter than "rational", and this kills the obvious idea.

For two synchronous clocks, the worst-case setup window is the smallest
positive gap between a launch edge of the source and a capture edge of the
destination:

| Pair | Ratio | Periods | Worst-case setup window | Verdict |
|---|---|---|---:|---|
| 333.333 : 166.667 | **2:1** | 3 / 6 ns | **3.000 ns** (full fast period) | clock-enable / elastic buffer works |
| 333.333 : 111.111 | **3:1** | 3 / 9 ns | **3.000 ns** | works |
| 333.333 : 83.333 | **4:1** | 3 / 12 ns | **3.000 ns** | works |
| 200 : 100 | **2:1** | 5 / 10 ns | 5.000 ns | works (this is today's core/pb) |
| **333.333 : 200** | 5:3 | 3 / 5 ns | **1.000 ns** (edges at 9 -> 10) | **unusable** |
| **333.333 : 100** | 10:3 | 3 / 10 ns | **1.000 ns** (edges at 9 -> 10) | **unusable** |

**So making the MIG UI clock "synchronous" to a 200 MHz or 100 MHz CPU is
worse than useless.**  A 5:3 or 10:3 relationship produces a 1.000 ns
worst-case launch-to-capture window on a device where the current design's
*best* paths are ~9.4 ns.  Nothing routes through that.  You would be
forced back to a full elastic buffer with registers on both sides — which
is what `axi_async_bridge` already is.

This is a genuinely useful negative result: **the only ratios worth making
synchronous are integer ones, and 100 and 200 MHz are not integer divisors
of 333.333.**  Any plan that keeps the CPU at 100 or 200 keeps an async
CDC on the DDR path, no matter how the clocks are sourced.

### 5.3 What that implies: the CPU clock should be a UI divisor

If the CPU/fabric clock is taken as `UI / N` via `BUFGCE_DIV`, every
crossing in the machine becomes an integer clock-enable, and the async
bridges leave the hot path entirely.  The available CPU rates are:

| CPU rate | = UI / | vs today's 100 MHz | vs the 200 MHz core goal |
|---:|---:|---|---|
| **111.111** | 3 | **+11%** | 56% |
| **166.667** | 2 | **+67%** | 83% |
| **333.333** | 1 | +233% | 167% |
| 83.333 | 4 | -17% | 42% |

**111.111 MHz is strictly better than today's 100 MHz** and turns the
DDR-path async bridge into a 3:1 clock enable.  **166.667 MHz is 83% of the
core's own 200 MHz goal** — a core that closes 200 closes 166.667 — and
turns it into a 2:1 clock enable.

The 200 MHz target in `CLAUDE.md` is a round number chosen as a design
goal, not a requirement imposed by anything external.  **Retargeting the
core to 166.667 MHz costs 17% of a goal that has never been met (the CPU
is at -0.612 ns against 100 MHz today) and buys a clock tree with no
asynchronous crossing anywhere on the memory path.**  That is a trade this
project should take.

### 5.4 The end-state proposal

```
                    sys_clk_p 200 MHz (T24, dedicated DDR ref pin)
                            |
                       MIG MMCM (M=5 D=1, VCO 1000 MHz)
                            |
                  CLKOUT0 = 333.333 MHz  ==  MIG UI  ==  FABRIC
                            |
                       BUFGCE_DIV /2
                            |
                     166.667 MHz  ==  CPU + L2
                            |
                       BUFGCE_DIV /4  (or /2 from CPU clk)
                            |
                      41.667 MHz  ==  pb island
```

| Boundary | Ratio | Mechanism | Replaces |
|---|---|---|---|
| MIG UI <-> fabric | **1:1** | none | `u_core_to_mig_ui` (717 LUT) |
| fabric <-> CPU/L2 | **2:1** | clock-enable / 2-deep elastic buffer | — (new boundary) |
| CPU <-> pb | **4:1** | clock-enable | `u_pb_s1_cdc` (771 LUT) |
| GT/JTAG debug island | async | keep `u_dbg_pb_to_core` (340 LUT) | — |

**One async crossing survives, deliberately: the JTAG/debug island.**  It
must stay on the independent GT-derived clock, because a clock tree rooted
in the MIG MMCM dies when the MIG resets or fails to calibrate — and
debugging a MIG that failed to calibrate is exactly when you need JTAG.
That crossing is 340 LUT, is off the hot path, and already exists.

`pb_clk` moving from 50 to 41.667 MHz is a non-event: `fpga_top` already
derives `phi2_tick` from an explicit `PB_CLK_HZ` generic through an NCO
(`docs/clocking.md` §3 item 3), so the Q700 783.36 kHz VIA timebase is
preserved by construction.  `PB_CLK_HZ` is already a Makefile variable
(`Makefile:2228`).

### 5.5 The constraint that independently rules out going slower

`async_fifo.v:472-487` has `REQ_MIN_HOLD = 8`, which bounds the usable
clock ratio to about **1:4** (`docs/soc_bus_review.md` §5).  `core_clk :
mig_ui_clk` is 1:3.33 today — inside the bound with ~20% margin.

**Dropping `core_clk` to 50 MHz — the plausible reflex if the new core
fails timing — makes it 1:6.7 and breaks the reset handshake.**  There is
already a `make fpga-50mhz` path (`Makefile:2361-2384`) that sets
`CORE_CLK_HZ=50_000_000`.  On a real-MIG build that path is unsafe, and
nothing asserts it.  Under the §5.4 proposal the problem disappears
entirely, because the boundary stops being an async FIFO.

---

## 6. Width vs frequency — the 64-bit-at-333 proposal, costed

The proposition: `128 b @ 100 MHz = 1.6 GB/s` versus `64 b @ 333 MHz =
2.7 GB/s`, so a narrower/faster fabric could be smaller *and* faster at
once.  The bandwidth arithmetic is correct.  **The area arithmetic is not,
and the interaction with the front door is actively harmful.**

### 6.1 What halving the bus actually recovers

Post-route, per module (`utilization_route.rpt`; ESTIMATE column is the
width-proportional share judged from the RTL structure):

| Module | Post-route | Width-proportional share | ESTIMATE recovered at 64 b |
|---|---|---|---:|
| `axi_xbar` | 4,779 LUT / 994 FF | ~40-50% of LUT, **0% of FF** (pure comb mux fabric, no data registers) | **~1,000-1,200 LUT** |
| `axi_narrow_to_wide` x3 | 780 LUT / 1,323 FF | ~60% of FF (4 gather lanes -> 2) | ~500 FF, ~150 LUT |
| `axi_async_bridge` x2 live | 1,389 LUT | W+R FIFOs only (~45%) | **~200 LUT, or ZERO — see 6.2** |
| `axi_vram_priority_mux3` | 184 LUT / 25 FF | ~80% of LUT (one 2:1 over 144 b) | ~72 LUT |
| `l2c_bypass` | 213 LUT / 320 FF | ~85% of FF | ~140 FF |
| `axi_wide_to_axilite` x3 | 405 LUT | ~15% | ~60 LUT |
| `peripheral_bus` | 1,168 LUT / 291 FF | **~4%** (holds no 128-bit register at all) | ~20 LUT |
| `l2c_mshr` | 20,953 LUT / 10,168 FF | ~5,900 of 10,168 FF are fabric-width | ~2,900 FF; **LUT saving unclear** |
| **Total** | **187,011 LUT** | | **~1,500-1,700 LUT (0.8%), ~3,600 FF (2.8%)** |

**Under 1% of the LUTs.**  FF is at 27% utilisation and is not the
constrained resource; LUT at 86.2% is.

The reason the saving is so small is structural and worth stating: **most
of the "128-bit" area in this design is not fabric-width at all.**

- `l2c_data` is **64 URAM = 100% of the device**, and it is sized by the
  **64 B cache line**, not the bus.  It has no AXI port and no width
  parameter (`l2c_data.v`, 73 lines: 8 ways x 4096 sets x 512 b, each way
  = 8 URAM288 wide x 1 deep).  A 64-bit fabric changes it by **zero bits**.
- `l2c_mshr`'s dominant cost — the ~59% that `docs/l2c_perf.md` §3 traced
  to `m_line`'s per-bit write mux — is over a **4,096-bit line array**,
  which is line-width.  Not recoverable by narrowing the bus.
- `l2c_victim`'s 1,024 FF (`v_data0`/`v_data1`) are line-width, and its
  read mux goes from 4:1-over-128 b to 8:1-over-64 b — **identical LUT
  count**.  This module is the clean example: halving the bus saves
  literally nothing.
- `l2c_ctrl`'s expensive mux is the 8:1-over-512 b way select.  Line-width.
  The fabric-width mux was *already* engineered out (`l2c_ctrl.v:270-274`
  records replacing a 128 b 32:1 mux, saving ~600 LUT).
- `axi_ddr4_mig_bridge`'s wide storage (`wrout_data`, `rdin_data`,
  `rd_cur_data` — 1,280 bits) is sized by the **MIG's 256-bit** side.
  Unchanged.

### 6.2 Three things that get worse

**(a) Buffer storage does not shrink — it is set by bytes-in-flight.**
`axi_async_bridge` W/R FIFOs are 16 entries deep (`:91-95`), i.e. 16 *beats*
= 256 B today.  At 64 b, 16 beats is 128 B.  To hold the same byte-depth you
need `W_DEPTH_LOG2 = 5`, and 32 x 82 b = 2,624 LUTRAM bits against today's
16 x 146 b = 2,336.  **Slightly worse.**  The ~200 LUT saving above only
exists if you accept half the buffered bytes — on a path where a `vhdd_ddr`
block is already 32 beats and would become 64.

**(b) It doubles L2 front-door occupancy per burst.**  The L2 front door is
**beat-granular**, not transaction-granular (`docs/soc_bus_review.md` F2;
`l2c_ctrl.v` `ar_beat <= ar_beat+1`).  It charges ~3 cycles *per beat*.
Halving the width doubles every burst's beat count, so **a 64 B line fill
goes from 4 front-door beats to 8**, and `docs/soc_bus_review.md:74`
measured that bursting already "buys almost nothing" here (3.54 cyc/beat
for a 4-beat burst vs 3.09 for pipelined singles).  A 3.33x clock does not
cleanly cover a 2x beat-count increase on a path that is latency-bound
rather than bandwidth-bound.

**Narrowing the fabric before F2 lands makes throughput worse, not
better.**  That ordering constraint is absolute.

**(c) Instruction fetch genuinely needs 128 bits atomically.**
`if_to_axi.v:1-8` and `cpu/rtl/mac_top.v:9`: the I-side is a "simple
req/valid, **128-bit line**" delivered whole in one cycle to
`if_stage`/`predecode`, with a four-word big-endian reversal at
`if_to_axi.v:142-145`.  `arlen=0, arsize=4`.  At 64 b you must add a
holding register and a 2-beat FSM — or shrink the I-cache line, which is a
core change.  `cpu_socket.vh:71-72` reserves `{128, 256}` for
`CPU_SOCKET_AXI_DW`; **64 is not in the socket contract.**

### 6.3 The migration cost

Modules with a *hardcoded* 128 that must be rewritten, not re-parameterised:

`axi_ddr4_mig_bridge.v` (**1,654 lines, zero width parameters**, and it
carries the MAX_FANOUT/timing fixes — the riskiest file in the repo to
touch), `l2c_mshr.v` (~25 sites, plus `m_beat` 2->3 bits and every
`*128 +: 128` quadrant slice, plus `L2C_QUAD_BITS` 2->3), `l2c_ctrl.v`
(~12), `l2c_victim.v` (4), `l2c_bypass.v` (6), `l2c.v` internal wires (5 —
`DATA_WIDTH` is decorative below the top level), `axi_narrow_to_wide.v`
(4-lane structure, x2 file copies), `if_to_axi.v` (`DATA_WIDTH` declared
and ignored), `axi_wide_to_axilite.v` (4-way case arms), `peripheral_bus.v`,
`scanout_line_fetch.v`, `vhdd_ddr.v` (fully hardcoded),
`axi_pb_lane_shim.v` (assumes a 4:1 lane ratio).

Cleanly parameterised, one-line change: `axi_xbar.v`, `axi_async_bridge.v`,
`async_fifo.v`, `axi_vram_priority_mux3.v`, `ddr_ctrl.v` (HW branch).

### 6.4 Verdict

**At equal bandwidth, 128-bit-at-166.667 dominates 64-bit-at-333.333 on
every axis except a ~1% LUT saving:**

| | 64 b @ 333.333 | 128 b @ 166.667 |
|---|---|---|
| Bandwidth | 2.67 GB/s | 2.67 GB/s |
| Critical-path reduction required | **3.33x** | **1.67x** |
| MIG ratio | 1:1 (no crossing) | **2:1 integer** (clock-enable) |
| L2 front-door beats per 64 B fill | **8** | 4 |
| I-fetch | needs a new 2-beat FSM | unchanged |
| Buffer storage | unchanged or worse | unchanged |
| Area | **-1,500 to -1,700 LUT (0.8%)** | 0 |
| Files needing rewrite | **13+, incl. a 1,654-line unparameterised bridge** | 0 |

**Recommendation: do not narrow the fabric.**  If width is ever revisited,
revisit it *upward* — `cpu_socket.vh` already reserves 256, and a 256-bit
fabric at the MIG UI clock would delete `axi_ddr4_mig_bridge`'s width
conversion entirely.  That is the direction with a real prize in it; 64-bit
is not.

One narrowing that **is** right and is already done: the S1 peripheral path.
`axi_pb_s1_cdc.v:19-21` carries 32 payload bits instead of 128 and removes
"~73% of this bridge's FIFO storage".  **Narrow the paths that reach 8-bit
registers; keep the memory path wide.**  That is the correct rule, and it
is the opposite of a global narrowing.

---

## 7. Time-multiplexing — concurrency in time instead of area

The idea: with a fabric:CPU ratio of R, one physical structure iterated R
times per CPU cycle presents as R-way concurrency to the core without R
copies of the hardware.  **The idea is right and is worth ~5-9k LUT.  Two
things about how it was framed need correcting, and both make it a better
proposal, not a worse one.**

### 7.1 Correction 1: the MSHR win is available at today's clock

The framing was that `m_line`'s per-bit muxing "exists because all 8
entries are accessed in parallel".  Reading the RTL, that is not quite the
mechanism.  The array is expensive because it is **written and read as a
flat 4,096-bit register file** — `docs/l2c_perf.md` §3 decomposes the
remaining ~8,600 LUT (after the landed 5.2 fix) as *"five separate flat
dynamic reads of `m_line[act]`/`r_wdata[act][rk_i]` at 32:1-over-128b or
8:1-over-512b (~5,300)"*.

But **the traffic is already serial**: fill beats arrive from the DDR
bridge at **one beat per cycle**, so an 8-entry x 4-beat array only ever
needs **one write port**.  The flat register file is not buying any
parallelism that the traffic uses.

| Structure | Today | As a memory | ESTIMATE |
|---|---|---|---|
| `m_line` | 8 x 512 b = **4,096 FF** + flat write/read mux | **32 x 128 b** LUTRAM ({entry, beat}) | ~150-250 LUT replaces 4,096 FF + ~2,500-3,000 LUT of mux |
| `r_wdata` / `r_wstrb` | 8 x 4 x 144 b = **4,608 FF** + 32:1-over-128 b read | **32 x 144 b** LUTRAM ({entry, replay_slot}) | ~200-300 LUT replaces 4,608 FF + ~2,000-2,600 LUT of mux |
| `p_wdata` / `p_wstrb` | 8 x 144 b = 1,152 FF + 8:1 read | 8 x 144 b LUTRAM | ~60-100 LUT replaces 1,152 FF + ~400 LUT |

**ESTIMATE total: ~4,900-6,000 LUT and ~9,900 FF recovered, for ~400-650
LUT of LUTRAM and address decode.**  Net **ESTIMATE ~4,300-5,600 LUT**, on
top of the 10-14k that `docs/l2c_perf.md` 5.2 already estimates.  That
would take `u_mshr` from 20,953 toward the 5-9k range.

Note the resource swap: LUTRAM is at **4,856 of the design's LUTs** and
BRAM is at **115 of 480 tiles (24%)** — both have room, while LUT is at
86.2%.  `docs/l2c_perf.md` §2 makes exactly this point: *"BRAM is ~52%
free.  That asymmetry is what makes 'move fabric registers into BRAM' a
strict win here."*  At 32-deep, distributed RAM is the better fit than
BRAM, and it has **asynchronous read** — so the read side costs **zero
added latency**.

**None of this needs a faster clock.**  It is available today.  Which
matters, because it means the largest area recovery in the SoC is not
blocked on the re-clock.

### 7.2 What the faster clock actually buys here

The one place the multiplex adds serialisation is **install**: today
`l2c_mshr` reads all 512 bits of `m_line` at once to write `l2c_data`.  As
a memory, that becomes **4 sequential 128-bit reads + 1 atomic write**, so
+3 cycles on the miss path.

| L | +3 cycles as a share of a miss | at fabric:CPU = 2:1 | at 3.33:1 |
|---|---|---|---|
| 400 ns (assumed) | 7.5% | +1.5 CPU cyc | +0.9 CPU cyc |
| 120 ns (realistic ESTIMATE) | **25%** | +1.5 CPU cyc | +0.9 CPU cyc |

**That is the correct statement of the coordinator's insight: the faster
clock does not enable the time-multiplex, it pays for the serialisation the
time-multiplex introduces** — and it matters most in exactly the case
(short real DDR latency) where the multiplex would otherwise cost 25%.

### 7.3 Correction 2: the front-door multiplex ratio is 1.1x, not 3x

The framing was that "one physical lookup pipeline at 3.3x clock serves ~3
ops per CPU cycle".  **It serves ~1.1.**  The 3.00 cyc/hit figure
(`docs/l2c_perf.md` §1.1) is **occupancy, not latency** — §1.3 is explicit
that the front door takes one op at a time through
`S_IDLE`/`S_WAIT`/`S_LOOKUP`, and that "getting below 3 requires
overlapping two ops".  So:

```
    333.333 MHz / 3 cycles per op  =  111.1 M ops/s
```

| CPU clock | L2 ops per CPU cycle from ONE pipeline at 333.333 |
|---|---:|
| 100 MHz | **1.11** |
| 166.667 MHz | 0.67 |
| 200 MHz | 0.56 |

The good news is that 0.56 is still comfortably above demand.  An OoO 68k
with a 2-wide front end and a working L1 presents well under 0.2 L2 ops per
CPU cycle.  **So a single physical front door in a faster domain is
sufficient at every CPU rate under consideration**, and
`docs/l2c_perf.md` §7.4's two-stage front door (**+400-700 LUT**, explicitly
deferred because the design is at 86.2% LUT and congestion 6) becomes
**unnecessary**.  That is a real area saving attributable to the clock — it
is just 400-700 LUT, not thousands, and it should be claimed honestly at
that size.

**The headroom does not evaporate at 200 MHz** (the coordinator's concern):
0.56 ops/CPU-cycle is still ~3x demand.  What *does* degrade with a faster
CPU is **hit latency in CPU cycles**, which is §8's subject, not this one.

### 7.4 Candidate ranking

| Structure | Multiplex? | Why |
|---|---|---|
| `l2c_mshr` `m_line` / `r_wdata` / `p_wdata` | **Yes — top candidate** | Traffic is already 1 beat/cycle; needs one write port; recovers ESTIMATE ~4-6k LUT and ~10k FF; throughput-bound, absorbs latency |
| L2 front door lookup | **Yes, implicitly** | Not a change — just don't duplicate it (7.4 of `l2c_perf`); one pipeline in a 2x domain already exceeds demand |
| DMA descriptor engine | **Yes** | Not built yet; build it multiplexed from the start. Pure throughput, zero latency sensitivity |
| Scanout line fetch | **Yes** | `scanout_line_fetch` ring is 4 Kbit and conserved at any width; it is a streaming prefetcher, latency-insensitive by construction |
| `axi_xbar` arbitration | **No** | Already width-independent FSMs; only 4,779 LUT total, and R-return is deliberately *combinational* (`axi_xbar.v:3446-3466`) to save a cycle. Multiplexing would add the cycle back |
| `l2c_data` (URAM array) | **No** | 64 URAM is 100% of the device and is set by the 64 B line. Already effectively time-multiplexed (2-cycle registered read) |
| `l2c_victim` | **No** | 1,024 FF of line storage, 2 entries. Nothing to multiplex |
| L2 tag lookup / way compare | **No** | This is the latency the CPU stalls on. Serialising the 8-way compare directly adds hit latency |

### 7.5 Where it hurts, stated plainly

A time-multiplexed structure has **equal or better throughput and worse
single-access latency**.  The rule that follows:

- **Absorbs it:** MSHR fill assembly, install, DMA, scanout prefetch,
  victim writeback.  All throughput-bound, all already behind a DDR round
  trip that is 10-40x the added cycles.
- **Does not absorb it:** anything on the L2 *hit* path the CPU is stalled
  on — tag compare, way select, response return.

And one aggravating factor specific to this machine: **the L1 D-cache is
blocking.**  `cpu/rtl/core/mem/dcache.v:61` — "Only one outstanding
operation at a time"; `:899` — `req_ready = (state == S_IDLE) && !cooldown`,
no hit-under-miss.  So today the CPU *does* stall on every L2 hit, and
added L2 hit latency is felt in full.  `m68k-core-040-ooo` is expected to
fix this, but **until it does, no latency may be added to the L2 hit
path**, and that is an independent reason to keep the hit path out of any
multiplex.

---

## 8. Where does L2 live?

L2 is the point of coherence and sits between a 100/200 MHz consumer and a
333.333 MHz supplier.  The answer depends entirely on **what kind of
crossing separates it from the CPU**, which is why §5.2 had to come first.

### 8.1 The cost of the crossing, both ways

Structural latency of `axi_async_bridge` (derived from
`async_fifo.v:564-661` and `axi_async_bridge.v:287-296`): a read transaction
costs **~4 source clocks + ~4 destination clocks** (AR = 1 src + 3 dst; R =
1 dst + 3 src; AW pays a 5th for the `aw_stage_valid` elastic register).
`docs/soc_bus_review.md` §5 *measured* `u_core_to_mig_ui` at 3.0 core cycles
round trip — lower than the structural bound, so treat 3-5 core cycles as
the range and use the pessimistic end for a latency-critical decision.

| Arrangement | CPU | Crossing | CDC round trip | L2 hit itself | **Total hit** |
|---|---|---|---:|---:|---:|
| **L2 in CPU domain** (today) | 100 | none | 0 | 3 cyc = 30 ns | **30 ns** |
| L2 in fabric domain, **async** | 100 | 10:3 async FIFO | 4x10 + 4x3 = **52 ns** | 3 cyc @333 = 9 ns | **61 ns (2.0x worse)** |
| L2 in fabric domain, **async** | 200 | 5:3 async FIFO | 4x5 + 4x3 = **32 ns** | 9 ns | **41 ns vs 15 ns (2.7x worse)** |
| **L2 in fabric domain, integer 2:1** | **166.667** | elastic buffer, ~1 dst cyc each way | 3 + 6 = **9 ns** | 9 ns | **18 ns = 3 CPU cyc — neutral** |
| L2 in CPU domain | 166.667 | none | 0 | 3 cyc = 18 ns | 18 ns |

**Two conclusions, and they are the crux of the whole study:**

1. **Across an asynchronous CDC, moving L2 into the fabric domain roughly
   doubles hit latency in CPU cycles, and it gets worse as the CPU gets
   faster** (2.0x at 100 MHz, 2.7x at 200 MHz).  The CDC's cost is dominated
   by the *slow* side — 4 edges of the CPU clock — so it does not shrink as
   the fabric speeds up.  With today's clock family, **L2 must stay in the
   CPU domain.**
2. **Across an integer 2:1 synchronous crossing it is latency-neutral, and
   it doubles hit throughput for free** (111 M ops/s instead of 55.6).  It
   also makes `docs/l2c_perf.md` §7.4's two-stage front door (+400-700 LUT)
   unnecessary.

**The integer-ratio result from §5.2 is what makes L2-in-the-fabric-domain
viable at all.**  Without it the option is simply bad.

### 8.2 The recommended split, and why it is a migration

Do not split `l2c` internally — its data array is 64 URAM at 100% device
occupancy with a single clock (URAM288 has one CLK for both ports), so the
tag/data/front-door complex cannot straddle a boundary cheaply.  Split the
**fabric** instead, at a boundary that already exists:

```
   CPU domain                             |  fabric domain (MIG UI)
   -------------------------------------- | -----------------------------
   m68k core, L1I, L1D                    |
   axi_xbar (CPU-facing ports)            |
   l2c: front door, tags, u_data (URAM)   |
   l2c: MSHR fill engine, victim buffer   |
   ------------------[ CROSSING ]--------- |
                                          |  axi_vram_priority_mux3
                                          |  scanout_ddr_reader / line_fetch
                                          |  vhdd_ddr (RAM disk)
                                          |  DMA engine (when built)
                                          |  axi_ddr4_mig_bridge -> MIG
```

**Today the crossing sits one hop lower** — `docs/soc_bus_review.md` §3a
hop 7 is `axi_vram_priority_mux3` (core domain) and hop 8 is
`u_core_to_mig_ui`.  The proposal is to **move one existing bridge up past
the VRAM lane mux**, which puts scanout, the RAM disk and the future DMA
engine on the fast side.

That delivers exactly the stated goal — *"running at ddr clk gives headroom
for dma and vram activity"* — because:

- Scanout drops from **71% of its lane's capability to ~21%**
  (`scanout_ddr_reader.v:70-75`), so its cost becomes genuinely "sporadic
  and hidden".
- DMA and RAM-disk bursts stop consuming CPU-domain fabric slots entirely.
- `MAX_BULK_AHEAD = 2` (F3) can be relaxed on bandwidth grounds rather than
  rationed.
- The CDC that survives carries only L2 fill and victim traffic — 64 B
  bursts behind a 100-400 ns DDR latency, where a 3-5 cycle crossing is
  **1-5% overhead**, i.e. exactly where a crossing should be placed.
- `u_vhdd_ddr_cdc` disappears (the RAM disk would already be fabric-side).

**Place the crossing where the latency is already large.  It is currently
one hop away from there.**

### 8.3 A caveat on the crossing count

Crossings compound, and this design already demonstrates it: a JTAG host
read of a `debug_ctrl` register crosses **four** boundaries — core->pb
(`u_pb_s1_cdc`), pb->core (`u_dbg_pb_to_core`), and both returns — for
roughly **8 core + 8 pb clocks ≈ 240 ns of pure bridge latency** before any
JTAG shift time.  Adding a fabric domain must not add a second crossing to
any path that already has one.  The split above adds zero new crossings to
the CPU's DDR path; it relocates the single existing one.

---

## 9. Closing timing — what it would actually take

### 9.1 The good news: the fabric is not what is failing

**All 50 worst paths in the current post-route report are inside `u_cpu`**
(`build/vivado/timing_summary.rpt:6334-6892`): 42 start at
`u_cpu/u_cpu/u_iq_fp/e_valid_reg[5]` and end in
`u_cpu/u_cpu/u_fpu/u_div/{quot,rem,result}_reg[*]`; 8 start at
`u_iq_mem/e_ccr_has_dst_reg[1]`.  36-42 logic levels, **64-77% route
delay**.  **Zero** violating paths in `l2c`, `axi_xbar`, any CDC bridge,
`ddr_ctrl` or the video path.

So "this design barely closes 100 MHz" is true of the **CPU**, and the CPU
is the component being replaced.  **The fabric's Fmax has never been
measured**, because the CPU's paths mask it.  We are making a re-clocking
decision without the single number the decision depends on.  §11 fixes that.

Two datapoints that are encouraging:

- **`mmcm_clkout6` = 166.667 MHz closes with +1.571 ns (26% of period)** on
  5,336 endpoints, in this build, on this device.
- **`mmcm_clkout0` = 333.333 MHz closes at exactly 0.000 ns** on 30,927
  endpoints.  333 MHz is achievable on a `-2` KU5P — for hand-floorplanned
  vendor IP, at literally zero margin.

The second one is the honest warning as much as the encouragement: **the
MIG achieves 333 MHz with no margin at all**, and it is a purpose-built,
floorplanned hard-IP-adjacent block. Our fabric is neither.

### 9.2 The bad news: congestion and URAM anchoring

From `build/vivado/reports/congestion_route.rpt` (dated **Aug 10**, i.e.
**9 days stale** and from the `-0.174 ns` era, not the current `-0.612`):

- 5 placer windows, **all level 5**, **all at URAM 100%**.
- `u_l2c/g_active.u_ctrl` appears in **3 of 5** — rows 25, 26, 29 — always
  third, at 11-13%.  Confirmed: it is co-resident with `u_commit` in two
  windows and with `u_rob` in one.  (The brief's "interleaved with ROB *and*
  commit" is right in substance; it never shares a window with both at
  once.)
- **The mechanism is the URAM column.**  `u_data`'s 64 URAM288 sites are a
  fixed, unmovable resource column that anchors L2 directly on top of the
  CPU's hottest placement region.  Every congestion window reports URAM at
  100%.
- At the **router** stage, peak congestion is **level 6** and is
  overwhelmingly CPU (`u_cpu` 75%, `u_rob`, `u_iq_mem`, `u_commit`,
  `u_dec`); L2 drops to 4% in one window of four.

**So re-clocking `l2c` is a placement problem before it is a timing
problem**, and the placement problem is not solvable by floorplanning L2
elsewhere — the URAM column is where it is.  What *is* solvable: the CPU
being replaced may not want the same region, and the MSHR time-multiplex
(§7.1) removes ~10k FF and ~5k LUT from precisely the block that is
co-resident with `u_commit`.

### 9.3 What each target would require

| Target | Path budget | Reduction needed | Structural work |
|---|---:|---:|---|
| 100 MHz (today) | 10.0 ns | 1.0x | — |
| **111.111 MHz** (UI/3) | 9.0 ns | **1.11x** | Essentially none for the fabric. Absorbs into the pipelining F1/F2 need anyway. |
| **166.667 MHz** (UI/2) | 6.0 ns | **1.67x** | Register the xbar's combinational R-return mux (4 masters x 6:1 over 128 b, `axi_xbar.v:3446-3466`) — costs ~1 cycle and ~600 FF. Split `l2c_ctrl`'s `S_LOOKUP` (8-way tag compare + victim/MSHR/skew hazard checks + way-select mux, all one cycle). `m_line` -> memory (§7.1) removes the flat 512-bit read cones. |
| 333.333 MHz (UI) | 3.0 ns | **3.33x** | All of the above, plus: 2-3 stage split of every remaining lookup, a floorplan (Pblocks) for the fabric, and acceptance that the MIG itself only makes 333 at 0.000 ns margin. |

**ESTIMATE for 166.667:** the three items above cost roughly **+800-1,200
FF and +200-400 LUT** in added pipeline registers, against **-4,300 to
-5,600 LUT** recovered by the MSHR time-multiplex in the same work.  **Net
area-negative.**

**ESTIMATE for 333.333:** not costable without knowing the fabric's actual
Fmax.  On the evidence available it is a research project, not a migration.

### 9.4 Two constraints that get harder with frequency

- **`set_bus_skew` coverage is incomplete.**  `synth/vivado.tcl:1484` applies
  a Gray-pointer bus-skew bound by *register name pattern*, matching only
  `async_fifo.v`.  **`fb_reader_cdc_fifo`'s `wgray`/`rgray` pointers
  (`fb_reader.v:372-490`) are not covered.**  Bus skew tolerance shrinks
  with the clock period, so this latent gap gets worse at every step.  It is
  self-documented as collision-fragile at `vivado.tcl:1354-1371`.
- **`REQ_MIN_HOLD = 8`** (`async_fifo.v:472-487`) bounds the usable ratio to
  ~1:4.  Today's 1:3.33 has ~20% margin; `make fpga-50mhz`
  (`Makefile:2361-2384`) would put it at 1:6.7 and **break the reset
  handshake**, with nothing asserting it.  Under §5's proposal this
  disappears, because the boundary stops being an async FIFO.

---

## 10. Migration path — four steps, each independently shippable

Every step keeps the machine booting.  Steps 1 and 2 are the concurrency
work and must land before any re-clock.

### Step 0 — Measure the fabric's Fmax (see §11).  Blocks nothing else.

### Step 1 — Concurrency (F1 + F2 + F3 + F4, as ONE change set)

Owned elsewhere; listed because everything after depends on it.  Per
`docs/l2c_perf.md` §4.4 and `docs/soc_bus_review.md` §8, **landing any
subset measures as zero**.  Also land the socket contract requirement
(`docs/soc_bus_review.md` §7a): **`m68k-core-040-ooo` must rotate AXI IDs on
both masters**, or the 6.6x same-ID cliff re-imposes the limit silently.

Until this lands, the fabric runs at **5-33% of the clock it already has**,
and re-clocking multiplies a small number by a small number.

### Step 2 — MSHR time-multiplex (§7.1), area-negative, clock-neutral

`m_line` / `r_wdata` / `p_wdata` from flat flip-flop arrays to
{entry, beat}-addressed LUTRAM.  ESTIMATE **-4,300 to -5,600 LUT, -9,900
FF**.  Available at today's clock; the +3 install cycles cost 7.5% of a
miss at L=400 ns.  This is the change that buys back the area needed to
pipeline for step 3, *and* it thins the block that is co-resident with
`u_commit` in 3 of 5 congestion windows.

### Step 3 — Re-root the fabric clock, at the same frequency

Move `core_clk`/`pb_clk` from the AB7 GT refclk (`fabric_clk100`, 100 MHz)
onto a `BUFGCE_DIV` off the MIG UI clock: **111.111 MHz = UI/3**, pb =
UI/8 = 41.667 MHz.  This is **+11% on the core clock** and converts
`u_core_to_mig_ui` from an async FIFO to a **3:1 synchronous** crossing.

What this costs, honestly:

- **The fabric clock now dies when the MIG resets or fails to calibrate.**
  The JTAG/debug island must stay on the independent GT clock — it already
  does (`u_dbg_pb_to_core`, 340 LUT, off the hot path).  This is the single
  biggest risk in the whole plan and it must be validated before anything
  else moves.
- `synth/vivado.tcl:329` **hard-pins `PB_CLK_HZ` to 50 MHz** in validation;
  that check must be relaxed.  The Q700 783.36 kHz VIA timebase is safe by
  construction — `fpga_top` derives `phi2_tick` from an NCO off an explicit
  `PB_CLK_HZ` generic (`fpga_top_clocks.vh:884-901`).
- The three BUFG_GTs share one deliberately-unsynchronised `CLR`
  (`fpga_top_clocks.vh:100-133`), which is what keeps them phase-coherent.
  A re-rooted tree needs an equivalent story.
- `set_clock_groups -asynchronous` at `vivado.tcl:1598` must be replaced by
  a *synchronous* relationship, which means Vivado starts timing ~31k
  endpoints it currently ignores.  **Expect this step to surface real
  violations that were previously invisible.**  That is the point of doing
  it at the same frequency first.

### Step 4 — Raise the fabric to 333.333 (UI), CPU to 166.667 (UI/2)

Only after step 0 has produced a number.  Move
`axi_vram_priority_mux3`, `scanout_ddr_reader`, `vhdd_ddr` and the DMA
engine to the fabric domain (§8.2) — the crossing relocates up one hop and
becomes a 2:1 synchronous elastic buffer.  Fabric ceiling **5.33 GB/s**
against a 1.2-1.75 GB/s demand: **3-4x headroom**.

If the CPU cannot make 166.667, stop at step 3 with the fabric at UI/3 or
UI/2 and the CPU at UI/3.  Every intermediate point is a valid resting
place, which is the property that makes this a migration.

**Explicitly not in the plan: narrowing the fabric to 64 bits (§6.4), and
re-generating the MIG at a different speed bin (§5.1 — pinned in four
places, two of which are equality checks against an external golden
project).**

---

## 11. Recommendation, and the one measurement that matters

### 11.1 Recommendation

1. **Do the concurrency work first.**  It delivers 6.5x where frequency
   delivers 1.39x, and it costs nothing in timing closure (§4).  F1+F2+F3+F4
   as one change set, plus ID rotation in the socket contract.
2. **Then time-multiplex the MSHR** (§7.1).  ESTIMATE -4,300 to -5,600 LUT
   and -9,900 FF, available at today's clock, and it funds the pipelining
   that a re-clock needs.
3. **Then re-root the fabric onto the MIG UI clock family at UI/3 =
   111.111 MHz** (§10 step 3) — a +11% clock and, more importantly, the
   conversion of the DDR crossing from asynchronous to **integer
   synchronous**.
4. **Then target fabric = 333.333 (UI), CPU = 166.667 (UI/2)** (§10 step 4),
   with scanout, DMA and the RAM disk on the fast side.
5. **Do not narrow the fabric** (§6).  ~1% LUT for a 13-file migration that
   makes throughput worse until F2 lands.  If width is revisited, revisit it
   *upward* toward the 256 reserved in `cpu_socket.vh`.
6. **Retarget the core to 166.667 MHz rather than 200.**  200 is a round
   number with no external requirement behind it; 166.667 is 83% of it and
   is the only rate in that neighbourhood that is an integer divisor of the
   MIG UI clock.  That single choice removes every asynchronous crossing
   from the memory path.

Answering the original question directly: **yes, the fabric should run
faster than the CPU — at exactly 2x, not 3.33x — and the L2 should move
with the fabric only once the crossing is integer-synchronous.  Until then
L2 stays with the CPU.**

### 11.2 Assumption register

| Assumption | Value used | Sensitivity |
|---|---|---|
| **DDR round-trip latency** | **40 +/- 8 core cycles = 400 +/- 80 ns** | **High, and inverted.** Every conclusion about *when* the fabric clock binds scales linearly with it. A *shorter* latency makes the re-clock **more** urgent (§3.2): at L=120 ns the 100 MHz fabric saturates at 3 outstanding fills, and an 8-entry MSHR becomes unfeedable. Never measured (`scanout_ddr_reader.v:77-80`). |
| Fabric Fmax | **Unknown** | Total. The CPU masks it (§9.1). |
| CDC round trip | 3.0 core cyc measured / 4 src + 4 dst derived | Moderate; used pessimistically in §8.1 |
| MSHR multiplex saving | ESTIMATE 4.3-5.6k LUT | Moderate; structural, from `l2c_perf.md` §3's own decomposition |
| Congestion data | **9 days stale** (Aug 10, `-0.174 ns` era) | Directional only |

### 11.3 The single measurement

**Synthesise and implement the SoC with the CPU stubbed out, with
`fabric_clk100` constrained at 3.000 ns, and read the WNS.**

That one number decides everything above:

- It is the only unknown that separates "step 4 is a migration" from "step 4
  is a research project" (§9.3).
- The infrastructure already exists — `.claude/skills/m68k-build-bitstream`
  documents that a bare `make impl` builds the **CPU stub**, and
  `build/timing_archive/nol2c_*.rpt` shows L2-disabled variant builds are a
  routine thing here.  So this is a configuration of an existing flow, not
  new work.
- Sweep it: 3.000 / 6.000 / 9.000 ns gives the fabric's Fmax curve and
  tells you directly whether UI/1, UI/2 or UI/3 is the reachable target.
- Report per-hierarchy WNS so `l2c`, `axi_xbar` and `scanout_*` are costed
  separately — §9.3's structural work list is a guess until then.

**The second-most-valuable measurement, and it is nearly free:** instrument
the real DDR round trip on hardware (a cycle counter between AR accept and
first R at `axi_ddr4_mig_bridge`'s MIG-side port, read back over JTAG).
Every margin claim in `docs/l2c_perf.md`, `docs/soc_bus_review.md`,
`docs/dma_engine_design.md` and this document rests on a number nobody has
ever measured, and §3.2 shows the conclusions move by a factor of 3 across
its plausible range.

---

## Appendix A — corrections this study makes to existing documentation

Not fixed here (this document changes nothing but itself); recorded so the
owning documents can be corrected.

| Location | Says | Actually |
|---|---|---|
| `rtl/board/ddr_ctrl.v:42` | "DDR4-2400, 1200 MHz memory clock" | **DDR4-2666, 1333.33 MHz** (`.xci` `DDR4_TimePeriod 750`; `pll_clk[0]` = 2666.666 MHz) |
| `docs/dma_engine_design.md:125`, `docs/soc_bus_review.md:549` | "~9.6 GB/s" DRAM peak | **10.67 GB/s** |
| `docs/clocking.md:37,525`; `docs/hardware_feasibility.md:244,281` | `clk_ddr_ui` = 300 MHz | **333.333 MHz** |
| `docs/clocking.md:37` | `clk_scsi` 10 MHz, `clk_eth_rgmii`, `clk_usb`, `clk_audio` are domains | **None exist.** SCSI/SONIC/ASC are all plain `pb_clk` |
| `docs/clocking.md:526-529` | "MIG provides its own CDC, so we do NOT insert our bridge here" | `ddr_ctrl.v:779` instantiates `u_core_to_mig_ui` on the production path (already flagged as F9) |
| `rtl/soc/fpga_top_clocks.vh:632-634, 1023` | "pb_clk = sys_clk/4 on HW" | That is the SIM_MODEL relation; on HW it is **/2** of a 100 MHz root |
| `rtl/soc/fpga_top.v:366` | `CORE_CLK_HZ = 200_000_000` default | Real value is a synth generic; actual is 100 MHz |
| `fpga_top_clocks.vh:459-464` | btn debounce "10 ms @ 200 MHz" | **20 ms** on the real 100 MHz build |
| `synth/ddr4.xdc:4` | "32-bit, x8x4" | `MT40A512M16LY-075` is **x16**, so **two** components |
| Brief / this task | `u_vhdd_ddr_cdc` "now compiled out" | Gated by `ENABLE_DDR_RAMDISK`, which grep does not find in `Makefile` or `vivado.tcl` — **yet it appears at 672 LUT in `utilization_route.rpt:282`.** One of the two is stale; **worth resolving**, it is 0.36% of a design at 86.2% LUT |
| `docs/soc_bus_review.md` §3c | `u_pb_s1_cdc` = 771 LUT, 8.0 core cycles, 57% of a VIA read | The **32-bit** `axi_pb_s1_cdc` is now the live variant (`PB_S1_WIDE_CDC` undefined); the 771 LUT figure is pre-change. Re-measure before re-quoting |
