# SoC Interconnect: Critical Architectural Review

**Status:** analysis + costed proposals. No fabric changes made.
**Date:** 2026-08-19
**Scope:** `rtl/soc/axi_xbar.v`, `rtl/soc/l2c*.v` (read-only — owned by
another agent this session), `rtl/soc/axi_async_bridge.v`,
`rtl/soc/peripheral_bus.v`, `rtl/board/axi_ddr4_mig_bridge.v`, and the
CPU-side socket adapters in `cpu/rtl/sys/`.
**Forcing function:** the fabric must host `m68k-core-040-ooo`, a larger
OoO core replacing today's `u_cpu` (107,855 of 187,011 LUT = 58%).

## 0. Method, and what is measured vs. derived

Everything below is either (a) read directly out of the RTL with a
file:line citation, (b) taken from an existing post-route report in
`build/vivado/reports/` or `synth/timing_reports/`, or (c) **measured**
by running a Verilator testbench. No Vivado was run; no board or JTAG was
touched. Estimates are labelled as estimates.

The measured numbers come from `tb/tb_l2c_chain.v` +
`tb/tb_l2c_chain.cpp`, which instantiate the real
`l2c → axi_async_bridge → axi_ddr4_mig_bridge → sim_mig_backend` chain.
The shipped target runs that model at `DDR_READ_LATENCY_BASE = 200`
(`tb/tb_l2c_chain.v:45`), which is ~4x pessimistic against real DDR4. All
figures in this document were re-measured with
`-GDDR_READ_LATENCY_BASE=40 -GDDR_READ_LATENCY_JITTER=8`, built into a
scratchpad directory so no repo file changed. Hit-throughput figures come
from an added scenario in a **scratchpad copy** of `tb_l2c_chain.cpp`;
the repo copy is untouched.

Clocking, as actually built (`Makefile:2226-2228`, `TARGET_FREQ_MHZ=100`,
`CORE_CLK_HZ=100_000_000`, `PB_CLK_HZ=50_000_000`; confirmed by the
`fabric_clk100` 10.000 ns period in
`synth/timing_reports/timing_20260819_131707.rpt`):

| Domain | Frequency | Ratio to core |
|---|---|---|
| `core_clk` | 100 MHz | 1x |
| `pb_clk` | 50 MHz | core/2 |
| `mig_ui_clk` | MIG-owned | asynchronous |
| `pclk` (video) | mode-dependent | asynchronous |

Note the RTL *defaults* say 200 MHz (`rtl/soc/fpga_top.v:366`); the
shipping build overrides them. Cycle counts below are core_clk cycles at
100 MHz (10 ns each) unless stated.

> **Line-number caveat.** `rtl/soc/l2c_ctrl.v` and `rtl/soc/l2c_mshr.v`
> were being actively modified by another agent while this review was
> written, and the working tree also carries uncommitted changes to
> `fpga_top_dma.vh` and `fpga_top_peripherals.vh`. Line numbers for those
> files are correct as of the final pass but will drift; the symbol names
> (`ar_have`, `s_arready`, `st`, `ar_beat`, `id_busy_c`,
> `MAX_BULK_AHEAD`, `rs_state`, `ws_state`) are the durable references.
> Section 4b quantifies the one L2 change that landed mid-review.

---

## 1. Executive summary

The interconnect is cheap (~10 K LUT of glue against 187 K total) and
correct — it has been hardened, exhaustively, against wedges, abandoned
bursts, reset races and stale responses. That hardening is real
engineering and should be preserved. But it was built one correctness
fix at a time, and the cumulative shape is a fabric in which **every
path is single-outstanding, end to end, at five stacked levels.**

Three numbers frame the whole review, all measured this session:

1. **The L2 charges per beat, not per transaction, so bursting buys
   almost nothing.** Measured on the current tree: a 4-beat 64 B burst
   hit costs 3.54 cyc/beat; pipelined single-beat hits cost 3.09
   cyc/beat. A burst is *slower per byte* than pipelined singles.
   Every widening and burst-packing optimisation upstream of the L2
   (task #170's narrow→wide packer included) is pushing against a wall
   it cannot move.
2. **Miss concurrency is worth ~5x, and the fabric throws it away.**
   Measured at a realistic 40±8-cycle DDR: 8 concurrent line fills
   complete in 110 cycles (13.8 cyc/miss); 8 sequential fills take 546
   cycles (68.3 cyc/miss) — a **4.96x** gap. The L2's MSHR can sustain
   8 fills. The crossbar permits **exactly one outstanding read per
   master port** (`rtl/soc/axi_xbar.v:3208-3235`), and the L2 slave port
   accepts **one AR at a time globally** (`l2c_ctrl.v:119`,
   `s_arready = !ar_have`). So no master can ever get more than 1.
3. **At full MLP the L2 returns misses faster than it returns hits.**
   8 concurrent misses = 512 B in 110 cycles = 4.65 B/cyc. Burst hits =
   4.52 B/cyc. A cache whose miss path outruns its hit path is not
   buying much, and this follows directly from (1).

The corollary for the redesign: **the bottleneck is not width and not
DDR. It is transaction concurrency and the single-op L2 front door.**
A wider CPU port bought without fixing those two will measure as no
change at all — and will look like the widening "didn't work", which is
exactly the kind of false conclusion that sets a project back months.

### This is no longer a throughput play — it is a prerequisite

**Update, and it changes the ranking of everything below.** The core the
SoC exists to host, `m68k-core-040-ooo`, is **multi-outstanding on both
its instruction and data sides**. The current fabric's one-outstanding
design is not a neutral simplification: it is justified, in three places
in series, by an assumption that is about to become false —

| Where | The justification, quoted | Status once the new core lands |
|---|---|---|
| `rtl/soc/if_to_axi.v:16-18` | "One outstanding transaction at a time. The core's if_stage already self-serialises its fetches … so there is no queue here." | **False** |
| `rtl/soc/axi_narrow_to_wide.v:72-76` | "One outstanding transaction per direction … Both the core LSU and boot_fsm self-serialise, so no queue here." | **False** |
| `axi_xbar.v:3208` + `l2c_ctrl.v:119` | crossbar 1-outstanding per master; L2 accepts one AR globally | Unchanged, and now binding |

Those comments are accurate today (`cpu/rtl/core/mem/dcache.v:61` "Only
one outstanding operation at a time"; `cpu/rtl/core/fetch/icache.v:12`
"Only one outstanding request at a time"; `if_stage.v:61` "ties up the
single-outstanding I-cache"). They become **actively misleading** the day
the core is swapped, because they will read as "this is fine" to whoever
is debugging why the new core is slow.

And the penalty for getting the ID discipline wrong is now measured. The
L2C agent measured the same-ID cliff directly: **8 outstanding misses
with unique IDs cost 8.38 cyc/op; the same 8 misses with a single ID cost
55.40 cyc/op — a 6.6× cliff.** A multi-outstanding core that reuses one
ARID lands on the wrong side of that cliff and will be misread as "the
new core is slow" rather than as a fabric limitation.

### Area context

The interconnect's area is not where the problem is, but one number is
worth flagging because it is being *paid for and not used*:

| Block | LUT | % of design | Source |
|---|---|---|---|
| `u_cpu` (being replaced) | 107,855 | 57.7% | `utilization_route.rpt` |
| `u_l2c` | 24,222 | 13.0% | ” |
| ` └ u_mshr` | **20,953** | **11.2%** | ” |
| `u_mig_ddr4` | 10,282 | 5.5% | ” |
| `u_xbar` | 4,779 | 2.6% | ” |
| `u_repo_to_pcie_mig` | 1,648 | 0.9% | ” |
| `u_pb_s1_cdc` | 771 | 0.4% | ” |
| `u_core_to_mig_ui` | 717 | 0.4% | ” |
| `u_vhdd_ddr_cdc` | 672 | 0.4% | ” |

`u_mshr` is **20,953 LUT — 11.2% of the entire design, and 86% of the L2
cache** — to provide 8-way miss concurrency that is capped **three times
over** before it can be used:

- the crossbar admits **1** outstanding read per master port (F1),
- the L2 front door admits **1** AR at a time, globally (F2),
- and the VRAM lane mux admits **2** bulk reads ahead (F3).

That is the single largest piece of dead capability in the SoC. It is not
wasted *design* — the capability is real, verified, and measured at
4.96× — it is wasted because nothing in the fabric can feed it. Fixing
the fabric is therefore not "spending area to buy performance"; it is
unlocking 21 K LUT of performance already bought and paid for.

---

## 2. Topology, as actually built

The production flags are pinned in `tools/build_bitstream.sh:63-71`:
`L2C_ENABLE=1 VRAM_IN_DDR=1 USE_REAL_MIG=1 TARGET_FREQ_MHZ=100`. With
those set, the real chain contains one arbiter that is easy to miss —
`u_vram_lane_mux` — which does not appear in `fpga_top.v`'s header
diagram:

```
  CPU (m68k_axi_wrapper)                       core_clk 100 MHz
   ├─ if_stage ──► if_to_axi ──────────────► xbar M2  (read-only, 128b)
   │                (1 outstanding)
   └─ dcache ──► axi_narrow_to_wide ───────► xbar M0  (128b)
                  (32→128, 1 rd + 1 wr)        ▲
   boot_fsm ──► axi_narrow_to_wide ───────────┘ (reset-hold mux, not an arbiter)
   JTAG-AXI ──► axi_narrow_to_wide ────────► xbar M1
   vhdd_ddr ──► axi_async_bridge ──────────► xbar M3   (from pb_clk)

                        ┌──────── axi_xbar 4-slot × 6-slave ────────┐
                        │  1 outstanding read  per slot             │
                        │  1 outstanding write per slot             │
                        │  writes additionally lock the slave AW→B  │
                        └───┬───┬───┬───┬───┬───┬────────────────────┘
                            │   │   │   │   │   └─ S5 SD JTAG writer
                            │   │   │   │   └───── S4 DAFB shim
                            │   │   │   └───────── S3 VRAM write lane → mux3
                            │   │   └───────────── S2 vhdd_ctrl regs (was DMA cfg)
                            │   └───────────────── S1 peripheral bus
                            ▼
                          S0 ── u_l2c ── front door: 1 BEAT at a time
                                  ├─ u_mshr    (8 fills)      20,953 LUT
                                  ├─ u_victim  (1 writeback)
                                  └─ u_bypass  (1 op, single beat)
                                        │
                                        ▼
                       u_vram_lane_mux (axi_vram_priority_mux3)
                         scanout has PRIORITY; MAX_BULK_AHEAD = 2
                                        │
                                        ▼
                       u_ddr ─ u_core_to_mig_ui (async FIFO ×5)
                                 core_clk 100 MHz ──► mig_ui_clk 333.25 MHz
                                        │
                             u_repo_to_pcie_mig (128b → 256b)
                                 8 outstanding reads, 4 writes
                                        │
                                        ▼
                                   MIG / DDR4
```

Two structural remarks before the numbers:

- **Scanout outranks the CPU.** `axi_vram_priority_mux3.v:252-256`:
  `ar_pick_c` picks `AR_SCAN` whenever `scan_arvalid` is high unless the
  `MAX_SCAN_AHEAD = 4` quota has expired. Video refresh is a hard
  real-time consumer, so this is defensible — but it means CPU cache
  fills are the *lowest-priority* traffic in the system, and the fill
  concurrency they are allowed is `MAX_BULK_AHEAD = 2`
  (`axi_vram_priority_mux3.v:100`, enforced at `:240` and `:262`).
- **There is no DMA engine at all, and the xbar header is stale about
  it.** `axi_xbar.v:9-18` says DMA's M4 master is "stubbed"; in fact
  `dma_ctrl` was deleted outright on 2026-08-01 after being measured at
  **2,024 LUT / 814 FF of dead weight** on the routed build
  (`fpga_top_dma.vh:18-26`), and S2 now hosts `vhdd_ctrl`'s register
  file (`fpga_top_dma.vh:27-34`). `grep -rn dma_ctrl rtl/` returns zero
  instantiations. **Every byte of disk data on this SoC moves through
  the CPU as programmed I/O.** That is the single biggest reason the
  machine feels slow at disk-bound work, and no fabric change fixes it.
- **Scanout is not on the crossbar and not in the L2.** Under
  `VRAM_IN_DDR` (the default, `Makefile:2267`) the URAM framebuffer is
  not instantiated at all (`fpga_top_video.vh:184, 604-637`);
  `scanout_ddr_reader` reads pixels from a 32 MB DDR carveout
  (`axi_defs.vh:173-174`) and attaches directly to `u_vram_lane_mux`.
  `docs/memhier.md` still says the opposite in several places — see §6, footgun F9.

---

## 3. Hop-by-hop map — the core table

Latency figures are per-transaction in **core_clk cycles at 100 MHz**.
"Outstanding" is the maximum number of transactions that hop will accept
before refusing READY.

### 3a. CPU data load → DDR (the hot path)

| # | Hop | Width | Outstanding | Latency (cyc) | Serialisation / notes |
|---|---|---|---|---|---|
| 1 | `dcache` FSM | 32b | **1** | 1-2 | `req_ready = (state == S_IDLE) && !cooldown` — `cpu/rtl/core/mem/dcache.v:899`. **Fully blocking**: no hit-under-miss. |
| 2 | `axi_narrow_to_wide` | 32→128b | **1 rd + 1 wr** | ~1 | `n_arready = !ar_valid_q` — `cpu/rtl/sys/axi_narrow_to_wide.v:80-83`. Packs an 8-beat 32b line refill into a 2-beat 128b INCR burst (task #170). |
| 3 | `axi_xbar` M0 read slot | 128b | **1** | ~1 (AR registered) | `req_rd_ddr[mi] = (rs_state[mi]==RS_IDLE) && ...` — `axi_xbar.v:3208`. R return is **combinational** (`:3446-3466`) so 0 added cycles back, at the cost of a 4×6×128b mux. |
| 4 | `axi_xbar` per-slave AR reg | 128b | **1 per slave** | 0-1 | `if (any_req_rd_s0 && !s_arvalid_r[0])` — `axi_xbar.v:3751`. Single-entry, no skid: a second AR to S0 waits for the first to be accepted. |
| 5 | `l2c` front door | 128b | **1 BEAT, and 1 AR globally** | **3.1-3.5 / beat (measured)** | `st` is one register: `S_IDLE→S_WAIT→S_LOOKUP` — `l2c_ctrl.v:130, 307-380`. Beat-granular, not transaction-granular (`ar_beat <= ar_beat+1`, `:332`). `s_arready = !ar_have` (`:119`) blocks **every** master for a whole burst. |
| 5b | `l2c` same-ID accept gate | — | — | — | A new accept is blocked while its AXI ID has any live MSHR/bypass op — `docs/l2c_spec.md:463-470`. **A single-ID master gets zero miss overlap.** |
| 6 | `l2c_mshr` (miss only) | 128b | **8** | — | `parameter N = 8` — `l2c_mshr.v:42`; 1 AR/cycle issue (`:189`); fills are `arlen=3, arsize=4` = 64 B aligned (`:185-188`). 20,953 LUT. |
| 7 | `axi_vram_priority_mux3` | 128b | **2 (bulk)** | ~1 | `MAX_BULK_AHEAD = 2` — `:100`, `:240`, `:262`. Scanout has priority (`:252`). **This is the binding fill-concurrency limit in production.** |
| 8 | `u_core_to_mig_ui` (`axi_async_bridge`) | 128b | FIFO-depth | ~3 dest cyc each way | 5 independent gray-pointer async FIFOs — `axi_async_bridge.v:11-25`. **Not a handshake**; cost is amortised over a burst. AW/AR/B depth 4, W/R depth 16 (`:91-95`). |
| 9 | `axi_ddr4_mig_bridge` | 128→256b | **8 rd / 4 wr** | ~2-3 mig cyc | `RMAX_OUTSTANDING = 8`, `WMAX_OUTSTANDING = 4` — `axi_ddr4_mig_bridge.v:99-100`. Ring, not FSM. 64 B-aligned fills pack 2:1 with zero waste. |
| 10 | MIG + DDR4 array | 256b | MIG-internal | **~40 (ASSUMED)** | Explicitly flagged as an assumption, not a measurement, at `scanout_ddr_reader.v:77-80`. |

**Derived total, clean L2 miss, CPU-observed: ~72-80 cycles** (measured
68.3 at the L2 slave port with a 40±8-cycle DDR model, plus hops 1-4).
**Effective concurrency: 1**, because hop 1 blocks the D-cache port and
hop 3 blocks the crossbar slot.

### 3b. CPU instruction fetch → DDR

| # | Hop | Width | Outstanding | Notes |
|---|---|---|---|---|
| 1 | `if_to_axi` | 128b | **1** | "One outstanding transaction at a time. The core's if_stage already self-serialises its fetches" — `cpu/rtl/sys/if_to_axi.v:16-18`. |
| 2 | `axi_xbar` M2 read slot | 128b | **1** | Separate `rs_state[2]`, so I-fetch is not blocked by a data-side **peripheral** access. This is the fabric's one genuinely good decoupling — and the L2 partly undoes it (F2): both masters converge on S0, where `s_arready = !ar_have` re-serialises them for the duration of any burst. |
| 3-10 | as 3a from hop 5 | | | |

Fetch requests are `arlen=0, arsize=4` — **a single 16-byte beat, one at
a time**, with the next request gated on the previous response. At a
measured 5.2 cyc issue-and-wait L2 hit plus ~2-3 cycles of xbar and
adapter, the sustained I-fetch ceiling is **16 B / ~8 cyc ≈ 2 B/cycle**.
(The 3.09 cyc/beat pipelined figure is unreachable here: `if_to_axi` and
the xbar M2 slot are both strictly one-at-a-time, so I-fetch only ever
gets the issue-and-wait rate.) For a 2-wide 68k decoder that is roughly
break-even today and a hard wall for anything wider — and it is the
reason a wider front end in the new core will not pay for itself until
F1 and F2 are fixed.

### 3c. CPU → Mac peripheral (VIA, SCC, SCSI, ASC…)

| # | Hop | Width | Domain | Cost |
|---|---|---|---|---|
| 1 | `dcache` non-cacheable bypass FSM | 32b | core | Parks in `S_BY_LD_R` for the whole round trip — `dcache.v:801, 1467`. **Blocks all data memory access, including DDR hits.** |
| 2 | `axi_narrow_to_wide` | 32→128b | core | 1 outstanding |
| 3 | `axi_xbar` M0 slot → S1 | 128b | core | **Same `rs_state[0]` / `ws_state[0]` as DDR traffic** (`axi_xbar.v:3208` vs `:3212`) — head-of-line blocking across slaves |
| 4 | `u_pb_s1_cdc` AR (`ar_fifo`) | **128b** → **32b** (Phase 1a) | core→pb | **5.5 core** (3 pb edges) — unchanged by 1a; AR carries no data bits |
| 5 | `peripheral_bus` FSM | 128b→8b | pb 50 MHz | 2 pb = **4 core** |
| 6 | `via1.v:725` ack | 8b | pb | 1 pb = **2 core**. **No wait states at all** — `pb_ack <= pb_wr \| pb_rd`, unconditional |
| 7 | `u_pb_s1_cdc` R (`r_fifo`) | 128b | pb→core | **2.5 core** (3 core edges) |

**Measured: 14.0 core cycles (140 ns), xbar-S1 AR accept → S1 RVALID.**
Confirmed by compiling the real `axi_async_bridge` at 128b/ID6 against a
cycle-accurate model of the VIA1 read FSM. **8.0 of those 14 cycles (57%)
are CDC.** Back-to-back VIA reads issue one every 4 pb_clk = 8 core
cycles (`peripheral_bus.v:1571`, `s_arready = !rd_busy`).

Adding the D-cache, `axi_narrow_to_wide` and crossbar hops puts a
CPU-observed VIA access at roughly **20-25 core cycles**. That is ~5×
*faster* than real Q700 silicon (~1.28 µs). **The peripheral bus itself is
not the problem and should be left alone** — it has zero wait states and
its 3-cycle FSM is entirely reasonable.

The problem is hops 1 and 3: **a single VIA register poll costs the CPU
its entire data-memory port for ~20-25 cycles**, because
`dcache.v:899` refuses every subsequent data access — including D-cache
hits — until the bypass access retires, and `axi_xbar.v:3208/:3212` gate
S0 and S1 reads on the same `rs_state[0]`. System 7 polls VIA IFR in
loops and bit-bangs the RTC through VIA1 port B one bit at a time.

One notable waste: hop 4 carries a **128-bit** payload across the CDC to
reach an **8-bit** register. That is why `u_pb_s1_cdc` is 771 LUT and why
`u_pb_s1_cdc/ar_fifo`, `aw_fifo` and `w_fifo` appear **131 times** as
path endpoints in `synth/timing_reports/timing_20260819_131707.rpt`. A
32-bit CDC would be a quarter of the flops and routing for identical
function.

---

## 4. Measured results

Reproduce with (scratchpad build — no repo file was modified):

```
verilator --cc --exe --build --assert -O3 -Irtl/soc -Irtl/board \
  -GSTALL_ENABLE=1 -GDDR_READ_LATENCY_BASE=40 -GDDR_READ_LATENCY_JITTER=8 \
  --top-module tb_l2c_chain <L2C_CHAIN_RTL from Makefile:5828> tb_hit.cpp
```

`tb_hit.cpp` is a scratchpad copy of `tb/tb_l2c_chain.cpp` with one added
scenario that warms 32 L2 lines and then times three access patterns.

> **Note on the DDR model.** `tb/tb_l2c_chain.v:45-46` defaults to
> `DDR_READ_LATENCY_BASE = 200, JITTER = 64`, roughly 4-5x pessimistic
> against real DDR4. That setting flatters the fabric, because it buries
> fabric overhead under an unrealistic DRAM latency. Every figure below
> was taken at 40 ± 8. Note also that `docs/l2c_spec.md:484-486` and
> `docs/dma_engine_design.md:62-64` still quote "66 vs 204 cycles,
> 3.09x", which predates the latency model entirely and should be
> retired.

### 4a. Miss concurrency (`concurrent_fill_overlap`)

| DDR model | 8 concurrent | 8 sequential | cyc/miss (conc.) | cyc/miss (seq.) | speedup |
|---|---|---|---|---|---|
| 200 ± 64 (repo default) | 318 | 2106 | 39.8 | 263.3 | 6.62× |
| **40 ± 8 (realistic)** | **110** | **546** | **13.8** | **68.3** | **4.96×** |

The harness asserts `MIG queue peak = 8` and `ARs before first R = 8` in
both runs: all 8 fills genuinely reach the MIG before the first response
returns. The capability is real, implemented, verified, and 20,953 LUT
were spent on it.

Subtracting the ~44-cycle average modeled DRAM latency from the
68.3-cycle sequential figure leaves **~24 cycles of L2 + CDC + bridge
overhead per 64 B fill**, measured at the L2 slave port and therefore
*excluding* the crossbar and the CPU-side adapters. Adding those puts a
CPU-observed clean L2 miss at roughly **72-80 cycles**.

### 4b. Hit throughput — the finding that reframes everything

Two measurements, taken hours apart, because `rtl/soc/l2c_ctrl.v` was
being modified by another agent during this review. Both are real; the
delta is instructive.

| Access pattern | before (`HEAD`) | after (working tree) | change |
|---|---|---|---|
| Sequential single-beat hits, issue+wait | 5.12 cyc | 5.22 cyc | — |
| **Pipelined** single-beat hits | 4.22 cyc | **3.09 cyc** | **−27%** |
| 4-beat 64 B burst hit | 17.53 cyc (4.38/beat) | **14.16 cyc (3.54/beat)** | **−19%** |
| Best sustained hit bandwidth | 3.79 B/cyc | **5.17 B/cyc** | **+36%** |

The working-tree change removes the `S_HITRESP` state from the front-door
FSM (`l2c_ctrl.v:123-130`), collapsing the hit path from four states to
three. **That is the right direction and it is worth 36%.** It is also
evidence for the central claim of this review: the front door, not the
requester and not DRAM, sets the ceiling — a single state removed from it
moved system bandwidth by more than a third.

Three conclusions survive the change:

1. **Bursting still does not help.** 3.54 cyc/beat inside a burst vs 3.09
   cyc/beat for pipelined singles — a burst is *worse per byte*. The
   front door charges per beat (`l2c_ctrl.v:332`,
   `ar_beat <= ar_beat + 8'd1`), so a 4-beat burst is four full trips
   round `S_IDLE → S_WAIT → S_LOOKUP`. This is structural, not tuning.
2. **The requester cannot fix it.** Issue-and-wait costs 5.22 cyc;
   pipelining to the maximum the port allows gets 3.09. Everything below
   3 cycles/beat is unreachable without pipelining the lookup itself.
3. **The ceiling is now ~5.2 B/cycle ≈ 520 MB/s at 100 MHz**, shared by
   CPU I-fetch, CPU data, JTAG debug and (on `HEAD`) the RAM disk.

### 4c. The comparison that should not be true

| | bytes | cycles | B/cycle |
|---|---|---|---|
| 8 concurrent L2 **misses** (to DRAM) | 512 | 110 | **4.65** |
| L2 **burst hits** | 64 | 14.16 | 4.52 |

**At full miss concurrency the L2 returns data from DRAM at essentially
the same rate it returns data from its own arrays.** The cache is not, at
the margin, buying bandwidth — only latency. That is the single clearest
statement of what is wrong, and it is measured, not argued.

---

## 5. Clock domains and CDC cost

There are **four** clock domains in the shipped bitstream, but only
**three genuinely asynchronous boundaries**, and this distinction turns
out to matter:

| Domain | Freq | Source | Async to core? |
|---|---|---|---|
| `core_clk` | 100 MHz | `fabric_clk_odiv2` → BUFG_GT DIV=0 | — |
| `pb_clk` | 50 MHz | **same** `fabric_clk_odiv2` → BUFG_GT DIV=1 | **NO — mesochronous** |
| `mig_ui_clk` | 333.25 MHz | MIG's own PLL | yes |
| `pclk` | 148.5 MHz | video MMCM | yes |

`rtl/soc/fpga_top_clocks.vh:135-163` derives `core_clk` and `pb_clk` from
one BUFG_GT source, and `synth/vivado.tcl:1590` puts them in the **same
clock group** — they are never declared asynchronous, and `vivado.tcl:1530`
says so explicitly ("real, related fabric clocks and meet timing fine
when checked synchronously").

### Measured CDC cost per bridge

All three AXI CDC bridges are the same module, `axi_async_bridge` — five
independent gray-pointer `async_fifo`s, one per AXI channel
(`axi_async_bridge.v:11-25`). **This is the right structure**: it is not a
handshake, so a burst pays the crossing once, not per beat. Verified
directly — an 8-beat burst returned beats at 20 ns spacing, exactly the
producer domain's rate, i.e. zero per-beat CDC penalty.

Latency is **3 destination-clock edges** per channel, not the textbook 2,
because `async_fifo.v:656-662` registers the `empty` flag on top of the
two `ASYNC_REG` gray-pointer stages. AW pays a 4th (`:264-296`, an
explicit 333 MHz Fmax trade).

| Bridge | Domains | Round trip (measured) | Verdict |
|---|---|---|---|
| `u_core_to_mig_ui` | core → mig_ui | **3.0 core cycles** | Correct. Negligible against DRAM latency. |
| `u_pb_s1_cdc` | core → pb | **8.0 core cycles** (57% of a 14-cycle VIA read) | Over-provisioned — see below |
| `u_dbg_pb_to_core` | pb → core | 4.0 pb cycles | Debug only |
| `u_pram_cdc` | core ↔ pb | **16 core cycles PER BYTE** | 4-phase handshake; JTAG-only, acceptable |

### The finding: `u_pb_s1_cdc` is a fully asynchronous FIFO across a
### synchronous 2:1 boundary

`u_pb_s1_cdc` carries a **128-bit** payload, through five gray-pointer
async FIFOs with a full T6 coupled-reset handshake, across a boundary
Vivado already times synchronously, to reach an **8-bit** register.

It costs 771 LUT, contributes 8 of the 14 cycles of every Mac peripheral
access, and `u_pb_s1_cdc/{ar,aw,w}_fifo` appear **131 times** as timing
path endpoints in `synth/timing_reports/timing_20260819_131707.rpt` —
`w_fifo` alone is the 10th-largest path-endpoint owner in the design, on
a build that is failing timing at −0.612 ns.

A related-clock 2:1 boundary needs a rate-adapting elastic buffer, not
this. **Caveat, and it is a real one:** the T6 reset handshake also exists
to survive one-sided JTAG debug resets, which a naive elastic buffer would
not. That capability must be preserved by any replacement — see §9.

### Two latent constraints worth writing down

- **`REQ_MIN_HOLD = 8` bounds the usable clock ratio to about 1:4**
  (`async_fifo.v:472-487`). `core_clk : mig_ui_clk` is 1:3.33 — inside
  the bound, with ~20% margin. Dropping core_clk to 50 MHz (a plausible
  response to the new core failing timing) would make it **1:6.7 and
  break the reset handshake.** This deserves an elaboration-time
  assertion, not a comment.
- **`scanout_placement_sync.v`** crosses a **75-bit** geometry tuple from
  core_clk to pclk as seven independent 2-FF synchronizers plus a
  two-samples-agree filter (`:143-154`, `:422-469`). It is defended by a
  downstream validity conjunction and an atomic frame-boundary commit,
  and the module is honest about this — but it is a heuristic, not a
  proof, and it is the weakest CDC construction in the tree.

---

## 6. Footguns, ranked by measured or derived cost

### F1 — One outstanding transaction per crossbar master port
**Cost: up to 4.96× on miss-bound code (measured). Severity: critical.**

`axi_xbar.v:3208-3235` gates every read request on
`rs_state[mi] == RS_IDLE`; `axi_xbar.v:2090-2115` does the same for writes
on `ws_state[mi] == WS_IDLE`. Both arrays are indexed by *master slot*
(`:3100`, `:1509`), and `RS_IDLE` is not re-entered until `RLAST` has been
accepted (`:3698-3700`). A master cannot have a second transaction in
flight, to any slave, ever.

The design considered fixing this and declined, with a stated reason —
`axi_xbar.v:1634-1653`: write-side AW pipelining "was analyzed and
deliberately NOT implemented … B responses can complete out of order,
which violates AXI same-ID B ordering for a master reusing one AWID (the
LSU does) unless the xbar reorders B."

**That reasoning is correct for writes and does not transfer to reads.**
AXI4 requires in-order completion only *per ID*, and the crossbar already
routes R by ID (`s0_rtgt = s0_rid[XID_WIDTH-1 -: SLOT_W]`, `:3408`). A
read-side fix that allows N outstanding reads per master **with distinct
IDs** needs no reordering logic at all. The write-side objection stands
and should be respected.

### F2 — The L2 front door is beat-granular and globally single-AR
**Cost: caps all DDR-side bandwidth at ~5.2 B/cycle (measured). Severity: critical.**

Two separate serialisations in one module:

- **Beat granularity.** `l2c_ctrl.v:130` declares one `st` register for
  the whole cache. Every *beat* walks `S_IDLE → S_WAIT → S_LOOKUP`
  (`:307-380`), with `ar_beat <= ar_beat + 8'd1` at `:332`. A 4-beat
  burst is four full trips. Measured: 3.54 cyc/beat in a burst, vs 3.09
  cyc/beat for pipelined singles — **bursting is worse per byte.**
- **Global single-AR.** `l2c_ctrl.v:119` —
  `assign s_arready = !ar_have && !rst_busy`. `ar_have` is set on AR
  accept (`:322-325`) and cleared only when the burst's **last** beat is
  consumed (`:332`). For the whole decomposition of a burst, the L2
  accepts **no other AR from any master**.

That second point undoes the crossbar's one genuinely good decoupling.
The xbar gives I-fetch its own slot (M2, `rs_state[2]`) so it is not
blocked by a data-side peripheral access — but both masters converge on
S0, and there they re-serialise against each other at `s_arready`. **CPU
instruction fetch and CPU data access are effectively serialised at the
L2 whenever either one is doing a burst.**

`docs/l2c_spec.md:274-277` states the intent plainly: "throughput is
correctness-first, not maximised, for this task." That was right for v1.
It is now the ceiling on the whole system. The in-flight `S_HITRESP`
removal (§4b) is the correct first step and is worth 36%; the remaining
3-cycle floor and the global `ar_have` gate are the rest of it.

### F3 — Fill concurrency is throttled to 2 by the VRAM lane mux
**Cost: ~4× on DDR read bandwidth (derived). Severity: high. Cheapest fix here.**

`axi_vram_priority_mux3.v:100` — `parameter MAX_BULK_AHEAD = 2`, enforced
at `:240` (`bulk_admit = (bulk_count < MAX_BULK_AHEAD)`) and `:262`.
"Bulk" is `l2c` *and* the S3 VRAM write lane combined. The MSHR issues 8;
the MIG bridge accepts 8; this mux allows **2**.

Peak DDR read bandwidth, derived at a ~49-cycle round trip:

| Configuration | Outstanding | Bandwidth |
|---|---|---|
| **Production (`MAX_BULK_AHEAD = 2`)** | 2 | **≈ 261 MB/s** |
| Throttle lifted to 8 | 8 | ≈ 1.04 GB/s |
| 128b core-clk port ceiling | — | 1.6 GB/s |
| DDR4-2400 ×32 raw | — | 9.6 GB/s |

**The DDR path currently delivers ~2.7% of the DRAM's raw bandwidth and
~16% of its own port ceiling.** The parameter's comment justifies 2 as
protecting "scanout's buffering slack" — a real concern, but scanout
*already* holds unconditional AR priority (`:252-256`) and has a separate
`MAX_SCAN_AHEAD = 4` forward-progress quota. The bound is static where it
should be responsive to the scanout ring's actual occupancy.

### F4 — The same-ID accept gate silently serialises single-ID masters
**Cost: MEASURED 6.6× (8.38 → 55.40 cyc/op). Severity: critical once the new core lands, and invisible to every existing test.**

**Measured** by the L2C agent, 8 outstanding misses through the L2:

| ID discipline | cyc/op | vs. best |
|---|---|---|
| 8 unique ARIDs | **8.38** | 1.0× |
| 1 reused ARID | **55.40** | **6.6× worse** |


`l2c_ctrl.v:155` —
`id_busy_c = mshr_idq_busy || (byp_active_valid && byp_active_id == cur_id)`;
`docs/l2c_spec.md:463-470` confirms a new front-door accept is held off
whenever its AXI ID matches any live MSHR or bypass op. "**Cross-ID
ordering remains unconstrained by design.**"

Today this gate costs nothing, because everything upstream is already
1-outstanding — so it has never been observed and **no test would catch
it.** The crossbar composes IDs as `{slot[1:0], master_id[3:0]}`
(`axi_xbar.v:3752`), so the DDR-facing masters do differ from each other.

But `axi_narrow_to_wide` and `if_to_axi` each use a **fixed** `ID_TAG`
for all their traffic. **The moment F1 is fixed, a core that reuses a
single ID will measure 6.6× worse than one that rotates** — and the
natural conclusion will be "multi-outstanding didn't help", which is the
wrong conclusion drawn from a correct measurement. This belongs in the
socket contract, not in a postmortem — see §7a.

### F5 — Head-of-line blocking across slaves, twice, nested
**Cost: ~20-25 cycles of total data-memory stall per peripheral access. Severity: high.**

- `cpu/rtl/core/mem/dcache.v:899` — `req_ready = (state == S_IDLE) && !cooldown`.
  A non-cacheable I/O load parks the FSM in `S_BY_LD_R` (`:801, 1467`)
  for the full round trip, refusing **all** subsequent data memory
  operations, including D-cache hits. No hit-under-miss.
- `axi_xbar.v:3208` vs `:3212` — `req_rd_ddr[mi]` and `req_rd_io[mi]`
  share the same `rs_state[mi]`. A read to S1 blocks that master's reads
  to S0.

The outer one binds. For System 7's VIA polling loops and its bit-banged
RTC access this is the dominant cost, and it is architectural rather than
a wait-state artifact. **This one is fixed in the CPU, not the fabric** —
worth stating clearly so it lands in the right repo's backlog.

### F6 — The RAM disk streams through a single-outstanding bypass engine — **RETIRED (Phase 1b)**
**Was: 19× throughput loss and a 14-21 µs CPU read blackout. Severity: now latent-only.**

**Closed by the coordinator's `ENABLE_DDR_RAMDISK` work** (Phase 1b): the
DDR RAM disk is compiled out and `BYP_WIN_EN` is gated to match, so the
single-outstanding bypass engine has **no consumer in the shipped
bitstream** and `l2c_bypass`'s 213 LUT go with it. The analysis below is
retained because it is the precondition anyone re-enabling a bypass
window — direct-color scanout being the obvious candidate — needs to
read first.

`fpga_top_ddr.vh:142-149` enables the L2 bypass window for the RAM disk
(a 256 MB window on the flattened `0x5xxx_xxxx` range,
`axi_defs.vh:213-214`). `l2c_bypass.v:10-18` processes requests "one at a
time, strictly in front-door acceptance order", single-beat
(`len=0, size=4`).

`vhdd_ddr.v:243-248` issues 512 B blocks as one 32-beat INCR burst. The
L2 front door shreds that into **32 separate single-beat bypass ops**,
each costing RTT + 5 cycles:

| RTT | cycles/512 B block | throughput |
|---|---|---|
| 40 | 1,440 | 35.6 MB/s |
| 60 | 2,080 | 24.6 MB/s |

versus ~683 MB/s if the 32-beat burst went through intact — **a ~19×
loss**. And because `s_arready = !ar_have` (F2), those 1,440-2,080 cycles
are a **contiguous window in which no CPU read is accepted at all** —
14-21 µs of total CPU read blackout per 512 B block, at roughly 3-11%
duty cycle.

**Three things keep this off the critical list.** First, the throughput
loss is masked ~10× over: the Mac side moves every SCSI byte as
programmed I/O through the 53C96 pseudo-DMA handshake at ~1-3 MB/s
(`rtl/mac/glue.v:219-225`), and `vhdd_ddr.v:149`'s `WR_BYTE_GAP = 16`
deliberately paces writes to 3.1 MB/s. Even at 25 MB/s the bypass engine
is 5-18× a real Q700's SCSI bus. Second, the working tree has just
compiled the whole path out behind `ENABLE_DDR_RAMDISK`
(`fpga_top_peripherals.vh:1973-2001`, uncommitted). Third, that macro is
defined nowhere in `Makefile` or `synth/vivado.tcl`.

`docs/l2c_spec.md:432-441` already warns that bandwidth-hungry consumers
need "their own multi-outstanding bypass engine". **Do not fix this now.
Do record that the reason to fix it, if the RAM disk returns, is the
14-21 µs read blackout — not the throughput.**

### F6b — The CPU-side adapters are one-outstanding by construction
**Cost: caps the new core at 1 in flight regardless of the crossbar. Severity: critical (prerequisite).**

Two modules sit between the CPU socket and the crossbar, and **both are
hard-limited to one transaction**:

- `rtl/soc/if_to_axi.v:16-18` — instruction fetch, `arlen=0`, one AR at a
  time, no queue. A prefetching front end gets nothing.
- `rtl/soc/axi_narrow_to_wide.v:72-76` — data side, "one outstanding
  transaction per direction", `n_arready = !ar_valid_q` (`:80-83`).

Both cite core self-serialisation as the reason (quoted in §1), and both
are correct *today*. Neither is parameterised over outstanding depth, and
`axi_narrow_to_wide.v:74-76` explicitly defers the work: "Widening the
number of *concurrent* transactions is a separate (and much larger)
change — see docs/memory_model_review.md §1."

Note also that these two files exist in **two byte-identical copies** —
`rtl/soc/` and `cpu/rtl/sys/` — and only the `rtl/soc/` copies are
compiled (`Makefile:1687` says so; `cpu/rtl/sys` is an include path only,
and `Makefile:2535` carries an rsync hint to keep them in sync). Editing
the `cpu/` copy — which the socket documentation's "the shims relocate
CPU-side (Phase 4)" wording invites — **silently has no effect on the
build.** They are in sync today; this is a hazard, not a current bug.

### F7 — Fixed-width hand-written arbitration, with all four slots full
**Cost: engineering time, not cycles. Severity: medium, rising to blocking.**

`axi_xbar.v:772-773` — `localparam NW = 4; NR = 4;` with
`rr_pick_w`/`rr_pick_r` hardcoded to four named inputs (`:2009-2027`,
`:3370-3388`) and six-way slave dispatch written longhand at every site
(`:1974-1979`, `:2528-2593`, `:3640-3695`, `:3752-3840`). The header at
`:71-79` says this was deliberate: "a deliberate choice to avoid resizing
load-bearing, hand-written round-robin arbitration logic that is not
parameterized over width."

All four read slots are occupied on `HEAD` (M0 LSU, M1 debug, M2 IF, M3
vhdd_ddr — `axi_xbar.v:100-120`). **There is no fifth slot**, and the
uncommitted RAM-disk removal is explicitly motivated by wanting M3 back
(`fpga_top_peripherals.vh:1982-1987`). A new core wanting separate
load / store / fetch / table-walk ports cannot be accommodated without
rewriting this by hand.

### F8 — A 128-bit CDC to reach an 8-bit register — **FIXED (Phase 1a)**
**Was: 771 LUT and 131 timing-path endpoints. Severity: medium (timing).**

See §5. `fpga_top_peripherals.vh` instantiated `u_pb_s1_cdc` at
`DATA_WIDTH(128)` while everything downstream is 8 bits; the W FIFO alone
was 16 × 128 bits. On a build failing timing at −0.612 ns, that was 131
path endpoints spent on a 4× width mismatch.

**Closed this session** — the CDC now carries a 32-bit payload
(`axi_pb_s1_cdc`), removing ~95 of the 131 endpoints and an estimated
~200 LUT net. Implementation, evidence and the equivalence testbench are
in §8, Phase 1a. Reverting is one define (`PB_S1_WIDE_CDC`).

The residual, and it is the bigger prize: **§5's finding that this is a
fully asynchronous FIFO across a *synchronous* 2:1 boundary still
stands.** `core_clk` and `pb_clk` are BUFG_GT divisions of one source and
sit in the same Vivado clock group. Phase 1a made the crossing cheaper;
it did not remove a crossing that arguably should not be asynchronous at
all. That is Phase 3b, and it is gated on preserving the T6 one-sided
reset recovery.

### F9 — Stale documentation that will mislead the next reader
**Cost: wasted investigation time. Severity: medium — this is how a fabric redesign goes wrong.**

Found while verifying the above; none of it was corrected, because
correcting it was not this task:

| Doc | Claim | Reality |
|---|---|---|
| `docs/memhier.md:305, 676, 762-770, 834` | "VRAM lives in URAM, not DDR"; DDR-backed VRAM "not scheduled" | `VRAM_IN_DDR ?= 1` in `Makefile:2267`; the URAM instance is not even elaborated (`fpga_top_video.vh:184`) |
| `docs/clocking.md:37` | `clk_ddr_ui` = 300 MHz | 333.25 MHz (`synth/gen_ddr4_mig.tcl:153`) |
| `docs/clocking.md:526-529` | "MIG provides its own CDC, so we do NOT insert our bridge here" | `ddr_ctrl.v:779` instantiates `u_core_to_mig_ui` on the production path. Paths cited (`rtl/sys/…`) no longer exist. |
| `docs/l2c_spec.md:484-486`, `docs/dma_engine_design.md:62-64` | "66 vs 204 cycles, 3.09×" | 110 vs 546, 4.96× (measured) — the quoted figure predates the latency model |
| `axi_xbar.v:9-18` | DMA M4 "stubbed" | `dma_ctrl` deleted entirely 2026-08-01; slot 3 reassigned to `vhdd_ddr` |
| `docs/ddr4_mig_bridge_contract.md:85-91` | rejected read returns one truncated SLVERR beat | RTL returns the full burst, SLVERR on every beat (`axi_ddr4_mig_bridge.v:1514-1530`) |
| `docs/memhier.md:518` vs `scanout_ddr_reader.v:77` | "L2 miss ~20 cycles" vs "~40-cycle assumption" | Mutually inconsistent; **neither is measured** |

The last row is the important one. **There is no measured real-hardware
DDR round-trip latency anywhere in this repo.** `ddr_ctrl.v:190-191`
exposes handshake counters but no latency counter. Every bandwidth and
scanout-margin claim in the tree rests on an unverified 40-cycle
assumption that `scanout_ddr_reader.v:77-80` is honest enough to flag as
such. At 60 cycles, the 1024×768×24bpp scanout mode does not close
(189 MB/s demand against a 180 MB/s ceiling).

---

## 7. What breaks when `m68k-core-040-ooo` arrives

The socket (`rtl/soc/cpu_socket.vh`) is in good shape: three AXI
interfaces, `CPU_SOCKET_AXI_DW = 128` native on both masters,
`CPU_SOCKET_AXI_IW = 4` (16 IDs per master), and an explicit `{128, 256}`
width contract with 256 reserved. No m68k-specific signal crosses it. The
Phase-4 refactor that moved `if_to_axi` and `axi_narrow_to_wide` CPU-side
was the right call and makes the core swap tractable.

What the new core will step on, in the order it will hit them:

| # | What the core does | What happens | Silent? |
|---|---|---|---|
| 1 | Issues N concurrent L1 misses from N MSHRs | Crossbar accepts **1** and refuses the rest (`axi_xbar.v:3208`). The core stalls at its own AXI port. | No — visible as low IPC |
| 2 | Fix #1, but reuse one ARID | L2's same-ID gate (`l2c_ctrl.v:155`) serialises them anyway. **Measures as zero improvement.** | **YES** |
| 3 | Issues wider bursts (e.g. 64 B lines, `arlen=3`) | L2 charges 3.5 cyc/beat, so a 4-beat burst costs 14 cycles and blocks every other master's AR for all 14 (`l2c_ctrl.v:119`). Wider lines make head-of-line blocking *worse*. | **YES** |
| 4 | Adds a 4th master port (e.g. table-walk) | **No slot exists.** `NW = NR = 4`, all occupied, arbiter not parameterised (`axi_xbar.v:772-773`). | No — fails to elaborate |
| 5 | Widens to `AXI_DW = 256` per the socket's reserved option | Nothing downstream supports it: xbar is `DATA_WIDTH` 128 throughout, L2 is 128 (`l2c.v` default), the MIG bridge converts 128→256. A 256b core needs a new narrowing shim, and gains nothing — the L2 front door is beat-rate-limited, so doubling beat width halves the beat count but the 3-cycle-per-beat charge is what dominates. | **YES** — looks like a win, delivers ~0 |
| 6 | Prefetches instructions ahead | `if_to_axi` is 1-outstanding by construction (`cpu/rtl/sys/if_to_axi.v:16-18`), and it is CPU-side, so this is fixable in the core repo — but the xbar M2 slot is also 1-outstanding, so fixing it CPU-side alone achieves nothing. | **YES** |
| 7 | Is physically larger than 107,855 LUT | Design is at **187,011 / 216,960 = 86%** with congestion level 6 and WNS **−0.612 ns**. `u_l2c/g_active.u_ctrl` already appears in 3 of 5 placer congestion windows at 11-13%, physically interleaved with `u_cpu/u_commit` and `u_cpu/u_rob` (`congestion_route.rpt`). | No — fails to route |

Row 7 is the one that constrains everything else in this document.

> **On "spending area to fix the fabric is affordable":** it is *not*,
> unconditionally. At 86% LUT occupancy, congestion level 6, and three
> builds failing timing, the design has no slack. The interconnect is
> cheap in absolute terms, but every LUT added to `u_l2c` lands in the
> *same congestion window* as the CPU's ROB and commit logic. Area is
> affordable only if the new core is smaller than the old one, or if the
> additions are placed away from that window. Proposals below are costed
> with that in mind, and the cheapest ones are cheapest precisely because
> they *remove* logic.

Five of the seven rows are marked **silent**: they cost performance
without failing any test. That is the defining property of this fabric's
problems, and §9 deals with it directly.

---

## 7a. Socket contract for `m68k-core-040-ooo`

Two obligations, one on each side. Neither is enforced by anything today,
and violating either is silent.

### What the new core MUST do

**Rotate its AXI IDs.** Both masters have `CPU_SOCKET_AXI_IW = 4`
(`cpu_socket.vh`), i.e. 16 IDs each — ample. The rule:

> A core with N transactions in flight must present them under N
> *distinct* ARIDs. Reusing one ARID across concurrent misses costs
> **6.6×** (measured: 8.38 → 55.40 cyc/op) and no test will fail.

Concretely: tag each L1 MSHR / fill buffer with its index and drive that
index as the low ARID bits. This is nearly free in the core and is the
difference between the fabric work paying off and appearing not to.

It follows that `if_to_axi`'s `ID_TAG` and `axi_narrow_to_wide`'s
`ID_TAG` parameters — fixed constants today — must become per-transaction
ID *pass-through*.

### What the fabric SHOULD do, so a non-rotating core is merely slow

A core that does not rotate IDs should degrade to today's throughput, not
worse, and should be *diagnosable*. Three cheap measures:

1. **Never let same-ID cost more than serialisation.** The L2's gate
   (`l2c_ctrl.v:155`) is conservative — it blocks a fresh accept on *any*
   live same-ID op rather than only ops that would race a different MSHR
   entry (`docs/l2c_spec.md:470-473` admits this). Narrowing it to the
   actual hazard turns the 6.6× cliff into a much gentler slope. Owned by
   the L2C agent; described, not implemented here.
2. **Make the cliff visible.** Expose a "front-door accepts blocked by
   same-ID" counter next to the existing hit/miss counters. Without it,
   the failure mode is indistinguishable from "the core is slow".
3. **Assert it in simulation.** A tb that drives N concurrent misses under
   one ID and checks `mshr_occupancy` never exceeds 1 would have caught
   this class permanently. See §9/S1.

---

## 8. Costed redesign proposal

Sequenced so that each phase is independently landable and independently
measurable. **Phase 1 exists because Phase 2 is unmeasurable without it.**

### The sequencing trap, stated first

The three concurrency limits are **in series**:

```
CPU port          L2 front door         VRAM lane mux
1 outstanding  →  1 AR globally    →    2 bulk ahead    →  MIG (8)
   (F1)              (F2)                  (F3)
```

Fixing any **one** of them moves the system by approximately nothing,
because the next one downstream immediately binds. A team that lands F1
alone, measures no change, and concludes "the crossbar wasn't the
problem" will have drawn a false conclusion from a correct measurement —
and will likely not revisit it. **F1, F2 and F3 must be landed and
measured as one change, or landed separately but measured only at the
end.** This is the single most important process point in this document.

### Phase 0 — Instrumentation (prerequisite, ~300 LUT, no timing risk)

| Item | Change | Area | Risk |
|---|---|---|---|
| **0a** | Expose `dbg_l2c_hit_count` / `miss_count` / `mshr_occupancy` as AXI-readable debug registers. They are currently ILA probes only (`fpga_top_debug_vio.vh:169-182`), so reading them needs a capture session. | ~100 LUT | None |
| **0b** | Add a DDR round-trip latency counter to `ddr_ctrl.v` (AR-accept → first R, min/max/accumulator). `ddr_ctrl.v:190-191` has handshake counters but no latency counter. | ~200 LUT | None |
| **0c** | Change `tb/tb_l2c_chain.v:45-46` defaults to a realistic latency, or add a make variable. The current 200±64 makes the fabric look ~4× better than it is. | 0 | None |

**Why this is Phase 0 and not an afterthought:** every performance claim
in this repo — including the ~40-cycle DDR RTT that every scanout margin
depends on — is currently an unverified assumption
(`scanout_ddr_reader.v:77-80` says so explicitly). Phase 0 costs ~300 LUT
and converts the rest of this plan from argument into measurement.

#### Measuring the real DDR round trip on hardware — no rebuild needed

The 40-cycle figure everything depends on is an assumption
(`scanout_ddr_reader.v:77-80` says so). Phase 0b adds a proper counter,
but that needs a bitstream, and builds are queued. **This can be measured
today with the board that is already running**, using only existing JTAG
REPL commands.

The enabling fact: `debug_ctrl.v:616-617, 1537-1538` already exposes a
**64-bit free-running cycle counter** at `OFF_CYCLE_LO`/`OFF_CYCLE_HI`,
i.e. **`0x5090_1000` / `0x5090_1004`** in the debug window.

Procedure — a pointer-chase, which is the right shape because F1/F2 mean
the fabric delivers exactly one outstanding miss anyway, so this measures
what the machine actually does:

1. Hold the CPU: `/tmp/jcmd.sh "reset hold"`.
2. Build a dependent-load chain in RAM with `w` commands: at each of N
   addresses strided by **64 B** (the L2 line size, so every hop is a new
   line), store the address of the next hop. Stride across **more than
   the L2's capacity** so every load misses; walk the region in a
   pseudo-random order so no prefetcher or open-row effect helps.
3. Write a short m68k loop that chases the chain K times
   (`MOVE.L (A0),A0`, decrement, branch) and ends in `bra .`.
4. Read the cycle counter: `/tmp/jcmd.sh "r 0x50901000"` (and `...1004`).
5. `/tmp/jcmd.sh "reset-and-halt-after <n>"` for an instruction count that
   completes the loop, then read the counter again.
6. **`(cycles_after − cycles_before) / K` = end-to-end load-to-use latency
   for one L2 miss**, in core_clk cycles.
7. Repeat with the chain rewritten to point every hop at the **same**
   cache line. That run is all L2 hits, and should land near the measured
   ~3.1-5.2 cycles from §4b. **The difference between the two runs is the
   DRAM + MIG + CDC component** — the number this repo has never had.

Cross-check, free: the value should be consistent with the boot_fsm zero
pass, which is a 256 MiB single-outstanding write stream and takes ~3.7 s
of the measured 4.06 s reset-to-`rom_loaded` — i.e. ~22 cycles per 16 B
write. Reads should be slower than that (writes retire on B, not on
data), so **a read result below ~25 cycles or above ~90 would indicate a
methodology problem, not a slow DRAM.**

Caveats worth stating before anyone acts on the number: the counter is in
the CPU's debug domain, so it counts core_clk and is unaffected by the
MIG's own clock; and the result includes the D-cache, `axi_narrow_to_wide`
and crossbar hops, so it is the **CPU-observed** latency, which is the
useful one for §3a, not the isolated DRAM figure. Subtract the ~24 cycles
of L2-and-below overhead measured in §4a to recover the latter.

### Phase 1 — Free wins (net area **negative**, timing risk **negative**)

| Item | Change | Area | Timing risk |
|---|---|---|---|
| **1a** | **DELIVERED this session** — narrow the S1 CDC payload to 32 bits. See the implementation note below. | **−185 to −225 LUT (est.)** | **Improves** — removes ~95 of 131 path endpoints from a failing build |
| **1b** | **DONE (by the coordinator)** — DDR RAM disk compiled out behind `ENABLE_DDR_RAMDISK`, with `BYP_WIN_EN` gated to match. | **−1,550 LUT** (`u_vhdd_ddr` 665 + `u_vhdd_ddr_cdc` 672 + `l2c_bypass` 213), frees xbar slot M3 | None |
| **1c** | Correct the stale docs in §6/F9. | 0 | None |

> **Correction to an earlier draft of 1b.** It recommended deleting
> `scsi_trace_ring.v`. **Do not** — `tb-scsi-trace-ring` builds against
> it and it is one port-list away from being re-instantiated. It is
> already out of the *build*, which is the part that mattered; the LUT
> saving was already counted, and deleting the source would only lose a
> tested module. The corrected 1b figure above (−1,550 LUT) is the
> RAM-disk removal alone, and it also retires footgun **F6** outright:
> with `BYP_WIN_EN` gated off, the single-outstanding bypass engine has
> no consumer in the shipped bitstream.

#### Implementation note — Phase 1a as landed

Two new modules, both **purely combinational** (no state, no reset, no
ordering assumption):

- `rtl/soc/axi_pb_lane_shim.v` — `axi_pb_lane_narrow` (128→32, recovers
  the lane from WSTRB) and `axi_pb_lane_widen` (32→128, replicates W into
  all four lanes; OR-reduces R, which is exact because
  `peripheral_bus.v:1573-1576` hard-zeroes the unselected lanes).
- `rtl/soc/axi_pb_s1_cdc.v` — port- and parameter-compatible drop-in for
  `axi_async_bridge`, sandwiching **the same bridge** at `DATA_WIDTH(32)`
  between the two shims.

**`axi_wide_to_axilite` was evaluated and rejected**, despite being the
"already proven on three other slave ports" option, for two reasons found
by reading it: (i) it converts to AXI-Lite, which **drops the AXI ID**
that `axi_xbar.v:3408` needs to route S1's R beats back to the right
master slot; (ii) it is one-outstanding, which would have defeated the
CDC's 4-deep AW/AR pre-staging. Keeping full AXI4 with IDs was the
correct construction and cost two ~150-line combinational modules.

**Revised area estimate, and why it is lower than this document
originally claimed.** The original −400 to −500 LUT assumed the bridge's
771 LUT scaled with payload width. It does not: only `w_fifo` (272 LUT)
and `r_fifo` (131 LUT) are width-dependent; the rest is gray-pointer,
reset-handshake and `REQ_MIN_HOLD` control logic that does not shrink.
Payloads fall 146→38 and 138→42 bits, so those two FIFOs should drop to
~110-120 LUT combined, and the shims cost ~60-100. **Net ≈ −200 LUT
(estimate — no Vivado run was permitted, so this is derived from the
existing utilization report, not measured.)**

**The timing claim, however, holds and is the real payoff.** Of the 131
`u_pb_s1_cdc` path endpoints in
`synth/timing_reports/timing_20260819_131707.rpt`, **107 are in `w_fifo`
(58) and `r_fifo` (49)**, and 95 of those are the `mem_reg_*` LUTRAM data
arrays — precisely what narrowing removes. Only 17 are control logic
(`req_r_hold_cnt`, gray pointers). On a build failing at −0.612 ns this
is the cheapest routing relief available.

**Reset semantics preserved by construction.** The shims have no reset,
so `u_pb_s1_cdc` remains on `core_rst_bank[1]` / `pb_core_rst_bank[3]` —
the *core* banks, not the full-reset banks — and the host path to
`debug_ctrl` still survives a JTAG/button full reset.

**Escape hatch:** define `PB_S1_WIDE_CDC` to restore the 128-bit bridge
with no RTL edit (`fpga_top_peripherals.vh:196-203`).

**Gates run:** `make lint` clean; `tb-peripheral-bus` 55/55, `tb-via1`
37/37, `tb-via2` 23/23, `tb-pb-scsi` 6/6, `tb-scsi` 41/41, `tb-sd-boot`
11/11. **None of those exercise the S1 CDC** — they instantiate
`peripheral_bus` directly — so a new equivalence testbench was written:
`tb-pb-lane-shim` runs 5,118 transactions through REF
(`axi_async_bridge #(128)`) and DUT (`axi_pb_s1_cdc`) chains in parallel
against identical models of `peripheral_bus`'s lane policy, and asserts
byte-exact equality of responses and of both slaves' final memories:
**25,516 PASS / 0 FAIL**. It was mutation-tested (four seeded faults, all
caught) to confirm it is not vacuous.

**System-level A/B, the strongest evidence available without a board.**
`make tb-fpga-top-rom CPU=m68k FPGA_TOP_ROM_MAX_INSTS=400000` — the real
`fpga_top`, the real m68k core, the real Quadra 700 ROM — was run twice,
once with the narrowed CDC and once with `PB_S1_WIDE_CDC` defined:

| Build | sim time | retired | PC at stop |
|---|---|---|---|
| `axi_pb_s1_cdc` (32b payload) | `t=3799960` | 400,000 | `0x40847516` |
| `axi_async_bridge` (128b, escape hatch) | `t=3799960` | 400,000 | `0x40847516` |

**Cycle-for-cycle identical.** Matching `t=` proves the narrowed CDC adds
zero cycles as well as zero behavioural difference, across 400 k
instructions of real firmware including live VIA/SCC/SONIC traffic.

The WSTRB precondition assertion was **compiled into that build**
(verified present in the generated `Vfpga_top___024root__DepSet_*.cpp`)
and **never fired** — so real ROM firmware never presents a multi-lane
strobe to S1. Worth noting because `synthesis translate_off` is a
synthesis pragma, not a Verilator one: Verilator keeps the block, which
is exactly what makes this assertion useful rather than decorative.

**One latent precondition, now asserted rather than assumed.** The narrow
shim recovers the write lane from WSTRB, which is exact for every master
on S1 today but *not* for a hypothetical beat strobing two lanes where
the address lane is not the lowest. The information is unrecoverable at
32 bits (the right answer is always "the address lane", which the W
channel cannot see without AW/W correlation state). State was
deliberately not added — this bridge's reset behaviour is load-bearing —
so `axi_pb_lane_shim.v` carries a simulation-only assertion that fires
the moment the case becomes reachable, turning a silent-corruption risk
into a loud one.

Phase 1 is the only phase that *helps* the currently-failing timing
closure, and it should land regardless of what happens to the rest.

### Phase 2 — Concurrency (a PREREQUISITE for the new core, not an optimisation)

**Re-ranked.** When this document was first written, Phase 2 was a
throughput play against a core that self-serialises anyway. It is not
that any more: `m68k-core-040-ooo` is multi-outstanding on both sides, so
**every item below is a prerequisite for the core the SoC exists to
host**, not a speculative gain. A multi-outstanding core dropped into
today's fabric is throttled to 1 in flight by three modules in series and
then charged 6.6× on top if it reuses an ARID.

**And it still measures ~zero unless 2a, 2a-adapters and 2b/2c all move.**
That is the in-series trap restated: widening the crossbar while
`if_to_axi` still holds one AR and the L2 still accepts one AR globally
changes nothing observable. Land them together, or land them separately
and measure only at the end.


| Item | Change | Area (est.) | Timing risk |
|---|---|---|---|
| **2a** | **Crossbar: N outstanding reads per master.** Replace scalar `rs_state[mi]` with a 4-entry per-master read-tracking table keyed on the master's ARID. R routing already demultiplexes on `s0_rid[XID_WIDTH-1 -: SLOT_W]` (`axi_xbar.v:3408`) and `XID_WIDTH = ID_WIDTH+2 = 6` already carries 4 master-ID bits — **no ID widening needed.** Leave the write side alone: `axi_xbar.v:1634-1653`'s objection to write pipelining is correct. | **+3,000 LUT / +1,500 FF** (u_xbar 4,779 → ~7,800) | **Moderate-high.** Mitigate by **registering the R-return stage** — `mr_rvalid_slv`/`mr_rdata_slv` are today a combinational 4×6×128b mux (`:3446-3466`). Registering it costs 1 cycle of read latency and *removes* a long combinational path from a congested region. Do this as a separable prerequisite (2a-pre) and measure timing before 2a lands. |
| **2a-adapters** | **Queue + ID pass-through in the CPU-side adapters.** `if_to_axi` (`:16-18`) and `axi_narrow_to_wide` (`:72-76`) are both hard-limited to one transaction and both use a fixed `ID_TAG`. They need a small outstanding-transaction table (4-8 entries) and must forward the core's own ARID instead of a constant. Without this, 2a is unreachable — the core cannot present a second transaction to the crossbar at all. **Also update the stale header comments**, which currently justify the limit by core self-serialisation (§9/S9). | +800-1,500 LUT each (est.) | Low-moderate. These are leaf adapters, not in the congestion window. `axi_narrow_to_wide.v:74-76` already scopes this as "a separate (and much larger) change". |
| **2a-boot** | **Multi-outstanding writes in `boot_fsm`.** See the costing note below. | +300-600 LUT (est.) | Low — boot-time only, cannot affect steady-state timing paths. |
| **2b** | **L2 front door: accept AR while a burst decomposes.** Two burst contexts instead of one, so `s_arready` (`l2c_ctrl.v:119`) is not held for a whole burst. This removes the I-fetch/D-side re-serialisation. **Owned by another agent — described, not implemented here.** | +500-1,000 LUT | Moderate; `u_l2c/g_active.u_ctrl` is in 3 of 5 congestion windows |
| **2c** | **L2 front door: pipeline `S_WAIT`/`S_LOOKUP`** so beat N+1 is in WAIT while beat N is in LOOKUP. Constrained by the single-ported tag/data arrays, so it needs a same-set bypass. Target: 3.5 → ~1.5 cyc/beat. **Owned by another agent.** | +1,000-2,000 LUT | High — this is the tag-array timing path |
| **2d** | **Raise `MAX_BULK_AHEAD` 2 → 6** (`axi_vram_priority_mux3.v:100`), or better, make it responsive to `fb_reader`'s ring occupancy rather than static. | **~0 LUT** (one counter bit) | Low. Scanout keeps unconditional AR priority (`:252-256`) and its `MAX_SCAN_AHEAD = 4` quota. |

#### Costing note — `boot_fsm` multi-outstanding writes (item 2a-boot)

**Measured on hardware: reset → `rom_loaded` is 4.06 s ± 2 ms across five
runs**, of which the SD ROM copy is only ~0.13-0.34 s. The remaining
~3.7 s is the 256 MiB RAM pre-zero pass — a pure write stream through
`axi_narrow_to_wide`'s **single-outstanding** write path
(`axi_narrow_to_wide.v:80-83`, `n_awready = !aw_valid_q`).

Each 16 B wide write costs a full AW→W→B round trip through
xbar → L2 → CDC → MIG with nothing overlapped. 256 MiB / 16 B ≈ 16.8 M
transactions; 3.7 s at 100 MHz ≈ 370 M cycles ≈ **22 cycles per write**,
which is consistent with a serialised round trip and confirms the pass is
latency-bound rather than DRAM-bandwidth-bound (256 MiB at even 261 MB/s
would take ~1.0 s).

With 4-8 writes in flight the pass should approach the bandwidth bound,
i.e. **~3.7 s → ~1.0-1.5 s, a 2.5-3.7× cold-boot improvement** (estimate;
it assumes the write path's B responses can be retired out of the
critical loop, and it will be capped by whatever `MAX_BULK_AHEAD` and the
MIG bridge's `WMAX_OUTSTANDING = 4` allow).

Two caveats worth stating:
- The zero pass is now **gated to cold resets only** (`boot_zero_en =
  ~boot_warm_q`, `fpga_top_boot_master.vh:138`, feeding `boot_fsm.v:239`
  `zero_en`, checked at `:1498`). So this is a cold-boot win specifically;
  warm/debug resets already skip it.
- `boot_fsm` writes while the CPU is held in reset and shares xbar slot 0
  with the CPU LSU through a reset-hold mux, not an arbiter
  (`axi_xbar.v:30-70`). Multi-outstanding boot writes therefore cannot
  contend with CPU traffic — which is exactly why this is the
  **lowest-risk** place in the design to prove out a multi-outstanding
  write path before doing it on the LSU side. Worth sequencing first for
  that reason alone.

**Phase 2 total: ~+4,500 to +6,000 LUT (2.1-2.8% of the device),**
against a measured 4.96× available on miss-bound traffic and a derived
~3× on hit-bound traffic. Net after Phase 1's −2,000: **+2,500 to
+4,000 LUT.**

**This is only affordable if the new core is not substantially larger
than the current one.** If it is, Phase 2 must wait for a core-side area
reduction, and only Phase 1 + Phase 0 + 2d should land.

### Phase 3 — Structural (next project, not this one)

| Item | Change | Rationale |
|---|---|---|
| **3a** | Parameterise the crossbar's fan-in arrays and round-robin pickers over `NW`/`NR`. | `axi_xbar.v:71-79` documents the decision not to; that decision expires the moment a 5th master is needed, which is the next core-side request. |
| **3b** | Replace `u_pb_s1_cdc`'s async FIFO with a related-clock elastic buffer. `core_clk` and `pb_clk` are BUFG_GT divisions of one source and are in the same Vivado clock group (`fpga_top_clocks.vh:145-163`, `synth/vivado.tcl:1590`). | Saves ~8 core cycles/2 and several hundred LUT — **but the T6 coupled-reset handshake exists to survive one-sided JTAG debug resets, and any replacement must preserve that.** Not a drop-in. |
| **3c** | Give the crossbar a per-slave outstanding budget instead of a per-master one, and split the read fan-in from the write fan-in. | The current shape charges a peripheral read against the same slot as a DDR read (F5). |
| **3d** | A real DMA engine. | `dma_ctrl` is deleted; every byte of disk data crosses the CPU as PIO at ~1-3 MB/s. **No fabric change fixes this**, and it is very likely the largest single user-visible performance deficit in the machine. |

---

## 8a. The adapter layer — a structural criticism

The individual footguns above are symptoms. The disease is that **every
seam in this SoC gets a bespoke translator, and each one independently
re-decides width, outstanding depth and burst support.**

### Inventory (measured, `wc -l` + instantiation grep)

| Module | Lines | Live instantiation sites |
|---|---|---|
| `axi_narrow_to_wide` | 991 | 3 (CPU, boot_fsm, JTAG) |
| `axi_vram_priority_mux3` | 495 | 1 |
| `axi_async_bridge` | 465 | 8 |
| `axi_pb_lane_shim` | 344 | 2 (inside `axi_pb_s1_cdc`) |
| `axi_pb_s1_cdc` | 304 | 1 (S1, via the `PB_S1_CDC_MODULE` macro) |
| `axi_bridge_w_pad` | 277 | 1 |
| `axil_split2` | 262 | 1 |
| `axi_n64_to_wide` | 234 | **0 — dead** |
| `axi_wide_to_axilite` | 213 | 3 |
| `axi_bridge_stale_sink` | 207 | 2 |
| `axil_async_bridge` | 204 | 1 |
| `axi_vram_smoke_mux` | 196 | 3 |
| `if_to_axi` | 151 | 2 |
| `axil_null_slave` | 125 | 1 |
| **Total adapters** | **4,468** | |
| `axi_xbar` | 3,890 | 1 |
| **Total interconnect glue** | **8,358** | |

**Corrections to an earlier count of this inventory**, since both errors
point the wrong way:

- `axi_pb_lane_shim` (344) and `axi_pb_s1_cdc` (304) are **not dead** —
  they are new in this session and are Phase 1a itself. `axi_pb_s1_cdc`
  is the live S1 bridge, instantiated at
  `fpga_top_peripherals.vh:203`. It is elaborated through a
  `PB_S1_CDC_MODULE` macro (so the escape hatch is a one-define revert),
  which means `grep "axi_pb_s1_cdc #("` finds nothing — a real
  grep-visibility cost that is now called out in a comment at the
  instantiation site. It is **not** a pre-existing module that Phase 1a
  could simply have instantiated.
- `if_to_axi` is **not testbench-only** — `cpu/rtl/core/m68k_axi_wrapper.v:1142`
  instantiates it in every `CPU=m68k` build, i.e. in the real bitstream.
  It resolves to `rtl/soc/if_to_axi.v` because `cpu/rtl/sys` is an
  include path only, never a source list (`Makefile:1687, 1698`).

**Genuinely dead: `axi_n64_to_wide` alone, 234 lines** — its only mention
under `rtl/` is a comment in `fpga_top_dma.vh:12` explaining that the DMA
seat it widened was removed. `synth/vivado.tcl` still reads it. Dead
modules cost no LUTs, but they cost review time and they mislead.

### The pattern

Thirteen live adapter types, and **every one of them independently landed
on "one outstanding, single-beat"**:

| Adapter | Outstanding | Burst |
|---|---|---|
| `if_to_axi` | 1 | no (`arlen=0`) |
| `axi_narrow_to_wide` | 1 rd + 1 wr | packs, then serialises |
| `axi_xbar` slot | 1 rd + 1 wr | pass-through |
| `l2c` front door | 1 AR globally | **charged per beat** |
| `l2c_bypass` | 1 | no (single-beat) |
| `axi_wide_to_axilite` | 1 rd + 1 wr | no |
| `axi_vram_priority_mux3` | 2 bulk | yes |
| `peripheral_bus` | 1 rd + 1 wr | no |

That is not coincidence. It is what happens when concurrency is a
property each translator decides for itself rather than a property the
interconnect guarantees. And it is precisely why the in-series finding
holds: a CPU transaction crosses `axi_narrow_to_wide` → `axi_xbar` → L2
front door → CDC → MIG bridge, and **every hop is entitled to serialise
it independently**. Only the last one (`axi_ddr4_mig_bridge`, 8 rd / 4
wr) declines to.

The same fragmentation shows in width: 32 → 128 → 32 → 128 → 32 → 128 →
256 between the CPU's D-cache and the DRAM. Each conversion is locally
justified; the sequence is not.

### What the fabric should look like instead

**One internal protocol, one internal width, adapted once at each true
edge — with outstanding capability and burst support preserved end to end
by construction rather than re-derived per hop.**

There are exactly four true edges. Everything else is internal and should
not translate anything:

| True edge | Why it is real | Adapter that belongs there |
|---|---|---|
| CPU socket | Different repo, different team, versioned contract | none needed — `cpu_socket.vh` is already 128b AXI4 |
| DDR PHY | MIG owns its clock and 256b UI | `axi_ddr4_mig_bridge` (keep — already 8-outstanding) |
| Peripheral domain | 50 MHz, 8-bit registers, genuinely slow | one width+CDC adapter (Phase 1a is the first half of it) |
| Video / scanout | `pclk`, hard real-time | `scanout_*` + its credit ring |

Concretely, for the next project:

1. **Fix the internal bus at 128b AXI4 with a 6-bit ID** — the width it
   already mostly is. Then `axi_narrow_to_wide` (991 lines, the single
   largest adapter) **disappears entirely** the moment the core drives
   the socket natively, which `cpu_socket.vh` already specifies. Keep one
   instance for boot_fsm/JTAG or widen those two masters instead.
2. **Make outstanding depth a bus property, not a module property.**
   Every internal module either passes IDs through and tracks N in
   flight, or it is not allowed on the internal bus. A module that can
   only do 1 is an *edge* adapter by definition.
3. **Make burst support a bus property too.** The L2 charging per beat is
   the most expensive single violation of this, and it is invisible from
   outside the module.
4. **Put arbitration in one place.** `axi_vram_priority_mux3` (495 lines)
   exists because the crossbar could not be extended (F7). Priority and
   quality-of-service belong in the crossbar, not in a second arbiter
   bolted onto one slave port.

### What it would cost, and what survives

| | Today | Proposed |
|---|---|---|
| Adapter types | 13 live | ~5 |
| Adapter lines | 4,468 | ~2,000 (est.) |
| Crossbar | 3,890, fixed 4×6 | ~2,500 (est.), parameterised N×M |
| Width conversions, CPU→DRAM | 6 | 2 (peripheral edge, DDR edge) |
| Serialisation points, CPU→DRAM | 5 | 1 (the L2 lookup, pipelined) |

**Survives as-is:** `axi_async_bridge` (the CDC is genuinely well built —
five gray-pointer FIFOs, correct, burst-amortised, and its
`axi_bridge_w_pad` / `axi_bridge_stale_sink` reset-recovery machinery is
load-bearing and should not be re-derived), `axi_ddr4_mig_bridge`, the
`scanout_*` chain.

**Folds into one peripheral-edge adapter:** `axi_wide_to_axilite`,
`axi_pb_lane_shim`, `axi_pb_s1_cdc`, `axil_split2`, `axil_async_bridge`,
`axil_null_slave`.

**Disappears:** `axi_narrow_to_wide`, `if_to_axi` (both obsolete once the
core drives the socket natively), `axi_n64_to_wide` (already dead),
`axi_vram_priority_mux3` and `axi_vram_smoke_mux` (fold into the
crossbar).

**Honest caveat on the cost line:** this is a rewrite of the
interconnect, not a refactor, and the ~2,000-line estimate excludes the
correctness hardening that the current modules have accumulated —
watchdogs, poison latches, abandoned-burst padding, one-sided-reset
recovery. That hardening is the most valuable thing in the current
fabric and reproducing it is most of the real work. **This is a
next-project plan, not a this-project plan**, and nothing in §8 depends
on it.

---

## 9. What would silently regress — no test would fail

These are the ones that bite for years, because the design keeps working
and only gets slower. None of them is covered by `make test`, `make fuzz`,
or any `tb-*` target today.

| # | Change | What silently regresses | Why no test catches it | Detection |
|---|---|---|---|---|
| **S1** | New core uses a single AXI ID for all data traffic | L2's same-ID gate (`l2c_ctrl.v:155`) serialises every access. All the Phase-2 work delivers **zero**. | Every test passes; AXI is fully compliant; the gate is a *correctness* feature working as designed. | `dbg_l2c_mshr_occupancy` never exceeds 1. **Assert this in a tb.** |
| **S2** | Anyone raises the L1 line size, or the new core issues longer bursts | `l2c_ctrl.v:119` holds `s_arready` low for the whole burst decomposition, so head-of-line blocking grows *linearly with burst length*. Longer bursts make the system slower. | Bursts are correct. Throughput tests use one master. | A two-master tb measuring master B's AR-accept latency while master A bursts. **Does not exist.** |
| **S3** | Someone routes a new consumer through an L2 bypass window | `l2c_bypass.v` is single-op single-beat; the front door shreds bursts into single beats (F6). ~19× throughput loss and a multi-µs CPU read blackout. | `tb-l2c-bypass-window` tests correctness, not throughput. `docs/l2c_spec.md:432-441` warns in prose only. | Add a throughput assertion to `tb-l2c-bypass-window`. |
| **S4** | `MAX_BULK_AHEAD` or `MAX_SCAN_AHEAD` retuned for a scanout fix | Silently halves or doubles CPU DDR bandwidth. Nothing connects these constants to a CPU-side metric. | They are scanout parameters; nobody looks at CPU throughput when changing them. | Pin both in `docs/bench_baseline.md` with the CPU bandwidth they imply. |
| **S5** | `CORE_CLK_DIVIDE` raised to 2 (a plausible reaction to the new core failing timing) | `async_fifo.v:472-487`'s `REQ_MIN_HOLD = 8` bounds the clock ratio to ~1:4. `core:mig_ui` is 1:3.33 today; at 50 MHz core it becomes **1:6.7 and the reset handshake breaks.** | Only manifests on a reset during traffic — rare, timing-dependent, and looks like a random hang. | **Add an elaboration-time assertion** deriving `REQ_MIN_HOLD` from the ratio. |
| **S6** | Scanout mode raised to 1024×768×24bpp | Demand 189 MB/s against a ceiling of 180-251 MB/s depending on the real DDR RTT. At 60 cycles **it does not close**, and the failure mode is torn or displaced video, not an error. | The 40-cycle RTT is an assumption (`scanout_ddr_reader.v:77-80`), never measured. | Phase 0b's latency counter. |
| **S7** | Any burst-shape "optimisation" upstream of the L2 | Measured: burst hits are **worse per byte** than pipelined singles (3.54 vs 3.09 cyc/beat). Packing more into bursts makes it slower. | Bursts are correct and *look* like the optimisation. Task #170's header claims "~339 vs ~52 cycles" — true for the AXI round trips it removed, but it does not survive the L2 front door. | Compare B/cycle, never cycles-per-transaction. |
| **S8** | Registering the xbar's R-return stage (proposal 2a-pre) | Adds 1 cycle to *every* read. If Phase 2 is then abandoned, this is a pure regression left in the tree. | It is a timing fix; nobody re-measures latency after a timing fix. | Land 2a-pre and 2a together, or not at all. |

| **S9** | The new core lands while `if_to_axi.v:16-18` and `axi_narrow_to_wide.v:72-76` still say "the core self-serialises, so there is no queue here" | Nothing regresses — but the comments become **false**, and they read as "this is fine" to whoever is debugging why the new core is slow. The single-outstanding limit will look intentional and justified. | Comments are not tested. They are accurate today. | Update both headers **in the same change that swaps the core**, not after. |
| **S10** | Someone edits `cpu/rtl/sys/if_to_axi.v` or `cpu/rtl/sys/axi_narrow_to_wide.v` | **The edit has no effect.** Only the byte-identical `rtl/soc/` copies are compiled (`Makefile:1687, 1698`); `cpu/rtl/sys` is an include path only. The socket doc's "the shims relocate CPU-side (Phase 4)" wording actively invites this mistake. | Build succeeds, tests pass, behaviour unchanged. | `Makefile:2535`'s rsync hint, or delete one copy. |

**The common shape:** every one of these is a *correctness feature or a
correct-looking optimisation* whose performance side-effect is invisible.
The fabric has excellent correctness tests and **no throughput tests at
all**. That asymmetry is the root cause of the state this document
describes.

**Concrete recommendation:** add a `tb-fabric-throughput` target that
asserts B/cycle floors for (a) L2 hit stream, (b) 8-way concurrent miss
stream, (c) two-master interleave, (d) bypass-window stream, and (e)
**N concurrent misses under a single ID vs. N unique IDs** — that last
one pins the 6.6× cliff of F4/S1, which is the highest-cost invisible
failure in the list. Wire it into the same gate as `make fuzz`. Five
assertions would have caught seven of the ten rows above.

---

## 10. The single highest-value change, and how to confirm it

### First, a distinction that matters

"Mandatory" and "highest-value" are different questions, and the answers
differ:

- **Mandatory** is all of Phase 2 — crossbar, CPU-side adapters, and the
  L2 front door together. The new core is multi-outstanding; without
  these it is throttled to 1 in flight and charged 6.6× on top if it
  reuses an ARID. This is not a choice.
- **Highest-value**, if the question is "where does one unit of effort
  buy the most", is the L2 front door, for the reasons below.

They are not in conflict: the front door is both the largest single win
*and* one of the three things Phase 2 must land anyway.

### The change

**Remove the L2 front door's global single-AR gate and its per-beat
serialisation** — `l2c_ctrl.v:119` (`s_arready = !ar_have`) and the
per-beat walk at `:307-380`.

Not the crossbar. Not the DDR path. Not width. Here is why:

1. **It is the only limit that binds for a single master.** F1 (crossbar)
   and F3 (mux) cap *concurrency*; if the workload has no memory-level
   parallelism to exploit — and a 68k running System 7 largely does not —
   they cost nothing. The front door caps *every* access, dependent or
   not.
2. **It is the only limit that affects hits**, which in a working cache
   are the overwhelming majority of accesses. Fixing the crossbar or the
   mux does nothing whatsoever for a hit.
3. **It is the only limit that couples masters to each other.** The
   crossbar carefully gives I-fetch its own slot; `ar_have` throws that
   away by blocking every master's AR for the duration of any burst.
4. **It is already moving, and the movement is measured.** Removing one
   state (`S_HITRESP`) bought **36%** — 3.79 → 5.17 B/cycle. That is the
   strongest available evidence that this is where the ceiling lives.
5. **It makes everything else worth doing.** Phase 2a's crossbar work and
   the CPU-side adapter queues are both pointless while the L2 accepts
   one AR at a time — and, symmetrically, the front-door work is
   unobservable until the adapters and crossbar can present a second
   transaction. That mutual dependency is the in-series trap, and it is
   why "highest-value" does not mean "do this one alone".

### The measurement that would confirm it

**Read the L2 hit/miss ratio on real hardware under a real System 7
boot.** That single number decides whether this analysis is right:

- **Hit rate high (> ~85%):** hit bandwidth dominates, only the front door
  matters, and this recommendation is correct. Proceed with 2b/2c first.
- **Hit rate low (< ~60%):** miss concurrency dominates, and F1 + F3
  (crossbar + mux) should lead instead.

The counters already exist — `dbg_l2c_hit_count`, `dbg_l2c_miss_count`,
`dbg_l2c_mshr_occupancy` (`l2c.v`, exposed at
`fpga_top_debug_vio.vh:169-182`). **But they are ILA probes only**, so
reading them requires a capture session rather than a
`/tmp/jcmd.sh "r 0x..."`. That is Phase 0a, ~100 LUT, and it is the
prerequisite for making any of this a decision rather than an argument.

Alongside it, read `dbg_l2c_mshr_occupancy`. If it never exceeds 1 under
load, the same-ID gate (S1) is active and **no amount of crossbar work
will help until the core rotates its AXI IDs.**

### If only one thing lands

Land **Phase 1a** — narrow the S1 path to 32 bits before the CDC.

It is the only proposal that is negative in area (−400 to −500 LUT),
negative in timing risk (removes 131 path endpoints from a build that is
failing at −0.612 ns), reuses a module that already exists and is already
proven on three other slave ports, and is completely independent of the
concurrency work and of the CPU swap. It does not make the machine
meaningfully faster. It makes the next bitstream more likely to close,
which right now is worth more.

## Prior art for Phase 2a — `agent/soc-if-outstanding`

**Read this before implementing per-master outstanding-read tracking.** A
branch already implements a working subset, with tests. It is NOT the design
to adopt wholesale, but the hard parts are solved there.

What it does: a parameterised `CPU_IF_RD_DEPTH` (default 5) distinct-ID
context table in `axi_xbar`, lifting the 1-outstanding limit for CPU
instruction fetch on the cacheable S0 route. Includes duplicate-ID ordering,
per-context watchdogs, and two **non-vacuous** testbench scenarios: five S0
ARs must be *accepted* before any R exists, and one timeout must SLVERR all
five.

Why it is not sufficient for the v2 core, in the branch's own words:

> "Keeping this as a small **M2/S0-only CAM** ... The generic `rs_state[]`
> array is indexed by PHYSICAL MASTER and is **intentionally left one-deep
> for M0/M1/M3** and for M2's non-cacheable routes."

So it covers instruction fetch only. The v2 core is multi-outstanding on
**both** I and D sides, and the D side (M0/LSU) is the more valuable half for
miss concurrency. Phase 2a's per-master ARID-keyed table subsumes it.

It also independently chose **distinct IDs**, which the L2C measurements later
confirmed as essential: 8.38 cyc/op with unique IDs versus 55.40 with one ID,
a **6.6x cliff** (`docs/l2c_perf.md`).

**Harvest, do not merge.** The branch is ~106 files divergent from `main`, so
lifting the mechanism and the tests is less work than reconciling it. Extracted
copies (in case the branch does not survive the planned history-less
republish):

    ~/rescued-worktree-work/prior-art-if-outstanding/
        axi_xbar.diff   (568 lines)   the context-table implementation
        tb.diff        (2842 lines)   the two scenarios -- the valuable part
        commits.txt                   original commit messages

Remember the sequencing warning above: this limit is one of four in series.
Landing it alone measures as zero.
