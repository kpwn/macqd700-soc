# L2 cache (`l2c`) — measured performance and area

Status: 2026-08-19.  Everything below the "Baseline" heading is **measured**
in `tb/tb_l2c.cpp`'s new performance suite or read out of the **post-route**
report `build/vivado/reports/utilization_route.rpt` / `timing_synth.rpt`.
Numbers that are derived rather than measured are labelled ESTIMATE.

No Vivado was run for this work — no post-route number here is new.  Every
area figure for a *proposed* change is an estimate built on the shipped
post-route report plus the cell-level structure visible in the synth timing
report, and is stated as such.

---

## 0. Executive summary

| | Before | After (this change set) |
|---|---|---|
| Sustained hit throughput | 4.00 cyc/op (400 MB/s) | **3.00 cyc/op (533 MB/s)** |
| 64 B read-hit burst | 16.00 cyc | **12.01 cyc** |
| `l2c` post-route LUT | 24,222 (of which `u_mshr` 20,953) | ESTIMATE **~10,000-14,000** |
| Front-door FSM states | 4 | **3** |
| `m_line` write sources | 3 | **1** |
| Gates | 31/31 `tb-l2c`, 4/4 chain, 17/17 vram-chain, lint clean | same |

The three headline results of the *measurement* work, independent of any
change:

1. **The RAM-disk bypass window is the real, current throughput defect.**
   A 512 B RAM-disk block read measures **1,376 cycles (37 MB/s)** where the
   same 512 B through the cacheable path costs 506 cycles and a native
   32-beat pass-through would cost ~72.  Worse, it head-of-line blocks the
   CPU: with a disk stream running, CPU hit cost goes **3.00 -> 23.21
   cycles (7.7x)** at a 40-cycle DDR latency and **101.5 cycles (34x)** at
   200.  Section 5.1.
2. **`u_mshr` is 20,953 LUT — twice the whole MIG DDR4 controller — and
   ~59% of that is one avoidable structure**, the 3-source write mux on the
   4096-bit `m_line` array.  Post-route evidence, not conjecture.  Section
   5.2.  Half of that has been removed in this change set.
3. **The same-ID accept gate is a 6.6x cliff that the incoming core will
   fall off.**  Measured: 8 outstanding misses with unique IDs = 8.38
   cyc/op; the *same* 8 misses under one AXI ID = **55.40 cyc/op**.  Today
   nothing can trigger it, because every master reaching `xbar S0` is
   one-outstanding with a constant ID.  `m68k-core-040-ooo` is
   multi-outstanding on both the I and D side, so the moment it lands this
   gate becomes the difference between a working L2 and one that measures
   as if it were not there — and it will read as "the new core is slow",
   not as a fabric bug.  Section 4.  This is the same limit
   `docs/soc_bus_review.md` labels **F4**; the numbers above are its
   quantification.

**Sequencing warning, applies to everything below.**  `l2c`'s front door is
one of four single-outstanding limits stacked in series across the fabric
(`docs/soc_bus_review.md` F1-F4; enumerated in section 4.4).  Fixing any one
of them alone measures as **zero**, because the next one binds immediately.
The two changes landed here — the hit-rate fix in 5.3 and the area work in
5.2/5.4 — are deliberately of a kind that *is* independently measurable; the
concurrency work in 5.0 and 7.3 is not, and must be landed together with F1
and F3.

**Two recommendations from an earlier draft of this document are withdrawn**
(reducing `MSHR_N` and `MSHR_REPLAY_N` on the grounds they are unreachable,
and deferring the same-ID gate fix as unnecessary).  Both rested on today's
one-outstanding masters and do not survive `m68k-core-040-ooo`.  Sections
4.2, 5.5, 7.1 and 7.3 carry the retractions inline rather than silently
deleting them.

---

## 1. Measured baseline

Stimulus conditions, identical for every row: memory model at a **fixed**
first-beat latency with **zero** beat gap and AWREADY/WREADY/ARREADY held
high; requester holds RREADY/BREADY high.  Deterministic on purpose — this
isolates `l2c`'s own occupancy from DDR jitter.  MB/s is at the real
**100 MHz** `core_clk`.  "hdr->data" is the measured AR-accept-to-first-R
(or AW-accept-to-B) delay, which includes queueing at the front door.

Reproduce with:

```
make tb-l2c
L2C_PERF=1 L2C_SKIP_DIRECTED=1 L2C_RAND_OPS=200 build/l2c/Vtb_l2c
```

The suite is `perf_suite()` in `tb/tb_l2c.cpp`; it runs after the
correctness suite, resets the DUT between scenarios to get a cold cache
(which drops dirty lines by design), and does no data checking.

### 1.1 DDR latency = 40 cycles (representative of the real MIG+CDC path)

| Scenario | Stimulus | cyc/op | MB/s | hdr->data |
|---|---|---:|---:|---:|
| `hit_read` | 16 B read hit, 1 outstanding | 5.00 | 320 | 4.0 |
| `hit_read` | 16 B read hit, 2 outstanding | **3.00** | **533** | 5.0 |
| `hit_read` | 16 B read hit, 8 outstanding | 3.00 | 533 | 5.0 |
| `hit_read_single_id` | 16 B read hit, 8 out, 1 ID | 3.00 | 533 | 5.0 |
| `hit_write` | 16 B write hit, 8 outstanding | 3.00 | 533 | 5.0 |
| `hit_read_64B_burst` | 64 B (4-beat) read-hit burst | 12.01 | 533 | 5.0 |
| `miss_read` | 16 B read miss, 1 outstanding | 54.20 | 29.5 | 53.2 |
| `miss_read` | 16 B read miss, 2 outstanding | 28.25 | 56.6 | 55.4 |
| `miss_read` | 16 B read miss, 4 outstanding | 14.88 | 108 | 58.0 |
| `miss_read` | 16 B read miss, 8 outstanding | 8.38 | 191 | 63.4 |
| `miss_read` | 16 B read miss, 16 outstanding | 8.03 | 199 | 71.5 |
| `miss_read_single_id` | 16 B read miss, 8 out, **1 ID** | **55.40** | **28.9** | 105.8 |
| `miss_read_stride4k` | 16 B read miss, 4 KB stride | 8.38 | 191 | 63.4 |
| `miss_write_alloc` | 16 B write miss (allocate) | 8.38 | 191 | 63.4 |
| `stream_read_64B` | 64 B read burst, 8 out, uniq IDs | 63.84 | 100 | 53.8 |
| `stream_read_64B_single_id` | 64 B read burst, 8 out, 1 ID | 65.69 | 97.4 | 55.6 |
| `stream_write_64B` | 64 B write burst, 8 out | 62.84 | 102 | 64.8 |
| `cacheable_read_512B` | 512 B (32-beat) cacheable read | 505.83 | 101 | 61.6 |
| `bypass_read_512B` | 512 B (32-beat) **bypass** read | **1376.25** | **37.2** | 116.6 |
| `bypass_write_512B` | 512 B (32-beat) bypass write | 1377.25 | 37.2 | 1451.5 |
| `bypass_read_16B` | 16 B single-beat bypass read | 44.02 | 36.4 | 85.3 |
| `mixed_cpu_hits_plus_bypass` | 256 hits + 4x512 B blocks, shared S0 AR | 5942 total | — | **23.21 cyc/hit** |

### 1.2 DDR latency = 200 cycles (the tb's own default; latency-sensitivity check)

Only the rows that change materially:

| Scenario | cyc/op | MB/s |
|---|---:|---:|
| `hit_read` (8 out) | 3.00 | 533 |
| `miss_read` (1 out) | 214.20 | 7.5 |
| `miss_read` (8 out) | 28.38 | 56.4 |
| `miss_read_single_id` (8 out, 1 ID) | 215.40 | 7.4 |
| `stream_read_64B` | 223.84 | 28.6 |
| `cacheable_read_512B` | 1785.83 | 28.7 |
| `bypass_read_512B` | 6496.25 | 7.9 |
| `mixed_cpu_hits_plus_bypass` | — | **101.51 cyc/hit** |

Reading the two tables together: everything on the **hit** path is
pipeline-bound (identical at both latencies).  Everything on the **miss**
path is latency-bound and scales cleanly with MSHR depth — `miss_read`
divides by ~6.5x going from 1 to 8 outstanding, so the MSHR's fill
concurrency genuinely works.  The **bypass** path is latency-bound *per
16 B beat*, which is why it is 32x worse than it should be.

### 1.3 Where a hit's cycles actually go

From the FSM (`rtl/soc/l2c_ctrl.v`, `S_IDLE`/`S_WAIT`/`S_LOOKUP`) confirmed
against the measured 3.00 cyc/op sustained rate:

| Cycle | State | What happens |
|---|---|---|
| t | `S_IDLE` | accept one beat; present `cur_set` to the tag+data arrays |
| t+1 | `S_WAIT` | tag BRAM read returns (1 cy), data URAM still in flight (2 cy) |
| t+2 | `S_LOOKUP` | 8-way tag compare, victim/MSHR/skew hazard checks, way-select data mux, load the response skid |
| t+3 | — | R/B presented; front door is already accepting the next beat |

So sustained hit cost is exactly the **array read latency (2) + resolve
(1)**.  There is no arbitration cost and no response-hold cost any more
(there used to be: see 6.1).  Getting below 3 requires overlapping two ops'
`S_WAIT`/`S_LOOKUP`, i.e. duplicating the `req_*` pipeline registers — see
7.4 for the cost.

---

## 2. Post-route area baseline

From `build/vivado/reports/utilization_route.rpt` (Design State: Physopt
postRoute, `xcku5p-ffvb676-2-i`).  Containment **is** nested — `u_ctrl`
encloses `u_mshr`, `u_data` and `u_tags`; do not add them to `u_ctrl`.

```
fpga_top                     187,011 LUT  128,583 FF   98 RAMB36  34 RAMB18  64 URAM
  u_cpu                      107,855 LUT   (58%, being replaced)
  u_l2c                       24,222 LUT   12,878 FF   17 RAMB36            64 URAM
    g_active.u_ctrl           23,434 LUT   11,451 FF   17 RAMB36            64 URAM
      (u_ctrl itself)             41 LUT    1,270 FF
      u_mshr                  20,953 LUT   10,168 FF                            <-- 87% of l2c
      u_data                   2,256 LUT        0 FF                     64 URAM
      u_tags                     163 LUT        0 FF   17 RAMB36
    g_active.u_victim            569 LUT    1,089 FF
    g_active.u_bypass            213 LUT      320 FF
  u_mig_ddr4                  10,282 LUT   (a complete DDR4 controller)
  u_xbar                       4,779 LUT
```

Device headroom: LUT **86.2%** used, URAM **64/64 (full)**, BRAM ~115 of 240
RAMB36-equivalents used — **BRAM is ~52% free**.  That asymmetry is what
makes "move fabric registers into BRAM" a strict win here.

`u_data`'s 2,256 LUT with 0 FF is the 8-way read mux, not the array: the
two 512-bit pipeline registers in `rtl/soc/l2c_data.v:53-54` were absorbed
into URAM288's own output registers, and Vivado attributed the way-select
mux to this level of hierarchy.

---

## 3. Why `u_mshr` is 20,953 LUT — evidence, not conjecture

`build/vivado/reports/timing_synth.rpt` contains a path ending at
`u_l2c/g_active.u_ctrl/u_mshr/m_line_reg[1][127]/D`.  Its final three
stages are:

```
LUT3  u_mshr/m_line[1][127]_i_3   -> net m_line[3199]
LUT6  u_mshr/m_line[1][127]_i_4
LUT6  u_mshr/m_line[1][127]_i_1   -> m_line_reg[1][127]/D
```

Those cell names are **per-bit** (`[1][127]`), so they are not shared: the
write network costs **three dedicated LUTs for every one of `m_line`'s
4096 bits ~= 12,300 LUT**, i.e. ~59% of the module, before anything else.
Two further cells in the same cone are shared (`m_line[7][511]_i_13`,
fanout 512; `m_line[7][127]_i_6`, fanout 10).

The mechanism: `m_line` (`rtl/soc/l2c_mshr.v:115`) was written from **three**
sources — returning fill beats, the primary write's merge at `S_INSTALL`,
and each replay write's merge at `S_SWR` — so every bit needed a 4:1
select (three data sources plus hold).  Cross-check: 4096 m_line bits +
1024 `p_wdata` + 4096 `r_wdata` + the rest = ~10.6 kbit of storage against
the reported 10,168 FF, so the FF side is fully accounted for and the LUTs
are all *muxing*.

The remaining ~8,600 LUT decomposes (ESTIMATE, from the same structure) as:
five separate flat dynamic reads of `m_line[act]`/`r_wdata[act][rk_i]` at
32:1-over-128b or 8:1-over-512b (~5,300), the two `merge_quad` byte-lane
selects (~260), `p_wdata[act]`/`p_wstrb[act]`/`p_*` 8:1 reads (~400), the
32-slot replay write-enable decode and the `idq_busy` associative search
across 8 primaries + 32 replay slots (~1,000), and the eight per-entry
state bits, beat counters, round-robin rotates and `l2c_pri8` encodes
(~1,600).

Note the associative search the coordinator asked about is **not** the
dominant term: `idq_busy_c` (`rtl/soc/l2c_mshr.v:136-141`) compares a 6-bit
ID across 8+32 slots, which is ~40 6-bit comparators, order 300-500 LUT.
Real, worth removing eventually (see 4), but it is not where the 21 K went.
**The muxing is the dominant term, and it was reducible.**

---

## 4. One ID, one outstanding — a premise with an expiry date

### 4.1 What is true today

Every AXI master that can reach `xbar S0` — and therefore `l2c` — drives a
**compile-time constant** AXI ID and allows **one outstanding transaction**:

| Master | xbar port | ID | Outstanding |
|---|---|---|---|
| CPU data (LSU / dcache bypass / boot FSM) | M0 | `rtl/soc/axi_narrow_to_wide.v:716,934` — `ID_TAG` | `axi_narrow_to_wide.v:72`; the L1-D above it is itself blocking (`cpu/rtl/core/mem/dcache.v:61`, "Only one outstanding operation at a time") |
| CPU instruction fetch | M2 | `rtl/soc/if_to_axi.v:98` — `ID_TAG` | `if_to_axi.v:16`; L1-I likewise (`cpu/rtl/core/fetch/icache.v:12`, "Only one outstanding request at a time") |
| Host / JTAG (xdma) | M1 | `rtl/soc/fpga_top_debug_host.vh:478,489` — `4'b0` | single |
| RAM disk (`vhdd_ddr`) | via M0 decode | `AXI_ID = 4'd3` (`fpga_top_peripherals.vh:2005`) | one 32-beat burst at a time |

So the measurement in 1.1 is correct and the same-ID gate
(`rtl/soc/l2c_ctrl.v:155`) is genuinely free **right now**: no master can
present a second live op under one ID.

### 4.2 Why that stops being true, and what it costs

**The core in the tree is being replaced.**  `m68k-core-040-ooo` is
multi-outstanding on both the instruction and data side, and the boot FSM
is expected to follow.  Two conclusions invert:

**(a) The 8-entry MSHR is not dead capability — it is capability that is
about to become reachable.**  An earlier draft of this document argued for
shrinking `N` on the grounds that no master could ever fill it.  **That
recommendation is withdrawn.**  It was correct about today's traffic and
wrong about the machine this SoC exists to host.  `N=8` should be left
alone.  `docs/soc_bus_review.md`'s area table makes the same point from the
other direction: `u_mshr`'s 20,953 LUT is capability "already bought and
paid for" that the fabric cannot yet feed — the fix is to feed it, not to
throw it away.

**(b) `N=8` and relaxing the same-ID gate are complementary, not exclusive.**
The earlier framing ("you get eight MSHRs only by relaxing the gate; you get
the area back only by keeping it") assumed a single-outstanding world.  With
a multi-outstanding master you need **both**: the depth to hold the misses,
*and* an accept path that will let them in.

The cost of getting (b) wrong is measured, not estimated:

| 8 outstanding read misses | cyc/op | MB/s | ratio |
|---|---:|---:|---:|
| unique AXI ID per transaction | 8.38 | 191 | 1.0x |
| one shared AXI ID | **55.40** | **28.9** | **6.6x worse** |

55.40 is within 2% of the 1-outstanding figure (54.20).  **A multi-outstanding
core that does not rotate IDs gets exactly zero miss concurrency from an
8-entry MSHR.**  Nothing fails; IPC is simply low, and the natural reading is
that the L2 is not helping.

### 4.3 What the new core must do, and what the fabric must do anyway

**Core side (required):** rotate AXI IDs across in-flight transactions on
**both** masters.  The socket allocates `CPU_SOCKET_AXI_IW = 4` — 16 IDs per
master — and the crossbar composes `{slot[1:0], master_id[3:0]}`
(`axi_xbar.v:3752`), so the per-master ID space is not shared and needs no
central allocation.  One ID per outstanding L1 MSHR entry is sufficient and
is the cheapest possible discipline.  This belongs in the **socket
contract**, written down before the core is integrated — see
`docs/soc_bus_review.md` F4, which makes the same point and warns it "belongs
in the socket contract, not in a postmortem".

**Fabric side (defence in depth):** do not rely on the core getting it
right.  The gate should stop punishing a same-ID master that has no
ordering conflict.  Two relaxations, in increasing order of cost, both
detailed in 7.3:

1. Allow a same-ID accept when the new op would **merge into the same MSHR
   entry** (same 64 B line).  The replay FIFO is strictly FIFO, so response
   order is preserved for free.  This alone fixes every multi-beat burst
   from a fixed-ID master — the `stream_read_64B` / `cacheable_read_512B`
   rows in 1.1 — with no ordering machinery at all.
2. Service MSHR completions in **allocation order** rather than round-robin,
   which makes cross-entry same-ID ordering hold by construction.  This is a
   *deletion*: `rr_ptr`, the rotate/encode pair and the starvation-avoidance
   argument in `docs/l2c_spec.md:300-320` all go away, replaced by a FIFO
   that cannot starve.

Both are worth doing regardless of what the core does, because they also
make the *bypass* and *boot FSM* paths behave.

### 4.4 Three limits in series — why fixing one measures as nothing

This is the sequencing point, and it is the one most likely to be lost.
`docs/soc_bus_review.md` §6 labels the stack:

| | Limit | Where | Effect alone |
|---|---|---|---|
| **F1** | 1 outstanding read per crossbar master port | `axi_xbar.v:3208` | caps every master at 1 |
| **F2** | `l2c` front door: 1 AR globally, and 1 *beat* at a time | `l2c_ctrl.v:119`, `st` FSM | caps all masters jointly at 1 |
| **F4** | `l2c` same-ID accept gate | `l2c_ctrl.v:155` | caps each *ID* at 1 |
| **F3** | VRAM lane mux `MAX_BULK_AHEAD = 2` | `axi_vram_priority_mux3.v:100` | caps fills reaching DRAM at 2 |

**These are in series.**  Fixing any one alone leaves the next binding
immediately, so it measures as zero improvement — and the predictable
conclusion is "that wasn't the bottleneck", which is a wrong conclusion
drawn from a correct measurement.  `docs/soc_bus_review.md` §8 states the
same requirement: **F1, F2 and F3 must land together**, and F4 must land
with them or a fixed-ID core silently re-imposes the limit.

Two consequences specific to this document:

- **My 1.1 miss-concurrency numbers are an upper bound on the real SoC.**
  `tb_l2c` connects `l2c`'s master port straight to the memory model.  In
  production, F3 caps concurrent bulk fills at 2, so the shipped machine
  sits nearer the `miss_read, 2 outstanding` row (28.25 cyc/op at L=40)
  than the 8-outstanding row (8.38), no matter what the MSHR can do.
- **The `S_HITRESP` removal in 5.3 is a genuine, independently measurable
  win** (it is a *rate* fix, not a *concurrency* fix, so it is not gated on
  F1/F3) — but it is only 1 of the 3 cycles.  Making the front door
  transaction-granular rather than beat-granular is the F2 work, and that
  one is not independently measurable.

## 5. Findings, ranked by (throughput gained + area saved + complexity removed)

The ranking has a time axis, so it is stated as two lists rather than
fudged into one:

- **#1 today, on the machine that exists:** 5.1, the RAM-disk bypass window.
- **#1 the day `m68k-core-040-ooo` lands:** 5.0, the same-ID accept gate.
- **#1 for area, unconditionally:** 5.2, already landed.

### 5.0 [#1 for the incoming core, not landed] The same-ID accept gate

**Measured cost: 6.6x on the miss path** (8.38 -> 55.40 cyc/op, section
4.2).  Zero cost today; total cost the moment a multi-outstanding core
arrives without ID rotation.  Invisible to every existing test, because no
existing master can generate the stimulus — `tb_l2c`'s own requester only
exposes it because the perf suite added a deliberately single-ID scenario
(`miss_read_single_id`).

This is `docs/soc_bus_review.md` **F4**.  Do not land it alone: it is one of
four limits in series (section 4.4), and on its own it measures as nothing.
Land it with F1 and F2, and put ID rotation in the socket contract
regardless (section 4.3).

Implementation and cost: 7.3.

### 5.1 [#1 today, not landed — needs a change outside `l2c*`] The RAM-disk bypass window is on the wrong side of the address decode

**Measured**: `bypass_read_512B` = **1,376 cyc = 37.2 MB/s** at L=40
(6,496 cyc = 7.9 MB/s at L=200).  Exactly `32 x (L + 3)` — every 16 B beat
pays a full DDR round trip.  The identical 512 B through the cacheable path
is 506 cyc; a native 32-beat pass-through burst would be ~72 cyc (~700 MB/s)
— **18x** what the bypass engine delivers.

**And it stalls the CPU.**  `mixed_cpu_hits_plus_bypass` measures 256 CPU
hits interleaved with 4 x 512 B block reads: 5,942 cycles, i.e. **23.21
cycles per CPU hit against a 3.00-cycle baseline (7.7x)** at L=40 and
**101.51 cycles (34x)** at L=200.  Mechanism: the front door has exactly one
AR register (`ar_have`, `rtl/soc/l2c_ctrl.v:119`), a 32-beat burst occupies
it for all 32 beats, and `l2c_bypass` (`rtl/soc/l2c_bypass.v:204`,
`req_ready = (st == S_IDLE)`) will not take the next beat until the previous
beat's response has fully retired.  Every CPU read behind it waits.

`docs/l2c_spec.md:432` predicted exactly this: the engine "was sized for the
VRAM-in-DDR carve-out ... VRAM writes are infrequent relative to DRAM
latency" and warns a bandwidth-hungry consumer "would not be fine" and needs
"their own multi-outstanding bypass engine".  `rtl/soc/fpga_top_ddr.vh:145-148`
then pointed it at a 256 MB streaming block device.  **The mismatch is real
and the spec called it in advance.**

**Recommended fix — and it is a deletion, not an addition.**  Do for the RAM
disk exactly what T16 did for VRAM: give the aperture its own `axi_xbar`
slave port so it never reaches `xbar S0`, and merge it downstream in
`axi_vram_priority_mux3` (or a 4th lane).  Then `vhdd_ddr`'s 32-beat bursts
reach DRAM unmodified, and **the entire bypass mechanism inside `l2c` can be
deleted**:

| Deleted | Where | Post-route LUT |
|---|---|---|
| `l2c_bypass` instance | `rtl/soc/l2c.v:199-218` | 213 LUT, 320 FF |
| bypass leg of the AR/R route queue (`rdq_src`, pointers, `ar_hold_*`) | `rtl/soc/l2c.v:225-285` | ESTIMATE ~150-250 LUT + ~20 FF |
| the whole AW/W/B master arbiter (it exists *only* to arbitrate bypass vs. victim) | `rtl/soc/l2c.v:286-326` | ESTIMATE ~120-200 LUT |
| third leg of the R/B response mux | `rtl/soc/l2c.v:352-399` | ESTIMATE ~80-150 LUT |
| `byp_*` ports, `is_bypass_c`, `do_bypass_c`, `byp_active` term of `id_busy_c` | `rtl/soc/l2c_ctrl.v:72-77,148,155,190,330-341` | ESTIMATE ~60-120 LUT |

ESTIMATE total **~620-950 LUT and ~340 FF** out of `l2c`, one 288-line file
and three arbiters gone, plus the disappearance of the priority-inversion
hazard that `docs/l2c_spec.md:349-407` documents at length ("bypass wins
every one of these three arbitrations" exists solely because a stuck bypass
response wedges the whole front door).  **Throughput up ~18x on the disk
path, CPU stall during disk I/O eliminated, and `l2c` gets smaller and
simpler.**  Best ratio of any change that is *independently measurable on
the machine that exists today* — 5.0 outranks it once the new core lands,
but 5.0 cannot be landed or measured alone (section 4.4).

I did not implement it: it requires editing `rtl/soc/axi_xbar.v` and
`rtl/soc/fpga_top_ddr.vh`, outside the files I own for this task.  The
`l2c`-side deletions are mechanical once the decode moves.

Interim mitigation if the decode change is not wanted yet: nothing cheap
exists.  Making `l2c_bypass` multi-outstanding means giving it its own
ordering tracker, which is more logic and more complexity for a path that
should not be there at all.  Do the decode move.

### 5.2 [#2, LANDED] `m_line`'s three write sources collapsed to one

`rtl/soc/l2c_mshr.v:206-233`.  Only one MSHR entry is ever installed or
replayed at a time (`act`), and the walk cannot start until that entry's
fill is complete (`scan_vec_c` requires `m_fill_done`), so no beat can land
in it afterwards.  The merges therefore never needed random access into the
4096-bit array: the chosen line is snapshotted into a single 512-bit
`act_line` register at `S_SCAN`, every merge happens there, and `m_line`
becomes a **pure fill-assembly buffer written only by returning R beats** —
whose D input is now `m_rdata` directly, with **no data mux at all**.

ESTIMATE, from the 3-LUT-per-bit post-route evidence in section 3:

- removed: ~12,300 LUT (the per-bit write mux)
- removed: ~4,800 LUT (five flat dynamic reads become quadrant picks off a plain register)
- added: ~1,000 LUT (`act_line`'s own 3-source 512-bit mux) + 512 FF
- added: ~1,200 LUT (the one remaining `m_line[scan_idx_c]` 8:1 read)
- **net ESTIMATE ~14,000-15,000 LUT saved**, taking `u_mshr` to roughly
  5,000-7,000 LUT and `u_l2c` to roughly 9,000-11,000 LUT.

Confidence: the 12,300 term is solidly evidenced (per-bit cell names in the
post-route-era synth report); the 4,800 term is a structural estimate and
could be materially smaller if Vivado was already sharing those reads.  A
sensible planning figure is **10,000-14,000 LUT**, i.e. 5-7% of the device
— and, at congestion level 6, ~12 K fewer cells in one module is congestion
relief as much as it is area.

Timing: strictly better.  The removed arc was a merge cone fanning out to a
4096-bit register array; the added arcs are shorter.  `u_mshr` does not
appear in the worst-path list of `timing_place.rpt`, so there is no critical
path to disturb.

Verified: 31/31 `tb-l2c` (including `mshr_replay_partial_write`, which is
exactly the read-after-write-within-one-entry case this restructure has to
preserve), 3 x 50,000-op `tb-l2c-stress` seeds, 4/4 `tb-l2c-chain`, 17/17
`tb-vram-ddr-chain`, `tb-l2c-bypass-all`, `make lint` clean.

### 5.3 [#3, LANDED] The hit response no longer costs a state

`rtl/soc/l2c_ctrl.v:123-130, 253-254, 353-392`.  `S_HITRESP` existed only to
hold the op for the cycle its R/B drained — a 4th cycle on **every hit**,
even when the response was accepted immediately.  `hit_rsp_*` is now a plain
1-deep skid: `S_LOOKUP` loads it and returns straight to `S_IDLE`, and only
retries when the skid is still full (`hit_rsp_block_c`).

**Measured: 4.00 -> 3.00 cyc/op sustained (400 -> 533 MB/s, +33%);
64 B hit burst 16.00 -> 12.01 cyc.**  Latency unchanged.  One FSM state
removed; no new state, no new register.  Area-neutral to slightly negative.

Misses are deliberately *not* gated on the skid — their response comes from
`l2c_mshr`'s own channel much later — so the miss numbers are unchanged, as
the table confirms.

No new RAW hazard: `st <= S_IDLE` takes effect at the *end* of `S_LOOKUP`,
so the next op presents its array address one cycle after any hit-write
lands, exactly as before.

### 5.4 [#4, LANDED] One shared way-select read mux in the hit/victim path

`rtl/soc/l2c_ctrl.v:269-277`.  The hit-data path used a flat 128-bit 32:1
select (8 ways x 4 quadrants) *on top of* the dirty-victim path's 512-bit
8:1 select.  The two are mutually exclusive by construction — a victim is
only pushed on `s_lookup_miss_go`, which requires `!any_hit_c` — so one
512-bit 8:1 mux now serves both, with a cheap 4:1 quadrant pick after it.

ESTIMATE **~600 LUT** saved.  Small, free, and the code is shorter.

### 5.5 [LANDED, but the recommendation is withdrawn] `MSHR_REPLAY_N` is now a parameter

`rtl/soc/l2c.v:47-61` -> `l2c_ctrl` -> `l2c_mshr`.  Default **unchanged at
4**, so no existing build moves.

**An earlier draft recommended flipping it to 2** on the grounds that only
three masters exist, each one-outstanding, so at most two secondaries can
ever queue behind one primary.  **That justification is withdrawn** for the
same reason as the `N` recommendation in 4.2: a multi-outstanding core with
rotating IDs can put many distinct IDs on one line concurrently, and then
the replay FIFO saturates at any depth.  Keep the default at 4.

What the validation *does* still support, and it is worth recording
precisely because it is stronger than the withdrawn argument:

- At `MSHR_REPLAY_N=2, MSHR_REPLAY_K=1` the full suite is **31 PASS / 0
  FAIL**, including the 20,000-op randomized scoreboard driven by a
  requester running **24 concurrent transactions across 59 distinct AXI
  IDs**, reaching **max MSHR occupancy 8**.  So the resized replay FIFO is
  *correct* under a genuinely multi-outstanding, multi-ID master — this is
  not a single-outstanding-only result.
- What that run cannot tell you is the **throughput** cost, because the
  suite is a correctness scoreboard, not a benchmark.  Saturation shows up
  as `merge_ready` deasserting and `S_LOOKUP` retrying — a stall, never a
  wrong answer — and nothing in the suite measures how often.

So the parameter is a genuine knob with a proven-safe floor, and it should
be left at 4 until someone measures replay-FIFO occupancy under the new
core's real traffic.  ESTIMATE if it is ever taken: ~2,300 FF and
~900-1,400 LUT.

`test_mshr_merge_queue_saturation` had the value 4 baked into it; it is now
written against a `L2C_REPLAY_N` compile define defaulting to 4
(`tb/tb_l2c.cpp:891-902`), so a resized FIFO is testable rather than looking
like a failure.  That tb change is worth keeping either way.

### 5.6 [#6, LANDED] Dead output removed

`l2c_mshr.inst_tag_valid` was a registered output tied to `()` at its only
instantiation (`rtl/soc/l2c_ctrl.v`, formerly wrapped in a
`lint_off PINCONNECTEMPTY`).  Removed, along with the lint suppression that
existed to hide it.  1 FF plus its enable logic.  Trivial area, but it is
the exact species the coordinator flagged: state kept for a consumer that
does not exist.

---

## 6. Dead, vestigial and over-general logic — full inventory

| Item | Verdict | Cost |
|---|---|---|
| `l2c_mshr.inst_tag_valid` | **Dead — removed** (5.6) | 1 FF |
| `L2_BYPASS_ALL` | Permanently 0 in every SoC build; only `tb-l2c-bypass-all` sets it | **Zero silicon** (generate branch). Keep: it is a genuine bring-up escape hatch and costs only reading time |
| `NUM_BYPASS_WINDOWS` | 1 at every instantiation | **Zero silicon** at 1 (generate loop). Becomes moot entirely under 5.1 |
| `CACHEABLE_BASE` / `CACHEABLE_SIZE` | Consumed **only** by the `translate_off` disjointness assert (`l2c_bypass.v:168-189`) | Zero silicon. Note they are *not* kept in sync with the real S0 span, which is fine only because they are simulation-only |
| `EXTERNAL_WRITE_RESET_RECOVERY` | Default 0 is dead — the sole instantiation passes 1 (`fpga_top_ddr.vh:149`) | Zero. Consider making 1 the default so the shipped path is the default path |
| `dbg_hit_count` / `dbg_miss_count` | **Live** (VIO `probe_in21`), but they are **two free-running 32-bit counters in the hot front-door module** — the exact shape that killed two builds tonight | ~64 FF + ~70 LUT + two long routes to the VIO. Recommend gating them behind the same `ifdef` as the VIO that reads them |
| `dbg_write_snap` / `dbg_master_snap` | Only consumed inside `ENABLE_ILA`; optimised away otherwise | Zero when ILA is off |
| `l2c.v` AW/W/B arbiter + `rdq` source queue | Exist **only** to arbitrate bypass against victim/MSHR | ESTIMATE ~250-450 LUT, all recoverable under 5.1 |
| `l2c_victim` 2 entries | Genuinely used (concurrent same-set dirty evictions are a tested scenario) | 569 LUT / 1,089 FF — leave alone |
| `l2c_mshr` `N=8` | **Not dead — leave alone.** Unreachable today only because the masters are one-outstanding; `m68k-core-040-ooo` makes it reachable (section 4.2). An earlier draft's proposal to shrink it is withdrawn | — |

---

## 7. Costed proposals, not implemented

### 7.1 `m_line` -> BRAM

After 5.2, `m_line` is **written once per beat and read once per walk** —
i.e. a stream, not a register file.  That is exactly the shape that belongs
in BRAM, and BRAM is ~52% free (section 2).  Organise it as 32 x 128 bits
(8 entries x 4 quadrants): one write port taking the fill beat, one read
port walked 4x during `S_SCAN` to load `act_line`.  Removes the remaining
4,096 FF and the ~1,200-LUT 8:1 read mux for **1 RAMB18** and 3 extra cycles
per miss (against a 40-200 cycle DDR round trip — unmeasurable).
ESTIMATE: **-4,096 FF, -1,000 LUT, +1 RAMB18.**

This scales *with* `N` rather than against it: it is the change that makes a
**deeper** MSHR affordable if the new core turns out to want more than 8
entries, since additional entries then cost BRAM rather than fabric
registers and their write mux.

> An earlier draft also proposed `N` 8 -> 4.  **Withdrawn** — see 4.2.  The
> capability is about to be needed.  (For the record, it would not have been
> a safe parameter flip anyway: `m_rready` (`l2c_mshr.v:245-246`) indexes
> `m_v[r_idx_c]` with a 3-bit ID that can exceed `N-1` on a stale or
> misrouted R beat, `l2c_pri8` is fixed at 8 inputs, and the candidate
> vectors are hardcoded `[7:0]`.)

### 7.2 Replay storage -> LUTRAM FIFO

`r_wdata`/`r_wstrb`/`r_id`/`r_qoff`/`r_last`/`r_need` are 32 slots x ~155
bits with a 32:1 read mux.  Read strictly in FIFO order during the walk, so
distributed RAM fits perfectly: ~155 bits x 32 deep is ~80 LUTRAMs against
~4,960 FF and a ~1,300-LUT mux today.  ESTIMATE **~1,000-1,500 LUT and
~4,900 FF**, at the cost of one extra cycle per replay step.  Note this supersedes 5.5's parameter flip rather than
complementing it: a LUTRAM FIFO makes the depth nearly free, so the right
move is to keep `REPLAY_N=4` (or raise it) *and* stop paying fabric
registers for it.

### 7.3 Relaxing the same-ID accept gate — required work, not optional

Cross-reference: `docs/soc_bus_review.md` **F4**; measured cost in 4.2
(6.6x); sequencing constraint in 4.4.

> An earlier draft called this "not needed today and actively harmful to do
> now, because it is the only thing keeping the 8-entry MSHR from being
> needed."  **Withdrawn.**  That argument only held while the masters were
> one-outstanding, and it had the incentive exactly backwards: the gate is
> not protecting the MSHR from being useful, it is preventing the MSHR from
> being used.

Two steps, either of which is independently correct:

**Step 1 — same-line merge (cheap, fixes bursts outright).**  Allow a
same-ID accept when the new op would merge into the *same* MSHR entry.  The
replay FIFO is strictly FIFO, so per-ID response order is preserved with no
extra tracking.  Needs a line-match check at accept time, i.e. a second
lookup port on `l2c_mshr` fed by `cur_set`/`cur_tag` (today `lu_set`/`lu_tag`
are driven from the registered `req_*`, one cycle too late).  Eight 26-bit
comparators: ESTIMATE **~250-350 LUT**.

This alone repairs every multi-beat burst from a fixed-ID master.  Today a
64 B read burst pays a full DDR round trip on beat 0 and blocks beats 1-3
behind it even though they hit the very line being filled — measured
`stream_read_64B` 63.84 cyc/op and `cacheable_read_512B` 505.83 cyc/op
(1.1), where the latter is 8 *serial* round trips for one 512 B burst.

**Step 2 — allocation-order completion (a deletion).**  Service MSHR
completions in allocation order instead of round-robin, which makes
cross-entry same-ID ordering hold by construction.  `rr_ptr`, the
rotate/encode pair (`l2c_mshr.v:144-147`) and the whole starvation-avoidance
argument in `docs/l2c_spec.md:300-320` are replaced by a FIFO that cannot
starve.  ESTIMATE: **area-negative, ~100-200 LUT recovered.**

Remaining case after both steps: a same-ID *hit* issued behind a same-ID
*miss* must still not overtake it.  That one genuinely needs a hold, but it
is a hold on one op rather than on the whole front door.

### 7.4 Two-stage front door (3 -> 2 cyc/hit)

This addresses only the *rate* half of `docs/soc_bus_review.md` **F2**.
F2's other half — the front door being beat-granular with a single global
AR register (`l2c_ctrl.v:119`), which serialises whole bursts and blocks
every other master for their duration — is the concurrency half, is much
more valuable, and is a different change.  Do not confuse the two: making
hits 2 cycles instead of 3 does nothing for a master that cannot get a
second AR accepted.

Duplicating `req_*` (addr/set/tag/id/wdata/wstrb/qoff/last/need/illegal
~= 190 FF) plus a second copy of the victim/MSHR/skew hazard checks and a
same-set forwarding case between the two in-flight ops.  ESTIMATE **+400-700
LUT** for +50% hit throughput (533 -> 800 MB/s).  **Not now**: it is
area-positive in a design at 86.2% LUT and congestion 6, and it adds a
hazard case rather than removing one.  The transaction-granular rework
(F2 proper) should be designed first — if that lands, it may subsume this
one, and doing them in the other order means paying for the pipeline
registers twice.

### 7.5 Full-line write without a fill — LANDED 2026-08-19, see section 11

An earlier revision of this section said "not recommended", on the grounds
that buffering a burst's beats before dispatch was real complexity for a
case only `memset`-like traffic hits.  **That is WITHDRAWN.**  The
measurement in section 11 shows it is worth **4.4-5.1x on every streaming
write the machine does**, the buffering turned out to be ~90 lines confined
to `l2c_ctrl.v`'s front door, and it costs no fill-path or MSHR change at
all.  The judgement was wrong because it was made without measuring the
streaming-write path end to end.

---

## 8. L1/L2 snooping — can it be cheap without sacrificing performance?

### 8.1 Is L2 inclusive of L1?

**At fill time, yes; as an invariant, no — and this was not written down
anywhere.**

Every L1 fill of a DDR-backed address reaches L2: L1-D misses go out on the
CPU D-side master -> `axi_narrow_to_wide` -> xbar M0 -> S0 -> `l2c`, and
I-fetch misses go `if_to_axi` -> M2 -> S0 -> `l2c`.  `l2c` allocates on both
(`docs/l2c_spec.md:221-278`).  L1-D writebacks arrive as ordinary AXI writes
and are write-allocated.  Non-cacheable L1 bypass stores also pass through.
So **nothing enters L1 without passing through L2**.

But **L2 never back-invalidates**: `s_lookup_miss_go` picks a PLRU victim
and invalidates the tag (`rtl/soc/l2c_ctrl.v:381-392`) with no notification
upward.  L2 is 2 MB/8-way against L1-D's 4 KB/4-way, so eviction of a
still-L1-resident line is uncommon but entirely possible.  Therefore
**absence from L2 does not imply absence from L1**, and the L2 tag array is
*not* a sound snoop filter as it stands.  This should be added to
`docs/l2c_spec.md` as an explicit non-invariant.

### 8.2 There is no incoherent DMA writer into Mac RAM today

Worth stating before costing anything.  Q700 SCSI is *pseudo*-DMA — the CPU
copies block data with MOVE loops — so disk data enters RAM through the
CPU's own L1.  The RAM disk's AXI master writes only the RAM-disk aperture
(`0x7000_0000` -> DDR `0x5000_0000`, `axi_defs.vh:99,212`), which does not
alias cached Mac RAM.  The remaining DDR writers are `boot_fsm` (pre-boot,
caches off) and JTAG/xdma (CPU halted).  **So this is insurance for the
planned DMA engine (`docs/dma_engine_design.md`) and future GbE/M.2 — not a
live bug.**  That matters for ranking: it must not displace 5.1 or 5.2.

### 8.3 The expensive half is already built

`cpu/rtl/core/mem/dcache.v` already exposes **both** directions:

- **Outbound invalidate** (`dcache.v:241-243`): `snoop_valid` pulses one
  cycle on any D-side write that changes memory; `snoop_addr` is the 32 B
  line-aligned physical address.  `icache.v` consumes it in an always-on
  invalidate block.  This is the proven broadcast template.
- **Inbound probe** (`dcache.v:245-250`): `snoop_query_valid`/
  `snoop_query_addr` -> `snoop_resp_valid`/`snoop_resp_hit_dirty`/
  `snoop_resp_data[127:0]`, a 3-stage pipeline with four dedicated per-way
  synchronous read ports.  **It already returns the dirty data.**  `icache.v`
  drives it on every I-cache miss (`S_PRE_FILL_SNOOP`).

So neither direction needs a new L1 structure.  What is missing is an
arbiter on the probe port (today only the I-cache drives it) and an L1-D
inbound *invalidate* port (L1-D has none; L1-I does).

### 8.4 Direction 1 — DMA write, invalidate L1: nearly free, confirmed

L2 sees every DMA write at its front door.  Source
`{snoop_inv_valid, snoop_inv_addr[31:6]}` from the accepted-write path in
`l2c_ctrl` (the `write_sel && do_accept_c` case at
`rtl/soc/l2c_ctrl.v:329-348`), qualified by "requester is not the CPU"
— a compare against the xbar's master-index field of `cur_id`, which is
`s_axi_arid[5:4]`/`awid[5:4]` given `XID_WIDTH = ID_WIDTH + 2`
(`rtl/soc/axi_xbar.v:282`).

Cost in `l2c`: 1 valid FF + 26 address FFs + a 2-bit compare.
**ESTIMATE ~30 FF, ~10 LUT.**  Confirmed nearly free.

Cost outside `l2c` (described, not implemented): L1-I already has the
consumer; L1-D needs an invalidate port shaped like its existing per-line
maintenance walker.  **Granularity: one L2 line is 64 B and one L1 line is
32 B, so each broadcast must invalidate BOTH halves** — either two pulses or
a 64 B-aligned address that the L1 side expands to two lines.  Getting this
wrong leaves the odd half stale, which is exactly the class of bug that
looks like random corruption weeks later.

### 8.5 Direction 2 — DMA read, L1 may hold dirty data

Options, costed:

| Option | Cost | Verdict |
|---|---|---|
| (a) Probe L1 on every DMA read | 4 probes per 64 B line (the probe returns 16 B), 3 cycles each plus contention with I-fetch misses. ~12-16 cycles per 64 B and it steals the port `icache.v` uses on **every** I-miss | Correct but pays on every access. Reject as the default |
| (b) Exact 128-entry inclusion directory (L1-D is 4 KB = 128 lines) | Storage is trivial (128 x 28 bits) but the DMA probe needs a **128-way CAM**: ESTIMATE ~700-900 LUT. Worse, L2 cannot infer L1's replacement choices, so L1 must report every fill *and* every eviction — a new interface | LUT-positive on a LUT-bound device, and it needs L1 changes. Reject |
| (c) 1 bit per L2 line "may be in L1" | 4096 sets x 8 ways x 1 bit = 32 Kbit. **Not free in the existing tag array**: `l2c_tags` entries are `2 + TAG_BITS` = 16 bits and 8 ways x 4096 x 16 b already maps to exactly 16 RAMB36 (plus 1 for PLRU = the 17 reported), so a 17th bit needs the tool to repack 2 x (4096x8) as 4096x18 — likely, since 2 RAMB36 give 4096x18 just as easily, but **I cannot confirm a repack without a synth run**. Worst case +8 RAMB36, which the ~125 free blocks absorb. **The soundness problem is the real issue**: the bit dies when L2 evicts the line, so this option *requires* adding L2 back-invalidation to make inclusion an invariant | Good if you also want inclusion; the back-invalidate walk is the expensive part |
| (d) **Hash-indexed presence filter, separate from the tags** | A 16 Kbit array (**1 RAMB18**, from ~125 free) indexed by a hash of `addr[31:6]`; set on every CPU-D miss or write-hit, never cleared except by a full clear whenever L1-D is fully flushed (which the CPU already does — `flush_all_req`/CPUSH ALL). No false negatives, so it is sound **without** requiring inclusion or back-invalidation. False positives grow between clears and degrade gracefully toward option (a) | **Recommended.** ESTIMATE ~50-100 LUT + 1 RAMB18 in `l2c`, no L1 change beyond the probe-port arbiter |
| (e) No hardware; driver `CPUSH` | Zero area. Works only if every DMA buffer is flushed by software, which is what classic Mac drivers actually did on 68040 | The honest fallback, and the right answer if the DMA engine is deferred |

**Recommended probe shape** (cheapest and simplest): on a filter hit, do a
**tag-only** probe per 32 B half; if `snoop_resp_hit_dirty`, force L1 to
write the line back through its existing per-line maintenance walker rather
than returning data through the snoop data path.  The writeback then arrives
at L2 as an ordinary AXI write and is absorbed by the normal write path.
**No new data path anywhere** — the common case (clean or absent) costs only
the tag probe, and the dirty case reuses machinery that already exists and
is already tested.

### 8.6 L1 tag bandwidth under realistic and worst-case DMA

- **Filter miss (the common case): zero probes.**  That is the whole point.
- **Realistic case** — DMA into a buffer the CPU recently touched, e.g. a
  disk read into a Mac RAM buffer: filter hits, 2 tag probes per 64 B L2
  line at 3 cycles each ~= 6 cycles per 64 B if the port is free.  Against
  the measured 8.38 cyc/64 B of miss-path throughput that is roughly a 2x
  slowdown **on the DMA stream only**.
- **Worst case** — a stream whose lines are all L1-resident and dirty: every
  line becomes a forced L1 writeback, and I-fetch misses queue behind the
  probes on the shared port.  `dcache.v`'s own comment warns this port sits
  on a hot path ("this path is on EVERY I-cache miss fill ... its latency is
  not free for IPC").  **Mitigation: give the probe port a fixed-priority
  arbiter with I-fetch winning**, and let DMA probes take the idle cycles.
  DMA is throughput-sensitive and can absorb jitter; I-fetch is
  latency-sensitive and cannot.  State that ordering explicitly wherever the
  arbiter lands.

### 8.7 Interaction with the MSHR work

Same move, same reasoning: fabric registers -> BRAM, because BRAM is what
this device has spare.  7.1 (`m_line` -> 1 RAMB18) and 8.5(d) (filter ->
1 RAMB18) are independent, but if both land they should share the review
that establishes how much BRAM headroom the design keeps for the new CPU.

---

## 9. Verification

| Gate | Result |
|---|---|
| `make tb-l2c` | **31 PASS / 0 FAIL** |
| `make tb-l2c` with `MSHR_REPLAY_N=2, MSHR_REPLAY_K=1` | **31 PASS / 0 FAIL**, incl. the 20,000-op scoreboard at 24 concurrent transactions / 59 distinct IDs / max MSHR occupancy 8 — i.e. validated against a multi-outstanding, multi-ID master, not just today's one-outstanding ones (the *area* argument for taking it is nonetheless withdrawn, 5.5) |
| `make tb-l2c-stress` (3 seeds x 50,000 ops) | **3/3 seeds pass** |
| `make tb-l2c-chain` | **4 PASS / 0 FAIL** |
| `make tb-l2c-bypass-all` | **3 PASS / 0 FAIL** |
| `make tb-vram-ddr-chain` | **17 PASS / 0 FAIL** |
| `make lint` | clean, exit 0 |

`tb-vram-ddr-chain-nol2c` was not run: it has a known pre-existing failure
(three L2 scenarios inside an explicitly no-L2 build) unrelated to this work.

No Vivado was run.  Nothing was committed.

Neither the same-ID gate (5.0) nor any of the F1/F2/F3 fabric limits was
touched, so nothing in this change set alters the concurrency behaviour the
sequencing in section 10 depends on.

---

## 10. The single highest-value next step

**Move the RAM-disk aperture off `xbar S0` onto its own slave port, exactly
as T16 did for VRAM, and then delete `l2c_bypass` and its three arbiters.**

It remains the top item because it is the only one that is simultaneously
the largest *measured* throughput win on the machine that exists today (18x
on the disk path; the 7.7-34x CPU stall during disk I/O simply disappears),
an area *saving* (ESTIMATE ~620-950 LUT and ~340 FF out of `l2c`), and a
deletion of one file and three arbiters — including the priority-inversion
hazard that `docs/l2c_spec.md` spends half a section explaining.  It is also
**independently measurable**, which almost nothing else in section 4.4's
series is.  `docs/soc_bus_review.md` reaches the same conclusion as its F6.

### Sequencing, explicitly

1. **Now, independently landable and independently measurable:** the RAM-disk
   decode move (5.1) and the area work already landed here (5.2-5.4, 5.6).
   None of these depend on the fabric's concurrency limits.
2. **Before `m68k-core-040-ooo` is integrated, as one change set:** F1
   (crossbar per-master outstanding), F2 (`l2c` transaction-granular front
   door), F3 (VRAM lane mux `MAX_BULK_AHEAD`) and F4 (5.0 / 7.3, the
   same-ID gate).  **Landing any subset measures as zero.**  Budget them as
   one item, review them as one item, and benchmark only after all four.
3. **In the socket contract, before the core arrives:** ID rotation on both
   CPU masters (section 4.3).  This costs the fabric nothing and removes the
   single most likely source of a wrong conclusion during core bring-up.
4. **After a synth run confirms where 5.2 actually landed:** `m_line` -> BRAM
   (7.1).  It is also what makes a *deeper* MSHR affordable if the new core
   wants one.

The thing most likely to go wrong is not any of these changes.  It is
someone landing step 2 partially, measuring nothing, and concluding the L2
was never the problem.  Sections 4.4 and 5.0 exist to make that harder.

---

## 11. Full-line write without a fill — measured

Landed 2026-08-19 in `rtl/soc/l2c_ctrl.v` (`flw_*`, `gst`).  Retires
section 7.5's "not recommended".

### 11.1 The defect

Every write miss in `l2c_ctrl` was a read-allocate.  Zeroing 256 MiB
dragged 4 Mi 64 B lines in from DDR purely to overwrite every byte of them
and write them back dirty.  The cost is per LINE, so burst length cannot
amortise it — measured on the real boot write path, going from narrow
AWLEN 15 to 255 bought 3%, while removing `l2c` from the chain entirely
bought 4x.

### 11.2 What was built

**Allocate-without-fetch**, not write-through / no-allocate.  `l2c` is the
SoC-wide point of coherency and does not snoop, so whatever it holds is
the newest copy and every other master's read reaches DDR only through it.
Allocating preserves that: the line lands valid+dirty with the written
data and any reader hits it.  Write-through would open a window in which
the newest data is in flight to DDR and present in NEITHER `l2c` nor (yet)
DRAM, so a concurrent fill for the same line could return pre-write data
and install it — a stale-data state not observable today — and would need
a new "in-flight write-through address" hazard query on the fill path, the
analogue of `victim_query_hit`.

Detection is **conservative and burst-shape only**, decided from the AW
header: INCR, `awsize` = 16 B, 64 B-aligned base, `(awlen+1)` a multiple
of 4.  Beats of such a burst are gathered four at a time at the front door
(they never enter the tag pipeline individually) and the assembled line is
dispatched as ONE request.  All strobes set -> install valid+dirty with no
fill.  Any beat partial -> the gather replays the four quadrants as
ordinary per-beat requests, which is bit-for-bit the pre-existing
behaviour, at a cost of 4 extra cycles on a line that is about to pay a
~70-cycle fill anyway.  The gather only arms when the FIRST beat already
has all 16 strobes set, so a byte-enabled store stream never enters it.

The MSHR is not involved: no `alloc`, no `merge`, no AR.  The install
reuses the existing victim-selection, dirty-eviction and PLRU machinery
unchanged, and writes tag and data on the SAME clock edge, so there is no
cycle in which the tag advertises a line whose data has not arrived.  A
full-line write that finds the line already resident takes the ordinary
hit-write path with a full-line strobe.  A full-line write that finds an
in-flight MSHR fill for its own line retries until that entry installs and
then resolves as a hit — it never merges (an MSHR entry holds one 128-bit
quadrant) and never allocates.

### 11.3 Measured

`make tb-l2c-wstream` / `tb-l2c-wstream-off` (new, `tb/tb_l2c_wstream.v`):
the real boot write path, `32-bit master -> axi_narrow_to_wide -> [l2c] ->
axi_async_bridge -> axi_ddr4_mig_bridge -> sim_mig_backend`, single
outstanding narrow transaction, W beats back to back — boot_fsm's exact
shape.  `sim_mig_backend` at a fixed 64-cycle read latency, no jitter, no
command stalls.  256 MiB column is the extrapolation at the real 100 MHz
`core_clk`.

| narrow AWLEN | before (cyc/word) | after (cyc/word) | speedup | 256 MiB before | 256 MiB after |
|---|---|---|---|---|---|
| 0   | 11.188 | 11.188 | 1.00x | 7.508 s | 7.508 s |
| 15  |  6.625 |  1.500 | 4.42x | 4.446 s | 1.007 s |
| 63  |  6.484 |  1.312 | 4.94x | 4.352 s | 0.881 s |
| 255 |  6.449 |  1.266 | 5.09x | 4.328 s | 0.849 s |

For reference, the same traffic with `l2c` NOT ELABORATED at all
(`tb-l2c-wstream-off`) — the ceiling the 2026-08-19 measurement identified:

| narrow AWLEN | l2c removed (cyc/word) | 256 MiB |
|---|---|---|
| 0   | 15.000 | 10.066 s |
| 15  |  1.875 |  1.258 s |
| 63  |  1.406 |  0.944 s |
| 255 |  1.289 |  0.865 s |

**The cache is now FASTER than not having it** on this traffic at every
burst shape it can catch, because the write retires into SRAM instead of
crossing the CDC to DDR.  The remaining limit is `axi_narrow_to_wide`'s
gather (4 narrow beats per 5 cycles) plus its single-outstanding AW latch,
not `l2c`.

AWLEN 0 is unchanged by construction and always will be: a single 32-bit
beat can never cover a line.  boot_fsm uses AWLEN 63.

### 11.4 Timing cost — NOT measured, no Vivado was run

The **hit resolve** path — tag compare, way select, `rd_line_c`,
`hit_rsp_*` — is untouched.  What grows is the front-door ACCEPT/select
cone:

- `write_avail` gains `flw_avail_c || (... && !flw_gath_rdy_c)`, and
  `flw_gath_rdy_c` contains `byp_match2_hit` (a masked 32-bit compare on
  `aw_cur_addr`, ~2 LUT levels) and `&s_wstrb` (16-input AND, ~2 levels).
  That cone feeds `read_sel`/`write_sel` -> `do_accept_c` -> `s_wready`
  and the `tags_raddr` select.  **Estimate +2-3 LUT levels on
  `s_wready`**, whose existing critical term is `id_busy_c`'s ID CAM.
- `tw_en`/`dw_en` gain one OR term (`s_lookup_flw_go`), a sibling cone of
  `s_lookup_miss_go`'s: **+1 level**, register -> logic -> BRAM write
  enable.
- `dw_data`/`dw_strb`/`cur_addr` go from 2:1 to 3:1 muxes.  A 2:1 and a
  3:1 are both one LUT6, so **no added depth** and essentially no added
  area on those.

If the next phase-boundary synth shows this cone binding, the two levers,
in order of preference:

1. Drop `&s_wstrb` from `flw_arm_c`.  It exists only to keep byte-enabled
   store streams on the exact pre-existing path; without it they arm the
   gather and take the 4-cycle replay penalty, and the 16-input AND leaves
   the select cone.
2. Register the per-line arm decision one beat early (at
   `aw_beat[1:0] == 2'b11`), which takes `byp_match2_hit` out of the cone
   as well.  Costs one flop and a small amount of care around a burst that
   ends on that beat.

Area: +512 FF (`flw_data`) +64 FF (`flw_strb`) + ~40 FF of control, plus
one extra masked-compare in `l2c_bypass` for the second classification
port.  Against `l2c`'s post-route 24,222 LUT / and the ~10-14 K the
section 5.2 work brought it to, this is noise.

### 11.5 Verification

`tb-l2c` gained seven scenarios.  The discriminator is the memory model's
count of ARs actually presented on `l2c`'s DDR master port: a fetched line
costs exactly one, a no-fetch install costs zero.

RED before / GREEN after: `full_line_write_no_fetch` (1 fill AR, expected
0), `full_line_write_multi_line_burst` (4, expected 0),
`full_line_write_evicts_dirty_victim` (1, expected 0).

GREEN before AND after, deliberately —
`partial_line_write_still_fetches` (one beat with a partial strobe: the
fetch is mandatory and the unwritten bytes must come back as DRAM
content), `unaligned_burst_still_fetches` (a 4-beat burst 16 B into a
line straddles two lines and fetches both), and
`full_line_write_over_resident_dirty`.  The first two are the negative
guards that stop a future widening of the detector from silently dropping
a fetch a partially-covered line needs.

`bypass_read_during_full_line_gather` exists because the first version of
this change had a real bug, found by `L2C_SEED=1` and not by any directed
test: `is_bypass_c` was qualified by `flw_avail_c` alone rather than by
`flw_avail_c && write_sel`, so a bypass-window READ that won the front
door while an assembled line was waiting took the CACHE path.  The
permanent guard is now a sim-only invariant inside `l2c_ctrl.v` — "a
bypass-window address must never be accepted onto the cache path", which
was true by construction before `is_bypass_c` became conditional and is
now asserted on every accept in every tb run.  The scenario is the
stimulus that drives the window; the assertion is what catches it.

`l2c_bypass.v` also gained an elaboration assert that every enabled window
is at least 64 B granular.  The gather classifies a line ONCE, on its
64 B-aligned base, and does not re-ask for quadrants 1-3; that is only
sound under this invariant, which every window ever passed in (>= 1 MB)
satisfies trivially.

Gates: `make lint` 8/8 configs, `tb-l2c` 38 PASS (and 38 PASS across
`L2C_SEED` 1-8), `tb-l2c-chain` 4, `tb-l2c-chain-off` 2,
`tb-l2c-bypass-all` 3, `tb-axi-xbar` 408, `tb-ddr-model` 13,
`tb-axi-ddr-contract` 9, `tb-axi-async-bridge` 14,
`tb-axi-ddr4-mig-bridge` 46, `tb-vram-ddr-chain` 17, `tb-sd-boot` 11,
`tb-sd-boot-zero` 5 — all 0 FAIL.

---

## 12. Sectored valid + dirty — measured

Section 11 removed the fill for a write that covers a whole 64 B line.
This removes it for a write that covers any 128-bit **quadrant**, and
removes the writeback for every quadrant that was never written.  Both are
the same mechanism seen from opposite ends: allocation asks "which
quadrants do I have", writeback asks "which quadrants are modified".

128 bits is not an arbitrary sub-division.  It is the width of l2c's own
datapath (an MSHR entry holds exactly one quadrant; `dw_strb` was already
64 byte-strobes), and it is **exactly the 68040's L1 D-cache line size** —
so a copyback L1 line push, which is what the incoming core's dominant
write traffic will be, is precisely one fully-covered quadrant.

### 12.1 The defect, measured

`tb-l2c-sctr` / `tb-l2c-sctr-off` (new, `tb/tb_l2c_sctr.cpp`) drive the
same chain as `tb-l2c-wstream` — 32-bit master → `axi_narrow_to_wide` →
[l2c] → `axi_async_bridge` → `axi_ddr4_mig_bridge` → `sim_mig_backend` —
but write only part of each 64 B line.  The working set is 128 sets × 16
tags, walked tag-major, so passes 8-15 are steady state: every line both
misses and evicts a dirty victim.  DDR beats are counted on l2c's own
128-bit master port (`dbg_l2m_*_fire`), where one beat is one quadrant;
the MIG port is 256 bits wide and cannot tell a 16 B writeback from a
32 B one.

BEFORE (l2c as of 00f079c):

```
  B/line   cycles/word   fill R beats/line   writeback W beats/line
     4        89.000            4.000                4.000
    16        22.750            4.000                4.000
    32        12.000            4.000                4.000
    64         1.500            0.000                3.996   <- section 11
```

The same traffic with **l2c not elaborated at all**:

```
     4        15.000            0.000                1.000
    16         3.750            0.000                1.000
    32         2.500            0.000                2.000
    64         1.875            0.000                4.000
```

So section 11's finding survived intact for anything short of a whole
line: l2c was **6.1x slower than not having a cache** at 16 B/line and
4.8x slower at 32 B/line, and it turned 1 quadrant of write traffic into
8 quadrants of DDR traffic.  A single byte written anywhere marked all
64 B dirty and pushed the whole line.

### 12.2 What was built

Valid and dirty become four bits each per way — `vsec[3:0]`, `dsec[3:0]`
— so a tag entry is `{vsec, dsec, tag[13:0]}`, 22 bits instead of 16.

- **A write that fully covers a quadrant needs no fill**, whatever the
  rest of the line looks like: pick the way, write data + `vsec[q]` +
  `dsec[q]` on one clock edge.  00f079c's whole-line case is now just
  "all four quadrants covered at once", and its AW-shape gather survives
  as a *throughput* shortcut (one tag-pipeline request per line instead
  of four), not as the thing that removes the fetch.  The gather's
  `G_RPLY` replay path — previously pure loss, four extra cycles plus a
  mandatory fetch — now installs each fully-strobed quadrant for free and
  fetches only the genuinely partial ones.
- **Only dirty quadrants are written back.**  `l2c_victim.v` emits the
  contiguous span from the lowest to the highest dirty quadrant:
  `awaddr = base + first*16`, `awlen = last - first`.

Three rules keep it correct, and all three are load-bearing:

1. **Allocation keys on tag match, not on hit.**  A line whose tag matches
   but whose requested quadrant is invalid must fill into *that way*.
   Allocating a second way for the same tag breaks the per-set tag
   uniqueness the whole design assumes.  `way_hit_c` (tag match AND
   quadrant valid) drives the response; `tag_match_c` (tag match AND any
   quadrant valid) drives way selection and eviction.
2. **A write onto a quadrant the cache already owns, on a line with a live
   MSHR entry, retries — it does not merge.**  `l2c_mshr` merges into
   `act_line`, which holds FETCHED data: correct for a quadrant that was
   invalid, stale for one the cache owns.  Retrying costs the fill's
   remaining latency and cannot livelock, because the fill completes
   without needing the front door.  This is what keeps `merge_quad` sound
   and it is why the merge path only ever sees invalid quadrants.
3. **The valid/dirty merge is a read-modify-write**, and it is sound only
   because the tag array is read at accept time and `skew_hazard_c`
   bounces any request whose set an MSHR install touched in the two cycles
   since.  Do not remove that bounce.

A quadrant inside a writeback span that is **not** dirty goes out with
`wstrb=0`.  It may never have been fetched at all, so the array holds
whatever the URAM last had; writing that to DRAM is silent corruption
rather than wasted bandwidth.  Only non-contiguous masks (`4'b1001` and
friends) have such a gap.

`l2c_victim.v` also snapshots the dirty mask into `act_dsec_r` when a
drain starts rather than reading it live: IMPORTANT-A's reset path forces
`act_idx` back to 0 while `S_RSTDRAINW` is still completing a burst DRAM
is owed, and a live read would take the wrong slot's mask and get the
filler burst's LENGTH wrong — exactly the beat-count desync that path
exists to prevent.

### 12.3 Measured

AFTER, same harness, same seeds:

```
  B/line   cycles/word   fill R beats   writeback W beats   speedup   vs no-cache
     4        89.000        4.000             1.000          1.00x     0.17x
    16         3.745        0.000             0.998          6.08x     1.00x
    32         1.873        0.000             1.998          6.41x     1.34x
    64         1.500        0.000             3.996          1.00x     1.25x
```

At 16 B/line — one 68040 L1D line, fully strobed — l2c now **exactly
matches** the no-cache ceiling (3.745 vs 3.750 cyc/word) while still being
a cache, and DDR traffic per touched line falls from 8 quadrants to 1.  At
32 B/line it is 1.34x faster than not having a cache.

`tb-l2c-wstream` is unchanged to the digit — 11.188 / 1.500 / 1.312 /
1.266 cyc/word at narrow AWLEN 0/15/63/255 — so section 11's case has not
regressed.

The 4 B/line row is untouched by design: a write covering 4 of a
quadrant's 16 bytes genuinely needs the other 12 from DRAM, so the fill is
mandatory and its ~64-cycle round trip is what those 89 cycles are.  Its
writeback did drop from 4 beats to 1.  Shortening the fill AR to just the
missing quadrant (4 R beats → 1) is a self-contained follow-up inside
`l2c_mshr` — see 12.5.

### 12.4 Timing cost — NOT synthesised, structural estimate

No Vivado was run.  The analysis below is against
`build/vivado/reports/timing_place.rpt`, which predates 00f079c.

**The binding path is not touched at all, and that is the whole point.**
The design's worst `fabric_clk100` setup path — worst of the entire SoC —
is the l2c front-door accept cone:

```
  Slack (MET) 0.077ns of 10.000ns
  Source:      u_xbar/ws_slv_reg[0][1]/C
  Destination: u_boot_n2w/aw_wrx_left_q_reg[0]/D
  Data Path:   9.948ns  (logic 1.747ns 17.6%,  route 8.201ns 82.4%)
  21 logic levels; 6 of them are u_mshr's idq_busy ID CAM, ending at
  u_ctrl/s_axi_wready_INST_0
```

The next four worst paths (0.198-0.250 ns) are the same cone; after those
the worst is the CPU FPU divider at 0.405 ns.  **No l2c-internal endpoint
appears in the ten worst paths of this clock group at all** — so hit
resolve, the tag write and the victim path all have at least 0.407 ns of
slack, and the reported gap to the next group suggests far more.

Sectoring adds **nothing** to that cone: `do_accept_c` / `s_wready` read
no tag-array state whatsoever.  Two notes worth keeping:

- The cone is 82% ROUTING, and it re-enters l2c on `s_wvalid` and leaves
  on `s_wready`, crossing the xbar→l2c→xbar→n2w hierarchy boundary twice.
  It is a placement/congestion path, not a logic-depth path.  Anything
  that raises congestion around `u_l2c`/`u_xbar` charges it directly.
- The final `s_wready` LUT is a LUT5 in this report.  00f079c's added term
  most likely consumed its last free input at zero depth cost — which
  means **there is no free input left**: the next term added to that cone
  costs a full extra level, and 0.077 ns does not buy one.

Per-path, for the changes that were made:

| path | before | after | note |
|---|---|---|---|
| accept cone (`s_wready`, `do_accept_c`) | binding, 0.077 ns | **unchanged** | reads no tag state |
| hit resolve (`way_hit_c` → `rd_line_c` → `hit_rsp_*`) | ~7 levels | **+0 levels** | see below |
| tag-array write data (`tw_vsec`/`tw_dsec`) | 1 level | **+2-3 levels** | the RMW; had ≥0.407 ns |
| victim AW/W (`m_awaddr`, `m_wstrb`, `m_wlast`) | fixed | **+2 levels** | 4-bit priority chains |

Hit resolve is the claim worth spelling out, because getting it wrong
would be much more expensive than area.  Comparing `TAG_BITS = 14` bits
against 14 needs 28 LUT inputs, i.e. **five** LUT6 partials at level 1,
leaving level 2 with five partials and one spare input.  Sectoring adds,
at level 1 and in parallel:

- `vsec_w[req_qoff]` — 4 data bits + 2 select bits = exactly one LUT6;
- `rvalid[w] = |vsec_w` — a 4-input OR, one LUT6 (inside `l2c_tags.v`).

Level 2 then computes `way_hit_c[w] = 5 partials + vsec_w[req_qoff]` and
`tag_match_c[w] = 5 partials + rvalid[w]`, six inputs each, in parallel.
**Depth is unchanged at two levels**; a second `l2c_pri8` runs beside the
first, and `rd_way_c` moves from `any_hit_c ? hit_way_c : victim_sel_way`
to `any_tm_c ? tm_way_c : victim_sel_way` — same shape, same arrival, and
the 512-bit way-select mux behind it is untouched.  Note the level-2 LUTs
are now **full**: a further per-way term would cost a real level.

`s_lookup_hit` gains `!(req_is_write && mshr_lu_hit)`.  `mshr_lu_hit`
arrives from registered `req_set`/`req_tag` through ~4 levels, while
`way_hit_c` arrives from a BRAM Q (~1.5 ns) plus 2 levels, so it is the
earlier signal and folds into the existing AND tree without adding depth.

Area: tag entries 16 → 22 bits moves `u_tags` from 2 to 3 RAMB36 per way,
**17 → 25 RAMB36** (+8, on a device with ~125 RAMB36-equivalents free —
BRAM is the one resource this design has spare, and URAM at 64/64 is
untouched because the data array does not change).  LUT delta is small and
additive: one extra `l2c_pri8` (~10), two 8:1-over-4b selects (~60), the
victim span chains (~40), MSHR mask registers (~30 FF).  Against 187 K
LUT that is noise as a count — but the design sits at 86% LUT with
congestion level 6, where a 33-LUT delta has previously flipped
`route_design`, so treat it as a congestion risk rather than an area one.

### 12.5 Not done

**Partial-line FETCH.**  `l2c_mshr` still issues `arlen=3` and fetches the
whole line even when it only needs one quadrant.  Fetching just the
missing span (`araddr = base + first*16`, `arlen = last - first`, beats
demultiplexed from `first` rather than 0) would take the 4 B/line row from
4 fill beats to 1.  It does not help the cycle count — that row is
dominated by the fill's round-trip latency, not its length — and it is a
self-contained change inside `l2c_mshr.v` that can land on its own.

**Deleting the gather.**  With sectoring, each fully-strobed 16 B beat
qualifies for the no-fetch install by itself, so `flw_*` no longer
contributes anything to DDR traffic — only throughput (one tag-pipeline
request per line instead of four, ~12 cycles instead of ~4+3).  At the
measured 21 cycles/line the `axi_narrow_to_wide` gather is the limit, not
l2c, so removing `flw_*` would cost nothing today **and would take
00f079c's addition back out of the binding accept cone**.  That is worth
revisiting the next time that cone is the reason a build misses timing;
it is not worth doing speculatively, because a future wide multi-beat
master would want the shortcut back.

### 12.6 Verification

`tb-l2c` 38 → 45 scenarios, 0 FAIL, and 45/45 across `L2C_SEED` 1-8.

RED before / GREEN after (built against `HEAD`'s RTL to confirm):
`sector_write_no_fetch` (1 fill AR, expected 0),
`sector_dirty_writeback_is_partial` (4 W beats, expected 1),
`sector_mixed_dirty_span` (4 beats at the line base, expected 2 at
base+16), `sector_fill_into_resident_way`, and
`unaligned_burst_still_fetches` — whose expectation this change
deliberately inverts (4 fully-strobed beats straddling two lines now cost
zero fills, and its two whole-line readbacks are what still force the
unwritten quadrants to come from DRAM).

GREEN both sides — the negative guards, verified green against `HEAD`'s
RTL too:

- `sector_partial_quadrant_still_fetches` — a write covering 8 of a
  quadrant's 16 bytes must still fetch (exactly 1 AR), and the bytes it
  did not write must read back as DRAM content.  This is the guard that
  stops the no-fetch install from being widened past full coverage.
- `sector_clean_quadrant_not_written_back` — quadrants 0 and 3 written,
  1 and 2 never written *and never fetched*, so the array holds URAM
  garbage for them and the dirty mask `4'b1001` forces a 4-beat span.
  After eviction, DRAM's copy of quadrants 1 and 2 must be unchanged.
  This is the `wstrb=0` gap guard, and it catches corruption, not policy.
- `sector_write_during_fill_retries` — Rule 2: a write onto a quadrant the
  cache owns, issued while the same line has a fill in flight for a
  different quadrant, must survive the install.
- `partial_line_write_still_fetches` and
  `full_line_write_over_resident_dirty`, unchanged from §11.

`setup_pending_eviction` in `tb_l2c.cpp` and
`test_reset_mid_l2_victim_writeback` in `tb_vram_ddr_chain.cpp` now dirty
all four quadrants of each way.  Both are about a reset landing part way
through a writeback BURST, and with per-quadrant dirty bits a one-quadrant
write evicts as a single beat — there would be no "part way" left to land
in.  `setup_pending_eviction`'s ninth access also became a partial-strobe
write so it still allocates an MSHR entry and puts a real fill in flight,
which `reset_compose_fill_and_writeback` needs on the R side.

The sim-only bypass invariant added in 00f079c ("a bypass-window address
must never be accepted onto the cache path") is untouched and still armed
on every accept in every run.

Gates: `make lint` 8/8 configs exit 0 — `tb-l2c` 45 (and 45 across
`L2C_SEED` 1-8) — `tb-l2c-stress` 3 seeds × 50 000 ops — `tb-l2c-chain` 4
— `tb-l2c-chain-off` 2 — `tb-l2c-bypass-all` 3 — `tb-l2c-bypass-window` 0
failed — `tb-axi-xbar` 408 — `tb-ddr-model` 13 — `tb-axi-ddr-contract` 9 —
`tb-axi-async-bridge` 14 — `tb-axi-ddr4-mig-bridge` 46 —
`tb-vram-ddr-chain` 17 — `tb-sd-boot` 11 — `tb-sd-boot-zero` 5 —
`tb-l2c-wstream` / `-off` unchanged.  All 0 FAIL.

---

## 13. Pipelined victim writeback — measured

§12's predecessor measurement (commit `54919fe`) established that a dirty
writeback was **round-trip serialized twice**: `l2c_victim.v` had one
drain FSM (`S_AW -> S_W -> S_B`) and `l2c.v`'s AW/W/B arbiter held its
grant from AW *presentation* to `BRESP`, so the master write port carried
exactly one transaction at a time and the second victim slot was
decorative. On conflict-eviction traffic the fit was

    cycles/op = 3.0 + 0.746 x DDR_write_latency        (R^2 ~ 1)

That commit's recommendation — deeper buffer, payload in LUTRAM, AW
arbiter that does not hold the port to B — is what this section measures.

### 13.1 What changed

- `l2c_victim.v`: `SLOTS` entries (default 8) as a three-pointer FIFO.
  The drain sequencer is `S_AW -> S_W -> S_AW`; there is no `S_B`.
  Completion is a count (`out_cnt`), and entries retire at `bptr` on B.
- Payload moved out of flops into a flat 1-D `(* ram_style =
  "distributed" *)` array, exactly the shape `878d002` had to flatten the
  MSHR replay payload into. A 2-D unpacked array does not infer here.
- `l2c.v`: the AW/W/B arbiter locks the port to one **owner** while that
  owner has writes outstanding (AXI4 has no WID, so W bursts from two
  sources may never interleave) but lets that owner run many writes in
  flight. Victim AWs are withheld as soon as bypass asks for the port, so
  bypass cannot be starved.

### 13.2 Deterministic model — `tb-l2c` `L2C_PERF=1`, same harness as `54919fe`

Conflict-eviction traffic (every op both misses and evicts a dirty
victim), cycles/op:

| scenario | DDR-40 before | DDR-40 after | DDR-200 before | DDR-200 after |
|---|---|---|---|---|
| `evict_write`, 1 outstanding | 33.35 | **5.37** | 152.72 | **20.05** |
| `evict_write`, 4 outstanding | 32.85 | **4.84** | 152.22 | **19.53** |
| `evict_write`, 8 outstanding | 32.85 | **4.84** | 152.22 | **19.53** |
| `evict_read_after_dirty` | 14.81 | **5.89** | 64.50 | **20.89** |
| `evict_write_bursty` (burst only) | 35.82 | **4.97** | 167.07 | **20.59** |

6.8x at DDR-40 and 7.8x at DDR-200, against the 3.75 cyc/op floor
`54919fe` measured by shorting B to one cycle after WLAST. **Every other
row of the perf suite is bit-identical** — all 21 hit / miss / stream /
bypass / mixed rows at both latencies, unchanged to the hundredth of a
cycle. This is a writeback-path change and it measures as exactly that.

### 13.3 Faithful chain — `tb-l2c-sctr` (real CDC + MIG bridge + posted writes)

This is the control that matters, because `tb_l2c.cpp`'s model delays B
by the full read latency where a real MIG posts. Steady state, cycles per
32-bit word:

| bytes written per 64 B line | before | after |
|---|---|---|
| 4 | 89.000 | 89.000 |
| **16** (one 68040 L1D copyback push) | **3.745** | **2.000** |
| 32 | 1.873 | **1.625** |
| 64 | 1.500 | 1.500 |

and the pressure taps at 16 B/line: victim buffer FULL **86.6% -> 0.0%**,
miss blocked on it **46.6% -> 0.0%**, peak writebacks outstanding 2.
The 4 B/line row is fill-bound (4 fill-R beats per line) and the 64 B/line
row is W-beat-bound at 4 beats per line, so neither can move; 16 B/line —
the shape a copyback L1D push actually presents — is 1.87x.

### 13.4 Does eager writeback come back on the table?

`54919fe` shelved it because the drain engine was 95-99% busy and there
was no idle to exploit. That conclusion was explicitly contingent on this
bottleneck, so it is re-measured here. "Busy" is now split in two, because
with a pipelined engine "the FSM is not idle" stopped meaning anything:

- **writeback in flight** — at least one writeback exists (the honest
  successor to the old `st != S_IDLE`).
- **AW/W sequencer busy** — the master write port's actual occupancy, i.e.
  the bandwidth a background cleaner would have to compete for.

Whole `tb-l2c` run, 2.22 M cycles of mixed traffic:

| | before | after |
|---|---|---|
| victim buffer full | 4.67% | **0.17%** |
| miss blocked on victim space | 0.97% | **0.00%** (0 cycles) |
| writeback in flight | 20% | 17.63% |
| AW/W sequencer busy | — | **2.23%** |

Conflict-eviction traffic, 8 slots (4 outstanding requests):

| | DDR-40 | DDR-200 |
|---|---|---|
| miss blocked on victim space | 37.9% (was 90.9%) | 84.6% (was 98.0%) |
| writeback in flight | 84.3% | 96.1% |
| AW/W sequencer busy | **30.9%** | **7.7%** |
| mean writebacks outstanding | 6.25 of 8 | 7.57 of 8 |

Bursty traffic (128-op bursts, 4000-cycle idle gaps) — the best case the
eager-writeback proposal was given: burst cyc/op 35.82 -> **4.97** at
DDR-40 and 167.07 -> **20.59** at DDR-200; miss-blocked within the burst
91.6% -> 39.3% and 98.2% -> 85.4%; the gaps are still ~96% idle.

**The answer: eager writeback stays shelved, and now for a stronger
reason than before.** On mixed traffic a miss is blocked on victim space
for exactly zero cycles out of 2.22 M, so there is nothing to win. On
saturated conflict traffic misses *do* still block — but the buffer is
full of writebacks **already committed to DRAM and waiting on B** (mean
7.57 of 8 outstanding at DDR-200), not of dirty lines that were cleaned
too late. Eager writeback needs free *slots* to put work into and would
be competing for the same ones; it would make this regime worse, not
better. What is left is a pure **depth vs. DRAM-latency** relationship,
and depth is a parameter:

| `VICTIM_SLOTS` | DDR-40 cyc/op | miss-blocked | DDR-200 cyc/op | miss-blocked |
|---|---|---|---|---|
| 2 | 17.18 | 82.5% | 76.87 | 96.1% |
| 4 | 8.94 | 66.4% | 38.63 | 92.2% |
| **8 (shipped)** | **4.84** | 37.9% | **19.53** | 84.6% |
| 16 | 3.00 | **0.0%** | 10.01 | 70.0% |

(Reproduce with `verilator ... -GVICTIM_SLOTS=n` on `tb_l2c`; the tb's
occupancy taps are 5 bits so depths to 31 measure rather than wrap.)
cycles/op tracks `0.746 x L / SLOTS + ~3`. At DDR-40, 16 slots reaches
3.00 cyc/op — identical to a pure *hit* stream, i.e. eviction has stopped
costing anything at all, and *below* the 3.75 floor `54919fe` measured
(that experiment still had the serialized 2-slot engine underneath it).

**8 is shipped, not 16**, on three grounds and one of them is unverified:
the LUTRAM payload is the same either way (both depths are one LUT6 per
bit), but 16 doubles the metadata flops and, more importantly, doubles
the line-address CAM that feeds `l2c_ctrl`'s `s_lookup_active` — and
hit-resolve's level-2 LUTs are full after `980342c`. The real chain's
round trip is also ~14 core cycles rather than the model's 40-200
(`axi_ddr4_mig_bridge` early-acks B at WLAST-to-MIG and caps writes at
`WMAX_OUTSTANDING = 4`), and §13.3 shows 8 slots already takes
miss-blocked to 0.0% there with a peak occupancy of 2. Going to 16 should
be decided by a timing report on the query cone, not by this model.

### 13.5 Area — structural estimate, NOT synthesised

No Vivado was run. `878d002`'s post-synth attribution had `u_victim` at
580 LUT / 1,101 FF. Structurally:

- **Removed:** 2 x 512 payload flops (1,024 FF) and the 8:1-over-128b read
  mux that selecting `{slot, quadrant}` out of them elaborated.
- **Added:** `v_pay` — 8 deep x 512 bits, one write port at `wptr`, one
  asynchronous read port at `dptr` — which is 512 LUT6 of LUTRAM
  (`RAM32X1D`-class, one per bit at any depth up to 32), plus a 4:1
  quadrant select (~128 LUT6), plus 8 x 37 = 296 metadata flops and ~50
  LUT of query CAM.

So roughly **-700 FF and +250-400 LUT, most of it LUTRAM**, for 4x the
outstanding capacity. **Verify inference rather than assuming it**:
Vivado prints "Trying to implement RAM 'X' in registers" when it fails,
and it said exactly that for `m_dsec_reg` while `878d002`'s `r_pay_reg`
inferred cleanly. If the LUT delta turns out to matter, the available
4x saving is to address the array `{slot, quadrant}` (32 x 128 b = 128
LUT6) and accept the push over four cycles — `l2c_ctrl` already holds
`victim_push_*` stable until `push_ready`, so no staging register is
needed. That was deliberately not taken here: it trades a measured,
low-risk change for a new multi-cycle accept path worth ~0.2% of the
device's LUTs.

### 13.6 Verification

`tb-l2c` 50 -> 53 scenarios, 0 FAIL, and 53/53 across `L2C_SEED` 1-8.
Three new scenarios, each RED-verified against a mutant with everything
else still green:

- `deep_victim_query_covers_every_slot` — the coherency one. l2c does not
  snoop, so `victim_query_hit` is the only thing between a re-read of a
  just-evicted line and stale DRAM. One set is filled eight ways dirty
  and then evicted eight deep with AWREADY blocked, so nothing drains and
  DRAM's copy stays stale; a read of a buffered line must stall and then
  return the victim's bytes. RED mutant: narrow the query generate loop
  to 2 slots -> the probe read completes while AWREADY is still blocked
  and returns the wrong bytes.
  *The first version of this test evicted with WRITES and passed the
  mutant*, because that leaves the set eight-deep in dirty lines and the
  probe then stalls on `victim_push_ready` instead of on the query —
  indistinguishable from outside. It evicts with READS (clean installs)
  for exactly that reason; the comment in the test says so.
- `victim_writeback_bresp_error_surfaces` — a writeback has no requester
  to notify (Minor-16), so a non-OKAY BRESP is counted and printed rather
  than silently dropped, and the entry still retires instead of wedging
  the buffer. RED mutant: stop counting -> fails alone.
- `deep_victim_partial_writebacks_stay_partial` — eight lines, each dirty
  in exactly one quadrant and each a different quadrant, queued at once
  and then released; every AW must be one beat at its own entry's
  quadrant. RED mutant: take the AW shape from the head slot's mask
  instead of this entry's -> fails, and so does the randomized
  scoreboard.

The self-checking taps from `54919fe` stay armed and gained a second
invariant: **outstanding <= occupancy + post-reset strays**, i.e. a
writeback cannot be outstanding without still owning its slot. RED
mutant: retire at WLAST instead of at B -> `eviction_tap_self_check`
fails alone. That is the durability rule the query depends on, so it is
checked every cycle of every test rather than argued for in a comment.

`tb_vram_ddr_chain.v`'s `dbg_l2_victim_state` tap became the semantic
`dbg_l2_victim_wburst`: the C++ compared against a hard-coded `3'd2`, and
the state encoding moved when `S_B` was deleted, which would have turned
`reset_mid_l2_victim_writeback` quietly false instead of failing loudly.

Gates: `make lint` 8/8 configs — `tb-l2c` 53 (and 53 across `L2C_SEED`
1-8) — `tb-l2c-stress` 3 x 50,000 ops — `tb-l2c-chain` 4 —
`tb-l2c-chain-off` 2 — `tb-l2c-bypass-all` 4 — `tb-l2c-bypass-window` 0
failed — `tb-l2c-wstream` / `-off` — `tb-l2c-sctr` / `-off` —
`tb-axi-xbar` 408 — `tb-ddr-model` 13 — `tb-axi-ddr-contract` 9 —
`tb-axi-async-bridge` 14 — `tb-axi-ddr4-mig-bridge` 46 —
`tb-vram-ddr-chain` 17 — `tb-sd-boot` 11 — `tb-sd-boot-zero` 5 —
`tb-n2w-watchdog-on` 104 / `-off` 28. All 0 FAIL.

## 14. Pipelined bypass engine — measured

`7d49e1f` measured the L2C front door, found it costs nothing, and named
the next target on its way past: `l2c_bypass.v` was a single
request/response engine (`S_IDLE -> S_AW -> S_W -> S_B`,
`S_IDLE -> S_AR -> S_R`) and `l2c_ctrl` decomposes a bypass burst into one
request **per 16 B beat**, so a 512 B transfer paid **32 serialized DRAM
round trips**. This section is the measurement that followed, the depth
argument, and what shipped.

Every number below comes from `L2C_PERF=1 build/l2c/Vtb_l2c` against the
deterministic model (fixed first-beat latency, zero beat gap, always
ready) at 100 MHz. The "before" column was produced by rebuilding
`7d49e1f`'s RTL and running it, not quoted from an earlier doc.

### 14.1 Before / after

| row | DDR-40 before | DDR-40 after | DDR-200 before | DDR-200 after |
|---|---|---|---|---|
| `bypass_read_512B`  | 1376.25 cyc / 37.2 MB/s | **179.25 / 285.6** | 6496.25 / 7.9 | **819.25 / 62.5** |
| `bypass_write_512B` | 1377.25 / 37.2 | **190.25 / 269.1** | 6497.25 / 7.9 | **830.25 / 61.7** |
| `bypass_read_16B`   | 44.02 / 36.4 | 43.03 / 37.2 | 204.02 / 7.8 | 203.03 / 7.9 |
| `mixed_cpu_hits_plus_bypass` | 23.21 cyc/hit-equiv | **5.23** | 101.51 | **12.80** |

**All 21 non-bypass rows are bit-identical**, DDR-40 and DDR-200 alike
(`hit_read` 3.00, `evict_write` 4.84/19.53, `stream_write_64B` 7.03,
`two_cacheable_masters` 3.01 cyc/beat, the whole table).

`bypass_read_16B` barely moves and that is correct: it is 64 separate
single-beat bursts, each with its own AXI id, and the engine takes one id
at a time (§14.4). There is nothing to pipeline — the row is pure latency
and always was.

Front-door attribution on the mixed row, which is the more interesting
half:

| | DDR-40 before | after | DDR-200 before | after |
|---|---|---|---|---|
| idle cycles | 5430 | 810 | 25474 | 2766 |
| of which `idbusy` | **5040** | **0** | **24240** | **0** |
| of which `bypnr` | 0 | 420 | 633 | 2367 |
| longest unbroken AR refusal | 1293 cyc | 138 | 6304 | 627 |

The stall did not disappear, it **changed category**: from a false
ordering hazard (`idbusy`) to real queue backpressure (`bypnr`). That is
what "the door is doing its job" looks like — and it is what the new
`bypass_burst_leaves_front_door_usable` scenario asserts on, rather than
on a cycle count alone.

### 14.2 Two axes, measured separately — and they turn out not to be additive

The brief for this work asked whether the win is bandwidth, or the
`idbusy` blocking of unrelated CPU traffic, since those are separable and
might want different fixes. They were measured as a 2x2: engine depth
against the `l2c_ctrl` `id_busy_c` qualifier, one build each.

DDR-40, `bypass_read_512B` cycles / `mixed` cycles-per-hit-equivalent:

| | `id_busy_c` applied bypass→bypass (as shipped in `7d49e1f`) | qualified with `!is_bypass_c` |
|---|---|---|
| **depth 1** | 1376.25 / 23.21 *(= HEAD)* | 1376.25 / 23.21 |
| **depth 8** | 719.25 / 13.60 | **179.25 / 5.23** |

Read that table before believing either change on its own.

- **The `id_busy_c` qualifier alone buys exactly nothing** — not "a
  little", nothing: the depth-1 row is identical to the digit either way.
  With a one-slot engine there is no second beat to accept, so removing
  the reason to refuse it changes no cycle.
- **Depth alone buys 1.91x, not 7.68x.** Not zero, and the reason is a
  one-cycle timing detail rather than anything intended: `byp_active_valid`
  only rises the cycle *after* a push, so the front door slips exactly two
  beats into the queue before the hold-off latches. Depth 8 therefore
  behaves like depth 2. That is the whole of it.
- **Together they are 7.68x.** The qualifier is an *enabler* with no
  standalone value, and the depth is capped at ~2x without it. Shipping
  either half alone would have been a wasted landing.

And the second question — whether CPU hits could recover with bypass
bandwidth unchanged — has a structural answer, not just an empirical one:
**no**. `l2c_ctrl` has one `ar_have` tracker, so a bypass burst owns the
read side of the front door from its first beat until its last beat is
*accepted*. Nothing about the id hold-off changes that; only finishing the
burst sooner does. The mixed row's 23.21 -> 5.23 is the bandwidth win
showing up as a latency win for unrelated traffic. `7d49e1f` measured a
second front-door tracker and rejected it on cost; that conclusion stands,
and this is the reason it did not need revisiting.

### 14.3 Depth: where the curve saturates, and where the *system* does

Full sweep, both changes in, one Verilated build per depth
(`make tb-l2c-bypdepth`, `L2C_BYP_DEPTHS ?= 1 2 4 8 16 32`). Depth 1 is
the in-harness reference point: it reproduces `7d49e1f`'s **read** numbers
to the digit (1376.25 / 6496.25), which is what makes the rest of the
column trustworthy. Its **write** numbers run one cycle per beat longer
than v1 (1408.25 vs 1377.25) because v2 separates response *capture* from
response *delivery* — v1 fused them in `S_B -> S_RSP`. That extra stage is
what lets responses pipeline at all, and it is invisible at every depth
>= 2, where it overlaps.

| `BYPASS_SLOTS` | read 512B @40 | @200 | write 512B @40 | @200 | mixed cyc/hit @40 | @200 |
|---|---|---|---|---|---|---|
| 1  | 1376.25 | 6496.25 | 1408.25 | 6528.25 | 23.21 | 101.51 |
| 2  | 689.25  | 3249.25 | 706.25  | 3266.25 | 13.13 | 50.77  |
| 4  | 347.25  | 1627.25 | 358.25  | 1638.25 | 7.80  | 25.43  |
| **8**  | **179.25** | **819.25** | **190.25** | **830.25** | **5.23** | **12.80** |
| 16 | 101.25  | 421.25  | 118.25  | 438.25  | 4.15  | 6.66   |
| 32 | 74.25   | 234.25  | 106.25  | 266.25  | 3.96  | 3.98   |

**The isolated model does not saturate before depth 32.** It cannot: the
transfer is 32 beats, the floor is `latency + beats` (one whole burst in
flight at once), and 74.25 at DDR-40 is that floor. So "pick where the
curve flattens" gives no answer here and the depth has to be argued from
somewhere else. Two places, in order of how binding they are.

**1. The DDR path caps it at 8 (this is the binding one).**
`axi_ddr4_mig_bridge` accepts `RMAX_OUTSTANDING = 8` reads and
`WMAX_OUTSTANDING = 4` writes
(`docs/ddr4_mig_bridge_contract.md`, "Multi-outstanding / cut-through
contract"). `axi_bridge_stale_sink` carries a 4-bit outstanding counter
on top of that. Past 8 outstanding reads the bridge simply backpressures
and the extra slots move the queue from one side of the boundary to the
other; the table above only keeps improving because `tb_l2c.cpp`'s memory
model is idealised (always-ready, unbounded outstanding). This is the same
shape of argument `0994eaf8` used to ship `axi_narrow_to_wide` at depth 4:
take the depth where the *system* saturates, not where the isolated model
does.

**2. It is also exactly one 512 B block.** 8 slots x 16 B = 128 B, and a
512 B RAM-disk transfer is 4 round trips instead of 32 — the shape the
window exists for (`AXI_RAMDISK_L2_BYP_BASE`, `rtl/soc/vhdd_ddr.v`).

Raising `BYPASS_SLOTS` past 8 is a **paired** change with the bridge's
own caps, and the parameter is there so that pairing is a two-line edit
rather than a rewrite.

### 14.4 What the sweep found that the numbers did not

At `BYPASS_SLOTS=32` and DDR-200 the design **hung**. `l2c.v`'s `aw_out`
(AW-accepted minus B-accepted for the current write-port owner) was a
hardcoded `reg [4:0]`, sized when the deepest write source was
`l2c_victim`'s 8 slots. 32 outstanding bypass writes wrapped `5'd32` to
zero, `owner_locked_c` dropped mid-flight, and the arbiter deadlocked. At
DDR-40 the queue never filled far enough to reach it, so only the high-
latency half of the sweep exposed it.

It is now sized from the parameters —
`AW_OUT_W = ceil(log2(max(VICTIM_SLOTS, BYPASS_SLOTS))) + 1`, which is
**4 bits at the shipped 8/8, one bit narrower than the constant it
replaces**.

The same sweep also caught two testbench-side bugs that the shipped depth
would never have shown: `dbg_by_occ`/`dbg_by_inflight` were 5-bit taps, so
an occupancy of 32 read back as 0 and tripped the `inflight <= occupancy`
self-check on a perfectly healthy engine; and the concurrency assertion
asked for `inflight >= SLOTS`, which is unreachable once the depth reaches
the burst length — the last beat's dispatch races the first response, so
the peak is `beats - 1`. Both are now expressed in terms of the depth.

Filed here in full because it is the argument for running a depth sweep
even when you already know which depth you intend to ship: three of the
four things it found were only reachable off the shipping configuration,
and one of them was a real deadlock.

### 14.5 Structural cost, by construction

Estimated the way `386ea29`'s area section was, from the shape of the RTL,
with no Vivado run. The reference point is real: `l2c_bypass` v1 measured
**213 LUT / 320 FF post-route** (recorded in `rtl/soc/fpga_top_ddr.vh`).
v1's 320 FF is essentially its five payload registers — `op_wdata` 128 +
`op_rdata` 128 + `op_addr` 32 + `op_wstrb` 16 + id/flags/state.

v2 at `SLOTS=8`:

| | |
|---|---|
| `s_req` LUTRAM ({addr, wdata, wstrb} = 176 b) | 176 LUT |
| `s_rdat` LUTRAM (128 b) | 128 LUT |
| pointer arithmetic, slot muxes, `req_id` compare, window compare | ~70 LUT |
| per-slot control 5 b x 8, four `CW`-bit pointers, `q_id`, `dst`, `bout_cnt`, `bsink_cnt` | ~72 FF |

**≈ +160 LUT, −250 FF against v1** — +0.07% of the KU5P's 216,960 LUTs
and −0.06% of its 432,960 FFs. The payload moves from flops into
distributed RAM, which is the same trade `878d002` and `386ea29` both
took, and it is why the FF count falls while the LUT count rises.

Two things worth stating explicitly, given the −0.154 ns true WNS
(`578c2d7` §4.4):

- **Depth is nearly free from 1 to 32.** Both payload arrays are one
  LUT6 per bit at any depth up to 32 (a `RAM32X1S` is 32 deep natively) —
  width is what costs, exactly as `l2c_victim.v`'s header argues. Going
  8 -> 32 adds ~130 FF of per-slot control and ~40 LUT of wider slot
  muxes. **Depth 8 is not an area decision.** It is §14.3's downstream
  cap.
- **Nothing lands in `l2c_ctrl`'s cones.** This is the structural
  difference from `386ea29`, and it is why the depth argument comes out
  differently here. `l2c_victim`'s depth grows a per-slot line-address CAM
  that feeds `s_lookup_active` in the hit-resolve cone, which is why
  `VICTIM_SLOTS` stopped at 8. `l2c_bypass` is queried by **one** id
  comparator (`byp_active_id == cur_id`) regardless of depth — the
  one-id-at-a-time rule is what buys that. The only `l2c_ctrl` edit is one
  extra AND term (`&& !is_bypass_c`) on a signal whose inputs are already
  in that same accept-stage cone; no new comparator, no new level.

`l2c.v`'s deltas are a wash: the read route queue goes 8 -> 16 entries
(1 bit each, still one LUT6 of `RAM32X1S`, +1 pointer bit) so that it
covers the MSHR's 8 fills *and* bypass's 8 beats instead of capping their
sum, and `aw_out` goes 5 -> 4 bits.

Note that with `ENABLE_DDR_RAMDISK` off — today's default — `BYP_WIN_EN`
is 0, `match_hit` is a constant 0 and the whole engine prunes away. The
area above is only paid in the configuration this work exists to unblock.

### 14.6 What this unblocks

`rtl/soc/fpga_top_ddr.vh` disables the DDR RAM disk, and the reason it
records is this engine:

> "It also had no business carrying that traffic: the engine is
> single-request, single-outstanding, single-beat as a HARD v1
> precondition (docs/l2c_spec.md:432) ... Streaming block storage is
> exactly the profile that spec warns needs its own multi-outstanding
> engine."

That reason no longer holds. Re-enabling `ENABLE_DDR_RAMDISK` remains a
separate decision (the aperture decode has to move in lockstep, per that
same comment), but the engine is no longer the thing standing in the way.

### 14.7 Verification

`tb-l2c` 54 -> 61: five new directed scenarios plus two whole-run
invariants that are checked on **every cycle of every scenario**,
including the randomized scoreboard and the perf suite.

The whole-run invariants:

- `bypass_rw_exclusion_check` — a bypass read and a bypass write are never
  outstanding on the master port at the same time. Measured from the AXI
  **wires** (bypass traffic carries ID bit `[ID_WIDTH-1] = 1`, per
  `l2c_defs.vh`; the memory model echoes IDs on B/R), deliberately not
  from a DUT tap — a tap and the logic it watches can be wrong together.
  Post-reset strays are split into a `stale` bucket the same way
  `l2c_bypass` splits `bout_cnt` from `bsink_cnt`, because a stray from a
  dead epoch legitimately coexists with fresh traffic of the other
  direction. **A data comparison cannot substitute for this**:
  `tb_l2c.cpp`'s memory model applies writes at W-beat time and samples
  read data at R-drive time, so an out-of-order read still returns
  post-write bytes in this harness. The bug would be invisible here and
  real on silicon.
- `bypass_tap_self_check` — `inflight <= occupancy <= depth`. A slot must
  still be owned while its transaction is presented. Retiring at response
  *capture* instead of at response *delivery* would satisfy every cycle
  count in the file and silently break `rsp_id`.

Every scenario RED-verified against a named mutant, and the read/write
split was verified in **both directions** because that is the trap this
project has now walked into twice (`386ea29`'s deep-query test and
`7d49e1f`'s `s_awready` case both passed their own mutant because a read
path covered for a write path):

| mutant | fails |
|---|---|
| `hdr_go_c &&= (d_wr_c OR out_c == 0)` — reads serialize, writes still pipeline | `bypass_read_pipelines_to_depth`, `bypass_burst_leaves_front_door_usable` — **and not** `bypass_write_pipelines_to_depth` |
| `hdr_go_c &&= (!d_wr_c OR out_c == 0)` — writes serialize, reads still pipeline | `bypass_write_pipelines_to_depth`, `reset_mid_bypass_write_pipeline` — **and not** `bypass_read_pipelines_to_depth` |
| `req_ready = empty_c` — depth 1 both ways | all four throughput scenarios |
| `dir_ok_c = 1'b1` — no drain on direction change | `bypass_read_write_never_overlap`, `bypass_rw_exclusion_check` |
| `id_busy_c` un-qualified (the pre-fix form) | `bypass_read_pipelines_to_depth`, `bypass_write_pipelines_to_depth`, `bypass_burst_leaves_front_door_usable` |
| reset `D_HDR` branch loads `bsink_cnt <= 0` | `reset_mid_bypass_write_pipeline`, **round 2 only** |
| `l2c.v` releases a BYPASS-owned write grant on reset (the pre-fix form) | `reset_mid_bypass_write_pipeline`, **round 1 only** |

The last three wedge the design outright rather than reporting a tidy
`[FAIL]` line — the write port never recovers, so the reset scenario's
nine post-reset eviction readbacks each time out and the run then aborts
in `alloc_id()` with every id stuck on an unfinished write. Every scenario
ahead of the reset one still passes in those logs, which is what makes the
attribution unambiguous.
| `rsp_valid = done_c` — answer every write beat | `unmatched_response_protocol_check` |
| `used_c = wptr - fptr` — retire a slot at response CAPTURE, not delivery | `bypass_read_pipelines_to_depth`, `bypass_read_during_full_line_gather`, and the randomized scoreboard (wrong data + phantom R/B) |

**`reset_mid_bypass_write_pipeline` failed its own mutant on the first
attempt**, exactly as the brief predicted. The reset always landed while
the sequencer was still streaming (`D_W`), and the `D_W` reset branch —
which the mutant left alone — loads `bsink_cnt` too. The `D_HDR` branch,
the one the mutant broke, was never reached. It now runs **two** rounds
and the second one waits for `inflight == occupancy` before resetting,
which is precisely "every accepted slot is dispatched and the sequencer is
parked awaiting BRESPs", i.e. `D_HDR`. The reason is written into the
test.

The two rounds turned out to catch **different** mutants — the last two
rows above — and neither round catches both, which is the retrospective
justification for keeping them separate rather than folding them into one
scenario. It is visible in the failure addresses: the arbiter mutant fails
round 1's eviction sweep (`0x2E0000 + way*stride`) and the sink mutant
fails round 2's (`0x2E0040 + way*stride`).

One harness fix fell out of the mutation work and is worth keeping:
`test_randomized_scoreboard`'s main loop had **no unconditional
no-progress guard** (the `STALL` report is behind `L2C_DEBUG_PROGRESS`),
so any DUT wedge hung the binary forever rather than failing. Two lethal
mutants each hung it for over twelve minutes before being killed by hand,
which makes a wedge indistinguishable from a slow run and makes mutation
testing unusable. It now fails after 500,000 cycles without a completion.

Cycle budgets in the new scenarios are expressed as
`ceil(beats / dbg_by_slots) x 300` rather than as constants, so the same
directed suite is meaningful at every depth in the sweep — all six builds
run 61/61.

Gates, 0 FAIL: `make lint` 8/8 configs — `tb-l2c` 61 across `L2C_SEED`
1-8 — `tb-l2c-bypdepth` {1,2,4,8,16,32} 61 each — `tb-l2c-stress`
3 x 50,000 — `tb-l2c-chain` / `-off` — `tb-l2c-bypass-all` —
`tb-l2c-bypass-window` — `tb-l2c-wstream` / `-off` — `tb-l2c-sctr` /
`-off` (64 B/line still 0.000 fill beats) — `tb-axi-xbar` 408 —
`tb-ddr-model` — `tb-axi-ddr-contract` — `tb-axi-async-bridge` —
`tb-axi-ddr4-mig-bridge` — `tb-vram-ddr-chain` — `tb-sd-boot` —
`tb-sd-boot-zero` — `tb-n2w-watchdog-on` / `-off`.
