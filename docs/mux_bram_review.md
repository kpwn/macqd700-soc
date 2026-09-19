# Fabric-RAM review — where the SoC builds memory out of LUTs

**Date:** 2026-08-19
**Basis:** post-route `build/vivado/reports/utilization_route.rpt` (physopt postRoute,
2026-08-19 13:16) and the synthesis log `/tmp/build_bitstream_20260819_135606.log`.
No Vivado was run for this review; every number below is either read out of those
artifacts or is an **estimate** labelled as such.

---

## 0. Why this matters — the resource picture

From `build/vivado/reports/utilization_synth.rpt:35-97`:

| Resource | Used | Available | % |
|---|---:|---:|---:|
| **CLB LUTs** | **187,025** | 216,960 | **86.20** |
| — LUT as Logic | 181,600 | 216,960 | 83.70 |
| — **LUT as Memory (LUTRAM/SRL)** | **5,425** | **99,840** | **5.43** |
| CLB Registers | 127,531 | 433,920 | 29.39 |
| **Block RAM Tile** | **115** | **480** | **23.96** |
| **URAM** | **64** | **64** | **100.00** |
| DSP | 27 | 1,824 | 1.48 |

Read that table twice. We are at 86.2 % LUT with congestion level 6 and WNS +0.024 ns,
**and 94,415 distributed-RAM cells and 365 BRAM tiles are sitting unused.** Every bit of
storage we hold in flip-flops plus a fabric mux is being paid for in the one currency
that is blocking the build, out of a wallet that is 5 % full in the currency that is not.

URAM is full (all 64 are `l2c_data`'s eight 4 K × 512 ways). **Nothing in this document
proposes URAM.**

---

## 1. Method

Vivado prints every array it *successfully* infers as memory, in three tables near the
end of synthesis (`Distributed RAM: Implemented`, `Block RAM: Implemented`,
`ROM: Implemented`) plus per-array `[Synth 8-xxxx]` messages. So:

```
enumerate register arrays declared in RTL  −  arrays the log says were inferred
                                           =  arrays built out of fabric
```

The RTL side was swept with a parser that resolves `parameter`/`localparam`/`` `define ``
expressions and handles **1-bit arrays and 2-D unpacked arrays**, which a naive
`reg [W:0] name [0:D]` grep misses entirely (that omission hid `lb_word`, `ch_buf`,
`r_wdata`, `m_v`, `p_need`, …). 453 array declarations across `rtl/` + `cpu/rtl/`.

The log side (`grep -oE 'The RAM "[^"]+" of size'` plus the three inference tables at
log lines 3499-3700) gives 100 inferred arrays.

### 1.1 The five reasons inference fails

Every fabric array in this design failed for one or more of these. The reason determines
whether the fix is a coding change or a redesign — so it is reported per candidate.

| # | Cause | Fix class |
|---|---|---|
| **A** | **More writes per cycle than the primitive has ports** — including an unrolled `for` loop that writes N words of one array in one cycle, and a function that pokes 6 named entries at once. Hard blocker, ≤ 2 ports on any RAM. | usually mechanical (bank the array, or hoist the multi-write entries out) |
| **B** | **2-D unpacked declaration** `reg [W-1:0] x [0:A][0:B]`. Vivado's RAM extractor does not recognise this shape *at all*, even with a single `{i,j}` index pair on both sides. | mechanical (flatten to 1-D with an explicit concatenated address) |
| **C** | **Associative / parallel read** — every entry compared or read in one cycle. This is a CAM, not a RAM. | not fixable, and usually not worth fixing (see §4) |
| **D** | **Reset `for`-loop clearing the array.** Blocks BRAM *and* LUTRAM outright (neither has a per-bit reset). Note: the loop itself costs ~0 LUT — it maps onto the FFs' own R/S pins — the damage is that it forces the *read* path into a fabric mux. | mechanical if the array is written-before-read; otherwise needs a sweep FSM |
| **E** | **Asynchronous read into a shared destination.** A continuous `assign dout = mem[a];`, or a registered read whose destination register is *also* written by non-RAM sources (a mux before the flop). Blocks BRAM specifically; LUTRAM still possible. | mechanical if the consumer tolerates a registered read, structural if not |

**BRAM read latency is the catch.** A register-array read is combinational; a BRAM read is
registered. Converting is only free when the read address is already available a cycle
early (captured into a pending register at `ar_fire`/`aw_fire`, or held stable across a
CDC handshake). Where that is not true, the conversion costs a pipeline stage — and
that cost is stated explicitly per candidate below. Distributed RAM keeps the
asynchronous read and is therefore latency-free, which is why several
recommendations below target LUTRAM rather than BRAM.

---

## 2. Inventory — arrays Vivado DID infer (no action)

100 arrays inferred, ~5,425 LUT-as-memory + 115 BRAM tiles + 64 URAM. Grouped:

| Where | Arrays | Implemented as |
|---|---|---|
| `l2c_data` `g_way[0..7].mem` | 8 × 4 K × 512 | **64 URAM288** (full) |
| `l2c_tags` `g_way[0..7].mem`, `plru_mem` | 8 × 4 K × 16, 4 K × 7 | 17 RAMB36 |
| `video_top/u_scanout/u_disp` `line_mem_p0..2`, `clut_mem` | 3 × 72 K × 8, 256 × 24 | 54 RAMB36 + 1 RAMB18 |
| `sd_ctrl` `blk_buf` (×3 instances), `fb_reader/u_req_fifo` | 512 × 8, 256 × 22 | 4 RAMB18 |
| `debug_ctrl` `pc_trace` | 256 × 32 | 1 RAMB18 (log: *"too shallow to use URAM. Choosing BRAM"*) |
| `pram_sd` `sec_buf`, `scsi` `sec_buf` | 512 × 8 | 2 RAMB18 / 32 RAM64M8 |
| `axi_ddr4_mig_bridge` `rq_*`/`wq_*`/`rdin_*`/`wrout_*` (24 arrays) | 2-8 deep | RAM32M16 / RAM16X1D |
| `async_fifo` `mem` in every CDC bridge (`u_ddr`, `u_pb_s1_cdc`, `u_dbg_pb_to_core`, `u_vhdd_ddr_cdc`) | 4-16 deep | RAM32M16, `ram_style` attribute |
| `boot_fsm` `word_fifo_{addr,data,strb}` | 32 × 32/32/4 | RAM32M16 |
| `video` `ramdac_clut_{r,g,b}` | 3 × 256 × 8 | **RAM256X1S × 24** — confirmed inferred |
| `scanout_ddr_reader` `q_line`, `q_off` | 64 × 15, 64 × 7 | RAM64M8 |
| `vhdd_ddr` `blkbuf` | 32 × 128 | RAM32M16 × 20 |
| `l2c_mshr` `p_wr/p_last/p_need/p_qoff/p_wdata/p_wstrb` | 8 deep | RAM16X1D / RAM32M16 |
| `commit` `mb_*` (9 arrays) | 64 deep | RAM64M8 |
| `iq_fp` `e_*` (5 arrays) | 8 deep | RAM32M16 / RAM16X1D |
| `icache` `tag_ram0..3`, `data_ram0..3` | 64 × 22, 64 × 128 | RAM64M8 — but see §3.6 |
| `asc` `fifo_a`, `fifo_b` | 1 K × 8 | RAM256X1D × 64 + RAM64M8 × 192 — but see §3.5 |
| `pic16c5x` `prog` | 512 × 12 | **LUT ROM** — correct, see §4 |

Two entries in this table are on the candidate list anyway (`asc`, `icache`): the log
records `[Synth 8-6849] Infeasible attribute ram_style = "block" … trying to implement
using LUTRAM` for them. The RTL *asked* for BRAM and Vivado **refused**. That is a
coding-pattern defect, not a success.

---

## 3. Inventory — arrays built out of FABRIC, ranked

Rank is by (estimated LUT recovered ÷ risk). "Bits" is total stored bits.

| # | Array | File:line | Dims | Bits | Why fabric (§1.1) | Est. LUT today | Fix | Latency cost | Risk |
|---|---|---|---|---:|---|---:|---|---|---|
| **1** | `lb_word` | `rtl/soc/scanout_line_fetch.v:107` | 4 × 32 × 32 b | 4,096 | **A** (4 writes/cy) + **B** (2-D) | **~6,058** measured¹ | bank on the beat → 32 × 128 LUTRAM | **none** | low — **LANDED** |
| **2** | `regs` | `rtl/mac/video.v:275` | 256 × 32 | 8,192 | **E** (2 dynamic async reads + 6 constant taps) | **~5,400** est. | 1W/2R RAM, both read addrs available 1 cy early | **none** | med |
| **3** | `m_line` + `r_wdata`/`r_wstrb`/… | `rtl/soc/l2c_mshr.v:115,121` | 8 × 512, 8×4×128 | 9,408 | **A** (3 write sources) + **B** (2-D) | ~12,300 + ~1,800 | see §3.3 — **another agent owns this file** | none (LUTRAM) | n/a |
| **4** | `pram` | `rtl/mac/rtc.v:238` | 256 × 8 | 2,048 | **A** (4 write sources) + **D** + **E** (async `assign`) | **~1,450** est. | 2-port sync RAM + clear-sweep FSM | none² | med |
| **5** | `fifo_a`/`fifo_b` | `rtl/mac/asc.v:332` | 2 × 1 K × 8 | 16,384 | **A/E** (2 wavetable reads same cy + CPU readback aliased into `pb_rdata`) | **1,152 LUTRAM + ~400 logic** | serialize wave reads, drop CPU readback | none³ | med |
| **6** | `data_ram0..3` | `cpu/rtl/core/fetch/icache.v:211` | 4 × 64 × 128 | 32,768 | **E** (`data_ram_q[victim_way]` written from 3 non-RAM sources) | ~600 LUTRAM + 200-500 logic | scalar `data_ram_q0..3` like `dcache.v:342` | none | low-med |
| **7** | `sonic_reg` | `rtl/mac/q700_eth_sonic.v:82` | 64 × 16 | 1,024 | **A** (6-way simultaneous write) + **D** | **~310** est. | hoist 7 side-effect regs, rest → LUTRAM | none (must stay async⁴) | med |
| **8** | `u_rsp_fifo/mem` | `rtl/board/video_phy/fb_reader.v:413` | 256 × 32 | 8,192 | **E** (async `assign rd_data = mem[…]`) | 160 LUTRAM + ~80 | registered read port | **none**⁵ | low |
| **9** | `param_ram` | `rtl/mac/iwm_stub.v:80` | 16 × 8 | 128 | **D** only | ~40 | drop the reset loop | none | low |
| **10** | `wra`/`wrb` | `rtl/mac/scc.v:154` | 2 × 16 × 8 | 256 | **C** + **A** | ~80 | — leave it, genuine register file | — | — |
| — | `ch_buf`, `ch_desc_buf` | `rtl/soc/dma_ctrl.v:153,157` | 4×16×64, 4×4×64 | 5,120 | **B** | **0 — module de-instantiated** | fix before re-instantiating | — | — |
| — | `ring` | `rtl/soc/scsi_trace_ring.v:179` | — | — | — | **0 — module de-instantiated** | — | — | — |
| — | `prog` | `rtl/mac/pic16c5x.v:42` | 512 × 12 | 6,144 | ROM, correctly LUT | ~150-250 | do not touch — see §4 | — | — |

¹ measured, not estimated: `u_scanout_reader/u_fetch` = 6,058 LUT / 4,298 FF / 0 LUTRAM / 0 BRAM.
² the expensive port converts free; the second port has ~30 pb_clk of slack. See §3.4.
³ 22 kHz consumer against a 100 MHz clock.
⁴ `peripheral_bus.v:1283-1292` whitelists `SLOT_SONIC` as a same-cycle read, so distributed RAM only.
⁵ the sibling `u_req_fifo` instance of the same module already maps to a RAMB18.

### Where the LUTs actually are (post-route, SoC side)

| Instance | Module | LUT | LUTRAM | FF | BRAM | Verdict |
|---|---|---:|---:|---:|---:|---|
| `u_cpu` | `m68k_axi_wrapper` | 107,855 | 1,112 | 67,139 | 25×18 | see §5 |
| `u_l2c` | `l2c` | 24,222 | 92 | 12,878 | 17×36 + 64 URAM | §3.3, owned elsewhere |
| `u_mig_ddr4` | vendor MIG | **10,282** | 452 | 12,364 | 26 | the calibration point |
| `u_scanout_reader` | `scanout_ddr_reader` | 6,100 | 26 | 4,362 | 0 | **#1 — fixed** |
| `u_dafb` | `video` | 6,053 | 96 | 8,688 | 0 | **#2** |
| `u_xbar` | `axi_xbar` | 4,779 | 0 | 994 | 0 | genuine crossbar, §4 |
| `u_pram_sd` | `pram_sd` | 3,252 | 0 | 659 | 3 | **false lead**, §4 |
| `u_scsi` | `scsi` | 3,013 | 160 | 701 | 0 | uncommitted, not reviewed |
| `u_video` | `video_top` | 2,924 | 160 | 2,120 | 56 | #8 lives here |
| `u_dbg_vio` | Xilinx VIO IP | 2,683 | 0 | 4,834 | 0 | §4 — one-line TCL win |
| `u_ddr` | `ddr_ctrl` | 2,362 | 726 | 1,252 | 0 | all inferred, fine |
| `u_asc` | `asc` | 1,733 | **1,152** | 514 | 0 | **#5** |
| `u_rtc` | `rtc` | 1,684 | 0 | 2,201 | 0 | **#4** |
| `u_pbus` | `peripheral_bus` | 1,168 | 0 | 291 | 0 | no arrays, §4 |

### 3.1 `scanout_line_fetch.lb_word` — LANDED

`reg [31:0] lb_word [0:NUM_LINE_BUF-1][0:LINE_WORDS-1]`, 4 slots × 32 words × 32 b = 4,096 bits.
Parameters confirmed from the instantiation chain: `fpga_top_video.vh:604` overrides
neither, `scanout_ddr_reader.v:119` defaults `NUM_LINE_BUF=4`, `scanout_ddr_reader.v:174`
hard-codes `LINE_OFF_W=7` → `BEATS=8`, `LINE_WORDS=32`.

**Why it was fabric — two independent hard blockers:**

- **A.** The fill loop wrote *four* words per cycle:
  `for (bi=0;bi<4;bi=bi+1) lb_word[f_slot][{f_beat, bi[1:0]}] <= …`. `bi` is an
  elaboration-time constant, so this is four simultaneous write ports.
- **B.** The 2-D unpacked declaration is not a shape the extractor recognises.

**Cost:** the single read at line 169 —
`lb_word[head_hit_slot_c][req_off[LINE_OFF_W-1:2]]` — is a flat **128:1 × 32-bit
combinational mux** whose select is itself a tag-comparator output. Textbook minimum is
32 + 8 + 2 + 1 = 43 LUT6 per output bit ≈ **1,376 LUT**; the measured 6,058 is ~4×
that because `-directive PerformanceOptimized -keep_equivalent_registers`
(`synth/vivado.tcl:1150`) and `phys_opt_design -directive AggressiveExplore`
(`synth/vivado.tcl:1869`) replicate a mux cone whose 4,096 FF sources are physically
scattered. 1.5 LUT per stored bit.

**Fix (landed):** the four write addresses differ *only* in the low 2 index bits, and each
word's data is a static slice of `m_rdata` — so banking on the beat collapses all four
writes into one:

```verilog
(* ram_style = "distributed" *)
reg [127:0] lb_beat [0:(NUM_LINE_BUF*BEATS)-1];        // 32 × 128 b
// write: one port
lb_beat[{f_slot, f_beat}] <= beat_swizzle_c;
// read: one async port + the pre-existing 4:1 word select
wire [127:0] hit_beat_c = lb_beat[{head_hit_slot_c, req_off[LINE_OFF_W-1:4]}];
wire [31:0]  hit_word_c = hit_beat_c[{req_off[3:2], 5'd0} +: 32];
```

`beat_swizzle_c` reverses bytes within each 32-bit lane, reproducing byte-for-byte what
the old per-word loop stored. Depth 32, 1W + 1 async R = the `RAM32X1D`/`RAM32M` shape.

**Latency: zero.** The read stays combinational, so `hit` still pops the caller's queue
head in the same cycle and the `scanout_ddr_reader` contract
(`scanout_ddr_reader.v:17-21`: *"every rd_en is accepted … produces exactly one later
rd_valid, IN ORDER"*) is untouched. No read-during-write hazard: a slot only becomes a
write target ≥3 cycles after `lb_fetching` gates it out of `head_hit_c` (line 155).

**Expected recovery: ~5,900 LUT and ~4,100 FF, for ~160 LUTRAM.** To be confirmed at the
next synth.

**Validation (2026-08-19).** Against the working tree at the time of the change:

| Testbench | Baseline | With change |
|---|---|---|
| `tb-scanout-frames` | pass | **pass** |
| `tb-scanout-frames-negctl` | pass | **pass** |
| `tb-scanout-ddr-frames` | pass | **pass** |
| `tb-scanout-placement-sync` | pass | **pass** |
| `tb-dafb-scanout` | pass | **pass** |
| `tb-dafb-mode-matrix` | pass | **pass** (1956/0) |
| `tb-dafb-mode-matrix-ddr-aperture` | pass | **pass** |
| `tb-fb-reader-ddr-chain` | pass | **pass** (4096/4096 pixels, 0 mismatches ×3 scenarios) |
| `tb-vram-ddr-chain`, `tb-vram-xbar-e2e`, `tb-vram-scaler-firstlight` | pass | **pass** |
| `tb-video-pattern`, `tb-video-checkerboard`, `tb-video-smoke-ddr`, `tb-video-smoke-ddr-negctl` | pass | **pass** |
| `tb-video-smoke` | **fail** (pre-existing, identical diagnostic on a reverted file) | fail, unchanged |
| `make lint` | 0 warnings | **0 warnings** |

> **Concurrency note.** While this review was in progress another session began a large
> rewrite of the fetch FSM in the *same* file (`f_state`/`f_slot`/`f_beat` renamed to
> `fill_*`/`r_beat`, plus a new reset-drain path and a new `RLAST at beat` assertion that
> does not exist at HEAD). That rewrite has **adopted the `lb_beat` banking above** —
> `lb_beat[{fill_slot_c, r_beat}] <= beat_swizzle_c` at line 511 — and no further edits
> were made to the file from here. `tb-fb-reader-ddr-chain` currently trips that new
> assertion in its reset-skew scenario; the three pixel-exact scenarios still report
> 4096/4096 with 0 mismatches, so the storage reshape is not implicated. Coordinate with
> that session before touching `scanout_line_fetch.v` again.

### 3.2 `video.regs` — #2, biggest remaining, PROPOSAL

`reg [31:0] regs [0:255]` (`rtl/mac/video.v:275`), 8,192 bits. The 2026-05-22 comment
above the declaration says it was converted from a flat vector to an array specifically to
get LUTRAM, and the reset loop was deliberately removed for the same reason. **It still
did not infer** — it is absent from all three inference tables, and `u_dafb` measures
6,053 LUT / **8,688 FF** / 96 LUTRAM (the 96 is the three CLUTs, which *did* infer).
8,192 of those 8,688 FFs are `regs`.

**Why (cause E):** two *dynamic* async read ports plus six constant-index taps:

| Site | Read |
|---|---|
| `video.v:482` | `write_prev = regs[aw_word_idx[7:0]]` — dynamic |
| `video.v:1310` | `read_reg = regs[idx[7:0]]` — dynamic |
| `:924, :933, :1235` | `regs[REG_SWATCH_CTRL][0]` / `[2]` |
| `:982, :993` | `regs[REG_SWATCH_CURSOR_LINE]` |
| `:1238` | `regs[REG_SWATCH_BASE+8'h08][11:0]` |
| `:1282, :1299` | `regs[REG_FIRST_HIT]` |
| `:1301` | `regs[REG_DAFB_TEST]` |
| `:1305` | `regs[REG_RAMDAC_PBCTRL][7:0]` |

Single write: `regs[aw_idx] <= write_word` at `:960`.

**Cost estimate:** a 256:1 mux on 32 bits ≈ 32 × (64+16+4+1) = 2,720 LUT6 per read port;
two ports ≈ **5,440 LUT**, against 5,957 measured logic LUT in `u_dafb`. `regs` is
essentially the entire module.

**Proposed fix — and it is latency-free, which is the whole point:**

1. Mirror the six constant-index taps into dedicated shadow FFs, each updated by
   `merge_wstrb(shadow, w_data, w_strb)` on its own `aw_idx ==` compare. `REG_RAMDAC_PBCTRL`
   already has one (`r_pcbr`). Cost ~160 FF + 6 comparators.
2. Give `regs` two *registered* read ports:
   - write path: address `aw_fire ? s_axi_awaddr[9:2] : aw_idx`. `aw_word_idx` is latched at
     `aw_fire` (`:943`) and `do_write = aw_pending && w_pending && !bvalid` (`:842`) can only
     be true from the *next* cycle, so the registered read output is ready exactly when
     `write_prev` is consumed. **Zero added cycles.**
   - read path: address `ar_fire ? s_axi_araddr[9:2] : ar_idx`. `read_reg()` is evaluated in
     the `ar_pending && !rvalid` cycle, one after `ar_fire` (`:1219`). **Zero added cycles.**
     If `rvalid` stalls, `ar_pending` and the address both hold, so the output holds.
3. Gate behind a module parameter (`REGS_SYNC_READ`, default 1) wrapping only the two
   read-port `generate` branches, so a revert is one parameter.

Result: 1W + 2 registered R, 256 × 32 → **1-2 RAMB18** (or ~128 LUTRAM if left implied).
**Est. recovery ~5,400 LUT + ~8,200 FF.**

**Why this is a proposal and not landed:** `video.v` is boot-critical and the working tree
already has **pre-existing** failures in exactly the register area this touches —
`tb-dafb` is 1807 pass / 10 fail (all `+0x20` IRQ-status readback), and `tb-dafb-via-irq`
and `tb-video` also fail on an unmodified tree. Landing a register-file restructure on top
of a red baseline in the same subsystem is not a "clearly-safe mechanical" change. Fix the
IRQ baseline first, then land this — the design above is complete and the per-site
analysis is done.

### 3.3 `l2c_mshr` — READ-ONLY, owned by another agent

Reported for completeness; **no edit was made to `rtl/soc/l2c*.v`.**

`u_mshr` = 20,953 LUT / 10,168 FF — twice a MIG DDR4 controller. The FF count is fully
explained by the fabric arrays (9,408 array bits + ~750 scalar). Findings:

- **`m_line [0:7]` (8 × 512 b) is read by a SINGLE dynamic index, not associatively.** It is
  expensive because at HEAD it was **written from three different sources** in the same
  process. The log says so directly: `[Synth 8-4767] Trying to implement RAM 'm_line_reg'
  in registers … Reason: 1: RAM has multiple writes via different ports in same process.`
  The write-recirculate/merge mux is a 3-deep LUT cone on **each of 4,096 bits** ≈
  **~12,300 LUT, ~59 % of the module and ~51 % of all of `u_l2c`.**
- **`r_wdata [0:7][0:3]`** is a textbook 32 × 128 single-port RAM — one `{entry,slot}` index
  on both sides — and Vivado **never even tried**: no `8-6904`, no `8-4767`, no mention in
  the log at all. The only difference from the six `p_*` arrays that *did* infer is the
  second unpacked dimension. **Cause B, confirmed by controlled experiment inside the
  design itself.**
- **The CAM is nearly free.** All the associative compares together (`lu_match` 8 × 27 b,
  `bw_mask`, `idq`, `r_cnt`) are **~200 LUT, under 1.5 %.** The MSHR is not expensive
  because it is a CAM; it is expensive because it is 8 Kbit of randomly-addressed
  flip-flops wearing a 3-deep combinational hat.

**Build/source skew, important:** `vivado.log:1211` synthesised `l2c_mshr` from line 40;
in the working tree `module l2c_mshr` is at line 56. The uncommitted `act_line` refactor
(`l2c_mshr.v:226-233`) already collapses the multi-write network and is **not** reflected
in the 20,953 figure. Predicted post-refactor: ~7-9 K LUT. Further LUTRAM conversion of
`m_line` (as 4 × `[127:0] q0..q3 [0:7]`) and the `r_*` arrays (flattened to `[0:31]`)
would take it to ~4-6 K with **zero added cycles** (async LUTRAM read) and **zero impact
on the L2 hit path** — the hit path reads only `lu_hit`/`lu_idx`/`bw_mask`, which stay in
fabric. **Do not use BRAM here:** 8 deep × 512 wide is a pathological BRAM shape and it
would add a `S_SCAN` wait state.

### 3.4 `rtc.pram` — #4

`reg [7:0] pram [0:255]` (`rtl/mac/rtc.v:238`). `u_rtc` = 1,684 LUT / 2,201 FF / 0 LUTRAM.

Two full-depth 256:1 × 8-bit read muxes (≈ 680 LUT each) plus ~200 LUT of write decode
≈ **1,650**, against 1,684 measured. The read muxes *are* the module.

Blockers, in order: **(A)** four write sources — `:469`, `:491`, `:628`, and the clear
loop at `:640`; **(E)** `assign pram_ext_rdata = pram[pram_ext_addr];` at `:309` is a
continuous assign, so there is no register for a BRAM to absorb; **(D)** the 256-entry
clear loop. The in-file comment at `:305-308` already diagnoses this correctly.

**Latency:** the expensive `pram_ext_rdata` port feeds `pram_cdc` only, and
`pram_cdc.v:60` is `assign b_addr = a_addr;` — a level-held request. `req_b_rise` comes
out of a 2-FF synchroniser, so `b_addr` is stable for ≥ 2 `pb_clk` edges before
`rdata_b <= b_rdata` samples (`pram_cdc.v:89-96`). **A registered read costs zero cycles
here**; only the doc contract at `pram_cdc.v:33` needs updating. The XPRAM port at `:581`
*is* genuinely same-cycle (the 8th address bit arrives on that edge) but `shift_out` is
not consumed until the next `rtc_clk_fall` (`:604-610`), tens of `pb_clk` later — so
registering the address there is also free.

**Fix:** 2-port synchronous RAM (port A = serial protocol, port B = snapshot), plus move
`pram_clear` from a broadside loop to a 256-cycle sweep FSM. `pram_default` is already held
1,024 pb_clk by `DEFAULT_HOLD_LOG2=11` (`pram_sd.v:176`), which is ample.
**Est. recovery ~1,450 LUT + ~2,050 FF for 1 RAMB18.** Covered by `tb-rtc`,
`tb-pram-clear-pulse`, `tb-rom-boot-rtc-smoke`.

### 3.5 `asc.fifo_a/fifo_b` — #5, the "infeasible ram_style" case

`(* ram_style = "block" *) reg [7:0] fifo_a [0:1023];` (`rtl/mac/asc.v:332-333`). The RTL
**asks for BRAM and Vivado refuses**: `[Synth 8-6849] Infeasible attribute ram_style =
"block" … trying to implement using LUTRAM`. The mapping table shows
`RAM256X1D × 32  RAM64M8 × 96` per FIFO — the `× 96` is read-port **replication**, and it
is where `u_asc`'s 1,152 LUTRAM comes from. 16 Kbit total = **one RAMB36**.

Two killers: `asc.v:1080-1083` reads each FIFO at **two different wavetable addresses on
the same edge** (plus the sample pop at `:1045` and the write) = 1W + 3R; and
`asc.v:1315` `pb_rdata <= fifo_a[pb_addr[9:0]]` shares a destination register with ~60
other case branches, so the RAM output must pass a mux before the flop and no output
register can be absorbed (cause E). The comment at `asc.v:327-331` claims a prior "fix 3"
made these BRAM-eligible; it removed the combinational form but left both real blockers.

**Fix (mechanical, but a behaviour change):** serialize the two wavetable reads across two
clocks — the consumer is a 22 kHz sample tick (`RATE_DEFAULT=35` on a 783,360 Hz phi2,
`asc.v:323`), i.e. thousands of `clk` cycles of slack; give each array one dedicated read
address and one dedicated destination register (copy the shape at `dcache.v:589-601`);
and drop the CPU readback at `:1315`, which the module's own comment at `:1303-1305` says
is write-only from the CPU side and which **no testbench reads**.
**Est. recovery ~1,152 LUTRAM + 300-450 logic LUT for 1-2 RAMB18.** Covered by `tb-asc`,
`tb-asc-mame-replay`, `tb-rom-boot-scc-asc-smoke`.

### 3.6 `icache.data_ram0..3` — #6 (CPU track, but same defect)

Same "infeasible ram_style=block" message. Root cause is precise: the read
`data_ram_q[0] <= data_ram0[req_set]` (`icache.v:668`) has a combinational address, which
is only BRAM-legal if `data_ram_q[0]` can *become* the BRAM output register — and it
cannot, because `data_ram_q` is a **variably-indexed** array also written from three
non-RAM sources (`icache.v:486` reset loop, `:745` snoop, `:795` fill). The proof that
port count is not the issue: `dcache.v:589-601` has *five* read addresses per way and
**does** get a TDP BRAM, because its destinations are **scalar** `data_ram_q0..3` written
only by the RAM.

Fix: split `data_ram_q` into four scalars, move the fill/snoop bypass into a separate
`fill_data_q` selected by the existing `hit_data` mux. No protocol change; reads only
happen in `S_IDLE` and writes only in the fill states, so no collision. ~600 LUTRAM +
200-500 logic LUT for 16 RAMB18. **Bonus:** 16 RAMB18 buys 512 sets as cheaply as 64 —
converting and raising `NUM_SETS` to 512 gives a **32 KB I-cache in the same patch.**

---

## 4. Things that look like this defect but are not

- **`u_xbar` (4,779 LUT).** Genuine 4 × 6 crossbar routing mux at 128 bits. Every payload
  "array" (`mw_wdata`, `mr_rdata`, `axi_xbar.v:791,996`) is a **wire bundle**, not storage.
  The `reg` arrays are per-slot scalar control state summing to ~940-1,000 bits against
  994 measured FF — the module holds no data buffer at all. Leave it.
- **`u_pram_sd` (3,252 LUT) — a false lead.** 422 of that is the child `sd_ctrl` instance;
  the array `sec_buf [0:511]` at `pram_sd.v:311` is **already** a BRAM (2 RAMB18, cell
  `sec_buf_reg_1_bram_0` in `timing_synth.rpt:76678`), deliberately un-reset with a comment
  explaining exactly why. A full read of the module finds ~395 bits of state (vs 428 FF
  reported) and an honest bottom-up LUT count of ~350. The remaining ~2,480 are **other
  modules' cells attributed here** by `-flatten_hierarchy rebuilt`: `timing_synth.rpt:32410`
  names `u_pram_sd/pram_clear_pb_meta_reg`, a signal declared at
  `fpga_top_peripherals.vh:1012` in a clock domain `pram_sd` does not have. Eleven
  instantiated modules have no row in the report at all for the same reason. **Re-measure
  with `-flatten_hierarchy none` before chasing any leaf-row anomaly.**
- **`u_pbus` (1,168 LUT).** Zero arrays. The cost is `decode_slot()` **instantiated twice**
  (`peripheral_bus.v:493-494`, ~250-300 LUT of duplicated 11-deep comparator chain),
  128-bit lane placement (~250), and two 32-bit watchdogs. Real ~200-300 LUT available from
  computing `decode_slot` once on a muxed address, but it is not a RAM problem.
- **`pic16c5x.prog` — the hypothesis was wrong.** `rtl/mac/adb_pic_fw.hex` is a real
  512-word image (507 of 512 non-zero) and Vivado read it successfully
  (`[Synth 8-3876]`, `vivado.log:1340`). It is a **512 × 12 LUT ROM** (~150-250 LUT), which
  is correct and expected — a never-written array is a ROM and Vivado bills ROMs as logic
  LUTs. There is no "thousands of LUTs" cliff waiting. `instr = prog[pc]` is a blocking
  combinational fetch feeding ADB bit-bang timing; making it BRAM is structural. **Leave it.**
- **`scc.wra`/`wrb`.** Parallel constant-index reads and 16-way simultaneous writes — a
  genuine register file. ~80 LUT. Leave it.
- **`dma_ctrl` and `scsi_trace_ring` are de-instantiated.** Their 5,120 and N bits cost
  nothing today (`fpga_top_dma.vh:17-25` records `u_dma_ctrl` measured 2,024 LUT / 814 FF
  of dead weight before removal). **Fix `ch_buf[4][16]` (cause B) before re-instantiating.**
- **`u_dbg_vio` (2,683 LUT / 4,834 FF)** is a generated Xilinx VIO IP, not RTL, configured
  at `synth/vivado.tcl:669-760`. 903 probe bits produce 4,834 FF (~5.4 FF/bit) because
  `CONFIG.C_EN_PROBE_IN_ACTIVITY {1}` (line 750) adds per-bit toggle-detect logic.
  **Setting it to `{0}` should recover roughly half the FFs and most of the LUTs** with no
  loss of value readback — `synth/vio_dashboard.tcl` does not use the activity indicator.
  Note the IP-cache trap at `synth/vivado.tcl:672-681`: any width/config change **requires**
  bumping `probe_map=vNN` or the stale IP is silently reused. `ENABLE_VIO=0` is blocked by
  policy at `synth/vivado.tcl:200`.

---

## 5. Appendix — the CPU side (out of scope, flagged)

The brief targets the SoC, but the LUT budget is global and the same defect is present:

| Instance | LUT | LUTRAM | FF | BRAM | Note |
|---|---:|---:|---:|---:|---|
| `u_rob` | **14,154** | **0** | **23,787** | **0** | 52 parallel 64-deep arrays. Largest fabric-storage structure in the design. Genuinely part-CAM (broadcast wakeup, flush-all) — but `commit`'s `mb_*` arrays at the *same* depth 64 all inferred as RAM64M8, so the boundary is worth auditing entry by entry. |
| `u_alu` | 12,873 | 0 | 1,227 | 0 | logic, not storage |
| `u_dec` | 12,668 | 0 | 569 | 0 | logic, not storage |
| `u_commit` | 9,321 | 28 | 2,026 | 0 | `mb_*` already inferred |
| `u_dcache` | 8,355 | 0 | 4,252 | 24×18 | data already BRAM — the reference implementation |
| `u_iq_fp` | 5,905 | 16 | 232 | 0 | 8-entry FP IQ at 5,905 LUT is worth a look |
| `u_icache` | 3,674 | 1,000 | 1,357 | 0 | §3.6 |

---

## 6. Recommended order of work

| Order | Item | Est. LUT | Risk | Gate |
|---|---|---:|---|---|
| **0** | ~~`scanout_line_fetch` beat banking~~ | **~5,900 + 4,100 FF** | low | **LANDED**, tbs green |
| 1 | `l2c_mshr` — land the `act_line` refactor already in the tree, then `r_*` → 1-D and `m_line` → 4 × LUTRAM | ~15,000 | med | owned by the l2c agent |
| 2 | `video.regs` → 1W/2R RAM (§3.2) | ~5,400 + 8,200 FF | med | **fix the 10 pre-existing `tb-dafb` IRQ failures first** |
| 3 | `rtc.pram` → 2-port RAM + clear sweep (§3.4) | ~1,450 + 2,050 FF | med | `tb-rtc`, `tb-pram-clear-pulse` |
| 4 | `asc` FIFOs → BRAM (§3.5) | ~1,550 | med | `tb-asc`, `tb-asc-mame-replay` |
| 5 | `icache.data_ram*` → BRAM, optionally 8× capacity (§3.6) | ~800 | low-med | CPU track |
| 6 | `debug_vio` `C_EN_PROBE_IN_ACTIVITY {0}` + trim `probe_in22` (187 b) | ~1,300 + ~2,400 FF | low | must bump `probe_map=vNN` |
| 7 | `peripheral_bus` single `decode_slot` | ~250 | low | `tb-peripheral-bus` |
| 8 | `fb_reader/u_rsp_fifo` registered read | ~240 | low | `tb-fb-reader-ddr-chain` |
| 9 | `q700_eth_sonic` hoist + LUTRAM | ~250 | med | `tb-q700-eth-sonic` |
| — | `iwm_stub.param_ram` | ~35 | low | not worth the churn |

Items 0-4 alone are on the order of **28,000 LUT**, or ~13 % of the device — roughly
2.7 MIG DDR4 controllers, and comfortably the budget for a larger CPU core.

---

## 7. The general rule, for new RTL

1. **Never declare a 2-D unpacked array** (`reg [W-1:0] x [0:A][0:B]`). Vivado's RAM
   extractor does not see it. Flatten to `[0:A*B-1]` with an explicit `{i,j}` address.
   This one rule accounts for `lb_word`, `r_wdata`/`r_wstrb`/`r_qoff`/`r_need`/`r_wr`, and
   `ch_buf`/`ch_desc_buf`.
2. **One write site per array.** An unrolled `for` loop that writes N words in one cycle is
   N write ports. Bank the array so the natural write granularity is one entry.
3. **No reset `for`-loop over an array.** It costs ~0 LUT itself but blocks all RAM
   inference. If the array is written-before-read, drop it (and say so in a comment, as
   `pram_sd.v:303-310` and `video.v` already do). If not, use a sweep FSM.
4. **A registered read must land in a register nothing else writes.** `dcache.v:342-345`
   (scalar `data_ram_q0..3`, BRAM ✓) vs `icache.v:211` (`data_ram_q[victim_way]` written
   from 3 sources, BRAM ✗) is the same file-pair proof.
5. **Constant-index taps count as read ports.** Mirror them into shadow FFs instead — six
   32-bit shadows cost 192 FF and buy back a 2,720-LUT mux (`video.regs`).
6. **Async read ⇒ LUTRAM, registered read ⇒ BRAM.** Choose deliberately. If the consumer
   needs same-cycle data (`peripheral_bus.v:1283-1292`'s `pb_rd_same_cycle_ok` whitelist),
   distributed RAM is the only free option — and at 5.4 % utilisation there is plenty.
