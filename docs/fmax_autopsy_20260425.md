# Fmax autopsy — main `2f9acd4`, post-place

Place-only checkpoint, 100 MHz target, real MIG, full-video build. Reports
land in `build/vivado_100mhz_debug/autopsy/`.

## Headline

| Clock | Period | WNS | Failing |
|---|---|---|---|
| **fabric_clk100** (CPU + peripherals) | 10 ns | **−9.119 ns** | **6799** |
| pclk_unbuf (HDMI 148.5 MHz) | 6.73 ns | +0.129 | 0 |
| sys_clk_p (MIG 200 MHz) | 5 ns | +3.371 | 0 |
| mmcm_clkout0 (MIG UI 333 MHz) | 3 ns | +0.046 | 0 |
| mmcm_clkout6 (HDMI 166 MHz) | 6 ns | +2.661 | 0 |

Routing typically gains ~0.5 ns post-place; **−9.1 ns is not closeable at 100 MHz with route-only**. This is a regression vs the working `3c812f1` bitstream (WNS=+0.06 ns) — caused by the H4/H5/I1/I3/I4/#256 stack landing without retiming.

200 MHz target needs WNS ≥ 0 at 5 ns period, i.e. another **−14 ns** must come out of the worst path on top of the 100 MHz fix.

## Critical path #1 (cluster B: 670/1000 paths, 69-83 logic levels)

```
Source:      u_cpu/u_if/pc_reg[1]_rep__33/C  (PC, replicated)
Destination: u_cpu/u_if/rot_l0_data_reg[124]/CE  (fetch rotation buffer write enable)
Data Path:   19.059 ns  (logic 5.31 ns / route 13.75 ns)
Logic Levels: 83  (49× LUT6, 13× LUT2, 8× LUT5, 6× CARRY8, ...)
```

Net fanouts on the path: 41, 124, 65, 22, 32, 19, **316**, **205**, 122, 70, 68, 64, 38, 35 ... — wide combinational fan-out causing the 13.75 ns of net delay. Logic delay alone is fine; the fan-out + congestion is the killer.

What's on this path: `PC → V2 decode semantics (decode_semantics, decode_ea_v2, decode_uop_assemble all instantiated under u_if) → q_arch_src_a, q_is_rts, q_elim_arch_dst, q_size, q_imm, q_exc_vec, ... → rotation-buffer write-enable computation`. Effectively: **everything the front-end computes between fetched bytes and dispatch payload, all in one cycle, then gating fetch advancement**.

The previous landing `#231 fetch/decode 60-level critical path retime` brought this from ~60 to closeable at 100 MHz; subsequent landings re-grew it to 83 levels.

### Why it grew

| Landing | Damage |
|---|---|
| #249 H4 (PRF tag widen 6→7, ROB tag widen 5→6) | +1 LUT level on every tag-comparison node |
| #251 H5 (q_d_* lane-1 wiring) | Doubled mux-width into q_*; lane-1 needs the same V2 outputs in parallel |
| #252 I1 (lane-1 elim) | Extra logic stage for elim-vs-allocate decision |
| #254 I3 (split-IQ pair fire) | Extra mux into IQ enqueue gating |
| #255 I4 (ROB 32→64) | head_ptr/tail_ptr widened 5→6 bits + 64-entry compares |
| #256 (jtag_debug_full_reset) | OR-gate added in front of every reset receiver — see fanout section |

## Critical path cluster A (~330 of top-1000 paths, 36-43 levels)

Below the 69-83-level cluster sits a "second tier" at 36-43 levels. These are also way over 200 MHz budget. Did not single-step through report (paths #1-50 all converge on the same #1 critical endpoint) but the structural pattern strongly suggests:

- ALU/AGU result-bus → CDB tag broadcast → IQ wakeup CAM → next-cycle issue arbitration
- Or: dispatch-stage `q_*` register → RAT alloc → ROB enqueue → IQ enqueue → wakeup

Need to confirm with `report_timing -nworst 50 -unique_pins` (or by route-completion) before pipelining.

## DRC: combinational loop (LUTLP-1)

```
1 violation, 435 LUT cells in the loop
Root nets: u_cpu/u_rat/e_pdst_b[14][0]_i_*  (lane-1 dual-dst phys-tag write, arch reg 14 / A6)
Crosses through u_cpu/u_ccr_rat/q_valid_reg
```

Source: lane-1 dual-destination rename (`#79` dual-dst-prf + `#252` I1 lane-1 elim). The feedback path: lane-1 dual-dst write → CCR-RAT validation → free-list state → lane-1 alloc_ok_b → back into lane-1 dual-dst write logic.

**Vivado treats the loop as worst-case delay** — that's a big chunk of the 6799 failing endpoints (downstream paths get inflated arrival times).

Fix path: register the lane-1 dual-dst feedback. Dual-dst is rare (only `MULL.L SZ=1`, `DIVL.L SZ=1`, `EXG`-style ops) — adding 1 cycle latency on that rename path is acceptable.

## High-fanout offenders (with negative slack)

| Net | Fanout | Worst Slack | Source | Likely RTL fix |
|---|---|---|---|---|
| `u_clk_rst/rst_pipe[3]` | **20355** | **−6.79 ns** | FDPE | Add a 5th + 6th rst_pipe stage; absorb #256's `core_rst | jtag_debug_full_reset` OR into the pipeline before broadcast |
| `u_boot_fsm/rom_loading_reg_0[0]` | 1390 | **−7.18 ns** | LUT5 | `boot_rom_loading` gate is broadcast from a LUT, not a FF. Register at the source, broadcast the registered version |
| `u_cpu/u_rob/head_ptr_reg[3]_rep__1[1]` | 5467 | **−1.72 ns** | FDRE | Vivado already auto-replicated; needs RTL-side replication into per-stage groups, or convert ROB tag-CAM to per-entry registered comparators |
| `u_cpu/u_rob/head_ptr_reg[3]_rep__1[0]` | 5450 | −1.64 ns | FDRE | Same |
| `u_cpu/u_alu/bpu_update_pc[0..1]` | 574/568 | −0.45/−0.28 | LUT3 | Register one stage between ALU branch resolution and BPU update broadcast (#16's bpu-phase2 already commit-time but PC bus still hits BPU combinationally) |

## LUTRAM → BRAM migration potential

**Total LUTRAMs in design: 2876.** Distribution:

| Block | LUTRAMs | Verdict |
|---|---|---|
| u_ddr (axi_async_bridge + axi_ddr4_mig_bridge) | 1244 | **Keep LUTRAM** — CDC FIFOs, 8-32 deep × wide, BRAM would worsen timing (BRAM read latency + the FIFO is the CDC barrier) |
| u_debug/debug_ctrl (PC trace ring) | 640 | **Keep LUTRAM** — small ring buffer for debug trace, low fanout |
| u_dafb / video pipeline | ~600 | Mix of small CLUT/staging RAMs; some candidates if a path lands on them |
| u_boot_fsm | 40 | Tiny SD command FIFO; keep |
| dbg_hub | 20 | Internal Vivado IP; can't touch |
| **u_cpu (CPU body)** | **0** | All RAMs already BRAM/URAM (PRF, ROB, IQ entries are reg arrays + comparators, not RAMs) |

**Bottom line: nothing to migrate.** Task `#92 dcache-bram-infer` already cleaned the big-LUT-sink (46K LUTs of L1D LUTRAM moved to BRAM4). The 2876 remaining LUTRAMs are either intrinsically LUTRAM-friendly (CDC FIFOs) or too small/non-critical to matter.

The PRF (mentioned earlier as a candidate) is currently inferred as **distributed/registered FFs**, not LUTRAM — moving it to BRAM is a possibility but requires:
- Read latency goes from 0 cycles → 1 cycle (BRAM is registered-read).
- F1 pipeline stage (`#126` registered PRF read + CDB bypass) already absorbs that cost — so the migration is timing-free in principle.
- But: the PRF has many read ports (3-4 from iq_int, 2 from iq_mem, 1 from commit/RAT) and BRAM has only 2 ports. Need port-replication (4× BRAM blocks for 4 read ports), trading BRAM count for LUT savings.
- 96 entries × 32 bits = 3 KB per replica. 4 replicas = 12 KB total. KU5P has 18 Mb BRAM ≫ 12 KB. Acceptable.

This is an optional Stage-3 lever. Not a critical-path fix on this build.

## Resource utilization

| Resource | Used | Available | % |
|---|---|---|---|
| LUTs | 204041 | 216960 | **94%** |
| FFs | 137079 | ~432K | 32% |
| RAMB36 | 44 | 480 | 9% |
| RAMB18 | 7 | 960 | <1% |
| URAM | 24 | 64 | 38% |
| DSP | 12 | 1080 | 1% |

LUT pressure at 94% is the second concern. Adding pipeline stages adds FFs (we have headroom), but new mux/widening also adds LUTs (which we don't). Some of the 200K LUTs are decode.v's huge case-statement output muxes — V2 migration is supposed to shrink those. Stage-E sheet deletion (per task #216 completed) should already have helped; further V2 coverage will keep helping.

DSP at 12 (out of 1080) — we're using DSPs only for MUL.L sub-lane. There's ~1000 free DSPs; opportunities to use them for big adders, comparators, shifters on critical paths exist (e.g. CARRY8 chains in the critical path could become DSP58 macros).

## Reach-200 MHz roadmap

### Stage 0 (must-do, blocks even 100 MHz close)

1. **Break combinational loop in `u_rat/e_pdst_b[14]`** — register lane-1 dual-dst phys-tag write feedback. 1 cycle latency on rare op (acceptable, no IPC measurable impact). Fixes 435-LUT loop AND removes the worst-case-delay treatment from many of the 6799 failing endpoints.

2. **Fix `rst_pipe[3]` fanout/slack** — task #256's `core_rst | jtag_debug_full_reset` OR-gate has been placed at the broadcast point. Move the OR up into `u_clk_rst`, register the result into a new `rst_pipe[4]/[5]` stage before broadcasting. Should bring slack from −6.79 to ≥ 0 ns on the global reset paths.

3. **Register `u_boot_fsm/rom_loading_reg_0`** — currently a LUT5 broadcasts to 1390 sinks. Register at the source.

After stage 0, expect WNS ~ −2 to −3 ns at 100 MHz target. Most of the 6799 failing endpoints will clear once the loop is broken.

### Stage 1 (close 100 MHz cleanly with margin)

4. **Pipeline ROB head_ptr broadcast** — fanout 5450, currently 1 wide net. Add a 1-cycle pipeline buffer FF (per-bit replicated) between head_ptr and per-entry compares. ROB age compares already tolerate 1-cycle update lag (head only moves on commit; commit doesn't issue ops same cycle).

5. **Pipeline `bpu_update_pc[0..1]`** — single FF between `u_alu/bpu_update_pc` flop and BPU. Branch resolution is already 1-cycle from ALU result, so 1 more cycle of training delay is fine.

After stage 1, expect WNS ~ +0.5 to +1.5 ns at 100 MHz. Real bitstream possible.

### Stage 2 (push toward 150-180 MHz)

6. **Crack the if_stage `q_*` dispatch register chain into 3 stages** — this is the Big Retime. Currently:

   ```
   PC, fetch bytes  →  V2 decode semantics  →  q_arch_src_a, q_is_rts,
                                                q_elim_arch_dst, q_size,
                                                q_imm, q_exc_vec, ...
                                            →  rot_l0_data_reg/CE
   ```

   Split into:

   ```
   F1:  PC + fetch bytes  →  v2_sem_*  →  REG (decode_semantics outputs registered)
   F2:  v2_sem_* + ext_words  →  v2_src_ea_*, v2_dst_ea_*  →  REG
   F3:  v2_src_ea_*, v2_dst_ea_*  →  uop_assemble outputs  →  REG (q_*)
   F4:  q_*  →  rotation buffer + dispatch
   ```

   Pipeline stages = 3 added to fetch→dispatch. **IPC cost is real but absorbable** because:
   - Frontend stalls behind branches/cache-miss are already the dominant IPC limiter on this design (per `docs/bench_baseline.md` peak IPC = 0.386).
   - 3 cycles of extra fetch latency hide behind I-cache miss penalty (4-way set-associative, multi-cycle BRAM read already takes 2 cycles, so total fetch→dispatch is moving from ~5 → 8 cycles — worth it for 2× clock).

   Critical path drops from 83 levels to ~15-20 per stage. Expect WNS at 150 MHz close.

### Stage 3 (200 MHz polish)

7. **Pipeline cluster-A paths (36-43 levels)** — once #6 lands, what was cluster A becomes the new worst. Same retiming pattern applied to whichever stage they're in (likely RAT/ROB writes → IQ enqueue or ALU result → CDB → IQ wakeup).

8. **PRF port replication + BRAM migration** — 4 replicas of 96×32 PRF in BRAM (4 RAMB36s = 12 KB). Read latency stays at 1 cycle (already absorbed by F1 register). Saves the LUTs currently used as the PRF mux network. Frees LUT pressure for the retime.

9. **Convert wide CARRY8 chains to DSP58E2** — there are 6 CARRY8 in the worst path (CARRY8 is fast, but at 200 MHz becomes ~1 ns). DSP-add at 200 MHz is 1 cycle fully pipelined. The 1080-cell DSP budget is barely touched (12 used).

After stage 3, expect WNS ≥ 0 at 200 MHz.

## What I'm NOT recommending

- **`set_property ALLOW_COMBINATORIAL_LOOPS TRUE` on the RAT loop** — masks the real bug, leaves Vivado's timer pessimistic, and we lose the ability to trust closure reports.
- **Tightening `TARGET_FREQ_MHZ` to 200 MHz before stages 0-2 land** — the design will not place; place_design will give up. Need stages in order.
- **Aggressive `Performance_*` impl strategies** — tried unsuccessfully on a few prior runs (per archived timing reports). Without RTL surgery, place is bottle-necked by the 83-level path; no impl strategy can rescue that.
- **MIG path optimizations** — sys_clk_p closes with +3.4 ns slack at 200 MHz already. MIG-side timing is fine.
