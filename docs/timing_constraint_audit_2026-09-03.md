# Timing-constraint audit, 2026-09-03

Audit of `synth/vivado.tcl`'s CDC / clock-group constraint set, triggered by
`docs/BUG_calibration_word_misplaced_0d00.md` Part 17's documented-but-unfixed
`get_active_core_clock()` bug, and by a post-place timing report on
`build/vivado_divfix_100mhz` that appeared to show 16 failing endpoints outside
the CPU.

Everything below was measured, not reasoned about in the abstract. All Vivado
runs were **read-only** analysis passes over existing checkpoints
(`open_checkpoint` + `report_*`); no checkpoint, bitstream or build directory
was written.

---

## 0. Headline

| | |
|---|---|
| Constraint bugs found | 4, all live in every production build, all fixed |
| Violations *removed* by the fixes at `CORE_CLK_DIVIDE=1` | **0** |
| Violations *removed* by the fixes at `CORE_CLK_DIVIDE=8` | **6695** of 8146 failing endpoints (all phantom fabric↔MIG cross-domain) |
| *New* violations introduced by the fixes | **0**, in both cases |
| Checks *added* (i.e. made stricter) | 30 Gray-pointer `set_bus_skew` bounds that had been applying to nothing; post-route timing + bus-skew reports |
| Timing exceptions added that hide anything | **none** — no `set_false_path` was added by this work |

Genuine violations that remain, and are deliberately left reported, are listed
in §5.

---

## 1. Correcting the premise: what the 16 failing endpoints actually were

The investigation was briefed as: `build/vivado_divfix_100mhz/reports/timing_place.rpt`
shows WNS −0.368 ns with 16 failing endpoints, broken down as `dbg_hub` (−0.077),
`u_dbg_pb_to_core/u_bridge/w_fifo/wptr_bin_reg[3]` (−0.041) and
`u_pbus/wr_addr_q_reg[18/14]` (−0.102).

That breakdown does not hold up. Read directly out of the report:

* The **16 failing endpoints at WNS −0.368 ns are setup**, and every one of them
  is **intra-`mmcm_clkout0`** — the DDR4 MIG UI domain at 333 MHz. The endpoints
  are `u_ddr/u_repo_to_pcie_mig/wr_pad_remaining_reg[0..8]/CE` and
  `wr_pad_idx_reg[0]/CE`, reached through 12 logic levels of real RTL
  (`CARRY8=1 LUT2=1 LUT4=4 LUT6=6`, 2.976 ns data path against a 3.000 ns
  requirement). Nothing to do with `dbg_hub`, `u_pbus` or any FIFO pointer.
* The three numbers quoted in the brief (−0.077 / −0.041 / −0.102) are the
  **WHS (hold)** column of the report's Intra Clock Table for `INTERNAL_TCK`,
  `pb_clk` and friends — a different check, mis-read as the setup breakdown.

More importantly:

* **The build in question met timing.** It finished while this audit was
  starting. Its own post-route summary (`build/vivado_divfix_100mhz/timing_summary.rpt`,
  reproduced independently here from `checkpoints/route.dcp`) reads:

  ```
  WNS +0.031  TNS 0.000  failing setup endpoints 0   / 358390
  WHS +0.010  THS 0.000  failing hold  endpoints 0   /  77170
  WPWS -0.029 TPWS -0.029 failing PW   endpoints 1   / 121997
  ```

  Every one of the 16 post-place setup violations and all 1986 post-place hold
  violations was closed by `route_design` + post-route `phys_opt_design`. Hold
  violations at the *place* stage are the expected steady state of this flow —
  the router fixes hold by inserting routing detour, and it is not asked to
  before then.

**Consequence for the task as briefed:** none of the three named paths needed a
constraint, and constraining any of them would have hidden a check that is
currently passing. See §3 for the per-path justification.

---

## 2. Bugs found and fixed

### Bug 1 — `get_active_core_clock()` never finds the real core clock

*(This is the bug `BUG_calibration_word_misplaced_0d00.md` Part 17 documented
and deliberately left alone.)*

The proc guessed the core clock by name: `core_clk`, then `fabric_clk100`, then
`sysclk200`. Neither `core_clk` nor `sys_clk` is ever created by an explicit
`create_clock` — `synth/fpga_top.xdc` and `synth/fpga_top_real_mig.xdc` create
only `sysclk200` (on port `sys_clk_p`) and `fabric_clk100` (on port
`fabric_clk_p`); everything else is auto-derived by Vivado.

Why it appeared to work at `CORE_CLK_DIVIDE=1` and silently broke elsewhere:
`u_fabric_core_bufg_gt` (`rtl/soc/fpga_top_clocks.vh:145`) takes
`DIV = FABRIC_GT_CORE_DIV`, which is `3'd0` at `CORE_CLK_DIVIDE=1`. A `BUFG_GT`
with `DIV=0` does not divide, so Vivado creates no new generated clock and
propagates `fabric_clk100` straight through — the name fallback lands on the
right object by luck. At divide 2/4/8 the `BUFG_GT` does divide, Vivado
auto-derives a generated clock named `sys_clk`, and `fabric_clk100` is left
with **zero endpoints**.

Measured, from the two archived reports:

| Build | core clock object | `fabric_clk100` endpoints | `sys_clk` endpoints |
|---|---|---|---|
| `vivado_divfix_100mhz` (DIVIDE=1) | `fabric_clk100` | 281609 | *(no such clock)* |
| `timing_20260903_160349.rpt` (DIVIDE=8) | `sys_clk` | *(blank — none)* | 271892 |

So on every non-`DIVIDE=1` build, the fabric↔MIG `set_clock_groups -asynchronous`
was being constructed against an empty clock, and the entire core domain stayed
cross-timed against the 333 MHz MIG UI domain.

**Fix:** resolve the clock from the netlist *pin* that generates it
(`*u_fabric_core_bufg_gt/O`), which is correct at every divide value and immune
to Vivado's auto-naming. This is the same technique `apply_video_cdc_constraints`
already used for the pixel clock. Name lookups are kept only as a fallback for
netlists with no `fpga_top_clocks.vh` (CPU-only OOC, `SIM_MODEL`), and `sys_clk`
is now among the names tried.

### Bug 2 — `pb_clk_src` is not a stable clock name

Every pb-clock constraint looked the clock up as the literal string `pb_clk_src`.
The auto-derived name is not stable, and this is visible in the repo's own build
logs:

* `synth/slowclock_diag_impl.out:5109,5195` — resolved. Prints
  `=== VIDEO CDC FALSE_PATH ===` and `REAL MIG CDC CLOCK GROUPS: (fabric_clk100 pb_clk_src)`.
* `synth/slowclock8_impl.out:5262` — did **not** resolve. No false-path line,
  and the MIG group reads `(fabric_clk100)` only.
* `build/vivado_divfix_100mhz` — did **not** resolve. Its Clock Summary names the
  clock `pb_clk`, so `get_clocks pb_clk_src` returns empty and every pb_clk
  constraint silently did nothing.

**Fix:** `get_active_pb_clock`, resolving via `*u_fabric_pb_bufg_gt/O`.

### Bug 3 — `set_clock_groups` does not walk generated clocks

Even with the right root name, `set_clock_groups -asynchronous -group fabric_clk100 …`
constrains **only `fabric_clk100`**. Per UG903 it does not implicitly include
that clock's generated children. The fabric family is
`fabric_clk100 → {sys_clk, pb_clk, pclk_unbuf → al9134_clk_fwd}`, so `sys_clk`
(the real core clock at DIVIDE≠1), `pb_clk`, `pclk_unbuf` and `al9134_clk_fwd`
were all left outside the group. Same on the MIG side for the `pll_clk[*]`
XIPHY clocks generated off `mmcm_clkout0`.

**Fix:** `_clock_family` helper wrapping `get_clocks -include_generated_clocks`,
applied to both sides of the MIG and PCIe groupings.

### Bug 4 — the Gray-pointer `set_bus_skew` bound applied to **zero** instances, always

`synth/vivado.tcl` carries ~60 lines of (correct, well-argued) commentary about
why the async-FIFO Gray pointers need a `set_bus_skew` inter-bit bound that
survives `set_clock_groups -asynchronous`. It then never applied it. Verbatim
from every build log, and reproduced here directly against
`build/vivado_divfix_100mhz/checkpoints/synth.dcp`:

```
=== ASYNC_FIFO GRAY-POINTER BUS_SKEW BOUND (T6): 0 instance(s)/direction(s)
    actually constrained, 30 attempted-but-skipped ===
```

(30 = 15 `async_fifo` instances × 2 crossing directions, across
`u_ddr/u_core_to_mig_ui`, `u_dbg_pb_to_core/u_bridge` and `u_pb_s1_cdc/u_bridge`.)

Root cause, isolated by instrumenting the proc on `synth.dcp`:

```tcl
set src_anchor  [lindex $src_bus 0]
set src_clk_pin [get_pins -quiet -of_objects $src_anchor -filter {IS_CLOCK}]
```

`$src_bus` is a Vivado collection that has already shimmered to a plain Tcl
list, so `lindex` yields a **name string**, not a cell object — and
`get_pins -of_objects <string>` glob-matches that string. Every such name ends
in a bit index:

```
u_dbg_pb_to_core/u_bridge/ar_fifo/wptr_gray_reg[0]
```

and in a glob pattern `[0]` is a **character class**, so the pattern only matches
a cell literally named `…wptr_gray_reg0`, which does not exist. `-quiet`
swallowed the miss and the "source clock unresolved" skip path fired for every
instance. Measured directly:

```
get_pins -of_objects <name string>            -> 0 pins   (old code)
get_pins -of_objects [get_cells <name>]       -> 1 pin, clock = pb_clk
get_pins "<name>/C"                           -> 1 pin, clock = pb_clk
```

**Fix:** `_t6_clock_of_cell_name`, which re-resolves the name to a real cell
object first and falls back to naming the clock pin explicitly. After the fix
the same pass reports **30 of 30 constrained, 0 skipped**, on both a
`DIVIDE=1` and a `DIVIDE=8` checkpoint.

Note this fix makes analysis **stricter**, not looser: it adds 30 real
constraints that were previously absent. §4 shows they all pass.

### Not fixed, on purpose — the `TARGET_FREQ_MHZ` `set_max_delay` block

`synth/vivado.tcl`'s `TARGET_FREQ_MHZ` block uses the same broken name list.
It was left alone and annotated in place. That block is a **relaxation**
(`set_max_delay -datapath_only` overrides the clock-period check and drops
clock-skew/uncertainty accounting). It is currently a no-op at DIVIDE≠1 (lands
on the endpoint-less `fabric_clk100`) and correct at DIVIDE=1. "Fixing" it would
make a relaxation fire on more builds — the opposite of the point of this audit.

---

## 3. Per-path adjudication: artifact or genuine?

The rule applied throughout: an exclusion is only justified if the RTL makes the
path asynchronous or false **by construction**. Where that could not be shown,
the path was left reported.

### `u_pbus/wr_addr_q_reg[18]` → `u_dbg_pb_to_core/u_bridge/aw_fifo/mem_reg_0_3_14_27/RAMC/I`

**GENUINE. Not a CDC. Not constrained. Left reported.**

From the report itself, both ends:

```
Source:      u_pbus/wr_addr_q_reg[18]/C
               (FDRE clocked by pb_clk {rise@0.000 fall@10.000 period=20.000})
Destination: u_dbg_pb_to_core/u_bridge/aw_fifo/mem_reg_0_3_14_27/RAMC/I
               (RAMD32 clocked by pb_clk {rise@0.000 fall@10.000 period=20.000})
Path Type:   Hold (Min at Slow Process Corner)
Requirement: 0.000ns (pb_clk rise@0.000 - pb_clk rise@0.000)
Logic Levels: 0
```

Same clock, same edge. This is a plain intra-domain hold path — the peripheral
bus writing an address into the bridge's write-data FIFO memory. It fails by
0.102 ns at the place stage purely from clock-network skew across the
fanout-8398 `pb_clk` global net (SCD 2.306 ns vs DCD 2.939 ns, only 0.410 ns
removed as pessimism). A `set_clock_groups -asynchronous` cannot even affect a
same-clock path, and a `set_false_path` here would delete a real check.

It is closed by the router: post-route WHS for this build is **+0.010 ns with
zero failing hold endpoints**.

### `u_dbg_pb_to_core/u_bridge/w_fifo/wptr_bin_reg[3]` → `w_fifo/mem_reg_0_15_14_27/RAMA/WADR3`

**GENUINE. Not a CDC. Not constrained. Left reported.**

The brief's premise — that this is "a Gray-coded async-FIFO pointer,
asynchronous by construction" — is factually wrong for this register.
`rtl/board/async_fifo.v` distinguishes them explicitly:

```verilog
reg [PW-1:0] wptr_bin;   // binary write pointer          (line 318)
reg [PW-1:0] wptr_gray;  // Gray write pointer (sent to rclk)  (line 319)
...
always @(posedge wclk ...) begin
    wptr_bin  <= wptr_bin_next;
    wptr_gray <= wptr_gray_next;
    if (...) mem[wptr_bin[DEPTH_LOG2-1:0]] <= wr_data;   // line 573
end
```

`wptr_bin` never crosses anything: it is written on `wclk` and read on `wclk` as
the distributed-RAM write address. The registers that actually cross are
`wptr_gray → wptr_gray_r1_r` (line 646) and `rptr_gray → rptr_gray_w1_r`
(line 594), both `(* ASYNC_REG = "TRUE" *)`. The report agrees — source and
destination are both `clocked by pb_clk`, requirement `0.000ns`, 0 logic levels.

Also closed by the router (post-route WHS +0.010, zero failing).

The *real* CDC registers in this module are handled — by the existing
`set_clock_groups -asynchronous` (fixed, §2) plus the Gray-pointer
`set_bus_skew` inter-bit bound (made functional, §2 Bug 4).

### `dbg_hub/…/U_CMD6_WR/shift_reg_in_reg[*]` on `INTERNAL_TCK`

**GENUINE (marginal, vendor IP). Not constrained. Left reported.**

These are hold violations (−0.077 ns worst), intra-`INTERNAL_TCK`, entirely
inside Xilinx's XSDB debug-hub IP: `shift_reg_in_reg[n]/C` → the hub's own write
FIFO `RAM_reg_0_15_0_13/RAM*/I`, driven by 4.9 ns of raw skew on the TCK
distribution network (SCD 3.78 ns vs DCD 8.69 ns). Post-route they are met
(`INTERNAL_TCK` WHS +0.017 ns, zero failing).

On the standard Xilinx treatment: `C_CLK_INPUT_FREQ_HZ` / `C_ENABLE_CLK_DIVIDER`
govern whether the hub divides its clock, which is what determines the
"TCK must be below the core clock or `dbg_hub` enumeration fails" behaviour this
project has already established. They have no bearing on these intra-TCK hold
paths. The hub is already correctly declared — `INTERNAL_TCK` appears in the
Clock Summary properly constrained at 50 ns / 20 MHz, `report_drc` raises no
debug-hub clock issue, and enumeration works on hardware today. **Nothing was
changed here**, precisely so the working TCK relationship is not disturbed.
Adding a false path or clock group over `INTERNAL_TCK` would be exactly the
"wrong `set_false_path`" this audit is trying to avoid.

### `u_ddr/u_repo_to_pcie_mig/wr_pad_remaining_reg[*]/CE` on `mmcm_clkout0`

**GENUINE. Real logic. Not constrained. Left reported.** See §5.

### The fabric↔MIG cross-domain family

**ARTIFACT — genuinely asynchronous by construction. Correctly excluded.**

Justification from the RTL and the board, not from convenience:

* The entire fabric family is sourced from the `fabric_clk_p/n` MGTREFCLK pair on
  **AB7/AB6** — `rtl/soc/fpga_top_clocks.vh:38-163`: one `IBUFDS_GTE4` →
  `fabric_clk_odiv2` → three separate `BUFG_GT` instances (`u_fabric_core_bufg_gt`
  → `sys_clk`, `u_fabric_pb_bufg_gt` → `pb_clk_src`, `u_fabric_ref_bufg_gt` →
  `video_ref_clk`).
* The entire MIG PHY/UI family is sourced from the physically separate
  `sys_clk_p/n` 200 MHz pair on **T24/U24** (`synth/fpga_top.xdc:26-33`) through
  the MIG's own MMCM.
* Two independent board oscillators on two different pin pairs, with no phase
  relationship that STA can or should assume.
* Every real crossing between the two goes through `rtl/soc/axi_async_bridge.v`
  (5× `rtl/board/async_fifo.v`, one per AXI channel: Gray-coded pointers into
  `ASYNC_REG` 2-flop synchronisers — Pattern B in `docs/clocking.md`), or a plain
  2-flop status synchroniser (Pattern C).
* The one property those FIFOs genuinely depend on and which
  `set_clock_groups` does *not* cover — that all bits of a Gray word land within
  one source period of each other — is bounded separately by `set_bus_skew`,
  which is checked independently of the async grouping. That bound is now
  actually applied (§2 Bug 4), so the exclusion is paired with the check that
  makes it safe rather than standing alone.

### `pclk_unbuf ↔ pb_clk` false path

**Kept as authored, but it is now live where it previously was not** (it depended
on the broken `pb_clk_src` lookup). Verified to remove **zero** endpoints in
`build/vivado_divfix_100mhz`: the only `pclk_unbuf → pb_clk` crossing in that
netlist is the `u_dafb_vbl_cdc` `pulse_cdc` synchroniser, already excluded at
*endpoint* granularity by `synth/fpga_top.xdc:117`, and there are no
`pb_clk → pclk_unbuf` paths at all (neither direction appears in the Inter Clock
Table). Flagged in the source: these two clocks *do* share a root
(`fabric_clk_p`), so this is a clock-level exception over related clocks. A
future crossing here should get its own endpoint-scoped exception in
`fpga_top.xdc` rather than quietly inheriting this one.

---

## 4. Validation

Method: `open_checkpoint` an existing checkpoint read-only, enumerate every
failing endpoint, apply the new constraint procs on top, enumerate again, and
diff the two sets item by item (not just by count). Scripts and raw outputs are
throwaway; the numbers are reproducible from the checkpoints named below.

### `build/vivado_divfix_100mhz/checkpoints/place.dcp` — `CORE_CLK_DIVIDE=1`

```
BEFORE  WNS=-0.368  failing_setup=32   WHS=-0.949  failing_hold=2216
AFTER   WNS=-0.368  failing_setup=32   WHS=-0.949  failing_hold=2216
```

(Counts here are timing *paths*; `report_timing_summary` counts unique
*endpoints*, hence 32 vs 16 — `mmcm_clkout0` and `mmcm_clkout0_1` are two clock
objects on one physical BUFG output. Before/after are counted identically.)

Endpoint-set diff: **0 removed, 0 new — byte-identical.** Expected: at DIVIDE=1
the old name lookup already landed on the right core clock, so the only change
is the newly-live pb_clk grouping and false path (both provably empty here) and
the 30 new bus-skew constraints.

Resolution changed as intended:

```
core_clk resolved -> 'fabric_clk100'      (unchanged)
pb_clk   resolved -> 'pb_clk'             (was: empty)
T6 bus skew: 30 constrained, 0 skipped    (was: 0 constrained, 30 skipped)
MIG group now: (fabric_clk100 pb_clk al9134_clk_fwd pclk_unbuf)
           vs  (mmcm_clkout0 mmcm_clkout0_1 mmcm_clkout6 mmcm_clkout6_1
                pll_clk[0] pll_clk[0]_1 pll_clk[1] pll_clk[1]_1
                pll_clk[0]_DIV pll_clk[0]_1_DIV pll_clk[1]_DIV pll_clk[1]_1_DIV)
```

Newly-enforced bus-skew check: 42 constraints reported (12 pre-existing vendor
XPM ones + 30 new), **worst slack +2.387 ns, zero failing**. The check is now
real and it passes.

### `build/vivado_slowclock8_diag/checkpoints/route.dcp` — `CORE_CLK_DIVIDE=8`

This is where the bug actually bites.

```
BEFORE  WNS=-4.163  failing endpoints 8146
AFTER   WNS=-2.620  failing endpoints 1451
```

Endpoint-set diff: **6695 removed, 0 new.**

`core_clk` now resolves to `sys_clk` (was `fabric_clk100`, which has no endpoints
in this build). The removed endpoints are the fabric-domain half of the
cross-domain pairs:

```
5315  u_l2c            152  u_xbar
 276  u_cpu            139  u_video
 261  u_jtag_n2w       106  u_ddr (fabric side)
 235  u_dbg_vio         16  u_boot_fsm
 172  u_scanout_reader  11  u_vram_lane_mux
```

— i.e. the whole SoC being cross-timed against a 333 MHz clock it only ever
touches through `axi_async_bridge`. What remains after the fix is entirely
inside the MIG UI domain:

```
2724  u_ddr        178  u_mig_ddr4
```

which matches the report's own Intra Clock Table exactly (`mmcm_clkout0` intra:
−2.620 ns, 1451 endpoints). **The residual is genuine and is left reported.**

This also shows the fault was not merely cosmetic. The router spent its effort
on 6695 phantom cross-domain paths and left the real intra-MIG-UI paths at
−2.620 ns. The `DIVIDE=1` build of the same design closes at **+0.031 ns**.

### Post-route baseline, regenerated read-only from `route.dcp`

```
WNS +0.031  failing setup 0 / 358390
WHS +0.010  failing hold  0 /  77170
WPWS -0.029 failing PW    1 / 121997
bus skew: 12 constraints, worst +9.321 ns, 0 failing
```

---

## 5. Genuine violations that remain

1. **`u_ddr/u_repo_to_pcie_mig/wr_pad_remaining_reg[0..8]/CE`, `wr_pad_idx_reg[0]/CE`
   — setup, intra-`mmcm_clkout0` (333 MHz MIG UI), −0.368 ns post-place.**
   Real RTL, 12 logic levels, 2.976 ns of data path against a 3.000 ns
   requirement, 67% route. Not constrained — there is nothing false about it.
   It closes post-route in the `DIVIDE=1` build (+0.031 ns) and does **not**
   close in the `DIVIDE=8` build (−2.620 ns, 1451 endpoints), which is now the
   only remaining failure there and is worth its own investigation.

2. **`u_mig_ddr4/…/xiphy_rxtx_bitslice/D[1]` vs `D[2]` — Min Skew, −0.029 ns.**
   The single failing endpoint in the shipped `DIVIDE=1` build. A hard-block
   XIPHY min-skew requirement inside vendor MIG IP. Not addressable by a clock
   constraint; the repo already has `build/repair_min_skew_ranged.tcl` and
   `tight_setup_hold_pins.txt` from prior work on this class.

3. **Post-place hold violations generally** (1986 endpoints, WHS −0.949 ns in
   `timing_place.rpt`). Not violations of the shipped design — the router closes
   all of them. They are reported here only because the place-stage report was
   being read as if it were final. See §6.

---

## 6. Reporting defect: no post-route report under `reports/`

`build/*/reports/` contained `timing_synth.rpt` and `timing_place.rpt` but no
route-stage peer. The post-route summary *was* being written — to
`$output_dir/timing_summary.rpt`, plus a timestamped archive copy under
`synth/timing_reports/` — but not where the other per-stage reports live and not
under a `timing_route` name, so it is easy to miss. This audit's own briefing
missed it, and drew conclusions from place-stage numbers about a build that had
already met timing.

Added to `synth/vivado.tcl`:

* `reports/timing_route.rpt` — `report_timing_summary` on the routed design,
  matching the `timing_synth` / `timing_place` convention.
* `reports/bus_skew_route.rpt` and a `=== BUS SKEW SUMMARY ===` stdout block —
  bus skew is checked independently of `set_clock_groups -asynchronous` and is
  **not** folded into `report_timing_summary`'s WNS/WHS/TNS. Now that 30 real
  Gray-pointer skew bounds actually apply, a CI gate reading only WNS would
  otherwise pass a build whose CDC skew bound is violated.

The existing `timing_summary.rpt` and its archive copy are unchanged, so any
tooling that reads them keeps working.

---

## 7. What was deliberately not done

* **No `set_false_path` was added by this work.** Not one.
* `dbg_hub` / `INTERNAL_TCK` left entirely alone (§3) — the TCK-below-core-clock
  behaviour this project depends on is not disturbed.
* No RTL touched.
* The `TARGET_FREQ_MHZ` `set_max_delay` relaxation left as-is and annotated (§2).
* No hardware, JTAG lease or bitstream touched; no build directory written.

## 8. Follow-ups worth someone's time

* The `DIVIDE=8` build's residual −2.620 ns / 1451 endpoints in
  `u_ddr` + `u_mig_ddr4` (§5.1) is now unmasked and unexplained. It should be
  re-measured on a fresh build with the corrected constraints rather than
  inferred from a checkpoint built under the broken ones — the router's effort
  allocation, not just the report, was distorted.
* `_t6_group_cells_by_instance_prefix` still matches async-FIFO instances by
  hierarchical *name pattern* across the whole design, which the existing
  in-file "FRAGILITY NOTE" already flags. Untouched here; scoping by
  `REF_NAME == async_fifo` would be more robust.
* A full implementation run with these constraints has not been done. All
  validation above is STA-only over existing checkpoints, which is sound for
  "does the constraint set change what is reported" but does not measure how
  place & route behave under the corrected constraints. The `DIVIDE=8` numbers
  in §4 strongly suggest the corrected constraints will *improve* P&R outcomes
  on divided-clock builds; that should be confirmed by building one.
