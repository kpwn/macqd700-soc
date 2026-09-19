# Missed Opportunities — a "what are we not seeing" sweep

**Date:** 2026-08-19 · **Scope:** the surface *between* the targeted reviews —
`rtl/mac/` peripherals, `rtl/soc/peripheral_bus.v`, the SD stack, `boot_fsm`,
reset architecture, the debug block, and the test/build harness.

**Not re-litigated here** (cite, don't redo): `docs/soc_bus_review.md`,
`docs/l2c_perf.md`, `docs/mux_bram_review.md`, `docs/soc_clocking_study.md`,
`docs/dma_engine_design.md`, `docs/release_audit.md`, `docs/branch_triage.md`.
Scanout prefetch/concurrency was under active edit during this sweep and is
excluded.

**Core-version note.** The CPU is expected to be replaced by the **v2 core**.
`rtl/soc/cpu_socket.vh` is the seam, and everything in this document is on the
**SoC side of that seam** — it survives the core swap. Two consequences worth
stating up front:

- `debug_ctrl` (4,635 LUT) lives at `cpu/rtl/core/debug/debug_ctrl.v`, i.e.
  **CPU-side**. Auditing it for area is a v2-core question, not a SoC question,
  and is deliberately down-ranked below.
- `debug_vio` (2,683 LUT) is SoC-side (`rtl/soc/fpga_top_debug_vio.vh`) and
  does survive. So does every peripheral, the SD stack, and `peripheral_bus`.

---

## Executive summary

The single largest finding is structural and explains most of the rest:

> **`ALL_TBS` covers 82 of 139 declared `tb-*` targets. The other 57 are never
> built by any aggregate. Running them found 8 red targets, and the redness has
> a pattern: when someone deliberately corrects the RTL to match MAME, the
> testbench asserting the old behaviour goes red *silently and permanently*,
> because it is an orphan.**

The Makefile already documents the orphan gap honestly
(`Makefile:6650-6688`) and even names 47 targets as UNAUDITED. Nobody had run
them. This document does.

Ranking below is by **(value / risk)**. Where a decision looks odd but has a
comment explaining it, that is stated and the decision is left alone.

---

# TIER 1 — silently wrong, act soon

## 1.0 — `debug_stop_manager` exists twice; the copy that ships is 3 weeks stale and missing two documented halt-wedge fixes

**Value: highest in this document · Risk: low-moderate** — a live correctness
bug in the shipped bitstream, and the unit test validates the wrong copy.

`rtl/soc/fpga_top.v:154-163` declares an inline copy of the module and says:

> debug_stop_manager — **VERBATIM COPY** of
> `cpu/rtl/core/debug/debug_stop_manager.v` … **THIS copy is the one the real
> build elaborates — keep it byte-identical** with the CPU repo when bumping
> the submodule.

It is not byte-identical. `synth/vivado.tcl:435` explicitly skips the CPU file
(`if {[file tail $f] eq "debug_stop_manager.v"} { continue }`), so the SoC copy
is what synthesises — and the two module bodies are **198 lines (SoC) vs 294
lines (CPU)**.

The divergence is dated precisely. SoC copy last synced `a8154f4` (2026-07-27).
The CPU copy has gained two commits since, **never mirrored**:

- `7fdcba29` 2026-07-28 — *"debug: make watchpoint and break-PC halts
  continuable + macro-precise"*
- `0052e492` 2026-08-06 — *"core: a debug halt must not strand commit's
  IRQ-entry drain handshake"*

Four behavioural deltas are in the CPU copy and **absent from the bitstream**:

1. **The precise-stop overlay is still wired to every auto-halt source.**
   `cpu/…/debug_stop_manager.v:347` is `dbg_precise_stop_req_q <=
   dbg_precise_stop_src` (double-fault only); `rtl/soc/fpga_top.v:355` is
   `dbg_precise_stop_req_q <= dbg_auto_halt_event_w` (everything). The CPU-side
   comment explains the consequence: driving the unregistered `flush_en` on a
   registered halt decision masks µops already in the IQs/F1 **while their ROB
   entries are never cleared** — *"the head sits on a µop with nothing in flight
   anywhere … the CPU retired nothing ever again."* It then names the signature:
   *"the same signature reported from real hardware for `break-pc` + continue."*
   The shipped bitstream still has that bug on watchpoint, halt-after-N and
   halt-on-exception halts. **This is a strong candidate explanation for the
   wedges that the `m68k-jtag-wedge-recovery` skill exists to recover from.**
2. **Watchpoint halt is µop-aligned, not macro-aligned.** CPU gates on
   `dbg_retire_boundary_now` (`rob_is_last_uop`); SoC gates on
   `dbg_uop_retire_pulse`. A watchpoint on the first µop of a cracked
   `BSET`/`BCLR`/`TAS`-on-memory (or `MOVEM`/`BSR`/`PEA`) halts mid-instruction,
   reports the **previous** macro's PC, and hands the host a half-executed
   register view.
3. **Watchpoint-pending is 1 bit, not a saturating count.** CPU has
   `WATCH_PEND_MAX = 2'd2` / `reg [1:0]`; SoC has `reg`. An RMW with `rw` armed
   produces two watched accesses in one macro; the second is silently dropped.
4. **The 2026-08-06 one-cycle auto-halt handoff hole is unfixed.** The CPU copy
   adds `dbg_auto_halt_stretch_r`; the SoC copy has the bare
   `| dbg_auto_halt_event_w`. Per its comment, in that cycle *"the CPU is not
   halted … it executes a full retire cycle behind the host's back"*, making
   *"every halt's reported architectural state one macro stale at random"*.

**And the test cannot catch it:** `Makefile:243` builds `tb-debug-stop` from
`$(RTL_DIR)/soc/fpga_top.v` — the unit tb validates the **stale** copy. All four
CPU-side fixes have zero SoC-side coverage, and the suite stays green.

**Action:** copy the CPU body over the SoC body (the diff shows no SoC-only
additions, so a straight replace looks safe — re-diff at the moment of the
change). Then make recurrence impossible: either delete the inline copy and stop
excluding the CPU file, or add a `make check-cpu-sync` rule that diffs the two
module bodies and fails. **There is no such check today.**

**v2-core note:** this duplication sits *across* the CPU socket seam. The v2
core swap is the natural moment to delete the SoC copy outright rather than
re-sync it a third time.

---

## 1.1 — The SCC baud-rate divider is ~4× wrong: computed for a clock the SCC does not run on

**Value: high · Risk: medium** (changes ROM serial-poll timing)

`rtl/mac/scc.v:68-75`:

```verilog
// Pre-scaler from `clk` to the BRG reference tick.  At the Mac
// integration (`clk` = 200 MHz) the Q700 PCLK is 3.672 MHz, so
// PCLK_DIV = 54 (200e6 / 3.672e6 ≈ 54.5) is close enough
parameter integer PCLK_DIV = 32'd54,
```

The SCC is **not** clocked at 200 MHz. `rtl/soc/fpga_top_peripherals.vh:1477`
instantiates it as `scc u_scc (... .clk(pb_clk) ...)` with **no parameter
override**, and `pb_clk` is fixed at 50 MHz — `rtl/soc/fpga_top.v:371`
(`PB_CLK_HZ = 50_000_000`, commented *"Hardware keeps this fixed at 50 MHz"*),
enforced by a hard error in `synth/vivado.tcl:328-330`.

So the BRG reference is `50e6 / 54 = 926 kHz`, not 3.672 MHz — **3.97× slow**.
Every baud rate and every Tx-empty poll interval is ~4× long, through
`scc.v:127` (`pclk_tick`) → `:133` (`brg_ref`) → `:749-773` (BRG countdown) →
`tx_bits_left_*`. The correct divisor for 50 MHz is **14** (`50e6/3.672e6 = 13.6`).

**Why nobody caught it:** `tb-scc` overrides the parameter to `-GPCLK_DIV=2`
(`Makefile:2962`) to make byte times short, so the tb can never observe the
default. The default value is exercised only in the bitstream.

`rtl/mac/asc.v:317-322` already carries the corrected form of this same story
(*"VIA_PHI2_HZ = 783_360 (Q700-faithful, NOT 1 MHz)"*), so this is a straggler
from the 200 MHz era, not a systemic misunderstanding. The identical stale
"200 MHz / 1 MHz" phrasing in `via1.v:63,65,78`, `via2.v:60-62,76` and
`rtc.v:40,74,85` is **cosmetic only** — those modules derive everything from
`phi2_tick`.

**Action:** change the default to 14 and re-comment for 50 MHz; validate with a
cold-boot run, not `tb-scc`. Flag the timing shift explicitly — this speeds up
the ROM's MacsBug SCC Tx poll loop, which interacts with the boot frontier.

---

## 1.2 — `sd_scsi_bridge` toggle handshakes glitch on asymmetric reset → fake "transfer succeeded"

**Value: high · Risk: low-moderate**

`rtl/soc/sd_scsi_bridge.v:144-154, 210-220, 276-280`. Every core→pb crossing is
an unqualified toggle **reset to `1'b0`**, and the pb side edge-detects it with
no arming qualifier (`:178-185`). If a toggle happens to be `1` when its side's
reset asserts, the reset *is* a 1→0 edge.

The two resets are genuinely skewed, by construction:

| Side | Source | Path |
|---|---|---|
| core | `soc_full_rst_bank[5] \| warm_peripheral_reset` (`fpga_top_sd.vh:479`) | direct combinational OR |
| pb | `pb_full_rst_bank[2]` (`fpga_top_peripherals.vh:1096-1097`) | `xpm_cdc_async_rst` `DEST_SYNC_FF(4)` + `BUFG` (`:1122-1141`) |

So on **every** debug-full-reset and every CPU-requested `warm_peripheral_reset`
the core side resets ~4 pb-cycles + BUFG delay before the pb side. Consequences:

- `core_done_tog` → spurious `pb_done` with `pb_error_lat` also freshly zeroed:
  the SCSI target is told the transfer **completed successfully**.
- `core_rd_tog` (`:211`) → one phantom byte into `scsi.v`'s 512-byte sector
  ring, permanently desyncing the ring pointer.
- `core_wr_tog` (`:276`) → producer advances `drain_ptr`, shifting every
  following byte.

This **directly contradicts the module's own header**, `sd_scsi_bridge.v:36-40`:
*"Reset on either side asynchronously zeroes the local state; the toggle
counterparts on the other side will see no edge and stay idle."* That holds only
if the toggle was already 0.

This is a plausible mechanism for the "boot failures are exceptions downstream
of garbage DATA" pattern already recorded in the project notes.

**Action:** pair each toggle with an `armed` bit (also reset) and gate the
edge-detect on it, or fold the synchronised `pb_rst` into the bridge's
`core_rst` so both sides release together. Add a `tb_scsi_sd_e2e` scenario that
asserts **one** reset only — no current scenario does.

---

## 1.3 — The shipping 53C96 SCSI write path wedges forever on a provider error; the test that proves it is an orphan

**Value: high · Risk: n/a to report, medium to fix** (`rtl/mac/scsi.v` is owned
by another agent — propose, don't edit)

`tb-scsi-sd-e2e-c96` and `tb-scsi-sd-e2e-c96-core100` both fail 2/18, with a
self-diagnosing message:

```
FAIL c16: WEDGE — DRQ never rose for byte 16 of 20480, target still parked in
DATA OUT (status=0x90 fifo=0x00).  REQ is deasserted with the ring full and
nothing can re-arm it: the provider stopped draining and S_DATA_OUT has no
vh_done/vh_error escape.
```

The equivalent bug **was already fixed once** for the other front end — commit
`c44aeff` (2026-08-08), *"scsi: escape a mid-stream provider error in
S_DATA_OUT — REQ was parking forever"*. The 53C96 pseudo-DMA path did not get
the same escape.

**The reason this is invisible is the sharpest instance of the orphan gap:**

| Target | In `ALL_TBS`? | Scenarios compiled | Result |
|---|---|---|---|
| `tb-scsi-sd-e2e` | **yes** | 11 | PASS |
| `tb-scsi-sd-e2e-c96` | no | 18 | **2 FAIL** |
| `tb-scsi-sd-e2e-c96-core100` | no | 18 | **2 FAIL** |

Same `tb/tb_scsi_sd_e2e.cpp`; the c96 variants add `+define+SCSI_E2E_C96`
(`Makefile:3809`, `:3818`). The gated target does not merely skip the c96
*configuration* — it **does not compile scenarios c12–c18 at all**.

The Quadra 700 ships the 53C96. **The gated SCSI test covers the front end the
machine does not use, and the one it does use is untested.** On hardware this
means an SD write error hangs the SCSI bus permanently instead of reporting a
check condition.

**Action:** (a) add both c96 targets to `ALL_TBS`; (b) give `S_DATA_OUT` the
same `vh_done`/`vh_error` escape `c44aeff` gave the other path.

---

## 1.4 — The floppy interrupt is hardwired off, under a comment that says it was just fixed

**Value: medium-high · Risk: none for the comment**

`rtl/mac/iwm_stub.v:83` — `assign irq = 1'b0;`

`rtl/soc/fpga_top_peripherals.vh:1462` — `.ca2_in(~iwm_irq_w)`, under a ~20-line
comment at `:1455` explaining that *"`iwm_irq_w` was already being generated by
u_iwm and wired to a … it was simply CONSUMED NOWHERE, so the floppy interrupt
was produced and then discarded"*, and that *"the Q700 ROM programs VIA2
PCR = 0x22 … so it genuinely expects an edge on this pin."*

`iwm_irq_w`'s only driver is that constant `1'b0` (`:2219`). So `~iwm_irq_w` is
constant `1'b1` — **exactly the tie-off the comment says was the bug**. Vivado
folds it; VIA2 CA2 can never take an edge.

The comment even names the bug class it is now an instance of (*"the same
wrong-comment-over-a-dead-interrupt-input pattern that hid the VIA2 CA1
tie-off"*). A reader will believe the floppy hint line is live. It is not.

Same instantiation: `iwm_dma_req_w` (`:2220`) and `iwm_mode_w` (`:2221`) are
driven and consumed nowhere; `dma_req` is also `1'b0` (`iwm_stub.v:84`).

**Action:** fix the comment now (free). Making the IRQ real is a *feature*, and
should be scoped as one.

---

## 1.5 — Three testbenches have been red for weeks-to-months; all three are stale tests, and the RTL is right

**Value: high (removes noise + restores three real gates) · Risk: very low** —
these are test-side fixes.

| Target | Result | Red since | Root cause |
|---|---|---|---|
| `tb-dafb` | 1807 pass / **10 FAIL** | 2026-07-26 | stale test |
| `tb-dafb-via-irq` | 16 pass / **6 FAIL** | 2026-07-26 | same root cause |
| `tb-pic16c5x` | 11/12 · **1 FAIL** | 2026-05-19 | stale test |

**DAFB (both targets — one root cause, not two).** Every failure reduces to
"the IRQ status register at `+0x20` reads `0x00000000`". Commit `0bc53b3`
(2026-07-26), *"fix(dafb): assert slot IRQ from int_status (VBL|cursor), not
VBL+fake enable"*, deliberately **retired an invented DAFB IRQ-enable register
at `+0x1C`** because MAME has no such thing, and made VBL arming depend on
`SWATCH_CTRL` (`+0x104`) bit 0 (`rtl/mac/video.v:923`). The commit did not touch
the testbenches. `tb/tb_dafb.cpp:494-525` still writes `+0x1C` as *"local IRQ
enable"*, and neither tb ever writes `+0x104` bit 0 — so `vblank_pending` never
sets and `+0x20` correctly reads 0.

Note the RTL still carries `REG_IRQ_ENABLE` (`video.v:298`) and still honours
writes to it (`video.v:1006`) — a register kept alive only for these dead tests.

**pic16c5x.** Scenario 12 sets `TRIS=0xff` and expects a port read to return the
pin value. `rtl/mac/pic16c5x.v:100` implements `direct_read = (portb_in &
portb_latch)` — the **open-drain PIC1650/1654S** model, per the comment at
`:88-98` and commit `7153fee` (2026-05-19, *"ADB PIC1654S open-drain I/O …
MAME parity"*), which states outright that *"trisa/trisb … are unused on this
model (the ADB firmware never executes TRIS)"*. The test asserts generic
PIC16C54 TRIS semantics this model deliberately does not implement.

**Action:** rewrite the three stale scenarios against current semantics, then
**add all three targets to `ALL_TBS`**. Consider deleting `REG_IRQ_ENABLE` from
`video.v` once `tb_dafb.cpp` stops writing it.

---

## 1.6 — Three VIO probe widths disagree between the sim stub, the IP config and the RTL — and the lint that would catch it suppresses the warning

**Value: high · Risk: low-moderate**

| Signal | RTL width | IP config | Sim stub |
|---|---|---|---|
| `vio_scsi_sd` → `probe_in22` | **187** (`fpga_top_debug_vio.vh:306`) | **187** (`vivado.tcl:742`) | **76** (`verilator_xilinx_stubs.v:168`) |
| `vram_rd_addr` → `probe_in3` | **22** (`fpga_top_video.vh:122`, `FB_ADDR_W`) | **20** (`vivado.tcl:723`) | — |
| `vio_boot_ctrl` → `probe_out0` | **5** (`fpga_top_clocks.vh:384`) | **5** | **4** (`stubs.v:175`) |

Consequences: `probe_in3` **silently drops the top 2 address bits**, so
`vio-read vram_rd_addr` reports a wrapped address — and the comment right above
`FB_ADDR_W` warns that under-widening "silently truncates/aliases any DAFB base
register programmed above the representable range", which is precisely what the
probe now does. In any `VIO_ENABLE` lint or sim, `vio_boot_ctrl[4]` — **the PRAM
zap bit** — is undriven (and `-Wno-UNDRIVEN` hides that too).

This is *exactly the bug class* `synth/vivado.tcl:675-682` was written to
prevent: *"vio_scsi_sd came back 76 bits after being widened to 187. That cost a
full ~2 h build."* The stub was never updated, and `lint-configs` cannot fail on
it because the matrix passes `-Wno-WIDTHTRUNC` (`Makefile:1804-1805`).
`make lint-configs` passes 8/8 today.

**Action:** fix the three widths; add a `lint-vio-widths` target (or a small
Python check) that cross-reads `C_PROBE_IN*_WIDTH` against the stub and the RTL
bus widths **with WIDTHTRUNC enabled**. Cheaper than dropping the suppression
repo-wide (see §4.3).

**Cache trap to know about:** `gen_debug_vio_ip`'s marker check
(`vivado.tcl:683-704`) hardcodes width literals. Changing any probe width
requires bumping `probe_map=v23`, `probe_count=`, the per-probe literals in
**both** the check and the write, *and* the marker filename — or the stale IP is
silently reused.

---

# TIER 2 — complexity that can be deleted

## 2.1 — `rtl/mac/glue.v` is dead, has drifted, AND has a gated test giving false confidence in the address map

**Value: high · Risk: low**

`glue.v` (301 lines) has **zero instantiations** anywhere in `rtl/` or `tb/`
(`grep -rnE "^[[:space:]]*glue[[:space:]]*(#\(|u_|…)"` → 0 hits). The real
decode lives in `rtl/soc/axi_xbar.v` (RAM/ROM/alias) and
`rtl/soc/peripheral_bus.v` (per-slave chip selects).

It has already **drifted from the design it purports to describe**:

- no `ADBINJ` slot (cf. `peripheral_bus.v:312 SLOT_ADBINJ`);
- it asserts `fault` (bus error) on unmapped I/O, where the shipped design
  returns a quiet OKAY/open-bus (`peripheral_bus.v:311-318 SLOT_FAULT` → VOID) —
  the MAME-canonical behaviour;
- its header (`glue.v:60`) points at `rtl/sys/peripheral_bus.v`, a path that
  stopped existing at the June `ab1d2c5` refactor;
- all 5 parameters (`glue.v:90-94`) are never overridden.

**The worst part is not the dead code.** `tb-glue` **is in `ALL_TBS`**
(`Makefile:6692`). The gated suite spends time proving a decoder that is not in
the design, and that disagrees with the real one, is "correct". That is
*negative* value: a green row that reads as address-map coverage.

CLAUDE.md still lists it as *"GLUE / address decoder: `rtl/mac/glue.v`
(upgrade from stub is task #86)"*.

**Action:** delete `glue.v`, `tb/tb_glue.cpp`, the `tb-glue` target and its
`ALL_TBS` entry; drop the CLAUDE.md line. Check the ~10 docs referencing
`glue.v` first (`docs/mame_integration.md`, `docs/gameplan.md`, …) — none should
be treating it as the map spec.

## 2.2 — The provisioning bitstream (~1,160 lines) is dead, and it is the stated justification for another dead path

**Value: medium-high · Risk: low**

- `rtl/soc/sd_provision_top.v` (309 lines): **no instantiation in `rtl/`, `tb/`
  or `synth/`**. Not in `vivado.tcl`.
- `rtl/soc/sd_provision_core.v` (365 lines): instantiated only by the dead
  `sd_provision_top.v:251` and by `tb/tb_sd_provision_top.v:55`.
- `rtl/board/sd_bulk_writer.v` (486 lines): instantiated only by
  `sd_provision_core.v:228` → testbench-only.

`synth/vivado.tcl:543` already says *"sd_provision.v was removed — boot is via
JTAG-AXI now"*, and `fpga_top_sd.vh:5-8` agrees. Three files survived.

**Knock-on:** `sd_ctrl.v:178-181` justifies keeping the CMD25 multi-block-write
machinery alive because *"The staged and verified provisioning path retains
CMD25 throughput"* — naming a path that is in no bitstream. With
`MULTI_WRITE_AS_CMD24(1)` at the only shipped instance (`fpga_top_sd.vh:516`),
the whole CMD25 path (`SC_CMD25` at `sd_ctrl.v:1052`, the stop-tran/`S_AB_*`
close, ~91 lines of `S_W_*`) is **dead in the shipped build**.

**Action:** delete `sd_provision_top.v` (free — nothing references it), then
decide deliberately on `sd_provision_core`/`sd_bulk_writer` and the CMD25 path.
At ~86.8% LUT utilisation this is worth doing.

## 2.3 — A comment asserts a deletion that never happened

**Value: medium · Risk: none**

`rtl/soc/fpga_top_video.vh:327-332` states:

> The standalone `rtl/soc/vram_cpu_byteswap.v` shim module (and its sole
> consumer, `tb/tb_framebuffer_pixel`, which reproduced the same BE→LE
> permutation outside the xbar) **were removed as dead production code** — the
> swap lives only in axi_xbar.v S3 now (build-truth-hygiene cleanup).

Neither was removed. `rtl/soc/vram_cpu_byteswap.v` (190 lines) still exists,
`tb/tb_framebuffer_pixel.{cpp,v}` still exist, `tb-framebuffer-pixel` is still a
target (and **passes**), and the module is still in the `lint-fpga-top` /
`tb-fpga-top-rom` source list. It is *not* in `synth/vivado.tcl` and is
instantiated by no RTL — so the "dead" half of the claim is true; only the
"removed" half is false.

**Action:** either finish the deletion or correct the comment. Do not leave a
comment describing a state of the tree that is not the tree.

## 2.4 — `asc.v`: the EASC version is a `localparam` constant 0, and four tb scenarios pass by skipping

**Value: high for the false-green tests · Risk: none for the comment fixes**

`asc.v:299` — `localparam [8-1:0] VERSION_USE = VERSION_EASC;` (= `8'h00`,
`:278`) ⇒ `asc.v:300` `IS_EASC = 0` **always**. Every `IS_EASC` branch is
unreachable: `:541, :587, :590-591, :632, :714, :764, :1039-1040, :1086, :1351`.

Because `VERSION_USE` is a **localparam, not a parameter**, it cannot be flipped
with `-G`. Therefore `asc.v:294-297` — *"is in place and exercised by tb-asc
scenarios 40-43"* — is **false**. Those scenarios gate on a runtime
`bus_read(0x800) == 0xB0` (`tb_asc.cpp:202-204`) and **soft-skip with PASS**
(`tb_asc.cpp:1792, 1832, 1854, 1900`: `"[skip] non-EASC build"`). Four green
tests that execute nothing.

It also contradicts `asc.v:19-20` and **CLAUDE.md** (*"ASC (EASC variant —
343S1036, version `0xB0`)"*). `R_VERSION` actually reads back `0x00`.

**Honest caveat:** `asc.v:282-298` documents the rollback in detail (flipping it
regressed to MacsBug entry). The *state* is deliberate and is not re-litigated
here. What is wrong is the claimed coverage and the claimed version.

**Action:** fix `:294-297`, `:19-20` and the CLAUDE.md line. Promoting
`VERSION_USE` to a real `parameter` is the cheap way to make scenarios 40-43
mean something.

## 2.5 — Dead flexibility: parameters that are one value everywhere

**Value: low-medium each · Risk: none**

Never overridden at any site: `adb_phy.CLK_MHZ` (`adb_phy.v:120`, explicitly
sunk at `:653` — it *looks* like retiming the PHY is a `-G` away; it is not);
`scc.LOG_PROBES` (`scc.v:79` — `:922` unreachable in every build);
`rtc.SEC_DIV_FAST_RATIO` (`rtc.v:80`); `q700_eth_sonic.MAC0..MAC5` (`:23-28`, 6
params); `adb_keyboard.DEFAULT_ADDR/DEFAULT_HANDLER/FIFO_LOG2` (`:51-53`);
`adb_mouse.DEFAULT_ADDR/DEFAULT_HANDLER` (`:26-27`); `adb_modem.NUM_DEVICES`
(`:58`); `boot_fsm.START_SECTOR` / `CARD_LBAS_DEFAULT` / `CARD_LBAS_MIN`;
`pram_sd.SYS_PERSIST_BASE` (`pram_sd.v:164`, "documentation only", consumed only
by a lint sink at `:906`).

Genuinely two-valued — **keep**: `adb_modem.NO_SERVICE_DELAY`, `scc.PCLK_DIV`
(see 1.1), `rtc.SEC_DIV`, `pic16c5x.PROGHEX`, `via1.ENABLE_INTERNAL_VBL`
(generate-gated, documented `via1.v:203-218`, zero synth cost).

## 2.6 — Registers written but never read (confirmed by Vivado `[Synth 8-6014]`)

`via1.v:287-289` `adb_cb1_idle_count` / `adb_cb1_idle_pulse_state` /
`adb_cb2_idle_pulse_state` plus orphan localparams `:293-300` — delete together
with the stale comment in 3.3. `iwm_stub.v:66-67` `data_reg`/`mark_reg` (16 FFs,
pure dead stores). `asc.v:473-476` eight EASC pointer shadows (64 FFs) and
`asc.v:393` `fifo_ctl` (8 FFs) — both documented. `pic16c5x.v:48` `wdt_counter`
counts but never times out (the PIC watchdog is not modelled — fine for the ADB
firmware; worth one line of comment).

`scc.v:154-155` `wra[0,6,7,8,10]`/`wrb[…]` are **deliberate** — `live_wr_slot()`
(`:162-193`) is well documented. One nit: the comment at `:150-152` says the
gating prevents Vivado dead-code elimination, but `vivado.log` still emits
8-6014 for all ten. Behaviour is fine; the stated goal is not achieved.

## 2.7 — `u_adb_modem` in `fpga_top` exists for tb coverage that does not need it

`fpga_top_peripherals.vh:1292`, documented `:1240-1263` as existing *"purely so
tb/tb_adb.cpp can exercise adb_modem.v"*. But `tb/tb_adb.v:117` instantiates
`adb_modem` **standalone** — the fpga_top copy contributes nothing to that
coverage, and synth already strips most of it (`vivado.log:1737-1738` removes
`cmd_byte_reg`, `reply_b0_reg`). Not fully strippable: `via_rx_byte`/
`via_rx_valid` feed `via1` (`:1211-1212`).

Also note the block comment at `:1252-1254` claims `adb_rx_valid` is
*"hardcoded 1'b0"* — already undone by the fix at `:1203-1212`. **Stale.**

## 2.8 — `debug_vio`: ~115 of 903 probe bits are constant, superseded or duplicated, plus activity detection nobody reads

**Value: ~1,000-1,300 LUT and ~2,400 FF · Risk: low to moderate**

Measured: `u_dbg_vio` = **2,683 LUT / 4,834 FF** for **903** probe input bits
(`vivado.tcl:720-745`) → ≈2.83 LUT and ≈5.06 FF per bit.

- **Two 32-bit probes are hardwired to zero.**
  `fpga_top_debug_ctrl.vh:46-47` — `wire [31:0] dbg_pc = 32'd0;` and
  `wire [31:0] dbg_committed = 32'd0;`, *"Tied off here only so the VIO dashboard
  probes … stay bound."* Consumed at `fpga_top_debug_vio.vh:457`/`:461`.
  **64 bits ≈ 180 LUT / 325 FF spent displaying zero** — and worse than dead:
  `vio-read` prints `dbg_pc = 00000000`, which reads as "the CPU is at PC 0".
  The live values moved CPU-side behind `dbg_axi` (`OFF_LAST_PC`,
  `OFF_INST_LO/HI`). This is a lying instrument, not a missing one.
- **Four probes are superseded by `probe_in24`.** `video_top.v:550-567`'s
  `dbg_video_snap` already carries hcount/vcount/`sc_de`/`sc_rd_en`/
  `sc_rd_valid`/`sc_rgb` **latched on one pclk edge**, making `probe_in1`,
  `probe_in2`, `probe_in4` and `probe_in11` redundant (49 bits ≈ 140 LUT /
  250 FF). `tools/jtag_repl.tcl:4759-4763` already says reading them separately
  is *"each its own JTAG transaction, seconds apart"* and cannot be correlated.
- **Exact duplicates:** `probe_in0` (`hdmi_mmcm_locked`, also in
  `vio_boot_video[5]`) and `probe_in8` (`s0_wready`, also in `vio_ddr_axi`).
- **`CONFIG.C_EN_PROBE_IN_ACTIVITY {1}`** (`vivado.tcl:750`) — **nothing reads
  probe activity.** `jtag_repl.tcl:4749` and `synth/vio_dashboard.tcl` both read
  `INPUT_VALUE`; grepping `ACTIVITY` across `tools/` + `synth/` returns only this
  one line. Activity detection costs roughly a previous-value register + sticky
  register + compare *per bit* — consistent with the observed 5.06 FF/bit (a
  plain probe is ~2-3). Plausibly **700-1,000 LUT / ~1,800 FF**.

**Risk note:** the activity bits do drive the Vivado HW Manager GUI's per-bit
indicators, so a human doing GUI bring-up loses that. Everything else is low
risk. Worth weighing against the design sitting at **87% LUT / congestion level
6**, where `fpga_top_peripherals.vh:1830-1834` records that a **33-LUT** netlist
delta once flipped `route_design` from clean to 346 residual node overlaps.

## 2.9 — `debug_ctrl` (CPU-side): PC trace ring is 4× oversized, and 10 shadow registers have no writer

**Down-ranked: this is v2-core territory** (`cpu/rtl/core/debug/debug_ctrl.v`),
recorded so the v2 work does not re-inherit it.

Good news first: the PC trace ring is **not** the area problem — the dedicated
read register at `debug_ctrl.v:1389-1392` worked, and the 256×32 ring inferred
as a single **RAMB18**, not the 1-3k LUT of distributed RAM its own comment
feared. The 4,567 logic LUTs are the AXI-Lite decode and read mux over **128
register offsets** in a 20-bit space.

- **`PC_TRACE_DEPTH` is 256** (`m68k_axi_wrapper.v:926`). Every consumer in
  `tools/` asks for far less — the full set of `pc-trace <N>` values is
  24/24/24/32/32/32/40/40/40/48/48/64/64. **Max ever requested is 64.** Dropping
  to 64 frees the RAMB18. *(`jtag_repl.tcl:786-787` claims the override is
  "currently 64" — it is 256. Behaviour is fine; it reads `OFF_CAP_TRACE` at
  runtime.)*
- **10 host-shadow arch registers no tool ever writes:** `OFF_ARCH_CACR`, `_TC`,
  `_ITT0/1`, `_DTT0/1`, `_URP`, `_SRP`, `_SFC`, `_DFC` (`debug_ctrl.v:631-644,
  680-684`) ≈ **320 FF** plus read-mux and apply-path arms. `tools/gdbdbg.py`
  uses the D0/A0/USP/SSP/ISP/SR/VBR/PC shadows, but for MMU state it exclusively
  uses the `OFF_LIVE_MMU_*` readbacks (`gdbdbg.py:682-685`, `gdbstub.py:720`).
  The shadow MMU half is write-only storage with no writer.
- **`OFF_ARCH_*` (host shadow) vs `OFF_LIVE_*` (live CPU) is a real trap** —
  `jtag_repl.tcl:801` has to annotate `set OFF_ARCH_D0 0x2000 ;# host-write
  shadow (NOT live CPU state)` because the split is not obvious from the map.
  Document it once in `docs/hw_debug.md`.

---

# TIER 3 — invariants documented but not enforced (and the reverse)

## 3.1 — The shipping SD/SCSI volume runs with its bounded-response watchdog **disabled**, as a two-week-old experiment

**Value: high (a decision that should be re-affirmed or reverted, not left to rot)**

`rtl/soc/fpga_top_sd.vh:515` — `.REQ_WDOG_ENABLE(0)` on `u_sd_ctrl_scsi`.

`sd_ctrl.v:108-122` states the contract about as strongly as a comment can:

> "REQ_WDOG_* below is the backstop … It bounds every state at once, including
> states added later. **Do not gate it**, and do not replace it with per-state
> timeouts."

and `:164-172`: *"DISABLING IS NOT FREE … With it off, such a request parks
forever and the caller wedges rather than failing. Only set 0 deliberately, to
test whether a spurious expiry is killing a healthy-but-slow request."*

The rationale block at `fpga_top_sd.vh:495-512` confirms this was a
**2026-08-03 user-directed diagnostic**. It has been in the shipping config for
over two weeks, and its knock-on effects are already visible:
`fpga_top_sd.vh:304-311` had to be rewritten because an older comment claimed a
pram-grant stall *"fell out on sd_ctrl's own global request watchdog (~10 s)"* —
now false.

This compounds 1.3: the SCSI DATA_OUT wedge has **no watchdog behind it**.

**Action:** decide. Revert, or write down that it stays and why. Trivial either way.

## 3.2 — Header table in `peripheral_bus.v` contradicted its own decoder — **FIXED in this sweep**

`peripheral_bus.v:21-25` advertised `0x0F1_E000..0x0F1_FFFF` as an *"SCC
alt-base"* that *"lands on SCC rather than aliasing through to SWIM"*. The
decoder says the exact opposite at `:410-449`: *"NO SCC ALTERNATE BASE. The
whole `0x50F1_Exxx` window belongs to SWIM/IWM … the alias is deleted rather
than narrowed again"* (task #246, settled 2026-08-05).

Anyone reading the file's stated quick-reference map was told `0x50F1_E020`
reaches SCC. It reaches SWIM/IWM. **Corrected.** (comment-only)

## 3.3 — `via1.v` ADB comment describes a world that no longer exists

`via1.v:565-572` claims *"adb_cb1_line / adb_cb2_line are left at their reset
value of `1'b1` (idle high). No edge → no IFR set → no spurious IRQ. Replace
with a real ADB transceiver when ADB lands."*

Contradicted below it: `via1.v:676-679` samples `adb_cb1_line <= cb1_in`
every cycle from `u_adb_pic_modem`; `:337-340` resets them to `cb1_in`/`cb2_in`,
not `1'b1`; `:439,446` **do** write `adb_cb2_idle_pulse_state`. ADB landed.

This tells a reader the CB1/CB2 IFR bits cannot fire — the interrupt path the
entire ADB stack depends on. **Action:** fix the comment (free).

## 3.4 — Stale comments in the boot/SD path

- `boot_fsm.v:1521-1526` says the zero pass writes *"single-beat 4-byte zeros
  (AWLEN=0) … Single-beat is required because the narrow→wide AXI adapter does
  not support multi-beat bursts."* **Both halves false**: `boot_fsm.v:463` is
  `ZERO_AWLEN = 4'd15`, and `axi_narrow_to_wide.v:16-24` gained INCR-burst
  support in task #170. The trailing `// 8'd0 = 1 beat` at `:1535` is stale too.
  *(The burst geometry itself is correct — verified 64 B aligned, never crosses
  4 KiB, exact end condition at `:1591`.)*
- `fpga_top_boot_master.vh:90-94` recommends an action already taken (*"If that
  becomes painful, raise ZERO_AWLEN to use 16-beat bursts"*), three lines above a
  newer, accurate `MEASURED 2026-08-19` block. Two contradictory comments about
  the same thing in one file.
- `boot_fsm.v:439-440` *"~10M cycles for 4 MiB … (~100 ms)"* — superseded by the
  measured 4.06 s for 256 MiB at `:227-232`.
- `sd_spi_mux.v:7-8, 19-22, 73, 114-115` — names `sd_provision` (removed) as a
  live phase-B owner; describes `b_sel` as a real selection (it is `1'b1`
  constant, `fpga_top_sd.vh:98`); describes a two-bit one-hot that is now four.
- `peripheral_bus.v:1719` says "16 KB mapped window"; the ASC decode window is
  8 KB (`:479-480`).
- `pram_sd.v:159-161` claims four modules "all agree on" `SD_RESERVED_LBAS =
  8192`. One of the four is `sd_image_lba_map` (`sd_ctrl.v:1878`) — a module
  with **zero instantiations**.

## 3.5 — Behaviour enforced in RTL that no document mentions

**The DAFB VBL arm gate.** `rtl/mac/video.v:923` gates `vblank_pending` on
`regs[REG_SWATCH_CTRL][0]` (`+0x104` bit 0). If that bit is never set, the DAFB
VBL interrupt is **dead** — and `video.v:784-800` spells out why that is
catastrophic here: *"the $0160 bit-6 VBL guard gates ALL deferred tasks, so a
dead DAFB VBL freezes the cursor and starves level-2 slot dispatch."*

`docs/vbl_irq.md` is the document a reader would consult, and it **does not
mention `+0x104` or `SWATCH_CTRL` at all** (grep count: 0). It describes the
chain as unconditional. Given the recorded boot frontier is "event loop dead",
this is the single most important undocumented invariant found.

**Action:** add the arm gate to `docs/vbl_irq.md`. *(Its five stale RTL paths
were corrected in this sweep — see 4.4.)*

## 3.6 — An elaboration-time safety check that three files swear by **does not exist**

**Value: high · Risk: none to fix the docs; low to implement the check**

`cpu/rtl/core/decode/uop_pkg.v:106-111`:

> `rat.v` (int) and `fp_rat.v` (FP) each carry a **HARD ELABORATION-TIME CHECK**
> — see the `g_prf_size_check` generate blocks — that fails the build outright
> if `PREG_*_W` is not exactly ceil(log2(`PHYS_*_REGS`)) … **Do not remove
> those; they are the only thing standing between a mis-set knob and a silent
> truncation bug.**

Repeated as established fact in `Makefile:1707-1709` and `synth/vivado.tcl:1112-1115`
("fails synthesis outright on a mismatch").

Measured: `grep -rn 'g_prf_size_check' cpu/` returns **only the comment itself**.
`grep -rn '\$error\|\$fatal' cpu/rtl/core/rename/` returns **nothing**. There is
no such generate block anywhere.

Latent rather than active today — Vivado derives `PREG_INT_W` correctly at
`vivado.tcl:1118-1121` — but `make impl CPU=m68k PRF_INT=<n>` with an
inconsistent width would silently truncate every physical-register tag through
ROB / iq_int / iq_mem / iq_fp / RAT / all four CDBs, which is precisely the
failure mode the same comment describes as *"data landing in the wrong register,
not a build break"*.

**Action:** implement the check or delete all three claims that it exists. A
guardrail nobody has verified is worse than a known absence.

*(This one is CPU-side and therefore v2-core-relevant — but the two **claims**
live in this repo's `Makefile` and `synth/vivado.tcl`, so they are ours to fix
either way.)*

## 3.7 — `make lint` cannot fire *any* elaboration-time guard in the repo

**Value: high · Risk: none**

Verified empirically with a minimal reproduction — a `generate if` containing
`initial $error(...)` under `verilator --lint-only --cc -Wall --top-module t`
exits **0 with no diagnostic**. That is exactly the shape of `make lint`.

So all three `boot_fsm.v` guards (`:248` NUM_SECTORS non-zero, `:251` fits the
4 MiB window, `:254` fits the 16-bit sector path) are **invisible to the only
gate a contributor actually runs**. They fire at sim runtime only, and only if a
testbench happens to elaborate that configuration.

Combine with 3.6: the repo's two guardrail mechanisms are "a check that does not
exist" and "checks the gate cannot see".

## 3.8 — Cross-clock watchdog nesting: four numbers in three files, coupled by a clock ratio, checked nowhere

`rtl/soc/axi_xbar.v:349-358`:

> S1 carries its OWN bounded ack watchdog (`PB_WATCHDOG_LOG2 = 24` @ 50 MHz
> pb_clk ≈ 335 ms) … The xbar's OUTER timeout for S1 **must therefore exceed**
> that INNER guarantee or a legitimately slow disk access would be SLVERR'd and
> S1 poisoned mid-transfer.

The invariant is `2^WD_LOG2_S1 / CORE_CLK_HZ > 2^PB_WATCHDOG_LOG2 / PB_CLK_HZ`,
spanning `axi_xbar.v:358` (`WD_LOG2_S1 = 27`, not overridden),
`peripheral_bus.v:124` (`PB_WATCHDOG_LOG2 = 24`, hardcoded *again* at
`fpga_top_peripherals.vh:324`), and two clock generics (`fpga_top.v:366`, `:371`).

Four independently-editable numbers, zero elaboration check, zero test. The RTL
comment reasons at 100 MHz (4× margin); `fpga_top.v:366` declares the default as
200 MHz (2× margin) — **the margin already varies by build without anyone
computing it.** Failure mode: a poisoned I/O slave mid-disk-transfer, i.e. an
intermittent boot hang.

## 3.9 — Constants duplicated across files with an assertion that they agree

- **`Q700_IO_MIRROR_MASK = 24'hFC0000`** is declared independently at
  `glue.v:344` and `peripheral_bus.v:345` — no shared define. The two decoders
  use *different structures* (glue: parallel one-hot; peripheral_bus: priority
  chain), and `glue.v:64` asserts they match. **They already diverge:**
  `peripheral_bus` serves `SLOT_ADBINJ` at `0x001_1000` (`:311`), which glue
  classifies as `io_unmapped` → `fault`. Harmless only because glue is dead
  (§2.1) — which makes its header claim doubly untrue.
- **`SD_RESERVED_LBAS = 8192`** exists in `boot_fsm.v:245`,
  `sd_scsi_lba_mapper.v:18` and `vhdd_sd.v:56`. `vhdd_sd.v:33` boasts of feeding
  the mapper "from one localparam so the two cannot [diverge]" — true for those
  two, but `boot_fsm` is outside that scope and guards `NUM_SECTORS` against its
  own private copy. Lowering the disk-side value silently overlaps SCSI LBA 0
  with the ROM window while boot_fsm's guard stays green. *(And per §3.4, a
  fourth copy lives in the dead `sd_image_lba_map`.)*

## 3.10 — Load-bearing RTL behaviour that no document mentions

Beyond the DAFB VBL gate (§3.5):

- **`core_rst_bank[]` / `soc_full_rst_bank[]` are pure fanout replication.**
  `rtl/board/clk_rst.v:200-201` assigns `{BANK_W{...}}` — **every bit is the same
  signal**. But consumers select specific indices as if they meant something
  (measured: `soc_full_rst_bank[5]`×15, `[0]`×12, `core_rst_bank[4]`×11, …), with
  region names only in scattered comments. **No document lists the index→region
  map.** Two invisible failure modes: a mis-indexed consumer is sim-identical and
  shows up only as a post-route timing regression; and collapsing the bus to a
  scalar is sim-neutral while destroying the timing intent.
- **Cold-vs-warm boot is inferred from *which reset tree* fired.**
  `fpga_top_boot_master.vh:118-139` — `boot_warm_q` is cleared by
  `core_rst_bank[6]` and set by `jtag_debug_full_reset_eff`, exploiting that
  `clk_rst.v` drives `core_rst <= rst_req` (cold only) but
  `soc_full_rst <= rst_req | dbg_full_rst_in`. **Anyone unifying those two resets
  — an entirely reasonable cleanup, and `docs/reset_story.md` is literally about
  unifying resets — silently makes every reset cold (4 s) or every reset warm
  (boots on stale RAM).** Documented in that comment block and nowhere else.
- **`ZERO_BYTES` is decoupled from every other RAM-size constant.** Four
  independent "RAM size" numbers exist — `AXI_RAM_SIZE` 1 GiB
  (`axi_defs.vh:48`), `RAM_WINDOW_LG2_DFLT` 22 = 4 MiB (`axi_xbar.v:781`),
  `RAM_SIZE_LOG2` 28 (`glue.v:96`), `ZERO_BYTES` 256 MiB
  (`fpga_top_boot_master.vh:142`) — none cross-checked. The undocumented cost is
  measured only in an RTL comment (`:104-108`): **the zero pass *is* the boot
  time**, 4.06 s of it, against 0.13-0.34 s for the SD ROM copy. Shrink the RAM
  window for area and you keep paying 4 s a boot.
- **`PULSE_CYCLES = 128`** (`peripheral_reset_sequencer.v:7`) — a bare magic
  reset-pulse width with no comment on why 128 and no doc mention.

## 3.11 — `boot_fsm` error-cause collision

`boot_fsm.v:1093-1097` — the `default:` arm of `case (cur_cmd)` in `ST_R1_WAIT`
is statically unreachable, **and** its `err_cause <= 3'd4` collides with the
genuinely reachable `word_fifo_full` error at `:1421-1424`. A field
`dbg_err_cause == 4` is therefore ambiguous, despite the port comment at
`:199-201` advertising both as live causes.

---

# TIER 4 — hazards a reader would not expect

## 4.1 — `ALL_TBS` and `TB_KNOWN_BROKEN` do not mean what they look like

`Makefile:6650-6688` documents this honestly, and it is worth repeating because
it is the root cause of most of Tier 1:

- **139** `tb-*` targets are declared; **82** are in `ALL_TBS`. The other **57**
  are built by nothing.
- **Membership of `TB_KNOWN_BROKEN` alone does nothing.** `tb-all` iterates
  `ALL_TBS` and only consults `TB_KNOWN_BROKEN` to decide whether a failure
  gates the exit code. A target in `TB_KNOWN_BROKEN` but not `ALL_TBS` is
  *exactly as invisible as an unlisted orphan*.

**Full orphan sweep results** (this document's measurement — 34 runnable
orphans; the fpga_top-based and MAME-lockstep families were not run):

| Target | Result | Class |
|---|---|---|
| `tb-dafb` | 1807 pass / **10 FAIL** | stale test (1.5) |
| `tb-dafb-via-irq` | 16 pass / **6 FAIL** | stale test, same root cause (1.5) |
| `tb-pic16c5x` | 11/12 · **1 FAIL** | stale test (1.5) |
| `tb-scsi-sd-e2e-c96` | 16 pass / **2 FAIL** | **real RTL bug** (1.3) |
| `tb-scsi-sd-e2e-c96-core100` | 16 pass / **2 FAIL** | **real RTL bug** (1.3) |
| `tb-dafb-mode-matrix-legacy-src` | 1932 pass / **24 FAIL** | invalid test config† |
| `tb-model-rtl-eq`, `tb-model-rtl-consistency`, `tb-hw-checkerboard-path` | exit 2 | retirement tombstones (by design) |
| *the other 25* | PASS | fine — should be gated so they stay that way |

† All 24 are *"mode is 1152x870, scanner would render 1024x768"* — the variant
sets `MM_SRC_W=1024 MM_SRC_H=768` (`Makefile:1588`) then asserts every mode fits
the scanner bound. The 870-line modes cannot. Test-config artifact, not RTL.

**Action:** promote the 25 green orphans + the 3 fixed ones + both c96 targets
into `ALL_TBS`; give the tombstones their own list so they stop reading as
failures; delete or fix `tb-dafb-mode-matrix-legacy-src`.

## 4.2 — `make lint` lints a configuration that has never shipped, and the target that fixes this is run by nothing

**Value: high · Risk: none**

`make lint` → `lint-fpga-top` (`Makefile:1744`), which passes **only**
`-DSIM_MODEL`. But the bitstream defaults are:

| Define | Bitstream default | In `lint-fpga-top`? |
|---|---|---|
| `L2C_ENABLE` | `1` (`Makefile:2266`) | **no** |
| `VRAM_IN_DDR` | `1` (`Makefile:2267`) | **no** |
| `ENABLE_VIO` | `1` (`Makefile:2221`, `vivado.tcl:146`) | **no** |

So the default whole-design lint covers **no configuration that any board build
has ever used**. Concrete consequence: the entire `` `ifdef VIO_ENABLE`` block
in `fpga_top_debug_vio.vh` — every VIO probe — is skipped, and
`vio_hard_reset` (`fpga_top_clocks.vh:270`) appears undriven on the **platform
reset path**, because its only driver is `fpga_top_debug_vio.vh:488`.

`lint-configs` (`Makefile:1786`) exists precisely to lint the eight real define
combinations. **Nothing invokes it** — not `test`, not `tb-all`, not any script
(`grep -rn "lint-configs" Makefile tools/ docs/` → the definition, two comments,
and one doc mention).

**Action:** make `lint-configs` part of the pre-commit / pre-impl routine, or
make `lint` default to the shipping combination.

## 4.3 — The suppressed lint warnings are only **35 sites** — this is a cheap win nobody has priced

`make lint` runs with `-Wno-UNDRIVEN -Wno-PINMISSING -Wno-WIDTHTRUNC`
(`Makefile:1751-1753`). Baseline lint is clean. Re-enabling all three yields:

```
18  %Warning-WIDTHTRUNC
14  %Warning-UNDRIVEN
 3  %Warning-PINMISSING
```

**35 total.** That is a morning's work to clear, after which the suppressions can
come off permanently and the class of bug they hide stops being invisible.
Notably in there:

- **Out-of-range array indexing** (real bug class — an OOB read is `X` in sim and
  whatever synthesis feels like): `boot_fsm.v:1463` *"Bit extraction of
  array[5:0] requires 3 bit index, not 4"*; `sd_ctrl.v:1245` and
  `sd_jtag_writer.v:239` *"array[511:0] requires 9 bit index, not 10"*;
  `i2c_init.v:193`.
- `fpga_top_boot_master.vh:238` missing pin `n_arsize` — **benign** (`n_arvalid`
  is tied `1'b0`, boot_fsm has no read master) but a one-line explicit tie-off
  removes it.
- `fpga_top_sdmin.v:141` missing `zero_en`; `fpga_top_peripherals.vh:476`
  missing `pll_pixel_clock`.
- Two lint paths **disagree on strictness**: `LINT_FLAGS` (`Makefile:1653`, used
  by `make lint MODULE=x`) does *not* suppress `UNDRIVEN`; `lint-fpga-top` does.
  A per-module lint and a whole-design lint give different answers.

**The already-known instance of this class**, worth citing rather than
rediscovering — `fpga_top_dma.vh:198-228`: *"an undriven valid into the xbar is X
in simulation and a silent correctness hazard … **NOTE `make lint` does NOT
catch this: it runs with `-Wno-UNDRIVEN`.**"*

**And the live instance of the same hazard:** the `` `ifdef ENABLE_DDR_RAMDISK``
at `fpga_top_peripherals.vh:2035-2156` has **no `` `else`` branch**, leaving
`vhdd_rd_busy_pb`/`_error_pb`/`_state_pb`/`_wdog_pb` (`:1672-1675`) undriven —
yet consumed at `:1689` and `:1704-1705`, where they cross into a **CPU-visible
CSR readback**. An X source in a status register. The `vhb_*` group
(`:1625-1637`) similarly feeds `u_vhdd_mux`'s B port with no driver (safe today
only because `dev_sel` is provably 0, masked at `:1668-1671`). Trivial fix: add
the `` `else`` tie-offs, exactly as `fpga_top_dma.vh` already does.

## 4.4 — Documentation cites RTL paths that do not exist, at scale

**161** distinct `rtl/**.v` paths are cited across `docs/` + `CLAUDE.md`. **108
do not exist.** Of those, 29 resolve under `cpu/` (the submodule move — a
mechanical `sed`), and **79 are simply gone**.

`CLAUDE.md` is the worst offender, and it is the file every session is told to
read first. Its "Directory Structure" block describes a tree that is not this
repo (`rtl/core/`, `rtl/sys/`, `rtl/mac_top.v` — the real top is
`rtl/soc/fpga_top.v`), and it points at files that do not exist:
`docs/microarch.md`, `docs/mac_compat.md` (both cited repeatedly, including from
the "Mac Hardware Integration" section), `tools/decode_check.py`,
`tb/models/mem_model.cpp`.

Its **build commands are worse than stale — they are wrong**:

| CLAUDE.md says | Reality |
|---|---|
| `make sim` | **retirement tombstone**, exits nonzero (`Makefile:105`) |
| `make decode-check` | **target does not exist** |
| `make test` → "628 PASS / 0 DEFER / 0 FAIL" | `test: tb-all` (`Makefile:117`) reports `PASS/XFAIL/FAIL (of 82)`. The 628 figure is from the retired monorepo suite and **cannot be produced by any command in this repo.** |
| `make fuzz` / `make fuzz-deep` | `fuzz:` depends on `sim` (a tombstone). The `fuzz-deep` gate was retired by the user; CLAUDE.md still documents it as mandatory. |

And CLAUDE.md states **factually wrong hardware behaviour**, which is worse than
a stale path:

| CLAUDE.md | Reality |
|---|---|
| `:160` unmapped RAM returns **`0xFFFFFFFF` + OKAY** "per MAME-canonical" | **`0x00000000` + OKAY** — `axi_xbar.v:3478-3495` (*"open-bus value is uniform 0x00000000 for ALL apertures … matching MAME's default `set_unmap_value = 0`"*), `axi_defs.vh:9-10`. Both cite MAME; the RTL is right. |
| `:168-171` "**2 MB VRAM hardcoded** … not parameterized" | **4 MB** under `VRAM_IN_DDR` (the default) — `axi_defs.vh:124-128`. `fpga_top_video.vh:94-103` calls it *"a deliberate deviation… visible to Mac OS as 4 MB of VRAM, which is not what Q700 silicon has"*. |
| `:174` "GLUE / address decoder: `rtl/mac/glue.v`" | Dead code (§2.1) |
| `:611-618` "Authoritative refs" | **6 of 8 missing** — they live in `cpu/docs/` |

The `0xFFFFFFFF` error is **replicated in `docs/rom_boot_bringup.md` at 7 places**
(`:836, 842, 850, 853, 885, 907, 942`), including the canonical-policy table and
the claim that *"the Q700 ROM SIMM-detect routine reads 0xFFFFFFFF past the
install boundary"*. Anyone debugging RAM sizing greps for the wrong sentinel.

**Other docs a new contributor would trust, and shouldn't:**

- **`docs/memhier.md`** says the L2 cache **does not exist** — `:261-263`
  *"`rtl/**/l2*.v` is a zero-hit grep in both repos"*, `:434` *"Do not create
  `rtl/soc/l2.v` with real cache logic yet"*. There are 11 `l2c*.v` files and it
  ships **enabled by default**. Geometry is wrong too: `:450`/`:479` say 512
  sets / 256 KB / 8-way vs `l2c_defs.vh:25-31` = 8-way / **4096 sets / 2 MB**.
  *(This is a second, larger instance of the VRAM/URAM drift already known.)*
- **`docs/clocking.md`** tells you to set knobs that do not exist or are
  hard-rejected: `HDMI_TEST_PATTERN=1` "first-board default" (no such
  parameter; `fpga_top_video.vh:698` hardwires `.TEST_PATTERN(0)`), and
  `CORE_CLK_DIVIDE=4` + `CORE_CLK_HZ=50_000_000`, which
  `synth/vivado.tcl:318-326` **hard-errors** on for a real-MIG build.
- **`docs/reset_story.md:229`** says the unified reset clobbers RAM
  (*"DDR contents … are clobbered by the pre-zero pass. This is good enough."*).
  **It does the opposite:** `fpga_top_boot_master.vh:124-142` sets
  `boot_zero_en = ~boot_warm_q`, and `boot_warm_q` is *set* by
  `jtag_debug_full_reset_eff` — the canonical unified reset **skips the zero pass
  and boots on the previous session's RAM**. The RTL says so outright: *"a warm
  reset is NOT a valid way to reproduce a cold-boot bug."* This is exactly the
  contamination the doc's own §3 matrix exists to prevent.
- **Stale comments inside the build scripts:** `vivado.tcl:1077`
  *"L2C_ENABLE — DO NOT UNCOMMENT YET"* and `:1089` *"l2c\*.v currently absent
  from this file's file list"* — both false (read at `:533-542`, default on). The
  correction at `:1091-1093` was *appended* rather than replacing the warning, so
  the block now says both things. `axi_xbar.v:375` says the default RAM window is
  *"26 (64 MiB)"*; `axi_xbar.v:781` says `6'd22; // 4 MiB` — self-contradiction
  400 lines apart in one file. `axi_xbar.v:1` and its ASCII topology still say
  **3-master**; there are four (`m3_*` added 2026-08-03, described in the prose at
  `:106-120` but never drawn).

**And the most fragile thing in the repo:** CLAUDE.md **never mentions
`CPU=stub` / `CPU=m68k`**. The highest-cost trap in the project — a bare
`make impl` silently builds a CPU-less bitstream, ~50 minutes to discover — is
documented *only* in `.claude/skills/m68k-build-bitstream/SKILL.md`, which is
**untracked** (`?? .claude/skills/m68k-build-bitstream/`). It is one
`git clean -fdx` from gone.

**Action:** CLAUDE.md needs a pass. The documented session-start baseline check
is currently unrunnable, which means every session silently skips it — and the
one piece of tribal knowledge that saves an hour per mistake isn't in git.

*(Fixed in this sweep: `docs/vbl_irq.md`'s five dead paths —
`rtl/mac/video/video_top.v` → `rtl/board/video_phy/video_top.v`,
`rtl/mac/video/vtg.v` → `rtl/board/video_phy/vtg.v`, `rtl/sys/pulse_cdc.v` →
`rtl/board/pulse_cdc.v`, and two `rtl/fpga_top_*.vh` → `rtl/soc/`.)*

## 4.5 — A new module must be added in **two** places, and only one of them is checked

`docs/scanout_credit_ring.md:194-197` records it:

> A new module must be wired in two places: the Makefile *and*
> `synth/vivado.tcl`'s explicit `read_verilog` list. Only `cpu/rtl/core` is
> globbed there; a module missing from the TCL passes lint, `lint-configs` and
> every tb, then **dies in synthesis** with `[Synth 8-439]`.

This is true, it is a ~50-minute-feedback-loop landmine, and it is written down
in exactly one place that nobody would find. It belongs in `CLAUDE.md` /
`docs/agent_policy.md`.

**Current state (measured):** 7 files are in the fpga_top sim/lint list but not
in `vivado.tcl`. Five are legitimate (`sim_mig_backend.v`,
`verilator_xilinx_stubs.v` are sim-only; `fpga_top_sdmin.v`,
`sd_provision_top.v`, `sd_provision_core.v`, `sd_bulk_writer.v` belong to other
tops — see 2.2). The remaining one is `vram_cpu_byteswap.v` (2.3). **No live
landmine right now** — but nothing prevents the next one.

**Correction — this is mostly already solved, and the doc quoted above is
stale.** `tools/check_synth_sources.py` exists and is a hard prerequisite of
both targets (`Makefile:2551-2561`: `synth: check-cpu-sync check-synth-sources`,
same for `impl`). So the "dies in synthesis 50 minutes later" outcome is now a
2-second failure at the top of the run.

Two residual gaps worth closing:
1. The checker does not cover the `cpu/rtl/core` **glob exclusions** — notably
   the `debug_stop_manager.v` skip at `vivado.tcl:435`, which is exactly the
   hole §1.0 fell through.
2. `docs/scanout_credit_ring.md:194-197` still tells readers the landmine is
   live. Update it to point at the checker.

## 4.6 — Guards that exist, run, and cannot fail

Four targets that look like gates and are not:

- **`make check-cpu-sync` is a no-op in the default configuration.**
  `Makefile:2513-2540` was created after a real incident where *"TWO full
  synth+impl runs silently baked the stale RTL"*. But it is wrapped in
  `ifeq ($(CPU),m68k)` and `CPU ?= stub` — **measured: `make check-cpu-sync` →
  "Nothing to be done", exit 0.** Its non-default arm *also* skips with a
  reassuring message when the sibling checkout is absent (*"…not found, skipping
  (submodule-only checkout)"*, exit 0), so on any CI box or fresh clone the guard
  evaporates. `impl: check-cpu-sync check-synth-sources` (`:2561`) is therefore
  satisfied trivially by a default `make impl`.
  *(Live state: `make check-cpu-sync CPU=m68k` currently **fails** — `cpu/` is
  stale versus the sibling checkout.)*
- **`make timing` cannot fail.** `Makefile:2736-2742` greps the report if
  present and prints "No timing report found" if not. **Measured exit code with
  the current stale report: 0.** It never inspects WNS — so it exits 0 on a
  design that misses timing *and* on a tree that has never been implemented.
  CLAUDE.md:245 presents it as *"Get timing summary (WNS, worst path)"*.
- **`fpga_top.buildinfo` records 15 build knobs and omits the one that matters.**
  `vivado.tcl:2097-2123` writes `part`, `enable_vio`, `l2c_enable`,
  `vram_in_ddr`, `core_clk_hz`, `build_id` … **there is no `cpu=` line.**
  `verify-fpga-debug-artifacts` (`Makefile:2591-2600`) checks `enable_vio=1` but
  cannot check the CPU. So the **stub-bitstream trap is invisible both before the
  build (no guard on `CPU`) and after it (no manifest field)**. Note that
  `l2c_enable` was added after exactly this class of incident — `vivado.tcl:2255-2259`
  explains that before it, *"the only way to tell after the fact was to grep the
  routed design for l2c cells"*. The identical argument applies to the CPU and
  has never been made.
- **`make compile-tests` always exits 0.** `Makefile:120-135` puts `2>/dev/null`
  on assemble/link/objcopy, prints `[SKIP] <name>`, and ends on an `echo`. With
  no m68k toolchain installed, **every test silently skips and the target reports
  success.**

**The counter-example worth copying:** `tools/check_synth_sources.py`, wired into
both `synth` and `impl` (`Makefile:2551-2561`), catches the `find`-vs-`read_verilog`
divergence that bit twice in one day. Its header states the diagnosis for nearly
everything in this section: *"Both times the module was fully verified and fully
invisible. That is the same shape as every other instrument failure on this
project: **a green result that measured nothing**."* The fix pattern exists
in-repo; it has simply never been applied to `CPU=`, to XPASS, to `make timing`,
or to the orphan list. *(It also already covers §4.5 — better than I credited
above; the remaining gap is that it does not cover the `cpu/` glob exclusions.)*

## 4.7 — `TB_KNOWN_BROKEN` never expires and never notices a fix

`tb-all` (`Makefile:6758-6784`) has two structural gaps beyond §4.1:

- **No XPASS detection.** If a known-broken tb starts passing, nothing notices
  and it stays permanently un-gated. Some entries are years-of-drift candidates.
- **No reason matching.** A `TB_KNOWN_BROKEN` entry that begins failing for a
  brand-new reason is still counted `[XFAIL]` and still green.

The list currently holds 8 entries, and per the Makefile's own notes several are
**real defects**, not environment issues: `tb-debug-stop` (*"3/6 …scenarios fail
on their own merits — a real core-debug bug"* — see §1.0, which explains why),
`tb-video` / `tb-video-smoke` (*"real pixel-pipeline mismatches"*).

## 4.8 — Sim and bitstream disagree on the L2 cache, and the one CPU-bearing sim is an orphan

`SIM_L2C_ENABLE ?= 0` (`Makefile:1829`) vs `L2C_ENABLE ?= 1` (`:2266`). The
Makefile states the consequence itself (`:1817-1827`): *"every board build has
shipped a memory path that **no CPU simulation has executed a single instruction
through**. That is the largest known sim/HW difference."*

Compounding it — and this is the part not written down — **`tb-fpga-top-rom`,
the only sim that runs the real CPU through the real SoC, is an orphan** (§4.1).
So no `make test` run touches the L2 with a CPU under *any* setting. Cited, not
re-litigated: this is task #240.

## 4.9 — Other harness hazards

- **`tb_cold_boot.v` does not override `ZERO_BYTES`**, so it zeroes 4 MiB where
  production zeroes 256 MiB. The burst-count depth production actually walks is
  untested by the cold-boot harness.
- **`tb-sd-bridge-pulse`** is a measurement harness whose `main()` ends
  `return 0` unconditionally, and it currently prints **LOSS** for SP=4 while
  exiting 0. Already documented at `Makefile:6673-6680` as the reason it is not
  in `ALL_TBS` — correct call, but it should be given a real exit status.
- **Correctly handled, cite as the good pattern:**
  `tb-axi-ddr4-mig-bridge` (`Makefile:2847-2854`) uses `set -o pipefail` and
  greps for `ASSERTION FAIL` because *"the C++ scenarios do NOT watch for those
  strings, so before this gate an assertion could fire on every cycle and the tb
  would still print 'All N scenarios PASSED'."* Swept the RTL for other
  `$display`-only self-checks: `rtl/board/axi_ddr4_mig_bridge.v` is the **only**
  file with them. Class is contained.

---

# Already resolved — do not re-report

- **`axi_pb_s1_cdc` is no longer unused.** It is now the **default**;
  `axi_async_bridge` is the opt-in behind `` `ifdef PB_S1_WIDE_CDC``
  (`fpga_top_peripherals.vh:206-210`). The file even carries a GREP NOTE at
  `:199-205` explaining why `grep "axi_pb_s1_cdc #("` finds nothing.
- **`scsi_trace_ring.v`** — **re-instantiated 2026-09-09** and now ON by
  default, behind `` `ifdef SCSI_TRACE_ENABLE`` (env `ENABLE_SCSI_TRACE`,
  default 1) in `fpga_top_peripherals.vh`, with `tb-scsi-trace-pb` covering
  the integration through a real `peripheral_bus.v` + `scsi.v` and the real
  `vhdd_ctrl` register map. It was uninstantiated from 2026-08-08 to
  2026-09-09 for routability; if a build fails to route, set
  `ENABLE_SCSI_TRACE=0` rather than editing the RTL.
- **`rtl/soc/axi_pb_lane_shim.v`** is **untracked in-flight work** by another
  agent, not dead code.
- `orwell_stub.v`, `q700_eth_sonic.v`, `adb_pic_modem.v` + `pic16c5x.v` are all
  genuinely in the synthesised path (post-route: `adb_pic_modem` 666 LUT /
  291 FF, `q700_eth_sonic` 303 LUT / 944 FF). `rtl/mac/adb_pic_fw.hex` is real —
  512 lines, 507 non-zero, loaded at `pic16c5x.v:58`, confirmed in
  `vivado.log:1378`. **This contradicts the "repo's ADB ROM is zero-filled" note
  in the project memory index** — for this file at least, that note is wrong.
- No IOSB / djMEMC / Cuda / RBV / Egret / V8 shims exist anywhere in `rtl/mac/`.
  The only cross-chipset residue is the Sonora-vs-EASC split in `asc.v` (2.4).

# Checked and correct — recorded so nobody re-walks them

`sd_scsi_lba_mapper.v:41-44` LBA mapping (no off-by-one; exclusive-end and
34-bit widening both right). Big-endian byte order end-to-end
(`boot_fsm.v:1428` → `axi_narrow_to_wide.v:26-31` → `peripheral_bus.v:689`).
Zero-pass burst geometry (`boot_fsm.v:1591`). `axi_xbar.v:803-877` boot/CPU
write merge (`m0_wsel_q` freezes for the life of a slot-0 write — genuinely good
work). `peripheral_bus.v` decode: no reachable overlaps, full catch-all to
`SLOT_FAULT`, unmapped reads OKAY+0 per MAME, `PB_ACK_TIMEOUT` guarantees every
handshake completes, and the `pb_rd_same_cycle_ok` whitelist (`:1283-1293`) is
accurate. No unreachable states or wrapping counters in `boot_fsm.v`.
`via2`'s dangling outputs (`fpga_top_peripherals.vh:798-801`) are justified —
Q700 VIA2 Port A is input-only. `via1.ENABLE_INTERNAL_VBL` generate gating is
free.

**Considered and rejected:** unifying `via1.v` and `via2.v` (they share ~94
byte-identical lines, ~39% of via2). Both are separately validated against MAME
(`tb-via1-lockstep`, `tb-via2`) and via1 carries Q700-specific ADB/RTC side
channels. This is "not how I'd do it", not "wrong" — and the lockstep harness
would have to be extended to via2 *first*. **Do not attempt opportunistically.**

---

# Things everyone should know about this codebase that are written down nowhere

1. **A green `make test` means 82 of 139 testbenches passed.** The other 57 are
   built by nothing. If your module's tb is not in `ALL_TBS`, it is not tested,
   and it will go red without anyone noticing — for months (§4.1, §1.5).
2. **Being in `TB_KNOWN_BROKEN` does not put a target in the build.** A target
   listed only there is exactly as invisible as an unlisted orphan
   (`Makefile:6665-6669`).
3. **`make lint` lints a configuration that has never shipped** — no L2C, no
   VRAM-in-DDR, no VIO, all three of which default *on* for bitstreams. Use
   `make lint-configs`; nothing runs it for you (§4.2).
4. **`make lint` and `make lint MODULE=x` disagree on strictness.** The
   whole-design path suppresses `UNDRIVEN`; the per-module path does not (§4.3).
5. **A new module must be added to `synth/vivado.tcl` by hand.** Lint, every tb
   and `lint-configs` all pass without it; synthesis dies 50 minutes later with
   `[Synth 8-439]` (§4.5).
6. **`make sim` and `make decode-check` do not work**, and CLAUDE.md's
   session-start baseline ("628 PASS / 0 DEFER / 0 FAIL") cannot be produced by
   any command in this repo (§4.4).
7. **When you correct RTL to match MAME, go find the testbench that asserts the
   old behaviour.** Three separate deliberate corrections (DAFB IRQ model, PIC
   open-drain I/O, SCSI provider-error escape) each left a permanently-red
   orphan tb behind (§1.5, §1.3).
8. **The DAFB VBL interrupt is armed by `SWATCH_CTRL` `+0x104` bit 0**, not by
   any DAFB register. If that bit is clear, VBL is dead, and a dead VBL freezes
   the cursor and starves level-2 slot dispatch (`video.v:784-800`). This is
   absent from `docs/vbl_irq.md` (§3.5).
9. **The shipping SD/SCSI path currently has its watchdog disabled** — a
   two-week-old diagnostic (`fpga_top_sd.vh:515`) against an explicit "do not
   gate it" contract (`sd_ctrl.v:108-122`) (§3.1).
10. **The Q700 ships the 53C96, and the gated SCSI test covers the other front
    end.** `tb-scsi-sd-e2e` compiles 11 scenarios; the ungated c96 variants
    compile 18 (§1.3).
11. **`pb_clk` is fixed at 50 MHz** (`fpga_top.v:371`, hard-errored in
    `vivado.tcl:328-330`). Any peripheral comment that reasons from a 200 MHz
    clock is pre-CDC-era and should be distrusted — `scc.v` still divides for it
    and is 4× off as a result (§1.1).
12. **`debug_ctrl` is CPU-side, `debug_vio` is SoC-side.** With the v2 core
    coming, the 4,635 LUT of `debug_ctrl` is a v2-core question; the 2,683 LUT of
    `debug_vio` is ours and survives.
13. **`debug_stop_manager` exists twice, and the copy that ships is the SoC one.**
    `synth/vivado.tcl:435` excludes the CPU file. Nothing checks that they match,
    and today they don't (§1.0).
14. **The `g_prf_size_check` elaboration guard that `uop_pkg.v`, the `Makefile`
    and `vivado.tcl` all cite does not exist** — and `make lint` could not fire it
    even if it did, because Verilator's lint mode does not evaluate
    `initial $error` in generate blocks (§3.6, §3.7).
15. **Unmapped reads return `0x00000000`, not `0xFFFFFFFF`.** CLAUDE.md and
    `docs/rom_boot_bringup.md` (×7) say the opposite. The RTL is right (§4.4).
16. **VRAM is 4 MB, not 2 MB**, under the default `VRAM_IN_DDR` — a deliberate
    deviation from Q700 silicon, documented at `fpga_top_video.vh:94-103` and
    contradicted by CLAUDE.md (§4.4).
17. **`make check-cpu-sync`, `make timing` and `make compile-tests` all exit 0
    unconditionally in the default configuration**, and `fpga_top.buildinfo`
    has no `cpu=` field — so nothing, before or after a build, can tell you
    whether the bitstream contains a real CPU (§4.6).
18. **The reset test suite is entirely orphaned.** `docs/reset_story.md` is the
    canonical unified-reset document and *every* testbench validating it
    (`tb-clk-rst`, `tb-cpu-rst-stretch`, `tb-dbg-rst-pulse`,
    `tb-debug-full-reset`, `tb-boot-release-gate`, `tb-reset-debounce`,
    `tb-reset-debounce-idle-high`, `tb-peripheral-reset-sequencer`,
    `tb-pram-clear-pulse`) is outside `ALL_TBS`. All of them pass today.
19. **Cold-vs-warm boot is encoded in *which reset tree fired*, not in a control
    bit** — so unifying the two reset trees (the obvious cleanup, and the subject
    of `reset_story.md`) silently makes every boot cold or every boot warm
    (§3.10).
20. **`CLAUDE.md:390`'s "keep modules ≤ 300 lines" is violated by 49 of 96 RTL
    files** (`scsi.v` 5,670 = 19×; `axi_xbar.v` 3,890; `sd_ctrl.v` 1,885;
    `peripheral_bus.v` 1,813; `boot_fsm.v` 1,649), and "no tabs" by 2. Nothing
    checks either. A stated-and-ignored convention teaches contributors that the
    whole conventions block is decorative — either enforce it or delete it.

---

## Suggested order of attack

**Do these first — they are free.** Nothing below the line costs more than an
afternoon, and the first three cost almost nothing:

| # | Item | Value | Risk |
|---|---|---|---|
| 1 | §4.1 add the ~25 green orphans (esp. the 9 reset tbs) to `ALL_TBS` | turns invisible tests into gates; they already pass | **none** |
| 2 | §4.6 add `cpu=` to `buildinfo` + a verify check; make `impl` refuse `CPU=stub` without an opt-out | closes the ~50-min stub trap permanently | **none** |
| 3 | §4.4 commit `.claude/skills/m68k-build-bitstream/` | it is untracked; one `git clean` from gone | **none** |

**Then, by (value / risk):**

| # | Item | Value | Risk |
|---|---|---|---|
| 4 | §1.0 re-sync `debug_stop_manager`, then make the divergence impossible | live halt-wedge / mid-macro-halt bug in the shipped bitstream | low-mod |
| 5 | §1.2 `sd_scsi_bridge` reset glitch | silent data corruption on real HW | low-mod |
| 6 | §1.3 c96 SCSI `S_DATA_OUT` escape + gate both c96 targets | hang-on-error in the *shipping* SCSI path | med |
| 7 | §3.1 decide on `REQ_WDOG_ENABLE(0)` | it is the backstop missing behind #6 | trivial |
| 8 | §1.6 fix the 3 VIO probe widths + a width-check target | wrong probe values; prevents a repeat ~2 h build loss | low-mod |
| 9 | §1.1 SCC `PCLK_DIV` 54 → 14 | 4× wrong on a shipping peripheral | med |
| 10 | §3.2-style: correct `probe_out0` bit-map in `hw_debug.md`, `vio_dashboard.tcl`, `reset_story.md` | today the docs point "scc_uart" at the **PRAM-zap** bit | none |
| 11 | §1.5 fix 3 stale tbs, then gate them | restores three real gates | very low |
| 12 | §4.3 clear the 35 lint sites, drop the suppressions | closes a whole bug class | low |
| 13 | §4.2 wire `lint-configs` into the routine (+ an `ila` row) | lints what actually ships | none |
| 14 | §3.6/§3.7 implement `g_prf_size_check` **or** delete the three claims | a guardrail that is fiction | none / low |
| 15 | §4.4 CLAUDE.md + `rom_boot_bringup.md` + `memhier.md` + `clocking.md` pass | every session is told to read these | none |
| 16 | §2.1 delete `glue.v` + `tb-glue` | removes *false* address-map coverage | low |
| 17 | §2.8 drop the constant/superseded VIO probes; consider `C_EN_PROBE_IN_ACTIVITY {0}` | ~1,000-1,300 LUT at 87% util / congestion 6 | low-mod |
| 18 | §1.4, §3.3, §3.4, §2.3, §2.4 comment sweep | stops actively misleading readers | none |
| 19 | §2.2 delete `sd_provision_*`, then decide on the CMD25 path | ~1,160 lines | low / med |
| 20 | §4.6/§4.7 give `make timing` a real WNS check; add XPASS detection to `tb-all` | two more "green results that measured nothing" | none |

---

## What was changed in this sweep

Comment/doc-only, reversible, nothing structural:

- `rtl/soc/peripheral_bus.v:21-25` — header table said the `0x0F1_Exxx` window
  was an "SCC alt-base"; the decoder deleted that carve-out in task #246. Table
  now matches `decode_slot()` (§3.2).
- `docs/vbl_irq.md` — five RTL paths corrected
  (`rtl/mac/video/video_top.v` → `rtl/board/video_phy/video_top.v`,
  `rtl/mac/video/vtg.v` → `rtl/board/video_phy/vtg.v`,
  `rtl/sys/pulse_cdc.v` → `rtl/board/pulse_cdc.v`, and two
  `rtl/fpga_top_*.vh` → `rtl/soc/`).

Everything else in this document is a **proposal**, not an edit.

## Method / reproducibility

- 34 orphan testbenches were built and run (`make <target>`); results in §4.1.
  The `fpga_top`-based (`tb-fpga-top-rom*`, `tb-cold-boot*`, `tb-rom-boot*`) and
  MAME-lockstep families were **not** run — they are heavy and a bitstream build
  was in progress. **They remain unaudited.**
- Lint counts come from re-running the exact `lint-fpga-top` command with
  `-Wno-UNDRIVEN`, `-Wno-PINMISSING` and `-Wno-WIDTHTRUNC` stripped.
- Area figures are from `build/vivado/reports/utilization_route.rpt`
  (Aug 19 2026, `Design State: Physopt postRoute`, real CPU).
- **No Vivado, synthesis, implementation, board or JTAG command was run.**

---

## Follow-up: `debug_stop_manager` swap ATTEMPTED AND REVERTED (2026-08-19)

The finding is confirmed, but the recommended fix is **not** a drop-in.
Measured:

* Port lists are **identical** (31 each), so it elaborates either way.
* `tb_debug_stop_manager.cpp` against the **inline** copy: **9/9 scenarios
  PASS**.
* The same tb against the **cpu/** copy: only 5 scenarios run and **4
  assertions FAIL** — all of them about `dbg_precise_stop_req`, the
  "unregistered flush overlay" the cpu copy's header describes:
  `break fire precise-stops younger work`, `pended watch hit halts at
  retire boundary`, `halt-after requests precise stop`, `step-macro
  precise DBG break flushes younger work`.

So the two are **not behaviourally interchangeable**; the cpu copy changed
precise-stop semantics and the SoC's testbench encodes the older ones. The
swap was reverted rather than shipped: this is the halt path, i.e. the
mechanism used to diagnose everything else, and an unvalidated change there
is worse than the stale module.

Also noted while testing: the tb prints **"All 5 scenarios PASSED" while 4
assertions had failed** — the summary counts scenarios, not assertions.
`make` catches it via exit status, but a human reading the tail of the log
would not. Worth fixing on its own.

**What a real fix requires**, in order: decide which precise-stop semantics
are correct for the SoC; update `tb_debug_stop_manager.cpp` to the chosen
semantics; verify the CPU's own tests still pass against it; then remove the
duplicate and the two collision guards (`synth/vivado.tcl` and the
`CPU_M68K_SRCS` glob in the Makefile), and retarget the tb's Verilator
recipe, which also names `rtl/soc/fpga_top.v` explicitly.

The underlying defect stands and is worth fixing: every bitstream ships the
198-line manager while the cpu/ copy carries halt-wedge fixes dated
2026-07-28 whose comments describe "a permanent wedge" — plausibly the
wedges `m68k-jtag-wedge-recovery` exists to work around.
