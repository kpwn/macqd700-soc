# reset_story.md — m68k-ooo unified reset audit

**Status:** Phases 1–4 LANDED.
- Phase 1 (audit + open-questions answered): see §7.
- Phase 2 (RTL implementation): `cold_reset_hold` (DBG_CONTROL bit 4) +
  `cold_reset_pulse` (bit 5) plumbed through `debug_ctrl.v` →
  `fpga_top_clocks.vh`; `axi_error_sticky_r` reset moved from `core_rst`
  to `soc_full_rst`; legacy bit 2 (`dbg_soft_rst`) folded into the
  unified-reset path as a deprecation alias for one release.
- Phase 3 (tools): `tools/jtag_repl.tcl` collapsed to `reset` /
  `reset hold` / `reset release` / `reset hold-status` /
  `reset-and-halt-after` / `sweep`; legacy `reset-halt-after`,
  `full-reset`, `full-reset-and-halt` kept as aliases that print a
  deprecation notice for one release.  `tools/jtag_bringup_tui.py`
  stale `force_cpu_rst` label fixed (now `scc_uart_sel_b`).
  `synth/jtag_bringup.tcl debug_reset_halt` and
  `debug_run_from_reset_halt_after` rewritten to use the unified-reset
  bits; `m68k-fpga-halt-bisect/halt_after_n.tcl` updated to use the
  unified reset (CTL_COLD_RESET_HOLD | CTL_COLD_RESET_PULSE) in place
  of the JTAG-VIO bit-3 + race-window pattern.
- Phase 4 (tests + docs): `tb/tb_debug_full_reset.{v,cpp}` extended
  with three new scenarios — `dbg_cold_reset_pulse` drives unified
  reset, `cold_reset_hold` survives the pulse, `cold_reset_hold` alone
  gates `cpu_rst` (no soc_full_rst).  `tb/tb_debug_ctrl.cpp` adds
  `test_unified_reset_bits` covering bit-2/4/5 register semantics.

---

## 1. Existing reset flavors in RTL

### 1.1 Sources

| Source | Form | Drives | Notes |
|---|---|---|---|
| `cpu_resetn` (T19 board pin) | board pushbutton, debounced 1024 cy | `platform_resetn` → `rst_in` of both `clk_rst` instances → `core_rst`, `pb_rst`, `soc_full_rst`, `pb_full_rst` | True cold reset. Asynchronous. |
| `btn[3]` (board pushbutton) | debounced 1024 cy | same as above (ANDed into `platform_resetn`); also drives `fabric_gt_clr` to re-init BUFG_GT divider | True cold reset. |
| `btn[2]` (board pushbutton) | debounced 2_000_000 cy → edge-detected → 1024-cy stretched pulse | `dbg_rst_src_level` → `jtag_debug_full_reset_eff` | "debug full reset" path, OR'd with VIO bit 3. |
| `vio_boot_ctrl[3]` (JTAG VIO) | level-driven probe-out | edge-detected + watchdog → `jtag_debug_full_reset_eff` | The "umbrella" / "warm" reset. |
| `vio_boot_ctrl[0]` ("bypass_sd") | level | `boot_fsm_rst` (forces boot_fsm into reset) and gates `boot_rom_ready` via `(jtag_boot_bypass && jtag_boot_release)` | Skips SD→DDR copy; CPU runs against host-loaded DDR. |
| `vio_boot_ctrl[1]` ("release_cpu") | level | (combined with bit 0) → `boot_rom_ready` | Used after host writes ROM through JTAG-AXI. |
| `vio_boot_ctrl[2]` ("scc_uart_sel_b") | level | UART channel mux | NOT a reset — repurposed; was once `jtag_cpu_hold` (a CPU-only halt that has been subsumed by bit[3]). |
| `debug_ctrl.OFF_CONTROL` bit 1 (CPU halt) | level via JTAG-AXI | `dbg_halt_req` → `debug_stop_manager.dbg_core_halt` | Holds CPU at retire boundary. NOT a reset; just a halt. |
| `debug_ctrl.OFF_CONTROL` bit 2 (soft-rst) | one-cycle pulse via JTAG-AXI | `dbg_soft_rst` → ORed into `cpu_rst` | **Soft CPU reset only.** Resets the CPU pipeline (cache, ROB, RAT, IQ, MMU TLB) but leaves DDR contents, boot FSM, ALL peripherals, AXI xbar, dbg latches untouched. **This is the contamination source.** |
| `dbg_init_done_override` (CONTROL bit 3) | level | `platform_init_done` override | NOT a reset; sim/host gate. |

### 1.2 Synchronised broadcast networks (`clk_rst.v`)

There are TWO `clk_rst` instances (core-clock + pb-clock domains). Each
synthesises FOUR named broadcasts via `xpm_cdc_async_rst` + `BUFG`:

- `core_rst` / `core_rst_bank[7:0]` — board cold reset only (NO debug-full-reset).
  - DDR PHY (`u_ddr`) is on this one — intentionally NOT on `soc_full_rst`
    so debug-full-reset does not pay the ~100 ms MIG re-cal cost.
- `soc_full_rst` / `soc_full_rst_bank[7:0]` — board cold reset OR `dbg_full_rst_in`.
  - Bank slots: [0]=video, [1]=peripheral_bus + mac stub, [2]=DAFB+IRQ resync, [3]=irq_agg+NMI, [4]=AXI xbar+arb, [5]=DMA+SD, [6]=boot_master narrow→wide, [7]=local clocks.vh comb gates.
- `pb_rst` (pb-domain) — board cold reset only.
  - JTAG host-side bridges (`u_dbg_axi_to_axil` etc.) stay on `pb_rst` so
    a debug-full-reset does NOT take the JTAG channel down.
- `pb_full_rst` / `pb_full_rst_bank[3:0]` — board cold reset OR debug-full-reset (CDC sync'd).
  - Bank slots: [0]=via1, [1]=rtc+via2, [2]=scc+scsi, [3]=asc+iwm+sonic+i2s+ext.

### 1.3 Final CPU reset OR (in `fpga_top_clocks.vh`)

```verilog
wire cpu_rst_or = soc_full_rst || dbg_soft_rst || !boot_rom_ready ||
                  !cpu_rst_settle_done;
// + 8-cycle stretcher
wire cpu_rst    = cpu_rst_or || (|cpu_rst_stretch);
```

`cpu_rst` is the actual reset every CPU module sees (`u_cpu`,
`u_debug_stop`, AXI bridge wrappers `cpu_d_*_xbar`, `ifa_*_xbar`).

### 1.4 What survives `dbg_soft_rst` (CPU-only soft reset)

These are the contamination sources today:

| State | Reset by `dbg_soft_rst`? | Reset by `soc_full_rst` / debug-full-reset? |
|---|---|---|
| CPU pipeline (ROB, RAT, IQ, dcache, icache, MMU TLB, register files) | YES (CPU rst stretcher 8 cy) | YES |
| `debug_ctrl` perf counters, `cycle_count`, EXC latches, PC trace ring | NO | YES (via `counters_clear`) |
| `debug_ctrl` host-set control regs (`halt_after_inst`, `break_pc`, `halt_exc_mask`, `ctrl_halt_req`, arch shadows) | NO | NO (intentional — JTAG session survival) |
| `boot_fsm` state | NO | YES (`boot_fsm_rst`) |
| ROM image in DDR | NO (DDR keeps last write) | YES — ROM is re-copied from SD, plus 4 MiB pre-zero pass |
| VIA1 overlay bit (`overlay_bit`) | NO | YES (via1 sees `pb_full_rst_bank[0]`) |
| All other peripherals (VIA2, SCC, SCSI, ASC, IWM, RTC, SONIC, DAFB) | NO | YES (`pb_full_rst_bank[*]` / `soc_full_rst_bank[*]`) |
| AXI xbar in-flight FIFOs | NO | YES |
| DMA controller | NO | YES |
| irq_agg, NMI edge sync | NO | YES |
| `axi_error_sticky_r` (VIO) and `err_s0_aw_addr_r` etc. | NO | YES (via `core_rst` — not soc_full_rst) |
| Video framebuffer URAM | NO | partial (URAM cells are content-addressed; `clear_req=soc_full_rst` triggers wipe FSM) |
| DDR PHY / MIG calibration | NO | NO (intentional — saves 100 ms) |
| Board sys_clk MMCM | NO | NO (intentional — preserves clock tree) |
| HDMI MMCM | NO | YES (via `video_mmcm_resetn` OR'ing `soc_full_rst_bank[0]`) |

**TL;DR**: `dbg_soft_rst` only resets the CPU. Everything else
contaminates. The `m68k-fpga-halt-bisect` skill's warning is correct.

### 1.5 What survives `soc_full_rst` (umbrella reset, today)

- DDR PHY + content (the boot_fsm re-fills DDR from SD via CMD18, ~0.13 s).
- `sys_clk` MMCM tree.
- JTAG host-side AXI bridge (debug bridge stays alive — by design).
- `debug_ctrl` host-set control regs (`halt_after_inst`, `break_pc`,
  `halt_exc_mask`, `ctrl_halt_req`, arch shadows). Intentional — survives
  so the host can stage breakpoints and arm them ACROSS a reset.

That last property is exactly the "hold after reset" semantic the user
asked for, but only partially: today the halt_after / break_pc state
persists, but **`ctrl_halt_req` is cleared at the FIRST CPU reset cycle**
because it lives behind `rst` (= `core_rst`), not `counters_clear`. (See
`debug_ctrl.v:729`.) The CPU comes out of reset *running*; you have to
race-write `ctrl_halt_req=1` between reset deassertion and the first
fetch — the REPL `full-reset-and-halt` does this by writing CONTROL=1
*before* the reset and again *after*, with timing windows.

---

## 2. Existing reset flavors in tools

### 2.1 `tools/jtag_repl.tcl`

| Command | What it does |
|---|---|
| `reset-halt-after <N> [wait_ms]` | Soft-reset only (CPU + halt-after staging). **Subject to the contamination warning.** |
| `full-reset` | VIO bit 3 high 200 ms → low → wait 1500 ms. True umbrella reset. |
| `full-reset-and-halt [N]` | CONTROL=halt_req → VIO bit 3 high 200 ms → low → wait 800 ms → re-arm halt-after → CONTROL=0. Used to land at "halted post-cold-boot". |
| `sweep <wait_ms> N1 N2 ...` | Calls `reset_halt_after` per N. **Same contamination problem.** |
| `vio-set <hex>` | Direct VIO probe write. Power-user. |

### 2.2 `tools/jtag_bringup_tui.py`

| Subcommand | What it does |
|---|---|
| `hold` | (CPU-only halt via debug_ctrl) |
| `release` | (clear halt) |
| `debug-full-reset` | Pulse VIO bit 3 (parameters: `hold_ms`, `release`). |
| `debug-reset-halt` | Soft-reset CPU, halt. |
| `debug-run-from-reset-halt-after` | Soft-reset + halt-after. |
| `debug-sweep-reset-halt-after` | Sweep variant. |

The TUI's status pretty-print at line 488-489 still labels VIO bit 2 as
`force_cpu_rst` — **stale comment**; the RTL (line 311) renamed it to
`scc_uart_sel_b` two refactors ago. The `jtag_cpu_hold` overlay name on
line 145 is also dead.

### 2.3 `m68k-fpga-halt-bisect` skill (`halt_after_n.tcl`)

Implements the canonical "between-iteration umbrella reset" pattern:
halt → VIO=8 → 200 ms → VIO=0 → 800 ms → arm halt-after → release halt.
Manual halt is asserted FIRST so the CPU comes out of reset already
halted.

---

## 3. Missing-coverage matrix (where contamination has been observed)

| Failure mode | Root cause |
|---|---|
| "CPU sometimes runs garbage between iterations of `sweep`" | `reset_halt_after` (soft-only). DDR + overlay + peripherals carry forward. |
| "VBR points at a stale handler from previous run" | Same. Peripheral interrupt latches and VBR/SR shadow can mislead the new boot. |
| "VIA1 overlay already cleared at start of N=1000 iteration" | DDRB[3]=1, ORB[3]=0 from previous run survives soft-reset; xbar serves DDR (not ROM) at low addresses. |
| "DDR contents from previous boot's RAM scribbles persist" | Soft-reset doesn't re-run boot_fsm; the 4 MiB pre-zero pass at boot_fsm.ST_ZERO_AW only runs on a true `boot_fsm_rst`. |
| "EXC_COUNT looks weirdly large at N=1000" | `counters_clear` is gated on `soc_full_rst`, not on `dbg_soft_rst`. |
| "`ctrl_halt_req` lost the race; CPU runs a few hundred insts before halt re-asserts" | CONTROL bit 0 is cleared by `core_rst` (== `rst`) at start of reset cycle. The "hold across reset" contract is not implementable today as a single bit. |

---

## 4. Proposed unified reset semantics

### 4.1 Goal

Exactly **one** reset operation visible to host tools. Resets ALL
in-FPGA state (CPU pipeline + ROB/RAT/IQ + caches + TLB + debug
counters + EXC latches + peripherals + boot FSM + DDR ROM region +
VIA1 overlay + AXI xbar + DMA + irq_agg + ...). The only state that
SURVIVES is one sticky "hold after reset" bit chosen by the host
between reset assertion and reset deassertion.

### 4.2 The one bit that survives: `cold_reset_hold`

A single new register bit, owned by `debug_ctrl`, in a clock domain
that survives `soc_full_rst`. Semantics:

| Bit | Set by | Cleared by | Effect while set |
|---|---|---|---|
| `cold_reset_hold` | `OFF_CONTROL` write (new bit, e.g. bit 4) — JTAG-AXI host | explicit write to clear (write 0 to bit 4); NOT cleared by any reset short of board cold reset (`core_rst`) | After `cold_reset` deasserts, the CPU stays held in reset (via OR into `cpu_rst_or`) until `cold_reset_hold` is cleared. |

**Lifecycle:**
1. Host: write CONTROL.cold_reset_hold = 1.
2. Host: trigger `cold_reset` (via OFF_CONTROL.cold_reset_pulse, see below).
3. RTL: pulse asserts `soc_full_rst` for the canonical 1024-cy stretch (5 µs @ 200 MHz). Boot FSM, peripherals, CPU, debug counters all rewind. **`cold_reset_hold` itself is in the JTAG-AXI register block on `core_rst` (NOT `soc_full_rst`/`counters_clear`), so it survives.**
4. RTL: after `soc_full_rst` deasserts, boot_fsm starts the SD→DDR ROM copy. When done, `boot_rom_ready` rises. Normally CPU would now run. But `cold_reset_hold=1` keeps `cpu_rst` high.
5. Host: in this held window, configures halt-after / break_pc / halt-exc mask / arch_shadow / etc. The debug regs are in the "survives soft-reset" group anyway, but with the CPU held everything is settled.
6. Host: clears `cold_reset_hold` (write CONTROL bit 4 = 0). CPU enters fetch from a guaranteed cold state with breakpoints already armed.

This is a strict superset of today's `full-reset-and-halt` ergonomics
without the race window.

### 4.3 The one trigger: `cold_reset_pulse`

Replace the level-driven `vio_boot_ctrl[3]` and the JTAG-AXI
`dbg_soft_rst` pulse with a single edge-triggered pulse generator
inside `debug_ctrl`. Mechanism:

- New CONTROL bit (e.g. bit 5) `cold_reset_pulse` — write-1-to-pulse.
- Drives a one-cycle `dbg_cold_reset_pulse` output → fpga_top_clocks.vh
  ORs into the existing `dbg_rst_src_level` chain (alongside btn[2] and
  vio_boot_ctrl[3]). The 1024-cycle stretcher already in place handles
  pulse extension and DDR drain.

The board-button (btn[2]) and the JTAG-VIO (vio_boot_ctrl[3]) paths
remain — they're physical/operational fallbacks. **The TOOLING never
uses them in the unified surface; tools always go through the
JTAG-AXI debug_ctrl pulse.** That collapses the host-tool surface to
one register write.

### 4.4 What gets RESET by the unified reset (vs today)

The unified reset = today's `soc_full_rst` (broadcast to bank[7:0] +
pb_full_rst_bank[3:0] + counters_clear), PLUS:

- `dbg_soft_rst` is removed (the CPU-only soft-reset path goes away).
- The `boot_rom_ready` gating already keeps the CPU held until the SD copy
  completes — this stays.
- DDR PHY / MIG cal: still NOT reset (100 ms penalty is unacceptable for
  iterative bring-up). DDR *contents* in the ROM region are clobbered by
  the boot_fsm re-copy; DDR contents in RAM are clobbered by the
  pre-zero pass. **This is good enough — explicitly documented.**
- sys_clk MMCM tree: still NOT reset (would glitch every clock domain).
- HDMI MMCM: stays as today (resets on `soc_full_rst_bank[0]`).
- `axi_error_sticky_r` and `err_s0_*_addr_r`: TODAY reset by `core_rst`
  only (board cold reset). Phase-2 RTL change: move these to
  `soc_full_rst` so the unified reset clears AXI error history too.
  *Open question: do we want to preserve AXI error history across an
  iteration? Probably no — it's per-run forensic state. Move it.*

### 4.5 Things that CAN'T be reset cleanly without RTL work

- **DDR PHY / MIG**: can be reset, but pays 100 ms re-cal. Documented.
- **JTAG host-side AXI bridge** (`u_dbg_axi_to_axil` and chain): if we
  reset this, the JTAG channel that's *driving* the reset goes down.
  Stays on `pb_rst` (board cold reset only).
- **Sys_clk MMCM tree**: tearing it down breaks every clock domain.
  Stays.

These three are inherently exempt; the doc must say so explicitly.

### 4.6 Tooling consolidation

After Phase 2:

- REPL: single `reset` command with optional `hold` / `release`
  modifiers. `reset` = pulse + clear hold. `reset hold` = set hold,
  pulse, leave held. `release` = clear hold (deferred resume).
  `reset-halt-after`, `full-reset`, `full-reset-and-halt`, `sweep`
  collapse to compositions of `reset hold` + arm + `release`.
- TUI: single `cold-reset` subcommand with same modifiers. Old
  subcommands become aliases that print a deprecation notice for one
  release, then are removed.
- Old VIO bits 0/1/2/3 of `vio_boot_ctrl`: 0 (bypass_sd) and 1
  (release_cpu) stay — they're orthogonal "host-loaded ROM" plumbing.
  Bit 2 (scc_uart_sel_b) stays — orthogonal mux. Bit 3 stays as the
  physical/recovery path (JTAG VIO and btn[2]) but the tooling no
  longer uses it.

### 4.7 New REPL surface (proposed)

```
reset                  — fire one pulse, do not hold (default)
reset hold             — set hold bit, fire pulse, leave CPU held
reset release          — clear hold bit (resumes CPU; no pulse)
reset hold-status      — read CONTROL.cold_reset_hold

reset-and-halt-after N — convenience: reset hold + arm halt-after N + release
sweep <wait> N1 N2..  — convenience: per-N reset hold + arm + release + wait + snap
```

The legacy `full-reset`, `full-reset-and-halt`, `reset-halt-after`
become aliases that emit a deprecation warning and call the new path.
**Recommendation: keep the aliases for one release**, then delete in a
follow-up. Bench scripts that hard-code these names exist; one-cycle
deprecation gives them a window to update.

---

## 5. Phase-2 sketch (NOT implemented yet)

1. `debug_ctrl.v`:
   - Add `OFF_CONTROL` bit 4 = `cold_reset_hold` (level, rst-bank: `rst` only — survives `counters_clear` AND survives `dbg_full_rst_in`).
   - Add `OFF_CONTROL` bit 5 = `cold_reset_pulse` (write-1-to-pulse).
   - Output `dbg_cold_reset_hold` and `dbg_cold_reset_pulse` ports.
   - Remove or deprecate `ctrl_soft_rst_pulse` / `dbg_soft_rst`. Keep
     for one release as a wire that maps to `dbg_cold_reset_pulse` for
     compatibility, with a comment.

2. `fpga_top_clocks.vh`:
   - OR `dbg_cold_reset_pulse` into `dbg_rst_src_level`.
   - Replace `cpu_rst_or = soc_full_rst || dbg_soft_rst || !boot_rom_ready || !cpu_rst_settle_done`
     with `cpu_rst_or = soc_full_rst || dbg_cold_reset_hold || !boot_rom_ready || !cpu_rst_settle_done`.
     I.e. the hold bit replaces the soft-rst pulse as a CPU-only gating
     reason.

3. Move `axi_error_sticky_r` (in `fpga_top_debug_vio.vh:159`) from
   `core_rst` to `soc_full_rst` so AXI-error forensic state clears with
   the unified reset. (Or add a `counters_clear` analog.)

4. Make sure `cold_reset_hold` is in a clock-domain that survives the
   `soc_full_rst` it triggers. `debug_ctrl` already runs on `core_clk`
   with `core_rst` as its reset (not `soc_full_rst` and not
   `counters_clear`). So `cold_reset_hold` naturally survives. ✓

## 6. Phase 4 — sim test

A new `tb-reset-cleanslate` directed sim test:
- Boot, run a few hundred cycles, write known patterns to D-cache
  (force a dirty line), set `ctrl_halt_req`, scribble a few peripheral
  registers (VIA1 ORB, ASC FIFO, scratch memory), assert pending IRQs.
- Issue the unified reset.
- Verify: PRF=0, ROB head/tail = 0, RAT = identity, dcache valid bits
  all clear, icache valid bits all clear, TLB invalid, VIA1 ORB =
  reset value (`0x80`), VIA1 overlay_bit = 1, ASC FIFO empty, IRQ
  state idle, debug perf counters = 0, EXC latches = 0,
  cycle_count = 0.
- Verify the hold flag survives: set hold, reset, observe CPU stays
  in reset (cpu_rst high), peripherals settled. Clear hold, observe
  CPU resumes from cold-boot vector-0 fetch.

---

## 7. Open questions — RESOLVED (Phase 1 → Phase 2 handoff)

User-confirmed defaults:
- (a) Bit positions in `OFF_CONTROL`: bit 4 = `cold_reset_hold`, bit 5
  = `cold_reset_pulse`. Bits 0/1/2/3 = halt/step/legacy soft_rst/
  init_done_ovr stay where they are for compat. **Confirmed.**
- (b) Keep legacy `vio_boot_ctrl[3]` + btn[2] hardware recovery paths.
  **Confirmed.**
- (c) Legacy REPL/TUI commands stay as aliases for one release, then
  deletion in a follow-up. **Confirmed.**
- (d) Move `axi_error_sticky_r` reset from `core_rst` to
  `soc_full_rst`. **Confirmed.**
- (e) Keep `dbg_init_done_override` (CONTROL bit 3) as the init gate.
  **Confirmed.**

## 8. Phase-2 landing summary

RTL files touched:
- `rtl/core/debug/debug_ctrl.v` — added `ctrl_cold_reset_hold`
  (DBG_CONTROL bit 4 — sticky on `core_rst` only) and
  `ctrl_cold_reset_pulse` (bit 5 — write-1-to-pulse, auto-clears).
  New output ports `dbg_cold_reset_hold` and `dbg_cold_reset_pulse`.
  `dbg_soft_rst` retained as a deprecation alias that ORs the legacy
  bit-2 pulse with the new bit-5 pulse so legacy callers land on the
  same unified-reset path.  `reset_cause_r` now also records 'soft'
  for the new bit-5 pulse.
- `rtl/fpga_top_clocks.vh` — `dbg_cold_reset_pulse` ORs into
  `dbg_rst_src_level` alongside `jtag_debug_full_reset` (VIO bit 3)
  and `btn2_sync` (btn[2]).  The 1024-cycle stretcher therefore
  treats the JTAG-AXI pulse identically to the VIO/btn paths.  The
  CPU reset OR replaces `dbg_soft_rst` with `dbg_cold_reset_hold`:
  the CPU stays held while bit 4 is set, *not* on a 1-cycle pulse
  glitch — a simpler invariant.  `dbg_soft_rst` itself is still
  declared as a wire and driven (legacy alias) so the deprecation
  contract holds, but it is NOT a CPU-reset gating signal anymore.
- `rtl/fpga_top_debug_ctrl.vh` — plumbs the two new ports out of
  `u_debug` to top-level wires.
- `rtl/fpga_top_debug_vio.vh` — `axi_error_sticky_r` and
  `err_s0_*_addr_r` reset domain moved from `core_rst` to
  `soc_full_rst` so the unified reset clears AXI error history too.

Coverage check (per docs/reset_story.md §1.5 + §4.4):
The unified reset now reaches every state-bearing module that
`soc_full_rst` reaches today, PLUS the previously
`core_rst`-only `axi_error_sticky_r` and friends.  No state-bearing
module is left out without explicit documented exemption.  Exemptions
(per §4.5) remain:
- DDR PHY / MIG calibration (would pay 100ms re-cal — DDR contents
  are clobbered by boot_fsm refill anyway).
- JTAG host-side AXI bridge (`u_dbg_axi_to_axil`) — resetting the
  bridge that is *driving* the reset request would deadlock the
  channel.
- sys_clk MMCM tree — tearing it down would glitch every clock domain.

These three are documented exempt because resetting them either
defeats the iterative-bringup UX (DDR re-cal) or self-destroys the
mechanism that issued the reset (JTAG bridge / sys_clk).  No state in
these subsystems contaminates the CPU's view of the world: DDR
contents that the CPU sees are re-initialised by the boot_fsm copy
and the 4 MiB pre-zero pass; the JTAG bridge holds no architectural
state visible to the CPU; the sys_clk tree is just a clock.
