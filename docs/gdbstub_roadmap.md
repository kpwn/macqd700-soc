# gdbstub Roadmap — m68k-ooo Debug Infrastructure

> Long-running plan for evolving the in-core debug primitives into a
> gdb-remote-serial-protocol target.  The host gdbstub will speak GRSP
> on a TCP port and drive the JTAG-AXI fabric through the existing
> `tools/jtag_repl.tcl` REPL pattern.
>
> Owner: gdbstub-foundation agent.  See the spawning prompt for scope
> rules; this file is the cumulative status / progress log.

---

## Phase 1 Audit (initial drop, `main @ e9ddfa29`)

This phase is a paper exercise — no RTL changes.  Goal: enumerate every
debug primitive that ships today, compare it against what the gdb remote
protocol needs, and lay out small landable chunks for the Phase 2-6 work
that follows.

### 1.1  Inventory of shipped debug primitives

All offsets below are BAR-local (subtract `DBG_BASE = 0x50900000` for the
absolute AXI address used by `tools/jtag_repl.tcl`).

| #  | Primitive                          | RTL location                                       | Host offset(s)                    | Status                  |
|----|------------------------------------|----------------------------------------------------|-----------------------------------|-------------------------|
| 1  | Manual halt (level)                | `debug_ctrl.v` `ctrl_halt_req`                     | `OFF_CONTROL` 0x008 bit 0         | Shipped                 |
| 2  | Soft reset (pulse)                 | `debug_ctrl.v` `ctrl_soft_rst_pulse`               | `OFF_CONTROL` 0x008 bit 2         | Shipped                 |
| 3  | Single-step (pulse)                | `debug_ctrl.v` `ctrl_step_pulse` → `debug_stop_manager.dbg_step_active_r` | `OFF_CONTROL` 0x008 bit 1         | Shipped                 |
| 4  | init-done override                 | `debug_ctrl.v` `ctrl_init_done_override`           | `OFF_CONTROL` 0x008 bit 3         | Shipped                 |
| 5  | Halt-after-N (64-bit retired-inst counter compare) | `debug_stop_manager.dbg_halt_after_hit_now` | `OFF_HALT_AFTER_LO/HI` 0x030/0x034 + `OFF_HALT_CTL` 0x03C bit 0 | Shipped                 |
| 6  | Break PC (single PC)               | `debug_stop_manager.dbg_break_pc_hit_now`          | `OFF_BREAK_PC` 0x038 + `OFF_HALT_CTL` 0x03C bit 1 | Shipped (1 slot)        |
| 7  | Halt-on-exception (256-bit mask)   | `debug_stop_manager.dbg_halt_exc_hit_now`          | `OFF_HALT_EXC_MASK_0..7` 0x060-0x07C + `OFF_HALT_CTL` 0x03C bit 6 | Shipped                 |
| 8  | Auto-halt latch (level until cleared) | `debug_ctrl.auto_halt_latched_r`               | `OFF_HALT_CTL` 0x03C bit 4-7 (read), bit 2 (write-1-to-clear) | Shipped                 |
| 9  | Halt-hit PC capture                | `debug_ctrl.halt_hit_pc_r`                         | `OFF_HALT_HIT_PC` 0x044           | Shipped                 |
| 10 | Halt-hit retired-inst capture (64) | `debug_ctrl.halt_hit_inst_r`                       | `OFF_HALT_HIT_INST_LO/HI` 0x048/0x04C | Shipped                 |
| 11 | Halt reason bits (3 bits)          | `dbg_auto_halt_reason[2:0]`                        | `OFF_HALT_REASON` 0x040           | Shipped                 |
| 12 | Live PC                            | `commit.v` → `dbg_pc`                              | `OFF_PC` 0x010                    | Shipped                 |
| 13 | Last committed PC                  | `debug_ctrl.last_pc_r`                             | `OFF_LAST_PC` 0x014               | Shipped                 |
| 14 | Live VBR / SR / A7                 | `commit.v` → `dbg_live_vbr/sr/a7_in`               | `OFF_LIVE_VBR/SR/A7` 0x2100/4/8   | Shipped                 |
| 15 | Live D0-D7 / A0-A7 (snap chain)    | `m68k_core_execute.vh` snap chain → `snap_value_i` | `OFF_LIVE_D0..A7` 0x2110-0x214C   | Shipped (halt-only)     |
| 16 | Arch shadow apply (write D0..A7, SR, VBR, CACR, ITT/DTT/TC/URP/SRP, USP/SSP/ISP, SFC/DFC, PC) | `debug_ctrl.arch_shadow_*` + `arch_apply_state` FSM | `OFF_ARCH_*` 0x2000-0x2084 + `OFF_ARCH_APPLY` 0x2078 | Shipped                 |
| 17 | Last exception capture (vec, PC, fault addr) | `debug_ctrl.exc_vec_r/exc_pc_r/exc_fault_addr_r` | `OFF_EXC_VEC` 0x024, `OFF_EXC_PC` 0x028, `OFF_EXC_FAULT_ADDR` 0x054 | Shipped                 |
| 18 | Cycle counter (64-bit)             | `fpga_top_debug_ctrl.vh` `cycle_count`             | `OFF_CYCLE_LO/HI` 0x1000/0x1004   | Shipped                 |
| 19 | Inst counter (64-bit)              | `debug_stop_manager.dbg_inst_count`                | `OFF_INST_LO/HI` 0x1008/0x100C    | Shipped                 |
| 20 | Mispred / flush / exc counters (32-bit) | `debug_ctrl` local counters                  | `OFF_MISPRED_COUNT/FLUSH/EXC` 0x1010-0x1018 | Shipped                 |
| 21 | PC trace ring (64-deep on bitstream, 1024 in unit-tb) | `debug_ctrl.pc_trace[]` | `OFF_PC_TRACE_BASE` 0x10000 + `OFF_PC_TRACE_HEAD` 0x11000 | Shipped                 |
| 22 | cpu_halted (double-fault HALT)     | `commit.v` → `cpu_halted_w` → effective_halt OR    | Visible via `dbg_halted` bit in `OFF_STATUS` 0x00C bit 0 | Shipped (latched)       |
| 23 | Redirect PC + trigger              | `debug_ctrl.redirect_pc_r`                         | `OFF_REDIRECT_PC` 0x018 + `OFF_REDIRECT_TRIGGER` 0x01C | Shipped                 |
| 24 | IRQ inject pulse (3-bit level)     | `debug_ctrl.irq_inject_lvl_r`                      | `OFF_IRQ_INJECT` 0x020            | Shipped                 |
| 25 | Memory R/W via JTAG-AXI fabric     | xbar → ddr / vram / mmio                            | Anywhere in 0x00000000-0xFFFFFFFF | Shipped                 |
| 26 | Counters-clear on warm reset       | `counters_clear` input from `soc_full_rst`         | (handled by full-reset)           | Shipped                 |
| 27 | Boundary-pc capture from commit    | `commit.v` → `dbg_boundary_pc/kind/exc_vec/...`    | (consumed by `debug_stop_manager`)| Shipped (internal)      |

### 1.2  Gap analysis vs gdb remote serial protocol

The gdbstub host translates GRSP packets into JTAG-AXI ops.  Below is each
relevant primitive, what it needs from the RTL, and the gap.

| GRSP   | Description                                  | What the RTL needs                                          | Gap today                                                                                  |
|--------|----------------------------------------------|--------------------------------------------------------------|--------------------------------------------------------------------------------------------|
| `?`    | Why-stopped reply                            | Stop reason + hit-pc + (faulting addr / hit slot index)      | All present individually but not packed for atomic readback (see Phase 4)                  |
| `g`    | Read all general registers                   | D0-D7, A0-A7, SR, PC                                         | All present (`live-arch` cmd in REPL).  Snap chain only valid while halted — gdbstub will  |
|        |                                              |                                                              | have to enforce halt before `g` (gdb only sends `g` when stopped, so this is fine).        |
| `G`    | Write all general registers                  | Apply path for D0-D7/A0-A7/SR/PC                             | Shipped via `arch_shadow_*` + `arch_apply` FSM                                             |
| `m`    | Read N bytes of memory                       | AXI memory reads at any addr                                 | Shipped                                                                                    |
| `M`    | Write N bytes                                | AXI memory writes                                            | Shipped                                                                                    |
| `c`    | Continue                                     | Release halt                                                 | Shipped (write 0 to OFF_CONTROL)                                                           |
| `s`    | Single step                                  | step_pulse                                                   | Shipped                                                                                    |
| `Z0`/`z0` | SW breakpoint (host writes 0x4AFC ILLEGAL) | Memory write + halt-on-vec-4 mask                          | Shipped — gdbstub sets bit 4 in halt-exc mask and writes ILLEGAL into target memory        |
| `Z1`/`z1` | HW breakpoint                              | Multiple PC slots                                            | **GAP — only 1 slot today.  Phase 2 widens to 4.**                                        |
| `Z2`/`z2` | Write watchpoint                           | Store-side EA compare                                        | **GAP — does not exist.  Phase 3.**                                                       |
| `Z3`/`z3` | Read watchpoint                            | Load-side EA compare                                         | **GAP — Phase 3 follow-on.**                                                              |
| `Z4`/`z4` | Access watchpoint                          | Either side                                                  | **GAP — Phase 3 follow-on (combine Z2+Z3).**                                              |
| `T`/`?` | Stop reply with structured reason          | { reason, hit-pc, fault-addr, vec, slot-index }              | Today: 4 separate reads.  Phase 4 packs into a 128-bit virtual register.                  |
| `qSupported` / `qAttached` / `vMustReplyEmpty` etc. | host capability negotiation       | (host-side only)                                             | Phase 5 — host-side gdbstub                                                                |
| `qXfer:features:read` | gdb-side target.xml (register layout) | host-side only                                              | Phase 5                                                                                    |
| `vCont` / `vCont?` | thread/extended continue              | release-halt                                                 | Single-thread target — `vCont;c` and `vCont;s` map directly to Phase 1 primitives          |
| `qSymbol` / `qOffsets` | symbol negotiation                  | host-side only                                               | Phase 5                                                                                    |
| `D` / `k` | detach / kill                              | release halt + clear breakpoints                             | Shipped (host clears bp/wp slots and writes 0 to OFF_CONTROL)                              |

### 1.3  Existing test coverage

| Test                                | Covers                                                             |
|-------------------------------------|--------------------------------------------------------------------|
| `tb_debug_ctrl.cpp` (`make tb-debug`) | version reg, ctrl level/pulse bits, cycle/inst counter, PC trace ring, redirect, exc capture, IRQ inject, RAM-window-lg2, auto-halt latch, unmapped-zero, arch-shadow apply |
| `cpu/tb/tb_debug_stop_manager.cpp` (`make tb-debug-stop`, from EITHER repo root) | step releases for one retire, halt-after retired-inst count semantics, break_pc only on retire kind, clear_run cancels pending step, macro-aligned watchpoint deferral, saturating watch-pend count, `dbg_precise_stop_req` restricted to the double-fault source |
| `tb_debug_full_reset.cpp` (`make tb-debug-full-reset`) | warm-reset clears counters but preserves ctrl regs                |
| `tb_dbg_rst_pulse.cpp` (`make tb-dbg-rst-pulse`) | soft-rst pulse fires correctly                                    |

The unit-tb framework is solid — every new primitive added in Phase 2/3
should land with a directed test in `tb_debug_ctrl.cpp` or
`cpu/tb/tb_debug_stop_manager.cpp` exercising the new path.  Note the SoC
repo no longer keeps its own copy of either the module or that tb: the
SoC `tb-debug-stop` target builds the cpu/ sources directly (2026-08-19),
so module and contract can only move together.

### 1.4  Architectural constraints to preserve

- **F2-rearch reg-stage** — `debug_stop_manager` registers the wide
  comparator output before driving `dbg_core_halt` (line 197 of
  fpga_top.v).  Any new comparator (Phase 2 BP slots, Phase 3 WP slots)
  must mirror this — fold the OR-tree into `dbg_auto_halt_event_w`,
  let the existing `_q` register provide the F2 cushion.
- **No SystemVerilog** — Verilog-2005 only.  Per-slot loops want
  `genvar`+`generate` blocks, not packed-array `for`.
- **counters_clear vs rst** — host-set control regs (break_pc array,
  watchpoint slots) belong on `rst` (= core_rst), not on
  `counters_clear` (= soc_full_rst).  Otherwise a JTAG full-reset wipes
  the user's breakpoints out from under them.
- **debug_ctrl line budget** — current size 1152 lines.  Adding 4 BP
  slots + 4 WP slots will push past 1300.  CLAUDE.md says "≤ 300 lines
  per module."  This module has long been an exception (it's a register
  file, not pipeline logic), but if growth gets worse I'll consider
  factoring the watchpoint comparator out as `debug_wp.v`.
- **Halt-on-exc latency** — the existing 1-cycle skid means halt-after-N
  fires at instruction N+1.  Keep this discipline; document the same
  N+1 skid for any new primitive that runs through the F2 reg stage.

### 1.5  Prioritised work plan

Each phase below should land as one focused commit (or a small series),
with sim regression + lint clean before merge.  Order optimised for "fewest
host-side rewrites later" — gdbstub host work is gated on the structured
stop-reason readback (Phase 4), so RTL widenings come first.

1. **Phase 2 (next)** — Multi-slot HW breakpoints (4 PC slots).  Widens
   `dbg_break_pc` from 1×32-bit + 1 enable to 4×32-bit + 4 enables.  REPL
   commands `bp set/clear/list`.  Directed unit-tb test that exercises
   only-matching-slot-fires.

2. **Phase 3** — Write watchpoints (4 slots).  Tap `commit_store_en` +
   final store EA from LSU into the F2 reg stage.  Slot config: enable +
   addr + 4-bit byte mask.  REPL `wp set/clear/list`.  Wire a new reason
   bit (`dbg_auto_halt_reason[3]`) for "watchpoint hit" and a `wp_hit_slot`
   readback so the host can find which slot triggered.

3. **Phase 4** — Stop-reason structured readback.  Combine
   `{ reason, hit-pc, fault-addr, vec, slot-index, retired-inst-count }`
   into a single 128-bit virtual readout (4 sequential 32-bit offsets).
   This makes the gdbstub `?`/`T` reply atomic — no race between separate
   register reads.

4. **Phase 5** — Host-side gdbstub prototype.  `tools/gdbstub.py`,
   listens TCP 1234, drives a fork of `jtag_repl.tcl` via the existing
   FIFO pattern, translates GRSP packets.  Validation: `gdb-multiarch
   --eval-command "target remote :1234"` attaches, can `c`, `s`, `g`,
   read memory, set both SW and HW breakpoints.

5. **Phase 6 (stretch)** — Conditional breakpoints.  Per-slot
   `{ reg_idx, op, imm32 }` comparator AND'd with the PC match.  Likely
   only worth doing once we have host-side pressure — gdb fires a HW BP
   then evaluates the condition host-side anyway (it's slower but works
   without RTL support).

### 1.6  Phase 2 design preview (next concrete deliverable)

**Choice: 4 BP slots, not 8.**  Cost-benefit:

- **4 slots:** 4×32 bits (= 128 PC FFs) + 4 enables + 4×32-bit comparator =
  ~600 LUTs of register + comparator logic + a 4-input OR.  Comfortably
  fits behind the F2 reg stage.  Matches the most-common gdb workflow
  (one breakpoint per stack frame: caller, callee, two extras for "what
  the hell did the BIOS do").

- **8 slots:** 256 PC FFs + 8 comparators + 8-input OR.  Roughly 2× the
  area for a use-case (>4 simultaneous HW BP) that almost never appears
  in practice; gdb falls back to SW BP when out of HW slots.

- **Decision: 4 slots** for the first cut.  Easy to widen later if any
  user code actually consumes them all.

**Register layout (proposed):**

```
0x100  OFF_BREAK_PC_0           — slot 0 PC compare
0x104  OFF_BREAK_PC_1           — slot 1 PC compare
0x108  OFF_BREAK_PC_2           — slot 2 PC compare
0x10C  OFF_BREAK_PC_3           — slot 3 PC compare
0x110  OFF_BREAK_PC_EN          — bits[3:0] = per-slot enable
0x114  OFF_BREAK_PC_HIT_SLOT    — last hit slot (0-3); read-only
```

The legacy `OFF_BREAK_PC` (0x038) and `dbg_break_pc_enable` (HALT_CTL bit
1) become aliases for slot 0 — preserves bring-up tooling, no
backwards-compat hacks needed.

**Comparator structure (in `debug_stop_manager`):**
```verilog
wire [3:0] bp_match;
genvar gi;
generate for (gi = 0; gi < 4; gi = gi + 1) begin : gen_bp
    assign bp_match[gi] = dbg_retire_boundary_event &&
                          dbg_break_pc_enable[gi] &&
                          (dbg_boundary_pc == dbg_break_pc[gi]);
end endgenerate
wire dbg_break_pc_hit_now = |bp_match;
```

The `bp_match` bus feeds a priority encoder for `OFF_BREAK_PC_HIT_SLOT`.

**Top-level wiring:**  `dbg_break_pc` becomes `[3:0][31:0]` (or 4 separate
32-bit busses to keep Verilog-2005 happy).  `dbg_break_pc_enable` becomes
4 bits.  `debug_stop_manager` carries them through; `debug_ctrl` exposes
them via the new offsets above.

**Test:** `test_break_pc_multi_slot()` in `tb_debug_stop_manager.cpp` —
arm 3 slots at distinct PCs, drive the boundary stream past each, verify
auto_halt_event fires only once per matching slot and the slot index
register matches.

---

## Phase 1 status: COMPLETE

This audit document is the deliverable.  No RTL changed.  Next session
opens with Phase 2 RTL work guided by §1.6 above.

## Progress log

(appended in chronological order; each phase appends one entry)

### 2026-05-04 — Phase 1 audit landed
Initial inventory + gap analysis + work plan.  No RTL changes, just this
document.  Ready for Phase 2 multi-slot breakpoint implementation.
