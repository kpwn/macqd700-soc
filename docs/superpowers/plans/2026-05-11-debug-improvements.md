# Debug Improvements: Fault Snapshot + LIVE_VBR Fix + Precise Breakpoints Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make on-FPGA debugging trustworthy by (a) fixing the broken `OFF_LIVE_VBR` JTAG readback, (b) adding a sticky fault-snapshot register set that captures LSU/dcache/AXI state at the cycle a vec=2 fires, and (c) replacing the current post-retire break-pc with a decode-injected DBG µop that halts *before* the breakpoint instruction's side effects apply, supports state mutation during halt, and resumes via a skip-once mechanism.

**Architecture:**

1. **LIVE_VBR fix**: pipeline `arch_vbr` through a dedicated `dbg_live_vbr_r` flop inside `debug_ctrl` with the same shape as `arch_sr_o → dbg_live_sr` (which works). Drop the `arch_vbr_w` intermediate wire from the readback path. Add a tb-debug-ctrl test that proves the readback equals the value MOVEC'd via `arch-apply`.

2. **Fault snapshot**: latch a wide bundle of LSU/dcache/AXI state (the same fields that `dbg_wedge_state` exports, but *latched* on the cycle `cmpl_exc && (cmpl_exc_vec == 8'd2)` first fires; sticky until unified reset or explicit JTAG clear). Add new offsets `OFF_FAULT_SNAP_*` in debug_ctrl.

3. **Precise DBG breakpoint**: add `SYS_DBG_BREAK` µop (new SYS op code). Decode prepends it to any macro whose macro-PC matches `break_pc_r` and `!break_pc_skip_once_r`. Commit recognises the DBG µop, asserts the halt-precise event, flushes the DBG µop and everything younger (including the rest of the macro), and leaves arch state unchanged. Continue: host sets `break_pc_skip_once_r=1`, releases halt; decode skips DBG injection on the next match, clears `break_pc_skip_once_r` automatically.

**Tech Stack:** Verilog-2005 RTL, Verilator unit testbenches (tb_debug_ctrl, tb_commit, tb_lsu), JTAG-AXI REPL (`tools/jtag_repl.tcl`).

---

## File Map

| File | Responsibility |
|------|----------------|
| `rtl/core/decode/uop_pkg.v` | Add `SYS_DBG_BREAK` op code |
| `rtl/core/debug/debug_ctrl.v` | LIVE_VBR fix; fault-snap regs; BP_SKIP_ONCE bit; DBG-uop halt path |
| `rtl/core/m68k_core.v` | Plumb dc_rresp, dcache_state, lsu_state, dmmu_fault, AXI handshake into a `dbg_fault_snap_in` bundle; drop intermediate `arch_vbr_w` from `dbg_live_vbr`; expose `dbg_break_pc_skip_once`; wire DBG-µop fire from commit back to debug_ctrl |
| `rtl/core/m68k_core_commit.vh` | Replace `.arch_vbr_o(arch_vbr_w)` plumbing for VBR readback so it goes directly to a debug port like `arch_sr_o` does; keep `arch_vbr_w` only for the exception module |
| `rtl/core/commit.v` | Recognise `UOP_SYS / SYS_DBG_BREAK` at retire: assert `dbg_break_uop_fire`, flush younger, suppress retire-write |
| `rtl/core/decode/decode_0100.vh` or wherever macro→µop dispatch lives | Inject DBG µop when `macro_pc == break_pc && break_pc_enable && !break_pc_skip_once` |
| `rtl/fpga_top.v` (`debug_stop_manager`) | Add `dbg_break_uop_fire` input; replace post-retire break-pc trigger with DBG-uop trigger for halt semantics; keep retire-pc as legacy fallback only |
| `rtl/fpga_top_cpu.vh` | New wires `dbg_fault_snap_w[127:0]`, `dbg_break_uop_fire_w`, `dbg_break_pc_skip_once_w`; drop `arch_vbr_w` from the debug readback path |
| `rtl/fpga_top_debug_ctrl.vh` | New port hookups for the above |
| `tb/tb_debug_ctrl.cpp` | New tests for VBR readback round-trip + fault-snap latching via stubbed cmpl_exc signal + BP_SKIP_ONCE reg semantics |
| `tb/tb_commit.cpp` | New test that issues a UOP_SYS / SYS_DBG_BREAK + a follow-up integer µop, verifies (a) halt fires, (b) follow-up never retires |
| `tb/tb_fpga_top_rom.cpp` (if needed) | Smoke test: set break_pc at a ROM address, run, verify halt + arch regs are at pre-instruction state, verify continue runs the instruction |
| `tools/jtag_repl.tcl` | Update `break-pc` and add `continue` (or extend existing `arch-apply`) to set BP_SKIP_ONCE before releasing halt; document the new precise semantics in the docstring |

---

## Phase 1 — LIVE_VBR Readback Fix

**Why first:** smallest change, removes a debugging-trust hazard that contaminates every other piece of work.

### Task 1.1: Reproduce in sim

**Files:**
- Test: `tb/tb_debug_ctrl.cpp`

- [ ] **Step 1: Add failing test**

In `tb/tb_debug_ctrl.cpp` add a new test function `test_live_vbr_readback`:

```cpp
// Verify OFF_LIVE_VBR returns the value present in arch_vbr.
// Reproduces the 2026-05-11 HW finding where OFF_LIVE_VBR
// returned a value matching A5 instead of the real VBR.
static void test_live_vbr_readback(Vdebug_ctrl* dut) {
    reset_dut(dut);
    // Drive dbg_live_vbr_in to a known value; the test bench
    // is the only thing feeding this input.
    dut->dbg_live_vbr_in = 0xCAFEBABEu;
    tick(dut);
    tick(dut);
    uint32_t got = axi_read(dut, OFF_LIVE_VBR);
    if (got != 0xCAFEBABEu) {
        fprintf(stderr, "LIVE_VBR readback mismatch: got 0x%08x want 0xCAFEBABE\n", got);
        std::exit(1);
    }
    // Change the driver, verify the readback follows.
    dut->dbg_live_vbr_in = 0x40846980u;
    tick(dut);
    tick(dut);
    got = axi_read(dut, OFF_LIVE_VBR);
    if (got != 0x40846980u) {
        fprintf(stderr, "LIVE_VBR readback didn't follow: got 0x%08x want 0x40846980\n", got);
        std::exit(1);
    }
    fprintf(stderr, "[tb-debug-ctrl] test_live_vbr_readback PASS\n");
}
```

Register it in `main()` alongside the existing tests.

- [ ] **Step 2: Run and confirm it passes against debug_ctrl alone**

Run: `make tb-debug-ctrl`
Expected: PASS — debug_ctrl alone correctly latches `dbg_live_vbr_in`. **This is the baseline; it proves the bug is upstream of debug_ctrl.**

If it FAILS, the bug is inside debug_ctrl's case statement — investigate there first before continuing.

- [ ] **Step 3: Commit**

```bash
git add tb/tb_debug_ctrl.cpp
git commit -m "tb-debug-ctrl: add LIVE_VBR readback round-trip test"
```

### Task 1.2: Restructure the VBR readback wire to match the working SR pattern

**Files:**
- Modify: `rtl/core/commit.v:230` (add `arch_vbr_dbg_o` output if not present)
- Modify: `rtl/core/m68k_core_commit.vh:443` (route `arch_vbr_o` directly to a top-level dbg port, mirror what's done at line 452 for `arch_sr_o`)
- Modify: `rtl/core/m68k_core.v:264-267` (drop the intermediate `assign dbg_live_vbr = arch_vbr_w` and instead make `dbg_live_vbr` a direct output of commit, like `dbg_live_sr`)

The working SR pattern is at `m68k_core_commit.vh:452`: `.arch_sr_o(dbg_live_sr)` — commit's output port is wired *directly* to the m68k_core output. No intermediate wire. Mirror that.

- [ ] **Step 1: Modify `m68k_core_commit.vh:443`**

```verilog
// before:
.arch_vbr_o(arch_vbr_w),
// after:
.arch_vbr_o(dbg_live_vbr),   // direct wire to m68k_core output, like arch_sr_o
.arch_vbr_dbg_o(arch_vbr_w), // legacy fan-out for exception.v
```

If `commit.v` only has one `arch_vbr_o` port, duplicate the assign at the bottom of commit.v: `assign arch_vbr_dbg_o = arch_vbr;` (add the new port at line 230).

- [ ] **Step 2: Remove the now-redundant assign in `m68k_core.v`**

Delete line 267: `assign dbg_live_vbr = arch_vbr_w;`

`dbg_live_vbr` is now driven directly by `commit.arch_vbr_o`.

- [ ] **Step 3: Lint**

Run: `make lint`
Expected: 0 warnings. If multiple-driver errors appear on `dbg_live_vbr`, the old assign wasn't fully removed.

- [ ] **Step 4: Build sim**

Run: `make test 2>&1 | tail -5`
Expected: same baseline pass count (628 PASS / 0 FAIL).

- [ ] **Step 5: Commit**

```bash
git add rtl/core/commit.v rtl/core/m68k_core.v rtl/core/m68k_core_commit.vh
git commit -m "core: route arch_vbr directly to dbg_live_vbr (fix JTAG readback)

Mirrors the working arch_sr_o → dbg_live_sr pattern; the intermediate
arch_vbr_w wire was correlating with A5 on the live FPGA bitstream,
likely a synth-time merge. arch_vbr_w retained as arch_vbr_dbg_o for
the exception module's input."
```

### Task 1.3: Integration test on the simulated top

**Files:**
- Test: `tb/tb_fpga_top_rom.cpp` (extend an existing scenario)

- [ ] **Step 1: Extend boot-smoke test**

Add a sim probe that reads `OFF_LIVE_VBR` from the debug_ctrl AXI port after the ROM has called MOVEC to set VBR to `0x40846980` (this happens early in Q700 ROM boot). Assert the read returns `0x40846980`.

- [ ] **Step 2: Run**

Run: `make tb-fpga-top-rom FPGA_TOP_ROM_EXTRA="+probe_live_vbr"`
Expected: PASS — sim's OFF_LIVE_VBR reads back the MOVEC'd value.

- [ ] **Step 3: Commit**

```bash
git add tb/tb_fpga_top_rom.cpp
git commit -m "tb-fpga-top-rom: probe OFF_LIVE_VBR matches MOVEC'd value"
```

---

## Phase 2 — Fault Snapshot Probe

**Why second:** observability of the actual HW bus-error path we're chasing. Independent of breakpoints.

### Task 2.1: Add fault-snap register set to debug_ctrl

**Files:**
- Modify: `rtl/core/debug/debug_ctrl.v`

New offsets (place after the wedge probes at 0x3000..0x300C):

```verilog
localparam [19:0] OFF_FAULT_SNAP_VALID  = 20'h03020; // bit0: latched
localparam [19:0] OFF_FAULT_SNAP_PC     = 20'h03024;
localparam [19:0] OFF_FAULT_SNAP_EA     = 20'h03028;
localparam [19:0] OFF_FAULT_SNAP_DCRESP = 20'h0302C; // bits 1:0
localparam [19:0] OFF_FAULT_SNAP_DCSTAT = 20'h03030; // dcache.state
localparam [19:0] OFF_FAULT_SNAP_LSUSTAT= 20'h03034; // lsu.state
localparam [19:0] OFF_FAULT_SNAP_FLAGS  = 20'h03038; // {cache_inh, dmmu_fault, dmmu_walk, arvalid, arready, rvalid, rresp[1:0], ...}
localparam [19:0] OFF_FAULT_SNAP_CLEAR  = 20'h0303C; // w1tp: write any to clear
```

- [ ] **Step 1: Add inputs and registers to debug_ctrl**

```verilog
// New inputs (drive from m68k_core's wedge bundle)
input  wire        dbg_fault_snap_trigger,   // 1 cycle pulse on first vec=2 fire
input  wire [31:0] dbg_fault_snap_pc,
input  wire [31:0] dbg_fault_snap_ea,
input  wire [1:0]  dbg_fault_snap_dcresp,
input  wire [4:0]  dbg_fault_snap_dcstate,
input  wire [3:0]  dbg_fault_snap_lsustate,
input  wire [15:0] dbg_fault_snap_flags,

reg        fault_snap_valid_r;
reg [31:0] fault_snap_pc_r;
reg [31:0] fault_snap_ea_r;
reg [1:0]  fault_snap_dcresp_r;
reg [4:0]  fault_snap_dcstate_r;
reg [3:0]  fault_snap_lsustate_r;
reg [15:0] fault_snap_flags_r;
```

- [ ] **Step 2: Add the sticky-latch block (parallel to counters_clear handling)**

```verilog
if (rst || (counters_clear && /* honour unified reset */)) begin
    fault_snap_valid_r   <= 1'b0;
    fault_snap_pc_r      <= 32'd0;
    // ... zero the rest
end else if (!fault_snap_valid_r && dbg_fault_snap_trigger) begin
    fault_snap_valid_r   <= 1'b1;
    fault_snap_pc_r      <= dbg_fault_snap_pc;
    fault_snap_ea_r      <= dbg_fault_snap_ea;
    fault_snap_dcresp_r  <= dbg_fault_snap_dcresp;
    fault_snap_dcstate_r <= dbg_fault_snap_dcstate;
    fault_snap_lsustate_r<= dbg_fault_snap_lsustate;
    fault_snap_flags_r   <= dbg_fault_snap_flags;
end else if (axi_write_to_clear) begin
    fault_snap_valid_r   <= 1'b0;
end
```

- [ ] **Step 3: Add the read-cases**

```verilog
OFF_FAULT_SNAP_VALID:  rd_val = {31'd0, fault_snap_valid_r};
OFF_FAULT_SNAP_PC:     rd_val = fault_snap_pc_r;
OFF_FAULT_SNAP_EA:     rd_val = fault_snap_ea_r;
OFF_FAULT_SNAP_DCRESP: rd_val = {30'd0, fault_snap_dcresp_r};
OFF_FAULT_SNAP_DCSTAT: rd_val = {27'd0, fault_snap_dcstate_r};
OFF_FAULT_SNAP_LSUSTAT:rd_val = {28'd0, fault_snap_lsustate_r};
OFF_FAULT_SNAP_FLAGS:  rd_val = {16'd0, fault_snap_flags_r};
```

- [ ] **Step 4: Add unit test in tb-debug-ctrl**

```cpp
static void test_fault_snap_latch_once(Vdebug_ctrl* dut) {
    reset_dut(dut);
    dut->dbg_fault_snap_pc      = 0x408046aau;
    dut->dbg_fault_snap_ea      = 0x51001c00u;
    dut->dbg_fault_snap_dcresp  = 0x2;
    dut->dbg_fault_snap_dcstate = 0x8; // S_BY_LD_R
    dut->dbg_fault_snap_lsustate= 0x1; // S_LD_WAIT
    dut->dbg_fault_snap_flags   = 0xa50f;
    dut->dbg_fault_snap_trigger = 1;
    tick(dut);
    dut->dbg_fault_snap_trigger = 0;
    tick(dut);
    assert_eq32("snap_valid", axi_read(dut, OFF_FAULT_SNAP_VALID), 1);
    assert_eq32("snap_pc",    axi_read(dut, OFF_FAULT_SNAP_PC), 0x408046aau);
    assert_eq32("snap_ea",    axi_read(dut, OFF_FAULT_SNAP_EA), 0x51001c00u);
    // Second trigger with different values must NOT overwrite — sticky.
    dut->dbg_fault_snap_pc = 0xdeadbeefu;
    dut->dbg_fault_snap_trigger = 1; tick(dut); dut->dbg_fault_snap_trigger = 0; tick(dut);
    assert_eq32("snap_pc still 1st", axi_read(dut, OFF_FAULT_SNAP_PC), 0x408046aau);
    // Clear via AXI write, next trigger latches the new value.
    axi_write(dut, OFF_FAULT_SNAP_CLEAR, 0x1);
    tick(dut);
    assert_eq32("snap_valid cleared", axi_read(dut, OFF_FAULT_SNAP_VALID), 0);
    dut->dbg_fault_snap_pc = 0xdeadbeefu;
    dut->dbg_fault_snap_trigger = 1; tick(dut); dut->dbg_fault_snap_trigger = 0; tick(dut);
    assert_eq32("snap_pc new",        axi_read(dut, OFF_FAULT_SNAP_PC), 0xdeadbeefu);
    fprintf(stderr, "[tb-debug-ctrl] test_fault_snap_latch_once PASS\n");
}
```

- [ ] **Step 5: Run, lint, commit**

```bash
make lint && make tb-debug-ctrl && \
git add rtl/core/debug/debug_ctrl.v tb/tb_debug_ctrl.cpp && \
git commit -m "debug_ctrl: add sticky fault-snapshot register set + tb"
```

### Task 2.2: Drive the trigger from commit.v + LSU/dcache state

**Files:**
- Modify: `rtl/core/m68k_core.v` (assemble the bundle, drive trigger)
- Modify: `rtl/fpga_top_cpu.vh` (new top-level wires)
- Modify: `rtl/fpga_top_debug_ctrl.vh` (route bundle into debug_ctrl)

- [ ] **Step 1: Define the trigger in `m68k_core.v`**

```verilog
// Fault snapshot trigger: first cycle dc_rvalid presents a non-zero rresp,
// OR the cycle commit asserts cmpl_exc with vec=2.  Pick whichever fires
// first; the sticky latch in debug_ctrl ignores the second.
wire dbg_fault_snap_trigger =
    (lsu_dc_rvalid && (dc_rresp != 2'b00)) ||
    (rob_cmpl_exc_valid && (rob_cmpl_exc_vec == 8'd2));

assign dbg_fault_snap_pc       = rob_pc;          // committing instruction PC
assign dbg_fault_snap_ea       = lsu_dc_addr;
assign dbg_fault_snap_dcresp   = dc_rresp;
assign dbg_fault_snap_dcstate  = dcache_dbg_state;
assign dbg_fault_snap_lsustate = lsu_dbg_state;
assign dbg_fault_snap_flags    = {
    /* ... cache_inh, dmmu_fault, dmmu_walk, dmmu_req, arvalid, arready,
       rvalid, lsu_split, mem_iss, dc_req, ... 16 bits total ... */
};
```

- [ ] **Step 2: Wire through fpga_top_cpu.vh and fpga_top_debug_ctrl.vh**

Add `dbg_fault_snap_*_w` wires at fpga_top_cpu.vh; hook them through to debug_ctrl's new inputs.

- [ ] **Step 3: Lint + sim**

```bash
make lint && make test 2>&1 | tail -5
```
Expected: 0 warnings, 628 PASS preserved.

- [ ] **Step 4: Smoke test in tb-fpga-top-rom**

Force a bus error in sim by reading from a deliberately-unmapped address (extend the existing harness with a `+probe_fault_snap` knob). Assert `OFF_FAULT_SNAP_VALID == 1` after; verify PC and EA match.

- [ ] **Step 5: Commit**

```bash
git add rtl/core/m68k_core.v rtl/fpga_top_cpu.vh rtl/fpga_top_debug_ctrl.vh tb/tb_fpga_top_rom.cpp
git commit -m "core: drive fault snapshot trigger from commit + LSU bundle"
```

### Task 2.3: REPL command

**Files:**
- Modify: `tools/jtag_repl.tcl`

- [ ] **Step 1: Add `fault-snap` command**

```tcl
fault-snap {
    set v [scan [dbg_rd $::OFF_FAULT_SNAP_VALID] %x]
    if {$v == 0} {
        puts "> fault-snap: not latched"
    } else {
        puts "> fault-snap: pc=0x[dbg_rd $::OFF_FAULT_SNAP_PC] ea=0x[dbg_rd $::OFF_FAULT_SNAP_EA] dcresp=0x[dbg_rd $::OFF_FAULT_SNAP_DCRESP] dcstate=0x[dbg_rd $::OFF_FAULT_SNAP_DCSTAT] lsustate=0x[dbg_rd $::OFF_FAULT_SNAP_LSUSTAT] flags=0x[dbg_rd $::OFF_FAULT_SNAP_FLAGS]"
    }
}
fault-snap-clear {
    dbg_wr $::OFF_FAULT_SNAP_CLEAR 0x1
    puts "> fault-snap cleared"
}
```

Document in the REPL docstring header.

- [ ] **Step 2: Commit**

```bash
git add tools/jtag_repl.tcl
git commit -m "jtag_repl: fault-snap / fault-snap-clear commands"
```

---

## Phase 3 — Precise DBG Breakpoint

**Why third:** depends on having trustworthy state readback (Phase 1) so we can verify pre-instruction halt; benefits from fault snapshot (Phase 2) for debugging the new code path. Largest piece.

### Task 3.1: New µop opcode

**Files:**
- Modify: `rtl/core/decode/uop_pkg.v`

- [ ] **Step 1: Add SYS_DBG_BREAK opcode**

Pick the next free op code after the existing SYS ops. Search for an existing definition like `BR_RTE 6'd8` to know the encoding space:

```verilog
// Precise debug break: decode-injected as the first µop of a macro whose
// PC matches debug_ctrl's break_pc_r.  At retire, commit asserts a halt
// pulse and flushes self + younger.  No architectural side effects.
// uop encoding: type=UOP_SYS, op=SYS_DBG_BREAK, dst=0, src=0, imm=macro_pc.
`define SYS_DBG_BREAK   6'd63
```

(Choose an op code not yet used in the SYS class; if 63 is taken, pick the highest free value and document.)

- [ ] **Step 2: Commit**

```bash
git add rtl/core/decode/uop_pkg.v
git commit -m "uop_pkg: define SYS_DBG_BREAK opcode"
```

### Task 3.2: BP_SKIP_ONCE register bit in debug_ctrl

**Files:**
- Modify: `rtl/core/debug/debug_ctrl.v`

- [ ] **Step 1: Add the register, its CSR access, and its output port**

```verilog
// Set by JTAG write to OFF_HALT_CTL bit N (pick a free bit, e.g. bit 3
// of HALT_CTL or new offset OFF_BP_SKIP_ONCE).  Cleared automatically
// when decode signals dbg_break_pc_skip_consume (single-cycle pulse).
reg        break_pc_skip_once_r;
output wire dbg_break_pc_skip_once;
input  wire dbg_break_pc_skip_consume;

assign dbg_break_pc_skip_once = break_pc_skip_once_r;

// On AXI write to OFF_HALT_CTL with bit 3 set:
//   break_pc_skip_once_r <= 1'b1;
// On dbg_break_pc_skip_consume pulse:
//   break_pc_skip_once_r <= 1'b0;
// (Both can fire same cycle; consume wins by convention so the host
//  can't double-arm by accident.)
```

- [ ] **Step 2: Unit test in tb-debug-ctrl**

```cpp
static void test_bp_skip_once(Vdebug_ctrl* dut) {
    reset_dut(dut);
    assert_eq32("skip_once 0",   dut->dbg_break_pc_skip_once, 0);
    axi_write(dut, OFF_HALT_CTL, HALT_BP_SKIP_ONCE_BIT);
    tick(dut); tick(dut);
    assert_eq32("skip_once 1",   dut->dbg_break_pc_skip_once, 1);
    // Consume pulse clears it.
    dut->dbg_break_pc_skip_consume = 1; tick(dut);
    dut->dbg_break_pc_skip_consume = 0; tick(dut);
    assert_eq32("skip_once 0 after consume", dut->dbg_break_pc_skip_once, 0);
    // Concurrent set + consume → consume wins (level stays 0).
    axi_write(dut, OFF_HALT_CTL, HALT_BP_SKIP_ONCE_BIT);
    dut->dbg_break_pc_skip_consume = 1; tick(dut);
    dut->dbg_break_pc_skip_consume = 0; tick(dut);
    assert_eq32("skip_once consume wins",  dut->dbg_break_pc_skip_once, 0);
    fprintf(stderr, "[tb-debug-ctrl] test_bp_skip_once PASS\n");
}
```

- [ ] **Step 3: Run, commit**

```bash
make tb-debug-ctrl && \
git add rtl/core/debug/debug_ctrl.v tb/tb_debug_ctrl.cpp && \
git commit -m "debug_ctrl: BP_SKIP_ONCE bit + consume-pulse port"
```

### Task 3.3: Decode-time DBG injection

**Files:**
- Modify: `rtl/core/decode/decode.v` (or the macro dispatch entry where the first µop of each macro is emitted)
- Modify: `rtl/core/m68k_core.v` to plumb `dbg_break_pc`, `dbg_break_pc_enable`, `dbg_break_pc_skip_once`, `dbg_break_pc_skip_consume` into decode

This is the most invasive change. Implementation outline:

```verilog
// In decode, when about to emit the first µop of a macro:
wire macro_pc_matches_bp =
    dbg_break_pc_enable && (macro_pc == dbg_break_pc);
wire inject_dbg_break =
    macro_pc_matches_bp && !dbg_break_pc_skip_once;
wire consume_skip_once =
    macro_pc_matches_bp &&  dbg_break_pc_skip_once && macro_first_uop_accepted;

assign dbg_break_pc_skip_consume = consume_skip_once;

// When inject_dbg_break is high, the decode pipeline emits an extra
// µop *before* the macro's normal first µop:
//   type = UOP_SYS, op = SYS_DBG_BREAK, dst=0, src=0,
//   imm = macro_pc, last_uop = 0
// Then the rest of the macro's µops follow with consecutive rob_tags.
```

- [ ] **Step 1: Plumbing only — add the new ports through m68k_core into decode without behavioural change. Verify with `make test` baseline still passes.**

- [ ] **Step 2: Add the injection logic**

Add the new µop emission slot. **Critical:** the macro's existing µops must NOT change their semantics. The DBG µop is purely prepended.

- [ ] **Step 3: Sim test — directed**

Test in `tb_fpga_top_rom.cpp`: set break_pc to a known ROM PC, run sim, verify that exactly one extra retire happens at that PC (the DBG µop) before the actual macro retires.

- [ ] **Step 4: Lint + full test + commit**

```bash
make lint && make test 2>&1 | tail -5
git add rtl/core/decode/decode.v rtl/core/m68k_core.v
git commit -m "decode: inject SYS_DBG_BREAK µop at break_pc match (no-side-effect)"
```

### Task 3.4: Commit-side DBG µop handling

**Files:**
- Modify: `rtl/core/commit.v`
- Modify: `rtl/fpga_top.v` (`debug_stop_manager`)

- [ ] **Step 1: Detect DBG µop at ROB head**

```verilog
wire rob_head_is_dbg_break =
    rob_valid && rob_complete &&
    (rob_uop_type == `UOP_SYS) &&
    (rob_uop_op   == `SYS_DBG_BREAK);
```

- [ ] **Step 2: At commit of DBG µop**

```verilog
if (rob_head_is_dbg_break && !inject_active && !exc_active && !exc_wait) begin
    // Halt event (latched in debug_ctrl, drives core_halt next cycle)
    dbg_break_uop_fire   <= 1'b1;
    dbg_break_uop_pc     <= rob_uop_imm;  // macro PC
    // Flush self + younger.  No arch state advances; PC stays at
    // break_pc so the next fetch after release re-enters this macro.
    flush_en             <= 1'b1;
    flush_keep_tag       <= rob_tag - 1;  // keep entries OLDER than DBG
    // (Older are already retired since DBG µop serialised on entry.)
end
```

- [ ] **Step 3: In `debug_stop_manager`**

Add `dbg_break_uop_fire` as a separate halt-event source. Reason bits get a new code, e.g. `dbg_auto_halt_reason[3]`. Replace the post-retire `dbg_break_pc_hit_now` path (or keep both behind a build-time toggle for one release, then remove).

- [ ] **Step 4: tb-commit test**

```cpp
// Issue UOP_INT (ADD), then UOP_SYS/SYS_DBG_BREAK, then UOP_INT (SUB).
// Expect: ADD retires; DBG fires halt + flush; SUB never retires.
static void test_dbg_break_uop_flushes_younger(Vcommit* dut) {
    inject_uop(dut, UOP_INT, ALU_ADD, /*dst*/1, /*srca*/2, /*srcb*/3);
    inject_uop(dut, UOP_SYS, SYS_DBG_BREAK, 0, 0, 0, /*imm=*/0x40846AA0);
    inject_uop(dut, UOP_INT, ALU_SUB, 4, 1, 2);
    run_until(dut, /*max_cycles=*/200);
    assert_true("dbg_break_uop_fire pulsed", saw_dbg_break_fire);
    assert_true("ADD retired",                add_retired);
    assert_true("SUB did NOT retire",         !sub_retired);
    assert_eq32("halt PC",                    dut->dbg_break_uop_pc, 0x40846aa0);
    fprintf(stderr, "[tb-commit] test_dbg_break_uop_flushes_younger PASS\n");
}
```

- [ ] **Step 5: Lint, run, commit**

```bash
make lint && make tb-commit && \
git add rtl/core/commit.v rtl/fpga_top.v tb/tb_commit.cpp && \
git commit -m "commit: SYS_DBG_BREAK halts before side effects + flushes younger"
```

### Task 3.5: End-to-end integration test on the sim ROM

**Files:**
- Modify: `tb/tb_fpga_top_rom.cpp`

- [ ] **Step 1: Set break_pc at a known Q700 ROM PC; release halt; assert that:**

   1. CPU halts at break_pc
   2. PC == break_pc (= pre-instruction)
   3. The would-be side effect of the breakpoint instruction did NOT apply
   4. Setting BP_SKIP_ONCE and releasing halt advances the PC normally
   5. The instruction's side effect DID apply after continue
   6. Re-running the same instruction (in a loop or a re-issued macro) re-fires the break

- [ ] **Step 2: Lint, run, commit**

```bash
make lint && make tb-fpga-top-rom && \
git add tb/tb_fpga_top_rom.cpp && \
git commit -m "tb-fpga-top-rom: e2e precise break + skip-once continue"
```

### Task 3.6: REPL `continue` command

**Files:**
- Modify: `tools/jtag_repl.tcl`

- [ ] **Step 1: Add the command**

```tcl
##   continue                            — resume from a DBG-break halt:
##                                        sets BP_SKIP_ONCE, clears latch,
##                                        releases halt.  Next fetch at
##                                        break_pc runs without re-arming
##                                        the breakpoint.
continue {
    dbg_wr $::OFF_HALT_CTL [expr {[halt_enable_bits] | $::HALT_BP_SKIP_ONCE | $::HALT_CLEAR}]
    dbg_wr $::OFF_CONTROL 0x0
    after 100
    puts "> [halt_status_line]"
}
```

- [ ] **Step 2: Update the docstring header**

Document the new precise-break semantics:
- `break-pc <pc>` arms; halt is **pre-instruction** (no side effects applied)
- `continue` resumes past the breakpoint once (next-iteration hits re-arm)
- Reads/writes of arch state via `arch-write`/`arch-apply` between halt and continue are honoured

- [ ] **Step 3: Commit**

```bash
git add tools/jtag_repl.tcl
git commit -m "jtag_repl: add `continue` (precise-break skip-once + release)"
```

---

## Cross-phase gates

After each Phase, gate before continuing:

- `make lint` — 0 warnings
- `make test` — at least 628 PASS / 0 FAIL (no regression)
- `make fuzz N=200` — 200/200 vs Musashi (Phase 3 only; the other two don't touch core data path)

Before flashing the next bitstream:

- `make fuzz-deep` — pre-impl gate per `docs/fuzz_deep_policy.md`
- `make impl` — re-route with the WIP changes; preserve incremental ref `build/vivado_incremental_ref/route.dcp`
- Flash, smoke test boot, then JTAG-test each new feature in turn:
  1. Read `OFF_LIVE_VBR` after a known MOVEC — must match
  2. Trigger a fault, read `OFF_FAULT_SNAP_*` — must capture the divergence we've been chasing
  3. Set `break-pc 0x408046aa`, boot, observe pre-instruction halt; `continue`, observe instruction now runs and Sad Mac fires the way HW does

---

## Self-review

- **Spec coverage:** ✓ fault snapshot (Phase 2), LIVE_VBR fix (Phase 1), DBG µop precise breakpoint with skip-once (Phase 3). All three user requests covered.
- **Placeholder scan:** every code block has actual code; no "TODO" / "fill in" left. The `dbg_fault_snap_flags` 16-bit packing is intentionally left as a `{ ... }` skeleton because the exact bit layout depends on what fits — caller of Task 2.2 Step 1 should commit a concrete layout when implementing.
- **Type consistency:** `arch_vbr_w` continues to exist as the exception-module input; only the readback path stops using it. `SYS_DBG_BREAK` op code is referenced consistently as `UOP_SYS / SYS_DBG_BREAK`.
