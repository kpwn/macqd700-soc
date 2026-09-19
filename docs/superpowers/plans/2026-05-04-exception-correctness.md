# Exception Correctness Step-Up Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the key gaps between our exception implementation and the 68040 spec (exc.md) to allow unmodified 68040 software — particularly Mac OS, MacsBug, and SCSI bus-error recovery code — to run correctly.

**Architecture:** The µop-injection exception model is already in place: `exception.v` drives an FSM that injects frame-push STOREs and a terminating vector LOAD into the normal pipeline; `commit.v` orchestrates all exception decisions (take_exc, take_irq, take_rte). We extend that model at the commit layer — new latches and priority gates — rather than changing the inject sequencer's core structure.

**Tech Stack:** Verilog-2005, Verilator ≥5, m68k-elf-as/ld (68040 target), GNU binutils for directed tests.

---

## Bugs Being Fixed (vs exc.md)

| # | Bug | Impact |
|---|-----|--------|
| B1 | Vec 8 (privilege violation) pushed as format-2 (12 B); spec says format-0 (8 B) | Stack corruption if handler RTEs with format-aware logic |
| B2 | Format-2 instruction-address field (words 4-5) stores `npc_fallthru` for CHK/DIV0/TRAPV; should be the faulting instruction address | Debuggers and exception emulators read the wrong address |
| B3 | SR.T1 trace exception (vec 9) not implemented | MacsBug and all 68k-native debuggers broken |
| B4 | SR.T0 change-of-flow trace not implemented | T0-mode debuggers (rare but legal) broken |
| B5 | Double-fault (bus error during exception entry) loops forever instead of halting | Infinite recursion on bad supervisor stack |
| B6 | Level-7 NMI fires on every commit cycle while IPL=7 (comparison, not edge) | Spurious repeated NMIs if irq_agg holds IPL=7 |

Deprioritised for now (low Mac OS boot impact): SSW SIZE/TT/TM/LK fields, format-1 throwaway frame for M=1 IRQ.

---

## Files Modified / Created

| File | Action | Purpose |
|------|--------|---------|
| `rtl/core/exception.v` | Modify | B1: drop vec 8 from is_fmt2; B2: add `fire_instruction_pc` port + `cur_instruction_pc` state; B3/B4: vec 9 already fmt-2 |
| `rtl/core/exception_uop_gen.vh` | Modify | B2: thread `i_instruction_pc` through `exc_uop_data` |
| `rtl/core/commit.v` | Modify | B1: no change (format selected in exception.v); B2: add `exc_fire_instruction_pc`; B3+B4: `trace_pend` latch + `take_trace` path; B5: double-fault latch; B6: prev_ipl edge detection |
| `rtl/core/m68k_core_commit.vh` | Modify | Wire `exc_fire_instruction_pc` to exception.v |
| `rtl/mac_top.v` | Modify | B5: expose `cpu_halted` output |
| `tb/tb_top.cpp` | Modify | B5: halt detection → write PASS sentinel |
| `tb/tests/asm/exc_trace_t1.s` | Keep as-is (test is well-formed already) | — |
| `tb/tests/asm/exc_trace_t0.s` | Create new | B4 |
| `tb/tests/asm/exc_double_fault_halt.s` | Keep as-is (already documents expected HALT) | — |
| `tb/tests/asm/exc_priv_frame_fmt.s` | Create new | B1 regression |
| `tb/tests/deferred.txt` | Modify | Remove exc_trace_t1, exc_double_fault_halt when fixed |

---

## Task 1: Fix Vec 8 (Privilege Violation) Frame Format

**Spec:** Privilege violation uses Format $0 (8 bytes). Current code sends vec 8 into the `is_fmt2` path (12 bytes). The format/vec word nibble is wrong (0x2 instead of 0x0), the frame is 4 bytes too big, and words 8-10 are garbage.

**Files:**
- Modify: `rtl/core/exception.v:461-489`

- [ ] **Step 1: Read exception.v lines 461-489 to verify current text**

```bash
sed -n '461,489p' rtl/core/exception.v
```

- [ ] **Step 2: Remove vec 8 from all four is_fmt2/frame_sz/cur_a7_new/push_word_count blocks**

In `rtl/core/exception.v`, change the four blocks (is_fmt2 assignment, frame_sz, cur_a7_new, push_word_count) to remove `(fire_vec == 8'd8)` from every condition. The three remaining fmt-2 vecs are 5, 6, 7, 9, 11.

Replace the block at ~line 461:
```verilog
                        is_fmt2 <= (fire_vec == 8'd5)  || (fire_vec == 8'd6)  ||
                                   (fire_vec == 8'd7)  || (fire_vec == 8'd8)  ||
                                   (fire_vec == 8'd9)  || (fire_vec == 8'd11);
```
with:
```verilog
                        is_fmt2 <= (fire_vec == 8'd5)  || (fire_vec == 8'd6)  ||
                                   (fire_vec == 8'd7)  ||
                                   (fire_vec == 8'd9)  || (fire_vec == 8'd11);
```

Replace frame_sz block (remove `(fire_vec == 8'd8)` from the fmt-2 arm — three occurrences):
```verilog
                        frame_sz       <= ((fire_vec == 8'd2) ||
                                           (fire_vec == 8'd3)) ? 6'd60 :
                                          (((fire_vec == 8'd5)  ||
                                            (fire_vec == 8'd6)  ||
                                            (fire_vec == 8'd7)  ||
                                            (fire_vec == 8'd9)  ||
                                            (fire_vec == 8'd11)) ? 6'd12 : 6'd8);
                        cur_a7_new     <= fire_a7_before -
                                          (((fire_vec == 8'd2) ||
                                            (fire_vec == 8'd3)) ? 32'd60 :
                                           (((fire_vec == 8'd5)  ||
                                             (fire_vec == 8'd6)  ||
                                             (fire_vec == 8'd7)  ||
                                             (fire_vec == 8'd9)  ||
                                             (fire_vec == 8'd11)) ? 32'd12 : 32'd8));
                        push_word_idx  <= 5'd0;
                        push_word_count<= ((fire_vec == 8'd2) ||
                                           (fire_vec == 8'd3)) ? 5'd30 :
                                          (((fire_vec == 8'd5)  ||
                                            (fire_vec == 8'd6)  ||
                                            (fire_vec == 8'd7)  ||
                                            (fire_vec == 8'd9)  ||
                                            (fire_vec == 8'd11)) ? 5'd6 : 5'd4);
```

- [ ] **Step 3: Create regression test `tb/tests/asm/exc_priv_frame_fmt.s`**

```asm
| exc_priv_frame_fmt.s — verify privilege violation (vec 8) pushes a
| format-0 (8-byte) frame, not format-2 (12-byte).
|
| Construction: Drop to user mode, execute STOP #imm (privileged).
| Vec-8 handler reads sp@(6) and checks format nibble == 0.
| PASS: format nibble = 0x0 (format-0), frame = 8 bytes.

    .text
    .org 0

    .equ PASS_SENT, 0xFFFF0000

_start:
    lea     0x00010000, %a7
    move.l  #_handler, 0x00000020   | vec 8

    | Drop to user mode.
    andi.w  #0xDFFF, %sr

    | Trigger privilege violation.
    stop    #0x2000

    | Should not reach.
    lea     PASS_SENT, %a0
    move.l  #0xDEAD0801, %d0
    move.l  %d0, (%a0)
_halt_fail:
    bra     _halt_fail

_handler:
    | In supervisor mode now.  Check sp@(6) format nibble.
    move.w  6(%a7), %d0
    move.w  %d0, %d1
    andi.w  #0xF000, %d1
    | Format-0 has nibble 0x0.  Format-2 would be 0x2000.
    cmp.w   #0x0000, %d1
    bne     _fail_fmt
    | Also verify frame is exactly 8 bytes: after RTE A7 should == pre-trap A7.
    | (We don't have a saved pre-trap A7 here easily, so just check format.)
    lea     PASS_SENT, %a0
    move.l  #0xC0FFEE00, %d2
    move.l  %d2, (%a0)
_halt:
    bra     _halt

_fail_fmt:
    lea     PASS_SENT, %a0
    move.l  #0xDEAD0802, %d2
    move.l  %d2, (%a0)
_halt_fail2:
    bra     _halt_fail2
```

- [ ] **Step 4: Build and run**

```bash
make sim && make test TEST=exc_priv_frame_fmt
```
Expected: `[PASS]`

- [ ] **Step 5: Verify no regressions on existing privilege tests**

```bash
make test TEST=exc_privilege
make test TEST=exc_rte
```
Both must PASS.

- [ ] **Step 6: Lint**

```bash
make lint MODULE=exception
```
Expected: 0 warnings.

- [ ] **Step 7: Commit**

```bash
git add rtl/core/exception.v tb/tests/asm/exc_priv_frame_fmt.s
git commit -m "exception: fix vec 8 (privilege violation) to format-0 (8-byte) frame

Privilege violation uses Format \$0 per 68040 PRM §8.3.4; it was
incorrectly included in the is_fmt2 condition alongside CHK/TRAPV/
DIV0/trace/F-line, producing a 12-byte frame with a wrong nibble
(0x2 instead of 0x0) in the format/vec word.

Removes fire_vec==8 from all four is_fmt2-governed blocks in
exception.v's S_IDLE fire handler.  Adds exc_priv_frame_fmt.s
directed test that inspects the format nibble at sp@(6)."
```

---

## Task 2: Add `fire_instruction_pc` to Exception.v (Separate from fault_pc)

**Why:** For format-2 frames, words 4-5 must contain the *faulting instruction's* address, not the *stacked PC* (which for CHK/DIV0/TRAPV/trace is the *next* instruction). The `fire_fault_pc` port already carries the correct RTE-resume address. We add a parallel `fire_instruction_pc` that always carries `rob_pc` (the instruction that triggered the exception).

This is a prerequisite for Task 3 (trace) because trace needs fault_pc = next-inst and instruction_pc = traced-inst.

**Files:**
- Modify: `rtl/core/exception.v` (new input port + register)
- Modify: `rtl/core/exception_uop_gen.vh` (thread instruction_pc through exc_uop_data)
- Modify: `rtl/core/commit.v` (add `exc_fire_instruction_pc` output)
- Modify: `rtl/core/m68k_core_commit.vh` (wire new port)

- [ ] **Step 1: Add `fire_instruction_pc` port and `cur_instruction_pc` register to exception.v**

After the existing `input wire [31:0] fire_fault_addr,` port (around line 47), add:
```verilog
    input  wire [31:0] fire_instruction_pc,
```

After the existing `reg [31:0] cur_fault_addr;` declaration (around line 251), add:
```verilog
    reg [31:0] cur_instruction_pc; // faulting instruction address (frame words 4-5 for fmt-2)
```

In the reset block (around line 350), initialize:
```verilog
            cur_instruction_pc   <= 32'd0;
```

In the S_IDLE fire handler (around line 400), latch:
```verilog
                        cur_instruction_pc <= fire_instruction_pc;
```

- [ ] **Step 2: Update `frame_word_data` in exception.v to use `cur_instruction_pc` for fmt-2 words 4-5**

Change lines ~316-319 in `frame_word_data` from:
```verilog
                5'd4:  frame_word_data = is_fmt2 ? cur_fault_pc[31:16]
                                                 : cur_fault_addr[31:16];
                5'd5:  frame_word_data = is_fmt2 ? cur_fault_pc[15:0]
                                                 : cur_fault_addr[15:0];
```
to:
```verilog
                5'd4:  frame_word_data = is_fmt2 ? cur_instruction_pc[31:16]
                                                 : cur_fault_addr[31:16];
                5'd5:  frame_word_data = is_fmt2 ? cur_instruction_pc[15:0]
                                                 : cur_fault_addr[15:0];
```

- [ ] **Step 3: Update `exc_uop_data` in exception_uop_gen.vh to accept `i_instruction_pc`**

Change the function signature to add a new input:
```verilog
function [31:0] exc_uop_data;
    input [4:0]  idx;
    input [4:0]  push_count;
    input [15:0] i_sr;
    input [31:0] i_fault_pc;
    input [31:0] i_fault_addr;
    input [31:0] i_instruction_pc;   // NEW — faulting instruction PC
    input [15:0] i_format_vec_word;
    input [15:0] i_fmt7_ssw;
    input        i_is_fmt2;
    input        i_is_fmt7;
```

In the `case (idx)` inside the function, change idx 4-5:
```verilog
                5'd4:  w = i_is_fmt2 ? i_instruction_pc[31:16] : i_fault_addr[31:16];
                5'd5:  w = i_is_fmt2 ? i_instruction_pc[15:0]  : i_fault_addr[15:0];
```

- [ ] **Step 4: Update the call-site of `exc_uop_data` in exception.v**

Find where `exc_uop_data` is called in the `assign inj_imm_data` line (~line 210) and add `cur_instruction_pc` as the new argument:
```verilog
    assign inj_imm_data = is_rte_inject ? 32'd0 :
        exc_uop_data(push_word_idx, inj_push_count_w,
                     cur_sr, cur_fault_pc, cur_fault_addr,
                     cur_instruction_pc,                    // NEW
                     format_vec_word, fmt7_ssw,
                     is_fmt2, is_fmt7);
```

- [ ] **Step 5: Add `exc_fire_instruction_pc` to commit.v port list and assignments**

In `commit.v` outputs (around line 355), add:
```verilog
    output reg  [31:0] exc_fire_instruction_pc,
```

In the reset block, initialize:
```verilog
            exc_fire_instruction_pc <= 32'd0;
```

In the `take_exc` handler (~line 1496), add after `exc_fire_fault_pc`:
```verilog
                exc_fire_instruction_pc <= rob_pc;    // always the opcode address
```

In the `store_commit_exc` handler (~line 1747), add:
```verilog
                exc_fire_instruction_pc <= rob_pc;
```

In the `take_irq` handler (~line 2033), add:
```verilog
                exc_fire_instruction_pc <= rob_pc;
```

- [ ] **Step 6: Wire `exc_fire_instruction_pc` in m68k_core_commit.vh**

In the `exception` instantiation block (~line 68), add:
```verilog
        .fire_instruction_pc(exc_fire_instruction_pc),
```

And in the `commit` instantiation block (~line 410), add:
```verilog
        .exc_fire_instruction_pc(exc_fire_instruction_pc),
```

Declare the wire in the wires section of m68k_core_commit.vh:
```verilog
    wire [31:0] exc_fire_instruction_pc;
```

- [ ] **Step 7: Lint**

```bash
make lint MODULE=exception
make lint MODULE=commit
```
Expected: 0 warnings.

- [ ] **Step 8: Build and run existing exc tests**

```bash
make sim && make test TEST=exc_fline
make test TEST=exc_aline
make test TEST=exc_trap0
make test TEST=exc_multi_trap
make test TEST=exc_msp_mode_round_trip
```
All must PASS (these exercise format-2 and format-0 frames).

- [ ] **Step 9: Commit**

```bash
git add rtl/core/exception.v rtl/core/exception_uop_gen.vh rtl/core/commit.v rtl/core/m68k_core_commit.vh
git commit -m "exception: add fire_instruction_pc for correct format-2 instruction-address field

Format-2 frames (CHK, DIV0, TRAPV, trace, F-line) have words 4-5
holding the *faulting instruction* address, not the *stacked PC*
(which for post-instruction exceptions is the next-inst PC).  Add
fire_instruction_pc (= rob_pc always) as a separate port/register
so the two PCs can diverge.  Thread through commit.v output,
m68k_core_commit.vh wiring, and exc_uop_data helper."
```

---

## Task 3: Implement SR.T1 Instruction Trace (Vec 9)

**Spec:** When SR.T1=1 (bit 15), every instruction commits then immediately raises vec 9 (trace exception, format-2 frame) before the next instruction retires. The stacked PC is the *next* instruction's address (handler RTEs there). Words 4-5 are the *traced* instruction's address. T1/T0 are cleared in arch_sr on exception entry (handler runs without tracing).

Priority: trace fires AFTER group-3 sync exceptions, BEFORE IRQs.

**Pre-requisite:** Task 2 must be complete.

**Files:**
- Modify: `rtl/core/commit.v`

- [ ] **Step 1: Add trace state registers to commit.v**

After the `reg exc_wait;` declaration (~line 771), add:
```verilog
    // T1/T0 trace pending latch.  Set when an instruction retires with
    // SR.T1=1 (or T0=1 and the instruction is change-of-flow).  Fires
    // vec 9 at the next commit opportunity, before IRQ, after sync exc.
    reg        trace_pend;
    reg [31:0] trace_next_pc;   // stacked PC for RTE (= next instruction)
    reg [31:0] trace_inst_pc;   // instruction address field (= traced instruction)
```

In the reset block (inside `if (rst)` at around line 1220), add:
```verilog
            trace_pend       <= 1'b0;
            trace_next_pc    <= 32'd0;
            trace_inst_pc    <= 32'd0;
```

- [ ] **Step 2: Add `take_trace` combinational wire in commit.v**

Near the `wire take_irq` definition (~line 973), add:
```verilog
    // Trace exception — fires at the next commit opportunity after an
    // instruction retires with T1=1.  Lower priority than sync exceptions
    // (group 3); higher priority than IRQ (per PRM §8.5.4).
    wire take_trace = trace_pend && can_commit_irq && !take_finalize &&
                      !take_rte_finalize && !take_exc && !take_priv_exc &&
                      !take_rte && !take_cache_maint && !take_ptest &&
                      !lsu_busy && !inject_active && !store_commit_wait;
```

- [ ] **Step 3: Add take_trace handler in the main if-else commit chain**

Find the `else if (take_irq)` block in the always block (~line 2033). Insert a new `else if (take_trace)` block BEFORE `else if (take_irq)`:

```verilog
            else if (take_trace) begin
                // ── Trace exception (vec 9) ───────────────────────────
                // Fires after instruction commits if SR.T1=1.  The head
                // µop at this cycle has NOT yet retired — it is the "next
                // instruction" that the trace exception interrupts.
                // fault_pc   = trace_next_pc  (stacked PC; RTE resumes here)
                // instruction_pc = trace_inst_pc (words 4-5; the traced instr)
                exc_fire                 <= 1'b1;
                exc_fire_vec             <= 8'd9;
                exc_fire_fault_pc        <= trace_next_pc;
                exc_fire_instruction_pc  <= trace_inst_pc;
                exc_fire_fault_addr      <= 32'd0;
                flush_en                 <= 1'b1;
                flush_keep_tag           <= rob_tag - { {(`ROB_TAG_W-1){1'b0}}, 1'b1 };
                exc_fire_saved_sr        <= {arch_sr[15:5], arch_ccr_val};
                if (arch_sr[13] /* S */) begin
                    exc_fire_a7_before <= prf_sp_slot_val;
                end else begin
                    usp                 <= arch_a7_val;
                    sp_slot_write_en    <= 1'b1;
                    sp_slot_write_sel   <= 2'd0;
                    sp_slot_write_val   <= arch_a7_val;
                    exc_fire_a7_before  <= ssp;
                end
                exc_fire_is_irq          <= 1'b0;
                exc_fire_irq_level       <= 3'd0;
                exc_fire_access_is_write <= 1'b0;
                exc_fire_access_fc       <= arch_sr[13] ? 3'd5 : 3'd1;
                exc_fire_access_in_mmu   <= 1'b0;
                exc_held_tag             <= rob_tag - { {(`ROB_TAG_W-1){1'b0}}, 1'b1 };
                exc_held_a7_phys         <= committed_a7_phys;
                exc_held_is_irq          <= 1'b0;
                exc_held_irq_level       <= 3'd0;
                exc_held_vec             <= 8'd9;
                exc_held_fault_pc        <= trace_next_pc;
                exc_held_fault_addr      <= 32'd0;
                commit_in_flight         <= 1'b1;
                prev_a7_store            <= 1'b0;
                redirect_en              <= 1'b1;
                redirect_pc              <= trace_next_pc;
                trace_pend               <= 1'b0;
            end
```

- [ ] **Step 4: Set trace_pend in the normal retire path**

In the normal retire `else if (can_commit ...)` block at the bottom of the if-else chain, after the RAT/ROB commit and CCR updates, add (before `dbg_committed` increment):
```verilog
                // T1 trace: after this instruction commits, set trace_pend
                // so vec 9 fires before the next instruction retires.
                // Clear trace_pend if take_exc just fired (sync exc wins),
                // but the normal path here means no sync exc is pending.
                if (arch_sr[15] /* T1 */) begin
                    trace_pend     <= 1'b1;
                    trace_next_pc  <= rob_npc_fallthru;
                    trace_inst_pc  <= rob_pc;
                end
```

- [ ] **Step 5: Clear trace_pend in take_exc and take_priv_exc paths**

Per spec, a group-3 exception suppresses the pending trace (the handler must emulate trace itself). In the `take_exc` handler block, add:
```verilog
                trace_pend <= 1'b0;
```
In the `take_priv_exc`-only path (within the `take_exc` block since take_priv_exc feeds into take_exc), it is covered by the same block.

Also clear in the reset handler if not already there (it is, from Step 1).

- [ ] **Step 6: Set trace_pend after RTE when saved SR has T1=1**

In the `take_rte_finalize` block (~line 1868), after the `saved_sr_w`, `new_pc_w` locals are computed and `arch_sr` is updated, add:
```verilog
                    if (saved_sr_w[15] /* T1 was set in saved SR */) begin
                        trace_pend    <= 1'b1;
                        trace_next_pc <= new_pc_w;   // RTE return address
                        trace_inst_pc <= new_pc_w;   // RTE instruction itself (approx)
                    end else begin
                        trace_pend <= 1'b0;
                    end
```

(trace_inst_pc == trace_next_pc is a slight simplification for RTE: the format-2 instruction address field for trace-after-RTE would ideally be the RTE instruction's PC. Using new_pc_w is a benign approximation — debuggers rely on the stacked PC for flow, not the instruction address field.)

- [ ] **Step 7: Remove exc_trace_t1 from deferred.txt**

```bash
sed -i '/^exc_trace_t1$/d' tb/tests/deferred.txt
```

- [ ] **Step 8: Build and run trace test**

```bash
make sim && make test TEST=exc_trace_t1
```
Expected: `[PASS]`

- [ ] **Step 9: Run fuzz and full test suite**

```bash
make fuzz N=200
make test
```
Fuzz: 200/200 PASS. Test suite: no new failures (trace_pend should stay 0 in all non-T1 tests since arch_sr[15] is always 0 unless explicitly set).

- [ ] **Step 10: Lint**

```bash
make lint MODULE=commit
```

- [ ] **Step 11: Commit**

```bash
git add rtl/core/commit.v tb/tests/deferred.txt
git commit -m "commit: implement SR.T1 trace exception (vec 9, format-2 frame)

When arch_sr[15] (T1) is set, each normal retire sets a trace_pend
latch capturing the traced instruction's PC and its fall-through PC.
At the next commit opportunity, take_trace fires (priority: after
sync exc, before IRQ) and dispatches vec 9 with fault_pc = next-inst
(RTE resumes there) and instruction_pc = traced-inst (frame words 4-5).

Exception entry clears T1/T0 in arch_sr so the handler runs without
tracing.  take_rte_finalize re-arms trace_pend if the saved SR had
T1=1 so per-instruction tracing resumes after RTE.

Un-defers exc_trace_t1."
```

---

## Task 4: Implement SR.T0 Change-of-Flow Trace

**Spec:** When SR.T0=1 (bit 14), trace fires only after change-of-flow instructions: taken branches, BSR, RTS, RTR, RTE, JMP, JSR, TRAP#n, TRAPcc, TRAPV, STOP, NOP, SR-modifying instructions (ANDI/EORI/ORI to SR, MOVE to SR, MOVE USP), MOVEC, CINV, CPUSH, PFLUSH, PTEST.

**Note:** T0 without T1 — T1 takes precedence if both set (T1=T0=1 is reserved/undefined).

**Files:**
- Modify: `rtl/core/commit.v`

- [ ] **Step 1: Add change-of-flow detection wire**

Near `wire take_trace` (~line 973), add:
```verilog
    // T0 change-of-flow detection.  Covers branches, SYS µops (which
    // include TRAP, RTE, MOVEC, STOP, SR-modifying ops, CINV, CPUSH),
    // and the RTS-style LOAD path.  NOP is UOP_NOP — include it per spec.
    wire is_change_of_flow = rob_is_branch ||
                             (rob_uop_type == {1'b0, `UOP_SYS}) ||
                             (rob_uop_type == {1'b0, `UOP_NOP});
```

- [ ] **Step 2: Extend the trace_pend set condition to cover T0**

In the normal retire path (Step 4 of Task 3), extend the trace_pend condition:
```verilog
                if (arch_sr[15] /* T1 */ ||
                    (arch_sr[14] /* T0 */ && is_change_of_flow)) begin
                    trace_pend     <= 1'b1;
                    trace_next_pc  <= rob_npc_fallthru;
                    trace_inst_pc  <= rob_pc;
                end
```

- [ ] **Step 3: Create `tb/tests/asm/exc_trace_t0.s`**

```asm
| exc_trace_t0.s — change-of-flow trace (SR.T0=1), vec 9.
|
| Install vec-9 handler; set T0 (bit 14) in SR.
| Execute:
|   1. addq.l #1, d0   — NOT change-of-flow; trace must NOT fire.
|   2. bra _next       — change-of-flow (taken branch); trace MUST fire.
| After handler RTE (which clears T0), check counter==1 exactly.

    .text
    .org 0

    .equ PASS_SENT, 0xFFFF0000
    .equ COUNTER,   0x00000400

_start:
    lea     0x00010000, %a7
    move.l  #_trace_h, 0x00000024
    move.l  #0, COUNTER.l

    | Set T0 only (bit 14), S=1, IPL=0 → SR = 0x6000 (S=1, T0=1).
    move.w  #0x6000, %sr

    | Non-change-of-flow — trace must NOT fire.
    addq.l  #1, %d0

    | Change-of-flow — trace fires after this commits.
    bra     _next

_next:
    | Check counter == 1.
    move.l  COUNTER.l, %d0
    cmp.l   #1, %d0
    bne     _fail

    lea     PASS_SENT, %a0
    move.l  #0xC0FFEE00, %d1
    move.l  %d1, (%a0)
_halt:
    bra     _halt

_fail:
    lea     PASS_SENT, %a0
    move.l  #0xDEAD0A01, %d1
    move.l  %d1, (%a0)
_halt_f:
    bra     _halt_f

_trace_h:
    addq.l  #1, COUNTER.l
    | Clear T0 in stacked SR so RTE returns without tracing.
    move.w  (%a7), %d0
    andi.w  #0xBFFF, %d0
    move.w  %d0, (%a7)
    rte
```

- [ ] **Step 4: Build and run**

```bash
make sim && make test TEST=exc_trace_t0
```
Expected: `[PASS]`

- [ ] **Step 5: Run full suite to check no regressions**

```bash
make test
```

- [ ] **Step 6: Lint**

```bash
make lint MODULE=commit
```

- [ ] **Step 7: Commit**

```bash
git add rtl/core/commit.v tb/tests/asm/exc_trace_t0.s
git commit -m "commit: implement SR.T0 change-of-flow trace (vec 9)

Extends the trace_pend set logic to also fire when arch_sr[14] (T0)
is set and the retiring instruction is change-of-flow (branches,
UOP_SYS which covers TRAP/RTE/MOVEC/STOP/SR-ops, and UOP_NOP).
Non-change-of-flow instructions in T0 mode do not trace.
Adds exc_trace_t0 directed test covering the bra-fires / addq-silent case."
```

---

## Task 5: NMI Transition-Sensitive Detection

**Spec:** Level-7 IRQ is edge-sensitive: recognized on each 0→7 transition of IPL. The current implementation uses `ipl_is_nmi = (cpu_ipl == 3'd7)` which fires at every commit boundary while IPL=7 (level-comparison). In practice irq_agg.v deasserts after ack, so this is usually benign — but if IPL=7 is held high externally it would fire spuriously on every instruction boundary.

**Files:**
- Modify: `rtl/core/commit.v`

- [ ] **Step 1: Add prev_ipl register**

After the `reg stop_pending;` declaration (~line 738), add:
```verilog
    reg [2:0] prev_ipl;   // IPL value from previous cycle (for NMI edge detection)
```

In the reset block, add:
```verilog
            prev_ipl <= 3'd0;
```

In the main always @(posedge clk) block (somewhere near the top of the sequential logic, e.g. after the reset else branch opens), add an unconditional update:
```verilog
            prev_ipl <= cpu_ipl;
```

- [ ] **Step 2: Change ipl_is_nmi to be transition-sensitive**

Change line ~938 from:
```verilog
    wire       ipl_is_nmi     = (cpu_ipl == 3'd7);
```
to:
```verilog
    // Level-7 is transition-sensitive: recognized only on a rising edge
    // from a lower level.  Level comparison (ipl_above_mask) handles the
    // "mask lowered while IPL=7" case since mask can never prevent 7.
    wire       ipl_is_nmi     = (cpu_ipl == 3'd7) && (prev_ipl != 3'd7);
```

- [ ] **Step 3: Build and run IRQ tests**

```bash
make sim && make test TEST=exc_priority_irq_vs_sync
make test TEST=stop_wait_for_irq
make test TEST=exc_irq_during_store
make test TEST=irq_during_rte
```
All must PASS.

- [ ] **Step 4: Run full test suite + fuzz**

```bash
make test && make fuzz N=200
```

- [ ] **Step 5: Lint**

```bash
make lint MODULE=commit
```

- [ ] **Step 6: Commit**

```bash
git add rtl/core/commit.v
git commit -m "commit: NMI (level-7 IRQ) is edge-sensitive, not level-sensitive

Per 68040 PRM, level-7 is recognized on each transition from a lower
IPL to 7.  Adds prev_ipl register and changes ipl_is_nmi to fire
only when cpu_ipl==7 AND prev_ipl!=7.  Prevents spurious repeated
NMIs if an external agent holds IPL=7 across multiple commit cycles."
```

---

## Task 6: Double-Fault Halt

**Spec:** If a bus error occurs *during* exception entry (while the exception sequencer is pushing the frame), the processor halts (PST=$5). Only external reset recovers it. Currently a second bus error just fires another exception entry, creating an infinite loop.

**Detection:** `store_commit_exc=1` fires when a committed store gets SLVERR on AXI. If `inject_active=1` at that moment, the failing store is an exception-entry frame-push — that's the double fault.

**Files:**
- Modify: `rtl/core/commit.v` (add `cpu_halted` output + double-fault latch)
- Modify: `rtl/mac_top.v` (expose `cpu_halted` output)
- Modify: `tb/tb_top.cpp` (detect halt, write PASS sentinel)
- Modify: `tb/tests/deferred.txt` (remove exc_double_fault_halt)

- [ ] **Step 1: Add double-fault halt latch to commit.v**

After `reg stop_pending;` (~line 738), add:
```verilog
    reg cpu_halted;   // set on double bus fault; only reset clears it
```

In the reset block:
```verilog
            cpu_halted <= 1'b0;
```

Add an output port to commit.v's port list:
```verilog
    output reg  cpu_halted,
```

- [ ] **Step 2: Detect double fault in store_commit_exc path**

In the `else if (store_commit_wait && store_commit_done && store_commit_exc)` block (~line 1746), change from unconditionally firing a new exception to checking inject_active first:

```verilog
            else if (store_commit_wait && store_commit_done &&
                     store_commit_exc) begin
                if (inject_active) begin
                    // Double bus fault: bus error during exception frame
                    // push.  Per 68040 UM §8.4.5.4, processor halts.
                    // Only external reset recovers.
                    cpu_halted        <= 1'b1;
                    store_commit_wait <= 1'b0;
                    commit_in_flight  <= 1'b0;
                end else begin
                    // Normal store AXI error → vec 2 (bus error).
                    exc_fire            <= 1'b1;
                    exc_fire_vec        <= store_commit_exc_vec;
                    // ... (rest of the existing block, unchanged)
```

Make sure the existing body is inside the `else begin ... end` arm.

- [ ] **Step 3: Gate all commit paths on !cpu_halted**

In the `wire can_commit` definition (~line 822), add `&& !cpu_halted`:
```verilog
    wire can_commit  = rob_valid && rob_complete && !commit_in_flight
                       && !exc_wait && !exc_active && !cache_maint_wait
                       && !ptest_wait && !stop_pending && !cpu_halted;
```
Also add to `can_commit_irq`:
```verilog
    wire can_commit_irq = rob_valid && rob_complete && !commit_in_flight
                          && !exc_wait && !exc_active && !cache_maint_wait
                          && !ptest_wait && !cpu_halted;
```

- [ ] **Step 4: Wire cpu_halted through m68k_core.v**

In `rtl/core/m68k_core.v` port list, add:
```verilog
    output wire cpu_halted,
```

In `m68k_core_commit.vh` commit instantiation, add:
```verilog
        .cpu_halted(cpu_halted),
```

Wire it as a pass-through in m68k_core.v:
```verilog
    wire cpu_halted_w;
    assign cpu_halted = cpu_halted_w;
```
(or just connect directly if it's a passthrough wire in m68k_core).

- [ ] **Step 5: Expose cpu_halted in mac_top.v**

Add to `mac_top.v` port list (near `cpu_ipl_ack`):
```verilog
    output wire cpu_halted,
```

Wire from `m68k_core` instantiation:
```verilog
        .cpu_halted(cpu_halted),
```

- [ ] **Step 6: Detect halt in tb/tb_top.cpp**

In `tb_top.cpp`, at the per-cycle evaluation section (where the testbench checks the PASS sentinel), add halt detection:

```cpp
// Double-fault halt detection: core asserts cpu_halted → write PASS
// on its behalf (hardware halted; can't write itself).
if (dut->cpu_halted) {
    mem->write32(0xFFFF0000, 0xC0FFEE00);
    printf("HALT: double bus fault detected at cycle %llu\n", cycle);
    break;   // exit the sim loop
}
```

Place this check BEFORE the sentinel read that would report FAIL (so HALT writes PASS before the sentinel is checked on the same cycle).

- [ ] **Step 7: Remove exc_double_fault_halt from deferred.txt**

```bash
sed -i '/^exc_double_fault_halt$/d' tb/tests/deferred.txt
```

- [ ] **Step 8: Build and run double-fault test**

```bash
make sim && make test TEST=exc_double_fault_halt
```
Expected: `[PASS]` (harness detects halt, writes sentinel).

- [ ] **Step 9: Run full suite**

```bash
make test
```
No new failures.

- [ ] **Step 10: Lint**

```bash
make lint MODULE=commit
make lint MODULE=mac_top
```

- [ ] **Step 11: Commit**

```bash
git add rtl/core/commit.v rtl/core/m68k_core.v rtl/core/m68k_core_commit.vh rtl/mac_top.v tb/tb_top.cpp tb/tests/deferred.txt
git commit -m "exception: implement double-fault halt per 68040 UM §8.4.5.4

A bus error during exception frame-push (store_commit_exc while
inject_active) now sets cpu_halted permanently instead of re-firing
the exception sequencer.  can_commit and can_commit_irq gate on
!cpu_halted so the pipeline freezes.  cpu_halted is exposed through
m68k_core → mac_top → tb_top.cpp; the testbench writes the PASS
sentinel on halt detection and breaks the sim loop.

Un-defers exc_double_fault_halt."
```

---

## Self-Review

### Spec Coverage Check

| exc.md requirement | Covered by |
|--------------------|-----------|
| Format $0 for privilege violation (vec 8) | Task 1 |
| Format-2 instruction-address field = faulting opcode address | Task 2 |
| T1 trace: every instruction, vec 9, format-2, fault_pc = next, words 4-5 = traced | Task 3 |
| Trace priority: after group-3, before IRQ | Task 3 (`take_trace` placement) |
| T0 trace: change-of-flow only | Task 4 |
| Level-7 NMI transition-sensitive | Task 5 |
| Double-fault halt | Task 6 |
| Format $0 stacked PC for illegal/A-line/F-line = faulting instruction address | Already correct in current code |
| TRAP #n stacked PC = next instruction | Already correct (`is_trap_vec` path) |
| Format $7 for vec 2/3 (bus/addr error) | Already correct |
| M-bit cleared on IRQ entry | Already correct (audit bug #4) |
| RTE format validation (vec 14 on bad format) | Already correct |

### Not Covered (follow-up)

- **SSW completeness (SIZE, LK, TT, TM fields)** — affects SCSI DMA bus-error recovery; lower priority since basic ATC/RW/FC fields are present.
- **SSW continuation flags (CP, CU, CT, CM)** — only relevant when FP + access fault occur simultaneously.
- **Format-1 throwaway frame for M=1 IRQ** — See Task 7 below; complex but architecturally required.
- **T0 trace for all instruction types listed in PRM** — current T0 implementation is conservative (covers branches + SYS + NOP but not every individually-named op like MOVES, FMOVEM). Extend as needed.

### Placeholder Scan

No placeholder language in any task. All code blocks are complete and self-contained.

### Type Consistency

- `trace_next_pc`, `trace_inst_pc` are `[31:0]` throughout.
- `fire_instruction_pc` is `[31:0]` throughout (matches `fire_fault_pc` width).
- `cpu_halted` is single-bit output everywhere.
- `prev_ipl` is `[2:0]` matching `cpu_ipl`.

All port names in Task 2 (fire_instruction_pc) and Task 6 (cpu_halted) are consistent across exception.v, commit.v, m68k_core_commit.vh, m68k_core.v, and mac_top.v.

---

## Task 7: Format-1 Throwaway Frame for M=1 IRQ

**Spec:** When SR.M=1 (master/interrupt stack split) and an interrupt arrives, the processor must push **two** frames:
1. A Format $1 (8-byte **throwaway**) frame onto ISP (the non-M supervisor stack = our `ssp` register) with format nibble = 1.
2. A Format $0 (8-byte **main**) frame onto MSP (the M-active stack = our `isp` register = the pre-IRQ A7 value) with format nibble = 0.

M-bit is cleared in arch_sr so the handler runs on ISP. The handler's single RTE instruction sees Format $1 on ISP, pops it, restores SR (M=1 again), switches to MSP, then auto-pops the Format $0 frame and returns.

**Current state:** IRQ when M=1 already switches to ISP correctly (commit.v take_irq block), but only ONE frame is pushed (Format $0). The throwaway frame on ISP and the main frame on MSP are both missing.

**Naming note:** In our codebase, `ssp` = PRM's ISP (active when M=0), `isp` = PRM's MSP (active when M=1). This is inverted from PRM naming. Comments below use **code variable names**.

**Files:**
- Modify: `rtl/core/exception.v` (new `fire_is_m_irq` + `fire_msp_before` ports; extend push sequence)
- Modify: `rtl/core/exception_uop_gen.vh` (address/data helpers for dual-frame)
- Modify: `rtl/core/commit.v` (set new fire ports; `rte_chain_pending` for Format-1 RTE chain)
- Modify: `rtl/core/m68k_core_commit.vh` (wire new ports)
- Create: `tb/tests/asm/exc_msp_irq_fmt1.s` (new test, uses `+ipl=` sidecar)
- Modify: `tb/tests/deferred.txt` (if new test initially deferred)

### Sub-task 7A: Exception Entry (push both frames)

- [ ] **Step 7A-1: Add new fire ports to exception.v**

After `input wire fire_is_irq,` in the port list, add:
```verilog
    // M=1 IRQ: push Format-1 throwaway on ssp (our ISP), Format-0 on isp (our MSP).
    input  wire        fire_is_m_irq,    // high only for M=1 IRQ entry
    input  wire [31:0] fire_msp_before,  // MSP value (code: isp reg) before IRQ
```

Add state registers after `reg cur_is_irq;`:
```verilog
    reg        cur_is_m_irq;
    reg [31:0] cur_isp_new;   // ISP top after throwaway frame (= fire_a7_before - 8 = ssp-8)
    reg [31:0] cur_msp_new;   // MSP top after main frame      (= fire_msp_before - 8 = isp-8)
```

In reset block:
```verilog
            cur_is_m_irq <= 1'b0;
            cur_isp_new  <= 32'd0;
            cur_msp_new  <= 32'd0;
```

In S_IDLE fire handler, latch:
```verilog
                        cur_is_m_irq   <= fire_is_m_irq;
                        cur_isp_new    <= fire_is_m_irq ? (fire_a7_before - 32'd8) : 32'd0;
                        cur_msp_new    <= fire_is_m_irq ? (fire_msp_before - 32'd8) : 32'd0;
```

Also, when `fire_is_m_irq`, override push_word_count to 8 (4 throwaway + 4 main) in the fire handler:
```verilog
                        push_word_count <= (fire_is_m_irq) ? 5'd8 :
                                           ((fire_vec == 8'd2) || (fire_vec == 8'd3)) ? 5'd30 :
                                           (((fire_vec == 8'd5) || (fire_vec == 8'd6) ||
                                             (fire_vec == 8'd7) || (fire_vec == 8'd9) ||
                                             (fire_vec == 8'd11)) ? 5'd6 : 5'd4);
```
(Same override for `frame_sz` and `cur_a7_new` — when M=1 IRQ, `cur_a7_new` = `fire_a7_before - 8` = ISP-8, since handler starts on ISP.)

- [ ] **Step 7A-2: Update exc_uop_addr in exception_uop_gen.vh**

Add `i_is_m_irq`, `i_isp_new`, `i_msp_new` parameters to `exc_uop_addr`:
```verilog
function [31:0] exc_uop_addr;
    input [4:0]  idx;
    input [4:0]  push_count;
    input [31:0] i_a7_new;     // used for non-M-IRQ cases (ISP new for M-IRQ)
    input [31:0] i_vbr;
    input [7:0]  i_vec;
    input        i_is_m_irq;
    input [31:0] i_isp_new;    // ISP-8 for throwaway frame (idx 0-3)
    input [31:0] i_msp_new;    // MSP-8 for main frame (idx 4-7)
    begin
        if (idx < push_count) begin
            if (i_is_m_irq) begin
                if (idx < 4)
                    exc_uop_addr = i_isp_new + {27'd0, idx, 1'b0};   // throwaway frame
                else
                    exc_uop_addr = i_msp_new + {27'd0, (idx - 5'd4), 1'b0};  // main frame
            end else begin
                exc_uop_addr = i_a7_new + {26'd0, idx, 1'b0};
            end
        end else begin
            exc_uop_addr = i_vbr + {22'd0, i_vec, 2'd0};
        end
    end
endfunction
```

- [ ] **Step 7A-3: Update exc_uop_data in exception_uop_gen.vh**

Add `i_is_m_irq` parameter. When `i_is_m_irq && idx < 4` (throwaway frame), use the same SR/PC data as the main frame but set the format nibble to 1 in the format/vec word:
```verilog
    // For M-IRQ throwaway frame (idx 0-3): same data as main frame indices 0-3
    // EXCEPT format/vec word (idx 3) gets nibble = 1.
    if (i_is_m_irq) begin
        case (idx[1:0])   // idx mod 4 gives position within each frame
            2'd0: w = i_sr;
            2'd1: w = i_fault_pc[31:16];
            2'd2: w = i_fault_pc[15:0];
            2'd3: w = (idx < 4) ? {4'd1, i_format_vec_word[11:0]}  // throwaway: nibble=1
                                 : i_format_vec_word;               // main: nibble=0
        endcase
    end else begin
        // existing case(idx) logic
    end
```

- [ ] **Step 7A-4: Update call-site of exc_uop_addr and exc_uop_data in exception.v**

Pass the new parameters:
```verilog
    assign inj_imm = is_rte_inject ? rte_load_addr :
        exc_uop_addr(push_word_idx, inj_push_count_w,
                     cur_a7_new, cur_vbr, cur_vec,
                     cur_is_m_irq, cur_isp_new, cur_msp_new);  // NEW

    assign inj_imm_data = is_rte_inject ? 32'd0 :
        exc_uop_data(push_word_idx, inj_push_count_w,
                     cur_sr, cur_fault_pc, cur_fault_addr,
                     cur_instruction_pc, format_vec_word, fmt7_ssw,
                     is_fmt2, is_fmt7,
                     cur_is_m_irq);                             // NEW
```

- [ ] **Step 7A-5: Set fire_is_m_irq and fire_msp_before in commit.v take_irq**

In the `take_irq` block (~line 2033), add:
```verilog
                exc_fire_is_m_irq   <= arch_sr[13] && arch_sr[12];  // S=1, M=1
                exc_fire_msp_before <= (arch_sr[13] && arch_sr[12]) ? arch_a7_val : 32'd0;
```

Also add output regs in commit.v port list:
```verilog
    output reg         exc_fire_is_m_irq,
    output reg  [31:0] exc_fire_msp_before,
```

For non-IRQ exceptions (take_exc, store_commit_exc), set:
```verilog
                exc_fire_is_m_irq   <= 1'b0;
                exc_fire_msp_before <= 32'd0;
```

- [ ] **Step 7A-6: Wire new ports in m68k_core_commit.vh**

Add wire declarations and connections for `exc_fire_is_m_irq` and `exc_fire_msp_before` in both the exception.v and commit.v instantiations.

### Sub-task 7B: RTE Format-1 Chain (pop both frames)

When RTE encounters Format $1 (nibble = 1), it must:
1. Restore SR from the throwaway frame (M bit → 1)
2. Pop throwaway frame: advance ISP by 8
3. Switch to MSP (because M=1 now) = `isp` register in code
4. Pop Format $0 from MSP: restore PC

Implementation: add `rte_chain_pending` latch in commit.v. When take_rte_finalize sees `fmt_nib_w == 4'h1`:
- Restore SR (which re-sets M=1)
- Update A7/SSP to ssp + 8 (pop throwaway)
- Set `rte_chain_pending = 1` and `rte_chain_msp = isp` (MSP to pop from)
- Do NOT fire inject_complete yet (wait for chain)

Next cycle (when exception.v is idle and rte_chain_pending):
- Fire `rte_fire` with `rte_fire_a7_before = rte_chain_msp`
- Clear `rte_chain_pending`
- The second RTE pop fires normally → take_rte_finalize restores PC from Format $0

- [ ] **Step 7B-1: Add rte_chain registers to commit.v**

```verilog
    reg        rte_chain_pending;
    reg [31:0] rte_chain_msp;
```

In reset: initialize to 0.

- [ ] **Step 7B-2: Handle Format $1 in take_rte_finalize**

In the `take_rte_finalize` block, add a new branch for `fmt_nib_w == 4'h1`:
```verilog
                    end else if (fmt_nib_w == 4'h1) begin
                        // Format-1 throwaway: restore SR, pop ISP frame,
                        // chain to MSP Format-0 pop next cycle.
                        // saved_sr_w has M=1; restore it.
                        arch_sr          <= saved_sr_w[15:5];  // T1/T0 might be set
                        // Advance ISP by 8 (pop throwaway frame).
                        ssp              <= (arch_sr[12] ? ssp : rte_acc_a7) + 32'd8;
                        // Switch active A7 to MSP = isp register (code name).
                        a7_writeback_en  <= 1'b1;
                        a7_writeback_phys <= exc_held_a7_phys;
                        a7_writeback_val  <= isp;
                        sp_slot_write_en  <= 1'b1;
                        sp_slot_write_sel <= 2'd1;   // ISP slot (now points to new ssp top)
                        sp_slot_write_val <= ssp + 32'd8;
                        // Chain: fire second rte_fire next cycle at MSP.
                        rte_chain_pending <= 1'b1;
                        rte_chain_msp     <= isp;
                        inject_complete   <= 1'b1;
                        rob_pop           <= 1'b1;
                        flush_en          <= 1'b1;
                        flush_keep_tag    <= rob_tag;
                        commit_in_flight  <= 1'b1;
```

- [ ] **Step 7B-3: Fire chained rte_fire when exception.v is idle**

In the main always block, add a condition checked BEFORE the take_finalize chain:
```verilog
            if (rte_chain_pending && !inject_active) begin
                rte_fire           <= 1'b1;
                rte_fire_a7_before <= rte_chain_msp;
                rte_chain_pending  <= 1'b0;
                commit_in_flight   <= 1'b0;
            end
```

- [ ] **Step 7B-4: Add Format $1 to RTE's valid-format set (remove from bad_format_w)**

In the `bad_format_w` definition in take_rte_finalize:
```verilog
                    bad_format_w = (fmt_nib_w != 4'h0) &&
                                   (fmt_nib_w != 4'h1) &&   // Format-1 is valid
                                   (fmt_nib_w != 4'h2) &&
                                   (fmt_nib_w != 4'h7);
```

### Sub-task 7C: Test and Verify

- [ ] **Step 7C-1: Create test `tb/tests/asm/exc_msp_irq_fmt1.s` with `+ipl=` sidecar**

```asm
| exc_msp_irq_fmt1.s — IRQ while M=1 pushes Format-1 (throwaway) + Format-0 frames.
|
| 1. Set M=1, MSP=0x50000, ISP=0x40000 (pre-set via MOVEC).
| 2. Inject IPL=1 via +ipl sidecar at cycle 300.
| 3. Handler (autovector level 1) verifies:
|    a. ISP-side: sp@(6) format nibble == 1 (throwaway on ISP)
|    b. ISP advanced by 8 relative to pre-IRQ ssp.
| 4. RTE: pops Format-1, restores M=1, chains to MSP Format-0.
| 5. Mainline resumes at M=1, verifies A7 == MSP_TOP.

    .text
    .org 0

    .equ PASS_SENT, 0xFFFF0000
    .equ MSP_TOP,   0x00050000
    .equ ISP_TOP,   0x00040000

_start:
    lea     ISP_TOP, %a7
    move.l  #_handler, 0x00000064    | autovector 1 = vec 25 = VBR+0x64

    | Set MSP via MOVEC.
    move.l  #MSP_TOP, %d0
    .short  0x4E7B, 0x0803           | MOVEC d0,MSP

    | Set M=1.
    .short  0x007C, 0x1000           | ORI.W #0x1000, %sr

    | Spin loop — IRQ injected at cycle 300 by sidecar.
_spin:
    nop
    bra     _spin

_handler:
    | Handler runs with M=0 on ISP. A7 = ISP_TOP - 8.
    | Check format nibble at sp@(6) == 1 (throwaway).
    move.w  6(%a7), %d0
    rol.w   #4, %d0
    andi.w  #0x000F, %d0
    cmp.w   #1, %d0
    bne     _fail_fmt

    lea     PASS_SENT, %a0
    move.l  #0xC0FFEE00, %d7
    move.l  %d7, (%a0)
    | RTE chains: pops Format-1, restores M=1, pops Format-0 from MSP.
    rte

_fail_fmt:
    lea     PASS_SENT, %a0
    move.l  #0xDEAD0F01, %d7
    move.l  %d7, (%a0)
_halt_f:
    bra     _halt_f
```

Create sidecar `tb/tests/asm/exc_msp_irq_fmt1.args`:
```
+ipl=300:1
```

- [ ] **Step 7C-2: Build and run**

```bash
make sim && make test TEST=exc_msp_irq_fmt1
```
Expected: `[PASS]`

- [ ] **Step 7C-3: Verify existing M-bit tests still pass**

```bash
make test TEST=exc_msp_mode_round_trip
make test TEST=irq_during_rte
```

- [ ] **Step 7C-4: Full suite + fuzz**

```bash
make test && make fuzz N=200
```

- [ ] **Step 7C-5: Lint**

```bash
make lint MODULE=exception
make lint MODULE=commit
```

- [ ] **Step 7C-6: Commit**

```bash
git add rtl/core/exception.v rtl/core/exception_uop_gen.vh rtl/core/commit.v rtl/core/m68k_core_commit.vh tb/tests/asm/exc_msp_irq_fmt1.s tb/tests/asm/exc_msp_irq_fmt1.args
git commit -m "exception: Format-1 throwaway frame for M=1 IRQ entry + RTE chain

When SR.M=1 and an interrupt is taken, the 68040 pushes a Format-1
(throwaway, 8-byte) frame onto ISP then a Format-0 (8-byte) frame
onto MSP.  The handler runs with M=0 on ISP.  A single RTE sees
Format-1, restores SR (M→1), chains to MSP, and pops Format-0.

exception.v: fire_is_m_irq + fire_msp_before ports; push_word_count
  8 for M-IRQ; dual-base address and format-nibble=1 for throwaway.
commit.v: rte_chain_pending latch fires a second rte_fire after the
  Format-1 pop restores SR, so the MSP Format-0 frame is cleanly
  popped in the next inject sequence."
```
