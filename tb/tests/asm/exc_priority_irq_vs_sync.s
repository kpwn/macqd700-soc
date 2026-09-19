| exc_priority_irq_vs_sync.s — sync exception wins over pending IRQ.
|
| Goal: a TRAP #0 (synchronous, in-flight) is dispatched as the next
| in-flight micro-op decoder commits, while an IRQ at IPL>0 is
| concurrently asserted.  Per 68040 UM §8.1.1 ("Exception Priority"),
| the sync (instruction-generated) exception wins for its own boundary
| — the TRAP frame is pushed FIRST, then the IRQ frame is pushed on
| top of the TRAP handler's stack pointer.  Hence:
|   1. TRAP #0 handler runs first — visible by ordering check.
|   2. IRQ handler runs second — visible by counter == 2.
|   3. After both RTE: control resumes after TRAP #0 in mainline.
|
| Construction:
|   - vec 32 (TRAP #0) handler writes 1 to ORDER, increments COUNTER.
|   - vec 25 (level-1 auto-vector IRQ) handler writes 2 to ORDER if
|     ORDER==1 else writes 0xFF.  Increments COUNTER.
|
| HARNESS NEEDS: testbench must inject IPL=1 in the same cycle window
| as the TRAP retire.  Without that, the IRQ never fires and only
| TRAP runs, so this test today verifies only the SYNC path.  The
| FAIL paths still help diagnose harness behaviour.
|
| PASS sentinel: 0xC0FFEE00 if (ORDER==2 && COUNTER==2) — both fired
|                              in correct order.
| FAIL sentinels:
|   0xDEAD0B01 — ORDER==1 but COUNTER==1 (IRQ never injected; sync only)
|   0xDEAD0B02 — ORDER==0xFF (TRAP+IRQ wrong order — IRQ first)
|   0xDEAD0B03 — TRAP handler did not fire

    .text
    .org 0

    .equ PASS_SENT, 0xFFFF0000
    .equ ORDER,     0x00000500
    .equ COUNTER,   0x00000504

_start:
    lea     0x00010000, %a7
    move.l  #0, ORDER.l
    move.l  #0, COUNTER.l
    move.l  #_trap_h, 0x00000080         | vec 32 (TRAP #0)
    move.l  #_irq_h,  0x00000064         | vec 25 (autovec lvl 1)

    | SR with IPL=0 so any IPL>=1 fires.
    move.w  #0x2000, %sr

    | Trigger.
    trap    #0

    | After both handlers RTE, check.
    | Per 68040 PRM §8.5.4, an IRQ is taken AT THE COMPLETION of the
    | next instruction after the boundary where IRQ becomes eligible,
    | not by squashing that instruction.  The TRAP RTE re-armed IRQ
    | eligibility, so the first instruction after RTE retires fully
    | BEFORE the IRQ takes — meaning if we read COUNTER directly here,
    | we'd see pre-IRQ COUNTER (=1).  Insert four NOPs so the IRQ
    | fires after one of THEM, and the COUNTER/ORDER reads observe the
    | post-IRQ state.  (Pre-fix the core squashed the in-flight load
    | on IRQ entry, masking this sequencing — see Bug B / docs/take_irq_rethink.md.)
    nop
    nop
    nop
    nop
    move.l  COUNTER.l, %d0
    move.l  ORDER.l, %d1
    cmp.l   #2, %d0
    bne     _check_partial
    cmp.l   #2, %d1
    bne     _fail_order

    | PASS — both fired, sync first.
    lea     PASS_SENT, %a1
    move.l  #0xC0FFEE00, %d2
    move.l  %d2, (%a1)
_halt:
    bra     _halt

_check_partial:
    | If counter==1 and order==1: only TRAP ran (IRQ never injected).
    | This is the today-default; flag DEAD0B01 for the harness to see.
    cmp.l   #1, %d0
    bne     _fail_no_trap
    cmp.l   #1, %d1
    bne     _fail_order
    lea     PASS_SENT, %a1
    move.l  #0xDEAD0B01, %d2
    move.l  %d2, (%a1)
_halt_partial:
    bra     _halt_partial

_fail_order:
    lea     PASS_SENT, %a1
    move.l  #0xDEAD0B02, %d2
    move.l  %d2, (%a1)
_halt_fo:
    bra     _halt_fo

_fail_no_trap:
    lea     PASS_SENT, %a1
    move.l  #0xDEAD0B03, %d2
    move.l  %d2, (%a1)
_halt_nt:
    bra     _halt_nt

_trap_h:
    | Sync exception ran first.
    addq.l  #1, COUNTER.l
    move.l  ORDER.l, %d0
    cmp.l   #0, %d0
    bne     _trap_h_wrong
    move.l  #1, ORDER.l
    rte
_trap_h_wrong:
    | TRAP fired AFTER IRQ — that's the wrong-order failure.
    move.l  #0xFF, ORDER.l
    rte

_irq_h:
    | IRQ second.  Increment counter; if ORDER==1 record 2; else 0xFF.
    addq.l  #1, COUNTER.l
    move.l  ORDER.l, %d0
    cmp.l   #1, %d0
    bne     _irq_h_wrong
    move.l  #2, ORDER.l
    rte
_irq_h_wrong:
    move.l  #0xFF, ORDER.l
    rte
