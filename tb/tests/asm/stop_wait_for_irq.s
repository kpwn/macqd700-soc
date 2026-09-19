| stop_wait_for_irq.s — STOP halts execution until an IRQ arrives
| (audit bug #4).
|
| Per 68040 PRM §4.183: `STOP #imm` is a privileged instruction that
|   1. loads SR from the immediate operand,
|   2. halts the processor (no further instructions retire),
|   3. waits until an IRQ at level > new SR.IPL (or NMI) arrives,
|   4. then takes that interrupt's exception entry as usual.
|
| Our current implementation (`decode_uop_assemble.v:15504-15517`):
|   * decodes STOP as UOP_SYS / SYS_NOP,
|   * consumes the opcode + immediate (len_bytes=4),
|   * sets requires_supervisor=1 (so user-mode STOP traps to vec 8),
|   * but DOES NOT load SR from the immediate,
|   * and DOES NOT pause execution — the next instruction retires
|     normally on the cycle after STOP.
|
| Result: any code that uses STOP as a low-power wait-for-IRQ idle
| (Mac OS Process Manager idle, ROM "halt-on-fail" diagnostic
| paths) charges right past STOP into whatever follows.
|
| ─────────────────────────────────────────────────────────────────────
| What this test does
| ─────────────────────────────────────────────────────────────────────
| 1. Set vec 25 (autovec lvl 1) handler.
| 2. Initialise SR to (S=1, IPL=7) so no IRQ can fire yet.
| 3. Pre-mark a "STOP_HIT" sentinel that the handler will check.
| 4. Execute STOP #0x2000 — this is supposed to:
|       (a) load SR = 0x2000 (S=1, IPL=0)
|       (b) wait for next IRQ.
| 5. Testbench injects IPL=1 some cycles after the STOP retires.
| 6. Vector 25 handler runs, sets a "HANDLER_RAN" flag, RTE.
| 7. Mainline resumes after STOP, validates that the dispatched
|    sequence was: STOP retire → STOP halt → IRQ entry →
|    handler → RTE → mainline-after-STOP, in that order.
|
| Key assertion: between STOP retire and IRQ entry there is a
| non-zero gap during which NO instructions retire.  Today,
| because STOP is a NOP, mainline-after-STOP runs IMMEDIATELY
| (often before the testbench has even injected IPL=1) — so
| HANDLER_RAN stays 0 and the test ends with FAIL_NOIRQ.
|
| ─────────────────────────────────────────────────────────────────────
| HARNESS NEEDS: same as irq_during_movem.s.
|
| PASS sentinel: 0xC0FFEE00 (mainline retired AFTER handler).
| FAIL sentinels:
|   0xDEAD0401 — STOP didn't load SR (arch_sr.IPL still 7 after STOP).
|   0xDEAD0402 — IRQ never fired (STOP didn't halt — mainline raced
|                past before testbench injected).
|   0xDEAD0403 — Mainline retired BEFORE handler — wrong ordering.

    .text
    .org 0

    .equ PASS_SENT, 0xFFFF0000
    .equ STACK_BASE, 0x00012000
    .equ HANDLER_RAN, 0x00011000
    .equ POST_STOP_RAN, 0x00011004
    .equ SR_AFTER_STOP, 0x00011008

_start:
    lea     STACK_BASE, %a7
    move.l  #_lvl1_handler, 0x00000064  | vec 25 (autovec lvl 1)

    move.l  #0, HANDLER_RAN
    move.l  #0, POST_STOP_RAN

    | Set IPL=7 so no IRQ can fire until STOP changes the mask.
    move.w  #0x2700, %sr

    | The STOP under test.  Should load SR = 0x2000 (drop mask to 0)
    | and halt until IRQ.  Testbench raises IPL=1 a few hundred
    | cycles after STOP retires.
    stop    #0x2000

    | Mainline after STOP.  Snapshot the live SR to confirm the
    | mask was actually loaded.  Then mark POST_STOP_RAN and check
    | ordering invariants.
    move.w  %sr, %d0
    andi.l  #0xFFFF, %d0
    move.l  %d0, SR_AFTER_STOP

    move.l  #1, POST_STOP_RAN

    | Validate.
    | (a) HANDLER_RAN must be 1 — the IRQ handler must have run.
    move.l  HANDLER_RAN, %d1
    cmpi.l  #1, %d1
    bne     _fail_noirq

    | (b) Saved SR (visible only to handler; we capture it there)
    | must equal 0x2000 — proves STOP loaded the new SR.  Today
    | bug #4 means STOP doesn't load the SR, so the saved SR will
    | be 0x2700 (the pre-STOP value).  Check via SR_AFTER_STOP =
    | the live SR snapshot taken AFTER mainline resumes (post-RTE).
    | This snapshot must be 0x2000 + IPL set by handler entry =
    | actually back to 0x2000 because RTE restores the saved SR.
    move.l  SR_AFTER_STOP, %d2
    andi.l  #0xFFFF, %d2
    cmpi.l  #0x2000, %d2
    bne     _fail_no_sr_load

_pass:
    lea     PASS_SENT, %a1
    move.l  #0xC0FFEE00, %d3
    move.l  %d3, (%a1)
_halt:
    bra     _halt

_fail_no_sr_load:
    lea     PASS_SENT, %a1
    move.l  #0xDEAD0401, %d3
    move.l  %d3, (%a1)
_h1:
    bra     _h1

_fail_noirq:
    lea     PASS_SENT, %a1
    move.l  #0xDEAD0402, %d3
    move.l  %d3, (%a1)
_h2:
    bra     _h2

_fail_order:
    lea     PASS_SENT, %a1
    move.l  #0xDEAD0403, %d3
    move.l  %d3, (%a1)
_h3:
    bra     _h3

_lvl1_handler:
    | Sanity: POST_STOP_RAN should be 0 here — mainline-after-STOP
    | hasn't retired yet (because STOP halted).  If it's non-zero,
    | execution raced past STOP, retired the post-STOP code, and
    | only THEN did the IRQ fire — meaning STOP didn't halt.
    move.l  POST_STOP_RAN, %d4
    cmpi.l  #0, %d4
    bne     _handler_fail_order

    move.l  #1, HANDLER_RAN
    rte

_handler_fail_order:
    lea     PASS_SENT, %a1
    move.l  #0xDEAD0403, %d3
    move.l  %d3, (%a1)
_hho:
    bra     _hho
