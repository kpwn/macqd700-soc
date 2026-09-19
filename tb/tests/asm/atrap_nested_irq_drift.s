| atrap_nested_irq_drift.s — Nested A-line dispatcher with IRQ injection.
|
| Closer model of the Q700 boot scenario: the toolbox runs MANY A-line
| traps, often NESTED (= toolbox call A calls toolbox B which calls C).
| The dispatcher exits each level via ADDQ #4, A7; RTS.  IRQs fire
| concurrently from the testbench via +ipl=cycle:1 events.
|
| Difference from atrap_addq_rts_irq_drift.s: that test had ONE A-line
| opcode in a tight loop with a flat dispatcher.  This test has THREE
| LEVELS of nested A-line traps within the mainline iteration:
|   mainline iter:
|     A-line opcode 1 → dispatcher L1 → handler L1 calls A-line opcode 2
|                                       → dispatcher L2 → handler L2
|                                           calls A-line opcode 3
|                                           → dispatcher L3 → handler L3
|                                           returns
|                                       returns
|                                   returns
|
| With IRQs landing at varied phases, this exercises:
|   - IRQ during nested A-line dispatcher push (= MOVEM-push window)
|   - IRQ during nested A-line dispatcher work (= JSR/RTS chain inside)
|   - IRQ during nested A-line dispatcher exit (= MOVEM-pop / ADDQ / RTS)
|   - IRQ between two outer A-line returns (= stack 3 deep, popping)
|
| If our HW MSP-drift bug is triggered by IRQ-mid-dispatcher-pop with
| specific stack depth or specific cycle offset, this test maximises
| coverage of those windows.
|
| PASS: A7 stays exactly INIT_SSP after N iterations.
| FAIL: writes drifted A7 to sentinel.

    .text
    .org 0

    .equ INIT_SSP,        0x000FE000
    .equ PASS_SENT,       0xFFFF0000
    .equ ITER_COUNT_ADDR, 0x000F0000
    .equ TARGET_ITERS,    500

_start:
    move.w  #0x2700, %sr                   | sup, mask IRQs during setup
    move.l  #INIT_SSP, %sp
    move.l  #_aline_dispatcher_L1, 0x00000028  | vec 10 (A-line)
    move.l  #_irq_handler,          0x00000064  | vec 25 (autovec L1)
    clr.l   ITER_COUNT_ADDR

    | Drop IPL so IRQs can fire.
    move.w  #0x2000, %sr

_iter_loop:
    | Check A7 stable before each iter.
    move.l  %sp, %d0
    cmp.l   #INIT_SSP, %d0
    bne     _fail_drift

    | Tight A-line burst — 4 traps per iter.  Each fires the dispatcher
    | which calls handler_L1 → nested 0xA9F0 → handler_inner → JSR L2/L3
    | chain.  Net A7 per trap = 0; CPU spends most time inside dispatcher.
    .short  0xA001
    .short  0xA001
    .short  0xA001
    .short  0xA001

_after_outer:
    addq.l  #1, ITER_COUNT_ADDR
    move.l  ITER_COUNT_ADDR, %d0
    cmp.l   #TARGET_ITERS, %d0
    blt     _iter_loop

    | All iters done with A7 stable → PASS.
    move.l  #0xC0FFEE00, PASS_SENT
    bra     .

_fail_drift:
    move.l  %d0, PASS_SENT
    bra     .

| ── A-trap dispatcher: Q700 ADDQ+RTS pattern with opcode-based routing.
| Mimics Mac OS Toolbox dispatcher behavior: reads the A-line opcode
| from the saved PC, dispatches to a handler based on the opcode value.
| 0xA001 → outer handler (= handler_L1)
| 0xA9F0 → inner handler (= handler_inner, called from handler_L1's
|         nested A-line trap)
_aline_dispatcher_L1:
    movem.l %d1-%d2/%a1-%a2, -(%sp)        | push 16 bytes
    movea.l 18(%sp), %a2                    | A2 = saved PC (= A-line opcode addr)
    move.w  (%a2), %d1                      | D1 = opcode (= 0xA001 or 0xA9F0)
    addq.l  #2, %a2                         | advance past opcode
    move.l  %a2, 20(%sp)                    | overlay PC slot

    | Route based on opcode.
    cmp.w   #0xA001, %d1
    beq.s   _dispatch_outer
    | else 0xA9F0 → inner.
    jsr     _handler_inner
    bra.s   _dispatch_exit
_dispatch_outer:
    jsr     _handler_L1
_dispatch_exit:
    movem.l (%sp)+, %d1-%d2/%a1-%a2
    addq.w  #4, %a7
    rts

_handler_L1:
    move.l  %d3, -(%sp)
    move.l  %a3, -(%sp)
    | Fire a NESTED A-line trap (0xA9F0).  This re-enters the dispatcher
    | which routes the 0xA9F0 opcode to _handler_inner, NOT back to
    | _handler_L1.  Safe nesting (no infinite recursion).
    .short  0xA9F0
    | Then call _handler_L2 via plain JSR (= JSR chain pressure).
    jsr     _handler_L2
    move.l  (%sp)+, %a3
    move.l  (%sp)+, %d3
    rts

_handler_inner:
    | Inner handler called by dispatcher for opcode 0xA9F0.
    move.l  %d6, -(%sp)
    move.l  %a6, -(%sp)
    moveq   #7, %d6
    lea     0x000F2000, %a6
    move.l  %d6, (%a6)
    move.l  (%sp)+, %a6
    move.l  (%sp)+, %d6
    rts

_handler_L2:
    move.l  %d4, -(%sp)
    move.l  %a4, -(%sp)
    move.l  #0x12345678, %d4
    lea     0x000F1000, %a4
    move.l  %d4, (%a4)
    jsr     _handler_L3
    move.l  (%sp)+, %a4
    move.l  (%sp)+, %d4
    rts

_handler_L3:
    move.l  %d5, -(%sp)
    move.l  %a5, -(%sp)
    moveq   #3, %d5
    move.l  %d5, (%a5)
    nop
    nop
    nop
    move.l  (%sp)+, %a5
    move.l  (%sp)+, %d5
    rts

| ── IRQ handler ───────────────────────────────────────────────────────
_irq_handler:
    movem.l %d0-%d3/%a0-%a3, -(%sp)
    nop
    nop
    movem.l (%sp)+, %d0-%d3/%a0-%a3
    rte
