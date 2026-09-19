| vbl_storm_msp_drift.s — Drive many vec-25 (level-1 autovec / VBL) IRQs in
| succession and confirm A7 (= MSP/SSP in supervisor) returns to initial
| value after each RTE.
|
| Motivation: HW Q700 boot shows the SSP drifts by -4 bytes across the
| ~4000 IRQs taken during ROM POST.  Each IRQ entry+RTE should be A7-
| neutral; if there's a per-IRQ leak it shows up as monotonic drift.
| Directed RTE tests (rte_format_0_simple, irq_during_rte) pass because
| they only fire ONE IRQ.  This test fires MANY.
|
| Design:
|   - At entry, install vec 25 handler in low-RAM vector table (VBR=0).
|   - Init SSP at 0x000FE000.
|   - Vec 25 handler mimics Q700 ROM 0x40809B60: MOVEM push, dummy work,
|     MOVEM pop, RTE.
|   - Main loop reads A7, cmp against INIT_SSP.  If diverged → FAIL with
|     drifted A7 written to sentinel.  Else, check IRQ count vs target;
|     hit target → PASS.
|
| Testbench: drives many +ipl=N:1 events in vbl_storm_msp_drift.args.
|
| PASS sentinel: 0xC0FFEE00 at 0xFFFF0000.
| FAIL: writes drifted A7 value to 0xFFFF0000.

    .text
    .org 0

    .equ INIT_SSP,        0x000FE000
    .equ PASS_SENT,       0xFFFF0000
    .equ IRQ_COUNT_ADDR,  0x000F0000
    .equ TARGET_IRQS,     50
    .equ VEC25_ADDR,      0x00000064   | byte offset for vec 25

_start:
    | Confirm supervisor.  SR = 0x2700 → switch IPL to 0 so IRQs can fire.
    move.w  #0x2700, %sr           | sup, IPL=7 (mask all) during setup

    | Set SSP to known initial value.
    move.l  #INIT_SSP, %sp

    | Install vec 25 handler in low-RAM vector table (VBR defaults to 0).
    move.l  #_irq_handler, VEC25_ADDR

    | Zero the IRQ counter.
    clr.l   IRQ_COUNT_ADDR

    | Now drop IPL to 0 so IRQs can fire.
    move.w  #0x2000, %sr           | S=1, IPL=0, no flags

_main_loop:
    | Read A7 into D0.
    move.l  %sp, %d0
    | Compare against expected initial SSP.
    cmp.l   #INIT_SSP, %d0
    bne     _fail_drift            | A7 drifted from initial → FAIL.

    | Check IRQ counter.
    move.l  IRQ_COUNT_ADDR, %d1
    cmp.l   #TARGET_IRQS, %d1
    blt     _main_loop             | Not enough IRQs yet, keep looping.

    | Hit TARGET_IRQS with A7 stable → PASS.
    move.l  #0xC0FFEE00, PASS_SENT
    bra     .                      | Spin for testbench poll.

_fail_drift:
    | Write the actual drifted A7 value (in D0) to sentinel.
    move.l  %d0, PASS_SENT
    bra     .

| ── vec 25 IRQ handler ──────────────────────────────────────────────
| Mimics Q700 ROM handler at 0x40809B60 — including the nested JSR
| structure that the ROM uses.  Each IRQ does:
|   1. MOVEM.L D0-D3/A0-A3, -(A7)   [push 8 regs = 32 bytes]
|   2. JSR _sub_a                   [push 4-byte ret addr]
|   3. JSR _sub_b                   [push 4-byte ret addr]
|   4. MOVEM.L (A7)+, D0-D3/A0-A3   [pop 8 regs]
|   5. RTE                          [pop format-0 frame = 8 bytes]
| Net A7 change should be 0 if both push/pop pairs and RTE are correct.
_irq_handler:
    movem.l %d0-%d3/%a0-%a3, -(%sp)

    | Bump IRQ counter.  Use D4 (not in MOVEM regs).
    move.l  IRQ_COUNT_ADDR, %d4
    addq.l  #1, %d4
    move.l  %d4, IRQ_COUNT_ADDR

    | DEBUG: write marker showing handler ran.
    move.l  #0xDEADC0DE, 0x000F0010

    | Nested JSR pattern like the Q700 handler.
    jsr     _sub_a
    jsr     _sub_b

    movem.l (%sp)+, %d0-%d3/%a0-%a3
    rte

_sub_a:
    | Subroutine A — multi-level recursion + reg saves on local stack.
    movem.l %d0-%d7, -(%sp)   | push 8 longs
    move.l  #0x12345678, %d0
    move.l  #0x87654321, %d1
    | Inner JSR chain to test JSR/RTS balance under deeper nesting.
    jsr     _sub_inner_1
    jsr     _sub_inner_2
    | Memory writes to test store-buffer + drain on RTS.
    move.l  %d0, 0x000F1000
    move.l  %d1, 0x000F1004
    move.l  %d2, 0x000F1008
    movem.l (%sp)+, %d0-%d7   | pop 8 longs
    rts

_sub_b:
    | Subroutine B — A6-based frame + LINK/UNLK.
    link    %a6, #-16          | allocate 16-byte frame on stack
    move.l  #0xDEADBEEF, -4(%a6)
    move.l  #0xCAFEF00D, -8(%a6)
    move.l  -4(%a6), %d6       | read back
    move.l  -8(%a6), %d7
    unlk    %a6
    rts

_sub_inner_1:
    | Innermost — does memory accesses + return.
    move.l  #0x11111111, %d2
    rts

_sub_inner_2:
    | Innermost — call yet another inner.
    jsr     _sub_inner_3
    move.l  #0x22222222, %d3
    rts

_sub_inner_3:
    nop
    nop
    rts
