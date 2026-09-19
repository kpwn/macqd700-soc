| irq_epilogue_movem_odd_sp.s -- IRQ handler epilogue MOVEM from SP%4==2.
|
| Live FPGA wedge shape:
|   40809b84: movem.l (sp)+,d0-d3/a0-a3
|   40809b88: rte
| with A7 word-aligned but not long-aligned.  movem_postinc_odd_sp.s
| covers the instruction in isolation; this covers the real exception path:
| IRQ entry leaves SSP%4==2, handler saves/restores the ROM register set,
| then RTE returns to mainline.
|
| PASS: 0xC0FFEE00 at 0xFFFF0000.
| FAIL: 0xDEAD0001 if the IRQ never arrived.

    .text
    .org 0

    .equ PASS_SENT, 0xFFFF0000
    .equ IRQ_COUNT, 0x00000500

_start:
    move.l  #0, IRQ_COUNT.l

    | Use a supervisor stack that is even but not long-aligned.  Format-0
    | IRQ entry pushes an even-sized frame, and the ROM-style MOVEM prologue
    | pushes 8 longs, so the handler epilogue still restores from SP%4==2.
    lea     0x00020002, %a7

    move.l  #0x00010000, %d0
    movec   %d0, %vbr
    move.l  #_irq_handler, 0x00010064    | vec 25, level-1 autovector

    | Drop IPL so the testbench's +ipl=...:1 pulse can fire.
    move.w  #0x2000, %sr

    | Leave enough retire boundaries for the injected IRQ to arrive, run the
    | handler, and return.
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop

    move.l  IRQ_COUNT.l, %d0
    cmp.l   #1, %d0
    bne     _fail_no_irq

    lea     PASS_SENT, %a0
    move.l  #0xC0FFEE00, %d0
    move.l  %d0, (%a0)
_halt:
    bra     _halt

_fail_no_irq:
    lea     PASS_SENT, %a0
    move.l  #0xDEAD0001, %d0
    move.l  %d0, (%a0)
_halt_fail:
    bra     _halt_fail

_irq_handler:
    movem.l %d0-%d3/%a0-%a3, -(%a7)
    addq.l  #1, IRQ_COUNT.l
    movem.l (%a7)+, %d0-%d3/%a0-%a3
    rte
