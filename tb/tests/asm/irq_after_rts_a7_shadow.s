| irq_after_rts_a7_shadow.s -- pending IRQ after RTS must use post-RTS A7.
|
| Reproduces the Q700 VBL-wrapper race:
|   JSR pushes return PC, dispatcher JMPs to handler, handler RTSes.
| If a level-1 IRQ is already pending when RTS retires, IRQ entry must push
| its frame from the post-RTS A7, not the one-cycle-stale SP shadow.
|
| PASS: 0xC0FFEE00 at 0xFFFF0000.

    .text
    .org 0

    .equ PASS_SENT, 0xFFFF0000
    .equ STACK_TOP, 0x00080000
    .equ IRQ_SP,    0x00011000
    .equ IRQ_COUNT, 0x00011004

_start:
    move.w  #0x2000, %sr
    move.l  #_irq1, 0x00000064
    move.l  #0, IRQ_SP
    move.l  #0, IRQ_COUNT
    lea     STACK_TOP, %a7
    lea     _dispatch, %a3

    move.l  #64, %d7
_loop:
    jsr     (%a3)
    cmp.l   #STACK_TOP, %a7
    bne     _fail_sp_main
    subq.l  #1, %d7
    bne     _loop

    move.l  IRQ_COUNT, %d0
    cmp.l   #0, %d0
    beq     _pass
    move.l  IRQ_SP, %d0
    cmp.l   #0x0007fff8, %d0          | IRQ frame is 8 bytes
    bne     _fail_sp_irq

_pass:
    lea     PASS_SENT, %a0
    move.l  #0xC0FFEE00, %d0
    move.l  %d0, (%a0)
_halt:
    stop    #0x2700
    bra     _halt

_dispatch:
    lea     _leaf, %a0
    jmp     (%a0)

_leaf:
    rts

_irq1:
    move.l  %a7, IRQ_SP
    move.l  IRQ_COUNT, %d0
    addq.l  #1, %d0
    move.l  %d0, IRQ_COUNT
    rte

_fail_sp_main:
    lea     PASS_SENT, %a0
    move.l  #0xDEAD0001, %d0
    move.l  %d0, (%a0)
    bra     _halt

_fail_sp_irq:
    lea     PASS_SENT, %a0
    move.l  #0xDEAD0002, %d0
    move.l  %d0, (%a0)
    bra     _halt
