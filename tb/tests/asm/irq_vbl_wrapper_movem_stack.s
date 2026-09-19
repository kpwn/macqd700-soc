| irq_vbl_wrapper_movem_stack.s -- ROM VBL wrapper-shaped MOVEM/JSR check.
|
| The Q700 VBL wrapper does:
|   movem.l d0-d3/a0-a3,-(sp)
|   lea     callee(pc),a3
|   jsr     (a3)
|   movem.l (sp)+,d0-d3/a0-a3
|   rte
|
| PASS: 0xC0FFEE00 at 0xFFFF0000.

    .text
    .org 0

    .equ PASS_SENT, 0xFFFF0000
    .equ STACK_TOP, 0x00180000
    .equ SAVE_SP,   0x00001000

_start:
    lea     STACK_TOP, %a7
    move.l  #_vbl, 0x00000064       | autovector level 1

    move.l  #0x00000000, %d0
    move.l  #0x00000003, %d1
    move.l  #0x4080edb0, %d2
    move.l  #0x00000900, %d3
    move.l  #0xfffbe208, %a0
    move.l  #0x4080a1d0, %a1
    move.l  #0x4082414a, %a2
    move.l  #0x40800000, %a3

    move.w  #0x2000, %sr            | supervisor, unmask level 1
_spin:
    addq.l  #1, %d7
    bra     _spin

_vbl:
    movem.l %d0-%d3/%a0-%a3, -(%a7)
    move.l  %a7, SAVE_SP.l
    lea     _callee, %a3
    jsr     (%a3)
_after_jsr:
    movem.l (%a7)+, %d0-%d3/%a0-%a3
    rte

_callee:
    move.l  %a7, %d6
    move.l  (%a7), %d7
    cmp.l   #_after_jsr, %d7
    bne     _fail

    move.l  SAVE_SP.l, %a4
    cmp.l   #0x00000000, (%a4)+
    bne     _fail
    cmp.l   #0x00000003, (%a4)+
    bne     _fail
    cmp.l   #0x4080edb0, (%a4)+
    bne     _fail
    cmp.l   #0x00000900, (%a4)+
    bne     _fail
    cmp.l   #0xfffbe208, (%a4)+
    bne     _fail
    cmp.l   #0x4080a1d0, (%a4)+
    bne     _fail
    cmp.l   #0x4082414a, (%a4)+
    bne     _fail
    cmp.l   #0x40800000, (%a4)+
    bne     _fail

    lea     PASS_SENT, %a0
    move.l  #0xC0FFEE00, %d0
    move.l  %d0, (%a0)
_halt:
    stop    #0x2700
    bra     _halt

_fail:
    lea     PASS_SENT, %a0
    move.l  #0xDEADBEEF, %d0
    move.l  %d0, (%a0)
_halt_fail:
    stop    #0x2700
    bra     _halt_fail
