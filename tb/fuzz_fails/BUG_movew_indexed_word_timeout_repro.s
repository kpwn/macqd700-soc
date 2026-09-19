| BUG_movew_indexed_word_timeout_repro.s -- word-index brief-extension hang

    .text
    .org 0

_start:
    lea     0x00100000, %a0
    move.l  #0x12340000, %d1
    move.l  #0x00010002, %d0

    move.w  #0x1234, 0x00100014
    move.w  #0xBEEF, 0x00110014

    move.w  (0x12,%a0,%d0.w), %d1
    cmp.l   #0x12341234, %d1
    bne     _fail

_pass:
    lea     0xFFFF0000, %a3
    move.l  #0xC0FFEE00, %d3
    move.l  %d3, (%a3)
_halt:
    bra     _halt

_fail:
    lea     0xFFFF0000, %a3
    move.l  #0xDEADBEEF, %d3
    move.l  %d3, (%a3)
    bra     _fail
