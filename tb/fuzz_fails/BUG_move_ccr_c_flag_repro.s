    .text
    .org 0
_start:
    moveq   #1, %d0         | sets N=0 Z=0 V=0 C=0
    subq.l  #2, %d0         | D0 = -1, sets N=1 C=1 (borrow)
    moveq   #0, %d1         | sets N=0 Z=1 V=0 C=0 — should clear C
    move.l  #0xC0FFEE00, %d7
    lea     0xFFFF0000, %a1
    move.l  %d7, (%a1)
_halt:
    bra     _halt
