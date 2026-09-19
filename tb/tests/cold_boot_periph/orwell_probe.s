| orwell_probe.s — Orwell controls reset-value stub must read as zero.

    .text
    .org 0

_start:
    lea     0x5000E000, %a0
    move.b  (%a0), %d0
    tst.b   %d0
    bne.s   _fail
    move.b  #0x5A, (%a0)
    move.b  0x00FC(%a0), %d0
    tst.b   %d0
    beq.s   _pass
_fail:
    move.l  #0xDEADBEEF, 0xFFFF0000
_pass:
    move.l  #0xC0FFEE00, 0xFFFF0000
