| dafb_regs.s — program DAFB base/stride/bpp and read base back.

    .text
    .org 0

_start:
    lea     0xF9800000, %a0
    move.l  #0x00000100, 0x0008(%a0)
    move.l  #0x0000061E, 0x000C(%a0)
    move.l  #0x00000030, 0x0010(%a0)
    move.l  0x0008(%a0), %d0
    cmpi.l  #0x00000100, %d0
    beq.s   _pass
    move.l  #0xDEADBEEF, 0xFFFF0000
_pass:
    move.l  #0xC0FFEE00, 0xFFFF0000
