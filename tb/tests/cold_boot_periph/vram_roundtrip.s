| vram_roundtrip.s — write one 8bpp pixel to VRAM and read it back.

    .text
    .org 0

_start:
    lea     0xF9000000, %a0
    move.b  #0x5A, (%a0)
    move.b  (%a0), %d0
    move.b  %d0, 1(%a0)
    cmpi.b  #0x5A, %d0
    beq.s   _pass
    move.l  #0xDEADBEEF, 0xFFFF0000
_pass:
    move.l  #0xC0FFEE00, 0xFFFF0000
