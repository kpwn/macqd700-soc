| iwm_probe.s — probe the live RTL SWIM/IWM stub through the platform bus.

    .text
    .org 0

_start:
    lea     0x5001E000, %a0
    move.b  0x1A00(%a0), %d0         | reg D selects IWM status
    cmpi.b  #0x80, %d0
    bne.s   _fail
    move.b  #0x17, 0x1E00(%a0)       | reg F writes IWM mode after select
    move.b  0x1C00(%a0), %d0         | reg E reads selected status
    cmpi.b  #0x97, %d0
    beq.s   _pass
_fail:
    move.l  #0xDEADBEEF, 0xFFFF0000
_pass:
    move.l  #0xC0FFEE00, 0xFFFF0000
