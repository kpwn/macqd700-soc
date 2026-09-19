| rom_scc_mmio_btst_dbf_timeout.s -- BTST/DBF loop against Q700 SCC MMIO.
|
| Same shape as the ROM loop at 0x408478b4, but the polled byte is the
| real Q700 SCC mirror address observed on FPGA.  This isolates the
| bring-up hang where D4 stops decrementing after a BTST memory read from
| the peripheral window.

    .text
    .org 0

_start:
    lea     0x00020000, %a7
    lea     0x50f0c022, %a3
    move.w  #0x00ff, %d4
    moveq   #0, %d6

_loop:
    btst    #0, (%a3)
    bne     _done
    dbf     %d4, _loop
    moveq   #3, %d6

_done:
    cmpi.l  #3, %d6
    bne     _fail
    cmpi.l  #0x0000ffff, %d4
    bne     _fail

_pass:
    lea     0xffff0000, %a0
    move.l  #0xc0ffee00, %d0
    move.l  %d0, (%a0)
_halt:
    bra     _halt

_fail:
    lea     0xffff0000, %a0
    move.l  #0xbadc0c22, %d0
    move.l  %d0, (%a0)
_halt_fail:
    bra     _halt_fail
