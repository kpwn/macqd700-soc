| scsi_no_device.s — probe absent SCSI ID and expect open bus-free reads.

    .text
    .org 0

_start:
    lea     0x5000F000, %a0
    move.b  #0x82, (%a0)              | initiator ID7 + probed target ID1
    move.b  #0x05, 0x0004(%a0)        | SEL | DATA_BUS
    move.b  0x0010(%a0), %d0          | reg4: bus status
    tst.b   %d0
    bne.s   _fail
    move.b  0x0014(%a0), %d0          | reg5: bus and status
    andi.b  #0x54, %d0                | DRQ | IRQ | BUSY_ERROR must stay clear
    beq.s   _pass
_fail:
    move.l  #0xDEADBEEF, 0xFFFF0000
_pass:
    move.l  #0xC0FFEE00, 0xFFFF0000
