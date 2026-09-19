| via1_overlay.s — clear VIA1 overlay via ORB[3] and read ORA back.
|
| Q700 board strap: VIA1 PA pins read 0xC1 in input mode (matches the
| pa_in (8'hC1) wired in tb/tb_cold_boot.v + fpga_top_peripherals.vh,
| which mirrors the Universal ROM's expected strap descriptor).  After
| clearing ORB[3] (= dropping the low-RAM ROM overlay), ORA must still
| return that strap value because we never set DDRA — DDRA reset = 0
| (all input), so ORA reads pa_in directly.

    .text
    .org 0

_start:
    lea     0x50000000, %a0
    move.b  #0x08, (%a0)             | preload ORB[3] high while DDRB[3]=input
    move.b  #0x08, 0x0400(%a0)       | DDRB[3] = output
    clr.b   (%a0)                    | ORB[3] = 0 -> overlay clear
    move.b  0x0200(%a0), %d0         | ORA reads strap pa_in value
    cmp.b   #0xC1, %d0
    beq.s   _pass
    move.l  #0xDEADBEEF, 0xFFFF0000
_pass:
    move.l  #0xC0FFEE00, 0xFFFF0000
