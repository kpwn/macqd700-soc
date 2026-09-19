| movem_postinc_an_in_list.s -- MOVEM postincrement with the base An in
| the destination register list.
|
| PRM behavior: if the postincrement base register is also loaded by the
| MOVEM, the loaded value remains in An; the final postincrement must not
| clobber it.
|
| PASS: 0xC0FFEE00 at 0xFFFF0000.
| FAIL: 0xDEADBEEF at 0xFFFF0000.

    .text
    .org 0

_start:
    lea     0x00030000, %a0
    move.l  #0x00123456, (%a0)
    move.l  #0x00ABCDEF, 4(%a0)

    movem.l (%a0)+, %a0-%a1

    move.l  %a0, %d0
    cmp.l   #0x00123456, %d0
    bne     _fail
    move.l  %a1, %d0
    cmp.l   #0x00ABCDEF, %d0
    bne     _fail

_pass:
    lea     0xFFFF0000, %a2
    move.l  #0xC0FFEE00, %d0
    move.l  %d0, (%a2)
_halt:
    bra     _halt

_fail:
    lea     0xFFFF0000, %a2
    move.l  #0xDEADBEEF, %d0
    move.l  %d0, (%a2)
_halt_fail:
    bra     _halt_fail
