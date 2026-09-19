| byte_lane_indexed_mmu_irq_storm.s — Q700 byte-lane with MMU on +
|                                      IRQ storm hitting every loop iter
|
| Variant of byte_lane_indexed_mmu_irq.s.  Same setup, but the test
| does MANY loop passes (not just one dbf) so the testbench's +ipl=
| injections land at multiple specific cycles inside the loop —
| catching the IRQ-entry-during-store-buffered window and the
| IRQ-entry-during-cracked-uop window.

    .text
    .org 0

_start:
    lea     0x00100000, %a1
    move.l  #_panic, %d0
    move.l  %d0, 0(%a1)
    move.l  %d0, 4(%a1)
    move.l  %d0, 8(%a1)
    move.l  %d0, 12(%a1)
    move.l  %d0, 16(%a1)
    move.l  %d0, 20(%a1)
    move.l  %d0, 24(%a1)
    move.l  %d0, 28(%a1)
    move.l  %d0, 32(%a1)
    move.l  %d0, 36(%a1)
    move.l  #_irq_handler, 0x00100064
    move.l  #0x00100000, %d0
    movec   %d0, %vbr

    move.l  #0x4000C000, %d0
    movec   %d0, %itt0
    move.l  #0x007FA000, %d0
    movec   %d0, %dtt0
    move.l  #0xFF00A000, %d0
    movec   %d0, %dtt1
    move.l  #0x00200000, %d0
    movec   %d0, %urp
    movec   %d0, %srp
    move.l  #0x00008770, %d0
    movec   %d0, %tc
    move.w  #0x2000, %sr

    | IRQ counter at sentinel address (TT-mapped).
    move.l  #0x00000000, 0x00100200

    | Outer counter — run the byte-lane test 16 times so IRQ storm
    | catches different windows.
    moveq   #15, %d5

.outer:
    | Q700 recipe at A0=0.
    suba.l  %a0, %a0
    moveq   #0, %d6
    move.l  #0x54696E61, %d1
    move.l  (%a0), %d3
    move.l  4(%a0), %d4
    move.l  %d1, (%a0)
    moveq   #3, %d2

.loop:
    move.l  #-1, 4(%a0)
    cmp.b   (0,%a0,%d2:w*1), %d1
    bne     .fail_set
    not.b   %d1
    not.b   (0,%a0,%d2:w*1)
    cmp.b   (0,%a0,%d2:w*1), %d1
    beq     .skip_fail
.fail_set:
    bset    %d2, %d6
.skip_fail:
    ror.l   #8, %d1
    dbra    %d2, .loop

    move.l  %d3, (%a0)
    move.l  %d4, 4(%a0)

    tst.l   %d6
    bne     _fail

    dbra    %d5, .outer

    lea     0xFFFF0000, %a1
    move.l  #0xC0FFEE00, %d0
    move.l  %d0, (%a1)
_halt:
    bra     _halt

_fail:
    | Encode failure: high word = outer-iter index, low word = mask
    move.l  %d5, %d7
    swap    %d7
    or.l    %d6, %d7
    lea     0xFFFF0000, %a1
    move.l  #0xDEADBEEF, %d0
    move.l  %d0, (%a1)
_halt_fail:
    bra     _halt_fail

_panic:
    move.l  #0xFA000099, %d7
    lea     0xFFFF0000, %a1
    move.l  #0xDEADBEEF, %d0
    move.l  %d0, (%a1)
_halt_panic:
    bra     _halt_panic

_irq_handler:
    movem.l %d0-%d1/%a0, -(%sp)
    addq.l  #1, 0x00100200
    | Touch unrelated memory under MMU.
    lea     0x00100300, %a0
    move.l  #0xCAFEBABE, (%a0)
    move.l  4(%a0), %d0
    move.l  %d0, 8(%a0)
    movem.l (%sp)+, %d0-%d1/%a0
    rte
