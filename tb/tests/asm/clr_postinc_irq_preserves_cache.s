| clr_postinc_irq_preserves_cache.s -- queue-init stores survive IRQ entry
|
| This guards the Q700 ROM shape:
|   clr.w   (A1)+
|   clr.l   (A1)+
|   clr.l   (A1)
| followed shortly by a level-1 interrupt.  If exception entry invalidates
| dirty D-cache lines instead of preserving them, the handler observes stale
| 0xff data.
|
| Run manually with an injected level-1 IRQ, e.g.
|   build/sim/Vmac_top +test=clr_postinc_irq_preserves_cache \
|     +timeout=200000 +ipl=800:1
|
| PASS sentinel: 0xC0FFEE00 -> 0xFFFF0000.

    .text
    .org 0

_start:
    move.l  #_irq1, 0x00000064

    lea     0x00103802, %a1
    move.l  #0xffffffff, (%a1)
    move.l  #0xffffffff, 4(%a1)
    move.l  #0xffffffff, 8(%a1)

    clr.w   (%a1)+
    clr.l   (%a1)+
    clr.l   (%a1)

    move.w  #0x2000, %sr
_wait:
    bra     _wait

_irq1:
    move.w  #0x2700, %sr
    cmp.w   #0x0000, 0x00103802
    bne     _fail
    cmp.l   #0x00000000, 0x00103804
    bne     _fail
    cmp.l   #0x00000000, 0x00103808
    bne     _fail
    cmp.w   #0xffff, 0x0010380c
    bne     _fail

_pass:
    lea     0xFFFF0000, %a0
    move.l  #0xC0FFEE00, %d0
    move.l  %d0, (%a0)
_halt:
    bra     _halt

_fail:
    lea     0xFFFF0000, %a0
    move.l  #0xDEADBEEF, %d0
    move.l  %d0, (%a0)
_halt_fail:
    bra     _halt_fail
