| via1_t1_irq.s — arm VIA1 Timer-1 IRQ, take level-1 autovector, RTE.
|
| Reset state has VIA1 ORB[3] = 1 → low-RAM window aliases ROM.  Drop
| the overlay before writing the autovector so the write reaches DRAM
| instead of being silently dropped against ROM.  Pre-fix the test
| wrote the vector first; T1 then fired against post-reset garbage at
| PC=0x64 and the CPU never reached the sentinel.

    .text
    .org 0

_start:
    moveq   #0, %d7
    move.l  #0x00002000, %a7
    lea     0x50000000, %a0
    | Drop the low-RAM ROM overlay first — see header comment above.
    move.b  #0x08, (%a0)              | ORB[3] = 1 (matches reset)
    move.b  #0x08, 0x0400(%a0)        | DDRB[3] = output
    clr.b   (%a0)                     | ORB[3] = 0 → overlay cleared
    move.l  #_handler, 0x64           | vec 25 = level-1 autovector
    move.w  #0x2000, %sr              | supervisor, IRQ mask = 0
    move.b  #0xC0, 0x1C00(%a0)        | IER set T1 enable
    move.b  #0x04, 0x0800(%a0)        | T1CL latch low
    clr.b   0x0A00(%a0)               | T1CH latch high + start
_wait:
    tst.b   %d7
    beq.s   _wait
    move.l  #0xC0FFEE00, 0xFFFF0000

_handler:
    move.b  0x0800(%a0), %d0          | read T1CL -> clear IFR[T1]
    addq.b  #1, %d7
    rte
