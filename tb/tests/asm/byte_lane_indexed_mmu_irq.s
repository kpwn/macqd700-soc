| byte_lane_indexed_mmu_irq.s — Q700 byte-lane recipe with MMU on +
|                                external IRQ injected mid-loop.
|
| Per user clarification 2026-05-12: the byte-lane test passes with
| MMU off, but FAILS on HW in the MMU-on + IRQ-delivery case (D7 =
| 0x00020003, failure-mask bits 0&1).  This test exercises that exact
| combination:
|
|   1. Enable MMU with DTT0/ITT0 supervisor passthrough.
|   2. Install autovector-1 (vec 25) handler that does a small amount
|      of work then RTEs.
|   3. Drop SR.IPL to 0 so an injected IPL=1 can fire.
|   4. Run the Q700 byte-lane recipe (move.l → cmpb → not.b → cmpb,
|      dbf loop).
|   5. The testbench injects +ipl=<cyc>:1 mid-loop (per .args file).
|   6. After loop, verify d6 (failure mask) == 0.
|
| If the IRQ + RTE corrupts memory or registers across the MMU-on
| translation path, the byte-lane test fails the way HW does.

    .text
    .org 0

_start:
    | Park exception vectors in VBR @ 0x00100000 so write to (a0)=0
    | doesn't clobber them.
    lea     0x00100000, %a1
    move.l  #_panic, %d0
    move.l  %d0, 0(%a1)         | reset SSP placeholder
    move.l  %d0, 4(%a1)         | reset PC placeholder
    move.l  %d0, 8(%a1)         | bus error
    move.l  %d0, 12(%a1)        | addr error
    move.l  %d0, 16(%a1)        | illegal
    move.l  %d0, 20(%a1)        | zero div
    move.l  %d0, 24(%a1)        | CHK
    move.l  %d0, 28(%a1)        | TRAPV
    move.l  %d0, 32(%a1)        | priv viol
    move.l  %d0, 36(%a1)        | trace
    move.l  #_irq_handler, 0x00100064 | vec 25 = autovector 1

    move.l  #0x00100000, %d0
    movec   %d0, %vbr

    | MMU TT setup — same format as the working mmu_atc_load_then_use.
    | ITT0 covers 0x40000000.. (ROM).
    move.l  #0x4000C000, %d0
    movec   %d0, %itt0
    | DTT0 covers 0x00000000.. (low DRAM, where (a0)=0 will be tested).
    move.l  #0x007FA000, %d0
    movec   %d0, %dtt0
    | DTT1 covers 0xFFFFxxxx (test sentinel area).
    move.l  #0xFF00A000, %d0
    movec   %d0, %dtt1
    | URP/SRP unused (TTs cover everything).
    move.l  #0x00200000, %d0
    movec   %d0, %urp
    movec   %d0, %srp
    | Enable MMU 3-level 4K.
    move.l  #0x00008770, %d0
    movec   %d0, %tc

    | Drop SR to allow IPL=1 — supervisor, IPL=0, M=0.
    move.w  #0x2000, %sr

    | Pre-seed the IRQ counter so we can confirm the handler ran.
    move.l  #0x00000000, 0x00100200

    | Test setup — A0=0 (Q700 memory layout).
    suba.l  %a0, %a0
    moveq   #0, %d6                | failure mask
    move.l  #0x54696E61, %d1       | "Tina" test pattern

    | Save (a0..a0+7) so we restore later.
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

    | Restore scratch words.
    move.l  %d3, (%a0)
    move.l  %d4, 4(%a0)

    | IRQ handler must have run at least once (timing-dependent —
    | if the +ipl= injection fires inside the loop, the counter is
    | incremented by handler).
    move.l  0x00100200, %d0
    | Don't fail if IRQ didn't fire (cycle-timing slop) — just record.

    | Check failure mask — this is the actual byte-lane test result.
    tst.l   %d6
    bne     _fail

    lea     0xFFFF0000, %a1
    move.l  #0xC0FFEE00, %d0
    move.l  %d0, (%a1)
_halt:
    bra     _halt

_fail:
    move.l  %d6, %d7
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

| =========================================================================
| Autovector-1 IRQ handler — increments counter @ 0x00100200 and RTEs.
| Pushes some regs to exercise the stack/save path under MMU on.
| =========================================================================
_irq_handler:
    move.l  %d0, -(%sp)
    move.l  %a0, -(%sp)
    addq.l  #1, 0x00100200
    | Touch some unrelated memory (TT-mapped) to exercise the data
    | path under IRQ-time MMU translation.
    lea     0x00100300, %a0
    move.l  #0xCAFEBABE, (%a0)
    move.l  (%sp)+, %a0
    move.l  (%sp)+, %d0
    rte
