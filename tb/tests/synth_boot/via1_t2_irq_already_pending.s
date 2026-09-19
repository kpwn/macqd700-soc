| via1_t2_irq_already_pending.s — reproduce the 2026-08-27 real-hardware
| boot-investigation shape: arm VIA1 Timer2's IRQ WHILE SR IS STILL MASKED
| (matching the ROM's own T2CH-arm-before-unmask ordering), let it genuinely
| expire while masked (so IFR is set well before SR ever unmasks -- the
| "already pending" case, not the ordinary fresh-edge-while-running case
| every other via1_t*_irq test exercises), THEN do a few extra inhibited
| MMIO byte accesses (mirroring the SCC WR9..WR1 burst + IFR/IER polling
| that ran in between on real hardware), THEN unmask, THEN busy-wait.
|
| Real hardware: the CPU never took this interrupt -- ran a tight loop to
| its full natural bound with SR unmasked (IPL=0) and VIA1's own IRQ
| summary line continuously asserted the whole time. If that's real (not a
| stale-bitstream or single-boot-run artifact), this test hangs (times out
| in _wait) or diverges from whatever pass/fail signal the harness uses.
| If it passes, the SoC-level VIA1/irq_agg/socket wiring is clean for this
| exact shape and the real bug is elsewhere (stuck FSM from an unrelated
| earlier boot event, or the deployed bitstream not matching its claimed
| build_id).
|
| See tb/tests/cold_boot_periph/via1_t1_irq.s for the base pattern (ROM
| overlay drop, vector install, IER/latch addressing) -- this test
| deliberately reverses its unmask-vs-arm ordering and uses Timer2 instead
| of Timer1 to match the real repro instead of the ordinary case.

    .text
    .org 0

| Mandatory 68k cold-reset vector table (see idle_loop.s's header comment --
| the CPU's own ResetVectorPlugin reads these two long-words off address
| 0/4 for its real reset fetch; omitting this makes the CPU decode this
| program's own opcodes as SSP/PC and free-run into garbage).
_vectors:
    .long   0x00500000     | initial SSP (unused -- _start sets a7 for real)
    .long   _start          | initial PC = ROM_BASE + 8

_start:
    moveq   #0, %d7
    move.l  #0x00002000, %a7
    lea     0x50000000, %a0
    | Drop the low-RAM ROM overlay first (see via1_t1_irq.s header comment).
    move.b  #0x08, (%a0)              | ORB[3] = 1 (matches reset)
    move.b  #0x08, 0x0400(%a0)        | DDRB[3] = output
    clr.b   (%a0)                     | ORB[3] = 0 -> overlay cleared
    move.l  #_handler, 0x64           | vec 25 = level-1 autovector

    | Stay MASKED (boot default, S=1 I=7) while arming Timer2 -- matches
    | the real ROM's T2CH-write-then-IER-enable-then-later-unmask order.
    move.b  #0xA0, 0x1C00(%a0)        | IER: set + enable Timer2 (bit5)
    move.b  #0x08, 0x1000(%a0)        | T2CL latch = 8 (small -> expires fast)
    clr.b   0x1200(%a0)               | T2CH <- 0 -> starts the real countdown

    | Busy-wait, STILL MASKED, long enough for the real VIA1 counter (T2CL=8,
    | T2CH=0 -> 8 phi2 ticks) to genuinely expire and set IFR bit5 -- this is
    | the "already pending before unmask" condition itself, not simulated.
    moveq   #40, %d6
_arm_settle:
    subq.w  #1, %d6
    bne.s   _arm_settle

    | Extra inhibited MMIO traffic AFTER the interrupt is already pending,
    | mirroring the real ROM's SCC WR9..WR1 burst + further VIA1 polling
    | that ran between the T2CH arm and the eventual SR unmask.
    move.b  0x1A00(%a0), %d0          | read IFR (poll, no ack -- T1CL-style
                                       | clear-on-read only applies to T*C-L)
    move.b  #0xC1, 0x0C00(%a0)        | a harmless VIA1 register write (ACR)
    move.b  0x1A00(%a0), %d0          | poll IFR again

    | NOW unmask -- interrupt is already pending (IFR bit5 set, IER bit5
    | set) at this exact point, same as the real hardware capture.
    move.w  #0x2000, %sr              | supervisor, IRQ mask = 0

_wait:
    tst.b   %d7
    beq.s   _wait
    move.l  #0xC0FFEE00, 0xFFFF0000

_handler:
    move.b  0x1000(%a0), %d0          | read T2CL -> clear IFR[T2]
    addq.b  #1, %d7
    rte
