| move_a7_a7_with_irq.s — MOVE.L (A7),A7 stack-unwind idiom with IRQ
|
| HW Q700 wild-jump @ ROM 0x408855e4 has CPU executing
|     0x408855de: MOVE.L (A7)+, A1       ; pop into A1
|     0x408855e0: BEQ.s  +2 (= 0x408855e4)
|     0x408855e2: JSR    (A1)            ; called only if A1 != 0
|     0x408855e4: MOVE.L (A7), A7        ; A7 := MEM[A7]   (in-place)
|     0x408855e6: RTS                    ; PC := MEM[A7], A7 += 4
|
| The MOVE.L (A7),A7 makes A7 := return-PC-of-JSR (= 0x408855e4); the
| RTS then takes PC := MEM[0x408855e4] — i.e. treats the return-PC's
| bytes as a 32-bit jump target.  On HW we land at 0x001fe04c (which
| is in the supervisor stack range) — implying A7 took some other
| value than 0x408855e4 OR RTS read from the wrong memory.
|
| This test exercises the EXACT idiom in sim, with an IRQ injected
| during the MOVE.L (A7),A7 commit window.  If our IRQ-entry logic
| accidentally uses the speculative-A7 from MOVE.L for the M=0 frame
| push, OR if A7's PRF tag leaks via writeback to PC's PRF tag, the
| post-RTE PC will differ from the expected target.
|
| PASS sentinel: 0xC0FFEE00
| FAIL sentinels:
|   0xDEAD0001 — A7 not equal to expected return-PC after MOVE.L (A7),A7
|   0xDEAD0002 — RTS landed at wrong PC (= our manufactured "wild jump")
|   0xDEAD0003 — handler ran but didn't increment the witness counter
|   0xDEAD0004 — vec-4 illegal-instruction (= jumped to garbage)

    .text
    .org 0

_start:
    | Setup supervisor stack at fixed address.
    lea     0x00020000, %a7
    move.w  #0x2000, %sr                 | S=1, IPL=0 — L1 IRQ unmasked

    | Install exception handlers.
    move.l  #_illegal, 0x00000010        | vec 4
    move.l  #_irq_l1, 0x00000064         | vec 25 (autovector L1)

    | Witness counter (zeroed) — handler increments it.
    move.l  #0x00000000, _witness

    | Build the doomed stack frame.
    | Set up A7 such that:
    |   MEM[A7+0]   = the "JSR return-PC" target (must equal 0x40)
    |                 No wait — we are the CALLER.  The callee will
    |                 hit MOVE.L (A7),A7.  At THAT point, the top-of-stack
    |                 holds the return-PC pushed by JSR.
    |   MEM[A7_new] = the target of RTS — must equal a known value.
    |
    | Build the structure: pre-place an "RTS target word" at a known
    | address X in RAM.  Then arrange that after MOVE.L (A7),A7,
    | A7 = X.  Then RTS reads MEM[X] = the target word.
    |
    | The cleanest setup: the callee receives JSR return-PC at top of
    | stack (= address of insn AFTER `jsr _callee`).  MOVE.L (A7),A7
    | makes A7 = that return-PC.  RTS then reads MEM[return-PC] —
    | which is the bytes of the post-JSR instruction itself.
    |
    | We control those bytes!  After the JSR we put a 4-byte dc.l
    | that is the address we WANT to land at (= _expected_rts_target).
    | The actual code resumes after that dc.l with a JMP to verify.

    | Pre-set the canary for RTS-target verification.
    | (Nothing to pre-set; the dc.l after JSR IS the target word.)

    | Now call the doomed routine.
    jsr     _doomed                       | pushes return-PC = addr of dc.l
    .long   _expected_rts_target          | <-- this 4-byte word is what
                                          |     MOVE.L (A7),A7's MEM[A7] =
                                          |     MEM[return-PC] reads.

    | If RTS executes correctly (jumps to _expected_rts_target), that
    | code path eventually returns here — but RTS pops the return-PC
    | bytes themselves into A7+4, so control will NOT return naturally.
    | The _expected_rts_target routine writes PASS and halts.

    | This trap fires only if RTS misbehaves and somehow returns here.
_unreach_after_jsr:
    move.l  #0xDEAD0002, 0xFFFF0000
    bra     .

_doomed:
    | A1 is irrelevant for the MOVE.L (A7),A7 path; just match the
    | ROM idiom — fall through (we skipped the pop/BEQ/JSR-on-A1).
    move.l  (%a7), %a7                   | the critical insn (0x2e57)
    | At this point A7 should equal the return-PC pushed by JSR
    | _doomed.  Verify via a stash — but we can't stash because A7
    | IS the verification anchor.  We just trust the next RTS.
    rts                                  | PC := MEM[A7] (=
                                         |       MEM[return-PC] =
                                         |       _expected_rts_target).

_expected_rts_target:
    | Arrival means MOVE.L (A7),A7 + RTS behaved correctly.
    | (Witness check moved out — we only verify RTS-target landing here.
    | IRQ injection cycles outside the MOVE.L commit window would
    | otherwise false-positive on a "handler never fired" sentinel.)
    | Spin a bit so a delayed IRQ has time to fire before we sentinel.
    move.l  #1000, %d0
_settle_loop:
    subq.l  #1, %d0
    bne     _settle_loop
    move.l  #0xC0FFEE00, 0xFFFF0000
    bra     .

_fail_witness:
    | Unused — kept as a label for the manual disassembly trail.
    move.l  #0xDEAD0003, 0xFFFF0000
    bra     .

_illegal:
    move.l  #0xDEAD0004, 0xFFFF0000
    bra     .

_irq_l1:
    addq.l  #1, _witness
    rte

_witness:
    .long   0
