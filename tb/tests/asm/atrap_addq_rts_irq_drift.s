| atrap_addq_rts_irq_drift.s — Reproducer for HW Q700 -4-byte MSP drift.
|
| Pattern: many A-line traps with the Mac OS Toolbox-style dispatcher
| (= exits via `ADDQ #4, SP; RTS` instead of RTE).  IRQs fire concurrently
| via testbench at various cycle offsets.  After N traps + IRQs, verify
| A7 == initial SSP.
|
| HW observation (2026-05-22): Q700 boot drifts MSP by -4 across ~4000
| A-line traps.  msp-trace shows the drift correlates with IRQ entries
| (SR's X-flag changes between consecutive A-line traps).  Hypothesis:
| an IRQ landing at a specific cycle during the dispatcher's MOVEM-pop /
| ADDQ / RTS sequence causes a 4-byte leak.
|
| The Q700 A-line dispatcher uses format-0 vec-10 entry (8 bytes), then:
|     ...handler work...
|     movea.l (sp)+, ...           ; restore caller regs (MOVEM-pop)
|     addq.w  #4, %a7              ; skip the format/vec slot
|     rts                          ; pop saved-PC as return addr
|
| The format-0 frame on entry:
|     [SP+0]: SR (2 bytes)
|     [SP+2]: PC (4 bytes)
|     [SP+6]: format/vec (2 bytes)
|
| ADDQ.W #4, A7 skips 4 bytes (= SR + format/vec slots assuming overwrite).
| RTS pops 4 bytes (= the saved PC slot).  Net pop = 8 bytes.
| Combined with entry's 8-byte push: balanced.
|
| Test config: PASS = A7 stays exactly at INIT_SSP after N iterations.
| FAIL = A7 drift (writes drifted value to sentinel).
|
| Companion .args file sweeps +ipl=N:1 across many cycle offsets to land
| IRQs in different windows of the dispatcher.

    .text
    .org 0

    .equ INIT_SSP,        0x000FE000
    .equ PASS_SENT,       0xFFFF0000
    .equ TRAP_COUNT_ADDR, 0x000F0000
    .equ TARGET_TRAPS,    200

_start:
    | Supervisor, IPL=7 during setup.
    move.w  #0x2700, %sr
    move.l  #INIT_SSP, %sp

    | Install handlers in low-RAM vector table (VBR=0 default).
    move.l  #_aline_dispatcher, 0x00000028     | vec 10 (A-line)
    move.l  #_irq_handler,      0x00000064     | vec 25 (autovec L1)

    | Zero counter.
    clr.l   TRAP_COUNT_ADDR

    | Now drop IPL to 0 so injected IRQs can fire.
    move.w  #0x2000, %sr

_trap_loop:
    | Verify A7 still at INIT_SSP before each trap.
    move.l  %sp, %d0
    cmp.l   #INIT_SSP, %d0
    bne     _fail_drift

    | Fire an A-line trap.
    .short  0xA001                | _AHndlBase opcode

_after_trap:
    | Increment counter (run in mainline).
    addq.l  #1, TRAP_COUNT_ADDR

    | Loop until target hit.
    move.l  TRAP_COUNT_ADDR, %d0
    cmp.l   #TARGET_TRAPS, %d0
    blt     _trap_loop

    | All N traps done with A7 stable → PASS.
    move.l  #0xC0FFEE00, PASS_SENT
    bra     .

_fail_drift:
    | Write drifted A7 to sentinel for diagnostic.
    move.l  %d0, PASS_SENT
    bra     .

| ── A-line dispatcher (Q700-style: ADDQ #4, A7; RTS exit) ────────────
| Mimics the ROM dispatcher pattern that's known to be sensitive to
| IRQ-during-exit windows.
_aline_dispatcher:
    | MOVEM-style register save (mimics ROM handler at 0x408099B0).
    | Pushes 4 longs = 16 bytes.  Post-MOVEM, original frame at SP+16:
    |   SP+16: SR (2 bytes)
    |   SP+18: PC high (2 bytes)
    |   SP+20: PC low  (2 bytes)
    |   SP+22: format/vec (2 bytes)
    movem.l %d1-%d2/%a1-%a2, -(%sp)

    | Read saved PC (= addr of A-line opcode) from SP+18.
    movea.l 18(%sp), %a2          | A2 = saved PC

    | Advance past the 2-byte A-line opcode.
    addq.l  #2, %a2

    | Q700 trick: overlay the SP+20..23 slot (= PC low + format/vec) with
    | the advanced PC.  After ADDQ #4 below, A7 lands at SP+20, RTS then
    | pops 4 bytes = the overlaid advanced PC.  This is the EXACT pattern
    | the Q700 ROM A-trap dispatcher uses (per commit.v line ~538 comment).
    move.l  %a2, 20(%sp)

    | Nested JSR pattern (mimics ROM dispatcher's JSR ([0x400+D2.W*4]))
    | — adds JSR/RTS push/pop pressure inside the dispatcher exit window.
    jsr     _aline_inner
    nop

    | Restore registers (= MOVEM-pop).  A7 += 16, back at frame top.
    movem.l (%sp)+, %d1-%d2/%a1-%a2

    | Q700 exit: ADDQ #4, A7 → skip SR + PC-high slots.  A7 now points at
    | the overlaid advanced PC.  RTS pops it as the return address.
    addq.w  #4, %a7
    rts

| ── A-line inner subroutine (mimics ROM toolbox-handler) ────────────
_aline_inner:
    | Save D0/A0 (preserve A-line caller's regs even more).
    move.l  %d0, -(%sp)
    move.l  %a0, -(%sp)
    moveq   #5, %d0
    lea     0x000F8000, %a0
    move.l  %d0, (%a0)
    move.l  (%sp)+, %a0
    move.l  (%sp)+, %d0
    rts

| ── IRQ handler (vec 25, level-1 autovec) ───────────────────────────
| Mimics Q700 IRQ handler at 0x40809B60 — register save, NOP, restore,
| RTE.  This is what the testbench fires via +ipl events.
_irq_handler:
    movem.l %d0-%d3/%a0-%a3, -(%sp)
    nop
    nop
    nop
    movem.l (%sp)+, %d0-%d3/%a0-%a3
    rte
