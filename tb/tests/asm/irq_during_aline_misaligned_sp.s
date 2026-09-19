| irq_during_aline_misaligned_sp.s — IRQ raised mid-A-line trap with odd SP.
|
| Reproduces the Q700 Sad Mac trigger pattern observed on HW (2026-05-14):
|   - CPU in supervisor mode, A7 odd (Mac OS uses odd SP legitimately).
|   - A-line opword trapped (vec 10).  Format-0 frame pushed; A7 stays odd.
|   - IRQ L1 fires mid-dispatcher.  Second frame pushed to still-odd SP.
|   - Hypothesis: nested IRQ on odd SP corrupts state, RTE returns garbage.
|
| Args file injects IPL=1 mid-dispatcher.  Test base case (no .args) must
| also pass — proves odd-SP A-line entry alone is OK.
|
| PASS sentinel: 0xC0FFEE00.
| FAIL sentinels:
|   0xDEAD0501 — A-line handler never reached.
|   0xDEAD0502 — _after_aline not reached (RTE corrupted return PC).
|   0xDEAD0503 — IRQ_COUNT != expected (handler missed or refired).
|   0xDEAD0504 — A7 not restored to setup value.

    .text
    .org 0

    .equ PASS_SENT,    0xFFFF0000
    .equ SSP_ODD,      0x00017FFF       | odd-aligned supervisor stack
    .equ IRQ_COUNT,    0x00011000
    .equ ALINE_HIT,    0x00011004

_start:
    | Set vectors first while A7 is still the reset value (= 0x800000).
    move.l  #_aline_handler, 0x00000028   | vec 10 (A-line)
    move.l  #_irq_handler,   0x00000064   | vec 25 (autovec L1)
    move.l  #0, IRQ_COUNT
    move.l  #0, ALINE_HIT

    | Establish odd SP — Mac OS Q700 legitimately uses odd A7.
    move.l  #SSP_ODD+1, %a7               | A7 = 0x18000 (even)
    subq.l  #1, %a7                       | A7 = 0x17FFF (odd)

    | Ensure supervisor + IPL=0 (so the injected IRQ unmasks).
    move.w  #0x2000, %sr

    | A-line trap site.
    .short  0xA001
_after_aline:
    | Verify A-line handler ran.
    move.l  ALINE_HIT, %d0
    cmp.l   #1, %d0
    bne     _fail_aline

    | Verify A7 restored to the odd setup value.
    cmp.l   #SSP_ODD, %a7
    bne     _fail_sp

    | Verify IRQ handler ran (if .args injected IPL=1; passes through
    | as 0 in the base case where no IRQ fires — accept both 0 and 1).
    move.l  IRQ_COUNT, %d0
    cmp.l   #1, %d0
    beq     _irq_ok
    cmp.l   #0, %d0
    bne     _fail_irq
_irq_ok:

    | Bump A7 to a safe even spot for the rest.  Final result writes use
    | absolute addressing so A7 alignment doesn't matter — but moveal
    | itself is safe regardless.
    move.l  #0x00018000, %a7

    lea     PASS_SENT, %a0
    move.l  #0xC0FFEE00, %d0
    move.l  %d0, (%a0)
_halt_pass:
    bra     _halt_pass

_fail_aline:
    move.l  #0x00018000, %a7
    lea     PASS_SENT, %a0
    move.l  #0xDEAD0501, %d0
    move.l  %d0, (%a0)
_halt_fa:
    bra     _halt_fa

_fail_sp:
    move.l  #0x00018000, %a7
    lea     PASS_SENT, %a0
    move.l  #0xDEAD0504, %d0
    move.l  %d0, (%a0)
_halt_fsp:
    bra     _halt_fsp

_fail_irq:
    move.l  #0x00018000, %a7
    lea     PASS_SENT, %a0
    move.l  #0xDEAD0503, %d0
    move.l  %d0, (%a0)
_halt_firq:
    bra     _halt_firq

| ── A-line handler ───────────────────────────────────────────────
| Increment counter, advance stacked PC by 2 (past the A-line opword)
| so RTE returns to _after_aline.  Run ~30 NOPs to give the testbench
| a window for IRQ injection mid-dispatcher.  Frame format is Format-2
| (12 bytes): SR at 0, PC at 2..4, fmt/vec at 6, inst-addr at 8..A.
|
| Stacked PC for A-line is at SP+2..SP+5 (PC[31:16] then PC[15:0]).
| Saved PC = address of A-line opword.  Advance it by 2 to skip past.

    .align 2
_aline_handler:
    addq.l  #1, ALINE_HIT.l
    | Advance stacked PC past the A-line opword.  PC[15:0] is at SP+4.
    addq.w  #2, 4(%a7)
    | Run NOPs to widen the IRQ-injection window.
    .rept 30
    nop
    .endr
    rte

| ── IRQ L1 handler ──────────────────────────────────────────────
    .align 2
_irq_handler:
    addq.l  #1, IRQ_COUNT.l
    rte
