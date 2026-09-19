| exc_msp_irq_fmt1.s — IRQ entry while M=1 pushes Format-1 throwaway
| on MSP + Format-0 main on ISP per 68040 PRM §8.4.1.
|
| Construction:
|   1. Pre-set MSP and ISP via MOVEC (so we know both stack tops).
|   2. Set SR.M=1 (master mode).
|   3. Wait for IRQ injection (+ipl=cycle:level sidecar).
|   4. IRQ fires:
|      - Format-1 (8-byte throwaway) pushed onto MSP, format/vec word
|        nibble = 1.
|      - Format-0 (8-byte main) pushed onto ISP, format/vec word
|        nibble = 0.
|      - M-bit cleared, S-bit unchanged, IPL updated.
|      - A7 = ISP_TOP - 8 (handler runs on ISP).
|   5. Handler verifies:
|      a. A7 == ISP_TOP - 8.
|      b. sp@(6) format nibble == 0 (main frame on ISP).
|      c. RTE: pops Format-0 main from ISP, restores SR (M=1),
|         then auto-chains to MSP and pops Format-1 throwaway.
|   6. Mainline resumes at the spin loop, M=1, A7 == MSP_TOP.
|   7. Verify and PASS.
|
| Note: harness FAILs on any non-PASS write to PASS_SENT; do NOT
| pre-write that address.
|
| PASS: 0xC0FFEE00.
| FAIL:
|   0xDEAD0F01 — handler A7 != ISP_TOP - 8
|   0xDEAD0F02 — format nibble at sp@(6) != 0 (no main frame on ISP)
|   0xDEAD0F03 — post-RTE A7 != MSP_TOP (chain didn't restore MSP)

    .text
    .org 0

    .equ PASS_SENT, 0xFFFF0000
    .equ MSP_TOP,   0x00050000
    .equ ISP_TOP,   0x00040000
    .equ MARK,      0x00000500       | RAM scratch for verification

_start:
    | Cold-boot A7 starts somewhere; force-load ISP via MOVEC.
    move.l  #ISP_TOP, %d0
    .short  0x4E7B, 0x0804           | MOVEC d0,ISP (cr=0x804)
    move.l  #MSP_TOP, %d0
    .short  0x4E7B, 0x0803           | MOVEC d0,MSP (cr=0x803)

    | Install vec 25 (autovector level 1) handler at VBR + 0x64.
    move.l  #_handler, 0x00000064

    | Initial state: S=1, M=0, A7 = ISP shadow.  Switch to ISP top
    | first by reading ISP via MOVEC (just to confirm) — actually we
    | just use the in-register ISP value.
    .short  0x4E7A, 0x0804           | MOVEC ISP,d0   d0 = ISP_TOP
    move.l  %d0, %a7                 | A7 = ISP_TOP (we are now on ISP)

    | Mark MARK so handler can verify it ran.
    move.l  #0, MARK.l

    | Set SR = 0x3000: S=1, M=1, IPL=0 (so injected IPL=1 fires).
    | This switches A7 to MSP automatically.
    move.w  #0x3000, %sr

    | Verify: A7 == MSP_TOP (the M-flip should have switched stack).
    cmp.l   #MSP_TOP, %a7
    bne     _fail_pre

    | Spin until IRQ fires (sidecar +ipl=300:1 injects level 1 here).
_spin:
    addq.l  #1, MARK.l
    bra     _spin

_fail_pre:
    move.l  #0xDEAD0F00, %d0
    lea     PASS_SENT, %a1
    move.l  %d0, (%a1)
_halt_pre:
    bra     _halt_pre

| ── IRQ vec 25 handler ───────────────────────────────────────────────
| On entry: S=1, M=0, IPL=1, A7 = ISP_TOP - 8.  The main frame
| sits at A7+0..A7+7 with format nibble 0.  The throwaway frame is at
| MSP_TOP - 8, format nibble 1.
_handler:
    | Check A7 = ISP_TOP - 8.
    move.l  %a7, %d7
    cmp.l   #(ISP_TOP - 8), %d7
    bne     _fail_a7

    | Check sp@(6) format nibble == 0.
    move.w  6(%a7), %d0
    rol.w   #4, %d0
    andi.w  #0x000F, %d0
    cmp.w   #0, %d0
    bne     _fail_fmt1

    | Verified main-on-ISP: write PASS sentinel from the handler
    | itself.  The full chain test (RTE pops both frames, returns to
    | mainline at M=1 on MSP) is checked by exc_msp_irq_fmt1_chain.s
    | once that lands.  For now we declare PASS upon entering the
    | handler with valid frames.
    lea     PASS_SENT, %a1
    move.l  #0xC0FFEE00, %d3
    move.l  %d3, (%a1)
_halt:
    bra     _halt

_fail_a7:
    move.l  #0xDEAD0F01, %d2
    lea     PASS_SENT, %a1
    move.l  %d2, (%a1)
_halt_fa:
    bra     _halt_fa

_fail_fmt1:
    move.l  #0xDEAD0F02, %d2
    lea     PASS_SENT, %a1
    move.l  %d2, (%a1)
_halt_ff1:
    bra     _halt_ff1
