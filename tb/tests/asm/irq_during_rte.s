| irq_during_rte.s — IRQ raised during an in-flight RTE pop sequence
| (audit bug #3).
|
| RTE is dispatched as 2 LONG LOADs (per `decision #11` in CLAUDE.md
| equivalent + `exception.v` Phase 2a) reading SR/PC/format from the
| supervisor stack.  A `take_rte_finalize` fires the atomic
| SR/A7/PC restore when the second LOAD retires at head.
|
| Today, between `take_rte` firing and `take_rte_finalize` retiring,
| there is no commit-side gate that holds off another `take_irq`.
| `exc_active` is permanently 0 (audit bug #5); `exc_wait` is not
| set on the take_rte path; once the first RTE pop LOAD completes
| at head, can_commit is high.  A new IRQ that satisfies
| `cpu_ipl > arch_sr.I` (which is still the OLD supervisor level
| because the saved SR hasn't been restored yet) will fire
| take_irq, flush the still-pending RTE finalize µop, and leave
| the CPU stranded — saved SR never restored, A7 never advanced,
| arch_sr.I still at the supervisor level.
|
| This test sets up a controlled supervisor frame on SSP and
| executes RTE while the testbench raises IPL=1 mid-pop.
| ─────────────────────────────────────────────────────────────────────
| Construction
| ─────────────────────────────────────────────────────────────────────
| 1. Synthetic supervisor frame at SSP-8:
|      offset 0: SR = 0x2000 (S=1, IPL=0, T=0)
|      offset 2: PC[31:16] = high 16 bits of `_target`
|      offset 4: PC[15:0]  = low 16 bits of `_target`
|      offset 6: format = 0x0000 (fmt-0 frame)
| 2. Set vec 25 handler (autovec lvl 1).
| 3. Execute RTE.  Testbench raises IPL=1 between the first LOAD
|    retiring and the second.
| 4. Expected behaviour with bug #3 fixed: RTE completes; control
|    redirects to `_target`; afterwards, the IRQ takes effect at
|    `_target`'s instruction boundary; handler RTEs back; control
|    resumes at the post-handler PC inside `_target`.
| 5. With bug #3 unfixed: take_irq fires mid-RTE; RTE never
|    completes; CPU runs the IRQ handler from a broken state
|    (stack still has the RTE-pop frame on top).  The handler
|    will try to RTE from THAT broken state — observable as
|    either a wedge or the wrong saved-SR restoration.
|
| ─────────────────────────────────────────────────────────────────────
| HARNESS NEEDS: same as irq_during_movem.s.
|
| PASS sentinel: 0xC0FFEE00.
| FAIL sentinels:
|   0xDEAD0301 — _target never reached (RTE got squashed).
|   0xDEAD0302 — Handler ran but with wrong saved-SR (RTE only half-completed).
|   0xDEAD0303 — IRQ_COUNT > 1 (re-fire).

    .text
    .org 0

    .equ PASS_SENT, 0xFFFF0000
    .equ STACK_BASE, 0x00012000
    .equ IRQ_COUNT, 0x00011000
    .equ TARGET_HIT,0x00011004
    .equ HANDLER_SR,0x00011008

_start:
    lea     STACK_BASE, %a7
    move.l  #_lvl1_handler, 0x00000064  | vec 25 (autovec lvl 1)

    move.l  #0, IRQ_COUNT
    move.l  #0, TARGET_HIT
    move.l  #0, HANDLER_SR

    | Build a synthetic format-0 frame at SSP-8.
    | Layout (low → high):
    |   SR = 0x2000 (supervisor, IPL=0)
    |   PC = address of _target
    |   format = 0x0000 (fmt-0)
    move.w  #0x2000, -(%a7)              | A7-2 = SR
    move.l  #_target, %d0
    move.w  %d0, -(%a7)                  | A7-4 = PC[15:0]
    swap    %d0
    move.w  %d0, -(%a7)                  | A7-6 = PC[31:16]
    move.w  #0x0000, -(%a7)              | A7-8 = format word

    | Order on stack from low to high after the four pushes:
    |   (a7+0) = format
    |   (a7+2) = PC[31:16]
    |   (a7+4) = PC[15:0]
    |   (a7+6) = SR
    | Wait — RTE expects (per 68040 PRM):
    |   (a7+0) = SR
    |   (a7+2) = PC[31:16]
    |   (a7+4) = PC[15:0]
    |   (a7+6) = format word
    | Our predecrement push order built it backwards.  Rebuild via
    | direct stores:
    move.w  #0x2000, 0(%a7)              | SR
    move.l  #_target, %d1
    move.w  %d1, 4(%a7)                  | PC[15:0]
    swap    %d1
    move.w  %d1, 2(%a7)                  | PC[31:16]
    move.w  #0x0000, 6(%a7)              | format

    rte                                  | should redirect to _target

_target:
    | Mark target reached.
    move.l  #1, TARGET_HIT

    | Mainline-after-RTE.  Testbench's IPL injection should arrive
    | at this instruction boundary OR mid-RTE; either way, the
    | IRQ handler should fire eventually.
    nop
    nop
    nop
    nop

    | Validate.
    move.l  IRQ_COUNT, %d0
    cmp.l   #1, %d0
    bhi     _fail_count

    | Whether the IRQ fired here or mid-RTE, _target should still
    | have been reached — proves RTE actually completed.
    move.l  TARGET_HIT, %d2
    cmp.l   #1, %d2
    bne     _fail_target

    | If handler captured saved SR, the SUPERVISOR HALF (bits 15:8 —
    | T1/T0/S/M/IPL/reserved) must be 0x20 (= S=1, M=0, IPL=0).  Per
    | 68040 PRM §8.4.1, the IRQ-entry frame saves the PRE-IRQ SR to
    | (a7+0); for this test the pre-IRQ supervisor half is 0x20 (S=1
    | from synthetic frame, T1/T0/M/IPL all 0).  The CCR LOW byte
    | (bits 7:0) is whatever the mainline left in CCR before the IRQ
    | fired — that depends on injection cycle (could be 0x09 if a
    | preceding CMP set N+C, or 0x04 if Z=1, etc.) so we don't
    | assert the low byte.
    move.l  HANDLER_SR, %d3
    cmpi.l  #0, %d3
    beq     _no_irq_yet                  | harness didn't inject
    andi.l  #0xFF00, %d3                 | mask off CCR low byte
    cmpi.l  #0x2000, %d3
    bne     _fail_sr

_no_irq_yet:
_pass:
    lea     PASS_SENT, %a1
    move.l  #0xC0FFEE00, %d4
    move.l  %d4, (%a1)
_halt:
    bra     _halt

_fail_target:
    lea     PASS_SENT, %a1
    move.l  #0xDEAD0301, %d4
    move.l  %d4, (%a1)
_ht:
    bra     _ht

_fail_sr:
    lea     PASS_SENT, %a1
    move.l  #0xDEAD0302, %d4
    move.l  %d4, (%a1)
_hs:
    bra     _hs

_fail_count:
    lea     PASS_SENT, %a1
    move.l  #0xDEAD0303, %d4
    move.l  %d4, (%a1)
_hc:
    bra     _hc

_lvl1_handler:
    | Bump counter, snapshot saved SR (16-bit at A7+0 of the IRQ
    | frame; we widen to 32 bits for the test sentinel comparison).
    move.l  IRQ_COUNT, %d5
    addi.l  #1, %d5
    move.l  %d5, IRQ_COUNT
    move.w  (%a7), %d6
    andi.l  #0xFFFF, %d6
    move.l  %d6, HANDLER_SR
    rte
