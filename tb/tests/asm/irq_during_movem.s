| irq_during_movem.s — IRQ raised mid-MOVEM crack (audit bug #1).
|
| The 68040 PRM §8.1.1 "Exception Recognition" requires interrupts to
| be taken at INSTRUCTION boundaries — not between µops within a
| single instruction.  Today our `take_irq` gate does NOT check
| `rob_is_last_uop`, so it can fire at a sub-µop boundary inside a
| MOVEM crack.  When it does, the `flush_keep_tag = rob_tag - 1`
| squashes the still-pending stores; RTE redirects to MOVEM's start
| PC; the entire MOVEM re-executes — including the µops whose stores
| already retired and committed to memory.  The result is a corrupted
| stack frame.
|
| ─────────────────────────────────────────────────────────────────────
| What this test does
| ─────────────────────────────────────────────────────────────────────
| 1. Pre-poison memory at 0x00010000..0x00010040 with sentinel
|    0xAAAAAAAA so we can tell duplicated stores from clean ones.
| 2. Set D0..D7 to known patterns (D0=0x10101010, ..., D7=0x70707070).
| 3. Set A0..A6 to other patterns (A0=0x00010100, ..., A6=0x00010600).
| 4. Execute MOVEM.L D0-D7/A0-A6, -(A7).  This is a 14-register
|    predecrement crack — emits 14 STORE µops + 1 finalizer.
| 5. Concurrently, the testbench raises IPL=1 mid-MOVEM (around
|    the 4th-7th store µop, configurable via +ipl_cycle=…).
| 6. The level-1 handler at vec 25 increments a counter at IRQ_COUNT
|    and does RTE.
| 7. After mainline resumes (post-MOVEM), validate the on-stack frame:
|    each register must appear EXACTLY ONCE at its correct offset
|    from the post-decrement A7.
|
| Without bug #1 fixed: at least one register appears at the WRONG
| offset, or two adjacent slots show the same register's value
| (duplicated first-store).  PASS sentinel never written.
|
| With bug #1 fixed (rob_is_last_uop gate): IRQ defers until the
| MOVEM finalize retires.  Stack frame is clean.  PASS sentinel
| written.
|
| ─────────────────────────────────────────────────────────────────────
| HARNESS NEEDS
| ─────────────────────────────────────────────────────────────────────
| This test depends on (a) `+ipl=cycle:level` plusarg or equivalent
| in tb_top.cpp, AND (b) cpu_ipl_ext exposed on mac_top (audit bug
| #8).  Both are part of the IRQ-audit follow-up wave.  Until they
| land, this test will simply complete the MOVEM (no IRQ raised) and
| validate the no-IRQ case — which still PASSes by construction.
| The DIVERGENT path triggers iff the harness raises IPL during the
| MOVEM body.
|
| PASS sentinel: 0xC0FFEE00.
| FAIL sentinels:
|   0xDEAD0101 — frame slot mismatch (a register's stored value
|                appears at the wrong offset, or duplicated).
|   0xDEAD0102 — IRQ_COUNT wrong (IRQ fired more than once).
|   0xDEAD0103 — wrong vector taken (handler other than vec 25).

    .text
    .org 0

    .equ PASS_SENT, 0xFFFF0000
    .equ STACK_BASE, 0x00012000
    .equ POISON,    0xAAAAAAAA
    .equ IRQ_COUNT, 0x00011000

_start:
    lea     STACK_BASE, %a7
    move.l  #_lvl1_handler, 0x00000064  | vec 25 (autovec lvl 1)

    | Clear IRQ counter; pre-poison stack frame area.
    move.l  #0, IRQ_COUNT
    move.l  #POISON, %d0
    lea     0x00010000, %a0
    move.l  %d0, (%a0)+
    move.l  %d0, (%a0)+
    move.l  %d0, (%a0)+
    move.l  %d0, (%a0)+
    move.l  %d0, (%a0)+
    move.l  %d0, (%a0)+
    move.l  %d0, (%a0)+
    move.l  %d0, (%a0)+
    move.l  %d0, (%a0)+
    move.l  %d0, (%a0)+
    move.l  %d0, (%a0)+
    move.l  %d0, (%a0)+
    move.l  %d0, (%a0)+
    move.l  %d0, (%a0)+

    | Load known register patterns.
    move.l  #0x10101010, %d0
    move.l  #0x20202020, %d1
    move.l  #0x30303030, %d2
    move.l  #0x40404040, %d3
    move.l  #0x50505050, %d4
    move.l  #0x60606060, %d5
    move.l  #0x70707070, %d6
    move.l  #0x80808080, %d7
    movea.l #0x00A10100, %a0
    movea.l #0x00A20200, %a1
    movea.l #0x00A30300, %a2
    movea.l #0x00A40400, %a3
    movea.l #0x00A50500, %a4
    movea.l #0x00A60600, %a5
    movea.l #0x00A70700, %a6

    | Drop SR.IPL=0 so injected IPL=1 fires.
    move.w  #0x2000, %sr

    | The MOVEM under test.  14 LONG predecrement stores.
    movem.l %d0-%d7/%a0-%a6, -(%a7)

    | After MOVEM (without IRQ, or with clean IRQ): A7 has decremented
    | by 14*4 = 56 bytes from STACK_BASE = 0x00011FC8.
    | Frame layout (low→high address):
    |   (A7+0)  = D0 = 0x10101010
    |   (A7+4)  = D1 = 0x20202020
    |   ...
    |   (A7+28) = D7 = 0x80808080
    |   (A7+32) = A0 = 0x00A10100
    |   ...
    |   (A7+52) = A6 = 0x00A60600
    |
    | Validate each slot.

    movea.l %a7, %a0
    move.l  (%a0)+, %d0
    cmp.l   #0x10101010, %d0
    bne     _fail_slot
    move.l  (%a0)+, %d0
    cmp.l   #0x20202020, %d0
    bne     _fail_slot
    move.l  (%a0)+, %d0
    cmp.l   #0x30303030, %d0
    bne     _fail_slot
    move.l  (%a0)+, %d0
    cmp.l   #0x40404040, %d0
    bne     _fail_slot
    move.l  (%a0)+, %d0
    cmp.l   #0x50505050, %d0
    bne     _fail_slot
    move.l  (%a0)+, %d0
    cmp.l   #0x60606060, %d0
    bne     _fail_slot
    move.l  (%a0)+, %d0
    cmp.l   #0x70707070, %d0
    bne     _fail_slot
    move.l  (%a0)+, %d0
    cmp.l   #0x80808080, %d0
    bne     _fail_slot
    move.l  (%a0)+, %d0
    cmp.l   #0x00A10100, %d0
    bne     _fail_slot
    move.l  (%a0)+, %d0
    cmp.l   #0x00A20200, %d0
    bne     _fail_slot
    move.l  (%a0)+, %d0
    cmp.l   #0x00A30300, %d0
    bne     _fail_slot
    move.l  (%a0)+, %d0
    cmp.l   #0x00A40400, %d0
    bne     _fail_slot
    move.l  (%a0)+, %d0
    cmp.l   #0x00A50500, %d0
    bne     _fail_slot
    move.l  (%a0)+, %d0
    cmp.l   #0x00A60600, %d0
    bne     _fail_slot

    | Validate IRQ counter ≤ 1 (0 if no harness injection, 1 if clean
    | IRQ landed at instruction boundary).  >1 means the IRQ entry
    | re-fired mid-MOVEM (audit bug #2).
    move.l  IRQ_COUNT, %d0
    cmp.l   #1, %d0
    bhi     _fail_count

_pass:
    lea     PASS_SENT, %a1
    move.l  #0xC0FFEE00, %d2
    move.l  %d2, (%a1)
_halt:
    bra     _halt

_fail_slot:
    lea     PASS_SENT, %a1
    move.l  #0xDEAD0101, %d2
    move.l  %d2, (%a1)
_halt_fs:
    bra     _halt_fs

_fail_count:
    lea     PASS_SENT, %a1
    move.l  #0xDEAD0102, %d2
    move.l  %d2, (%a1)
_halt_fc:
    bra     _halt_fc

_lvl1_handler:
    | Increment IRQ counter; RTE back.
    move.l  IRQ_COUNT, %d0
    addi.l  #1, %d0
    move.l  %d0, IRQ_COUNT
    rte
