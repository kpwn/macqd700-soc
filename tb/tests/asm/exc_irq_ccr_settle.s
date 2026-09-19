| exc_irq_ccr_settle.s — IRQ entry must save the boundary instruction's
| CCR, not the previous instruction's (CCR-settle race, task #41).
|
| take_irq_fire_q is the ONE exception-entry path that bypasses the
| commit_in_flight interlock (it arms at a clean macro boundary and
| fires the NEXT cycle).  Every other path — take_exc / take_priv_exc /
| take_rte / take_trace / take_irq_preempt — has commit_in_flight in its
| gate (via sync_exc_pretest or can_commit_irq), which forces a >=1-cycle
| post-retire gap so arch_ccr_val (= ccr_prf[crat_tag], lagging
| ccr_commit_en by one cycle) is already settled.  take_irq_fire_q had
| no such gap, so the IRQ frame saved the CCR of the instruction BEFORE
| the boundary.  Fixed by gating take_irq_fire_q with ccr_settle_in_flight.
|
| Repro: MOVE.W clears C (C=0); CMP.W sets C=1.  A level-1 IRQ is
| injected by the testbench (.args +ipl=N:1) tuned to arm at the
| CMP -> BCS boundary, in the CMP's CCR-commit settle window.  The
| autovector-1 handler just RTEs.  Post-RTE the BCS must see C=1.
|
| With the bug: IRQ frame saves C=0 (the MOVE's CCR); RTE restores it;
| BCS falls through -> 0xDEAD0001.
| Fixed: IRQ fire waits one cycle for the CMP's CCR to settle; C=1 is
| saved and restored; BCS is taken -> PASS.
|
| PASS sentinel: 0xC0FFEE00.
| FAIL sentinel: 0xDEAD0001 (C lost across IRQ entry/RTE).
|
| NOTE: the +ipl cycle is timing-tuned (see exc_irq_ccr_settle.args).
| If front-end latency shifts, re-tune so the IRQ takes the CMP->BCS
| boundary — the +irq_at_cycle ROM repro is the authoritative check.

    .text
    .org 0

_start:
    lea     0x00020000, %a7              | SSP
    move.l  #_irq_handler, 0x00000064    | vector 25 (autovector L1) @ 0x64
    move.w  #0x2000, %sr                 | S=1, IPL=0 — level-1 IRQ unmasked

    | Boundary pair.  MOVE.W clears V,C (C=0); CMP.W sets C=1.
    move.w  #0x1000, %d2                 | D2.w=0x1000; C := 0
_cmp_site:
    cmp.w   #0x2000, %d2                 | 0x1000-0x2000 → borrow → C := 1
_bcs_site:
    bcs.s   _pass                        | C=1 → taken.  stale C=0 → fall through

_fail:
    lea     0xFFFF0000, %a0
    move.l  #0xDEAD0001, %d0             | C lost across IRQ entry / RTE
    move.l  %d0, (%a0)
_halt_fail:
    bra     _halt_fail

_pass:
    lea     0xFFFF0000, %a0
    move.l  #0xC0FFEE00, %d0
    move.l  %d0, (%a0)
_halt:
    bra     _halt

    | Autovector level-1 handler — minimal RTE-er.  Returns to _bcs_site
    | with the CCR restored from the frame the IRQ entry built.
_irq_handler:
    rte
