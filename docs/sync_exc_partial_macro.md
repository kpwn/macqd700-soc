# Sync exception during multi-µop crack — partial-macro replay bug

**Status**: FIXED 2026-05-04 via Path A (atomic crack commit).  Both
repros now PASS (`exc_partial_macro_move_mem_mem`,
`exc_partial_macro_movem_predec`).  Full regression 609/0/0; fuzz 200/200
vs Musashi.  See "Implementation — Path A landed" section near the end.

The Path-B (Format-7 SSW continuation) approach remains as a future
silicon-fidelity upgrade for VM-enabled System 7 demand-paging support;
it is not currently scheduled.

## Bug shape

The 68040 ISA's CISC instructions are cracked at decode time into multiple
µops.  Every µop in a single cracked macro carries the **same `pc`** field
through the ROB (= the macro's start PC).  When a µop with `rob_exc=1`
reaches commit (e.g. a STORE that bus-errors in the middle of a crack),
the synchronous-exception path in `commit.v::take_exc` fires with:

```
exc_fire_fault_pc       <= take_exc_fault_pc;     // = rob_exc_fault_pc OR rob_pc
exc_fire_instruction_pc <= rob_pc;                // = macro start
```

Both addresses point at the **macro's start**, not the failing µop's
position within the macro.  After the OS handler `RTE`s, fetch redirects
to the macro start and the entire macro re-executes.  But every µop that
retired BEFORE the failing one has already committed its architectural
side effects (postinc/predec writebacks, register loads, A7 push, scratch
register updates).  Re-executing the macro applies those side effects a
SECOND time.

This is the same shape as **Bug B** (commit `669b7dc3`), which was the IRQ
sibling — IRQ-fires-mid-macro caused the same partial-macro replay.  Bug B's
fix `take_irq_arm` / `take_irq_fire_q` defers IRQ entry to the natural
macro boundary.  No analogous fix exists for sync exceptions because they
are *caused* by a µop that ALREADY ran past the boundary — there's nothing
to defer.

## Cracks vulnerable to the bug

Audit of `rtl/core/decode/decode_uop_assemble.v` (every place that emits
`uop_is_last = 1'b0` for some phase and a memory access in a later phase):

| Family / shape                            | Crack µops                                        | Vulnerable phase                                  | Observable side effects on partial replay |
|-------------------------------------------|---------------------------------------------------|---------------------------------------------------|-------------------------------------------|
| `MOVE.L (An)+,(Am)+`                      | LOAD; An+=4; STORE; Am+=4                         | STORE faults                                      | An advanced by 8 (twice)                  |
| `MOVE.L -(An),-(Am)`                      | An-=4; LOAD; Am-=4; STORE                         | STORE faults                                      | Am decremented by 8                       |
| `MOVE.L (An)+,(Am)`                       | LOAD; An+=4; STORE                                | STORE faults                                      | An advanced by 8                          |
| `MOVE.L (An),(d16,Am)`                    | LOAD; STORE                                       | STORE faults                                      | (none — LOAD has no side effect)          |
| `MOVE.L -(An),(Am)+`                      | An-=4; LOAD; STORE; Am+=4                         | STORE faults                                      | An decremented by 8                       |
| `MOVEM.L list,-(An)`                      | MOV An→TMP1; An-=N*sz; STORE_0; ...; STORE_N-1   | any STORE faults                                  | An decremented by 2*N*sz                  |
| `MOVEM.L (An)+,list`                      | LOAD_0 → reg_0; ...; LOAD_N-1 → reg_N-1; An+=N*sz | any LOAD faults after first                       | First k registers loaded twice (one re-load with re-faulted value); An incorrectly advanced |
| `BSR (mode 0,16,32-bit disp)`             | STORE ret_pc → -(A7); BR_BRA target               | STORE faults (rare — A7 should always be valid)  | (the predec writeback IS the STORE — single µop, gated by rob_exc, so probably safe — needs recheck) |
| `JSR brief-indexed (d8,An,Xn)`            | TMP1=An+d; TMP2=Xn[*sc]; ...; TMP1+=TMP2; STORE ret_pc; BR_JMP | STORE faults  | (TMP1/TMP2 are scratch — not architectural — so partial replay is safe)        |
| `LINK.W An,#disp16`                       | STORE An→-(A7) (writes A7); MOV A7→An; A7+=disp   | STORE faults                                      | (STORE writes A7 atomically — phase 0 alone, OK; later phases don't memory-access)      |
| `UNLK An`                                 | MOV A7←An; LOAD (A7)→An; A7+=4                    | LOAD faults                                       | A7 corrupted (was set to An; on RTE the macro re-executes and An+=4 fires twice — depending on path) |
| `MOVEP.L Dy,(d16,Ax)` (4 byte-stores)     | STORE_b3; STORE_b2; STORE_b1; STORE_b0            | any STORE after first                             | Earlier byte STOREs already committed — OS may observe partially-updated mem |
| `MOVEP.L (d16,Ax),Dy` (4 byte-loads + merge) | LOAD_b3; merge_b3; ... LOAD_b0; merge_b0       | any LOAD after first                              | Dy partially updated — OS may observe wrong Dy |
| `BCD -(Ay),-(Ax)` mem-mem                 | Ay-=1; LOAD; Ax-=1; LOAD; ALU; STORE              | LOAD or STORE after ph0/ph2 fault                 | Ay/Ax decremented twice                   |
| `CMP2/CHK2 EA,Rn`                         | LOAD lo→TMP1; LOAD hi→TMP2; ALU                   | second LOAD faults                                | TMP1 pollution — but TMP1 is scratch, so safe |

**Highest-impact in practice**: `MOVE.L mem,mem` (postinc/predec), because
the inner-loop in `BlockMove` / system memory copies uses this idiom, and
those routines are commonly the boundary at which a demand-paged Mac OS
hits a translation fault.  `MOVEM.L list,-(An)` is comparable for the
register-save prologue of any subroutine.

**Lowest-impact**: cracks whose only "side effect" before the faulting
µop is a write into TMP0/TMP1/TMP2 (scratch).  Those are recovered by
the rename-RAT rollback at flush.  Examples: indexed JSR, CMP2/CHK2.

## Proposed fixes

### Path A — Atomic commit (true mid-instruction restart)

Make every cracked macro commit atomically: either ALL its µops retire,
or NONE of them retire.  On a partial-fault we walk back the ROB to the
macro's first µop, surface IT as the head, and use ITS PC (= macro
start, same as today) as `saved_pc`.  After RTE the macro re-executes
from a clean architectural state.

Plumbing required:

1. **Per-ROB-entry `is_first_uop`** field (1 bit) — so commit can detect
   "head is mid-macro-N" by looking at `!is_first_uop && head's PC`.
2. **Tentative-retire mode** — every non-last µop in a crack must keep
   its arch-state side effects PENDING until the last µop retires
   without `rob_exc`.  Concretely:
   - `rat_commit_en` is gated `false` for non-last µops.  Their RAT
     mappings are kept in a "shadow" cRAT until the last µop retires,
     at which point a batch commit moves all shadow entries into cRAT.
   - `ccr_commit_en` similarly gated.
   - `commit_store_en` gated — stores within a crack stay in the LSU's
     speculative buffer, not committed to AXI, until the last µop OK.
     (This is hard: stores already need post-retire commit_store_en;
     stalling commit_store_en for non-last µops blocks LSU forward
     progress.)
   - PRF-old recycle gated — frees of `phys_old` for non-last µops
     are deferred to the macro boundary.
3. **Buffered postinc/predec** — these update arch An via `rat_commit`,
   so they're naturally swept up by (1).  But `phys_old` recycling is
   what costs the rollback.

**Cost**: 1-bit per ROB entry + a cRAT shadow region + gating on RAT/CCR
commit + LSU integration for stores.  Roughly the same magnitude as the
Bug B rework (`669b7dc3`), but with deeper LSU intrusion.  Estimated
effort: 1-2 weeks of focused work; risks are LSU/store-buffer
serialisation hazards (already a delicate corner) and a free-list
starvation regression if `phys_old` deferral pins too many physical
registers.

**Behavior on partial replay**: macro re-executes cleanly.  Matches
"clean atomic restart" silicon mode (acceptable per PRM §8.4.5 even on
real 68040, though silicon prefers continuation).

### Path B — Format-7 continuation state (silicon-correct)

The 68040 silicon's actual model (PRM §8.4.5.1, MC68040UM §3.4.1) is
**continuation**: the format-7 frame carries the full mid-instruction
state (Stage A/B/C ALU+EA latches, BO/BI/B0..B7 buffers, CT/CU/CP/CM
continuation bits in SSW).  When the OS handler RTEs, the CPU resumes
the partially-executed instruction from where it left off, with all
prior side effects intact.  This is what real Mac OS demand-paging
relies on.

Our format-7 frame implementation:
- Pushes 30 words but only populates the SR/PC/format/SSW/fault_addr
  slots — see `rtl/core/exception_uop_gen.vh:139`.  The continuation
  bits in SSW (CP/CU/CT/CM) are left at 0.  See also
  `tb/tests/deferred.txt` line 92: "**SSW SIZE/LK/TT/TM/MA +
  continuation flags (CP/CU/CT/CM) NOT YET POPULATED**".
- The CPU does not implement reading continuation bits on RTE.
  An OS that issues a frame-aware RTE expecting continuation will see
  all-zero CT/CU/CP/CM and presumably decide "atomic restart" (CT=0
  ⇒ no continuation needed) — which is the non-silicon-spec but legal
  fallback we use today.

**Cost of full Path B**: comparable to Path A, but the work is in
populating the SSW/Stage*/Buffer* fields on EVERY exception entry, not
in adding atomic-commit gating.  Plus an RTE-side decoder that, on
non-zero CT, restores the Stage*/Buffer* state and resumes from where
the µop sequence stopped (rather than re-running from the macro start).
The latter is a complete second decode/execute path through the
exception sequencer — 2-3 weeks minimum.

### Path C — Document and DEFER (recommended near-term)

The bug is real but its OS-level impact is narrow on Q700:
- Mac OS Classic (System 6/7/8) does NOT use demand paging in the user
  fault path.  RAM is fully mapped from boot; bus errors fire only on
  unmapped I/O probes (which the SIMM-detection code handles via
  PC-skip handlers, not natural-RTE).
- Virtual Memory in System 7 does use demand paging for the VM file —
  but it relies on real-68040 silicon continuation.  Running a VM-
  enabled System 7 on this CPU might bug, but VM is not a tier-1
  bring-up target (System 6 cold boot is).
- A-line / F-line (vec 10/11) cracks in our impl don't have a side-
  effect-bearing µop before the trap firing — the decoder just emits
  a single rob_exc=1 µop.  No vulnerability there.

Recommendation: ship Path C now — these tests in `deferred.txt`,
documented limitation in `isa_status.md` — and tackle Path A or B as
a Phase-3 polish item once VM-enabled System 7 boot becomes a
target.  Track as task `partial_macro_replay_fix`.

## Tests

`tb/tests/asm/exc_partial_macro_move_mem_mem.s`
- Setup: `move.l (a0)+, (a1)+` with A0 mapped, A1 unmapped.
- Handler: first entry patches A1 to mapped DST_GOOD, plain RTE.
- Pre-fix RTL: macro re-executes from start, A0 = init+8, dst gets
  the WRONG long (re-LOADed from advanced A0).  Sentinel = 0xBAD0A008
  (BAD A0 with delta=8).
- Post-fix RTL: A0 = init+4, dst = original LOAD payload.  Sentinel =
  0xC0FFEE00.

`tb/tests/asm/exc_partial_macro_movem_predec.s`
- Setup: `movem.l %d0-%d2, -(%a3)` with A3 = 0xAAAA0000 (unmapped).
- Handler: first entry plain RTE, second entry skips +4.
- Pre-fix RTL: ph1's `An -= 12` retires before fault → A3 decremented
  twice (delta = -24).  Sentinel = 0xBAD0A724 ("BAD A7 -24").
- Post-fix RTL: A3 decremented once (delta = -12).  Sentinel = 0xC0FFEE00.

Both tests reside in `tb/tests/deferred.txt` until the fix lands.

## Bug B's fix vs. this bug

The Bug B fix (commit `669b7dc3`) handles IRQ-during-multi-µop-crack by
**deferring** the IRQ entry until the macro completes.  This works
because IRQ is asynchronous — we can choose when to take it.

Sync exceptions are different: the µop that raises `rob_exc` is the
*cause* of the exception.  We can't defer it past the macro boundary
because the macro never reaches its boundary — that µop is the failing
operation.  The choice is between:

- "Atomic restart" (Path A): roll back ALL prior µops in the macro,
  re-run the macro after RTE — no µop's side effects ever surface.
- "Continuation" (Path B): commit ALL prior µops' side effects, save
  enough state in the format-7 frame that the post-RTE CPU can pick
  up at the failing µop and finish only the remaining ones.

Real 68040 silicon does Path B.  We do neither — we commit prior
µops' side effects (de-facto continuation) but save only the macro
PC (de-facto atomic restart).  This worst-of-both-worlds combination
is what creates the partial-macro replay.

## References

- `docs/take_irq_rethink.md` — Bug B (the IRQ sibling).
- `docs/exception_uop_refactor.md` — exception sequencer architecture.
- `rtl/core/commit.v::take_exc` (line 944) — sync exception fire.
- `rtl/core/decode/decode_uop_assemble.v` — every `uop_is_last = 1'b0`
  emission is a potential vulnerability site.
- `rtl/core/exception_uop_gen.vh::exc_uop_data` — Format-7 frame body.
- MC68040 User's Manual §3.4 (instruction restart vs. continuation),
  §8.4.5 (access-error frame format).
- `tb/tests/deferred.txt` — partial-macro tests + the SSW continuation
  bits gap referenced at line 92.

## Implementation — Path A landed (2026-05-04)

Path A's "atomic crack commit" landed in `rtl/core/commit.v` as a
deferred-commit + drain-replay micro-FSM.  The implementation is
narrower than the original audit envisioned — only `rat_commit_en /
rat_free_en` for non-A7 destinations is deferred, plus an explicit
A7 carve-out for BSR/JSR push compatibility.

### What's deferred and what isn't

| Path                        | Deferral?     | Rationale |
|-----------------------------|---------------|-----------|
| `rat_commit_en` (non-A7)    | YES           | The bug.  Mid-crack faults must rollback non-A7 arch dst. |
| `rat_free_en`  (non-A7)     | YES           | Pairs with rat_commit; same FIFO entry. |
| `rat_commit_en` for A7      | NO (eager)    | BSR/JSR's last_uop is a BR that may mispredict, firing flush_en mid-crack.  rat.v's flush rebuild rolls ratmap to cRAT — if A7 weren't already in cRAT, the rollback would lose the BSR's pushed-A7 mapping.  Eager A7 commit means cRAT[A7] is always live.  Tradeoff: BSR/JSR ph0 STORE faults during a re-executed macro aren't protected, but A7 partial-fault is rare in practice (stack pointer is nearly always mapped) and not a tier-1 boot path. |
| `ccr_commit_en`             | NO (eager)    | CCR is just flags.  Macro re-execution recomputes CCR identically (idempotent), so eager commit can't compound side effects.  Eager CCR also avoids accumulating CCR deferrals that lengthen drain cycles + race FPU completion timing. |
| `commit_store_en`           | NO (eager)    | Stores at non-last positions are idempotent under re-execution (same address+data on the second pass).  Deferring stores would require draining LSU's S_ST_BUF from commit, a deeper refactor with no correctness gain. |
| Mid-crack mispredict-flush  | DROP-BUFFER   | When a BR/JMP last_uop fires mispredict (`flush_en` pulse), the rat.v flush rebuilds ratmap from (stale) cRAT and free_bm from `ref_count==0`.  Buffered phys regs have rc=0 and would erroneously land in free_bm, then drain would re-bind them in cRAT — invariant violation.  Discard the buffer instead and let the macro re-execute from scratch (`mispredict ⊆ mb_drop`). |

### Flush serialization for FPCR/FPSR writes (related fix)

The drain's variable-cycle delay shifts the relative timing between
SYS_FMOVE_FPCR_WR retire and downstream FDIV issue, which previously
relied on natural retire ordering to serialize FPCR.DZ-enable bit 10
against the FDIV's `is_fdiv_dz` decision.  Added SYS_FMOVE_FPCR_WR
and SYS_FMOVE_FPSR_WR (with `rob_is_last_uop` guard) to
`head_sys_flush_after_retire` so a flush + redirect-to-fall-through
fires after the FPCR/FPSR write retires, forcing younger speculative
FP ops to refetch with the updated FPCR.  The `last_uop` guard
preserves FMOVEM.L control-list cracks (which use SYS_FMOVE_FPCR_WR
as a non-last µop and would otherwise self-flush).

### State

Per-cycle commit-buffer FIFO in `commit.v`:

```verilog
localparam MB_DEPTH = 32;            // power-of-2 sized FIFO
localparam MB_PTR_W = 5;             // log2(MB_DEPTH)
reg [MB_PTR_W:0] mb_head, mb_tail;   // wrap bit + index
wire mb_empty = (mb_head == mb_tail);
reg                       mb_has_dst     [0:MB_DEPTH-1];
reg [4:0]                 mb_arch_dst    [0:MB_DEPTH-1];
reg [`PREG_INT_W-1:0]     mb_phys_dst    [0:MB_DEPTH-1];
reg [`PREG_INT_W-1:0]     mb_phys_old    [0:MB_DEPTH-1];
reg                       mb_ccr_has_dst [0:MB_DEPTH-1];  // unused; eager
reg [`PREG_CCR_W-1:0]     mb_ccr_phys_dst[0:MB_DEPTH-1];  // unused
reg [`PREG_CCR_W-1:0]     mb_ccr_phys_old[0:MB_DEPTH-1];  // unused
reg                       mb_is_a7       [0:MB_DEPTH-1];  // unused (A7 eager)
reg                       drain_active;
```

### Key predicates

- `can_commit && !drain_active` — block normal commit during drain so
  the rat_commit / rat_free / ccr_commit ports are exclusive.
- `eager_int_commit = rob_has_dst && (rob_arch_dst == REG_A7 ||
  (rob_is_last_uop && mb_empty))` — fire commit immediately.
- `defer_int_commit = (!rob_is_last_uop || !mb_empty) && !mb_drop` —
  push to FIFO; bump tail; set `drain_active` if last_uop just queued.
- `mb_drop = take_exc || take_priv_exc || take_finalize ||
  take_rte_finalize || take_cache_maint || take_ptest || take_rte ||
  take_trace || take_irq_preempt || take_irq_fire_q || (store-fault) ||
  mispredict` — discard buffer + reset pointers.
- `head0_dual_ok &= mb_empty && rob_is_last_uop && lb_is_last_uop` —
  serialize lane-B dual retire across crack boundaries.

### Drain block

Pulses one buffered entry's commit per cycle.  Read `mb[mb_head_idx]`,
fire `rat_commit_en/rat_free_en` for that entry, advance `mb_head`.
When `mb_head + 1 == mb_tail` after the advance, drop `drain_active`.

### Test outcomes

- `exc_partial_macro_move_mem_mem`: PASS — silicon-correct A0 += 4
  (handler patches A1 to mapped DST_GOOD, retried macro completes).
- `exc_partial_macro_movem_predec`: PASS — A3 stays unchanged (delta=0
  → Path A "atomic restart" leaves A3 at its pre-macro value when the
  handler can't patch the unmapped target).  Test was widened to
  accept delta=0 alongside the silicon-continuation delta=-12.
- Regression: 609/0/0 pre-fix → 609/0/0 post-fix (the 2 newly-passing
  partial-macro tests offset for full count).
- Fuzz: 200/200 vs Musashi.

### Tradeoffs and known limits

- **Drain adds 1 cycle per buffered crack µop.**  For typical 2-3 µop
  cracks this is +2-3 cycles per macro retire.  For MOVEM.L (An)+ with
  16-reg list, drain is +16 cycles.  Overhead is small relative to the
  cracks' own dispatch+execute time (typically 20+ cycles).
- **A7-writing partial-replay (BSR/JSR push fault) NOT covered**.  See
  the A7 carve-out rationale above.  If a BSR's STORE-A7 faults during
  a re-executed macro, A7 will double-decrement.  Tracked but
  considered acceptable on the pragmatic grounds that A7 is the stack
  pointer (almost always mapped) and Mac OS doesn't expose this corner
  on a tier-1 boot path.  A future fix would need either Path-B
  continuation or a more invasive A7-deferred-commit + branch-resolve
  reordering.
- **Drain blocks `take_exc` during its cycles.**  If a fault on a
  speculatively-younger entry fires `cmpl_exc` mid-drain, take_exc
  fires after drain completes (one cycle later).  Acceptable.

### Files modified

- `rtl/core/commit.v` — macro_buf state + drain block + mb_drop wire +
  defer/eager logic in normal-retire branch + dual-commit gate +
  FPCR/FPSR flush addition.
- `tb/tests/asm/exc_partial_macro_movem_predec.s` — accept delta=0
  (Path A) alongside delta=-12 (Path B) as PASS.
- `tb/tests/deferred.txt` — un-DEFER both partial-macro tests.
