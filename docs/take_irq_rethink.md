# take_irq rethink — IRQ should fire AFTER current instruction retires

## Problem (Bug B root cause)

Today's `take_irq` predicate (commit.v:1028) requires:
- `rob_complete=1` (head µop has completed)
- `rob_is_last_uop=1` (we're at a macro boundary)
- `!inject_active`, `!lsu_busy`, etc.

When all hold, `take_irq` **preempts** the head µop:
- The head µop is squashed (it never retires)
- `flush_keep_tag = rob_tag - 1` discards head + everything younger
- `exc_fire_fault_pc = rob_pc` saves the head's own PC
- After RTE, fetch resumes at the head's PC and the head re-executes

This is what causes Bug B: when the head is the SUB.L of a `SUB; BGE` loop
back-edge, the SUB has been issued/dispatched/completed (CCR write tag
allocated, result on CDB) but **not yet retired**. The IRQ flush rolls
back the SUB's CCR-RAT mapping. After RTE, the SUB re-dispatches —
but in the failing test, the BGE's CCR-source-tag wakeup ends up
seeing a stale CCR phys (or the rebuilt ratmap reads the wrong
phys_dst). The loop never terminates.

The 68040 PRM §8.5.4 actually mandates the **opposite** model:

> An interrupt exception is recognized only at the completion of the
> current instruction — that is, after the current instruction commits
> all of its results.

So the head µop must fully retire FIRST, then the IRQ fires with
`saved_pc = the NEXT instruction's PC`. After RTE, fetch resumes at
the next instruction — no need to re-execute the just-retired one.

That's also a much simpler microarchitecture: no in-flight rollback,
no "what if the head's CCR/RAT/PRF state is half-committed" corner.

## Fix design

Convert `take_irq` from a **preempt** to a **defer**:

1. The predicate stays the same (`can_commit_irq && !store_commit_wait &&
   !rob_exc && !lsu_busy && !inject_active && !rob_is_store &&
   rob_is_last_uop && (ipl_above_mask || ipl_is_nmi)`).
2. When the predicate fires, instead of taking the IRQ THIS cycle,
   we **arm** an `irq_armed` latch and **let the normal retire path
   complete** (rob_pop, rat_commit, ccr_commit, dbg_committed++, etc.).
3. We capture two pieces of state at arm time:
   - `irq_armed_saved_pc <= actual_next` — the PC of the next
     instruction (`rob_br_taken ? rob_br_target : rob_npc_fallthru`).
     This becomes the IRQ frame's saved-PC.
   - `irq_armed_saved_sr <= {arch_sr[15:5], arch_ccr_val}` — the SR
     state at the moment of the just-retired instruction. This becomes
     the IRQ frame's saved-SR.
   - `irq_armed_level <= cpu_ipl` — the level being taken.
4. On the **next** cycle, if `irq_armed=1`, fire `exc_fire` with the
   captured state. The flush_keep_tag is `rob_tag - 1` (= "discard
   the new head and everything younger") since whatever the front-end
   speculatively fetched between cycles is wrong-path.

### Why we need to defer by a cycle (not collapse into one)

Two reasons one might want to collapse the retire and fire into one
cycle (avoid the latency hit):

1. **NBA conflicts** — the retire path drives `rob_pop`, `rat_*_en`,
   `ccr_commit_en`, etc. The fire path drives `exc_fire`, `flush_en`,
   `flush_keep_tag`, `redirect_en`, `redirect_pc`. These are mostly
   disjoint, but `flush_en` + `rob_pop` is a documented illegal
   combination (rob.v's flush handler races with rob_pop's
   head-advance). Splitting across cycles keeps them clean.
2. **Same-cycle dispatch** — between arm and fire, decode can dispatch
   one or two more µops into the ROB tail. Those become wrong-path
   speculative work; the fire-cycle flush squashes them. That's the
   cost of the deferred model. In practice this is 2-3 µops per IRQ,
   negligible.

### Cost

One-cycle latency added to IRQ entry. For a 100 MHz core that's 10 ns.
For a 60 Hz VBL interrupt (16.6 ms period), the relative overhead is
6×10⁻⁷. Negligible.

## What changes in commit.v

### New state (add after `reg cpu_halted;`)

```verilog
// IRQ-defer pattern — see docs/take_irq_rethink.md.  When take_irq's
// predicate fires we arm this latch and let the head µop retire
// normally; on the next cycle we fire the actual exc_fire with
// saved_pc = the just-retired instruction's actual_next.  This
// matches the 68040 PRM §8.5.4 model where IRQs fire BETWEEN
// completed instructions, not by preempting the in-flight head.
reg        irq_armed;
reg [31:0] irq_armed_saved_pc;
reg [15:0] irq_armed_saved_sr;
reg [2:0]  irq_armed_level;
```

### Reset

```verilog
irq_armed         <= 1'b0;
irq_armed_saved_pc <= 32'd0;
irq_armed_saved_sr <= 16'd0;
irq_armed_level    <= 3'd0;
```

### Predicate change

The current `take_irq` becomes the ARM predicate. Add a new "fire"
gate that fires when `irq_armed` and the rest of the post-retire
conditions allow.

```verilog
wire take_irq_arm = can_commit_irq && !store_commit_wait &&
                    !rob_exc && !lsu_busy &&
                    !inject_active &&
                    !(rob_is_store) &&
                    rob_is_last_uop &&
                    !irq_armed &&             // not already armed
                    ipl_nonzero && (ipl_above_mask || ipl_is_nmi);

wire take_irq_fire = irq_armed && can_commit_irq &&
                     !store_commit_wait &&
                     !rob_exc && !lsu_busy &&
                     !inject_active;
```

### Arm action (drop into the normal-retire branch as a side action)

Inside the `else if (can_commit && !rob_exc && !take_priv_exc && !take_rte && ...)`
branch, add:

```verilog
if (take_irq_arm) begin
    irq_armed         <= 1'b1;
    irq_armed_saved_pc <= actual_next;     // next-instruction PC
    irq_armed_saved_sr <= {arch_sr[15:5], arch_ccr_val};
    irq_armed_level    <= cpu_ipl;
end
```

Note: this fires alongside the normal retire actions. The head µop
fully commits this cycle.

### Fire action (new branch in the if-else chain)

Replace the current `else if (take_irq) begin ... end` block with
`else if (take_irq_fire)`. The body uses the captured `irq_armed_*`
state instead of `rob_pc` / `cpu_ipl`:

```verilog
else if (take_irq_fire) begin
    exc_fire            <= 1'b1;
    exc_fire_vec        <= {5'd0, irq_armed_level} + 8'd24;
    exc_fire_fault_pc   <= irq_armed_saved_pc;     // next-instr PC
    exc_fire_instruction_pc <= irq_armed_saved_pc;
    exc_fire_fault_addr <= 32'd0;
    flush_en            <= 1'b1;
    // Discard whatever the front-end speculatively dispatched
    // between arm and fire.  rob_tag - 1 keeps nothing.
    flush_keep_tag      <= rob_tag - { {(`ROB_TAG_W-1){1'b0}}, 1'b1 };
    exc_fire_saved_sr   <= irq_armed_saved_sr;     // pre-retire SR
    // ... a7/sp setup unchanged ...
    exc_fire_is_irq     <= 1'b1;
    exc_fire_irq_level  <= irq_armed_level;
    exc_fire_is_m_irq   <= arch_sr[13] && arch_sr[12];
    exc_fire_msp_before <= (arch_sr[13] && arch_sr[12]) ?
                            arch_a7_val : 32'd0;
    // ... access_* setup unchanged ...
    exc_held_*          <= ...;
    ipl_ack             <= 1'b1;
    commit_in_flight    <= 1'b0;
    prev_a7_store       <= 1'b0;
    stop_pending        <= 1'b0;
    irq_armed           <= 1'b0;     // disarm
end
```

### Edge cases handled correctly

- **STOP-pending**: When STOP is in effect, `take_irq` must still fire
  to break out. The `take_irq_arm` predicate uses `can_commit_irq`
  (the STOP-relaxed gate), so even if `stop_pending=1`, the arm
  fires. The normal retire path under STOP-pending is gated off
  (`can_commit && !stop_pending`), so the just-retired-then-fire
  doesn't apply (there's nothing to retire). Special-case this:
  when STOP-pending, take_irq_arm should ALSO act like the old
  preempt path — fire immediately on this cycle since there's no
  head to retire.
- **Trace+IRQ**: If trace is pending at the same boundary as IRQ,
  trace fires first (by priority order), the trace handler executes,
  on its return the IRQ is still pending and fires next.
- **Same-cycle exception**: If `take_exc` (group-3 sync) and IRQ
  predicate both true (rare — would mean the head has both rob_exc
  set AND IRQ at boundary), take_exc wins. The IRQ stays pending
  until the exception handler retires its first instruction. That
  matches PRM ordering (sync before async).

## Validation

Re-run the two reproducers:
- `tb/tests/asm/irq_during_postinc_loop.s` with `+ipl=1000:1` →
  must PASS (currently TIMEOUT).
- `tb/tests/asm/atrap_dispatch_with_irq.s` with `+ipl=1000:1` →
  must PASS (currently TIMEOUT).

Also check existing IRQ tests don't regress:
- `exc_irq_during_store`, `irq_during_rte`, `irq_during_movem`,
  `irq_during_bsr`, `stop_wait_for_irq`, `exc_priority_irq_vs_sync`.

The `irq_during_*` tests previously validated the rob_is_last_uop
gate. Under the new defer model they should still pass because
the macro boundary is still respected — we just defer the FIRE
by one cycle.

## Open question for review

The STOP-pending special-case is worth thinking about. The current
take_irq fires on `can_commit_irq` which is `stop_pending`-relaxed.
With deferral, the arm fires (good — we want to arm) but the normal
retire path is gated off (because `stop_pending`). That means
`irq_armed` gets set but never fires (no retire happens). Two
options:

1. STOP-pending case keeps the old preempt model (fire same cycle,
   no defer). Simpler, but a small special case.
2. The fire path uses `can_commit_irq` (STOP-relaxed) too, so it
   fires next cycle regardless of stop_pending. Cleaner.

Pick (2). The fire-cycle's flush_keep_tag = rob_tag - 1 still
correctly clears state, and the IRQ entry naturally clears
stop_pending on its way through.

## Implementation — 2026-05-04 landed fix

Landed on commit-pending — same-cycle arm + 1-cycle deferred fire.

**commit.v changes:**

* `take_irq` predicate split into three:
  * `take_irq_arm = take_irq_common && !stop_pending && !irq_armed` — fires
    inside the normal-retire branch alongside `rob_pop`.  Captures
    `irq_armed_saved_pc <= actual_next` (the next-instruction PC) and
    `irq_armed_level <= cpu_ipl`.  Pulses `flush_en` with
    `flush_keep_tag = rob_tag` (keep the retiring head, squash
    speculatively-dispatched younger entries).  Raises `drain_for_irq`.
  * `take_irq_preempt = take_irq_common && stop_pending && !irq_armed` —
    legacy preempt path for STOP-pending (head past STOP is wrong-path
    speculative; squashing-and-re-fetching is correct).
  * `take_irq_fire_q = irq_armed && !lsu_busy && !inject_active &&
    !exc_active && !exc_wait && (ipl_above_mask || ipl_is_nmi)` — fires
    on the cycle after arm, builds saved_sr from the now-authoritative
    `arch_ccr_val` / `arch_sr` (the just-retired head's CCR/SR commits
    settle by this cycle).  Reuses the same body as preempt, with
    `take_irq_fire_q ? irq_armed_saved_pc : rob_pc` selecting the
    saved-PC source.  **Does NOT pulse a fresh flush_en** (the arm-cycle
    flush already cleared ROB; pulsing again with the stale `rob_tag`
    sentinel would race rob.v's `tail_ptr` update and corrupt the queue).

* `irq_armed_clear` cancels the latch when IPL goes away or any other
  redirect path (take_priv_exc / take_finalize / take_rte_finalize /
  take_cache_maint / take_ptest / take_exc / take_rte / take_trace) wins
  the cycle.  The IRQ stays pending in irq_agg and re-arms at the next
  clean boundary inside the new path.

* `drain_for_irq` is a registered output to the front-end.  Cleared
  when `inject_active` rises (exception.v has taken over the dispatch
  lane) — clearing it at fire-cycle would create a 1-cycle window in
  which decode could slip a user µop into the q-latch before
  exc_inj_valid asserts.

**m68k_core_fetch.vh changes:**

* `rn_ready` gated additionally on `!drain_for_irq` (decode can't
  consume new bytes while drain is asserted).
* `q_valid` latch fill gated on `!(drain_for_irq || exc_inject_active)`
  in the non-inject branch — prevents decode's `d_uop_valid` from
  capturing a user µop into the q-latch during arm→fire and during
  inter-inject-µop gaps where `exc_inj_valid` momentarily drops to 0.

**Test update:** `tb/tests/asm/exc_priority_irq_vs_sync.s` — added 4
NOPs between the TRAP and the COUNTER read.  The pre-fix preempt model
re-executed the COUNTER read on RTE (so the read saw post-IRQ
COUNTER); the deferred-fire model lets the read complete BEFORE the
IRQ takes (matches PRM §8.5.4).  The NOPs give the IRQ a clean
boundary to fire on, so the COUNTER read after them sees post-IRQ
state.

**Status:**
* Bug B repros (`irq_during_postinc_loop`, `atrap_dispatch_with_irq`)
  PASS for nearly all `+ipl=N:1` injection cycles in [400..2000].
* Full sim regression: 607/607 PASS, 0 DEFER, 0 FAIL.
* Bitstream: build/vivado/fpga_top.bit synthesises with WNS=+0.009 ns
  at 100 MHz target (impl run 2026-05-04 17:32 — includes the core
  fix; the q_valid + drain_for_irq tweaks landed after synth start
  and need a follow-up bitstream).

**Known follow-up — cyc=200 corner.**  `+ipl=200:1` for both Bug B
repros TIMEOUTs.  At cyc=200 the IRQ injects during the test's
**init phase** while the LSU has init stores in flight; my arm-cycle
flush squashes ROB but the iq_mem / LSU state ends up stranded such
that the first inject store (frame push 0 to 0x1fff8) iss's into
LSU's S_MMU_WAIT and never receives the dcache accept — no ROB
entries past the inject finalize ever retire.  All other tested
injection windows pass.  The MMU side of the LSU stall hasn't been
root-caused; needs a focused debug session with mmu/dcache trace.
Not blocking the main fix's landing — IRQ-during-init-phase is a
narrower window than IRQ-during-loop, and the original Bug B
(IRQ-during-loop) is fully resolved.

## cyc=200 root cause — dual-commit + arm-flush race (2026-05-04 follow-up)

**Status: FIXED.**  The cyc=200 stall (and any other cyc where the IRQ
arms on a clean macro boundary while head+1 is also a plain-INT
last-µop ready to retire on lane B) was a **dual-commit + arm-flush
race**, not an MMU/LSU issue.

The symptom seen in trace was correct: the inject sequence's first
frame-push store entered LSU `S_MMU_WAIT`, transitioned through to
`S_ST_BUF`, but never received `commit_store_en`.  The reason was
that ROB head had advanced ONE SLOT TOO FAR past the inject store's
ROB tag.

### The race

When `take_irq_arm` fires inside the normal-retire branch in commit.v,
two things happen on the SAME cycle:

1.  `rob_pop <= 1'b1` (head retires normally).
2.  `flush_en <= 1'b1` with `flush_keep_tag <= rob_tag` (head only;
    squash everything younger).

Per `take_irq_rethink.md` §"Why we need to defer by a cycle", these
NBAs deliberately split the retire from the IRQ fire across two
cycles — the rob.v flush walk computes
`tail_ptr <= head_ptr + flush_keep_dist + 1` which, with
`flush_keep_dist = 0` (= keep just head), correctly sets
`tail = head + 1`.  When the only pop is single-lane (head_ptr += 1
the same cycle), the result is `head = tail = old_head + 1` — empty
ROB, exactly what we want.

But commit.v's normal-retire branch ALSO supports **lane-B dual
retire**: when the head+1 µop is a plain-INT last-µop already
complete, `lb_commit_en <= 1'b1` fires alongside `rob_pop`, and
`commit_head_pop_dual` advances `head_ptr` by **2** instead of 1.

When dual-retire and `take_irq_arm` collide:

```
rob.v sees:   commit_en=1, lb_commit_en=1, flush_en=1
              flush_keep_dist=0  (keep just OLD head)

dual-pop:     head_ptr <= head_ptr + 2  →  head = old + 2
flush walk:   tail_ptr <= head_ptr + 1  →  tail = old + 1   (uses MASTER head_ptr)

End of cycle: head = old + 2, tail = old + 1 → head is ONE SLOT PAST tail.
```

The ROB is now corrupted.  Subsequent inject dispatch goes to
`tail_idx = old+1`, getting tag `old+1`.  But the ROB head is at
`old+2`, pointing at a stale slot whose `e_pc` happens to be the
just-dispatched lane-B µop's PC (e.g. 0x408000a6 = `moveq #64,%d3`,
the next instruction after the IRQ boundary).

The inject store's `cmpl_en` correctly fires for tag `old+1` and
sets `e_complete[old+1] <= 1`.  But commit reads `e_complete[head_idx]`
(= `e_complete[old+2]`), which is 0 — so `can_commit` stays low and
`commit_store_en` never fires.  LSU sits in `S_ST_BUF` forever.

### Why "cyc=200" (and only some cycles)

The bug needs **lane-B dual-retire to be eligible at the arm cycle**.
That requires head+1 to be:

- plain INT (`UOP_INT`),
- last-µop of its macro (`is_last_uop`),
- not a branch / store / sys / dual-dst / A7 writer,
- already complete.

In the postinc-loop test, the user macros are mostly `MOVEQ` and
`LEA` (1-µop, plain INT) sequences in the init phase.  When an IRQ
arms while a `MOVEQ` is at head and the NEXT `MOVEQ` has already
completed at head+1, dual-retire fires alongside arm and triggers
the race.  The init phase has back-to-back MOVEQs so the eligibility
window is wide.

Inside the loop body, head+1 is usually a STORE (the postinc store
half of `MOVE.L (A0)+,(A1)+`) which fails `can_retire_b`'s
`!lb_is_store` check — so dual-retire doesn't fire and the bug
doesn't reproduce.  That's why most loop-injection cycles passed
even pre-fix and only init-phase cycles tripped.

The original `+ipl=200:1` lands during init.  `+ipl=400..2000:1`
tend to land during the loop body.  The exact arm cycle depends on
the propagation delay from IPL injection to the next clean macro
boundary — which is why the failure window was narrow but
reproducible.

### Fix

Suppress lane-B dual-retire on the cycle `take_irq_arm` fires.  One
line in commit.v's normal-retire branch:

```diff
-                if (head0_dual_ok && can_retire_b) begin
+                if (head0_dual_ok && can_retire_b && !take_irq_arm) begin
                     lb_commit_en <= 1'b1;
```

Two correctness arguments back the suppression beyond the rob.v
race:

1.  `actual_next` (the saved-PC for the IRQ frame) is computed from
    the head's `rob_npc_fallthru` / `rob_br_target` — i.e. "the PC
    that comes AFTER head."  If lane B (= head+1) also retires this
    cycle, lane B IS that PC, and architecturally it has executed.
    The IRQ should then save lane-B's `npc_fallthru`, not head's.
    Letting lane B retire alongside arm-flush would stack the wrong
    return PC.
2.  The arm-cycle flush is supposed to squash "everything past head"
    so the IRQ frame can save a precise pre-IRQ PC.  Allowing lane B
    to retire silently advances the architectural state past the
    boundary — defeating the precise-fault model the deferred-fire
    architecture was built for.

Suppressing lane-B retire keeps head advancement to exactly +1,
matching the flush walk's `tail = head + 1`, leaving an empty ROB
for the inject sequence.  Lane B and any speculatively-dispatched
younger entries are squashed by the arm-cycle flush and re-fetched
after RTE.

### Validation

`+ipl=N:1` sweep for both repros:

```
irq_during_postinc_loop:    200, 400, 500, 600, 700, 800, 1000, 1200, 1500, 2000 → ALL PASS
atrap_dispatch_with_irq:    200, 400, 500, 800, 1000, 1200, 1500, 2000          → ALL PASS
                            (cyc=600, 700: pre-existing FAIL — different bug,
                             also fails on baseline `669b7dc3` pre-cyc=200 fix)
```

Full sim regression: 607 PASS / 0 FAIL / 2 expected DEFER (unchanged).
Fuzz: 200/200 vs Musashi (unchanged).

## Earlier implementation attempt — 2026-05-04 partial fix (REVERTED)

A first-cut implementation of the deferred-fire model was attempted
on top of `c3ced088`.  It improved the picture (fault_pc became the
NEXT instruction instead of the SUB itself, as expected) but had
two issues that prevented merge:

1. **Stale CCR in saved_sr**: capturing `arch_ccr_val` at the arm
   cycle saw the PRE-retire CCR (the just-retiring macro's CCR
   commit takes effect via NBA at the next cycle).  Moving the
   capture to the fire cycle helped some windows but not all —
   suggests CCR settles 2 cycles after retire-cycle, not 1.  Need
   to either: (a) defer arm by one more cycle (3-cycle pipeline),
   (b) use a forwarding path that reads the in-flight CCR-CDB
   broadcast directly, or (c) latch the SUB's just-broadcasted
   CCR result on the retire cycle (similar to how store-buffer
   works for stores).

2. **Inject sequence stall**: with deferral, the IRQ fire happens
   one cycle later — by which time the front-end has already
   speculatively dispatched 1-2 µops past the retired macro.  The
   flush_keep_tag = rob_tag - 1 squashes them, but the iq_mem
   may still hold their state, blocking the inject sequence's
   terminating LOAD from dispatching.  3 existing IRQ tests
   regressed because of this:
   - `exc_priority_irq_vs_sync` (FAIL with deferred fix)
   - `irq_during_movem` (FAIL with deferred fix)
   - `stop_wait_for_irq` (FAIL with deferred fix)

   The iq_mem flush handler may need a stronger reset on the IRQ
   entry path (clear all pending stores too, not just the buffered
   ones).

**Repro-only sweep with the partial fix**:

```
cycle=200 → PASS    cycle=300..400 → TIMEOUT   cycle=450..600 → PASS
cycle=650..1000 → TIMEOUT  cycle=1100..1200 → PASS
```

So the deferred model fixes some but not all windows.  The remaining
failures are likely the inject-sequence stall mentioned above, or
the 2-cycle CCR settle.

**Next attempt should:**
1. Defer arm-to-fire by 2 cycles (let CCR fully propagate).
2. Audit iq_mem flush behaviour during IRQ entry — make sure no
   pending state from speculative post-arm dispatches blocks the
   inject sequence.
3. Re-verify all existing IRQ tests pass before landing.
4. Then re-verify Bug B repros pass at all sweep cycles.

The investigation is at task #16 in the session task list.  The
RTL change has been REVERTED on `main` — only this doc + the
two repro tests (`tb/tests/asm/irq_during_postinc_loop.s`,
`tb/tests/asm/atrap_dispatch_with_irq.s`) are committed.

## HW JTAG bisection — 2026-05-04 evening (post-Bug-B-fix-source, pre-fresh-synth)

The bitstream at `build/vivado/fpga_top.bit` (synth 17:32) PRE-DATES the
Bug B fix commit `669b7dc3` (17:39).  The fix is committed in source but
the running bitstream does NOT include it.

JTAG bisection on this pre-fix bitstream:

* `RAM_WINDOW_LG2` register has a duplicate-offset bug at 0x00054
  (collides with `OFF_EXC_FAULT_ADDR`).  Reads via JTAG-AXI returned
  the read-only `exc_fault_addr_r`=0; writes still hit
  `ram_window_lg2_r` (Verilog duplicate-case picks the first labelled
  case for each operation, and the write is in the second case).
  The actual `ram_window_lg2_q` clamps to `RAM_WINDOW_LG2_MIN`=22
  (4 MiB) when the source register reads as 0, so the visible RAM was
  in fact 4 MiB regardless.  Fix: move `OFF_RAM_WINDOW_LG2` from
  0x00054 to 0x00058 (next free slot before the halt-exc-mask block
  at 0x00060).

* Boot timing on real HW is much slower than sim because HW reads ROM
  straight from SD/flash and runs the full RAM-test / checksum / chime
  paths (sim's `mame-fastdiag,chime-skip` patch is sim-side-only).
  Bisection points:
    - retired ~1M:  ROM-checksum loop @ 0x40847510-1c (subq+bne).
    - retired ~5M:  RAM-init loop @ 0x408472f6.
    - retired ~8M:  another settle-loop @ 0x40847ac0 (`movew #0x2300,sr`
      followed by `subq.l #1, %d0; bne`).
    - retired ~9M:  exc_count climbs to 0x12=18 with vec=0x19=25
      (level-1 IRQ autovector — VIA1 timer IS firing).
    - eventually:   exc_count reaches 0x13c=316 with vec=11 (F-line
      at 0x253c) — Marco's documented BlockMove-dispatcher
      divergence cascade.  PC ends up walking ROM zero-pad, which RTSes
      to 0x252e (BlockMove A0 source pointer), which is low RAM with
      0xFF…FF garbage interpreted as F-line, FPSP (still partial)
      can't unwind, stack overflows, CPU walks into 0x18000000+
      (out-of-window — returns OKAY+0).

* The "exc_count=2 / exc_vec=9 / exc_pc=0x40847bfc" snapshot from the
  earlier session was a transient mid-boot state, NOT steady-state.
  By the time the JTAG snapshots a halt request from the running CPU,
  the boot has progressed past the trace phase and onward to the
  IRQ-and-F-line phase.  The two early TRACE exceptions still fire
  (T1=0 in arch_sr) and remain task #19 — most likely root cause is
  a saved-SR with T1=1 in one of the first cold-boot exception frames
  whose RTE re-arms trace.  Pre-Bug-B-fix bitstream may also expose
  IRQ-during-cold-boot races that wouldn't reach sim regression.

**Next action**: re-synth `build/vivado/fpga_top.bit` from current
source (Bug B fix included) and re-bisect — confirm that the
BlockMove-dispatcher divergence (Marco's task) actually moves with
the Bug B fix.  Pre-fix bitstream test isn't a clean signal.
# Task #19 — Spurious TRACE @ boot — JTAG bisection findings

**Bitstream**: `build/vivado/fpga_top.bit` synth 17:32 (PRE Bug B fix; PRE
debug_ctrl OFF_RAM_WINDOW_LG2 fix).

## Confirmed via JTAG halt-on-exc

1. **Halt-on-exc mask requires explicit gating in HALT_CTL.bit6**
   (halt_exc_enable_r).  Setting bit 9 in the 256-bit mask only halts
   when `w 0x5090003c 0x44` writes both the enable AND a one-shot
   clear pulse beforehand.  Otherwise the latch stays from the
   previous run and reads as the static "enabled" state.

2. **Spurious trace IS NON-DETERMINISTIC across resets**.  In ~5
   `full-reset-and-halt` cycles, only ~2 produced an `exc_vec=9`
   firing.  The other 3 cycles run cleanly (only `exc_vec=25` IRQs
   fire, no traces).  This points at a race condition, not a
   deterministic logic bug.

3. **First exception is always IRQ vec 25**, fault_pc =
   0x40847bfa (the BEQS in the settle-loop) or 0x40847bfc (the
   SUBQ).  The IRQ frame on the supervisor stack:
   - saved SR = `0x2000` (clean: T1=0, T0=0, S=1, M=0, IPL=0).
   - saved PC = 0x40847bfa (or bfc).
   - format = 0x0064 (Format-0, vec 0x19=25).

4. **When trace fires (vec 9), exc_count=2**.  Frame on stack at
   A7=0xfe86:
   - **saved SR = `0x4084`** ← bug.  Decode: T1=0, **T0=1**, S=0
     (USER mode!), M=0, IPL=0, bit 7=1, CCR.Z=1.
   - saved PC = 0x40847bfc.
   - format = 0x2024 (Format-2, vec 9).
   - inst PC = 0x40847bfc.

5. **Live arch_sr at trace handler entry = 0x2080** — bit 13 (S)=1,
   bit 7=1, T-bits cleared by `take_finalize` per spec.  Confirms
   the saved_sr was captured PRE-`take_finalize` (when arch_sr was
   in the corrupt 0x4084 state).

6. **PC ring trace** around the trace fire shows the mainline loop
   running (0x40847bf6/bfa/bfc), then IRQ handler entry 0x40847d06,
   handler body 0x40847d08..0x40847d42 (`move.l (sp)+, d0`, `rte`),
   then a single `pc=0x00000000` entry (suspicious — possibly an
   uninitialised ring slot, or an actual bad fetch), then back to
   0x40847bfc (mainline).  No SR-write instructions in this window.

## Hypothesis

The pre-Bug-B-fix `take_irq` body in `commit.v` does NOT clear
`trace_pend` when it preempts the head.  If `trace_pend` was armed
just before the IRQ took (by an earlier µop's retire seeing
`arch_sr.T1=1` or `T0+CoF`), the IRQ entry preserves it.  After the
IRQ handler RTEs, `trace_pend` is still 1.  The first user µop
retiring after RTE fires `take_trace`.

But the saved_sr=0x4084 with **S=0** is incompatible with the
post-IRQ-RTE arch_sr=0x2000 (S=1).  Either:
  a) `take_trace` fires BEFORE `take_finalize` of the IRQ entry
     completes, so saved_sr captures arch_sr in some half-updated
     state.
  b) A race in `arch_sr` writes — multiple paths set arch_sr in the
     same cycle and the muxed result is corrupt.
  c) Bit 7 of arch_sr is being set by an unaccounted-for path,
     suggesting our impl uses bit 7 internally for something else
     (the saved_sr always had bit 7=1, even on the clean IRQ frame's
     saved_sr=0x2000... actually no, IRQ frame saved_sr=0x2000 has
     bit 7=0).

## Repro recipe

```
# Set vec-9 only, full-reset-and-halt loop, look for vec=9 halt
for i in 1..10:
  halt-exc-mask raw 0 0x200
  halt-exc-mask raw 1 0
  full-reset-and-halt
  w 0x5090003c 0x44
  w 0x50900008 0
  sleep 30
  halt-status
```

Probability of vec-9 firing ≈ 30-50% per reset (rough estimate).

## Likely path forward

1. **Re-synth bitstream with Bug B fix** (commit 669b7dc3 onwards).
   The deferred-fire model may itself fix the spurious trace by
   clearing `trace_pend` in the new `take_irq_fire_q` path.  Also
   clears the duplicate-offset OFF_RAM_WINDOW_LG2 issue.

2. **In sim**: instrument arch_sr writes with `$display` for any
   path that sets arch_sr[14] (T0).  Boot the rom-boot harness with
   IRQ injection at the equivalent cycle.  See if T0 gets set
   spuriously.

3. **Read the inj µop retire path**: confirm whether inject µops
   pass through the trace_pend arming gate at line 2580 (they
   shouldn't — but `is_last_uop=1` + `is_branch=1` for the
   finalize LOAD might inadvertently arm it).

## Task #19 — RESOLVED 2026-05-04 evening

Re-tested on fresh bitstream synthesised from HEAD=33b174d1 (Bug B
fix + Bug B cyc=200 fix + sync-exc partial-macro Path-A +
OFF_RAM_WINDOW_LG2 offset fix + dual-commit disabled,
NO_INCREMENTAL clean route).

JTAG halt-on-vec-9 sweep:
- 28+ snapshots across multiple cold-reset cycles, multiple wait
  windows (8s, 12s, 30s, 60s).
- ZERO halt-on-vec-9 fires (reason=0x80, latch never set).
- Pre-fix bitstream had vec-9 fire on ~30-50% of cold resets.

The deferred-fire `take_irq_fire_q` path replaces the preempt-and-
squash model that was racing trace_pend.  No saved_sr corruption
captured in any of the new-bitstream traces.  Spurious TRACE bug
eliminated.

**Side-effect observation**: boot still hits the BlockMove
dispatcher cascade (Marco's Bug B downstream divergence), but the
fault signature shifted from vec 4 (illegal) at 0x40000000 to vec
2 (bus error) at 0x4080010e (`movel %a4@(-20), %fp`) once dual-
commit was disabled.  Underlying divergence is the same; only the
downstream symptom moved.  Tracked separately.
