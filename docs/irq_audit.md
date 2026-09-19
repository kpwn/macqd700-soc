# 68040 Interrupt Subsystem Audit

Sub-agent audit, 2026-05-03. Read-only audit of the IRQ entry/exit path,
peripheral aggregator, NMI handling, RTE serialization, and host-side
IRQ injection.  Findings drive a follow-up RTL wave (no fixes landed in
this audit).

Focus files:
- `rtl/mac/irq_agg.v` — peripheral IRQ aggregator (priority encode + NMI latch).
- `rtl/core/commit.v` — `take_irq` / `arch_sr.I` / `ipl_ack` / saved-frame build.
- `rtl/core/exception.v` — frame-push µop generator + RTE pop µop generator.
- `rtl/core/m68k_core_commit.vh` / `m68k_core_rename.vh` — internal vs external agg merge.
- `rtl/fpga_top_peripherals.vh` — external agg + NMI button sync chain.
- `rtl/fpga_top_debug_ctrl.vh` / `rtl/core/debug/debug_ctrl.v` — host JTAG IRQ inject.
- `rtl/core/decode/decode_semantics.v` / `decode_uop_assemble.v` — STOP / RTE / privileged decode.

Severity legend: H = blocks Mac OS boot under reasonable conditions; M
= correctness corner case that won't show on simple boot but is real;
L = ambiguity / cleanliness / dead code.

---

## Headline bugs

### 1. `take_irq` does not gate on instruction-boundary (`rob_is_last_uop`) — H

`rtl/core/commit.v:841-843` defines
```verilog
wire take_irq  = can_commit && !store_commit_wait &&
                 !rob_exc && !lsu_busy &&
                 ipl_nonzero && (ipl_above_mask || ipl_is_nmi);
```

There is **no `rob_is_last_uop` term**.  Per 68040 PRM §8.1.1
("Exception Recognition") IRQs are recognised between instructions, NOT
between µops within the same instruction.  Today, take_irq fires the
moment ROB head is complete and non-faulting — including when head is
e.g. the µop3-of-7 of a MOVEM crack or the µop2-of-2 of a BSR.

Concrete corruption path (MOVEM.L `<reglist>,-(A7)` with N≥2 stored
registers):
1. µop0 (first store) retires, AXI write commits to memory.
2. µop1 (second store) reaches head, complete.
3. take_irq fires; `flush_keep_tag <= rob_tag-1` squashes µop1+ +
   sets `exc_fire_fault_pc <= rob_pc` (= start-of-MOVEM PC).
4. Handler runs.  RTE redirects to start-of-MOVEM.
5. MOVEM re-executes ALL stores from the top — including µop0's,
   which already wrote.  Memory is corrupted by the duplicate write
   pair (the first re-store hits the same address, then A7 is
   advanced again).  For predecrement -(A7), this manifests as the
   pre-IRQ stack frame being overwritten by the duplicate first
   store, then the IRQ frame's saved PC being lost.

BSR (2-µop crack) is the same shape: first µop is `STORE ret_pc to
A7-4`, second is the actual branch.  IRQ between them double-pushes
ret_pc and the BSR target is never taken.

Fix is to AND `rob_is_last_uop` (already plumbed at `commit.v:32`) into
the `take_irq` predicate.  Trace exception (`take_trace`) at
`commit.v:875` correctly already gates on the last µop indirectly via
`trace_pending` being set by retire of `last_uop` only.

### 2. `take_irq` can re-fire mid-IRQ-entry / mid-RTE / mid-sync-entry — H

`commit.v:2032`:
```verilog
ipl_ack             <= 1'b1;
commit_in_flight    <= 1'b0;     // <-- should be 1'b1
prev_a7_store       <= 1'b0;
```

Every other entry-fire path in this module sets
`commit_in_flight <= 1'b1` as the same-cycle re-fire guard: take_exc
(line 1434), take_finalize (1707), take_rte (1522), take_trace
(1956), exc_done IRQ branch (2624).  IRQ entry uniquely sets it to
`0`.  Combined with the µop-injection model (`exc_active` is hard-
zero — see bug #5 below), there is **no re-fire guard** for IRQ
entry.

Failure mode: IRQ at level N fires, `ipl_ack` pulses, `flush_en`
pulses with `keep_tag = rob_tag-1`.  Next cycle, ROB is empty
(`rob_valid=0`) so can_commit=0 and take_irq is blocked — for that
cycle.  Once the first injected STORE completes at head (a few
cycles later, lsu_busy drops between AXI bursts), can_commit goes
high again WITH the SAME OLD `arch_sr[10:8]` (level update happens
only at finalize, line 1664).  If `cpu_ipl` is still ≥ N (the
peripheral hasn't been ack'd by the still-not-running handler),
take_irq fires AGAIN, mid-injection.  flush squashes the in-flight
inject µops, exception.v's FSM is now stranded in `S_INJECT_PUSH`
waiting for `disp_accept` that will never come because ROB is
empty.  Deadlock or — worse — exception.v drops the second fire
silently (S_IDLE-only acceptance, see bug #5) and the second
`ipl_ack` clears the NMI latch for a non-existent dispatch.

A `commit_in_flight <= 1'b1` here is a one-line fix; pair with
gating `take_irq` on `state == S_IDLE` from exception.v.

### 3. `take_irq` does not block during in-flight RTE / sync-exc inject sequence — H

The check `!exc_active` on `can_commit` is dead because **exc_active is
permanently 0** in the µop-injection model
(`exception.v:75` retains the port for backward compat, `:317-352`
never set it to 1).  `exc_wait` is set ONLY for the legacy
"store-commit-fault" path (`commit.v:1668`); take_exc / take_rte /
take_finalize / take_irq all comment out exc_wait by design.

Result: between an IRQ fire and its finalize, OR between an RTE fire
and rte_finalize, OR between a sync-exc fire and its finalize, there
is no commit-side gate that holds off another take_irq except for
`commit_in_flight` (1 cycle) and incomplete inject µops.  Once the
inject sequence has any complete µop at head with lsu_busy=0 and
commit_in_flight=0, a fresh IRQ that satisfies `cpu_ipl >
arch_sr.I` (NMI always satisfies; level≥1 satisfies during user-mode
RTE pop where saved I is still 0) will preempt and corrupt the
in-flight sequence.

The fix is either to (a) re-purpose `exc_active` to mean "exception.v
state != S_IDLE" so the dead gate becomes live, or (b) add a new
`inject_in_progress` register set on take_exc/take_irq/take_rte and
cleared on inject_complete, and AND it into all four `take_*` gates.
(a) is cheaper.

### 4. STOP instruction is decoded as a no-op (does not load SR, does not wait for IRQ) — H

`rtl/core/decode/decode_uop_assemble.v:15504-15517`:
```verilog
else if (sem_sysop_is_stop) begin
    // PRM §4.183: STOP #imm loads SR from the immediate and
    // halts fetch until an IRQ ≥ SR.IPL.  Supervisor-only.
    // Sim-pragmatic: emit as UOP_SYS / SYS_NOP with
    // requires_supervisor=1 (matches legacy shape; the halt
    // side-effect is not modelled).
    uop_type            = `UOP_SYS;
    uop_op              = SYS_NOP;
    requires_supervisor = 1'b1;
    len_bytes           = 4'd4;
    uop_is_last         = 1'b1;
end
```

The opcode word + `#imm` are consumed (len=4) and execution
**continues** past STOP without any SR update or IRQ-wait.  ROM /
diagnostic firmware that does `STOP #0x2700` to halt-and-wait will
power right past the stop and continue executing whatever follows.

The most-obvious failure paths:
* Q700 ROM cold boot's "STOP after sad-mac" never actually halts; the
  CPU charges into garbage memory.
* Mac OS Process Manager's idle path uses STOP #0x2000 to drop IPL
  and wait for the next IRQ — today it still runs at full clip even
  when IRQ-only work is pending.

Low-blast fix: emit STOP as a new SYS subop (e.g. `SYS_STOP`), commit
applies `arch_sr <= imm[15:0]`, then sets a new `stop_pending`
register.  Add a `stop_pending` term to `can_commit` AND let
`take_irq` clear it when an IRQ at the new level fires.

### 5. `exc_active` is permanently 0 — dead gate on `can_commit` — M

`rtl/core/exception.v:75` declares `output reg active`, but the only
assignments are reset → 0 (`:317`), S_IDLE → 0 (`:351`), S_INJECT_WAIT
→ 0 (`:542`), S_DONE → 0 (`:550`).  No path ever sets it to 1.

`commit.v:735` reads it into `can_commit = ... && !exc_active && ...`,
but the term is dead.  This isn't a bug on its own (the µop-injection
model relies on `commit_in_flight` for serialisation), but it's a
load-bearing-zero — bug #3 directly exploits this gap.

The cleanup is two-line: assign `active <= (state != S_IDLE)` and let
the existing `can_commit` gate do its job.  Then remove the
`commit_in_flight <= 1'b0` set at `commit.v:2032` (bug #2's fix
follow-on).

### 6. `dbg_irq_inject_lvl` / `dbg_irq_inject_pulse` are wired but not connected — M

`rtl/core/debug/debug_ctrl.v:98-99` outputs the IRQ-inject bundle.
`rtl/fpga_top_debug_ctrl.vh:21-22, 68-69` declares wires and ties them
through.  `rtl/fpga_top_debug_vio.vh:57-58` routes them only to a VIO
observability bundle.  **No path ever drives cpu_ipl_ext or any
peripheral IRQ line from these signals.**

Result: the m68kctl JTAG `irq-inject` command is functionally a no-op
on the live FPGA — the host can write the level register and observe
the pulse on a VIO scope, but cpu_ipl never moves.  This breaks the
"deferred until +ipl=… support lands" plan called out in
`tb/tests/asm/exc_irq_during_store.s:19-24` and
`exc_priority_irq_vs_sync.s:17-21`.

The hookup is a 4-input OR into the external irq_agg's `nmi_edge`
pin (or a small mux that overrides cpu_ipl_ext for one cycle on the
inject pulse).  Trivial RTL; the gating story is what needs care
(prefer "OR with peripheral agg" so it's additive, not replacive).

### 7. Internal `irq_agg` in `m68k_core_commit.vh` is dead with all-zero inputs — L

`rtl/core/m68k_core_commit.vh:47-58`:
```verilog
irq_agg u_irq_agg (
    .clk(clk), .rst(rst),
    .via1_irq   (1'b0),
    .via2_irq   (1'b0),
    .scsi_irq   (1'b0),
    .scc_irq    (1'b0),
    .snd_irq    (1'b0),
    .rsvd_irq6  (1'b0),
    .nmi_edge   (1'b0),
    .ipl_ack    (ipl_ack),
    .ipl        (cpu_ipl_int)
);
```

Every input is hard-zero, so `cpu_ipl_int` is always 0.  The merge at
`m68k_core_rename.vh:748-749` `cpu_ipl = max(cpu_ipl_ext, cpu_ipl_int)`
is therefore an effective `cpu_ipl = cpu_ipl_ext`.  This isn't
incorrect — but the dead aggregator burns ~30 LUTs and the
double-aggregator architecture is confusing.  Either:
- delete the internal agg and rename `cpu_ipl_ext` → `cpu_ipl` (cleanest),
- or wire the internal agg from glue.v's peripheral fanout once a real
  Mac-internal-only agg is needed (e.g. for a CPU-side debug-injection
  path; see bug #6's hookup proposal).

### 8. `mac_top.v` does not expose `cpu_ipl_ext` — M

`rtl/mac_top.v` instantiates `m68k_core` but does NOT route
`cpu_ipl_ext` to any port (the input is left dangling).  Consumers of
mac_top (everything except fpga_top, which uses fpga_top_cpu.vh
directly) cannot drive an external IPL.  In particular the unit
testbench `tb/tb_top.cpp` instantiating mac_top has no
`cpu_ipl_ext` to drive — which is the root cause of the
"HARNESS NEEDS: testbench injecting cpu_ipl at a controlled cycle"
note in `exc_irq_during_store.s` and `exc_priority_irq_vs_sync.s`.

Trivial fix: add `input wire [2:0] cpu_ipl_ext` to mac_top.v's port
list and pass through to the m68k_core instance.  Then add a
`+ipl=<cycle>:<level>` plusarg to tb_top.cpp.

### 9. Saved frame's SR captures CCR via `arch_ccr_val`, but I-mask via `arch_sr[10:8]` — should both be the live boundary value — L

`commit.v:1878` (IRQ entry) and `:1390` (sync entry):
```verilog
exc_fire_saved_sr   <= {arch_sr[15:5], arch_ccr_val};
```

`arch_sr[15:5]` is the architectural supervisor half (T1/T0/S/M/I + 3
reserved bits), `arch_ccr_val` is the renamed CCR slot's current
value.  This is correct for sync exceptions where the trapping µop
hasn't yet retired, BUT for IRQ — because the head µop hasn't
retired either — there's a subtle ordering question: if a younger
flag-writer is in-flight whose tag is the live ratmap entry but
whose CDB broadcast hasn't landed at retire time, `arch_ccr_val` is
the **older** committed CCR.  In practice the IRQ takes a clean
boundary so any uncommitted flag-write is squashed by the flush —
making the older CCR the right thing to save.  But this needs an
explicit invariant comment, not just an implicit derivation.

Recommend: add a comment block near `commit.v:1878` confirming
`arch_ccr_val` is the committed-state CCR at the IRQ boundary, and
note that the squash semantics rely on it.

### 10. `exc_done`-branch (commit.v:2587-2728) is dead code under µop-injection — L

`exc_done` is permanently 0 (`exception.v:347`).  The `if (exc_done)
… else if (exc_done_is_irq)` blocks at `commit.v:2587-2728` —
including the explicit `arch_sr[10:8] <= exc_done_irq_level` write
at `:2707` — never execute.  Today, the SR.I update for IRQ entry
happens at `take_finalize` (`:1664`).  Both paths agree on the
update value, so this is a cleanup item, not a bug.  Removing the
dead branch would clarify ownership and shave ~140 lines.

### 11. RTE injected µops have `inj_noflush=1` but ROB ignores it — M

`exception.v:208`: `assign inj_noflush = 1'b1;` for both exception-entry
and RTE-pop µops.  But `rtl/core/rename/rob.v` never references
`noflush` (grep finds zero hits in rob.v for the term).

Consequence: an in-flight RTE pop sequence can be squashed by ANY
flush_en pulse — including a take_irq mispredict on ROB head.  The
RTE never completes; A7 stays at the pre-pop value; arch_sr.I stays
at the supervisor level.  Dependent on bug #3's gating, this might
or might not show up; with bug #3's fix in place it stops being a
trigger.  But the unimplemented `noflush` contract should be either
honoured or removed from exception.v.

Cleanest fix is to honour `noflush` in rob.v's flush handler (skip
invalidate of e_noflush=1 entries), and add a gate at decode/
dispatch that no `noflush` µop ever co-mingles with `noflush=0`
µops at the head (already guaranteed by inj_valid gating rn_ready).

### 12. No double-fault HALT modelling — M

Per 68040 §8.4.5.4: a bus error (or address error) during an exception
entry's frame push or vector-table read is a "double fault" —
the CPU latches HALT and stops fetching/executing.  Today there is
no `halt_signal` register, no path that latches a fatal state.  An
injected store that hits `rob_exc` would re-fire `take_exc` from
the inject µop's head, leading to recursive double-frame-push
recursion (or, depending on inject µop noflush handling, immediate
deadlock).

The deferred test `exc_double_fault_halt.s`
(`tb/tests/deferred.txt:exc_double_fault_halt`) exists.  Recommended
addition: a `core_halted` register, set on detection of "exc_fire
while exception.v.state != S_IDLE" or "rob_exc on a noflush µop";
expose to dbg via debug_ctrl + freeze fetch.

### 13. NMI rising-edge detection vs ipl_ack race — L (tested, low risk)

`irq_agg.v:97-114`:
```verilog
wire nmi_rise = nmi_edge && !nmi_edge_q;
…
if (nmi_rise)        nmi_pending <= 1'b1;
else if (ipl_ack && ipl == 3'd7) nmi_pending <= 1'b0;
```

Edge wins over ack on the same cycle.  Documented behaviour in the
header.  Looks correct.  One subtle point: `ipl == 3'd7` is the
combinational output (which itself is a function of `nmi_pending`),
so the ack path effectively reads "nmi_pending was already 1 last
cycle".  There's a rare 1-cycle window where:
- cycle N: nmi_pending=1, CPU pulses ipl_ack=1, ipl=7 → cleared.
- cycle N: a new NMI rising edge arrives same cycle → set wins.
- cycle N+1: nmi_pending=1, ipl=7 → CPU sees the same NMI again.

This is the documented "fresh NMI overrides ack" behaviour, but
it's worth explicitly testing — there's no current directed test
for it.

### 14. `ipl_ack` does not differentiate sync from IRQ — L

`commit.v:1695`: in `take_finalize`, `ipl_ack <= inject_is_irq;` — fine,
only pulses for IRQ entry finalize.

`commit.v:2031`: in `take_irq`, `ipl_ack <= 1'b1;` — pulses on the
TAKE cycle too.  So for a single IRQ event, `ipl_ack` pulses TWICE:
once at take_irq fire, once at take_finalize.  Both pulses target
the same NMI-clear edge for level 7.  Not a bug today (clearing an
already-cleared latch is a no-op), but if a "spurious-fresh-NMI
between take_irq and take_finalize" arrives, the second ack would
clobber it.  Recommend dropping the `:2031` pulse in favour of the
take_finalize pulse only — the take_irq cycle's flush_en already
guarantees the take_irq won't re-fire same-cycle, and the
finalize's pulse is the one that truly says "the IRQ has been
serviced into the handler."

### 15. No CORE_DEBUG IRQ-entry print — L

Other key boundary events have `$display` blocks under CORE_DEBUG (e.g.
`commit.v:1378-1383` for ILLEGAL/A-line/F-line).  IRQ entry
(`take_irq` block, `:1864-1933`) has no such print.  `[EXC FIRE]`
prints from `exception.v:378-380` cover both sync and IRQ once
the fire propagates, but a commit-side `[IRQ TAKE]` line with
`level / pc / saved_sr / a7_before / arch_sr_I` would be a
lifesaver during live HW bring-up — particularly for the via1
T1-storm bisect-on-stack-corruption story already in `via1_t1_irq_storm.s`.

(Recommendation only; do not add yourself per audit-only mandate.)

### 16. Spurious-interrupt vector not implemented (L for Q700, but documented gap)

Per 68040 PRM, vec 24 is generated when an IACK cycle returns BERR.
Q700 is all-autovector (no IACK cycle is ever performed), so this
is correct-by-construction for Mac targets.  However:
- `exc_fire_vec <= {5'd0, cpu_ipl} + 8'd24` would map cpu_ipl=0 to
  vec 24.  `take_irq` is gated on `ipl_nonzero`, so this can't fire.
  Correct.
- A future non-Mac target running a 68040 with real IACK cycles
  would need the spurious vec path.  Today none of the bus
  termination paths can drive a "BERR-during-IACK" event because
  no IACK is ever issued.  Document as "intentionally absent" in
  `docs/exceptions.md`.

---

## Mac OS bootability impact

| Phase                                              | Symptom                                                                                                       | Status |
|----------------------------------------------------|---------------------------------------------------------------------------------------------------------------|--------|
| Q700 ROM cold boot, light IRQ rate                 | Works today (live FPGA boot reaches the sad-mac splash; IRQs fire occasionally for VBL).                      | OK     |
| Q700 ROM cold boot, sustained T1 IRQ storm         | Eventually wedges (`via1_t1_irq_storm.s` reproduces the SP drift).  Bug #1 (mid-MOVEM IRQ) is the canary.    | H      |
| ROM "Fail-safe STOP after sad-mac"                 | Charges past STOP; runs garbage.  Bug #4.                                                                     | H      |
| System 6 init (rare IRQ contention)                | Possibly works.  The IRQ during MOVEM-on-stack-frame patterns are rare in System 6 (plain 68000 idiom).      | OK?    |
| System 7 Process Manager idle (STOP-based)         | Hot loop instead of low-power wait; minor symptom, not a hang, but performance regression.  Bug #4.          | M      |
| System 7 cooperative dispatch + Sound IRQ          | Bug #1 + bug #2 conspire: inflight MOVEM during VBL IRQ → corrupted register save area → process state lost. | H      |
| Mac OS 8.x nanokernel preemption                   | Same as above + MSP-based stack switching meets bug #2's mid-inject re-fire window.  Will not boot.          | H      |
| Power-key NMI (live FPGA test)                     | Works (`btn[1]` → `nmi_btn_pulse` → `nmi_edge` → vec 31).  No flash hang in current bring-up.                 | OK     |
| Test harness IRQ injection (`exc_irq_during_*.s`)  | All deferred today.  Bug #6 + bug #8 are the entire reason these can't run.                                   | DEFER  |

---

## Recommended fixes

File-by-file change list, ordered by risk × value.

### `rtl/core/commit.v`

(1) **Bug #1**: gate `take_irq` on instruction boundary.  Line 841:
```verilog
wire take_irq  = can_commit && !store_commit_wait &&
                 !rob_exc && !lsu_busy &&
                 rob_is_last_uop &&            // ← ADD
                 ipl_nonzero && (ipl_above_mask || ipl_is_nmi);
```

(2) **Bug #2**: change `commit_in_flight <= 1'b0` to `1'b1` at line 2032.

(3) **Bug #3 (a)**: pair with exception.v line 351: `active <= (state != S_IDLE);` plus reset clear at line 317.  Then `take_irq` (and friends) inherit the gate via `can_commit`'s existing `!exc_active` term.

(4) **Bug #14**: drop `ipl_ack <= 1'b1` at line 2031 — keep the take_finalize-side pulse only.

(5) **Bug #10 (cleanup)**: delete `exc_done` branches at `:2587-2728` (one big `if (exc_done) … else …` block).  Keep only the µop-injection paths.

### `rtl/core/exception.v`

(1) **Bug #3 (a) cont.**: assign `active <= (state != S_IDLE);` so the
existing `can_commit && !exc_active` gate becomes meaningful.  Same
fix obviates bug #5.

(2) **Bug #11**: either honour `inj_noflush` in rob.v (preferred) or
delete the `inj_noflush` port and the wire at `:208`.

### `rtl/core/decode/decode_uop_assemble.v` + `decode_semantics.v`

(1) **Bug #4**: replace SYS_NOP-emit-on-STOP with a new `SYS_STOP`
sub-op that carries the SR-immediate operand.  At commit retire of
SYS_STOP, do `arch_sr <= imm[15:0]; ccr_restore_en <= 1'b1;
ccr_restore_val <= imm[4:0]; stop_pending <= 1'b1;`.  Add
`!stop_pending` to `can_commit` (so subsequent dispatched µops
can't retire) and let `take_irq` clear `stop_pending` on entry.

### `rtl/core/rename/rob.v`

(1) **Bug #11**: add `e_noflush[]` per-entry, set on dispatch from a
new `disp_noflush` port (already plumbed through dispatch arbiter
since `inj_noflush` reaches there).  At flush, skip invalidate of
entries with e_noflush=1.

### `rtl/mac_top.v`

(1) **Bug #8**: surface `cpu_ipl_ext` as an input port; pass through
to the inner `m68k_core` instance.  Default-tie to 3'd0 in
`rtl/mac_top.v` consumers that don't drive it.  Update
`tb/tb_top.cpp` to expose a `+ipl=cycle:level` plusarg.

### `rtl/fpga_top_peripherals.vh` (or new `rtl/fpga_top_irq.vh`)

(1) **Bug #6**: connect `dbg_irq_inject_pulse` + `dbg_irq_inject_lvl`
to a small mux that overrides cpu_ipl_ext for one cycle, OR (better)
mix into the external irq_agg's `nmi_edge` (level 7 pulse) +
add a `level_force` port to irq_agg for levels 1-6.  The pulse
already lives in core_clk domain via debug_ctrl's reset gating.

### `rtl/core/m68k_core_commit.vh` + `m68k_core_rename.vh`

(1) **Bug #7 (cleanup)**: delete the internal `irq_agg` instance,
rename `cpu_ipl_ext` → `cpu_ipl`, drop the max-merge logic in
m68k_core_rename.vh.

### `docs/exceptions.md`

Document bug #16 (no spurious-interrupt path) explicitly as
"Q700 is autovector-only; vec 24 is intentionally unreachable".

---

## Estimated scope

| Bug   | LOC delta | Risk | Files touched                             |
|-------|-----------|------|-------------------------------------------|
| #1    | +1        | low  | commit.v (line 841)                       |
| #2    | -1 / +1   | low  | commit.v (line 2032)                      |
| #3+#5 | +2        | low  | exception.v (line 351 + reset)            |
| #4    | +30       | med  | decode_uop_assemble.v, commit.v, ROB plumb|
| #6    | +20       | med  | fpga_top_peripherals.vh + irq_agg port    |
| #8    | +10       | low  | mac_top.v + tb_top.cpp plusarg            |
| #10   | -140      | low  | commit.v (delete dead branch)             |
| #11   | +20       | med  | rob.v noflush + dispatch port             |
| #12   | +25       | med  | commit.v (halt latch + dbg port)          |

Total ≈ +110 / -140 LOC.  Highest risk is #4 (STOP) — touches
decode-emit + commit-retire + commit fetch-stall, must coexist with
take_irq's stop-clear path.  Highest reward-per-risk is #1 (one line,
unblocks Mac OS multitask — the biggest single Mac-OS-bootability
delta in the audit).

---

## Test plan

### Existing tests that already exercise these paths

| Test                                  | Status   | Bug exercised |
|---------------------------------------|----------|---------------|
| `via1_t1_irq_storm.s`                 | PASS     | Indirectly #1 (TRAP surrogate; not a real mid-MOVEM stress) |
| `priv_user_stop_traps.s`              | PASS     | #4 (only for STOP-from-user vec-8 path; supervisor STOP not exercised) |
| `exc_irq_during_store.s`              | DEFER    | #1 + #6 + #8 (cannot inject IRQ from harness today) |
| `exc_priority_irq_vs_sync.s`          | DEFER    | #6 + #8 (same) |
| `exc_double_fault_halt.s`             | DEFER    | #12 |
| `exc_trace_t1.s`                      | DEFER    | trace path; orthogonal but mentioned for completeness |
| `exc_msp_mode_round_trip.s`           | PASS     | None directly; sets up MSP/ISP for IRQ-entry M-clear test |

### New directed tests recommended (DEFER until fix lands)

These are filed as `.s` sources under `tb/tests/asm/` but NOT
activated in the regression gate (kept in deferred manifest until
the corresponding fix ships).

1. **`irq_during_movem.s`** — bug #1 regression.  Set up a memory
   region with sentinel; `MOVEM.L D0-D7,-(A7)` with D0..D7 = known
   pattern; testbench raises IPL=1 mid-MOVEM (after first 2 stores
   per cycle-counter); IRQ handler reads the sentinel at the
   pre-MOVEM A7 location.  PASS sentinel: A7 frame matches the
   first MOVEM's full register save once + IRQ frame on top, no
   duplicate stores.  FAIL sentinel: pre-MOVEM stack location
   shows an unexpected D-register value (= duplicated first-store
   leak).  See file for full body.

2. **`irq_during_bsr.s`** — bug #1 regression for the 2-µop crack.
   `BSR sub` with an IRQ raised between the two µops; verify
   ret_pc is on the stack exactly once and the BSR target is taken
   exactly once.

3. **`irq_during_rte.s`** — bug #3 regression.  Set up a saved frame
   on SSP, execute RTE, raise IPL≥1 between RTE pop µops; verify
   final SR / PC / SP all match what RTE would have produced
   without the interruption (the IRQ takes effect AFTER RTE
   completes, on the resumed PC's instruction boundary).

4. **`stop_wait_for_irq.s`** — bug #4 regression.  Set IPL mask via
   STOP #0x2000, raise IPL=1 from harness; verify STOP halts
   fetch (no instructions retire) until the IRQ raises, then
   handler runs once, RTE returns past STOP, mainline continues.

5. **`stop_no_wait_when_already_pending.s`** — bug #4 corner.  STOP
   #0x2000 with IPL already at 1; STOP must immediately fire IRQ
   (not wait), since the new-mask "0" allows level-1 to break in.
   Different from above in that the IRQ is pre-existing, not
   incoming.

6. **`irq_inject_via_debug.s`** — bug #6 regression.  Boot, then
   m68kctl `irq-inject 5`; verify vec 29 fires.  Cannot live in
   `tb/tests/asm/` (needs JTAG); add a tb-fpga-top-rom-style
   harness instead, OR a tb_top.cpp `+ipl=` directed test.

These six sources cover bugs #1, #3, #4, #6 directly; bugs #2 (mid-
inject re-fire) and #11 (RTE-noflush) are reachable by carefully
timing the IPL-injection harness pulse — bug #2 needs IPL still
asserted after take_irq fires, bug #11 needs IPL asserted during
RTE pop.

### Filed as part of this audit

- `tb/tests/asm/irq_during_movem.s` — DEFER, repro for bug #1.
- `tb/tests/asm/irq_during_bsr.s` — DEFER, repro for bug #1.
- `tb/tests/asm/irq_during_rte.s` — DEFER, repro for bug #3.
- `tb/tests/asm/stop_wait_for_irq.s` — DEFER, repro for bug #4.

(`stop_no_wait_when_already_pending.s` and `irq_inject_via_debug.s`
deferred until +ipl harness lands; spec captured in this audit.)

### Fuzz extension

`tools/fuzz/gen_program.py` does not currently emit STOP, RTE-from-
nested-frame, or mid-MOVEM-IRQ sequences.  When bug #4 fix lands,
extend the generator to emit STOP #imm with random IPL masks and
let Musashi golden-ref check the resume PC.  For bug #1, the
generator needs harness IPL injection — see bug #6 — before random
mid-MOVEM IRQ stress can run.

---

## Status (2026-05-03 rolling)

This audit is read-only.  No RTL changes landed in the audit commit.
Four directed-test sources have been filed under `tb/tests/asm/` and
added to `tb/tests/deferred.txt`; they will fail until the
corresponding bug fixes ship.  Bugs are tracked here, not in the
TaskList yet — promote into tasks as the follow-up wave starts.
