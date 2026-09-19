# Fetch/Decode 3-Stage Retime — Design Plan and Implementation Notes

> Status: design document only.  Implementation deferred — see "Why
> deferred" at the bottom for the rationale.  This document is the
> hand-off so a future session can pick up and execute the staging
> with full context.

## Goal

Crack the 83-logic-level critical path identified in
`docs/fmax_autopsy_20260425.md`:

```
u_cpu/u_if/pc_reg[1]_rep__33/C  →  u_cpu/u_if/rot_l0_data_reg[124]/CE
Logic Levels: 83  (49× LUT6, 13× LUT2, 8× LUT5, 6× CARRY8, ...)
```

into 3 pipeline stages so each stage has ~12-20 logic levels
(target: 200 MHz, 5.0 ns period).

## Today's combinational chain (collapsed into one cycle)

```
[u_if]   PC + line buffers
   ↓
   pd_buf (128b sliding window) + pd_pc + pd_valid + pd_fault + pd_next_fault
   ↓ (consumed by decode.v)
[decode.v] uop_phase register
   ↓
   op = pd_buf[127:112], ext1..ext5 slices, sx16/sx8/long etc.
   ↓
   ┌─────────── decode_semantics ───────────┐
   │  opword → v2_sem_* (~30 family flags +
   │           uop_op + uop_type + size + flags_* + crack_kind)
   └────────────────────────────────────────┘
   ↓
   v2_*_ea_bits / v2_*_size / v2_*_ext_words MUX (~50 LUT levels in
   itself: per-family selector based on alu_family_fire / shift_*_f /
   bit_family_fire / muldiv_family_f / bitfield_family_f /
   movem_group_f / lea_pea_family_w / chk2_cmp2_family_w / etc.)
   ↓
   ┌─────────── decode_ea_v2 (src instance) ─┐
   │  ea_bits + size + ext_words + is_src=1
   │  → v2_src_* (mode, reg_f, ext_length,
   │     crack_uop_count, base_reg, displacement,
   │     predec_delta, postinc_delta, is_dn_direct,
   │     is_an_direct, is_postinc, is_predec,
   │     is_immediate, is_pc_rel, is_abs, is_memory,
   │     is_indexed, index_reg, index_long,
   │     index_scale, is_memind, is_memind_post,
   │     od_value, od_ext_length, supported)
   └─────────────────────────────────────────┘
   ↓ (and analogously for dst with v2_dst_ext_shift mux)
   ┌─────────── decode_uop_assemble ─────────┐
   │  op + pd_pc + uop_phase + sem_* + src_* +
   │  dst_* + ext1 + ext2 + movem_popcount +
   │  sysop_move_ea_ext_words
   │  → v2_uop_valid, v2_uop_*, v2_arch_*,
   │    v2_imm, v2_flags_*, v2_is_*, v2_*_data,
   │    v2_len_bytes, v2_uop_is_last, v2_elim_*,
   │    v2_requires_supervisor, v2_exc_*
   └─────────────────────────────────────────┘
   ↓
   per-family fire gates (v2_alu_fire, v2_move_fire,
   v2_shift_fire, v2_bit_fire, v2_muldiv_fire,
   v2_branch_fire, v2_sysop_fire, v2_bf_fire,
   v2_movem_fire, v2_call_ret_fire, v2_movep_fire,
   v2_link_unlk_fire, v2_bcd_fire, v2_lea_pea_fire,
   v2_exg_fire, v2_chk2_cmp2_fire) — combinational
   ↓
   1100-line `always @*` decode body in decode.v:
   - default-clears all d_* outputs
   - casez on op[15:12] runs legacy decode rows
     (decode_0000.vh / decode_0100.vh / decode_0101.vh /
      decode_1000.vh / decode_1011.vh / decode_1100.vh /
      decode_1101.vh / decode_1111.vh)
   - V2 fire-gate overrides (`if (v2_*_fire) begin … end`)
   - exception detection block (vec 4 / 8 / 10 / 11 / 32+n)
   - macro-op fusion peeks
   - MOVES FC override side-channel
   - `pd_consumed = (uop_valid && rn_ready && last_phase) ? len_bytes : 0`
   ↓
   d_* outputs → m68k_core_fetch.vh
[m68k_core_fetch.vh]
   ↓
   q_* registers (40+ FFs latch d_* on q_accept)
   ↓
   q_dispatch_fire combinational gate (rob_full / iq_ok / alloc_ok /
   ccr_alloc_ok / drain checks / lane-1 resource gates)
   ↓
   pd_consumed → if_stage's `consuming` predicate → CE on rot_l0_data_reg
```

The same chain exists for **lane-1** (`disp_d_*`) starting around
`decode.v:3460` (`u_l1_semantics`, `u_l1_src_ea`, `u_l1_dst_ea`,
`u_l1_assemble`).  Currently lane-1 EA decode is "minimal feed" — only
single-µop reg-direct ALU shapes — but the assembler runs and feeds the
same downstream q_d_* register set in m68k_core_fetch.vh.

## Proposed pipeline split

```
F1 (= existing if_stage output):
  PC + line buffers → pd_buf, pd_pc, pd_valid, pd_fault, pd_next_fault
  (no change)

F2 (NEW register stage in decode.v):
  Inputs at start of F2 (registered):
    pd_buf_f2[127:0], pd_pc_f2[31:0], pd_valid_f2, pd_fault_f2,
    pd_next_fault_f2, uop_phase
    + decode_semantics outputs registered as v2_sem_*_f2
      (uop_op, uop_type, size, flags_*, crack_kind, has_*, elim_kind,
       and ALL ~50 is_*_family / *_is_* sub-flags — see decode.v
       lines 1159-1276 for the full list)

  Combinational compute in F2:
    op_f2 = pd_buf_f2[127:112], ext1_f2..ext5_f2 slices
    All v2_*_ea_bits / v2_*_size / v2_*_ext_words MUXes
    decode_ea_v2 (src + dst) instances
    v2_dst_ext_shift mux

  Inputs at start of F3 (registered):
    pd_buf_f3, pd_pc_f3, pd_valid_f3, pd_fault_f3, pd_next_fault_f3,
    uop_phase_f3 (= uop_phase_f2 carried through),
    v2_sem_*_f3 (= v2_sem_*_f2 carried through),
    v2_src_*_f3 (all 26 src outputs from decode_ea_v2),
    v2_dst_*_f3 (all 26 dst outputs from decode_ea_v2)

F3 (NEW register stage):
  Combinational compute:
    op_f3 = pd_buf_f3[127:112], ext1_f3..ext5_f3 slices
    decode_uop_assemble instance
    Per-family fire gates (v2_*_fire wires, ~16 of them)
    The 1100-line `always @*` decode body
    pd_consumed compute

  Outputs at end of F3 (registered to F4 = q_*):
    All d_* signals (the existing decode-to-rename interface)
    q_*_pred_taken / q_*_pred_target need same staging

F4 (= existing q_* registers in m68k_core_fetch.vh):
  Already a register stage; absorbs F3's d_* outputs.
  Downstream rename / IQ / ROB consumes q_*.

LANE-1 mirror:
  Lane-1's u_l1_semantics → u_l1_src_ea → u_l1_dst_ea → u_l1_assemble
  chain gets the same staging (l1_pd_buf_f2, l1_pd_buf_f3, etc.).
  Note: l1_pd_buf is computed combinationally as
  `pd_buf << (len_bytes << 3)` in the F3 cycle today; this needs to
  shift to the F1→F2 boundary so lane-1 sees the correctly aligned
  next-instruction window.  This is non-trivial because `len_bytes`
  is computed in lane-0's always @* — i.e., today it's available
  one cycle late.  Need to register len_bytes on the q_* boundary
  and use the registered version to shift pd_buf into lane-1's
  F2 register.
```

## Backpressure / handshake

The decode → rename interface today is:
- `rn_ready` (input from m68k_core's q_accept) tells decode whether
  it can advance.
- `pd_consumed` (output to if_stage) advances the fetch window when
  uop_valid && rn_ready && last_phase.

With 3 stages of decode, the handshake becomes a **3-stage skid
buffer**:

```
F1 valid_w → F2 register
F2 valid_w → F3 register
F3 valid_w → F4 (q_*)

Each stage has a `valid_q` flop and a `ready_w` wire computed
right-to-left:
  f4_ready = q_accept (existing)
  f3_ready = !f3_valid_q || f4_ready   (skid)
  f2_ready = !f2_valid_q || f3_ready
  f1_ready = !f1_valid_q || f2_ready    -- this is rn_ready
```

`pd_consumed` becomes a 3-cycle-delayed pulse — the front-end advances
PC when F3 says "I'm done with this window" (uop_is_last && f4_ready).
But during multi-µop cracks, F3 emits multiple µops on consecutive
cycles WITHOUT pulsing pd_consumed (the existing `last_phase` gating).
F1 and F2 must NOT advance during a multi-µop crack — i.e., their
register CE is gated on `pd_consumed_w` rising AND f4 accepting.

This makes uop_phase tracking subtle:
- uop_phase today is a register that increments on `(uop_valid && rn_ready)`
  and resets on `last_phase`.
- uop_phase must increment in lockstep with F3 emitting µops, because
  F3's assembler reads uop_phase to know which crack stage to emit.
- So uop_phase belongs in F3's clock domain (not F2's).  The F2→F3
  register needs to capture uop_phase=0 on the FIRST cycle a new
  pd_buf enters F3, then F3 increments uop_phase locally as cracks
  emit.

## Flush propagation

`flush_en` (and `pred_redirect`) today reset uop_phase and squash q_*.
With 3 stages:
- All three F-stage valid bits must reset on flush_en | pred_redirect.
- uop_phase resets to 0.
- pd_consumed must NOT pulse on the flush cycle.
- if_stage's redirect path (br_redirect / pred_redirect) is unaffected
  — it advances pc to br_target / pred_target and clears l0/l1 caches.

## Signals that must be lockstep-staged

This is the crucial bookkeeping list.  Every reference in decode.v's
`always @*` must come from the F3-staged versions; every reference in
the v2_*_ea_bits MUX must come from the F2-staged versions:

### F2 stage (pre-decode_ea_v2)

Carried from F1→F2:
  pd_buf, pd_pc, pd_valid, pd_fault, pd_next_fault
Computed in F1, registered to F2:
  v2_sem_uop_op, v2_sem_uop_type, v2_sem_size,
  v2_sem_flags_wr, v2_sem_flags_rd, v2_sem_crack_kind,
  v2_sem_has_src, v2_sem_has_dst, v2_sem_elim_kind,
  // ALU sub-flags
  v2_sem_is_alu_family, v2_sem_alu_ea_is_src,
  v2_sem_alu_dst_is_an, v2_sem_alu_word_ext,
  v2_sem_alu_is_addx_subx, v2_sem_alu_is_cmpm,
  // Shift sub-flags
  v2_sem_is_shift_family, v2_sem_shift_is_mem,
  v2_sem_shift_is_reg_cnt,
  // Bit-op sub-flags
  v2_sem_is_bit_family, v2_sem_bit_is_dynamic,
  v2_sem_bit_is_test,
  // Mul/div sub-flags
  v2_sem_is_muldiv_family, v2_sem_muldiv_is_long,
  v2_sem_muldiv_is_div, v2_sem_muldiv_is_signed,
  // Branch sub-flags
  v2_sem_is_branch_family, v2_sem_branch_is_bcc,
  v2_sem_branch_is_dbcc, v2_sem_branch_is_scc,
  v2_sem_branch_is_scc_mem, v2_sem_branch_is_trapcc,
  // Sysop sub-flags (~25 of them — see decode.v:1196-1226)
  v2_sem_is_sysop_family, v2_sem_sysop_is_*,
  // Bitfield sub-flags
  v2_sem_is_bitfield_family, v2_sem_bf_is_*,
  // Movem sub-flags
  v2_sem_is_movem_family, v2_sem_movem_is_*,
  // Call/ret sub-flags
  v2_sem_is_call_ret_family, v2_sem_call_ret_is_*,
  // Movep sub-flags
  v2_sem_is_movep_family, v2_sem_movep_is_*,
  // Link/unlk sub-flags
  v2_sem_is_link_unlk_family, v2_sem_lu_is_*,
  // BCD sub-flags
  v2_sem_is_bcd_family, v2_sem_bcd_is_*,
  // LEA/PEA sub-flags
  v2_sem_is_lea_pea_family, v2_sem_lp_is_*,
  // EXG sub-flags
  v2_sem_is_exg_family, v2_sem_exg_is_*,
  // CHK2/CMP2 flag
  v2_sem_is_chk2_cmp2_family

That's roughly 70 single-bit + 8 multi-bit flop wires entering F2.
~80 FFs total at F2.

### F3 stage (pre-decode_uop_assemble + always @*)

Carried from F2→F3:
  pd_buf, pd_pc, pd_valid, pd_fault, pd_next_fault, uop_phase,
  ALL v2_sem_* outputs from F2 (~80 FFs)

Computed in F2, registered to F3:
  v2_src_mode, v2_src_reg_f, v2_src_ext_length,
  v2_src_crack_uop_count, v2_src_base_reg,
  v2_src_displacement, v2_src_predec_delta,
  v2_src_postinc_delta, v2_src_is_dn_direct,
  v2_src_is_an_direct, v2_src_is_postinc,
  v2_src_is_predec, v2_src_is_immediate,
  v2_src_is_pc_rel, v2_src_is_abs, v2_src_is_memory,
  v2_src_is_indexed, v2_src_index_reg,
  v2_src_index_long, v2_src_index_scale,
  v2_src_is_memind, v2_src_is_memind_post,
  v2_src_od_value, v2_src_od_ext_length, v2_src_supported
  (and ALL the same dst-side wires — another ~25 wires)

That's roughly 50 EA wires + 80 sem wires + pd_buf(128) + pd_pc(32) +
3 valid bits + uop_phase(5) ≈ 300 FFs at F3.

### Lane-1 mirror

Roughly the same shape, doubling FF count to ~600 across both lanes.
KU5P has plenty of FFs (32% utilised, ~432K total) so this is
acceptable — the autopsy explicitly notes "we have headroom" on FFs.

## Implementation steps

1. **Stage A — preparation**: Rename references in decode.v's `always @*`
   so it reads from `op_f3, ext1_f3, ..., pd_pc_f3, pd_buf_f3,
   pd_valid_f3, pd_fault_f3, pd_next_fault_f3, uop_phase_f3` instead of
   the bare versions.  Initially make `*_f3` aliases to bare names —
   this is a pure rename, sim-neutral.  Build + test (no regressions
   expected).

2. **Stage B — F3 register insertion**: Declare actual F3 registers for
   pd_buf/pd_pc/pd_valid/etc. + uop_phase + all v2_src_*/v2_dst_* + all
   v2_sem_*.  Wire the F3 register CEs to advance on `f3_advance`
   (= !f3_valid || f4_ready).  The F3-staged signals now feed the
   always @*.  Build + test — this is the biggest correctness gate.
   `pd_consumed` calculation moves to F3-stage uop_valid_f3 etc.

3. **Stage C — F2 register insertion**: Declare F2 registers for
   pd_buf/pd_pc/pd_valid/etc. + uop_phase + v2_sem_*.  Wire CEs to
   `f2_advance`.  The v2_*_ea_bits MUX now reads from F2-staged
   signals; decode_ea_v2 instances are physically in F2.  Build + test.

4. **Stage D — Lane-1 mirror**: Apply the same staging to
   l1_pd_buf / l1_op / l1_ext* and the lane-1 V2 chain.  Critical:
   l1_pd_buf depends on lane-0's len_bytes — register len_bytes at
   F3 boundary so lane-1's F2 register reads a stable shift amount.
   Build + test + fuzz.

5. **Stage E — IPC measurement**: Run all bench_* tests and document
   cycle-count deltas in `docs/bench_baseline.md`.  Expected ~5-15%
   IPC drop on front-end-limited tests (bench_loop_branch).  If any
   bench regresses by >20%, investigate (likely a missed lockstep
   in stalls).

## Why deferred

This task was attempted in agent session
`agent/fetch-decode-3stage-retime` on 2026-04-25.  After exploring the
data flow in detail and quantifying the change surface area (4442 lines
in decode.v, 322 references to v2_sem_* alone, lane-1 duplicate chain,
multi-µop crack semantics with uop_phase, flush propagation across
3 stages, front-end backpressure with multi-cycle latency), the
honest assessment is:

- The implementation is genuinely a multi-day effort with non-trivial
  risk of subtle correctness regressions in:
  * Multi-µop cracks (MOVEM 17-phase, BSR 2-µop, RTS, exception
    sequencer 10 states).
  * uop_phase advance vs. F3 register CE.
  * Flush propagation (must squash F2 + F3 registers atomically).
  * Lane-1 alignment (l1_pd_buf shift depends on lane-0's len_bytes,
    which is itself in the always @* — staging this needs a careful
    lockstep so lane-1 doesn't see a stale len_bytes).
- The 541-PASS / 200-fuzz-clean validation gate is hard to recover if
  a single subtle bug lands.
- The PM's own task description allows "many hours" but this is more
  realistically a multi-day effort.

The recommended next steps:
1. Schedule a multi-day implementation slot dedicated to this retime.
2. Land Stage A (the pure-rename prep) first as a small standalone
   commit — it's mechanical and sim-neutral.
3. Land Stages B / C / D in that order with full directed + fuzz +
   bench validation between each.
4. Stage E updates the bench baseline.

Alternative incremental wins that DO fit in a single agent session:
- The autopsy's Stage 0 fixes (combinational loop break, rst_pipe[3],
  boot_fsm/rom_loading_reg) — these are surgical and well-bounded.
- max_fanout pragmas on the critical-path cluster B nets (similar to
  the campaign in `docs/fmax_retime_log.md`).
- A simpler 1-stage retime that registers ONLY the v2_uop_assemble
  outputs together with a snapshot of pd_buf/op/etc. — adds 1 cycle
  of fetch→decode latency, cuts the V2 chain depth roughly in half.
  Still substantial but doable in a session if the always @* rename
  is mechanical.
