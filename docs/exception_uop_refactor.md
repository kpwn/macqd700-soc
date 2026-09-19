# exception.v µop-injection refactor — Phase 1 (entry only)

## Why

`rtl/core/exception.v` currently owns its own `dc_req/dc_addr/dc_rdata`
port, muxed against the LSU at `exc_active`.  This is a **shadow memory
path** that does not see the LSU's store buffer, store-buffer-forwarding,
or in-flight LSU split-LONG state.

Concrete failure: the Q700 ROM's vec-25 install at `0x40847b9c-0x40847ba0`
is `move.l %a3, (%a2)` with `%a2 = 0xFF32` — an **unaligned LONG store**
that the LSU splits into two beats (write to `0xFF30` low half + write
to `0xFF34` high half).  ~30 instructions later the ROM unmasks IRQs
(`move #0x2000, sr` at `0x40847bf2`).  T1 fires almost immediately,
exception.v dispatches via `*(VBR + 25*4) = *(0xFF32)` using its own
split-LONG read path.  On the live FPGA only 6 of 61 IRQ entries
successfully reach the installed handler — the other 55 read stale
data, dispatch to garbage, and cascade.

Routing the vector read (and the frame push) through the LSU eliminates
the shadow path: the LSU's store buffer covers the read by construction,
split-LONG logic is shared, and the dcache sees a single coherent client.

## Scope (Phase 1)

* **Refactor: exception ENTRY only** — frame push (1..30 word stores)
  and vector read (1 LONG load).  Both currently live in
  `S_FRAME_PUSH_*` and `S_READ_VECTOR_*` in `exception.v`.
* **Out of scope (Phase 2):** RTE pop, MOVEC-VBR completion, sync
  exceptions' fault stacking from a different fault site.  The current
  state machine for RTE stays intact.

## Mechanism

`exception.v` becomes a **µop injector** at the dispatch boundary.  When
`exc_fire` rises:

1. Decode is paused (`disp_en = 0` from the decode side; `exc_active` is
   already a global gate).
2. exception.v drives the same `disp_*` signals decode normally drives,
   but with µops it generates from the current vec / fmt / fault_pc.
3. The dispatched µops flow through rename → ROB → iq_mem → LSU →
   dcache identically to ordinary stores/loads.  The LSU's store
   buffer covers them automatically.
4. The last injected µop carries `is_exc_finalize=1`.  When it reaches
   commit head, commit performs the existing exc_done atomic update:
   `arch_sr.S/I/T` write, A7 swap if entering from user mode, redirect
   to the loaded handler PC (carried via `cmpl_data`).

### Injected µop sequence (format-0, 8 bytes, IRQ entry from supervisor)

| µop | Type   | Op             | Address              | Data                | Notes |
|-----|--------|----------------|----------------------|---------------------|-------|
| 0   | STORE  | normal         | new_a7 + 0           | cur_sr (16b)        | imm_is_data, fc=5 |
| 1   | STORE  | normal         | new_a7 + 2           | cur_fault_pc[31:16] | imm_is_data, fc=5 |
| 2   | STORE  | normal         | new_a7 + 4           | cur_fault_pc[15:0]  | imm_is_data, fc=5 |
| 3   | STORE  | normal         | new_a7 + 6           | format_vec_word     | imm_is_data, fc=5 |
| 4   | LOAD   | EXC_VEC_LOAD   | vbr + (vec << 2)     | (loaded into cmpl)  | fc=5, is_exc_finalize |

For format-2 frames (TRAPV / CHK / privilege) the sequence is 6 µops
of the same shape; format-7 (bus/addr error) is 30 µops.  The
exception.v generator emits one µop per cycle until the frame is
complete, then the vector load.

`new_a7` is read from `PHYS_SSP_TAG` (or `PHYS_ISP_TAG` per `arch_sr.M`)
**directly**, not via the ratmap — Phase A's reserved slot invariant.
The decrement (`a7 - frame_size`) and per-µop offsets (+0/+2/+4/+6) are
computed by exception.v as immediates baked into each µop's address
field at injection time.

### IRQ entry from user mode

Same sequence, but inject one extra "USP write" µop first that captures
`arch_a7_val` into the USP slot via the `sp_slot_write_*` port.  This
mirrors the current logic at `commit.v:1549`; commit-side machinery
absorbs it.

## Required µop encoding additions

### 1. `is_exc_finalize` (1 bit)
Set on the **last** injected µop of an exception sequence.  At commit,
the head-µop's exc_finalize bit drives the same `exc_done_*` actions
that today fire when `done_pulse && !exc_done_is_rte` in
`commit.v:2242-2275`.  Inert (= 0) on every µop decode emits today.

### 2. `noflush` (1 bit)
Marks µops that must not be squashed by a flush.  Exception µops carry
this — once injection starts, the sequence runs to completion.
Conceptually identical to the existing `commit_store_en` discipline for
LSU stores but at the per-µop granularity.  Inert (= 0) on every µop
decode emits today.

### 3. Reuse of existing fields

| Need                          | Existing field                                                    |
|-------------------------------|--------------------------------------------------------------------|
| Per-µop store data (no PRF read) | `iss_imm_is_data + iss_imm_data` (already on UOP_STORE path)   |
| FC=5 supervisor access         | `iss_fc_override_valid + iss_fc_override` (used today by SFC/DFC) |
| SP source not via ratmap       | `prf_sp_slot_val` (Phase A, already wired into commit.v:1547)     |
| Branch redirect from load data | `cmpl_br_target / cmpl_br_taken` (BSR/RTS path; reusable)         |

## Validation plan

* `make lint` clean.
* `tb-fpga-top-rom` reaches a deeper PC than today (currently hangs at
  PC=0xac for unrelated reasons; that has to be unblocked first or
  worked around with a scoped tb).
* `tb/tests/asm/unaligned_long_vector_dispatch.s` (added 2026-05-03 in
  Phase-0 work) — must still pass; add a new variant where a real LSU
  split-LONG store is **in-flight** (still in the store buffer) when
  the TRAP fires, to catch what the current sim doesn't.
* `via1_t1_irq_storm.s` continues to pass.
* Fuzz 200/200 PASS regression gate.
* Re-bitstream + halt-after-N sweep — the divergence at N≈10M
  should disappear (D3 should reach 10 in the Q700 timer-test loop).

## Phase 1 commit plan

1. **Foundation** (landed, 3c32344d): docs, `UOP_NOFLUSH_BIT` +
   `UOP_EXC_FINAL_BIT` field allocations in `uop_pkg.v`,
   `exception_uop_gen.vh` helper functions, included from exception.v
   (defined-but-unused).
2. **Injector** (landed, c3c0913a): the µop-bundle field generators
   above + tb-exception-uop-gen unit test (44/44 PASS, including
   the unaligned vec-25 → 0xFF32 case).
3. **Arbiter wiring** (landed, 3c32344d / c3c0913a / 6575ee47 /
   0f305d6d): dispatch arbiter mux on every q_* signal at the
   lane-0 latch in m68k_core_fetch.vh, with q_d_valid suppression
   on lane-1.  Routes injection through lane-0 because
   `q_dispatch_fire` is gated on `q_valid`.  `disp_accept` feedback
   port from m68k_core_commit.vh into exception.v drives the FSM's
   push_word_idx advance.
4. **Cutover FSM states** (landed, 3e92d8e1, partial): S_INJECT_PUSH
   and S_INJECT_LOAD added.  S_INJECT_WAIT is a stub awaiting
   step 2b (cmpl plumbing).  `EXC_UOP_INJECT` remains undefined by
   default — runtime is bit-identical to pre-refactor.
5. **Step 2b**: cmpl-side plumbing — `is_exc_finalize` per-entry bit
   in rob.v, `rob_finalize_cmpl_pulse` / `rob_finalize_cmpl_data`
   outputs, wiring through `m68k_core_commit.vh` to a new
   `cmpl_finalize_*` input on exception.v, and commit.v handling
   of finalize µops at retire (atomic SR/A7/PC redirect using
   `cmpl_finalize_data` as handler PC).
6. **Vector BTB**: tiny 16-entry table mapping `vec[3:0]` → handler PC
   so the front-end can redirect on `exc_fire` *before* the LOAD µop
   resolves — see §"Phase 1.4 vector BTB" below.
7. **Cutover**: flip `EXC_UOP_INJECT` default on; old dc_req path
   becomes dead code.  Validation gauntlet (make test, fuzz, all unit
   tbs) under EXC_UOP_INJECT=1, then bitstream rebuild and HW sweep.
8. **Cleanup**: delete the dead state machine; collapse mux in
   `m68k_core_memory.vh:413` (`dc_req_sel = exc_active ? exc_dc_req : dc_req`).

## Phase 1.4 step 3 — debug status (snapshot 2026-05-03 evening)

| pass | snapshot |
|------|----------|
| 504/57 | first run (baseline 558/3 on default) |
| 525/36 | step 3a + 3b (this commit chain): 5 fixes — `active=0` in cutover, `requires_supervisor=0` for inject µops, `exc_wait` gated by `ifdef EXC_UOP_INJECT` in take_exc/take_irq, no `rob_pop` alongside flush in take_exc, `is_rts=1` for the finalize µop so LSU drives `cmpl_br_target` with the load result.  +21 PASS. |

`exc_trap0` is the canary — it now PASSES end-to-end through the
µop-injection path in 329 cycles (committed=73, last_pc at handler's
infinite-loop sentinel write).  The fundamental architecture is
working.

Remaining 33 failures (excluding the 3 pre-existing baseline fails:
cinv_line_basic, cpush_line_basic, rom_checksum_loop):

* **RTE-heavy** (~15 tests): exc_*_rte, mmu_*_rte, exc_user_vbr_rte_matrix.
  RTE still uses the legacy dc_req path; entry via cutover + RTE
  via legacy may have state-coordination issues (e.g. exception.v's
  cur_is_rte register, or commit.v's exc_wait/exc_active state
  set by take_rte that take_finalize doesn't clear).  **Phase 2
  of the refactor — RTE µop-injection.**
* **Format-2 frames** (~7 tests): chk_*, chk2_*, div*, exc_aline_*,
  illegal_opcode_trap.  Either an fmt-2-specific bug in the inject
  path or some semantics-mismatch.  Worth re-running with DEBUG=1
  on chk_bounds (simplest) to localise.
* **Format-7 frames** (~3 tests): exc_bus_error_*.  30-µop
  sequence; one corner-case likely.
* **MMU + RTE** (~3 tests): mmu_nested_fault, mmu_pagefault_rte_basic,
  mmu_wp_fix_rte.  Mostly Phase 2 territory.
* **Privileged** (~5 tests): movec_vbr*, moves_pri*.  These take
  `take_priv_exc` which folds into `take_exc` — already gated.
  Likely a different issue, e.g. the µop being marked
  `requires_supervisor` on dispatch causing the trap to never
  retire (we explicitly set `q_requires_supervisor=0` for inject
  µops, but the user µop that causes the priv violation still has
  it set).

## Phase 1.4 step 3 — original debug snapshot

`make test EXC_UOP_INJECT=1` first run: **504/57** (vs 558/3 baseline
on the default path).  All 54 new failures are exception-handling
tests — TRAP, CHK, RTE, illegal-opcode, privilege, MMU, plus the
two new tbs (`unaligned_long_vector_dispatch`, `via1_t1_irq_storm`).
This confirms the µop-injection path **is being exercised**; the
structural plumbing is wired correctly end-to-end (lint clean,
default-off path bit-identical to pre-refactor).  The failures are
in the runtime semantics of the new path — likely candidates:

* **Frame writes** — confirm the LSU's split-LONG store helpers
  (`m68k_mem_split_*`) interpret the frame addresses correctly
  for word stores at SSP+i*2 (every odd offset from SSP=0xFECE
  hits ea[1:0]=2 → split needed even for word stores at certain
  byte offsets).  Verify against `tb-exception-uop-gen` which
  already validated the EA computation.
* **Vector LOAD result routing** — the finalize µop uses
  `is_branch=1 + is_load=1` so LSU drives `cmpl1_br_target` with
  the read data.  Confirm LSU actually does this for arbitrary
  loads, not just RTS — earlier comments suggested RTS uses
  `is_rts=1` to gate the branch-target capture.  May need a
  finalize-specific gate in LSU.
* **ROB head ordering** — finalize µop is pushed AFTER 4 frame
  stores.  Stores commit-store-en at retire; LSU buffers stores
  until commit_store_en.  If LSU doesn't issue the LOAD until
  prior stores drain, the pipeline could deadlock if dispatch
  waits for the LOAD to complete.  Check `iss_ready` / store
  buffer interaction.
* **SR.S transition timing** — the finalize handler sets S=1 at
  retire, but the injected µops *during* injection should already
  see S=1 for FC=5 supervisor data access.  The fc_override path
  bypasses arch_sr.S read, so this should be correct — but worth
  confirming via a directed test (e.g., trap from user mode).
* **flush_keep_tag at finalize-retire** — the new commit handler
  sets `flush_keep_tag <= rob_tag`, popping the µop and flushing
  newer.  Verify this doesn't squash legitimate non-finalize
  µops dispatched after.

Recommended debug order: pick `exc_trap0` (simplest TRAP #0
exercise), build sim with `EXC_UOP_INJECT=1 DEBUG=1`, capture the
boundary-event stream, find the first divergence vs the default-off
build's expected behaviour.

## Phase 1.3 arbiter sketch

The decode→dispatch latch at `m68k_core_fetch.vh:680-728` captures
`d_*` (lane-0) and `d_d_*` (lane-1) signals into `q_*` / `q_d_*`
each cycle that `dispatch_fire` holds.  The arbiter is a *single
mux* on the source side of that latch.

```verilog
// New exception-side dispatch port (driven by exception.v when
// inj_valid=1 — combinational decode of (state, push_word_idx,
// vec, fmt, a7_new) using exception_uop_gen.vh helpers).
wire        inj_valid;
wire [2:0]  inj_uop_type;
wire [5:0]  inj_uop_op;
wire [1:0]  inj_uop_size;
wire [31:0] inj_imm;
wire [31:0] inj_imm_data;
wire        inj_imm_is_data;
wire        inj_has_src_a;
wire [4:0]  inj_arch_src_a;     // PHYS_SSP_TAG read goes through
                                 //  rename's reserved-slot path (not
                                 //  ratmap)
wire        inj_has_dst;        // 0 for stores; 1 for vector LOAD
                                 //  (writes nothing arch-visible — its
                                 //  cmpl_data is consumed by commit at
                                 //  finalize)
wire        inj_is_store;
wire        inj_is_load;
wire        inj_fc_override_valid;  // = 1
wire [2:0]  inj_fc_override;        // = 3'd5
wire        inj_noflush;            // = 1
wire        inj_is_exc_finalize;    // = (idx == push_count)
wire [31:0] inj_uop_pc;             // synthetic PC (for ROB / dbg)

// Single mux at the latch:
wire        sel_inject = inj_valid;            // exception.v dictates

q_d_type        <= sel_inject ? inj_uop_type   : d_uop_type;
q_d_op          <= sel_inject ? inj_uop_op     : d_uop_op;
q_d_size        <= sel_inject ? inj_uop_size   : d_uop_size;
q_d_imm         <= sel_inject ? inj_imm        : d_imm;
q_d_imm_data    <= sel_inject ? inj_imm_data   : d_imm_data;
q_d_imm_is_data <= sel_inject ? inj_imm_is_data: d_imm_is_data;
q_d_is_store    <= sel_inject ? inj_is_store   : d_is_store;
q_d_is_load     <= sel_inject ? inj_is_load    : d_is_load;
// ... etc. for every q_d_* signal that comes from d_d_* today
q_dispatch_fire <= sel_inject ? inj_valid      : decode_fire;
```

Key invariants the arbiter enforces:

* **No mid-stream mixing**: when `inj_valid` is high, decode is
  treated as if `decode_fire = 0`.  The decode `pd_consumed` strobe
  is gated off, so the predecode buffer holds its position.  When
  the exception sequence completes (last µop dispatched), decode
  resumes from where it was.
* **Lane-1 (2-wide decode) is suppressed** during injection.
  `q_d_valid <= 0` when `sel_inject`, so iq_int / iq_mem only see
  the lane-0 exception µop on inject cycles.  Single-issue is
  fine — the exception sequence is bandwidth-bounded by frame
  push count, not pipeline width.
* **Flush behaviour**: if a flush event occurs *during* injection
  (e.g. the squashed user µop's PC update arrives late), the
  injected µops in flight carry `noflush=1` and must NOT be
  rolled back.  The flush logic in `rob.v` and `rat.v` consults
  the per-µop `noflush` bit and skips them.
* **A7 read**: the SSP push µops need `a7_new = ssp - frame_size`.
  exception.v computes this combinationally from `prf_sp_slot_val`
  (the Phase-A protected slot) — no rename allocation, no PRF
  read port pressure.

### Where the wires live

| Signal              | Driver               | Consumer                   |
|---------------------|----------------------|----------------------------|
| `inj_*`             | `exception.v` (new outputs, combinational from FSM state + helpers) | Arbiter mux in `m68k_core_fetch.vh` |
| `inj_valid`         | `exception.v` (`state == S_INJECT && push_idx <= push_count`) | Arbiter mux              |
| `disp_uop_complete` | `lsu.v` cmpl0 / cmpl1 (existing `cmpl_en` path, identifies the finalize µop by ROB tag) | `exception.v` to advance push_idx |
| `done_handler_pc`   | (legacy path for now) eventually `commit.v` consumes the finalize µop's `cmpl_data` | exception.v → IF redirect |

The new `inj_*` wires only exist when `EXC_UOP_INJECT` is defined.
With it undefined, the arbiter compiles to `sel_inject = 1'b0` and
the muxes degenerate to the existing `d_*` paths — bit-identical to
today.

## Phase 1.4 vector BTB (speculative handler fetch)

**Idea**: keep a small N=16-entry table indexed by `cur_vec[3:0]`
holding the last-observed handler PC for that vector.  On
`exc_fire`, predict `handler_pc = vec_btb[vec[3:0]]` and redirect
IF *immediately* — in parallel with the frame-push µops being
injected.  The vector LOAD µop still fires as part of the injected
sequence; if its result matches the prediction, no extra work.
If it mismatches, we treat it like a branch mispredict: flush the
front-end of speculatively-decoded handler instructions, redirect
to the actual handler PC.

For Mac OS the warm-up is one IRQ — vec 25 trains the BTB to its
hot Sound Manager handler PC, and every subsequent vec-25 IRQ
predicts correctly.  The 4-cycle frame push and the 5-10 cycle
LOAD round-trip turn from a serial stall into background work.

### BTB shape

```verilog
// 16 entries × {valid, 32-bit PC} = 16 × 33 = 528 bits — distrib RAM
reg        vec_btb_valid [0:15];
reg [31:0] vec_btb_pc    [0:15];

// Lookup (combinational from cur_vec[3:0]) at exc_fire:
wire [3:0] vec_idx = cur_vec[3:0];
wire       vec_pred_valid = vec_btb_valid[vec_idx];
wire [31:0] vec_pred_pc   = vec_btb_pc   [vec_idx];

// Training (registered, fires at exc_done with done_is_irq | done_is_sync_exc):
always @(posedge clk) begin
    if (rst) begin
        for (i = 0; i < 16; i = i + 1) vec_btb_valid[i] <= 1'b0;
    end else if (exc_done && !exc_done_is_rte) begin
        vec_btb_valid[exc_done_vec[3:0]] <= 1'b1;
        vec_btb_pc   [exc_done_vec[3:0]] <= exc_done_handler_pc;
    end
end
```

### Why 16 entries

Vectors 0–15 cover the Mac's hot autovec range (vec 24..31 = 0x18..0x1F,
masked to 0x8..0xF in a 16-entry table).  Vec 25 (level-1 IRQ) is the
~kHz-rate one that dominates handler-fetch traffic.  Sync exceptions
(vec 4=illegal, vec 6=CHK, vec 7=TRAPV, vec 9=trace) also benefit but
fire much less frequently.  Larger tables are cheap in distributed RAM
but add training-stale risk; 16 is a sweet spot for KU5P resources.

### Speculation control

* `exc_fire` AND `vec_pred_valid` ⇒ `if_pred_redirect_en = 1`,
  `if_pred_redirect_pc = vec_pred_pc`.  Same port the BPU already
  drives for branch predictions — extend the priority encoder to
  let exception predictions win (they're the more disruptive event).
* Front-end starts decoding at `vec_pred_pc` while the µop sequence
  injects.  Decoded µops post-redirect get tagged with the IRQ's
  ROB tag boundary so they can be squashed if the actual handler
  PC mismatches.
* `inj_valid` drives the dispatch arbiter as in Phase 1.3, so the
  injected µops still issue.  They have higher priority than the
  speculatively-decoded handler µops (which sit in the predecode
  buffer behind a "wait for finalize" gate).
* When the LOAD µop resolves: compare `cmpl_data` against
  `vec_pred_pc`.  Match: handler µops behind the gate dispatch
  normally.  Mismatch: flush handler-side decode, redirect to
  actual `cmpl_data`.

### Cost estimate

| Resource | Phase 1.3 arbiter | Phase 1.4 BTB | Combined |
|---|---:|---:|---:|
| LUTs    | ~50 (mux + dispatch FSM in exception.v) | ~80 (lookup + train) | ~130 |
| FFs     | ~20 (FSM state)        | ~530 (16 × 33)        | ~550 |
| BRAM    | 0                      | 0 (distrib)           | 0 |
| Fmax impact | trivial            | one new redirect path into `if_stage` (~3 ns budget today, plenty of headroom) | trivial |

### When to land

After Phase 1.3 cutover proves the µop path is functionally correct,
Phase 1.4 is purely an IPC win — no risk of regression to correctness
because mispredicts are caught at LOAD-resolve time.  Feature-flag
behind `ENABLE_VEC_BTB` for an A/B benchmark of IRQ-handling latency
on the same boot path.

## Phase 2 (future) — RTE µop-injection + fmt-7 fix

Phase 1.4 step 3b validates the µop-injection architecture for
exception ENTRY.  Phase 2 extends it to RTE (return from exception)
and addresses an iq_mem-full corner case for fmt-7 (bus/addr error)
frames.

### 2a — RTE µop-injection

Today RTE pop still uses `exception.v`'s legacy dc_req path.
Phase 2a routes it through LSU using the same pattern as entry:

* 4 LOAD µops to read SSP+0/2/4/6 (fmt-0; 6 for fmt-2; 30 for fmt-7).
* Each LOAD captures a 16-bit word.  Cumulative state lives in
  exception.v's `cur_rte_*` regs (SR, PC_hi, PC_lo, format).
* Last LOAD has an `is_rte_finalize` bit (analogous to
  `is_exc_finalize`).  At its retire, commit performs the atomic
  RTE action: restore SR (may flip S=1→0, swap A7), redirect PC
  to popped value.
* Routing: cmpl_data of each LOAD needs to reach exception.v.
  Two options:
  1. New ROB output `rte_pop_data_pulse` + `_idx` + `_value` that
     fires on cmpl for any µop with `is_rte_pop_word`.  exception.v
     watches and accumulates into cur_rte_*.
  2. Each LOAD allocates a phys_dst (one of TMP slots), commit
     reads PRF[tmp] at finalize time to assemble.  Avoids the new
     pulse but uses 4 phys regs per RTE.

Option 1 is cleaner; option 2 reuses existing infra.  Lean toward 1.

### 2b — fmt-7 frame iq_mem-full deadlock

fmt-7 (bus/addr error) is 30 µops but iq_mem is 8-deep.  After 8
stores dispatch, iq_mem fills.  LSU services one store at a time;
stores drain on commit_store_en (which fires when the store
retires from ROB).  With 30 stores in flight and only 8 fitting
in iq_mem, the chain stalls.

Diagnosis needed: are stores actually completing through LSU and
retiring from ROB, or is something blocking the chain?  If they
retire, iq_mem should drain.  If not, find the blocker.

Possible fixes:
* Widen iq_mem to 32 entries — straightforward but costs LUTs.
* Fast-path fmt-7 store coalescing — recognize the contiguous
  aligned writes and emit a single multi-beat AXI burst.  Big win:
  cuts 30 µops to 1.
* Emit fmt-7 frame as a series of LONG stores instead of WORD
  stores (15 µops vs 30) — modest improvement.

fmt-7 is the bus/addr error path — used only when something
catastrophic happens (e.g., MMU page fault, double-bus-error).
On most workloads it's never exercised.  Could defer 2b until
after 2a lands.

### 2c — Decode-pause coordination (low-risk follow-up)

`q_dispatch_fire` pulses for inject µops, which `rn_ready` reflects
to decode as "your packet was consumed".  Decode advances
`pd_consumed` and silently drops the next user µop.  *In practice*
the redirect at finalize-µop retire flushes the predecode buffer
so this doesn't cause correctness issues — but it IS wasted decode
work.  Fix: gate `pd_consumed` on `!exc_inj_valid`.  One-line.

### 2d — Privileged-µop interaction

Tests like `movec_vbr*` and `moves_pri*` may fail due to the user
µop having `q_requires_supervisor=1` set by decode.  This makes
`q_rob_drain_req=1` for the user µop, gating its dispatch on
`rob_empty`.  In a sequence where the priv µop is preceded by
other µops, this could deadlock the dispatch.  Worth checking
whether `take_priv_exc` cleanup needs the same treatment as
`take_exc`/`take_irq`.

## Risks (per agreed-on session 2026-05-03)

* **Mid-injection flush** — solved by `noflush=1` on all injected µops.
* **Speculative S in ratmap** — irrelevant; injected µops carry FC=5
  metadata and read SP from the protected slot.
* **In-flight SR readers** — keep their old SR phys tag; predate the
  exception.  After commit boundary, fresh decode reads the new SR.
* **Self-modifying handler install** — fixed *by construction* once
  exception reads share the LSU store buffer with the just-issued
  store.  This is the Q700 ROM bug we're chasing.
