# Exceptions — synchronous + asynchronous story

How the OoO core takes synchronous exceptions (illegal inst, TRAP,
bus error, etc.) and asynchronous interrupts (IPL levels from VIA1
/ VIA2 / SCC) and delivers them precisely at commit.  Read this
before spawning any phase-2 exception work.

Companion docs:
- [`gameplan.md`](gameplan.md) §"Phase 2" — exception-path is a
  phase-2 prerequisite
- [`peripheral_arch.md`](peripheral_arch.md) — peripherals that
  drive the IPL pins
- [`memhier.md`](memhier.md) §"MMU" — page-fault interaction

---

## 68040 exception model (reference, compressed)

**256-entry vector table.**  Base pointer is `VBR` (vector base
register, supervisor-only, accessed via MOVEC).  Each vector is a
32-bit handler PC.  Vector number × 4 + VBR = handler address.

Canonical vectors we must handle (by Mac OS boot):

| Vec | Purpose | Class | Phase |
|-----|---------|-------|-------|
| 0–1 | Reset SSP + Reset PC | special | boot |
| 2   | Access (bus) fault | sync | 2 |
| 3   | Address error | sync | 2 |
| 4   | Illegal instruction | sync | 2 |
| 5   | Divide by zero | sync | 3 |
| 6   | CHK / CHK2 | sync | 3 |
| 7   | TRAPcc / TRAPV | sync | 3 |
| 8   | Privilege violation | sync | 2 |
| 9   | Trace (single-step) | sync | 3 |
| 10  | A-line trap | sync | 2 (**critical — Mac OS Toolbox is 100 % A-line**) |
| 11  | F-line trap | sync | 2 (FPU dispatch) |
| 14  | Format error (RTE bad frame) | sync | 3 |
| 15  | Uninitialized interrupt | async | 2 |
| 24  | Spurious interrupt | async | 3 |
| 25–31 | Autovector level 1–7 | async | 2 |
| 32–47 | TRAP #0 – #15 | sync | 2 |
| 48–54 | FPU exceptions | sync | 3 (FPU bringup) |
| 64–255 | User-defined / NuBus | vectored | 3 |

**Stack frame formats.**  68040 supports multiple formats.  For phase
2 we implement:
- **Format 0 (4-word frame)**: SR + PC — most synchronous traps + IRQs.
- **Format 2 (6-word frame)**: adds PC-of-faulting-inst.  Used for
  bus error, address error, MMU.

Phase 3 adds the remaining formats (trace, FPU frame-B, etc.).

**Supervisor mode.**  On any exception entry:
1. Save current SR.
2. Set SR.S ← 1 (supervisor).
3. Clear SR.T1 ← 0, SR.T0 ← 0 (trace disabled in handler).
4. For IRQs: set SR.I ← level-just-taken.
5. For interrupts: select ISP (SR.M = 1).  For others: select SSP.
6. Push frame to selected stack: SP ← SP − framesize; mem[SP+0] =
   SR, mem[SP+2] = PC (+ format bits + vector offset + any frame-
   specific fields).
7. PC ← mem[VBR + vec × 4] (read via supervisor data cycle).
8. Resume fetch at new PC.

**RTE (return from exception)**:
1. Pop SR from mem[SP+0], advance SP.
2. Pop PC from mem[SP+2], advance SP.
3. Check format word matches expected — if not, raise Format Error
   (vector 14).
4. Fetch resumes at the restored PC with the restored SR.

**Three stack pointers.**
- **USP** (user SP) — when SR.S = 0; arch reg A7.
- **SSP** (supervisor SP) — when SR.S = 1, SR.M = 0; arch reg A7.
- **ISP** (interrupt SP) — when SR.S = 1, SR.M = 1; arch reg A7.
The ACTIVE A7 is whichever SR bits select.  Swapping on exception
entry + RTE is part of the SR-update logic.

**IPL pins.**  3-bit level input (`ipl[2:0]`) driven externally by
the Mac peripheral block.  Encoding: 0 = no IRQ pending, 1–6 =
prioritised IRQ level, 7 = NMI (can't be masked).  Active IRQ is
taken when `ipl[2:0] > SR.I[2:0]`.  Level 7 is edge-triggered to
avoid perpetual NMIs.

---

## Synchronous exceptions — the RTL path

### Detection sources

Each pipeline stage can detect and raise a per-μop exception.  The
source is annotated on the μop's ROB entry so commit can dispatch
precisely.

| Stage | Detects | Vector | Module |
|-------|---------|--------|--------|
| decode | Illegal opcode | 4 | `decode.v` |
| decode | A-line (op[15:12] = 1010) | 10 | `decode.v` |
| decode | F-line (op[15:12] = 1111), FPU disabled | 11 | `decode.v` |
| decode | Privileged inst, SR.S = 0 | 8 | `decode.v` |
| decode | TRAP #n literal | 32+n | `decode.v` |
| ALU | Divide by zero | 5 | `alu.v` / `mul_div.v` |
| ALU | CHK / CHK2 out of bounds | 6 | `alu.v` |
| ALU | TRAPV, V = 1 | 7 | `alu.v` |
| AGU / LSU | Unaligned access | 3 | `lsu.v` |
| LSU | Bus error (BRESP = SLVERR / DECERR) | 2 | `lsu.v` |
| MMU | Page fault | 2 (with frame format 2) | `mmu.v` |
| commit | Format error (RTE bad frame) | 14 | `commit.v` |

### Propagation — new ROB fields

Add to each ROB entry:
- `exc_valid` (1 bit) — this μop raised an exception
- `exc_vec` (8 bits) — vector number to take
- `exc_fault_pc` (32 bits) — PC of the faulting inst (for format-2
  frame)
- `exc_fault_addr` (32 bits) — address causing the fault (bus /
  address error / page fault).  For split LSU accesses, this must be
  the original EA of the instruction, not the aligned beat address.

These are populated on the cmpl channels (cmpl0 from ALU, cmpl1
from LSU, cmpl for MMU when it gets built).  Decode-time exceptions
(illegal, A-line, F-line, privilege) are latched into the ROB at
dispatch — no execute needed; the μop carries its fate from
decode.

### Commit-time dispatch

Normal commit path today: ROB head retires → arch_ccr updates →
branch redirect if mispredicted.

Exception path extension:
1. At the current head entry, check `exc_valid`.  If 0, normal retire.
2. If `exc_valid = 1`:
   a. Squash all newer uops (reuse branch-mispredict flush
      machinery: assert `flush_en`, wipe ROB, free phys regs via
      cRAT rollback).
   b. Stall fetch.
   c. Enter the **exception sequencer** microprogram (see below).
   d. The faulting μop itself is "retired" without updating arch
      state.  For page-fault retry it'll be re-fetched when the
      handler returns.

### Exception sequencer microprogram

Lives in a new module `rtl/core/exception.v`.  On exception trigger:

```
state 0: SAVE_SR
    // Store current SR into an internal register so we can push it
    exc_sr_save <= arch_sr;

state 1: UPDATE_SR
    // Enter supervisor.  For IRQ: set SR.I.  Clear trace.
    arch_sr.S  <= 1;
    arch_sr.T1 <= 0;
    arch_sr.T0 <= 0;
    if (is_irq) arch_sr.I <= irq_level;
    // Select ISP for IRQ, SSP otherwise
    active_a7_sel <= (is_irq && arch_sr.M) ? ISP : SSP;

state 2: COMPUTE_FRAME_SZ
    // Format 0 (4 words) or Format 2 (6 words + 2-byte filler)
    frame_sz <= (vec == 2 || vec == 3) ? 12 : 8;

state 3: PUSH_PC_LO
    axi_addr <= active_a7 - frame_sz + 4;
    axi_wdata <= saved_fault_pc[15:0];
    // ...

state 4: PUSH_PC_HI
    // ...

state 5: PUSH_SR_PLUS_FORMAT
    // Word 0: frame format (4 bits) + vector offset (12 bits) + SR
    vector_offset <= {vec[7:0], 2'b00};
    axi_wdata <= {format_code, vector_offset[11:0], exc_sr_save[15:0]};

state 6..7: PUSH_EXTRA (format 2 only)
    // fault addr, etc.

state 8: LOAD_HANDLER_PC
    axi_addr  <= vbr + (vec << 2);
    // read → handler_pc

state 9: JUMP
    // Fetch redirect to handler_pc, clear flush, resume.
    pc_redirect <= 1'b1;
    pc_redirect_target <= handler_pc;
    active_a7 <= active_a7 - frame_sz;
```

Latency ~12 cycles steady-state (a few AXI writes for the frame
push, one AXI read for the vector, one cycle to redirect fetch).
Good enough for phase 2; phase 4 can pipeline it.

The sequencer is a single in-flight machine — at most one
exception in progress at a time.  Nested exceptions (bus error
during handler push) are detected and escalate to double-fault →
reset (phase 3 concern).

### Supervisor / stack-pointer selection

Add to the architectural state in `commit.v`:
- `arch_sr[15:0]` — status register.  CCR already there (arch_ccr).
  Merge: arch_sr = {T1, T0, S, 0, M, 0, I[2:0], 0, 0, 0, arch_ccr}.
- `usp[31:0]`, `ssp[31:0]`, `isp[31:0]` — three stack pointers.
- The "active A7" is a combinational mux from {USP, SSP, ISP}
  selected by {SR.S, SR.M}.

Integer renaming (RAT) treats arch A7 as usual.  At commit of a μop
that updates A7, write back to whichever of USP/SSP/ISP is active
this cycle (determined by committed SR).

On supervisor entry (exception taken):
- Save outgoing A7 to the departing stack (USP if coming from user).
- Switch active mux to SSP or ISP.
- Sequencer pushes frame to that stack.

On RTE:
- Restore SR from stack first.  That determines which stack we're
  returning to.  Continue popping PC.
- Switch active A7 mux back.

MOVE USP (privileged) reads / writes the inactive USP register.

---

## Asynchronous exceptions — the IRQ story

### Peripheral IRQ aggregation

Each peripheral drives a single-bit `irq` line.  The aggregator
(new module `rtl/mac/irq_agg.v` or inside glue) encodes the highest
pending level into `ipl[2:0]`.

| Peripheral | Level | Vector (autovector) |
|------------|-------|---------------------|
| VIA1 (60 Hz VBL, ADB, RTC, sound)  | 1 | 25 |
| VIA2 (NuBus slot interrupts aggregated) | 2 | 26 |
| SCSI (NCR 5380 end-of-command) | 3 | 27 |
| SCC (serial RX/TX ready) | 4 | 28 |
| Sound DMA done | 5 | 29 |
| (reserved for real-time) | 6 | 30 |
| Power key (NMI) | 7 | 31 (edge-triggered) |

Autovector is simplest and matches 68040 Mac hw.  Vectored
interrupts (IACK cycle where the peripheral supplies its own vector
on the bus) are phase 3 when NuBus actually needs them.

### Commit-time sampling

IRQ is taken at commit head, not at any other point in the pipeline
— this keeps the exception precise.  Each commit cycle:

```
// priority: synchronous exception from head > IRQ > normal retire
if (head.exc_valid)
    take head's exception
else if (ipl[2:0] > arch_sr.I[2:0] || ipl == 3'd7)
    take IRQ level ipl, vector = 24 + ipl
    // squash head + everything newer; see sync path above
    // exc_fault_pc = head.pc
else
    normal retire
```

IRQ latency is bounded by commit throughput + sequencer depth +
handler fetch.  Worst case: ~30 cycles from ipl asserting to
handler's first inst committing.  Mac OS's VIA1 60 Hz tick
tolerates ~1 ms of latency easily.

### Level-7 NMI

Non-maskable (not gated by SR.I).  Edge-triggered to avoid
perpetual retaking while the line stays high (Mac power button is
level-latched, so the aggregator edge-detects for level 7
specifically).

### IPL CDC

The Mac peripheral block runs on the peripheral-bus clock; the CPU
core runs on the system clock.  If they're the same 200 MHz domain,
no CDC needed.  If different, the ipl[2:0] lines cross via a 2-FF
synchroniser in the aggregator.  Glitch tolerance: the aggregator
holds ipl stable for ≥ 2 cycles (peripheral IRQ lines are level-
driven and long-lived; glitches shouldn't happen).

---

## Exception precision — the OoO discipline

The rule is simple: **no architectural state mutation for a μop
newer than the faulting one**.  This is guaranteed by:

- **ROB in-order commit**: only retirements from the head modify
  arch state (arch_ccr, arch_sr, USP/SSP/ISP, VBR).  Squashed uops
  never touch it.
- **cRAT rollback**: on flush, the committed RAT mapping is restored
  (same mechanism used by branch mispredict today).  No "stuck
  rename" survives the squash.
- **Store buffer drain**: stores sit in the LSU post-commit FSM,
  not yet written to memory.  On exception, squash any
  post-exception-point stores that haven't yet issued AXI writes.
  Stores from the faulting μop itself (if it was a store)
  are discarded — the handler may re-execute.
- **Precise faulting-PC**: the exception sequencer uses the ROB
  entry's `pc` (PC of the faulting inst), not the current fetch
  PC.  This is correct because the ROB entry was stamped at
  dispatch.

**Bus error on store**: tricky case.  The store has already retired
from the ROB at issue time — by the time the LSU's AXI write gets
SLVERR, the ROB head has moved on.  Two options:

1. **Delayed retire**: keep the store in the ROB until AXI B
   response lands; if BRESP = OKAY, retire; if SLVERR / DECERR,
   raise exception at that ROB slot.  Penalises store-heavy
   workloads.
2. **Imprecise bus error**: accept that bus errors from stores are
   imprecise — the handler sees PC pointing at some inst AFTER the
   faulting store.  Mac OS uses bus errors for "memory sizing"
   probes only (deliberate), which tolerate imprecision.

Phase 2 picks (2) — simpler, matches Mac OS usage.  Phase 3 can
upgrade to (1) if real-hw measurements show a need.

---

## Phase gating

| Phase | Scope | Agents |
|-------|-------|--------|
| 2.1 | Minimum viable synchronous: illegal, A-line, F-line, privilege, TRAP #n, simple bus/addr error.  Format 0 + 2 stack frames.  Sequencer module.  Arch SR + USP/SSP/ISP split. | exception-path, supervisor-mode |
| 2.2 | Autovector IRQ: VIA1 level 1, VIA2 level 2, SCC level 4.  `irq_agg.v`.  IPL sampling at commit. | irq-aggregator |
| 2.3 | MOVEC for VBR / CACR / ITT / DTT.  RTE.  Tests. | movec-vbr |
| 3   | Trace.  CHK/CHK2/TRAPV/TRAPcc.  Format error.  Divide-by-zero.  Nested exceptions (double fault → reset).  Vectored interrupts (IACK).  Format-B FPU frame. | exception-path-phase3, fpu-bringup |
| 4   | Fast A-line path (bypass full sequencer for known Toolbox hot paths — ~3 cycles instead of ~12). | exception-fast-path |

The existing `exception-path` entry in the historical Phase 2
agent fleet is the phase-2.1 starter; the table above fleshes out
what the follow-ups look like.

---

## Agent fleet (historical entries — orchestrator pattern retired)

| Agent | Files | Phase | Depends on |
|-------|-------|-------|------------|
| exception-path | rtl/core/exception.v (new) + commit.v + rob.v + m68k_core.v | 2.1 | ROB valid-bit observability port (already touched by bpu-phase2-filter path) |
| supervisor-mode | decode.v + commit.v (SR state, USP/SSP/ISP, MOVE USP, privileged-inst check) | 2.1 | exception-path |
| irq-aggregator | rtl/mac/irq_agg.v (new) + rtl/mac/glue.v + m68k_core.v (ipl inputs) | 2.2 | exception-path |
| movec-vbr | decode.v + commit.v (VBR, CACR, ITT0/1, DTT0/1 via MOVEC; RTE) | 2.3 | exception-path |
| exception-path-phase3 | rtl/core/exception.v (extend) + commit.v | 3 | phase-2 exception landed, MMU real, FPU stub |
| exception-fast-path | rtl/core/exception.v (extend) + commit.v | 4 | phase-3 exception stable |

Conflict rules:
- exception-path runs ALONE on commit.v + m68k_core.v (wide
  footprint).
- supervisor-mode is sequenced immediately after — also touches
  commit.v.
- irq-aggregator mostly lives in rtl/mac/; small m68k_core.v port
  additions only.  Can parallelise with anything in rtl/core/.
- movec-vbr touches decode.v + commit.v — serialise after
  supervisor-mode.

---

## Integration with existing machinery

**Reuse branch-mispredict flush.**  Squashing uops on exception is
the same machinery as squashing on branch mispredict: `flush_en`
pulses, ROB wipes entries after the keep-tag, cRAT restores, PRF
free-list rebuilds.  The only delta is WHERE the keep-tag comes
from (commit head, not the resolved-branch's ROB tag).

**Reuse store-commit discipline.**  The existing "store retires
from ROB, then LSU issues AXI write" design is already exception-
friendly — stores that were squashed never issue.  A store ahead of
the fault does issue (already retired).  This matches phase-2's
"bus error on store is imprecise" policy.

**Reuse CCR rename mechanism.**  SR is logically CCR + {T1, T0,
S, M, I[2:0]} — extend the architectural SR register in commit.v
to carry those extra bits; CCR continues to rename as it does now;
the supervisor-mode bits DON'T rename (written only by commit in
response to exception entry/RTE).

**Reuse debug_ctrl trace rings.**  The EXC_TRACE ring in
`debug_ctrl.v` is already specified per `debug_pcie.md` — just
wire it up when exception-path lands.  Same for IRQ_TRACE.

---

## Open questions

1. **Single sequencer FSM vs microcoded lookup?**  Phase 2 uses a
   hand-rolled FSM (10 states).  If phase 3 adds many vectors with
   custom behaviours, microcoded might be cleaner.  Propose hand-
   rolled for now; revisit in phase 3.

2. **Bus error on store: imprecise (phase 2 policy) or delayed
   retire (phase 3)?**  Already called above.  Confirm with
   real-hw Mac OS behaviour measurements once the core boots.

3. **A-line fast path in phase 4: how aggressive?**  Mac OS Toolbox
   is 100 % A-line.  A 3-cycle fast-path (bypass sequencer,
   hardcoded stack-frame write + direct handler jump) could make
   Toolbox calls nearly free.  Cost: exception RTL complexity.
   Measure phase-3 A-line overhead first.

4. **NMI edge detection: in aggregator or in CPU?**  Placing it in
   aggregator simplifies the CPU's ipl input (always level-held).
   Placing it in CPU keeps aggregator stateless.  Slight edge
   (pun) to putting it in the aggregator since it already has
   state for level priority.

5. **Exception trace ring coverage.**  `EXC_TRACE` records {pc,
   vec, sr, cycle}.  Is that enough, or do we also want
   fault_addr for bus/page faults?  Probably yes — bump
   EXC_TRACE entries to 32 bytes (add fault_addr + frame_format).
   Affects `debug_ctrl.v` trace-ring sizing; amend `debug_pcie.md`
   when the exception-path agent lands.
