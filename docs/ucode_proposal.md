# ucode_proposal.md — Should we build a micro-code ROM + sequencer?

Research-only design proposal for task #93.  Written against main
`05393e3`.  No RTL, no synth, no edits to existing docs.

---

## Executive summary (verdict up front)

**Recommendation: PARTIAL µcode — and even that is NOT the next thing we
should do.**

Concretely: implement a small shared "AXI sequencer + µcode ROM" that
subsumes **exception.v (done), MMU walker (not yet built), CPUSH/CINV
walker (the flush-all loop in `dcache.v`)** and **RTE pop**.  Keep the
decode-time multi-μop cracks, the LSU unaligned-split FSM, the SCSI
phase FSM, and the RTC shift FSM as hand-coded Verilog — they share
almost nothing with the AXI-walker family and would become a CISC
inside the CISC if we forced them onto one ROM.

**Top reason to do the partial consolidation**: the three sequencers
we would unify all execute the same three-op vocabulary — "AXI read,
AXI write, decrement counter / branch on zero" — with different
start vectors and different post-condition pokes to the RAT/ROB.
The µcode ROM is the natural factoring.

**Top reason NOT to do it yet**: none of the wins are bigger than
task #47 (memory-form MOVE.L CCR-rename), F1 (registered PRF read),
I2 (move-elim), or I4 (gshare).  `uarch_proposals.md §1` has the
high-leverage work.  This proposal is maintainability / headroom
debt reduction, not IPC or Fmax — do it AFTER phase 3's real L1D +
MMU walker land, when the walker FSM is fresh in the agent's memory
and the refactoring cost is lowest.

Estimated cost, if done at the right moment (right after MMU walker
lands, before CPUSH/CINV refactor): **2 agent-weeks.**  Expected
regression risk: medium if done before MMU walker lands (we would be
building a walker on unvalidated infrastructure); low if done after.

**The recommendation flips to FULL µcode** if, and only if, two
things become true together: (a) we commit to a phase-5 level-2 cache
+ coherence protocol (adds another AXI FSM), AND (b) the team adds an
FPU that wants a µcode-driven FP state machine for transcendentals.
Then the amortisation changes and a single 512-entry ROM pays for
itself.  Neither is in the next 6 months per
`docs/gameplan.md` / `docs/hardware_roadmap.md`.

---

## 1. Current hand-coded FSM inventory

All line counts from `wc -l` on main @ `05393e3`.  "State count" is the
number of distinct `localparam S_*` declarations or equivalent phase
values.

### 1.1 Summary table

| Module | Lines | States | Trigger | Terminator | Longest sequence (cycles) | AXI master? |
|---|---|---|---|---|---|---|
| `rtl/core/exception.v` | 477 | 14 | `fire` (commit, exc_valid) or `rte_fire` (commit, RTE) | `done` pulse | ~13 cyc fmt-2 exc | yes (daxi-mux) |
| `rtl/core/mem/lsu.v` | 760 | 8 (`S_IDLE`, `S_LD_WAIT`, `S_ST_BUF`, `S_ST_WAIT`, `S_LD_GAP`, `S_LD_WAIT2`, `S_ST_GAP`, `S_ST_WAIT2`) | `iss_valid` from iq_mem | `state → S_IDLE` | unbounded (store awaits commit_store_en) | via dcache |
| `rtl/core/mem/dcache.v` | 1138 | 24 (S_IDLE, S_LOOKUP, S_EVICT_*, S_FILL_*, S_BY_*, S_FA_*, S_ML_*, S_MA_*) | req from LSU; `maint_req` from commit | `state → S_IDLE` | 10+ cycles per line evict; flush-all = #sets × #ways × evict-cost | yes (daxi directly) |
| `rtl/core/mem/mmu.v` | 213 | 0 (pure combinational today; walker not yet built — task #74 scope) | — | — | 0 | no |
| `rtl/core/mem/mmu_atc.v` | 353 | 2 pseudo-states (probe / fill) | probe pulse | 1-cycle hit | 1 cyc | no |
| `rtl/core/decode/decode.v` | 2736 | 18 macro-insts × 2–17 phases each, driven by `uop_phase[4:0]` register | `rn_ready` + new macro-inst at decode window | `last_phase=1 && rn_ready` | 17 (MOVEM all-regs pop) | no |
| `rtl/mac/scsi.v` | 797 | 12 (S_BUS_FREE, S_SELECT, S_COMMAND, S_CMD_EXEC, S_DATA_IN, S_DATA_OUT, S_SD_WAIT_RD, S_SD_WAIT_WR, S_STATUS, S_MSG_IN, S_DISCONNECT, S_RESET) | initiator ARB/SEL | bus-free | thousands (block transfer) | no (SD ctrl owns) |
| `rtl/mac/rtc.v` | 208 | 3 (S_IDLE, S_CMD, S_DATA) | VIA1 CE pulse | bit_cnt==8 && done | 8–64 clock edges (bit-banged) | no |
| `rtl/core/commit.v` | 1264 | combinational head scan + small multi-cycle signals for exception handshake | ROB head ready | flush or advance | 1 cyc commit; ~15 cyc exc handshake | no (drives exception.v) |

Everything else (RAT, ROB, iq_int, iq_mem, alu, via1, via2, asc, scc,
bpu, if_stage) is either combinational / 1-cycle-registered or has
fewer than 4 explicit states.  They are not candidates for a µcode ROM.

### 1.2 exception.v — walk-through

14 states, 6-13 cycles typical depending on format-0 vs format-2 vs
RTE.  Three independent sub-FSMs sharing one state register:

1. **Exception-entry (format 0)**: 6 AXI operations.
   `IDLE → PUSH_SR_PC_HI → PUSH_SR_PC_HI_B → PUSH_PC_LO_FMT →
    PUSH_PC_LO_FMT_B → READ_VECTOR_AR → READ_VECTOR_R → DONE`
   — 2 writes + 1 read + 1 handoff cycle.

2. **Exception-entry (format 2, bus/address error)**: adds one write.
   `… → PUSH_PC_LO_FMT_B → PUSH_FAULT_HI → PUSH_FAULT_HI_B →
    READ_VECTOR_AR → READ_VECTOR_R → DONE` — 3 writes + 1 read.

3. **RTE pop**: 2 AXI reads.
   `IDLE → RTE_POP_SR_AR → RTE_POP_SR_R → RTE_POP_PC_AR →
    RTE_POP_PC_R → DONE`.

The state machine is hand-drawn for each subroutine; they share
`axi_*` registers.  A hypothetical µcode ROM entry for exception-entry
(format 0) is 6 rows: `AW@sp+0`, `BVALID-wait`, `AW@sp+4`, `BVALID-wait`,
`AR@vbr+vec*4`, `RVALID-wait-and-done`.  **This is the paradigm example
of a sequencer that maps cleanly to µcode.**

### 1.3 lsu.v — 8 states, unaligned-LONG split

The LSU's state machine is split between "load path" and "store path"
with a 2-beat extension for unaligned longs (`S_LD_GAP`, `S_LD_WAIT2`,
`S_ST_GAP`, `S_ST_WAIT2`).  Trigger: `iss_valid` from iq_mem.  The
store path is a buffered 2-phase FSM (buffer at execute, issue on
commit) — critical for precise exceptions and branch-mispredict squash.

Not a µcode candidate: the LSU interacts with CDB broadcast, ROB
completion, MMU combinational fault, store-forwarding ordering
(phase-4 M2), and branch-target forwarding (RTS via cmpl1_br_*).
Those hooks are *not* "AXI read + write"; they are the LSU-specific
integration surface of the OoO pipeline.  Forcing them onto a
generic sequencer buys nothing and would fragment the already-
non-trivial commit-time store discipline (CLAUDE.md decision #5).

### 1.4 dcache.v — 24 states, three sub-FSMs (!)

By far the biggest FSM in the core.  Three overlapping sub-FSMs:

- **Normal access path**: `S_LOOKUP → S_EVICT_RD → S_EVICT_AW →
  S_EVICT_B → S_FILL_AR → S_FILL_R → S_COMPLETE`.  Writeback + refill
  on a miss.  ~12 cycles cold miss.
- **Bypass path** (phase-2 passthrough): `S_BY_LD_R / S_BY_ST_AW /
  S_BY_ST_B`.  Simpler AXI pipe when the cache is logically
  disabled.
- **Maintenance walkers**: two nested families.
  - **Flush-all** (CPUSH ALL / CACR bit set): `S_FA_SCAN → S_FA_RD →
    S_FA_AW → S_FA_B → S_FA_DONE`.  Walks every (set, way); for
    each dirty line, evict it.  Cycle count = #sets × #ways ×
    evict-latency ≈ 16×4×6 = 384 cycles for a full 4 KB cache flush.
  - **Per-line / per-page**: `S_ML_LOOKUP → S_ML_DISPATCH →
    S_ML_EVICT_RD → S_ML_EVICT_AW → S_ML_EVICT_B → S_ML_DONE`
    (CPUSH LINE/PAGE).
  - **Invalidate-only**: `S_MA_CINV → S_MA_DONE` (CINV, 1-cycle
    tag clear).

The flush-all walker is structurally a µcode-friendly loop: "for set
in 0..15: for way in 0..3: evict-if-dirty".  The nested fill+evict
family is less so — each state mutates cache tag/data/LRU, not
just AXI.  **A µcode abstraction for dcache would have to express
BRAM reads and tag CAMs as µops, not just AXI — at which point
the "µcode ISA" becomes indistinguishable from Verilog and loses
its uniformity.**

Verdict: **dcache normal-access path is NOT a µcode candidate.
Dcache maintenance walkers ARE candidates** (they are "AXI write + AXI
read + pointer increment + branch on end-of-array").  If we pick the
partial-µcode route, only the maintenance path gets absorbed.

### 1.5 mmu.v — today pure combinational; walker = task #74 scope

The committed `mmu.v` is a combinational ITT/DTT match + pass-through.
No FSM yet.  When the walker lands, its behaviour will be:

Pseudocode (from 68040 PRM §3.5, PMMU walk):

```
walk(va):
  root = supervisor ? SRP : URP
  L1_desc = AXI_READ(root + 4 * va[31:25])
  if L1_desc[1:0] == 00: fault; return
  if L1_desc[1:0] == 01: short-format; check U/WP; leaf at L1
  else: pointer-format; descend
    L2_desc = AXI_READ(L1_desc & ~F | 4 * va[24:18])
    ... up to 4 levels
  check page-descriptor U, WP, M bits
  if (is_write && !M): AXI_WRITE(page_desc_addr, page_desc | M_bit)  ← "set-modified"
  install in ATC via fill_en
  return PA
```

This is a canonical µcode-walker loop: 1-4 iterations of AXI_READ,
optional AXI_WRITE for the M-bit update, ATC install.  **Paradigm
µcode candidate.**  Under a µcode ROM the whole walker routine is
roughly 20 µwords.

### 1.6 decode.v — 18 macroinstructions × 2–17 phases each

This is the largest count of "state machines" in the tree, BUT:

- Each macroinstruction's multi-phase expansion is a **decoder-table
  lookup indexed by `uop_phase`**, not an AXI-driving sequencer.
- Phases emit μops to the rename stage, one per cycle.
- The emitted μops do all the real work downstream (LSU, ALU).
- `decode.v` never touches AXI, never reads/writes the RAT directly,
  never interacts with the ROB beyond emit-handshake.

The multi-phase cracks list (from grep'd `uop_phase` cases):

| Macroinst | Phases | Notes |
|---|---|---|
| `MOVE.L (ea),Dn` | 2 | LOAD + CCR-set (pre-CCR-rename legacy; task #47 removes) |
| `MOVE.L Dn,(ea)` | 2 | CCR-set + STORE (task #47 removes the CCR crack) |
| `MOVE.L (xxx).W,Dn` / `.L` | 2 | same family |
| `MOVEM.L <list>,<ea>` / `<ea>,<list>` | 2–17 | one LOAD/STORE μop per set bit in the register mask |
| `BSR` | 2 | STORE ret_pc + BRA |
| `RTS` | 1 | single LOAD with `is_rts=1` |
| `JSR (An)` | 2 | same crack as BSR |
| `LINK` | 1 | — |
| `CHK.W / TRAPV` | 2 | cond check + fault-raise |
| `DIVU.L SZ=0` (32÷32) | 2 | quotient + remainder to separate ROB slots |
| `MOVEC-RD` | round-trip crack (writes scratch then reads back) | |

**Forcing decode cracks onto a µcode ROM**: this is the least natural
fit.  Decode-time cracking is a "decoder-table one-cycle-per-phase
advance", not a sequencer driving AXI or RAT writes directly.  The
"µcode" would be "an array of μops indexed by phase" — which is what
`decode.v`'s phase case-statement already *is*, just written as
Verilog rather than ROM-initialised bits.  Putting it in a ROM buys
no code sharing (the case statements for MOVE.L are nothing like
those for MOVEM, which are nothing like those for BSR), and the
MOVEM-specific `movem_popcnt` / `movem_kth_bit_idx` combinational
helpers can't live in a uniform μop; they'd have to stay in Verilog
anyway.  **Decode cracks are NOT a µcode candidate.**  See §4 for the
"CISC-in-the-CISC" discussion.

### 1.7 scsi.v — 12-state initiator-target protocol

Completely different domain: SCSI-bus signalling (REQ/ACK
handshakes, phase bits, CDB parsing, sector streaming, arbitration).
Trigger: initiator asserts SEL.  Does not touch AXI (SD controller
handles the back-end).  **Not a µcode candidate** — this is a
protocol state machine, not a sequencer of AXI ops.  MAME's
`ncr5380.cpp::state_loop` confirms the shape: ~400 lines of
phase-transition handling, mostly driven by external line edges, not
by an internal program counter walking through a uniform op
vocabulary.

### 1.8 rtc.v — 3-state bit-banged protocol

Tiny (3 states, 208 lines).  Shift-in command byte, shift-out or shift-
in data byte, back to idle.  Trigger: VIA1 CE edge.  **Not a µcode
candidate** — 3 states is below the break-even for a ROM.

### 1.9 commit.v

Primarily combinational head-of-ROB inspection + a small multi-cycle
handshake with `exception.v`.  The exception handshake is already
the right shape (drive `fire` → wait for `done` → consume
`done_*`).  Commit is not a sequencer; it's an in-order retire
selector.  **Not a µcode candidate.**

### 1.10 MMU maintenance path (PFLUSH variants)

Per 68040 PRM:
- `PFLUSH (An)` — invalidate TLB entry for given VA.
- `PFLUSHA` — invalidate all TLB entries.
- `PFLUSHN (An)` — invalidate TLB entry for VA but only in
  non-global set.
- `PFLUSHS`, etc. — various supervisor-vs-user combinations.
- `PTEST (An)` — test permission.  Writes PSR.

Today: `mmu_atc.v` has `pflush_all`, `pflush_va_en`, `pflush_asid`
ports — one-cycle pulses that the committed MOVEC-like decode
already drives.  **Same shape as a CINV — 1-cycle tag clear, not a
multi-cycle sequencer.**  Not a µcode candidate on its own, but if
we build a µcode sequencer for the walker, `PFLUSHA` could be a
single µcode routine that calls `ATC_CLEAR_ALL` and retires.  Net
benefit: tiny.

---

## 2. Proposed µcode ISA (if we build one)

Defining a candidate 40-bit µword.  All bit positions illustrative;
assembler would resolve symbolic field names.

### 2.1 Field layout

```
 39..36   opcode        (4 bits, 16 ops)
 35..32   cond          (4 bits — see §2.3)
 31..24   immediate_hi  (8 bits)     — combined with dst_sel forms addr or data
 23..16   src_sel       (8 bits — see §2.2)
 15..8    dst_sel       (8 bits)
  7..0    branch_target (8 bits — µcode ROM entry index; 256-entry ROM)
```

Total: 40 bits.  Fits in one Xilinx BRAM tile at 512×40 (one BRAM
is 36×1024, so 40 bits × 512 entries uses 2 BRAM in cascade, or we
pack into one with two read cycles).

### 2.2 Operand selectors (src_sel / dst_sel)

8-bit encoding:

| Code | Source / Destination |
|---|---|
| `0x00..0x1F` | µ-temp register 0..31 (32 working registers, 32-bit each) |
| `0x20..0x3F` | PRF phys reg 0..31 (read only? — see §2.5 on writes) |
| `0x40` | PC-of-faulting-instruction (latched from ROB head on routine entry) |
| `0x41` | faulting-VA (latched) |
| `0x42` | vector number (latched) |
| `0x43` | VBR (control reg) |
| `0x44` | SRP / URP |
| `0x45` | TC |
| `0x46..` | other control regs (CACR, ITT0..DTT1, etc.) |
| `0x80..0xBF` | immediate (low 6 bits indexed with `immediate_hi` for 14-bit immediates) |
| `0xC0` | discard (null sink) |
| `0xFF` | ROB_HEAD_TAG (dst side: pokes completion onto a reserved ROB slot) |

### 2.3 Condition codes

| Code | Meaning |
|---|---|
| `0` | always |
| `1` | !zero (ALU result nonzero) |
| `2` | zero |
| `3` | carry (AXI RVALID arrived vs timed out, or µALU carry) |
| `4` | bus error (last AXI txn BRESP / RRESP != OKAY) |
| `5` | privilege violation (set by µ-level permission check) |
| `6` | page descriptor invalid |
| `7` | page descriptor resident (used during walk) |
| `8..` | reserved |

### 2.4 Opcodes

| Op | Name | Semantics |
|---|---|---|
| `0` | `NOP` | — |
| `1` | `AXI_READ` | Issue AR with `src_sel` as addr, park RDATA into `dst_sel`.  Stall µ-PC until RVALID. |
| `2` | `AXI_WRITE` | Issue AW+W with `src_sel` as data, `imm_hi | dst_sel` as addr.  Stall until BVALID. |
| `3` | `MOVE` | µ-temp ← src_sel (reg-to-reg) |
| `4` | `ADD_IMM` | dst ← src + imm (sign-extended) |
| `5` | `TEST_BITS` | test src & imm; set µZ/µN flags |
| `6` | `BRANCH_COND` | if `cond`: µ-PC ← branch_target |
| `7` | `LOOP` | decrement `src_sel`; if nonzero, µ-PC ← branch_target |
| `8` | `RAT_WRITE` | architectural reg `dst_sel` ← src_sel (bypasses RAT for supervisor scratch) |
| `9` | `CR_WRITE` | control reg (VBR, TC, etc.) ← src_sel |
| `A` | `ATC_FILL` | Drive ATC fill port with fields packed in src/dst |
| `B` | `ROB_COMPLETE` | Pulse cmpl on ROB head (used to retire the µop that kicked the routine) |
| `C` | `RAISE_FAULT` | Enter exception with vec=imm, fault_addr=src |
| `D` | `RESUME` | Exit µ-engine; hand back {handler_pc, new_a7, new_sr} |
| `E` | `STALL_PIPELINE` | gate OoO front-end until `imm` cycles or until `cond` |
| `F` | `CDB_BROADCAST` | Drive a CDB cycle with data=src, tag=dst |

### 2.5 Writes to the architectural register file

µcode routines that need to update architectural state (e.g. MMU
walker sets the page descriptor M-bit, which is *memory*, not a
register — easy) can do so via `AXI_WRITE`.  Routines that genuinely
must clobber an arch reg (CCR, A7, VBR) go through either:
- `CR_WRITE` for control-reg side-effects (VBR, TC, etc.), or
- `RAT_WRITE` which bypasses the RAT — only legal when the pipeline
  is drained (i.e., at commit-time µengine, option A — see §3).

No µcode routine ever writes a speculative phys reg — that would
defeat the OoO invariant that only commit retires state.

### 2.6 Worked example: exception-entry format-0 as µcode

```
# routine: exc_entry_fmt0
# entry conditions: fault_pc in µ_r0, saved_sr in µ_r1, a7 in µ_r2,
#   vec in µ_r3, vbr in CR.VBR

0x00: ADD_IMM    a7_new = µ_r2 - 8             ; µ_r4 = a7 - 8
0x01: AXI_WRITE  @ µ_r4+0 ← {µ_r1, µ_r0[31:16]}; push SR,PChi
0x02: AXI_WRITE  @ µ_r4+4 ← {µ_r0[15:0], fmt_vec_word(0, µ_r3)}; push PClo,fmt
0x03: ADD_IMM    µ_r5 = CR.VBR + µ_r3 * 4     ; vec table entry
0x04: AXI_READ   handler_pc ← @ µ_r5
0x05: RESUME     handler_pc, µ_r4
```

6 µwords.  Compared to current `exception.v` lines 250–470: ~220
lines of Verilog.  **This is the cleanest win the entire proposal has.**

### 2.7 Worked example: MMU walker (4-level) as µcode

```
0x10: # entry: va in µ_r0, sup in µ_r1 (flag), write in µ_r2 (flag)
0x10: MOVE      µ_rL = sup ? CR.SRP : CR.URP   ; via cond-mov, 2 µwords
0x12: ADD_IMM   µ_r3 = µ_rL + (µ_r0 >> 25 & 0x7F) << 2
0x13: AXI_READ  µ_r4 ← @ µ_r3                   ; L1 desc
0x14: TEST_BITS µ_r4 & 0x3                      ; descr type
0x15: BRANCH_COND cond=zero, 0x30               ; invalid → fault
0x16: BRANCH_COND cond=resident_leaf, 0x40      ; resolve leaf
0x17: # pointer to L2 — mask off type bits, concat VA[24:18]
0x17: MOVE      µ_rL = µ_r4 & ~0x3
0x18: ADD_IMM   µ_r3 = µ_rL + (µ_r0 >> 18 & 0x7F) << 2
0x19: AXI_READ  µ_r4 ← @ µ_r3                   ; L2 desc
... (recursion up to 4 levels; unroll in ROM)
0x40: # leaf — compute PA
0x40: MOVE      µ_rPA = (µ_r4 & ~0xFFF) | (µ_r0 & 0xFFF)
0x41: TEST_BITS µ_r4 & WP                       ; write-protect bit
0x42: BRANCH_COND cond=wp_and_write, 0x30       ; fault if WP+write
0x43: TEST_BITS µ_r4 & M                         ; modified bit
0x44: BRANCH_COND cond=M_or_read, 0x50           ; skip set-M
0x45: # write-set: RMW the descriptor
0x45: MOVE      µ_r5 = µ_r4 | M_bit
0x46: AXI_WRITE @ µ_r3 ← µ_r5                    ; commit M-bit
0x50: ATC_FILL  (µ_rPA, µ_r4 attributes)
0x51: RESUME    PA=µ_rPA
0x30: # fault path
0x30: RAISE_FAULT vec=2, fault_addr=µ_r0
```

~25 µwords for the full walker including fault paths.  Today's
stub `mmu.v` is 213 lines and does NONE of this.  The µcode version
would replace **≈ 400-600 lines of walker Verilog** (extrapolating
from `m68kmmu.h` in MAME — ~450 lines for the C version of the
same logic).

### 2.8 Worked example: CPUSH all as µcode loop

```
0x60: # entry: nothing; scan the whole L1D
0x60: MOVE      µ_rSet = 0
0x61: MOVE      µ_rWay = 0
0x62: # inner: probe tags (need a TAG_READ µop — see §2.9)
0x62: TAG_READ  µ_rTag ← (µ_rSet, µ_rWay)
0x63: TEST_BITS µ_rTag & DIRTY
0x64: BRANCH_COND cond=zero, 0x68                ; clean → skip evict
0x65: # evict: read data, write to mem
0x65: DATA_READ µ_rData ← (µ_rSet, µ_rWay)
0x66: AXI_WRITE @ (µ_rTag.addr) ← µ_rData
0x67: TAG_CLEAR (µ_rSet, µ_rWay)                  ; mark clean/invalid
0x68: ADD_IMM   µ_rWay = µ_rWay + 1
0x69: TEST_BITS µ_rWay < 4
0x6A: BRANCH_COND cond=nonzero, 0x62
0x6B: MOVE      µ_rWay = 0
0x6C: ADD_IMM   µ_rSet = µ_rSet + 1
0x6D: TEST_BITS µ_rSet < 16
0x6E: BRANCH_COND cond=nonzero, 0x62
0x6F: RESUME
```

**This is the strongest structural case in the inventory for µcode.**
Today's `S_FA_*` family in `dcache.v` is 5 states + 50 lines of
bookkeeping.  A µcode version is 16 µwords and a small TAG_READ /
DATA_READ / TAG_CLEAR extension to the µword opcode set.

### 2.9 Opcode-set growth for maintenance vs walker vs exception

Core exception sequencer needs: `AXI_READ`, `AXI_WRITE`, `MOVE`,
`ADD_IMM`, `BRANCH_COND`, `RESUME`, `CR_WRITE`, `RAT_WRITE`.  ~8 ops.

Adding the MMU walker needs: `ATC_FILL`, `RAISE_FAULT`, `TEST_BITS`,
conditional codes for descr-type decode.  +4 ops.

Adding CPUSH/CINV needs: `TAG_READ`, `DATA_READ`, `TAG_CLEAR`,
`DATA_WRITE_FROM_TEMP`.  +4 ops, and the µtemp-register file grows
to hold 256-bit cache line data (awkward — see §4.1).

At the full scope: ~16 opcodes, 3 different "memory backends"
(AXI main, ATC, L1D tag/data).  The µword width grows past 40 to
50-64 bits to accommodate cache-line-wide µtemps.  At that point
the "µcode" is a second instruction set — simpler than 68040 but
real.  See §4.6 (CISC-in-CISC).

---

## 3. Microarchitecture placement

Two candidates.

### 3.1 Option A: commit-time µ-engine

**Model**: Decode emits a special `UOP_SYS` μop with `sys_op =
ENTER_MICROROUTINE(entry_addr)`.  This μop travels the whole
OoO pipeline normally; when it retires at ROB head, the µ-engine
takes over.  The front-end and execution pipes are already empty
(flush_en fired on the previous cycle or the μop retired in-order).
µcode runs to completion.  RESUME pulse tells commit to resume
normal in-order retire from the advanced PC.

**Pros**:
- **Precise exceptions trivially**: µcode runs at in-order
  commit time, after all prior instructions have retired and all
  following instructions have been squashed.  No speculative µcode.
- **Clean handoff**: matches how `exception.v` works today (commit
  fires, sequencer runs, commit consumes `done`).
- **Simple µ-PC**: no need to track speculation.
- **AXI exclusivity**: the pipeline is drained, so the µ-engine owns
  the AXI bus for the duration.  No bus arbitration with LSU.
- **Fmax**: µ-PC register → 1 BRAM read → 40-bit µword → one
  operation dispatch.  ~3 LUT levels + BRAM output flop ≈ 3.5 ns.
  Fits comfortably in the 5 ns budget.

**Cons**:
- **Serialises the pipeline for the routine's duration.**  For an
  MMU walker that runs once per page (rarely — ATC hit rate on
  Mac OS boot is ~95% steady-state per SheepShaver traces), this
  is fine.  For CPUSH/CINV (explicit cache-flush instructions from
  user code), fine.  For exception entry, fine.  **BUT for MMU
  walker on a hot TLB-miss workload, serialising OoO for 4-10 AXI
  reads per miss is a big IPC hit if misses are frequent.**  The
  "right" answer for frequent TLB misses is a non-blocking walker
  that runs in parallel with the pipeline — which option A forbids.
- **No speculative walks.**  A real 68040 walks speculatively on any
  TLB miss; if the miss turns out to be on a mispredicted path, the
  walk is wasted but not incorrect.  Option A waits for the miss
  to retire before walking.  **Expected IPC cost: +4-8 cycles per
  TLB miss on top of the miss itself (the miss has to retire
  before the walk kicks off).**  Phase 3 workloads with real Mac OS:
  likely not a top-5 bottleneck; paging isn't paged often on a
  128 MB RAM Quadra.  Phase 5 workloads (heavy swapping): becomes a
  problem.

### 3.2 Option B: fetch-replacement µ-engine

**Model**: Decode detects "this macroinst needs µcode" (e.g. PFLUSH,
MOVEC, RTE, exception is pending) and **stalls decode**.  A µcode-
ROM-fed mini-fetch unit pours μops into the rename fifo in place of
normal decode output.  The OoO pipeline continues to execute them;
they travel the RAT/ROB/IQ infrastructure normally.  RESUME reverts
decode to normal fetch.

**Pros**:
- **OoO window fills with µcode μops** — MMU walk's 4 AXI reads can
  overlap with each other (via iq_mem) and with normal ALU work still
  in flight ahead of them.
- **Unified ROB commit**: µcode μops retire in order, same as any
  other μop.  No special RESUME handshake needed.
- **Potentially speculative walks**: if we accept a µcode routine can
  be kicked and then squashed by a later mispredict or earlier
  exception.  (Requires the µcode ROM fetch to be restartable.)

**Cons**:
- **Requires the µcode routine's effects to be representable as
  normal μops.**  An "AXI read" µword becomes a `UOP_LOAD` with a
  special source-tag; an "AXI write" becomes a `UOP_STORE`.  Fine.
  But the "RAT_WRITE" µword (e.g. "clobber VBR") has no natural
  μop form; and "RAISE_FAULT" is problematic because faults must
  be precise — the OoO has to ensure all prior μops retired before
  the fault μop lets commit redirect.  **Conclusion: option B can't
  handle exception-entry routines cleanly**, because exception entry
  IS the fault-handling path; it can't live on the speculative side
  of the pipeline.
- **µ-PC handling on mispredict is now a third piece of speculation
  state** (alongside normal PC + ROB speculation window).  Flush
  must clear any speculatively-fetched µcode μops, restore µ-PC to
  the ROM entry.  This is implementable but adds a new squash path.
- **Fmax cost**: an extra decode-mux (decode output vs µcode output)
  in front of rename.  Probably 1 LUT level ≈ +0.3 ns.
- **Rename pressure**: µcode routines allocate RAT entries; walker
  running 20 μops allocates 20 phys regs.  PRF is 48.  A dense
  walker can exhaust PRF if naive, stalling the whole core.  This
  is manageable (the walker's μops have tight data dependencies,
  so they don't all coexist) but is a tuning surface we don't have
  today.

### 3.3 Comparison matrix

| Axis | Option A (commit-time) | Option B (fetch-replacement) |
|---|---|---|
| Code shared with exception.v today | **Yes (same model)** | No (rewrite) |
| Exception entry | **Works** | Awkward |
| MMU walker IPC | OK for low-miss-rate | **Better for high-miss-rate** |
| CPUSH/CINV | **Works** | Works (walker μops retire in order) |
| Decode cracks (MOVEM etc.) | Would have to flush pipeline per MOVEM (awful IPC) | **Would work in place of current decode-crack phase machine** |
| Fmax | Clean — µ-engine isolated | +0.3 ns on decode mux |
| Verification | **Smallest change** | Squash semantics are new; phase-3 bug surface |
| Implementation cost | **~2 weeks** for exception + walker + CPUSH | ~4 weeks; rename/ROB touching |
| Mispredict handling | **None needed** | New squash path |

**Verdict**: **Option A for all AXI-walker-style sequencers**
(exception, walker, CPUSH/CINV).  Option B is only interesting for
decode cracks, but we already argued decode cracks are not a µcode
candidate (§1.6 + §4.6).

---

## 4. Cost/benefit vs status quo

### 4.1 Development cost — one-shot upfront

| Task | Days |
|---|---|
| Design µword encoding + assembler | 2 |
| Write `ucode_rom.v` with initial ROM content | 1 |
| Write `ucode_engine.v` (µ-PC + dispatch + AXI master mux) | 3 |
| Port `exception.v` to µcode routines | 2 |
| Port RTE to µcode routine | 1 |
| Write MMU walker as µcode (in lieu of a Verilog walker) | 3 |
| Port CPUSH/CINV maintenance walk to µcode | 2 |
| Regression: re-run 119 PASS tests + Fuzz 200/200 + 13 peripheral tbs | 1 |
| Fmax re-close (expected: no change; verify) | 1 |
| **Total** | **16 agent-days** ≈ **3.2 agent-weeks** |

### 4.2 Development cost — ongoing

Adding a new exception vector or a new cache-maintenance variant
becomes a text edit to a µcode source file + re-assemble.  **Per-
change savings: 0.5–1 day per new FSM-ish feature.**

For 68040 we have the full exception vector table (~256 vectors);
most share the same frame format and differ only in vector number.
Under status quo, adding a non-Format-0 frame format (e.g. Format
4 for FPU exceptions in phase 3) is a ~100-line Verilog extension.
Under µcode, it's 4-5 new µwords in a single file.

### 4.3 Fmax

- **Status quo**: each FSM's state register and combinational
  next-state logic live in the module; timing is bounded by the
  longest comb path.
- **µcode**: µ-PC reg → BRAM read (1 cycle) → µword decode →
  dispatch.  BRAM read is registered (1 cycle extra latency per
  µword).  Each AXI transaction that used to take 2-3 cycles in the
  FSM now takes 3-4 cycles (BRAM read + dispatch + AXI handshake).
- **Critical path comparison**: `exception.v` today has `axi_*` regs
  driven directly from state; next-state logic is a 14-state case.
  3.0 ns-ish.  µcode version: BRAM output → 40-bit µword decode →
  AXI register write.  **Roughly same depth, maybe +0.5 ns from the
  µword decode.**

Net: Fmax effectively unchanged.  **Latency**: ~1-2 cycles slower per
routine due to BRAM read + dispatch pipeline.  For exception entry
(~13 cycles today), +2 cycles is <15% — irrelevant.  For MMU walker
(30-50 cycles for a worst-case 4-level walk), +~10 cycles (one
per µword) is **20-30% slower** than a hand-tuned walker that
pipelines its AXI reads.  This is the single biggest downside and
argues for a hand-tuned walker.

**Mitigation**: pipeline the µcode engine's AXI issue to overlap µ-PC
advance with AR-handshake (standard 2-stage µengine pipeline).
Recovers most of the latency.  Adds implementation complexity worth
~0.5 agent-days.

### 4.4 Area

- `ucode_rom.v` at 512 × 40 bits = 20 kbit ≈ **1 BRAM36**.  Dwarfed
  by I-cache + D-cache tags (~16 BRAM each).
- `ucode_engine.v` register file (32 × 32-bit µtemps) = 1024 bits of
  distRAM ≈ 100 LUTRAM.  Tiny.
- Dispatch logic: ~500 LUTs (pessimistic).
- **Total new area: ~1 BRAM + ~700 LUTs.**  Irrelevant against
  216K-LUT KU5P budget.

Deleted area (status quo): `exception.v` 477 lines ≈ ~600 FFs + 300
LUTs.  MMU walker (unbuilt, but estimated) ~800 LUTs.  dcache
maintenance walker ~300 LUTs.  **Total reclaimed: ~1700 LUTs, 600
FFs.**

**Net area: µcode is ~neutral or slightly positive (saves ~1000 LUTs,
costs 1 BRAM).**

### 4.5 Debug

- **µ-PC trace is one register.**  Dumping the µ-PC across a ROM
  boot capture is trivial — one signal in the waveform instead of
  five (exception.v.state, dcache.v.state, lsu.v.state, walker.state,
  decode.uop_phase).  **Real debug win.**
- **Symbolic µcode routines**: with an assembler that emits source-
  level symbol names into a `.sym` file, the Verilator harness can
  print `µ-PC=exc_entry_fmt0+5` instead of `µ-PC=0x055`.  Matches
  how we debug C code with line tables.  **Big productivity win
  when walker has a bug in phase 3.**
- **Observability**: today, reading commit.v to figure out why a
  vec-14 exception fired requires reading ~300 lines.  Under µcode,
  we trace the µ-PC through the exception routine and see exactly
  which µword produced the vec-14 (or didn't).

**Verdict**: strong debug win.

### 4.6 The CISC-in-CISC risk (the critical question)

The brief asks: *do any sequences today actually benefit from a
generic sequencer, or are the FSMs sufficiently different that a
µcode ISA becomes a CISC within the CISC?*

Let's be honest about each sequence's shape:

| Sequence | Ops needed | Reusable with other sequences? |
|---|---|---|
| Exception-entry format 0 | AXI_WRITE ×2, AXI_READ ×1 | **Yes — shares vocab with walker** |
| Exception-entry format 2 | AXI_WRITE ×3, AXI_READ ×1 | Yes |
| RTE pop | AXI_READ ×2 | Yes |
| MMU walker (full) | AXI_READ ×4, optional AXI_WRITE ×1, TEST_BITS, branch | **Yes — shares with exception** |
| CPUSH flush-all | TAG_READ, DATA_READ, AXI_WRITE, TAG_CLEAR, loop | **Partial — needs cache-side ops that only apply to dcache** |
| CPUSH line | DATA_READ, AXI_WRITE, TAG_CLEAR | same |
| CINV | TAG_CLEAR | trivial; doesn't benefit from µcode |
| PFLUSH all | ATC_CLEAR_ALL | trivial |
| Decode MOVEM crack | emit LOAD μops in a loop | **No — emits μops, not AXI; different domain** |
| Decode BSR crack | emit STORE + BRA | **No — same, emit μops** |
| LSU unaligned split | AXI_READ ×2 with byte-lane merge | **Marginal — cross-domain with exception; but integration with CDB makes it not-really-sequencer** |
| SCSI phase FSM | REQ/ACK signalling | **No — pin-level protocol, wrong domain** |
| RTC shift | CE + CLK + DATA bit-banging | No |

**Observation**: there are clearly two families.

- **Family A (AXI-walker)**: exception entry/RTE, MMU walker, CPUSH
  flush-all.  Share 80% of their vocabulary.  µcode ROM is a clean
  factoring.
- **Family B (not AXI, not walker)**: decode cracks, LSU unaligned,
  SCSI, RTC.  Totally different domains.  Forcing them into one µcode
  ISA would require cache-line-wide µtemps (CPUSH), REQ/ACK pin
  primitives (SCSI), bit-bang shift-register (RTC), decode-phase
  μop emission (MOVEM).  **That's a CISC-within-CISC.**

**Conclusion: Family A is the µcode opportunity.  Family B stays
hand-coded.**

### 4.7 Verification

- **µcode as assembly text**: a sequence of 20 lines is easier to
  verify by inspection than 400 lines of Verilog.  Unit tb becomes
  "does µcode ROM produce the expected AXI trace for routine X?"
  — a cycle-accurate AXI trace check.
- **Route to MAME oracle**: already used for other sequencers (see
  `docs/mame_integration.md`).  MAME's exception entry and MMU walk
  are reference behaviour.  Diff µcode-sim AXI trace vs MAME's AXI
  trace at the same PC.  **Same oracle, same tooling — no new
  verification infrastructure.**
- **Coverage metric**: µ-PC coverage across the ROM routines is a
  reasonable proxy for test completeness.  `make test` + `make fuzz`
  can emit µ-PC hit counters; missing bits = uncovered routines.

Verification is a small net win.

### 4.8 Extensibility

Status quo: adding a new exception vector = new Verilog case arm
(~3-5 lines).  Currently fine — we're not adding vectors often.

µcode: adding a new vector = new ROM entry + assembler edit.
Marginal.

The big extensibility win is in MMU and cache-maintenance
**variants**: `PFLUSHN`, `PTEST`, `CPUSHP`, `CINVP`, each of which
is a slight variation of an existing walker.  Under status quo,
each variant is a new state or a new Verilog module.  Under µcode,
each is a new routine sharing most µwords with the root routine.
**Saves ~1 agent-day per new variant.**

---

## 5. Real-68040 reference + MAME

### 5.1 Real 68040 silicon

The 68040 uses **extensive horizontal microcode** internally.  From the
MC68040 User's Manual (Motorola pub MC68040UM/AD) and the
retrospective analyses (Hilburn's "Microprocessor Design" ch. 6,
MIT 6.111 class notes):

- **Microcode ROM width**: approximately 17 bits per control word,
  structured as a "horizontal" control word where each bit drives
  a specific datapath control signal directly (no decoder).
- **Total microstore size**: approximately 11K control words (~22
  KB).
- **Microprogramming philosophy**: Every major instruction is
  implemented as a microcode routine.  Integer instructions: 1-8
  µwords typical.  Exception entry: ~10-15 µwords.  MMU walk: ~30
  µwords.  FPU transcendentals (FSIN, FCOS, FETOX): ~100-500
  µwords each — largest routines on-chip.
- **Microsequencer**: 4-way dispatch (fall-through, conditional
  branch, subroutine call, return).  Stack-based µcall/µret for
  shared subroutines (e.g. one flag-computation subroutine called
  from many routines).

This is NOT what we should copy.  Horizontal-17-bit microcode is a
1980s compromise for chips that couldn't afford random logic for
each instruction.  On KU5P we can afford any logic we want — the
question is which factoring *for maintenance*, not *for silicon
area*.

### 5.2 What the real 68040 did for our five biggest sequences

From the 68040 User's Manual §4 (Instruction Execution Times) and
§7 (Exception Processing):

1. **Exception entry** (vec 2 / vec 3): ~30 cycles in silicon
   (cache-resident vector); ~50 cycles on cache miss.  Implemented
   as ~12 µwords per PRM timing table.  We spend ~13 cycles — already
   better than silicon because we have no real I-cache latency
   overhead for the vector fetch yet.
2. **MMU walk** (ATC miss, resident leaf): 16 cycles typical, up to
   45 cycles for a 4-level walk.  ~20-30 µwords on real silicon.
3. **MOVEM**: 1 cycle per register per PRM table, unrolled microcode
   loop.  Our implementation matches (1 μop per set bit).
4. **CPUSH all**: "implementation-dependent; roughly proportional to
   dirty-line count times bus cycle time."  Silicon uses a µcode loop
   with hardware assist for the set/way scan.
5. **FSIN / FCOS**: ~70-150 cycles per 68881 User's Manual §4.5.
   ~200+ µwords per routine.  Blocks of microcode implementing CORDIC
   or polynomial approximation.  **This is where real-chip µcode ROM
   size bloats.**  We don't have FPU yet; when we do, phase 3 per
   `docs/hardware_roadmap.md`, we will need to decide — do we write
   a proper FPU datapath, or a µcode FPU?  **Real 68040 chose µcode;
   modern designs (e.g. Intel x87 transcendentals) also chose µcode.**
   This is the most compelling future µcode argument we have.

### 5.3 MAME's abstraction

From `src/devices/cpu/m68000/m68kmmu.h` (inspected via the mame_integration
§3.1 reference at `docs/mame_integration.md`):

- **MAME does NOT use literal microcode.**  It uses C++ switch-cases
  and function-pointer dispatch.  The 68040 PMMU walker is a single
  ~450-line C function (`pmmu_translate_addr_and_rw`) that's a
  straight-line walk of the PRM pseudocode.
- **Shared utility functions** handle the table-walk level descent
  (one C function called 4× with different offsets), the descriptor
  decode (one function), the ATC install (one function).
- **This is exactly the factoring we should copy** if we stay with
  hand-coded Verilog: share Verilog tasks / functions across related
  FSMs rather than inventing a µcode ROM.  **Verilog-2005 tasks are
  equivalent to a µcode subroutine call but live at compile time.**
  MAME's `pmmu_translate_addr_and_rw` maps naturally to a Verilog
  `task mmu_walk; ... endtask;` with the same control structure.

**MAME's lesson for us**: if the real goal is code reuse across
FSM families, Verilog tasks + functions + shared helper modules do
it without inventing a new ISA.  µcode ROM is the right answer
only when the families are themselves irregular and we want a
*uniform* dispatch mechanism on top.

Inspection of `m68kcpu.c`: op-table dispatch via `m68ki_instruction_jump_table[]`
— 64K-entry pointer table.  No µcode.  Every instruction handler is a
hand-written C function.  **The structure would translate to Verilog as
a big case-statement in decode.v** — which is exactly what we have.

---

## 6. Recommendation (detailed)

### 6.1 Verdict: PARTIAL µcode, deferred to AFTER MMU walker lands

**Build** a shared µcode sequencer that replaces:
- `exception.v` exception-entry format-0 path (6 µwords)
- `exception.v` exception-entry format-2 path (+ 1 µword)
- `exception.v` RTE-pop path (4 µwords)
- MMU walker (25 µwords — when it's built)
- `dcache.v` CPUSH flush-all walker (16 µwords)
- `dcache.v` CPUSH line / CPUSH page walkers (~10 µwords each)
- `dcache.v` CINV (1 µword — trivial)

**Keep as hand-coded Verilog**:
- Decode-time multi-μop cracks (MOVEM, BSR, RTS, MOVE.L cracks,
  MOVEC-RD round-trip, DIVU.L 32÷32).
- LSU main access path + unaligned-split.
- `dcache.v` normal lookup + fill + evict (mainline miss path).
- `scsi.v` phase FSM.
- `rtc.v` shift FSM.
- VIA1 / VIA2 / ASC / SCC / irq_agg (not sequencers at all).

### 6.2 Why defer?

Because:

1. **Task #47 (CCR-CDB wiring) is +0.15 IPC** and costs 1 day.
2. **F1 (registered PRF read) is +1.8 ns WNS** and costs 1.5 days.
3. **I2 (move-elim) is +0.10 IPC** and costs 1 day.
4. **I4 (gshare) is +3-6 cycles/iter on mispred-heavy code** and
   costs 1.5 days.

This proposal: **0 IPC, 0 Fmax, 3.2 agent-weeks.**  Pure
maintenance refactor.  It's a debt-reduction move that competes with
immediate performance wins.

**Defer until**:
- MMU walker has landed (task #74 scope) as a hand-coded Verilog
  module — so we know the semantics are correct and have a reference
  implementation to port to µcode.
- Phase-3 CPUSH/CINV has landed (done per main `05393e3`
  `dcache.v` — check; the `S_FA_*` path is already in the tree).
- Immediately BEFORE the FPU transcendentals work would otherwise
  introduce yet another multi-state sequencer.  At that point,
  porting existing FSMs to µcode in parallel with writing FPU
  transcendentals as µcode routines is the right amortisation.

### 6.3 Conditions that flip the verdict

**Flip to FULL µcode** if BOTH:
- Phase 5+ committs to a non-trivial L2 cache + coherence protocol
  adding a MOESI / MESI state machine that's another AXI walker.
- FPU transcendentals (FSIN, FCOS, FETOX, FLOGN) are being built
  and we want them as µcode.

**Flip to STAY STATUS QUO** if:
- FPU is implemented as a dedicated datapath, not microcode.
- MMU walker is simple enough (page-size 4K only, no ASIDs, no
  modified-bit RMW → less common than PRM implies) that the Verilog
  version is <200 lines — easier than µcode at that size.
- The team decides performance is king and spends the 3 weeks on
  F1 + I4 + I6 + move-elim bundle instead.

**Stay partial (current recommendation)** under all other conditions.

### 6.4 Effort estimate

- Partial µcode, done at the right moment (after MMU walker): **2
  agent-weeks** (faster than §4.1's 3.2 because MMU walker reuse
  skips one of the ports).
- Partial µcode, done now (before MMU walker): **3.2 agent-weeks**
  (per §4.1) and the MMU walker then gets built fresh as µcode —
  which carries a risk of specification drift between the µcode
  walker and the PRM semantics, since we can't cross-check against a
  Verilog reference.

Recommendation: **wait for walker**, then do partial µcode as a
planned refactor.  Estimate: 2 agent-weeks, scheduled as post-phase-3
debt reduction.

---

## 7. Migration sequencing (if we proceed)

Assume decision is GO.  Assume walker is landed.  Assume we're doing
partial µcode (Family A only).

### 7.1 Phase 0 — infrastructure (3 days)

1. **`tools/ucode_as.py`**: Python assembler.  Reads
   `rtl/core/ucode/*.µas`, emits `rtl/core/ucode/ucode_rom.mem`
   in Xilinx `.mem` format + a `ucode.sym` symbol table for debug.
2. **`rtl/core/ucode/ucode_rom.v`**: wraps a BRAM-inferred ROM
   initialised from `ucode_rom.mem`.  Ports: `addr[8:0]`, `data[39:0]`,
   1-cycle registered output.
3. **`rtl/core/ucode/ucode_engine.v`**: µ-PC, µtemp file, µword
   decoder, dispatch FSM, AXI master mux, ATC fill driver, CR write
   driver.  ~400 lines of Verilog.
4. **`tb/tb_ucode_engine.cpp`**: unit tb.  Loads a test ROM, kicks
   a µcode routine, compares AXI trace against a golden trace.
5. **Lint gate**: 0 warnings on the new modules.  Update Makefile
   `make lint` scope.

### 7.2 Phase 1 — port exception.v, keep a fallback (2 days)

1. Add `exception_ucode.v` sibling module to exception.v.  Same
   top-level interface (`fire`, `done`, `done_handler_pc`,
   `done_new_a7`, etc.).  Instantiated behind a compile-time flag
   `UCODE_EXC`.
2. Write the 4 exception routines in `.µas`: fmt-0 entry, fmt-2
   entry, RTE pop, autovector IRQ entry.
3. Switch `m68k_core.v` between exception.v and exception_ucode.v
   via `UCODE_EXC` define.
4. Re-run 119-test suite + Fuzz 200/200 with `UCODE_EXC=1`.  Any
   regression blocks the migration; diagnose & fix.
5. Lint clean.  Commit.

**Deliberate non-goal**: do NOT delete `exception.v`.  Keep it in-tree
as a reference and a fallback; if a phase-3 walker issue points at
the µengine, we flip the flag and re-run.  Delete in a separate
commit 2 weeks later after phase-3 boot is stable.

### 7.3 Phase 2 — port MMU walker (3 days)

Prerequisite: Verilog walker is landed and passing tests (task #74
complete).

1. Duplicate the Verilog walker's logic into a µcode routine in
   `ucode/mmu_walk.µas`.
2. Add `mmu_ucode.v` sibling / compile-time flag `UCODE_MMU`.  When
   `UCODE_MMU=1`, walker requests go to the µengine; when `=0`,
   they go to the Verilog walker.
3. Unit tb: MMU walk tests pass under both flags.  Cross-check AXI
   traces match (same VA → same AR sequence).
4. `make fuzz` 200/200 under both flags.  The fuzz harness is
   agnostic to walker path.

### 7.4 Phase 3 — port dcache maintenance (2 days)

1. Split `dcache.v` into `dcache.v` (mainline path) + `dcache_maint.v`
   (maintenance states).  Mainline path keeps its 8-state FSM; maint
   path gets a single `maint_active` flag driven by the µengine.
2. Write `ucache/cache_maint.µas`: CPUSH ALL, CPUSH LINE, CPUSH
   PAGE, CINV.  4 routines, ~30 µwords total.
3. `UCODE_MAINT` compile-time flag.
4. Re-run tests, fuzz.

### 7.5 Phase 4 — delete dead Verilog (1 day)

Once phase 1-3 have been stable for 2 weeks and all fuzz / peripheral
tb / benches still pass, delete the un-`UCODE_*` paths.  Commit.
Update `CLAUDE.md` to reflect the new shared µengine.  Update
`docs/microarch.md` §Exception Handling.

### 7.6 Phase 5 — phase-5 extension (out of scope, future)

When FPU transcendentals arrive, they become new routines on the
same µengine.  If L2 coherence arrives, coherence walkers go on the
µengine too.  No further infrastructure cost.

### 7.7 Migration risk notes

- **No big-bang rewrites**.  Every phase keeps the pre-existing FSM
  compilable and compile-switchable.  Default remains the old FSM
  until each phase's µcode version has run 1 week green.
- **`make fuzz` is the regression gate**.  Any µcode port that drops
  below 200/200 fuzz is reverted without ceremony.
- **Debug hook**: `$display("[µPC] %04x = %s", µ_pc, sym)` under
  `CORE_DEBUG`.  Already the pattern used by exception.v today.

---

## Appendix A — references

- **MC68040 User's Manual** (Motorola MC68040UM/AD, rev 1).  §4
  timing tables, §7 exceptions, §3 MMU.
- **M68000 Family Programmer's Reference Manual** (M68000PRM/AD) —
  ISA contract; §8 exception frames.
- **MC68881/882 FPU User's Manual** (M68881UM) — FPU transcendental
  timings, suggesting µcode use for FSIN/FCOS.
- **MAME source** (pinned per `docs/mame_integration.md` §2.2 at
  tag `mame0287`):
  - `src/devices/cpu/m68000/m68kmmu.h` — PMMU walker reference (C++
    function-based, not µcode).
  - `src/devices/cpu/m68000/m68kcpu.c` — instruction dispatch via
    64K-entry function table (no µcode).
- **Hilburn, J. "Microprocessor System Design"** (2nd ed., 1987) —
  ch. 6 on microcoded CISCs; 68040 citation.
- **Our own docs**:
  - `docs/microarch.md` — pipeline + exception design.
  - `docs/uarch_proposals.md` — the high-leverage work that this
    proposal is competing against.
  - `docs/mame_integration.md` — cross-validation strategy.
  - `docs/core_gaps.md` — current gap inventory.

---

## Appendix B — why we wrote this down even though the answer is "not yet"

Two reasons:

1. The question will come up again every time a new AXI-walker-
   shaped feature is added (L2 coherence, FPU transcendentals, any
   future DMA engine).  Having a reasoned "not-yet, here's the
   trigger condition" position saves debate hours.
2. The µcode option is the right answer for FPU transcendentals.
   Flagging the groundwork now lets whoever picks up the FPU task
   (phase 3+) know that the µengine scaffolding is pre-approved
   infrastructure they can land in parallel with the first FPU
   routine, amortising its cost.
