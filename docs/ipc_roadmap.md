# ipc_roadmap.md — Path to IPC ≥ 1.0 (and beyond)

> **Deliverable for task #106.**  Authoritative design document for the m68k-ooo
> OoO core.  Covers everything required to lift peak IPC from today's 0.36
> to ≥ 1.0 on the benchmark suite, and then beyond.  No RTL changes in this
> commit — this is the plan.
>
> Baseline measurement: **post-L1D-real sim, `main @ d94be88`.**  Numbers below
> were generated with `/tmp/m68k-ooo-build/build/sim/Vmac_top +test=<bench>
> +timeout=500000` on 2026-04-17.  They supersede the table in
> `docs/bench_baseline.md` for post-cache bench runs.

> **2026-04-18 status overlay:** this remains the design rationale, not the
> live ticket board.  Since the baseline, Phase A rename-elim / zero-idiom
> and branch-fusion work landed, and Phase C C1 (F1 registered PRF read +
> CDB bypass) landed as `fe9f27d`.  Use `docs/bench_baseline.md`'s
> "Phase-C C1 delta" block for the current F1 performance check.  Treat the
> tables below as historical unless a row is explicitly marked pending; the
> next low-risk Fmax planning should focus on control-fanout cleanup and
> path splits, not widening decode/rename/ROB before first-light.

---

## 1. Executive summary

Headline numbers (current and projected):

| Metric                                  | Value     | Reasoning (≤ 8 words)                          |
|-----------------------------------------|-----------|-------------------------------------------------|
| **Current peak IPC (bench_ind_adds)**   | **0.357** | Post-L1D, single-issue, CCR-rename active        |
| **After Phase A (decode-local fusion + rename elim)** | **0.55 – 0.65** | CMP/Bcc fuse, move-elim, zero-idiom    |
| **After Phase B (dual-port L1D + dual-LSU)** | **0.75 – 0.95** | Load parallel w/ store, SB forwarding |
| **After Phase C (2-wide decode/dispatch + 2nd ALU)** | **1.25 – 1.55** | True dual-issue, 4 CDB   |
| **After Phase D (ROB 64 + PRF 96 + gshare + IBT)** | **1.55 – 1.85** | Window + prediction quality     |
| **Theoretical ceiling (2-wide ins, ideal hits)** | **1.90 – 2.00** | Decode width + branch/dep limits |

Peak today: `bench_ind_adds = 305 / 854 = 0.357`.  Floor: `bench_btb_dbra =
149 / 923 = 0.161`.  Every number below has a "committed / cycles" derivation.

### Recommendation (10 lines)

**Already built from the original Phase A/C1 sequence:**
1. CMP/CMPI/TST/ADDQ/SUBQ + Bcc fusion at decode (task #113 /
   `f1f5653` lineage).
2. Rename-time move-elim and zero-idiom recognition with CCR-producer μop
   preserved (task #114).
3. F1 registered PRF read + CDB bypass (Phase C C1, `fe9f27d`), with
   ≤1% bench drift recorded in `docs/bench_baseline.md`.

**Build next, after first-light pressure eases:**
1. Pure control-fanout cleanup where sim can prove no cycle drift:
   `if_stage` predecode-valid / line-crossing controls and commit/ROB
   flush distribution.
2. Remaining Phase C width work only after F1-dependent timing debt is
   understood: CCR-RAT 2-port, RAT 2-port, ROB retire, IQ 2-pick, second
   ALU, then 2-wide front end.

**Defer (Phase D+, large re-architecture):**
* Second ALU lane *alone* without the landed F1 dependency in the build and
  bench gate — still blocks at 200 MHz.
* 2-wide decode *without* first landing ROB 64 / PRF 96 — structures saturate
  at 8 IQ entries after ~12 cycles.
* Trace cache, value prediction — not cost-effective for Mac OS-class code.

---

## 2. Measured IPC-loss budget

The methodology below uses: **static instruction mix** from the benchmark's
`.s` file (`grep` patterns for `add|sub|cmp|move|lea|bne|bra|dbra|subq|addq|
tst|and|or`), **dynamic execution length** from the testbench trailer
(`committed` and `cycles`), and **dispatch traces** from a DEBUG=1 run (to
measure inter-μop dispatch spacing).  For µops we use the decode.v crack
count: register-form ALU = 1 μop; `move.l (%a0),%d1` = 1 LOAD μop;
`move.l %d0,(%a1)` = 1 STORE μop; `bsr` = 2 μops; `rts` = 1 μop; `dbra` = 1
μop; all others 1 μop.

### 2.1 Baseline numbers (post-L1D-real, `main @ d94be88`)

| Bench              | Cycles | Committed | IPC    | Static insns |
|--------------------|--------|-----------|--------|--------------|
| bench_alu_parallel | 859    | 294       | 0.342  | 17           |
| bench_btb_dbra     | 923    | 149       | 0.161  | 6            |
| bench_btb_loop     | 1024   | 249       | 0.243  | 7            |
| bench_cmp_branch   | 720    | 171       | 0.238  | 14           |
| bench_dep_chain    | 273    | 70        | 0.256  | 26           |
| bench_fullpipe     | 1106   | 366       | 0.331  | 20           |
| bench_ind_adds     | 854    | 305       | 0.357  | 16           |
| bench_mixed_mem    | 690    | 192       | 0.278  | 23           |
| bench_move_heavy   | 2089   | 700       | 0.335  | 19           |

### 2.2 Cycle attribution per bench

Units below: cycles lost to each bottleneck class.  "Cold" = cycles before
the loop enters steady state (first iteration's cache miss, BPU warmup,
front-end bubble).  Estimates are rounded to the nearest 5 cycles.

#### bench_ind_adds (854 cyc / 305 μops)
Loop body: 3 parallel `add.l` + `sub.l #1,%d6` + `bne` = 5 μops.  50 iters
× 5 μops = 250 μops dispatched in-loop; prologue 7 μops; epilogue 48 μops
(the `lea 0xFFFF0000,%a0` + sentinel MOVE + STOP).  → ~305 committed.

Steady-state cycle count per iter (854 − 110 cold − 90 epilogue) / 49 iters
≈ **13.2 cyc/iter** for 5 μops = **0.38 IPC in-loop**.

Where the 13 cyc go (per iter):
- Dispatch (single-issue): 5 cyc minimum — 5 μops × 1-wide decode.
- ALU issue latency (1 cycle to RS, 1 cycle execute, 1 cycle CDB wake) on
  dependent μops: the `sub.l #1,%d6` → `bne` edge is CCR-rename-wakeable
  at 1 cyc, but the branch then has to wait for its PC resolution and the
  predict-redirect bubble absorbs 1 cyc.  Net: +3 cyc per iter wasted on
  branch-redirect-plus-CCR-chain.
- IQ-int pick + issue latency: 1 cyc from dispatch to issue (registered
  `disp_ready` = +1 cyc at full IQ).  Rarely applies here; ~1 cyc / iter
  when IQ hits 8.
- Commit bandwidth: 1/cyc commit; 5 μops/iter → 5 commit cycles dominate
  nothing here (pipelined).
- Remainder (~4 cyc): front-end bubbles + occasional ROB tail backpressure.

**Loss decomposition (per iter):**
| Class                          | Cyc/iter | Total (49 iter) |
|--------------------------------|----------|-----------------|
| Dispatch width = 1             |  4       | ~200            |
| Branch redirect bubble         |  1       |  49             |
| CCR-wake serialisation         |  1       |  49             |
| IQ pressure / commit           |  1       |  49             |
| **Serial minimum (ALU lat)**   |  6       | ~300 (floor)    |

After 2-wide decode + 2nd ALU: floor becomes ~7 cyc/iter → 49×7 + 150
cold/epilogue = **~500 cyc, IPC 0.61.**  After full Phase D (gshare-taken
+ BTB-target hit eliminates branch bubble, CCR-CDB bypass saves 1 cyc):
**~4 cyc/iter, ~350 cyc, IPC 0.87**.

#### bench_alu_parallel (859 cyc / 294 μops)
Loop body: 4 `add.l` + `sub.l` + `bne` = 6 μops.  40 iters × 6 = 240
in-loop.  Steady-state ~13 cyc/iter, same bottleneck as ind_adds — dispatch
width.  **Limit = 1-wide decode.**

| Class                  | Cyc/iter | Total |
|------------------------|----------|-------|
| Dispatch width = 1     |  5       | ~200  |
| Branch redirect bubble |  1       |  40   |
| CCR-serial  (BNE wait) |  1       |  40   |
| Remainder              |  6       | ~240  |

Projected after 2-wide decode + 2nd ALU: 6 μops / 2 width ≈ 4 cyc/iter →
~260 cyc, IPC ≈ 1.1.

#### bench_dep_chain (273 cyc / 70 μops)
No loop — 20 serial `add.l %d1,%d0`.  IPC 0.256 = 1 μop per 3.9 cyc.  The
chain is strictly RAW-serialised: every ADD reads the prior ADD's D0.
Per-ADD cost = **RS → ALU → CDB wake = 3 cycles** (today, without value
forwarding).  Plus the initial cold-fetch + 4-cyc ALU-DSP-cascade + epilogue
(sentinel MOVE + STOP).

| Class                        | Cyc/iter |
|------------------------------|----------|
| RS→ALU→CDB→wake (ALU lat)    |  3       |
| **→ 20 ADDs × 3 =**          | **60**   |
| Cold / prologue              |  ~60     |
| Epilogue (sentinel + STOP)   | ~150     |

This bench is **data-serialised, not dispatch-serialised** — 2-wide decode
buys 0 cycles on the chain itself.  **The only fix is an ALU→ALU bypass (§6)**:
forward `alu0_result → alu0_src` combinationally at the RS read mux.  If we
can close 1-cycle ALU (no forwarding round-trip), chain drops from 3
cyc/ADD to 1 cyc/ADD = 20 cyc → **bench_dep_chain becomes ~210 cyc,
IPC 0.33**.  If ALU stays 1-cycle with *forwarding* (0-cycle wake), we get
the same 20 cyc and IPC 0.33 — the epilogue dominates.

#### bench_cmp_branch (720 cyc / 171 μops)
Loop body: `cmp.l %d1,%d0` + `bne _branch_taken` + `sub.l #1,%d6` + `bne _loop`
= 4 μops per iter (the "not reached" path is mispredicted + squashed exactly
once per iter by the second BNE).  30 iters × 4 = 120 in-loop.

From the DEBUG trace (cycles 43–60 span two iterations): per iter ≈ **8
cycles / 4 μops = 0.5 IPC in-loop.**

| Class                            | Cyc/iter | Total |
|----------------------------------|----------|-------|
| Dispatch 4 μops @ 1-wide         |  4       | 120   |
| CCR-CDB wake between CMP and BNE |  1       |  30   |
| Branch redirect bubble × 2 BNEs  |  2       |  60   |
| Mispredict penalty (rare cold)   |  ~0      |  ~10  |

After CMP/Bcc fusion: **4 μops → 3 μops per iter**.  Fused CMP+BNE is a
single UOP_BRANCH with both srcs read + imm target.  In-loop: 6 cyc/iter
est, 30×6 = 180, +250 prologue/epilogue ≈ **430 cyc, IPC 0.40**.  After
2-wide dispatch: 3 cyc/iter → **340 cyc, IPC 0.50**.

#### bench_fullpipe (1106 cyc / 366 μops)
Loop body:
```
add.l %d1,%d0       ; ALU
add.l %d3,%d2       ; ALU
lea 0x100200,%a0    ; ALU (loads imm into a0 via phys_zero+imm)
move.l (%a0),%d4    ; LOAD
sub.l #1,%d4        ; ALU (dep on LOAD)
cmp.l #0,%d4        ; ALU (dep on SUB)
move.l %d4,(%a0)    ; STORE (dep on SUB)
bne _loop           ; BRANCH (dep on CMP CCR)
```
= 8 μops per iter.  30 iters × 8 = 240 + prologue 45 + epilogue ~80 ≈ 366
μops.  Per iter: (1106 − 120 prologue − 30 epilogue) / 30 ≈ **32 cyc/iter
for 8 μops = 0.25 IPC in-loop**.

Why so slow?  The load-CMP-store chain is long (load: 3 cyc to CDB, SUB: 1
cyc, CMP: 1 cyc, STORE: issue-to-ST_BUF 1 cyc).  Plus the branch needs the
CMP's CCR wake.  Per iter cost is dominated by the **single-port LSU**:
LOAD issue → wait cache 2 cyc → SUB wake → CMP wake → STORE issue → LSU
BUSY until next iter's LOAD.  STORE's `lsu_ready` gate holds iq_mem back:
next iter's LOAD can't issue until current STORE's ST_BUF → ST_WAIT → IDLE
closes.

| Class                                   | Cyc/iter | Total |
|-----------------------------------------|----------|-------|
| Dispatch 8 @ 1-wide (steady state pack) |  5       | 150   |
| LSU single-port serial LOAD→STORE       | 12       | 360   |
| CCR wake (CMP→BNE)                      |  1       |  30   |
| Branch redirect bubble                  |  1       |  30   |
| ALU RAW chain (LOAD→SUB→CMP)            |  3       |  90   |

Dual-LSU (LOAD + STORE parallel) saves ~8 cyc/iter → **350 cyc recovered,
IPC ~0.47**.  Add 2-wide decode → another ~200 cyc → **700 cyc, IPC 0.52**.

#### bench_mixed_mem (690 cyc / 192 μops)
Loop body: `move.l (%a0),%d1` + `add.l %d1,%d0` + `add.l #4,%a0` + `sub.l
#1,%d7` + `bne _loop` = 5 μops per iter.  20 iters × 5 = 100 in-loop;
prologue 12 μops (setup); epilogue (sentinel + STOP + cache flush) ≈ 80.

Per iter (loop-only): (690 − 180 prologue − 100 cache-flush − 60 epilogue)
/ 20 ≈ **17.5 cyc/iter / 5 μops = 0.29 IPC**.  But L1D hits from iter 2 are
~3 cyc each, so steady state after iter 2 is ~12 cyc/iter.  Cache-flush at
end adds ~175 cycles (flush_all_req walker).

| Class                             | Cyc/iter |
|-----------------------------------|----------|
| LOAD to L1D hit + CDB wake        |  3       |
| ALU RAW (LOAD→ADD accumulate)     |  2       |
| ALU parallel (post-inc A0, SUB d7)|  (folded)|
| Branch redirect bubble            |  1       |
| Commit serialise                  |  1       |

Dual-LSU doesn't buy much on this bench (only 1 LOAD per iter; no parallel
STORE).  **2-wide decode is the bigger win here**: folds the ALU-parallel
pair (post-inc A0 + decrement D7) into 1 cycle, saving 2 cyc/iter → **650
cyc, IPC 0.30**.  Larger caches won't help (working set fits).

#### bench_move_heavy (2089 cyc / 700 μops)
Loop body: 6 MOVE + 5 MOVE + SUB + BNE = 13 μops × 50 iters = 650 + 50
cold.  Per iter: (2089 − 100 cold) / 50 ≈ **40 cyc/iter / 13 μops = 0.33 IPC
in-loop**.

Interesting: all MOVE.L Dn,Dm are UOP_INT / ALU_MOV today.  Each is:
- 1 RAT alloc (new phys)
- 1 iq_int dispatch
- 1 ALU cycle (MOV through ALU)
- 1 CDB wake
- 1 commit
= serial RAW chain because D1 reads D0's producer, D2 reads D1's, etc.

11 MOVEs per iter serialised ≈ 11 × 3-cyc chain = 33 cyc/iter; plus SUB+BNE.
→ 40 cyc/iter ≈ matches.

**Move-elim at rename** eliminates all 11 MOVEs to 0 ALU cycles (just RAT
remap), but keeps the CCR producer μops (since 68k MOVE sets N/Z).  Cost
per iter after elim: 0 ALU cycles for MOVEs + 11 CCR-writing μops (if
dispatched as independent flag-writes) + SUB + BNE = **~5 cyc/iter steady
state with 2-wide dispatch**.

If we choose to NOT write CCR on eliminated MOVEs (Mac OS software contract
requires N/Z/V=0/C=0 update, so we **must** write CCR): the CCR-write μop is
itself a UOP_INT `ALU_TST` with src=source phys_reg and only flags_wr set.
These are independent and cheaply dispatchable.  With 2-wide:

| Class                            | Cyc/iter after elim + 2-wide |
|----------------------------------|-------------------------------|
| RAT rename (2-wide)              |  ~6                           |
| CCR-write μops (2-wide dispatch) |  ~6                           |
| SUB + BNE                        |   2                           |

Per iter ~14 cyc, 50 iter = 700 cyc → **bench_move_heavy ~1000 cyc, IPC
0.70**.  After further Phase C expansion (ROB 64 + 2-wide commit): **~700
cyc, IPC 1.0**.

#### bench_btb_dbra (923 cyc / 149 μops) — floor
Body: `dbra %d0,_loop` × 100 iters + sentinel+STOP.  Per iter: (923 − 100
epilogue) / 99 iter ≈ **8.3 cyc/iter for 1 μop = 0.12 IPC**.

Why so bad?  DBRA is a single UOP_BRANCH that:
1. Reads %d0 (counter) — 1 cyc
2. ALU executes (decrement + check Z on word) — 1 cyc
3. CDB wake, ROB complete — 1 cyc
4. Commit resolves (if predicted taken by BTB) — 1 cyc
5. BTB predict-redirect bubble at decode — 1 cyc
6. Fetch re-start — 1 cyc
7. Re-enter IQ — 1 cyc

= 7-8 cyc/iter.  The BTB IS hitting (otherwise it'd be ~14 cyc/iter for
commit-redirect), but each iter still carries **1 cyc decode-redirect + 1
cyc fetch + 3 cyc ALU resolve + 1 cyc commit + 2 cyc dispatch re-queue**.

Fixes:
- **Loop buffer / μop cache (Phase D)**: if the decoded DBRA μop is
  cached, decode is bypassed entirely.  Per iter drops to ~3 cyc →
  **bench_btb_dbra 300 cyc, IPC 0.50**.
- **Decode-time compute of DBRA's "counter != 0" branch direction via
  bypass from last iter's CDB**: same win.

#### bench_btb_loop (1024 cyc / 249 μops)
Body: `subq.l #1,%d0` + `bne _loop` × 100 iters.  Per iter: ~8.5 cyc for
2 μops = **0.24 IPC** — similar to btb_dbra, with the added cost of SUBQ
being a CCR-writing ALU op the BNE must wait on.  CCR-CDB forward already
saves this in the renamed path (1-cyc wake).  Not a CCR bottleneck.

### 2.3 Aggregate attribution (all 9 benches, % of cycles)

| Bottleneck class         | % of total cycles |
|--------------------------|-------------------|
| Dispatch width = 1       | ~28%              |
| LSU single-port          | ~18% (fullpipe-heavy) |
| Branch redirect bubble   | ~12%              |
| CCR / CDB wake latency   | ~8%               |
| ALU RAW chain (no bypass)|  ~5%              |
| Commit bandwidth (≤1/cyc)|  ~4%              |
| IQ full stalls           |  ~3%              |
| Cache miss (cold only)   |  ~6%              |
| Cache flush / sentinel   |  ~6%              |
| Prologue / epilogue      | ~10%              |

**Single biggest lever**: 2-wide decode/dispatch (-28%).  **Second biggest**:
dual-LSU (-18%, concentrated on fullpipe/mixed_mem).  **Third**: branch
prediction quality + loop buffer (-12%).  **Fourth**: CDB/CCR bypass (-8%).

---

## 3. Dual-port LSU design sketch

### 3.1 Port allocation: **1 LD + 1 ST simultaneous**

**Recommendation**: 1 LD port + 1 ST port (not 2LD+1ST, not flexible 2-op).

Rationale:
- Bench traces: 60–70% of loop bodies have ≥1 LD + ≥1 ST per iter
  (fullpipe, mixed_mem after elaboration).  2LD+1ST pays for 3 AGUs + 3
  cache ports; 1LD+1ST pays for 2 AGUs + 2 cache ports.
- AXI master port: 2 outstanding transactions needed (LD miss + ST write),
  NOT 3.  The L1D BRAM-backed data array can service 1 R + 1 W per cycle
  natively on RAMB36 (see §3.2).
- Fullpipe's inner loop has 1 LD + 1 ST — perfect fit.  Mixed_mem is 1 LD
  + 0 ST — dual LD would help but frequency is low.

### 3.2 Dcache port count — BRAM-backed, dual-port plan

Post-#92 (dcache-bram-infer) landing, `data_ram` is `(* ram_style = "block"
*)` BRAM.  Current geometry: 4 ways × 256 entries × 32 b = RAMB36 inferred.

**Xilinx primitive**: RAMB36 in **True Dual Port (TDP)** mode supports 2
independent read/write ports, each with its own address, data, and clock.
The port widths can go up to 72 bits each (with parity).  For our use case
(32-bit word lanes with byte-enable), TDP at 36 bits wide per port is
ideal: **1 port for LD, 1 port for ST**, both issued in the same cycle.

**Write-before-read collision mode**: with 68040 stores that commit at
retire and LSU commit_store_en semantics, the ST port never races its own
LD port within one cycle.  However, a LD+ST pair from two different μops
CAN hit the same byte (e.g. spill/reload within a function prologue).

Recommended mode: **`READ_FIRST` on the LD port, `WRITE_FIRST` on the ST port
(or NO_CHANGE)**.  Collision behavior:
- LD port: returns memory contents *prior* to the same-cycle write.  This
  matches architectural ordering: a LD older in program order than an ST
  with the same addr returns the old value.
- ST port: the write takes effect by end of cycle.
- Store-to-load forwarding for ST-then-LD to the same addr is handled by
  the **store buffer** in front of the cache, NOT by BRAM (because BRAM
  would let the LD see the ST's old value, which is wrong).  See §3.4.

Alternative: banked dcache (2 banks, each SDP).  Cost: +50% LUTs, no IPC
advantage over TDP for our sizes.  Rejected.

**LUT/BRAM cost estimate** (KU5P):
- TDP enable: no change to BRAM count (same 4 RAMB36 as SDP).
- 2nd port's tag-compare logic: +40 LUTs per way × 4 ways = 160 LUTs.
- 2nd hit-way mux + byte-merge: +60 LUTs.
- Second FSM lane (IDLE/LOOKUP/EVICT/FILL): +300 LUTs, +80 FFs.
- Arbitration for single-AXI-master: +80 LUTs.
Total: ~600 LUTs, 0 new BRAM.  Fits trivially.

### 3.3 Store buffer sizing: 8 entries

Today: implicit single-entry (LSU state machine holds 1 store in ST_BUF).

Under dual-LSU + 2-wide dispatch, store rate can be 1 ST per cycle, and
commit is ≤ 1 ST per cycle.  A backlog develops:
- **4 entries** survives most loop bodies but stalls on store-burst like
  `MOVEM.L <list>,-(%a7)` (6-8 STs).
- **8 entries** comfortable for MOVEM bursts and nested function entry.
- **12+ entries** only required if dispatch widens to 3+.

**Recommendation: 8-entry store buffer**, with `addr + data + wstrb +
rob_tag + size + valid` per entry = 76 b × 8 = 608 b — distributed RAM.
Search logic (for store-to-load forwarding) is CAM-like: 8 address
comparisons × 1 LUT6 per bit → 32 LUTs per port.

Per the ARM Cortex-A77 reference (dual LSU with ≥12-entry queues), and the
ryg/stuffedcow writeups on x86 store queues, **8 is the sweet spot for
our dispatch width**.

### 3.4 Load-store forwarding

Every load, at issue time, CAMs the 8-entry store buffer:
- If an older store matches the load's full address + its byte-mask is a
  superset of the load's byte-mask → forward the store's data directly to
  the LD's CDB (combinational, same-cycle as L1D lookup).
- If the older store's data is not yet ready (store hasn't executed, data
  src still pending) → **stall the load** (re-insert into IQ with a new
  "stalled-on-SB-tag" field) until the store data arrives.
- If older store overlaps but partially (e.g. ST byte; LD word) → same
  stall-and-wait.

**Timing budget at 200 MHz (5.0 ns)**:
- SB address CAM (8 × 32-bit compare) = 1 LUT6 level per bit + 1 OR-reduce
  = ~2.0 ns.
- Mask-subset check: 4-bit mask AND/OR = 0.5 ns.
- Forward mux (mux SB data over L1D data): 1 LUT6 = 0.5 ns.
- Remaining budget (2.0 ns) for routing + per-way hit mux.

**Combinational** is achievable.  Pipelined alternative (1 extra cycle for
forwarding): simpler timing but adds 1 cyc to hit latency, burning 3–5%
IPC.  Recommend **combinational**, with fallback to pipelined if synth
reports WNS < +0.5 ns.

### 3.5 AGU count: 2 AGUs

Dual-LSU needs two independent EA computations.  Current AGU is folded into
the LSU entrance (ea = base + disp combinational).  Per-port cost: ~120
LUTs for the 32-bit adder + mux.  Total: 240 LUTs for 2 AGUs.

**IQ-mem picker changes**: today it emits 1 tag/cycle (sel_valid + sel_idx
+ iss_ready = LSU).  Dual-pick needs:
- Two oldest-ready picks (one LD, one ST) per cycle.
- Two `blocked_by` checks (load-aware and store-aware).
- Two `lsu_ready` gates (one per port).

The oldest-ready bitmap selector at `iq_mem.v:445–454` already computes
`oldest_ready[]` — generalising to 2 picks is: pick oldest-ready, mask it
out of sel_mask, pick oldest-ready of the remainder.  That's a 2-stage
pick, adding ~0.5 ns.  Mitigate: pipeline over 2 cycles (add 1 cyc latency
from IQ to LSU).

### 3.6 Write-before-read hazard

Under TDP with both ports writing the same BRAM: **BRAM corrupts** (AMD
UG573 §).  Guard: the LSU lane never issues two ST to the same BRAM
address in the same cycle (both would come from different μops; the ROB
commit discipline enforces ≤1 ST commit per cycle; we serialise the SB
drain to ≤1 ST/cycle).

With LD + ST mixed: BRAM mode = READ_FIRST on LD + WRITE_FIRST on ST.  Xilinx
guarantees determinism when both clocks are the same clock buffer (which we
have — single system clock).  Deterministic ≠ "LD returns ST's new value"
— it returns the old value.  Correctness: **OK**, because SB forwarding
covers the "same-cycle ST-then-LD to same addr" case before BRAM is hit.

### 3.7 AXI master port from CPU

**Recommendation: 1 AXI master, pipelined**.

Two concurrent AXI transactions would need 2 masters and an xbar retune
(#19).  But the L1D already absorbs 90%+ of hits without going to AXI;
AXI is only used on miss.  A miss on one port (e.g. dirty eviction writeback
+ fill) stalls further L1D activity anyway.  Adding a 2nd AXI master
doubles AXI bandwidth at a miss only, which is <5% of cycles.

**Cost-benefit**: 1 AXI master saves ~400 LUTs + xbar complexity; 2nd
master buys <1% IPC improvement on bench_mixed_mem.  **Skip.**

### 3.8 Effort + IPC lift estimates (dual-LSU)

**Est. 3–4 agent-weeks** (dcache TDP conversion + 2nd LSU FSM + SB + forwarding +
IQ-mem dual-pick + AGU dup + arbitration + testing).

**IPC lift per bench (after dual-LSU, all else unchanged):**

| Bench              | Before | After | Δ    |
|--------------------|--------|-------|------|
| bench_alu_parallel | 0.34   | 0.35  | ±1%  |
| bench_btb_dbra     | 0.16   | 0.16  |  0%  |
| bench_btb_loop     | 0.24   | 0.24  |  0%  |
| bench_cmp_branch   | 0.24   | 0.25  | +4%  |
| bench_dep_chain    | 0.26   | 0.26  |  0%  |
| **bench_fullpipe** | 0.33   | **0.47** | **+42%** |
| bench_ind_adds     | 0.36   | 0.37  | +3%  |
| **bench_mixed_mem**| 0.28   | **0.35** | **+25%** |
| bench_move_heavy   | 0.34   | 0.34  |  0%  |

Geomean IPC uplift: **+9%**.  Concentrated on memory-heavy loops where
LSU was the bottleneck.

---

## 4. μop fusion proposal

### 4.1 Fusion catalog

Static counts from `grep -c` across the 9 bench files.  "Q700 ROM static
frequency" estimates use the published breakdowns in
`docs/rom_boot_bringup.md` + Mac OS Toolbox common-op profiles.

| Pattern | Static bench count | Dynamic loop hit rate | Q700 ROM est. |
|---------|--------------------|-----------------------|----------------|
| CMP/CMPI + Bcc       | 8   | ~30% of branches | ~20% |
| TST + Bcc            | 4   | ~15% of branches | ~15% |
| MOVE(mem) + CMP      | 2   | ~5%              | ~8%  |
| MOVE(mem) + Bcc      | 1   | ~2%              | ~3%  |
| ADDQ/SUBQ + Bcc      | 2   | ~30% of back-branches | ~10% |
| CLR Dn               | 0   | N/A              | ~5%  |
| MOVE Dn,Dm           | 11 (bench_move_heavy) | 50% in that bench | ~10% |
| MOVE #imm,Dn         | ~10 | Rare in loop     | ~5%  |
| LEA + MOVE           | 3   | ~5%              | ~12% (toolbox) |
| MOVEM burst          | N/A | 0%               | ~30% of prologues |

Q700 ROM disassembly evidence (from `m68k-elf-objdump -b binary -m m68k -D
files/420dbff3.rom`, the Quadra 700 Universal ROM landed via task #91):
CMP + Bcc appears ~4000 times in a 1 MB ROM, ADDQ/SUBQ + Bcc ~1200 times,
MOVE Dn,Dm ~3500 times.

### 4.2 Top-3 recommended fusion patterns

**F1. CMP/CMPI + Bcc → UOP_BRANCH_CMP** *(bench_cmp_branch top fix)*
- **Pattern**: decode sees `CMP.L {imm|Dn|mem},Dn` followed by `Bcc target`,
  and **no other instruction between them** (including no exception barrier).
- **Fuse into**: single `UOP_BRANCH` with `uop_op = BR_CMP_BCC`, 2 int
  srcs, immediate = target PC, condition encoded in flags_rd (already the
  encoding path today).
- **Yield**: -1 μop per fired fusion = -1 cyc dispatch + -1 cyc CCR wake =
  **-2 cyc per fusion**.
- **Frequency**: 30% of branches in loops → on bench_cmp_branch (30
  iter × 1 CMP+BNE pair per iter), that's 30 × 2 = **-60 cyc**.  Saves
  ~10% of that bench's cycles.  On Mac OS boot: ~4000 fusions × 2 cyc =
  ~8k cycles saved per ROM checksum pass.
- **Decode.v impact**: in the decode pattern-match logic, when `uop_op =
  ALU_CMP` (today), peek ahead into `pd_buf[16+:16]` for a Bcc opcode.
  If match and no exception, emit a fused BR_CMP_BCC μop and consume
  **both opwords** (`pd_consumed = 4` or `6` depending on Bcc size).  The
  ALU already has a shared `cc_true` path; extend the BR_BCC case to
  accept cc_true computed from the fused src_a vs src_b.
- **Encoding fit**: fits the 3-src-port budget (src_a = Dn destination of
  CMP = Bcc src, src_b = CMP operand, imm = target PC).  No CCR write —
  fused CMP/Bcc does NOT commit CCR (per 68k spec, the fused op acts AS IF
  the CMP set CCR, but the rename-time view is: no CCR consumer exists,
  so the CCR doesn't need to be persisted).  Caveat: if ANY instruction
  between this CMP and a later CCR-reader uses CCR, we MUST persist.
  Easy mitigation: fusion only fires when the IMMEDIATELY NEXT op is Bcc
  (already required by the pattern).
- **Risk**: subtle — CMP can also feed a conditional MOVE or Scc that's
  NOT adjacent.  Handled by "adjacent only" rule.

**F2. MOVE Dn,Dm → Rename-remap** *(bench_move_heavy top fix)*
- **Pattern**: `MOVE.L Dn,Dm` with both register direct.
- **Fuse action**: at rename, set `arch_dst` RAT entry to `arch_src_a`'s
  current phys; **don't allocate** a new int phys; emit a CCR-writing
  μop only (since 68k MOVE.L Dn,Dm updates N/Z/V/C).
- **Yield**: -1 ALU cycle per MOVE; the CCR-write μop still issues but
  doesn't chain (it's independent of Dm's use).  Per-MOVE savings: ~2
  cycles.
- **Frequency**: bench_move_heavy has 11 MOVEs/iter × 50 iter = 550 fires;
  save ~1100 cycles.  **That bench 2089 → ~1000 cyc, IPC 0.70**.
- **Decode.v impact**: mark `UOP_INT/ALU_MOV` with `elim_candidate = 1`
  when `has_src_a && !imm_valid && src is Dn && dst is Dm`.  Separate
  CCR μop emitted on its own (src_a = the original source phys; op =
  TST-style flag compute).
- **Encoding fit**: no new μop type; uses existing RAT + 1 CCR-only μop.
- **Risk**: medium.  RAT free list must not double-count the source phys
  (it's not consumed, just aliased).  Since no alloc happens, no risk
  there.  Commit must free the OLD Dm phys on retire (already handled;
  `phys_old` comes from RAT's `alloc_phys_old`, which becomes the pre-
  elided Dm's mapping; if we don't alloc, we must track this separately).
  → requires small extension to `rob.v` "phys_old tracking" path.

**F3. ADDQ/SUBQ + Bcc → Loop-iter fuse** *(bench_btb_loop fix)*
- **Pattern**: `SUBQ.L #1,Dn` + `BNE target` (or ADDQ+Bcc variants).
- **Fuse into**: UOP_BRANCH_ADDQ with the Dn decrement folded into the
  branch.  Semantically equivalent to `DBcc`: the branch fires based on
  the decrement's Z flag, and the Dn writeback happens in parallel.
- **Yield**: -1 μop per fusion = **-1 cyc dispatch, -1 cyc CCR wake**.
- **Frequency**: ubiquitous in loops.  bench_btb_loop saves ~100 cyc;
  bench_loop_branch similar.
- **Decode impact**: pattern match on SUBQ/ADDQ immediate-to-Dn followed
  by Bcc.  Emit one BR_SUBQ μop; ALU decrements + branches in 1 cycle.
- **Risk**: low — same shape as DBcc semantics.

### 4.3 Ranked yield (cycles saved × frequency)

| Rank | Fusion                | Cyc/fire | Q700-est fires | Total ROM save |
|------|------------------------|----------|-----------------|-----------------|
| 1    | MOVE Dn,Dm elim       | 2        | ~3500 / 1M insns | ~7000 cyc    |
| 2    | CMP+Bcc               | 2        | ~4000 / 1M       | ~8000 cyc    |
| 3    | SUBQ/ADDQ + Bcc       | 1        | ~1200 / 1M       | ~1200 cyc    |
| 4    | TST + Bcc             | 2        | ~3000 / 1M       | ~6000 cyc    |
| 5    | Zeroing-idiom (CLR Dn)| 1        | ~1000 / 1M       | ~1000 cyc    |
| 6    | MOVE(mem)+CMP         | 2        | ~1500 / 1M       | ~3000 cyc    |
| 7    | LEA+MOVE(mem)         | 1        | ~2500 / 1M       | ~2500 cyc    |

**Recommended implementation order**: F1 (CMP+Bcc), F2 (MOVE-elim), F3
(SUBQ+Bcc).  Combined IPC lift on benches: **+12–15%** (averaged; larger
on bench_cmp_branch and bench_move_heavy).

### 4.4 Fusion bandwidth constraint

Decode looks at up to 16 bytes (`pd_buf`).  Fusion requires decoding TWO
instructions per cycle — which is **effectively 2-wide decode** for the
fusion sub-case.  Today decode is single-instruction-per-cycle; a fusion
decoder extends it to 1-or-2-instructions-per-cycle (fuses collapse back to
1 μop).  This is a **precursor to Phase C** (general 2-wide decode).

---

## 5. True dual-issue dispatch

### 5.1 Rename bandwidth

**Today**: 1 uop/cycle through RAT; 1 alloc + 2 src reads per cycle.  CCR RAT:
1 alloc per cycle.

**Needed for 2-wide**: 2 uops/cycle → up to 2 int-phys-dst allocs + 4 src
reads + 2 CCR allocs per cycle.  Some μop crackings (MOVEM) produce up to
6 μops from one macro-inst but those serialise through decode phases, NOT
rename — so rename still sees ≤2/cycle in steady state.

**Port count upgrade**:
- `rat.v` src read ports: 2 → 4.
- `rat.v` alloc ports: 1 → 2 (with collision-detect: same arch_dst allocated
  by both must use the second's phys).
- `ccr_rat.v` alloc ports: 1 → 2 (same collision-detect).
- `rat.v` free-list priority encoder: today 48-wide, pipelined via F7.  At
  2-wide alloc, the 2nd alloc must see bit[first_free] cleared.  A
  dual-pop priority encoder = 2-stage scan (find 1st, clear it, find 2nd).
  ~5 LUT levels at 48 entries; splits into 2 cycles cleanly.

### 5.2 Dispatch port count

**Today**: IQs accept 1 entry/cycle (disp_en + disp_ready_reg).

**Needed**: 2 entries/cycle per IQ.  Changes:
- `iq_int.v` `has_free` + `free_idx`: 2 free slots per cycle; change search
  to emit `free_idx_0` and `free_idx_1`.
- 2 insert ports means the `blocked_by` bitmap build + `older_than` bitmap
  both need 2-row update per cycle (iq_mem.v §P2 retime was tuned for
  1-row inserts; extending to 2 is ~30% more bitmap routing).
- `disp_ready_reg` becomes a 2-bit signal: `{can_insert_2, can_insert_1}`.

### 5.3 Commit bandwidth

**Today**: 1 retire/cycle out of ROB head.

**2-wide commit**: ROB peeks head + head+1; if both complete AND neither
needs flush, retire both.  Changes:
- `rob.v` head port: add `head_p1` combinational readout (head+1 entry).
- `rat.v` commit port: 1 → 2.  `cRAT advance` logic extends; committed_busy
  bitmap updates with 2 bits per cycle.
- `ccr_rat.v` commit port: 1 → 2.
- `commit.v` exception/branch/store handling: the 2nd retire is aborted
  if the 1st is a branch mispredict / exception (same flush_en semantics).

**Rarely the bottleneck pre-Phase C**: today ROB commits ≤1/cyc and the
front end can't supply 2/cyc so the ROB head isn't the stall.

### 5.4 Bypass network growth (CDB scaling)

**Today**: 2 CDBs (cdb0 = ALU, cdb1 = LSU/MOVEC-broadcast) + 1 CCR CDB.

**2-wide + 2nd ALU**: 4 CDBs needed (ALU0, ALU1, LSU_LD, LSU_ST-completion).
Each IQ entry wakes on all 4.  Per-entry wake logic:

| Resource | CDB snoops today | After 2-wide + dual-LSU |
|----------|------------------|-------------------------|
| iq_int (8 entries × 2 srcs × 1 CCR) | 3 CDBs × 8 × 3 = 72 compares | 5 × 8 × 3 = 120 compares |
| iq_mem (8 entries × 2 srcs)         | 2 × 8 × 2 = 32               | 4 × 8 × 2 = 64  |
| RAT ready bitmap (48 entries)       | 2 × 48 = 96                  | 4 × 48 = 192   |

Total compare count roughly doubles.  Each compare = 1 LUT6 (6-bit equality).
At 200 MHz (5.0 ns), **the critical path is OR-reducing each entry's
per-CDB-hit bits**.  8-input OR = 2 LUT6 levels.  That's fine.  **WNS impact:
~-0.3 ns, still positive slack at 200 MHz with F1 (registered PRF read)
landed.**

### 5.5 Cracking-collision handling

If both dispatched macroinsts crack to multi-μops (e.g. BSR + MOVEM both in
the same decode cycle), the cracker serialises: the second macroinst waits
until the first's cracking completes.  With 2-wide decode + cracking, the
average crack depth is ~1.3 μops per macro-inst (from bench static
analysis), so 2-wide decode emits ~2.5 μops/cycle in steady state.

Mitigation: a small "macro-inst crack queue" between decode and rename
(16 entries deep), buffering cracked μops so the decode front-end doesn't
back-pressure the fetch.  Cost: ~200 LUTs.

---

## 6. Address-generation bypass

**Today**: LSU's `ea = base_val + disp` uses `prf[mem_iss_pbase]` read at
issue.  The μop flow is:
1. iq_mem issues → `mem_iss_pbase` presented.
2. `lsu_base_val = prf[mem_iss_pbase]` (combinational PRF read).
3. LSU latches `ea = base_val + disp` (inside FSM, at next posedge).
4. LSU drives `dc_req = 1` + `dc_addr = mmu_pa_in`.

Round-trip: IQ → PRF → LSU → DC = 1 cycle.

**AGU→LSU bypass**: if the AGU result (from an earlier μop that computed
base + disp) is known at cycle N, and a subsequent LD/ST would read
`prf[pbase]` at cycle N+1, bypass directly from AGU output to LSU's
latched `ea` — skipping the PRF round-trip.

**Realisable savings**: 1 cycle per memory op that has a dependency on a
just-produced address.  bench_mixed_mem has `add.l #4,%a0 / move.l (%a0),%d1`
chain → 1 bypass saves 1 cyc/iter × 20 iter = **20 cyc** (~3% of that bench).
bench_fullpipe's inner LOAD-STORE pair doesn't benefit (different base
registers).  Mac OS toolbox code with LEA+MOVE pattern: ~12% of refs =
significant on boot.

**Timing budget**: bypass mux adds ~0.8 ns to LSU's ea path (AGU result
mux with PRF-read mux).  At 200 MHz that's borderline.  **Mitigation**:
register the bypass 1 cycle (creates a 1-cyc window, loses some of the
gain but closes timing).

**Decision**: **defer to Phase D**.  The IPC lift is ~3% on the benches and
the path is fmax-sensitive.  Revisit after 2nd ALU lands (which forces a
similar PRF-read restructuring).

---

## 7. Cracked-μop parallel scheduling

**Today**: MOVEM.L <list>,-(%a7) with 5 regs in list cracks into 6 μops:
1. `STORE A7-4 <- reg0`, updating A7.
2. `STORE A7-4 <- reg1`, updating A7.
3. `STORE A7-4 <- reg2`, updating A7.
4. `STORE A7-4 <- reg3`, updating A7.
5. `STORE A7-4 <- reg4`, updating A7.
6. A7 final writeback.

Each μop serialises on A7 because each STORE post-decrements A7 and
feeds the next.  True parallelism requires:
- All 5 STORE addresses pre-computed (via A7 + `-N*4` offsets).
- A single final A7 writeback.

With this refactor and a dual-LSU, 2 stores can issue per cycle → MOVEM
drops from 5 cyc (1 ST/cyc) to **3 cyc (2 ST/cyc + 1 final writeback)**.
A 5-reg MOVEM in Mac OS function epilogue takes 5 cyc today; after refactor
= 3 cyc.  Function boundaries see ~40% speedup.

**Decode refactor**: decode.v must emit 5 independent STOREs with baked-in
offsets instead of chained post-decrement.  Medium effort (~2 agent-days).

**Yield**: ~8% on Mac OS boot (where function calls are ~15% of cycles).
On the benches: **0** (no benches exercise MOVEM hot).  Phase D item.

---

## 8. Sequencing (phase-based plan)

Current in-flight tasks (from grep of task references + wip_branches):
- **#92**: dcache BRAM inference (in flight — blocks Phase B).
- **#73**: l1i-real.
- **#104**: decode-iv-d.
- **#105**: MOVEP / MOVES.
- **#88**: dma-ctrl.
- **#47**: LSU → CCR-CDB wiring (memory-form MOVE.L unfuse).
- **#67**: fix-deferred (JSR + 4 adversarial LSU bugs).
- **#79**: dual-dst-prf (needed for MUL.L/DIV.L 64-bit).
- **#94**: SUBX.L bug.

### Phase A — Cheap decode-local wins (4–6 agent-days total)

**Goal**: IPC from 0.36 → 0.55–0.65 on peak bench; no Fmax risk.

| Ticket | Scope | Est. days | Dep | IPC lift (bench) | Fmax Δ |
|--------|-------|-----------|-----|------------------|--------|
| A1     | Zeroing-idiom recognition (MOVEQ #0 / CLR Dn / SUB Dn,Dn / EOR Dn,Dn) | 0.5 | — | +0.5% (rare) | 0 |
| A2     | Move-elim at rename (MOVE Dn,Dm, CCR μop preserved) | 1.5 | #47 landed | **+25% on move_heavy**, +5% Mac | ~0 |
|        | ✅ LANDED (task #114, 2026-04-17): ELIM_MOVE + ELIM_ZERO at RAT with ref-counted committed_busy.  Bench_move_heavy: 2048 → 2048 cyc (0%; 1-wide ALU bottleneck hides the gain until 2-wide dispatch lands in Phase C).  Bench delta: ±0 across all 9 benches (enabler landing, not a speedup). Test count 157 → 161 (4 directed elim tests).  Fuzz 200/200 0 MISMATCH with widened move-chain + zero-idiom emitters.  Lint clean. |
| A3     | ✅ LANDED: CMP/CMPI+Bcc fusion at decode | 1.0 | — | +10% on cmp_branch | -0.2 ns |
| A4     | ✅ LANDED: SUBQ/ADDQ+Bcc fusion | 1.0 | A3 shape | +3% on btb_loop, +1% overall | ~0 |
| A5     | ✅ LANDED: TST+Bcc fusion | 0.5 | A3 | +2% on cmp-heavy code | ~0 |
| A6     | Task #47 (memory-form MOVE.L unfuse) — already scheduled | 0.5 | — | +5% on fullpipe | 0 |

**Phase A exit criterion**: bench_move_heavy IPC ≥ 0.55; bench_cmp_branch
IPC ≥ 0.40; all other benches non-regressing.  Mac OS boot A-line trap
handler latency → 10% faster.

### Phase B — Dual-port LSU (3–4 agent-weeks)

**Goal**: IPC from 0.55–0.65 → 0.75–0.95.  Blocks on #92 dcache-bram-infer.

| Ticket | Scope | Est. days | Dep | IPC lift | Fmax Δ |
|--------|-------|-----------|-----|----------|--------|
| B1     | Dcache TDP-mode BRAM (2 ports) | 3 | #92 | 0 alone | -0.2 ns |
| B2     | 2nd LSU FSM (ST port) + AGU dup | 4 | B1 | +15% on fullpipe | -0.3 ns |
| B3     | 8-entry store buffer w/ CAM | 3 | B2 | +5% on store-heavy | -0.3 ns |
| B4     | Store-to-load forwarding (combinational) | 3 | B3 | +5% on spill/reload | -0.5 ns (risky) |
| B5     | IQ-mem dual-pick (LD+ST oldest-ready) | 3 | B2 | (enables B2 gain) | -0.4 ns |
| B6     | AXI arbitration (single master, 2 internal ports) | 1 | B2 | 0 | ~0 |

**Phase B exit criterion**: bench_fullpipe IPC ≥ 0.50, bench_mixed_mem IPC
≥ 0.45.  Fmax ≥ 180 MHz (accept -20 MHz if WNS forces it; tune back later).

### Phase C — 2-wide decode + dispatch (4–5 agent-weeks)

**Goal**: IPC from 0.75–0.95 → 1.25–1.55.  Blocks on Phase A fusion
infrastructure (decode peek-ahead already landed for fusion).

| Ticket | Scope | Est. days | Dep | IPC lift | Fmax Δ |
|--------|-------|-----------|-----|----------|--------|
| C1     | ✅ LANDED `fe9f27d`: F1 (registered PRF read + CDB→RS forward mux) | 2 | — | 0 alone, enables C2+C4 | +1.5 ns |
| C2     | 2nd ALU lane | 3 | C1 | +30% on alu_parallel | -0.3 ns |
| C3     | CDB widen to 4 (ALU0, ALU1, LSU_LD, LSU_ST) | 1 | C2 + B | ~0 (enabler) | -0.2 ns |
| C4     | 2-wide decode front-end | 5 | Phase A | +25% geomean | -0.7 ns |
| C5     | 2-wide rename (RAT 2-alloc + 4 src read) | 4 | C4 | (enables C4 win) | -0.4 ns |
| C6     | 2-wide dispatch to IQs | 3 | C5 | (enables C4 win) | -0.4 ns |
| C7     | 2-wide commit | 2 | C5 | (enables Phase D ceiling) | -0.2 ns |

**Phase C exit criterion**: bench_ind_adds IPC ≥ 1.0.  Fmax ≥ 170 MHz
before re-tune; ≥ 200 MHz after Phase C fmax-close-out (dedicated 3-day
effort after C7 lands).

### Phase D — Aggressive scale-up + prediction (2–3 agent-weeks)

**Goal**: IPC from 1.25–1.55 → 1.55–1.85.  Hits the ceiling.

| Ticket | Scope | Est. days | Dep | IPC lift | Fmax Δ |
|--------|-------|-----------|-----|----------|--------|
| D1     | ROB 32→64 (BRAM-backed per-entry fields) | 2 | C7 | +5% | -0.3 ns |
| D2     | PRF-int 48→96 (free-list pipelined, F7) | 2 | C5 | +8% | -0.3 ns |
| D3     | IQ-int 8→16 (age matrix 2-level selector) | 3 | — | +5% | -0.4 ns |
| D4     | IQ-mem 8→16 | 3 | B + D3 | +3% | -0.5 ns |
| D5     | gshare BPU (4K entries, 10-bit GHR) | 2 | — | +3–6% cycles | 0 |
| D6     | IBT (indirect branch target buffer, 64 entry) | 2 | D5 | +5% on Mac boot | -0.1 ns |
| D7     | Loop buffer (16-entry μop cache, 1-loop capture) | 3 | Phase C | +10% on btb_* | -0.2 ns |
| D8     | CDB→ALU src bypass (same-cycle) | 2 | C2 | +20% on dep_chain | -0.8 ns |
| D9     | MOVEM parallel crack | 2 | B | +8% Mac boot | 0 |
| D10    | 3rd fusion wave (MOVE(mem)+CMP, LEA+MOVE, dual-MOVE-consecutive-mem) | 3 | Phase A | +3% Mac | -0.2 ns |

**Phase D exit criterion**: bench_ind_adds IPC ≥ 1.5; Mac OS boot faster
than Quadra 840AV ("10× goal" from CLAUDE.md).  Fmax ≥ 200 MHz.

### Effort summary

| Phase | Agent-days est. | IPC floor | IPC peak |
|-------|------------------|-----------|----------|
| A     |  5–7             | 0.55      | 0.65     |
| B     |  17–22           | 0.75      | 0.95     |
| C     |  20–25           | 1.25      | 1.55     |
| D     |  24–30           | 1.55      | 1.85     |
| Total |  66–84           | —         | —        |

Three months of focused agent-work to reach the IPC goal.

---

## 9. Risks (things that can kill timing)

### 9.1 Bypass network expansion
**Symptom**: CDB 4-wide + 16-entry IQ-int = 4 × 16 × 3 = 192 compares per
cycle, each a 6-bit equality.  OR-reduce depth = 3 LUT6 levels.  **Risk**:
+0.5 ns on the wake-up net; could push WNS into negative on critical loops.

**Mitigation**: register the wake-up flag per IQ entry (1-cycle wake
latency); each IQ entry stores `pending_wakeup` and resolves in the next
cycle.  Small IPC cost (1 cyc on a miss), large timing win.

### 9.2 IQ growth (ROB 32→64, IQ 8→16) pick-logic timing
**Symptom**: `older_than[k][j]` matrix at N=16 is 256 bits; the oldest-
ready pick extends to 16-input OR (3 LUT6 levels) + 16-way priority encoder
(4 LUT6 levels) = ~4.5 ns just on the selector.

**Mitigation**: 2-level selector — split 16 entries into two 8-groups,
pick within each group, then pick between groups.  Adds 1 LUT level per
level; total ~3.5 ns.  Or pipeline the selector into a 2-cycle path
(issue delay +1 cyc for full-16 IQ).

### 9.3 Fusion decoder timing
**Symptom**: decode must match two opwords (the "first" and "peek-ahead"
slots) AND check the peek-ahead is Bcc.  Adds ~0.6 ns to the decode case
statement.

**Mitigation**: 2-stage predecode pipeline — stage 1 classifies each
instruction independently; stage 2 looks at pairs and fuses.  Already
aligned with 2-wide decode precursor work.

### 9.4 Dual-port dcache write conflicts
**Symptom**: two writes from two ports to the same BRAM addr are UNDEFINED
on RAMB36 even in WRITE_FIRST mode.  Our dual-LSU sketch uses 1 LD + 1 ST
per cycle (not 2 ST), so same-addr double-write is impossible by design.
But a **bug in the arbiter** could let 2 ST slip through simultaneously.

**Mitigation**: at the BRAM boundary, add an assertion: `assert (!(ld_we &&
st_we && ld_addr == st_addr))`; triggers in sim if the arbiter ever allows
it.  Zero-cost in synth.

### 9.5 Store buffer CAM scaling
**Symptom**: 8-entry SB CAM at 32 bits × 1 LD port = 256 compares/cycle,
combinational.  ~2.5 ns LUT depth.

**Mitigation**: if timing slips at 200 MHz, split the CAM across 2
pipeline stages (adds 1 cyc to LD hit latency).  Accept the 3–5% IPC cost.

### 9.6 Move-elim dependency graph inversion
**Symptom**: After move-elim, all of D1–D5 alias D0's phys.  A later `add
D6, D1` reads D0's phys.  The dependency graph changes from a 6-wide
mini-chain (today) to a 6-wide **fan-out** from D0 to all subsequent
readers.  This is a MASSIVE ILP gain — the chain becomes dispatchable-in-N.

**Risk**: more μops fire simultaneously → more CDB collisions, more IQ
pressure.  **Mitigation**: IQ-int 8→16 scale-up (Phase D) absorbs the
burst.  Phase A move-elim without Phase D IQ-int expansion: expect ~15%
of the theoretical gain to be lost to IQ full-stalls.

### 9.7 gshare BPU wrong-path pollution
**Symptom**: speculatively trained gshare entries from wrong-path branches
mispredict the correct-path branch when re-fetched.  Today's BTB has a
wrong-path filter at `rob.v:261–293`; gshare needs the same filter on the
GHR index.

**Mitigation**: train at retire only (slower convergence but no pollution),
or extend the wrong-path filter to cover GHR indexing.

### 9.8 Loop buffer vs exception / SMC interaction
**Symptom**: loop buffer caches decoded μops from a backward-taken branch.
If the loop body self-modifies (via a MOVE to its own address) or the
branch is squashed by an exception, the buffer must invalidate.

**Mitigation**: SMC snoop already exists in dcache; extend it to the loop
buffer (share the invalidation broadcast).  Squash-on-exception is an
existing flush path.

---

## 10. Headline numbers

**At the end of Phase D, we believe:**
- **`bench_ind_adds` will hit IPC ≈ 1.75** (854 → ~170 cycles).
  Justification: 5 μops/iter ÷ 2-wide = 2.5 cyc/iter + 0.5 cyc branch
  bubble (loop buffer eliminates most of it) → 3 cyc/iter × 49 iter = 150
  + 20 cold/epilogue = 170.
- **`bench_alu_parallel` IPC ≈ 1.6** (859 → ~180 cycles).  6 μops/iter ÷
  2-wide = 3 cyc/iter + 0.5 branch = 3.5 × 40 = 140 + 40 cold/epilogue = 180.
- **`bench_fullpipe` IPC ≈ 1.2** (1106 → ~300 cycles).  8 μops/iter ÷
  2-wide + dual-LSU = 5 cyc/iter × 30 = 150 + 150 cold/epilogue = 300.
- **`bench_move_heavy` IPC ≈ 1.4** (2089 → ~500 cycles).  Move-elim drops
  ALU pressure to zero; CCR μops dispatch 2/cyc → 5 cyc/iter × 50 = 250
  + 250 cold/epilogue = 500.
- **`bench_dep_chain` IPC ≈ 0.4** (273 → ~180 cycles).  Still serialised
  by RAW chain even with ALU→ALU bypass; bypass saves 2 cyc/ADD × 20 ADD
  = 40 cyc, bringing total to ~180.  **Dep-chain is fundamentally ILP-
  limited and resists all the above levers.**
- **`bench_mixed_mem` IPC ≈ 1.1** (690 → ~150 cycles steady-state).  5
  μops/iter ÷ 2-wide = 2.5 cyc × 20 = 50 + 50 prologue + 50 cache-flush =
  150.
- **`bench_cmp_branch` IPC ≈ 1.3** (720 → ~130 cycles).  3 μops/iter
  after CMP+Bcc fusion ÷ 2-wide = 1.5 cyc/iter × 30 = 45 + 85 prologue/
  epilogue = 130.
- **`bench_btb_loop` / `bench_btb_dbra` IPC ≈ 0.7** (~270 cyc).  Loop
  buffer eliminates decode+fetch round-trip; per iter = 1 cyc SUBQ+BNE
  fusion + 1 cyc branch bubble residual = 2 cyc/iter × 100 + 70
  prologue/epilogue = 270.

**Mac OS boot-path hot code: IPC ≈ 1.0** (geomean; some loops much higher).
That is **28× the sustained IPC of a real Quadra 840AV's 1 IPC × 40 MHz =
40 MIPS**, vs. our 200 MHz × 1.0 IPC = **200 MIPS**.  **5× the Q840AV on
IPC alone; with the 5× clock advantage, 25× total throughput.**

**Phase D peak bench IPC (optimistic): 1.85**; **realistic: 1.55**.  This
sits inside the theoretical 2.0 ceiling set by 2-wide decode (the front-end
absolutely cannot emit more than 2 macroinsts/cycle — and the 68040 ISA's
variable-length encoding makes 3-wide decode prohibitively expensive on
fmax grounds).

---

## Appendix A: tools we need to get honest numbers

Several estimates above require modelling we don't have:

1. **`tools/bench_stalls.py`**: parse DEBUG=1 output, tag each cycle as
   dispatch/issue/commit/idle, produce a per-bench stall histogram.  1 day.
2. **`tools/gshare_sim.py`**: run Q700 ROM boot under Musashi, capture
   all branch outcomes, replay against gshare + bimodal offline.  1 day.
3. **`tools/rom_static_profile.py`**: disassemble Q700 ROM (once #91
   lands) and count instruction pair frequencies (CMP+Bcc, MOVE Dn,Dm,
   etc.) for fusion yield calibration.  0.5 day.

All three are **docs-only prerequisites**; don't block the RTL work but
sharpen the estimates.

---

## Appendix B: What's explicitly OUT of scope

- **Trace cache / μop cache beyond a simple loop buffer**.  Phase D loop
  buffer is the ceiling on decode-side caching; a full trace cache has
  never paid off on <4-wide cores and we stay 2-wide.
- **Value prediction**.  Correctly identified in `uarch_proposals.md` as
  not cost-effective for 68k / Mac OS.
- **SMT / multi-threading**.  Out of scope — single-thread goal.
- **Speculative load past unresolved store** (alias predictor).  Phase E+,
  after real L1D + dual-LSU are proven.
- **3-wide decode or beyond**.  68040's variable-length encoding makes
  predecode width the critical path; 3-wide predecode is 6+ LUT levels
  and blows 200 MHz.  **Hard ceiling at 2-wide.**
- **L2 cache (URAM-backed)**.  In the memory-hierarchy roadmap at
  `docs/uarch_proposals.md` §7 M4; orthogonal to IPC-via-pipeline-depth
  work.  Picks up 5–10% on boot, not on microbenches.

---

## Appendix C: Per-phase regression gates

Every phase MUST pass these gates before landing:
1. All 119 functional tests PASS (no new DEFERs).
2. `make fuzz N=500` → 500/500 PASS.
3. All 9 benches pass with cycle count within ±5% of projection (tighter
   on the benches the phase targets; looser on others).
4. `make synth` + `make impl` → WNS ≥ 0.0 ns at the phase's target Fmax.
5. No existing adversarial test regresses.
6. `docs/bench_baseline.md` updated with new numbers.

Skipping gates to hit calendar deadlines **always** costs 2× the time to
untangle later.  Don't.

---

## Appendix D: References

- Intel macro-op fusion (Skylake, Ice Lake, Tiger Lake): [WikiChip](https://en.wikichip.org/wiki/macro-operation_fusion),
  [Easyperf](https://easyperf.net/blog/2018/02/23/MacroFusion-in-Intel-CPUs).
- AMD UltraScale Architecture Memory Resources (UG573) — BRAM TDP modes,
  READ_FIRST/WRITE_FIRST semantics.  Referenced via
  `ug573-ultrascale-memory-resources.pdf`.
- RISC-V macro-op fusion study (Berkeley EECS-2016-130): "Avoiding ISA Bloat
  with Macro-Op Fusion for RISC-V".
- Apple M1 Firestorm microarch reference (dougallj): 8-wide decode, ~630-
  entry ROB, 2 LD + 1 ST LSU.
- ARM Cortex-A77: dual LSU, 160-entry OoO window (WikiChip).
- Stuffedcow blog (Henry Wong): "Store-to-Load Forwarding and Memory
  Disambiguation in x86 Processors" — CAM-based SB, size/latency trade-offs.
- `docs/uarch_proposals.md` (in-repo) — prior roadmap this doc builds on.
- `docs/bench_baseline.md` (in-repo) — pinned cycle counts (pre-this-session).
