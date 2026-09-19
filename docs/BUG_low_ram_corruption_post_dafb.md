# BUG: low-RAM (0x2000-0x20BC) corrupted with self-referential pointers post-DAFB

**Status**: OPEN.  Identified on HW 2026-05-21 via JTAG dcache-probe +
PC trace ring inspection.
**Severity**: Medium — boot reaches DAFB checkerboard rendering, then
wedges in IRQ→wild-jump→IRQ cycle until A7 underflows and DBL_FAULTs.

## Symptom

After fresh reset, Q700 ROM boot:
1. Reaches DAFB checkerboard (visible on HDMI) — same milestone as
   pre-SMC-fix HEAD.
2. Around 25M retired insts: CPU spins in a `DBF D2, .` delay loop
   at ROM `0x40898ae2` (= some Mac OS timed wait).
3. Around 28-30M retired: exits the DBF loop, enters a VBL IRQ
   dispatch cycle.
4. The VBL handler at `0x40809b60` dispatches via `0x40809bc0`,
   which JMPs through `MEM[0x2078]`.
5. `MEM[0x2078] = 0x00002074` — a chained self-referential pointer
   in the VBLQueue node region.
6. CPU JMPs to PC=`0x2074`, executes the linked-list bytes as
   `ORI.B #imm, D0` instructions, walks low RAM as code.
7. Eventually decoded bytes form a `JMP` to a wild address in the
   `0x6dxxxxxx` range.
8. CPU at wild PC fetches from unmapped memory (= open-bus returns
   `0xFFFFFFFF` per docs/rom_boot_bringup.md §4.2a).
9. `0xFFFF` decodes as F-line trap → enters FPSP handler at
   `0x4088d9fe` → emulates → RTEs back to wild PC.
10. Cycle repeats until SP underflows past low RAM into peripheral
    space, then vec-4 illegal → vec-2 bus error → **DBL_FAULT** at
    `0x40802842` with `A7=0xfffffeba`.

## Direct evidence (HW @ 25M retired insts, pre-wild-jump)

JTAG dump of low RAM:
```
MEM[0x00000118] = 0x00002000     ← VBLQueue.qHead (LowMem global)
MEM[0x000001d4] = 0x50f00000     ← VIA1 base (correct)
MEM[0x000006e4] = 0x4080a1ee     ← valid ROM function pointer
MEM[0x00000d94] = 0xffffffff     ← TST.L gate at 0x40809b70

MEM[0x2040] = 0x0000203c
MEM[0x2044] = 0x00002040
MEM[0x2048] = 0x00002044
...
MEM[0x2074] = 0x00002070
MEM[0x2078] = 0x00002074         ← JMP target reads this
MEM[0x207c] = 0x00002078
MEM[0x2080] = 0x0000207c
MEM[0x2084] = 0x00002080
...
MEM[0x20BC] = 0x000020b8
```

Pattern: `MEM[N*4] = (N-1)*4` — each entry holds the previous entry's
address.  This is **NOT a valid VBLTask structure** (VBLTask layout
is qLink/qType/vblAddr/vblCount/vblPhase, 14 bytes per task; valid
qLink would point at the next task's qLink, not "self - 4").

PC trace ring at 30M retired (= wild-jump phase):
```
trace[19] pc=0x40809b84   ← MOVEM.L (A7)+, D0-D3/A0-A3
trace[20] pc=0x00000000   ← RTE pops saved_PC = 0
trace[21] pc=0x6dbbb3cd   ← wild PC reached from PC=0 execution
trace[22] pc=0x40809b60   ← VBL IRQ caught at wild PC, handler re-entered
trace[23] pc=0x40809bc0   ← dispatch routine
...
trace[31] pc=0x00000000   ← RTE again to 0
trace[32] pc=0x6dc75101   ← different wild PC (varies per iteration)
trace[33] pc=0x40809b60   ← IRQ re-entry
...
```

The wild PCs in `0x6dxxxxxx` range INCREMENT each iteration —
suggesting the corrupt RAM data being walked as code differs per
iteration (= maybe the cache state changes between iterations).

## What it is NOT

- **NOT a MOVE.L (An),An decoder bug**: `move_an_an_all.s` covers all
  A0-A6 with misaligned sentinels → 7/7 PASS in sim.
- **NOT the SMC coherency bug fixed in `7e7eea0d`**: that was a
  different wild-jump (to `0x001fe04c`) caused by stale DDR4 reads
  during I-cache fills.  This new bug is at `0x6dxxxxxx` and has
  different mechanics.
- **NOT an F-line emulator bug**: the F-line storm IS the FPSP
  faithfully handling 0xFFFF read from open-bus; it's a SYMPTOM, not
  the cause.

## Hypotheses for the corruption

(A) **Boot init step skipped**.  Mac OS ROM has init code that zeros
or populates the VBLQueue node region.  If our boot path skipped that
step (e.g., due to a peripheral that didn't respond as expected), the
region holds whatever was there from earlier — possibly a memory test
pattern OR garbage from a prior wild execution.

(B) **Cache coherency leftover from earlier wild execution**.  If an
earlier wild jump (= during pre-DAFB boot) executed code that wrote
to low RAM via D-cache stores, and those stores never reached DDR4
before the wild execution ended, the D-cache holds dirty lines.
Later reads via I-cache (post-snoop-fix) see the dirty data — which
encodes the self-referential pattern.

(C) **MEM[0x2078] is what some Mac OS init writes here intentionally**,
and the IRQ handler is supposed to be VECTORED elsewhere on a
properly-initialized boot.  Our impl is reaching this handler PATH
via a different bug upstream.

## 2026-05-21 update — MAME comparison + dispatch ruled out

MAME at the equivalent boot point (snapshot via `mame_state_dump.py
--pc 0x40898ae2`) has **IDENTICAL** `MEM[0x2078] = 0x2074`.  The
self-referential pattern at 0x2040..0x20BC is **normal Mac OS state**,
not corruption — it's the VBLQueue node region.

The IRQ dispatch chain at ROM `0x40809bc0` is:

```
0x40809bc0: MOVEA.L 0x01d4.W, A1        ; A1 = MEM[0x1d4] = 0x50f00000 (VIA1)
0x40809bc4: MOVEQ #0x7f, D0             ; D0 = 0x7F
0x40809bc6: AND.B 0x1a00(A1), D0        ; D0 &= VIA1[0x1a00] (byte)
0x40809bca: AND.B 0x1c00(A1), D0        ; D0 &= VIA1[0x1c00] (byte)
0x40809bce: MOVEA.W (d8, PC, D0.W*2), A0  ; brief-format PC-INDEXED!
0x40809bd2: MOVEA.L (A0), A0
0x40809bd4: JMP (A0)
```

The `307b 022a` opcode is **NOT (d16, PC)** as I initially thought —
it's `(d8, PC, D0.W*scale)` brief-format with **scale=2**.

For D0=0x23 at the dispatch moment:
- EA = pd_pc + 2 + d8 + D0.W * 2 = 0x40809BD0 + 0x2A + 0x46 = `0x40809C40`
- ROM_W[0x40809C40] = `0x0192`
- A0 = sign_ext_w(0x0192) = `0x00000192`
- MEM_L[0x192] = `0x4080B140` (valid ROM dispatch handler)
- JMP target = `0x4080B140` ✓

**JTAG break at PC=`0x40809BD2` on HW confirms A0=`0x00000192`** —
matching MAME's expected behavior.  So our HW correctly honors the
scale=2 brief-format addressing mode.  The dispatch is FINE.

## 2026-05-21 — JTAG-AXI bypasses D-cache (red herring)

Early reads of `MEM[0x17FFF8]` via JTAG-AXI returned `0x40801228`
(= SR=`0x4080`, invalid).  This looked like the IRQ frame was
corrupted on entry.

**This was a JTAG observation artifact.**  JTAG-AXI bypasses D-cache
and reads DDR4 directly.  The IRQ frame push lands in dirty D-cache;
DDR4 holds stale pre-push data.

D-cache probe at set=31 way=2 (tag=`0x5FF` = line `0x17FFE0`) revealed
the REAL IRQ frame contents:

```
word 6 (0x17FFF8):  0x20004080   ← SR=0x2000 (S=1), PC_hi=0x4080
word 7 (0x17FFFC):  0x02420064   ← PC_lo=0x0242, fmt/vec=0x0064 (VBL)
```

So the IRQ frame is **CORRECT** at entry: saved_PC = `0x40800242`,
SR = `0x2000`, vec = 25 (VBL autovector).

## Refined hypothesis (post-2026-05-21-investigation)

The wild-jump cycle's `RTE → PC=0` must come from the saved_PC
being corrupted to 0 **after** the initial IRQ entry push, **before**
the handler's RTE pops it.

Candidate causes:

(A) **Handler code at `0x4080B140` corrupts the frame.**  This
handler reads/writes Mac OS low-mem globals (0x1A00, 0x020C, 0x021F,
0x0208, 0x0200, etc.).  None of those should touch the IRQ frame.

(B) **Nested IRQ/exception during handler.**  SR after entry =
`0x2100` (I=1, masking L1).  L2+ IRQs could preempt, but Q700 VIA1
fires only at L1 in normal boot.  Unlikely.

(C) **SMC snoop pipeline race.**  The 2-cycle snoop response (commit
`e8e44d80`) could have a race where a D-cache STORE to the same line
as the IRQ frame happens between the snoop_query and snoop_resp,
causing the I-cache to install incoherent data — but this would
affect ICACHE FETCHES, not the frame stored on stack.  Unless the
RTE's load of saved_PC goes through I-cache (= it doesn't, RTE is
a data LOAD).

(D) **A D-cache writeback/eviction with wrong address.**  If the
D-cache writes back the IRQ frame line to the WRONG DDR4 address,
the frame in cache stays correct but DDR4 gets stale.  Then if the
line gets evicted+refilled, the refill reads stale DDR4 and the
frame becomes "0".  This is the inverse of the SMC bug.

(E) **D-cache snoop response returns wrong data.**  The snoop_resp
pipeline added in commit `e8e44d80` registers the data BRAM read on
cycle 1 (after the query at cycle 0).  If a STORE to the same line
happens at cycle 0.5 (between query and BRAM mux), the snoop sees
stale tag (= "dirty" from old line) but FRESH data (= the new line's
data after eviction).  Mismatch → wrong data returned to I-cache.

## 2026-05-21 further narrowing — wild-jump triggers from A-line trap

Bisected the retired-instruction count of the first wild PC via
halt-after-N:

| N (retired)  | pc_live        | exc_pc       | exc_count | state |
|--------------|----------------|--------------|-----------|-------|
| 29,207,000   | `0x4080e158`   | `0x4080bc52` | `0x12C6`  | ROM   |
| 29,208,000   | `0x6db6e891`   | `0x4086abb2` | `0x12C6`  | WILD  |

Within a 1,000-retire window, the CPU went wild.  The **last latched
exception** (`exc_pc`) was at ROM `0x4086abb2` = `a029` =
**`_HLock` toolbox A-line trap (vec 10)**.

A-line vec-10 handler is at ROM `0x408099b0` (= `MEM[VBR+0x28]`,
VBR=0).  It's a Mac OS toolbox dispatcher that:

1. Pushes A2 + D2 on stack.
2. `MOVEA.L 10(A7), A2`  — reads the IRQ frame's saved PC slot.
3. `MOVE.W (A2)+, D2`    — reads the A-line opcode word, post-incs A2.
4. Classifies the A-line by range (CMPI #0xA800, SUBI #0xAC00).
5. Branches to one of three dispatch paths.
6. Each path uses **`MOVE.L A2, 0x0C(A7)` or `0x14(A7)`** to overwrite
   the IRQ frame's lower-half (PC_lo + fmt/vec) with A2 (= post-A-line
   PC's bytes).
7. Path A (0x408099c6) also does `MOVE.L (full_format_src), 8(A7)` to
   overwrite SR + PC_hi from a dispatch-table entry.
8. Pops registers, RTSes (handler exits via RTS — RTS pops outer
   caller's PC; the modified IRQ frame remains for the outer RTE).

The handler at path A uses **full-format brief addressing**
(`2f70 25a0 1e00 0008` = `MOVE.L (full_format_src), 8(A7)` with
ext1=0x25A0 IS=1 BS=0 BD=word, BD=0x1E00).  If our V2 decoder doesn't
support this exact full-format shape correctly, the write could go
to wrong address or read wrong source.

**Concrete sub-hypothesis for next attempt**: the A-line handler's
full-format MOVE.L at `0x408099c6` writes to the wrong location,
corrupting the IRQ frame's SR + PC_hi slot.  When the outer RTE
eventually pops the frame, PC ends up wild.

But path A is only taken for `D2 in [0xA800, 0xAC00)`.  `_HLock` =
0xA029 is BELOW 0xA800, so BCS at 0x408099be is taken to path
0x408099f0 instead.  Path 0x408099f0 uses `MOVE.L A2, 0x14(A7)`
(no full-format).  So full-format isn't the issue for the `_HLock`
case.

**Better sub-hypothesis**: the handler's `MOVE.L A2, 0x14(A7)` at
0x408099f6 (path 2) writes 4 bytes at A7+0x14, intended to
overwrite the IRQ frame's PC_lo + fmt/vec slot.  After 2 register
pushes (D1, A1, 8 bytes) + 1 earlier push (A2 + D2, 8 bytes) we
should have A7+0x14 = IRQ_frame +12 = PC_lo slot.

If our impl's `MOVE.L An, (d16, A7)` store goes to a slightly-off
address, the saved_PC gets corrupted but in a SPECIFIC way (= the
post-A-line PC ends up in the wrong word of the frame).

The wild PCs are in `0x6dxxxxxx` range.  If saved_PC = `0x6db6XXXX`
in the frame, RTE returns to wild.  Where does `0x6dXX` come from?

`0x6dxxxxxx` could come from the original `_HLock` PC `0x4086abb4`
shifted by 2 bytes + offset.  Worth bit-comparing.

## 2026-05-21 — A-line dispatcher disassembled, MOVE.L verified correct

Disassembled the A-line path 2 dispatcher at `0x408099f0..0x40809a18`.
The dispatch sequence:

```
0x408099f0: push D1                            ; A7 -= 4
0x408099f2: push A1                            ; A7 -= 4
0x408099f4: MOVE.W D2, D1
0x408099f6: MOVE.L A2, 0x14(A7)                ; write A2 to MEM[A7+0x14]
0x408099fa: ANDI.W #0x0100, D2
0x408099fe: BNE.s _path2b (= 0x40809a20)
0x40809a00: MOVE.B D1, D2
0x40809a02: push A0                            ; A7 -= 4
0x40809a04: JSR (memind via A0)                ; dispatch via toolbox table
0x40809a0a: pop A0
0x40809a0c: pop A1
0x40809a0e: pop D1
0x40809a10: pop D2
0x40809a12: pop A2                             ; A7 = IRQ frame start
0x40809a14: TST.L D0
0x40809a16: ADDQ.L #4, A7                      ; skip SR + PC_hi slot
0x40809a18: RTS                                ; pop 4 bytes (= the slot
                                               ;   we modified at +0x14)
                                               ; PC = saved-PC-overridden
```

So the dispatcher's RTS pops the IRQ frame's PC_lo + fmt/vec slot,
which was overwritten with `A2` (= post-A-line PC) via the
`MOVE.L A2, 0x14(A7)` store.  This RTS-as-RTE is the toolbox trick.

**HW verified working**: at exc_count=0x9B (= early boot, 155
exceptions), broke at PC=`0x408099fa` right after the
`MOVE.L A2, 0x14(A7)` store.  A7=`0x17FFE4`, A2=`0x40803E0A` (=
post-A-line PC for that specific A-line at `0x40803E08`).
D-cache probe at set=31 way=0 word=6 shows data=`0x40803E0A` — the
store landed at MEM[0x17FFF8] (= A7+0x14) **correctly**.

So at exc_count=0x9B, the dispatcher's frame-overwrite works
correctly.  The bug surfaces at exc_count=0x12C6 (= 4806, much
later in boot).

## Leading hypothesis (post-2026-05-21-instrumentation)

**Snoop-pipeline race** introduced by commit `e8e44d80` (2-cycle
dcache snoop response).  The dispatcher's `MOVE.L A2, 0x14(A7)`
store goes to D-cache.  If a CONCURRENT I-cache fill issues a
snoop_query for the same line at the same cycle as the LSU's store
to that line:

- Cycle N: D-cache LSU sees `MOVE.L A2` store.  Stages 1 of store
  pipeline.
- Cycle N: I-cache asserts snoop_query for same line address.
  D-cache combinationally reads tag/valid/dirty → "hit dirty".
- Cycle N+1: D-cache latches snoop_hit_dirty_q=1, snoop_idx_q's =
  the queried indices.  Meanwhile, the store COMMITS to BRAM.
- Cycle N+2: D-cache reads BRAM via the stage-2 mux using
  `snoop_idx_q's`.  BRAM has the POST-STORE data.

If the I-cache subsequently INSTALLS that post-store data as a code
line (= because it was a "dirty hit" per stage 1), it caches the
wrong data — but that affects I-cache only.

For the IRQ frame corruption, what would matter is: does the STORE
itself get committed correctly when a concurrent snoop query is in
flight?  The dcache.v's FSM should handle this since the store and
snoop_query don't share BRAM write ports.

But there could be a race in the EVICTION path.  If the snoop
query DURING a store triggers an UNexpected behavior (e.g., the
dirty bit get flipped or the line gets marked clean), a later
eviction could write back STALE data.

## Outstanding investigation queue

1. Sim directed test: `MOVE.L An, (d16, A7)` under D-cache pressure
   with concurrent I-cache fills (= force snoop queries to the same
   line).  See if any combination produces wrong store output.
2. Identify the specific instruction in the failing path that
   corrupts the IRQ frame.  Snapshot HW at exc_count=0x12C6 just
   BEFORE the wild jump, dcache-probe the IRQ frame line, see
   what's there vs what should be there.
3. Bisect: revert snoop pipelining only (= back to 1-cycle snoop,
   which fails timing).  If that bitstream boots cleanly (= no
   wild jump), the pipeline IS the cause.  If still wild,
   pipeline is innocent.
4. Audit dcache.v's store-vs-snoop concurrency more carefully —
   write specific tb-dcache scenarios that exercise
   store+snoop_query at the same cycle to the same line.

## 2026-05-21 — Stack push DROPPED at line 0x1FE080

Halted EXACTLY at exc_count=0x12C6 (just at wild-jump moment).
PC=`0x6db721d1` (= wild PC mid-execution), exc_pc=`0x4086abb2`
(= last latched A-line), A7=`0x001fe084`, D3=`-10` (= loop completed).

The wild jump is triggered by `RTS` at ROM `0x4086abc8` (= the loop
routine's exit RTS).  At RTS:

```
MEM[A7..A7+3] = ?  (= the return PC the loop routine should pop)
```

**D-cache state at the wild halt** — probing all set/way for tag
`0x7F8` (= address range 0x1FE000-0x1FE3FF, the supervisor stack):

| Cache line   | dcache state              | content     |
|--------------|---------------------------|-------------|
| `0x1FE000`   | set=0 way=3, valid=1, dirty=0 | `0xDDD800F7` (proper stack data) |
| `0x1FE020`   | set=1 way=3, valid=1, dirty=0 | `0x40809AE6` (proper) |
| `0x1FE040`   | set=2 way=1, valid=1, dirty=0 | `0x02EE00EC` (proper) |
| `0x1FE060`   | set=3 way=2, valid=1, dirty=0 | `0x000020F8` (proper — saved stack frame including a modified IRQ frame) |
| **`0x1FE080`** | **set=4 way=1, valid=1, dirty=0** | **`0x6DB6DB6D` (= memory-test bit pattern!)** |

The loop at `0x4086abae` pops 10 longwords from stack via
`MOVEA.L (A7)+, A1`.  Pre-loop A7 = `0x1FE05C`, post-loop A7 =
`0x1FE084` (= 0x1FE05C + 40).  The 10th pop reads from `0x1FE080..3`
which contains **`0x6DB6DB6D`** — the unmodified memory-test pattern.

Then `RTS` at `0x4086abc8` reads MEM[A7] = MEM[`0x1FE084..7`] =
`0xB6DB6DB6` (= word 1 of the bit-pattern line) and jumps there.
The CPU then fetches from that high address via MMU, lands on
multiple wild PCs, eventually halts at `0x6db721d1`.

**Root finding**: caller's push of the 10th argument to `0x1FE080`
**never landed in D-cache or DDR4**.  Lines 0x1FE000-0x1FE07F all
have proper stack content (= pushes succeeded); only the 10th slot
at `0x1FE080..3` is missing.

This is **a store-drop or store-misdirect bug** somewhere in our
LSU / D-cache path.  Specifically:

- Either the LSU's store didn't commit to D-cache for one specific
  push.
- Or the store went to a different cache line/address.
- Or the line was evicted with a corrupted writeback (= writeback
  to wrong DDR4 address).

The bug fires AT exc_count > ~0x12C6.  Earlier in boot (exc_count
~0x9B), the same code path executes correctly (verified via JTAG
break at 0x408099fa + dcache-probe showing the dispatcher's MOVE.L
lands at the correct address).

The transition from "works" to "fails" likely correlates with
cache utilization filling up (= 4-way × 32 sets fully populated by
later boot) and eviction pressure increasing.

## Top suspect (refined 2026-05-21)

**Eviction-write-back race during heavy cache traffic.**  When the
caller's push triggers a write-allocate miss that needs to evict
a DIRTY line, our `dcache.v` writeback path may have an off-by-one
or address-corruption bug under high concurrency.

The snoop pipeline (commit `e8e44d80`) added a second observer of
the cache state; if it interferes with the writeback path in any
way (= sequencer state shared, BRAM port shared), the writeback's
address or data could be wrong.

Next concrete action: write a tb-dcache scenario that:
1. Fills a line dirty.
2. Triggers eviction (= store to a different line at the same set,
   forcing PLRU to evict).
3. Concurrent snoop_query for the dirty line being evicted.
4. Verify the writeback hits the correct DDR4 address with correct
   data.

## 2026-05-21 — Confirmed: store-DROP at caller's FIRST push

JTAG halt-after at N=29.207M (= PRE-wild) reveals:

- Set=4 (= where line `0x1FE080` would live) has **NO line with
  tag=0x7F8** at all.  The 4 ways hold other addresses' lines.
- Set=3 way=2 has tag=0x7F8 valid=1 **dirty=1** holding **proper
  stack content** at line `0x1FE060`.

So **lines `0x1FE000`-`0x1FE07F` got the caller's pushes** (= via
write-allocate misses that brought them into cache dirty), but
**line `0x1FE080` was never installed by any push**.

Caller A7 layout (= deduced from post-loop A7=`0x1FE084`):
- Initial caller A7 = `0x1FE084`.
- Push 1 → `MEM[0x1FE080..0x1FE083]`  ← **THIS PUSH DROPPED**
- Push 2 → `MEM[0x1FE07C..0x1FE07F]`
- ... pushes 3-10 all landed correctly in lower lines
- JSR push → `MEM[0x1FE058..0x1FE05B]` = return PC
- Loop pops 10 values → A7 = `0x1FE084`
- RTS reads `MEM[0x1FE084]` (= word 1 of line `0x1FE080`) which was
  fetched from DDR4 fresh (bit-pattern) on the 10th pop's miss.

**The dropped push is the FIRST push of the caller's argument
sequence.**  All subsequent pushes to LOWER addresses succeeded.

## 2026-05-21 — Final narrowing: loop iter count + write-allocate miss

Re-read ROM at `0x4086abac`: `76fa` = **`MOVEQ #-6, D3`**.  So D3
starts at -6, not 0.  Loop body decrements D3 by 1 per iter, exits
when D3 < -9 (= D3 reaches -10).  Loop runs **4 iterations**, not 10.

Corrected stack arithmetic:
- Pre-loop A7 = `0x1FE074` (= post-loop A7 `0x1FE084` minus 16).
- Caller's 4 args occupy `MEM[0x1FE074..0x1FE083]`.
- After 4 pops, A7 = `0x1FE084`.
- RTS reads `MEM[0x1FE084]` as return PC (= **Mac OS convention:
  arg4 placed at the HIGHEST stack address, popped as the return
  PC via `MOVEM (A7)+, A1 × 4` + RTS**).

**The dropped store is the caller's push of arg4 to `MEM[0x1FE084]`**
(which would land in line `0x1FE080..9F`).  This push is a
**write-allocate MISS store** (= the line was not in cache before).

For a write-allocate miss store, dcache.v's path is:
1. `S_IDLE` → accept request, store to `lat_*` regs.
2. `S_LOOKUP` → tag check; miss → S_EVICT_RD (if victim dirty) else S_FILL_AR.
3. `S_EVICT_*` → write back dirty victim.
4. `S_FILL_AR` / `S_FILL_R` → fetch line from DDR4, write all 8 beats into BRAM.
5. `S_COMPLETE` → BRAM-merge the store on top of the freshly-filled
   word at `{lat_set, lat_woff}`, set dirty=1.

The HW evidence (= line valid=1, dirty=0, bit-pattern data at the
critical word slot) shows **steps 1-4 completed but step 5's merge
did not land**.  Possible specific failures:

- `S_COMPLETE` was entered but `lat_is_write` was somehow cleared.
- `S_COMPLETE` was skipped entirely (= different state transition).
- The merge write fired but to WRONG `ram_we_way` or `ram_waddr_all`.
- LSU dropped the store before it reached dcache (= it never got
  this far through the FSM).

## 2026-05-21 — dcache.v audit + storm tests conclude

Audited dcache.v `S_COMPLETE` state path carefully:
- State path is mandatory: `S_FILL_R` (beat 7, !fill_err) → `S_COMPLETE` → `S_IDLE`.
- `lat_is_write` only set at `S_IDLE` request; never cleared mid-FSM.
- `fill_err` only set if AXI R returns non-OK; for our HW evidence
  (valid=1), no fill error occurred.
- ram_we_all driven only in 4 places: default 0, S_HIT_RESP (write hit),
  S_FILL_R (fill beat), S_COMPLETE (merge).  No conflicting drivers.
- BRAM write template is standard byte-write SDP, used by all other
  tests that PASS.

Added 4 directed tb-dcache scenarios mirroring exact HW failure
(write-allocate miss with bit-pattern DDR4 backing, plus concurrent
snoop storm at every cycle of the FSM):

  - test_write_allocate_miss_with_bitpattern_backing  PASS
  - test_write_allocate_miss_with_concurrent_snoop    PASS
  - test_write_allocate_miss_under_full_storm         PASS

**34/34 tb-dcache scenarios PASS.**  The dcache.v's write-allocate
merge is robust to every concurrency pattern I can synthesize in
unit tb.

## 2026-05-21 — MAME state-replay reveals A7 DRIFT (not store-drop!)

Snapshotted MAME at PC=`0x4086abae` (= the loop entry).  Compared
arch state to our HW at the same PC.  Most registers MATCH exactly
(D2, A6, D7, etc.) but **A7 differs by 12 bytes**:

| Reg | MAME           | HW                  |
|-----|----------------|---------------------|
| D2  | 0x4080EDB0     | 0x4080EDB0 ✓        |
| A6  | 0x40804182     | 0x40804182 ✓        |
| A7  | **0x1FE068**   | **0x1FE074 (+12)**  |

**MAME's A7 is 12 bytes LOWER than our HW** at the same code point.
This means somewhere upstream our CPU pushed 3 fewer longwords OR
popped 3 more longwords than MAME did.

Consequence: the loop pops a stack window that's SHIFTED UP BY 12
BYTES.  The 4th pop lands in uninitialized memory (= the bit-pattern
region above the proper stack).  RTS then dispatches via the
bit-pattern bytes → wild PC.

**Previous "store-drop" characterization was WRONG.**  The bug is
not in the LSU/dcache store path.  The bug is **upstream stack
pointer arithmetic / push-pop balance**.

Also worth noting: MAME's DRAM at 0x001fe080..0x001fe08F ALSO
contains the bit pattern (= unchanged from memory-test phase).  So
both MAME and HW have the SAME bit pattern there.  The difference
is which addresses each CPU's A7 hits — MAME stays below 0x1FE080,
our HW reaches into the bit-pattern region.

## Refined hypothesis class

The 12-byte = 3-longword drift suggests:

1. **A-line dispatcher path A imbalance**: re-analysis of dispatcher
   path 0x408099c0 (taken for opcodes 0xA800-0xABFF) shows that the
   path pushes 16B (IRQ frame + 2 register pushes) but only pops 12B
   (2 register pops + RTS).  Net A7 drift = -4 bytes per invocation.
   3 invocations → -12 byte drift (= LOWER A7, matching MAME having
   LOWER A7 than us — wait, that's BACKWARDS).
   
   If MAME's A7 = 0x1FE068 and HW's = 0x1FE074, our HW has HIGHER
   A7 = FEWER bytes on stack.  So our HW pushed fewer or popped more.
   That's the OPPOSITE of path A's -4-per-call signature.
   
   If MAME took path A 3 times and we did NOT take it (or took less),
   MAME's A7 drifts lower.  3 extra path-A invocations on MAME side
   = -12 byte drift = matches.
   
   So MAY be: HW's path-A is somehow EARLY-EXITING or BCS-skipping,
   while MAME enters path A.  Same A-line opcode, different paths.

2. **RTE pop count mismatch**: if our RTE on some path pops 4 bytes
   instead of 8 (or vice versa), each occurrence shifts A7 by ±4.

3. **MOVEM mismatch**: if our MOVEM.L (A7)+, Dn-Dm pops fewer regs
   than the MOVEM.L Dn-Dm, -(A7) pushed, A7 drifts upward.

## 2026-05-21 — Bisect chain results: USP divergence

Compared MAME vs HW arch at progressively closer PCs.  Result chain:

| PC          | MAME SR / A7   | HW SR / A7     | Match? |
|-------------|----------------|----------------|--------|
| 0x40898AE2  | 0x2008 / 0x1FE00C | 0x2008 / 0x1FE00C | ✓ |
| 0x40887850  | 0x2704 / 0x17FF36 | 0x2700 / 0x17FF36 | ✓ |
| 0x4080DDE4  | 0x2704 / 0x17FFB6 | 0x2700 / 0x17FFB6 | ✓ |
| 0x408099F0  | 0x2709 / 0x17FFEC | 0x2700 / 0x17FFEC | ✓ |
| 0x40809A0A  | 0x2704 / 0x17FFE0 | 0x2700 / 0x17FFE0 | ✓ |
| 0x40803E0A  | 0x2704 / 0x17FFFC | 0x2700 / 0x17FFFC | ✓ |
| **0x4086AB9C** | **0x2000 / 0x1FE078** | **0x2004 / 0x1FE070** | **DIFFERS** |
| 0x4086ABAE  | 0x2008 / 0x1FE068 | 0x2004 / 0x1FE070 | DIFFERS |

Between PC `0x40803E0A` (supervisor, A7=SSP=0x17FFFC, matches MAME)
and PC `0x4086AB9C` (USER mode, A7=USP), **HW's USP is 8 bytes LOWER
than MAME's**.

The transition supervisor→user mode happens via RTE that pops SR
with S=0.  At that moment A7 switches from SSP (which matched) to
USP (which differs).  So USP was set differently sometime upstream.

Also notable: at `0x4086AB9C`, HW's D0=0x234E vs MAME's D0=0x234C
(+2).  Small register divergences accumulating.

## 2026-05-21 — Sim state-replay clears PEA implementation

Snapshotted MAME at `PC=0x4086AB9C` with EXACT correct USP=0x1FE078.
Loaded into Vfpga_top via `+state_replay=` and dumped arch at
0x4086ABAE / ABB0 / ABB4:

| PC                 | retired | A7         | Note |
|--------------------|---------|------------|------|
| 0x4086AB9C (start) | 0       | 0x1FE078   | seeded from MAME |
| 0x4086ABB4 (DBF)   | 1910    | **0x1FE068** | **-16 = 4 longwords pushed correctly** |

**Conclusion: PEA (d16, PC) is NOT broken.**  Given MAME's correct
pre-PEA state, our RTL's 4 PEAs decrement A7 correctly by 16 bytes,
matching MAME exactly.  The bug discovered earlier via HW arch
comparison ("PEAs don't decrement A7 on HW") was misinterpreted;
what we actually saw was that A7 was ALREADY WRONG before the 4
PEAs ran (HW already had A7=0x1FE070, not 0x1FE078).

The real bug is the upstream USP drift (-8) between supervisor PC
`0x40803E0A` (where A7=SSP matched MAME) and user-mode PC `0x4086AB9C`
(where A7=USP was already -8 from MAME).  USP gets stamped at the
moment of sup→user RTE; the divergence is in whatever set USP
between those points, or in the RTE itself.

## 2026-05-21 — FIRST sup→user transition found at PC=0x4080023A

Full MAME 30-emul-sec trace (~1.27 GB, mame_full.log) covers
boot from reset through to floppy poll.  In the window
0x408012A8 → 0x4086AB9C:

- Exactly TWO unique `MOVE #$2000, SR` instructions:
  - **0x4080023A** — fires ONCE.  This is the FIRST drop to user
    mode in the boot path.
  - 0x4080A2A8 — fires 40× (likely a per-IRQ dispatch re-entry).
- ZERO `MOVE.L An, USP` instructions in the window.
- Two unique RTE PCs: 0x40809B88 and 0x40809BBC (= IRQ handler
  returns, 50 RTE executions total).

So **USP never gets written via `MOVE.L An, USP` on the boot
path**.  USP gets set by either:
- A `MOVEA.L An, A7` AFTER SR.S goes to 0 (= writes user A7
  directly).  ROM 0x40800246 is `2e48 = MOVEA.L A0, A7`, which
  fires right after BSR cascade from 0x4080023A — strong candidate.
- An RTE that pops a frame with S=0 (= switches A7 from SSP to
  USP-bank, but the USP-bank value was set BEFORE).

ROM disassembly around 0x4080023A (the first sup→user point):
```
4080022e: 88f8           OR.B
40800230: 41f9 0004      LEA $0004xxxx, A0     | A0 = 0x00041800-ish?
40800234: bd40           EOR.W D6, D0
40800236: 4ebb 88f8      JSR (d8,PC,A0.l*..)   | indexed JSR
4080023a: 46fc 2000      MOVE #$2000, SR       | <<< DROP TO USER MODE
4080023e: 6100 09a0      BSR.W +0x9a0          | -> 0x40800BE0  (user mode call)
40800242: 6100 024c      BSR.W +0x24c          | -> 0x40800490
40800246: 2e48           MOVEA.L A0, A7        | <<< WRITES USER A7 (USP)
40800248: 90fc 2000      SUBA.W #$2000, A0
4080024c: 1f38 1efc      MOVE.B $1efc.W, -(A7)
```

If our HW's `MOVEA.L A0, A7` at 0x40800246 writes the WRONG
value (or A0 is wrong), USP lands -8 from MAME's.

## Next steps when shell environment is restored

1. Snap MAME at 0x4080023A (= just before SR=0x2000).
   Verify A0, A7 (= SSP), and what's pushed onto USP by the
   BSRs at 0x4080023E and 0x40800242 (= these run in USER mode
   with uninit USP unless USP was set upstream).
2. Snap MAME at 0x40800246 (= after BSRs, before MOVEA.L A0,A7).
3. Snap MAME at 0x40800248 (= after MOVEA.L A0,A7, A7 = A0).
4. Halt HW at the same 3 PCs.  Compare A0, A7.
5. The first PC where A0 or A7 diverges pinpoints the bug.

If A0 differs at 0x4080023A: bug is upstream (= what computed
A0).  If A0 matches but A7 differs after MOVEA.L A0,A7: the
MOVEA.L An,A7 instruction in user mode mishandles destination
(should write USP, may be writing SSP or wrong bank).

## ⚠️ CORRECTION: this is SSP/MSP drift, NOT USP

SR=0x2000 has bit 13 (S) = 1 → SUPERVISOR mode, NOT user mode.
The boot path in window 0x408012A8 → 0x4086AB9C is ENTIRELY
supervisor — there is NO sup→user transition.  The drift is
in the supervisor stack pointer (MSP via our PRF[20] / SSP).

Further HW investigation shows:
- Our impl uses 3 PRF slots: USP=PRF[19], SSP/MSP=PRF[20], ISP=PRF[21].
- Boot uses A7 = MSP throughout supervisor mode.
- IRQs push to ISP (separate from MSP) → mainline A7 (=MSP) unchanged across IRQ.
- MAME has same A7-banking behavior (A7=MSP during sup, A7=ISP during IRQ).

## OLD hypothesis class: USP-handling bug (SUPERSEDED)

The 8-byte USP difference suggests one of:

1. **USP initialization mismatch**: an early `MOVE.L #usp_init, USP`
   wrote different values on HW vs MAME.  Unlikely — both should
   process the same immediate identically.

2. **USP read/write bug**: `MOVE.L An, USP` or `MOVE.L USP, An` mishandles.
   These are privileged instructions; if our impl has a bug, USP
   could drift.

3. **User-mode push/pop mismatch**: user-mode code between the
   sup→user transition and `0x4086AB9C` executed different
   numbers of pushes vs pops between HW and MAME.  This requires
   actual user-mode code divergence, which would itself need a bug.

4. **Exception entry stashing wrong USP**: when entering an exception
   from user mode (= S goes 0→1), the CPU saves USP somewhere.  If
   our impl saves/restores incorrectly, USP drifts.

## Outstanding investigation queue (current)

1. **Bisect WHERE the USP drift happens.** Snapshot MAME at PCs
   BETWEEN `0x40803E0A` and `0x4086AB9C`, especially at the
   supervisor→user transition point (= some RTE).  Compare USP.

2. **Audit our impl's USP storage**: PRF tag for USP, how
   `MOVE.L An, USP` writes it, how exception entry saves it.

3. **Single-step on HW through the user-mode code path between
   0x40803E0A's RTS and 0x4086AB9C's PEA sequence** — find the
   first instruction whose retire produces an A7 value different
   from MAME.

The bug must be:

**(A) Outside dcache.v** — most likely in:
  - **LSU**: drops stores under specific conditions (= reorder buffer
    interaction, snoop kicker race).
  - **MMU walker**: returns wrong PTE under TLB pressure; store goes
    to a wrong physical address.
  - **axi_async_bridge** / async FIFO: CDC race between core_clk
    and fabric_clk100, drops a beat.
  - **axi_xbar**: multi-master arbitration drops a beat under load.
  - **axi_ddr4_mig_bridge** / ddr_ctrl / MIG IP: vendor IP edge case.

**(B) HW timing race** not reproducible at sim granularity (= a
specific clock-edge alignment that simulation doesn't model).

**(C) Pipeline interaction** added by commit `e8e44d80` (2-cycle snoop
response) that interacts with the rest of the system in a way
that's not local to dcache.v.

Concrete next-step fix options:

1. **Bisect by reverting** `e8e44d80` (snoop pipelining).  Re-synth.
   If HW boot is clean post-DAFB, the snoop pipelining caused the
   regression — and we need a different timing solution for the
   1-cycle snoop response.  ~1-2 h synth (timing uncertain).

2. **Add HW debug instrumentation**: write a small RTL probe that
   captures every LSU store request + every dcache S_COMPLETE merge
   to a ring buffer.  Re-synth.  Boot to wild-jump.  Inspect ring
   buffer for the specific cycle the store was dropped/misdirected.
   ~2-3 h work + 1-2 h synth.

3. **Hypothetical fix without root cause**: add a redundant safety
   net — periodic CPUSH or D-cache flush before high-stakes RTS.
   This is patch-not-fix.  Not recommended without root-cause.

4. **MAME state-replay deep dive**: snapshot MAME at exc_count just
   before failure (e.g., halt-after 29.205M equivalent in MAME), replay
   in sim with all peripheral state, IRQ injection at exact cycles.
   See if sim repros.  ~2-3 h.

## Related

- `docs/uarch_decisions.md §17` — SMC I-cache/D-cache snoop fix
  (commit `7e7eea0d`) and 2-cycle snoop pipelining (commit
  `e8e44d80`).
- `docs/BUG_icache_dcache_coherency.md` — the pre-SMC-fix wild jump
  (different bug, different address, same SYMPTOM class).
- `[[project-smc-fix-landed]]` memory.

## Repro on HW

Currently loaded bitstream on KU5P FPGA:
`/home/qwertyoruiop/m68k-ooo-bitstream-snoop-fix/build/vivado/fpga_top.bit`

```bash
/tmp/jcmd.sh "reset-and-halt-after 30000000"
sleep 6
/tmp/jcmd.sh "pc-trace 64"     # see the IRQ→0→wild PC cycle
/tmp/jcmd.sh "r 0x00002078"    # = 0x00002074 (corrupt)
/tmp/jcmd.sh "dump-mem 0x2040 32"   # see the full pattern
```

## 2026-05-21 — Halt-bisect on HW (m68k-fpga-halt-bisect skill)

Used JTAG REPL halt-after-N + advance to map the cascade onset:

| N retired | PC          | A7         | exc_count | Note |
|-----------|-------------|------------|-----------|------|
| 25,000,000 | 0x40898AE2  | 0x1FE00C  | 0x0FA4 (4004) | DBF loop, matches MAME |
| 26,000,000 | 0x40898AE2  | 0x1FE00C  | 0x0FAB (4011) | still in DBF |
| 26,000,000+1 (advance 1) | **0x40802842** | **0xFFFFFEBA** | **0x3955 (14677)** | DBL_FAULT |

**The cascade is INSTANTANEOUS** — within ONE retired instruction
boundary after the DBF loop exits, the CPU has fired ~10,650 extra
exceptions and underflowed A7 deep into peripheral space.

This means we cannot bisect the wild-jump cause via halt-after — the
cascade is too fast and DBL_FAULT halt fires before halt-after.

### BRA.W target compute verified correct

The post-DBF `BRA.W` at PC=0x40898AEC with disp=0xFF6E should target
PC=0x40898A5C (= `pd_pc + 2 + signed_disp` per 68k PRM § Bcc).  Our
RTL in `rtl/core/decode/decode_uop_assemble.v` ~line 16812 computes
exactly this:

```
imm = pd_pc + 32'd2 + {{16{ext1[15]}}, ext1};
```

So BRA.W is not the bug.  The target 0x40898A5C = MOVEM.L (A7)+,
mask=0x3CFC, which pops 10 longwords.  Cleanup code chain continues
from there, eventually reaches the wild-jump trigger.

### The fundamental issue

The wild-jump (PC=0x6Dxxx via JMP through corrupt low-RAM table)
is the *downstream consequence* of accumulated SSP/MSP drift across
~4000 IRQ exception entries during boot.  Each IRQ entry+RTE round
should be net A7-neutral; on HW some race accumulates a -4-byte
leak occasionally.  Sim directed tests don't repro because they
don't model realistic peripheral IRQ timing.

### Recommended next steps

1. **Build a sim test with realistic IRQ storm**: drive VBL IRQs at
   60Hz on a state-replay-loaded snapshot; track MSP across N
   iterations; look for monotonic drift.
2. **Audit RTE-pop / exception-entry interaction**: inspect what
   happens when an IRQ fires *during* the RTE pop sequence in
   exception.v — specifically the `is_rte_inject` / `S_INJECT_LOAD`
   state and `take_rte_finalize` gate.
3. **Compare MSP arch dump across N samples in sim vs same code
   path on HW**: state-replay + arch_dump_at_pc at IRQ entry/exit
   in sim, compare with HW PC trace ring at same events.


## 2026-05-22 — State-replay + IRQ injection finding

**Setup**: state-replay sim from MAME's snapshot at PC=0x40898AE2
(= last healthy point before cascade onset on HW), with `+irq_at_cycle=
1000000:1:100:5000` driving 100 level-1 (VBL autovec) IRQs at 5K-retire
intervals.  `arch_dump_at_pc=0x40809b60` captures A7 at every IRQ
entry.

**Result**: A7 (= live MSP slot, PRF[20]) = `0x001FDFE4` at **every
single** IRQ entry.  Across 100 IRQs with realistic ROM handler
running, **sim shows ZERO MSP drift**.

The unique-A7 value 0x1FDFE4 corresponds to post-IRQ-frame-push +
post-MOVEM-push-of-D0-D3/A0-A3:
  Initial MSP = 0x1FE00C (= MAME snapshot SP)
  IRQ frame push (8 bytes): 0x1FE004
  MOVEM.L D0-D3/A0-A3 push (32 bytes): 0x1FDFE4  ✓

So sim's IRQ entry sequence is mathematically correct, and stays
perfectly balanced across 100 iterations.

**Bonus finding**: State-replay sim WITHOUT IRQs reaches PC=0x4080A8E6
(= floppy poll loop, the EXPECTED healthy boot endpoint) at retired=5M.
With or without IRQs in sim, no wild jump.  The HW Q700 boot ends in
a wild jump cascade at PC=0x6Dxxxxxx; sim does not.

**Conclusion**: The HW bug is not reproducible in sim with the current
peripheral/IRQ model.  The leak event requires either:
1. **MMU TLB / dcache eviction race** that doesn't occur in sim's
   memory model.
2. **Specific peripheral interaction** (VIA1 timer rollover, DAFB
   scanout vs CPU access, SCSI/SCC state) that the testbench doesn't
   match HW.
3. **Pipeline-level race** that the bitstream synthesizes differently
   from Verilator's evaluation order.

Recommended hardware-side debugging path:
- Add HW-side instrumentation to record live MSP at every IRQ-entry
  retire (= via a dedicated debug-CSR slot updated by commit on
  is_exc_finalize).  Then dump after boot via JTAG and find first
  divergent IRQ.
- Compare MAME's full PC trace to HW's PC trace ring (with
  pc_align_diff) to find the first instruction where execution
  diverges.


## 2026-05-22 — Quick fix did NOT help; instrumentation reveals A-line trap drift

Applied quick fix to commit.v:2697 (`new_a7_w = rte_fire_a7_before
+ frame_size_w` instead of `active_sp_val + frame_size_w`) and baked
bitstream `b7bdd1d1`.  Sim regression PASS=688/689.

**HW result: same A7=0x1FE074 drift at PC=0x4086AB9C**.  The race I
fixed (RTE-finalize using live `active_sp_val` instead of latched
`rte_fire_a7_before`) is NOT the bug source.

**New finding from msp-trace instrumentation**: All exc_ring events
during boot are vec=0x0A (= A-line traps), NOT level-1 IRQs as
hypothesised.  The Q700 ROM uses A-line traps as the Mac OS Toolbox
dispatcher mechanism — every toolbox call is an A-line trap.

The Q700 A-line dispatcher exits via `ADDQ #4, SP; RTS` (NOT RTE) —
which is why no kind=RTE events appear in the ring.

Observed -4 Δmsp pattern between consecutive A-line traps occurs with
SR changes (X-flag drops, e.g. 0x2714 → 0x2700), suggesting an IRQ
fires between them.  The leak is per-IRQ, not per-A-line-dispatch.

Per-IRQ +Δmsp == 0 SHOULD hold (= entry pushes to ISP, RTE pops same).
But MSP drifts -4 across the IRQ.  Possibilities:
1. **Wrong-bank push**: IRQ entry sees `active_sp_val` = MSP instead
   of ISP, pushes to MSP not ISP.  Either prf_sp_slot_val read the
   wrong bank or a7_mirror_pending raced.
2. **RTE pop reads wrong frame size**: if our RTE detects format=2
   from a corrupted frame format word, pops 12 instead of 8.

Need: HW-side instrumentation to capture **IRQ-entry and RTE events
separately** — current ring shows EXC only.  Either drop the kind=2
A-line trap noise (= only record IRQs vec 24+), or extend the ring
to also fire on actual IRQ-RTE retires (= the RTE pattern that DOES
happen for IRQ handlers, just not for A-line dispatchers).


## 2026-05-22 — Option 2 (state-replay IRQ storm) + Option 3 (banking audit)

**Option 2 — state-replay + heavy IRQ injection in sim**: Loaded MAME
snapshot at PC=0x40898AE2 into Vfpga_top with `+irq_at_cycle=500000:
1:2000:2000` (2000 IRQs at 2000-retire gap).  Captured arch dump at
every IRQ entry (PC=0x40809B60).

Result: **819 IRQ entries, A7 = 0x001FDFE4 UNIFORM (single unique
value), MSP = 0x001FDFE4 UNIFORM**.  Zero drift across 800+ IRQs in
sim with realistic ROM handler + MMU + state-replayed peripherals.

This is the strongest sim evidence yet that the HW bug is NOT in our
RTL's IRQ-handling logic.  Identical RTL + identical starting state +
many IRQs = zero drift.  The bug requires a HW-only condition.

**Option 3 — banking audit**:

Audited commit.v + m68k_core_execute.vh + exception.v paths for
IRQ entry / RTE / banking races.  Findings:

1. `prf_sp_slot_val` selection rule (m68k_core_execute.vh:165-168):
   `S=0 → USP ; S=1, M=0 → SSP ; S=1, M=1 → ISP`.  Naming inversion
   vs PRM (our "SSP" = PRM-ISP; our "ISP" = PRM-MSP) — but
   functionally consistent throughout.
2. `a7_settle_in_flight` gates `take_irq_fire_q`, `take_exc`,
   `take_priv_exc` — IRQs cannot fire while A7 is mid-NBA settle.
3. `take_finalize` / `take_rte_finalize` do NOT gate on
   a7_settle — but they USE the latched fire-time value
   (`rte_fire_a7_before` post-fix), so no live `active_sp_val`
   read during finalize.
4. Frame format for vec 10 (A-line): format-0 (8 bytes push, 8 bytes
   pop via dispatcher ADDQ+RTS).  Vec 25 (autovec L1 IRQ): format-0
   (8 bytes push, 8 bytes pop via RTE).
5. Bank write on IRQ entry: writes SSP slot (= PRM-ISP, PRF[20]) for
   M=0 IRQ.  Correct.
6. M=1 IRQ dual-frame: handled separately with fire_msp_before.
   Q700 boot stays M=0 throughout, so dual-frame path doesn't
   trigger.

**No obvious bug found in the IRQ banking path that would cause
-4 MSP drift on M=0 boot.**

## Conclusion (provisional)

The HW Q700 boot bug is **not reproducible by directed sim or
state-replay-driven IRQ storms**.  The required conditions appear
HW-specific:
- Synth tool optimization differences (= Vivado timing not
  matching Verilator evaluation).
- MMU walk timing interactions with CPU pipeline.
- Cache eviction races (snoop pipeline depth differences).
- Peripheral IRQ scheduling not matching MAME (= different IRQ
  sub-cycle landing windows).

The MSP-drift instrumentation (commit 0357fb5f, `msp-trace` REPL
command) is now available in the bitstream for HW-side
investigation.  Need extended ring depth (256+) to capture
sufficient history through the boot, and IRQ-event filter to skip
A-line trap noise.


## 2026-05-22 — Bisection via break-pc on RTE / IRQ entry

Used JTAG REPL break-pc on 0x40809B88 (IRQ-handler RTE) and 0x40809B60
(IRQ-handler entry) with manual toggle-off / advance / toggle-on to
walk through IRQ events.  `continue` (= skip_once) does NOT advance
past the BP latch — appears to be the same auto-arm-loss bug noted in
the m68k-fpga-halt-bisect skill comment ("on the current bitstream the
auto-arm bit reads back as 0 by the time the host issues `continue`").

Manual toggle traversal hits at 0x40809B88:
| Hit | A7 | exc_count | Notes |
|-----|------|-----------|-------|
| 1 | 0x17FFF8 | 0x1022 | first boot-time IRQ, on ISP region |
| 2 | 0x1FDFD8 | 0x1045 | dispatcher running on MSP region |
| 3 | 0x1FDFD8 | 0x1061 | same A7 — handler reentered for second IRQ |
| 4 | 0x1FDFD8 | 0x107D | same |
| 5 | 0x1FDFD8 | 0x1099 | same |
| 6 | 0x1FE050 | 0x13BB | mainline progressed to different routine |
| 7 | (empty) | 0x2BE5 | DBL_FAULT zone reached |

Between hits 2 and 5, A7=0x1FDFD8 is UNCHANGED across 4 IRQs.  So
4 IRQ entries (= 4 frame pushes of 8 bytes each) all landed at the
SAME A7.  That means the corresponding 4 RTE pops all returned A7
to the same pre-IRQ value.  These 4 IRQs are BALANCED.

The drift must accumulate elsewhere — possibly in OTHER IRQ events
that fire at different mainline PCs (= different SR / cache state),
OR in the A-line dispatcher's ADDQ+RTS sequence when an IRQ happens
to land mid-dispatch.

The ring-of-32 msp-trace only captures the LAST 32 events.  At
exc_count ≈ 5000, the ring shows the tail (= mostly A-line traps,
since A-line:IRQ ratio is ~30:1 in this boot).  The leaking IRQ
events are likely outside the ring window.

## Action items for next bitstream rebake

1. **Filter ring**: change `exc_event_valid` in fpga_top_debug_ctrl.vh
   to skip vec=0x0A (A-line traps) — only record IRQs / sync exceptions.
2. **Extend ring depth** to 256 entries (= 8 KB at 32B per entry).
3. **Add per-RTE pop-bytes counter** to LSU and expose via JTAG so we
   can see if any RTE actually popped < 8 bytes.
4. **Add break-pc skip-once auto-arm verification** — currently the
   skill comment notes "auto-arm bit reads back as 0".  Need to
   re-investigate the slot tracking.


## 2026-05-22 — SIM_DDR_READ_DELAY infrastructure + dead end

Added `SIM_DDR_READ_DELAY` parameter to `rtl/sys/ddr_ctrl.v` (= injects
N cycles between AR fire and rvalid assertion) and Makefile knob
(`SIM_DDR_READ_DELAY=N make tb-fpga-top-rom`).  Tested N=50 and N=200.

Results in state-replay sim with `+irq_at_cycle` heavy injection:
| N (cycles) | IRQs done | Unique A7 values | Drift |
|-----------|---------------|------------------|-------|
| 0         | 800+          | 1 (0x1FDFE4)     | 0     |
| 50        | 500           | 1 (0x1FDFE4)     | 0     |
| 200       | 60+           | 1 (0x1FDFE4)     | 0     |

Even at 200-cycle read latency (= worst-case real-MIG bank-miss +
refresh), sim shows zero MSP drift.  Bug is NOT primarily caused
by DDR read latency in IRQ/RTE path.

Remaining hypotheses for the HW-only divergence:
- DDR WRITE latency / store-buffer drain timing (not yet modeled).
- Per-bank/row DRAM state (uniform delay doesn't mimic real refresh).
- Verilator evaluation-order vs synthesized gate-level timing.

Path forward: HW-side instrumentation (filter ring, LSU pop-bytes
counter) remains the most likely route to actually catch the leaking
event.


## 2026-05-22 — Deterministic cascade narrowed via MOVEM-pop bisection

Halted HW at each MOVEM.L (A7)+ inside the IRQ handler (PC=0x40809B84,
the instruction immediately before the RTE at 0x40809B88) and dumped
the IRQ frame about to be popped.  Used manual break-pc toggle with
+200-retire advance between hits.

| Hit | A7 | IRQ frame[0..3] | IRQ frame[4..7] | saved_PC | exc_count |
|-----|----|-----------------|-----------------|----------|-----------|
| 1   | 0x17FFD8 | 0x20004080 | 0x02420064 | 0x40800242 | 0x1022 |
| 2   | 0x1FDFE4 | 0x20084089 | 0x8AE20064 | 0x40898AE2 | 0x1042 |
| 3   | 0x1FDFE4 | 0x20084089 | 0x8AE20064 | 0x40898AE2 | 0x105A |
| 4   | 0x1FDFE4 | 0x20084089 | 0x8AE20064 | 0x40898AE2 | 0x1072 |
| 5   | 0x1FDFE4 | 0x20084089 | 0x8AE20064 | 0x40898AE2 | 0x108A |
| **6** | **0x1FE05C** | **0x20046DBC** | **0x06B10064** | **0x6DBC06B1 ← WILD** | **0x13A8** |
| 7+  | (corrupting) | — | — | — | 0x288C+ |

**Hits 1-5 are MAME-matching** (saved_PC in valid ROM range, A7 at
expected supervisor stack values).  Between hit 5 and hit 6 (= +798
exception events), the cascade starts.  By hit 6, A7 is at 0x1FE05C
and saved_PC is already 0x6DBC06B1 (= wild PC range).

**Deterministic**: `exc_count=0x13A8` at wild-jump start, `exc_count=
0x288C` at corruption peak, `exc_count=0x3A23` at final DBL_FAULT.
Same values across runs.

The bug fires AFTER the DBF loop exits (hit 5 = inside DBF, hit 6 =
post-DBF + already wild).  Between those two states, mainline code
exits DBF and continues into the A-line dispatcher chain, where the
MSP drift accumulates and eventually corrupts an IRQ frame's PC slot.

**Next session pickup**: narrow the window between exc_count 0x108A
and 0x13A8 by stepping at finer retired-count granularity.  The
first hit at 0x40809B84 with saved_PC != ROM-range pinpoints the
bad IRQ.

