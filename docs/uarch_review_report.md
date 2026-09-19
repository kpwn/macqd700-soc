# uArch Review Report - FPGA Timing, IPC, and Correctness Risk

Date: 2026-04-19  
Scope: read-through review of core docs and selected RTL for KU5P-class FPGA
bring-up, 200/250 MHz timing goals, and eventual >1 IPC.

## Executive Read

The current project sequencing is right: keep ROM/video first-light ahead of
broad IPC and Fmax work. The core is already too complex to treat timing or IPC
as a late cleanup task, but it is not yet structurally capable of >1 IPC.
Current benchmark docs still show peak IPC below 0.4
(`docs/bench_baseline.md:55-69`), and the RTL explains why: single dispatch,
single ROB allocation/retire, single integer issue, one ALU, and one blocking
LSU.

The highest-risk short-term issue is not width. It is precise exception/RTE
behavior around the current ROM frontier. Several exception shortcuts are
documented as phase simplifications, and the handoff says the live boot blocker
is already in exception/vector/cache-flush territory.

## Must Fix Before Hardware Smoke

### 1. Exception/RTE Shortcuts Can Break ROM Bring-Up

References:
- `rtl/core/commit.v:1275-1284`
- `rtl/core/commit.v:1305-1310`
- `rtl/core/m68k_core.v:128-137`

`commit.v` restores only `SR[15:5]` on RTE and intentionally leaves CCR stale.
It also ignores malformed RTE frame format errors. I-fetch MMU faults are also
ignored in the current I-side path. These shortcuts can pass directed tests yet
fail exactly where the ROM now is: bus-error/vector/RTE behavior after memory
probing.

Why it matters:
- Correctness: Mac ROM handlers rely on precise stacked state.
- Debug cost: stale CCR or incomplete RTE can present as a bogus low-PC,
  F-line, or endless exception loop.
- First-light risk: the current handoff already names vector/exception state as
  the frontier.

Low-effort/high-reward fixes:
- Add an RTE CCR-restore write path into CCR-RAT/CCR-PRF.
- Add a ROM-shaped bus-error -> handler -> RTE directed test.
- Add a format-error assertion or explicit vector-14 path rather than silent
  completion.

### 2. Full D-Cache Flush on Every Exception/RTE Is Too Expensive

References:
- `rtl/core/m68k_core.v:2086-2113`
- `rtl/core/m68k_core.v:2182-2194`
- `rtl/core/mem/dcache.v:1063-1163`

The exception gate holds `exc_fire`/`rte_fire` behind a D-cache flush because
the exception sequencer bypasses L1D. The comments document about 34 cycles for
clean cache and about 1200 cycles worst-case dirty. The D-cache walker actually
sweeps all sets and ways before releasing the exception.

Why it matters:
- Correctness: the gate is there for real coherency reasons, so removing it is
  unsafe.
- IPC: frequent ROM probe exceptions can become huge stalls.
- Bring-up: a long, legal flush can look like a deadlock without counters.

Low-effort/high-reward fixes:
- Add a dirty-any bitmap and bypass the full walker when no dirty lines exist.
- Count flush requests, dirty lines written, and cycles spent in the gate.
- Later, route exception stack/vector traffic through a coherent path or flush
  only the relevant vector/stack lines.

## Should Plan Before 200 MHz Closure

### 3. Structural Single-Wide Core Caps IPC Below Target

References:
- `rtl/core/m68k_core.v:796-832`
- `rtl/core/rename/rob.v:382-423`
- `rtl/core/rename/rob.v:473-478`
- `rtl/core/issue/iq_int.v:1-15`
- `rtl/core/m68k_core.v:1539-1574`

The core has one global dispatch handshake, one ROB tail allocation, one ROB
head retirement, one integer issue port, and one ALU. That makes >1 committed
uop/cycle impossible regardless of predictor or cache tuning.

Why it matters:
- IPC: second ALU alone will not help if dispatch, ROB, PRF ports, and CDBs stay
  one-wide.
- Fmax: widening all structures later will perturb many currently tuned paths.

Low-effort/high-reward fixes:
- Do not add a second ALU as an isolated change.
- Treat 2-wide as a bundle: RAT/CCR-RAT ports, ROB alloc/retire, PRF read/write
  ports, two-pick IQ, CDB expansion, and commit accounting.
- Keep benchmark and occupancy counters live now so the width bundle has data.

### 4. Flat Multi-Read PRF Will Not Scale Cleanly

References:
- `rtl/core/m68k_core.v:1218-1232`
- `rtl/core/m68k_core.v:1236-1252`
- `docs/bench_baseline.md:7-32`

The PRF is a flat 48 x 32 reg array with combinational reads for ALU, LSU,
committed A7, and MOVEC. The F1 registered-read stage is the right direction and
has small measured IPC impact, but a larger PRF or second ALU will multiply mux
and bypass complexity.

Why it matters:
- Fmax: 96 physical regs plus more read ports will deepen muxing and routing.
- IPC: a poorly staged PRF can erase the win from width.

Low-effort/high-reward fixes:
- Keep F1.
- Plan a replicated or banked registered-read PRF before 2-wide execution.
- Add counters for PRF free-list low-watermark and allocation stalls.

### 5. Predictor Correctness Filters Add Timing Logic

References:
- `rtl/core/fetch/bpu.v:18-22`
- `rtl/core/fetch/bpu.v:52-70`
- `rtl/core/rename/rob.v:272-302`
- `rtl/core/m68k_core.v:1578-1636`

BPU lookup is combinational. Execute-time training is filtered by a
combinational scan for older latent mispredictions in the ROB. This is clever
and useful for IPC, but it places predictor policy in timing-sensitive logic.

Why it matters:
- Fmax: the ROB scan and predictor lookup can become critical as the ROB grows.
- IPC: falling back to commit-only training costs loop warm-up, but may be a
  valid timing trade during 200 MHz closure.

Low-effort/high-reward fixes:
- Add counters for execute-trained, commit-trained, filter-blocked, and
  mispredict events.
- If timing slips, register the filter or temporarily disable execute training
  before touching precise-state logic.

### 6. D-Cache Metadata Uses Wide Parallel Clears

References:
- `rtl/core/mem/dcache.v:258-276`
- `rtl/core/mem/dcache.v:1146-1158`
- `rtl/core/fetch/icache.v:333-340`

Tag/valid/dirty and byte-write masks are held in flops for fast compare and
one-cycle clear behavior. For 4 KB caches this is acceptable, but the clear-all
paths and metadata fanout should not be allowed to grow casually.

Why it matters:
- Fmax: wide valid/dirty clears are fine at current size but become poor style
  if L1s grow.
- Area: byte-write mask flops are deliberate, but should remain a measurement,
  not a habit.

Low-effort/high-reward fixes:
- Add synthesis-visible metadata high-water and dirty-any reductions.
- Do not scale L1 size without changing invalidate/flush implementation.

## Later IPC Work

### 7. Memory Execution Is Correctness-First and IPC-Hostile

References:
- `rtl/core/mem/lsu.v:292-305`
- `rtl/core/mem/lsu.v:921-929`
- `rtl/core/mem/dcache.v:53-59`
- `rtl/core/issue/iq_mem.v:171-257`
- `rtl/core/issue/iq_mem.v:597-659`

The LSU accepts one operation when idle, stores wait in `S_ST_BUF` until commit,
and the D-cache upstream contract is one outstanding operation. `iq_mem` uses
O(N^2) dependency bitmaps and intentionally bubbles after inserts while a row is
built.

Why it matters:
- IPC: load/store overlap is limited, and store-heavy loops serialize.
- Fmax: `iq_mem` will become expensive if scaled from 8 to 16 entries.

Later fixes:
- Split AGU from cache access.
- Add a real store queue and store-to-load forwarding.
- Support nonblocking L1 misses only after precise exception/store ordering is
solid.

### 8. Decode Holds the Front End Through Cracks

References:
- `rtl/core/decode/decode.v:1-16`
- `rtl/core/decode/decode.v:185-200`
- `rtl/core/decode/decode.v:8741-8746`

Decode emits one uop per cycle and advances fetch only on the final phase of a
multi-uop instruction. This is simple and robust for ROM bring-up, but it wastes
front-end bandwidth and makes decode a monolith.

Why it matters:
- IPC: cracked instructions stall the fetch/decode window.
- Fmax: the decoder is already large and mostly combinational.

Later fixes:
- Add a crack/uop queue.
- Keep unsupported decode forms trapping loudly until first-light is stable.
- Defer broad decoder restructuring until after the ROM logo path works.

## Scary But Should Wait Until ROM First-Light

- RAS simplification: flush clears useful return history, but this is
  prediction quality, not current correctness (`rtl/core/m68k_core.v:342-351`).
- 2-wide decode/dispatch: necessary for >1 IPC, but too invasive for the
  first-light phase.
- Larger ROB/PRF/IQs: useful only after width work; premature now.
- Gshare/IBT predictor work: good later IPC work, but current BTB/RAS is enough
  for bring-up.
- L2 cache and nonblocking memory: not relevant until L1 correctness and ROM
  behavior are proven.

## Instrumentation To Add Now

Hardware-visible debug currently exposes committed/cycle basics, but several
performance counters are tied off in the FPGA debug path
(`rtl/fpga_top.v:1708-1714`), and BPU debug counters are disconnected in core
(`rtl/core/m68k_core.v:311-313`).

Add these counters before IPC/Fmax work resumes:

- Dispatch stall reason: ROB full, int IQ full, mem IQ full, int PRF empty, CCR
  PRF empty, MOVEC/RTE drain, flush.
- Occupancy high-water: ROB, int IQ, mem IQ, PRF free count, CCR free count.
- Branch: predictions, soft-hit redirects, hard redirects, execute training,
  commit training, filter-blocked updates, mispredict flushes.
- Fetch: I-cache hit/miss, victim soft-hit, line-cross bubbles, decode invalid
  cycles.
- Memory: D-cache hit/miss, dirty evictions, flush-all requests, dirty lines
  written, flush cycles, LSU state residency.
- Exception: vector counts, exception-gate cycles, RTE count, IRQ latency from
  `cpu_ipl` assertion to `exc_fire`.
- Commit: retired uops, branch retires, store retires, exception retires,
  cycles with complete ROB head but blocked by LSU/cache-maint/exception gate.

## Recommendation

For the current phase, spend effort on exception correctness and measurement,
not width. The best near-term hardware smoke investment is a small set of
exception/RTE correctness fixes plus counters that can explain whether the ROM
is executing, flushing, trapping, or genuinely wedged. Once sim first-light is
stable, restart Fmax work with control-fanout and flush-gate measurement before
opening the 2-wide IPC bundle.
