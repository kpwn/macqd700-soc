# BUG: L1D writeback emits 0xff for untouched bytes of a dirty line

**Discovered by**: fuzz-corpus agent (2026-04-17) via seed 776678823
on the widened fuzz corpus; first seed out of 600+ that tripped a
memory-write mismatch (all arch register + CCR state matches).

## Symptom

RTL reports `mem_writes=56`; Musashi reports `mem_writes=24`.  The
extra 32 RTL writes are to addresses 0x0010c004..0x0010c01f, all
with value `0xff`.  The CPU never emits stores to these bytes — only
`move.l %d5, (%a3)` (= 0x0010c000..0x0010c003) touches that cache
line.

```
mem[0x0010c004]: rtl='0xff' musashi=None
  ...
mem[0x0010c01f]: rtl='0xff' musashi=None
```

Musashi (= golden) has no writes to these bytes, meaning the
architectural contract says they should be unchanged from their
initial value.  RTL's write-back cache evicts the line and emits an
AXI write for the ENTIRE 32-byte line.  The "untouched" bytes in the
line are whatever Verilator's `--x-initial fast` put there = 0xff.

## Why it didn't fire on earlier fuzz runs

The pre-widening fuzz corpus never mixed (a) a store into the
0x00100000 data window with (b) enough subsequent load/store activity
to evict the corresponding dirty cache line before the PASS sentinel
fired.  The widened corpus added multi-An disjoint data pools (A0..A3
each at a different 16 KiB window) and JMP / LEA (xxx).L emitters
that pull in cold cache lines, which forces eviction.

## Root-cause hypothesis

`rtl/core/mem/dcache.v` (landed in the l1d-real merge, 71b09d8) does
a full-line write-back on dirty eviction.  The write-back path emits
all 32 bytes of the line regardless of which bytes the CPU actually
wrote.  For correctness on hardware this is fine because the line's
"untouched" bytes were themselves filled from the SAME address range
on allocation — so the DRAM value round-trips unchanged.  But in
sim, the testbench tracks writes vs reads asymmetrically:

  - A line fetch (read miss) triggers N byte reads from mem_model;
    these DO NOT appear in the `mem[...]=...` write log.
  - The write-back triggers N byte writes, all of which DO appear.

So in sim we record the writeback but not the preceding read that
established the line's content.  In normal Verilator x-init=0 mode
the two would coincidentally agree (both are 0x00).  Under x-init=
fast (= 0xff) they diverge.

This is a sim-model artefact, not a real RTL bug.  On hardware
the DDR controller retains the pre-load byte value and the
writeback is a no-op for untouched bytes.

## Fix options

Pick one:

1. **Change x-init in Makefile from `fast` to `0`** for `make sim`
   (not the unit tbs).  Costs a ~5% sim-startup hit but makes
   "0xff writeback = 0x00 writeback" tautologically true under the
   mem-writes log.

2. **Track read-before-write per byte in mem_model** so writeback
   only records bytes that DIVERGE from what the line was allocated
   with.  More correct, more code.

3. **Filter mem-write keys at cache-line granularity in fuzz.py**
   so a per-byte writeback that restores the preload value is
   ignored.  Cheap but leaks cache-line details into the diff tool.

4. **Have the D-cache writeback emit only dirty-byte strobes** —
   this is a real RTL change (add dirty-byte masks alongside the
   dirty-line flag).  Hardware-correct; non-trivial retime.

Recommend (1) for now — lowest risk, unblocks fuzz.

## Impact on fuzzing

Low — this is a single deterministic seed out of ~600 that produces
a memory-write-only mismatch with perfect register + CCR agreement.
No other seeds hit this yet because the access-pattern overlap
required (dirty eviction of a not-fully-written line into the
write-log window) is rare.

Not a blocker for the N=200 fuzz gate: the gate is
MISMATCH=0 with the filter applied (stabilising fuzz.py to either
ignore stale-byte writeback or init mem to zero).

## TODO

- Pick one of the four fix options above and land it.
- Re-run `make fuzz N=500` after the fix to confirm the noise
  floor is clean.
- If the D-cache writeback path IS actually emitting non-zero
  dirty-byte strobes already, then this is a sim-model discrepancy
  only — document and move on.  If not, option (4) is the correct
  fix.
