# BUG: I-cache fill reads stale DDR4 instead of live D-cache (SMC coherency gap)

**Status**: OPEN.  Identified on real HW 2026-05-20 via JTAG dcache-probe.
**Severity**: High — causes the Q700 ROM Sad Mac (vec 4 illegal at
PC=`0x001fe04c`) we've been chasing.  Boot is stuck behind this.

## Symptom

Q700 ROM boot on HW (bitstream HEAD = `91864935`) takes a wild jump
into the supervisor stack region at PC=`0x001fe04c`, decodes invalid
opcode `0x003F` (= `ORI.B EA mode 7/reg 7`, undefined), raises vec 4
illegal-instruction, drops into the Sad Mac display loop.

The wild jump itself is incidental.  The **real** bug surfaces when
the I-cache fetches at the wild-jump address: it reads STALE bytes
from DDR4 instead of the LIVE bytes the D-cache holds.

## Direct evidence (HW @ retire 29.2M, just before the wild jump)

JTAG `dcache-probe` of the suspicious stack line at `0x001fdfa0`
(D-cache set 29, way 0):

```
tag=0x07F7 valid=1 dirty=0  ← line IS in D-cache
data[0..7] (32 bytes)        ← live values the CPU sees
```

JTAG `r 0x001fe04c` (= DDR4 direct, bypassing D-cache):
```
0x003fac00                   ← stale leftover from earlier boot phase
```

D-cache view of mem[`0x001fe04c`] elsewhere in the run was `0x4080A80A`
(= a saved BSR return PC) — a valid ROM address.  These two values
disagree because the D-cache is write-back; dirty writes sit in the
cache until eviction.  DDR4 still holds whatever was there earlier in
boot, never updated by the runtime stack pushes.

When the wild jump fires:
1. CPU's PC := `0x001fe04c`.
2. I-cache miss for that line → I-cache fills directly from DDR4.
3. DDR4 returns `0x003F…` (stale) — NOT the live D-cache value.
4. Decoder sees `0x003F`, undefined ORI variant → vec 4.

If the I-cache had snooped the D-cache and gotten the live bytes
(e.g. `0x4080A80A` = `NEG.L D0` + A-line trap), execution would NOT
have hit an illegal opcode at this address.  The wild jump itself is
a separate bug, but it would have manifested differently / harmlessly.

## Why sim doesn't reproduce

Sim's behavioral memory model is coherent — every D-cache write is
visible to subsequent I-cache reads in zero time.  The wild-jump path
in sim fetches the LIVE bytes, decodes them as valid instructions,
and execution continues (eventually wedging in a different place,
or matching MAME for many more PCs).  Only on real HW with a
write-back D-cache and a directly-AXI-attached I-cache fill does the
divergence appear.

## RTL evidence

- `rtl/core/mem/dcache.v` — write-back; dirty bit per line.  Two
  places fire `snoop_valid` (S_HIT_RESP line 1065, S_COMPLETE line
  1199) on writes that land in cache.
- `rtl/core/fetch/icache.v` — `snoop_valid + snoop_addr` input port
  exists and **does** invalidate matching I-cache lines on D-side
  writes (lines 349-359 of icache.v).  This is correct for SMC where
  the I-cache had previously cached the code line.
- **Missing**: the I-cache fill path (S_FILL_REQ, lines 477-500)
  issues `mem_req → mem_rvalid` directly to AXI/DDR4 with **no
  consultation of the D-cache**.  If the address being filled has a
  dirty D-cache line, the I-cache gets the stale DDR4 view.

## Why the 2M-PC MAME lockstep diff didn't catch it

The 2M-PC lockstep window covers only ROM-resident PCs (`0x4080xxxx`
+ early boot vector dispatch).  ROM addresses are never written to,
so D-cache never holds dirty lines for them; I-cache fills from DDR4
always return correct data.  The bug only fires the first time PC
enters a RAM address that the runtime stack work has dirtied — way
past 2M PCs into boot.

## Fix

I-cache fill must serialize behind a "drain dirty data to DDR4"
operation on the D-cache for the target line.  D-cache already
implements this primitive at the maintenance port:

```
maint_is_inv = 0  → CPUSH semantics: writeback dirty, leave clean+valid
maint_scope  = 01 → LINE (single line by maint_addr[31:5])
```

**Proposed change (option 3 per the HW debug session):**

1. Add a second source on the D-cache maintenance port: an "I-cache
   read-snoop" req from `u_icache` to `u_dcache`, multiplexed with
   the existing commit-side CPUSH/CINV path.

2. Modify `icache.v` S_FILL_REQ entry: insert a new state
   `S_PRE_FILL_CPUSH` that fires `dc_pre_fill_req=1` with the line
   address; on `dc_pre_fill_done`, proceed to S_FILL_REQ as today.

3. After D-cache writeback completes, DDR4 holds the live data; the
   normal AXI fill returns coherent bytes.

4. Optimization (later): skip the CPUSH for known-read-only address
   ranges (the 1 MB ROM window at `0x40800000..0x408FFFFF`).  Saves
   the CPUSH latency for the 99 %+ of fetches that hit ROM.

## Test plan

1. Build a directed sim test: write known bytes to a RAM address via
   a D-cache hit, then JSR/JMP to that address.  Pre-fix: the
   instruction fetched is whatever was at that RAM address BEFORE
   the D-cache write (= stale).  Post-fix: instruction fetched is the
   D-cache-resident value.

2. Run the existing fuzz-deep gate to confirm no regression on
   user-mode integer ISA.

3. Run `make tb-fpga-top-rom +state_replay=/tmp/snap_post_delay` —
   sim should reach further into the boot now that the I-cache reads
   coherent data.  Whether it hits the wild jump itself or boots past
   is informative either way.

4. Build + program a new bitstream on HW.  Boot should not Sad Mac
   at the same vec-4 PC — either it progresses or fails differently.

## Related

- The 4-byte `Format $1 throwaway` over-push bug found while walking
  the M=0 IRQ entry stack discipline — self-consistent (push/pop
  match) but PRM-divergent.  Separate BUG_md needed.

- The MOVE.L (A7),A7 + RTS upstream sequence at ROM `0x408855e4`
  remains the trigger.  This bug fix doesn't prevent the wild jump,
  but it changes what the CPU executes once it lands — from
  guaranteed-vec-4 (stale 0x003F) to whatever the D-cache resident
  bytes decode as (likely valid m68k instructions).
