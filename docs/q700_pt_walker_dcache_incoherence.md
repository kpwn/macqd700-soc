# Q700 boot vec=2 root cause: MMU walker reads stale DDR, bypassing dcache

**Date**: 2026-05-05.  **HW symptom**: bus error vec=2 at PC=0x4080010E,
fault_addr=0x003FFFC0.  Reproduced on commit 0e46587c bitstream.

## TL;DR

Our MMU page-table walker has its own AXI master that goes directly to
DDR, **bypassing the L1 D-cache**.  The Q700 ROM (per MAME-traced execution)
enables the MMU *before* the OS does `CPUSHA both` to flush the dirty
PT lines.  In MAME this works because MAME has no functional dcache.
On our HW the walker reads stale DDR values for the PT entries while
the actual fresh PT entries sit dirty in our dcache — so the walker
faults on translate (or returns wrong PA).

## Evidence

### 1. PT corruption pattern observed on HW

JTAG-AXI read of the page table after vec=2 halt:

| Slot | PA          | HW value      | MAME-correct  | Note |
|------|-------------|---------------|---------------|------|
| ROOT[0]  | 0x003FFA00 | 0x003FF80A | 0x003FF80A | OK |
| PTR[0]   | 0x003FF800 | 0x003FF70A | 0x003FF70A | OK |
| PTR[1]   | 0x003FF804 | 0x003FF40B | 0x003FF60A | **WRONG** |
| PTR[2..14] |          | shifted to MAME ptr[i+2] |  | **WRONG** |
| PTR[15]  | 0x003FF83C | 0x00000018 | 0x003FE80A | **WRONG** (placeholder!) |
| PTR[16..127] |        | 0x00000018 | 0x00000018 | OK (placeholder) |
| PAGE[31] | 0x003FE87C | 0x003BE039 | 0x003FE039 | **WRONG** (1-bit) |

JTAG-AXI reads bypass the dcache, so they see what *DDR* currently
holds.  These wrong values are what the *walker* sees too — both go
through the same xbar to MIG to DDR backing.

### 2. State-replay sim with MAME's snap_pt_iter0 produces CORRECT PT

`build/fpga_top_rom/Vfpga_top +state_replay=snap_pt_iter0
+watch_pa=0x003FF800..0x003FF840` shows all 16 valid descriptors
(0x003FF70A..0x003FE80A) written correctly to DDR backing, and ptr[16]
= 0x18 (placeholder) starts loop2.  Sim sees these because the test-
bench's DDR backing is updated on every dcache write that egresses to
AXI — sim cannot reproduce the "dcache holds it but DDR is stale" race
because by the time the watch polls, the AXI write has completed.

The key difference between sim and HW: our sim's DDR model **completes
writes immediately**, so dcache writes are atomically visible.  Real
DDR4 has a bounded but real latency, and real dcache holds dirty lines
indefinitely until eviction or CPUSH.  In HW, the walker's read can
race ahead of the dirty-line writeback.

### 3. MAME-traced ROM CPUSH/MMU-enable ordering

`cpush both` only appears AT PC=0x40885032 in MAME's trace.  PFLUSHA
appears at PC=0x40880DF2 (MMU init region).  The MMU is enabled inside
the 0x40880Dxx code (PMOVE to TC), which is **before** the first CPUSHA.

This is correct per the 68040 architecture: the on-chip walker is
coherent with the on-chip data cache, so the OS doesn't need to flush
before enabling the MMU.  Our discrete walker doesn't have that
coherence.

## RTL evidence

`rtl/core/m68k_core_memory.vh:241-250` shows the walker has its own
AXI master (`dmmu_w_ar_addr` etc.) which is muxed against the dcache's
AXI on the shared D-AXI bus (`m68k_core_commit.vh:243-294` —
`walker_owns_bus = walker_axi_active`).  When the walker is active it
takes the bus *to DDR*, completely bypassing the dcache.

`rtl/core/mem/dcache.v:98+` confirms write-back semantics for normal
stores.  PT entry stores hit the dcache, mark the line dirty, and stay
there until eviction or CPUSH.

The existing mitigation at `m68k_core_memory.vh:222-237` only blocks
the walker during *pending dcache flushes* (for the PFLUSH/MOVEC-TC →
flush sequencer path) — it does NOT make the walker dcache-coherent
for normal data stores.

## Fix options

### Option A — walker reads via dcache (architecturally correct)

Re-route the walker's read path through the dcache as a (read-only,
no-allocate or with-allocate) request.  Hits return the dirty/clean
cache value; misses fetch from DDR.  Walker writebacks (U/M-bit
updates) similarly go through the dcache.

This matches the 68040 spec — on-chip walker is part of the data
memory pipeline.  Cost: a state machine in dcache to honor walker
requests, and arbitration with normal LSU traffic.

### Option B — dcache snoop on walker AXI

Keep the walker AXI direct, but have the dcache snoop the walker's
AR address.  If a dirty line matches, the dcache writes it back BEFORE
the walker's read completes (or supplies the data through a side
channel).

Cost: extra snoop port on dcache, race conditions to manage.

### Option C — implicit CPUSHA on MOVEC-TC

When the OS does `MOVEC dn, TC` (enable MMU), our commit logic could
synthesize a CPUSHA-DC + drain before completing the MOVEC.  This
guarantees DDR is consistent with dcache before the first walk.

Cost: surprising performance impact when OSes legitimately re-enable
the MMU after partial PT updates; may hide other coherence bugs.
Lowest risk for *boot* but anti-feature for general use.

### Option D (workaround) — force PT region cache-inhibit via DTT

Configure DTT0/DTT1 to mark the PT region as cache-inhibit.  Stores
to PT memory bypass the dcache, going directly to DDR.  Walker reads
see them.

Cost: requires the OS to set up DTT correctly, which we don't control
for stock Q700 ROM.  Not a real fix — depends on guest cooperation.

## Recommendation

Pursue **Option A** (walker via dcache).  It's the architecturally
correct fix and matches what real 68040 silicon does.  The dcache
already supports CPUSH/CINV maintenance and a read port; adding a
walker-request port is incremental.

## Verification path

After fix:
1. Boot Q700 ROM on HW; expect to advance past PC=0x4080010E with no
   vec=2 fault (the PT walk now sees the dirty cache values).
2. New directed test: `tb-mmu-walker-dcache-coherence` — write a PT
   entry via LSU (write-back), enable MMU, trigger a walk that reads
   the just-written PT — assert walker reads the cached value, not
   stale DDR.
3. Re-run the existing `tb-mmu-walker-boot` 18-scenario suite to
   confirm no regression.
