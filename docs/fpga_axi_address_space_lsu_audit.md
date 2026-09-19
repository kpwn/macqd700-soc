# FPGA AXI Address Space and LSU Concurrency Audit

Date: 2026-04-22

## Address-Space Audit

Desired rule: every CPU-visible live device should be reachable through the
same AXI-visible fabric path that host/JTAG/XDMA uses.  CPU-local intercepts
are debug-host blind spots and should be removed or treated as temporary sim
scaffolding.

Current `fpga_top` map after this change:

| CPU address | AXI target | CPU access | Host/JTAG/XDMA access | Notes |
| --- | --- | --- | --- | --- |
| `0x0000_0000..0x0FFF_FFFF` | xbar S0 DDR | yes | yes | RAM window, DDR-flattened. |
| `0x4000_0000..0x40FF_FFFF` | xbar S0 DDR | yes | yes | ROM window, DDR-flattened; ROM writes still restricted by xbar master policy. |
| `0x5000_0000..0x50FF_FFFF` | xbar S1 `peripheral_bus` | yes | yes | Mac MMIO island behind core->pb async bridge; debug/provision are legacy service BARs bridged back to core. |
| `0x5010_0000..0x501F_FFFF` | xbar S2 DMA config | yes | yes | Carved out before S1. |
| `0x6000_0000..0x607F_FFFF` | xbar S0 DDR | yes | yes | Legacy framebuffer DDR alias. |
| `0xF900_0000..0xF90F_FFFF` | xbar S3 `vram` | yes | yes | VRAM pixel aperture; xbar strips `AXI_VRAM_BASE`. |
| `0xF980_0000..0xF980_03FF` | xbar S4 -> `axi_wide_to_axilite` -> `video` | yes | yes | DAFB is a first-class xbar slave; xbar strips `AXI_DAFB_BASE`. |

Known remaining mismatches:

- `mac_top.v` still has a local DAFB intercept for Verilator ROM harness
  shape.  This note and patch target production `fpga_top`; sim-top parity can
  be cleaned up separately once the test harness has an AXI host-peer model.
- `glue.v` remains an observational CPU decoder.  It classifies DAFB/VRAM and
  faults video gaps, but production routing is now owned by `axi_xbar` and
  `peripheral_bus`.
- Unowned `0xF9xx_xxxx` video gaps still DECERR/fault.  That is intentional:
  do not silently alias unknown video registers to DAFB or VRAM.

Low-risk patch implemented:

- Added `AXI_DAFB_BASE/SIZE` and xbar decode of `0xF980_0000..0xF980_03FF`
  onto S4.
- Bridged xbar S4 directly to the live DAFB AXI-Lite shim.
- Removed the `fpga_top` CPU-local DAFB write/read route, so CPU and
  host/JTAG/XDMA use the same slave semantics.

## LSU Concurrency Audit

Current contract:

- `iq_mem` has one `lsu_ready` input and issues at most one memory uop when
  the LSU is idle.
- `lsu.v` has one active operation slot.  Loads wait in `S_LD_WAIT`; stores
  wait in `S_ST_BUF` until `commit_store_en`, then issue to dcache.
- `dcache.v` accepts one upstream `req` and owns one downstream AXI read or
  write sequence at a time.  Cacheable misses can fill/evict over several AXI
  beats; non-cacheable/MMIO bypass is single-beat and blocking.
- `m68k_core.v` uses `lsu_busy` plus exception/cache-maintenance gating to
  preserve precise exceptions and stop the exception sequencer from stealing
  the data AXI bus while a data request is in flight.

Minimum safe multi-outstanding contract:

1. Only cacheable RAM/ROM/VRAM loads may become multi-outstanding.
2. Uncached, cache-inhibited, MMIO/device, maintenance, split unaligned, and
   store transactions remain single-outstanding and ordered.
3. A younger load may complete early only if its ROB tag is still live and not
   squashed.  Flush must poison all in-flight load slots before they can drive
   CDB/complete.
4. Stores keep commit-time issue.  No younger load may bypass an older store
   unless store address/byte-mask disambiguation and forwarding are implemented.
5. Exceptions remain precise: any faulting load reports by ROB tag, suppresses
   younger completions, and blocks new memory issue until the commit flush.
6. Dcache/AXI responses need IDs or slot tags.  The existing single `dc_req`
   and single `rvalid/bvalid` upstream interface is not enough.

Actionable module plan:

- `rtl/core/issue/iq_mem.v`: split `lsu_ready` into `lsu_ld_ready` and
  `lsu_st_ready`; keep stores gated by commit/order.  Do not issue loads past
  unresolved older stores in the first step.
- `rtl/core/mem/lsu.v`: add a small load-slot table containing ROB tag,
  phys dst, size, PA, cacheability, split state, squashed bit, and exception
  status.  Keep one store slot/FSM unchanged.
- `rtl/core/mem/dcache.v`: either expose an indexed load request/response
  interface or implement the slot table below dcache.  For a first bounded
  step, allow multiple clean cacheable read misses only after verifying no
  dirty eviction/writeback is active.
- `rtl/core/m68k_core.v`: extend CDB arbitration for back-to-back LSU load
  completions and make `lsu_busy` distinguish "MMIO/store/maintenance blocking"
  from "cacheable load slots outstanding".
- Tests: start with module-level LSU/dcache tests for two cacheable reads
  completing out of order by ID, flush poisoning one slot, and MMIO read
  blocking all later memory issue.  Then add a core test with a faulting older
  load and a younger cacheable load response arriving first.

Non-goals for the first LSU patch:

- Store-to-load forwarding.
- Speculative load past unresolved older stores.
- Multi-outstanding uncached/MMIO/device transactions.
- Dirty-victim eviction overlap with independent load fills.
