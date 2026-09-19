# Cache/VRAM RTL Timing Pipeline Notes

Date: 2026-04-21

This branch starts the RTL timing track without touching the live MIG/Vivado
worktree.  The only active RTL latency change here is the VRAM scanout read
pipeline; cache changes need a follow-up that can intentionally trade hit
latency against IPC.

## Live Timing Evidence

The live first-light place/physopt run reported WNS around -4.2 ns, improving
to roughly -4.139 ns during post-place physopt.  Repeated optimization attempts
hit cache and MMU structures:

- `u_cpu/u_dcache/tags[...]`
- `u_cpu/u_dcache/bwr[...]`
- `u_cpu/u_dcache/valid[...]`
- `u_cpu/u_dcache/dirty[...]`
- `u_cpu/u_icache/tags[...]`
- `u_cpu/u_icache/data_ram[...]`
- `u_cpu/u_dmmu/u_walker/...`

This points at two different classes of timing debt:

1. RAM output paths: BRAM/URAM output plus muxing should be explicitly
   pipelined where the consumer protocol can absorb latency.
2. Metadata fanout/compare/clear paths: cache tag, valid, dirty, and byte-write
   mask arrays are currently flop-based with broad parallel compare or clear
   behavior.  Physopt not being able to replicate some of these nets means RTL
   structure probably needs to change, not just attributes.

## VRAM Change Landed

`rtl/sys/vram.v` now makes the scanout streaming read path two cycles:

- `READ_LATENCY_B` is 2 for the XPM URAM port.
- `rd_valid` and the lane selector are delayed by two `rd_clk` cycles.
- The Verilator memory model matches the same two-cycle port-B behavior.
- The AXI/debug port-A path stays at the existing one-cycle RAM read contract.

Rationale: `fb_reader.v` already consumes `v_rd_data` only when `v_rd_valid`
is asserted and has a large fixed `RETURN_LATENCY` window before the scaler
consumes the response.  The extra VRAM cycle is therefore covered by existing
elastic buffering and does not change HDMI scanout behavior in the focused
tests.

## D-Cache Plan

`rtl/core/mem/dcache.v` already has a synchronous BRAM read stage for the four
per-way data RAMs.  Adding another registered output stage is mechanically
straightforward but changes every hit that consumes `hit_data`:

- load hit response gains one cycle,
- store hit merge/writeback gains one cycle unless forwarded from a separate
  store buffer,
- dirty eviction, CPUSH, and flush-all writeback reads gain one cycle per
  written word unless staged with prefetch overlap.

Because the live timing also names `tags`, `valid`, `dirty`, and `bwr`, the
next D-cache pass should not only add a BRAM output register.  It should split
the hit path into explicit stages:

1. request latch + data RAM address launch,
2. tag compare / metadata snapshot,
3. registered data select and response or store merge.

That costs +1 cycle on load-hit latency unless paired with a store buffer /
load-use forwarding plan.  For no-IPC-loss work, the safer first target is the
maintenance/writeback side: add a dirty-any bitmap, avoid full-cache scans when
clean, and pipeline `bwr`/dirty readback for CPUSH/flush walkers.  That reduces
metadata fanout pressure without touching normal load hits.

## I-Cache Needs A Sibling Pass

`rtl/core/fetch/icache.v` deserves the same treatment.  It has flop tags/valid,
parallel hit compare, same-cycle snoop/maintenance clears, and a 4-way
128-bit BRAM data mux.  The live `u_icache/tags[...]` and
`u_icache/data_ram[...]` evidence is consistent with that structure.

The I-cache retime should be separate from this branch because it affects the
front-end fetch contract:

- add a registered tag/valid snapshot or split snoop/maintenance clears out of
  the fetch hit path,
- add an optional data-output stage after the per-way BRAM read,
- update `if_stage.v` hit-latency assumptions and `tb_icache`/`tb_if_stage`
  together.

The I-cache has lower IPC risk than D-cache because `if_stage.v` already uses a
req/valid handshake and has prefetch/soft-hit machinery, but it still changes
branch/line-cross timing and should be verified as a front-end change.

## MMU Walker Note

The `u_dmmu/u_walker` evidence is real but out of this branch's write scope.
Treat it as a follow-up timing track for descriptor-read issue/response
staging.  It should not be mixed with cache and VRAM changes unless a timing
report shows a shared cache/MMU metadata path.
