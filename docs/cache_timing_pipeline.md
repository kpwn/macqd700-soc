# Cache Timing Pipeline Notes

Date: 2026-04-21

This branch owns the cache-side slice of the RTL timing track.  It deliberately
does not duplicate `agent/timing-cache-vram-pipeline`, which already moved the
VRAM scanout read path to a two-cycle registered contract.

## Live Evidence

The current live Vivado place/physopt evidence repeatedly points at cache
metadata and RAM-output paths while pushing toward 200 MHz:

- `u_cpu/u_icache/tags[...]`
- `u_cpu/u_icache/data_ram[...]`
- `u_cpu/u_dcache/tags[...]`
- `u_cpu/u_dcache/valid[...]`
- `u_cpu/u_dcache/dirty[...]`
- `u_cpu/u_dcache/bwr[...]`

That evidence suggests two separate issues: metadata fanout from small
flop-backed arrays, and RAM-output/data-select paths that need explicit staging
where the consumer protocol can absorb it.

## I-Cache Change

`rtl/core/fetch/icache.v` now snapshots the indexed set's four tags, four valid
bits, and PLRU state when `S_IDLE` accepts a fetch and launches the per-way BRAM
read.  `S_LOOKUP` compares against that local snapshot instead of rereading the
tag/valid/PLRU arrays through `lat_set`.

This is intentionally narrow:

- upstream and downstream ports are unchanged;
- hit latency stays at the existing two cycles from `req` to `rvalid`;
- miss/fill sequencing is unchanged;
- same-cycle snoop plus fetch-start preserves the old array-based behavior by
  clearing the snapshotted valid bit when the snoop targets the requested line.

The expected timing effect is modest but low risk: it removes normal hit/miss
lookup dependence on the shared metadata arrays after the request-accept edge,
leaving only the snapshot fanout and the existing 4-way 128-bit data mux.

## D-Cache Change

`rtl/core/mem/dcache.v` now splits the normal hit path into request accept,
metadata lookup, and registered data response stages:

- `S_IDLE` accepts the request, snapshots the indexed tag/valid/dirty/PLRU
  metadata, and launches the per-way data RAM read.
- `S_LOOKUP` compares only the metadata snapshot and captures the hit way.
- `S_HIT_RESP` consumes the per-way `data_ram_out_q*` registered RAM outputs
  and returns `rvalid` or applies the hit-store byte merge.

The LSU-facing protocol is unchanged: `req` remains high until `rvalid` or
`bvalid`, and only one operation may be outstanding.  The intentional contract
change is hit latency: warmed load/store hits now complete three clocks after
`req` assertion instead of two.  `tb/tb_dcache.cpp` checks that hits do not
respond early, do not return stale registered-output data, preserve byte-lane
store-to-load visibility, and still behave correctly immediately after a
miss/refill boundary.  `tb/tb_lsu.cpp` stretches the cache BFM response to
cover BYTE sign-extension with the new delayed response shape.

Dirty evict, flush-all, and CPUSH LINE data reads also consume
`data_ram_out_q*`.  Dirty evict and flush-all add explicit wait states before
writeback beats; CPUSH LINE uses its lookup cycle as the first word's
registered-output wait and uses the wait state only after launching later word
reads.  This keeps every data RAM consumer behind the same registered-output
boundary.

## Targeted OOC Evidence

`synth/dcache_ooc.tcl` is a focused reproducibility hook for this timing slice.
It reads only `dcache.v`, creates a 5 ns OOC clock, runs
`synth_design -mode out_of_context`, and forces `general.maxThreads 4`.

On April 21, 2026, Vivado 2023.1 OOC synthesis for `xcku5p-ffvb676-2-i`
reported:

- utilization: 17,530 LUTs, 8,126 FFs, 4 RAMB18, 0 LUTRAM, 0 URAM;
- RAM mapping: `data_ram_w0_reg`..`data_ram_w3_reg` each inferred as
  `RAMB18E2`;
- RAM output packing: each RAMB reported `DOA_REG=1` and `DOB_REG=1`;
- no surviving cells matched `data_ram_out_q*`, consistent with Vivado
  absorbing or optimizing the explicit output stage into the RAM boundary;
- constrained 5 ns OOC timing: WNS +2.011 ns, TNS 0, no unconstrained internal
  endpoints.

The generated reports live under `build/dcache_ooc/` and are intentionally
build artifacts rather than committed timing data.

The remaining no-IPC-risk D-cache subproblem is maintenance/flush fanout:
dirty-any or valid-any summaries could avoid full metadata scans when the cache
is clean.  That should be a separate patch because it touches walker policy
rather than the normal hit path.
