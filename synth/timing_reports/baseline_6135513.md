# Synth baseline @ 6135513 (2026-04-17)

First synth run of the m68k-ooo core (no full SoC yet; most peripheral
modules are stubs).  Target 200 MHz / 5.0 ns period.

## Timing (post-synth, pre-P&R)

- **WNS: -4.242 ns** (effective Fmax ≈ 108 MHz)
- TNS: -11,646 ns across 3,766 failing endpoints (of 18,285 total)
- WHS: -0.075 ns (minor hold violations)
- Verdict: **200 MHz not met at synth** — expect P&R to make it worse.
  Long way from target; Fmax push is phase 4 per gameplan.md.

## Utilization (KU5P, post-synth)

| Resource  | Used  | Total   | %    |
|-----------|-------|---------|------|
| CLB LUTs  | 16023 | 216960  | 7.4% |
| CLB FFs   | 9091  | 433920  | 2.1% |
| DSP48E2   | 8     | 1824    | 0.4% |
| BRAM      | 0     | 480     | 0.0% |
| URAM      | 0     | 64      | 0.0% |

BRAM=0 because no caches / register file / RAT are BRAM-inferred in the
current design — all built from distributed RAM / FFs.  Expect a real
step-up once L1I/L1D, ROB, and PRF move to BRAM.

## Context

The current mac_top.v is a thin wrapper that exposes core I/O to the
testbench.  Most peripheral modules (via*, scsi, scc, asc, glue, clk_rst)
are empty stubs.  mmu.v, dcache.v, icache.v are pass-through stubs.
So this baseline reflects the core pipeline + (CDB + iq_mem EA disambig
+ decoder-extend) as of 6135513 only.

## Next timing measurement

Re-run after:
- ras-integrate + bpu-phase2 land (expected modest change — +-0.5 ns)
- mac-top-integrator stitches real peripherals (expected WORSE by 1-2 ns
  from longer I/O paths + new clock domains)
- phase-4 Fmax push begins (expected to recover ~4 ns over a series
  of pipeline-retiming commits)
