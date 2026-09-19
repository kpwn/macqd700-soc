# Synth @ fmax-simple HEAD, R1…R6 merged, 200 MHz

Ran Vivado 2023.1 synth_only on 2026-04-17 with the
`agent/fmax-simple` branch at commit c68510c (6 max_fanout retimes
merged as b3ad843).

Target: KU5P xcku5p-ffvb676-2-i, sysclk200 @ 5.000 ns, directive
PerformanceOptimized, -keep_equivalent_registers, -no_lc,
-flatten_hierarchy rebuilt.

BRAM_LOG2_BEATS=12 build-side hack (not committed) to work around
the SIM_MODEL 524K-LUT DDR explosion.

## Design Timing Summary

| Metric | Value |
|--------|------:|
| WNS | **-19.892 ns** |
| TNS | -9232.581 ns |
| Failing setup endpoints | 11456 / 392322 |
| WHS | -0.070 ns (3 endpoints) |

Fmax estimate: 1 / (5.0 + 19.892) ≈ **40 MHz**.

## Top violating path (all top-60 are variations of this)

- **Source**: `u_cpu/prf_reg[19][2]/C` — PRF read, phys 19 bit 2
- **Destination**: `u_cpu/u_alu/result_reg[29]/D` — ALU result reg bit 29
- **Data path delay**: 24.737 ns (logic 12.263 ns / route 12.474 ns)
- **Logic levels**: **209** (161 × CARRY8 + 31 × LUT5 + 8 × LUT6 + 5 × LUT3 + mixed)
- **Path**: `prf[19][2]` → `u_iq_int/divs_quot_full` /
  `divul_quot` → `u_iq_int/divs_rem_full` → `u_alu/divsl_rem3__[1020..1043+]`
  → `u_alu/result_reg[*]/D`

This is the **flattened DIVS.L / DIVU.L 32×32 combinational
divider** (introduced by task #62).  The entire 32-iteration
restoring / non-restoring divide is unrolled into a single
combinational LUT+CARRY8 chain — 209 levels between a PRF read
and the ALU result register.

## Why the pragma retimes can't fix this

R1-R6 reduce fanout on CDB, flush_en, and pd_valid nets.  Their
expected WNS recovery (~0.3-0.5 ns compound) targets paths in the
-0.5 to -2 ns range (per docs/fmax_analysis.md top-10).  At
WNS -19.89 ns, no pragma change at the fanout level could move
the top 60 paths, which are ALL through the combinational divide.

The retimes ARE visibly active in the physopt log
(iss_phys_src_b_reg replicas from iq_int.v:94-97).  They simply
cannot compete with 209 combinational levels.

## Next critical fix

Pipeline DIVS.L / DIVU.L.  This is a multi-cycle routing ticket —
out of scope for the "simple zero-IPC-risk" campaign but the #1
priority for the next Fmax campaign.  Expected WNS recovery: ~+18
ns, leaving ~-1 to -2 ns of residual from the ALU flag + other
paths which the R1-R6 retimes are positioned to address.

## Utilization (post-synth)

From `build/vivado/reports/utilization_synth.rpt` (summary):

- 524288-LUT DDR SIM_MODEL shrunk to 4096 RAM256X1D via the
  BRAM_LOG2_BEATS=12 workaround.
- DSP48E2: 4 instances (MULS 32×32 multiplier cascade).
- BUFGCE: 2 instances (clk, hdmi).

Full report (gitignored): `build/vivado/reports/timing_synth.rpt`.
