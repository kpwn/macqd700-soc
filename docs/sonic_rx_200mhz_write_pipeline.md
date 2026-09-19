# SONIC RX packet-RAM write pipeline

The latest full-board 200 MHz recovery at `closure-soc` fails setup by
0.409 ns on `u_q700_sonic_rx/state_reg[0]` -> `frame_chunks` BRAM data.
The path crosses AXIS readiness, first-byte selection and byte insertion.
This worktree is isolated from that build and its uncommitted flow edits.

Register write data, address and validity as one transaction before the
packet RAM. Reset suppresses both pending and new writes. The existing
S_FILTER cycle drains the final write before S_DMA_PREP/S_DMA_LOAD read
chunk zero. No extra receive or DMA states are added; byte acceptance and
consecutive DMA request throughput remain unchanged.

Verification:

- Original RX test passes before and after, in both descriptor widths.
- Expanded test sweeps first payload lengths 60, 61, 62, 63, 64, 124, 125,
  128: single-chunk, exact boundaries, and split FCS. All 16 runs pass.
- Existing early-RDA, unswapped-CAM, missing-FCS and short-lookahead mutants
  are rejected after the RTL change.
- `check-synth-sources` and `check-storage-reset-pairing` pass.
- Routed OOC comparison at 5 ns, same part, same 1 ns input/output delays:

| Measurement | Baseline | Pipelined |
|---|---:|---:|
| Worst setup slack | +1.270 ns | +1.239 ns |
| Worst BRAM data-input slack | +1.270 ns | +3.939 ns |
| Worst hold slack | +0.042 ns | +0.042 ns |
| LUTs | 2274 | 2260 |
| Registers | 2063 | 2585 |
| BRAM tiles | 7.5 | 7.5 |

The intended BRAM-input cone is now a register-to-RAM path with zero logic
levels. Overall isolated WNS is slightly lower; this is not a claim that
the complete FPGA closes timing. Full-board routing at 200 MHz and boot
testing are still required. Ethernet remains enabled in SONIC DMA mode.

OOC script: `synth/sonic_rx_ooc.tcl`. Reports under
`build/sonic_rx_baseline_ooc` and `build/sonic_rx_pipeline_ooc`.
Test logs: `/tmp/codex-sonic-baseline.log`, `/tmp/codex-sonic-pipeline.log`,
`/tmp/codex-sonic-boundaries.log`.
