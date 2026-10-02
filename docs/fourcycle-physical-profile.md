# Four-cycle physical build profile

`make impl-fourcycle` is an opt-in, exact reproduction of the CPU and SoC feature
selection frozen in `/tmp/codex-physical-fourcycle-plan.json` on 2026-10-02.
It delegates to the existing split place/route flow. Normal `impl`, `synth`,
simulation, and bitstream defaults are unchanged. In particular, the normal
build keeps `CPU_AXI_DH=0` and `L2_DH_R_FALL_THROUGH=0`.

Use a clean CPU source tree carrying the four-cycle implementation. The source
override is passed through to both CPU generation and the Vivado input record:

```sh
make impl-fourcycle CPU040_SRC_DIR=/path/to/cpu-worktree \
  TARGET_FREQ_MHZ=100 CORE_CLK_HZ=100000000
make impl-fourcycle CPU040_SRC_DIR=/path/to/cpu-worktree \
  TARGET_FREQ_MHZ=200 CORE_CLK_HZ=200000000
```

Outputs default to `build/fourcycle-100mhz` and `build/fourcycle-200mhz` in this
SoC worktree. `FOURCYCLE_IMPL_DIR=/path/to/output` selects another output
directory. `TARGET_FREQ_MHZ` and `CORE_CLK_HZ` remain explicit and are both
recorded by the existing provenance flow; the target also accepts the Makefile's
derived clock value when only `TARGET_FREQ_MHZ` is supplied. The pinned feature
selection is in the `impl-fourcycle` recipe, including MSHR4, load ring8,
LS out-of-order issue, the four-cycle L1 path, the hot AR and R bypasses,
the real MIG checkpoint, and the board debug/peripheral settings. This profile
does not depend on the temporary JSON file at build time.

The four-cycle latency and correctness gates were validated on the frozen
experiment sources. The 200 MHz routed timing report and board boot/throughput
gates are pending. Keep this profile opt-in until those reports are reviewed.
The target does not program the FPGA.
