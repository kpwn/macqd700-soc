# debug-infra agent brief

Brief for the historical Phase 1.5 `debug-infra` sub-agent (the
orchestrator pattern referenced in earlier revisions has since been
retired in favour of the Task tool).
Foundation scope only — no FPGA, no `m68k_core.v` touch.

## Mission

Stand up the sim-side scaffolding for the PCIe / XDMA debug interface
specified in `docs/debug_pcie.md` so that by the time the board is free
(SD bringup done) the only remaining work is wiring and synthesis.

## In scope (new files only)

1. **`rtl/core/debug/debug_ctrl.v`** — synthesisable Verilog-2005 module
   implementing the AXI4-Lite slave shell and the **TIER 1** register
   block from `debug_pcie.md`:
   - DBG_VERSION, DBG_BUILD_ID, DBG_CONTROL, DBG_STATUS
   - DBG_PC, DBG_LAST_PC, DBG_REDIRECT_PC, DBG_REDIRECT_TRIGGER
   - DBG_IRQ_INJECT, DBG_EXC_VEC, DBG_EXC_PC, DBG_RESET_CAUSE
   - DBG_CYCLE_{LO,HI}, DBG_INST_{LO,HI}
   - DBG_MISPRED_COUNT, DBG_FLUSH_COUNT, DBG_EXC_COUNT
   - PC_TRACE ring (1024 × 32-bit) backed by BRAM, with PC_TRACE_HEAD
   Ports must match the `debug_ctrl.v` skeleton in `debug_pcie.md` verbatim
   (observability inputs, control outputs, AXI-Lite slave). Tie-off unused
   TIER 2/3 inputs with 0 for now — they will be filled in by `debug-obs`
   in phase 2.

2. **`tb/tb_debug_ctrl.cpp`** — Verilator unit tb that drives the AXI-Lite
   slave through enough traffic to verify:
   - DBG_VERSION read returns `0xDEB6_0003`
   - DBG_CONTROL bits round-trip (halt_req, step_pulse, etc.)
   - DBG_CYCLE increments once per clock when not halted
   - DBG_INST increments when `commit_event_valid` pulses
   - PC trace ring advances and wraps correctly
   - DBG_REDIRECT_TRIGGER emits the expected one-cycle
     `dbg_redirect_valid` pulse with the latched PC
   Use the existing `tb/tb_top.cpp` + `tb/models/mem_model.cpp` style.
   Add a Makefile target `make tb-debug` that builds and runs it; do NOT
   edit the main `sim` target.

3. **`tools/fpga_debug.py`** — the Python helper sketched in
   `debug_pcie.md` §"Python helper sketch", scoped to TIER 1 registers
   and the PC trace ring. Gate the mmap / /dev/xdma0 open so the
   module imports cleanly on a host with no FPGA (raise a clear error
   only when an `Fpga()` instance is constructed). Add docstring and
   a `__main__` block that prints VERSION / BUILD_ID / PC / cycles /
   insts / IPC — the "does this FPGA have my bitstream" smoke test.

## Explicitly out of scope

- **Do not edit `rtl/core/m68k_core.v`.** Observability ports get wired in
  a later phase-2 ticket (`debug-obs`), after ccr-rename and exception-path
  have landed their own `m68k_core.v` changes. Keep this module fully
  standalone for now — it only depends on its own ports.
- **Do not run Vivado, hw_server, or any JTAG tool.** The FPGA is held by
  the SD-bringup project. Synth / place / route is a separate `debug-xdma`
  agent that runs only after the board is released.
- **Do not wire into `mac_top.v`.** That comes with `debug-xdma`.
- Do not build out TIER 2 or TIER 3 registers yet. Leave the address
  decoder ready to grow (decode the full 20-bit BAR offset, return 0 with
  a clean BRESP/RRESP for unimplemented offsets).

## Reference materials

- `docs/debug_pcie.md` — full spec; treat as authoritative for register
  offsets, ring formats, port shapes.
- `rtl/core/commit.v`, `rtl/core/rob.v`, `rtl/core/fetch/bpu.v` —
  where the future observability signals come from (read for context,
  do NOT modify).
- `~/hdmi-bringup/` — reference pattern for how this host packages an
  FPGA build (XDC layout, Makefile shape). Note the `remote-synth`
  target ssh's to `10.200.0.11`; on this host that is a no-op — synth
  runs locally. Study it but don't invoke it.
- `~/prince/fpga/` (per `debug_pcie.md` §"Reference: prince integration
  pattern") — the verified XDMA + PCIe wiring for this board. Only
  relevant to the future `debug-xdma` agent.

## Done criteria

- `make tb-debug` passes.
- `make lint MODULE=debug_ctrl` clean.
- `python3 tools/fpga_debug.py --help` runs on a host with no FPGA.
- Commit on branch `agent/debug-infra` with a body that documents (a)
  the TIER 2 follow-up tickets, (b) any port-shape deviations from
  `debug_pcie.md`, (c) what the `debug-obs` agent will need to wire up
  per observability input.
- Do NOT merge to main; the orchestrator runs the full test suite and
  merges.

## Estimated scope

One session, ~400 lines of Verilog, ~200 lines of C++, ~150 lines of
Python. All additive; no regression risk.
