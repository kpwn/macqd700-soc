# Handoff: L2C + VRAM-in-DDR cutover (2026-07-24)

SoC-side work landed on `split/ku5p-base-board`. Contracts: `docs/l2c_spec.md`,
`docs/ddr4_mig_bridge_contract.md`. Task/review history: the session ledger
under the SoC session scratchpad (`briefs/t*.md`, `t*-report.md`).

## 1. New memory hierarchy — live in RTL, timing-closed in a bitstream

DDR path is now:

```
xbar S0 ─► l2c (2 MB, 8-way, 64 B lines — SoC point of coherency; `L2C_ENABLE`)
        ─► axi_vram_priority_mux3 (scanout strict-priority / l2c / S3 VRAM lane)
        ─► axi_async_bridge (+ axi_bridge_stale_sink reset-debt drain)
        ─► axi_ddr4_mig_bridge (8R/4W outstanding, cut-through, RAW on real B)
        ─► MIG ─► DDR4
```

- VRAM moved to a DDR carveout (`VRAM_IN_DDR`): the 0xF9000000 aperture
  translates to top-of-DRAM; S3 stays a real xbar slave (decode-level
  never-allocate — VRAM traffic never reaches the L2); scanout reads DDR via
  a dedicated strict-priority lane. The 2 MB URAM array is elided; the L2's
  64 URAM288 take its place.
- Both gates are OFF by default: default sim builds and old bitstreams are
  byte-identical (preprocessor-diff-proven per touched file).

## 2. Bitstreams ready to load

- `/home/qwertyoruiop/macqd700-soc/build/vivado/fpga_top.bit` (+ `.ltx`):
  **CPU=m68k + L2C_ENABLE + VRAM_IN_DDR, 100 MHz — WNS +0.012 ns, TNS 0,
  zero failing setup/hold endpoints (344k endpoints).** Built with
  `ALLOW_STALE_CPU=1` against `cpu/` exactly as currently checked out
  (INCLUDING uncommitted decode/alu edits — see §5).
- `/home/qwertyoruiop/macqd700-soc/build/vivado/fpga_top_stub_l2c_vramddr.bit`
  (+ `.ltx`): same SoC with the CPU stub — JTAG-only smoke of the
  L2→CDC→DDR path, VRAM carveout, and scanout without booting a Mac.

## 3. First-boot expectations

- 1bpp screen is **BLACK until the ROM programs the RAMDAC** — the 256-color
  CLUT scanout (MAME-faithful `dafb.cpp` per-depth indexing) replaced the old
  hardcoded palette; the old black-on-white-at-power-on was the deviation.
  Not a hang.
- 256-color mode works end-to-end (Monitors will offer it via normal sense +
  VRAM-size discovery; no signaling needed). Thousands/Millions are
  advertised by the ROM (2 MB + color sense) but direct-color scanout is NOT
  implemented — picking them renders garbage. Known, filed.

## 4. Reset semantics

- L2 fully clears on reset (walking tag clear ~4k cycles, AXI accepts held;
  xbar S0 watchdog at 2^18 dwarfs it). RAM after any reset = only what
  reached DRAM (dirty L2 lines are intentionally lost).
- JTAG core-only resets are safe for READ traffic: `axi_bridge_stale_sink`
  counts in-flight R/B debt m-side and drains it across s-side resets (this
  fixed a latent mainline stale-data bug that predated the cutover).
- OPEN GAP: a core-only reset landing mid-WRITE-burst can permanently shift
  AW/W pairing at the MIG (silent write corruption). Avoid `reset hold`
  under heavy write traffic until the W-filler-completion follow-up lands
  (design: wstrb=0 padding, same idiom as the MIG bridge's).

## 5. ACTION REQUIRED: cpu/ vs ~/m68k-ooo divergence

`cpu/` (this repo's submodule checkout) and `~/m68k-ooo` (CPU dev tree)
differ in 7 RTL files: `decode.v`, `decode_uop_assemble.v`,
`decode_semantics.v`, `decode_1111.vh`, `debug_ctrl.v`,
`m68k_core_execute.vh` (+1), and `cpu/` carries uncommitted changes.
The loaded m68k bitstream embodies the `cpu/` dirty state. Please commit/
reconcile so `make check-cpu-sync` passes and builds are reproducible.

## 6. Build-system changes

- Root `make test` = platform tb suite (48 targets, PASS/XFAIL/FAIL summary);
  `make lint` = `lint-fpga-top`. Core-track targets (`tb-lsu`, `tb-alu`, …)
  are loud stubs pointing at `make -C cpu <target>`.
- The pre-impl `fuzz-deep` gate is RETIRED (owner decision 2026-07-24).
  Note: root `make fuzz-deep` still depends on the retired `sim` target.
- After ANY topology reshape, clear the incremental reference
  (`INCREMENTAL_REF_DCP= make impl`) — the placer hard-fails against a
  stale pre-reshape routed checkpoint.
- Vivado gotcha: never START a prose comment with the token `translate_off`
  — Vivado's pragma matcher treats it as a real pragma and comments out the
  rest of the file (Verilator is lenient, sim won't catch it).
- Enable the cutover in synth via env: `L2C_ENABLE=1 VRAM_IN_DDR=1 make
  synth|impl` (hook in `synth/vivado.tcl` ~:966; all T14/T16 RTL is in the
  read_verilog list as of `fada4b4`).
- Known tooling issue (filed): the T6 gray-pointer `set_bus_skew` proc
  skips every async_fifo instance ("source clock unresolved") even at impl —
  the extra skew bounds are NOT applied; functional CDC still covered by
  async clock groups. Fix the proc's clock resolution.

## 7. Verification surface you can lean on

`tb-l2c` (22), `tb-l2c-chain` (4), `tb-vram-ddr-chain` (11 × both builds),
`tb-fb-reader-ddr-chain` (3), `tb-pb-scsi` (6 — real peripheral_bus+scsi
integration, the seam every old scsi tb bypassed), `tb-axi-ddr4-mig-bridge`
(41), `tb-axi-async-bridge` (13), `tb-axi-xbar` (196), plus hardened
tb-via1/2, tb-scc, tb-scsi suites. Wave-1 fixes: SCSI DRQ level-held
handshake, VIA timer write-vs-tick races, SCC RX push/pop netting, xbar
burst legalization + per-slave watchdog/poison/`slv_flush`. Watchdog
layering: xbar per-slave bound is 2^18 EXCEPT S1 at 2^27, which must stay
above peripheral_bus's 2^24 SD-tolerant ack watchdog — preserve that
ordering if you touch either.

## 8. Process rules binding all agents in this repo

- NEVER `git stash` — refs/stash is repo-global across all worktrees
  (a stash collision destroyed concurrent work once already).
- Verify `git symbolic-ref HEAD` before branch operations in the main
  checkout — stray agent cwds have detached/moved it twice.
- Reviews are adversarial: fail-before/pass-after evidence required for
  every fix; new modules get randomized scoreboards with backpressure and
  same-ID streams (weak golden models hide real bugs — proven repeatedly).
