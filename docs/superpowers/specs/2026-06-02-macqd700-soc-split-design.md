# Design: split the Mac Quadra 700 platform into `macqd700-soc`

**Date:** 2026-06-02
**Status:** approved (brainstorming) — pending spec review
**Goal:** carve the platform out of `m68k-ooo` into a new repo `~/macqd700-soc/`
so a *different* CPU can be integrated, and structure it so an FPGA
board-support layer can later split off for other devices.

---

## 1. Motivation

`m68k-ooo` currently holds three concerns in one tree: the OoO 68k **CPU**
(`rtl/core/*`), the **Quadra 700 machine** (`rtl/mac/*` + Mac glue), and the
**KU5P FPGA board support** (DDR4 MIG, clocks, HDMI PHY, JTAG-AXI/VIO/ILA,
SD/boot, constraints). A different CPU will be integrated with the machine, and
other FPGA devices will reuse the board support. Keeping all three in one repo
couples them; splitting along clean interfaces lets each evolve independently.

Verified boundary fact: **`rtl/core/*` instantiates no `rtl/mac`/`rtl/sys`
module** — the CPU is already self-contained. The only seam is `m68k_core`'s
port surface (`if_*`, `daxi_*`, `cpu_ipl_*`, and a large CPU-specific `dbg_*`
debug surface consumed by `fpga_top`'s `debug_ctrl`).

## 2. Target three-layer architecture

```
┌─────────────────────────────────────────────────────────────┐
│ CPU repo  (m68k-ooo, or a future core)                       │
│   rtl/core/* + m68k_axi_wrapper  ──presents──► CPU socket    │
└─────────────────────────────────────────────────────────────┘
                         │ AXI socket (dual master, IRQ, rst/halt)
┌─────────────────────────────────────────────────────────────┐
│ macqd700-soc  (THIS split)                                   │
│   rtl/mac/   = Quadra 700 machine (board-agnostic)           │
│   rtl/soc/   = integration: fpga_top, cpu_socket, fabric     │
│   rtl/board/ = KU5P BSP (future 3rd-repo split target)       │
└─────────────────────────────────────────────────────────────┘
                         │ BSP contract (AXI mem, framebuffer, clk/rst, irq)
                  KU5P board  (rtl/board/* — later its own repo)
```

This split delivers layers **CPU** and **machine+board (macqd700-soc)**. Inside
`macqd700-soc`, the machine (`rtl/mac`) and board (`rtl/board`) are organized
into separate directories *now*, with a documented seam, so the FPGA-BSP repo
split later is a near-trivial directory move + git filter-repo.

## 3. The CPU socket (CPU ↔ SoC contract)

New `rtl/soc/cpu_socket.vh` (interface macros/params) + `rtl/soc/cpu_stub.v`
(black-box). Contract:

- **`axi_i_*`** — AXI instruction master (read-only), `AXI_DW` ∈ {128, 256}
  (parameter), `AXI_AW`=32, `AXI_IW` param.
- **`axi_d_*`** — AXI data master (read/write), same width.
- **`irq_ipl[2:0]`** in, **`ipl_ack`** out (today's `cpu_ipl_ext`/`cpu_ipl_ack`).
- **`rst`**, **`halt_req`**, **`halt_ack`** — minimal reset/halt control.

The SoC always instantiates the socket. With no CPU submodule present it binds
`cpu_stub.v` so `fpga_top` elaborates and platform-only sim/synth/lint run
standalone. The real CPU is a **git submodule** at `cpu/`, pinned to a commit,
that provides a module matching the socket.

**m68k adapter (lives in the CPU repo):** `m68k_axi_wrapper` wraps `m68k_core`
to present the socket — folds `if_to_axi.v` (128-bit fetch → AXI instruction
master) and a narrow→wide bridge on `daxi_*` (32 → `AXI_DW`) reusing the
existing `axi_narrow_to_wide`. A natively-wide CPU drives the socket directly.

**Debug:** the rich CPU-specific debug (`debug_ctrl` + the full ILA probe map +
arch-capture + the `dbg_*` ROB/PRF/A7 surface) **moves into the CPU repo**,
wired through `m68k_axi_wrapper`. The socket carries only a **minimal generic
debug subset**: the JTAG-AXI window, `halt_req`/`halt_ack`, and a small
PC/retire tap. A future CPU brings its own rich debug. (Rationale: the current
`debug_ctrl`/ILA know ROB tags, PRF phys, A7 banking — not portable.)

## 4. Machine ↔ board BSP contract (intra-`macqd700-soc` seam)

`rtl/board/` provides, and `rtl/mac/` (+ `rtl/soc`) consume:
- **Memory:** an AXI slave backed by DDR4 (MIG) — the machine's RAM.
- **Framebuffer/scanout:** the DAFB *logic* (`rtl/mac`) drives a framebuffer
  read + timing-parameter interface; the board's HDMI PHY (MMCM, VTG, scaler,
  linebuf, I2C init) turns it into HDMI.
- **Audio:** ASC (`rtl/mac`) emits a sample stream; board PHY (`i2s`/`pwm`/
  HDMI-audio) drives pins.
- **Clocks/reset:** board MMCM/`clk_rst`/`reset_debounce` produce core/pb clocks
  + synchronized resets.
- **Boot/storage:** board SD controller + `boot_fsm`/`sd_jtag_writer` load the
  ROM/disk image into DDR before CPU release.
- **Debug transport:** board JTAG-AXI + VIO/ILA shell.

The seam is documented in `docs/bsp_contract.md` in the new repo.

## 5. File sorting (principle + initial assignment)

**Principle:** `board` = physical/KU5P-specific or vendor (reusable on other
boards); `mac` = Quadra-700 device logic (board-agnostic); `soc` = integration
+ generic fabric glue.

- **`rtl/mac/` (machine):** via1, via2, scsi, scc, asc, rtc, adb_* + pic16c5x,
  iwm_stub, orwell_stub, q700_eth_sonic, glue, irq_agg, video.v (DAFB
  scanner/register model).
- **`rtl/board/` (KU5P BSP):** ddr_ctrl, axi_ddr4_mig_bridge, sim_mig_backend,
  clk_rst, reset_debounce, sd_ctrl, sd_spi, sd_spi_mux, sd_jtag_writer, vram
  (URAM), async_fifo, pulse_cdc, uart_byte_bridge, audio_i2s, audio_pwm,
  audio_hdmi_bridge, the HDMI-PHY video files (mmcm_hdmi, vtg, scaler,
  linebuf_scanout, i2c_init, scanout_placement_sync, vram_smoke, video_top),
  vendor/* (MIG stub), synth/* (vivado.tcl, ku5p.xdc, debug_ila.tcl).
- **`rtl/soc/` (integration + fabric):** fpga_top.v + fpga_top_*.vh, mac_top.v,
  cpu_socket.vh, cpu_stub.v, axi_xbar, axi_async_bridge, axil_async_bridge,
  axi_narrow_to_wide, axi_n64_to_wide, axi_wide_to_axilite, peripheral_bus,
  dma_ctrl, boot_fsm.
- **CPU-side (`if_to_axi`)** moves to `m68k_axi_wrapper` in the CPU repo.

Ambiguous cases (vram, boot_fsm, audio bridges) are resolved during planning;
the principle above governs. The exact per-file manifest is produced in the
implementation plan.

## 6. What stays in `m68k-ooo`

`rtl/core/*`, the new `m68k_axi_wrapper` + relocated `debug_ctrl`/ILA, Musashi
(`tb/models`), fuzz (`tools/fuzz`), CPU unit tbs (tb_alu/rob/rat/iq*/decode),
ISA/uarch/decode docs, the CPU-half of the Makefile (test/fuzz/lint-core).

## 7. Repo mechanics (history-preserving)

1. Clone `m68k-ooo` → run **`git filter-repo`** keeping only the platform paths
   (rtl/mac, rtl/sys, rtl/vendor, rtl/fpga_top*, mac_top.v, synth, files, the
   platform tb/tools/docs subset). Result = `~/macqd700-soc/` with full history
   of those files.
2. Reorganize into `rtl/mac` / `rtl/board` / `rtl/soc` (history-preserving
   `git mv`).
3. Add `cpu_socket.vh` + `cpu_stub.v`; retarget `fpga_top_cpu.vh` from
   `m68k_core` to the socket; remove the CPU-specific `dbg_*` plumbing (now
   socket-minimal).
4. `make lint-fpga-top` (or its successor) green on the stub.
5. Add `cpu/` git submodule; in `m68k-ooo` add `m68k_axi_wrapper` + relocate
   `debug_ctrl`/ILA; point the submodule at it.
6. Integrate; boot-sim parity check vs the pre-split baseline.
7. In `m68k-ooo`, delete the now-moved platform files (kept in history) and
   reduce the Makefile to CPU targets. Update CLAUDE.md / docs.

## 8. Build / test split

- **macqd700-soc:** `synth`/`impl`/`lint-fpga-top`, platform unit tbs (via1/2,
  scsi, scc, asc, video, sd, axi-xbar, ddr, vram), ROM-boot + lockstep tbs
  (which require a CPU → run with the submodule CPU or the stub where
  CPU-independent), JTAG/SD/mame-replay tools.
- **m68k-ooo:** `test` (CPU directed), `fuzz`/`fuzz-deep` (Musashi), `lint-core`,
  decode-check.

## 9. Validation / acceptance

- `macqd700-soc` lints + elaborates `fpga_top` with `cpu_stub` (no CPU).
- With the m68k submodule + `m68k_axi_wrapper`: ROM-boot sim reaches the same
  milestone as the pre-split baseline (DAFB render), and a directed
  peripheral-tb subset passes.
- `m68k-ooo` `test`/`fuzz` unchanged (CPU untouched functionally).
- No file silently lost: every moved file is either in `macqd700-soc` or
  deleted-with-history-note in `m68k-ooo`.

## 10. Risks / open items

- **Width adaptation:** widening the m68k 32-bit `daxi` to 128/256 must preserve
  byte-lane/store-merge semantics; reuse the proven `axi_narrow_to_wide` and
  add a directed tb.
- **Debug relocation:** moving `debug_ctrl`/ILA to the CPU repo is the largest
  refactor; the socket's minimal-debug subset must still support JTAG-AXI
  bring-up of the SoC standalone.
- **Lockstep tbs** depend on a CPU; decide per-tb whether they live SoC-side
  (with submodule CPU) or move to the CPU repo.
- **filter-repo path list** must be exact; dry-run and diff file inventories
  before/after.
- The SP-drift wedge and BCHG-CCR regression are **CPU-side** and ride along in
  `m68k-ooo` unaffected by this split.
