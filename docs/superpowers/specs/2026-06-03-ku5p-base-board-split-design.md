# Design: split the KU5P board-support into `ku5p-base-board`

**Date:** 2026-06-03
**Status:** approved (brainstorming) — pending spec review
**Origin:** `macqd700-soc` @ tag `pre-board-split-b938795`
**Goal:** carve the KU5P board-support package (BSP) out of `macqd700-soc`
into a reusable repo `~/ku5p-base-board/`, consumed as a git submodule, so
other devices built on the same board can share it.

---

## 1. Motivation

`macqd700-soc` currently bundles three things: the Quadra-700 **machine**
(`rtl/mac`), the **integration/fabric** (`rtl/soc`: `fpga_top`, xbar, boot,
peripheral bus), and the **KU5P board support** (`rtl/board`: clocking/reset,
DDR4 MIG, SD, VRAM, audio PHYs, HDMI video PHY, vendor MIG stub). The board
support is device-independent and will be reused by other devices on the same
KU5P board. Splitting it out behind the already-documented machine↔board
contract lets each device share one BSP.

The machine↔board seam is already documented in `docs/bsp_contract.md` (six
seams: main-memory AXI slave, framebuffer/scanout, audio sample stream,
clocks/reset, SD/boot, debug transport). The board layer is already isolated in
`rtl/board/` (Phase-2 of the CPU split). This split is the natural next step.

## 2. Topology — board as a submodule dependency (mirrors the CPU)

```
   m68k-ooo (CPU)  ──cpu socket──┐
                                 ▼
   macqd700-soc  = INTEGRATOR (owns rtl/soc/fpga_top + rtl/mac machine)
                                 ▲
   ku5p-base-board (BSP) ──board modules── consumed at fpga_top
```

`macqd700-soc` stays the integrator and keeps `fpga_top`. It consumes **two**
submodules: `cpu/` (m68k, already mounted) and a new `board/`
(`ku5p-base-board`). A future device is its own integrator repo pulling the
same `board/` BSP. This is the same pattern as the CPU split.

## 3. Key difference from the CPU: no socket/stub

The CPU was one clean dual-AXI *socket* with a tie-off *stub* enabling a
fabric-only standalone build. The board is a **module library**: `fpga_top`
instantiates `ddr_ctrl` / `clk_rst` / `reset_debounce` / `video_phy/video_top`
/ `sd_ctrl` / `vram` / the audio PHYs directly. "macqd700-soc without a board"
is not a meaningful build (no memory, no clocks), so there is **no board-stub**.
The board is **always present**:

- **SIM** uses the board's own sim models (`sim_mig_backend` BRAM-backed DDR,
  `vram_smoke`, etc.) — exactly as today.
- **SYNTH** uses the real MIG (`axi_ddr4_mig_bridge` + the vendor IP).

The contract IS the six module interfaces in `docs/bsp_contract.md`; no new
`board_socket.vh` is introduced (the modules' header ports already define it).

## 4. What moves to `ku5p-base-board`

History-preserving `git filter-repo` of the board layer:

- **RTL:** `rtl/board/**` — the 17 BSP modules + `video_phy/` (HDMI PHY:
  `video_top`, `mmcm_hdmi`, `vtg`, `scaler`, `fb_reader`, `linebuf_scanout`,
  `i2c_init`, `scanout_placement_sync`, `vram_smoke`) + `vendor/`
  (`design_1_ddr4_0_1_stub.v`).
- **Board unit tbs:** the board's own testbenches (DDR-model, VRAM, SD-ctrl/
  SD-boot, video-PHY / scaler / DAFB-scanout, audio, clk/reset, async-fifo) +
  their harness sources, so the BSP is independently testable.
- **Docs:** `bsp_contract.md` (the board's public-interface doc) and any
  board-specific docs (DAFB/video, DDR/MIG, SD/boot bring-up).

**Stays device-side in `macqd700-soc`:** `synth/` (`vivado.tcl`, `ku5p.xdc`,
`debug_ila.tcl`) — the device builds its own bitstream from `fpga_top`; the XDC
constrains `fpga_top`'s top-level pins to the KU5P board. (A later split of the
board-pin constraints into the board repo is possible but out of scope.)
`rtl/mac` (machine), `rtl/soc` (integration/fabric), and the integration/lockstep
tbs stay.

## 5. `macqd700-soc` changes

1. `rtl/board` removed; the BSP mounted as a **git submodule at `board/`**,
   pinned to a commit.
2. `fpga_top` / `fpga_top_*.vh`, the `Makefile`, and `synth/vivado.tcl` retarget
   their `rtl/board/*` source paths and `-I`/`+incdir` dirs to the submodule
   (`board/rtl/...` or `board/...`, matching the board repo's internal layout).
3. No instance changes — `fpga_top` instantiates the same board module names,
   now sourced from the submodule.

## 6. Mechanics (mirror the CPU split's phases)

1. Tag baseline (`pre-board-split-b938795`, done) + capture board file
   inventory.
2. `git filter-repo` a fresh clone of `macqd700-soc` keeping only the board
   paths → `~/ku5p-base-board/`; verify inventory match; prune non-board
   leftovers; add README.
3. (Optional) organize the board repo internally (`rtl/...`, `tb/...`).
4. In `macqd700-soc`: remove `rtl/board`, add `board/` submodule, retarget
   `fpga_top`/Makefile/synth paths.
5. Verify: `lint-fpga-top CPU=m68k` elaborates with both submodules; **ROM-boot
   parity** (same DAFB milestone: `ddr_cal_done`, `q700_descriptor_selected`
   entry=0x4080390c, retired≈2,000,000, PC 0x40847516).
6. The board repo's own BSP unit tbs pass standalone.

## 7. Validation / acceptance

- `ku5p-base-board` builds + passes its BSP unit tbs standalone (DDR-model,
  VRAM, SD, video-PHY).
- `macqd700-soc` with `cpu/` + `board/` submodules: `lint-fpga-top CPU=m68k`
  0 errors; ROM-boot reaches the pre-split DAFB milestone (parity).
- Inventory diff: every `rtl/board` file accounted for in the new repo.
- No regression in `macqd700-soc` platform tbs that don't depend on the board.

## 8. Risks / open items

- **Path retargeting breadth:** the Makefile + `synth/vivado.tcl` reference the
  board files in several lists/`-I` dirs; a dry `lint-fpga-top` after retarget
  must show only-expected errors before adding the submodule, then 0 after.
- **filter-repo path list** must be exact; dry-run + inventory diff.
- **Board tbs that touch fabric:** some "board" tbs may instantiate fabric/soc
  modules (e.g. an SD↔SCSI bridge tb); classify per-tb (board-only vs
  integration), keep integration tbs device-side.
- **MIG vendor IP:** only the `vendor/` stub moves (RTL); the actual MIG IP
  generation config, if any, lives under `synth/` and stays device-side for
  now — flag if the board repo needs it for standalone synth (out of scope:
  the board repo is sim-testable; synth is device-side).
- The CPU submodule + socket are unaffected by this split.
