# Endianness audit — executive summary

(2026-04-28; full audit: `docs/endianness_audit.md`)

The m68k-ooo SoC sits on a contradiction: the 68k LSU emits
**BE-lane** (byte at lowest address at `wdata[31:24]`,
`wstrb[3]` hot for byte 0) while every Xilinx IP block (MIG UI,
JTAG-AXI master, XDMA) and the AXI4 spec itself uses **LE-lane**
(`wstrb[0]` guards `wdata[7:0]`).  The system survives because every
fabric consumer is byte-symmetric — DDR per-byte split, narrow→wide
replicate-and-place, peripheral_bus `wr_byte` mux all pick whichever
strb bit is hot regardless of which one "should" be.  Add to that
two more disagreements: VRAM uses **LE-pixel** (pixel 0 at lane bits
`[7:0]`) so the CPU↔VRAM hop needs an explicit byte-reverse, and
DDR's 16-byte beat layout requires `if_to_axi` to reverse the four
32-bit words within the line.  Result: at least three independent
within-lane byte-reverses live in the codebase, the if_to_axi 4-word
reverse, and a half-dozen "this is a custom convention, mind the
gap" comments scattered through `lsu.v`, `peripheral_bus.v`,
`vram.v`, `boot_fsm.v`, `if_to_axi.v`, and `tb_top.cpp`.

## What the audit found

**One concrete bug, fixed in this audit.**

  - Production VRAM CPU-write path was **double-swapped**.
    `axi_xbar.v` (commit b4623c6, Codex tb deepening) added a
    within-lane byte-reverse on its S3 master ports.
    `rtl/fpga_top_video.vh` (commit 098491b, the original P1
    pixel-mirror fix) had ALREADY inlined the same swap on
    S3↔vram.  The two cancel out — every CPU write to the VRAM
    aperture would show the original P1 mirror bug in the real
    bitstream.  Neither tb (`tb_vram_xbar_e2e`, `tb_framebuffer_pixel`)
    stitches both layers, so the regression slipped past CI.
    **Fix: removed the duplicate inline swap from
    `fpga_top_video.vh`**; the xbar's swap now stands alone.
    `tb-framebuffer-pixel` PASS, `tb-vram-xbar-e2e` 11/13 PASS
    (two unrelated pre-existing OOB-SLVERR mismatches).

**Six brittle spots, no current correctness impact** — but each is
a landmine for the next agent who touches the surrounding code:

  - **B2** — three independent copies of the within-lane byte-
    reverse (`vram_cpu_byteswap.v`, `axi_xbar.v` `vram_swap_word32`,
    the inline one just removed).  Single-source it through the
    module form.
  - **B3** — JTAG-AXI sub-word writes use LE-lane and end up byte-
    inverted vs CPU view.  Today's JTAG flow only uses full-word
    ops, so it's invisible.
  - **B4** — peripheral_bus `wr_byte` mux drops a byte on word-
    strided stores.  Today's Mac peripherals are 8-bit so it's
    invisible.
  - **B5** — boot_fsm BE-pack ↔ DDR LE-lane works only because
    four separate conventions all agree.  `tb_cold_boot::test_endian`
    pins this; **refutes the P1#7 hypothesis** from the
    2026-04-23 memory note.
  - **B6** — `tb_peripheral_bus`'s `axi_write` helper tests the
    LE-lane convention, not the BE-lane the LSU actually emits.
    Bug in the BE-lane peripheral path would slip past today's tb.
  - **B7** — JTAG `r addr` displays LE-rdata; sub-word inspection
    requires reading the high byte for "byte at lowest address".

**Boot-FSM 4-byte-reversal hypothesis (P1#7): REFUTED.**  The chain
`SD bytes → boot_fsm BE-pack → narrow→wide → DDR LE-lane → if_to_axi
4-word reverse → CPU BE-lane consumption` is consistent end-to-end.
`tb_cold_boot::test_endian` (registered, runs on every
`make tb-cold-boot`) gates this with a byte-by-byte assertion.  No
SD-boot byte-reversal bug exists today.

## Top 3 brittle spots ranked by risk × likelihood

1. **B1 / Production VRAM double-swap (FIXED)** — would have shown
   up the moment the next bitstream rendered a CPU-painted frame.
   Fix: `rtl/fpga_top_video.vh` lines 184–221 / 230–235 / 261, single
   commit.
2. **B2 / Three independent copies of the same byte-reverse** —
   risk that the xbar swap and the standalone module diverge as
   one is "fixed" without the other.  Fix: have `axi_xbar.v`
   instantiate `vram_cpu_byteswap` instead of inline functions.
3. **B6 / `tb_peripheral_bus` axi_write uses LE-lane** — a real
   peripheral-side BE-lane bug would slip past CI.  Fix: add a
   parallel BE-lane helper, re-run the VIA / SCC / ASC tests
   through it.

## Recommended canonical conventions (one per boundary type)

  - CPU LSU: BE-lane (status quo).  Document the inversion vs
    standard AXI4 wstrb-byte mapping at the top of `lsu.v`.
  - DDR storage: LE-lane (Xilinx MIG, status quo).
  - Peripheral bus byte select: pick lowest hot strb bit, take
    matching wdata byte.  Status quo.
  - VRAM xbar S3 boundary: ONE within-lane byte-reverse, applied
    in `axi_xbar.v`; `fpga_top_video.vh` MUST NOT inline a copy
    (now invariantly true after this audit).
  - JTAG / PCIe debug: standard AXI4 LE-lane.  Document the
    inversion vs CPU BE-lane for sub-word ops.

## Test gaps to file as follow-ups

  1. `tb_endianness_byte_lane.cpp` — exercise BE-lane sub-word
     writes through peripheral_bus for all four byte positions
     across all four lane positions.  **Provided by this audit**
     (see `tb/tb_endianness_byte_lane.cpp`).
  2. `tb-vram-xbar-cpu-end-to-end` — stitch narrow→wide + xbar +
     fpga_top_video.vh + vram together, drive a CPU-style BE-lane
     longword, scan-out check.  Would have caught B1 on commit
     day.
  3. `tb-jtag-axi-byte-poke` — drive xbar with a JTAG-AXI LE-lane
     byte write, verify CPU sees the byte at the CPU-lane
     position.  Catches B3 if JTAG ever accepts sub-word writes.

## Files added by this audit

  - `docs/endianness_audit.md` — full audit (~370 LoC).
  - `docs/endianness_audit_summary.md` — this file.
  - `tb/tb_endianness_byte_lane.cpp` — directed test of the
    peripheral_bus BE-lane convention (~250 LoC).

## Files touched by this audit

  - `rtl/fpga_top_video.vh` — removed duplicate inline VRAM
    byte-swap (60→20 LoC in the relevant block).
