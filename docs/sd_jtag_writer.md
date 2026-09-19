# SD-card writer over JTAG — implementation spec

> **Status:** design spec for a fresh session to implement. Not yet built.
>
> **Goal:** write arbitrary data (Mac ROM image, bootable disk image, …) to
> the FPGA's SD card *over JTAG*, with no physical access to the board.
> Speed does not matter — sector-at-a-time is fine.

## Why

The 1 MB Mac ROM (and any disk image) lives on the SD card; the on-FPGA
`boot_fsm` stages it into DDR at power-on (see `docs/rom_boot_bringup.md`).
Today the only way to change the SD contents is to pull the card and write
it from a host PC. When the board is remote (JTAG reachable, but no physical
access) that is impossible. This feature lets the JTAG host write SD sectors
through the existing JTAG→AXI master.

## The good news — the write path already exists

Almost everything needed is already in the tree; this feature is mostly an
**AXI front-end + wiring**, not new SD logic.

- **`rtl/sys/sd_ctrl.v`** — full SD-SPI command engine. It *already*
  implements `CT_CMD24` (WRITE_SINGLE_BLOCK) and `CT_CMD25`
  (WRITE_MULTIPLE_BLOCK). Its header literally says the write byte stream is
  meant for *"an AXI master burst-writer."* Ports:
  - `cmd_type[2:0]`, `lba[31:0]`, `block_count[15:0]`, `go` (1-cycle pulse)
  - write stream: `wr_ready` (1-cycle pulse per byte consumed), `wr_valid`
    (high during the data-block phase), `wr_data[7:0]`
  - `spi_cmd_*`/`spi_rsp_*` to `sd_spi`
  - `busy`, `done` (1-cycle pulse), `error` (sticky), `err_cause[3:0]`
- **`rtl/sys/sd_spi_mux.v`** — already has a **third, unused mux port named
  `prov`** (provision). `rtl/fpga_top_sd.vh` currently ties it off
  (`prov_cmd_valid(1'b0)`, `prov_cs_n_in(1'b1)`). It was clearly anticipated
  for exactly this. The SD writer drives that port.
- **JTAG→AXI master** — already present (`ENABLE_JTAG_AXI=1`); the JTAG REPL
  (`tools/jtag_repl.tcl`) issues `r`/`w` AXI reads/writes.

So the job: a small AXI-slave module that buffers a 512-byte sector, drives
`sd_ctrl` in `CMD24` mode through the `prov` mux port, and exposes status —
plus xbar wiring and a REPL helper.

## Design

### New module — `rtl/sys/sd_jtag_writer.v`

An AXI4-Lite (or the project's existing simple AXI-slave) peripheral.
Internally:
- a 512-byte sector buffer (`reg [7:0] buf [0:511]`, or a 128×32 BRAM);
- an instance of `sd_ctrl` (or shares the existing one — see arbitration);
- a tiny FSM: idle → on GO: pulse `sd_ctrl.go` with `cmd_type=CT_CMD24`,
  `lba=LBA`, `block_count=1`; stream the 512 buffer bytes out on each
  `wr_ready` pulse; wait for `done`; latch `error`/`err_cause`.

Register map (word-addressed; pick a free AXI region — see Wiring):

| Offset  | Name        | Access | Meaning |
|---------|-------------|--------|---------|
| 0x000   | `LBA`       | R/W    | target sector (block address) |
| 0x004   | `CTRL`      | W      | write `0x5D000001` = arm+GO (magic in [31:8] guards against stray writes); other values ignored |
| 0x008   | `STATUS`    | R      | bit0=busy, bit1=done-sticky, bit2=error, [7:4]=err_cause, [15:8]=last-LBA-low |
| 0x00C   | `BUFPTR`    | R/W    | byte index 0..511 into the sector buffer; auto-increments on each BUFDATA access |
| 0x010   | `BUFDATA`   | R/W    | write low byte → sector buffer[BUFPTR], BUFPTR++ (so the host streams 512 bytes by repeated writes); reading returns buffer[BUFPTR], BUFPTR++ for verify |
| 0x014   | `IDENT`     | R      | constant `0x5D7A0001` so the host can probe the block is present |

A 32-bit-wide `BUFDATA` window (write 4 bytes per AXI write, BUFPTR+=4) is a
fine optimisation but optional — slow is acceptable, 512 single-byte writes
per sector is OK.

### SD card initialisation

`sd_ctrl` is *data transport only* — it does **not** do card init
(CMD0/CMD8/ACMD41/CMD58). The card must already be in the initialised SPI
state before `CMD24`. Two options, in order of preference:

1. **Run the writer post-boot.** `boot_fsm` already initialises the card to
   read the ROM at power-on; the card stays initialised afterwards. Gate the
   writer so it only drives the bus once `boot_done` is asserted, and have
   the JTAG host hold the CPU halted (debug-CSR reset/halt — see
   `docs/reset_story.md`) while flashing so nothing else touches the bus.
   This is the simplest path and needs no new init logic.
2. If the writer must run *before* a normal boot (e.g. the SD is blank /
   wrong and `boot_fsm` itself fails), add a minimal init sub-FSM
   (CMD0 → CMD8 → ACMD41 loop → CMD58) ahead of the CMD24 path. `boot_fsm.v`
   already contains a working init sequence to copy from.

Recommend option 1 for v1.

### sd_spi_mux arbitration

`sd_spi_mux` muxes `boot` / `prov` / `scsi`. Extend its arbitration so the
`prov` port wins the bus when the writer's FSM is non-idle **and** the SCSI
side is idle (`!b_sel` or a quiescent check). Since the recommended flow
halts the CPU during flashing, SCSI will be idle — a simple
"`prov` has priority while `writer_busy`" is sufficient. Drive the mux's
`prov_*` inputs from `sd_jtag_writer`.

### Wiring

- Instantiate `sd_jtag_writer` in `rtl/fpga_top.v` / `rtl/fpga_top_sd.vh`.
- Add it as an AXI slave on `axi_xbar.v`. Pick a currently-unused decode
  region — the debug CSRs live around `0x5090_0000`; a clean choice is a new
  slot e.g. `0x50A0_0000`. Confirm against the live xbar decode in
  `rtl/sys/axi_xbar.v` + `rtl/fpga_top*.vh` and the existing slave list.
- Route `sd_jtag_writer`'s `sd_ctrl`-facing SPI signals into the `prov_*`
  port of `sd_spi_mux` (replacing the tied-off constants in
  `rtl/fpga_top_sd.vh`).
- The writer runs in the SD clock domain (same `clk`/`rst` as `boot_fsm` /
  `sd_spi`). The AXI side may be a different domain — use the project's
  existing `axil_async_bridge.v` / `axi_async_bridge.v` if so.

## JTAG REPL workflow

Add a helper to `tools/jtag_repl.tcl`: `sd-write <lba> <file>` that, per
512-byte chunk of `<file>`:
1. `w <BUFPTR_addr> 0` — reset the buffer pointer.
2. 512× `w <BUFDATA_addr> <byte>` — stream the sector (or 128× 32-bit).
3. `w <LBA_addr> <lba>` then `w <CTRL_addr> 0x5D000001` — arm + GO.
4. poll `r <STATUS_addr>` until `busy`==0; abort on `error`.
5. `lba++`, next chunk.

Document it in the REPL's command list (top-of-file docstring) alongside
`r/w/halt-status/...`. Manual use without the helper is also fine — it is
just AXI `w`s.

A 1 MB ROM = 2048 sectors; even at a few hundred sectors/sec over JTAG that
is well under a minute. "Slow is fine."

## Safety

Writing the SD is destructive. Mitigations:
- `CTRL` GO requires the magic `0x5D0000xx` upper bytes — a stray/partial
  AXI write will not trigger a sector write.
- Recommend the REPL helper refuse to write LBA 0..N without a `--force`
  flag if N covers the ROM staging region, to avoid bricking the boot image
  by accident (optional).
- Flash with the CPU halted (debug-CSR halt) so `boot_fsm`/SCSI are quiescent.

## Testing

- **`tb-sd-jtag-writer`** (new unit tb, model on `tb-sd-ctrl`): drive the AXI
  registers, fill the buffer, GO, and check the modelled SD card receives a
  well-formed `CMD24` + data token + 512 bytes + the data-response handshake.
  Use the existing SD-card sim model from `tb-sd-ctrl`/`tb-sd-boot`.
- **Round-trip**: in `tb-sd-boot` (or a new scenario), write a known pattern
  to a sector via the writer, then have `boot_fsm`/`sd_ctrl` read it back and
  compare.
- `make lint MODULE=sd_jtag_writer` clean; existing `tb-sd-ctrl` /
  `tb-sd-boot` must still pass (the `prov` port was previously tied off — the
  mux change must be regression-free for the `boot`/`scsi` ports).

## Files to touch

- `rtl/sys/sd_jtag_writer.v` — new.
- `rtl/sys/sd_spi_mux.v` — `prov`-port arbitration.
- `rtl/fpga_top_sd.vh` — instantiate, wire `prov_*`.
- `rtl/fpga_top.v` / `rtl/fpga_top_peripherals.vh` — AXI-slave hookup.
- `rtl/sys/axi_xbar.v` — new slave decode region.
- `synth/vivado.tcl` — add `read_verilog $rtl_dir/sys/sd_jtag_writer.v`.
- `tb/tb_sd_jtag_writer.cpp` + Makefile `tb-sd-jtag-writer` target.
- `tools/jtag_repl.tcl` — `sd-write` helper + docstring.
- `Makefile` — the new tb target.

## Estimated scope

Small–medium. No new SD-protocol logic (reuse `sd_ctrl`'s `CMD24`). The work
is one AXI-slave FSM (~150–250 lines), the mux arbitration tweak (~20 lines),
xbar/top wiring, a unit tb, and a REPL helper. One focused session.
