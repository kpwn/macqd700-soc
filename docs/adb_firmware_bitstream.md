# Local ADB firmware insertion

The PIC modem program is a dedicated, protected RAMB36E2. New builds produce
`fpga_top.blank.bit`, `fpga_top.blank.adb.mmi`, and `fpga_top.blank.adb.json`.
The bitstream has zero PIC program contents, even if a user's simulation hex
file exists in the checkout. The map and manifest contain no firmware.

**A blank image does not provide a working ADB modem. Do not program it.**
The initial `200mhz-20260919` bitstream has been withdrawn. It used LUTROM
and cannot use this method. The next release will supply the blank BRAM
image and matching map/manifest; download all three from the same release.

## Populate your image

This first implementation uses **AMD UpdateMEM**, either on PATH after
sourcing `settings64.sh`, or supplied with `--updatemem /path/to/updatemem`.
It does not require synthesis or place-and-route. It is not yet a Python-only
configuration-frame patcher and must not be advertised as Vivado-independent.

```sh
python3 tools/patch_adb_bitstream.py patch \
  --bit build/vivado/fpga_top.blank.bit \
  --mmi build/vivado/fpga_top.blank.adb.mmi \
  --manifest build/vivado/fpga_top.blank.adb.json \
  --firmware /path/to/your/342s0440-b.bin \
  --output build/vivado/fpga_top.local.bit
```

The input dump is exactly 1,024 bytes: 512 little-endian 12-bit words in
16-bit containers. Invalid lengths/upper bits are rejected, not silently
masked. The script checks both base-image and MMI SHA-256 hashes before
running UpdateMEM, refuses to overwrite any existing output, and removes its
private temporary firmware file afterward. These hashes bind the map to a
particular build; they are not a signature or independent legal clearance.

Program **`fpga_top.local.bit`** using [openFPGALoader](spi_flash.md), retaining
the corresponding `.ltx` and build information. Its firmware is initialized
directly from FPGA configuration at power-up; there is no runtime SD loader.
The local output contains your firmware and must not be redistributed.

## Timing and physical layout

The memory uses 1,024 32-bit words, with PIC words 0..511 in bits 11:0.
Unused bits/words and parity remain zero. A whole RAMB36 deliberately avoids
compiler-dependent packing, parity-lane instruction bits and RAMB18 updater
compatibility differences. Explicit instantiation and `DONT_TOUCH` preserve
the zero-filled memory and its consumers through optimization.

The modem is clocked by **50 MHz `pb_clk`**, not the 200 MHz CPU clock (see
`fpga_top_peripherals.vh`). The PC updates on its rising edge; memory reads on the
falling edge using its built-in clock inversion. The next rising edge sees
the correct instruction even with consecutive `cyc_en` pulses, redirects,
PIC branch bubbles and reset. No PIC execution or ADB GPIO edge is delayed.
Both PC-to-BRAM and BRAM-to-execute paths are **half-cycle paths** (10 ns in
this integration). Do not false-path them. This RTL change does **not** inherit
the historical whole-SoC timing closure: a new full implementation and board
check are still required.

The synthesis/optimization smoke test is:

```sh
vivado -mode batch -source synth/pic_bram_ooc.tcl -tclargs build/pic-bram-ooc
make tb-pic16c5x
python3 tools/test_patch_adb_bitstream.py
```

The main and resumed implementation scripts check the primitive, placement
and all 128 data / 16 parity INIT properties **before** writing any blank image,
then export its placement-specific MMI and matching file hashes. Old
checkpoints without the dedicated primitive are rejected before writing.
Do not pair files from different builds or rename an old image as a blank one.

## Standalone patcher follow-up

A Python-only patcher remains possible, but requires a validated UltraScale+
configuration-frame mapping plus CRC/ECC handling. Synthetic INIT patterns
written from the **same fixed implemented checkpoint** can establish a
per-build mapping; repeated synthesis/place-and-route cannot. Such a patcher
must be checked against vendor-generated images and on hardware before use.
This change does not ship an unverified raw-bit rewriting implementation.

## Validation recorded 2026-09-19

- 39 synthetic PIC scenarios pass: 13 instruction/reset/redirect cases under
  consecutive, sparse and irregular enables. The 1,270-line rising-edge
  trace matches the previous asynchronous RTL byte-for-byte (PC, W, ports,
  TRIS, retirement). The previous port-read test had stale TRIS-mux
  expectations; both RTL versions pass the corrected PIC1654S open-drain test.
- Vivado 2025.2 synthesis plus optimization retains one RAMB36E2, 589 LUTs and
  305 FFs with zero program INIT: neither ROM nor its consumers fold away.
- At the integrated 50 MHz peripheral clock, out-of-context timing estimates
  WNS +6.280 ns and WHS +0.071 ns, with no failing endpoints. This is not routed
  full-SoC signoff. A separate 200 MHz PIC-clock stress estimate fails setup;
  that is not the clock frequency used by this modem in the 200 MHz SoC.
- Synthetic same-placement UpdateMEM/Vivado comparisons match the complete
  configuration packet payload, including CRC/ECC, for **both uncompressed
  and compressed images**. Only the `.bit` container header is excluded.
  This caught and corrected both the legacy MMI `RAMB32` naming requirement
  for physical RAMB36 and the little-endian MEM token byte order.
- Eleven patcher/preflight tests and three firmware conversion tests
  pass. New hardware behavior and full-SoC timing await a fresh build/board
  check; the existing working release was not replaced or reprogrammed.

Repeat the real vendor-tool round-trip with
`synth/pic_bram_roundtrip.tcl` (usage is in its header). Its tiny test image
uses unconstrained test IO and **must never be programmed on hardware**.

Vendor references: [UpdateMEM MMI format](https://docs.amd.com/r/2025.1-English/ug1580-updatemem/MMI-File-Syntax),
[MEM format](https://docs.amd.com/r/2025.1-English/ug1580-updatemem/Memory-Files),
[UltraScale BRAM addressing](https://docs.amd.com/r/en-US/ug573-ultrascale-memory-resources/Address-Bus-ADDRARDADDR-and-ADDRBWRADDR).
