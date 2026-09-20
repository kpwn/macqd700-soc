# ADB firmware setup

Releases do not include Apple firmware. Before flashing, add your own
`342s0440-b.bin` ADB modem ROM to the bitstream. This is required even if
you use injected keyboard and mouse input.

## Using the Python bundle

Download `fpga_top.python.bit`, `fpga_top.python.adb-map.json` and
`adb-patcher.zip` from the same release. Extract the patcher, then run:

```sh
python3 adb-patcher/patch_adb_bitstream.py patch \
  --bit fpga_top.python.bit \
  --manifest fpga_top.python.adb-map.json \
  --firmware /path/to/your/342s0440-b.bin \
  --output fpga_top.local.bit
```

This needs only Python 3—no Vivado, UpdateMEM or additional Python packages.
If you cloned the repository, use `tools/patch_adb_bitstream.py` instead.

Flash **`fpga_top.local.bit`** using [openFPGALoader](spi_flash.md). Do not
flash the unpatched download: its ADB modem has no program. Keep the release's
matching `.ltx` and build information. The patched file contains your Apple
firmware; do not redistribute it.

Your ROM dump must contain 512 little-endian, 12-bit instructions in 16-bit
containers (1,024 bytes total). The patcher rejects incorrect sizes, invalid
upper bits, mismatched bitstream hashes, malformed maps and existing output
files. The Quadra boot ROM is separate and goes on SD; the ADB modem program
loads directly from the bitstream when the FPGA starts.

## Legacy bundles and local builds

The original `.blank.bit`, `.adb.mmi` and `.adb.json` bundle uses AMD
UpdateMEM. If your download has no `.adb-map.json`, either obtain the Python
bundle or run the legacy command with UpdateMEM on your PATH:

```sh
python3 tools/patch_adb_bitstream.py patch \
  --bit fpga_top.blank.bit \
  --mmi fpga_top.blank.adb.mmi \
  --manifest fpga_top.blank.adb.json \
  --firmware /path/to/your/342s0440-b.bin \
  --output fpga_top.local.bit
```

Use `--updatemem /path/to/updatemem` if needed. Always use companions from
the same build; maps are not interchangeable between builds or formats.

## Preparing a release (maintainers)

The Python bundle uses an uncompressed bitstream and a build-specific map
of the PIC's 6,144 instruction bits. Creating this bundle requires Vivado
and UpdateMEM; using it does not. Start with the final routed checkpoint:

```sh
vivado -mode batch -source synth/export_adb_python_base.tcl \
  -tclargs build/vivado/checkpoints/route.dcp build/python-release
python3 tools/build_adb_patch_map.py \
  --bit build/python-release/fpga_top.python.bit \
  --mmi build/python-release/fpga_top.python.adb.mmi \
  --manifest build/python-release/fpga_top.python.adb.json \
  --output build/python-release/fpga_top.python.adb-map.json
```

The export reuses placement and routing, and checks that the dedicated PIC
BRAM and parity are empty. The map generator uses synthetic instruction
patterns to identify the physical bits, then compares Python-patched images
against UpdateMEM's output. It writes the map only after every complete
configuration payload matches, including CRC, frame ECC and non-PIC data.

The patcher regenerates configuration CRCs. In the supported BRAM layout,
changing program contents does not change frame ECC; the exporter rejects
any mapping or vendor comparison that violates this. Compressed, encrypted
and other unsupported packet layouts are rejected, not guessed at.

Publish the `.python.bit` and `.python.adb-map.json` together. Run
`python3 tools/package_adb_patcher.py build/python-release/adb-patcher.zip`
to bundle the three Python modules, instructions, license and notices.
Do not publish local
ROM dumps or patched bitstreams. File hashes bind a map to its base image;
they are not signatures, so obtain both from a trusted release.

## Hardware layout

The PIC program occupies words 0–511, bits 11:0, of one protected RAMB36E2
configured as 1,024 × 32. Unused words, upper bits and parity stay zero.
The modem runs on the 50 MHz peripheral clock, fetching on the falling edge
and executing on the rising edge. Both paths remain constrained to 10 ns.
Firmware insertion does not alter logic, placement, routing or timing.

Tests use synthetic firmware only:

```sh
python3 tools/test_patch_adb_bitstream.py
python3 tools/test_adb_config.py
python3 tools/test_prepare_adb_firmware.py
```

The separate `synth/pic_bram_roundtrip.tcl` test compares UpdateMEM against
Vivado INIT writes at fixed placement. Its standalone test bitstreams must
never be programmed on a board.
