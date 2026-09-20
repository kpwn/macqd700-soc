# SD provisioning over JTAG

The provisioning bitstream gives the host direct access to the SD card;
it does not run the Mac. Loading it resets the running machine. Keep the
firmware-patched Mac bitstream and its matching `.ltx` ready to reload.

## Card layout

| Card sectors (512 bytes each) | Contents |
|---|---|
| 0–2047 | 1 MiB Quadra boot ROM |
| 2048–8175 | Reserved gap |
| 8176–8191 | System-persistence reservation; PRAM at 8191 |
| 8192 onward | Raw Macintosh disk image |

A disk-image sector maps to **card sector 8192 + image sector**. Never write
a disk image at card sector zero. Writing either region destroys its previous
contents; preserve any data you need first. The ADB modem ROM is not stored
here: it is [inserted into the FPGA bitstream](adb_firmware_bitstream.md).

## Tools

Build the provisioning image with `make sd-provision-impl`. Its top level
is [sd_provision_top.v](../rtl/soc/sd_provision_top.v), separate from the
normal Mac build. The implementation gate and board access must be scheduled
exclusively, just as for a normal hardware build.

The region-specific wrappers are:

- `tools/sd_write_rom.sh`: ROM writes, guarded below sector 2048.
- `tools/sd_os_swap.sh`: disk writes, guarded at or above sector 8192.

Inspect their arguments and image-selection settings before use. In
particular, select your actual patched Mac image through `MAIN` and its
probes through `LTX`, rather than relying on an old build-directory default.
For direct card-reader provisioning, use the [main README](../README.md#sd-card-layout-and-preparation).

## Bulk-writer interface

The JTAG REPL provides `sd-fast-status` and
`sd-write-fast <card-lba> <file>`. The latter requires the provisioning
image, stages batches in BRAM, writes them with CMD25, and reads them back
with CMD18 for CRC32 verification. Leave verification enabled. The dedicated
provisioning engine is distinct from the Mac's runtime CMD24 storage path.

Use the JTAG lease and FIFO lock for the entire transfer. Do not interleave
debug commands or reset the board during a write. On failure, leave the
provisioning image loaded and investigate before booting a partly written
disk. On success, reload the patched Mac image and matching probes.

Writing only used extents requires a validated allocation map, including
partition metadata and the alternate HFS volume header. A trimmed prefix is
not the same as an exact set of used extents. Check sector units, image-to-card
offsets and the final allocated block; verify every written extent.
