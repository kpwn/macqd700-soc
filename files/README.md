# User-supplied firmware

Firmware is not distributed with the source. Supply dumps you are entitled
to use; do not commit them as part of the source release. New builds emit a
blank ADB BRAM bitstream for [local firmware insertion](../docs/adb_firmware_bitstream.md).
The initial `200mhz-20260919` bitstream predates this flow and embeds ADB
firmware; it has not been silently replaced.

| Local path | Size | Consumer |
|---|---:|---|
| `files/420dbff3.rom` | 1,048,576 bytes | Quadra 700 boot ROM, SD provisioning and ROM simulations |
| `files/342s0440-b.bin` | 1,024 bytes | ADB PIC modem, FPGA builds and ADB integration simulation; also MAME |
| `files/pmuv2.bin` | 6,144 bytes | MAME configurations requiring PMU firmware |

Before running the real-firmware ADB test (not required for synthesis):

```sh
make prepare-adb-firmware
# Or use a dump stored outside this checkout:
make prepare-adb-firmware MAME_ADB_ROM=/path/to/342s0440-b.bin
```

This generates the ignored `rtl/mac/adb_pic_fw.hex` file: 512 little-endian
12-bit PIC instructions, rendered as one three-digit hex word per line.
This file is used only by simulation, not by FPGA synthesis. The
converter rejects wrong lengths and words outside the PIC instruction width.

`ddr4.xdc` is a board constraint file, not firmware, and remains tracked.
