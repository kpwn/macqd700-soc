# KU5P SPI programming with openFPGALoader

Use `openFPGALoader` for persistent programming on the RK-XCKU5P-F board.
This interrupts the running Mac, replaces the boot image at flash offset zero,
and boots the FPGA again. It does not provision or overwrite the SD card.

New builds produce `fpga_top.blank.bit`, which **must not be flashed as a
working Mac image**. First perform [local ADB firmware insertion](adb_firmware_bitstream.md)
and explicitly select its new output below. A blank image has no working ADB
modem. The historical verified release described below already contains it.

1. Verify the intended `.bit` hash and adjacent `.buildinfo`; do not pick
   an artifact merely because its directory is named `vivado` or `200`.
2. Coordinate exclusive board access using `tools/jtag_lease.sh`. Close
   Vivado's hardware target and release the USB cable from its hardware
   server. Closing the target alone may leave `hw_server` holding USB.
   Identify the owning process before stopping it; never use broad `pkill`.
3. Discover the cable with `openFPGALoader --scan-usb`. Confirm the FPGA
   with `--detect`, without `-f`. The tested cable uses `-c digilent`;
   supply its actual FTDI serial rather than selecting an arbitrary cable.
4. Program and verify, selecting the exact artifact:

```sh
tools/jtag_lease.sh run spi-program 900 -- \
  openFPGALoader -c digilent --ftdi-serial YOUR_CABLE_SERIAL \
    --freq 6000000 --fpga-part xcku5p-ffvb676 \
    -f --verify /absolute/path/to/fpga_top.bit
```

Wait for a successful exit after read-back verification. Do not disconnect
power during erase/programming. No bulk erase or flash-unprotect option is
needed for the tested board. Its reported JEDEC bytes were `c2 25 3a`; the
installed loader used basic protection detection for this unrecognized chip.

5. Restore the hardware server using its installed **wrapper**, not the
   `unwrapped/` binary (the wrapper sets up cable-driver libraries).
   Reattach the REPL without programming SRAM, attach the matching `.ltx`,
   then check `build-id` and `halt-status`. Use `perf live` for running
   instruction counts: `inst-count` is a halt snapshot and can legitimately
   read zero while the machine is running.

Verified on 2026-09-19: build `b22df792`, CPU `8dccf93d`, 200 MHz,
bitstream SHA-256
`390205857a11cf7fbc69100d73b870581dc56150278fce96653602d40fdcd598`.
Flash read-back passed; subsequent attachment reported that build ID,
retiring instructions and no halt/double fault. This was a flash boot,
not a separate physical power-cycle test.

This configuration embeds ADB firmware. The source tree excludes that
firmware; any prebuilt bitstream release must disclose its embedded content.
