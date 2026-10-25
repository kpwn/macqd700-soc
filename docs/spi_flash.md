# KU5P SPI programming with openFPGALoader

Use `openFPGALoader` for persistent programming on the RK-XCKU5P-F board.
This interrupts the running Mac, replaces the boot image at flash offset zero,
and boots the FPGA again. It does not provision or overwrite the SD card.

Add your own ADB modem ROM using the [firmware setup guide](adb_firmware_bitstream.md),
then flash the resulting `fpga_top.local.bit`. The unpatched release downloads
(`fpga_top.python.bit` and `fpga_top.blank.bit`) have no working ADB modem.

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

This local verification used a firmware-patched image. Published bitstreams
exclude Apple firmware.

## TQTT-KU5P

The TQTT board's FT232H cable (VID:PID `0403:6014`, tested serial
`210251A08870`) uses openFPGALoader's `digilent_ad` cable profile. The RK
board's `digilent` profile fails before erase on TQTT. Check the cable serial
with `--scan-usb` and the FPGA ID with `--detect` before writing. Stop the
JTAG REPL and the specific Vivado `hw_server` process that owns the cable,
under the JTAG lease, then run:

```sh
openFPGALoader -c digilent_ad --ftdi-serial 210251A08870 \
  --freq 6000000 --fpga-part xcku5p-ffvb676 --detect
openFPGALoader -c digilent_ad --ftdi-serial 210251A08870 \
  --freq 6000000 --fpga-part xcku5p-ffvb676 \
  -f --verify /absolute/path/to/fpga_top.local.bit
```

The tested board reported FPGA IDCODE `0x04a62093` and Micron flash JEDEC
`0x20bb18` (128 Mbit, 16 MiB). This differs from the 256 Mbit configuration
flash named in the available schematic. Check that the entire bitstream fits
the detected device; the tested image ended at `0x00ec0000`.

**Readback verification does not prove a flash boot.** On 2026-10-25, the
verified flash write left FPGA `DONE=0`; `openFPGALoader -r` did not bring it
up. After restoring `hw_server`, Vivado's `boot_hw_device [current_hw_device]`
completed after about 50 seconds with `DONE=1`. Reattach the REPL with
`JTAG_REPL_NO_PROGRAM=1` and the matching `.ltx`, then run `refresh` to bind
the debug cores and check `build-id`, `perf live`, and video. Do not use a
normal SRAM programming attach when verifying flash boot.

The tested 200 MHz TQTT image had SHA-256
`6ad31b3e181ee0eba1c8037aa0854b395134a620a023b6d784d6726622970db5`.
Readback passed, a flash boot reported build ID `0xB9BE30AF`, the CPU retired
instructions, and Mac OS appeared on HDMI capture. This image has a routed
setup timing miss of 0.296 ns, manual fan PWM, and no HDMI audio. A fresh
integrated image is still required after SD and audio work.

On 2026-10-25, the newer 100 MHz C96 write-completion image
(`/var/tmp/tqtt_c96_waitfix_100/vivado/fpga_top.local.bit`, SHA-256
`15af1955b9646b4558db806d62cf1bcd6abf3ed3afc3ef4479628a76725b4219`)
also passed `openFPGALoader --verify`. Vivado `boot_hw_device` reached
`DONE=HIGH`, and an attach without SRAM programming read build ID
`0xAC1FF528` and advancing CPU counters. However, `vhdd-status` read IDENT
`0x00000000` rather than `0x5D0D0001`, and the displayed Mac did not boot
normally. Loading the *same* `.local.bit` through JTAG immediately restored
the vHDD IDENT and its enabled 4,194,240-block SD volume. Therefore, DONE,
build ID, and CPU activity alone are insufficient flash-boot checks for this
image. The flash startup failure remains to be diagnosed; the board is
currently running the JTAG-loaded image. In that working JTAG state,
`vio_rst_bundle=0x2e` means platform reset released, DDR calibrated, ROM ready,
and HDMI I²C finished; `vio_boot_video=0x38` shows HDMI clock lock and no boot
error. Capture these probes after the next flash boot to locate the stalled
stage before reloading SRAM.
