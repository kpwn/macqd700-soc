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

**Readback verification does not prove a flash boot.** Inspect `DONE`, `EOS`,
and the expected debug cores after programming. The older image below needed
Vivado's `boot_hw_device [current_hw_device]` to start; the later compressed
image started without it. Issue `boot_hw_device` only if the target remains
unconfigured. Reattach the REPL with `JTAG_REPL_NO_PROGRAM=1` and the matching
`.ltx`, then run `refresh` to bind the debug cores and check `build-id`, vHDD,
SD status, and video. Do not use a normal SRAM programming attach when
verifying flash boot.

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
image. At that point the flash startup failure was unresolved, and the board
was restored with a JTAG-loaded image. In that working JTAG state,
`vio_rst_bundle=0x2e` means platform reset released, DDR calibrated, ROM ready,
and HDMI I²C finished; `vio_boot_video=0x38` shows HDMI clock lock and no boot
error. Those probes were not captured during the first failed flash boot.

The owner observed all four status LEDs dark and the fan at full speed in the
failed flash-boot state. That is consistent with a clock/reset or early boot
failure: the LEDs indicate DDR calibration, SD activity, ROM load, and HDMI
clock lock, while the fan's PWM output deliberately releases to full speed
when the fabric MMCM is unlocked. It is also consistent with the board still
running the SPI-over-JTAG programming bridge before a successful FPGA reboot.
The previous attach did **not** record VIO reset/boot probes in that failed
state, so these observations do not yet distinguish those cases. Read those
probes before any JTAG SRAM reload on the next attempt.

On a second `boot_hw_device` attempt with the same verified flash contents,
without rewriting flash or programming SRAM, the board did boot normally.
Vivado reported `DONE=HIGH` after 50 seconds. After `refresh`, the REPL read
build ID `0xAC1FF528`, `vio_rst_bundle=0x2e`, `vio_boot_video=0x38`, and
vHDD IDENT `0x5D0D0001` with the 4,194,240-block SD volume enabled. The CPU
retired instructions, `video-status` showed `dafb_live=1` at 640x480, and
framebuffer requests and responses matched with no misses. The fan controller
was active (`vio_fan_state=0xa7268`). This proves one successful warm
configuration boot from the existing flash image; it does not prove reliable
power-on boot. The earlier failed attempt remains unexplained.

For a `boot_hw_device` retry from the running JTAG REPL, `tcl boot_hw_device
[current_hw_device]` can take about 50 seconds. `tools/jt.sh` may return after
the first quiet output line while that Tcl command is still running. Hold the
JTAG lease until the REPL emits the completed `boot_hw_device` timing line and
`> READY`, then issue `refresh` and read the probes. Do not send a second JTAG
command merely because `jt.sh` returned.

## 200 MHz TQTT flash boot diagnosis (2026-10-25)

The integrated 200 MHz image
`/var/tmp/tqtt_c96_waitfix_200/vivado/fpga_top.local.bit` (SHA-256
`806657f10d76dc78518ed1f3aa1ce340a7deecadc8b709f91fad24dbb025f5df`)
booted Mac OS and ran Speedometer over JTAG before flashing. Its routed setup
slack is -0.052 ns; hold slack is +0.003 ns. The owner authorized loading
this small-miss image.

`openFPGALoader -c digilent_ad --ftdi-serial 210251A08870 --freq 6000000
--fpga-part xcku5p-ffvb676 -f --verify` erased, wrote, and read back the
entire image successfully on the detected Micron `0x20bb18` 128 Mbit flash.
**Immediately after the loader exited, this Mac image was not configured.**
Vivado read `REGISTER.CONFIG_STATUS=0x10801904`: `DONE_PIN=0`, `EOS=0`,
`GWE=0`, with zero JTAG AXI and VIO cores. The capture server was receiving
fresh black frames. The loader had used its SPI-over-JTAG bridge, and verified
flash contents alone had not started this target FPGA image. In this case,
the dark board indicated a missing configuration boot step rather than bad
flash bytes.

Vivado `boot_hw_device [current_hw_device]` then reported `DONE=HIGH` after
about 50 seconds. After `refresh_hw_device`, Vivado found one JTAG AXI and
one VIO core; `EOS=1`. A REPL attach with `JTAG_REPL_NO_PROGRAM=1` and the
matching `.ltx` read build ID `0x8D3A363A`, `vio_rst_bundle=0x2e`, vHDD
IDENT `0x5D0D0001`, and native SD High Speed active. The SCSI probe showed
2,184 completions, zero SD errors, and zero CHECK CONDITIONs. HDMI capture
first showed the checkra1n splash, then Finder after dismissing the normal
improper-shutdown dialog; the [captured Finder frame](evidence/2026-10-25-tqtt-200mhz-flash-boot.jpg)
is preserved in this repository. No SRAM bitstream was loaded during this
verification. A physical power-cycle boot remains untested, and the earlier
100 MHz attempt that reached DONE but lacked vHDD IDENT remains a separate
unresolved observation.

The full TQTT XDC used for this image did not set the configuration SPI bus
width, configuration clock rate, or bitstream compression, although the
standalone TQTT probes did. Its 15.4 MB `.bit` and roughly 50-second Vivado
`boot_hw_device` time are consistent with a slow default configuration path;
the observed duration alone does not prove the exact flash clock. The full
build now specifies SPIx4, 51 MHz configuration rate, and compression in
`synth/tqtt_fpga_top.xdc`, matching the standalone probes. That change is for
future generated images and has not yet been tested by a physical power cycle.

At a later Finder desktop check, the same flash-booted image still showed the
SD volume enabled with vHDD IDENT `0x5D0D0001`, no vHDD watchdog fires, and
2,522 SCSI-to-SD completions with zero SD errors and no captured first error.
The native High Speed probe remained asserted. The fan probe reported ADC code
635 (about 35.6 °C), duty 140/255, and tach 3,780 RPM. HDMI capture still
showed a live Finder desktop. These are continued-run health checks, not a
power-on boot test or a sustained disk integrity workload.

A bitstream-only rewrite of the same routed 200 MHz checkpoint with SPIx4,
51 MHz configuration rate, and compression passed Vivado bitstream DRC with
zero errors. Its blank `.bit` shrank from 15,431,259 to 12,690,253 bytes.
`updatemem` accepted the compressed file and produced the local ADB-patched
`/var/tmp/tqtt_c96_waitfix_200_bootopt/vivado/fpga_top.local.bit` (12,720,849
bytes, SHA-256
`1c57d1e2903f0aa5b8780169b40fbc7fa054be32540df482f0451e2cae08b931`).
This file was loaded over JTAG first and reached Mac OS with vHDD IDENT
`0x5D0D0001`, native SD High Speed, zero SD errors, and no framebuffer misses.

The same file was then written to TQTT SPI flash using the command above.
The detected Micron `0x20bb18` device was erased, programmed through
`0x00c30000`, and fully read back with `--verify`; openFPGALoader exited
successfully. Without a separate `boot_hw_device` command or SRAM load, a
fresh Vivado session read `REGISTER.CONFIG_STATUS=0x109079fc`, `DONE_PIN=1`,
`EOS=1`, `GWE=1`, one JTAG AXI core, and one VIO core. The REPL attached with
`JTAG_REPL_NO_PROGRAM=1` and read build ID `0x8D3A363A`, vHDD IDENT
`0x5D0D0001`, 4,194,240 SD blocks, native High Speed active, 2,187 SCSI-to-SD
completions, zero SD errors, zero CHECK CONDITIONs, and no vHDD watchdog fire.
Video was live at 640x480 with matching framebuffer requests/responses and no
misses. The fan was in automatic control at about 34 °C. HDMI capture showed
the [Finder desktop](evidence/2026-10-25-tqtt-200mhz-compressed-flash-auto-boot.jpg)
after dismissing the normal improper-shutdown dialog.

This proves a successful configuration start after flash programming for this
image. A physical power-cycle boot is still untested. Since compression,
SPI bus width, configuration mode, and rate changed together, this comparison
does not identify which setting enabled automatic startup.
