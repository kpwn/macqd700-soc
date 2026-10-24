# TQTT-KU5P bring-up

This board uses the same `xcku5p-ffvb676-2-i` FPGA as the RK target, but its
clock, DDR4, SD, Ethernet, and video wiring differ. `BOARD=rk` remains the
default. `BOARD=tqtt` now selects the full Mac SoC integration; its hardware
boot and 200 MHz timing closure remain in progress. The standalone SD/CRC
image is still available for board diagnostics.

## Board direction

TQTT is the primary target for the full Mac and for new physical USB, ADB,
and serial interfaces. RK remains a supported board. The default build can
move to `BOARD=tqtt` after a full TQTT image boots and meets the 200 MHz core
timing requirement. Until then the working RK default remains usable.
RK video parity is a later hardware task: carry TMDS through its QSFP port.
That path needs its own electrical and timing validation.

## First peripheral: SD card

Build with `make BOARD=tqtt fpga-sdmin-bitstream`. This uses the TQTT 100 MHz
USER1_CLK pair at T24/U24 and the board's SD slot at AE15/AD15/AF13/AF14.
The test reads 2048 sectors through the existing boot FSM and checks CRC16;
it does not write to the card. Its fast SPI clock is 25 MHz, and the init
clock is about 400 kHz. POR and JTAG VIO supply reset/control. The first
image does not drive the board's adjustable-rail reset or LEDs.

Artifacts are in `build/vivado_sdmin_tqtt/`. Once JTAG enumerates and the
board's rail voltages have been checked, load the `.bit` and `.ltx` pair with
`synth/program_bitstream.tcl`. `tools/jtag_repl_sdmin.tcl` can then read the
card status, last command, CRC values, and final image CRC over VIO. Compare
its image CRC with the expected card image before trusting a full boot.

First board run (2026-10-24): Vivado routed the image with zero DRC errors,
setup slack +0.593 ns and hold slack +0.012 ns. The actual JTAG target was
`xcku5p_0`. VIO reported `sector=2048`, `rom_loaded=1`, `error=0`,
`ctrl_retry=0`, and final sector CRC16 `0x1270 == 0x1270`. The image CRC32 was
`0xff52a648`; its expected value has not yet been checked against the card
image on the host. This proves the SD read path and CRC16 on this board, not
the contents of a particular ROM image.

The TQTT schematic's USB-C port feeds a USB2513 hub with FT232H JTAG and
CP2105 UART bridges. They should enumerate in `lsusb` before Vivado can find
a target. The SD image cannot be loaded while those devices are absent.

## Ethernet receive probe

Build with `make BOARD=tqtt fpga-tqtt-eth-probe-bitstream`; use
`tools/tqtt_eth_probe_status.tcl` to program it and sample the JTAG VIO
counters. This read-only GMII probe drives the required 125 MHz GTX clock and
releases PHY reset after the MMCM locks. It holds TXEN and TXER low. The
error counter only counts RXER during a valid receive frame.

With the board connected to the host's USB Ethernet adapter, the PHY
negotiated 1000 Mb/s full duplex. The sampled RX clock had 250,415,572 edges
in 2.003 s, or 125.02 MHz, and the frame count increased during host
transmission. The revised probe routed with zero DRC errors. After the link
settled, VIO saw 250,494,346 RX clock edges and one receive frame in 2.005 s,
with **zero in-frame RXER cycles**. The initial version counted RXER during
idle and could not distinguish real frame errors.

The transmit probe is `make fpga-tqtt-eth-tx-probe-bitstream`. It uses the
same pins and 125 MHz forwarded GTX clock, sends a broadcast test frame with
an Ethernet FCS every second, and exposes a Gray-coded TX count in VIO probe
3. The TX data changes on the falling clock edge for setup before the PHY's
next rising edge. On 2026-10-24 it routed with zero DRC errors, setup slack
+0.614 ns and hold slack +0.016 ns. JTAG counted two transmitted frames in
2.004 s and zero in-frame RX errors. The host's RTL8153 adapter negotiated
1000 Mb/s full duplex and `tcpdump` captured three consecutive frames with
source MAC `02:54:51:54:54:01`, EtherType `0x88b5`, the `TQTT` marker, and
incrementing frame IDs. The adapter reported `rx_errors=0` and
`tx_errors=0`. This proves standalone GMII TX and RX wiring; the Mac's shared
Taxi MAC still needs a GMII board variant. The HDMI audio image was restored
after testing.

## HDMI sound

HDMI audio is carried in data islands on the same three TMDS pairs as video;
it consumes no expansion-header pins. `rtl/board/audio_hdmi_bridge.v` is only
a formatter skeleton, and its outputs are currently disconnected from the
video scanout. The native ASC sample stream is around 22 kHz, while an HDMI
source should supply a supported audio rate such as 48 kHz.

The standalone HDMI probe uses the MIT-licensed `hdl-util/hdmi` packetizer
and encoder with a local UltraScale+ `ODDRE1` serializer. Build its 640x480p60
test pattern with `make BOARD=tqtt TQTT_HDMI_AUDIO=0
fpga-tqtt-hdmi-probe-bitstream`; set `TQTT_HDMI_AUDIO=1` to send real HDMI
packets with a quiet 48 kHz stereo 1 kHz test tone. The pixel clock is 25.2
MHz and the serializer clock is 126 MHz. The connector's four differential
pairs use `DIFF_SSTL12` on the board's 1.2 V bank. The video-only image routed
with 0 DRC errors, setup slack +2.866 ns, and hold slack +0.026 ns. It was
programmed over JTAG on 2026-10-24. The physical monitor displayed video,
and the USB capture card showed all eight color bars cleanly at 640x480.
The MJPEG server at `http://10.200.0.12:8080/` served fresh frames from this
source. The capture card briefly failed USB format negotiation with `EBUSY`;
rebinding its dedicated xHCI controller recovered the device.

The audio-enabled image also routed with zero DRC errors and positive setup
and hold slack (+3.161 ns and +0.042 ns). Its generated 48 kHz clock is
explicitly constrained. A 2-second 48 kHz stereo recording from the capture
card had sample values from -4096 to +4096 and rising edges every 48 samples,
confirming the 1 kHz tone in both matching channels. The same capture path
recorded all zeros with the video-only image. This test source does not yet
carry Mac ASC sound. That integration needs rate
conversion or sample holding and a clock-domain crossing into the HDMI
packetizer. The attached capture card has a USB audio endpoint for validation.

## DDR4 IP configuration

The TQTT schematic and DDR4 example identify two `MT40A256M16GE-075E`
devices, a 32-bit bus, 1 GiB total, and a 100 MHz reference clock. Generate
the board-specific MIG with `make tqtt-ddr4-mig-validate` or synthesize its
out-of-context DCP with `make tqtt-ddr4-mig`. These use
`build/ddr4_mig_tqtt/` and `synth/tqtt_ddr4.xdc`; the RK MIG and pin map are
separate. Vivado 2025.2 accepted the TQTT part, 100 MHz input, 256-bit AXI
data bus, and 30-bit AXI address bus on 2026-10-24. The 31-bit RK AXI address
setting is invalid for this 1 GiB memory geometry. MIG configuration alone
does not establish hardware calibration or memory integrity; those require a
standalone controller test on the board. The generated TQTT stub has 30-bit
AXI address ports; the checked-in RK stub has 31-bit ports, so the TQTT shell
must select its own stub when the full SoC is integrated. Out-of-context MIG
synthesis completed with no errors and produced the TQTT DCP.

Build the standalone hardware exercise with
`make fpga-tqtt-ddr4-probe-bitstream`. It stitches the TQTT MIG DCP into
`rtl/soc/tqtt_ddr4_probe.v`, using `synth_design -no_iobuf` so the MIG owns
the DDR pins and an explicit input buffer for the external reset switch.
The probe writes 4096 contiguous 256-bit words, then 4096 more at 256 KiB
spacing across the 1 GiB range, with distinct patterns. It reads all 8192
locations back and reports calibration, phase, count, and first error through
JTAG VIO. `tools/tqtt_ddr4_probe_status.tcl` programs the image and checks
for calibration, pass phase, all 8192 reads, and no AXI/data error.

On 2026-10-24 the expanded probe routed with zero DRC errors, setup slack
+0.078 ns and hold slack +0.013 ns. Hardware VIO reported `cal_done=1`,
`phase=5` (pass), `verified=0x2000` (8192), and `error_code=0`. This checks
the sampled addresses and all 256 data bits at each address; it does not
exercise every byte of DRAM. The HDMI audio bitstream was restored after the
test, and the capture feed resumed.

## Full SoC integration in progress

`BOARD=tqtt` now selects the TQTT 1 GiB DDR4 MIG, 8-bit GMII Taxi MAC,
direct TMDS pins, SD SPI, UART2, LEDs, and fan. The 50 MHz E18 oscillator
drives one fabric MMCM that produces a 200 MHz core clock, 50 MHz peripheral
clock, 100 MHz video reference, and 125 MHz GMII transmit clock. The 100 MHz
T24/U24 differential clock remains dedicated to MIG. The existing Mac scanout
runs at 720p60 and feeds a DVI-compatible TMDS encoder. HDMI audio from the Mac
ASC is still to be connected; the standalone 48 kHz audio probe remains available.

The RK physical DDR layout reserves its ROM and framebuffer above the first
1 GiB. That would wrap on TQTT's 30-bit MIG AXI address bus and corrupt RAM.
The TQTT build therefore decodes 512 MiB of guest RAM, places the 4 MiB ROM
at DDR `0x3F00_0000`, and places the 8 MiB framebuffer at `0x3F40_0000`.
`make check-tqtt-ddr-map` verifies that the ranges are disjoint and fit in
1 GiB. The guest-visible ROM and framebuffer addresses remain unchanged.

On 2026-10-24, `BOARD=tqtt TARGET_FREQ_MHZ=200 CORE_CLK_HZ=200000000
CORE_MMCM=1 USE_REAL_MIG=1 make vivado-dry-run` elaborated the full 68040
SoC with zero errors and zero critical warnings. `make tqtt-full` is the
200 MHz implementation command, using the reduced CPU debug profile and a
separate TQTT output directory. A successful elaboration does not establish
timing closure or a Mac boot; those are the next hardware gates.

`make tqtt-full-100` builds a separate 100 MHz functional bring-up image. Its
MMCM physically generates 100 MHz for the core, with a matching timing budget.
The 200 MHz target remains `make tqtt-full`.

The first full route passed synthesis, placement, DDR MIG stitching, and
routing, but missed the 200 MHz core budget by 0.682 ns. It also exposed
missing SD pad constraints. The second route, using `PLACE_DIRECTIVE=Explore`,
generated `build/vivado_tqtt/fpga_top.blank.bit` with zero DRC errors,
WNS -0.304 ns and WHS +0.002 ns after six post-route passes. The true core
clock is 5 ns. SD pad paths now close: CLK +0.104 ns, MOSI +0.284 ns,
CS +0.228 ns, and MISO +0.416 ns. The core image remains below the requested
200 MHz timing margin.

The second image was programmed on the KU5P via JTAG on 2026-10-24. VIO
reported DDR calibration complete, ROM load complete, core reset released,
no AXI error, and matching final-sector CRC16 `0x1270 == 0x1270` after
2048 sectors. JTAG CPU debug showed the core running without an exception,
but repeatedly visiting the ROM ADB manager wait loop at `0x4080A8E6`.
The capture card showed a gray rectangle on black instead of the Mac desktop.
This is an incomplete boot, not a validated machine. The loop is also
documented in `tb/models/rom_patch_sets.h`; a 100 MHz build is underway to
check whether the same behavior persists without the 200 MHz setup miss.
`tools/tqtt_full_status.tcl` reads all live VIO probes without reprogramming.

Before loading the full image, check the TQTT +ADJ rail that supplies bank 67
and the E18 oscillator input. The board reference warns that +ADJ should be
1.71-1.89 V, while its power page specifies 1.9 V. The 50 MHz oscillator is
connected through a 43 ohm series resistor. This clock path was not used by
the earlier DDR, SD, HDMI, or GMII probes, which used T24/U24 instead.
AMD's [DS922 recommended operating conditions](https://docs.amd.com/r/en-US/ds922-kintex-ultrascale-plus/Recommended-Operating-Conditions)
limit HP-bank VCCO to 1.9 V and input voltage to VCCO + 0.2 V.

Complete the 100 MHz comparison and diagnose the ADB wait, then close the
200 MHz core timing, confirm the 720p60 Mac screen, and verify full-SoC
DDR/SD and GMII RX/TX. HDMI audio and physical USB/ADB/serial follow.
RK remains selectable as `BOARD=rk` until the TQTT full image passes these
checks.

Board pin facts were checked against the TQTT XCKU5P schematic and example
constraints at <https://github.com/joshheyse/tqtt-ku5p>. No vendor example
RTL was copied into this repository.
