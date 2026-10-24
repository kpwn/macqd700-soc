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

Maximum reliable SD performance is also a TQTT completion requirement:
measure the current Mac disk, use the board's native four-bit interface,
validate the fastest stable clock with CRC and host-side sector comparisons,
and report end-to-end Mac throughput. See [TQTT SD performance plan](tqtt_sd_performance_plan.md).

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
it consumes no expansion-header pins. `rtl/board/audio_hdmi_bridge.v` is a
formatter skeleton and is not the packet source for the full Mac build. The
native ASC sample stream is around 22 kHz, while an HDMI
source should supply a supported audio rate such as 48 kHz.

The standalone HDMI probe uses the MIT-licensed `hdl-util/hdmi` packetizer
and encoder with a local UltraScale+ `ODDRE1` serializer. Build its 640x480p60
test pattern with `make BOARD=tqtt fpga-tqtt-hdmi-probe-bitstream`; it sends
real HDMI packets with a quiet 48 kHz stereo 1 kHz test tone by default.
Set `TQTT_HDMI_AUDIO=0` for the video-only probe. The pixel clock is 25.2
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
recorded all zeros with the video-only image. This standalone test source
does not carry Mac ASC sound. The attached capture card has a USB audio
endpoint for validation of the full Mac option.

The full Mac build enables HDMI audio by default with `make tqtt-full` and
uses `build/vivado_tqtt_hdmi_audio`. Set `TQTT_HDMI_AUDIO=0` for the
DVI-compatible 720p output. The HDMI audio path uses the ASC's stereo PCM
stream, repeats samples to make a
48 kHz HDMI stream, and uses the standalone probe's HDMI packet components.
The first 100 MHz full-Mac HDMI audio build in
`build/vivado_tqtt_100_hdmi_audio` completed routing on 2026-10-24 with
WNS +0.001 ns, WHS +0.009 ns, WPWS +0.181 ns, and no failing timing
endpoints. Its bitstream passed DRC, was patched with the local ADB firmware,
and was programmed on the KU5P. JTAG confirmed a running CPU, matching ROM
load CRC16 values, and the SD volume enabled at 4,194,240 blocks without a
volume error or watchdog fire. Its captured HDMI frame was black because
the packetizer coordinates were five pixel clocks ahead of RGB and DE.

On 2026-10-25 the capture path returned the same uniform black JPEG after
two loads of that full HDMI image, although JTAG sampled nonblack Mac
scanout RGB and reported completed framebuffer reads. The 100 MHz DVI full
image booted to the Mac improper-shutdown dialog on the same cable and
capture card. A standalone 720p probe in
`rtl/soc/tqtt_hdmi_integration_probe.sv` exercises the exact full-Mac
`tqtt_hdmi_audio_video` transmitter. Both its direct 100 MHz clock and
its TQTT 50 MHz fabric-MMCM clock variants showed color bars and produced
a captured stereo tone near 1 kHz. With the latter clock chain unchanged,
delaying the probe's RGB/DE/HS/VS five pixel clocks behind its coordinates
produced the **byte-identical black JPEG** seen from the full image.
`video_top`'s debug coordinates are only two output clocks behind the VTG,
while RGB/DE/HS/VS pass through the compositor's four-stage control pipe,
its output register, and those same two output clocks. The full-Mac HDMI
connection now delays coordinates by five additional clocks. The corrected
100 MHz image in `build/vivado_tqtt_100_hdmi_audio_align5` routed with
WNS +0.096 ns, WHS +0.010 ns, WPWS +0.219 ns, and zero DRC errors. After
patching in the local ADB firmware, it was programmed on 2026-10-25 and
captured the boot icon, Mac OS improper-shutdown dialog, and Finder desktop
with `FPGA HD` mounted. JTAG Return dismissed the dialog. The SD volume was
enabled at 4,194,240 blocks, with no error or framebuffer fetch miss.
The capture card recorded silence at the idle Finder desktop. The classic
`Sound` control panel opened and showed its alert list, but injected mouse
clicks did not visibly change the selected alert; recordings during these
attempts were also silent. A controlled alert sound test remains needed.
During a warm reset, the capture card recorded a roughly 0.29-second stereo
burst beginning about 2.9 seconds after reset. Both channels contained
nonzero samples (peak magnitude 32,512), unlike idle silence, so the Mac
audio source reaches the HDMI capture path. The waveform had a large negative
DC offset; audible chime quality and the ASC sample conversion still need
checking. After reset, Mac OS returned to the improper-shutdown dialog and
ADB Return dismissed it.

Build the matching-clock control with
`make TQTT_HDMI_INTEGRATION_PROBE_FABRIC=1 fpga-tqtt-hdmi-integration-probe-bitstream`. Add
`TQTT_HDMI_INTEGRATION_PROBE_SKEW=5` to reproduce the old coordinate
offset. The targets use separate output directories, and programming
either probe replaces the full Mac image until it is loaded again.

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
runs at 720p60 and feeds the ASC-to-HDMI transmitter by default. The
full-image HDMI audio path has routed and been programmed at 100 MHz, and
its video output has been captured through Mac OS boot to Finder. ASC audio
playback remains to be checked with an audible Mac sound event. The standalone
48 kHz audio probe remains available.

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
200 MHz timing margin. The routed report counts 1,307 failing core endpoints;
its worst path runs from `LsEuPlugin`'s aligned-send pointer to a `DcachePlugin`
dirty-bank register through 19 logic levels. Of the 5.302 ns data delay,
3.831 ns is routing. This is a CPU core path rather than an SD or HDMI pad
path.

The second **blank** image was programmed on the KU5P via JTAG on 2026-10-24. VIO
reported DDR calibration complete, ROM load complete, core reset released,
no AXI error, and matching final-sector CRC16 `0x1270 == 0x1270` after
2048 sectors. JTAG CPU debug showed the core running without an exception,
but repeatedly visiting the ROM ADB manager wait loop at `0x4080A8E6`.
The capture card showed a gray rectangle on black. This was caused by
programming the blank bitstream: the ADB PIC BRAM had no firmware. Full Mac
images must be patched with the local Apple ADB ROM before programming, as
described in [ADB firmware setup](adb_firmware_bitstream.md).

Patch the matching TQTT bundle with `tools/patch_adb_bitstream.py patch`
using the build's `.blank.bit`, `.blank.adb.mmi`, and `.blank.adb.json`, plus
your own `342s0440-b.bin`; program the resulting `.local.bit` with the build's
`.ltx`. The local firmware and patched image stay outside version control.
With the patched 200 MHz image programmed, the Mac reached the OS improper-
shutdown dialog. A Return key sent through the web UI dismissed it, and the
capture card showed the Mac desktop with mounted `FPGA HD` and the
Speedometer folder. JTAG `vhdd-status` reported the SD volume enabled at
4,194,240 blocks (about 2 GiB), with no RAM-disk watchdog or error. This
establishes a working full Mac boot and basic ADB input despite the remaining
-0.304 ns core setup miss. The separate 100 MHz build routed with WNS
+0.035 ns, WHS +0.010 ns, and WPWS +0.235 ns, with zero failing endpoints and
all user timing constraints met. Its blank bitstream and matching ADB metadata
were generated, then patched locally with the user's ADB firmware to
`build/vivado_tqtt_100/fpga_top.local.bit`. That earlier 100 MHz image was
not loaded or functionally tested. A second 200 MHz placement from the
original synthesis checkpoint is implemented with `ExtraPostPlacementOpt`
placement and `AggressiveExplore`
physical optimization/routing. Its final routed report has WNS -0.189 ns,
WHS +0.007 ns, 538 failing setup endpoints, zero failing hold endpoints, and
zero DRC errors. The new worst setup path is from a Dcache load-valid flop to
a Dcache valid-bank register through 22 logic levels. This improves the
earlier 200 MHz WNS by 0.115 ns but still does not close timing. Its blank
bitstream was generated and locally patched with the user's ADB firmware to
`build/vivado_tqtt_extra/fpga_top.local.bit`; it has not been loaded or
functionally tested. A guarded `AggressiveExplore` post-route physical pass
from that route checkpoint improved WNS to -0.092 ns with WHS 0.000 ns but
still missed 200 MHz setup; the pass saved a separate checkpoint and did
not generate a bitstream. A following `AlternateReplication` pass improved
WNS to -0.076 ns with WHS 0.000 ns and 395 failing setup endpoints, again
without timing closure or a bitstream. Its worst path is from the CPU fetch
alignment PC register to the gshare retained-repair suffix register. The
original patched image was active at the time of that test.

On 2026-10-24, the 100 MHz full-Mac HDMI audio image described above became
the active FPGA image, with its matching `.ltx` attached to the JTAG REPL.
The REPL reported build ID `0x8B8EE5FE`, a running CPU, and the expected SD
volume without an error. The host Ethernet link is up, but this boot did not
answer at `192.168.42.11`; a short DHCP/ARP capture saw no packets. The
capture server returned black frames. These observations do not establish
an HDMI audio playback or a completed Mac OS boot. The image uses reduced
debug, so `eth-status` telemetry is absent even though `ETH_ENABLE=1` was
included in synthesis.

The full-SoC GMII link also passed a Mac OS TCP/IP check on 2026-10-24. A
host USB Ethernet adapter connected directly to the KU5P negotiated 1000
Mb/s full duplex. With `192.168.42.1/24` on the host adapter and a DHCP
server bound only to that interface, the Mac's SONIC address
`00:05:02:00:00:00` completed Discover/Offer/Request/ACK and received
`192.168.42.11`. Five host ICMP echo requests received five replies, with
0% loss and 0.236 ms average round trip. This exercises full-SoC receive,
DMA, driver, and transmit paths. The Mac's Netscape browser then fetched
`http://192.168.42.1:8000/` over that direct link; the host server logged
`GET / HTTP/1.0` from `192.168.42.11` with status 200. This validates a
real TCP and HTTP application path through the full SoC as well.

For the current 640x480 Mac mode, run the MJPEG server with
`FPGA_VIDEO_WIDTH=1280 FPGA_VIDEO_HEIGHT=720 FPGA_VIDEO_FPS=60
FPGA_VIDEO_CROP=auto FPGA_VIDEO_SRC_W=640 FPGA_VIDEO_SRC_H=480`. Capturing
the 720p HDMI frame directly at 640x480 squeezes its centered 640x480 Mac
window into a square. The full-resolution capture plus crop restores its
4:3 shape in the web UI; the physical HDMI signal still carries the Mac
window at native size within the 1280x720 raster. Change the crop dimensions
if the Mac switches video mode.
`tools/tqtt_full_status.tcl` reads all live VIO probes without reprogramming.

### J10 four-wire fan control

Schematic page 4 shows a 12 V four-wire fan header. Q9 is an open-drain
inverter from FPGA E15 (`fan_pwm`) to the fan's PWM pin; FPGA low releases
the line and runs the fan at full speed. G17 (`fan_fb`) senses tach through
the board's 1.8 V pull-up. The previous full image tied E15 low, explaining
the constant full-speed noise. The new TQTT controller uses approximately
25 kHz PWM, runs at full speed for the first second after clock lock, then
defaults to 50% duty. JTAG VIO `probe_out2` can change duty live via
`fan-duty 50` in the JTAG REPL (25-100%; 100 releases the line). The
`vio_fan_rpm` input reports tach-derived RPM once per second, assuming two
tach pulses per revolution. Until temperature sensing and thermal testing
are in place, the RTL clamps duty to at least 25%; MMCM clock loss releases
the fan control line for full speed. The 200 MHz fan image at
`build/vivado_tqtt_fan/fpga_top.local.bit` was loaded with its matching
`fpga_top.ltx` on 2026-10-24 and booted Mac OS. At 50% duty, its tach
reported about 3,450 RPM and the die reached 35.9 °C during the initial
observation.
`tools/tqtt_fan_status.tcl` reads the FPGA die temperature and, when present,
the VIO RPM and duty probes without reprogramming. The full-speed baseline
on 2026-10-24 was 32.4 °C. This is die temperature, not the fan's airflow or
the hottest board component.

The desired automatic-control ceiling is 50 °C FPGA die temperature. Increase
duty before that ceiling and command full speed at or above 50 °C. The present
loaded image is manual. An opt-in automatic image can be built with
`make TQTT_FAN_AUTO=1 tqtt-full` in `build/vivado_tqtt_fan_auto`. It reads
the on-chip SYSMON temperature through the fabric DRP. KU5P's SYSMONE4 ADC
codes 600 to 664 (approximately 18–50 °C with the internal reference) map to
0–100% duty in steps of four PWM counts. Missing or implausible readings and a
tach stall command full speed. The board has not run this automatic image yet.
Its sensor conversion, tach fault behavior, and acoustic curve require hardware
validation before making it the default. In that image, `fan-duty 100` forces
full speed and `fan-duty auto` releases the override; the manual image keeps
its 25–100% JTAG range. `make test-tqtt-fan-thermal` checks the curve, stale
and invalid readings, and tach-stall fallback without loading a bitstream.
Validate RPM and temperature across idle and sustained Mac workloads before
calling the fan bring-up complete.

Before loading the full image, check the TQTT +ADJ rail that supplies bank 67
and the E18 oscillator input. The board reference warns that +ADJ should be
1.71-1.89 V, while its power page specifies 1.9 V. The 50 MHz oscillator is
connected through a 43 ohm series resistor. This clock path was not used by
the earlier DDR, SD, HDMI, or GMII probes, which used T24/U24 instead.
AMD's [DS922 recommended operating conditions](https://docs.amd.com/r/en-US/ds922-kintex-ultrascale-plus/Recommended-Operating-Conditions)
limit HP-bank VCCO to 1.9 V and input voltage to VCCO + 0.2 V.

Close the 200 MHz core timing and run full Mac benchmarks and storage integrity
checks. Confirm full-Mac HDMI audio and bring up physical USB/ADB/serial.
RK remains selectable as `BOARD=rk` until the TQTT full image passes these
checks.

Board pin facts were checked against the TQTT XCKU5P schematic and example
constraints at <https://github.com/joshheyse/tqtt-ku5p>. No vendor example
RTL was copied into this repository.
