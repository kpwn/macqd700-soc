# TQTT-KU5P bring-up

This board uses the same `xcku5p-ffvb676-2-i` FPGA as the RK target, but its
clock, DDR4, SD, Ethernet, and video wiring differ. `BOARD=rk` remains the
default. `BOARD=tqtt` currently selects the **standalone, read-only SD/CRC
image**; the full SoC build rejects this board until its other interfaces are
qualified. A successful SD image is not a Mac boot image.

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

## HDMI sound

HDMI audio is carried in data islands on the same three TMDS pairs as video;
it consumes no expansion-header pins. `rtl/board/audio_hdmi_bridge.v` is only
a formatter skeleton, and its outputs are currently disconnected from the
video scanout. The native ASC sample stream is around 22 kHz, while an HDMI
source should supply a supported audio rate such as 48 kHz. Bring up video
first, then add a true HDMI packet scheduler with ACR and Audio InfoFrames,
rate conversion or sample holding as appropriate, and verify the result via
the attached capture card's USB audio device. The dual MIT/Apache-2.0
`hdl-util/hdmi` project is a possible packetizer reference, but its Xilinx
serializer uses Series 7 `OSERDESE2` and must be adapted for UltraScale+.

## Remaining gates before the full SoC

1. **HDMI:** adapt the existing pixel scanout to three TMDS encoders and
   serializers. The connector's four pairs are on bank 65, which the board
   pin notes identify as 1.2 V. The vendor example calls them `LVDS`, but
   AMD UG571 specifies 1.8 V VCCO for LVDS output. Resolve the actual rail,
   legal I/O standard, sink electrical margin, DDC, and HPD before loading
   an HDMI image. Start with a 720p60 pattern and preserve the 74.25 MHz
   pixel clock used by the current SoC.
2. **Ethernet:** this board's RTL8211EG exposes GMII (8-bit), whereas the RK
   board's RTL8211F uses RGMII (4-bit). It needs a GMII PHY adapter and its
   own pin/timing constraints; changing only the pins would miswire data.
3. **DDR4 and clocks:** generate a TQTT-specific MIG for its 32-bit DDR4
   wiring and clock topology, prove calibration and AXI memory tests, then
   retime the SoC's core/peripheral/video domains. Do not reuse the RK MIG
   DCP or its XDC.
4. **Full Mac:** select the TQTT shell with `BOARD=tqtt`, boot from the SD
   image, verify video and Ethernet, then close timing at the required
   200 MHz core clock. Keep the RK shell as `BOARD=rk`.

Board pin facts were checked against the TQTT XCKU5P schematic and example
constraints at <https://github.com/joshheyse/tqtt-ku5p>. No vendor example
RTL was copied into this repository.
