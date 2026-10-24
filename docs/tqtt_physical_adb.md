# Physical ADB on TQTT PMOD3

The TQTT PMOD3/J12 header can carry the logic side of a real Apple Desktop
Bus connector. Its eight FPGA signals pass through U21 TXS0108E from bank 66
(+ADJ) to a 3.3 V header. The assembled board powers the translator's header
side at 3.3 V. It is not a 5 V ADB port by itself.

Apple's ADB data line is a single bidirectional open-collector line with a
470 ohm host pull-up to 5 V. The connector also provides 5 V device power.
The PMOD3 signal pins must stay on the 3.3 V side of an external interface;
neither the raw 5 V data line nor ADB power goes to the header.

## Candidate two-pin interface

Reserve PMOD3 signal index 0 (J12 pin 1, KU5P K22) for `adb_drive_n`
and signal index 1 (J12 pin 3, KU5P K23) for `adb_bus_sense`.

- Drive: a 3.3 V powered SN74LVC1G07 open-drain buffer takes
  `adb_drive_n` as its input and sinks the 5 V ADB data line only when
  its input is low. Its open-drain output is rated for up to 5.5 V and
  32 mA sink current. The ADB 470 ohm pull-up needs about 10.6 mA at 5 V.
  Ensure the input defaults high while the FPGA is unconfigured so the bus
  is released at startup.
- Sense: a 3.3 V powered SN74LVC1G17 Schmitt buffer takes the ADB data
  line at its 5.5 V tolerant input and drives a 3.3 V signal into PMOD3.
  A passive high-value divider alone is not a good match for the TXS0108E's
  internal pull-ups.
- Provide a current-limited 5 V supply and common ground to a 4-pin mini-DIN
  ADB socket. ADB connector pin 1 is data, pin 3 is +5 V, pin 4 is ground;
  pin 2 is the power-on signal and needs a separate design if required.

This is a candidate bench circuit, not a validated schematic. Check startup
levels, idle 5 V at the connector, low-level sink voltage/current, and
waveform timing before connecting a keyboard or mouse.

Board pin source: <https://github.com/joshheyse/tqtt-ku5p/blob/main/pins.xdc>.
ADB electrical source: <https://vintageapple.org/inside_o/pdf/Guide_to_Macintosh_Family_Hardware_2nd_Edition_1990.pdf>.
Buffer sources: <https://www.ti.com/lit/ds/symlink/sn74lvc1g07.pdf> and
<https://www.ti.com/lit/ds/symlink/sn74lvc1g17.pdf>.
TXS0108E source: <https://www.ti.com/lit/ds/symlink/txs0108e.pdf>.

## Full Mac RTL path

`fpga_top_peripherals.vh` currently connects the real ADB PIC firmware's
`pic_adb_out` to `adb_phy`, which generates simulated keyboard and mouse
responses and drives `adb_bus_in` back to the PIC. Physical ADB mode should
drive the external open-drain buffer from the PIC's release signal and feed
the buffered physical line back to `adb_bus_in`. The simulated responders
must be disabled in this mode; a real keyboard or mouse will otherwise
collide with the virtual device at its default ADB address. Keep the
existing simulation path for builds without a physical ADB connector.

Bring-up order: standalone PMOD3 pad/level probe, scope the ADB reset and
command waveforms, receive a real device response, then add a board build
option and boot the Mac with physical ADB selected. Preserve web/JTAG
injection only where it does not produce a duplicate ADB device.
