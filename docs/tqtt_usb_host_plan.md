# TQTT USB host path for Mac input

## Hardware boundary

The board's USB-C connector is the upstream connection to the USB2513 hub,
FT232H JTAG, and CP2105 UARTs. It is not an FPGA-controlled downstream host
port. A Mac keyboard or mouse needs an external USB-A host front end on an
expansion header.

PMOD0 (J1) and PMOD1 (J2) are direct 3.3 V FPGA interfaces in bank 86.
PMOD2 (J3) is another direct 3.3 V interface in bank 84. PMOD3 (J12) passes
through a TXS0108E level translator and should not be the first choice for
USB D+/D-. The board reference maps PMOD0 signal indices 0..3 to KU5P
package pins D10, D9, C9, and E11; header pins 1, 3, 5, and 7 respectively.
The USB data pins and socket mapping must be checked against the selected
host PMOD's schematic before adding the XDC. The USB-A socket also needs a
5 V VBUS supply, current limiting, common ground, and suitable line protection.
The nand2mario dual-socket USB host PMOD includes a 3.3 V-to-5 V boost
converter, USB-A sockets, and D+/D- front ends; its KiCad files are the
reference for the PMOD pinout and power budget.

Board reference: <https://github.com/joshheyse/tqtt-ku5p/blob/main/pins.xdc>.
Host PMOD: <https://github.com/nand2mario/usb_host_pmod>.

For a temporary single-port bench setup on PMOD0/J1, reserve header pin 1
(D10) as USB D+ and pin 3 (D9) as USB D-. Put approximately 22 ohms in
series with each data wire and 15 kohm from each USB-side data line to
ground. Supply the USB-A socket's VBUS from a current-limited 5 V source and
tie its ground to the PMOD ground on pin 9 or 11. The PMOD's pins 10 and 12
carry 3.3 V and must not be wired to USB VBUS. Use short data wires; the
standalone probe will confirm attach, enumeration, and reports before any
full-SoC build depends on this wiring. The second port can use PMOD0 signal
indices 2 and 3 after the first works.

## First useful feature: two direct HID ports

Use two instances of the m1nl full/low-speed HID host, one per USB-A socket.
It supports 1.5 and 12 Mbit/s keyboard and mouse devices, presents decoded
reports, and requires no guest Mac USB driver. It needs a 60 MHz clock in
full-speed mode. Add a separate MMCM output and synchronize reports into
the 50 MHz peripheral domain. Keep the host instances and PMOD pins behind
a TQTT-only build switch so RK's existing input path stays available.

The Mac should see ordinary ADB devices. Convert HID keyboard usage/modifier
sets into ADB make/break keycodes, including release events on disconnect.
Convert HID mouse button and signed relative movement into the existing
`adb_keyboard` and `adb_mouse` injection inputs. Add an arbiter and bounded
event queue so USB and the existing JTAG/web MMIO injector can both submit
events without losing simultaneous pulses. The existing
`rtl/mac/adb_inject.v` word-aligned aliases offer a reference for the event
format, but a fabric USB source should feed the arbiter directly.

Source: <https://github.com/m1nl/usb_hid_host>. The original
<https://github.com/nand2mario/usb_hid_host> only covers low-speed devices
and lists hub/full-speed support as future work.

Validation order:

1. Standalone PMOD probe: 60 MHz clock, attach/detach, enumerate one known
   keyboard and one known mouse, expose type/report/error counters in VIO.
2. Verify simultaneous reports from both sockets and clean release on unplug.
3. Integrate the HID-to-ADB bridge, check keys and mouse motion on the Mac
   desktop, and verify web/JTAG injection still works.
4. Route and boot the full TQTT image, then check 200 MHz timing and SD/DDR
   integrity with the USB logic enabled.

The first single-port probe is `make fpga-tqtt-usb-host-probe-bitstream`.
It uses PMOD0/J1 pin 1 for D+ and pin 3 for D-, derives 60 MHz from the
board's 100 MHz differential reference, and exposes five VIO inputs:

| Probe | Width | Contents |
| --- | ---: | --- |
| 0 | 32 | report count [31:16], type [7:6], error [5], busy [4], report toggle [3], D+ [2], D- [1], output enable [0] |
| 1 | 64 | last raw HID report |
| 2 | 32 | last modifier byte and first three keyboard usages |
| 3 | 32 | last mouse buttons and signed X/Y deltas |
| 4 | 64 | host diagnostic registers |

The probe bitstream is separate from the full Mac image. Do not program it
while preserving the running Mac session matters; FPGA reconfiguration resets
that session. When bench wiring is ready, program the probe with its matching
`.ltx` file, inspect attach/enum data with `tools/tqtt_usb_probe_status.tcl`,
then return to the patched full Mac image. The 2026-10-24 standalone build
produced `build/tqtt_usb_host_probe/tqtt_usb_host_probe.bit` and matching
`.ltx`; it routed with WNS +12.686 ns, WHS +0.016 ns, WPWS +2.000 ns,
zero failing endpoints, and zero DRC errors. Vivado loaded the 1024x4
host microcode ROM into a RAMB18E2. Physical USB wiring and HID enumeration
remain untested.

## Hub and general USB stack

Two independent sockets are useful immediately, but they do not implement a
USB hub. Neither small HID core enumerates a hub's downstream ports. A real
hub requires full-speed hub enumeration, port power/reset/status handling,
and low-speed preamble transactions through a full-speed hub. The SpinalHDL
OHCI controller already documents full/low-speed, keyboard/mouse, and hub
support, but it needs a controller-side USB host driver plus BMB-to-SoC
control/DMA integration and cache-coherent memory handling. Evaluate it in
an isolated peripheral probe before deciding whether to replace the small
HID hosts. Keep the HID-to-ADB event interface independent of the host
engine so the Mac side remains reusable.

OHCI reference: <https://spinalhdl.github.io/SpinalDoc-RTD/master/SpinalHDL/Libraries/Com/usb_ohci.html>.

No full-SoC USB host RTL or PMOD pin constraints are connected yet. The
hardware port, core, CDC, HID mapping, and hub stack each need validation.

The report-to-ADB keyboard converter now lives in
`rtl/mac/usb_hid_adb_keyboard.v`. It compares complete six-key HID snapshots,
emits make/break events with a ready/valid handshake, releases held keys on
disconnect, ignores rollover reports, and avoids duplicate key events. The
`tb-usb-hid-adb-keyboard` regression covers those behaviors and backpressure.
`usb_hid_adb_keyboard_cdc.v` now carries complete reports from the 60 MHz
USB domain to the 50 MHz peripheral domain through the existing asynchronous
FIFO, including a zero report when the host disconnects. The
`tb-usb-hid-adb-keyboard-cdc` regression checks report order and key release
across the two clocks. The path is not yet connected to the Mac ADB injector;
hardware enumeration and Mac-visible USB typing remain pending.
