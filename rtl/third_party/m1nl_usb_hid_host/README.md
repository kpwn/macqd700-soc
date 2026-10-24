# USB HID host core

Files `usb_hid_host.v`, `usb_hid_host_rom.v`, and `usb_hid_host_rom.mem` are
from <https://github.com/m1nl/usb_hid_host>, commit
`e16acb7adde46f7f4a2a9417feb97bf3a8a3129c` (Apache-2.0).

The ROM module's `$readmemh` path is adjusted for Vivado running from this
repository root. The core and ROM contents are otherwise unchanged.

This copy is initially used by the standalone TQTT USB host probe. The core
supports one directly connected HID device per instance; it does not
enumerate a USB hub.
