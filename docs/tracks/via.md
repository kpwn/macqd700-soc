# VIA production hardening

Scope: `rtl/mac/via1.v`, `rtl/mac/via2.v`, `tb/tb_via1.cpp`, `tb/tb_via2.cpp`.

Local reference used:
- `docs/mame_integration.md` for the 6522 summary-bit and handshake-clear notes.
- `docs/peripheral_arch.md` for the Mac reset defaults and port layout.

What this track tightened:
- Standard 6522 IFR summary handling stays `|(IFR[6:0] & IER[6:0])`.
- VIA1 now clears both CA-side and CB-side handshake flags on the matching ORA/ORB reads.
- VIA1 ADB empty-bus receive fallback now re-arms on SR writes while ACR remains in external shift-in mode, so repeated ROM probes do not depend on rewriting ACR.
- VIA2 now latches CB1 edges as well as CA1 edges, with PCR[4] selecting the edge polarity.
- VIA2 CA2/CB2 independent IRQ modes now respect PCR[2]/PCR[6] edge polarity instead of treating any toggle as an interrupt.
- VIA1/VIA2 ORB writes now acknowledge PB-side handshakes by clearing CB1 and non-independent CB2, matching the ORB read path while preserving independent CB2 IRQs.
- The focused Verilator tests now pin the handshake-read clears and the CB1 edge path.
- The ROM harness ADB shadow emits an `ADB.empty_bus` event when the no-device idle-high byte is synthesized.

Remaining risk:
- The timer models are still deliberately simple compared with the full MAME device scheduler.
- VIA1 still keeps its Mac-specific overlay readback layered on top of the base 6522 port read; PB[7:5] now follow the normal DDR mux instead of a synthetic RAM-size hint.
- VIA1's byte-granularity ADB transceiver port (`adb_rx_byte`/`adb_tx_byte`) is NOT the real boot path — it only fires when ACR is not in the real external-clock shift modes (011 RX / 111 TX), which real ADB traffic always uses.  The real path is VIA1's bit-level external-clock SR shift (`~line 587`) talking to `rtl/mac/adb_pic_modem.v` (the real PIC1654S firmware) over CB1/CB2; `rtl/mac/adb_phy.v` now answers that bit-level bus with real open-drain keyboard/mouse response frames (was previously a no-device watcher — see its header for the fix history).
