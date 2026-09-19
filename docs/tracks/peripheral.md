# Peripheral track

> Read `docs/agent_policy.md` first. This brief adds peripheral-track scope.

## Scope

Mac peripherals and the glue that lets the CPU talk to them. Everything
under `rtl/mac/*` plus the related unit tbs. Target chipset: **Quadra 700**
(discrete VIA1 + VIA2 + NCR 5380 + Z80 SCC + ASC/SONORA + DAFB + MEMCjr).

## Files you own

```
rtl/mac/{via1.v, via2.v, scsi.v, scc.v, asc.v, video.v, glue.v,
         irq_agg.v, rtc.v}
tb/{tb_via1, tb_via2, tb_scsi, tb_scc, tb_asc, tb_glue, tb_irq_agg,
    tb_rtc, tb_video, tb_vram}.cpp
```

Do NOT touch `rtl/core/*` (that's the core track), `rtl/sys/*` (platform),
`tb/tb_top.cpp` (coordination hotspot), `mac_top.v` / `fpga_top.v`
(platform — but you may READ them to understand wiring).

## Current state (per `docs/isa_status.md` Mac Peripherals section)

| Module | State | Notes |
|---|---|---|
| glue.v | ✅ real | Full Q700 address decoder + VIA1 ORB[3] overlay + fault/priv signals |
| via1.v | ✅ real | ADB byte SR + T1/T2 + RTC fan-out on PB[0:2] + 60 Hz VBL |
| rtc.v | ✅ real | 32-bit seconds + 256-byte PRAM + bit-shift protocol |
| scsi.v | ✅ real | NCR 5380 full phase FSM (BUS_FREE → SELECT → COMMAND → DATA → STATUS → MSG); READ(6) test covers command capture, data-phase DRQ, status/message, and IRQ persistence |
| irq_agg.v | ✅ real | Priority encode VIA1→L1 / VIA2→L2 / SCSI→L3 / SCC→L4 / ASC→L5 |
| via2.v | ✅ real | Full 6522 variant with NuBus/SCSI IRQ latches, timers, SR, IFR/IER |
| scc.v | ✅ real | Z85C30 async RS-422 subset with dual channels, TX/RX FIFO, loopback, IRQ summary |
| asc.v | ✅ real | SONORA FIFO sampler, version/mode/rate/volume regs, half-empty IRQ, sim audio sample output |
| video.v | 🚧 partial | DAFB register shim + HDMI scan-out; CLUT/base/stride/BPP/VBL live effects incomplete |

## Q700 address map (current RTL)

The Mac-device subdecode now follows the MAME-validated mirror documented in
`docs/rom_boot_bringup.md`: only `0x5000_0000..0x50FF_FFFF` is the Q700 I/O
mirror, and supported Mac device selects clear address bits covered by
`0x00FC_0000` before decoding.  That means `0x50F0_C000` aliases canonical
`0x5000_C000` (SCC), while `0x5100_xxxx` is outside the mirror and must not
be turned into a peripheral response.

```
0x0000_0000 – 0x0FFF_FFFF  RAM (overlay=0), ROM alias (overlay=1)
0x4000_0000 – 0x4FFF_FFFF  ROM image (1 MB Q700 @ files/420dbff3.rom)
0x5000_0000 – 0x5000_1FFF  VIA1 (Q700 16 regs at 0x200 stride)
0x5000_2000 – 0x5000_3FFF  VIA2 (Q700 16 regs at 0x200 stride)
0x5000_4000 – 0x5000_7FFF  empty gap; 0x50F0_4000 must stay unmapped
0x5000_8000 – 0x5000_8007  Ethernet ID/config PROM
0x5000_A000 – 0x5000_B0FF  SONIC reset/config registers
0x5000_C000 – 0x5000_DFFF  SCC (Z85C30)
0x5000_F000 – 0x5000_F0FF  DAFB TurboSCSI / NCR53C96 registers
0x5000_F100 – 0x5000_F101  DAFB TurboSCSI DMA handshake
0x5001_4000 – 0x5001_5FFF  ASC / EASC (SONORA)
0x5001_E000 – 0x5001_FFFF  IWM / SWIM (stub OK)
0x50xx_xxxx                mirrors of the canonical ranges above
0xF900_0000 – 0xF90F_FFFF  DAFB/VRAM pixel aperture
0xF980_0000 – 0xF980_03FF  DAFB register shim
0x9000_0000 – 0xEFFF_FFFF  NuBus slot $9..$E (0x1000_0000 stride)
```

`rtl/mac/glue.v` is the sim/front-door decoder and `rtl/sys/axi_xbar.v`
is the bitstream fabric decoder. Normal Mac MMIO reaches `peripheral_bus`
through xbar S1; DAFB registers are carved out to xbar S4. Ethernet
ID/SONIC now has a synthesized PROM/register block; Orwell controls and
SWIM/IWM still have probe-safe RTL stubs. Remaining gaps must not be aliased
to a nearby implemented peripheral just to satisfy a probe.

## IRQ levels (via irq_agg.v)

```
L1  VIA1 (ADB SR, VBL, T1/T2)
L2  VIA2 (NuBus slot IRQs, SCSI DRQ)
L3  SCSI (NCR 5380 IRQ line)
L4  SCC (serial)
L5  ASC (sound buffer empty)
L6  (reserved — DMA agent task #110 will claim)
L7  NMI (debugger)
```

## ROM overlay semantics

**The overlay bit is VIA1 ORB[3] (not VIA2 — common mistake).**
- Reset state: overlay=1 → low RAM window at 0x0000_0000 aliases the
  ROM image at 0x4000_0000. CPU fetches reset vector (0x0/0x4) from
  ROM.
- First boot step clears ORB[3] → overlay=0 → low RAM visible at 0.
- `glue.v` consumes `overlay_in` from VIA1 and muxes the address
  accordingly. ROM mirror at 0x4000_0000 is unaffected.

## Key design decisions

1. **Stubs return deterministic, not X.** Peripheral probes during ROM
   boot will hang on X values. When adding stub behavior, default to
   zero or a plausible constant — never leave a read path X.
2. **Slow-clock peripherals.** VIA1/VIA2 run on `phi2_tick`
   (783.36 kHz Q700 target), NOT directly on `clk_core`. SCC at
   3.672 MHz. See `docs/clocking.md` for the full 11-domain plan.
   Async FIFO + axi_async_bridge are available in `rtl/sys/` for CDC.
3. **Memory-mapped register width.** Most 68k peripherals are 8-bit
   wide. AXI transactions at the peripheral_bus are byte-strobed; your
   register decode checks `wstrb` / `rsize`.
4. **No INIT for stubs.** Mac OS INITs can install drivers for fake
   peripherals (task #90 phase-5), but until then, stubs must satisfy
   ROM-level probing, not Toolbox calls.

## Common pitfalls

- **Forgetting to advance RTC seconds.** ROM spins waiting for the RTC
  to tick. A stub that always returns 0 causes infinite probe loops.
- **VIA IFR bit polarity.** IFR bits are level-triggered active-high
  in the 6522 spec but read-cleared on some; verify against
  the Synertek 6522 datasheet.
- **NuBus slot IRQs are inverted** in VIA2 PA7:PA0 (active-low).
- **IWM signals are variable-CLV**, not fixed-MFM. Modern Floppy Emu
  clones understand the Apple protocol; don't pretend to be a PC FDC.
- **glue.v FC-based privilege.** `priv_violate` fires on user-mode I/O
  access. Tie into exception path only when MOVES is real (task #109).

## Testing rigour

Every peripheral upgrade needs:

1. A `tb-<peripheral>.cpp` unit tb scenario covering the corner you
   almost got wrong.
2. A ROM-boot test, once the peripheral is integrated — run
   `make tb-rom-boot` (see `docs/agent_policy.md`) and confirm the
   instruction count increases.
3. If the peripheral has an IRQ path, an irq-agg integration test
   showing the IRQ reaches the CPU and SR.I mask honors it.

## References

- `docs/peripheral_arch.md` — authoritative peripheral arch.
- `docs/clocking.md` — clock domains (§1-13 cover all 11 domains).
- `docs/mame_integration.md` — MAME `macquadra.cpp` + `m68kmmu.h`
  are the Tier-1 reference for exact peripheral semantics.
- `docs/legacy_phy_refs.md` — PHY reference (ADB, SCSI, LocalTalk,
  audio, video) for when your peripheral needs physical-layer
  awareness.
