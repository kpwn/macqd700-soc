# ROM Harness MAME Gap Audit

Date: 2026-04-21

Scope: Q700 ROM harness and peripheral-facing model behavior only. This does
not cover core decode, exception sequencing internals, or video scaler logic.

Primary MAME reference: tag `mame0287`.

## Source References Checked

| Area | MAME source | Local behavior checked |
|---|---|---|
| Q700 map and overlay | `src/mame/apple/macquadra700.cpp` | `tb/tb_rom_boot.cpp`, `rtl/mac/glue.v`, `tb/tb_glue.cpp` |
| VIA1/VIA2 6522 semantics | `src/devices/machine/6522via.cpp` | `tb/tb_rom_boot.cpp`, `rtl/mac/via1.v`, `rtl/mac/via2.v` |
| DAFB and TurboSCSI | `src/mame/apple/dafb.cpp` | `tb/tb_rom_boot.cpp`, `rtl/mac/glue.v`, `rtl/mac/video.v`, `rtl/mac/scsi.v` |
| RTC side channel | `src/mame/apple/macquadra700.cpp`, `src/mame/apple/macrtc.cpp` | `tb/tb_rom_boot.cpp`, `rtl/mac/rtc.v` |

## Matched Expectations

| Expectation | MAME observation | Current local status |
|---|---|---|
| High-ROM read disables reset overlay | `rom_switch_r()` installs RAM at zero and clears `m_overlay` on a `0x40000000..0x400fffff` read. | Harness has one-way high-ROM-fetch overlay clear and VIA1 ORB/DDRB clear observation. `rom-boot-overlay-clear-smoke` covers this. |
| Q700 I/O mirror | Q700 device windows use `.mirror(0x00fc0000)`; `0x5100xxxx` is outside this mirror. | `glue.v` and harness canonicalize `0x50xx_xxxx` with `0x00fc0000` and keep `0x5100xxxx` unmapped. `tb-glue` covers this. |
| VIA register stride | MAME shifts VIA offsets by 8 before masking 16 regs; Q700 map gives a 0x200 stride. | Harness and RTL use `(addr >> 9) & 0xf`. `tb-glue`, VIA unit tests, and ROM event smokes cover it. |
| VIA IFR summary | MAME derives bit 7 from `IFR & IER & 0x7f` in `output_irq()`. | Harness reads synthesize IFR bit 7 from pending enabled flags; RTL VIA models also track summary behavior. The directed VIA1 smoke now checks masked vs enabled summary behavior. |
| VIA1 Timer 2 polling | MAME's 6522 model treats T2 as a one-shot timed interrupt when `ACR[5]=0`; T2CL read clears `IFR.T2`. | Harness now tracks T2 latch/counter/running state, starts on T2CH write, sets `IFR.T2` on one-shot wrap, clears on T2CL read or IFR write-1-clear, and logs `VIA1.timer2_start` / `VIA1.timer2_wrap`. |
| VIA SR read/write clears SR interrupt | MAME `src/devices/machine/6522via.cpp` clears `INT_SR` on SR read (`mame0287` lines 764-778) and SR write (lines 961-974). | Harness clears stale SR-complete on write, sets completion later, returns empty-bus `0xff` on read, and now clears IFR.SR on SR read. `tb-rom-boot-adb-smoke` checks the read-clear event. |
| ADB visibility | MAME's ADB path is mediated through VIA1 SR mode and 6522 interrupt state. | Harness ADB events now carry a transaction id plus ACR/PCR/SR-mode context so ROM polling logs distinguish command writes, empty-bus completions, and SR read-clear acknowledgements. |
| RTC pins | Q700 `via_out_b()` drives RTC CE, data, and clock from VIA1 PB2/PB0/PB1; `via_in_b()` reads RTC data and ADB IRQ idle on PB3. | Harness RTC/PRAM side-channel is implemented and smoked by `tb-rom-boot-rtc-smoke`; RTC event details now include command direction and extended-register decode. |
| TurboSCSI windows | Q700 maps `0x5000f000..0x5000f0ff` to DAFB TurboSCSI regs and `0x5000f100..0x5000f101` to pseudo-DMA. | `glue.v` and harness restrict SCSI to this narrow window. `tb-rom-boot-scsi-smoke` covers directed visibility. |
| VRAM aperture | MAME maps DAFB VRAM at `0xf9000000..0xf91fffff`. | Harness `MemModel` now has a 2 MiB VRAM backing store and classifies only that range as RAM-like VRAM. |
| DAFB register aperture | MAME maps DAFB registers at `0xf9800000..0xf98003ff`. | `glue.v` now selects only this 1 KiB window for `cs_video`; adjacent `0xf9800400` and the old 4 KiB top `0xf9800ffc` fault in `tb-glue`. |
| ASC/EASC aperture | MAME maps EASC at `0x50014000..0x50015fff`. | `glue.v` now selects only this 8 KiB window; `0x50016000` and its `0x50f16000` mirror fault in `tb-glue`. |

## Remaining Bringup Tasks

| Priority | Gap | Why it matters | Suggested owner surface |
|---|---|---|---|
| P0 | ROM-run event smoke still reaches only VIA1/VIA2/RTC/VBL/early ADB on the real 70k-instruction path. | We cannot yet observe the ROM entering SCSI/SCC/ASC/DAFB/VRAM probe code without directed selftests. | Keep `tb-rom-boot-periph-events` as a reachability smoke, but add PC-stop/checkpoint runs at the next known ROM probe frontiers. |
| P0 | ROM writes are dropped with OKAY response in the harness; unmapped/open-bus data accesses also return OKAY in `daxi_resp_for_addr()`. | MAME map distinguishes unmapped regions from mapped ROM/read-only regions; real 68040 bus-error behavior will matter for RAM sizing, bad slot probes, and ROM diagnostic fallback paths. | Harness/model task: classify responses for read-only ROM writes and unmapped data cycles, then add a ROM-frontier opt-in smoke before changing default behavior. |
| P1 | DAFB TurboSCSI DRQ-check timing is only partially modeled in the harness. | MAME DAFB control bit 7/8 can hold off pseudo-DMA until DRQ. The harness now exposes phase-aware status/interrupt/sequence readback, but DMA still advances through a deterministic stub rather than a full 53C96/DAFB data path. | Harness SCSI stub first, then `rtl/mac/scsi.v`/DAFB integration. |
| P2 | SCC/ASC are real unit models but ROM harness access stubs are still simplified. | The unit models are useful for production, but the ROM harness may diverge if early ROM code expects side effects beyond zero/idle reads. | Add directed ROM-harness selftests for SCC and ASC event visibility before swapping behavior. |

## Change Made In This Audit

The 2026-04-21 VIA/ADB/RTC hardening pass made the ROM harness shadow model
closer to the production VIA/RTC behavior without changing unrelated device
surfaces:

- VIA1 Timer 2 now has explicit latch/counter/running state for timed
  one-shot mode, sets `IFR.T2`, clears via T2CL read or IFR write-1-clear,
  and survives checkpoint save/restore version 4.
- The VIA1 directed smoke also checks overlay/PB DDR-mux readback and IFR bit
  7 summary masking.
- ADB SR events include a transaction id and ACR/PCR/SR mode context so
  empty-bus completions can be correlated with ROM polling loops.
- RTC side-channel events include read/write direction and extended command
  decode details for normal PRAM, XPRAM, seconds, and control-register
  transactions.

The shared host `MemModel` now maps `0xf9000000..0xf91fffff` as persistent
VRAM. Before this audit, `tb_rom_boot` classified `0xf9xxxxxx` as RAM-like
VRAM for logging, but `MemModel` did not translate that range, so writes were
dropped and reads returned `0xff`. That made a future Happy/Sad Mac VRAM dump
impossible even if the ROM reached the framebuffer write loop.

The new `tb-mem-model` host test covers:

- VRAM base word persists.
- VRAM top word at `0xf91ffffc` persists.
- Adjacent addresses outside the 2 MiB MAME aperture remain unmapped.
- The existing sim-magic region still persists after the new VRAM allocation.

The 2026-04-21 glue hardening also tightened the ROM-probe-facing decode:

- DAFB register `cs_video` is limited to `0xf9800000..0xf98003ff`.
- ASC/EASC `cs_asc` is limited to `0x50014000..0x50015fff`.
- Ethernet ID/SONIC now has a synthesized PROM/register block; Orwell
  controls and SWIM/IWM still have probe-safe RTL stubs in `glue.v` and
  `peripheral_bus.v`. `tb-glue` and `tb-peripheral-bus` check their
  canonical and mirrored addresses hit owned device windows while the
  adjacent gaps still fault.
