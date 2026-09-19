# SWIM/IWM Track

> Narrow scope brief for the floppy controller stub used by the ROM boot
> harness and the stand-alone Verilator smoke test.

## Current contract

- Reset to idle with no IRQ/DMA activity.
- Keep register selection semantics stable enough for ROM probing.
- Return benign no-media values instead of zeros that can be mistaken for
  a broken bus.
- Do not claim actual media transfer, interrupts, or DMA yet.
- Preserve the SWIM alias pairs used by the ROM probe:
  0/8 data, 1/9 mark, 2/A error, 3/B parameter, 4/C phase, 5/D setup.
- Keep mode clear/set semantics on 6 and 7, and reset the parameter
  pointer when either mode register write occurs.
- Status reads at 0xE and handshake reads at 0xF should stay deterministic
  for both drive-present and no-media paths.
- Reset into IWM-compatible mode, not SWIM mode.  The Q700 ROM probes the
  `0x50f1e000` mirror as an IWM path before any real floppy media path exists.

## Current implementation

- ROM boot harness: `tb/tb_rom_boot.cpp`
- Stand-alone Verilator stub: `rtl/mac/iwm_stub.v`
- Verilator unit test: `tb/tb_iwm.cpp`

## Behavior to preserve

- IWM read-all-ones returns `0xff`.
- Mode/control writes use clear/set semantics and leave enough latched state
  for the ROM status sequence to observe progress.
- Register 15 control writes such as `0xc0` update the low status bits used by
  the early ROM probe.
- While still in IWM-compatible mode, SWIM-bank writes other than the register
  7 SWIM-entry hook must not pre-seed SWIM-visible error, parameter, setup, or
  mode state.  They only drive the IWM phase/control/status path.
- Status and handshake advertise no media cleanly.
- Parameter RAM stays probe-visible even when no media is present.
- IRQ and DMA remain low.

## Reference assumptions

No local MAME source checkout was available in `/home/qwertyoruiop` during the
2026-04-21 SWIM/IWM pass.  The RTL contract is therefore matched to the
MAME-derived `SwimStub` behavior already present in `tb/tb_rom_boot.cpp`: IWM
mode updates phase/control latches and selected status, ignores unrelated
SWIM-bank writes, and permits register 7 as the conservative SWIM-mode entry
used by the stand-alone unit test.

## Integration note

The stand-alone RTL stub is intentionally not wired into `mac_top.v` yet.
That keeps the decoder work narrow while ROM probing is being stabilized.
The ROM boot harness mirrors this same minimum IWM contract so patched
frontier runs and `tb-iwm` agree on the observable probe behavior.  When real
RTL integration lands, the same contract should be hooked to `glue_cs_iwm` and
the ROM boot harness should be retired from ownership of the live floppy
response path.
