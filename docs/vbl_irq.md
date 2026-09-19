# DAFB → VIA1 CA1 Vertical-Blank IRQ Chain (task #145)

> **STATUS 2026-09-06 — partially superseded.**  Everything below the
> "Wiring" heading described a chain the SoC no longer builds: VIA1's CA1
> has not been driven from `dafb_vbl_level` (nor from an XOR of it with
> VIA2 PB7) since the VIA2-PB7 board wire landed.  The current wiring, the
> measured rates, and the gate that actually covers the Mac's 60 Hz tick
> are in the new section **"Current chain (2026-09-06)"** at the end of
> this file.  Read that first; treat the middle of this document as
> history.

The Quadra 700 ROM and Mac OS rely on a periodic vertical-blank
interrupt for system-time advancement, cursor blink, the Time Manager,
and assorted scheduled tasks.  On real hardware the DAFB display
controller asserts a VBL signal on every frame's vertical-blank entry;
the signal is wired into VIA1's CA1 input via a chain that includes
VIA2 (which Mac OS later programs to mirror the rate as a 60 Hz square
wave on PB[7]).

This document describes the m68k-ooo implementation of that chain.

## Pulse origin

Source signal: `vbl_pulse_pclk` from `rtl/board/video_phy/video_top.v`.

The HDMI scan-out pipeline runs in the `pclk` (148.5 MHz) domain.  The
video timing generator (`rtl/board/video_phy/vtg.v`) emits a `vsync` strobe
at `vcount == V_ACTIVE` (1080 for 1080p60) — the first scanline beyond
the active region.  `video_top.v` registers `vsync` and emits
`vbl_pulse_pclk` as a 1-cycle pulse on its rising edge.

For 1080p60 timing this fires every 16.67 ms — exactly 60.000 Hz on
average.  This is HDMI-mode-locked and not derived from the Q700's
native 66.62 Hz 13" RGB timing (the rate MAME models for `macqd700`).
The Q700 ROM tolerates ±0.5 % rate variation per Inside Macintosh:
Devices §10 (Time Manager), and Mac OS itself supports both the 13"
RGB (66.62 Hz) and 12" RGB (60.15 Hz) timings, so HDMI's 60.000 Hz is
within the operating envelope.

## Clock-domain crossing

VIA1 lives in the `pb_clk` (50 MHz) peripheral-clock domain.  The DAFB
VBL pulse originates in `pclk` (148.5 MHz).  These domains are
asynchronous, so a level synchroniser is unsafe: a `pclk`-wide pulse
(~6.7 ns) is shorter than one `pb_clk` cycle (20 ns) and could be
missed entirely.

`rtl/board/pulse_cdc.v` implements a toggle-based pulse synchroniser:

```
  src_clk ──┐                ┌── dst_clk ──┐
            │                │             │
src_pulse ─[toggle]─[meta]─[sync]─[edge_det]─> dst_pulse
```

Each `src_pulse` flips a level register on the source side.  The level
crosses into the destination domain through a 2-flop synchroniser
(both flops marked `(* ASYNC_REG = "TRUE" *)`) and an edge detector on
the destination side fires `dst_pulse` for one destination-domain
cycle each time the synchronised level toggles.  This is the classic
Cummings pattern (SNUG 2008) and tolerates any frequency relationship
as long as consecutive source pulses are spaced at least one
destination-domain cycle apart — for our 60 Hz / 50 MHz combo, the
spacing is ~16.67 ms / 20 ns = ~833 333× the minimum, with comfortable
margin.

## Wiring (fpga_top_peripherals.vh)

```
                                       pclk
        ┌──────────┐    vbl_pulse     ┌─────────┐
   VTG─►│ video_top├─────────────────►│pulse_cdc├──┐
        └──────────┘                  └─────────┘  │ pb_clk
                                                   ▼
                                       dafb_vbl_pulse_pb
                                                   │
                                       ┌────────┐  │
                                       │extender│◄─┘
                                       └───┬────┘
                                  dafb_vbl_level
                                           │
                       via2_pb7_ca1 ──XOR──┴── via1_ca1_in
                                           │
                                           ▼
                                       VIA1.vblank_irq_in
                                           │
                                           ▼
                                       IFR.CA1 → IRQ → irq_agg → CPU L1
```

The pulse extender holds `dafb_vbl_level` high for `VBL_LEVEL_TICKS`
(32) `pb_clk` cycles after each pulse — long enough for VIA1's edge
detect (clocked on every `pb_clk` and gated by `phi2_tick`) to sample
both the rising and the falling edge across at least one `phi2`
boundary.

The XOR combination with `via2_pb7_ca1` preserves edges from the
ROM-software-driven VIA2 PB[7] chain (Mac OS programs VIA2 T1 in
free-run mode to drive PB[7] as a 60 Hz square wave) — every
transition on either source flips the output level, which VIA1's CA1
edge detector samples on every `pb_clk`.  PCR programmed for one
specific edge polarity (Q700 ROM default is low-to-high) yields one
CA1 IRQ per VBL.

## IRQ chain

```
DAFB VBL pulse (pclk)
  │ pulse_cdc
  ▼
dafb_vbl_pulse_pb (pb_clk)
  │ pulse extender
  ▼
dafb_vbl_level (level)
  │ XOR via2_pb7_ca1
  ▼
via1_ca1_in
  │ rising-edge detect inside via1.v
  ▼
VIA1 IFR.CA1 ← latches when PCR match
  │ AND IER.CA1
  ▼
VIA1 IRQ output
  │ 2-flop level sync to core_clk
  ▼
irq_agg → IPL = 3'd1
  │
  ▼
CPU autovec L1 dispatch (vector 25 = SR @ 0x64)
```

The CPU acks by reading VIA1 register 1 (ORA), which clears IFR.CA1
in `via1.v` (per 6522 spec).  IRQ drops; next VBL re-arms.

## How to verify

### Unit tb (sim-only)

```
make tb-via1               # 27/27 PASS — covers periodic CA1 edges
make tb-irq-agg            # 13/13 PASS — covers L1 priority encode
make tb-dafb-via-irq       # 21/21 PASS — DAFB→VIA1→irq_agg
make tb-vbl-rate           # NEW — gates the pulse_cdc + 60 Hz rate
make tb-video              # 17/17 PASS — proves vbl_pulse_pclk fires
```

`tb-vbl-rate` asserts:

1. `vbl_pulse_pclk` fires at 60.000 Hz ± 1 % over a 0.9 s window
   (measured from inter-pulse spacing, not pulses-per-window).
2. Every `pclk`-domain pulse crosses the CDC into a single
   `pb_clk`-domain pulse — no drops, no duplications.
3. VIA1 IFR.CA1 latches at least once per sub-window after the
   ROM-default PCR/IER programming.

### MAME baseline

```
MAME_VBL_TRACE_OUT=build/mame_runs/vbl_q700.csv \
MAME_VBL_TRACE_SECONDS=2 \
mame -rompath /tmp/mame_iwm/roms macqd700 \
     -window -resolution0 320x240 -nothrottle \
     -seconds_to_run 4 -sound none -skip_gameinfo \
     -autoboot_delay 0 \
     -autoboot_script tools/mame_vbl_capture.lua
```

Output: 120 DAFB events in 2 simulated seconds = 60.0 Hz when scaled
to wall-clock (the MAME emulator runs the Q700 13" RGB mode at
66.62 Hz natively — see `set_raw(31334400, 896, 0, 640, 525, 0, 480)`
in `~/mame/src/mame/apple/dafb.cpp` — but `register_frame_done` in the
Lua script counts one event per emulated screen frame, which matches
the canonical DAFB `vbl_tick` cadence in `dafb.cpp:945`).

## Lockstep tolerance

MAME runs the Q700's DAFB at 66.62 Hz native (35 kHz × 525 lines × 1 /
896 cycles).  Our HDMI pipeline emits at 60.000 Hz (1080p60).  The
two rates differ by ~10 %.

This is **not** a lockstep mismatch — it's a deliberate mode
divergence:

* MAME emulates the silicon's actual 13" RGB timing.
* Our HDMI is locked to 60.000 Hz because monitors expect that rate.
* Both rates are valid Mac OS modes; the ROM's Time Manager is
  invariant to the rate within ±0.5 % per IM:Devices.

The lockstep gate therefore checks **rate stability**, not rate
equality.  RTL must hold 60.000 Hz ± 1 % over a 0.9 s window
(`tb-vbl-rate`), and MAME's capture must hold its own native rate
across the same window — but the absolute rates are intentionally
different.

If we ever switch the HDMI mode to a Q700-native 66.62 Hz output
(would need the MMCM retuned for a 31.3344 MHz pixel clock and
640×480 letterboxing), the gate will tighten to ±0.5 % equality
against MAME.

## Files touched

* `rtl/board/video_phy/video_top.v` — added `vbl_pulse_pclk` output,
  rising-edge detect on `vsync`.
* `rtl/board/pulse_cdc.v` — new toggle-based pulse synchroniser.
* `rtl/soc/fpga_top_video.vh` — plumb `video_vbl_pulse_pclk` net.
* `rtl/soc/fpga_top_peripherals.vh` — instantiate `pulse_cdc`, pulse
  extender, and the XOR drive of `via1_ca1_in`.
* `tb/tb_via1.cpp` — new `test_periodic_dafb_vbl_irq` scenario.
* `tb/tb_vbl_rate.{v,cpp}` — new unit tb covering the full chain.
* `tb/tb_video_top.v` — added `vbl_pulse_pclk` port.
* `tools/mame_vbl_capture.lua` — MAME tap that emits one event per
  frame_done callback.

## Follow-ups

* MAME's `:via1` device only exposes the IRQ via Lua state if the
  state-interface registers it; for now the capture script falls back
  to counting DAFB-side events.  Adding a CB-tap for `m_via1->irq()`
  in a custom MAME build (or via `install_read_tap` on the VIA1 IFR
  register) would let us assert end-to-end VIA1 IFR.CA1 toggles.
* The pulse-extender constant `VBL_LEVEL_TICKS = 32` (640 ns at 50 MHz
  pb_clk) is conservative for a 60 Hz signal.  If a higher-rate VBL
  source is ever wired here (e.g. 480 Hz cursor scanline IRQ), the
  extender width must shrink to avoid masking adjacent pulses.


## Current chain (2026-09-06)

### What actually produces the Mac's 60 Hz tick

The Mac's `Ticks` global (0x016A) is advanced by the level-1 VBL
interrupt.  On this SoC that interrupt is a **VIA2 Timer-1 square wave**,
not the HDMI vertical blank:

```
pb_clk (50 MHz)
  │  phi2 NCO                       rtl/soc/fpga_top_clocks.vh:906-923
  ▼
phi2_tick  = 783 360 Hz
  │  VIA2 T1, ACR[7:6] = 11         rtl/mac/via2.v:416-446
  │  (free-run + PB7 output)        latch 0x196E written by the ROM
  ▼
via2_pb_out[7]  = 60.1 Hz square wave   rtl/mac/via2.v:553
  │  board wire                     rtl/soc/fpga_top_peripherals.vh:1058
  ▼                                  `wire via1_ca1_in = via2_pb_out[7];`
VIA1 CA1 → IFR.CA1 (PCR[0] edge)    rtl/mac/via1.v:645-649
  │  AND IER.CA1
  ▼
via1_irq → 2-FF sync into core_clk → irq_agg → IPL 1 → vector 25
```

`dafb_vbl_level` is still live, but it no longer touches VIA1: it is
resampled into `core_clk` and drives `video.v`'s `frame_tick`
(`fpga_top_peripherals.vh:1073-1081`), which raises the DAFB slot IRQ into
**VIA2** CA1 at level 2.  The XOR that this document used to draw is gone.

### Measured rates (`make tb-via-tick-rate`)

`tb/tb_via_tick_rate.{v,cpp}` rebuilds exactly the chain above — the phi2
NCO is copied verbatim from `fpga_top_clocks.vh` and both VIAs are the
real RTL — and measures rates in Hz from simulated time:

| quantity                              | measured   | expected  |
|---------------------------------------|-----------:|----------:|
| `phi2_tick`                           | 783 360.0 Hz | 783 360 Hz |
| PB7 toggles, ROM latch 0x196E         | 120.000 Hz | 120.29 Hz |
| VIA1 CA1 interrupts, ROM latch 0x196E | **60.000 Hz** | 60.147 Hz |
| VIA1 CA1 interrupts, latch 0x0CB7     | 120.000 Hz | 120.258 Hz |
| VIA1 CA1 interrupts, latch 0xFFFF     | 6.000 Hz   | 5.976 Hz  |

The last row is the important one for triage: **0xFFFF is the slowest
tick this chain can possibly produce.**  T1's latch is 16 bits, so no ROM
or Mac OS programming, and no T1-side RTL defect short of a broken
`phi2_tick`, can put the tick below ~6 Hz.  A hardware tick measured
slower than that (e.g. the ~1.4 Hz seen from `Ticks` = 312 after 220 s)
therefore **cannot** be a VIA2 T1 / PB7 rate problem — look at interrupt
delivery and servicing instead (irq_agg priority, VIA2's level-2 IRQ
starving level 1, peripheral-bus stalls, or the VBL handler's own
runtime).

### Which gate covers what

| gate                  | covers                                            | does NOT cover |
|-----------------------|---------------------------------------------------|----------------|
| `tb-via-tick-rate`    | phi2 rate, VIA2 T1→PB7→VIA1 CA1 tick rate in Hz | the 68k side |
| `tb-vbl-rate`         | HDMI VTG vblank pulse rate + pclk→pb_clk CDC     | the tick — it feeds VIA1 CA1 from `dafb_vbl_level`, which the SoC does not do |
| `tb-via2`             | 6522 register/timer semantics; PB7 *toggles*      | any rate — `test_t1_freerun_pb7` passes at 1.4 Hz and 60 Hz alike |
| `tb-dafb-via-irq`     | DAFB→VIA1→irq_agg route                          | rate; also a known-red gate (16/6) |


### Confirmed on silicon, 2026-09-06 (build 0x05157D00)

The sim result above was checked against the live board.  That bitstream
carries the same wiring (`git show 05157d00:rtl/soc/fpga_top_peripherals.vh`
line 1014 is `wire via1_ca1_in = via2_pb_out[7];`).

VIA2 config, read over JTAG — identical to what the sim programs:

```
VIA2 ACR  (+0x1600) = 0xC0      T1 free-run + PB7 output
VIA2 T1LL (+0x0C00) = 0x6E      latch = 0x196E
VIA2 T1LH (+0x0E00) = 0x19
VIA2 IER  (+0x1C00) = 0x82      only CA1 enabled; T1 IRQ is OFF
```

T1's counter high byte (VIA2 reg 5, +0x0A00 — a side-effect-free read)
was then sampled 1962 times over 2.3625 s.  Counting reload events (the
counter jumping back up to 0x19) gives the T1 wrap rate directly, and
each wrap toggles PB7:

| quantity              | silicon      | expected     | error   |
|-----------------------|-------------:|-------------:|--------:|
| T1 reloads / PB7 toggles | 120.209 Hz | 120.29 Hz  | −0.07 % |
| VIA1 CA1 edges (the tick) | **60.105 Hz** | 60.15 Hz | −0.07 % |

**The tick source is correct on hardware.**  A ~1.4 Hz `Ticks` rate is
therefore not produced here; the loss is downstream of `via1_ca1_in`.

Downstream state measured in the same session — note the board was
sitting in the known 53C96 status-poll wedge at the time, so these
characterise that wedge, not a healthy boot:

```
VIA1 IER (+0x1C00) = 0xA7   -> enabled = 0x27: CA2, CA1, SR, T2
VIA1 PCR (+0x1800) = 0x22   -> PCR[0]=0: CA1 latches on the FALLING edge
                               of PB7 (still one latch per PB7 period)
VIA1 IFR (+0x1A00) = 0x48   -> bit 1 (CA1) CLEAR in 1963/1963 samples
                               taken 1.2 ms apart over 2.38 s
CPU exception ring: 34 exceptions in 14.77 s = 2.30/s, alternating
   vec 0x19 (level 1, VIA1) and vec 0x1A (level 2, VIA2), all
   interrupting a tight ROM loop at 0x40899706/0x4089970A
   (`moveb %a3@(0x40),%d5` — the 53C96 status poll)
Ticks (0x016A) = 0x0001223F, unchanged across 36 s
```

Two things follow:

* ~1.15 level-1 dispatches per second against 60.105 Hz of CA1 edges is
  a **~52x loss between the VIA1 CA1 pin and the 68k's exception
  dispatch**, and level 2 is throttled by the same factor.  A
  VIA1-specific defect cannot throttle level 2, so the common cause is
  on the CPU/dispatch side (SR interrupt mask held high in the poll
  loop, or `ipl` sampling), not in either VIA.
* IFR.CA1 reading clear essentially always rules out "the interrupt is
  pending and never taken".  The flag is being cleared as fast as it is
  set — i.e. something is consuming CA1 (any read or write of VIA1
  register 1, the ORA *handshake* alias at +0x200, clears it per the
  6522 spec) between the edge and the dispatch.  That is the next thing
  to look at, together with the SR mask in the poll loop.
