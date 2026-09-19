# AN9134 Stereo Σ-Δ PWM Audio — Design Spec

**Status:** approved (brainstorm)
**Date:** 2026-05-04
**Scope:** First-light hardware audio path on the ALINX AN9134 / KU5P platform.
The AN9134 occupies the only 40-pin connector and does not route the SiI9134
audio inputs. This spec adds a single-pin-per-channel Σ-Δ PWM bridge that
hangs off the existing `AUDIO_PATH` selector and emits stereo audio on the
two NC pins of the AN9134 header (J1.35, J1.36) into a small external RC
filter + 3.5 mm jack.

---

## 1. Motivation & Constraints

The repo already has a complete ASC → audio bridge plumbing (see
`rtl/fpga_top_peripherals.vh:880-1052`):

- `rtl/mac/asc.v` — emits `audio_pcm_l[15:0]`, `audio_pcm_r[15:0]`,
  `audio_sample_valid` at the ASC sample rate (≤22.254 kHz).
- `rtl/sys/audio_i2s.v` — I2S serializer (BCLK/LRCLK/DATA), needs 3 pins +
  external codec. **Blocked**: AN9134 has only 2 NC pins.
- `rtl/sys/audio_hdmi_bridge.v` — IEC60958 subframe packer for HDMI
  data-island embedding. **Blocked**: AN9134's 40-pin header does not
  expose any of the SiI9134's audio inputs (MCLK, SCK, WS, SD0–SD3,
  SPDIF), confirmed against the Alinx pin table; the daughtercard simply
  does not break those pins out.

The AN9134 J1 connector exposes exactly two NC pins (35 and 36) and four
power pins (1 GND, 2 +5 V, 37 GND, 38 GND, 39 +3V3, 40 +3V3 — pins 39/40
are listed as power but described "NC" in the manual; treat as not
available). All other 32 pins carry HDMI traffic (RGB, sync, I2C, reset,
int).

That leaves Σ-Δ PWM (one pin per channel, analog reconstruction off-board)
as the only path that fits inside the existing pin budget without bodging
to the SiI9134 die. This spec adopts that path.

The ASC's effective resolution on real Mac silicon is ~8 bits at ≤22 kHz;
a first-order Σ-Δ at any reasonable OSR clears that bar by a wide margin.

## 2. Architecture

```
 ASC ──{audio_pcm_l[15:0], audio_pcm_r[15:0], audio_sample_valid}──► audio_pwm
                                                                       │
                                                              {pwm_l, pwm_r}
                                                                       │
                                                            FPGA pins (TBD ↔ J1.35/36)
                                                                       │
                                                          [AC-couple + RC LPF on PCB]
                                                                       │
                                                                  3.5 mm TRS jack
```

`audio_pwm` joins `audio_i2s` and `audio_hdmi_bridge` as a third sibling
under the `AUDIO_PATH` localparam at `rtl/fpga_top_peripherals.vh:971-976`,
encoded as `AUDIO_PATH_PWM = 3`. AN9134 builds default to PWM; I2S and HDMI
remain valid for future board revisions and stay tied off when PWM is
selected.

## 3. Σ-Δ Modulator Design

Per channel: **first-order error-feedback Σ-Δ** running at `pb_clk` (the
peripheral-bus clock domain that already feeds `audio_i2s` and
`audio_hdmi_bridge`).

- **Input format:** 16-bit signed PCM, two's complement.
- **Hold:** latch `pcm_l` / `pcm_r` on each `audio_sample_valid` strobe;
  the modulator integrates the held value every `pb_clk` tick. No
  interpolation filter — the stair-step held input plus the high OSR plus
  the analog LPF together perform reconstruction.
- **Domain conversion:** signed 16-bit → 16-bit offset binary
  (`x_u = signed16 + 16'h8000`, i.e. invert MSB). Silence (signed = 0) maps
  to `x_u = 0x8000` so the modulator settles at 50% duty.
- **Topology:** 16-bit accumulator with explicit carry-out as the 1-bit
  output:

  ```
  {pwm_out, err_next} = {1'b0, err} + {1'b0, x_u};   // 17-bit add
                                                      // pwm_out is bit 16,
                                                      // err_next is bits 15:0
  ```

  This is the standard first-order error-feedback Σ-Δ: the modulus-2^16
  wrap of the unsigned accumulator IS the error feedback, and the carry
  out IS the 1-bit output. No subtraction needed.

- **OSR:** at `PB_CLK_HZ = 50_000_000` and ASC max sample rate
  22.254 kHz, OSR ≈ 2247. First-order Σ-Δ in-band SNR is approximately
  `6.02 + 1.76 - 5.17 + 9·log₂(OSR) ≈ 103 dB`, ENOB ≈ 17 in the audio
  band — far past what ASC sources at ≤8 effective bits.
- **Output:** one bit per channel, toggling at up to `pb_clk / 2`.

## 4. RTL Boundary

### New module: `rtl/sys/audio_pwm.v`

```verilog
module audio_pwm #(
    parameter integer CORE_FREQ_HZ = 50_000_000
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] sample_l,
    input  wire [15:0] sample_r,
    input  wire        sample_valid,
    output wire        pwm_l,
    output wire        pwm_r
);
```

- Synchronous reset (active-high), matches project convention
  (`CLAUDE.md` §"Coding Conventions").
- Clock domain: same `pb_clk` that drives ASC and the existing I2S/HDMI
  bridges. No CDC inside the module.
- `CORE_FREQ_HZ` parameter is informational only (used for assertions and
  documentation); the modulator itself is rate-agnostic.

### Integration: `rtl/fpga_top_peripherals.vh`

Extend the `AUDIO_PATH` selector:

```verilog
localparam integer AUDIO_PATH_OFF  = 0;
localparam integer AUDIO_PATH_I2S  = 1;
localparam integer AUDIO_PATH_HDMI = 2;
localparam integer AUDIO_PATH_PWM  = 3;   // NEW

// Default for AN9134 builds
localparam integer AUDIO_PATH = AUDIO_PATH_PWM;
```

Add a fourth branch to the existing generate at lines 987-1041:

```verilog
else if (AUDIO_PATH == AUDIO_PATH_PWM) begin : gen_audio_pwm
    audio_pwm #(
        .CORE_FREQ_HZ (PB_CLK_HZ)
    ) u_audio_pwm (
        .clk         (pb_clk),
        .rst         (pb_full_rst_bank[3]),
        .sample_l    (asc_audio_pcm_l_w),
        .sample_r    (asc_audio_pcm_r_w),
        .sample_valid(asc_audio_sample_valid_w),
        .pwm_l       (pwm_l_w),
        .pwm_r       (pwm_r_w)
    );
    assign i2s_bclk_w               = 1'b0;
    assign i2s_lrclk_w              = 1'b0;
    assign i2s_data_w               = 1'b0;
    assign hdmi_audio_sf_l_w        = 28'h0;
    assign hdmi_audio_sf_r_w        = 28'h0;
    assign hdmi_audio_frame_valid_w = 1'b0;
end
```

Existing `AUDIO_PATH_I2S` / `AUDIO_PATH_HDMI` / `AUDIO_PATH_OFF` branches
extend to also tie off `pwm_l_w` / `pwm_r_w` to `1'b0`. The synth
optimiser drops the inactive bridges as before.

### Top-level wiring: `rtl/fpga_top.v`

Add two new top-level ports:

```verilog
output wire pwm_audio_l,
output wire pwm_audio_r,
```

Wire them from `pwm_l_w` / `pwm_r_w`. Both default to `1'b0` when
`AUDIO_PATH != AUDIO_PATH_PWM` so unused pins float at logic-low rather
than tristate.

### Synth-only consumer keepalive

The existing `audio_alive_tap` XOR (line 1049) gains
`^ pwm_l_w ^ pwm_r_w` so synth doesn't prune the bridge before the
top-level pins are claimed by XDC.

## 5. Pinout & XDC

### FPGA pin assignment

Resolved via three-way cross-reference between the AN9134 manual J1
pin table (which lists J1.35 / J1.36 as NC), `synth/hdmi.xdc` (which
covers J1.3–J1.34 explicitly), and `~/40pin.xdc` (the carrier-side
40-pin reference, indexed 0..16 across two rows of the 2×20 header).
The only two pins that appear in `~/40pin.xdc` but NOT in `hdmi.xdc`
are uniquely the J1.35 / J1.36 pair:

| AN9134 J1 | Carrier 40-pin index | FPGA package pin |
|-----------|----------------------|------------------|
| J1.35 (NC) | input-row idx 16    | **H13**          |
| J1.36 (NC) | output-row idx 16   | **J13**          |

### New file: `synth/audio_pwm.xdc`

```tcl
# audio_pwm.xdc — Σ-Δ PWM audio output to AN9134 J1.35/36 via external
# RC reconstruction filter and 3.5 mm jack.

set_property PACKAGE_PIN H13 [get_ports pwm_audio_l]   ;# AN9134 J1.35
set_property PACKAGE_PIN J13 [get_ports pwm_audio_r]   ;# AN9134 J1.36

set_property IOSTANDARD LVCMOS33 [get_ports {pwm_audio_l pwm_audio_r}]
set_property DRIVE      8        [get_ports {pwm_audio_l pwm_audio_r}]
set_property SLEW       SLOW     [get_ports {pwm_audio_l pwm_audio_r}]

# Analog endpoint — no timing requirement.
set_false_path -to [get_ports {pwm_audio_l pwm_audio_r}]
```

`SLEW SLOW` is intentional — the modulator output toggles at MHz rates and
slow slew reduces radiated EMI on a flying-lead bodge.

### `synth/vivado.tcl` plumbing

Add `read_xdc synth/audio_pwm.xdc` alongside the existing HDMI / DDR4 /
fpga_top XDC reads, gated identically to the audio bridge so SIM-only
builds don't fail when the file is absent during early branch iteration.

## 6. External Hardware (Off-FPGA)

The first-light board is a **mono single-MOSFET class-D speaker amp**
driven from `pwm_audio_l` (FPGA H13 → AN9134 J1.35).  Uses 5 V + GND
also tapped from the AN9134 J1 connector.  `pwm_audio_r` (J1.36) is
left for a later OPA2132-buffered line-out / headphone path on the
same board.

### BoM

| Ref | Value          | Notes                                              |
|-----|----------------|----------------------------------------------------|
| Q1  | IRLZ54N        | logic-level N-MOSFET, supplied by user             |
| D1  | 1N5819 (or SS14 / MBR0540) | Schottky 1 A / 40 V, flyback         |
| Csup | 100 µF / 16 V | bulk supply bypass                                 |
| Cout | 470 µF / 16 V | DC block; sets LF roll-off ≈ 40 Hz into 8 Ω        |
| Rg  | 47 Ω           | gate damp (kills ringing, protects FPGA pin)       |
| Rpd | 100 kΩ         | gate pull-down (boot safety: holds Q1 OFF before FPGA initialises) |
| (opt) Cgnd | 100 nF  | ceramic across Csup, kills HF supply impedance     |
| Speaker | 8 Ω 2 W    | user-supplied                                      |

### Topology

```
                       +5 V (AN9134 J1.2)
                        │
                        ├──[Csup 100 µF +]──── GND
                        │
                        ├── speaker(+) ───┐
                        │                  │
                        │                 [8 Ω 2 W]
                        │                  │
                        │              speaker(–)
                        │                  │
                        │              [Cout 470 µF +]   ← + lead THIS side (speaker)
                        │                  │
                        │                  │            ← Cout - lead THIS side (swnode)
                        │              swnode (Q1 drain, D1 anode joined)
                        │                  │
                        ├──[D1 cathode]    │
                        │  D1 anode ── swnode
                        │              │
                        │              [Q1 IRLZ54N drain]
                        │              [Q1 source] ── GND
                        │              [Q1 gate] ──[Rpd 100 kΩ]── GND
                        │                       └──[Rg 47 Ω]── pwm_audio_l
                        │
                       GND (AN9134 J1.1 / J1.37 / J1.38)
```

**Cout polarity is critical.**  In steady state at 50 % duty, the
speaker side sits at +5 V DC and `swnode_avg` = 2.65 V (D1's forward
drop pushes the OFF level to 5.3 V, not 5.0 V), so Vcout settles to
**+2.35 V with the + lead on the SPEAKER side**.  Installing the cap
backwards reverse-biases it, drops effective capacitance to <10 % of
nominal, raises the HP cutoff into the audible band (attenuating the
fundamental more than its harmonics), and produces strong even-order
distortion — symptom: 2nd harmonic at parity with or louder than the
fundamental, and a tone that gets quieter over seconds as reverse
charge accumulates.

`pwm_audio_l` (FPGA H13 / AN9134 J1.35) is the only signal-side flying
lead.  `pwm_audio_r` (FPGA J13 / AN9134 J1.36) is reserved.

### Behaviour

- **Q1 ON** (PWM = 1): `swnode` pulled to GND.  Current path
  +5V → speaker → Cout → Q1 → GND, charging Cout.
- **Q1 OFF** (PWM = 0): inductive flyback through D1 clamps `swnode`
  to +5 V; current re-circulates +5V → D1 → Cout → speaker → +5V.
- **At silence (PWM 50 % duty)**: Cout settles to ~2.5 V across it;
  speaker sees only AC at the switching frequency, filtered by its
  own voice-coil inductance.  No DC dissipation in the speaker.
- **At signal**: AC component swings the speaker.  Single-supply,
  single-ended → max output ≈ 0.4 W RMS into 8 Ω.  Plenty for ASC
  chime / system-sound program material; well under the 2 W speaker
  rating.

### Layout discipline

Keep the **Q1 source → Csup negative → speaker GND return** loop
physically tight (it's the high-di/dt path, the EMI hot spot).  TO-220
tab is electrically the drain — leave free or insulate; no heatsink
needed (worst-case dissipation < 100 mW under program material).

### Optional follow-on: passive line-out / OPA2132 buffer on J1.36

If a separate line-out is wanted, the unused `pwm_audio_r` pin (FPGA
J13 / J1.36) feeds a passive RC reconstruction filter into an OPA2132
non-inverting buffer driving a 3.5 mm jack.  RC values: 1 kΩ + 4.7 nF
gives `fc ≈ 33.9 kHz`; OPA2132 gain via standard non-inverting
network.  Out of scope for first light.

## 7. Test Plan

### `tb-audio-pwm` (new unit testbench)

Per `docs/agent_policy.md` ("when you modify or add a module, add or
extend its unit tb"):

1. **DC-zero stability** — drive `sample_l = sample_r = 0` for many ms of
   sim time, average the output, assert mean ≈ 0.5 (50% duty).
2. **Full-scale rails** — drive `+max_int16`, assert mean → 1.0; drive
   `-max_int16 - 1`, assert mean → 0.0.
3. **Audio-band fidelity** — drive a 1 kHz sine at half scale, FFT the
   1-bit output over a long enough window to resolve audio bins; assert
   the fundamental sits in the right bin and in-band noise floor is
   below a configurable threshold (start: −60 dBFS).
4. **Channel independence** — drive uncorrelated L/R, assert no
   cross-coupling.
5. **Reset cleanliness** — assert the modulator state clears on `rst`
   and the output settles back to 50% duty when input returns to zero.

Wire `tb-audio-pwm` into `make test` so regressions land loud.

### Top-level smoke

`make test TEST=audio_chime` (new): write a few hundred bytes of a known
chime PCM into the ASC FIFO via the existing peripheral-bus path, run
the bridge for the duration of the chime, capture the 1-bit waveform,
confirm it differs from the silence baseline. Lightweight — does not
need to validate audio quality, only that the data path is alive.

### Hardware bring-up checklist

1. **Pre-filter scope** — scope J1.35 directly during `make tb-rom-boot`
   bench load; confirm toggling at MHz rates (silence = mostly 50% duty,
   chime = visible amplitude modulation).
2. **Post-filter scope** — build the RC filter on perfboard, scope after
   C_LPF; confirm clean analog waveform.
3. **Headphone test** — plug in headphones, boot ROM, expect chime.

### Regression gates

- `make lint` floor unchanged (0 warnings).
- `make test` count: floor + 1 (`tb-audio-pwm`) — no other tests
  affected.
- `make fuzz N=200` unaffected (no decode/ALU/LSU touch).

## 8. Risks & Open Questions

| Item | Severity | Notes |
|------|----------|-------|
| ~~Carrier-side FPGA pin map for J1.35/36~~ | **resolved** | H13 (J1.35) and J13 (J1.36), per cross-reference of AN9134 manual + `synth/hdmi.xdc` + `~/40pin.xdc`. |
| Pin contention with carrier XDC | none observed | H13 / J13 do not appear in `synth/hdmi.xdc`, `synth/fpga_top.xdc`, `synth/fpga_top_real_mig.xdc`, or any other current XDC. They were the unique unconstrained pair on the 40-pin header. |
| EMI from MHz toggling on flying leads | low | `SLEW SLOW` mitigates. If audible interference shows up, add a 100 Ω + ferrite bead in series. Out of scope for first light. |
| ASC mono mode | none | ASC's mono path duplicates FIFO A into both `pcm_l` and `pcm_r` — `audio_pwm` is oblivious; mono just plays both channels. |
| Distortion from no interpolation filter | low | Stair-step + 22 kHz sample rate + 33.9 kHz analog cutoff means visible aliasing images above 22 kHz; inaudible. If ever a problem, add a half-band interpolator — out of scope. |
| Σ-Δ idle tones at DC | low | First-order is the loudest for tones; expect a faint whine when sample = 0 indefinitely. ASC silence still drains FIFOs at sample rate so this is bounded. Acceptable for first light. |
| Class-D MOSFET single-ended swing → DC bias on Cout | none | Cout settles to ~2.5 V at silence; speaker sees pure AC. No DC current through speaker at silence. |
| Output power ≪ 2 W speaker rating | accepted | ~0.4 W is the single-ended-from-5 V upper bound and is plenty for chime / system-sound program material. BTL upgrade (2× IRLZ54N) is the path to ~1.5 W if ever needed. |

## 9. Out of Scope

- Higher-order modulators (2nd-order MASH, etc.) — only consider if the
  first-order audio quality is unacceptable on bench.
- Interpolation FIR before the modulator — the ASC sample rate is so far
  below the OSR that the analog LPF handles it.
- Volume scaling — the ASC already applies its programmable volume
  before `audio_sample_out`; nothing to do here.
- Booting the I2S or HDMI paths — they remain in the tree, tied off in
  PWM builds, untouched.
- DAC daughtercard support — see option D in the brainstorm; deferred
  until / if a carrier with extra free pins shows up.
- Headphone-amp driver — output drives line-in or a powered amp / active
  speakers. Driving headphones directly off a 1 kΩ source impedance is
  weak; not a goal.

## 10. References

- ALINX AN9134 product page: <https://www.en.alinx.com/Product/Add-on-Modules/AN9134.html>
- AN9134 user manual (J1 pin table, confirms NC pins 35/36):
  <https://manuals.plus/alinx/an9134-hdmi-display-module-manual>
- SiI9134 datasheet (audio inputs not exposed by AN9134):
  <https://www.onwaytech.com/static/upload/file/20230921/1695276431555939.pdf>
- Existing audio bridge plumbing: `rtl/fpga_top_peripherals.vh:880-1052`
- ASC sample contract: `rtl/mac/asc.v` header
- AN9134 platform bring-up: `docs/an9134_50mhz_bringup.md`
