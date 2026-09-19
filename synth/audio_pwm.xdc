# audio_pwm.xdc — Σ-Δ PWM audio output to AN9134 NC pins J1.35 / J1.36.
#
# Drives an off-board class-D speaker amp (single IRLZ54N + Schottky +
# AC-coupling cap → 8 Ω 2 W speaker) and/or a passive RC-filter line-out.
# See docs/superpowers/specs/2026-05-04-an9134-pwm-audio-design.md for
# the topology rationale and the full board build description.
#
# Pin derivation — cross-referencing three sources:
#   1. AN9134 user manual J1 pin table: J1.35 and J1.36 are NC (the
#      AN9134 daughtercard does not connect them to the SiI9134).
#   2. synth/hdmi.xdc: maps 32 of the 34 signal pins on the 40-pin
#      header from FPGA package pin to AN9134 signal name.
#   3. ~/40pin.xdc (carrier-side reference): lists all 34 signal pins
#      indexed 0..16 under {GPIO_0_tri_i, GPIO2_0_tri_o}.  Index N maps
#      to J1.(3+2N) for the input row and J1.(4+2N) for the output row.
#      The two highest-index pins (H13 and J13) are not present in
#      hdmi.xdc, which uniquely identifies them as the J1.35 / J1.36
#      pair.
#
#       J1.35 (NC) → carrier 40-pin row idx 16 (input row)  → FPGA H13
#       J1.36 (NC) → carrier 40-pin row idx 16 (output row) → FPGA J13

# ── Pin assignment ────────────────────────────────────────────────────
# AN9134 J1.35 (NC) → pwm_audio_l → speaker amp (mono first-light path)
# AN9134 J1.36 (NC) → pwm_audio_r → reserved for future line-out / OPA2132
set_property PACKAGE_PIN H13 [get_ports pwm_audio_l]
set_property PACKAGE_PIN J13 [get_ports pwm_audio_r]

# ── Electrical ────────────────────────────────────────────────────────
# LVCMOS33 to match the rest of synth/hdmi.xdc (same I/O bank as the
# AN9134 HDMI pins).  DRIVE 8 mA mirrors the HDMI RGB drive.  SLEW SLOW
# is intentional — the modulator output toggles at MHz rates and slow
# edges cut radiated EMI on the flying leads.
set_property IOSTANDARD LVCMOS33 [get_ports {pwm_audio_l pwm_audio_r}]
set_property DRIVE      8        [get_ports {pwm_audio_l pwm_audio_r}]
set_property SLEW       SLOW     [get_ports {pwm_audio_l pwm_audio_r}]

# ── Timing ────────────────────────────────────────────────────────────
# Analog endpoint — the off-board RC filter / class-D switch has no
# setup/hold requirement on individual edges; the modulator's behaviour
# is a long-time-average property, not an edge-aligned protocol.
set_false_path -to [get_ports {pwm_audio_l pwm_audio_r}]
