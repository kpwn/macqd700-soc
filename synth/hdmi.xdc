# hdmi.xdc -- HDMI (AL9134 SiI9134) pin + timing constraints for m68k-ooo.
#
# Copied from ~/sd-hdmi-bringup/synth/sd_hdmi.xdc (AL9134 section only --
# system clock, reset, LEDs, buttons, SD pins stay in their respective
# owning XDCs).  Lives alongside the future ddr4.xdc / sd.xdc so multiple
# peripheral-integration agents can edit constraints concurrently without
# conflicting.  The mac-top-integrator is responsible for read_xdc'ing
# this file (and the others) from synth/vivado.tcl.
#
# Port names match rtl/mac/video/video_top.v.

# ── AL9134 I2C ────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN D10 [get_ports al9134_scl]
set_property PACKAGE_PIN D11 [get_ports al9134_sda]
set_property IOSTANDARD  LVCMOS33 [get_ports al9134_scl]
set_property IOSTANDARD  LVCMOS33 [get_ports al9134_sda]
set_property PULLUP true [get_ports al9134_scl]
set_property PULLUP true [get_ports al9134_sda]
set_false_path -to   [get_ports al9134_scl]
set_false_path -to   [get_ports al9134_sda]
set_false_path -from [get_ports al9134_sda]

# ── AL9134 control ────────────────────────────────────────────────────────────
set_property PACKAGE_PIN D13 [get_ports al9134_clk]
set_property PACKAGE_PIN F12 [get_ports al9134_de]
set_property PACKAGE_PIN H14 [get_ports al9134_hs]
set_property PACKAGE_PIN G14 [get_ports al9134_vs]
set_property PACKAGE_PIN J14 [get_ports al9134_resetn]
set_property PACKAGE_PIN J15 [get_ports al9134_int]
set_property IOSTANDARD LVCMOS33 [get_ports {al9134_clk al9134_de al9134_hs al9134_vs al9134_resetn}]
set_property IOSTANDARD LVCMOS33 [get_ports al9134_int]
set_false_path -from [get_ports al9134_int]
set_false_path -to   [get_ports al9134_resetn]

# ── AL9134 RGB data ───────────────────────────────────────────────────────────
# R[7:0] -> al9134_d[23:16]
set_property PACKAGE_PIN E11 [get_ports {al9134_d[23]}]
set_property PACKAGE_PIN E10 [get_ports {al9134_d[22]}]
set_property PACKAGE_PIN C11 [get_ports {al9134_d[21]}]
set_property PACKAGE_PIN B11 [get_ports {al9134_d[20]}]
set_property PACKAGE_PIN D9  [get_ports {al9134_d[19]}]
set_property PACKAGE_PIN C9  [get_ports {al9134_d[18]}]
set_property PACKAGE_PIN B9  [get_ports {al9134_d[17]}]
set_property PACKAGE_PIN A9  [get_ports {al9134_d[16]}]
# G[7:0] -> al9134_d[15:8]
set_property PACKAGE_PIN B10 [get_ports {al9134_d[15]}]
set_property PACKAGE_PIN A10 [get_ports {al9134_d[14]}]
set_property PACKAGE_PIN A13 [get_ports {al9134_d[13]}]
set_property PACKAGE_PIN A12 [get_ports {al9134_d[12]}]
set_property PACKAGE_PIN B14 [get_ports {al9134_d[11]}]
set_property PACKAGE_PIN A14 [get_ports {al9134_d[10]}]
set_property PACKAGE_PIN C14 [get_ports {al9134_d[9]}]
set_property PACKAGE_PIN C13 [get_ports {al9134_d[8]}]
# B[7:0] -> al9134_d[7:0]
set_property PACKAGE_PIN C12 [get_ports {al9134_d[7]}]
set_property PACKAGE_PIN B12 [get_ports {al9134_d[6]}]
set_property PACKAGE_PIN D14 [get_ports {al9134_d[5]}]
set_property PACKAGE_PIN E12 [get_ports {al9134_d[4]}]
set_property PACKAGE_PIN E13 [get_ports {al9134_d[3]}]
set_property PACKAGE_PIN F13 [get_ports {al9134_d[2]}]
set_property PACKAGE_PIN F14 [get_ports {al9134_d[1]}]
set_property PACKAGE_PIN G12 [get_ports {al9134_d[0]}]

set_property IOSTANDARD LVCMOS33    [get_ports {al9134_d[*]}]
set_property DRIVE      8           [get_ports {al9134_d[*] al9134_clk al9134_de al9134_hs al9134_vs}]
set_property SLEW       FAST        [get_ports {al9134_d[*] al9134_clk al9134_de al9134_hs al9134_vs}]

# PACK the RGB/control output registers into HDIOLOGIC (2026-09-13).
#
# This was FALSE for the 148.5 MHz bring-up path: xcku5p-ffvb676-2-i reports an
# 8 ns min-period on HDIOLOGIC FFs (125 MHz), so IOB TRUE caused pulse-width
# violations at 1080p60.  **That objection is gone** -- the pixel clock is now
# 99.0 MHz (10.101 ns), inside the rating.  See the 1280x960@60 note in
# rtl/soc/fpga_top_video.vh.
#
# With the flops in fabric, each bit's launch-to-pad distance was whatever
# placement gave it, so inter-bit skew on a 24-bit source-synchronous bus was
# unconstrained in practice even though set_output_delay below checked it.
# Packing them into the IOBs makes every bit launch from identical I/O logic,
# alongside the ODDRE1 that now forwards al9134_clk from the same column --
# so clock and data share matched paths and skew becomes a device constant.
set_property IOB        TRUE        [get_ports {al9134_d[*] al9134_de al9134_hs al9134_vs}]

# Output timing budget for AL9134 (matches hdmi-bringup, verified working).
# Match the video MMCM instance by suffix so pre-stitch synthesis still finds
# it, but the DDR4 MIG DCP cannot widen this to unrelated CLKOUT0 pins.
# ── Source-synchronous output modelling (2026-08-19) ──────────────────────
# al9134_clk is the pixel clock forwarded straight to a pin
# (video_top.v:702 `assign al9134_clk = pclk`).  ODDR/HDIOLOGIC clock
# forwarding is unavailable here for the same 8 ns min-period reason as the
# IOB FALSE above, so the BUFG-distributed clock drives an OBUF directly.
#
# Consequence: the launch flops see the FULL clock-network insertion delay,
# but a set_output_delay referenced to the INTERNAL MMCM output is analysed
# against a reference that does not.  Vivado charges the difference as skew.
# The 2026-08-19 route reported Clock Path Skew -2.299ns (SCD 4.685 /
# DCD 2.655) and 7 al9134_d[*] endpoints failing at WNS -0.126ns -- while the
# interface works correctly on hardware.  In a real source-synchronous
# interface the clock exits through that same delay, so it cancels.
#
# Declaring the forwarded clock ON THE PORT and constraining the data against
# IT models the interface correctly: data-at-pin vs clock-at-pin.
# NOTE: .xdc files are parsed in a restricted mode -- `if` is rejected with
# [Designutils 20-1307], so this must be plain unconditional constraints.
# al9134_clk is a top-level port of fpga_top, so it always resolves.
# SOURCE THE FORWARDED CLOCK FROM THE ODDRE1's C PIN (2026-09-13), not from the
# MMCM output.  The MMCM form was correct while `al9134_clk` was a BUFG driving an
# OBUF directly, because then the port really was a raw copy of CLKOUT0.  It is
# WRONG for the ODDRE1 structure now in video_top.v: the clock leaves through the
# ODDR + OBUF in the IOB, and the data leaves through IOB flops + OBUF.  Referencing
# the data to a clock object that omits the ODDR/OBUF leg makes STA compare two
# paths that do not physically exist together, and it charges the difference as
# skew -- which showed up as WHS -1.726 with 27 failing hold endpoints (10 of them
# al9134_d[*]) on the first ODDRE1 build, even though SETUP closed completely.
#
# NOTE ON THE PIN NAME: Vivado TRANSFORMS the ODDRE1 into an OSERDESE3 placed in
# HDIOLOGIC (confirmed on the routed netlist: REF=OSERDESE3, LOC=HDIOLOGIC_S_X0Y43),
# and that primitive's clock pin is `CLK`, not `C`.  Filtering for `/C` silently
# matches NOTHING, which leaves the old MMCM-sourced definition in place and looks
# like the fix did not work.
#
# Sourcing from the ODDR's own clock pin is the standard source-synchronous idiom: the
# generated clock is defined where the forwarded clock is actually launched, so
# clock and data are compared over their real, matched IOB paths.
# `-invert` IS LOAD-BEARING: video_top's ODDRE1 forwards the clock INVERTED
# (D1=0/D2=1) so its rising edge lands in the MIDDLE of the data eye.  Without
# `-invert` STA believes the forwarded clock rises at t=0, the same instant the RGB
# flops launch, and then:
#   * SETUP is checked launch@0 -> capture@period, giving an absurd +8.043 ns of
#     margin (a whole 10.101 ns cycle) -- which LOOKS like the interface is fine;
#   * HOLD is checked launch@0 -> capture@0 and fails by -2.037 ns on 27 endpoints.
# Declaring the inversion moves the capture edge to 5.051 ns, which is where it
# physically is, and the half-cycle of intended margin appears on BOTH checks.
# NOT -invert: u_pclk_fwd forwards its own clock IN PHASE (D1=1/D2=0).  It carried
# -invert while the ODDR was D1=0/D2=1; both flipped together, and they must stay
# consistent -- a generated clock whose polarity disagrees with the ODDR reports
# timing against an edge the hardware does not have.
#
# ⚠️ THE -source PIN NOW CARRIES pclk_fwd, NOT pclk (2026-09-15).  video_top clocks
# u_pclk_fwd from the MMCM's CLKOUT1, which is pclk shifted +90 deg.  This
# definition needs NO change for that -- and that is the point of sourcing at the
# ODDR's own clock pin.  The shift arrives through the master clock Vivado derives
# for CLKOUT1, so `report_clocks` should show al9134_clk_fwd with a master of the
# CLKOUT1-derived clock and its waveform a quarter period (3.367 ns) later than
# pclk's.  If it still shows pclk as the master, the RTL change did not take and
# every margin claimed below is fiction -- check that before reading any slack.
create_generated_clock -name al9134_clk_fwd -divide_by 1 \
    -source [get_pins -hierarchical -filter {NAME =~ "*u_video/u_pclk_fwd/CLK"}] \
    [get_ports al9134_clk]
# ── WHICH EDGE THE ENCODER CAPTURES ON: BOTH, BECAUSE IT IS UNRESOLVED ────
#                                                          (rewritten 2026-09-15)
#
# The 2026-09-14 form of this block carried ONLY `-clock_fall`, inferred from the
# premise "in-phase forwarding demonstrably works on hardware". That premise is
# FALSE as of 720p60: in-phase forwarding is exactly the configuration now showing
# a stable one-pixel per-channel displacement on the board. The inference has to go.
#
# WHAT IS ACTUALLY MEASURED, on build/vivado200_ra route.dcp (report_datasheet,
# xcku5p-ffvb676-2-i, pad delays from the same launch edge):
#
#     al9134_d[*] / de / hs / vs   8.930 - 8.973 ns (SLOW)   4.759 - 4.805 ns (FAST)
#     al9134_clk  rising           9.629 ns        (SLOW)    4.523 ns        (FAST)
#     Vivado's own bus-skew figure for al9134_d[23:0]:  0.043 ns
#
#   => with the ODDR clocked IN PHASE, the forwarded clock's RISING edge sits
#      +0.656..+0.699 ns (SLOW) / -0.236..-0.282 ns (FAST) from the data
#      transition. It is ON the transition, and the SIGN FLIPS with corner.
#
# So the old constraint was not wrong to report ~-1.7 ns against the rising edge.
# That violation was REAL; `-clock_fall` silenced a true statement about the
# hardware. (Cross-check: the as-routed worst setup slack against the falling edge
# is +4.798 ns; 4.798 - 6.734 = -1.936 ns against the rising edge -- the same
# number that was dismissed as phantom.)
#
# WHAT THE ENCODER ACTUALLY REQUIRES, from the SiI9134 data sheet SiI-DS-0193-F
# (input latch timing, replacing the previous ASSUMED -0.5 ns hold budget):
#     EDGE=1, rising latch :  TSIDR setup 1.0 ns   THIDR hold 0.5 ns
#     EDGE=0, falling latch:  TSIDF setup 1.0 ns   THIDF hold 0.8 ns
# -max 1.0 was already right and is now sourced. -min is tightened -0.5 -> -0.8,
# the worse of the two documented holds, because WHICH EDGE IS NOT ESTABLISHED:
#   * i2c_init.v's comment claims rising via reg 0x0A bit 0 -- but the SiI9134 has
#     no TPI (Lattice SiI9136-3 DS SiI-DS-1084-C, Table 2.1) and 0x0A is not the
#     edge register in either the legacy or the TPI map. That comment is wrong.
#   * In the SiI164/SiI9034 legacy lineage the edge bit is reg 0x08 bit 1, and
#     this design writes 0x08 <- 0x35, which has that bit CLEAR = falling.
#   * The board behaves as though it were RISING. That is unexplained, and the
#     SiI9134 programmer's reference could not be found, publicly or locally.
# See the register note in rtl/board/video_phy/i2c_init.v for the full state.
#
# THE RESOLUTION IS STRUCTURAL, NOT A GUESS. video_top forwards the clock from an
# MMCM output shifted +90 deg (a QUARTER period), so BOTH edges clear every data
# transition. Confirmed in the netlist of build/vivado200_closure:
# u_video/u_mmcm/u_mmcm CLKOUT1_PHASE = 90.000, and the MMCM pin delays differ by
# 2.61 ns (Prop_MMCM_CLKIN1_CLKOUT0 0.630 ns vs Prop_MMCM_CLKIN1_CLKOUT1 3.240 ns).
#
# ── BOTH EDGES ARE CONSTRAINED, AND THE RISING ONE NEEDS A MULTICYCLE ──────
#                                                                     (2026-09-16)
# `create_generated_clock -divide_by 1` sourced at the ODDR's clock pin does NOT
# put the +90 deg into the generated clock's WAVEFORM. Vivado normalises that to
# {rise@0.000 fall@6.734} -- identical to the launch clock -- and carries the shift
# as clock path DELAY instead:
#
#     pclk_fwd_unbuf  {0.000 6.734}  master .../u_mmcm/CLKOUT1
#     al9134_clk_fwd  {0.000 6.734}  master pclk_fwd_unbuf
#     Destination Clock Delay (DCD)  10.702 - 11.787 ns   (forwarded clock at pad)
#     Source Clock Delay      (SCD)   5.267 -  5.330 ns   (launch clock at flop)
#
# Latency is the physically correct place for it, and the FALLING-edge checks pair
# correctly because fall@6.734 is genuinely half a period after the launch edge.
# The modelled RISING edge, though, sits at phase zero, coincident with the launch
# edge. With identical waveforms Vivado applies its default same-waveform pairing
# -- setup launch N -> capture N+1, hold launch N -> capture N -- which for this
# interface is off by one cycle in BOTH directions at once:
#
#     setup rise@13.468  ->  +15.068 ns   a whole extra period of phantom margin
#     hold  rise@0.000   ->   -4.547 ns   demands the data change AFTER an edge
#                                         that arrives 2.27-4.07 ns INTO the eye
#
# Physically, capture N samples launch N here: the rising edge leaves the die
# 3.367 ns late by construction and reaches the pad at 11.787 ns, while the data
# settles at 7.713 ns. So the setup capture edge must be pulled back one cycle.
# UG903 "Multicycle Paths and Clock Phase-Shift" is the named case: the timing
# engine "selects launch and capture edges that produce the stricter setup
# constraint", and when that choice does not match intent you correct it with a
# setup multiplier -- "The hold edge derives from the setup change. You do not need
# to specify it." Its worked example moves setup FORWARD (multiplier 2) for a
# destination clock that is phase-shifted late; ours needs the mirror image,
# multiplier 0, because the shift is carried as latency rather than waveform.
#
# `-rise_to` scopes it to the rising captures only, so the already-correct
# falling-edge pair is untouched. MEASURED on build/vivado200_closure route.dcp,
# before and after, all four families:
#
#                              baseline            with the multicycle
#     setup rise    req 13.468  +15.068 .. +15.269   req  0.000  +1.600 .. +1.818
#     hold  rise    req  0.000   -4.547 ..  -4.490   req-13.468  +8.921 .. +8.963
#     setup fall    req  6.734   +8.334 ..  +8.552   req  6.734  +8.334 .. +8.535
#     hold  fall    req -6.734   +2.187 ..  +2.229   req -6.734  +2.187 .. +2.244
#
# All four positive, worst +1.600 ns. The falling pair does not move at all, which
# is the check that `-rise_to` scoped correctly. Against the SiI9134 data sheet's
# TSIDR/THIDR 1.0/0.5 ns and TSIDF/THIDF 1.0/0.8 ns, the interface is now verified
# by STA on BOTH candidate capture edges -- which was the point of the quarter-
# period shift, and is why the encoder's unresolved edge no longer matters.
#
# ⚠️ DO NOT ADD AN EXPLICIT HOLD MULTIPLIER. Tested on the same checkpoint: adding
# `set_multicycle_path -1 -hold -end -rise_to ...` on top of this puts the hold
# check straight back to req 0.000 / -4.547 ns. UG903 means it literally -- the
# hold edge is derived, and specifying it overrides the derivation.
#
# ⚠️ ORDER MATTERS: this must come AFTER the create_generated_clock above, or
# `get_clocks al9134_clk_fwd` returns empty and the multicycle silently applies to
# nothing -- which looks exactly like the -4.547 ns regression. No `-from` clock is
# named deliberately: nothing else is in this clock's domain, and naming the
# auto-derived `pclk_unbuf` would make the constraint depend on a net name in
# mmcm_hdmi.v that a rename could silently break.
#
# ⚠️ AND: these constraints are the only thing standing between phys_opt and 27
# output pins. If they report large violations they steal optimisation effort from
# the core clock's real shortfall. Read the video-pin slack in the build report
# before trusting the core WNS -- that is exactly how the -4.547 was caught.
set_output_delay -clock [get_clocks al9134_clk_fwd]             -max  1.0 [get_ports {al9134_d[*] al9134_de al9134_hs al9134_vs}]
set_output_delay -clock [get_clocks al9134_clk_fwd]             -min -0.8 [get_ports {al9134_d[*] al9134_de al9134_hs al9134_vs}]
set_output_delay -clock [get_clocks al9134_clk_fwd] -clock_fall -add_delay -max  1.0 [get_ports {al9134_d[*] al9134_de al9134_hs al9134_vs}]
set_output_delay -clock [get_clocks al9134_clk_fwd] -clock_fall -add_delay -min -0.8 [get_ports {al9134_d[*] al9134_de al9134_hs al9134_vs}]
set_multicycle_path 0 -setup -end -rise_to [get_clocks al9134_clk_fwd]

# ── WHY THERE IS NO set_bus_skew HERE ─────────────────────────────────────
# [[al9134-rgb-bus-skew-2026-09-08]] records "~1 ns of unconstrained skew across
# the 24-bit bus" and asks for a `set_bus_skew`. Both halves are now retired:
#
#   1. THE SKEW IS NOT ~1 ns. report_datasheet on the routed 200 MHz design gives
#      "Bus Skew: 0.043 ns" for al9134_d[23:0] (0.046 ns at the FAST corner), and
#      0.043 ns across all 27 outputs including de/hs/vs. The ~1 ns figure predates
#      `IOB TRUE` and the 148.5 -> 74.25 MHz move. It is 0.3% of the 13.468 ns
#      pixel period and 50x smaller than the 2.27 ns edge clearance above, so it is
#      not a contributor to anything.
#
#   2. `set_bus_skew` CANNOT BE WRITTEN FOR THIS INTERFACE. It is a CDC constraint;
#      its endpoints must be pins, cells or clocks. Tried on the routed design:
#        set_bus_skew -from <launch flops> -to [get_ports {al9134_d[*]}] 0.5
#          ERROR: [Constraints 18-612] ... 'to' does not contain any object of
#          type(s) '(pin,cell,clock)' ... The constraint will not be applied.
#        set_bus_skew -from <launch flops> -to [get_clocks al9134_clk_fwd] 0.5
#          ERROR: [Constraints 18-612] ... likewise rejected.
#      Adding it would put a hard ERROR in the XDC read, not a skew bound.
#
# Inter-bit skew on an output bus is already bounded by the four set_output_delay
# constraints above: every bit is checked against the same forwarded-clock edge, so
# the spread between bits shows up directly as a spread in their slacks (as routed:
# 0.149 ns setup, 0.143 ns hold across the 27 pins). `IOB TRUE` is what keeps it
# there. There is nothing further to constrain.
#
# Also measured, so nobody hunts for it again: this device has NO delay primitives
# to trim individual bits with -- `get_sites -filter {SITE_TYPE =~ *ODELAY*}`
# returns 0, and the al9134 pins are all in HDIO bank 87 (HDIOB_M/HDIOB_S), which
# has no IDELAY/ODELAY. Per-bit delay trim is not an option on this pinout.
