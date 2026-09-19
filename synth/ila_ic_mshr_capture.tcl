# synth/ila_ic_mshr_capture.tcl -- round 9: real-hardware ILA capture of the
# I-cache's per-slot MSHR control-file state (mshrValid/mshrArSent/
# mshrComplete/mshrErr/mshrPoison + FSM state + R-channel routing decision +
# slot 1's tracked physical address), at the SAME permanent
# `if_pc_stuck_q` freeze round 8 (Part 55) already characterized --
# docs/BUG_calibration_word_misplaced_0d00.md Part 55's precise closing
# recommendation.
#
# BACKGROUND: Part 55's round-8 capture found a real, permanent, two-sided
# AXI protocol deadlock on AXI ID=1 (IcachePlugin.scala's
# AxiIds.I_SPEC_BASE, the FIRST speculative/prefetch MSHR slot of 5): L2C
# has a fully-resolved cache-HIT fetch response for ID=1 sitting ready
# (probe46 hit_rsp_valid=1/hit_rsp_is_fetch=1, f_rvalid_q=1) that cpu040
# never drains (probe47 ifa_rready permanently 0), while cpu040
# SIMULTANEOUSLY holds a fresh, un-fired AR for the SAME id targeting a NEW
# address (probe48 ila_ifetch_araddr=0x40887240) that L2C's front door
# never accepts (probe47 ifa_arready permanently 0). DRAM/MIG proven
# completely idle (probe42 dbg_l2c_master_snap). Part 55 read
# IcachePlugin.scala directly and found every known speculative-slot free
# path (PREDECODE install-complete, error/poison teardown) gates on
# mshrComplete(e)==True FIRST -- which itself requires a genuine,
# protocol-legal 2-beat AXI R-channel fire (axi.r.ready actually high
# twice) -- but could not resolve statically whether this makes slot 1's
# bookkeeping self-consistent (pointing at L2C) or whether some other path
# corrupts it (pointing at cpu040). This round's three new probes (50/51/
# 52, see rtl/soc/fpga_top_debug_vio.vh's round-9 declaration comment for
# the full bit layout) answer this directly.
#
# PROBE MAPPING (this repo, round 9, C_NUM_OF_PROBES bumped 50->53; does
# NOT touch any prior round's mapping, which stays live in the SAME
# capture for cross-checking):
#
#   probe50 32b  ila_ic_mshr_snap:
#       [0]     reserved
#       [5:1]   mshrArSent[4:0]    (bit (1+i) = MSHR/AXI-id slot i)
#       [10:6]  mshrComplete[4:0]  (bit (6+i) = slot i)
#       [15:11] mshrErr[4:0]       (bit (11+i) = slot i)
#       [20:16] mshrPoison[4:0]    (bit (16+i) = slot i)
#       [23:21] fsmState (0=IDLE 1=REFILL 2=INSTALL_ARM 3=PREDECODE
#                          4=REPLAY 5=FAULT)
#       [24]    demandRspMatch  [25] pfRspMatch
#       [26]    ridIsDemand     [27] ridIsPf
#       [31:28] reserved
#     Slot 1 fields: mshrArSent[1]=bit2, mshrComplete[1]=bit7,
#     mshrErr[1]=bit12, mshrPoison[1]=bit17.
#   probe51 32b  ila_ic_mshr_snap2:
#       [4:0]   mshrValid[4:0] (bit i = slot i; slot 1 = bit 1)
#       [31:5]  reserved
#   probe52 32b  ila_ic_mshr_slot1_pa = mshrPa(AxiIds.I_SPEC_BASE) verbatim
#       -- the physical address MSHR slot 1 is CURRENTLY tracking.
#
# DECISION PROCEDURE (cross-reference against round 8's still-live probes
# 46-49 in the SAME capture):
#   - probe51[1] (mshrValid[1]) == 0 while probe46[2]/[3]
#     (hit_rsp_valid/hit_rsp_is_fetch) == 1 -> slot 1 was ALREADY FREED
#     while L2C still thinks a response for it is outstanding. Since the
#     only two IcachePlugin.scala free paths both require
#     mshrComplete(1)==True first, this means the core correctly drained
#     an OLDER id=1 transaction and L2C's own fq_have/f_rvalid_q skid
#     never cleared for it -- POINTS AT L2C (rtl/soc/l2c.v), NOT cpu040.
#   - probe51[1] == 1 (slot 1 still valid) AND probe52
#     (ila_ic_mshr_slot1_pa) == probe48 (ila_ifetch_araddr) -> slot 1 was
#     NEVER reallocated; the AR in flight IS for the exact same occupancy
#     whose response is stuck at L2C -- POINTS AT L2C (a front-door/
#     response-drain bug on a single continuous occupancy); cpu040's own
#     bookkeeping is self-consistent.
#   - probe51[1] == 1 AND probe52 != probe48, with probe50[2]
#     (mshrArSent[1]) == 0 -> slot 1 WAS reallocated to a NEW target
#     (mshrPa/mshrSet/mshrTag rewritten by the pfFreeMshr arm) while an
#     EARLIER occupancy's response was still live at L2C -- the smoking
#     gun for a genuine cpu040-side premature-free/reallocation-before-
#     drain bug. If ALSO probe50[7] (mshrComplete[1]) reads 0 for the
#     WHOLE captured pre-trigger history (never observed transitioning to
#     1), that is a SECOND, deeper finding: the slot was freed via a path
#     other than the two known mshrComplete-gated free sites, worth its
#     own follow-up round before writing an RTL fix.
#
# METHOD: identical two-phase undisturbed-boot discipline as every prior
# round. Prefer Part 53's re-arm-against-an-already-stuck-CPU technique
# over a fresh cold boot whenever a board from a prior trial is still
# sitting frozen at the target PC (the CPUSHL hang, per every round since
# Part 52, is permanent once reached).
#
# TRIGGER: reuses round 8's own `ila_if_pc_stuck[16]` sticky detector
# (unchanged this round -- no new trigger source needed, the hang
# mechanism is the same one round 8 already isolated), TRIGGER_POSITION
# near the END of the buffer (mostly pre-trigger), same reasoning as
# round 8's own capture script.
#
# **`wait_on_hw_ila -timeout N` is in MINUTES, not seconds.**
#
# Usage: identical calling convention to synth/ila_ifetch_axi_capture.tcl
# (see that file's own usage comment) --
#   vivado -mode batch -nojournal -nolog -source synth/ila_ic_mshr_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> [reset|noreset] [timeout_min]

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_ic_mshr_capture.tcl <ltx_file> <jcmd_path> \[reset|noreset\] \[timeout_min\]"
    exit 2
}

set ltx_file    [lindex $argv 0]
set jcmd_path   [lindex $argv 1]
set do_reset    [expr {[llength $argv] >= 3 ? [lindex $argv 2] : "reset"}]
set timeout_min [expr {[llength $argv] >= 4 ? [lindex $argv 3] : 2.0}]

puts "=== Opening Vivado hw_manager ==="
open_hw_manager
connect_hw_server -allow_non_jtag

set targets [get_hw_targets]
if {[llength $targets] == 0} {
    puts stderr "ERROR: no JTAG targets discovered."
    exit 2
}
current_hw_target [lindex $targets 0]
open_hw_target

set dev [lindex [get_hw_devices] 0]
current_hw_device $dev
set_property PROBES.FILE $ltx_file $dev
refresh_hw_device $dev -update_hw_probes true

set ilas [get_hw_ilas -of_objects $dev]
if {[llength $ilas] == 0} {
    puts stderr "ERROR: no hw_ila core found on device -- was ENABLE_ILA=1 CPU=m68k040 used, and is fpga_top.ltx current (round 9, C_NUM_OF_PROBES=53)?"
    exit 3
}
set ila [lindex $ilas 0]
puts "=== ILA core: $ila ==="

set p_trig [get_hw_probes ila_if_pc_stuck -of_objects $ila]
if {[llength $p_trig] == 0} {
    puts stderr "ERROR: expected trigger probe (ila_if_pc_stuck) not found."
    puts stderr "Available probes:"
    foreach p [get_hw_probes -of_objects $ila] { puts stderr "  $p" }
    exit 3
}

set_property CONTROL.TRIGGER_POSITION 3968 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila
set_property TRIGGER_COMPARE_VALUE eq17'b1XXXXXXXXXXXXXXXX $p_trig
puts "=== Trigger: ila_if_pc_stuck\[16\] (if_pc_stuck_q) == 1 -- fetch PC unchanged for >=512 cycles -- position=3968/4096 (mostly pre-trigger) ==="

run_hw_ila $ila
puts "=== ARMED at [clock format [clock seconds]] ==="

if {$do_reset eq "reset"} {
    after 300
    puts "=== Firing reset via: $jcmd_path reset ==="
    if {[catch {exec $jcmd_path reset} reset_out]} {
        puts "WARN: reset exec returned error/nonzero: $reset_out"
    } else {
        puts "reset output:\n$reset_out"
    }
} else {
    puts "=== NOT resetting (noreset) -- assumes the board is ALREADY stuck (re-arm-against-live-hang technique, Part 53 S6) ==="
}

puts "=== Waiting up to ${timeout_min} min for trigger (NOTE: MINUTES, not seconds) ==="
wait_on_hw_ila -timeout $timeout_min $ila
puts "=== wait_on_hw_ila returned at [clock format [clock seconds]] ==="

puts "=== Attempting upload ==="
if {[catch {upload_hw_ila_data $ila} uerr]} {
    puts "=== RESULT: NO_TRIGGER (upload threw: $uerr) -- the stuck condition was never reached in the window ==="
    exit 4
}

set data [get_hw_ila_data -of_objects $ila]
puts "=== data object: $data ==="
puts "=== RESULT: ATTEMPTING_DUMP ==="

catch {write_hw_ila_data -csv_file /tmp/ila_ic_mshr_capture.csv -force $data}
puts "=== Wrote /tmp/ila_ic_mshr_capture.csv (if supported) ==="

puts "=== DONE ==="
