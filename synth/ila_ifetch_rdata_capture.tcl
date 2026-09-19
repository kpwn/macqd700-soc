# synth/ila_ifetch_rdata_capture.tcl -- round 12: real-hardware ILA capture
# of the FULL 256-bit RDATA payload parked in l2c.v's orphaned
# fetch-reassembly output skid, at the SAME permanent `if_pc_stuck_q`
# freeze rounds 8-11 (Parts 55-58) already characterized --
# docs/BUG_calibration_word_misplaced_0d00.md Part 58's precise closing
# recommendation (itself repeating Part 57 S9's own recommendation).
#
# BACKGROUND: rounds 9-11 structurally proved the downstream deadlock
# mechanism (IcachePlugin.scala's R-channel accept gate is ID/slot-indexed,
# not generation-indexed) and narrowed, without closing, the root TRIGGER
# to two remaining theories: (a) a cascading mistaken-identity accept --
# the data parked in l2c.v's f_rvalid_q/f_rdata_q skid is a STALE response
# for a DIFFERENT (earlier) fetch than the one currently armed in MSHR
# slot 1 -- or (b) the data genuinely IS the correct response for the
# currently-armed address, just never drained (pure downstream-propagation/
# backpressure, no mistaken identity). This round's one new probe (56, see
# rtl/soc/fpga_top_debug_vio.vh's round-12 declaration comment for the
# full rationale) settles this DIRECTLY and DISPOSITIVELY via a single
# static capture: compare the captured RDATA against the REAL ROM content
# at the address MSHR slot 1 currently believes it is waiting on
# (probe52/ila_ic_mshr_slot1_pa, looked up offline from the exact ROM
# image deployed on the SD card this session).
#
# PROBE MAPPING (this repo, round 12, C_NUM_OF_PROBES bumped 56->57; does
# NOT touch any prior round's mapping, which stays live in the SAME
# capture for cross-checking):
#
#   probe56 256b  ila_ifetch_rdata = ifa_rdata -- the full 256-bit
#       reassembled instruction-fetch beat currently parked in l2c.v's
#       orphaned output skid register (f_rdata_q), only meaningful while
#       probe46[8] (f_rvalid_q) reads 1 (true in every capture since
#       Part 55).
#
# DECISION PROCEDURE (single static capture, re-arm against the same
# already-frozen hang per every prior round's own precedent -- this exact
# stuck state has reproduced byte-identically round over round since
# Part 55):
#   - Read probe52 (ila_ic_mshr_slot1_pa) -- the physical address MSHR
#     slot 1 currently believes it is waiting on.
#   - Independently look up (OFFLINE, from the exact ROM image deployed on
#     the SD card this session) the 32 bytes (256 bits) of REAL ROM
#     content that byte address actually holds, applying the same
#     AXI_ROM_MIRROR_BASE/AXI_ROM_IMAGE_SIZE fold fpga_top_ddr.vh's own
#     `ifa_araddr_folded` applies.
#   - Compare against probe56 (ila_ifetch_rdata), byte-for-byte:
#       MATCH    -> hypothesis (a) RULED OUT for this capture -- the bug
#                   is a pure non-draining/backpressure defect (b).
#       MISMATCH -> DIRECT, DISPOSITIVE PROOF of hypothesis (a): the
#                   response sitting in the skid belongs to a DIFFERENT,
#                   necessarily EARLIER fetch than the one MSHR slot 1 is
#                   currently, actively waiting on.
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
#   vivado -mode batch -nojournal -nolog -source synth/ila_ifetch_rdata_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> [reset|noreset] [timeout_min]

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_ifetch_rdata_capture.tcl <ltx_file> <jcmd_path> \[reset|noreset\] \[timeout_min\]"
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
    puts stderr "ERROR: no hw_ila core found on device -- was ENABLE_ILA=1 CPU=m68k040 used, and is fpga_top.ltx current (round 12, C_NUM_OF_PROBES=57)?"
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

catch {write_hw_ila_data -csv_file /tmp/ila_ifetch_rdata_capture.csv -force $data}
puts "=== Wrote /tmp/ila_ifetch_rdata_capture.csv (if supported) ==="

puts "=== DONE ==="
