# synth/ila_ic_mshr_gen_capture.tcl -- round 10: real-hardware ILA capture
# of per-slot MSHR generation/tag counters + a direct L2C fetch-ID tap, at
# the SAME permanent `if_pc_stuck_q` freeze rounds 8/9 (Parts 55/56)
# already characterized -- docs/BUG_calibration_word_misplaced_0d00.md
# Part 56's precise closing recommendation.
#
# BACKGROUND: Part 56's round-9 capture structurally PROVED the downstream
# deadlock mechanism (IcachePlugin.scala's R-channel accept gate is
# ID/slot-indexed, not generation-indexed -- once an AXI ID is reused for a
# new MSHR occupancy while an older transaction on that same ID is still
# outstanding, there is categorically no RTL path to ever drain it) but
# could NOT settle the root TRIGGER: is cpu040 reallocating slot 1 before
# the prior occupancy's response genuinely drained (a cpu040-side bug), or
# is this a genuine L2C-side response-tracking defect? This round's three
# new probes (53/54/55, see rtl/soc/fpga_top_debug_vio.vh's round-10
# declaration comment for the full bit layout) answer this directly via a
# per-slot generation counter (mshrGen, incremented on every fresh
# allocation) cross-referenced against the generation of the LAST AR
# actually accepted on the bus for that slot (mshrArSentGen, NOT reset at
# allocation) plus a direct L2C f_rid_q/hit_rsp_id tap.
#
# PROBE MAPPING (this repo, round 10, C_NUM_OF_PROBES bumped 53->56; does
# NOT touch any prior round's mapping, which stays live in the SAME
# capture for cross-checking):
#
#   probe53 32b  ila_ic_mshr_gen = mshrGen[4:0], 4 bits/slot, slot i in
#       bits [4*i+3:4*i] -- slot 1's generation is bits [7:4].
#   probe54 32b  ila_ic_mshr_arsent_gen = mshrArSentGen[4:0], same packing.
#       Latched to mshrGen(i) at the exact cycle axi.ar.fire sends slot i's
#       AR -- NOT reset at allocation, so it holds the generation number of
#       the LAST AR actually accepted for this slot across any later
#       reallocation.
#   probe55 16b  ila_l2c_fetch_id_snap (l2c.v):
#       [15] f_rvalid_q (output skid occupied)
#       [14] fetch_pick_hit (a fresh fetch-tagged hit is live this cycle)
#       [13:8] hit_rsp_id[5:0] (raw ID_WIDTH tag of that fresh hit)
#       [7:4] fq_idx (ID that fresh completion targets)
#       [3:0] f_rid_q (ID CURRENTLY parked in the output skid register)
#
# DECISION PROCEDURE (cross-reference against round 9's still-live probes
# 50-52 in the SAME capture):
#   - Read slot 1's nibble out of probe53 and probe54 (bits [7:4] of each).
#     If probe54[7:4] != probe53[7:4] WHILE probe50[2] (mshrArSent[1])
#     reads 0 -> DEFINITIVE PROOF of a cpu040-side reallocate-before-drain:
#     the currently-armed occupancy of slot 1 is a STRICTLY NEWER
#     generation than the last one whose AR was actually accepted onto the
#     bus. This settles Part 56's S8 root-trigger question against
#     cpu040 (cpu040 IS the trigger).
#   - If probe54[7:4] == probe53[7:4] instead (no reallocation happened
#     since the last AR-accept on this slot), the orphaned response cannot
#     be a cpu040-side premature-free artifact -- the root trigger must be
#     explained on the L2C side instead. Cross-check probe55[3:0]
#     (f_rid_q) == 1 to confirm L2C's own output skid genuinely believes it
#     is holding an ID=1 response, and probe55[14] (fetch_pick_hit) to see
#     whether L2C's front door is STILL actively trying to serve a fresh
#     ID=1 hit this cycle (would point at a genuine L2C response-tracking
#     defect).
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
#   vivado -mode batch -nojournal -nolog -source synth/ila_ic_mshr_gen_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> [reset|noreset] [timeout_min]

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_ic_mshr_gen_capture.tcl <ltx_file> <jcmd_path> \[reset|noreset\] \[timeout_min\]"
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
    puts stderr "ERROR: no hw_ila core found on device -- was ENABLE_ILA=1 CPU=m68k040 used, and is fpga_top.ltx current (round 10, C_NUM_OF_PROBES=56)?"
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

catch {write_hw_ila_data -csv_file /tmp/ila_ic_mshr_gen_capture.csv -force $data}
puts "=== Wrote /tmp/ila_ic_mshr_gen_capture.csv (if supported) ==="

puts "=== DONE ==="
