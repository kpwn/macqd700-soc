# synth/ila_move_sr_capture.tcl — arm hw_ila to bracket the SYS_MOVE_SR
# retire that corrupts A7/SR to 0x00000000 (task #89, v7b probes).
#
# Live break_pc bisection (coordinator) narrowed the corruption to a
# single instruction: `move (sp)+,sr` at ROM PC 0x408701AE.  Before it,
# A7/SR are sane (exc_count=0x12d0); immediately after, both read
# 0x00000000, with exactly one nested F-line exception (vec 0x0b, at an
# unrelated PC) taken inside that one instruction's execution window.
#
# This is a SYS_MOVE_SR retire (rob_uop_op == 6'd12), NOT an RTE, so the
# trigger targets probe44 (rob_uop_op) instead of the v6 probe33
# (take_rte_finalize).  Primary candidate: exc_count still == 0x12d0 at
# the moment SYS_MOVE_SR retires (the nested exception hasn't
# incremented it yet).
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source synth/ila_move_sr_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> <candidate_hex_or_any> <timeout_min>

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_move_sr_capture.tcl <ltx_file> <jcmd_path> \[candidate_hex|any\] \[timeout_min\]"
    exit 2
}

set ltx_file    [lindex $argv 0]
set jcmd_path   [lindex $argv 1]
set candidate   [expr {[llength $argv] >= 3 ? [lindex $argv 2] : "12d0"}]
set timeout_min [expr {[llength $argv] >= 4 ? [lindex $argv 3] : 1.5}]

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
    puts stderr "ERROR: no hw_ila core found on device."
    exit 3
}
set ila [lindex $ilas 0]
puts "=== ILA core: $ila ==="

set p44 [get_hw_probes ila_rob_uop_op -of_objects $ila]
set p13 [get_hw_probes dbg_ila_rob_pc_w -of_objects $ila]
if {[llength $p44] == 0 || [llength $p13] == 0} {
    puts stderr "ERROR: expected probes (ila_rob_uop_op, dbg_ila_rob_pc_w) not found."
    puts stderr "Available probes:"
    foreach p [get_hw_probes -of_objects $ila] { puts stderr "  $p" }
    exit 3
}

set_property CONTROL.TRIGGER_POSITION 3900 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila

# rob_uop_op == 6'd12 (SYS_MOVE_SR) == 6'h0c, at the KNOWN fixed ROM
# address of the corrupting instruction (`move (sp)+,sr` at 0x408701AE)
# -- PC-based trigger is deterministic regardless of real-time IRQ
# jitter, unlike an exc_count-based trigger.
set_property TRIGGER_COMPARE_VALUE eq6'h0c $p44
set_property TRIGGER_COMPARE_VALUE eq32'h408701ae $p13
puts "=== Trigger: rob_uop_op==12 (SYS_MOVE_SR) AND rob_pc==0x408701ae ==="

run_hw_ila $ila
puts "=== ARMED at [clock format [clock seconds]] ==="

after 300
puts "=== Firing reset via: $jcmd_path reset ==="
if {[catch {exec $jcmd_path reset} reset_out]} {
    puts "WARN: reset exec returned error/nonzero: $reset_out"
} else {
    puts "reset output:\n$reset_out"
}

puts "=== Waiting up to ${timeout_min} min for trigger ==="
wait_on_hw_ila -timeout $timeout_min $ila
puts "=== wait_on_hw_ila returned at [clock format [clock seconds]] ==="

if {[catch {upload_hw_ila_data $ila} uerr]} {
    puts "=== RESULT: NO_TRIGGER (upload threw: $uerr) ==="
    exit 4
}

set data [get_hw_ila_data -of_objects $ila]
puts "=== data object: $data ==="
puts "=== RESULT: ATTEMPTING_DUMP (caller must grep for 'No data to upload' WARNING above to confirm real vs empty) ==="

catch {write_hw_ila_data -csv_file /tmp/ila_move_sr_capture.csv -force $data}
puts "=== Wrote /tmp/ila_move_sr_capture.csv (if supported) ==="

puts "=== DONE ==="
