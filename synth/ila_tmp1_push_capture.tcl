# synth/ila_tmp1_push_capture.tcl — capture rat.v's ratmap[REG_TMP1] write
# port bracketing the "movew %sr,%sp@-" PUSH crack at 0x4086FF7A (task #91
# continuation, 2026-07-05). This is the ENTRY-side crack (phase0 ALU_SUB,
# phase1 SYS_MOVE_SR_DN TMP0->TMP1, phase2 STORE TMP1->(An)) -- a totally
# separate TMP1 user from the EXIT-side "move (sp)+,sr" pop crack at
# 0x408701AE that prior v7/v8 rounds already cleared of suspicion.
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source synth/ila_tmp1_push_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> [timeout_min] [csv_path]

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_tmp1_push_capture.tcl <ltx_file> <jcmd_path> \[timeout_min\] \[csv_path\]"
    exit 2
}

set ltx_file    [lindex $argv 0]
set jcmd_path   [lindex $argv 1]
set timeout_min [expr {[llength $argv] >= 3 ? [lindex $argv 2] : 2.0}]
set csv_path    [expr {[llength $argv] >= 4 ? [lindex $argv 3] : "/tmp/ila_tmp1_push_capture.csv"}]

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

set p_pc [get_hw_probes dbg_ila_rob_pc_w -of_objects $ila]
if {[llength $p_pc] == 0} {
    puts stderr "ERROR: expected probe dbg_ila_rob_pc_w not found."
    exit 3
}

set_property CONTROL.TRIGGER_POSITION 200 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila
set_property TRIGGER_COMPARE_VALUE eq32'h4086ff7a $p_pc
puts "=== Trigger: rob_pc==0x4086ff7a (shallow pretrigger, wide posttrigger for the push crack) ==="

puts "=== Holding CPU in reset via: $jcmd_path {reset hold} ==="
if {[catch {exec $jcmd_path "reset hold"} hold_out]} {
    puts "WARN: reset hold exec returned error/nonzero: $hold_out"
} else {
    puts "reset hold output:\n$hold_out"
}

run_hw_ila $ila
puts "=== ARMED (CPU still held) at [clock format [clock seconds]] ==="

after 300
puts "=== Releasing CPU from reset via: $jcmd_path {reset release} ==="
if {[catch {exec $jcmd_path "reset release"} release_out]} {
    puts "WARN: reset release exec returned error/nonzero: $release_out"
} else {
    puts "reset release output:\n$release_out"
}

puts "=== Waiting up to ${timeout_min} min for trigger ==="
wait_on_hw_ila -timeout $timeout_min $ila
puts "=== wait_on_hw_ila returned at [clock format [clock seconds]] ==="

catch {upload_hw_ila_data $ila} uerr
set data [get_hw_ila_data -of_objects $ila]
puts "=== data object: $data ==="
if {$data eq ""} {
    puts "=== RESULT: NO_TRIGGER ==="
    exit 4
}
catch {write_hw_ila_data -csv_file $csv_path -force $data} werr
puts "=== write_hw_ila_data returned: $werr ==="
puts "=== Wrote $csv_path -- CHECK ROW COUNT to confirm real trigger vs empty capture ==="
puts "=== DONE ==="
