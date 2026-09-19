# synth/ila_bisect_pc.tcl — generic single-PC reachability check for
# real HW, bisecting against the now-validated-clean golden RTL/MAME
# trace (task #98, 2026-07-05). Triggers on rob_pc==<target>, fires a
# fresh reset, and waits. If it triggers, that PC IS reached on real HW.
# If it times out (NO_TRIGGER), real HW's path diverges before that PC.
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source synth/ila_bisect_pc.tcl \
#       -tclargs <ltx_file> <jcmd_path> <target_pc_hex> [timeout_min] [csv_path]

if {[llength $argv] < 3} {
    puts stderr "ERROR: usage: ila_bisect_pc.tcl <ltx_file> <jcmd_path> <target_pc_hex> \[timeout_min\] \[csv_path\]"
    exit 2
}

set ltx_file    [lindex $argv 0]
set jcmd_path   [lindex $argv 1]
set target_pc   [lindex $argv 2]
set timeout_min [expr {[llength $argv] >= 4 ? [lindex $argv 3] : 1.5}]
set csv_path    [expr {[llength $argv] >= 5 ? [lindex $argv 4] : "/tmp/ila_bisect_capture.csv"}]

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

set p13 [get_hw_probes dbg_ila_rob_pc_w -of_objects $ila]
if {[llength $p13] == 0} {
    puts stderr "ERROR: expected probe dbg_ila_rob_pc_w not found."
    exit 3
}

set_property CONTROL.TRIGGER_POSITION 100 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila
set_property TRIGGER_COMPARE_VALUE eq32'h${target_pc} $p13
puts "=== Trigger: rob_pc==0x${target_pc} ==="

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
    puts "=== RESULT: NO_TRIGGER (0x${target_pc} NOT reached -- no data object) ==="
    exit 4
}
catch {write_hw_ila_data -csv_file $csv_path -force $data} werr
puts "=== write_hw_ila_data returned: $werr ==="
puts "=== Wrote $csv_path -- CHECK ROW COUNT to confirm real trigger vs empty capture ==="
puts "=== DONE ==="
