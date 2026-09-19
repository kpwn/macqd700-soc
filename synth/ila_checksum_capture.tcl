# synth/ila_checksum_capture.tcl — arm hw_ila to check whether real HW
# ever retires the checksum-loop exit instruction at ROM PC 0x40847522
# (task #91/checksum-retraction follow-up, 2026-07-05).
#
# Sim investigation this session chased "RTL computes the wrong 1MB ROM
# checksum" as the root cause of the whole boot-blocking chain, then
# retracted it as a chime-delay ROM-patch artifact (RTL's checksum
# computation is correct). This leaves an open question: does REAL
# hardware (unpatched, actual synthesized bitstream) even reach this
# exact PC before parking in the known STM/CTE wedge? break-pc has
# proven unreliable this session (fails to catch even guaranteed-early
# addresses like 0x4080008c), so use the already-loaded v7d ILA probe
# bitstream (build_id=0xd402a40d) instead, reusing its rob_pc probe with
# a fresh trigger condition -- no new synthesis needed.
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source synth/ila_checksum_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> [timeout_min]

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_checksum_capture.tcl <ltx_file> <jcmd_path> \[timeout_min\]"
    exit 2
}

set ltx_file    [lindex $argv 0]
set jcmd_path   [lindex $argv 1]
set timeout_min [expr {[llength $argv] >= 3 ? [lindex $argv 2] : 2.0}]

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

set p13 [get_hw_probes dbg_ila_rob_pc_w -of_objects $ila]
if {[llength $p13] == 0} {
    puts stderr "ERROR: expected probe dbg_ila_rob_pc_w not found."
    puts stderr "Available probes:"
    foreach p [get_hw_probes -of_objects $ila] { puts stderr "  $p" }
    exit 3
}

set_property CONTROL.TRIGGER_POSITION 100 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila

# rob_pc == 0x40847522 (checksum-loop exit/compare point). Single
# condition -- deterministic if reached at all, regardless of timing.
set_property TRIGGER_COMPARE_VALUE eq32'h40847522 $p13
puts "=== Trigger: rob_pc==0x40847522 (checksum-loop exit) ==="

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
puts "=== RESULT: TRIGGERED -- checksum-loop exit WAS reached on real HW ==="

catch {write_hw_ila_data -csv_file /tmp/ila_checksum_capture.csv -force $data}
puts "=== Wrote /tmp/ila_checksum_capture.csv (if supported) ==="

puts "=== DONE ==="
