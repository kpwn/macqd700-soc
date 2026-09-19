# synth/ila_l2c_migcal_capture.tcl — L2C/MIG-cal-race investigation
# (2026-07-24). Programs the ILA-instrumented CPU=stub L2C bitstream,
# arms the debug_ila on probe42 bit[2] (mf_arvalid rising — the moment
# L2C issues its first miss-fill read into DRAM), then triggers a
# board reset and waits for the capture. Dumps probe41 (l2c_ctrl.v
# front-door state), probe42 (l2c.v master + miss-fill state), and
# probe43 (mig_cal_done) around the trigger point.
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source synth/ila_l2c_migcal_capture.tcl \
#       -tclargs <bit_file> <ltx_file> [timeout_min]

set bit_file    [lindex $argv 0]
set ltx_file    [lindex $argv 1]
set timeout_min [expr {[llength $argv] >= 3 ? [lindex $argv 2] : 3.0}]

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
set_property PROGRAM.FILE $bit_file $dev
set_property PROBES.FILE  $ltx_file $dev
program_hw_devices $dev
puts "=== Programmed $bit_file at [clock format [clock seconds]] ==="
refresh_hw_device $dev -update_hw_probes true

set ilas [get_hw_ilas -of_objects $dev -quiet]
if {[llength $ilas] == 0} {
    puts stderr "ERROR: no hw_ila core found on device."
    exit 3
}
set ila [lindex $ilas 0]
puts "=== ILA core: $ila ==="

set p41 [get_hw_probes dbg_l2c_write_snap -of_objects $ila -quiet]
set p42 [get_hw_probes dbg_l2c_master_snap -of_objects $ila -quiet]
set p43 [get_hw_probes mig_cal_done -of_objects $ila -quiet]
if {[llength $p41] == 0 || [llength $p42] == 0 || [llength $p43] == 0} {
    puts stderr "ERROR: expected probe41/42/43 not found. Available probes:"
    foreach p [get_hw_probes -of_objects $ila -quiet] { puts stderr "  $p" }
    exit 3
}
puts "=== probe41 = $p41 ==="
puts "=== probe42 = $p42 ==="
puts "=== probe43 = $p43 ==="

# 2026-07-24 revision: mf_arvalid (probe42 bit[2]) NEVER fired across a
# 3-minute capture window -- L2C's miss-fill path isn't even being
# reached. Back up to the FIRST possible signal in the whole sequence:
# probe41 (dbg_l2c_write_snap) bit[10] = s_awvalid -- does boot_fsm's
# write even reach the xbar S0 / L2C front door at all?  Shallow
# pre-trigger since we want to see what happens IMMEDIATELY after, not
# what led up to it (there's presumably very little "before" activity
# if this is the first real transaction).
set_property CONTROL.TRIGGER_POSITION 100 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila
# probe41 is {s_awvalid,s_awready,s_wvalid,s_wready,aw_have,rst_busy,
#             id_busy_c,write_avail,write_sel,st[1:0]} -- s_awvalid is
# bit[10] (the MSB of this 11-bit vector).
set_property TRIGGER_COMPARE_VALUE eq11'b1XXXXXXXXXX $p41
puts "=== Trigger: probe41 bit\[10\] (s_awvalid) == 1 ==="

run_hw_ila $ila
puts "=== ARMED at [clock format [clock seconds]] ==="

puts "=== Relying on the natural post-configuration reset sequence (STARTUPE3.EOS-gated) to drive the first boot attempt -- no manual VIO reset pulse sent ==="
puts "=== Waiting up to ${timeout_min} min for trigger ==="
wait_on_hw_ila -timeout $timeout_min $ila
puts "=== wait_on_hw_ila returned at [clock format [clock seconds]] ==="

if {[catch {upload_hw_ila_data $ila} uerr]} {
    puts "=== RESULT: NO_TRIGGER (upload threw: $uerr) ==="
    exit 4
}

set data [get_hw_ila_data -of_objects $ila]
puts "=== data object: $data ==="
puts "=== RESULT: TRIGGERED ==="

catch {write_hw_ila_data -csv_file /tmp/ila_l2c_migcal_capture.csv -force $data}
puts "=== Wrote /tmp/ila_l2c_migcal_capture.csv (if supported) ==="

puts "=== DONE ==="
