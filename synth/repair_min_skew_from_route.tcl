# Repair one routed input branch without perturbing the rest of a legal route.
#
# Usage:
#   vivado -mode batch -source synth/repair_min_skew_from_route.tcl \
#     -tclargs <route.dcp> <output-dir> <hierarchical-input-pin> <min-delay-ps>
#
# The caller must inspect timing_summary.rpt.  In particular, setup/hold alone
# are not sufficient: WPWS/TPWS must also be clean before using the bitstream.

if {$argc != 4} {
    puts stderr "ERROR: expected <route.dcp> <output-dir> <input-pin> <min-delay-ps>"
    exit 2
}

set input_dcp   [file normalize [lindex $argv 0]]
set output_dir  [file normalize [lindex $argv 1]]
set pin_name    [lindex $argv 2]
set min_delay_ps [lindex $argv 3]

if {![file exists $input_dcp]} {
    puts stderr "ERROR: input checkpoint does not exist: $input_dcp"
    exit 2
}
if {![string is double -strict $min_delay_ps] || $min_delay_ps < 0} {
    puts stderr "ERROR: min-delay-ps must be a non-negative number"
    exit 2
}

set_param general.maxThreads 16
file mkdir $output_dir
file mkdir $output_dir/checkpoints
file mkdir $output_dir/reports

open_checkpoint $input_dcp

set target_pin [get_pins -quiet $pin_name]
if {[llength $target_pin] != 1} {
    puts stderr "ERROR: expected exactly one target pin, got [llength $target_pin]: $pin_name"
    exit 2
}
set target_net [get_nets -quiet -of_objects $target_pin]
if {[llength $target_net] != 1} {
    puts stderr "ERROR: expected exactly one target net, got [llength $target_net]"
    exit 2
}

puts "=== PIN-SCOPED MIN-SKEW REPAIR ==="
puts "pin:          $target_pin"
puts "net:          $target_net"
puts "min delay ps: $min_delay_ps"

# Only remove the branch from the nearest existing route node to this input.
# The rest of the fully routed net and design remain intact.
route_design -unroute -pins $target_pin
route_design -pins $target_pin -min_delay $min_delay_ps
route_design -finalize

write_checkpoint -force $output_dir/checkpoints/route.dcp
report_timing_summary -max_paths 50 -report_unconstrained \
    -file $output_dir/timing_summary.rpt
report_route_status -file $output_dir/reports/route_status.rpt
report_drc -file $output_dir/reports/drc.rpt

puts "=== WRITING CANDIDATE ARTIFACTS ==="
source [file join [file dirname [info script]] adb_firmware_mmi.tcl]
validate_adb_firmware_bram
write_bitstream -force $output_dir/fpga_top.blank.bit
export_adb_firmware_bundle $output_dir/fpga_top.blank.bit
write_debug_probes -force $output_dir/fpga_top.ltx

puts "=== DONE: audit timing_summary.rpt including WPWS/TPWS before use ==="
close_design
