# bitstream_from_route.tcl -- rewrite bit/debug files from a routed checkpoint.
#
# Usage:
#   vivado -mode batch -source synth/bitstream_from_route.tcl \
#       -tclargs <route_dcp> <output_dir> [extra_xdc]
#
# This is intentionally narrower than route_from_place.tcl.  It lets bring-up
# iterate on post-route bitstream-visible board constraints, such as DDR4 IO
# electrical properties, without rerunning place/route.

set route_dcp  [lindex $argv 0]
set output_dir [lindex $argv 1]
set extra_xdc  [lindex $argv 2]

if {$route_dcp eq "" || $output_dir eq ""} {
    puts stderr "ERROR: usage: bitstream_from_route.tcl <route_dcp> <output_dir> [extra_xdc]"
    exit 1
}
if {![file exists $route_dcp]} {
    puts stderr "ERROR: route dcp not found: $route_dcp"
    exit 1
}
if {$extra_xdc ne "" && ![file exists $extra_xdc]} {
    puts stderr "ERROR: extra xdc not found: $extra_xdc"
    exit 1
}

set_param general.maxThreads 16

file mkdir $output_dir
file mkdir $output_dir/checkpoints
file mkdir $output_dir/reports

puts "=== OPENING ROUTED CHECKPOINT ==="
open_checkpoint $route_dcp

if {$extra_xdc ne ""} {
    puts "=== APPLYING EXTRA XDC: $extra_xdc ==="
    read_xdc $extra_xdc
}

puts "=== WRITING UPDATED CHECKPOINT ==="
write_checkpoint -force $output_dir/checkpoints/route_bitstream_rewrite.dcp

puts "=== FINAL REPORTS ==="
report_drc -file $output_dir/reports/drc_bitstream_rewrite.rpt
report_timing_summary \
    -max_paths 20 \
    -file $output_dir/reports/timing_bitstream_rewrite.rpt

puts "=== WRITING BITSTREAM ==="
source [file join [file dirname [info script]] adb_firmware_mmi.tcl]
validate_adb_firmware_bram
write_bitstream -force $output_dir/fpga_top.blank.bit
export_adb_firmware_bundle $output_dir/fpga_top.blank.bit
write_debug_probes -force $output_dir/fpga_top.ltx

puts "=== DONE ==="
puts "route dcp: $output_dir/checkpoints/route_bitstream_rewrite.dcp"
puts "bit:       $output_dir/fpga_top.blank.bit"
puts "ltx:       $output_dir/fpga_top.ltx"
