# impl_from_dcp.tcl — run opt/place/phys_opt/route/bitstream from an
# existing synth.dcp.  Sidesteps re-running synth_design when iterating
# on implementation-only tweaks.
#
# Usage:
#   vivado -mode batch -source impl_from_dcp.tcl -tclargs <synth_dcp> <output_dir>

set synth_dcp  [lindex $argv 0]
set output_dir [lindex $argv 1]

if {![file exists $synth_dcp]} {
    puts stderr "ERROR: synth dcp not found: $synth_dcp"
    exit 1
}

file mkdir $output_dir
file mkdir $output_dir/checkpoints
file mkdir $output_dir/reports

puts "=== OPENING SYNTH CHECKPOINT ==="
open_checkpoint $synth_dcp

puts "=== OPTIMISING ==="
opt_design -directive Explore

puts "=== PLACING ==="
place_design -directive ExtraPostPlacementOpt

write_checkpoint -force $output_dir/checkpoints/place.dcp
report_utilization  -file $output_dir/reports/utilization_placed.rpt
report_timing_summary \
    -max_paths 10 \
    -file $output_dir/reports/timing_place.rpt

puts "=== PHYS OPT ==="
phys_opt_design -directive AggressiveExplore

puts "=== ROUTING ==="
route_design -directive AggressiveExplore

write_checkpoint -force $output_dir/checkpoints/route.dcp

puts "=== FINAL REPORTS ==="
report_timing_summary \
    -max_paths 50 \
    -report_unconstrained \
    -file $output_dir/timing_summary.rpt

report_utilization \
    -hierarchical \
    -file $output_dir/reports/utilization_route.rpt

report_timing_summary \
    -max_paths 20 \
    -file $output_dir/reports/timing_route.rpt

report_drc \
    -file $output_dir/reports/drc.rpt

puts "=== WNS SUMMARY ==="
report_timing_summary -max_paths 1

puts "=== GENERATING BITSTREAM ==="
source [file join [file dirname [info script]] adb_firmware_mmi.tcl]
validate_adb_firmware_bram
write_bitstream -force $output_dir/fpga_top.blank.bit
export_adb_firmware_bundle $output_dir/fpga_top.blank.bit

puts "=== IMPLEMENTATION COMPLETE ==="
puts "Route checkpoint: $output_dir/checkpoints/route.dcp"
puts "Bitstream:        $output_dir/fpga_top.blank.bit"
