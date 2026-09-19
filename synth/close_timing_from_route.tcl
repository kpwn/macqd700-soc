# Resume physical timing closure from an existing routed checkpoint.
#
# Usage:
#   vivado -mode batch -source synth/close_timing_from_route.tcl \
#     -tclargs <input-route.dcp> <output-dir> <phys-opt-directive|none> \
#              ?post-route-constraints.tcl?
#
# This deliberately never overwrites the source implementation.  A checkpoint
# and reports are always emitted for comparison; bit/LTX artifacts are emitted
# only when both setup and hold timing close.

if {$argc != 3 && $argc != 4} {
    puts stderr "ERROR: expected <input-route.dcp> <output-dir> <phys-opt-directive|none> ?post-route-constraints.tcl?"
    exit 2
}

set input_dcp  [file normalize [lindex $argv 0]]
set output_dir [file normalize [lindex $argv 1]]
set directive  [lindex $argv 2]
set post_route_constraints ""
if {$argc == 4} {
    set post_route_constraints [file normalize [lindex $argv 3]]
    if {![file exists $post_route_constraints]} {
        puts stderr "ERROR: post-route constraint script does not exist: $post_route_constraints"
        exit 2
    }
}

if {![file exists $input_dcp]} {
    puts stderr "ERROR: input checkpoint does not exist: $input_dcp"
    exit 2
}

file mkdir $output_dir
file mkdir $output_dir/reports
file mkdir $output_dir/checkpoints

open_checkpoint $input_dcp

if {$post_route_constraints ne ""} {
    puts "=== APPLYING POST-ROUTE CONSTRAINTS: $post_route_constraints ==="
    source $post_route_constraints
}

proc worst_slack {delay_type} {
    set paths [get_timing_paths -delay_type $delay_type -max_paths 1]
    if {[llength $paths] == 0} {
        error "no $delay_type timing path found"
    }
    return [get_property SLACK $paths]
}

set initial_wns [worst_slack max]
set initial_whs [worst_slack min]
puts [format "CLOSE_TIMING_INITIAL WNS=%.3f WHS=%.3f" \
    $initial_wns $initial_whs]

if {$directive eq "none"} {
    puts "=== POST-ROUTE PHYS_OPT: skipped ==="
} else {
    puts "=== POST-ROUTE PHYS_OPT: $directive ==="
    phys_opt_design -directive $directive
}

set final_wns [worst_slack max]
set final_whs [worst_slack min]
puts [format "CLOSE_TIMING_FINAL directive=%s WNS=%.3f WHS=%.3f" \
    $directive $final_wns $final_whs]

write_checkpoint -force $output_dir/checkpoints/route.dcp
report_timing_summary -max_paths 50 -report_unconstrained \
    -file $output_dir/timing_summary.rpt
report_route_status -file $output_dir/reports/route_status.rpt
report_drc -file $output_dir/reports/drc.rpt

if {$final_wns >= 0.0 && $final_whs >= 0.0} {
    puts "=== TIMING CLOSED: writing bitstream and debug probes ==="
    source [file join [file dirname [info script]] adb_firmware_mmi.tcl]
    validate_adb_firmware_bram
    write_bitstream -force $output_dir/fpga_top.blank.bit
    export_adb_firmware_bundle $output_dir/fpga_top.blank.bit
    write_debug_probes -force $output_dir/fpga_top.ltx
    puts [format "CLOSE_TIMING_RESULT PASS WNS=%.3f WHS=%.3f" \
        $final_wns $final_whs]
} else {
    puts [format "CLOSE_TIMING_RESULT FAIL WNS=%.3f WHS=%.3f" \
        $final_wns $final_whs]
}

close_design
