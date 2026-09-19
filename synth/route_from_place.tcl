# route_from_place.tcl -- resume the production implementation flow from a
# placed checkpoint after an interrupted build.
#
# Usage:
#   vivado -mode batch -source synth/route_from_place.tcl \
#       -tclargs <place.dcp> <output_dir> [build_id]

set place_dcp  [lindex $argv 0]
set output_dir [lindex $argv 1]
set build_id   [lindex $argv 2]

if {$place_dcp eq "" || $output_dir eq ""} {
    puts stderr "ERROR: usage: route_from_place.tcl <place.dcp> <output_dir> \[build_id\]"
    exit 1
}
if {![file exists $place_dcp]} {
    puts stderr "ERROR: place dcp not found: $place_dcp"
    exit 1
}

set_param general.maxThreads 16

file mkdir $output_dir
file mkdir $output_dir/checkpoints
file mkdir $output_dir/reports

puts "=== OPENING PLACED CHECKPOINT ==="
open_checkpoint $place_dcp

puts "=== PHYS OPT ==="
phys_opt_design -directive AggressiveExplore

# Match the production flow's post-phys-opt replication pass. These broad
# read-data cones are recurring congestion contributors in the CPU front end.
set rep_nets [get_nets -quiet -hier -filter \
    {NAME =~ "*dbg_overlay_q*" || NAME =~ "*crat_rb*" || NAME =~ "*vif_rdata*"}]
if {[llength $rep_nets] > 0} {
    puts "=== phys_opt -force_replication_on_nets on [llength $rep_nets] nets ==="
    phys_opt_design -force_replication_on_nets $rep_nets
}

puts "=== ROUTING ==="
if {[info exists ::env(ROUTE_DIRECTIVE)] && $::env(ROUTE_DIRECTIVE) ne ""} {
    puts "=== route_design -directive $::env(ROUTE_DIRECTIVE) (env override) ==="
    route_design -directive $::env(ROUTE_DIRECTIVE)
} else {
    route_design -directive Explore
}

# Route delay dominates the last setup violations on this design. Replay the
# same guarded post-route repair passes as the full production flow.
set setup_path [get_timing_paths -delay_type max -max_paths 1]
if {[llength $setup_path] > 0 && [get_property SLACK $setup_path] < 0} {
    puts "=== POST-ROUTE PHYS OPT (timing not met; attempting recovery) ==="
    if {[catch {phys_opt_design -directive AggressiveExplore} popt_err]} {
        puts "WARN: post-route phys_opt_design failed: $popt_err"
    }
    if {[catch {phys_opt_design -directive AlternateReplication} popt_err2]} {
        puts "WARN: post-route phys_opt_design (alt) failed: $popt_err2"
    }
} else {
    puts "=== POST-ROUTE PHYS OPT skipped: setup timing already met ==="
}

write_checkpoint -force $output_dir/checkpoints/route.dcp

puts "=== FINAL REPORTS ==="
report_timing_summary \
    -max_paths 50 \
    -report_unconstrained \
    -file $output_dir/timing_summary.rpt
report_utilization \
    -hierarchical \
    -file $output_dir/reports/utilization_route.rpt
report_power -file $output_dir/reports/power.rpt
report_drc -file $output_dir/reports/drc.rpt

puts "=== WNS SUMMARY ==="
report_timing_summary -max_paths 1

puts "=== WRITING BITSTREAM ==="
source [file join [file dirname [info script]] adb_firmware_mmi.tcl]
validate_adb_firmware_bram
write_bitstream -force $output_dir/fpga_top.blank.bit
export_adb_firmware_bundle $output_dir/fpga_top.blank.bit
write_debug_probes -force $output_dir/fpga_top.ltx

if {$build_id ne ""} {
    proc env_enabled {name} {
        return [expr {[info exists ::env($name)] && $::env($name) ne "0"}]
    }

    set buildinfo_file [file join $output_dir fpga_top.buildinfo]
    set buildinfo_fh [open $buildinfo_file w]
    puts $buildinfo_fh "part=[get_property PART [current_design]]"
    puts $buildinfo_fh "mode=full_impl_resume_from_place"
    puts $buildinfo_fh "real_fpga_build=1"
    puts $buildinfo_fh "ddr_path=real_mig"
    puts $buildinfo_fh "enable_vio=1"
    puts $buildinfo_fh "enable_jtag_axi=1"
    puts $buildinfo_fh "enable_pcie_xdma=0"
    puts $buildinfo_fh "host_debug=jtag_axi"
    puts $buildinfo_fh "target_freq_mhz=100.0"
    puts $buildinfo_fh "core_clk_hz=100000000"
    puts $buildinfo_fh "video_smoke=0"
    puts $buildinfo_fh "boot_rom_sectors=2048"
    puts $buildinfo_fh "l2c_enable=[env_enabled L2C_ENABLE]"
    puts $buildinfo_fh "vram_in_ddr=[env_enabled VRAM_IN_DDR]"
    puts $buildinfo_fh "build_id=$build_id"
    puts $buildinfo_fh "bitstream=fpga_top.blank.bit"
    puts $buildinfo_fh "adb_firmware=absent_requires_local_patch"
    puts $buildinfo_fh "debug_probes=fpga_top.ltx"
    puts $buildinfo_fh "generated_utc=[clock format [clock seconds] -gmt 1 -format {%Y-%m-%dT%H:%M:%SZ}]"
    close $buildinfo_fh
    puts "Build info: $buildinfo_file"
}

puts "=== DONE ==="
puts "route.dcp: $output_dir/checkpoints/route.dcp"
puts "bit:       $output_dir/fpga_top.blank.bit"
puts "ltx:       $output_dir/fpga_top.ltx"
