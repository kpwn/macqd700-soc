# Standalone TQTT DDR4 calibration and address-range AXI test.
# Usage: vivado -mode batch -source synth/vivado_tqtt_ddr4_probe.tcl -tclargs build/tqtt_ddr4_probe
if {[lindex $argv 0] eq ""} { error "output directory required" }
set output_dir [file normalize [lindex $argv 0]]
set proj_root [file normalize [file dirname [info script]]/..]
set part xcku5p-ffvb676-2-i
set mig_dir $proj_root/build/ddr4_mig_tqtt
set mig_dcp $mig_dir/design_1_ddr4_0_1.dcp
set mig_stub $proj_root/rtl/board/vendor/tqtt_ddr4_mig_stub.v
foreach path [list $mig_dcp $mig_stub] {
    if {![file exists $path]} { error "TQTT MIG artifact missing: $path; run make tqtt-ddr4-mig" }
}
file mkdir $output_dir
file mkdir $output_dir/ip
file mkdir $output_dir/reports
file mkdir $output_dir/checkpoints
set_param general.maxThreads 8

file delete -force $output_dir/ip/tqtt_ddr4_vio
create_project -in_memory -part $part -force
create_ip -name vio -vendor xilinx.com -library ip -version 3.0 \
    -module_name tqtt_ddr4_vio -dir $output_dir/ip
set_property -dict [list \
    CONFIG.C_NUM_PROBE_IN {4} \
    CONFIG.C_NUM_PROBE_OUT {0} \
    CONFIG.C_PROBE_IN0_WIDTH {32} \
    CONFIG.C_PROBE_IN1_WIDTH {32} \
    CONFIG.C_PROBE_IN2_WIDTH {32} \
    CONFIG.C_PROBE_IN3_WIDTH {32} \
] [get_ips tqtt_ddr4_vio]
generate_target {synthesis} [get_ips tqtt_ddr4_vio]
synth_ip [get_ips tqtt_ddr4_vio]

read_verilog $mig_stub
read_verilog $proj_root/rtl/soc/tqtt_ddr4_probe.v
read_ip $output_dir/ip/tqtt_ddr4_vio/tqtt_ddr4_vio.xci
read_xdc $proj_root/synth/tqtt_ddr4.xdc
synth_design -top tqtt_ddr4_probe -part $part -no_iobuf
set mig_cells [get_cells -quiet -hier -filter {REF_NAME == design_1_ddr4_0_1}]
if {[llength $mig_cells] != 1} { error "expected one MIG black box, got: $mig_cells" }
set mig_cell [lindex $mig_cells 0]
read_checkpoint -cell $mig_cell $mig_dcp
set_property DONT_TOUCH false [get_cells u_mig]
set_property KEEP_HIERARCHY false [get_cells u_mig]
write_checkpoint -force $output_dir/checkpoints/synth.dcp
opt_design
place_design
phys_opt_design
route_design
report_timing_summary -max_paths 10 -file $output_dir/reports/timing_final.rpt
report_drc -file $output_dir/reports/drc_final.rpt
if {[llength [get_timing_paths -setup -max_paths 1 -slack_lesser_than 0.0]] ||
    [llength [get_timing_paths -hold -max_paths 1 -slack_lesser_than 0.0]]} {
    error "TQTT DDR4 probe timing failed; bitstream withheld"
}
write_checkpoint -force $output_dir/checkpoints/route.dcp
write_bitstream -force $output_dir/tqtt_ddr4_probe.bit
write_debug_probes -force $output_dir/tqtt_ddr4_probe.ltx
puts "TQTT DDR4 probe complete: $output_dir/tqtt_ddr4_probe.bit"
