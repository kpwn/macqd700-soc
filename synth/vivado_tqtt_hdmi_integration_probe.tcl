# Route the exact full-Mac HDMI packetizer with independent 720p bars and tone.
if {[lindex $argv 0] eq ""} { error "output directory required" }
set output_dir [file normalize [lindex $argv 0]]
set use_fabric [lindex $argv 1]
set control_skew [lindex $argv 2]
if {$use_fabric eq ""} { set use_fabric 0 }
if {$control_skew eq ""} { set control_skew 0 }
if {$use_fabric ni {0 1}} { error "USE_FABRIC_CLOCK must be 0 or 1" }
if {$control_skew ni {0 5}} { error "CONTROL_SKEW_CYCLES must be 0 or 5" }
set root [file normalize [file dirname [info script]]/..]
file mkdir $output_dir $output_dir/reports
set_param general.maxThreads 8
read_verilog -sv [glob $root/rtl/third_party/hdl_util_hdmi/*.sv]
foreach source {
    rtl/board/async_fifo.v
    rtl/board/video_phy/mmcm_hdmi.v
    rtl/board/video_phy/vtg.v
    rtl/board/video_phy/tqtt_hdmi_audio_video.sv
    rtl/soc/tqtt_hdmi_integration_probe.sv
} { read_verilog -sv $root/$source }
read_xdc $root/synth/tqtt_hdmi_probe.xdc
read_xdc $root/synth/tqtt_hdmi_integration_probe.xdc
synth_design -top tqtt_hdmi_integration_probe -part xcku5p-ffvb676-2-i \
    -generic [list USE_FABRIC_CLOCK=$use_fabric CONTROL_SKEW_CYCLES=$control_skew]
set pixel_sync_input [get_pins -hier -filter {NAME =~ */pixel_meta_reg/D}]
if {[llength $pixel_sync_input] != 1} {
    error "expected one pixel-clock synchronizer input, got: $pixel_sync_input"
}
set_false_path -to $pixel_sync_input
if {!$use_fabric} {
    create_generated_clock -name probe_pb_50m \
        -source [get_pins direct_source.pb_div_reg/C] -divide_by 2 \
        [get_pins direct_source.u_pb_bufg/O]
}
create_generated_clock -name probe_audio_48k -source [get_pins u_hdmi/audio_clk_div_reg/C] \
    -divide_by 1042 [get_pins u_hdmi/u_audio_bufg/O]
set_clock_groups -asynchronous -group [get_clocks probe_audio_48k] \
    -group [get_clocks -filter {NAME != probe_audio_48k}]
opt_design
place_design
phys_opt_design
route_design
for {set i 0} {$i < 3} {incr i} {
    if {![llength [get_timing_paths -hold -max_paths 1 -slack_lesser_than 0.0]]} {break}
    phys_opt_design -hold_fix
    route_design
}
report_timing_summary -max_paths 10 -file $output_dir/reports/timing_final.rpt
if {[llength [get_timing_paths -setup -max_paths 1 -slack_lesser_than 0.0]] ||
    [llength [get_timing_paths -hold -max_paths 1 -slack_lesser_than 0.0]]} {
    error "HDMI integration probe timing failed; bitstream withheld"
}
write_bitstream -force $output_dir/tqtt_hdmi_integration_probe.bit
puts "TQTT HDMI integration probe complete (fabric=$use_fabric skew=$control_skew): $output_dir/tqtt_hdmi_integration_probe.bit"
