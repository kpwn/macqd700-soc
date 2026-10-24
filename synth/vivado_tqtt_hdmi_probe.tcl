# Standalone TQTT HDMI test pattern; TQTT_HDMI_AUDIO=1 adds 48 kHz PCM.
if {[lindex $argv 0] eq ""} { error "output directory required" }
set output_dir [file normalize [lindex $argv 0]]
set proj_root [file normalize [file dirname [info script]]/..]
set part xcku5p-ffvb676-2-i
set audio 0
if {[info exists ::env(TQTT_HDMI_AUDIO)]} { set audio $::env(TQTT_HDMI_AUDIO) }
if {$audio ni {0 1}} { error "TQTT_HDMI_AUDIO must be 0 or 1" }
file mkdir $output_dir
file mkdir $output_dir/reports
file mkdir $output_dir/checkpoints
set_param general.maxThreads 8

read_verilog -sv [glob $proj_root/rtl/third_party/hdl_util_hdmi/*.sv]
read_verilog -sv $proj_root/rtl/soc/tqtt_hdmi_probe.sv
read_xdc $proj_root/synth/tqtt_hdmi_probe.xdc
synth_design -top tqtt_hdmi_probe -part $part -generic HDMI_AUDIO=$audio
set pixel_sync_input [get_pins -hier -filter {NAME =~ */pixel_meta_reg/D}]
if {[llength $pixel_sync_input] != 1} {
    error "expected one pixel-clock synchronizer input, got: $pixel_sync_input"
}
set_false_path -to $pixel_sync_input
if {$audio} {
    set audio_clk_reg [get_pins audio_clk_div_reg/Q]
    set audio_clk_bufg [get_pins u_audio_bufg/O]
    if {[llength $audio_clk_reg] != 1 || [llength $audio_clk_bufg] != 1} {
        error "expected one generated audio clock register and BUFG"
    }
    create_generated_clock -name tqtt_audio_48k \
        -source [get_pins audio_clk_div_reg/C] -divide_by 2625 $audio_clk_bufg
    # hdl-util/hdmi uses synchronizers for its audio-to-pixel packet CDC.
    set_clock_groups -asynchronous \
        -group [get_clocks tqtt_audio_48k] \
        -group [get_clocks {tqtt_user_clk100 pixel_unbuf pixel_x5_unbuf}]
}
write_checkpoint -force $output_dir/checkpoints/synth.dcp
opt_design
place_design
phys_opt_design
route_design
report_timing_summary -max_paths 10 -file $output_dir/reports/timing_final.rpt
if {[llength [get_timing_paths -setup -max_paths 1 -slack_lesser_than 0.0]] ||
    [llength [get_timing_paths -hold -max_paths 1 -slack_lesser_than 0.0]]} {
    error "HDMI timing failed; bitstream withheld"
}
write_checkpoint -force $output_dir/checkpoints/route.dcp
write_bitstream -force $output_dir/tqtt_hdmi_probe.bit
puts "TQTT HDMI probe complete (audio=$audio): $output_dir/tqtt_hdmi_probe.bit"
