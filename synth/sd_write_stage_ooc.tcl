# Bounded, board-free BRAM inference check for the staged SD write path.
# Run under the project Vivado mutex, with an output directory argument.
if {[catch {
set root [file normalize [file dirname [info script]]/..]
set output [lindex $argv 0]
if {$output eq ""} {error "output directory required"}
file mkdir $output
set_param general.maxThreads 4
create_project -in_memory -part xcku5p-ffvb676-2-i
read_verilog $root/rtl/board/sd_ctrl.v
synth_design -top sd_ctrl -part xcku5p-ffvb676-2-i -mode out_of_context \
    -generic {WRITE_STAGE=1 READ_PIPELINE=1 MULTI_WRITE_AS_CMD24=0}
create_clock -name core_clk -period 5.0 [get_ports clk]
opt_design
set write_rams [get_cells -hier -filter {REF_NAME =~ RAMB* && NAME =~ *wr_sector_buf*}]
if {[llength $write_rams] != 1} {error "Expected one write staging BRAM; got $write_rams"}
report_utilization -file $output/utilization.rpt
report_timing_summary -file $output/timing_synth.rpt
write_checkpoint -force $output/sd_ctrl.dcp
puts "SD_WRITE_STAGE_OOC_PASS: one BRAM for both sector banks"
} failure]} {
    puts stderr $failure
    exit 1
}
