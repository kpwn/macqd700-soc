# Bounded synthesis/optimization check: no full SoC build or board access.
# vivado -mode batch -source synth/pic_bram_ooc.tcl -tclargs build/pic-bram-ooc
if {[catch {
set root [file normalize [file dirname [info script]]/..]
set output [lindex $argv 0]
if {$output eq ""} { error "output directory required" }
file mkdir $output
set_param general.maxThreads 4
create_project -in_memory -part xcku5p-ffvb676-2-i
read_verilog $root/rtl/mac/pic16c5x.v
synth_design -top pic16c5x -part xcku5p-ffvb676-2-i -mode out_of_context
# The integrated ADB modem runs on pb_clk=50 MHz, not the 200 MHz CPU clock.
create_clock -name pic_clk -period 20.0 [get_ports clk]
opt_design
set rom [get_cells -hier -filter {REF_NAME == RAMB36E2}]
if {[llength $rom] != 1} { error "PIC BRAM was lost or split" }
if {![regexp {^(1|1'b1|TRUE)$} [get_property IS_CLKARDCLK_INVERTED $rom]]} {
    error "PIC fetch edge changed: [get_property IS_CLKARDCLK_INVERTED $rom]"
}
if {[llength [get_cells -hier -filter {REF_NAME =~ FD*}]] < 50} {
    error "PIC execution logic appears to have been constant-folded"
}
write_checkpoint -force $output/pic.dcp
report_utilization -file $output/utilization.rpt
report_timing_summary -file $output/timing_synth.rpt
puts "PIC_BRAM_OOC_PASS: blank BRAM and CPU consumers survive optimization"
} failure]} {
    puts stderr $failure
    exit 1
}
