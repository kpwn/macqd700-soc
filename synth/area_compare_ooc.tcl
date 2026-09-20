# Matched, board-free synthesis census. Run under the project Vivado mutex.
# Arguments: baseline tree, candidate tree, output directory, top module.
if {[catch {
lassign $argv baseline candidate output top
if {$top eq ""} {error "usage: baseline candidate output top"}
set_param general.maxThreads 4
foreach label {baseline candidate} root [list $baseline $candidate] {
    create_project -in_memory -part xcku5p-ffvb676-2-i
    set_property include_dirs [list $root/rtl $root/rtl/mac $root/rtl/soc] [current_fileset]
    switch $top {
        video {read_verilog [list $root/rtl/mac/video.v $root/rtl/mac/mode_decode.v]}
        l2c_mshr {read_verilog [list $root/rtl/soc/l2c_mshr.v $root/rtl/soc/l2c_pri8.v]}
        boot_fsm {read_verilog [list $root/rtl/soc/boot_fsm.v $root/rtl/board/sd_ctrl.v]}
        default {error "unsupported top $top"}
    }
    set generics {}
    if {$top eq "boot_fsm"} {set generics {MIRROR_LOW_RAM=1 WORD_FIFO_LOG2P=6}}
    synth_design -top $top -part xcku5p-ffvb676-2-i -mode out_of_context \
        -directive PerformanceOptimized -keep_equivalent_registers -generic $generics
    create_clock -name clk -period 5 [get_ports clk]
    opt_design
    file mkdir $output/$label
    report_utilization -hierarchical -file $output/$label/utilization.rpt
    report_timing_summary -file $output/$label/timing.rpt
    write_checkpoint -force $output/$label/$top.dcp
    close_project
}
} failure]} {
    puts stderr $failure
    exit 1
}
