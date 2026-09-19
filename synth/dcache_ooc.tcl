# dcache_ooc.tcl -- targeted D-cache out-of-context synthesis evidence.
#
# Usage:
#   vivado -mode batch -source synth/dcache_ooc.tcl -tclargs build/dcache_ooc
#
# This is intentionally not a full top-level synth/impl flow.  It reads only
# dcache.v plus include paths, runs synth_design -mode out_of_context with 16
# Vivado threads, and writes focused utilization/timing/RAM reports.

set output_dir [lindex $argv 0]
if {$output_dir eq ""} {
    set output_dir "build/dcache_ooc"
}

set_param general.maxThreads 16

set proj_root [file normalize [file dirname [info script]]/..]
set out_dir   [file normalize $output_dir]
set rtl_dir   $proj_root/rtl
set part      "xcku5p-ffvb676-2-i"

file mkdir $out_dir

create_project -in_memory -part $part
set_property include_dirs [list \
    $rtl_dir/core/decode \
    $rtl_dir/core \
    $rtl_dir] [current_fileset]

read_verilog $rtl_dir/core/decode/uop_pkg.v
read_verilog $rtl_dir/core/mem/dcache.v

synth_design -top dcache -part $part -mode out_of_context

create_clock -period 5.000 -name ooc_clk [get_ports clk]

report_utilization -hierarchical -file $out_dir/dcache_ooc_util.rpt
report_timing_summary -max_paths 20 -file $out_dir/dcache_ooc_timing.rpt
report_ram_utilization -file $out_dir/dcache_ooc_ram.rpt
write_checkpoint -force $out_dir/dcache_ooc_synth.dcp

set brams [get_cells -hier -filter {REF_NAME =~ RAMB* || REF_NAME =~ URAM*}]
puts "DCACHE_OOC_MEM_CELLS [llength $brams]"
foreach c $brams {
    set ref [get_property REF_NAME $c]
    set prim [get_property PRIMITIVE_TYPE $c]
    set doa ""
    set dob ""
    catch {set doa [get_property DOA_REG $c]}
    catch {set dob [get_property DOB_REG $c]}
    puts "DCACHE_OOC_MEM_CELL $c REF=$ref PRIM=$prim DOA_REG=$doa DOB_REG=$dob"
}

set out_regs [get_cells -hier -filter {NAME =~ *data_ram_out_q*}]
puts "DCACHE_OOC_DATA_RAM_OUT_REG_CELLS [llength $out_regs]"
foreach r $out_regs {
    puts "DCACHE_OOC_DATA_RAM_OUT_REG $r REF=[get_property REF_NAME $r] PRIM=[get_property PRIMITIVE_TYPE $r]"
}
