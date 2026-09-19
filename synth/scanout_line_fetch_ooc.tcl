# scanout_line_fetch_ooc.tcl -- targeted out-of-context synthesis evidence for
# scanout_line_fetch.v, modelled on synth/dcache_ooc.tcl.
#
# Usage:
#   vivado -mode batch -source synth/scanout_line_fetch_ooc.tcl -tclargs build/slf_ooc
#
# WHY THIS EXISTS
# ---------------
# The 2026-07-31 LUT audit produced two irreconcilable figures for this module:
#   ~5,550 LUTs  (elimination argument: 95% of the module's flops are lb_word,
#                 and the ~69 logic-bearing lines can't account for the rest)
#   1,300-2,000  (mux-tree calculation: a 128:1 x 32-bit read mux is ~1,280 LUTs
#                 plus write decode)
# A 4.5x gap that neither analysis could close from source.  This settles it by
# measurement instead of arithmetic.
#
# PARAMETERS ARE NOT DEFAULTS -- READ THIS BEFORE CHANGING THEM
# -------------------------------------------------------------
# The module's own defaults are LINE_OFF_W=6 / LINE_IDX_W=15, which are NOT what
# the design instantiates.  The real values come from scanout_ddr_reader.v:
#   :176  localparam LINE_OFF_W = 7                -> LINE_BYTES=128, LINE_WORDS=32
#   :177  localparam LINE_IDX_W = ADDR_W - 7       -> 21-7 = 14 (ADDR_W=FB_ADDR_W=21,
#                                                     fpga_top_video.vh:449)
# and NUM_LINE_BUF=4 / DATA_WIDTH=128 / ID_WIDTH=6 pass through unoverridden from
# fpga_top_video.vh:448.
#
# Synthesising at the module defaults would size lb_word at 4x16x32 = 2,048 flops
# instead of the real 4x32x32 = 4,096 -- HALF the array, and therefore a number
# that answers a question nobody asked.  That failure mode (a measurement whose
# parameters switch off the thing being measured) is the single most common way
# a wrong conclusion has survived in this project.  If you edit these values,
# re-derive them from the instantiation chain first.

set output_dir [lindex $argv 0]
if {$output_dir eq ""} {
    set output_dir "build/slf_ooc"
}

set_param general.maxThreads 16

set proj_root [file normalize [file dirname [info script]]/..]
set out_dir   [file normalize $output_dir]
set rtl_dir   $proj_root/rtl
set part      "xcku5p-ffvb676-2-i"

file mkdir $out_dir

create_project -in_memory -part $part
set_property include_dirs [list \
    $rtl_dir/soc \
    $rtl_dir] [current_fileset]

read_verilog $rtl_dir/soc/scanout_line_fetch.v

# Real instantiated parameters -- see the header above.
synth_design -top scanout_line_fetch -part $part -mode out_of_context \
    -generic LINE_IDX_W=14 \
    -generic LINE_OFF_W=7 \
    -generic ID_WIDTH=6 \
    -generic NUM_LINE_BUF=4 \
    -generic DATA_WIDTH=128

# 200 MHz project target (5.0 ns), same as dcache_ooc.tcl.
create_clock -period 5.000 -name ooc_clk [get_ports clk]

report_utilization -hierarchical -file $out_dir/slf_ooc_util.rpt
report_timing_summary -max_paths 20 -file $out_dir/slf_ooc_timing.rpt
report_ram_utilization -file $out_dir/slf_ooc_ram.rpt
write_checkpoint -force $out_dir/slf_ooc_synth.dcp

# ── The headline numbers, echoed to stdout so the log alone answers the question
#
# NOTE (2026-07-31): do NOT filter on PRIMITIVE_GROUP / PRIMITIVE_SUBGROUP here.
# Those properties are populated for PLACED cells; after `synth_design` the
# design state is "Synthesized" and they come back empty, so the filters match
# nothing and print a confident **0** for both LUTs and FFs while the netlist is
# perfectly real.  That is exactly the silent-false-negative shape this project
# keeps getting bitten by, so it is worth the comment: filter on REF_NAME, and
# treat build/<dir>/slf_ooc_util.rpt as the authoritative number regardless.
set luts   [llength [get_cells -hier -filter {REF_NAME =~ LUT*}]]
set ffs    [llength [get_cells -hier -filter {REF_NAME =~ FD*}]]
set f7     [llength [get_cells -hier -filter {REF_NAME == MUXF7}]]
set f8     [llength [get_cells -hier -filter {REF_NAME == MUXF8}]]
set lutram [llength [get_cells -hier -filter {REF_NAME =~ RAM*X*}]]
set mem    [llength [get_cells -hier -filter {REF_NAME =~ RAMB* || REF_NAME =~ URAM*}]]

puts "SLF_OOC_TOTAL_LUT $luts"
puts "SLF_OOC_TOTAL_FF $ffs"
puts "SLF_OOC_MUXF7 $f7"
puts "SLF_OOC_MUXF8 $f8"
puts "SLF_OOC_LUTRAM $lutram"
puts "SLF_OOC_BRAM_URAM $mem"

# ── Attribute the LUTs to lb_word specifically.  This is the actual question:
# how much of the module is the 4x32x32 ring array's access fabric, versus the
# AXI fetch FSM / frame-wrap / error handling that the mux-tree estimate says
# must account for the remainder.
set lb_ff [llength [get_cells -hier -filter {NAME =~ *lb_word*}]]
puts "SLF_OOC_LB_WORD_CELLS $lb_ff"

# Expected from source: lb_word = NUM_LINE_BUF(4) x LINE_WORDS(32) x 32 bits
# = 4096 flops.  If SLF_OOC_TOTAL_FF is not ~4096 + a few hundred, the -generic
# overrides did not take and every number above is measuring the wrong module.
puts "SLF_OOC_EXPECTED_LB_FLOPS 4096"
if {$ffs < 4000} {
    puts "SLF_OOC_WARNING PARAMETER_OVERRIDE_MAY_NOT_HAVE_TAKEN ff_count=$ffs expected>=4096"
}
