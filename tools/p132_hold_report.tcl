#-----------------------------------------------------------------------------
# Part 132: measure the CPU clock domain's REAL hold (min-delay) slack.
#
# Every deployed bitstream is built with, from synth/vivado.tcl:
#
#     set_max_delay -from fabric_clk100 -to fabric_clk100 10.0 -datapath_only
#
# `-datapath_only` suppresses the MIN (hold) requirement on every path it
# covers, and at CORE_CLK_DIVIDE=1 `fabric_clk100` IS the core clock, so that
# exception covers the WHOLE CPU.  Vivado's own timing summary for the routed
# p123-reseed design confirms it verbatim:
#
#     From Clock:  fabric_clk100
#       To Clock:  fabric_clk100
#     Setup :  0  Failing Endpoints,  Worst Slack  0.912ns
#     Hold  : NA  Failing Endpoints,  Worst Slack     NA
#
# So the CPU has never had hold analysed, and therefore never had hold FIXED
# by the router, in any bitstream this repo has produced.  Every other clock
# domain sits at WHS +0.012..+0.025 ns -- the signature of a router that had
# to work to close hold -- while the CPU got none of that work.
#
# This script opens an already-ROUTED checkpoint (it builds nothing, writes no
# bitstream, touches no board), deletes that one exception, and re-runs timing
# so the hold numbers become real.  Read-only with respect to the build.
#
# Usage:
#   vivado -mode batch -source tools/p132_hold_report.tcl -tclargs <route.dcp> <outdir>
#-----------------------------------------------------------------------------

if {$argc < 2} {
    puts "ERROR: usage: -tclargs <route.dcp> <outdir>"
    exit 2
}
set dcp    [lindex $argv 0]
set outdir [lindex $argv 1]
file mkdir $outdir

puts "=== P132: opening $dcp ==="
open_checkpoint $dcp

# ---- 1. Baseline, exactly as built ---------------------------------------
report_timing_summary -max_paths 10 -file $outdir/00_baseline_summary.rpt
report_exceptions -file $outdir/01_exceptions_before.rpt

# ---- 2. Enumerate and delete the core-clock datapath_only max_delay -------
set killed 0
foreach e [get_timing_exceptions -quiet] {
    set nm [get_property -quiet NAME $e]
    set dp [get_property -quiet DATAPATH_ONLY $e]
    set md [get_property -quiet MAX_DELAY $e]
    puts "EXC: name=$nm datapath_only=$dp max_delay=$md"
    # Only the intra-core-clock datapath_only max_delay is removed.  Genuine
    # CDC exceptions (video / MIG / PCIe) are left completely alone.
    if {$dp eq "1"} {
        set froms [get_property -quiet FROM $e]
        set tos   [get_property -quiet TO   $e]
        puts "   -> from=$froms to=$tos"
        if {[string match "*fabric_clk100*" $froms] && [string match "*fabric_clk100*" $tos]} {
            puts "   -> DELETING (this is the CPU intra-domain hold suppressor)"
            delete_timing_constraint $e
            incr killed
        }
    }
}
puts "=== P132: deleted $killed exception(s) ==="
if {$killed == 0} {
    puts "WARNING: nothing deleted -- property names may differ on this Vivado build."
    puts "         Inspect $outdir/01_exceptions_before.rpt and adjust the filter."
}

# ---- 3. The real numbers -------------------------------------------------
report_exceptions -file $outdir/02_exceptions_after.rpt
report_timing_summary -delay_type min_max -max_paths 200 -report_unconstrained \
    -file $outdir/03_summary_with_hold.rpt
report_timing -delay_type min -max_paths 200 -sort_by slack -nworst 200 \
    -file $outdir/04_worst_hold_paths.rpt
report_timing -delay_type max -max_paths 50 -sort_by slack -nworst 50 \
    -file $outdir/05_worst_setup_paths_real.rpt

puts "=== P132: done, reports in $outdir ==="
exit 0
