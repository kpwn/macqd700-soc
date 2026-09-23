#!/usr/bin/env tclsh
# Run the actual flow's post-route control block against deterministic timing
# observations. No Vivado invocation, design mutation or filesystem artifacts.
set root [file dirname [file dirname [file normalize [info script]]]]
set sourcePath [expr {[llength $argv] ? [lindex $argv 0] : "$root/synth/vivado.tcl"}]
set fh [open $sourcePath r]
set src [read $fh]
close $fh
set first [string first {proc _pr_wns } $src]
set last [string first {write_checkpoint -force $output_dir/checkpoints/route.dcp} $src]
if {$first < 0 || $last <= $first} { error "Cannot locate post-route control block" }
set block [string range $src $first [expr {$last - 1}]]
proc check_case {block name initial schedule expectedCalls expectedTiming} {
    set model [interp create]
    try {
        interp eval $model [list set timing $initial]
        interp eval $model [list set schedule $schedule]
        interp eval $model {
            set calls {}
            set checkpoints [dict create]
            set output_dir /unused-post-route-model
            set ::env(POST_ROUTE_PHYSOPT_MAX) 6
            proc puts {args} {} ;# Keep case results readable.
            proc get_timing_paths {args} {
                return [lindex $args [expr {[lsearch -exact $args -delay_type] + 1}]]
            }
            proc get_property {property kind} {
                if {$property ne "SLACK"} { error "Unexpected property" }
                switch $kind {
                    max { return [lindex $::timing 0] }
                    min { return [lindex $::timing 1] }
                    min_max { return [expr {min([lindex $::timing 0], [lindex $::timing 1])}] }
                    default { error "Unexpected timing query: $kind" }
                }
            }
            proc phys_opt_design {args} {
                lappend ::calls $args
                if {[llength $::schedule]} {
                    set next [lindex $::schedule 0]
                    set ::schedule [lrange $::schedule 1 end]
                    if {$next eq "error"} { error "Modeled optimizer failure" }
                    set ::timing $next
                }
            }
            proc write_checkpoint {args} {
                dict set ::checkpoints [lindex $args end] $::timing
            }
            proc close_design {} {}
            proc open_checkpoint {path} { set ::timing [dict get $::checkpoints $path] }
            rename file real_file
            proc file {op args} {
                switch $op {
                    mkdir { return }
                    exists { return [dict exists $::checkpoints [lindex $args 0]] }
                    default { return [real_file $op {*}$args] }
                }
            }
        }
        interp eval $model $block
        set calls [interp eval $model {set calls}]
        set timing [interp eval $model {set timing}]
        if {[llength $calls] != $expectedCalls || $timing ne $expectedTiming} {
            error "$name: expected $expectedCalls calls / $expectedTiming; got $calls / $timing"
        }
        puts "PASS $name: [llength $calls] optimizer calls; WNS/WHS $timing"
    } finally {
        interp delete $model
    }
}
check_case $block already_closed {0.032 0.008} {} 0 {0.032 0.008}
check_case $block hold_only_closure {0.032 -0.120} {{0.032 0.008}} 1 {0.032 0.008}
check_case $block setup_closure {-0.100 0.020} {{0.010 0.010}} 1 {0.010 0.010}
check_case $block smaller_positive_setup {0.100 -0.010} {{0.050 0.010}} 1 {0.050 0.010}
check_case $block zero_slack_is_closed {0.032 -0.120} {{0.000 0.000}} 1 {0.000 0.000}
check_case $block repair_closes {-0.100 0.050} {{0.020 -0.100} {0.020 0.010}} 2 {0.020 0.010}
check_case $block setup_plateau_still_failing {-0.100 0.010} {{-0.100 0.010} {0.020 0.010}} 2 {0.020 0.010}
check_case $block hold_still_failing {0.032 -0.120} {{0.032 -0.040} {0.032 0.008}} 2 {0.032 0.008}
check_case $block failed_repair_rolls_back {-0.100 0.050} {{-0.050 -0.100} {-0.040 -0.060} {0.010 0.050}} 3 {0.010 0.050}
check_case $block optimizer_error_continues {-0.100 0.010} {error {0.020 0.010}} 2 {0.020 0.010}
check_case $block bounded_plateau {-0.100 0.010} {} 6 {-0.100 0.010}
check_case $block final_hold_repair {0.032 -0.120} {
    {0.032 -0.120} {0.032 -0.120} {0.032 -0.120}
    {0.032 -0.120} {0.032 -0.120} {0.032 -0.120} {0.032 0.008}
} 7 {0.032 0.008}
puts "Post-route exit policy PASS"
