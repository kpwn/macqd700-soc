#!/usr/bin/env tclsh
# Evaluate the exact profile guards used by the Vivado build, without Vivado.
set root [file dirname [file dirname [file normalize [info script]]]]
set fh [open "$root/synth/vivado.tcl" r]
set src [read $fh]
close $fh
set first [string first {set cpu_debug_profile } $src]
set last [string first {set enable_ipc_ila } $src]
if {$first < 0 || $last <= $first} { error "Cannot locate debug profile guards" }
set guards [string range $src $first [expr {$last - 1}]]
foreach cpu {0 1} {
    foreach profile {<unset> full reduced {} FULL lean} {
        set cpu_m68k040 $cpu
        unset -nocomplain ::env(CPU_DEBUG_PROFILE)
        if {$profile ne "<unset>"} { set ::env(CPU_DEBUG_PROFILE) $profile }
        set wanted [expr {$profile eq "<unset>" ? "full" : $profile}]
        set valid [expr {$wanted eq "full" || ($cpu && $wanted eq "reduced")}]
        set failed [catch {eval $guards} detail]
        if {$failed == $valid || (!$failed && $cpu_debug_profile ne $wanted)} {
            error "Debug profile guard failed: cpu=$cpu profile='$profile': $detail"
        }
        puts "PASS cpu=$cpu profile='$profile' accepted=[expr {!$failed}]"
    }
}
puts "CPU debug profile guards PASS"
