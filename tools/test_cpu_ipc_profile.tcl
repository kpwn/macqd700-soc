#!/usr/bin/env tclsh
# Exercise the actual Vivado profile guards before acquiring the Vivado lock.
# An optional source path permits verifying that the pre-fix flow is rejected.
set root [file dirname [file dirname [file normalize [info script]]]]
set sourcePath [expr {[llength $argv] ? [lindex $argv 0] : "$root/synth/vivado.tcl"}]
set fh [open $sourcePath r]
set src [read $fh]
close $fh
set first [string first {set cpu_ipc_profile } $src]
set last [string first {set perf_detail_enable } $src]
if {$first < 0 || $last <= $first} { error "Cannot locate Vivado profile guards" }
set guards [string range $src $first [expr {$last - 1}]]
proc check_profile {guards cpu profile expected expectedValue} {
    set cpu_m68k040 $cpu
    unset -nocomplain ::env(CPU_IPC_PROFILE)
    if {$profile ne "<unset>"} { set ::env(CPU_IPC_PROFILE) $profile }
    set failed [catch {eval $guards} detail]
    if {$failed == $expected || (!$failed && $cpu_ipc_profile ne $expectedValue)} {
        error "Profile check failed: cpu040=$cpu profile='$profile': $detail"
    }
    puts "PASS cpu040=$cpu profile='$profile' accepted=[expr {!$failed}]"
}
foreach profile {baseline throughput-v1 throughput-v2} {
    check_profile $guards 1 $profile 1 $profile
}
check_profile $guards 1 <unset> 1 baseline
check_profile $guards 0 baseline 1 baseline
foreach profile {throughput-v1 throughput-v2} {
    check_profile $guards 0 $profile 0 {}
}
foreach profile {{} throughput throughput-v3 THROUGHPUT-V2} {
    check_profile $guards 1 $profile 0 {}
}
puts "CPU IPC profile guards PASS"
