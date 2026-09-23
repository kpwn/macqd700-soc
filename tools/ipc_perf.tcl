# Source in the JTAG REPL. Snapshot pairs measure only running counter cycles;
# snapshots freeze counters, not the CPU, and preserve the prior run state.
proc ipc_perf_snapshot {} {
    set cap [dbg_rd 0x10b4]
    if {$cap eq "" || $cap != 0xd1011706} {
        error "Detailed ROB+dispatch counters unavailable (capability=$cap)"
    }
    set ctl [dbg_rd $::OFF_PERF_CTL]
    if {$ctl eq ""} { error "Failed performance control read" }
    dbg_wr $::OFF_PERF_CTL 0
    try {
        set snap [dict create]
        foreach name {cycle inst} lo [list $::OFF_PERF_CYCLE_LO $::OFF_PERF_INST_LO] {
            set low [dbg_rd $lo]; set high [dbg_rd [expr {$lo + 4}]]
            if {$low eq "" || $high eq ""} { error "Failed $name counter read" }
            dict set snap $name [expr {($high << 32) | $low}]
        }
        set words {}
        for {set n 0} {$n < 29} {incr n} {
            set value [dbg_rd [expr {0x1100 + 4*$n}]]
            if {$value eq ""} { error "Failed detailed counter $n read" }
            lappend words $value
        }
        dict set snap detail $words
        return $snap
    } finally {
        dbg_wr $::OFF_PERF_CTL [expr {$ctl & 2}]
    }
}
proc ipc_perf_delta {before after} {
    set cycles [expr {([dict get $after cycle] - [dict get $before cycle]) & 0xffffffffffffffff}]
    set inst [expr {([dict get $after inst] - [dict get $before inst]) & 0xffffffffffffffff}]
    if {$cycles == 0 || $cycles >= 0x100000000} {
        error "Window must contain 1..2^32-1 measured cycles (under 42.9s at 100MHz)"
    }
    puts [format "cycles=%d instructions=%d IPC=%.6f" $cycles $inst [expr {double($inst)/$cycles}]]
    set names {empty retire_one_uop retire_two_uops stall_recovery stall_halt stall_exception
        head_load head_store head_branch head_complex head_other ready_serial retire_no_macro
        retire_one_macro retire_two_macros slot1_absent slot1_incomplete slot1_alone slot1_other
        mispred_return mispred_other head_mispred_wait rob_full
        dispatch_no_input dispatch_blocked_rob dispatch_blocked_iq dispatch_accept_one
        dispatch_accept_two dispatch_blocked_both}
    foreach name $names a [dict get $before detail] b [dict get $after detail] {
        set delta [expr {($b - $a) & 0xffffffff}]
        puts [format "%-24s %12d %8.3f%% of cycles" $name $delta [expr {100.0*$delta/$cycles}]]
    }
}
