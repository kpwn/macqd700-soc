# Source in an attached Vivado hardware-manager/JTAG REPL with matching .ltx.
# Arming and uploading never halt or reset the CPU.
proc ipc_ila_core {} {
    set cores [get_hw_ilas -quiet -filter {CELL_NAME =~ *u_ipc_ila*}]
    if {[llength $cores] != 1} { error "Expected one u_ipc_ila; load the diagnostic bitstream and matching .ltx" }
    return [lindex $cores 0]
}
proc ipc_ila_probe {core index} {
    set probes [get_hw_probes -quiet -of_objects $core -filter "PORT_INDEX == $index"]
    if {[llength $probes] != 1} { error "Expected one probe on ILA port $index" }
    return [lindex $probes 0]
}
proc ipc_ila_arm {{pc now} {position 512}} {
    if {![string is integer -strict $position] || $position < 0 || $position > 1023} {
        error "Trigger position must be 0..1023"
    }
    set core [ipc_ila_core]
    set_property CONTROL.DATA_DEPTH 1024 $core
    set_property CONTROL.WINDOW_COUNT 1 $core
    set_property CONTROL.TRIGGER_MODE BASIC_ONLY $core
    set_property CONTROL.TRIGGER_POSITION $position $core
    if {$pc eq "now"} {
        # Immediate capture has no pre-trigger history; place the marker at zero.
        set_property CONTROL.TRIGGER_POSITION 0 $core
        run_hw_ila -trigger_now $core
    } else {
        if {![string is entier -strict $pc] || $pc < 0 || $pc > 0xffffffff || ($pc & 1)} {
            error "PC must be an even 32-bit address (for example 0x40800100)"
        }
        # Don't-care probes do not participate in the basic trigger equation.
        foreach n {0 1 2 3} width {33 33 23 6} {
            set_property TRIGGER_COMPARE_VALUE "eq${width}'b[string repeat X $width]" [ipc_ila_probe $core $n]
        }
        foreach n {0 1} {
            set_property TRIGGER_COMPARE_VALUE [format "eq33'h1%08X" $pc] [ipc_ila_probe $core $n]
        }
        set_property CONTROL.TRIGGER_CONDITION OR $core
        run_hw_ila $core
    }
    puts "IPC ILA armed: $pc; 1024 consecutive core cycles. Use ipc_ila_save after capture."
}
proc ipc_ila_save {path} {
    # Deliberately no blocking wait: a PC that never retires must not monopolize JTAG.
    set core [ipc_ila_core]
    set data [upload_hw_ila_data $core]
    write_hw_ila_data -csv_file $path $data
    puts "Saved $path"
}
