#!/usr/bin/env tclsh
# Exercise actual synthesis configuration, generic application and provenance
# without starting Vivado or regenerating the CPU.
set root [file dirname [file dirname [file normalize [info script]]]]
set fh [open "$root/synth/vivado.tcl" r]
set src [read $fh]
close $fh
proc between {src start finish} {
    set a [string first $start $src]
    set b [string first $finish $src $a]
    if {$a < 0 || $b <= $a} {error "Missing source block: $start"}
    return [string range $src $a [expr {$b - 1}]]
}
set config [between $src {# HOT_RETURN_CONFIG_BEGIN} {# HOT_RETURN_CONFIG_END}]
set generics [between $src {set top_generic_list [list} {puts "=== TOP GENERICS:}]
set provenance {}
foreach line [split $src \n] {
    if {[string match {puts $buildinfo_fh "l2_dh_r_fall_through=*} $line]} {lappend provenance $line}
}
if {[llength $provenance] != 1} {error "Missing or duplicated buildinfo selection"}
foreach {name value dh expected} {
    default absent 0 0
    off 0 1 0
    on 1 1 1
    missing_hot_door 1 0 {requires CPU_AXI_DH=1}
    invalid 2 1 {must be 0 or 1}
    empty {} 1 {must be 0 or 1}
} {
    set i [interp create]
    try {
        interp eval $i {unset -nocomplain ::env(L2_DH_R_FALL_THROUGH)}
        interp eval $i [list set cpu_axi_dh $dh]
        if {$value ne "absent"} {interp eval $i [list set ::env(L2_DH_R_FALL_THROUGH) $value]}
        interp eval $i {
            proc puts {args} {set ::last_puts $args}
            proc current_fileset {} {return sources_1}
            proc set_property {property value object} {
                if {$property ne "generic" || $object ne "sources_1"} {error "Wrong generic target"}
                set ::applied_generics $value
            }
            foreach v {core_clk_hz core_clk_divide pb_clk_hz video_smoke eth_icmp_responder boot_rom_sectors sd_safe_cmd25 build_id_generic} {set $v 0}
            set buildinfo_fh buildinfo
        }
        set failed [catch {interp eval $i $config} result]
        if {$expected ni {0 1}} {
            if {!$failed || ![string match "*$expected*" $result]} {error "$name: unexpected result '$result'"}
        } else {
            if {$failed} {error "$name: $result"}
            interp eval $i $generics
            set applied [interp eval $i {set applied_generics}]
            if {[llength [lsearch -all -exact $applied "L2_DH_R_FALL_THROUGH=$expected"]] != 1} {
                error "$name: wrong applied generic: $applied"
            }
            interp eval $i [lindex $provenance 0]
            if {[interp eval $i {set last_puts}] ne [list buildinfo "l2_dh_r_fall_through=$expected"]} {
                error "$name: wrong buildinfo selection"
            }
        }
        puts "PASS $name"
    } finally {interp delete $i}
}
