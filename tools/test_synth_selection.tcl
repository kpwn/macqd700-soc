#!/usr/bin/env tclsh
# Execute the actual input-record block with a tiny generated CPU fixture.
set root [file dirname [file dirname [file normalize [info script]]]]
set f [open "$root/synth/vivado.tcl" r]
set src [read $f]
close $f
set a [string first {# SYNTH_SELECTION_BEGIN} $src]
set b [string first {# SYNTH_SELECTION_END} $src]
if {$a < 0 || $b <= $a} {error "Missing synthesis selection block"}
set block [string range $src $a [expr {$b - 1}]]
set f [file tempfile scratch]
close $f
file delete $scratch
file mkdir $scratch
set saved_env [array get ::env]
try {
    set output_dir $scratch
    set cpu040_dir "$scratch/cpu source"
    file mkdir "$cpu040_dir/generated"
    set cpu_m68k040_v "$cpu040_dir/generated/M68kSocketTop.v"
    foreach {path contents} [list $cpu_m68k040_v {module M68kSocketTop; endmodule} \
                                 "$cpu040_dir/generated/init data.bin" 10101010] {
        set f [open $path w]; puts $f $contents; close $f
    }
    set proj_root $root
    set mode synth_only
    set part fixture_part
    set cpu_ipc_profile throughput-v2
    set cpu_debug_profile full
    set target_freq_mhz 200
    set core_clk_hz 200000000
    set build_id 0x12345678
    set ::env(CPU_LS_P1_EARLY_LOAD) 1
    set ::env(SPEC_LOAD_WAKEUP) 1
    set ::env(CORE_BUDGET_NS) 4.0
    set ::env(EXPERIMENT_SECRET) do_not_record
    foreach {cpu_m68k040 cpu_sel cpu_axi_dh l2_dh_r_fall_through} {0 stub 0 0 1 m68k040 1 1} {
        set top_generic_list [list L2_DH_R_FALL_THROUGH=$l2_dh_r_fall_through]
        set synth_def_args [list -verilog_define FIXTURE]
        eval $block
        set f [open "$output_dir/fpga_top.synth_selection" r]
        set record [read $f]; close $f
        if {[dict get $record mode] ne "synth_only" ||
            [dict get $record core_clk_hz] != 200000000 ||
            [dict get $record top_generics] ne $top_generic_list ||
            [dict get $record l2_dh_r_fall_through] != $l2_dh_r_fall_through} {error "Wrong effective selection"}
        if {[dict get $record physical_environment CORE_BUDGET_NS] ne "4.0"} {error "Missing timing stress selection"}
        if {[dict exists $record cpu_environment EXPERIMENT_SECRET] ||
            [dict get $record cpu_environment CPU_LS_P1_EARLY_LOAD] ne "1" ||
            [dict get $record cpu_environment SPEC_LOAD_WAKEUP] ne "1"} {error "Environment filtering failed"}
        set files [dict get $record generated_cpu_files]
        if {$cpu_m68k040} {
            if {[dict size $files] != 2 || [dict get $record cpu_revision] ne "UNKNOWN"} {error "Wrong fixture source identity"}
            foreach file [dict keys $files] {
                set path "$cpu040_dir/generated/$file"
                if {[dict get $files $file sha256] ne [lindex [exec sha256sum -- $path] 0] ||
                    [dict get $files $file bytes] != [file size $path]} {error "Wrong generated input hash"}
            }
        } elseif {[dict size $files] != 0 || [dict exists $record cpu_revision]} {error "Stub claimed CPU provenance"}
        puts "PASS synthesis selection $cpu_sel"
    }
} finally {
    array unset ::env
    array set ::env $saved_env
    file delete -force $scratch
}
