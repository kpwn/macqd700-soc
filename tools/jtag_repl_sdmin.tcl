# jtag_repl_sdmin.tcl — minimal interactive REPL for the standalone
# fpga_top_sdmin SD/CRC test harness (rtl/soc/fpga_top_sdmin.v). No
# JTAG-AXI/debug_ctrl in this design at all — VIO only. Mirrors
# tools/jtag_repl.tcl's stdin-loop spirit, much smaller (no reset/halt/
# breakpoint machinery, none of which exists in this harness).
#
# Usage:
#   vivado -mode tcl -nojournal -nolog -source tools/jtag_repl_sdmin.tcl \
#     -tclargs <bit> <ltx> < fifo_in > fifo_out
#
# Commands:
#   status            — read all probe_in values + decode vio_diag/vio_crc/vio_state/vio_flags
#                       + per-attempt outcome log (vio_attempt0..5)
#   ctrl <hex>         — write probe_out0 (bit0=soft-reset, bit1=cfg_skip_cmd59, bit2=cfg_force_hs)
#   restart            — pulse bit0 (soft-reset) high then low, leaving cfg bits unchanged
#   q / quit / exit

set bit_file [lindex $argv 0]
set ltx_file [lindex $argv 1]

open_hw_manager
connect_hw_server -allow_non_jtag
set hw_targets [get_hw_targets]
open_hw_target [lindex $hw_targets 0]
set dev [lindex [get_hw_devices] 0]
set_property PROGRAM.FILE  $bit_file $dev
set_property PROBES.FILE   $ltx_file $dev
program_hw_devices $dev
refresh_hw_device -update_hw_probes true $dev
puts "READY"
flush stdout

set ::vio_ctrl_val 0

proc get_vio {} {
    set vios [get_hw_vios -quiet]
    if {[llength $vios] == 0} { error "no VIO core found" }
    return [lindex $vios 0]
}

proc read_probe {name} {
    set vio [get_vio]
    refresh_hw_vio $vio
    foreach p [get_hw_probes -of_objects $vio -quiet] {
        if {[get_property NAME $p] eq $name} {
            return [get_property INPUT_VALUE $p]
        }
    }
    error "probe $name not found"
}

proc write_ctrl {hexval} {
    set vio [get_vio]
    foreach p [get_hw_probes -of_objects $vio -quiet] {
        if {[get_property NAME $p] eq "vio_ctrl"} {
            set_property OUTPUT_VALUE $hexval $p
            commit_hw_vio [list $p]
            set ::vio_ctrl_val $hexval
            return
        }
    }
    error "probe_out vio_ctrl not found"
}

while {1} {
    if {[gets stdin line] < 0} { break }
    set line [string trim $line]
    if {$line eq ""} { continue }
    set cmd [lindex $line 0]
    if {[catch {
        switch -- $cmd {
            names {
                set vio [get_vio]
                set names {}
                foreach p [get_hw_probes -of_objects $vio -quiet] {
                    lappend names [get_property NAME $p]
                }
                puts "> $names"
            }
            status {
                set diag  [read_probe "vio_diag"]
                set crc   [read_probe "vio_crc"]
                set state [read_probe "vio_state"]
                set flags [read_probe "vio_flags"]
                set lba   [read_probe "dbg_sdctrl_lba_lat"]
                set crc32 [read_probe "rom_crc32_final"]
                set pollc [read_probe "vio_poll_cnt"]
                set att0  [read_probe "vio_attempt0"]
                set att1  [read_probe "vio_attempt1"]
                set att2  [read_probe "vio_attempt2"]
                set att3  [read_probe "vio_attempt3"]
                set att4  [read_probe "vio_attempt4"]
                set att5  [read_probe "vio_attempt5"]
                scan $diag  %x diag_v
                scan $crc   %x crc_v
                scan $state %x state_v
                scan $flags %x flags_v
                scan $lba   %x lba_v
                scan $crc32 %x crc32_v
                scan $pollc %x pollc_v
                scan $att0  %x att0_v
                scan $att1  %x att1_v
                scan $att2  %x att2_v
                scan $att3  %x att3_v
                scan $att4  %x att4_v
                scan $att5  %x att5_v
                set sector    [expr {$diag_v & 0xFFFF}]
                set err_cause [expr {($diag_v >> 16) & 0x7}]
                set retry     [expr {($diag_v >> 19) & 0xF}]
                set crc_recv  [expr {$crc_v & 0xFFFF}]
                set crc_calc  [expr {($crc_v >> 16) & 0xFFFF}]
                set last_r1   [expr {$state_v & 0xFF}]
                set cur_cmd   [expr {($state_v >> 8) & 0xF}]
                set st        [expr {($state_v >> 12) & 0x3F}]
                set ctrl_err  [expr {($state_v >> 18) & 0xF}]
                set sdctrl_cmd [expr {($state_v >> 22) & 0xF}]
                set rst_f     [expr {$flags_v & 1}]
                set sdhc_f    [expr {($flags_v >> 1) & 1}]
                set fast_f    [expr {($flags_v >> 2) & 1}]
                set hs_f      [expr {($flags_v >> 3) & 1}]
                set crcen_f   [expr {($flags_v >> 4) & 1}]
                set err_f     [expr {($flags_v >> 5) & 1}]
                set loaded_f  [expr {($flags_v >> 6) & 1}]
                set loading_f [expr {($flags_v >> 7) & 1}]
                set last_real_r1 [expr {($flags_v >> 8) & 0xFF}]
                set last_crc7    [expr {($flags_v >> 16) & 0xFF}]
                puts [format "> diag: sector=%d err_cause=%d ctrl_retry=%d" $sector $err_cause $retry]
                puts [format "> crc: calc=0x%04x recv=0x%04x match=%s" $crc_calc $crc_recv [expr {$crc_calc == $crc_recv ? "YES" : "no"}]]
                puts [format "> state: st=%d cur_cmd=%d last_r1=0x%02x ctrl_err_cause=%d (0=none,1=R1_BAD,2=R1_TO,3=TOK_TO,4=DR_BAD,5=BUSY_TO,6=UNK_CMD,7=CRC_BAD) sdctrl_cur_cmd=%d (0=none,1=CMD17,2=CMD18,3=CMD24,4=CMD25,...,SC_CMD12 check code) lba_lat=0x%08x" $st $cur_cmd $last_r1 $ctrl_err $sdctrl_cmd $lba_v]
                puts [format "> flags: rst=%d is_sdhc=%d fast_mode=%d hs_mode=%d crc_enabled=%d error=%d rom_loaded=%d rom_loading=%d last_real_r1=0x%02x last_crc7_sent=0x%02x" \
                      $rst_f $sdhc_f $fast_f $hs_f $crcen_f $err_f $loaded_f $loading_f $last_real_r1 $last_crc7]
                puts [format "> vio_ctrl (last written) = 0x%x" $::vio_ctrl_val]
                puts [format "> rom_crc32=0x%08x last_poll_cnt=%d" $crc32_v $pollc_v]
                set atts [list $att0_v $att1_v $att2_v $att3_v $att4_v $att5_v]
                puts "> attempt log (outcome=0xF success, else sd_ctrl err_cause 0-7; entries beyond ctrl_retry are stale/from a prior run):"
                for {set i 0} {$i < 6} {incr i} {
                    set v [lindex $atts $i]
                    set a_sector  [expr {$v & 0xFFFF}]
                    set a_outcome [expr {($v >> 16) & 0xF}]
                    if {$a_outcome == 0xF} {
                        set a_desc "SUCCESS"
                    } else {
                        set a_desc [format "FAIL err_cause=%d" $a_outcome]
                    }
                    puts [format ">   attempt %d: sector=%d  %s" $i $a_sector $a_desc]
                }
            }
            ctrl {
                write_ctrl [lindex $line 1]
                puts [format "> vio_ctrl set to 0x%x" [lindex $line 1]]
            }
            restart {
                set base [expr {$::vio_ctrl_val & ~1}]
                write_ctrl [expr {$base | 1}]
                after 100
                write_ctrl $base
                puts "> restart pulsed"
            }
            q -
            quit -
            exit {
                puts "> bye"
                flush stdout
                exit 0
            }
            default {
                puts "> ERROR unknown cmd: $cmd"
            }
        }
    } err]} {
        puts "> ERROR $err"
    }
    puts "> READY"
    flush stdout
}
