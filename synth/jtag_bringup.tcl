# jtag_bringup.tcl -- first-board JTAG AXI and VIO control helper.
#
# Usage:
#   vivado -nojournal -nolog -mode batch -source synth/jtag_bringup.tcl \
#       -tclargs dashboard
#   vivado -nojournal -nolog -mode batch -source synth/jtag_bringup.tcl \
#       -tclargs snapshot-machine
#   vivado -nojournal -nolog -mode batch -source synth/jtag_bringup.tcl \
#       -tclargs vio-set 0x1
#   vivado -nojournal -nolog -mode batch -source synth/jtag_bringup.tcl \
#       -tclargs axi-read 0x40000000
#   vivado -nojournal -nolog -mode batch -source synth/jtag_bringup.tcl \
#       -tclargs axi-read-range 0x40000000 4
#   vivado -nojournal -nolog -mode batch -source synth/jtag_bringup.tcl \
#       -tclargs axi-write 0x40000000 0x4ef9002a
#   vivado -nojournal -nolog -mode batch -source synth/jtag_bringup.tcl \
#       -tclargs video-poke 0x100 0x61e 48 96 1
#   vivado -nojournal -nolog -mode batch -source synth/jtag_bringup.tcl \
#       -tclargs vram-wrap-poke 0xf0010 0x40000 4 256 1
#   vivado -nojournal -nolog -mode batch -source synth/jtag_bringup.tcl \
#       -tclargs rom-load files/420dbff3.rom 0x40000000
#
# Environment:
#   BIT_FILE      Optional bitstream to program before attaching.
#   LTX_FILE      Optional probes file; defaults to build/vivado/fpga_top.ltx.
#   PROGRAM_FPGA  Set to 1 to program BIT_FILE before running the command.
#   HW_TARGET_RE  Regexp filter for get_hw_targets.
#   HW_DEVICE_RE  Regexp filter for get_hw_devices.
#   HW_AXI_RE     Regexp filter for get_hw_axis (when multiple JTAG AXI cores exist).
#
# VIO probe_out0 contract (4 bits):
#   bit0 = bypass SD boot / hold boot_fsm reset.
#   bit1 = release CPU after host ROM load.
#   bit2 = SCC UART channel select (0 = channel A, 1 = channel B).
#          Originally a CPU-only halt — that function is now strictly
#          subsumed by bit3 debug_full_reset.
#   bit3 = full debug reset — equivalent of board cold reset for the
#          CPU-side fabric (re-arms reset_overlay_active_q + resets VIA1
#          AND holds the CPU).  Hold high for at least one core_clk;
#          release to make the CPU resume from the reset PC (vec-0 fetch
#          / 0x4000_002A) with the low-mem ROM alias live again.
#          See task #256 / docs/hw_debug.md.

set script_dir [file normalize [file dirname [info script]]]
set proj_root  [file normalize $script_dir/..]

proc usage {} {
    puts stderr {usage: jtag_bringup.tcl {dashboard [--machine]|dashboard-samples [count] [delay_ms]|snapshot-machine|vio-get|vio-set <value>|axi-read <addr>|axi-read-range <addr> [words]|axi-write <addr> <data>|debug-halt-status|debug-reset-halt|debug-run [enable_bits]|debug-run-from-reset-halt-after <inst_count> [wait_ms]|debug-sweep-reset-halt-after <wait_ms> <inst_count>...|debug-halt-after <inst_count> [halt_ctl_bits]|debug-break-pc <pc> [halt_ctl_bits]|debug-halt-exc [vec] [halt_ctl_bits]|debug-clear-halt [enable_bits]|debug-step [--halt-first]|debug-arch-write <reg> <value>|debug-arch-apply|video-poke [fb_base] [stride] [rows] [cols_words] [hold_cpu]|vram-wrap-poke [fb_base] [stride] [rows] [cols_words] [hold_cpu]|rom-load <file> [base]|debug-full-reset [hold_ms] [release]}}
    exit 2
}

if {[llength $::argv] < 1} {
    usage
}
set cmd [lindex $::argv 0]

proc parse_u32 {s label} {
    if {[catch {expr {$s + 0}} v]} {
        puts stderr "ERROR: invalid $label '$s'"
        exit 2
    }
    if {$v < 0 || $v > 0xffffffff} {
        puts stderr "ERROR: $label out of 32-bit range: '$s'"
        exit 2
    }
    return [expr {$v & 0xffffffff}]
}

proc parse_u64 {s label} {
    if {[catch {expr {$s + 0}} v]} {
        puts stderr "ERROR: invalid $label '$s'"
        exit 2
    }
    if {$v < 0 || $v > 0xffffffffffffffff} {
        puts stderr "ERROR: $label out of 64-bit range: '$s'"
        exit 2
    }
    return $v
}

proc hex8 {v} {
    return [format "%08X" [expr {$v & 0xffffffff}]]
}

proc first_matching {items env_name label} {
    if {[info exists ::env($env_name)] && $::env($env_name) ne ""} {
        set re $::env($env_name)
        foreach item $items {
            if {[regexp -- $re $item]} {
                return $item
            }
        }
        puts stderr "ERROR: no $label matched $env_name='$re'"
        foreach item $items { puts stderr "  $item" }
        exit 2
    }
    return [lindex $items 0]
}

proc attach_hw {} {
    puts "=== Opening Vivado hw_manager ==="
    open_hw_manager
    connect_hw_server -allow_non_jtag

    set targets [get_hw_targets]
    if {[llength $targets] == 0} {
        puts stderr "ERROR: no JTAG targets discovered."
        exit 2
    }
    set target [first_matching $targets HW_TARGET_RE "hardware target"]
    current_hw_target $target
    open_hw_target

    set devices [get_hw_devices]
    if {[llength $devices] == 0} {
        puts stderr "ERROR: target opened but no hardware devices were discovered."
        exit 2
    }
    set dev [first_matching $devices HW_DEVICE_RE "hardware device"]
    current_hw_device $dev

    set default_bit [file join $::proj_root build vivado fpga_top.bit]
    set default_ltx [file join $::proj_root build vivado fpga_top.ltx]
    set bit_file $default_bit
    if {[info exists ::env(BIT_FILE)]} {
        set bit_file [file normalize $::env(BIT_FILE)]
    }
    set ltx_file $default_ltx
    if {[info exists ::env(LTX_FILE)]} {
        set ltx_file [file normalize $::env(LTX_FILE)]
    }
    set do_program [expr {[info exists ::env(PROGRAM_FPGA)] && $::env(PROGRAM_FPGA) == "1"}]

    if {$do_program} {
        if {![file exists $bit_file]} {
            puts stderr "ERROR: BIT_FILE not found: $bit_file"
            exit 1
        }
        set_property PROGRAM.FILE $bit_file $dev
    }
    if {[file exists $ltx_file]} {
        set_property PROBES.FILE $ltx_file $dev
        set_property FULL_PROBES.FILE $ltx_file $dev
    } else {
        puts "WARN: LTX file not found; VIO and hw_axi discovery may fail: $ltx_file"
    }

    if {$do_program} {
        puts "=== Programming $bit_file ==="
        program_hw_devices $dev
    }
    refresh_hw_device $dev
    return $dev
}

proc get_first_vio {} {
    set vios [get_hw_vios -quiet]
    if {[llength $vios] == 0} {
        puts stderr "ERROR: no VIO cores found. Build with ENABLE_VIO=1 and attach the .ltx file."
        exit 3
    }
    set vio [lindex $vios 0]
    refresh_hw_vio $vio
    return $vio
}

proc has_vio {} {
    set vios [get_hw_vios -quiet]
    return [expr {[llength $vios] != 0}]
}

proc get_first_axi {} {
    set axis [get_hw_axis -quiet]
    if {[llength $axis] == 0} {
        puts stderr "ERROR: no JTAG-to-AXI cores found. Build with ENABLE_JTAG_AXI=1 and attach the .ltx file."
        exit 3
    }
    return [first_matching $axis HW_AXI_RE "JTAG AXI core"]
}

proc first_probe {vio candidates} {
    foreach name $candidates {
        set p [get_hw_probes -quiet -of_objects $vio $name]
        if {[llength $p] != 0} {
            return [lindex $p 0]
        }
    }
    return ""
}

proc get_boot_ctrl_probe {vio} {
    return [first_probe $vio {probe_out0 vio_boot_ctrl vio_boot_ctrl_1}]
}

proc print_probe_snapshot {vio key candidates property machine} {
    set p [first_probe $vio $candidates]
    if {$p eq ""} {
        if {$machine} {
            puts [format "%s=<missing>" $key]
        } else {
            puts [format "%-20s <missing>" $key]
        }
        return
    }
    if {[catch {set v [get_property $property $p]} err]} {
        if {$machine} {
            puts [format "%s=<missing>" $key]
        } else {
            puts [format "%-20s <unreadable: %s>" $key $err]
        }
        return
    }
    if {$machine} {
        puts [format "%s=0x%s" $key $v]
    } else {
        puts [format "%-20s 0x%s" $key $v]
    }
}

proc print_probe_bus_bits {vio base width machine} {
    if {!$machine} {
        return
    }
    for {set i 0} {$i < $width} {incr i} {
        set bit_name [format {%s[%d]} $base $i]
        set p [first_probe $vio [list $bit_name]]
        if {$p ne ""} {
            if {[catch {set v [get_property INPUT_VALUE $p]} err]} {
                puts [format "%s=<missing>" $bit_name]
            } else {
                puts [format "%s=0x%s" $bit_name $v]
            }
        }
    }
}

proc probe_value {vio candidates property} {
    set p [first_probe $vio $candidates]
    if {$p eq ""} {
        return "<missing>"
    }
    if {[catch {set v [get_property $property $p]}]} {
        return "<unreadable>"
    }
    return "0x$v"
}

proc dashboard_samples {count delay_ms} {
    set vio [get_first_vio]
    puts "JTAG_SAMPLE_BEGIN"
    for {set i 0} {$i < $count} {incr i} {
        refresh_hw_vio $vio
        set po [get_boot_ctrl_probe $vio]
        set po_v "<missing>"
        if {$po ne ""} {
            set po_v "0x[get_property OUTPUT_VALUE $po]"
        }
        puts [format "sample=%d h=%s v=%s rgb=%s dehs=%s vram_read=%s vram_addr=%s dafb=%s fbreq=%s fbrsp=%s fbmiss=%s rst=%s axierr=%s out=%s" \
            $i \
            [probe_value $vio {video_debug_hcount probe_in1} INPUT_VALUE] \
            [probe_value $vio {video_debug_vcount probe_in2} INPUT_VALUE] \
            [probe_value $vio {video_debug_rgb probe_in4} INPUT_VALUE] \
            [probe_value $vio {vio_hdmi_ctrl_1 vio_hdmi_ctrl probe_in10} INPUT_VALUE] \
            [probe_value $vio {vio_vram_read probe_in11} INPUT_VALUE] \
            [probe_value $vio {vram_rd_addr probe_in3} INPUT_VALUE] \
            [probe_value $vio {vio_dafb_cfg dafb_cfg probe_in14 probe_in16} INPUT_VALUE] \
            [probe_value $vio {fb_reader_req_count} INPUT_VALUE] \
            [probe_value $vio {fb_reader_rsp_count} INPUT_VALUE] \
            [probe_value $vio {fb_reader_miss_count} INPUT_VALUE] \
            [probe_value $vio {vio_rst_bundle_1 vio_rst_bundle probe_in7} INPUT_VALUE] \
            [probe_value $vio {vio_axi_error axi_error_status probe_in15 probe_in18} INPUT_VALUE] \
            $po_v]
        if {$i + 1 < $count} {
            after $delay_ms
        }
    }
    puts "JTAG_SAMPLE_END"
}

proc vio_get {} {
    set vio [get_first_vio]
    set po [get_boot_ctrl_probe $vio]
    if {$po eq ""} {
        puts stderr "ERROR: VIO boot control output probe not found (tried probe_out0/vio_boot_ctrl)."
        exit 3
    }
    set ov_hex [get_property OUTPUT_VALUE $po]
    scan $ov_hex %x ov
    set scc_chan [expr {(($ov >> 2) & 1) ? "B" : "A"}]
    puts [format "probe_out0=0x%s bypass_sd=%d release_cpu=%d scc_uart=%s full_dbg_rst=%d" \
        $ov_hex [expr {$ov & 1}] [expr {($ov >> 1) & 1}] \
        $scc_chan [expr {($ov >> 3) & 1}]]
}

proc vio_set {value} {
    set vio [get_first_vio]
    set po [get_boot_ctrl_probe $vio]
    if {$po eq ""} {
        puts stderr "ERROR: VIO boot control output probe not found (tried probe_out0/vio_boot_ctrl)."
        exit 3
    }
    set v [expr {$value & 0xF}]
    set_property OUTPUT_VALUE [format "%X" $v] $po
    commit_hw_vio $vio
    refresh_hw_vio $vio
    puts [format "probe_out0 set to 0x%X" $v]
    vio_get
}

proc snapshot {machine} {
    set vio [get_first_vio]
    if {$machine} {
        puts "JTAG_SNAPSHOT_BEGIN"
    } else {
        puts "=== VIO snapshot ==="
    }

    set probe_specs {
        {probe_in0  INPUT_VALUE {probe_in0  hdmi_mmcm_locked hdmi_mmcm_locked_1}}
        {probe_in1  INPUT_VALUE {probe_in1  video_debug_hcount}}
        {probe_in2  INPUT_VALUE {probe_in2  video_debug_vcount}}
        {probe_in3  INPUT_VALUE {probe_in3  vram_rd_addr}}
        {probe_in4  INPUT_VALUE {probe_in4  video_debug_rgb}}
        {probe_in5  INPUT_VALUE {probe_in5  dbg_pc}}
        {probe_in6  INPUT_VALUE {probe_in6  ddr_dbg_r_cnt}}
        {probe_in7  INPUT_VALUE {probe_in7  vio_rst_bundle_1 vio_rst_bundle}}
        {probe_in8  INPUT_VALUE {probe_in8  s0_wready s0_wready_1}}
        {probe_in9  INPUT_VALUE {probe_in9  dbg_committed}}
        {probe_in10 INPUT_VALUE {probe_in10 vio_hdmi_ctrl_1 vio_hdmi_ctrl}}
        {probe_in11 INPUT_VALUE {probe_in11 vio_vram_read}}
        {probe_in12 INPUT_VALUE {probe_in12 vio_ddr_axi}}
        {probe_in13 INPUT_VALUE {probe_in13 vio_boot_video}}
        {probe_in14 INPUT_VALUE {probe_in14 vio_dafb_cfg}}
        {probe_in15 INPUT_VALUE {probe_in15 vio_axi_error}}
    }
    foreach spec $probe_specs {
        lassign $spec key property candidates
        print_probe_snapshot $vio $key $candidates $property $machine
    }

    set named_specs {
        {hdmi_mmcm_locked INPUT_VALUE {hdmi_mmcm_locked hdmi_mmcm_locked_1}}
        {video_debug_hcount INPUT_VALUE {video_debug_hcount}}
        {video_debug_vcount INPUT_VALUE {video_debug_vcount}}
        {video_debug_rgb INPUT_VALUE {video_debug_rgb}}
        {dbg_pc INPUT_VALUE {dbg_pc}}
        {ddr_dbg_r_cnt INPUT_VALUE {ddr_dbg_r_cnt}}
        {vio_rst_bundle INPUT_VALUE {vio_rst_bundle_1 vio_rst_bundle}}
        {s0_awvalid INPUT_VALUE {s0_awvalid}}
        {s0_awready INPUT_VALUE {s0_awready}}
        {s0_wvalid INPUT_VALUE {s0_wvalid}}
        {s0_wready INPUT_VALUE {s0_wready s0_wready_1}}
        {s0_bvalid INPUT_VALUE {s0_bvalid}}
        {s0_bready INPUT_VALUE {s0_bready}}
        {s0_arvalid INPUT_VALUE {s0_arvalid}}
        {s0_arready INPUT_VALUE {s0_arready}}
        {s0_rvalid INPUT_VALUE {s0_rvalid}}
        {s0_rready INPUT_VALUE {s0_rready}}
        {vio_write_counts INPUT_VALUE {vio_write_counts}}
        {vio_vram_write INPUT_VALUE {vio_vram_write}}
        {boot_rom_loading INPUT_VALUE {boot_rom_loading}}
        {boot_error INPUT_VALUE {boot_error}}
        {vio_dafb_cfg INPUT_VALUE {vio_dafb_cfg dafb_cfg}}
        {vio_fb_reader_stats INPUT_VALUE {vio_fb_reader_stats fb_reader_stats}}
        {fb_reader_req_count INPUT_VALUE {fb_reader_req_count}}
        {fb_reader_rsp_count INPUT_VALUE {fb_reader_rsp_count}}
        {fb_reader_miss_count INPUT_VALUE {fb_reader_miss_count}}
        {boot_fsm_rst INPUT_VALUE {boot_fsm_rst}}
        {cpu_rst INPUT_VALUE {cpu_rst}}
        {vio_axi_error INPUT_VALUE {vio_axi_error axi_error_status}}
        {al9134_int INPUT_VALUE {al9134_int_IBUF al9134_int}}
    }
    foreach spec $named_specs {
        lassign $spec key property candidates
        set p [first_probe $vio $candidates]
        if {$p ne ""} {
            print_probe_snapshot $vio $key $candidates $property $machine
        }
    }
    print_probe_bus_bits $vio fb_reader_req_count 16 $machine
    print_probe_bus_bits $vio fb_reader_rsp_count 16 $machine
    print_probe_bus_bits $vio fb_reader_miss_count 16 $machine
    set po [get_boot_ctrl_probe $vio]
    if {$po eq ""} {
        if {$machine} {
            puts "probe_out0=<missing>"
        } else {
            puts "probe_out0 <missing>"
        }
    } else {
        set ov_hex [get_property OUTPUT_VALUE $po]
        if {$machine} {
            puts [format "probe_out0=0x%s" $ov_hex]
        } else {
            scan $ov_hex %x ov
            set scc_chan [expr {(($ov >> 2) & 1) ? "B" : "A"}]
            puts [format "probe_out0=0x%s bypass_sd=%d release_cpu=%d scc_uart=%s full_dbg_rst=%d" \
                $ov_hex [expr {$ov & 1}] [expr {($ov >> 1) & 1}] \
                $scc_chan [expr {($ov >> 3) & 1}]]
        }
    }
    if {$machine} {
        puts "JTAG_SNAPSHOT_END"
    }
}

proc axi_read32 {addr} {
    set axi [get_first_axi]
    axi_read32_on $axi $addr
}

proc axi_read32_on {axi addr} {
    set name [format "m68k_rd_%08X" $addr]
    catch {delete_hw_axi_txn [get_hw_axi_txns -quiet $name]}
    create_hw_axi_txn $name $axi -type READ -address [hex8 $addr] -len 1
    run_hw_axi [get_hw_axi_txns $name]
    report_hw_axi_txn [get_hw_axi_txns $name]
    delete_hw_axi_txn [get_hw_axi_txns -quiet $name]
}

proc axi_read32_value_on {axi addr} {
    set name [format "m68k_rd_%08X" $addr]
    catch {delete_hw_axi_txn [get_hw_axi_txns -quiet $name]}
    create_hw_axi_txn $name $axi -type READ -address [hex8 $addr] -len 1
    run_hw_axi [get_hw_axi_txns $name]
    set data [get_property DATA [get_hw_axi_txns $name]]
    delete_hw_axi_txn [get_hw_axi_txns -quiet $name]
    scan $data %x value
    return [expr {$value & 0xffffffff}]
}

proc axi_read_range {addr words} {
    if {$words == 0 || $words > 1024} {
        puts stderr "ERROR: AXI read range word count must be 1..1024"
        exit 2
    }
    set axi [get_first_axi]
    for {set i 0} {$i < $words} {incr i} {
        axi_read32_on $axi [expr {($addr + ($i * 4)) & 0xffffffff}]
    }
}

proc axi_write32 {addr data} {
    set axi [get_first_axi]
    set name [format "m68k_wr_%08X" $addr]
    catch {delete_hw_axi_txn [get_hw_axi_txns -quiet $name]}
    create_hw_axi_txn $name $axi -type WRITE -address [hex8 $addr] -data [hex8 $data]
    run_hw_axi [get_hw_axi_txns $name]
    report_hw_axi_txn [get_hw_axi_txns $name]
}

proc axi_write32_on {axi addr data report} {
    set name [format "m68k_wr_%08X" $addr]
    catch {delete_hw_axi_txn [get_hw_axi_txns -quiet $name]}
    if {[catch {
        create_hw_axi_txn $name $axi -type WRITE -address [hex8 $addr] -data [hex8 $data]
        run_hw_axi [get_hw_axi_txns $name]
        if {$report} {
            report_hw_axi_txn [get_hw_axi_txns $name]
        }
        delete_hw_axi_txn [get_hw_axi_txns -quiet $name]
    } err]} {
        puts stderr [format "ERROR: AXI write failed at 0x%08X data=0x%08X: %s" $addr $data $err]
        exit 1
    }
}

set DBG_AXI_BASE          0x50900000
set DBG_OFF_CONTROL      0x00008
set DBG_OFF_STATUS       0x0000C
set DBG_OFF_PC           0x00010
set DBG_OFF_LAST_PC      0x00014
set DBG_OFF_EXC_VEC      0x00024
set DBG_OFF_EXC_PC       0x00028
set DBG_OFF_RESET_CAUSE  0x0002C
set DBG_OFF_HALT_AFTER_LO 0x00030
set DBG_OFF_HALT_AFTER_HI 0x00034
set DBG_OFF_BREAK_PC     0x00038
set DBG_OFF_HALT_CTL     0x0003C
set DBG_OFF_HALT_REASON  0x00040
set DBG_OFF_HALT_HIT_PC  0x00044
set DBG_OFF_HALT_HIT_INST_LO 0x00048
set DBG_OFF_HALT_HIT_INST_HI 0x0004C
set DBG_OFF_HALT_EXC_VEC 0x00050
set DBG_OFF_HALT_EXC_MASK0 0x00060
set DBG_OFF_BREAK_PC_CTRL 0x00090
set DBG_OFF_ARCH_APPLY   0x02078
set DBG_OFF_ARCH_STATUS  0x0207C
set DBG_HALT_AFTER_ENABLE 0x1
set DBG_HALT_BREAK_PC_ENABLE 0x2
set DBG_HALT_CLEAR_LATCH  0x4
set DBG_HALT_EXC_ENABLE   0x40
set DBG_CTL_HALT_REQ          0x01
set DBG_CTL_STEP_PULSE        0x02
set DBG_CTL_SOFT_RST          0x04   ;# DEPRECATED — alias of CTL_COLD_RESET_PULSE
set DBG_CTL_INIT_DONE_OVR     0x08
set DBG_CTL_COLD_RESET_HOLD   0x10   ;# bit 4 — sticky CPU hold across unified reset
set DBG_CTL_COLD_RESET_PULSE  0x20   ;# bit 5 — canonical unified-reset trigger

proc dbg_addr {off} {
    return [expr {($::DBG_AXI_BASE + $off) & 0xffffffff}]
}

proc debug_read_reg {label off} {
    puts [format "%-18s @ 0x%08X" $label [dbg_addr $off]]
    axi_read32 [dbg_addr $off]
}

proc debug_status_word {} {
    set axi [get_first_axi]
    return [axi_read32_value_on $axi [dbg_addr $::DBG_OFF_STATUS]]
}

proc debug_halt_status {} {
    puts "=== debug_ctrl programmable halt status ==="
    debug_read_reg "DBG_CONTROL"     $::DBG_OFF_CONTROL
    debug_read_reg "DBG_STATUS"      $::DBG_OFF_STATUS
    debug_read_reg "DBG_PC"          $::DBG_OFF_PC
    debug_read_reg "DBG_LAST_PC"     $::DBG_OFF_LAST_PC
    debug_read_reg "DBG_EXC_VEC"     $::DBG_OFF_EXC_VEC
    debug_read_reg "DBG_EXC_PC"      $::DBG_OFF_EXC_PC
    debug_read_reg "DBG_RESET_CAUSE" $::DBG_OFF_RESET_CAUSE
    debug_read_reg "DBG_HALT_CTL"    $::DBG_OFF_HALT_CTL
    debug_read_reg "DBG_HALT_REASON" $::DBG_OFF_HALT_REASON
    debug_read_reg "DBG_HALT_HIT_PC" $::DBG_OFF_HALT_HIT_PC
    debug_read_reg "DBG_HALT_HIT_LO" $::DBG_OFF_HALT_HIT_INST_LO
    debug_read_reg "DBG_HALT_HIT_HI" $::DBG_OFF_HALT_HIT_INST_HI
    debug_read_reg "DBG_HALT_EXC"    $::DBG_OFF_HALT_EXC_VEC
}

proc debug_reset_halt {} {
    # Canonical unified reset (docs/reset_story.md): assert
    # cold_reset_hold (bit 4) so CPU stays held across the pulse, AND
    # cold_reset_pulse (bit 5) to fire the unified-reset stretcher.
    # No race window between pulse deassert and ctrl_halt_req re-arm.
    set axi [get_first_axi]
    axi_write32_on $axi [dbg_addr $::DBG_OFF_CONTROL] \
        [expr {$::DBG_CTL_COLD_RESET_HOLD | $::DBG_CTL_COLD_RESET_PULSE}] 1
    puts "debug reset-halt issued: unified cold-reset pulse + cold_reset_hold asserted"
    debug_halt_status
}

proc debug_wait_halted {timeout_ms} {
    set deadline [expr {[clock milliseconds] + $timeout_ms}]
    while {[clock milliseconds] < $deadline} {
        if {[expr {[debug_status_word] & 0x1}]} {
            return 1
        }
        after 1
    }
    return 0
}

proc debug_step {halt_first} {
    set status [debug_status_word]
    set halted [expr {$status & 0x1}]
    set running [expr {($status >> 3) & 0x1}]

    if {!$halted} {
        if {!$halt_first} {
            puts stderr "ERROR: CPU is running; use --halt-first to halt before stepping"
            exit 2
        }
        if {$running} {
            puts "=== CPU is running; halting before single-step ==="
        } else {
            puts "=== CPU is not halted; requesting halt before single-step ==="
        }
        set axi [get_first_axi]
        axi_write32_on $axi [dbg_addr $::DBG_OFF_CONTROL] $::DBG_CTL_HALT_REQ 1
        if {![debug_wait_halted 1000]} {
            puts stderr "ERROR: CPU did not halt within 1000 ms; cannot single-step safely"
            exit 1
        }
    }

    set axi [get_first_axi]
    set ctl_bits [expr {$::DBG_CTL_STEP_PULSE | $::DBG_CTL_HALT_REQ}]
    axi_write32_on $axi [dbg_addr $::DBG_OFF_CONTROL] $ctl_bits 1
    puts [format "debug single-step issued: control=0x%X" $ctl_bits]
    debug_halt_status
}

proc debug_run {enable_bits} {
    set axi [get_first_axi]
    set masked_enable [expr {$enable_bits & ($::DBG_HALT_AFTER_ENABLE | \
                                             $::DBG_HALT_BREAK_PC_ENABLE | \
                                             $::DBG_HALT_EXC_ENABLE)}]
    # Always clear latched auto-halt causes when resuming to avoid stale
    # halts from previous stop points, then restore requested enable bits.
    set ctl_bits [expr {$masked_enable | $::DBG_HALT_CLEAR_LATCH}]
    axi_write32_on $axi [dbg_addr $::DBG_OFF_HALT_CTL] $ctl_bits 1
    axi_write32_on $axi [dbg_addr $::DBG_OFF_CONTROL] 0x0 1
    puts [format "debug run issued: halt_ctl=0x%X control=0x0" $ctl_bits]
    debug_halt_status
}

proc debug_halt_after {inst_count ctl_bits} {
    set axi [get_first_axi]
    set lo [expr {$inst_count & 0xffffffff}]
    set hi [expr {($inst_count >> 32) & 0xffffffff}]
    axi_write32_on $axi [dbg_addr $::DBG_OFF_HALT_AFTER_LO] $lo 1
    axi_write32_on $axi [dbg_addr $::DBG_OFF_HALT_AFTER_HI] $hi 1
    axi_write32_on $axi [dbg_addr $::DBG_OFF_HALT_CTL] $ctl_bits 1
    puts [format "debug halt-after programmed: inst_count=%s halt_ctl=0x%X" \
        $inst_count $ctl_bits]
    debug_halt_status
}

proc debug_run_from_reset_halt_after {inst_count wait_ms} {
    # Canonical unified-reset path (docs/reset_story.md §4.4 — Phase 2):
    # 1. Set cold_reset_hold (bit 4) + cold_reset_pulse (bit 5).  The hold
    #    bit survives the pulse it triggers; the pulse fans into
    #    soc_full_rst via fpga_top_clocks.vh dbg_rst_src_level.  All
    #    state-bearing modules re-init: CPU pipeline, peripherals,
    #    boot_fsm (re-runs SD→DDR copy), VIA1 overlay, AXI sticky errors.
    # 2. After ~50ms the boot_fsm copy completes; CPU stays held by
    #    cold_reset_hold.  Stage halt-after counters and HALT_CTL.
    # 3. Clear cold_reset_hold — CPU starts fetching with halt-after armed.
    # No VIO/btn/dbg_soft_rst race window (legacy approach asserted halt
    # FIRST then pulsed reset, hoping ctrl_halt_req would survive — it
    # didn't, because CONTROL is reset by core_rst).
    set axi [get_first_axi]
    set lo [expr {$inst_count & 0xffffffff}]
    set hi [expr {($inst_count >> 32) & 0xffffffff}]
    set ctl_bits [expr {$::DBG_HALT_AFTER_ENABLE | $::DBG_HALT_CLEAR_LATCH}]

    puts [format "=== unified-reset, run, halt-after: inst_count=%s wait_ms=%d ===" \
        $inst_count $wait_ms]

    # 1. Hold + pulse via DBG_CONTROL.
    axi_write32_on $axi [dbg_addr $::DBG_OFF_CONTROL] \
        [expr {$::DBG_CTL_COLD_RESET_HOLD | $::DBG_CTL_COLD_RESET_PULSE}] 1
    after 50

    # 2. Stage halt-after.
    axi_write32_on $axi [dbg_addr $::DBG_OFF_HALT_AFTER_LO] $lo 1
    axi_write32_on $axi [dbg_addr $::DBG_OFF_HALT_AFTER_HI] $hi 1
    axi_write32_on $axi [dbg_addr $::DBG_OFF_HALT_CTL] $ctl_bits 1

    # 3. Release hold — CPU resumes from cold-boot vector with halt-after armed.
    axi_write32_on $axi [dbg_addr $::DBG_OFF_CONTROL] 0x0 1
    after $wait_ms

    debug_halt_status
    if {[has_vio]} {
        snapshot 1
    } else {
        puts "JTAG_SNAPSHOT_BEGIN"
        puts "probe_in*=<missing:no-vio>"
        puts "JTAG_SNAPSHOT_END"
    }
}

proc debug_sweep_reset_halt_after {counts wait_ms} {
    foreach inst_count $counts {
        puts [format "JTAG_SWEEP_BEGIN inst_count=%s wait_ms=%d" \
            $inst_count $wait_ms]
        debug_run_from_reset_halt_after $inst_count $wait_ms
        puts [format "JTAG_SWEEP_END inst_count=%s" $inst_count]
    }
}

proc debug_break_pc {pc ctl_bits} {
    set axi [get_first_axi]
    axi_write32_on $axi [dbg_addr $::DBG_OFF_BREAK_PC] $pc 1
    axi_write32_on $axi [dbg_addr $::DBG_OFF_BREAK_PC_CTRL] 0x1 1
    axi_write32_on $axi [dbg_addr $::DBG_OFF_HALT_CTL] $ctl_bits 1
    puts [format "debug PC breakpoint programmed: pc=0x%08X halt_ctl=0x%X" \
        $pc $ctl_bits]
    debug_halt_status
}

proc debug_halt_exc {vec ctl_bits} {
    set axi [get_first_axi]
    set lane [expr {($vec >> 5) & 7}]
    set mask [expr {1 << ($vec & 31)}]
    for {set i 0} {$i < 8} {incr i} {
        set lane_value [expr {$i == $lane ? $mask : 0}]
        axi_write32_on $axi [dbg_addr [expr {$::DBG_OFF_HALT_EXC_MASK0 + $i * 4}]] \
            $lane_value 1
    }
    axi_write32_on $axi [dbg_addr $::DBG_OFF_HALT_CTL] $ctl_bits 1
    puts [format "debug exception halt programmed: vec=%d halt_ctl=0x%X" \
        $vec $ctl_bits]
    debug_halt_status
}

proc debug_clear_halt {enable_bits} {
    set axi [get_first_axi]
    set ctl_bits [expr {($enable_bits | $::DBG_HALT_CLEAR_LATCH) & 0xffffffff}]
    axi_write32_on $axi [dbg_addr $::DBG_OFF_HALT_CTL] $ctl_bits 1
    puts [format "debug auto-halt latch cleared: halt_ctl=0x%X" $ctl_bits]
    debug_halt_status
}

proc debug_arch_offset {name} {
    set key [string toupper $name]
    if {[regexp {^D([0-7])$} $key -> idx]} {
        return [expr {0x02000 + ($idx * 4)}]
    }
    if {[regexp {^A([0-7])$} $key -> idx]} {
        return [expr {0x02020 + ($idx * 4)}]
    }
    array set offs {
        USP 0x02040 SSP 0x02044 ISP 0x02048 SR 0x0204C
        VBR 0x02050 CACR 0x02054 TC 0x02058 ITT0 0x0205C
        ITT1 0x02060 DTT0 0x02064 DTT1 0x02068 URP 0x0206C
        SRP 0x02070 PC 0x02074 SFC 0x02080 DFC 0x02084
    }
    if {![info exists offs($key)]} {
        puts stderr "ERROR: unknown arch register '$name'"
        exit 2
    }
    return $offs($key)
}

proc debug_arch_write {name value} {
    set axi [get_first_axi]
    set off [debug_arch_offset $name]
    axi_write32_on $axi [dbg_addr $off] $value 1
    puts [format "debug arch shadow %s <= 0x%08X" [string toupper $name] $value]
}

proc debug_arch_apply {} {
    set axi [get_first_axi]
    axi_write32_on $axi [dbg_addr $::DBG_OFF_ARCH_APPLY] 0x3 1
    puts "debug arch apply/resume issued"
    axi_read32 [dbg_addr $::DBG_OFF_ARCH_STATUS]
    debug_halt_status
}

proc byte_repeat32 {byte_value} {
    set b [expr {$byte_value & 0xff}]
    return [expr {($b << 24) | ($b << 16) | ($b << 8) | $b}]
}

proc video_poke {fb_base stride rows cols_words hold_cpu} {
    set vram_axi_base 0xF9000000
    set dafb_axi_base 0xF9800000

    if {$hold_cpu} {
        puts "=== Holding CPU via VIO probe_out0=0x4 while patching DAFB/VRAM ==="
        vio_set 0x4
    }

    set axi [get_first_axi]
    puts [format "=== DAFB setup: fb_base=0x%08X stride=0x%08X bpp=0x30 ===" $fb_base $stride]
    axi_write32_on $axi [expr {$dafb_axi_base + 0x08}] $fb_base 1
    axi_write32_on $axi [expr {$dafb_axi_base + 0x0c}] $stride 1
    axi_write32_on $axi [expr {$dafb_axi_base + 0x10}] 0x30 1

    puts "=== DAFB CLUT setup: entries 0..15 map to built-in 4-bit colours ==="
    for {set i 0} {$i < 16} {incr i} {
        axi_write32_on $axi [expr {$dafb_axi_base + 0x300 + ($i * 0x10)}] $i 0
    }

    puts [format "=== VRAM poke: rows=%d cols_words=%d at AXI 0x%08X + fb_base ===" \
        $rows $cols_words $vram_axi_base]
    set total [expr {$rows * $cols_words}]
    set count 0
    for {set y 0} {$y < $rows} {incr y} {
        for {set xw 0} {$xw < $cols_words} {incr xw} {
            set colour [expr {9 + ((($y / 8) + ($xw / 16)) % 7)}]
            set word [byte_repeat32 $colour]
            set addr [expr {$vram_axi_base + $fb_base + ($y * $stride) + ($xw * 4)}]
            axi_write32_on $axi $addr $word 0
            incr count
        }
        if {($y % 8) == 7 || $y == ($rows - 1)} {
            puts [format "poked %d / %d VRAM words" $count $total]
        }
    }
    puts "=== VRAM poke complete ==="
    puts [format "first pixel word should read back from 0x%08X" [expr {$vram_axi_base + $fb_base}]]
    axi_read32 [expr {$vram_axi_base + $fb_base}]
    snapshot 0
}

proc vram_wrap_poke {fb_base stride rows cols_words hold_cpu} {
    set vram_axi_base 0xF9000000
    set vram_mask     0x000FFFFF

    if {$hold_cpu} {
        puts "=== Holding CPU via VIO probe_out0=0x4 while patching active VRAM scanout addresses ==="
        vio_set 0x4
    }

    set axi [get_first_axi]
    puts [format "=== VRAM wrap poke: fb_base=0x%08X stride=0x%08X rows=%d cols_words=%d ===" \
        $fb_base $stride $rows $cols_words]
    puts "=== No DAFB writes: this paints the currently programmed scanout placement only ==="

    set total [expr {$rows * $cols_words}]
    set count 0
    for {set y 0} {$y < $rows} {incr y} {
        for {set xw 0} {$xw < $cols_words} {incr xw} {
            set colour [expr {9 + ((($y / 1) + ($xw / 16)) % 7)}]
            set word [byte_repeat32 $colour]
            set off [expr {($fb_base + ($y * $stride) + ($xw * 4)) & $vram_mask}]
            set addr [expr {$vram_axi_base + $off}]
            axi_write32_on $axi $addr $word 0
            incr count
        }
        puts [format "poked wrapped scanout row %d / %d (%d / %d VRAM words)" \
            [expr {$y + 1}] $rows $count $total]
    }
    puts "=== VRAM wrap poke complete ==="
    puts [format "first wrapped pixel word should read back from 0x%08X" \
        [expr {$vram_axi_base + ($fb_base & $vram_mask)}]]
    axi_read32 [expr {$vram_axi_base + ($fb_base & $vram_mask)}]
    snapshot 0
}

proc rom_load {path base} {
    if {![file exists $path]} {
        puts stderr "ERROR: ROM file not found: $path"
        exit 1
    }
    set fh [open $path rb]
    fconfigure $fh -translation binary -encoding binary
    set data [read $fh]
    close $fh

    binary scan $data H* hex
    set nbytes [string length $data]
    puts [format "=== ROM load: %s -> 0x%08X (%d bytes) ===" $path $base $nbytes]
    puts "=== Holding boot_fsm reset and CPU reset via VIO probe_out0=0x5 ==="
    vio_set 0x5

    set padded [expr {(($nbytes + 3) / 4) * 4}]
    for {set off 0} {$off < $padded} {incr off 4} {
        set word_hex [string range $hex [expr {$off * 2}] [expr {$off * 2 + 7}]]
        while {[string length $word_hex] < 8} {
            append word_hex "00"
        }
        scan $word_hex %x word
        axi_write32 [expr {$base + $off}] $word
        if {($off % 4096) == 0} {
            puts [format "loaded 0x%08X / 0x%08X bytes" $off $padded]
        }
    }
    puts "=== ROM load complete; releasing CPU via VIO probe_out0=0x3 ==="
    vio_set 0x3
}

proc debug_full_reset {hold_ms release} {
    # Cold-boot equivalent for the CPU-side fabric: pulse vio_boot_ctrl[3]
    # high for $hold_ms milliseconds to re-arm reset_overlay_active_q +
    # reset VIA1 (so ORB[3] returns to 1 and the low-mem ROM alias is
    # active again).  When $release is non-zero, drop the bit so the CPU
    # resumes from the reset PC; when zero, leave the bit asserted (the
    # caller will release it via vio-set after patching ROM/DRAM).
    puts "=== Asserting VIO debug-full-reset (probe_out0[3]=1) ==="
    vio_set 0x8
    if {$hold_ms > 0} {
        after $hold_ms
    }
    if {$release} {
        puts "=== Releasing CPU (probe_out0=0x0) — CPU resumes from reset PC ==="
        vio_set 0x0
    } else {
        puts "=== Holding debug-full-reset asserted; release with `vio-set 0` ==="
    }
    vio_get
}

attach_hw

switch -- $cmd {
    dashboard {
        if {[llength $::argv] == 1} {
            snapshot 0
        } elseif {[llength $::argv] == 2 && [lindex $::argv 1] eq "--machine"} {
            snapshot 1
        } else {
            usage
        }
    }
    dashboard-samples {
        if {[llength $::argv] > 3} { usage }
        set count 16
        set delay_ms 100
        if {[llength $::argv] >= 2} {
            set count [parse_u32 [lindex $::argv 1] "sample count"]
        }
        if {[llength $::argv] == 3} {
            set delay_ms [parse_u32 [lindex $::argv 2] "sample delay"]
        }
        if {$count == 0 || $count > 1024 || $delay_ms > 10000} {
            puts stderr "ERROR: count must be 1..1024 and delay_ms must be 0..10000"
            exit 2
        }
        dashboard_samples $count $delay_ms
    }
    snapshot-machine {
        if {[llength $::argv] != 1} { usage }
        snapshot 1
    }
    vio-get {
        if {[llength $::argv] != 1} { usage }
        vio_get
    }
    vio-set {
        if {[llength $::argv] != 2} { usage }
        vio_set [parse_u32 [lindex $::argv 1] "VIO value"]
    }
    axi-read {
        if {[llength $::argv] != 2} { usage }
        axi_read32 [parse_u32 [lindex $::argv 1] "AXI address"]
    }
    axi-read-range {
        if {[llength $::argv] < 2 || [llength $::argv] > 3} { usage }
        set words 16
        if {[llength $::argv] == 3} {
            set words [parse_u32 [lindex $::argv 2] "AXI read word count"]
        }
        axi_read_range [parse_u32 [lindex $::argv 1] "AXI address"] $words
    }
    axi-write {
        if {[llength $::argv] != 3} { usage }
        axi_write32 [parse_u32 [lindex $::argv 1] "AXI address"] \
                    [parse_u32 [lindex $::argv 2] "AXI data"]
    }
    debug-halt-status {
        if {[llength $::argv] != 1} { usage }
        debug_halt_status
    }
    debug-reset-halt {
        if {[llength $::argv] != 1} { usage }
        debug_reset_halt
    }
    debug-run {
        if {[llength $::argv] > 2} { usage }
        set enable_bits 0
        if {[llength $::argv] == 2} {
            set enable_bits [parse_u32 [lindex $::argv 1] "HALT_CTL enable bits"]
        }
        debug_run $enable_bits
    }
    debug-run-from-reset-halt-after {
        if {[llength $::argv] < 2 || [llength $::argv] > 3} { usage }
        set inst_count [parse_u64 [lindex $::argv 1] "instruction count"]
        set wait_ms 50
        if {[llength $::argv] == 3} {
            set wait_ms [parse_u32 [lindex $::argv 2] "wait milliseconds"]
        }
        if {$wait_ms > 10000} {
            puts stderr "ERROR: wait_ms must be 0..10000"
            exit 2
        }
        debug_run_from_reset_halt_after $inst_count $wait_ms
    }
    debug-sweep-reset-halt-after {
        if {[llength $::argv] < 3} { usage }
        set wait_ms [parse_u32 [lindex $::argv 1] "wait milliseconds"]
        if {$wait_ms > 10000} {
            puts stderr "ERROR: wait_ms must be 0..10000"
            exit 2
        }
        set counts {}
        foreach raw_count [lrange $::argv 2 end] {
            lappend counts [parse_u64 $raw_count "instruction count"]
        }
        debug_sweep_reset_halt_after $counts $wait_ms
    }
    debug-halt-after {
        if {[llength $::argv] < 2 || [llength $::argv] > 3} { usage }
        set inst_count [parse_u64 [lindex $::argv 1] "instruction count"]
        set ctl_bits [expr {$::DBG_HALT_AFTER_ENABLE | $::DBG_HALT_CLEAR_LATCH}]
        if {[llength $::argv] == 3} {
            set ctl_bits [parse_u32 [lindex $::argv 2] "HALT_CTL bits"]
        }
        debug_halt_after $inst_count $ctl_bits
    }
    debug-break-pc {
        if {[llength $::argv] < 2 || [llength $::argv] > 3} { usage }
        set pc [parse_u32 [lindex $::argv 1] "breakpoint PC"]
        set ctl_bits $::DBG_HALT_CLEAR_LATCH
        if {[llength $::argv] == 3} {
            set ctl_bits [parse_u32 [lindex $::argv 2] "HALT_CTL bits"]
        }
        debug_break_pc $pc $ctl_bits
    }
    debug-halt-exc {
        if {[llength $::argv] > 3} { usage }
        set vec 4
        if {[llength $::argv] >= 2} {
            set vec [parse_u32 [lindex $::argv 1] "exception vector"]
        }
        set ctl_bits $::DBG_HALT_CLEAR_LATCH
        if {[llength $::argv] == 3} {
            set ctl_bits [parse_u32 [lindex $::argv 2] "HALT_CTL bits"]
        }
        debug_halt_exc $vec $ctl_bits
    }
    debug-clear-halt {
        if {[llength $::argv] > 2} { usage }
        set enable_bits 0
        if {[llength $::argv] == 2} {
            set enable_bits [parse_u32 [lindex $::argv 1] "HALT_CTL enable bits"]
        }
        debug_clear_halt $enable_bits
    }
    debug-step {
        if {[llength $::argv] > 2} { usage }
        set halt_first 0
        if {[llength $::argv] == 2} {
            if {[lindex $::argv 1] ne "--halt-first"} { usage }
            set halt_first 1
        }
        debug_step $halt_first
    }
    debug-arch-write {
        if {[llength $::argv] != 3} { usage }
        debug_arch_write [lindex $::argv 1] [parse_u32 [lindex $::argv 2] "arch value"]
    }
    debug-arch-apply {
        if {[llength $::argv] != 1} { usage }
        debug_arch_apply
    }
    video-poke {
        if {[llength $::argv] > 6} { usage }
        set fb_base 0x100
        set stride 0x61e
        set rows 48
        set cols_words 96
        set hold_cpu 0
        if {[llength $::argv] >= 2} {
            set fb_base [parse_u32 [lindex $::argv 1] "framebuffer base"]
        }
        if {[llength $::argv] >= 3} {
            set stride [parse_u32 [lindex $::argv 2] "framebuffer stride"]
        }
        if {[llength $::argv] >= 4} {
            set rows [parse_u32 [lindex $::argv 3] "row count"]
        }
        if {[llength $::argv] >= 5} {
            set cols_words [parse_u32 [lindex $::argv 4] "column word count"]
        }
        if {[llength $::argv] == 6} {
            set hold_cpu [parse_u32 [lindex $::argv 5] "hold_cpu"]
        }
        if {$rows == 0 || $cols_words == 0 || $rows > 768 || $cols_words > 256} {
            puts stderr "ERROR: rows must be 1..768 and cols_words must be 1..256"
            exit 2
        }
        video_poke $fb_base $stride $rows $cols_words [expr {$hold_cpu != 0}]
    }
    vram-wrap-poke {
        if {[llength $::argv] > 6} { usage }
        set fb_base 0xf0010
        set stride 0x40000
        set rows 4
        set cols_words 256
        set hold_cpu 0
        if {[llength $::argv] >= 2} {
            set fb_base [parse_u32 [lindex $::argv 1] "framebuffer base"]
        }
        if {[llength $::argv] >= 3} {
            set stride [parse_u32 [lindex $::argv 2] "framebuffer stride"]
        }
        if {[llength $::argv] >= 4} {
            set rows [parse_u32 [lindex $::argv 3] "row count"]
        }
        if {[llength $::argv] >= 5} {
            set cols_words [parse_u32 [lindex $::argv 4] "column word count"]
        }
        if {[llength $::argv] == 6} {
            set hold_cpu [parse_u32 [lindex $::argv 5] "hold_cpu"]
        }
        if {$rows == 0 || $cols_words == 0 || $rows > 768 || $cols_words > 256} {
            puts stderr "ERROR: rows must be 1..768 and cols_words must be 1..256"
            exit 2
        }
        vram_wrap_poke $fb_base $stride $rows $cols_words [expr {$hold_cpu != 0}]
    }
    rom-load {
        if {[llength $::argv] < 2 || [llength $::argv] > 3} { usage }
        set base 0x40000000
        if {[llength $::argv] == 3} {
            set base [parse_u32 [lindex $::argv 2] "ROM base"]
        }
        rom_load [lindex $::argv 1] $base
    }
    debug-full-reset {
        # debug-full-reset [hold_ms] [release]
        # Pulses VIO probe_out0[3] (jtag_debug_full_reset) — the CPU-side
        # cold-boot equivalent reset.  Re-arms reset_overlay_active_q and
        # resets VIA1 so the low-memory ROM alias is live again on
        # release.  See task #256 / docs/hw_debug.md.
        if {[llength $::argv] > 3} { usage }
        set hold_ms 100
        set release 1
        if {[llength $::argv] >= 2} {
            set hold_ms [parse_u32 [lindex $::argv 1] "hold milliseconds"]
        }
        if {[llength $::argv] == 3} {
            set release [parse_u32 [lindex $::argv 2] "release flag (0/1)"]
        }
        if {$hold_ms > 10000} {
            puts stderr "ERROR: hold_ms must be 0..10000"
            exit 2
        }
        debug_full_reset $hold_ms [expr {$release != 0}]
    }
    default {
        usage
    }
}
