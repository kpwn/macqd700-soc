# vio_dashboard.tcl — open Vivado hw_manager, program the bitstream,
# attach the VIO probes, and print a text dashboard of current values.
#
# Usage (from any directory):
#   vivado -mode tcl -source synth/vio_dashboard.tcl \
#          -tclargs <bitstream.bit> <probes.ltx>
#
# Or with -nojournal -nolog for a scripted refresh:
#   vivado -nojournal -nolog -mode batch \
#          -source synth/vio_dashboard.tcl \
#          -tclargs <bitstream.bit> <probes.ltx>
#
# If invoked with no args, defaults to build/vivado/fpga_top.{bit,ltx}
# under the project root.
#
# ─── Probe map (matches the `debug_vio` IP instance in rtl/fpga_top.v) ───
#
# Generation flow: `create_ip -name vio` generates the IP OOC under
# $build/ip/debug_vio (see synth/vivado.tcl gen_debug_vio_ip).  The IP
# is instantiated directly in rtl/fpga_top.v under `ifdef VIO_ENABLE`;
# synth pass it as `-verilog_define VIO_ENABLE` when `ENABLE_VIO=1`.
#
# Compact probe map v27 (activity detection disabled; values remain readable).
#   probe0  32 bits  dbg_pc
#   probe1  6 bits  vio_rst_bundle
#   probe2  10 bits  vio_ddr_axi
#   probe3  8 bits  vio_boot_video
#   probe4  16 bits  vio_axi_error
#   probe5  32 bits  err_s0_aw_addr_r
#   probe6  32 bits  err_s0_ar_addr_r
#   probe7  32 bits  vio_boot_diag
#   probe8  32 bits  vio_boot_crc
#   probe9  68 bits  vio_l2c_stats
#   probe10  187 bits  vio_scsi_sd
#   probe11  48 bits  vio_fb_reader_stats
#   probe12  96 bits  video_dbg_snap
#   probe13  160 bits  video_dbg_place
#   probe14  84 bits  vio_scsi_c96
#   probe_out0[0] bypass SD; [1] release CPU; [2] SCC UART B select;
#   [3] full debug reset; [4] clear PRAM.
#   probe_out1: platform hard reset.
# CPU retirement counters remain available through JTAG-AXI performance CSRs.
# ---------------------------------------------------------------------------

set script_dir [file normalize [file dirname [info script]]]
set proj_root  [file normalize $script_dir/..]

if {[llength $argv] >= 2} {
    set bit_file [lindex $argv 0]
    set ltx_file [lindex $argv 1]
} else {
    set bit_file [file join $proj_root build vivado fpga_top.bit]
    set ltx_file [file join $proj_root build vivado fpga_top.ltx]
}

if {![file exists $bit_file]} {
    puts stderr "ERROR: bitstream not found: $bit_file"
    puts stderr "Run `make impl` first (PM-scheduled phase-boundary event)."
    exit 1
}
if {![file exists $ltx_file]} {
    puts stderr "ERROR: debug probes not found: $ltx_file"
    puts stderr "Expected a .ltx file emitted by write_debug_probes in vivado.tcl."
    exit 1
}

puts "=== Opening Vivado hw_manager ==="
open_hw_manager
connect_hw_server -allow_non_jtag

# Pick the first available target.  User can override via env if they
# have multiple JTAG cables attached.
set targets [get_hw_targets]
if {[llength $targets] == 0} {
    puts stderr "ERROR: no JTAG targets discovered.  Is the KU5P powered + cabled?"
    exit 2
}
current_hw_target [lindex $targets 0]
open_hw_target

# Pick the KU5P device on that target.
set dev [lindex [get_hw_devices] 0]
current_hw_device $dev
set_property PROGRAM.FILE    $bit_file $dev
set_property PROBES.FILE     $ltx_file $dev
set_property FULL_PROBES.FILE $ltx_file $dev

puts "=== Programming $bit_file ==="
program_hw_devices $dev
refresh_hw_device  $dev

# VIO objects show up once the device is refreshed with the .ltx loaded.
set vios [get_hw_vios]
if {[llength $vios] == 0} {
    puts stderr "ERROR: no VIO cores found after programming.  Did write_debug_probes succeed?"
    exit 3
}
set vio [lindex $vios 0]
puts "=== Attached VIO: $vio ==="

refresh_hw_vio $vio

# Text dashboard: one-shot snapshot of the named probes.  The hw_manager GUI
# gives live updates; this is the headless equivalent for the first
# sanity check post-program.
proc first_probe {vio candidates} {
    foreach name $candidates {
        set p [get_hw_probes -quiet -of_objects $vio $name]
        if {[llength $p] != 0} {
            return [lindex $p 0]
        }
    }
    return ""
}

proc show_probe {vio label candidates {property INPUT_VALUE}} {
    set p [first_probe $vio $candidates]
    if {$p eq ""} {
        puts [format "  %-28s: <probe missing>" $label]
        return ""
    }
    set v ""
    if {[catch {set v [get_property $property $p]} err]} {
        puts [format "  %-28s: <unreadable: %s>" $label $err]
        return ""
    }
    puts [format "  %-28s: 0x%s" $label $v]
    return $v
}

puts "────────────────────────────────────────────────────────────────"
puts " m68k-ooo JTAG VIO snapshot"
puts "────────────────────────────────────────────────────────────────"
show_probe $vio "dbg_pc" {dbg_pc_1 dbg_pc}
set rst_hex [show_probe $vio "rst/init bundle" {vio_rst_bundle_1 vio_rst_bundle}]
show_probe $vio "vio_ddr_axi" {vio_ddr_axi_1 vio_ddr_axi}
show_probe $vio "vio_boot_video" {vio_boot_video_1 vio_boot_video}
show_probe $vio "vio_axi_error" {vio_axi_error_1 vio_axi_error}
show_probe $vio "err_s0_aw_addr_r" {err_s0_aw_addr_r_1 err_s0_aw_addr_r}
show_probe $vio "err_s0_ar_addr_r" {err_s0_ar_addr_r_1 err_s0_ar_addr_r}
show_probe $vio "vio_boot_diag" {vio_boot_diag_1 vio_boot_diag}
show_probe $vio "vio_boot_crc" {vio_boot_crc_1 vio_boot_crc}
show_probe $vio "vio_l2c_stats" {vio_l2c_stats_1 vio_l2c_stats}
show_probe $vio "vio_scsi_sd" {vio_scsi_sd_1 vio_scsi_sd}
show_probe $vio "vio_fb_reader_stats" {vio_fb_reader_stats_1 vio_fb_reader_stats}
show_probe $vio "video_dbg_snap" {video_dbg_snap_1 video_dbg_snap}
show_probe $vio "video_dbg_place" {video_dbg_place_1 video_dbg_place}
show_probe $vio "vio_scsi_c96" {vio_scsi_c96_1 vio_scsi_c96}

if {$rst_hex ne ""} {
    scan $rst_hex %x rst
    puts [format "  %-28s: platform_resetn=%d core_rst=%d ddr_cal_done=%d boot_rom_ready=%d hdmi_i2c_done=%d fb_underflow=%d" \
        "rst/init decode" \
        [expr {($rst >> 5) & 1}] \
        [expr {($rst >> 4) & 1}] \
        [expr {($rst >> 3) & 1}] \
        [expr {($rst >> 2) & 1}] \
        [expr {($rst >> 1) & 1}] \
        [expr {$rst & 1}]]
}
puts "────────────────────────────────────────────────────────────────"
set po [first_probe $vio {vio_boot_ctrl probe_out0}]
if {[llength $po] != 0} {
    set ov_hex [get_property OUTPUT_VALUE $po]
    scan $ov_hex %x ov
    set bypass_sd    [expr {$ov & 1}]
    set release_cpu  [expr {($ov >> 1) & 1}]
    set scc_uart_sel_b [expr {($ov >> 2) & 1}]
    set full_dbg_rst [expr {($ov >> 3) & 1}]
    set pram_clear [expr {($ov >> 4) & 1}]
    puts [format "  probe_out0 jtag_boot_ctl : 0x%s (bypass_sd=%d release_cpu=%d scc_uart_sel_b=%d full_dbg_rst=%d pram_clear=%d)" \
                  $ov_hex $bypass_sd $release_cpu $scc_uart_sel_b $full_dbg_rst $pram_clear]
}
puts ""
puts "Switch to the Vivado GUI's Hardware Manager pane for live updates."
puts "Re-run this script (or `refresh_hw_vio \[get_hw_vios\]`) to snapshot again."
