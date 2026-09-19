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
#   Index  Width  Signal              Clock   What it tells you
#   ------ -----  ------------------- ------- --------------------------
#   probe0   1    hdmi_mmcm_locked    core    Pixel clock MMCM locked?
#                                             (1 = HDMI clocks alive)
#   probe1  12    video_debug_hcount core    VTG horizontal counter.
#   probe2  11    video_debug_vcount core    VTG vertical counter.
#   probe3  20    vram_rd_addr        pclk    Scanner VRAM read address
#                                             (should tick across ~768K
#                                              during active scan-out)
#   probe4  24    video_debug_rgb     pclk    Last RGB pixel sent to HDMI
#                                             transmitter
#   probe5  32    dbg_pc              core    Last-committed CPU PC —
#                                             shows boot progress / live
#   probe6  16    ddr_dbg_r_cnt[15:0] core    Low 16b of DDR read-beat
#                                             counter — "something is
#                                             happening on the bus"
#   probe7   6    {cpu_resetn,core_rst,ddr_cal_done,boot_rom_ready,
#                  hdmi_i2c_done,fb_underflow_sticky}
#                                     core    Reset + init state vector
#   probe8   1    s0_wready           core    DDR xbar write-ready (0 =
#                                             bus stuck)
#   probe9  32    dbg_committed       core    Catch-all: retired-insn
#                                             counter.
#   probe10  5    hdmi_ctrl           mixed   {resetn,i2c_done,de,vs,hs}
#   probe11  2    vram_read           mixed   {rd_valid,rd_en}
#   probe12 10    ddr_axi             core    DDR AXI handshake bits
#   probe13 32    write_counts        core    {vram_w_count,dafb_w_count}
#   probe14  8    vram_write          core    smoke + VRAM write handshakes
#   probe15  8    boot_video          mixed   boot/video status bundle
#   probe_out0[0] jtag_bypass_sd      core    Hold boot_fsm reset / bypass SD.
#   probe_out0[1] jtag_release_cpu    core    Release CPU after host ROM load.
#   probe_out0[2] jtag_force_cpu_rst  core    Force CPU reset while debugging.
#   probe_out0[3] jtag_full_dbg_rst   core    Full CPU-side cold-reset (task #256).
#   probe_out0[4] scc_uart_sel_b      core    0 = SCC channel A, 1 = channel B.
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
show_probe $vio "hdmi_mmcm_locked" {hdmi_mmcm_locked hdmi_mmcm_locked_1}
show_probe $vio "video_debug_hcount" {video_debug_hcount}
show_probe $vio "video_debug_vcount" {video_debug_vcount}
show_probe $vio "vram_rd_addr" {vram_rd_addr}
show_probe $vio "video_debug_rgb" {video_debug_rgb}
show_probe $vio "dbg_pc" {dbg_pc}
show_probe $vio "ddr_dbg_r_cnt" {ddr_dbg_r_cnt}
set rst_hex [show_probe $vio "rst/init bundle" {vio_rst_bundle_1 vio_rst_bundle}]
show_probe $vio "s0_wready" {s0_wready s0_wready_1}
show_probe $vio "dbg_committed" {dbg_committed}
show_probe $vio "hdmi_ctrl" {vio_hdmi_ctrl_1 vio_hdmi_ctrl}
show_probe $vio "vram_read" {vio_vram_read}
show_probe $vio "write_counts" {vio_write_counts}
show_probe $vio "vram_write" {vio_vram_write}
show_probe $vio "boot_rom_loading" {boot_rom_loading}
show_probe $vio "boot_error" {boot_error}
show_probe $vio "al9134_int" {al9134_int_IBUF al9134_int}

foreach label {
    s0_awvalid s0_awready s0_wvalid s0_wready s0_bvalid s0_bready
    s0_arvalid s0_arready s0_rvalid s0_rready
} {
    show_probe $vio $label [list $label]
}

if {$rst_hex ne ""} {
    scan $rst_hex %x rst
    puts [format "  %-28s: cpu_resetn=%d core_rst=%d ddr_cal_done=%d boot_rom_ready=%d hdmi_i2c_done=%d fb_underflow=%d" \
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
    set force_cpu_rst [expr {($ov >> 2) & 1}]
    set full_dbg_rst [expr {($ov >> 3) & 1}]
    set scc_uart [expr {(($ov >> 4) & 1) ? "B" : "A"}]
    puts [format "  probe_out0 jtag_boot_ctl : 0x%s (bypass_sd=%d release_cpu=%d force_cpu_rst=%d full_dbg_rst=%d scc_uart=%s)" \
                  $ov_hex $bypass_sd $release_cpu $force_cpu_rst $full_dbg_rst $scc_uart]
}
puts ""
puts "Switch to the Vivado GUI's Hardware Manager pane for live updates."
puts "Re-run this script (or `refresh_hw_vio \[get_hw_vios\]`) to snapshot again."
