# synth/ila_ifetch_axi_capture.tcl -- round 8: real-hardware ILA capture of
# the instruction-fetch AXI master (axi_i / ifa_*) and the L2C-arbitrated
# fetch sub-port, at the moment the IF-stage fetch PC (probe28, ila_if_pc)
# permanently stops advancing -- docs/BUG_calibration_word_misplaced_0d00.md
# Part 54's precise closing recommendation.
#
# BACKGROUND: Part 54's round-7 capture DEFINITIVELY REFUTED the coreHalted
# hypothesis (Part 53): a live-hang re-arm capture showed `coreHalted=0`,
# `haltReason=NONE`, `DcachePlugin.diagFaultValid=0` for the ENTIRE
# 4096-sample window. The real reason `headReady` never fires: `count > 0`
# reads FALSE the whole capture -- the ROB is genuinely, physically EMPTY.
# `dbg_ila_rob_pc_w` (the trigger every round since Part 19 has used) was
# proven to be a STALE `readAsync` artifact (RobPlugin.scala:882, an
# ungated async Mem read with no count>0/valid qualifier) -- NOT a live
# "still in the ROB" signal. Independently, the pre-existing `ila_if_pc`
# tap (probe28, a round-3 addition) is ALSO frozen in the same capture, at
# a DIFFERENT address (0x4088711e, 8 bytes earlier) -- zero transitions the
# whole window. This overturns the entire retire-side framing every round
# since Part 36 has operated under: the CPUSHL at 0x40887126 most likely
# already retired cleanly; the real permanent stall is UPSTREAM, in
# fetch/dispatch.
#
# THIS ROUND (8): retargets the trigger OFF the now-known-stale
# `dbg_ila_rob_pc_w` and onto a purpose-built "IF-stage fetch PC has not
# moved in >=512 consecutive cycles" detector (`ila_if_pc_stuck[16]`, a
# saturating counter built directly from `ila_if_pc`/`dbg_ila_if_pc_w` --
# threshold is >4x Part 54 S6's own measured normal-case maximum of 124
# cycles for a legitimate transient ROB-empty/refill dwell, comfortable
# margin against a false trigger). Adds direct taps on BOTH the core-side
# fetch AXI master (`axi_i` / `ifa_*`, already top-level SoC wires,
# fpga_top_cpu.vh -- NO cpu040 RTL change needed this round) and the
# L2C-arbitrated fetch sub-port (l2c.v's brand-new `dbg_fetch_snap` output
# -- neither of L2C's two pre-existing debug snapshots, dbg_l2c_write_snap/
# dbg_l2c_master_snap (probes 41/42), carries fetch-vs-LSU source-tagged
# state).
#
# PROBE MAPPING (this repo, round 8, C_NUM_OF_PROBES bumped 46->50; does
# NOT touch any prior round's mapping, which stays live in the SAME
# capture for cross-checking):
#
#   probe46 16b  ila_fetch_snap = dbg_l2c_fetch_snap (l2c.v):
#       [0] f_axi_arvalid      [1] f_axi_arready
#       [2] hit_rsp_valid      [3] hit_rsp_is_fetch
#       [4] mshr_rsp_valid     [5] mshr_rsp_is_fetch
#       [6] fetch_q_avail      [7] fetch_consume_c
#       [8] f_rvalid_q         [9] f_axi_rready (CPU-side ready)
#       [10] fq_pair_have (quadrant-pair straddling for this ID)
#   probe47 14b  ila_ifetch_core_state -- axi_i (ifa_*) core-side view,
#                unaffected by anything inside L2C:
#       [0] ifa_arvalid  [1] ifa_arready
#       [2] ifa_rvalid   [3] ifa_rready   [4] ifa_rlast
#       [8:5] ifa_arid[3:0]   [12:9] ifa_rid[3:0]
#   probe48 32b  ila_ifetch_araddr = ifa_araddr verbatim.
#   probe49 17b  ila_if_pc_stuck = {if_pc_stuck_q, if_pc_stall_cnt_q[15:0]}
#                -- THE NEW TRIGGER SOURCE. bit[16] goes sticky-1 once
#                probe28 (if_pc) has read unchanged for >=512 consecutive
#                cycles.
#
# WHAT TO LOOK FOR in a decoded capture:
#   1. probe47[0] (ifa_arvalid) stuck at 1 with probe47[1] (ifa_arready)
#      stuck at 0 for the whole post-stall window -> a genuine bus hang:
#      the CPU is presenting an AR nobody ever accepts. Cross-check
#      probe46[1] (f_axi_arready, the same signal just past the
#      ROM-mirror address fold) to see whether L2C's own front door is
#      the one refusing -- if so, read probe41/probe42 (existing L2C
#      snapshots, dbg_l2c_write_snap/dbg_l2c_master_snap) for WHY
#      (id_busy_c/rst_busy/pipe_id_haz_c stuck at the front door, or the
#      DRAM-side master port itself wedged, mf_arvalid stuck high with
#      mf_arready never firing).
#   2. probe47[0]/[1] BOTH idle (no AR ever presented, not even a
#      refused one) -> the stall is upstream of the AXI master entirely,
#      inside cpu040's own IcachePlugin miss-dispatch/MSHR-allocate
#      logic (never even reaching arHoldValid). This points at a cpu040
#      RTL bug (a lost fetch credit, a stuck MSHR entry, a missed
#      allocate condition), NOT an SoC/L2C/DRAM issue -- read
#      IcachePlugin.scala's IDLE/miss-detect arm directly next.
#   3. probe47[2] (ifa_rvalid) stuck at 1 with probe47[3] (ifa_rready)
#      stuck at 0 -> a genuine core-side backpressure/credit bug: L2C
#      delivered a response cpu040 never consumes. Cross-check
#      probe46[8]/[9] (f_rvalid_q/f_axi_rready) to confirm the SAME beat
#      is stuck at the L2C->CPU boundary (both would read stuck-1/stuck-0
#      too), not manufactured further downstream. If confirmed, read
#      IcachePlugin.scala's `axi.r.ready := demandRspMatch || pfRspMatch`
#      and the MSHR control-file bits (`mshrArSent`/`mshrComplete`/
#      `refillActive`) directly for why neither match holds despite a
#      live response.
#   4. probe46[10] (fq_pair_have) stuck at 1 with no matching probe46[6]/
#      [7] activity, while probe47/probe48 show the CPU-side AR/R channel
#      otherwise idle -> a stuck L2C-internal quadrant-pair reassembly
#      (first 128b quadrant captured, second never arrives) -- an L2C
#      RTL bug (l2c.v's `fq_have`/`fq_pair_have` table) distinct from
#      both of the above, upstream of the CPU-visible AXI signals ever
#      showing an outstanding AR at all.
#   5. If NONE of probe46/47/48 show anything stuck (all channels idle,
#      no outstanding AR, no pending response) even though probe49[16]
#      is set -> the fetch PC stall is not an AXI-transaction hang at
#      all; re-examine whether `dbg_ila_if_pc_w` (== btbUpdPc, per its
#      round-3 connection-site comment) is itself a reliable "IF-stage
#      fetch PC" proxy, or whether the real blocker is a purely
#      combinational/control-path stall with no outstanding bus
#      transaction (e.g. a stuck arbitration priority, a permanently
#      false accept condition) -- treat this as a DIFFERENT class of
#      finding, do not force an AXI-hang narrative onto data that does
#      not support it.
#
# METHOD: same two-phase undisturbed-boot discipline as every prior round
# (NOT halt-based -- Part 40's armed-breakpoint finding still applies).
# Once a trial reaches the frozen state (poll `halt-status` for a static
# `pc_live`), the board stays live and genuinely frozen -- prefer
# re-arming the ILA directly against the ALREADY-stuck CPU (Part 53's own
# technique) over a fresh cold boot whenever a board from a prior trial is
# still sitting there.
#
# TRIGGER_POSITION set NEAR THE END of the 4096-sample buffer (mostly
# PRE-trigger), UNLIKE every prior round's small position (~128) -- this
# round's trigger condition (`if_pc_stuck_q`) itself takes >=512 cycles to
# assert, so by the time it fires the "interesting" AR/R traffic (the last
# successful transaction before the wedge, and the transition into the
# stuck state) is ALREADY behind the trigger point. A small TRIGGER_POSITION
# would show only the already-settled frozen state, exactly what round 7
# already fully characterized for the retire side. Default here: 3968/4096
# (~97% pre-trigger), leaving a ~128-sample post-trigger tail to confirm
# the state stays settled afterward.
#
# **`wait_on_hw_ila -timeout N` is in MINUTES, not seconds** -- keep the
# Vivado-side timeout SHORT and rely on the caller's own `pc_live` polling
# (observing the frozen fetch PC directly) as the primary signal, with a
# post-hoc sanity check on the written CSV as the final gate.
#
# Usage (interactively, via jtag_repl.tcl's `tcl` passthrough, in the SAME
# session that already holds hw_target open):
#   tcl set ::ila0 [lindex [get_hw_ilas -of_objects $::hw_dev] 0]
#   tcl set ::p_stuck [get_hw_probes ila_if_pc_stuck -of_objects $::ila0]
#   tcl set_property CONTROL.TRIGGER_POSITION 3968 $::ila0
#   tcl set_property CONTROL.TRIGGER_CONDITION AND $::ila0
#   tcl set_property TRIGGER_COMPARE_VALUE eq17'b1XXXXXXXXXXXXXXXX $::p_stuck
#   tcl run_hw_ila $::ila0
#   [reset, or just wait if re-arming against an already-frozen CPU]
#   tcl wait_on_hw_ila -timeout 2 $::ila0
#   tcl upload_hw_ila_data $::ila0; set ::d [get_hw_ila_data -of_objects $::ila0]; write_hw_ila_data -csv_file /tmp/ila_ifetch_axi_capture.csv -force $::d
#
# Or standalone (separate `vivado -mode batch` process -- only if NOT
# already attached via jtag_repl.tcl, to avoid a second contending JTAG
# connection):
#   vivado -mode batch -nojournal -nolog -source synth/ila_ifetch_axi_capture.tcl \
#       -tclargs <ltx_file> <jcmd_path> [reset|noreset] [timeout_min]

if {[llength $argv] < 2} {
    puts stderr "ERROR: usage: ila_ifetch_axi_capture.tcl <ltx_file> <jcmd_path> \[reset|noreset\] \[timeout_min\]"
    exit 2
}

set ltx_file    [lindex $argv 0]
set jcmd_path   [lindex $argv 1]
set do_reset    [expr {[llength $argv] >= 3 ? [lindex $argv 2] : "reset"}]
set timeout_min [expr {[llength $argv] >= 4 ? [lindex $argv 3] : 2.0}]

puts "=== Opening Vivado hw_manager ==="
open_hw_manager
connect_hw_server -allow_non_jtag

set targets [get_hw_targets]
if {[llength $targets] == 0} {
    puts stderr "ERROR: no JTAG targets discovered."
    exit 2
}
current_hw_target [lindex $targets 0]
open_hw_target

set dev [lindex [get_hw_devices] 0]
current_hw_device $dev
set_property PROBES.FILE $ltx_file $dev
refresh_hw_device $dev -update_hw_probes true

set ilas [get_hw_ilas -of_objects $dev]
if {[llength $ilas] == 0} {
    puts stderr "ERROR: no hw_ila core found on device -- was ENABLE_ILA=1 CPU=m68k040 used, and is fpga_top.ltx current?"
    exit 3
}
set ila [lindex $ilas 0]
puts "=== ILA core: $ila ==="

set p_trig [get_hw_probes ila_if_pc_stuck -of_objects $ila]
if {[llength $p_trig] == 0} {
    puts stderr "ERROR: expected trigger probe (ila_if_pc_stuck) not found."
    puts stderr "Available probes:"
    foreach p [get_hw_probes -of_objects $ila] { puts stderr "  $p" }
    exit 3
}

set_property CONTROL.TRIGGER_POSITION 3968 $ila
set_property CONTROL.TRIGGER_CONDITION AND $ila
set_property TRIGGER_COMPARE_VALUE eq17'b1XXXXXXXXXXXXXXXX $p_trig
puts "=== Trigger: ila_if_pc_stuck\[16\] (if_pc_stuck_q) == 1 -- fetch PC unchanged for >=512 cycles -- position=3968/4096 (mostly pre-trigger) ==="

run_hw_ila $ila
puts "=== ARMED at [clock format [clock seconds]] ==="

if {$do_reset eq "reset"} {
    after 300
    puts "=== Firing reset via: $jcmd_path reset ==="
    if {[catch {exec $jcmd_path reset} reset_out]} {
        puts "WARN: reset exec returned error/nonzero: $reset_out"
    } else {
        puts "reset output:\n$reset_out"
    }
} else {
    puts "=== NOT resetting (noreset) -- assumes the board is ALREADY stuck (re-arm-against-live-hang technique, Part 53 S6) ==="
}

puts "=== Waiting up to ${timeout_min} min for trigger (NOTE: MINUTES, not seconds) ==="
wait_on_hw_ila -timeout $timeout_min $ila
puts "=== wait_on_hw_ila returned at [clock format [clock seconds]] ==="

puts "=== Attempting upload ==="
if {[catch {upload_hw_ila_data $ila} uerr]} {
    puts "=== RESULT: NO_TRIGGER (upload threw: $uerr) -- the stuck condition was never reached in the window ==="
    exit 4
}

set data [get_hw_ila_data -of_objects $ila]
puts "=== data object: $data ==="
puts "=== RESULT: ATTEMPTING_DUMP ==="

catch {write_hw_ila_data -csv_file /tmp/ila_ifetch_axi_capture.csv -force $data}
puts "=== Wrote /tmp/ila_ifetch_axi_capture.csv (if supported) ==="

puts "=== DONE ==="
