# resume_from_place.tcl -- route an existing place.dcp without redoing synthesis.
#
# The flow's own crash note prescribes exactly this ("the placement checkpoint
# survives, so re-open place.dcp and route it with another directive rather than
# rebuilding the ~50 minutes of synthesis and placement") but no code path
# existed for it. Everything below line 1 of the slice is copied VERBATIM out of
# vivado.tcl's post-route section, which depends only on $output_dir.
#
# Usage: vivado -mode batch -source synth/resume_from_place.tcl \
#            -tclargs <output_dir> <route_directive>
set output_dir      [lindex $argv 0]
set route_directive [lindex $argv 1]

# The copied slice below reads three variables that vivado.tcl establishes in its
# own prologue.  Leaving them undefined is not a cosmetic gap: the slice dies at
# the incremental-reuse test with `can't read "no_incremental"` AFTER route.dcp is
# written but BEFORE any report is, so a resumed route produced a checkpoint whose
# timing you then had to re-derive by hand.  Define them here with the same
# semantics vivado.tcl gives them.
set proj_root [file normalize [file dirname [info script]]/..]
set no_incremental 1
set incremental_ref_dcp ""
if {[info exists ::env(NO_INCREMENTAL)] && $::env(NO_INCREMENTAL) ne ""} {
    set no_incremental [expr {$::env(NO_INCREMENTAL) ? 1 : 0}]
}
if {[info exists ::env(INCREMENTAL_REF_DCP)] && $::env(INCREMENTAL_REF_DCP) ne ""} {
    set incremental_ref_dcp [file normalize $::env(INCREMENTAL_REF_DCP)]
}
file mkdir $output_dir/reports $proj_root/synth/timing_reports
open_checkpoint $output_dir/checkpoints/place.dcp
puts "=== RESUME: opened place.dcp, routing with -directive $route_directive ==="
phys_opt_design -directive AggressiveExplore
route_design -directive $route_directive
# video glitching on this board before. If a pass drives WHS negative the loop stops
# immediately and says so, rather than banking a setup gain that is not real.
proc _pr_wns {} { return [get_property SLACK [get_timing_paths -delay_type max -max_paths 1 -nworst 1]] }
proc _pr_whs {} { return [get_property SLACK [get_timing_paths -delay_type min -max_paths 1 -nworst 1]] }
if {[get_property SLACK [get_timing_paths -delay_type min_max]] < 0} {
    set pr_dirs {AggressiveExplore AlternateReplication AggressiveFanoutOpt Explore AlternateReplication AggressiveExplore}
    set pr_max  [expr {[info exists ::env(POST_ROUTE_PHYSOPT_MAX)] ? $::env(POST_ROUTE_PHYSOPT_MAX) : 6}]
    set pr_prev [_pr_wns]
    set pr_whs0 [_pr_whs]
    set pr_ckpt [file join $output_dir checkpoints post_route_holdclean.dcp]
    file mkdir [file join $output_dir checkpoints]
    puts [format "=== POST-ROUTE PHYS_OPT: start WNS %.3f WHS %.3f (max %d passes) ===" $pr_prev $pr_whs0 $pr_max]
    set pr_i 0
    foreach pr_d $pr_dirs {
        if {$pr_i >= $pr_max} { puts "=== POST-ROUTE PHYS_OPT: pass cap reached ==="; break }
        incr pr_i
        # Snapshot the current (hold-clean) state so a pass that trades hold away can be
        # rolled back instead of being shipped.
        if {[catch {write_checkpoint -force $pr_ckpt} pr_ck_err]} {
            puts "WARN: could not write rollback checkpoint: $pr_ck_err"
        }
        if {[catch {phys_opt_design -directive $pr_d} pr_err]} {
            puts "WARN: post-route phys_opt_design ($pr_d) failed: $pr_err"
            continue
        }
        set pr_now [_pr_wns]; set pr_hold [_pr_whs]
        puts [format "=== POST-ROUTE PHYS_OPT pass %d (%s): WNS %.3f (was %.3f, delta %+.3f)  WHS %.3f ===" \
                     $pr_i $pr_d $pr_now $pr_prev [expr {$pr_now - $pr_prev}] $pr_hold]
        # Closure takes precedence over relative gain: a hold-only repair, or
        # reduced but still positive setup slack, is already a successful route.
        if {$pr_now >= 0.0 && $pr_hold >= 0.0} {
            puts "=== POST-ROUTE PHYS_OPT: setup and hold closed ==="
            break
        }
        # Judge hold against WHERE WE STARTED, not against zero.  Routing can (and on the
        # 2026-09-15 200 MHz run did) leave WHS already negative -- so there was no
        # hold-clean state to roll back to, and a `$pr_hold < 0` test rejected EVERY pass,
        # including ones that left hold untouched: it discarded +0.213 ns of real setup
        # gain and still finished hold-broken.  A pass is only bad if it makes hold WORSE
        # than the netlist we started from.
        if {$pr_hold < $pr_whs0 - 0.001} {
            # A pass traded hold away.  Two things were wrong with simply breaking here:
            #   1. It STOPPED THE WHOLE CAMPAIGN on the first hold blip, so passes 2..N
            #      never ran -- the 2026-09-14 200 MHz build earned +0.291 ns on pass 1
            #      and then quit with 5 directives unused.
            #   2. It did NOT UNDO the offending pass, so the flow SHIPPED the
            #      hold-violating netlist anyway (that build: WHS -0.077, 70 failing hold
            #      endpoints) -- exactly the thing the comment above says corrupts data.
            # Correct behaviour: try to repair hold, keep the setup gain if repair works,
            # otherwise ROLL BACK to the last hold-clean checkpoint and continue trying
            # other directives from there.
            puts "=== POST-ROUTE PHYS_OPT: pass $pr_i drove HOLD negative (WHS $pr_hold); attempting repair ==="
            if {[catch {phys_opt_design -hold_fix} pr_hf_err]} {
                puts "WARN: phys_opt_design -hold_fix failed: $pr_hf_err"
            }
            set pr_hold [_pr_whs]; set pr_now [_pr_wns]
            # A successful repair is sufficient even if its positive hold
            # margin is smaller than the one before this optimization pass.
            if {$pr_now >= 0.0 && $pr_hold >= 0.0} {
                puts "=== POST-ROUTE PHYS_OPT: setup and hold closed after repair ==="
                break
            }
            if {$pr_hold < $pr_whs0 - 0.001} {
                puts [format "=== POST-ROUTE PHYS_OPT: hold repair FAILED (WHS %.3f); rolling back to last hold-clean checkpoint ===" $pr_hold]
                if {[file exists $pr_ckpt]} {
                    close_design
                    open_checkpoint $pr_ckpt
                    puts [format "=== POST-ROUTE PHYS_OPT: restored WNS %.3f WHS %.3f ===" [_pr_wns] [_pr_whs]]
                } else {
                    puts "=== POST-ROUTE PHYS_OPT: no checkpoint to restore; stopping with hold NEGATIVE (investigate) ==="
                    break
                }
                continue
            }
            puts [format "=== POST-ROUTE PHYS_OPT: hold repaired (WHS %.3f), WNS now %.3f; continuing ===" $pr_hold $pr_now]
        }
        if {$pr_now <= $pr_prev + 0.001} {
            # One directive plateau does not predict the next directive.
            # Preserve the best setup state if this pass regressed it, but
            # still try the remaining bounded schedule (including fanout).
            if {$pr_now < $pr_prev - 0.001 && [file exists $pr_ckpt]} {
                close_design
                open_checkpoint $pr_ckpt
                puts "=== POST-ROUTE PHYS_OPT: setup regressed; restored prior checkpoint ==="
            }
            puts "=== POST-ROUTE PHYS_OPT: no gain; trying next directive ==="
            continue
        }
        set pr_prev $pr_now
    }
    # Final hold repair can regress setup even without improving hold.
    # Preserve the current result and roll back on either timing regression
    # or tool failure. A failed checkpoint write must stop before mutation.
    if {[_pr_whs] < 0} {
        set pr_final_wns [_pr_wns]
        set pr_final_whs [_pr_whs]
        set pr_final_checkpoint [file join $output_dir checkpoints pre_final_hold_repair.dcp]
        write_checkpoint -force $pr_final_checkpoint
        puts [format "=== POST-ROUTE PHYS_OPT: final hold repair attempt (WHS %.3f) ===" $pr_final_whs]
        set pr_final_failed [catch {phys_opt_design -hold_fix} pr_fh_err]
        if {$pr_final_failed || [_pr_wns] < $pr_final_wns - 0.001 ||
                                [_pr_whs] < $pr_final_whs - 0.001} {
            puts "=== POST-ROUTE PHYS_OPT: final repair failed/regressed; restoring prior checkpoint ==="
            if {$pr_final_failed} {puts "WARN final hold_fix: $pr_fh_err"}
            close_design
            open_checkpoint $pr_final_checkpoint
        }
        puts [format "=== POST-ROUTE PHYS_OPT: after final repair WNS %.3f WHS %.3f ===" [_pr_wns] [_pr_whs]]
    }
    puts [format "=== POST-ROUTE PHYS_OPT done: WNS %.3f WHS %.3f after %d passes ===" [_pr_wns] [_pr_whs] $pr_i]
} else {
    puts "=== POST-ROUTE PHYS_OPT skipped: timing already met ==="
}

write_checkpoint -force $output_dir/checkpoints/route.dcp

# The SPI pads used to be false-pathed. Fail loudly if they disappear from
# analysis again, and preserve explicit reports for the physical IO budget.
foreach sd_output {sd_clk sd_mosi sd_cs_n} {
    set sd_paths [get_timing_paths -to [get_ports $sd_output] -delay_type max -max_paths 1]
    if {[llength $sd_paths] == 0} {error "SPI output $sd_output is not timed"}
    if {abs([get_property REQUIREMENT $sd_paths] - 4.0) > 0.001} {
        error "SPI output $sd_output lost its 4ns latency allocation"
    }
    puts "SPI_IO_BUDGET $sd_output slack=[get_property SLACK $sd_paths]"
}
set sd_input_paths [get_timing_paths -from [get_ports sd_miso] -delay_type max -max_paths 1]
if {[llength $sd_input_paths] == 0} {error "SPI input sd_miso is not timed"}
if {abs([get_property REQUIREMENT $sd_input_paths] - 3.0) > 0.001} {
    error "SPI input sd_miso lost its 3ns latency allocation"
}
puts "SPI_IO_BUDGET sd_miso slack=[get_property SLACK $sd_input_paths]"
report_timing -from [get_ports sd_miso] -delay_type max -max_paths 4 \
    -file $output_dir/reports/sd_miso_timing.rpt
report_timing -to [get_ports {sd_clk sd_mosi sd_cs_n}] -delay_type max -max_paths 6 \
    -file $output_dir/reports/sd_output_timing.rpt

# Incremental-compile reuse report + stash for the next iteration.  Vivado's
# report_incremental_reuse summarises how much placement+routing was carried
# over from the reference DCP — a per-impl indicator of how productive the
# incremental flow was for this change set.
if {!$no_incremental && $incremental_ref_dcp ne ""} {
    if {[file exists $incremental_ref_dcp]} {
        puts "=== INCREMENTAL REUSE REPORT ==="
        if {[catch {report_incremental_reuse \
                -file $output_dir/reports/incremental_reuse.rpt} reuse_err]} {
            puts "WARN: report_incremental_reuse failed: $reuse_err"
        } else {
            report_incremental_reuse
        }
    }
    # ONLY stash a reference that actually MET TIMING.
    #
    # This copy used to be unconditional, which made the incremental flow a
    # downward ratchet: a build that missed timing overwrote the reference,
    # so the NEXT incremental build started from a failing floorplan and was
    # biased to miss again, and so on.  Measured trajectory that exposed it
    # (2026-08-04): fabric_clk100 WNS 0.000 -> -0.173 -> -0.117 across three
    # consecutive builds whose RTL deltas were small and, in the last case,
    # confined to the LSU/MMU handshake while the violations sat in the MIG
    # DDR4 controller's own internals — i.e. placement drift, not logic.
    #
    # The comment on the "not found yet" branch above already promised
    # "will stash on success"; this makes that true.  A failing build now
    # leaves the previous good reference intact, so the next attempt still
    # starts from the best-known floorplan instead of the worst-recent one.
    # get_property on an empty path collection ERRORS rather than returning
    # "", so wrap the query: any failure means "we don't know", and not
    # knowing must not overwrite a good reference.
    set stash_ok 1
    if {[catch {
            set stash_wns [get_property SLACK \
                [get_timing_paths -delay_type max -max_paths 1]]
            set stash_whs [get_property SLACK \
                [get_timing_paths -delay_type min -max_paths 1]]
        } stash_err]} {
        set stash_ok 0
        puts "WARN: slack query failed ($stash_err)"
    }
    if {!$stash_ok || ![string is double -strict $stash_wns] \
                   || ![string is double -strict $stash_whs]} {
        puts "=== INCREMENTAL: slack unknown — NOT stashing (fail safe) ==="
    } elseif {$stash_wns < 0 || $stash_whs < 0} {
        puts [format \
            "=== INCREMENTAL: TIMING NOT MET (WNS %.3f, WHS %.3f) — reference left UNCHANGED ===" \
            $stash_wns $stash_whs]
        puts "===   (a failing routed DCP must never become the incremental reference)"
    } else {
        set ref_dir [file dirname $incremental_ref_dcp]
        if {![file exists $ref_dir]} {
            file mkdir $ref_dir
        }
        file copy -force $output_dir/checkpoints/route.dcp $incremental_ref_dcp
        puts [format \
            "=== INCREMENTAL: timing met (WNS %.3f, WHS %.3f) — stashed routed DCP to %s ===" \
            $stash_wns $stash_whs $incremental_ref_dcp]
    }
}

# ──────────────────────────────────────────────────────────────────────────────
# Final reports
# ──────────────────────────────────────────────────────────────────────────────
puts "=== FINAL REPORTS ==="
report_timing_summary \
    -max_paths 50 \
    -report_unconstrained \
    -file $output_dir/timing_summary.rpt

# POST-ROUTE timing under reports/, matching the naming convention of the
# other per-stage reports (reports/timing_synth.rpt, reports/timing_place.rpt).
#
# Added 2026-09-03.  The post-route summary was already being written -- but
# to $output_dir/timing_summary.rpt (plus the timestamped archive copy under
# synth/timing_reports/), NOT to reports/ and NOT under a timing_route name.
# Result: reports/ held timing_synth + timing_place and no route-stage peer,
# so anyone auditing a shipped build naturally read timing_place.rpt and drew
# conclusions from PLACE-stage numbers.  That is systematically wrong in both
# directions -- the router and post-route phys_opt close hold violations by
# construction and recover a large fraction of setup, and route can also
# introduce problems place never saw.
#
# Concrete instance of the harm (build/vivado_divfix_100mhz, 2026-09-03):
# reports/timing_place.rpt says WNS -0.368 ns / 16 failing setup endpoints
# and WHS -0.949 ns / 1986 failing hold endpoints, while the actual routed
# result for the same build is WNS +0.031 ns / WHS +0.010 ns / ZERO failing
# setup or hold endpoints.  An investigation started from the place report
# spent its time on 2002 endpoints that do not exist in the shipped
# bitstream.
report_timing_summary \
    -max_paths 10 \
    -file $output_dir/reports/timing_route.rpt

# Bus skew is checked INDEPENDENTLY of set_clock_groups -asynchronous and is
# NOT folded into report_timing_summary's headline WNS/WHS/TNS numbers, so a
# failing Gray-pointer skew bound would otherwise be invisible in every
# report this build emits.  That matters as of 2026-09-03: the async-FIFO
# Gray-pointer set_bus_skew pass (T6, above) had been applying to zero
# instances in every build; now that it actually applies to all of them, its
# result has to be recorded somewhere.
if {[catch {report_bus_skew -max_paths 10 \
        -file $output_dir/reports/bus_skew_route.rpt} bskew_err]} {
    puts "WARN: report_bus_skew failed: $bskew_err"
}

report_utilization \
    -hierarchical \
    -file $output_dir/reports/utilization_route.rpt

report_power \
    -file $output_dir/reports/power.rpt

report_drc \
    -file $output_dir/reports/drc.rpt

# Copy timing summary to archive with timestamp
set ts [clock format [clock seconds] -format "%Y%m%d_%H%M%S"]
file copy $output_dir/timing_summary.rpt \
          $proj_root/synth/timing_reports/timing_${ts}.rpt
puts "=== RESUME COMPLETE ==="

# ──────────────────────────────────────────────────────────────────────────────
# Bitstream.  Without this a resumed route yielded only a route.dcp, so the
# closing artifact of a split build could not be loaded without a third manual
# Vivado invocation.  Mirrors vivado.tcl's tail: validate the ADB BRAM is blank,
# write the blank bitstream, seal its patch bundle, export the probes.
# ──────────────────────────────────────────────────────────────────────────────
puts "=== WRITING BITSTREAM ==="
source [file dirname [info script]]/adb_firmware_mmi.tcl
validate_adb_firmware_bram
write_bitstream -force $output_dir/fpga_top.blank.bit
export_adb_firmware_bundle $output_dir/fpga_top.blank.bit
write_debug_probes -force $output_dir/fpga_top.ltx

# A buildinfo written here can only record what this process knows.  Say so
# explicitly rather than emitting a file that looks like the full manifest the
# single-process flow writes: an incomplete record that reads as authoritative
# is how the l2c_enable question cost a netlist grep on 2026-08-03.
set bi [open $output_dir/fpga_top.buildinfo.resume w]
puts $bi "note=written by resume_from_place.tcl; NOT the full manifest"
puts $bi "part=[get_property PART [current_design]]"
puts $bi "route_directive=$route_directive"
puts $bi "source_placement=$output_dir/checkpoints/place.dcp"
puts $bi "wns=[get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]"
puts $bi "whs=[get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]"
# Pulse width is a separate check class that the WNS/WHS pair does not cover, and
# this design has shipped a failing pulse-width endpoint inside the DDR4 PHY while
# both of those were positive.  Record it here too.
set rp_file $output_dir/reports/pulse_width.rpt
report_pulse_width -all_violators -file $rp_file
set rp_wpws "unknown"; set rp_fails "unknown"
if {![catch {
        set rp_fh [open $rp_file r]; set rp_txt [read $rp_fh]; close $rp_fh
        set rp_worst ""; set rp_n 0
        foreach rp_line [split $rp_txt "\n"] {
            if {[regexp {^(Min Skew|Max Skew|Min Period|Max Period|Low Pulse Width|High Pulse Width)\s+\S+\s+\S+\s+\S+\s+(-?[0-9.]+)\s+(-?[0-9.]+)\s+(-?[0-9.]+)\s} $rp_line -> rp_ct rp_req rp_act rp_slk]} {
                if {$rp_worst eq "" || $rp_slk < $rp_worst} { set rp_worst $rp_slk }
                if {$rp_slk < 0} { incr rp_n }
            }
        }
        if {$rp_worst ne ""} { set rp_wpws $rp_worst; set rp_fails $rp_n }
    } rp_err]} {} else { puts "WARN: could not extract WPWS: $rp_err" }
puts $bi "wpws=$rp_wpws"
puts $bi "pulse_width_failing_endpoints=$rp_fails"
puts "=== TIMING VERDICT: WNS=[get_property SLACK [get_timing_paths -delay_type max -max_paths 1]] WHS=[get_property SLACK [get_timing_paths -delay_type min -max_paths 1]] WPWS=$rp_wpws pulse_width_failures=$rp_fails ==="
puts $bi "bitstream=fpga_top.blank.bit"
puts $bi "debug_probes=fpga_top.ltx"
puts $bi "generated_utc=[clock format [clock seconds] -gmt 1 -format {%Y-%m-%dT%H:%M:%SZ}]"
close $bi
puts "=== RESUME BITSTREAM COMPLETE: $output_dir/fpga_top.blank.bit ==="
