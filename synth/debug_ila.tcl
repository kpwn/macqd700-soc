# synth/debug_ila.tcl — JTAG ILA IP generator (out-of-context, cached)
# ──────────────────────────────────────────────────────────────────────────
#
# Generates the `debug_ila` Vivado IP (xilinx.com:ip:ila) out-of-context
# into $output_dir/ip/debug_ila and caches the resulting DCP.  Mirrors the
# `gen_debug_vio_ip` flow in synth/vivado.tcl — see the long comment
# block at lines ~30-50 of vivado.tcl for why we MUST use the
# `create_ip` flow (not `create_debug_core` / `mark_debug` auto-insertion)
# in Vivado 2025.2.
#
# Probe map:
#
#   probe0   1b   a7_macro_locked           CPU pipeline (commit.v)
#   probe1   7b   a7_macro_pre_phys[6:0]
#   probe2   7b   committed_a7_phys[6:0]
#   probe3   1b   mb_drop
#   probe4   1b   flush_en
#   probe5   5b   {a7_illegitimate_step, rob_pop, rob_is_last_uop, mb_empty,
#                  drain_active}
#                 — bit[4] is the A7-delta classifier (= the trigger the
#                   user usually wants for the post-DAFB A7-drift wedge).
#   probe6   5b   rob_arch_dst[4:0]
#   probe7   7b   rob_phys_old[6:0]
#   probe8   7b   rob_phys_dst[6:0]
#   probe9   8b   {store_commit_exc, take_trace, take_exc, take_irq_arm,
#                  take_irq_fire_q, take_irq_preempt, take_rte,
#                  ccr_settle_in_flight}
#   probe10 32b   arch_a7_val[31:0]          live A7
#   probe11 16b   dbg_committed[15:0]        retire counter (low 16)
#   probe12 32b   rob_pc[31:0]               PC of retiring µop
#   probe13 33b   {raw_daxi_awaddr, raw_daxi_awvalid & dc_aw_is_evict}
#                                            — dcache writeback address+valid
#   probe14 40b   AXI W-channel snapshot:
#                  [39:8] AWADDR, [7] AWVALID, [6] AWREADY, [5] WVALID,
#                  [4]    WREADY, [3] BVALID, [2] BREADY, [1] WLAST,
#                  [0]    dc_aw_is_evict
#
# v3 (2026-05-26) — PRF write-port observation.  All physical PRF write
# sources from m68k_core_execute.vh are exported here so a single ILA
# capture identifies which writer is corrupting PRF[committed_a7_phys].
#
#   probe15  9b   {cdb0_en, cdb0_has_dst, cdb0_phys[6:0]}    — ALU lane A
#   probe16  9b   {cdb1_en, cdb1_has_dst, cdb1_phys[6:0]}    — LSU + movec_bcast
#   probe17  9b   {cdb2_en, cdb2_has_dst, cdb2_phys[6:0]}    — ALU lane B
#   probe18  8b   {cdb_alu_hi_en,         cdb_alu_hi_phys[6:0]} — MUL.L hi
#   probe19  9b   {a7_writeback_en, a7_writeback_phys[6:0], 1'b0}
#                                                             — exc-entry SP push
#   probe20 32b   cdb0_data[31:0]
#   probe21 32b   cdb1_data[31:0]
#   probe22 32b   a7_writeback_val[31:0]
#   probe23  1b   ila_prf_write_targets_01                    — derived trigger
#                 ORs all five write sources gated on dst-phys == 7'd1.
#                 vec_ssp is also OR'd when committed_a7_phys == 7'd1.
#                 sp_slot writes intentionally NOT OR'd here because
#                 they target the pinned USP/SSP/ISP slots, not the
#                 renameable phys 0x01.
#
# v4 (2026-05-27) — snap-mux bypass + IF/BPU PC tap.
#
#   probe24 32b   prf[committed_a7_phys]   — RAW PRF read, no mux gate.
#                                            True A7 regardless of halt.
#   probe25 32b   prf[7'd1]                — RAW phys-01 sanity tap
#                                            (committed_a7_phys=0x01 at wedge).
#   probe26  7b   snap_prf_idx[6:0]        — JTAG snap chain index
#                                            (= what arch_a7 mirrors when halted).
#   probe27  1b   effective_halt           — when 1, probe10 starts lying.
#   probe28 32b   if_pc[31:0]              — IF-stage fetch PC.  Catches
#                                            the WRONG-PC fetch (0x6db6db6d)
#                                            the cycle before pcmis.
#   probe29 32b   pred_target_sel[31:0]    — BPU/RAS predicted next-PC.
#                                            Shows if BPU mispredicted to
#                                            the wild target.
#
#   Trigger recipe:  HW Manager trigger on probe27 = 1.  Capture window
#                    is the cycle the CPU auto-halts; probe24 + probe26
#                    show the true A7 vs. snap address that probe10 is
#                    mirroring, and probe28/29 carry the IF/BPU PC
#                    history immediately preceding the halt.
#
# v5 (2026-07-04) — VIA1 PA0/DDRA strap-read race hunt (STM/CTE
# factory-diagnostic-module HW divergence).  See the long comment above
# `ila_via1_snap` in fpga_top_debug_vio.vh for the hypothesis.
#
#   probe30 24b   {ddra[7:0], ora[7:0], pb_via1_wr, pb_via1_rd,
#                  pb_via1_ack, 5'b0}      — raw VIA1 internal regs +
#                                            pb-side access strobes,
#                                            hierarchical-referenced
#                                            straight out of u_via1
#                                            (pb_clk domain — async into
#                                            this core_clk-clocked ILA,
#                                            acceptable for this
#                                            exploratory capture).
#
#   Trigger recipe: HW Manager trigger on probe12 (rob_pc) ==
#                   32'h40846cce (the VIA1 PA0 BTST instruction).
#                   Inspect probe30's ddra[0]/ora[0] at and slightly
#                   before the trigger sample: if ddra[0] != 0 at that
#                   point, the preceding DDRA-clear write had not yet
#                   propagated through the pb_clk-domain VIA1 register
#                   — confirming the cross-domain race hypothesis.
#
# v6 (2026-07-04) — A7/RTE-finalize investigation.
#
#   probe31  7b   exc_held_a7_phys[6:0]    — phys reg RTE/exc-entry
#                                            restores A7 from
#   probe32  1b   take_rte_finalize        — RTE-pop atomic-restore
#                                            pulse; trigger on this ==1
#                                            to bracket the event
#   probe33  1b   supervisor_mode          — committed SR.S bit
#
#   Trigger recipe: HW Manager trigger on probe32 == 1.  Cross-reference
#   probe2 (committed_a7_phys), probe31 (exc_held_a7_phys), and probe22
#   (a7_writeback_val) at and immediately after the trigger sample to
#   see exactly what A7 gets restored to and from which phys slot.
#
# v7 (2026-07-14) — MOVEA.L (SP),SP wild-jump / finalize address-data
# crossover investigation.  Exposes the exact signals commit.v's
# take_finalize path uses for redirect_pc, so a live HW capture can
# settle whether a suspected LSU/dcache address-data crossover is real,
# entirely via runtime ILA trigger reconfiguration (no RTL rebuild per
# PC of interest).
#
#   probe34 32b   exc_held_fault_pc[31:0]  — resume/interrupted PC
#                                            commit.v associates with
#                                            the completing finalize
#   probe35 32b   rob_brtgt[31:0]          — what take_finalize latches
#                                            into redirect_pc
#   probe36 32b   dc_rdata[31:0]           — raw LSU load data for the
#                                            same completing transaction
#   probe37  1b   take_finalize            — dedicated trigger bit
#
#   Trigger recipe: AND probe37 == 1 with probe34 == <target resume PC>
#   (see rtl/soc/fpga_top_debug_vio.vh for the exact Tcl) to isolate the
#   one finalize transaction under investigation among the many
#   unrelated ones firing during boot.  Compare probe36 (dc_rdata) vs
#   probe35 (rob_brtgt) at the trigger sample: MATCH means the LSU read
#   correct data and the corruption is downstream (ROB/completion-bus);
#   MISMATCH means the D-cache itself returned the wrong data.
#
# v8 (2026-07-14) — stale-ROB-slot / dispatch-clear race investigation.
# v7's live capture showed rob_brtgt landing on the wild jump target
# with dc_rdata matching it exactly, but no live decode-time path was
# found that could set is_rts=1 for the offending MOVEA.L (SP),SP.
# These probes let a capture tell "the retiring instruction's own LSU
# transaction really was is_rts-flagged" from "a stale/leftover
# branch-taken write from a PRIOR occupant of the same ROB slot".
#
#   probe38  1b   rob_brt                  — commit_br_taken, the
#                                            retiring ROB entry's own
#                                            "take this branch" bit
#   probe39  6b   snap_tag[5:0]            — ROB tag the is_rts-gated
#                                            LSU debug snapshot latched
#                                            (dbg_snap_tag)
#   probe40 32b   snap_ea[31:0]            — EA the is_rts-gated LSU
#                                            debug snapshot latched
#                                            (dbg_snap_ea)
#
#   Trigger recipe: same AND-on-probe34 recipe as v7, then read
#   probe38/39/40 at the trigger sample.  probe38==0 despite a wild
#   redirect would mean rob_brtgt was read while stale/not gated live
#   -- a genuine ROB-retire-mux bug.  probe38==1 with probe39 NOT
#   matching the retiring instruction's own ROB tag (cross-reference
#   against probe12/rob_pc's associated tag) means a stale completion
#   from a different, earlier occupant of the same slot landed here --
#   a dispatch-clear coverage gap in rob.v, not a decode bug.
#
# v9 (2026-07-24) — L2C/MIG-cal-race investigation. boot_fsm's first
# write into the L2C-fronted DDR path hangs on real HW (SD-loader LED
# flashes briefly then stops; s0_wready reads permanently 0 via VIO,
# confirmed on a VIO-only probe bypassing the stuck AXI-Lite path).
# l2c.v/l2c_ctrl.v have zero references to ddr_cal_done anywhere.
#
#   probe41 11b   l2c_ctrl.v front-door state:
#                 {s_awvalid,s_awready,s_wvalid,s_wready,aw_have,
#                  rst_busy,id_busy_c,write_avail,write_sel,st[1:0]}
#   probe42 11b   l2c.v master-port + miss-fill state:
#                 {m_axi_awvalid,m_axi_awready,m_axi_wvalid,
#                  m_axi_wready,m_axi_bvalid,aw_grant,aw_busy,aw_open,
#                  mf_arvalid,mf_arready,mf_rvalid}
#   probe43  1b   mig_cal_done             — DDR4 MIG calibration status
#
#   Trigger recipe: trigger on probe41[7] (s_wready falling edge is
#   also useful) or just capture continuously post-reset and look for
#   whether probe43 (mig_cal_done) is still 0 at the moment probe42's
#   mf_arvalid/mf_arready fire — that would confirm the fill-read races
#   ahead of calibration and explains the permanent hang.
#
# ── v10 probe map (2026-08-30) — RobPlugin headReady/sysRetire gate +
# coreHalted/haltReason/DcachePlugin-diagFault attribution. Boot-
# investigation round 7 (docs/BUG_calibration_word_misplaced_0d00.md Part
# 53's precise closing recommendation): round 6's real-hardware capture
# DEFINITIVELY REFUTED the S_DRAIN/sqDrained hypothesis for the live
# 0x40887126 CPUSHL hang (ExceptionUnit's commit-time sysOp sequencer never
# leaves IDLE — sqDrained/dcQuiesced both read TRUE), narrowing the real
# blocker to RobPlugin's own `headReady := (count>0) && completes(h0) &&
# !flushing && !coreHalted && !debugHalted` gate. `coreHalted` (a sticky
# fatal latch set by `dc.diagFault || exc.fsXlateFault || arbWedge ||
# rvHalt`) was flagged the single strongest lead.
#
#   probe44 32b   robGateState — see M68kCore.scala's dbg040 Area round-7
#                 doc comment for the exact bit layout:
#                   [0]     headReady
#                   [1]     completes(h0)
#                   [2]     sysValRdyStore(h0)
#                   [3]     coreHalted
#                   [4]     debugHalted
#                   [5]     count > 0
#                   [8:6]   RobPlugin.haltReason (HaltReason enum:
#                           0=NONE 1=DCACHE_DIAG 2=FS_XLATE
#                           3=RESET_VECTOR 4=ARBITER_WEDGE)
#                   [9]     DcachePlugin.diagFaultValid
#                   [12:10] DcachePlugin.diagFaultKind (0=WT-beat,
#                           1=drain-miss write-allocate, 2=eviction wb,
#                           3=CPUSH maintenance writeback)
#                   [14:13] DcachePlugin.diagFaultResp (raw AXI B-resp)
#   probe45 32b   diagFaultAddr[31:0] — DcachePlugin's own diagFaultAddr,
#                 verbatim (the physical address of whichever AXI
#                 writeback errored, if coreHalted turns out to be set)
#
#   Trigger recipe: same proven trigger source as every round since v1
#   (probe12 (rob_pc) == 0x40887126). Decision tree:
#     - probe44[3] (coreHalted) == 0 for the ENTIRE capture -> REFUTES the
#       coreHalted hypothesis; look at probe44[1]/[2] (completes(h0)/
#       sysValRdyStore(h0)) instead — whichever reads 0 is the real blocker.
#     - probe44[3] == 1 -> CONFIRMS coreHalted is latched. Read probe44[8:6]
#       (haltReason) to see WHICH producer fired. If haltReason==1
#       (DCACHE_DIAG), read probe44[12:10] (diagFaultKind) — kind=3 (CPUSH
#       maintenance writeback) would be the exact match for a CPUSHL-
#       triggered writeback error; read probe45 for the faulting address
#       and probe44[14:13] for the raw AXI response code.
#
# ── v11 probe map (2026-08-30) — instruction-fetch AXI master
# investigation. Boot-investigation round 8 (docs/
# BUG_calibration_word_misplaced_0d00.md Part 54's precise closing
# recommendation): round 7's real-hardware capture DEFINITIVELY REFUTED
# the coreHalted hypothesis (ROB genuinely, physically empty at the hang)
# and found the long-standing probe12 (rob_pc) == 0x40887126 trigger
# recipe (unchanged since Part 19) fires on a STALE readAsync artifact,
# not a live signal. This round retargets the trigger onto a purpose-
# built "IF-stage fetch PC has not moved in >=512 cycles" detector built
# from probe28 (ila_if_pc) and adds direct taps on both the core-side
# fetch AXI master (axi_i / ifa_*, top-level wires, no cpu040 RTL change)
# and the L2C-arbitrated fetch sub-port (l2c.v's new dbg_fetch_snap
# output — see rtl/soc/fpga_top_debug_vio.vh's round-8 comment block for
# the full bit layout + decision tree).
#
#   probe46 16b   fetch_snap    — l2c.v fetch-sourced front-door/response
#                 state (arvalid/arready, hit_rsp/mshr_rsp valid+is_fetch,
#                 fetch_q_avail/consume, f_rvalid_q/f_axi_rready,
#                 fq_pair_have)
#   probe47 14b   ifetch_core_state — axi_i (ifa_*) core-side AXI state:
#                 arvalid/arready/rvalid/rready/rlast + arid[3:0]/rid[3:0]
#   probe48 32b   ifetch_araddr — ifa_araddr verbatim
#   probe49 17b   if_pc_stuck   — {if_pc_stuck_q, if_pc_stall_cnt_q[15:0]}.
#                 THE NEW TRIGGER SOURCE: bit[16] goes sticky-1 once
#                 probe28 (if_pc) has read unchanged for >=512 consecutive
#                 cycles (>4x Part 54 S6's own measured normal-case
#                 maximum of 124 cycles).
#
#   Trigger recipe: probe49[16] == 1, TRIGGER_POSITION set near the END
#   of the 4096-sample buffer (mostly PRE-trigger) to capture the AR/R
#   traffic history leading up to the stall, not just the already-settled
#   frozen state.
#
# Total raw width:
#   probes0..14           = 1+7+7+1+1+5+5+7+7+8+32+16+32+33+40 = 202 bits
#   v3 add (probe15..23)  = 9+9+9+8+9+32+32+32+1 = 141 bits
#   v4 add (probe24..29)  = 32+32+7+1+32+32 = 136 bits
#   v5 add (probe30)      = 24 bits
#   v6 add (probe31..33)  = 7+1+1 = 9 bits
#   v7 add (probe34..37)  = 32+32+32+1 = 97 bits
#   v8 add (probe38..40)  = 1+6+32 = 39 bits
#   v9 add (probe41..43)  = 11+11+1 = 23 bits
#   v10 add (probe44..45) = 32+32 = 64 bits
#   v11 add (probe46..49) = 16+14+32+17 = 79 bits
#   ──────────────────────────────────────────────────────────────────
#   total                  = 814 bits
#
# 4096-sample depth x 735b = ~368 KiB raw.
# Practical Vivado-measured cost scales roughly with rounded-up 36Kb
# pages — KU5P has 480 RAMB36 / 960 RAMB18 — well within budget.  The
# ILA storage qualifier (1 pipeline stage) does not change.
# ──────────────────────────────────────────────────────────────────────────
proc gen_debug_ila_ip {output_dir part} {
    set ip_dir $output_dir/ip
    set ip_xci $ip_dir/debug_ila/debug_ila.xci
    set ip_dcp $ip_dir/debug_ila/debug_ila.dcp
    set ip_marker $ip_dir/debug_ila/m68k_ooo_ila_probe_map_v1.txt
    file mkdir $ip_dir

    set marker_ok 0
    if {[file exists $ip_marker]} {
        set marker_fh [open $ip_marker r]
        set marker_txt [read $marker_fh]
        close $marker_fh
        set marker_ok [expr {[string first "ila_probe_map=v14_ifetch_rdata" $marker_txt] >= 0 &&
                             [string first "part=$part" $marker_txt] >= 0 &&
                             [string first "probe_count=57" $marker_txt] >= 0 &&
                             [string first "probe5_width=5" $marker_txt] >= 0 &&
                             [string first "probe12_width=32" $marker_txt] >= 0 &&
                             [string first "probe13_width=33" $marker_txt] >= 0 &&
                             [string first "probe14_width=40" $marker_txt] >= 0 &&
                             [string first "probe23_width=1" $marker_txt] >= 0 &&
                             [string first "probe27_width=1" $marker_txt] >= 0 &&
                             [string first "probe30_width=24" $marker_txt] >= 0 &&
                             [string first "probe31_width=7" $marker_txt] >= 0 &&
                             [string first "probe33_width=1" $marker_txt] >= 0 &&
                             [string first "probe34_width=32" $marker_txt] >= 0 &&
                             [string first "probe37_width=1" $marker_txt] >= 0 &&
                             [string first "probe39_width=6" $marker_txt] >= 0 &&
                             [string first "probe40_width=32" $marker_txt] >= 0 &&
                             [string first "probe44_width=32" $marker_txt] >= 0 &&
                             [string first "probe45_width=32" $marker_txt] >= 0 &&
                             [string first "probe46_width=16" $marker_txt] >= 0 &&
                             [string first "probe47_width=14" $marker_txt] >= 0 &&
                             [string first "probe48_width=32" $marker_txt] >= 0 &&
                             [string first "probe49_width=17" $marker_txt] >= 0 &&
                             [string first "probe50_width=32" $marker_txt] >= 0 &&
                             [string first "probe51_width=32" $marker_txt] >= 0 &&
                             [string first "probe52_width=32" $marker_txt] >= 0 &&
                             [string first "probe53_width=32" $marker_txt] >= 0 &&
                             [string first "probe54_width=32" $marker_txt] >= 0 &&
                             [string first "probe55_width=16" $marker_txt] >= 0 &&
                             [string first "probe56_width=256" $marker_txt] >= 0 &&
                             [string first "depth=4096" $marker_txt] >= 0}]
    }
    if {[file exists $ip_dcp] && $marker_ok} {
        puts "=== Reusing cached debug_ila IP at $ip_dcp ==="
        return [list $ip_xci $ip_dcp]
    } elseif {[file exists [file dirname $ip_xci]]} {
        puts "=== Existing debug_ila IP is missing/stale/wrong-part; regenerating ==="
        file delete -force $ip_dir/debug_ila
    }

    puts "=== Generating debug_ila IP OOC into $ip_dir ==="
    create_project -in_memory -part $part -force
    create_ip -name ila -vendor xilinx.com -library ip -version 6.2 \
        -module_name debug_ila -dir $ip_dir
    set_property -dict [list \
        CONFIG.C_NUM_OF_PROBES         {57} \
        CONFIG.C_DATA_DEPTH            {4096} \
        CONFIG.C_TRIGOUT_EN            {false} \
        CONFIG.C_TRIGIN_EN             {false} \
        CONFIG.C_ADV_TRIGGER           {true} \
        CONFIG.C_INPUT_PIPE_STAGES     {1} \
        CONFIG.C_EN_STRG_QUAL          {1} \
        CONFIG.ALL_PROBE_SAME_MU       {true} \
        CONFIG.ALL_PROBE_SAME_MU_CNT   {2} \
        CONFIG.C_PROBE0_WIDTH          {1}  \
        CONFIG.C_PROBE1_WIDTH          {7}  \
        CONFIG.C_PROBE2_WIDTH          {7}  \
        CONFIG.C_PROBE3_WIDTH          {1}  \
        CONFIG.C_PROBE4_WIDTH          {1}  \
        CONFIG.C_PROBE5_WIDTH          {5}  \
        CONFIG.C_PROBE6_WIDTH          {5}  \
        CONFIG.C_PROBE7_WIDTH          {7}  \
        CONFIG.C_PROBE8_WIDTH          {7}  \
        CONFIG.C_PROBE9_WIDTH          {8}  \
        CONFIG.C_PROBE10_WIDTH         {32} \
        CONFIG.C_PROBE11_WIDTH         {16} \
        CONFIG.C_PROBE12_WIDTH         {32} \
        CONFIG.C_PROBE13_WIDTH         {33} \
        CONFIG.C_PROBE14_WIDTH         {40} \
        CONFIG.C_PROBE15_WIDTH         {9}  \
        CONFIG.C_PROBE16_WIDTH         {9}  \
        CONFIG.C_PROBE17_WIDTH         {9}  \
        CONFIG.C_PROBE18_WIDTH         {8}  \
        CONFIG.C_PROBE19_WIDTH         {9}  \
        CONFIG.C_PROBE20_WIDTH         {32} \
        CONFIG.C_PROBE21_WIDTH         {32} \
        CONFIG.C_PROBE22_WIDTH         {32} \
        CONFIG.C_PROBE23_WIDTH         {1}  \
        CONFIG.C_PROBE24_WIDTH         {32} \
        CONFIG.C_PROBE25_WIDTH         {32} \
        CONFIG.C_PROBE26_WIDTH         {7}  \
        CONFIG.C_PROBE27_WIDTH         {1}  \
        CONFIG.C_PROBE28_WIDTH         {32} \
        CONFIG.C_PROBE29_WIDTH         {32} \
        CONFIG.C_PROBE30_WIDTH         {24} \
        CONFIG.C_PROBE31_WIDTH         {7}  \
        CONFIG.C_PROBE32_WIDTH         {1}  \
        CONFIG.C_PROBE33_WIDTH         {1}  \
        CONFIG.C_PROBE34_WIDTH         {32} \
        CONFIG.C_PROBE35_WIDTH         {32} \
        CONFIG.C_PROBE36_WIDTH         {32} \
        CONFIG.C_PROBE37_WIDTH         {1}  \
        CONFIG.C_PROBE38_WIDTH         {1}  \
        CONFIG.C_PROBE39_WIDTH         {6}  \
        CONFIG.C_PROBE40_WIDTH         {32} \
        CONFIG.C_PROBE41_WIDTH         {11} \
        CONFIG.C_PROBE42_WIDTH         {11} \
        CONFIG.C_PROBE43_WIDTH         {1}  \
        CONFIG.C_PROBE44_WIDTH         {32} \
        CONFIG.C_PROBE45_WIDTH         {32} \
        CONFIG.C_PROBE46_WIDTH         {16} \
        CONFIG.C_PROBE47_WIDTH         {14} \
        CONFIG.C_PROBE48_WIDTH         {32} \
        CONFIG.C_PROBE49_WIDTH         {17} \
        CONFIG.C_PROBE50_WIDTH         {32} \
        CONFIG.C_PROBE51_WIDTH         {32} \
        CONFIG.C_PROBE52_WIDTH         {32} \
        CONFIG.C_PROBE53_WIDTH         {32} \
        CONFIG.C_PROBE54_WIDTH         {32} \
        CONFIG.C_PROBE55_WIDTH         {16} \
        CONFIG.C_PROBE56_WIDTH         {256} \
    ] [get_ips debug_ila]
    generate_target {synthesis} [get_ips debug_ila]
    synth_ip [get_ips debug_ila]
    set marker_fh [open $ip_marker w]
    puts $marker_fh "ila_probe_map=v14_ifetch_rdata"
    puts $marker_fh "part=$part"
    puts $marker_fh "probe_count=57"
    puts $marker_fh "probe5_width=5"
    puts $marker_fh "probe12_width=32"
    puts $marker_fh "probe13_width=33"
    puts $marker_fh "probe14_width=40"
    puts $marker_fh "probe23_width=1"
    puts $marker_fh "probe27_width=1"
    puts $marker_fh "probe30_width=24"
    puts $marker_fh "probe31_width=7"
    puts $marker_fh "probe33_width=1"
    puts $marker_fh "probe34_width=32"
    puts $marker_fh "probe37_width=1"
    puts $marker_fh "probe39_width=6"
    puts $marker_fh "probe40_width=32"
    puts $marker_fh "probe41_width=11"
    puts $marker_fh "probe42_width=11"
    puts $marker_fh "probe43_width=1"
    puts $marker_fh "probe44_width=32"
    puts $marker_fh "probe45_width=32"
    puts $marker_fh "probe46_width=16"
    puts $marker_fh "probe47_width=14"
    puts $marker_fh "probe48_width=32"
    puts $marker_fh "probe49_width=17"
    puts $marker_fh "probe50_width=32"
    puts $marker_fh "probe51_width=32"
    puts $marker_fh "probe52_width=32"
    puts $marker_fh "probe53_width=32"
    puts $marker_fh "probe54_width=32"
    puts $marker_fh "probe55_width=16"
    puts $marker_fh "probe56_width=256"
    puts $marker_fh "depth=4096"
    puts $marker_fh "advanced_trigger=true"
    puts $marker_fh "input_pipe_stages=1"
    close $marker_fh
    close_project
    return [list $ip_xci $ip_dcp]
}
