// rtl/fpga_top_debug_ctrl.vh — included from rtl/fpga_top.v
//
// CPU SOCKET binding + SoC-side debug/IRQ glue.
//
// Debug is now the CPU's EXPORTED dbg_axi SLAVE (see rtl/soc/cpu_socket.vh):
// the whole debug + control register map (build_id, live PC/arch state,
// halt/continue/step, breakpoints, arch-capture, dcache/icache probe+ops,
// ILA cfg, AND reset/halt via the debug-CSR) lives CPU-side behind that
// slave.  The SoC's JTAG-AXI master reaches it through the historical
// 0x5090_0000 debug window.  Since 2026-09-12 that window is served off
// THE DEBUG BUS (rtl/soc/axi_dbg_bus.v, instantiated in
// fpga_top_debug_host.vh) whenever a JTAG-AXI host is present:
//   debug_jtag_axi → axi_dbg_bus → dbg_core_* (32-bit AXI-Lite, 20-bit
//   addr, core_clk) → u_cpu.dbg_axi_*
// with NO crossbar, NO peripheral_bus and NO pb_clk crossing in between.
// Configurations with no JTAG-AXI host (PCIe/XDMA, lint/sim) keep the
// historical route: peripheral_bus dbg_* → axil_async_bridge → dbg_core_*.
// See the comment block at fpga_top_peripherals.vh's `ifdef JTAG_AXI_ENABLE.
//
// The legacy SoC-side debug_ctrl / debug_ctrl_stub (the ~130-port rich
// register block) and debug_stop_manager are GONE — they relocate into
// the CPU repo (Phase 4).  This include now only:
//   1. declares the few SoC-fabric control wires the reset tree / xbar /
//      VIO still reference (formerly debug_ctrl outputs);
//   2. resolves the CPU external IPL (peripheral IPL straight through —
//      JTAG IRQ-inject moves CPU-side via the debug-CSR);
//   3. instantiates cpu_stub through the socket, binding the masters to
//      the xbar CPU ports, the dbg_axi slave to the JTAG-AXI debug
//      window, and the control group to the SoC reset tree / xbar.
//
// Do not add module/endmodule wrappers or nettype directives.

    // ═══════════════════════════════════════════════════════════════════
    // SoC-fabric control wires formerly produced by debug_ctrl
    // ═══════════════════════════════════════════════════════════════════
    // dbg_ram_window_lg2 is consumed by the xbar (DDR RAM-window size).
    // It is now driven CPU-side (socket cpu_ram_window_lg2 → cpu.vh maps
    // it onto this wire).  The remaining wires below have no live SoC
    // consumer beyond a VIO anti-prune sink (fpga_top_debug_vio.vh); they
    // are tied off here so that sink stays valid and the JTAG IRQ-inject /
    // redirect / step features relocate cleanly CPU-side in Phase 4.
    wire [5:0]   dbg_ram_window_lg2;     // driven via cpu_ram_window_lg2 (cpu.vh)
    wire         dbg_step_req          = 1'b0;
    wire         dbg_init_done_override = 1'b0;
    wire [31:0]  dbg_redirect_pc       = 32'd0;
    wire         dbg_redirect_valid    = 1'b0;
    wire [2:0]   dbg_irq_inject_lvl    = 3'd0;
    wire         dbg_irq_inject_pulse  = 1'b0;
    // Live PC / retire-count taps relocated CPU-side in the socket split
    // (read them via the dbg_axi debug CSRs at 0x5090_xxxx).  Tied off
    // here only so the VIO dashboard probes (fpga_top_debug_vio.vh
    // probe_in5 / probe_in9) stay bound.
    wire [31:0]  dbg_pc                = 32'd0;
    wire [31:0]  dbg_committed         = 32'd0;

    // ═══════════════════════════════════════════════════════════════════
    // CPU external IPL
    // ═══════════════════════════════════════════════════════════════════
    // Peripheral IPL passes straight to the CPU.  JTAG IRQ injection now
    // lives CPU-side (DBG_IRQ_INJECT in the debug-CSR reached via dbg_axi),
    // so there is no SoC-side inject mux here any more.
    assign cpu_ipl_ext_w = cpu_ipl_periph_w;

    // ═══════════════════════════════════════════════════════════════════
    // CPU SOCKET: cpu_stub standalone; real CPU via submodule wrapper (CPU=m68k)
    // ═══════════════════════════════════════════════════════════════════
    // Bound here (rather than in fpga_top_cpu.vh) because the dbg_axi
    // slave terminates the JTAG-AXI debug window (dbg_core_*), whose wires
    // the peripheral bus declares in the include loaded just before this
    // one, and because cpu_ipl_ext_w is resolved above.
    // CPU=stub (default, standalone) binds cpu_stub; CPU=m68k (submodule)
    // binds m68k_axi_wrapper.  Both present the IDENTICAL socket surface
    // (rtl/soc/cpu_socket.vh), so only the module name differs — the
    // parameter overrides and ALL port bindings below are shared.  The
    // Makefile selects the build via +define+CPU_M68K (CPU=m68k).
    //
    // CPU=m68k040 (task #270 / SOC-3, cpu040/ submodule) binds
    // M68kSocketTop instead, in its OWN branch below rather than folding
    // into the shared cpu_stub/m68k_axi_wrapper body: M68kSocketTop (a)
    // takes NO module parameters (RESET_PC/FETCH_RESET_VECTORS/BUILD_ID
    // are all internal to the cpu040 core, not build-time knobs -- see
    // cpu040/src/main/scala/m68k040/top/M68kSocketTop.scala) and (b) has
    // not implemented the `ILA_ENABLE` dbg_ila_*/raw_daxi_* export group
    // (cpu_socket.vh group 7) -- so its port list is a strict SUBSET of
    // the shared body's, not a drop-in match for the same connection
    // list.  Confirmed against the actual generated port list
    // (cpu040/generated/M68kSocketTop.v, regenerated via `cd cpu040 &&
    // sbt "runMain m68k040.top.GenSocketTopVerilog"`): every remaining
    // port name/width/direction IS an exact match for cpu_socket.vh
    // (axi_i_rdata/wdata etc. at [255:0]/[127:0] as declared above).
    // CPU_M68K040 + ILA_ENABLE: 2026-08-27 boot-investigation update -- NOW
    // PARTIALLY SUPPORTED. The `dbg_ila_*_w` declarations below moved out of
    // the `else` branch (they were declared there only, out of scope for
    // this branch) so BOTH CPU selections can see them; cpu040 drives 9 of
    // the 44 (repurposed, see the mapping comment at the connection site
    // below -- reusing existing 1-bit slots rather than growing
    // synth/debug_ila.tcl's probe map / forcing an IP regen). The other 35
    // dbg_ila_*_w wires (v1-specific: commit_bundle, rob_phys_dst, cdb0/1
    // phys/data, a7_writeback, etc.) and all `raw_daxi_*` wires stay
    // declared-but-unconnected for CPU_M68K040, same as before this change
    // — harmless, an inert dashboard for those specific probe channels.
`ifdef ILA_ENABLE
    // ── ILA-only debug-export receiving wires ─────────────────────────
    // Socket exception (see rtl/soc/cpu_socket.vh group 7): promoted
    // straight out of whichever module binds u_cpu below.  Named to
    // match exactly what rtl/soc/fpga_top_debug_vio.vh's ILA_ENABLE
    // block already expects (it bit-rotted after the CPU→submodule
    // split; this is the fix — see docs/ila_a7_drift_probes.md).
    wire [25:0] dbg_ila_commit_bundle_w;
    wire [4:0]  dbg_ila_rob_arch_dst_w;
    wire [6:0]  dbg_ila_rob_phys_dst_w;
    wire [6:0]  dbg_ila_rob_phys_old_w;
    wire        dbg_ila_rob_pop_w;
    wire        dbg_ila_rob_is_last_uop_w;
    wire        dbg_ila_flush_en_w;
    wire [31:0] dbg_ila_rob_pc_w;
    wire [31:0] dbg_ila_arch_a7_w;
    wire        dbg_ila_dc_aw_is_evict_w;
    wire        dbg_ila_cdb0_en_w;
    wire        dbg_ila_cdb0_has_dst_w;
    wire [6:0]  dbg_ila_cdb0_phys_w;
    wire [31:0] dbg_ila_cdb0_data_w;
    wire        dbg_ila_cdb1_en_w;
    wire        dbg_ila_cdb1_has_dst_w;
    wire [6:0]  dbg_ila_cdb1_phys_w;
    wire [31:0] dbg_ila_cdb1_data_w;
    wire        dbg_ila_cdb2_en_w;
    wire        dbg_ila_cdb2_has_dst_w;
    wire [6:0]  dbg_ila_cdb2_phys_w;
    wire        dbg_ila_cdb_alu_hi_en_w;
    wire [6:0]  dbg_ila_cdb_alu_hi_phys_w;
    wire        dbg_ila_a7_writeback_en_w;
    wire [6:0]  dbg_ila_a7_writeback_phys_w;
    wire [31:0] dbg_ila_a7_writeback_val_w;
    wire        dbg_ila_vec_ssp_valid_w;
    wire        dbg_ila_sp_slot_write_en_w;
    wire [1:0]  dbg_ila_sp_slot_write_sel_w;
    wire [6:0]  dbg_ila_committed_a7_phys_w;
    wire [31:0] dbg_ila_real_a7_val_w;
    wire [31:0] dbg_ila_prf01_w;
    wire [6:0]  dbg_ila_snap_prf_idx_w;
    wire        dbg_ila_effective_halt_w;
    wire [31:0] dbg_ila_if_pc_w;
    wire [31:0] dbg_ila_pred_pc_w;
    // v6 (2026-07-04) — A7/RTE-finalize investigation taps.
    wire        dbg_ila_supervisor_mode_w;
    wire [6:0]  dbg_ila_exc_held_a7_phys_w;
    wire        dbg_ila_take_rte_finalize_w;
    // v7 (2026-07-14) — MOVEA.L (SP),SP wild-jump / finalize
    // address-data crossover investigation taps.
    wire [31:0] dbg_ila_exc_held_fault_pc_w;
    wire [31:0] dbg_ila_rob_brtgt_w;
    wire [31:0] dbg_ila_dc_rdata_w;
    wire        dbg_ila_take_finalize_w;
    // v8 (2026-07-14) — stale-ROB-slot / dispatch-clear race taps.
    wire        dbg_ila_rob_brt_w;
    wire [5:0]  dbg_ila_snap_tag_w;
    wire [31:0] dbg_ila_snap_ea_w;
    // Raw (pre-widen) data-master AXI W-channel snapshot — named to
    // match exactly what rtl/soc/fpga_top_debug_vio.vh already expects
    // (raw_daxi_*; same bit-rot class as the dbg_ila_* bundle above).
    wire [31:0] raw_daxi_awaddr;
    wire        raw_daxi_awvalid;
    wire        raw_daxi_awready;
    wire        raw_daxi_wvalid;
    wire        raw_daxi_wready;
    wire        raw_daxi_bvalid;
    wire        raw_daxi_bready;
    wire        raw_daxi_wlast;
    // 2026-08-30 boot-investigation ILA taps, round 7 (docs/
    // BUG_calibration_word_misplaced_0d00.md Part 53's precise closing
    // recommendation). Rounds 1-6 exhausted every genuinely free
    // `dbg_ila_*_w`/`raw_daxi_*` slot this build doesn't otherwise drive
    // WITHOUT touching a wire some other still-live consumer (an
    // OR-reduction bundle feeding another, currently-meaningful probe bit --
    // e.g. `dbg_ila_arch_a7_w`'s illegitimate-step detector, `dbg_ila_
    // commit_bundle_w`'s exc-ring-activity decode) reads as constant-0
    // TODAY for CPU_M68K040 but is NOT dead: reusing it would silently
    // perturb an already-relied-upon probe bit for every round's own
    // analysis. Rather than accept that risk, this round adds TWO BRAND
    // NEW wires + two brand new `debug_ila` IP probes (44/45, see
    // synth/debug_ila.tcl's C_NUM_OF_PROBES bump) instead of reusing an
    // existing one -- clean, zero risk to any prior round's probe
    // semantics, at the one-time cost of a `debug_ila` IP regen (this
    // build's own cached-IP marker in synth/debug_ila.tcl's `gen_debug_ila_
    // ip` handles this: bump the marker version string and it regenerates
    // automatically, same mechanism used for every prior probe-count bump
    // v1..v9).
    wire [31:0] dbg_ila_robgate_state_w;
    wire [31:0] dbg_ila_diag_fault_addr_w;
    // 2026-08-30 boot-investigation ILA taps, round 8 (docs/
    // BUG_calibration_word_misplaced_0d00.md Part 55's precise closing
    // recommendation): I-cache MSHR per-slot control-file state, see
    // M68kCore.scala's dbg040 Area round-8 comment for the exact bit
    // layout. Three BRAND NEW wires feeding three BRAND NEW `debug_ila`
    // probes (50/51/52), same reasoning as round 7's own wire-declaration
    // comment above (no existing free slot can be reused without risking
    // an already-relied-upon probe bit).
    wire [31:0] dbg_ila_ic_mshr_snap_w;
    wire [31:0] dbg_ila_ic_mshr_snap2_w;
    wire [31:0] dbg_ila_ic_mshr_slot1_pa_w;
    // 2026-08-31 boot-investigation ILA taps, round 10 (docs/
    // BUG_calibration_word_misplaced_0d00.md Part 56's precise closing
    // recommendation): per-slot MSHR generation/tag counters, see
    // M68kCore.scala's dbg040 Area round-10 comment for the exact bit
    // layout and decision procedure. Two BRAND NEW wires feeding two BRAND
    // NEW `debug_ila` probes.
    wire [31:0] dbg_ila_ic_mshr_gen_w;
    wire [31:0] dbg_ila_ic_mshr_arsent_gen_w;
`endif

`ifdef CPU_M68K040
`ifdef PERF_DETAIL_ENABLE
    wire [94:0] ipc_trace;
`endif
    M68kSocketTop u_cpu (
`ifdef PERF_DETAIL_ENABLE
        .perf_trace (ipc_trace),
`endif
        .clk(core_clk), .rst(cpu_rst),

        // ── axi_i_* : instruction read master → xbar M2 / l2c fetch port (ifa_*)
        .axi_i_arid   (ifa_arid   ), .axi_i_araddr (ifa_araddr ),
        .axi_i_arlen  (ifa_arlen  ), .axi_i_arsize (ifa_arsize ),
        .axi_i_arburst(ifa_arburst), .axi_i_arvalid(ifa_arvalid),
        .axi_i_arready(ifa_arready),
        .axi_i_rid    (ifa_rid    ), .axi_i_rdata  (ifa_rdata  ),
        .axi_i_rresp  (ifa_rresp  ), .axi_i_rlast  (ifa_rlast  ),
        .axi_i_rvalid (ifa_rvalid ), .axi_i_rready (ifa_rready ),

        // ── axi_d_* : data read/write master → xbar M0 (cpu_d_*) ──────
        .axi_d_awid   (cpu_d_awid   ), .axi_d_awaddr (cpu_d_awaddr ),
        .axi_d_awlen  (cpu_d_awlen  ), .axi_d_awsize (cpu_d_awsize ),
        .axi_d_awburst(cpu_d_awburst), .axi_d_awvalid(cpu_d_awvalid),
        .axi_d_awready(cpu_d_awready),
        .axi_d_wdata  (cpu_d_wdata  ), .axi_d_wstrb  (cpu_d_wstrb  ),
        .axi_d_wlast  (cpu_d_wlast  ), .axi_d_wvalid (cpu_d_wvalid ),
        .axi_d_wready (cpu_d_wready ),
        .axi_d_bid    (cpu_d_bid    ), .axi_d_bresp  (cpu_d_bresp  ),
        .axi_d_bvalid (cpu_d_bvalid ), .axi_d_bready (cpu_d_bready ),
        .axi_d_arid   (cpu_d_arid   ), .axi_d_araddr (cpu_d_araddr ),
        .axi_d_arlen  (cpu_d_arlen  ), .axi_d_arsize (cpu_d_arsize ),
        .axi_d_arburst(cpu_d_arburst), .axi_d_arvalid(cpu_d_arvalid),
        .axi_d_arready(cpu_d_arready),
        .axi_d_rid    (cpu_d_rid    ), .axi_d_rdata  (cpu_d_rdata  ),
        .axi_d_rresp  (cpu_d_rresp  ), .axi_d_rlast  (cpu_d_rlast  ),
        .axi_d_rvalid (cpu_d_rvalid ), .axi_d_rready (cpu_d_rready ),

        // ── dbg_axi_* : debug/control SLAVE ← JTAG-AXI window (dbg_core_*)
        // dbg_core_* is the 32-bit / 20-bit AXI-Lite window at the
        // historical 0x5090_0000 region.  With a JTAG-AXI host it is driven
        // straight from the debug bus (axi_dbg_bus); otherwise from the
        // peripheral-bus dbg_* path.  See this file's header.
        .dbg_axi_awaddr (dbg_core_awaddr ), .dbg_axi_awvalid(dbg_core_awvalid),
        .dbg_axi_awready(dbg_core_awready),
        .dbg_axi_wdata  (dbg_core_wdata  ), .dbg_axi_wstrb  (dbg_core_wstrb  ),
        .dbg_axi_wvalid (dbg_core_wvalid ), .dbg_axi_wready (dbg_core_wready ),
        .dbg_axi_bresp  (dbg_core_bresp  ), .dbg_axi_bvalid (dbg_core_bvalid ),
        .dbg_axi_bready (dbg_core_bready ),
        .dbg_axi_araddr (dbg_core_araddr ), .dbg_axi_arvalid(dbg_core_arvalid),
        .dbg_axi_arready(dbg_core_arready),
        .dbg_axi_rdata  (dbg_core_rdata  ), .dbg_axi_rresp  (dbg_core_rresp  ),
        .dbg_axi_rvalid (dbg_core_rvalid ), .dbg_axi_rready (dbg_core_rready ),

        // ── IRQ seam ──────────────────────────────────────────────────
        .cpu_ipl(cpu_ipl_ext_w),
        .ipl_ack(cpu_ipl_ack_w),

        // ── SoC-fabric control group (CPU → SoC) ──────────────────────
        .cpu_cold_reset_pulse(cpu_cold_reset_pulse),
        .cpu_cold_reset_hold (cpu_cold_reset_hold),
        .cpu_peripheral_reset(cpu_peripheral_reset),
        .cpu_ram_window_lg2  (cpu_ram_window_lg2),
        .cpu_mon_sense       (cpu_mon_sense),
        .init_done_seen      (ddr_cal_done)
`ifdef ILA_ENABLE
        ,
        // ── 2026-08-27 boot-investigation ILA taps (interrupt-recognition-
        // during-tight-loop bug) ────────────────────────────────────────
        // M68kSocketTop always presents these 10 ports (SpinalHDL cannot
        // conditionally omit a port on a Verilog macro); only the
        // CONNECTION is ILA_ENABLE-gated. Reuses 9 already-existing 1-bit
        // `dbg_ila_*_w` slots plus the 32-bit `dbg_ila_rob_pc_w` slot --
        // deliberately NOT their v1-era semantic names (this build has no
        // commit-bundle/CDB/A7-writeback concepts to report), see
        // synth/ila_rob_irq_recognition_capture.tcl for the authoritative
        // mapping table used when decoding a capture:
        //   dbg_ila_rob_pop_w         <- normalIrqGate
        //   dbg_ila_rob_is_last_uop_w <- flushing
        //   dbg_ila_flush_en_w        <- excIdle
        //   dbg_ila_dc_aw_is_evict_w  <- iplActive
        //   dbg_ila_cdb0_en_w         <- branchRedirect
        //   dbg_ila_cdb0_has_dst_w    <- p0.first
        //   dbg_ila_cdb1_en_w         <- preciseDrainBusyIn
        //   dbg_ila_cdb1_has_dst_w    <- inhibitedLoadBusyIn
        //   dbg_ila_cdb2_en_w         <- interruptPending
        //   dbg_ila_rob_pc_w          <- ROB head PC (for correlation)
        .dbg040_normalIrqGate      (dbg_ila_rob_pop_w),
        .dbg040_flushing           (dbg_ila_rob_is_last_uop_w),
        .dbg040_excIdle            (dbg_ila_flush_en_w),
        .dbg040_iplActive          (dbg_ila_dc_aw_is_evict_w),
        .dbg040_branchRedirect     (dbg_ila_cdb0_en_w),
        .dbg040_p0First            (dbg_ila_cdb0_has_dst_w),
        .dbg040_preciseDrainBusyIn (dbg_ila_cdb1_en_w),
        .dbg040_inhibitedLoadBusyIn(dbg_ila_cdb1_has_dst_w),
        .dbg040_interruptPending   (dbg_ila_cdb2_en_w),
        .dbg040_headPc             (dbg_ila_rob_pc_w),
        // ── 2026-08-28 boot-investigation ILA taps (RAS occupancy / BTB+FTB
        // training payload — docs/BUG_calibration_word_misplaced_0d00.md
        // Part 24): 12 more repurposed slots, ALL taken from wires that were
        // previously tied to a hardcoded 0 below (verified pure passthroughs
        // with no other consumer logic -- NOT any of the dbg_ila_cdb*/
        // dbg_ila_a7_writeback_*/dbg_ila_vec_ssp_valid_w/
        // dbg_ila_committed_a7_phys_w family, which still feed the live
        // `ila_prf_write_targets_01` OR-reduction in fpga_top_debug_vio.vh
        // and must stay untouched). See
        // synth/ila_ras_btb_ftb_capture.tcl for the authoritative mapping
        // table used when decoding a capture:
        //   dbg_ila_effective_halt_w    <- rasPredValid
        //   dbg_ila_real_a7_val_w       <- rasPredTarget (RAS top-of-stack)
        //   dbg_ila_rob_phys_dst_w      <- rasCount (occupancy, 0..16)
        //   dbg_ila_take_rte_finalize_w <- btbPredHitComb (query-time BTB hit
        //                                  for the CURRENT fetch PC)
        //   dbg_ila_prf01_w             <- btbPredTargetComb (query-time BTB
        //                                  predicted target)
        //   dbg_ila_supervisor_mode_w   <- btbUpdValid (retire-time BTB+FTB
        //                                  training write, shared bus)
        //   dbg_ila_if_pc_w             <- btbUpdPc (the PC being trained)
        //   dbg_ila_pred_pc_w           <- btbUpdTarget (the target value
        //                                  being written into that entry)
        //   dbg_ila_take_finalize_w     <- ftbRspValid (FTB's own C+1
        //                                  token-matched lookup response)
        //   dbg_ila_rob_brt_w           <- ftbRspHit
        //   dbg_ila_sp_slot_write_en_w  <- ftbRspFramedOk
        //   dbg_ila_exc_held_fault_pc_w <- ftbRspTarget
        .dbg040_rasPredValid       (dbg_ila_effective_halt_w),
        .dbg040_rasPredTarget      (dbg_ila_real_a7_val_w),
        .dbg040_rasCount           (dbg_ila_rob_phys_dst_w),
        .dbg040_btbPredHitComb     (dbg_ila_take_rte_finalize_w),
        .dbg040_btbPredTargetComb (dbg_ila_prf01_w),
        .dbg040_btbUpdValid        (dbg_ila_supervisor_mode_w),
        .dbg040_btbUpdPc           (dbg_ila_if_pc_w),
        .dbg040_btbUpdTarget       (dbg_ila_pred_pc_w),
        .dbg040_ftbRspValid        (dbg_ila_take_finalize_w),
        .dbg040_ftbRspHit          (dbg_ila_rob_brt_w),
        .dbg040_ftbRspFramedOk     (dbg_ila_sp_slot_write_en_w),
        .dbg040_ftbRspTarget       (dbg_ila_exc_held_fault_pc_w),
        // ── 2026-08-28 boot-investigation ILA taps, round 3 (FTQ lookup-
        // command window PC / FTQ head-entry fields — docs/
        // BUG_calibration_word_misplaced_0d00.md Part 24's own
        // "Recommended next steps" #1/#2): 6 more repurposed slots, ALL
        // taken from wires that were previously tied to a hardcoded 0
        // below (verified pure passthroughs with a single MARK_DEBUG
        // probe consumer each, and specifically NOT `dbg_ila_arch_a7_w`
        // (a real registered comparator input, see `ila_a7_step`/
        // `ila_a7_illegitimate_step`), NOT `dbg_ila_commit_bundle_w` (feeds
        // `ila_mb_empty`/`ila_drain_active`, part of Part 25's own
        // `ila_rob_pop_bundle` flush-signal decode), and NOT any of the
        // dbg_ila_cdb*/a7_writeback_*/vec_ssp_valid_w/committed_a7_phys_w
        // family (still feeds `ila_prf_write_targets_01`) -- same
        // exclusion list Part 24 already established. `raw_daxi_awaddr`
        // IS reused here (unlike Part 24's choice to leave it alone): its
        // only two consumers for CPU_M68K040 are `ila_axi_w_snap[39:8]`
        // and `ila_wb_addr[32:1]`, both currently constant zero upper bits
        // (their live bits -- `dbg_ila_dc_aw_is_evict_w` / `raw_daxi_awvalid
        // & dbg_ila_dc_aw_is_evict_w` -- are untouched by this reuse, since
        // `raw_daxi_awvalid` itself stays tied to 0 below). See
        // synth/ila_ftq_probes_capture.tcl for the authoritative mapping
        // table used when decoding a capture:
        //   dbg_ila_rob_brtgt_w  <- ftbCmdWindowPc (FTB lookup command's
        //                           live query address)
        //   raw_daxi_awaddr      <- decodePc (frontend's own live decode PC,
        //                           ungated unlike dbg_ila_rob_pc_w)
        //   dbg_ila_dc_rdata_w   <- ftqHeadBrPc (FTQ head entry's trained PC)
        //   dbg_ila_snap_ea_w    <- ftqHeadTarget (FTQ head entry's target)
        //   dbg_ila_rob_arch_dst_w[4:1] <- ftqHeadBrLen
        //   dbg_ila_rob_arch_dst_w[0]   <- ftqConfirm
        //   dbg_ila_rob_phys_old_w[6:1] <- ftqCount
        //   dbg_ila_rob_phys_old_w[0]   <- ftbCmdValid
        .dbg040_ftbCmdWindowPc     (dbg_ila_rob_brtgt_w),
        .dbg040_decodePc           (raw_daxi_awaddr),
        .dbg040_ftqHeadBrPc        (dbg_ila_dc_rdata_w),
        .dbg040_ftqHeadTarget      (dbg_ila_snap_ea_w),
        .dbg040_ftqHeadBrLen       (dbg_ila_rob_arch_dst_w[4:1]),
        .dbg040_ftqConfirm         (dbg_ila_rob_arch_dst_w[0]),
        .dbg040_ftqCount           (dbg_ila_rob_phys_old_w[6:1]),
        .dbg040_ftbCmdValid        (dbg_ila_rob_phys_old_w[0]),
        // ── 2026-09-05 p141: the walker-stall capture ────────────────────
        // Rounds 4-10 used to connect ~22 `.dbg040_*` ports here that exist on
        // NO cpu040 ref (beu*, robGateState, sqState, excSqState,
        // diagFaultAddr, icMshr*). That is why ILA_ENABLE + CPU_M68K040 has not
        // built since round 4 landed: nothing caught it because no Makefile lint
        // config passes ILA_ENABLE and every board bitstream since was built
        // with the ILA off. Those connections are removed and their wires tied
        // off below -- an undriven probe bit is a HARD Chipscope 16-213 error at
        // debug-core link, not a warning.
        //
        // Five of the freed 32-bit wires are repurposed for this capture. No
        // probe is added, no width changes, so C_NUM_OF_PROBES stays 57 and the
        // cached debug_ila DCP does NOT need regenerating -- deliberately, to
        // keep this build as close to p133/p140's placement as an ILA allows.
        //
        //   probe20  dbg_ila_cdb0_data_w        <- stallDc     (17 quiesce terms)
        //   probe21  dbg_ila_cdb1_data_w        <- stallGrant  (ownership/grants)
        //   probe22  dbg_ila_a7_writeback_val_w <- stallExc    (ExcUnit FSM)
        //   probe45  dbg_ila_diag_fault_addr_w  <- stallWalk   (both walkers)
        //   probe50  dbg_ila_ic_mshr_snap_w     <- macroCountLo (THE TRIGGER)
        //
        // Bit layouts are documented once, in cpu040's
        // docs/superpowers/specs/2026-09-05-p141-measure-the-walker-stall-on-silicon.md,
        // and are IDENTICAL to the live CSRs at 0x5090101C..0x50901028 -- the
        // same decoder reads both. Mind the polarity trap recorded there: the
        // dcIdleForMaint terms are NOT all the same polarity, and the four AXI
        // *Done flags block when CLEAR.
        .dbg040_stallDc                (dbg_ila_cdb0_data_w),
        .dbg040_stallGrant             (dbg_ila_cdb1_data_w),
        .dbg040_stallExc               (dbg_ila_a7_writeback_val_w),
        .dbg040_stallWalk              (dbg_ila_diag_fault_addr_w),
        .dbg040_macroCountLo           (dbg_ila_ic_mshr_snap_w)
`endif
    );
`ifdef ILA_ENABLE
    // ── Tie off the 35 v1-only dbg_ila_*_w/raw_daxi_* wires this build
    // does not drive ────────────────────────────────────────────────────
    // Vivado's debug-core linker (Chipscope 16-213) hard-errors on ANY
    // unconnected probe bit, not just a lint warning -- confirmed
    // empirically (opt_design failed on `u_dbg_ila/probe5` having 2
    // unconnected channels: `ila_mb_empty`/`ila_drain_active`, both
    // derived from `dbg_ila_commit_bundle_w`, which nothing drove for
    // CPU_M68K040). "Declared but unconnected, harmless" was true for
    // synthesis-level pruning but NOT true for debug-core linking --
    // every wire below must resolve to a defined value, even though none
    // of these v1-specific concepts (commit bundle, CDB phys/data,
    // A7-writeback, RTE-finalize investigation taps, etc.) exist for
    // cpu040. Tied to 0, not X -- an inert but well-defined dashboard
    // value, matching the intent of the original "harmless" comment.
    // 2026-08-28: 12 of these wires are now driven by the new RAS/BTB/FTB
    // dbg040_* ports above (rob_phys_dst, real_a7_val, take_rte_finalize,
    // prf01, supervisor_mode, if_pc, pred_pc, take_finalize, rob_brt,
    // sp_slot_write_en, exc_held_fault_pc, effective_halt) -- see the
    // mapping comment at the connection site. 2026-08-28 round 3: 6 MORE
    // wires (rob_arch_dst, rob_phys_old, rob_brtgt, dc_rdata, snap_ea,
    // raw_daxi_awaddr) are now driven by the new FTQ/FTB-command dbg040_*
    // ports -- see that same connection-site mapping comment. 2026-08-28
    // round 4: 3 MORE wires (snap_prf_idx, exc_held_a7_phys, snap_tag) are
    // now driven (bit-packed, not whole-wire this time) by the new
    // BranchEuPlugin-S1 / RobPlugin-retire-gating dbg040_* ports -- see
    // that same connection-site mapping comment. 2026-08-28 round 5: 1 MORE
    // wire (cdb0_data) is now driven (whole-wire, full 32b) by the new
    // direct-u1.pc dbg040_beuU1Pc port -- see that same connection-site
    // mapping comment. 2026-08-30 round 6: 2 MORE wires (cdb1_data,
    // a7_writeback_val) are now driven (whole-wire, full 32b each) by the new
    // StoreQueue/ExceptionUnit dbg040_sqState/dbg040_excSqState ports -- see
    // that same connection-site mapping comment. The rest stay tied to 0,
    // unchanged.
    assign dbg_ila_commit_bundle_w     = 26'd0;
    assign dbg_ila_arch_a7_w           = 32'd0;
    assign dbg_ila_cdb0_phys_w         = 7'd0;
    assign dbg_ila_cdb1_phys_w         = 7'd0;
    assign dbg_ila_cdb2_has_dst_w      = 1'd0;
    assign dbg_ila_cdb2_phys_w         = 7'd0;
    assign dbg_ila_cdb_alu_hi_en_w     = 1'd0;
    assign dbg_ila_cdb_alu_hi_phys_w   = 7'd0;
    assign dbg_ila_a7_writeback_en_w   = 1'd0;
    assign dbg_ila_a7_writeback_phys_w = 7'd0;
    assign dbg_ila_vec_ssp_valid_w     = 1'd0;
    assign dbg_ila_sp_slot_write_sel_w = 2'd0;
    assign dbg_ila_committed_a7_phys_w = 7'd0;
    assign raw_daxi_awvalid            = 1'd0;
    assign raw_daxi_awready            = 1'd0;
    assign raw_daxi_wvalid             = 1'd0;
    assign raw_daxi_wready             = 1'd0;
    assign raw_daxi_bvalid             = 1'd0;
    assign raw_daxi_bready             = 1'd0;
    assign raw_daxi_wlast              = 1'd0;
    // 2026-09-05 p141: 8 MORE wires return to the tie-off list, because the
    // rounds 4-10 ports that drove them do not exist on cpu040. Five OTHER
    // wires (cdb0_data, cdb1_data, a7_writeback_val, diag_fault_addr,
    // ic_mshr_snap) are NOT here -- they are now driven by the p141
    // stall-capture ports at the connection site above.
    assign dbg_ila_snap_prf_idx_w         = 7'd0;
    assign dbg_ila_exc_held_a7_phys_w     = 7'd0;
    assign dbg_ila_snap_tag_w             = 6'd0;
    assign dbg_ila_robgate_state_w        = 32'd0;
    assign dbg_ila_ic_mshr_snap2_w        = 32'd0;
    assign dbg_ila_ic_mshr_slot1_pa_w     = 32'd0;
    assign dbg_ila_ic_mshr_gen_w          = 32'd0;
    assign dbg_ila_ic_mshr_arsent_gen_w   = 32'd0;
`endif
`else
    // (`dbg_ila_*_w`/`raw_daxi_*` wire declarations now live BEFORE the
    // CPU_M68K040 split above, shared by both branches -- see the
    // 2026-08-27 boot-investigation comment there. Declaring them again
    // here would be a Verilog duplicate-declaration error.)
`ifdef CPU_M68K
    m68k_axi_wrapper #(
`else
    cpu_stub #(
`endif
        .RESET_PC           (32'h4000_002A),
        .FETCH_RESET_VECTORS(1'b1),
        // Thread the top-level build stamp to the CPU-side debug_ctrl
        // (OFF_BUILD_ID read 0 over JTAG without this; m68k_axi_wrapper
        // already forwards BUILD_ID → debug_ctrl internally, and cpu_stub
        // accepts-and-ignores it for socket parity).
        .BUILD_ID           (BUILD_ID)
    ) u_cpu (
        .clk(core_clk), .rst(cpu_rst),

        // ── axi_i_* : instruction read master → xbar M2 (ifa_*) ───────
        .axi_i_arid   (ifa_arid   ), .axi_i_araddr (ifa_araddr ),
        .axi_i_arlen  (ifa_arlen  ), .axi_i_arsize (ifa_arsize ),
        .axi_i_arburst(ifa_arburst), .axi_i_arvalid(ifa_arvalid),
        .axi_i_arready(ifa_arready),
        .axi_i_rid    (ifa_rid    ), .axi_i_rdata  (ifa_rdata  ),
        .axi_i_rresp  (ifa_rresp  ), .axi_i_rlast  (ifa_rlast  ),
        .axi_i_rvalid (ifa_rvalid ), .axi_i_rready (ifa_rready ),

        // ── axi_d_* : data read/write master → xbar M0 (cpu_d_*) ──────
        .axi_d_awid   (cpu_d_awid   ), .axi_d_awaddr (cpu_d_awaddr ),
        .axi_d_awlen  (cpu_d_awlen  ), .axi_d_awsize (cpu_d_awsize ),
        .axi_d_awburst(cpu_d_awburst), .axi_d_awvalid(cpu_d_awvalid),
        .axi_d_awready(cpu_d_awready),
        .axi_d_wdata  (cpu_d_wdata  ), .axi_d_wstrb  (cpu_d_wstrb  ),
        .axi_d_wlast  (cpu_d_wlast  ), .axi_d_wvalid (cpu_d_wvalid ),
        .axi_d_wready (cpu_d_wready ),
        .axi_d_bid    (cpu_d_bid    ), .axi_d_bresp  (cpu_d_bresp  ),
        .axi_d_bvalid (cpu_d_bvalid ), .axi_d_bready (cpu_d_bready ),
        .axi_d_arid   (cpu_d_arid   ), .axi_d_araddr (cpu_d_araddr ),
        .axi_d_arlen  (cpu_d_arlen  ), .axi_d_arsize (cpu_d_arsize ),
        .axi_d_arburst(cpu_d_arburst), .axi_d_arvalid(cpu_d_arvalid),
        .axi_d_arready(cpu_d_arready),
        .axi_d_rid    (cpu_d_rid    ), .axi_d_rdata  (cpu_d_rdata  ),
        .axi_d_rresp  (cpu_d_rresp  ), .axi_d_rlast  (cpu_d_rlast  ),
        .axi_d_rvalid (cpu_d_rvalid ), .axi_d_rready (cpu_d_rready ),

        // ── dbg_axi_* : debug/control SLAVE ← JTAG-AXI window (dbg_core_*)
        // dbg_core_* is the 32-bit / 20-bit AXI-Lite window at the
        // historical 0x5090_0000 region.  With a JTAG-AXI host it is driven
        // straight from the debug bus (axi_dbg_bus); otherwise from the
        // peripheral-bus dbg_* path.  See this file's header.
        .dbg_axi_awaddr (dbg_core_awaddr ), .dbg_axi_awvalid(dbg_core_awvalid),
        .dbg_axi_awready(dbg_core_awready),
        .dbg_axi_wdata  (dbg_core_wdata  ), .dbg_axi_wstrb  (dbg_core_wstrb  ),
        .dbg_axi_wvalid (dbg_core_wvalid ), .dbg_axi_wready (dbg_core_wready ),
        .dbg_axi_bresp  (dbg_core_bresp  ), .dbg_axi_bvalid (dbg_core_bvalid ),
        .dbg_axi_bready (dbg_core_bready ),
        .dbg_axi_araddr (dbg_core_araddr ), .dbg_axi_arvalid(dbg_core_arvalid),
        .dbg_axi_arready(dbg_core_arready),
        .dbg_axi_rdata  (dbg_core_rdata  ), .dbg_axi_rresp  (dbg_core_rresp  ),
        .dbg_axi_rvalid (dbg_core_rvalid ), .dbg_axi_rready (dbg_core_rready ),

        // ── IRQ seam ──────────────────────────────────────────────────
        .cpu_ipl(cpu_ipl_ext_w),
        .ipl_ack(cpu_ipl_ack_w),

        // ── SoC-fabric control group (CPU → SoC) ──────────────────────
        .cpu_cold_reset_pulse(cpu_cold_reset_pulse),
        .cpu_cold_reset_hold (cpu_cold_reset_hold),
        .cpu_peripheral_reset(cpu_peripheral_reset),
        .cpu_ram_window_lg2  (cpu_ram_window_lg2),
        .cpu_mon_sense       (cpu_mon_sense),
        .init_done_seen      (ddr_cal_done)
`ifdef ILA_ENABLE
        ,
        // ── ILA-only debug-export group (see wire declarations above) ──
        .dbg_ila_commit_bundle_w    (dbg_ila_commit_bundle_w),
        .dbg_ila_rob_arch_dst_w     (dbg_ila_rob_arch_dst_w),
        .dbg_ila_rob_phys_dst_w     (dbg_ila_rob_phys_dst_w),
        .dbg_ila_rob_phys_old_w     (dbg_ila_rob_phys_old_w),
        .dbg_ila_rob_pop_w          (dbg_ila_rob_pop_w),
        .dbg_ila_rob_is_last_uop_w  (dbg_ila_rob_is_last_uop_w),
        .dbg_ila_flush_en_w         (dbg_ila_flush_en_w),
        .dbg_ila_rob_pc_w           (dbg_ila_rob_pc_w),
        .dbg_ila_arch_a7_w          (dbg_ila_arch_a7_w),
        .dbg_ila_dc_aw_is_evict_w   (dbg_ila_dc_aw_is_evict_w),
        .dbg_ila_cdb0_en_w          (dbg_ila_cdb0_en_w),
        .dbg_ila_cdb0_has_dst_w     (dbg_ila_cdb0_has_dst_w),
        .dbg_ila_cdb0_phys_w        (dbg_ila_cdb0_phys_w),
        .dbg_ila_cdb0_data_w        (dbg_ila_cdb0_data_w),
        .dbg_ila_cdb1_en_w          (dbg_ila_cdb1_en_w),
        .dbg_ila_cdb1_has_dst_w     (dbg_ila_cdb1_has_dst_w),
        .dbg_ila_cdb1_phys_w        (dbg_ila_cdb1_phys_w),
        .dbg_ila_cdb1_data_w        (dbg_ila_cdb1_data_w),
        .dbg_ila_cdb2_en_w          (dbg_ila_cdb2_en_w),
        .dbg_ila_cdb2_has_dst_w     (dbg_ila_cdb2_has_dst_w),
        .dbg_ila_cdb2_phys_w        (dbg_ila_cdb2_phys_w),
        .dbg_ila_cdb_alu_hi_en_w    (dbg_ila_cdb_alu_hi_en_w),
        .dbg_ila_cdb_alu_hi_phys_w  (dbg_ila_cdb_alu_hi_phys_w),
        .dbg_ila_a7_writeback_en_w  (dbg_ila_a7_writeback_en_w),
        .dbg_ila_a7_writeback_phys_w(dbg_ila_a7_writeback_phys_w),
        .dbg_ila_a7_writeback_val_w (dbg_ila_a7_writeback_val_w),
        .dbg_ila_vec_ssp_valid_w    (dbg_ila_vec_ssp_valid_w),
        .dbg_ila_sp_slot_write_en_w (dbg_ila_sp_slot_write_en_w),
        .dbg_ila_sp_slot_write_sel_w(dbg_ila_sp_slot_write_sel_w),
        .dbg_ila_committed_a7_phys_w(dbg_ila_committed_a7_phys_w),
        .dbg_ila_real_a7_val_w      (dbg_ila_real_a7_val_w),
        .dbg_ila_prf01_w            (dbg_ila_prf01_w),
        .dbg_ila_snap_prf_idx_w     (dbg_ila_snap_prf_idx_w),
        .dbg_ila_effective_halt_w   (dbg_ila_effective_halt_w),
        .dbg_ila_if_pc_w            (dbg_ila_if_pc_w),
        .dbg_ila_pred_pc_w          (dbg_ila_pred_pc_w),
        .dbg_ila_supervisor_mode_w  (dbg_ila_supervisor_mode_w),
        .dbg_ila_exc_held_a7_phys_w (dbg_ila_exc_held_a7_phys_w),
        .dbg_ila_take_rte_finalize_w(dbg_ila_take_rte_finalize_w),
        .dbg_ila_exc_held_fault_pc_w(dbg_ila_exc_held_fault_pc_w),
        .dbg_ila_rob_brtgt_w        (dbg_ila_rob_brtgt_w),
        .dbg_ila_dc_rdata_w         (dbg_ila_dc_rdata_w),
        .dbg_ila_take_finalize_w    (dbg_ila_take_finalize_w),
        .dbg_ila_rob_brt_w          (dbg_ila_rob_brt_w),
        .dbg_ila_snap_tag_w         (dbg_ila_snap_tag_w),
        .dbg_ila_snap_ea_w          (dbg_ila_snap_ea_w),
        .raw_daxi_awaddr            (raw_daxi_awaddr),
        .raw_daxi_awvalid           (raw_daxi_awvalid),
        .raw_daxi_awready           (raw_daxi_awready),
        .raw_daxi_wvalid            (raw_daxi_wvalid),
        .raw_daxi_wready            (raw_daxi_wready),
        .raw_daxi_bvalid            (raw_daxi_bvalid),
        .raw_daxi_bready            (raw_daxi_bready),
        .raw_daxi_wlast             (raw_daxi_wlast)
`endif
    );
`endif // CPU_M68K040
