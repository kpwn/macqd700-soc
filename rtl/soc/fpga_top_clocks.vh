// rtl/fpga_top_clocks.vh — included from rtl/fpga_top.v
//
// Clock tree, IBUFDS/BUFG_GT, clk_rst, DDR cal sync, boot_rom/cpu_rst gates, phi2_tick generator.
//
// Generated mechanically from rtl/fpga_top.v during the fpga_top split
// (agent/p3-fpga-top-split). No functional change: this file contains
// the original lines verbatim, inside the fpga_top module scope via
// `include. Do not add module/endmodule wrappers or nettype directives.


    // ═══════════════════════════════════════════════════════════════════
    // Input buffering + clock tree
    // ═══════════════════════════════════════════════════════════════════
    wire sys_clk;
    wire video_ref_clk;
    wire pb_clk_src;
    // "the core clock is real".  Constant 1 on every topology whose core
    // clock is a buffer (SIM_MODEL, and the BUFG_GT divider tree); the
    // MMCM's LOCKED on the CORE_MMCM topology, where it gates the whole
    // SoC out of reset -- see the u_core_mmcm block.
    wire core_mmcm_locked;

`ifdef SIM_MODEL
    IBUFDS u_sys_clk_ibufds (
        .I  (sys_clk_p),
        .IB (sys_clk_n),
        .O  (sys_clk)
    );
    assign video_ref_clk = sys_clk;
    assign pb_clk_src = sys_clk;
    assign core_mmcm_locked = 1'b1;
`else
    // AB7/AB6 are MGTREFCLK pins.  Match the known-good pcie_test
    // util_ds_buf input path, then enter fabric through BUFG_GT.  Routing
    // ODIV2 directly to the video MMCM or BUFGCE_DIV is illegal on KU5P.
    wire fabric_clk_gt;
    wire fabric_clk_odiv2;
    localparam [2:0] FABRIC_GT_CORE_DIV =
        (CORE_CLK_DIVIDE == 1) ? 3'd0 :
        (CORE_CLK_DIVIDE == 2) ? 3'd1 :
        (CORE_CLK_DIVIDE == 4) ? 3'd3 :
                                 3'd7;

    IBUFDS_GTE4 #(
        .REFCLK_EN_TX_PATH (1'b0),
        .REFCLK_HROW_CK_SEL(2'b00),
        .REFCLK_ICNTL_RX   (2'b00)
    ) u_fabric_clk_ibufds_gte4 (
        .I     (fabric_clk_p),
        .IB    (fabric_clk_n),
        .CEB   (1'b0),
        .O     (fabric_clk_gt),
        .ODIV2 (fabric_clk_odiv2)
    );

    // ─── STARTUPE3.EOS — fresh-every-configuration divider-clear pulse ──
    // Board bring-up 2026-07-15: cpu_resetn/btn[3] were both continuously
    // released across a JTAG-triggered in-place bitstream reload (no edge
    // on either pin, since neither was pressed), so fabric_gt_clr below
    // never asserted and the BUFG_GT dividers simply inherited whatever
    // phase they were already in from the PREVIOUS bitstream — reproducing
    // the "MMCM never relocks / core_clk dead" symptom on a supposedly
    // known-good bitstream, and surviving even a full mains power-cycle of
    // the host (the FIRST config-from-flash at power-on may have come up
    // clean, but the subsequent JTAG override re-opened the same gap).
    // EOS is driven purely by the FPGA's own dedicated configuration
    // engine — it goes low at the start of EVERY configuration (JTAG,
    // SelectMAP, or flash-boot) and back high only once that specific
    // configuration's startup sequence completes, with no dependency on
    // any external clock or button.  OR'ing ~eos into fabric_gt_clr gives
    // the divider a genuine fresh clear-then-release on every single
    // bitstream load, closing the JTAG-reload gap the button-edge-only
    // scheme left open.
    wire eos;
    STARTUPE3 #(
        .PROG_USR      ("FALSE"),
        .SIM_CCLK_FREQ (0.0)
    ) u_startupe3 (
        .CFGCLK    (),
        .CFGMCLK   (),
        .DI        (),
        .EOS       (eos),
        .PREQ      (),
        .DO        (4'b0000),
        .DTS       (4'b1111),
        .FCSBO     (1'b1),
        .FCSBTS    (1'b1),
        .GSR       (1'b0),
        .GTS       (1'b0),
        .KEYCLEARB (1'b1),
        .PACK      (1'b0),
        .USRCCLKO  (1'b0),
        .USRCCLKTS (1'b1),
        .USRDONEO  (1'b1),
        .USRDONETS (1'b1)
    );

    // ─── BUFG_GT divider-state CLR ─────────────────────────────────────
    // Symptom this fixes: after a board reset (btn[3] or cpu_resetn drop)
    // the GT-derived clock chain comes back in a bad phase.  The video
    // MMCM never relocks (led[3] stays off) and `core_clk` is dead,
    // which wedges debug_vio + debug_jtag_axi (both clocked by core_clk).
    // Cold-power-up is fine because the BUFG_GT internal divider state
    // machine starts from a known-zero phase; a partial board reset did
    // NOT reset that state machine because we used to tie .CLR(1'b0).
    //
    // Drive .CLR from raw `~cpu_resetn | ~btn[3] | ~eos`.  Per UG572
    // §"BUFG_GT" the CLR pin is async-assert tolerant; deassert-sync to
    // the input clock is *recommended* but not required — without a
    // synchroniser the divider may produce a one-cycle phase glitch on
    // CLR release, which downstream MMCMs absorb via their own re-lock.
    // The ~eos term is what makes this reliable across a JTAG reload with
    // no button edge (see STARTUPE3 comment above) — cpu_resetn/btn[3]
    // alone only cover a physical board-reset event.
    //
    // We avoid an xpm_cdc_async_rst here because the only available
    // dest_clk would be either:
    //   (a) fabric_clk_gt (IBUFDS_GTE4.O) — DRC-illegal: that pin can
    //       only drive GT* primitive GTREFCLK inputs, not fabric flops.
    //   (b) a BUFG_GT output (sys_clk / video_ref_clk / pb_clk_src) —
    //       the very clocks we're resetting, so dest_clk would die
    //       while CLR is asserted and the deassert-sync would deadlock.
    //
    // The board-reset edge / EOS pulse feeds these CLRs unsynced;
    // mechanical bounce is fine (each bounce just retriggers the clear).
    // Deliberately NOT including jtag_debug_full_reset_eff: pulsing GT
    // CLR during a JTAG-driven debug reset would kill dbg_hub's own
    // clock mid-transaction.
    // btn[3] is active-low on this board (pressed = GND, released = pulled
    // HIGH); ~btn[3] is the active-high "pressed" form.
    // PCIe/XDMA note: the XDMA IP is generated with ext_sys_clk_bufg=true
    // so it does NOT hang its own BUFG_GT (+BUFG_GT_SYNC) on
    // fabric_clk_odiv2 — that combination is un-mergeable with the three
    // fabric BUFG_GTs below (their shared dynamic CLR vs the IP's
    // internal sync) and trips DRC REQP-1963/BFGTL-1 at place_design.
    // Instead the IP's sys_clk input is fed from video_ref_clk (the
    // DIV=0 100 MHz output below) in fpga_top_debug_host.vh, keeping
    // ODIV2's load set identical to the proven non-PCIe topology.
    wire fabric_gt_clr = ~cpu_resetn | ~btn[3] | ~eos;

    BUFG_GT u_fabric_ref_bufg_gt (
        .I      (fabric_clk_odiv2),
        .CE     (1'b1),
        .CEMASK (1'b0),
        .CLR    (fabric_gt_clr),
        .CLRMASK(1'b0),
        .DIV    (3'd0),
        .O      (video_ref_clk)
    );

`ifdef CORE_MMCM
    // ─── CORE_MMCM — core clock ABOVE the board oscillator ──────────────
    //
    // The board reference is a fixed 100 MHz MGTREFCLK and a BUFG_GT can
    // only DIVIDE, so nothing in the tree above can produce a core clock
    // faster than 100 MHz.  This branch adds an MMCM to multiply it.
    //
    // ADDITIVE BY CONSTRUCTION.  It hangs off `video_ref_clk` -- the
    // existing DIV=0 100 MHz BUFG_GT output -- and changes NOTHING else:
    // the video MMCM (fpga_top_video.vh, parameterised CLKIN1_PERIOD=
    // 10.000, CLKFBOUT_MULT_F=37.125) keeps its own 100 MHz input, and the
    // MIG clocks descend from sys_clk_p and never see this.  Re-declaring
    // the board input as 5 ns instead would have doubled every derived
    // video clock and contaminated both trees; that is why this is a new
    // MMCM and not an XDC edit.
    //
    // ONE VCO, BOTH CLOCKS.  VCO = 100 MHz x 12 = 1200 MHz, which is
    // inside the KU5P -2 MMCM range (800-1600 MHz) and leaves the PFD at
    // 100 MHz.  Every supported core frequency is 1200/N, and pb is
    // ALWAYS 1200/24 = exactly 50.000 MHz:
    //
    //     CORE_CLK_HZ    CLKOUT0_DIVIDE_F   core     pb      ratio
    //     200_000_000            6.0        200.000  50.000   4:1
    //     150_000_000            8.0        150.000  50.000   3:1
    //     120_000_000           10.0        120.000  50.000  12:5
    //     100_000_000           12.0        100.000  50.000   2:1
    //
    // pb_clk MUST stay at exactly 50 MHz and it is not a preference: every
    // Mac peripheral timebase in this SoC is derived from PB_CLK_HZ, not
    // from the core clock -- the VIA phi2 accumulator below, the SCC BRG
    // divider, ADB, the audio PWM oversampling ratio and the peripheral
    // bus timeouts.  A pb clock that is 50-point-something would be a
    // silent, hard-to-attribute timing error in all of them.  Taking both
    // clocks off the SAME VCO also makes them phase-related by
    // construction, which is what keeps the core<->pb crossings a
    // well-defined synchronous transfer rather than a CDC.
    //
    // RESET.  The MMCM's RST takes the same `fabric_gt_clr` term the
    // BUFG_GT dividers take (board reset / button / ~EOS), so a JTAG
    // reload still gets a genuine clear-and-relock -- that is the
    // STARTUPE3 fix above, preserved.  LOCKED is then folded into
    // `platform_resetn` (see core_mmcm_locked below) so the whole SoC is
    // held in ASYNC reset until the clock it runs on is real.  It has to
    // be an async assert into clk_rst's rst_in and not a term in
    // init_done: before lock there is no core clock, so nothing clocked by
    // it can sequence.
    localparam real CORE_MMCM_CLKOUT0_DIV =
        (CORE_CLK_HZ == 200000000) ? 6.0  :
        (CORE_CLK_HZ == 150000000) ? 8.0  :
        (CORE_CLK_HZ == 120000000) ? 10.0 :
                                     12.0;

    wire core_mmcm_clkout0;
    wire core_mmcm_clkout1;
    wire core_mmcm_fb;

    MMCME4_BASE #(
        .BANDWIDTH          ("OPTIMIZED"),
        .CLKIN1_PERIOD      (10.000),      // video_ref_clk, 100 MHz
        .DIVCLK_DIVIDE      (1),
        .CLKFBOUT_MULT_F    (12.000),      // VCO = 1200 MHz
        .CLKOUT0_DIVIDE_F   (CORE_MMCM_CLKOUT0_DIV),
        .CLKOUT1_DIVIDE     (24),          // pb: 1200/24 = 50.000 MHz exactly
        .STARTUP_WAIT       ("FALSE")
    ) u_core_mmcm (
        .CLKIN1   (video_ref_clk),
        .CLKFBIN  (core_mmcm_fb),
        .CLKFBOUT (core_mmcm_fb),
        .CLKFBOUTB(),
        .CLKOUT0  (core_mmcm_clkout0),
        .CLKOUT0B (),
        .CLKOUT1  (core_mmcm_clkout1),
        .CLKOUT1B (),
        .CLKOUT2  (), .CLKOUT2B (),
        .CLKOUT3  (), .CLKOUT3B (),
        .CLKOUT4  (), .CLKOUT5 (), .CLKOUT6 (),
        .LOCKED   (core_mmcm_locked),
        .PWRDWN   (1'b0),
        .RST      (fabric_gt_clr)
    );

    BUFG u_core_mmcm_core_bufg (.I(core_mmcm_clkout0), .O(sys_clk));
    BUFG u_core_mmcm_pb_bufg   (.I(core_mmcm_clkout1), .O(pb_clk_src));
`else
    BUFG_GT u_fabric_core_bufg_gt (
        .I      (fabric_clk_odiv2),
        .CE     (1'b1),
        .CEMASK (1'b0),
        .CLR    (fabric_gt_clr),
        .CLRMASK(1'b0),
        .DIV    (FABRIC_GT_CORE_DIV),
        .O      (sys_clk)
    );

    BUFG_GT u_fabric_pb_bufg_gt (
        .I      (fabric_clk_odiv2),
        .CE     (1'b1),
        .CEMASK (1'b0),
        .CLR    (fabric_gt_clr),
        .CLRMASK(1'b0),
        .DIV    (3'd1),
        .O      (pb_clk_src)
    );

    // No MMCM on this topology: the core clock is a BUFG_GT divider
    // output and is usable as soon as the divider leaves CLR.
    assign core_mmcm_locked = 1'b1;
`endif
`endif

    wire core_clk;
    wire core_rst;
    wire soc_full_rst;
    wire pb_clk;
    wire pb_rst;

    // Banked broadcast mirror of core_rst / soc_full_rst from u_clk_rst.
    // The bank ports are the per-region wire-bus convenience surface:
    // every bit is driven from u_clk_rst's BUFG-buffered broadcast net
    // (xpm_cdc_async_rst + explicit BUFG, see clk_rst.v for the
    // distribution rationale).  Consumers pick exactly one bit per
    // region purely for source-compatibility — physical fanout is
    // owned by the global clock network downstream of the BUFG, not
    // by per-bank replication.  Sim-neutral: every bit holds the
    // identical Q value as the single-wire `core_rst` / `soc_full_rst`.
    //
    // Bit allocation (driven by consumer fanout in
    // vivado_main_1d2af15/run.log):
    //   soc_full_rst_bank[0] — fpga_top_video.vh: VRAM read FIFO + scanout
    //                          + DAFB CLUT/FB-base reset path
    //   soc_full_rst_bank[1] — fpga_top_peripherals.vh: peripheral_bus
    //                          AXI slave + mac-stub PSC reset
    //   soc_full_rst_bank[2] — fpga_top_peripherals.vh: DAFB shim register
    //                          file + IRQ resync
    //   soc_full_rst_bank[3] — fpga_top_peripherals.vh: irq_agg + NMI
    //                          edge sync
    //   soc_full_rst_bank[4] — fpga_top_xbar.vh: AXI xbar instances + DDR
    //                          arbiter reset
    //   soc_full_rst_bank[5] — fpga_top_dma.vh: DMA controller +
    //                          fpga_top_sd.vh sd_provision/sd_ctrl
    //   soc_full_rst_bank[6] — fpga_top_boot_master.vh: boot-master
    //                          narrow→wide bridge + boot_fsm gate path
    //   soc_full_rst_bank[7] — local clocks.vh comb gates
    //                          (cpu_rst_settle, boot_fsm_rst, pb_rst sync,
    //                          cpu_rst final OR).
    wire [7:0] core_rst_bank;
    wire [7:0] soc_full_rst_bank;

    // ═══════════════════════════════════════════════════════════════════
    // Reset sequencer
    // ═══════════════════════════════════════════════════════════════════
    wire ddr_cal_done;
    // MIG calibration status is produced in the MIG UI clock domain and
    // can assert/deassert asynchronously relative to sys_clk.  Synchronize
    // before feeding clk_rst's init_done gate so reset release is fully
    // deterministic in the core clock domain.
    (* ASYNC_REG = "TRUE" *) reg ddr_cal_done_meta;
    (* ASYNC_REG = "TRUE" *) reg ddr_cal_done_sync;

    // ─── Debounce + sync the raw board reset inputs ─────────────────────
    //
    // Why this exists: cpu_resetn (T19) and btn[3] are raw mechanical
    // pushbuttons.  Without debouncing, a bouncy press creates multiple
    // reset transitions over a few ms.  Each transition reaches every
    // downstream async-clear domain (5+ of them) independently, so the
    // domains can desynchronise — some take the press, some take a
    // bounce as a release, and FFs end up in inconsistent states.
    //
    // reset_debounce sits in sys_clk and runs a 3-FF synchroniser plus
    // a "stable for DEBOUNCE_CYCLES samples" filter.  Both edges go
    // through the same filter, so press AND release are clean.  The
    // debounced outputs are then combined into platform_resetn (the
    // signal every downstream async-reset path uses).
    //
    // SIM_MODEL uses a tiny window (8 cycles) so unit tbs do not waste
    // time spinning the debouncer.  Real HW uses 1024 sys_clk cycles
    // (~5 us at 200 MHz) — short enough to feel instant, long enough
    // to filter electrical glitches.  Mechanical bounce is handled by
    // the mechanical action (release after first stable closure).
`ifdef SIM_MODEL
    localparam integer RST_DEBOUNCE_CYCLES = 8;
`else
    localparam integer RST_DEBOUNCE_CYCLES = 1024;
`endif

    wire cpu_resetn_db;
    wire btn3_resetn_db_raw;   // active-low: 0 means btn[3] is pressed
    reset_debounce #(.DEBOUNCE_CYCLES(RST_DEBOUNCE_CYCLES))
        u_cpu_resetn_db (
            .clk      (sys_clk),
            .raw_in_n (cpu_resetn),
            .out_n    (cpu_resetn_db)
        );
    reset_debounce #(.DEBOUNCE_CYCLES(RST_DEBOUNCE_CYCLES))
        u_btn3_resetn_db (
            .clk      (sys_clk),
            .raw_in_n (btn[3]),          // btn[3] active-low (pressed=GND);
                                         // raw_in_n active-low matches.
            .out_n    (btn3_resetn_db_raw)   // active-low after debounce
        );
    // VIO equivalent of a btn[3] press (2026-07-24 bring-up tooling): a
    // clean, already-glitch-free level from debug_vio's probe_out1 (see
    // fpga_top_debug_vio.vh, which drives this wire -- declared here
    // since default_nettype none is active for the whole fpga_top.v
    // body these .vh files are textually included into), OR'd in
    // downstream of debounce -- no debounce needed for a VIO-driven
    // signal.  Deliberately targets this (debounced, downstream)
    // platform reset, NOT fabric_gt_clr's raw BUFG_GT .CLR above: that
    // signal explicitly excludes any JTAG-driven reset source because
    // pulsing GT CLR during an in-flight JTAG transaction can kill
    // dbg_hub's own clock mid-transaction (see that signal's own
    // comment) -- the same hazard would apply to a VIO-driven pulse,
    // since VIO writes are themselves JTAG transactions.  This
    // platform-reset path is safe to drive over JTAG.
    wire vio_hard_reset;
`ifndef VIO_ENABLE
    // No VIO instance in this configuration -- nothing drives the probe.
    // Tie it off explicitly rather than leaving the wire undriven: it now
    // reaches the DDR4 MIG's sys_rst (see soc_hard_rst_req below), and an
    // undriven reset into a hard IP is not something to leave to
    // -Wno-UNDRIVEN and synthesis's X-to-0 convention.
    assign vio_hard_reset = 1'b0;
`endif
    wire btn3_resetn_db = btn3_resetn_db_raw && !vio_hard_reset;
    wire platform_reset_req = ~btn3_resetn_db;
    // core_mmcm_locked is constant 1 on every topology except CORE_MMCM.
    // There it holds the ENTIRE SoC in async reset until the MMCM has
    // locked, which is the only correct place for it: before lock there is
    // no core clock, so a term in clk_rst's `init_done` (which is sampled
    // in the core domain) could never sequence.  Async assert, and the
    // deassert is synchronised downstream by clk_rst's own
    // xpm_cdc_async_rst.
    wire platform_resetn = cpu_resetn_db && btn3_resetn_db && core_mmcm_locked;

    // ── WHOLE-DESIGN HARD RESET (2026-09-12, work item 4) ────────────
    // "have a vio reset for the whole design" (owner, 2026-09-12).  This
    // is the ONE named request that every resettable block takes,
    // including the DDR4 MIG.  Sources, all already folded into
    // platform_resetn above:
    //   * cpu_resetn      — the board cold-reset pin (= PCIe PERST# on
    //                       XDMA builds, same physical pin T19)
    //   * btn[3]          — the platform reset button, debounced
    //   * vio_hard_reset  — debug_vio probe_out1, driven by the REPL's
    //                       `vio-hard-reset` command
    //
    // WHY IT EXISTS.  Before this, the DDR4 MIG's `sys_rst` was tied to
    // `~btn[0]` and NOTHING ELSE (fpga_top_ddr.vh).  Not cpu_resetn, not
    // btn[3], not the VIO.  So the one block in the design that could
    // only be reset by physically pressing a button on the board was the
    // memory controller -- the block a wedged fabric most often ends up
    // waiting on.  "A VIO reset for the whole design" was not achievable,
    // and no amount of reading the reset tree showed it, because the MIG
    // instance lives in the `ifndef SIM_MODEL arm that `make lint` never
    // elaborates.
    //
    // SAFE BY CONSTRUCTION, in both directions:
    //   * Resetting the MIG drops c0_init_calib_complete -> ddr_cal_done
    //     (a direct assign in ddr_ctrl.v's real-MIG arm, not a sticky
    //     latch) -> platform_init_done -> clk_rst's `rst_req = rst_in |
    //     ~init_done`.  So the SoC automatically stays in reset until DDR
    //     has re-calibrated.  No new sequencing is needed.
    //   * It cannot take the debug link down with it.  core_clk comes
    //     from the fabric BUFG_GT path, NOT from mig_ui_clk, so debug_vio
    //     and dbg_hub keep running while the MIG re-calibrates -- which
    //     is the whole point of being able to do this over JTAG.
    //
    // Deliberately NOT included: `fabric_gt_clr`, the raw BUFG_GT .CLR.
    // See its own comment -- pulsing GT CLR during an in-flight JTAG
    // transaction can kill dbg_hub's clock, and a VIO write IS a JTAG
    // transaction.
    wire soc_hard_rst_req = ~platform_resetn;

    always @(posedge sys_clk or negedge platform_resetn) begin
        if (!platform_resetn) begin
            ddr_cal_done_meta <= 1'b0;
            ddr_cal_done_sync <= 1'b0;
        end else begin
            ddr_cal_done_meta <= ddr_cal_done;
            ddr_cal_done_sync <= ddr_cal_done_meta;
        end
    end
`ifdef SIM_MODEL
    // The behavioural DDR slave is clocked by core_clk and raises
    // ddr_cal_done only after core_rst releases.  Do not feed that
    // post-reset status back into the core clock/reset generator, or
    // first-light SIM_MODEL bitstreams hold the SoC in reset forever.
    wire platform_init_done = 1'b1;
`else
    wire platform_init_done = ddr_cal_done_sync;
`endif

    (* ASYNC_REG = "TRUE" *) reg ddr_cal_done_pb_meta;
    (* ASYNC_REG = "TRUE" *) reg ddr_cal_done_pb_sync;
    always @(posedge pb_clk_src or negedge platform_resetn) begin
        if (!platform_resetn) begin
            ddr_cal_done_pb_meta <= 1'b0;
            ddr_cal_done_pb_sync <= 1'b0;
        end else begin
            ddr_cal_done_pb_meta <= ddr_cal_done;
            ddr_cal_done_pb_sync <= ddr_cal_done_pb_meta;
        end
    end
`ifdef SIM_MODEL
    wire pb_platform_init_done = 1'b1;
`else
    wire pb_platform_init_done = ddr_cal_done_pb_sync;
`endif
`ifdef SIM_MODEL
    localparam integer CLK_RST_DIVIDE = CORE_CLK_DIVIDE;
    localparam integer CLK_RST_INPUT_BUFFERED = 0;
    localparam integer PB_CLK_RST_DIVIDE = 4;
    localparam integer PB_CLK_RST_INPUT_BUFFERED = 0;
`else
    // Real-MIG builds derive the divided core clock in BUFG_GT because
    // MGTREFCLK ODIV2 cannot route directly to BUFGCE_DIV.  clk_rst only
    // sequences reset release on that already-buffered clock.
    localparam integer CLK_RST_DIVIDE = 1;
    localparam integer CLK_RST_INPUT_BUFFERED = 1;
    localparam integer PB_CLK_RST_DIVIDE = 1;
    localparam integer PB_CLK_RST_INPUT_BUFFERED = 1;
`endif

    // Peripheral-bus clk_rst.  Forwarded to clk_rst's broadcast pipeline
    // so the JTAG/VIO + button-overlay debug-full-reset (`jtag_debug_full
    // _reset_eff`) takes down BOTH the CPU domain (u_clk_rst) and the
    // peripheral-bus domain (this instance).  Without this forward, the
    // VIA1/VIA2/SCC/ASC etc. on the peripheral bus would survive a JTAG
    // reset and leave stale interrupt latches pointing at dangling CPU
    // state.  soc_full_rst_out is unused on the pb domain (no peripheral
    // distinguishes "cold" vs "soft" reset today).
    //
    // Note: jtag_debug_full_reset_eff is in the sys_clk domain.  pb_clk
    // is sourced from pb_clk_src (a separate divided clock on real-MIG
    // builds), so this is a CDC concern.  Treated the same as
    // platform_resetn (already CDC-synchronised by the upstream button
    // sync FFs in this file); clk_rst's internal rst_pipe ASYNC_REG
    // shifter handles the metastability resync downstream of dbg_full_
    // rst_in for the same reason.
    // pb_clk-domain banks — fpga_top_peripherals.vh uses them to drive
    // the via1_rst_pb / pb_full_rst broadcast cone.  Connected per
    // task #soc-full-rst-tree to drop the pb-domain BUFG insertion from
    // the post-place log.
    wire [7:0] pb_core_rst_bank;
    wire [7:0] pb_soc_full_rst_bank;
    clk_rst #(
        .CLK_DIVIDE(PB_CLK_RST_DIVIDE),
        .INPUT_BUFFERED(PB_CLK_RST_INPUT_BUFFERED)
    ) u_pb_clk_rst (
        .clk_in          (pb_clk_src),
        .rst_in          (~platform_resetn),
        .init_done       (pb_platform_init_done),
        .dbg_full_rst_in (jtag_debug_full_reset_eff),
        .clk_out         (pb_clk),
        .rst_out         (pb_rst),
        .soc_full_rst_out(/* unused on peripheral-bus domain */),
        .core_rst_bank   (pb_core_rst_bank),
        .soc_full_rst_bank(pb_soc_full_rst_bank)
    );

    // Forward declarations consumed by reset gating before the producer
    // blocks are instantiated below.  vio_boot_ctrl is driven by the VIO
    // probe inside fpga_top_debug_vio.vh; it appears here as a forward
    // wire so the clk_rst broadcast pipeline can absorb the JTAG
    // debug-full-reset bit (Stage-0 fix #2 of fmax_autopsy_20260425.md).
    wire boot_rom_loaded;
    // dbg_soft_rst — DEPRECATED legacy alias (DBG_CONTROL bit 2).
    // dbg_cold_reset_pulse — canonical unified-reset trigger (DBG_CONTROL
    // bit 5).  debug_ctrl drives both wires from the same OR'd source so
    // either bit reaches the unified-reset path.
    wire dbg_soft_rst;
    wire dbg_cold_reset_pulse;
    // dbg_cold_reset_hold — DBG_CONTROL bit 4 (sticky level).  ORs into
    // cpu_rst_or so the CPU stays held after the unified reset deasserts,
    // until the host explicitly clears the bit.  Owned by debug_ctrl,
    // which lives on `core_rst` only, so the bit survives the
    // soc_full_rst it triggers.
    wire dbg_cold_reset_hold;
    wire via1_overlay_bit;
    // [4] = pram_clear (zap PRAM, Cmd-Opt-P-R equivalent) — see the bit-map
    // comment block further down.  New bit defaults LOW in both sim
    // constants: PRAM must never be zapped implicitly.
    wire [4:0] vio_boot_ctrl;
`ifdef FPGA_ROM_SIM
`ifdef FPGA_ROM_SIM_REALBOOT
    // realboot variant (tb-fpga-top-rom-realboot): keep FPGA_ROM_SIM's
    // big DDR-model sizing (ddr_ctrl.v addr_to_idx/BEATS, sized to hold
    // the real 1 MiB ROM at its real production addresses -- see that
    // file's FPGA_ROM_SIM-gated block) but do NOT bypass boot_fsm.
    // vio_boot_ctrl=0 here means jtag_boot_bypass=0/jtag_boot_release=0,
    // i.e. boot_rom_ready genuinely waits for boot_fsm's own real
    // SD-streaming-and-mirror completion, exactly like the real FPGA
    // path below -- this is the whole point of the realboot harness:
    // observe that real gating with a real CPU, without also paying for
    // the tiny default 64 KB DDR model that can't hold a real ROM image.
    assign vio_boot_ctrl = 5'b00000;
`else
    assign vio_boot_ctrl = 5'b00011;
`endif
`else
`ifndef VIO_ENABLE
    assign vio_boot_ctrl = 5'b00000;
`endif
`endif
    // ── Boot bypass — SIM ONLY ───────────────────────────────────────
    // FPGA_ROM_SIM preloads DRAM directly and must skip boot_fsm, so the
    // sim keeps the host-driven bypass/release bits.  On the real FPGA we
    // force NO bypass: boot_fsm always runs on every soc_full_rst, so its
    // SD ROM copy AND its RAM-window zero pass (boot_fsm.v ST_ZERO_*)
    // always execute — DRAM is wiped clean before the m68k runs.  Without
    // this, a JTAG-side-loaded boot left RAM uninitialised → the Q700 ROM
    // read stale stripe data → stale-RAM Sad Mac on every reset.
`ifdef FPGA_ROM_SIM
    wire jtag_boot_bypass       = vio_boot_ctrl[0];
    wire jtag_boot_release      = vio_boot_ctrl[1];
`else
    wire jtag_boot_bypass       = 1'b0;
    wire jtag_boot_release      = 1'b0;
    // vio_boot_ctrl[1:0] are unused on the real FPGA (no bypass) — sink
    // them so the VIO probe-out bus doesn't trip an UNUSED lint warning.
    // verilator lint_off UNUSED
    wire _unused_vio_boot       = &{1'b0, vio_boot_ctrl[1:0]};
    // verilator lint_on UNUSED
`endif
    wire jtag_scc_uart_sel_b    = vio_boot_ctrl[2];
    wire jtag_debug_full_reset  = vio_boot_ctrl[3];
    // Raw LEVEL from the VIO probe-out.  Never route this straight to the
    // RTC — see the jtag_pram_clear_eff one-shot below.
    wire jtag_pram_clear        = vio_boot_ctrl[4];

    // ─── Board-button JTAG control overlay ─────────────────────────────
    // btn[0] is the DDR sys_rst (already wired in fpga_top_ddr.vh).
    // btn[3] is the platform_reset_req (asynchronous DDR-side reset).
    // btn[1] / btn[2] were free; route them to NMI and debug-full-reset
    // respectively so the board has physical access to those JTAG-only
    // controls.  Both lines are async; sync + DEBOUNCE in sys_clk before
    // OR-ing with the JTAG/VIO source.
    //
    // Why debounce btn[1] (NMI): a mechanical tactile switch produces
    // 10s-100s of transitions per press over ~1-10 ms.  Without
    // filtering, irq_agg's rising-edge detector treats every bounce as
    // a fresh NMI rising edge → CPU pushes a new exception frame for
    // each bounce → SSP marches down by ~14 bytes per bounce until it
    // walks straight off the bottom of RAM (observed on bench: holding
    // btn[1] longer drops SP further, eventually into VRAM).  A 10 ms
    // stable-window kills that — only one debounced rising edge per
    // physical press reaches irq_agg, which is the contract the agg's
    // edge-detect was designed for.  btn[2] (debug full reset) goes
    // through its own rising-edge + pulse-stretch + watchdog stage
    // below, so a bouncy press there is benign (multiple rising edges
    // collapse onto one stretched pulse), but we still debounce it
    // for symmetry and to avoid wasting the watchdog re-arm window.
    //
    // 2_000_000 sys_clk cycles @ 200 MHz = 10 ms — covers the worst
    // tactile-switch bounce envelope with margin.  reset_debounce is
    // active-low (raw_in_n / out_n); btn[N] is active-low on the board
    // so feeds raw_in_n directly, and we invert the debounced active-
    // low output to get the active-high "pressed" form the consumers
    // (nmi_btn_pulse, debug_full_reset edge detector) want.
    localparam integer NMI_DEBOUNCE_CYCLES = 2_000_000;
    wire btn1_pressed_n_db;
    wire btn2_pressed_n_db;
    // IDLE_OUT_N=1: these are momentary buttons, NOT reset lines — the
    // safe/true idle state is "not pressed" (out_n=1), unlike the
    // cpu_resetn/btn[3] instances above which want out_n=0 ("reset
    // held") at power-up.  See reset_debounce.v's header comment
    // ("Power-up idle polarity") for the real-HW boot-NMI bug this
    // fixes: without this override, out_n powered up at 0 ("pressed")
    // for the first ~10 ms after every FPGA configuration, producing a
    // guaranteed spurious rising edge on nmi_btn_core that irq_agg.v
    // latched as a genuine NMI on every single boot.
    reset_debounce #(.DEBOUNCE_CYCLES(NMI_DEBOUNCE_CYCLES), .IDLE_OUT_N(1'b1))
        u_btn1_db (
            .clk      (sys_clk),
            .raw_in_n (btn[1]),
            .out_n    (btn1_pressed_n_db)
        );
    reset_debounce #(.DEBOUNCE_CYCLES(NMI_DEBOUNCE_CYCLES), .IDLE_OUT_N(1'b1))
        u_btn2_db (
            .clk      (sys_clk),
            .raw_in_n (btn[2]),
            .out_n    (btn2_pressed_n_db)
        );
    wire btn1_sync = ~btn1_pressed_n_db;
    wire btn2_sync = ~btn2_pressed_n_db;

    // ─── Edge-trigger + pulse-stretch + watchdog for debug_full_reset ──
    //
    // Why this exists: vio_boot_ctrl[3] is a level-driven probe-out bit.
    // If the host sets it high and the JTAG link becomes unreachable
    // (we hit this twice on the bench), the bit cannot be cleared
    // without re-loading the bitstream.  Without this stage the SoC
    // would then sit in soc_full_rst forever — no way to recover.
    //
    // Fix: detect the RISING EDGE of (vio bit || debounced btn2 press)
    // and emit a fixed-length pulse on jtag_debug_full_reset_eff.  The
    // pulse length (PULSE_CYCLES) covers the xpm_cdc_async_rst
    // DEST_SYNC_FF=4 chain in clk_rst, with margin.  After the pulse
    // expires the eff line auto-deasserts even if the source bit /
    // button is still held.  The user re-arms by dropping the source
    // and asserting it again (rising edge gated by `armed`).
    //
    // Watchdog: independent counter that force-clears `armed` after
    // STUCK_CYCLES of continuous high — recovers from a stuck VIO
    // probe-out without bitstream reload.  STUCK_CYCLES is set so the
    // OS-side host can hold the bit for a millisecond-class operation
    // and still re-arm normally; longer holds are treated as faults.
    //
    // Same treatment for btn2 (board cold-reset button): a held button
    // generates only one pulse per edge — repeated taps required to
    // re-fire — matching the JTAG path's semantic and preventing
    // mechanical bounce from looking like multiple resets.
`ifdef SIM_MODEL
    // Sim-friendly values: 32-cycle pulse, 1024-cycle stuck threshold.
    // Real hardware uses ~ms-class numbers (see else branch).
    localparam integer DBG_RST_PULSE_CYCLES = 32;
    localparam integer DBG_RST_STUCK_CYCLES = 1024;
`else
    // Pulse: 1024 cycles at 200 MHz = 5.12 µs.  The previous 64-cycle
    // (320 ns) value released the umbrella reset before the MIG/DDR4
    // could drain any in-flight AW/W/B handshake on the wide AXI port
    // (`ddr_ctrl` is intentionally on `core_rst`, NOT `soc_full_rst_bank`,
    // to skip the ~100 ms DDR re-cal — see the next-block comment).
    // The boot_master, narrow→wide bridge, and xbar all clear on the
    // umbrella reset; if that fired mid-handshake, the MIG would
    // retain a stale beat that wedged the next boot's first SD-image
    // write in `boot_fsm.ST_CTRL_RUN` (m_axi_awready never asserts).
    // Stretching the pulse to ~5 µs gives any in-flight beat plenty of
    // wall-clock time to retire on the MIG side.  See the SD-loader
    // every-other-reset memory note for the long form.
    // Stuck threshold: roughly 1 ms at 200 MHz = 200_000 cycles.
    localparam integer DBG_RST_PULSE_CYCLES = 1024;
    localparam integer DBG_RST_STUCK_CYCLES = 200_000;
`endif
    // Counter widths sized to hold the larger threshold.  Use clog2 +1
    // so the counter can saturate at STUCK_CYCLES without rolling.
    function integer clog2_local;
        input integer v;
        integer       i;
        begin
            clog2_local = 0;
            for (i = v - 1; i > 0; i = i >> 1) clog2_local = clog2_local + 1;
        end
    endfunction
    localparam integer DBG_RST_PCNT_W   = clog2_local(DBG_RST_PULSE_CYCLES + 1);
    localparam integer DBG_RST_STUCK_W  = clog2_local(DBG_RST_STUCK_CYCLES + 1);

    // dbg_rst_src_level — sources of the unified reset.  Includes:
    //   * JTAG VIO bit 3                 — legacy / backstop level path
    //   * btn[2] (debounced)             — physical / recovery button
    //   * dbg_cold_reset_pulse           — JTAG-AXI canonical trigger
    //                                      (DBG_CONTROL bit 5; debug_ctrl
    //                                      already widens to one cycle
    //                                      via the pulse-clear).  Held
    //                                      high for one core_clk cycle,
    //                                      which the rising-edge
    //                                      detector turns into the
    //                                      stretched pulse below.
    wire dbg_rst_src_level = jtag_debug_full_reset || btn2_sync ||
                             dbg_cold_reset_pulse;
    reg                          dbg_rst_src_q;
    reg                          dbg_rst_armed;
    reg [DBG_RST_PCNT_W-1:0]     dbg_rst_pulse_cnt;
    reg                          dbg_rst_pulse_active;
    reg [DBG_RST_STUCK_W-1:0]    dbg_rst_stuck_cnt;

    wire dbg_rst_rising = dbg_rst_src_level && !dbg_rst_src_q && dbg_rst_armed;

    always @(posedge sys_clk or negedge platform_resetn) begin
        if (!platform_resetn) begin
            dbg_rst_src_q        <= 1'b0;
            dbg_rst_armed        <= 1'b1;
            dbg_rst_pulse_cnt    <= {DBG_RST_PCNT_W{1'b0}};
            dbg_rst_pulse_active <= 1'b0;
            dbg_rst_stuck_cnt    <= {DBG_RST_STUCK_W{1'b0}};
        end else begin
            dbg_rst_src_q <= dbg_rst_src_level;

            // Pulse generator: rising edge starts a fixed-length pulse;
            // the pulse runs to completion regardless of the source.
            if (dbg_rst_rising) begin
                dbg_rst_pulse_active <= 1'b1;
                dbg_rst_pulse_cnt    <= DBG_RST_PULSE_CYCLES[DBG_RST_PCNT_W-1:0];
            end else if (dbg_rst_pulse_active) begin
                if (dbg_rst_pulse_cnt == {{(DBG_RST_PCNT_W-1){1'b0}}, 1'b1}) begin
                    dbg_rst_pulse_active <= 1'b0;
                    dbg_rst_pulse_cnt    <= {DBG_RST_PCNT_W{1'b0}};
                end else begin
                    dbg_rst_pulse_cnt <= dbg_rst_pulse_cnt - 1'b1;
                end
            end

            // Watchdog: if the source stays high for STUCK_CYCLES the
            // pulse generator disarms locally, so a broken VIO probe
            // (or wedged button) cannot keep the SoC in reset forever.
            // Re-arm requires a clean low → high transition observed
            // by the rising-edge detector.
            if (!dbg_rst_src_level) begin
                dbg_rst_stuck_cnt <= {DBG_RST_STUCK_W{1'b0}};
                dbg_rst_armed     <= 1'b1;
            end else if (dbg_rst_armed && !dbg_rst_pulse_active) begin
                if (dbg_rst_stuck_cnt == DBG_RST_STUCK_CYCLES[DBG_RST_STUCK_W-1:0]) begin
                    dbg_rst_armed <= 1'b0;
                end else begin
                    dbg_rst_stuck_cnt <= dbg_rst_stuck_cnt + 1'b1;
                end
            end
        end
    end

    // jtag_debug_full_reset_eff feeds clk_rst's debug-reset input below
    // and the soc_full_rst broadcast pipeline.  It is the stretched
    // pulse output of the edge-detector — NOT the raw level — so a
    // wedged probe-out / stuck button cannot hold the SoC in reset.
    // nmi_btn_pulse stays level-driven from the synced button: the
    // downstream irq_agg.v already does its own rising-edge detect
    // (`nmi_rise = nmi_edge && !nmi_edge_q`), so a held button still
    // takes the NMI exactly once per press.
    wire jtag_debug_full_reset_eff = dbg_rst_pulse_active;
    wire nmi_btn_pulse             = btn1_sync;

    // ─── Edge-trigger + pulse-stretch for PRAM clear (vio_boot_ctrl[4]) ─
    //
    // Same hazard as vio_boot_ctrl[3] above: VIO probe-outs are LEVEL
    // driven.  PRAM is now battery-backed (rtc.v deliberately does not
    // reset it), and `pram_clear` rewrites all 256 bytes on EVERY clock
    // it is high.  Feeding the raw level in would mean that for as long
    // as the operator leaves the VIO bit set — or forever, if the JTAG
    // link drops with the bit high — the RTC would swallow every PRAM
    // write Mac OS makes.  That misreads as "PRAM persistence is broken"
    // rather than "the zap is stuck on", so it is edge-triggered here.
    //
    // Rising edge → one fixed-length pulse.  A level stuck high produces
    // exactly ONE zap and then nothing: `pram_clear_src_q` tracks the
    // source, so the (src && !src_q) term can never re-assert without the
    // operator first dropping the bit.  No watchdog is needed (unlike the
    // reset path, there is no state being HELD that a stuck bit could
    // wedge — the pulse self-terminates from its own counter).
    //
    // Pulse width deliberately reuses DBG_RST_PULSE_CYCLES: this signal
    // makes the identical sys_clk → pb_clk crossing as
    // jtag_debug_full_reset_eff (2-FF sync in fpga_top_peripherals.vh),
    // and pb_clk = sys_clk / PB_CLK_RST_DIVIDE (4 on HW, 1 in sim), so
    // 1024 sys_clk cycles = 256 pb_clk cycles at the destination — orders
    // of magnitude wider than the 2-FF synchroniser needs.  ~5.12 µs of
    // held-clear is harmless: a bit-banged RTC PRAM transaction takes
    // tens of µs per byte, so no in-flight Mac OS write can be lost
    // without the operator having asked for a zap.
    // PRAM_CLEAR_ONESHOT_BEGIN
    // ^ Marker: tools/extract_pram_clear_oneshot.py lifts the block between
    //   these two markers VERBATIM into a standalone DUT for
    //   `make tb-pram-clear-pulse`.  The test therefore exercises the real
    //   shipped logic, not a hand-maintained copy that can silently drift
    //   out of sync with it.  Keep the markers; keep the block
    //   self-contained (only sys_clk / platform_resetn / jtag_pram_clear in,
    //   pram_clear_pulse_active out).
    reg                      pram_clear_src_q;
    reg [DBG_RST_PCNT_W-1:0] pram_clear_pulse_cnt;
    reg                      pram_clear_pulse_active;

    wire pram_clear_rising = jtag_pram_clear && !pram_clear_src_q;

    always @(posedge sys_clk or negedge platform_resetn) begin
        if (!platform_resetn) begin
            pram_clear_src_q        <= 1'b0;
            pram_clear_pulse_cnt    <= {DBG_RST_PCNT_W{1'b0}};
            pram_clear_pulse_active <= 1'b0;
        end else begin
            pram_clear_src_q <= jtag_pram_clear;
            if (pram_clear_rising) begin
                pram_clear_pulse_active <= 1'b1;
                pram_clear_pulse_cnt    <= DBG_RST_PULSE_CYCLES[DBG_RST_PCNT_W-1:0];
            end else if (pram_clear_pulse_active) begin
                if (pram_clear_pulse_cnt == {{(DBG_RST_PCNT_W-1){1'b0}}, 1'b1}) begin
                    pram_clear_pulse_active <= 1'b0;
                    pram_clear_pulse_cnt    <= {DBG_RST_PCNT_W{1'b0}};
                end else begin
                    pram_clear_pulse_cnt <= pram_clear_pulse_cnt - 1'b1;
                end
            end
        end
    end

    // PRAM_CLEAR_ONESHOT_END
    // Consumed by fpga_top_peripherals.vh, which CDCs it into pb_clk and
    // drives u_rtc.pram_clear.
    wire jtag_pram_clear_eff = pram_clear_pulse_active;

    clk_rst #(
        .CLK_DIVIDE(CLK_RST_DIVIDE),
        .INPUT_BUFFERED(CLK_RST_INPUT_BUFFERED)
    ) u_clk_rst (
        .clk_in          (sys_clk),
        // Use the debounced platform_resetn (= cpu_resetn_db && btn3_resetn_db)
        // so a bouncy mechanical press cannot fire multiple resets at the
        // 5+ async-clear consumers downstream.
        .rst_in          (~platform_resetn),
        .init_done       (platform_init_done),
        .dbg_full_rst_in (jtag_debug_full_reset_eff),
        .clk_out         (core_clk),
        .rst_out         (core_rst),
        .soc_full_rst_out(soc_full_rst),
        .core_rst_bank   (core_rst_bank),
        .soc_full_rst_bank(soc_full_rst_bank)
    );


    // CPU-only reset — hold the 68k in reset until the peripheral island has
    // had 50 ms out of reset and the ROM image is ready.
    // Normal path: boot_fsm copies ROM from SD and raises boot_rom_loaded.
    // JTAG path: VIO can reset/disable boot_fsm, hold the CPU while the
    // host writes DRAM through JTAG-to-AXI, then release only the CPU.
    //
    // vio_boot_ctrl[0] = bypass SD boot / hold boot_fsm reset
    // vio_boot_ctrl[1] = release CPU after external ROM load
    // vio_boot_ctrl[2] = SCC UART channel select (0 = channel A, 1 = B).
    //                    Routes the board UART through scc_uart_sel_b /
    //                    scc_uart_sel_a in fpga_top_peripherals.vh.
    //                    Originally bit[2] gated a CPU-only halt
    //                    (jtag_cpu_hold) — that function is now strictly
    //                    subsumed by bit[3] debug_full_reset (which both
    //                    holds the CPU AND re-arms the overlay through
    //                    VIA1 reset), freeing
    //                    bit[2] for the SCC selector needed by the
    //                    MAME-lockstep peripheral exerciser.
    // vio_boot_ctrl[3] = full debug reset — equivalent of a board-level
    //                    cold reset for ALL platform state except the
    //                    DDR PHY (MIG cal preserved — re-cal is ~100ms,
    //                    defeats the fast warm-reset UX) and the master
    //                    sys_clk MMCM tree.  Holds the CPU, invalidates
    //                    L1I/L1D, rewinds ROB/IQ/RAT/CCR-RAT/commit/
    //                    exception state, re-arms the boot_fsm so the
    //                    SD→DDR ROM copy starts over, and resets every
    //                    peripheral (VIA1/2, SCC, SCSI, ASC, IWM, RTC,
    //                    Orwell, SONIC, DAFB/video, DMA controller,
    //                    AXI xbar, irq_agg).  The peripheral service bus
    //                    FSM stays on the PB-domain core reset so JTAG can
    //                    still reach debug_ctrl to release cold_reset_hold.
    //                    ORB[3]
    //                    returns to its post-reset value so the xbar-level
    //                    low-memory ROM alias re-asserts.  Releasing bit[3]
    //                    makes the CPU resume from the reset PC (vec-0
    //                    fetch under FETCH_RESET_VECTORS=1) with the
    //                    low-memory ROM alias live again — a real
    //                    cold-boot semantic without re-programming the
    //                    bitstream.  Same wire is asserted by btn[2] so
    //                    a board operator can issue the same true
    //                    cold-boot from the physical button.  See task
    //                    #256 / docs/hw_debug.md.
    // vio_boot_ctrl[4] = zap PRAM — the RTL equivalent of a Macintosh's
    //                    Cmd-Opt-P-R.  Rewrites all 256 RTC PRAM bytes
    //                    from their power-on image.  Needed because PRAM
    //                    is now battery-backed: rtc.v deliberately does
    //                    NOT clear the array on reset (nor on bit[3]'s
    //                    debug-full-reset), so user settings survive a
    //                    warm reset exactly as on real silicon — and this
    //                    bit is therefore the ONLY software-reachable way
    //                    back to a known-good image if a bad PRAM image
    //                    ever wedges the ROM boot.
    //                    EDGE-triggered, not level: see the
    //                    jtag_pram_clear_eff one-shot above.  Leaving the
    //                    bit set does NOT hold PRAM clear.
    //                    JTAG REPL: `pram-clear`.
    //
    // soc_full_rst (= core_rst || jtag_debug_full_reset, registered) is
    // produced inside u_clk_rst so the OR sits in the broadcast pipeline
    // rather than at every receiver.  It holds cpu_rst through the re-arm.
    // Every core-clk consumer that previously took core_rst now takes
    // soc_full_rst so the JTAG/btn-driven debug-full-reset reaches them.
    // The peripheral-bus island uses pb_full_rst (declared in
    // fpga_top_peripherals.vh as `pb_rst | debug_full_reset_pb_sync`) for
    // the same coverage in pb_clk domain.
    //
    // Excluded from the broadcast (intentional):
    //   - DDR PHY / MIG calibration FSM:  re-cal takes ~100 ms; the
    //     content of DRAM is content-addressed and re-populated by
    //     boot_fsm anyway, and a debug-full-reset must NOT take 100 ms.
    //   - sys_clk MMCM clock tree:  re-locking the master clock would
    //     glitch every clock domain — current behaviour preserves the
    //     clock tree across core_rst already, so this matches.
    //   - debug_ctrl / debug_stop_manager:  JTAG-side controllers; they
    //     are owned by the host, not by the design under reset.
    // boot_rom_ready release-gate.
    //
    // Bug guarded:
    //   The previous mux meant that with vio_boot_ctrl=0x3 (bypass +
    //   release) set, then a debug_full_reset pulse, then bit[3]
    //   deasserted, the [1] release-CPU bit was still asserted — and
    //   the CPU started executing the moment soc_full_rst dropped,
    //   BEFORE the boot FSM had refreshed the ROM image.
    //
    // Fix:
    //   1. The ready signal now derives from the OR of boot_rom_loaded
    //      and (bypass && release) so the bypass path no longer SHADOWS
    //      a stale boot_rom_loaded — both still produce a release.
    //   2. Gate the whole thing on !soc_full_rst so the CPU stays held
    //      during the reset itself even if the JTAG release bit is
    //      already set.  After soc_full_rst drops, the release path
    //      proceeds normally — but boot_fsm_rst (below) folds in
    //      soc_full_rst too, so boot_rom_loaded re-clears and the CPU
    //      waits for the SD→DDR refresh to finish before resuming.
    // Hold the CPU across the boot-time PRAM install too.  boot_rom_loaded is
    // what tells pram_sd the card is usable AND what releases the CPU, so
    // without this the Mac races the load and reads rtc.v's defaults --
    // making the restore useless exactly when it matters.
    //
    // The hold is BOUNDED.  pram_sd resolves every operation on its own
    // watchdogs (falling back to defaults loudly), but this is boot-critical
    // logic: if the pending signal ever stuck high the board would simply
    // never start.  ~2^24 core cycles is ~0.17 s at 100 MHz -- far longer than
    // a single-sector read, far shorter than a human notices -- after which
    // the CPU is released regardless and the operator sees the verdict in
    // pram_sd's STATUS.  A convenience feature must never be able to brick
    // the boot.
    localparam integer PRAM_HOLD_LOG2 = 24;
    reg [PRAM_HOLD_LOG2:0] pram_hold_cnt = {(PRAM_HOLD_LOG2+1){1'b0}};
    wire pram_hold_expired = pram_hold_cnt[PRAM_HOLD_LOG2];
    always @(posedge core_clk) begin
        if (soc_full_rst) pram_hold_cnt <= {(PRAM_HOLD_LOG2+1){1'b0}};
        else if (!pram_hold_expired) pram_hold_cnt <= pram_hold_cnt + 1'b1;
    end
    wire pram_boot_hold = pram_autoload_pending && !pram_hold_expired;

    wire boot_rom_ready    = !soc_full_rst && !pram_boot_hold &&
                             (boot_rom_loaded ||
                              (jtag_boot_bypass && jtag_boot_release));
    // boot_fsm_rst now folds in soc_full_rst so a debug-full-reset
    // re-arms the SD→DDR ROM copy.  Without this the boot_fsm would
    // silently leave rom_loaded asserted across the warm reset and the
    // CPU would resume against the half-stale post-reset DRAM image.
    // dbg_cold_reset_hold is folded in for the SAME reason, and closes the
    // last hole in "every reset re-copies the ROM".  It is a CPU-reset
    // source (see cpu_rst_or below) that does NOT reach soc_full_rst, so
    // before this it restarted the 68k against a DRAM ROM image and
    // low-memory RAM window that were never re-copied — the exact
    // half-stale case the comment above describes, observed on hardware as
    // bus errors with no sensible faulting address after a debug reset.
    //
    // Safe by the same interlock: boot_fsm.v clears rom_loaded under
    // boot_fsm_rst, which drops boot_rom_ready, which keeps cpu_rst
    // asserted for the whole re-copy.  The CPU is therefore held across
    // the entire boot_fsm run, preserving axi_xbar's precondition that
    // boot-FSM and CPU traffic are never simultaneously live on slot 0
    // (that fan-in is a plain 2:1 mux, not an arbiter, and relies on it).
    // Because the hold is a sticky level, the FSM stays in reset while it
    // is asserted and runs the copy on release.
    wire boot_fsm_rst      = soc_full_rst || jtag_boot_bypass ||
                             dbg_cold_reset_hold;

    (* ASYNC_REG = "TRUE" *) reg pb_rst_core_meta;
    (* ASYNC_REG = "TRUE" *) reg pb_rst_core_sync;
    always @(posedge core_clk) begin
        if (soc_full_rst) begin
            pb_rst_core_meta <= 1'b1;
            pb_rst_core_sync <= 1'b1;
        end else begin
            pb_rst_core_meta <= pb_rst;
            pb_rst_core_sync <= pb_rst_core_meta;
        end
    end

`ifdef SIM_MODEL
    localparam integer CPU_RST_SETTLE_CYCLES_INT = 100;
`else
    localparam integer CPU_RST_SETTLE_CYCLES_INT = (CORE_CLK_HZ + 19) / 20;
`endif
    localparam [31:0] CPU_RST_SETTLE_CYCLES =
        (CPU_RST_SETTLE_CYCLES_INT < 1) ? 32'd1 : CPU_RST_SETTLE_CYCLES_INT;
    reg [31:0] cpu_rst_settle_count;
    reg        cpu_rst_settle_done;
    wire       cpu_rst_settle_reset = soc_full_rst || pb_rst_core_sync;
    always @(posedge core_clk) begin
        if (cpu_rst_settle_reset) begin
            cpu_rst_settle_count <= 32'd0;
            cpu_rst_settle_done  <= 1'b0;
        end else if (!cpu_rst_settle_done) begin
            if (cpu_rst_settle_count >= (CPU_RST_SETTLE_CYCLES - 32'd1)) begin
                cpu_rst_settle_done <= 1'b1;
            end else begin
                cpu_rst_settle_count <= cpu_rst_settle_count + 32'd1;
            end
        end
    end

    // ─── cpu_rst minimum-pulse stretcher ───────────────────────────────
    //
    // Bug guarded:
    //   A 1-cycle glitch on a CPU-only reset path (e.g. from a JTAG-AXI
    //   race) gives the CPU a 1-cycle reset, which is too short to put
    //   PRF / ROB / RAT / CCR-RAT into a fully-known state — those
    //   structures contain FFs that need multiple consecutive clock
    //   edges to clear.  An OoO pipeline is fragile to truncated reset
    //   pulses.
    //
    // Fix:
    //   Standard reset stretcher: any time the OR asserts, reload an
    //   8-cycle shift register with all 1s.  cpu_rst_stretched is the
    //   OR of the SR bits — it stays high for at least 8 cycles after
    //   the last input deasserts.  8 cycles is enough for the deepest
    //   reset cone in the core (the CCR-RAT free-list rebuild, which
    //   touches 17 arch regs over multiple cycles).
    //
    // Inputs to the OR:
    //   soc_full_rst        — board cold reset OR debug-full-reset
    //                         (covers VIO bit 3, btn[2], and the new
    //                         JTAG-AXI cold_reset_pulse path via
    //                         dbg_rst_src_level).
    //   dbg_cold_reset_hold — sticky CPU hold (DBG_CONTROL bit 4).
    //                         Replaces the legacy dbg_soft_rst (CPU-only
    //                         soft reset that contaminated DDR /
    //                         peripherals / boot FSM and lost the
    //                         "halt across reset" race).  When set,
    //                         the CPU stays in reset across and after
    //                         the unified reset until the host clears
    //                         the bit.
    //   !boot_rom_ready     — boot FSM hasn't finished SD→DDR copy.
    //   !cpu_rst_settle_done — post-reset settle window.
    wire cpu_rst_or = soc_full_rst || dbg_cold_reset_hold ||
                      !boot_rom_ready || !cpu_rst_settle_done;
    reg [7:0] cpu_rst_stretch;
    always @(posedge core_clk) begin
        if (cpu_rst_or)
            cpu_rst_stretch <= 8'hFF;
        else
            cpu_rst_stretch <= {cpu_rst_stretch[6:0], 1'b0};
    end
    wire cpu_rst = cpu_rst_or || (|cpu_rst_stretch);

    // ═══════════════════════════════════════════════════════════════════
    // phi2_tick — Q700 VIA timebase for 6522 timers
    // ═══════════════════════════════════════════════════════════════════
    // Match the Quadra 700's VIA timebase rather than a generic 1 MHz
    // placeholder.  Use the same NCO shape as the MAME peripheral bridge
    // so long ROM polling loops see the same average phase instead of an
    // integer-divide drift.
    localparam [31:0] PB_CLK_HZ_U32 = PB_CLK_HZ;
    localparam [31:0] VIA_PHI2_HZ_U32 = VIA_PHI2_HZ;
    reg [31:0] phi2_accum;
    reg        phi2_tick_r;
    wire [32:0] phi2_accum_sum = {1'b0, phi2_accum} + {1'b0, VIA_PHI2_HZ_U32};
    always @(posedge pb_clk) begin
        if (pb_rst) begin
            phi2_accum  <= 32'd0;
            phi2_tick_r <= 1'b0;
        end else if (phi2_accum_sum >= {1'b0, PB_CLK_HZ_U32}) begin
            phi2_accum  <= phi2_accum_sum[31:0] - PB_CLK_HZ_U32;
            phi2_tick_r <= 1'b1;
        end else begin
            phi2_accum  <= phi2_accum_sum[31:0];
            phi2_tick_r <= 1'b0;
        end
    end
    wire phi2_tick = phi2_tick_r;
