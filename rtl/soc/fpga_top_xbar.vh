// rtl/fpga_top_xbar.vh — included from rtl/fpga_top.v
//
// AXI xbar 3M x 6S slave-side wires, axi_xbar instance, S2 (128-bit)
// -> dma_ctrl.cfg_* bridge, S4 -> DAFB AXI-Lite bridge, and S5 ->
// SD JTAG writer AXI-Lite bridge.
//
// 2026-07-16 master-count reduction: was 5M (CPU LSU, host debug, boot
// FSM, CPU IF, DMA).  Boot FSM is now time-multiplexed onto the CPU LSU
// port (M0) via a `cpu_rst`-selected mux inside axi_xbar.v itself — see
// its header note "M0/boot merge" — bound below as the m0b_* sub-port.
// DMA's AXI4 master port (was M4) is stubbed out entirely; see
// `fpga_top_dma.vh` for how dma_ctrl's narrow AXI master inputs are tied
// to idle now that nothing forwards them to the xbar.
//
// Originally generated mechanically from rtl/fpga_top.v during the
// fpga_top split (agent/p3-fpga-top-split); the master-count reduction
// above is a genuine functional change on top of that mechanical split.
// Do not add module/endmodule wrappers or nettype directives.

    // ═══════════════════════════════════════════════════════════════════
    // AXI xbar — 3M × 6S (M0 = CPU LSU + boot FSM merge, M1 = host debug,
    // M2 = CPU IF; S2 = DMA config [master stubbed, see above],
    // S3 = VRAM aperture, S4 = DAFB register shim, S5 = SD JTAG writer)
    // ═══════════════════════════════════════════════════════════════════
    // S3 is the VRAM pixel aperture (0xF900_0000..0xF90F_FFFF) added in
    // task #147.  The xbar strips VRAM_BASE so the slave sees a zero-
    // based byte offset — matching rtl/sys/vram.v's contract.
    // Slave 0 (DDR)
    wire [5:0]   s0_awid;
    wire [31:0]  s0_awaddr;
    wire [7:0]   s0_awlen;
    wire [2:0]   s0_awsize;
    wire [1:0]   s0_awburst;
    wire         s0_awvalid;
    wire         s0_awready;
    // S0 W channel is PIPELINED (2026-09-12, FMax).  `s0w_*` is the raw
    // crossbar output; `s0_*` is what the DDR-side slave (u_l2c, or
    // u_vram_lane_mux/u_ddr when L2C_ENABLE is off) actually sees, one
    // register stage later.  See u_s0_w_slice below for why.
    wire [127:0] s0w_wdata;
    wire [15:0]  s0w_wstrb;
    wire         s0w_wlast;
    wire         s0w_wvalid;
    wire         s0w_wready;
    wire [127:0] s0_wdata;
    wire [15:0]  s0_wstrb;
    wire         s0_wlast;
    wire         s0_wvalid;
    wire         s0_wready;
    wire [5:0]   s0_bid;
    wire [1:0]   s0_bresp;
    wire         s0_bvalid;
    wire         s0_bready;
    wire [5:0]   s0_arid;
    wire [31:0]  s0_araddr;
    wire [7:0]   s0_arlen;
    wire [2:0]   s0_arsize;
    wire [1:0]   s0_arburst;
    wire         s0_arvalid;
    wire         s0_arready;
    wire [5:0]   s0_rid;
    wire [127:0] s0_rdata;
    wire [1:0]   s0_rresp;
    wire         s0_rlast;
    wire         s0_rvalid;
    wire         s0_rready;

    // Slave 1 (I/O peripheral bus)
    wire [5:0]   s1_awid;
    wire [31:0]  s1_awaddr;
    wire [7:0]   s1_awlen;
    wire [2:0]   s1_awsize;
    wire [1:0]   s1_awburst;
    wire         s1_awvalid;
    wire         s1_awready;
    wire [127:0] s1_wdata;
    wire [15:0]  s1_wstrb;
    wire         s1_wlast;
    wire         s1_wvalid;
    wire         s1_wready;
    wire [5:0]   s1_bid;
    wire [1:0]   s1_bresp;
    wire         s1_bvalid;
    wire         s1_bready;
    wire [5:0]   s1_arid;
    wire [31:0]  s1_araddr;
    wire [7:0]   s1_arlen;
    wire [2:0]   s1_arsize;
    wire [1:0]   s1_arburst;
    wire         s1_arvalid;
    wire         s1_arready;
    wire [5:0]   s1_rid;
    wire [127:0] s1_rdata;
    wire [1:0]   s1_rresp;
    wire         s1_rlast;
    wire         s1_rvalid;
    wire         s1_rready;

    // Slave 2 (DMA config — AXI-Lite bridged)
    wire [5:0]   s2_awid;
    wire [31:0]  s2_awaddr;
    wire [7:0]   s2_awlen;
    wire [2:0]   s2_awsize;
    wire [1:0]   s2_awburst;
    wire         s2_awvalid;
    wire         s2_awready;
    wire [127:0] s2_wdata;
    wire [15:0]  s2_wstrb;
    wire         s2_wlast;
    wire         s2_wvalid;
    wire         s2_wready;
    wire [5:0]   s2_bid;
    wire [1:0]   s2_bresp;
    wire         s2_bvalid;
    wire         s2_bready;
    wire [5:0]   s2_arid;
    wire [31:0]  s2_araddr;
    wire [7:0]   s2_arlen;
    wire [2:0]   s2_arsize;
    wire [1:0]   s2_arburst;
    wire         s2_arvalid;
    wire         s2_arready;
    wire [5:0]   s2_rid;
    wire [127:0] s2_rdata;
    wire [1:0]   s2_rresp;
    wire         s2_rlast;
    wire         s2_rvalid;
    wire         s2_rready;

    // Slave 3 (VRAM pixel aperture — URAM-backed)
    // Task #147: the xbar hands us a zero-based byte offset (VRAM_BASE
    // stripped) so we can tie this straight to the vram slave or (when
    // VIDEO_SMOKE=1) arbitrate with vram_smoke locally.
    wire [5:0]   s3_awid;
    wire [31:0]  s3_awaddr;
    wire [7:0]   s3_awlen;
    wire [2:0]   s3_awsize;
    wire [1:0]   s3_awburst;
    wire         s3_awvalid;
    wire         s3_awready;
    wire [127:0] s3_wdata;
    wire [15:0]  s3_wstrb;
    wire         s3_wlast;
    wire         s3_wvalid;
    wire         s3_wready;
    wire [5:0]   s3_bid;
    wire [1:0]   s3_bresp;
    wire         s3_bvalid;
    wire         s3_bready;
    wire [5:0]   s3_arid;
    wire [31:0]  s3_araddr;
    wire [7:0]   s3_arlen;
    wire [2:0]   s3_arsize;
    wire [1:0]   s3_arburst;
    wire         s3_arvalid;
    wire         s3_arready;
    wire [5:0]   s3_rid;
    wire [127:0] s3_rdata;
    wire [1:0]   s3_rresp;
    wire         s3_rlast;
    wire         s3_rvalid;
    wire         s3_rready;

    // Slave 4 (DAFB register shim — AXI-Lite bridged)
    wire [5:0]   s4_awid;
    wire [31:0]  s4_awaddr;
    wire [7:0]   s4_awlen;
    wire [2:0]   s4_awsize;
    wire [1:0]   s4_awburst;
    wire         s4_awvalid;
    wire         s4_awready;
    wire [127:0] s4_wdata;
    wire [15:0]  s4_wstrb;
    wire         s4_wlast;
    wire         s4_wvalid;
    wire         s4_wready;
    wire [5:0]   s4_bid;
    wire [1:0]   s4_bresp;
    wire         s4_bvalid;
    wire         s4_bready;
    wire [5:0]   s4_arid;
    wire [31:0]  s4_araddr;
    wire [7:0]   s4_arlen;
    wire [2:0]   s4_arsize;
    wire [1:0]   s4_arburst;
    wire         s4_arvalid;
    wire         s4_arready;
    wire [5:0]   s4_rid;
    wire [127:0] s4_rdata;
    wire [1:0]   s4_rresp;
    wire         s4_rlast;
    wire         s4_rvalid;
    wire         s4_rready;

    // Slave 5 (SD JTAG writer — AXI-Lite bridged)
    wire [5:0]   s5_awid;
    wire [31:0]  s5_awaddr;
    wire [7:0]   s5_awlen;
    wire [2:0]   s5_awsize;
    wire [1:0]   s5_awburst;
    wire         s5_awvalid;
    wire         s5_awready;
    wire [127:0] s5_wdata;
    wire [15:0]  s5_wstrb;
    wire         s5_wlast;
    wire         s5_wvalid;
    wire         s5_wready;
    wire [5:0]   s5_bid;
    wire [1:0]   s5_bresp;
    wire         s5_bvalid;
    wire         s5_bready;
    wire [5:0]   s5_arid;
    wire [31:0]  s5_araddr;
    wire [7:0]   s5_arlen;
    wire [2:0]   s5_arsize;
    wire [1:0]   s5_arburst;
    wire         s5_arvalid;
    wire         s5_arready;
    wire [5:0]   s5_rid;
    wire [127:0] s5_rdata;
    wire [1:0]   s5_rresp;
    wire         s5_rlast;
    wire         s5_rvalid;
    wire         s5_rready;

    wire [19:0]  dafb_axil_awaddr;
    wire [19:0]  dafb_axil_araddr;

    wire [19:0]  sdj_awaddr;
    wire         sdj_awvalid;
    wire         sdj_awready;
    wire [31:0]  sdj_wdata;
    wire [3:0]   sdj_wstrb;
    wire         sdj_wvalid;
    wire         sdj_wready;
    wire [1:0]   sdj_bresp;
    wire         sdj_bvalid;
    wire         sdj_bready;
    wire [19:0]  sdj_araddr;
    wire         sdj_arvalid;
    wire         sdj_arready;
    wire [31:0]  sdj_rdata;
    wire [1:0]   sdj_rresp;
    wire         sdj_rvalid;
    wire         sdj_rready;

    // task #269 (Level A): M2 idle tie-off, used ONLY when `L2C_ENABLE`
    // is defined -- in that config axi_i (ifa_*) binds directly to l2c's
    // dedicated fetch port instead (fpga_top_ddr.vh), closing the task
    // #266/SOC-1 open gap below, and the xbar's M2 slot goes unused.
    // Kept as a real (never-valid) master rather than removing the M2
    // port from this instantiation entirely: axi_xbar.v's own header
    // states its N_MASTERS/N_SLAVES parameters are "documentation only"
    // and its internal read/write fan-in arrays are FIXED-WIDTH local
    // params sized for a hardcoded round-robin picker, not generated off
    // those parameters -- so actually removing a master is a real
    // internal-arbiter restructuring of a 3890-line shared module, not a
    // mechanical port-list edit, and was deliberately NOT attempted here
    // (see the task #269 report).
    //
    // ══ SAFETY COUPLING TO THE CPU'S CONFIGURATION -- READ BEFORE CHANGING ══
    // This tie-off is not only a performance/width decision. Because axi_i does
    // NOT pass through this crossbar, and because this crossbar holds the ONLY
    // decode of the 0x5000_0000 peripheral window (axi_xbar.v:1249-1250,
    // `AXI_IO_BASE`/`AXI_IO_SIZE`), a speculative INSTRUCTION FETCH on a cpu040
    // build cannot reach a device register.  MC68040 UM 3.1.2/4: a cache-inhibited
    // read has an architecturally visible side effect (read-to-clear status, FIFO
    // pop, interrupt acknowledge) and must never be performed speculatively.
    //
    // cpu040 carries an I-side guard for exactly that hazard, and it is DISABLED
    // on this SoC via its `IcachePlugin(iFetchCanReachMmio = false)` default,
    // because here the hazard is unreachable and the guard sits in the fetch
    // command-accept path.
    //
    //   *** IF axi_i (ifa_*) IS EVER REROUTED THROUGH THIS CROSSBAR -- i.e. if the
    //   *** `else` branch below is taken on a cpu040 build (`L2C_ENABLE` undefined),
    //   *** or if a future master binding gives instruction fetch a path to
    //   *** XBAR_SLV_IO -- THEN cpu040 MUST BE BUILT WITH iFetchCanReachMmio = true.
    //
    // Today synth/vivado.tcl:1178 forces `L2C_ENABLE` on for every cpu040 build
    // (`$cpu_m68k040 || [info exists ::env(L2C_ENABLE)]`), so that branch is
    // unreachable for this core -- but that is one line away from changing.
    // See cpu040 docs/superpowers/specs/2026-09-05-p138c-fetch-gate-deadlock-fix.md.
    // (The D side is unaffected and needs no such note: axi_d IS this crossbar's
    // M0 master, so LsEuPlugin.p4LaunchOk / MmioCover guard it unconditionally.)
    wire [3:0]  m2_idle_arid    = 4'd0;
    wire [31:0] m2_idle_araddr  = 32'd0;
    wire [7:0]  m2_idle_arlen   = 8'd0;
    wire [2:0]  m2_idle_arsize  = 3'd0;
    wire [1:0]  m2_idle_arburst = 2'd0;
    wire        m2_idle_arvalid = 1'b0;
    wire        m2_idle_rready  = 1'b1;

    // ROM-overlay debug taps: driven by axi_xbar, consumed only by
    // fpga_top_debug_vio.vh (vio_boot_diag[24:23]).  Observation only.
    wire xbar_overlay_disabled;
    wire xbar_overlay_effective;
    // reset-wedge instrumentation -> vio_boot_diag[31:25]
    wire [5:0] xbar_slv_poisoned;
    wire       xbar_s1_slot_busy;

    axi_xbar #(
        // 2026-09-07 A/B: re-enabled with peripheral_bus's, same reasoning
        // (see fpga_top_peripherals.vh).  Set back to 0 once nothing starves.
        .ENABLE_WD (0),
        .DATA_WIDTH(128),
        .ID_WIDTH  (4),
        .N_MASTERS (3),
        .N_SLAVES  (6),
`ifdef VRAM_IN_DDR
        .S3_BACKEND_SURVIVES_FLUSH(1)
`else
        .S3_BACKEND_SURVIVES_FLUSH(0)
`endif
    ) u_xbar (
        // ── WHY THE CROSSBAR ITSELF IS STILL ON core_rst ──────────────
        // The old reason was wrong and is gone: "debug_ctrl is behind this
        // xbar and must accept the write that clears DBG_CONTROL[4]".  The
        // debug window is NOT behind this crossbar any more -- it is served
        // locally by axi_dbg_bus (fpga_top_debug_host.vh), which is exactly
        // what stopped a JTAG `reset` from stranding its own write.
        //
        // The remaining reason is S0.  `ddr_ctrl` and (with L2C_ENABLE)
        // `u_l2c` both sit on core_rst_bank[4] on purpose, to skip the
        // ~100 ms DDR re-calibration a soc_full_rst would force.  If this
        // instance moved to soc_full_rst_bank[4] while they did not, the
        // crossbar would forget an in-flight DDR transaction that the MIG
        // has NOT forgotten, and the late B/R would land on an idle slot --
        // the same hazard S3_BACKEND_SURVIVES_FLUSH exists to quarantine,
        // but for S0 and with no quarantine written.  Moving the whole
        // crossbar into soc_full_rst therefore needs that quarantine first;
        // it is the remaining piece of work item 3 and is NOT done.
        //
        // Every OTHER slave is already handled: S1/S2/S3/S4/S5 all reset on
        // the soc_full_rst event and are all in `is_flush_domain_slv()`, so
        // their in-flight transactions are flush-aborted with a clean local
        // SLVERR rather than abandoned.  S1 joined that set on 2026-09-12.
        .clk(core_clk), .rst(core_rst_bank[4]),
        .dbg_ram_window_lg2(dbg_ram_window_lg2),
        .cpu_overlay_active(via1_overlay_bit),
        // A 68040 RESET instruction re-arms the motherboard ROM overlay
        // without resetting the CPU or memory system.
        .cpu_overlay_reset(soc_full_rst_bank[4] | warm_peripheral_reset),
        // ROM-overlay observability -> vio_boot_diag[24:23] (2026-09-12).
        .cpu_overlay_disabled (xbar_overlay_disabled),
        .cpu_overlay_effective(xbar_overlay_effective),
        .dbg_slv_poisoned (xbar_slv_poisoned),
        .dbg_s1_slot_busy (xbar_s1_slot_busy),
        // M0/boot merge select — see axi_xbar.v header "M0/boot merge".
        // cpu_rst (fpga_top_clocks.vh) is exactly the CPU-held-in-reset
        // signal; boot_fsm only ever drives its AXI master while this is
        // asserted (boot_rom_loaded, which ungates cpu_rst, isn't raised
        // until boot_fsm's last write's BRESP lands with nothing else
        // outstanding — see boot_fsm.v ST_AXI_B_WAIT/ST_ZERO_B).
        .cpu_held_in_reset(cpu_rst),
        // slv_flush: the xbar instance itself is .rst(core_rst_bank[4])
        // above, but every slave BRIDGE below it is on the soc_full_rst
        // event -- the S2/S4/S5 axi_wide_to_axilite bridges on
        // soc_full_rst_bank[4], and (since 2026-09-12) the S1 CDC +
        // peripheral_bus on soc_full_rst_bank[1] / pb_soc_full_rst_bank[3].
        // Without this signal, a soc_full_rst with such a transaction in
        // flight makes the bridge forget it while the xbar keeps waiting:
        // the slot is stranded (measured: dbg_s1_slot_busy = 1 after a JTAG
        // `reset`) or, with ENABLE_WD=1, sticky-poisons a healthy slave.
        // Same event the bridges reset on -- see axi_xbar.v's slv_flush
        // port comment for the full contract.
        .slv_flush(soc_full_rst_bank[4]),
        // s1_far_reset: the pb island's THIRD reset source, which neither
        // core_rst nor soc_full_rst carries.  pb_full_rst (and therefore
        // every Mac peripheral behind peripheral_bus) is
        //     pb_rst || debug_full_reset_pb_sync || warm_peripheral_reset
        // and the last of those -- the 68040 RESET instruction, i.e. an
        // ordinary guest boot -- resets all of them while peripheral_bus
        // itself stays live on pb_soc_full_rst_bank[3] (deliberately, so the
        // still-running CPU keeps a front door).  Without this the crossbar
        // admits S1 traffic straight into a resetting island; four of
        // peripheral_bus's strobes are one-shot PULSES latched behind a
        // "kicked" flag only an ack can clear (the SCSI DMA-shim read and
        // write, and the ASC and ORWELL multi-byte write serialisers), so
        // one such pulse sticks rd_busy/wr_busy and takes S1 down for EVERY
        // master until core_rst -- with nothing to recover it, because
        // peripheral_bus's ENABLE_ACK_WATCHDOG defaults to 0 and is not
        // overridden below.  See the port's comment in axi_xbar.v.
        .s1_far_reset(warm_peripheral_reset),
        // m0b_master_reset: boot_fsm's own reset.  `boot_fsm_rst` is
        //     soc_full_rst || jtag_boot_bypass || dbg_cold_reset_hold
        // and the last two reach neither core_rst nor soc_full_rst, so a JTAG
        // DBG_CONTROL[4] hold resets boot_fsm out from under an in-flight
        // ROM-copy burst while cpu_held_in_reset stays 1 -- leaving
        // m0_abandon's mismatch term blind and the shared slot-0 port stuck
        // with S0/DDR parked mid-burst.  See the port's comment in axi_xbar.v,
        // and note u_boot_n2w below takes the same net for the same reason.
        .m0b_master_reset(boot_fsm_rst),

        // M0 — CPU data read/write (merged with boot FSM below via the
        // xbar's internal cpu_held_in_reset mux — the two are never live
        // at the same time, see axi_xbar.v header).
        .m0_awid   (cpu_d_awid   ), .m0_awaddr (cpu_d_awaddr ),
        .m0_awlen  (cpu_d_awlen  ), .m0_awsize (cpu_d_awsize ),
        .m0_awburst(cpu_d_awburst), .m0_awvalid(cpu_d_awvalid),
        .m0_awready(cpu_d_awready),
        .m0_wdata  (cpu_d_wdata  ), .m0_wstrb  (cpu_d_wstrb  ),
        .m0_wlast  (cpu_d_wlast  ), .m0_wvalid (cpu_d_wvalid ),
        .m0_wready (cpu_d_wready ),
        .m0_bid    (cpu_d_bid    ), .m0_bresp  (cpu_d_bresp  ),
        .m0_bvalid (cpu_d_bvalid ), .m0_bready (cpu_d_bready_xbar),
        .m0_arid   (cpu_d_arid   ), .m0_araddr (cpu_d_araddr ),
        .m0_arlen  (cpu_d_arlen  ), .m0_arsize (cpu_d_arsize ),
        .m0_arburst(cpu_d_arburst), .m0_arvalid(cpu_d_arvalid),
        .m0_arready(cpu_d_arready),
        .m0_rid    (cpu_d_rid    ), .m0_rdata  (cpu_d_rdata  ),
        .m0_rresp  (cpu_d_rresp  ), .m0_rlast  (cpu_d_rlast  ),
        .m0_rvalid (cpu_d_rvalid ), .m0_rready (cpu_d_rready_xbar),

        // M0b — boot FSM (write-only), time-multiplexed onto M0's
        // physical port above by cpu_held_in_reset.  boot_w_* wires come
        // from fpga_top_boot_master.vh's narrow-to-wide adapter feeding
        // boot_fsm.
        .m0b_awid   (boot_w_awid   ), .m0b_awaddr (boot_w_awaddr ),
        .m0b_awlen  (boot_w_awlen  ), .m0b_awsize (boot_w_awsize ),
        .m0b_awburst(boot_w_awburst), .m0b_awvalid(boot_w_awvalid),
        .m0b_awready(boot_w_awready),
        .m0b_wdata  (boot_w_wdata  ), .m0b_wstrb  (boot_w_wstrb  ),
        .m0b_wlast  (boot_w_wlast  ), .m0b_wvalid (boot_w_wvalid ),
        .m0b_wready (boot_w_wready ),
        .m0b_bid    (boot_w_bid    ), .m0b_bresp  (boot_w_bresp  ),
        .m0b_bvalid (boot_w_bvalid ), .m0b_bready (boot_w_bready ),

        // M1 — host debug (XDMA / JTAG-to-AXI)
        .m1_awid   (xdma_awid   ), .m1_awaddr (xdma_awaddr ),
        .m1_awlen  (xdma_awlen  ), .m1_awsize (xdma_awsize ),
        .m1_awburst(xdma_awburst), .m1_awvalid(xdma_awvalid),
        .m1_awready(xdma_awready),
        .m1_wdata  (xdma_wdata  ), .m1_wstrb  (xdma_wstrb  ),
        .m1_wlast  (xdma_wlast  ), .m1_wvalid (xdma_wvalid ),
        .m1_wready (xdma_wready ),
        .m1_bid    (xdma_bid    ), .m1_bresp  (xdma_bresp  ),
        .m1_bvalid (xdma_bvalid ), .m1_bready (xdma_bready ),
        .m1_arid   (xdma_arid   ), .m1_araddr (xdma_araddr ),
        .m1_arlen  (xdma_arlen  ), .m1_arsize (xdma_arsize ),
        .m1_arburst(xdma_arburst), .m1_arvalid(xdma_arvalid),
        .m1_arready(xdma_arready),
        .m1_rid    (xdma_rid    ), .m1_rdata  (xdma_rdata  ),
        .m1_rresp  (xdma_rresp  ), .m1_rlast  (xdma_rlast  ),
        .m1_rvalid (xdma_rvalid ), .m1_rready (xdma_rready ),

        // M2 — CPU instruction fetch, read-only.  Independent from CPU
        // LSU (M0) — never merge these two, per the maintainer.
        //
        // task #269 (Level A) CLOSES the task #266 / SOC-1 open gap that
        // used to live in this comment (this xbar's DATA_WIDTH=128 vs.
        // ifa_rdata's AXI_I_DW=256, an implicit zero-extend, not a real
        // 256b fetch path): when `L2C_ENABLE` is defined -- the shipping
        // default, Makefile `L2C_ENABLE ?= 1` -- ifa_* no longer binds
        // here at all.  It goes DIRECTLY to l2c's dedicated 256b fetch
        // port instead (fpga_top_ddr.vh), and M2 is tied permanently
        // idle (never valid) below.  See rtl/soc/l2c_ctrl.v's header and
        // the task #269 report for the full design.
        //
        // When `L2C_ENABLE` is UNDEFINED (opt-out build, no l2c instance
        // exists to route to) ifa_* falls back to this M2 port exactly
        // as before task #269 -- the width mismatch above is UNCHANGED
        // in that configuration; fixing it would mean widening
        // axi_xbar.v itself (out of scope, see the task #269 report).
        // Inert either way today: no CPU on this branch ever drives
        // axi_i_arvalid (cpu_stub ties it low).
`ifdef L2C_ENABLE
        .m2_arid   (m2_idle_arid   ), .m2_araddr (m2_idle_araddr ),
        .m2_arlen  (m2_idle_arlen  ), .m2_arsize (m2_idle_arsize ),
        .m2_arburst(m2_idle_arburst), .m2_arvalid(m2_idle_arvalid),
        .m2_arready(),
        .m2_rid    (), .m2_rdata  (),
        .m2_rresp  (), .m2_rlast  (),
        .m2_rvalid (), .m2_rready (m2_idle_rready),
`else
        .m2_arid   (ifa_arid   ), .m2_araddr (ifa_araddr ),
        .m2_arlen  (ifa_arlen  ), .m2_arsize (ifa_arsize ),
        .m2_arburst(ifa_arburst), .m2_arvalid(ifa_arvalid),
        .m2_arready(ifa_arready),
        .m2_rid    (ifa_rid    ), .m2_rdata  (ifa_rdata  ),
        .m2_rresp  (ifa_rresp  ), .m2_rlast  (ifa_rlast  ),
        .m2_rvalid (ifa_rvalid ), .m2_rready (ifa_rready_gated),
`endif

        // M3 — the shared DMA engine (u_dma_engine, fpga_top_dma.vh),
        // which holds this seat unconditionally since the DDR-backed
        // RAM-disk volume that used to share it was deleted (2026-09-10).
        // Reoccupies the fan-in slot dma_ctrl's stubbed M4 vacated.
        .m3_awid   (m3_awid   ), .m3_awaddr (m3_awaddr ),
        .m3_awlen  (m3_awlen  ), .m3_awsize (m3_awsize ),
        .m3_awburst(m3_awburst), .m3_awvalid(m3_awvalid),
        .m3_awready(m3_awready),
        .m3_wdata  (m3_wdata  ), .m3_wstrb  (m3_wstrb  ),
        .m3_wlast  (m3_wlast  ), .m3_wvalid (m3_wvalid ),
        .m3_wready (m3_wready ),
        .m3_bid    (m3_bid    ), .m3_bresp  (m3_bresp  ),
        .m3_bvalid (m3_bvalid ), .m3_bready (m3_bready ),
        .m3_arid   (m3_arid   ), .m3_araddr (m3_araddr ),
        .m3_arlen  (m3_arlen  ), .m3_arsize (m3_arsize ),
        .m3_arburst(m3_arburst), .m3_arvalid(m3_arvalid),
        .m3_arready(m3_arready),
        .m3_rid    (m3_rid    ), .m3_rdata  (m3_rdata  ),
        .m3_rresp  (m3_rresp  ), .m3_rlast  (m3_rlast  ),
        .m3_rvalid (m3_rvalid ), .m3_rready (m3_rready ),

        // S0 — DDR
        .s0_awid   (s0_awid   ), .s0_awaddr (s0_awaddr ),
        .s0_awlen  (s0_awlen  ), .s0_awsize (s0_awsize ),
        .s0_awburst(s0_awburst), .s0_awvalid(s0_awvalid),
        .s0_awready(s0_awready),
        .s0_wdata  (s0w_wdata ), .s0_wstrb  (s0w_wstrb ),
        .s0_wlast  (s0w_wlast ), .s0_wvalid (s0w_wvalid),
        .s0_wready (s0w_wready),
        .s0_bid    (s0_bid    ), .s0_bresp  (s0_bresp  ),
        .s0_bvalid (s0_bvalid ), .s0_bready (s0_bready ),
        .s0_arid   (s0_arid   ), .s0_araddr (s0_araddr ),
        .s0_arlen  (s0_arlen  ), .s0_arsize (s0_arsize ),
        .s0_arburst(s0_arburst), .s0_arvalid(s0_arvalid),
        .s0_arready(s0_arready),
        .s0_rid    (s0_rid    ), .s0_rdata  (s0_rdata  ),
        .s0_rresp  (s0_rresp  ), .s0_rlast  (s0_rlast  ),
        .s0_rvalid (s0_rvalid ), .s0_rready (s0_rready ),

        // S1 — peripheral bus
        .s1_awid   (s1_awid   ), .s1_awaddr (s1_awaddr ),
        .s1_awlen  (s1_awlen  ), .s1_awsize (s1_awsize ),
        .s1_awburst(s1_awburst), .s1_awvalid(s1_awvalid),
        .s1_awready(s1_awready),
        .s1_wdata  (s1_wdata  ), .s1_wstrb  (s1_wstrb  ),
        .s1_wlast  (s1_wlast  ), .s1_wvalid (s1_wvalid ),
        .s1_wready (s1_wready ),
        .s1_bid    (s1_bid    ), .s1_bresp  (s1_bresp  ),
        .s1_bvalid (s1_bvalid ), .s1_bready (s1_bready ),
        .s1_arid   (s1_arid   ), .s1_araddr (s1_araddr ),
        .s1_arlen  (s1_arlen  ), .s1_arsize (s1_arsize ),
        .s1_arburst(s1_arburst), .s1_arvalid(s1_arvalid),
        .s1_arready(s1_arready),
        .s1_rid    (s1_rid    ), .s1_rdata  (s1_rdata  ),
        .s1_rresp  (s1_rresp  ), .s1_rlast  (s1_rlast  ),
        .s1_rvalid (s1_rvalid ), .s1_rready (s1_rready ),

        // S2 — DMA config (AXI-Lite bridged by axi_wide_to_axilite)
        .s2_awid   (s2_awid   ), .s2_awaddr (s2_awaddr ),
        .s2_awlen  (s2_awlen  ), .s2_awsize (s2_awsize ),
        .s2_awburst(s2_awburst), .s2_awvalid(s2_awvalid),
        .s2_awready(s2_awready),
        .s2_wdata  (s2_wdata  ), .s2_wstrb  (s2_wstrb  ),
        .s2_wlast  (s2_wlast  ), .s2_wvalid (s2_wvalid ),
        .s2_wready (s2_wready ),
        .s2_bid    (s2_bid    ), .s2_bresp  (s2_bresp  ),
        .s2_bvalid (s2_bvalid ), .s2_bready (s2_bready ),
        .s2_arid   (s2_arid   ), .s2_araddr (s2_araddr ),
        .s2_arlen  (s2_arlen  ), .s2_arsize (s2_arsize ),
        .s2_arburst(s2_arburst), .s2_arvalid(s2_arvalid),
        .s2_arready(s2_arready),
        .s2_rid    (s2_rid    ), .s2_rdata  (s2_rdata  ),
        .s2_rresp  (s2_rresp  ), .s2_rlast  (s2_rlast  ),
        .s2_rvalid (s2_rvalid ), .s2_rready (s2_rready ),

        // S3 — VRAM pixel aperture (URAM-backed `vram`), task #147.
        // xbar already strips VRAM_BASE from s3_awaddr / s3_araddr
        // before this port is driven; the slave sees a zero-based
        // byte offset into its framebuffer.
        .s3_awid   (s3_awid   ), .s3_awaddr (s3_awaddr ),
        .s3_awlen  (s3_awlen  ), .s3_awsize (s3_awsize ),
        .s3_awburst(s3_awburst), .s3_awvalid(s3_awvalid),
        .s3_awready(s3_awready),
        .s3_wdata  (s3_wdata  ), .s3_wstrb  (s3_wstrb  ),
        .s3_wlast  (s3_wlast  ), .s3_wvalid (s3_wvalid ),
        .s3_wready (s3_wready ),
        .s3_bid    (s3_bid    ), .s3_bresp  (s3_bresp  ),
        .s3_bvalid (s3_bvalid ), .s3_bready (s3_bready ),
        .s3_arid   (s3_arid   ), .s3_araddr (s3_araddr ),
        .s3_arlen  (s3_arlen  ), .s3_arsize (s3_arsize ),
        .s3_arburst(s3_arburst), .s3_arvalid(s3_arvalid),
        .s3_arready(s3_arready),
        .s3_rid    (s3_rid    ), .s3_rdata  (s3_rdata  ),
        .s3_rresp  (s3_rresp  ), .s3_rlast  (s3_rlast  ),
        .s3_rvalid (s3_rvalid ), .s3_rready (s3_rready ),

        // S4 — DAFB register shim. xbar strips AXI_DAFB_BASE so the
        // bridge/shim see local register offsets.
        .s4_awid   (s4_awid   ), .s4_awaddr (s4_awaddr ),
        .s4_awlen  (s4_awlen  ), .s4_awsize (s4_awsize ),
        .s4_awburst(s4_awburst), .s4_awvalid(s4_awvalid),
        .s4_awready(s4_awready),
        .s4_wdata  (s4_wdata  ), .s4_wstrb  (s4_wstrb  ),
        .s4_wlast  (s4_wlast  ), .s4_wvalid (s4_wvalid ),
        .s4_wready (s4_wready ),
        .s4_bid    (s4_bid    ), .s4_bresp  (s4_bresp  ),
        .s4_bvalid (s4_bvalid ), .s4_bready (s4_bready ),
        .s4_arid   (s4_arid   ), .s4_araddr (s4_araddr ),
        .s4_arlen  (s4_arlen  ), .s4_arsize (s4_arsize ),
        .s4_arburst(s4_arburst), .s4_arvalid(s4_arvalid),
        .s4_arready(s4_arready),
        .s4_rid    (s4_rid    ), .s4_rdata  (s4_rdata  ),
        .s4_rresp  (s4_rresp  ), .s4_rlast  (s4_rlast  ),
        .s4_rvalid (s4_rvalid ), .s4_rready (s4_rready ),

        // S5 — SD JTAG writer. xbar strips AXI_SD_JTAG_BASE so the
        // bridge/writer see local register offsets.
        .s5_awid   (s5_awid   ), .s5_awaddr (s5_awaddr ),
        .s5_awlen  (s5_awlen  ), .s5_awsize (s5_awsize ),
        .s5_awburst(s5_awburst), .s5_awvalid(s5_awvalid),
        .s5_awready(s5_awready),
        .s5_wdata  (s5_wdata  ), .s5_wstrb  (s5_wstrb  ),
        .s5_wlast  (s5_wlast  ), .s5_wvalid (s5_wvalid ),
        .s5_wready (s5_wready ),
        .s5_bid    (s5_bid    ), .s5_bresp  (s5_bresp  ),
        .s5_bvalid (s5_bvalid ), .s5_bready (s5_bready ),
        .s5_arid   (s5_arid   ), .s5_araddr (s5_araddr ),
        .s5_arlen  (s5_arlen  ), .s5_arsize (s5_arsize ),
        .s5_arburst(s5_arburst), .s5_arvalid(s5_arvalid),
        .s5_arready(s5_arready),
        .s5_rid    (s5_rid    ), .s5_rdata  (s5_rdata  ),
        .s5_rresp  (s5_rresp  ), .s5_rlast  (s5_rlast  ),
        .s5_rvalid (s5_rvalid ), .s5_rready (s5_rready )
    );

    // ═══════════════════════════════════════════════════════════════════
    // S0 write-data register slice  (2026-09-12, FMax)
    // ═══════════════════════════════════════════════════════════════════
    // The DDR slave's WREADY used to be consumed COMBINATIONALLY by the
    // crossbar's per-master write FSMs (axi_xbar.v `gen_wready`), while
    // that same FSM's state combinationally selected the WDATA/WSTRB/
    // WVALID it presented (`s?_wowner`).  With `l2c_ctrl.v` deriving
    // `s_axi_wready` from `accept_slot_c` -- which contains `q_adv_c`, the
    // whole stage-2 resolve outcome -- that closed one 20-to-24-level
    // combinational path from one crossbar slot's `ws_state` to another
    // slot's, straight through the L2 front door.  On the routed CPU-less
    // 200 MHz measurement those two cones were the design's WNS (-0.624 ns)
    // and ~1000 more failing endpoints; see axi_w_skid.v's header for the
    // measured paths.
    //
    // This slice cuts them apart: the crossbar now sees a registered
    // "there is room" bit, and the slave sees registered W payload.  One
    // cycle of W latency, no throughput change (see the module header).
    //
    // Reset MUST match both endpoints.  The crossbar, u_l2c and u_ddr are
    // all core_rst_bank[4] on purpose (see the big note on u_xbar's .rst
    // above); the slice joins them, so a reset can never leave the
    // crossbar counting a beat the slice has thrown away.
    //
    // Only S0 is sliced.  The 2026-09-12 measurement showed every other
    // slave port at zero failing endpoints -- the peripheral bus, VRAM,
    // DAFB and SD/JTAG slaves all answer WREADY out of shallow logic -- so
    // slicing them would have bought latency for nothing.
    //
    // ⚠️ AMENDED 2026-09-16.  That "zero" was about the READY direction and
    // about the masters that existed then.  On build/vivado200_fmax2 the
    // DMA engine's W BROADCAST owned 41 failing endpoints landing in S1
    // (u_pb_s1_cdc's FIFO), S3 (the DDR-aperture lane into the MIG UI FIFO)
    // and the vhdd controller -- the FORWARD direction, and none of them
    // through this slice, because the DMA does not reach DDR via S0/L2C.
    // That family is cut at its source instead: dma_engine.sv now carries
    // its own axi_w_skid on its master port, so this S0 slice still stands
    // alone on the slave side.
    axi_w_skid #(
        .DATA_WIDTH(128),
        .STRB_WIDTH(16)
    ) u_s0_w_slice (
        .clk(core_clk), .rst(core_rst_bank[4]),
        .s_wdata (s0w_wdata ), .s_wstrb (s0w_wstrb ),
        .s_wlast (s0w_wlast ), .s_wvalid(s0w_wvalid),
        .s_wready(s0w_wready),
        .m_wdata (s0_wdata  ), .m_wstrb (s0_wstrb  ),
        .m_wlast (s0_wlast  ), .m_wvalid(s0_wvalid ),
        .m_wready(s0_wready )
    );

    // xbar S2 (128-bit AXI4) → dma_ctrl.cfg_* (AXI-Lite)
    // Bank bit [4] (xbar / DDR arbiter region).
    axi_wide_to_axilite #(.ID_WIDTH(6), .DATA_WIDTH(128)) u_dma_s2_bridge (
        .clk(core_clk), .rst(soc_full_rst_bank[4]),
        .s_awid   (s2_awid   ), .s_awaddr (s2_awaddr ),
        .s_awlen  (s2_awlen  ), .s_awsize (s2_awsize ),
        .s_awburst(s2_awburst), .s_awvalid(s2_awvalid),
        .s_awready(s2_awready),
        .s_wdata  (s2_wdata  ), .s_wstrb  (s2_wstrb  ),
        .s_wlast  (s2_wlast  ), .s_wvalid (s2_wvalid ),
        .s_wready (s2_wready ),
        .s_bid    (s2_bid    ), .s_bresp  (s2_bresp  ),
        .s_bvalid (s2_bvalid ), .s_bready (s2_bready ),
        .s_arid   (s2_arid   ), .s_araddr (s2_araddr ),
        .s_arlen  (s2_arlen  ), .s_arsize (s2_arsize ),
        .s_arburst(s2_arburst), .s_arvalid(s2_arvalid),
        .s_arready(s2_arready),
        .s_rid    (s2_rid    ), .s_rdata  (s2_rdata  ),
        .s_rresp  (s2_rresp  ), .s_rlast  (s2_rlast  ),
        .s_rvalid (s2_rvalid ), .s_rready (s2_rready ),
        .l_awaddr (dma_cfg_awaddr ), .l_awvalid(dma_cfg_awvalid),
        .l_awready(dma_cfg_awready),
        .l_wdata  (dma_cfg_wdata  ), .l_wstrb  (dma_cfg_wstrb  ),
        .l_wvalid (dma_cfg_wvalid ), .l_wready (dma_cfg_wready ),
        .l_bresp  (dma_cfg_bresp  ), .l_bvalid (dma_cfg_bvalid ),
        .l_bready (dma_cfg_bready ),
        .l_araddr (dma_cfg_araddr ), .l_arvalid(dma_cfg_arvalid),
        .l_arready(dma_cfg_arready),
        .l_rdata  (dma_cfg_rdata  ), .l_rresp  (dma_cfg_rresp  ),
        .l_rvalid (dma_cfg_rvalid ), .l_rready (dma_cfg_rready )
    );

    // xbar S4 (128-bit AXI4) -> DAFB register shim (AXI-Lite)
    // Bank bit [4] (xbar / DDR arbiter region).
    axi_wide_to_axilite #(.ID_WIDTH(6), .DATA_WIDTH(128)) u_dafb_s4_bridge (
        .clk(core_clk), .rst(soc_full_rst_bank[4]),
        .s_awid   (s4_awid   ), .s_awaddr (s4_awaddr ),
        .s_awlen  (s4_awlen  ), .s_awsize (s4_awsize ),
        .s_awburst(s4_awburst), .s_awvalid(s4_awvalid),
        .s_awready(s4_awready),
        .s_wdata  (s4_wdata  ), .s_wstrb  (s4_wstrb  ),
        .s_wlast  (s4_wlast  ), .s_wvalid (s4_wvalid ),
        .s_wready (s4_wready ),
        .s_bid    (s4_bid    ), .s_bresp  (s4_bresp  ),
        .s_bvalid (s4_bvalid ), .s_bready (s4_bready ),
        .s_arid   (s4_arid   ), .s_araddr (s4_araddr ),
        .s_arlen  (s4_arlen  ), .s_arsize (s4_arsize ),
        .s_arburst(s4_arburst), .s_arvalid(s4_arvalid),
        .s_arready(s4_arready),
        .s_rid    (s4_rid    ), .s_rdata  (s4_rdata  ),
        .s_rresp  (s4_rresp  ), .s_rlast  (s4_rlast  ),
        .s_rvalid (s4_rvalid ), .s_rready (s4_rready ),
        .l_awaddr (dafb_axil_awaddr), .l_awvalid(dafb_awvalid),
        .l_awready(dafb_awready),
        .l_wdata  (dafb_wdata  ), .l_wstrb  (dafb_wstrb  ),
        .l_wvalid (dafb_wvalid ), .l_wready (dafb_wready ),
        .l_bresp  (dafb_bresp  ), .l_bvalid (dafb_bvalid ),
        .l_bready (dafb_bready ),
        .l_araddr (dafb_axil_araddr), .l_arvalid(dafb_arvalid),
        .l_arready(dafb_arready),
        .l_rdata  (dafb_rdata  ), .l_rresp  (dafb_rresp  ),
        .l_rvalid (dafb_rvalid ), .l_rready (dafb_rready )
    );

    assign dafb_awaddr = {12'h000, dafb_axil_awaddr};
    assign dafb_araddr = {12'h000, dafb_axil_araddr};

    // xbar S5 (128-bit AXI4) -> SD JTAG writer (AXI-Lite)
    // Bank bit [4] (xbar / DDR arbiter region).
    axi_wide_to_axilite #(.ID_WIDTH(6), .DATA_WIDTH(128)) u_sdj_s5_bridge (
        .clk(core_clk), .rst(soc_full_rst_bank[4]),
        .s_awid   (s5_awid   ), .s_awaddr (s5_awaddr ),
        .s_awlen  (s5_awlen  ), .s_awsize (s5_awsize ),
        .s_awburst(s5_awburst), .s_awvalid(s5_awvalid),
        .s_awready(s5_awready),
        .s_wdata  (s5_wdata  ), .s_wstrb  (s5_wstrb  ),
        .s_wlast  (s5_wlast  ), .s_wvalid (s5_wvalid ),
        .s_wready (s5_wready ),
        .s_bid    (s5_bid    ), .s_bresp  (s5_bresp  ),
        .s_bvalid (s5_bvalid ), .s_bready (s5_bready ),
        .s_arid   (s5_arid   ), .s_araddr (s5_araddr ),
        .s_arlen  (s5_arlen  ), .s_arsize (s5_arsize ),
        .s_arburst(s5_arburst), .s_arvalid(s5_arvalid),
        .s_arready(s5_arready),
        .s_rid    (s5_rid    ), .s_rdata  (s5_rdata  ),
        .s_rresp  (s5_rresp  ), .s_rlast  (s5_rlast  ),
        .s_rvalid (s5_rvalid ), .s_rready (s5_rready ),
        .l_awaddr (sdj_awaddr ), .l_awvalid(sdj_awvalid),
        .l_awready(sdj_awready),
        .l_wdata  (sdj_wdata  ), .l_wstrb  (sdj_wstrb  ),
        .l_wvalid (sdj_wvalid ), .l_wready (sdj_wready ),
        .l_bresp  (sdj_bresp  ), .l_bvalid (sdj_bvalid ),
        .l_bready (sdj_bready ),
        .l_araddr (sdj_araddr ), .l_arvalid(sdj_arvalid),
        .l_arready(sdj_arready),
        .l_rdata  (sdj_rdata  ), .l_rresp  (sdj_rresp  ),
        .l_rvalid (sdj_rvalid ), .l_rready (sdj_rready )
    );
