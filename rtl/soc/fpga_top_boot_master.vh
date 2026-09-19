// rtl/fpga_top_boot_master.vh — included from rtl/fpga_top.v
//
// boot_fsm master (xbar m0b, time-multiplexed onto M0 — CPU LSU): boot
// FSM instance + 32-bit narrow-to-128 AXI adapter.  Was a standalone
// xbar M2 seat before the 2026-07-16 master-count reduction merged it
// onto M0 via a cpu_rst-selected mux inside axi_xbar.v (see its header
// note "M0/boot merge"); CPU IF took the freed M2 slot instead.
//
// Generated mechanically from rtl/fpga_top.v during the fpga_top split
// (agent/p3-fpga-top-split). No functional change: this file contains
// the original lines verbatim, inside the fpga_top module scope via
// `include. Do not add module/endmodule wrappers or nettype directives.

    // ═══════════════════════════════════════════════════════════════════
    // boot_fsm master (m0b, merged onto M0) — 32-bit narrow, adapted to 128-bit
    // ═══════════════════════════════════════════════════════════════════
    wire        boot_rom_loading;
    wire        boot_error;
    // Real-HW CRC-retry diagnosis taps (see boot_fsm.v's dbg_sector /
    // dbg_err_cause / dbg_ctrl_retry comments) — wired to a VIO probe in
    // fpga_top_debug_vio.vh (vio_boot_diag) instead of left unused, so a
    // stuck boot can be root-caused without a fresh rebuild every time.
    wire [15:0] boot_dbg_sector;
    wire [2:0]  boot_dbg_err_cause;
    wire [3:0]  boot_dbg_ctrl_retry;
    wire [15:0] boot_dbg_rd_crc_calc;
    wire [15:0] boot_dbg_rd_crc_recv;
    // 1 once boot_fsm's own sd_ctrl (CMD18 ROM bulk-load) has enabled
    // SD-card CRC16 checking via CMD59 during init.  Wired into BOTH
    // sd_ctrl instances' crc_check_en (this one and the runtime SCSI
    // sd_ctrl_scsi instance in fpga_top_sd.vh) — see sd_ctrl.v's
    // crc_check_en port comment and boot_fsm.v's sd_crc_enabled comment.
    wire        boot_sd_crc_enabled;
    // Card size in sectors, decoded from the SD CSD during init.  The SCSI
    // target's reported capacity is derived from this minus the reserved
    // ROM window -- never hardcoded (see scsi.v disk_num_lbas comment).
    wire [31:0] boot_card_num_lbas;

    wire        boot_spi_cmd_valid;
    wire        boot_spi_cmd_ready;
    wire [7:0]  boot_spi_cmd_data;
    wire        boot_spi_rsp_valid;
    wire [7:0]  boot_spi_rsp_data;
    wire        boot_spi_cs_n;
    wire        boot_spi_fast;
    wire        boot_spi_hs;

    wire [3:0]  boot_m_awid;
    wire [31:0] boot_m_awaddr;
    wire [7:0]  boot_m_awlen;
    wire [2:0]  boot_m_awsize;
    wire [1:0]  boot_m_awburst;
    wire        boot_m_awvalid;
    wire        boot_m_awready;
    wire [31:0] boot_m_wdata;
    wire [3:0]  boot_m_wstrb;
    wire        boot_m_wlast;
    wire        boot_m_wvalid;
    wire        boot_m_wready;
    // boot_m_bid is an INPUT to boot_fsm — drive it from a constant 0.
    wire [3:0]  boot_m_bid = 4'd0;
    wire [1:0]  boot_m_bresp;
    wire        boot_m_bvalid;
    wire        boot_m_bready;

    // ZERO_BYTES: pre-zero the whole RAM WINDOW, not boot_fsm's 4 MiB default.
    //
    // A platform reset (btn[3] or `vio-hard-reset`, both via
    // platform_resetn -> u_clk_rst.rst_in -> soc_full_rst -> boot_fsm_rst)
    // is the equivalent of a power cycle and MUST present to the ROM as a
    // COLD boot.  On real hardware, power-on RAM is indeterminate so the
    // warm-start marker `WmStFlag` ('WLSC' at low-mem 0xCFC) cannot match and
    // the ROM runs its full RAM sizing/test.  Only a software restart is
    // supposed to warm-start.
    //
    // With boot_fsm's 4 MiB default on an 8 MiB (or larger) machine, RAM
    // above 0x0040_0000 kept the PREVIOUS boot's contents across a hard
    // reset.  That region is where the OS actually lives -- measured on this
    // board: ApplZone 0x00797920, the Finder's sub-zone 0x007af380, its
    // menu-command tables at 0x007af7ac / 0x007b4e00.  Structured ghosts of a
    // near-identical previous boot are far more dangerous than random
    // garbage: any "is this already installed / already valid?" probe that
    // checks a signature or a plausible pointer FALSE-POSITIVES on them,
    // where true power-on garbage would fail the check.  It also makes the
    // machine non-reproducible boot-to-boot and non-comparable against MAME,
    // which cold-starts from a deterministic fill.
    //
    // Sized to the RAM window rather than the installed SIMM size so the
    // guarantee holds for every DDR-backed configuration.
    //
    // COST, and where it actually comes from (measured 2026-08-19):
    //
    // boot_fsm now issues the pass as 256-beat (1 KiB) 32-bit INCR bursts
    // with WVALID held back to back -- 1.016 cycles/word against an ideal
    // slave, i.e. 0.68 s for 256 MiB at 100 MHz (tb-sd-boot-zero prints the
    // number on every run).  That is NOT what the pass costs on hardware,
    // because boot_fsm is not the bottleneck: L2C is.
    //
    // With L2C_ENABLE=1 (the shipping default since 2026-08-03) every write
    // miss in l2c_ctrl is a READ-ALLOCATE.  Zeroing 256 MiB touches 4 Mi
    // 64-byte lines, none of them resident, so the pass drags a full line
    // in from DDR before overwriting all of it with zeros, then writes the
    // dirty line back.  Measured on the real chain (axi_narrow_to_wide ->
    // l2c -> async bridge -> MIG bridge -> sim MIG, DDR read latency 40):
    //
    //     narrow AWLEN        15 (old)     63       255     L2C bypassed
    //     cycles/word           5.19       5.05      5.01    1.25 (n2w cap)
    //
    // i.e. burst length buys ~3% while L2C is in the path, and ~4x once it
    // is not.  The lever that matters is a full-line-write no-fetch path in
    // l2c_ctrl / l2c_mshr (a write burst covering a whole 64 B line with all
    // strobes set does not need the fill).  Until that lands, expect the
    // pass to stay in the 3-4 s range regardless of what this master does.
    //
    // ── Cold vs warm boot: gate the 256 MiB RAM pre-zero pass ─────────
    // MEASURED 2026-08-19: the zero pass IS the boot time.  Five
    // reset-to-rom_loaded cycles came in at 4.060/4.061/4.062/4.061/4.062 s
    // (+/-2 ms, zero CRC retries, all 2048 sectors), and the SD ROM copy is
    // only ~0.13-0.34 s of that.  256 MiB at ~63 MB/s is the rest -- and
    // per the COST note above that rate is set by L2C's read-allocate on
    // write miss, not by the 32-bit width of this master's port.
    //
    // The pass exists so a reset presents as a GENUINE cold boot rather than
    // leaving the previous session's RAM lying around for the ROM to find.
    // That is worth 4 s on a real cold reset and pure waste on the warm
    // resets used to iterate, so decide per boot instead of at compile time.
    //
    // How cold is detected -- no new control bit, because the two resets
    // already differ at the source (rtl/board/clk_rst.v):
    //   core_rst      <= rst_req                       (power-on, btn3,
    //                                                   vio-hard-reset)
    //   soc_full_rst  <= rst_req | dbg_full_rst_in     (the above, PLUS the
    //                                                   JTAG debug-full
    //                                                   reset)
    // clk_rst.v's own header says core_rst is "board cold reset only -- does
    // NOT" include the debug request.  So a flag reset by core_rst and SET
    // by the debug-full reset reads as: 0 = this boot follows a cold reset,
    // 1 = it follows a JTAG one.
    //
    // Consequence to keep in mind: a warm reset now boots on the PREVIOUS
    // session's RAM.  That is the point (it is what buys the 4 s), but it
    // means a warm reset is NOT a valid way to reproduce a cold-boot bug --
    // use the cold path when the RAM contents could matter.
    reg  [1:0] boot_dbgrst_sync;
    reg        boot_warm_q;
    always @(posedge core_clk) begin
        if (core_rst_bank[6]) begin
            boot_dbgrst_sync <= 2'b00;
            boot_warm_q      <= 1'b0;          // cold: run the zero pass
        end else begin
            // jtag_debug_full_reset_eff is generated in the sys_clk domain
            // (see fpga_top_clocks.vh); two FFs to land it in core_clk.
            boot_dbgrst_sync <= {boot_dbgrst_sync[0], jtag_debug_full_reset_eff};
            if (boot_dbgrst_sync[1])
                boot_warm_q <= 1'b1;           // warm: skip it
        end
    end
    wire boot_zero_en = ~boot_warm_q;

    boot_fsm #(
        .NUM_SECTORS(BOOT_ROM_SECTORS),
`ifdef FPGA_ROM_SIM_REALBOOT
        // realboot harness (tb-fpga-top-rom-realboot): the RTL always
        // zeros the FULL 256 MiB RAM window on every reset for real
        // production correctness (see the comment above) -- ~1M AXI
        // write bursts, tens of minutes of Verilator wall-clock, all
        // BEFORE any CPU instruction ever runs. This harness's whole
        // point is to observe boot_fsm's real completion gating plus
        // the first few thousand genuine post-boot CPU instructions,
        // NOT to re-verify the zero pass itself (already covered by
        // tb-sd-boot-zero and friends) -- shrink the window so the
        // realboot smoke run stays a few minutes of sim wall-clock
        // instead of tens of minutes. Sim-only: unused on the real
        // FPGA path and unused by the default tb-fpga-top-rom target,
        // both of which never define FPGA_ROM_SIM_REALBOOT.
        .ZERO_BYTES (32'h0010_0000),     // 1 MiB RAM window (realboot smoke only)
`else
        .ZERO_BYTES (32'h1000_0000),     // 256 MiB RAM window
`endif
`ifdef CPU_M68K040
        // See boot_fsm.v's MIRROR_LOW_RAM parameter comment: cpu040's
        // axi_i fetch bypasses the crossbar's ROM-overlay redirect
        // (Level A / task #269), so the boot loader mirrors the ROM
        // image into low RAM directly instead of relying on a runtime
        // redirect no single point is left to reliably trigger.
        // WORD_FIFO_LOG2P doubled to keep the same drain-vs-ingest
        // margin the word FIFO always had, now that MIRROR_LOW_RAM
        // pushes two entries per assembled word instead of one.
        .MIRROR_LOW_RAM(1'b1),
        .WORD_FIFO_LOG2P(6),
        // Deterministic (black) framebuffer at reset instead of
        // whatever DRAM garbage powered up, so a boot that never gets
        // far enough to paint doesn't show visual noise that reads as
        // a display fault.
        .VRAM_ZERO_BASE (`AXI_VRAM_BASE),
        .VRAM_ZERO_BYTES(`AXI_VRAM_SIZE),
`endif
        .AXI_ID(4'd0)
    ) u_boot_fsm (
        .clk(core_clk), .rst(boot_fsm_rst),
        .zero_en(boot_zero_en),

        // Diagnostic-only overrides (see boot_fsm.v's port comment) —
        // tied off so production behavior is unchanged: CMD59 is sent,
        // HS mode never engages (9521f93). Only fpga_top_sdmin's
        // standalone SD/CRC test harness drives these from VIO.
        .cfg_skip_cmd59(1'b0),
        .cfg_force_hs  (1'b0),

        .spi_cmd_valid(boot_spi_cmd_valid),
        .spi_cmd_ready(boot_spi_cmd_ready),
        .spi_cmd_data (boot_spi_cmd_data ),
        .spi_rsp_valid(boot_spi_rsp_valid),
        .spi_rsp_data (boot_spi_rsp_data ),
        .spi_cs_n     (boot_spi_cs_n     ),
        .spi_fast_mode(boot_spi_fast     ),
        .spi_hs_mode  (boot_spi_hs       ),

        .m_axi_awid   (boot_m_awid   ),
        .m_axi_awaddr (boot_m_awaddr ),
        .m_axi_awlen  (boot_m_awlen  ),
        .m_axi_awsize (boot_m_awsize ),
        .m_axi_awburst(boot_m_awburst),
        .m_axi_awvalid(boot_m_awvalid),
        .m_axi_awready(boot_m_awready),
        .m_axi_wdata  (boot_m_wdata  ),
        .m_axi_wstrb  (boot_m_wstrb  ),
        .m_axi_wlast  (boot_m_wlast  ),
        .m_axi_wvalid (boot_m_wvalid ),
        .m_axi_wready (boot_m_wready ),
        .m_axi_bid    (boot_m_bid    ),
        .m_axi_bresp  (boot_m_bresp  ),
        .m_axi_bvalid (boot_m_bvalid ),
        .m_axi_bready (boot_m_bready ),

        .rom_loading(boot_rom_loading),
        .rom_loaded (boot_rom_loaded ),
        .error      (boot_error      ),
        .sd_crc_enabled(boot_sd_crc_enabled),
        .card_num_lbas (boot_card_num_lbas),

        .dbg_st        (/* unused */),
        .dbg_cur_cmd   (/* unused */),
        .dbg_last_r1   (/* unused */),
        .dbg_last_rx   (/* unused */),
        .dbg_acmd41_try(/* unused */),
        .dbg_rsp_count (/* unused */),
        .dbg_sector    (boot_dbg_sector),
        // SDHC class + OCR byte exposed via the boot_fsm dbg_* taps.
        // Bind names but leave routing to a future VIO probe slot.
        .dbg_is_sdhc   (/* unused */),
        .dbg_ocr0      (/* unused */),
        .dbg_err_cause (boot_dbg_err_cause),
        .dbg_ctrl_retry(boot_dbg_ctrl_retry),
        .dbg_rd_crc_calc(boot_dbg_rd_crc_calc),
        .dbg_rd_crc_recv(boot_dbg_rd_crc_recv),
        .dbg_ctrl_err_cause(/* unused */),
        .dbg_ctrl_last_real_r1(/* unused */),
        .dbg_sdctrl_cur_cmd(/* unused */),
        .dbg_sdctrl_lba_lat(/* unused */),
        .dbg_sdctrl_last_crc7_sent(/* unused */),
        .dbg_sdctrl_last_poll_cnt(/* unused */),
        .dbg_attempt_log0(/* unused */),
        .dbg_attempt_log1(/* unused */),
        .dbg_attempt_log2(/* unused */),
        .dbg_attempt_log3(/* unused */),
        .dbg_attempt_log4(/* unused */),
        .dbg_attempt_log5(/* unused */)
    );

    // Adapter — boot_fsm emits 32-bit single-beat writes for the first-light
    // ROM copy.  The narrow-to-wide adapter expands each word into the
    // 128-bit xbar contract; general burst support is tracked separately.
    wire [3:0]   boot_w_awid;
    wire [31:0]  boot_w_awaddr;
    wire [7:0]   boot_w_awlen;
    wire [2:0]   boot_w_awsize;
    wire [1:0]   boot_w_awburst;
    wire         boot_w_awvalid;
    wire         boot_w_awready;
    wire [127:0] boot_w_wdata;
    wire [15:0]  boot_w_wstrb;
    wire         boot_w_wlast;
    wire         boot_w_wvalid;
    wire         boot_w_wready;
    wire [3:0]   boot_w_bid;
    wire [1:0]   boot_w_bresp;
    wire         boot_w_bvalid;
    wire         boot_w_bready;

    axi_narrow_to_wide #(
        .ID_WIDTH(4),
        .ID_TAG  (4'd2)
    ) u_boot_n2w (
        // ⚠ CORRECTED 2026-09-18 (race audit): this MUST be the same event
        // that resets its own master.  It used to be soc_full_rst_bank[6],
        // "drains the boot-master narrow→wide bridge so a debug-full-reset
        // starts the next ROM copy with empty queues" -- which is true of a
        // debug-full-reset and false of the other two terms of
        //
        //     boot_fsm_rst = soc_full_rst || jtag_boot_bypass || dbg_cold_reset_hold
        //
        // `dbg_cold_reset_hold` (DBG_CONTROL[4], a sticky level a JTAG host
        // raises at an arbitrary cycle) resets boot_fsm WITHOUT asserting
        // soc_full_rst.  boot_fsm then drops m_axi_awvalid/wvalid mid-burst
        // -- it issues awlen up to 255 -- while this adapter stays live with
        // `aw_valid_q` set and beats still owed.  aw_valid_q clears ONLY on
        // `aw_release = aw_valid_q && w_last_hs && aw_iss_ok`, a wide WLAST
        // that will never come, and `n_awready = !aw_valid_q && ...` is then
        // 0 FOREVER: the restarted boot_fsm's very first AW is refused,
        // `rom_loaded` never rises, and `cpu_rst` is held until the FPGA is
        // reconfigured.  ENABLE_ABANDON_TIMEOUT defaults to 0 and is not
        // overridden here, so nothing bounds it.
        //
        // Pairing the two resets is only half the fix: the adapter dropping
        // its wide burst leaves the crossbar's shared slot-0 write with
        // S0/DDR parked mid-burst.  That half is closed by the xbar's
        // `m0b_master_reset` input, wired to this same net in
        // fpga_top_xbar.vh and pinned by tb_axi_xbar scenario 55 -- which,
        // without it, trips the crossbar's own "WLAST on the wrong beat"
        // assertion on the NEXT write.  Neither half is safe alone.
        //
        // NO UNIT TEST covers this line itself: tb_rom_boot is removed and
        // tb_cold_boot is disabled, and tb_boot_release_gate.v is a
        // hand-copied replica of the reset expression (it codes
        // `boot_fsm_rst = soc_full_rst || jtag_boot_bypass`, with no hold
        // term at all) so it cannot fail if this were reverted.  Recorded
        // honestly rather than claimed as covered.
        .clk(core_clk), .rst(boot_fsm_rst),

        // Narrow side (AW has no awprot on boot_fsm — tie to 0)
        .n_awaddr (boot_m_awaddr ),
        .n_awprot (3'b000        ),
        // Connect boot_fsm's REAL length outputs -- do not tie these to 0.
        // boot_fsm's ZERO_AWLEN parameter defaults to 255 (a 1 KiB, 1 KiB-
        // aligned burst, so it can never cross the AXI 4 KiB boundary that
        // nothing between here and the MIG checks or splits).  A hardcoded
        // 8'd0 here would silently TRUNCATE that burst to one beat --
        // zeroing 1/256 of RAM and releasing the CPU onto it, which is the
        // same failure mode as the BUG-1 swallowed-BRESP defect.
        // These inputs arrived with the cpu-side burst widener; production
        // lint runs -Wno-PINMISSING, so leaving them unconnected would
        // elaborate them UNDRIVEN with no diagnostic at all.
        .n_awlen  (boot_m_awlen  ),
        .n_arlen  (8'd0          ),   // boot_fsm has no read master (AR unused)
        .n_awvalid(boot_m_awvalid),
        .n_awready(boot_m_awready),
        .n_wdata  (boot_m_wdata  ),
        .n_wstrb  (boot_m_wstrb  ),
        .n_wlast  (boot_m_wlast  ),
        .n_wvalid (boot_m_wvalid ),
        .n_wready (boot_m_wready ),
        .n_bresp  (boot_m_bresp  ),
        .n_bvalid (boot_m_bvalid ),
        .n_bready (boot_m_bready ),
        // boot_fsm is write-only — tie off narrow read
        .n_araddr (32'b0),
        .n_arprot (3'b000),
        .n_arvalid(1'b0),
        .n_arready(/* unused */),
        .n_rdata  (/* unused */),
        .n_rresp  (/* unused */),
        .n_rlast  (/* unused */),
        .n_rvalid (/* unused */),
        .n_rready (1'b0),

        // Wide side
        .w_awid   (boot_w_awid   ), .w_awaddr (boot_w_awaddr ),
        .w_awlen  (boot_w_awlen  ), .w_awsize (boot_w_awsize ),
        .w_awburst(boot_w_awburst), .w_awvalid(boot_w_awvalid),
        .w_awready(boot_w_awready),
        .w_wdata  (boot_w_wdata  ), .w_wstrb  (boot_w_wstrb  ),
        .w_wlast  (boot_w_wlast  ), .w_wvalid (boot_w_wvalid ),
        .w_wready (boot_w_wready ),
        .w_bid    (boot_w_bid    ), .w_bresp  (boot_w_bresp  ),
        .w_bvalid (boot_w_bvalid ), .w_bready (boot_w_bready ),
        .w_arid   (/* unused */), .w_araddr (/* unused */),
        .w_arlen  (/* unused */), .w_arsize (/* unused */),
        .w_arburst(/* unused */), .w_arvalid(/* unused */),
        .w_arready(1'b0),
        .w_rid    (4'b0), .w_rdata  (128'b0),
        .w_rresp  (2'b0), .w_rlast  (1'b0),
        .w_rvalid (1'b0), .w_rready (/* unused */)
    );

    // boot_fsm's m_axi_bid is an input — the adapter doesn't forward BID
    // (single outstanding, no tag matching needed on narrow side).
    // Wire it as a reg alias → 0; boot_fsm ignores it anyway.

