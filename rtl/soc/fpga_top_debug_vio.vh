// rtl/fpga_top_debug_vio.vh — included from rtl/fpga_top.v
//
// LED wiring, host-unused debug signal sinks, and the JTAG VIO probe (VIO_ENABLE) with AXI error tracking.
//
// Generated mechanically from rtl/fpga_top.v during the fpga_top split
// (agent/p3-fpga-top-split). No functional change: this file contains
// the original lines verbatim, inside the fpga_top module scope via
// `include. Do not add module/endmodule wrappers or nettype directives.

    // ═══════════════════════════════════════════════════════════════════
    // LEDs — board-side bring-up status (left-to-right power-on sequence)
    //
    //   led[0] = ddr_cal_done       — DDR4 calibration finished
    //   led[1] = ~spi_cs_n_wire     — SD bus active (chip select asserted),
    //                                  i.e. boot ROM copy in progress, SD
    //                                  provisioning R/W, or future SCSI SD
    //                                  access.  Flickers during transfers.
    //   led[2] = boot_rom_loaded    — boot ROM image fully copied to DRAM
    //   led[3] = hdmi_mmcm_locked   — HDMI clocks locked
    //
    // Error / reset status moves to JTAG VIO probes (vio_rst_bundle and
    // vio_boot_video already carry core_rst, boot_error, fb_underflow_sticky,
    // etc.).  Physical buttons no longer XOR onto LEDs; for visual feedback
    // on a press, watch the Mac respond (NMI = btn[1]) or the LED row briefly
    // flash as core_rst re-asserts and DDR cal status drops (debug-full-reset
    // = btn[2]).
    //
    // Button assignments (consumed in fpga_top_clocks.vh):
    //   btn[0] = DDR sys_rst                 (consumed in fpga_top_ddr.vh)
    //   btn[1] = NMI request                 (level 7 edge into irq_agg)
    //   btn[2] = debug full reset            (OR'd with vio_boot_ctrl[3])
    //   btn[3] = platform_reset_req          (board-level async reset)
    //
    // All four btn[] inputs have a real consumer now, so Vivado will not
    // prune them and the prior LED-XOR anti-prune trick is no longer
    // needed.
    // ═══════════════════════════════════════════════════════════════════
    assign led[0] = ddr_cal_done;
    assign led[1] = ~spi_cs_n_wire;
    assign led[2] = boot_rom_loaded;
    assign led[3] = hdmi_mmcm_locked;

    // Silence lint on the smoke-done indicator when VIDEO_SMOKE=0 (it
    // is tied to 0 in the gen_no_smoke branch); when VIDEO_SMOKE=1 the
    // smoke writer asserts it after the last VRAM word lands.  Driven
    // but intentionally unused in the current pinout — a future bring-up
    // can XOR it onto a spare LED when the board has one.
    wire _unused_smoke = &{1'b0, smoke_done};
    wire _unused_scc_uart = &{1'b0, scc_uart_tx_busy, scc_rts_a_n, scc_dtr_a_n,
                              scc_rts_b_n, scc_dtr_b_n};
    // CPU SOCKET: the rich debug taps (boundary stream, last_pc, dcache
    // probe) no longer cross the socket — they are CPU-private behind
    // dbg_axi.  Only the SoC-side leftover control wires (now tied off in
    // fpga_top_debug_ctrl.vh, relocating CPU-side in Phase 4) and the
    // platform dafb_fb_bpp_reg remain to sink here.
    wire _unused_core_dbg = &{1'b0,
                              dbg_step_req, dbg_redirect_pc,
                              dbg_redirect_valid, dbg_irq_inject_lvl,
                              dbg_irq_inject_pulse, dafb_fb_bpp_reg};

    // VIO probe trim (2026-04-26 debug-bloat-cull): wires below were
    // wired into vio_write_counts (probe_in13) and vio_fb_reader_stats
    // (probe_in17).  Both probes were dropped to relieve LUT pressure;
    // the underlying counters in fpga_top_video.vh / video_top.v retain
    // their producers and Vivado prunes them automatically.  Sink them
    // here so the lint pass stays clean in both VIO-enabled and
    // VIO-disabled builds.
    wire _unused_dropped_vio_probes = &{1'b0,
                                         vram_write_count,
                                         dafb_write_count};

    // RESTORED 2026-08-01: vio_fb_reader_stats was culled above for LUT
    // pressure, and it is exactly the instrument this project then spent a
    // full day without.  req_count vs rsp_count is a DIRECT measure of
    // dropped fetch responses -- their difference is the drop count.  The
    // surviving fb_underflow_sticky cannot do this job: it ORs a candidate
    // CAUSE (fb_reader's response FIFO overflowed) with an unavoidable
    // EFFECT (the scanout ran out of pixels, which ANY wedged fetcher
    // produces), so one bit can never separate them.
    //
    // Ordering is req in the low half so a plain `vio-read` shows
    // {miss, rsp, req} left-to-right.  NOTE: read the `_1`-suffixed probe
    // name -- the bare name is a 1-bit alias.
    wire [47:0] vio_fb_reader_stats = {fb_reader_miss_count,
                                       fb_reader_rsp_count,
                                       fb_reader_req_count};

    // ═══════════════════════════════════════════════════════════════════
    // JTAG VIO debug instance (IP-generation flow)
    //
    // Gated by `VIO_ENABLE` — the Vivado TCL pre-synth step generates
    // the `debug_vio` IP (see synth/vivado.tcl gen_debug_vio_ip) and
    // passes `-verilog_define VIO_ENABLE` to synth_design when
    // `ENABLE_VIO=1` env var is set.  When disabled, no IP is
    // generated and this block is elided entirely — the bitstream has
    // no VIO.
    //
    // Probe map — see synth/vivado.tcl and synth/vio_dashboard.tcl for
    // the canonical documentation.
    //
    // ═══════════════════════════════════════════════════════════════════
`ifdef VIO_ENABLE
    localparam [3:0] AXI_ERR_SRC_NONE = 4'd0;
    localparam [3:0] AXI_ERR_SRC_DDR  = 4'd1;
    localparam [3:0] AXI_ERR_SRC_IO   = 4'd2;
    localparam [3:0] AXI_ERR_SRC_DMA  = 4'd3;
    localparam [3:0] AXI_ERR_SRC_VRAM = 4'd4;
    localparam [3:0] AXI_ERR_SRC_BOOT = 4'd5;
    localparam [3:0] AXI_ERR_SRC_XDMA = 4'd6;

    wire [5:0]  vio_rst_bundle = {platform_resetn, core_rst, ddr_cal_done,
                                   boot_rom_ready, hdmi_i2c_done,
                                   fb_underflow_sticky};
    wire [4:0]  vio_hdmi_ctrl = {al9134_resetn, hdmi_i2c_done,
                                  video_debug_de, video_debug_vs,
                                  video_debug_hs};
    wire [1:0]  vio_vram_read = {vram_rd_valid, vram_rd_en};
    wire [9:0]  vio_ddr_axi = {s0_awvalid, s0_awready,
                                s0_wvalid,  s0_wready,
                                s0_bvalid,  s0_bready,
                                s0_arvalid, s0_arready,
                                s0_rvalid,  s0_rready};
    wire [7:0]  vio_boot_video = {boot_rom_loading, boot_error,
                                   hdmi_mmcm_locked, hdmi_i2c_done,
                                   video_debug_de, vram_rd_en,
                                   vram_rd_valid, al9134_int};
    wire [95:0] vio_dafb_cfg = {dafb_fb_base_px, dafb_fb_stride_px,
                                dafb_fb_bpp_reg};
    // Real-HW CMD18/CRC-retry diagnosis (boot_fsm.v dbg_sector /
    // dbg_err_cause / dbg_ctrl_retry) — added so a stuck boot can be
    // root-caused (which sector, which error, how many retries ran)
    // without a fresh rebuild every time it happens.
    // [24:23] ROM-overlay state (2026-09-12).  Folded into vio_boot_diag's
    // spare MSBs rather than a new probe_in so no VIO IP regeneration is
    // needed (probe_in26 is the highest the generated core carries).
    //   bit 24 = cpu_overlay_disabled_q  — the sticky "first CPU ROM read
    //            seen" latch in axi_xbar, cleared by cpu_overlay_reset
    //   bit 23 = effective_cpu_overlay_active — the AND of VIA1's overlay
    //            bit with !disabled; this is what actually redirects the
    //            CPU's low-memory accesses to the ROM alias
    // After a debug `reset` that leaves the machine dead, bit 23 == 0 means
    // the overlay did NOT re-arm, so the 68k's vector fetch at 0x0/0x4 went
    // to plain DRAM instead of ROM — that is the open wedge hypothesis (see
    // docs/reset_wedge_diagnosis.md).  bit 23 == 1 refutes it and the hunt
    // moves to the CPU side.
    // [31]    xbar_s1_slot_busy   — a write/read slot still outstanding to S1 (the
    //                               peripheral-bus path the 0x5090 debug window uses).
    //                               A swallowed BRESP strands exactly this.
    // [30:25] xbar_slv_poisoned[5:0] — the xbar's STICKY per-slave poison latches.
    //                               S1 is bit 26 (index 1). If bit 26 is set after a
    //                               wedging `reset`, the poison theory is CONFIRMED;
    //                               if it is clear, the poison is NOT the mechanism and
    //                               the hunt moves elsewhere. Either answer is progress.
    wire [31:0] vio_boot_diag = {xbar_s1_slot_busy, xbar_slv_poisoned,
                                  xbar_overlay_disabled,
                                  xbar_overlay_effective, boot_dbg_ctrl_retry,
                                  boot_dbg_err_cause, boot_dbg_sector};
    // Raw computed-vs-received CRC16 for the failing block — if
    // dbg_rd_crc_recv reads back a FIXED value (e.g. 0x0000) regardless
    // of actual boot ROM content across independent attempts, that
    // proves the card isn't sending a genuine per-block CRC at all
    // (some cheap SD controllers accept CMD59 without ever actually
    // computing one), not a real data-integrity problem.
    // ADB live-poke diagnosis (2026-07-25) -- keyboard/mouse never
    // worked; VIA1 was observed (via JTAG dump-mem of its own
    // registers) stuck in "shift in under external clock" with CB1
    // never firing -- i.e. adb_phy.v never gets a device response
    // clocked back in. This bundle exposes adb_modem's own dispatch
    // state (adb_dbg_state) and the actual command/address it last
    // sent to the keyboard/mouse device models, plus each device's own
    // response-ready/empty/SRQ status, so the next live session can
    // see WHICH device (if either) is being addressed and whether ITS
    // response path is the one that's silent, without needing a
    // rebuild to add visibility after the fact again.
    wire [19:0] vio_adb_dbg = {
        adb_dbg_state,          // [19:16] adb_modem's own state
        adb_dev_cmd_valid,      // [15]
        adb_dev_cmd_addr,       // [14:11]
        adb_dev_cmd_op,         // [10:8]
        adb_kbd_resp_valid,     // [7]
        adb_kbd_resp_empty,     // [6]
        adb_kbd_srq,            // [5]
        adb_ms_resp_valid,      // [4]
        adb_ms_resp_empty,      // [3]
        adb_ms_srq,             // [2]
        adb_dev_listen_valid,   // [1]
        1'b0                    // [0] pad
    };
    // L2C hit/miss/occupancy counters (2026-07-25) -- so the next live
    // boot can show whether the cache is doing anything useful, not
    // just that it isn't SLVERR'ing anymore. See l2c_ctrl.v's
    // dbg_hit_count/dbg_miss_count/dbg_mshr_occupancy port comments for
    // exactly what's counted. Free-running (wraps on overflow); a
    // point-in-time VIO read is a snapshot, not a rate -- sample twice
    // with a known time delta to derive a hit rate if needed.
    // Guarded: the counter wires are declared inside fpga_top_ddr.vh's
    // own `ifdef L2C_ENABLE block (l2c isn't elaborated at all without
    // it), and L2C_ENABLE is opt-in per synth/vivado.tcl, so an
    // unguarded reference here breaks every non-L2C build.  Reads back
    // as all-zero when l2c is absent, which is the honest answer.
`ifdef L2C_ENABLE
    wire [67:0] vio_l2c_stats = {
        dbg_l2c_hit_count,        // [67:36]
        dbg_l2c_miss_count,       // [35:4]
        dbg_l2c_mshr_occupancy    // [3:0]
    };
`else
    wire [67:0] vio_l2c_stats = 68'd0;
`endif
    // ── SCSI→SD disk-I/O completion counters (2026-07-25) ──────────────
    // Added to answer ONE question that is currently unobservable on real
    // hardware: when Mac OS stalls forever polling ioResult (File Manager
    // disk read that never finishes -- see the ioResult/SysError-12
    // investigation), does the SCSI→SD read actually COMPLETE?
    //
    // scsi.v issues a backing-store request with sd_go and then waits for
    // sd_done / sd_error. Completion/error totals show whether traffic is
    // progressing, while the first-error record below preserves sd_ctrl's
    // cause, command and LBA before a later request overwrites its live
    // diagnostic outputs.
    //
    // Why this had to be added: there is NO other way to see this state.
    // JTAG-AXI cannot read the SCSI registers -- peripheral_bus exposes
    // them byte-granular (scsi_addr[8:0], 8-bit data) and JTAG-AXI is a
    // word-only master (same limitation synth/adb_inject_verify.tcl
    // documents for the ADB byte map), and NCR5380 register reads have
    // side effects, so probing them from JTAG would perturb the driver.
    //
    // Clocked in the SCSI's own domain (pb_clk / pb_full_rst_bank[2], the
    // same reset scsi.v takes) so the counts clear when the SCSI FSM
    // rewinds.  All three events are edge-detected: sd_done/sd_error are
    // consumed as level-ish handshakes by scsi.v, so counting them raw
    // would add one per cycle held.  Counters saturate instead of
    // wrapping -- after a long boot a wrapped small number reads exactly
    // like "barely any I/O", which is the wrong conclusion to hand
    // someone at 3am.
    reg [15:0] scsi_sd_done_count_r;
    reg [7:0]  scsi_sd_err_count_r;
    reg        scsi_sd_done_d_r;
    reg        scsi_sd_error_d_r;

    // First backing-store failure, captured in sd_ctrl's core_clk domain.
    // sd_ctrl's debug outputs are transaction-latched, so unlike the live
    // pb_clk request wires this record cannot be overwritten by later I/O.
    reg        scsi_sd_err_sticky_r;
    reg [31:0] scsi_sd_err_lba_r;
    reg [3:0]  scsi_sd_err_cause_r;
    reg [3:0]  scsi_sd_err_cmd_r;
    reg [7:0]  scsi_sd_err_detail_r;
    reg [15:0] scsi_sd_err_crc_calc_r;
    reg [15:0] scsi_sd_err_crc_recv_r;

    always @(posedge core_clk) begin
        if (soc_full_rst_bank[5]) begin
            scsi_sd_err_sticky_r  <= 1'b0;
            scsi_sd_err_lba_r     <= 32'd0;
            scsi_sd_err_cause_r   <= 4'd0;
            scsi_sd_err_cmd_r     <= 4'd0;
            scsi_sd_err_detail_r  <= 8'd0;
            scsi_sd_err_crc_calc_r <= 16'd0;
            scsi_sd_err_crc_recv_r <= 16'd0;
        end else if (core_scsi_error && !scsi_sd_err_sticky_r) begin
            scsi_sd_err_sticky_r  <= 1'b1;
            // Multi-block commands expose a base plus block index.  The
            // SCSI-safe write path emits individual CMD24s whose latched LBA
            // is already exact, so adding block_idx would double-count.
            scsi_sd_err_lba_r     <= ((core_scsi_dbg_cur_cmd == 4'd2) ||
                                      (core_scsi_dbg_cur_cmd == 4'd4)) ?
                                     (core_scsi_dbg_lba_lat +
                                      {16'd0, core_scsi_dbg_block_idx}) :
                                     core_scsi_dbg_lba_lat;
            scsi_sd_err_cause_r   <= core_scsi_err_cause;
            scsi_sd_err_cmd_r     <= core_scsi_dbg_cur_cmd;
            // R1 is the useful detail for command failures; token waits use
            // the terminal poll count. CRC failures are exposed in full on
            // vio_boot_crc below.
            scsi_sd_err_detail_r  <= (core_scsi_err_cause == 4'd3) ?
                                     core_scsi_dbg_last_poll_cnt :
                                     (core_scsi_err_cause == 4'd4) ?
                                     core_scsi_dbg_last_write_resp :
                                     core_scsi_dbg_last_real_r1;
            scsi_sd_err_crc_calc_r <= core_scsi_dbg_rd_crc_calc;
            scsi_sd_err_crc_recv_r <= core_scsi_dbg_rd_crc_recv;
        end
    end

    // Boot and SCSI share this fixed-width probe. A SCSI failure takes
    // precedence after ROM loading so CRC mismatches retain both operands.
    wire [31:0] vio_boot_crc = scsi_sd_err_sticky_r ?
                               {scsi_sd_err_crc_calc_r,
                                scsi_sd_err_crc_recv_r} :
                               {boot_dbg_rd_crc_calc,
                                boot_dbg_rd_crc_recv};

    always @(posedge pb_clk) begin
        if (pb_full_rst_bank[2]) begin
            scsi_sd_done_count_r <= 16'd0;
            scsi_sd_err_count_r  <= 8'd0;
            scsi_sd_done_d_r     <= 1'b0;
            scsi_sd_error_d_r    <= 1'b0;
        end else begin
            scsi_sd_done_d_r  <= scsi_sd_done_w;
            scsi_sd_error_d_r <= scsi_sd_error_w;

            if (scsi_sd_done_w && !scsi_sd_done_d_r &&
                ~&scsi_sd_done_count_r)
                scsi_sd_done_count_r <= scsi_sd_done_count_r + 16'd1;
            if (scsi_sd_error_w && !scsi_sd_error_d_r &&
                ~&scsi_sd_err_count_r)
                scsi_sd_err_count_r <= scsi_sd_err_count_r + 8'd1;
        end
    end

    // scsi.v CHECK CONDITION attribution, appended ABOVE the existing
    // fields so every previously-documented bit position keeps its meaning
    // (tools/jt.sh "vio-read scsi" parsers index from the LSB end).
    //   [96]    medium_not_present (sticky; set by the READ backing-store
    //           timeout arms, which return sense key 2 / ASC 3A)
    //   [95:92] sense_key   [91:84] sense_asc
    //   [83:76] CHECK CONDITION count (saturating)
    // Also surface what the RANGE CHECK actually sees.  vhdd_mux.v:178 is
    //   m_num_lbas = dev_sel ? b_num_lbas : a_num_lbas
    // so a stale/wrong vh_dev_sel makes scsi.v range-check a READ against the
    // WRONG volume's capacity and reject it with ILLEGAL REQUEST / ASC 0x21 --
    // producing CHECK CONDITION with no back-end activity, which is exactly
    // the measured signature. These two make that immediately checkable.
    //   [129:98] vh_num_lbas as seen by scsi.v   [97] vh_dev_sel
    //   [186:155] xfer_lba   [154:131] xfer_blocks   [130] chk_ok
    // ── 53C96 initiator-state probe (probe_in26) ─────────────────────
    // Added 2026-08-19 for the post-boot-fix hang: the ROM's untimed
    // wait-for-INT at 0x40899704 spins forever after `W reg2=0xEE;
    // W reg3=0x10`.  Deliberately a SEPARATE probe rather than widening
    // vio_scsi_sd: that wire is 187 bits and a Xilinx VIO probe caps at
    // 256, so the 81-bit payload would not fit — and keeping it separate
    // leaves every existing vio_scsi_sd bit offset untouched.
    //
    // Bit offsets AS READ OFF THIS PROBE (probe_in26, 84 bits):
    //   [83]    c96_sel_stopped
    //   [82]    c96_sel_active
    //   [81]    c96_xfer_active   ** live connection (MAME mode==MODE_I) **
    //   [80:79] c96_dma_dir       0=NONE 1=IN 2=OUT
    //   [78:69] c96_accept_pend
    //   [68:52] c96_xfr_left
    //   [51]    c96_xfr_armed     ** read this first **
    //   [50]    c96_xfr_dma
    //   [49:46] c96_xfr_phase     ** latched at dispatch **
    //   [45:42] phase             ** live **  (mismatch = phase-drift hole)
    //   [41:40] c96_command_pos
    //   [39:32] c96_command_q     (reg 3 echo)
    //   [31:24] c96_cmd_q1
    //   [23:19] c96_fifo_pos      (reg 7)
    //   [18:11] c96_istatus       non-destructive; reg 5 read would clear
    //   [10:3]  c96_status_sticky {GROSS,PARITY,_,TCC,TC0,_,_,_}
    //   [2]     c96_irq_pending
    //   [1]     t_req             target-side REQ
    //   [0]     c96_nondma_supply
    // Phase encoding for [49:46]/[45:42]:
    //   0 BUS_FREE 1 SELECT 2 COMMAND 3 CMD_EXEC 4 DATA_IN 5 DATA_OUT
    //   6 VH_WAIT_RD 7 VH_WAIT_WR 8 STATUS 9 MSG_IN 10 DISCONNECT 11 RESET
    // Values 3/6/7 in [49:46] are states MAME's xfr_phase can never hold.
    wire [83:0] vio_scsi_c96 = scsi_dbg_c96_state_w;

    wire [186:0] vio_scsi_sd = {
        scsi_dbg_xfer_lba_w,
        scsi_dbg_xfer_blocks_w,
        scsi_dbg_chk_ok_w,
        scsi_vh_num_lbas_w,
        scsi_vh_dev_sel_w,
        scsi_dbg_mnp_w,
        scsi_dbg_sense_key_w,
        scsi_dbg_sense_asc_w,
        scsi_dbg_cc_count_w,
        scsi_sd_err_lba_r,      // [75:44] first failing SD LBA
        scsi_sd_done_count_r,   // [43:28] completions seen
        scsi_sd_err_count_r,    // [27:20] errors seen
        scsi_sd_err_cause_r,    // [19:16] sd_ctrl ERR_* classification
        scsi_sd_err_cmd_r,      // [15:12] sd_ctrl SC_CMD* encoding
        scsi_sd_err_detail_r,   // [11:4]  R1, read poll count, or write response
        scsi_sd_err_sticky_r,   // [3]     first-error snapshot valid
        scsi_sd_busy_w,         // [2]     backing store busy right now
        scsi_irq_pb,            // [1]     SCSI IRQ to VIA2
        scsi_drq_w              // [0]     SCSI DRQ
    };

    // Note: vio_write_counts (vram_write_count + dafb_write_count),
    // vio_vram_write (smoke handshakes), and vio_fb_reader_stats were
    // dropped 2026-04-26 to free ~5k LUTs of VIO IP-internal storage.
    // The same data is reachable through the JTAG-AXI debug_ctrl
    // register block; only the at-a-glance VIO dashboard view was lost.

    reg [3:0] axi_error_src_r;
    reg [1:0] axi_error_resp_r;
    reg       axi_error_seen_r;
    reg       axi_error_sticky_r;

    // Address capture for first SLVERR/DECERR.  Two latches per slave we
    // care about: track the most recent AW/AR address and freeze it when
    // the matching B/R channel returns a non-OKAY response.  Captures the
    // FIRST error only (sticky); cleared by soc_full_rst (the unified
    // reset — Phase-2 of docs/reset_story.md, was core_rst pre-unify).
    // AXI is pipelined, so "most recent" can be off by one outstanding
    // txn — good enough to localize, not for forensic precision.
    reg [31:0] last_s0_aw_addr_r;
    reg [31:0] last_s0_ar_addr_r;
    reg [31:0] err_s0_aw_addr_r;
    reg [31:0] err_s0_ar_addr_r;
    reg        err_s0_aw_captured_r;
    reg        err_s0_ar_captured_r;

    wire ddr_axi_error = (s0_bvalid && s0_bready && (s0_bresp != 2'b00)) ||
                         (s0_rvalid && s0_rready && (s0_rresp != 2'b00));
    wire io_axi_error  = (s1_bvalid && s1_bready && (s1_bresp != 2'b00)) ||
                         (s1_rvalid && s1_rready && (s1_rresp != 2'b00));
    wire dma_axi_error = (s2_bvalid && s2_bready && (s2_bresp != 2'b00)) ||
                         (s2_rvalid && s2_rready && (s2_rresp != 2'b00));
    wire vram_axi_error = (s3_bvalid && s3_bready && (s3_bresp != 2'b00)) ||
                          (s3_rvalid && s3_rready && (s3_rresp != 2'b00));
    wire xdma_axi_error = (xdma_bvalid && xdma_bready && (xdma_bresp != 2'b00)) ||
                          (xdma_rvalid && xdma_rready && (xdma_rresp != 2'b00));
    wire boot_axi_error = boot_error;
    wire [15:0] vio_axi_error = {
        8'd0,
        axi_error_seen_r,
        axi_error_sticky_r,
        axi_error_src_r[3:0],
        axi_error_resp_r[1:0]
    };

    always @(posedge core_clk) begin
        if (soc_full_rst) begin
            axi_error_src_r   <= AXI_ERR_SRC_NONE;
            axi_error_resp_r  <= 2'b00;
            axi_error_seen_r  <= 1'b0;
            axi_error_sticky_r <= 1'b0;
            last_s0_aw_addr_r    <= 32'd0;
            last_s0_ar_addr_r    <= 32'd0;
            err_s0_aw_addr_r     <= 32'd0;
            err_s0_ar_addr_r     <= 32'd0;
            err_s0_aw_captured_r <= 1'b0;
            err_s0_ar_captured_r <= 1'b0;
        end else begin
            // Track most-recent AW/AR address per slave we capture for.
            // Updated on handshake; used as the "blame" addr if the next
            // B/R response from this slave is non-OKAY.
            if (s0_awvalid && s0_awready) last_s0_aw_addr_r <= s0_awaddr;
            if (s0_arvalid && s0_arready) last_s0_ar_addr_r <= s0_araddr;

            // Capture-on-first-error for S0 (DDR) — the only slave whose
            // SLVERR is currently mysterious.  Sticky: first error wins.
            if (!err_s0_aw_captured_r &&
                s0_bvalid && s0_bready && (s0_bresp != 2'b00)) begin
                err_s0_aw_addr_r     <= last_s0_aw_addr_r;
                err_s0_aw_captured_r <= 1'b1;
            end
            if (!err_s0_ar_captured_r &&
                s0_rvalid && s0_rready && (s0_rresp != 2'b00)) begin
                err_s0_ar_addr_r     <= last_s0_ar_addr_r;
                err_s0_ar_captured_r <= 1'b1;
            end
            if (ddr_axi_error) begin
                axi_error_src_r  <= AXI_ERR_SRC_DDR;
                axi_error_seen_r <= 1'b1;
                axi_error_sticky_r <= 1'b1;
                axi_error_resp_r <= (s0_bvalid && s0_bready && (s0_bresp != 2'b00))
                    ? s0_bresp : s0_rresp;
            end else if (io_axi_error) begin
                axi_error_src_r  <= AXI_ERR_SRC_IO;
                axi_error_seen_r <= 1'b1;
                axi_error_sticky_r <= 1'b1;
                axi_error_resp_r <= (s1_bvalid && s1_bready && (s1_bresp != 2'b00))
                    ? s1_bresp : s1_rresp;
            end else if (dma_axi_error) begin
                axi_error_src_r  <= AXI_ERR_SRC_DMA;
                axi_error_seen_r <= 1'b1;
                axi_error_sticky_r <= 1'b1;
                axi_error_resp_r <= (s2_bvalid && s2_bready && (s2_bresp != 2'b00))
                    ? s2_bresp : s2_rresp;
            end else if (vram_axi_error) begin
                axi_error_src_r  <= AXI_ERR_SRC_VRAM;
                axi_error_seen_r <= 1'b1;
                axi_error_sticky_r <= 1'b1;
                axi_error_resp_r <= (s3_bvalid && s3_bready && (s3_bresp != 2'b00))
                    ? s3_bresp : s3_rresp;
            end else if (xdma_axi_error) begin
                axi_error_src_r  <= AXI_ERR_SRC_XDMA;
                axi_error_seen_r <= 1'b1;
                axi_error_sticky_r <= 1'b1;
                axi_error_resp_r <= (xdma_bvalid && xdma_bready && (xdma_bresp != 2'b00))
                    ? xdma_bresp : xdma_rresp;
            end else if (boot_axi_error) begin
                axi_error_src_r  <= AXI_ERR_SRC_BOOT;
                axi_error_seen_r <= 1'b1;
                axi_error_sticky_r <= 1'b1;
                axi_error_resp_r <= 2'b10;
            end
        end
    end

    (* DONT_TOUCH = "true" *) debug_vio u_dbg_vio (
        .clk       (core_clk),
        .probe_in0 (hdmi_mmcm_locked),
`ifdef PCIE_XDMA_ENABLE
        // PCIe bring-up diagnostics (bits [2:0]); hcount[11:3] retained
        // in the upper bits.  [0]=user_lnk_up, [1]=GT power-good
        // (sys_clk_ce_out), [2]=XDMA axi_aresetn (user_clk domain up).
        .probe_in1 ({video_debug_hcount[11:3],
                     pcie_axi_aresetn, pcie_sysclk_ce, pcie_user_lnk_up}),
`else
        .probe_in1 (video_debug_hcount),
`endif
        .probe_in2 (video_debug_vcount),
        .probe_in3 (vram_rd_addr),
        .probe_in4 (video_debug_rgb),
        .probe_in5 (dbg_pc),
        .probe_in6 (ddr_dbg_r_cnt[15:0]),
        .probe_in7 (vio_rst_bundle),
        .probe_in8 (s0_wready),
        .probe_in9 (dbg_committed),
        .probe_in10(vio_hdmi_ctrl),
        .probe_in11(vio_vram_read),
        .probe_in12(vio_ddr_axi),
        .probe_in13(vio_boot_video),     // was probe_in15
        .probe_in14(vio_dafb_cfg),       // was probe_in16
        .probe_in15(vio_axi_error),      // was probe_in18
        .probe_in16(err_s0_aw_addr_r),   // first DDR write SLVERR addr
        .probe_in17(err_s0_ar_addr_r),   // first DDR read SLVERR addr
        .probe_in18(vio_boot_diag),      // boot_fsm sector/err_cause/retry
        .probe_in19(vio_boot_crc),       // computed-vs-received CRC16
        .probe_in20(vio_adb_dbg),        // adb_modem state + kbd/mouse resp status
        .probe_in21(vio_l2c_stats),      // l2c hit/miss/occupancy counters
        .probe_in22(vio_scsi_sd),
        .probe_in23(vio_fb_reader_stats), // req/rsp/miss — drop detector        // SCSI->SD go/done/err counts + last LBA
        // Task #243 — coherent one-pclk-edge scan-out snapshot.  Unlike
        // video_debug_hcount/vcount/rgb (three INDEPENDENT probes, each
        // sampled in its own JTAG transaction and therefore impossible to
        // correlate), every field of these two is captured on the same
        // clock edge in video_top.  Decode with the video-status command.
        .probe_in24(video_dbg_snap),      // {hcount,vcount,de,dafb_live,rd_*,uflow_lb,uflow_fbr,scale,rgb,bpp,bytes,hres,vres}
        .probe_in25(video_dbg_place),     // {committed fb_base_px, committed fb_stride_px}
        .probe_in26(vio_scsi_c96),        // 53C96 initiator state — see the bit map above
        // 5 bits as of the battery-backed-PRAM change (was 4).  Width is
        // set by CONFIG.C_PROBE_OUT0_WIDTH in gen_debug_vio_ip
        // (synth/vivado.tcl) — bump both together or the IP/RTL widths
        // disagree.  bit4 = pram_clear; see fpga_top_clocks.vh bit-map.
        .probe_out0(vio_boot_ctrl),
        .probe_out1(vio_hard_reset)      // VIO equivalent of a btn[3] press
                                          // (platform_reset_req/platform_resetn
                                          // only -- see fpga_top_clocks.vh's
                                          // btn3_resetn_db comment for why this
                                          // deliberately does NOT touch the raw
                                          // fabric_gt_clr / BUFG_GT .CLR path)
    );
`endif

    // ═══════════════════════════════════════════════════════════════════
    // JTAG ILA debug instance (IP-generation flow)
    //
    // Gated by `ILA_ENABLE` — the Vivado TCL pre-synth step generates
    // the `debug_ila` IP (see synth/debug_ila.tcl gen_debug_ila_ip) and
    // passes `-verilog_define ILA_ENABLE` to synth_design when
    // `ENABLE_ILA=1` env var is set.  When disabled, no IP is
    // generated and this block is elided entirely.
    //
    // Probe map — see synth/debug_ila.tcl and
    // docs/ila_a7_drift_probes.md for the canonical documentation.
    // Hunts the post-DAFB A7-drift / boot-wedge HW-only race
    // (exc_pc=0x4086abb2 / exc_count=0x12c5 / pc_live=0x6db6db6d).
    //
    // v3 (2026-05-26): probes 16..24 added to observe ALL five PRF
    // write sources (cdb0, cdb1, cdb2, cdb_alu_hi, a7_writeback) and
    // a 1-bit derived trigger (probe24) that fires when any of those
    // sources targets phys 0x01 (= the post-rename A7 phys observed
    // corrupting at /tmp/ila_capture_cdb1fix.csv sample 3503 with
    // flush_en=0, rob_pop=0, drain_active=1).
    // ═══════════════════════════════════════════════════════════════════
`ifdef ILA_ENABLE
    // Decompose the commit-internal bundle into the named ILA probes.
    wire        ila_a7_macro_locked       = dbg_ila_commit_bundle_w[25];
    wire [6:0]  ila_a7_macro_pre_phys     = dbg_ila_commit_bundle_w[24:18];
    wire [6:0]  ila_committed_a7_phys     = dbg_ila_commit_bundle_w[17:11];
    wire        ila_mb_drop               = dbg_ila_commit_bundle_w[10];
    wire        ila_mb_empty              = dbg_ila_commit_bundle_w[9];
    wire        ila_drain_active          = dbg_ila_commit_bundle_w[8];
    wire        ila_store_commit_exc      = dbg_ila_commit_bundle_w[7];
    wire        ila_take_trace            = dbg_ila_commit_bundle_w[6];
    wire        ila_take_exc              = dbg_ila_commit_bundle_w[5];
    wire        ila_take_irq_arm          = dbg_ila_commit_bundle_w[4];
    wire        ila_take_irq_fire_q       = dbg_ila_commit_bundle_w[3];
    wire        ila_take_irq_preempt      = dbg_ila_commit_bundle_w[2];
    wire        ila_take_rte              = dbg_ila_commit_bundle_w[1];
    wire        ila_ccr_settle_in_flight  = dbg_ila_commit_bundle_w[0];

    // probe_5: {a7_illegitimate_step, rob_pop, rob_is_last_uop, mb_empty,
    //           drain_active} (5b — bit[4] is the canonical A7-drift trigger)
    // ila_a7_illegitimate_step is computed below (depends on ila_prev_a7
    // which is the same-cycle delta against arch_a7).  Forward-decl
    // here as a wire so the bundle assign type-checks; the actual
    // driver lives below.
    (* MARK_DEBUG = "true" *) wire ila_a7_illegitimate_step;
    wire [4:0]  ila_rob_pop_bundle = {ila_a7_illegitimate_step,
                                       dbg_ila_rob_pop_w,
                                       dbg_ila_rob_is_last_uop_w,
                                       ila_mb_empty,
                                       ila_drain_active};

    // probe_9: exception/trace 8-bit summary
    wire [7:0]  ila_exc_summary = {ila_store_commit_exc,
                                    ila_take_trace,
                                    ila_take_exc,
                                    ila_take_irq_arm,
                                    ila_take_irq_fire_q,
                                    ila_take_irq_preempt,
                                    ila_take_rte,
                                    ila_ccr_settle_in_flight};

    // ─── A7 illegitimate-step trigger.  Compute the cycle-to-cycle
    // delta of arch_a7 and assert when it is NOT one of the legitimate
    // 68k push/pop steps {±4, ±8, ±12, ±16, ±32, 0}.  Sampled on
    // every clock; the ILA's trigger setup can either gate on this
    // bit directly, or capture continuously and trigger via the VIO
    // probe_out0[2] (free debug bit) in HW Manager.
    reg [31:0] ila_prev_a7;
    always @(posedge core_clk) begin
        if (core_rst) ila_prev_a7 <= 32'd0;
        else          ila_prev_a7 <= dbg_ila_arch_a7_w;
    end
    wire [31:0] ila_a7_step = dbg_ila_arch_a7_w - ila_prev_a7;
    // Legitimate 68k SP steps: ±{4,8,12,16,32} and 0 (no change).
    // Compute against signed deltas.  Reading the spec, the user wants
    // the negative of the legitimate set too; sign-extend the small
    // immediates explicitly so 32-bit compares work.
    wire ila_a7_step_legit =
         (ila_a7_step == 32'sd0)
      || (ila_a7_step == 32'sd4)   || (ila_a7_step == -32'sd4)
      || (ila_a7_step == 32'sd8)   || (ila_a7_step == -32'sd8)
      || (ila_a7_step == 32'sd12)  || (ila_a7_step == -32'sd12)
      || (ila_a7_step == 32'sd16)  || (ila_a7_step == -32'sd16)
      || (ila_a7_step == 32'sd32)  || (ila_a7_step == -32'sd32);
    assign ila_a7_illegitimate_step =
        ~core_rst & ~ila_a7_step_legit & (dbg_ila_arch_a7_w != ila_prev_a7);

    // ─── AXI write-channel snapshot (32 + 8 = 40b) ────────────────────
    (* MARK_DEBUG = "true" *) wire [39:0] ila_axi_w_snap = {
        raw_daxi_awaddr,        // [39:8]
        raw_daxi_awvalid,       // [7]
        raw_daxi_awready,       // [6]
        raw_daxi_wvalid,        // [5]
        raw_daxi_wready,        // [4]
        raw_daxi_bvalid,        // [3]
        raw_daxi_bready,        // [2]
        raw_daxi_wlast,         // [1]
        dbg_ila_dc_aw_is_evict_w // [0]
    };

    // ─── Dcache writeback address probe (32b addr + 1b valid).  This
    // is the same wire as raw_daxi_awaddr at the moment the dcache
    // raises aw_valid, so we pack {raw_daxi_awaddr, raw_daxi_awvalid &
    // dbg_ila_dc_aw_is_evict_w}.  Provides a dedicated channel for
    // "writeback address" triggers independent of the more general
    // probe_15 above.
    (* MARK_DEBUG = "true" *) wire [32:0] ila_wb_addr = {
        raw_daxi_awaddr,
        raw_daxi_awvalid & dbg_ila_dc_aw_is_evict_w
    };

    // ─── v3 probe map (2026-05-26): PRF write-port observation.
    // Pack each CDB's {en, has_dst, phys[6:0]} into a 9-bit probe so
    // the trigger setup can match on (probe == 9'b1_1_0000001) to
    // catch a "writing phys 0x01 with valid dst" event in one shot.
    // probe19 (cdb_alu_hi) has no has_dst — the alu.v gate already
    // pre-ANDs mul_s3_has_dst_b into cdb_alu_hi_en — so the en bit is
    // already the effective "real dst write" condition.  We pack a
    // dummy 1 in bit[7] of probe19 to keep all CDB probes the same
    // 9-bit width for trigger-script uniformity.
    (* MARK_DEBUG = "true" *) wire [8:0] ila_cdb0_pack = {
        dbg_ila_cdb0_en_w,
        dbg_ila_cdb0_has_dst_w,
        dbg_ila_cdb0_phys_w
    };
    (* MARK_DEBUG = "true" *) wire [8:0] ila_cdb1_pack = {
        dbg_ila_cdb1_en_w,
        dbg_ila_cdb1_has_dst_w,
        dbg_ila_cdb1_phys_w
    };
    (* MARK_DEBUG = "true" *) wire [8:0] ila_cdb2_pack = {
        dbg_ila_cdb2_en_w,
        dbg_ila_cdb2_has_dst_w,
        dbg_ila_cdb2_phys_w
    };
    (* MARK_DEBUG = "true" *) wire [7:0] ila_cdb_alu_hi_pack = {
        dbg_ila_cdb_alu_hi_en_w,
        dbg_ila_cdb_alu_hi_phys_w
    };
    // a7_writeback: {en, phys[6:0], pad}.  Reserve a pad bit so the
    // probe is 9b like the CDB probes; the LSB is left as 'x' wire
    // tied to 0 since a7_writeback has no "has_dst" — when en=1 the
    // write fires unconditionally.
    (* MARK_DEBUG = "true" *) wire [8:0] ila_a7_wb_pack = {
        dbg_ila_a7_writeback_en_w,
        dbg_ila_a7_writeback_phys_w,
        1'b0
    };

    // ─── Derived TRIGGER: any PRF write to phys 0x01 (= post-rename
    // A7 phys we're tracking).  Used by the HW Manager as a one-bit
    // trigger so the setup is just "trigger when probe24 == 1".
    // Includes ALL five PRF write sources from m68k_core_execute.vh.
    // sp_slot writes target the pinned USP/SSP/ISP_TAG phys regs
    // (not phys 0x01) by construction, so they cannot corrupt the
    // committed-A7 phys — omitted from the OR here.  vec_ssp_valid
    // writes committed_a7_phys, so include it conditionally on the
    // committed_a7_phys equalling 0x01 at that moment.
    (* MARK_DEBUG = "true" *) wire ila_prf_write_targets_01 =
          (dbg_ila_cdb0_en_w        & dbg_ila_cdb0_has_dst_w
                                    & (dbg_ila_cdb0_phys_w        == 7'd1))
        | (dbg_ila_cdb1_en_w        & dbg_ila_cdb1_has_dst_w
                                    & (dbg_ila_cdb1_phys_w        == 7'd1))
        | (dbg_ila_cdb2_en_w        & dbg_ila_cdb2_has_dst_w
                                    & (dbg_ila_cdb2_phys_w        == 7'd1))
        | (dbg_ila_cdb_alu_hi_en_w  & (dbg_ila_cdb_alu_hi_phys_w  == 7'd1))
        | (dbg_ila_a7_writeback_en_w
                                    & (dbg_ila_a7_writeback_phys_w == 7'd1))
        | (dbg_ila_vec_ssp_valid_w  & (dbg_ila_committed_a7_phys_w == 7'd1));

    // ─── v4 probe map (2026-05-27) — true-A7 / snap-mux artifact bypass
    // + IF/BPU PC tap.  See synth/debug_ila.tcl probe24..probe29 and
    // docs/ila_a7_drift_probes.md.  These probes capture the truth
    // about A7 corruption that probe10 (=dbg_ila_arch_a7_w) gets wrong
    // when effective_halt asserts and the snap mux flips.  Trigger
    // recipe: set HW Manager trigger on `dbg_ila_effective_halt = 1`
    // and inspect `dbg_ila_real_a7_val` vs probe10 — if they differ,
    // probe10 was lying due to the snap-mux artifact.
    (* MARK_DEBUG = "true" *) wire [31:0] ila_real_a7_val   = dbg_ila_real_a7_val_w;
    (* MARK_DEBUG = "true" *) wire [31:0] ila_prf01_val     = dbg_ila_prf01_w;
    (* MARK_DEBUG = "true" *) wire [6:0]  ila_snap_prf_idx  = dbg_ila_snap_prf_idx_w;
    (* MARK_DEBUG = "true" *) wire        ila_effective_halt= dbg_ila_effective_halt_w;
    (* MARK_DEBUG = "true" *) wire [31:0] ila_if_pc         = dbg_ila_if_pc_w;
    (* MARK_DEBUG = "true" *) wire [31:0] ila_pred_pc       = dbg_ila_pred_pc_w;

    // ─── v5 probe map (2026-07-04) — VIA1 PA0/DDRA strap-read race hunt.
    // Chasing the STM/CTE factory-diagnostic-module HW divergence: ROM PC
    // 0x40846cc2 clears VIA1 DDRA bit0 (forces PA0 to read as an input),
    // then PC 0x40846cce immediately BTSTs VIA1 ORA-no-handshake bit0
    // (PA0).  RTL's pa_in strap is a hardcoded constant 8'hC1 (PA0=1),
    // matching MAME's default (diagnostic mode disabled) — so this read
    // should see bit0=1 and skip the STM/CTE entry gate (d7 bit26 stays
    // clear).  HW is nonetheless observed parking in the STM/CTE loop,
    // which requires d7 bit26 to have been set — which only happens if
    // this exact read saw bit0=0.  Hypothesis: the immediately-preceding
    // DDRA write (different MMIO offset, same VIA1 instance, pb_clk
    // domain, cross-clock from the CPU's core_clk) has not yet been
    // observed by via1.v's registers by the time the very next load
    // executes, so the read observes a stale ddra (bit0 still 1 = output)
    // and thus returns the OUTPUT LATCH bit (ora[0]) instead of the true
    // external strap (pa_in[0]).  Tap via1.v's raw ddra/ora registers +
    // the pb-side access strobes directly (hierarchical ref into u_via1,
    // instantiated in this same file) so a capture triggered on
    // `dbg_ila_rob_pc_w == 32'h40846cce` shows the actual ddra/ora state
    // VIA1 held at the moment of this specific read, independent of what
    // the CPU's own retiring register value shows.
    (* MARK_DEBUG = "true" *) wire [23:0] ila_via1_snap = {
        u_via1.ddra,      // [23:16] DDRA byte — bit0 should be 0 (input) here
        u_via1.ora,       // [15:8]  ORA output-latch byte
        pb_via1_wr,       // [7]
        pb_via1_rd,       // [6]
        pb_via1_ack,      // [5]
        5'b0
    };

    // ─── v6 probe map (2026-07-04) — A7/RTE-finalize investigation.
    // The post-DAFB A7-corruption bisect (docs/ila_a7_drift_probes.md /
    // the CPU repo's lost-store + IRQ-arm-at-RTE-pop bisect chain) wants
    // to bracket a live `take_rte_finalize` event and watch what phys
    // reg the exception/RTE machinery believes A7 lives in
    // (exc_held_a7_phys) alongside the already-present committed_a7_phys
    // (probe2) and a7_writeback_val (probe22).  probe32 is a dedicated
    // 1-bit trigger source — set HW Manager trigger on probe32 == 1 to
    // capture the cycle the RTE-pop atomic SR/A7/PC restore fires.
    (* MARK_DEBUG = "true" *) wire [6:0] ila_exc_held_a7_phys = dbg_ila_exc_held_a7_phys_w;
    (* MARK_DEBUG = "true" *) wire       ila_take_rte_finalize = dbg_ila_take_rte_finalize_w;
    (* MARK_DEBUG = "true" *) wire       ila_supervisor_mode   = dbg_ila_supervisor_mode_w;

    // ─── v7 probe map (2026-07-14) — MOVEA.L (SP),SP wild-jump /
    // finalize address-data crossover investigation.  exc_held_fault_pc
    // is the resume PC commit.v associates with the completing finalize
    // transaction; rob_brtgt is what take_finalize actually latches into
    // redirect_pc; dc_rdata is the raw LSU load data for that same
    // transaction.  Compare dc_rdata vs rob_brtgt at trigger time to
    // tell upstream-of-LSU (D-cache returned wrong data) from
    // downstream-in-ROB (completion-bus/LVT) corruption.
    //
    // Trigger recipe (Vivado HW Manager Tcl, runtime-configurable, NOT
    // an RTL rebuild): AND probe37 (take_finalize) == 1 with probe34
    // (exc_held_fault_pc) == 32'h408855e6 to isolate the ONE finalize
    // transaction under investigation among the many unrelated ones
    // firing during boot:
    //   set p34 [get_hw_probes u_dbg_ila/probe34 -of_objects [get_hw_ilas hw_ila_1]]
    //   set p37 [get_hw_probes u_dbg_ila/probe37 -of_objects [get_hw_ilas hw_ila_1]]
    //   set_property TRIGGER_COMPARE_VALUE eq32'h408855e6 $p34
    //   set_property TRIGGER_COMPARE_VALUE eq1'b1         $p37
    //   set_property CONTROL.TRIGGER_CONDITION AND [get_hw_ilas hw_ila_1]
    (* MARK_DEBUG = "true" *) wire [31:0] ila_exc_held_fault_pc = dbg_ila_exc_held_fault_pc_w;
    (* MARK_DEBUG = "true" *) wire [31:0] ila_rob_brtgt         = dbg_ila_rob_brtgt_w;
    (* MARK_DEBUG = "true" *) wire [31:0] ila_dc_rdata          = dbg_ila_dc_rdata_w;
    (* MARK_DEBUG = "true" *) wire        ila_take_finalize     = dbg_ila_take_finalize_w;

    // ─── v8 probe map (2026-07-14) — stale-ROB-slot / dispatch-clear
    // race investigation.  rob_brt (= commit_br_taken, the ROB's own
    // retiring-entry "take this branch" bit) settles whether v7's
    // rob_brtgt observation was even gated live at retire, or just a
    // stale LVT value nobody was supposed to read.  snap_tag/snap_ea
    // are the is_rts-gated LSU debug-snapshot's ROB tag and EA — cross
    // -reference against the retiring instruction's own tag (probe12
    // dbg_ila_rob_pc_w's associated tag isn't itself probed, but the
    // rob_pc value plus snap_tag/snap_ea together tell you whether the
    // is_rts completion captured here truly belongs to the retiring
    // MOVEA or is leftover from a different, earlier occupant of the
    // same ROB slot.
    (* MARK_DEBUG = "true" *) wire        ila_rob_brt  = dbg_ila_rob_brt_w;
    (* MARK_DEBUG = "true" *) wire [5:0]  ila_snap_tag = dbg_ila_snap_tag_w;
    (* MARK_DEBUG = "true" *) wire [31:0] ila_snap_ea  = dbg_ila_snap_ea_w;

    // ─── round 7 probe map (2026-08-30) — RobPlugin headReady/sysRetire
    // gate + coreHalted/haltReason/DcachePlugin-diagFault attribution (see
    // rtl/soc/fpga_top_debug_ctrl.vh's round-7 connection-site comment and
    // M68kCore.scala's dbg040 Area doc comment for the exact bit layout).
    // Two BRAND NEW probes (44/45), not a reused free slot.
    (* MARK_DEBUG = "true" *) wire [31:0] ila_robgate_state  = dbg_ila_robgate_state_w;
    (* MARK_DEBUG = "true" *) wire [31:0] ila_diag_fault_addr = dbg_ila_diag_fault_addr_w;

    // probe43 samples `mig_cal_done`, which is declared inside
    // fpga_top_ddr.vh's `ifndef SIM_MODEL arm -- there is no MIG at all in a
    // SIM_MODEL elaboration. probe43 referenced it unconditionally, so the
    // whole ILA block could not be linted under -DSIM_MODEL (which every
    // lint-configs row passes), and that is part of why the ILA arm's real
    // defect went unnoticed. A real ENABLE_ILA=1 bitstream does NOT define
    // SIM_MODEL, so the hardware behaviour is unchanged: this only gives the
    // linter a defined value to look at.
`ifdef SIM_MODEL
    wire ila_mig_cal_done = 1'b0;
`else
    wire ila_mig_cal_done = mig_cal_done;
`endif

    // ─── round 8 probe map (2026-08-30) — instruction-fetch AXI master
    // investigation (docs/BUG_calibration_word_misplaced_0d00.md Part
    // 54's precise closing recommendation). Round 7's real-hardware
    // capture DEFINITIVELY REFUTED the coreHalted hypothesis and found
    // the long-standing `dbg_ila_rob_pc_w == 0x40887126` trigger recipe
    // (unchanged since Part 19) fires on a STALE `readAsync` artifact,
    // not a live signal -- the ROB is genuinely, physically EMPTY
    // (count==0) at the hang, and the IF-stage fetch PC tap (probe28,
    // ila_if_pc == dbg_ila_if_pc_w) is independently frozen too, at a
    // DIFFERENT address 8 bytes earlier. This round retargets the
    // trigger onto a purpose-built "IF-stage fetch PC has not moved in
    // N cycles" detector (Part 54 S8's own suggested implementation) and
    // adds direct taps on BOTH the core-side fetch AXI master (ifa_* --
    // already top-level wires, fpga_top_cpu.vh, no cpu040 RTL change
    // needed) and the L2C-arbitrated fetch sub-port (l2c.v's brand-new
    // dbg_fetch_snap output, since neither of L2C's two pre-existing
    // debug snapshots -- dbg_l2c_write_snap/dbg_l2c_master_snap, probes
    // 41/42 -- carries fetch-vs-LSU source-tagged state).
    //
    //   probe46 16b   ila_fetch_snap = dbg_l2c_fetch_snap (l2c.v):
    //                   [0] f_axi_arvalid   [1] f_axi_arready
    //                   [2] hit_rsp_valid   [3] hit_rsp_is_fetch
    //                   [4] mshr_rsp_valid  [5] mshr_rsp_is_fetch
    //                   [6] fetch_q_avail   [7] fetch_consume_c
    //                   [8] f_rvalid_q      [9] f_axi_rready (CPU side)
    //                   [10] fq_pair_have (quadrant-pair straddling)
    //   probe47 14b   ila_ifetch_core_state -- the CPU-side (axi_i / ifa_*)
    //                 view, top-level wires, unaffected by anything
    //                 inside L2C:
    //                   [0] ifa_arvalid [1] ifa_arready
    //                   [2] ifa_rvalid  [3] ifa_rready [4] ifa_rlast
    //                   [8:5] ifa_arid[3:0]  [12:9] ifa_rid[3:0]
    //                   [13] reserved
    //   probe48 32b   ila_ifetch_araddr = ifa_araddr -- the address of
    //                 whatever AR is currently held/being presented.
    //   probe49 17b   ila_if_pc_stuck = {if_pc_stuck_q, if_pc_stall_cnt_q}:
    //                   [16] if_pc_stuck_q (STICKY once the fetch PC has
    //                        gone >= 512 consecutive cycles unchanged --
    //                        >4x Part 54 S6's own measured normal-case
    //                        maximum of 124 cycles, comfortable margin)
    //                   [15:0] if_pc_stall_cnt_q (saturating cycle count
    //                        since the fetch PC last changed)
    //
    //   Trigger recipe: probe49[16] (if_pc_stuck_q) == 1, TRIGGER_POSITION
    //   set near the END of the 4096-sample buffer (mostly PRE-trigger)
    //   so the capture shows the AR/R traffic history leading up to and
    //   through the moment the fetch PC stops advancing, not just the
    //   already-settled frozen state. Decision tree:
    //     - probe47[0] (ifa_arvalid) stuck at 1 with probe47[1]
    //       (ifa_arready) stuck at 0 -> a genuine bus hang: the CPU is
    //       presenting an AR nobody ever accepts. Cross-check probe46[1]
    //       (f_axi_arready, same signal downstream of the ROM-mirror
    //       address fold) to confirm L2C's front door itself is the
    //       one refusing, then probe41/42 (existing L2C snapshots) for
    //       WHY (id_busy_c/rst_busy/pipe_id_haz_c stuck, or the DRAM-side
    //       master port itself wedged).
    //     - probe47[0]/[1] both idle (no AR ever presented) -> the stall
    //       is upstream of the AXI master entirely -- inside cpu040's own
    //       IcachePlugin miss-dispatch/MSHR-allocate logic, never even
    //       reaching arHoldValid. Points at a cpu040 RTL bug, not an
    //       SoC/L2C/DRAM issue.
    //     - probe47[2] (ifa_rvalid) stuck at 1 with probe47[3]
    //       (ifa_rready) stuck at 0 -> a genuine core-side backpressure/
    //       credit bug: L2C delivered a response cpu040 never consumes.
    //       Cross-check probe46[8]/[9] (f_rvalid_q/f_axi_rready) to
    //       confirm the same beat is stuck at the L2C->CPU boundary, not
    //       manufactured downstream of it.
    //     - probe46[10] (fq_pair_have) stuck at 1 with no matching
    //       probe46[6]/[7] activity -> a stuck L2C-internal quadrant-pair
    //       reassembly (first quadrant captured, second never arrives) --
    //       an L2C RTL bug distinct from both of the above.
    wire        if_pc_stuck_q;
    wire [15:0] if_pc_stall_cnt_q;
    reg  [31:0] if_pc_prev_q;
    reg  [15:0] if_pc_stall_cnt_r;
    reg         if_pc_stuck_r;
    localparam IF_PC_STUCK_THRESH = 16'd512;
    always @(posedge core_clk) begin
        if (dbg_ila_if_pc_w != if_pc_prev_q) begin
            if_pc_stall_cnt_r <= 16'd0;
            if_pc_stuck_r     <= 1'b0;
        end else if (if_pc_stall_cnt_r < IF_PC_STUCK_THRESH) begin
            if_pc_stall_cnt_r <= if_pc_stall_cnt_r + 16'd1;
        end else begin
            if_pc_stuck_r <= 1'b1;
        end
        if_pc_prev_q <= dbg_ila_if_pc_w;
    end
    assign if_pc_stuck_q     = if_pc_stuck_r;
    assign if_pc_stall_cnt_q = if_pc_stall_cnt_r;

    (* MARK_DEBUG = "true" *) wire [15:0] ila_fetch_snap      = dbg_l2c_fetch_snap;
    (* MARK_DEBUG = "true" *) wire [13:0] ila_ifetch_core_state =
        {1'b0, ifa_rid[3:0], ifa_arid[3:0], ifa_rlast, ifa_rready,
         ifa_rvalid, ifa_arready, ifa_arvalid};
    (* MARK_DEBUG = "true" *) wire [31:0] ila_ifetch_araddr   = ifa_araddr;
    (* MARK_DEBUG = "true" *) wire [16:0] ila_if_pc_stuck     =
        {if_pc_stuck_q, if_pc_stall_cnt_q};

    // ─── round 9 probe map (2026-08-30) — I-cache MSHR per-slot
    // control-file investigation (docs/BUG_calibration_word_misplaced_
    // 0d00.md Part 55's precise closing recommendation). Part 55's
    // capture found a real, permanent, two-sided AXI protocol deadlock
    // on AXI ID=1 (IcachePlugin.scala's AxiIds.I_SPEC_BASE, the FIRST
    // speculative/prefetch MSHR slot of 5): L2C has a fully-resolved
    // cache-HIT fetch response for ID=1 sitting ready (probe46's
    // hit_rsp_valid/f_rvalid_q) that cpu040 never drains
    // (ifa_rready == probe47[3] permanently 0), while cpu040
    // SIMULTANEOUSLY holds a fresh, un-fired AR for the SAME id
    // targeting a NEW address (probe48/ila_ifetch_araddr) that L2C's
    // front door never accepts. Part 55 could not resolve statically
    // whether IcachePlugin.scala's own MSHR bookkeeping is internally
    // self-consistent (a legitimate free-then-reallocate of slot 1 to a
    // new target -- an L2C-side stale-response bug) or corrupt (slot 1's
    // mshrArSent/mshrComplete history is itself wrong -- a cpu040-side
    // bug). These three new probes expose exactly that:
    //
    //   probe50 32b   ila_ic_mshr_snap = dbg_ila_ic_mshr_snap_w (EXACT
    //                 mirror of M68kCore.scala's dbg040 Area `icMshrSnap`
    //                 layout -- see that file for the authoritative copy):
    //                   [0]     reserved
    //                   [5:1]   mshrArSent[4:0]   (bit (1+i) = slot i)
    //                   [10:6]  mshrComplete[4:0] (bit (6+i) = slot i)
    //                   [15:11] mshrErr[4:0]      (bit (11+i) = slot i)
    //                   [20:16] mshrPoison[4:0]   (bit (16+i) = slot i)
    //                   [23:21] fsmState (0=IDLE 1=REFILL 2=INSTALL_ARM
    //                                     3=PREDECODE 4=REPLAY 5=FAULT)
    //                   [24]    demandRspMatch  [25] pfRspMatch
    //                   [26]    ridIsDemand     [27] ridIsPf
    //                   [31:28] reserved
    //                 Slot 1 (AXI ID=1, the slot under investigation) is
    //                 always bit position (field_lsb + 1): mshrArSent[1]
    //                 = bit 2, mshrComplete[1] = bit 7, mshrErr[1] = bit
    //                 12, mshrPoison[1] = bit 17.
    //   probe51 32b   ila_ic_mshr_snap2 = dbg_ila_ic_mshr_snap2_w:
    //                   [4:0]   mshrValid[4:0] (bit i = slot i; slot 1 =
    //                           bit 1)
    //                   [31:5]  reserved
    //   probe52 32b   ila_ic_mshr_slot1_pa = dbg_ila_ic_mshr_slot1_pa_w
    //                   = mshrPa(AxiIds.I_SPEC_BASE) verbatim -- the
    //                   physical address MSHR slot 1 is CURRENTLY
    //                   tracking.
    //
    //   Decision procedure (no fresh cold-boot needed -- re-arm against
    //   the same already-frozen CPUSHL hang, per every prior round's own
    //   precedent):
    //     - probe51[1] (mshrValid[1]) == 0 at the SAME sample where
    //       probe46[2]/[3] (hit_rsp_valid/hit_rsp_is_fetch) == 1 -> slot
    //       1 was ALREADY FREED while L2C still thinks a response for it
    //       is outstanding. Since IcachePlugin.scala's only two free
    //       paths both require mshrComplete(1)==True first (a genuine,
    //       protocol-legal 2-beat AXI fire with axi.r.ready actually
    //       high), this would mean the core correctly drained an OLDER
    //       id=1 transaction and L2C's own fq_have/f_rvalid_q skid never
    //       cleared for it -- POINTS AT L2C (rtl/soc/l2c.v), not
    //       cpu040.
    //     - probe51[1] == 1 (slot 1 still valid) AND probe52
    //       (ila_ic_mshr_slot1_pa) == probe48 (ila_ifetch_araddr, the
    //       address of the AR currently held) -> slot 1 was NEVER
    //       reallocated; the AR in flight IS for the exact same
    //       occupancy whose response is stuck at L2C -- POINTS AT L2C
    //       (a front-door/response-drain bug on a single continuous
    //       occupancy), cpu040's own bookkeeping is self-consistent.
    //     - probe51[1] == 1 AND probe52 != probe48, with probe50[2]
    //       (mshrArSent[1]) == 0 -> slot 1 WAS reallocated to a NEW
    //       target (mshrPa/mshrSet/mshrTag rewritten by the pfFreeMshr
    //       arm) while an EARLIER occupancy's response was still live at
    //       L2C -- the smoking gun for a genuine cpu040-side premature-
    //       free/reallocation-before-drain bug. If ALSO probe50[7]
    //       (mshrComplete[1]) reads 0 at every sample of the stuck
    //       window (never observed transitioning 1 at any point this
    //       capture's pre-trigger history covers), that would mean the
    //       slot was freed via a path OTHER than the two known
    //       mshrComplete-gated free sites -- itself a second, deeper
    //       finding worth its own follow-up round.
    (* MARK_DEBUG = "true" *) wire [31:0] ila_ic_mshr_snap     = dbg_ila_ic_mshr_snap_w;
    (* MARK_DEBUG = "true" *) wire [31:0] ila_ic_mshr_snap2    = dbg_ila_ic_mshr_snap2_w;
    (* MARK_DEBUG = "true" *) wire [31:0] ila_ic_mshr_slot1_pa = dbg_ila_ic_mshr_slot1_pa_w;

    // ─── round 10 probe map (2026-08-31) — per-slot MSHR generation/tag
    // counters + a direct L2C fetch-ID tap (docs/BUG_calibration_word_
    // misplaced_0d00.md Part 56's precise closing recommendation). Part 56
    // structurally proved the DOWNSTREAM deadlock mechanism (IcachePlugin's
    // R-channel accept gate is ID/slot-indexed, not generation-indexed) but
    // could not settle whether the TRIGGER is cpu040 reallocating slot 1's
    // AXI ID before an older occupancy's response drained, or a genuine
    // L2C-side response-tracking defect. These three new probes settle it
    // directly:
    //
    //   probe53 32b   ila_ic_mshr_gen = dbg_ila_ic_mshr_gen_w
    //                   [19:0] mshrGen[4:0], 4 bits/slot, slot i in bits
    //                   [4*i+3:4*i] -- slot 1's generation is bits [7:4].
    //                   Increments (wrapping) every time IcachePlugin.scala
    //                   freshly allocates that slot (demand or speculative).
    //   probe54 32b   ila_ic_mshr_arsent_gen = dbg_ila_ic_mshr_arsent_gen_w
    //                   same packing as probe53. Latched to mshrGen(i)'s
    //                   value at the exact cycle `axi.ar.fire` sends slot
    //                   i's AR out onto the real bus -- NOT reset at
    //                   allocation, so it always holds the generation
    //                   number of the LAST AR actually accepted for this
    //                   slot, surviving any later reallocation.
    //   probe55 16b   ila_l2c_fetch_id_snap = dbg_l2c_fetch_id_snap (l2c.v):
    //                   [15] f_rvalid_q (output skid occupied)
    //                   [14] fetch_pick_hit (a fresh fetch-tagged hit is
    //                        live at L2C's front door this cycle)
    //                   [13:8] hit_rsp_id[5:0] (raw ID_WIDTH tag of that
    //                        fresh hit, if [14] is set)
    //                   [7:4] fq_idx (ID that fresh completion targets)
    //                   [3:0] f_rid_q (ID CURRENTLY parked in the output
    //                        skid register -- the orphaned response's own
    //                        AXI ID)
    //
    //   Decision procedure (no fresh cold-boot needed -- re-arm against the
    //   same already-frozen CPUSHL hang, per every prior round's own
    //   precedent -- this exact stuck state has reproduced byte-identically
    //   round over round since Part 55):
    //     - Read slot 1's nibble out of probe53 and probe54 (bits [7:4] of
    //       each). If probe54[7:4] != probe53[7:4] WHILE probe50[2]
    //       (mshrArSent[1]) reads 0 -> DEFINITIVE PROOF of a cpu040-side
    //       reallocate-before-drain: the currently-armed occupancy of slot
    //       1 is a STRICTLY NEWER generation than the last one whose AR was
    //       actually accepted onto the bus, i.e. IcachePlugin.scala freed
    //       and reallocated the slot while an older AR's response was still
    //       outstanding. This settles Part 56's S8 root-trigger question in
    //       cpu040's favour (against it, i.e. cpu040 IS the trigger).
    //     - If probe54[7:4] == probe53[7:4] instead (no reallocation
    //       happened since the last AR-accept on this slot), the orphaned
    //       response cannot be a cpu040-side premature-free artifact -- the
    //       root trigger must be explained on the L2C side instead. Cross-
    //       check probe55[3:0] (f_rid_q) == 1 to confirm L2C's own output
    //       skid genuinely believes it is holding an ID=1 response, and
    //       probe55[14] (fetch_pick_hit) to see whether L2C's front door is
    //       STILL actively trying to serve a *fresh* ID=1 hit this cycle
    //       (would point at a genuine L2C response-tracking defect, e.g.
    //       the skid register never actually clearing despite a real drain,
    //       or mis-tagging a completion with the wrong ID).
    (* MARK_DEBUG = "true" *) wire [31:0] ila_ic_mshr_gen         = dbg_ila_ic_mshr_gen_w;
    (* MARK_DEBUG = "true" *) wire [31:0] ila_ic_mshr_arsent_gen  = dbg_ila_ic_mshr_arsent_gen_w;
    (* MARK_DEBUG = "true" *) wire [15:0] ila_l2c_fetch_id_snap   = dbg_l2c_fetch_id_snap;

    // ─── round 12 probe (2026-08-31/09-01) — direct RDATA tap on the
    // orphaned fetch-reassembly skid register (docs/
    // BUG_calibration_word_misplaced_0d00.md Part 58's precise closing
    // recommendation, itself repeating Part 57 S9's own recommendation).
    // Every round 8-11 capture has shown, frozen: f_rvalid_q=1 (l2c.v's
    // one-deep output skid IS occupied) and f_axi_rready=0 (cpu040 never
    // drains it) for a response tagged AXI ID=1 (f_rid_q=1, matching I-
    // cache MSHR slot 1). Rounds 9-11 settled the ID/generation-indexed
    // bookkeeping around this orphan as far as counters alone can, but
    // could not decide between two remaining theories: (a) a cascading
    // mistaken-identity accept -- the data actually parked in the skid is
    // a STALE response for a DIFFERENT (earlier) fetch than the one
    // currently armed in MSHR slot 1 (probe52/ila_ic_mshr_slot1_pa) -- or
    // (b) the data genuinely IS the correct response for the
    // currently-armed address, just never drained (a pure downstream-
    // propagation/backpressure bug with no mistaken identity involved).
    // `ifa_rdata` is fpga_top_ddr.vh's own top-level wire connecting
    // l2c.v's `f_axi_rdata` (== the registered `f_rdata_q` skid payload
    // whenever `f_axi_rvalid`/`f_rvalid_q` reads 1, per l2c.v's own
    // `assign f_axi_rdata = f_rdata_q;`) straight into cpu040's fetch AXI
    // R-data input -- already wired at the top level, no l2c.v or
    // cpu040 port changes needed, only a new MARK_DEBUG tap here.
    //
    //   probe56 256b  ila_ifetch_rdata = ifa_rdata -- the FULL 256-bit
    //       reassembled instruction-fetch beat currently parked in the
    //       orphaned skid register (only meaningful while probe46[8]
    //       (f_rvalid_q) reads 1, true in every capture since Part 55).
    //
    //   Decision procedure (single static capture, re-arm against the
    //   same already-frozen hang per every prior round's own precedent):
    //     - Read probe52 (ila_ic_mshr_slot1_pa) -- the physical address
    //       MSHR slot 1 currently believes it is waiting on.
    //     - Independently look up (OFFLINE, from the exact ROM image
    //       deployed on the SD card this session, per the
    //       AXI_ROM_MIRROR_BASE/AXI_ROM_IMAGE_SIZE fold fpga_top_ddr.vh's
    //       `ifa_araddr_folded` already applies) the 32 bytes (256 bits)
    //       of REAL ROM content that byte address actually holds.
    //     - Compare against probe56 (ila_ifetch_rdata), byte-for-byte:
    //         MATCH    -> the parked data genuinely IS the response for
    //                     the currently-armed address; hypothesis (a) is
    //                     RULED OUT for this capture -- the bug is a
    //                     pure non-draining/backpressure defect (b), not
    //                     a mistaken-identity accept.
    //         MISMATCH -> DIRECT, DISPOSITIVE PROOF of hypothesis (a):
    //                     the response sitting in the skid belongs to a
    //                     DIFFERENT (necessarily earlier, per Part 57
    //                     S7's own "the orphan must predate generation
    //                     3" argument) fetch than the one MSHR slot 1 is
    //                     currently, actively waiting on.
    (* MARK_DEBUG = "true" *) wire [255:0] ila_ifetch_rdata = ifa_rdata;

    (* DONT_TOUCH = "true" *) debug_ila u_dbg_ila (
        .clk     (core_clk),
        .probe0  (ila_a7_macro_locked),       // 1b
        .probe1  (ila_a7_macro_pre_phys),     // 7b
        .probe2  (ila_committed_a7_phys),     // 7b
        .probe3  (ila_mb_drop),               // 1b
        .probe4  (dbg_ila_flush_en_w),        // 1b
        .probe5  (ila_rob_pop_bundle),        // 5b
        .probe6  (dbg_ila_rob_arch_dst_w),    // 5b
        .probe7  (dbg_ila_rob_phys_old_w),    // 7b
        .probe8  (dbg_ila_rob_phys_dst_w),    // 7b
        .probe9  (ila_exc_summary),           // 8b
        .probe10 (dbg_ila_arch_a7_w),         // 32b
        .probe11 (dbg_committed[15:0]),       // 16b
        .probe12 (dbg_ila_rob_pc_w),          // 32b
        .probe13 (ila_wb_addr),               // 33b — wb addr + valid
        .probe14 (ila_axi_w_snap),            // 40b — AXI W snapshot
        // v3 probe map (2026-05-26) — PRF write-port observation.
        .probe15 (ila_cdb0_pack),             // 9b — cdb0 {en,hd,phys[6:0]}
        .probe16 (ila_cdb1_pack),             // 9b — cdb1 {en,hd,phys[6:0]}
        .probe17 (ila_cdb2_pack),             // 9b — cdb2 {en,hd,phys[6:0]}
        .probe18 (ila_cdb_alu_hi_pack),       // 8b — alu_hi {en,phys[6:0]}
        .probe19 (ila_a7_wb_pack),            // 9b — a7_wb {en,phys[6:0],0}
        .probe20 (dbg_ila_cdb0_data_w),       // 32b — cdb0 data
        .probe21 (dbg_ila_cdb1_data_w),       // 32b — cdb1 data
        .probe22 (dbg_ila_a7_writeback_val_w),// 32b — a7_wb value
        .probe23 (ila_prf_write_targets_01),  // 1b — derived trigger
        // v4 probe map (2026-05-27) — snap-mux bypass + IF/BPU PC tap.
        .probe24 (ila_real_a7_val),           // 32b — RAW prf[committed_a7_phys]
        .probe25 (ila_prf01_val),             // 32b — RAW prf[7'd1]
        .probe26 (ila_snap_prf_idx),          // 7b  — snap chain index
        .probe27 (ila_effective_halt),        // 1b  — halt -> snap-mux flip
        .probe28 (ila_if_pc),                 // 32b — IF-stage fetch PC
        .probe29 (ila_pred_pc),               // 32b — BPU/RAS pred target
        // v5 probe map (2026-07-04) — VIA1 PA0/DDRA strap-read race hunt.
        .probe30 (ila_via1_snap),             // 24b — {ddra[7:0],ora[7:0],wr,rd,ack,5'b0}
        // v6 probe map (2026-07-04) — A7/RTE-finalize investigation.
        .probe31 (ila_exc_held_a7_phys),      // 7b  — exc_held_a7_phys
        .probe32 (ila_take_rte_finalize),     // 1b  — take_rte_finalize (trigger)
        .probe33 (ila_supervisor_mode),       // 1b  — arch_sr.S (committed)
        // v7 probe map (2026-07-14) — finalize address-data crossover.
        .probe34 (ila_exc_held_fault_pc),     // 32b — resume/interrupted PC
        .probe35 (ila_rob_brtgt),             // 32b — what commit.v latches into redirect_pc
        .probe36 (ila_dc_rdata),              // 32b — raw LSU load data
        .probe37 (ila_take_finalize),         // 1b  — take_finalize (trigger)
        // v8 probe map (2026-07-14) — stale-ROB-slot / dispatch-clear race.
        .probe38 (ila_rob_brt),               // 1b  — commit_br_taken (retiring entry)
        .probe39 (ila_snap_tag),              // 6b  — is_rts LSU snapshot's own ROB tag
        .probe40 (ila_snap_ea),               // 32b — is_rts LSU snapshot's EA
        // v9 probe map (2026-07-24) — L2C/MIG-cal-race investigation:
        // boot_fsm's first write into the L2C-fronted DDR path hangs on
        // real HW (SD-loader LED flashes briefly then stops). See
        // l2c_ctrl.v/l2c.v/fpga_top_ddr.vh's dbg_l2c_write_snap /
        // dbg_l2c_master_snap / mig_cal_done comments for the theory.
        .probe41 (dbg_l2c_write_snap),        // 11b — l2c_ctrl.v front-door state
        .probe42 (dbg_l2c_master_snap),       // 11b — l2c.v master-port + miss-fill state
        .probe43 (ila_mig_cal_done),              // 1b  — DDR4 MIG calibration status
        // v10 probe map (2026-08-30) — RobPlugin headReady/sysRetire gate +
        // coreHalted attribution investigation (round 7, see above).
        .probe44 (ila_robgate_state),         // 32b — RobPlugin/DcachePlugin packed state
        .probe45 (ila_diag_fault_addr),       // 32b — DcachePlugin diagFaultAddr
        // v11 probe map (2026-08-30) — instruction-fetch AXI master
        // investigation (round 8, see the ila_fetch_snap/ila_ifetch_*/
        // ila_if_pc_stuck declaration comment above for the full bit
        // layout and decision tree).
        .probe46 (ila_fetch_snap),            // 16b — l2c.v fetch-sourced state
        .probe47 (ila_ifetch_core_state),     // 14b — axi_i (ifa_*) core-side state
        .probe48 (ila_ifetch_araddr),         // 32b — ifa_araddr
        .probe49 (ila_if_pc_stuck),           // 17b — {stuck, stall_cnt} (trigger source)
        // v12 probe map (2026-08-30) — I-cache MSHR per-slot control-file
        // investigation (round 9, see the ila_ic_mshr_snap/_snap2/
        // _slot1_pa declaration comment above for the full bit layout
        // and decision procedure).
        .probe50 (ila_ic_mshr_snap),          // 32b — mshrArSent/Complete/Err/Poison + fsm/rspMatch
        .probe51 (ila_ic_mshr_snap2),         // 32b — mshrValid[4:0]
        .probe52 (ila_ic_mshr_slot1_pa),      // 32b — mshrPa(slot 1) verbatim
        // v13 probe map (2026-08-31) — per-slot MSHR generation/tag
        // counters + direct L2C fetch-ID tap (round 10, see the
        // ila_ic_mshr_gen/_arsent_gen/_l2c_fetch_id_snap declaration
        // comment above for the full bit layout and decision procedure).
        .probe53 (ila_ic_mshr_gen),           // 32b — mshrGen[4:0], 4 bits/slot
        .probe54 (ila_ic_mshr_arsent_gen),    // 32b — mshrArSentGen[4:0], 4 bits/slot
        .probe55 (ila_l2c_fetch_id_snap),     // 16b — l2c.v f_rid_q/fq_idx/hit_rsp_id
        // v14 probe map (2026-08-31/09-01) — direct RDATA tap on the
        // orphaned fetch-reassembly skid (round 12, see the
        // ila_ifetch_rdata declaration comment above for the full
        // rationale and decision procedure).
        .probe56 (ila_ifetch_rdata)           // 256b — ifa_rdata (parked skid payload)
    );

`endif
