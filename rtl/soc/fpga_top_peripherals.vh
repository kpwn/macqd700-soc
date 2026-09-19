// rtl/fpga_top_peripherals.vh — included from rtl/fpga_top.v
//
// Peripheral AXI bus (xbar S1) -> pb_clk Mac MMIO island
// {VIA1, VIA2, ENET, SONIC, Orwell, SCC, SCSI, ASC, SWIM/IWM} plus the
// debug_ctrl service BAR.  sd_provision was removed (boot is via
// JTAG-AXI now; the SD card is flashed by the host before power-up).
// DAFB registers are no longer behind S1; xbar S4 bridges directly to
// the DAFB AXI-Lite shim.
//
// Generated mechanically from rtl/fpga_top.v during the fpga_top split
// (agent/p3-fpga-top-split). No functional change: this file contains
// the original lines verbatim, inside the fpga_top module scope via
// `include. Do not add module/endmodule wrappers or nettype directives.

    // ═══════════════════════════════════════════════════════════════════
    // Peripheral bus (S1) — crosses into pb_clk and fans out to Mac
    // peripherals (VIA1/VIA2/SCC/SCSI/ASC + ADB alias) plus the
    // debug_ctrl service BAR (sd_provision was removed; SD provisioning
    // is now host-side, before power-up).  See docs/peripheral_arch.md
    // for the I/O-region address map.
    // ═══════════════════════════════════════════════════════════════════

    // pb-clk synchroniser for the JTAG/btn debug-full-reset.  Forward-
    // declared here (instead of next to via1) so the AXI async bridge
    // below can take pb_full_rst on its m_rst port without forward-using
    // a wire whose driver appears further down the file.  The actual
    // sync FFs and `assign pb_full_rst` line live near the original
    // via1_rst_pb declaration.
    wire pb_full_rst;
    // 4-bit wire bus that mirrors pb_full_rst.  Driven from a SINGLE
    // FF (`pb_full_rst_q`, declared further down) tagged with
    // `MAX_FANOUT = 256` so Vivado replicates the source automatically
    // — no manual banking.  All bits hold the identical Q value as
    // `pb_full_rst` one cycle later; the bus is kept for source-
    // compatibility with consumers that already index `[N]`.
    wire [3:0] pb_full_rst_bank;
    wire [19:0] pb_dbg_awaddr, pb_dbg_araddr;
    wire        pb_dbg_awvalid, pb_dbg_awready;
    wire [31:0] pb_dbg_wdata;  wire [3:0] pb_dbg_wstrb;
    wire        pb_dbg_wvalid, pb_dbg_wready;
    wire [1:0]  pb_dbg_bresp;
    wire        pb_dbg_bvalid, pb_dbg_bready;
    wire        pb_dbg_arvalid, pb_dbg_arready;
    wire [31:0] pb_dbg_rdata;  wire [1:0] pb_dbg_rresp;
    wire        pb_dbg_rvalid, pb_dbg_rready;

    // pb_* handshake wires for Mac peripherals
    wire [3:0]  pb_via1_addr;  wire [7:0] pb_via1_wdata;
    wire        pb_via1_wr;    wire       pb_via1_rd;
    wire [7:0]  pb_via1_rdata; wire       pb_via1_ack;

    wire [3:0]  pb_via2_addr;  wire [7:0] pb_via2_wdata;
    wire        pb_via2_wr;    wire       pb_via2_rd;
    wire [7:0]  pb_via2_rdata; wire       pb_via2_ack;

    wire [2:0]  pb_enet_addr;  wire [7:0] pb_enet_wdata;
    wire        pb_enet_wr;    wire       pb_enet_rd;
    wire [7:0]  pb_enet_rdata; wire       pb_enet_ack;

    wire [5:0]  pb_sonic_addr; wire [15:0] pb_sonic_wdata;
    wire [1:0]  pb_sonic_wstrb;
    wire        pb_sonic_wr;   wire       pb_sonic_rd;
    wire [15:0] pb_sonic_rdata; wire      pb_sonic_ack;

    wire [7:0]  pb_orwell_addr; wire [7:0] pb_orwell_wdata;
    wire        pb_orwell_wr;   wire       pb_orwell_rd;
    wire [7:0]  pb_orwell_rdata; wire      pb_orwell_ack;

    wire [3:0]  pb_scc_addr;   wire [7:0] pb_scc_wdata;
    wire        pb_scc_wr;     wire       pb_scc_rd;
    wire [7:0]  pb_scc_rdata;  wire       pb_scc_ack;
    (* ASYNC_REG = "TRUE" *) reg scc_uart_sel_meta;
    (* ASYNC_REG = "TRUE" *) reg scc_uart_sel_pb;
    always @(posedge pb_clk) begin
        if (pb_rst) begin
            scc_uart_sel_meta <= 1'b0;
            scc_uart_sel_pb   <= 1'b0;
        end else begin
            scc_uart_sel_meta <= jtag_scc_uart_sel_b;
            scc_uart_sel_pb   <= scc_uart_sel_meta;
        end
    end
    wire        scc_uart_sel_b = scc_uart_sel_pb;
    wire        scc_uart_sel_a = ~scc_uart_sel_b;
    wire        scc_uart_rx_valid;
    wire [7:0]  scc_uart_rx_data;
    wire        scc_uart_tx_valid = scc_uart_sel_b ? scc_tx_b_valid : scc_tx_a_valid;
    wire [7:0]  scc_uart_tx_data  = scc_uart_sel_b ? scc_tx_b_data  : scc_tx_a_data;
    wire        scc_uart_tx_busy;
    wire        scc_tx_a_valid;
    wire [7:0]  scc_tx_a_data;
    wire        scc_tx_b_valid;
    wire [7:0]  scc_tx_b_data;
    wire        scc_rts_a_n;
    wire        scc_dtr_a_n;
    wire        scc_rts_b_n;
    wire        scc_dtr_b_n;

    wire [8:0]  pb_scsi_addr;  wire [7:0] pb_scsi_wdata;
    wire        pb_scsi_wr;    wire       pb_scsi_rd;
    wire [7:0]  pb_scsi_rdata; wire       pb_scsi_ack;

    wire [11:0] pb_asc_addr;   wire [7:0] pb_asc_wdata;
    wire        pb_asc_wr;     wire       pb_asc_rd;
    wire [7:0]  pb_asc_rdata;  wire       pb_asc_ack;

    wire [3:0]  pb_iwm_addr;   wire [7:0] pb_iwm_wdata;
    wire        pb_iwm_wr;     wire       pb_iwm_rd;
    wire [7:0]  pb_iwm_rdata;  wire       pb_iwm_ack;

    // ADB injection MMIO at 0x5001_1000..0x5001_1FFF (mac_off 0x011000).
    // Byte-granular read/write window into the keyboard FIFO + mouse
    // accumulator state.  See rtl/mac/adb_inject.v for the register map.
    wire [7:0]  pb_adbinj_addr;  wire [7:0] pb_adbinj_wdata;
    wire        pb_adbinj_wr;    wire       pb_adbinj_rd;
    wire [7:0]  pb_adbinj_rdata; wire       pb_adbinj_ack;

    wire [31:0] pb_dafb_awaddr, pb_dafb_wdata, pb_dafb_araddr;
    wire [3:0]  pb_dafb_wstrb;
    wire        pb_dafb_awvalid, pb_dafb_wvalid, pb_dafb_bready;
    wire        pb_dafb_arvalid, pb_dafb_rready;

    wire [19:0] dbg_core_awaddr, dbg_core_araddr;
    wire        dbg_core_awvalid, dbg_core_awready;
    wire [31:0] dbg_core_wdata;  wire [3:0] dbg_core_wstrb;
    wire        dbg_core_wvalid, dbg_core_wready;
    wire [1:0]  dbg_core_bresp;
    wire        dbg_core_bvalid, dbg_core_bready;
    wire        dbg_core_arvalid, dbg_core_arready;
    wire [31:0] dbg_core_rdata;  wire [1:0] dbg_core_rresp;
    wire        dbg_core_rvalid, dbg_core_rready;

`ifdef ETH_DEBUG_ENABLE
    wire [19:0] dbg_mux_awaddr, dbg_mux_araddr;
    wire dbg_mux_awvalid, dbg_mux_awready, dbg_mux_wvalid, dbg_mux_wready;
    wire [31:0] dbg_mux_wdata; wire [3:0] dbg_mux_wstrb;
    wire [1:0] dbg_mux_bresp; wire dbg_mux_bvalid, dbg_mux_bready;
    wire dbg_mux_arvalid, dbg_mux_arready;
    wire [31:0] dbg_mux_rdata; wire [1:0] dbg_mux_rresp;
    wire dbg_mux_rvalid, dbg_mux_rready;
`endif

    wire [5:0]   pb_s1_awid;
    wire [31:0]  pb_s1_awaddr;
    wire [7:0]   pb_s1_awlen;
    wire [2:0]   pb_s1_awsize;
    wire [1:0]   pb_s1_awburst;
    wire         pb_s1_awvalid;
    wire         pb_s1_awready;
    wire [127:0] pb_s1_wdata;
    wire [15:0]  pb_s1_wstrb;
    wire         pb_s1_wlast;
    wire         pb_s1_wvalid;
    wire         pb_s1_wready;
    wire [5:0]   pb_s1_bid;
    wire [1:0]   pb_s1_bresp;
    wire         pb_s1_bvalid;
    wire         pb_s1_bready;
    wire [5:0]   pb_s1_arid;
    wire [31:0]  pb_s1_araddr;
    wire [7:0]   pb_s1_arlen;
    wire [2:0]   pb_s1_arsize;
    wire [1:0]   pb_s1_arburst;
    wire         pb_s1_arvalid;
    wire         pb_s1_arready;
    wire [5:0]   pb_s1_rid;
    wire [127:0] pb_s1_rdata;
    wire [1:0]   pb_s1_rresp;
    wire         pb_s1_rlast;
    wire         pb_s1_rvalid;
    wire         pb_s1_rready;

    wire s1_buser_unused;
    wire s1_ruser_unused;
    wire pb_s1_awlock_unused;
    wire [3:0] pb_s1_awcache_unused;
    wire [2:0] pb_s1_awprot_unused;
    wire [3:0] pb_s1_awqos_unused;
    wire pb_s1_awuser_unused;
    wire pb_s1_wuser_unused;
    wire pb_s1_arlock_unused;
    wire [3:0] pb_s1_arcache_unused;
    wire [2:0] pb_s1_arprot_unused;
    wire [3:0] pb_s1_arqos_unused;
    wire pb_s1_aruser_unused;

    // ── S1 CDC payload width (Phase 1a, docs/soc_bus_review.md §8) ───
    // The S1 bridge used to carry a 128-bit payload across core_clk ->
    // pb_clk to reach peripherals whose widest register is 8 bits.  Its
    // W and R FIFOs are 16 entries deep, so that width cost ~285 LUT of
    // LUTRAM and 95 of the 131 timing-path endpoints this instance
    // contributes (measured in
    // synth/timing_reports/timing_20260819_131707.rpt: 58 endpoints in
    // w_fifo + 49 in r_fifo, of which 95 are `mem_reg_*` data arrays).
    //
    // `axi_pb_s1_cdc` is port- and parameter-compatible with
    // `axi_async_bridge` and simply sandwiches the SAME bridge, carrying
    // 32 payload bits, between two combinational lane shims
    // (rtl/soc/axi_pb_lane_shim.v).  The CDC's gray-pointer crossing,
    // T6 coupled-reset handshake, T14 stale-beat sinks and multi-entry
    // AW/AR pre-staging are unchanged, and the shims have no reset of
    // their own -- so the deliberate choice below to hold this bridge on
    // the *core* reset banks (keeping the host path to debug_ctrl alive
    // across a JTAG/button full reset) is preserved exactly.
    //
    // ESCAPE HATCH: define PB_S1_WIDE_CDC to restore the original
    // 128-bit-payload bridge without an RTL edit.  Nothing else in the
    // instantiation below changes between the two.
    //
    // GREP NOTE: the instance below is elaborated through the
    // `PB_S1_CDC_MODULE macro, so `grep "axi_pb_s1_cdc #("` will NOT
    // find it and the module can look dead.  It is not: the live
    // instance is `u_pb_s1_cdc` a few lines down.  Both candidate module
    // names appear literally in the `ifdef pair immediately below, so a
    // plain `grep axi_pb_s1_cdc rtl/` does find this site.
`ifdef PB_S1_WIDE_CDC
  `define PB_S1_CDC_MODULE axi_async_bridge
`else
  `define PB_S1_CDC_MODULE axi_pb_s1_cdc
`endif
    `PB_S1_CDC_MODULE #(
        .DATA_WIDTH(128),
        .ADDR_WIDTH(32),
        .ID_WIDTH  (6),
        .USER_WIDTH(1)
    ) u_pb_s1_cdc (
        // ── S1 IS IN THE SoC RESET DOMAIN (2026-09-12, work item 3) ──
        // This bridge used to sit on core_rst_bank[1] / pb_core_rst_bank[3]
        // so it would survive a soc_full_rst, for ONE reason: the
        // 0x5090_0000 debug window was behind S1, and the host had to be
        // able to reach debug_ctrl across the reset it had just requested.
        //
        // That is what made the reset strand its own write.  MEASURED
        // 2026-09-12: after a JTAG `reset`, dbg_s1_slot_busy = 1 (0 while
        // running) with dbg_slv_poisoned = 0x00 throughout -- the crossbar
        // slot is stranded, and with ENABLE_WD = 0 nothing releases it, so
        // S1 goes write-dead for every master until core_rst.
        //
        // The debug window now lands on axi_dbg_bus instead
        // (fpga_top_debug_host.vh), so nothing needs S1 alive any more.
        // Both sides of this bridge take the soc_full_rst event, which is
        // the same signal the crossbar gets as `slv_flush` -- so an
        // in-flight S1 transaction is flush-aborted with a clean local
        // SLVERR exactly the way S2/S4/S5 already are, instead of being
        // abandoned with no release path.
        //
        // The m-side takes pb_soc_full_rst_bank, NOT pb_full_rst_bank: the
        // latter also carries warm_peripheral_reset (a 68040 RESET
        // instruction), which does NOT reset the CPU and does NOT raise
        // slv_flush.  Resetting the S1 front door under a still-running CPU
        // would create a brand-new stranding class.
        .s_clk(core_clk),
        .s_rst(soc_full_rst_bank[1]),
        .s_awid   (s1_awid   ),
        .s_awaddr (s1_awaddr ),
        .s_awlen  (s1_awlen  ),
        .s_awsize (s1_awsize ),
        .s_awburst(s1_awburst),
        .s_awlock (1'b0),
        .s_awcache(4'd0),
        .s_awprot (3'd0),
        .s_awqos  (4'd0),
        .s_awuser (1'b0),
        .s_awvalid(s1_awvalid),
        .s_awready(s1_awready),
        .s_wdata  (s1_wdata  ),
        .s_wstrb  (s1_wstrb  ),
        .s_wlast  (s1_wlast  ),
        .s_wuser  (1'b0),
        .s_wvalid (s1_wvalid ),
        .s_wready (s1_wready ),
        .s_bid    (s1_bid    ),
        .s_bresp  (s1_bresp  ),
        .s_buser  (s1_buser_unused),
        .s_bvalid (s1_bvalid ),
        .s_bready (s1_bready ),
        .s_arid   (s1_arid   ),
        .s_araddr (s1_araddr ),
        .s_arlen  (s1_arlen  ),
        .s_arsize (s1_arsize ),
        .s_arburst(s1_arburst),
        .s_arlock (1'b0),
        .s_arcache(4'd0),
        .s_arprot (3'd0),
        .s_arqos  (4'd0),
        .s_aruser (1'b0),
        .s_arvalid(s1_arvalid),
        .s_arready(s1_arready),
        .s_rid    (s1_rid    ),
        .s_rdata  (s1_rdata  ),
        .s_rresp  (s1_rresp  ),
        .s_rlast  (s1_rlast  ),
        .s_ruser  (s1_ruser_unused),
        .s_rvalid (s1_rvalid ),
        .s_rready (s1_rready ),

        .m_clk(pb_clk),
        .m_rst(pb_soc_full_rst_bank[3]),  // pb-bank [3] (axi_async + bridges)
        .m_awid   (pb_s1_awid   ),
        .m_awaddr (pb_s1_awaddr ),
        .m_awlen  (pb_s1_awlen  ),
        .m_awsize (pb_s1_awsize ),
        .m_awburst(pb_s1_awburst),
        .m_awlock (pb_s1_awlock_unused),
        .m_awcache(pb_s1_awcache_unused),
        .m_awprot (pb_s1_awprot_unused),
        .m_awqos  (pb_s1_awqos_unused),
        .m_awuser (pb_s1_awuser_unused),
        .m_awvalid(pb_s1_awvalid),
        .m_awready(pb_s1_awready),
        .m_wdata  (pb_s1_wdata  ),
        .m_wstrb  (pb_s1_wstrb  ),
        .m_wlast  (pb_s1_wlast  ),
        .m_wuser  (pb_s1_wuser_unused),
        .m_wvalid (pb_s1_wvalid ),
        .m_wready (pb_s1_wready ),
        .m_bid    (pb_s1_bid    ),
        .m_bresp  (pb_s1_bresp  ),
        .m_buser  (1'b0),
        .m_bvalid (pb_s1_bvalid ),
        .m_bready (pb_s1_bready ),
        .m_arid   (pb_s1_arid   ),
        .m_araddr (pb_s1_araddr ),
        .m_arlen  (pb_s1_arlen  ),
        .m_arsize (pb_s1_arsize ),
        .m_arburst(pb_s1_arburst),
        .m_arlock (pb_s1_arlock_unused),
        .m_arcache(pb_s1_arcache_unused),
        .m_arprot (pb_s1_arprot_unused),
        .m_arqos  (pb_s1_arqos_unused),
        .m_aruser (pb_s1_aruser_unused),
        .m_arvalid(pb_s1_arvalid),
        .m_arready(pb_s1_arready),
        .m_rid    (pb_s1_rid    ),
        .m_rdata  (pb_s1_rdata  ),
        .m_rresp  (pb_s1_rresp  ),
        .m_rlast  (pb_s1_rlast  ),
        .m_ruser  (1'b0),
        .m_rvalid (pb_s1_rvalid ),
        .m_rready (pb_s1_rready )
    );

    wire unused_pb_s1_sideband = &{1'b0, s1_buser_unused, s1_ruser_unused,
        pb_s1_awlock_unused, pb_s1_awcache_unused, pb_s1_awprot_unused,
        pb_s1_awqos_unused, pb_s1_awuser_unused, pb_s1_wuser_unused,
        pb_s1_arlock_unused, pb_s1_arcache_unused, pb_s1_arprot_unused,
        pb_s1_arqos_unused, pb_s1_aruser_unused};

    peripheral_bus #(
        .ID_WIDTH  (6),
        .DATA_WIDTH(128),
        // Ack-timeout watchdog: 2^24 pb-clk cycles (~335 ms @ 50 MHz).
        // Generous on purpose — a legitimate DRQ-gated SCSI DMA-shim
        // beat can wait on real SD-card latency (>100 ms worst-case);
        // do NOT tighten this for sim convenience (testbenches override
        // it in their own builds instead).
        .PB_WATCHDOG_LOG2(24),
        // Peripheral-reset barrier tail.  4 pb_clk cycles covers the
        // 2-FF dafb_scsi0_ctrl_sync_q reload above (which is reset by the
        // same pb_full_rst broadcast as u_scsi, and whose reset state
        // makes scsi.v report "DMA ready" regardless of what the DAFB
        // programmed) with 2 cycles of margin.
        .PERIPH_RST_TAIL(4)
    ) u_pbus (
        // Resets WITH the SoC, as of 2026-09-12 (work item 3).  It used to
        // stay alive across a JTAG/btn full-reset so debug_ctrl -- reached
        // through this bus at 0x5090_0000 -- could accept the DBG_CONTROL[4]
        // release write.  The debug window moved to axi_dbg_bus, so that
        // reason is gone, and keeping the front door alive while its own
        // peripherals were being reset underneath it was how an in-flight
        // transaction got stranded.  See the long note on u_pb_s1_cdc above.
        //
        // pb_soc_full_rst_bank, not pb_full_rst_bank: warm_peripheral_reset
        // (68040 RESET instruction) must NOT reset this FSM, because the CPU
        // keeps running through it.
        .clk(pb_clk), .rst(pb_soc_full_rst_bank[3]),  // pb-bank [3]

        // ...but the PERIPHERALS underneath take pb_full_rst_bank, which
        // ALSO carries warm_peripheral_reset.  That divergence is the
        // whole point of the note above -- and it is exactly what strands
        // an in-flight transaction: a one-shot pb_* strobe fired while the
        // device is held in reset is never sampled and never acked, so
        // wr_busy/rd_busy latch forever and S1 goes dead to every master.
        // Telling the bus about the peripherals' reset lets it close its
        // front door and re-arm the strobe instead.  Any bank bit works --
        // all four are the same pb_full_rst_buf net.
        .periph_rst(pb_full_rst_bank[3]),

        .s_awid   (pb_s1_awid   ), .s_awaddr (pb_s1_awaddr ),
        .s_awlen  (pb_s1_awlen  ), .s_awsize (pb_s1_awsize ),
        .s_awburst(pb_s1_awburst), .s_awvalid(pb_s1_awvalid),
        .s_awready(pb_s1_awready),
        .s_wdata  (pb_s1_wdata  ), .s_wstrb  (pb_s1_wstrb  ),
        .s_wlast  (pb_s1_wlast  ), .s_wvalid (pb_s1_wvalid ),
        .s_wready (pb_s1_wready ),
        .s_bid    (pb_s1_bid    ), .s_bresp  (pb_s1_bresp  ),
        .s_bvalid (pb_s1_bvalid ), .s_bready (pb_s1_bready ),
        .s_arid   (pb_s1_arid   ), .s_araddr (pb_s1_araddr ),
        .s_arlen  (pb_s1_arlen  ), .s_arsize (pb_s1_arsize ),
        .s_arburst(pb_s1_arburst), .s_arvalid(pb_s1_arvalid),
        .s_arready(pb_s1_arready),
        .s_rid    (pb_s1_rid    ), .s_rdata  (pb_s1_rdata  ),
        .s_rresp  (pb_s1_rresp  ), .s_rlast  (pb_s1_rlast  ),
        .s_rvalid (pb_s1_rvalid ), .s_rready (pb_s1_rready ),

        .dbg_awaddr (pb_dbg_awaddr ), .dbg_awvalid(pb_dbg_awvalid), .dbg_awready(pb_dbg_awready),
        .dbg_wdata  (pb_dbg_wdata  ), .dbg_wstrb  (pb_dbg_wstrb  ),
        .dbg_wvalid (pb_dbg_wvalid ), .dbg_wready (pb_dbg_wready ),
        .dbg_bresp  (pb_dbg_bresp  ), .dbg_bvalid (pb_dbg_bvalid ), .dbg_bready(pb_dbg_bready),
        .dbg_araddr (pb_dbg_araddr ), .dbg_arvalid(pb_dbg_arvalid), .dbg_arready(pb_dbg_arready),
        .dbg_rdata  (pb_dbg_rdata  ), .dbg_rresp  (pb_dbg_rresp  ),
        .dbg_rvalid (pb_dbg_rvalid ), .dbg_rready (pb_dbg_rready ),

        .dafb_awaddr (pb_dafb_awaddr ), .dafb_awvalid(pb_dafb_awvalid), .dafb_awready(1'b1),
        .dafb_wdata  (pb_dafb_wdata  ), .dafb_wstrb  (pb_dafb_wstrb  ),
        .dafb_wvalid (pb_dafb_wvalid ), .dafb_wready (1'b1),
        .dafb_bresp  (2'b11), .dafb_bvalid (1'b1), .dafb_bready(pb_dafb_bready),
        .dafb_araddr (pb_dafb_araddr ), .dafb_arvalid(pb_dafb_arvalid), .dafb_arready(1'b1),
        .dafb_rdata  (32'd0), .dafb_rresp  (2'b11),
        .dafb_rvalid (1'b1), .dafb_rready (pb_dafb_rready),

        .via1_addr (pb_via1_addr ), .via1_wdata(pb_via1_wdata),
        .via1_wr   (pb_via1_wr   ), .via1_rd   (pb_via1_rd   ),
        .via1_rdata(pb_via1_rdata), .via1_ack  (pb_via1_ack  ),

        .via2_addr (pb_via2_addr ), .via2_wdata(pb_via2_wdata),
        .via2_wr   (pb_via2_wr   ), .via2_rd   (pb_via2_rd   ),
        .via2_rdata(pb_via2_rdata), .via2_ack  (pb_via2_ack  ),

        .enet_addr (pb_enet_addr ), .enet_wdata(pb_enet_wdata),
        .enet_wr   (pb_enet_wr   ), .enet_rd   (pb_enet_rd   ),
        .enet_rdata(pb_enet_rdata), .enet_ack  (pb_enet_ack  ),

        .sonic_addr(pb_sonic_addr), .sonic_wdata(pb_sonic_wdata),
        .sonic_wstrb(pb_sonic_wstrb),
        .sonic_wr  (pb_sonic_wr  ), .sonic_rd  (pb_sonic_rd  ),
        .sonic_rdata(pb_sonic_rdata), .sonic_ack(pb_sonic_ack),

        .orwell_addr(pb_orwell_addr), .orwell_wdata(pb_orwell_wdata),
        .orwell_wr  (pb_orwell_wr  ), .orwell_rd  (pb_orwell_rd  ),
        .orwell_rdata(pb_orwell_rdata), .orwell_ack(pb_orwell_ack),

        .scc_addr  (pb_scc_addr  ), .scc_wdata (pb_scc_wdata ),
        .scc_wr    (pb_scc_wr    ), .scc_rd    (pb_scc_rd    ),
        .scc_rdata (pb_scc_rdata ), .scc_ack   (pb_scc_ack   ),

        .scsi_addr (pb_scsi_addr ), .scsi_wdata(pb_scsi_wdata),
        .scsi_wr   (pb_scsi_wr   ), .scsi_rd   (pb_scsi_rd   ),
        .scsi_rdata(pb_scsi_rdata), .scsi_ack  (pb_scsi_ack  ),
        .scsi_dma_rd_ready(scsi_dma_rd_ready_w),
        .scsi_dma_wr_ready(scsi_dma_wr_ready_w),
        .scsi_dma16_lo_beat(pb_scsi_dma16_lo_beat),

        .asc_addr  (pb_asc_addr  ), .asc_wdata (pb_asc_wdata ),
        .asc_wr    (pb_asc_wr    ), .asc_rd    (pb_asc_rd    ),
        .asc_rdata (pb_asc_rdata ), .asc_ack   (pb_asc_ack   ),

        .iwm_addr  (pb_iwm_addr  ), .iwm_wdata (pb_iwm_wdata ),
        .iwm_wr    (pb_iwm_wr    ), .iwm_rd    (pb_iwm_rd    ),
        .iwm_rdata (pb_iwm_rdata ), .iwm_ack   (pb_iwm_ack   ),

        .adbinj_addr (pb_adbinj_addr ), .adbinj_wdata(pb_adbinj_wdata),
        .adbinj_wr   (pb_adbinj_wr   ), .adbinj_rd   (pb_adbinj_rd   ),
        .adbinj_rdata(pb_adbinj_rdata), .adbinj_ack  (pb_adbinj_ack  )
    );

    wire unused_pb_dafb_decode = &{1'b0, pb_dafb_awaddr, pb_dafb_wdata,
        pb_dafb_wstrb, pb_dafb_awvalid, pb_dafb_wvalid, pb_dafb_bready,
        pb_dafb_araddr, pb_dafb_arvalid, pb_dafb_rready};

    // dbg bridge: keep `core_rst` / `pb_rst` — JTAG host side stays alive
    // across a debug-full-reset by design (the host owns the reset bit;
    // resetting the host-facing AXI bridge would break the channel that's
    // *driving* the reset).  The prov_* peer bridge was removed alongside
    // sd_provision (boot is via JTAG-AXI now).
    // ══════════════════════════════════════════════════════════════════
    // DEBUG WINDOW — served by THE DEBUG BUS when a JTAG host is present
    // ══════════════════════════════════════════════════════════════════
    // Was (and still is, in configurations with no JTAG-AXI bridge):
    //   peripheral_bus dbg_* -> axil_async_bridge (pb_clk -> core_clk)
    //     -> dbg_mux_*/dbg_core_* -> cpu dbg_axi.
    //
    // That routed every host debug access through the SHARED crossbar and
    // through peripheral_bus itself -- the block that stalls when a
    // peripheral stops making forward progress -- so the instrument died
    // exactly when it was needed (p150, 0xE934ECC7: every JTAG-AXI read
    // failed, debug CSR / DRAM / low RAM alike).  Worse, a JTAG `reset` IS
    // a write into this window, so the reset request itself travelled S1
    // and the reset swallowed its own BRESP, stranding the S1 slot
    // (measured 2026-09-12: xbar_s1_slot_busy 0 -> 1 across a `reset`,
    // slv_poisoned 0x00 throughout).
    //
    // With a JTAG host, u_dbg_bus (fpga_top_debug_host.vh) now decodes the
    // 0x5090_0000 window at the master and hands it straight here.  Both
    // ends are already on core_clk, so the CDC the old bridge existed to
    // perform is simply not needed -- this path is shorter as well as
    // unwedgeable, and it is not in the SoC reset domain.
    //
    // The `else` arm below keeps the old bridge verbatim: it covers the
    // PCIe/XDMA host (whose 128-bit master does not pass through
    // axi_dbg_bus) and the no-host lint/sim configurations.  Neither is a
    // shipping bitstream today -- synth/vivado.tcl refuses PCIe and JTAG-AXI
    // together, and ENABLE_JTAG_AXI defaults to 1.
    //
    // ⚠️ KNOWN RESIDUAL in that arm, named here rather than left to be
    // rediscovered.  peripheral_bus now resets on the soc_full_rst event
    // (see u_pbus below) while that bridge's s_rst stays on pb_rst, so the
    // two are no longer the same event.  A soc_full_rst with a debug
    // transaction in flight rewinds peripheral_bus while the bridge keeps
    // the transaction; the B that comes back then sits in the bridge's FIFO
    // with dbg_bready low, and the NEXT debug access would consume it as its
    // own response.  It is not fixed here because every candidate fix just
    // moves the mismatch one hop further down: the CPU's dbg_axi CSR block
    // lives in a POR-only reset domain, so putting this bridge on
    // soc_full_rst would strand a response THERE instead.  The right answer
    // for the PCIe host is the same one the JTAG host got -- put its master
    // on a debug bus too -- and until someone builds a PCIe bitstream the
    // hazard is unreachable: soc_full_rst is only ever pulsed by
    // vio_boot_ctrl[3], which is a tied constant in every sim configuration.
`ifdef JTAG_AXI_ENABLE
`ifdef ETH_DEBUG_ENABLE
    // Debug bus -> the CPU/eth address split (SEL_BIT 19 inside the window).
    assign dbg_mux_awaddr  = jtagdbg_awaddr[19:0];
    assign dbg_mux_awvalid = jtagdbg_awvalid;
    assign jtagdbg_awready = dbg_mux_awready;
    assign dbg_mux_wdata   = jtagdbg_wdata;
    assign dbg_mux_wstrb   = jtagdbg_wstrb;
    assign dbg_mux_wvalid  = jtagdbg_wvalid;
    assign jtagdbg_wready  = dbg_mux_wready;
    assign jtagdbg_bresp   = dbg_mux_bresp;
    assign jtagdbg_bvalid  = dbg_mux_bvalid;
    assign dbg_mux_bready  = jtagdbg_bready;
    assign dbg_mux_araddr  = jtagdbg_araddr[19:0];
    assign dbg_mux_arvalid = jtagdbg_arvalid;
    assign jtagdbg_arready = dbg_mux_arready;
    assign jtagdbg_rdata   = dbg_mux_rdata;
    assign jtagdbg_rresp   = dbg_mux_rresp;
    assign jtagdbg_rvalid  = dbg_mux_rvalid;
    assign dbg_mux_rready  = jtagdbg_rready;
`else
    assign dbg_core_awaddr  = jtagdbg_awaddr[19:0];
    assign dbg_core_awvalid = jtagdbg_awvalid;
    assign jtagdbg_awready  = dbg_core_awready;
    assign dbg_core_wdata   = jtagdbg_wdata;
    assign dbg_core_wstrb   = jtagdbg_wstrb;
    assign dbg_core_wvalid  = jtagdbg_wvalid;
    assign jtagdbg_wready   = dbg_core_wready;
    assign jtagdbg_bresp    = dbg_core_bresp;
    assign jtagdbg_bvalid   = dbg_core_bvalid;
    assign dbg_core_bready  = jtagdbg_bready;
    assign dbg_core_araddr  = jtagdbg_araddr[19:0];
    assign dbg_core_arvalid = jtagdbg_arvalid;
    assign jtagdbg_arready  = dbg_core_arready;
    assign jtagdbg_rdata    = dbg_core_rdata;
    assign jtagdbg_rresp    = dbg_core_rresp;
    assign jtagdbg_rvalid   = dbg_core_rvalid;
    assign dbg_core_rready  = jtagdbg_rready;
`endif
    // AXI-Lite slave: every beat is the last one.
    assign jtagdbg_rlast    = 1'b1;

    // The peripheral-bus 0x5090 window has no consumer any more.  Its only
    // user was the JTAG master, which now takes the debug bus.  Answer it
    // immediately (OKAY, zero) rather than leaving it unanswered: with the
    // ack watchdogs disabled an unanswered access would hang the peripheral
    // bus forever, which is exactly the class of bug this work removes.
    // SLOT_DBG is also peripheral_bus's DEFAULT decode fallback, so this is
    // what an unmapped CPU access to the I/O window terminates on -- OKAY+0
    // is the same open-bus semantic SLOT_FAULT already uses.
    // Reset with peripheral_bus itself (pb_soc_full_rst_bank[3]), not with
    // pb_rst: if the bus FSM rewound and this valid latch did not, a stale
    // BVALID/RVALID would be consumed as the response to the next access.
    reg  pb_dbg_bvalid_q, pb_dbg_rvalid_q;
    always @(posedge pb_clk) begin
        if (pb_soc_full_rst_bank[3]) begin
            pb_dbg_bvalid_q <= 1'b0;
            pb_dbg_rvalid_q <= 1'b0;
        end else begin
            if (pb_dbg_wvalid && !pb_dbg_bvalid_q) pb_dbg_bvalid_q <= 1'b1;
            else if (pb_dbg_bready)                pb_dbg_bvalid_q <= 1'b0;
            if (pb_dbg_arvalid && !pb_dbg_rvalid_q) pb_dbg_rvalid_q <= 1'b1;
            else if (pb_dbg_rready)                 pb_dbg_rvalid_q <= 1'b0;
        end
    end
    assign pb_dbg_awready = 1'b1;
    assign pb_dbg_wready  = 1'b1;
    assign pb_dbg_bresp   = 2'b00;
    assign pb_dbg_bvalid  = pb_dbg_bvalid_q;
    assign pb_dbg_arready = 1'b1;
    assign pb_dbg_rdata   = 32'h0000_0000;
    assign pb_dbg_rresp   = 2'b00;
    assign pb_dbg_rvalid  = pb_dbg_rvalid_q;
`else
    axil_async_bridge #(.DATA_WIDTH(32), .ADDR_WIDTH(20)) u_dbg_pb_to_core (
        .s_clk(pb_clk),
        .s_rst(pb_rst),
        .s_awaddr (pb_dbg_awaddr ),
        .s_awvalid(pb_dbg_awvalid),
        .s_awready(pb_dbg_awready),
        .s_wdata  (pb_dbg_wdata  ),
        .s_wstrb  (pb_dbg_wstrb  ),
        .s_wvalid (pb_dbg_wvalid ),
        .s_wready (pb_dbg_wready ),
        .s_bresp  (pb_dbg_bresp  ),
        .s_bvalid (pb_dbg_bvalid ),
        .s_bready (pb_dbg_bready ),
        .s_araddr (pb_dbg_araddr ),
        .s_arvalid(pb_dbg_arvalid),
        .s_arready(pb_dbg_arready),
        .s_rdata  (pb_dbg_rdata  ),
        .s_rresp  (pb_dbg_rresp  ),
        .s_rvalid (pb_dbg_rvalid ),
        .s_rready (pb_dbg_rready ),
        .m_clk(core_clk),
        .m_rst(core_rst),
`ifdef ETH_DEBUG_ENABLE
        .m_awaddr (dbg_mux_awaddr ), .m_awvalid(dbg_mux_awvalid), .m_awready(dbg_mux_awready),
        .m_wdata  (dbg_mux_wdata  ), .m_wstrb(dbg_mux_wstrb),
        .m_wvalid (dbg_mux_wvalid ), .m_wready(dbg_mux_wready),
        .m_bresp  (dbg_mux_bresp  ), .m_bvalid(dbg_mux_bvalid), .m_bready(dbg_mux_bready),
        .m_araddr (dbg_mux_araddr ), .m_arvalid(dbg_mux_arvalid), .m_arready(dbg_mux_arready),
        .m_rdata  (dbg_mux_rdata  ), .m_rresp(dbg_mux_rresp),
        .m_rvalid (dbg_mux_rvalid ), .m_rready(dbg_mux_rready)
`else
        .m_awaddr (dbg_core_awaddr ), .m_awvalid(dbg_core_awvalid), .m_awready(dbg_core_awready),
        .m_wdata  (dbg_core_wdata  ), .m_wstrb(dbg_core_wstrb),
        .m_wvalid (dbg_core_wvalid ), .m_wready(dbg_core_wready),
        .m_bresp  (dbg_core_bresp  ), .m_bvalid(dbg_core_bvalid), .m_bready(dbg_core_bready),
        .m_araddr (dbg_core_araddr ), .m_arvalid(dbg_core_arvalid), .m_arready(dbg_core_arready),
        .m_rdata  (dbg_core_rdata  ), .m_rresp(dbg_core_rresp),
        .m_rvalid (dbg_core_rvalid ), .m_rready(dbg_core_rready)
`endif
    );
`endif

`ifdef ETH_DEBUG_ENABLE
    axil_split2 #(.ADDR_W(20), .SEL_BIT(19)) u_dbg_cpu_eth_split (
        .clk(core_clk), .rst(core_rst),
        .u_awaddr(dbg_mux_awaddr),.u_awvalid(dbg_mux_awvalid),.u_awready(dbg_mux_awready),
        .u_wdata(dbg_mux_wdata),.u_wstrb(dbg_mux_wstrb),.u_wvalid(dbg_mux_wvalid),.u_wready(dbg_mux_wready),
        .u_bresp(dbg_mux_bresp),.u_bvalid(dbg_mux_bvalid),.u_bready(dbg_mux_bready),
        .u_araddr(dbg_mux_araddr),.u_arvalid(dbg_mux_arvalid),.u_arready(dbg_mux_arready),
        .u_rdata(dbg_mux_rdata),.u_rresp(dbg_mux_rresp),.u_rvalid(dbg_mux_rvalid),.u_rready(dbg_mux_rready),
        .d0_awaddr(dbg_core_awaddr),.d0_awvalid(dbg_core_awvalid),.d0_awready(dbg_core_awready),
        .d0_wdata(dbg_core_wdata),.d0_wstrb(dbg_core_wstrb),.d0_wvalid(dbg_core_wvalid),.d0_wready(dbg_core_wready),
        .d0_bresp(dbg_core_bresp),.d0_bvalid(dbg_core_bvalid),.d0_bready(dbg_core_bready),
        .d0_araddr(dbg_core_araddr),.d0_arvalid(dbg_core_arvalid),.d0_arready(dbg_core_arready),
        .d0_rdata(dbg_core_rdata),.d0_rresp(dbg_core_rresp),.d0_rvalid(dbg_core_rvalid),.d0_rready(dbg_core_rready),
        .d1_awaddr(eth_dbg_awaddr),.d1_awvalid(eth_dbg_awvalid),.d1_awready(eth_dbg_awready),
        .d1_wdata(eth_dbg_wdata),.d1_wstrb(eth_dbg_wstrb),.d1_wvalid(eth_dbg_wvalid),.d1_wready(eth_dbg_wready),
        .d1_bresp(eth_dbg_bresp),.d1_bvalid(eth_dbg_bvalid),.d1_bready(eth_dbg_bready),
        .d1_araddr(eth_dbg_araddr),.d1_arvalid(eth_dbg_arvalid),.d1_arready(eth_dbg_arready),
        .d1_rdata(eth_dbg_rdata),.d1_rresp(eth_dbg_rresp),.d1_rvalid(eth_dbg_rvalid),.d1_rready(eth_dbg_rready)
    );
`endif

    uart_byte_bridge #(
        .CLK_HZ(PB_CLK_HZ),
        .BAUD  (115200)
    ) u_scc_board_uart (
        .clk     (pb_clk),
        // pb_full_rst — UART byte FIFO state cycles with the SCC reset
        // so a debug-full-reset doesn't echo stale board input back into
        // the freshly-rewound SCC.  pb-bank [2] (SCC region).
        .rst     (pb_full_rst_bank[2]),
        .uart_rx (uart_rtl_0_rxd),
        .uart_tx (uart_rtl_0_txd),
        .rx_valid(scc_uart_rx_valid),
        .rx_data (scc_uart_rx_data),
        .tx_valid(scc_uart_tx_valid),
        .tx_data (scc_uart_tx_data),
        .tx_busy (scc_uart_tx_busy)
    );

    video u_dafb (
        .clk          (core_clk),
        // soc_full_rst — DAFB CLUT/base/stride registers and VBL state
        // rewind on a debug-full-reset so the next scan-out frame uses
        // post-reset defaults again.  Bank bit [2] (DAFB shim region).
        .rst          (soc_full_rst_bank[2]),
        .s_axi_awaddr (dafb_awaddr),
        .s_axi_awvalid(dafb_awvalid),
        .s_axi_awready(dafb_awready),
        .s_axi_wdata  (dafb_wdata),
        .s_axi_wstrb  (dafb_wstrb),
        .s_axi_wvalid (dafb_wvalid),
        .s_axi_wready (dafb_wready),
        .s_axi_bresp  (dafb_bresp),
        .s_axi_bvalid (dafb_bvalid),
        .s_axi_bready (dafb_bready),
        .s_axi_araddr (dafb_araddr),
        .s_axi_arvalid(dafb_arvalid),
        .s_axi_arready(dafb_arready),
        .s_axi_rdata  (dafb_rdata),
        .s_axi_rresp  (dafb_rresp),
        .s_axi_rvalid (dafb_rvalid),
        .s_axi_rready (dafb_rready),
        .fb_base_px   (dafb_fb_base_px),
        .fb_stride_px (dafb_fb_stride_px),
        .fb_bpp_reg   (dafb_fb_bpp_reg),
        .bpp_shift    (dafb_bpp_shift),
        .fb_bytes_per_px (dafb_fb_bytes_per_px),
        .depth_supported (dafb_depth_supported),
        .hres         (dafb_hres),
        .vres         (dafb_vres),
        .clut_we      (dafb_clut_we),
        .clut_waddr   (dafb_clut_waddr),
        .clut_wdata   (dafb_clut_wdata),
        .irq          (dafb_irq_w),
        // ── TurboSCSI shim glue ─────────────────────────────────────
        // m_scsi_ctrl[0] (DAFB +0x24, low 9 bits) and m_drq[0] from
        // the NCR 53C9x.  scsi_drq_w lives on pb_clk; sync into the
        // core_clk DAFB domain.  scsi0_ctrl_out is core_clk; sync
        // back into pb_clk where scsi.v consumes it.  Static config
        // bits (DRQ-check enables) so a 2-FF synchroniser is enough.
        .scsi0_ctrl_out (dafb_scsi0_ctrl_core),
        .scsi0_drq_in   (dafb_scsi0_drq_sync),
        // core_clk-synchronised per-frame level (see the synchroniser
        // declared below, near via1_ca1_in) — video.v edge-detects it.
        .frame_tick     (dafb_vbl_level_core_sync),
        // Apple Display Sense code, runtime-settable over JTAG.  Comes from
        // the CPU debug-CSR (OFF_MON_SENSE 0x0005C) across the socket as
        // cpu_mon_sense (declared + documented in fpga_top_cpu.vh).  All 7
        // bits: bit 6 is the extended-monitor flag video.v's
        // sense_response() gates the convolution on.  Mac OS samples sense
        // at DAFB init, so a new code takes effect on the next CPU reset —
        // which the value survives, since it lives in the CPU's POR-only
        // debug reset domain.
        .monitor_sense  (cpu_mon_sense)
    );

    // pb_clk → core_clk synchroniser for NCR DRQ.
    wire        dafb_scsi0_drq_sync;
    (* ASYNC_REG = "TRUE" *) reg dafb_scsi0_drq_meta;
    (* ASYNC_REG = "TRUE" *) reg dafb_scsi0_drq_sync_q;
    always @(posedge core_clk) begin
        if (soc_full_rst_bank[2]) begin
            dafb_scsi0_drq_meta   <= 1'b0;
            dafb_scsi0_drq_sync_q <= 1'b0;
        end else begin
            dafb_scsi0_drq_meta   <= scsi_drq_w;
            dafb_scsi0_drq_sync_q <= dafb_scsi0_drq_meta;
        end
    end
    assign dafb_scsi0_drq_sync = dafb_scsi0_drq_sync_q;

    // core_clk → pb_clk synchroniser for the m_scsi_ctrl[0] config
    // word.  Treat each bit independently — the host writes this
    // register once during driver init (before any DMA cycle), so
    // any transient skew across the 9 bits is harmless.
    wire [8:0]  dafb_scsi0_ctrl_core;
    wire [8:0]  dafb_scsi0_ctrl_pb;
    (* ASYNC_REG = "TRUE" *) reg [8:0] dafb_scsi0_ctrl_meta;
    (* ASYNC_REG = "TRUE" *) reg [8:0] dafb_scsi0_ctrl_sync_q;
    always @(posedge pb_clk) begin
        if (pb_full_rst_bank[2]) begin
            dafb_scsi0_ctrl_meta   <= 9'd0;
            dafb_scsi0_ctrl_sync_q <= 9'd0;
        end else begin
            dafb_scsi0_ctrl_meta   <= dafb_scsi0_ctrl_core;
            dafb_scsi0_ctrl_sync_q <= dafb_scsi0_ctrl_meta;
        end
    end
    assign dafb_scsi0_ctrl_pb = dafb_scsi0_ctrl_sync_q;

    // ═══════════════════════════════════════════════════════════════════
    wire sonic_irq_pb;
    wire via1_irq_pb;
    wire via2_irq_pb;
    wire scsi_irq_pb;
    wire scc_irq_pb;
    wire asc_irq_pb;
    (* ASYNC_REG = "TRUE" *) reg via1_irq_meta, via1_irq_sync;
    (* ASYNC_REG = "TRUE" *) reg via2_irq_meta, via2_irq_sync;
    (* ASYNC_REG = "TRUE" *) reg scsi_irq_meta, scsi_irq_sync;
    (* ASYNC_REG = "TRUE" *) reg scc_irq_meta, scc_irq_sync;
    (* ASYNC_REG = "TRUE" *) reg asc_irq_meta, asc_irq_sync;
    always @(posedge core_clk) begin
        // soc_full_rst — IRQ sync FFs cycle alongside the peripherals
        // they observe so a debug-full-reset doesn't carry a stale
        // pre-reset IRQ edge into the post-reset core.  Bank bit [2]
        // (DAFB / IRQ-resync region — same placement zone as u_dafb).
        if (soc_full_rst_bank[2]) begin
            via1_irq_meta <= 1'b0;
            via1_irq_sync <= 1'b0;
            via2_irq_meta <= 1'b0;
            via2_irq_sync <= 1'b0;
            scsi_irq_meta <= 1'b0;
            scsi_irq_sync <= 1'b0;
            scc_irq_meta  <= 1'b0;
            scc_irq_sync  <= 1'b0;
            asc_irq_meta  <= 1'b0;
            asc_irq_sync  <= 1'b0;
        end else begin
            via1_irq_meta <= via1_irq_pb;
            via1_irq_sync <= via1_irq_meta;
            via2_irq_meta <= via2_irq_pb;
            via2_irq_sync <= via2_irq_meta;
            scsi_irq_meta <= scsi_irq_pb;
            scsi_irq_sync <= scsi_irq_meta;
            scc_irq_meta  <= scc_irq_pb;
            scc_irq_sync  <= scc_irq_meta;
            asc_irq_meta  <= asc_irq_pb;
            asc_irq_sync  <= asc_irq_meta;
        end
    end
    assign via1_irq_w = via1_irq_sync;
    assign via2_irq_w = via2_irq_sync;
    assign scsi_irq_w = scsi_irq_sync;
    assign scc_irq_w  = scc_irq_sync;
    assign asc_irq_w  = asc_irq_sync;

    q700_eth_sonic #(
`ifdef ETH_ENABLE
        .PACKET_ENGINE(ETH_ICMP_RESPONDER == 0)
`else
        .PACKET_ENGINE(0)
`endif
    ) u_q700_eth_sonic (        .clk       (pb_clk),
        // pb_full_rst — Ethernet ID/SONIC reg state cycles on debug-
        // full-reset.  pb-bank [3] (SONIC + bridge region).
        .rst       (pb_full_rst_bank[3]),
        .enet_cs   (pb_enet_wr | pb_enet_rd),
        .enet_rd   (pb_enet_rd),
        .enet_wr   (pb_enet_wr),
        .enet_addr (pb_enet_addr),
        .enet_wdata(pb_enet_wdata),
        .enet_rdata(pb_enet_rdata),
        .enet_ack  (pb_enet_ack),

        .sonic_cs   (pb_sonic_wr | pb_sonic_rd),
        .sonic_rd   (pb_sonic_rd),
        .sonic_wr   (pb_sonic_wr),
        .sonic_addr (pb_sonic_addr),
        .sonic_wdata(pb_sonic_wdata),
        .sonic_wstrb(pb_sonic_wstrb),
        .sonic_rdata(pb_sonic_rdata),
        .sonic_ack  (pb_sonic_ack),
        .sonic_irq  (sonic_irq_pb),
        .dbg_cr(sonic_dbg_cr_pb), .dbg_dcr(sonic_dbg_dcr_pb),
        .dbg_imr(sonic_dbg_imr_pb),
        .dbg_isr(sonic_dbg_isr_pb),
        .tx_cmd_valid(sonic_tx_cmd_valid_pb), .tx_cmd_ready(sonic_tx_cmd_ready_pb),
        .tx_cmd_dcr(sonic_tx_cmd_dcr_pb), .tx_cmd_utda(sonic_tx_cmd_utda_pb),
        .tx_cmd_ctda(sonic_tx_cmd_ctda_pb),
        .tx_done_valid(sonic_tx_done_valid_pb), .tx_done_ready(sonic_tx_done_ready_pb),
        .tx_done_error(sonic_tx_done_error_pb), .tx_done_pint(sonic_tx_done_pint_pb),
        .tx_done_ctda(sonic_tx_done_ctda_pb),
        .tx_done_tcr(sonic_tx_done_tcr_pb), .tx_done_tps(sonic_tx_done_tps_pb),
        .tx_done_tfc(sonic_tx_done_tfc_pb),
        .tx_halt(sonic_tx_halt_pb),
        .rx_enabled(sonic_rx_enabled_pb), .rx_cfg_valid(sonic_rx_cfg_valid_pb),
        .rx_cfg_ready(sonic_rx_cfg_ready_pb), .rx_cfg_op(sonic_rx_cfg_op_pb),
        .rx_cfg_dcr(sonic_rx_cfg_dcr_pb),
        .rx_cfg_rcr(sonic_rx_cfg_rcr_pb), .rx_cfg_urda(sonic_rx_cfg_urda_pb),
        .rx_cfg_crda(sonic_rx_cfg_crda_pb), .rx_cfg_urra(sonic_rx_cfg_urra_pb),
        .rx_cfg_rsa(sonic_rx_cfg_rsa_pb), .rx_cfg_rea(sonic_rx_cfg_rea_pb),
        .rx_cfg_rrp(sonic_rx_cfg_rrp_pb), .rx_cfg_rwp(sonic_rx_cfg_rwp_pb),
        .rx_cfg_eobc(sonic_rx_cfg_eobc_pb), .rx_cfg_rsc(sonic_rx_cfg_rsc_pb),
        .rx_cfg_llfa(sonic_rx_cfg_llfa_pb), .rx_cfg_cdp(sonic_rx_cfg_cdp_pb),
        .rx_cfg_cdc(sonic_rx_cfg_cdc_pb),
        .rx_done_valid(sonic_rx_done_valid_pb), .rx_done_ready(sonic_rx_done_ready_pb),
        .rx_done_error(sonic_rx_done_error_pb), .rx_done_rcr(sonic_rx_done_rcr_pb),
        .rx_done_crda(sonic_rx_done_crda_pb), .rx_done_crba0(sonic_rx_done_crba0_pb),
        .rx_done_crba1(sonic_rx_done_crba1_pb), .rx_done_rbwc0(sonic_rx_done_rbwc0_pb),
        .rx_done_rbwc1(sonic_rx_done_rbwc1_pb), .rx_done_rrp(sonic_rx_done_rrp_pb),
        .rx_done_rsc(sonic_rx_done_rsc_pb), .rx_done_llfa(sonic_rx_done_llfa_pb),
        .rx_done_trba0(sonic_rx_done_trba0_pb), .rx_done_trba1(sonic_rx_done_trba1_pb),
        .rx_done_tbwc0(sonic_rx_done_tbwc0_pb), .rx_done_tbwc1(sonic_rx_done_tbwc1_pb),
        .rx_done_isr_set(sonic_rx_done_isr_set_pb),
        .rx_done_cdp(sonic_rx_done_cdp_pb), .rx_done_cdc(sonic_rx_done_cdc_pb),
        .rx_done_ce(sonic_rx_done_ce_pb)
    );

    orwell_stub u_orwell_stub (
        .cs    (pb_orwell_wr | pb_orwell_rd),
        .rd    (pb_orwell_rd),
        .wr    (pb_orwell_wr),
        .addr  (pb_orwell_addr),
        .wdata (pb_orwell_wdata),
        .rdata (pb_orwell_rdata),
        .ack   (pb_orwell_ack)
    );

    // Mac peripherals — VIA1 is a real 6522; VIA2/SCC/ASC/SCSI have RTL
    // behavior; Ethernet ID/SONIC is a real reset/config register block;
    // Orwell / SWIM-IWM remain probe-safe stubs.
    // Address offsets within the 0x5000_0000 I/O window:
    //   0x000_0000  SCC     (4 KB)
    //   0x000_1000  SCSI    (MAME Q700 TurboSCSI regs + DMA shim)
    //   0x000_2000  ASC     (16 KB)
    //   0x080_0000  (REMOVED — was sd_provision; now AXI DECERR via SLOT_FAULT)
    //   0x0F0_0000  VIA1    (4 KB)  — ROM overlay + ADB + timers
    //   0x0F0_2000  VIA2    (4 KB)  — NuBus slot IRQs (stub)
    //   0x0F0_4000  ADB     (alias to VIA1 SR — routed through VIA1)
    // ═══════════════════════════════════════════════════════════════════
    /* verilator lint_off UNUSEDSIGNAL */
    wire [7:0]  via1_pa_out, via1_pa_mask, via1_pb_out, via1_pb_mask;
    wire        via1_adb_rx_ready;
    wire [7:0]  via1_adb_tx_byte;
    wire        via1_adb_tx_valid;
    /* verilator lint_on UNUSEDSIGNAL */
    wire        via1_overlay_pb;
    (* ASYNC_REG = "TRUE" *) reg via1_overlay_meta;
    (* ASYNC_REG = "TRUE" *) reg via1_overlay_sync;
    always @(posedge core_clk) begin
        if (core_rst) begin
            via1_overlay_meta <= 1'b1;
            via1_overlay_sync <= 1'b1;
        end else begin
            via1_overlay_meta <= via1_overlay_pb;
            via1_overlay_sync <= via1_overlay_meta;
        end
    end
    assign via1_overlay_bit = via1_overlay_sync;

    // RTC side-channel (VIA1 PB0..PB2)
    wire        via1_rtc_enb;
    wire        via1_rtc_clk;
    wire        via1_rtc_data_o;
    wire        via1_rtc_data_oe;
    wire        via1_rtc_data_i;
    wire        via1_rtc_cko;
    // SWIM (floppy) interrupt -> VIA2 CA2 (MAME macquadra700.cpp:875).
    // Declared here rather than at u_iwm because the VIA2 instantiation
    // above consumes it and Verilog wants the net declared first.
    wire        iwm_irq_w;

    // pb_in[0] routes rtc_data back into the VIA read path when the CPU
    // tristates PB0 (DDRB[0]=0).  PB3 is the active-low ADB modem IRQ
    // pin (per src/mame/apple/macquadra700.cpp::via_in_b: when
    // m_adb_irq_pending is 0, val |= 0x08; i.e. bit 3 reads HIGH while
    // the modem is idle and LOW while an ADB transaction is pending).
    // via1.v's ORB[3] read mux now follows the standard 6522 DDR mux
    // (pb_in[3] when DDRB[3]=0, orb[3] when DDRB[3]=1) so the CPU
    // observes ~adb_irq_pending here directly.  The internal overlay
    // latch — used by glue/xbar to alias low memory to ROM at reset —
    // stays decoupled on the via1_overlay_pb output.
    wire        adb_irq_pending;
    wire        adb_cb1;
    wire        adb_cb2;
    wire        via1_cb2_out;
    wire        via1_cb2_oe;
    wire        pic_cb1_out;
    wire        pic_cb2_out;
    wire        pic_cb2_oe;
    wire        pic_adb_out;
    wire        adb_bus_in;
    wire        adb_rtcc_in;
    wire [7:0]  via1_pb_in = {4'b0, ~adb_irq_pending, 2'b00, via1_rtc_data_i};

    assign adb_cb1 = pic_cb1_out;
    assign adb_cb2 = via1_cb2_oe ? via1_cb2_out :
                     (pic_cb2_oe ? pic_cb2_out  : 1'b1);

    // adb_phy now drives the device-bus fan-out to adb_keyboard.v /
    // adb_mouse.v directly (real bit-level ADB responses — see
    // rtl/mac/adb_phy.v header).  These wires are declared ahead of
    // u_adb_phy's instantiation and forward-reference the kbd/mouse
    // response signals declared further down (Verilog wires are not
    // order-sensitive at elaboration).
    wire        adb_dev_cmd_valid;
    wire [3:0]  adb_dev_cmd_addr;
    wire [2:0]  adb_dev_cmd_op;
    wire        adb_dev_listen_valid;
    wire [7:0]  adb_dev_listen_b0;
    wire [7:0]  adb_dev_listen_b1;

    wire        adb_kbd_resp_valid;
    wire        adb_kbd_resp_empty;
    wire [7:0]  adb_kbd_resp_b0;
    wire [7:0]  adb_kbd_resp_b1;
    wire        adb_kbd_srq;

    wire        adb_ms_resp_valid;
    wire        adb_ms_resp_empty;
    wire [7:0]  adb_ms_resp_b0;
    wire [7:0]  adb_ms_resp_b1;
    wire        adb_ms_srq;

    adb_phy #(
        .CLK_MHZ(32)
    ) u_adb_phy (
        .clk        (pb_clk),
        .rst        (pb_full_rst_bank[0]),
        .phi2_tick  (phi2_tick),
        .pic_adb_out(pic_adb_out),
        .adb_in     (adb_bus_in),
        .rtcc_in    (adb_rtcc_in),
        .dev_cmd_valid   (adb_dev_cmd_valid),
        .dev_cmd_addr    (adb_dev_cmd_addr),
        .dev_cmd_op      (adb_dev_cmd_op),
        .dev_listen_valid(adb_dev_listen_valid),
        .dev_listen_b0   (adb_dev_listen_b0),
        .dev_listen_b1   (adb_dev_listen_b1),
        .dev_resp_valid  ({adb_ms_resp_valid, adb_kbd_resp_valid}),
        .dev_resp_empty  ({adb_ms_resp_empty, adb_kbd_resp_empty}),
        .dev_resp_b0     ({adb_ms_resp_b0,    adb_kbd_resp_b0}),
        .dev_resp_b1     ({adb_ms_resp_b1,    adb_kbd_resp_b1}),
        .dev_srq         ({adb_ms_srq,        adb_kbd_srq})
    );

    // VIA2 board-level pins.  Port A is the NuBus slot-interrupt register.
    // MAME macquadra700 `via2_in_a()` returns `0x80 | m_nubus_irq_state`,
    // and `nubus_slot_interrupt()` does `slot -= 9` then indexes
    // masks[8] = {0x1,0x2,...,0x80}, clearing the bit while asserted.  So:
    //   PA7      = hard-wired 1 (the `0x80 |`)
    //   PA6      = slot $F  = built-in DAFB video IRQ  (dafb_irq_w ->
    //                         nubus_slot_interrupt(0xf) in MAME)
    //   PA5..PA1 = slots $E..$A = no devices here -> idle high (5'h1F)
    //   PA0      = slot $9  = SONIC (MAME: sonic out_int_cb -> via2
    //                         write_pa0, .invert() = active low)
    //
    // SCSI does NOT appear on Port A at all.  MAME wires the 53C96 as:
    //   irq_handler_cb -> via2 write_cb2, .invert()   [see .cb2_in below]
    //   drq_handler_cb -> dafb turboscsi_drq_w<0>     [the DAFB, not a VIA]
    //
    // This line used to read `{~scsi_irq_pb, ~scsi_drq_w, 5'h1F, ...}`,
    // which put SCSI DRQ into the bit the ROM reads as the *video slot*
    // interrupt.  HW-observed 2026-07-25 on build 0xECCD9A00: PA = 0xBF
    // (PA6 low) with DRQ stuck asserted, so the ROM's slot dispatcher at
    // 0x40806ECA spun forever on its `orb PA / notl / bne` test over
    // PA[6:0] — it kept dispatching the slot-$F handler, which cannot ack
    // a SCSI DRQ.  That starved every other interrupt: exc_count froze,
    // ADB stopped being polled and MTemp never left its initial (15,15).
    // Q700 VIA2 Port A reads 0xFF when no IRQs are pending (MAME dumps at
    // +0x2200 ORA-handshake and +0x3E00 ORA-no-handshake are both 0xFF).
    // Assigned below, once dafb_irq_pb_sync has been declared.
    wire [7:0]  via2_pa_in;
    // Port B is a CONSTANT ON PURPOSE.  Do NOT "fix" this the way Port A
    // above was fixed -- the two are not the same class of signal.
    //
    // MAME ground truth, src/mame/apple/macquadra700.cpp:
    //     u8 quadrax00_state::via2_in_b()
    //     {
    //         return 0xcf;        // indicate no NuBus transaction error
    //     }
    // (macii.cpp:563 is identical.)  Unlike via2_in_a(), which MAME derives
    // from live m_nubus_irq_state and which therefore genuinely had to become
    // dynamic here, via2_in_b() is derived from nothing on any Mac II- or
    // Quadra-class machine.  A hardwired 0xCF *is* the faithful model, and
    // MAME boots this exact ROM + disk to the Finder with it.
    //
    // 2026-08-26: a hardware boot hung forever polling PB6 at RAM 0x0000a8c0
    // and this constant was blamed for it.  It is not the cause.  That code
    // is a relocated copy of ROM 0x408fbd98, which lives inside
    // `.Display_Video_Apple_RBV` -- the Macintosh IIci/IIsi RAM-Based Video
    // driver, whose byte-spaced pseudo-VIA register at base+2 aliases onto a
    // real Q700 VIA2's ORB.  A healthy Q700 loads only
    // `.Display_Video_Apple_DAFB` and never executes that routine (verified
    // over 120 emulated seconds with a control breakpoint).  The real bug is
    // the video-driver mis-selection, one layer up.
    //
    // Making PB6 toggle would be actively HARMFUL: it would let that driver's
    // handshake pass spuriously and run on into writes at base+0x0A/+0x0B/
    // +0x10, all of which alias onto VIA2 ORB -- whose real bits are the DFAC
    // audio serial lines (PB0 latch, PB3 data, PB4 clock) and PB7 -> VIA1 CA1.
    // Silent audio-codec and 60.15 Hz-chain corruption instead of a visible
    // hang.  See docs/BUG_via2_pb6_constant_boot_hang.md for the full evidence.
    //
    // Bit meaning per the Q700/MAME reset straps: no slot IRQs, TM0A/TM1A high.
    wire [7:0]  via2_pb_in = 8'hCF;
    // via2_pb_out is now a real consumer: VIA1's CA1 source (below) reads
    // via2_pb_out[7], the board-level VIA2-PB7 -> VIA1-CA1 wire.  Pulled
    // out of the UNUSEDSIGNAL block below since it's no longer dangling.
    wire [7:0]  via2_pb_out;
    /* verilator lint_off UNUSEDSIGNAL */
    wire [7:0]  via2_pa_out, via2_pa_mask, via2_pb_mask;
    wire        via2_ca2_out, via2_cb1_out, via2_cb2_out;
    /* verilator lint_on UNUSEDSIGNAL */

    // ── DAFB slot-IRQ (virtual NuBus slot $F, built-in video) → VIA2 CA1
    //    → Mac OS slot handler → VIA2 PB7 → VIA1 CA1 (task: restore the
    //    faithful MAME/Q700 chain, no shortcuts) ─────────────────────────
    //
    // Previously VIA2.ca1_in was tied 1'b1 (dead: edge-triggered input,
    // so IFR.CA1 could never latch) and VIA1.ca1_in was driven directly
    // from the HDMI-VTG-derived dafb_vbl_level below, entirely bypassing
    // VIA2.  Live JTAG reads showed Mac OS had armed VIA2 IER.CA1
    // (IER=0x92, IFR=0x48) waiting on an interrupt that structurally
    // could never arrive.
    //
    // MAME's macquadra700 wiring is dafb_irq -> nubus_slot_interrupt(0xf)
    // -> VIA2.CA1; the ROM's slot-interrupt handler then re-fires the
    // classic VBL path by toggling VIA2 ORB bit 7, which real Quadra-
    // class hardware wires straight to VIA1's CA1 pin (see via1.v /
    // via2.v header commentary) — this is how a machine whose only VBL
    // source is the built-in/NuBus video slot (no dedicated VIA1-direct
    // VBL pin) still feeds the historical VIA1-CA1-anchored VBL task
    // queue used on every earlier Mac model.
    //
    // dafb_irq_w (video.v's `irq` output — REG_IRQ_STATUS-gated,
    // active-HIGH level: 1 = vblank pending && IRQ_ENABLE) is declared in
    // fpga_top_cpu.vh and driven by u_dafb above (`.irq(dafb_irq_w)`) but
    // was never actually consumed anywhere until now.  video.v runs on
    // core_clk; VIA2 runs on pb_clk, so this needs its own 2-FF
    // ASYNC_REG synchroniser (same idiom as dafb_scsi0_drq_sync above).
    wire        dafb_irq_pb_sync;
    (* ASYNC_REG = "TRUE" *) reg dafb_irq_pb_meta;
    (* ASYNC_REG = "TRUE" *) reg dafb_irq_pb_sync_q;
    always @(posedge pb_clk) begin
        if (pb_full_rst_bank[1]) begin
            dafb_irq_pb_meta   <= 1'b0;
            dafb_irq_pb_sync_q <= 1'b0;
        end else begin
            dafb_irq_pb_meta   <= dafb_irq_w;
            dafb_irq_pb_sync_q <= dafb_irq_pb_meta;
        end
    end
    assign dafb_irq_pb_sync = dafb_irq_pb_sync_q;

    // Port A slot-IRQ aggregate (see the Port A comment block above for the
    // MAME bit mapping).  PA6 is the DAFB slot-$F line and is an active-low
    // LEVEL, held asserted until software W1C-acks the DAFB interrupt source
    // (VBL at +0x114/+0x20, cursor scanline at +0x10C) — exactly the sense
    // of MAME's nubus_slot_interrupt(0xf, dafb_irq).
    // Port A deliberately carries the LEVEL, not an edge: the ROM's slot
    // dispatcher needs it to (a) identify which slot interrupted and
    // (b) observe the line go back high once its handler has acked, which
    // is what lets the dispatcher's `notl/bne` loop terminate.  The
    // recurring negative EDGE that latches VIA2 IFR.CA1 is seen by via2.v's
    // PA[6:0] edge detector below.  The explicit ca1_in pin is still driven
    // from the same DAFB level so direct-CA1 tests and older instrumentation
    // observe the same slot event.
    assign via2_pa_in = {1'b1, ~dafb_irq_pb_sync, 5'h1F, ~sonic_irq_pb};

    // Polarity: via2.v hard-codes CA1 as negative-edge for the Q700 slot
    // aggregate — PCR resets to 8'h00 (PCR[0]=0 = negative-edge), and
    // via2.v's own header states plainly "the Q700 VIA2 the ROM programs
    // CA1 for negative-edge slot-IRQ", matching real NuBus slot lines
    // (idle high, assert low; see via2.v's pa_in comment block and
    // ca1_edge_ext's `!ca1_in && ca1_prev` negative-edge test).
    // dafb_irq_w is active-HIGH (1 = pending), so invert it here to
    // present the expected idle-high / assert-low sense on ca1_in.
    //
    // MAME ground truth (macqd700 + disk, 12 emulated s): the ROM's VIA2
    // dispatcher at 0x40809BE0 computes `IFR & IER & 0x7F` and dispatched
    // **424/424 times with d0 = 2, i.e. bit 1 = CA1**, at ~35/sec.  So CA1
    // is definitively the right pin.  Do not gate this with `dafb_vbl_level`:
    // DAFB's cursor-scanline interrupt is not aligned to the HDMI frame pulse.
    // Live failure signature was DAFB +0x108 bit2 set, PA6 asserted low, and
    // VIA2 IFR.CA1 clear, so the cursor task never propagated MTemp to
    // RawMouse/Mouse even though ADB motion reports were consumed.
    wire        via2_ca1_in = ~dafb_irq_pb_sync;

    // ── DAFB VBL → u_dafb frame_tick chain (task #T8) ──────────────────
    // Independent of the CA1/interrupt path above.  The HDMI scan-out
    // pipeline (pclk domain, 148.5 MHz) emits a single-cycle frame-start
    // tick (vbl_pulse_pclk).  pulse_cdc converts each tick into a
    // 1-cycle pb_clk-domain pulse; we then extend it into a ~32
    // pb_clk-cycle level (`dafb_vbl_level`) purely so it can be safely
    // resampled into core_clk below to drive video.v's `frame_tick`
    // input.  `dafb_vbl_level` no longer drives any CA1 input directly
    // (see the real slot-IRQ chain above) — kept only for frame_tick.
    wire dafb_vbl_pulse_pb;
    pulse_cdc u_dafb_vbl_cdc (
        .src_clk  (pclk),
        .src_rst  (1'b0),  // pclk-domain reset is internal to video_top.
        .src_pulse(video_vbl_pulse_pclk),
        .dst_clk  (pb_clk),
        .dst_rst  (pb_full_rst_bank[0]),
        .dst_pulse(dafb_vbl_pulse_pb)
    );

    // Pulse extender: hold the level high for VBL_LEVEL_TICKS pb_clk
    // cycles after each pulse.  At pb_clk = 50 MHz, 32 cycles = 640 ns —
    // plenty wide for the core_clk resampler (dafb_vbl_level_core_sync
    // below) to reliably catch both the rising and falling edge and
    // produce one clean per-frame tick for video.v's frame_tick input.
    localparam integer VBL_LEVEL_TICKS = 32;
    reg [5:0] dafb_vbl_extend;
    reg       dafb_vbl_level;
    always @(posedge pb_clk) begin
        if (pb_full_rst_bank[0]) begin
            dafb_vbl_extend <= 6'd0;
            dafb_vbl_level  <= 1'b0;
        end else if (dafb_vbl_pulse_pb) begin
            dafb_vbl_extend <= VBL_LEVEL_TICKS[5:0];
            dafb_vbl_level  <= 1'b1;
        end else if (dafb_vbl_extend != 6'd0) begin
            dafb_vbl_extend <= dafb_vbl_extend - 6'd1;
            if (dafb_vbl_extend == 6'd1)
                dafb_vbl_level <= 1'b0;
        end
    end

    // VIA1 CA1 source: VIA2 Port B bit 7 — what the Mac OS slot-IRQ
    // handler actually toggles in response to VIA2.CA1 firing (see the
    // real slot-IRQ chain above).  Real Quadra-class hardware physically
    // shorts VIA2 PB7 straight to VIA1's CA1 pin (no inversion, no DDR
    // gating at the board level — matches via1.v/via2.v header
    // commentary describing this as a direct board-level wire).  VIA1's
    // PCR[0] defaults low-to-high (Q700 ROM default, `pcr_ca1_low_to_high`
    // in via1.v), so the OS handler's ORB[7] 0→1 pulse re-fires VIA1 CA1
    // once per VBL, same as the historical compact-Mac VIA1-CA1-direct
    // wiring, just relayed through VIA2 now instead of skipped.
    //
    // 2026-05-17 history: an earlier attempt (commit 72e4b446) also drove
    // VIA1 CA1 from VIA2 PB7, but at that time VIA2.CA1 was STILL tied
    // off (this same bug) — so PB7 was driven by a free-running VIA-
    // Timer-1 NCO instead of a real Mac OS response to a genuine slot
    // interrupt, decoupled from the real displayed frame, and was
    // reverted for killing the VBL-task-driven cursor/floppy animation.
    // This time VIA2.CA1 is wired to the real dafb_irq_w slot-IRQ source
    // above, so PB7 toggles in genuine response to real vblank events,
    // through the real ROM handler instead of a synthetic NCO — the
    // animation-killing failure mode of the 72e4b446 attempt does not
    // apply here.
    // NOTE 2026-07-25: driving VIA1.CA1 from via2_pb_out[7] (the "real"
    // board wire the Mac OS slot handler is supposed to pulse) was tried
    // in build 0x837CC4E6 together with the CA1 edge fix.  Level 2 DID
    // start dispatching and the VBL guard DID start releasing, but the
    // HDMI output went BLACK and boot never completed.  The PB7-pulse
    // assumption was never verified against a ROM trace — it was inferred
    // from 6522 convention — so it is the prime suspect for the lost
    // video.  Revert to the known-good direct VBL level here and keep the
    // (independently proven) VIA2.CA1 edge fix above.  Re-attempt the PB7
    // chain only after confirming, from a MAME trace, that the handler
    // actually toggles VIA2 ORB bit 7.
    // RESOLVED 2026-07-26 — drive VIA1 CA1 from VIA2 PB7, per MAME.
    //
    // The precondition the note above set ("confirm from a MAME trace that
    // the handler actually toggles VIA2 ORB bit 7") has been answered, and
    // the answer is that the premise was WRONG: Mac OS does NOT toggle PB7
    // in software.  A MAME watchpoint on VIA2 ORB (mirror 0x50F02000)
    // across a full disk boot captured only 45 ORB writes TOTAL, all during
    // ROM init, and the count stopped growing as the boot progressed --
    // nothing like a 60 Hz cadence.
    //
    // PB7 is driven by TIMER 1 IN HARDWARE.  Read live over JTAG on build
    // 0x0BC53B30:
    //     VIA2 ACR  = 0xC0  -> ACR[7]=1 = "PB7 output", ACR[6]=1 = T1 free-run
    //     VIA2 T1LH:T1LL = 0x196E = 6510 phi2 ticks
    // and VIA_PHI2_HZ = 783_360 (fpga_top.v:362, NCO-generated so there is
    // no integer-divide drift), giving 6510/783360 = 8.31 ms per half period
    // => a 60.1 Hz square wave on PB7 -- exactly the Q700 VBL rate.
    // via2.v implements this: `t1_pb7 <= ~t1_pb7` on each T1 wrap when
    // ACR[7] (line 425) and `pb_out = acr_t1_pb7 ? {t1_pb7, orb[6:0]} : orb`
    // (line 553).  MAME's via2_out_b does `m_via1->write_ca1(data>>7)`
    // (macquadra700.cpp:653/663), so those hardware toggles ARE the VBL.
    //
    // Driving VIA1 CA1 from the HDMI-VTG-derived `dafb_vbl_level` instead
    // was a shortcut: it happens to be ~60 Hz, but it is the wrong SOURCE.
    // The real machine's VBL is a VIA TIMER, not the video vblank, so the
    // rate must follow whatever the OS programs into T1 -- not our display
    // mode.  (Build 0x837CC4E6 tried PB7 and went black, but at that time
    // VIA2.CA1 and port A were both still mis-wired and the build had 284
    // failing timing endpoints; those are all fixed now.)
    //
    // MEASURED 2026-09-06 — this wire is CORRECT, and it is now gated.
    // `make tb-via-tick-rate` (tb/tb_via_tick_rate.{v,cpp}) rebuilds this
    // exact chain -- the phi2 NCO copied verbatim from
    // fpga_top_clocks.vh:906, the real via2.v and via1.v, and this same
    // `via1_ca1_in = via2_pb_out[7]` assignment -- and measures, in Hz:
    //     phi2_tick                     = 783 360.0 Hz  (exact)
    //     via2_pb_out[7] toggles        = 120.000 Hz
    //     VIA1 CA1 interrupts           =  60.000 Hz    <- the Mac tick
    // with the ROM's own ACR=0xC0 / T1 latch=0x196E.  So the chain is not
    // a rate suspect.  It also measures the FLOOR: at latch 0xFFFF -- the
    // slowest a 16-bit T1 can be programmed -- the tick is 6.000 Hz.  A
    // hardware tick below ~6 Hz therefore cannot originate here at any
    // programming; look at interrupt DELIVERY and servicing instead
    // (irq_agg's strict priority means a held via2_irq presents ipl=2 and
    // starves this level-1 line entirely), not at the timer.
    //
    // The pre-existing 60 Hz gate, tb-vbl-rate, could never have caught a
    // wrong tick rate: it still fed VIA1 CA1 from `dafb_vbl_level`, the
    // pre-PB7 shortcut this comment block replaced.  Its scope has been
    // corrected; see docs/vbl_irq.md "Current chain (2026-09-06)".
    wire via1_ca1_in = via2_pb_out[7];

    // ── DAFB VBL → u_dafb frame_tick (task #T8) ────────────────────────
    // u_dafb (video.v) runs on core_clk, not pb_clk, so `dafb_vbl_level`
    // above needs its own crossing into core_clk before it can drive
    // video.v's `frame_tick` input — the pb_clk-domain CA1 chain above
    // does not, by itself, reach the core_clk domain.  `dafb_vbl_level`
    // is already pulse-extended to VBL_LEVEL_TICKS (32) pb_clk cycles
    // wide, so a plain 2-FF ASYNC_REG synchroniser (the same idiom this
    // file already uses for via1_irq_pb/via2_irq_pb/scsi_irq_pb/etc. and
    // for dafb_scsi0_drq_sync just above) safely resamples it into
    // core_clk without needing a dedicated pulse_cdc instance — this is
    // NOT a new CDC mechanism, just another use of the file's standard
    // slow-status-bit synchroniser.  video.v rising-edge-detects the
    // resampled level internally to produce a single per-frame tick.
    (* ASYNC_REG = "TRUE" *) reg dafb_vbl_level_core_meta;
    (* ASYNC_REG = "TRUE" *) reg dafb_vbl_level_core_sync;
    always @(posedge core_clk) begin
        if (soc_full_rst_bank[2]) begin
            dafb_vbl_level_core_meta <= 1'b0;
            dafb_vbl_level_core_sync <= 1'b0;
        end else begin
            dafb_vbl_level_core_meta <= dafb_vbl_level;
            dafb_vbl_level_core_sync <= dafb_vbl_level_core_meta;
        end
    end

    (* ASYNC_REG = "TRUE" *) reg debug_full_reset_pb_meta;
    (* ASYNC_REG = "TRUE" *) reg debug_full_reset_pb_sync;
    always @(posedge pb_clk) begin
        if (pb_rst) begin
            debug_full_reset_pb_meta <= 1'b0;
            debug_full_reset_pb_sync <= 1'b0;
        end else begin
            debug_full_reset_pb_meta <= jtag_debug_full_reset_eff;
            debug_full_reset_pb_sync <= debug_full_reset_pb_meta;
        end
    end

    // PRAM zap (vio_boot_ctrl[4]) — same sys_clk → pb_clk crossing as
    // debug_full_reset above, same 2-FF ASYNC_REG treatment.  The source
    // is already a fixed-length one-shot (jtag_pram_clear_eff, see
    // fpga_top_clocks.vh) that is DBG_RST_PULSE_CYCLES wide in sys_clk =
    // 256 pb_clk cycles on HW (PB_CLK_RST_DIVIDE=4) / 32 in sim, so the
    // pulse cannot be swallowed by this synchroniser.
    //
    // NOT reset by pb_full_rst on purpose — it is reset by pb_rst (the
    // plain peripheral-bus reset) only.  Tying it to the full-reset cone
    // would let a debug-full-reset cancel an in-flight zap, which is the
    // exact combination an operator uses to recover a wedged boot.
    (* ASYNC_REG = "TRUE" *) reg pram_clear_pb_meta;
    (* ASYNC_REG = "TRUE" *) reg pram_clear_pb_sync;
    always @(posedge pb_clk) begin
        if (pb_rst) begin
            pram_clear_pb_meta <= 1'b0;
            pram_clear_pb_sync <= 1'b0;
        end else begin
            // Second source, ORed in here rather than in fpga_top_clocks.vh
            // so the VIO one-shot stays exactly as extracted by
            // tools/extract_pram_clear_oneshot.py: pram_sd.v asserts
            // `pram_sd_default` (core_clk, held 2^DEFAULT_HOLD_LOG2 = 2048
            // core cycles = 1024 pb_clk cycles) when a `pram-load` cannot
            // validate the saved sector and therefore has to put the array
            // back on rtc.v's OWN post-reset image.  Reusing pram_clear is
            // the whole point — there is one definition of "the defaults"
            // in this design and it is rtc.v's pram_reset_value().
            pram_clear_pb_meta <= jtag_pram_clear_eff | pram_sd_default;
            pram_clear_pb_sync <= pram_clear_pb_meta;
        end
    end

    // ── PRAM snapshot/restore back door (rtl/soc/pram_sd.v) ───────────
    //
    // pram_sd lives on core_clk (it owns an sd_ctrl, and sd_spi is
    // core_clk); rtc.v lives on pb_clk.  pram_cdc.v is the single
    // crossing.  The wires are declared HERE, not in fpga_top_sd.vh,
    // purely because fpga_top.v includes peripherals before sd and the
    // rtc instantiation below needs them.  pram_sd itself, which drives
    // the core-clk side, is instantiated in fpga_top_sd.vh next to the
    // rest of the SD plumbing.
    wire        pram_sd_default;      // core_clk, from pram_sd
    wire        pram_x_req;           // core_clk, from pram_sd
    wire        pram_x_we;
    wire [7:0]  pram_x_addr;
    wire [7:0]  pram_x_wdata;
    wire [7:0]  pram_x_rdata;         // core_clk, to pram_sd
    wire        pram_x_ack;
    wire [7:0]  rtc_pram_ext_addr;    // pb_clk, to rtc
    wire [7:0]  rtc_pram_ext_wdata;
    wire        rtc_pram_ext_we;
    wire [7:0]  rtc_pram_ext_rdata;   // pb_clk, from rtc

    pram_cdc u_pram_cdc (
        .a_clk   (core_clk),
        .a_rst   (core_rst_bank[4]),
        .a_req   (pram_x_req),
        .a_we    (pram_x_we),
        .a_addr  (pram_x_addr),
        .a_wdata (pram_x_wdata),
        .a_rdata (pram_x_rdata),
        .a_ack   (pram_x_ack),
        // pb_rst (not pb_full_rst): a debug-full-reset must not strand a
        // half-finished restore handshake, and the A-side timeout in
        // pram_sd already bounds the case where it does.
        .b_clk   (pb_clk),
        .b_rst   (pb_rst),
        .b_addr  (rtc_pram_ext_addr),
        .b_wdata (rtc_pram_ext_wdata),
        .b_we    (rtc_pram_ext_we),
        .b_rdata (rtc_pram_ext_rdata)
    );
    // pb_full_rst is the pb_clk equivalent of soc_full_rst — board cold
    // reset OR JTAG debug-full-reset (synchronized to pb_clk).  Every
    // peripheral that should be cold-rewound by a debug-full-reset uses
    // this; the previous via1-only `via1_rst_pb` is now an alias.  See
    // task #256 / fpga_top_clocks.vh comment block.
    assign pb_full_rst = pb_rst || debug_full_reset_pb_sync ||
                         warm_peripheral_reset;

    // Broadcast distribution for pb_full_rst (~2458 loads on the
    // post-vivado_main_d1c9f11 BUFG-insertion log).  Earlier attempts
    // hand-banked the source FF or relied on MAX_FANOUT auto-
    // replication; both still tripped Vivado's auto-BUFG path, which
    // costs ~2 ns of data-path delay at every receiving FF (the placer
    // treats the BUFG-driven net as a regular signal, not a clock).
    //
    // Match clk_rst.v's UG974-conformant pattern: route the local
    // pb_full_rst through xpm_cdc_async_rst (so STA understands the
    // synchroniser semantics — replication policies see ASYNC_REG
    // inside the macro, not on user FFs we want replicated downstream)
    // then through an EXPLICIT BUFG, so the broadcast lives on the
    // global clock network where reset arrival is balanced as clock
    // skew instead of being charged to the data path.
    //
    // pb_rst and debug_full_reset_pb_sync are already in pb_clk.  The
    // warm-reset request arrives from core_clk and is intentionally an
    // asynchronous assertion source here.  The XPM macro provides the
    // required synchronous pb_clk deassertion (DEST_SYNC_FF=4), then the
    // explicit BUFG distributes it.
    wire pb_full_rst_unbuf;
    wire pb_full_rst_buf;

    xpm_cdc_async_rst #(
        .DEST_SYNC_FF   (4),
        .INIT_SYNC_FF   (0),
        .RST_ACTIVE_HIGH(1)
    ) u_pb_full_rst_sync (
        .src_arst (pb_full_rst),
        .dest_clk (pb_clk),
        .dest_arst(pb_full_rst_unbuf)
    );

    BUFG u_pb_full_rst_bufg (
        .I(pb_full_rst_unbuf),
        .O(pb_full_rst_buf)
    );

    // pb_full_rst_bank is a 4-bit wire bus kept for source-compatibility
    // with consumers that already index `[N]`; all bits are driven from
    // the single BUFG-buffered broadcast net.  Vivado's global clock
    // network handles physical distribution downstream of the BUFG.
    assign pb_full_rst_bank = {4{pb_full_rst_buf}};

    // Bank bit allocation (driven by pb-side consumer fanout):
    //   pb_full_rst_bank[0] -> via1 (largest single sink)
    //   pb_full_rst_bank[1] -> rtc + via2
    //   pb_full_rst_bank[2] -> scc + scsi
    //   pb_full_rst_bank[3] -> asc + iwm + sonic + i2s + ext bridge sinks
    wire via1_rst_pb   = pb_full_rst_bank[0];

    // VIA1 lives in pb_clk, but debug-full-reset must still return ORB[3]
    // to its post-reset value so the xbar-level low-memory ROM alias is
    // re-asserted.  Only VIA1 gets this extra reset; the rest of the
    // peripheral island stays live across a CPU-side debug restart.
    adb_pic_modem u_adb_pic_modem (
        .clk            (pb_clk),
        .rst            (pb_full_rst_bank[0]),
        .phi2_tick      (phi2_tick),
        .cb1_out        (pic_cb1_out),
        .cb2_out        (pic_cb2_out),
        .cb2_oe         (pic_cb2_oe),
        .cb2_in         (adb_cb2),
        .adb_bus_in     (adb_bus_in),
        .rtcc_in        (adb_rtcc_in),
        .pic_adb_out    (pic_adb_out),
        // ADB state command: the 68k drives VIA1 ORB[5:4]; MAME's
        // macquadra700 via_out_b feeds (ORB & 0x30) >> 4 to the modem.
        .adb_state      (via1_pb_out[5:4]),
        .adb_irq_pending(adb_irq_pending)
    );

    via1 #(
        .ENABLE_INTERNAL_VBL(1'b0)
    ) u_via1 (
        .clk(pb_clk), .rst(via1_rst_pb),
        .phi2_tick (phi2_tick),
        .pb_addr   (pb_via1_addr ),
        .pb_wdata  (pb_via1_wdata),
        .pb_wr     (pb_via1_wr   ),
        .pb_rd     (pb_via1_rd   ),
        .pb_rdata  (pb_via1_rdata),
        .pb_ack    (pb_via1_ack  ),
        // Port A reset strap = 0xC1.  MAME peripheral dump at boot-idle
        // shows VIA1 ORA-handshake (+0x200) and ORA-no-handshake (+0x1E00)
        // both = 0xC1, so the developer-jumper input PA0 is 1 in the
        // default Q700 config (`0xC0 | BIT(config, 0)` with config bit 0
        // tied 1).  Earlier 0xC0 attempts diverged at the d7 bit 26 latch
        // (0x40846d5a) and dropped the boot into the operator prompt at
        // 0x4084a82e.
        .pa_in     (8'hC1),
        .pb_in     (via1_pb_in),
        .pa_out    (via1_pa_out),
        .pa_mask   (via1_pa_mask),
        .pb_out    (via1_pb_out),
        .pb_mask   (via1_pb_mask),
        .overlay_bit(via1_overlay_pb),
        // ADB transceiver, byte-level port: this is NOT the real ADB
        // boot path (see the "ADB modem + bus devices" comment block
        // below) — via1.v's acr_old_sr_path only routes traffic through
        // adb_rx_byte/adb_tx_byte when ACR is NOT programmed for the
        // real external-clock shift modes (011 RX / 111 TX) that real
        // ADB traffic uses, so this port only fires in the synthetic
        // old_byte_test_mode (cb1_in==cb2_in==0) case tb_adb.cpp
        // exercises via adb_modem.v standalone.  adb_rx_valid was
        // previously hardcoded 1'b0 here — dangling, meaning even that
        // test-mode path could never complete an RX handshake.  Wired
        // to the modem's real via_rx_valid output for correctness; real
        // ADB traffic is unaffected either way since acr_old_sr_path
        // gates it off.  CB1/CB2 below carry the REAL ADB traffic to/
        // from u_adb_pic_modem (the real PIC1654S firmware) and
        // u_adb_phy (the bit-level device responder).
        .adb_rx_byte (adb_via_rx_byte),
        .adb_rx_valid(adb_via_rx_valid),
        .adb_rx_ready(via1_adb_rx_ready),
        .adb_tx_byte (via1_adb_tx_byte),
        .adb_tx_valid(via1_adb_tx_valid),
        .cb1_in      (adb_cb1),
        .cb2_in      (adb_cb2),
        .cb2_out     (via1_cb2_out),
        .cb2_oe      (via1_cb2_oe),
        // RTC side-channel
        .rtc_enb    (via1_rtc_enb),
        .rtc_clk    (via1_rtc_clk),
        .rtc_data_o (via1_rtc_data_o),
        .rtc_data_oe(via1_rtc_data_oe),
        .rtc_data_i (via1_rtc_data_i),
        .rtc_cko    (via1_rtc_cko),
        .vblank_irq_in(via1_ca1_in),
        .irq       (via1_irq_pb)
    );

    // ── ADB modem + bus devices ────────────────────────────────────────
    //
    // Two parallel chains exist here:
    //
    //  1. The REAL boot-time path: VIA1's bit-level external-clock SR
    //     shift register (rtl/mac/via1.v ~line 587, ACR[4:2] ∈
    //     {011,111}) talks to u_adb_pic_modem (the real PIC1654S
    //     firmware) over CB1/CB2.  The PIC's own ADB-bus pin
    //     (pic_adb_out / adb_bus_in) is answered at the bit level by
    //     u_adb_phy, which now decodes real ADB commands and drives
    //     real open-drain response frames — see rtl/mac/adb_phy.v.
    //     u_adb_phy fans decoded commands out to u_adb_kbd / u_adb_mouse
    //     via the adb_dev_cmd_*/adb_dev_listen_* wires declared above
    //     (next to u_adb_phy's instantiation).
    //
    //  2. u_adb_modem below is a byte-level "cousin" abstraction
    //     (VIA1.adb_tx_*/adb_rx_* <-> modem <-> same kbd/mouse device-
    //     bus) that bypasses the PIC/CB1/CB2 bit-banging entirely.  It
    //     is NOT part of the real boot path: via1.v only routes
    //     adb_tx_valid/adb_rx_valid through this byte port when
    //     acr_old_sr_path is true, which real ADB traffic (ACR[4:2] =
    //     011/111) never satisfies — and via1_adb_rx_valid (this
    //     modem's reply) is not even wired into VIA1's adb_rx_valid
    //     input below (hardcoded 1'b0).  It exists purely so
    //     tb/tb_adb.cpp can exercise adb_modem.v + adb_keyboard.v +
    //     adb_mouse.v in isolation; left in place, inert, for that
    //     regression coverage.  Its dev_cmd_* outputs are intentionally
    //     unconnected (orphan wires below) so they don't contend with
    //     u_adb_phy for the shared kbd/mouse device-bus.

    wire [7:0]  adb_via_rx_byte;
    wire        adb_via_rx_valid;
    wire        adb_modem_irq_unused;

    // u_adb_modem's device-bus outputs are orphaned on purpose (see
    // above) — u_adb_phy is the sole real driver of adb_dev_cmd_*.
    /* verilator lint_off UNUSEDSIGNAL */
    wire        adb_modem_dev_cmd_valid;
    wire [3:0]  adb_modem_dev_cmd_addr;
    wire [2:0]  adb_modem_dev_cmd_op;
    wire        adb_modem_dev_listen_valid;
    wire [7:0]  adb_modem_dev_listen_b0;
    wire [7:0]  adb_modem_dev_listen_b1;
    /* verilator lint_on UNUSEDSIGNAL */

    wire        adb_inj_kc_valid;
    wire [7:0]  adb_inj_kc_byte;
    wire [7:0]  adb_kbd_status;

    wire        adb_inj_btn_valid;
    wire        adb_inj_btn_state;
    wire        adb_inj_dx_valid;
    wire signed [7:0] adb_inj_dx;
    wire        adb_inj_dy_valid;
    wire signed [7:0] adb_inj_dy;
    wire [7:0]  adb_mouse_status;

    /* verilator lint_off UNUSEDSIGNAL */
    wire [3:0]  adb_dbg_state;
    /* verilator lint_on UNUSEDSIGNAL */

    adb_modem #(
        .NUM_DEVICES     (2),
        .NO_SERVICE_DELAY(16)
    ) u_adb_modem (
        .clk             (pb_clk),
        // pb_full_rst — modem state cycles on debug-full-reset alongside
        // VIA1 so a JTAG/btn cold reset resets ADB consistently.  pb-
        // bank [0] (same as VIA1).
        .rst             (pb_full_rst_bank[0]),
        .phi2_tick       (phi2_tick),
        .via_rx_byte     (adb_via_rx_byte),
        .via_rx_valid    (adb_via_rx_valid),
        .via_rx_ready    (via1_adb_rx_ready),
        .via_tx_byte     (via1_adb_tx_byte),
        .via_tx_valid    (via1_adb_tx_valid),
        .adb_irq_pending (adb_modem_irq_unused),
        .dev_cmd_valid   (adb_modem_dev_cmd_valid),
        .dev_cmd_addr    (adb_modem_dev_cmd_addr),
        .dev_cmd_op      (adb_modem_dev_cmd_op),
        .dev_listen_valid(adb_modem_dev_listen_valid),
        .dev_listen_b0   (adb_modem_dev_listen_b0),
        .dev_listen_b1   (adb_modem_dev_listen_b1),
        .dev_resp_valid  ({adb_ms_resp_valid, adb_kbd_resp_valid}),
        .dev_resp_empty  ({adb_ms_resp_empty, adb_kbd_resp_empty}),
        .dev_resp_b0     ({adb_ms_resp_b0,    adb_kbd_resp_b0}),
        .dev_resp_b1     ({adb_ms_resp_b1,    adb_kbd_resp_b1}),
        .dev_srq         ({adb_ms_srq,        adb_kbd_srq}),
        .dbg_state       (adb_dbg_state)
    );

    adb_keyboard u_adb_kbd (
        .clk             (pb_clk),
        .rst             (pb_full_rst_bank[0]),
        .dev_cmd_valid   (adb_dev_cmd_valid),
        .dev_cmd_addr    (adb_dev_cmd_addr),
        .dev_cmd_op      (adb_dev_cmd_op),
        .dev_resp_valid  (adb_kbd_resp_valid),
        .dev_resp_empty  (adb_kbd_resp_empty),
        .dev_resp_b0     (adb_kbd_resp_b0),
        .dev_resp_b1     (adb_kbd_resp_b1),
        .dev_listen_valid(adb_dev_listen_valid),
        .dev_listen_b0   (adb_dev_listen_b0),
        .dev_listen_b1   (adb_dev_listen_b1),
        .dev_srq         (adb_kbd_srq),
        .inj_kc_valid    (adb_inj_kc_valid),
        .inj_kc_byte     (adb_inj_kc_byte),
        .inj_status      (adb_kbd_status)
    );

    adb_mouse u_adb_mouse (
        .clk             (pb_clk),
        .rst             (pb_full_rst_bank[0]),
        .dev_cmd_valid   (adb_dev_cmd_valid),
        .dev_cmd_addr    (adb_dev_cmd_addr),
        .dev_cmd_op      (adb_dev_cmd_op),
        .dev_resp_valid  (adb_ms_resp_valid),
        .dev_resp_empty  (adb_ms_resp_empty),
        .dev_resp_b0     (adb_ms_resp_b0),
        .dev_resp_b1     (adb_ms_resp_b1),
        .dev_listen_valid(adb_dev_listen_valid),
        .dev_listen_b0   (adb_dev_listen_b0),
        .dev_listen_b1   (adb_dev_listen_b1),
        .dev_srq         (adb_ms_srq),
        .inj_btn_valid   (adb_inj_btn_valid),
        .inj_btn_state   (adb_inj_btn_state),
        .inj_dx_valid    (adb_inj_dx_valid),
        .inj_dx          (adb_inj_dx),
        .inj_dy_valid    (adb_inj_dy_valid),
        .inj_dy          (adb_inj_dy),
        .inj_status      (adb_mouse_status)
    );

    adb_inject u_adb_inject (
        .clk             (pb_clk),
        .rst             (pb_full_rst_bank[0]),
        .pb_addr         (pb_adbinj_addr),
        .pb_wdata        (pb_adbinj_wdata),
        .pb_wr           (pb_adbinj_wr),
        .pb_rd           (pb_adbinj_rd),
        .pb_rdata        (pb_adbinj_rdata),
        .pb_ack          (pb_adbinj_ack),
        .inj_kc_valid    (adb_inj_kc_valid),
        .inj_kc_byte     (adb_inj_kc_byte),
        .kbd_status      (adb_kbd_status),
        .inj_btn_valid   (adb_inj_btn_valid),
        .inj_btn_state   (adb_inj_btn_state),
        .inj_dx_valid    (adb_inj_dx_valid),
        .inj_dx          (adb_inj_dx),
        .inj_dy_valid    (adb_inj_dy_valid),
        .inj_dy          (adb_inj_dy),
        .mouse_status    (adb_mouse_status)
    );

    // Off-chip RTC peripheral — seconds counter + command shift FSM.
    rtc #(
        .SEC_DIV(VIA_PHI2_HZ)
    ) u_rtc (
        // pb_full_rst — RTC clock-shift FSM rewinds on debug-full-reset.
        // The 32-bit seconds counter naturally restarts at 0 too.
        //
        // PRAM, however, now SURVIVES this reset: it models the
        // battery-backed parameter RAM of a real Quadra 700, so user
        // settings (display depth, boot device, volume) persist across a
        // warm reset exactly as they do in silicon.  Its power-on image
        // is the configuration-time `initial` in rtc.v — all-zero under
        // SYNTHESIS, i.e. a real bitstream load still comes up with a
        // blank PRAM, unchanged from before.  pb-bank [1].
        .clk(pb_clk), .rst(pb_full_rst_bank[1]),
        .phi2_tick  (phi2_tick),
        .rtc_enb    (via1_rtc_enb),
        .rtc_clk    (via1_rtc_clk),
        .rtc_data_o (via1_rtc_data_o),
        .rtc_data_oe(via1_rtc_data_oe),
        // PRAM zap (Cmd-Opt-P-R equivalent), driven from VIO probe-out
        // vio_boot_ctrl[4] via a rising-edge one-shot (fpga_top_clocks.vh)
        // and the 2-FF pb_clk synchroniser above.  This is the recovery
        // path if a bad PRAM image ever wedges the ROM boot.  JTAG REPL
        // command: `pram-clear`.
        .pram_clear (pram_clear_pb_sync),
        // Snapshot/restore back door for rtl/soc/pram_sd.v — manual JTAG
        // `pram-save` / `pram-load` only.  Not reachable from the 68k.
        .pram_ext_addr  (rtc_pram_ext_addr ),
        .pram_ext_we    (rtc_pram_ext_we   ),
        .pram_ext_wdata (rtc_pram_ext_wdata),
        .pram_ext_rdata (rtc_pram_ext_rdata),
        .cko        (via1_rtc_cko),
        .rtc_data_i (via1_rtc_data_i)
    );

    via2 u_via2 (
        // pb_full_rst — VIA2 NuBus IRQ latches and timers rewind on a
        // debug-full-reset alongside VIA1.  pb-bank [1].
        .clk(pb_clk), .rst(pb_full_rst_bank[1]),
        .phi2_tick(phi2_tick),
        .pb_addr (pb_via2_addr ),
        .pb_wdata(pb_via2_wdata),
        .pb_wr   (pb_via2_wr   ),
        .pb_rd   (pb_via2_rd   ),
        .pb_rdata(pb_via2_rdata),
        .pb_ack  (pb_via2_ack  ),
        .pa_in   (via2_pa_in   ),
        .pa_out  (via2_pa_out  ),
        .pa_mask (via2_pa_mask ),
        .pb_in   (via2_pb_in   ),
        .pb_out  (via2_pb_out  ),
        .pb_mask (via2_pb_mask ),
        // via2_ca1_in: real DAFB built-in-video slot-IRQ (virtual slot
        // $F), synchronised + polarity-matched above.  See the "DAFB
        // slot-IRQ -> VIA2 CA1" comment block just above via2_pa_in/
        // via2_pb_in for the full derivation.
        .ca1_in  (via2_ca1_in),
        // ca2_in: the SWIM (floppy) controller's interrupt.  MAME
        // macquadra700.cpp:875 is authoritative:
        //     m_swimpic->hint_callback().set(m_via2,
        //         FUNC(via6522_device::write_ca2)).invert();
        // so VIA2 CA2 is the SWIM hint line, ACTIVE LOW at the 6522 pin --
        // the same sense as cb1_in/cb2_in just below.
        //
        // This was previously tied to 1'b1 with a comment asserting VIA2
        // CA2 is "genuinely unused on Q700 -- no known Q700 peripheral
        // drives VIA2 CA2".  That claim is WRONG, and it is the same
        // wrong-comment-over-a-dead-interrupt-input pattern that hid the
        // VIA2 CA1 tie-off (fixed dc7e299) and the port-A mis-mapping.
        // `iwm_irq_w` was already being generated by u_iwm and wired to a
        // top-level net -- it was simply CONSUMED NOWHERE, so the floppy
        // interrupt was produced and then discarded.
        //
        // The Q700 ROM programs VIA2 PCR = 0x22, i.e. PCR[3:1] = 001 =
        // "CA2 independent interrupt input, negative edge" (read live over
        // JTAG 2026-07-26), so it genuinely expects an edge on this pin.
        //
        // WHAT IS AND IS NOT FIXED HERE (be precise, 2026-08-19).  The
        // WIRING above is correct and is what was fixed: CA2 is no longer
        // tied to 1'b1, and the day u_iwm raises a hint the ROM will see
        // the negative edge with no further change at this seam.  The
        // SOURCE is still constant: `iwm_irq_w` is driven by u_iwm's
        // `irq` output, and rtl/mac/iwm_stub.v assigns that `1'b0`
        // unconditionally (iwm_stub.v:83).  So `~iwm_irq_w` is constant
        // 1'b1 today, Vivado folds it, and VIA2 CA2 STILL CANNOT TAKE AN
        // EDGE.  Do not read the paragraphs above as "the floppy
        // interrupt now fires" — it does not, and cannot, until the stub
        // grows real SWIM/ISM hint generation.
        //
        // Why the source is dead rather than merely unfinished:
        // iwm_stub.v is a no-media REGISTER stub.  It models the Q700
        // power-on state "drive attached, no disk" and deliberately has
        // no seek/read/write/error machinery, so there is no condition in
        // it that a real SWIM would raise ism_hint for.  Making the IRQ
        // real means implementing ISM hint generation (mode-register
        // interrupt enables + the ERROR/data/handshake conditions MAME's
        // swim1_device raises) — a FEATURE, correctly scoped as its own
        // task, not a wire fix.  The same is true of `iwm_dma_req_w`
        // (iwm_stub.v:84, also constant 1'b0) and `iwm_mode_w`, both of
        // which are driven and consumed nowhere.
        .ca2_in  (~iwm_irq_w),
        // Q700 board glue routes sound and SCSI service through VIA2,
        // active low at the 6522 input pins.
        .cb1_in  (~asc_irq_pb),
        .cb2_in  (~scsi_irq_pb),
        .ca2_out (via2_ca2_out),
        .cb1_out (via2_cb1_out),
        .cb2_out (via2_cb2_out),
        .irq     (via2_irq_pb  )
    );

    // PCLK_DIV DERIVED, not defaulted (fix 2026-08-19).  scc.v's default is
    // 54, computed in its header as "200e6 / 3.672e6" — but this instance
    // runs on `pb_clk`, which is PB_CLK_HZ = 50 MHz, not 200.  The SCC's
    // BRG reference was therefore 50e6/54 = 926 kHz against the Q700's real
    // PCLK of 3.672 MHz, i.e. **every baud rate ~4x too slow**.
    //
    // tb-scc cannot catch this: it overrides PCLK_DIV to 2 to make a byte
    // time fit in a few simulation cycles, so the shipped value is never
    // exercised by any test.
    //
    // Deriving it from PB_CLK_HZ means a future clock change cannot
    // silently reintroduce the skew.  Rounded to nearest:
    // (50e6 + 1.836e6) / 3.672e6 = 14.
    scc #(
        .PCLK_DIV((PB_CLK_HZ + 32'd1_836_000) / 32'd3_672_000)
    ) u_scc (
        // pb_full_rst — SCC TX/RX FIFO state, BRG counters and IRQ
        // latches rewind so the post-reset CPU sees a freshly-init'd
        // serial channel.  pb-bank [2].
        .clk(pb_clk), .rst(pb_full_rst_bank[2]),
        .pb_addr (pb_scc_addr ),
        .pb_wdata(pb_scc_wdata),
        .pb_wr   (pb_scc_wr   ),
        .pb_rd   (pb_scc_rd   ),
        .pb_rdata(pb_scc_rdata),
        .pb_ack  (pb_scc_ack  ),
        .rx_a_valid(scc_uart_sel_a ? scc_uart_rx_valid : 1'b0),
        .rx_a_data (scc_uart_rx_data),
        .rx_b_valid(scc_uart_sel_b ? scc_uart_rx_valid : 1'b0),
        .rx_b_data (scc_uart_rx_data),
        .cts_a_n   (1'b1),
        .dcd_a_n   (1'b1),
        .sync_a_n  (1'b1),
        .cts_b_n   (1'b1),
        .dcd_b_n   (1'b1),
        .sync_b_n  (1'b1),
        .tx_a_valid(scc_tx_a_valid),
        .tx_a_data (scc_tx_a_data ),
        .tx_b_valid(scc_tx_b_valid),
        .tx_b_data (scc_tx_b_data ),
        .rts_a_n   (scc_rts_a_n   ),
        .dtr_a_n   (scc_dtr_a_n   ),
        .rts_b_n   (scc_rts_b_n   ),
        .dtr_b_n   (scc_dtr_b_n   ),
        .irq     (scc_irq_pb  )
    );

    // SCSI NCR 5380 target emulation — full phase FSM.  It talks to a
    // virtual HDD over the vhdd contract (rtl/vhdd.vh) and knows nothing
    // about how the blocks are stored; vhdd_sd below is the provider that
    // maps that volume onto the SD card.  Below vhdd_sd, the request goes
    // through sd_scsi_bridge to a dedicated sd_ctrl instance on core_clk
    // (see fpga_top_sd.vh).  The bridge handles the pb_clk ↔ core_clk
    // crossing; sd_spi runs on core_clk so it can hit 50 MHz SPI in HS
    // mode.  The SCSI side of sd_spi_mux's phase-B is driven once
    // boot_fsm releases the bus (b_sel ← boot_rom_loaded).
    /* verilator lint_off UNUSEDSIGNAL */
    wire        scsi_drq_w;
    wire        scsi_dma_rd_ready_w;
    wire        scsi_dma_wr_ready_w;
    wire        pb_scsi_dma16_lo_beat;
    wire [83:0] scsi_dbg_c96_state_w;
    wire        scsi_sd_rd_ready_w;
    wire        scsi_sd_wr_valid_w;
    /* verilator lint_on UNUSEDSIGNAL */
    // scsi.v ↔ vhdd_sd (the vhdd contract, pb_clk).
    wire [31:0] scsi_vh_chk_lba_w;
    wire [23:0] scsi_vh_chk_blocks_w;
    wire        scsi_vh_chk_ok_w;
    wire        scsi_vh_req_write_w;
    wire        scsi_vh_req_multi_w;
    wire [31:0] scsi_vh_req_lba_w;
    wire [15:0] scsi_vh_req_block_count_w;
    wire        scsi_vh_req_go_w;
    wire        scsi_vh_busy_w;
    wire        scsi_vh_done_w;
    wire        scsi_vh_error_w;
    wire        scsi_vh_rd_valid_w;
    wire [7:0]  scsi_vh_rd_data_w;
    wire        scsi_vh_rd_ready_w;
    wire        scsi_vh_wr_ready_w;
    wire        scsi_vh_wr_valid_w;
    wire [7:0]  scsi_vh_wr_data_w;
    wire        scsi_vh_wr_avail_w;
    // vhdd_readahead ↔ vhdd_sd (the vhdd contract again, pb_clk).
    // The read-ahead cache is a vhdd-to-vhdd shim: it is the PROVIDER on
    // the mux's A port and the MASTER on vhdd_sd.  See
    // rtl/soc/vhdd_readahead.v for why it exists (the measured 280x) and
    // what it costs.
    wire [31:0] vhc_chk_lba_w;
    wire [23:0] vhc_chk_blocks_w;
    wire        vhc_chk_ok_w;
    wire        vhc_req_write_w;
    wire        vhc_req_multi_w;
    wire [31:0] vhc_req_lba_w;
    wire [15:0] vhc_req_block_count_w;
    wire        vhc_req_go_w;
    wire        vhc_busy_w;
    wire        vhc_done_w;
    wire        vhc_error_w;
    wire        vhc_rd_valid_w;
    wire [7:0]  vhc_rd_data_w;
    wire        vhc_rd_ready_w;
    wire        vhc_wr_ready_w;
    wire        vhc_wr_valid_w;
    wire [7:0]  vhc_wr_data_w;
    wire        vhc_wr_avail_w;

    // vhdd_sd ↔ sd_scsi_bridge (SD transport, pb_clk).
    wire [2:0]  scsi_sd_cmd_type_w;
    wire [31:0] scsi_sd_lba_w;
    wire [15:0] scsi_sd_block_count_w;
    wire        scsi_sd_go_w;
    wire [7:0]  scsi_sd_wr_data_w;
    // Bridge outputs (driven from rtl/fpga_top_sd.vh after sd_scsi_bridge
    // and sd_ctrl_scsi are instantiated; both files are part of the same
    // fpga_top module body).
    wire        scsi_sd_busy_w;
    wire        scsi_sd_done_w;
    wire        scsi_sd_error_w;
    wire        scsi_sd_rd_valid_w;
    wire [7:0]  scsi_sd_rd_data_w;
    wire        scsi_sd_wr_ready_w;
    wire        scsi_sd_wr_avail_w;

    // Exposed disk size = card sectors - the reserved ROM window, CLAMPED.
    //
    // Two clamps, both load-bearing:
    //  * lower: 0 if the card is smaller than the reserved window, so an
    //    implausibly small/undetected card cannot underflow into a huge value.
    //  * upper: SCSI_MAX_LBAS.  Reporting the RAW card size is wrong for this
    //    machine — a modern 8-32 GB SD card would advertise a multi-gigabyte
    //    SCSI disk to System 7, which predates large-drive support (the 2 GB
    //    HFS/driver barrier is real), while the provisioned image's partition
    //    map declares only ~1e6 blocks.  That mismatch between READ CAPACITY
    //    and the on-disk DDM is exactly the sort of thing an old driver
    //    mishandles.  Cap at 2 GB worth of 512-byte sectors.
    //
    // NOTE (2026-07-28): enabling CSD-derived capacity measurably CHANGED boot
    // behaviour (the deadlock moved from 342 to ~357 SCSI transactions), so
    // this value is not inert — keep it in a range the OS was designed for.
    // REGISTERED, not combinational.  The first cut of this clamp was a pair of
    // 32-bit compares + muxes feeding the SCSI target's last_lba and the LBA
    // mapper, and the build came back at WNS -0.056 ns / 94 failing endpoints
    // (the previous bitstream closed clean).  card_num_lbas settles once
    // during SD init and never changes afterwards, so there is no reason for
    // this to be combinational — register it and let the consumers see a
    // plain flop output.
    // 2 GiB MINUS 32 KiB, deliberately NOT 2 GiB exactly.
    //
    // 4194304 LBAs x 512 B is exactly 2^31 bytes, which is the signed
    // 32-bit boundary.  Era-appropriate Mac software computes capacity in
    // BYTES as a signed long, so an exactly-2-GiB volume reads back
    // negative -- TattleTech reported this disk's capacity as 0 on live
    // hardware 2026-08-20 while the SCSI layer was reporting correctly
    // (num_lbas 0x400000, medium present, no check conditions, reads
    // working).  The bug was this constant, not the target.
    //
    // 4194240 = 0x3FFFC0 keeps the byte count at 0x7FFF_E000, clear of the    // boundary, and is a whole multiple of 64 blocks so it stays
    // cylinder-aligned for geometry that cares.
    localparam [31:0] SCSI_MAX_LBAS = 32'd4194240;   // 2 GiB - 32 KiB
    // The reserved SD window, in one place.  It is the vhdd_sd provider's
    // property (it owns the bias), but the capacity reported to the OS
    // has to exclude exactly the same window — so the clamp below and the
    // provider's RESERVED_LBAS parameter are driven from this single
    // localparam rather than from two copies of 8192 that can drift.
    localparam [31:0] SD_RESERVED_LBAS = 32'd8192;   // 4 MiB / 512 B
    reg [31:0] scsi_disk_num_lbas;

    // ═══════════════════════════════════════════════════════════════════
    // vHDD seam: one SCSI master, two providers
    // ═══════════════════════════════════════════════════════════════════
    //   u_scsi ──vhdd──► u_vhdd_mux ──► u_vhdd_sd  (ID 0, SD, persistent)
    //                               ──► provider B  (ID 1, currently only
    //                                               under ENABLE_NET_VHDD)
    //
    // The mux is the module rtl/vhdd.vh predicted ("when a second
    // provider lands, the selection between providers is real logic and a
    // real module can earn its place then").  It routes on vh_dev_sel,
    // which u_scsi latches at the instant a selection succeeds.
    wire        scsi_vh_dev_sel_w;
    wire [31:0] scsi_vh_num_lbas_w;

    // mux ↔ provider A (SD)
    wire [31:0] vha_chk_lba_w;
    wire [23:0] vha_chk_blocks_w;
    wire        vha_chk_ok_w;
    wire        vha_req_write_w;
    wire        vha_req_multi_w;
    wire [31:0] vha_req_lba_w;
    wire [15:0] vha_req_block_count_w;
    wire        vha_req_go_w;
    wire        vha_busy_w;
    wire        vha_done_w;
    wire        vha_error_w;
    wire        vha_rd_valid_w;
    wire [7:0]  vha_rd_data_w;
    wire        vha_rd_ready_w;
    wire        vha_wr_ready_w;
    wire        vha_wr_valid_w;
    wire [7:0]  vha_wr_data_w;
    wire        vha_wr_avail_w;

    // mux ↔ provider B (SCSI ID 1; see the provider block below)
    wire [31:0] vhb_chk_lba_w;
    wire [23:0] vhb_chk_blocks_w;
    wire        vhb_chk_ok_w;
    wire        vhb_req_write_w;
    wire        vhb_req_multi_w;
    wire [31:0] vhb_req_lba_w;
    wire [15:0] vhb_req_block_count_w;
    wire        vhb_req_go_w;
    wire        vhb_busy_w;
    wire        vhb_done_w;
    wire        vhb_error_w;
    wire        vhb_rd_valid_w;
    wire [7:0]  vhb_rd_data_w;
    wire        vhb_rd_ready_w;
    wire        vhb_wr_ready_w;
    wire        vhb_wr_valid_w;
    wire [7:0]  vhb_wr_data_w;
    wire        vhb_wr_avail_w;

    // ── core_clk → pb_clk: host-set static config ─────────────────────
    // dev_en and the RAM-disk size come from u_vhdd_ctrl (core_clk, xbar
    // S2 register file).  Same ASYNC_REG idiom this file already uses for
    // dafb_scsi0_ctrl_pb.  These are STATIC CONFIG: a JTAG write changes
    // them, nothing else does.
    //
    // dev_en is 2 independent bits, so a 1-cycle skew between them is
    // harmless.  rd_blocks is a 32-bit vector and a plain 2-FF sync CAN
    // present a mixed value for one edge, so the pb-side copy is only
    // updated while the RAM-disk provider is IDLE — a size change can
    // then never land mid-extent-check or mid-transfer.  (Changing the
    // size while Mac OS has the volume mounted still confuses the OS;
    // that is a host-side concern, and jtag_repl.tcl warns about it.)
    (* ASYNC_REG = "TRUE" *) reg [1:0]  vhdd_dev_en_meta;
    (* ASYNC_REG = "TRUE" *) reg        vhdd_wprot_meta, vhdd_wprot_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0]  vhdd_dev_en_sync;
    (* ASYNC_REG = "TRUE" *) reg [31:0] vhdd_rd_blocks_meta;
    (* ASYNC_REG = "TRUE" *) reg [31:0] vhdd_rd_blocks_sync;
    reg [31:0] vhdd_rd_blocks_pb;
`ifdef ENABLE_NET_VHDD
    // The Ethernet-backed volume occupies the slot the RAM disk vacated, so
    // ID 1 has a provider again and the mask below must NOT apply.  It still
    // ships disabled: CTRL_RESET is 3'b001, so the CSR bit has to be set
    // (vhdd-enable) before scsi.v will answer to ID 1 -- and the volume is
    // useless until `vhdd-net` has configured the endpoint anyway.
    wire [1:0] vhdd_dev_en_pb = vhdd_dev_en_sync;
`else
    // No provider B: force its enable low so scsi.v never accepts a
    // selection of SCSI ID 1.  vhdd_mux is a combinational selector on
    // dev_sel -- if dev_sel could go high with no B provider, the master
    // would wait forever on a port nothing drives.  Masking here (the one
    // point dev_en reaches the pb domain) makes that unreachable by
    // construction rather than by convention.  The CSR bit still exists
    // and still reads back; it simply has no target.
    wire [1:0] vhdd_dev_en_pb = {1'b0, vhdd_dev_en_sync[0]};
`endif
    wire       vhdd_rd_busy_pb;
    wire       vhdd_rd_error_pb;
    wire [3:0] vhdd_rd_state_pb;
    wire [15:0] vhdd_rd_wdog_pb;

    always @(posedge pb_clk) begin
        if (pb_full_rst_bank[2]) begin
            vhdd_dev_en_meta    <= 2'b01;   // SD only until the CSR is read
            vhdd_dev_en_sync    <= 2'b01;
            // Fail SAFE on reset: hold the volume locked until the CSR value
            // has actually crossed, so a reset can never open a window in
            // which writes reach the card unprotected.
            vhdd_wprot_meta     <= 1'b1;
            vhdd_wprot_sync     <= 1'b1;
            vhdd_rd_blocks_meta <= 32'd0;
            vhdd_rd_blocks_sync <= 32'd0;
            vhdd_rd_blocks_pb   <= 32'd0;
        end else begin
            vhdd_dev_en_meta    <= vhdd_dev_en_core;
            vhdd_dev_en_sync    <= vhdd_dev_en_meta;
            vhdd_wprot_meta     <= vhdd_sd_wprot_core;
            vhdd_wprot_sync     <= vhdd_wprot_meta;
            vhdd_rd_blocks_meta <= vhdd_rd_blocks_core;
            vhdd_rd_blocks_sync <= vhdd_rd_blocks_meta;
            if (!vhdd_rd_busy_pb)
                vhdd_rd_blocks_pb <= vhdd_rd_blocks_sync;
        end
    end

    // ── pb_clk → core_clk: provider-B status, for the CSR ─────────────
    // Read-only telemetry; a sampled-in-a-gap value is acceptable (the
    // same caveat that applies to every status snapshot in this file).
    (* ASYNC_REG = "TRUE" *) reg [21:0] vhdd_rd_stat_meta;
    (* ASYNC_REG = "TRUE" *) reg [21:0] vhdd_rd_stat_sync;
    always @(posedge core_clk) begin
        if (soc_full_rst_bank[5]) begin
            vhdd_rd_stat_meta <= 22'd0;
            vhdd_rd_stat_sync <= 22'd0;
        end else begin
            vhdd_rd_stat_meta <= {vhdd_rd_wdog_pb, vhdd_rd_state_pb,
                                  vhdd_rd_error_pb, vhdd_rd_busy_pb};
            vhdd_rd_stat_sync <= vhdd_rd_stat_meta;
        end
    end
    assign vhdd_rd_busy_core  = vhdd_rd_stat_sync[0];
    assign vhdd_rd_error_core = vhdd_rd_stat_sync[1];
    assign vhdd_rd_state_core = vhdd_rd_stat_sync[5:2];
    assign vhdd_rd_wdog_core  = vhdd_rd_stat_sync[21:6];
    assign vhdd_sd_blocks_core = scsi_disk_num_lbas;

    always @(posedge core_clk) begin
        if (core_rst) begin
            scsi_disk_num_lbas <= 32'd0;
        end else if (boot_card_num_lbas <= SD_RESERVED_LBAS) begin
            scsi_disk_num_lbas <= 32'd0;
        end else if ((boot_card_num_lbas - SD_RESERVED_LBAS) > SCSI_MAX_LBAS) begin
            scsi_disk_num_lbas <= SCSI_MAX_LBAS;
        end else begin
            scsi_disk_num_lbas <= boot_card_num_lbas - SD_RESERVED_LBAS;
        end
    end
    // TWO target IDs on the bus: ID 0 is the persistent SD-backed volume,
    // ID 1 is provider B (only built under ENABLE_NET_VHDD today; the
    // DDR-backed RAM disk that used to live there was removed 2026-09-10).
    // ID 1 is the lowest free Apple-convention ID (0 = internal HD,
    // 7 = initiator) and stays clear of ID 3, traditionally the CD-ROM.
    // scsi.v CHECK CONDITION attribution (see .dbg_* below).  These name
    // WHICH arm returned CHECK CONDITION -- information the 53C96 trace
    // ring does NOT carry even when built in (the ring records register
    // traffic, not which internal arm produced a status byte), so these
    // stay useful independently of ENABLE_SCSI_TRACE.
    wire        scsi_dbg_chk_ok_w;
    wire [23:0] scsi_dbg_xfer_blocks_w;
    wire [31:0] scsi_dbg_xfer_lba_w;
    wire        scsi_dbg_mnp_w;
    wire [3:0]  scsi_dbg_sense_key_w;
    wire [7:0]  scsi_dbg_sense_asc_w;
    wire [7:0]  scsi_dbg_cc_count_w;

    scsi #(
        .TURBOSCSI_C96(1'b1),
        .TARGET_ID    (3'd0),
        .TARGET_ID_B  (3'd1),
        .TARGET_B_EN  (1'b1)
    ) u_scsi (
        .wprot(vhdd_wprot_sync),
        // Debug observability: which CHECK CONDITION arm fired.  scsi.v can
        // return CHECK CONDITION on paths that leave sd_ctrl's counters
        // untouched and complete no back-end request, so a live CHECK
        // CONDITION is otherwise unattributable (the 53C96 trace ring
        // records register traffic, not the arm that produced a status
        // byte).  Surfaced on vio_scsi_sd below.
        .dbg_chk_ok            (scsi_dbg_chk_ok_w),
        .dbg_xfer_blocks       (scsi_dbg_xfer_blocks_w),
        .dbg_xfer_lba          (scsi_dbg_xfer_lba_w),
        .dbg_medium_not_present(scsi_dbg_mnp_w),
        .dbg_sense_key         (scsi_dbg_sense_key_w),
        .dbg_sense_asc         (scsi_dbg_sense_asc_w),
        .dbg_check_cond_count  (scsi_dbg_cc_count_w),
        // 53C96 initiator-state probe -> vio_scsi_c96 (probe_in26).
        // Observation only; see the port decl in rtl/mac/scsi.v.
        .dbg_c96_state         (scsi_dbg_c96_state_w),
        // Capacity of whichever volume the live transaction belongs to —
        // selected by u_vhdd_mux below from vh_dev_sel.
        .vh_num_lbas(scsi_vh_num_lbas_w),
        // pb_full_rst — SCSI phase FSM rewinds to BUS_FREE on debug-
        // full-reset; in-flight CDB bytes and sector buffer state drop.
        // pb-bank [2].
        .clk(pb_clk), .rst(pb_full_rst_bank[2]),
        .dev_en(vhdd_dev_en_pb),
        .pb_addr (pb_scsi_addr ),
        .pb_wdata(pb_scsi_wdata),
        .pb_wr   (pb_scsi_wr   ),
        .pb_rd   (pb_scsi_rd   ),
        .pb_rdata(pb_scsi_rdata),
        .pb_ack  (pb_scsi_ack  ),
        .irq     (scsi_irq_pb  ),
        .drq     (scsi_drq_w   ),
        .dma_rd_ready (scsi_dma_rd_ready_w),
        .dma_wr_ready (scsi_dma_wr_ready_w),
        // Virtual HDD — provider is u_vhdd_sd just below.
        .vh_chk_lba        (scsi_vh_chk_lba_w),
        .vh_chk_blocks     (scsi_vh_chk_blocks_w),
        .vh_chk_ok         (scsi_vh_chk_ok_w),
        .vh_req_write      (scsi_vh_req_write_w),
        .vh_req_multi      (scsi_vh_req_multi_w),
        .vh_req_lba        (scsi_vh_req_lba_w),
        .vh_req_block_count(scsi_vh_req_block_count_w),
        .vh_req_go         (scsi_vh_req_go_w),
        .vh_busy           (scsi_vh_busy_w),
        .vh_done           (scsi_vh_done_w),
        .vh_error          (scsi_vh_error_w),
        .vh_rd_valid       (scsi_vh_rd_valid_w),
        .vh_rd_data        (scsi_vh_rd_data_w),
        .vh_rd_ready       (scsi_vh_rd_ready_w),
        .vh_wr_ready       (scsi_vh_wr_ready_w),
        .vh_wr_valid       (scsi_vh_wr_valid_w),
        .vh_wr_data        (scsi_vh_wr_data_w),
        .vh_wr_avail       (scsi_vh_wr_avail_w),
        .vh_dev_sel        (scsi_vh_dev_sel_w),
        // TurboSCSI shim — DAFB +0x24 ctrl word (DRQ-check enables) sync'd
        // into pb_clk just above.  scsi.v gates pb_ack on the DMA window
        // when DRQ is required but not yet asserted.
        .scsi_ctrl_in  (dafb_scsi0_ctrl_pb),
        // Explicit "this beat is the low half of a split 16-bit
        // pseudo-DMA aperture access" flag from the fabric — see
        // c96_dma16_hi_granted in rtl/mac/scsi.v.
        .pb_dma16_lo_beat (pb_scsi_dma16_lo_beat)
    );

    // ══════════════════════════════════════════════════════════════════
    // 53C96 register-access trace ring (debug observer)
    // ══════════════════════════════════════════════════════════════════
    // Snoops the SAME pb port u_scsi is attached to and never drives it.
    // Exists because reading the live 53C96 over JTAG is destructive (a
    // reg-2 read pops the FIFO, a reg-5 read clears the pending IRQ), so
    // the register-access history that leads into a wedge is otherwise
    // unobservable on hardware.  Readout is through u_vhdd_ctrl's
    // AXI-Lite block at VHDD_BASE+0x20..0x28.  See
    // rtl/soc/scsi_trace_ring.v for the entry format and for why the
    // read-side poll filter (not a watchdog) is what preserves the
    // pre-wedge window.
    //
    // Same reset as u_scsi (pb_full_rst_bank[2]) so the ring rewinds
    // exactly when the SCSI FSM it is recording does -- a ring that
    // survived a reset the recorded device did not would splice two
    // different boots into one trace.
    // ── BUILD GATE: `SCSI_TRACE_ENABLE  (env ENABLE_SCSI_TRACE, default 1)
    // ------------------------------------------------------------------
    // HISTORY, kept because it is the reason this is a GATE and not an
    // unconditional instantiation.  The ring was de-instantiated twice
    // (2026-08-08 and 2026-08-18) for routability: this design sits at
    // ~83% LUT / 64-of-64 URAM / congestion level 6, and at that
    // operating point a 33-LUT netlist delta was MEASURED to flip
    // route_design from clean to 346 residual node overlaps.  Removing
    // the ring frees LUTs, ~4 RAMB36 and routing resource; it does NOT
    // move URAM off 64/64 (every URAM comes from l2c_data.v's explicit
    // (* ram_style = "ultra" *) 8-way array, not from here).
    //
    // WHY IT IS BACK, AND ON BY DEFAULT (2026-09-09).  Storage is now a
    // first-class part of the project and System 7.5.3 hangs with the
    // Finder blocked forever on an async open-resource-fork whose result
    // field never leaves "in progress", while the disk side reports idle
    // and error-free and the CPU takes no faults.  Distinguishing "the
    // request never reached the controller" from "it completed and was
    // never reported back" REQUIRES the 53C96 register traffic, and that
    // traffic cannot be sampled live: a reg-2 read pops the FIFO and a
    // reg-5 read clears the pending IRQ, so JTAG polling destroys the
    // thing it measures.  The ring is the only non-destructive way to
    // see it.
    //
    // If a build fails to route with the ring in, build it out with
    // ENABLE_SCSI_TRACE=0 rather than editing this file; the `else arm
    // below restores exactly the previous tie-off, and the host tool
    // then reports the ring as absent (wr_ptr/wrapped/frozen all 0),
    // which tools/jtag_repl.tcl already calls out.
    //
    // The ring shares u_scsi's reset (pb_full_rst_bank[2]) so it rewinds
    // exactly when the SCSI FSM it records does -- a ring that survived a
    // reset the recorded device did not would splice two different boots
    // into one trace.  Readout runs in core_clk, the domain u_vhdd_ctrl
    // lives in, and is only trustworthy while rd_frozen reads 1 (see the
    // CDC note in rtl/soc/scsi_trace_ring.v).
`ifdef SCSI_TRACE_ENABLE
    scsi_trace_ring #(
        .DEPTH_LG2 (12)                  // 4096 entries; must match
                                         // VHDD_TRACE_DEPTH in
                                         // tools/jtag_repl.tcl and the
                                         // 12-bit trace_rd_addr/trace_wrptr
                                         // ports on vhdd_ctrl.
    ) u_scsi_trace_ring (
        // Capture: snoop-only taps on the SAME wires u_scsi is attached
        // to.  Nothing here is an output, so the ring cannot perturb the
        // bus it is recording.
        .pb_clk    (pb_clk),
        .pb_rst    (pb_full_rst_bank[2]),
        .pb_addr   (pb_scsi_addr ),
        .pb_wdata  (pb_scsi_wdata),
        .pb_wr     (pb_scsi_wr   ),
        .pb_rd     (pb_scsi_rd   ),
        .pb_rdata  (pb_scsi_rdata),
        .pb_ack    (pb_scsi_ack  ),
        // Readout: u_vhdd_ctrl (fpga_top_dma.vh), core_clk, xbar S2 at
        // VHDD_BASE+0x20..0x28.
        .rd_clk    (core_clk),
        .rd_rst    (soc_full_rst_bank[5]),
        .rd_freeze (scsi_trace_freeze),
        .rd_clear  (scsi_trace_clear),
        .rd_addr   (scsi_trace_rd_addr),
        .rd_data   (scsi_trace_rd_data),
        .rd_wrptr  (scsi_trace_wrptr),
        .rd_wrapped(scsi_trace_wrapped),
        .rd_frozen (scsi_trace_frozen)
    );
`else
    // Ring built out.  The vhdd_ctrl readout registers stay wired and
    // read as a permanently empty, never-frozen ring -- which is exactly
    // what `scsi-trace status` reports as "this bitstream has no trace
    // ring".
    assign scsi_trace_rd_data = 32'h0000_0000;
    assign scsi_trace_wrptr   = 12'h000;
    assign scsi_trace_wrapped = 1'b0;
    assign scsi_trace_frozen  = 1'b0;
`endif

    // SD-card provider for the virtual HDD.  Combinational: it re-codes
    // the request as an SD command class and biases the LBA past the
    // reserved boot window.  Everything below it (sd_scsi_bridge,
    // sd_ctrl, sd_spi) is unchanged and lives in fpga_top_sd.vh.
    // ── The provider router ───────────────────────────────────────────
    vhdd_mux u_vhdd_mux (
        .dev_sel          (scsi_vh_dev_sel_w),
        .m_num_lbas       (scsi_vh_num_lbas_w),
        .m_chk_lba        (scsi_vh_chk_lba_w),
        .m_chk_blocks     (scsi_vh_chk_blocks_w),
        .m_chk_ok         (scsi_vh_chk_ok_w),
        .m_req_write      (scsi_vh_req_write_w),
        .m_req_multi      (scsi_vh_req_multi_w),
        .m_req_lba        (scsi_vh_req_lba_w),
        .m_req_block_count(scsi_vh_req_block_count_w),
        .m_req_go         (scsi_vh_req_go_w),
        .m_busy           (scsi_vh_busy_w),
        .m_done           (scsi_vh_done_w),
        .m_error          (scsi_vh_error_w),
        .m_rd_valid       (scsi_vh_rd_valid_w),
        .m_rd_data        (scsi_vh_rd_data_w),
        .m_rd_ready       (scsi_vh_rd_ready_w),
        .m_wr_ready       (scsi_vh_wr_ready_w),
        .m_wr_valid       (scsi_vh_wr_valid_w),
        .m_wr_data        (scsi_vh_wr_data_w),
        .m_wr_avail       (scsi_vh_wr_avail_w),

        .a_num_lbas       (scsi_disk_num_lbas),
        .a_chk_lba        (vha_chk_lba_w),
        .a_chk_blocks     (vha_chk_blocks_w),
        .a_chk_ok         (vha_chk_ok_w),
        .a_req_write      (vha_req_write_w),
        .a_req_multi      (vha_req_multi_w),
        .a_req_lba        (vha_req_lba_w),
        .a_req_block_count(vha_req_block_count_w),
        .a_req_go         (vha_req_go_w),
        .a_busy           (vha_busy_w),
        .a_done           (vha_done_w),
        .a_error          (vha_error_w),
        .a_rd_valid       (vha_rd_valid_w),
        .a_rd_data        (vha_rd_data_w),
        .a_rd_ready       (vha_rd_ready_w),
        .a_wr_ready       (vha_wr_ready_w),
        .a_wr_valid       (vha_wr_valid_w),
        .a_wr_data        (vha_wr_data_w),
        .a_wr_avail       (vha_wr_avail_w),

        .b_num_lbas       (vhdd_rd_blocks_pb),
        .b_chk_lba        (vhb_chk_lba_w),
        .b_chk_blocks     (vhb_chk_blocks_w),
        .b_chk_ok         (vhb_chk_ok_w),
        .b_req_write      (vhb_req_write_w),
        .b_req_multi      (vhb_req_multi_w),
        .b_req_lba        (vhb_req_lba_w),
        .b_req_block_count(vhb_req_block_count_w),
        .b_req_go         (vhb_req_go_w),
        .b_busy           (vhb_busy_w),
        .b_done           (vhb_done_w),
        .b_error          (vhb_error_w),
        .b_rd_valid       (vhb_rd_valid_w),
        .b_rd_data        (vhb_rd_data_w),
        .b_rd_ready       (vhb_rd_ready_w),
        .b_wr_ready       (vhb_wr_ready_w),
        .b_wr_valid       (vhb_wr_valid_w),
        .b_wr_data        (vhb_wr_data_w),
        .b_wr_avail       (vhb_wr_avail_w)
    );

    // ── Read-ahead cache, between the mux's A port and the SD provider ─
    //
    // WHY IT IS HERE AND NOT INSIDE vhdd_sd.  The thing being amortised
    // is a PROPERTY OF THE PROVIDER (a per-command card latency), but the
    // amortisation itself is provider-agnostic: it is "turn N small
    // sequential requests into one big one", which is true of any backing
    // store with a fixed per-request cost.  Keeping it as its own
    // vhdd-to-vhdd shim leaves vhdd_sd combinational and single-purpose,
    // and lets the cache be built out with one parameter (ENABLE=0) for
    // an A/B without touching a wire.
    //
    // WHY ONLY ON PORT A.  Provider B is the Ethernet volume, whose cost
    // model is a network round trip, not a card command; it wants its own
    // policy, and the RAM budget here is spent where the boot actually
    // reads from.
    //
    // `inval` — the coherency backstop.  Everything on this list can
    // change the card, or the session with it, behind the cache's back,
    // and all four are exactly the ORs that reset the core side of
    // sd_scsi_bridge (see rtl/soc/fpga_top_sd.vh).  They are core_clk
    // levels held for many cycles, so a plain 2-FF ladder catches them;
    // they are also all multi-cycle by construction, so a missed edge is
    // not a thing that can happen.  (Content-wise a reset does not change
    // what is ON the card — the hazard being closed here is a session
    // that was torn down mid-run and a boot/provisioning path that may
    // have rewritten sectors while the SCSI side was held.)
    (* ASYNC_REG = "TRUE" *) reg [1:0] vhdd_ra_inval_meta;
    always @(posedge pb_clk) begin
        if (pb_full_rst_bank[2]) vhdd_ra_inval_meta <= 2'b11;
        else vhdd_ra_inval_meta <= {vhdd_ra_inval_meta[0],
                                    soc_full_rst_bank[5] | warm_peripheral_reset
                                                         | warm_storage_reset
                                                         | dbg_cold_reset_hold};
    end
    wire vhdd_ra_inval_pb = vhdd_ra_inval_meta[1];

    vhdd_readahead #(
        // 2 ways x 32 blocks = 32 KiB = 8 RAMB36.  See the module header
        // for the amortisation this buys and why two ways.
        .BLOCKS_PER_WAY(32),
        .ADAPTIVE_READAHEAD(1),
        .ENABLE        (1)
    ) u_vhdd_readahead (
        .clk              (pb_clk),
        .rst              (pb_full_rst_bank[2]),
        .inval            (vhdd_ra_inval_pb),

        .num_lbas         (scsi_disk_num_lbas),
        .chk_lba          (vha_chk_lba_w),
        .chk_blocks       (vha_chk_blocks_w),
        .chk_ok           (vha_chk_ok_w),
        .req_write        (vha_req_write_w),
        .req_multi        (vha_req_multi_w),
        .req_lba          (vha_req_lba_w),
        .req_block_count  (vha_req_block_count_w),
        .req_go           (vha_req_go_w),
        .busy             (vha_busy_w),
        .done             (vha_done_w),
        .error            (vha_error_w),
        .rd_valid         (vha_rd_valid_w),
        .rd_data          (vha_rd_data_w),
        .rd_ready         (vha_rd_ready_w),
        .wr_ready         (vha_wr_ready_w),
        .wr_valid         (vha_wr_valid_w),
        .wr_data          (vha_wr_data_w),
        .wr_avail         (vha_wr_avail_w),

        .p_chk_lba        (vhc_chk_lba_w),
        .p_chk_blocks     (vhc_chk_blocks_w),
        .p_chk_ok         (vhc_chk_ok_w),
        .p_req_write      (vhc_req_write_w),
        .p_req_multi      (vhc_req_multi_w),
        .p_req_lba        (vhc_req_lba_w),
        .p_req_block_count(vhc_req_block_count_w),
        .p_req_go         (vhc_req_go_w),
        .p_busy           (vhc_busy_w),
        .p_done           (vhc_done_w),
        .p_error          (vhc_error_w),
        .p_rd_valid       (vhc_rd_valid_w),
        .p_rd_data        (vhc_rd_data_w),
        .p_rd_ready       (vhc_rd_ready_w),
        .p_wr_ready       (vhc_wr_ready_w),
        .p_wr_valid       (vhc_wr_valid_w),
        .p_wr_data        (vhc_wr_data_w),
        .p_wr_avail       (vhc_wr_avail_w)
    );

    vhdd_sd #(
        .RESERVED_LBAS(SD_RESERVED_LBAS)
    ) u_vhdd_sd (
        .num_lbas       (scsi_disk_num_lbas),
        .chk_lba        (vhc_chk_lba_w),
        .chk_blocks     (vhc_chk_blocks_w),
        .chk_ok         (vhc_chk_ok_w),
        .req_write      (vhc_req_write_w),
        .req_multi      (vhc_req_multi_w),
        .req_lba        (vhc_req_lba_w),
        .req_block_count(vhc_req_block_count_w),
        .req_go         (vhc_req_go_w),
        .busy           (vhc_busy_w),
        .done           (vhc_done_w),
        .error          (vhc_error_w),
        .rd_valid       (vhc_rd_valid_w),
        .rd_data        (vhc_rd_data_w),
        .rd_ready       (vhc_rd_ready_w),
        .wr_ready       (vhc_wr_ready_w),
        .wr_valid       (vhc_wr_valid_w),
        .wr_data        (vhc_wr_data_w),
        .wr_avail       (vhc_wr_avail_w),
        .sd_cmd_type    (scsi_sd_cmd_type_w),
        .sd_lba         (scsi_sd_lba_w),
        .sd_block_count (scsi_sd_block_count_w),
        .sd_go          (scsi_sd_go_w),
        .sd_busy        (scsi_sd_busy_w),
        .sd_done        (scsi_sd_done_w),
        .sd_error       (scsi_sd_error_w),
        .sd_rd_valid    (scsi_sd_rd_valid_w),
        .sd_rd_data     (scsi_sd_rd_data_w),
        .sd_rd_ready    (scsi_sd_rd_ready_w),
        .sd_wr_ready    (scsi_sd_wr_ready_w),
        .sd_wr_valid    (scsi_sd_wr_valid_w),
        .sd_wr_data     (scsi_sd_wr_data_w),
        .sd_wr_avail    (scsi_sd_wr_avail_w)
    );

    // ═══════════════════════════════════════════════════════════════════
    // Provider B (SCSI ID 1) — vacant
    // ═══════════════════════════════════════════════════════════════════
    // The DDR-backed RAM disk (vhdd_ddr) that occupied this slot was
    // REMOVED on 2026-09-10 (owner directive: "vhdd ddr should be
    // killed").  Removed with it: rtl/soc/vhdd_ddr.v, its unit tb, its
    // pb_clk->core_clk AXI bridge (u_vhdd_ddr_cdc), the 0x7000_0000
    // aperture and its xbar decode/flatten arms, and the l2c bypass
    // window that carved the flattened range out of the cacheable span.
    //
    // xbar master seat M3 is now unconditionally the shared DMA engine's
    // (fpga_top_dma.vh) rather than being shared with this provider under
    // a compile-time switch.
    //
    // The vhdd seam itself is untouched: u_vhdd_mux still has a B port,
    // and scsi.v still answers two target IDs.  ENABLE_NET_VHDD builds
    // the Ethernet-backed volume into this slot (below).  With neither,
    // vhdd_dev_en_pb masks bit 1 low, so scsi.v never accepts a selection
    // of ID 1 and the unconnected B port is unreachable by construction.

`ifdef ENABLE_NET_VHDD
    // ── PROVIDER B: THE ETHERNET-BACKED VOLUME (SCSI ID 1) ────────────
    // Takes the slot vhdd_ddr vacated, so vhdd_mux and scsi.v are untouched:
    // the master already answers to two target IDs and dev_sel is already the
    // one latched bit that selects between them.  See docs/net_vhdd_design.md.
    //
    // It is a cheaper tenant than the RAM disk was.  That slot was freed for
    // ~1.5k LUT and, worth more, xbar master seat M3 on a read fan-in that was
    // 4/4 full.  This provider reaches its backing store over Ethernet, not
    // AXI, so it takes back neither the seat nor the L2C bypass window.
    //
    // Volume size reuses the RD_BLOCKS CSR (0x008) that sized the RAM disk --
    // already JTAG-settable and already synchronised to pb_clk.
    //
    // Endpoint config crosses core_clk -> pb_clk as quasi-static state: it is
    // written once over JTAG before the volume is used, and `vhdd-net` writes
    // our-MAC LAST, so the client stays absent until the rest has landed.  A
    // torn intermediate value is therefore never sampled by a live transfer.
    (* ASYNC_REG = "TRUE" *) reg [175:0] net_cfg_meta, net_cfg_sync;
    always @(posedge pb_clk) begin
        net_cfg_meta <= {net_vhdd_our_mac, net_vhdd_our_ip, net_vhdd_our_port,
                         net_vhdd_dst_mac, net_vhdd_dst_ip, net_vhdd_dst_port};
        net_cfg_sync <= net_cfg_meta;
    end
    wire [47:0] net_cfg_our_mac  = net_cfg_sync[175:128];
    wire [31:0] net_cfg_our_ip   = net_cfg_sync[127:96];
    wire [15:0] net_cfg_our_port = net_cfg_sync[95:80];
    wire [47:0] net_cfg_dst_mac  = net_cfg_sync[79:32];
    wire [31:0] net_cfg_dst_ip   = net_cfg_sync[31:0] >> 0;
    wire [15:0] net_cfg_dst_port = net_cfg_sync[15:0];

    wire        nvh_req_valid, nvh_req_ready;
    wire [7:0]  nvh_req_op;
    wire [31:0] nvh_req_lba;
    wire [15:0] nvh_req_block_count, nvh_req_tag;
    wire [7:0]  nvh_req_pay_tdata;
    wire        nvh_req_pay_tvalid, nvh_req_pay_tready, nvh_req_pay_tlast;
    wire        nvh_reply_valid, nvh_reply_ready;
    wire [15:0] nvh_reply_tag;
    wire [7:0]  nvh_reply_status;
    wire [7:0]  nvh_reply_pay_tdata;
    wire        nvh_reply_pay_tvalid, nvh_reply_pay_tready, nvh_reply_pay_tlast;

    net_block_framer u_net_block_framer (
        .clk(pb_clk), .rst(pb_full_rst_bank[2]),
        .our_mac(net_cfg_our_mac), .our_ip(net_cfg_our_ip),
        .our_port(net_cfg_our_port),
        .dst_mac(net_cfg_dst_mac), .dst_ip(net_cfg_dst_ip),
        .dst_port(net_cfg_dst_port),
        .req_valid(nvh_req_valid), .req_ready(nvh_req_ready),
        .req_op(nvh_req_op), .req_lba(nvh_req_lba),
        .req_block_count(nvh_req_block_count), .req_tag(nvh_req_tag),
        .req_payload_tdata(nvh_req_pay_tdata),
        .req_payload_tvalid(nvh_req_pay_tvalid),
        .req_payload_tready(nvh_req_pay_tready),
        .req_payload_tlast(nvh_req_pay_tlast),
        .tx_tdata(net_blk_tx_tdata), .tx_tvalid(net_blk_tx_tvalid),
        .tx_tready(net_blk_tx_tready), .tx_tlast(net_blk_tx_tlast),
        .rx_tdata(net_blk_rx_tdata), .rx_tvalid(net_blk_rx_tvalid),
        .rx_tready(net_blk_rx_tready), .rx_tlast(net_blk_rx_tlast),
        .reply_valid(nvh_reply_valid), .reply_ready(nvh_reply_ready),
        .reply_tag(nvh_reply_tag), .reply_status(nvh_reply_status),
        .reply_payload_tdata(nvh_reply_pay_tdata),
        .reply_payload_tvalid(nvh_reply_pay_tvalid),
        .reply_payload_tready(nvh_reply_pay_tready),
        .reply_payload_tlast(nvh_reply_pay_tlast)
    );

    // TIMEOUTS ARE TUNED FOR PACKET LOSS, NOT FOR DRIVER PATIENCE.
    //
    // Stalling before the data phase is safe here, and not by luck: this bus
    // serviced period SCSI disks with 15-30 ms seeks plus ~8 ms of average
    // rotational latency at 3600 RPM, so the Mac's driver was built to sit
    // through tens of milliseconds.  A sub-millisecond LAN round trip is far
    // inside what it already tolerates, and vhdd_sd has stalled it the same
    // way for every CMD17 since day one.  So disconnect/reselect is NOT
    // needed to cover the cold start.
    //
    // What the retransmit timeout actually decides is how long a LOST packet
    // costs.  The module default is 40 ms at this clock, which is ~200x a LAN
    // round trip: correct, but it turns one dropped frame into a 40 ms stall
    // when 4 ms would do.  4 ms is still ~20x RTT, so it cannot retransmit
    // over a merely slow reply, and the whole retry budget (4 attempts) then
    // costs ~16 ms worst case -- comfortably inside one legacy seek.
    //
    // Expressed against PB_CLK_HZ rather than as raw cycles so that a clock
    // change moves the TIME, which is the thing that was reasoned about.
    vhdd_net #(
        .REPLY_TIMEOUT(PB_CLK_HZ / 250),     // 4 ms
        .MAX_RETRIES  (3),
        .STALL_TIMEOUT(PB_CLK_HZ * 4)        // 4 s: the abandoned-master net
    ) u_vhdd_net (
        // pb-bank [2] — same reset domain as u_scsi and as the link's
        // blk_clk face, so a debug full reset rewinds target, provider and
        // transport together.
        .clk(pb_clk), .rst(pb_full_rst_bank[2]),
        .num_lbas       (vhdd_rd_blocks_pb),
        .chk_lba        (vhb_chk_lba_w),
        .chk_blocks     (vhb_chk_blocks_w),
        .chk_ok         (vhb_chk_ok_w),
        .req_write      (vhb_req_write_w),
        .req_multi      (vhb_req_multi_w),
        .req_lba        (vhb_req_lba_w),
        .req_block_count(vhb_req_block_count_w),
        .req_go         (vhb_req_go_w),
        .busy           (vhb_busy_w),
        .done           (vhb_done_w),
        .error          (vhb_error_w),
        .rd_valid       (vhb_rd_valid_w),
        .rd_data        (vhb_rd_data_w),
        .rd_ready       (vhb_rd_ready_w),
        .wr_ready       (vhb_wr_ready_w),
        .wr_valid       (vhb_wr_valid_w),
        .wr_data        (vhb_wr_data_w),
        .wr_avail       (vhb_wr_avail_w),
        .net_req_valid  (nvh_req_valid), .net_req_ready(nvh_req_ready),
        .net_req_op     (nvh_req_op), .net_req_lba(nvh_req_lba),
        .net_req_block_count(nvh_req_block_count), .net_req_tag(nvh_req_tag),
        .net_req_payload_tdata (nvh_req_pay_tdata),
        .net_req_payload_tvalid(nvh_req_pay_tvalid),
        .net_req_payload_tready(nvh_req_pay_tready),
        .net_req_payload_tlast (nvh_req_pay_tlast),
        .net_reply_valid(nvh_reply_valid), .net_reply_ready(nvh_reply_ready),
        .net_reply_tag(nvh_reply_tag), .net_reply_status(nvh_reply_status),
        .net_reply_payload_tdata (nvh_reply_pay_tdata),
        .net_reply_payload_tvalid(nvh_reply_pay_tvalid),
        .net_reply_payload_tready(nvh_reply_pay_tready),
        .net_reply_payload_tlast (nvh_reply_pay_tlast)
    );
`endif // ENABLE_NET_VHDD

    // Task #122: SONORA ASC.  Shares the 1 MHz phi2 timebase with VIA1
    // for the FIFO sample-rate divider.
    //
    // Task #123 (b): audio_sample_out is now routed to either an I2S
    // serialiser (external-codec path) or the HDMI data-island bridge
    // (HDMI-audio path) per the AUDIO_PATH parameter:
    //   0 = tied off (bring-up default until a codec lands on the board)
    //   1 = I2S external codec
    //   2 = HDMI audio data-island
    //
    // Both bridges consume {audio_sample_out, audio_sample_valid} from
    // the ASC on pb_clk.  The I2S variant drives three dedicated FPGA
    // pins (future XDC addition); the HDMI variant feeds a handshake
    // port on video_top's audio data-island scheduler (not yet instanced
    // — the bridge's outputs are left dangling with a pin-connect-empty
    // lint suppression until the HDMI controller exposes the port).
    /* verilator lint_off PINCONNECTEMPTY */
    /* verilator lint_off UNUSEDSIGNAL */
    wire [15:0] asc_audio_sample_out_w;
    wire [15:0] asc_audio_pcm_l_w;
    wire [15:0] asc_audio_pcm_r_w;
    wire        asc_audio_sample_valid_w;
    /* verilator lint_on UNUSEDSIGNAL */
    asc u_asc (
        // pb_full_rst — ASC FIFO + sample-rate divider rewind on
        // debug-full-reset; pending half-empty IRQ clears.  pb-bank [3].
        .clk(pb_clk), .rst(pb_full_rst_bank[3]),
        .phi2_tick(phi2_tick),
        .pb_addr (pb_asc_addr ),
        .pb_wdata(pb_asc_wdata),
        .pb_wr   (pb_asc_wr   ),
        .pb_rd   (pb_asc_rd   ),
        .pb_rdata(pb_asc_rdata),
        .pb_ack  (pb_asc_ack  ),
        .audio_sample_out  (asc_audio_sample_out_w),
        .audio_pcm_l       (asc_audio_pcm_l_w),
        .audio_pcm_r       (asc_audio_pcm_r_w),
        .audio_sample_valid(asc_audio_sample_valid_w),
        .irq     (asc_irq_pb  )
    );

    /* verilator lint_off UNUSEDSIGNAL */
    wire       iwm_dma_req_w;
    wire [7:0] iwm_mode_w;
    /* verilator lint_on UNUSEDSIGNAL */
    iwm_stub u_iwm (
        .clk          (pb_clk),
        // pb_full_rst — IWM stub register state cycles on debug-full-reset.
        // pb-bank [3].
        .rst          (pb_full_rst_bank[3]),
        .cs           (pb_iwm_wr | pb_iwm_rd),
        .rd           (pb_iwm_rd),
        .wr           (pb_iwm_wr),
        .reg_sel      (pb_iwm_addr),
        .wdata        (pb_iwm_wdata),
        // drive_present=1 selects the MAME-faithful "drive attached,
        // no media" SWIM handshake (0x0c).  See rtl/mac/iwm_stub.v
        // header for the full rationale; this lets the Q700 ROM enter
        // the floppy-poll loop instead of bailing to MacsBug.
        .drive_present(1'b1),
        .rdata        (pb_iwm_rdata),
        .irq          (iwm_irq_w),
        .dma_req      (iwm_dma_req_w),
        .mode_o       (iwm_mode_w)
    );
    assign pb_iwm_ack = pb_iwm_wr | pb_iwm_rd;

    // ── Audio output path selector (Task #123) ───────────────────────
    // Defparam lives here so a board-specific top wrapper (or a
    // `synth_design -generic AUDIO_PATH=N` invocation) can pick the
    // active path without touching this file.  0 = OFF, 1 = I2S, 2 =
    // HDMI, 3 = PWM (Σ-Δ on AN9134 NC pins J1.35/36).  The inactive
    // paths are optimised away by synthesis because they only drive
    // dangling / tied-off outputs.
    localparam integer AUDIO_PATH_OFF  = 0;
    localparam integer AUDIO_PATH_I2S  = 1;
    localparam integer AUDIO_PATH_HDMI = 2;
    localparam integer AUDIO_PATH_PWM  = 3;
    // For this landing we default to PWM — the active board is the
    // ALINX AN9134 carrier, whose 40-pin connector exposes only J1.35
    // and J1.36 as NC.  PWM is the only audio path that fits inside
    // that pin budget.  See docs/superpowers/specs/2026-05-04-an9134-
    // pwm-audio-design.md for the full rationale and the off-board
    // class-D speaker amp it drives.  I2S and HDMI bridges remain in
    // the tree, tied off, for future board revisions.
    localparam integer AUDIO_PATH = AUDIO_PATH_PWM;

    // I2S outputs (tied off unless AUDIO_PATH=I2S is selected).  Dangling
    // until the XDC exposes the three codec pins; a follow-up task adds
    // the pin constraints.
    wire i2s_bclk_w, i2s_lrclk_w, i2s_data_w;
    // HDMI audio sub-frame outputs.  Fed into video_top's data-island
    // scheduler when that port exists — today it's dangling.
    wire [27:0] hdmi_audio_sf_l_w, hdmi_audio_sf_r_w;
    wire        hdmi_audio_frame_valid_w;
    // Σ-Δ PWM outputs (one bit per channel).  Routed to the top-level
    // pwm_audio_l/r ports below; synth/audio_pwm.xdc pins them to
    // FPGA H13 (AN9134 J1.35) and J13 (AN9134 J1.36) — the only two
    // unconstrained pins on the carrier 40-pin connector.
    wire pwm_l_w, pwm_r_w;

    generate
        if (AUDIO_PATH == AUDIO_PATH_I2S) begin : gen_audio_i2s
            audio_i2s #(
                .CORE_FREQ_HZ    (PB_CLK_HZ),
                .I2S_BCLK_FREQ_HZ(  3_072_000),
                .BITS_PER_SAMPLE (16),
                .SLOT_BITS       (32)
            ) u_audio_i2s (
                .clk        (pb_clk),
                // pb_full_rst — I2S serializer state matches ASC reset
                // so a debug-full-reset doesn't dribble out the
                // mid-sample residue from before the warm reset.
                // pb-bank [3].
                .rst        (pb_full_rst_bank[3]),
                .sample_in  (asc_audio_sample_out_w),
                .sample_valid(asc_audio_sample_valid_w),
                .i2s_bclk   (i2s_bclk_w ),
                .i2s_lrclk  (i2s_lrclk_w),
                .i2s_data   (i2s_data_w )
            );
            // HDMI bridge tied off in this variant.
            assign hdmi_audio_sf_l_w        = 28'h0;
            assign hdmi_audio_sf_r_w        = 28'h0;
            assign hdmi_audio_frame_valid_w = 1'b0;
            // PWM tied off in this variant.
            assign pwm_l_w = 1'b0;
            assign pwm_r_w = 1'b0;
        end
        else if (AUDIO_PATH == AUDIO_PATH_HDMI) begin : gen_audio_hdmi
            audio_hdmi_bridge u_audio_hdmi (
                .clk          (pb_clk),
                // pb_full_rst — alternative path to the I2S serializer.
                // pb-bank [3].
                .rst          (pb_full_rst_bank[3]),
                .sample_in    (asc_audio_sample_out_w),
                .sample_valid (asc_audio_sample_valid_w),
                .subframe_l   (hdmi_audio_sf_l_w),
                .subframe_r   (hdmi_audio_sf_r_w),
                .frame_valid  (hdmi_audio_frame_valid_w),
                // Accept as fast as the bridge can offer — when the real
                // HDMI data-island scheduler exists its ready signal
                // replaces this tie-high.
                .frame_ready  (1'b1)
            );
            // I2S tied off in this variant.
            assign i2s_bclk_w  = 1'b0;
            assign i2s_lrclk_w = 1'b0;
            assign i2s_data_w  = 1'b0;
            // PWM tied off in this variant.
            assign pwm_l_w = 1'b0;
            assign pwm_r_w = 1'b0;
        end
        else if (AUDIO_PATH == AUDIO_PATH_PWM) begin : gen_audio_pwm
            audio_pwm #(
                .CORE_FREQ_HZ (PB_CLK_HZ)
            ) u_audio_pwm (
                .clk         (pb_clk),
                // pb_full_rst — Σ-Δ accumulator state matches ASC reset
                // so a debug-full-reset returns the modulators to the
                // canonical 50% silence duty.  pb-bank [3].
                .rst         (pb_full_rst_bank[3]),
                .sample_l    (asc_audio_pcm_l_w),
                .sample_r    (asc_audio_pcm_r_w),
                .sample_valid(asc_audio_sample_valid_w),
                .pwm_l       (pwm_l_w),
                .pwm_r       (pwm_r_w)
            );
            // I2S and HDMI tied off in this variant.
            assign i2s_bclk_w               = 1'b0;
            assign i2s_lrclk_w              = 1'b0;
            assign i2s_data_w               = 1'b0;
            assign hdmi_audio_sf_l_w        = 28'h0;
            assign hdmi_audio_sf_r_w        = 28'h0;
            assign hdmi_audio_frame_valid_w = 1'b0;
        end
        else begin : gen_audio_off
            assign i2s_bclk_w               = 1'b0;
            assign i2s_lrclk_w              = 1'b0;
            assign i2s_data_w               = 1'b0;
            assign hdmi_audio_sf_l_w        = 28'h0;
            assign hdmi_audio_sf_r_w        = 28'h0;
            assign hdmi_audio_frame_valid_w = 1'b0;
            assign pwm_l_w                  = 1'b0;
            assign pwm_r_w                  = 1'b0;
        end
    endgenerate

    // PWM channels routed up to the AN9134 NC pins through the top-level
    // pwm_audio_l/r ports.  Pin assignments live in synth/audio_pwm.xdc.
    assign pwm_audio_l = pwm_l_w;
    assign pwm_audio_r = pwm_r_w;

    // Keep the audio-path wires from being pruned by synth.  PWM has
    // real top-level pins (pwm_audio_l/r → H13/J13) so the modulator
    // outputs are claimed by XDC and won't disappear today, but include
    // them in the tap as belt-and-suspenders against future configs
    // that might gate the top-level ports off.  I2S triplet and HDMI
    // subframes are still dangling until their consumers exist.
    /* verilator lint_off UNUSEDSIGNAL */
    wire audio_alive_tap = i2s_bclk_w ^ i2s_lrclk_w ^ i2s_data_w
                         ^ hdmi_audio_frame_valid_w
                         ^ (|hdmi_audio_sf_l_w) ^ (|hdmi_audio_sf_r_w)
                         ^ pwm_l_w ^ pwm_r_w;
    /* verilator lint_on UNUSEDSIGNAL */
    /* verilator lint_on PINCONNECTEMPTY */

    // ═══════════════════════════════════════════════════════════════════
    // IRQ aggregator — peripheral IRQ lines → cpu_ipl[2:0]
    // ═══════════════════════════════════════════════════════════════════
    // Level assignments per docs/exceptions.md §"Asynchronous exceptions":
    //   level 1 → VIA1  (60 Hz VBL, ADB, RTC, sound command)
    //   level 2 → VIA2  (NuBus slot IRQ aggregate / board-status path,
    //                     including Q700 SCSI CB2 and ASC CB1 service)
    //   level 3 → unused here; Q700 SCSI reaches the CPU through VIA2
    //   level 4 → SCC   (serial)
    //   level 5 → unused here; Q700 ASC reaches the CPU through VIA2
    //   level 6 → DMA controller (dma_ctrl.v) — done + error level IRQ
    //   level 7 → NMI  (board btn[1] via nmi_btn_pulse, sys_clk-synced
    //                    in fpga_top_clocks.vh; resync to core_clk here)
    //
    // ipl_ack: routed from commit.v's 1-cycle IRQ-dispatch pulse via
    // m68k_core's cpu_ipl_ack output (see fpga_top_cpu.vh).  Without
    // this hook the NMI rising-edge latch in irq_agg.v never clears,
    // so any spurious edge (board boot, btn glitch) wedges IPL=7
    // forever and the CPU loops on vec-31.
    // The core also runs its own internal irq_agg with inputs tied to 0;
    // cpu_ipl_ext is max'd with the internal agg in m68k_core.v so this
    // external path is what actually drives IRQ servicing this round.
    //
    // nmi_btn_pulse arrives in the sys_clk domain (synced from btn[1] in
    // fpga_top_clocks.vh).  Re-synchronise into core_clk before handing
    // it to irq_agg, which expects a single-clock-domain edge input.
    (* ASYNC_REG = "TRUE" *) reg nmi_btn_meta, nmi_btn_core;
    always @(posedge core_clk) begin
        // soc_full_rst — match irq_agg below so the NMI edge sync drops
        // any in-flight pulse on debug-full-reset.  Bank bit [3]
        // (irq_agg / NMI region).
        if (soc_full_rst_bank[3] || warm_peripheral_reset) begin
            nmi_btn_meta <= 1'b0;
            nmi_btn_core <= 1'b0;
        end else begin
            nmi_btn_meta <= nmi_btn_pulse;
            nmi_btn_core <= nmi_btn_meta;
        end
    end
    irq_agg u_irq_agg (
        // soc_full_rst — IPL latches and NMI edge clear on debug-full-
        // reset so the post-reset CPU comes up at IPL=0 cleanly.  Bank
        // bit [3] (irq_agg / NMI region).
        .clk(core_clk), .rst(soc_full_rst_bank[3] | warm_peripheral_reset),
        .via1_irq  (via1_irq_w),
        .via2_irq  (via2_irq_w),
        .scsi_irq  (1'b0),
        .scc_irq   (scc_irq_w ),
        .snd_irq   (1'b0 ),
        .rsvd_irq6 (dma_irq_w),   // DMA completion / error — task #116
        .nmi_edge  (nmi_btn_core),
        .ipl_ack   (cpu_ipl_ack_w),
        .ipl       (cpu_ipl_periph_w)
    );

    // Keep VIA1's overlay state observable on the LEDs.  The live CPU path
    // now consumes the reset overlay inside the xbar decoder, while host
    // debug masters still see raw physical addresses.
    /* verilator lint_off UNUSEDSIGNAL */
    wire via1_overlay_dbg = via1_overlay_bit;
    /* verilator lint_on UNUSEDSIGNAL */
