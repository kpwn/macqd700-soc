// rtl/fpga_top_sd.vh — included from rtl/fpga_top.v
//
// SD SPI bus: boot_fsm phase-A owns the bus during ROM load; once
// boot_rom_loaded asserts the mux switches to phase-B and SCSI claims
// the SPI master through its own sd_ctrl instance.  sd_provision was
// removed (host pre-flashes the SD card; CPU boots via JTAG-AXI) so the
// phase-B(0) prov_* port group of the mux is tied off; phase-B(1) is
// the SCSI path.
//
// Topology
// ════════
//   scsi.v (pb_clk, 50 MHz) ──┐
//                              │  sd_scsi_bridge.v   (CDC, this file)
//   sd_ctrl_scsi (core_clk) ──┘──┐
//                                  │  sd_spi_mux  (combinational select)
//   boot_fsm + its sd_ctrl ───────┘──┐
//                                       │  sd_spi (core_clk)
//                                       └──→ physical pins
//
// All sd_* logic on the SCSI side runs on core_clk so it shares the
// domain with sd_spi (which lives on core_clk to hit 50 MHz SPI in HS
// mode).  The pb_clk/core_clk crossing is contained inside one module
// (sd_scsi_bridge) — see its header for the CDC primitives used.
//
// Generated mechanically from rtl/fpga_top.v during the fpga_top split
// (agent/p3-fpga-top-split). This file contains the original lines
// verbatim plus the SCSI hookup, inside the fpga_top module scope via
// `include.  Do not add module/endmodule wrappers or nettype directives.

    // ═══════════════════════════════════════════════════════════════════
    // SD SPI mux + sd_spi — phase A = boot, phase B = SCSI
    // ═══════════════════════════════════════════════════════════════════
    wire        spi_cmd_valid;
    wire        spi_cmd_ready;
    wire [7:0]  spi_cmd_data;
    wire        spi_rsp_valid;
    wire [7:0]  spi_rsp_data;
    wire        spi_cs_n_wire;
    wire        spi_fast_mode;
    wire        spi_hs_mode;

    // Phase-B(0) wires: sd_jtag_writer ↔ sd_spi_mux prov_* port.
    wire        prov_spi_cmd_valid;
    wire        prov_spi_cmd_ready;
    wire [7:0]  prov_spi_cmd_data;
    wire        prov_spi_rsp_valid;
    wire [7:0]  prov_spi_rsp_data;
    wire        prov_spi_cs_n_w;
    wire        prov_spi_fast_w;
    wire        prov_spi_hs_w;
    wire        sdj_writer_busy;

    // Phase-B(2) wires: pram_sd ↔ sd_spi_mux pram_* port.
    wire        pram_spi_cmd_valid;
    wire        pram_spi_cmd_ready;
    wire [7:0]  pram_spi_cmd_data;
    wire        pram_spi_rsp_valid;
    wire [7:0]  pram_spi_rsp_data;
    wire        pram_spi_cs_n_w;
    wire        pram_spi_fast_w;
    wire        pram_spi_hs_w;
    wire        pram_sd_req;
    reg         pram_gnt_q;          // driven below, once core_scsi_busy exists
    // Declared before the mux because an in-flight write retains phase-B
    // ownership while a SoC reset is being closed safely.
    wire        core_scsi_busy;
    wire        pram_sd_busy_w;
    // Any active phase-B owner must retain the mux and shared byte engine
    // until it has released the card.  In particular, pram_sd's CMD24 is
    // independent of the SCSI controller and cannot be protected by
    // core_scsi_busy alone.
    wire        storage_owner_busy = core_scsi_busy | pram_sd_busy_w |
                                     sdj_writer_busy;

    peripheral_reset_sequencer #(
        .PULSE_CYCLES(128)
    ) u_peripheral_reset_sequencer (
        .clk              (core_clk),
        .rst              (soc_full_rst_bank[5]),
        .reset_req        (cpu_peripheral_reset),
        .storage_busy     (storage_owner_busy),
        .storage_reset_req(warm_storage_reset),
        .peripheral_reset (warm_peripheral_reset)
    );

    // Phase-B(1) wires: sd_ctrl_scsi ↔ sd_spi_mux scsi_* port.
    wire        scsi_spi_cmd_valid;
    wire        scsi_spi_cmd_ready;
    wire [7:0]  scsi_spi_cmd_data;
    wire        scsi_spi_rsp_valid;
    wire [7:0]  scsi_spi_rsp_data;
    wire        scsi_spi_cs_n_w;

    sd_spi_mux u_spi_mux (
        .boot_done     (boot_rom_loaded | storage_owner_busy),
        // b_sel normally routes phase-B to SCSI.  sd_spi_mux overrides
        // this while the provision/JTAG writer asserts CS or a command.
        .b_sel         (1'b1           ),

        .boot_cmd_valid(boot_spi_cmd_valid),
        .boot_cmd_ready(boot_spi_cmd_ready),
        .boot_cmd_data (boot_spi_cmd_data ),
        .boot_rsp_valid(boot_spi_rsp_valid),
        .boot_rsp_data (boot_spi_rsp_data ),
        .boot_cs_n_in  (boot_spi_cs_n     ),
        .boot_fast_mode(boot_spi_fast     ),
        .boot_hs_mode  (boot_spi_hs       ),

        // Phase-B(0) — JTAG SD-card writer.
        .prov_cmd_valid(prov_spi_cmd_valid),
        .prov_cmd_ready(prov_spi_cmd_ready),
        .prov_cmd_data (prov_spi_cmd_data ),
        .prov_rsp_valid(prov_spi_rsp_valid),
        .prov_rsp_data (prov_spi_rsp_data ),
        .prov_cs_n_in  (prov_spi_cs_n_w   ),
        .prov_fast_mode(prov_spi_fast_w   ),
        .prov_hs_mode  (prov_spi_hs_w     ),

        // Phase-B(2) — PRAM persistence sector (SD LBA 8191).  Unlike the
        // provision port this one does NOT preempt: pram_gnt_q below only
        // rises on a cycle where sd_ctrl_scsi is idle.
        .pram_gnt      (pram_gnt_q        ),
        .pram_cmd_valid(pram_spi_cmd_valid),
        .pram_cmd_ready(pram_spi_cmd_ready),
        .pram_cmd_data (pram_spi_cmd_data ),
        .pram_rsp_valid(pram_spi_rsp_valid),
        .pram_rsp_data (pram_spi_rsp_data ),
        .pram_cs_n_in  (pram_spi_cs_n_w   ),
        .pram_fast_mode(pram_spi_fast_w   ),
        .pram_hs_mode  (pram_spi_hs_w     ),

        // Phase-B(1) — SCSI sd_ctrl owns the SPI bus once boot is done.
        .scsi_cmd_valid(scsi_spi_cmd_valid),
        .scsi_cmd_ready(scsi_spi_cmd_ready),
        .scsi_cmd_data (scsi_spi_cmd_data ),
        .scsi_rsp_valid(scsi_spi_rsp_valid),
        .scsi_rsp_data (scsi_spi_rsp_data ),
        .scsi_cs_n_in  (scsi_spi_cs_n_w   ),
        // SCSI path runs at the high-speed SPI rate the card was
        // switched to during boot init (CMD6).  fast_mode + hs_mode both 1
        // -> HS_HALF divider (sd_spi.v default HS_HALF=2, FAST_HALF=4 —
        // hardware-validated ratios, NOT an absolute-frequency pin).  SCK
        // is core_clk/(2*HALF), so it scales with whatever core_clk the
        // build actually runs: 50 MHz HS / 25 MHz fast-non-HS at the
        // 200 MHz core_clk these ratios were validated against, or
        // 25 MHz HS / 12.5 MHz fast-non-HS at the canonical 100 MHz
        // core_clk build.  Do not repoint FAST_HALF/HS_HALF at an
        // absolute target frequency — that would silently halve the
        // validated SCK rate the moment core_clk goes back to 200 MHz.
        .scsi_fast_mode(1'b1),
        .scsi_hs_mode  (1'b1),

        .spi_cmd_valid(spi_cmd_valid),
        .spi_cmd_ready(spi_cmd_ready),
        .spi_cmd_data (spi_cmd_data ),
        .spi_rsp_valid(spi_rsp_valid),
        .spi_rsp_data (spi_rsp_data ),
        .spi_cs_n_in  (spi_cs_n_wire ),
        .spi_fast_mode(spi_fast_mode),
        .spi_hs_mode  (spi_hs_mode  )
    );

    // ═══════════════════════════════════════════════════════════════════
    // Split the AXI_SD_JTAG_BASE window: [8]=0 → sd_jtag_writer,
    //                                    [8]=1 → pram_sd
    // ═══════════════════════════════════════════════════════════════════
    // Adding a seventh xbar slave would mean touching axi_xbar.v's
    // flattened s0..s5 port arrays in a dozen places; the 1 MiB SD-JTAG
    // window already holds a single 32-byte register file, so pram_sd
    // gets the next 256-byte block behind the same slave port instead.
    //   0x50A0_0000  sd_jtag_writer (unchanged — existing `sd-write` Tcl
    //                keeps working byte-for-byte)
    //   0x50A0_0100  pram_sd        (`pram-save` / `pram-load` / `pram-dump`)
    // The split lives OUTSIDE the `DISABLE_SD_JTAG_WRITER `ifdef on
    // purpose: PRAM persistence is a runtime feature and must be present
    // in production bitstreams, where the ~17.5K-LUT SD-JTAG writer is not.
    wire [19:0]  sdw_awaddr;
    wire         sdw_awvalid;
    wire         sdw_awready;
    wire [31:0]  sdw_wdata;
    wire [3:0]   sdw_wstrb;
    wire         sdw_wvalid;
    wire         sdw_wready;
    wire [1:0]   sdw_bresp;
    wire         sdw_bvalid;
    wire         sdw_bready;
    wire [19:0]  sdw_araddr;
    wire         sdw_arvalid;
    wire         sdw_arready;
    wire [31:0]  sdw_rdata;
    wire [1:0]   sdw_rresp;
    wire         sdw_rvalid;
    wire         sdw_rready;

    wire [19:0]  prm_awaddr;
    wire         prm_awvalid;
    wire         prm_awready;
    wire [31:0]  prm_wdata;
    wire [3:0]   prm_wstrb;
    wire         prm_wvalid;
    wire         prm_wready;
    wire [1:0]   prm_bresp;
    wire         prm_bvalid;
    wire         prm_bready;
    wire [19:0]  prm_araddr;
    wire         prm_arvalid;
    wire         prm_arready;
    wire [31:0]  prm_rdata;
    wire [1:0]   prm_rresp;
    wire         prm_rvalid;
    wire         prm_rready;

    axil_split2 #(
        .ADDR_W (20),
        .SEL_BIT(8)
    ) u_sdj_split (
        .clk       (core_clk),
        .rst       (soc_full_rst_bank[5]),
        .u_awaddr  (sdj_awaddr ), .u_awvalid(sdj_awvalid), .u_awready(sdj_awready),
        .u_wdata   (sdj_wdata  ), .u_wstrb  (sdj_wstrb  ),
        .u_wvalid  (sdj_wvalid ), .u_wready (sdj_wready ),
        .u_bresp   (sdj_bresp  ), .u_bvalid (sdj_bvalid ), .u_bready (sdj_bready ),
        .u_araddr  (sdj_araddr ), .u_arvalid(sdj_arvalid), .u_arready(sdj_arready),
        .u_rdata   (sdj_rdata  ), .u_rresp  (sdj_rresp  ),
        .u_rvalid  (sdj_rvalid ), .u_rready (sdj_rready ),

        .d0_awaddr (sdw_awaddr ), .d0_awvalid(sdw_awvalid), .d0_awready(sdw_awready),
        .d0_wdata  (sdw_wdata  ), .d0_wstrb  (sdw_wstrb  ),
        .d0_wvalid (sdw_wvalid ), .d0_wready (sdw_wready ),
        .d0_bresp  (sdw_bresp  ), .d0_bvalid (sdw_bvalid ), .d0_bready (sdw_bready ),
        .d0_araddr (sdw_araddr ), .d0_arvalid(sdw_arvalid), .d0_arready(sdw_arready),
        .d0_rdata  (sdw_rdata  ), .d0_rresp  (sdw_rresp  ),
        .d0_rvalid (sdw_rvalid ), .d0_rready (sdw_rready ),

        .d1_awaddr (prm_awaddr ), .d1_awvalid(prm_awvalid), .d1_awready(prm_awready),
        .d1_wdata  (prm_wdata  ), .d1_wstrb  (prm_wstrb  ),
        .d1_wvalid (prm_wvalid ), .d1_wready (prm_wready ),
        .d1_bresp  (prm_bresp  ), .d1_bvalid (prm_bvalid ), .d1_bready (prm_bready ),
        .d1_araddr (prm_araddr ), .d1_arvalid(prm_arvalid), .d1_arready(prm_arready),
        .d1_rdata  (prm_rdata  ), .d1_rresp  (prm_rresp  ),
        .d1_rvalid (prm_rvalid ), .d1_rready (prm_rready )
    );

    // ═══════════════════════════════════════════════════════════════════
    // pram_sd — manual PRAM <-> SD sector persistence (SD LBA 8191)
    // ═══════════════════════════════════════════════════════════════════
    // Save and load are BOTH manual: the only trigger is a JTAG write of
    // a magic word to this register file's CTRL.  Nothing the Mac does
    // can start an SD transfer here; see rtl/soc/pram_sd.v's header.
    // Install the saved PRAM before the 68k can read it.  boot_rom_loaded
    // both tells pram_sd the card is usable AND releases the CPU, so the
    // module raises pram_autoload_pending and fpga_top_clocks.vh holds the
    // CPU on it (with its own bounded timeout).
    wire pram_autoload_pending;
    pram_sd #(.AUTOLOAD_ON_BOOT(1)) u_pram_sd (
        .clk       (core_clk),
        .autoload_pending (pram_autoload_pending),
        // A full reset may arrive after CMD24 has put the card in receive
        // mode.  Let the bounded pram_sd/sd_ctrl operation finish before
        // clearing its parent FSM and dropping the ownership request.
        .rst       (soc_full_rst_bank[5] && !pram_sd_busy_w),
        .boot_done (boot_rom_loaded),

        .s_awaddr  (prm_awaddr ), .s_awvalid(prm_awvalid), .s_awready(prm_awready),
        .s_wdata   (prm_wdata  ), .s_wstrb  (prm_wstrb  ),
        .s_wvalid  (prm_wvalid ), .s_wready (prm_wready ),
        .s_bresp   (prm_bresp  ), .s_bvalid (prm_bvalid ), .s_bready (prm_bready ),
        .s_araddr  (prm_araddr ), .s_arvalid(prm_arvalid), .s_arready(prm_arready),
        .s_rdata   (prm_rdata  ), .s_rresp  (prm_rresp  ),
        .s_rvalid  (prm_rvalid ), .s_rready (prm_rready ),

        .spi_cmd_valid (pram_spi_cmd_valid),
        .spi_cmd_ready (pram_spi_cmd_ready),
        .spi_cmd_data  (pram_spi_cmd_data ),
        .spi_rsp_valid (pram_spi_rsp_valid),
        .spi_rsp_data  (pram_spi_rsp_data ),
        .spi_cs_n_in   (pram_spi_cs_n_w   ),
        .spi_fast_mode (pram_spi_fast_w   ),
        .spi_hs_mode   (pram_spi_hs_w     ),

        .sd_req    (pram_sd_req),
        .sd_gnt    (pram_gnt_q ),

        // A side of pram_cdc (declared in fpga_top_peripherals.vh, which
        // fpga_top.v includes before this file).
        .pram_req   (pram_x_req  ),
        .pram_we    (pram_x_we   ),
        .pram_addr  (pram_x_addr ),
        .pram_wdata (pram_x_wdata),
        .pram_rdata (pram_x_rdata),
        .pram_ack   (pram_x_ack  ),

        // Drives rtc.v's pram_clear (ORed into the existing pb_clk
        // synchroniser) when a load has to fall back to the defaults.
        .pram_default (pram_sd_default),
        .pram_sd_busy (pram_sd_busy_w )
    );

    // ── SD bus grant for pram_sd — never preempt a live SCSI transfer ──
    //
    // The grant may only RISE on a cycle where sd_ctrl_scsi is genuinely
    // idle, and once taken it is held until pram_sd drops its request.
    // That is the whole safety argument for touching the card while the
    // Mac is running: an in-flight SCSI block transfer can never have the
    // SPI mux pulled out from under it.
    //
    // What this does NOT cover, stated plainly: a SCSI command the Mac
    // issues AFTER the grant is taken will find the mux pointed at
    // pram_sd.  It does NOT time out — an older revision of this comment
    // claimed it fell out on "sd_ctrl's own global request watchdog
    // (~10 s)", but this very file instantiates u_sd_ctrl_scsi with
    // REQ_WDOG_ENABLE(0) (see ~120 lines below), which is precisely the
    // knob that removes that escape.  The request PARKS with busy high
    // until pram_sd drops its request and the mux swings back.  That is
    // still not corruption — sd_ctrl's byte sub-FSM never got cmd_ready,
    // so nothing of the SCSI command ever reached the wire — but it is a
    // stall, not a bounded error.  The documented operating restriction
    // is to halt the CPU around pram-save / pram-load;
    // tools/jtag_repl.tcl warns loudly when it is not halted.
    //
    // The OTHER thing "sd_ctrl_scsi is idle" does not mean, and the one
    // that actually bit (2026-08-09): it does not mean THE CARD is idle.
    // A CMD24/CMD25 leaves the card in a receive state that only a
    // completed block + (for CMD25) a 0xFD stop-tran token can leave, and
    // sd_ctrl used to drop `busy` on every write-path error without
    // sending either — so pram_sd's CMD24 at LBA 8191 was consumed as
    // write data for the abandoned session and programmed inside the HDD
    // image at 8192+.  Fixed in sd_ctrl.v by the S_AB_* graceful close;
    // see tb/tb_sd_ctrl.cpp test_cmd25_abort_closes_the_card.  Reset now
    // uses that same close before this file releases SCSI ownership; see
    // test_reset_mid_cmd25_closes_card_before_restart.
    //
    // sdj_writer_busy is in the condition too so a JTAG bulk sector write
    // and a PRAM save can never both think they own the bus.
    always @(posedge core_clk) begin
        if (soc_full_rst_bank[5] && !pram_sd_busy_w) begin
            pram_gnt_q <= 1'b0;
        end else if (!pram_sd_req) begin
            pram_gnt_q <= 1'b0;
        end else if (!pram_gnt_q && boot_rom_loaded &&
                     !core_scsi_busy && !sdj_writer_busy) begin
            pram_gnt_q <= 1'b1;
        end
    end

    // 2026-05-22 — gate the JTAG SD-card writer behind `DISABLE_SD_JTAG_WRITER.
    // The writer is debug-only (host-side SD card provisioning via JTAG); it
    // burns 17.5K LUTs (~8% of the chip) on the runtime path even when never
    // used.  When disabled, the AXI slave port returns OKAY/zero and the SD
    // SPI mux's prov_* port is tied off so SCSI still owns the bus.
`ifdef DISABLE_SD_JTAG_WRITER
    // AXI-Lite terminator on the SPLIT's d0 port (sdw_*), NOT the
    // xbar-facing sdj_* window — pram_sd on d1 has to stay reachable in
    // this build, which is precisely the production build.
    //
    // This used to be a handful of combinational `assign`s
    // (awready=awvalid, bvalid=awvalid&wvalid, ...).  That worked while
    // the only upstream was axi_wide_to_axilite, which holds AWVALID up
    // until BVALID.  It does NOT work behind axil_split2, which is
    // store-and-forward: the split drops AWVALID/WVALID as soon as they
    // are accepted and only then looks for BVALID — so a combinational
    // bvalid=awvalid&wvalid has already fallen by the time anyone looks,
    // and the write never completes.  axil_null_slave registers its
    // response and holds it until BREADY, which is the actual contract.
    axil_null_slave #(
        .ADDR_WIDTH(20),
        .DATA_WIDTH(32)
    ) u_sdw_null (
        .clk    (core_clk),
        .rst    (soc_full_rst_bank[5]),
        .awaddr (sdw_awaddr ), .awvalid(sdw_awvalid), .awready(sdw_awready),
        .wdata  (sdw_wdata  ), .wstrb  (sdw_wstrb  ),
        .wvalid (sdw_wvalid ), .wready (sdw_wready ),
        .bresp  (sdw_bresp  ), .bvalid (sdw_bvalid ), .bready (sdw_bready ),
        .araddr (sdw_araddr ), .arvalid(sdw_arvalid), .arready(sdw_arready),
        .rdata  (sdw_rdata  ), .rresp  (sdw_rresp  ),
        .rvalid (sdw_rvalid ), .rready (sdw_rready )
    );
    // SPI prov_* tie-offs: never asserts → mux gives SPI bus to SCSI.
    assign prov_spi_cmd_valid = 1'b0;
    assign prov_spi_cmd_data  = 8'd0;
    assign prov_spi_cs_n_w    = 1'b1;
    assign prov_spi_fast_w    = 1'b0;
    assign prov_spi_hs_w      = 1'b0;
    assign sdj_writer_busy    = 1'b0;
    // sdw_* are all consumed by u_sdw_null above now; only the tied-off
    // provision SPI returns are genuinely unread here.
    wire _unused_sdj = &{1'b0, prov_spi_cmd_ready,
                          prov_spi_rsp_valid, prov_spi_rsp_data, 1'b0};
`else
    sd_jtag_writer u_sd_jtag_writer (
        .clk           (core_clk),
        .rst           (soc_full_rst_bank[5]),
        .boot_done     (boot_rom_loaded),

        .s_awaddr      (sdw_awaddr),
        .s_awvalid     (sdw_awvalid),
        .s_awready     (sdw_awready),
        .s_wdata       (sdw_wdata),
        .s_wstrb       (sdw_wstrb),
        .s_wvalid      (sdw_wvalid),
        .s_wready      (sdw_wready),
        .s_bresp       (sdw_bresp),
        .s_bvalid      (sdw_bvalid),
        .s_bready      (sdw_bready),
        .s_araddr      (sdw_araddr),
        .s_arvalid     (sdw_arvalid),
        .s_arready     (sdw_arready),
        .s_rdata       (sdw_rdata),
        .s_rresp       (sdw_rresp),
        .s_rvalid      (sdw_rvalid),
        .s_rready      (sdw_rready),

        .spi_cmd_valid (prov_spi_cmd_valid),
        .spi_cmd_ready (prov_spi_cmd_ready),
        .spi_cmd_data  (prov_spi_cmd_data),
        .spi_rsp_valid (prov_spi_rsp_valid),
        .spi_rsp_data  (prov_spi_rsp_data),
        .spi_cs_n_in   (prov_spi_cs_n_w),
        .spi_fast_mode (prov_spi_fast_w),
        .spi_hs_mode   (prov_spi_hs_w),
        .writer_busy   (sdj_writer_busy)
    );
`endif

    // ═══════════════════════════════════════════════════════════════════
    // SCSI SD path — sd_ctrl on core_clk, bridge to scsi.v on pb_clk
    // ═══════════════════════════════════════════════════════════════════
    // sd_scsi_bridge handles the only CDC on the SD path.  scsi.v drives
    // its sd_* outputs on pb_clk; the bridge reproduces them on core_clk
    // for sd_ctrl_scsi.  sd_ctrl_scsi drives the SPI handshake into the
    // mux's scsi_* port; the same domain as sd_spi means no further CDC.

    // core-side bridge → sd_ctrl request fields
    wire [2:0]  core_scsi_cmd_type;
    wire [31:0] core_scsi_lba;
    wire [15:0] core_scsi_block_count;
    wire        core_scsi_go;
    wire        core_scsi_done;
    wire        core_scsi_error;
    /* verilator lint_off UNUSEDSIGNAL */
    wire [3:0]  core_scsi_err_cause;
    wire [15:0] core_scsi_dbg_rd_crc_calc;
    wire [15:0] core_scsi_dbg_rd_crc_recv;
    wire [7:0]  core_scsi_dbg_last_real_r1;
    wire [3:0]  core_scsi_dbg_cur_cmd;
    wire [31:0] core_scsi_dbg_lba_lat;
    wire [15:0] core_scsi_dbg_block_idx;
    wire [7:0]  core_scsi_dbg_last_write_resp;
    wire [7:0]  core_scsi_dbg_last_poll_cnt;
    wire        core_scsi_wr_valid;
    /* verilator lint_on UNUSEDSIGNAL */
    wire        core_scsi_rd_valid;
    wire [7:0]  core_scsi_rd_data;
    wire        core_scsi_rd_ready;
    wire        core_scsi_wr_ready;
    wire [7:0]  core_scsi_wr_data;
    wire        core_scsi_wr_avail;

    sd_scsi_bridge u_sd_scsi_bridge (
        // pb_clk side — connects to scsi.v's sd_* ports (declared in
        // fpga_top_peripherals.vh; both .vh files share the fpga_top
        // module scope).
        .pb_clk         (pb_clk),
        // pb-bank [2] — same reset domain as scsi.v itself, so the
        // bridge state matches scsi.v on a debug-full-reset.
        .pb_rst         (pb_full_rst_bank[2]),
        .pb_cmd_type    (scsi_sd_cmd_type_w),
        .pb_lba         (scsi_sd_lba_w),
        .pb_block_count (scsi_sd_block_count_w),
        .pb_go          (scsi_sd_go_w),
        .pb_busy        (scsi_sd_busy_w),
        .pb_done        (scsi_sd_done_w),
        .pb_error       (scsi_sd_error_w),
        .pb_rd_valid    (scsi_sd_rd_valid_w),
        .pb_rd_data     (scsi_sd_rd_data_w),
        .pb_rd_ready    (scsi_sd_rd_ready_w),
        .pb_wr_ready    (scsi_sd_wr_ready_w),
        .pb_wr_data     (scsi_sd_wr_data_w),
        .pb_wr_avail    (scsi_sd_wr_avail_w),

        // core_clk side — connects to sd_ctrl_scsi below.
        .core_clk         (core_clk),
        // Do not reset the bridge until sd_ctrl has closed the card session.
        // ⚠️ `warm_storage_reset` MUST be in this OR (added 2026-09-07).
        //
        // The bridge's transfer-loss rescue keys on an EPOCH TOGGLE of this
        // reset: core_rst's rising edge flips core_rst_epoch, the pb side
        // resyncs it, and the resulting pb_peer_reset drives pb_abort, which
        // is the ONLY thing that clears pb_busy_internal when a request dies
        // without a done.
        //
        // `warm_storage_reset` is what the 68040 RESET instruction drives, and
        // it kills an in-flight sd_ctrl CMD18 read: sd_ctrl jumps to IDLE with
        // NO done and NO error (its read path has no graceful close).  With
        // that signal absent here, no epoch toggled, the rescue never fired,
        // and pb_busy_internal latched high FOREVER -- vh_busy stuck,
        // vh_buf_count 0, so the 53C96 could never obtain the byte it owed and
        // the CPU stalled forever inside the ROM's blind MOVE.W pseudo-DMA
        // burst (measured on p152 at ROM 0x40899664, pinned 10/10).
        // `warm_peripheral_reset` could not stand in for it: the sequencer
        // only pulses that once `storage_busy` clears, which is precisely what
        // never happens here.
        //
        // This is the forward-progress fix.  The alternative -- re-enabling
        // REQ_WDOG_ENABLE on u_sd_ctrl_scsi -- only bounds the damage with a
        // timeout, which is the pattern being removed from this design.
        // ⚠️ `dbg_cold_reset_hold` MUST be in this OR for the SAME reason
        // `warm_storage_reset` was added on 2026-09-07 — see the epoch note above.
        //
        // A BUTTON reset reaches here via soc_full_rst_bank[5] and toggles the epoch, so
        // the rescue fires and the bridge recovers. A JTAG/debug cold reset does NOT:
        // `dbg_cold_reset_hold` deliberately does not reach soc_full_rst (fpga_top_clocks.vh
        // says so where it folds it into `cpu_rst_or`), and it is not a 68040 RESET
        // instruction either, so neither warm_* signal fires. It therefore re-armed the boot
        // FSM — every reset re-copies the ROM — WITHOUT toggling the storage epoch. If it
        // landed mid-CMD18, `pb_busy_internal` latched high forever, the ROM re-copy it had
        // just demanded could never complete, and `boot_rom_ready` never rose: the 68k stayed
        // in reset until a power cycle. Observed as "a reset kills the board".
        .core_rst         (soc_full_rst_bank[5] | warm_peripheral_reset
                                                | warm_storage_reset
                                                | dbg_cold_reset_hold),
        .core_cmd_type    (core_scsi_cmd_type),
        .core_lba         (core_scsi_lba),
        .core_block_count (core_scsi_block_count),
        .core_go          (core_scsi_go),
        .core_busy        (core_scsi_busy),
        .core_done        (core_scsi_done),
        .core_error       (core_scsi_error),
        .core_rd_valid    (core_scsi_rd_valid),
        .core_rd_data     (core_scsi_rd_data),
        .core_rd_ready    (core_scsi_rd_ready),
        .core_wr_ready    (core_scsi_wr_ready),
        .core_wr_data     (core_scsi_wr_data),
        .core_wr_avail    (core_scsi_wr_avail)
    );

    // REQ_WDOG_ENABLE(0) — the per-request watchdog's ERR_WDOG escape is
    // OFF for the SCSI volume (2026-08-03, user directive).  Rationale:
    // after any reset the machine loads the OS for a few seconds and then
    // fails the load, and the suspicion is that a healthy-but-slow request
    // is being killed by a spurious expiry rather than a genuinely dead
    // provider.  With the escape gone, such a request now PARKS instead of
    // erroring, which is the point: a park is directly observable via
    // `wedge-status` (lsu=LD_WAIT on 0x50F0Fxxx) and names the stalled
    // request, where the error path silently fell back to the ROM's disk
    // prompt and destroyed the evidence.
    //
    // Measured before landing this, so the next reader does not re-walk it:
    // the boot reaches the disk prompt with exc_count=0x1874 inside ~4 s of
    // reset, and REQ_WDOG_BASE_TICKS is ~10.07 s — so on THAT capture the
    // watchdog demonstrably had not fired yet, and no vec-2 bus error ever
    // appeared either.  The failure is intermittent, so this build tests
    // the watchdog hypothesis on the runs where it DOES get further.
    //
    // The PRAM instance (pram_sd.v) deliberately keeps its watchdog.
    sd_ctrl #(
        .REQ_WDOG_ENABLE      (0),
        .MULTI_WRITE_AS_CMD24 (1),
        // At core=200 MHz this is 80 ns: four 50 MHz pb clocks, exceeding
        // the read toggle/data sampling window (three pb clocks). Lower
        // supported core frequencies only increase that margin.
        .RD_FLUSH_PACE_CYCLES (16),
        .READ_PIPELINE        (1)
    ) u_sd_ctrl_scsi (
        .clk           (core_clk),
        // soc_full_rst — share bank [5] with sd_spi so a debug-full-
        // reset rescues a hung in-flight SCSI block transfer.
        .rst           (soc_full_rst_bank[5] | warm_storage_reset),

        // CRC_ON_OFF (CMD59) is a whole-card-session setting, sent once
        // by boot_fsm's own init sequence (rtl/fpga_top_boot_master.vh)
        // before this phase-B instance ever gets the SPI bus — no
        // second CMD59 needed here.
        .crc_check_en  (boot_sd_crc_enabled),
        .cmd_type      (core_scsi_cmd_type),
        .lba           (core_scsi_lba),
        .block_count   (core_scsi_block_count),
        .go            (core_scsi_go),

        .rd_valid      (core_scsi_rd_valid),
        .rd_data       (core_scsi_rd_data),
        .rd_ready      (core_scsi_rd_ready),

        .wr_ready      (core_scsi_wr_ready),
        .wr_valid      (core_scsi_wr_valid),
        .wr_data       (core_scsi_wr_data),
        .wr_avail      (core_scsi_wr_avail),

        .spi_cmd_valid (scsi_spi_cmd_valid),
        .spi_cmd_ready (scsi_spi_cmd_ready),
        .spi_cmd_data  (scsi_spi_cmd_data ),
        .spi_rsp_valid (scsi_spi_rsp_valid),
        .spi_rsp_data  (scsi_spi_rsp_data ),

        .busy          (core_scsi_busy),
        .done          (core_scsi_done),
        .error         (core_scsi_error),
        .err_cause     (core_scsi_err_cause),
        .dbg_rd_crc_calc(core_scsi_dbg_rd_crc_calc),
        .dbg_rd_crc_recv(core_scsi_dbg_rd_crc_recv),
        .dbg_last_real_r1(core_scsi_dbg_last_real_r1),
        .dbg_cur_cmd   (core_scsi_dbg_cur_cmd),
        .dbg_lba_lat   (core_scsi_dbg_lba_lat),
        .dbg_last_crc7_sent(/* unused */),
        .dbg_last_write_resp(core_scsi_dbg_last_write_resp),
        .dbg_block_idx (core_scsi_dbg_block_idx),
        .dbg_last_poll_cnt(core_scsi_dbg_last_poll_cnt)
    );

    // CS_N for the SCSI phase: assert low while sd_ctrl_scsi is busy
    // (it owns the bus for the duration of one block transfer; between
    // transactions CS releases as the SD spec expects).  Registered to
    // align with sd_spi's internal cs_n_in handshake.
    reg scsi_spi_cs_n_q;
    always @(posedge core_clk) begin
        if ((soc_full_rst_bank[5] || warm_storage_reset) && !core_scsi_busy)
                                  scsi_spi_cs_n_q <= 1'b1;
        else                      scsi_spi_cs_n_q <= ~core_scsi_busy;
    end
    assign scsi_spi_cs_n_w = scsi_spi_cs_n_q;

    // sd_spi's module-default dividers (SLOW_HALF=250, FAST_HALF=4,
    // HS_HALF=2) are deliberately NOT overridden here: they are ratios
    // validated against real hardware at a 200 MHz core_clk (50 MHz HS /
    // 25 MHz fast SCK), and SCK = core_clk/(2*HALF) scales automatically
    // with whatever core_clk the build actually runs — 25 MHz HS / 12.5
    // MHz fast at the canonical 100 MHz core_clk build.  Do NOT pin
    // FAST_HALF/HS_HALF at an absolute target frequency derived from
    // CORE_CLK_HZ: that silently halves the validated SCK rate the
    // moment a build goes back to 200 MHz.
    //
    // realboot harness exception (tb-fpga-top-rom-realboot,
    // FPGA_ROM_SIM_REALBOOT): SPI is inherently 1 bit/edge -- the
    // 1 MiB real ROM image over CMD18 is ~67M core_clk cycles at the
    // default FAST_HALF=4 divider (period = 2*HALF core_clk cycles),
    // tens of minutes of Verilator wall-clock for genuinely correct,
    // un-skipped, bit-accurate real protocol data.
    //
    // FAST_HALF=HS_HALF=1 (SCK toggling every core_clk cycle -- the
    // fastest this exact sd_spi.v shape can run) was tried first and
    // caused a genuine stall: real SD command traffic (CMD0..CMD9)
    // went through fine, then the run sat for 2M+ core_clk cycles with
    // NO further command activity -- far beyond boot_fsm.v's own
    // POLL_TIMEOUT=4096-byte-poll R1 budget (worst case ~65K cycles),
    // so this is not just "relatively more visible", it is a real
    // timing hazard at that extreme divider (most likely inside
    // sd_spi.v's own byte-engine, which was never validated at
    // sub-2-cycle SCK half-periods). Backed off to FAST_HALF=HS_HALF=2:
    // this is NOT a new, unvalidated setting -- HS_HALF=2 is the exact
    // ratio already proven on real hardware (see the header comment
    // above), just applied to the ordinary post-init "fast" phase
    // instead of requiring a real CMD6 HS switch (which this harness's
    // card model always refuses, matching real Q700-shipped cards).
    // Real 2x cut vs. default FAST_HALF=4, zero new timing risk.
    // SLOW_HALF trimmed 250->32 (8x): only the tiny CMD0..CMD9 init
    // exchange runs at this rate, so the absolute savings are small,
    // but 32 is still comfortably above the divider that stalled at 1.
    // Pure simulated-bus-speed knob (no protocol/FSM/content change),
    // sim-only / additive: the real FPGA path and the default
    // tb-fpga-top-rom target never define FPGA_ROM_SIM_REALBOOT, so the
    // hardware-validated 200 MHz-core_clk ratios above are untouched.
    sd_spi
`ifdef FPGA_ROM_SIM_REALBOOT
        #( .SLOW_HALF(32), .FAST_HALF(2), .HS_HALF(2) )
`endif
        u_sd_spi (
        .clk      (core_clk     ),
        // soc_full_rst — debug-full-reset rescues a hung SPI transaction
        // alongside the boot_fsm re-arm.  Bank bit [5] (DMA / SD region).
        // sd_ctrl defers reset while closing a live CMD24/CMD25.  Keep the
        // byte engine alive for the same interval; resetting it here would
        // strand sd_ctrl in BS_WAIT and leave the card session open.
        .rst      ((soc_full_rst_bank[5] || warm_storage_reset) &&
                   !storage_owner_busy),
        .fast_mode(spi_fast_mode),
        .hs_mode  (spi_hs_mode  ),
        .cs_n_in  (spi_cs_n_wire),
        .cmd_valid(spi_cmd_valid),
        .cmd_ready(spi_cmd_ready),
        .cmd_data (spi_cmd_data ),
        .rsp_valid(spi_rsp_valid),
        .rsp_data (spi_rsp_data ),
        .spi_clk  (sd_clk       ),
        .spi_mosi (sd_mosi      ),
        .spi_miso (sd_miso      ),
        .spi_cs_n (sd_cs_n      )
    );
