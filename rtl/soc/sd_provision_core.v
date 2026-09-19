// sd_provision_core.v — synthesisable core of the dedicated SD-card
// provisioning bitstream (everything except clocks + the JTAG-AXI IP).
//
// Purpose
//   Standalone fast bulk SD-card writer.  This is NOT part of fpga_top —
//   it is the guts of sd_provision_top.v, a separate minimal bitstream
//   that is JTAG-loaded transiently to flash a disk image onto the SD
//   card, then replaced by the normal fpga_top bitstream.  Nothing here
//   is instantiated by fpga_top.v / mac_top.v.
//
// Topology
//   boot_fsm (card init: CMD0/8/55+41/58/6 + one dummy CMD18 sector,
//             AXI write master answered by a local always-OK stub)
//        │ boot_* port
//        ▼
//   sd_spi_mux (boot_done = rom_loaded, b_sel = 0 → prov owns phase B)
//        ▲ prov_* port                      │
//   sd_bulk_writer (burst AXI4 slave,       ▼
//     128 KiB staging + CMD25/CMD18)      sd_spi ──► physical SD pins
//
//   boot_fsm is reused verbatim for its hardware-validated SD init
//   sequence; NUM_SECTORS=1 makes it read a single throwaway sector
//   (LBA 0) into the stub before releasing the bus to the bulk writer.
//
// Split rationale: keeping the core free of IBUFDS_GTE4/BUFG_GT/clk_rst
// and the Vivado jtag_axi IP lets the unit tb (tb-sd-provision) drive
// the AXI face and model the SD card at the physical SPI pins, exactly
// like tb-sd-boot.
//
// Verilog-2005, synchronous active-high rst.

`default_nettype none

module sd_provision_core #(
    parameter integer STAGE_SECTORS = 256,
    // sd_spi clock dividers — production defaults sized for the fpga_top
    // clock lineage (tb overrides these for fast simulation).  At the
    // 100 MHz fabric_clk-derived core clock of sd_provision_top these
    // yield 200 kHz init / 12.5 MHz fast / 25 MHz HS SPI.
    parameter SPI_SLOW_HALF = 250,
    parameter SPI_FAST_HALF = 4,
    parameter SPI_HS_HALF   = 2
) (
    input  wire        clk,
    input  wire        rst,

    // 32-bit AXI4 slave face (from the JTAG-AXI master).
    input  wire [31:0] s_awaddr,
    input  wire [7:0]  s_awlen,
    input  wire        s_awvalid,
    output wire        s_awready,
    input  wire [31:0] s_wdata,
    input  wire [3:0]  s_wstrb,
    input  wire        s_wlast,
    input  wire        s_wvalid,
    output wire        s_wready,
    output wire [1:0]  s_bresp,
    output wire        s_bvalid,
    input  wire        s_bready,

    input  wire [31:0] s_araddr,
    input  wire [7:0]  s_arlen,
    input  wire        s_arvalid,
    output wire        s_arready,
    output wire [31:0] s_rdata,
    output wire [1:0]  s_rresp,
    output wire        s_rlast,
    output wire        s_rvalid,
    input  wire        s_rready,

    // Physical SD card pins.
    output wire        sd_clk,
    output wire        sd_mosi,
    input  wire        sd_miso,
    output wire        sd_cs_n,

    // Status (LEDs / tb observation).
    output wire        card_ready,
    output wire        init_error,
    output wire        writer_busy
);

    // ──────────────────────────────────────────────────────────────────
    // boot_fsm — SD card init + one dummy sector, then bus release.
    // ──────────────────────────────────────────────────────────────────
    wire       bf_cmd_valid;
    wire       bf_cmd_ready;
    wire [7:0] bf_cmd_data;
    wire       bf_rsp_valid;
    wire [7:0] bf_rsp_data;
    wire       bf_cs_n;
    wire       bf_fast_mode;
    wire       bf_hs_mode;
    wire       bf_rom_loaded;
    wire       bf_error;

    // boot_fsm AXI write master → local always-OK single-beat stub
    // (the dummy sector data is discarded).
    wire [3:0]  bf_awid;
    wire [31:0] bf_awaddr_unused;
    wire [7:0]  bf_awlen_unused;
    wire [2:0]  bf_awsize_unused;
    wire [1:0]  bf_awburst_unused;
    wire        bf_awvalid;
    wire [31:0] bf_wdata_unused;
    wire [3:0]  bf_wstrb_unused;
    wire        bf_wlast_unused;
    wire        bf_wvalid;
    wire        bf_bready;

    reg       stub_aw_seen_q;
    reg       stub_w_seen_q;
    reg [3:0] stub_bid_q;
    wire      stub_bvalid = stub_aw_seen_q && stub_w_seen_q;

    always @(posedge clk) begin
        if (rst) begin
            stub_aw_seen_q <= 1'b0;
            stub_w_seen_q  <= 1'b0;
            stub_bid_q     <= 4'd0;
        end else begin
            if (bf_awvalid && !stub_aw_seen_q) begin
                stub_aw_seen_q <= 1'b1;
                stub_bid_q     <= bf_awid;
            end
            if (bf_wvalid && !stub_w_seen_q) stub_w_seen_q <= 1'b1;
            if (stub_bvalid && bf_bready) begin
                stub_aw_seen_q <= 1'b0;
                stub_w_seen_q  <= 1'b0;
            end
        end
    end

    boot_fsm #(
        .NUM_SECTORS  (32'd1),          // init + one throwaway sector
        .START_SECTOR (32'd0),
        .ROM_BASE_ADDR(32'h0000_0000),
        .AXI_ID       (4'd0),
        .ZERO_BYTES   (32'd0)           // no RAM-zero pass — no RAM here
    ) u_boot_fsm (
        .clk            (clk),
        .rst            (rst),
        // Don't-care here: ZERO_BYTES=0 above already compiles the RAM
        // pre-zero pass out entirely.  Tied HIGH rather than low so this
        // reads as "nothing is being suppressed at this instance" -- the
        // cold/warm gating that matters lives in fpga_top_boot_master.vh.
        .zero_en        (1'b1),

        .cfg_skip_cmd59 (1'b0),
        .cfg_force_hs   (1'b0),

        .spi_cmd_valid  (bf_cmd_valid),
        .spi_cmd_ready  (bf_cmd_ready),
        .spi_cmd_data   (bf_cmd_data),
        .spi_rsp_valid  (bf_rsp_valid),
        .spi_rsp_data   (bf_rsp_data),
        .spi_cs_n       (bf_cs_n),
        .spi_fast_mode  (bf_fast_mode),
        .spi_hs_mode    (bf_hs_mode),

        .m_axi_awid     (bf_awid),
        .m_axi_awaddr   (bf_awaddr_unused),
        .m_axi_awlen    (bf_awlen_unused),
        .m_axi_awsize   (bf_awsize_unused),
        .m_axi_awburst  (bf_awburst_unused),
        .m_axi_awvalid  (bf_awvalid),
        .m_axi_awready  (!stub_aw_seen_q),

        .m_axi_wdata    (bf_wdata_unused),
        .m_axi_wstrb    (bf_wstrb_unused),
        .m_axi_wlast    (bf_wlast_unused),
        .m_axi_wvalid   (bf_wvalid),
        .m_axi_wready   (!stub_w_seen_q),

        .m_axi_bid      (stub_bid_q),
        .m_axi_bresp    (2'b00),
        .m_axi_bvalid   (stub_bvalid),
        .m_axi_bready   (bf_bready),

        .rom_loading    (),
        .rom_loaded     (bf_rom_loaded),
        .error          (bf_error),
        .sd_crc_enabled (),
        .card_num_lbas  (),

        .dbg_st         (),
        .dbg_cur_cmd    (),
        .dbg_last_r1    (),
        .dbg_last_rx    (),
        .dbg_acmd41_try (),
        .dbg_rsp_count  (),
        .dbg_sector     (),
        .dbg_is_sdhc    (),
        .dbg_ocr0       (),
        .dbg_err_cause  (),
        .dbg_ctrl_retry (),
        .dbg_rd_crc_calc(),
        .dbg_rd_crc_recv(),
        .dbg_sdctrl_cur_cmd(),
        .dbg_sdctrl_lba_lat(),
        .dbg_sdctrl_last_crc7_sent(),
        .dbg_sdctrl_last_poll_cnt(),
        .dbg_ctrl_err_cause(),
        .dbg_ctrl_last_real_r1(),
        .dbg_attempt_log0(),
        .dbg_attempt_log1(),
        .dbg_attempt_log2(),
        .dbg_attempt_log3(),
        .dbg_attempt_log4(),
        .dbg_attempt_log5()
    );

    assign card_ready = bf_rom_loaded;
    assign init_error = bf_error;

    // ──────────────────────────────────────────────────────────────────
    // sd_bulk_writer — burst AXI4 slave + CMD25/CMD18 engine.
    // ──────────────────────────────────────────────────────────────────
    wire       prov_cmd_valid;
    wire       prov_cmd_ready;
    wire [7:0] prov_cmd_data;
    wire       prov_rsp_valid;
    wire [7:0] prov_rsp_data;
    wire       prov_cs_n;
    wire       prov_fast_mode;
    wire       prov_hs_mode;

    sd_bulk_writer #(
        .STAGE_SECTORS (STAGE_SECTORS)
    ) u_bulk_writer (
        .clk           (clk),
        .rst           (rst),
        .card_ready    (bf_rom_loaded),
        .init_error    (bf_error),

        .s_awaddr      (s_awaddr),
        .s_awlen       (s_awlen),
        .s_awvalid     (s_awvalid),
        .s_awready     (s_awready),
        .s_wdata       (s_wdata),
        .s_wstrb       (s_wstrb),
        .s_wlast       (s_wlast),
        .s_wvalid      (s_wvalid),
        .s_wready      (s_wready),
        .s_bresp       (s_bresp),
        .s_bvalid      (s_bvalid),
        .s_bready      (s_bready),
        .s_araddr      (s_araddr),
        .s_arlen       (s_arlen),
        .s_arvalid     (s_arvalid),
        .s_arready     (s_arready),
        .s_rdata       (s_rdata),
        .s_rresp       (s_rresp),
        .s_rlast       (s_rlast),
        .s_rvalid      (s_rvalid),
        .s_rready      (s_rready),

        .spi_cmd_valid (prov_cmd_valid),
        .spi_cmd_ready (prov_cmd_ready),
        .spi_cmd_data  (prov_cmd_data),
        .spi_rsp_valid (prov_rsp_valid),
        .spi_rsp_data  (prov_rsp_data),
        .spi_cs_n_in   (prov_cs_n),
        .spi_fast_mode (prov_fast_mode),
        .spi_hs_mode   (prov_hs_mode),

        .writer_busy   (writer_busy)
    );

    // ──────────────────────────────────────────────────────────────────
    // sd_spi_mux + sd_spi — boot_fsm owns the bus until init completes,
    // then the bulk writer (prov port, b_sel=0) owns phase B.
    // ──────────────────────────────────────────────────────────────────
    wire       spi_cmd_valid;
    wire       spi_cmd_ready;
    wire [7:0] spi_cmd_data;
    wire       spi_rsp_valid;
    wire [7:0] spi_rsp_data;
    wire       spi_cs_n_in;
    wire       spi_fast_mode;
    wire       spi_hs_mode;

    sd_spi_mux u_spi_mux (
        .boot_done      (bf_rom_loaded),
        .b_sel          (1'b0),          // phase B → prov (bulk writer)
        // No pram_sd in the provisioning bitstream (PRAM persistence is a
        // runtime fpga_top feature); tie its phase-B port off.
        .pram_gnt       (1'b0),
        .pram_cmd_valid (1'b0),
        .pram_cmd_ready (),
        .pram_cmd_data  (8'h00),
        .pram_rsp_valid (),
        .pram_rsp_data  (),
        .pram_cs_n_in   (1'b1),
        .pram_fast_mode (1'b0),
        .pram_hs_mode   (1'b0),

        .boot_cmd_valid (bf_cmd_valid),
        .boot_cmd_ready (bf_cmd_ready),
        .boot_cmd_data  (bf_cmd_data),
        .boot_rsp_valid (bf_rsp_valid),
        .boot_rsp_data  (bf_rsp_data),
        .boot_cs_n_in   (bf_cs_n),
        .boot_fast_mode (bf_fast_mode),
        .boot_hs_mode   (bf_hs_mode),

        .prov_cmd_valid (prov_cmd_valid),
        .prov_cmd_ready (prov_cmd_ready),
        .prov_cmd_data  (prov_cmd_data),
        .prov_rsp_valid (prov_rsp_valid),
        .prov_rsp_data  (prov_rsp_data),
        .prov_cs_n_in   (prov_cs_n),
        .prov_fast_mode (prov_fast_mode),
        .prov_hs_mode   (prov_hs_mode),

        // SCSI port — no SCSI in the provisioning bitstream.
        .scsi_cmd_valid (1'b0),
        .scsi_cmd_ready (),
        .scsi_cmd_data  (8'h00),
        .scsi_rsp_valid (),
        .scsi_rsp_data  (),
        .scsi_cs_n_in   (1'b1),
        .scsi_fast_mode (1'b0),
        .scsi_hs_mode   (1'b0),

        .spi_cmd_valid  (spi_cmd_valid),
        .spi_cmd_ready  (spi_cmd_ready),
        .spi_cmd_data   (spi_cmd_data),
        .spi_rsp_valid  (spi_rsp_valid),
        .spi_rsp_data   (spi_rsp_data),
        .spi_cs_n_in    (spi_cs_n_in),
        .spi_fast_mode  (spi_fast_mode),
        .spi_hs_mode    (spi_hs_mode)
    );

    sd_spi #(
        .SLOW_HALF (SPI_SLOW_HALF),
        .FAST_HALF (SPI_FAST_HALF),
        .HS_HALF   (SPI_HS_HALF)
    ) u_sd_spi (
        .clk       (clk),
        .rst       (rst),
        .fast_mode (spi_fast_mode),
        .hs_mode   (spi_hs_mode),
        .cs_n_in   (spi_cs_n_in),
        .cmd_valid (spi_cmd_valid),
        .cmd_ready (spi_cmd_ready),
        .cmd_data  (spi_cmd_data),
        .rsp_valid (spi_rsp_valid),
        .rsp_data  (spi_rsp_data),
        .spi_clk   (sd_clk),
        .spi_mosi  (sd_mosi),
        .spi_miso  (sd_miso),
        .spi_cs_n  (sd_cs_n)
    );

    // verilator lint_off UNUSED
    wire _unused = &{1'b0, bf_awaddr_unused, bf_awlen_unused,
                     bf_awsize_unused, bf_awburst_unused, bf_wdata_unused,
                     bf_wstrb_unused, bf_wlast_unused, 1'b0};
    // verilator lint_on UNUSED

endmodule

`default_nettype wire
