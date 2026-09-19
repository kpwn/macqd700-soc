// tb_sd_boot_top.v — wrapper module for the boot_fsm / sd_spi / sd_spi_mux
// unit testbench (tb_sd_boot.cpp).
//
// Not for synthesis — this is a Verilator-only harness that:
//   * Instantiates one sd_spi with tiny clock dividers (SLOW_HALF=2,
//     FAST_HALF=2, HS_HALF=2) so each byte takes only ~32 core cycles.
//     Production values live on the sd_spi defaults (200 MHz → 400 kHz
//     / 25 MHz / 33 MHz).
//   * Instantiates boot_fsm with NUM_SECTORS=16 (= 8 KiB) so the tb
//     finishes quickly rather than simulating the 2048-sector real boot.
//   * Instantiates sd_spi_mux with boot_done=0 (so boot_fsm owns the
//     bus for the entire test) and ties off the other two masters.
//   * Exposes the physical SD pins (spi_clk, spi_mosi, spi_miso,
//     spi_cs_n) and the AXI4-MM master pins of boot_fsm to the C++ tb.

module tb_sd_boot_top #(
    // Size of boot_fsm's RAM pre-zero pass (ST_ZERO_AW/W/B).
    //
    // Default 0 preserves the historical scenarios exactly: they validate
    // the SD/AXI ROM-copy contract and count AW transactions, so an extra
    // zeroing pass would perturb both.  The `tb-sd-boot-zero` Makefile
    // target overrides this with -GZERO_BYTES=<n> (kept in lockstep with
    // the C++ side's TB_ZERO_BYTES define) to exercise the zero pass and
    // its BRESP-error handling.
    parameter [31:0] ZERO_BYTES = 32'd0,
    // Beats-minus-one per zero-pass AXI write burst.  Mirrors boot_fsm's
    // own default (the production shape) so the tb measures what the SoC
    // actually issues; tb-sd-boot-zero-shape overrides it with -G to sweep
    // the burst geometry and prove the pass is correct at every shape, not
    // just the one the production build happens to use.
    parameter [7:0]  ZERO_AWLEN = 8'd255,

    // ROM-mirror + VRAM-zero passthroughs (task: ROM-overlay-bypass, see
    // boot_fsm.v's own MIRROR_LOW_RAM/VRAM_ZERO_* parameter docs).  All
    // default to boot_fsm's own v1-equivalent defaults, so every
    // pre-existing scenario (tb-sd-boot, tb-sd-boot-zero) is unaffected;
    // the tb-sd-boot-mirror target is the only one that overrides them.
    parameter        MIRROR_LOW_RAM     = 1'b0,
    parameter [31:0] MIRROR_IMAGE_BYTES = 32'h0010_0000,
    parameter [31:0] VRAM_ZERO_BASE     = 32'd0,
    parameter [31:0] VRAM_ZERO_BYTES    = 32'd0,
    parameter        WORD_FIFO_LOG2P    = 5
) (
    input  wire        clk,
    input  wire        rst,
    // Runtime enable for boot_fsm's RAM pre-zero pass.  Driven by the C++
    // tb; the existing scenarios hold it HIGH so their behaviour is
    // unchanged, and the skip case gets its own scenario.
    input  wire        zero_en,

    // ── physical SD pins (driven by tb_sd_boot.cpp sd-card model) ────
    output wire [31:0] card_num_lbas,
    output wire        spi_clk,
    output wire        spi_mosi,
    input  wire        spi_miso,
    output wire        spi_cs_n,

    // ── AXI4-MM write master (slave side modelled by tb_sd_boot.cpp) ──
    output wire [3:0]  m_axi_awid,
    output wire [31:0] m_axi_awaddr,
    output wire [7:0]  m_axi_awlen,
    output wire [2:0]  m_axi_awsize,
    output wire [1:0]  m_axi_awburst,
    output wire        m_axi_awvalid,
    input  wire        m_axi_awready,

    output wire [31:0] m_axi_wdata,
    output wire [3:0]  m_axi_wstrb,
    output wire        m_axi_wlast,
    output wire        m_axi_wvalid,
    input  wire        m_axi_wready,

    input  wire [3:0]  m_axi_bid,
    input  wire [1:0]  m_axi_bresp,
    input  wire        m_axi_bvalid,
    output wire        m_axi_bready,

    // ── status taps ──────────────────────────────────────────────────
    output wire        rom_loading,
    output wire        rom_loaded,
    output wire        error,
    output wire        sd_crc_enabled,
    output wire        spi_fast_mode,
    output wire        spi_hs_mode,

    output wire [5:0]  dbg_st,
    output wire [7:0]  dbg_last_r1,
    output wire [15:0] dbg_sector,
    output wire [2:0]  dbg_err_cause,
    output wire [3:0]  dbg_ctrl_retry,
    output wire [15:0] dbg_rd_crc_calc,
    output wire [15:0] dbg_rd_crc_recv
);

    // ── boot_fsm → sd_spi_mux (phase-A port) ─────────────────────────
    wire       bf_cmd_valid;
    wire       bf_cmd_ready;
    wire [7:0] bf_cmd_data;
    wire       bf_rsp_valid;
    wire [7:0] bf_rsp_data;
    wire       bf_cs_n;
    wire       bf_fast_mode;
    wire       bf_hs_mode;

    // ── sd_spi_mux → sd_spi ─────────────────────────────────────────
    wire       spi_cmd_valid;
    wire       spi_cmd_ready;
    wire [7:0] spi_cmd_data;
    wire       spi_rsp_valid;
    wire [7:0] spi_rsp_data;
    wire       spi_cs_n_in;
    wire       spi_fast_mode_w;
    wire       spi_hs_mode_w;

    assign spi_fast_mode = spi_fast_mode_w;
    assign spi_hs_mode   = spi_hs_mode_w;

    // ── boot_fsm (small-sector production-equivalent model) ─────────
    boot_fsm #(
        .NUM_SECTORS  (32'd16),
        .ROM_BASE_ADDR(32'h4000_0000),
        .AXI_ID       (4'd0),
        // 0 (the default) disables the RAM pre-zero pass, keeping the
        // original sequencing (ST_AXI_B_WAIT → ST_DONE) intact for the
        // ROM-copy scenarios.  Overridden by tb-sd-boot-zero.
        .ZERO_BYTES   (ZERO_BYTES),
        .ZERO_AWLEN   (ZERO_AWLEN),
        .MIRROR_LOW_RAM    (MIRROR_LOW_RAM),
        .MIRROR_IMAGE_BYTES(MIRROR_IMAGE_BYTES),
        .VRAM_ZERO_BASE    (VRAM_ZERO_BASE),
        .VRAM_ZERO_BYTES   (VRAM_ZERO_BYTES),
        .WORD_FIFO_LOG2P   (WORD_FIFO_LOG2P)
    ) u_boot_fsm (
        .zero_en       (zero_en),
        .clk            (clk),
        .rst            (rst),

        // Diagnostic-only overrides — tied off to preserve existing
        // scenario behavior exactly (CMD59 sent, HS mode never engages).
        .cfg_skip_cmd59 (1'b0),
        .cfg_force_hs   (1'b0),

        .card_num_lbas  (card_num_lbas),

        .spi_cmd_valid  (bf_cmd_valid),
        .spi_cmd_ready  (bf_cmd_ready),
        .spi_cmd_data   (bf_cmd_data),
        .spi_rsp_valid  (bf_rsp_valid),
        .spi_rsp_data   (bf_rsp_data),
        .spi_cs_n       (bf_cs_n),
        .spi_fast_mode  (bf_fast_mode),
        .spi_hs_mode    (bf_hs_mode),

        .m_axi_awid     (m_axi_awid),
        .m_axi_awaddr   (m_axi_awaddr),
        .m_axi_awlen    (m_axi_awlen),
        .m_axi_awsize   (m_axi_awsize),
        .m_axi_awburst  (m_axi_awburst),
        .m_axi_awvalid  (m_axi_awvalid),
        .m_axi_awready  (m_axi_awready),

        .m_axi_wdata    (m_axi_wdata),
        .m_axi_wstrb    (m_axi_wstrb),
        .m_axi_wlast    (m_axi_wlast),
        .m_axi_wvalid   (m_axi_wvalid),
        .m_axi_wready   (m_axi_wready),

        .m_axi_bid      (m_axi_bid),
        .m_axi_bresp    (m_axi_bresp),
        .m_axi_bvalid   (m_axi_bvalid),
        .m_axi_bready   (m_axi_bready),

        .rom_loading    (rom_loading),
        .rom_loaded     (rom_loaded),
        .error          (error),
        .sd_crc_enabled (sd_crc_enabled),

        .dbg_st         (dbg_st),
        .dbg_cur_cmd    (),
        .dbg_last_r1    (dbg_last_r1),
        .dbg_last_rx    (),
        .dbg_acmd41_try (),
        .dbg_rsp_count  (),
        .dbg_sector     (dbg_sector),
        .dbg_is_sdhc    (),
        .dbg_ocr0       (),
        .dbg_err_cause  (dbg_err_cause),
        .dbg_ctrl_retry (dbg_ctrl_retry),
        .dbg_rd_crc_calc(dbg_rd_crc_calc),
        .dbg_rd_crc_recv(dbg_rd_crc_recv),
        .dbg_ctrl_err_cause(),
        .dbg_ctrl_last_real_r1(),
        .dbg_sdctrl_cur_cmd(),
        .dbg_sdctrl_lba_lat(),
        .dbg_sdctrl_last_crc7_sent(),
        .dbg_sdctrl_last_poll_cnt(),
        .dbg_attempt_log0(),
        .dbg_attempt_log1(),
        .dbg_attempt_log2(),
        .dbg_attempt_log3(),
        .dbg_attempt_log4(),
        .dbg_attempt_log5()
    );

    // ── sd_spi_mux (boot_fsm owns bus during test) ──────────────────
    sd_spi_mux u_mux (
        .boot_done      (1'b0),      // phase A throughout test
        .b_sel          (1'b0),
        // PRAM-persistence port — tied off (phase B is never entered here).
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

        // Provision port — tied off.
        .prov_cmd_valid (1'b0),
        .prov_cmd_ready (),
        .prov_cmd_data  (8'h00),
        .prov_rsp_valid (),
        .prov_rsp_data  (),
        .prov_cs_n_in   (1'b1),
        .prov_fast_mode (1'b0),
        .prov_hs_mode   (1'b0),

        // SCSI port — tied off.
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
        .spi_fast_mode  (spi_fast_mode_w),
        .spi_hs_mode    (spi_hs_mode_w)
    );

    // ── sd_spi with tight-but-sane dividers for fast sim ────────────
    // Keep half-period >= 4 core cycles so the MISO double-flop
    // synchroniser has time to settle before the sample point.
    sd_spi #(
        .SLOW_HALF (9'd4),
        .FAST_HALF (9'd4),
        .HS_HALF   (9'd4)
    ) u_spi (
        .clk       (clk),
        .rst       (rst),

        .fast_mode (spi_fast_mode_w),
        .hs_mode   (spi_hs_mode_w),

        .cs_n_in   (spi_cs_n_in),

        .cmd_valid (spi_cmd_valid),
        .cmd_ready (spi_cmd_ready),
        .cmd_data  (spi_cmd_data),

        .rsp_valid (spi_rsp_valid),
        .rsp_data  (spi_rsp_data),

        .spi_clk   (spi_clk),
        .spi_mosi  (spi_mosi),
        .spi_miso  (spi_miso),
        .spi_cs_n  (spi_cs_n)
    );

endmodule
