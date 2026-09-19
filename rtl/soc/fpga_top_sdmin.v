// fpga_top_sdmin.v — standalone real-HW SD-card + CRC16 test harness.
//
// Purpose: the full fpga_top build (DDR4 MIG + HDMI + the whole CPU) takes
// 35-45 minutes per synth+impl+bitstream cycle, which is far too slow for
// iterating on the CMD18/CRC16 boot-blocker investigation (2026-07-23).
// This is a minimal, separate top-level module that instantiates ONLY
// boot_fsm + sd_ctrl + sd_spi + a trivial always-ready AXI4 write sink
// (data content is irrelevant here — only whether the SD read completes,
// where it fails, and the raw CRC16 values matter) + a debug_vio core for
// control/status, with no DDR4/HDMI/CPU/peripheral-bus dependency at all.
//
// Because there is no DDR4 MIG calibration and no OoO CPU to place/route,
// this should synthesize+implement+bitstream in well under 5 minutes,
// letting many hypotheses be tested from ONE flashed bitstream: VIO
// probe_out0 bits select cfg_skip_cmd59 / cfg_force_hs (boot_fsm.v's
// diagnostic overrides) and a soft-reset bit restarts the whole SD-read
// sequence on demand — no reflash needed between experiments.
//
// Clock: reuses the board's DDR4-reference sys_clk_p/n pair (LVDS,
// DIFF_SSTL12, bank 66) as a plain fabric clock via IBUFDS+BUFG — the
// same primitive pairing fpga_top_clocks.vh already uses for SIM_MODEL
// builds (real-MIG builds instead route this pair to MIG's own C0 clock
// and take the fabric clock from a separate MGTREFCLK pair, which this
// harness has no need for since it has no MIG at all).

module fpga_top_sdmin (
    input  wire        sys_clk_p,
    input  wire        sys_clk_n,
    input  wire        cpu_resetn,

    output wire        sd_clk,
    output wire        sd_mosi,
    input  wire        sd_miso,
    output wire        sd_cs_n,

    output wire [3:0]  led
);

    // ═══════════════════════════════════════════════════════════════
    // Clock
    // ═══════════════════════════════════════════════════════════════
    wire sys_clk_ibuf;
    IBUFDS u_sys_clk_ibuf (
        .I  (sys_clk_p),
        .IB (sys_clk_n),
        .O  (sys_clk_ibuf)
    );

    wire core_clk;
    BUFG u_core_clk_bufg (
        .I (sys_clk_ibuf),
        .O (core_clk)
    );

    // ═══════════════════════════════════════════════════════════════
    // Reset: power-on-reset counter (declaration-time FF init values are
    // the standard, synthesizable Xilinx idiom for this — loaded from
    // the bitstream at configuration, not a disallowed procedural
    // `initial` block) OR'd with a debounced cpu_resetn button OR'd with
    // a VIO soft-reset bit (so an experiment can be re-run without
    // reflashing or pressing a physical button).
    // ═══════════════════════════════════════════════════════════════
    reg [7:0] por_cnt = 8'hFF;
    reg       por_rst = 1'b1;
    always @(posedge core_clk) begin
        if (por_cnt != 8'd0) begin
            por_cnt <= por_cnt - 8'd1;
            por_rst <= 1'b1;
        end else begin
            por_rst <= 1'b0;
        end
    end

    reg [2:0] cpu_resetn_sync = 3'b111;
    always @(posedge core_clk)
        cpu_resetn_sync <= {cpu_resetn_sync[1:0], cpu_resetn};

    wire vio_soft_rst;
    wire rst = por_rst || !cpu_resetn_sync[2] || vio_soft_rst;

    // ═══════════════════════════════════════════════════════════════
    // boot_fsm + sd_ctrl (internal) + sd_spi — no sd_spi_mux needed
    // since this harness has exactly one SPI consumer.
    // ═══════════════════════════════════════════════════════════════
    wire        bf_cmd_valid;
    wire        bf_cmd_ready;
    wire [7:0]  bf_cmd_data;
    wire        bf_rsp_valid;
    wire [7:0]  bf_rsp_data;
    wire        bf_cs_n;
    wire        bf_fast_mode;
    wire        bf_hs_mode;

    wire        rom_loading, rom_loaded, boot_error, sd_crc_enabled;
    wire [5:0]  dbg_st;
    wire [3:0]  dbg_cur_cmd;
    wire [7:0]  dbg_last_r1;
    wire [7:0]  dbg_last_rx;
    wire [15:0] dbg_acmd41_try;
    wire [23:0] dbg_rsp_count;
    wire [15:0] dbg_sector;
    wire        dbg_is_sdhc;
    wire [7:0]  dbg_ocr0;
    wire [2:0]  dbg_err_cause;
    wire [3:0]  dbg_ctrl_retry;
    wire [15:0] dbg_rd_crc_calc;
    wire [15:0] dbg_rd_crc_recv;
    wire [3:0]  dbg_ctrl_err_cause;
    wire [7:0]  dbg_ctrl_last_real_r1;
    wire [3:0]  dbg_sdctrl_cur_cmd;
    wire [31:0] dbg_sdctrl_lba_lat;
    wire [7:0]  dbg_sdctrl_last_crc7_sent;
    wire [7:0]  dbg_sdctrl_last_poll_cnt;
    wire [19:0] dbg_attempt_log0, dbg_attempt_log1, dbg_attempt_log2;
    wire [19:0] dbg_attempt_log3, dbg_attempt_log4, dbg_attempt_log5;

    wire [3:0]  m_awid;
    wire [31:0] m_awaddr;
    wire [7:0]  m_awlen;
    wire [2:0]  m_awsize;
    wire [1:0]  m_awburst;
    wire        m_awvalid;
    wire        m_awready;
    wire [31:0] m_wdata;
    wire [3:0]  m_wstrb;
    wire        m_wlast;
    wire        m_wvalid;
    wire        m_wready;
    wire [3:0]  m_bid;
    wire [1:0]  m_bresp;
    wire        m_bvalid;
    wire        m_bready;

    wire        cfg_skip_cmd59, cfg_force_hs;

    boot_fsm #(
        .NUM_SECTORS  (32'd2048),   // matches the real Q700 ROM boot load
        .ROM_BASE_ADDR(32'h4000_0000),
        .AXI_ID       (4'd0),
        .ZERO_BYTES   (32'd0)       // no RAM-zero pass — nothing to zero here
    ) u_boot_fsm (
        .clk(core_clk), .rst(rst),

        .cfg_skip_cmd59(cfg_skip_cmd59),
        .cfg_force_hs  (cfg_force_hs),

        .spi_cmd_valid(bf_cmd_valid), .spi_cmd_ready(bf_cmd_ready),
        .spi_cmd_data (bf_cmd_data),
        .spi_rsp_valid(bf_rsp_valid), .spi_rsp_data (bf_rsp_data),
        .spi_cs_n     (bf_cs_n),
        .spi_fast_mode(bf_fast_mode), .spi_hs_mode(bf_hs_mode),

        .m_axi_awid(m_awid), .m_axi_awaddr(m_awaddr), .m_axi_awlen(m_awlen),
        .m_axi_awsize(m_awsize), .m_axi_awburst(m_awburst),
        .m_axi_awvalid(m_awvalid), .m_axi_awready(m_awready),
        .m_axi_wdata(m_wdata), .m_axi_wstrb(m_wstrb), .m_axi_wlast(m_wlast),
        .m_axi_wvalid(m_wvalid), .m_axi_wready(m_wready),
        .m_axi_bid(m_bid), .m_axi_bresp(m_bresp), .m_axi_bvalid(m_bvalid),
        .m_axi_bready(m_bready),

        .rom_loading(rom_loading), .rom_loaded(rom_loaded), .error(boot_error),
        .sd_crc_enabled(sd_crc_enabled),
        .card_num_lbas (),

        .dbg_st(dbg_st), .dbg_cur_cmd(dbg_cur_cmd), .dbg_last_r1(dbg_last_r1),
        .dbg_last_rx(dbg_last_rx), .dbg_acmd41_try(dbg_acmd41_try),
        .dbg_rsp_count(dbg_rsp_count), .dbg_sector(dbg_sector),
        .dbg_is_sdhc(dbg_is_sdhc), .dbg_ocr0(dbg_ocr0),
        .dbg_err_cause(dbg_err_cause), .dbg_ctrl_retry(dbg_ctrl_retry),
        .dbg_rd_crc_calc(dbg_rd_crc_calc), .dbg_rd_crc_recv(dbg_rd_crc_recv),
        .dbg_ctrl_err_cause(dbg_ctrl_err_cause),
        .dbg_ctrl_last_real_r1(dbg_ctrl_last_real_r1),
        .dbg_sdctrl_cur_cmd(dbg_sdctrl_cur_cmd),
        .dbg_sdctrl_lba_lat(dbg_sdctrl_lba_lat),
        .dbg_sdctrl_last_crc7_sent(dbg_sdctrl_last_crc7_sent),
        .dbg_sdctrl_last_poll_cnt(dbg_sdctrl_last_poll_cnt),
        .dbg_attempt_log0(dbg_attempt_log0),
        .dbg_attempt_log1(dbg_attempt_log1),
        .dbg_attempt_log2(dbg_attempt_log2),
        .dbg_attempt_log3(dbg_attempt_log3),
        .dbg_attempt_log4(dbg_attempt_log4),
        .dbg_attempt_log5(dbg_attempt_log5)
    );

    sd_spi #(
        .SLOW_HALF (250),
        .FAST_HALF (4),
        .HS_HALF   (2)
    ) u_spi (
        .clk(core_clk), .rst(rst),
        .fast_mode(bf_fast_mode), .hs_mode(bf_hs_mode),
        .cs_n_in(bf_cs_n),
        .cmd_valid(bf_cmd_valid), .cmd_ready(bf_cmd_ready), .cmd_data(bf_cmd_data),
        .rsp_valid(bf_rsp_valid), .rsp_data(bf_rsp_data),
        .spi_clk(sd_clk), .spi_mosi(sd_mosi), .spi_miso(sd_miso), .spi_cs_n(sd_cs_n)
    );

    // ═══════════════════════════════════════════════════════════════
    // Trivial always-ready AXI4 write sink. Data content is irrelevant
    // to this harness (only whether the SD read completes/where it
    // fails/the raw CRC16 values matter) — single-beat only (boot_fsm
    // never issues AWLEN>0), so AW/W are always ready. B must STAY
    // asserted until bready actually consumes it (a bug caught here
    // 2026-07-24: an earlier version re-derived b_valid_r fresh from
    // wlast every cycle instead of holding it, so if boot_fsm's own
    // bready — a registered, one-cycle-late reaction to bvalid rising,
    // see boot_fsm.v's writer_can_launch/bready logic — didn't land on
    // the exact same cycle bvalid pulsed, the B response was missed
    // entirely and boot_fsm waited forever, backing up word_fifo until
    // it hit word_fifo_full/err_cause=4). Matches tb_sd_boot.cpp's
    // AxiSlave model (b_pending stays true until bready is observed).
    // ═══════════════════════════════════════════════════════════════
    assign m_awready = 1'b1;
    assign m_wready  = 1'b1;
    assign m_bresp   = 2'b00;

    reg        b_valid_r;
    reg [3:0]  awid_latched;
    always @(posedge core_clk) begin
        if (rst) begin
            b_valid_r <= 1'b0;
        end else begin
            if (m_awvalid && m_awready) awid_latched <= m_awid;
            if (m_wvalid && m_wready && m_wlast) b_valid_r <= 1'b1;
            else if (m_bvalid && m_bready)       b_valid_r <= 1'b0;
        end
    end
    assign m_bvalid = b_valid_r;
    assign m_bid    = awid_latched;

    // ═══════════════════════════════════════════════════════════════
    // End-to-end CRC32 (zlib/IEEE 802.3 compatible) over every ROM byte
    // actually written via AXI, in real memory (big-endian) order —
    // lets the host verify the WHOLE transfer (not just one block's
    // CRC16) is bit-for-bit correct: python's zlib.crc32() over
    // files/420dbff3.rom[:NUM_SECTORS*512] should equal this readout
    // once rom_loaded fires. m_wdata's byte lanes are exactly 4
    // consecutive ROM bytes in normal MSB-first order (boot_fsm.v packs
    // {pack_b0,pack_b1,pack_b2,ctrl_rd_data} = the SD byte stream
    // in order), so feeding wdata[31:24..7:0] in that order matches the
    // real file layout directly.
    // ═══════════════════════════════════════════════════════════════
    function [31:0] crc32_step;
        input [31:0] c;
        input [7:0]  b;
        integer k;
        reg [31:0] x;
        begin
            x = c ^ {24'd0, b};
            for (k = 0; k < 8; k = k + 1)
                x = x[0] ? ((x >> 1) ^ 32'hEDB88320) : (x >> 1);
            crc32_step = x;
        end
    endfunction

    reg [31:0] rom_crc32_r;
    always @(posedge core_clk) begin
        if (rst) begin
            rom_crc32_r <= 32'hFFFFFFFF;
        end else if (m_wvalid && m_wready) begin
            rom_crc32_r <= crc32_step(crc32_step(crc32_step(crc32_step(
                rom_crc32_r, m_wdata[31:24]), m_wdata[23:16]), m_wdata[15:8]), m_wdata[7:0]);
        end
    end
    wire [31:0] rom_crc32_final = rom_crc32_r ^ 32'hFFFFFFFF;

    // ═══════════════════════════════════════════════════════════════
    // Debug VIO — control (probe_out0) + status (probe_in0..3)
    // ═══════════════════════════════════════════════════════════════
    wire [2:0] vio_ctrl;
    assign vio_soft_rst   = vio_ctrl[0];
    assign cfg_skip_cmd59 = vio_ctrl[1];
    assign cfg_force_hs   = vio_ctrl[2];

    wire [31:0] vio_diag  = {9'd0, dbg_ctrl_retry, dbg_err_cause, dbg_sector};
    wire [31:0] vio_crc   = {dbg_rd_crc_calc, dbg_rd_crc_recv};
    wire [31:0] vio_state = {6'd0, dbg_sdctrl_cur_cmd, dbg_ctrl_err_cause,
                              dbg_st, dbg_cur_cmd, dbg_last_r1};
    wire [31:0] vio_flags = {8'd0, dbg_sdctrl_last_crc7_sent,
                              dbg_ctrl_last_real_r1,
                              rom_loading, rom_loaded, boot_error,
                              sd_crc_enabled, bf_hs_mode, bf_fast_mode,
                              dbg_is_sdhc, rst};
    wire [31:0] vio_poll_cnt = {24'd0, dbg_sdctrl_last_poll_cnt};

    // Concatenation expressions wired straight to a probe_inN port don't
    // preserve a usable name for JTAG lookup-by-name (Vivado picks some
    // constituent net's name instead — the "VIO probe naming quirk" that
    // cost a rebuild+flash+test cycle earlier this session). Give each a
    // real named wire, same pattern as vio_diag/vio_crc/etc above.
    wire [31:0] vio_attempt0 = {12'd0, dbg_attempt_log0};
    wire [31:0] vio_attempt1 = {12'd0, dbg_attempt_log1};
    wire [31:0] vio_attempt2 = {12'd0, dbg_attempt_log2};
    wire [31:0] vio_attempt3 = {12'd0, dbg_attempt_log3};
    wire [31:0] vio_attempt4 = {12'd0, dbg_attempt_log4};
    wire [31:0] vio_attempt5 = {12'd0, dbg_attempt_log5};

    (* DONT_TOUCH = "true" *) sdmin_vio u_vio (
        .clk       (core_clk),
        .probe_in0 (vio_diag),
        .probe_in1 (vio_crc),
        .probe_in2 (vio_state),
        .probe_in3 (vio_flags),
        .probe_in4 (dbg_sdctrl_lba_lat),
        .probe_in5 (rom_crc32_final),
        .probe_in6 (vio_poll_cnt),
        .probe_in7 (vio_attempt0),
        .probe_in8 (vio_attempt1),
        .probe_in9 (vio_attempt2),
        .probe_in10(vio_attempt3),
        .probe_in11(vio_attempt4),
        .probe_in12(vio_attempt5),
        .probe_out0(vio_ctrl)
    );

    // ═══════════════════════════════════════════════════════════════
    // LEDs — quick visual sanity without JTAG (led[0]=heartbeat,
    // led[1]=rom_loaded, led[2]=boot_error, led[3]=sd_crc_enabled)
    // ═══════════════════════════════════════════════════════════════
    reg [23:0] heartbeat = 24'd0;
    always @(posedge core_clk) heartbeat <= heartbeat + 24'd1;
    assign led[0] = heartbeat[23];
    assign led[1] = rom_loaded;
    assign led[2] = boot_error;
    assign led[3] = sd_crc_enabled;

endmodule
