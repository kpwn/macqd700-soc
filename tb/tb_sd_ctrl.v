// tb_sd_ctrl.v — Verilator wrapper for the sd_ctrl focused unit test.
//
// Instantiates sd_ctrl + sd_spi wired directly (no mux, no init logic).
// The C++ testbench fronts an SD-card software model on the physical SPI
// pins and drives the command handshake + read/write streaming ports.
//
// SPI clock dividers shrunk to 4 for fast simulation; same convention
// as tb_sd_boot_top.v.
//
// ── FOUR sd_ctrl instances, one selected at a time ───────────────────
// The shipping per-request watchdog budget is ~1.007e9 core cycles for a
// 1-block request (REQ_WDOG_BASE_TICKS=245760 << REQ_WDOG_TICK_LOG2=12).
// Running the two "park until the watchdog fires" scenarios against that
// costs ~2e9 simulated cycles and would dominate the entire tb suite.
//
// Shrinking the ASSERTION is not an option — the park scenarios check the
// fire time two-sided, and that two-sided check is what pins the C++
// tb's WDOG_* mirrors to the RTL's real constants.  So instead we scale
// TIME, by instantiating extra copies of the same module with one
// watchdog parameter shortened, and pick which copy a scenario drives:
//
//   u_ship       shipping defaults.  Every functional scenario, plus the
//                slow-but-progressing negative control and the
//                boot-sized-budget arithmetic check, run here.
//
//   u_fast_tick  REQ_WDOG_TICK_LOG2 12 -> 3 (8 cycles/tick instead of
//                4096).  BASE_TICKS and BLK_SHIFT are the SHIPPING ones,
//                so the tick BUDGET is bit-identical to the shipping
//                part and only the wall-clock scale changes.  That is
//                deliberate: it means the park scenarios' two-sided
//                timing check pins the shipped REQ_WDOG_BASE_TICKS
//                itself (the number this task changes and the one a
//                future editor will fiddle with), not a test-only stand-
//                in for it.  1-block expiry: 245792*8 = 1,966,336 cycles.
//
//   u_tiny_base  REQ_WDOG_BASE_TICKS 245760 -> 1, shipping TICK_LOG2 and
//                BLK_SHIFT.  Covers the one constant u_fast_tick cannot:
//                the shipping 4096-cycle prescaler.  1-block expiry:
//                (1+32)*4096 = 135,168 cycles, so pinning it is nearly
//                free.  Between the two, all three RTL constants are
//                two-sidedly pinned to observed hardware behaviour.
//
//   u_wdog_off   u_tiny_base's parameters EXACTLY, plus REQ_WDOG_ENABLE=0.
//                The A/B twin that proves the disable knob does what it
//                says: same park scenario, ERR_WDOG on u_tiny_base and
//                none here.
//
// Only the selected instance is wired to the SPI master, and only it is
// given `go`; the others sit in S_IDLE with their watchdogs unarmed.

module tb_sd_ctrl #(
    parameter integer READ_PIPELINE = 0
) (
    input  wire        clk,
    input  wire        rst,

    // Which sd_ctrl copy the scenario is driving: 0 = u_ship (default),
    // 1 = u_fast_tick, 2 = u_tiny_base, 3 = u_wdog_off.  See the header.
    input  wire [1:0]  wdog_dut_sel,

    // 1 enables sd_ctrl's internal CRC16 validate/retry path (default 0
    // preserves every pre-existing scenario's exact behaviour).
    input  wire        crc_check_en,

    // ── Command request ──────────────────────────────────────────────
    input  wire [2:0]  cmd_type,
    input  wire [31:0] lba,
    input  wire [15:0] block_count,
    input  wire        go,

    // ── Read byte stream ─────────────────────────────────────────────
    output wire        rd_valid,
    output wire [7:0]  rd_data,
    input  wire        rd_ready,

    // ── Write byte stream ────────────────────────────────────────────
    output wire        wr_ready,
    output wire        wr_valid,
    input  wire [7:0]  wr_data,
    // Producer-availability back-pressure.  Every pre-existing scenario
    // drives this high, so their behaviour is unchanged; the dedicated
    // wr_avail stall scenario in tb_sd_ctrl.cpp drives it low mid-block
    // and proves the engine actually pauses instead of shipping stale
    // bytes to the card.
    input  wire        wr_avail,

    // ── Status ───────────────────────────────────────────────────────
    output wire        busy,
    output wire        done,
    output wire        error,
    output wire [3:0]  err_cause,

    // ── Physical SPI pins (driven by the software SD model) ──────────
    output wire        spi_clk,
    output wire        spi_mosi,
    input  wire        spi_miso,
    output wire        spi_cs_n,

    // ── Caller-side CS override ──────────────────────────────────────
    input  wire        cs_n_in,

    // Test-only visibility for exhaustive reset landing-point coverage.
    output wire [5:0]  dbg_state,
    output wire [1:0]  dbg_byte_state,
    output wire [9:0]  dbg_byte_idx,
    output wire        dbg_byte_done,
    output wire [7:0]  dbg_byte_rsp,
    output wire [7:0]  dbg_last_write_resp,
    output wire [15:0] dbg_block_idx
);

    // sd_ctrl ↔ sd_spi wiring (the single shared SPI master; the
    // selected instance owns it, the other two see a dead interface).
    wire       cmd_valid;
    wire       cmd_ready;
    wire [7:0] cmd_data;
    wire       rsp_valid;
    wire [7:0] rsp_data;

    localparam [1:0] SEL_SHIP = 2'd0,
                     SEL_FAST = 2'd1,
                     SEL_TINY = 2'd2,
                     SEL_WOFF = 2'd3;

    wire sel_ship = (wdog_dut_sel == SEL_SHIP);
    wire sel_fast = (wdog_dut_sel == SEL_FAST);
    wire sel_tiny = (wdog_dut_sel == SEL_TINY);
    wire sel_woff = (wdog_dut_sel == SEL_WOFF);

    // Per-instance fan-out / fan-in.  `go` is gated so an unselected
    // instance never starts a request it could not complete.
    wire       s_rd_valid,  f_rd_valid,  t_rd_valid,  w_rd_valid;
    wire [7:0] s_rd_data,   f_rd_data,   t_rd_data,   w_rd_data;
    wire       s_wr_ready,  f_wr_ready,  t_wr_ready,  w_wr_ready;
    wire       s_wr_valid,  f_wr_valid,  t_wr_valid,  w_wr_valid;
    wire       s_busy,      f_busy,      t_busy,      w_busy;
    wire       s_done,      f_done,      t_done,      w_done;
    wire       s_error,     f_error,     t_error,     w_error;
    wire [3:0] s_err_cause, f_err_cause, t_err_cause, w_err_cause;
    wire       s_cmd_valid, f_cmd_valid, t_cmd_valid, w_cmd_valid;
    wire [7:0] s_cmd_data,  f_cmd_data,  t_cmd_data,  w_cmd_data;

    assign rd_valid  = sel_fast ? f_rd_valid  : sel_tiny ? t_rd_valid  : sel_woff ? w_rd_valid  : s_rd_valid;
    assign rd_data   = sel_fast ? f_rd_data   : sel_tiny ? t_rd_data   : sel_woff ? w_rd_data   : s_rd_data;
    assign wr_ready  = sel_fast ? f_wr_ready  : sel_tiny ? t_wr_ready  : sel_woff ? w_wr_ready  : s_wr_ready;
    assign wr_valid  = sel_fast ? f_wr_valid  : sel_tiny ? t_wr_valid  : sel_woff ? w_wr_valid  : s_wr_valid;
    assign busy      = sel_fast ? f_busy      : sel_tiny ? t_busy      : sel_woff ? w_busy      : s_busy;
    assign done      = sel_fast ? f_done      : sel_tiny ? t_done      : sel_woff ? w_done      : s_done;
    assign error     = sel_fast ? f_error     : sel_tiny ? t_error     : sel_woff ? w_error     : s_error;
    assign err_cause = sel_fast ? f_err_cause : sel_tiny ? t_err_cause : sel_woff ? w_err_cause : s_err_cause;

    assign cmd_valid = sel_fast ? f_cmd_valid : sel_tiny ? t_cmd_valid : sel_woff ? w_cmd_valid : s_cmd_valid;
    assign cmd_data  = sel_fast ? f_cmd_data  : sel_tiny ? t_cmd_data  : sel_woff ? w_cmd_data  : s_cmd_data;

    assign dbg_state = sel_fast ? u_fast_tick.st :
                       sel_tiny ? u_tiny_base.st :
                       sel_woff ? u_wdog_off.st : u_ship.st;
    assign dbg_byte_state = sel_fast ? u_fast_tick.bst :
                            sel_tiny ? u_tiny_base.bst :
                            sel_woff ? u_wdog_off.bst : u_ship.bst;
    assign dbg_byte_idx = sel_fast ? u_fast_tick.byte_idx :
                          sel_tiny ? u_tiny_base.byte_idx :
                          sel_woff ? u_wdog_off.byte_idx : u_ship.byte_idx;
    assign dbg_byte_done = sel_fast ? u_fast_tick.bdone :
                           sel_tiny ? u_tiny_base.bdone :
                           sel_woff ? u_wdog_off.bdone : u_ship.bdone;
    assign dbg_byte_rsp = sel_fast ? u_fast_tick.br :
                          sel_tiny ? u_tiny_base.br :
                          sel_woff ? u_wdog_off.br : u_ship.br;
    assign dbg_last_write_resp = sel_fast ? u_fast_tick.dbg_last_write_resp :
                                 sel_tiny ? u_tiny_base.dbg_last_write_resp :
                                 sel_woff ? u_wdog_off.dbg_last_write_resp :
                                            u_ship.dbg_last_write_resp;
    assign dbg_block_idx = sel_fast ? u_fast_tick.dbg_block_idx :
                           sel_tiny ? u_tiny_base.dbg_block_idx :
                           sel_woff ? u_wdog_off.dbg_block_idx :
                                      u_ship.dbg_block_idx;

    // Shipping parameters — the configuration that actually ships.
    sd_ctrl #(.READ_PIPELINE(READ_PIPELINE)) u_ship (
        .clk           (clk),
        .rst           (rst),

        .crc_check_en  (crc_check_en),
        .cmd_type      (cmd_type),
        .lba           (lba),
        .block_count   (block_count),
        .go            (go & sel_ship),

        .rd_valid      (s_rd_valid),
        .rd_data       (s_rd_data),
        .rd_ready      (rd_ready),

        .wr_ready      (s_wr_ready),
        .wr_valid      (s_wr_valid),
        .wr_data       (wr_data),
        .wr_avail      (wr_avail),

        .spi_cmd_valid (s_cmd_valid),
        .spi_cmd_ready (cmd_ready & sel_ship),
        .spi_cmd_data  (s_cmd_data),
        .spi_rsp_valid (rsp_valid & sel_ship),
        .spi_rsp_data  (rsp_data),

        .busy          (s_busy),
        .done          (s_done),
        .error         (s_error),
        .err_cause     (s_err_cause),

        .dbg_rd_crc_calc(),
        .dbg_rd_crc_recv(),
        .dbg_last_real_r1(),
        .dbg_cur_cmd(),
        .dbg_lba_lat(),
        .dbg_last_crc7_sent(),
        .dbg_last_write_resp(),
        .dbg_block_idx(),
        .dbg_last_poll_cnt()
    );

    // Shipping tick BUDGET, 512x shorter prescaler.  Pins the shipped
    // REQ_WDOG_BASE_TICKS / REQ_WDOG_BLK_SHIFT.
    sd_ctrl #(
        .READ_PIPELINE (READ_PIPELINE),
        .REQ_WDOG_TICK_LOG2 (3)
    ) u_fast_tick (
        .clk           (clk),
        .rst           (rst),

        .crc_check_en  (crc_check_en),
        .cmd_type      (cmd_type),
        .lba           (lba),
        .block_count   (block_count),
        .go            (go & sel_fast),

        .rd_valid      (f_rd_valid),
        .rd_data       (f_rd_data),
        .rd_ready      (rd_ready),

        .wr_ready      (f_wr_ready),
        .wr_valid      (f_wr_valid),
        .wr_data       (wr_data),
        .wr_avail      (wr_avail),

        .spi_cmd_valid (f_cmd_valid),
        .spi_cmd_ready (cmd_ready & sel_fast),
        .spi_cmd_data  (f_cmd_data),
        .spi_rsp_valid (rsp_valid & sel_fast),
        .spi_rsp_data  (rsp_data),

        .busy          (f_busy),
        .done          (f_done),
        .error         (f_error),
        .err_cause     (f_err_cause),

        .dbg_rd_crc_calc(),
        .dbg_rd_crc_recv(),
        .dbg_last_real_r1(),
        .dbg_cur_cmd(),
        .dbg_lba_lat(),
        .dbg_last_crc7_sent(),
        .dbg_last_write_resp(),
        .dbg_block_idx(),
        .dbg_last_poll_cnt()
    );

    // Shipping prescaler + per-block shift, base collapsed to 1 tick.
    // Pins the shipped REQ_WDOG_TICK_LOG2.
    sd_ctrl #(
        .REQ_WDOG_BASE_TICKS (22'd1)
    ) u_tiny_base (
        .clk           (clk),
        .rst           (rst),

        .crc_check_en  (crc_check_en),
        .cmd_type      (cmd_type),
        .lba           (lba),
        .block_count   (block_count),
        .go            (go & sel_tiny),

        .rd_valid      (t_rd_valid),
        .rd_data       (t_rd_data),
        .rd_ready      (rd_ready),

        .wr_ready      (t_wr_ready),
        .wr_valid      (t_wr_valid),
        .wr_data       (wr_data),
        .wr_avail      (wr_avail),

        .spi_cmd_valid (t_cmd_valid),
        .spi_cmd_ready (cmd_ready & sel_tiny),
        .spi_cmd_data  (t_cmd_data),
        .spi_rsp_valid (rsp_valid & sel_tiny),
        .spi_rsp_data  (rsp_data),

        .busy          (t_busy),
        .done          (t_done),
        .error         (t_error),
        .err_cause     (t_err_cause),

        .dbg_rd_crc_calc(),
        .dbg_rd_crc_recv(),
        .dbg_last_real_r1(),
        .dbg_cur_cmd(),
        .dbg_lba_lat(),
        .dbg_last_crc7_sent(),
        .dbg_last_write_resp(),
        .dbg_block_idx(),
        .dbg_last_poll_cnt()
    );

    // u_tiny_base's TWIN, with the watchdog ESCAPE disabled.  Identical in
    // every other parameter, so a scenario that parks the transfer fires
    // ERR_WDOG on SEL_TINY and must NOT fire here.  That pairing is the
    // positive control for REQ_WDOG_ENABLE: without it, "no ERR_WDOG" is
    // indistinguishable from "the scenario never stalled in the first
    // place" — which is exactly how a disable knob silently becomes a
    // no-op and takes its own coverage down with it.
    sd_ctrl #(
        .REQ_WDOG_BASE_TICKS (22'd1),
        .REQ_WDOG_ENABLE     (0)
    ) u_wdog_off (
        .clk           (clk),
        .rst           (rst),

        .crc_check_en  (crc_check_en),
        .cmd_type      (cmd_type),
        .lba           (lba),
        .block_count   (block_count),
        .go            (go & sel_woff),

        .rd_valid      (w_rd_valid),
        .rd_data       (w_rd_data),
        .rd_ready      (rd_ready),

        .wr_ready      (w_wr_ready),
        .wr_valid      (w_wr_valid),
        .wr_data       (wr_data),
        .wr_avail      (wr_avail),

        .spi_cmd_valid (w_cmd_valid),
        .spi_cmd_ready (cmd_ready & sel_woff),
        .spi_cmd_data  (w_cmd_data),
        .spi_rsp_valid (rsp_valid & sel_woff),
        .spi_rsp_data  (rsp_data),

        .busy          (w_busy),
        .done          (w_done),
        .error         (w_error),
        .err_cause     (w_err_cause),

        .dbg_rd_crc_calc(),
        .dbg_rd_crc_recv(),
        .dbg_last_real_r1(),
        .dbg_cur_cmd(),
        .dbg_lba_lat(),
        .dbg_last_crc7_sent(),
        .dbg_last_write_resp(),
        .dbg_block_idx(),
        .dbg_last_poll_cnt()
    );

    sd_spi #(
        .SLOW_HALF (9'd4),
        .FAST_HALF (9'd4),
        .HS_HALF   (9'd4)
    ) u_spi (
        .clk       (clk),
        // Match fpga_top: sd_ctrl may defer reset to close a live write, so
        // the shared byte engine must remain alive until busy drops.
        .rst       (rst && !busy),

        .fast_mode (1'b1),
        .hs_mode   (1'b0),

        .cs_n_in   (cs_n_in),

        .cmd_valid (cmd_valid),
        .cmd_ready (cmd_ready),
        .cmd_data  (cmd_data),

        .rsp_valid (rsp_valid),
        .rsp_data  (rsp_data),

        .spi_clk   (spi_clk),
        .spi_mosi  (spi_mosi),
        .spi_miso  (spi_miso),
        .spi_cs_n  (spi_cs_n)
    );

endmodule
