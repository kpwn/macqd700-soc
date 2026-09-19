// tb_pram_sd_top.v — wrapper for the PRAM-to-SD persistence unit tb.
//
// Wires together EXACTLY the three shipped modules that make up the
// feature, in the same two clock domains the real SoC uses:
//
//     pram_sd (core_clk) ── pram_cdc ── rtc (pb_clk)
//          │
//          └── sd_ctrl (inside pram_sd) ── SPI byte port ── tb SD card model
//
// Nothing is stubbed.  In particular the PRAM array under test is the
// REAL rtl/mac/rtc.v array, reachable from the testbench two ways:
//
//   * through pram_sd's snapshot/restore path (the thing being tested), and
//   * through rtc.v's own bit-banged serial command protocol (the
//     INDEPENDENT witness).
//
// That second path is the whole point of using the real rtc.v here.  If
// pram_sd were quietly operating on a private shadow copy instead of the
// live array, a test that both wrote and read through pram_sd would pass
// while measuring nothing.  Every content assertion in tb_pram_sd.cpp
// therefore crosses the two paths.
//
// The pb_clk input is driven by the testbench at half the core_clk rate,
// matching the shipping build (core 100 MHz / pb 50 MHz), and can be
// FROZEN by the testbench to prove pram_sd's CDC timeout is real.

`default_nettype none

module tb_pram_sd_top #(
    // Deliberately tiny relative to the shipping values so the bounded-
    // response MECHANISMS are provable in a short simulation.  See
    // pram_sd.v's parameter block for what the real budgets are.
    parameter integer ARB_WAIT_LOG2     = 10,
    parameter integer CDC_WAIT_LOG2     = 10,
    parameter integer DEFAULT_HOLD_LOG2 = 6,
    parameter integer AUTOLOAD_ON_BOOT  = 0
) (
    input  wire        core_clk,
    input  wire        core_rst,
    input  wire        pb_clk,
    input  wire        pb_rst,
    input  wire        boot_done,

    // ── AXI-Lite face of pram_sd ──────────────────────────────────────
    input  wire [19:0] s_awaddr,
    input  wire        s_awvalid,
    output wire        s_awready,
    input  wire [31:0] s_wdata,
    input  wire [3:0]  s_wstrb,
    input  wire        s_wvalid,
    output wire        s_wready,
    output wire [1:0]  s_bresp,
    output wire        s_bvalid,
    input  wire        s_bready,
    input  wire [19:0] s_araddr,
    input  wire        s_arvalid,
    output wire        s_arready,
    output wire [31:0] s_rdata,
    output wire [1:0]  s_rresp,
    output wire        s_rvalid,
    input  wire        s_rready,

    // ── SPI byte port — the tb models the SD card here ────────────────
    output wire        spi_cmd_valid,
    input  wire        spi_cmd_ready,
    output wire [7:0]  spi_cmd_data,
    input  wire        spi_rsp_valid,
    input  wire [7:0]  spi_rsp_data,
    output wire        spi_cs_n,
    output wire        spi_fast_mode,
    output wire        spi_hs_mode,

    // ── SD bus arbitration ────────────────────────────────────────────
    output wire        sd_req,
    output wire        autoload_pending,
    input  wire        sd_arb_allow,
    output wire        sd_gnt,

    // ── rtc.v's real serial command bus (the independent witness) ─────
    input  wire        rtc_enb,
    input  wire        rtc_clk,
    input  wire        rtc_data_o,
    input  wire        rtc_data_oe,
    output wire        rtc_data_i,
    input  wire        phi2_tick,
    // Direct Cmd-Opt-P-R zap, for test setup.  ORed with pram_sd's
    // defaults-fallback strobe exactly as fpga_top_peripherals.vh does.
    input  wire        rtc_pram_clear,

    // ── Observability ─────────────────────────────────────────────────
    output wire        pram_default,
    output wire        pram_sd_busy
);

    wire        pram_x_req;
    wire        pram_x_we;
    wire [7:0]  pram_x_addr;
    wire [7:0]  pram_x_wdata;
    wire [7:0]  pram_x_rdata;
    wire        pram_x_ack;
    reg         sd_gnt_q;

    // Match fpga_top_sd.vh: reset cannot revoke a grant or reset pram_sd
    // while an explicit operation still owns the card.
    assign sd_gnt = sd_gnt_q;
    always @(posedge core_clk) begin
        if (core_rst && !pram_sd_busy) begin
            sd_gnt_q <= 1'b0;
        end else if (!sd_req) begin
            sd_gnt_q <= 1'b0;
        end else if (!sd_gnt_q && boot_done && sd_arb_allow) begin
            sd_gnt_q <= 1'b1;
        end
    end

    wire [7:0]  rtc_ext_addr;
    wire [7:0]  rtc_ext_wdata;
    wire        rtc_ext_we;
    wire [7:0]  rtc_ext_rdata;

    pram_sd #(
        .AUTOLOAD_ON_BOOT  (AUTOLOAD_ON_BOOT),
        .ARB_WAIT_LOG2     (ARB_WAIT_LOG2),
        .CDC_WAIT_LOG2     (CDC_WAIT_LOG2),
        .DEFAULT_HOLD_LOG2 (DEFAULT_HOLD_LOG2)
    ) u_pram_sd (
        .clk       (core_clk),
        .rst       (core_rst && !pram_sd_busy),
        .boot_done (boot_done),
        .autoload_pending (autoload_pending),

        .s_awaddr  (s_awaddr ), .s_awvalid(s_awvalid), .s_awready(s_awready),
        .s_wdata   (s_wdata  ), .s_wstrb  (s_wstrb  ),
        .s_wvalid  (s_wvalid ), .s_wready (s_wready ),
        .s_bresp   (s_bresp  ), .s_bvalid (s_bvalid ), .s_bready (s_bready ),
        .s_araddr  (s_araddr ), .s_arvalid(s_arvalid), .s_arready(s_arready),
        .s_rdata   (s_rdata  ), .s_rresp  (s_rresp  ),
        .s_rvalid  (s_rvalid ), .s_rready (s_rready ),

        .spi_cmd_valid (spi_cmd_valid),
        .spi_cmd_ready (spi_cmd_ready),
        .spi_cmd_data  (spi_cmd_data ),
        .spi_rsp_valid (spi_rsp_valid),
        .spi_rsp_data  (spi_rsp_data ),
        .spi_cs_n_in   (spi_cs_n     ),
        .spi_fast_mode (spi_fast_mode),
        .spi_hs_mode   (spi_hs_mode  ),

        .sd_req (sd_req),
        .sd_gnt (sd_gnt),

        .pram_req   (pram_x_req  ),
        .pram_we    (pram_x_we   ),
        .pram_addr  (pram_x_addr ),
        .pram_wdata (pram_x_wdata),
        .pram_rdata (pram_x_rdata),
        .pram_ack   (pram_x_ack  ),

        .pram_default (pram_default),
        .pram_sd_busy (pram_sd_busy)
    );

    pram_cdc u_pram_cdc (
        .a_clk   (core_clk),
        .a_rst   (core_rst),
        .a_req   (pram_x_req),
        .a_we    (pram_x_we),
        .a_addr  (pram_x_addr),
        .a_wdata (pram_x_wdata),
        .a_rdata (pram_x_rdata),
        .a_ack   (pram_x_ack),
        .b_clk   (pb_clk),
        .b_rst   (pb_rst),
        .b_addr  (rtc_ext_addr),
        .b_wdata (rtc_ext_wdata),
        .b_we    (rtc_ext_we),
        .b_rdata (rtc_ext_rdata)
    );

    // Same 2-FF core_clk -> pb_clk crossing, and the same OR with the
    // operator zap, that fpga_top_peripherals.vh builds.  Reproduced here
    // rather than stubbed so the tb exercises the real path by which a
    // failed load reaches rtc.v's post-reset image.
    (* ASYNC_REG = "TRUE" *) reg pram_clear_meta;
    (* ASYNC_REG = "TRUE" *) reg pram_clear_sync;
    always @(posedge pb_clk) begin
        if (pb_rst) begin
            pram_clear_meta <= 1'b0;
            pram_clear_sync <= 1'b0;
        end else begin
            pram_clear_meta <= rtc_pram_clear | pram_default;
            pram_clear_sync <= pram_clear_meta;
        end
    end

    rtc #(
        .SEC_DIV(32'd1_000_000)
    ) u_rtc (
        .clk         (pb_clk),
        .rst         (pb_rst),
        .phi2_tick   (phi2_tick),
        .rtc_enb     (rtc_enb),
        .rtc_clk     (rtc_clk),
        .rtc_data_o  (rtc_data_o),
        .rtc_data_oe (rtc_data_oe),
        .pram_clear  (pram_clear_sync),
        .pram_ext_addr  (rtc_ext_addr ),
        .pram_ext_we    (rtc_ext_we   ),
        .pram_ext_wdata (rtc_ext_wdata),
        .pram_ext_rdata (rtc_ext_rdata),
        .cko         (),
        .rtc_data_i  (rtc_data_i)
    );

endmodule

`default_nettype wire
