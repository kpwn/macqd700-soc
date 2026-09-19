// tb_sd_provision_top.v — wrapper for the sd_provision_core unit
// testbench (tb_sd_provision.cpp).
//
// Not for synthesis — Verilator-only harness that instantiates
// sd_provision_core with tiny sd_spi clock dividers (each SPI byte takes
// ~64 core cycles instead of the production 100 MHz timings) and the
// PRODUCTION staging RAM depth (STAGE_SECTORS=256, matching
// rtl/soc/sd_provision_top.v) so the verify-range scenarios can use the
// same 256-sector batch grid the JTAG host actually issues.  The C++
// side drives the 32-bit burst AXI face directly (standing in for the
// burst-capable JTAG-to-AXI master IP of sd_provision_top) and models
// the SD card at the physical SPI pins, init sequence included, like
// tb_sd_boot.cpp.

module tb_sd_provision_top (
    input  wire        clk,
    input  wire        rst,

    // ── 32-bit burst AXI slave face (driven by tb_sd_provision.cpp) ──
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

    // ── physical SD pins (card modelled by tb_sd_provision.cpp) ──────
    output wire        spi_clk,
    output wire        spi_mosi,
    input  wire        spi_miso,
    output wire        spi_cs_n,

    // ── status taps ──────────────────────────────────────────────────
    output wire        card_ready,
    output wire        init_error,
    output wire        writer_busy
);

    sd_provision_core #(
        .STAGE_SECTORS (256),
        .SPI_SLOW_HALF (4),
        .SPI_FAST_HALF (4),
        .SPI_HS_HALF   (4)
    ) u_core (
        .clk        (clk),
        .rst        (rst),

        .s_awaddr   (s_awaddr),
        .s_awlen    (s_awlen),
        .s_awvalid  (s_awvalid),
        .s_awready  (s_awready),
        .s_wdata    (s_wdata),
        .s_wstrb    (s_wstrb),
        .s_wlast    (s_wlast),
        .s_wvalid   (s_wvalid),
        .s_wready   (s_wready),
        .s_bresp    (s_bresp),
        .s_bvalid   (s_bvalid),
        .s_bready   (s_bready),
        .s_araddr   (s_araddr),
        .s_arlen    (s_arlen),
        .s_arvalid  (s_arvalid),
        .s_arready  (s_arready),
        .s_rdata    (s_rdata),
        .s_rresp    (s_rresp),
        .s_rlast    (s_rlast),
        .s_rvalid   (s_rvalid),
        .s_rready   (s_rready),

        .sd_clk     (spi_clk),
        .sd_mosi    (spi_mosi),
        .sd_miso    (spi_miso),
        .sd_cs_n    (spi_cs_n),

        .card_ready (card_ready),
        .init_error (init_error),
        .writer_busy(writer_busy)
    );

endmodule
