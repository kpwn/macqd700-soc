// tb_scsi_sd_e2e.v — End-to-end Verilog wrapper for SCSI → bridge → sd_ctrl →
// sd_spi → SD card chain.
//
// Instantiates:
//   - scsi (NCR 5380, TURBOSCSI_C96=0) on pb_clk
//   - sd_scsi_bridge (CDC) — pb_clk ↔ core_clk
//   - sd_ctrl (CMD17/24 etc.) on core_clk
//   - sd_spi (SPI master byte-stream) on core_clk, with shrunken SPI dividers
//     so each byte transfers in a few core cycles for fast simulation.
//
// The C++ testbench drives:
//   - scsi.v's pb_addr / pb_wdata / pb_wr / pb_rd register interface (Mac
//     5380 selection / CDB / DATA / STATUS / MSG_IN sequence)
//   - the SPI pin-level signals (spi_clk / spi_mosi / spi_cs_n out, spi_miso
//     in) — a host-side SD card model on the SPI bytes responds to CMD17 /
//     CMD24 with R1=0x00 + 0xFE token + 512 bytes + 2 CRC.
//
// Two clocks are used and toggled at different rates by the C++ tb to
// exercise the bridge CDC.
//
// TURBOSCSI_C96 SELECTION  (`SCSI_E2E_C96)
// ════════════════════════════════════════
// The default build wires scsi.v as the bare 5380 (TURBOSCSI_C96=0) and
// the C++ driver speaks the 5380 register/REQ-ACK protocol.  That is NOT
// the path the Mac actually uses: the Q700 ROM and the 7.5.3 SCSI Manager
// drive the NCR 53C96 through the DAFB pseudo-DMA shim.  So the shipping
// combination — a MULTI-BLOCK WRITE through C96 pseudo-DMA — had no
// end-to-end coverage anywhere in the tree, which is exactly why the
// producer-side ring back-pressure in scsi.v S_DATA_OUT could be
// structurally dead (guards that only ever SET t_req) without a single
// test noticing.
//
// `SCSI_E2E_C96 builds the identical RTL stack with TURBOSCSI_C96=1;
// the C++ driver switches to the 53C96 register protocol under the same
// define.  Both builds share every source file — see the
// SCSI_SD_E2E_BUILD_RULE template in the Makefile.

module tb_scsi_sd_e2e (
    input  wire        pb_clk,
    input  wire        core_clk,
    input  wire        rst,

    // ── Peripheral-bus slave (driven by tb to scsi.v) ──────────────────
    input  wire [8:0]  pb_addr,
    input  wire [7:0]  pb_wdata,
    input  wire        pb_wr,
    input  wire        pb_rd,
    output wire [7:0]  pb_rdata,
    output wire        pb_ack,

    // ── IRQ / DRQ from scsi (informational) ────────────────────────────
    output wire        irq,
    output wire        drq,

    // ── SPI pins (driven by tb's SD-card model) ────────────────────────
    output wire        spi_clk,
    output wire        spi_mosi,
    input  wire        spi_miso,
    output wire        spi_cs_n
);

    // ── scsi ↔ bridge wires (pb side) ──────────────────────────────────
    wire [2:0]  pb_sd_cmd_type;
    wire [31:0] pb_sd_lba;
    wire [15:0] pb_sd_block_count;
    wire        pb_sd_go;
    wire        pb_sd_busy;
    wire        pb_sd_done;
    wire        pb_sd_error;
    wire        pb_sd_rd_valid;
    wire [7:0]  pb_sd_rd_data;
    wire        pb_sd_rd_ready;
    wire        pb_sd_wr_ready;
    wire [7:0]  pb_sd_wr_data;
    wire        pb_sd_wr_valid;
    wire        pb_sd_wr_avail;

    // ── bridge ↔ sd_ctrl wires (core side) ─────────────────────────────
    wire [2:0]  core_sd_cmd_type;
    wire [31:0] core_sd_lba;
    wire [15:0] core_sd_block_count;
    wire        core_sd_go;
    wire        core_sd_busy;
    wire        core_sd_done;
    wire        core_sd_error;
    wire        core_sd_rd_valid;
    wire [7:0]  core_sd_rd_data;
    wire        core_sd_rd_ready;
    wire        core_sd_wr_ready;
    wire [7:0]  core_sd_wr_data;
    wire        core_sd_wr_valid;
    wire        core_sd_wr_avail;
    wire [3:0]  core_sd_err_cause;

    // ── sd_ctrl ↔ sd_spi wires ─────────────────────────────────────────
    wire        spi_cmd_valid;
    wire        spi_cmd_ready;
    wire [7:0]  spi_cmd_data;
    wire        spi_rsp_valid;
    wire [7:0]  spi_rsp_data;
    /* verilator lint_off UNUSEDSIGNAL */
    wire        scsi_dma_rd_ready_unused;
    wire        scsi_dma_wr_ready_unused;
    /* verilator lint_on UNUSEDSIGNAL */

    // ══════════════════════════════════════════════════════════════════
    // SCSI target on pb_clk
    // ══════════════════════════════════════════════════════════════════
    // scsi.v + its SD virtual-HDD provider, composed by tb_scsi_vhdd_sd.v.
    // The SD_LBA_BIAS=8192 reserved window now lives in vhdd_sd.v; the
    // end-to-end byte checks below still prove it end to end.
    // Build-time front-end selection — see the header comment.
`ifdef SCSI_E2E_C96
    localparam TB_TURBOSCSI_C96 = 1'b1;
`else
    localparam TB_TURBOSCSI_C96 = 1'b0;
`endif

    // READ-AHEAD SELECTION  (`SCSI_E2E_READAHEAD)
    // Same RTL stack, same C++ driver, same assertions -- only the cache
    // is switched on.  The point is to prove the whole chain still moves
    // the right bytes when the SD traffic underneath is one CMD18 per run
    // instead of one CMD17 per block.  RA_BLOCKS is small here on purpose:
    // the run length does not change what is being tested, and a long one
    // would multiply the SPI byte count (and the run time) for nothing.
`ifdef SCSI_E2E_READAHEAD
    localparam TB_READAHEAD = 1;
`else
    localparam TB_READAHEAD = 0;
`endif

`ifdef SCSI_E2E_PRODUCTION
    localparam TB_RA_BLOCKS = 32;
`else
    localparam TB_RA_BLOCKS = 4;
`endif

`ifdef SCSI_E2E_PRODUCTION
`ifndef SCSI_E2E_LEGACY_PACE
    localparam TB_RA_ADAPTIVE = 1;
`else
    localparam TB_RA_ADAPTIVE = 0;
`endif
`else
    localparam TB_RA_ADAPTIVE = 0;
`endif

    tb_scsi_vhdd_sd #(
        .TARGET_ID     (3'd0),
        .SD_LBA_BIAS   (32'd8192),
        .TURBOSCSI_C96 (TB_TURBOSCSI_C96),
        .READAHEAD     (TB_READAHEAD),
        .RA_BLOCKS     (TB_RA_BLOCKS),
        .RA_ADAPTIVE   (TB_RA_ADAPTIVE)
    ) u_scsi (
        // Write-protect is a vhdd_ctrl runtime setting (CTRL bit 2), not a
        // property of the SCSI target; these harnesses predate it and test
        // the writable behaviour, so tie it off.
        .wprot(1'b0),
        .disk_num_lbas (32'd1048576),
        .clk           (pb_clk),
        .rst           (rst),

        .pb_addr       (pb_addr),
        // This harness drives whole byte accesses; nothing here splits a
        // host 16-bit pseudo-DMA access into two beats, so the low-half
        // flag is tied low (the historical behaviour of this port before
        // tb_scsi_vhdd_sd.v exposed it for the SCSI differential fuzzer).
        .pb_dma16_lo_beat(1'b0),
        .pb_wdata      (pb_wdata),
        .pb_wr         (pb_wr),
        .pb_rd         (pb_rd),
        .pb_rdata      (pb_rdata),
        .pb_ack        (pb_ack),

        .irq           (irq),
        .drq           (drq),

        // DRQ-gate mirrors for peripheral_bus's DMA-shim pulse gating —
        // unused here (bare-5380 path, TURBOSCSI_C96=0 keeps them
        // constant 1); connected explicitly to satisfy PINMISSING lint.
        .dma_rd_ready  (scsi_dma_rd_ready_unused),
        .dma_wr_ready  (scsi_dma_wr_ready_unused),

        .sd_cmd_type    (pb_sd_cmd_type),
        .sd_lba         (pb_sd_lba),
        .sd_block_count (pb_sd_block_count),
        .sd_go          (pb_sd_go),
        .sd_busy        (pb_sd_busy),
        .sd_done        (pb_sd_done),
        .sd_error       (pb_sd_error),
        .sd_rd_valid    (pb_sd_rd_valid),
        .sd_rd_data     (pb_sd_rd_data),
        .sd_rd_ready    (pb_sd_rd_ready),
        .sd_wr_ready    (pb_sd_wr_ready),
        .sd_wr_valid    (pb_sd_wr_valid),
        .sd_wr_data     (pb_sd_wr_data),
        .sd_wr_avail    (pb_sd_wr_avail),
        // TurboSCSI DAFB shim ctrl register — only consumed when
        // TURBOSCSI_C96=1; the bare-5380 path ignores it.  Tie low.
        .scsi_ctrl_in   (9'd0)
    );

    // ══════════════════════════════════════════════════════════════════
    // Bridge: pb_clk ↔ core_clk CDC
    // ══════════════════════════════════════════════════════════════════
    sd_scsi_bridge u_bridge (
        .pb_clk          (pb_clk),
        .pb_rst          (rst),
        .pb_cmd_type     (pb_sd_cmd_type),
        .pb_lba          (pb_sd_lba),
        .pb_block_count  (pb_sd_block_count),
        .pb_go           (pb_sd_go),
        .pb_busy         (pb_sd_busy),
        .pb_done         (pb_sd_done),
        .pb_error        (pb_sd_error),
        .pb_rd_valid     (pb_sd_rd_valid),
        .pb_rd_data      (pb_sd_rd_data),
        .pb_rd_ready     (pb_sd_rd_ready),
        .pb_wr_ready     (pb_sd_wr_ready),
        .pb_wr_data      (pb_sd_wr_data),
        .pb_wr_avail     (pb_sd_wr_avail),

        .core_clk        (core_clk),
        .core_rst        (rst),
        .core_cmd_type   (core_sd_cmd_type),
        .core_lba        (core_sd_lba),
        .core_block_count(core_sd_block_count),
        .core_go         (core_sd_go),
        .core_busy       (core_sd_busy),
        .core_done       (core_sd_done),
        .core_error      (core_sd_error),
        .core_rd_valid   (core_sd_rd_valid),
        .core_rd_data    (core_sd_rd_data),
        .core_rd_ready   (core_sd_rd_ready),
        .core_wr_ready   (core_sd_wr_ready),
        .core_wr_data    (core_sd_wr_data),
        .core_wr_avail   (core_sd_wr_avail)
    );

    // pb-side: scsi.v drives sd_wr_valid as informational; the bridge does
    // not transport it.  The pb-side rd_ready is tied high by scsi.v.
    // No connection needed from sd_wr_valid here.
    /* verilator lint_off UNUSED */
    wire _unused_pb_sd_wr_valid = pb_sd_wr_valid;
    /* verilator lint_on UNUSED */

    // ══════════════════════════════════════════════════════════════════
    // sd_ctrl + sd_spi on core_clk
    // ══════════════════════════════════════════════════════════════════
    sd_ctrl #(
`ifdef SCSI_E2E_PRODUCTION
`ifndef SCSI_E2E_LEGACY_PACE
        .RD_FLUSH_PACE_CYCLES(16),
        .READ_PIPELINE(1),
`endif
`endif
`ifdef SCSI_E2E_CMD25
`ifdef SCSI_E2E_WRITE_STAGE
        .WRITE_STAGE(1),
`endif
        .MULTI_WRITE_AS_CMD24(0)
`else
        .MULTI_WRITE_AS_CMD24(1)
`endif
    ) u_sd_ctrl (
        .clk           (core_clk),
        .rst           (rst),

        // Production-shaped tests validate CRC across the entire path;
        // legacy variants retain the original CRC-disabled configuration.
`ifdef SCSI_E2E_PRODUCTION
        .crc_check_en  (1'b1),
`else
        .crc_check_en  (1'b0),
`endif
        .cmd_type      (core_sd_cmd_type),
        .lba           (core_sd_lba),
        .block_count   (core_sd_block_count),
        .go            (core_sd_go),

        .rd_valid      (core_sd_rd_valid),
        .rd_data       (core_sd_rd_data),
        .rd_ready      (core_sd_rd_ready),

        .wr_ready      (core_sd_wr_ready),
        .wr_valid      (core_sd_wr_valid),
        .wr_data       (core_sd_wr_data),
        .wr_avail      (core_sd_wr_avail),

        .spi_cmd_valid (spi_cmd_valid),
        .spi_cmd_ready (spi_cmd_ready),
        .spi_cmd_data  (spi_cmd_data),
        .spi_rsp_valid (spi_rsp_valid),
        .spi_rsp_data  (spi_rsp_data),

        .busy          (core_sd_busy),
        .done          (core_sd_done),
        .error         (core_sd_error),
        .err_cause     (core_sd_err_cause),
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

    /* verilator lint_off UNUSED */
    wire _unused_err_cause = |core_sd_err_cause;
    wire _unused_core_wr_valid = core_sd_wr_valid;
    /* verilator lint_on UNUSED */

    // Legacy variants retain 25 MHz SPI. Production variants match hardware:
    // divider=2 gives 50 MHz at core=200 MHz and 25 MHz at core=100 MHz.
`ifdef SCSI_E2E_CORE100
    localparam [8:0] TB_SPI_HALF = 9'd2;
`else
`ifdef SCSI_E2E_PRODUCTION
    localparam [8:0] TB_SPI_HALF = 9'd2;
`else
    localparam [8:0] TB_SPI_HALF = 9'd4;
`endif
`endif
    sd_spi #(
        .SLOW_HALF (TB_SPI_HALF),
        .FAST_HALF (TB_SPI_HALF),
        .HS_HALF   (TB_SPI_HALF)
    ) u_sd_spi (
        .clk       (core_clk),
        .rst       (rst),

        .fast_mode (1'b1),
        .hs_mode   (1'b0),

        .cs_n_in   (1'b0),    // CS held asserted throughout the tb

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
