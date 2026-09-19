// sd_jtag_writer.v — AXI-Lite front-end for SD-card sector writes over JTAG.
//
// Buffers one 512-byte sector via a small register aperture, then issues a
// CMD24 WRITE_SINGLE_BLOCK through sd_ctrl.  The SD card must already be in
// SPI mode and initialised by the boot path; writes are accepted only after
// boot_done is asserted.
//
// Register map, byte offsets:
//   0x00 LBA     R/W target sector
//   0x04 CTRL    W   0x5D000001 starts a write when idle and boot_done=1
//   0x08 STATUS  R   bit0 busy, bit1 done-sticky, bit2 error,
//                    bits[7:4] err_cause, bits[15:8] last_lba[7:0]
//   0x0C BUFPTR  R/W byte pointer, 0..511
//   0x10 BUFDATA R/W byte stream window, auto-increments per byte lane
//   0x14 IDENT   R   0x5D7A0001
//
// Verilog-2005, synchronous active-high rst.

`default_nettype none

module sd_jtag_writer (
    input  wire        clk,
    input  wire        rst,
    input  wire        boot_done,

    // 32-bit AXI-Lite slave face.
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

    // SPI byte interface to sd_spi_mux provision port.
    output wire        spi_cmd_valid,
    input  wire        spi_cmd_ready,
    output wire [7:0]  spi_cmd_data,
    input  wire        spi_rsp_valid,
    input  wire [7:0]  spi_rsp_data,
    output wire        spi_cs_n_in,
    output wire        spi_fast_mode,
    output wire        spi_hs_mode,

    output wire        writer_busy
);

    localparam [2:0] CT_CMD24 = 3'd3;

    localparam [5:0]
        REG_LBA     = 6'h00,
        REG_CTRL    = 6'h04,
        REG_STATUS  = 6'h08,
        REG_BUFPTR  = 6'h0C,
        REG_BUFDATA = 6'h10,
        REG_IDENT   = 6'h14;

    localparam [31:0] CTRL_GO_MAGIC = 32'h5D00_0001;
    localparam [31:0] IDENT_VALUE   = 32'h5D7A_0001;

    // 2026-05-22 — force BRAM (saves ~64 LUTs at the cost of 1 BRAM18)
    (* ram_style = "block" *) reg [7:0] sector_buf [0:511];

    reg [31:0] lba_q;
    reg [8:0]  bufptr_q;
    reg [31:0] last_lba_q;
    reg        done_sticky_q;
    reg        error_q;
    reg [3:0]  err_cause_q;

    reg        aw_pending_q;
    reg [19:0] awaddr_q;
    reg        w_pending_q;
    reg [31:0] wdata_q;
    reg [3:0]  wstrb_q;
    reg        bvalid_q;

    reg        rvalid_q;
    reg [31:0] rdata_q;

    reg        start_q;
    reg        ctrl_go_q;
    wire       ctrl_busy;
    wire       ctrl_done;
    wire       ctrl_error;
    wire [3:0] ctrl_err_cause;
    wire       ctrl_wr_ready;
    wire       ctrl_wr_valid_unused;
    reg [7:0]  ctrl_wr_data_q;
    reg [9:0]  wr_index_q;

    wire        idle = !start_q && !ctrl_busy;
    wire [5:0]  wr_reg = awaddr_q[5:0] & 6'h3C;
    wire [5:0]  rd_reg = s_araddr[5:0] & 6'h3C;
    wire [31:0] status_value =
        {16'h0000, last_lba_q[7:0], err_cause_q, error_q, done_sticky_q,
         writer_busy};

    integer reset_i;

    assign s_awready = !aw_pending_q && !bvalid_q;
    assign s_wready  = !w_pending_q && !bvalid_q;
    assign s_bresp   = 2'b00;
    assign s_bvalid  = bvalid_q;

    assign s_arready = !rvalid_q;
    assign s_rdata   = rdata_q;
    assign s_rresp   = 2'b00;
    assign s_rvalid  = rvalid_q;

    assign writer_busy   = start_q || ctrl_busy;
    assign spi_cs_n_in   = !writer_busy;
    assign spi_fast_mode = 1'b1;
    assign spi_hs_mode   = 1'b1;

    always @(posedge clk) begin
        if (rst) begin
            lba_q        <= 32'h0000_0000;
            bufptr_q     <= 9'd0;
            last_lba_q   <= 32'h0000_0000;
            done_sticky_q <= 1'b0;
            error_q      <= 1'b0;
            err_cause_q  <= 4'd0;
            aw_pending_q <= 1'b0;
            awaddr_q     <= 20'h00000;
            w_pending_q  <= 1'b0;
            wdata_q      <= 32'h0000_0000;
            wstrb_q      <= 4'h0;
            bvalid_q     <= 1'b0;
            rvalid_q     <= 1'b0;
            rdata_q      <= 32'h0000_0000;
            start_q      <= 1'b0;
            ctrl_go_q    <= 1'b0;
            ctrl_wr_data_q <= 8'hFF;
            wr_index_q   <= 10'd0;
            for (reset_i = 0; reset_i < 512; reset_i = reset_i + 1)
                sector_buf[reset_i] <= 8'h00;
        end else begin
            ctrl_go_q <= 1'b0;

            if (s_awvalid && s_awready) begin
                aw_pending_q <= 1'b1;
                awaddr_q     <= s_awaddr;
            end
            if (s_wvalid && s_wready) begin
                w_pending_q <= 1'b1;
                wdata_q     <= s_wdata;
                wstrb_q     <= s_wstrb;
            end

            if (aw_pending_q && w_pending_q && !bvalid_q) begin
                aw_pending_q <= 1'b0;
                w_pending_q  <= 1'b0;
                bvalid_q     <= 1'b1;
                case (wr_reg)
                    REG_LBA: begin
                        if (wstrb_q[0]) lba_q[7:0]   <= wdata_q[7:0];
                        if (wstrb_q[1]) lba_q[15:8]  <= wdata_q[15:8];
                        if (wstrb_q[2]) lba_q[23:16] <= wdata_q[23:16];
                        if (wstrb_q[3]) lba_q[31:24] <= wdata_q[31:24];
                    end
                    REG_CTRL: begin
                        if ((wdata_q == CTRL_GO_MAGIC) && idle && boot_done) begin
                            done_sticky_q <= 1'b0;
                            error_q       <= 1'b0;
                            err_cause_q   <= 4'd0;
                            last_lba_q    <= lba_q;
                            wr_index_q    <= 10'd0;
                            ctrl_wr_data_q <= sector_buf[0];
                            ctrl_go_q     <= 1'b1;
                            start_q       <= 1'b1;
                        end else if ((wdata_q == CTRL_GO_MAGIC) && idle && !boot_done) begin
                            done_sticky_q <= 1'b1;
                            error_q       <= 1'b1;
                            err_cause_q   <= 4'hF;
                        end
                    end
                    REG_BUFPTR: begin
                        if (wstrb_q[0]) bufptr_q[7:0] <= wdata_q[7:0];
                        if (wstrb_q[1]) bufptr_q[8]   <= wdata_q[8];
                    end
                    REG_BUFDATA: begin
                        if (wstrb_q[0]) begin
                            sector_buf[bufptr_q] <= wdata_q[7:0];
                            bufptr_q <= bufptr_q + 9'd1;
                        end
                        if (wstrb_q[1]) begin
                            sector_buf[bufptr_q + (wstrb_q[0] ? 9'd1 : 9'd0)] <= wdata_q[15:8];
                            bufptr_q <= bufptr_q + {8'd0, wstrb_q[0]} + 9'd1;
                        end
                        if (wstrb_q[2]) begin
                            sector_buf[bufptr_q + {7'd0, wstrb_q[0] + wstrb_q[1]}] <= wdata_q[23:16];
                            bufptr_q <= bufptr_q + {7'd0, wstrb_q[0] + wstrb_q[1]} + 9'd1;
                        end
                        if (wstrb_q[3]) begin
                            sector_buf[bufptr_q + {7'd0, wstrb_q[0] + wstrb_q[1] + wstrb_q[2]}] <= wdata_q[31:24];
                            bufptr_q <= bufptr_q + {7'd0, wstrb_q[0] + wstrb_q[1] + wstrb_q[2]} + 9'd1;
                        end
                    end
                    default: ;
                endcase
            end

            if (bvalid_q && s_bready) bvalid_q <= 1'b0;

            if (s_arvalid && s_arready) begin
                rvalid_q <= 1'b1;
                case (rd_reg)
                    REG_LBA:     rdata_q <= lba_q;
                    REG_STATUS:  rdata_q <= status_value;
                    REG_BUFPTR:  rdata_q <= {23'd0, bufptr_q};
                    REG_BUFDATA: begin
                        rdata_q <= {24'd0, sector_buf[bufptr_q]};
                        bufptr_q <= bufptr_q + 9'd1;
                    end
                    REG_IDENT:   rdata_q <= IDENT_VALUE;
                    default:     rdata_q <= 32'h0000_0000;
                endcase
            end else if (rvalid_q && s_rready) begin
                rvalid_q <= 1'b0;
            end

            if (start_q) start_q <= 1'b0;

            if (ctrl_wr_ready) begin
                if (wr_index_q < 10'd511) begin
                    wr_index_q <= wr_index_q + 10'd1;
                    ctrl_wr_data_q <= sector_buf[wr_index_q + 10'd1];
                end
            end

            if (ctrl_done) begin
                done_sticky_q <= 1'b1;
                error_q       <= ctrl_error;
                err_cause_q   <= ctrl_err_cause;
            end
        end
    end

    sd_ctrl u_sd_ctrl (
        .clk           (clk),
        .rst           (rst),
        // crc_check_en is a READ-side validate/retry gate; this writer
        // only ever issues CMD24, and writes always send a real computed
        // CRC16 regardless.  Low = the legacy behaviour this module has
        // always had.
        .crc_check_en  (1'b0),
        .cmd_type      (CT_CMD24),
        .lba           (lba_q),
        .block_count   (16'd1),
        .go            (ctrl_go_q),
        .rd_valid      (),
        .rd_data       (),
        .rd_ready      (1'b1),
        .wr_ready      (ctrl_wr_ready),
        .wr_valid      (ctrl_wr_valid_unused),
        .wr_data       (ctrl_wr_data_q),
        // wr_avail: this caller always has the byte staged before the
        // engine asks for it, so it takes the legacy unpaced write
        // stream unchanged.  See rtl/vhdd.vh / sd_ctrl.v header.
        .wr_avail      (1'b1),
        .spi_cmd_valid (spi_cmd_valid),
        .spi_cmd_ready (spi_cmd_ready),
        .spi_cmd_data  (spi_cmd_data),
        .spi_rsp_valid (spi_rsp_valid),
        .spi_rsp_data  (spi_rsp_data),
        .busy          (ctrl_busy),
        .done          (ctrl_done),
        .error         (ctrl_error),
        .err_cause     (ctrl_err_cause),
        // Debug taps — unused here; connected explicitly (same style as
        // rtl/soc/pram_sd.v) so PINMISSING stays clean when sd_ctrl grows
        // a port.
        /* verilator lint_off PINCONNECTEMPTY */
        .dbg_rd_crc_calc   (),
        .dbg_rd_crc_recv   (),
        .dbg_last_real_r1  (),
        .dbg_cur_cmd       (),
        .dbg_lba_lat       (),
        .dbg_last_crc7_sent(),
        .dbg_last_write_resp(),
        .dbg_block_idx     (),
        .dbg_last_poll_cnt ()
        /* verilator lint_on PINCONNECTEMPTY */
    );

endmodule

`default_nettype wire
