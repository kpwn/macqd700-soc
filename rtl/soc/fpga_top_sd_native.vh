// Experimental TQTT full-SoC four-bit SD path. ROM boot and SCSI share one
// 100 MHz LiteSDCard backend. The boot and SCSI CDCs preserve their existing
// reset/error contracts; SCSI requests above 32 sectors are chunked before
// reaching the backend's 16 KiB DMA RAM. PRAM-on-card and JTAG provisioning
// still need native implementations before this replaces the SPI build.

    // No SPI bus exists in this build. boot_fsm skips its SPI init states.
    assign boot_spi_cmd_ready = 1'b0;
    assign boot_spi_rsp_valid = 1'b0;
    assign boot_spi_rsp_data = 8'hff;

    wire native_card_busy;
    (* ASYNC_REG = "TRUE" *) reg [1:0] native_busy_core_sync;
    always @(posedge core_clk) begin
        if (soc_full_rst_bank[5]) native_busy_core_sync <= 2'b00;
        else native_busy_core_sync <= {native_busy_core_sync[0], native_card_busy};
    end
    peripheral_reset_sequencer #(.PULSE_CYCLES(128))
    u_peripheral_reset_sequencer (
        .clk(core_clk), .rst(soc_full_rst_bank[5]),
        .reset_req(cpu_peripheral_reset),
        .storage_busy(native_busy_core_sync[1]),
        .storage_reset_req(warm_storage_reset),
        .peripheral_reset(warm_peripheral_reset)
    );

    // A debug reset can arrive during a native multi-block command. Let
    // LiteSDCard finish the on-card phase (including CMD12) before resetting
    // its controller; writes are fully staged in local RAM before CMD24/25.
    // The CDC peer-reset epoch reports an aborted request to SCSI. Capture
    // even a short reset request until the card session closes, then hold
    // the 100 MHz backend in reset for three storage clocks.
    // warm_storage_reset is an immediate request, not permission to cut
    // power to an active CMD24/CMD25. The sequencer waits for card_busy to
    // fall before pulsing warm_peripheral_reset; reset the backend then.
    // JTAG ROM bypass holds boot_fsm_rst high, but the running Mac still
    // needs its SD-backed SCSI disk. Reset the card on actual SoC/cold
    // resets, independently of whether ROM came from SD or JTAG.
    wire native_reset_req = soc_full_rst | dbg_cold_reset_hold |
                            warm_peripheral_reset;
    wire native_card_session_active;
    wire native_rst;
    sd_native_reset_guard u_native_reset_guard (
        .clk(video_ref_clk), .reset_req(native_reset_req),
        .card_session_active(native_card_session_active), .rst(native_rst)
    );

    wire native_ready, native_init_error;
    wire [31:0] native_card_num_lbas, native_rca;
    wire [4:0] native_init_phase;
    (* ASYNC_REG = "TRUE" *) reg [1:0] native_ready_core_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] native_error_core_sync;
    always @(posedge core_clk) begin
        if (boot_fsm_rst) begin
            native_ready_core_sync <= 2'b00;
            native_error_core_sync <= 2'b00;
        end else begin
            native_ready_core_sync <= {native_ready_core_sync[0], native_ready};
            native_error_core_sync <= {native_error_core_sync[0], native_init_error};
        end
    end
    assign boot_native_ready = native_ready_core_sync[1];
    assign boot_native_init_error = native_error_core_sync[1];
    // Capacity is stable from before ready rises until the next native reset.
    assign boot_native_card_lbas = native_card_num_lbas;

    (* ASYNC_REG = "TRUE" *) reg [1:0] native_boot_done_sync;
    always @(posedge video_ref_clk) begin
        if (native_rst) native_boot_done_sync <= 2'b00;
        else native_boot_done_sync <=
            {native_boot_done_sync[0],
             (boot_rom_loaded | (jtag_boot_bypass && jtag_boot_release)) &&
             native_ready};
    end

    // ROM loader (core clock) to card backend (100 MHz).
    wire [2:0] nb_cmd_type;
    wire [31:0] nb_lba;
    wire [15:0] nb_block_count;
    wire nb_go, nb_busy, nb_done, nb_error, nb_rd_valid, nb_rd_ready;
    wire [7:0] nb_rd_data;
    sd_scsi_native_bridge u_native_boot_cdc (
        .pb_clk(core_clk), .pb_rst(boot_fsm_rst),
        .pb_cmd_type(3'd2), .pb_lba(boot_native_lba),
        .pb_block_count(boot_native_block_count), .pb_go(boot_native_go),
        .pb_busy(boot_native_busy), .pb_done(boot_native_done),
        .pb_error(boot_native_error),
        .pb_rd_valid(boot_native_rd_valid),
        .pb_rd_data(boot_native_rd_data),
        .pb_rd_ready(boot_native_rd_ready),
        .pb_wr_ready(), .pb_wr_data(8'b0), .pb_wr_avail(1'b0),
        .core_clk(video_ref_clk), .core_rst(native_rst),
        .core_cmd_type(nb_cmd_type), .core_lba(nb_lba),
        .core_block_count(nb_block_count), .core_go(nb_go),
        .core_busy(nb_busy), .core_done(nb_done), .core_error(nb_error),
        .core_rd_valid(nb_rd_valid), .core_rd_data(nb_rd_data),
        .core_rd_ready(nb_rd_ready), .core_wr_ready(1'b0),
        .core_wr_data(), .core_wr_avail()
    );

    // Existing SCSI-facing interface, now with a FIFO CDC instead of the
    // SPI-rate toggle data path.
    wire [2:0] ns_cmd_type;
    wire [31:0] ns_lba;
    wire [15:0] ns_block_count;
    wire ns_go, ns_busy, ns_done, ns_error, ns_rd_valid, ns_rd_ready;
    wire [7:0] ns_rd_data, ns_wr_data;
    wire ns_wr_ready, ns_wr_avail;
    sd_scsi_native_bridge u_native_scsi_cdc (
        .pb_clk(pb_clk), .pb_rst(pb_full_rst_bank[2]),
        .pb_cmd_type(scsi_sd_cmd_type_w), .pb_lba(scsi_sd_lba_w),
        .pb_block_count(scsi_sd_block_count_w), .pb_go(scsi_sd_go_w),
        .pb_busy(scsi_sd_busy_w), .pb_done(scsi_sd_done_w),
        .pb_error(scsi_sd_error_w),
        .pb_rd_valid(scsi_sd_rd_valid_w),
        .pb_rd_data(scsi_sd_rd_data_w),
        .pb_rd_ready(scsi_sd_rd_ready_w),
        .pb_wr_ready(scsi_sd_wr_ready_w),
        .pb_wr_data(scsi_sd_wr_data_w),
        .pb_wr_avail(scsi_sd_wr_avail_w),
        .core_clk(video_ref_clk), .core_rst(native_rst),
        .core_cmd_type(ns_cmd_type), .core_lba(ns_lba),
        .core_block_count(ns_block_count), .core_go(ns_go),
        .core_busy(ns_busy), .core_done(ns_done), .core_error(ns_error),
        .core_rd_valid(ns_rd_valid), .core_rd_data(ns_rd_data),
        .core_rd_ready(ns_rd_ready), .core_wr_ready(ns_wr_ready),
        .core_wr_data(ns_wr_data), .core_wr_avail(ns_wr_avail)
    );

    wire [2:0] nc_cmd_type;
    wire [31:0] nc_lba;
    wire [15:0] nc_block_count;
    wire nc_go, nc_busy, nc_done, nc_error, nc_rd_valid, nc_rd_ready;
    wire [7:0] nc_rd_data, nc_wr_data;
    wire nc_wr_ready, nc_wr_avail;
    sd_native_scsi_chunker u_native_scsi_chunker (
        .clk(video_ref_clk), .rst(native_rst),
        .cmd_type(ns_cmd_type), .lba(ns_lba),
        .block_count(ns_block_count), .go(ns_go),
        .busy(ns_busy), .done(ns_done), .error(ns_error),
        .rd_valid(ns_rd_valid), .rd_data(ns_rd_data),
        .rd_ready(ns_rd_ready), .wr_ready(ns_wr_ready),
        .wr_data(ns_wr_data), .wr_avail(ns_wr_avail),
        .card_cmd_type(nc_cmd_type), .card_lba(nc_lba),
        .card_block_count(nc_block_count), .card_go(nc_go),
        .card_busy(nc_busy), .card_done(nc_done),
        .card_error(nc_error), .card_rd_valid(nc_rd_valid),
        .card_rd_data(nc_rd_data), .card_rd_ready(nc_rd_ready),
        .card_wr_ready(nc_wr_ready), .card_wr_data(nc_wr_data),
        .card_wr_avail(nc_wr_avail)
    );

    wire [2:0] card_cmd_type;
    wire [31:0] card_lba;
    wire [15:0] card_block_count;
    wire card_go, card_done, card_error, card_rd_valid, card_rd_ready;
    wire [7:0] card_rd_data, card_wr_data;
    wire card_wr_ready, card_wr_avail;
    sd_native_owner_mux u_native_owner (
        .clk(video_ref_clk), .rst(native_rst),
        .boot_phase_complete(native_boot_done_sync[1]),
        .boot_cmd_type(nb_cmd_type), .boot_lba(nb_lba),
        .boot_block_count(nb_block_count), .boot_go(nb_go),
        .boot_busy(nb_busy), .boot_done(nb_done), .boot_error(nb_error),
        .boot_rd_valid(nb_rd_valid), .boot_rd_data(nb_rd_data),
        .boot_rd_ready(nb_rd_ready),
        .scsi_cmd_type(nc_cmd_type), .scsi_lba(nc_lba),
        .scsi_block_count(nc_block_count), .scsi_go(nc_go),
        .scsi_busy(nc_busy), .scsi_done(nc_done), .scsi_error(nc_error),
        .scsi_rd_valid(nc_rd_valid), .scsi_rd_data(nc_rd_data),
        .scsi_rd_ready(nc_rd_ready), .scsi_wr_ready(nc_wr_ready),
        .scsi_wr_data(nc_wr_data), .scsi_wr_avail(nc_wr_avail),
        .card_cmd_type(card_cmd_type), .card_lba(card_lba),
        .card_block_count(card_block_count), .card_go(card_go),
        .card_busy(native_card_busy), .card_done(card_done),
        .card_error(card_error), .card_rd_valid(card_rd_valid),
        .card_rd_data(card_rd_data), .card_rd_ready(card_rd_ready),
        .card_wr_ready(card_wr_ready), .card_wr_data(card_wr_data),
        .card_wr_avail(card_wr_avail)
    );

    wire [15:0] native_dma_words;
    wire [3:0] native_cmd_event;
`ifdef TQTT_NATIVE_SD_HS
    localparam integer NATIVE_SD_HIGH_SPEED = 1;
`else
    localparam integer NATIVE_SD_HIGH_SPEED = 0;
`endif
    sd_native_backend #(.HIGH_SPEED(NATIVE_SD_HIGH_SPEED)) u_native_card (
        .clk(video_ref_clk), .rst(native_rst),
        .ready(native_ready), .init_error(native_init_error),
        .card_num_lbas(native_card_num_lbas), .rca(native_rca),
        .init_phase(native_init_phase),
        .cmd_type(card_cmd_type), .lba(card_lba),
        .block_count(card_block_count), .go(card_go),
        .busy(native_card_busy),
        .card_session_active(native_card_session_active),
        .done(card_done), .error(card_error),
        .rd_valid(card_rd_valid), .rd_data(card_rd_data),
        .rd_ready(card_rd_ready), .wr_ready(card_wr_ready),
        .wr_data(card_wr_data), .wr_avail(card_wr_avail),
        .debug_dma_words(native_dma_words),
        .debug_cmd_event(native_cmd_event),
        .sd_clk(sd_clk), .sd_cmd(sd_cmd), .sd_dat(sd_dat)
    );

    // Preserve a first-error diagnostic in the existing VIO slot. Native
    // LiteSDCard exposes event bits, not the old SPI R1/CRC operands.
    (* ASYNC_REG = "TRUE" *) reg [2:0] native_scsi_error_core_sync;
    always @(posedge core_clk) begin
        if (soc_full_rst_bank[5]) native_scsi_error_core_sync <= 3'b000;
        else native_scsi_error_core_sync <=
             {native_scsi_error_core_sync[1:0], scsi_sd_error_w};
    end
    wire core_scsi_error = native_scsi_error_core_sync[1] &&
                           !native_scsi_error_core_sync[2];
    wire core_scsi_busy = native_busy_core_sync[1];
    wire [3:0] core_scsi_err_cause = {3'b000, native_cmd_event[2]};
    wire [3:0] core_scsi_dbg_cur_cmd = {1'b0, ns_cmd_type};
    wire [31:0] core_scsi_dbg_lba_lat = ns_lba;
    wire [15:0] core_scsi_dbg_block_idx = 16'd0;
    wire [15:0] core_scsi_dbg_rd_crc_calc = 16'd0;
    wire [15:0] core_scsi_dbg_rd_crc_recv = 16'd0;
    wire [7:0] core_scsi_dbg_last_real_r1 = 8'd0;
    wire [7:0] core_scsi_dbg_last_write_resp = 8'd0;
    wire [7:0] core_scsi_dbg_last_poll_cnt = 8'd0;

    // The old SPI PRAM/JTAG services have no valid native transport yet.
    // Terminate their AXI window and let RTC use its live/default PRAM.
    wire pram_autoload_pending = 1'b0;
    assign pram_sd_default = 1'b0;
    assign pram_x_req = 1'b0;
    assign pram_x_we = 1'b0;
    assign pram_x_addr = 8'b0;
    assign pram_x_wdata = 8'b0;
    axil_null_slave #(.ADDR_WIDTH(20), .DATA_WIDTH(32)) u_native_sdj_null (
        .clk(core_clk), .rst(soc_full_rst_bank[5]),
        .awaddr(sdj_awaddr), .awvalid(sdj_awvalid), .awready(sdj_awready),
        .wdata(sdj_wdata), .wstrb(sdj_wstrb),
        .wvalid(sdj_wvalid), .wready(sdj_wready),
        .bresp(sdj_bresp), .bvalid(sdj_bvalid), .bready(sdj_bready),
        .araddr(sdj_araddr), .arvalid(sdj_arvalid), .arready(sdj_arready),
        .rdata(sdj_rdata), .rresp(sdj_rresp),
        .rvalid(sdj_rvalid), .rready(sdj_rready)
    );
