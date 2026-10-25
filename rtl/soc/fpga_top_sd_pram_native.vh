// Native SD PRAM persistence at the same LBA and JTAG address as SPI mode.
// The lower half of the SD-JTAG aperture remains a null slave until a
// native bulk card writer exists; the upper half serves pram_sd.
    wire pram_autoload_pending;
    wire pram_sd_busy_w;
    wire [19:0] native_prm_awaddr, native_sdw_awaddr;
    wire native_prm_awvalid, native_prm_awready;
    wire native_sdw_awvalid, native_sdw_awready;
    wire [31:0] native_prm_wdata, native_sdw_wdata;
    wire [3:0] native_prm_wstrb, native_sdw_wstrb;
    wire native_prm_wvalid, native_prm_wready;
    wire native_sdw_wvalid, native_sdw_wready;
    wire [1:0] native_prm_bresp, native_sdw_bresp;
    wire native_prm_bvalid, native_prm_bready;
    wire native_sdw_bvalid, native_sdw_bready;
    wire [19:0] native_prm_araddr, native_sdw_araddr;
    wire native_prm_arvalid, native_prm_arready;
    wire native_sdw_arvalid, native_sdw_arready;
    wire [31:0] native_prm_rdata, native_sdw_rdata;
    wire [1:0] native_prm_rresp, native_sdw_rresp;
    wire native_prm_rvalid, native_prm_rready;
    wire native_sdw_rvalid, native_sdw_rready;

    axil_split2 #(.ADDR_W(20), .SEL_BIT(8)) u_native_sdj_split (
        .clk(core_clk), .rst(soc_full_rst_bank[5]),
        .u_awaddr(sdj_awaddr), .u_awvalid(sdj_awvalid), .u_awready(sdj_awready),
        .u_wdata(sdj_wdata), .u_wstrb(sdj_wstrb),
        .u_wvalid(sdj_wvalid), .u_wready(sdj_wready),
        .u_bresp(sdj_bresp), .u_bvalid(sdj_bvalid), .u_bready(sdj_bready),
        .u_araddr(sdj_araddr), .u_arvalid(sdj_arvalid), .u_arready(sdj_arready),
        .u_rdata(sdj_rdata), .u_rresp(sdj_rresp),
        .u_rvalid(sdj_rvalid), .u_rready(sdj_rready),
        .d0_awaddr(native_sdw_awaddr), .d0_awvalid(native_sdw_awvalid),
        .d0_awready(native_sdw_awready), .d0_wdata(native_sdw_wdata),
        .d0_wstrb(native_sdw_wstrb), .d0_wvalid(native_sdw_wvalid),
        .d0_wready(native_sdw_wready), .d0_bresp(native_sdw_bresp),
        .d0_bvalid(native_sdw_bvalid), .d0_bready(native_sdw_bready),
        .d0_araddr(native_sdw_araddr), .d0_arvalid(native_sdw_arvalid),
        .d0_arready(native_sdw_arready), .d0_rdata(native_sdw_rdata),
        .d0_rresp(native_sdw_rresp), .d0_rvalid(native_sdw_rvalid),
        .d0_rready(native_sdw_rready),
        .d1_awaddr(native_prm_awaddr), .d1_awvalid(native_prm_awvalid),
        .d1_awready(native_prm_awready), .d1_wdata(native_prm_wdata),
        .d1_wstrb(native_prm_wstrb), .d1_wvalid(native_prm_wvalid),
        .d1_wready(native_prm_wready), .d1_bresp(native_prm_bresp),
        .d1_bvalid(native_prm_bvalid), .d1_bready(native_prm_bready),
        .d1_araddr(native_prm_araddr), .d1_arvalid(native_prm_arvalid),
        .d1_arready(native_prm_arready), .d1_rdata(native_prm_rdata),
        .d1_rresp(native_prm_rresp), .d1_rvalid(native_prm_rvalid),
        .d1_rready(native_prm_rready)
    );
    axil_null_slave #(.ADDR_WIDTH(20), .DATA_WIDTH(32)) u_native_sdw_null (
        .clk(core_clk), .rst(soc_full_rst_bank[5]),
        .awaddr(native_sdw_awaddr), .awvalid(native_sdw_awvalid),
        .awready(native_sdw_awready), .wdata(native_sdw_wdata),
        .wstrb(native_sdw_wstrb), .wvalid(native_sdw_wvalid),
        .wready(native_sdw_wready), .bresp(native_sdw_bresp),
        .bvalid(native_sdw_bvalid), .bready(native_sdw_bready),
        .araddr(native_sdw_araddr), .arvalid(native_sdw_arvalid),
        .arready(native_sdw_arready), .rdata(native_sdw_rdata),
        .rresp(native_sdw_rresp), .rvalid(native_sdw_rvalid),
        .rready(native_sdw_rready)
    );

    wire [2:0] native_pram_cmd_type;
    wire [31:0] native_pram_lba;
    wire [15:0] native_pram_block_count;
    wire native_pram_go, native_pram_req, native_pram_gnt;
    wire native_pram_busy, native_pram_done, native_pram_error;
    wire native_pram_rd_valid, native_pram_rd_ready;
    wire [7:0] native_pram_rd_data;
    wire native_pram_wr_ready, native_pram_wr_avail;
    wire [7:0] native_pram_wr_data;
    pram_sd #(.AUTOLOAD_ON_BOOT(1), .NATIVE_TRANSPORT(1)) u_native_pram_sd (
        .clk(core_clk), .rst(soc_full_rst_bank[5] && !pram_sd_busy_w),
        .boot_done(boot_rom_loaded),
        .autoload_pending(pram_autoload_pending),
        .s_awaddr(native_prm_awaddr), .s_awvalid(native_prm_awvalid),
        .s_awready(native_prm_awready), .s_wdata(native_prm_wdata),
        .s_wstrb(native_prm_wstrb), .s_wvalid(native_prm_wvalid),
        .s_wready(native_prm_wready), .s_bresp(native_prm_bresp),
        .s_bvalid(native_prm_bvalid), .s_bready(native_prm_bready),
        .s_araddr(native_prm_araddr), .s_arvalid(native_prm_arvalid),
        .s_arready(native_prm_arready), .s_rdata(native_prm_rdata),
        .s_rresp(native_prm_rresp), .s_rvalid(native_prm_rvalid),
        .s_rready(native_prm_rready),
        .spi_cmd_valid(), .spi_cmd_ready(1'b0), .spi_cmd_data(),
        .spi_rsp_valid(1'b0), .spi_rsp_data(8'b0),
        .spi_cs_n_in(), .spi_fast_mode(), .spi_hs_mode(),
        .native_cmd_type(native_pram_cmd_type), .native_lba(native_pram_lba),
        .native_block_count(native_pram_block_count),
        .native_go(native_pram_go), .native_busy(native_pram_busy),
        .native_done(native_pram_done), .native_error(native_pram_error),
        .native_rd_valid(native_pram_rd_valid),
        .native_rd_data(native_pram_rd_data),
        .native_rd_ready(native_pram_rd_ready),
        .native_wr_ready(native_pram_wr_ready),
        .native_wr_data(native_pram_wr_data),
        .native_wr_avail(native_pram_wr_avail),
        .sd_req(native_pram_req), .sd_gnt(native_pram_gnt),
        .pram_req(pram_x_req), .pram_we(pram_x_we),
        .pram_addr(pram_x_addr), .pram_wdata(pram_x_wdata),
        .pram_rdata(pram_x_rdata), .pram_ack(pram_x_ack),
        .pram_default(pram_sd_default), .pram_sd_busy(pram_sd_busy_w)
    );

    sd_scsi_native_bridge u_native_pram_cdc (
        .pb_clk(core_clk),
        .pb_rst(soc_full_rst_bank[5] && !pram_sd_busy_w),
        .pb_cmd_type(native_pram_cmd_type), .pb_lba(native_pram_lba),
        .pb_block_count(native_pram_block_count), .pb_go(native_pram_go),
        .pb_busy(native_pram_busy), .pb_done(native_pram_done),
        .pb_error(native_pram_error),
        .pb_rd_valid(native_pram_rd_valid),
        .pb_rd_data(native_pram_rd_data),
        .pb_rd_ready(native_pram_rd_ready),
        .pb_wr_ready(native_pram_wr_ready),
        .pb_wr_data(native_pram_wr_data),
        .pb_wr_avail(native_pram_wr_avail),
        .core_clk(video_ref_clk), .core_rst(native_rst),
        .core_cmd_type(np_cmd_type), .core_lba(np_lba),
        .core_block_count(np_block_count), .core_go(np_go),
        .core_busy(np_busy), .core_done(np_done), .core_error(np_error),
        .core_rd_valid(np_rd_valid), .core_rd_data(np_rd_data),
        .core_rd_ready(np_rd_ready), .core_wr_ready(np_wr_ready),
        .core_wr_data(np_wr_data), .core_wr_avail(np_wr_avail)
    );

    (* ASYNC_REG = "TRUE" *) reg [1:0] native_pram_req_sync;
    reg native_pram_grant_q;
    always @(posedge video_ref_clk) begin
        if (native_rst) begin
            native_pram_req_sync <= 2'b00;
            native_pram_grant_q <= 1'b0;
        end else begin
            native_pram_req_sync <= {native_pram_req_sync[0], native_pram_req};
            if (!native_pram_req_sync[1] && !np_busy && !np_go)
                native_pram_grant_q <= 1'b0;
            else if (!native_pram_grant_q && native_pram_req_sync[1] &&
                     native_boot_done_sync[1] && native_ready &&
                     !native_card_busy && !nb_go && !nc_go)
                native_pram_grant_q <= 1'b1;
        end
    end
    assign native_pram_grant = native_pram_grant_q;
    (* ASYNC_REG = "TRUE" *) reg [1:0] native_pram_gnt_sync;
    always @(posedge core_clk) begin
        if (soc_full_rst_bank[5] && !pram_sd_busy_w)
            native_pram_gnt_sync <= 2'b00;
        else native_pram_gnt_sync <=
            {native_pram_gnt_sync[0], native_pram_grant};
    end
    assign native_pram_gnt = native_pram_gnt_sync[1];
