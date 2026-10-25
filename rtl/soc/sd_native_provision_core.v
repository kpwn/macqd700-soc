// Dedicated TQTT native-mode JTAG card provisioner. The host uses the same
// sd_bulk_writer AXI register/staging contract as the SPI provisioner, but
// each operation is limited to the native backend's 32-sector DMA RAM.
`default_nettype none
module sd_native_provision_core (
    input  wire        clk,
    input  wire        rst,
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
    output wire        sd_clk,
    inout  wire        sd_cmd,
    inout  wire [3:0]  sd_dat,
    output wire        card_ready,
    output wire        init_error,
    output wire        writer_busy
);
    wire [2:0] cmd_type;
    wire [31:0] lba;
    wire [15:0] block_count;
    wire go, busy, done, error;
    wire rd_valid, rd_ready, wr_ready, wr_avail;
    wire [7:0] rd_data, wr_data;
    wire high_speed_active;
    sd_bulk_writer #(.STAGE_SECTORS(32), .NATIVE_TRANSPORT(1)) u_writer (
        .clk(clk), .rst(rst),
        .card_ready(card_ready), .init_error(init_error),
        .s_awaddr(s_awaddr), .s_awlen(s_awlen),
        .s_awvalid(s_awvalid), .s_awready(s_awready),
        .s_wdata(s_wdata), .s_wstrb(s_wstrb),
        .s_wlast(s_wlast), .s_wvalid(s_wvalid),
        .s_wready(s_wready), .s_bresp(s_bresp),
        .s_bvalid(s_bvalid), .s_bready(s_bready),
        .s_araddr(s_araddr), .s_arlen(s_arlen),
        .s_arvalid(s_arvalid), .s_arready(s_arready),
        .s_rdata(s_rdata), .s_rresp(s_rresp),
        .s_rlast(s_rlast), .s_rvalid(s_rvalid),
        .s_rready(s_rready),
        .spi_cmd_valid(), .spi_cmd_ready(1'b0),
        .spi_cmd_data(), .spi_rsp_valid(1'b0),
        .spi_rsp_data(8'b0), .spi_cs_n_in(),
        .spi_fast_mode(), .spi_hs_mode(),
        .native_cmd_type(cmd_type), .native_lba(lba),
        .native_block_count(block_count), .native_go(go),
        .native_busy(busy), .native_done(done),
        .native_error(error), .native_high_speed(high_speed_active),
        .native_rd_valid(rd_valid),
        .native_rd_data(rd_data), .native_rd_ready(rd_ready),
        .native_wr_ready(wr_ready), .native_wr_data(wr_data),
        .native_wr_avail(wr_avail), .writer_busy(writer_busy)
    );
    sd_native_backend #(.HIGH_SPEED(1)) u_card (
        .clk(clk), .rst(rst), .ready(card_ready),
        .init_error(init_error), .high_speed_active(high_speed_active),
        .card_num_lbas(), .rca(), .init_phase(),
        .cmd_type(cmd_type), .lba(lba),
        .block_count(block_count), .go(go),
        .busy(busy), .card_session_active(),
        .done(done), .error(error),
        .rd_valid(rd_valid), .rd_data(rd_data),
        .rd_ready(rd_ready), .wr_ready(wr_ready),
        .wr_data(wr_data), .wr_avail(wr_avail),
        .debug_dma_words(), .debug_cmd_event(),
        .sd_clk(sd_clk), .sd_cmd(sd_cmd), .sd_dat(sd_dat)
    );
endmodule
`default_nettype wire
