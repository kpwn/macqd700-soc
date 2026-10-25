// TQTT four-bit SD transport. One LiteSDCard controller is initialized in
// hardware and then serves bounded 1..32-sector block requests. The block
// engine owns its 16 KiB DMA RAM; consumers use byte streams. Keep this
// module on a 100 MHz clock, independent of the CPU's 100/200 MHz setting.
// A fresh card power cycle is required when switching from the SPI backend.
`default_nettype none
module sd_native_backend (
    input  wire        clk,
    input  wire        rst,
    output wire        ready,
    output wire        init_error,
    output wire [31:0] card_num_lbas,
    output wire [31:0] rca,
    output wire [4:0]  init_phase,
    input  wire [2:0]  cmd_type,
    input  wire [31:0] lba,
    input  wire [15:0] block_count,
    input  wire        go,
    output wire        busy,
    output wire        done,
    output wire        error,
    output wire        rd_valid,
    output wire [7:0]  rd_data,
    input  wire        rd_ready,
    output wire        wr_ready,
    input  wire [7:0]  wr_data,
    input  wire        wr_avail,
    output wire [15:0] debug_dma_words,
    output wire [3:0]  debug_cmd_event,
    output wire        sd_clk,
    inout  wire        sd_cmd,
    inout  wire [3:0]  sd_dat
);
    wire [3:0] init_cmd_event;
    wire init_cmd_timeout;
    wire [29:0] init_wb_adr, block_wb_adr;
    wire [1:0] init_wb_bte, block_wb_bte;
    wire [2:0] init_wb_cti, block_wb_cti;
    wire init_wb_cyc, block_wb_cyc;
    wire [31:0] init_wb_dat_w, block_wb_dat_w;
    wire init_wb_stb, block_wb_stb;
    wire [3:0] init_wb_sel, block_wb_sel;
    wire init_wb_we, block_wb_we;
    wire [31:0] wb_dat_r;
    wire wb_ack, wb_err;

    sd_native_init_controller u_init (
        .clk(clk), .rst(rst), .ready(ready), .error(init_error),
        .card_num_lbas(card_num_lbas), .rca(rca),
        .debug_phase(init_phase), .debug_cmd_event(init_cmd_event),
        .debug_cmd_timeout(init_cmd_timeout),
        .wb_adr(init_wb_adr), .wb_bte(init_wb_bte),
        .wb_cti(init_wb_cti), .wb_cyc(init_wb_cyc),
        .wb_dat_r(wb_dat_r), .wb_dat_w(init_wb_dat_w),
        .wb_ack(wb_ack), .wb_err(wb_err),
        .wb_sel(init_wb_sel), .wb_stb(init_wb_stb), .wb_we(init_wb_we)
    );

    wire [29:0] block_dma_adr;
    wire block_dma_ack, block_dma_cyc, block_dma_stb, block_dma_we;
    wire [31:0] block_dma_dat_r, block_dma_dat_w;
    wire block_dma_err;
    wire [3:0] block_dma_sel;
    sd_native_block_engine u_block (
        .clk(clk), .rst(rst || !ready),
        .cmd_type(cmd_type), .lba(lba), .block_count(block_count), .go(go),
        .busy(busy), .done(done), .error(error),
        .rd_valid(rd_valid), .rd_data(rd_data), .rd_ready(rd_ready),
        .wr_ready(wr_ready), .wr_data(wr_data), .wr_avail(wr_avail),
        .debug_dma_words(debug_dma_words),
        .debug_cmd_event(debug_cmd_event),
        .wb_ctrl_adr(block_wb_adr), .wb_ctrl_bte(block_wb_bte),
        .wb_ctrl_cti(block_wb_cti), .wb_ctrl_cyc(block_wb_cyc),
        .wb_ctrl_dat_r(wb_dat_r), .wb_ctrl_dat_w(block_wb_dat_w),
        .wb_ctrl_ack(wb_ack), .wb_ctrl_err(wb_err),
        .wb_ctrl_sel(block_wb_sel), .wb_ctrl_stb(block_wb_stb),
        .wb_ctrl_we(block_wb_we),
        .wb_dma_ack(block_dma_ack), .wb_dma_adr(block_dma_adr),
        .wb_dma_cyc(block_dma_cyc), .wb_dma_dat_r(block_dma_dat_r),
        .wb_dma_dat_w(block_dma_dat_w), .wb_dma_err(block_dma_err),
        .wb_dma_sel(block_dma_sel), .wb_dma_stb(block_dma_stb),
        .wb_dma_we(block_dma_we)
    );

    wire [29:0] wb_adr = ready ? block_wb_adr : init_wb_adr;
    wire [1:0] wb_bte = ready ? block_wb_bte : init_wb_bte;
    wire [2:0] wb_cti = ready ? block_wb_cti : init_wb_cti;
    wire wb_cyc = ready ? block_wb_cyc : init_wb_cyc;
    wire [31:0] wb_dat_w = ready ? block_wb_dat_w : init_wb_dat_w;
    wire [3:0] wb_sel = ready ? block_wb_sel : init_wb_sel;
    wire wb_stb = ready ? block_wb_stb : init_wb_stb;
    wire wb_we = ready ? block_wb_we : init_wb_we;
    wire irq, sd_cmd_dir, sd_dat0_dir, sd_dat13_dir;
    litesdcard_core u_sd (
        .clk(clk), .rst(rst), .irq(irq), .sdcard_cd(1'b0),
        .sdcard_clk(sd_clk), .sdcard_cmd(sd_cmd),
        .sdcard_cmd_dir(sd_cmd_dir), .sdcard_dat0_dir(sd_dat0_dir),
        .sdcard_dat13_dir(sd_dat13_dir), .sdcard_data(sd_dat),
        .wb_ctrl_ack(wb_ack), .wb_ctrl_adr(wb_adr),
        .wb_ctrl_bte(wb_bte), .wb_ctrl_cti(wb_cti),
        .wb_ctrl_cyc(wb_cyc), .wb_ctrl_dat_r(wb_dat_r),
        .wb_ctrl_dat_w(wb_dat_w), .wb_ctrl_err(wb_err),
        .wb_ctrl_sel(wb_sel), .wb_ctrl_stb(wb_stb), .wb_ctrl_we(wb_we),
        .wb_dma_ack(block_dma_ack), .wb_dma_adr(block_dma_adr),
        .wb_dma_bte(), .wb_dma_cti(), .wb_dma_cyc(block_dma_cyc),
        .wb_dma_dat_r(block_dma_dat_r), .wb_dma_dat_w(block_dma_dat_w),
        .wb_dma_err(block_dma_err), .wb_dma_sel(block_dma_sel),
        .wb_dma_stb(block_dma_stb), .wb_dma_we(block_dma_we)
    );
endmodule
`default_nettype wire
