// Standalone read-only hardware-native SD probe. Initialization and block
// reads are performed by RTL; VIO only supplies LBA/count and records the
// result. Load only after the card has been power-cycled out of SPI mode.
module tqtt_sd_native_engine_probe (
    input  wire       sys_clk_p,
    input  wire       sys_clk_n,
    output wire       sd_clk,
    inout  wire       sd_cmd,
    inout  wire [3:0] sd_dat
);
    wire clk_ibuf, clk;
    IBUFDS u_clk_ibuf (.I(sys_clk_p), .IB(sys_clk_n), .O(clk_ibuf));
    BUFG u_clk_bufg (.I(clk_ibuf), .O(clk));
    reg [7:0] por = 8'hff;
    always @(posedge clk) if (por != 0) por <= por - 1'b1;

    wire [31:0] vio_lba, vio_control;
    wire rst = (por != 0) || vio_control[7];
    wire init_ready, init_error;
    wire [31:0] card_num_lbas, rca;
    wire [4:0] init_phase;
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
        .clk(clk), .rst(rst), .ready(init_ready), .error(init_error),
        .card_num_lbas(card_num_lbas), .rca(rca),
        .debug_phase(init_phase), .debug_cmd_event(init_cmd_event),
        .debug_cmd_timeout(init_cmd_timeout),
        .wb_adr(init_wb_adr), .wb_bte(init_wb_bte),
        .wb_cti(init_wb_cti), .wb_cyc(init_wb_cyc),
        .wb_dat_r(wb_dat_r), .wb_dat_w(init_wb_dat_w),
        .wb_ack(wb_ack), .wb_err(wb_err),
        .wb_sel(init_wb_sel), .wb_stb(init_wb_stb), .wb_we(init_wb_we)
    );

    reg [31:0] lba_q;
    reg [5:0] count_q;
    reg request_seen;
    reg block_go;
    wire block_busy, block_done, block_error;
    wire block_rd_valid;
    wire [7:0] block_rd_data;
    wire [15:0] block_dma_words;
    wire [3:0] block_cmd_event;
    wire dma_ack, dma_err;
    wire [29:0] dma_adr;
    wire dma_cyc, dma_stb, dma_we;
    wire [31:0] dma_dat_r, dma_dat_w;
    wire [3:0] dma_sel;

    sd_native_block_engine u_block (
        .clk(clk), .rst(rst || !init_ready),
        .cmd_type(count_q == 1 ? 3'd1 : 3'd2),
        .lba(lba_q), .block_count({10'b0, count_q}), .go(block_go),
        .busy(block_busy), .done(block_done), .error(block_error),
        .rd_valid(block_rd_valid), .rd_data(block_rd_data),
        .rd_ready(1'b1), .wr_ready(), .wr_data(8'b0), .wr_avail(1'b0),
        .debug_dma_words(block_dma_words), .debug_cmd_event(block_cmd_event),
        .wb_ctrl_adr(block_wb_adr), .wb_ctrl_bte(block_wb_bte),
        .wb_ctrl_cti(block_wb_cti), .wb_ctrl_cyc(block_wb_cyc),
        .wb_ctrl_dat_r(wb_dat_r), .wb_ctrl_dat_w(block_wb_dat_w),
        .wb_ctrl_ack(wb_ack), .wb_ctrl_err(wb_err),
        .wb_ctrl_sel(block_wb_sel), .wb_ctrl_stb(block_wb_stb),
        .wb_ctrl_we(block_wb_we),
        .wb_dma_ack(dma_ack), .wb_dma_adr(dma_adr),
        .wb_dma_cyc(dma_cyc), .wb_dma_dat_r(dma_dat_r),
        .wb_dma_dat_w(dma_dat_w), .wb_dma_err(dma_err),
        .wb_dma_sel(dma_sel), .wb_dma_stb(dma_stb), .wb_dma_we(dma_we)
    );

    wire [29:0] wb_adr = init_ready ? block_wb_adr : init_wb_adr;
    wire [1:0] wb_bte = init_ready ? block_wb_bte : init_wb_bte;
    wire [2:0] wb_cti = init_ready ? block_wb_cti : init_wb_cti;
    wire wb_cyc = init_ready ? block_wb_cyc : init_wb_cyc;
    wire [31:0] wb_dat_w = init_ready ? block_wb_dat_w : init_wb_dat_w;
    wire [3:0] wb_sel = init_ready ? block_wb_sel : init_wb_sel;
    wire wb_stb = init_ready ? block_wb_stb : init_wb_stb;
    wire wb_we = init_ready ? block_wb_we : init_wb_we;
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
        .wb_dma_ack(dma_ack), .wb_dma_adr(dma_adr),
        .wb_dma_bte(), .wb_dma_cti(), .wb_dma_cyc(dma_cyc),
        .wb_dma_dat_r(dma_dat_r), .wb_dma_dat_w(dma_dat_w),
        .wb_dma_err(dma_err), .wb_dma_sel(dma_sel),
        .wb_dma_stb(dma_stb), .wb_dma_we(dma_we)
    );

    function [31:0] crc32_byte;
        input [31:0] old_crc;
        input [7:0] byte_in;
        integer bit_idx;
        reg [31:0] c;
        begin
            c = old_crc ^ {24'b0, byte_in};
            for (bit_idx = 0; bit_idx < 8; bit_idx = bit_idx + 1)
                c = c[0] ? (c >> 1) ^ 32'hedb88320 : c >> 1;
            crc32_byte = c;
        end
    endfunction
    reg [31:0] crc = 32'hffffffff;
    reg [31:0] result_crc = 0;
    reg [31:0] cycles = 0;
    reg [31:0] result_cycles = 0;
    reg [14:0] byte_count = 0;
    reg running = 0;
    reg done_toggle = 0;
    reg last_error = 0;
    always @(posedge clk) begin
        if (rst) begin
            lba_q <= 0;
            count_q <= 0;
            request_seen <= vio_control[6];
            block_go <= 0;
            crc <= 32'hffffffff;
            result_crc <= 0;
            cycles <= 0;
            result_cycles <= 0;
            byte_count <= 0;
            running <= 0;
            done_toggle <= 0;
            last_error <= 0;
        end else begin
            block_go <= 0;
            if (init_ready && !block_busy && !block_go &&
                vio_control[6] != request_seen) begin
                request_seen <= vio_control[6];
                lba_q <= vio_lba;
                count_q <= vio_control[5:0];
                crc <= 32'hffffffff;
                byte_count <= 0;
                cycles <= 0;
                running <= 1;
                block_go <= 1;
            end
            if (running) cycles <= cycles + 1'b1;
            if (block_rd_valid) begin
                crc <= crc32_byte(crc, block_rd_data);
                byte_count <= byte_count + 1'b1;
            end
            if (block_done) begin
                running <= 0;
                last_error <= block_error;
                result_crc <= crc ^ 32'hffffffff;
                result_cycles <= cycles;
                done_toggle <= ~done_toggle;
            end
        end
    end
    wire [31:0] status = {byte_count, 7'b0, init_phase, init_error,
                          init_ready, block_busy, last_error, done_toggle};
    native_sd_vio u_vio (
        .clk(clk),
        .probe_in0(card_num_lbas), .probe_in1(rca),
        .probe_in2(status), .probe_in3(result_crc),
        .probe_in4(result_cycles),
        .probe_out0(vio_lba), .probe_out1(vio_control),
        .probe_out2(), .probe_out3()
    );
endmodule
