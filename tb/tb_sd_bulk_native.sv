`timescale 1ns/1ps
module tb_sd_bulk_native;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst = 1;
    reg [31:0] awaddr = 0, wdata = 0;
    reg awvalid = 0, wvalid = 0;
    wire awready, wready, bvalid, arready, rvalid, rlast;
    wire [1:0] bresp, rresp;
    wire [31:0] rdata;
    wire [2:0] native_cmd_type;
    wire [31:0] native_lba;
    wire [15:0] native_block_count;
    wire native_go, native_rd_ready, native_wr_avail;
    wire [7:0] native_wr_data;
    wire writer_busy;
    reg card_active = 0, card_write = 0, card_done = 0;
    reg [10:0] card_idx = 0;
    reg [7:0] card_mem [0:1023];
    wire native_wr_ready = card_active && card_write && native_wr_avail;
    wire native_rd_valid = card_active && !card_write && native_rd_ready;
    wire [7:0] native_rd_data = card_mem[card_idx[9:0]];

    sd_bulk_writer #(.STAGE_SECTORS(2), .NATIVE_TRANSPORT(1)) dut (
        .clk(clk), .rst(rst), .card_ready(1'b1), .init_error(1'b0),
        .s_awaddr(awaddr), .s_awlen(8'd0), .s_awvalid(awvalid),
        .s_awready(awready), .s_wdata(wdata), .s_wstrb(4'hf),
        .s_wlast(1'b1), .s_wvalid(wvalid), .s_wready(wready),
        .s_bresp(bresp), .s_bvalid(bvalid), .s_bready(1'b1),
        .s_araddr(32'b0), .s_arlen(8'b0), .s_arvalid(1'b0),
        .s_arready(arready), .s_rdata(rdata), .s_rresp(rresp),
        .s_rlast(rlast), .s_rvalid(rvalid), .s_rready(1'b1),
        .spi_cmd_valid(), .spi_cmd_ready(1'b0), .spi_cmd_data(),
        .spi_rsp_valid(1'b0), .spi_rsp_data(8'b0),
        .spi_cs_n_in(), .spi_fast_mode(), .spi_hs_mode(),
        .native_cmd_type(native_cmd_type), .native_lba(native_lba),
        .native_block_count(native_block_count), .native_go(native_go),
        .native_busy(card_active), .native_done(card_done),
        .native_error(1'b0), .native_high_speed(1'b1),
        .native_rd_valid(native_rd_valid),
        .native_rd_data(native_rd_data), .native_rd_ready(native_rd_ready),
        .native_wr_ready(native_wr_ready), .native_wr_data(native_wr_data),
        .native_wr_avail(native_wr_avail), .writer_busy(writer_busy)
    );

    function automatic [31:0] pattern_word(input int index);
        pattern_word = {8'((index*4+3)^8'h5a), 8'((index*4+2)^8'h5a),
                        8'((index*4+1)^8'h5a), 8'((index*4)^8'h5a)};
    endfunction
    always @(posedge clk) begin
        if (rst) begin
            card_active <= 0;
            card_done <= 0;
            card_idx <= 0;
        end else begin
            card_done <= 0;
            if (native_go && !card_active) begin
                if (native_lba != 100 || native_block_count != 2 ||
                    (native_cmd_type != 2 && native_cmd_type != 4))
                    $fatal(1, "wrong native bulk command");
                card_write <= native_cmd_type == 4;
                card_idx <= 0;
                card_active <= 1;
            end else if (card_active &&
                         ((card_write && native_wr_ready) ||
                          (!card_write && native_rd_valid))) begin
                if (card_write) card_mem[card_idx[9:0]] <= native_wr_data;
                if (card_idx == 1023) begin
                    card_active <= 0;
                    card_done <= 1;
                end else card_idx <= card_idx + 1;
            end
        end
    end

    task automatic reg_write(input [31:0] addr, input [31:0] value);
        @(negedge clk);
        awaddr = addr;
        wdata = value;
        awvalid = 1;
        wvalid = 1;
        @(negedge clk);
        awvalid = 0;
        @(negedge clk);
        wvalid = 0;
        wait(bvalid);
        repeat (2) @(negedge clk);
    endtask
    task automatic run_op(input [31:0] cmd);
        reg_write(32'h4, cmd);
        wait(dut.done_sticky_q);
        if (dut.error_q) $fatal(1, "native bulk op error %h", cmd);
        if (~dut.crc_q != 32'hf16469ae)
            $fatal(1, "native bulk CRC mismatch: %h", ~dut.crc_q);
    endtask
    initial begin
        for (int i = 0; i < 256; i++) dut.stage_ram[i] = pattern_word(i);
        for (int i = 0; i < 1024; i++) card_mem[i] = 0;
        repeat (4) @(negedge clk);
        rst = 0;
        reg_write(32'h0, 32'd100);
        reg_write(32'hc, 32'd2);
        run_op(32'h5db0_0001); // WRITE
        for (int i = 0; i < 1024; i++)
            if (card_mem[i] != 8'(i ^ 8'h5a))
                $fatal(1, "native bulk write byte %d got %h", i, card_mem[i]);
        for (int i = 0; i < 256; i++) dut.stage_ram[i] = 0;
        run_op(32'h5db0_0003); // READ to staging
        repeat (3) @(negedge clk);
        for (int i = 0; i < 256; i++)
            if (dut.stage_ram[i] != pattern_word(i))
                $fatal(1, "native bulk read word %d got %h", i, dut.stage_ram[i]);
        run_op(32'h5db0_0002); // CRC verify without rewriting staging
        $display("PASS native bulk writer two-sector WRITE/READ/VERIFY");
        $finish;
    end
    initial begin
        #200000;
        $fatal(1, "native bulk writer timed out");
    end
endmodule
