`timescale 1ns/1ps
module tb_pram_sd_native;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst = 1;
    reg [19:0] awaddr = 0, araddr = 0;
    reg awvalid = 0, wvalid = 0, arvalid = 0;
    reg [31:0] wdata = 0;
    wire awready, wready, bvalid, arready, rvalid;
    wire [1:0] bresp, rresp;
    wire [31:0] rdata;
    wire [2:0] native_cmd_type;
    wire [31:0] native_lba;
    wire [15:0] native_block_count;
    wire native_go, native_rd_ready, native_wr_avail;
    wire [7:0] native_wr_data;
    reg card_active = 0, card_write = 0, card_done = 0;
    reg [9:0] card_idx = 0;
    reg [7:0] sector [0:511];
    reg [7:0] live_pram [0:255];
    wire pram_req, pram_we, pram_default;
    wire [7:0] pram_addr, pram_wdata;
    reg [7:0] pram_rdata = 0;
    reg pram_ack = 0;
    wire sd_req, busy, autoload_pending;
    wire native_wr_ready = card_active && card_write && native_wr_avail;
    wire native_rd_valid = card_active && !card_write && native_rd_ready;
    wire [7:0] native_rd_data = sector[card_idx[8:0]];

    pram_sd #(.NATIVE_TRANSPORT(1), .AUTOLOAD_ON_BOOT(1),
              .ARB_WAIT_LOG2(12),
              .CDC_WAIT_LOG2(8), .DEFAULT_HOLD_LOG2(11)) dut (
        .clk(clk), .rst(rst), .boot_done(1'b1),
        .autoload_pending(autoload_pending),
        .s_awaddr(awaddr), .s_awvalid(awvalid), .s_awready(awready),
        .s_wdata(wdata), .s_wstrb(4'hf), .s_wvalid(wvalid),
        .s_wready(wready), .s_bresp(bresp), .s_bvalid(bvalid),
        .s_bready(1'b1), .s_araddr(araddr), .s_arvalid(arvalid),
        .s_arready(arready), .s_rdata(rdata), .s_rresp(rresp),
        .s_rvalid(rvalid), .s_rready(1'b1),
        .spi_cmd_valid(), .spi_cmd_ready(1'b0), .spi_cmd_data(),
        .spi_rsp_valid(1'b0), .spi_rsp_data(8'b0),
        .spi_cs_n_in(), .spi_fast_mode(), .spi_hs_mode(),
        .native_cmd_type(native_cmd_type), .native_lba(native_lba),
        .native_block_count(native_block_count), .native_go(native_go),
        .native_busy(card_active), .native_done(card_done),
        .native_error(1'b0), .native_rd_valid(native_rd_valid),
        .native_rd_data(native_rd_data), .native_rd_ready(native_rd_ready),
        .native_wr_ready(native_wr_ready), .native_wr_data(native_wr_data),
        .native_wr_avail(native_wr_avail),
        .sd_req(sd_req), .sd_gnt(sd_req),
        .pram_req(pram_req), .pram_we(pram_we),
        .pram_addr(pram_addr), .pram_wdata(pram_wdata),
        .pram_rdata(pram_rdata), .pram_ack(pram_ack),
        .pram_default(pram_default), .pram_sd_busy(busy)
    );

    always @(posedge clk) begin
        if (rst) begin
            pram_ack <= 0;
            card_active <= 0;
            card_done <= 0;
            card_idx <= 0;
        end else begin
            if (pram_req && !pram_ack) begin
                pram_rdata <= live_pram[pram_addr];
                if (pram_we) live_pram[pram_addr] <= pram_wdata;
                pram_ack <= 1;
            end else if (!pram_req) pram_ack <= 0;
            card_done <= 0;
            if (native_go && !card_active) begin
                if (native_lba != 8191 || native_block_count != 1 ||
                    (native_cmd_type != 1 && native_cmd_type != 3))
                    $fatal(1, "wrong native PRAM sector request");
                card_write <= native_cmd_type == 3;
                card_idx <= 0;
                card_active <= 1;
            end else if (card_active &&
                         ((card_write && native_wr_ready) ||
                          (!card_write && native_rd_valid))) begin
                if (card_write) sector[card_idx[8:0]] <= native_wr_data;
                if (card_idx == 511) begin
                    card_active <= 0;
                    card_done <= 1;
                end else card_idx <= card_idx + 1;
            end
        end
    end

    task automatic issue(input [31:0] cmd);
        @(negedge clk);
        awaddr = 20'h004;
        awvalid = 1;
        wdata = cmd;
        wvalid = 1;
        @(negedge clk);
        awvalid = 0;
        wvalid = 0;
        wait(bvalid);
        wait(busy);
        wait(!busy);
        if (!dut.done_sticky_q || dut.result_q != 0)
            $fatal(1, "PRAM command %h failed: result=%d", cmd, dut.result_q);
    endtask

    initial begin
        for (int i = 0; i < 256; i++) live_pram[i] = 8'(i ^ 8'h5a);
        for (int i = 0; i < 512; i++) sector[i] = 0;
        repeat (4) @(negedge clk);
        rst = 0;
        wait(autoload_pending);
        wait(!autoload_pending);
        if (dut.result_q != 4'd5 || !dut.defaults_q)
            $fatal(1, "blank native PRAM sector did not select defaults");
        issue(32'h5052_0001); // SAVE
        if ({sector[0], sector[1], sector[2], sector[3]} != 32'h50524d31)
            $fatal(1, "native SAVE sector header differs from SPI format");
        for (int i = 0; i < 256; i++)
            if (sector[i+16] != 8'(i ^ 8'h5a))
                $fatal(1, "native SAVE byte %d: got %h", i, sector[i+16]);
        for (int i = 0; i < 256; i++) live_pram[i] = 0;
        repeat (4) @(negedge clk);
        rst = 1;
        repeat (4) @(negedge clk);
        rst = 0;
        wait(autoload_pending);
        wait(!autoload_pending);
        if (dut.result_q != 0 || dut.defaults_q)
            $fatal(1, "saved native PRAM sector did not autoload");
        for (int i = 0; i < 256; i++)
            if (live_pram[i] != 8'(i ^ 8'h5a))
                $fatal(1, "native autoload byte %d: got %h", i, live_pram[i]);
        issue(32'h5052_0002); // Manual LOAD still uses the same sector.
        $display("PASS native PRAM SAVE/LOAD and boot autoload round trip");
        $finish;
    end
    initial begin
        #500000;
        $fatal(1, "native PRAM round trip timed out");
    end
endmodule
