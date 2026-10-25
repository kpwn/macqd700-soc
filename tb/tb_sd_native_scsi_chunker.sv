`timescale 1ns/1ps
module tb_sd_native_scsi_chunker;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst = 1;
    reg [2:0] cmd_type = 0;
    reg [31:0] lba = 0;
    reg [15:0] block_count = 0;
    reg go = 0;
    wire busy, done, error;
    wire rd_valid, wr_ready, card_go, card_rd_ready, card_wr_avail;
    wire [7:0] rd_data, card_wr_data;
    wire [2:0] card_cmd_type;
    wire [31:0] card_lba;
    wire [15:0] card_block_count;
    reg card_busy = 0, card_done = 0, card_error = 0;
    integer card_timer = 0;
    integer seen = 0;
    integer expected_cmd = 2;
    integer base_lba = 100;
    integer error_on = 0;
    sd_native_scsi_chunker dut (
        .clk(clk), .rst(rst), .cmd_type(cmd_type), .lba(lba),
        .block_count(block_count), .go(go), .busy(busy),
        .done(done), .error(error), .rd_valid(rd_valid),
        .rd_data(rd_data), .rd_ready(1'b1), .wr_ready(wr_ready),
        .wr_data(8'ha5), .wr_avail(1'b1),
        .card_cmd_type(card_cmd_type), .card_lba(card_lba),
        .card_block_count(card_block_count), .card_go(card_go),
        .card_busy(card_busy), .card_done(card_done),
        .card_error(card_error), .card_rd_valid(1'b1),
        .card_rd_data(8'hc3), .card_rd_ready(card_rd_ready),
        .card_wr_ready(1'b1), .card_wr_data(card_wr_data),
        .card_wr_avail(card_wr_avail)
    );
    always @(posedge clk) begin
        if (rst) begin
            card_busy <= 0;
            card_done <= 0;
            card_error <= 0;
            card_timer <= 0;
            seen <= 0;
        end else begin
            card_done <= 0;
            if (card_go) begin
                if (card_busy || seen >= 3 ||
                    card_lba != base_lba + (seen == 0 ? 0 : seen == 1 ? 32 : 64) ||
                    card_block_count != (seen == 2 ? 1 : 32) ||
                    card_cmd_type != (seen == 2 ? expected_cmd-1 : expected_cmd))
                    $fatal(1, "bad chunk %0d lba=%0d blocks=%0d type=%0d",
                           seen, card_lba, card_block_count, card_cmd_type);
                card_busy <= 1;
                card_timer <= 3;
                seen <= seen + 1;
            end else if (card_timer != 0) begin
                card_timer <= card_timer - 1;
                if (card_timer == 1) begin
                    card_busy <= 0;
                    card_done <= 1;
                    card_error <= error_on == seen;
                end
            end
        end
    end
    task automatic request(input [2:0] typ, input integer inject_error);
        begin
            @(negedge clk);
            cmd_type = typ;
            lba = base_lba;
            block_count = 65;
            expected_cmd = typ;
            error_on = inject_error;
            go = 1;
            @(negedge clk);
            go = 0;
            wait (done);
            #1;
            if (error != (inject_error != 0) ||
                seen != (inject_error == 0 ? 3 : inject_error))
                $fatal(1, "request completion mismatch type=%0d chunks=%0d error=%0d",
                       typ, seen, error);
            @(negedge clk);
            rst = 1;
            repeat (3) @(negedge clk);
            rst = 0;
        end
    endtask
    initial begin
        repeat (3) @(negedge clk);
        rst = 0;
        request(3'd2, 0);
        request(3'd4, 0);
        request(3'd2, 2);
        @(negedge clk);
        cmd_type = 3'd1;
        block_count = 2;
        go = 1;
        @(negedge clk);
        go = 0;
        #1;
        if (!done || !error || seen != 0)
            $fatal(1, "invalid single-block request not rejected");
        $display("PASS native SD SCSI chunker: 65-sector reads/writes, error and invalid request");
        $finish;
    end
endmodule
