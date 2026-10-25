`timescale 1ns/1ps
module tb_sd_native_owner_mux;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst = 1;
    reg phase = 0;
    reg boot_go = 0, scsi_go = 0;
    reg card_busy = 0, card_done = 0, card_error = 0;
    reg card_rd_valid = 0, card_wr_ready = 0;
    reg [7:0] card_rd_data = 0;
    wire [2:0] card_cmd_type;
    wire [31:0] card_lba;
    wire [15:0] card_block_count;
    wire card_go, card_rd_ready, card_wr_avail;
    wire [7:0] card_wr_data;
    wire boot_busy, boot_done, boot_error, boot_rd_valid;
    wire [7:0] boot_rd_data;
    wire scsi_busy, scsi_done, scsi_error, scsi_rd_valid, scsi_wr_ready;
    wire [7:0] scsi_rd_data;
    sd_native_owner_mux dut (
        .clk(clk), .rst(rst), .boot_phase_complete(phase),
        .boot_cmd_type(3'd2), .boot_lba(32'd0),
        .boot_block_count(16'd32), .boot_go(boot_go),
        .boot_busy(boot_busy), .boot_done(boot_done),
        .boot_error(boot_error), .boot_rd_valid(boot_rd_valid),
        .boot_rd_data(boot_rd_data), .boot_rd_ready(1'b1),
        .scsi_cmd_type(3'd4), .scsi_lba(32'd8192),
        .scsi_block_count(16'd2), .scsi_go(scsi_go),
        .scsi_busy(scsi_busy), .scsi_done(scsi_done),
        .scsi_error(scsi_error), .scsi_rd_valid(scsi_rd_valid),
        .scsi_rd_data(scsi_rd_data), .scsi_rd_ready(1'b0),
        .scsi_wr_ready(scsi_wr_ready), .scsi_wr_data(8'ha5),
        .scsi_wr_avail(1'b1),
        .card_cmd_type(card_cmd_type), .card_lba(card_lba),
        .card_block_count(card_block_count), .card_go(card_go),
        .card_busy(card_busy), .card_done(card_done),
        .card_error(card_error), .card_rd_valid(card_rd_valid),
        .card_rd_data(card_rd_data), .card_rd_ready(card_rd_ready),
        .card_wr_ready(card_wr_ready), .card_wr_data(card_wr_data),
        .card_wr_avail(card_wr_avail)
    );
    initial begin
        repeat (3) @(negedge clk);
        rst = 0;
        boot_go = 1;
        #1;
        if (!card_go || card_lba != 0 || card_block_count != 32 ||
            card_cmd_type != 2) $fatal(1, "boot request not selected");
        @(negedge clk);
        boot_go = 0;
        card_busy = 1;
        card_rd_valid = 1;
        card_rd_data = 8'hc3;
        #1;
        if (!boot_busy || scsi_busy || !boot_rd_valid || scsi_rd_valid ||
            !card_rd_ready || boot_rd_data != 8'hc3)
            $fatal(1, "boot stream routed to wrong owner");
        scsi_go = 1;
        @(negedge clk);
        scsi_go = 0;
        #1;
        if (!scsi_done || !scsi_error || !boot_busy)
            $fatal(1, "early SCSI request not rejected");
        @(negedge clk);
        card_done = 1;
        card_busy = 0;
        card_rd_valid = 0;
        #1;
        if (!boot_done || boot_error || scsi_done)
            $fatal(1, "boot completion routed incorrectly");
        @(negedge clk);
        card_done = 0;
        phase = 1;
        scsi_go = 1;
        #1;
        if (!card_go || card_lba != 8192 || card_block_count != 2 ||
            card_cmd_type != 4) $fatal(1, "SCSI request not selected");
        @(negedge clk);
        scsi_go = 0;
        card_busy = 1;
        card_rd_valid = 1;
        card_wr_ready = 1;
        #1;
        if (!scsi_busy || boot_busy || !scsi_rd_valid || boot_rd_valid ||
            card_rd_ready || !scsi_wr_ready || !card_wr_avail ||
            card_wr_data != 8'ha5) $fatal(1, "SCSI stream routed incorrectly");
        phase = 0; // owner must remain SCSI until this request completes
        #1;
        if (!scsi_rd_valid || boot_rd_valid || !scsi_busy)
            $fatal(1, "in-flight owner followed phase change");
        phase = 1;
        card_done = 1;
        card_error = 1;
        card_busy = 0;
        card_rd_valid = 0;
        #1;
        if (!scsi_done || !scsi_error || boot_done)
            $fatal(1, "SCSI error completion routed incorrectly");
        $display("PASS native SD owner mux boot/SCSI isolation, rejection and handoff");
        $finish;
    end
endmodule
