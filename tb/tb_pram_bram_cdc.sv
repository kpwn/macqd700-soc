`timescale 1ns/1ps
module tb_pram_bram_cdc #(
    parameter integer A_HALF_PS = 5000,
    parameter integer B_PHASE_PS = 0
);
    reg a_clk = 0, b_clk = 0;
    always #(A_HALF_PS * 1ps) a_clk = !a_clk;
    initial begin
        if (B_PHASE_PS > 0) #(B_PHASE_PS * 1ps);
        forever #10 b_clk = !b_clk;
    end
    reg a_rst = 1, b_rst = 1, rtc_rst = 1;
    reg req = 0, we = 0, clear = 0;
    reg [7:0] addr = 0, wdata = 0;
    wire ack, b_we, busy;
    wire [7:0] rdata, b_addr, b_wdata, b_rdata;
    pram_cdc crossing (
        .a_clk(a_clk), .a_rst(a_rst), .a_req(req), .a_we(we),
        .a_addr(addr), .a_wdata(wdata), .a_rdata(rdata), .a_ack(ack),
        .b_clk(b_clk), .b_rst(b_rst), .b_addr(b_addr), .b_wdata(b_wdata),
        .b_we(b_we), .b_ready(!busy), .b_rdata(b_rdata)
    );
    rtc storage (
        .clk(b_clk), .rst(rtc_rst), .phi2_tick(1'b0),
        .rtc_enb(1'b1), .rtc_clk(1'b0), .rtc_data_o(1'b0), .rtc_data_oe(1'b0),
        .pram_clear(clear), .pram_busy(busy), .pram_ext_addr(b_addr),
        .pram_ext_we(b_we), .pram_ext_wdata(b_wdata), .pram_ext_rdata(b_rdata),
        .cko(), .rtc_data_i()
    );
    always @(posedge b_clk)
        if (b_we && busy) $fatal(1, "restore was accepted during clear");

    task automatic finish_request;
        while (!ack) @(negedge a_clk);
        req = 0;
        while (ack) @(negedge a_clk);
        repeat (2) @(negedge a_clk);
    endtask
    task automatic put(input [7:0] address, input [7:0] value);
        @(negedge a_clk);
        addr = address; wdata = value; we = 1; req = 1;
        finish_request();
    endtask
    task automatic check(input [7:0] address, input [7:0] expected);
        @(negedge a_clk);
        addr = address; we = 0; req = 1;
        while (!ack) @(negedge a_clk);
        if (rdata !== expected)
            $fatal(1, "PRAM[%02x]=%02x expected %02x", address, rdata, expected);
        finish_request();
    endtask

    initial begin
        repeat (8) @(negedge a_clk);
        a_rst = 0; b_rst = 0; rtc_rst = 0;
        for (int i = 0; i < 256; i++) put(8'(i), 8'(i ^ 8'ha5));
        for (int i = 0; i < 256; i++) check(8'(i), 8'(i ^ 8'ha5));

        // Queue a restore early in the sweep, at both ends of the address space.
        // A separate RTC reset halfway through must not cancel clearing or write.
        for (int endpoint = 0; endpoint < 2; endpoint++) begin
            @(negedge b_clk); clear = 1;
            @(negedge b_clk); clear = 0;
            @(negedge a_clk);
            addr = endpoint == 0 ? 8'h00 : 8'hff;
            wdata = 8'h79; we = 1; req = 1;
            repeat (80) begin
                @(negedge a_clk);
                if (ack) $fatal(1, "request acknowledged before clear completed");
            end
            rtc_rst = 1;
            repeat (20) @(negedge a_clk);
            rtc_rst = 0;
            finish_request();
            for (int i = 0; i < 256; i++)
                check(8'(i), (i == (endpoint == 0 ? 0 : 255)) ? 8'h79 : 8'h00);
        end

        // Read the final sweep address: catches an acknowledgement using stale
        // read-before-write data from the clock that cleared the final byte.
        put(8'hff, 8'hde);
        @(negedge b_clk); clear = 1;
        @(negedge b_clk); clear = 0;
        check(8'hff, 8'h00);

        // Restore remains usable while the CPU/RTC protocol is held in reset.
        rtc_rst = 1;
        put(8'h81, 8'h63);
        check(8'h81, 8'h63);
        rtc_rst = 0;
        check(8'h81, 8'h63);
        $display("PRAM_BRAM_CDC_PASS: all bytes, pending clear-time reads/writes, reset retention");
        $finish;
    end
    initial begin
        #1000000;
        $fatal(1, "PRAM CDC test timed out");
    end
endmodule
