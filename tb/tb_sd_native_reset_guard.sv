`timescale 1ns/1ps
module tb_sd_native_reset_guard;
    reg clk = 0;
    always #5 clk = ~clk;
    reg reset_req = 0;
    reg card_session_active = 0;
    wire rst;
    sd_native_reset_guard dut (
        .clk(clk), .reset_req(reset_req),
        .card_session_active(card_session_active), .rst(rst)
    );
    initial begin
        repeat (5) @(negedge clk);
        if (rst) $fatal(1, "power-on reset did not release");
        card_session_active = 1;
        reset_req = 1;
        @(negedge clk);
        reset_req = 0;
        repeat (5) begin
            @(negedge clk);
            if (rst) $fatal(1, "reset interrupted active card command");
        end
        card_session_active = 0;
        @(negedge clk);
        if (!rst) $fatal(1, "latched reset was lost");
        repeat (4) @(negedge clk);
        if (rst) $fatal(1, "latched reset did not release");
        reset_req = 1;
        @(negedge clk); // capture the request into pending
        repeat (3) begin
            @(negedge clk);
            if (!rst) $fatal(1, "held idle reset was not asserted");
        end
        reset_req = 0;
        repeat (4) @(negedge clk);
        if (rst) $fatal(1, "held idle reset did not release");
        $display("PASS native SD reset guard: active command deferred, short request retained");
        $finish;
    end
endmodule
