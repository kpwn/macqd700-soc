`timescale 1ns/1ps
module tb_tqtt_fan_thermal;
    reg clk = 0;
    always #5 clk = !clk;
    reg locked = 0;
    reg [9:0] adc_code = 0;
    reg adc_valid = 0;
    reg [15:0] rpm = 0;
    wire [7:0] duty;
    wire fault;
    tqtt_fan_thermal #(.CLK_HZ(16)) dut (
        .clk(clk), .locked(locked), .adc_code(adc_code),
        .adc_valid(adc_valid), .rpm(rpm), .duty(duty), .fault(fault)
    );

    task automatic sample(input [9:0] code, input [7:0] expected);
        @(negedge clk);
        adc_code = code;
        adc_valid = 1;
        @(negedge clk);
        adc_valid = 0;
        if (duty !== expected) $fatal(1, "code %0d: duty %0d, expected %0d", code, duty, expected);
    endtask

    initial begin
        repeat (2) @(negedge clk);
        if (duty !== 8'hff) $fatal(1, "reset must be full speed");
        locked = 1;
        rpm = 3000;
        sample(10'd600, 8'd0);
        sample(10'd616, 8'd64);
        sample(10'd636, 8'd144);
        sample(10'd664, 8'hff);
        sample(10'd636, 8'd144);
        repeat (33) @(negedge clk);
        if (duty !== 8'hff || fault !== 0) $fatal(1, "stale sensor must run full speed");
        sample(10'd636, 8'd144);
        sample(10'd900, 8'hff);
        if (fault !== 0) $fatal(1, "invalid conversion should recover");
        sample(10'd636, 8'd144);

        locked = 0;
        repeat (2) @(negedge clk);
        locked = 1;
        rpm = 0;
        sample(10'd636, 8'd144);
        // Keep readings fresh while the fan remains stalled.
        repeat (7) begin
            repeat (15) @(negedge clk);
            sample(10'd636, fault ? 8'hff : 8'd144);
        end
        if (fault !== 1 || duty !== 8'hff) $fatal(1, "tach stall must latch full speed");
        $display("TQTT_FAN_THERMAL_PASS");
        $finish;
    end
endmodule
