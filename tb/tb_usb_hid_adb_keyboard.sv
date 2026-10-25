`timescale 1ns/1ps
module tb_usb_hid_adb_keyboard;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst = 1;
    reg report_valid = 0, connected = 0;
    reg [7:0] modifiers = 0;
    reg [47:0] usages = 0;
    wire report_ready, event_valid;
    wire [7:0] event_byte;
    reg event_ready = 1;
    reg [7:0] expected [0:15];
    integer expected_count = 0, seen = 0;

    usb_hid_adb_keyboard dut (
        .clk(clk), .rst(rst),
        .report_valid(report_valid), .report_ready(report_ready),
        .connected(connected), .modifiers(modifiers), .usages(usages),
        .event_valid(event_valid), .event_byte(event_byte),
        .event_ready(event_ready)
    );

    always @(posedge clk) begin
        if (!rst && event_valid && event_ready) begin
            if (seen >= expected_count || event_byte !== expected[seen])
                $fatal(1, "ADB event %0d got 0x%02x, expected 0x%02x",
                       seen, event_byte, expected[seen]);
            seen <= seen + 1;
        end
    end

    task automatic want(input [7:0] code);
        expected[expected_count] = code;
        expected_count = expected_count + 1;
    endtask
    task automatic send_report(input bit present, input [7:0] mods,
                               input [47:0] keys);
        wait(report_ready);
        @(negedge clk);
        connected = present;
        modifiers = mods;
        usages = keys;
        report_valid = 1;
        @(negedge clk);
        report_valid = 0;
        wait(report_ready);
        repeat (2) @(negedge clk);
    endtask

    initial begin
        repeat (3) @(negedge clk);
        rst = 0;
        want(8'h00); // HID A -> ADB A down
        send_report(1, 0, {8'h04, 40'b0});
        send_report(1, 0, {8'h04, 40'b0}); // repeat snapshot, no event

        want(8'h0b); // B down, A remains down
        send_report(1, 0, {8'h04, 8'h05, 32'b0});

        want(8'h80); // A up, then Shift down, B remains down
        want(8'h38);
        event_ready = 0;
        fork
            send_report(1, 8'h02, {8'h05, 40'b0});
            begin
                wait(event_valid);
                if (event_byte !== 8'h80) $fatal(1, "wrong stalled event");
                repeat (3) begin
                    @(negedge clk);
                    if (!event_valid || event_byte !== 8'h80)
                        $fatal(1, "event changed during backpressure");
                end
                event_ready = 1;
            end
        join

        send_report(1, 8'h02, {8'h01, 40'b0}); // HID rollover, ignore
        want(8'h8b); // disconnect releases B and Shift
        want(8'hb8);
        send_report(0, 0, 0);

        want(8'h08); // duplicate C usage emits one keydown
        send_report(1, 0, {8'h06, 8'h06, 32'b0});
        want(8'h88);
        send_report(0, 0, 0);
        if (seen != expected_count)
            $fatal(1, "only %0d of %0d events received", seen, expected_count);
        $display("PASS USB HID keyboard snapshots, chords, stall, rollover, disconnect");
        $finish;
    end
    initial begin
        #100000;
        $fatal(1, "USB HID ADB keyboard test timed out");
    end
endmodule
