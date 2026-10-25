`timescale 1ns/1ps
module tb_usb_hid_adb_keyboard_cdc;
    reg usb_clk = 0, pb_clk = 0;
    always #8 usb_clk = ~usb_clk;
    always #10 pb_clk = ~pb_clk;
    reg usb_rst = 1, pb_rst = 1;
    reg [1:0] usb_type = 0;
    reg usb_full_report = 0;
    reg [7:0] usb_modifiers = 0;
    reg [47:0] usb_usages = 0;
    wire report_coalesced, report_valid, report_ready, connected;
    wire [7:0] modifiers;
    wire [47:0] usages;
    wire event_valid;
    wire [7:0] event_byte;
    reg [7:0] expected [0:3];
    integer seen = 0;

    usb_hid_adb_keyboard_cdc u_cdc (
        .usb_clk(usb_clk), .usb_rst(usb_rst),
        .usb_type(usb_type), .usb_full_report(usb_full_report),
        .usb_modifiers(usb_modifiers), .usb_usages(usb_usages),
        .report_coalesced(report_coalesced),
        .pb_clk(pb_clk), .pb_rst(pb_rst),
        .report_valid(report_valid), .report_ready(report_ready),
        .connected(connected), .modifiers(modifiers), .usages(usages)
    );
    usb_hid_adb_keyboard u_translate (
        .clk(pb_clk), .rst(pb_rst),
        .report_valid(report_valid), .report_ready(report_ready),
        .connected(connected), .modifiers(modifiers), .usages(usages),
        .event_valid(event_valid), .event_byte(event_byte),
        .event_ready(1'b1)
    );
    always @(posedge pb_clk) begin
        if (!pb_rst && event_valid) begin
            if (seen >= 4 || event_byte !== expected[seen])
                $fatal(1, "CDC event %0d got 0x%02x", seen, event_byte);
            seen <= seen + 1;
        end
    end
    task automatic host_report(input [47:0] keys);
        @(negedge usb_clk);
        usb_type = 2'd1;
        usb_usages = keys;
        usb_full_report = 1;
        @(negedge usb_clk);
        usb_full_report = 0;
    endtask
    initial begin
        expected[0] = 8'h00; // A down
        expected[1] = 8'h0b; // B down
        expected[2] = 8'h80; // A up
        expected[3] = 8'h8b; // B up on disconnect
        repeat (20) @(negedge usb_clk);
        usb_rst = 0;
        pb_rst = 0;
        repeat (40) @(negedge usb_clk);
        host_report({8'h04, 40'd0});
        host_report({8'h04, 8'h05, 32'd0});
        host_report({8'h05, 40'd0});
        @(negedge usb_clk);
        usb_type = 2'd0; // no full_report pulse on unplug
        wait(seen == 4);
        repeat (40) @(negedge pb_clk);
        if (report_coalesced) $fatal(1, "unexpected report coalescing");
        $display("PASS USB HID keyboard 60-to-50 MHz CDC and disconnect");
        $finish;
    end
    initial begin
        #200000;
        $fatal(1, "USB HID CDC test timed out");
    end
endmodule
