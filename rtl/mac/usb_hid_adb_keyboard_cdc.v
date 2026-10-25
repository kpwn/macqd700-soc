// Transfer complete keyboard snapshots from the 60 MHz USB host to pb_clk.
// A type transition away from keyboard emits a zero snapshot so the ADB
// converter releases held keys. The latest snapshot is retained if the FIFO
// fills; ordinary reports arrive milliseconds apart and drain in <1 us.
`default_nettype none
module usb_hid_adb_keyboard_cdc (
    input  wire        usb_clk,
    input  wire        usb_rst,
    input  wire [1:0]  usb_type,
    input  wire        usb_full_report,
    input  wire [7:0]  usb_modifiers,
    input  wire [47:0] usb_usages,
    output reg         report_coalesced,
    input  wire        pb_clk,
    input  wire        pb_rst,
    output wire        report_valid,
    input  wire        report_ready,
    output wire        connected,
    output wire [7:0]  modifiers,
    output wire [47:0] usages
);
    reg was_keyboard;
    reg pending_valid;
    reg [56:0] pending_data;
    wire keyboard_now = usb_type == 2'd1;
    wire push = (usb_full_report && keyboard_now) ||
                (was_keyboard && !keyboard_now);
    wire [56:0] push_data = keyboard_now ?
                            {1'b1, usb_modifiers, usb_usages} : 57'd0;
    wire fifo_full, fifo_empty;
    wire [56:0] fifo_data;
    wire fifo_wr_en = !fifo_full && (pending_valid || push);
    wire [56:0] fifo_wr_data = pending_valid ? pending_data : push_data;

    always @(posedge usb_clk) begin
        if (usb_rst) begin
            was_keyboard <= 1'b0;
            pending_valid <= 1'b0;
            pending_data <= 57'd0;
            report_coalesced <= 1'b0;
        end else begin
            was_keyboard <= keyboard_now;
            if (pending_valid) begin
                if (push) begin
                    // Emit the older pending snapshot if room exists, then
                    // keep the newest one. On overflow, coalesce state.
                    pending_data <= push_data;
                    if (fifo_full) report_coalesced <= 1'b1;
                end else if (!fifo_full) begin
                    pending_valid <= 1'b0;
                end
            end else if (push && fifo_full) begin
                pending_valid <= 1'b1;
                pending_data <= push_data;
            end
        end
    end

    async_fifo #(.WIDTH(57), .DEPTH_LOG2(4)) u_reports (
        .wclk(usb_clk), .wrst(usb_rst),
        .wr_en(fifo_wr_en), .wr_data(fifo_wr_data),
        .wr_full(fifo_full), .wr_almost_full(), .wr_reset_engaged(),
        .rclk(pb_clk), .rrst(pb_rst),
        .rd_en(report_valid && report_ready), .rd_data(fifo_data),
        .rd_empty(fifo_empty), .rd_almost_empty(), .rd_reset_engaged()
    );
    assign report_valid = !fifo_empty;
    assign {connected, modifiers, usages} = fifo_data;
endmodule
`default_nettype wire
