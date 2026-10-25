// Carry USB mouse reports from 60 MHz into the 50 MHz Mac peripheral domain.
// The FIFO preserves each report while the consumer keeps up. If saturated,
// a one-entry mailbox retains the latest button/movement report and raises a
// sticky diagnostic; earlier overflowed movement cannot be reconstructed.
`default_nettype none
module usb_hid_adb_mouse_cdc (
    input  wire               usb_clk,
    input  wire               usb_rst,
    input  wire [1:0]         usb_type,
    input  wire               usb_full_report,
    input  wire               usb_button_left,
    input  wire signed [7:0]  usb_dx,
    input  wire signed [7:0]  usb_dy,
    output reg                report_coalesced,
    input  wire               pb_clk,
    input  wire               pb_rst,
    output wire               report_valid,
    input  wire               report_ready,
    output wire               connected,
    output wire               button_left,
    output wire signed [7:0] dx,
    output wire signed [7:0] dy
);
    reg was_mouse;
    reg pending_valid;
    reg [17:0] pending_data;
    wire mouse_now = usb_type == 2'd2;
    wire push = (usb_full_report && mouse_now) || (was_mouse && !mouse_now);
    wire [17:0] push_data = mouse_now ?
                            {1'b1, usb_button_left, usb_dx, usb_dy} : 18'd0;
    wire fifo_full, fifo_empty;
    wire [17:0] fifo_data;
    wire fifo_wr_en = !fifo_full && (pending_valid || push);
    wire [17:0] fifo_wr_data = pending_valid ? pending_data : push_data;

    always @(posedge usb_clk) begin
        if (usb_rst) begin
            was_mouse <= 1'b0;
            pending_valid <= 1'b0;
            pending_data <= 18'd0;
            report_coalesced <= 1'b0;
        end else begin
            was_mouse <= mouse_now;
            if (pending_valid) begin
                if (push) begin
                    pending_data <= push_data;
                    if (fifo_full) report_coalesced <= 1'b1;
                end else if (!fifo_full) pending_valid <= 1'b0;
            end else if (push && fifo_full) begin
                pending_valid <= 1'b1;
                pending_data <= push_data;
            end
        end
    end

    async_fifo #(.WIDTH(18), .DEPTH_LOG2(4)) u_reports (
        .wclk(usb_clk), .wrst(usb_rst),
        .wr_en(fifo_wr_en), .wr_data(fifo_wr_data),
        .wr_full(fifo_full), .wr_almost_full(), .wr_reset_engaged(),
        .rclk(pb_clk), .rrst(pb_rst),
        .rd_en(report_valid && report_ready), .rd_data(fifo_data),
        .rd_empty(fifo_empty), .rd_almost_empty(), .rd_reset_engaged()
    );
    assign report_valid = !fifo_empty;
    assign {connected, button_left, dx, dy} = fifo_data;
endmodule
`default_nettype wire
