// Native SD SCSI crossing: reuse the validated request/reset handshake from
// sd_scsi_bridge, but replace its SPI-paced byte toggles with dual-clock
// FIFOs. The native four-bit bus can deliver one byte every four 100 MHz
// cycles, which the legacy toggle data path cannot conserve.
//
// A successful read completion is held until every requested byte has been
// delivered to the pb_clk consumer. A card/controller error or peer reset
// completes immediately with error. FIFO pointers are flushed on either
// clock-domain reset and after the visible transaction completion.
`default_nettype none
module sd_scsi_native_bridge (
    input  wire        pb_clk,
    input  wire        pb_rst,
    input  wire [2:0]  pb_cmd_type,
    input  wire [31:0] pb_lba,
    input  wire [15:0] pb_block_count,
    input  wire        pb_go,
    output wire        pb_busy,
    output wire        pb_done,
    output wire        pb_error,
    output reg         pb_rd_valid,
    output reg  [7:0]  pb_rd_data,
    input  wire        pb_rd_ready,
    output wire        pb_wr_ready,
    input  wire [7:0]  pb_wr_data,
    input  wire        pb_wr_avail,

    input  wire        core_clk,
    input  wire        core_rst,
    output wire [2:0]  core_cmd_type,
    output wire [31:0] core_lba,
    output wire [15:0] core_block_count,
    output wire        core_go,
    input  wire        core_busy,
    input  wire        core_done,
    input  wire        core_error,
    input  wire        core_rd_valid,
    input  wire [7:0]  core_rd_data,
    output wire        core_rd_ready,
    input  wire        core_wr_ready,
    output wire [7:0]  core_wr_data,
    output wire        core_wr_avail
);
    wire ctl_busy;
    wire ctl_done;
    wire ctl_error;
    sd_scsi_bridge u_control (
        .pb_clk(pb_clk), .pb_rst(pb_rst),
        .pb_cmd_type(pb_cmd_type), .pb_lba(pb_lba),
        .pb_block_count(pb_block_count), .pb_go(pb_go),
        .pb_busy(ctl_busy), .pb_done(ctl_done), .pb_error(ctl_error),
        .pb_rd_valid(), .pb_rd_data(), .pb_rd_ready(1'b1),
        .pb_wr_ready(), .pb_wr_data(8'b0), .pb_wr_avail(1'b1),
        .core_clk(core_clk), .core_rst(core_rst),
        .core_cmd_type(core_cmd_type), .core_lba(core_lba),
        .core_block_count(core_block_count), .core_go(core_go),
        .core_busy(core_busy), .core_done(core_done),
        .core_error(core_error),
        .core_rd_valid(1'b0), .core_rd_data(8'b0), .core_rd_ready(),
        .core_wr_ready(1'b0), .core_wr_data(), .core_wr_avail()
    );

    reg read_active = 0;
    reg write_active = 0;
    reg [24:0] expected_bytes = 0;
    reg [24:0] delivered_bytes = 0;
    reg done_pending = 0;
    reg done_error = 0;
    reg pb_done_q = 0;
    reg pb_error_q = 0;
    wire read_complete = !read_active || delivered_bytes == expected_bytes;
    wire finish = done_pending && (done_error || read_complete);
    assign pb_done = pb_done_q;
    assign pb_error = pb_error_q;
    assign pb_busy = ctl_busy || done_pending;

    wire stream_arst = pb_rst || core_rst || pb_done_q;
    wire rd_fifo_full;
    wire rd_fifo_empty;
    wire [7:0] rd_fifo_data;
    wire rd_pop = read_active && pb_rd_ready && !rd_fifo_empty;
    assign core_rd_ready = !rd_fifo_full;
    sd_async_byte_fifo #(.ADDR_W(7)) u_read_fifo (
        .wr_clk(core_clk), .rd_clk(pb_clk), .arst(stream_arst),
        .wr_en(core_rd_valid), .wr_data(core_rd_data),
        .wr_full(rd_fifo_full), .rd_en(rd_pop),
        .rd_data(rd_fifo_data), .rd_empty(rd_fifo_empty)
    );

    wire wr_fifo_full;
    wire wr_fifo_empty;
    wire [7:0] wr_fifo_data;
    wire wr_push = write_active && pb_wr_avail && !wr_fifo_full;
    assign pb_wr_ready = wr_push;
    assign core_wr_avail = !wr_fifo_empty;
    assign core_wr_data = wr_fifo_data;
    sd_async_byte_fifo #(.ADDR_W(7)) u_write_fifo (
        .wr_clk(pb_clk), .rd_clk(core_clk), .arst(stream_arst),
        .wr_en(wr_push), .wr_data(pb_wr_data),
        .wr_full(wr_fifo_full), .rd_en(core_wr_ready),
        .rd_data(wr_fifo_data), .rd_empty(wr_fifo_empty)
    );

    always @(posedge pb_clk) begin
        if (pb_rst) begin
            read_active <= 0;
            write_active <= 0;
            expected_bytes <= 0;
            delivered_bytes <= 0;
            done_pending <= 0;
            done_error <= 0;
            pb_done_q <= 0;
            pb_error_q <= 0;
            pb_rd_valid <= 0;
            pb_rd_data <= 0;
        end else begin
            pb_done_q <= 0;
            pb_rd_valid <= rd_pop;
            if (rd_pop) begin
                pb_rd_data <= rd_fifo_data;
                delivered_bytes <= delivered_bytes + 1'b1;
            end
            if (pb_go && !pb_busy) begin
                read_active <= pb_cmd_type == 3'd1 || pb_cmd_type == 3'd2;
                write_active <= pb_cmd_type == 3'd3 || pb_cmd_type == 3'd4;
                expected_bytes <= {pb_block_count, 9'b0};
                delivered_bytes <= 0;
                done_pending <= 0;
                done_error <= 0;
            end
            if (ctl_done) begin
                done_pending <= 1;
                done_error <= ctl_error;
                pb_error_q <= ctl_error;
            end
            if (finish) begin
                done_pending <= 0;
                read_active <= 0;
                write_active <= 0;
                pb_done_q <= 1;
            end
        end
    end
`ifndef SYNTHESIS
    always @(posedge core_clk) begin
        if (!core_rst && core_rd_valid && rd_fifo_full)
            $error("native SD read FIFO overflow: producer ignored core_rd_ready");
        if (!core_rst && core_wr_ready && wr_fifo_empty)
            $error("native SD write FIFO underflow: consumer ignored core_wr_avail");
    end
`endif
endmodule
`default_nettype wire
