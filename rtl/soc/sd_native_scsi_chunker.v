// Translate one SCSI block request into bounded native SD DMA operations.
// The physical engine owns 16 KiB and accepts 1..32 sectors per command;
// SCSI may request more. Keep the byte stream continuous across commands
// and report exactly one final done/error to the CDC bridge.
`default_nettype none
module sd_native_scsi_chunker (
    input  wire        clk,
    input  wire        rst,
    input  wire [2:0]  cmd_type,
    input  wire [31:0] lba,
    input  wire [15:0] block_count,
    input  wire        go,
    output wire        busy,
    output reg         done,
    output reg         error,
    output wire        rd_valid,
    output wire [7:0]  rd_data,
    input  wire        rd_ready,
    output wire        wr_ready,
    input  wire [7:0]  wr_data,
    input  wire        wr_avail,
    output wire [2:0]  card_cmd_type,
    output wire [31:0] card_lba,
    output wire [15:0] card_block_count,
    output wire        card_go,
    input  wire        card_busy,
    input  wire        card_done,
    input  wire        card_error,
    input  wire        card_rd_valid,
    input  wire [7:0]  card_rd_data,
    output wire        card_rd_ready,
    input  wire        card_wr_ready,
    output wire [7:0]  card_wr_data,
    output wire        card_wr_avail
);
    localparam [1:0] IDLE = 0, START = 1, RUN = 2;
    reg [1:0] state;
    reg write_q;
    reg [31:0] lba_q;
    reg [15:0] remaining;
    wire [15:0] chunk_blocks = remaining > 16'd32 ? 16'd32 : remaining;
    assign busy = state != IDLE;
    assign card_cmd_type = write_q ? (chunk_blocks == 1 ? 3'd3 : 3'd4) :
                           (chunk_blocks == 1 ? 3'd1 : 3'd2);
    assign card_lba = lba_q;
    assign card_block_count = chunk_blocks;
    assign card_go = state == START && !card_busy;
    assign rd_valid = card_rd_valid && state == RUN;
    assign rd_data = card_rd_data;
    assign card_rd_ready = rd_ready;
    assign wr_ready = card_wr_ready && state == RUN;
    assign card_wr_data = wr_data;
    assign card_wr_avail = wr_avail && state == RUN;
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            write_q <= 0;
            lba_q <= 0;
            remaining <= 0;
            done <= 0;
            error <= 0;
        end else begin
            done <= 0;
            if (state == IDLE) begin
                if (go) begin
                    error <= 0;
                    if (block_count == 0 || cmd_type < 3'd1 ||
                        cmd_type > 3'd4 ||
                        ((cmd_type == 3'd1 || cmd_type == 3'd3) &&
                         block_count != 1) ||
                        ({1'b0, lba} + {17'd0, block_count} > 33'h100000000)) begin
                        error <= 1;
                        done <= 1;
                    end else begin
                        write_q <= cmd_type == 3'd3 || cmd_type == 3'd4;
                        lba_q <= lba;
                        remaining <= block_count;
                        state <= START;
                    end
                end
            end else if (state == START) begin
                if (!card_busy) state <= RUN;
            end else if (card_done) begin
                if (card_error) begin
                    error <= 1;
                    done <= 1;
                    state <= IDLE;
                end else if (remaining == chunk_blocks) begin
                    done <= 1;
                    state <= IDLE;
                end else begin
                    remaining <= remaining - chunk_blocks;
                    lba_q <= lba_q + {16'd0, chunk_blocks};
                    state <= START;
                end
            end
        end
    end
endmodule
`default_nettype wire
