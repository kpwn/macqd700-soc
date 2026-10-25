// Dual-clock byte FIFO for the native SD transport. Gray-coded pointers
// cross clock domains; the data word becomes visible only after the write
// pointer has passed two destination-clock synchronizer stages.
//
// Assert arst for any reset of either endpoint. Pointers clear together,
// discarding an interrupted transaction; the surrounding SCSI bridge must
// report that interruption as an error through its reset-epoch path.
`default_nettype none
module sd_async_byte_fifo #(
    parameter integer ADDR_W = 6
) (
    input  wire       wr_clk,
    input  wire       rd_clk,
    input  wire       arst,
    input  wire       wr_en,
    input  wire [7:0] wr_data,
    output wire       wr_full,
    input  wire       rd_en,
    output wire [7:0] rd_data,
    output wire       rd_empty
);
    localparam integer DEPTH = 1 << ADDR_W;
    reg [7:0] mem [0:DEPTH-1];

    reg [ADDR_W:0] wr_bin = 0;
    reg [ADDR_W:0] wr_gray = 0;
    reg [ADDR_W:0] rd_bin = 0;
    reg [ADDR_W:0] rd_gray = 0;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_W:0] rd_gray_wr_a = 0;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_W:0] rd_gray_wr_b = 0;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_W:0] wr_gray_rd_a = 0;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_W:0] wr_gray_rd_b = 0;

    wire wr_take = wr_en && !wr_full;
    wire rd_take = rd_en && !rd_empty;
    wire [ADDR_W:0] wr_bin_next = wr_bin + {{ADDR_W{1'b0}}, wr_take};
    wire [ADDR_W:0] rd_bin_next = rd_bin + {{ADDR_W{1'b0}}, rd_take};

    // Full is recognized when the current write pointer has wrapped once
    // past the synchronized read pointer; empty means pointers match.
    assign wr_full = wr_gray == {~rd_gray_wr_b[ADDR_W:ADDR_W-1],
                                rd_gray_wr_b[ADDR_W-2:0]};
    assign rd_empty = rd_gray == wr_gray_rd_b;
    assign rd_data = mem[rd_bin[ADDR_W-1:0]];

    // The data array has no reset requirement: cleared pointers make old
    // contents unreachable, including any write made while reset is high.
    // Keep its write port outside the asynchronous pointer-reset process so
    // Vivado can infer memory instead of flops.
    always @(posedge wr_clk) begin
        if (wr_take) mem[wr_bin[ADDR_W-1:0]] <= wr_data;
    end

    always @(posedge wr_clk or posedge arst) begin
        if (arst) begin
            wr_bin <= 0;
            wr_gray <= 0;
            rd_gray_wr_a <= 0;
            rd_gray_wr_b <= 0;
        end else begin
            rd_gray_wr_a <= rd_gray;
            rd_gray_wr_b <= rd_gray_wr_a;
            if (wr_take) begin
                wr_bin <= wr_bin_next;
                wr_gray <= (wr_bin_next >> 1) ^ wr_bin_next;
            end
        end
    end

    always @(posedge rd_clk or posedge arst) begin
        if (arst) begin
            rd_bin <= 0;
            rd_gray <= 0;
            wr_gray_rd_a <= 0;
            wr_gray_rd_b <= 0;
        end else begin
            wr_gray_rd_a <= wr_gray;
            wr_gray_rd_b <= wr_gray_rd_a;
            if (rd_take) begin
                rd_bin <= rd_bin_next;
                rd_gray <= (rd_bin_next >> 1) ^ rd_bin_next;
            end
        end
    end
endmodule
`default_nettype wire
