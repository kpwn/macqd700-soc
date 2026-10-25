// Native SD block transport for up to 32 contiguous sectors per request.
// LiteSDCard DMA terminates in a local 16 KiB RAM. Reads are drained to the
// SCSI byte stream after CRC/data completion; writes are staged before the
// card command. The caller must initialize the card first and keep this
// engine in reset until it owns the LiteSDCard control Wishbone port.
`default_nettype none
module sd_native_block_engine #(
    parameter integer POLL_TIMEOUT_CYCLES = 100000000,
    parameter integer EVENT_TIMEOUT_CYCLES = 100000000,
    parameter integer CSR_TIMEOUT_CYCLES = 1024
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [2:0]  cmd_type,
    input  wire [31:0] lba,
    input  wire [15:0] block_count,
    input  wire        go,
    output wire        busy,
    // A card command may be open. Preserve the controller across a debug
    // reset until CMD12/data completion; the staged write RAM is already
    // full before this interval begins.
    output wire        card_session_active,
    output reg         done,
    output reg         error,
    output wire        rd_valid,
    output reg  [7:0]  rd_data,
    input  wire        rd_ready,
    output wire        wr_ready,
    input  wire [7:0]  wr_data,
    input  wire        wr_avail,
    output reg  [15:0] debug_dma_words,
    output wire [3:0]  debug_cmd_event,

    output wire [29:0] wb_ctrl_adr,
    output wire [1:0]  wb_ctrl_bte,
    output wire [2:0]  wb_ctrl_cti,
    output wire        wb_ctrl_cyc,
    input  wire [31:0] wb_ctrl_dat_r,
    output wire [31:0] wb_ctrl_dat_w,
    input  wire        wb_ctrl_ack,
    input  wire        wb_ctrl_err,
    output wire [3:0]  wb_ctrl_sel,
    output wire        wb_ctrl_stb,
    output wire        wb_ctrl_we,

    output wire        wb_dma_ack,
    input  wire [29:0] wb_dma_adr,
    input  wire        wb_dma_cyc,
    output reg  [31:0] wb_dma_dat_r,
    input  wire [31:0] wb_dma_dat_w,
    output wire        wb_dma_err,
    input  wire [3:0]  wb_dma_sel,
    input  wire        wb_dma_stb,
    input  wire        wb_dma_we
);
    localparam [3:0]
        S_IDLE      = 4'd0,
        S_FILL      = 4'd1,
        S_CFG       = 4'd2,
        S_CFG_WAIT  = 4'd3,
        S_CMD       = 4'd4,
        S_CMD_WAIT  = 4'd5,
        S_DATA      = 4'd6,
        S_DATA_WAIT = 4'd7,
        S_DMA       = 4'd8,
        S_DMA_WAIT  = 4'd9,
        S_STOP      = 4'd10,
        S_STOP_WAIT = 4'd11,
        S_STREAM    = 4'd12;
    localparam integer POLL_W = $clog2(POLL_TIMEOUT_CYCLES + 1);
    localparam [POLL_W-1:0] POLL_LAST = POLL_TIMEOUT_CYCLES - 1;
    reg [3:0] state;
    reg [2:0] cmd_type_q;
    reg [31:0] lba_q;
    reg [15:0] blocks_q;
    reg [2:0] cfg_step;
    reg [14:0] byte_index;
    reg [31:0] write_accum;
    reg [POLL_W-1:0] poll_timer;
    wire writing = cmd_type_q == 3'd3 || cmd_type_q == 3'd4;
    wire multi = cmd_type_q == 3'd2 || cmd_type_q == 3'd4;
    wire [14:0] total_bytes = {blocks_q[5:0], 9'b0};
    assign busy = state != S_IDLE;
    assign card_session_active = state == S_CMD || state == S_CMD_WAIT ||
                                 state == S_DATA || state == S_DATA_WAIT ||
                                 state == S_DMA || state == S_DMA_WAIT ||
                                 state == S_STOP || state == S_STOP_WAIT;
    assign wr_ready = state == S_FILL && wr_avail;
    assign rd_valid = state == S_STREAM && rd_ready && byte_index < total_bytes;

    reg [31:0] ram [0:4095];
    reg [31:0] stream_word;
    // Synchronous BRAM read. On the fourth byte of a word, fetch the next
    // word at the same edge that advances byte_index. That keeps the byte
    // stream continuous without asking Vivado for an async 16 KiB read port.
    wire [14:0] fetch_byte_index = byte_index + {14'b0, rd_valid};
    wire [11:0] fetch_word_addr = fetch_byte_index[13:2];
    always @(*) begin
        case (byte_index[1:0])
            2'd0: rd_data = stream_word[31:24];
            2'd1: rd_data = stream_word[23:16];
            2'd2: rd_data = stream_word[15:8];
            default: rd_data = stream_word[7:0];
        endcase
    end

    wire dma_addr_ok = ~|wb_dma_adr[29:12];
    reg dma_ack_q;
    wire dma_take = wb_dma_cyc && wb_dma_stb && dma_addr_ok && !dma_ack_q;
    assign wb_dma_ack = dma_ack_q;
    assign wb_dma_err = wb_dma_cyc && wb_dma_stb && !dma_addr_ok;
    wire fill_word = wr_ready && byte_index[1:0] == 2'd3;
    wire dma_write_word = dma_take && wb_dma_we && wb_dma_sel == 4'hf;
    wire ram_we = fill_word || dma_write_word;
    wire [11:0] ram_waddr = dma_write_word ? wb_dma_adr[11:0] :
                             byte_index[13:2];
    wire [31:0] ram_wdata = dma_write_word ? wb_dma_dat_w :
                            {write_accum[31:8], wr_data};
    always @(posedge clk) begin
        if (ram_we) ram[ram_waddr] <= ram_wdata;
        wb_dma_dat_r <= ram[wb_dma_adr[11:0]];
        stream_word <= ram[fetch_word_addr];
    end
    always @(posedge clk) begin
        if (rst) begin
            dma_ack_q <= 0;
            write_accum <= 0;
            debug_dma_words <= 0;
        end else begin
            dma_ack_q <= dma_take;
            if (go && state == S_IDLE) debug_dma_words <= 0;
            if (wr_ready) begin
                case (byte_index[1:0])
                    2'd0: write_accum[31:24] <= wr_data;
                    2'd1: write_accum[23:16] <= wr_data;
                    2'd2: write_accum[15:8] <= wr_data;
                    default: begin end
                endcase
            end
            if (dma_take) begin
                debug_dma_words <= debug_dma_words + 1'b1;
            end
        end
    end

    wire command_start = (state == S_CMD || state == S_STOP) && !command_busy;
    wire command_busy, command_done, command_error;
    wire [5:0] command_index = state == S_STOP ? 6'd12 :
                               cmd_type_q == 3'd1 ? 6'd17 :
                               cmd_type_q == 3'd2 ? 6'd18 :
                               cmd_type_q == 3'd3 ? 6'd24 : 6'd25;
    wire [31:0] command_argument = state == S_STOP ? 32'b0 : lba_q;
    wire [1:0] command_response_type = state == S_STOP ? 2'd3 : 2'd1;
    wire [1:0] command_data_type = state == S_STOP ? 2'd0 :
                                   writing ? 2'd2 : 2'd1;
    wire [127:0] command_response;
    wire command_timeout;
    wire cmd_csr_valid, cmd_csr_ready, cmd_csr_write;
    wire [15:0] cmd_csr_addr;
    wire [31:0] cmd_csr_wdata;
    wire cmd_csr_rsp_valid, cmd_csr_rsp_error, cmd_csr_rsp_timeout;
    wire [31:0] cmd_csr_rsp_rdata;
    sd_native_command #(.EVENT_TIMEOUT_CYCLES(EVENT_TIMEOUT_CYCLES))
    u_command (
        .clk(clk), .rst(rst), .start(command_start),
        .command(command_index), .argument(command_argument),
        .response_type(command_response_type), .data_type(command_data_type),
        .busy(command_busy), .done(command_done), .error(command_error),
        .timeout(command_timeout), .event_status(debug_cmd_event),
        .response(command_response),
        .csr_req_valid(cmd_csr_valid), .csr_req_ready(cmd_csr_ready),
        .csr_req_write(cmd_csr_write), .csr_req_addr(cmd_csr_addr),
        .csr_req_wdata(cmd_csr_wdata), .csr_rsp_valid(cmd_csr_rsp_valid),
        .csr_rsp_rdata(cmd_csr_rsp_rdata),
        .csr_rsp_error(cmd_csr_rsp_error),
        .csr_rsp_timeout(cmd_csr_rsp_timeout)
    );

    wire [15:0] dma_csr_base = writing ? 16'h2000 : 16'h0800;
    reg direct_csr_valid, direct_csr_write;
    reg [15:0] direct_csr_addr;
    reg [31:0] direct_csr_wdata;
    wire direct_csr_ready, direct_csr_rsp_valid, direct_csr_rsp_error;
    wire [31:0] direct_csr_rsp_rdata;
    always @(*) begin
        direct_csr_valid = 0;
        direct_csr_write = 0;
        direct_csr_addr = 0;
        direct_csr_wdata = 0;
        if (state == S_CFG) begin
            direct_csr_valid = 1;
            direct_csr_write = 1;
            case (cfg_step)
                3'd0: begin direct_csr_addr = dma_csr_base + 16'h000c; end
                3'd1: begin direct_csr_addr = dma_csr_base; end
                3'd2: begin direct_csr_addr = dma_csr_base + 16'h0004; end
                3'd3: begin
                    direct_csr_addr = dma_csr_base + 16'h0008;
                    direct_csr_wdata = {7'b0, blocks_q, 9'b0};
                end
                3'd4: begin
                    direct_csr_addr = dma_csr_base + 16'h000c;
                    direct_csr_wdata = 1;
                end
                3'd5: begin
                    direct_csr_addr = 16'h1024;
                    direct_csr_wdata = 512;
                end
                default: begin
                    direct_csr_addr = 16'h1028;
                    direct_csr_wdata = {16'b0, blocks_q};
                end
            endcase
        end else if (state == S_DATA || state == S_DMA) begin
            direct_csr_valid = 1;
            direct_csr_addr = state == S_DATA ? 16'h1020 :
                              dma_csr_base + 16'h0010;
        end
    end
    wire command_owner = command_busy || command_start;
    wire master_req_valid = command_owner ? cmd_csr_valid : direct_csr_valid;
    wire master_req_write = command_owner ? cmd_csr_write : direct_csr_write;
    wire [15:0] master_req_addr = command_owner ? cmd_csr_addr : direct_csr_addr;
    wire [31:0] master_req_wdata = command_owner ? cmd_csr_wdata : direct_csr_wdata;
    wire master_req_ready;
    wire master_rsp_valid, master_rsp_error, master_rsp_timeout;
    wire [31:0] master_rsp_rdata;
    assign cmd_csr_ready = command_owner && master_req_ready;
    assign direct_csr_ready = !command_owner && master_req_ready;
    assign cmd_csr_rsp_valid = command_owner && master_rsp_valid;
    assign cmd_csr_rsp_rdata = master_rsp_rdata;
    assign cmd_csr_rsp_error = master_rsp_error;
    assign cmd_csr_rsp_timeout = master_rsp_timeout;
    assign direct_csr_rsp_valid = !command_owner && master_rsp_valid;
    assign direct_csr_rsp_error = master_rsp_error;
    assign direct_csr_rsp_rdata = master_rsp_rdata;
    sd_native_csr_master #(.TIMEOUT_CYCLES(CSR_TIMEOUT_CYCLES)) u_csr (
        .clk(clk), .rst(rst),
        .req_valid(master_req_valid), .req_ready(master_req_ready),
        .req_write(master_req_write), .req_addr(master_req_addr),
        .req_wdata(master_req_wdata),
        .rsp_valid(master_rsp_valid), .rsp_rdata(master_rsp_rdata),
        .rsp_error(master_rsp_error), .rsp_timeout(master_rsp_timeout),
        .wb_adr(wb_ctrl_adr), .wb_bte(wb_ctrl_bte),
        .wb_cti(wb_ctrl_cti), .wb_cyc(wb_ctrl_cyc),
        .wb_dat_r(wb_ctrl_dat_r), .wb_dat_w(wb_ctrl_dat_w),
        .wb_ack(wb_ctrl_ack), .wb_err(wb_ctrl_err),
        .wb_sel(wb_ctrl_sel), .wb_stb(wb_ctrl_stb), .wb_we(wb_ctrl_we)
    );

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            cmd_type_q <= 0;
            lba_q <= 0;
            blocks_q <= 0;
            cfg_step <= 0;
            byte_index <= 0;
            poll_timer <= 0;
            done <= 0;
            error <= 0;
        end else begin
            done <= 0;
            if ((state == S_DATA || state == S_DATA_WAIT ||
                 state == S_DMA || state == S_DMA_WAIT) &&
                poll_timer == POLL_LAST) begin
                state <= S_IDLE;
                done <= 1;
                error <= 1;
            end else begin
                if (state == S_DATA || state == S_DATA_WAIT ||
                    state == S_DMA || state == S_DMA_WAIT)
                    poll_timer <= poll_timer + 1'b1;
                case (state)
                    S_IDLE: if (go) begin
                        error <= 0;
                        if (block_count == 0 || block_count > 32 ||
                            cmd_type < 3'd1 || cmd_type > 3'd4) begin
                            done <= 1;
                            error <= 1;
                        end else begin
                            cmd_type_q <= cmd_type;
                            lba_q <= lba;
                            blocks_q <= block_count;
                            byte_index <= 0;
                            cfg_step <= 0;
                            state <= cmd_type >= 3'd3 ? S_FILL : S_CFG;
                        end
                    end
                    S_FILL: if (wr_ready) begin
                        byte_index <= byte_index + 1'b1;
                        if (byte_index + 1'b1 == total_bytes) begin
                            cfg_step <= 0;
                            state <= S_CFG;
                        end
                    end
                    S_CFG: if (direct_csr_ready) state <= S_CFG_WAIT;
                    S_CFG_WAIT: if (direct_csr_rsp_valid) begin
                        if (direct_csr_rsp_error) begin
                            state <= S_IDLE; done <= 1; error <= 1;
                        end else if (cfg_step == 6) state <= S_CMD;
                        else begin
                            cfg_step <= cfg_step + 1'b1;
                            state <= S_CFG;
                        end
                    end
                    S_CMD: if (!command_busy) state <= S_CMD_WAIT;
                    S_CMD_WAIT: if (command_done) begin
                        if (command_error) begin
                            state <= S_IDLE; done <= 1; error <= 1;
                        end else begin
                            poll_timer <= 0;
                            state <= S_DATA;
                        end
                    end
                    S_DATA: if (direct_csr_ready) state <= S_DATA_WAIT;
                    S_DATA_WAIT: if (direct_csr_rsp_valid) begin
                        if (direct_csr_rsp_error || direct_csr_rsp_rdata[3:1] != 0) begin
                            state <= S_IDLE; done <= 1; error <= 1;
                        end else if (direct_csr_rsp_rdata[0]) begin
                            poll_timer <= 0;
                            state <= S_DMA;
                        end else state <= S_DATA;
                    end
                    S_DMA: if (direct_csr_ready) state <= S_DMA_WAIT;
                    S_DMA_WAIT: if (direct_csr_rsp_valid) begin
                        if (direct_csr_rsp_error) begin
                            state <= S_IDLE; done <= 1; error <= 1;
                        end else if (direct_csr_rsp_rdata[0]) begin
                            state <= multi ? S_STOP : writing ? S_IDLE : S_STREAM;
                            if (!multi && writing) done <= 1;
                            byte_index <= 0;
                        end else state <= S_DMA;
                    end
                    S_STOP: if (!command_busy) state <= S_STOP_WAIT;
                    S_STOP_WAIT: if (command_done) begin
                        if (command_error) begin
                            state <= S_IDLE; done <= 1; error <= 1;
                        end else if (writing) begin
                            state <= S_IDLE; done <= 1;
                        end else state <= S_STREAM;
                    end
                    S_STREAM: if (rd_valid) begin
                        byte_index <= byte_index + 1'b1;
                        if (byte_index + 1'b1 == total_bytes) begin
                            state <= S_IDLE;
                            done <= 1;
                        end
                    end
                    default: state <= S_IDLE;
                endcase
            end
        end
    end
`ifndef SYNTHESIS
    initial if (POLL_TIMEOUT_CYCLES < 1)
        $error("sd_native_block_engine POLL_TIMEOUT_CYCLES must be positive");
`endif
endmodule
`default_nettype wire
