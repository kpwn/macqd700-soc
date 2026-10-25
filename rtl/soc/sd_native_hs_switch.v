// Negotiate SD High Speed at the safe 25 MHz clock, using the CMD6 64-byte
// status data before changing LiteSDCard's divider to 50 MHz. The DMA port
// is exclusively owned by this module until done; ordinary block I/O starts
// only afterwards. If the card declines High Speed, stay at 25 MHz.
`default_nettype none
module sd_native_hs_switch #(
    parameter integer POLL_TIMEOUT_CYCLES = 100000000,
    parameter integer EVENT_TIMEOUT_CYCLES = 100000000,
    parameter integer CSR_TIMEOUT_CYCLES = 1024
) (
    input  wire        clk,
    input  wire        rst,
    output reg         done,
    output reg         error,
    output reg         high_speed_active,
    output wire        card_session_active,
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
    output wire [31:0] wb_dma_dat_r,
    input  wire [31:0] wb_dma_dat_w,
    output wire        wb_dma_err,
    input  wire [3:0]  wb_dma_sel,
    input  wire        wb_dma_stb,
    input  wire        wb_dma_we
);
    localparam [3:0] S_CFG = 0, S_CFG_WAIT = 1, S_CMD = 2,
        S_CMD_WAIT = 3, S_DATA = 4, S_DATA_WAIT = 5,
        S_DMA = 6, S_DMA_WAIT = 7, S_CHECK = 8,
        S_DIV = 9, S_DIV_WAIT = 10, S_DONE = 11, S_FAIL = 12;
    localparam integer POLL_W = $clog2(POLL_TIMEOUT_CYCLES + 1);
    localparam [POLL_W-1:0] POLL_LAST = POLL_TIMEOUT_CYCLES - 1;
    reg [3:0] state;
    reg [2:0] cfg_step;
    reg [POLL_W-1:0] poll_timer;
    reg [31:0] status_word3, status_word4;
    reg seen_word3, seen_word4;
    reg dma_ack_q;

    assign card_session_active = state == S_CMD || state == S_CMD_WAIT ||
                                 state == S_DATA || state == S_DATA_WAIT ||
                                 state == S_DMA || state == S_DMA_WAIT;
    wire dma_addr_ok = ~|wb_dma_adr[29:4];
    wire dma_take = wb_dma_cyc && wb_dma_stb && dma_addr_ok &&
                    !dma_ack_q && !rst && !done && !error;
    assign wb_dma_ack = dma_ack_q;
    assign wb_dma_err = wb_dma_cyc && wb_dma_stb && !dma_addr_ok;
    assign wb_dma_dat_r = 32'b0;
    always @(posedge clk) begin
        if (rst) begin
            dma_ack_q <= 0;
            status_word3 <= 0;
            status_word4 <= 0;
            seen_word3 <= 0;
            seen_word4 <= 0;
        end else begin
            dma_ack_q <= dma_take;
            if (dma_take && wb_dma_we && wb_dma_sel == 4'hf) begin
                if (wb_dma_adr[3:0] == 3) begin
                    status_word3 <= wb_dma_dat_w;
                    seen_word3 <= 1;
                end
                if (wb_dma_adr[3:0] == 4) begin
                    status_word4 <= wb_dma_dat_w;
                    seen_word4 <= 1;
                end
            end
        end
    end

    wire command_start = state == S_CMD && !command_busy;
    wire command_busy, command_done, command_error;
    wire cmd_req_valid, cmd_req_ready, cmd_req_write;
    wire [15:0] cmd_req_addr;
    wire [31:0] cmd_req_wdata;
    wire cmd_rsp_valid, cmd_rsp_error, cmd_rsp_timeout;
    wire [31:0] cmd_rsp_rdata;
    sd_native_command #(.EVENT_TIMEOUT_CYCLES(EVENT_TIMEOUT_CYCLES))
    u_command (
        .clk(clk), .rst(rst), .start(command_start),
        .command(6'd6), .argument(32'h80fffff1),
        .response_type(2'd1), .data_type(2'd1),
        .busy(command_busy), .done(command_done), .error(command_error),
        .timeout(), .event_status(), .response(),
        .csr_req_valid(cmd_req_valid), .csr_req_ready(cmd_req_ready),
        .csr_req_write(cmd_req_write), .csr_req_addr(cmd_req_addr),
        .csr_req_wdata(cmd_req_wdata), .csr_rsp_valid(cmd_rsp_valid),
        .csr_rsp_rdata(cmd_rsp_rdata), .csr_rsp_error(cmd_rsp_error),
        .csr_rsp_timeout(cmd_rsp_timeout)
    );

    reg direct_req_valid, direct_req_write;
    reg [15:0] direct_req_addr;
    reg [31:0] direct_req_wdata;
    wire direct_req_ready, direct_rsp_valid, direct_rsp_error;
    wire [31:0] direct_rsp_rdata;
    always @(*) begin
        direct_req_valid = 0;
        direct_req_write = 0;
        direct_req_addr = 0;
        direct_req_wdata = 0;
        if (state == S_CFG) begin
            direct_req_valid = 1;
            direct_req_write = 1;
            case (cfg_step)
                0: direct_req_addr = 16'h080c; // stop/reset read DMA
                1: direct_req_addr = 16'h0800; // DMA base = 0
                2: direct_req_addr = 16'h0804;
                3: begin direct_req_addr = 16'h0808; direct_req_wdata = 64; end
                4: begin direct_req_addr = 16'h080c; direct_req_wdata = 1; end
                5: begin direct_req_addr = 16'h1024; direct_req_wdata = 64; end
                default: begin direct_req_addr = 16'h1028; direct_req_wdata = 1; end
            endcase
        end else if (state == S_DATA || state == S_DMA) begin
            direct_req_valid = 1;
            direct_req_addr = state == S_DATA ? 16'h1020 : 16'h0810;
        end else if (state == S_DIV) begin
            direct_req_valid = 1;
            direct_req_write = 1;
            direct_req_addr = 16'h2804;
            direct_req_wdata = 2; // 100 MHz controller / 2 = 50 MHz SD
        end
    end
    wire command_owner = command_busy || command_start;
    wire master_req_valid = command_owner ? cmd_req_valid : direct_req_valid;
    wire master_req_write = command_owner ? cmd_req_write : direct_req_write;
    wire [15:0] master_req_addr = command_owner ? cmd_req_addr : direct_req_addr;
    wire [31:0] master_req_wdata = command_owner ? cmd_req_wdata : direct_req_wdata;
    wire master_req_ready, master_rsp_valid, master_rsp_error, master_rsp_timeout;
    wire [31:0] master_rsp_rdata;
    assign cmd_req_ready = command_owner && master_req_ready;
    assign direct_req_ready = !command_owner && master_req_ready;
    assign cmd_rsp_valid = command_owner && master_rsp_valid;
    assign cmd_rsp_rdata = master_rsp_rdata;
    assign cmd_rsp_error = master_rsp_error;
    assign cmd_rsp_timeout = master_rsp_timeout;
    assign direct_rsp_valid = !command_owner && master_rsp_valid;
    assign direct_rsp_error = master_rsp_error;
    assign direct_rsp_rdata = master_rsp_rdata;
    sd_native_csr_master #(.TIMEOUT_CYCLES(CSR_TIMEOUT_CYCLES))
    u_csr (
        .clk(clk), .rst(rst),
        .req_valid(master_req_valid), .req_ready(master_req_ready),
        .req_write(master_req_write), .req_addr(master_req_addr),
        .req_wdata(master_req_wdata),
        .rsp_valid(master_rsp_valid), .rsp_rdata(master_rsp_rdata),
        .rsp_error(master_rsp_error), .rsp_timeout(master_rsp_timeout),
        .wb_adr(wb_ctrl_adr), .wb_bte(wb_ctrl_bte), .wb_cti(wb_ctrl_cti),
        .wb_cyc(wb_ctrl_cyc), .wb_dat_r(wb_ctrl_dat_r),
        .wb_dat_w(wb_ctrl_dat_w), .wb_ack(wb_ctrl_ack),
        .wb_err(wb_ctrl_err), .wb_sel(wb_ctrl_sel),
        .wb_stb(wb_ctrl_stb), .wb_we(wb_ctrl_we)
    );

    always @(posedge clk) begin
        if (rst) begin
            state <= S_CFG;
            cfg_step <= 0;
            poll_timer <= 0;
            done <= 0;
            error <= 0;
            high_speed_active <= 0;
        end else begin
            if (state == S_DATA || state == S_DATA_WAIT ||
                state == S_DMA || state == S_DMA_WAIT) begin
                poll_timer <= poll_timer + 1'b1;
            end
            if ((state == S_DATA || state == S_DATA_WAIT ||
                 state == S_DMA || state == S_DMA_WAIT) &&
                poll_timer == POLL_LAST) state <= S_FAIL;
            else
            case (state)
                S_CFG: if (direct_req_ready) state <= S_CFG_WAIT;
                S_CFG_WAIT: if (direct_rsp_valid) begin
                    if (direct_rsp_error) state <= S_FAIL;
                    else if (cfg_step == 6) state <= S_CMD;
                    else begin cfg_step <= cfg_step + 1'b1; state <= S_CFG; end
                end
                S_CMD: if (!command_busy) state <= S_CMD_WAIT;
                S_CMD_WAIT: if (command_done) begin
                    if (command_error) state <= S_FAIL;
                    else begin poll_timer <= 0; state <= S_DATA; end
                end
                S_DATA: if (direct_req_ready) state <= S_DATA_WAIT;
                S_DATA_WAIT: if (direct_rsp_valid) begin
                    if (direct_rsp_error || direct_rsp_rdata[3:1] != 0)
                        state <= S_FAIL;
                    else if (direct_rsp_rdata[0]) begin
                        poll_timer <= 0;
                        state <= S_DMA;
                    end else state <= S_DATA;
                end
                S_DMA: if (direct_req_ready) state <= S_DMA_WAIT;
                S_DMA_WAIT: if (direct_rsp_valid) begin
                    if (direct_rsp_error) state <= S_FAIL;
                    else if (direct_rsp_rdata[0]) state <= S_CHECK;
                    else state <= S_DMA;
                end
                S_CHECK: begin
                    if (!seen_word3 || !seen_word4) state <= S_FAIL;
                    else if ((status_word3[23:16] & 8'h02) != 0 &&
                             status_word4[27:24] == 4'd1) state <= S_DIV;
                    else state <= S_DONE; // card remains in default-speed mode
                end
                S_DIV: if (direct_req_ready) state <= S_DIV_WAIT;
                S_DIV_WAIT: if (direct_rsp_valid) begin
                    if (direct_rsp_error) state <= S_FAIL;
                    else begin high_speed_active <= 1; state <= S_DONE; end
                end
                S_DONE: done <= 1;
                S_FAIL: error <= 1;
                default: state <= S_FAIL;
            endcase
        end
    end
`ifndef SYNTHESIS
    initial if (POLL_TIMEOUT_CYCLES < 1)
        $error("sd_native_hs_switch POLL_TIMEOUT_CYCLES must be positive");
`endif
endmodule
`default_nettype wire
