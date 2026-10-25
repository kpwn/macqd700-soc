// Initialize a freshly powered SDHC/SDXC card through LiteSDCard's CSR and
// command interfaces. This sequence enters native four-bit default-speed
// mode and obtains capacity from CSD v2. High Speed CMD6 selection follows
// in the block/DMA controller, which can inspect the returned 64-byte data.
`default_nettype none
module sd_native_init #(
    parameter integer INIT_WAIT_CYCLES = 500000,
    parameter integer RETRY_WAIT_CYCLES = 100000,
    parameter integer CMD_GAP_CYCLES = 4096,
    parameter integer INIT_DIVIDER = 256,
    parameter integer RUN_DIVIDER = 4,
    parameter integer OCR_RETRIES = 1000
) (
    input  wire         clk,
    input  wire         rst,
    output reg          ready,
    output reg          error,
    output reg  [31:0]  card_num_lbas,
    output reg  [31:0]  rca,
    output wire [4:0]   debug_phase,

    output wire         csr_req_valid,
    input  wire         csr_req_ready,
    output wire         csr_req_write,
    output wire [15:0]  csr_req_addr,
    output wire [31:0]  csr_req_wdata,
    input  wire         csr_rsp_valid,
    input  wire         csr_rsp_error,

    output wire         cmd_start,
    input  wire         cmd_busy,
    output reg  [5:0]   cmd_index,
    output reg  [31:0]  cmd_argument,
    output reg  [1:0]   cmd_response_type,
    output wire [1:0]   cmd_data_type,
    input  wire         cmd_done,
    input  wire         cmd_error,
    input  wire [127:0] cmd_response
);
    localparam [4:0]
        P_DIV_INIT    = 5'd0,
        P_INIT_CLOCKS = 5'd1,
        P_CMD0        = 5'd2,
        P_CMD8        = 5'd3,
        P_DIV_RUN     = 5'd4,
        P_CMD55       = 5'd5,
        P_ACMD41      = 5'd6,
        P_CMD2        = 5'd7,
        P_CMD3        = 5'd8,
        P_CMD9        = 5'd9,
        P_CMD7        = 5'd10,
        P_CMD55_WIDTH = 5'd11,
        P_ACMD6       = 5'd12,
        P_CMD16       = 5'd13,
        P_READY       = 5'd14;
    localparam [1:0] S_LAUNCH = 2'd0,
                     S_WAIT   = 2'd1,
                     S_DELAY  = 2'd2,
                     S_HALT   = 2'd3;
    localparam [15:0] LAST_OCR_TRY = OCR_RETRIES - 1;
    reg [4:0] phase;
    reg [1:0] state;
    reg [4:0] delay_next_phase;
    reg [31:0] delay_count;
    reg [15:0] ocr_tries;
    assign debug_phase = phase;
    wire phase_is_csr = phase == P_DIV_INIT || phase == P_INIT_CLOCKS ||
                        phase == P_DIV_RUN;
    assign csr_req_valid = !rst && state == S_LAUNCH && phase_is_csr;
    assign csr_req_write = 1'b1;
    assign csr_req_addr = phase == P_INIT_CLOCKS ? 16'h2808 : 16'h2804;
    assign csr_req_wdata = phase == P_DIV_INIT ? INIT_DIVIDER :
                           phase == P_INIT_CLOCKS ? 32'd1 : RUN_DIVIDER;
    assign cmd_start = !rst && state == S_LAUNCH && !phase_is_csr &&
                       phase != P_READY && !cmd_busy;
    assign cmd_data_type = 2'b00;
    wire [32:0] csd_lbas = ({11'b0, cmd_response[69:48]} + 33'd1) << 10;

    always @(*) begin
        cmd_index = 0;
        cmd_argument = 0;
        cmd_response_type = 1;
        case (phase)
            P_CMD0: begin cmd_index = 0; cmd_response_type = 0; end
            P_CMD8: begin cmd_index = 8; cmd_argument = 32'h000001aa; end
            P_CMD55: begin cmd_index = 55; end
            P_ACMD41: begin
                cmd_index = 41;
                cmd_argument = 32'h70ff8000;
                cmd_response_type = 3;
            end
            P_CMD2: begin cmd_index = 2; cmd_response_type = 2; end
            P_CMD3: begin cmd_index = 3; end
            P_CMD9: begin
                cmd_index = 9; cmd_argument = rca; cmd_response_type = 2;
            end
            P_CMD7: begin
                cmd_index = 7; cmd_argument = rca; cmd_response_type = 3;
            end
            P_CMD55_WIDTH: begin cmd_index = 55; cmd_argument = rca; end
            P_ACMD6: begin cmd_index = 6; cmd_argument = 2; end
            P_CMD16: begin cmd_index = 16; cmd_argument = 512; end
            default: begin end
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            phase <= P_DIV_INIT;
            state <= S_LAUNCH;
            delay_next_phase <= P_CMD0;
            delay_count <= 0;
            ocr_tries <= 0;
            ready <= 0;
            error <= 0;
            card_num_lbas <= 0;
            rca <= 0;
        end else case (state)
            S_LAUNCH: begin
                if (phase_is_csr) begin
                    if (csr_req_ready) state <= S_WAIT;
                end else if (phase != P_READY && !cmd_busy) begin
                    state <= S_WAIT;
                end
            end
            S_WAIT: begin
                if (phase_is_csr) begin
                    if (csr_rsp_valid) begin
                        if (csr_rsp_error) begin
                            error <= 1;
                            state <= S_HALT;
                        end else case (phase)
                            P_DIV_INIT: begin
                                phase <= P_INIT_CLOCKS;
                                state <= S_LAUNCH;
                            end
                            P_INIT_CLOCKS: begin
                                delay_count <= INIT_WAIT_CYCLES - 1;
                                delay_next_phase <= P_CMD0;
                                state <= S_DELAY;
                            end
                            P_DIV_RUN: begin
                                // The card stays at identification speed
                                // until ACMD41 reports ready. Only then can
                                // the clock rise to the transfer rate.
                                phase <= P_CMD2;
                                state <= S_LAUNCH;
                            end
                            default: begin error <= 1; state <= S_HALT; end
                        endcase
                    end
                end else if (cmd_done) begin
                    if (cmd_error) begin
                        error <= 1;
                        state <= S_HALT;
                    end else case (phase)
                        P_CMD0: begin
                            delay_count <= CMD_GAP_CYCLES - 1;
                            delay_next_phase <= P_CMD8;
                            state <= S_DELAY;
                        end
                        P_CMD8: begin
                            if (cmd_response[11:0] != 12'h1aa) begin
                                error <= 1; state <= S_HALT;
                            end else begin
                                delay_count <= CMD_GAP_CYCLES - 1;
                                delay_next_phase <= P_CMD55;
                                state <= S_DELAY;
                            end
                        end
                        P_CMD55: begin
                            delay_count <= CMD_GAP_CYCLES - 1;
                            delay_next_phase <= P_ACMD41;
                            state <= S_DELAY;
                        end
                        P_ACMD41: begin
                            if (cmd_response[31]) begin
                                if (!cmd_response[30]) begin
                                    error <= 1; state <= S_HALT;
                                end else begin
                                    delay_count <= CMD_GAP_CYCLES - 1;
                                    delay_next_phase <= P_DIV_RUN;
                                    state <= S_DELAY;
                                end
                            end else if (ocr_tries == LAST_OCR_TRY) begin
                                error <= 1; state <= S_HALT;
                            end else begin
                                ocr_tries <= ocr_tries + 1'b1;
                                delay_count <= RETRY_WAIT_CYCLES - 1;
                                delay_next_phase <= P_CMD55;
                                state <= S_DELAY;
                            end
                        end
                        P_CMD2: begin
                            delay_count <= CMD_GAP_CYCLES - 1;
                            delay_next_phase <= P_CMD3;
                            state <= S_DELAY;
                        end
                        P_CMD3: begin
                            rca <= {cmd_response[31:16], 16'b0};
                            delay_count <= CMD_GAP_CYCLES - 1;
                            delay_next_phase <= P_CMD9;
                            state <= S_DELAY;
                        end
                        P_CMD9: begin
                            if (cmd_response[127:126] != 2'b01 || csd_lbas[32]) begin
                                error <= 1; state <= S_HALT;
                            end else begin
                                card_num_lbas <= csd_lbas[31:0];
                                delay_count <= CMD_GAP_CYCLES - 1;
                                delay_next_phase <= P_CMD7;
                                state <= S_DELAY;
                            end
                        end
                        P_CMD7: begin
                            delay_count <= CMD_GAP_CYCLES - 1;
                            delay_next_phase <= P_CMD55_WIDTH;
                            state <= S_DELAY;
                        end
                        P_CMD55_WIDTH: begin
                            delay_count <= CMD_GAP_CYCLES - 1;
                            delay_next_phase <= P_ACMD6;
                            state <= S_DELAY;
                        end
                        P_ACMD6: begin
                            delay_count <= CMD_GAP_CYCLES - 1;
                            delay_next_phase <= P_CMD16;
                            state <= S_DELAY;
                        end
                        P_CMD16: begin
                            delay_count <= CMD_GAP_CYCLES - 1;
                            delay_next_phase <= P_READY;
                            state <= S_DELAY;
                        end
                        default: begin error <= 1; state <= S_HALT; end
                    endcase
                end
            end
            S_DELAY: begin
                if (delay_count == 0) begin
                    phase <= delay_next_phase;
                    if (delay_next_phase == P_READY) begin
                        ready <= 1;
                        state <= S_HALT;
                    end else state <= S_LAUNCH;
                end else delay_count <= delay_count - 1'b1;
            end
            default: begin end
        endcase
    end
`ifndef SYNTHESIS
    initial if (INIT_WAIT_CYCLES < 1 || RETRY_WAIT_CYCLES < 1 ||
                CMD_GAP_CYCLES < 1 || INIT_DIVIDER < 2 ||
                INIT_DIVIDER > 511 || RUN_DIVIDER < 2 ||
                RUN_DIVIDER > 511 ||
                OCR_RETRIES < 1 || OCR_RETRIES > 65536)
        $error("sd_native_init timing/retry parameters invalid");
`endif
endmodule
`default_nettype wire
