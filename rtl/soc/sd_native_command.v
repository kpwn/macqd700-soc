// Execute one LiteSDCard command through its CSR register bank. All CSR
// accesses and the card-completion poll are bounded. The four response
// words are returned in card bit order, high word first.
`default_nettype none
module sd_native_command #(
    parameter integer EVENT_TIMEOUT_CYCLES = 100000000
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         start,
    input  wire [5:0]   command,
    input  wire [31:0]  argument,
    input  wire [1:0]   response_type,
    input  wire [1:0]   data_type,
    output wire         busy,
    output reg          done,
    output reg          error,
    output reg          timeout,
    output reg  [3:0]   event_status,
    output reg  [127:0] response,

    output reg          csr_req_valid,
    input  wire         csr_req_ready,
    output reg          csr_req_write,
    output reg  [15:0]  csr_req_addr,
    output reg  [31:0]  csr_req_wdata,
    input  wire         csr_rsp_valid,
    input  wire [31:0]  csr_rsp_rdata,
    input  wire         csr_rsp_error,
    input  wire         csr_rsp_timeout
);
    localparam [3:0]
        S_IDLE       = 4'd0,
        S_ARG        = 4'd1,
        S_ARG_WAIT   = 4'd2,
        S_CMD        = 4'd3,
        S_CMD_WAIT   = 4'd4,
        S_SEND       = 4'd5,
        S_SEND_WAIT  = 4'd6,
        S_EVENT      = 4'd7,
        S_EVENT_WAIT = 4'd8,
        S_RESPONSE   = 4'd9,
        S_RSP_WAIT   = 4'd10;
    localparam integer TIMER_W = $clog2(EVENT_TIMEOUT_CYCLES + 1);
    localparam [TIMER_W-1:0] LAST_CYCLE = EVENT_TIMEOUT_CYCLES - 1;
    reg [3:0] state;
    reg [5:0] command_q;
    reg [31:0] argument_q;
    reg [1:0] response_type_q;
    reg [1:0] data_type_q;
    reg [1:0] response_word;
    reg [TIMER_W-1:0] event_timer;
    assign busy = state != S_IDLE;

    always @(*) begin
        csr_req_valid = 0;
        csr_req_write = 0;
        csr_req_addr = 0;
        csr_req_wdata = 0;
        case (state)
            S_ARG: begin
                csr_req_valid = 1;
                csr_req_write = 1;
                csr_req_addr = 16'h1000;
                csr_req_wdata = argument_q;
            end
            S_CMD: begin
                csr_req_valid = 1;
                csr_req_write = 1;
                csr_req_addr = 16'h1004;
                csr_req_wdata = {18'b0, command_q, 1'b0, data_type_q,
                                 3'b0, response_type_q};
            end
            S_SEND: begin
                csr_req_valid = 1;
                csr_req_write = 1;
                csr_req_addr = 16'h1008;
                csr_req_wdata = 1;
            end
            S_EVENT: begin
                csr_req_valid = 1;
                csr_req_addr = 16'h101c;
            end
            S_RESPONSE: begin
                csr_req_valid = 1;
                csr_req_addr = 16'h100c + {12'b0, response_word, 2'b0};
            end
            default: begin end
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            command_q <= 0;
            argument_q <= 0;
            response_type_q <= 0;
            data_type_q <= 0;
            response_word <= 0;
            event_timer <= 0;
            done <= 0;
            error <= 0;
            timeout <= 0;
            event_status <= 0;
            response <= 0;
        end else begin
            done <= 0;
            if (state != S_IDLE && state >= S_EVENT &&
                event_timer != LAST_CYCLE)
                event_timer <= event_timer + 1'b1;
            if (state != S_IDLE && state >= S_EVENT &&
                event_timer == LAST_CYCLE) begin
                state <= S_IDLE;
                done <= 1;
                error <= 1;
                timeout <= 1;
            end else case (state)
                S_IDLE: if (start) begin
                    command_q <= command;
                    argument_q <= argument;
                    response_type_q <= response_type;
                    data_type_q <= data_type;
                    response_word <= 0;
                    event_timer <= 0;
                    event_status <= 0;
                    response <= 0;
                    error <= 0;
                    timeout <= 0;
                    state <= S_ARG;
                end
                S_ARG: if (csr_req_ready) state <= S_ARG_WAIT;
                S_ARG_WAIT: if (csr_rsp_valid) begin
                    if (csr_rsp_error) begin
                        state <= S_IDLE; done <= 1; error <= 1;
                        timeout <= csr_rsp_timeout;
                    end else state <= S_CMD;
                end
                S_CMD: if (csr_req_ready) state <= S_CMD_WAIT;
                S_CMD_WAIT: if (csr_rsp_valid) begin
                    if (csr_rsp_error) begin
                        state <= S_IDLE; done <= 1; error <= 1;
                        timeout <= csr_rsp_timeout;
                    end else state <= S_SEND;
                end
                S_SEND: if (csr_req_ready) state <= S_SEND_WAIT;
                S_SEND_WAIT: if (csr_rsp_valid) begin
                    if (csr_rsp_error) begin
                        state <= S_IDLE; done <= 1; error <= 1;
                        timeout <= csr_rsp_timeout;
                    end else state <= S_EVENT;
                end
                S_EVENT: if (csr_req_ready) state <= S_EVENT_WAIT;
                S_EVENT_WAIT: if (csr_rsp_valid) begin
                    if (csr_rsp_error) begin
                        state <= S_IDLE; done <= 1; error <= 1;
                        timeout <= csr_rsp_timeout;
                    end else if (csr_rsp_rdata[0]) begin
                        event_status <= csr_rsp_rdata[3:0];
                        if (csr_rsp_rdata[3:1] != 0) begin
                            state <= S_IDLE; done <= 1; error <= 1;
                            timeout <= csr_rsp_rdata[2];
                        end else state <= S_RESPONSE;
                    end else state <= S_EVENT;
                end
                S_RESPONSE: if (csr_req_ready) state <= S_RSP_WAIT;
                S_RSP_WAIT: if (csr_rsp_valid) begin
                    if (csr_rsp_error) begin
                        state <= S_IDLE; done <= 1; error <= 1;
                        timeout <= csr_rsp_timeout;
                    end else begin
                        case (response_word)
                            2'd0: response[127:96] <= csr_rsp_rdata;
                            2'd1: response[95:64]  <= csr_rsp_rdata;
                            2'd2: response[63:32]  <= csr_rsp_rdata;
                            2'd3: response[31:0]   <= csr_rsp_rdata;
                        endcase
                        if (response_word == 3) begin
                            state <= S_IDLE;
                            done <= 1;
                        end else begin
                            response_word <= response_word + 1'b1;
                            state <= S_RESPONSE;
                        end
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end
`ifndef SYNTHESIS
    initial if (EVENT_TIMEOUT_CYCLES < 1)
        $error("sd_native_command EVENT_TIMEOUT_CYCLES must be positive");
`endif
endmodule
`default_nettype wire
