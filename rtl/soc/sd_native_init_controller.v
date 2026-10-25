// Connect native SD initialization, command sequencing, and bounded CSR
// Wishbone access. The command engine owns the CSR port from start through
// its final response; between commands the init FSM can set PHY registers.
`default_nettype none
module sd_native_init_controller #(
    parameter integer INIT_WAIT_CYCLES = 500000,
    parameter integer RETRY_WAIT_CYCLES = 100000,
    parameter integer CMD_GAP_CYCLES = 4096,
    parameter integer OCR_RETRIES = 1000,
    parameter integer EVENT_TIMEOUT_CYCLES = 100000000,
    parameter integer CSR_TIMEOUT_CYCLES = 1024
) (
    input  wire        clk,
    input  wire        rst,
    output wire        ready,
    output wire        error,
    output wire [31:0] card_num_lbas,
    output wire [31:0] rca,
    output wire [4:0]  debug_phase,
    output wire [3:0]  debug_cmd_event,
    output wire        debug_cmd_timeout,

    output wire [29:0] wb_adr,
    output wire [1:0]  wb_bte,
    output wire [2:0]  wb_cti,
    output wire        wb_cyc,
    input  wire [31:0] wb_dat_r,
    output wire [31:0] wb_dat_w,
    input  wire        wb_ack,
    input  wire        wb_err,
    output wire [3:0]  wb_sel,
    output wire        wb_stb,
    output wire        wb_we
);
    wire init_req_valid, init_req_ready, init_req_write;
    wire [15:0] init_req_addr;
    wire [31:0] init_req_wdata;
    wire init_rsp_valid, init_rsp_error;
    wire cmd_start, cmd_busy, cmd_done, cmd_error;
    wire [5:0] cmd_index;
    wire [31:0] cmd_argument;
    wire [1:0] cmd_response_type, cmd_data_type;
    wire [127:0] cmd_response;

    sd_native_init #(
        .INIT_WAIT_CYCLES(INIT_WAIT_CYCLES),
        .RETRY_WAIT_CYCLES(RETRY_WAIT_CYCLES),
        .CMD_GAP_CYCLES(CMD_GAP_CYCLES),
        .OCR_RETRIES(OCR_RETRIES)
    ) u_init (
        .clk(clk), .rst(rst), .ready(ready), .error(error),
        .card_num_lbas(card_num_lbas), .rca(rca),
        .debug_phase(debug_phase),
        .csr_req_valid(init_req_valid), .csr_req_ready(init_req_ready),
        .csr_req_write(init_req_write), .csr_req_addr(init_req_addr),
        .csr_req_wdata(init_req_wdata), .csr_rsp_valid(init_rsp_valid),
        .csr_rsp_error(init_rsp_error),
        .cmd_start(cmd_start), .cmd_busy(cmd_busy), .cmd_index(cmd_index),
        .cmd_argument(cmd_argument), .cmd_response_type(cmd_response_type),
        .cmd_data_type(cmd_data_type), .cmd_done(cmd_done),
        .cmd_error(cmd_error), .cmd_response(cmd_response)
    );

    wire cmd_req_valid, cmd_req_ready, cmd_req_write;
    wire [15:0] cmd_req_addr;
    wire [31:0] cmd_req_wdata;
    wire cmd_rsp_valid, cmd_rsp_error, cmd_rsp_timeout;
    wire [31:0] cmd_rsp_rdata;
    sd_native_command #(
        .EVENT_TIMEOUT_CYCLES(EVENT_TIMEOUT_CYCLES)
    ) u_command (
        .clk(clk), .rst(rst), .start(cmd_start),
        .command(cmd_index), .argument(cmd_argument),
        .response_type(cmd_response_type), .data_type(cmd_data_type),
        .busy(cmd_busy), .done(cmd_done), .error(cmd_error),
        .timeout(debug_cmd_timeout), .event_status(debug_cmd_event),
        .response(cmd_response),
        .csr_req_valid(cmd_req_valid), .csr_req_ready(cmd_req_ready),
        .csr_req_write(cmd_req_write), .csr_req_addr(cmd_req_addr),
        .csr_req_wdata(cmd_req_wdata), .csr_rsp_valid(cmd_rsp_valid),
        .csr_rsp_rdata(cmd_rsp_rdata), .csr_rsp_error(cmd_rsp_error),
        .csr_rsp_timeout(cmd_rsp_timeout)
    );

    // Include the launch cycle, before u_command's busy FF has risen.
    wire command_owner = cmd_busy || cmd_start;
    wire master_req_valid = command_owner ? cmd_req_valid : init_req_valid;
    wire master_req_write = command_owner ? cmd_req_write : init_req_write;
    wire [15:0] master_req_addr = command_owner ? cmd_req_addr : init_req_addr;
    wire [31:0] master_req_wdata = command_owner ? cmd_req_wdata : init_req_wdata;
    wire master_req_ready;
    wire master_rsp_valid, master_rsp_error, master_rsp_timeout;
    wire [31:0] master_rsp_rdata;
    assign cmd_req_ready = command_owner && master_req_ready;
    assign init_req_ready = !command_owner && master_req_ready;
    assign cmd_rsp_valid = command_owner && master_rsp_valid;
    assign cmd_rsp_rdata = master_rsp_rdata;
    assign cmd_rsp_error = master_rsp_error;
    assign cmd_rsp_timeout = master_rsp_timeout;
    assign init_rsp_valid = !command_owner && master_rsp_valid;
    assign init_rsp_error = master_rsp_error;

    sd_native_csr_master #(
        .TIMEOUT_CYCLES(CSR_TIMEOUT_CYCLES)
    ) u_csr (
        .clk(clk), .rst(rst),
        .req_valid(master_req_valid), .req_ready(master_req_ready),
        .req_write(master_req_write), .req_addr(master_req_addr),
        .req_wdata(master_req_wdata),
        .rsp_valid(master_rsp_valid), .rsp_rdata(master_rsp_rdata),
        .rsp_error(master_rsp_error), .rsp_timeout(master_rsp_timeout),
        .wb_adr(wb_adr), .wb_bte(wb_bte), .wb_cti(wb_cti),
        .wb_cyc(wb_cyc), .wb_dat_r(wb_dat_r), .wb_dat_w(wb_dat_w),
        .wb_ack(wb_ack), .wb_err(wb_err), .wb_sel(wb_sel),
        .wb_stb(wb_stb), .wb_we(wb_we)
    );
endmodule
`default_nettype wire
