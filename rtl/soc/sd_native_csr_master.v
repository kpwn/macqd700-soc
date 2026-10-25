// One-outstanding Wishbone CSR master for LiteSDCard. The native storage
// sequencer presents byte addresses; this module converts to Wishbone word
// addresses and guarantees a terminal response even if the CSR bus stalls.
`default_nettype none
module sd_native_csr_master #(
    parameter integer TIMEOUT_CYCLES = 1024
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        req_valid,
    output wire        req_ready,
    input  wire        req_write,
    input  wire [15:0] req_addr,
    input  wire [31:0] req_wdata,
    output reg         rsp_valid,
    output reg  [31:0] rsp_rdata,
    output reg         rsp_error,
    output reg         rsp_timeout,

    output reg  [29:0] wb_adr,
    output wire [1:0]  wb_bte,
    output wire [2:0]  wb_cti,
    output wire        wb_cyc,
    input  wire [31:0] wb_dat_r,
    output reg  [31:0] wb_dat_w,
    input  wire        wb_ack,
    input  wire        wb_err,
    output wire [3:0]  wb_sel,
    output wire        wb_stb,
    output reg         wb_we
);
    localparam integer TIMER_W = $clog2(TIMEOUT_CYCLES + 1);
    localparam [TIMER_W-1:0] LAST_CYCLE = TIMEOUT_CYCLES - 1;
    reg busy;
    reg [TIMER_W-1:0] timer;
    assign req_ready = !busy;
    assign wb_cyc = busy;
    assign wb_stb = busy;
    assign wb_bte = 2'b00;
    assign wb_cti = 3'b000;
    assign wb_sel = 4'hf;

    always @(posedge clk) begin
        if (rst) begin
            busy <= 0;
            timer <= 0;
            wb_adr <= 0;
            wb_dat_w <= 0;
            wb_we <= 0;
            rsp_valid <= 0;
            rsp_rdata <= 0;
            rsp_error <= 0;
            rsp_timeout <= 0;
        end else begin
            rsp_valid <= 0;
            if (!busy) begin
                if (req_valid) begin
                    wb_adr <= {16'b0, req_addr[15:2]};
                    wb_dat_w <= req_wdata;
                    wb_we <= req_write;
                    timer <= 0;
                    busy <= 1;
                end
            end else if (wb_ack || wb_err) begin
                busy <= 0;
                rsp_valid <= 1;
                rsp_rdata <= wb_dat_r;
                rsp_error <= wb_err;
                rsp_timeout <= 0;
            end else if (timer == LAST_CYCLE) begin
                busy <= 0;
                rsp_valid <= 1;
                rsp_rdata <= 0;
                rsp_error <= 1;
                rsp_timeout <= 1;
            end else begin
                timer <= timer + 1'b1;
            end
        end
    end
`ifndef SYNTHESIS
    initial if (TIMEOUT_CYCLES < 1)
        $error("sd_native_csr_master TIMEOUT_CYCLES must be positive");
`endif
endmodule
`default_nettype wire
