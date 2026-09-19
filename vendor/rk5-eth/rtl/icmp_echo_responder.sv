`timescale 1ns / 1ps
`default_nettype none

// Minimal, fixed-address Ethernet endpoint for board bring-up.
//
// The MAC removes/checks the FCS and presents one byte per cycle.  This block
// buffers one frame, recognizes ARP requests and IPv4 ICMP echo requests, and
// emits the corresponding reply.  It deliberately has no software-visible
// state; the byte-stream boundary is the intended future SONIC/DMA seam.
module icmp_echo_responder #(
    parameter logic [47:0] LOCAL_MAC = 48'h02_00_00_00_00_01,
    parameter logic [31:0] LOCAL_IP  = 32'hc0_a8_64_02,
    parameter integer MAX_FRAME_LEN = 1536
)(
    input  wire logic       clk,
    input  wire logic       rst,

    input  wire logic [7:0] rx_tdata,
    input  wire logic       rx_tvalid,
    input  wire logic       rx_tlast,
    input  wire logic       rx_tuser,
    output wire logic       rx_tready,

    output wire logic [7:0] tx_tdata,
    output wire logic       tx_tvalid,
    output wire logic       tx_tlast,
    output wire logic       tx_tuser,
    input  wire logic       tx_tready,

    output wire logic       rx_activity,
    output wire logic       reply_activity,
    output wire logic [31:0] arp_reply_count,
    output wire logic [31:0] icmp_reply_count
);

localparam integer PTR_W = $clog2(MAX_FRAME_LEN);

typedef enum logic [1:0] {RX_FRAME, CHECK_FRAME, TX_FRAME} state_t;
state_t state_reg = RX_FRAME;

logic [7:0] frame_mem [0:MAX_FRAME_LEN-1];
logic [PTR_W-1:0] rx_ptr_reg = '0;
logic [PTR_W-1:0] tx_ptr_reg = '0;
logic [PTR_W:0] frame_len_reg = '0;
logic [PTR_W:0] tx_len_reg = '0;
logic frame_bad_reg = 1'b0;
logic rx_activity_reg = 1'b0;
logic reply_activity_reg = 1'b0;
logic [31:0] arp_reply_count_reg = '0;
logic [31:0] icmp_reply_count_reg = '0;

wire [16:0] icmp_checksum_add =
    {1'b0, frame_mem[36], frame_mem[37]} + 17'h0_0800;
wire [15:0] icmp_checksum_fold =
    icmp_checksum_add[15:0] + {{15{1'b0}}, icmp_checksum_add[16]};

wire is_arp_request =
    !frame_bad_reg && frame_len_reg >= 42 &&
    frame_mem[12] == 8'h08 && frame_mem[13] == 8'h06 &&
    frame_mem[14] == 8'h00 && frame_mem[15] == 8'h01 &&
    frame_mem[16] == 8'h08 && frame_mem[17] == 8'h00 &&
    frame_mem[18] == 8'h06 && frame_mem[19] == 8'h04 &&
    frame_mem[20] == 8'h00 && frame_mem[21] == 8'h01 &&
    {frame_mem[38], frame_mem[39], frame_mem[40], frame_mem[41]} == LOCAL_IP;

wire is_icmp_echo_request =
    !frame_bad_reg && frame_len_reg >= 42 &&
    {frame_mem[0], frame_mem[1], frame_mem[2], frame_mem[3],
     frame_mem[4], frame_mem[5]} == LOCAL_MAC &&
    frame_mem[12] == 8'h08 && frame_mem[13] == 8'h00 &&
    frame_mem[14] == 8'h45 &&
    frame_mem[20][5:0] == 6'd0 && frame_mem[21] == 8'h00 &&
    frame_mem[23] == 8'h01 &&
    {frame_mem[30], frame_mem[31], frame_mem[32], frame_mem[33]} == LOCAL_IP &&
    frame_mem[34] == 8'h08 && frame_mem[35] == 8'h00 &&
    {frame_mem[16], frame_mem[17]} >= 16'd28 &&
    {1'b0, frame_mem[16], frame_mem[17]} <= {{(17-(PTR_W+1)){1'b0}}, frame_len_reg} - 17'd14;

assign rx_tready = state_reg == RX_FRAME;
assign tx_tdata = frame_mem[tx_ptr_reg];
assign tx_tvalid = state_reg == TX_FRAME;
assign tx_tlast = state_reg == TX_FRAME && {1'b0, tx_ptr_reg} == tx_len_reg-1'b1;
assign tx_tuser = 1'b0;
assign rx_activity = rx_activity_reg;
assign reply_activity = reply_activity_reg;
assign arp_reply_count = arp_reply_count_reg;
assign icmp_reply_count = icmp_reply_count_reg;

always_ff @(posedge clk) begin
    if (rst) begin
        state_reg <= RX_FRAME;
        rx_ptr_reg <= '0;
        tx_ptr_reg <= '0;
        frame_len_reg <= '0;
        tx_len_reg <= '0;
        frame_bad_reg <= 1'b0;
        rx_activity_reg <= 1'b0;
        reply_activity_reg <= 1'b0;
        arp_reply_count_reg <= '0;
        icmp_reply_count_reg <= '0;
    end else begin
        case (state_reg)
            RX_FRAME: begin
                if (rx_tvalid && rx_tready) begin
                    rx_activity_reg <= ~rx_activity_reg;
                    frame_bad_reg <= frame_bad_reg | rx_tuser;

                    if (rx_ptr_reg < PTR_W'(MAX_FRAME_LEN-1))
                        frame_mem[rx_ptr_reg] <= rx_tdata;
                    else
                        frame_bad_reg <= 1'b1;

                    if (rx_tlast) begin
                        frame_len_reg <= {1'b0, rx_ptr_reg} + 1'b1;
                        rx_ptr_reg <= '0;
                        state_reg <= CHECK_FRAME;
                    end else begin
                        rx_ptr_reg <= rx_ptr_reg + 1'b1;
                    end
                end
            end

            CHECK_FRAME: begin
                frame_bad_reg <= 1'b0;

                if (is_arp_request) begin
                    // Ethernet destination = request source.
                    frame_mem[0] <= frame_mem[6];
                    frame_mem[1] <= frame_mem[7];
                    frame_mem[2] <= frame_mem[8];
                    frame_mem[3] <= frame_mem[9];
                    frame_mem[4] <= frame_mem[10];
                    frame_mem[5] <= frame_mem[11];
                    {frame_mem[6], frame_mem[7], frame_mem[8],
                     frame_mem[9], frame_mem[10], frame_mem[11]} <= LOCAL_MAC;
                    frame_mem[20] <= 8'h00;
                    frame_mem[21] <= 8'h02;
                    // ARP target = original sender.
                    frame_mem[32] <= frame_mem[22];
                    frame_mem[33] <= frame_mem[23];
                    frame_mem[34] <= frame_mem[24];
                    frame_mem[35] <= frame_mem[25];
                    frame_mem[36] <= frame_mem[26];
                    frame_mem[37] <= frame_mem[27];
                    frame_mem[38] <= frame_mem[28];
                    frame_mem[39] <= frame_mem[29];
                    frame_mem[40] <= frame_mem[30];
                    frame_mem[41] <= frame_mem[31];
                    {frame_mem[22], frame_mem[23], frame_mem[24],
                     frame_mem[25], frame_mem[26], frame_mem[27]} <= LOCAL_MAC;
                    {frame_mem[28], frame_mem[29], frame_mem[30],
                     frame_mem[31]} <= LOCAL_IP;
                    tx_ptr_reg <= '0;
                    tx_len_reg <= 42;
                    state_reg <= TX_FRAME;
                    arp_reply_count_reg <= arp_reply_count_reg + 1'b1;
                    reply_activity_reg <= ~reply_activity_reg;
                end else if (is_icmp_echo_request) begin
                    // Ethernet and IPv4 destinations become request sources.
                    frame_mem[0] <= frame_mem[6];
                    frame_mem[1] <= frame_mem[7];
                    frame_mem[2] <= frame_mem[8];
                    frame_mem[3] <= frame_mem[9];
                    frame_mem[4] <= frame_mem[10];
                    frame_mem[5] <= frame_mem[11];
                    {frame_mem[6], frame_mem[7], frame_mem[8],
                     frame_mem[9], frame_mem[10], frame_mem[11]} <= LOCAL_MAC;
                    frame_mem[30] <= frame_mem[26];
                    frame_mem[31] <= frame_mem[27];
                    frame_mem[32] <= frame_mem[28];
                    frame_mem[33] <= frame_mem[29];
                    {frame_mem[26], frame_mem[27], frame_mem[28],
                     frame_mem[29]} <= LOCAL_IP;
                    frame_mem[34] <= 8'h00;
                    // Incremental checksum update for ICMP type 8 -> 0.
                    frame_mem[36] <= icmp_checksum_fold[15:8];
                    frame_mem[37] <= icmp_checksum_fold[7:0];
                    tx_ptr_reg <= '0;
                    tx_len_reg <= frame_len_reg;
                    state_reg <= TX_FRAME;
                    icmp_reply_count_reg <= icmp_reply_count_reg + 1'b1;
                    reply_activity_reg <= ~reply_activity_reg;
                end else begin
                    state_reg <= RX_FRAME;
                end
            end

            TX_FRAME: begin
                if (tx_tvalid && tx_tready) begin
                    if (tx_tlast) begin
                        tx_ptr_reg <= '0;
                        state_reg <= RX_FRAME;
                    end else begin
                        tx_ptr_reg <= tx_ptr_reg + 1'b1;
                    end
                end
            end

            default: state_reg <= RX_FRAME;
        endcase
    end
end

endmodule

`default_nettype wire
