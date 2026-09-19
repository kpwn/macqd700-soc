// net_block_framer.sv -- Ethernet/IPv4/UDP framing for the network block link.
//
// This module is deliberately only a byte-stream framing boundary.  It owns no
// SCSI state, storage buffer policy, DMA, configuration registers, ARP, or MAC
// arbitration.  In particular, dst_mac is already the next-hop address: a
// later CSR block may change it, but this block never tries to resolve it.
//
// All multi-byte fields below are emitted and consumed most-significant byte
// first (network byte order).  The SoC's AXI memory byte-lane convention does
// NOT apply here; every AXI-stream beat is exactly the next byte on the wire.
// Taxi supplies/accepts Ethernet frames without preamble or FCS.
//
// Application protocol contract (byte offsets are within the UDP payload):
//
//   Request:  0..3  magic       = 0x4e424844 (ASCII "NBHD")
//             4     op          = 0x00 read, 0x01 write
//             5..8  LBA         unsigned 32-bit, network order
//             9..10 block_count unsigned 16-bit, network order
//            11..12 tag         unsigned 16-bit, network order
//            13..   write data  exactly block_count * 512 bytes; absent on read
//
//   Reply:    0..3  magic       = 0x4e424844 (ASCII "NBHD")
//             4..5  tag         unsigned 16-bit, network order
//             6     status      0x00 success; other values are daemon-defined
//             7..   payload     length is UDP length minus 15; normally absent
//                               for writes and block_count * 512 for reads
//
// A request is accepted only when it fits the IPv4 1500-byte MTU.  This means
// at most two 512-byte blocks in a write datagram.  Larger transfers must be
// windowed by the future transaction layer; read request headers themselves
// remain small regardless of their requested count.  IPv4 options and
// fragmentation are intentionally unsupported.
`default_nettype none

module net_block_framer (
    input  wire        clk,
    input  wire        rst,

    input  wire [47:0] our_mac,
    input  wire [31:0] our_ip,
    input  wire [15:0] our_port,
    input  wire [47:0] dst_mac,
    input  wire [31:0] dst_ip,
    input  wire [15:0] dst_port,

    input  wire        req_valid,
    output wire        req_ready,
    input  wire [7:0]  req_op,
    input  wire [31:0] req_lba,
    input  wire [15:0] req_block_count,
    input  wire [15:0] req_tag,

    input  wire [7:0]  req_payload_tdata,
    input  wire        req_payload_tvalid,
    output wire        req_payload_tready,
    input  wire        req_payload_tlast,

    output wire [7:0]  tx_tdata,
    output wire        tx_tvalid,
    input  wire        tx_tready,
    output wire        tx_tlast,

    input  wire [7:0]  rx_tdata,
    input  wire        rx_tvalid,
    output wire        rx_tready,
    input  wire        rx_tlast,

    output wire        reply_valid,
    input  wire        reply_ready,
    output wire [15:0] reply_tag,
    output wire [7:0]  reply_status,

    output wire [7:0]  reply_payload_tdata,
    output wire        reply_payload_tvalid,
    input  wire        reply_payload_tready,
    output wire        reply_payload_tlast
);
    // Packet lengths mix protocol-sized fields with elaboration-time integer
    // constants.  Every narrowing point is bounded by the MTU checks below.
    /* verilator lint_off WIDTHEXPAND */
    /* verilator lint_off WIDTHTRUNC */
    localparam [31:0] PROTOCOL_MAGIC = 32'h4e424844;
    localparam [7:0] OP_READ = 8'h00;
    localparam [7:0] OP_WRITE = 8'h01;
    localparam [15:0] ETHERTYPE_IPV4 = 16'h0800;
    localparam [7:0] IPV4_VERSION_IHL = 8'h45;
    localparam [7:0] IPV4_PROTOCOL_UDP = 8'd17;
    localparam [7:0] IPV4_TTL = 8'd64;
    localparam [15:0] IPV4_FLAG_DF = 16'h4000;
    localparam integer IPV4_MTU_BYTES = 1500;
    localparam integer ETH_HEADER_BYTES = 14;
    localparam integer IPV4_HEADER_BYTES = 20;
    localparam integer UDP_HEADER_BYTES = 8;
    localparam integer REQUEST_HEADER_BYTES = 13;
    localparam integer REPLY_HEADER_BYTES = 7;
    localparam integer BLOCK_BYTES = 512;
    localparam integer TX_HEADER_BYTES = ETH_HEADER_BYTES + IPV4_HEADER_BYTES +
        UDP_HEADER_BYTES + REQUEST_HEADER_BYTES;
    localparam integer RX_PAYLOAD_OFFSET = ETH_HEADER_BYTES + IPV4_HEADER_BYTES +
        UDP_HEADER_BYTES + REPLY_HEADER_BYTES;
    localparam integer MAX_REPLY_PAYLOAD = IPV4_MTU_BYTES - IPV4_HEADER_BYTES -
        UDP_HEADER_BYTES - REPLY_HEADER_BYTES;
    localparam integer MAX_WRITE_PAYLOAD = IPV4_MTU_BYTES - IPV4_HEADER_BYTES -
        UDP_HEADER_BYTES - REQUEST_HEADER_BYTES;

    localparam [1:0] TX_IDLE = 2'd0, TX_HEADER = 2'd1, TX_PAYLOAD = 2'd2;
    localparam [1:0] RX_CAPTURE = 2'd0, RX_PRESENT = 2'd1, RX_PAYLOAD = 2'd2;

    reg [1:0] tx_state;
    reg [5:0] tx_index;
    reg [16:0] tx_payload_left;
    reg [47:0] tx_our_mac, tx_dst_mac;
    reg [31:0] tx_our_ip, tx_dst_ip, tx_lba;
    reg [15:0] tx_our_port, tx_dst_port, tx_count, tx_tag;
    reg [7:0] tx_op;
    reg [15:0] tx_ip_total_len, tx_udp_len, tx_ip_checksum;

    wire [24:0] requested_payload_bytes = {9'd0, req_block_count} << 9;
    wire request_op_valid = (req_op == OP_READ) || (req_op == OP_WRITE);
    // A read request carries no payload, so its SIZE was previously waved
    // through unconditionally -- but the REPLY has to carry block_count*512
    // back, and anything over two blocks cannot fit under the MTU.  Accepting
    // such a request would put a frame on the wire that the host daemon can
    // only answer with STATUS_TOO_MANY_BLOCKS, i.e. a guaranteed round-trip
    // failure that the transaction layer would then retry.  Bound it here,
    // against the REPLY budget, so it fails at the seam that knows the limit.
    // Both directions therefore cap at 2 blocks (1024 <= 1459 and <= 1465;
    // three blocks is 1536 and fits neither).
    wire request_size_valid = (req_block_count != 16'd0) &&
        ((req_op == OP_READ) ? (requested_payload_bytes <= MAX_REPLY_PAYLOAD)
                             : (requested_payload_bytes <= MAX_WRITE_PAYLOAD));
    assign req_ready = (tx_state == TX_IDLE) && request_op_valid && request_size_valid;

    function automatic [15:0] ipv4_checksum;
        input [15:0] total_len;
        input [15:0] identification;
        input [31:0] source_ip;
        input [31:0] destination_ip;
        reg [19:0] sum;
        reg [16:0] fold;
        begin
            sum = {4'd0, 16'h4500} + total_len + identification + IPV4_FLAG_DF +
                {IPV4_TTL, IPV4_PROTOCOL_UDP} + source_ip[31:16] +
                source_ip[15:0] + destination_ip[31:16] + destination_ip[15:0];
            fold = {1'b0, sum[15:0]} + sum[19:16];
            fold = {1'b0, fold[15:0]} + fold[16];
            ipv4_checksum = ~fold[15:0];
        end
    endfunction

    reg [7:0] tx_header_byte;
    always @(*) begin
        tx_header_byte = 8'd0;
        case (tx_index)
            0: tx_header_byte = tx_dst_mac[47:40];
            1: tx_header_byte = tx_dst_mac[39:32];
            2: tx_header_byte = tx_dst_mac[31:24];
            3: tx_header_byte = tx_dst_mac[23:16];
            4: tx_header_byte = tx_dst_mac[15:8];
            5: tx_header_byte = tx_dst_mac[7:0];
            6: tx_header_byte = tx_our_mac[47:40];
            7: tx_header_byte = tx_our_mac[39:32];
            8: tx_header_byte = tx_our_mac[31:24];
            9: tx_header_byte = tx_our_mac[23:16];
            10: tx_header_byte = tx_our_mac[15:8];
            11: tx_header_byte = tx_our_mac[7:0];
            12: tx_header_byte = ETHERTYPE_IPV4[15:8];
            13: tx_header_byte = ETHERTYPE_IPV4[7:0];
            14: tx_header_byte = IPV4_VERSION_IHL;
            15: tx_header_byte = 8'h00;
            16: tx_header_byte = tx_ip_total_len[15:8];
            17: tx_header_byte = tx_ip_total_len[7:0];
            18: tx_header_byte = tx_tag[15:8];
            19: tx_header_byte = tx_tag[7:0];
            20: tx_header_byte = IPV4_FLAG_DF[15:8];
            21: tx_header_byte = IPV4_FLAG_DF[7:0];
            22: tx_header_byte = IPV4_TTL;
            23: tx_header_byte = IPV4_PROTOCOL_UDP;
            24: tx_header_byte = tx_ip_checksum[15:8];
            25: tx_header_byte = tx_ip_checksum[7:0];
            26: tx_header_byte = tx_our_ip[31:24];
            27: tx_header_byte = tx_our_ip[23:16];
            28: tx_header_byte = tx_our_ip[15:8];
            29: tx_header_byte = tx_our_ip[7:0];
            30: tx_header_byte = tx_dst_ip[31:24];
            31: tx_header_byte = tx_dst_ip[23:16];
            32: tx_header_byte = tx_dst_ip[15:8];
            33: tx_header_byte = tx_dst_ip[7:0];
            34: tx_header_byte = tx_our_port[15:8];
            35: tx_header_byte = tx_our_port[7:0];
            36: tx_header_byte = tx_dst_port[15:8];
            37: tx_header_byte = tx_dst_port[7:0];
            38: tx_header_byte = tx_udp_len[15:8];
            39: tx_header_byte = tx_udp_len[7:0];
            // A zero UDP checksum means "not computed" in IPv4 (RFC 768).
            // If checksum generation is ever added and its result is zero,
            // the encoded value MUST instead be 0xffff.
            40, 41: tx_header_byte = 8'h00;
            42: tx_header_byte = PROTOCOL_MAGIC[31:24];
            43: tx_header_byte = PROTOCOL_MAGIC[23:16];
            44: tx_header_byte = PROTOCOL_MAGIC[15:8];
            45: tx_header_byte = PROTOCOL_MAGIC[7:0];
            46: tx_header_byte = tx_op;
            47: tx_header_byte = tx_lba[31:24];
            48: tx_header_byte = tx_lba[23:16];
            49: tx_header_byte = tx_lba[15:8];
            50: tx_header_byte = tx_lba[7:0];
            51: tx_header_byte = tx_count[15:8];
            52: tx_header_byte = tx_count[7:0];
            53: tx_header_byte = tx_tag[15:8];
            54: tx_header_byte = tx_tag[7:0];
            default: tx_header_byte = 8'd0;
        endcase
    end

    assign tx_tdata = (tx_state == TX_HEADER) ? tx_header_byte : req_payload_tdata;
    assign tx_tvalid = (tx_state == TX_HEADER) ||
        ((tx_state == TX_PAYLOAD) && req_payload_tvalid);
    assign tx_tlast = ((tx_state == TX_HEADER) && (tx_index == TX_HEADER_BYTES-1) &&
        (tx_payload_left == 0)) || ((tx_state == TX_PAYLOAD) &&
        req_payload_tvalid && (tx_payload_left == 1));
    assign req_payload_tready = (tx_state == TX_PAYLOAD) && tx_tready;

    always @(posedge clk) begin
        if (rst) begin
            tx_state <= TX_IDLE;
            tx_index <= 0;
            tx_payload_left <= 0;
            tx_our_mac <= 0;
            tx_dst_mac <= 0;
            tx_our_ip <= 0;
            tx_dst_ip <= 0;
            tx_lba <= 0;
            tx_our_port <= 0;
            tx_dst_port <= 0;
            tx_count <= 0;
            tx_tag <= 0;
            tx_op <= 0;
            tx_ip_total_len <= 0;
            tx_udp_len <= 0;
            tx_ip_checksum <= 0;
        end else begin
            case (tx_state)
                TX_IDLE: if (req_valid && req_ready) begin
                    tx_our_mac <= our_mac;
                    tx_dst_mac <= dst_mac;
                    tx_our_ip <= our_ip;
                    tx_dst_ip <= dst_ip;
                    tx_our_port <= our_port;
                    tx_dst_port <= dst_port;
                    tx_lba <= req_lba;
                    tx_count <= req_block_count;
                    tx_tag <= req_tag;
                    tx_op <= req_op;
                    tx_udp_len <= UDP_HEADER_BYTES + REQUEST_HEADER_BYTES +
                        ((req_op == OP_WRITE) ? requested_payload_bytes[15:0] : 16'd0);
                    tx_ip_total_len <= IPV4_HEADER_BYTES + UDP_HEADER_BYTES +
                        REQUEST_HEADER_BYTES +
                        ((req_op == OP_WRITE) ? requested_payload_bytes[15:0] : 16'd0);
                    tx_ip_checksum <= ipv4_checksum(
                        IPV4_HEADER_BYTES + UDP_HEADER_BYTES + REQUEST_HEADER_BYTES +
                            ((req_op == OP_WRITE) ? requested_payload_bytes[15:0] : 16'd0),
                        req_tag, our_ip, dst_ip);
                    tx_payload_left <= (req_op == OP_WRITE) ?
                        requested_payload_bytes[16:0] : 17'd0;
                    tx_index <= 0;
                    tx_state <= TX_HEADER;
                end
                TX_HEADER: if (tx_tvalid && tx_tready) begin
                    if (tx_index == TX_HEADER_BYTES-1) begin
                        tx_index <= 0;
                        tx_state <= (tx_payload_left == 0) ? TX_IDLE : TX_PAYLOAD;
                    end else begin
                        tx_index <= tx_index + 1'b1;
                    end
                end
                TX_PAYLOAD: if (req_payload_tvalid && tx_tready) begin
                    tx_payload_left <= tx_payload_left - 1'b1;
                    if (tx_payload_left == 1)
                        tx_state <= TX_IDLE;
                end
                default: tx_state <= TX_IDLE;
            endcase
        end
    end

`ifndef SYNTHESIS
    // The byte count determines the IP/UDP lengths before transmission.  An
    // upstream early/late tlast is therefore a contract violation, not a way
    // to resize a frame after its headers are already on the wire.
    always @(posedge clk) begin
        if (!rst && tx_state == TX_PAYLOAD && req_payload_tvalid && tx_tready &&
            (req_payload_tlast != (tx_payload_left == 1)))
            $error("net_block_framer write payload tlast does not match block_count");
    end
`endif

    reg [1:0] rx_state;
    reg [11:0] rx_index;
    reg rx_bad;
    reg [15:0] rx_ip_total_len, rx_udp_len, rx_flags_fragment;
    reg [7:0] rx_ip_high_byte;
    reg [19:0] rx_ip_sum;
    reg [15:0] captured_tag;
    reg [7:0] captured_status;
    reg [10:0] captured_payload_len;
    reg [10:0] reply_payload_index;
    reg [7:0] reply_payload_mem [0:MAX_REPLY_PAYLOAD-1];

    wire rx_fire = rx_tvalid && rx_tready;
    wire [16:0] rx_checksum_fold1 = {1'b0, rx_ip_sum[15:0]} + rx_ip_sum[19:16];
    wire [16:0] rx_checksum_fold2 = {1'b0, rx_checksum_fold1[15:0]} +
        rx_checksum_fold1[16];
    wire rx_checksum_valid = (rx_checksum_fold2[15:0] == 16'hffff);
    wire [15:0] expected_udp_len = rx_ip_total_len - IPV4_HEADER_BYTES;
    wire [15:0] expected_reply_payload_len = rx_udp_len -
        (UDP_HEADER_BYTES + REPLY_HEADER_BYTES);
    wire rx_lengths_valid = (rx_ip_total_len >= IPV4_HEADER_BYTES +
        UDP_HEADER_BYTES + REPLY_HEADER_BYTES) &&
        (rx_ip_total_len <= IPV4_MTU_BYTES) &&
        (rx_udp_len >= UDP_HEADER_BYTES + REPLY_HEADER_BYTES) &&
        (rx_udp_len == expected_udp_len) &&
        (expected_reply_payload_len <= MAX_REPLY_PAYLOAD) &&
        ((rx_index + 1'b1) >= (ETH_HEADER_BYTES + rx_ip_total_len));

    reg rx_fixed_mismatch;
    always @(*) begin
        rx_fixed_mismatch = 1'b0;
        case (rx_index)
            0: rx_fixed_mismatch = (rx_tdata != our_mac[47:40]);
            1: rx_fixed_mismatch = (rx_tdata != our_mac[39:32]);
            2: rx_fixed_mismatch = (rx_tdata != our_mac[31:24]);
            3: rx_fixed_mismatch = (rx_tdata != our_mac[23:16]);
            4: rx_fixed_mismatch = (rx_tdata != our_mac[15:8]);
            5: rx_fixed_mismatch = (rx_tdata != our_mac[7:0]);
            12: rx_fixed_mismatch = (rx_tdata != ETHERTYPE_IPV4[15:8]);
            13: rx_fixed_mismatch = (rx_tdata != ETHERTYPE_IPV4[7:0]);
            14: rx_fixed_mismatch = (rx_tdata != IPV4_VERSION_IHL);
            23: rx_fixed_mismatch = (rx_tdata != IPV4_PROTOCOL_UDP);
            30: rx_fixed_mismatch = (rx_tdata != our_ip[31:24]);
            31: rx_fixed_mismatch = (rx_tdata != our_ip[23:16]);
            32: rx_fixed_mismatch = (rx_tdata != our_ip[15:8]);
            33: rx_fixed_mismatch = (rx_tdata != our_ip[7:0]);
            36: rx_fixed_mismatch = (rx_tdata != our_port[15:8]);
            37: rx_fixed_mismatch = (rx_tdata != our_port[7:0]);
            42: rx_fixed_mismatch = (rx_tdata != PROTOCOL_MAGIC[31:24]);
            43: rx_fixed_mismatch = (rx_tdata != PROTOCOL_MAGIC[23:16]);
            44: rx_fixed_mismatch = (rx_tdata != PROTOCOL_MAGIC[15:8]);
            45: rx_fixed_mismatch = (rx_tdata != PROTOCOL_MAGIC[7:0]);
            default: rx_fixed_mismatch = 1'b0;
        endcase
    end

    assign rx_tready = (rx_state == RX_CAPTURE);
    assign reply_valid = (rx_state == RX_PRESENT);
    assign reply_tag = captured_tag;
    assign reply_status = captured_status;
    assign reply_payload_tdata = reply_payload_mem[reply_payload_index];
    assign reply_payload_tvalid = (rx_state == RX_PAYLOAD);
    assign reply_payload_tlast = (rx_state == RX_PAYLOAD) &&
        (reply_payload_index + 1'b1 == captured_payload_len);

    always @(posedge clk) begin
        if (rst) begin
            rx_state <= RX_CAPTURE;
            rx_index <= 0;
            rx_bad <= 0;
            rx_ip_total_len <= 0;
            rx_udp_len <= 0;
            rx_flags_fragment <= 0;
            rx_ip_high_byte <= 0;
            rx_ip_sum <= 0;
            captured_tag <= 0;
            captured_status <= 0;
            captured_payload_len <= 0;
            reply_payload_index <= 0;
        end else begin
            case (rx_state)
                RX_CAPTURE: if (rx_fire) begin
                    if (rx_index == 0) begin
                        rx_bad <= rx_fixed_mismatch;
                        rx_ip_total_len <= 0;
                        rx_udp_len <= 0;
                        rx_flags_fragment <= 0;
                        rx_ip_sum <= 0;
                    end else if (rx_fixed_mismatch ||
                        (rx_index >= ETH_HEADER_BYTES + IPV4_MTU_BYTES)) begin
                        rx_bad <= 1'b1;
                    end

                    if ((rx_index >= ETH_HEADER_BYTES) &&
                        (rx_index < ETH_HEADER_BYTES + IPV4_HEADER_BYTES)) begin
                        if (!rx_index[0])
                            rx_ip_high_byte <= rx_tdata;
                        else
                            rx_ip_sum <= rx_ip_sum + {rx_ip_high_byte, rx_tdata};
                    end
                    case (rx_index)
                        16: rx_ip_total_len[15:8] <= rx_tdata;
                        17: rx_ip_total_len[7:0] <= rx_tdata;
                        20: rx_flags_fragment[15:8] <= rx_tdata;
                        21: rx_flags_fragment[7:0] <= rx_tdata;
                        38: rx_udp_len[15:8] <= rx_tdata;
                        39: rx_udp_len[7:0] <= rx_tdata;
                        46: captured_tag[15:8] <= rx_tdata;
                        47: captured_tag[7:0] <= rx_tdata;
                        48: captured_status <= rx_tdata;
                        default: begin end
                    endcase

                    if ((rx_index >= RX_PAYLOAD_OFFSET) &&
                        (rx_index < ETH_HEADER_BYTES + IPV4_HEADER_BYTES + rx_udp_len) &&
                        ((rx_index - RX_PAYLOAD_OFFSET) < MAX_REPLY_PAYLOAD))
                        reply_payload_mem[rx_index - RX_PAYLOAD_OFFSET] <= rx_tdata;

                    if (rx_tlast) begin
                        rx_index <= 0;
                        if (!rx_bad && !rx_fixed_mismatch && rx_lengths_valid &&
                            rx_checksum_valid && ((rx_flags_fragment & 16'h3fff) == 0)) begin
                            captured_payload_len <= expected_reply_payload_len[10:0];
                            reply_payload_index <= 0;
                            rx_state <= RX_PRESENT;
                        end else begin
                            rx_bad <= 0;
                        end
                    end else begin
                        rx_index <= rx_index + 1'b1;
                    end
                end
                RX_PRESENT: if (reply_valid && reply_ready) begin
                    if (captured_payload_len == 0)
                        rx_state <= RX_CAPTURE;
                    else begin
                        reply_payload_index <= 0;
                        rx_state <= RX_PAYLOAD;
                    end
                end
                RX_PAYLOAD: if (reply_payload_tvalid && reply_payload_tready) begin
                    if (reply_payload_tlast) begin
                        reply_payload_index <= 0;
                        rx_state <= RX_CAPTURE;
                    end else begin
                        reply_payload_index <= reply_payload_index + 1'b1;
                    end
                end
                default: rx_state <= RX_CAPTURE;
            endcase
        end
    end
endmodule

`default_nettype wire
