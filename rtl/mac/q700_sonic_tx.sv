// MAME reference/excerpt/adaptation attribution: Copyright Patrick Mackinlay.
// MAME-derived portions retain BSD-3-Clause; original contributions are MIT.
// See THIRD_PARTY_NOTICES.md and LICENSES/MAME-BSD-3-Clause.txt.
// q700_sonic_tx.sv -- SONIC transmit-descriptor engine.
//
// Descriptor semantics live here; the shared dma_engine remains peripheral
// neutral.  All memory operations use its byte-granular 1..64 byte client
// contract.  Payload fragments are split into 64-byte chunks and requests are
// issued on consecutive cycles when the DMA client is ready.  Responses may
// return out of order and are reassembled in a small frame buffer before the
// byte-wide MAC stream is started.
`default_nettype none
module q700_sonic_tx #(
    parameter integer MAX_FRAGMENTS = 16,
    parameter integer MAX_CHUNKS = 48
) (
    input  wire         clk,
    input  wire         rst,

    input  wire         start_valid,
    output wire         start_ready,
    input  wire [15:0]  start_dcr,
    input  wire [15:0]  start_utda,
    input  wire [15:0]  start_ctda,

    output reg  [31:0]  dma_req_addr,
    output reg  [6:0]   dma_req_len,
    output reg  [511:0] dma_req_wdata,
    output reg  [7:0]   dma_req_tag,
    output reg          dma_req_write,
    output reg          dma_req_valid,
    input  wire         dma_req_ready,
    input  wire [511:0] dma_rsp_rdata,
    input  wire [7:0]   dma_rsp_tag,
    input  wire [6:0]   dma_rsp_len,
    input  wire [1:0]   dma_rsp_status,
    input  wire         dma_rsp_write,
    input  wire         dma_rsp_valid,
    output wire         dma_rsp_ready,

    output wire [7:0]   tx_axis_tdata,
    output wire         tx_axis_tvalid,
    input  wire         tx_axis_tready,
    output wire         tx_axis_tlast,
    input  wire         tx_cpl_valid,
    output wire         tx_cpl_ready,

    output reg          done_valid,
    input  wire         done_ready,
    output reg          done_error,
    // TCR_PINT (0x8000) in the descriptor's config half asks for an interrupt
    // on THIS frame's completion.  descriptor_tcr keeps only the config bits
    // (& 0xF000) and done_tcr is masked to the status half (& 0x07FF), so the
    // request would otherwise be lost between the two.
    output reg          done_pint,
    // CR_HTX.  ds:1656-1659: HTX "halts the transmit command after the
    // current transmission has completed... The SONIC samples this bit after
    // writing to the TXpkt.status field" -- which is exactly ST_STATUS_WAIT
    // below.  Without this port the engine had no halt at all: writing HTX
    // cleared the CR bit while the engine kept walking the list to EOL.
    input  wire         halt,
    output reg  [15:0]  done_ctda,
    output reg  [15:0]  done_tcr,
    output reg  [15:0]  done_tps,
    output reg  [15:0]  done_tfc,

    output wire [3:0]   dbg_state,
    output wire [31:0]  dbg_descriptor_addr
);
    // Parameterized array depths make Verilator conservatively size several
    // indices and constant comparisons.  Runtime bounds are enforced below.
    /* verilator lint_off WIDTHTRUNC */
    /* verilator lint_off WIDTHEXPAND */
    localparam [15:0] TCR_PTX = 16'h0001;
    localparam [3:0]
        ST_IDLE       = 4'd0,
        ST_HDR_REQ    = 4'd1,
        ST_HDR_WAIT   = 4'd2,
        ST_FRAG_REQ   = 4'd3,
        ST_FRAG_WAIT  = 4'd4,
        ST_PAYLOAD    = 4'd5,
        ST_COLLECT    = 4'd6,
        ST_STREAM     = 4'd7,
        ST_CPL        = 4'd8,
        ST_STATUS_REQ = 4'd9,
        ST_STATUS_WAIT= 4'd10,
        ST_LINK_REQ   = 4'd11,
        ST_LINK_WAIT  = 4'd12,
        ST_DONE       = 4'd13,
        ST_PRIME      = 4'd14;
    localparam [7:0] TAG_HEADER = 8'hf0;
    localparam [7:0] TAG_FRAGMENT = 8'hf1;
    localparam [7:0] TAG_STATUS = 8'hf2;
    localparam [7:0] TAG_LINK = 8'hf3;

    reg [3:0] state;
    reg wide_desc;
    reg [2:0] word_bytes;
    reg [15:0] upper_addr;
    reg [31:0] descriptor_addr;
    reg [15:0] descriptor_tcr;
    reg [11:0] fragment_count;
    reg [4:0] fragment_index;
    reg [31:0] fragment_addr [0:MAX_FRAGMENTS-1];
    reg [15:0] fragment_len [0:MAX_FRAGMENTS-1];
    reg [4:0] issue_fragment;
    reg [15:0] issue_offset;
    reg [5:0] chunks_issued;
    reg [5:0] chunks_received;
    reg [5:0] total_chunks;
    (* ram_style = "block" *) reg [511:0] chunk_data [0:MAX_CHUNKS-1];
    reg [5:0] chunk_read_addr;
    reg [511:0] chunk_read_data;
    reg [6:0] chunk_len [0:MAX_CHUNKS-1];
    reg [5:0] stream_chunk;
    reg [6:0] stream_byte;
    reg [511:0] stream_shift;
    reg failed;
    integer k;

    function automatic [15:0] descriptor_word;
        input [511:0] data;
        input integer word_index;
        input bit is_wide;
        integer byte_index;
        begin
            byte_index = word_index * (is_wide ? 4 : 2) + (is_wide ? 2 : 0);
            descriptor_word = {data[byte_index*8 +: 8],
                               data[(byte_index+1)*8 +: 8]};
        end
    endfunction

    wire [15:0] current_fragment_remaining =
        fragment_len[issue_fragment] - issue_offset;
    wire [6:0] next_chunk_len =
        (current_fragment_remaining > 16'd64) ? 7'd64 :
                                                current_fragment_remaining[6:0];
    wire payload_req_fire = (state == ST_PAYLOAD) && dma_req_valid && dma_req_ready;
    wire payload_rsp = dma_rsp_valid && (dma_rsp_tag < MAX_CHUNKS);
    wire [15:0] link_rsp_word = descriptor_word(dma_rsp_rdata, 0, wide_desc);
    // TDA layout is status, config, packet size, fragment count.  The
    // status word is written back after completion, but is still part of the
    // descriptor header and must not be mistaken for TCR/config.
    wire [15:0] header_tfc_word = descriptor_word(dma_rsp_rdata, 3, wide_desc);

    assign start_ready = (state == ST_IDLE);
    assign dma_rsp_ready = 1'b1;
    assign tx_axis_tvalid = (state == ST_STREAM);
    assign tx_axis_tdata = stream_shift[7:0];
    assign tx_axis_tlast = (state == ST_STREAM) &&
                           (stream_chunk == total_chunks-1'b1) &&
                           (stream_byte == chunk_len[stream_chunk]-1'b1);
    assign tx_cpl_ready = (state == ST_CPL);
    assign dbg_state = state;
    assign dbg_descriptor_addr = descriptor_addr;

    // Give synthesis one unambiguous synchronous read port and one write
    // port.  chunk_read_addr is advanced while the current 64-byte chunk is
    // streaming, so the next chunk is available without inserting a bubble.
    always @(posedge clk) begin
        chunk_read_data <= chunk_data[chunk_read_addr];
        if (payload_rsp)
            chunk_data[dma_rsp_tag[5:0]] <= dma_rsp_rdata;
    end

    always @(*) begin
        dma_req_addr = 32'd0;
        dma_req_len = 7'd0;
        dma_req_wdata = 512'd0;
        dma_req_tag = 8'd0;
        dma_req_write = 1'b0;
        dma_req_valid = 1'b0;
        case (state)
            ST_HDR_REQ: begin
                dma_req_addr = descriptor_addr;
                dma_req_len = word_bytes * 4;
                dma_req_tag = TAG_HEADER;
                dma_req_valid = 1'b1;
            end
            ST_FRAG_REQ: begin
                dma_req_addr = descriptor_addr + (4 + fragment_index*3)*word_bytes;
                dma_req_len = word_bytes * 3;
                dma_req_tag = TAG_FRAGMENT;
                dma_req_valid = 1'b1;
            end
            ST_PAYLOAD: begin
                if ((issue_fragment < fragment_count) &&
                    (chunks_issued < MAX_CHUNKS)) begin
                    dma_req_addr = fragment_addr[issue_fragment] + issue_offset;
                    dma_req_len = next_chunk_len;
                    dma_req_tag = chunks_issued;
                    dma_req_valid = (next_chunk_len != 0);
                end
            end
            ST_STATUS_REQ: begin
                dma_req_addr = descriptor_addr;
                dma_req_len = word_bytes;
                dma_req_wdata[7:0] = wide_desc ? 8'h00 : TCR_PTX[15:8];
                dma_req_wdata[15:8] = wide_desc ? 8'h00 : TCR_PTX[7:0];
                dma_req_wdata[23:16] = wide_desc ? TCR_PTX[15:8] : 8'h00;
                dma_req_wdata[31:24] = wide_desc ? TCR_PTX[7:0] : 8'h00;
                dma_req_tag = TAG_STATUS;
                dma_req_write = 1'b1;
                dma_req_valid = 1'b1;
            end
            ST_LINK_REQ: begin
                dma_req_addr = descriptor_addr + (4 + fragment_count*3)*word_bytes;
                dma_req_len = word_bytes;
                dma_req_tag = TAG_LINK;
                dma_req_valid = 1'b1;
            end
            default: begin end
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            state <= ST_IDLE;
            done_valid <= 1'b0;
            done_error <= 1'b0;
            done_pint <= 1'b0;
            done_ctda <= 16'd0;
            done_tcr <= 16'd0;
            done_tps <= 16'd0;
            done_tfc <= 16'd0;
            chunk_read_addr <= 0;
            for (k = 0; k < MAX_CHUNKS; k = k + 1)
                chunk_len[k] <= 7'd0;
        end else begin
            if (payload_rsp) begin
                chunks_received <= chunks_received + 1'b1;
                if (dma_rsp_status != 0 || dma_rsp_write)
                    failed <= 1'b1;
            end

            case (state)
                ST_IDLE: begin
                    done_valid <= 1'b0;
                    if (start_valid) begin
                        wide_desc <= start_dcr[5];
                        word_bytes <= start_dcr[5] ? 3'd4 : 3'd2;
                        upper_addr <= start_utda;
                        // Bit zero of CTDA is the EOL flag loaded from the
                        // preceding descriptor link.  It is status, not part
                        // of the naturally aligned descriptor address.
                        descriptor_addr <= {start_utda, start_ctda & 16'hfffe};
                        done_ctda <= start_ctda;
                        done_tcr <= 16'd0;
                        done_tps <= 16'd0;
                        done_tfc <= 16'd0;
                        done_error <= 1'b0;
                        done_pint <= 1'b0;
                        failed <= 1'b0;
                        state <= ST_HDR_REQ;
                    end
                end
                ST_HDR_REQ: if (dma_req_ready) state <= ST_HDR_WAIT;
                ST_HDR_WAIT: if (dma_rsp_valid && dma_rsp_tag == TAG_HEADER) begin
                    descriptor_tcr <= descriptor_word(dma_rsp_rdata, 1, wide_desc) & 16'hf000;
                    done_tps <= descriptor_word(dma_rsp_rdata, 2, wide_desc);
                    done_tfc <= header_tfc_word;
                    fragment_count <= header_tfc_word[11:0];
                    fragment_index <= 0;
                    if (dma_rsp_status != 0 || dma_rsp_write ||
                        (header_tfc_word == 0) || (header_tfc_word > MAX_FRAGMENTS)) begin
                        failed <= 1'b1;
                        state <= ST_DONE;
                    end else state <= ST_FRAG_REQ;
                end
                ST_FRAG_REQ: if (dma_req_ready) state <= ST_FRAG_WAIT;
                ST_FRAG_WAIT: if (dma_rsp_valid && dma_rsp_tag == TAG_FRAGMENT) begin
                    fragment_addr[fragment_index] <=
                        {descriptor_word(dma_rsp_rdata, 1, wide_desc),
                         descriptor_word(dma_rsp_rdata, 0, wide_desc)};
                    fragment_len[fragment_index] <= descriptor_word(dma_rsp_rdata, 2, wide_desc);
                    if (dma_rsp_status != 0 || dma_rsp_write ||
                        descriptor_word(dma_rsp_rdata, 2, wide_desc) == 0) begin
                        failed <= 1'b1;
                        state <= ST_DONE;
                    end else if (fragment_index + 1'b1 == fragment_count) begin
                        issue_fragment <= 0;
                        issue_offset <= 0;
                        chunks_issued <= 0;
                        chunks_received <= 0;
                        total_chunks <= 0;
                        chunk_read_addr <= 0;
                        state <= ST_PAYLOAD;
                    end else begin
                        fragment_index <= fragment_index + 1'b1;
                        state <= ST_FRAG_REQ;
                    end
                end
                ST_PAYLOAD: begin
                    if (payload_req_fire) begin
                        chunk_len[chunks_issued] <= next_chunk_len;
                        chunks_issued <= chunks_issued + 1'b1;
                        total_chunks <= chunks_issued + 1'b1;
                        if (current_fragment_remaining <= 16'd64) begin
                            issue_fragment <= issue_fragment + 1'b1;
                            issue_offset <= 0;
                            if (issue_fragment + 1'b1 == fragment_count)
                                state <= ST_COLLECT;
                        end else issue_offset <= issue_offset + 16'd64;
                    end else if ((issue_fragment < fragment_count) &&
                                 (chunks_issued == MAX_CHUNKS)) begin
                        failed <= 1'b1;
                        state <= ST_DONE;
                    end
                end
                ST_COLLECT: begin
                    if (chunks_received == total_chunks) begin
                        if (failed) state <= ST_DONE;
                        else begin
                            stream_chunk <= 0;
                            stream_byte <= 0;
`ifdef SONIC_TX_MUTANT_NO_PRIME
                            stream_shift <= chunk_read_data;
                            chunk_read_addr <= 1;
                            state <= ST_STREAM;
`else
                            chunk_read_addr <= 0;
                            state <= ST_PRIME;
`endif
                        end
                    end
                end
                // Allow the synchronous packet BRAM read to settle after the
                // final response write.  This is essential for one-chunk
                // frames, where read and write otherwise target slot zero on
                // the edge collection completes.
                ST_PRIME: begin
                            stream_shift <= chunk_read_data;
                            chunk_read_addr <= 1;
                            state <= ST_STREAM;
                end
                ST_STREAM: if (tx_axis_tvalid && tx_axis_tready) begin
                    if (stream_byte + 1'b1 == chunk_len[stream_chunk]) begin
                        stream_byte <= 0;
                        if (stream_chunk + 1'b1 == total_chunks)
                            state <= ST_CPL;
                        else begin
                            stream_chunk <= stream_chunk + 1'b1;
                            stream_shift <= chunk_read_data;
                            chunk_read_addr <= stream_chunk + 2'd2;
                        end
                    end else begin
                        stream_byte <= stream_byte + 1'b1;
                        stream_shift <= {8'd0, stream_shift[511:8]};
                    end
                end
                ST_CPL: if (tx_cpl_valid) state <= ST_STATUS_REQ;
                ST_STATUS_REQ: if (dma_req_ready) state <= ST_STATUS_WAIT;
                ST_STATUS_WAIT: if (dma_rsp_valid && dma_rsp_tag == TAG_STATUS) begin
                    if (dma_rsp_status != 0 || !dma_rsp_write) begin
                        failed <= 1'b1;
                        state <= ST_DONE;
                    end else begin
                        // NOT `& 16'h07ff`.  descriptor_tcr holds the
                        // CONFIG half (0xF000, masked at ST_HDR_WAIT), so
                        // ANDing with the status mask erased it and TCR read
                        // back 0x0001 forever -- PINT/POWC/CRCI/EXDIS never
                        // appeared.  ds:1899-1900: bits 15-12 of TXpkt.config
                        // are loaded into TCR; mame.cpp:369+427 keeps them and
                        // ORs in PTX.  Retaining them is also what makes PINT
                        // edge-detection possible at all (see q700_eth_sonic.v).
                        done_tcr <= descriptor_tcr | TCR_PTX;
                        // ds:1224-1227: "If the halt transmit command is
                        // issued... the CTDA register is NOT loaded".  So a
                        // halt skips the link read entirely and done_ctda
                        // keeps pointing at the descriptor just transmitted,
                        // rather than advancing to the next.  Same gate as
                        // mame.cpp:434 (`if (!(m_reg[CR] & CR_HTX))`), which
                        // covers both the reload and the chain.
                        state <= halt ? ST_DONE : ST_LINK_REQ;
                    end
                end
                ST_LINK_REQ: if (dma_req_ready) begin
                    state <= ST_LINK_WAIT;
                end
                ST_LINK_WAIT: if (dma_rsp_valid && dma_rsp_tag == TAG_LINK) begin
                    if (dma_rsp_status != 0 || dma_rsp_write) begin
                        failed <= 1'b1;
                        state <= ST_DONE;
                    end else begin
`ifdef SONIC_TX_MUTANT_LINK_ADDR
                        done_ctda <= link_rsp_word[0] ?
                            descriptor_addr[15:0] +
                                (4 + fragment_count*3)*word_bytes :
                            link_rsp_word;
`else
                        done_ctda <= link_rsp_word;
`endif
                        if (link_rsp_word[0]) begin
                            state <= ST_DONE;
                        end else begin
                            descriptor_addr <= {upper_addr, link_rsp_word};
                            state <= ST_HDR_REQ;
                        end
                    end
                end
                ST_DONE: begin
                    done_valid <= 1'b1;
                    done_error <= failed;
                    done_pint  <= descriptor_tcr[15] && !failed;
                    if (done_ready) begin
                        done_valid <= 1'b0;
                        state <= ST_IDLE;
                    end
                end
                default: state <= ST_IDLE;
            endcase
        end
    end

`ifndef SYNTHESIS
    initial begin
        if (MAX_CHUNKS > 63 || MAX_CHUNKS < 1)
            $error("q700_sonic_tx MAX_CHUNKS must be 1..63");
        if (MAX_FRAGMENTS < 1)
            $error("q700_sonic_tx MAX_FRAGMENTS must be positive");
    end
`endif
    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused_rsp_len = |dma_rsp_len;
    /* verilator lint_on UNUSEDSIGNAL */
    /* verilator lint_on WIDTHEXPAND */
    /* verilator lint_on WIDTHTRUNC */
endmodule
`default_nettype wire
