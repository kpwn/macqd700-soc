// rtl/board/vhdd_net.sv — Ethernet-backed vHDD provider (transaction layer).
//
// WHAT THIS IS
// ════════════
// The third `vhdd` provider (contract in rtl/vhdd.vh), alongside vhdd_sd.
// It turns the master's block requests into UDP round trips through
// net_block_framer, and hides the round-trip latency behind a ping-pong
// staging buffer.
//
//   scsi.v ──vhdd──► vhdd_net ──req/reply──► net_block_framer ──► MAC
//
// THE CONSTRAINT EVERYTHING FOLLOWS FROM
// ══════════════════════════════════════
// Q700 SCSI is CPU-driven pseudo-DMA: the CPU spins in a tight loop moving
// bytes during the data phase.  A network round trip INSIDE that phase does
// not read as a slow disk, it reads as a HANG.  So a chunk's data must be
// resident before the master is allowed to drain it, and the refill for
// chunk N+1 must overlap the drain of chunk N.  That is the whole reason
// for the ping-pong buffer -- it is a staging buffer, not a cache.
//
// WINDOWING
// ═════════
// Every frame carries at most WINDOW_BLOCKS (2) blocks in BOTH directions:
// a 1500-byte MTU minus IPv4+UDP+protocol headers leaves room for 1024
// payload bytes, and three blocks (1536) fits neither the request nor the
// reply budget.  net_block_framer enforces that at its own seam; this
// module never asks for more.  That is a limit on the WIRE, not on SCSI --
// req_block_count is 16 bits and READ(10) may legitimately ask for 65535,
// which simply becomes many windows.
//
// RELIABILITY
// ═══════════
// UDP is lossy, so each in-flight request carries a tag, and a reply whose
// tag does not match the outstanding one is ignored rather than mistaken
// for the answer (a late reply to a retransmitted request is exactly that
// case, and accepting it would deliver the WRONG BLOCK with good status).
// A request that goes unanswered for REPLY_TIMEOUT cycles is retransmitted
// up to MAX_RETRIES times before the transfer fails.
//
// Two bounded-response obligations from the contract, both load-bearing:
//
//   * A retry budget that is exhausted must produce `done | error`, never
//     silence.  A wedged SCSI transaction is worse than a failed one: the
//     master can retry an error, but it cannot recover from a provider that
//     simply never answers.
//   * Holding rd_ready low (or starving us of write bytes) must NOT stop
//     that clock.  A master that stalls long enough gets `done | error`
//     too.  This is why STALL_TIMEOUT exists as well as REPLY_TIMEOUT --
//     without it, a master that walks away mid-transfer parks `busy`
//     forever and the volume is dead until reset.
//
// WRITE ORDERING
// ══════════════
// Writes are write-THROUGH: `done` is not asserted until the last window
// has been acknowledged by the host.  The design doc contemplates
// write-back (complete the SCSI command, flush asynchronously) as a later
// optimisation; it is deliberately not v1, because reporting success for
// bytes still sitting in a buffer means a power loss or a dropped reply
// silently loses data the Mac believes is committed.  The ping-pong still
// applies -- the master fills slot B while slot A is on the wire -- so the
// steady-state cost of write-through is one round trip at the END of a
// transfer, not one per window.

module vhdd_net #(
    // Blocks per frame.  2 is the MTU ceiling; see WINDOWING above.
    parameter integer WINDOW_BLOCKS = 2,
    // Cycles to wait for a reply before retransmitting.
    parameter integer REPLY_TIMEOUT = 32'd2_000_000,
    // Retransmissions before the transfer is failed.
    parameter integer MAX_RETRIES = 3,
    // Cycles the master may stall us before we give up and report error.
    parameter integer STALL_TIMEOUT = 32'd200_000_000
) (
    input  wire        clk,
    input  wire        rst,

    // ── vhdd master side (contract in rtl/vhdd.vh) ────────────────────
    input  wire [31:0] num_lbas,

    input  wire [31:0] chk_lba,
    input  wire [23:0] chk_blocks,
    output wire        chk_ok,

    input  wire        req_write,
    input  wire        req_multi,
    input  wire [31:0] req_lba,
    input  wire [15:0] req_block_count,
    input  wire        req_go,

    output wire        busy,
    output reg         done,
    output reg         error,

    output wire        rd_valid,
    output wire [7:0]  rd_data,
    input  wire        rd_ready,

    output wire        wr_ready,
    input  wire        wr_valid,
    input  wire [7:0]  wr_data,
    input  wire        wr_avail,

    // ── net_block_framer side ─────────────────────────────────────────
    output reg         net_req_valid,
    input  wire        net_req_ready,
    output reg  [7:0]  net_req_op,
    output reg  [31:0] net_req_lba,
    output reg  [15:0] net_req_block_count,
    output reg  [15:0] net_req_tag,

    output wire [7:0]  net_req_payload_tdata,
    output reg         net_req_payload_tvalid,
    input  wire        net_req_payload_tready,
    output wire        net_req_payload_tlast,

    input  wire        net_reply_valid,
    output wire        net_reply_ready,
    input  wire [15:0] net_reply_tag,
    input  wire [7:0]  net_reply_status,

    input  wire [7:0]  net_reply_payload_tdata,
    input  wire        net_reply_payload_tvalid,
    output wire        net_reply_payload_tready,
    input  wire        net_reply_payload_tlast
);

    localparam integer BLOCK_BYTES  = 512;
    localparam integer SLOT_BYTES   = WINDOW_BLOCKS * BLOCK_BYTES;   // 1024
    localparam integer SLOT_AW      = $clog2(SLOT_BYTES);            // 10

    localparam [7:0] OP_READ  = 8'h00;
    localparam [7:0] OP_WRITE = 8'h01;

    // ── staging buffer ────────────────────────────────────────────────
    // Two slots, one BRAM.  Inferred, not instantiated: this is a small
    // simple dual-port and the tools infer it cleanly at both widths.
    reg [7:0] stage [0:(2*SLOT_BYTES)-1];
    reg [7:0] stage_rd_data;

    // Slot ownership.  For a READ the network FILLS and the master DRAINS;
    // for a WRITE those roles swap.  Tracking "filled" rather than
    // "readable"/"writable" keeps one meaning for both directions.
    reg [1:0]  slot_filled;      // slot holds data nobody has consumed yet
    reg [SLOT_AW:0] slot_len [0:1];   // valid bytes in each slot

    reg        net_slot;         // slot the network side is working on
    reg        mst_slot;         // slot the master side is working on

    // ── transfer bookkeeping ──────────────────────────────────────────
    reg        xfer_active;
    reg        xfer_write;
    reg [31:0] xfer_lba;         // LBA of the next window to REQUEST
    reg [15:0] xfer_blocks_left; // blocks not yet requested
    reg [15:0] mst_blocks_left;  // blocks not yet handed to/from the master
    reg        failed;

    reg [31:0] stall_timer;

    wire [15:0] window_blocks =
        (xfer_blocks_left > WINDOW_BLOCKS[15:0]) ? WINDOW_BLOCKS[15:0]
                                                 : xfer_blocks_left;

    // ── extent probe (combinational, no side effects) ─────────────────
    // Fails closed on a zero-length extent and on capacity overflow.  The
    // 33-bit sum is deliberate: chk_lba + chk_blocks can wrap 32 bits, and
    // a wrapped compare would report a wildly out-of-range extent as fine.
    wire [32:0] chk_end = {1'b0, chk_lba} + {9'd0, chk_blocks};
    assign chk_ok = (chk_blocks != 24'd0) && (chk_end <= {1'b0, num_lbas});

    assign busy = xfer_active;

    // ══════════════════════════════════════════════════════════════════
    // NETWORK SIDE
    // ══════════════════════════════════════════════════════════════════
    // N_PRIME exists because the staging buffer is BRAM with a REGISTERED
    // read port: the byte for address A is not on stage_rd_data until the
    // cycle after A is presented.  Asserting payload tvalid in the same
    // cycle we set the address would put the PREVIOUS window's last byte
    // on the wire as this window's first.  One priming cycle per window,
    // then the address runs one ahead of the consumer.
    localparam [2:0] N_IDLE = 3'd0, N_REQ = 3'd1, N_WPAY = 3'd2,
                     N_WAIT = 3'd3, N_RPAY = 3'd4, N_PRIME = 3'd5;

    reg [2:0]  nstate;
    reg [15:0] net_tag;
    reg [31:0] reply_timer;
    reg [15:0] retries;
    reg [SLOT_AW:0] net_index;      // byte cursor within the network's slot
    reg [SLOT_AW:0] net_expect;     // bytes this window should move
    reg [15:0] net_window_blocks;
    reg [31:0] net_window_lba;

    // A reply is ours only if its tag matches the outstanding request.  A
    // late reply to a request we already retransmitted carries the OLD tag
    // and must be dropped -- taking it would pair chunk N's data with
    // chunk N+1's buffer and silently return the wrong block.
    wire reply_is_ours = net_reply_valid && (net_reply_tag == net_tag);

    // Always accept, unconditionally, in every state.  A reply whose tag
    // does not match is DISCARDED rather than back-pressured: holding it
    // would park the frame at the head of the channel and stall the reply
    // we are actually waiting for behind it, turning one lost packet into
    // a permanent wedge.  Selectivity lives in reply_is_ours, not here.
    assign net_reply_ready = 1'b1;
    assign net_reply_payload_tready = 1'b1;

    assign net_req_payload_tdata = stage_rd_data;
    assign net_req_payload_tlast = (net_index + 1'b1) == net_expect;

    // ══════════════════════════════════════════════════════════════════
    // MASTER SIDE
    // ══════════════════════════════════════════════════════════════════
    reg [SLOT_AW:0] mst_index;
    reg [SLOT_AW:0] mst_expect;
    reg             mst_busy;      // master is working a slot right now
    reg             mst_prime;     // one cycle of BRAM read latency (reads only)

    wire mst_slot_ready = slot_filled[mst_slot];

    // Reads: present a byte whenever the master's slot holds one.
    assign rd_valid = xfer_active && !xfer_write && mst_busy;
    assign rd_data  = stage_rd_data;

    // Writes: ask for a byte only when there is somewhere to put it AND the
    // master actually has one.  wr_avail is REAL back-pressure -- taking a
    // byte while it is low latches the previous byte again, which is how
    // the SD provider once wrote a stale block with good status.
    assign wr_ready = xfer_active && xfer_write && mst_busy && wr_avail;

    integer i;

    always @(posedge clk) begin
        done <= 1'b0;

        if (rst) begin
            xfer_active <= 1'b0; xfer_write <= 1'b0;
            xfer_lba <= 32'd0; xfer_blocks_left <= 16'd0;
            mst_blocks_left <= 16'd0; failed <= 1'b0; error <= 1'b0;
            slot_filled <= 2'b00; net_slot <= 1'b0; mst_slot <= 1'b0;
            nstate <= N_IDLE; net_tag <= 16'd0; reply_timer <= 32'd0;
            retries <= 16'd0; net_index <= 0; net_expect <= 0;
            net_window_blocks <= 16'd0; net_window_lba <= 32'd0;
            net_req_valid <= 1'b0; net_req_payload_tvalid <= 1'b0;
            net_req_op <= 8'd0; net_req_lba <= 32'd0;
            net_req_block_count <= 16'd0; net_req_tag <= 16'd0;
            mst_index <= 0; mst_expect <= 0; mst_busy <= 1'b0;
            mst_prime <= 1'b0;
            stall_timer <= 32'd0;
            for (i = 0; i < 2; i = i + 1) slot_len[i] <= 0;
        end else begin
            // ── accept a new transfer ─────────────────────────────────
            if (req_go && !xfer_active) begin
                xfer_active      <= 1'b1;
                xfer_write       <= req_write;
                xfer_lba         <= req_lba;
                xfer_blocks_left <= req_block_count;
                mst_blocks_left  <= req_block_count;
                failed           <= 1'b0;
                error            <= 1'b0;
                slot_filled      <= 2'b00;
                net_slot         <= 1'b0;
                mst_slot         <= 1'b0;
                mst_busy         <= 1'b0;
                mst_prime        <= 1'b0;
                nstate           <= N_IDLE;
                retries          <= 16'd0;
                stall_timer      <= 32'd0;
                for (i = 0; i < 2; i = i + 1) slot_len[i] <= 0;
            end

            // ── the stall clock ───────────────────────────────────────
            // Runs whenever a transfer is live.  Any real progress on
            // either side resets it.  Its whole job is to guarantee that
            // `busy` is not a permanent state.
            if (xfer_active) stall_timer <= stall_timer + 1'b1;

            // ══════════════════════════════════════════════════════════
            // network side
            // ══════════════════════════════════════════════════════════
            case (nstate)
            N_IDLE: begin
                // Start the next window as soon as the network's slot is
                // free.  For a read "free" means the master has drained
                // it; for a write it means the master has filled it.
                if (xfer_active && !failed && (xfer_blocks_left != 16'd0) &&
                    (xfer_write ? slot_filled[net_slot] : !slot_filled[net_slot])) begin
                    net_req_op          <= xfer_write ? OP_WRITE : OP_READ;
                    net_req_lba         <= xfer_lba;
                    net_req_block_count <= window_blocks;
                    net_req_tag         <= net_tag;
                    net_req_valid       <= 1'b1;
                    net_window_blocks   <= window_blocks;
                    net_window_lba      <= xfer_lba;
                    net_expect          <= {window_blocks[SLOT_AW-9:0], 9'd0};  // blocks*512, exactly SLOT_AW+1 bits
                    net_index           <= 0;
                    retries             <= 16'd0;
                    nstate              <= N_REQ;
                end
            end
            N_REQ: begin
                if (net_req_ready) begin
                    net_req_valid <= 1'b0;
                    reply_timer   <= 32'd0;
                    if (xfer_write) begin
                        nstate <= N_PRIME;      // let stage_rd_data settle
                    end else begin
                        nstate <= N_WAIT;
                    end
                end
            end
            N_PRIME: begin
                net_req_payload_tvalid <= 1'b1;
                nstate <= N_WPAY;
            end
            N_WPAY: begin
                if (net_req_payload_tready) begin
                    stall_timer <= 32'd0;
                    if (net_req_payload_tlast) begin
                        net_req_payload_tvalid <= 1'b0;
                        nstate      <= N_WAIT;
                        reply_timer <= 32'd0;
                    end else begin
                        net_index <= net_index + 1'b1;
                    end
                end
            end
            N_WAIT: begin
                reply_timer <= reply_timer + 1'b1;
                if (reply_is_ours) begin
                    stall_timer <= 32'd0;
                    if (net_reply_status != 8'h00) begin
                        failed <= 1'b1;
                        nstate <= N_IDLE;
                    end else if (xfer_write) begin
                        // Window acknowledged: release the slot and advance.
                        slot_filled[net_slot] <= 1'b0;
                        net_slot         <= ~net_slot;
                        net_tag          <= net_tag + 1'b1;
                        xfer_lba         <= net_window_lba + {16'd0, net_window_blocks};
                        xfer_blocks_left <= xfer_blocks_left - net_window_blocks;
                        nstate           <= N_IDLE;
                    end else begin
                        net_index <= 0;
                        nstate    <= N_RPAY;
                    end
                end else if (reply_timer >= REPLY_TIMEOUT[31:0]) begin
                    // Retransmit with the SAME tag: the request is
                    // idempotent, and reusing the tag means a reply to
                    // either copy is acceptable.
                    if (retries >= MAX_RETRIES[15:0]) begin
                        failed <= 1'b1;
                        nstate <= N_IDLE;
                    end else begin
                        retries       <= retries + 1'b1;
                        net_req_valid <= 1'b1;
                        net_index     <= 0;
                        reply_timer   <= 32'd0;
                        nstate        <= N_REQ;   // re-primes via N_PRIME
                    end
                end
            end
            N_RPAY: begin
                if (net_reply_payload_tvalid) begin
                    stall_timer <= 32'd0;
                    net_index <= net_index + 1'b1;
                    if (net_reply_payload_tlast ||
                        ((net_index + 1'b1) == net_expect)) begin
                        slot_len[net_slot]    <= net_expect;
                        slot_filled[net_slot] <= 1'b1;
                        net_slot         <= ~net_slot;
                        net_tag          <= net_tag + 1'b1;
                        xfer_lba         <= net_window_lba + {16'd0, net_window_blocks};
                        xfer_blocks_left <= xfer_blocks_left - net_window_blocks;
                        nstate           <= N_IDLE;
                    end
                end
            end
            default: nstate <= N_IDLE;
            endcase

            // ══════════════════════════════════════════════════════════
            // master side
            // ══════════════════════════════════════════════════════════
            if (xfer_active && !mst_busy && (mst_blocks_left != 16'd0)) begin
                if (xfer_write) begin
                    // Claim a free slot to fill from the master.
                    if (!slot_filled[mst_slot]) begin
                        mst_expect <= {((mst_blocks_left > WINDOW_BLOCKS[15:0])
                                        ? WINDOW_BLOCKS[SLOT_AW-9:0]
                                        : mst_blocks_left[SLOT_AW-9:0]), 9'd0};
                        mst_index <= 0;
                        mst_busy  <= 1'b1;
                    end
                end else begin
                    // Drain a slot the network has filled.  Prime first --
                    // rd_valid must not go high until stage_rd_data actually
                    // holds byte 0 of THIS slot.
                    if (slot_filled[mst_slot] && !mst_prime) begin
                        mst_expect <= slot_len[mst_slot];
                        mst_index  <= 0;
                        mst_prime  <= 1'b1;
                    end else if (mst_prime) begin
                        mst_prime <= 1'b0;
                        mst_busy  <= 1'b1;
                    end
                end
            end else if (xfer_active && mst_busy) begin
                if (xfer_write) begin
                    if (wr_ready) begin
                        stall_timer <= 32'd0;
                        if ((mst_index + 1'b1) == mst_expect) begin
                            slot_len[mst_slot]    <= mst_expect;
                            slot_filled[mst_slot] <= 1'b1;
                            mst_slot        <= ~mst_slot;
                            mst_busy        <= 1'b0;
                            mst_blocks_left <= mst_blocks_left -
                                {{(16-(SLOT_AW-8)){1'b0}}, mst_expect[SLOT_AW:9]};
                        end else begin
                            mst_index <= mst_index + 1'b1;
                        end
                    end
                end else begin
                    if (rd_ready) begin
                        stall_timer <= 32'd0;
                        if ((mst_index + 1'b1) == mst_expect) begin
                            slot_filled[mst_slot] <= 1'b0;
                            mst_slot        <= ~mst_slot;
                            mst_busy        <= 1'b0;
                            mst_blocks_left <= mst_blocks_left -
                                {{(16-(SLOT_AW-8)){1'b0}}, mst_expect[SLOT_AW:9]};
                        end else begin
                            mst_index <= mst_index + 1'b1;
                        end
                    end
                end
            end

            // ══════════════════════════════════════════════════════════
            // completion
            // ══════════════════════════════════════════════════════════
            // A write is not done until its last window has been ACKED, so
            // it waits for the network side to drain too -- that is what
            // write-through means and it is the point of it.
            if (xfer_active && failed) begin
                xfer_active <= 1'b0;
                mst_busy    <= 1'b0;
                error       <= 1'b1;
                done        <= 1'b1;
                net_req_valid <= 1'b0;
                net_req_payload_tvalid <= 1'b0;
            end else if (xfer_active && (mst_blocks_left == 16'd0) &&
                         !mst_busy &&
                         (!xfer_write || ((xfer_blocks_left == 16'd0) &&
                                          (nstate == N_IDLE) &&
                                          (slot_filled == 2'b00)))) begin
                xfer_active <= 1'b0;
                error       <= 1'b0;
                done        <= 1'b1;
            end else if (xfer_active && (stall_timer >= STALL_TIMEOUT[31:0])) begin
                // Bounded response: `busy` must not be permanent.  A master
                // that walks away mid-transfer gets an error it can retry,
                // not a volume that is dead until reset.
                xfer_active <= 1'b0;
                mst_busy    <= 1'b0;
                error       <= 1'b1;
                done        <= 1'b1;
                net_req_valid <= 1'b0;
                net_req_payload_tvalid <= 1'b0;
            end
        end
    end

    // ── staging buffer ports ──────────────────────────────────────────
    // One writer and one reader per cycle: for a read the network writes
    // and the master reads; for a write those swap.
    wire        stage_we   = xfer_write ? wr_ready
                                        : (net_reply_payload_tvalid && (nstate == N_RPAY));
    wire [SLOT_AW:0] stage_wa = xfer_write ? {mst_slot, mst_index[SLOT_AW-1:0]}
                                           : {net_slot, net_index[SLOT_AW-1:0]};
    wire [7:0]  stage_wd   = xfer_write ? wr_data : net_reply_payload_tdata;
    // Read address runs ONE AHEAD of the consumer, so the registered output
    // holds the byte for the cursor's CURRENT position.  While a consumer is
    // stalled the address holds, which simply re-reads the same byte.
    wire [SLOT_AW-1:0] net_ra_next =
        (nstate == N_PRIME) ? {SLOT_AW{1'b0}}
                            : net_index[SLOT_AW-1:0] +
                              ((net_req_payload_tvalid && net_req_payload_tready)
                               ? {{(SLOT_AW-1){1'b0}}, 1'b1} : {SLOT_AW{1'b0}});
    wire [SLOT_AW-1:0] mst_ra_next =
        mst_prime ? {SLOT_AW{1'b0}}
                  : mst_index[SLOT_AW-1:0] +
                    ((mst_busy && rd_ready) ? {{(SLOT_AW-1){1'b0}}, 1'b1}
                                            : {SLOT_AW{1'b0}});
    wire [SLOT_AW:0] stage_ra = xfer_write ? {net_slot, net_ra_next}
                                           : {mst_slot, mst_ra_next};

    always @(posedge clk) begin
        if (stage_we) stage[stage_wa] <= stage_wd;
        stage_rd_data <= stage[stage_ra];
    end

    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused = &{1'b0, req_multi, wr_valid, net_reply_payload_tlast};
    /* verilator lint_on UNUSEDSIGNAL */

endmodule
