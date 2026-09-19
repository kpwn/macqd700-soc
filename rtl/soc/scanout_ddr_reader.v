// scanout_ddr_reader.v -- DDR-backed drop-in replacement for vram.v's
// streaming scan-out read port (T14, VRAM-in-DDR migration).
//
// Under `VRAM_IN_DDR`, the VRAM aperture (4 MB in that build -- see
// AXI_VRAM_SIZE in rtl/soc/axi_defs.vh; 2 MB on the URAM build) moves from a URAM array
// (rtl/board/vram.v) to a 32 MiB carveout (AXI_VRAM_DDR_CARVEOUT_BASE,
// rtl/soc/axi_defs.vh). fb_reader.v still expects vram.v's exact
// streaming-port shape (rd_clk/rd_rst/rd_addr/rd_en -> rd_data/rd_valid);
// this module is that port-compatible substitute, fetching 128 B (8-beat,
// 16 B/beat) INCR AXI4 bursts from the carveout instead of reading a
// local array. The ring buffer, prefetch, eviction-safety, and AXI4
// fetch-engine logic live in the companion module scanout_line_fetch.v
// (split out to keep both files under the project's ~300-line
// convention, docs/agent_policy.md); this module owns the request queue
// and the streaming-port timing.
//
// CONTRACT: every `rd_en` is accepted (no backpressure, matching vram.v)
// and produces exactly one later `rd_valid` pulse, IN ORDER, no drops --
// fb_reader.v's in_flight_cnt/credit machinery assumes this. Latency is
// VARIABLE (ring hit vs. miss), unlike vram.v's fixed latency; fb_reader
// already tolerates variable latency (up to 15 outstanding by default).
//
// REQUEST QUEUE (correctness-critical): every accepted rd_en pushes
// {line,offset} onto an in-order FIFO. The drain logic only ever looks at
// the HEAD (query into scanout_line_fetch's `hit`/`hit_byte`): hit ->
// pop+respond (1-cycle registered latency); miss -> the WHOLE queue
// stalls until the head's line is resident. This is what makes
// back-to-back requests for a still-fetching line correct (the common
// case -- a whole LINE_BYTES (128 B) worth of consecutive requests can
// land on one in-flight fetch)
// instead of silently dropping them (a bug an earlier draft had,
// checking hit/miss against live `rd_addr` instead of the queue head).
//
// RING / PREFETCH / EVICTION / RESET (scanout_line_fetch.v): NUM_LINE_BUF
// (8) line buffers, a PREFETCH_DEPTH (5) line LOOKAHEAD WINDOW tracked
// from the queue head, up to MAX_OUTSTANDING (4) AXI bursts in flight,
// a REFILL_THRESH (2) hysteresis that batches the refills,
// monotonic-consumption eviction safety, and a bounded post-reset drain
// (F_RSTDRAIN, mirroring l2c_mshr.v's S_DRAIN, docs/l2c_spec.md S5) for
// bursts already accepted downstream at reset.  See
// scanout_line_fetch.v's header for the full writeup.
//
// v1 ran ONE-LINE-AHEAD out of the same four buffers with ONE burst in
// flight, and justified the latter as avoiding "widening the AXI ID
// field ... out of scope per the brief".  That was a SCOPING decision,
// not a design conclusion, and it turns out NO widening is needed: read
// data returns in AR-ACCEPTANCE order (docs/ddr4_mig_bridge_contract.md)
// and axi_vram_priority_mux3.v's route FIFO already depends on exactly
// that, so multi-outstanding needs only an in-order slot FIFO and the
// same single AXI ID.
//
// BANDWIDTH (re-derived; the previous version of this block was wrong in a
// way worth recording so it is not re-introduced).
//
// WHAT THE OLD NOTE GOT WRONG: it quoted "~1.45 B/cycle (~145 MB/s)" as
// this module's throughput. That is the RING FILL capability, which sits
// BEHIND the streaming port and therefore never bound anything. The port
// itself accepts at most one request per clk; when it returned ONE byte per
// request the port ceiling was 1 B/cycle = 100 MB/s at 100 MHz, strictly
// below the fill capability. linebuf_scanout.v's header repeated the 145
// MB/s figure as a *port* limit; both are corrected now.
//
// Relatedly: it is NOT true that 15/16 of each beat was discarded and
// re-requested (a natural misreading of the byte-wide port). Every beat's
// full 16 B was always written into the ring in ONE cycle
// (scanout_line_fetch.v's F_R state), and the ring then served up to
// LINE_BYTES consecutive requests from it. The old inefficiency was the
// one-byte-per-REQUEST drain, not the fill.
//
// TODAY, with the 4-byte drain group + 128 B lines, at 100 MHz:
//   port ceiling      = 4 B/request * 1 request/cycle   = 400 MB/s
//   ring fill ceiling, v1 (ONE burst in flight, no overlap):
//                     = LINE_BYTES / (latency + BEATS)
//                     = 128 B / (~40 + 8) cycles = 2.67 B/cycle = 267 MB/s
//   ring fill ceiling, NOW (MAX_OUTSTANDING bursts pipelined):
//                     = MAX_OUTSTANDING * LINE_BYTES / (latency + MAX_OUTSTANDING*BEATS)
//                     = 4 * 128 B / (~40 + 32) cycles = 7.1 B/cycle = 711 MB/s
//   ...and that is only the RAMP figure; once the pipeline is full the
//   ceiling is the scanout share of the shared R channel (16 B/cycle),
//   not the round trip at all.
// So the round trip stopped being the binding term.  It is now the
// arbiter share, which is exactly the knob we WANT to be binding, because
// it is the one that trades against CPU concurrency explicitly.
//
// Demand at 60 Hz, in LINE traffic (what the ring actually moves -- whole
// lines, so the 24bpp pad byte is free: it is already inside the line):
//   1024x768  8bpp : 1024 B/row  ->  8 lines/row -> 0.35 M lines/s ->  47 MB/s (18%)
//   1024x768 24bpp : 4096 B/row  -> 32 lines/row -> 1.47 M lines/s -> 189 MB/s (71%)
// 1024x768x24bpp closed with only ~1.4x margin against v1's 267 MB/s, and
// that margin is what forced axi_vram_priority_mux3.v's MAX_BULK_AHEAD
// down to 2 -- i.e. the DISPLAY was bounding the CPU's memory system.
// Against the pipelined figure the same mode has >3x margin even while
// the CPU holds MAX_BULK_AHEAD non-scan bursts queued ahead of it.
//
// CAVEAT (IMPORTANT -- read before trusting any margin above): the
// ~40-cycle DDR round trip is an ASSUMPTION inherited from the original
// T14 writeup.  There is NO measured real-hardware DDR read latency
// anywhere in this repo.  v1's numbers scaled LINEARLY with it, which is
// what made them fragile; the multi-outstanding fill above is
// deliberately structured so the round trip is amortised across
// MAX_OUTSTANDING bursts and the steady-state rate does not depend on it
// at all.  tb-scanout-ddr-frames now models the round trip explicitly
// (DDR_READ_LATENCY_BASE/JITTER, default 40 +/- 8) and sweeps it; the
// gate at 200 +/- 64 exists to show the conclusion is not
// latency-sensitive.
//
// RESET UNDER A CORE-ONLY (S-SIDE-ONLY) RESET -- RESOLVED (T14 round 3,
// M1 stale-header cleanup): tb-vram-ddr-chain's reset_mid_stream
// scenario originally failed for SOME reset timings (any timing where
// an AXI burst had already been accepted into axi_async_bridge.v's
// R-channel CDC before reset) -- root-caused to that bridge's shared
// async_fifo T6 coupled-reset handshake having no AXI burst-semantics
// awareness: a core-only reset leaves the downstream slave (mig_clk
// side) unaware it should abandon an already-accepted burst, so its
// tail beats got replayed as stale data once the handshake settled and
// backpressure released. Confirmed directly at the time via
// hierarchical DRAM-content probing: sim_mig_backend's own storage held
// the CORRECT post-reset values while reads still returned stale data
// -- the corruption was strictly in-flight in the shared CDC bridge,
// never in this module's write path or its own reset-drain (AR-issued/
// RLAST-seen accounting independently verified via instrumentation).
// FIXED in axi_async_bridge.v itself via rtl/soc/axi_bridge_stale_sink.v
// (an m-side outstanding-burst counter that sinks exactly the owed
// stale completions after an s-side reset before resuming normal
// forwarding -- see that file's header for the full mechanism, and
// t14-report.md ROUND 3 for the fix trail). reset_mid_stream now passes
// deterministically across 5 reset timings (1/2/3/4/6 pre-reset ticks),
// in both the L2C-enabled and no-L2C builds. roundtrip and
// concurrent_streaming (no reset involved) were never affected.
//
`default_nettype none

module scanout_ddr_reader #(
    parameter ADDR_W       = 21,   // byte index width (matches FB_ADDR_W)
    parameter BPP          = 8,    // read-port ADDRESS granularity: 1 B/index
    // Streaming-port data width: a 4-byte drain group.  See the port
    // contract on `rd_data` below.  Only 32 is supported.
    parameter RD_DATA_W    = 32,
    parameter DATA_WIDTH   = 128,
    parameter ID_WIDTH     = 6,
    parameter [ID_WIDTH-1:0] AXI_ID = {ID_WIDTH{1'b0}},
    parameter [31:0] CARVEOUT_BASE = 32'h4600_0000,
    parameter [31:0] CARVEOUT_SIZE = 32'h0200_0000,
    // Ring depth.  Must EXCEED PREFETCH_DEPTH+1 (scanout_line_fetch.v
    // checks it at elaboration): the lookahead window pins that many
    // slots, and eviction needs at least one more.
    parameter NUM_LINE_BUF  = 8,
    parameter PREFETCH_DEPTH  = 5,  // lines of lookahead ahead of the queue head
    parameter MAX_OUTSTANDING = 4,  // concurrent AXI read bursts
    parameter REFILL_THRESH   = 2,  // refill hysteresis (1 = trickle)
    parameter QDEPTH_LOG2   = 6     // request-queue depth = 2**6 = 64 entries
) (
    input  wire                  clk,
    input  wire                  rst,

    // ── Streaming read port -- vram.v-compatible (drop-in) ─────────────
    input  wire                  rd_clk,   // unused (see vram.v header: rd_clk MUST == clk)
    input  wire                  rd_rst,   // unused; this module's own `rst` governs state
    input  wire [ADDR_W-1:0]     rd_addr,
    input  wire                  rd_en,
    // 4-byte drain group (see also vram.v, which presents the SAME contract
    // for the URAM backend so the two stay drop-in interchangeable):
    //   [31:24] = byte at rd_addr        -- ALWAYS valid, any alignment
    //   [23:16] = byte at rd_addr+1  \
    //   [15:8]  = byte at rd_addr+2   >- valid ONLY when rd_addr[1:0]==0
    //   [7:0]   = byte at rd_addr+3  /
    // The alignment caveat is what removes ALL straddle handling: an aligned
    // 4-byte group cannot cross a LINE_BYTES boundary, so one ring line
    // always holds the whole group.  The indexed (1/2/4/8bpp) scan-out path
    // consumes only [31:24]; the 24bpp direct-colour path issues aligned
    // requests and takes [23:16]/[15:8]/[7:0] as R/G/B.
    output reg  [RD_DATA_W-1:0]  rd_data,
    output reg                   rd_valid,

    // ── AXI4 read-only master ───────────────────────────────────────────
    output wire [ID_WIDTH-1:0]   m_arid,
    output wire [31:0]           m_araddr,
    output wire [7:0]            m_arlen,
    output wire [2:0]            m_arsize,
    output wire [1:0]            m_arburst,
    output wire                  m_arvalid,
    input  wire                  m_arready,
    input  wire [ID_WIDTH-1:0]   m_rid,
    input  wire [DATA_WIDTH-1:0] m_rdata,
    input  wire [1:0]            m_rresp,
    input  wire                  m_rlast,
    input  wire                  m_rvalid,
    output wire                  m_rready
);

    // synthesis translate_off
    initial begin
        if (BPP != 8) begin
            $display("scanout_ddr_reader: only BPP=8 is supported (got %0d)", BPP);
            $fatal(1);
        end
        if (RD_DATA_W != 32) begin
            $display("scanout_ddr_reader: only RD_DATA_W=32 is supported (got %0d)", RD_DATA_W);
            $fatal(1);
        end
    end
    // synthesis translate_on

    // log2(line size).  7 => 128 B lines = 8 beats * 16 B/beat.  Raised from
    // 6 (64 B / 4 beats) to lift the single-outstanding burst ceiling -- see
    // the BANDWIDTH derivation in this file's header.
    localparam LINE_OFF_W   = 7;
    localparam LINE_IDX_W   = ADDR_W - LINE_OFF_W;

    wire [LINE_IDX_W-1:0] req_line = rd_addr[ADDR_W-1:LINE_OFF_W];
    wire [LINE_OFF_W-1:0] req_off  = rd_addr[LINE_OFF_W-1:0];

    // ── Request queue (correctness-critical, see header) ────────────────
    localparam QDEPTH = (1 << QDEPTH_LOG2);
    reg [LINE_IDX_W-1:0] q_line [0:QDEPTH-1];
    reg [LINE_OFF_W-1:0] q_off  [0:QDEPTH-1];
    reg [QDEPTH_LOG2-1:0] q_head, q_tail;
    reg [QDEPTH_LOG2:0]   q_count;

    wire q_empty = (q_count == {(QDEPTH_LOG2+1){1'b0}});
    wire q_full  = (q_count == QDEPTH[QDEPTH_LOG2:0]);
    wire [LINE_IDX_W-1:0] q_head_line = q_line[q_head];
    wire [LINE_OFF_W-1:0] q_head_off  = q_off[q_head];

    // synthesis translate_off
    always @(posedge clk) begin
        if (!rst && rd_en && q_full) begin
            $display("scanout_ddr_reader: request queue overflow -- caller exceeded the outstanding-request contract");
            $fatal(1);
        end
    end
    // synthesis translate_on

    wire        fetch_hit;
    wire [31:0] fetch_hit_data;

    scanout_line_fetch #(
        .LINE_IDX_W(LINE_IDX_W), .LINE_OFF_W(LINE_OFF_W),
        .ID_WIDTH(ID_WIDTH), .AXI_ID(AXI_ID),
        .CARVEOUT_BASE(CARVEOUT_BASE), .CARVEOUT_SIZE(CARVEOUT_SIZE),
        .NUM_LINE_BUF(NUM_LINE_BUF), .PREFETCH_DEPTH(PREFETCH_DEPTH),
        .MAX_OUTSTANDING(MAX_OUTSTANDING), .REFILL_THRESH(REFILL_THRESH),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_fetch (
        .clk(clk), .rst(rst),
        .req_valid(!q_empty), .req_line(q_head_line), .req_off(q_head_off),
        .hit(fetch_hit), .hit_data(fetch_hit_data),
        .m_arid(m_arid), .m_araddr(m_araddr), .m_arlen(m_arlen),
        .m_arsize(m_arsize), .m_arburst(m_arburst),
        .m_arvalid(m_arvalid), .m_arready(m_arready),
        .m_rid(m_rid), .m_rdata(m_rdata), .m_rresp(m_rresp),
        .m_rlast(m_rlast), .m_rvalid(m_rvalid), .m_rready(m_rready)
    );

    // ── Request-queue push (every accepted rd_en) + pop-and-respond
    //    (queue-head-only drain, see header) ─────────────────────────────
    wire pop_now = fetch_hit;

    always @(posedge clk) begin
        if (rst) begin
            q_head   <= {QDEPTH_LOG2{1'b0}};
            q_tail   <= {QDEPTH_LOG2{1'b0}};
            q_count  <= {(QDEPTH_LOG2+1){1'b0}};
            rd_data  <= {RD_DATA_W{1'b0}};
            rd_valid <= 1'b0;
        end else begin
            // Push
            if (rd_en) begin
                q_line[q_tail] <= req_line;
                q_off [q_tail] <= req_off;
                q_tail <= q_tail + {{(QDEPTH_LOG2-1){1'b0}}, 1'b1};
            end

            // Pop + respond (registered, 1-cycle latency from decision to
            // rd_valid, matching vram.v's registered-read class)
            if (pop_now) begin
                rd_data  <= fetch_hit_data;
                rd_valid <= 1'b1;
                q_head   <= q_head + {{(QDEPTH_LOG2-1){1'b0}}, 1'b1};
            end else begin
                rd_valid <= 1'b0;
            end

            // Occupancy accounting (push and pop can both happen the same
            // cycle).
            case ({rd_en, pop_now})
                2'b10: q_count <= q_count + {{(QDEPTH_LOG2){1'b0}}, 1'b1};
                2'b01: q_count <= q_count - {{(QDEPTH_LOG2){1'b0}}, 1'b1};
                default: q_count <= q_count;
            endcase
        end
    end

endmodule

`default_nettype wire
