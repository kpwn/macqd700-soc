// scanout_line_fetch.v -- ring buffer + AXI4 burst fetch engine for
// scanout_ddr_reader.v (T14, VRAM-in-DDR migration). Split out of
// scanout_ddr_reader.v to keep both modules under the project's
// ~300-line convention (docs/agent_policy.md).
//
// Owns: the NUM_LINE_BUF-deep line-buffer ring, the LOOKAHEAD prefetch
// window (tracked from the caller's QUEUE HEAD, not the latest
// acceptance -- see scanout_ddr_reader.v's header for why), the
// monotonic-consumption eviction-safety rule, and the AXI4 read-only
// master.
//
// ── 2026-08-19 REWORK: "sporadic and hidden" DDR usage ────────────────
// v1 had TWO self-imposed limits that together made scanout a CONSTANT,
// LATENCY-BOUND tax on the DDR path rather than a rare, deep excursion:
//
//   (a) ONE-LINE-AHEAD prefetch out of FOUR allocated line buffers.
//       Three of the four slots could never hold anything, so the ring
//       carried ~1 line (128 B) of latency tolerance.
//   (b) ONE AXI BURST IN FLIGHT.  v1's header justified this as avoiding
//       "widening the AXI ID field ... out of scope per the brief".  That
//       was a SCOPING decision, and on re-examination no widening is
//       needed AT ALL: docs/ddr4_mig_bridge_contract.md guarantees read
//       data returns strictly in AR-ACCEPTANCE order (up to
//       RMAX_OUTSTANDING=8), and axi_vram_priority_mux3.v's route FIFO
//       relies on exactly that same property already.  So N outstanding
//       bursts need only an N-deep in-order FIFO of ring-slot indices --
//       `os_slot` below -- and one shared AXI ID, unchanged.
//
// Consequence of (a)+(b): one 128 B line cost (round trip + BEATS)
// cycles END TO END and could not overlap the next, so the ring fill
// rate was ~128 B / (40 + 8) = 2.67 B/cycle and scanout had to be given
// near-absolute arbiter priority (axi_vram_priority_mux3.v's
// MAX_BULK_AHEAD=2) to make its deadline.  That cap was the binding
// limit on CPU/L2 miss concurrency -- the display was bounding the CPU's
// memory system, for want of buffering that was already allocated.
//
// NOW: PREFETCH_DEPTH lines of lookahead over NUM_LINE_BUF slots, up to
// MAX_OUTSTANDING concurrent bursts, and a REFILL_THRESH hysteresis so
// the engine sits idle until it owes itself REFILL_THRESH lines and then
// refills the WHOLE window back-to-back.  DDR usage becomes a burst of
// up to PREFETCH_DEPTH pipelined 8-beat reads every REFILL_THRESH line-
// times, instead of one latency-exposed read every line-time.  Fill rate
// becomes bounded by the R channel (16 B/cycle shared), not by the round
// trip; the round trip is absorbed by the lookahead window.
//
// SIZING RULE (checked at elaboration): NUM_LINE_BUF must exceed the
// lookahead window (PREFETCH_DEPTH + 1 lines: the head plus the
// lookahead) so a victim always exists, and MAX_OUTSTANDING must not
// exceed the window either (there is nothing else to fetch).
//
// Interface to the caller (scanout_ddr_reader.v): `req_valid`/`req_line`/
// `req_off` present the caller's QUEUE HEAD every cycle; `hit`/`hit_data`
// report combinationally whether that line is resident and, if so, its
// 4-byte group at `req_off`. The caller owns the request queue and the
// rd_data/rd_valid streaming-port timing entirely -- this module has no
// opinion on either.
//
// RESET: ring always resets. Bursts already ACCEPTED downstream (counted
// by `os_count`, plus an AR handshake completing on the SAME edge rst
// asserts -- f_state alone would misclassify that race, matching
// docs/l2c_spec.md's Important-A v2 aw_busy-vs-aw_open writeup) drain
// through the dedicated F_RSTDRAIN state (bounded 128 cycles or the owed
// RLAST count, mirroring l2c_mshr.v's S_DRAIN, docs/l2c_spec.md S5)
// instead of being abandoned, so the shared mux/downstream chain never
// wedges. An UNACCEPTED AR (F_AR, nothing committed downstream) is simply
// abandoned. This module's own reset fires essentially the same moment
// axi_bridge_stale_sink.v's reset_event does (same core-side reset), and
// that sink discards an abandoned burst's remaining beats -- including
// its RLAST -- on axi_async_bridge.v's m-side, before they ever reach
// this module's m_rvalid/m_rlast (downstream of the sink through the
// mux). So F_RSTDRAIN here typically exits via its 128-cycle TIMEOUT,
// not the owed-RLAST count (the sink already ate them) -- both exits are
// correct and equally cheap (this state only discards), so that's
// informational, not a defect.
//
`default_nettype none

module scanout_line_fetch #(
    parameter LINE_IDX_W    = 15,
    parameter LINE_OFF_W    = 6,
    parameter ID_WIDTH      = 6,
    parameter [ID_WIDTH-1:0] AXI_ID = {ID_WIDTH{1'b0}},
    parameter [31:0] CARVEOUT_BASE = 32'h4600_0000,
    parameter [31:0] CARVEOUT_SIZE = 32'h0200_0000,
    // Ring depth.  Must be > PREFETCH_DEPTH + 1 (see SIZING RULE above).
    parameter NUM_LINE_BUF   = 8,
    // Lines of lookahead ahead of the caller's queue head.  This is the
    // latency tolerance: PREFETCH_DEPTH * LINE_BYTES of data may be
    // in-ring-or-in-flight ahead of what the caller has asked for.
    parameter PREFETCH_DEPTH = 5,
    // Concurrent AXI read bursts.  Responses return in AR-acceptance
    // order (see header), so this needs no ID widening -- just the
    // MAX_OUTSTANDING-deep in-order slot FIFO below.
    parameter MAX_OUTSTANDING = 4,
    // Hysteresis: how many lookahead lines must be MISSING before a
    // (non-blocking) refill excursion starts.  1 disables the hysteresis
    // and restores continuous trickle-refill.  Larger = fewer, deeper,
    // rarer DDR excursions, at the cost of runway (the engine starts the
    // refill REFILL_THRESH line-times before the deepest resident line is
    // consumed).
    parameter REFILL_THRESH   = 2,
    parameter DATA_WIDTH     = 128
) (
    input  wire                      clk,
    input  wire                      rst,

    // ── Caller's queue-head query (combinational) ───────────────────────
    input  wire                      req_valid,
    input  wire [LINE_IDX_W-1:0]     req_line,
    input  wire [LINE_OFF_W-1:0]     req_off,
    output wire                      hit,
    // 4-byte drain group.  [31:24] is the byte at req_off (valid for ANY
    // req_off); [23:0] are the bytes at req_off+1/+2/+3 and are valid only
    // when req_off[1:0]==0.  An ALIGNED 4-byte group can never cross a
    // LINE_BYTES boundary (LINE_BYTES is a multiple of 4), which is what
    // removes all straddle handling from this module -- see the caller's
    // header for the port contract this feeds.
    output wire [31:0]               hit_data,

    // ── AXI4 read-only master ───────────────────────────────────────────
    output wire [ID_WIDTH-1:0]       m_arid,
    output wire [31:0]               m_araddr,
    output wire [7:0]                m_arlen,
    output wire [2:0]                m_arsize,
    output wire [1:0]                m_arburst,
    output reg                       m_arvalid,
    input  wire                      m_arready,
    input  wire [ID_WIDTH-1:0]       m_rid,
    input  wire [DATA_WIDTH-1:0]     m_rdata,
    input  wire [1:0]                m_rresp,
    input  wire                      m_rlast,
    input  wire                      m_rvalid,
    output wire                      m_rready
);

    // LINE_BYTES is DERIVED from LINE_OFF_W (rather than hardcoded at 64)
    // so the caller's request-queue offset width and this module's line
    // size can never silently disagree.  At the production LINE_OFF_W=7 a
    // line is 128 B = 8 beats * 16 B/beat.
    localparam LINE_BYTES  = (1 << LINE_OFF_W);
    localparam [31:0] LINE_BYTES32 = LINE_BYTES;
    localparam BEATS       = LINE_BYTES / 16;   // 16 B/beat (DATA_WIDTH=128)
    localparam [7:0] ARLEN = BEATS - 1;
    localparam BEAT_W      = LINE_OFF_W - 4;    // log2(BEATS)
    localparam RING_W      = (NUM_LINE_BUF <= 2) ? 1 : $clog2(NUM_LINE_BUF);
    // Width of a lookahead index K in 0..PREFETCH_DEPTH.
    localparam K_W         = $clog2(PREFETCH_DEPTH + 1);
    localparam KCNT_W      = $clog2(PREFETCH_DEPTH + 2);
    // Outstanding-burst FIFO geometry.
    localparam OS_W        = (MAX_OUTSTANDING <= 1) ? 1 : $clog2(MAX_OUTSTANDING);
    localparam OSC_W       = $clog2(MAX_OUTSTANDING + 1);

    // synthesis translate_off
    initial begin
        if (NUM_LINE_BUF <= (PREFETCH_DEPTH + 1)) begin
            $display("scanout_line_fetch: NUM_LINE_BUF (%0d) must EXCEED PREFETCH_DEPTH+1 (%0d) -- the lookahead window would leave no evictable victim",
                     NUM_LINE_BUF, PREFETCH_DEPTH + 1);
            $fatal(1);
        end
        if (MAX_OUTSTANDING > (PREFETCH_DEPTH + 1)) begin
            $display("scanout_line_fetch: MAX_OUTSTANDING (%0d) exceeds the lookahead window (%0d lines) -- there is nothing for the extra bursts to fetch",
                     MAX_OUTSTANDING, PREFETCH_DEPTH + 1);
            $fatal(1);
        end
        if (REFILL_THRESH < 1 || REFILL_THRESH > PREFETCH_DEPTH) begin
            $display("scanout_line_fetch: REFILL_THRESH must be in [1,PREFETCH_DEPTH]");
            $fatal(1);
        end
    end
    // synthesis translate_on

    reg                       lb_valid   [0:NUM_LINE_BUF-1];
    reg                       lb_fetching[0:NUM_LINE_BUF-1];
    // Set on a frame wrap for a slot whose burst is already in flight: the
    // data is about to be superseded, so the fill must NOT validate it.
    // (Without this the ring loses MAX_OUTSTANDING slots for a whole frame
    // -- the wrap flush cannot touch a fetching slot, and the `lb_line <
    // req_line` eviction rule cannot reclaim an OLD-frame line, whose index
    // is numerically LARGER than the new frame's.)
    reg                       lb_kill    [0:NUM_LINE_BUF-1];
    reg [LINE_IDX_W-1:0]      lb_line    [0:NUM_LINE_BUF-1];
    // Ring storage is one AXI BEAT (128 b) per entry, addressed by the flat
    // index {slot, beat}.  Within a beat the four 32-bit ring words keep the
    // historical big-endian-in-word layout: word j (line offset 16*beat+4j)
    // lives at bits [j*32 +: 32] with beat byte 4j+0 at that word's [31:24].
    //
    // 2026-08-19 -- reshaped from `reg [31:0] lb_word [0:SLOTS-1][0:WORDS-1]`
    // (see docs/mux_bram_review.md).  The old shape was unimplementable as a
    // RAM for TWO independent reasons and Vivado built all 4096 bits out of
    // discrete FFs plus a 128:1 x 32-bit combinational read mux -- 6,058 LUT
    // and 4,298 FF for what is 4 Kbit of line buffer:
    //   (a) the fill wrote FOUR words per cycle (one unrolled loop over the
    //       four 32-bit lanes of a 128-bit beat) = four write ports;
    //   (b) 2-D unpacked arrays are not a shape Vivado's RAM extractor
    //       recognises at all, even with a single {i,j} index pair.
    // Banking on the beat fixes both: ONE write port at {fill_slot,r_beat}
    // with the whole beat as data, and ONE asynchronous read port.  That is
    // the RAM32X1D / RAM64X1D pattern, so it maps to distributed RAM.  The
    // read stays fully combinational -- no latency or protocol change (`hit`
    // still pops the queue head in the same cycle).  BRAM is deliberately NOT
    // used here: a BRAM read port is REGISTERED, which would add a cycle to
    // `hit` and change the caller's drain protocol, for the sake of 8 Kbit.
    (* ram_style = "distributed" *)
    reg [127:0]               lb_beat    [0:(NUM_LINE_BUF*BEATS)-1];

    integer hs;
    integer kk;

    // ── Frame-wrap detection (review C1) ─────────────────────────────────
    // The ring had no frame-boundary flush and no coherence vs. CPU
    // writes: `hit` served a resident lb_line==req_line match FOREVER.
    // Two symptoms, one fix: (a) a line could stay hit-pinned across
    // many frames, silently serving stale content after a CPU repaint;
    // (b) for frames <= NUM_LINE_BUF+1 lines, the eviction rule
    // `lb_line[hs] < req_line` finds no victim at the wrap boundary
    // (every resident line is numerically LARGER than the new frame's
    // low req_line) -- permanent deadlock.
    //
    // Fix: req_line (the caller's queue HEAD) is architecturally
    // non-decreasing WITHIN one frame (consumption is monotonic -- the
    // same invariant the eviction rule already relies on), so any
    // DECREASE is unambiguously a new frame (vsync-equivalent) -- no
    // separate vsync-tick input needed. `last_req_line` snapshots the
    // previous cycle's req_line (only while req_valid); `frame_wrap_c`
    // fires for exactly ONE cycle per wrap. It MUST be a transition
    // detector, not a running max: a max-based design would stay
    // "wrapped" for the whole rest of the new frame (as long as
    // req_line is below the OLD frame's peak -- typically most of it),
    // permanently suppressing `hit` and re-flushing every freshly-
    // fetched slot the instant it becomes valid -- a frame-long
    // deadlock, not a fix. The transition detector self-clears the
    // cycle after the wrap since `last_req_line` catches up immediately.
    //
    // On `frame_wrap_c`, every NON-FETCHING slot is invalidated and every
    // FETCHING slot is marked `lb_kill` (below).  `hit` is ALSO gated off
    // combinationally on the wrap-detecting cycle itself: the flush's
    // effect is registered (next cycle), so without this gate a stale slot
    // could still win `head_hit_c` and pop one more stale byte first.  New
    // allocations are gated off for that one cycle too, so an allocation
    // can never race the flush.  The wrap-triggering request just sees one
    // miss-shaped cycle and self-corrects the very next cycle.
    reg [LINE_IDX_W-1:0] last_req_line;
    wire frame_wrap_c = req_valid && (req_line < last_req_line);

    // Is req_line already resident (hit)?
    reg                head_hit_c;
    reg [RING_W-1:0]   head_hit_slot_c;
    always @(*) begin
        head_hit_c      = 1'b0;
        head_hit_slot_c = {RING_W{1'b0}};
        for (hs = 0; hs < NUM_LINE_BUF; hs = hs + 1) begin
`ifdef SCANOUT_INJECT_LINESTART_UNDERRUN
            // FAULT INJECTION (sensitivity probe only): serve the ring even
            // while the line is still in flight -- i.e. exactly the
            // "line-start fetch has not returned, scanner consumes stale ring
            // contents" failure the underrun hypothesis predicts.
            if ((lb_valid[hs] || lb_fetching[hs]) && (lb_line[hs] == req_line)) begin
`else
            if (lb_valid[hs] && !lb_fetching[hs] && (lb_line[hs] == req_line)) begin
`endif
                head_hit_c      = 1'b1;
                head_hit_slot_c = hs[RING_W-1:0];
            end
        end
    end
    assign hit = req_valid && head_hit_c && !frame_wrap_c;

    // ── 4-byte drain assembly ────────────────────────────────────────────
    // ONE array lookup gets the aligned word containing req_off; a 4:1 byte
    // mux then places the byte AT req_off into the top lane so the
    // always-valid lane is correct for unaligned offsets too.  When
    // req_off[1:0]==0 the mux selects [31:24] and hit_data == hit_word_c
    // exactly, i.e. the aligned case is a straight pass-through.
    wire [127:0] hit_beat_c = lb_beat[{head_hit_slot_c, req_off[LINE_OFF_W-1:4]}];
    wire [31:0]  hit_word_c = hit_beat_c[{req_off[3:2], 5'd0} +: 32];
    reg  [7:0]  hit_byte_c;
    always @(*) begin
        case (req_off[1:0])
            2'd0:    hit_byte_c = hit_word_c[31:24];
            2'd1:    hit_byte_c = hit_word_c[23:16];
            2'd2:    hit_byte_c = hit_word_c[15:8];
            default: hit_byte_c = hit_word_c[7:0];
        endcase
    end
    assign hit_data = {hit_byte_c, hit_word_c[23:0]};

    // ── Lookahead window occupancy ───────────────────────────────────────
    // For each slot, its line's DISTANCE ahead of the queue head.  A slot
    // holding an already-consumed (older) line produces a huge modular
    // distance and so can never be mistaken for a window member.  Using the
    // distance rather than PREFETCH_DEPTH+1 separate `lb_line == req_line+K`
    // comparators keeps this to ONE subtractor per slot.
    wire [LINE_IDX_W-1:0] lb_dist_c [0:NUM_LINE_BUF-1];
    wire                  lb_in_win_c [0:NUM_LINE_BUF-1];
    wire                  lb_busy_c   [0:NUM_LINE_BUF-1];
    genvar gs;
    generate
        for (gs = 0; gs < NUM_LINE_BUF; gs = gs + 1) begin : g_dist
            assign lb_dist_c[gs]   = lb_line[gs] - req_line;
            assign lb_busy_c[gs]   = lb_valid[gs] || lb_fetching[gs];
            assign lb_in_win_c[gs] = lb_busy_c[gs] &&
                                     (lb_dist_c[gs] <= PREFETCH_DEPTH);
        end
    endgenerate

    // have_c[K] == 1 iff line (req_line + K) is resident OR in flight.
    reg [PREFETCH_DEPTH:0] have_c;
    always @(*) begin
        have_c = {(PREFETCH_DEPTH+1){1'b0}};
        for (hs = 0; hs < NUM_LINE_BUF; hs = hs + 1) begin
            if (lb_in_win_c[hs]) have_c[lb_dist_c[hs][K_W-1:0]] = 1'b1;
        end
    end

    // Lowest missing K.  K==0 is the caller's BLOCKING miss (the queue head
    // itself), so scanning low-to-high naturally serves demand before
    // lookahead.
    reg               alloc_want_c;
    reg [K_W-1:0]     alloc_k_c;
    always @(*) begin
        alloc_want_c = 1'b0;
        alloc_k_c    = {K_W{1'b0}};
        for (kk = PREFETCH_DEPTH; kk >= 0; kk = kk - 1) begin
            if (!have_c[kk]) begin
                alloc_want_c = 1'b1;
                alloc_k_c    = kk[K_W-1:0];
            end
        end
    end
    wire [LINE_IDX_W-1:0] alloc_target_c =
        req_line + {{(LINE_IDX_W-K_W){1'b0}}, alloc_k_c};

    // How many LOOKAHEAD lines (K >= 1) are absent -- the refill deficit.
    reg [KCNT_W-1:0] missing_ahead_c;
    always @(*) begin
        missing_ahead_c = {KCNT_W{1'b0}};
        for (kk = 1; kk <= PREFETCH_DEPTH; kk = kk + 1) begin
            if (!have_c[kk])
                missing_ahead_c = missing_ahead_c + {{(KCNT_W-1){1'b0}}, 1'b1};
        end
    end

    // ── Refill hysteresis ("sporadic and hidden") ────────────────────────
    // Idle until the deficit reaches REFILL_THRESH, then refill the WHOLE
    // window before going idle again.  This converts a steady one-burst-per-
    // line-time trickle into a REFILL_THRESH-deep pipelined excursion every
    // REFILL_THRESH line-times, which is what lets the arbiter hand the rest
    // of the DDR path to the CPU for the gaps in between.  A BLOCKING head
    // miss always arms it: falling behind is never a reason to wait.
    reg burst_arm;

    // ── Eviction safety (victim-hazard guard) ───────────────────────────
    // A slot is safe to reallocate only if unused, or its line is
    // STRICTLY OLDER than req_line (consumption is monotonic, so it can
    // never be needed again).  NUM_LINE_BUF > PREFETCH_DEPTH+1 (checked at
    // elaboration) is what guarantees one exists.
    reg                victim_found_c;
    reg [RING_W-1:0]   victim_slot_c;
    always @(*) begin
        victim_found_c = 1'b0;
        victim_slot_c  = {RING_W{1'b0}};
        for (hs = 0; hs < NUM_LINE_BUF; hs = hs + 1) begin
            if (!lb_valid[hs] && !lb_fetching[hs]) begin
                if (!victim_found_c) begin
                    victim_found_c = 1'b1;
                    victim_slot_c  = hs[RING_W-1:0];
                end
            end
        end
        if (!victim_found_c) begin
            for (hs = 0; hs < NUM_LINE_BUF; hs = hs + 1) begin
                if (lb_valid[hs] && !lb_fetching[hs] &&
                    (!req_valid || (lb_line[hs] < req_line))) begin
                    if (!victim_found_c) begin
                        victim_found_c = 1'b1;
                        victim_slot_c  = hs[RING_W-1:0];
                    end
                end
            end
        end
    end

    // ── Outstanding-burst bookkeeping (in-order, no ID widening) ────────
    // Read data returns in AR-ACCEPTANCE order (docs/ddr4_mig_bridge_
    // contract.md), and axi_vram_priority_mux3.v's route FIFO delivers
    // this master's bursts back to it in that same order, so an in-order
    // FIFO of ring-slot indices is a COMPLETE reorder record.  The shared
    // AXI_ID is unchanged; no field anywhere in the chain widens.
    reg [RING_W-1:0]  os_slot [0:MAX_OUTSTANDING-1];
    reg [OS_W-1:0]    os_wptr, os_rptr;
    reg [OSC_W-1:0]   os_count;
    reg [BEAT_W-1:0]  r_beat;

    localparam F_IDLE = 2'd0, F_AR = 2'd1, F_RSTDRAIN = 2'd2;
    reg [1:0]             f_state;
    reg [RING_W-1:0]      f_slot;
    reg [LINE_IDX_W-1:0]  f_line;
    reg [OSC_W-1:0]       drain_owed;
    reg [7:0]             drain_ctr;

    wire [RING_W-1:0] fill_slot_c = os_slot[os_rptr];
    wire fill_active_c = (os_count != {OSC_W{1'b0}}) && (f_state != F_RSTDRAIN);
    wire fill_beat_c   = fill_active_c && m_rvalid;
    wire fill_last_c   = fill_beat_c && m_rlast;
    // A wrap on the very cycle a burst completes must still suppress it:
    // `lb_kill` is registered, so it cannot have been set yet.
    wire fill_kill_c   = lb_kill[fill_slot_c] || frame_wrap_c;

    assign m_arid    = AXI_ID;
    assign m_araddr  = CARVEOUT_BASE + {f_line, {LINE_OFF_W{1'b0}}};
    assign m_arlen   = ARLEN;     // BEATS beats (8 at LINE_BYTES=128)
    assign m_arsize  = 3'd4;      // 16 B/beat
    assign m_arburst = 2'b01;     // INCR
    // Drain unconditionally through rst (accept and discard whatever
    // beats are already committed) and through the dedicated F_RSTDRAIN
    // state.
    assign m_rready  = fill_active_c || (f_state == F_RSTDRAIN) || rst;

    // Fill-side byte swizzle: reverse the byte order within each 32-bit lane
    // of the incoming beat, so lane j of the stored beat is
    // {byte(4j+0), byte(4j+1), byte(4j+2), byte(4j+3)} -- byte-for-byte the
    // value the old per-word fill loop wrote into lb_word[slot][{beat,j}].
    wire [127:0] beat_swizzle_c;
    genvar gw;
    generate
        for (gw = 0; gw < 4; gw = gw + 1) begin : g_beat_swizzle
            assign beat_swizzle_c[gw*32 +: 32] =
                { m_rdata[(gw*4+0)*8 +: 8],
                  m_rdata[(gw*4+1)*8 +: 8],
                  m_rdata[(gw*4+2)*8 +: 8],
                  m_rdata[(gw*4+3)*8 +: 8] };
        end
    endgenerate

    // synthesis translate_off
    always @(posedge clk) begin
        if (!rst && m_arvalid) begin
            if (m_araddr < CARVEOUT_BASE || (m_araddr + LINE_BYTES32) > (CARVEOUT_BASE + CARVEOUT_SIZE)) begin
                $display("scanout_line_fetch: ARADDR 0x%08x escapes carveout [0x%08x..0x%08x)",
                          m_araddr, CARVEOUT_BASE, CARVEOUT_BASE + CARVEOUT_SIZE);
                $fatal(1);
            end
        end
        if (!rst && fill_beat_c && m_rlast && (r_beat != (BEATS-1))) begin
            $display("scanout_line_fetch: RLAST at beat %0d, expected %0d", r_beat, BEATS-1);
            $fatal(1);
        end
    end
`ifdef SCANOUT_RESET_DEBUG
    reg [7:0] dbg_ar_issued, dbg_rlast_seen;
    reg dbg_rst_q;
    always @(posedge clk) begin
        dbg_rst_q <= rst;
        if (m_arvalid && m_arready) dbg_ar_issued <= dbg_ar_issued + 8'd1;
        if (m_rvalid && m_rlast && m_rready) dbg_rlast_seen <= dbg_rlast_seen + 8'd1;
        if (rst && !dbg_rst_q)
            $display("[%0t] SCANOUT_RESET_DEBUG: reset-assert ar_issued=%0d rlast_seen=%0d outstanding=%0d f_state=%0d",
                      $time, dbg_ar_issued, dbg_rlast_seen, dbg_ar_issued - dbg_rlast_seen, f_state);
        if (!rst && dbg_rst_q)
            $display("[%0t] SCANOUT_RESET_DEBUG: reset-deassert ar_issued=%0d rlast_seen=%0d outstanding=%0d",
                      $time, dbg_ar_issued, dbg_rlast_seen, dbg_ar_issued - dbg_rlast_seen);
    end
`endif
    // synthesis translate_on

    // A refill excursion may start when the deficit has built up, or
    // immediately on a blocking head miss.
    wire arm_now_c   = (missing_ahead_c >= REFILL_THRESH) || !have_c[0];
    wire disarm_now_c = (missing_ahead_c == {KCNT_W{1'b0}}) && have_c[0];
    wire alloc_ok_c  = req_valid && !frame_wrap_c && alloc_want_c &&
                       victim_found_c &&
                       (os_count < MAX_OUTSTANDING[OSC_W-1:0]) &&
                       ((alloc_k_c == {K_W{1'b0}}) || burst_arm);

    wire os_push_c = (f_state == F_AR) && m_arvalid && m_arready;
    wire os_pop_c  = fill_last_c;

    // ── Reset-drain accounting ──────────────────────────────────────────
    // `rst` can be held for many cycles (a pclk-domain reset synchronised
    // into this domain is typically ~10-20 clocks wide), and m_rready is
    // asserted THROUGHOUT it, so stale RLASTs can and do retire while rst
    // is still high.  The owed count must therefore be latched ONCE on
    // entry and then decremented on every RLAST seen -- recomputing it from
    // os_count each reset cycle would reset it to zero on cycle 2 (os_count
    // is cleared on cycle 1) and drop the drain state on the floor.  That
    // is not hypothetical: with one burst in flight the whole burst fitted
    // inside the reset window so the bug was invisible; with
    // MAX_OUTSTANDING bursts it is not, and the tail of burst 2 gets
    // mis-attributed to the first POST-reset burst (caught by the
    // short-RLAST assertion above).
    wire [OSC_W-1:0] rst_owed_base_c =
        (f_state == F_RSTDRAIN) ? drain_owed
                                : (os_count + (os_push_c ? {{(OSC_W-1){1'b0}}, 1'b1}
                                                        : {OSC_W{1'b0}}));
    wire rst_rlast_c = m_rvalid && m_rlast;   // m_rready is high through rst
    wire [OSC_W-1:0] rst_owed_next_c =
        (rst_rlast_c && (rst_owed_base_c != {OSC_W{1'b0}}))
            ? (rst_owed_base_c - {{(OSC_W-1){1'b0}}, 1'b1})
            : rst_owed_base_c;

    integer li;
    always @(posedge clk) begin
        if (rst) begin
            f_slot    <= {RING_W{1'b0}};
            f_line    <= {LINE_IDX_W{1'b0}};
            m_arvalid <= 1'b0;
            r_beat    <= {BEAT_W{1'b0}};
            os_wptr   <= {OS_W{1'b0}};
            os_rptr   <= {OS_W{1'b0}};
            os_count  <= {OSC_W{1'b0}};
            burst_arm <= 1'b0;
            // Drain every burst already ACCEPTED downstream: os_count, plus
            // an AR handshake completing on THIS edge (see header for the
            // race this guards against).  See `rst_owed_base_c` above for
            // why this is latch-once-then-decrement, not recompute.
            drain_owed <= rst_owed_next_c;
            if (f_state != F_RSTDRAIN) begin
                drain_ctr <= 8'd0;
                f_state   <= (rst_owed_next_c == {OSC_W{1'b0}}) ? F_IDLE
                                                                : F_RSTDRAIN;
            end else if ((rst_owed_next_c == {OSC_W{1'b0}}) ||
                         (drain_ctr >= 8'd127)) begin
                f_state   <= F_IDLE;
            end else begin
                drain_ctr <= drain_ctr + 8'd1;
                f_state   <= F_RSTDRAIN;
            end
            for (li = 0; li < NUM_LINE_BUF; li = li + 1) begin
                lb_valid[li]    <= 1'b0;
                lb_fetching[li] <= 1'b0;
                lb_kill[li]     <= 1'b0;
                lb_line[li]     <= {LINE_IDX_W{1'b0}};
            end
            last_req_line <= {LINE_IDX_W{1'b0}};
        end else begin
            // ── Refill hysteresis ────────────────────────────────────
            if (arm_now_c)         burst_arm <= 1'b1;
            else if (disarm_now_c) burst_arm <= 1'b0;

            // ── Fill side (independent of the AR-issue FSM) ──────────
            if (fill_beat_c) begin
                // One 16 B beat lands in ONE ring entry per cycle.  Beat
                // byte i is m_rdata[i*8 +: 8] and belongs at line offset
                // r_beat*16 + i; `beat_swizzle_c` (above) reverses the
                // bytes within each 32-bit lane so ring word j keeps beat
                // byte 4j at [31:24], matching the hit_data contract.
                lb_beat[{fill_slot_c, r_beat}] <= beat_swizzle_c;
                if (m_rlast) begin
                    lb_valid[fill_slot_c]    <= (m_rresp == 2'b00) && !fill_kill_c;
                    lb_fetching[fill_slot_c] <= 1'b0;
                    lb_kill[fill_slot_c]     <= 1'b0;
                    r_beat                   <= {BEAT_W{1'b0}};
                    os_rptr <= (os_rptr == (MAX_OUTSTANDING-1)) ? {OS_W{1'b0}}
                                                                : (os_rptr + {{(OS_W-1){1'b0}}, 1'b1});
                end else begin
                    r_beat <= r_beat + {{(BEAT_W-1){1'b0}}, 1'b1};
                end
            end

            // ── AR-issue FSM ─────────────────────────────────────────
            case (f_state)
                F_IDLE: begin
                    m_arvalid <= 1'b0;
                    if (alloc_ok_c) begin
                        f_slot                     <= victim_slot_c;
                        f_line                     <= alloc_target_c;
                        lb_line[victim_slot_c]     <= alloc_target_c;
                        lb_valid[victim_slot_c]    <= 1'b0;
                        lb_fetching[victim_slot_c] <= 1'b1;
                        lb_kill[victim_slot_c]     <= 1'b0;
                        m_arvalid                  <= 1'b1;
                        f_state                    <= F_AR;
                    end
                end

                F_AR: begin
                    if (m_arvalid && m_arready) begin
                        m_arvalid <= 1'b0;
                        f_state   <= F_IDLE;
                    end
                end

                F_RSTDRAIN: begin
                    // Accept+discard beats (m_rready asserted
                    // unconditionally here); bounded by the owed RLAST
                    // count or 128 cycles, whichever first (l2c_mshr.v's
                    // own bound).  Same accounting as the rst branch.
                    drain_owed <= rst_owed_next_c;
                    if ((rst_owed_next_c == {OSC_W{1'b0}}) ||
                        (drain_ctr >= 8'd127)) begin
                        f_state <= F_IDLE;
                    end else begin
                        drain_ctr <= drain_ctr + 8'd1;
                    end
                end

                default: f_state <= F_IDLE;
            endcase

            // ── Outstanding-burst FIFO ───────────────────────────────
            if (os_push_c) begin
                os_slot[os_wptr] <= f_slot;
                os_wptr <= (os_wptr == (MAX_OUTSTANDING-1)) ? {OS_W{1'b0}}
                                                            : (os_wptr + {{(OS_W-1){1'b0}}, 1'b1});
            end
            case ({os_push_c, os_pop_c})
                2'b10:   os_count <= os_count + {{(OSC_W-1){1'b0}}, 1'b1};
                2'b01:   os_count <= os_count - {{(OSC_W-1){1'b0}}, 1'b1};
                default: os_count <= os_count;
            endcase

            // Frame-wrap bookkeeping + flush (review C1).  Textually AFTER
            // everything above so it composes with those writes: a slot
            // F_IDLE is simultaneously allocating reads `!lb_fetching[li]`
            // as the OLD (pre-cycle) value -- but allocation is gated off
            // during a wrap (`alloc_ok_c`), so that case cannot arise.  A
            // slot COMPLETING its fill this cycle is excluded from the kill
            // arm because `fill_kill_c` has already suppressed it.
            // `last_req_line` updates whenever req_valid regardless of
            // hit/miss/wrap so frame_wrap_c stays a 1-cycle transition
            // detector, never a sticky/latched state.
            if (req_valid) last_req_line <= req_line;
            if (frame_wrap_c) begin
                for (li = 0; li < NUM_LINE_BUF; li = li + 1) begin
                    if (!lb_fetching[li]) begin
                        lb_valid[li] <= 1'b0;
                    end else if (!(fill_last_c && (fill_slot_c == li[RING_W-1:0]))) begin
                        lb_kill[li] <= 1'b1;
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
