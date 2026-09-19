// l2c_bypass.v -- never-allocate bypass-window pass-through engine,
//                 PIPELINED (many bypass beats outstanding at once).
//
// A parameterized list of {base, mask} windows (v1: NUM_WINDOWS=1,
// default disabled -- sized for a future VRAM-in-DDR carve-out) that
// never enter the tag/data arrays: matching requests are forwarded
// straight to the shared AXI master port and back.  Precondition:
// bypass windows are address-disjoint from the cacheable RAM/ROM/FB span
// -- asserted below (translate_off/on).  That disjointness is also what
// makes it sound for this engine to order nothing against the cache
// path: no address can ever reach both.
//
// WHY THIS MODULE WAS REWRITTEN (2026-08-20, measured in 7d49e1f).
// v1 was a single request/response engine, S_IDLE -> S_AW -> S_W -> S_B
// and S_IDLE -> S_AR -> S_R, so it ran ONE FULL DDR ROUND TRIP PER 16 B
// BEAT.  l2c_ctrl decomposes a multi-beat bypass burst into one per-beat
// request, so a 512 B (32-beat) transfer paid 32 serialized round trips:
//
//     cycles/beat ~ 3 + DDR_latency      (43.0 at L=40, 203.0 at L=200)
//     512 B read: 1376 cycles = 37.2 MB/s at 100 MHz
//
// Worse, the stall was VISIBLE TO UNRELATED TRAFFIC.  Every beat of the
// burst sat at the front door refused by l2c_ctrl's Critical-3 id_busy_c
// hold-off, which meant the burst owned l2c_ctrl's single `ar_have`
// tracker for the whole 1376 cycles and CPU hits queued behind it:
// mixed_cpu_hits_plus_bypass measured 23.2 cycles per hit-equivalent
// against 3.00 for the same hits alone, with 5040 of 5430 front-door
// idle cycles attributed to idbusy.
//
// So: SLOTS entries (default 8) and no round trip in the accept path.
// Four pointers over one slot ring, all CW = log2(SLOTS)+1 bits wide:
//
//   wptr -- next slot a front-door accept lands in
//   dptr -- slot the AR or AW/W sequencer is presenting; advances at
//           AR-accept (reads) or W-accept (writes)
//   fptr -- oldest slot awaiting its R/B; advances when one arrives
//   cptr -- oldest slot whose response has not been handed back to
//           l2c.v's R/B arbiter; advances on the rsp handshake
//
//   cptr <= fptr <= dptr <= wptr  (mod 2*SLOTS), occupancy = wptr - cptr
//
// A slot is NOT freed when its DRAM response lands -- only when that
// response has actually been delivered upstream.  That is what lets
// rsp_id be a single register (see ONE ID AT A TIME) instead of a
// per-slot field, and it is why cptr exists separately from fptr.
//
// ORDERING IS EXACTLY AS STRONG AS v1's.  This is a pure pipelining
// change; nothing the engine promised got weaker.
//
//   * ONE ID AT A TIME.  req_ready refuses a request whose id differs
//     from the queue's current id until the queue drains, so every
//     outstanding transaction carries the same ARID/AWID.  AXI4 then
//     REQUIRES the slave to return their responses in acceptance order,
//     which is what makes the purely positional bookkeeping above legal
//     -- the same argument l2c_victim.v makes for its single wb_id, and
//     the reason neither needs a response-matching CAM.  It costs
//     nothing on the traffic that matters: l2c_ctrl decomposes one burst
//     into per-beat requests that all carry that burst's own id, so a
//     512 B transfer is 32 same-id beats and pipelines end to end.
//   * NO READ/WRITE OVERLAP.  AXI orders nothing between the R and B
//     channels, so a read issued while a write is outstanding could pass
//     it inside the slave and return pre-write data.  v1 made that
//     impossible by construction (a write's BRESP retired before the
//     next request was even issued).  Here `dir_ok_c` withholds dispatch
//     when the slot at dptr disagrees in direction with the ops already
//     in flight, so a direction change drains first.  Back-to-back
//     accesses to the same bypass address therefore still resolve in
//     front-door acceptance order, read or write.  Real bypass traffic
//     (SD / RAM-disk block transfers) is unidirectional per burst, so
//     this costs one drain per direction change and nothing else.
//
// ID bit [ID_WIDTH-1] is forced to 1 on every outbound transaction so
// top-level response routing can tell bypass traffic apart from core
// traffic without a CAM.
//
// PAYLOAD IN LUTRAM, NOT FLOPS (878d002 / l2c_victim.v precedent).  Two
// FLAT 1-D arrays with `(* ram_style = "distributed" *)`, which is the
// shape that actually infers here (a 2-D unpacked array does NOT):
//
//   s_req  -- {addr, wdata, wstrb}, 176 b.  Written at wptr by the
//             front-door accept, read at dptr by the sequencer.
//   s_rdat -- read data, 128 b.  Written at fptr when R lands, read at
//             cptr by the response port.
//
// They are SEPARATE arrays because distributed RAM has ONE write port and
// there are two independent writers (accept and R-capture); merging them
// would force a shared-port arbitration that buys nothing.  Both reads
// are asynchronous, so the sequencer and the response port are
// cycle-for-cycle what a flop array would have been -- no pipeline stage,
// nothing new to get wrong.  VERIFY, don't assume: Vivado prints "Trying
// to implement RAM 'X' in registers" when inference fails.  Per-slot
// control bits (s_wr/s_need/s_last/s_resp) stay in flops: they are five
// bits, and s_wr is read at two different pointers in the same cycle.
//
// RESET (IMPORTANT-A, same shape as l2c_victim.v).  Bypass WRITES are the
// half that needs its own bookkeeping:
//
//   bout_cnt  -- AW-accepted minus B-received.  DRAM owes us exactly
//                this many BRESPs.  Deliberately does NOT clear on rst.
//   bsink_cnt -- how many of those belong to PRE-reset transactions whose
//                slots no longer exist.  Loaded from bout_cnt at reset;
//                while it is nonzero no new AW is presented, so every B
//                arriving in that window is unambiguously a stray and
//                frees no slot.
//
// AW never runs ahead of its own single W beat (D_HDR is left for D_W on
// AW-accept), so at most ONE transaction can be AW-accepted with its W
// unsent; D_RSTW completes that one with a wstrb=0 filler beat (data is
// irrelevant once no byte lane is enabled) and D_RSTB then consumes the
// BRESPs DRAM still owes.  Those must be sunk rather than left to arrive
// later and get mis-consumed as some LATER transaction's completion -- a
// perpetual off-by-one.  l2c.v's write-port arbiter correspondingly
// carries a BYPASS-owned grant and its own aw_out count across reset for
// exactly this window; without that the strays would route to l2c_victim
// and be mis-consumed as one of ITS writebacks' completion.
//
// Reads need no such counter: l2c.v's read route queue clears on reset
// and routes any late pre-reset R into l2c_mshr's S_DRAIN sink, which
// already has to absorb up to eight strays because the MSHR fill path is
// itself multi-outstanding.  Bypass reads join that existing regime
// rather than adding a second one.
//
// Verilog-2005, sync active-high rst, up to SLOTS single-beat
// transactions outstanding at a time.

`default_nettype none

module l2c_bypass #(
    parameter ADDR_WIDTH    = 32,
    parameter ID_WIDTH      = 6,
    parameter NUM_WINDOWS   = 1,
    // Packed {base,mask} pairs, one per window.  MASK CONVENTION: 1 bits
    // are the window's FIXED (tag) bits, which must equal BASE's own
    // bits at those positions to hit; 0 bits are the in-window OFFSET,
    // don't-cares for the match.  A contiguous-high-bits mask like
    // 0xFFF0_0000 therefore selects a 1MB window (2**20, the popcount of
    // the 0 bits) -- decoded below as `(addr & mask) == (base & mask)`.
    parameter [NUM_WINDOWS*ADDR_WIDTH-1:0] WIN_BASE = {ADDR_WIDTH{1'b0}},
    parameter [NUM_WINDOWS*ADDR_WIDTH-1:0] WIN_MASK = {ADDR_WIDTH{1'b0}},
    parameter [NUM_WINDOWS-1:0]            WIN_EN   = {NUM_WINDOWS{1'b0}},
    // Cacheable span, for the disjointness assertion only.
    parameter [ADDR_WIDTH-1:0] CACHEABLE_BASE = 32'h0000_0000,
    parameter [ADDR_WIDTH-1:0] CACHEABLE_SIZE = 32'h4100_0000,
    // Queue depth = how many bypass beats may be outstanding.  Power of
    // two, 1..32.  SLOTS=1 reproduces the pre-2026-08-20 serialized
    // engine's throughput and exists so the depth sweep has an in-harness
    // reference point.  The default is set by l2c.v's BYPASS_SLOTS, whose
    // comment carries the argument for 8 (short version: the isolated
    // model does not saturate before 32, but axi_ddr4_mig_bridge takes
    // only 8 outstanding reads / 4 writes, so past 8 the extra slots move
    // the queue rather than shorten the wait).  Measured curve in
    // docs/l2c_perf.md S14.3.
    parameter SLOTS         = 8,
    // Set when an async boundary below this master owns one-sided-reset
    // completion. Replaying filler beats here would duplicate that padding.
    parameter EXTERNAL_RESET_RECOVERY = 0
) (
    input  wire                     clk,
    input  wire                     rst,

    // Address classification (pure function -- usable by l2c.v's front
    // door before it even routes the request here).
    input  wire [ADDR_WIDTH-1:0]    match_addr,
    output wire                     match_hit,
    // Second, independent classification port (same pure function).  The
    // front door needs the WRITE-side address classified in a cycle where
    // `match_addr` may be carrying the READ-side address instead -- see
    // l2c_ctrl.v's flw_arm_c.  Deriving one from the other would put the
    // read/write select in the arming cone and close a combinational loop
    // (write_avail -> read_sel -> write_avail), so it gets its own port.
    input  wire [ADDR_WIDTH-1:0]    match_addr2,
    output wire                     match_hit2,

    // Front-door request (one beat at a time; up to SLOTS may be in
    // flight, and they all carry the same id -- see the header).
    input  wire                     req_valid,
    output wire                     req_ready,
    input  wire                     req_is_write,
    input  wire [ADDR_WIDTH-1:0]    req_addr,
    /* verilator lint_off UNUSEDSIGNAL */
    // Top bit intentionally dropped -- m_awid/m_arid below replace it
    // with the fixed bypass-engine tag bit (see l2c_defs.vh ID-tagging
    // note).  The rest is latched into q_id and echoed back on rsp_id.
    input  wire [ID_WIDTH-1:0]      req_id,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire [127:0]             req_wdata,
    input  wire [15:0]              req_wstrb,
    input  wire                     req_need_resp,
    // This engine services a multi-beat s_axi burst one BEAT at a time
    // (l2c_ctrl.v decomposes it and re-fires req_valid per beat) --
    // req_last threads through whether THIS beat is the ORIGINAL burst's
    // actual last beat, so rsp_last below can be correct instead of
    // unconditionally 1 (see rsp_last for the bug this fixes).
    input  wire                     req_last,

    // Response back to l2c.v's arbitrated R/B channels.
    output wire                     rsp_valid,
    input  wire                     rsp_ready,
    output wire                     rsp_is_write,
    output wire [ID_WIDTH-1:0]      rsp_id,
    output wire [127:0]             rsp_rdata,
    output wire [1:0]               rsp_resp,
    // Round-4 fix (probe-confirmed via the CRITICAL mask fix's first
    // real multi-beat soak): previously hardcoded to 1'b1 in l2c.v's
    // response mux, which forced RLAST on EVERY beat of a multi-beat
    // read into a bypass window -- the requester's per-id tracking
    // closed the burst out after just the FIRST beat, then flagged
    // every subsequent real beat as an unmatched/phantom R (protocol
    // violation) and read stale/wrong data for beats 1+.  Now echoes the
    // per-slot req_last latched at accept time.
    output wire                     rsp_last,

    // Active-transaction query (combinational) -- l2c.v threads this to
    // l2c_ctrl's front door so it can hold off accepting a NEW request
    // whose ID matches an outstanding bypass op (AXI4 same-ID response
    // ordering, see docs/l2c_spec.md S9 / Critical-3 fix).  Valid
    // whenever this engine holds an accepted-but-undelivered op.  Note
    // that l2c_ctrl qualifies it with !is_bypass_c: a request that is
    // ITSELF headed here needs no hold-off, because this queue already
    // delivers same-id responses in acceptance order.  Applying it there
    // anyway is what serialized every beat of a burst in v1.
    output wire                     active_valid,
    output wire [ID_WIDTH-1:0]      active_id,

    // AXI-master sub-port (arbitrated onto the shared physical port).
    output wire [ID_WIDTH-1:0]      m_awid,
    output wire [ADDR_WIDTH-1:0]    m_awaddr,
    output wire [7:0]               m_awlen,
    output wire [2:0]               m_awsize,
    output wire [1:0]               m_awburst,
    output wire                     m_awvalid,
    input  wire                     m_awready,
    output wire [127:0]             m_wdata,
    output wire [15:0]              m_wstrb,
    output wire                     m_wlast,
    output wire                     m_wvalid,
    input  wire                     m_wready,
    /* verilator lint_off UNUSEDSIGNAL */
    // All outstanding writes share one AWID, so AXI4 orders their BRESPs
    // and retirement is positional -- BID carries no information here.
    input  wire [ID_WIDTH-1:0]      m_bid,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire [1:0]               m_bresp,
    input  wire                     m_bvalid,
    output wire                     m_bready,

    output wire [ID_WIDTH-1:0]      m_arid,
    output wire [ADDR_WIDTH-1:0]    m_araddr,
    output wire [7:0]               m_arlen,
    output wire [2:0]               m_arsize,
    output wire [1:0]               m_arburst,
    output wire                     m_arvalid,
    input  wire                     m_arready,
    /* verilator lint_off UNUSEDSIGNAL */
    // Single-beat ARLEN=0 reads only, all sharing one ARID -- RID/RLAST
    // carry nothing the pointer bookkeeping does not already know.
    input  wire [ID_WIDTH-1:0]      m_rid,
    input  wire                     m_rlast,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire [127:0]             m_rdata,
    input  wire [1:0]               m_rresp,
    input  wire                     m_rvalid,
    output wire                     m_rready
);

    // -- Window match (combinational) --------------------------------------
    wire [NUM_WINDOWS-1:0] win_hit;
    wire [NUM_WINDOWS-1:0] win_hit2;
    genvar gw;
    generate
        for (gw = 0; gw < NUM_WINDOWS; gw = gw + 1) begin : g_win
            wire [ADDR_WIDTH-1:0] base = WIN_BASE[gw*ADDR_WIDTH +: ADDR_WIDTH];
            wire [ADDR_WIDTH-1:0] mask = WIN_MASK[gw*ADDR_WIDTH +: ADDR_WIDTH];
            // CRITICAL fix (survived 3 review rounds): mask convention is
            // FIXED HIGH BITS = 1 (the window's base/tag), the complement
            // (low bits) = the in-window OFFSET -- matching both the
            // disjointness assert below (size = ~WIN_MASK+1, only valid
            // if the low ~WIN_MASK bits are the offset) and every WIN_MASK
            // parameter value ever passed in (e.g. 0xFFF0_0000 = 1MB
            // window, top 12 bits fixed).  The compare must therefore
            // match on the FIXED bits (`& mask`), not the offset bits
            // (`& ~mask`, the bug) -- the inverted-sense version let an
            // in-window address whose offset bits differ from the base's
            // own (e.g. base 0x0000_0000 mask 0xFFF0_0000, probe
            // 0x0100_0100) take the CACHE path instead of bypass (breaks
            // invariant 1, S1: VRAM-in-DDR would thrash L2), while a
            // cacheable address that happened to share the window's
            // OFFSET bits (e.g. 0x0010_0000) took the BYPASS path instead
            // of cache (breaks invariant 2: a dirty cached line's
            // eviction writeback can land on and clobber a later bypass
            // write to the same physical DRAM address -- real corruption,
            // not just a classification error).
            assign win_hit[gw]  = WIN_EN[gw] && ((match_addr  & mask) == (base & mask));
            assign win_hit2[gw] = WIN_EN[gw] && ((match_addr2 & mask) == (base & mask));
        end
    endgenerate
    assign match_hit  = |win_hit;
    assign match_hit2 = |win_hit2;

    // synthesis translate_off
    generate
        for (gw = 0; gw < NUM_WINDOWS; gw = gw + 1) begin : g_assert
            always @(posedge clk) begin
                if (WIN_EN[gw]) begin
                    if ((WIN_BASE[gw*ADDR_WIDTH +: ADDR_WIDTH] + (~WIN_MASK[gw*ADDR_WIDTH +: ADDR_WIDTH] + 1)) > CACHEABLE_BASE &&
                        WIN_BASE[gw*ADDR_WIDTH +: ADDR_WIDTH] < (CACHEABLE_BASE + CACHEABLE_SIZE)) begin
                        // $fatal (not $display, Minor-15) -- a bypass
                        // window overlapping the cacheable span breaks
                        // the point-of-coherency invariant (docs/
                        // l2c_spec.md S1 invariant 1); this must stop
                        // the sim, not just log a line that's easy to
                        // miss in a long run.
                        $display("L2C_BYPASS ASSERT: window %0d overlaps cacheable span [0x%08x,0x%08x)",
                                 gw, CACHEABLE_BASE, CACHEABLE_BASE + CACHEABLE_SIZE);
                        $fatal(1);
                    end
                    // 64 B GRANULARITY, load-bearing for l2c_ctrl.v's
                    // full-line-write gather: that path classifies a line
                    // ONCE, on its 64 B-aligned base address, and then
                    // gathers all four 16 B quadrants without re-asking.
                    // That is only sound if every address inside a
                    // 64 B-aligned block classifies identically, i.e. the
                    // window mask leaves bits [5:0] as don't-care offset
                    // bits.  Every window ever passed in is >= 1 MB
                    // (0xFFF0_0000, 0xF000_0000), so this is free today --
                    // it exists so a future sub-64 B window trips the tb
                    // instead of silently caching a bypass-window line.
                    if ((WIN_MASK[gw*ADDR_WIDTH +: ADDR_WIDTH] & {{(ADDR_WIDTH-6){1'b0}}, 6'h3F}) != {ADDR_WIDTH{1'b0}}) begin
                        $display("L2C_BYPASS ASSERT: window %0d mask 0x%08x is finer than 64 B",
                                 gw, WIN_MASK[gw*ADDR_WIDTH +: ADDR_WIDTH]);
                        $fatal(1);
                    end
                end
            end
        end
    endgenerate
    // synthesis translate_on

    // -- Slot ring ---------------------------------------------------------
    localparam SB = (SLOTS <= 2)  ? 1 : (SLOTS <= 4)  ? 2 :
                    (SLOTS <= 8)  ? 3 : (SLOTS <= 16) ? 4 : 5;
    localparam CW = SB + 1;                     // pointer / counter width
    localparam [CW-1:0] SLOTS_V = SLOTS;
    localparam [CW-1:0] ZERO_C  = {CW{1'b0}};
    localparam [CW-1:0] ONE_C   = {{(CW-1){1'b0}}, 1'b1};
    localparam REQ_BITS = ADDR_WIDTH + 128 + 16;

    reg [CW-1:0]       wptr, dptr, fptr, cptr;
    reg [ID_WIDTH-1:0] q_id;                    // the one id in flight

    // Per-slot control (flops -- five bits, and s_wr is read at two
    // different pointers in the same cycle, which a RAM cannot do).
    reg                s_wr   [0:SLOTS-1];
    reg                s_need [0:SLOTS-1];
    reg                s_last [0:SLOTS-1];
    reg [1:0]          s_resp [0:SLOTS-1];

    // Per-slot payload (LUTRAM, flat 1-D, two independent writers -- see
    // the header for why these are two arrays and not one).
    (* ram_style = "distributed" *)
    reg [REQ_BITS-1:0] s_req  [0:SLOTS-1];
    (* ram_style = "distributed" *)
    reg [127:0]        s_rdat [0:SLOTS-1];

    wire [CW-1:0] used_c = wptr - cptr;         // accepted, not yet delivered
    wire [CW-1:0] out_c  = dptr - fptr;         // presented, response pending
    wire          full_c  = (used_c == SLOTS_V);
    wire          empty_c = (used_c == ZERO_C);
    wire          todo_c  = (wptr != dptr);     // a slot not yet presented
    wire          done_c  = (fptr != cptr);     // a response not yet delivered

    wire [SB-1:0] wslot_c = (SLOTS == 1) ? {SB{1'b0}} : wptr[SB-1:0];
    wire [SB-1:0] dslot_c = (SLOTS == 1) ? {SB{1'b0}} : dptr[SB-1:0];
    wire [SB-1:0] fslot_c = (SLOTS == 1) ? {SB{1'b0}} : fptr[SB-1:0];
    wire [SB-1:0] cslot_c = (SLOTS == 1) ? {SB{1'b0}} : cptr[SB-1:0];

    // ONE ID AT A TIME (see header): a request whose id differs from the
    // one already queued waits for the queue to drain.  This carries the
    // whole AXI response-ordering argument -- it makes every outstanding
    // transaction share an ARID/AWID, so the slave MUST answer them in
    // acceptance order and positional retirement is correct.
    assign req_ready = !full_c && (empty_c || (req_id == q_id));

    // -- Sequencer ---------------------------------------------------------
    // D_HDR is also the idle state: with nothing to present it simply
    // holds m_awvalid/m_arvalid low.  There is no response state at all --
    // a response is captured wherever the pointers say it belongs, which
    // is the whole point of this rewrite.
    // D_RSTW/D_RSTB are entered only from the reset branch below.
    localparam D_HDR = 2'd0, D_W = 2'd1, D_RSTW = 2'd2, D_RSTB = 2'd3;
    reg [1:0]    dst;
    reg [CW-1:0] bout_cnt;   // AW-accepted minus B-received -- survives rst
    reg [CW-1:0] bsink_cnt;  // of those, pre-reset strays owning no slot

    wire [REQ_BITS-1:0]   d_req_c   = s_req[dslot_c];
    wire [ADDR_WIDTH-1:0] d_addr_c  = d_req_c[REQ_BITS-1 -: ADDR_WIDTH];
    wire [127:0]          d_wdata_c = d_req_c[143:16];
    wire [15:0]           d_wstrb_c = d_req_c[15:0];
    wire                  d_wr_c    = s_wr[dslot_c];
    wire                  f_wr_c    = s_wr[fslot_c];

    // A direction change drains first -- AXI orders nothing between R and
    // B, so a read must never be in flight alongside a write.  Every
    // outstanding op therefore shares one direction, which is why
    // comparing against the OLDEST outstanding (fslot) is sufficient.
    wire dir_ok_c = (out_c == ZERO_C) || (d_wr_c == f_wr_c);
    // No new header while pre-reset strays are still being sunk: that is
    // what makes "every B arriving now is a stray" unambiguous.
    wire hdr_go_c = todo_c && dir_ok_c && (bsink_cnt == ZERO_C);

    // Every master-port output is gated by !rst UNIFORMLY, not just the
    // D_RSTW/D_RSTB terms -- on the cycle rst FIRST asserts, `dst` has not
    // transitioned yet (it is an NBA, taking effect only on the NEXT
    // edge), so an ungated (dst == D_W) would present one extra beat of
    // REAL data that nothing has counted.  l2c_victim.v hit exactly this
    // during bring-up; see its comment for the permanent hang it caused.
    // Gating unconditionally also makes "an AW cannot be accepted on the
    // cycle rst asserts" true, which the reset case analysis relies on.
    assign m_awid    = {1'b1, q_id[ID_WIDTH-2:0]};
    assign m_awaddr  = d_addr_c;
    assign m_awlen   = 8'd0;
    // ARSIZE/AWSIZE hardcoded to 4 (16B, the full 128b bus width) --
    // Minor-17: this is a full-line-width access regardless of the
    // ORIGINAL request's actual size, correct only under the precondition
    // that every bypass window is a plain memory carve-out (VRAM, a
    // future DDR framebuffer, etc.) where over-reading/over-writing
    // adjacent bytes within the same 16B-aligned quadrant has no side
    // effect (unlike a FIFO/status register that clears-on-read or
    // similar). WSTRB below still gates which bytes actually land on a
    // write, so writes are precise; reads return the full 16B regardless
    // and the requester picks out what it asked for, same convention as
    // the cache hit path. Document this precondition wherever a NEW
    // bypass window is configured (docs/l2c_spec.md S8/Important-14).
    assign m_awsize  = 3'd4;
    assign m_awburst = 2'd1;
    assign m_awvalid = !rst && (dst == D_HDR) && hdr_go_c && d_wr_c;
    assign m_wdata   = d_wdata_c;
    // The abandoned burst's filler beat enables no byte lane, so its data
    // is irrelevant and DRAM is not left waiting for beats forever.
    assign m_wstrb   = (dst == D_RSTW) ? 16'h0000 : d_wstrb_c;
    assign m_wlast   = 1'b1;                    // AWLEN=0: every beat is last
    assign m_wvalid  = !rst && ((dst == D_W) || (dst == D_RSTW));
    // Always ready for a BRESP while a write is outstanding -- there is no
    // state left to wait in.
    assign m_bready  = !rst && ((bsink_cnt != ZERO_C) ||
                                ((out_c != ZERO_C) && f_wr_c));

    assign m_arid    = {1'b1, q_id[ID_WIDTH-2:0]};
    assign m_araddr  = d_addr_c;
    assign m_arlen   = 8'd0;
    assign m_arsize  = 3'd4;
    assign m_arburst = 2'd1;
    assign m_arvalid = !rst && (dst == D_HDR) && hdr_go_c && !d_wr_c;
    // The capture slot is the op's OWN slot, allocated and not yet
    // retired, so there is never anything to wait for here -- and in
    // particular RREADY does not depend on the upstream RREADY, which
    // would add a combinational ready chain from s_axi straight to m_axi.
    assign m_rready  = !rst && (out_c != ZERO_C) && !f_wr_c;

    assign active_valid = !empty_c;
    assign active_id    = q_id;

    // A non-last write beat gets no B upstream (Critical-1); its slot is
    // retired silently below.
    wire c_need_c = s_need[cslot_c];
    assign rsp_valid    = done_c && c_need_c;
    assign rsp_is_write = s_wr[cslot_c];
    // Safe as a single register rather than a per-slot field: q_id may
    // only change while used_c == 0, i.e. when no accepted op is still
    // waiting to be answered.
    assign rsp_id       = q_id;
    assign rsp_rdata    = s_rdat[cslot_c];
    assign rsp_resp     = s_resp[cslot_c];
    assign rsp_last     = s_last[cslot_c];

    wire aw_fire_c  = m_awvalid && m_awready;
    wire ar_fire_c  = m_arvalid && m_arready;
    wire b_fire_c   = m_bvalid  && m_bready;
    wire r_fire_c   = m_rvalid  && m_rready;
    wire rsp_fire_c = done_c && (!c_need_c || rsp_ready);

    // -- Observability (sim taps) -----------------------------------------
    // Depth as a WIRE so a testbench tap does not have to hierarchically
    // reference a parameter (which not every tool resolves).
    /* verilator lint_off UNUSEDSIGNAL */
    // SIX bits, not five: at SLOTS=32 the occupancy value 32 does not fit
    // in five and a testbench tap reading it as 0 makes the
    // "inflight <= occupancy" self-check fire on a perfectly healthy
    // engine.  Found by the depth sweep at BYPASS_SLOTS=32.
    wire [7:0] slots_c;
    wire [5:0] occ_c;
    wire [5:0] inflight_c;
    assign slots_c    = SLOTS;
    assign occ_c      = used_c;
    assign inflight_c = out_c;
    /* verilator lint_on UNUSEDSIGNAL */

    integer i;
    always @(posedge clk) begin
        if (rst) begin
            // Decide, from the PRE-reset dst, whether DRAM is owed
            // anything before this engine goes idle.  A second reset
            // landing while still draining/sinking from an earlier one
            // must NOT restart or double-count -- bout_cnt/bsink_cnt
            // already reflect it.
            if (EXTERNAL_RESET_RECOVERY != 0) begin
                dst <= D_HDR; bout_cnt <= ZERO_C; bsink_cnt <= ZERO_C;
            end else begin
                case (dst)
                    D_W: begin
                        // One write is mid-flight with its W beat unsent;
                        // finish it with a filler, then sink what is owed.
                        dst <= D_RSTW; bsink_cnt <= bout_cnt;
                    end
                    D_HDR: begin
                        // Nothing mid-burst.  Anything already accepted
                        // still owes us a BRESP.  (An AW PRESENTED but
                        // not accepted promised DRAM nothing -- resetting
                        // clean there is what keeps l2c.v's arbiter from
                        // waiting forever for a B that is never coming.)
                        bsink_cnt <= bout_cnt;
                        dst <= (bout_cnt != ZERO_C) ? D_RSTB : D_HDR;
                    end
                    D_RSTW, D_RSTB: begin
                        // stay -- already draining/sinking, state intact.
                    end
                    default: begin
                        // Unreachable in 2-state; this is the power-up X
                        // case, the only place bout_cnt/bsink_cnt are ever
                        // initialised rather than carried.
                        dst <= D_HDR; bout_cnt <= ZERO_C; bsink_cnt <= ZERO_C;
                    end
                endcase
            end
            wptr <= ZERO_C; dptr <= ZERO_C; fptr <= ZERO_C; cptr <= ZERO_C;
            for (i = 0; i < SLOTS; i = i + 1) begin
                s_wr[i] <= 1'b0; s_need[i] <= 1'b0;
            end
        end else begin
            // -- Front-door accept ----------------------------------------
            if (req_valid && req_ready) begin
                if (empty_c) q_id <= req_id;
                s_wr  [wslot_c] <= req_is_write;
                // A read always answers; only writes carry need_resp.
                s_need[wslot_c] <= req_is_write ? req_need_resp : 1'b1;
                s_last[wslot_c] <= req_last;
                s_req [wslot_c] <= {req_addr, req_wdata, req_wstrb};
                wptr <= wptr + ONE_C;
            end

            // -- AR / AW+W sequencer --------------------------------------
            case (dst)
                D_HDR: begin
                    if (aw_fire_c)      dst  <= D_W;
                    else if (ar_fire_c) dptr <= dptr + ONE_C;
                end
                D_W: if (m_wready) begin
                    dptr <= dptr + ONE_C;
                    dst  <= D_HDR;
                end
                // Complete the abandoned single-beat burst, then sink the
                // BRESPs DRAM still owes.
                D_RSTW: if (m_wready) dst <= D_RSTB;
                // Exiting the sink state is keyed on bsink_cnt reaching
                // zero rather than on "this B was the last one", so it
                // cannot matter in which order this block and the B block
                // below happen to land on the same cycle.
                default: if (bsink_cnt == ZERO_C) dst <= D_HDR;   // D_RSTB
            endcase

            // -- Outstanding-write accounting (survives rst) ---------------
            case ({aw_fire_c, b_fire_c})
                2'b10:   bout_cnt <= bout_cnt + ONE_C;
                2'b01:   bout_cnt <= bout_cnt - ONE_C;
                default: bout_cnt <= bout_cnt;   // 00, or 11 = net zero
            endcase

            // -- Response capture -----------------------------------------
            if (b_fire_c) begin
                if (bsink_cnt != ZERO_C) begin
                    // A pre-reset stray: its slot no longer exists, so it
                    // frees nothing.  No new AW is presented while
                    // bsink_cnt != 0, so every B here is unambiguously one.
                    bsink_cnt <= bsink_cnt - ONE_C;
                end else begin
                    s_resp[fslot_c] <= m_bresp;
                    fptr <= fptr + ONE_C;
                end
            end
            if (r_fire_c) begin
                s_resp[fslot_c] <= m_rresp;
                s_rdat[fslot_c] <= m_rdata;
                fptr <= fptr + ONE_C;
            end

            // -- Retire ----------------------------------------------------
            if (rsp_fire_c) cptr <= cptr + ONE_C;
        end
    end

endmodule

`default_nettype wire
