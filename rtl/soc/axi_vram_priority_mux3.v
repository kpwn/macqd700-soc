// axi_vram_priority_mux3.v -- 3:1 (2 full R/W + 1 read-only) AXI4 priority
// arbiter for the DDR path (T16, "decode-level VRAM lane" reshape of T14's
// VRAM-in-DDR migration).
//
// Formerly `axi_ro_priority_mux2.v` (T14): a 2:1 READ-ONLY mux merging the
// CPU-path AR/R with scanout_ddr_reader's AR/R, with AW/W/B wired straight
// through around it. T14 routed VRAM-aperture CPU traffic through l2c's
// configured never-allocate bypass window to keep it out of the L2C tag
// array. T16 reshapes so that invariant is enforced structurally, at the
// DECODE LEVEL instead (axi_xbar.v's S3 slave decode, reverted to route
// the VRAM aperture to a genuine S3 slave port again, same as URAM mode) --
// l2c never sees VRAM-aperture traffic AT ALL any more (bypass window back
// to disabled defaults, docs/l2c_spec.md invariants unchanged, enforcement
// moved up a layer). This module is what widens to carry that S3 traffic
// around l2c instead of through it: renamed (role fundamentally changed --
// full R/W, 3 sources instead of RO 2 sources) and extended in place,
// keeping the reviewed grant-lock / no-valid-retraction / reset-hold
// idioms verbatim (see below for exactly what's reused vs. new).
//
// ── Three sources ──────────────────────────────────────────────────────
// - `l2c_*`  -- the L2 cache's own master port (or the raw xbar S0
//   passthrough when L2C_ENABLE is undefined). Full R/W. This is ALL
//   RAM/ROM/legacy-FB traffic plus anything else that reaches xbar S0 --
//   never VRAM-aperture traffic any more (that's the whole point of the
//   decode-level move: L2C_ENABLE's tag array is never polluted by VRAM
//   writes because VRAM traffic never reaches xbar S0/l2c's slave port).
// - `s3_*`   -- the VRAM-aperture CPU lane: xbar's S3 slave port's AW/W/B
//   and AR/R, with the byte-swap (already applied, unconditionally, at
//   the xbar S3 boundary -- vram_swap_words/vram_swap_strb, unchanged
//   T14 placement/semantics) and address-translate (aperture-zero-based
//   offset -> DDR carveout absolute address, `+CARVEOUT_BASE`, applied by
//   this module's caller -- see fpga_top_video.vh's S3-backend comment)
//   already done upstream of this port. Full R/W.
// - `scan_*` -- scanout_ddr_reader's real-time HDMI-scanout consumer.
//   Read-only (scanout never writes). Bounded high priority -- see below.
//
// ── Priority contract ─────────────────────────────────────────────────
// AR/R (3-way): `scan_*` normally wins when the channel is idle.  After
// `MAX_SCAN_AHEAD` scan bursts have been accepted while either non-scan
// source is continuously pending, one non-scan burst is forced.  Four scan
// bursts is the conservative default: it admits 512 bytes of scanout data
// between forced bulk grants while bounding service for CPU/L2 and S3 VRAM
// reads.  It also matches the scanout reader's own MAX_OUTSTANDING, so a
// scanout REFILL EXCURSION (scanout_line_fetch.v batches its refills
// deliberately -- fewer, deeper, rarer DDR visits rather than a constant
// low-level tax) is never chopped in half by the fairness quota.
//
// `l2c_*` and `s3_*` round-robin (`ar_rr`, flips on every non-scan
// grant -- the CPU-path VRAM
// writer is single-outstanding by construction (68040 LSU has one
// outstanding op), so it cannot flood this channel, but sustained l2c
// eviction/fill traffic must not starve it either -- hence RR, not fixed
// priority, matching l2c.v's own documented `rw_favor` precedent,
// docs/l2c_spec.md S4).
//
// AW/W/B (2-way, l2c_* vs s3_*, scanout never writes): plain round-robin
// (`aw_rr`, flips on every grant) -- same starvation-avoidance rationale.
//
// Because `scan_*` never contends for AW/W/B and normally wins each new AR
// grant, scanout is admitted promptly ahead of newly-presented CPU traffic.
// A forced bulk grant can occur only after the documented scan quota. Reads
// already accepted downstream retain AXI ordering and are not preempted.
// The non-scan admission cap therefore IS the fabric-added queueing bound
// ahead of a newly arriving scan request.  That cap is ADAPTIVE:
//
//   scanout ACTIVE (an AR pending, or bursts of its own still in the route
//                   FIFO)  ->  `MAX_BULK_AHEAD`
//   scanout QUIET          ->  `MAX_BULK_AHEAD_QUIET`
//
// The point of the split is that scanout's DDR usage is now SPORADIC by
// construction (scanout_line_fetch.v refills in batched excursions and is
// idle in between), and there is no reason for the CPU to be paying
// scanout's reservation during the gaps -- which is most of the time.  The
// SUSTAINED bound a scan request sees is `MAX_BULK_AHEAD`; the quiet cap
// only ever adds a ONE-OFF convoy at the start of an excursion, bounded by
// `MAX_BULK_AHEAD_QUIET` bursts, which is what the scanout reader's
// multi-line lookahead window exists to absorb.
//
// Once an AR is PRESENTED downstream its admission is locked
// (`ar_hold_valid` feeds `ar_src_admit_c`): scanout going from quiet to
// active must never retract an already-asserted `m_arvalid`, which AXI
// forbids.  That term is load-bearing, not defensive -- without it, a
// scan request arriving one cycle after a bulk AR was presented into
// READY-low would drop `m_arvalid` mid-handshake.
//
// None of this can by itself bound a downstream slave that stops making
// progress; AXI has no maximum READY/response latency. A non-scan AR
// already presented while READY is low also remains stable until accepted,
// as AXI requires.
//
// ── AXI presentation, routing, and reset rules ─────────────────────────
// - GRANT LOCK / no-valid-retraction (Critical-4/5 class, docs/
//   l2c_spec.md S9): an AR source presented into READY-low is held through
//   its handshake; AW/W/B retains its source through B. Thus `m_*valid` and
//   payload never swap mid-presentation if a higher-priority source rises.
// - The AW/W/B side retains the reviewed single-outstanding lock. The
//   AR/R side now uses an 8-entry accepted-request route FIFO, matching
//   the downstream MIG bridge's default read capacity. No ID widening is
//   required because that bridge returns repo-side bursts in accepted-AR
//   order; the FIFO records only which upstream source owns each burst.
// - Reset clears the AR route FIFO; the production async bridge's stale
//   sink owns and discards all pre-reset read-response debt downstream.
//   AW/W/B retains T14's `aw_open` reset-hold semantics because an accepted
//   write burst may still need its W beats completed locally.
// - BOUNDED RESET-HOLD SAFETY NET (127-cycle cap, `rst_hold_ctr`): per
//   the T14 module's own M2 finding, in the real chain a core-only reset
//   leaves `axi_bridge_stale_sink.v` (downstream, inside
//   axi_async_bridge.v) sinking an abandoned burst's remaining beats
//   ENTIRELY on its own m-side -- including the terminal RLAST/B -- so
//   an upstream reset-held grant would otherwise wait forever for a
//   terminal event that structurally cannot arrive. The AW/W/B side's
//   127-cycle timeout is therefore the expected normal exit path: an
//   abandoned AW/W burst's B is just as thoroughly sunk by the B-channel half of
//   `axi_bridge_stale_sink.v` (round-3 T14 fix, "B-channel got the same
//   treatment").
`default_nettype none

module axi_vram_priority_mux3 #(
    parameter ID_WIDTH   = 6,
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 128,
    // Downstream returns reads in accepted-AR order, so this is also the
    // hard maximum number of non-scan bursts that can be queued ahead of a
    // newly-arriving scan request -- and, because it caps `bulk_count`, it
    // is ALSO the ceiling on how many CPU/L2 miss fills can be in flight on
    // the DDR path at once.  Those two facts are the same knob, which is
    // why it matters so much.
    //
    // ── 2026-08-19: 2 -> 4, and WHY it could not be raised before ──────
    // The old value was 2, documented as "preserves useful L2/MIG overlap
    // without allowing the eight-entry cache fill window to consume all of
    // scanout's buffering slack".  That was correct FOR THE SCANOUT READER
    // OF THE TIME: scanout_line_fetch.v ran ONE AXI burst in flight with
    // ONE line of prefetch, so a 128 B line cost a whole DDR round trip end
    // to end (~48 cycles at the assumed 40-cycle round trip) and scanout's
    // entire latency tolerance was that single line.  Anything queued ahead
    // of it came straight off the deadline.  The consequence was that the
    // DISPLAY bounded the CPU's memory system: L2 measures ~8.4 cyc/op at 8
    // outstanding fills and was pinned near 28 cyc/op by this cap.
    //
    // scanout_line_fetch.v now runs MAX_OUTSTANDING (4) bursts against a
    // PREFETCH_DEPTH (5) line lookahead window, so its fill rate is set by
    // its share of the R channel rather than by the round trip, and its
    // tolerance is 5 lines rather than 1.  That is what pays for this.
    //
    // ── What bounds it, and why 4 and not 8 ───────────────────────────
    // The AR route FIFO is RDQ_DEPTH (8) deep -- matched to the downstream
    // bridge's RMAX_OUTSTANDING (docs/ddr4_mig_bridge_contract.md), so
    // there is nothing to gain from making it deeper.  `bulk_admit` gates
    // only NON-scan sources, so RDQ_DEPTH - MAX_BULK_AHEAD is a hard
    // RESERVATION of route-FIFO slots for scanout, and it is the only
    // structural guarantee scanout has that a CPU miss storm cannot squeeze
    // its AXI pipeline shut.  Set the reservation below the scanout
    // engine's own MAX_OUTSTANDING and scanout silently loses pipelining
    // exactly when the CPU is busiest -- i.e. it goes back to being
    // round-trip-bound, which is the failure this whole rework removes,
    // and it does so in a way that only shows up as a VISIBLE display
    // artifact under load.  So:
    //
    //     MAX_BULK_AHEAD <= RDQ_DEPTH - SCAN_RESERVE
    //
    // with SCAN_RESERVE = the scanout reader's MAX_OUTSTANDING.  4 is that
    // bound at today's 8/4.  Raising it further is a real option ONLY
    // together with a deeper route FIFO *and* a downstream bridge that
    // accepts more than 8 outstanding reads -- not on its own.
    //
    // Measured (tb-scanout-ddr-frames, mode-transition scenario: tightest
    // mode + live placement change + saturating CPU miss storm; see that
    // file for the DDR round-trip assumption, which is 40 +/- 8 core clocks
    // and is NOT a hardware measurement).  Whole-run totals, "A again" leg:
    //
    //   engine  cap  DDRlat   bursts/excursion  starved cyc  bulk cyc/op
    //   old      2   40+/-8     1.00 (peak 1)     1,383,977     17.65   pass
    //   old      6   40+/-8     1.00 (peak 1)     1,390,010      5.95   pass
    //   old      2   200+/-64   1.00 (peak 1)    10,430,116     65.30   FAIL
    //   new      2   40+/-8     2.00 (peak 4)           603     17.05   pass
    //   new      4   40+/-8     2.00 (peak 4)           607      8.65   pass
    //   new      6   40+/-8     2.00 (peak 4)           619      6.02   pass
    //   new      4   200+/-64  14.20 (peak 4)       437,876     32.74   pass
    //   new      6   200+/-64  14.30 (peak 4)     1,288,595     23.95   pass
    //
    // Read the 200+/-64 rows first.  The OLD engine at the OLD cap of 2
    // does not merely get slow there, it fails outright (87% of pixels
    // wrong, line_underflow_sticky set) -- i.e. its correctness depended
    // entirely on a DDR round-trip figure nobody has measured.  The new
    // engine passes at 200+/-64 even at cap 6.  Raising the cap is the
    // smaller half of this change; making the deadline insensitive to the
    // round trip is the larger half.
    //
    // 6 measures clean too, and is 1.4x better again for the CPU.  It is
    // not the default because it spends the scan reservation: at 200+/-64
    // its starvation is 3x cap-4's, i.e. it is trading exactly the margin
    // that the unmeasured latency figure makes uncertain.  Cap 4 is the
    // value whose safety argument is structural (see SCAN_RESERVE) rather
    // than empirical.
    parameter MAX_BULK_AHEAD = 4,
    // Non-scan admission cap while scanout is QUIET -- no AR pending and no
    // bursts of its own in the route FIFO.  Defaults to the whole route
    // FIFO: when the display is not asking for anything there is nothing to
    // reserve slots for, and the CPU should get the entire read pipeline.
    // Set equal to MAX_BULK_AHEAD to disable the adaptive behaviour and
    // restore a flat cap.
    parameter MAX_BULK_AHEAD_QUIET = 8,
    // Route-FIFO slots reserved for scanout.  Not used in the datapath --
    // it exists so the relationship above is checked at elaboration instead
    // of living only in a comment.  Keep it equal to the scanout reader's
    // MAX_OUTSTANDING.
    parameter SCAN_RESERVE = 4,
    // Maximum consecutive scan bursts accepted while l2c or s3 is pending.
    // The bound is in accepted bursts, not clocks; AXI cannot bound a slave
    // which stops asserting READY or returning responses.
    parameter MAX_SCAN_AHEAD = 4,
    // The production async bridge completes writes abandoned by a
    // source-side reset; clear local ownership in that composition.
    parameter EXTERNAL_WRITE_RESET_RECOVERY = 0
) (
    input  wire                    clk,
    input  wire                    rst,

    // ── l2c-path side (l2c.m_axi, or xbar S0 passthrough) -- full R/W ──
    input  wire [ID_WIDTH-1:0]     l2c_awid,
    input  wire [ADDR_WIDTH-1:0]   l2c_awaddr,
    input  wire [7:0]              l2c_awlen,
    input  wire [2:0]              l2c_awsize,
    input  wire [1:0]              l2c_awburst,
    input  wire                    l2c_awvalid,
    output wire                    l2c_awready,
    input  wire [DATA_WIDTH-1:0]   l2c_wdata,
    input  wire [DATA_WIDTH/8-1:0] l2c_wstrb,
    input  wire                    l2c_wlast,
    input  wire                    l2c_wvalid,
    output wire                    l2c_wready,
    output wire [ID_WIDTH-1:0]     l2c_bid,
    output wire [1:0]              l2c_bresp,
    output wire                    l2c_bvalid,
    input  wire                    l2c_bready,
    input  wire [ID_WIDTH-1:0]     l2c_arid,
    input  wire [ADDR_WIDTH-1:0]   l2c_araddr,
    input  wire [7:0]              l2c_arlen,
    input  wire [2:0]              l2c_arsize,
    input  wire [1:0]              l2c_arburst,
    input  wire                    l2c_arvalid,
    output wire                    l2c_arready,
    output wire [ID_WIDTH-1:0]     l2c_rid,
    output wire [DATA_WIDTH-1:0]   l2c_rdata,
    output wire [1:0]              l2c_rresp,
    output wire                    l2c_rlast,
    output wire                    l2c_rvalid,
    input  wire                    l2c_rready,

    // ── s3 (VRAM-aperture CPU) lane -- already address-translated to the
    //    carveout + byte-swapped upstream (xbar S3 boundary) -- full R/W ──
    input  wire [ID_WIDTH-1:0]     s3_awid,
    input  wire [ADDR_WIDTH-1:0]   s3_awaddr,
    input  wire [7:0]              s3_awlen,
    input  wire [2:0]              s3_awsize,
    input  wire [1:0]              s3_awburst,
    input  wire                    s3_awvalid,
    output wire                    s3_awready,
    input  wire [DATA_WIDTH-1:0]   s3_wdata,
    input  wire [DATA_WIDTH/8-1:0] s3_wstrb,
    input  wire                    s3_wlast,
    input  wire                    s3_wvalid,
    output wire                    s3_wready,
    output wire [ID_WIDTH-1:0]     s3_bid,
    output wire [1:0]              s3_bresp,
    output wire                    s3_bvalid,
    input  wire                    s3_bready,
    input  wire [ID_WIDTH-1:0]     s3_arid,
    input  wire [ADDR_WIDTH-1:0]   s3_araddr,
    input  wire [7:0]              s3_arlen,
    input  wire [2:0]              s3_arsize,
    input  wire [1:0]              s3_arburst,
    input  wire                    s3_arvalid,
    output wire                    s3_arready,
    output wire [ID_WIDTH-1:0]     s3_rid,
    output wire [DATA_WIDTH-1:0]   s3_rdata,
    output wire [1:0]              s3_rresp,
    output wire                    s3_rlast,
    output wire                    s3_rvalid,
    input  wire                    s3_rready,

    // ── scanout side (scanout_ddr_reader.v) -- read-only, strict top
    //    priority on AR/R ─────────────────────────────────────────────
    input  wire [ID_WIDTH-1:0]     scan_arid,
    input  wire [ADDR_WIDTH-1:0]   scan_araddr,
    input  wire [7:0]              scan_arlen,
    input  wire [2:0]              scan_arsize,
    input  wire [1:0]              scan_arburst,
    input  wire                    scan_arvalid,
    output wire                    scan_arready,
    output wire [ID_WIDTH-1:0]     scan_rid,
    output wire [DATA_WIDTH-1:0]   scan_rdata,
    output wire [1:0]              scan_rresp,
    output wire                    scan_rlast,
    output wire                    scan_rvalid,
    input  wire                    scan_rready,

    // ── merged master port -> u_ddr's slave AW/W/B/AR/R ────────────────
    output wire [ID_WIDTH-1:0]     m_awid,
    output wire [ADDR_WIDTH-1:0]   m_awaddr,
    output wire [7:0]              m_awlen,
    output wire [2:0]              m_awsize,
    output wire [1:0]              m_awburst,
    output wire                    m_awvalid,
    input  wire                    m_awready,
    output wire [DATA_WIDTH-1:0]   m_wdata,
    output wire [DATA_WIDTH/8-1:0] m_wstrb,
    output wire                    m_wlast,
    output wire                    m_wvalid,
    input  wire                    m_wready,
    input  wire [ID_WIDTH-1:0]     m_bid,
    input  wire [1:0]              m_bresp,
    input  wire                    m_bvalid,
    output wire                    m_bready,
    output wire [ID_WIDTH-1:0]     m_arid,
    output wire [ADDR_WIDTH-1:0]   m_araddr,
    output wire [7:0]              m_arlen,
    output wire [2:0]              m_arsize,
    output wire [1:0]              m_arburst,
    output wire                    m_arvalid,
    input  wire                    m_arready,
    input  wire [ID_WIDTH-1:0]     m_rid,
    input  wire [DATA_WIDTH-1:0]   m_rdata,
    input  wire [1:0]              m_rresp,
    input  wire                    m_rlast,
    input  wire                    m_rvalid,
    output wire                    m_rready
);

    localparam AR_L2C = 2'd0, AR_S3 = 2'd1, AR_SCAN = 2'd2;

    // ═══════════════════════════════════════════════════════════════════
    // AR/R channel: 3-way (l2c, s3, scan), scan strict-top, else RR
    // ═══════════════════════════════════════════════════════════════════
    localparam RDQ_DEPTH = 8;
    reg [1:0] rdq_src [0:RDQ_DEPTH-1];
    reg [2:0] rdq_wptr, rdq_rptr;
    reg [3:0] rdq_count;
    reg [3:0] bulk_count;
    reg       ar_rr;                 // RR favor bit between l2c/s3 only
    reg       ar_hold_valid;
    reg [1:0] ar_hold_src;
    reg [7:0] scan_ahead_count;

    wire rdq_full = (rdq_count == 4'd8);
    wire rdq_empty = (rdq_count == 4'd0);
    // Scan bursts currently sitting in the route FIFO.  `bulk_count` is the
    // non-scan half of `rdq_count` by construction, so the difference is
    // exactly the scan half -- no second counter needed.
    wire [3:0] scan_in_rdq_c = rdq_count - bulk_count;
    wire scan_quiet_c = !scan_arvalid && (scan_in_rdq_c == 4'd0);
    wire [3:0] bulk_cap_c = scan_quiet_c ? MAX_BULK_AHEAD_QUIET[3:0]
                                         : MAX_BULK_AHEAD[3:0];
    wire bulk_admit = (bulk_count < bulk_cap_c);
    wire bulk_pending_c = l2c_arvalid || s3_arvalid;
    wire [1:0] bulk_pick_c =
                            (l2c_arvalid && s3_arvalid) ? (ar_rr ? AR_S3 : AR_L2C) :
                            l2c_arvalid ? AR_L2C : AR_S3;
    // Once the quota expires, stop accepting further scan bursts until a
    // bulk slot is available and one pending bulk request is admitted.  This
    // makes MAX_SCAN_AHEAD a hard accepted-burst bound even while the older
    // bulk requests are still waiting for responses.
    wire force_bulk_c = scan_arvalid && bulk_pending_c &&
                        (scan_ahead_count >= MAX_SCAN_AHEAD);
    wire [1:0] ar_pick_c = (scan_arvalid && !force_bulk_c) ? AR_SCAN :
                           bulk_pending_c ? bulk_pick_c : AR_L2C;
    wire [1:0] ar_src_c = ar_hold_valid ? ar_hold_src : ar_pick_c;
    wire ar_src_valid_c = (ar_src_c == AR_SCAN) ? scan_arvalid :
                          (ar_src_c == AR_S3) ? s3_arvalid : l2c_arvalid;

    assign m_arid    = (ar_src_c == AR_SCAN) ? scan_arid    : (ar_src_c == AR_S3) ? s3_arid    : l2c_arid;
    assign m_araddr  = (ar_src_c == AR_SCAN) ? scan_araddr  : (ar_src_c == AR_S3) ? s3_araddr  : l2c_araddr;
    assign m_arlen   = (ar_src_c == AR_SCAN) ? scan_arlen   : (ar_src_c == AR_S3) ? s3_arlen   : l2c_arlen;
    assign m_arsize  = (ar_src_c == AR_SCAN) ? scan_arsize  : (ar_src_c == AR_S3) ? s3_arsize  : l2c_arsize;
    assign m_arburst = (ar_src_c == AR_SCAN) ? scan_arburst : (ar_src_c == AR_S3) ? s3_arburst : l2c_arburst;
    // `ar_hold_valid`: an AR already PRESENTED downstream keeps its
    // admission.  With a static cap this term was unnecessary (bulk_admit
    // could only fall on an AR accept, and the presented AR is the only one
    // that can be accepted).  With the adaptive cap it is REQUIRED: scanout
    // going from quiet to active would otherwise retract m_arvalid
    // mid-handshake, which AXI forbids.
    wire ar_src_admit_c = (ar_src_c == AR_SCAN) || bulk_admit || ar_hold_valid;
    assign m_arvalid = ar_src_valid_c && ar_src_admit_c && !rdq_full;

    assign scan_arready = (ar_src_c == AR_SCAN) && ar_src_admit_c && !rdq_full && m_arready;
    assign s3_arready   = (ar_src_c == AR_S3)   && ar_src_admit_c && !rdq_full && m_arready;
    assign l2c_arready  = (ar_src_c == AR_L2C)  && ar_src_admit_c && !rdq_full && m_arready;
    wire rdq_push = m_arvalid && m_arready;

    wire [1:0] rdq_head_src = rdq_src[rdq_rptr];
    assign m_rready = !rdq_empty && ((rdq_head_src == AR_SCAN) ? scan_rready :
                                     (rdq_head_src == AR_S3) ? s3_rready : l2c_rready);

    assign scan_rvalid = !rdq_empty && (rdq_head_src == AR_SCAN) && m_rvalid;
    assign scan_rid    = m_rid; assign scan_rdata = m_rdata; assign scan_rresp = m_rresp; assign scan_rlast = m_rlast;

    assign s3_rvalid = !rdq_empty && (rdq_head_src == AR_S3) && m_rvalid;
    assign s3_rid    = m_rid; assign s3_rdata = m_rdata; assign s3_rresp = m_rresp; assign s3_rlast = m_rlast;

    assign l2c_rvalid = !rdq_empty && (rdq_head_src == AR_L2C) && m_rvalid;
    assign l2c_rid    = m_rid; assign l2c_rdata = m_rdata; assign l2c_rresp = m_rresp; assign l2c_rlast = m_rlast;
    wire rdq_pop = m_rvalid && m_rready && m_rlast;

    // synthesis translate_off
    initial begin
        if (MAX_BULK_AHEAD < 1 || MAX_BULK_AHEAD > RDQ_DEPTH) begin
            $display("axi_vram_priority_mux3: MAX_BULK_AHEAD must be in [1,%0d]", RDQ_DEPTH);
            $fatal(1);
        end
        if (MAX_BULK_AHEAD_QUIET < MAX_BULK_AHEAD || MAX_BULK_AHEAD_QUIET > RDQ_DEPTH) begin
            $display("axi_vram_priority_mux3: MAX_BULK_AHEAD_QUIET must be in [MAX_BULK_AHEAD,%0d]", RDQ_DEPTH);
            $fatal(1);
        end
        if ((MAX_BULK_AHEAD + SCAN_RESERVE) > RDQ_DEPTH) begin
            $display("axi_vram_priority_mux3: MAX_BULK_AHEAD (%0d) + SCAN_RESERVE (%0d) exceeds RDQ_DEPTH (%0d) -- a CPU miss storm could squeeze scanout's AXI pipeline shut and the failure would appear as a display artifact, not a test failure",
                     MAX_BULK_AHEAD, SCAN_RESERVE, RDQ_DEPTH);
            $fatal(1);
        end
        if (MAX_SCAN_AHEAD < 1 || MAX_SCAN_AHEAD > 255) begin
            $display("axi_vram_priority_mux3: MAX_SCAN_AHEAD must be in [1,255]");
            $fatal(1);
        end
    end
    always @(posedge clk) begin
        if (!rst && bulk_count > MAX_BULK_AHEAD_QUIET) begin
            $display("axi_vram_priority_mux3: bulk admission bound exceeded (%0d > %0d)",
                     bulk_count, MAX_BULK_AHEAD_QUIET);
            $fatal(1);
        end
        // The SUSTAINED bound: while scanout has anything of its own in the
        // route FIFO, no further bulk burst may be admitted past
        // MAX_BULK_AHEAD.  (bulk_count itself can still be above it, left
        // over from a quiet-period convoy -- that is the documented one-off,
        // and it drains without new admissions.)
        if (!rst && !scan_quiet_c && bulk_admit && (bulk_count >= MAX_BULK_AHEAD)) begin
            $display("axi_vram_priority_mux3: bulk admitted past MAX_BULK_AHEAD while scanout is active (%0d >= %0d)",
                     bulk_count, MAX_BULK_AHEAD);
            $fatal(1);
        end
        if (!rst && scan_ahead_count > MAX_SCAN_AHEAD) begin
            $display("axi_vram_priority_mux3: scan fairness quota exceeded (%0d > %0d)",
                     scan_ahead_count, MAX_SCAN_AHEAD);
            $fatal(1);
        end
    end
    // synthesis translate_on

    always @(posedge clk) begin
        if (rst) begin
            rdq_wptr <= 3'd0; rdq_rptr <= 3'd0; rdq_count <= 4'd0;
            bulk_count <= 4'd0;
            ar_rr <= 1'b0; ar_hold_valid <= 1'b0; ar_hold_src <= AR_L2C;
            scan_ahead_count <= 8'd0;
        end else begin
            // Lock only traffic actually presented downstream.  A bulk
            // source blocked by MAX_BULK_AHEAD has m_arvalid=0 and must
            // remain preemptible by a later scanout request.
            if (!ar_hold_valid && m_arvalid && !m_arready) begin
                ar_hold_valid <= 1'b1;
                ar_hold_src <= ar_pick_c;
            end else if (rdq_push) begin
                ar_hold_valid <= 1'b0;
            end
            if (rdq_push) begin
                rdq_src[rdq_wptr] <= ar_src_c;
                rdq_wptr <= rdq_wptr + 3'd1;
                if (ar_src_c != AR_SCAN) ar_rr <= !ar_rr;
            end
            if (!bulk_pending_c) begin
                scan_ahead_count <= 8'd0;
            end else if (rdq_push) begin
                if (ar_src_c == AR_SCAN) begin
                    if (scan_ahead_count < MAX_SCAN_AHEAD)
                        scan_ahead_count <= scan_ahead_count + 8'd1;
                end else begin
                    scan_ahead_count <= 8'd0;
                end
            end
            if (rdq_pop) rdq_rptr <= rdq_rptr + 3'd1;
            case ({rdq_push, rdq_pop})
                2'b10: rdq_count <= rdq_count + 4'd1;
                2'b01: rdq_count <= rdq_count - 4'd1;
                default: rdq_count <= rdq_count;
            endcase
            case ({rdq_push && (ar_src_c != AR_SCAN),
                   rdq_pop && (rdq_head_src != AR_SCAN)})
                2'b10: bulk_count <= bulk_count + 4'd1;
                2'b01: bulk_count <= bulk_count - 4'd1;
                default: bulk_count <= bulk_count;
            endcase
        end
    end

    // ═══════════════════════════════════════════════════════════════════
    // AW/W/B channel: 2-way (l2c, s3) plain RR -- scanout never writes.
    // ═══════════════════════════════════════════════════════════════════
    reg       aw_grant;              // 0 = l2c, 1 = s3
    reg       aw_busy;
    reg       aw_rr;
    reg       aw_held_through_reset;
    reg [7:0] aw_rst_hold_ctr;
    reg       aw_open;               // true iff AW genuinely accepted downstream, B not yet seen

    wire aw_pick_c = (l2c_awvalid && s3_awvalid) ? aw_rr :
                      l2c_awvalid ? 1'b0 :
                      s3_awvalid  ? 1'b1 : 1'b0;
    wire aw_grant_live_c = aw_busy ? aw_grant : aw_pick_c;
    wire aw_accept_now_c = m_awvalid && m_awready;

    always @(posedge clk) begin
        if (rst) begin
            aw_rr <= 1'b0;
            if ((EXTERNAL_WRITE_RESET_RECOVERY == 0) &&
                (aw_open || aw_accept_now_c)) begin
                aw_open <= 1'b1;
                if (aw_accept_now_c && !aw_open) begin
                    aw_grant <= aw_grant_live_c;
                    aw_busy <= 1'b1;
                end
                if (!aw_held_through_reset) begin
                    aw_held_through_reset <= 1'b1;
                    aw_rst_hold_ctr <= 8'd0;
                end
            end else begin
                aw_grant <= 1'b0;
                aw_busy  <= 1'b0;
                aw_held_through_reset <= 1'b0;
                aw_rst_hold_ctr <= 8'd0;
                aw_open  <= 1'b0;
            end
        end else begin
            if (aw_accept_now_c && !aw_open) aw_open <= 1'b1;
            if (aw_held_through_reset) begin
                if (m_bvalid && m_bready) begin
                    aw_busy <= 1'b0;
                    aw_held_through_reset <= 1'b0;
                    aw_open <= 1'b0;
                end else if (aw_rst_hold_ctr >= 8'd127) begin
                    aw_grant <= 1'b0;
                    aw_busy  <= 1'b0;
                    aw_held_through_reset <= 1'b0;
                    aw_open  <= 1'b0;
                end else begin
                    aw_rst_hold_ctr <= aw_rst_hold_ctr + 8'd1;
                end
            end else if (!aw_busy) begin
                if (l2c_awvalid || s3_awvalid) begin
                    aw_grant <= aw_grant_live_c;
                    aw_busy  <= 1'b1;
                    aw_rr    <= !aw_rr;
                end
            end else if (m_bvalid && m_bready) begin
                aw_busy <= 1'b0;
                aw_open <= 1'b0;
            end
        end
    end

    assign m_awid    = aw_grant_live_c ? s3_awid    : l2c_awid;
    assign m_awaddr  = aw_grant_live_c ? s3_awaddr  : l2c_awaddr;
    assign m_awlen   = aw_grant_live_c ? s3_awlen   : l2c_awlen;
    assign m_awsize  = aw_grant_live_c ? s3_awsize  : l2c_awsize;
    assign m_awburst = aw_grant_live_c ? s3_awburst : l2c_awburst;
    assign m_awvalid = !aw_open && (aw_grant_live_c ? s3_awvalid : l2c_awvalid);

    assign s3_awready  = !aw_open &&  aw_grant_live_c && m_awready;
    assign l2c_awready = !aw_open && !aw_grant_live_c && m_awready;

    assign m_wdata  = aw_grant_live_c ? s3_wdata  : l2c_wdata;
    assign m_wstrb  = aw_grant_live_c ? s3_wstrb  : l2c_wstrb;
    assign m_wlast  = aw_grant_live_c ? s3_wlast  : l2c_wlast;
    // Deliberately impose AW-before-W.  Until an AW handshake establishes
    // both the owner and downstream write context, no W source is visible.
    assign m_wvalid = aw_busy && aw_open &&
                      (aw_grant ? s3_wvalid : l2c_wvalid);

    assign s3_wready  = aw_busy && aw_open &&  aw_grant && m_wready;
    assign l2c_wready = aw_busy && aw_open && !aw_grant && m_wready;

    assign s3_bvalid  = aw_busy && aw_open &&  aw_grant && m_bvalid;
    assign s3_bid     = m_bid; assign s3_bresp = m_bresp;

    assign l2c_bvalid = aw_busy && aw_open && !aw_grant && m_bvalid;
    assign l2c_bid    = m_bid; assign l2c_bresp = m_bresp;

    assign m_bready = aw_busy && aw_open &&
                      (aw_grant ? s3_bready : l2c_bready);

`ifdef VERILATOR
    // Payload-stability check: once a channel is granted-and-presented,
    // its payload must not change until accepted (mirrors T14's own
    // check + l2c_spec.md's axi_payload_stability_check, S9/S10).
    reg                  dbg_prev_arvalid;
    reg [ADDR_WIDTH-1:0] dbg_prev_araddr;
    reg [ID_WIDTH-1:0]   dbg_prev_arid;
    reg                  dbg_prev_awvalid;
    reg [ADDR_WIDTH-1:0] dbg_prev_awaddr;
    reg [ID_WIDTH-1:0]   dbg_prev_awid;
    always @(posedge clk) begin
        if (rst) begin
            dbg_prev_arvalid <= 1'b0;
            dbg_prev_awvalid <= 1'b0;
        end else begin
            if (dbg_prev_arvalid && !m_arready) begin
                if (m_araddr !== dbg_prev_araddr || m_arid !== dbg_prev_arid) begin
                    $display("axi_vram_priority_mux3: AR payload changed while VALID-not-READY (addr %08x -> %08x)",
                              dbg_prev_araddr, m_araddr);
                    $fatal(1);
                end
            end
            dbg_prev_arvalid <= m_arvalid && !m_arready;
            dbg_prev_araddr  <= m_araddr;
            dbg_prev_arid    <= m_arid;
            if (dbg_prev_awvalid && !m_awready) begin
                if (m_awaddr !== dbg_prev_awaddr || m_awid !== dbg_prev_awid) begin
                    $display("axi_vram_priority_mux3: AW payload changed while VALID-not-READY (addr %08x -> %08x)",
                              dbg_prev_awaddr, m_awaddr);
                    $fatal(1);
                end
            end
            dbg_prev_awvalid <= m_awvalid && !m_awready;
            dbg_prev_awaddr  <= m_awaddr;
            dbg_prev_awid    <= m_awid;
            if (m_wvalid && (!aw_busy || !aw_open)) begin
                $display("axi_vram_priority_mux3: W presented without an accepted AW owner");
                $fatal(1);
            end
        end
    end
`endif

endmodule

`default_nettype wire
