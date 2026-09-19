// vhdd_readahead.v — sequential read-ahead cache for the virtual-HDD
// contract (rtl/vhdd.vh).
//
// WHY THIS EXISTS — the measured 280x
// ═══════════════════════════════════
// Measured on the 200 MHz bitstream (2026-09-15), booting Mac OS off the
// SD-backed SCSI volume:
//
//   SCSI volume throughput .............  13.6 blocks/s  (~70 ms / block)
//   boot_fsm, 2048 sectors, ONE CMD18 ... ~3800 sectors/s
//
// Same card, same sd_ctrl, same 50 MHz SCK.  A 512-byte block is only
// ~82 us of wire time, and boot_fsm proves the transport sustains
// ~263 us/sector end to end.  So ~69.7 ms of every 70 ms block is
// PER-COMMAND latency — R1 poll plus data-token wait — and boot_fsm is
// fast for exactly one reason: it pays that latency ONCE for 2048
// sectors instead of once per sector.
//
// The Mac's SCSI driver asks for one or two blocks at a time, so
// scsi.v's (correct) CMD17/CMD18 choice cannot amortise anything.  This
// module amortises it instead: on a miss it fetches a RUN of sequential
// blocks with one CMD18 and serves the following requests out of BRAM.
// Mac OS boot reads are overwhelmingly sequential, which is the whole
// premise — and it is the same premise boot_fsm already validates on
// this card.
//
//   scsi.v ──vhdd──► vhdd_mux ──vhdd──► vhdd_readahead ──vhdd──► vhdd_sd
//                                       (this)                   (SD)
//
// It implements the vhdd contract on BOTH faces, so neither neighbour
// knows it is there.  It is NOT a provider: it has no idea what a block
// is stored on.
//
// SHAPE
// ═════
// Two ways, BLOCKS_PER_WAY blocks each, byte-addressed BRAM.  Two ways
// and not one because HFS boot traffic interleaves two sequential
// streams — B-tree nodes and file data — and a single window thrashes
// between them, which would make things WORSE than today (a miss then
// costs a whole run instead of one block).  Two ways with LRU keeps both
// streams resident.  Three would buy little for another 4 BRAMs.
//
// Default 2 x 32 blocks = 32 KiB ~= 8 RAMB36 on the KU5P.  The 200 MHz
// build uses 159 RAMB36 + 44 RAMB18 of ~240 RAMB36-equivalent, so this
// is ~3% of the device's block RAM (~14% of what is still free) and NONE
// of the URAM, all 64 of which are spoken for by L2.  It is also
// entirely in pb_clk (50 MHz), so it adds nothing to the 200 MHz core
// timing problem.
//
// MEASURED amortisation — tb/tb_vhdd_readahead.cpp, 256 sequential
// single-block reads, re-costed with the board's own 69.7 ms/command and
// 0.263 ms/block.  BLOCKS_PER_WAY is one parameter in
// rtl/soc/fpga_top_peripherals.vh if a later build wants to spend more:
//
//     per way   RAM     commands   blocks/s     KB/s    vs today
//        -        -        256        14.3       7.1      1.0x   (today)
//        8      8 KiB       32       111.4      55.7      7.8x
//       16     16 KiB       16       216.5     108.2     15.1x
//       32     32 KiB        8       409.6     204.8     28.6x   (default)
//       64     64 KiB        4       739.6     369.8     51.7x
//      128    128 KiB        2      1238.3     619.2     86.6x
//
// Why stop at 32.  A run that is not fully used is wasted card time: a
// purely RANDOM read costs 69.7 + F*0.263 ms, i.e. +12% at F=32 but +48%
// at F=128.  Mac boot traffic is mostly sequential, so a bigger run
// almost certainly wins — but 32 is the size that cannot plausibly make
// anything worse, and the knob is one line.
//
// SERVE THROUGH THE FILL — and why fill-then-serve is WRONG here
// ══════════════════════════════════════════════════════════════
// On a miss the module starts a multi-block read and, one byte behind
// the card, immediately starts handing the master its own blocks out of
// the same BRAM.  `way_bytes` is the fill frontier and the serve engine
// is not allowed to overtake it; that single comparison is the entire
// synchronisation between the two.
//
// The obvious simpler shape — absorb the whole run, THEN serve — was
// built first, passed every unit test in tb/tb_vhdd_readahead.cpp, and
// broke the machine.  scsi.v's stuck-supply watchdog (VH_STUCK_TIMEOUT,
// rtl/mac/scsi.v) is BUSY-INDEPENDENT by design: it counts contiguous
// cycles of "multi-block read, ring empty, no byte arriving" in
// S_VH_WAIT_RD / S_DATA_IN and, on expiry, completes the command as
// CHECK CONDITION / HARDWARE ERROR.  A quiet fill is indistinguishable
// from a dead card, and every multi-block read failed.  Caught by
// tb-scsi-sd-e2e-ra; fenced now by `first_byte_is_not_delayed_by_the_run`
// in the unit bench, which measures first-byte latency against the
// provider's command latency rather than trusting the shape.
//
// So the invariant this module owes scsi.v is not "answer eventually" —
// it is: NEVER LEAVE THE MASTER'S RING EMPTY FOR VH_STUCK_TIMEOUT WHILE
// IT IS OWED BYTES.  Serving through the fill satisfies it the same way
// the un-cached path always did: the first byte out is the first byte in.
//
// Two further consequences of the split, both deliberate:
//
//  * The card is NEVER back-pressured on a cached read.  Every
//    starvation failure this project has chased — withheld pseudo-DMA
//    beats, c96_shim_rd_starved, the supply pre-staging work in scsi.v —
//    comes from the card's stream being paced by the Mac's drain.  Here
//    the card streams into BRAM and the Mac drains out of BRAM.
//
//  * `busy` DROPS as soon as the master's own blocks are served, while
//    the tail of the run is still landing.  The next request is accepted
//    immediately, and if it is sequential it is answered from the run in
//    flight — including from blocks that have not arrived yet, which the
//    serve engine simply waits for.  Holding `busy` across the tail
//    instead would re-create the same watchdog exposure one level up.
//
// The one case that still waits is a genuine SEEK arriving mid-tail: a
// CMD18 cannot be aborted, so the new request holds until the run ends.
// That is bounded by BLOCKS_PER_WAY * per-block stream time — 8.2 ms at
// 32 blocks against a 160 ms watchdog — and a seek was going to pay a
// full 70 ms command latency regardless.  BLOCKS_PER_WAY MUST KEEP THAT
// MARGIN: at the measured 0.263 ms/block the hard ceiling is ~608 blocks
// and anything past ~128 is asking for trouble on a slower card.
//
// CORRECTNESS — this is a CACHE, so coherency is the whole job
// ════════════════════════════════════════════════════════════
//  * WRITES. Invalidate every overlapping way on req_go, including a
//    fill in progress. Non-overlapping streams survive metadata writes.
//    Invalidate the whole affected way, not individual bytes; even a
//    failed/partial write must never leave an old cached copy available.
//    Use 33-bit half-open extents so end-LBA arithmetic cannot wrap.
//    Malformed write extents conservatively invalidate both ways.
//  * STRADDLING READS.  A hit requires the WHOLE extent
//    [lba, lba+count) to be inside one way's FILLED region.  A read that
//    straddles two ways, or runs off the end of a partially filled way,
//    is a miss and refetches from its own first block.  Never stitched.
//  * SEEKS.  A miss always refetches starting at the requested LBA into
//    the LRU way; the other way (the other stream) survives.
//  * OVERSIZED REQUESTS.  count > BLOCKS_PER_WAY bypasses the cache
//    entirely (straight pass-through, provider sees the master's own
//    request).  Such a request is already amortised — it is one CMD18
//    for many blocks, which is the thing this module exists to create.
//  * CAPACITY.  The fetch run is clamped to num_lbas so read-ahead can
//    never address past the end of the volume; if the clamp cannot cover
//    the master's own request the request is passed through untouched.
//  * MEDIUM CHANGE.  A change in num_lbas invalidates both ways: that is
//    a different volume behind the same port.
//  * RESETS.  `rst` (pb_full_rst_bank[2] in the SoC) clears everything.
//    `inval` is a separate level for resets that belong to the OTHER
//    side of the SD path — soc_full_rst_bank[5], warm_storage_reset,
//    warm_peripheral_reset, dbg_cold_reset_hold — synchronised into this
//    domain by the platform.  Asserting it invalidates both ways AND
//    poisons an in-flight fill, and aborts an in-flight serve as
//    done|error rather than handing over data from a volume that may no
//    longer be the same one.  The cache can go stale in exactly one
//    direction that matters (someone wrote the card behind our back) and
//    every such path is either below the reserved window (boot_fsm,
//    pram_sd — different LBAs entirely) or gated by one of those resets.
//  * ERRORS.  A fetch that ends in `error` is never cached.  If enough
//    blocks landed to satisfy the master's own extent before the error,
//    they are served and the request succeeds — a bad sector at run+20
//    must not fail a good read of run+0.  Otherwise the error is passed
//    up unchanged.
//
// BOUNDED RESPONSE (rtl/vhdd.vh, load-bearing behaviour 2)
// ═══════════════════════════════════════════════════════
// The fetch inherits the provider's bound, whatever it is: we add no
// timeout there, deliberately, because u_sd_ctrl_scsi runs with
// REQ_WDOG_ENABLE(0) by owner directive so that a healthy-but-slow card
// PARKS observably instead of erroring.  Adding a timeout in front of it
// would undo that decision.
//
// The SERVE phase is different — there the provider is idle and the only
// thing that can stall is the CONSUMER holding rd_ready low, which the
// contract explicitly says must still terminate.  SERVE_WDOG_BITS bounds
// exactly that, and nothing else: ~335 ms at 50 MHz, two orders of
// magnitude past any legitimate pseudo-DMA gap (scsi.v's own
// VH_STUCK_TIMEOUT is 160 ms).
//
// NOT IN THE PATH
// ═══════════════
// boot_fsm has its own sd_ctrl and its own port on sd_spi_mux; it never
// touches the vhdd seam, so its single-CMD18 ROM load is untouched.

`default_nettype none

`include "vhdd.vh"

module vhdd_readahead #(
    // Blocks per way.  MUST be a power of two.  Total BRAM is
    // 2 * BLOCKS_PER_WAY * 512 bytes.
    parameter integer BLOCKS_PER_WAY  = 32,
    // 0 builds the module out entirely: a bit-for-bit pass-through with
    // no BRAM, no state, no added cycle.  Used by the testbenches to get
    // a true "before" baseline from the identical netlist.
    parameter integer ENABLE          = 1,
    // Consumer-stall bound for the SERVE phase only (see above).
    parameter integer SERVE_WDOG_BITS = 24
) (
    input  wire        clk,
    input  wire        rst,
    // Level.  High == every cached run is void (see RESETS above).
    input  wire        inval,

    // ── master face: we are the vhdd PROVIDER ─────────────────────────
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
    output wire        done,
    output wire        error,

    output wire        rd_valid,
    output wire [7:0]  rd_data,
    input  wire        rd_ready,

    output wire        wr_ready,
    input  wire        wr_valid,
    input  wire [7:0]  wr_data,
    input  wire        wr_avail,

    // ── provider face: we are the vhdd MASTER ─────────────────────────
    output wire [31:0] p_chk_lba,
    output wire [23:0] p_chk_blocks,
    input  wire        p_chk_ok,

    output wire        p_req_write,
    output wire        p_req_multi,
    output wire [31:0] p_req_lba,
    output wire [15:0] p_req_block_count,
    output wire        p_req_go,

    input  wire        p_busy,
    input  wire        p_done,
    input  wire        p_error,

    input  wire        p_rd_valid,
    input  wire [7:0]  p_rd_data,
    output wire        p_rd_ready,

    input  wire        p_wr_ready,
    output wire        p_wr_valid,
    output wire [7:0]  p_wr_data,
    output wire        p_wr_avail
);

    // The extent probe has no side effect and no state: straight through
    // on both arms.
    assign p_chk_lba    = chk_lba;
    assign p_chk_blocks = chk_blocks;
    assign chk_ok       = p_chk_ok;

    // The write byte stream is never cached and never re-timed.
    assign p_wr_valid   = wr_valid;
    assign p_wr_data    = wr_data;
    assign p_wr_avail   = wr_avail;
    assign wr_ready     = p_wr_ready;

    // ══════════════════════════════════════════════════════════════════
    // Geometry
    // ══════════════════════════════════════════════════════════════════
    localparam integer BLK_BYTES = `VHDD_BLOCK_BYTES;      // 512
    localparam integer BLK_LG2   = 9;                      // log2(512)

    function integer f_clog2;
        input integer v;
        integer i;
        begin
            f_clog2 = 0;
            for (i = v - 1; i > 0; i = i >> 1) f_clog2 = f_clog2 + 1;
        end
    endfunction

    localparam integer BW        = f_clog2(BLOCKS_PER_WAY);  // block index bits
    localparam integer AW        = 1 + BW + BLK_LG2;         // {way, blk, byte}
    localparam integer WAY_BYTES = BLOCKS_PER_WAY * BLK_BYTES;

    // Byte counters must hold WAY_BYTES itself (the "full" value), so one
    // bit wider than the way address.
    localparam integer CW        = BW + BLK_LG2;

    // Sized copies of the two geometry constants.  Part-selecting an
    // `integer` localparam is a portability trap (it is a 32-bit signed
    // object, and tools disagree about slicing one), so everything that
    // needs bits takes them from here.
    localparam [CW:0]   WAY_BYTES_C = WAY_BYTES[CW:0];
    localparam [BW:0]   BPW_C       = BLOCKS_PER_WAY[BW:0];

generate
if (ENABLE == 0) begin : g_bypass
    // ── Built out.  Identical to having no module here at all. ────────
    assign p_req_write       = req_write;
    assign p_req_multi       = req_multi;
    assign p_req_lba         = req_lba;
    assign p_req_block_count = req_block_count;
    assign p_req_go          = req_go;
    assign busy              = p_busy;
    assign done              = p_done;
    assign error             = p_error;
    assign rd_valid          = p_rd_valid;
    assign rd_data           = p_rd_data;
    assign p_rd_ready        = rd_ready;

    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused_byp = &{1'b0, clk, rst, inval, num_lbas, 1'b0};
    /* verilator lint_on UNUSEDSIGNAL */
end else begin : g_ra

    // ══════════════════════════════════════════════════════════════════
    // Storage — simple dual port, byte wide.  The FILL writes one port
    // and the SERVE reads the other, and they genuinely run at the same
    // time (that is the whole point of "serve through the fill"), so
    // these must stay two separate ports.
    // ══════════════════════════════════════════════════════════════════
    reg [7:0] cache_mem [0:(2*WAY_BYTES)-1];

    reg [AW-1:0] fill_ptr;
    reg [AW-1:0] serve_ptr;
    reg [7:0]    serve_q;

    // ══════════════════════════════════════════════════════════════════
    // Tags
    // ══════════════════════════════════════════════════════════════════
    // way_bytes is the load-bearing one: how many bytes of this way's run
    // are ACTUALLY IN THE RAM right now.  It is zeroed when a fill claims
    // the way and counts up byte by byte as the card delivers, so it
    // doubles as the stream frontier that a serve running alongside the
    // fill must not overtake.
    //
    // way_valid is only ever set at a CLEAN end of fill.  A way being
    // filled is INVALID and is reached through the in-flight path below,
    // so "valid" always means "this run is complete and trustworthy".
    reg          way_valid [0:1];
    reg [31:0]   way_base  [0:1];   // volume LBA of the way's block 0
    reg [CW:0]   way_bytes [0:1];   // bytes present, from that base
    reg          lru;               // way to evict next

    reg [31:0]   last_num_lbas;
    wire         medium_changed = (num_lbas != last_num_lbas);
    // Everything that makes a cached run void THIS cycle.  Read
    // combinationally, never through a register, because a non-blocking
    // flag would still be 0 on the very cycle the kill arrives — and that
    // cycle can also be the fill's `p_done`.
    wire         cache_kill = inval || medium_changed;

    // ══════════════════════════════════════════════════════════════════
    // Two cooperating FSMs
    // ══════════════════════════════════════════════════════════════════
    // FILL owns the provider: one multi-block read, absorbed into a way,
    // never back-pressured.
    //
    // REQUEST owns the master: exactly one vhdd request at a time,
    // answered out of the RAM.
    //
    // They are separate because the interesting case is the one where
    // both are live.
    localparam [1:0] F_IDLE  = 2'd0,
                     F_ISSUE = 2'd1,   // one cycle: our own req_go
                     F_FILL  = 2'd2;

    localparam [2:0] R_IDLE   = 3'd0,
                     R_WAIT   = 3'd1,  // provider busy; hold the request
                     R_PASSGO = 3'd2,  // one cycle: forward the master's go
                     R_PASS   = 3'd3,  // uncached request in flight
                     R_SERVE  = 3'd4,
                     R_FIN    = 3'd5,  // last byte on the wire
                     R_ERR    = 3'd6;

    reg [1:0]    fst;
    reg [2:0]    rq;

    reg [31:0]   f_lba;       // first LBA of the run being filled
    reg [15:0]   f_count;     // blocks asked for
    reg          f_way;
    reg          f_poison;    // invalidated mid-fill: do not cache it

    reg          pend_pass;   // what R_WAIT is waiting to do

    reg [15:0]   serve_left;  // bytes still owed to the master
    reg          serve_way;

    reg          done_r, err_r, rd_valid_r;
    reg [SERVE_WDOG_BITS-1:0] serve_wdog;

    wire fill_active   = (fst != F_IDLE);
    wire prov_busy_any = fill_active || p_busy;

    // ── Hit test ──────────────────────────────────────────────────────
    // Full containment in ONE way.  Never stitched across two: a read
    // that straddles is a miss and refetches from its own first block.
    wire [31:0] off0 = req_lba - way_base[0];
    wire [31:0] off1 = req_lba - way_base[1];
    wire in0 = (req_lba >= way_base[0]) && (off0 < BLOCKS_PER_WAY);
    wire in1 = (req_lba >= way_base[1]) && (off1 < BLOCKS_PER_WAY);
    wire [16:0] need0 = {1'b0, off0[15:0]} + {1'b0, req_block_count};
    wire [16:0] need1 = {1'b0, off1[15:0]} + {1'b0, req_block_count};
    wire [16:0] blocks0 = {{(17-(BW+1)){1'b0}}, way_bytes[0][CW:BLK_LG2]};
    wire [16:0] blocks1 = {{(17-(BW+1)){1'b0}}, way_bytes[1][CW:BLK_LG2]};

    wire hit0 = way_valid[0] && in0 && (need0 <= blocks0);
    wire hit1 = way_valid[1] && in1 && (need1 <= blocks1);

    wire write_request = req_go && req_write;
    wire [32:0] write_blocks = req_multi ? {17'd0, req_block_count} : 33'd1;
    wire [32:0] write_end = {1'b0, req_lba} + write_blocks;
    wire write_extent_bad = (write_blocks == 33'd0) || write_end[32];
    wire [32:0] way_end0 = {1'b0, way_base[0]} + BLOCKS_PER_WAY;
    wire [32:0] way_end1 = {1'b0, way_base[1]} + BLOCKS_PER_WAY;
    wire [1:0] write_kill;
    assign write_kill[0] = write_request && (write_extent_bad ||
        (({1'b0, req_lba} < way_end0) && (write_end > {1'b0, way_base[0]})));
    assign write_kill[1] = write_request && (write_extent_bad ||
        (({1'b0, req_lba} < way_end1) && (write_end > {1'b0, way_base[1]})));

    // IN-FLIGHT hit: the extent is inside the run the card is STILL
    // delivering.  This is the sequential case, and it is the one that
    // matters — it lets the block after the one just served be answered
    // with no provider request at all, out of bytes that have not even
    // arrived yet.  way_base[f_way] is written at fill START precisely so
    // this arithmetic works while the run is in flight.
    wire        inf   = f_way ? in1   : in0;
    wire [16:0] needf = f_way ? need1 : need0;
    //
    // f_poison/cache_kill are in here and that is LOAD-BEARING: an
    // invalidation (a write, a peer reset, a medium change) that lands
    // while a run is still streaming must take the IN-FLIGHT run with it,
    // not just the two completed ways.  Without this a write during the
    // tail would be followed by a read served from the pre-write copy the
    // card is still delivering — which is precisely the silent
    // disk-corruption case this module is not allowed to have.
    wire hit_fly = fill_active && !f_poison && !cache_kill && !write_kill[f_way] &&
                   inf && (needf <= {1'b0, f_count});

    wire        hit_any = hit0 || hit1 || hit_fly;
    wire        hit_way = hit0 ? 1'b0 : (hit1 ? 1'b1 : f_way);
    // Only the low BW bits are ever used: `in0`/`in1` already proved the
    // offset is inside the way.
    wire [BW-1:0] hit_off = hit_way ? off1[BW-1:0] : off0[BW-1:0];

    // ── Is this a request the cache may handle at all? ────────────────
    wire [31:0] lbas_left = num_lbas - req_lba;
    wire        in_volume = (req_lba < num_lbas);
    wire [15:0] fetch_len = (lbas_left >= BLOCKS_PER_WAY)
                              ? {{(16-(BW+1)){1'b0}}, BPW_C}
                              : lbas_left[15:0];
    wire cacheable = !req_write &&
                     (req_block_count != 16'd0) &&
                     ({{(16-(BW+1)){1'b0}}, req_block_count[BW:0]}
                        == req_block_count) &&
                     (req_block_count[BW:0] <= BPW_C) &&
                     in_volume &&
                     (fetch_len >= req_block_count);

    // ── Serve engine ──────────────────────────────────────────────────
    // A byte may leave the RAM only when it is actually IN the RAM.
    // way_bytes is the fill frontier, so this one comparison is what
    // makes serving through a live fill safe — and what makes a serve
    // that has outrun a DEAD fill detectable rather than silent.
    wire [CW:0] serve_off   = {1'b0, serve_ptr[CW-1:0]};
    wire        serve_avail = (serve_off < way_bytes[serve_way]);
    wire        serve_re    = (rq == R_SERVE) && (serve_left != 16'd0) &&
                              rd_ready && serve_avail;
    // The bytes we still owe will never come: the fill that was going to
    // deliver them has finished, or died, short.
    wire        serve_dead  = (rq == R_SERVE) && (serve_left != 16'd0) &&
                              !serve_avail && !fill_active;
    // Consumer stall: the byte IS available and rd_ready is low.  This,
    // and only this, is what SERVE_WDOG_BITS bounds — waiting for the
    // CARD is the provider's clock to keep, not ours.
    wire        serve_stall = (rq == R_SERVE) && (serve_left != 16'd0) &&
                              serve_avail && !rd_ready;

    wire fill_we = (fst == F_FILL) && p_rd_valid &&
                   (way_bytes[f_way] < WAY_BYTES_C);

    // ── Start strobes, shared by the two FSMs ─────────────────────────
    // A request enters R_SERVE from R_IDLE (a fresh go) or from R_WAIT (a
    // held one).  In both cases the master's req_* are still the ones the
    // request was made with: rtl/vhdd.vh guarantees they hold until the
    // next req_go, and `busy` has been high throughout, so there cannot
    // have been one.
    wire serve_from_idle = (rq == R_IDLE) && req_go && cacheable &&
                           (hit_any || !prov_busy_any);
    wire serve_from_wait = (rq == R_WAIT) && !prov_busy_any && !pend_pass;
    wire serve_start     = serve_from_idle || serve_from_wait;
    wire start_is_hit    = hit_any;
    wire        serve_start_way = start_is_hit ? hit_way : lru;
    wire [AW-1:0] serve_start_ptr =
            start_is_hit ? {hit_way, hit_off, {BLK_LG2{1'b0}}}
                         : {lru,     {(AW-1){1'b0}}};
    // A miss also launches the fill that is going to feed it.
    wire fill_start = serve_start && !start_is_hit;

    // ── Provider face ─────────────────────────────────────────────────
    assign p_req_write       = (fst == F_ISSUE) ? 1'b0              : req_write;
    assign p_req_multi       = (fst == F_ISSUE) ? (f_count > 16'd1) : req_multi;
    assign p_req_lba         = (fst == F_ISSUE) ? f_lba             : req_lba;
    assign p_req_block_count = (fst == F_ISSUE) ? f_count           : req_block_count;
    assign p_req_go          = (fst == F_ISSUE) || (rq == R_PASSGO);

    // Never back-pressure the card except when the master's own bytes are
    // the ones in flight (R_PASS).  Everywhere else a 1 also lets an
    // abandoned stream drain to `done` instead of parking in `busy`,
    // which rtl/vhdd.vh requires.
    assign p_rd_ready = (rq == R_PASS) ? rd_ready : 1'b1;

    // ── Master face ───────────────────────────────────────────────────
    assign rd_valid = (rq == R_PASS) ? p_rd_valid : rd_valid_r;
    assign rd_data  = (rq == R_PASS) ? p_rd_data  : serve_q;
    assign done     = (rq == R_PASS) ? p_done     : done_r;
    assign error    = (rq == R_PASS) ? p_error    : err_r;
    // NOTE WHAT IS NOT IN HERE: `fill_active`.  Once the master's own
    // blocks have been served, `busy` drops even though the card is still
    // streaming the rest of the run into the RAM — so the next request is
    // accepted immediately and, if it is sequential, answered from the
    // run in flight.  Parking `busy` for the tail instead would leave the
    // master sitting in S_VH_WAIT_RD with an empty ring, which is exactly
    // what scsi.v's busy-INDEPENDENT stuck-supply watchdog counts.
    assign busy     = (rq != R_IDLE) | req_go | done_r;

    // ══════════════════════════════════════════════════════════════════
    // Storage ports
    // ══════════════════════════════════════════════════════════════════
    always @(posedge clk) begin
        if (fill_we) cache_mem[fill_ptr] <= p_rd_data;
    end
    always @(posedge clk) begin
        if (serve_re) serve_q <= cache_mem[serve_ptr];
    end

    // ══════════════════════════════════════════════════════════════════
    // FILL
    // ══════════════════════════════════════════════════════════════════
    integer w;
    always @(posedge clk) begin
        if (rst) begin
            fst      <= F_IDLE;
            f_lba    <= 32'd0;
            f_count  <= 16'd0;
            f_way    <= 1'b0;
            f_poison <= 1'b0;
            fill_ptr <= {AW{1'b0}};
            lru      <= 1'b0;
            last_num_lbas <= 32'd0;
            for (w = 0; w < 2; w = w + 1) begin
                way_valid[w] <= 1'b0;
                way_base[w]  <= 32'd0;
                way_bytes[w] <= {(CW+1){1'b0}};
            end
        end else begin
            last_num_lbas <= num_lbas;

            // Resets/medium changes kill everything. A write only kills
            // overlapping ways, including the speculative tail of a fill.
            for (w = 0; w < 2; w = w + 1)
                if (cache_kill || write_kill[w]) way_valid[w] <= 1'b0;
            if (cache_kill || write_kill[f_way]) f_poison <= 1'b1;

            case (fst)
                F_IDLE: ;   // launched by fill_start below

                // One cycle.  p_req_go is high here with f_lba/f_count
                // already stable — the "valid on the go cycle" the
                // contract asks of a vhdd master.
                F_ISSUE: fst <= F_FILL;

                F_FILL: begin
                    if (fill_we) begin
                        fill_ptr         <= fill_ptr + 1'b1;
                        way_bytes[f_way] <= way_bytes[f_way] + 1'b1;
                    end
                    if (p_done) begin
                        fst <= F_IDLE;
                        // Cache the run only if it completed cleanly and
                        // nothing invalidated it while it was landing.  A
                        // run that errored is USED (the serve may already
                        // have taken good blocks off the head of it) but
                        // never KEPT.
                        if (!p_error && !f_poison && !cache_kill && !write_kill[f_way] &&
                            (way_bytes[f_way][CW:BLK_LG2] != 0)) begin
                            way_valid[f_way] <= 1'b1;
                            lru              <= ~f_way;
                        end
                    end
                end

                default: fst <= F_IDLE;
            endcase

            // ── LRU ───────────────────────────────────────────────────
            // A way that just answered a request is the most recently
            // used one, so the OTHER way is next out.  Written here and
            // not in the REQUEST block so `lru` keeps a single driver.
            // Mutually exclusive with fill_start below (that fires only
            // on a MISS), and it beats the fill-end update in the case
            // where both land together, which is the right order.
            if (serve_start && start_is_hit) lru <= ~hit_way;

            // ── Launch ────────────────────────────────────────────────
            // Only ever asserted with fst == F_IDLE (serve_start requires
            // !prov_busy_any on the miss path), so it cannot interrupt a
            // run in flight.
            if (fill_start) begin
                f_lba          <= req_lba;
                f_count        <= fetch_len;
                f_way          <= lru;
                f_poison       <= 1'b0;
                fill_ptr       <= {lru, {(AW-1){1'b0}}};
                way_valid[lru] <= 1'b0;
                way_base[lru]  <= req_lba;
                way_bytes[lru] <= {(CW+1){1'b0}};
                fst            <= F_ISSUE;
            end
        end
    end

    // ══════════════════════════════════════════════════════════════════
    // REQUEST
    // ══════════════════════════════════════════════════════════════════
    always @(posedge clk) begin
        if (rst) begin
            rq         <= R_IDLE;
            done_r     <= 1'b0;
            err_r      <= 1'b0;
            rd_valid_r <= 1'b0;
            serve_ptr  <= {AW{1'b0}};
            serve_left <= 16'd0;
            serve_way  <= 1'b0;
            serve_wdog <= {SERVE_WDOG_BITS{1'b0}};
            pend_pass  <= 1'b0;
        end else begin
            done_r     <= 1'b0;
            rd_valid_r <= serve_re;

            case (rq)
                R_IDLE: begin
                    err_r <= 1'b0;
                    if (req_go) begin
                        if (!cacheable) begin
                            pend_pass <= 1'b1;
                            rq <= prov_busy_any ? R_WAIT : R_PASSGO;
                        end else if (hit_any) begin
                            rq <= R_SERVE;
                        end else if (prov_busy_any) begin
                            // A genuine seek while the tail of the
                            // previous run is still landing.  A CMD18
                            // cannot be aborted mid-stream, so hold the
                            // request until the card is free: bounded by
                            // the run length, and a seek was going to
                            // cost a whole command latency anyway.
                            pend_pass <= 1'b0;
                            rq <= R_WAIT;
                        end else begin
                            rq <= R_SERVE;
                        end
                    end
                end

                R_WAIT: if (!prov_busy_any)
                            rq <= pend_pass ? R_PASSGO : R_SERVE;

                R_PASSGO: rq <= R_PASS;

                R_PASS: if (p_done) begin
                    pend_pass <= 1'b0;
                    rq <= R_IDLE;
                end

                R_SERVE: begin
                    if (serve_re) begin
                        serve_ptr  <= serve_ptr + 1'b1;
                        serve_left <= serve_left - 16'd1;
                        if (serve_left == 16'd1) rq <= R_FIN;
                    end
                    if (serve_stall) begin
                        serve_wdog <= serve_wdog + 1'b1;
                        if (&serve_wdog) begin
                            // Consumer stalled with data waiting.  The
                            // contract says answer anyway.
                            err_r <= 1'b1;
                            rq    <= R_ERR;
                        end
                    end else begin
                        serve_wdog <= {SERVE_WDOG_BITS{1'b0}};
                    end
                    if (serve_dead || cache_kill) begin
                        err_r <= 1'b1;
                        rq    <= R_ERR;
                    end
                end

                // Last byte is on the wire this cycle (rd_valid_r);
                // completion lands the cycle after it, so the master has
                // certainly counted it.
                R_FIN: begin
                    done_r <= 1'b1;
                    rq     <= R_IDLE;
                end

                R_ERR: begin
                    done_r <= 1'b1;
                    rq     <= R_IDLE;
                end

                default: rq <= R_IDLE;
            endcase

            // ── Serve setup, on every entry into R_SERVE ──────────────
            if (serve_start) begin
                serve_way  <= serve_start_way;
                serve_ptr  <= serve_start_ptr;
                serve_left <= {req_block_count[15-BLK_LG2:0], {BLK_LG2{1'b0}}};
                serve_wdog <= {SERVE_WDOG_BITS{1'b0}};
            end
        end
    end

end

endgenerate

endmodule

`default_nettype wire
