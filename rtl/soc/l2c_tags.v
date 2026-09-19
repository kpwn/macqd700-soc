// l2c_tags.v — L2 cache tag + valid/dirty state array, 8-way, plus the
// per-set 8-way tree-PLRU state array.
//
// SECTORED (2026-08-19, docs/l2c_perf.md S12).  Valid and dirty are each
// FOUR bits per way -- one per 128-bit quadrant of the 64 B line, which is
// exactly one 68040 L1D line.  The entry is {vsec[3:0], dsec[3:0], tag}.
//
//   vsec[q] -- quadrant q of this line holds real data.  A line may be
//              partially valid: a fully-strobed 16 B write installs its
//              own quadrant with NO fill, leaving the other three invalid.
//   dsec[q] -- quadrant q has been modified since it was fetched, so it
//              must be written back on eviction.  A single byte written
//              anywhere used to mark all 64 B dirty and push the whole
//              line; now only the touched quadrants are pushed.
//
// INVARIANT (relied on by l2c_victim.v's partial writeback): dsec[q]
// implies vsec[q], and a dirty quadrant holds valid data in ALL 16 of its
// bytes -- a partial-strobe write can only reach a quadrant that is
// already valid, and a full-strobe write supplies every byte itself.
//
// `rvalid` is retained as the per-way LINE-valid bit (|vsec) so
// l2c_victim_sel.v and the eviction path keep their existing semantics;
// it is a 4-input OR of the array output, computed in parallel with the
// tag compare and consumed by the same LUT level, so it costs no depth.
//
// One independent BRAM per way (8 total), each 4096 x {vsec,dsec,tag}.
// Registered read (1 cycle): raddr presented this cycle, all 8 ways'
// {valid,dirty,tag} valid on rvalid/rdirty/rtag the *next* cycle.  Callers
// that also read l2c_data.v (2-cycle URAM latency) should hold raddr
// stable for 2 cycles and use the tag output from the 2nd cycle (still
// correct -- BRAM output holds the same value once raddr stops moving).
//
// rd_en (2026-09-15) is a clock enable on the read output register, the
// same net l2c_data.v's rd_en is and for the same reason: l2c_ctrl now
// drives raddr from a REGISTER one cycle ahead of the array, so a stalled
// consumer can no longer hold its view by re-pointing a combinational
// raddr at its own set.  It holds the output instead.  Tie high for the
// old free-running behaviour.  Nine BRAMs (8 tag + 1 PLRU), so this adds
// no high-fanout control net.
//
// PLRU state is a separate 4096 x 7b array (7 tree-node bits for 8 ways),
// same read/write timing as the tag arrays.  The tree-encode/decode
// functions themselves (l2c_plru_victim / l2c_plru_next) live in
// l2c_defs.vh and are called by l2c.v, which owns the read-modify-write
// sequencing (this module just stores raw bits).
//
// Two write paths, mutually exclusive per cycle (arbitrated by the
// caller, l2c.v):
//   - wr_en:  normal one-way write (hit-path dirty-mark, evict-invalidate,
//             MSHR install/replay-write).
//   - clr_en: reset-walk broadcast clear -- invalidates *all* 8 ways of
//             clr_set in the same cycle (l2c_reset.v drives this while
//             walking all 4096 sets after reset), and clears that set's
//             PLRU state to 0.
//
// Latency: 1 cycle (BRAM registered read).  Verilog-2005, sync reset only
// for control (the array contents are cleared by the walking FSM, not by
// `rst` directly -- BRAM has no bulk-clear primitive).
//
// ── ARRAY COLLISION: WHAT THIS MODULE DOES *NOT* PROMISE ──────────────────
// The reads below are non-blocking reads of `mem` / `plru_mem` on the RHS,
// so SIMULATION returns the pre-write value when a read and a write hit the
// same address in one cycle.  Do not build on that.  These are BRAM rather
// than URAM, so a READ_FIRST WRITE_MODE at least EXISTS here -- but this is
// a simple-dual-port array with independent read and write addresses, and
// UG901's RW_ADDR_COLLISION entry says that for exactly this shape Vivado
// "infers a block RAM and sets the write mode to WRITE_FIRST for best
// timing", adding that "if a design writes to the same address it is reading
// from, the RAM output is unpredictable".  Vivado can only honour a
// read-first DESCRIPTION when the read and write share one address signal,
// which these do not.  See l2c_data.v's header for the URAM half of the same
// argument.
//
// l2c does NOT depend on collision behaviour in either array -- see the
// ARRAY READ-AFTER-WRITE note in l2c_ctrl.v -- and `make tb-l2c-collision`
// is the standing proof.

`default_nettype none

module l2c_tags #(
    parameter SETS     = 4096,
    parameter SET_BITS = 12,
    parameter WAYS     = 8,
    parameter WAY_BITS = 3,
    parameter TAG_BITS = 14
) (
    input  wire                       clk,

    // Read port -- shared address, all ways in parallel.
    input  wire [SET_BITS-1:0]        raddr,
    input  wire                       rd_en,
    output wire [WAYS-1:0]            rvalid,   // per-way LINE valid (|vsec)
    output wire [WAYS*4-1:0]          rvsec,    // per-way, per-quadrant valid
    output wire [WAYS*4-1:0]          rdsec,    // per-way, per-quadrant dirty
    output wire [WAYS*TAG_BITS-1:0]   rtag,

    // Normal write port (one way at a time).  vsec/dsec are written whole,
    // not per-bit: the caller (l2c_ctrl.v) supplies the already-merged
    // masks, which it can do because the tag array is read at accept time
    // and `skew_hazard_c` retries any request whose set was written in the
    // two cycles since -- so the value it merges into is never stale.
    // 2026-09-15 (FMax): the per-way write enable arrives ALREADY DECODED,
    // as `wr_wen_oh`, instead of as (wr_en, wr_way).  It used to be
    // `wr_en && (wr_way == w)`, which put TWO logic levels -- the
    // install-vs-resolve wr_en mux and the way compare -- between
    // l2c_ctrl's `tw_en` and this array's WEA pins, on top of a third for
    // tw_en itself.  The caller can fold all of that into one LUT because
    // the way decode is a function of registered state (mshr_inst_way) or
    // of a cone that is one level SHORTER than tw_en's (sel_way_c), so it
    // is ready earlier; see l2c_ctrl.v's `tags_wen_oh_c`.
    //
    // The broadcast clear folds in there too, so `clr_en` survives here
    // only to select the address and the all-zero data.
    input  wire [WAYS-1:0]            wr_wen_oh,
    input  wire [SET_BITS-1:0]        wr_set,
    input  wire [3:0]                 wr_vsec,
    input  wire [3:0]                 wr_dsec,
    input  wire [TAG_BITS-1:0]        wr_tag,

    // Reset-walk broadcast clear.
    input  wire                       clr_en,
    input  wire [SET_BITS-1:0]        clr_set,

    // PLRU state (raw 7b tree vector, one array, same read/write pattern).
    output wire [6:0]                 rplru,
    input  wire                       plru_wr_en,
    input  wire [SET_BITS-1:0]        plru_wr_set,
    input  wire [6:0]                 plru_wr_data
);

    localparam ENTRY_W = 8 + TAG_BITS; // {vsec[3:0], dsec[3:0], tag}

    genvar w;
    generate
        for (w = 0; w < WAYS; w = w + 1) begin : g_way
            (* ram_style = "block" *) reg [ENTRY_W-1:0] mem [0:SETS-1];
            reg [ENTRY_W-1:0] dout;

            wire            this_wen   = wr_wen_oh[w];
            wire [SET_BITS-1:0] this_waddr = clr_en ? clr_set : wr_set;
            wire [ENTRY_W-1:0]  this_wdata = clr_en
                                              ? {ENTRY_W{1'b0}}
                                              : {wr_vsec, wr_dsec, wr_tag};

            always @(posedge clk) begin
                if (this_wen)
                    mem[this_waddr] <= this_wdata;
                if (rd_en) begin
`ifdef L2C_ARRAY_COLLISION_HOSTILE
                    // RED-VERIFY BUILD ONLY (see the header's ARRAY COLLISION
                    // note).  Poison deliberately reads back as ALL QUADRANTS
                    // VALID with an all-ones tag -- the dangerous direction,
                    // because it manufactures a FALSE HIT rather than a benign
                    // miss.
                    if (this_wen && (this_waddr == raddr))
                        dout <= {4'b1111, 4'b1111, {TAG_BITS{1'b1}}};
                    else
`elsif L2C_ARRAY_COLLISION_WRITEFIRST
                    if (this_wen && (this_waddr == raddr))
                        dout <= this_wdata;
                    else
`endif
                    dout <= mem[raddr];
                end
            end

            assign rvsec[w*4 +: 4]                        = dout[ENTRY_W-1 -: 4];
            assign rdsec[w*4 +: 4]                        = dout[ENTRY_W-5 -: 4];
            assign rvalid[w]                              = |dout[ENTRY_W-1 -: 4];
            assign rtag[w*TAG_BITS +: TAG_BITS]            = dout[TAG_BITS-1:0];

            // synthesis translate_off
            // INVARIANT: within one set, at most ONE way may hold a given
            // tag with any quadrant valid.  l2c_ctrl.v's header calls this
            // out as the uniqueness "the whole design assumes" -- way_hit_c,
            // tag_match_c and the fill-into-the-tag-matching-way rule are
            // all written as if a tag can live in at most one way of a set,
            // and l2c_victim/l2c_mshr inherit it.  Two ways with one tag
            // means two lines claim one address: a read answers from
            // whichever way the priority encoder reaches first, an eviction
            // writes back one of them, and the other's dirty data is lost.
            //
            // Before the lookup was pipelined this held by construction:
            // every allocation read the tag array one cycle after the
            // previous allocation wrote it.  A pipelined lookup resolves
            // against a snapshot taken two cycles earlier, so uniqueness
            // now rests on l2c_ctrl's accept-time set/line interlock AND on
            // skew_hazard_c's install-shadow window being deep enough.
            // Neither is self-evident and the corruption they prevent
            // surfaces arbitrarily far from its cause, so it is checked on
            // the exact edge it could be introduced: any write that leaves
            // a way valid, against every other way of that set.
            always @(posedge clk) begin
                if ((|wr_wen_oh) && !clr_en && (|wr_vsec) &&
                    !wr_wen_oh[w] &&
                    (|mem[wr_set][ENTRY_W-1 -: 4]) &&
                    (mem[wr_set][TAG_BITS-1:0] == wr_tag)) begin
                    $display("L2C_TAGS ASSERT: set %0d tag %0h left valid in BOTH way %0d and the way being written (wen_oh %b)",
                             wr_set, wr_tag, w, wr_wen_oh);
                    $fatal(1);
                end
            end
            // synthesis translate_on
        end
    endgenerate

    (* ram_style = "block" *) reg [6:0] plru_mem [0:SETS-1];
    reg [6:0] plru_dout;
    wire            plru_this_wen   = clr_en || plru_wr_en;
    wire [SET_BITS-1:0] plru_this_waddr = clr_en ? clr_set : plru_wr_set;
    wire [6:0]          plru_this_wdata = clr_en ? 7'b0 : plru_wr_data;

    always @(posedge clk) begin
        if (plru_this_wen)
            plru_mem[plru_this_waddr] <= plru_this_wdata;
        if (rd_en) begin
`ifdef L2C_ARRAY_COLLISION_HOSTILE
            if (plru_this_wen && (plru_this_waddr == raddr)) plru_dout <= 7'h5a;
            else
`elsif L2C_ARRAY_COLLISION_WRITEFIRST
            if (plru_this_wen && (plru_this_waddr == raddr)) plru_dout <= plru_this_wdata;
            else
`endif
            plru_dout <= plru_mem[raddr];
        end
    end
    assign rplru = plru_dout;

endmodule

`default_nettype wire
