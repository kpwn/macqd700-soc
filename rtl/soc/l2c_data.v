// l2c_data.v — L2 cache data array, 8-way, URAM-backed.
//
// One independent URAM array per way (8 total), each 4096 x 512 bits
// (64 B lines) = 256 KB/way x 8 ways = 2 MB total, matching
// docs/l2c_spec.md geometry.  `(* ram_style = "ultra" *)` per-way, never
// a single 2D array (platform.md BRAM-inference rule extended to URAM).
//
// 2-cycle REGISTERED read (matches UltraScale+ URAM288 native latency --
// address presented cycle 0, data valid cycle 2).  Byte-lane write
// (64 individual enables from wr_strb).  One write port, arbitrated by
// the caller (l2c.v) between the hit-path write and MSHR install/replay
// writes -- never driven from two sources in the same cycle.
//
// Verilog-2005, sync clk only (no content reset -- l2c.v never reads a
// data-array location whose tag is not simultaneously valid, so
// power-up/reset garbage in the data array is never observable).
//
// ── ARRAY COLLISION: WHAT THIS MODULE DOES *NOT* PROMISE ──────────────────
// The read below is a non-blocking read of `mem` on the RHS, so SIMULATION
// returns the pre-write value when a read and a write hit the same address
// in one cycle.  SILICON DOES NOT PROMISE THAT, and it is important that no
// caller come to depend on it:
//
//   * URAM288 has NO write-mode attribute at all -- UG573 ch.2 is explicit,
//     "there are no user definable read-first, write-first, no-change modes
//     with UltraRAM", and UG901 ch.5 repeats it.  READ_FIRST is a BRAM
//     WRITE_MODE; it does not exist here.
//   * The behaviour is DEFINED but port-ordered: UG573 "Dual Port SRAM Array
//     Operations" guarantees port A executes before port B, so a read gets
//     OLD data only if the tool put the READ on port A and the WRITE on
//     port B.  With the write on A, the reader gets NEW data -- deterministic
//     silicon, opposite to this model.
//   * Which port Vivado assigns for an INFERRED simple-dual-port URAM with
//     independent read and write addresses is not documented anywhere.  What
//     IS documented (UG901, RW_ADDR_COLLISION) is that for exactly this shape
//     Vivado defaults to WRITE_FIRST "for best timing", that on collision
//     "the RAM output is unpredictable", and that the only bypass it offers
//     produces WRITE_FIRST -- never read-first.  UG901 ch.5 warns in terms
//     that "the RTL and post-synthesis simulations could be different".
//
// So this is a genuine simulator-versus-silicon divergence class, and it is
// invisible to every ordinary test because Verilog gives read-first for free.
// l2c does NOT depend on it -- see the ARRAY READ-AFTER-WRITE note in
// l2c_ctrl.v for the two mechanisms that actually make it safe -- and
// `make tb-l2c-collision` is the standing proof, rebuilding this file so a
// collided read returns poison (or the new data) and requiring the whole
// suite to pass anyway.  If you ever make a caller consume a collided read,
// that target is what will tell you.

`default_nettype none

module l2c_data #(
    parameter SETS       = 4096,
    parameter SET_BITS   = 12,
    parameter WAYS       = 8,
    parameter WAY_BITS   = 3,
    parameter LINE_BITS  = 512,
    parameter LINE_BYTES = 64
) (
    input  wire                          clk,

    // Read port -- shared address, all ways in parallel, 2-cycle latency.
    //
    // rd_en is a clock enable on BOTH output registers (2026-09-15, l2c_ctrl
    // array-address pipelining).  It was the SECOND register only: `raddr`
    // was then a combinational mux that l2c_ctrl re-pointed at the stalled
    // stage's own set every stalled cycle, so the FIRST register could
    // free-run and regenerate that stage's output on demand.  raddr is now
    // driven from a REGISTER (l2c_ctrl's `ra_q`), one cycle ahead of the
    // array, so "re-issue the set I still need" costs a whole cycle and can
    // no longer be used as a hold.  The pipeline instead advances as a rigid
    // shift (l2c_ctrl's `pipe_adv_c`): every stage moves together or none
    // does, which makes ONE clock enable correct for both registers -- and
    // keeps this module's array-CE fanout at exactly the one net it already
    // had, rather than adding a second high-fanout control net to 64 URAMs.
    // Tie high for the un-pipelined behaviour.
    input  wire [SET_BITS-1:0]           raddr,
    input  wire                          rd_en,
    output wire [WAYS*LINE_BITS-1:0]     rdata,

    // Write port (one way at a time).
    // 2026-09-15 (FMax): per-way write enable, ALREADY DECODED -- same
    // change, same reason, as l2c_tags.v's `wr_wen_oh`.  `wr_en && (wr_way
    // == w)` cost two logic levels between l2c_ctrl's `dw_en` and these
    // URAMs' write-enable pins; the caller folds both into one LUT whose
    // way-decode input is ready earlier than dw_en is.
    input  wire [WAYS-1:0]               wr_wen_oh,
    input  wire [SET_BITS-1:0]           wr_set,
    input  wire [LINE_BITS-1:0]          wr_data,
    input  wire [LINE_BYTES-1:0]         wr_strb
);

    // UG901 requires ALL writes to a given inferred RAM to originate from
    // a SINGLE always block/process -- one process per byte (as this used
    // to be, 64 processes writing slices of the same `mem`) breaks BRAM/
    // URAM inference even though URAM288 genuinely does support per-byte
    // write enables.  Single always with a for-loop over bytes below is
    // the correct coding pattern for the same functional byte-enable
    // write.
    genvar w; integer b;
    generate
        for (w = 0; w < WAYS; w = w + 1) begin : g_way
            (* ram_style = "ultra" *) reg [LINE_BITS-1:0] mem [0:SETS-1];
            reg [LINE_BITS-1:0] dout_r1;
            reg [LINE_BITS-1:0] dout;

            wire this_wen = wr_wen_oh[w];

            always @(posedge clk) begin
                if (this_wen) begin
                    for (b = 0; b < LINE_BYTES; b = b + 1)
                        if (wr_strb[b]) mem[wr_set][b*8 +: 8] <= wr_data[b*8 +: 8];
                end
                if (rd_en) begin
`ifdef L2C_ARRAY_COLLISION_HOSTILE
                    // RED-VERIFY BUILD ONLY -- never defined in synth/vivado.tcl.
                    // See the ARRAY COLLISION note in the header: a read that
                    // collides with a same-address write returns POISON instead
                    // of the pre-write data, modelling the worst case the
                    // silicon is permitted to do.  `make tb-l2c-collision`
                    // requires the suite to pass anyway, which is the standing
                    // proof that no collided read is ever consumed.
                    if (this_wen && (wr_set == raddr)) begin
                        for (b = 0; b < LINE_BYTES; b = b + 1)
                            dout_r1[b*8 +: 8] <= wr_strb[b] ? 8'hA5
                                                            : mem[raddr][b*8 +: 8];
                    end else
`elsif L2C_ARRAY_COLLISION_WRITEFIRST
                    // Same build, the REALISTIC alternative: URAM port ordering
                    // with the write on port A gives the reader the NEW data.
                    if (this_wen && (wr_set == raddr)) begin
                        for (b = 0; b < LINE_BYTES; b = b + 1)
                            dout_r1[b*8 +: 8] <= wr_strb[b] ? wr_data[b*8 +: 8]
                                                            : mem[raddr][b*8 +: 8];
                    end else
`endif
                    dout_r1 <= mem[raddr];
                    dout    <= dout_r1;
                end
            end

            assign rdata[w*LINE_BITS +: LINE_BITS] = dout;
        end
    endgenerate

endmodule

`default_nettype wire
