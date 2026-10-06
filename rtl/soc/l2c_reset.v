// l2c_reset.v -- L2 walking tag-clear FSM.
//
// BRAM/URAM has no bulk-clear primitive, so on `rst` this sweeps
// set = 0 .. SETS-1, pulsing l2c_tags' clr_en/clr_set (which invalidates
// all WAYS ways of that set in one cycle -- see l2c_tags.v).  `busy` is
// held for the whole walk (~SETS cycles); l2c.v holds the slave port's
// AW/AR ready low while busy, far below the xbar S0 watchdog bound
// (2^18 cycles, docs/tracks/platform.md) -- see docs/l2c_spec.md S7.
//
// Verilog-2005, sync active-high rst.  Also drives `mshr_victim_clr`, a
// one-cycle pulse when the walk completes, that l2c.v uses to
// synchronously clear the MSHR table and victim buffer (both are plain
// registers, cleared directly by l2c.v's own `rst`, so this pulse is
// only used to gate the front door back open -- see `done`).

`default_nettype none

module l2c_reset #(
    parameter SET_BITS = 12  // sweeps set = 0 .. 2**SET_BITS-1
) (
    input  wire                clk,
    input  wire                rst,
    output wire                busy,
    output wire                done,      // 1-cycle pulse, walk just finished
    output wire                clr_en,
    output wire [SET_BITS-1:0] clr_set
);

    reg                walking;
    reg [SET_BITS-1:0] set_ctr;
    reg                done_r;
    // 2026-10-06 (200 MHz FMax): `busy` is its OWN register, a copy of
    // `walking` with an identical waveform.  `walking` also drives clr_en and
    // the set counter; sharing that flop with the front-door gates made it a
    // fanout-107 hub in the routed 200 MHz build.  busy_q is computed from
    // the same next-state terms as `walking` (set on rst, cleared on the
    // last set), so it is cycle-exact -- verified: `make tb-l2c` output is
    // byte-identical to the pre-change tree with this form.  (A one-cycle-
    // extended form, rst || walking, would also be safe -- the doors just
    // open a cycle later -- but it shifts every downstream testbench's
    // random interleaving, so the exact copy is preferred.)  l2c_ctrl.v no
    // longer uses `busy` in its accept cone at all (see "RESET WALK GATES
    // ONLY THE HEADER DOORS" there); MAX_FANOUT lets synthesis replicate it
    // for the remaining header-ready / MSHR-reset consumers.
    (* max_fanout = 16 *) reg busy_q;

    assign busy    = busy_q;
    assign done    = done_r;
    assign clr_en  = walking;
    assign clr_set = set_ctr;

    always @(posedge clk) begin
        done_r <= 1'b0;
        busy_q <= rst || (walking && (set_ctr != {SET_BITS{1'b1}}));
        if (rst) begin
            walking <= 1'b1;
            set_ctr <= {SET_BITS{1'b0}};
        end else if (walking) begin
            if (set_ctr == {SET_BITS{1'b1}}) begin
                walking <= 1'b0;
                done_r  <= 1'b1;
            end else begin
                set_ctr <= set_ctr + 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
