// l2c_victim_sel.v -- pure combinational victim-way selection for a miss.
//
// Given the 8 ways' current valid bits, a "busy" mask (ways already
// claimed by an in-flight MSHR entry in this same set -- see
// l2c_mshr.v's bw_mask), and the set's PLRU tree state, picks the way to
// evict: prefer any non-busy INVALID way; else the PLRU-indicated way if
// it isn't busy; else any other non-busy way (last-resort, keeps
// correctness even if PLRU's specific pick is momentarily claimed).
// `victim_ok` is 0 only when every way in the set is currently claimed by
// an in-flight MSHR entry -- the caller (l2c_ctrl.v) stalls the miss in
// that (rare, high-contention) case rather than double-claiming a way.
//
// Verilog-2005, pure combinational (no clk/rst).

`default_nettype none

module l2c_victim_sel (
    input  wire [7:0] rvalid,
    input  wire [7:0] busy_mask,
    input  wire [6:0] plru_t,
    output wire        victim_ok,
    output wire [2:0]  victim_way
);
    `include "l2c_plru_funcs.vh"

    wire [7:0] inv_free_c = (~rvalid) & (~busy_mask);
    wire        any_inv_c;
    wire [2:0]  inv_way_c;
    l2c_pri8 u_inv (.req(inv_free_c), .hit(any_inv_c), .idx(inv_way_c), .onehot());

    wire [2:0] plru_way_c = l2c_plru_victim(plru_t);
    wire       plru_ok_c  = !busy_mask[plru_way_c];

    wire [7:0] free_mask_c = ~busy_mask;
    wire        any_free_c;
    wire [2:0]  free_way_c;
    l2c_pri8 u_free (.req(free_mask_c), .hit(any_free_c), .idx(free_way_c), .onehot());

    assign victim_way = any_inv_c ? inv_way_c : (plru_ok_c ? plru_way_c : free_way_c);
    assign victim_ok  = any_inv_c || plru_ok_c || any_free_c;

endmodule

`default_nettype wire
