// l2c_plru_funcs.vh -- 8-way tree-PLRU pure functions.
//
// Deliberately UNGUARDED (no `ifndef): Verilog functions are module-
// scoped, so each module that calls l2c_plru_victim/l2c_plru_next needs
// its own textual copy pasted in via `include -- an include guard here
// would mean only the first module in the whole compilation to include
// this file actually gets working functions, and every other includer
// would fail with an undefined-function error.  Every module below
// includes this file AT MOST ONCE, so no guard is needed to prevent
// double-inclusion within a single module either.
//
// 7-bit tree vector layout:
//   t[0] = root (0: victim in ways0-3, 1: victim in ways4-7)
//   t[1] = ways0-3 node (0: victim way0-1, 1: victim way2-3)
//   t[2] = ways4-7 node (0: victim way4-5, 1: victim way6-7)
//   t[3] = way0/1 leaf (0: victim way1, 1: victim way0)
//   t[4] = way2/3 leaf (0: victim way3, 1: victim way2)
//   t[5] = way4/5 leaf (0: victim way5, 1: victim way4)
//   t[6] = way6/7 leaf (0: victim way7, 1: victim way6)
// On access to `way`, each node on the root->way path is set to point
// AWAY from the accessed child (so it becomes the next victim direction
// on the *other* side).  Used by l2c_victim_sel.v.

function [2:0] l2c_plru_victim;
    input [6:0] t;
    reg w2, w1, w0;
    begin
        w2 = t[0];
        w1 = w2 ? t[2] : t[1];
        w0 = w2 ? (w1 ? ~t[6] : ~t[5]) : (w1 ? ~t[4] : ~t[3]);
        l2c_plru_victim = {w2, w1, w0};
    end
endfunction

function [6:0] l2c_plru_next;
    input [6:0] t;
    input [2:0] way;
    reg [6:0] nt;
    begin
        // Leaf bits use the OPPOSITE polarity from the group bits (see the
        // layout comment above: leaf t[3]=1 means "victim is the *first*
        // (lower-numbered) way of the pair", vs. group t[1]=1 meaning
        // "victim is the *second* (higher) half") -- so the leaf next-state
        // is the accessed way's own low bit (identity), not its negation.
        // Verified by hand-trace: t=0 -> victim=way1; access way1 ->
        // t[0]/t[1]<=1, t[3]<=way[0](=1) -> next victim=way5 (correctly
        // moves away from the just-touched way1, not back onto it).
        nt = t;
        if (way[2] == 1'b0) begin
            nt[0] = 1'b1;
            if (way[1] == 1'b0) begin nt[1] = 1'b1; nt[3] = way[0]; end
            else                begin nt[1] = 1'b0; nt[4] = way[0]; end
        end else begin
            nt[0] = 1'b0;
            if (way[1] == 1'b0) begin nt[2] = 1'b1; nt[5] = way[0]; end
            else                begin nt[2] = 1'b0; nt[6] = way[0]; end
        end
        l2c_plru_next = nt;
    end
endfunction
