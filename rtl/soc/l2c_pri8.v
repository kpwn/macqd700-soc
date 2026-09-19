// l2c_pri8.v -- tiny fixed-width (8-entry) lowest-index priority encoder.
//
// Shared by l2c_mshr.v for its three 8-entry scans (lookup-match,
// free-slot, next-to-service) -- pure combinational, no state.
//
// Verilog-2005.

`default_nettype none

module l2c_pri8 (
    input  wire [7:0] req,  // req[i] = 1 means "candidate i"
    output wire        hit,
    output wire [2:0]  idx
);

    assign hit = |req;
    assign idx = req[0] ? 3'd0 :
                 req[1] ? 3'd1 :
                 req[2] ? 3'd2 :
                 req[3] ? 3'd3 :
                 req[4] ? 3'd4 :
                 req[5] ? 3'd5 :
                 req[6] ? 3'd6 : 3'd7;

endmodule

`default_nettype wire
