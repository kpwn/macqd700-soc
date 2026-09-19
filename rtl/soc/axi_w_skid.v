// axi_w_skid.v — one-channel AXI4 write-data register slice (skid buffer).
//
// WHY THIS EXISTS (2026-09-12, FMax).
//
// `axi_xbar.v` computes every master's WREADY as a live combinational
// function of every slave's WREADY (see `gen_wready` / `mw_wready_raw`),
// and it selects each slave's WDATA/WSTRB/WLAST/WVALID from a live
// combinational decode of all four slots' `ws_state[]` (see `s?_wowner`).
// `l2c_ctrl.v` in turn computes `s_axi_wready` from `accept_slot_c`, which
// contains `q_adv_c` -- the FULL stage-2 resolve outcome, the deepest cone
// in the design.
//
// Put together, a single unbroken combinational path ran
//
//     xbar ws_state[a] -> s0_wvalid/s0_wstrb
//       -> l2c front door -> l2c s_axi_wready
//       -> xbar mw_wready[b] -> ws_bcnt[b] -> ws_state[b]
//
// i.e. one slot's write FSM to ANOTHER slot's write FSM, through the whole
// L2 cache front door.  Measured on the routed CPU-less 200 MHz build it
// was TWENTY logic levels / 5.418 ns and it owned the design's WNS
// (-0.624 ns).  A second cone -- `req_set` -> MSHR busy-way mask ->
// victim select -> `s_lookup_miss_go` -> `q_adv_c` -> `s_axi_wready` ->
// the same xbar write FSM -- was 24 levels / 5.578 ns and owned another
// ~1000 failing endpoints, including every `axi_narrow_to_wide` gather
// register behind a master port.
//
// Neither is fixable by making the L2C front door shallower alone: the
// ready signal is *exported* into another module's state machine, so the
// two modules' worst cones ADD.  A register slice on the W channel is the
// standard structural answer -- it makes the xbar's view of WREADY a plain
// register bit ("the slice has room") and the L2C's view of WVALID/WDATA a
// plain register output, so the two cones are cut apart and each is timed
// on its own.
//
// THROUGHPUT IS UNCHANGED.  `s_wready = !skd_v` is fully registered (it
// does NOT look at `m_wready`), which is the entire point; the classic
// cost of that is one bubble when a stalled slice drains, and this design
// pays it only on the cycle the skid entry moves to the output register.
// In steady state -- downstream ready every cycle, or ready every other
// cycle as the L2C's rw_favor alternator produces -- the upstream accept
// rate exactly matches the downstream consume rate.  It costs one cycle of
// W LATENCY, which the front door already tolerates (it cannot take a W
// beat until `aw_have` is registered anyway).
//
// ORDERING/PROTOCOL NOTES.  This is a pure channel FIFO: beats leave in the
// order they arrived, WLAST travels with its beat, and nothing is dropped
// or duplicated.  The xbar's abandoned-burst padder (`WS_PAD_W`) counts
// beats the *slave interface* accepted, which is now this slice; the slice
// then delivers exactly those beats, with the same WLAST, to the slave, so
// the slave's own burst FSM still sees AWLEN+1 beats and returns to idle
// before the xbar admits the next AW.  `sw_owned[]` is not released until
// the slave's B has been consumed, and a slave cannot emit B before it has
// taken every W beat, so the slice is always empty at an ownership change.
//
// Reset must be the SAME reset as both endpoints (core_rst_bank[4] for the
// xbar/L2C/DDR trio) -- a slice reset independently of its neighbours would
// swallow a beat the xbar still counts as delivered.
//
// SECOND USER (2026-09-16, FMax): dma_engine.sv instantiates this on its OWN
// master W port, i.e. slice-then-crossbar rather than crossbar-then-slave.
// The reset rule is trivially satisfied there (the slice sits inside the
// module and shares its `rst` port).  The reason is different too: not a
// cross-module ready/valid loop but a cross-die BROADCAST -- one BRAM read
// port feeding the W muxes of three slaves placed in three corners of the
// die.  See the note in dma_engine.sv for the routed measurement.
//
// Verilog-2005, sync active-high rst.
`default_nettype none

module axi_w_skid #(
    parameter DATA_WIDTH = 128,
    parameter STRB_WIDTH = DATA_WIDTH/8
) (
    input  wire                    clk,
    input  wire                    rst,

    // Upstream (crossbar side).
    input  wire [DATA_WIDTH-1:0]   s_wdata,
    input  wire [STRB_WIDTH-1:0]   s_wstrb,
    input  wire                    s_wlast,
    input  wire                    s_wvalid,
    output wire                    s_wready,

    // Downstream (slave side).
    output wire [DATA_WIDTH-1:0]   m_wdata,
    output wire [STRB_WIDTH-1:0]   m_wstrb,
    output wire                    m_wlast,
    output wire                    m_wvalid,
    input  wire                    m_wready
);
    localparam PAY_W = DATA_WIDTH + STRB_WIDTH + 1;

    wire [PAY_W-1:0] s_pay = {s_wlast, s_wstrb, s_wdata};

    reg [PAY_W-1:0] out_pay, skd_pay;
    reg             out_v,   skd_v;

    // The whole reason this module exists: REGISTERED, and with no
    // reference to m_wready.  Do not "optimise" this into
    // `!skd_v || m_wready` -- that re-creates the cross-module loop.
    assign s_wready = !skd_v;

    assign m_wvalid = out_v;
    assign {m_wlast, m_wstrb, m_wdata} = out_pay;

    always @(posedge clk) begin
        if (rst) begin
            out_v <= 1'b0;
            skd_v <= 1'b0;
        end else begin
            if (!out_v || m_wready) begin
                // Output register is free (or drains this cycle): refill it
                // from the skid if the skid holds anything, else straight
                // from the upstream beat we are accepting this cycle.
                if (skd_v) begin
                    out_v   <= 1'b1;
                    out_pay <= skd_pay;
                    skd_v   <= 1'b0;
                end else begin
                    out_v   <= s_wvalid;
                    out_pay <= s_pay;
                end
            end else if (s_wvalid && !skd_v) begin
                // Output register is stalled but we advertised s_wready, so
                // this beat has been handed to us and must be kept.
                skd_v   <= 1'b1;
                skd_pay <= s_pay;
            end
        end
    end

    // synthesis translate_off
    // AXI payload stability: once WVALID is asserted, WDATA/WSTRB/WLAST may
    // not change until the beat is accepted.  This is the property a badly
    // written skid silently breaks -- it still hands over the right NUMBER
    // of beats, so a functional test can pass while a real slave that
    // samples mid-stall gets a torn one.
    reg             chk_v;
    reg [PAY_W-1:0] chk_pay;
    always @(posedge clk) begin
        if (rst) begin
            chk_v <= 1'b0;
        end else begin
            chk_v   <= m_wvalid && !m_wready;
            chk_pay <= out_pay;
            if (chk_v && m_wvalid && (chk_pay !== out_pay)) begin
                $display("AXI_W_SKID ASSERT: W payload changed while stalled at WVALID");
                $fatal(1);
            end
        end
    end
    // synthesis translate_on

endmodule

`default_nettype wire
