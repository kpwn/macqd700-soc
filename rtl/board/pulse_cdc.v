// pulse_cdc.v — toggle-based 1-cycle pulse synchroniser.
//
// Purpose
// ───────
// Crosses a single-cycle level pulse from the source clock domain into
// a 1-cycle pulse on the destination clock domain.  Tolerates any
// frequency relationship as long as the source pulse spacing is at
// least the destination domain's "guaranteed observation" interval —
// i.e. consecutive source pulses must be far enough apart that the
// destination side sees the toggled level for ≥ 1 destination cycle
// before it flips again.  For a slow destination domain (e.g. 50 MHz
// pb_clk) and a fast source domain (e.g. 148.5 MHz pclk) this is fine
// for any pulse rate up to ~10 MHz.
//
// Why not a level synchroniser?  A level signal would cause VIA1's
// own edge detector to fire only when the level actually changes.
// For a periodic signal that's two CA1 edges per period (rise +
// fall) — that's fine for a 60 Hz signal where we want one VBL per
// frame, BUT a level synchroniser cannot guarantee a destination-
// domain pulse if the source pulse is shorter than the dest period.
// The toggle-based pattern below sidesteps that: every source pulse
// flips a level, the level survives until the next source pulse, and
// the destination edge detector emits exactly one dst-domain pulse
// per source pulse.
//
// Pattern (Cummings, "Clock Domain Crossing", SNUG 2008):
//
//       src_clk ──┐                ┌── dst_clk ──┐
//                 │                │             │
//   src_pulse ─[toggle]─[meta]─[sync]─[edge_det]─> dst_pulse
//
// Resets are synchronous and active high in their respective domains.
//
`default_nettype none

module pulse_cdc (
    input  wire src_clk,
    input  wire src_rst,
    input  wire src_pulse,

    input  wire dst_clk,
    input  wire dst_rst,
    output reg  dst_pulse
);

    // Source side: toggle a level every time src_pulse fires.
    reg src_toggle;
    always @(posedge src_clk) begin
        if (src_rst)
            src_toggle <= 1'b0;
        else if (src_pulse)
            src_toggle <= ~src_toggle;
    end

    // Destination side: 2-flop synchroniser + edge detector.  The
    // dst_pulse is a 1-cycle strobe each time the synchronised level
    // toggles.
    (* ASYNC_REG = "TRUE" *) reg dst_meta;
    (* ASYNC_REG = "TRUE" *) reg dst_sync;
    reg dst_sync_d;

    always @(posedge dst_clk) begin
        if (dst_rst) begin
            dst_meta   <= 1'b0;
            dst_sync   <= 1'b0;
            dst_sync_d <= 1'b0;
            dst_pulse  <= 1'b0;
        end else begin
            dst_meta   <= src_toggle;
            dst_sync   <= dst_meta;
            dst_sync_d <= dst_sync;
            dst_pulse  <= dst_sync ^ dst_sync_d;
        end
    end

endmodule

`default_nettype wire
