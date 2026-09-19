// clut.v -- stage 10 of the scan-out pipeline: THE RAMDAC PALETTE.
// ---------------------------------------------------------------------------
// index -> RGB.  Owns the 256-entry palette memory and its write port, and
// nothing else: it does not know about bit depths (stage 9 owns those), scale
// factors (stage 11) or borders (stage 12).  docs/video_path_review.md S4.1
// stage 10; previously an inline BRAM inside scanout_display.v.
//
// TRUE DUAL-CLOCK, DUAL-PORT BRAM.  The write port runs in the DAFB shim's
// clock domain (`wclk` = core_clk at the real instantiation site), driven
// straight from video.v's AC842 RAMDAC write protocol; the read port is in
// pclk.  There is NO synchroniser and that is deliberate -- Vivado infers a
// native RAMB18/36 true-dual-port primitive with independent clocks, and a
// mid-frame palette write is ALLOWED to land immediately (transient sparkle
// on the entries in flight is authentic RAMDAC behaviour).
//
// The memory is deliberately NOT synchronously reset: it comes up zero via
// bitstream INIT / Verilator zero-init and software reprograms it through the
// real RAMDAC path.  Resetting it would cost a write-port mux for no gain.
// All indexed depths index the SAME 256-entry table, matching MAME's
// dafb_base::screen_update().
//
// THE BYPASS LANE, AND WHY IT LIVES HERE.  At 24bpp the pixel does not go
// through the palette at all -- it is the line store's raw {p0,p1,p2}.  That
// candidate still has to arrive at the compositor on the SAME cycle as the
// palette read, or the final 24-bit mux would be choosing between two
// different pixels.  So `bypass_in` is registered here, in the one module
// that defines what "one palette read of latency" means.  Putting the
// matching register in the compositor instead would put the two halves of a
// single latency contract in two files, which is precisely the failure mode
// docs/video_path_review.md S4.1 rule 1 exists to prevent.  BOTH candidates
// are registered unconditionally; the depth mux is downstream, which avoids
// piping `fetch_direct` alongside the data.
//
// LATENCY: exactly 1 registered cycle, on `rclk`, for BOTH outputs.  This is
// stage 4 of scanout_display.v's five-stage ladder; see the LATENCY LADDER
// note there.  Changing it changes the ladder and every parallel control pipe
// in the compositor must move with it.
//
// DSP: none.  There is no arithmetic in this module at all -- it is a memory
// and two registers.
// ---------------------------------------------------------------------------
`default_nettype none

module clut (
    // ── Read side (pixel clock) ──────────────────────────────────────
    input  wire        rclk,
    input  wire        resetn,
    input  wire [7:0]  index,
    // Latency-matched passthrough for the 24bpp direct-colour candidate.
    input  wire [23:0] bypass_in,
    output reg  [23:0] rgb,
    output reg  [23:0] bypass_rgb,

    // ── Write side (DAFB shim clock -- NOT rclk) ─────────────────────
    input  wire        wclk,
    input  wire        we,
    input  wire [7:0]  waddr,
    input  wire [23:0] wdata
);

    (* ram_style = "block" *) reg [23:0] clut_mem [0:255];

    always @(posedge wclk) begin
        if (we)
            clut_mem[waddr] <= wdata;
    end

    always @(posedge rclk) begin
        if (!resetn) begin
            rgb        <= 24'h00_00_00;
            bypass_rgb <= 24'h00_00_00;
        end else begin
            rgb        <= clut_mem[index];
            bypass_rgb <= bypass_in;
        end
    end

endmodule

`default_nettype wire
