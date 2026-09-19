// tb_scanout_frames.v -- MULTI-FRAME linebuf_scanout gate at SHIPPING geometry.
// ---------------------------------------------------------------------------
// Why this tb exists (read before "simplifying" it):
//
// The 2026-07-31 hardware bug -- exactly LINE_COUNT (64) source rows render at
// the top of the screen and everything below is black -- survived the entire
// pre-existing video tb suite for four structural reasons, and every one of
// them is a deliberate property of THIS file:
//
//   1. MULTIPLE CONSECUTIVE FRAMES.  The failure is a failure to RE-ARM the
//      fetch walk between frames.  tb-framebuffer-pixel, tb-dafb-scanout and
//      tb-vram-scaler-firstlight all assert inside a single frame walk, so
//      they structurally cannot see it.  This tb runs N frames back to back
//      and asserts on EVERY frame independently.
//
//   2. MORE THAN LINE_COUNT SOURCE ROWS.  The park happens exactly at the
//      ring wrap (source row 64 needs slot 0 back).  A source height <= 64
//      never reaches it.  Here vres is 480 with LINE_COUNT_LOG2=6, so the
//      64-entry ring wraps 7.5 times per frame.
//
//   3. ROW-VARYING CONTENT.  With identical rows a parked fetcher and a
//      working one produce the SAME image.  Every scenario here paints
//      content whose value is a function of the source row, so the tb can
//      name which source row each display line actually shows.
//
//   4. SHIPPING GEOMETRY.  tb-framebuffer-pixel elaborates 64x64 into 64x64
//      with LINE_COUNT_LOG2=4 and no blanking at all; the real failure needs
//      a real 1920x1080 VTG whose ~45-line vertical blanking lets the fetch
//      walk run 64 rows AHEAD of the display before the frame starts.  This
//      tb instantiates the production vtg.v and the production
//      linebuf_scanout parameterisation (SRC 1024x768, DST 1920x1080,
//      LINE_COUNT_LOG2=6, ADDR_W=21, FB_MAX_PIXELS=2 MiB).
//
// The fetch port is modelled in C++ (tb_scanout_frames.cpp) rather than by
// instantiating fb_reader + vram, because the C++ model can INJECT a dropped
// response -- the fault af8fe56's watchdog was written to survive -- which no
// RTL-only rig can do.  tb-fb-reader-ddr-chain and tb-vram-ddr-chain already
// cover the real fetch-port RTL; this tb covers the scanner's frame FSM.
// ---------------------------------------------------------------------------
`default_nettype none

module tb_scanout_frames #(
    parameter SRC_W          = 1024,
    parameter SRC_H          = 768,
    parameter DST_W          = 1920,
    parameter DST_H          = 1080,
    parameter ADDR_W         = 21,
    parameter FETCH_W        = 32,
    parameter FB_MAX_PIXELS  = 32'h0020_0000,
    parameter LINE_COUNT_LOG2 = 6
) (
    input  wire                 pclk,
    input  wire                 rst,

    // Runtime placement / depth, driven straight from the harness.  In the
    // real design these come from scanout_placement_sync (frame-boundary
    // committed); every scenario here holds them constant across a whole
    // run, so the sync stage would be a pass-through and is omitted.
    input  wire [ADDR_W-1:0]    fb_base_px,
    input  wire [ADDR_W-1:0]    fb_stride_px,
    input  wire [2:0]           bpp_shift,
    input  wire [2:0]           bytes_per_px,
    input  wire [11:0]          hres,
    input  wire [11:0]          vres,
    input  wire [2:0]           scale_n,
    // Boot-splash switchover, driven straight from the harness.  In the real
    // design this is scanout_placement_sync's sticky "the DAFB committed a
    // renderable placement" latch; here it is a scenario knob so one rig can
    // run the same geometry with the splash up and with it retired.
    input  wire                 dafb_live,

    // CLUT write port (harness-driven, same clock -- the real one is
    // core_clk; the CDC is not what this tb is about).
    input  wire                 clut_we,
    input  wire [7:0]           clut_waddr,
    input  wire [23:0]          clut_wdata,

    // Fetch port to the C++ framebuffer model.
    output wire                 fb_rd_en,
    output wire [ADDR_W-1:0]    fb_rd_addr,
    input  wire                 fb_rd_ready,
    input  wire [FETCH_W-1:0]   fb_rd_data,
    input  wire                 fb_rd_valid,

    // Scanned-out video.
    output wire [23:0]          rgb,
    output wire                 de_out,
    output wire                 hs_out,
    output wire                 vs_out,
    output wire                 line_underflow_sticky,

    // VTG state, exported so the harness can bucket output pixels by frame
    // and by display line without re-deriving the timing.
    output wire [11:0]          hcount,
    output wire [10:0]          vcount,
    output wire                 de_vtg,

    // ── Fetch-walk observability ─────────────────────────────────────
    // These are the signals that distinguish "the fetcher is walking the
    // frame" from "the fetcher parked at the ring wrap".  Exported as
    // probes so the tb asserts on the MECHANISM as well as on the pixels.
    //
    // Post-rewrite the mechanism is the CREDIT COUNT and a 4-state FSM, so
    // that is what is exported.  `dbg_credits` is the direct replacement for
    // the old `dbg_req_slot_reusable` / `dbg_awaiting_frame_start` pair: a
    // fetcher parked at the ring wrap is one holding zero credits while the
    // display is still advancing, and that is now impossible by construction
    // (credits are returned unconditionally) -- the tb asserts it anyway.
    output wire                 dbg_prefetch_active,
    output wire                 dbg_can_request,
    output wire [2:0]           dbg_state,
    output wire [15:0]          dbg_credits,
    output wire [15:0]          dbg_stale_count,
    output wire [15:0]          dbg_outstanding,
    output wire [15:0]          dbg_req_y,
    output wire [15:0]          dbg_req_x,
    output wire [15:0]          dbg_rsp_y,
    output wire [15:0]          dbg_y_src,
    output wire                 dbg_outstanding_empty
);

    localparam SRC_Y_W = $clog2(SRC_H);
    localparam SRC_X_W = $clog2(SRC_W);

    wire vtg_de, vtg_hs, vtg_vs;

    vtg #(
        .H_ACTIVE(DST_W),
        .V_ACTIVE(DST_H)
    ) u_vtg (
        .pclk   (pclk),
        .resetn (~rst),
        .hcount (hcount),
        .vcount (vcount),
        .de     (vtg_de),
        .hsync  (vtg_hs),
        .vsync  (vtg_vs)
    );

    assign de_vtg = vtg_de;

    linebuf_scanout #(
        .SRC_W           (SRC_W),
        .SRC_H           (SRC_H),
        .FB_MAX_PIXELS   (FB_MAX_PIXELS),
        .DST_W           (DST_W),
        .DST_H           (DST_H),
        .FETCH_W         (FETCH_W),
        .ADDR_W          (ADDR_W),
        .LINE_COUNT_LOG2 (LINE_COUNT_LOG2)
    ) u_scanout (
        .pclk        (pclk),
        .resetn      (~rst),
        .hcount      (hcount),
        .vcount      (vcount),
        .de_in       (vtg_de),
        .hs_in       (vtg_hs),
        .vs_in       (vtg_vs),
        .fb_base_px  (fb_base_px),
        .fb_stride_px(fb_stride_px),
        .bpp_shift   (bpp_shift),
        .bytes_per_px(bytes_per_px),
        .hres        (hres),
        .vres        (vres),
        .scale_n     (scale_n),
        .dafb_live   (dafb_live),
        .clut_wclk   (pclk),
        .clut_we     (clut_we),
        .clut_waddr  (clut_waddr),
        .clut_wdata  (clut_wdata),
        .fb_rd_en    (fb_rd_en),
        .fb_rd_addr  (fb_rd_addr),
        .fb_rd_ready (fb_rd_ready),
        .fb_rd_data  (fb_rd_data),
        .fb_rd_valid (fb_rd_valid),
        .rgb         (rgb),
        .de_out      (de_out),
        .hs_out      (hs_out),
        .vs_out      (vs_out),
        .line_underflow_sticky(line_underflow_sticky)
    );

    assign dbg_prefetch_active   = u_scanout.prefetch_active;
    assign dbg_can_request       = u_scanout.can_request;
    assign dbg_state             = {1'b0, u_scanout.fetch_state};
    assign dbg_credits           = {{(16-(LINE_COUNT_LOG2+1)){1'b0}},
                                    u_scanout.u_fetch.credits};
    assign dbg_stale_count       = {4'd0, u_scanout.u_fetch.stale_count};
    assign dbg_outstanding       = {4'd0, u_scanout.u_fetch.outstanding};
    assign dbg_req_y             = {{(16-SRC_Y_W){1'b0}}, u_scanout.u_fetch.req_y};
    assign dbg_req_x             = {{(16-SRC_X_W){1'b0}}, u_scanout.u_fetch.req_x};
    assign dbg_rsp_y             = {{(16-SRC_Y_W){1'b0}}, u_scanout.u_fetch.rsp_y};
    assign dbg_y_src             = {{(16-SRC_Y_W){1'b0}}, u_scanout.u_disp.y_src};
    assign dbg_outstanding_empty = u_scanout.outstanding_empty;

endmodule

`default_nettype wire
