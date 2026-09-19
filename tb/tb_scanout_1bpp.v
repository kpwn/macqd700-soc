// tb_scanout_1bpp.v -- PIXEL-EXACT 1bpp scanout gate, at ANY destination mode.
// ---------------------------------------------------------------------------
// WHY THIS RIG EXISTS.
//
// tb-scanout-frames already runs a "1bpp" scenario, but it paints every byte
// of a source row with the SAME value (0x00 or 0xFF) and varies only per row.
// That makes it structurally blind to the entire HORIZONTAL half of the 1bpp
// path: the sub-byte slice order in pixel_unpack.v, the `x_src >> bpp_shift`
// byte index in scanout_display.v, and the x_lo pipeline that has to stay
// aligned with the line-store read.  A defect in any of them renders a
// uniform row correctly.
//
// It is also pinned to DST 1920x1080, where the Q700's 640x480 source scales
// at N=2.  Every mode change since has been checked against a rig that cannot
// see the mode -- and 788cd2b0 moved the shipping raster to 1280x720, where
// 640x480 scales at N=1 for the first time.
//
// So this rig is deliberately the opposite of that one on both axes:
//
//   * the source pattern varies PER BYTE and per BIT WITHIN the byte, so every
//     one of the 640 source pixels on a row is individually checked;
//   * the destination raster is a PARAMETER, exported to the harness as
//     constant ports (cfg_*), so one harness checks every mode the SoC can be
//     built for and a future mode change only adds a make target.
//
// The scaling factor is NOT told to the DUT: `scale_n` is driven 0, which asks
// place_plan.v to apply its own policy, and the harness re-derives N with an
// independent expression.  A disagreement about N is therefore a failure here
// rather than a shared assumption.
// ---------------------------------------------------------------------------
`default_nettype none

// >>> EVERY DEFAULT BELOW IS THE SHIPPING BUILD'S VALUE, taken from
// >>> rtl/soc/fpga_top_video.vh.  They are NOT the round numbers the older
// >>> video tbs use: the scanner actually ships with SRC_W = 1152, which is
// >>> not a power of two, a 4 MB aperture and ADDR_W = 22.  A tb that quietly
// >>> elaborates 1024x768 into 2 MB is testing a machine nobody builds --
// >>> which is how eight separate harness constants in this tree drifted off
// >>> the design they were meant to gate.
module tb_scanout_1bpp #(
    parameter SRC_W           = 1152,   // fpga_top_video.vh FB_W
    parameter SRC_H           = 1024,   // fpga_top_video.vh FB_H
    // Destination raster + its blanking.  Defaults are the SHIPPING mode
    // (720p60, see rtl/soc/fpga_top_video.vh); -GDST_W=... overrides them.
    parameter DST_W           = 1280,
    parameter DST_H           = 720,
    parameter H_FP            = 110,
    parameter H_SYNC          = 40,
    parameter H_BP            = 220,
    parameter V_FP            = 5,
    parameter V_SYNC          = 5,
    parameter V_BP            = 20,
    parameter ADDR_W          = 22,              // FB_ADDR_W, VRAM_IN_DDR
    parameter FETCH_W         = 32,              // FB_FETCH_W
    parameter FB_MAX_PIXELS   = 32'h0040_0000,   // FB_MAX_PIXELS, VRAM_IN_DDR
    parameter LINE_COUNT_LOG2 = 6                // video_top.v's instantiation
) (
    input  wire                 pclk,
    input  wire                 rst,

    input  wire [ADDR_W-1:0]    fb_base_px,
    input  wire [ADDR_W-1:0]    fb_stride_px,
    input  wire [2:0]           bpp_shift,
    input  wire [2:0]           bytes_per_px,
    input  wire [11:0]          hres,
    input  wire [11:0]          vres,
    input  wire [2:0]           scale_n,
    input  wire                 dafb_live,

    input  wire                 clut_we,
    input  wire [7:0]           clut_waddr,
    input  wire [23:0]          clut_wdata,

    output wire                 fb_rd_en,
    output wire [ADDR_W-1:0]    fb_rd_addr,
    input  wire                 fb_rd_ready,
    input  wire [FETCH_W-1:0]   fb_rd_data,
    input  wire                 fb_rd_valid,

    output wire [23:0]          rgb,
    output wire                 de_out,
    output wire                 hs_out,
    output wire                 vs_out,
    output wire                 line_underflow_sticky,

    output wire [11:0]          hcount,
    output wire [10:0]          vcount,

    // ── Elaborated geometry, exported so ONE harness covers every mode ──
    output wire [15:0]          cfg_dst_w,
    output wire [15:0]          cfg_dst_h,
    output wire [15:0]          cfg_h_total,
    output wire [15:0]          cfg_v_total,
    output wire [15:0]          cfg_src_w,
    output wire [15:0]          cfg_src_h,
    output wire [31:0]          cfg_fb_bytes,
    // What place_plan.v actually committed for the live hres/vres.  The
    // harness re-derives this independently and compares.
    output wire [2:0]           cfg_scale_n
);

    assign cfg_dst_w   = DST_W;
    assign cfg_dst_h   = DST_H;
    assign cfg_h_total = DST_W + H_FP + H_SYNC + H_BP;
    assign cfg_v_total = DST_H + V_FP + V_SYNC + V_BP;
    assign cfg_src_w    = SRC_W;
    assign cfg_src_h    = SRC_H;
    assign cfg_fb_bytes = FB_MAX_PIXELS;
    assign cfg_scale_n = u_scanout.u_disp.plan_scale_n;

    wire vtg_de, vtg_hs, vtg_vs;

    vtg #(
        .H_ACTIVE (DST_W), .H_FP (H_FP), .H_SYNC (H_SYNC), .H_BP (H_BP),
        .V_ACTIVE (DST_H), .V_FP (V_FP), .V_SYNC (V_SYNC), .V_BP (V_BP)
    ) u_vtg (
        .pclk   (pclk),
        .resetn (~rst),
        .hcount (hcount),
        .vcount (vcount),
        .de     (vtg_de),
        .hsync  (vtg_hs),
        .vsync  (vtg_vs)
    );

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
        .line_underflow_sticky (line_underflow_sticky)
    );

endmodule

`default_nettype wire
