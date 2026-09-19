// tb_scanout_display_latency.v -- harness for the FIVE-STAGE LOCKSTEP gate.
// ---------------------------------------------------------------------------
// scanout_display.v's latency ladder is five registered stages from a
// combinational `x_src` to `rgb`, and SIX parallel pipes have to match it
// exactly: de, hs, vs, the border decision, the pixel-valid gate and the boot
// splash.  docs/video_path_review.md S6 records that ladder as "hand-verified"
// -- i.e. its only guarantee was a person having read it once, and it now
// spans three files (scanout_display.v, clut.v, compositor.v).
//
// An off-by-one in any one of those pipes lands precisely at `border_x`: the
// first pixels of a line.  That is a symptom that has actually been reported
// off hardware, and it is a symptom a tb that counts lines or checks frame
// timing cannot see.
//
// This harness exists so the property is MEASURED.  It is deliberately tiny
// and synchronous -- one clock, a synthetic raster, a C++-driven line store
// and palette -- so the C++ side can determine each pipe's depth INDEPENDENTLY
// and name the one that slipped, instead of reporting "some pixels differ".
//
// The raster is small (320x200 total, 256x160 visible) so a scenario is a few
// hundred thousand cycles rather than the tens of millions a 1080p tb needs.
// Nothing here depends on the production geometry: the property under test is
// a pipeline depth, which is geometry-independent by construction.
// ---------------------------------------------------------------------------
`default_nettype none

module tb_scanout_display_latency #(
    parameter SRC_W           = 64,
    parameter SRC_H           = 64,
    parameter DST_W           = 256,
    parameter DST_H           = 160,
    parameter H_TOTAL         = 320,
    parameter V_TOTAL         = 200,
    parameter HS_START        = 270,
    parameter HS_END          = 280,
    parameter VS_START        = 170,
    parameter VS_END          = 175,
    parameter LINE_COUNT_LOG2 = 2,
    parameter SPLASH_SCALE    = 4
) (
    input  wire        pclk,
    input  wire        resetn,

    // Mode / placement inputs, driven straight from C++.
    input  wire [2:0]  bpp_shift,
    input  wire [11:0] hres,
    input  wire [11:0] vres,
    input  wire [2:0]  scale_n,
    input  wire        fetch_direct,
    input  wire        dafb_live,

    // Ring answer, driven from C++ so a single-cycle starvation can be
    // injected at a chosen column (that is how pix_valid_pipe is probed).
    input  wire        disp_ready,

    // Line-store write port.
    input  wire        wr_en,
    input  wire [15:0] wr_addr,
    input  wire [7:0]  wr_d0,
    input  wire [7:0]  wr_d1,
    input  wire [7:0]  wr_d2,

    // Palette write port (same clock here; the real instantiation crosses).
    input  wire        clut_we,
    input  wire [7:0]  clut_waddr,
    input  wire [23:0] clut_wdata,

    // -- Observation ---------------------------------------------------
    // The stage-0 raster state, so C++ can align its model to the same
    // cycle the DUT sampled.
    output wire [11:0] hcount,
    output wire [10:0] vcount,
    output wire        de_in,
    output wire        hs_in,
    output wire        vs_in,
    output wire        sof,

    output wire [23:0] rgb,
    output wire        de_out,
    output wire        hs_out,
    output wire        vs_out,
    output wire        line_underflow_sticky,
    output wire [11:0] disp_y
);

    localparam LINE_MEM_AW = $clog2((1 << LINE_COUNT_LOG2) * SRC_W);

    // -- Synthetic raster ----------------------------------------------
    // Same shape as vtg.v: free-running counters, DE over the visible box,
    // sync pulses outside it, `sof` at (0,0).
    reg [11:0] h_cnt;
    reg [10:0] v_cnt;

    always @(posedge pclk) begin
        if (!resetn) begin
            h_cnt <= 12'd0;
            v_cnt <= 11'd0;
        end else if (h_cnt == H_TOTAL - 1) begin
            h_cnt <= 12'd0;
            v_cnt <= (v_cnt == V_TOTAL - 1) ? 11'd0 : (v_cnt + 11'd1);
        end else begin
            h_cnt <= h_cnt + 12'd1;
        end
    end

    assign hcount = h_cnt;
    assign vcount = v_cnt;
    assign de_in  = (h_cnt < DST_W) && (v_cnt < DST_H);
    assign hs_in  = (h_cnt >= HS_START) && (h_cnt < HS_END);
    assign vs_in  = (v_cnt >= VS_START) && (v_cnt < VS_END);
    assign sof    = (h_cnt == 12'd0) && (v_cnt == 11'd0);

    // -- Ring model -----------------------------------------------------
    // Every source row is resident in slot (row mod LINE_COUNT); `disp_ready`
    // is a tb input so starvation can be injected on demand.
    wire [LINE_COUNT_LOG2-1:0] disp_slot = disp_y[LINE_COUNT_LOG2-1:0];

    scanout_display #(
        .SRC_W           (SRC_W),
        .SRC_H           (SRC_H),
        .DST_W           (DST_W),
        .DST_H           (DST_H),
        .LINE_COUNT_LOG2 (LINE_COUNT_LOG2),
        .SPLASH_SCALE    (SPLASH_SCALE)
    ) u_dut (
        .pclk        (pclk),
        .resetn      (resetn),
        .hcount      (hcount),
        .vcount      (vcount),
        .de_in       (de_in),
        .hs_in       (hs_in),
        .vs_in       (vs_in),
        .sof         (sof),
        .bpp_shift   (bpp_shift),
        .hres        (hres),
        .vres        (vres),
        .scale_n     (scale_n),
        .fetch_direct(fetch_direct),
        .dafb_live   (dafb_live),
        .disp_y      (disp_y),
        .disp_ready  (disp_ready),
        .disp_slot   (disp_slot),
        .credit_return(),
        .wr_en_p0    (wr_en),
        .wr_en_p1    (wr_en),
        .wr_en_p2    (wr_en),
        .wr_addr     (wr_addr[LINE_MEM_AW-1:0]),
        .wr_d0       (wr_d0),
        .wr_d1       (wr_d1),
        .wr_d2       (wr_d2),
        .clut_wclk   (pclk),
        .clut_we     (clut_we),
        .clut_waddr  (clut_waddr),
        .clut_wdata  (clut_wdata),
        .rgb         (rgb),
        .de_out      (de_out),
        .hs_out      (hs_out),
        .vs_out      (vs_out),
        .line_underflow_sticky (line_underflow_sticky)
    );

endmodule

`default_nettype wire
