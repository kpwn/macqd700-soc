// tb_video_top.v -- Verilator wrapper for the HDMI video pipeline unit tb.
//
// Exposes video_top's useful signals as plain wires.  Skips full tri-state
// I2C modelling (Verilator + inout is painful and we don't need to ACK
// every byte -- the i2c_init state machine runs regardless of whether a
// slave ACKs; NAK just bumps a diagnostic counter).  We instead pull SDA
// high permanently and expose the drive-low strobes via hierarchical
// reference so the C++ harness can count I2C bit transitions.
//
// Scale: default video_top (1024x768 -> 1920x1080p60 letterbox).  One
// full-frame simulation is ~2.07 M pclks -- Verilator runs this in a
// couple of seconds on a modern CPU.

`default_nettype none

module tb_video_top #(
    parameter TEST_PATTERN = 0
) (
    input  wire        clk,
    input  wire        rst_n,

    // Pulled up to 1 by the harness; i2c_init drives these low only via
    // its internal oe strobes (pulled out hierarchically below).
    output wire        i2c_scl_drive_low,
    output wire        i2c_sda_drive_low,

    // Video outputs
    output wire [23:0] al9134_d,
    output wire        al9134_hs,
    output wire        al9134_vs,
    output wire        al9134_de,
    output wire        al9134_clk,
    output wire        al9134_resetn,
    output wire [11:0] hcount,
    output wire [10:0] vcount,
    output wire        test_pattern_mode,
    output wire [1:0]  test_pattern_kind,
    output wire        fb_underflow_sticky,

    // Status
    output wire        mmcm_locked,
    output wire        hdmi_i2c_done,

    // VBL pulse — pclk-domain frame-start tick.
    output wire        vbl_pulse_pclk,

    // 256-entry RAMDAC CLUT write port, exposed so the harness can program a
    // palette.  Previously tied off at 0, which was survivable only because
    // this wrapper elaborated video_top with a 24-bit fetch port and took the
    // old direct-colour passthrough (no CLUT lookup at all).  The fetch port
    // is byte-wide at every depth now and this wrapper runs the scanner in its
    // production 8bpp indexed mode, so an all-zero CLUT would render the
    // entire frame black.
    input  wire        clut_we,
    input  wire [7:0]  clut_waddr,
    input  wire [23:0] clut_wdata,

    // VRAM backdoor for the tb
    output wire        vram_rd_en,
    output wire [19:0] vram_rd_addr,
    // video_top's FETCH_W is the DATA width of the VRAM streaming fetch port.
    // The port stays byte-ADDRESSED at every pixel depth but returns the
    // 4-BYTE GROUP starting at the requested address:
    //   [31:24] byte at vram_rd_addr        (always valid, any alignment)
    //   [23:0]  bytes at +1/+2/+3           (valid only when 4-byte aligned)
    // The runtime pixel depth arrives on scanout_bytes_per_px instead (see
    // linebuf_scanout.v's header).  (Was 8 when the port returned one byte,
    // and before that 24, to select the old elaboration-time direct-colour
    // path, which no longer exists.)
    input  wire [31:0] vram_rd_data,
    input  wire        vram_rd_valid,

    // ── Second video_top, PRODUCTION-SHAPED, runtime placement ────────
    // The instance above is the historical fixed-geometry one: its
    // placement inputs are tied to base 0 / stride 1024 / 8bpp /
    // 1024x768, and its ADDR_W/FB_MAX_PIXELS are the small defaults.
    // That configuration cannot express the DAFB modes that actually
    // black out on hardware, because 1024 is exactly SRC_W and every
    // build-time-width gate in the scanout chain happens to pass there.
    //
    // `g2_*` drives a SECOND instance elaborated the way fpga_top_video.vh
    // elaborates the real one under VRAM_IN_DDR (ADDR_W=22, FB_MAX_PIXELS
    // = the 4 MB aperture) with EVERY placement term as a runtime input,
    // so the harness can replay register values read off the live board.
    // The first instance is untouched, so the pre-existing scenarios stay
    // bit-identical.
    input  wire [21:0] g2_fb_base_px,
    input  wire [21:0] g2_fb_stride_px,
    input  wire [2:0]  g2_bpp_shift,
    input  wire [2:0]  g2_bytes_per_px,
    input  wire        g2_depth_supported,
    input  wire [11:0] g2_hres,
    input  wire [11:0] g2_vres,
    input  wire        g2_clut_we,
    input  wire [7:0]  g2_clut_waddr,
    input  wire [23:0] g2_clut_wdata,

    output wire [23:0] g2_rgb,
    output wire        g2_de,
    output wire [11:0] g2_hcount,
    output wire [10:0] g2_vcount,
    output wire        g2_underflow_sticky,
    output wire        g2_vram_rd_en,
    output wire [21:0] g2_vram_rd_addr,
    input  wire [31:0] g2_vram_rd_data,
    input  wire        g2_vram_rd_valid,

    // Placement-gate observability: which term rejected a placement, and
    // what the scanner actually committed for the resident frame.
    output wire        g2_fetch_placement_valid,
    output wire        g2_fetch_stride_sane,
    output wire        g2_fetch_frame_in_mem,
    output wire [21:0] g2_committed_stride,
    output wire [11:0] g2_committed_hres,
    output wire [2:0]  g2_committed_bytes_per_px
);

    // al9134_scl / al9134_sda are inout ports of video_top.  Under the
    // VERILATOR branch video_top always drives them (1 when released,
    // 0 when oe strobe active) so we don't need external driver wires
    // here -- just let the inout resolve naturally.
    wire al9134_scl_w;
    wire al9134_sda_w;

    video_top #(
        .SRC_W   (1024),
        .SRC_H   (768),
        .FETCH_W (32),
        .ADDR_W  (20),
        .TEST_PATTERN(TEST_PATTERN)
    ) u_video (
        .clk_ref_p     (clk),
        .clk_ref_n     (clk),          // VERILATOR branch of mmcm_hdmi
                                       // ignores _n; real FPGA gets diff
        .ext_resetn    (rst_n),
        .vram_clk      (clk),
        .vram_rst      (~rst_n),
        // Coupled scan-out reset (task #194).  This tb has no VRAM read
        // port of its own to reset, but the pin must be connected: leaving
        // it (or dbg_video_snap/dbg_video_place) unconnected is a
        // PINMISSING warning, and this target builds -Wall + warnings-as-
        // errors, so an unconnected output is a BUILD FAILURE, not a lint
        // nit.  That is exactly why tb-video was already red before this
        // change -- see the two dbg_* pins below.
        .vram_side_rst (),
        .dbg_video_snap (),
        .dbg_video_place(),
        .pclk_out      (),
        .mmcm_locked   (mmcm_locked),
        .hdmi_i2c_done (hdmi_i2c_done),
        .fb_underflow_sticky(fb_underflow_sticky),
        .fb_reader_req_count (),
        .fb_reader_rsp_count (),
        .fb_reader_miss_count(),
        .debug_hcount  (hcount),
        .debug_vcount  (vcount),
        .debug_rgb     (),
        .debug_de      (),
        .debug_hs      (),
        .debug_vs      (),
        .vbl_pulse_pclk(vbl_pulse_pclk),
        .al9134_scl    (al9134_scl_w),
        .al9134_sda    (al9134_sda_w),
        .al9134_d      (al9134_d),
        .al9134_hs     (al9134_hs),
        .al9134_vs     (al9134_vs),
        .al9134_de     (al9134_de),
        .al9134_clk    (al9134_clk),
        .al9134_resetn (al9134_resetn),
        .al9134_int    (1'b0),
        .scanout_fb_base_px  (20'd0),
        .scanout_fb_stride_px(20'd1024),
        .scanout_bpp_shift   (3'd0),
        .scanout_bytes_per_px(3'd1),
        .scanout_depth_supported (1'b1),
        .scanout_hres        (12'd1024),
        .scanout_vres        (12'd768),
        .scanout_clut_we     (clut_we),
        .scanout_clut_waddr  (clut_waddr),
        .scanout_clut_wdata  (clut_wdata),
        .vram_rd_en    (vram_rd_en),
        .vram_rd_addr  (vram_rd_addr),
        .vram_rd_data  (vram_rd_data),
        .vram_rd_valid (vram_rd_valid)
    );

    // ── g2: production-shaped instance with runtime placement ─────────
    wire g2_scl_w;
    wire g2_sda_w;

    video_top #(
        .SRC_W        (1024),
        .SRC_H        (768),
        .FETCH_W      (32),
        .ADDR_W       (22),                 // fpga_top_video.vh FB_ADDR_W
        .FB_MAX_PIXELS(32'h0040_0000),      // fpga_top_video.vh, VRAM_IN_DDR
        .TEST_PATTERN (0)
    ) u_video_g2 (
        .clk_ref_p     (clk),
        .clk_ref_n     (clk),
        .ext_resetn    (rst_n),
        .vram_clk      (clk),
        .vram_rst      (~rst_n),
        // Coupled scan-out reset (task #194).  This tb has no VRAM read
        // port of its own to reset, but the pin must be connected: leaving
        // it (or dbg_video_snap/dbg_video_place) unconnected is a
        // PINMISSING warning, and this target builds -Wall + warnings-as-
        // errors, so an unconnected output is a BUILD FAILURE, not a lint
        // nit.  That is exactly why tb-video was already red before this
        // change -- see the two dbg_* pins below.
        .vram_side_rst (),
        .dbg_video_snap (),
        .dbg_video_place(),
        .pclk_out      (),
        .mmcm_locked   (),
        .hdmi_i2c_done (),
        .fb_underflow_sticky(g2_underflow_sticky),
        .fb_reader_req_count (),
        .fb_reader_rsp_count (),
        .fb_reader_miss_count(),
        .debug_hcount  (g2_hcount),
        .debug_vcount  (g2_vcount),
        .debug_rgb     (g2_rgb),
        .debug_de      (g2_de),
        .debug_hs      (),
        .debug_vs      (),
        .vbl_pulse_pclk(),
        .al9134_scl    (g2_scl_w),
        .al9134_sda    (g2_sda_w),
        .al9134_d      (),
        .al9134_hs     (),
        .al9134_vs     (),
        .al9134_de     (),
        .al9134_clk    (),
        .al9134_resetn (),
        .al9134_int    (1'b0),
        .scanout_fb_base_px  (g2_fb_base_px),
        .scanout_fb_stride_px(g2_fb_stride_px),
        .scanout_bpp_shift   (g2_bpp_shift),
        .scanout_bytes_per_px(g2_bytes_per_px),
        .scanout_depth_supported (g2_depth_supported),
        .scanout_hres        (g2_hres),
        .scanout_vres        (g2_vres),
        .scanout_clut_we     (g2_clut_we),
        .scanout_clut_waddr  (g2_clut_waddr),
        .scanout_clut_wdata  (g2_clut_wdata),
        .vram_rd_en    (g2_vram_rd_en),
        .vram_rd_addr  (g2_vram_rd_addr),
        .vram_rd_data  (g2_vram_rd_data),
        .vram_rd_valid (g2_vram_rd_valid)
    );

    // debug RGB/control/count outputs are aligned with the registered
    // al9134 pins, so the harness can assert on them without reaching into
    // the pin bundle or compensating for the output pipeline latency.
    assign g2_fetch_placement_valid = u_video_g2.u_scanout.fetch_placement_valid;
    assign g2_fetch_stride_sane     = u_video_g2.u_scanout.fetch_stride_sane;
    assign g2_fetch_frame_in_mem    = u_video_g2.u_scanout.fetch_frame_in_mem;
    assign g2_committed_stride      = u_video_g2.fb_stride_px;
    assign g2_committed_hres        = u_video_g2.hres;
    assign g2_committed_bytes_per_px= u_video_g2.bytes_per_px;

    // Pull out the drive-low strobes from the i2c_init instance so the
    // tb can count transitions without an actual ACKing slave.
    assign i2c_scl_drive_low = u_video.u_i2c.scl_oe;
    assign i2c_sda_drive_low = u_video.u_i2c.sda_oe;
    assign test_pattern_mode = (TEST_PATTERN != 0);
    assign test_pattern_kind =
        (TEST_PATTERN == 0) ? 2'd0 :
        (TEST_PATTERN == 1) ? 2'd1 :
                              2'd2;

endmodule

`default_nettype wire
