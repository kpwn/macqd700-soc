// tb_dafb_mode_matrix.v -- WHOLE-MATRIX DAFB mode/depth scanout harness.
// ---------------------------------------------------------------------------
// Every other video tb in this repo pins ONE geometry (usually 640x480, the
// mode the board happened to be in) and one or two depths.  That is exactly
// the shape of coverage that lets "640x480 works" stand in for "the runtime
// geometry plumbing works" -- a mode that works because it equals a default is
// indistinguishable from a mode that works because the plumbing works.
//
// This wrapper exists to walk the FULL (monitor mode x pixel depth) space of
// the Quadra 700 DAFB in one rig:
//
//   * `monitor_sense` is an INPUT, not a hardcoded 7'h06, so the mono-monitor
//     CLUT branch (video.v `mono_monitor`, MAME dafb.cpp:760) is reachable.
//   * the CLUT is loaded through the REAL AC842 RAMDAC write protocol on
//     video.v's AXI-Lite port and exported to the scanner through video.v's
//     own clut_we/clut_waddr/clut_wdata channel -- so the palette path is
//     under test, not bypassed.
//   * `dafb_live` is wired for real from scanout_placement_sync into
//     linebuf_scanout, so "the boot splash never retires" is an observable
//     failure of a mode rather than an invisible one (tb_dafb_24bpp_capacity
//     ties it to 1'b1 and can never see it).
//   * SRC_W / SRC_H / FB_MAX_PIXELS / ADDR_W are PARAMETERS, so the same
//     harness can elaborate the shipping 1024x768 scanner and a widened one
//     and show which modes each admits.
//
// The VRAM model returns an address-derived pattern (see `vram_byte`), not a
// constant: a constant framebuffer cannot distinguish "the right byte reached
// the right pixel" from "some byte reached every pixel", which is the failure
// mode the harness is here to exclude.
// ---------------------------------------------------------------------------
`default_nettype none

module tb_dafb_mode_matrix #(
    // Scanner elaborated bounds.  Shipping build values are the defaults
    // (rtl/soc/fpga_top_video.vh, VRAM_IN_DDR off).
`ifdef MM_SRC_W
    parameter SRC_W         = `MM_SRC_W,
`else
    parameter SRC_W         = 1024,
`endif
`ifdef MM_SRC_H
    parameter SRC_H         = `MM_SRC_H,
`else
    parameter SRC_H         = 768,
`endif
`ifdef MM_FB_MAX_PIXELS
    parameter FB_MAX_PIXELS = `MM_FB_MAX_PIXELS,
`else
    parameter FB_MAX_PIXELS = 32'h0020_0000,   // 2 MiB URAM aperture
`endif
`ifdef MM_ADDR_W
    parameter ADDR_W        = `MM_ADDR_W,
`else
    parameter ADDR_W        = 21,
`endif
    parameter DST_W         = 1920,
    parameter DST_H         = 1080,
    parameter FETCH_W       = 32,
    parameter MEM_LATENCY   = 31
) (
    input  wire        pclk,
    input  wire        rst,

    // AXI4-Lite write channel into the DAFB register shim.
    input  wire [31:0] dafb_awaddr,
    input  wire        dafb_awvalid,
    output wire        dafb_awready,
    input  wire [31:0] dafb_wdata,
    input  wire [3:0]  dafb_wstrb,
    input  wire        dafb_wvalid,
    output wire        dafb_wready,
    output wire        dafb_bvalid,
    input  wire        dafb_bready,

    // Monitor sense code (MAME dafb.cpp monitor_config list).
    input  wire [6:0]  monitor_sense,

    // Video timing, driven by the harness.
    input  wire [11:0] hcount,
    input  wire [10:0] vcount,
    input  wire        de_in,
    input  wire        hs_in,
    input  wire        vs_in,

    output wire [23:0] scanout_rgb,
    output wire        scanout_de,
    output wire        line_underflow_sticky,

    // ── DAFB shim decode (what Mac OS asked for) ─────────────────────
    output wire [31:0] dbg_shim_base,
    output wire [31:0] dbg_shim_stride,
    output wire [2:0]  dbg_shim_bytes_per_px,
    output wire [2:0]  dbg_shim_bpp_shift,
    output wire        dbg_shim_depth_supported,
    output wire [11:0] dbg_shim_hres,
    output wire [11:0] dbg_shim_vres,

    // ── placement admission gate ─────────────────────────────────────
    output wire        dbg_ps_depth_ok,
    output wire        dbg_ps_stride_sane,
    output wire        dbg_ps_in_range,
    output wire [31:0] dbg_ps_frame_last_addr,
    output wire [31:0] dbg_ps_min_stride,

    // ── what actually got committed ──────────────────────────────────
    output wire [31:0] dbg_committed_base,
    output wire [31:0] dbg_committed_stride,
    output wire [2:0]  dbg_committed_bytes_per_px,
    output wire [2:0]  dbg_committed_bpp_shift,
    output wire [11:0] dbg_committed_hres,
    output wire [11:0] dbg_committed_vres,
    output wire [2:0]  dbg_scale_n,
    output wire        dbg_dafb_live,

    // ── fetch engine ─────────────────────────────────────────────────
    output wire        dbg_fetch_direct,
    output wire        dbg_fetch_wide,
    output wire        dbg_fetch_stride_sane,
    output wire        dbg_fetch_frame_in_mem,
    output wire        dbg_can_request,
    output wire        dbg_req_addr_oob,
    output wire [15:0] dbg_fetch_row_px_last,
    output wire [15:0] dbg_fetch_row_y_last,
    output wire        dbg_fb_rd_en,
    output wire        dbg_fb_rd_valid,

    // ── CLUT export observability (the tint referee) ─────────────────
    output wire        dbg_clut_we,
    output wire [7:0]  dbg_clut_waddr,
    output wire [23:0] dbg_clut_wdata
);

    wire [31:0] shim_base;
    wire [31:0] shim_stride;
    wire [2:0]  shim_bpp_shift;
    wire [2:0]  shim_bytes_per_px;
    wire        shim_depth_supported;
    wire [11:0] shim_hres;
    wire [11:0] shim_vres;
    wire        shim_clut_we;
    wire [7:0]  shim_clut_waddr;
    wire [23:0] shim_clut_wdata;

    video u_dafb (
        .clk          (pclk),
        .rst          (rst),
        .s_axi_awaddr (dafb_awaddr),
        .s_axi_awvalid(dafb_awvalid),
        .s_axi_awready(dafb_awready),
        .s_axi_wdata  (dafb_wdata),
        .s_axi_wstrb  (dafb_wstrb),
        .s_axi_wvalid (dafb_wvalid),
        .s_axi_wready (dafb_wready),
        .s_axi_bresp  (),
        .s_axi_bvalid (dafb_bvalid),
        .s_axi_bready (dafb_bready),
        .s_axi_araddr (32'd0),
        .s_axi_arvalid(1'b0),
        .s_axi_arready(),
        .s_axi_rdata  (),
        .s_axi_rresp  (),
        .s_axi_rvalid (),
        .s_axi_rready (1'b0),
        .fb_base_px   (shim_base),
        .fb_stride_px (shim_stride),
        .fb_bpp_reg   (),
        .bpp_shift    (shim_bpp_shift),
        .fb_bytes_per_px (shim_bytes_per_px),
        .depth_supported (shim_depth_supported),
        .hres         (shim_hres),
        .vres         (shim_vres),
        .clut_we      (shim_clut_we),
        .clut_waddr   (shim_clut_waddr),
        .clut_wdata   (shim_clut_wdata),
        .irq          (),
        .pll_pixel_clock(),
        .scsi0_ctrl_out(),
        .scsi0_drq_in (1'b0),
        .frame_tick   (1'b0),
        .monitor_sense(monitor_sense)
    );

    assign dbg_shim_base            = shim_base;
    assign dbg_shim_stride          = shim_stride;
    assign dbg_shim_bytes_per_px    = shim_bytes_per_px;
    assign dbg_shim_bpp_shift       = shim_bpp_shift;
    assign dbg_shim_depth_supported = shim_depth_supported;
    assign dbg_shim_hres            = shim_hres;
    assign dbg_shim_vres            = shim_vres;
    assign dbg_clut_we              = shim_clut_we;
    assign dbg_clut_waddr           = shim_clut_waddr;
    assign dbg_clut_wdata           = shim_clut_wdata;

    wire frame_start = (hcount == 12'd0) && (vcount == 11'd0);

    wire [ADDR_W-1:0] sc_base;
    wire [ADDR_W-1:0] sc_stride;
    wire [2:0]        sc_bpp_shift;
    wire [2:0]        sc_bytes_per_px;
    wire [11:0]       sc_hres;
    wire [11:0]       sc_vres;
    wire [2:0]        sc_scale_n;
    wire              sc_dafb_live;

    scanout_placement_sync #(
        .ADDR_W       (ADDR_W),
        .SRC_W        (SRC_W),
        .SRC_H        (SRC_H),
        .FB_MAX_PIXELS(FB_MAX_PIXELS),
        .FB_BASE_PX   (0),
        .FB_STRIDE_PX (SRC_W),
        .DST_W        (DST_W),
        .DST_H        (DST_H)
    ) u_placement (
        .pclk                    (pclk),
        .rst                     (rst),
        .frame_start             (frame_start),
        .scanout_fb_base_px      (shim_base[ADDR_W-1:0]),
        .scanout_fb_stride_px    (shim_stride[ADDR_W-1:0]),
        .scanout_bpp_shift       (shim_bpp_shift),
        .scanout_bytes_per_px    (shim_bytes_per_px),
        .scanout_depth_supported (shim_depth_supported),
        .scanout_hres            (shim_hres),
        .scanout_vres            (shim_vres),
        .fb_base_px              (sc_base),
        .fb_stride_px            (sc_stride),
        .bpp_shift               (sc_bpp_shift),
        .bytes_per_px            (sc_bytes_per_px),
        .hres                    (sc_hres),
        .vres                    (sc_vres),
        .reject_reason             (),
        .placement_rejected_sticky (),
        .reject_base_px            (),
        .reject_stride_px          (),
        .reject_hres_px            (),
        .reject_vres_px            (),
        .reject_reason_latched     (),
        .scale_n                 (sc_scale_n),
        .dafb_live               (sc_dafb_live)
    );

    assign dbg_ps_depth_ok          = u_placement.depth_ok_stable;
    assign dbg_ps_stride_sane       = u_placement.stable_stride_sane;
    assign dbg_ps_in_range          = u_placement.stable_placement_in_range;
    assign dbg_ps_frame_last_addr   = u_placement.frame_last_addr;
    assign dbg_ps_min_stride        = {{(32-ADDR_W){1'b0}}, u_placement.min_stride_bytes};
    assign dbg_committed_base       = {{(32-ADDR_W){1'b0}}, sc_base};
    assign dbg_committed_stride     = {{(32-ADDR_W){1'b0}}, sc_stride};
    assign dbg_committed_bytes_per_px = sc_bytes_per_px;
    assign dbg_committed_bpp_shift  = sc_bpp_shift;
    assign dbg_committed_hres       = sc_hres;
    assign dbg_committed_vres       = sc_vres;
    assign dbg_scale_n              = sc_scale_n;
    assign dbg_dafb_live            = sc_dafb_live;

    // ── Fetch port + address-derived VRAM model ──────────────────────
    wire               fb_rd_en;
    wire [ADDR_W-1:0]  fb_rd_addr;
    reg                fb_rd_valid;
    reg  [FETCH_W-1:0] fb_rd_data;
    wire               fb_rd_ready = 1'b1;

    // Address-derived pattern.  Must match the C++ model byte-for-byte.
    function [7:0] vram_byte;
        input [ADDR_W+1:0] a;
        begin
            vram_byte = a[7:0] + (a[15:8] * 8'd3) + (a[ADDR_W-1:16] * 8'd7) + 8'd17;
        end
    endfunction

    wire [ADDR_W+1:0] ra = {2'b00, fb_rd_addr};
    wire [FETCH_W-1:0] rword = {vram_byte(ra),     vram_byte(ra + 1),
                                vram_byte(ra + 2), vram_byte(ra + 3)};

    localparam MAXLAT = 64;
    reg               lat_v [0:MAXLAT-1];
    reg [FETCH_W-1:0] lat_d [0:MAXLAT-1];
    integer li;
    always @(posedge pclk) begin
        if (rst) begin
            fb_rd_valid <= 1'b0;
            fb_rd_data  <= {FETCH_W{1'b0}};
            for (li = 0; li < MAXLAT; li = li + 1) begin
                lat_v[li] <= 1'b0;
                lat_d[li] <= {FETCH_W{1'b0}};
            end
        end else begin
            fb_rd_valid <= lat_v[MEM_LATENCY-1];
            fb_rd_data  <= lat_d[MEM_LATENCY-1];
            for (li = MAXLAT-1; li > 0; li = li - 1) begin
                lat_v[li] <= lat_v[li-1];
                lat_d[li] <= lat_d[li-1];
            end
            lat_v[0] <= fb_rd_en;
            lat_d[0] <= rword;
        end
    end

    linebuf_scanout #(
        .SRC_W           (SRC_W),
        .SRC_H           (SRC_H),
        .FB_MAX_PIXELS   (FB_MAX_PIXELS),
        .DST_W           (DST_W),
        .DST_H           (DST_H),
        .FETCH_W         (FETCH_W),
        .ADDR_W          (ADDR_W),
        .LINE_COUNT_LOG2 (6)
    ) u_scanout (
        .pclk        (pclk),
        .resetn      (~rst),
        .hcount      (hcount),
        .vcount      (vcount),
        .de_in       (de_in),
        .hs_in       (hs_in),
        .vs_in       (vs_in),
        .fb_base_px  (sc_base),
        .fb_stride_px(sc_stride),
        .bpp_shift   (sc_bpp_shift),
        .bytes_per_px(sc_bytes_per_px),
        .hres        (sc_hres),
        .vres        (sc_vres),
        .scale_n     (sc_scale_n),
        .dafb_live   (sc_dafb_live),
        .clut_wclk   (pclk),
        .clut_we     (shim_clut_we),
        .clut_waddr  (shim_clut_waddr),
        .clut_wdata  (shim_clut_wdata),
        .fb_rd_en    (fb_rd_en),
        .fb_rd_addr  (fb_rd_addr),
        .fb_rd_ready (fb_rd_ready),
        .fb_rd_data  (fb_rd_data),
        .fb_rd_valid (fb_rd_valid),
        .rgb         (scanout_rgb),
        .de_out      (scanout_de),
        .hs_out      (),
        .vs_out      (),
        .line_underflow_sticky(line_underflow_sticky)
    );

    assign dbg_fetch_direct       = u_scanout.fetch_direct;
    assign dbg_fetch_wide         = u_scanout.fetch_wide;
    assign dbg_fetch_stride_sane  = u_scanout.fetch_stride_sane;
    assign dbg_fetch_frame_in_mem = u_scanout.fetch_frame_in_mem;
    assign dbg_can_request        = u_scanout.can_request;
    assign dbg_req_addr_oob       = u_scanout.req_addr_oob;
    assign dbg_fetch_row_px_last  = {6'd0, u_scanout.u_fetch.fetch_row_px_last};
    assign dbg_fetch_row_y_last   = {6'd0, u_scanout.u_fetch.fetch_row_y_last};
    assign dbg_fb_rd_en           = fb_rd_en;
    assign dbg_fb_rd_valid        = fb_rd_valid;

endmodule

`default_nettype wire
