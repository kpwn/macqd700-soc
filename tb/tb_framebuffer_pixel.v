// tb_framebuffer_pixel.v — CPU -> VRAM -> fb_reader -> scaler pixel-exact tb
//
// Sibling of tb_vram_scaler_firstlight.v, but stripped to a pixel-exact
// geometry: 64x64 @ 8bpp, SCALE 1:1, no border, ACTIVE = DST = SRC.  The
// companion harness (tb_framebuffer_pixel.cpp) programs the real 256-entry
// AC842 RAMDAC CLUT via video.v's register interface (T9), writes a known
// pattern through the real URAM-backed vram slave, walks one full DST_W x
// DST_H scanout, captures scanout_rgb into a framebuffer image, and
// asserts each output pixel pixel-exact against the programmed palette
// entry for pattern(x,y) -- across the full 0x00-0xFF index range, not
// just the low nibble (see scenario-D in the .cpp, which specifically
// exercises indices above 15).  Any blue-line / steady-state VRAM-fetch
// regression in the fb_reader pipeline fails here loudly instead of
// showing up as a single line on HDMI hardware.
//
// Why separate from tb_vram_scaler_firstlight:
//   firstlight proves the pipeline MOVES pixels (fragmented-read /
//   address-trace equivalence), but never asserts per-output-RGB against
//   a CPU-written pattern and never captures a full frame.  This tb is
//   the missing "visual ground truth" gate.

`default_nettype none

module tb_framebuffer_pixel (
    input  wire         pclk,
    input  wire         vram_clk,
    input  wire         rst,

    // DAFB register-shim port (CPU writes base/stride/BPP/CLUT here).
    input  wire [31:0]  dafb_awaddr,
    input  wire         dafb_awvalid,
    output wire         dafb_awready,
    input  wire [31:0]  dafb_wdata,
    input  wire [3:0]   dafb_wstrb,
    input  wire         dafb_wvalid,
    output wire         dafb_wready,
    output wire [1:0]   dafb_bresp,
    output wire         dafb_bvalid,
    input  wire         dafb_bready,
    input  wire [31:0]  dafb_araddr,
    input  wire         dafb_arvalid,
    output wire         dafb_arready,
    output wire [31:0]  dafb_rdata,
    output wire [1:0]   dafb_rresp,
    output wire         dafb_rvalid,
    input  wire         dafb_rready,

    // VRAM AXI slave — VRAM-NATIVE port.  Writes/reads hit the vram
    // slave directly, using vram.v's little-endian lane convention
    // (byte at lowest address in wdata[7:0] of the selected 32-bit lane).
    // Used by the byte-granular per-lane writes in scenario-A.
    input  wire [3:0]   vram_awid,
    input  wire [31:0]  vram_awaddr,
    input  wire [7:0]   vram_awlen,
    input  wire [2:0]   vram_awsize,
    input  wire [1:0]   vram_awburst,
    input  wire         vram_awvalid,
    output wire         vram_awready,
    input  wire [127:0] vram_wdata,
    input  wire [15:0]  vram_wstrb,
    input  wire         vram_wlast,
    input  wire         vram_wvalid,
    output wire         vram_wready,
    output wire [3:0]   vram_bid,
    output wire [1:0]   vram_bresp,
    output wire         vram_bvalid,
    input  wire         vram_bready,

    input  wire [3:0]   vram_arid,
    input  wire [31:0]  vram_araddr,
    input  wire [7:0]   vram_arlen,
    input  wire [2:0]   vram_arsize,
    input  wire [1:0]   vram_arburst,
    input  wire         vram_arvalid,
    output wire         vram_arready,
    output wire [3:0]   vram_rid,
    output wire [127:0] vram_rdata,
    output wire [1:0]   vram_rresp,
    output wire         vram_rlast,
    output wire         vram_rvalid,
    input  wire         vram_rready,

    // CPU-SIDE port — wired straight through to the vram slave.  This
    // used to pass through a vram_cpu_byteswap shim that reordered bytes
    // within each 32-bit lane; that module was removed from the design
    // and then from the tree (2026-09-03), so the path is now what
    // fpga_top actually builds.  Scenario-B still emulates what
    // axi_narrow_to_wide + LSU (68k big-endian byte lanes) drive at the
    // xbar S3 boundary in production; it just does so without a shim the
    // production design does not contain.
    input  wire [3:0]   vram_cpu_awid,
    input  wire [31:0]  vram_cpu_awaddr,
    input  wire [7:0]   vram_cpu_awlen,
    input  wire [2:0]   vram_cpu_awsize,
    input  wire [1:0]   vram_cpu_awburst,
    input  wire         vram_cpu_awvalid,
    output wire         vram_cpu_awready,
    input  wire [127:0] vram_cpu_wdata,
    input  wire [15:0]  vram_cpu_wstrb,
    input  wire         vram_cpu_wlast,
    input  wire         vram_cpu_wvalid,
    output wire         vram_cpu_wready,
    output wire [3:0]   vram_cpu_bid,
    output wire [1:0]   vram_cpu_bresp,
    output wire         vram_cpu_bvalid,
    input  wire         vram_cpu_bready,

    input  wire [3:0]   vram_cpu_arid,
    input  wire [31:0]  vram_cpu_araddr,
    input  wire [7:0]   vram_cpu_arlen,
    input  wire [2:0]   vram_cpu_arsize,
    input  wire [1:0]   vram_cpu_arburst,
    input  wire         vram_cpu_arvalid,
    output wire         vram_cpu_arready,
    output wire [3:0]   vram_cpu_rid,
    output wire [127:0] vram_cpu_rdata,
    output wire [1:0]   vram_cpu_rresp,
    output wire         vram_cpu_rlast,
    output wire         vram_cpu_rvalid,
    input  wire         vram_cpu_rready,

    // Video-timing interface (harness drives a simple 64x64 frame walk).
    input  wire [11:0]  hcount,
    input  wire [10:0]  vcount,
    input  wire         de_in,
    input  wire         hs_in,
    input  wire         vs_in,

    // Capture probes — the harness asserts against scanout_rgb at
    // (hcount, vcount) when scanout_de is high.
    output wire [23:0]  scanout_rgb,
    output wire         scanout_de,
    output wire         fb_underflow_sticky,
    output wire [31:0]  fb_base_px_out,
    output wire [31:0]  fb_stride_px_out,
    output wire [31:0]  fb_bpp_reg_out,
    // Runtime-depth observability: what the DAFB shim decoded, and what the
    // scanner actually latched for the frame currently resident in line_mem.
    output wire [2:0]   fb_bytes_per_px_out,
    output wire         depth_supported_out,
    output wire [2:0]   committed_bytes_per_px,
    output wire         dbg_fetch_direct,
    // Latched wide-fetch mode: 1 = one aligned 4-byte request per 24bpp
    // pixel, 0 = the three-byte-per-pixel fallback (or any indexed depth).
    // Exposed so the tb can POSITIVELY assert which fetch shape ran -- both
    // shapes render identical pixels, so without this a silently-disabled
    // wide path would leave the whole suite green and prove nothing.
    output wire         dbg_fetch_wide,
    output wire [20:0]  committed_fb_stride_px,
    output wire [20:0]  committed_fb_base_px,
    // LIVE (non-sticky) restatement of linebuf_scanout's underflow condition.
    // line_underflow_sticky can never distinguish "one frame of black during a
    // mode switch, which real hardware also shows" from "steady-state
    // starvation", because it latches forever.  The harness counts pulses of
    // this per frame instead, so a switch artifact and a persistent failure
    // are separable.
    output wire         dbg_line_underflow_ev,

    // Debug probes for bring-up instrumentation.
    output wire         dbg_fb_rd_en,
    output wire [20:0]  dbg_fb_rd_addr,
    output wire         dbg_fb_rd_ready,
    output wire         dbg_fb_rd_valid,
    // 4-byte streaming-read group ([31:24] is the byte at the requested
    // address; the lower three bytes are only meaningful for a 4-byte
    // aligned request).  Widened with the read port — the harness prints
    // the whole group so an unexpected lane is visible in the trace.
    output wire [31:0]  dbg_fb_rd_data,
    output wire         dbg_vram_rd_en,
    output wire [20:0]  dbg_vram_rd_addr,
    output wire         dbg_vram_rd_valid,
    output wire [31:0]  dbg_vram_rd_data,
    output wire         dbg_line_valid0,
    output wire         dbg_prefetch_active,
    output wire         dbg_display_line_ready,
    output wire [5:0]   dbg_y_src,
    output wire [5:0]   dbg_x_src,
    output wire [5:0]   dbg_rsp_x,
    output wire [5:0]   dbg_rsp_y,
    output wire         dbg_fb_reader_underflow,
    output wire         dbg_line_underflow,
    output wire         dbg_pvp1,
    output wire         dbg_pvp2,
    output wire         dbg_restart_pending,
    output wire         dbg_fb_rd_valid2
);

    // 64x64 pixel-exact geometry.  No border, no scale — DST = SRC.
    localparam SRC_W      = 64;
    localparam SRC_H      = 64;
    localparam DST_W      = 64;
    localparam DST_H      = 64;
    localparam ACTIVE_W   = 64;
    localparam ACTIVE_H   = 64;
    localparam BORDER_X   = 0;
    localparam BORDER_Y   = 0;
    localparam SCALE_NUM  = 1;
    localparam SCALE_DEN  = 1;
    // VRAM pixel packing / read-port ADDRESS granularity: 1 byte per index.
    // Unchanged by the read-data widening — this still sizes vram.v's
    // BYTES_PER_PX / PX_PER_WORD / N_PIXELS.
    localparam VRAM_BPP   = 8;
    // Width of the VRAM streaming read-port DATA bus.  The port returns the
    // 4-BYTE GROUP starting at the requested address, so this is 32 and is
    // NOT a pixel depth (linebuf_scanout.v hard-errors on anything else).
    localparam FETCH_W    = 32;
    // ADDR_W must cover the full 2 MiB vram.v VRAM_BYTES aperture (the
    // default u_vram below does not override VRAM_BYTES), matching the
    // fpga_top_video.vh production FB_ADDR_W widening — 20 bits could
    // not even represent a fb_base_px >= 0x100000 (bit 20 falls off).
    // VRAM_ADDR_W must match vram.v's own PX_ADDR_W at this
    // configuration (FB_WIDTH_PX=256, FB_HEIGHT_PX=80, BPP=8, default
    // VRAM_BYTES=2MiB): PX_ADDR_SPAN = max(256*80, 2MiB/1) = 2,097,152
    // pixels -> 21 bits.
    localparam ADDR_W     = 21;
    localparam VRAM_ADDR_W = 21;

    wire [31:0]  shim_fb_base_px;
    wire [31:0]  shim_fb_stride_px;
    wire [31:0]  shim_fb_bpp_reg;
    wire         shim_clut_we;
    wire [7:0]   shim_clut_waddr;
    wire [23:0]  shim_clut_wdata;
    // Depth channel taken LIVE from the DAFB shim rather than tied off, so
    // the harness switches pixel depth the way Mac OS does: by writing the
    // AC842 PCBR register.  That is what makes the 8 -> 24 -> 8 scenario a
    // real runtime-switch test instead of an elaboration-time one.
    wire [2:0]   shim_bpp_shift;
    wire [2:0]   shim_bytes_per_px;
    wire         shim_depth_supported;

    video u_dafb (
        .clk          (vram_clk),
        .rst          (rst),
        .s_axi_awaddr (dafb_awaddr),
        .s_axi_awvalid(dafb_awvalid),
        .s_axi_awready(dafb_awready),
        .s_axi_wdata  (dafb_wdata),
        .s_axi_wstrb  (dafb_wstrb),
        .s_axi_wvalid (dafb_wvalid),
        .s_axi_wready (dafb_wready),
        .s_axi_bresp  (dafb_bresp),
        .s_axi_bvalid (dafb_bvalid),
        .s_axi_bready (dafb_bready),
        .s_axi_araddr (dafb_araddr),
        .s_axi_arvalid(dafb_arvalid),
        .s_axi_arready(dafb_arready),
        .s_axi_rdata  (dafb_rdata),
        .s_axi_rresp  (dafb_rresp),
        .s_axi_rvalid (dafb_rvalid),
        .s_axi_rready (dafb_rready),
        .fb_base_px   (shim_fb_base_px),
        .fb_stride_px (shim_fb_stride_px),
        .fb_bpp_reg   (shim_fb_bpp_reg),
        .bpp_shift    (shim_bpp_shift),
        .fb_bytes_per_px (shim_bytes_per_px),
        .depth_supported (shim_depth_supported),
        .hres         (),
        .vres         (),
        .clut_we      (shim_clut_we),
        .clut_waddr   (shim_clut_waddr),
        .clut_wdata   (shim_clut_wdata),
        .irq          (),
        .pll_pixel_clock(),
        .scsi0_ctrl_out(),
        .scsi0_drq_in (1'b0),
        // This tb exercises pixel/placement geometry, not vblank cadence.
        .frame_tick   (1'b0),
        // Monitor sense: pin the historical MONITOR_TYPE default (7'h06).
        .monitor_sense(7'h06)
    );

    assign fb_base_px_out   = shim_fb_base_px;
    assign fb_stride_px_out = shim_fb_stride_px;
    assign fb_bpp_reg_out   = shim_fb_bpp_reg;
    assign fb_bytes_per_px_out    = shim_bytes_per_px;
    assign depth_supported_out    = shim_depth_supported;
    assign committed_bytes_per_px = scanout_bytes_per_px;
    assign dbg_fetch_direct       = u_scanout.fetch_direct;
    assign dbg_fetch_wide         = u_scanout.fetch_wide;
    assign committed_fb_stride_px = scanout_fb_stride_px;
    assign committed_fb_base_px   = scanout_fb_base_px;
    assign dbg_line_underflow_ev  = u_scanout.u_disp.underflow_armed
                                  & u_scanout.u_disp.rd_req
                                  & ~u_scanout.display_line_ready;

    wire frame_start = (hcount == 12'd0) && (vcount == 11'd0);
    wire [ADDR_W-1:0] scanout_fb_base_px;
    wire [ADDR_W-1:0] scanout_fb_stride_px;
    wire [2:0]        scanout_bpp_shift;
    wire [2:0]        scanout_bytes_per_px;

    scanout_placement_sync #(
        .ADDR_W       (ADDR_W),
        .SRC_W        (SRC_W),
        .SRC_H        (SRC_H),
        // Match vram.v's 2 MiB VRAM_BYTES aperture (default, not
        // overridden on u_vram below), NOT the default SRC_W*SRC_H —
        // otherwise a fb_base_px in the upper half of the 2 MiB VRAM
        // aperture gets rejected by the placement range gate before it
        // ever reaches the scanner (see Bug 2 / scenario C).
        .FB_MAX_PIXELS(32'h0020_0000),
        .FB_BASE_PX   (0),
        .FB_STRIDE_PX (SRC_W),
        .DST_W        (DST_W),
        .DST_H        (DST_H)
    ) u_placement (
        .pclk                 (pclk),
        .rst                  (rst),
        .frame_start          (frame_start),
        .scanout_fb_base_px   (shim_fb_base_px[ADDR_W-1:0]),
        .scanout_fb_stride_px (shim_fb_stride_px[ADDR_W-1:0]),
        .scanout_bpp_shift    (shim_bpp_shift),
        .scanout_bytes_per_px (shim_bytes_per_px),
        .scanout_depth_supported (shim_depth_supported),
        .scanout_hres         (12'd64),
        .scanout_vres         (12'd64),
        .fb_base_px           (scanout_fb_base_px),
        .fb_stride_px         (scanout_fb_stride_px),
        .bpp_shift            (scanout_bpp_shift),
        .bytes_per_px         (scanout_bytes_per_px),
        .hres                 (),
        .vres                 (),
        .reject_reason             (),
        .placement_rejected_sticky (),
        .reject_base_px            (),
        .reject_stride_px          (),
        .reject_hres_px            (),
        .reject_vres_px            (),
        .reject_reason_latched     (),
        .scale_n              (),
        // These tbs cover the normal video path only; the boot-splash
        // switchover is exercised in tb_scanout_placement_sync /
        // tb_scanout_frames.
        .dafb_live            ()
    );

    wire        fb_rd_en;
    wire [ADDR_W-1:0] fb_rd_addr;
    wire        fb_rd_ready;
    // 4-byte group per request (see linebuf_scanout.v / vram.v headers).
    wire [FETCH_W-1:0] scaler_fb_rd_data;
    wire        scaler_fb_rd_valid;
    wire [FETCH_W-1:0] real_vram_rd_data;
    wire        real_vram_rd_valid;
    wire        vram_rd_en;
    wire [ADDR_W-1:0] vram_rd_addr;
    wire        fb_reader_underflow_sticky;
    wire        line_underflow_sticky;

    assign fb_underflow_sticky = fb_reader_underflow_sticky | line_underflow_sticky;
    // Debug: break out which sticky fired for bring-up bisection.
    assign dbg_fb_reader_underflow = fb_reader_underflow_sticky;
    assign dbg_line_underflow      = line_underflow_sticky;
    // pix_valid_pipe moved into the compositor when scan-out stages 9-12 were
    // split out of scanout_display.v (docs/video_path_review.md S4.1); the
    // pipe itself is unchanged, only its owner.
    assign dbg_pvp1                = u_scanout.u_disp.u_compositor.pix_valid_pipe[0];
    assign dbg_pvp2                = u_scanout.u_disp.u_compositor.pix_valid_pipe[1];
    assign dbg_restart_pending     = u_scanout.in_resync;
    assign dbg_fb_rd_valid2        = scaler_fb_rd_valid;

    assign dbg_fb_rd_en      = fb_rd_en;
    assign dbg_fb_rd_addr    = fb_rd_addr;
    assign dbg_fb_rd_ready   = fb_rd_ready;
    assign dbg_fb_rd_valid   = scaler_fb_rd_valid;
    assign dbg_fb_rd_data    = scaler_fb_rd_data;
    assign dbg_vram_rd_en    = vram_rd_en;
    assign dbg_vram_rd_addr  = vram_rd_addr;
    assign dbg_vram_rd_valid = real_vram_rd_valid;
    assign dbg_vram_rd_data  = real_vram_rd_data;

    assign dbg_line_valid0        = u_scanout.u_fetch.line_valid[0];
    assign dbg_prefetch_active    = u_scanout.prefetch_active;
    assign dbg_display_line_ready = u_scanout.display_line_ready;
    assign dbg_y_src              = u_scanout.u_disp.y_src;
    assign dbg_x_src              = u_scanout.u_disp.x_src;
    assign dbg_rsp_x              = u_scanout.u_fetch.rsp_x;
    assign dbg_rsp_y              = u_scanout.u_fetch.rsp_y;

    fb_reader #(
        .ADDR_W             (ADDR_W),
        .DATA_W             (FETCH_W),
        .REQ_FIFO_DEPTH_LOG2(3),
        .RSP_FIFO_DEPTH_LOG2(4)
    ) u_fb_reader (
        .pclk       (pclk),
        .resetn     (~rst),
        .vram_clk   (vram_clk),
        .vram_rst   (rst),
        .s_rd_en    (fb_rd_en),
        .s_rd_addr  (fb_rd_addr),
        .s_rd_ready (fb_rd_ready),
        .s_rd_data  (scaler_fb_rd_data),
        .s_rd_valid (scaler_fb_rd_valid),
        .v_rd_en    (vram_rd_en),
        .v_rd_addr  (vram_rd_addr),
        .v_rd_data  (real_vram_rd_data),
        .v_rd_valid (real_vram_rd_valid),
        .underflow_sticky(fb_reader_underflow_sticky),
        .req_count  (),
        .rsp_count  (),
        .miss_count ()
    );

    linebuf_scanout #(
        .SRC_W           (SRC_W),
        .SRC_H           (SRC_H),
        // Match vram.v's 2 MiB VRAM_BYTES aperture — see the matching
        // comment on u_placement's FB_MAX_PIXELS above (Bug 2 / scenario
        // C: a fb_base_px in the upper VRAM half must not be rejected by
        // the fetch_frame_in_mem gate).
        .FB_MAX_PIXELS   (32'h0020_0000),
        .DST_W           (DST_W),
        .DST_H           (DST_H),
        .FETCH_W         (FETCH_W),
        .ADDR_W          (ADDR_W),
        .LINE_COUNT_LOG2 (4)
    ) u_scanout (
        .pclk        (pclk),
        .resetn      (~rst),
        .hcount      (hcount),
        .vcount      (vcount),
        .de_in       (de_in),
        .hs_in       (hs_in),
        .vs_in       (vs_in),
        .fb_base_px  (scanout_fb_base_px),
        .fb_stride_px(scanout_fb_stride_px),
        .bpp_shift   (scanout_bpp_shift),
        .bytes_per_px(scanout_bytes_per_px),
        // 1× passthrough — tb does pixel-exact 64×64 in 64×64 active.
        .hres        (12'd64),
        .vres        (12'd64),
        .scale_n     (3'd1),
        // Existing scanout tbs cover the NORMAL video path, so the boot
        // splash is retired here (dafb_live=1).  tb_scanout_frames.v drives
        // it low to cover the splash itself.
        .dafb_live   (1'b1),
        .clut_wclk   (vram_clk),
        .clut_we     (shim_clut_we),
        .clut_waddr  (shim_clut_waddr),
        .clut_wdata  (shim_clut_wdata),
        .fb_rd_en    (fb_rd_en),
        .fb_rd_addr  (fb_rd_addr),
        .fb_rd_ready (fb_rd_ready),
        .fb_rd_data  (scaler_fb_rd_data),
        .fb_rd_valid (scaler_fb_rd_valid),
        .rgb         (scanout_rgb),
        .de_out      (scanout_de),
        .hs_out      (),
        .vs_out      (),
        .line_underflow_sticky(line_underflow_sticky)
    );

    // ── Byte-swap shim output (CPU-side port after reorder) ──────────
    wire [3:0]   cpu_s_awid;
    wire [31:0]  cpu_s_awaddr;
    wire [7:0]   cpu_s_awlen;
    wire [2:0]   cpu_s_awsize;
    wire [1:0]   cpu_s_awburst;
    wire         cpu_s_awvalid;
    wire         cpu_s_awready;
    wire [127:0] cpu_s_wdata;
    wire [15:0]  cpu_s_wstrb;
    wire         cpu_s_wlast;
    wire         cpu_s_wvalid;
    wire         cpu_s_wready;
    wire [3:0]   cpu_s_bid;
    wire [1:0]   cpu_s_bresp;
    wire         cpu_s_bvalid;
    wire         cpu_s_bready;
    wire [3:0]   cpu_s_arid;
    wire [31:0]  cpu_s_araddr;
    wire [7:0]   cpu_s_arlen;
    wire [2:0]   cpu_s_arsize;
    wire [1:0]   cpu_s_arburst;
    wire         cpu_s_arvalid;
    wire         cpu_s_arready;
    wire [3:0]   cpu_s_rid;
    wire [127:0] cpu_s_rdata;
    wire [1:0]   cpu_s_rresp;
    wire         cpu_s_rlast;
    wire         cpu_s_rvalid;
    wire         cpu_s_rready;

    // ── xbar S3 VRAM byte-lane reorder, modelled inline ──────────────
    // This hop used to instantiate rtl/soc/vram_cpu_byteswap.v.  That
    // module was removed from the design when the identical transform
    // moved INTO the crossbar -- axi_xbar.v's vram_swap_word32()
    // ({w[7:0],w[15:8],w[23:16],w[31:24]}), applied to S3 wdata, wstrb
    // and rdata -- leaving the shim instantiated by nothing but this
    // testbench.  It was deleted from the tree 2026-09-03.
    //
    // The REORDER IS STILL REAL and still has to be modelled here, or
    // scenario-B stops reproducing the P1 pixel-mirror condition it
    // exists to guard.  It is done inline, mirroring axi_xbar.v, so this
    // tb now models where the swap actually lives instead of a module
    // the shipped design does not contain.  Keep in sync with
    // axi_xbar.v's vram_swap_word32/vram_swap_words/vram_swap_strb.
    genvar bsw;
    generate
        for (bsw = 0; bsw < 4; bsw = bsw + 1) begin : g_vram_bswap
            // W: byte-reverse data and strobe within each 32-bit lane.
            assign cpu_s_wdata[bsw*32 +: 32] = {
                vram_cpu_wdata[bsw*32 +:  8], vram_cpu_wdata[bsw*32 +  8 +: 8],
                vram_cpu_wdata[bsw*32 + 16 +: 8], vram_cpu_wdata[bsw*32 + 24 +: 8] };
            assign cpu_s_wstrb[bsw*4 +: 4] = {
                vram_cpu_wstrb[bsw*4 + 0], vram_cpu_wstrb[bsw*4 + 1],
                vram_cpu_wstrb[bsw*4 + 2], vram_cpu_wstrb[bsw*4 + 3] };
            // R: same reversal on the way back.
            assign vram_cpu_rdata[bsw*32 +: 32] = {
                cpu_s_rdata[bsw*32 +:  8], cpu_s_rdata[bsw*32 +  8 +: 8],
                cpu_s_rdata[bsw*32 + 16 +: 8], cpu_s_rdata[bsw*32 + 24 +: 8] };
        end
    endgenerate
    // Every other channel was a pure pass-through in the shim too.
    assign cpu_s_awid    = vram_cpu_awid;
    assign cpu_s_awaddr  = vram_cpu_awaddr;
    assign cpu_s_awlen   = vram_cpu_awlen;
    assign cpu_s_awsize  = vram_cpu_awsize;
    assign cpu_s_awburst = vram_cpu_awburst;
    assign cpu_s_awvalid = vram_cpu_awvalid;
    assign cpu_s_wlast   = vram_cpu_wlast;
    assign cpu_s_wvalid  = vram_cpu_wvalid;
    assign cpu_s_bready  = vram_cpu_bready;
    assign cpu_s_arid    = vram_cpu_arid;
    assign cpu_s_araddr  = vram_cpu_araddr;
    assign cpu_s_arlen   = vram_cpu_arlen;
    assign cpu_s_arsize  = vram_cpu_arsize;
    assign cpu_s_arburst = vram_cpu_arburst;
    assign cpu_s_arvalid = vram_cpu_arvalid;
    assign cpu_s_rready  = vram_cpu_rready;
    assign vram_cpu_awready = cpu_s_awready;
    assign vram_cpu_wready  = cpu_s_wready;
    assign vram_cpu_bid     = cpu_s_bid;
    assign vram_cpu_bresp   = cpu_s_bresp;
    assign vram_cpu_bvalid  = cpu_s_bvalid;
    assign vram_cpu_arready = cpu_s_arready;
    assign vram_cpu_rid     = cpu_s_rid;
    assign vram_cpu_rresp   = cpu_s_rresp;
    assign vram_cpu_rlast   = cpu_s_rlast;
    assign vram_cpu_rvalid  = cpu_s_rvalid;

    // ── Two-master -> one-slave mux ──────────────────────────────────
    // The testbench drives at most one of {vram_*, vram_cpu_*} at any
    // given moment; this mux latches ownership from AWVALID/ARVALID
    // and holds it until the corresponding B/R completion, so W beats
    // stay with the owner of the burst after AW deasserts.
    reg cpu_aw_active_q;
    reg cpu_ar_active_q;
    always @(posedge vram_clk) begin
        if (rst) begin
            cpu_aw_active_q <= 1'b0;
            cpu_ar_active_q <= 1'b0;
        end else begin
            // Acquire ownership at AW-valid / AR-valid when idle.
            if (!cpu_aw_active_q &&
                (vram_cpu_awvalid || vram_awvalid)) begin
                cpu_aw_active_q <= vram_cpu_awvalid;
            end
            // Release ownership at B handshake.
            if (cpu_aw_active_q && cpu_s_bvalid && cpu_s_bready) begin
                cpu_aw_active_q <= 1'b0;
            end else if (!cpu_aw_active_q && vram_bvalid && vram_bready) begin
                // Non-CPU side release — no-op on cpu_aw_active_q.
            end

            if (!cpu_ar_active_q &&
                (vram_cpu_arvalid || vram_arvalid)) begin
                cpu_ar_active_q <= vram_cpu_arvalid;
            end
            if (cpu_ar_active_q && cpu_s_rvalid && cpu_s_rready
                && cpu_s_rlast) begin
                cpu_ar_active_q <= 1'b0;
            end
        end
    end
    wire cpu_aw_active = cpu_aw_active_q |
        (!cpu_aw_active_q && !vram_awvalid && vram_cpu_awvalid);
    wire cpu_ar_active = cpu_ar_active_q |
        (!cpu_ar_active_q && !vram_arvalid && vram_cpu_arvalid);

    wire [3:0]   mux_awid    = cpu_aw_active ? cpu_s_awid    : vram_awid;
    wire [31:0]  mux_awaddr  = cpu_aw_active ? cpu_s_awaddr  : vram_awaddr;
    wire [7:0]   mux_awlen   = cpu_aw_active ? cpu_s_awlen   : vram_awlen;
    wire [2:0]   mux_awsize  = cpu_aw_active ? cpu_s_awsize  : vram_awsize;
    wire [1:0]   mux_awburst = cpu_aw_active ? cpu_s_awburst : vram_awburst;
    wire         mux_awvalid = vram_awvalid | cpu_s_awvalid;
    wire [127:0] mux_wdata   = cpu_aw_active ? cpu_s_wdata   : vram_wdata;
    wire [15:0]  mux_wstrb   = cpu_aw_active ? cpu_s_wstrb   : vram_wstrb;
    wire         mux_wlast   = cpu_aw_active ? cpu_s_wlast   : vram_wlast;
    wire         mux_wvalid  = vram_wvalid | cpu_s_wvalid;
    wire [3:0]   mux_arid    = cpu_ar_active ? cpu_s_arid    : vram_arid;
    wire [31:0]  mux_araddr  = cpu_ar_active ? cpu_s_araddr  : vram_araddr;
    wire [7:0]   mux_arlen   = cpu_ar_active ? cpu_s_arlen   : vram_arlen;
    wire [2:0]   mux_arsize  = cpu_ar_active ? cpu_s_arsize  : vram_arsize;
    wire [1:0]   mux_arburst = cpu_ar_active ? cpu_s_arburst : vram_arburst;
    wire         mux_arvalid = vram_arvalid | cpu_s_arvalid;

    // Slave signals from vram:
    wire         vram_slv_awready;
    wire         vram_slv_wready;
    wire [3:0]   vram_slv_bid;
    wire [1:0]   vram_slv_bresp;
    wire         vram_slv_bvalid;
    wire         vram_slv_arready;
    wire [3:0]   vram_slv_rid;
    wire [127:0] vram_slv_rdata;
    wire [1:0]   vram_slv_rresp;
    wire         vram_slv_rlast;
    wire         vram_slv_rvalid;

    // Fan-out: each master sees the slave outputs ANDed with their
    // own-lane selector.  The "no outstanding" master gets zeros.
    assign vram_awready    = vram_slv_awready & ~cpu_aw_active;
    assign cpu_s_awready   = vram_slv_awready &  cpu_aw_active;
    assign vram_wready     = vram_slv_wready  & ~cpu_aw_active;
    assign cpu_s_wready    = vram_slv_wready  &  cpu_aw_active;
    assign vram_bid        = vram_slv_bid;
    assign vram_bresp      = vram_slv_bresp;
    assign vram_bvalid     = vram_slv_bvalid  & ~cpu_aw_active;
    assign cpu_s_bid       = vram_slv_bid;
    assign cpu_s_bresp     = vram_slv_bresp;
    assign cpu_s_bvalid    = vram_slv_bvalid  &  cpu_aw_active;
    assign vram_arready    = vram_slv_arready & ~cpu_ar_active;
    assign cpu_s_arready   = vram_slv_arready &  cpu_ar_active;
    assign vram_rid        = vram_slv_rid;
    assign vram_rdata      = vram_slv_rdata;
    assign vram_rresp      = vram_slv_rresp;
    assign vram_rlast      = vram_slv_rlast;
    assign vram_rvalid     = vram_slv_rvalid  & ~cpu_ar_active;
    assign cpu_s_rid       = vram_slv_rid;
    assign cpu_s_rdata     = vram_slv_rdata;
    assign cpu_s_rresp     = vram_slv_rresp;
    assign cpu_s_rlast     = vram_slv_rlast;
    assign cpu_s_rvalid    = vram_slv_rvalid  &  cpu_ar_active;
    wire         mux_bready = cpu_aw_active ? cpu_s_bready : vram_bready;
    wire         mux_rready = cpu_ar_active ? cpu_s_rready : vram_rready;

    vram #(
        .FB_WIDTH_PX (256),
        .FB_HEIGHT_PX(80),
        // BPP is the pixel packing / read-port address granularity;
        // RD_DATA_W is the read-port DATA width, which vram.v requires to be
        // exactly 4*BPP -- equal to FETCH_W here because VRAM_BPP is 8.
        .BPP         (VRAM_BPP),
        .RD_DATA_W   (4 * VRAM_BPP),
        .DATA_WIDTH  (128),
        .ID_WIDTH    (4)
    ) u_vram (
        .clk(vram_clk),
        .rst(rst),
        .clear_req(1'b0),

        .s_awid   (mux_awid),
        .s_awaddr (mux_awaddr),
        .s_awlen  (mux_awlen),
        .s_awsize (mux_awsize),
        .s_awburst(mux_awburst),
        .s_awvalid(mux_awvalid),
        .s_awready(vram_slv_awready),
        .s_wdata  (mux_wdata),
        .s_wstrb  (mux_wstrb),
        .s_wlast  (mux_wlast),
        .s_wvalid (mux_wvalid),
        .s_wready (vram_slv_wready),
        .s_bid    (vram_slv_bid),
        .s_bresp  (vram_slv_bresp),
        .s_bvalid (vram_slv_bvalid),
        .s_bready (mux_bready),
        .s_arid   (mux_arid),
        .s_araddr (mux_araddr),
        .s_arlen  (mux_arlen),
        .s_arsize (mux_arsize),
        .s_arburst(mux_arburst),
        .s_arvalid(mux_arvalid),
        .s_arready(vram_slv_arready),
        .s_rid    (vram_slv_rid),
        .s_rdata  (vram_slv_rdata),
        .s_rresp  (vram_slv_rresp),
        .s_rlast  (vram_slv_rlast),
        .s_rvalid (vram_slv_rvalid),
        .s_rready (mux_rready),
        .rd_clk   (vram_clk),
        .rd_rst   (rst),
        .rd_addr  (vram_rd_addr[VRAM_ADDR_W-1:0]),
        .rd_en    (vram_rd_en),
        .rd_data  (real_vram_rd_data),
        .rd_valid (real_vram_rd_valid)
    );

endmodule

`default_nettype wire
