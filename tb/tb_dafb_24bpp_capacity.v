// tb_dafb_24bpp_capacity.v -- PRODUCTION-GEOMETRY DAFB depth-switch scanout tb.
// ---------------------------------------------------------------------------
// Why this exists (and why tb-framebuffer-pixel could not catch the bug it
// covers): tb_framebuffer_pixel.v elaborates the scanout path at SRC_W=SRC_H=64
// against a 2 MiB FB_MAX_PIXELS.  Every placement-range term there is three
// orders of magnitude below the aperture, so the "does the whole frame fit in
// VRAM" gate in scanout_placement_sync.v / linebuf_scanout.v is structurally
// unreachable in that harness no matter which depth it runs.  It proves pixel
// formatting; it cannot prove placement admission.
//
// This tb instantiates the SHIPPING geometry instead --
//     SRC_W=1024  SRC_H=768  FB_MAX_PIXELS=0x0020_0000 (2 MiB URAM aperture)
//     ADDR_W=21   DST_W=1920 DST_H=1080
// exactly as rtl/soc/fpga_top_video.vh does for the no-VRAM_IN_DDR build --
// and drives the real rtl/mac/video.v DAFB shim with the register values read
// off the live board over JTAG at both depths:
//
//            8bpp (works)          24bpp (black screen on HW)
//   BASE     +0x000 = 0x008        +0x000 = 0x008     (m_base = 8<<9 = 4096)
//   STRIDE   +0x008 = 0x100        +0x008 = 0x400     (m_stride = raw<<2)
//   CONFIG   +0x010 = 0x030        +0x010 = 0x032
//   PCBR     +0x220 = 0x098        +0x220 = 0x09c     (AC842 mode 0x18/0x1c)
//
// Swatch H/V params are programmed for the 640x480 mode the board was in
// (hres = HFP-HAL, vres = (VFP>>1)-(VAL>>1); see rtl/mac/video.v).
//
// The VRAM model returns a synthetic pattern whose 4-byte group at any
// 4-ALIGNED address is {0x00, 0xff, 0xff, 0xff} -- i.e. xRGB WHITE at 24bpp
// (matching the 0x00ffffff words read out of the live framebuffer), and CLUT
// indices {0, 255, 255, 255} at 8bpp.  With the gray-ramp CLUT the harness
// loads, both depths must therefore paint a mostly-white active window.
// ---------------------------------------------------------------------------
`default_nettype none

module tb_dafb_24bpp_capacity (
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

    // Direct CLUT load (the AC842 RAMDAC write protocol is already proven by
    // tb-framebuffer-pixel; this tb is about placement admission, so it loads
    // the palette straight into linebuf_scanout's BRAM port).
    input  wire        clut_we,
    input  wire [7:0]  clut_waddr,
    input  wire [23:0] clut_wdata,

    // ── Fetch-port memory model shape ────────────────────────────────
    // The production path does NOT look like a 1-cycle always-ready RAM:
    // linebuf_scanout talks to fb_reader.v, a pclk<->vram_clk CDC bridge
    // with ~31+ cycles of round-trip latency and a request FIFO that
    // deasserts s_rd_ready when it backs up.  Latency matters here because
    // linebuf's re-arm gate is `outstanding_empty` (req counters == rsp
    // counters): with a 1-cycle model the walk is essentially never
    // outstanding, so a whole class of drain/re-arm bug is unreachable.
    // It matters MORE at 1bpp than at 8bpp, because a 1bpp row is only 80
    // requests -- less than the round-trip latency -- so the request walk
    // runs more than a full row ahead of the response walk, which never
    // happens at 8bpp's 640 requests per row.
    //   mem_latency  : cycles from accept to fb_rd_valid (1..255)
    //   mem_stall_lfsr: when 1, fb_rd_ready pseudo-randomly deasserts
    input  wire [7:0]  mem_latency,
    input  wire        mem_backpressure,
    // When 1, the fetch port goes through the REAL fb_reader.v CDC bridge
    // (pclk <-> vram_clk, gray-pointer async FIFOs, credit throttle) exactly
    // as video_top.v wires it in production, instead of straight to the
    // memory model.  vram_clk is driven independently by the harness.
    input  wire        mem_via_fb_reader,
    input  wire        vram_clk,
    // Pulse for one cycle to make the direct memory model swallow exactly
    // ONE response.  Models the one thing the fetch port can do that
    // linebuf_scanout has no defence against: return fewer responses than
    // requests (fb_reader.v drops on response-FIFO overflow, and any
    // reset-bank skew between linebuf and fb_reader has the same effect).
    input  wire        mem_drop_rsp,

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

    // ── scanout_placement_sync admission gate ────────────────────────
    output wire        dbg_ps_depth_ok,
    output wire        dbg_ps_stride_sane,
    output wire        dbg_ps_in_range,
    output wire [31:0] dbg_ps_frame_last_addr,
    output wire [31:0] dbg_ps_min_stride,

    // ── what actually got committed to the scanner ───────────────────
    output wire [20:0] dbg_committed_base,
    output wire [20:0] dbg_committed_stride,
    output wire [2:0]  dbg_committed_bytes_per_px,
    output wire [11:0] dbg_committed_hres,
    output wire [11:0] dbg_committed_vres,

    // ── linebuf_scanout's own admission gate + fetch state ───────────
    output wire        dbg_fetch_direct,
    output wire        dbg_fetch_wide,
    output wire        dbg_fetch_stride_sane,
    output wire        dbg_fetch_frame_in_mem,
    output wire        dbg_can_request,
    output wire        dbg_prefetch_active,
    output wire        dbg_display_line_ready,
    output wire [31:0] dbg_req_addr_limit,
    output wire [20:0] dbg_row_last_off,

    // ── Fetch liveness ───────────────────────────────────────────────
    // The live board's VIO probe `vio_vram_read = {vram_rd_valid,
    // vram_rd_en}` showed the scanout still issuing VRAM reads at a
    // comparable rate at 24bpp as at 8bpp.  That observation cannot
    // discriminate between "the 24bpp placement is running" and "the OLD
    // 8bpp placement is still running because the 24bpp one was never
    // admitted": 640x480 costs 307,200 requests per frame at 8bpp (one byte
    // per pixel) AND at WIDE 24bpp (one aligned 4-byte word per pixel).
    // Exported so the harness can show both numbers.
    output wire        dbg_fb_rd_en,
    output wire        dbg_fb_rd_valid,
    output wire        dbg_fb_rd_ready,

    // ── Restart / re-arm machinery (the 1bpp wedge) ──────────────────
    // The fetch walk terminates at fetch_row_y_last and is re-armed by the
    // unconditional start-of-frame resync.  Post-rewrite that is ONE FSM
    // plus a credit count, so these probes name the state and the credits
    // rather than four "pending" booleans.
    output wire        dbg_placement_changed,
    output wire        dbg_in_resync,
    output wire [2:0]  dbg_state,
    output wire [15:0] dbg_credits,
    output wire [15:0] dbg_stale_count,
    output wire        dbg_outstanding_empty,
    output wire        dbg_req_addr_oob,
    output wire [15:0] dbg_req_y,
    output wire [15:0] dbg_req_x,
    output wire [15:0] dbg_rsp_y,
    output wire [15:0] dbg_y_src,
    output wire [15:0] dbg_fetch_row_y_last,
    output wire [15:0] dbg_row_y_last_in,
    output wire [15:0] dbg_fetch_row_px_last,
    output wire [15:0] dbg_row_last_idx_in,
    output wire [31:0] dbg_fb_rd_addr,
    output wire [15:0] dbg_mem_drops,
    output wire        dbg_fbr_underflow_sticky,
    output wire [15:0] dbg_fbr_req_count,
    output wire [15:0] dbg_fbr_rsp_count,
    output wire [15:0] dbg_fbr_miss_count
);

    // ── Shipping geometry (rtl/soc/fpga_top_video.vh, VRAM_IN_DDR off) ──
    localparam SRC_W         = 1024;
    localparam SRC_H         = 768;
    localparam FB_MAX_PIXELS = 32'h0020_0000;   // 2 MiB URAM aperture
    localparam ADDR_W        = 21;
    localparam FETCH_W       = 32;
    localparam DST_W         = 1920;
    localparam DST_H         = 1080;

    wire [31:0] shim_base;
    wire [31:0] shim_stride;
    wire [2:0]  shim_bpp_shift;
    wire [2:0]  shim_bytes_per_px;
    wire        shim_depth_supported;
    wire [11:0] shim_hres;
    wire [11:0] shim_vres;

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
        .clut_we      (),
        .clut_waddr   (),
        .clut_wdata   (),
        .irq          (),
        .pll_pixel_clock(),
        .scsi0_ctrl_out(),
        .scsi0_drq_in (1'b0),
        .frame_tick   (1'b0),
        .monitor_sense(7'h06)          // Mac Hi-Res 12-14" 640x480
    );

    assign dbg_shim_base            = shim_base;
    assign dbg_shim_stride          = shim_stride;
    assign dbg_shim_bytes_per_px    = shim_bytes_per_px;
    assign dbg_shim_bpp_shift       = shim_bpp_shift;
    assign dbg_shim_depth_supported = shim_depth_supported;
    assign dbg_shim_hres            = shim_hres;
    assign dbg_shim_vres            = shim_vres;

    wire frame_start = (hcount == 12'd0) && (vcount == 11'd0);

    wire [ADDR_W-1:0] sc_base;
    wire [ADDR_W-1:0] sc_stride;
    wire [2:0]        sc_bpp_shift;
    wire [2:0]        sc_bytes_per_px;
    wire [11:0]       sc_hres;
    wire [11:0]       sc_vres;
    wire [2:0]        sc_scale_n;

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
        // Normal video path only; the boot-splash switchover is covered
        // in tb_scanout_placement_sync / tb_scanout_frames.
        .dafb_live               ()
    );

    assign dbg_ps_depth_ok        = u_placement.depth_ok_stable;
    assign dbg_ps_stride_sane     = u_placement.stable_stride_sane;
    assign dbg_ps_in_range        = u_placement.stable_placement_in_range;
    assign dbg_ps_frame_last_addr = u_placement.frame_last_addr;
    assign dbg_ps_min_stride      = {11'd0, u_placement.min_stride_bytes};
    assign dbg_committed_base       = sc_base;
    assign dbg_committed_stride     = sc_stride;
    assign dbg_committed_bytes_per_px = sc_bytes_per_px;
    assign dbg_committed_hres       = sc_hres;
    assign dbg_committed_vres       = sc_vres;

    // ── Scanout fetch port + trivial VRAM model ──────────────────────
    wire              fb_rd_en;
    wire [ADDR_W-1:0] fb_rd_addr;
    reg               fb_rd_valid;
    reg  [FETCH_W-1:0] fb_rd_data;
    // ── Two INDEPENDENT memory models, one per fetch path ────────────
    // They must not share state: the unselected path's model would keep
    // answering requests and inject phantom v_rd_valid pulses into the other
    // consumer.  Each is held in reset while its path is deselected.
    //
    //   direct : linebuf_scanout -> latency pipe (pclk)
    //   cdc    : linebuf_scanout -> REAL fb_reader.v -> latency pipe (vram_clk)
    //
    // The CDC configuration is the production wiring (video_top.v), and it is
    // the only one where request/response accounting crosses a clock domain --
    // which matters because linebuf_scanout's re-arm gate is
    // `outstanding_empty`, i.e. request count == response count.
    wire              fb_rd_ready;
    localparam MAXLAT = 256;

    function [7:0] vram_byte;
        input [ADDR_W+1:0] a;
        begin
            // 4-byte group at any aligned address = {00, ff, ff, ff}:
            // xRGB WHITE at 24bpp, CLUT indices {0,255,255,255} at 8bpp.
            vram_byte = (a[1:0] == 2'b00) ? 8'h00 : 8'hff;
        end
    endfunction

    // Pseudo-random request-port backpressure (the direct model only; the CDC
    // model's backpressure is fb_reader's own request FIFO).
    reg [15:0] stall_lfsr;
    always @(posedge pclk) begin
        if (rst) stall_lfsr <= 16'hACE1;
        else     stall_lfsr <= {stall_lfsr[14:0],
                                stall_lfsr[15] ^ stall_lfsr[13] ^
                                stall_lfsr[12] ^ stall_lfsr[10]};
    end
    wire [7:0] lat_idx = (mem_latency == 8'd0) ? 8'd0 : (mem_latency - 8'd1);

    // ── Direct model (pclk) ──────────────────────────────────────────
    wire dir_sel   = !mem_via_fb_reader;
    wire dir_ready = mem_backpressure ? (stall_lfsr[2:0] != 3'd0) : 1'b1;
    wire [ADDR_W+1:0] dir_a = {2'b00, fb_rd_addr};
    wire [FETCH_W-1:0] dir_word = {vram_byte(dir_a),     vram_byte(dir_a + 1),
                                   vram_byte(dir_a + 2), vram_byte(dir_a + 3)};
    reg                dir_v [0:MAXLAT-1];
    reg [FETCH_W-1:0]  dir_d [0:MAXLAT-1];
    reg                dir_valid;
    reg [FETCH_W-1:0]  dir_data;
    reg                drop_arm;
    reg [15:0]         drop_count;
    assign dbg_mem_drops = drop_count;
    always @(posedge pclk) begin
        if (rst || !dir_sel) begin
            drop_arm   <= 1'b0;
            if (rst) drop_count <= 16'd0;
        end else if (drop_arm) begin
            if (fb_rd_en && dir_ready) begin
                drop_arm   <= 1'b0;
                drop_count <= drop_count + 16'd1;
            end
        end else if (mem_drop_rsp) begin
            drop_arm <= 1'b1;
        end
    end
    integer di;
    always @(posedge pclk) begin
        if (rst || !dir_sel) begin
            dir_valid <= 1'b0;
            dir_data  <= {FETCH_W{1'b0}};
            for (di = 0; di < MAXLAT; di = di + 1) begin
                dir_v[di] <= 1'b0;
                dir_d[di] <= {FETCH_W{1'b0}};
            end
        end else begin
            dir_valid <= dir_v[lat_idx];
            dir_data  <= dir_d[lat_idx];
            for (di = MAXLAT-1; di > 0; di = di - 1) begin
                dir_v[di] <= dir_v[di-1];
                dir_d[di] <= dir_d[di-1];
            end
            dir_v[0] <= (fb_rd_en && dir_ready) && !drop_arm;
            dir_d[0] <= dir_word;
        end
    end

    // ── CDC model (vram_clk), behind the REAL fb_reader ──────────────
    wire              fbr_s_ready;
    wire [FETCH_W-1:0] fbr_s_data;
    wire              fbr_s_valid;
    wire              fbr_v_en;
    wire [ADDR_W-1:0] fbr_v_addr;
    wire [ADDR_W+1:0] cdc_a = {2'b00, fbr_v_addr};
    wire [FETCH_W-1:0] cdc_word = {vram_byte(cdc_a),     vram_byte(cdc_a + 1),
                                   vram_byte(cdc_a + 2), vram_byte(cdc_a + 3)};
    reg                cdc_v [0:MAXLAT-1];
    reg [FETCH_W-1:0]  cdc_d [0:MAXLAT-1];
    reg                cdc_valid;
    reg [FETCH_W-1:0]  cdc_data;
    integer ci;
    always @(posedge vram_clk) begin
        if (rst || !mem_via_fb_reader) begin
            cdc_valid <= 1'b0;
            cdc_data  <= {FETCH_W{1'b0}};
            for (ci = 0; ci < MAXLAT; ci = ci + 1) begin
                cdc_v[ci] <= 1'b0;
                cdc_d[ci] <= {FETCH_W{1'b0}};
            end
        end else begin
            cdc_valid <= cdc_v[lat_idx];
            cdc_data  <= cdc_d[lat_idx];
            for (ci = MAXLAT-1; ci > 0; ci = ci - 1) begin
                cdc_v[ci] <= cdc_v[ci-1];
                cdc_d[ci] <= cdc_d[ci-1];
            end
            cdc_v[0] <= fbr_v_en;
            cdc_d[0] <= cdc_word;
        end
    end

    fb_reader #(
        .ADDR_W        (ADDR_W),
        .DATA_W        (FETCH_W),
        .RETURN_LATENCY(31)
    ) u_fb_reader (
        .pclk       (pclk),
        // Held in reset while the direct path is selected, so it starts
        // clean every time the CDC path is engaged.
        .resetn     (~rst & mem_via_fb_reader),
        .vram_clk   (vram_clk),
        .vram_rst   (rst | ~mem_via_fb_reader),
        .s_rd_en    (mem_via_fb_reader ? fb_rd_en : 1'b0),
        .s_rd_addr  (fb_rd_addr),
        .s_rd_ready (fbr_s_ready),
        .s_rd_data  (fbr_s_data),
        .s_rd_valid (fbr_s_valid),
        .v_rd_en    (fbr_v_en),
        .v_rd_addr  (fbr_v_addr),
        .v_rd_data  (cdc_data),
        .v_rd_valid (cdc_valid),
        .underflow_sticky(dbg_fbr_underflow_sticky),
        .req_count  (dbg_fbr_req_count),
        .rsp_count  (dbg_fbr_rsp_count),
        .miss_count (dbg_fbr_miss_count)
    );

    assign fb_rd_ready = mem_via_fb_reader ? fbr_s_ready : dir_ready;
    always @(*) begin
        if (mem_via_fb_reader) begin
            fb_rd_data  = fbr_s_data;
            fb_rd_valid = fbr_s_valid;
        end else begin
            fb_rd_data  = dir_data;
            fb_rd_valid = dir_valid;
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
        // Normal video path only; the boot-splash switchover is covered
        // in tb_scanout_placement_sync / tb_scanout_frames.
        .dafb_live   (1'b1),
        .clut_wclk   (pclk),
        .clut_we     (clut_we),
        .clut_waddr  (clut_waddr),
        .clut_wdata  (clut_wdata),
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
    assign dbg_prefetch_active    = u_scanout.prefetch_active;
    assign dbg_display_line_ready = u_scanout.display_line_ready;
    assign dbg_req_addr_limit     = u_scanout.u_fetch.req_addr_limit;
    assign dbg_row_last_off       = u_scanout.u_fetch.row_last_off;
    assign dbg_fb_rd_en           = fb_rd_en;
    assign dbg_fb_rd_valid        = fb_rd_valid;
    assign dbg_fb_rd_ready        = fb_rd_ready;

    assign dbg_placement_changed         = u_scanout.placement_changed;
    // Post-rewrite equivalents.  The four "pending" flags are gone -- a
    // restart is a STATE now -- so the probes name the state instead.
    assign dbg_in_resync                 = u_scanout.in_resync;
    assign dbg_state                     = {1'b0, u_scanout.fetch_state};
    assign dbg_credits                   = {2'd0, u_scanout.u_fetch.credits};
    assign dbg_stale_count               = {4'd0, u_scanout.u_fetch.stale_count};
    assign dbg_outstanding_empty         = u_scanout.outstanding_empty;
    assign dbg_req_addr_oob              = u_scanout.req_addr_oob;
    assign dbg_req_y              = {6'd0, u_scanout.u_fetch.req_y};
    assign dbg_req_x              = {6'd0, u_scanout.u_fetch.req_x};
    assign dbg_rsp_y              = {6'd0, u_scanout.u_fetch.rsp_y};
    assign dbg_y_src              = {6'd0, u_scanout.u_disp.y_src};
    assign dbg_fetch_row_y_last   = {6'd0, u_scanout.u_fetch.fetch_row_y_last};
    assign dbg_row_y_last_in      = {6'd0, u_scanout.u_fetch.row_y_last_in};
    assign dbg_fetch_row_px_last  = {6'd0, u_scanout.u_fetch.fetch_row_px_last};
    assign dbg_row_last_idx_in    = {6'd0, u_scanout.u_fetch.row_last_idx_in};
    assign dbg_fb_rd_addr         = {11'd0, fb_rd_addr};

endmodule

`default_nettype wire
