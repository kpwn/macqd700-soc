// tb_mame_vram_scaler_dump.v -- MAME VRAM dump -> fb_reader -> scaler image.
//
// This wrapper keeps the dump path focused on the synthesizable scanout
// machinery.  The C++ harness serves pixels from a raw MAME/bridge VRAM dump
// through fb_reader's VRAM-side request/response port; linebuf_scanout then
// prefetches and emits RGB exactly as the production scaler path does.

`default_nettype none

module tb_mame_vram_scaler_dump (
    input  wire         pclk,
    input  wire         vram_clk,
    input  wire         rst,

    input  wire [11:0]  hcount,
    input  wire [10:0]  vcount,
    input  wire         de_in,
    input  wire         hs_in,
    input  wire         vs_in,

    // BPP mode select for the RTL-side decoder.  0=8bpp (legacy SW
    // pre-unpacking path); 3=1bpp (RTL unpacks bits, expects raw
    // VRAM bytes from the harness — no SW slicing).
    input  wire [2:0]   bpp_shift,
    // Stride between source rows in BYTES (raw VRAM units, NOT
    // pixels-per-byte-shifted).  Plumbed from harness so 1bpp Mac
    // configs (640x480, stride=1024 bytes) match MAME geometry.
    input  wire [19:0]  fb_base_px,
    input  wire [19:0]  fb_stride_px,

    output wire         v_rd_en,
    output wire [19:0]  v_rd_addr,
    // 4-byte group starting at v_rd_addr, modelled by the C++ harness:
    //   [31:24] byte at v_rd_addr    (always valid, any alignment)
    //   [23:0]  bytes at +1/+2/+3    (valid only when 4-byte aligned)
    // Matches vram.v / scanout_ddr_reader.v's streaming-port contract.
    input  wire [31:0]  v_rd_data,
    input  wire         v_rd_valid,

    output wire [23:0]  scanout_rgb,
    output wire         scanout_de,
    output wire         fb_reader_underflow,
    output wire         line_underflow,
    output wire         fb_underflow_sticky
);

    localparam SRC_W       = 640;
    localparam SRC_H       = 480;
    localparam DST_W       = 640;
    localparam DST_H       = 480;
    localparam ACTIVE_W    = 640;
    localparam ACTIVE_H    = 480;
    localparam BORDER_X    = 0;
    localparam BORDER_Y    = 0;
    localparam SCALE_NUM   = 1;
    localparam SCALE_DEN   = 1;
    // Width of the streaming fetch-port DATA bus: a 4-byte group per
    // request.  Not a pixel depth (the depth comes in on bpp_shift).
    localparam FETCH_W     = 32;
    localparam ADDR_W      = 20;
    localparam [ADDR_W-1:0] SRC_W_ADDR = SRC_W;

    wire        fb_rd_en;
    wire [19:0] fb_rd_addr;
    wire        fb_rd_ready;
    wire [FETCH_W-1:0] fb_rd_data;
    wire        fb_rd_valid;

    fb_reader #(
        .ADDR_W             (ADDR_W),
        .DATA_W             (FETCH_W),
        .REQ_FIFO_DEPTH_LOG2(4),
        .RSP_FIFO_DEPTH_LOG2(5)
    ) u_fb_reader (
        .pclk       (pclk),
        .resetn     (~rst),
        .vram_clk   (vram_clk),
        .vram_rst   (rst),
        .s_rd_en    (fb_rd_en),
        .s_rd_addr  (fb_rd_addr),
        .s_rd_ready (fb_rd_ready),
        .s_rd_data  (fb_rd_data),
        .s_rd_valid (fb_rd_valid),
        .v_rd_en    (v_rd_en),
        .v_rd_addr  (v_rd_addr),
        .v_rd_data  (v_rd_data),
        .v_rd_valid (v_rd_valid),
        .underflow_sticky(fb_reader_underflow),
        .req_count  (),
        .rsp_count  (),
        .miss_count ()
    );

    // ── 256-entry CLUT write port: preload the legacy 16-color table ──
    // T9 retired the flat 384-bit clut_rgb bus in favour of a real
    // clut_we/clut_waddr/clut_wdata write-pulse export (see rtl/mac/
    // video.v and rtl/board/video_phy/linebuf_scanout.v).  This module
    // has no CPU-driven RAMDAC register file to poke -- unlike the
    // other T9-updated tb wrappers, which forward a real AXI-lite write
    // sequence into video.v -- so it sequences the writes itself: one
    // entry per vram_clk cycle for the 16 cycles immediately after
    // reset release, then stops (matches the write port's own no-reset,
    // "software programs once" contract).
    function [23:0] legacy_clut_entry;
        input [3:0] idx;
        begin
            case (idx)
                4'h0: legacy_clut_entry = 24'hffffff;
                4'h1: legacy_clut_entry = 24'h000000;
                4'h2: legacy_clut_entry = 24'hdddddd;
                4'h3: legacy_clut_entry = 24'hbbbbbb;
                4'h4: legacy_clut_entry = 24'h999999;
                4'h5: legacy_clut_entry = 24'h777777;
                4'h6: legacy_clut_entry = 24'h555555;
                4'h7: legacy_clut_entry = 24'h333333;
                4'h8: legacy_clut_entry = 24'hff0000;
                4'h9: legacy_clut_entry = 24'h00aa00;
                4'ha: legacy_clut_entry = 24'h0000ff;
                4'hb: legacy_clut_entry = 24'hffaa00;
                4'hc: legacy_clut_entry = 24'h00aaaa;
                4'hd: legacy_clut_entry = 24'haa00aa;
                4'he: legacy_clut_entry = 24'h4444ff;
                default: legacy_clut_entry = 24'h111111;
            endcase
        end
    endfunction

    reg [3:0]  clut_load_idx;
    reg        clut_load_done;
    reg        clut_we_r;
    reg [7:0]  clut_waddr_r;
    reg [23:0] clut_wdata_r;

    always @(posedge vram_clk) begin
        if (rst) begin
            clut_load_idx  <= 4'd0;
            clut_load_done <= 1'b0;
            clut_we_r      <= 1'b0;
            clut_waddr_r   <= 8'd0;
            clut_wdata_r   <= 24'd0;
        end else if (!clut_load_done) begin
            clut_we_r      <= 1'b1;
            clut_waddr_r   <= {4'd0, clut_load_idx};
            clut_wdata_r   <= legacy_clut_entry(clut_load_idx);
            if (clut_load_idx == 4'hF)
                clut_load_done <= 1'b1;
            clut_load_idx  <= clut_load_idx + 4'd1;
        end else begin
            clut_we_r <= 1'b0;
        end
    end

    linebuf_scanout #(
        .SRC_W          (SRC_W),
        .SRC_H          (SRC_H),
        // Match Q700 VRAM aperture (2 MB) so 1bpp Mac configs with
        // base=0x1000 stride=1024 are not rejected by the in-range check
        // (raw byte addresses span up to ~0x79000 for 480 source rows).
        .FB_MAX_PIXELS  (32'h200000),
        .DST_W          (DST_W),
        .DST_H          (DST_H),
        .FETCH_W        (FETCH_W),
        .ADDR_W         (ADDR_W),
        .LINE_COUNT_LOG2(5)
    ) u_scanout (
        .pclk        (pclk),
        .resetn      (~rst),
        .hcount      (hcount),
        .vcount      (vcount),
        .de_in       (de_in),
        .hs_in       (hs_in),
        .vs_in       (vs_in),
        .fb_base_px  (fb_base_px),
        .fb_stride_px(fb_stride_px),
        .bpp_shift   (bpp_shift),
        .bytes_per_px(3'd1),   // SW-pre-unpacked byte-per-pixel stream
        // 1× passthrough scale for the SW-pre-unpacked pixel stream.
        .hres        (12'd640),
        .vres        (12'd480),
        .scale_n     (3'd1),
        // Normal video path only; the boot-splash switchover is covered
        // in tb_scanout_placement_sync / tb_scanout_frames.
        .dafb_live   (1'b1),
        .clut_wclk   (vram_clk),
        .clut_we     (clut_we_r),
        .clut_waddr  (clut_waddr_r),
        .clut_wdata  (clut_wdata_r),
        .fb_rd_en    (fb_rd_en),
        .fb_rd_addr  (fb_rd_addr),
        .fb_rd_ready (fb_rd_ready),
        .fb_rd_data  (fb_rd_data),
        .fb_rd_valid (fb_rd_valid),
        .rgb         (scanout_rgb),
        .de_out      (scanout_de),
        .hs_out      (),
        .vs_out      (),
        .line_underflow_sticky(line_underflow)
    );

    assign fb_underflow_sticky = fb_reader_underflow | line_underflow;

endmodule

`default_nettype wire
