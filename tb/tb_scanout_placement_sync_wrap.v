// tb_scanout_placement_sync_wrap.v -- parameterized scanout placement DUTs.

`default_nettype none

module tb_scanout_placement_sync_wrap (
    input  wire        pclk,
    input  wire        rst,
    input  wire        frame_start,

    input  wire [19:0] rom_scanout_fb_base_px,
    input  wire [19:0] rom_scanout_fb_stride_px,
    output wire [19:0] rom_fb_base_px,
    output wire [19:0] rom_fb_stride_px,

    input  wire [19:0] hw_scanout_fb_base_px,
    input  wire [19:0] hw_scanout_fb_stride_px,
    output wire [19:0] hw_fb_base_px,
    output wire [19:0] hw_fb_stride_px,

    // Geometry-channel torn-value test instance: base/stride tied to
    // fixed valid constants so the equality filter on bpp_shift/hres/
    // vres can be exercised in isolation.
    input  wire [2:0]  geom_scanout_bpp_shift,
    // Runtime VRAM-bytes-per-pixel channel: 1 = 8bpp indexed, 4 = 24bpp
    // direct colour.  Drives the placement range/stride check's footprint
    // term, so a 24bpp placement is measured at 4 B/px.
    input  wire [2:0]  geom_scanout_bytes_per_px,
    // Depth-gate test input: low models an unmapped AC842 depth code the
    // scanner cannot render.
    input  wire        geom_scanout_depth_supported,
    input  wire [19:0] geom_scanout_fb_base_px,
    // Driveable stride so the 24bpp footprint gate can be exercised: at
    // 4 B/px a legal stride is >= 4*hres, far above the 8bpp default.
    input  wire [19:0] geom_scanout_fb_stride_px,
    input  wire [11:0] geom_scanout_hres,
    input  wire [11:0] geom_scanout_vres,
    output wire [2:0]  geom_bpp_shift,
    output wire [2:0]  geom_bytes_per_px,
    output wire [11:0] geom_hres,
    output wire [11:0] geom_vres,
    // Exposed so the depth gate's effect on the placement commit is
    // observable: an unsupported depth must retain the last committed
    // base/stride rather than accepting a new pair.
    output wire [19:0] geom_fb_base_px,
    output wire [19:0] geom_fb_stride_px,
    // Committed INTEGER output scale N (place_plan.v).  This instance is
    // the one with driveable hres/vres, so it is where the scaling POLICY
    // is exercised -- including the Q700 832x624 mode, which lands on N=1.
    output wire [2:0]  geom_scale_n,
    // ── Loud-failure channel (docs/video_path_review.md S4.2) ────────
    // Exposed on the two instances whose inputs are fully driveable, so a
    // rejection can be provoked deliberately and the REASON checked rather
    // than inferred from "the placement did not move".
    output wire [3:0]  geom_reject_reason,
    output wire        geom_rejected_sticky,
    output wire [19:0] geom_reject_base_px,
    output wire [19:0] geom_reject_stride_px,
    output wire [11:0] geom_reject_hres_px,
    output wire [11:0] geom_reject_vres_px,
    output wire [3:0]  geom_reject_reason_latched,
    // Boot-splash switchover.  Sticky "the DAFB committed a renderable
    // placement with a real geometry"; this instance is the one with
    // driveable depth_supported/hres/vres, so it is where the latch's
    // reset value, its set condition and its stickiness are observable.
    output wire        geom_dafb_live,

    // 24bpp-at-1024-wide instance.  Same 1024x768 geometry as u_hw but with
    // a 4 MB FB_BYTE_LIMIT (the VRAM_IN_DDR aperture), which is what makes
    // 1024x768x24bpp -- 3,145,728 bytes -- representable at all.  Proves the
    // aperture is the thing that gates it: the identical placement is
    // rejected by u_geom's 2 MB limit and accepted here.
    //
    // hres/vres are DRIVEABLE here (they used to be tied to 1024x768
    // constants).  The 4 MB aperture is the production VRAM_IN_DDR shape, so
    // this is the only instance in which a real shipping 24bpp placement can
    // be admission-tested -- and 1024x768 was the only geometry it had ever
    // been asked about.  Drive them to 1024/768 for the pre-existing
    // scenarios and every one of them is bit-for-bit unchanged.
    input  wire [2:0]  d24_scanout_bytes_per_px,
    input  wire [21:0] d24_scanout_fb_base_px,
    input  wire [21:0] d24_scanout_fb_stride_px,
    input  wire [11:0] d24_scanout_hres,
    input  wire [11:0] d24_scanout_vres,
    output wire [2:0]  d24_scale_n,
    output wire [21:0] d24_fb_base_px,
    output wire [21:0] d24_fb_stride_px,
    output wire [2:0]  d24_bytes_per_px,

    // ── MODE-TRANSITION instance ─────────────────────────────────────
    // The SHIPPING VRAM_IN_DDR shape (ADDR_W=22, SRC 1024x768, 4 MB
    // aperture -- rtl/soc/fpga_top_video.vh), with EVERY input driveable
    // and EVERY output observable.  The four instances above each pin some
    // subset of the input tuple to a constant, so none of them can express
    // a mode CHANGE: a real Mac OS depth/resolution switch moves base,
    // stride, bpp_shift, bytes_per_px, hres and vres, in that many separate
    // CPU register writes, and the interesting states are the intermediate
    // ones where only some of them have landed.
    input  wire [21:0] tr_scanout_fb_base_px,
    input  wire [21:0] tr_scanout_fb_stride_px,
    input  wire [2:0]  tr_scanout_bpp_shift,
    input  wire [2:0]  tr_scanout_bytes_per_px,
    input  wire        tr_scanout_depth_supported,
    input  wire [11:0] tr_scanout_hres,
    input  wire [11:0] tr_scanout_vres,
    output wire [21:0] tr_fb_base_px,
    output wire [21:0] tr_fb_stride_px,
    output wire [2:0]  tr_bpp_shift,
    output wire [2:0]  tr_bytes_per_px,
    output wire [11:0] tr_hres,
    output wire [11:0] tr_vres,
    output wire [2:0]  tr_scale_n,
    output wire [3:0]  tr_reject_reason,
    output wire        tr_rejected_sticky,
    output wire [21:0] tr_reject_base_px,
    output wire [21:0] tr_reject_stride_px,
    output wire [11:0] tr_reject_hres_px,
    output wire [11:0] tr_reject_vres_px,
    output wire [3:0]  tr_reject_reason_latched
);

    scanout_placement_sync #(
        .ADDR_W       (20),
        .SRC_W        (512),
        .SRC_H        (342),
        .FB_MAX_PIXELS(1024 * 768),
        .FB_BASE_PX   (0),
        .FB_STRIDE_PX (1024)
    ) u_rom (
        .pclk                 (pclk),
        .rst                  (rst),
        .frame_start          (frame_start),
        .scanout_fb_base_px   (rom_scanout_fb_base_px),
        .scanout_fb_stride_px (rom_scanout_fb_stride_px),
        .scanout_bpp_shift    (3'd0),
        .scanout_bytes_per_px (3'd1),
        .scanout_depth_supported (1'b1),
        .scanout_hres         (12'd512),
        .scanout_vres         (12'd342),
        .fb_base_px           (rom_fb_base_px),
        .fb_stride_px         (rom_fb_stride_px),
        .bpp_shift            (),
        .bytes_per_px         (),
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
        .dafb_live            ()
    );

    scanout_placement_sync #(
        .ADDR_W       (20),
        .SRC_W        (1024),
        .SRC_H        (768),
        .FB_MAX_PIXELS(1024 * 768),
        .FB_BASE_PX   (0),
        .FB_STRIDE_PX (1024)
    ) u_hw (
        .pclk                 (pclk),
        .rst                  (rst),
        .frame_start          (frame_start),
        .scanout_fb_base_px   (hw_scanout_fb_base_px),
        .scanout_fb_stride_px (hw_scanout_fb_stride_px),
        .scanout_bpp_shift    (3'd0),
        .scanout_bytes_per_px (3'd1),
        .scanout_depth_supported (1'b1),
        .scanout_hres         (12'd1024),
        .scanout_vres         (12'd768),
        .fb_base_px           (hw_fb_base_px),
        .fb_stride_px         (hw_fb_stride_px),
        .bpp_shift            (),
        .bytes_per_px         (),
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
        .dafb_live            ()
    );

    // Geometry + depth-gate instance.  FB_MAX_PIXELS here is the real Q700
    // build's 2 MB VRAM byte capacity (rtl/soc/fpga_top_video.vh), NOT the
    // tight SRC_W*SRC_H used by u_rom/u_hw above.  The tight bound leaves
    // literally zero headroom (base + stride*(SRC_H-1) + SRC_W-1 =
    // 786431 = FB_MAX_PIXELS-1 at base 0), so any nonzero base is rejected by
    // the range check and the depth gate could not be observed independently.
    scanout_placement_sync #(
        .ADDR_W       (20),
        .SRC_W        (1024),
        .SRC_H        (768),
        .FB_MAX_PIXELS(32'h0020_0000),
        .FB_BASE_PX   (0),
        .FB_STRIDE_PX (1024)
    ) u_geom (
        .pclk                 (pclk),
        .rst                  (rst),
        .frame_start          (frame_start),
        .scanout_fb_base_px   (geom_scanout_fb_base_px),
        .scanout_fb_stride_px (geom_scanout_fb_stride_px),
        .scanout_bpp_shift    (geom_scanout_bpp_shift),
        .scanout_bytes_per_px (geom_scanout_bytes_per_px),
        .scanout_depth_supported (geom_scanout_depth_supported),
        .scanout_hres         (geom_scanout_hres),
        .scanout_vres         (geom_scanout_vres),
        .fb_base_px           (geom_fb_base_px),
        .fb_stride_px         (geom_fb_stride_px),
        .bpp_shift            (geom_bpp_shift),
        .bytes_per_px         (geom_bytes_per_px),
        .hres                 (geom_hres),
        .vres                 (geom_vres),
        .reject_reason             (geom_reject_reason),
        .placement_rejected_sticky (geom_rejected_sticky),
        .reject_base_px            (geom_reject_base_px),
        .reject_stride_px          (geom_reject_stride_px),
        .reject_hres_px            (geom_reject_hres_px),
        .reject_vres_px            (geom_reject_vres_px),
        .reject_reason_latched     (geom_reject_reason_latched),
        .scale_n              (geom_scale_n),
        .dafb_live            (geom_dafb_live)
    );

    // 1024x768 with the VRAM_IN_DDR 4 MB aperture.  ADDR_W=22 because
    // 1024x768x24bpp reaches byte 3,145,727 and 21 bits cannot represent it
    // (this is exactly the FB_ADDR_W widening rtl/soc/fpga_top_video.vh does
    // under VRAM_IN_DDR).
    scanout_placement_sync #(
        .ADDR_W       (22),
        .SRC_W        (1024),
        .SRC_H        (768),
        .FB_MAX_PIXELS(32'h0040_0000),
        .FB_BASE_PX   (0),
        .FB_STRIDE_PX (1024)
    ) u_d24 (
        .pclk                 (pclk),
        .rst                  (rst),
        .frame_start          (frame_start),
        .scanout_fb_base_px   (d24_scanout_fb_base_px),
        .scanout_fb_stride_px (d24_scanout_fb_stride_px),
        .scanout_bpp_shift    (3'd0),
        .scanout_bytes_per_px (d24_scanout_bytes_per_px),
        .scanout_depth_supported (1'b1),
        .scanout_hres         (d24_scanout_hres),
        .scanout_vres         (d24_scanout_vres),
        .fb_base_px           (d24_fb_base_px),
        .fb_stride_px         (d24_fb_stride_px),
        .bpp_shift            (),
        .bytes_per_px         (d24_bytes_per_px),
        .hres                 (),
        .vres                 (),
        .reject_reason             (),
        .placement_rejected_sticky (),
        .reject_base_px            (),
        .reject_stride_px          (),
        .reject_hres_px            (),
        .reject_vres_px            (),
        .reject_reason_latched     (),
        .scale_n              (d24_scale_n),
        .dafb_live            ()
    );

    // Shipping VRAM_IN_DDR shape, fully driveable -- see the port block.
    scanout_placement_sync #(
        .ADDR_W       (22),
        .SRC_W        (1024),
        .SRC_H        (768),
        .FB_MAX_PIXELS(32'h0040_0000),
        .FB_BASE_PX   (0),
        .FB_STRIDE_PX (1024)
    ) u_trans (
        .pclk                 (pclk),
        .rst                  (rst),
        .frame_start          (frame_start),
        .scanout_fb_base_px   (tr_scanout_fb_base_px),
        .scanout_fb_stride_px (tr_scanout_fb_stride_px),
        .scanout_bpp_shift    (tr_scanout_bpp_shift),
        .scanout_bytes_per_px (tr_scanout_bytes_per_px),
        .scanout_depth_supported (tr_scanout_depth_supported),
        .scanout_hres         (tr_scanout_hres),
        .scanout_vres         (tr_scanout_vres),
        .fb_base_px           (tr_fb_base_px),
        .fb_stride_px         (tr_fb_stride_px),
        .bpp_shift            (tr_bpp_shift),
        .bytes_per_px         (tr_bytes_per_px),
        .hres                 (tr_hres),
        .vres                 (tr_vres),
        .reject_reason             (tr_reject_reason),
        .placement_rejected_sticky (tr_rejected_sticky),
        .reject_base_px            (tr_reject_base_px),
        .reject_stride_px          (tr_reject_stride_px),
        .reject_hres_px            (tr_reject_hres_px),
        .reject_vres_px            (tr_reject_vres_px),
        .reject_reason_latched     (tr_reject_reason_latched),
        .scale_n              (tr_scale_n),
        .dafb_live            ()
    );

endmodule

`default_nettype wire
