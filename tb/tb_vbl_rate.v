// tb_vbl_rate.v -- video_top → pulse_cdc DAFB vblank rate gate.
//
// SCOPE (corrected 2026-09-06): this gate covers the DAFB vblank PULSE
// and its pclk→pb_clk crossing.  It does NOT cover the Mac's 60 Hz tick.
//
// It used to claim it did, via `assign via1_ca1_in = dafb_vbl_level;`
// below -- but `fpga_top_peripherals.vh` has not wired VIA1's CA1 that
// way since the VIA2-PB7 board wire landed; it now reads
// `wire via1_ca1_in = via2_pb_out[7];`, i.e. the tick is a VIA2 Timer-1
// square wave, not the HDMI VTG's vblank.  Leaving the old name here made
// this look like a tick-rate gate while measuring a net the SoC does not
// build, so a wrong tick rate could never have failed it.  The tick chain
// has its own gate now: `make tb-via-tick-rate` (tb/tb_via_tick_rate.*).
//
// `dafb_vbl_level` is still live in the SoC -- it is resampled into
// core_clk to drive video.v's `frame_tick`, which is what raises the DAFB
// slot IRQ -- so the rate and CDC coverage below remain worth gating.
//
// Exposes:
//   * The pclk-domain `vbl_pulse_pclk` strobe out of video_top.
//   * The pb_clk-domain `dafb_vbl_pulse_pb` strobe out of pulse_cdc.
//   * VIA1.IFR.CA1 latching when that level is fed to a VIA1 CA1 pin
//     (edge-detect coverage only -- NOT the board's tick source).
//
// Drives clk_ref_p at 200 MHz, vram clock = pclk (skipping the CDC bridge
// inside fb_reader — we don't fetch real pixels).  The C++ harness counts
// dafb_vbl_pulse_pb pulses over a fixed simulated window and asserts the
// rate matches 60.000 Hz ± tolerance for our 1080p60 HDMI mode.

`default_nettype none

module tb_vbl_rate (
    input  wire        clk_ref,        // 200 MHz video PLL ref
    input  wire        ext_resetn,
    input  wire        pb_clk,         // 50 MHz peripheral clock
    input  wire        pb_rst,
    input  wire        phi2_tick,

    // pclk-domain frame-start strobe (1 pclk cycle wide).
    output wire        vbl_pulse_pclk,
    output wire        pclk_out,

    // pb_clk-domain pulse out of pulse_cdc.
    output wire        dafb_vbl_pulse_pb,
    // pb_clk-domain DAFB vblank level (post pulse-extender).  Named for
    // what it is: this is NOT the board's via1_ca1_in (see header).
    output wire        dafb_vbl_level_o,

    // VIA1 peripheral-bus access (drives bus in pb_clk).
    input  wire [3:0]  pb_via1_addr,
    input  wire [7:0]  pb_via1_wdata,
    input  wire        pb_via1_wr,
    input  wire        pb_via1_rd,
    output wire [7:0]  pb_via1_rdata,
    output wire        pb_via1_ack,

    output wire        via1_irq
);

    // ── video_top instance ────────────────────────────────────────────
    wire mmcm_locked, hdmi_i2c_done, fb_underflow_sticky;
    /* verilator lint_off UNUSEDSIGNAL */
    wire [15:0] req_count, rsp_count, miss_count;
    wire [11:0] dbg_h;
    wire [10:0] dbg_v;
    wire [23:0] dbg_rgb;
    wire        dbg_de, dbg_hs, dbg_vs;
    wire [23:0] al9134_d_w;
    wire        al9134_hs_w, al9134_vs_w, al9134_de_w, al9134_clk_w;
    wire        al9134_resetn_w;
    wire        al9134_scl_w, al9134_sda_w;
    wire        vram_rd_en;
    wire [19:0] vram_rd_addr;
    /* verilator lint_on UNUSEDSIGNAL */
    // VRAM read port -- video_top's FETCH_W is the DATA width of the
    // byte-ADDRESSED streaming port, which returns a 4-byte group per request
    // at every pixel depth (the runtime depth arrives on
    // scanout_bytes_per_px instead).  This tb never returns real pixels; it
    // only measures VBL cadence.
    wire [31:0] vram_rd_data = 32'd0;
    wire        vram_rd_valid = 1'b0;

    video_top #(
        .SRC_W   (1024),
        .SRC_H   (768),
        .FETCH_W (32),
        .ADDR_W  (20),
        .TEST_PATTERN(0)
    ) u_video (
        .clk_ref_p     (clk_ref),
        .clk_ref_n     (clk_ref),
        .ext_resetn    (ext_resetn),
        .vram_clk      (clk_ref),
        .vram_rst      (~ext_resetn),
        // Coupled scan-out reset (task #194) -- see tb_video_top.v.  This
        // target is -Wall warnings-as-errors, so every output pin must be
        // connected, even to nothing.
        .vram_side_rst (),
        .dbg_video_snap (),
        .dbg_video_place(),
        .pclk_out      (pclk_out),
        .mmcm_locked   (mmcm_locked),
        .hdmi_i2c_done (hdmi_i2c_done),
        .fb_underflow_sticky(fb_underflow_sticky),
        .fb_reader_req_count(req_count),
        .fb_reader_rsp_count(rsp_count),
        .fb_reader_miss_count(miss_count),
        .debug_hcount  (dbg_h),
        .debug_vcount  (dbg_v),
        .debug_rgb     (dbg_rgb),
        .debug_de      (dbg_de),
        .debug_hs      (dbg_hs),
        .debug_vs      (dbg_vs),
        .vbl_pulse_pclk(vbl_pulse_pclk),
        .al9134_scl    (al9134_scl_w),
        .al9134_sda    (al9134_sda_w),
        .al9134_d      (al9134_d_w),
        .al9134_hs     (al9134_hs_w),
        .al9134_vs     (al9134_vs_w),
        .al9134_de     (al9134_de_w),
        .al9134_clk    (al9134_clk_w),
        .al9134_resetn (al9134_resetn_w),
        .al9134_int    (1'b0),
        .scanout_fb_base_px  (20'd0),
        .scanout_fb_stride_px(20'd1024),
        .scanout_bpp_shift   (3'd0),
        .scanout_bytes_per_px(3'd1),
        .scanout_depth_supported (1'b1),
        .scanout_hres        (12'd1024),
        .scanout_vres        (12'd768),
        .scanout_clut_we     (1'b0),
        .scanout_clut_waddr  (8'd0),
        .scanout_clut_wdata  (24'd0),
        .vram_rd_en    (vram_rd_en),
        .vram_rd_addr  (vram_rd_addr),
        .vram_rd_data  (vram_rd_data),
        .vram_rd_valid (vram_rd_valid)
    );

    // ── pulse_cdc: pclk → pb_clk pulse synchroniser ───────────────────
    pulse_cdc u_dafb_vbl_cdc (
        .src_clk  (pclk_out),
        .src_rst  (1'b0),
        .src_pulse(vbl_pulse_pclk),
        .dst_clk  (pb_clk),
        .dst_rst  (pb_rst),
        .dst_pulse(dafb_vbl_pulse_pb)
    );

    // ── Pulse extender to drive VIA1 CA1 level ───────────────────────
    localparam integer VBL_LEVEL_TICKS = 32;
    reg [5:0] dafb_vbl_extend;
    reg       dafb_vbl_level;
    always @(posedge pb_clk) begin
        if (pb_rst) begin
            dafb_vbl_extend <= 6'd0;
            dafb_vbl_level  <= 1'b0;
        end else if (dafb_vbl_pulse_pb) begin
            dafb_vbl_extend <= VBL_LEVEL_TICKS[5:0];
            dafb_vbl_level  <= 1'b1;
        end else if (dafb_vbl_extend != 6'd0) begin
            dafb_vbl_extend <= dafb_vbl_extend - 6'd1;
            if (dafb_vbl_extend == 6'd1)
                dafb_vbl_level <= 1'b0;
        end
    end

    assign dafb_vbl_level_o = dafb_vbl_level;

    // ── VIA1 instance ─────────────────────────────────────────────────
    /* verilator lint_off UNUSEDSIGNAL */
    wire [7:0] via1_pa_out, via1_pa_mask, via1_pb_out, via1_pb_mask;
    wire       via1_overlay_bit;
    wire       via1_adb_rx_ready;
    wire [7:0] via1_adb_tx_byte;
    wire       via1_adb_tx_valid;
    wire       via1_rtc_enb, via1_rtc_clk;
    wire       via1_rtc_data_o, via1_rtc_data_oe;
    /* verilator lint_on UNUSEDSIGNAL */

    via1 #(
        .ENABLE_INTERNAL_VBL(1'b0)
    ) u_via1 (
        .clk         (pb_clk),
        .rst         (pb_rst),
        .phi2_tick   (phi2_tick),
        .pb_addr     (pb_via1_addr),
        .pb_wdata    (pb_via1_wdata),
        .pb_wr       (pb_via1_wr),
        .pb_rd       (pb_via1_rd),
        .pb_rdata    (pb_via1_rdata),
        .pb_ack      (pb_via1_ack),
        .pa_in       (8'hC1),
        .pb_in       (8'h00),
        // CB1/CB2 are unused by this tb (it measures VBL cadence off CA1
        // only), but the pins must be CONNECTED: this target builds -Wall
        // with warnings-as-errors, so an unconnected pin is a build
        // failure.  Tying the inputs to 0 matches what they already were
        // -- Verilator's --x-initial fast gave undriven nets 0 -- so this
        // changes no behaviour, it only lets the target build.
        .cb1_in      (1'b0),
        .cb2_in      (1'b0),
        .cb2_out     (),
        .cb2_oe      (),
        .pa_out      (via1_pa_out),
        .pa_mask     (via1_pa_mask),
        .pb_out      (via1_pb_out),
        .pb_mask     (via1_pb_mask),
        .overlay_bit (via1_overlay_bit),
        .adb_rx_byte (8'h00),
        .adb_rx_valid(1'b0),
        .adb_rx_ready(via1_adb_rx_ready),
        .adb_tx_byte (via1_adb_tx_byte),
        .adb_tx_valid(via1_adb_tx_valid),
        .rtc_enb     (via1_rtc_enb),
        .rtc_clk     (via1_rtc_clk),
        .rtc_data_o  (via1_rtc_data_o),
        .rtc_data_oe (via1_rtc_data_oe),
        .rtc_data_i  (1'b0),
        .rtc_cko     (1'b1),
        .vblank_irq_in(dafb_vbl_level),
        .irq         (via1_irq)
    );

endmodule

`default_nettype wire
