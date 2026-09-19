// mmcm_hdmi.v
// ---------------------------------------------------------------------------
// MMCME4_ADV wrapper — generates the HDMI pixel clock from the board
// reference clock.
//
// Ported verbatim (module body unchanged) from ~/sd-hdmi-bringup/rtl/mmcm_hdmi.v
// for the m68k-ooo mac-on-FPGA project.  Keeps the CLKOUT0_DIVIDE_F=10.000
// default so pclk = 148.5 MHz (1920x1080p60).  The CLKOUT0_DIVIDE_F=20.0
// 720p60 variant is retained for optional use.
//
// Default PLL math:
//   CLKIN  = 200 MHz  -> CLKIN1_PERIOD = 5.000 ns
//   DIVCLK_DIVIDE = 5  -> VCO input = 40 MHz
//   CLKFBOUT_MULT_F = 37.125  -> VCO = 1485.000 MHz  (in-range for KU5P -2)
//   CLKOUT0_DIVIDE_F = 10.0   -> PCLK = 148.5 MHz  (1920x1080p60)
//   CLKOUT0_DIVIDE_F = 20.0   -> PCLK = 74.25 MHz  (1280x720p60)
//
// Ports:
//   clk_in_p / clk_in_n  — differential input or externally buffered input
//   resetn               — active-low async reset
//   pclk                 — pixel clock output (BUFG'd)
//   locked               — MMCM LOCKED (high when stable)
// ---------------------------------------------------------------------------
`default_nettype none

module mmcm_hdmi #(
    // Default VCO = 200 MHz / DIVCLK_DIVIDE * CLKFBOUT_MULT_F = 1485 MHz.
    // pclk = VCO / CLKOUT0_DIVIDE_F.
    //   10.0 -> 148.5 MHz  (1920x1080p60)   [default — m68k-ooo]
    //   20.0 ->  74.25 MHz (1280x720p60)
    parameter real CLKIN1_PERIOD     = 5.000,
    parameter integer DIVCLK_DIVIDE  = 5,
    parameter real CLKFBOUT_MULT_F   = 37.125,
    parameter real CLKOUT0_DIVIDE_F  = 10.000,
    // When EXTERNAL_IBUFDS = 1, treat `clk_in_p` as an already-buffered
    // single-ended clock (skip the internal IBUFDS).  Required when
    // multiple consumers share the same differential input pin pair —
    // Vivado disallows two IBUFDS on the same IO port.  fpga_top uses
    // this mode and feeds the top-level buffered sys_clk in.  In the
    // stand-alone sd-hdmi-bringup module (and default), EXTERNAL_IBUFDS = 0
    // and mmcm_hdmi buffers its own diff pair.  `clk_in_n` is ignored
    // in external-ibufds mode.
    parameter integer EXTERNAL_IBUFDS = 0,
    // ── CLKOUT1: the pixel clock again, phase-shifted, for OFF-CHIP FORWARDING ──
    // Phase of CLKOUT1 (`pclk_fwd`) relative to CLKOUT0 (`pclk`), in degrees.
    // 90.0 puts a QUARTER PERIOD between the RGB data launch (pclk rising) and
    // BOTH edges of the clock that is forwarded to the encoder.  See the long
    // note at video_top's ODDRE1 for why a quarter and not a half.
    //
    // ⚠️ CLKOUT1_DIVIDE is an INTEGER attribute -- only CLKOUT0 has the
    // fractional `_F` form.  So CLKOUT1_DIVIDE_I must equal CLKOUT0_DIVIDE_F
    // exactly AND be a whole number, which constrains the VCO: at 74.25 MHz
    // pclk the VCO must be 74.25 * CLKOUT1_DIVIDE_I.  74.25 MHz cannot be
    // reached from a 100 MHz reference with an integer CLKOUT divide unless
    // DIVCLK_DIVIDE/CLKFBOUT_MULT_F are chosen for it -- fpga_top_video.vh
    // does exactly that (DIVCLK 5 / MULT 74.25 -> VCO 1485 -> /20).
    parameter real    CLKOUT1_PHASE_DEG = 90.000,
    parameter integer CLKOUT1_DIVIDE_I  = 10
) (
    input  wire clk_in_p,
    input  wire clk_in_n,
    input  wire resetn,
    output wire pclk,
    // Same frequency as pclk, shifted by CLKOUT1_PHASE_DEG.  Intended ONLY as
    // the C input of the ODDRE1 that forwards the pixel clock off-chip -- it
    // must not clock logic, or it becomes a second pixel-clock domain.
    output wire pclk_fwd,
    output wire locked
);

    wire clk_in_buf;
    wire clkfb;
    wire pclk_unbuf;
    wire pclk_fwd_unbuf;

`ifdef VERILATOR
    // In Verilator we can't model Xilinx primitives.  Pass through the
    // p input as the internal clock and let the test drive it at whatever
    // rate it likes.  `locked` is asserted after reset release.
    assign clk_in_buf = clk_in_p;
    assign pclk       = clk_in_buf;
    // No phase shift is modellable in Verilator; the forwarded-clock domain is
    // not exercised functionally (video_top's `ifdef VERILATOR branch forwards
    // pclk directly), so pass pclk through.
    assign pclk_fwd   = clk_in_buf;

    reg locked_r = 1'b0;
    always @(posedge clk_in_buf or negedge resetn) begin
        if (~resetn) locked_r <= 1'b0;
        else         locked_r <= 1'b1;
    end
    assign locked = locked_r;

    // tie off unused nets so Verilator doesn't complain
    wire _unused_ok = &{1'b0, clk_in_n, clkfb, pclk_unbuf, pclk_fwd_unbuf, 1'b0};
`else
    // Differential input buffer (skipped when EXTERNAL_IBUFDS=1 — caller
    // already provides a buffered single-ended clock on clk_in_p).
    generate
        if (EXTERNAL_IBUFDS == 0) begin : g_ibufds
            IBUFDS #(
                .DIFF_TERM    ("FALSE"),
                .IBUF_LOW_PWR ("FALSE"),
                .IOSTANDARD   ("DIFF_SSTL12")
            ) u_ibufds (
                .I  (clk_in_p),
                .IB (clk_in_n),
                .O  (clk_in_buf)
            );
        end else begin : g_no_ibufds
            assign clk_in_buf = clk_in_p;
            // clk_in_n unused when external-ibufds mode active
            /* verilator lint_off UNUSED */
            wire _unused_clk_in_n = clk_in_n;
            /* verilator lint_on UNUSED */
        end
    endgenerate

    // MMCM
    MMCME4_ADV #(
        .BANDWIDTH            ("OPTIMIZED"),
        .CLKIN1_PERIOD        (CLKIN1_PERIOD),
        .DIVCLK_DIVIDE        (DIVCLK_DIVIDE),
        .CLKFBOUT_MULT_F      (CLKFBOUT_MULT_F),
        .CLKFBOUT_PHASE       (0.000),
        .CLKOUT0_DIVIDE_F     (CLKOUT0_DIVIDE_F),
        .CLKOUT0_PHASE        (0.000),
        .CLKOUT0_DUTY_CYCLE   (0.500),
        // CLKOUT1 = pclk shifted by CLKOUT1_PHASE_DEG.  CLKOUT1_DIVIDE is an
        // INTEGER attribute (only CLKOUT0 has the fractional _F form), so the
        // divide must be a whole number -- true for every mode this MMCM is
        // parameterised for (10 -> 148.5 MHz, 20 -> 74.25 MHz, 12.5 is used
        // only on the AB7/AB6 100 MHz reference where CLKFBOUT_MULT_F differs;
        // see the $error guard in video_top).
        .CLKOUT1_DIVIDE       (CLKOUT1_DIVIDE_I),
        .CLKOUT1_PHASE        (CLKOUT1_PHASE_DEG),
        .CLKOUT1_DUTY_CYCLE   (0.500),
        .CLKOUT4_CASCADE      ("FALSE"),
        .COMPENSATION         ("INTERNAL"),
        .STARTUP_WAIT         ("FALSE"),
        .REF_JITTER1          (0.010)
    ) u_mmcm (
        .CLKIN1   (clk_in_buf),
        .CLKIN2   (1'b0),
        .CLKINSEL (1'b1),
        .CLKFBIN  (clkfb),
        .CLKFBOUT (clkfb),
        .CLKOUT0  (pclk_unbuf),
        .CLKOUT1  (pclk_fwd_unbuf),
        // unused outputs
        .CLKOUT0B (), .CLKOUT1B (), .CLKOUT2  (),
        .CLKOUT2B (), .CLKOUT3  (), .CLKOUT3B (), .CLKOUT4  (),
        .CLKOUT5  (), .CLKOUT6  (), .CLKFBOUTB(),
        .LOCKED   (locked),
        .PWRDWN   (1'b0),
        .RST      (~resetn),
        .PSCLK    (1'b0), .PSEN (1'b0), .PSINCDEC (1'b0),
        .PSDONE   (), .CLKINSTOPPED (), .CLKFBSTOPPED (),
	        .DCLK     (1'b0), .DEN (1'b0), .DWE (1'b0),
	        .DADDR    (7'd0), .DI  (16'd0), .DO  (), .DRDY (),
	        .CDDCDONE (), .CDDCREQ (1'b0)
	    );

    // Global clock buffer for pixel clock
    BUFG u_bufg_pclk (
        .I (pclk_unbuf),
        .O (pclk)
    );

    // Second BUFG for the phase-shifted FORWARDING clock.  It drives exactly
    // one load -- the ODDRE1 in video_top that pushes the clock off-chip -- so
    // it is not a second logic domain.  A BUFG (rather than a direct route) is
    // required because the ODDRE1/OSERDESE3 C pin must be driven by a global
    // clock resource.
    BUFG u_bufg_pclk_fwd (
        .I (pclk_fwd_unbuf),
        .O (pclk_fwd)
    );
`endif // VERILATOR

endmodule
`default_nettype wire
