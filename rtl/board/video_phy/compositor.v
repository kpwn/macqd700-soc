// compositor.v -- stage 12 of the scan-out pipeline: CENTRING (and the logo).
// ---------------------------------------------------------------------------
// active region + borders -> the full DST frame.  docs/video_path_review.md
// S4.1 stage 12.  It owns centring and nothing else, plus the boot logo, which
// the review names as its natural home ("currently tangled into the scaler").
//
// ONE EXPRESSION FOR THE ACTIVE WINDOW.  place_plan.v (stage 4) decides WHERE
// the window is; this module is the ONLY place that turns that decision into a
// per-pixel in/out answer, and it EXPORTS that answer (`in_active_x`,
// `in_active_y`, `in_border`) rather than letting anyone else re-derive it.
// The source-coordinate walk (upscale.v) and the line-store read address
// (scanout_display.v) both consume these outputs.  That is the review's rule
// that exactly one expression for the active window exists in the design; the
// export direction is what enforces it.
//
// >>> THE LATENCY LADDER.  This module carries the CONTROL half of a contract
// >>> whose DATA half is in scanout_display.v.  Read that file's LATENCY
// >>> LADDER note before changing CTRL_DELAY or adding a register here.
// >>>
// >>> The data path from a combinational `x_src` to `rgb` is five registered
// >>> stages: line_rd_addr, pix_data_pipe, pix_data_pipe2 (scanout_display),
// >>> clut/bypass (clut.v), and this module's output register.  So everything
// >>> sampled combinationally at stage 0 -- de/hs/vs, the border decision, the
// >>> pixel-valid gate and the splash pixel -- must be delayed by exactly
// >>> CTRL_DELAY = 4 before the output register makes five.
// >>>
// >>> An off-by-one lands precisely at `border_x`, the first pixels of a line,
// >>> and that is a symptom that has actually been reported off hardware.  If
// >>> you shorten one pipe, shorten them ALL and the data path with them.
//
// BOOT SPLASH.  While `dafb_live` is low (the DAFB has not yet committed a
// renderable placement -- see scanout_placement_sync.v) the output stage
// SUBSTITUTES a centred, integer-scaled 32x32 monochrome logo for the normal
// pixel path.  It is deliberately a display-side substitution ONLY:
//
//   * every splash term is combinational off `hcount`/`vcount`, the same
//     counters the border/DE logic already uses, then piped CTRL_DELAY deep
//     so it lands on the same stage the final mux reads;
//   * it touches NOTHING on the fetch side -- not disp_y, disp_ready,
//     disp_slot, credit_return, the read request, pix_valid, frame_delivered
//     or line_underflow_sticky -- so the credit ring cannot tell it exists and
//     it generates zero DDR/VRAM traffic.  docs/scanout_credit_ring.md
//     catalogues four separate interaction bugs in that ring; the splash is
//     kept structurally incapable of becoming a fifth.  Moving it here from
//     the scaler makes that separation STRUCTURAL rather than a convention:
//     this module has no fetch-side port to reach;
//   * the splash does NOT go through the CLUT.  The palette comes up all-zero
//     and is only programmed by the OS later, so a CLUT-routed splash would
//     be invisible for exactly as long as the splash is meant to be up;
//   * `splash_en` is re-latched from `dafb_live` only at `sof`, so the
//     switchover always lands on a frame boundary and can never tear.
//
// DSP: none, deliberately, and this is the stage where the review most
// expects one (S4.1 lists "x/y against border_x/border_y" as an ALU
// compare/pattern-detect fit).  Two things rule it out here.  First, the
// window compare must stay COMBINATIONAL: `in_border` feeds the line-store
// read request and `in_active_x` feeds the source walk in the SAME cycle, so
// registering it does not deepen a pipeline, it re-times two consumers.
// Second, the one operand worth pre-computing (`border_x + active_w`) is
// frame-constant, so registering it would be free of throughput cost but
// would skew by one cycle at a placement commit -- and placement commits at a
// mode switch are exactly what tb_scanout_ddr_frames exercises.  A DSP's value
// in this path is its free A/B/M/P pipeline registers (S4.1), and this stage
// cannot spend them.  The genuine registered-arithmetic nodes the review names
// are in the ADDRESS path, not here.
// ---------------------------------------------------------------------------
`default_nettype none

module compositor #(
    // Boot-splash pixel replication factor.  POWER OF TWO ONLY -- the walk
    // divides by it with a right shift.  4 => 128x128 on screen, 8 => 256x256.
    parameter SPLASH_SCALE = 4,
    // Control-pipe depth.  MUST equal the number of registered stages the
    // pixel data crosses before it reaches this module's output register.
    // See the LATENCY LADDER note above; this is not a tuning knob.
    parameter CTRL_DELAY   = 4
) (
    input  wire        clk,
    input  wire        resetn,

    // -- Raster position + sync, from the VTG -------------------------
    input  wire [11:0] hcount,
    input  wire [10:0] vcount,
    input  wire        de_in,
    input  wire        hs_in,
    input  wire        vs_in,
    input  wire        sof,

    // -- Committed placement, from place_plan (stage 4) ---------------
    // CONSUMED, never re-derived.  There is no scaling policy in this file.
    input  wire [12:0] active_w_px,
    input  wire [12:0] active_h_px,
    input  wire [11:0] border_x_px,
    input  wire [11:0] border_y_px,

    // 1 once the DAFB has committed a renderable placement (sticky, and only
    // ever changes on a frame boundary).  While 0, the boot splash is shown.
    input  wire        dafb_live,

    // -- The one active-window expression, exported upstream ----------
    output wire        in_active_x,
    output wire        in_active_y,
    output wire        in_border,

    // -- Pixel candidates ---------------------------------------------
    // Combinational, stage 0: this destination pixel has a valid source byte.
    input  wire        pix_valid_in,
    // Depth select, latched upstream; chooses between the two candidates.
    input  wire        fetch_direct,
    // Both arrive CTRL_DELAY stages after the stage-0 signals above.
    input  wire [23:0] clut_rgb,
    input  wire [23:0] direct_rgb,

    output reg  [23:0] rgb,
    output reg         de_out,
    output reg         hs_out,
    output reg         vs_out
);

    // -- The active window --------------------------------------------
    assign in_active_x = (hcount >= {1'b0, border_x_px}) &&
                         ({1'b0, hcount} < ({1'b0, border_x_px} + active_w_px));
    assign in_active_y = (vcount >= {1'b0, border_y_px[10:0]}) &&
                         ({1'b0, vcount} < ({1'b0, border_y_px[10:0]} + active_h_px));
    assign in_border   = ~(in_active_x & in_active_y);

    // -- Boot splash bitmap ROM ---------------------------------------
    // 32 rows x 32 bits, 1bpp.  Bit x (LSB-first) is COLUMN x, so bit 0 is
    // the LEFTMOST pixel of the row; a set bit is logo (foreground).  The
    // ASCII in the trailing comments is the literal render of each word
    // under that rule -- it is the check on the bit order, kept in-line so a
    // future edit that mirrors or transposes the art is visible in review.
    //
    // This is a constant FUNCTION, not a memory: no reg array, no write
    // port, no address register, so it cannot infer BRAM or LUTRAM.  It
    // synthesises as combinational logic -- 5 address bits x 32 output bits,
    // i.e. a LUT5 per output bit, two packed per LUT6 -- and the only
    // consumer is the single bit `splash_word[splash_col]`, so the ROM and
    // the 32:1 bit select collapse into one 10-input function.  Expected
    // cost is a few tens of LUTs and zero BRAM.  NOT MEASURED: this repo's
    // synth flow is not run here, so treat the number as an inference from
    // the RTL shape, not a utilisation report.
    function [31:0] splash_bitmap;
        input [4:0] r;
        begin
            case (r)
                5'd0 : splash_bitmap = 32'h00000000;  // ................................
                5'd1 : splash_bitmap = 32'h00000a00;  // .........#.#....................
                5'd2 : splash_bitmap = 32'h00000400;  // ..........#.....................
                5'd3 : splash_bitmap = 32'h00005540;  // ......#.#.#.#.#.................
                5'd4 : splash_bitmap = 32'h00007fc0;  // ......#########.................
                5'd5 : splash_bitmap = 32'h00003f80;  // .......#######..................
                5'd6 : splash_bitmap = 32'h00003f80;  // .......#######..................
                5'd7 : splash_bitmap = 32'h00001f00;  // ........#####...................
                5'd8 : splash_bitmap = 32'h00001f00;  // ........#####...................
                5'd9 : splash_bitmap = 32'h00001f00;  // ........#####...................
                5'd10: splash_bitmap = 32'h00003f80;  // .......#######..................
                5'd11: splash_bitmap = 32'h0000ffe0;  // .....###########................
                5'd12: splash_bitmap = 32'h00003f80;  // .......#######..................
                5'd13: splash_bitmap = 32'h00003f80;  // .......#######..................
                5'd14: splash_bitmap = 32'h00003f83;  // ##.....#######..................
                5'd15: splash_bitmap = 32'h00103f9f;  // #####..#######......#...........
                5'd16: splash_bitmap = 32'h18103ffb;  // ##.###########......#......##...
                5'd17: splash_bitmap = 32'h0e3fffd5;  // #.#.#.################...###....
                5'd18: splash_bitmap = 32'h1beabfab;  // ##.#.#.#######.#.#.#.#####.##...
                5'd19: splash_bitmap = 32'h480d7fd5;  // #.#.#.#########.#.##.......#..#.
                5'd20: splash_bitmap = 32'hf80abfab;  // ##.#.#.#######.#.#.#.......#####
                5'd21: splash_bitmap = 32'h480d7fd5;  // #.#.#.#########.#.##.......#..#.
                5'd22: splash_bitmap = 32'h1beabfab;  // ##.#.#.#######.#.#.#.#####.##...
                5'd23: splash_bitmap = 32'h0e3fffd5;  // #.#.#.################...###....
                5'd24: splash_bitmap = 32'h18107ffb;  // ##.############.....#......##...
                5'd25: splash_bitmap = 32'h00107fdf;  // #####.#########.....#...........
                5'd26: splash_bitmap = 32'h00007fc3;  // ##....#########.................
                5'd27: splash_bitmap = 32'h0000ffe0;  // .....###########................
                5'd28: splash_bitmap = 32'h0000ffe0;  // .....###########................
                5'd29: splash_bitmap = 32'h0000ffe0;  // .....###########................
                5'd30: splash_bitmap = 32'h0001fff0;  // ....#############...............
                default: splash_bitmap = 32'h0001fff0;// ....#############...............
            endcase
        end
    endfunction

    // -- Boot splash geometry -----------------------------------------
    // Centred on the ACTIVE WINDOW this module already computed, not on a
    // hardcoded 640x480 -- the machine also runs 832x624 and 1024x768, and
    // pre-DAFB it runs with hres/vres still 0 (active_w = active_h = 0,
    // border_x = DST_W/2, border_y = DST_H/2), which degenerates to the
    // centre of the display.  All of it is combinational off hcount/vcount.
    localparam integer SPLASH_SHIFT = $clog2(SPLASH_SCALE);
    localparam integer SPLASH_PIX   = 32 * SPLASH_SCALE;
    localparam [13:0]  SPLASH_HALF  = SPLASH_PIX / 2;
    localparam [23:0]  SPLASH_FG    = 24'hFF_FF_FF;
    localparam [23:0]  SPLASH_BG    = 24'h00_00_00;

    // Centre of the active window = its left/top edge plus half its extent.
    wire [13:0] splash_mid_x = {2'b00, border_x_px} + {2'b00, active_w_px[12:1]};
    wire [13:0] splash_mid_y = {2'b00, border_y_px} + {2'b00, active_h_px[12:1]};
    wire [13:0] splash_x0 = (splash_mid_x > SPLASH_HALF)
                          ? (splash_mid_x - SPLASH_HALF) : 14'd0;
    wire [13:0] splash_y0 = (splash_mid_y > SPLASH_HALF)
                          ? (splash_mid_y - SPLASH_HALF) : 14'd0;

    wire [13:0] splash_dx = {2'b00, hcount} - splash_x0;
    wire [13:0] splash_dy = {3'b000, vcount} - splash_y0;
    wire in_splash_x = ({2'b00,  hcount} >= splash_x0)
                    && (splash_dx < SPLASH_PIX);
    wire in_splash_y = ({3'b000, vcount} >= splash_y0)
                    && (splash_dy < SPLASH_PIX);
    wire [4:0] splash_col  = splash_dx[SPLASH_SHIFT+4:SPLASH_SHIFT];
    wire [4:0] splash_rowi = splash_dy[SPLASH_SHIFT+4:SPLASH_SHIFT];
    wire [31:0] splash_word = splash_bitmap(splash_rowi);
    // Bit x (LSB-first) is COLUMN x, so bit 0 is the LEFTMOST pixel.
    wire splash_pix = in_splash_x && in_splash_y && splash_word[splash_col];

    // -- Control pipes -------------------------------------------------
    // Bit 0 of each vector is stage 1 (newest), bit CTRL_DELAY-1 the stage
    // aligned with clut_rgb/direct_rgb that the final mux reads.  ALL of them
    // are the same depth by construction -- one parameter, one index -- so a
    // future edit cannot shorten one and leave the others.
    localparam integer CTRL_TOP = CTRL_DELAY - 1;

    // NEGATIVE-CONTROL FAULT INJECTION -- never defined in a production or
    // sim build.  `make tb-scanout-display-latency-negctl` elaborates this
    // file with -DSCANOUT_INJECT_SHORT_BORDER_PIPE and REQUIRES the lockstep
    // gate to fail; a green run there is the real failure ("the gate cannot
    // see the thing it gates").  Without that proof, "every pipe measured 5"
    // would be equally consistent with a checker that measures nothing.
    //
    // The injected fault is deliberately the exact one the hardware symptom
    // points at: the border decision arriving one cycle BEFORE the pixel it
    // is supposed to gate, which lights the column at border_x - 1 and blanks
    // the last column of the active window.
`ifdef SCANOUT_INJECT_SHORT_BORDER_PIPE
    localparam integer CTRL_TOP_BORDER = CTRL_DELAY - 2;
`else
    localparam integer CTRL_TOP_BORDER = CTRL_TOP;
`endif

    reg [CTRL_DELAY-1:0] pix_valid_pipe;
    reg [CTRL_DELAY-1:0] de_pipe;
    reg [CTRL_DELAY-1:0] hs_pipe;
    reg [CTRL_DELAY-1:0] vs_pipe;
    reg [CTRL_DELAY-1:0] border_pipe;
    reg [CTRL_DELAY-1:0] splash_pipe;

    // Registered once per frame at `sof`, never mid-frame: the splash can
    // only turn off on a frame boundary.  Reset HIGH so the very first frame
    // after reset -- long before the ROM has touched the DAFB -- is a splash
    // frame rather than a black one.
    reg splash_en;

    always @(posedge clk) begin
        if (!resetn) begin
            pix_valid_pipe <= {CTRL_DELAY{1'b0}};
            de_pipe        <= {CTRL_DELAY{1'b0}};
            hs_pipe        <= {CTRL_DELAY{1'b0}};
            vs_pipe        <= {CTRL_DELAY{1'b0}};
            border_pipe    <= {CTRL_DELAY{1'b1}};
            splash_pipe    <= {CTRL_DELAY{1'b0}};
            splash_en      <= 1'b1;
            rgb            <= 24'h00_00_00;
            de_out         <= 1'b0;
            hs_out         <= 1'b0;
            vs_out         <= 1'b0;
        end else begin
            // The ONLY place splash_en moves.  dafb_live is itself only
            // updated on the frame boundary upstream, so by the time it is
            // sampled here it has been stable for a whole frame.
            if (sof)
                splash_en <= ~dafb_live;

            pix_valid_pipe <= {pix_valid_pipe[CTRL_DELAY-2:0], pix_valid_in};
            de_pipe        <= {de_pipe[CTRL_DELAY-2:0],        de_in};
            hs_pipe        <= {hs_pipe[CTRL_DELAY-2:0],        hs_in};
            vs_pipe        <= {vs_pipe[CTRL_DELAY-2:0],        vs_in};
            border_pipe    <= {border_pipe[CTRL_DELAY-2:0],    in_border};
            splash_pipe    <= {splash_pipe[CTRL_DELAY-2:0],    splash_pix};

            // Boot splash SUBSTITUTES for the whole pixel path -- it wins
            // over the border too, because before the DAFB is programmed
            // hres/vres are 0, the active window is empty and EVERY visible
            // pixel is border.  Nothing above this point is conditioned on
            // splash_en, so the fetch side is bit-identical either way.
            if (splash_en) begin
                rgb <= (de_pipe[CTRL_TOP] && splash_pipe[CTRL_TOP])
                       ? SPLASH_FG : SPLASH_BG;
            end else if (border_pipe[CTRL_TOP_BORDER] || !pix_valid_pipe[CTRL_TOP]) begin
                rgb <= 24'h00_00_00;
            end else if (fetch_direct) begin
                rgb <= direct_rgb;
            end else begin
                rgb <= clut_rgb;
            end
            de_out <= de_pipe[CTRL_TOP];
            hs_out <= hs_pipe[CTRL_TOP];
            vs_out <= vs_pipe[CTRL_TOP];
        end
    end

endmodule

`default_nettype wire
