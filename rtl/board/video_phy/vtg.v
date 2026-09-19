// vtg.v -- Video Timing Generator
// ---------------------------------------------------------------------------
// Ported verbatim from ~/sd-hdmi-bringup/rtl/vtg.v for the m68k-ooo
// mac-on-FPGA project.  Module body unchanged.
//
// Generates CEA-861 HDMI sync + blanking signals. Parameters default to
// 1920x1080p60 (pixel clock = 148.5 MHz); override for 1280x720p60 timing
// with H_ACTIVE=1280, H_FP=110, H_SYNC=40, H_BP=220,
//      V_ACTIVE=720,  V_FP=5,   V_SYNC=5,  V_BP=20.
//
// Both 720p60 and 1080p60 use positive HS/VS polarity.
// Counters are 12-bit h / 11-bit v so any resolution up to 4096x2048 fits.
//
// Outputs hcount/vcount during active region only (0-based).
// 'de' (data enable) is high only during active pixels.
// Sync signals are registered -- HS/VS are output one pclk cycle after
// the corresponding blanking boundary (standard display pipeline).
//
// Latency: 1 pclk cycle (registered outputs).
// ---------------------------------------------------------------------------
`default_nettype none

module vtg #(
    parameter H_ACTIVE = 1920,
    parameter H_FP     = 88,
    parameter H_SYNC   = 44,
    parameter H_BP     = 148,
    parameter V_ACTIVE = 1080,
    parameter V_FP     = 4,
    parameter V_SYNC   = 5,
    parameter V_BP     = 36
) (
    input  wire        pclk,
    input  wire        resetn,       // active-low, sync to pclk
    output reg  [11:0] hcount,       // 0 .. H_ACTIVE-1, valid when de
    output reg  [10:0] vcount,       // 0 .. V_ACTIVE-1, valid when de
    output reg         de,           // data enable (active pixel)
    output reg         hsync,        // horizontal sync (active-high)
    output reg         vsync         // vertical sync   (active-high)
);

    // Derived totals (1080p60: 2200 x 1125, 720p60: 1650 x 750)
    localparam H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP;
    localparam V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP;

    localparam H_SYNC_START = H_ACTIVE + H_FP;
    localparam H_SYNC_END   = H_ACTIVE + H_FP + H_SYNC;
    localparam V_SYNC_START = V_ACTIVE + V_FP;
    localparam V_SYNC_END   = V_ACTIVE + V_FP + V_SYNC;

    reg [11:0] hcnt;
    reg [10:0] vcnt;

    wire h_last = (hcnt == H_TOTAL - 1);
    wire v_last = (vcnt == V_TOTAL - 1);

    always @(posedge pclk) begin
        if (~resetn) begin
            hcnt   <= 12'd0;
            vcnt   <= 11'd0;
            de     <= 1'b0;
            hsync  <= 1'b0;
            vsync  <= 1'b0;
            hcount <= 12'd0;
            vcount <= 11'd0;
        end else begin
            if (h_last)
                hcnt <= 12'd0;
            else
                hcnt <= hcnt + 12'd1;

            if (h_last) begin
                if (v_last)
                    vcnt <= 11'd0;
                else
                    vcnt <= vcnt + 11'd1;
            end

            de    <= (hcnt < H_ACTIVE) && (vcnt < V_ACTIVE);
            hsync <= (hcnt >= H_SYNC_START) && (hcnt < H_SYNC_END);
            vsync <= (vcnt >= V_SYNC_START) && (vcnt < V_SYNC_END);

            hcount <= hcnt;
            vcount <= vcnt;
        end
    end

endmodule
`default_nettype wire
