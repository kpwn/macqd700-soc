// vram_smoke.v — Reset-time VRAM preload writer ("smoke pattern").
// ---------------------------------------------------------------------------
// Purpose
//   De-risk the HDMI video chain (MMCM, VTG, scaler, fb_reader, URAM VRAM,
//   HDMI TX, pin-out) WITHOUT needing a working CPU / ROM boot to first
//   paint the framebuffer.  On reset release this module walks through
//   every pixel in VRAM and drives hardcoded colour-bar data via the
//   vram.v AXI4 slave port.  When it finishes it parks in a `DONE` state
//   and is bit-identical silent for the rest of the session — the only
//   VRAM writer after that is the CPU / XDMA path.
//
// Gated by the VIDEO_SMOKE parameter on the top-level module.  When
// VIDEO_SMOKE=0 the instance is not built and its AXI master ports are
// never driven, so the pass=0 path is bit-identical to the pre-existing
// VRAM-tied-off design.
//
// Bar pattern (independent of BPP)
//   The source frame (W x H) is split into 8 vertical bars of width W/8.
//   Bar colours (left to right):
//     [WHITE, YELLOW, CYAN, GREEN, MAGENTA, RED, BLUE, BLACK]
//   These are SMPTE-style colour bars — a standard smoke signal that's
//   visible on any RGB display.  At BPP=8 the byte carries a low-nibble
//   DAFB CLUT index selected to match those colours in video_top's reset
//   CLUT.  At BPP=24 full RGB888.  At BPP=32 we write xRGB with the high
//   byte zero.
//
// ROW_BAND_LOG2 — vertical banding (default 0 = OFF, pattern unchanged)
//   Plain vertical bars make every source ROW byte-identical, so an image
//   built from them cannot distinguish "the scanner is walking rows" from
//   "the scanner is stuck re-displaying one row".  That distinction is the
//   entire point of the CPU-less scanout rig (linebuf_scanout.v's fetcher
//   parks at source row LINE_COUNT-1 = 63 when the display stops
//   consuming, and a frozen `vram_rd_addr` alone cannot tell you which
//   side wedged).  With ROW_BAND_LOG2 = N (N > 0) the bar phase advances
//   by one bar every 2**N source rows, so vertical progress is visible as
//   a staircase.  Setting N = 6 (64 rows) makes exactly one band per
//   line-buffer ring wrap, which is the signature to look for.
//   ROW_BAND_LOG2 = 0 leaves the pattern BIT-FOR-BIT as it was before this
//   parameter existed (tb_video_smoke.cpp's golden model relies on that).
//
// SYNTHESIS NOTE — FB_WIDTH_PX SHOULD BE A POWER OF TWO
//   pixel_data() below computes `pix % FB_WIDTH_PX`, `pix / FB_WIDTH_PX`
//   and `x_rel / (FB_WIDTH_PX/8)` on a RUNTIME word index, once per lane
//   (PX_PER_WORD = 16 lanes at BPP=8/DATA_WIDTH=128).  With a power-of-two
//   FB_WIDTH_PX every one of those folds to a bit-slice and costs nothing;
//   with, say, 640 they become 16 real constant-divisor dividers.  Keep
//   FB_WIDTH_PX a power of two and express a narrower visible window by
//   programming the DAFB stride to FB_WIDTH_PX and its hres to the
//   narrower value — the scanner then shows the left hres columns.
//
// Sequencing
//   State machine:
//     S_WAIT   — wait 8 cycles after reset for the vram slave to settle
//     S_WRITE  — issue AW+W+B for each AXI word, incrementing word_idx
//     S_DONE   — parked; all AXI outputs held low
//   One outstanding write at a time, AWLEN=0 single-beat bursts for
//   maximum simplicity (matches vram.v's FSM tolerances on any vendor).
//   Total latency at 200 MHz for 1024×768 ×24 bpp: N_WORDS≈147k * 3cy
//   ≈ 2.2 ms. Fires exactly ONCE per reset.
//
// Parameters
//   FB_WIDTH_PX   : source width in pixels
//   FB_HEIGHT_PX  : source height in pixels
//   BPP           : 8, 24, or 32 — matches vram.v's packing
//   DATA_WIDTH    : AXI data width (must match vram DATA_WIDTH, default 128)
//   ID_WIDTH      : AXI ID width (must match vram, default 4)
//
// Ports
//   clk / rst     — same domain as the vram.v AXI slave port
//   done          — latched high when all pixels have been written; feeds
//                   an observability LED on the FPGA top
//   m_aw* / m_w* / m_b*  — AXI4 master write-side (AR/R tied off)
//
// Verilog-2005, synchronous active-high reset.  No SystemVerilog.
// ---------------------------------------------------------------------------
`default_nettype none

module vram_smoke #(
    parameter FB_WIDTH_PX  = 1024,
    parameter FB_HEIGHT_PX = 768,
    parameter BPP          = 24,
    parameter DATA_WIDTH   = 128,
    parameter ID_WIDTH     = 4,
    // 0 = vertical bars only (legacy pattern, bit-for-bit).  N > 0 =
    // advance the bar phase by one bar every 2**N source rows.  See the
    // header.
    parameter ROW_BAND_LOG2 = 0
) (
    input  wire                       clk,
    input  wire                       rst,

    output reg                        done,

    // ── AXI4 master — write side only ─────────────────────────────────
    output reg  [ID_WIDTH-1:0]        m_awid,
    output reg  [31:0]                m_awaddr,
    output reg  [7:0]                 m_awlen,
    output reg  [2:0]                 m_awsize,
    output reg  [1:0]                 m_awburst,
    output reg                        m_awvalid,
    input  wire                       m_awready,

    output reg  [DATA_WIDTH-1:0]      m_wdata,
    output reg  [DATA_WIDTH/8-1:0]    m_wstrb,
    output reg                        m_wlast,
    output reg                        m_wvalid,
    input  wire                       m_wready,

    input  wire [ID_WIDTH-1:0]        m_bid,
    input  wire [1:0]                 m_bresp,
    input  wire                       m_bvalid,
    output reg                        m_bready
);

    // ── Derived sizing ─────────────────────────────────────────────────
    localparam integer N_PIXELS       = FB_WIDTH_PX * FB_HEIGHT_PX;
    localparam integer PX_PER_WORD    = DATA_WIDTH / BPP;      // e.g. 128/8=16
    localparam integer BYTES_PER_WORD = DATA_WIDTH / 8;        // 16 at 128
    // For BPP values where DATA_WIDTH is not an integer multiple of BPP
    // (e.g. 24bpp @128), PX_PER_WORD floors, and the final few bits of
    // each word are unused — matching vram.v's lane-mux convention.
    localparam integer N_WORDS        = (N_PIXELS + PX_PER_WORD - 1) / PX_PER_WORD;

    function integer clog2;
        input integer x;
        integer i;
        begin
            clog2 = 0;
            for (i = x - 1; i > 0; i = i >> 1) clog2 = clog2 + 1;
            if (clog2 == 0) clog2 = 1;
        end
    endfunction

    localparam integer WORD_IDX_W = clog2(N_WORDS);
    localparam integer X_IDX_W    = clog2(FB_WIDTH_PX);
    localparam integer BAR_W_PX   = FB_WIDTH_PX / 8;

    // ── Bar colour table (SMPTE-ish) ───────────────────────────────────
    // Indexed 0..7 (left to right).  Full RGB even for BPP=8/24/32 —
    // the per-BPP pack step truncates appropriately.
    function [23:0] bar_rgb;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: bar_rgb = 24'hFF_FF_FF;   // WHITE
                3'd1: bar_rgb = 24'hFF_FF_00;   // YELLOW
                3'd2: bar_rgb = 24'h00_FF_FF;   // CYAN
                3'd3: bar_rgb = 24'h00_FF_00;   // GREEN
                3'd4: bar_rgb = 24'hFF_00_FF;   // MAGENTA
                3'd5: bar_rgb = 24'hFF_00_00;   // RED
                3'd6: bar_rgb = 24'h00_00_FF;   // BLUE
                3'd7: bar_rgb = 24'h00_00_00;   // BLACK
                default: bar_rgb = 24'h80_80_80;
            endcase
        end
    endfunction

    function [3:0] bar_clut_index;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: bar_clut_index = 4'hF;   // WHITE
                3'd1: bar_clut_index = 4'hB;   // YELLOW
                3'd2: bar_clut_index = 4'hE;   // CYAN
                3'd3: bar_clut_index = 4'hA;   // GREEN
                3'd4: bar_clut_index = 4'hD;   // MAGENTA
                3'd5: bar_clut_index = 4'h9;   // RED
                3'd6: bar_clut_index = 4'hC;   // BLUE
                3'd7: bar_clut_index = 4'h0;   // BLACK
                default: bar_clut_index = 4'h8;
            endcase
        end
    endfunction

    // Per-pixel data at pixel index `pix` ∈ [0, N_PIXELS).
    // (pix = y*W + x) — bar index is x/BAR_W_PX, saturated at 7.
    function [BPP-1:0] pixel_data;
        input integer pix;
        integer x_rel;
        integer y_rel;
        integer bar_idx;
        reg [23:0] rgb;
        begin
            // Extract x coord (LSBs of linear index assuming linear Y-major layout).
            x_rel = pix % FB_WIDTH_PX;
            y_rel = pix / FB_WIDTH_PX;
            bar_idx = x_rel / BAR_W_PX;
            if (bar_idx > 7) bar_idx = 7;
            // Optional vertical banding.  Wraps mod 8 so the bar table is
            // always indexed in range; at ROW_BAND_LOG2 == 0 the term is
            // skipped entirely and the pattern is unchanged.
            if (ROW_BAND_LOG2 != 0)
                bar_idx = (bar_idx + (y_rel >> ROW_BAND_LOG2)) % 8;
            rgb = bar_rgb(bar_idx[2:0]);
            // Pack per BPP.  Note: scaler.v at BPP=8 selects the live
            // DAFB CLUT with fb_rd_data[3:0], so the smoke path must put
            // the colour identity in the low nibble.
            if (BPP == 8) begin
                pixel_data = {4'h0, bar_clut_index(bar_idx[2:0])};
            end else if (BPP == 24) begin
                // RGB888, low-byte = B, mid = G, high = R (matches scaler).
                pixel_data = rgb;
            end else if (BPP == 32) begin
                // xRGB8888, MSB byte zero.
                pixel_data = {8'h00, rgb};
            end else begin
                pixel_data = {BPP{1'b0}};
            end
        end
    endfunction

    // Build the DATA_WIDTH-wide word for word index `widx` (0 ..
    // N_WORDS-1).  Packs PX_PER_WORD pixels, lane 0 = lowest-address
    // pixel (little-endian-lane convention matching vram.v).
    //
    // For BPP=24 / DATA_WIDTH=128: PX_PER_WORD=5 (128/24 floors to 5),
    // so 5 * 24 = 120 bits used, top 8 bits padded zero.  This matches
    // vram.v's lane-mux which only reads lanes 0..PX_PER_WORD-1.
    //
    // Pure combinational; synthesis will flatten into a wide mux driven
    // by widx.  For a 49k-word FB this is ~49k*128 = 6 Mb of ROM after
    // folding — but it's NOT stored; the synthesiser computes the RHS
    // from `widx` via the pack function, yielding a compact LUT+arith
    // expression.
    function [DATA_WIDTH-1:0] word_data;
        input integer widx;
        integer lane;
        integer pix;
        reg [DATA_WIDTH-1:0] acc;
        begin
            acc = {DATA_WIDTH{1'b0}};
            for (lane = 0; lane < PX_PER_WORD; lane = lane + 1) begin
                pix = widx * PX_PER_WORD + lane;
                if (pix < N_PIXELS) begin
                    acc[lane * BPP +: BPP] = pixel_data(pix);
                end
            end
            word_data = acc;
        end
    endfunction

    // ── FSM ────────────────────────────────────────────────────────────
    // 3-bit state: we need a SETUP cycle that registers m_awvalid <= 1
    // WITHOUT simultaneously examining m_awready in the same cycle.  If
    // we collapse "assert awvalid" and "detect awready" into one state,
    // the non-blocking `m_awvalid <= 0` on handshake fires BEFORE
    // m_awvalid has actually reached 1 in the DUT, so the slave never
    // sees the asserted request and we deadlock.  S_AW_H is the hold/
    // wait-for-awready state that reads awready AFTER awvalid is already
    // registered high.
    localparam [2:0] S_WAIT  = 3'd0,
                     S_AW    = 3'd1,   // assert awvalid
                     S_AW_H  = 3'd2,   // hold awvalid, wait for awready
                     S_W     = 3'd3,
                     S_B     = 3'd4;

    reg [2:0]               state;
    reg [WORD_IDX_W-1:0]    word_idx;
    reg [3:0]               wait_ctr;

    // Byte address = word_idx * BYTES_PER_WORD.
    wire [31:0] waddr_bytes = word_idx * BYTES_PER_WORD;

    always @(posedge clk) begin
        if (rst) begin
            state      <= S_WAIT;
            wait_ctr   <= 4'd0;
            word_idx   <= {WORD_IDX_W{1'b0}};
            done       <= 1'b0;

            m_awid     <= {ID_WIDTH{1'b0}};
            m_awaddr   <= 32'd0;
            m_awlen    <= 8'd0;
            m_awsize   <= 3'b100;         // 16 bytes per beat
            m_awburst  <= 2'b01;          // INCR
            m_awvalid  <= 1'b0;
            m_wdata    <= {DATA_WIDTH{1'b0}};
            m_wstrb    <= {(DATA_WIDTH/8){1'b1}};
            m_wlast    <= 1'b0;
            m_wvalid   <= 1'b0;
            m_bready   <= 1'b1;
        end else begin
            case (state)
                S_WAIT: begin
                    // Hold AXI idle a few cycles after reset — lets vram.v
                    // come out of its own reset and assert awready.
                    wait_ctr <= wait_ctr + 4'd1;
                    if (wait_ctr == 4'd7)
                        state <= S_AW;
`ifdef VRAM_SMOKE_DEBUG
                    $display("[smoke] S_WAIT ctr=%0d", wait_ctr);
`endif
                end

                S_AW: begin
                    // Register the AW payload and assert awvalid.  DO NOT
                    // examine awready in this same cycle — awvalid is
                    // still pending a non-blocking update.
                    m_awid    <= {ID_WIDTH{1'b0}};
                    m_awaddr  <= waddr_bytes;
                    m_awlen   <= 8'd0;         // single beat
                    m_awsize  <= 3'b100;       // 16 bytes
                    m_awburst <= 2'b01;        // INCR
                    m_awvalid <= 1'b1;
                    state     <= S_AW_H;
`ifdef VRAM_SMOKE_DEBUG
                    $display("[smoke] S_AW  setup widx=%0d addr=0x%08x",
                             word_idx, waddr_bytes);
`endif
                end

                S_AW_H: begin
                    // awvalid is high now (registered last cycle).  Wait
                    // for awready, then issue W beat.
`ifdef VRAM_SMOKE_DEBUG
                    $display("[smoke] S_AW_H widx=%0d data=0x%032h",
                             word_idx, word_data(word_idx));
`endif
                    if (m_awready) begin
                        m_awvalid <= 1'b0;
                        m_wdata   <= word_data(word_idx);
                        m_wstrb   <= {(DATA_WIDTH/8){1'b1}};
                        m_wlast   <= 1'b1;
                        m_wvalid  <= 1'b1;
                        state     <= S_W;
                    end
                end

                S_W: begin
`ifdef VRAM_SMOKE_DEBUG
                    $display("[smoke] S_W widx=%0d wvalid=%0d wready=%0d",
                             word_idx, m_wvalid, m_wready);
`endif
                    if (m_wready) begin
                        m_wvalid <= 1'b0;
                        m_wlast  <= 1'b0;
                        m_bready <= 1'b1;
                        state    <= S_B;
                    end
                end

                S_B: begin
`ifdef VRAM_SMOKE_DEBUG
                    $display("[smoke] S_B widx=%0d bvalid=%0d bready=%0d",
                             word_idx, m_bvalid, m_bready);
`endif
                    if (m_bvalid) begin
                        // Completed one word; advance.
`ifdef VRAM_SMOKE_DEBUG
                        $display("[smoke] B ack @widx=%0d", word_idx);
`endif
                        if (word_idx == (N_WORDS - 1)) begin
                            done  <= 1'b1;
                            state <= S_WAIT;   // parked; wait_ctr won't wrap back
                            // Note: we don't go back to S_AW because `done`
                            // gates the top-level comparison below — see the
                            // if-gate at S_WAIT entry.
                            wait_ctr <= 4'd0;
                        end else begin
                            word_idx <= word_idx + 1'b1;
                            state    <= S_AW;
                        end
                    end
                end

                default: state <= S_WAIT;
            endcase

            // Once done, stay parked (drop out of S_WAIT even if the
            // 8-cycle counter rolls over).
            if (done) begin
                state     <= S_WAIT;
                wait_ctr  <= 4'd0;
                m_awvalid <= 1'b0;
                m_wvalid  <= 1'b0;
            end
        end
    end

    // silence lint on unused AXI reply fields
    wire _unused = &{1'b0, m_bid, m_bresp};

endmodule
`default_nettype wire
