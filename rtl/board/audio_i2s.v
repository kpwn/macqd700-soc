// audio_i2s.v — ASC 16-bit sample stream → I2S serial output.
//
// Task #123 (b): external-codec audio path.  Bridges the ASC's core-clock
// `audio_sample_out[15:0]` + `audio_sample_valid` handshake to an I2S
// output pair intended for a future external audio codec chip (e.g.
// PCM5102, CS4344, ADAU1977 class DAC).
//
// Frame shape (left-justified; also acceptable to most "I2S-compatible"
// codecs that tolerate the 0-BCLK delay variant):
//
//                ┌─ 32 BCLKs ─┐┌─ 32 BCLKs ─┐
//         LRCLK: ──┐             ┌─────────────┐
//                  │     LEFT    │    RIGHT    │
//                  └─────────────┘             └─── ...
//         DATA:  MSB ─────── LSB zeros  MSB ─────── LSB zeros
//         BCLK:  ╔╗╔╗╔╗╔╗╔╗╔╗╔╗╔╗╔╗╔╗╔╗╔╗╔╗╔╗╔╗╔╗...
//
// Conventions:
//   * DATA bits change on the BCLK falling edge and the codec samples
//     them on the BCLK rising edge.
//   * LRCLK=0 selects the LEFT slot, LRCLK=1 the RIGHT.
//   * Each slot is `SLOT_BITS` wide; the BITS_PER_SAMPLE MSBs hold the
//     sample (MSB-first, sign-extended) and the remaining bits are zero.
//
// ASC sample-rate contract:
//   audio_sample_valid pulses for one core_clk cycle per sample (~22 kHz
//   by default on the Mac).  The serialiser keeps the most recent {L,R}
//   pair in a holding register and re-emits it every LRCLK frame until
//   fresh data arrives — avoids clicks on sample-rate/I2S-frame-rate
//   mismatch.
//
// Timing parameters:
//   CORE_FREQ_HZ        core_clk frequency (passed from fpga_top).
//   I2S_BCLK_FREQ_HZ    target BCLK frequency.  Default 3.072 MHz =
//                       48 000 × 64 (64-bit frames × 48 kHz).
//   BITS_PER_SAMPLE     payload bits per channel (16, 24, or 32).
//   SLOT_BITS           total bits per channel slot (32).
//
// For 100 MHz core clock and 3.072 MHz BCLK, HALF_PERIOD ≈ 16 core cycles
// → actual BCLK ≈ 3.125 MHz, well within ±5 % codec tolerance.  For the
// unit tb we scale CORE_FREQ_HZ down so a full frame fits in a few
// hundred cycles.
//
// Verilog-2005, synchronous reset, module ≤ 300 lines.
// Latency: 1 core_clk cycle from `sample_valid` to the holding register;
// up to one full frame (SLOT_BITS × 2 BCLKs) until the new sample
// appears on DATA.

`default_nettype none

module audio_i2s #(
    parameter integer CORE_FREQ_HZ     = 100_000_000,
    parameter integer I2S_BCLK_FREQ_HZ =   3_072_000,
    parameter integer BITS_PER_SAMPLE  = 16,
    parameter integer SLOT_BITS        = 32
) (
    input  wire        clk,          // core_clk (CORE_FREQ_HZ)
    input  wire        rst,

    // Sample input.  Upper byte = LEFT, lower byte = RIGHT (matches
    // asc.audio_sample_out big-endian convention {a_scaled, b_scaled}).
    input  wire [15:0] sample_in,
    input  wire        sample_valid,

    // I2S output.  Drive directly to FPGA pins or through ODDR for skew
    // control on long traces.
    output wire        i2s_bclk,
    output wire        i2s_lrclk,
    output wire        i2s_data
);

    // ── BCLK divider ─────────────────────────────────────────────────
    // HALF_PERIOD = core cycles per BCLK half-period.  Round up so BCLK
    // is at most the nominal target (never exceeds the codec's max).
    localparam integer HALF_PERIOD_RAW =
        (CORE_FREQ_HZ + (2 * I2S_BCLK_FREQ_HZ) - 1) / (2 * I2S_BCLK_FREQ_HZ);
    localparam integer HALF_PERIOD =
        (HALF_PERIOD_RAW < 1) ? 1 : HALF_PERIOD_RAW;
    localparam integer DIV_W = 10;   // enough for < 1024 core cycles/half

    reg [DIV_W-1:0] div_cnt;
    reg             bclk_q;

    wire div_tick = (div_cnt == {DIV_W{1'b0}});
    // Reload value: HALF_PERIOD - 1.
    wire [DIV_W-1:0] div_reload = HALF_PERIOD[DIV_W-1:0] - {{(DIV_W-1){1'b0}}, 1'b1};

    always @(posedge clk) begin
        if (rst) begin
            div_cnt <= div_reload;
            bclk_q  <= 1'b0;
        end else begin
            if (div_tick) begin
                div_cnt <= div_reload;
                bclk_q  <= ~bclk_q;
            end else begin
                div_cnt <= div_cnt - {{(DIV_W-1){1'b0}}, 1'b1};
            end
        end
    end

    // Drive BCLK combinationally from the toggle flop.
    assign i2s_bclk = bclk_q;

    // ── Falling-edge detector ────────────────────────────────────────
    // "About to fall" = div_tick && bclk_q (bclk_q is 1 now, will flip to
    // 0 on this edge).  We use it to advance the slot counter and shift
    // one data bit out.
    wire bclk_fall = div_tick && bclk_q;

    // ── Holding registers ────────────────────────────────────────────
    // Latched on any sample_valid pulse.  Consumed at frame boundary.
    reg [7:0] hold_left;
    reg [7:0] hold_right;

    always @(posedge clk) begin
        if (rst) begin
            hold_left  <= 8'h00;
            hold_right <= 8'h00;
        end else if (sample_valid) begin
            hold_left  <= sample_in[15:8];
            hold_right <= sample_in[ 7:0];
        end
    end

    // Sign-extended per-channel samples in the current holding state.
    wire signed [7:0] samp_l = hold_left;
    wire signed [7:0] samp_r = hold_right;
    wire [BITS_PER_SAMPLE-1:0] samp_l_ext =
        {{(BITS_PER_SAMPLE-8){samp_l[7]}}, samp_l};
    wire [BITS_PER_SAMPLE-1:0] samp_r_ext =
        {{(BITS_PER_SAMPLE-8){samp_r[7]}}, samp_r};

    // Pack a sample into a SLOT_BITS slot, MSB-left, tail zero.
    function [SLOT_BITS-1:0] pack_slot;
        input [BITS_PER_SAMPLE-1:0] v;
        begin
            pack_slot = { v, {(SLOT_BITS-BITS_PER_SAMPLE){1'b0}} };
        end
    endfunction

    // ── Slot + bit counters ──────────────────────────────────────────
    // bit_cnt advances once per BCLK falling edge.  SLOT_BITS per slot,
    // then LRCLK flips and a new slot loads.
    localparam integer BIT_CNT_W = 6;
    reg [BIT_CNT_W-1:0] bit_cnt;
    reg                 lrclk_q;
    reg [SLOT_BITS-1:0] shift_reg;
    reg                 data_q;

    // Current and next slot content muxed from lrclk_q.
    wire [SLOT_BITS-1:0] slot_left  = pack_slot(samp_l_ext);
    wire [SLOT_BITS-1:0] slot_right = pack_slot(samp_r_ext);

    // `next_lrclk` is the LRCLK level for the slot we're ABOUT to load at
    // bit_cnt=0.  It flips once per slot.
    always @(posedge clk) begin
        if (rst) begin
            bit_cnt   <= {BIT_CNT_W{1'b0}};
            lrclk_q   <= 1'b1;          // first load below flips to 0
                                        // (LEFT) on the first bclk_fall
            shift_reg <= {SLOT_BITS{1'b0}};
            data_q    <= 1'b0;
        end else if (bclk_fall) begin
            if (bit_cnt == {BIT_CNT_W{1'b0}}) begin
                // Starting a new slot.  Flip LRCLK, load the holding
                // reg's sample for the new slot, present the MSB on
                // DATA.  LRCLK and the first bit of the slot become
                // visible on the SAME BCLK rising edge, which is the
                // left-justified I2S convention (some codecs call this
                // "LJ mode" rather than strict Philips I2S — for a
                // Philips-strict receiver, re-time the LRCLK edge one
                // BCLK earlier via the TX FIFO on the board).
                if (~lrclk_q == 1'b0) begin
                    shift_reg <= {slot_left [SLOT_BITS-2:0], 1'b0};
                    data_q    <= slot_left [SLOT_BITS-1];
                end else begin
                    shift_reg <= {slot_right[SLOT_BITS-2:0], 1'b0};
                    data_q    <= slot_right[SLOT_BITS-1];
                end
                lrclk_q <= ~lrclk_q;
                bit_cnt <= {{(BIT_CNT_W-1){1'b0}}, 1'b1};
            end else begin
                data_q    <= shift_reg[SLOT_BITS-1];
                shift_reg <= {shift_reg[SLOT_BITS-2:0], 1'b0};
                if (bit_cnt == SLOT_BITS[BIT_CNT_W-1:0] - {{(BIT_CNT_W-1){1'b0}}, 1'b1}) begin
                    bit_cnt <= {BIT_CNT_W{1'b0}};
                end else begin
                    bit_cnt <= bit_cnt + {{(BIT_CNT_W-1){1'b0}}, 1'b1};
                end
            end
        end
    end

    assign i2s_lrclk = lrclk_q;
    assign i2s_data  = data_q;

    // ── Lint silencing ───────────────────────────────────────────────
    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused_ok = &{1'b0, div_reload, 1'b0};
    /* verilator lint_on UNUSEDSIGNAL */

endmodule

`default_nettype wire
