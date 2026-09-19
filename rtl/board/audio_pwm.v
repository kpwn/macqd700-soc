// audio_pwm.v — ASC signed-16 PCM → first-order Σ-Δ PWM, 1 pin/channel.
//
// Third sibling alongside audio_i2s.v and audio_hdmi_bridge.v under the
// AUDIO_PATH selector in rtl/fpga_top_peripherals.vh.  Targets the ALINX
// AN9134 carrier where the daughtercard occupies all 32 useful HDMI pins
// of the 40-pin connector and leaves only J1.35 / J1.36 (NC) free —
// not enough room for a 3-wire I2S codec, but enough for a per-channel
// 1-bit Σ-Δ PWM output that drives an off-board reconstruction filter
// or a class-D switch (single IRLZ54N + Schottky + AC-coupling cap →
// 8 Ω 2 W speaker).  See docs/superpowers/specs/2026-05-04-an9134-pwm-
// audio-design.md for the analog-side board.
//
// Algorithm: first-order error-feedback Σ-Δ modulator, one per channel.
// Sample-and-hold latches the latest signed PCM on every audio_sample_valid
// strobe; the modulator integrates the held value at every clk tick.  The
// stair-step held input + high oversampling ratio (PB_CLK_HZ / 22.254 kHz
// ≈ 2247 at 50 MHz) + the analog LPF outside the FPGA together perform
// reconstruction; no interpolation FIR is needed.
//
// Per channel:
//
//     x_u   = sample[15] ? {1'b0, sample[14:0]}          // negative half
//                        : {1'b1, sample[14:0]};         // positive half
//                        // ≡ signed→offset binary (invert MSB),
//                        // silence (0) → 0x8000 → 50% duty.
//     {pwm, err_next} = {1'b0, err} + {1'b0, x_u};       // 17-bit add
//                                                        // pwm = carry-out
//                                                        // err_next = mod 2^16
//
// The modulus-2^16 wrap of the unsigned accumulator IS the error
// feedback: anything that overflowed left in `err_next` to be added
// back next cycle, while the carry-out is the 1-bit modulator output.
//
// In-band SNR (first-order Σ-Δ):
//     SNR = 6.02·N_modulator + 1.76 - 5.17 + 9·log2(OSR)
//     N_modulator = 1, OSR ≈ 2247  →  SNR ≈ 103 dB  →  ENOB ≈ 17 bits
//   in the 0–20 kHz band — far past what the ASC sources at ≤8 effective
//   bits.  Quality limit is the analog LPF and the speaker, not the
//   modulator.
//
// Latency: 1 clk cycle from audio_sample_valid → updated hold reg.
//          1 clk cycle from hold reg → modulator output.
// Total : 2 clk cycles from sample_valid to first modulator step
//          reflecting the new sample.
//
// Verilog-2005, synchronous reset, ≤ 300 lines.

`default_nettype none

module audio_pwm #(
    parameter integer CORE_FREQ_HZ = 50_000_000   // informational; module
                                                  // is rate-agnostic.
) (
    input  wire        clk,
    input  wire        rst,

    // Signed 16-bit PCM, two's complement.  Silence = 16'h0000.
    input  wire [15:0] sample_l,
    input  wire [15:0] sample_r,

    // One-cycle pulse per ASC sample (pb_clk-aligned).
    input  wire        sample_valid,

    // 1-bit Σ-Δ output per channel.  Drive directly to FPGA pin →
    // analog reconstruction off-board.
    output wire        pwm_l,
    output wire        pwm_r
);

    // ── Sample-and-hold ──────────────────────────────────────────────
    reg [15:0] hold_l;
    reg [15:0] hold_r;

    always @(posedge clk) begin
        if (rst) begin
            hold_l <= 16'h0000;
            hold_r <= 16'h0000;
        end else if (sample_valid) begin
            hold_l <= sample_l;
            hold_r <= sample_r;
        end
    end

    // Signed → offset-binary (invert MSB).  Silence (0) → 0x8000.
    wire [15:0] x_u_l = {~hold_l[15], hold_l[14:0]};
    wire [15:0] x_u_r = {~hold_r[15], hold_r[14:0]};

    // ── First-order error-feedback Σ-Δ ───────────────────────────────
    // The 16-bit unsigned accumulator wraps mod 2^16; the carry-out is
    // the 1-bit modulator output.  No subtraction needed — wrapping IS
    // the error feedback.
    reg [15:0] err_l;
    reg [15:0] err_r;
    reg        pwm_l_q;
    reg        pwm_r_q;

    wire [16:0] sum_l = {1'b0, err_l} + {1'b0, x_u_l};
    wire [16:0] sum_r = {1'b0, err_r} + {1'b0, x_u_r};

    always @(posedge clk) begin
        if (rst) begin
            err_l   <= 16'h0000;
            err_r   <= 16'h0000;
            pwm_l_q <= 1'b0;
            pwm_r_q <= 1'b0;
        end else begin
            err_l   <= sum_l[15:0];
            err_r   <= sum_r[15:0];
            pwm_l_q <= sum_l[16];
            pwm_r_q <= sum_r[16];
        end
    end

    assign pwm_l = pwm_l_q;
    assign pwm_r = pwm_r_q;

    // CORE_FREQ_HZ is informational; tap it so the parameter isn't pruned.
    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused_freq = |{CORE_FREQ_HZ[31:0], 1'b0};
    /* verilator lint_on UNUSEDSIGNAL */

endmodule

`default_nettype wire
