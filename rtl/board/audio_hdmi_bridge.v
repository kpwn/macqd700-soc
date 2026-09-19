// audio_hdmi_bridge.v — ASC audio sample → HDMI data-island formatter.
//
// Task #123 (b), HDMI-audio variant.  HDMI "Audio Sample Packets" carry
// IEC 60958 (S/PDIF) sub-frames inside the data-island period between
// video pixels — the HDMI controller already exists and has a spare
// data-island input, this module formats the ASC's 16-bit stereo stream
// into the IEC sub-frame shape the HDMI controller expects and exposes
// a simple ready/valid handshake.
//
// The real HDMI 1.4 Audio Sample Packet carries up to 4 sub-frames of
// 28 bits each in IEC 60958 channel-status format:
//
//   bit 27..8   : 20-bit sample (MSB-first inside these 20 bits; we pad
//                 our 16-bit sample with 4 zero LSBs)
//   bit 7..4    : P / C / U / V parity/channel-status/user/validity bits
//   bit 3..0    : preamble (B/M/W — frame type)
//
// Simplification for this bring-up:
//   * We emit a single sub-frame per channel per sample.
//   * Preamble uses the "start of block" code (Z = 4'b0111) on the first
//     LEFT of every 192-sample block, otherwise M (left = 4'b0011) and
//     W (right = 4'b0010).  Matches the IEC convention for consumer.
//   * Parity (P) is the even-parity XOR of bits [27:4]; C/U bits are
//     zero (default consumer channel-status block); V=1 (valid).
//   * The 192-frame block counter rolls over every 192 LEFT frames and
//     is used only for the Z/M/W preamble selection — no channel-status
//     packing is done yet.  Downstream HDMI controller is free to
//     overwrite the C bits with its own channel-status block generator.
//
// Output handshake:
//   * frame_valid pulses for one clk when a fresh {L,R} sub-frame pair
//     is presented on {subframe_l, subframe_r}.  The consumer latches
//     both on the same cycle.  ready back-pressure is supported: if
//     ready=0 when frame_valid would go high we stall and emit on the
//     next cycle when ready=1.  In practice the HDMI data-island
//     scheduler pulls one sub-frame at a time as blanking opens; the
//     upstream sample rate (~22 kHz) is far below the HDMI audio
//     capacity (≥32 kHz), so back-pressure is unusual.
//
// This is a SIM / skeleton implementation.  Real HDMI audio also wants:
//   - Audio InfoFrame transmission (sampling rate, channel count, format)
//   - Audio Clock Regeneration packets (CTS / N values)
//   - ACP / ISRC packets
// Those live inside the HDMI controller proper and are out of scope for
// this bridge.
//
// Verilog-2005, synchronous reset, module ≤ 300 lines.

`default_nettype none

module audio_hdmi_bridge (
    input  wire        clk,
    input  wire        rst,

    // Upstream sample from ASC (big-endian {L, R} 8-bit pair).
    input  wire [15:0] sample_in,
    input  wire        sample_valid,

    // Downstream HDMI data-island interface — one sub-frame pair per
    // handshake.
    output reg  [27:0] subframe_l,
    output reg  [27:0] subframe_r,
    output reg         frame_valid,
    input  wire        frame_ready
);

    // ── Block counter (192 frames per IEC channel-status block) ──────
    // Block length is 192 AES/EBU frames.  The Z preamble flags the
    // first frame of each block.
    reg [7:0] block_cnt;   // 0 .. 191

    // Preamble codes (B/M/W per IEC-60958 Table 6):
    localparam [3:0] PREAMBLE_Z = 4'b0111;  // first left of block
    localparam [3:0] PREAMBLE_M = 4'b0011;  // normal left
    localparam [3:0] PREAMBLE_W = 4'b0010;  // right

    // ── Sign-extend + left-align inside the 20-bit sample field ──────
    // 16-bit → 20-bit left-aligned: pad 4 zero bits below the LSB.
    wire signed [7:0] samp_l = sample_in[15:8];
    wire signed [7:0] samp_r = sample_in[ 7:0];
    wire        [19:0] samp_l20 = {{(20-8){samp_l[7]}}, samp_l[7:0]} << 4
                                   | 20'h0;
    wire        [19:0] samp_r20 = {{(20-8){samp_r[7]}}, samp_r[7:0]} << 4
                                   | 20'h0;
    // NB: the <<4 sign-extends then shifts so the MSB lives at bit 19.
    // The trailing '| 20'h0' is a noop kept for readability of the
    // two-step "sign-extend then left-align" chain.

    // ── Compose a sub-frame (preamble + V/U/C/P + 20-bit sample) ─────
    // Format: { sample20[19:0], V, U, C, P, preamble[3:0] }
    //         27                 7 6 5 4  3            0
    // V=1 (valid), U=0, C=0 (consumer, copy-permit, etc. will be owned
    // by a downstream channel-status packer if we add one).  P is even
    // parity over bits [27:4] — compute below.
    /* verilator lint_off BLKSEQ */
    // Blocking assigns are idiomatic inside Verilog-2005 functions —
    // `body` and `parity` are function-local temporaries, not flops.
    function [27:0] pack_subframe;
        input [19:0] sample20;
        input [3:0]  preamble;
        reg  [23:0] body;   // bits [27:4] pre-parity
        reg         parity;
        begin
            body   = {sample20, 3'b100};   // V=1, U=0, C=0
            // Even parity: P is chosen so XOR(body) ^ P = 0.
            parity = ^body;
            pack_subframe = {body, parity, preamble};
        end
    endfunction
    /* verilator lint_on BLKSEQ */

    // ── FSM ──────────────────────────────────────────────────────────
    // On sample_valid we latch the {L,R} sub-frames into the output regs
    // and raise frame_valid.  If frame_ready was low we stall; otherwise
    // frame_valid drops after one cycle.
    reg pending;   // a latched frame is waiting for the consumer

    always @(posedge clk) begin
        if (rst) begin
            subframe_l   <= 28'h0;
            subframe_r   <= 28'h0;
            frame_valid  <= 1'b0;
            pending      <= 1'b0;
            block_cnt    <= 8'd0;
        end else begin
            // Consumer handshake: when frame_ready & frame_valid both
            // high, the sub-frame was consumed this cycle — drop valid.
            if (frame_valid && frame_ready) begin
                frame_valid <= 1'b0;
                pending     <= 1'b0;
            end

            if (sample_valid) begin
                // Determine preamble for this sample's LEFT sub-frame.
                if (block_cnt == 8'd0)
                    subframe_l <= pack_subframe(samp_l20, PREAMBLE_Z);
                else
                    subframe_l <= pack_subframe(samp_l20, PREAMBLE_M);
                subframe_r <= pack_subframe(samp_r20, PREAMBLE_W);
                frame_valid <= 1'b1;
                pending     <= 1'b1;

                // Advance the 192-frame block counter on every LEFT.
                if (block_cnt == 8'd191) block_cnt <= 8'd0;
                else                     block_cnt <= block_cnt + 8'd1;
            end

            // Keep valid asserted while the consumer stalls.  If both
            // `sample_valid` fires and the previous frame is still
            // pending, we overwrite the held sub-frame — sample rate
            // drop, matching I2S's under-run policy.
            if (pending && !frame_ready && !sample_valid) begin
                frame_valid <= 1'b1;
            end
        end
    end

    // ── Lint silencing ───────────────────────────────────────────────
    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused_ok = &{1'b0, samp_l20[3:0], samp_r20[3:0], 1'b0};
    /* verilator lint_on UNUSEDSIGNAL */

endmodule

`default_nettype wire
