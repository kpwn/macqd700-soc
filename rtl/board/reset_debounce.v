// reset_debounce.v — synchronise + debounce a mechanical push-button
//
// Purpose
//   The board cpu_resetn pin (T19) and btn[3] are raw mechanical
//   inputs.  A bouncy press creates multiple low/high transitions
//   over a few ms — without filtering, every transition reaches the
//   5+ downstream async-clear domains independently, desynchronising
//   them and exposing FFs that only pulsed reset for one input edge.
//
//   This module does two jobs:
//     1. 3-FF synchroniser into the sample clock domain.
//     2. Debounce: output flips state only after the synchronised
//        input has been STABLE for DEBOUNCE_CYCLES consecutive samples.
//
//   The behaviour is symmetric on both edges so an inadvertent
//   bouncy release also produces exactly one clean transition.
//
// Active-high convention
//   Input:  raw_in_n is active-low (cpu_resetn / btn-not-pressed).
//   Output: out_n is active-low, debounced + synced.
//   On reset_pin_low / button-press: both raw_in_n and out_n drop to 0.
//   On release: out_n returns to 1 only after raw_in_n has stayed
//   high for DEBOUNCE_CYCLES samples.
//
// Latency
//   3 sync FFs + DEBOUNCE_CYCLES debounce window.  Output is registered.
//
// Active-low semantic kept so callers can `&&` multiple sources
// directly into platform_resetn = a && b && c.
//
// Power-up idle polarity (IDLE_OUT_N) — real-HW boot-NMI bug, 2026-07-17
//   `out_n`'s power-up value used to be hardwired to 0 ("hold reset
//   asserted at configuration time") regardless of what the caller
//   actually wanted that to mean.  That is the SAFE default for the
//   original cpu_resetn/btn[3] system-reset use (0 = reset held is
//   the fail-safe state before the debounce window has had a chance
//   to observe the true pin level), but it is the WRONG default for
//   any *non-reset* momentary button wired through this same module
//   (NMI btn[1], debug-full-reset btn[2]): for those, `out_n==0`
//   means "button is being read as PRESSED", so between FPGA
//   configuration and the first completed debounce window (~10 ms
//   for NMI_DEBOUNCE_CYCLES) the consumer sees a false, guaranteed,
//   100%-reproducible "pressed" level even though the physical button
//   was never touched.  For btn[1] this produced a genuine 0->1 rising
//   edge on `nmi_btn_core` the moment `soc_full_rst_bank[3]` released
//   (its sync regs come up in reset at 0, then sample the still-
//   falsely-"pressed"=1 `nmi_btn_pulse`), which irq_agg.v's edge
//   detector latches as a real NMI — the CPU takes vec 31 mid-boot
//   (observed: inside a ROM delay loop at 0x40847abe, SR interrupt
//   mask=3 so only level-7 could preempt it) and free-run execution
//   drifts into unrelated ROM code (a dead SCC-poll loop the CPU
//   never returns from) with a stack pointer that was never given a
//   real return-address chain.  100% reproducible on every bitstream
//   (re)configuration/power-up — not a rare board glitch, not a
//   mechanical-bounce artifact, and NOT cleared by a plain JTAG
//   logic-only "reset" (this module has no `rst` input — its `initial`
//   value is a configuration-time-only INIT, so once the first real
//   debounce window completes post-configuration it never re-arms).
//
//   Fix: make the idle polarity a parameter.  Default (0) preserves
//   today's behaviour for the reset-class instantiations
//   (cpu_resetn/btn[3] in fpga_top_clocks.vh, cpu_resetn in
//   sd_provision_top.v) where "hold reset" is genuinely the safe
//   power-up assumption.  Momentary-button instantiations (NMI
//   btn[1], debug-full-reset btn[2]) must override IDLE_OUT_N=1 so
//   `out_n` — and the sync chain that feeds its "stable" comparison —
//   powers up already agreeing with the true idle ("not pressed")
//   pin level, so no spurious edge is ever presented downstream.

`default_nettype none

module reset_debounce #(
    // 1024 cycles at 200 MHz = ~5 us — well above mechanical bounce
    // (typical 1-10 ms) is overkill, but the goal is to filter
    // electrical noise, not the full bounce envelope.  Override on
    // instantiation if a longer window is wanted.
    parameter integer DEBOUNCE_CYCLES = 1024,
    // Power-up/configuration-time value of `out_n` (and the sync
    // chain feeding it).  0 = "reset/button held" (safe default for
    // reset-class users).  Momentary non-reset buttons (NMI,
    // debug-full-reset) must instantiate with IDLE_OUT_N=1 — see
    // header comment above.
    parameter           IDLE_OUT_N     = 1'b0
) (
    input  wire clk,
    input  wire raw_in_n,    // raw async pin (active-low)
    output reg  out_n        // debounced, registered (active-low)
);
    function integer clog2_local;
        input integer v;
        integer       i;
        begin
            clog2_local = 0;
            for (i = v - 1; i > 0; i = i >> 1) clog2_local = clog2_local + 1;
        end
    endfunction
    // +1 so the counter has headroom to compare equal to
    // DEBOUNCE_CYCLES without rolling over.
    localparam integer CNT_W = clog2_local(DEBOUNCE_CYCLES + 1);

    // 3-FF synchroniser; mark the first two ASYNC_REG.
    (* ASYNC_REG = "TRUE" *) reg sync_meta;
    (* ASYNC_REG = "TRUE" *) reg sync_q;
    reg                       sync_qq;
    initial begin
        // [0] slice: IDLE_OUT_N is an unsized (32-bit) parameter, like
        // DEBOUNCE_CYCLES above, so -G overrides can pass a plain
        // integer literal without a sized-literal shell-quoting dance;
        // only bit 0 is meaningful and everything here is 1-bit.
        sync_meta = IDLE_OUT_N[0];
        sync_q    = IDLE_OUT_N[0];
        sync_qq   = IDLE_OUT_N[0];
        out_n     = IDLE_OUT_N[0]; // hold at the caller's safe idle state
                                   // at configuration time (see header).
    end
    always @(posedge clk) begin
        sync_meta <= raw_in_n;
        sync_q    <= sync_meta;
        sync_qq   <= sync_q;
    end

    // Debounce counter: count consecutive samples where sync_qq
    // differs from out_n.  When the count hits DEBOUNCE_CYCLES, flip
    // out_n.  Any sample that matches out_n resets the counter to 0
    // (so we count CONSECUTIVE differing samples, not aggregate).
    reg [CNT_W-1:0] cnt;
    initial cnt = {CNT_W{1'b0}};
    always @(posedge clk) begin
        if (sync_qq == out_n) begin
            cnt <= {CNT_W{1'b0}};
        end else begin
            if (cnt == DEBOUNCE_CYCLES[CNT_W-1:0]) begin
                out_n <= sync_qq;
                cnt   <= {CNT_W{1'b0}};
            end else begin
                cnt <= cnt + 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
