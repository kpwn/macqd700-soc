// async_fifo.v — dual-clock asynchronous FIFO with Gray-coded pointers.
//
// Purpose
// ───────
// Reusable clock-domain-crossing (CDC) FIFO.  One writer on `wclk`, one
// reader on `rclk`.  Pointers cross domains via Gray code + 2-flop
// synchronisers — the classic Cummings pattern (SNUG 2002).  Data storage
// is inferable BRAM for depth > 32, LUTRAM for smaller depths.
//
// Every callers-facing handshake is the valid / ready / full / empty
// pattern — no combinational paths cross the domain.  Data written on
// `wclk` may be popped on `rclk` with the usual 2-3 rclk-cycle latency
// for the full/empty pointer to propagate.
//
// Parameters
// ──────────
//   WIDTH         : payload width (default 32).
//   DEPTH_LOG2    : log2 of depth — depth = 1 << DEPTH_LOG2.  Power of 2
//                   ONLY.  Default 4 (16 entries).
//   ALMOST_FULL_THRESHOLD  : when occupancy ≥ this, wr_almost_full = 1.
//                   Default = depth - 2.
//   ALMOST_EMPTY_THRESHOLD : when occupancy ≤ this, rd_almost_empty = 1.
//                   Default = 2.  Note: the "occupancy" seen on each side
//                   is the Gray-synchronised pointer difference, which is
//                   conservative for both sides (writer over-estimates
//                   fullness, reader under-estimates it).
//
// Ports — writer domain (`wclk`)
// ──────────────────────────────
//   wrst          : synchronous active-high reset on `wclk`.  Resets the
//                   write pointer.  Cross-coupled internally via a
//                   request/ack handshake (see "Reset semantics" below)
//                   — safe to assert independently of `rrst`, and safe
//                   for a single wclk-cycle pulse (no minimum-width
//                   requirement on the caller's side; the handshake
//                   itself stretches and confirms delivery).
//   wr_en         : push enable.  Ignored when wr_full is high.
//   wr_data       : payload.
//   wr_full       : FIFO cannot accept another push this cycle.
//   wr_almost_full: occupancy-high watermark crossed.  Advisory.
//
// Ports — reader domain (`rclk`)
// ──────────────────────────────
//   rrst          : synchronous active-high reset on `rclk`.  Resets the
//                   read pointer.  Cross-coupled internally via the same
//                   handshake — safe to assert independently of `wrst`
//                   (e.g. a downstream MIG recalibration or PCIe link
//                   retrain that only pulses one side), and safe for a
//                   single rclk-cycle pulse.
//   rd_en         : pop enable.  Ignored when rd_empty is high.
//   rd_data       : payload at current read pointer.  Valid whenever
//                   rd_empty is low.
//   rd_empty      : FIFO has nothing to pop this cycle.
//   rd_almost_empty: occupancy-low watermark crossed.  Advisory.
//
// Inference
// ─────────
// Storage is a simple dual-port RAM:
//   • DEPTH_LOG2 ≤ 5  → Vivado infers distributed RAM (LUTRAM) — small
//     depth, latency savings, matches Xilinx UG901 "Simple Dual-Port
//     RAM" LUT template.
//   • DEPTH_LOG2 ≥ 6  → `ram_style = "block"` is only a HINT.  The read
//     port below (`assign rd_data = mem[...]`) is COMBINATIONAL, and
//     UG901 BRAM primitives have no combinational read port — a true
//     RAMB18/36 SDP inference requires a registered read.  In practice
//     Vivado will keep this as distributed RAM (or reject the "block"
//     hint and warn) regardless of DEPTH_LOG2.  Do NOT rely on this
//     path to save BRAM; restructuring to a registered read (and the
//     corresponding one extra cycle of read latency the Cummings
//     pointer logic would need to account for) is tracked as separate
//     follow-up work, not done here.
// The `ram_style` attribute is applied below; synthesis tools silently
// fall back if the size/shape is inappropriate.
//
// Reset semantics
// ───────────────
// Each side has its own sync reset, but the two sides are CROSS-COUPLED
// internally via a request/ack handshake so a ONE-SIDED reset assertion
// is safe — no external wiring changes are required at any call site,
// and no minimum pulse width is required of the caller.
//
// Naive level-OR coupling (sync the raw reset directly into the other
// domain, effective_rst = local_raw | sync2(remote_raw)) has two real
// hazards this module avoids:
//
//   (a) MIN-PULSE HAZARD: a raw reset held for less than ~2 remote-clock
//       periods can be missed entirely by a plain 2-flop synchroniser —
//       the desync this module exists to prevent would simply return
//       for short pulses.
//   (b) ASSERT-WINDOW PHANTOM-POP HAZARD: if the local pointer is forced
//       to zero the INSTANT the local raw reset fires, that is a
//       multi-bit Gray-code jump (not the normal single-bit increment),
//       and the remote side may still be actively/live reading for the
//       ~2-3 remote-clock cycles it takes the remote side to even learn
//       about the reset.  In real silicon (not visible in Verilator,
//       which has no per-bit routing skew) the remote synchroniser can
//       sample a torn mix of old/new Gray bits mid-transition, producing
//       a transient value that was never a real pointer state and
//       causing a phantom pop of garbage data.
//
// A first handshake revision (T6 initial landing) closed both of the
// above but had two further, more subtle gaps found in review:
//
//   (c) GLITCH-SAMPLING HAZARD: `wrst`/`rrst` are only *documented* as
//       synchronous — real callers sometimes wire a combinational reset
//       cone straight in (e.g. ddr_ctrl.v's m-side `mig_ui_rst ||
//       !mig_cal_done`).  Exporting the engaged level directly into the
//       cross-domain synchroniser means that synchroniser's input can
//       itself glitch while that cone settles, risking a spurious
//       same-cycle capture.
//   (d) STALE-ACK RE-ENTRY HAZARD: after a handshake completes and a
//       side's sticky request clears, its 2-flop ack-receive pipe can
//       still hold a stale "1" draining out for a couple more local
//       cycles.  A SECOND, independent one-sided reset arriving inside
//       that stale window would read the leftover "1" as an ack for
//       itself (it isn't) and zero its pointer against a genuinely live
//       remote — reopening (b) for back-to-back resets.
//
// Fix: a request/ack handshake per direction, built from six 2-flop-or-
// simpler stages per side (all `ASYNC_REG`-tagged where they cross a
// domain):
//
//   req_w / req_r      : STICKY per-side request latches.  `req_w` sets
//                         the instant `wrst` fires and STAYS SET —
//                         regardless of how long `wrst` itself remains
//                         asserted — until released (see below).  This
//                         guarantees the outgoing signal synced into the
//                         other domain is a stable level, not a
//                         narrow pulse, for ANY remote clock ratio,
//                         fixing (a).  Symmetric for `req_r`.
//   req_w_s1/s2,
//   req_r_s1/s2        : 2-flop syncs of req_w into rclk and req_r into
//                         wclk.
//   w_eng / r_eng       : per-domain "engaged" level = local raw reset OR
//                         local sticky request OR the synced-in remote
//                         request.  Combinational; goes high IMMEDIATELY
//                         and gates LOCAL wr_full/rd_empty/wr_accept/
//                         rd_accept to their busy state right away —
//                         traffic is blocked from the very first cycle.
//                         Does NOT by itself zero the pointer, and is
//                         NOT what gets exported to the other domain
//                         (see w_eng_r/r_eng_r below) — fixing (c).
//   w_eng_r / r_eng_r  : registered, SOURCE-DOMAIN-LOCAL copies of
//                         w_eng/r_eng, one cycle behind.  ONLY these
//                         registered copies ever cross into the other
//                         domain's synchroniser, so that synchroniser's
//                         input is always a settled level, never a
//                         same-cycle combinational glitch.  Because
//                         local gating (above) still uses the
//                         combinational w_eng/r_eng directly, it engages
//                         a full cycle BEFORE its own registered export
//                         copy even latches — local gating strictly
//                         LEADS what the remote side can possibly
//                         observe, which only strengthens the "remote is
//                         already blind before I zero" argument below.
//   r_eng_s1/s2,
//   w_eng_s1/s2        : 2-flop syncs of r_eng_r back into wclk (the
//                         implicit ACK domain W watches) and of w_eng_r
//                         back into rclk (the implicit ACK domain R
//                         watches).  "Implicit" because it is simply the
//                         remote side's own engaged status, synchronised
//                         back — no separate ack protocol/toggle needed.
//                         Force-CLEARED to 0 every cycle the local side
//                         is not CURRENTLY engaged for any reason
//                         (`!w_eng` / `!r_eng` — NOT the narrower
//                         `!req_w`/`!req_r`: in a purely one-sided reset
//                         the engaging side's own req_w/req_r never
//                         latches at all, its engagement comes entirely
//                         via the RELAYED remote-request term in
//                         w_eng/r_eng, and that relay case legitimately
//                         needs to trust this ack pipe too — gating on
//                         req_w/req_r alone was tried first and broke
//                         every one-sided base scenario, since the
//                         force-clear was then permanently active
//                         whenever only the remote side ever raised a
//                         request) — fixing (d)'s "stale bits still
//                         sitting in the pipe" half.
//   r_eng_s2_valid,
//   w_eng_s2_valid     : `r_eng_s2 & w_eng`, `w_eng_s2 & r_eng`.  The
//                         force-clear above only guarantees the pipe
//                         STARTS clean by the time a domain next becomes
//                         engaged; it does not stop the exact
//                         transitional cycle (engagement just dropped,
//                         force-clear's registered effect not yet
//                         landed) from combinationally reading whatever
//                         stale value was there a moment earlier.
//                         Gating the ack READ by the CURRENT engaged
//                         value (a combinational AND, so it takes effect
//                         the same cycle) closes that residual
//                         single-cycle window too — fixing (d)'s other
//                         half.  A no-op when genuinely mid-handshake
//                         (already engaged).
//   w_zero / r_zero     : local engaged AND remote-engaged-observed
//                         (w_eng & r_eng_s2_valid, and r_eng &
//                         w_eng_s2_valid).  ONLY once this is true does
//                         the local pointer actually get forced to zero.
//                         By construction this happens strictly after
//                         the remote side's own r_eng/w_eng has already
//                         gated its wr_full/rd_empty (that gating is
//                         what `r_eng`/`w_eng` going high directly
//                         causes, and the ack path adds a full extra
//                         round-trip sync delay — through a registered
//                         export, not a raw glitchable one — on top of
//                         that) — so by the time the multi-bit
//                         jump-to-zero actually happens, the remote
//                         consumer is already provably blind to the
//                         source pointer's Gray value, fixing (b), and
//                         cannot be spoofed by a leftover ack from an
//                         earlier, already-completed handshake, fixing
//                         (d).
//   REQ_MIN_HOLD /
//   req_w_hold_cnt,
//   req_r_hold_cnt     : a THIRD-round review finding closed a residual
//                         gap in (a)/(d)'s interaction:
//   (e) RATIO-DEFEATED REQUEST-COLLAPSE HAZARD: once the Finding-3
//       force-clear/read-gate (d) is in place, a request can still
//       legitimately clear quickly if it happens to observe a GENUINE
//       (non-stale) ack from a still-in-flight CROSS-SIDE event within
//       just a couple of local cycles — collapsing the outgoing request
//       pulse's total visible duration to ~2 local cycles.  At a high
//       local:remote clock ratio (5:1 or higher) that ~2-cycle window
//       can fall entirely BETWEEN the remote domain's 2-flop
//       synchroniser samples, so the remote never engages for that
//       request at all.  If the remote had already popped a beat the
//       local side pushed just before this, the two sides' pointers can
//       end up persistently desynced — not a transient torn-bit issue
//       like (b)/(d), but a discrete, Verilator-observable miss.  All
//       current call sites hold their resets for many cycles, so this
//       was not exposed in practice, but the header's single-cycle-
//       pulse-safe claim was a genuine contract gap.  Fixed with a
//       per-side saturating minimum-hold counter: once `req_w`/`req_r`
//       latches, it stays asserted for AT LEAST `REQ_MIN_HOLD` local
//       cycles no matter what the ack does in the meantime — restoring
//       guaranteed request visibility regardless of how quickly a
//       genuine ack happens to arrive.  `REQ_MIN_HOLD = 8` (a
//       `localparam`, bump it if ever needed): 2 remote-domain sync
//       flops need the source level held stable for only ~1 full
//       remote-clock PERIOD to be guaranteed sampled at least once; 8
//       local cycles covers that for any local:remote ratio up to
//       ~1:4 (i.e. remote clock rate >= 1/4 of local), with margin —
//       verified in tb_async_fifo.cpp at a 1:5 ratio (which the 8-cycle
//       hold still comfortably covers).  After the min-hold has
//       elapsed, release falls back to the pre-existing ack-driven
//       logic unchanged.  The residual exposure after this fix
//       collapses to the already-documented (b)/(d) transient
//       real-silicon analog-tear class, not a discrete miss.
//
// RELEASE: req_w clears once (a) the minimum REQ_MIN_HOLD-cycle window
// has elapsed AND (b) the local raw `wrst` is low AND (c) the
// round-trip ack (r_eng_s2_valid) has been observed — i.e. domain W
// will not resume normal operation until it has positive confirmation
// domain R saw the request and engaged, and the request was visible for
// long enough to guarantee that regardless of ack timing. This works
// for any clock ratio up to the ~1:4 bound above (verified in
// tb_async_fifo.cpp at 1:5), including a stopped remote clock: if the
// remote clock never ticks, `req_w` (and thus `w_eng`, and thus
// wr_full/rd_empty gating) simply stays asserted forever, which is the
// SAFE outcome (the local side stays quiesced rather than proceeding on
// an unconfirmed reset). Free-running clocks converge in a small,
// bounded number of cycles in each domain (roughly: max(1 to latch +
// REQ_MIN_HOLD, round-trip-to-ack) + 1 to register the export + 2 to
// sync out + 2 to sync the ack back, per direction).
//
// This makes the FIFO safe under any one-sided reset scenario (e.g. a
// MIG UI recalibration that only pulses the read-side m_rst, or a PCIe
// link retrain that only pulses the write-side s_rst), for any relative
// clock ratio up to ~1:4 with margin verified at 1:5 (verified in
// tb_async_fifo.cpp with same-rate, 1:5-ratio, and phase-swept 1:5-ratio
// back-to-back-opposite-side scenarios), for a SINGLE-LOCAL-CYCLE pulse
// on the caller's raw reset input (the REQ_MIN_HOLD counter guarantees
// this, not just documents it), and for back-to-back one-sided resets
// separated by only a few cycles (verified in tb_async_fifo.cpp), all
// without requiring every instance site to hand-wire a shared/ANDed
// reset.
//
// synth: lives at the boundary between any two CPU clock domains in H0+.
// sim: Verilator can drive wclk and rclk independently in a unit tb; the
// full mac_top today still runs one clk (tb sees trivial identical-rate
// behaviour, which is still a valid FIFO test).

`default_nettype none

module async_fifo #(
    parameter WIDTH                   = 32,
    parameter DEPTH_LOG2              = 4,
    parameter ALMOST_FULL_THRESHOLD   = (1 << DEPTH_LOG2) - 2,
    parameter ALMOST_EMPTY_THRESHOLD  = 2
) (
    // ── Writer domain ──────────────────────────────────────────────
    input  wire              wclk,
    input  wire              wrst,
    input  wire              wr_en,
    input  wire [WIDTH-1:0]  wr_data,
    output wire              wr_full,
    output wire              wr_almost_full,
    output wire              wr_reset_engaged,

    // ── Reader domain ──────────────────────────────────────────────
    input  wire              rclk,
    input  wire              rrst,
    input  wire              rd_en,
    output wire [WIDTH-1:0]  rd_data,
    output wire              rd_empty,
    output wire              rd_almost_empty,
    output wire              rd_reset_engaged
);

    localparam DEPTH = (1 << DEPTH_LOG2);
    localparam PW    = DEPTH_LOG2 + 1;    // pointer width with MSB wrap bit

    // ── Storage (inferable BRAM or LUTRAM) ─────────────────────────
    // Style hint: LUTRAM for small depths, BRAM for large.  Vivado
    // recognises "distributed" and "block".
    (* ram_style = (DEPTH_LOG2 <= 5) ? "distributed" : "block" *)
    reg [WIDTH-1:0] mem [0:DEPTH-1];

    // ── Write-side state ───────────────────────────────────────────
    reg  [PW-1:0] wptr_bin;           // binary write pointer
    reg  [PW-1:0] wptr_gray;          // Gray write pointer (sent to rclk)
    reg  [PW-1:0] rptr_gray_w1;       // Gray read pointer synced into wclk, stage 1
    reg  [PW-1:0] rptr_gray_w2;       // ... stage 2 (used by writer)

    // ── Read-side state ────────────────────────────────────────────
    // MAX_FANOUT (2026-08-03): rptr_bin addresses the memory array AND its
    // decode reaches OUT of this module -- in ddr_ctrl's u_core_to_mig_ui the
    // aw_fifo's read pointer lands on distributed-RAM write-enables inside
    // u_repo_to_pcie_mig (axi_ddr4_mig_bridge).  At 333 MHz that fanout is the
    // sole source of every setup violation in the domain:
    //     u_ddr/u_core_to_mig_ui/aw_fifo/rptr_bin_reg[0]_replica/C
    //         -> u_ddr/u_repo_to_pcie_mig/wq_touched_reg[*]/D          (-0.253)
    //         -> u_ddr/u_repo_to_pcie_mig/wrout_data_reg_*/RAM*/WE     (-0.244)
    // Vivado already replicates it on its own (the _replica suffix) but not far
    // enough.  This caps the loads per copy so replication happens in synth,
    // where the copies can be placed near their loads, instead of as a
    // post-place patch.  Pure synthesis hint: no functional change, and the
    // gray-coded CDC pointers are untouched.
    (* MAX_FANOUT = 16 *)
    reg  [PW-1:0] rptr_bin;           // binary read pointer
    reg  [PW-1:0] rptr_gray;          // Gray read pointer (sent to wclk)
    reg  [PW-1:0] wptr_gray_r1;       // Gray write pointer synced into rclk, stage 1
    reg  [PW-1:0] wptr_gray_r2;       // ... stage 2 (used by reader)

    // Synchroniser attribute for Vivado CDC timing ignore.
    (* ASYNC_REG = "TRUE" *) reg dummy_sync_attr_marker_w; // (unused — attr hints)
    (* ASYNC_REG = "TRUE" *) reg dummy_sync_attr_marker_r;

    // ── Cross-domain reset request/ack handshake ────────────────────
    // See "Reset semantics" in the header comment for the full
    // rationale.  req_w/req_r are STICKY (latched, not free-running) so
    // a single-cycle raw reset pulse is guaranteed to be seen by the
    // other domain regardless of clock ratio.  w_eng/r_eng gate traffic
    // immediately (no hazard: purely local blocking).  w_zero/r_zero —
    // gated on the round-trip ack — are the only conditions that
    // actually zero a pointer, so the multi-bit Gray jump-to-zero always
    // happens strictly after the remote consumer has already gated
    // itself blind to the source pointer.

    // Sticky local request latches.
    reg req_w, req_r;

    // req_w synced into rclk (R's view of "W is requesting").
    (* ASYNC_REG = "TRUE" *) reg req_w_s1, req_w_s2;
    always @(posedge rclk) begin
        req_w_s1 <= req_w;
        req_w_s2 <= req_w_s1;
    end

    // req_r synced into wclk (W's view of "R is requesting").
    (* ASYNC_REG = "TRUE" *) reg req_r_s1, req_r_s2;
    always @(posedge wclk) begin
        req_r_s1 <= req_r;
        req_r_s2 <= req_r_s1;
    end

    // Per-domain "engaged" level: local raw reset, OR our own sticky
    // request, OR the remote's (synced) request.  Combinational — gates
    // LOCAL traffic (wr_full/rd_empty/wr_accept/rd_accept) the instant
    // any of these fire, with no handshake delay.  Does NOT by itself
    // zero a pointer, and (see below) is NOT what gets exported to the
    // other domain.
    wire w_eng = wrst | req_w | req_r_s2;
    wire r_eng = rrst | req_r | req_w_s2;
    assign wr_reset_engaged = w_eng;
    assign rd_reset_engaged = r_eng;

    // Registered EXPORT copies of the engaged level, one cycle behind
    // the combinational w_eng/r_eng above.  `wrst`/`rrst` are only
    // *documented* as synchronous — real callers sometimes feed a
    // combinational reset cone straight in (e.g. ddr_ctrl.v's m-side
    // `mig_ui_rst || !mig_cal_done`), so w_eng/r_eng can themselves
    // glitch transiently while that cone settles.  Registering here,
    // ONCE, in the SOURCE domain before the value ever crosses to the
    // other domain's synchroniser, guarantees the synchroniser's input
    // is always a clean, already-settled level — never a same-cycle
    // combinational glitch.  Local gating (wr_full/rd_empty/w_zero's/
    // r_zero's own local term) keeps using the combinational w_eng/
    // r_eng directly, so it still engages with zero delay; this
    // registration only affects what the REMOTE side observes, and
    // since local gating engages a full cycle BEFORE the exported copy
    // even latches, local gating strictly LEADS what the remote can
    // possibly see — which only strengthens the "remote is already
    // blind before I zero" argument the whole handshake depends on.
    reg w_eng_r, r_eng_r;
    always @(posedge wclk) w_eng_r <= w_eng;
    always @(posedge rclk) r_eng_r <= r_eng;

    // r_eng_r synced into wclk — the implicit ACK domain W watches to
    // learn "R has engaged (and is therefore already gated blind to my
    // pointer)".  Force-cleared to 0 every cycle W is not CURRENTLY
    // engaged for any reason (`!w_eng` — NOT `!req_w`: in a purely
    // one-sided reset, e.g. only rrst ever fires, req_w never latches at
    // all, but w_eng still goes high via the RELAYED req_r_s2 term, and
    // that relayed engagement is exactly the case that legitimately
    // needs to trust this ack pipe).  Without this force-clear, once a
    // handshake completes and w_eng drops, this 2-flop pipe can still
    // hold a stale "1" draining out for a couple more wclk cycles: if a
    // BRAND NEW reset (either side) arrives inside that stale window, it
    // would immediately read the leftover "1" as if R had already
    // engaged for THIS new event (it hasn't) and zero wptr against a
    // genuinely live R, reopening the exact phantom-pop hazard the
    // handshake exists to close — this time across two back-to-back
    // resets instead of one.  Gating on `w_eng`'s CURRENT (combinational,
    // but built purely from already-registered/input state) value rather
    // than on `wrst` alone matters: a new wrst arriving the very same
    // cycle the old engagement drops cannot suppress this clear, because
    // w_eng itself already reads 0 that cycle regardless.
    (* ASYNC_REG = "TRUE" *) reg r_eng_s1, r_eng_s2;
    always @(posedge wclk) begin
        if (!w_eng) begin
            r_eng_s1 <= 1'b0;
            r_eng_s2 <= 1'b0;
        end else begin
            r_eng_s1 <= r_eng_r;
            r_eng_s2 <= r_eng_s1;
        end
    end

    // w_eng_r synced into rclk — the implicit ACK domain R watches.
    // Same force-clear-while-not-engaged treatment, mirrored (gated on
    // `!r_eng`, for the same relay-case reason as above).
    (* ASYNC_REG = "TRUE" *) reg w_eng_s1, w_eng_s2;
    always @(posedge rclk) begin
        if (!r_eng) begin
            w_eng_s1 <= 1'b0;
            w_eng_s2 <= 1'b0;
        end else begin
            w_eng_s1 <= w_eng_r;
            w_eng_s2 <= w_eng_s1;
        end
    end

    // The force-clear above only guarantees the pipe STARTS clean by the
    // time w_eng next goes high — it does not, by itself, stop the exact
    // transitional cycle (the one where w_eng has just dropped to 0 but
    // the force-clear's registered effect hasn't landed yet) from
    // combinationally reading whatever stale value was still sitting in
    // r_eng_s2 a moment before.  Gate the ack read by w_eng's CURRENT
    // value too — a purely combinational AND, so it takes effect on that
    // exact same cycle, not one cycle later — so it is never trusted
    // outside a cycle where THIS domain is currently engaged for some
    // reason.  (When genuinely mid-handshake this is a no-op:
    // r_eng_s2_valid == r_eng_s2.)
    wire r_eng_s2_valid = r_eng_s2 & w_eng;
    wire w_eng_s2_valid = w_eng_s2 & r_eng;

    // Zero-gate: local engaged AND remote-engagement-observed (via the
    // glitch-free, stale-proof ack above).  Only condition that
    // actually forces a pointer to {PW{1'b0}}.
    wire w_zero = w_eng & r_eng_s2_valid;
    wire r_zero = r_eng & w_eng_s2_valid;

    // REQ_MIN_HOLD: minimum number of LOCAL cycles a sticky request stays
    // asserted after latching, regardless of ack — see the min-hold
    // counters below.  Without this, a request that happens to clear
    // quickly against a stale-but-now-legitimately-gated ack (i.e. one
    // that survived the Finding-3 force-clear/read-gate because it
    // genuinely belongs to a DIFFERENT, still-in-flight cross-side
    // event — see header comment hazard (e)) can collapse to as little
    // as ~2 local cycles.  At a high local:remote clock ratio that
    // window can fall entirely BETWEEN the remote domain's synchroniser
    // samples, so the remote never engages for that request at all.
    // REQ_MIN_HOLD=8 guarantees at least one full remote-clock period of
    // stable assertion (2 sync flops need only ~1 remote period to
    // definitely catch a level; 8 local cycles covers that for any
    // local:remote ratio up to ~1:4, with margin) — see the header
    // comment for the full ratio-bound writeup.  Bump this localparam if
    // a deployment ever needs a faster-than-~4x local:remote ratio.
    localparam REQ_MIN_HOLD  = 8;
    localparam HOLD_CNT_W    = 4;  // ceil(log2(REQ_MIN_HOLD+1)) with margin

    reg [HOLD_CNT_W-1:0] req_w_hold_cnt, req_r_hold_cnt;
    wire req_w_holding = (req_w_hold_cnt < REQ_MIN_HOLD[HOLD_CNT_W-1:0]);
    wire req_r_holding = (req_r_hold_cnt < REQ_MIN_HOLD[HOLD_CNT_W-1:0]);

    // Sticky request set/clear.  Set instantly on the local raw pulse
    // (latched, so even a 1-cycle pulse is captured) and held for AT
    // LEAST REQ_MIN_HOLD cycles no matter what the ack does in the
    // meantime (the min-hold counter restarts to 0 every cycle `wrst`
    // is asserted, so it only starts counting down once the raw pulse
    // itself has ended).  Only once that minimum has elapsed does
    // release fall back to waiting on the round-trip ack
    // (`r_eng_s2_valid`) as before — i.e. only once we know BOTH the
    // minimum-visibility window has passed AND the remote side has
    // definitely seen (and engaged on) our request.  If the remote
    // clock never ticks, this simply never clears — safe (permanently
    // quiesced, not an incorrect early release).
    always @(posedge wclk) begin
        if (wrst) begin
            req_w          <= 1'b1;
            req_w_hold_cnt <= {HOLD_CNT_W{1'b0}};
        end else if (req_w_holding) begin
            req_w          <= 1'b1;
            req_w_hold_cnt <= req_w_hold_cnt + {{(HOLD_CNT_W-1){1'b0}}, 1'b1};
        end else if (r_eng_s2_valid) begin
            req_w <= 1'b0;
        end
    end

    always @(posedge rclk) begin
        if (rrst) begin
            req_r          <= 1'b1;
            req_r_hold_cnt <= {HOLD_CNT_W{1'b0}};
        end else if (req_r_holding) begin
            req_r          <= 1'b1;
            req_r_hold_cnt <= req_r_hold_cnt + {{(HOLD_CNT_W-1){1'b0}}, 1'b1};
        end else if (w_eng_s2_valid) begin
            req_r <= 1'b0;
        end
    end

    // Binary↔Gray helpers (combinational).
    function [PW-1:0] bin2gray;
        input [PW-1:0] b;
        begin
            bin2gray = b ^ (b >> 1);
        end
    endfunction
    function [PW-1:0] gray2bin;
        input [PW-1:0] g;
        integer        i;
        reg   [PW-1:0] b;
        begin
            b[PW-1] = g[PW-1];
            for (i = PW-2; i >= 0; i = i - 1)
                b[i] = b[i+1] ^ g[i];
            gray2bin = b;
        end
    endfunction

    // ── Next-pointer arithmetic ────────────────────────────────────
    // wr_accept/rd_accept are gated on w_eng/r_eng (immediate) — no new
    // pushes/pops are accepted from the very first engaged cycle, same
    // as before.  wptr_bin_next/rptr_bin_next therefore naturally HOLD
    // steady (zero bits change) during the "engaged but not yet acked"
    // window; only w_zero/r_zero forces the actual jump-to-zero.
    wire            wr_accept = wr_en && !wr_full && !w_eng;
    wire            rd_accept = rd_en && !rd_empty && !r_eng;
    wire [PW-1:0]   wptr_bin_next  = wptr_bin  + {{(PW-1){1'b0}}, wr_accept};
    wire [PW-1:0]   rptr_bin_next  = rptr_bin  + {{(PW-1){1'b0}}, rd_accept};
    wire [PW-1:0]   wptr_gray_next = bin2gray(wptr_bin_next);
    wire [PW-1:0]   rptr_gray_next = bin2gray(rptr_bin_next);

    // ── Write-domain pointer update + RAM write ────────────────────
    always @(posedge wclk) begin
        if (w_zero) begin
            wptr_bin   <= {PW{1'b0}};
            wptr_gray  <= {PW{1'b0}};
            dummy_sync_attr_marker_w <= 1'b0;
        end else begin
            wptr_bin   <= wptr_bin_next;
            wptr_gray  <= wptr_gray_next;
            if (wr_accept) begin
                mem[wptr_bin[DEPTH_LOG2-1:0]] <= wr_data;
            end
            dummy_sync_attr_marker_w <= 1'b1;
        end
    end

    // ── Write-side sync of read pointer from rclk → wclk ───────────
    // 2-flop synchroniser — Gray code ensures at most one bit changes
    // per update, so metastability resolves to either old or new
    // pointer (never a garbled multi-bit intermediate).  Reset on
    // w_eng (immediate): forcing our own local receive-buffer to zero
    // early is always safe — it does not create a hazard for anyone
    // else, unlike the source pointer itself (wptr_bin/wptr_gray above,
    // gated on the delayed w_zero).
    (* ASYNC_REG = "TRUE" *) reg [PW-1:0] rptr_gray_w1_r;
    (* ASYNC_REG = "TRUE" *) reg [PW-1:0] rptr_gray_w2_r;
    always @(posedge wclk) begin
        if (w_eng) begin
            rptr_gray_w1_r <= {PW{1'b0}};
            rptr_gray_w2_r <= {PW{1'b0}};
        end else begin
            rptr_gray_w1_r <= rptr_gray;
            rptr_gray_w2_r <= rptr_gray_w1_r;
        end
    end
    always @(*) begin
        rptr_gray_w1 = rptr_gray_w1_r;
        rptr_gray_w2 = rptr_gray_w2_r;
    end

    // Full flag: classic Cummings form.  In Gray code, the FIFO is
    // full when the write pointer is one full lap ahead of the read
    // pointer:  wptr_gray == { ~rptr_gray_synced[PW-1:PW-2], rptr_gray_synced[PW-3:0] }
    wire [PW-1:0] rptr_gray_w_s = rptr_gray_w2_r;
    wire          full_cond     = (wptr_gray_next ==
                                   {~rptr_gray_w_s[PW-1:PW-2],
                                    rptr_gray_w_s[PW-3:0]});
    reg           wr_full_r;
    always @(posedge wclk) begin
        if (w_eng) wr_full_r <= 1'b0;
        else       wr_full_r <= full_cond;
    end
    assign wr_full = wr_full_r | w_eng;

    // Occupancy on writer side (binary subtract of synced rptr).
    wire [PW-1:0] rptr_bin_w  = gray2bin(rptr_gray_w_s);
    wire [PW-1:0] w_occupancy = wptr_bin - rptr_bin_w;
    assign wr_almost_full = !w_eng &&
                            (w_occupancy >= ALMOST_FULL_THRESHOLD[PW-1:0]);

    // ── Read-domain pointer update ─────────────────────────────────
    always @(posedge rclk) begin
        if (r_zero) begin
            rptr_bin   <= {PW{1'b0}};
            rptr_gray  <= {PW{1'b0}};
            dummy_sync_attr_marker_r <= 1'b0;
        end else begin
            rptr_bin   <= rptr_bin_next;
            rptr_gray  <= rptr_gray_next;
            dummy_sync_attr_marker_r <= 1'b1;
        end
    end

    // ── Read-side sync of write pointer from wclk → rclk ───────────
    // Reset on r_eng (immediate) — same rationale as the write-side
    // receive buffer above.
    (* ASYNC_REG = "TRUE" *) reg [PW-1:0] wptr_gray_r1_r;
    (* ASYNC_REG = "TRUE" *) reg [PW-1:0] wptr_gray_r2_r;
    always @(posedge rclk) begin
        if (r_eng) begin
            wptr_gray_r1_r <= {PW{1'b0}};
            wptr_gray_r2_r <= {PW{1'b0}};
        end else begin
            wptr_gray_r1_r <= wptr_gray;
            wptr_gray_r2_r <= wptr_gray_r1_r;
        end
    end
    always @(*) begin
        wptr_gray_r1 = wptr_gray_r1_r;
        wptr_gray_r2 = wptr_gray_r2_r;
    end

    // Empty flag: when synced wptr equals our rptr.
    wire empty_cond = (rptr_gray_next == wptr_gray_r2_r);
    reg  rd_empty_r;
    always @(posedge rclk) begin
        if (r_eng) rd_empty_r <= 1'b1;
        else       rd_empty_r <= empty_cond;
    end
    assign rd_empty = rd_empty_r | r_eng;

    // Occupancy on reader side.
    wire [PW-1:0] wptr_bin_r  = gray2bin(wptr_gray_r2_r);
    wire [PW-1:0] r_occupancy = wptr_bin_r - rptr_bin;
    assign rd_almost_empty = r_eng ||
                             (r_occupancy <= ALMOST_EMPTY_THRESHOLD[PW-1:0]);

    // ── RAM read port (combinational out of `mem`) ─────────────────
    // Match the SDP BRAM inference pattern: unregistered read data
    // indexed by binary read pointer.  For LUTRAM (DEPTH_LOG2 ≤ 5)
    // this is also correct.  Note the classic async-FIFO caveat: the
    // reader must gate rd_data consumption on rd_empty being low,
    // otherwise the first read after reset yields x (Verilator: 0).
    assign rd_data = mem[rptr_bin[DEPTH_LOG2-1:0]];

endmodule

`default_nettype wire
