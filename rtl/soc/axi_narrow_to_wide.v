// axi_narrow_to_wide.v — 32-bit AXI master → 128-bit AXI master adapter.
//
// The CPU LSU and boot_fsm are both 32-bit AXI masters.  The system xbar
// (and the DDR controller) are 128-bit AXI4.  This adapter turns narrow
// transactions into wide ones with correct byte-lane placement and wstrb
// replication.
//
// Two transaction shapes are supported:
//
//   * SINGLE BEAT (n_awlen / n_arlen == 0) — one 32-bit word placed in
//     the 128-bit lane selected by addr[3:2], one wide beat, awsize/arsize
//     derived from the narrow strobe/size so byte-addressed peripheral
//     registers keep their true address.  Bit-for-bit the behaviour this
//     module has always had.
//
//   * INCR BURST (n_awlen / n_arlen != 0, 32-bit beats) — task #170.
//     N narrow 4-byte beats are packed into ceil((lane0+N)/4) wide 16-byte
//     beats and issued as ONE wide INCR burst (awsize/arsize = 4 → 16 B).
//     The D-cache's 32 B line refill (n_arlen = 7) therefore becomes a
//     single 2-beat 128-bit read burst instead of eight independent
//     single-beat 32-bit reads, and a dirty-line writeback (n_awlen = 7)
//     becomes a single 2-beat 128-bit write burst instead of eight
//     independent AW+W+B round trips.  At a 40-cycle DDR round trip that
//     is the difference between ~339 and ~52 cycles for a clean miss.
//
// Byte-lane mapping (unchanged, and load-bearing — the m68k is big-endian
// and the 32-bit word's internal byte order is never touched here): the
// 32-bit word at byte address A lands in 128-bit lane A[3:2].  A burst
// therefore fills lanes 0,1,2,3 of wide beat 0 with the words at line
// base +0,+4,+8,+12 and lanes 0..3 of wide beat 1 with +16..+28 — exactly
// the same placement eight independent single-beat transactions produced.
//
// Sub-word store support (Design A, task P1) — SINGLE-BEAT path only:
//   Sub-word stores reach this adapter with a byte-granular `n_awaddr`
//   (e.g. 0x..3 for a byte at offset 3) and a one- or two-bit wstrb.
//   AXI4 requires `w_awaddr` to be aligned to `w_awsize` — so driving
//   `w_awsize=2` (4 B) with a byte-granular `w_awaddr` violates the spec
//   and `ddr_ctrl`'s `align_error(size, addr)` check correctly returns
//   SLVERR.  We derive the wide-side `awsize` from `n_wstrb`:
//
//     one contiguous bit (0001/0010/0100/1000) → awsize=0 (1 B)
//     two contiguous bits  (0011/0110/1100)    → awsize=1 (2 B)
//     any other pattern                         → awsize=2 (4 B, sparse)
//
//   and present a `w_awaddr` whose low bits satisfy the alignment rule
//   for that awsize.  `w_wstrb` remains the authoritative byte-lane
//   selector, so sparse patterns (non-contiguous) fall through to the
//   4-byte, word-aligned case and the slave honours whichever lanes
//   are strobed — AXI permits sparse byte strobes with awsize=2.
//
//   Sub-word load support:
//   Reads DO carry a size (`n_arsize`, mirroring `n_wstrb` on the write
//   side) sourced from the same `m68k_mem_strb`/`arsize_from_wstrb`
//   machinery in dcache.v that sizes bypass stores.  We derive a legal
//   `w_araddr`/`w_arsize` pair from it exactly like the AW path derives
//   `aw_addr_aligned`/`aw_size_der` from `n_wstrb`: arsize=0 (1 B) keeps
//   the byte-granular address untouched; arsize=1 (2 B) clears bit 0;
//   arsize=2 (4 B) clears bits[1:0].  This matters for byte-addressed,
//   side-effecting peripheral registers (SCC/VIA/IWM/SCSI/ADB) where
//   force-aligning the read address to the containing 32-bit word would
//   silently select the WRONG register (harmless for plain DRAM, wrong
//   for I/O).  The lane slice on the response side (`ar_lane_q`, bits
//   [3:2] of the ORIGINAL narrow address) is unaffected by this — it
//   only ever depended on the upper address bits, never on [1:0].
//
//   Bursts never take the derived-size path: a burst is by construction a
//   cache-line fill / writeback of naturally-aligned 4-byte beats, so the
//   wide side always uses awsize/arsize = 4 (16 B) at a 16-B-aligned
//   address, which is the only encoding a 128-bit slave can legally take
//   for a multi-beat transfer of full beats.
//
// MULTI-OUTSTANDING (2026-08-20).  This module used to hold exactly ONE
// transaction per direction: n_awready was `!aw_valid_q` and aw_valid_q
// cleared only on B, so the narrow master could not issue a second write
// until the first one's response had crossed the whole fabric and come
// back.  At the measured ~60-cycle B latency of the shipping chain
// (n2w -> l2c -> async bridge -> MIG bridge -> DDR) that is a full round
// trip of dead time between every transaction, and it is one of the two
// reasons the boot RAM pre-zero pass could not be pipelined (the other
// was l2c_ctrl's own front door, since fixed).
//
// WR_OUTSTANDING / RD_OUTSTANDING now bound how many narrow transactions
// may be in flight in each direction.  The structure that makes this
// cheap is that the NARROW SLAVE SIDE HAS NO ID SIGNALS (see the port
// list: n_b* is {resp,valid,ready}, n_r* is {data,resp,last,valid,ready})
// and the WIDE MASTER SIDE ISSUES EVERYTHING UNDER ONE CONSTANT ID
// (`ID_TAG`).  AXI4 requires an interconnect to return same-ID responses
// in issue order, so responses can never overtake each other and there is
// nothing to match: identity is positional at both ends.
//
//   WRITE — the front end (AW capture + the 128-bit gather buffer) still
//     owns exactly ONE transaction at a time, because width conversion on
//     the W channel is inherently sequential and partially-gathered beats
//     from two different writes must never interleave into one wide beat.
//     What changed is WHEN it lets go: the front end releases at the
//     transaction's last WIDE W BEAT instead of at B, and a credit
//     counter (`wr_out_q`) tracks how many accepted writes are still
//     owed a B.  So AW N+1 is accepted while N is still crossing the
//     fabric, and the B latency is pipelined away entirely.  B responses
//     are forwarded verbatim, in order, one per accepted AW.
//
//   READ — AR contexts are pushed into a small FIFO (`arq_*`).  Three
//     pointers walk it: `ar_wp` (push), `ar_ip` (wide-AR issue) and
//     `ar_rp` (response engine).  The wide AR for read N+1 goes out while
//     read N's data is still arriving, and the response engine dribbles
//     each context's beats out in order.  An empty-FIFO push bypasses
//     straight into the response engine so a lone read keeps exactly the
//     latency it had before.
//
// ONE TRANSACTION IN THE GATHER PATH — the invariant, re-derived for two
// banks.  Width conversion on W is inherently sequential, and the rule
// that keeps two different transactions' beats out of one wide beat is
// that the gather front end owns exactly one transaction at a time.
// 0994eaf8 got that by construction from a single bank; with two banks it
// has to be argued, so here it is.
//
//   CLAIM.  At every cycle in which a narrow AW is accepted, both gather
//   banks are empty, all eight strobe nibbles are zero, and no beat of
//   any other transaction is resident anywhere in the gather path.
//
//   1. n_awready requires !aw_valid_q.
//   2. aw_valid_q is set only by aw_take and cleared only by aw_release
//      or by a watchdog expiry.  (The expiry force-clears wo_valid_q and
//      wg_full_q in the same cycle, so that branch establishes the claim
//      directly.)
//   3. aw_release requires w_wvalid && w_wready && w_wlast, i.e. the beat
//      carrying WLAST has been TAKEN by the wide side.
//   4. w_wlast is wo_last_q, which is set to 1 only for the group that
//      contains the transaction's final narrow beat (cur_left == 1),
//      whether that group reaches wo_* by bypass or by transfer.
//   5. Wide beats leave the gather path in production order.  There are
//      exactly two producers of wo_*: wg_bypass, which requires
//      !wg_full_q (via place_en), and wg_xfer, which requires wg_full_q.
//      They are mutually exclusive, and a group can only park after every
//      earlier group of the same transaction has already been presented.
//      So nothing can overtake, and nothing later than the WLAST group
//      exists.
//   6. By 3-5, at the aw_release cycle wo_valid_q is being cleared by its
//      own handshake and wg_full_q is already 0.  aw_release additionally
//      force-clears both.
//   7. aw_valid_q is a register, so the earliest cycle a new AW can be
//      accepted is the cycle AFTER aw_release.  By 6 both banks are empty
//      then.
//   8. Strobes: wg_s0..3 are zeroed whenever a group LEAVES the gather
//      bank (both in the wg_bypass branch and in the wg_xfer branch) and
//      again at aw_take; wo_s0..3 are FULLY written — all four nibbles,
//      from exactly one group's data — on every load of wo_*, so wo_*
//      cannot union two groups' strobes even in principle.
//
//   Consequently the two banks always hold beats of the SAME transaction,
//   which is strictly stronger than "no strobe union across transactions"
//   and is what the mixed-burst scenario in tb_axi_widen.cpp and
//   scenarios G/H/I/J in tb_n2w_pipe.cpp pin.
//
//   The WATCHDOG's busy predicate does NOT change: wo_valid_q and
//   wg_full_q can only be set while aw_valid_q is 1 (by 6, both are clear
//   whenever aw_valid_q is 0), so `aw_valid_q || wr_busy` already covers
//   "a bank is stalled" for both banks.  The second bank did not create a
//   state the timer cannot see.
//
// MEASURED DEPTH CURVE.  `make tb-n2w-pipe` builds this module at each
// depth and streams a saturated narrow master against a multi-outstanding
// wide slave with a 60-cycle response latency (what the shipping
// n2w -> l2c -> async bridge -> MIG chain measures, and the same number
// tb_sd_boot_top's own fabric model uses).  Narrow-side cycles per 32-bit
// word, lower is better:
//
//                          depth 1   depth 2   depth 4   depth 8
//   single-beat writes       62.98     31.52     15.83      8.08
//   16-beat burst writes      4.87      2.47      1.32      1.18
//   64-beat burst writes      1.97      1.04      1.04      1.04
//   64-beat burst reads       2.20      1.28      1.28      1.28
//
// (The write rows are post-double-buffering; see GATHER OUTPUT DOUBLE
// BUFFERING below.  Before it they were 5.06/2.57/1.37/1.37 and
// 2.20/1.28/1.28/1.28.  The READ row is untouched and still carries the
// same 1.28 width-conversion floor on its own side.)
//
// SIZING, from that curve.  DEPTH 4 IS SHIPPED AND DEPTH 8 BUYS NOTHING
// ON ANY BURST SHAPE.  The 64-beat shapes saturate at depth 2 and the
// 16-beat shapes at depth 4; past that the narrow W channel is the limit,
// not the fabric round trip.  Only single-beat writes keep scaling, and
// they scale as latency/depth by construction (60/N) because there is
// nothing else to overlap — no real master here streams unrelated single
// -beat writes at full rate, so that column is a bound, not a workload.
// Depth 4 is chosen because it is the smallest depth that saturates every
// burst shape the actual masters use: the D-cache's 8-narrow-beat line
// fill / writeback, and boot_fsm's 64- or 256-beat RAM pre-zero bursts.
//
// GATHER OUTPUT DOUBLE BUFFERING (2026-08-20).
// ────────────────────────────────────────────
// The write rows above used to sit at a 1.28 floor that no amount of
// outstanding-ness could move, because it was not an outstanding-ness
// limit at all.  It was the gather buffer: four narrow beats pack into
// one wide beat, and with ONE bank the cycle that wide beat is handed to
// the wide side is a cycle n_wready is low — so four beats cost five, on
// every burst write in the design.
//
// There are now TWO banks (see the wg_* / wo_* declarations):
//
//   wg_* — GATHER.  Narrow beats accumulate here, lane by lane.
//   wo_* — OUTPUT.  Drives w_wdata / w_wstrb / w_wlast / w_wvalid.
//
// The beat that COMPLETES a group is written straight into wo_*, merged
// with the three lanes already in wg_*, and wg_* is freed in the same
// cycle (`wg_bypass`).  Nothing has to move afterwards, so the next
// group's lane 0 lands on the very next cycle and a burst streams one
// narrow beat per cycle.  When the wide side will not take the presented
// beat, the completing group parks in wg_* instead (`wg_park`,
// `wg_full_q`), backpressures the narrow W channel, and moves out whole
// when wo_* frees (`wg_xfer`).
//
// WHAT THIS COSTS, by construction (no Vivado run):
//   +146 flops  — wo_d0..3 (128) + wo_s0..3 (16) + wo_valid_q + wo_last_q
//                 + wg_full_q + wg_last_pend_q, less the two flops the
//                 single-bank version's wg_valid_q / wg_last_q used.
//   ~+150 LUT6  — one per wo_* data/strobe bit (144) for the bypass /
//                 transfer select, plus ~6 for wo_free / wg_bypass /
//                 wg_park / wg_xfer / wo_ld_l0..3.
//   On a KU5P that is +0.034% of the flops and +0.069% of the LUTs, per
//   instance.
//
// AND WHAT IT DOES NOT COST — this is the part that matters with a
// negative timing budget:
//   * NO added logic depth.  The wo_* select folds into the same LUT6
//     that already computes place_data (`w_place_skid ? wraw_data_q :
//     n_wdata`): five inputs, one level, exactly as the single-bank
//     wg_* input was.
//   * NO change to the wide-side W output path.  w_wdata / w_wstrb /
//     w_wlast / w_wvalid still come straight off flops; the second bank
//     went in FRONT of them as a flop stage, not behind them as a mux.
//   * NO new term on any ready/valid path.  n_wready's gather term goes
//     from !wg_valid_q to !wg_full_q — one register swapped for another.
//     w_wready appears only in `wo_free`, which is consumed by flop
//     inputs alone.  Putting the wide side's WREADY on a narrow-side
//     ready output is exactly what c2264764 spent a timing fix removing,
//     and what 0994eaf8 refused to undo for aw_release; it is not undone
//     here either.
//
// RESIDUAL, stated precisely: the burst rows land at 1.04, not 1.00.  The
// gather path itself is at exactly 1.0 — 64 narrow beats are accepted on
// 64 consecutive cycles, which tb-n2w-pipe scenario G asserts as a
// handshake property, not a rate.  What is left is one cycle per
// TRANSACTION (65 cycles per 64-beat burst, 17 per 16-beat burst): the
// deliberate front-end release bubble described on aw_release, which
// 0994eaf8 kept rather than fold aw_release into n_awready.  The 16-beat
// row at depth 4 (1.32) additionally hits the B-credit limit — depth 8
// takes it to 1.18 — because a faster W channel retires transactions
// faster than four credits cover a 60-cycle B latency.
//
// The READ side has the mirror-image dead cycle and is deliberately NOT
// touched here: rg_valid_q clears on the group-end narrow handshake, so
// the next wide beat is captured a cycle later and reads still measure
// 1.28.  A second rg_* bank is the same shape of change; it is a separate
// one.
//
// Depth must be a power of two (the read FIFO's pointers wrap naturally);
// an elaboration guard below enforces it.  Depth 1 reproduces the
// historical single-outstanding behaviour and is pinned by tb-n2w-pipe-1
// scenario F; it costs ~1.5% against the pre-rework code (62.98 vs 62.00
// on single beats) for the one-cycle front-end release bubble described
// on aw_release, which the old code got for free because AW acceptance
// waited for B anyway.
//
// Abandoned-transaction watchdog (2026-07-17 — JTAG-AXI 0xBADA0BAD
// bring-up investigation):
//   Both aw_valid_q (write) and ar_valid_q (read) are "one outstanding
//   transaction" latches with no escape if the wide (xbar) side never
//   completes the handshake: n_awready/n_arready are literally
//   `!aw_valid_q`/`!ar_valid_q`, so a wide-side stall (slave wedged,
//   an intermediate CDC bridge desynced, a starved arbiter grant, or
//   simply a downstream slave that will never answer) permanently
//   deasserts *READY on the narrow side and every future transaction
//   through this instance hangs forever — indistinguishable, from the
//   narrow master's perspective (e.g. Vivado's `hw_axi` JTAG-to-AXI
//   bridge driving `u_jtag_n2w` in fpga_top_debug_host.vh), from the
//   slave itself being dead.  `debug_ctrl.v` already had to defend its
//   OWN AXI-Lite slave against exactly this class of abandonment
//   (`r_read_timeout_cnt`, see rtl/core/debug/debug_ctrl.v) because a
//   killed JTAG/Tcl host process can walk away mid-transaction; this
//   module sits one hop closer to that same host and had no equivalent
//   protection at all.
//
//   TIMEOUT_CYCLES bounds how long aw_valid_q/ar_valid_q may sit
//   outstanding *without making progress* (see "SIZING THE TIMEOUT"
//   below for how the value is chosen).  Any accepted beat on any channel
//   restarts the counter, so a long but healthy burst can never trip it.
//   On expiry the abandoned transaction is TERMINATED TOWARDS THE NARROW
//   MASTER with a well-formed AXI error response (see "ABANDONMENT
//   RETURNS SLVERR" below) — the latch is force-cleared so *READY
//   eventually re-asserts, but the
//   wide-side slave (or an interconnect between here and it) may not
//   have been told the master gave up, so its eventual, late B/R beat is
//   still real and must not be silently dropped onto a *different*,
//   already-in-flight transaction.  A "drain" latch (aw_drain_q /
//   ar_drain_q) keeps *READY held low and keeps accepting-and-discarding
//   wide-side completions until the abandoned transaction's final beat
//   arrives (B, or R with RLAST — a burst's stale response can be more
//   than one beat), or until a second full timeout period elapses with
//   nothing arriving, at which point recovery gives up waiting and
//   resumes normal operation — a downstream that has been silent for two
//   full timeout windows is not going to answer a stale request either.
//   This makes the watchdog safe against the classic "abandon now, stale
//   response mis-routed onto the next request" CDC/reset-asymmetry
//   hazard, at the cost of one extra timeout-period stall in the
//   (already rare, recovery-path-only) worst case.
//
//   ABANDONMENT RETURNS SLVERR (2026-08-02).
//   ─────────────────────────────────────────
//   Force-clearing aw_valid_q / ar_valid_q used to be the WHOLE of
//   "recovery", and it answered the narrow master with nothing at all:
//   n_bvalid was `aw_valid_q && w_bvalid` and n_rvalid was
//   `ar_valid_q && rg_valid_q`, both of which the watchdog had just
//   zeroed.  The master therefore waited forever — the CPU LSU parked in
//   LD_WAIT on a peripheral load, Vivado `run_hw_axi` blocking with no
//   output.  An abandoned transaction was an UNDIAGNOSABLE WEDGE.
//
//   It now terminates as a bus error instead, which the CPU already
//   knows how to handle: rtl/core/mem/lsu.v tests `dc_rresp != 2'b00`
//   and `dc_bresp != 2'b00` and folds them into cmpl_exc /
//   store_commit_exc, so the abandonment surfaces as a precise vector-2
//   access fault carrying a fault address.  That is also what real 68040
//   hardware does when a device does not answer.  A fault is
//   diagnosable; a hang is not.  NOTE this is a MITIGATION, not a fix:
//   a wide side that stops answering is still a bug and is tracked
//   separately.  All this does is convert the wedge into a fault.
//
//   The response is well-formed, because a half-formed one is just
//   another hang:
//
//     WRITE  — one BVALID with BRESP=SLVERR, held until BREADY
//              (`aw_err_q`).  AXI4 gives exactly one B per transaction,
//              burst or not, so one beat is the complete response.
//              Any narrow W beats the master had not yet handed over
//              (`aw_wrx_left_q`, decremented on every narrow W
//              handshake) are accepted-and-discarded first
//              (`aw_wswal_q`) so the W channel is not left misaligned
//              into the master's NEXT write — that would silently
//              corrupt an unrelated transaction, which is worse than
//              the hang this replaces.
//
//     READ   — EVERY remaining beat of the burst, each with
//              RRESP=SLVERR, RLAST on the final one (`ar_err_left_q`,
//              seeded from `ar_beats_left_q`, which is AxLEN+1 at AR
//              capture and already decremented by every narrow beat
//              delivered BEFORE the watchdog fired).  Delivering a
//              single beat for a multi-beat read would leave the master
//              waiting for the rest, i.e. replace one hang with another.
//              RDATA on those beats is 0 (AXI leaves it don't-care when
//              RRESP is an error; 0 matches the crossbar's local-response
//              convention in macqd700-soc's axi_xbar.v).
//
//   There is no BID/RID to preserve: the narrow slave side of this
//   adapter has no ID signals at all (see the port list — n_b* is
//   {resp,valid,ready}, n_r* is {data,resp,last,valid,ready}).  It is a
//   strictly one-outstanding-transaction-per-direction port, so response
//   identity is positional, not tagged.  The WIDE side's w_bid/w_rid are
//   already unused for the same reason.
//
//   Both *READY stay deasserted until the synthesized response has been
//   taken, so the error can never be mis-attributed to a later
//   transaction accepted in the meantime.
//
//   NOT AFFECTED: unmapped-address open bus.  A read of unmapped space
//   is answered by the crossbar's local-response path with RRESP=OKAY
//   (macqd700-soc axi_xbar.v `local_rsp_is_err_r`, which is asserted
//   only for burst-reject / poisoned / flush-reject, never for a plain
//   XBAR_SLV_NONE decode).  This adapter forwards `w_rresp` verbatim on
//   that path and only substitutes SLVERR on a watchdog expiry of an
//   ACCEPTED transaction, so RAM sizing at boot — which probes unmapped
//   space and requires a non-error answer — is untouched.
//
//   SIZING THE TIMEOUT — INVERTED LAYERING BUG, fixed 2026-08-02.
//   ─────────────────────────────────────────────────────────────
//   The original default was 20'hFFFFF (~10.5 ms @ 100 MHz core_clk,
//   ~5.2 ms @ 200 MHz), chosen to match debug_ctrl.v's
//   R_READ_TIMEOUT_MAX.  That is WRONG for every instance whose wide
//   side faces the SoC crossbar, because it inverts the watchdog
//   layering: the innermost adapter gives up long before the fabric
//   underneath it has even finished trying.
//
//   The stack, from the slave outwards (values from macqd700-soc):
//     * peripheral_bus.v PB_WATCHDOG_LOG2 = 24 -> 2^24 pb_clk cycles
//       ~= 335 ms @ 50 MHz.  Sized for a DRQ-checked SCSI beat waiting
//       on real SD-card latency (>100 ms worst case).
//     * axi_xbar.v WD_LOG2_S1      = 27 -> 2^27 core_clk cycles
//       (~1.34 s @ 100 MHz, ~671 ms @ 200 MHz).  Deliberately chosen to
//       exceed the peripheral bus's own guarantee.
//     * THIS module.  At 2^20 it fired ~128x earlier than the xbar
//       bound it is supposed to sit outside of, so a perfectly healthy
//       but slow VIA/SCC/SCSI access was abandoned mid-flight.
//
//   Abandonment is not benign even now that it returns SLVERR (see
//   "ABANDONMENT RETURNS SLVERR" above): a healthy-but-slow peripheral
//   access that trips this watchdog becomes a spurious vector-2 access
//   fault, which the OS will report as a bus error on a perfectly good
//   address.  A diagnosable fault beats the permanent hang this used to
//   produce, but the watchdog must still be sized so it never fires on
//   a transaction the fabric underneath is going to complete.
//
//   The default is therefore now 2^28-1 = 32'h0FFF_FFFF core_clk
//   cycles: strictly greater than the xbar's largest per-slave bound
//   (2^27-1) with 2x margin, so on any xbar-connected instance the
//   FABRIC always terminates the transaction first (with SLVERR if the
//   slave really is dead) and this watchdog never fires at all.  It is
//   a genuine last resort for a wide side that is not merely slow but
//   structurally absent.  Wall-clock cost of that last resort:
//   ~2.68 s @ 100 MHz core_clk (the shipping build), ~1.34 s @ 200 MHz.
//
//   TIMEOUT_CYCLES and both counters are 32 bits wide.  They were 20,
//   with hardcoded 20'd0 / 20'd1 literals, which meant a larger
//   override silently truncated — raising the parameter alone was NOT
//   a valid fix.
//
// Latency: AW/AR appear one cycle after the narrow *VALID.  Read data is
// registered once (`rg_*`) before being sliced back to the narrow side —
// this both removes a wide-R→narrow-R combinational path (Fmax) and is
// what lets one wide beat feed four narrow beats.

`default_nettype none

module axi_narrow_to_wide #(
    parameter ID_WIDTH   = 4,
    parameter [ID_WIDTH-1:0] ID_TAG = {ID_WIDTH{1'b0}},
    // Cycles an outstanding write/read transaction may sit WITHOUT
    // PROGRESS before the watchdog force-clears it.  See the
    // "SIZING THE TIMEOUT" block in the header for why this must exceed
    // the downstream fabric's own per-slave watchdog, and why the field
    // is 32 bits and not the original 20.  Overridable so a unit tb can
    // shrink it to a handful of cycles.
    // 2026-08-03: 0x0FFF_FFFF (2.68 s @ 100 MHz) RE-INVERTED THE LAYERING
    // THIS FILE'S HEADER DOCUMENTS FIXING.  sd_ctrl's per-request watchdog
    // was raised ~1 s -> ~10.07 s the same day (macqd700-soc 9e55a27), so
    // the ordering became:
    //     AXI abandonment  2.68 s   -> vector-2 BUS ERROR (fatal)
    //     sd_ctrl watchdog 10.07 s  -> retryable CHECK CONDITION (graceful)
    // The fatal path pre-empted the graceful one by 3.75x, so sd_ctrl's
    // bounded-response recovery could NEVER run: any SD read slower than
    // 2.68 s became a System Error instead of a retry.
    //
    // 0x7735_9400 = 2,000,000,000 cycles = 20 s @ 100 MHz, ~2x sd_ctrl's
    // worst case (245760 + (blocks<<5) ticks x 4096 = 10.40 s at 256
    // blocks).  Both counters live on core_clk, so the ratio is
    // clock-independent.
    //
    // THE INVARIANT, restated because breaking it is silent and fatal:
    // this value MUST exceed every downstream per-slave watchdog.  If you
    // raise one of those, raise this one too, or the graceful recovery
    // path you just tuned becomes unreachable.
    // ABANDONMENT WATCHDOG -- DISABLED BY DEFAULT (2026-08-20).
    //
    // Set ENABLE_ABANDON_TIMEOUT to 1 to restore it.  At 0 the fire condition
    // is a constant false, so synthesis strips both 32-bit counters, the
    // aw_err_q/aw_wswal_q/ar_err_left_q SLVERR-response machinery they feed,
    // and -- the reason it was killed -- the combinational path from the live
    // WREADY into a 20-second timer.  A post-route trace measured that segment
    // at 3.014 ns of a 9.519 ns path (31.7%), the largest single segment of
    // the design's #2 critical path.
    //
    // WHAT IS GIVEN UP, stated plainly: a wide slave that never answers now
    // hangs the narrow master forever instead of being abandoned with SLVERR
    // after 20 s.  That is a real loss of liveness recovery.  It is a
    // defensible trade here because 20 s is already indistinguishable from a
    // hang for an interactive machine, because the SLVERR it synthesises has
    // itself caused misdiagnosis (a peripheral-bus ack watchdog's SLVERR
    // presented as a CPU bus error and sent an investigation after the wrong
    // fault for hours on 2026-08-19), and because a true fabric wedge is more
    // diagnosable as a stuck transaction over JTAG than as a late bus error.
    parameter integer ENABLE_ABANDON_TIMEOUT = 0,
    parameter [31:0] TIMEOUT_CYCLES = 32'h7735_9400,
    // Maximum narrow transactions in flight per direction.  MUST be a
    // power of two (the read context FIFO's pointers wrap naturally); the
    // elaboration guard below stops a non-power-of-two silently mis-sizing
    // the array.  1 restores the historical single-outstanding behaviour
    // exactly and is what the watchdog tbs' single-transaction scenarios
    // would see either way.  See "MEASURED DEPTH CURVE" in the header for
    // why 4 is shipped.
    parameter integer WR_OUTSTANDING = 4,
    parameter integer RD_OUTSTANDING = 4
) (
    input  wire                   clk,
    input  wire                   rst,

    // ── Narrow (32-bit) slave side (faces the CPU / boot_fsm master) ─
    // AW
    input  wire [31:0]            n_awaddr,
    input  wire [2:0]             n_awprot,
    input  wire [7:0]             n_awlen,    // 0 = single beat, else INCR burst
    input  wire                   n_awvalid,
    output wire                   n_awready,
    // W
    input  wire [31:0]            n_wdata,
    input  wire [3:0]             n_wstrb,
    input  wire                   n_wlast,
    input  wire                   n_wvalid,
    output wire                   n_wready,
    // B
    output wire [1:0]             n_bresp,
    output wire                   n_bvalid,
    input  wire                   n_bready,
    // AR
    input  wire [31:0]            n_araddr,
    input  wire [2:0]             n_arprot,
    input  wire [2:0]             n_arsize,   // 0=1B,1=2B,2=4B (mirrors n_wstrb sizing)
    input  wire [7:0]             n_arlen,    // 0 = single beat, else INCR burst
    input  wire                   n_arvalid,
    output wire                   n_arready,
    // R
    output wire [31:0]            n_rdata,
    output wire [1:0]             n_rresp,
    output wire                   n_rlast,
    output wire                   n_rvalid,
    input  wire                   n_rready,

    // ── Wide (128-bit) master side (onto the xbar / DDR) ──────────────
    // AW
    output wire [ID_WIDTH-1:0]    w_awid,
    output wire [31:0]            w_awaddr,
    output wire [7:0]             w_awlen,
    output wire [2:0]             w_awsize,
    output wire [1:0]             w_awburst,
    output wire                   w_awvalid,
    input  wire                   w_awready,
    // W
    output wire [127:0]           w_wdata,
    output wire [15:0]            w_wstrb,
    output wire                   w_wlast,
    output wire                   w_wvalid,
    input  wire                   w_wready,
    // B
    input  wire [ID_WIDTH-1:0]    w_bid,
    input  wire [1:0]             w_bresp,
    input  wire                   w_bvalid,
    output wire                   w_bready,
    // AR
    output wire [ID_WIDTH-1:0]    w_arid,
    output wire [31:0]            w_araddr,
    output wire [7:0]             w_arlen,
    output wire [2:0]             w_arsize,
    output wire [1:0]             w_arburst,
    output wire                   w_arvalid,
    input  wire                   w_arready,
    // R
    input  wire [ID_WIDTH-1:0]    w_rid,
    input  wire [127:0]           w_rdata,
    input  wire [1:0]             w_rresp,
    input  wire                   w_rlast,
    input  wire                   w_rvalid,
    output wire                   w_rready
);

    // AXI *SIZE encoding for a full 128-bit (16 byte) beat.
    localparam [2:0] WIDE_SIZE = 3'd4;
    // AXI4 xRESP encodings.  Only OKAY and SLVERR are ever produced
    // locally here; everything else is forwarded verbatim from the wide
    // side.  SLVERR (not DECERR) is the right code for "the transaction
    // was accepted and then never completed": there IS a decoded slave,
    // it just failed to answer.  DECERR means "no slave decoded", which
    // is emphatically NOT what happened and would also collide with the
    // unmapped-address open-bus policy documented in the header.
    localparam [1:0] RESP_SLVERR = 2'b10;
    // Beat-counter width: up to 256 narrow beats (AXI max AxLEN=255) → 9 b.
    localparam       BEATW     = 9;

    // ── Outstanding-transaction sizing ────────────────────────────────
    // Credit counters need to represent 0..DEPTH inclusive, hence +1.
    localparam integer WCNTW = $clog2(WR_OUTSTANDING + 1);
    localparam integer RCNTW = $clog2(RD_OUTSTANDING + 1);
    // Read context FIFO: RD_OUTSTANDING entries, power-of-two so the
    // pointers wrap for free.  Pointers carry one extra MSB so
    // "empty" (wp == rp) is distinguishable from "full".  Minimum index
    // width of 1 keeps depth-1 legal without a zero-width vector.
    localparam integer RPTRW = (RD_OUTSTANDING <= 1) ? 1 : $clog2(RD_OUTSTANDING);
    localparam integer RQ_N  = (1 << RPTRW);

    // synthesis translate_off
    initial begin
        if ((WR_OUTSTANDING < 1) ||
            ((WR_OUTSTANDING & (WR_OUTSTANDING - 1)) != 0))
            $error("axi_narrow_to_wide: WR_OUTSTANDING (%0d) must be a power of two >= 1",
                   WR_OUTSTANDING);
        if ((RD_OUTSTANDING < 1) ||
            ((RD_OUTSTANDING & (RD_OUTSTANDING - 1)) != 0))
            $error("axi_narrow_to_wide: RD_OUTSTANDING (%0d) must be a power of two >= 1",
                   RD_OUTSTANDING);
    end
    // synthesis translate_on

    // Number of wide beats - 1 needed to carry AxLEN+1 narrow 4-byte
    // beats when the first one lands in lane addr[3:2]: the narrow beats
    // occupy lanes [lane .. lane+AxLEN], so the span covers
    // ((lane + AxLEN) >> 2) + 1 wide beats.
    wire [9:0] aw_wlen_sum  = {2'b00, n_awlen} + {8'b0, n_awaddr[3:2]};
    wire [7:0] aw_wlen_calc = aw_wlen_sum[9:2];
    // (The read side's equivalent is computed at ISSUE time from the AR
    // context FIFO — see ari_wlen below — because the wide AR for read
    // N+1 goes out while read N's data is still coming back.)

    // ═══════════════════════════════════════════════════════════════════
    // Write path
    // ═══════════════════════════════════════════════════════════════════
    reg [31:0]      aw_addr_q;
    reg [1:0]       aw_lane_q;        // lane the NEXT narrow W beat lands in
    reg             aw_burst_q;
    reg [7:0]       aw_wlen_q;        // wide AWLEN for this transaction
    reg [BEATW-1:0] aw_beats_left_q;  // narrow W beats still to be placed
    reg             aw_valid_q;       // AW captured, transaction outstanding
    reg             aw_accepted_q;    // wide AW has been accepted
    reg [3:0]       aw_first_strb_q;  // first narrow wstrb (sizes single beats)
    reg             aw_strb_known_q;

    // ── DOUBLE-BUFFERED 128-bit gather (2026-08-20) ──────────────────
    // TWO banks, not one:
    //
    //   wg_* — the GATHER bank.  Narrow beats accumulate here, lane by
    //          lane, exactly as they always have.
    //   wo_* — the OUTPUT bank.  This, and only this, drives the wide W
    //          channel.  w_wdata / w_wstrb / w_wlast / w_wvalid are still
    //          straight flop outputs — the wide-side output path is
    //          bit-for-bit the same logic depth it was with one bank.
    //
    // WHY.  With a single bank the cycle the assembled wide beat is
    // handed to the wide side is a cycle nothing can be placed into it,
    // so four narrow beats cost five narrow cycles (1.25 cycles/word,
    // 1.28 measured with the per-transaction overhead).  With two, the
    // completing narrow beat of a group is written STRAIGHT INTO wo_*
    // (`wg_bypass`) while wg_* is simultaneously freed for the next
    // group, so the handover costs nothing and a burst streams at one
    // narrow beat per cycle.
    //
    // The merge is a plain 2:1 select on the wo_* flop INPUTS
    // (`place_data` for the lane being placed, `wg_*` for the other
    // three).  It shares a LUT with the skid select that already feeds
    // `place_data`, so it adds a flop stage and NO combinational depth,
    // and it deliberately does not touch any ready/valid path:
    // `n_wready` reads `wg_full_q`, a register, never `w_wready`.
    reg [31:0]      wg_d0, wg_d1, wg_d2, wg_d3;
    reg [3:0]       wg_s0, wg_s1, wg_s2, wg_s3;
    // The gather bank holds a COMPLETE wide beat it could not hand to the
    // output bank, because the wide side had not taken the previous one.
    // This is the only thing that backpressures the narrow W channel, and
    // it can only ever be set by wide-side backpressure — in a stream
    // against a ready slave it is never set at all.
    reg             wg_full_q;
    reg             wg_last_pend_q;   // ...and that parked beat is the last
    // Output bank — the wide W channel's registers.
    reg [31:0]      wo_d0, wo_d1, wo_d2, wo_d3;
    reg [3:0]       wo_s0, wo_s1, wo_s2, wo_s3;
    reg             wo_valid_q;       // an assembled wide beat is presented
    reg             wo_last_q;        // ...and it is the final wide beat

    // 1-deep raw skid for the (protocol-legal but unused by any real
    // master here) case of WVALID arriving before AWVALID: we cannot know
    // the target lane yet, so hold the beat verbatim until AW lands.
    reg [31:0]      wraw_data_q;
    reg [3:0]       wraw_strb_q;
    reg             wraw_valid_q;

    // Write-abandonment watchdog state — see module header comment.
    reg [31:0]      aw_timeout_cnt;
    reg             aw_drain_q;       // absorb a stale wide B beat; AWREADY low
    // ── Abandonment error-response state (see "ABANDONMENT RETURNS
    //    SLVERR" in the header) ──────────────────────────────────────
    // aw_err_q     : a BRESP=SLVERR is owed to the narrow master and has
    //                not been taken yet.  Exactly one B per abandoned
    //                write, held until n_bready, per AXI4.
    // aw_wswal_q   : narrow W beats still to be accepted-and-discarded
    //                so the abandoned burst does not leave the W channel
    //                misaligned into the master's NEXT write.
    // aw_wrx_left_q: narrow W beats of the CURRENT transaction that the
    //                master has not handed over yet.  Distinct from
    //                aw_beats_left_q, which counts beats not yet PLACED
    //                into the gather buffer and therefore lags by the
    //                one-deep raw skid.  Seeds aw_wswal_q on expiry.
    // aw_err_cnt_q counts SLVERR B responses still owed to the narrow
    // master.  It is a COUNT, not a flag, because an expiry with N writes
    // in flight owes N responses — one per AW the master handed over.
    reg [WCNTW-1:0] aw_err_cnt_q;
    reg [BEATW-1:0] aw_wswal_q;
    reg [BEATW-1:0] aw_wrx_left_q;
    wire            aw_wswal_busy = (aw_wswal_q != {BEATW{1'b0}});
    // Kept as a 1-bit wire under its historical name so the unit-tb
    // harnesses' hierarchical taps (tb/tb_n2w_watchdog.v) still read
    // "an abandoned write is owed a response".
    wire            aw_err_q      = (aw_err_cnt_q != {WCNTW{1'b0}});

    // ── Multi-outstanding write bookkeeping ───────────────────────────
    // wr_out_q     : narrow AWs accepted whose narrow B has not been
    //                returned.  This is the credit that bounds how many
    //                writes the master may have in flight, and it is what
    //                gates n_bvalid: a wide B is only ever forwarded if
    //                some accepted write is still owed one.
    // aw_iss_out_q : wide AWs handshaken whose wide B has not come back.
    //                Only used to size the abandonment drain — it is the
    //                exact number of late B beats the fabric may still
    //                produce for transactions we have given up on.
    reg [WCNTW-1:0] wr_out_q;
    reg [WCNTW-1:0] aw_iss_out_q;
    // One bit wider than the credit counters it is seeded from: at
    // DEPTH = 1 those collapse to a single bit and the "<= 1" test below
    // would be constant-true, which is both a lint warning and a real
    // loss of the distinction between "one late B still owed" and "none".
    reg [WCNTW:0]   aw_drain_cnt_q;
    wire            wr_full   = (wr_out_q == WR_OUTSTANDING[WCNTW-1:0]);
    wire            wr_busy   = (wr_out_q != {WCNTW{1'b0}});

    wire            aw_take   = n_awready && n_awvalid;
    wire            w_take    = n_wready  && n_wvalid;
    // The lane/burst/count context for a W beat arriving right now: from
    // the registers once AW is captured, combinationally off the AW
    // channel on the (common) cycle AW and W are accepted together.
    wire            aw_ctx_ok = aw_valid_q || aw_take;
    wire [1:0]      cur_lane  = aw_valid_q ? aw_lane_q  : n_awaddr[3:2];
    wire            cur_burst = aw_valid_q ? aw_burst_q : (n_awlen != 8'd0);
    wire [BEATW-1:0] cur_left = aw_valid_q ? aw_beats_left_q
                                           : ({1'b0, n_awlen} + {{(BEATW-1){1'b0}}, 1'b1});

    // A wide beat is complete when the gather buffer's last lane is
    // written, or when the narrow burst runs out of beats first.
    wire            wg_group_end = (cur_lane == 2'd3) || (cur_left == {{(BEATW-1){1'b0}}, 1'b1});
    // Direct placement is possible when we know the lane and the gather
    // bank is not parked (i.e. it is not holding a complete wide beat the
    // output bank could not take).  With double buffering this is NO
    // LONGER "the previous wide beat has been handed over": a group can
    // start accumulating while the previous group is still being
    // presented on the wide W channel, which is exactly the dead cycle
    // this removes.
    wire            w_place_now  = w_take && aw_ctx_ok && !wg_full_q;
    // Deferred placement of a skidded beat, once AW has landed.
    wire            w_place_skid = wraw_valid_q && aw_valid_q && !wg_full_q;

    // Data/strobe actually being placed, and where.
    wire [31:0]     place_data = w_place_skid ? wraw_data_q : n_wdata;
    wire [3:0]      place_strb = w_place_skid ? wraw_strb_q : n_wstrb;
    wire            place_en   = w_place_now || w_place_skid;

    // ── Gather → output handover ─────────────────────────────────────
    // The output bank can take a beat this cycle if it is empty, or if
    // the one it holds is being taken right now.  This term contains the
    // live wide-side WREADY *and is only ever consumed by flop inputs*
    // (wo_valid_q, wg_full_q).  It must never reach n_wready / n_awready:
    // that is the segment c2264764 spent a timing fix removing, and the
    // same reason 0994eaf8 kept aw_release out of n_awready.
    wire            wo_free   = !wo_valid_q || (w_wvalid && w_wready);
    // A group that completes while the output bank is free goes straight
    // there, merged with the beat being placed on this very cycle.  This
    // is the whole optimisation: no cycle is spent moving it.
    wire            wg_bypass = place_en && wg_group_end && wo_free;
    // A group that completes while the output bank is busy parks in the
    // gather bank and backpressures the narrow W channel next cycle.
    wire            wg_park   = place_en && wg_group_end && !wo_free;
    // ...and moves out as soon as the output bank frees.  Mutually
    // exclusive with placement by construction: place_en requires
    // !wg_full_q, wg_xfer requires wg_full_q.
    wire            wg_xfer   = wg_full_q && wo_free;
    // Per-lane DATA select for the merged bypass write.  Non-burst single
    // beats replicate into every lane (historical, load-bearing — see the
    // byte-lane mapping note in the header), so every lane takes
    // place_data; a burst takes place_data only in the lane being placed
    // and the gather bank's contents everywhere else.  The STROBE select
    // is deliberately different — see the wo_s0..3 assignments.
    wire            wo_ld_l0  = wg_bypass && (!cur_burst || (cur_lane == 2'd0));
    wire            wo_ld_l1  = wg_bypass && (!cur_burst || (cur_lane == 2'd1));
    wire            wo_ld_l2  = wg_bypass && (!cur_burst || (cur_lane == 2'd2));
    wire            wo_ld_l3  = wg_bypass && (!cur_burst || (cur_lane == 2'd3));

    wire            aw_progress = (w_awvalid && w_awready)
                                || (w_wvalid  && w_wready)
                                || (n_wvalid  && n_wready)
                                || (w_bvalid  && w_bready);

    // ── Front-end release (multi-outstanding) ─────────────────────────
    // The gather front end hands its transaction over to the credit
    // counter as soon as the LAST WIDE W BEAT has been taken by the wide
    // side — not when B comes back.  Everything the transaction still
    // needs from this module after that point is "one B, in order",
    // which wr_out_q tracks without any per-transaction state.
    //
    // The wide AW must have been issued first.  A wide slave is entitled
    // to accept W before AW, and dropping aw_valid_q with w_awvalid still
    // high would lose the address entirely.
    //
    // DELIBERATELY NOT COMBINATIONAL INTO n_awready.  Folding aw_release
    // into n_awready would remove a one-cycle bubble between back-to-back
    // transactions, at the price of a w_wready -> n_awready path.  That is
    // the exact combinational segment this file spent a whole timing fix
    // getting OFF the live WREADY (see the aw_progress_q comment below),
    // and the bubble it buys back is one cycle per transaction — 1.6% on
    // the 64-beat bursts the boot pre-zero and D-cache traffic use.
    wire            w_last_hs   = w_wvalid && w_wready && w_wlast;
    wire            aw_iss_ok   = aw_accepted_q || (w_awvalid && w_awready);
    wire            aw_release  = aw_valid_q && w_last_hs && aw_iss_ok;
    // Forwarded (non-synthesized) narrow B handshake.
    wire            n_b_fwd_hs  = wr_busy && w_bvalid && n_bready;

    // TIMING (2026-08-20).  aw_progress contains (w_wvalid && w_wready), and
    // w_wready is the L2C/xbar WREADY.  Used combinationally below, that put a
    // TWENTY-SECOND abandonment timer (TIMEOUT_CYCLES = 2e9 cycles @100 MHz)
    // on the live write handshake: a post-route trace showed this segment as
    // 3.014 ns of a 9.519 ns path -- 31.7% of it, and the largest single
    // segment of the design's #2 critical path.
    //
    // Register it.  Observing progress one cycle late shifts a 20 s timeout by
    // 10 ns.  Both the CLEAR and the INCREMENT below read the registered copy,
    // deliberately: registering only the clear would let live progress race the
    // exact-equality compare (aw_timeout_fire) and false-trip
    // the watchdog on the boundary cycle.
    reg             aw_progress_q;
    always @(posedge clk) aw_progress_q <= rst ? 1'b0 : aw_progress;

    // Constant 0 when the watchdog is disabled -- this is what lets synthesis
    // strip the counter and everything downstream of it.
    wire aw_timeout_fire = (ENABLE_ABANDON_TIMEOUT != 0) &&
                           (aw_timeout_cnt == TIMEOUT_CYCLES);
    wire aw_timeout_run  = (ENABLE_ABANDON_TIMEOUT != 0);

    always @(posedge clk) begin
        if (rst) begin
            aw_addr_q       <= 32'b0;
            aw_lane_q       <= 2'b00;
            aw_burst_q      <= 1'b0;
            aw_wlen_q       <= 8'd0;
            aw_beats_left_q <= {BEATW{1'b0}};
            aw_valid_q      <= 1'b0;
            aw_accepted_q   <= 1'b0;
            aw_first_strb_q <= 4'b0;
            aw_strb_known_q <= 1'b0;
            wg_d0           <= 32'b0;
            wg_d1           <= 32'b0;
            wg_d2           <= 32'b0;
            wg_d3           <= 32'b0;
            wg_s0           <= 4'b0;
            wg_s1           <= 4'b0;
            wg_s2           <= 4'b0;
            wg_s3           <= 4'b0;
            wg_full_q       <= 1'b0;
            wg_last_pend_q  <= 1'b0;
            wo_d0           <= 32'b0;
            wo_d1           <= 32'b0;
            wo_d2           <= 32'b0;
            wo_d3           <= 32'b0;
            wo_s0           <= 4'b0;
            wo_s1           <= 4'b0;
            wo_s2           <= 4'b0;
            wo_s3           <= 4'b0;
            wo_valid_q      <= 1'b0;
            wo_last_q       <= 1'b0;
            wraw_data_q     <= 32'b0;
            wraw_strb_q     <= 4'b0;
            wraw_valid_q    <= 1'b0;
            aw_timeout_cnt  <= 32'd0;
            aw_drain_q      <= 1'b0;
            aw_drain_cnt_q  <= {(WCNTW+1){1'b0}};
            aw_err_cnt_q    <= {WCNTW{1'b0}};
            aw_wswal_q      <= {BEATW{1'b0}};
            aw_wrx_left_q   <= {BEATW{1'b0}};
            wr_out_q        <= {WCNTW{1'b0}};
            aw_iss_out_q    <= {WCNTW{1'b0}};
        end else begin
            // ── Outstanding-write credit ─────────────────────────────
            // +1 when the master hands over an AW, -1 when it takes the
            // matching B.  Both can happen on the same cycle once more
            // than one write is in flight, hence the combined update.
            // The watchdog's expiry branch below force-zeroes this and
            // therefore has to come after it in the block.
            case ({aw_take, n_b_fwd_hs})
                2'b10:   wr_out_q <= wr_out_q + {{(WCNTW-1){1'b0}}, 1'b1};
                2'b01:   wr_out_q <= wr_out_q - {{(WCNTW-1){1'b0}}, 1'b1};
                default: ;
            endcase
            // Wide AWs issued but not yet answered.  Sizes the
            // abandonment drain: exactly this many late B beats may still
            // arrive for transactions we are about to give up on.
            case ({(w_awvalid && w_awready),
                   (w_bvalid && w_bready && (aw_iss_out_q != {WCNTW{1'b0}}))})
                2'b10:   aw_iss_out_q <= aw_iss_out_q + {{(WCNTW-1){1'b0}}, 1'b1};
                2'b01:   aw_iss_out_q <= aw_iss_out_q - {{(WCNTW-1){1'b0}}, 1'b1};
                default: ;
            endcase

            // ── Narrow AW capture ────────────────────────────────────
            if (aw_take) begin
                aw_addr_q       <= n_awaddr;
                aw_lane_q       <= n_awaddr[3:2];
                aw_burst_q      <= (n_awlen != 8'd0);
                aw_wlen_q       <= aw_wlen_calc;
                aw_beats_left_q <= {1'b0, n_awlen} + {{(BEATW-1){1'b0}}, 1'b1};
                aw_valid_q      <= 1'b1;
                aw_accepted_q   <= 1'b0;
                // Fresh transaction: no stale strobes left in EITHER
                // gather bank.  See "ONE TRANSACTION IN THE GATHER PATH"
                // in the header for the full re-derivation under double
                // buffering; the short form is that n_awready requires
                // !aw_valid_q, aw_valid_q is only cleared by aw_release,
                // aw_release requires the beat carrying WLAST to have
                // been TAKEN, wide beats leave in production order, and
                // WLAST marks the transaction's final beat — so at the
                // earliest cycle this AW can be accepted both banks are
                // empty (wo_valid_q = 0, wg_full_q = 0) and every one of
                // the eight strobe registers has already been zeroed by
                // the departure of the group that last used it.  This
                // zeroing is therefore belt-and-braces, kept because the
                // failure it guards against (two transactions' strobes
                // unioned into one wide beat) is silent.
                wg_s0           <= 4'b0;
                wg_s1           <= 4'b0;
                wg_s2           <= 4'b0;
                wg_s3           <= 4'b0;
                // Beats this transaction still owes us, net of one being
                // handed over in this same cycle (AW+W concurrent — what
                // every real narrow master here does) or already sitting
                // in the raw skid from before AW landed.  Those two are
                // mutually exclusive: n_wready is low while the skid is
                // occupied, so w_take cannot fire then.
                aw_wrx_left_q   <= ({1'b0, n_awlen} + {{(BEATW-1){1'b0}}, 1'b1}) -
                                   ((w_take || wraw_valid_q)
                                        ? {{(BEATW-1){1'b0}}, 1'b1}
                                        : {BEATW{1'b0}});
            end else if (w_take && (aw_wrx_left_q != {BEATW{1'b0}})) begin
                aw_wrx_left_q   <= aw_wrx_left_q - {{(BEATW-1){1'b0}}, 1'b1};
            end

            // ── Narrow W capture ─────────────────────────────────────
            // Either placed straight into the gather buffer, or skidded
            // when the target lane is not yet known.  Beats accepted
            // while aw_wswal_q is draining an ABANDONED transaction's
            // leftovers are discarded outright — they belong to a write
            // that has already been answered with SLVERR, so letting
            // them skid would splice them onto the master's next write.
            if (w_take && !w_place_now && !aw_wswal_busy) begin
                wraw_data_q  <= n_wdata;
                wraw_strb_q  <= n_wstrb;
                wraw_valid_q <= 1'b1;
            end
            if (w_place_skid) wraw_valid_q <= 1'b0;

            if (w_take && !aw_strb_known_q && !aw_wswal_busy) begin
                aw_first_strb_q <= n_wstrb;
                aw_strb_known_q <= 1'b1;
            end

            // ── Discard leftover W beats of an abandoned write ───────
            // n_wlast terminates early in case the master's burst was
            // shorter than the AWLEN it advertised; the count terminates
            // it otherwise.  Either way the channel lands back on a
            // transaction boundary.
            if (aw_wswal_busy && w_take) begin
                if (n_wlast || (aw_wswal_q == {{(BEATW-1){1'b0}}, 1'b1}))
                    aw_wswal_q <= {BEATW{1'b0}};
                else
                    aw_wswal_q <= aw_wswal_q - {{(BEATW-1){1'b0}}, 1'b1};
            end

            // ── Retire the synthesized SLVERR B ──────────────────────
            // Gated on the ACTUAL narrow B handshake, not merely on
            // n_bready: n_bvalid holds the synthesized B back until
            // aw_wswal_q has drained (see the n_bvalid assign), so a
            // bare `aw_err_q && n_bready` retires the response during
            // the W-swallow window without ever having presented it —
            // which is the original "abandoned write is answered with
            // nothing" bug, reintroduced one level down.  Caught by
            // tb_axi_widen_watchdog scenario 9.
            if (aw_err_q && !aw_wswal_busy && n_bready)
                aw_err_cnt_q <= aw_err_cnt_q - {{(WCNTW-1){1'b0}}, 1'b1};

            // ── Placement into the gather bank ──────────────────────
            // Skipped entirely on a bypass: the completing beat goes
            // straight into the output bank below, and this bank is
            // simultaneously reset to "empty" so the NEXT group can start
            // accumulating on the very next cycle.  That is the dead
            // handover cycle, removed.
            if (place_en && !wg_bypass) begin
                if (!cur_burst) begin
                    // Single beat: replicate into every lane exactly as the
                    // pre-burst adapter did (wstrb picks the live lane), so
                    // slaves that key off data placement rather than strobes
                    // see no change at all.
                    wg_d0 <= place_data;
                    wg_d1 <= place_data;
                    wg_d2 <= place_data;
                    wg_d3 <= place_data;
                    case (cur_lane)
                        2'd0: begin wg_s0 <= place_strb; wg_s1 <= 4'b0;
                                    wg_s2 <= 4'b0;       wg_s3 <= 4'b0; end
                        2'd1: begin wg_s0 <= 4'b0;       wg_s1 <= place_strb;
                                    wg_s2 <= 4'b0;       wg_s3 <= 4'b0; end
                        2'd2: begin wg_s0 <= 4'b0;       wg_s1 <= 4'b0;
                                    wg_s2 <= place_strb; wg_s3 <= 4'b0; end
                        2'd3: begin wg_s0 <= 4'b0;       wg_s1 <= 4'b0;
                                    wg_s2 <= 4'b0;       wg_s3 <= place_strb; end
                    endcase
                end else begin
                    case (cur_lane)
                        2'd0: begin wg_d0 <= place_data; wg_s0 <= place_strb; end
                        2'd1: begin wg_d1 <= place_data; wg_s1 <= place_strb; end
                        2'd2: begin wg_d2 <= place_data; wg_s2 <= place_strb; end
                        2'd3: begin wg_d3 <= place_data; wg_s3 <= place_strb; end
                    endcase
                end
            end
            if (place_en) begin
                aw_lane_q       <= cur_lane + 2'd1;
                aw_beats_left_q <= cur_left - {{(BEATW-1){1'b0}}, 1'b1};
            end
            // A completed group the output bank could not take parks here
            // and backpressures the narrow W channel from the next cycle.
            if (wg_park) begin
                wg_full_q      <= 1'b1;
                wg_last_pend_q <= (cur_left == {{(BEATW-1){1'b0}}, 1'b1});
            end

            // ── Load the output bank ────────────────────────────────
            // Two sources, mutually exclusive by construction (wg_bypass
            // requires !wg_full_q via place_en, wg_xfer requires
            // wg_full_q):
            //   wg_bypass — the completing narrow beat merged with the
            //               three lanes already in the gather bank.  The
            //               per-lane select is wo_ld_l0..3.
            //   wg_xfer   — a parked group, moved out whole once the wide
            //               side takes the beat that was blocking it.
            // EVERY one of the eight output-bank strobe nibbles is
            // written on EVERY load, from exactly one group's data, so
            // the output bank can never union two groups' strobes — the
            // hazard `mixed-burst` in tb_axi_widen.cpp pins.
            if (wg_bypass) begin
                wo_d0      <= wo_ld_l0 ? place_data : wg_d0;
                wo_d1      <= wo_ld_l1 ? place_data : wg_d1;
                wo_d2      <= wo_ld_l2 ? place_data : wg_d2;
                wo_d3      <= wo_ld_l3 ? place_data : wg_d3;
                // STROBES ARE NOT REPLICATED.  Data goes into every lane
                // on the single-beat path (historical, and slaves that
                // key off data placement depend on it), but exactly ONE
                // lane may be strobed there — replicating place_strb the
                // way place_data is replicated would write four copies of
                // the word.  A burst takes the gather bank's strobes in
                // the lanes it is not placing; a single beat takes zero,
                // because it has no other lanes.
                wo_s0      <= (cur_lane == 2'd0) ? place_strb
                                                 : (cur_burst ? wg_s0 : 4'b0);
                wo_s1      <= (cur_lane == 2'd1) ? place_strb
                                                 : (cur_burst ? wg_s1 : 4'b0);
                wo_s2      <= (cur_lane == 2'd2) ? place_strb
                                                 : (cur_burst ? wg_s2 : 4'b0);
                wo_s3      <= (cur_lane == 2'd3) ? place_strb
                                                 : (cur_burst ? wg_s3 : 4'b0);
                wo_valid_q <= 1'b1;
                wo_last_q  <= (cur_left == {{(BEATW-1){1'b0}}, 1'b1});
                // The gather bank is free again THIS cycle — the next
                // group's lane 0 lands in it on the next one.
                wg_s0      <= 4'b0;
                wg_s1      <= 4'b0;
                wg_s2      <= 4'b0;
                wg_s3      <= 4'b0;
            end else if (wg_xfer) begin
                wo_d0      <= wg_d0;
                wo_d1      <= wg_d1;
                wo_d2      <= wg_d2;
                wo_d3      <= wg_d3;
                wo_s0      <= wg_s0;
                wo_s1      <= wg_s1;
                wo_s2      <= wg_s2;
                wo_s3      <= wg_s3;
                wo_valid_q <= 1'b1;
                wo_last_q  <= wg_last_pend_q;
                wg_full_q  <= 1'b0;
                wg_s0      <= 4'b0;
                wg_s1      <= 4'b0;
                wg_s2      <= 4'b0;
                wg_s3      <= 4'b0;
            end else if (w_wvalid && w_wready) begin
                wo_valid_q <= 1'b0;
            end

            // ── Wide-side handshakes ─────────────────────────────────
            if (aw_valid_q && w_awvalid && w_awready) aw_accepted_q <= 1'b1;

            // ── Release the gather front end ─────────────────────────
            // Used to be "release on B completion", which is what made
            // this module single-outstanding.  The front end now lets go
            // one full fabric round trip earlier: at the last wide W
            // beat, with the wide AW already issued.  The transaction
            // lives on only as a credit in wr_out_q, which owes it one B.
            //
            // aw_take cannot coincide with this — n_awready requires
            // !aw_valid_q and aw_release requires aw_valid_q — so the
            // ordering against the capture block above does not matter.
            if (aw_release) begin
                aw_valid_q      <= 1'b0;
                aw_accepted_q   <= 1'b0;
                aw_strb_known_q <= 1'b0;
                aw_beats_left_q <= {BEATW{1'b0}};
                aw_wrx_left_q   <= {BEATW{1'b0}};
                // Both banks.  Provably already empty here — aw_release
                // is the handshake that takes the beat carrying WLAST,
                // wide beats leave in production order, and WLAST marks
                // the transaction's final beat, so nothing of this
                // transaction can still be resident and nothing of the
                // next one can have arrived (n_awready is still low).
                // Kept as a blanket clear for the same reason the single
                // -bank version had one: a stale gather bank is silent.
                wo_valid_q      <= 1'b0;
                wg_full_q       <= 1'b0;
            end

            // ── Abandonment watchdog ─────────────────────────────────
            // aw_timeout_cnt free-runs while the module is in EITHER the
            // primary outstanding-transaction phase (aw_valid_q) or the
            // stale-beat-drain phase (aw_drain_q) below; held at 0 the
            // rest of the time.  One counter suffices — the two phases
            // are mutually exclusive in time (aw_drain_q only ever sets
            // in the same cycle aw_valid_q clears).  Any accepted beat
            // on any channel counts as progress and restarts it, so a
            // long healthy burst can never trip the watchdog.
            //
            // MULTI-OUTSTANDING: ONE TIMER OVER THE WHOLE IN-FLIGHT SET,
            // not one per transaction.  Two reasons, and they are not
            // just area:
            //   * The wide side issues every transaction under one
            //     constant ID, so responses cannot overtake each other.
            //     "The oldest transaction is stuck" and "nothing in the
            //     set is progressing" are therefore the same predicate —
            //     a per-transaction timer would have nothing extra to
            //     say, it would just re-derive the same answer N times.
            //   * The failure this defends against is a FABRIC WEDGE
            //     (slave dead, CDC bridge desynced, arbiter grant
            //     starved).  That stalls the whole set at once.  There is
            //     no realistic mode in which one transaction of a
            //     same-ID stream hangs while its siblings retire.
            // The busy predicate widens from "a transaction owns the
            // front end" to "the front end is busy OR some accepted write
            // is still owed its B", which is what actually needs a
            // liveness guarantee now that a write outlives the front end.
            if ((aw_valid_q || wr_busy) && !(w_bvalid && w_bready)) begin
                if (aw_progress_q) begin
                    aw_timeout_cnt <= 32'd0;
                end else if (aw_timeout_fire) begin
                    // Give up on this transaction.  Force-clear every
                    // piece of state tied to it (including the W-side
                    // captures, which would otherwise stay latched once
                    // aw_valid_q drops, permanently blocking n_wready)
                    // and arm the drain latch so a late B beat is
                    // absorbed rather than mis-attributed to the next
                    // write.
                    //
                    // The narrow master is NOT left unanswered: aw_err_q
                    // owes it a BRESP=SLVERR (one B per transaction,
                    // burst or not), and aw_wswal_q owes it acceptance
                    // of whatever W beats it had not handed over yet.
                    // See "ABANDONMENT RETURNS SLVERR" in the header.
                    aw_valid_q      <= 1'b0;
                    aw_accepted_q   <= 1'b0;
                    aw_strb_known_q <= 1'b0;
                    aw_beats_left_q <= {BEATW{1'b0}};
                    aw_wrx_left_q   <= {BEATW{1'b0}};
                    // Both gather banks, for the same reason the single
                    // -bank version cleared one: state tied to the
                    // abandoned transaction that would otherwise stay
                    // latched and block n_wready forever.
                    wo_valid_q      <= 1'b0;
                    wg_full_q       <= 1'b0;
                    wraw_valid_q    <= 1'b0;
                    aw_timeout_cnt  <= 32'd0;
                    aw_drain_q      <= 1'b1;
                    // One SLVERR B per write the master handed over and
                    // has not been answered for.  wr_out_q is >= 1
                    // whenever this branch runs (it is incremented at
                    // aw_take, and the guard above excludes the cycle a
                    // B retires), so the response is never empty.
                    aw_err_cnt_q    <= wr_out_q;
                    wr_out_q        <= {WCNTW{1'b0}};
                    // Late wide B beats still owed to us by the fabric:
                    // exactly the wide AWs issued and unanswered, plus one
                    // handshaking on this very cycle (aw_progress is
                    // registered, so an AW accepted right now does not
                    // stop the expiry — but the slave does have it).
                    aw_drain_cnt_q  <= {1'b0, aw_iss_out_q} +
                                       ((w_awvalid && w_awready)
                                            ? {{WCNTW{1'b0}}, 1'b1}
                                            : {(WCNTW+1){1'b0}});
                    aw_iss_out_q    <= {WCNTW{1'b0}};
                    // Net off a W beat handed over on this very cycle.
                    // With a REGISTERED aw_progress the expiry can coincide
                    // with a live w_take (progress is observed a cycle late),
                    // and aw_wrx_left_q is decremented above by that same
                    // handshake.  Seeding from the pre-handshake value leaves
                    // the swallow counter waiting for a beat the master has
                    // already sent; n_bvalid is gated on it draining, so the
                    // SLVERR B is held back for one further TIMEOUT_CYCLES.
                    aw_wswal_q      <= aw_wrx_left_q -
                                       ((w_take && (aw_wrx_left_q != {BEATW{1'b0}}))
                                            ? {{(BEATW-1){1'b0}}, 1'b1}
                                            : {BEATW{1'b0}});
                    // synthesis translate_off
                    $display("axi_narrow_to_wide [%0t]: %0d write(s) in flight (front end at 0x%08h) abandoned after %0d idle cycles; answering the narrow master with %0d BRESP=SLVERR response(s) (%0d W beat(s) to discard, %0d late wide B beat(s) to absorb)",
                             $time, wr_out_q, aw_addr_q, TIMEOUT_CYCLES,
                             wr_out_q, aw_wrx_left_q, aw_iss_out_q);
                    // synthesis translate_on
                end else begin
                    if (aw_timeout_run) aw_timeout_cnt <= aw_timeout_cnt + 32'd1;
                end
            end else if (aw_drain_q || aw_wswal_busy) begin
                // Recovery phase.  Two independent obligations run here:
                //   * aw_drain_q — sink exactly one late wide-side B beat
                //     (n_bvalid never sources from w_bvalid while
                //     aw_valid_q is 0, so this can never surface a stale
                //     response to the narrow master).
                //   * aw_wswal_q — accept-and-discard the abandoned
                //     write's outstanding narrow W beats, decremented in
                //     the W section above.
                // Both are bounded by one further TIMEOUT_CYCLES of no
                // progress, after which recovery stops waiting: a wide
                // side silent for two full windows is not going to answer
                // a stale request, and a master that owes W beats it
                // never sends is already violating AXI.  aw_err_q is
                // deliberately NOT force-cleared here — the master must
                // get its response, and BVALID must hold until BREADY.
                // Absorb one late wide B per abandoned, ISSUED write.
                // aw_drain_cnt_q may legitimately be 0 (nothing had been
                // issued yet when the watchdog fired); the flag then just
                // holds *READY low until the drain timeout, exactly as it
                // did when the drain was a bare one-shot.
                if (w_bvalid && aw_drain_q) begin
                    if (aw_drain_cnt_q <= {{WCNTW{1'b0}}, 1'b1}) begin
                        aw_drain_q     <= 1'b0;
                        aw_drain_cnt_q <= {(WCNTW+1){1'b0}};
                    end else begin
                        aw_drain_cnt_q <= aw_drain_cnt_q -
                                          {{WCNTW{1'b0}}, 1'b1};
                    end
                end
                if (aw_timeout_fire) begin
                    aw_drain_q     <= 1'b0;
                    aw_drain_cnt_q <= {(WCNTW+1){1'b0}};
                    aw_wswal_q     <= {BEATW{1'b0}};
                    aw_timeout_cnt <= 32'd0;
                end else begin
                    if (aw_timeout_run) aw_timeout_cnt <= aw_timeout_cnt + 32'd1;
                end
            end else begin
                aw_timeout_cnt <= 32'd0;
            end
        end
    end

    // Narrow AW ready: accept only when free, not draining a
    // previously-abandoned transaction's stale response, and not still
    // owing that transaction its synthesized SLVERR B / W-beat discard.
    // The last two terms are what stop the error response being
    // mis-attributed to a NEW write accepted in the meantime.
    // wr_full additionally caps how many writes may be in flight at once.
    // At WR_OUTSTANDING = 1 this reduces to the historical behaviour
    // exactly: the credit is only returned by B, so no second AW is
    // accepted until the first write has fully completed.
    assign n_awready = !aw_valid_q && !wr_full &&
                       !aw_drain_q && !aw_err_q && !aw_wswal_busy;

    // Narrow W ready: one narrow beat in flight at a time (either sitting
    // in the raw skid or not yet gathered), and never more beats than the
    // transaction's AWLEN.  When idle (no AW yet) we stay ready — a
    // master is entitled to raise WVALID first, and the abandonment
    // watchdog relies on this coming back after recovery.
    // While aw_wswal_q is non-zero we are unconditionally ready: those
    // beats belong to an already-answered, abandoned write and are
    // accepted only to be thrown away (see the W section above).
    //
    // DOUBLE-BUFFERING SHOWS UP HERE AND ONLY HERE on the ready path:
    // the term used to be !wg_valid_q ("no assembled wide beat is
    // pending"), which went low for the whole handover cycle.  It is now
    // !wg_full_q ("the gather bank is not parked"), which is set ONLY
    // when the wide side backpressured.  Still a plain register — no
    // w_wready, no w_awready, nothing combinational from the wide side
    // reaches this signal (c2264764).
    assign n_wready = aw_wswal_busy ? 1'b1
                                    : (!wraw_valid_q && !wg_full_q && !aw_drain_q &&
                                       (!aw_valid_q || (aw_beats_left_q != {BEATW{1'b0}})));

    // ── Derive AXI-legal awsize + awaddr from the narrow wstrb ────────
    //
    // Single-beat path only; bursts use WIDE_SIZE at a 16-B-aligned
    // address.  `aw_first_strb_q` is latched on the first narrow W
    // handshake; the handshake gate below forces AW to wait for it.
    wire [3:0] aw_strb      = aw_first_strb_q;
    wire       strb_size0   = (aw_strb == 4'b0001) || (aw_strb == 4'b0010) ||
                              (aw_strb == 4'b0100) || (aw_strb == 4'b1000);
    wire       strb_size1   = (aw_strb == 4'b0011) || (aw_strb == 4'b0110) ||
                              (aw_strb == 4'b1100);
    wire [2:0] aw_size_der  = strb_size0 ? 3'd0 :
                              strb_size1 ? 3'd1 :
                                           3'd2;
    // AXI requires awaddr aligned to awsize.  For awsize=0 the byte
    // address is already legal; for awsize=1 clear bit 0; for awsize=2
    // clear bits [1:0].  Upper bits (beat index) are untouched, so the
    // slave still dispatches to the correct 128-bit beat.
    wire [31:0] aw_addr_aligned = (aw_size_der == 3'd0) ?  aw_addr_q :
                                  (aw_size_der == 3'd1) ? {aw_addr_q[31:1], 1'b0} :
                                                          {aw_addr_q[31:2], 2'b00};

    // Wide AW.  Single-beat: drive once we have captured narrow AW *and*
    // the narrow W wstrb (needed to size it).  All current narrow masters
    // (CPU dcache bypass, boot_fsm, jtag) assert AW+W concurrently, so
    // this extra gate does not add latency.  Burst: no wstrb dependency —
    // awsize is fixed at a full 16-byte beat — so AW goes out immediately
    // and the W beats stream behind it.
    assign w_awid    = ID_TAG;
    assign w_awaddr  = aw_burst_q ? {aw_addr_q[31:4], 4'b0000} : aw_addr_aligned;
    assign w_awlen   = aw_burst_q ? aw_wlen_q : 8'd0;
    assign w_awsize  = aw_burst_q ? WIDE_SIZE : aw_size_der;
    assign w_awburst = 2'b01;                  // INCR
    assign w_awvalid = aw_valid_q && !aw_accepted_q &&
                       (aw_burst_q || aw_strb_known_q);

    // Wide W channel — straight off the OUTPUT bank's flops, exactly as
    // it came straight off the single bank's flops before.  Double
    // buffering added a flop stage in front of these, not a mux behind
    // them: the wide-side W output path is unchanged.
    assign w_wdata  = {wo_d3, wo_d2, wo_d1, wo_d0};
    assign w_wstrb  = {wo_s3, wo_s2, wo_s1, wo_s0};
    assign w_wlast  = wo_last_q;
    assign w_wvalid = wo_valid_q && aw_valid_q;

    // B channel: forward back to narrow.  During aw_drain_q, also accept
    // (and silently discard) a late B beat orphaned by the abandonment
    // watchdog above — n_bvalid stays gated on aw_valid_q, which is
    // already 0 whenever aw_drain_q is 1, so this can never surface a
    // stale response to the narrow master.  One B per transaction, burst
    // or not, exactly as AXI4 specifies.
    //
    // aw_err_q additionally sources a LOCALLY synthesized B for a
    // transaction the watchdog abandoned.  It is mutually exclusive with
    // the forwarded B: the expiry that sets aw_err_q clears aw_valid_q in
    // the same cycle, and n_awready stays low until aw_err_q retires, so
    // aw_valid_q cannot come back underneath it.
    // wr_busy, not aw_valid_q: a write that has left the gather front end
    // is still owed a B, and that is precisely the case this module used
    // to have no state for.
    assign w_bready = (wr_busy && n_bready) || aw_drain_q;
    //
    // The synthesized B waits for aw_wswal_q to drain so BVALID still
    // follows acceptance of the burst's last W beat, as AXI4 requires.
    // That wait cannot deadlock: the recovery phase in the always block
    // force-clears aw_wswal_q after one further TIMEOUT_CYCLES, so a
    // master that never sends the beats it owes still gets its B.
    // aw_wswal_q only ever counts down while aw_err_q is set (n_awready
    // is low throughout), so n_bvalid rises once and then holds until
    // n_bready — it never glitches back to 0.
    assign n_bvalid = (wr_busy && w_bvalid) || (aw_err_q && !aw_wswal_busy);
    assign n_bresp  = aw_err_q ? RESP_SLVERR : w_bresp;

    // ═══════════════════════════════════════════════════════════════════
    // Read path — capture AR, drive wide AR, dribble each 128-bit beat
    // back out as up to four 32-bit narrow beats.
    // ═══════════════════════════════════════════════════════════════════
    // Response-engine working registers.  These hold the ONE context
    // currently being dribbled out; every other in-flight read lives in
    // the arq_* FIFO below until the engine gets to it.
    reg [31:0]      ar_addr_q;         // diagnostics only (watchdog $display)
    reg [1:0]       ar_lane_q;
    reg [BEATW-1:0] ar_beats_left_q;   // narrow R beats still to deliver
    reg             ar_valid_q;

    // Registered wide read beat, sliced lane-by-lane to the narrow side.
    reg [31:0]      rg_d0, rg_d1, rg_d2, rg_d3;
    reg [1:0]       rg_resp_q;
    reg             rg_valid_q;

    // Read-abandonment watchdog state — mirrors the write side above
    // (see module header comment).  Without this, a wide-side stall
    // permanently latches ar_valid_q at 1, which is exactly the "slave
    // stuck asserting ARREADY=0 forever" failure shape (n_arready is
    // literally `!ar_valid_q`) — indistinguishable from the far end
    // being genuinely dead, from the narrow master's point of view.
    reg [31:0]      ar_timeout_cnt;
    reg             ar_drain_q;   // absorb stale wide R beats; ARREADY low
    // One bit wider than ar_iss_out_q, for the same DEPTH = 1 reason
    // spelled out on aw_drain_cnt_q above.
    reg [RCNTW:0]   ar_drain_cnt_q;  // ...for this many outstanding reads

    // Abandonment error-response state (see "ABANDONMENT RETURNS SLVERR"
    // in the header).  ar_err_left_q is the number of narrow R beats the
    // abandoned read still owes the master — seeded from ar_beats_left_q,
    // which starts at ARLEN+1 and has already been decremented by every
    // beat delivered before the watchdog fired, so beats already returned
    // are never re-sent and beats never sent are never dropped.  A
    // single SLVERR beat on a multi-beat read would leave the master
    // waiting for the rest: one hang traded for another.
    //
    // MULTI-OUTSTANDING: ar_err_left_q counts the beats of ONE context.
    // ar_err_mode_q says the whole in-flight set is being answered with
    // SLVERR, and the FIFO walk below reloads ar_err_left_q per queued
    // context so every abandoned read gets its own beat count and its own
    // RLAST.  That is what keeps the documented "a single SLVERR beat on
    // a multi-beat read strands the master" hazard from getting WORSE
    // with several reads queued: the error path replays the same
    // per-transaction sequencing the normal path uses, it just sources
    // SLVERR/zero instead of waiting for wide R data.
    reg [BEATW-1:0] ar_err_left_q;
    reg             ar_err_mode_q;
    wire ar_err_active = (ar_err_left_q != {BEATW{1'b0}});

    // ── AR context FIFO (multi-outstanding reads) ─────────────────────
    // One entry per accepted narrow AR, holding only what the two
    // consumers need: the wide-AR issuer wants addr/len/size, the
    // response engine wants the lane (addr[3:2]) and the beat count
    // (len+1).  Everything else is derived.
    //
    // Three pointers walk it, all (RPTRW+1) bits wide so the extra MSB
    // separates "empty" from "wrapped":
    //   ar_wp — push, on a narrow AR handshake
    //   ar_ip — wide-AR issue.  Runs AHEAD of the response engine: this
    //           is the whole point, read N+1's AR crosses the fabric
    //           while read N's data is still coming back.
    //   ar_rp — response-engine load.  An entry is dead the moment the
    //           engine has copied its lane/beat count into the working
    //           registers, so advancing at LOAD (not at completion) is
    //           correct and keeps the FIFO one entry smaller.
    // Backpressure is NOT the pointer difference but ar_out_q, the count
    // of narrow reads accepted and not yet fully delivered — that is the
    // number the master cares about and it bounds the pointer spread too.
    reg [31:0]      arq_addr [0:RQ_N-1];
    reg [7:0]       arq_len  [0:RQ_N-1];
    reg [2:0]       arq_size [0:RQ_N-1];
    reg [RPTRW:0]   ar_wp, ar_ip, ar_rp;
    reg [RCNTW-1:0] ar_out_q;
    reg [RCNTW-1:0] ar_iss_out_q;   // wide ARs issued, RLAST not yet seen

    wire            ar_full     = (ar_out_q == RD_OUTSTANDING[RCNTW-1:0]);
    wire            ar_ld_avail = (ar_wp != ar_rp);
    wire            ar_iss_avail= (ar_wp != ar_ip);
    wire [RPTRW-1:0] ar_wp_idx  = ar_wp[RPTRW-1:0];
    wire [RPTRW-1:0] ar_ip_idx  = ar_ip[RPTRW-1:0];
    wire [RPTRW-1:0] ar_rp_idx  = ar_rp[RPTRW-1:0];

    wire ar_take      = n_arready && n_arvalid;
    // Historical name, kept so tb/tb_n2w_watchdog.v's hierarchical taps
    // still resolve.  With the issue pointer decoupled from the response
    // engine this reads "the wide AR for the context the engine is
    // working on has gone out", i.e. the issue pointer has moved past
    // that context's slot (which is the one just below ar_rp).
    wire [RPTRW:0] ar_rsp_slot = ar_rp - {{RPTRW{1'b0}}, 1'b1};
    /* verilator lint_off UNUSEDSIGNAL */
    // Nothing inside the module reads this — it exists purely as a
    // hierarchical tap for tb/tb_n2w_watchdog.v, which pinned the old
    // register of the same name.
    wire ar_accepted_q = ar_valid_q && (ar_ip != ar_rsp_slot);
    /* verilator lint_on UNUSEDSIGNAL */

    wire r_wide_take  = ar_valid_q && !rg_valid_q && w_rvalid && w_rready;
    // Narrow R handshake on the NORMAL (forwarded-data) path only.  The
    // synthesized-error path has its own take (ar_err_take) so the two
    // never share a beat counter — ar_valid_q is 0 for the whole error
    // phase, so this is naturally 0 there.
    wire n_r_take     = ar_valid_q && rg_valid_q && n_rready;
    wire ar_err_take  = ar_err_active && n_rready;
    wire r_group_end  = (ar_lane_q == 2'd3) ||
                        (ar_beats_left_q == {{(BEATW-1){1'b0}}, 1'b1});
    wire ar_last_beat = (ar_beats_left_q == {{(BEATW-1){1'b0}}, 1'b1});
    wire ar_progress  = (w_arvalid && w_arready) || (w_rvalid && w_rready)
                     || n_r_take;

    // ── Response-engine context load control ─────────────────────────
    // Declared here rather than up with ar_take because it reads
    // n_r_take / ar_last_beat: Verilog-2005 wants a net declared before
    // it is referenced, and Verilator's leniency about that is not a
    // reason to hand Vivado a forward reference.
    //
    // "The engine is free" INCLUDES the cycle its current context retires
    // its final beat, so back-to-back reads chain with no dead cycle
    // between them.  The always block relies on this: the load is written
    // after the lane handoff so that when the two coincide, the load wins.
    wire ar_eng_free  = !ar_valid_q || (n_r_take && ar_last_beat);
    // Empty-FIFO bypass: load the response engine straight from the
    // narrow AR channel so a lone read keeps EXACTLY the latency it had
    // when this module was single-outstanding.  Without it every read
    // would pay one extra cycle going through the array, which on a CPU
    // load is a cycle of pure added load-use latency for no benefit.
    wire ar_ld_bypass = ar_take && !ar_ld_avail && ar_eng_free &&
                        !ar_err_mode_q && !ar_drain_q;
    wire ar_ld_fifo   = ar_eng_free && !ar_err_mode_q && !ar_drain_q &&
                        ar_ld_avail;
    wire ar_ld_any    = ar_ld_bypass || ar_ld_fifo;

    // Same treatment as aw_progress_q above, and for the same reason: keep the
    // 20-second abandonment timer off the live read handshake.  Clear and
    // increment both read the registered copy.
    reg  ar_progress_q;
    always @(posedge clk) ar_progress_q <= rst ? 1'b0 : ar_progress;

    wire ar_timeout_fire = (ENABLE_ABANDON_TIMEOUT != 0) &&
                           (ar_timeout_cnt == TIMEOUT_CYCLES);
    wire ar_timeout_run  = (ENABLE_ABANDON_TIMEOUT != 0);

    integer ri;
    always @(posedge clk) begin
        if (rst) begin
            ar_addr_q       <= 32'b0;
            ar_lane_q       <= 2'b0;
            ar_beats_left_q <= {BEATW{1'b0}};
            ar_valid_q      <= 1'b0;
            rg_d0           <= 32'b0;
            rg_d1           <= 32'b0;
            rg_d2           <= 32'b0;
            rg_d3           <= 32'b0;
            rg_resp_q       <= 2'b0;
            rg_valid_q      <= 1'b0;
            ar_timeout_cnt  <= 32'd0;
            ar_drain_q      <= 1'b0;
            ar_drain_cnt_q  <= {(RCNTW+1){1'b0}};
            ar_err_left_q   <= {BEATW{1'b0}};
            ar_err_mode_q   <= 1'b0;
            ar_wp           <= {(RPTRW+1){1'b0}};
            ar_ip           <= {(RPTRW+1){1'b0}};
            ar_rp           <= {(RPTRW+1){1'b0}};
            ar_out_q        <= {RCNTW{1'b0}};
            ar_iss_out_q    <= {RCNTW{1'b0}};
            for (ri = 0; ri < RQ_N; ri = ri + 1) begin
                arq_addr[ri] <= 32'b0;
                arq_len[ri]  <= 8'd0;
                arq_size[ri] <= 3'd2;
            end
        end else begin
            // ── Outstanding-read credit ──────────────────────────────
            // +1 on a narrow AR handshake, -1 when the last narrow beat
            // of a read is delivered — on either the normal path or the
            // synthesized-SLVERR one, since both complete a transaction
            // from the master's point of view.
            case ({ar_take,
                   ((n_r_take && ar_last_beat) ||
                    (ar_err_take && (ar_err_left_q ==
                                     {{(BEATW-1){1'b0}}, 1'b1})))})
                2'b10:   ar_out_q <= ar_out_q + {{(RCNTW-1){1'b0}}, 1'b1};
                2'b01:   ar_out_q <= ar_out_q - {{(RCNTW-1){1'b0}}, 1'b1};
                default: ;
            endcase
            // Wide ARs issued whose RLAST has not come back — sizes the
            // abandonment drain, same role as aw_iss_out_q.
            case ({(w_arvalid && w_arready),
                   (w_rvalid && w_rready && w_rlast &&
                    (ar_iss_out_q != {RCNTW{1'b0}}))})
                2'b10:   ar_iss_out_q <= ar_iss_out_q + {{(RCNTW-1){1'b0}}, 1'b1};
                2'b01:   ar_iss_out_q <= ar_iss_out_q - {{(RCNTW-1){1'b0}}, 1'b1};
                default: ;
            endcase

            // Retire one synthesized SLVERR beat.
            if (ar_err_take)
                ar_err_left_q <= ar_err_left_q - {{(BEATW-1){1'b0}}, 1'b1};

            // ── Error-mode FIFO walk ─────────────────────────────────
            // The abandoned set is answered ONE TRANSACTION AT A TIME,
            // reusing the FIFO's own ARLEN per entry, so every abandoned
            // read gets exactly its own beat count and its own RLAST.
            // Collapsing the set into one long SLVERR burst would strand
            // the master on every read but the first — the same failure
            // the single-beat-for-a-multi-beat-read hazard describes, one
            // level up.  Runs a cycle after ar_err_left_q hits zero,
            // which is free: this is the recovery path.
            if (ar_err_mode_q && (ar_err_left_q == {BEATW{1'b0}})) begin
                if (ar_ld_avail) begin
                    ar_err_left_q <= {1'b0, arq_len[ar_rp_idx]} +
                                     {{(BEATW-1){1'b0}}, 1'b1};
                    ar_rp         <= ar_rp + {{RPTRW{1'b0}}, 1'b1};
                end else begin
                    ar_err_mode_q <= 1'b0;
                end
            end

            // ── Push the accepted AR into the context FIFO ───────────
            // Unconditional, even on the bypass path: the wide-AR issuer
            // reads the array, so the entry has to exist for it even when
            // the response engine took a shortcut around it.
            if (ar_take) begin
                arq_addr[ar_wp_idx] <= n_araddr;
                arq_len [ar_wp_idx] <= n_arlen;
                arq_size[ar_wp_idx] <= n_arsize;
                ar_wp <= ar_wp + {{RPTRW{1'b0}}, 1'b1};
            end

            // ── Wide AR issue ────────────────────────────────────────
            // Decoupled from the response engine: this is what lets read
            // N+1's address cross the fabric while read N's data is still
            // arriving.
            if (w_arvalid && w_arready)
                ar_ip <= ar_ip + {{RPTRW{1'b0}}, 1'b1};

            // Capture one wide beat.
            if (r_wide_take) begin
                rg_d0      <= w_rdata[31:0];
                rg_d1      <= w_rdata[63:32];
                rg_d2      <= w_rdata[95:64];
                rg_d3      <= w_rdata[127:96];
                rg_resp_q  <= w_rresp;
                rg_valid_q <= 1'b1;
            end

            // Hand one 32-bit lane to the narrow master.
            if (n_r_take) begin
                ar_lane_q       <= ar_lane_q + 2'd1;
                ar_beats_left_q <= ar_beats_left_q - {{(BEATW-1){1'b0}}, 1'b1};
                if (r_group_end) rg_valid_q <= 1'b0;
                if (ar_last_beat) ar_valid_q <= 1'b0;
            end

            // ── Response-engine context load ─────────────────────────
            // Deliberately placed AFTER the lane handoff above so that a
            // load landing on the same cycle as the previous context's
            // final beat WINS — that is what makes back-to-back reads
            // cost zero turnaround cycles instead of one.  The load
            // condition explicitly includes that completing handshake
            // (see ar_ld_fifo / ar_ld_bypass), so this is not an
            // accidental race: the two are meant to coincide.
            if (ar_ld_any) begin
                ar_addr_q       <= ar_ld_bypass ? n_araddr : arq_addr[ar_rp_idx];
                ar_lane_q       <= ar_ld_bypass ? n_araddr[3:2]
                                                : arq_addr[ar_rp_idx][3:2];
                ar_beats_left_q <= (ar_ld_bypass ? {1'b0, n_arlen}
                                                 : {1'b0, arq_len[ar_rp_idx]}) +
                                   {{(BEATW-1){1'b0}}, 1'b1};
                ar_valid_q      <= 1'b1;
                rg_valid_q      <= 1'b0;
                ar_rp           <= ar_rp + {{RPTRW{1'b0}}, 1'b1};
            end

            // ── Abandonment watchdog (read side) ──────────────────────
            // Same single-counter, two-phase (outstanding / drain)
            // structure as the write side; see the comment there.
            // Busy predicate widened for multi-outstanding, same
            // single-timer-over-the-whole-set rationale as the write side
            // above: "the engine is working" OR "some accepted read is
            // still owed data".
            if ((ar_valid_q || (ar_out_q != {RCNTW{1'b0}})) &&
                !(n_r_take && ar_last_beat)) begin
                if (ar_progress_q) begin
                    ar_timeout_cnt <= 32'd0;
                end else if (ar_timeout_fire) begin
                    // Abandon — but owe the narrow master every beat of
                    // the burst it has not yet received, each SLVERR,
                    // RLAST on the last.  ar_beats_left_q is >= 1 for as
                    // long as ar_valid_q is 1 (it is only cleared by the
                    // final-beat handshake, which also clears
                    // ar_valid_q), so ar_err_left_q is always seeded
                    // non-zero here and the response is never empty.
                    ar_valid_q     <= 1'b0;
                    rg_valid_q     <= 1'b0;
                    ar_timeout_cnt <= 32'd0;
                    ar_drain_q     <= 1'b1;
                    // Late wide R bursts still owed to us: one RLAST per
                    // wide AR issued and unanswered, plus one handshaking
                    // on this very cycle (ar_progress is registered, so a
                    // live AR handshake does not stop the expiry).
                    ar_drain_cnt_q <= {1'b0, ar_iss_out_q} +
                                      ((w_arvalid && w_arready)
                                           ? {{RCNTW{1'b0}}, 1'b1}
                                           : {(RCNTW+1){1'b0}});
                    ar_iss_out_q   <= {RCNTW{1'b0}};
                    // Every read the master handed over is now answered
                    // locally with SLVERR, so nothing queued may still go
                    // out on the wide side.  Parking the issue pointer at
                    // the write pointer retires the un-issued ones without
                    // losing their FIFO entries: the error walk below still
                    // needs each one's ARLEN to get its RLAST right.
                    ar_ip          <= ar_wp;
                    ar_err_mode_q  <= 1'b1;
                    // Net off a beat delivered on this very cycle — see the
                    // matching comment on aw_wswal_q.  n_r_take decrements
                    // ar_beats_left_q above, and with a registered ar_progress
                    // that handshake can land on the expiry cycle; seeding
                    // from the pre-handshake value owes the master one beat
                    // too many, i.e. ARLEN+2 beats for one read.  Safe: the
                    // guard above excludes (n_r_take && ar_last_beat), so
                    // ar_beats_left_q is >= 2 whenever n_r_take is 1 here.
                    // When ar_valid_q is 0 (reads queued but the engine
                    // has not loaded one yet) there is no current context
                    // to seed from; leave the count at 0 and let the error
                    // walk below pull the first one out of the FIFO.
                    ar_err_left_q  <= ar_valid_q
                                      ? (ar_beats_left_q -
                                         (n_r_take ? {{(BEATW-1){1'b0}}, 1'b1}
                                                   : {BEATW{1'b0}}))
                                      : {BEATW{1'b0}};
                    // synthesis translate_off
                    $display("axi_narrow_to_wide [%0t]: %0d read(s) in flight (front end at 0x%08h) abandoned after %0d idle cycles; answering the narrow master with RRESP=SLVERR beats, RLAST at each transaction's own boundary (%0d beat(s) left on the current one)",
                             $time, ar_out_q, ar_addr_q, TIMEOUT_CYCLES,
                             ar_beats_left_q);
                    // synthesis translate_on
                end else begin
                    if (ar_timeout_run) ar_timeout_cnt <= ar_timeout_cnt + 32'd1;
                end
            end else if (ar_drain_q) begin
                // Absorb the abandoned transactions' stale responses.  A
                // burst's response is more than one beat, so drain until
                // RLAST rather than after a fixed single beat — and with
                // several reads abandoned at once, until the LAST of them.
                // ar_drain_cnt_q may legitimately be 0 (nothing had been
                // issued when the watchdog fired); the flag then behaves
                // exactly like the historical one-shot and is cleared by
                // the first stray RLAST or by the drain timeout.
                if (w_rvalid && w_rlast) begin
                    if (ar_drain_cnt_q <= {{RCNTW{1'b0}}, 1'b1}) begin
                        ar_drain_q     <= 1'b0;
                        ar_drain_cnt_q <= {(RCNTW+1){1'b0}};
                        ar_timeout_cnt <= 32'd0;
                    end else begin
                        ar_drain_cnt_q <= ar_drain_cnt_q -
                                          {{RCNTW{1'b0}}, 1'b1};
                        ar_timeout_cnt <= 32'd0;
                    end
                end else if (ar_timeout_fire) begin
                    ar_drain_q     <= 1'b0;
                    ar_drain_cnt_q <= {(RCNTW+1){1'b0}};
                    ar_timeout_cnt <= 32'd0;
                end else begin
                    if (ar_timeout_run) ar_timeout_cnt <= ar_timeout_cnt + 32'd1;
                end
            end else begin
                ar_timeout_cnt <= 32'd0;
            end
        end
    end

    // Narrow AR ready: accept only when free, not draining a
    // previously-abandoned transaction's stale response, and not still
    // owing that transaction its synthesized SLVERR beats — otherwise
    // those beats would be delivered as (part of) the NEXT read's data.
    // ar_full (not !ar_valid_q) is the steady-state gate now: several
    // reads may be in flight, and the credit is only returned when a read
    // has delivered its final narrow beat.  At RD_OUTSTANDING = 1 this
    // reduces to the historical behaviour exactly.  ar_err_mode_q (not
    // ar_err_active) holds AR off for the WHOLE abandoned set, including
    // the gap cycles between one context's last SLVERR beat and the next
    // context being pulled out of the FIFO.
    assign n_arready = !ar_full && !ar_drain_q && !ar_err_mode_q;

    // ── Derive AXI-legal arsize + araddr from the narrow n_arsize ─────
    //
    // Mirrors aw_size_der/aw_addr_aligned above.  arsize=0 (byte) keeps
    // the byte-granular address as-is (already AXI-legal); arsize=1
    // (half-word) clears bit 0; arsize=2 (word/long, the common case —
    // naturally-aligned LONG loads) clears bits[1:0].  Byte/word-sized
    // bypass loads (SCC/VIA/IWM/SCSI/ADB register reads) reach the
    // peripheral at their true byte address instead of being silently
    // rounded down to the containing 32-bit word — that rounding was
    // harmless for DRAM (word-aligned reads + narrow slice is a safe,
    // common pattern) but wrong for byte-addressed, side-effecting I/O
    // register decode.
    // Sourced from the FIFO entry the ISSUE pointer is on, not from the
    // response engine's working registers — those two are decoupled now,
    // and it is the issue pointer running ahead that hides the AR->first-R
    // latency of the read behind it.
    wire [31:0] ari_addr  = arq_addr[ar_ip_idx];
    wire [7:0]  ari_len   = arq_len [ar_ip_idx];
    wire [2:0]  ari_size  = arq_size[ar_ip_idx];
    wire        ari_burst = (ari_len != 8'd0);
    wire [9:0]  ari_wlen_sum = {2'b00, ari_len} + {8'b0, ari_addr[3:2]};
    wire [7:0]  ari_wlen     = ari_wlen_sum[9:2];

    wire        ar_size0 = (ari_size == 3'd0);
    wire        ar_size1 = (ari_size == 3'd1);
    wire [31:0] ar_addr_aligned = ar_size0 ? ari_addr :
                                  ar_size1 ? {ari_addr[31:1], 1'b0} :
                                             {ari_addr[31:2], 2'b00};

    assign w_arid    = ID_TAG;
    assign w_araddr  = ari_burst ? {ari_addr[31:4], 4'b0000} : ar_addr_aligned;
    assign w_arlen   = ari_burst ? ari_wlen : 8'd0;
    assign w_arsize  = ari_burst ? WIDE_SIZE : ari_size;
    assign w_arburst = 2'b01;
    // Issue whenever the FIFO holds an un-issued context.  No dependency
    // on the response engine at all: that decoupling IS the read-side
    // pipelining.  ar_ip is parked at ar_wp on a watchdog expiry, which
    // is what stops abandoned reads from still going out on the wide side.
    assign w_arvalid = ar_iss_avail;

    // During ar_drain_q, also accept (and silently discard) late R beats
    // orphaned by the abandonment watchdog above — n_rvalid stays gated
    // on ar_valid_q, which is already 0 whenever ar_drain_q is 1, so a
    // stale beat can never be mis-delivered as the response to whatever
    // NEW read this instance has since accepted.
    assign w_rready  = (ar_valid_q && !rg_valid_q) || ar_drain_q;
    // The synthesized-error path (ar_err_active) is mutually exclusive
    // with the forwarded path: the expiry that arms it clears ar_valid_q
    // in the same cycle, and n_arready stays low until the last error
    // beat retires, so ar_valid_q cannot come back underneath it.
    assign n_rvalid  = (ar_valid_q && rg_valid_q) || ar_err_active;
    assign n_rresp   = ar_err_active ? RESP_SLVERR : rg_resp_q;
    assign n_rlast   = ar_err_active
                       ? (ar_err_left_q == {{(BEATW-1){1'b0}}, 1'b1})
                       : ar_last_beat;
    // Slice the 32-bit word from the registered 128-bit beat.  AXI leaves
    // RDATA don't-care when RRESP is an error; 0 is used rather than the
    // stale rg_* contents so a master that ignores RRESP cannot mistake
    // a leftover beat of the abandoned burst for real data.
    assign n_rdata   = ar_err_active         ? 32'h0000_0000 :
                       (ar_lane_q == 2'd0) ? rg_d0 :
                       (ar_lane_q == 2'd1) ? rg_d1 :
                       (ar_lane_q == 2'd2) ? rg_d2 :
                                             rg_d3;

    // Lint sinks for narrow-side fields this adapter deliberately does
    // not forward (PROT is dropped by the wide-side ID/mapping model;
    // BID/RID are unused because there is one outstanding transaction).
    /* verilator lint_off UNUSEDSIGNAL */
    // (`*_wlen_sum[1:0]` are the sub-wide-beat remainder of the lane+len
    // span; only bits [9:2] — the wide-beat count — are meaningful.)
    wire _unused_n2w = &{1'b0, n_awprot, n_arprot, w_bid, w_rid,
                         aw_wlen_sum[1:0], ari_wlen_sum[1:0]};
    /* verilator lint_on UNUSEDSIGNAL */

    // synthesis translate_off
    // The narrow master must mark exactly the last beat of its burst.
    always @(posedge clk) begin
        if (!rst && n_wvalid && n_wready && aw_ctx_ok) begin
            if (n_wlast && (cur_left != {{(BEATW-1){1'b0}}, 1'b1}))
                $error("axi_narrow_to_wide: n_wlast asserted with %0d narrow beats left",
                       cur_left);
            if (!n_wlast && (cur_left == {{(BEATW-1){1'b0}}, 1'b1}))
                $error("axi_narrow_to_wide: final narrow W beat without n_wlast");
        end
    end
    // synthesis translate_on

endmodule

`default_nettype wire
