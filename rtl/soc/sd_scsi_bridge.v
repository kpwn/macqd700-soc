// sd_scsi_bridge.v — pb_clk ↔ core_clk CDC for the SCSI sd_ctrl path.
//
// Role
// ════
// scsi.v lives on pb_clk (50 MHz) and exposes a sd_ctrl-shaped backing-
// store interface (sd_cmd_type/lba/block_count/sd_go out; sd_busy/done/
// error/rd_valid/rd_data/wr_ready in; sd_wr_data out).  The actual
// sd_ctrl + sd_spi run on core_clk (100 or 200 MHz) so they can hit the
// 50 MHz SD-spec SPI ceiling.  This module is the only CDC on the SD
// path — it presents the same sd_ctrl-shaped interface on the pb side
// and drives a real sd_ctrl on the core side.
//
// Why one bridge, not async-FIFOs everywhere
// ═══════════════════════════════════════════
// The SPI byte rate is ~3 MB/s — a byte every ~320 ns.  At pb_clk that
// is ~16 cycles, at core_clk it is ~32-64 cycles.  Both clocks are >>
// 10× faster than the data rate, so a single-element toggle handshake
// per byte is ample — no async FIFO depth is required.  Levels are
// 2-FF synchronised; pulses use req/ack toggles; latched request fields
// (cmd_type, lba, block_count) are stable across the entire transaction
// so the receiving domain samples them after the go-edge has propagated.
//
// CDC primitives used
// ═══════════════════
//  - 2-FF synchroniser for level signals (busy, error)
//  - Toggle-edge-detect for one-cycle pulses (go pb→core, done core→pb,
//    rd_valid core→pb, wr_ready core→pb)
//  - Stable-during-transaction sampling for request/response data
//    (cmd_type/lba/block_count, rd_data, wr_data)
//  - 2-FF level synchroniser pb→core for the two REAL back-pressure
//    levels: pb_rd_ready (consumer has room) and pb_wr_avail (producer
//    has a byte).  Both reset to the permissive value.
//
// Reset — asymmetric reset is a FIRST-CLASS case, not a corner case
// ══════════════════════════════════════════════════════════════════
// The two sides are driven from genuinely skewed reset trees:
//
//   core : soc_full_rst_bank[5] | warm_peripheral_reset  — combinational
//   pb   : pb_full_rst_bank[2]  — xpm_cdc_async_rst DEST_SYNC_FF(4)+BUFG
//
// so on EVERY debug-full-reset and every CPU-requested warm peripheral
// reset the core side resets ~4 pb cycles before the pb side.  There is
// therefore always a window in which one side is live and the other is
// being zeroed.
//
// Two rules make that window safe:
//
//  1. **Cross-domain toggles and their receive-side sync ladders carry
//     no local reset.**  A toggle's ABSOLUTE value carries no meaning —
//     only its edges do — so zeroing one at reset manufactures an edge
//     out of thin air.  Historically that produced, on a one-sided
//     reset: a spurious pb_done with pb_error freshly zeroed (the SCSI
//     target was told the transfer COMPLETED SUCCESSFULLY), a phantom
//     byte into scsi.v's 512-byte sector ring, and a phantom core_go.
//     Leaving the toggles free-running removes the fabricated edge
//     entirely; the reset-gated *pulse output* registers below still
//     guarantee no pulse escapes while the local side is held in reset.
//
//  2. **Each side publishes a reset-epoch toggle and watches the
//     peer's.**  An observed peer reset while a transfer is in flight
//     terminates that transfer as pb_done + pb_error (a detected error
//     the SCSI layer turns into a check condition) instead of leaving
//     the pb side parked on pb_busy forever.  An unsolicited pb_done —
//     one with no request outstanding — is suppressed outright.
//
// Net contract: a reset on one side only can LOSE a transfer, and will
// report that loss as an error.  It can never fabricate a completion.

`default_nettype none

module sd_scsi_bridge (
    // ── pb_clk side (interface to scsi.v) ──────────────────────────────
    input  wire        pb_clk,
    input  wire        pb_rst,
    input  wire [2:0]  pb_cmd_type,
    input  wire [31:0] pb_lba,
    input  wire [15:0] pb_block_count,
    input  wire        pb_go,
    output wire        pb_busy,
    output wire        pb_done,
    output wire        pb_error,
    output wire        pb_rd_valid,
    output wire [7:0]  pb_rd_data,
    input  wire        pb_rd_ready,    // back-pressure (synced to core)
    output wire        pb_wr_ready,
    input  wire [7:0]  pb_wr_data,
    input  wire        pb_wr_avail,    // back-pressure (synced to core)

    // ── core_clk side (interface to sd_ctrl) ───────────────────────────
    input  wire        core_clk,
    input  wire        core_rst,
    output wire [2:0]  core_cmd_type,
    output wire [31:0] core_lba,
    output wire [15:0] core_block_count,
    output wire        core_go,
    input  wire        core_busy,
    input  wire        core_done,
    input  wire        core_error,
    input  wire        core_rd_valid,
    input  wire [7:0]  core_rd_data,
    output wire        core_rd_ready,  // synced pb_rd_ready (back-pressure)
    input  wire        core_wr_ready,
    output wire [7:0]  core_wr_data,
    output wire        core_wr_avail   // synced pb_wr_avail (back-pressure)
);

    // ══════════════════════════════════════════════════════════════════
    // Peer-reset detection (rule 2 in the header)
    // ══════════════════════════════════════════════════════════════════
    // Each side inverts a free-running epoch toggle on every rising edge
    // of its OWN reset.  These FFs deliberately carry no reset — the
    // whole point is to survive one.  The epoch is a level that stays
    // flipped forever, so it needs no minimum pulse width to cross.
    //
    // The receiving ladders are free-running too, but every ACTION taken
    // on a detected peer reset lives inside the local `else` (not-in-
    // reset) arm below.  So when both sides reset together — the normal
    // SoC case — the detection is simply swallowed and no error is
    // reported for a reset that was intended.

    reg pb_rst_q      = 1'b0;
    reg pb_rst_epoch  = 1'b0;
    always @(posedge pb_clk) begin
        pb_rst_q <= pb_rst;
        if (pb_rst && !pb_rst_q) pb_rst_epoch <= ~pb_rst_epoch;
    end

    reg core_rst_q     = 1'b0;
    reg core_rst_epoch = 1'b0;
    always @(posedge core_clk) begin
        core_rst_q <= core_rst;
        if (core_rst && !core_rst_q) core_rst_epoch <= ~core_rst_epoch;
    end

    // core epoch → pb side
    reg pb_peer_ep_a = 1'b0, pb_peer_ep_b = 1'b0, pb_peer_ep_prev = 1'b0;
    always @(posedge pb_clk) begin
        pb_peer_ep_a    <= core_rst_epoch;
        pb_peer_ep_b    <= pb_peer_ep_a;
        pb_peer_ep_prev <= pb_peer_ep_b;
    end
    wire pb_peer_reset = pb_peer_ep_b ^ pb_peer_ep_prev;

    // pb epoch → core side
    reg core_peer_ep_a = 1'b0, core_peer_ep_b = 1'b0, core_peer_ep_prev = 1'b0;
    always @(posedge core_clk) begin
        core_peer_ep_a    <= pb_rst_epoch;
        core_peer_ep_b    <= core_peer_ep_a;
        core_peer_ep_prev <= core_peer_ep_b;
    end
    wire core_peer_reset = core_peer_ep_b ^ core_peer_ep_prev;

    // ══════════════════════════════════════════════════════════════════
    // Request path (pb_clk → core_clk): cmd_type/lba/block_count + go
    // ══════════════════════════════════════════════════════════════════
    // pb side latches the request fields when pb_go pulses; the request
    // toggles `pb_go_tog` in the same cycle.  The receiving side syncs
    // the toggle through 2 FFs and edge-detects to issue a 1-cycle
    // core_go pulse.  Because the core domain only acts on the toggle
    // edge, the latched request fields are guaranteed stable for many
    // core_clk cycles before sampling (sd_ctrl byte rate is ~320 ns;
    // the sync ladder closes in ~3 core cycles).

    reg [2:0]  pb_cmd_type_lat;
    reg [31:0] pb_lba_lat;
    reg [15:0] pb_block_count_lat;
    reg        pb_go_tog;
    reg        pb_busy_internal;        // 1 between go and done on pb side

    // pb_go_tog carries NO reset (header rule 1): zeroing it while it
    // sits at 1 is a fabricated 1→0 edge that launches an SD command
    // against stale lba/block_count.  The request LATCHES keep their
    // reset — they are only ever sampled after a go edge.
    always @(posedge pb_clk) begin
        if (pb_rst) begin
            pb_cmd_type_lat    <= 3'd0;
            pb_lba_lat         <= 32'd0;
            pb_block_count_lat <= 16'd0;
        end else if (pb_go && !pb_busy_internal) begin
            pb_cmd_type_lat    <= pb_cmd_type;
            pb_lba_lat         <= pb_lba;
            pb_block_count_lat <= pb_block_count;
        end
    end
    always @(posedge pb_clk) begin
        if (!pb_rst && pb_go && !pb_busy_internal) pb_go_tog <= ~pb_go_tog;
    end

    // core-side go toggle resync + edge detect.  Free-running ladder;
    // the emitted pulse is registered and reset-gated so no core_go can
    // escape while the core side is held in reset.
    reg core_go_tog_a = 1'b0, core_go_tog_b = 1'b0, core_go_tog_prev = 1'b0;
    always @(posedge core_clk) begin
        core_go_tog_a    <= pb_go_tog;
        core_go_tog_b    <= core_go_tog_a;
        core_go_tog_prev <= core_go_tog_b;
    end
    reg core_go_pulse;
    always @(posedge core_clk) begin
        if (core_rst) core_go_pulse <= 1'b0;
        else          core_go_pulse <= core_go_tog_b ^ core_go_tog_prev;
    end
    assign core_go = core_go_pulse;

    // The latched fields cross the domain by virtue of being stable for
    // the entire transaction.  No additional sync — sd_ctrl samples them
    // on the core_go pulse, which arrives ≥ 2 core_clk cycles after the
    // latches were written.
    assign core_cmd_type    = pb_cmd_type_lat;
    assign core_lba         = pb_lba_lat;
    assign core_block_count = pb_block_count_lat;

    // ══════════════════════════════════════════════════════════════════
    // Response path: busy / done / error
    // ══════════════════════════════════════════════════════════════════
    // busy is a level — 2-FF sync from core to pb is the canonical move.
    // done is a 1-cycle pulse on the core side — toggle and edge-detect.
    // error is sampled on the core side at the moment of done so it is
    // stable when the pb side observes pb_done.

    // core: track busy locally; toggle done_tog when done pulses; latch
    // error at done time.
    // core_done_tog carries NO reset (header rule 1).  A core reset with
    // the toggle at 1 used to look, from the pb side, exactly like a
    // completion — and with core_error_lat freshly zeroed it looked like
    // a SUCCESSFUL one.
    reg core_done_tog = 1'b0;
    reg core_error_lat;
    // Sticky: the pb side reset out from under an in-flight transfer, so
    // whatever sd_ctrl reports next belongs to a request whose issuer is
    // gone.  Flag it as an error rather than let it be mis-attributed to
    // whatever the freshly-reset pb side asks for next.
    reg core_peer_rst_sticky;
    always @(posedge core_clk) begin
        if (core_rst) begin
            core_error_lat      <= 1'b0;
            core_peer_rst_sticky<= 1'b0;
        end else begin
            if (core_done) core_error_lat <= core_error | core_peer_rst_sticky |
                                             core_peer_reset;
            if (core_peer_reset)   core_peer_rst_sticky <= 1'b1;
            else if (core_done)    core_peer_rst_sticky <= 1'b0;
        end
    end
    always @(posedge core_clk) begin
        if (!core_rst && core_done) core_done_tog <= ~core_done_tog;
    end

    // pb: 2-FF sync busy and done_tog; edge-detect the toggle; latch
    // error from the core domain on the done edge.
    // Free-running done-toggle ladder (header rule 1).
    reg pb_done_tog_a = 1'b0, pb_done_tog_b = 1'b0, pb_done_tog_prev = 1'b0;
    always @(posedge pb_clk) begin
        pb_done_tog_a    <= core_done_tog;
        pb_done_tog_b    <= pb_done_tog_a;
        pb_done_tog_prev <= pb_done_tog_b;
    end
    wire pb_done_edge = pb_done_tog_b ^ pb_done_tog_prev;

    // A completion is only meaningful against an outstanding request.
    // An unsolicited done — no pb_go ever issued, or issued and already
    // completed — is exactly the "fabricated completion" class and is
    // dropped rather than reported.
    wire pb_abort    = pb_peer_reset & pb_busy_internal;
    wire pb_complete = (pb_done_edge & pb_busy_internal) | pb_abort;

    reg pb_busy_a, pb_busy_b;
    reg pb_error_lat;
    reg pb_done_pulse;
    always @(posedge pb_clk) begin
        if (pb_rst) begin
            pb_busy_a        <= 1'b0;
            pb_busy_b        <= 1'b0;
            pb_error_lat     <= 1'b0;
            pb_done_pulse    <= 1'b0;
            pb_busy_internal <= 1'b0;
        end else begin
            pb_busy_a        <= core_busy;
            pb_busy_b        <= pb_busy_a;
            pb_done_pulse    <= pb_complete;
            if (pb_complete) begin
                // pb_abort wins: the core side vanished mid-transfer, so
                // core_error_lat describes nothing.  Report the error.
                pb_error_lat <= pb_abort ? 1'b1 : core_error_lat;
            end
            // pb_busy_internal: rises when go is sampled, falls when
            // done propagates back — or when the peer reset kills the
            // transfer, which is what stops the pb side parking on
            // pb_busy forever.
            if (pb_go && !pb_busy_internal) pb_busy_internal <= 1'b1;
            else if (pb_complete)           pb_busy_internal <= 1'b0;
        end
    end

    assign pb_busy  = pb_busy_b | pb_busy_internal;
    assign pb_done  = pb_done_pulse;
    assign pb_error = pb_error_lat;

    // ══════════════════════════════════════════════════════════════════
    // Read byte stream (core → pb): rd_valid pulse + rd_data
    // ══════════════════════════════════════════════════════════════════
    // sd_ctrl pulses core_rd_valid for one core_clk cycle per byte, with
    // core_rd_data valid that cycle.  We latch the byte on the core side
    // and toggle a flag.  pb side syncs the toggle, edge-detects to
    // produce pb_rd_valid; pb_rd_data is the (now-stable) latched byte.
    //
    // Backpressure: pb_rd_ready (level, pb domain) is synchronized into
    // the core domain through 2 FFs and fed to sd_ctrl's rd_ready.  The
    // stream is paced BEFORE the next SPI byte is issued, so the only
    // in-flight exposure after rd_ready falls is the 2-FF sync latency
    // plus one SPI byte — scsi.v's high-water mark (496 of 512) leaves
    // 16 bytes of headroom for that.  Per-byte pacing (each byte seen
    // with cycles to spare) is unchanged: rd_ready is about aggregate
    // ring occupancy, not per-byte handshaking.

    // core_rd_tog carries NO reset (header rule 1): a fabricated edge
    // here pushes one phantom byte into scsi.v's 512-byte sector ring
    // and permanently desyncs the ring pointer.
    reg [7:0] core_rd_data_lat;
    reg       core_rd_tog = 1'b0;
    always @(posedge core_clk) begin
        if (core_rst) core_rd_data_lat <= 8'h00;
        else if (core_rd_valid) core_rd_data_lat <= core_rd_data;
    end
    always @(posedge core_clk) begin
        if (!core_rst && core_rd_valid) core_rd_tog <= ~core_rd_tog;
    end

    reg pb_rd_tog_a = 1'b0, pb_rd_tog_b = 1'b0, pb_rd_tog_prev = 1'b0;
    always @(posedge pb_clk) begin
        pb_rd_tog_a    <= core_rd_tog;
        pb_rd_tog_b    <= pb_rd_tog_a;
        pb_rd_tog_prev <= pb_rd_tog_b;
    end
    wire pb_rd_edge = pb_rd_tog_b ^ pb_rd_tog_prev;

    reg pb_rd_valid_pulse;
    reg [7:0] pb_rd_data_reg;
    always @(posedge pb_clk) begin
        if (pb_rst) begin
            pb_rd_valid_pulse <= 1'b0;
            pb_rd_data_reg    <= 8'h00;
        end else begin
            pb_rd_valid_pulse <= pb_rd_edge;
            if (pb_rd_edge) pb_rd_data_reg <= core_rd_data_lat;
        end
    end

    assign pb_rd_valid    = pb_rd_valid_pulse;
    assign pb_rd_data     = pb_rd_data_reg;

    // pb_rd_ready → core_rd_ready: 2-FF level synchronizer.  Resets to
    // "ready" so consumers that never back-pressure are unaffected.
    reg core_rd_ready_ff1, core_rd_ready_ff2;
    always @(posedge core_clk) begin
        if (core_rst) begin
            core_rd_ready_ff1 <= 1'b1;
            core_rd_ready_ff2 <= 1'b1;
        end else begin
            core_rd_ready_ff1 <= pb_rd_ready;
            core_rd_ready_ff2 <= core_rd_ready_ff1;
        end
    end
    assign core_rd_ready  = core_rd_ready_ff2;

    // ══════════════════════════════════════════════════════════════════
    // Write byte stream: wr_ready pulse (core → pb) + wr_data (pb → core)
    // ══════════════════════════════════════════════════════════════════
    // sd_ctrl asks for a byte by pulsing core_wr_ready for one core_clk
    // cycle.  scsi.v keeps pb_wr_data continuously valid (the byte at
    // sec_buf[drain_ptr]); on the pb-side pb_wr_ready pulse it advances
    // drain_ptr and presents the next byte.
    //
    // Direction-by-direction:
    //   wr_ready pulse  (core → pb): toggle handshake, edge-detect on pb
    //   wr_data  level  (pb → core): 2-FF sync per data bit
    //
    // The round-trip (core wr_ready → pb advance → new pb_wr_data → core
    // visibility via sync) is ~6 cycles total — vastly less than the
    // ~320 ns per-byte SPI cadence, so sd_ctrl never sees stale data.

    // core_wr_tog carries NO reset (header rule 1): a fabricated edge
    // here advances scsi.v's drain_ptr, shifting every following byte.
    reg core_wr_tog = 1'b0;
    always @(posedge core_clk) begin
        if (!core_rst && core_wr_ready) core_wr_tog <= ~core_wr_tog;
    end

    reg pb_wr_tog_a = 1'b0, pb_wr_tog_b = 1'b0, pb_wr_tog_prev = 1'b0;
    always @(posedge pb_clk) begin
        pb_wr_tog_a    <= core_wr_tog;
        pb_wr_tog_b    <= pb_wr_tog_a;
        pb_wr_tog_prev <= pb_wr_tog_b;
    end

    reg pb_wr_ready_pulse;
    always @(posedge pb_clk) begin
        if (pb_rst) pb_wr_ready_pulse <= 1'b0;
        else        pb_wr_ready_pulse <= pb_wr_tog_b ^ pb_wr_tog_prev;
    end
    assign pb_wr_ready = pb_wr_ready_pulse;

    // pb_wr_avail → core_wr_avail: 2-FF level synchronizer, the exact
    // mirror of the pb_rd_ready → core_rd_ready ladder above.  Resets to
    // "available" so producers that never back-pressure are unaffected.
    //
    // In-flight exposure, and why `count != 0` is enough on the master
    // side: sd_ctrl consults this in S_W_DATA_S, i.e. BEFORE issuing an
    // SPI byte.  The round trip from "sd_ctrl took a byte" back to
    // "core_wr_avail reflects the new occupancy" is core_wr_ready →
    // toggle → 3 pb cycles → master decrements → 2 core cycles ≈ 70 ns,
    // while one SPI byte is ~320 ns.  So by the time sd_ctrl is back in
    // S_W_DATA_S the level has long since settled, and a stale-high
    // sample cannot let it issue a byte the master no longer has.  This
    // is the same argument the read side makes with its 16-byte
    // high-water margin, taken from the other end.
    reg core_wr_avail_ff1, core_wr_avail_ff2;
    always @(posedge core_clk) begin
        if (core_rst) begin
            core_wr_avail_ff1 <= 1'b1;
            core_wr_avail_ff2 <= 1'b1;
        end else begin
            core_wr_avail_ff1 <= pb_wr_avail;
            core_wr_avail_ff2 <= core_wr_avail_ff1;
        end
    end
    assign core_wr_avail = core_wr_avail_ff2;

    // wr_data: pb → core via per-bit 2-FF sync.  The data is held stable
    // by scsi.v between pb_wr_ready pulses, so when sd_ctrl latches on
    // its next core_wr_ready the synchronised value has long since
    // settled.  Per-bit sync is fine because sd_ctrl only cares about
    // the byte value at the wr_ready cycle, never about transient mid-
    // settle states.
    reg [7:0] core_wr_data_a, core_wr_data_b;
    always @(posedge core_clk) begin
        if (core_rst) begin
            core_wr_data_a <= 8'h00;
            core_wr_data_b <= 8'h00;
        end else begin
            core_wr_data_a <= pb_wr_data;
            core_wr_data_b <= core_wr_data_a;
        end
    end
    assign core_wr_data = core_wr_data_b;

endmodule

`default_nettype wire
