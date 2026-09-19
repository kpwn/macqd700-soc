// sd_ctrl.v — Unified SD-card data-transport engine (SPI mode).
//
// Purpose
//   Consolidates the per-sector data transfer state machines previously
//   duplicated in boot_fsm.v (CMD17) and sd_provision.v (CMD17 + CMD24).
//   Ported from sd-hdmi-bringup/rtl/sd_ctrl.v which has full hardware-
//   validated CMD17 / CMD18 / CMD24 / CMD25 / CMD12 coverage.  The port
//   strips out the init sequence (CMD0/8/55+41/58/6) and the framebuffer
//   write path — those remain callers' responsibility — keeping this
//   module focused on data transport only.
//
// What we support
//   cmd_type  |  name                      | block_count used? | streams
//   ----------|---------------------------|-------------------|--------
//   CT_IDLE   |  idle / no-op             | n/a               | none
//   CT_CMD17  |  READ_SINGLE_BLOCK        | no (forced 1)     | rd_*
//   CT_CMD18  |  READ_MULTIPLE_BLOCK+CMD12| yes               | rd_*
//   CT_CMD24  |  WRITE_SINGLE_BLOCK       | no (forced 1)     | wr_*
//   CT_CMD25  |  WRITE_MULTIPLE_BLOCK     | yes               | wr_*
//
// Assumptions
//   * Card has already been initialised (CMD0/8/55+41/58/[6]) by the
//     caller.  We do NOT touch spi_fast_mode / spi_hs_mode — the caller
//     drives the SPI master's mode pins directly.
//   * CS_N is held low by the caller while a command is in flight.  We
//     do not touch spi_cs_n (it's on the caller's side of the mux).
//   * The SPI byte interface uses the `sd_spi` module's standard
//     cmd_valid/ready + rsp_valid/data handshake.
//   * `lba` is the argument the caller wants in the command frame.  For
//     CMD17/18/24/25 on an SDHC card that's the block index; for SDSC
//     it's the byte address (caller pre-scales).  We don't care.
//   * `block_count` is the number of 512-byte blocks in CMD18 / CMD25
//     (must be >=1; implicitly 1 for CMD17/CMD24).  Upper bound is
//     16 bits — far above any practical single transfer.
//
// Data streaming interfaces
//   Reads (CMD17, CMD18): output byte stream, consumer-paced.
//       rd_valid pulses high for exactly one clock per byte.  The byte
//       is on rd_data.  rd_ready is REAL back-pressure: while it is low
//       the engine pauses before issuing the next SPI byte (and before
//       hunting for the next block's data token) — SPI is host-clocked,
//       so stalling between bytes is always legal for the card.  Up to
//       one already-issued byte may still complete after rd_ready
//       falls; consumers must keep >= 2 bytes of headroom below their
//       high-water mark.  Callers that never back-pressure tie
//       rd_ready high and get the legacy unpaced behaviour.
//       HISTORY: rd_ready used to be "purely informational" on the
//       theory that any consumer keeps pace with the ~320 ns/byte SPI
//       stream.  The Mac SCSI CMD18 ring consumer does NOT keep pace
//       unboundedly — a VBL ISR stalling the ROM's chunked drain let
//       the stream overrun scsi.v's 512-byte ring and wedge the boot
//       (HW wedge at ROM PC 0x40899322, 2026-07-15).
//
//   Writes (CMD24, CMD25): input byte stream, producer-paced.
//       wr_ready pulses high for exactly one clock when we're ready to
//       consume a byte.  The producer must have wr_data valid for that
//       cycle; wr_valid is an output status flag (= "we are in the
//       middle of a data block") — purely informational.  A caller that
//       reads wr_ready edges and presents the next byte on the next
//       cycle works correctly.
//       wr_avail is REAL back-pressure, the symmetric twin of rd_ready:
//       "the producer has at least one byte ready RIGHT NOW".  While it
//       is low the engine pauses before latching wr_data and before
//       issuing the next SPI byte — SPI is host-clocked, so stalling
//       between bytes is always legal for the card.  Callers that keep
//       a whole block buffered before they start tie it high and get
//       the legacy unpaced behaviour bit for bit.
//       HISTORY: there used to be NO write-side back-pressure at all.
//       S_W_DATA_S latched wr_data and issued the SPI byte
//       unconditionally whenever the byte engine was idle, so a
//       producer that stalled mid-block (on hardware: an interrupt
//       pre-empting the Mac's pseudo-DMA loop) had the drain run ahead
//       of the fill, and STALE bytes from the previous block went to
//       the card — at the correct LBA, with GOOD status.  scsi.v's
//       ring counter clamped the resulting underflow at 0 instead of
//       flagging it, so nothing anywhere noticed.  This is the exact
//       defect the READ side had until 2026-07-15 (see rd_ready above);
//       the write side is now symmetric.
//       BOUNDING: the new stall point is covered by the SAME global
//       per-request watchdog as every other pause in this module (see
//       BOUNDED RESPONSE below).  REQ_WDOG_* increments unconditionally
//       every cycle a request is live, so a producer that never comes
//       back turns into `done | error` (ERR_WDOG) rather than silence.
//       Do NOT add a per-state timeout for it.
//
// Completion / status
//   `done`       — one-cycle pulse when the requested transfer finishes
//                  (OK or error).  The caller must de-assert `go` before
//                  this pulse.
//   `busy`       — high from the cycle `go` is sampled until `done`.
//   `error`      — sticky; high from the cycle the error was detected
//                  until the next `go`.
//   `err_cause`  — 4-bit classification:
//                     0 = none
//                     1 = R1 non-zero                  (cmd rejected)
//                     2 = R1 poll timeout              (card silent)
//                     3 = 0xFE data token timeout      (read stall)
//                     4 = data-response token bad      (write reject)
//                     5 = busy timeout                 (write stuck)
//                     6 = unsupported cmd_type         (caller bug)
//                     7 = data-block CRC16 mismatch    (bad read)
//                     8 = global request watchdog      (see below)
//
// BOUNDED RESPONSE (the vhdd contract, rtl/vhdd.vh)
//   Every OTHER timeout in this module is gated on an event — poll_cnt
//   and busy_cnt only advance on a completed SPI byte (`bdone`),
//   token_cnt only on a completed token poll.  Both read-side pause
//   points (S_TOK_SEND and S_RD_FLUSH_S) are gated on `rd_ready`, and
//   the write-side pause point (S_W_DATA_S) is gated on `wr_avail`;
//   while either is low no SPI byte ever completes — so every one of
//   those counters freezes at once and `busy` stays high forever.  That
//   is a real deadlock: the SCSI target waits on this module, the CPU
//   waits on the SCSI target's pseudo-DMA handshake, and the thing that
//   would eventually raise rd_ready / wr_avail is the CPU.
//   REQ_WDOG_* below is the backstop: ONE counter, armed at `go`,
//   incremented UNCONDITIONALLY every cycle the request is live, cleared
//   only when the request terminates.  It bounds every state at once,
//   including states added later.  Do not gate it, and do not replace it
//   with per-state timeouts.
//
// SPI interface
//   Same shape as sd_spi.v's byte handshake.  CS_N is caller-owned.
//
// Line budget
//   Full reference in ~890 lines (with init + framebuffer).  This port
//   fits in ~420 lines by leaving init + fb + dcache integration out.
//
// Port line-range cross-reference (upstream vs this module)
//   upstream.v ST_FRAME_SEND/WAIT        : lines 392-410  → S_CMD_SEND/WAIT
//   upstream.v ST_R1_SEND/WAIT           : lines 413-548  → S_R1_SEND/WAIT
//   upstream.v ST_TOKEN_SEND/WAIT        : lines 600-622  → S_TOK_SEND/WAIT
//   upstream.v ST_DATA_SEND/WAIT (read)  : lines 625-660  → S_RD_SEND/WAIT
//   upstream.v ST_CRC_SEND/WAIT          : lines 663-713  → S_CRC_SEND/WAIT
//   upstream.v ST_W_TOK_* / ST_W_DATA_*  : lines 751-802  → S_W_TOK_* /
//                                                            S_W_DATA_*
//   upstream.v ST_W_RESP_* / ST_W_BUSY_* : lines 804-853  → S_W_RESP_* /
//                                                            S_W_BUSY_*
//   upstream.v ST_W_STOP_* / ST_W_FBUSY_*: lines 856-884  → S_W_STOP_* /
//                                                            S_W_FBUSY_*

module sd_ctrl #(
    // ── Global per-request watchdog budget shape ──────────────────────
    // These defaults ARE the shipping values; see the GLOBAL PER-REQUEST
    // WATCHDOG block far below for the derivation and the trade-off.
    // They are parameters (not localparams) for exactly one reason: the
    // unit testbench (tb/tb_sd_ctrl.v) instantiates extra copies of this
    // module with a shortened prescaler / base so the watchdog MECHANISM
    // can be proven in a few million simulated cycles instead of the
    // ~1e9 the shipping budget costs.  No RTL caller overrides them, and
    // none should — a caller that wants a different bound is really
    // asking for a different contract.
    //
    //   REQ_WDOG_TICK_LOG2   prescaler: 1 tick = 2**this core cycles.
    //   REQ_WDOG_BLK_SHIFT   per-block allowance = 2**this ticks.
    //   REQ_WDOG_BASE_TICKS  fixed floor, in ticks.
    //
    //   REQ_WDOG_ENABLE      0 removes the watchdog's ONLY effect: the
    //                        `wdog_expired` -> S_ERROR/ERR_WDOG escape.
    //                        The counter still arms, counts and clears
    //                        exactly as before, so nothing else in the
    //                        module changes shape or timing.  Default 1
    //                        keeps the bounded-response contract, so
    //                        every existing caller and tb is unaffected.
    //
    //                        DISABLING IS NOT FREE: the watchdog is what
    //                        turns a provider that never answers into a
    //                        bounded ERROR.  With it off, such a request
    //                        parks forever and the caller wedges rather
    //                        than failing.  Only set 0 deliberately, to
    //                        test whether a *spurious* expiry is killing
    //                        a healthy-but-slow request — a hang then
    //                        localises the stall instead of hiding it
    //                        behind a fallback path.
    parameter integer REQ_WDOG_TICK_LOG2  = 12,
    parameter integer REQ_WDOG_BLK_SHIFT  = 5,
    parameter [21:0]  REQ_WDOG_BASE_TICKS = 22'd245760,
    parameter integer REQ_WDOG_ENABLE     = 1,
    // Preserve a caller's continuous multi-block stream while issuing one
    // fully closed CMD24 per block.  The SCSI volume enables this so a reset
    // or producer failure cannot strand later traffic inside CMD25.  The
    // staged and verified provisioning path retains CMD25 throughput.
    parameter integer MULTI_WRITE_AS_CMD24 = 0,
    // Read replay spacing in core clocks (1..63). Callers using a toggle
    // CDC must keep the byte stable through its complete sampling window.
    // Default retains the legacy cadence for boot and provisioning users.
    parameter integer RD_FLUSH_PACE_CYCLES = 32,
    // Two sector banks overlap SPI reception with CRC-validated delivery.
    // Disabled by default for callers retaining the single-buffer behavior.
    parameter integer READ_PIPELINE = 0
) (
    input  wire        clk,
    input  wire        rst,

    // CRC16-CCITT (poly 0x1021) validation gate for reads.  Low (the
    // default — leave unconnected on any existing caller/tb) preserves
    // bit-exact legacy behaviour: the 2 trailing data-block bytes are
    // read off the wire and discarded, exactly as before.  High enables:
    // (a) buffering each 512-byte block internally before releasing it
    // to rd_valid/rd_data, so a bad block is NEVER visible to the
    // caller; (b) validating the received CRC16 against one computed
    // over the buffered bytes; (c) on mismatch, CMD17/CMD24 (standalone
    // single-block) transparently retries the WHOLE command up to
    // CRC_RETRY_MAX times before surfacing ERR_CRC_BAD — safe because
    // nothing has been streamed to the caller yet.  CMD18 (multi-block)
    // does not retry per-block (the SD protocol cannot replay a block
    // mid-stream without a CMD12 abort + fresh command at the failed
    // block's LBA — left as a caller-level concern): a bad block cleanly
    // aborts the whole multi-block run (CMD12 + ERR_CRC_BAD) exactly like
    // the existing ERR_TOK_TO path, letting the caller's OWN existing
    // error handling (which already maps sd_ctrl `error` to SCSI CHECK
    // CONDITION / MEDIUM ERROR) drive OS-level retry.  The card must
    // actually have CRC_ON_OFF (CMD59) enabled by the caller BEFORE
    // asserting this — see boot_fsm.v's init sequence.  Writes always
    // send a real computed CRC16 in the data block regardless of this
    // input (harmless whether or not the card is validating it).
    input  wire        crc_check_en,

    // ── Command request handshake ─────────────────────────────────────
    input  wire [2:0]  cmd_type,       // see CT_* localparams below
    input  wire [31:0] lba,            // block address / byte address
    input  wire [15:0] block_count,    // for CMD18/25; >=1
    input  wire        go,             // one-cycle pulse: start the cmd

    // ── Read byte stream (CMD17 / CMD18) ──────────────────────────────
    output reg         rd_valid,       // 1 cycle per byte received
    output reg  [7:0]  rd_data,
    input  wire        rd_ready,       // back-pressure: low pauses stream

    // ── Write byte stream (CMD24 / CMD25) ─────────────────────────────
    output reg         wr_ready,       // 1 cycle per byte consumed
    output reg         wr_valid,       // high during data-block phase
    input  wire [7:0]  wr_data,
    // Real back-pressure: "the producer has a byte ready right now".
    // Low pauses the data-block stream before the next SPI byte.  See
    // the "Writes" section of the header for the history and for why
    // this is bounded by REQ_WDOG_* rather than a per-state timeout.
    // Callers with the whole block already buffered tie this high.
    input  wire        wr_avail,

    // ── SPI byte interface (to sd_spi / sd_spi_mux) ───────────────────
    output reg         spi_cmd_valid,
    input  wire        spi_cmd_ready,
    output reg  [7:0]  spi_cmd_data,
    input  wire        spi_rsp_valid,
    input  wire [7:0]  spi_rsp_data,

    // ── Status ────────────────────────────────────────────────────────
    output reg         busy,
    output reg         done,           // one-cycle pulse
    output reg         error,          // sticky
    output reg  [3:0]  err_cause,

    // ── CRC diagnosis taps ────────────────────────────────────────────
    // Real-HW finding 2026-07-23: a whole-run CMD18 retry (boot_fsm.v)
    // failed identically on every attempt, sector 0 every time — raises
    // the question of whether THIS card's data-block CRC16 field is
    // even genuine once CRC_ON_OFF (CMD59) is accepted, versus a fixed/
    // bogus placeholder some cheap SD controllers send regardless of
    // the CRC-mode setting. rd_crc_calc / a captured received-CRC value
    // both hold their last value once `error` latches (the FSM stops
    // advancing), so reading them after a CRC-caused error shows
    // exactly what was compared for the failing block.
    output wire [15:0] dbg_rd_crc_calc,
    output wire [15:0] dbg_rd_crc_recv,
    // Last real R1 byte (MSB clear) this module received for ANY
    // command it sent — lets a caller distinguish WHICH R1 error bit
    // fired (e.g. 0x08=COM_CRC_ERROR vs 0x04=ILLEGAL_COMMAND) instead of
    // just seeing the generic err_cause=ERR_R1_BAD.
    output reg  [7:0]  dbg_last_real_r1,
    // Direct visibility into what command was actually latched/on-the-
    // wire (SC_* encoding) and its argument, for diagnosing whether an
    // R1 rejection is really for a fresh CMD18 or a leftover CMD12
    // abort, and whether lba_lat genuinely holds the expected value.
    output reg  [3:0]  dbg_cur_cmd,
    output reg  [31:0] dbg_lba_lat,
    output reg  [7:0]  dbg_last_crc7_sent,
    // Last byte sampled while polling for a write data-response token.
    // On ERR_DR_BAD this is the rejected token (0x0B/0x0D), or 0xFF if
    // the card never produced a token before the poll timeout.
    output reg  [7:0]  dbg_last_write_resp,
    // Current zero-based block within CMD18/CMD25.  Stable on error so a
    // caller can turn dbg_lba_lat into the exact failing physical sector.
    output wire [15:0] dbg_block_idx,
    // How many 0xFF poll bytes were consumed before the real R1 was
    // captured (0 = R1 arrived on the very first poll byte). Helps
    // distinguish "R1 sampled immediately, first byte after CRC7" from
    // "genuine busy-wait polling occurred first" when diagnosing a
    // corrupted-looking R1 value.
    output reg  [7:0]  dbg_last_poll_cnt
);

    assign dbg_block_idx = block_idx;

    // ──────────────────────────────────────────────────────────────────
    // Command types (exported to callers via `localparam` defines that
    // they can replicate — kept simple to avoid a shared .vh header).
    // ──────────────────────────────────────────────────────────────────
    localparam [2:0]
        CT_IDLE   = 3'd0,
        CT_CMD17  = 3'd1,
        CT_CMD18  = 3'd2,
        CT_CMD24  = 3'd3,
        CT_CMD25  = 3'd4;

    // Error cause encodings (also defined in the header above)
    localparam [3:0]
        ERR_NONE       = 4'd0,
        ERR_R1_BAD     = 4'd1,
        ERR_R1_TO      = 4'd2,
        ERR_TOK_TO     = 4'd3,
        ERR_DR_BAD     = 4'd4,
        ERR_BUSY_TO    = 4'd5,
        ERR_UNK_CMD    = 4'd6,
        ERR_CRC_BAD    = 4'd7,   // computed CRC16 != card's data-block CRC
        ERR_WDOG       = 4'd8;   // global per-request watchdog expired

    // Internal "current SD command on the wire" encoding.
    localparam [3:0]
        SC_NONE  = 4'd0,
        SC_CMD17 = 4'd1,
        SC_CMD18 = 4'd2,
        SC_CMD24 = 4'd3,
        SC_CMD25 = 4'd4,
        SC_CMD12 = 4'd5;

    // ──────────────────────────────────────────────────────────────────
    // BYTE-LEVEL SUB-FSM (same idiom as upstream sd_ctrl.v / boot_fsm.v)
    //   Main FSM pulses `bgo` with `bt` = byte to send.  On completion
    //   the sub-FSM pulses `bdone` with `br` = received byte.
    // ──────────────────────────────────────────────────────────────────
    localparam [1:0]
        BS_IDLE = 2'd0,
        BS_SEND = 2'd1,
        BS_WAIT = 2'd2;

    reg  [1:0] bst;
    reg        bgo;
    reg  [7:0] bt;
    reg        bdone;
    reg  [7:0] br;

    // A reset arriving during CMD24/CMD25 cannot simply erase this FSM:
    // the card remains in its write session and consumes the next owner's
    // traffic as payload.  local_rst is deferred until the write has gone
    // through the bounded S_AB_* close below.  FPGA register initialisation
    // covers the only interval before the first external reset edge.
    reg  reset_close_pending = 1'b0;
    wire local_rst;

    // Forward declaration — driven far below, next to the main FSM's
    // other timeout constants (see the GLOBAL PER-REQUEST WATCHDOG
    // block).  Declared here because the byte sub-FSM has to unstick
    // itself when the watchdog fires.
    wire wdog_expired;
    wire wr_reset_byte_retire_required;

    always @(posedge clk) begin
        if (local_rst) begin
            bst           <= BS_IDLE;
            spi_cmd_valid <= 1'b0;
            bdone         <= 1'b0;
            br            <= 8'h00;
            spi_cmd_data  <= 8'hFF;
        end else begin
            bdone <= 1'b0;
            case (bst)
                BS_IDLE: begin
                    spi_cmd_valid <= 1'b0;
                    if (bgo) begin
                        spi_cmd_data  <= bt;
                        spi_cmd_valid <= 1'b1;
                        bst           <= BS_SEND;
                    end
                end
                BS_SEND: begin
                    if (spi_cmd_ready) begin
                        spi_cmd_valid <= 1'b0;
                        bst           <= BS_WAIT;
                    end
                end
                BS_WAIT: begin
                    if (spi_rsp_valid) begin
                        br    <= spi_rsp_data;
                        bdone <= 1'b1;
                        bst   <= BS_IDLE;
                    end else if (wdog_expired &&
                                 !wr_reset_byte_retire_required) begin
                        // The main FSM is being forced to S_ERROR this
                        // cycle.  If we are here because the SPI master
                        // never answered, leaving bst parked in BS_WAIT
                        // would make the NEXT request stall too (every
                        // state entry gates on `bst == BS_IDLE`), so the
                        // watchdog would have to fire all over again.
                        // Safe to drop out here specifically: spi_cmd_valid
                        // is already low in BS_WAIT, so no in-flight
                        // valid/ready handshake is broken.  BS_SEND is
                        // deliberately NOT aborted — dropping cmd_valid
                        // before cmd_ready would violate sd_spi's
                        // handshake, and a stale bdone landing in S_IDLE
                        // is harmless (S_IDLE ignores it).
                        bst   <= BS_IDLE;
                        bdone <= 1'b0;
                    end
                end
                default: bst <= BS_IDLE;
            endcase
        end
    end

    // ──────────────────────────────────────────────────────────────────
    //                           MAIN FSM
    // ──────────────────────────────────────────────────────────────────
    localparam [5:0]
        S_IDLE        = 5'd0,
        S_CMD_SEND    = 5'd1,
        S_CMD_WAIT    = 5'd2,
        S_R1_SEND     = 5'd3,
        S_R1_WAIT     = 5'd4,
        // Read (single and multi-block share the token/data/crc loop)
        S_TOK_SEND    = 5'd5,
        S_TOK_WAIT    = 5'd6,
        S_RD_SEND     = 5'd7,
        S_RD_WAIT     = 5'd8,
        S_CRC_SEND    = 5'd9,
        S_CRC_WAIT    = 5'd10,
        // Write
        S_W_GAP_S     = 5'd11,   // 0xFF gap before first data token
        S_W_GAP_W     = 5'd12,
        S_W_TOK_S     = 5'd13,   // 0xFE single-block, 0xFC multi-block
        S_W_TOK_W     = 5'd14,
        S_W_DATA_S    = 5'd15,   // stream 512 bytes from wr_data
        S_W_DATA_W    = 5'd16,
        S_W_DCRC_S    = 5'd17,   // 2 dummy CRC bytes
        S_W_DCRC_W    = 5'd18,
        S_W_RESP_S    = 5'd19,   // read data-response byte
        S_W_RESP_W    = 5'd20,
        S_W_BUSY_S    = 5'd21,   // poll busy release
        S_W_BUSY_W    = 5'd22,
        S_W_STOP_S    = 5'd23,   // 0xFD stop-tran token (CMD25 tail)
        S_W_STOP_W    = 5'd24,
        S_W_FBUSY_S   = 5'd25,   // final busy poll after stop-tran
        S_W_FBUSY_W   = 5'd26,
        S_DONE        = 5'd27,
        S_ERROR       = 5'd28,
        // One dummy 0xFF byte-clock issued right after CS# asserts, before
        // the command frame.  Standard SD-SPI idiom (e.g. Chan's FatFs MMC
        // driver: CS_LOW(); xmit_spi(0xFF);) — gives the card >=8 clock
        // edges with CS# already low before the first real command bit.
        // Added 2026-07-15 during the second-command-silence bring-up
        // investigation: a live ILA capture showed the command frame bytes
        // for the SECOND CMD25 in a back-to-back pair are bit-correct, yet
        // the card never drives R1 (MISO pinned high, i.e. truly silent,
        // not busy) — consistent with the card's command decoder not being
        // in a clean state to receive the very next clocked byte as a
        // fresh command after a prior transaction, absent this dummy byte.
        S_CMD_PRE_S   = 6'd29,
        S_CMD_PRE_W   = 6'd30,
        // Buffered-read flush: replay the just-validated 512-byte block
        // from blk_buf to rd_valid/rd_data, paced by a local delay
        // counter (flush_pace_cnt/FLUSH_PACE_CYCLES below) rather than
        // a free-running 1-byte/cycle blast or the real SPI byte-clock
        // — see the state's own implementation comment for why touching
        // the physical SPI bus here is actively wrong for CMD18.
        S_RD_FLUSH_S  = 6'd31,
        // CMD12 (STOP_TRANSMISSION) sent to abort a multi-block read gets
        // one undefined "stuff" byte before its real R1 (SD Physical
        // Layer Simplified Spec — the card is still finishing internal
        // read-pipeline state when CMD12 lands). This byte is NOT
        // guaranteed to be 0xFF, so the generic S_R1_WAIT "poll until
        // non-0xFF" loop can misinterpret it as a bogus real R1 — found
        // 2026-07-24 via fpga_top_sdmin per-attempt telemetry: attempt 0
        // reached sector=2048 (full, CRC32-verified correct transfer)
        // then got err_cause=ERR_R1_BAD (R1=0x04) on CMD12 itself, and
        // every subsequent whole-run retry failed immediately at
        // sector=0 with the same code — consistent with the card being
        // left in a confused state by a misframed CMD12 response.
        // Unconditionally discard exactly one byte after CMD12's frame,
        // before starting the normal R1 poll.
        S_CMD12_STUFF_S = 6'd32,
        S_CMD12_STUFF_W = 6'd33,

        // ────────── GRACEFUL WRITE-SESSION CLOSE (S_AB_*) ──────────
        //
        // WHY THIS EXISTS (2026-08-09).  A CMD24/CMD25 write puts the
        // CARD into a receive state that only the host can leave: the
        // card counts exactly 512 data bytes + 2 CRC bytes per block,
        // and a CMD25 stays in "sequential write" mode until it sees the
        // 0xFD stop-tran token.  Deasserting CS# does NOT end either.
        //
        // Every error exit on the write path used to jump STRAIGHT to
        // S_ERROR, which deasserts `busy` and pulses `done` while the
        // card is still mid-block or still in sequential-write mode.
        // The next master to touch the card (pram_sd's CMD24 at LBA
        // 8191, or the next SCSI request) then has its COMMAND FRAME,
        // its R1 poll bytes and its 512-byte payload consumed by the
        // card as write DATA — programmed at the CARD's own running
        // write pointer, which is inside the HDD image at LBA 8192+.
        // That single defect produces both reported symptoms at once:
        // the HDD image is clobbered AND nothing is ever written at
        // 8191, so PRAM never persists.  See tb/tb_sd_ctrl.cpp
        // scenarios test_cmd25_abort_leaves_card_open and
        // test_cmd25_abort_then_write_hits_wrong_lba.
        //
        // The close sequence is the protocol-faithful recovery, not a
        // delay or a watchdog:
        //   PAD   — finish the current 512-byte data block with 0x00 so
        //           the card's byte counter reaches its block boundary.
        //   CRC   — send a DELIBERATELY INVALID CRC16 (the complement of
        //           the running value) so the card REJECTS the padded
        //           block instead of committing a half-real sector.  Per
        //           the SD Physical Layer spec a CRC-rejected block in a
        //           multiple-block write makes the card stop accepting
        //           data until the stop-tran token — exactly what we are
        //           about to send.
        //   RESP  — consume the data-response byte.
        //   BUSY  — poll until the card releases MISO.
        //   STOP  — 0xFD stop-tran (CMD25 only; a CMD24 block boundary
        //           already returns the card to command-wait).
        //   FBUSY — final busy poll, then S_ERROR.
        //
        // `error` / `err_cause` are latched by whoever decided to abort
        // and are NEVER overwritten here, so the caller still sees the
        // ORIGINAL cause (ERR_DR_BAD / ERR_BUSY_TO / ERR_WDOG).  Byte
        // budget is fixed (<= 512 + 2 + 1 + BUSY_TIMEOUT + 1 +
        // BUSY_TIMEOUT), so the close is bounded without any new
        // per-state timeout.
        S_AB_PAD_S    = 6'd34,
        S_AB_PAD_W    = 6'd35,
        S_AB_CRC_S    = 6'd36,
        S_AB_CRC_W    = 6'd37,
        S_AB_RESP_S   = 6'd38,
        S_AB_RESP_W   = 6'd39,
        S_AB_BUSY_S   = 6'd40,
        S_AB_BUSY_W   = 6'd41,
        S_AB_STOP_S   = 6'd42,
        S_AB_STOP_W   = 6'd43,
        S_AB_FBUSY_S  = 6'd44,
        S_AB_FBUSY_W  = 6'd45,
        S_RD_DRAIN    = 6'd46;

    // Widened from [4:0] (5 bits, 0..31) to [5:0] to make room for the
    // new S_RD_FLUSH_S state above the pre-existing 0..30 range.
    reg [5:0]  st;
    reg [3:0]  cur_cmd;        // SC_* — which command frame is on-the-wire
    reg [2:0]  frame_idx;      // 0..5 for 6-byte CMD frame
    reg [9:0]  byte_idx;       // 0..511 within a data block
    reg [15:0] block_idx;      // 0..block_count_latched-1
    reg [19:0] poll_cnt;
    reg [19:0] token_cnt;
    reg [19:0] busy_cnt;

    // ── CRC7 (SD command-frame CRC, poly x^7+x^3+1) ───────────────────
    // Real-HW finding 2026-07-23: this card enforces command-frame CRC7
    // validation UNCONDITIONALLY (rejects CMD18 with R1=COM_CRC_ERROR
    // even with CMD59/CRC_ON_OFF never sent at all) — the "CRC7 is only
    // required on CMD0/CMD8 while CRC-mode is off" relaxation some SD
    // controllers allow does not hold on this card. Every command frame
    // here previously sent a fixed dummy CRC7+stop byte (8'h01), which
    // is spec-legal ONLY under that relaxation. Algorithm/constants
    // verified against ZipCPU's sdspi bench/cpp/sdspisim.cpp
    // (SDSPISIM::cmdcrc) — a real, hardware-validated SD-SPI driver's
    // own test model — computed over the 5 command-header bytes
    // (command byte + 4 argument bytes), NOT including the CRC7/stop
    // byte itself.
    // verilator lint_off BLKSEQ
    function [7:0] crc7_frame;
        input [7:0] b0, b1, b2, b3, b4;
        integer i, j;
        reg [7:0] fill;
        begin
            fill = 8'h00;
            for (i = 0; i < 5; i = i + 1) begin
                case (i)
                    0: fill = fill ^ b0;
                    1: fill = fill ^ b1;
                    2: fill = fill ^ b2;
                    3: fill = fill ^ b3;
                    4: fill = fill ^ b4;
                endcase
                for (j = 0; j < 8; j = j + 1) begin
                    if (fill[7])
                        fill = (fill << 1) ^ 8'h12;
                    else
                        fill = fill << 1;
                end
            end
            crc7_frame = (fill & 8'hFE) | 8'h01;
        end
    endfunction
    // verilator lint_on BLKSEQ

    // ── CRC16-CCITT (poly 0x1021, init 0x0000, non-reflected) ─────────
    // Standard SD/MMC data-block CRC.  One instance computes over
    // received read bytes (compared against the card's trailing 2
    // bytes when crc_check_en); a second computes over transmitted
    // write bytes (always sent for real, regardless of crc_check_en —
    // harmless whether or not the card is validating it).
    // verilator lint_off BLKSEQ
    function [15:0] crc16_step;
        input [15:0] c;
        input [7:0]  b;
        integer k;
        reg [15:0] x;
        begin
            x = c ^ {b, 8'h00};
            for (k = 0; k < 8; k = k + 1)
                x = x[15] ? ((x << 1) ^ 16'h1021) : (x << 1);
            crc16_step = x;
        end
    endfunction
    // verilator lint_on BLKSEQ

    reg [15:0] rd_crc_calc;      // running CRC over the buffered block
    reg [7:0]  rd_crc_recv_hi;   // first (high) CRC byte off the wire
    reg [15:0] rd_crc_recv_latched; // received CRC, latched at compare time
                                     // (see S_CRC_WAIT) — NOT derived from
                                     // live `br`, which CMD12-abort traffic
                                     // reuses immediately afterward.
    assign dbg_rd_crc_calc = rd_crc_calc;
    assign dbg_rd_crc_recv = rd_crc_recv_latched;
    // dbg_cur_cmd / dbg_lba_lat are `output reg`, latched at S_R1_WAIT
    // alongside dbg_last_real_r1 (see that comment) — NOT continuously
    // assigned from the live cur_cmd/lba_lat, which reset to SC_NONE/0
    // once the FSM settles back to S_IDLE, well before any caller gets
    // a chance to read them.
    reg [15:0] wr_crc_calc;      // running CRC over the transmitted block
    reg        wr_resp_seen;     // first non-idle data-response byte captured
    reg [2:0]  crc_retry_cnt;    // CMD17/24 whole-command retry count
    localparam [2:0] CRC_RETRY_MAX = 3'd5;
    // Single-block staging buffer: a read is fully received and CRC-
    // validated here BEFORE any byte reaches rd_valid/rd_data, so a
    // failed block is never observed by the caller and a retry is
    // side-effect-free.  Reused per-block for CMD18 (validated/flushed
    // one block at a time — never holds more than one block).
    (* ram_style = "block" *) reg [7:0] blk_buf [0:(READ_PIPELINE ? 1024 : 512)-1];
    reg [1:0] rd_bank_valid;
    reg rd_fill_bank, rd_drain_bank;
    reg [8:0] blk_flush_idx;
    reg [5:0] flush_pace_cnt;
    // Purely a CDC-margin safety pace for S_RD_FLUSH_S — see that
    // state's comment.  Needs only to comfortably clear the
    // sd_scsi_bridge toggle-sync window (~12 core_clk cycles at 200 MHz).
    // The shared default is 32; SCSI selects 16 (80 ns at 200 MHz) to
    // reduce the non-overlapped replay cost while retaining CDC margin.
    localparam [5:0] FLUSH_PACE_CYCLES = RD_FLUSH_PACE_CYCLES;

    // ── Write-session close classification (see the S_AB_* block above) ─
    //
    // "The card is in a state only the host can leave."  True from the
    // moment CMD24/CMD25's R1 is accepted (S_W_GAP_S) until either the
    // single block's busy releases (CMD24) or the stop-tran token has
    // been sent (CMD25).  S_W_STOP_*/S_W_FBUSY_* are deliberately NOT in
    // the set: those ARE the close, so an abort from there has nothing
    // left to do but report.
    wire st_wr_indata = (st == S_W_TOK_W)  || (st == S_W_DATA_S) ||
                        (st == S_W_DATA_W);
    wire st_wr_incrc  = (st == S_W_DCRC_S) || (st == S_W_DCRC_W);
    wire st_wr_inresp = (st == S_W_RESP_S) || (st == S_W_RESP_W);
    wire st_wr_inbusy = (st == S_W_BUSY_S) || (st == S_W_BUSY_W);
    // Card is in command-wait (CMD24) or await-token (CMD25): no data
    // block is open, so the close is just the stop-tran token.
    wire st_wr_intok  = (st == S_W_GAP_S)  || (st == S_W_GAP_W) ||
                        (st == S_W_TOK_S);
    wire wr_close_needed = st_wr_indata | st_wr_incrc | st_wr_inresp |
                           st_wr_inbusy | st_wr_intok;
    // Where an abort from the current state must re-enter the protocol.
    wire [5:0] wr_close_entry = st_wr_indata ? S_AB_PAD_S
                              : st_wr_incrc  ? S_AB_CRC_S
                              : st_wr_inresp ? S_AB_RESP_S
                              : st_wr_inbusy ? S_AB_BUSY_S
                              :                S_AB_STOP_S;

    // A *_W state has one SPI byte in flight until bdone.  Reset/watchdog
    // closure must retire that byte before changing protocol state: W has no
    // address or byte tag, so replaying or skipping it shifts the card's
    // data/CRC/response framing.  The exact post-retirement entry below is
    // also used on the bdone cycle, where the ordinary state transition and
    // this override execute together (last nonblocking assignment wins).
    wire wr_close_wait_byte = ((st == S_W_GAP_W)  || (st == S_W_TOK_W)  ||
                               (st == S_W_DATA_W) || (st == S_W_DCRC_W) ||
                               (st == S_W_RESP_W) || (st == S_W_BUSY_W)) &&
                              !bdone;
    wire wr_resp_is_token = (br[4] == 1'b0) && (br[0] == 1'b1);
    wire [5:0] wr_close_entry_exact =
        (st == S_W_TOK_W)  ? S_AB_PAD_S :
        (st == S_W_DATA_W) ? ((byte_idx == 10'd511) ? S_AB_CRC_S
                                                        : S_AB_PAD_S) :
        (st == S_W_DCRC_W) ? ((byte_idx == 10'd1) ? S_AB_RESP_S
                                                       : S_AB_CRC_S) :
        (st == S_W_RESP_W) ? (wr_resp_is_token ? S_AB_BUSY_S
                                               : S_AB_RESP_S) :
        (st == S_W_BUSY_W) ? ((br != 8'h00) ? S_AB_STOP_S
                                               : S_AB_BUSY_S) :
                              wr_close_entry;
    wire [9:0] wr_close_byte_idx_exact =
        (st == S_W_TOK_W)  ? 10'd0 :
        (st == S_W_DATA_W) ? ((byte_idx == 10'd511) ? 10'd0
                                                        : byte_idx + 10'd1) :
        (st == S_W_DCRC_W) ? ((byte_idx == 10'd1) ? 10'd0 : 10'd1) :
                              byte_idx;

    // cur_cmd remains SC_CMD24/25 from command-frame launch through the
    // terminal state, including the normal and abort close tails.  Keep the
    // byte engine alive for that whole interval; narrowing this to only the
    // data states leaves an R1-accept/reset race that can still strand the
    // card in write mode.
    wire write_reset_active = busy &&
                              ((cur_cmd == SC_CMD24) || (cur_cmd == SC_CMD25));
    assign wr_reset_byte_retire_required = reset_close_pending &&
                                           write_reset_active &&
                                           wr_close_needed;
    assign local_rst = (rst || reset_close_pending) && !write_reset_active;

    always @(posedge clk) begin
        if (local_rst) begin
            reset_close_pending <= 1'b0;
        end else if (rst && write_reset_active) begin
            reset_close_pending <= 1'b1;
        end
    end

    // One-shot re-arm of the global request watchdog so the close
    // sequence gets its own bounded budget instead of being killed on
    // entry by the still-asserted expiry that sent us there.  Set for
    // exactly one cycle by the main FSM, consumed by the watchdog block.
    reg wdog_rearm;
    reg wdog_rearmed_q;   // sticky: at most one re-arm per request

    // Latched request
    reg [2:0]  cmd_type_lat;
    reg [31:0] lba_lat;
    reg [15:0] block_count_lat;

    // Frame-byte LUT (cur_cmd + frame_idx) → outgoing byte.
    reg [7:0] frame_byte;
    always @(*) begin
        case (cur_cmd)
            SC_CMD17: case (frame_idx)
                3'd0:    frame_byte = 8'h51;
                3'd1:    frame_byte = lba_lat[31:24];
                3'd2:    frame_byte = lba_lat[23:16];
                3'd3:    frame_byte = lba_lat[15: 8];
                3'd4:    frame_byte = lba_lat[ 7: 0];
                default: frame_byte = crc7_frame(8'h51, lba_lat[31:24],
                                       lba_lat[23:16], lba_lat[15:8], lba_lat[7:0]);
            endcase
            SC_CMD18: case (frame_idx)
                3'd0:    frame_byte = 8'h52;   // READ_MULTIPLE_BLOCK
                3'd1:    frame_byte = lba_lat[31:24];
                3'd2:    frame_byte = lba_lat[23:16];
                3'd3:    frame_byte = lba_lat[15: 8];
                3'd4:    frame_byte = lba_lat[ 7: 0];
                default: frame_byte = crc7_frame(8'h52, lba_lat[31:24],
                                       lba_lat[23:16], lba_lat[15:8], lba_lat[7:0]);
            endcase
            SC_CMD24: case (frame_idx)
                3'd0:    frame_byte = 8'h58;   // WRITE_BLOCK
                3'd1:    frame_byte = lba_lat[31:24];
                3'd2:    frame_byte = lba_lat[23:16];
                3'd3:    frame_byte = lba_lat[15: 8];
                3'd4:    frame_byte = lba_lat[ 7: 0];
                default: frame_byte = crc7_frame(8'h58, lba_lat[31:24],
                                       lba_lat[23:16], lba_lat[15:8], lba_lat[7:0]);
            endcase
            SC_CMD25: case (frame_idx)
                3'd0:    frame_byte = 8'h59;   // WRITE_MULTIPLE_BLOCK
                3'd1:    frame_byte = lba_lat[31:24];
                3'd2:    frame_byte = lba_lat[23:16];
                3'd3:    frame_byte = lba_lat[15: 8];
                3'd4:    frame_byte = lba_lat[ 7: 0];
                default: frame_byte = crc7_frame(8'h59, lba_lat[31:24],
                                       lba_lat[23:16], lba_lat[15:8], lba_lat[7:0]);
            endcase
            SC_CMD12: case (frame_idx)
                // STOP_TRANSMISSION — argument is "stuff bits", sent as
                // all-zero by convention. This command's CRC7+stop byte
                // was left at the old dummy placeholder (8'h01) when
                // every other command was fixed for real CRC7 (this
                // card enforces CRC7 unconditionally, see crc7_frame()
                // comment) — found 2026-07-24 via fpga_top_sdmin
                // per-attempt telemetry: CMD12 consistently got
                // R1=0x04 (ILLEGAL_COMMAND) after an otherwise fully
                // correct, CRC32-verified transfer.
                3'd0:    frame_byte = 8'h4C;   // STOP_TRANSMISSION
                3'd1:    frame_byte = 8'h00;
                3'd2:    frame_byte = 8'h00;
                3'd3:    frame_byte = 8'h00;
                3'd4:    frame_byte = 8'h00;
                default: frame_byte = crc7_frame(8'h4C, 8'h00, 8'h00, 8'h00, 8'h00);
            endcase
            default: frame_byte = 8'hFF;
        endcase
    end

    // Timeouts (generous — card latencies are bounded in the SD spec
    // but vary widely across cards; we pick numbers that survive the
    // slowest test cards but bound host hang).
    //
    // POLL_TIMEOUT bumped 8192->1000000 (2026-07-15, HW bring-up): the
    // OLD value gave only ~5.2ms of R1-wait budget at this design's
    // 12.5MHz fast-mode SPI rate (8192 poll bytes * 8 bits * 2*
    // SPI_FAST_HALF core cycles/bit / 100MHz core clock), while
    // BUSY_TIMEOUT already budgets ~640ms for the PRECEDING write's
    // busy-release.  Real-hardware bring-up of the SD-provisioning
    // bitstream (rtl/soc/sd_provision_top.v) showed a reproducible
    // failure shape: a CMD25 multi-block write completes cleanly
    // (busy-release seen, data CRC32-verified correct), but the VERY
    // NEXT command -- whether a CMD18 read-back verify or another
    // CMD25 batch -- gets "R1 timeout (card silent)" well within a
    // few ms.  This matches a well-documented real-world SD card
    // quirk: some cards do post-write internal housekeeping (wear
    // leveling / block remapping) that can briefly re-busy the card
    // for tens of ms AFTER it has already reported "not busy" via the
    // SPI busy-token protocol, before it's ready to accept a genuinely
    // NEW command's R1 handshake.  Matching POLL_TIMEOUT's budget to
    // BUSY_TIMEOUT's order of magnitude gives real cards room for that
    // housekeeping instead of bounding it to 1/128th of the write-busy
    // budget for no protocol-mandated reason.
    localparam [19:0] POLL_TIMEOUT  = 20'd1000000;   // R1
    localparam [19:0] TOKEN_TIMEOUT = 20'd500000;    // 0xFE token
    localparam [19:0] BUSY_TIMEOUT  = 20'd1000000;   // write busy

    // ══════════════════════════════════════════════════════════════════
    // GLOBAL PER-REQUEST WATCHDOG  (the bounded-response guarantee)
    // ══════════════════════════════════════════════════════════════════
    // Armed when `go` is accepted in S_IDLE; ticks every cycle the
    // request is live regardless of what any state is waiting for;
    // cleared in S_DONE / S_ERROR.  On expiry the FSM is forced to
    // S_ERROR from WHATEVER state it is in, which deasserts `busy` and
    // pulses `done` with err_cause = ERR_WDOG.
    //
    // Structure: a free-running prescaler feeds a tick counter, so the
    // wide budget fits a 22-bit compare instead of a 34-bit one.  The
    // prescaler is itself ungated — "prescaled" is not "conditional".
    //
    // WHY THE BUDGET SCALES WITH block_count
    // ──────────────────────────────────────
    // A flat constant cannot work here, and the measured numbers say so.
    // From tb_sd_ctrl (which uses the same FAST_HALF=4 SPI divider the
    // shipping build does, so its cycle counts are the real ones — and
    // they are clock-INVARIANT, since SCK is derived from clk):
    //     CMD17, 1 block  ......  53,146 cycles
    //     CMD18, 4 blocks ..... 211,288 cycles  → 52,822 cycles/block
    //     CMD17 R1 timeout .... 70,000,572 cycles (POLL_TIMEOUT=1e6
    //                           poll bytes at ~70 cycles/byte)
    // block_count is caller data: boot_fsm.v issues ONE CMD18 for all
    // NUM_SECTORS=2048 ROM sectors (~108M cycles, ~1.08 s at 100 MHz),
    // and a SCSI READ(10) carries a 16-bit block count straight from the
    // CDB, so 65535 blocks is reachable from software.  A flat budget
    // tight enough to be worth calling a timeout would kill the ROM load
    // outright, while a flat budget large enough for 65535 blocks (~34 s
    // of legitimate transfer) would be that much dead air on a 1-block
    // request.  So: fixed base + per-block allowance.  Note the base has
    // since grown to 10 s in its own right — that does not retire this
    // argument, it just moves it: at 65535 blocks the per-block term is
    // still 8.6x the base.
    //
    //   REQ_WDOG_TICK_LOG2   12  → 4096 cycles per tick.
    //   REQ_WDOG_BLK_SHIFT    5  → 32 ticks = 131,072 cycles/block, 2.48x
    //                              the measured 52,822.  Applied as a
    //                              shift, so no multiplier.  UNCHANGED by
    //                              the 2026-08-02 base bump: per-block
    //                              scaling exists so a large transfer gets
    //                              proportionally more time, and that
    //                              reasoning is independent of the floor.
    //   REQ_WDOG_BASE_TICKS  245760 → 1,006,632,960 cycles ≈ 10.07 s at
    //                              100 MHz / 5.03 s at 200 MHz.
    //
    // WHAT THE BASE BUYS, AND WHAT IT DOES NOT  (why 10 s, not 1 s)
    // ─────────────────────────────────────────────────────────────
    // This watchdog is NOT what frees the CPU in the real deadlock.  The
    // CPU is already released by the PERIPHERAL-BUS watchdog at ~335 ms,
    // which SLVERRs the stuck access and the m68k takes vector 2.  What
    // sd_ctrl's watchdog does is unstick the SCSI TARGET, so the drive
    // becomes usable again instead of being wedged for the rest of the
    // session.  So the base is not a CPU-hang budget — it is "how long
    // is a card allowed to look dead before we give up on the request".
    // At 10 s the OS faults early (335 ms) and then finds the target
    // unavailable for the remaining ~9.7 s.  That is the accepted
    // trade: bounded beats infinite, and a base short enough to feel
    // snappy is a base that starts killing slow-but-healthy cards.
    //
    // The floor on the base: it must sit ABOVE the largest legitimate
    // fixed-cost phase, else it preempts a more specific diagnosis.
    //   * POLL_TIMEOUT's R1 wait alone .......  70,000,572 cycles → 14.4x
    //   * a full R1 poll AND a full write-busy
    //     poll in the same command ........... 140,001,144 cycles →  7.2x
    // Both are now comfortably covered, so ERR_R1_TO *and* ERR_BUSY_TO
    // both survive as reported causes.  (At the previous 1 s base only
    // the first did — the doubled case degraded to ERR_WDOG.)
    //
    // Sanity at the extremes:
    //   1 block    → 245,792 ticks ≈ 1.007G cycles vs 53k legitimate
    //   2048 blocks→ 311,296 ticks ≈ 1.275G cycles vs 108M legit (11.9x)
    //   65535 blks → 2,342,880 ticks ≈ 9.60G cycles vs 3.44G legit (2.8x)
    // The last one is ~96 s at 100 MHz — long, but it is a 33 MB request
    // that legitimately takes ~34 s, and bounded beats infinite.
    //
    // COUNTER WIDTH: wdog_ticks / wdog_limit are 22 bits (max 4,194,303).
    // The largest limit constructible is the 65535-block one above,
    // 2,342,880 ticks — 1.79x of headroom.  wdog_ticks never runs past
    // wdog_limit (the compare forces S_ERROR, which clears it), so 22
    // bits is sufficient for both.  Re-do this sum before raising either
    // REQ_WDOG_BASE_TICKS or REQ_WDOG_BLK_SHIFT again.
    //
    // The three constants are module parameters — see the header at the
    // top of the file for why (unit-tb time-scaling only).

    reg  [REQ_WDOG_TICK_LOG2-1:0] wdog_pre;
    reg  [21:0] wdog_ticks;
    reg  [21:0] wdog_limit;
    reg         wdog_armed;

    // Same block_count normalisation S_IDLE applies to block_count_lat,
    // evaluated combinationally so the limit can be latched on the very
    // cycle `go` is sampled.
    wire [15:0] wdog_blocks = ((cmd_type == CT_CMD17) || (cmd_type == CT_CMD24))
                                ? 16'd1
                                : ((block_count == 16'd0) ? 16'd1 : block_count);
    // blocks << REQ_WDOG_BLK_SHIFT, done as a shift by a constant (pure
    // wiring, no multiplier).  Written as a zero-extend-then-shift rather
    // than a {1'b0, blocks, {SHIFT{1'b0}}} concat so the expression stays
    // 22 bits wide if REQ_WDOG_BLK_SHIFT is ever overridden.  Max
    // 65535<<5 = 2,097,120, + base 245,760 = 2,342,880, inside 22 bits.
    wire [21:0] wdog_blk_ticks  = {6'd0, wdog_blocks} << REQ_WDOG_BLK_SHIFT;
    wire [21:0] wdog_limit_next = REQ_WDOG_BASE_TICKS + wdog_blk_ticks;
    wire        wdog_go_ok      = go && (cmd_type >= CT_CMD17) &&
                                        (cmd_type <= CT_CMD25);
    // REQ_WDOG_ENABLE gates ONLY the escape, not the counter — see the
    // parameter comment at the top of the file.  Keeping the counter live
    // when disabled means the arm/clear logic below stays on exactly one
    // code path, so turning the knob cannot perturb anything else.
    assign      wdog_expired    = (REQ_WDOG_ENABLE != 0) &&
                                  wdog_armed && (wdog_ticks >= wdog_limit);

    always @(posedge clk) begin
        if (local_rst) begin
            wdog_armed <= 1'b0;
            wdog_pre   <= {REQ_WDOG_TICK_LOG2{1'b0}};
            wdog_ticks <= 22'd0;
            wdog_limit <= 22'd0;
        end else if ((st == S_DONE) || (st == S_ERROR)) begin
            // The ONLY clear points, per the contract comment above.
            wdog_armed <= 1'b0;
            wdog_pre   <= {REQ_WDOG_TICK_LOG2{1'b0}};
            wdog_ticks <= 22'd0;
        end else if (st == S_IDLE) begin
            // Arm on an accepted `go`.  A rejected one (CT_IDLE or an
            // out-of-range cmd_type) completes inside S_IDLE without
            // ever leaving it, so it must NOT arm.
            wdog_armed <= wdog_go_ok;
            wdog_pre   <= {REQ_WDOG_TICK_LOG2{1'b0}};
            wdog_ticks <= 22'd0;
            if (wdog_go_ok) wdog_limit <= wdog_limit_next;
        end else if (wdog_rearm) begin
            // The request is being force-closed (see the S_AB_* block).
            // Give the close sequence ONE fresh single-block budget: it
            // is a fixed, short byte sequence, so it does not need — and
            // must not be given — the per-block allowance of the request
            // it is aborting.  `wdog_rearmed_q` in the main FSM makes
            // this at most one re-arm per request, so the total bounded-
            // response guarantee is (original budget + one-block budget),
            // still finite.  This is NOT the fix for the abandoned-write
            // bug; the fix is the close sequence itself.  This only stops
            // the level-held expiry from cancelling that sequence.
            wdog_pre   <= {REQ_WDOG_TICK_LOG2{1'b0}};
            wdog_ticks <= 22'd0;
            wdog_limit <= REQ_WDOG_BASE_TICKS +
                          (22'd1 << REQ_WDOG_BLK_SHIFT);
        end else begin
            // Unconditional: no `bdone`, no `rd_ready`, no state test.
            wdog_pre <= wdog_pre + {{(REQ_WDOG_TICK_LOG2-1){1'b0}}, 1'b1};
            if (&wdog_pre) wdog_ticks <= wdog_ticks + 22'd1;
        end
    end

    // ──────────────────────────────────────────────────────────────────
    // Sequential
    // ──────────────────────────────────────────────────────────────────
    always @(posedge clk) begin
        if (local_rst) begin
            st              <= S_IDLE;
            cur_cmd         <= SC_NONE;
            frame_idx       <= 3'd0;
            byte_idx        <= 10'd0;
            block_idx       <= 16'd0;
            poll_cnt        <= 20'd0;
            token_cnt       <= 20'd0;
            busy_cnt        <= 20'd0;
            cmd_type_lat    <= CT_IDLE;
            lba_lat         <= 32'd0;
            block_count_lat <= 16'd0;

            bgo             <= 1'b0;
            bt              <= 8'hFF;

            rd_valid        <= 1'b0;
            rd_data         <= 8'h00;
            wr_ready        <= 1'b0;
            wr_valid        <= 1'b0;

            busy            <= 1'b0;
            done            <= 1'b0;
            error           <= 1'b0;
            err_cause       <= ERR_NONE;
            rd_crc_calc     <= 16'h0000;
            rd_crc_recv_hi  <= 8'h00;
            rd_crc_recv_latched <= 16'h0000;
            dbg_last_real_r1 <= 8'h00;
            dbg_last_write_resp <= 8'hFF;
            dbg_cur_cmd      <= 4'h0;
            dbg_lba_lat      <= 32'h0;
            dbg_last_crc7_sent <= 8'h00;
            dbg_last_poll_cnt  <= 8'h00;
            wr_crc_calc     <= 16'h0000;
            wr_resp_seen    <= 1'b0;
            crc_retry_cnt   <= 3'd0;
            blk_flush_idx   <= 9'd0;
            flush_pace_cnt  <= 6'd0;
            rd_bank_valid   <= 2'b00;
            rd_fill_bank    <= 1'b0;
            rd_drain_bank   <= 1'b0;
            wdog_rearm      <= 1'b0;
            wdog_rearmed_q  <= 1'b0;
        end else begin
            // Default one-cycle pulses
            bgo      <= 1'b0;
            rd_valid <= 1'b0;
            wr_ready <= 1'b0;
            done     <= 1'b0;
            wdog_rearm <= 1'b0;

`ifdef VERILATOR
            if (READ_PIPELINE && st == S_RD_WAIT && bdone &&
                rd_bank_valid[rd_fill_bank])
                $fatal(1, "sd_ctrl: overwriting an undrained read bank");
            if (READ_PIPELINE && st == S_DONE && rd_bank_valid != 2'b00)
                $fatal(1, "sd_ctrl: successful completion before read drain");
`endif

            // Independent consumer: only a CRC-validated sector is visible.
            // A bank stays owned by the consumer until its last byte leaves;
            // the SPI engine cannot start another sector in that bank yet.
            if (READ_PIPELINE && busy && st != S_ERROR &&
                rd_bank_valid[rd_drain_bank] && rd_ready) begin
                if (flush_pace_cnt == FLUSH_PACE_CYCLES - 1) begin
                    flush_pace_cnt <= 6'd0;
                    rd_data <= blk_buf[{rd_drain_bank, blk_flush_idx}];
                    rd_valid <= 1'b1;
                    if (blk_flush_idx == 9'd511) begin
                        blk_flush_idx <= 9'd0;
                        rd_bank_valid[rd_drain_bank] <= 1'b0;
                        rd_drain_bank <= ~rd_drain_bank;
                    end else begin
                        blk_flush_idx <= blk_flush_idx + 9'd1;
                    end
                end else begin
                    flush_pace_cnt <= flush_pace_cnt + 6'd1;
                end
            end

            // Sample `go` only when idle; reject if cmd_type is unknown.
            case (st)
                S_IDLE: begin
                    wr_valid <= 1'b0;
                    if (go) begin
                        rd_bank_valid <= 2'b00;
                        rd_fill_bank <= 1'b0;
                        rd_drain_bank <= 1'b0;
                        blk_flush_idx <= 9'd0;
                        flush_pace_cnt <= 6'd0;
                        cmd_type_lat    <= cmd_type;
                        lba_lat         <= lba;
                        // Force block_count to 1 for single-block ops.
                        if (cmd_type == CT_CMD17 || cmd_type == CT_CMD24) begin
                            block_count_lat <= 16'd1;
                        end else begin
                            block_count_lat <= (block_count == 16'd0)
                                ? 16'd1 : block_count;
                        end
                        block_idx <= 16'd0;
                        byte_idx  <= 10'd0;
                        frame_idx <= 3'd0;
                        poll_cnt  <= 20'd0;
                        token_cnt <= 20'd0;
                        busy_cnt  <= 20'd0;
                        crc_retry_cnt <= 3'd0;
                        error     <= 1'b0;
                        err_cause <= ERR_NONE;
                        wdog_rearmed_q <= 1'b0;
                        busy      <= 1'b1;
                        // Dispatch on command type.
                        case (cmd_type)
                            CT_CMD17: begin cur_cmd <= SC_CMD17; st <= S_CMD_PRE_S; end
                            CT_CMD18: begin cur_cmd <= SC_CMD18; st <= S_CMD_PRE_S; end
                            CT_CMD24: begin cur_cmd <= SC_CMD24; st <= S_CMD_PRE_S; end
                            CT_CMD25: begin
                                cur_cmd <= MULTI_WRITE_AS_CMD24 ? SC_CMD24 : SC_CMD25;
                                st <= S_CMD_PRE_S;
                            end
                            default: begin
                                error     <= 1'b1;
                                err_cause <= ERR_UNK_CMD;
                                busy      <= 1'b0;
                                done      <= 1'b1;
                                st        <= S_IDLE;
                            end
                        endcase
                    end
                end

                // ────────── pre-command dummy clock (see localparam comment) ──────────
                S_CMD_PRE_S: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= S_CMD_PRE_W;
                    end
                end
                S_CMD_PRE_W: begin
                    if (bdone) st <= S_CMD_SEND;
                end

                // ────────── 6-byte CMD frame send ──────────
                S_CMD_SEND: begin
                    if (bst == BS_IDLE) begin
                        bt  <= frame_byte;
                        bgo <= 1'b1;
                        st  <= S_CMD_WAIT;
                        // Debug: capture the actual CRC7+stop byte that
                        // goes out on frame_idx==5, to directly confirm
                        // crc7_frame() is really computing what gets
                        // transmitted (rules out a synthesis-level
                        // function-evaluation bug vs. a genuine
                        // electrical/framing issue elsewhere).
                        if (frame_idx == 3'd5) dbg_last_crc7_sent <= frame_byte;
                    end
                end
                S_CMD_WAIT: begin
                    if (bdone) begin
                        if (frame_idx == 3'd5) begin
                            frame_idx <= 3'd0;
                            poll_cnt  <= 20'd0;
                            // See S_CMD12_STUFF_S localparam comment.
                            if (cur_cmd == SC_CMD12) st <= S_CMD12_STUFF_S;
                            else                     st <= S_R1_SEND;
                        end else begin
                            frame_idx <= frame_idx + 3'd1;
                            st        <= S_CMD_SEND;
                        end
                    end
                end

                // ────────── CMD12-only: discard one undefined stuff byte ──
                S_CMD12_STUFF_S: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= S_CMD12_STUFF_W;
                    end
                end
                S_CMD12_STUFF_W: begin
                    if (bdone) st <= S_R1_SEND;
                end

                // ────────── R1 poll ──────────
                S_R1_SEND: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= S_R1_WAIT;
                    end
                end
                S_R1_WAIT: begin
                    if (bdone) begin
                        if (br[7] == 1'b0) begin
                            // Real R1. Latched here (not read live from
                            // `br` elsewhere) for the same reason
                            // dbg_rd_crc_recv needed its own latch: `br`
                            // is a shared, multi-purpose register that
                            // later traffic (CMD12 abort, next command)
                            // reuses immediately afterward.
                            dbg_last_real_r1 <= br;
                            dbg_last_poll_cnt <= poll_cnt[7:0];
                            // Latch cur_cmd/lba_lat too — by the time a
                            // caller reads the LIVE wires after settling
                            // into S_ERROR->S_DONE->S_IDLE, cur_cmd has
                            // already reset to SC_NONE, making it
                            // useless for "what command was this R1
                            // actually for" diagnosis.
                            dbg_cur_cmd <= cur_cmd;
                            dbg_lba_lat <= lba_lat;
                            if (br == 8'h00) begin
                                // Accepted — branch by command.
                                case (cur_cmd)
                                    SC_CMD17, SC_CMD18: begin
                                        token_cnt <= 20'd0;
                                        byte_idx  <= 10'd0;
                                        st        <= S_TOK_SEND;
                                    end
                                    SC_CMD24, SC_CMD25: begin
                                        byte_idx <= 10'd0;
                                        st       <= S_W_GAP_S;
                                    end
                                    SC_CMD12: begin
                                        // The mandatory stuff byte is now
                                        // explicitly discarded in
                                        // S_CMD12_STUFF_S/_W before this
                                        // poll loop ever starts, so this
                                        // really is CMD12's real R1.
                                        st <= READ_PIPELINE ? S_RD_DRAIN : S_DONE;
                                    end
                                    default: begin
                                        error     <= 1'b1;
                                        err_cause <= ERR_UNK_CMD;
                                        st        <= S_ERROR;
                                    end
                                endcase
                            end else begin
                                // Any low-bit set in R1 ⇒ rejected.
                                error     <= 1'b1;
                                err_cause <= ERR_R1_BAD;
                                st        <= S_ERROR;
                            end
                        end else begin
                            // Still 0xFF — keep polling.
                            if (poll_cnt >= POLL_TIMEOUT) begin
                                error     <= 1'b1;
                                err_cause <= ERR_R1_TO;
                                st        <= S_ERROR;
                            end else begin
                                poll_cnt <= poll_cnt + 20'd1;
                                st       <= S_R1_SEND;
                            end
                        end
                    end
                end

                // ────────── READ PATH ──────────
                // Single-bank consumer pacing: rd_ready low pauses the stream before
                // the next SPI byte is issued (SPI is host-clocked, so
                // stalling between bytes — or before the next block's
                // data token — is always legal for the card).  Gated at
                // S_TOK_SEND too because token polling clocks the card:
                // once the 0xFE token arrives the data block follows on
                // subsequent clocks, so we must not hunt for it while
                // the consumer has no room.  token_cnt only advances on
                // completed polls, so the timeout is naturally frozen
                // while paused.  Callers that tie rd_ready high (boot
                // ROM load, JTAG/bulk writers) see no change. With READ_PIPELINE,
                // reserve a free bank instead: consumer stalls can fill two
                // banks, but never overwrite one still awaiting delivery.
                S_TOK_SEND: begin
                    if (bst == BS_IDLE &&
                        (READ_PIPELINE ? !rd_bank_valid[rd_fill_bank] : rd_ready)) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= S_TOK_WAIT;
                    end
                end
                S_TOK_WAIT: begin
                    if (bdone) begin
                        if (br == 8'hFE) begin
                            byte_idx    <= 10'd0;
                            rd_crc_calc <= 16'h0000;   // fresh per-block CRC
                            st          <= S_RD_SEND;
                        end else if (token_cnt >= TOKEN_TIMEOUT) begin
                            error     <= 1'b1;
                            err_cause <= ERR_TOK_TO;
                            st        <= S_ERROR;
                        end else begin
                            token_cnt <= token_cnt + 20'd1;
                            st        <= S_TOK_SEND;
                        end
                    end
                end

                // Receive 512 bytes into the internal staging buffer
                // (blk_buf) — NOT rd_valid/rd_data directly.  No rd_ready
                // gating here: the destination bank was reserved before its token,
                // not the caller's structure, so the caller's readiness
                // is irrelevant until the flush stage below actually
                // hands bytes over.  SPI is still fully host-paced by
                // the bgo/bdone sub-FSM as always.
                S_RD_SEND: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= S_RD_WAIT;
                    end
                end
                S_RD_WAIT: begin
                    if (bdone) begin
                        blk_buf[{(READ_PIPELINE ? rd_fill_bank : 1'b0), byte_idx[8:0]}] <= br;
                        rd_crc_calc       <= crc16_step(rd_crc_calc, br);
                        if (byte_idx == 10'd511) begin
                            byte_idx <= 10'd0;
                            st       <= S_CRC_SEND;
                        end else begin
                            byte_idx <= byte_idx + 10'd1;
                            st       <= S_RD_SEND;
                        end
                    end
                end

                // 2 CRC bytes.  When crc_check_en is low this is bit-
                // exact legacy behaviour (bytes received and discarded);
                // when high, the first byte is stashed and the second is
                // compared against rd_crc_calc below.
                S_CRC_SEND: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= S_CRC_WAIT;
                    end
                end
                S_CRC_WAIT: begin
                    if (bdone) begin
                        if (byte_idx == 10'd1) begin
                            byte_idx <= 10'd0;
                            // Latch the received CRC HERE, before any
                            // possible CMD12-abort traffic reuses `br`
                            // for its own R1 byte — `br` is a shared,
                            // multi-purpose register, not exclusive to
                            // this comparison.
                            rd_crc_recv_latched <= {rd_crc_recv_hi, br};
                            if (crc_check_en &&
                                ({rd_crc_recv_hi, br} != rd_crc_calc)) begin
                                // CRC mismatch — no byte of THIS sector has
                                // reached the caller. Earlier validated banks
                                // may still be draining concurrently.
                                if (cur_cmd == SC_CMD17 &&
                                    crc_retry_cnt < CRC_RETRY_MAX) begin
                                    // Single-block: retry the WHOLE
                                    // command from scratch (fresh frame,
                                    // R1, token, data, CRC) — the SD
                                    // protocol has no notion of "replay
                                    // this block" mid-stream, but CMD17
                                    // is a brand-new command each time,
                                    // so this is straightforward and
                                    // side-effect-free.
                                    crc_retry_cnt <= crc_retry_cnt + 3'd1;
                                    frame_idx     <= 3'd0;
                                    poll_cnt      <= 20'd0;
                                    token_cnt     <= 20'd0;
                                    st            <= S_CMD_PRE_S;
                                end else begin
                                    // CMD18 (can't replay a block
                                    // mid-stream — must abort) or CMD17
                                    // retries exhausted: cleanly stop the
                                    // card and surface the error, exactly
                                    // like the existing ERR_TOK_TO path.
                                    // The caller's OWN existing error
                                    // handling (already wired to SCSI
                                    // CHECK CONDITION / MEDIUM ERROR)
                                    // drives any further OS-level retry.
                                    error     <= 1'b1;
                                    err_cause <= ERR_CRC_BAD;
                                    if (cur_cmd == SC_CMD18) begin
                                        cur_cmd   <= SC_CMD12;
                                        frame_idx <= 3'd0;
                                        block_idx <= 16'd0;
                                        // Real-HW finding 2026-07-24: this
                                        // jumped straight to S_CMD_SEND,
                                        // skipping the pre-command dummy
                                        // clock (S_CMD_PRE_S) every OTHER
                                        // command dispatch goes through —
                                        // exactly the "second command in a
                                        // back-to-back pair" scenario that
                                        // state exists to protect (see its
                                        // own header comment, added
                                        // 2026-07-15 for the SAME class of
                                        // bug on CMD25). CMD12 always
                                        // follows immediately after a
                                        // CMD18 stream, aborted or not —
                                        // this asymmetry left the card
                                        // potentially unable to cleanly
                                        // decode CMD12, corrupting it and
                                        // leaving the card confused for
                                        // whatever command comes next.
                                        st        <= S_CMD_PRE_S;
                                    end else begin
                                        st <= S_ERROR;
                                    end
                                end
                            end else begin
                                // CRC OK (or checking disabled) — release
                                // the validated block to the caller.
                                if (READ_PIPELINE) begin
                                    rd_bank_valid[rd_fill_bank] <= 1'b1;
                                    rd_fill_bank <= ~rd_fill_bank;
                                    if (cur_cmd == SC_CMD18) begin
                                        if (block_idx + 16'd1 >= block_count_lat) begin
                                            cur_cmd <= SC_CMD12;
                                            frame_idx <= 3'd0;
                                            block_idx <= 16'd0;
                                            st <= S_CMD_PRE_S;
                                        end else begin
                                            block_idx <= block_idx + 16'd1;
                                            token_cnt <= 20'd0;
                                            st <= S_TOK_SEND;
                                        end
                                    end else begin
                                        st <= S_RD_DRAIN;
                                    end
                                end else begin
                                    blk_flush_idx  <= 9'd0;
                                    flush_pace_cnt <= 6'd0;
                                    st             <= S_RD_FLUSH_S;
                                end
                            end
                        end else begin
                            rd_crc_recv_hi <= br;
                            byte_idx       <= 10'd1;
                            st             <= S_CRC_SEND;
                        end
                    end
                end

                // Replay the validated block to rd_valid/rd_data, gated
                // on rd_ready (consumer back-pressure) and paced by a
                // pure LOCAL delay counter — deliberately NOT the
                // bgo/bt/bst byte sub-FSM (i.e. no spi_cmd_valid/ready
                // activity at all here). Using the real SPI byte-clock
                // for this pacing was tried and is WRONG: for a CMD18
                // multi-block run the card streams block N+1 immediately
                // and continuously behind block N over the SAME physical
                // clock edges — any extra dummy SPI byte-clocks issued
                // here during a "replay" would silently consume real
                // bytes of the NEXT block off the wire and discard them,
                // desyncing the whole multi-block gather permanently
                // (caught by test_cmd18_multi_read regressing during
                // development). FLUSH_PACE_CYCLES only needs to clear
                // the sd_scsi_bridge CDC's toggle-sync margin (2-3
                // pb_clk cycles, ~12 core_clk cycles at 200 MHz). The
                // caller selects a pace appropriate to its clock ratio;
                // this delay is not a substitute for a FIFO handshake.
                S_RD_FLUSH_S: begin
                    if (rd_ready) begin
                        if (flush_pace_cnt == FLUSH_PACE_CYCLES - 1) begin
                            flush_pace_cnt <= 6'd0;
                            rd_data        <= blk_buf[blk_flush_idx];
                            rd_valid       <= 1'b1;
                            if (blk_flush_idx == 9'd511) begin
                                // Decide what comes next — same logic the
                                // legacy inline S_CRC_WAIT branch used.
                                if (cur_cmd == SC_CMD18) begin
                                    if (block_idx + 16'd1 >= block_count_lat) begin
                                        cur_cmd   <= SC_CMD12;
                                        frame_idx <= 3'd0;
                                        block_idx <= 16'd0;
                                        // Same fix as the error-abort
                                        // CMD12 dispatch above (see its
                                        // comment) — this is the OTHER
                                        // site that skipped the required
                                        // pre-command dummy clock before
                                        // CMD12, on the successful
                                        // (non-error) completion path.
                                        st        <= S_CMD_PRE_S;
                                    end else begin
                                        block_idx <= block_idx + 16'd1;
                                        token_cnt <= 20'd0;
                                        st        <= S_TOK_SEND;
                                    end
                                end else begin
                                    st <= S_DONE;
                                end
                            end else begin
                                blk_flush_idx <= blk_flush_idx + 9'd1;
                            end
                        end else begin
                            flush_pace_cnt <= flush_pace_cnt + 6'd1;
                        end
                    end
                end

                // ────────── WRITE PATH ──────────
                // 0xFF gap byte after R1.
                S_W_GAP_S: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= S_W_GAP_W;
                    end
                end
                S_W_GAP_W: begin
                    if (bdone) st <= S_W_TOK_S;
                end

                // Data token: 0xFE for CMD24 single-block, 0xFC for
                // CMD25 multi-block.
                S_W_TOK_S: begin
                    if (bst == BS_IDLE) begin
                        bt  <= (cur_cmd == SC_CMD25) ? 8'hFC : 8'hFE;
                        bgo <= 1'b1;
                        st  <= S_W_TOK_W;
                    end
                end
                S_W_TOK_W: begin
                    if (bdone) begin
                        byte_idx    <= 10'd0;
                        wr_valid    <= 1'b1;   // data-block phase begins
                        wr_crc_calc <= 16'h0000;
                        st          <= S_W_DATA_S;
                    end
                end

                // Stream 512 bytes from wr_data (producer-paced).
                // `wr_avail` is the producer's "I have a byte" level.
                // Without it this state issued the SPI byte whatever the
                // producer was doing, so a stalled producer got the
                // PREVIOUS block's stale content written to the card at
                // the correct LBA with GOOD status.  Pausing here is
                // always legal — SPI is host-clocked — and is bounded by
                // the global request watchdog.
                S_W_DATA_S: begin
                    if ((bst == BS_IDLE) && wr_avail) begin
                        bt          <= wr_data;
                        wr_ready    <= 1'b1;          // consumed one byte
                        wr_crc_calc <= crc16_step(wr_crc_calc, wr_data);
                        bgo         <= 1'b1;
                        st          <= S_W_DATA_W;
                    end
                end
                S_W_DATA_W: begin
                    if (bdone) begin
                        if (byte_idx == 10'd511) begin
                            byte_idx <= 10'd0;
                            wr_valid <= 1'b0;
                            st       <= S_W_DCRC_S;
                        end else begin
                            byte_idx <= byte_idx + 10'd1;
                            st       <= S_W_DATA_S;
                        end
                    end
                end

                // 2 real CRC16 bytes, high byte first — computed over the
                // 512 bytes just transmitted (was: 2 dummy 0x00 bytes).
                // Sending a real CRC is harmless whether or not the card
                // is actually validating it (CRC_ON_OFF via CMD59); it
                // only matters once a caller enables checking on that
                // card, at which point a dummy 0x00 would make every
                // write fail with ERR_DR_BAD.
                S_W_DCRC_S: begin
                    if (bst == BS_IDLE) begin
                        bt  <= (byte_idx == 10'd0) ? wr_crc_calc[15:8]
                                                   : wr_crc_calc[7:0];
                        bgo <= 1'b1;
                        st  <= S_W_DCRC_W;
                    end
                end
                S_W_DCRC_W: begin
                    if (bdone) begin
                        if (byte_idx == 10'd1) begin
                            byte_idx <= 10'd0;
                            poll_cnt <= 20'd0;
                            dbg_last_write_resp <= 8'hFF;
                            wr_resp_seen <= 1'b0;
                            st       <= S_W_RESP_S;
                        end else begin
                            byte_idx <= 10'd1;
                            st       <= S_W_DCRC_S;
                        end
                    end
                end

                // Data response byte — format is xxx0_<s2,s1,s0>_1 with
                // "010" in the status field = accepted.
                S_W_RESP_S: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= S_W_RESP_W;
                    end
                end
                S_W_RESP_W: begin
                    if (bdone) begin
                        // Preserve the first byte in which the card drove
                        // anything other than idle.  Overwriting on every
                        // poll made a malformed token or early busy look like
                        // total silence once the final poll returned 0xFF.
                        if (!wr_resp_seen && br != 8'hFF) begin
                            dbg_last_write_resp <= br;
                            wr_resp_seen <= 1'b1;
                        end
                        if (br[4] == 1'b0 && br[0] == 1'b1) begin
                            if (br[3:1] == 3'b010) begin
                                busy_cnt <= 20'd0;
                                st       <= S_W_BUSY_S;
                            end else begin
                                // The card REJECTED the block.  It is
                                // still in its write state (and, for
                                // CMD25, still in sequential-write mode)
                                // — close it before reporting.  See the
                                // S_AB_* block comment.
                                error     <= 1'b1;
                                err_cause <= ERR_DR_BAD;
                                busy_cnt  <= 20'd0;
                                st        <= S_AB_BUSY_S;
                            end
                        end else begin
                            if (poll_cnt >= POLL_TIMEOUT) begin
                                error     <= 1'b1;
                                err_cause <= ERR_DR_BAD;
                                busy_cnt  <= 20'd0;
                                st        <= S_AB_BUSY_S;
                            end else begin
                                poll_cnt <= poll_cnt + 20'd1;
                                st       <= S_W_RESP_S;
                            end
                        end
                    end
                end

                // Busy poll — card drives MISO low while writing.
                S_W_BUSY_S: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= S_W_BUSY_W;
                    end
                end
                S_W_BUSY_W: begin
                    if (bdone) begin
                        if (br != 8'h00) begin
                            // Busy released.  Next block, or end.
                            if (cur_cmd == SC_CMD25) begin
                                if (block_idx + 16'd1 >= block_count_lat) begin
                                    // All blocks sent — issue stop-tran token.
                                    st <= S_W_STOP_S;
                                end else begin
                                    block_idx <= block_idx + 16'd1;
                                    byte_idx  <= 10'd0;
                                    st        <= S_W_TOK_S;
                                end
                            end else if (MULTI_WRITE_AS_CMD24 &&
                                         cmd_type_lat == CT_CMD25 &&
                                         block_idx + 16'd1 < block_count_lat) begin
                                // Close this block completely before issuing
                                // the next command.  The byte stream remains
                                // continuous from the caller's perspective.
                                block_idx <= block_idx + 16'd1;
                                lba_lat   <= lba_lat + 32'd1;
                                frame_idx <= 3'd0;
                                poll_cnt  <= 20'd0;
                                byte_idx  <= 10'd0;
                                st        <= S_CMD_PRE_S;
                            end else begin
                                // CMD24, or the final decomposed block.
                                st <= S_DONE;
                            end
                        end else if (busy_cnt >= BUSY_TIMEOUT) begin
                            // Card never released busy.  For CMD25 it is
                            // still in sequential-write mode; send the
                            // stop-tran before reporting (see S_AB_*).
                            error     <= 1'b1;
                            err_cause <= ERR_BUSY_TO;
                            st        <= S_AB_STOP_S;
                        end else begin
                            busy_cnt <= busy_cnt + 20'd1;
                            st       <= S_W_BUSY_S;
                        end
                    end
                end

                // CMD25 tail: stop-tran token + final busy.
                S_W_STOP_S: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFD;          // stop tran token
                        bgo <= 1'b1;
                        st  <= S_W_STOP_W;
                    end
                end
                S_W_STOP_W: begin
                    if (bdone) begin
                        busy_cnt <= 20'd0;
                        st       <= S_W_FBUSY_S;
                    end
                end
                S_W_FBUSY_S: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= S_W_FBUSY_W;
                    end
                end
                S_W_FBUSY_W: begin
                    if (bdone) begin
                        if (br != 8'h00) begin
                            st <= S_DONE;
                        end else if (busy_cnt >= BUSY_TIMEOUT) begin
                            error     <= 1'b1;
                            err_cause <= ERR_BUSY_TO;
                            st        <= S_ERROR;
                        end else begin
                            busy_cnt <= busy_cnt + 20'd1;
                            st       <= S_W_FBUSY_S;
                        end
                    end
                end

                // ══════════════════════════════════════════════════════
                // GRACEFUL WRITE-SESSION CLOSE  (see the S_AB_* comment
                // in the state-encoding block near the top of the file)
                // ══════════════════════════════════════════════════════
                //
                // Invariants for every state below:
                //   * `error` and `err_cause` are ALREADY latched by the
                //     site that decided to abort and are never touched
                //     here — the caller sees the original cause.
                //   * No error escalation: a timeout inside the close
                //     just moves on to the next step.  The close is
                //     best-effort card hygiene, not a new failure mode.
                //   * `wr_ready` is never pulsed, so the producer is not
                //     asked for bytes it may not have.

                // Pad the open data block out to its 512-byte boundary
                // with 0x00.  byte_idx is the normal data loop's own
                // "next byte to send" counter, normalised by the close-entry
                // logic after any in-flight byte retires.  This must be exact:
                // a surplus byte would become CRC[15:8], shift the remaining
                // close traffic, and could let a torn sector commit.
                S_AB_PAD_S: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'h00;
                        bgo <= 1'b1;
                        st  <= S_AB_PAD_W;
                    end
                end
                S_AB_PAD_W: begin
                    if (bdone) begin
                        if (byte_idx == 10'd511) begin
                            byte_idx <= 10'd0;
                            wr_valid <= 1'b0;
                            st       <= S_AB_CRC_S;
                        end else begin
                            byte_idx <= byte_idx + 10'd1;
                            st       <= S_AB_PAD_S;
                        end
                    end
                end

                // Deliberately INVALID CRC16 (complement of the running
                // value).  We want the card to REJECT this block: it
                // contains real producer bytes followed by pad, i.e. a
                // half-written sector we must not let it commit.  A
                // CRC-rejected block in a multiple-block write also puts
                // the card into "ignore data until stop-tran", which is
                // precisely the next thing we send.
                S_AB_CRC_S: begin
                    if (bst == BS_IDLE) begin
                        bt  <= (byte_idx == 10'd0) ? ~wr_crc_calc[15:8]
                                                   : ~wr_crc_calc[7:0];
                        bgo <= 1'b1;
                        st  <= S_AB_CRC_W;
                    end
                end
                S_AB_CRC_W: begin
                    if (bdone) begin
                        if (byte_idx == 10'd1) begin
                            byte_idx <= 10'd0;
                            st       <= S_AB_RESP_S;
                        end else begin
                            byte_idx <= 10'd1;
                            st       <= S_AB_CRC_S;
                        end
                    end
                end

                // Consume exactly one data-response byte.  Its value is
                // irrelevant — we already have an error to report — but
                // it must come off the wire before the busy poll, or the
                // poll would exit immediately on the (non-zero) response
                // and we would send the stop-tran while the card is
                // still programming.
                S_AB_RESP_S: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= S_AB_RESP_W;
                    end
                end
                S_AB_RESP_W: begin
                    if (bdone) begin
                        busy_cnt <= 20'd0;
                        st       <= S_AB_BUSY_S;
                    end
                end

                // Wait for the card to release MISO.  On timeout, press
                // on to the stop-tran anyway: a card that never releases
                // busy is not going to be helped by giving up earlier,
                // and the stop-tran costs one byte.
                S_AB_BUSY_S: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= S_AB_BUSY_W;
                    end
                end
                S_AB_BUSY_W: begin
                    if (bdone) begin
                        if (br != 8'h00 || busy_cnt >= BUSY_TIMEOUT) begin
                            st <= S_AB_STOP_S;
                        end else begin
                            busy_cnt <= busy_cnt + 20'd1;
                            st       <= S_AB_BUSY_S;
                        end
                    end
                end

                // Stop-tran token — the ONLY thing that takes a CMD25
                // card out of sequential-write mode.  A CMD24 card is
                // already back in command-wait after its block, so it
                // skips straight to the report.
                S_AB_STOP_S: begin
                    if (cur_cmd != SC_CMD25) begin
                        st <= S_ERROR;
                    end else if (bst == BS_IDLE) begin
                        bt  <= 8'hFD;
                        bgo <= 1'b1;
                        st  <= S_AB_STOP_W;
                    end
                end
                S_AB_STOP_W: begin
                    if (bdone) begin
                        busy_cnt <= 20'd0;
                        st       <= S_AB_FBUSY_S;
                    end
                end
                S_AB_FBUSY_S: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= S_AB_FBUSY_W;
                    end
                end
                S_AB_FBUSY_W: begin
                    if (bdone) begin
                        if (br != 8'h00 || busy_cnt >= BUSY_TIMEOUT) begin
                            st <= S_ERROR;
                        end else begin
                            busy_cnt <= busy_cnt + 20'd1;
                            st       <= S_AB_FBUSY_S;
                        end
                    end
                end

                // ────────── Terminal states ──────────
                // Keep busy and the request watchdog armed until the final
                // validated byte is delivered. CMD12 may finish before it.
                S_RD_DRAIN: begin
                    if (rd_bank_valid == 2'b00) st <= S_DONE;
                end

                S_DONE: begin
                    busy    <= 1'b0;
                    done    <= 1'b1;
                    cur_cmd <= SC_NONE;
                    st      <= S_IDLE;
                end

                S_ERROR: begin
                    rd_bank_valid <= 2'b00;
                    busy    <= 1'b0;
                    done    <= 1'b1;
                    cur_cmd <= SC_NONE;
                    st      <= S_IDLE;
                end

                default: st <= S_IDLE;
            endcase

            // ── GLOBAL PER-REQUEST WATCHDOG: expiry ───────────────────
            // Deliberately placed AFTER the state case: these are
            // non-blocking assignments in the same block, so last write
            // wins and this overrides whatever transition the current
            // state chose — from ANY state, with zero per-state edits
            // and no way for a future state to be forgotten.
            //
            // We do NOT try to send CMD12 to stop a multi-block read
            // here (unlike the CRC-abort path, which does).  When the
            // watchdog fires we have no reason to believe the transport
            // will complete another byte, and issuing a command that
            // needs its own R1 would just re-enter an unbounded wait —
            // the opposite of the point.  S_ERROR deasserts `busy` and
            // pulses `done`, which is the whole obligation.
            //
            // 2026-08-09: the ONE exception is a live WRITE session.
            // There, "stop touching the bus" is not a safe way to give
            // up — the CARD is mid-block / mid-sequential-write and will
            // eat the NEXT master's command frame and payload as write
            // data, programming it at its own running pointer (i.e. into
            // the HDD image).  So a watchdog expiry with a write session
            // open detours through the S_AB_* close first, with ONE
            // re-armed single-block watchdog budget so this cannot
            // recurse.  `wdog_rearm` also suppresses the override for
            // the cycle before the counter clears, since wdog_expired is
            // level-held and would otherwise cancel the detour instantly.
            if (wdog_expired && !wdog_rearm && (st != S_IDLE) &&
                (st != S_DONE) && (st != S_ERROR)) begin
                if (!error) begin
                    error     <= 1'b1;
                    err_cause <= ERR_WDOG;
                end
                if (wr_close_needed && !wdog_rearmed_q) begin
                    wdog_rearm     <= 1'b1;
                    wdog_rearmed_q <= 1'b1;
                    busy_cnt       <= 20'd0;
                    bgo            <= 1'b0;
                    wr_ready       <= 1'b0;
                    if (st == S_W_TOK_W) byte_idx <= 10'd0;
                    if (st_wr_incrc)     byte_idx <= 10'd0;
                    st             <= wr_close_entry;
                end else begin
                    st <= S_ERROR;
                end
            end

            // Reset has priority over ordinary write progress, but not over
            // protocol closure.  Once the request reaches a state where the
            // card has accepted the write command, enter the same bounded
            // close used by error exits.  Pre-R1 requests keep advancing
            // until acceptance or failure, and normal/abort tails already in
            // progress are left alone.  local_rst fires after busy falls.
            if (reset_close_pending && wr_close_needed &&
                !wdog_rearmed_q && !wr_close_wait_byte) begin
                wdog_rearm     <= 1'b1;
                wdog_rearmed_q <= 1'b1;
                busy_cnt       <= 20'd0;
                bgo            <= 1'b0;
                wr_ready       <= 1'b0;
                byte_idx       <= wr_close_byte_idx_exact;
                st             <= wr_close_entry_exact;
            end
        end
    end

endmodule

// sd_image_lba_map — 4 MiB SD-image split helper.
//
// Purpose
//   Maps a region-local LBA onto the physical SD-image sector.  The
//   current platform split reserves the first 4 MiB of the image
//   (8192 sectors) for ROM/provisioning.  Raw disk consumers use the
//   same helper with raw_sel=1 to bias their LBAs above that window.
//
// Latency
//   Zero. Pure combinational address translation.
module sd_image_lba_map #(
    parameter [31:0] RAW_BASE_LBA = 32'd8192
) (
    input  wire [31:0] region_lba,
    input  wire        raw_sel,
    output wire [31:0] sd_lba
);
    assign sd_lba = raw_sel ? (region_lba + RAW_BASE_LBA) : region_lba;
endmodule
