// rtl/soc/sonic_trace_ring.v — non-destructive DP83932C SONIC register-access
// trace ring
//
// PURPOSE
// -------
// Record the exact sequence of register accesses the Mac's Ethernet driver
// makes to the on-board SONIC, so it can be diffed against the same sequence
// captured from MAME (which runs this driver successfully).  Same
// methodology, and deliberately the same event-stream shape, as
// rtl/soc/scsi_trace_ring.v -> tools/scsi96_trace_diff.py: emit
// (rw, reg, data) triples and find the FIRST divergence.
//
// WHY THIS EXISTS AT ALL
// ----------------------
// Every remaining Ethernet question is register-level and post-mortem
// snapshots cannot answer any of them:
//
//   * IMR currently reads 0x0000, so no SONIC interrupt can ever assert.
//     Did the driver never write IMR, or write it and have something clear
//     it?  A snapshot cannot tell those apart.  A trace can.
//   * What did the driver actually put in CAM entry 15 (CDP/CDC/CE writes)?
//   * Does CR_TXP stick set after ~8 transmits and silently block every
//     later transmit?  Again: "never cleared" vs "cleared then re-set" is a
//     sequence question.
//
// This module is a PURE OBSERVER on the peripheral_bus <-> q700_eth_sonic
// port.  It never drives a single signal back into that port and never
// issues an access of its own -- the whole point is to measure without
// perturbing.  (SONIC reads are not themselves destructive, unlike the
// 53C96's reg-2/reg-5, but a debug master poking the register file would
// still interleave phantom accesses into the very stream being recorded.)
//
// WHAT IT CAPTURES
// ----------------
// One entry per COMPLETED SONIC beat (the cycle sonic_ack is high with a
// request outstanding), carrying:
//
//   * direction        CPU write vs CPU read
//   * register index   sonic_addr[5:0], the native 16-bit register number
//   * data             the full 16-bit word.  For writes this is what the
//                      CPU presented; for reads it is what q700_eth_sonic
//                      actually returned.
//   * byte strobes     sonic_wstrb[1:0].  NOT decoration: this driver does
//                      half-register writes, and the SONIC decodes the two
//                      halves of CR completely differently (the low byte is
//                      command-decoded through sonic_command_low(), the high
//                      byte handles RRRA/LCAM).  A trace that lost the
//                      strobes could not tell "wrote 0x0002 to CR" from
//                      "wrote 0x00 to CR's high half", which are different
//                      commands.
//
//     Reads record strobes 2'b11.  This is a REPORTED FACT, not a guess:
//     q700_eth_sonic drives sonic_rdata with the whole register regardless
//     of the CPU's access size, and peripheral_bus.v does the byte/word
//     lane selection on its own side (see its SLOT_SONIC read arm), so the
//     CPU's read granularity is genuinely not observable at this port.
//     sonic_wstrb is driven from the WRITE channel's latched state and is
//     meaningless during a read, so sampling it there would record noise.
//
// THE POLL FILTER  (this is the load-bearing design decision)
// -----------------------------------------------------------
// A wedged driver spins forever re-reading a status register.  A plain
// last-N-wins ring would be completely overwritten by that poll loop before
// anyone could read it out, destroying precisely the history we want -- the
// events LEADING INTO the wedge.
//
// WHICH REGISTER DOES A WEDGED SONIC DRIVER SPIN ON?  Two, and both matter:
//
//   * ISR (0x05) -- the interrupt status register.  With IMR = 0 no
//     interrupt can assert, so the driver's service path degenerates into
//     re-reading ISR and finding nothing to do.  This is the primary poll.
//   * CR  (0x00) -- transmit issue waits for CR_TXP to self-clear
//     ("while (CR & TXP);").  If TXP sticks set, this becomes an infinite
//     spin on a constant value, which is exactly the failure hypothesis
//     under investigation.
//
// THE RULE, and why each half of it is the way it is:
//
//   1. WRITES ARE ALWAYS RECORDED.  Never filtered, not even a byte-for-byte
//      repeat of the previous write.  Writes are the commands; "the driver
//      re-issued CR_TXP eight times and the eighth never completed" is the
//      signal we are hunting, and a de-duplicating filter would erase it.
//      This also means the register-programming sequence at driver init --
//      IMR, RCR, DCR, the CAM load, the descriptor pointers -- is captured
//      in full regardless of what values it writes.
//
//   2. A READ IS RECORDED ONLY IF ITS 16-BIT VALUE DIFFERS FROM THE LAST
//      VALUE READ FROM THAT SAME REGISTER INDEX.  Each of the 64 register
//      indices carries its own shadow + valid bit, so a spin on ISR cannot
//      evict CR's shadow (or vice versa) and re-admit itself.
//
//   3. A WRITE TO A REGISTER INVALIDATES THAT REGISTER'S READ SHADOW, so the
//      first read after any write is ALWAYS recorded.  This is not a nicety:
//      almost every SONIC register is write-masked (sonic_reg_mask()), CR is
//      command-decoded rather than stored, and ISR is write-1-to-clear.  The
//      read-back after a write is therefore the only place the trace shows
//      what the write ACTUALLY did, and it must never be dropped because it
//      happened to match a value read before the write.
//
// WHY VALUE-IDENTITY IS A SOUND PROXY HERE -- stronger than it was for the
// 53C96: SONIC register reads have NO side effects.  ISR is cleared by
// writing 1s, not by reading; every other register read simply returns the
// stored word.  So two consecutive equal reads of one register provably
// carry no state transition between them, and dropping the second loses
// nothing but the fact that the driver asked again.  Which brings us to:
//
// WHAT MUST STILL BE CAPTURED SO THE TRACE STAYS USEFUL
// -----------------------------------------------------
// Suppressing a poll loop entirely would throw away one thing that matters a
// great deal for THIS bug: whether the driver is still spinning at all.
// "Driver is hammering ISR and never sees a bit set" and "driver stopped
// touching the SONIC completely" are different diagnoses with different
// fixes, and a silent ring looks identical in both cases.  So the filter
// counts what it drops, in two places:
//
//   * PER ENTRY: `skipped`, a saturating 0..127 count of reads filtered out
//     between the previous recorded entry and this one.  Reading a dump, a
//     nonzero skipped on an entry says "the driver spun here first".  127
//     means ">= 127" and is not a defect -- the exact depth of a long spin
//     is not interesting, its existence is.
//   * LIVE: `rd_filtered`, a saturating 16-bit total of every suppressed
//     read since the last re-arm, readable WITHOUT freezing.  Sampling it
//     twice a second apart answers "spinning or silent" directly, and it is
//     the companion measurement to sampling `rd_wrptr` twice (see the
//     `sonic-trace` command in tools/jtag_repl.tcl).
//
// Consequences of the filter, by construction:
//   * A wedge that spins on a constant ISR/CR advances NOTHING in the ring:
//     the pre-wedge history is preserved indefinitely, with no trigger logic
//     and no watchdog.  rd_filtered keeps climbing, so the wedge is still
//     visible as a live measurement.
//   * Healthy operation is close to transparent: ISR/CR genuinely change as
//     frames complete, and every write survives unconditionally.
//
// That self-freezing property is why there is deliberately NO watchdog
// auto-freeze here (same reasoning as scsi_trace_ring.v): any timeout long
// enough not to misfire on a legitimate idle gap is also long enough to be
// useless, and a shorter one would silently hand back the wrong window.  The
// host freezes explicitly and CONFIRMS quiescence first by measurement.
//
// ENTRY FORMAT (32 bits, MSB-first so a raw hex dump reads left to right):
//     [31]     rw        1 = CPU write, 0 = CPU read
//     [30:29]  strb      sonic_wstrb for writes; 2'b11 for reads (see above)
//     [28:23]  reg       SONIC register index (sonic_addr[5:0])
//     [22:7]   data      the 16-bit word transferred
//     [6:0]    skipped   saturating count of poll-filtered reads dropped
//                        immediately before this entry (127 = ">= 127")
//
// There is deliberately no timestamp field: 16-bit data plus real byte
// strobes plus the suppressed-poll count fill the word, and for the
// questions above (did a write ever happen; in what order; did a spin
// precede it) a wall-clock is decoration.  tools/scsi96_trace_diff.py
// ignores the SCSI ring's timestamp column for the same reason.
//
// CLOCK DOMAINS
// -------------
// Capture runs in pb_clk (the Mac MMIO island, the domain q700_eth_sonic
// takes).  Readout runs in the readout clock (core_clk, where the AXI-Lite
// eth_debug_regs page lives).  The ring RAM is simple dual-port (write
// pb_clk / read rd_clk).
//
// RAM readout is only meaningful while FROZEN -- with `freeze` asserted,
// wr_ptr and every RAM location are static, so there is no coherency hazard
// on a value that is not changing.  `rd_frozen` tells the host when that
// precondition actually holds.
//
// The two counters the host samples LIVE -- wr_ptr and the filtered total --
// are GRAY-CODED across the boundary rather than raw-synchronised.  That is
// a deliberate difference from scsi_trace_ring.v, which only guarantees its
// wr_ptr while frozen and then samples it live anyway to test quiescence: a
// torn multi-bit sample there can read as "unchanged" and turn the
// quiescence check into a coin flip.  Gray coding makes every sampled value
// a value the counter genuinely held, so "did it move?" is a real
// measurement.  Cost is two small XOR chains.
//
// Latency: an entry appears in the RAM 1 pb_clk after the sonic_ack that
// completed the beat.

`default_nettype none

module sonic_trace_ring #(
    parameter integer DEPTH_LG2 = 12    // 4096 entries
) (
    // ── Capture domain: the peripheral_bus <-> q700_eth_sonic port ────
    // Every one of these is an INPUT.  This module has no path back into
    // the SONIC port by construction.
    input  wire                  pb_clk,
    input  wire                  pb_rst,
    input  wire [5:0]            pb_addr,     // sonic_addr  (register index)
    input  wire [15:0]           pb_wdata,    // sonic_wdata
    input  wire [1:0]            pb_wstrb,    // sonic_wstrb (writes only)
    input  wire                  pb_wr,       // sonic_wr
    input  wire                  pb_rd,       // sonic_rd
    input  wire [15:0]           pb_rdata,    // sonic_rdata
    input  wire                  pb_ack,      // sonic_ack

    // ── Readout domain ────────────────────────────────────────────────
    input  wire                  rd_clk,
    input  wire                  rd_rst,
    input  wire                  rd_freeze,   // level: stop capturing
    input  wire                  rd_clear,    // level-TOGGLE: clear + re-arm
    input  wire [DEPTH_LG2-1:0]  rd_addr,
    output reg  [31:0]           rd_data,
    output wire [DEPTH_LG2-1:0]  rd_wrptr,
    output wire [15:0]           rd_filtered, // suppressed reads since re-arm
    output wire                  rd_wrapped,
    output wire                  rd_frozen
);

    localparam integer DEPTH = (1 << DEPTH_LG2);

    // Gray -> binary.  Written once at 32 bits and reused for both counters:
    // a value below 2^N gray-codes to a value below 2^N, so zero-extending
    // an N-bit gray code and converting at 32 bits returns the same N-bit
    // binary value.
    function [31:0] gray2bin32;
        input [31:0] g;
        integer k;
        reg [31:0] b;
        begin
            b[31] = g[31];
            for (k = 30; k >= 0; k = k - 1)
                b[k] = b[k+1] ^ g[k];
            gray2bin32 = b;
        end
    endfunction

    // ══════════════════════════════════════════════════════════════════
    // Control CDC: readout domain -> capture domain
    // ══════════════════════════════════════════════════════════════════
    (* ASYNC_REG = "TRUE" *) reg freeze_meta, freeze_sync;
    (* ASYNC_REG = "TRUE" *) reg clear_meta,  clear_sync;
    reg clear_sync_d;
    always @(posedge pb_clk) begin
        if (pb_rst) begin
            freeze_meta  <= 1'b0;
            freeze_sync  <= 1'b0;
            clear_meta   <= 1'b0;
            clear_sync   <= 1'b0;
            clear_sync_d <= 1'b0;
        end else begin
            freeze_meta  <= rd_freeze;
            freeze_sync  <= freeze_meta;
            clear_meta   <= rd_clear;
            clear_sync   <= clear_meta;
            clear_sync_d <= clear_sync;
        end
    end
    // rd_clear is a TOGGLE, not a pulse: the readout domain flips it and we
    // edge-detect here.  A pulse generated in another domain could be missed
    // entirely; a toggle cannot be.
    wire clear_ev = (clear_sync ^ clear_sync_d);

    // ══════════════════════════════════════════════════════════════════
    // Beat tracker
    // ══════════════════════════════════════════════════════════════════
    // q700_eth_sonic answers COMBINATIONALLY: `sonic_ack = sonic_cs &&
    // (sonic_rd || sonic_wr)` and `sonic_rdata` is a mux off the register
    // file, so request and ack land in the SAME cycle, and pb_rdata is valid
    // on that cycle.  (peripheral_bus.v drives sonic_wr / sonic_rd as
    // one-cycle pulses precisely because of that.)
    //
    // That is the opposite of scsi.v, which REGISTERS its ack -- so this
    // cannot use scsi_trace_ring.v's "latch on the request cycle, commit on a
    // later ack" shape: `pend` would be set at the end of the very cycle the
    // ack was high and the commit would never fire.  Instead the live port
    // values are used when nothing is pending, and the latch path is kept
    // only so a future registered-ack or ack-withholding SONIC still records
    // correctly.
    reg        pend;
    reg        lat_wr;
    reg [5:0]  lat_reg;
    reg [15:0] lat_wdata;
    reg [1:0]  lat_strb;

    wire        req_now   = pb_wr | pb_rd;
    wire        cap_wr    = pend ? lat_wr    : pb_wr;
    wire [5:0]  cap_reg   = pend ? lat_reg   : pb_addr;
    wire [15:0] cap_wdata = pend ? lat_wdata : pb_wdata;
    wire [1:0]  cap_strb  = pend ? lat_strb  : pb_wstrb;

    wire [15:0] beat_data  = cap_wr ? cap_wdata : pb_rdata;
    // Reads report 2'b11: the SONIC returns the whole register and the CPU's
    // read granularity is resolved inside peripheral_bus.v, not here.
    wire [1:0]  beat_strb  = cap_wr ? cap_strb : 2'b11;
    wire        beat_commit = (pend || req_now) && pb_ack;

    // ── Poll filter state: last value READ from each register ─────────
    reg [15:0] last_val [0:63];
    reg [63:0] last_vld;

    // Writes are ALWAYS kept -- see rule 1 in the header.
    wire beat_keep = cap_wr ||
                     !last_vld[cap_reg] ||
                     (last_val[cap_reg] != beat_data);

    // ── Ring ──────────────────────────────────────────────────────────
    (* ram_style = "block" *)
    reg [31:0] ring [0:DEPTH-1];
    reg [DEPTH_LG2-1:0] wr_ptr;
    reg [DEPTH_LG2-1:0] wr_ptr_gray;
    reg                 wrapped;
    reg                 frozen;

    // Suppressed-poll accounting (see "WHAT MUST STILL BE CAPTURED").
    reg [6:0]  skipped;          // since the last RECORDED entry, saturating
    reg [15:0] filtered_total;   // since the last re-arm, saturating
    reg [15:0] filtered_gray;

    wire [DEPTH_LG2-1:0] wr_ptr_next = wr_ptr + {{(DEPTH_LG2-1){1'b0}}, 1'b1};
    wire [15:0] filtered_next = (&filtered_total) ? filtered_total
                                                  : (filtered_total + 16'd1);

    wire [31:0] entry = {cap_wr, beat_strb, cap_reg, beat_data, skipped};

    // Capture is live only when neither the synchronised request nor the
    // sticky flag says otherwise.
    wire capture_en = !frozen && !freeze_sync;

    always @(posedge pb_clk) begin
        if (pb_rst) begin
            pend           <= 1'b0;
            lat_wr         <= 1'b0;
            lat_reg        <= 6'd0;
            lat_wdata      <= 16'd0;
            lat_strb       <= 2'b00;
            wr_ptr         <= {DEPTH_LG2{1'b0}};
            wr_ptr_gray    <= {DEPTH_LG2{1'b0}};
            wrapped        <= 1'b0;
            frozen         <= 1'b0;
            last_vld       <= 64'd0;
            skipped        <= 7'd0;
            filtered_total <= 16'd0;
            filtered_gray  <= 16'd0;
        end else begin
            // Explicit freeze from the host.  Sticky until cleared, so a
            // glitch on the way in cannot un-freeze a captured window.
            if (freeze_sync) frozen <= 1'b1;

            if (clear_ev) begin
                wr_ptr         <= {DEPTH_LG2{1'b0}};
                wr_ptr_gray    <= {DEPTH_LG2{1'b0}};
                wrapped        <= 1'b0;
                frozen         <= 1'b0;
                last_vld       <= 64'd0;
                skipped        <= 7'd0;
                filtered_total <= 16'd0;
                filtered_gray  <= 16'd0;
            end

            // Latch a request that did NOT complete in its own cycle.  With
            // today's combinational sonic_ack this never fires; it exists so
            // an ack-withholding SONIC would still be recorded correctly.
            if (!pend && req_now && !pb_ack) begin
                pend      <= 1'b1;
                lat_wr    <= pb_wr;
                lat_reg   <= pb_addr;
                lat_wdata <= pb_wdata;
                lat_strb  <= pb_wstrb;
            end

            if (beat_commit) begin
                pend <= 1'b0;
                // Maintain the filter shadow even while frozen, so an
                // unfreeze that is not a clear would resume from a truthful
                // baseline.  The clear/re-arm path DELIBERATELY wipes it
                // (last_vld <= 0 above): every capture window must be
                // self-contained, so the first read of each register after a
                // re-arm is always recorded rather than being suppressed by
                // history the dump does not contain.
                if (cap_wr) begin
                    last_vld[cap_reg] <= 1'b0;
                end else begin
                    last_val[cap_reg] <= beat_data;
                    last_vld[cap_reg] <= 1'b1;
                end
                if (capture_en) begin
                    if (beat_keep) begin
                        ring[wr_ptr] <= entry;
                        wr_ptr      <= wr_ptr_next;
                        wr_ptr_gray <= wr_ptr_next ^ (wr_ptr_next >> 1);
                        if (&wr_ptr) wrapped <= 1'b1;
                        skipped <= 7'd0;
                    end else begin
                        if (!(&skipped)) skipped <= skipped + 7'd1;
                        filtered_total <= filtered_next;
                        filtered_gray  <= filtered_next ^ (filtered_next >> 1);
                    end
                end
            end
        end
    end

    // ══════════════════════════════════════════════════════════════════
    // Readout domain
    // ══════════════════════════════════════════════════════════════════
    // RAM read port.  Only meaningful while `rd_frozen` reads 1 -- see the
    // header comment.
    always @(posedge rd_clk)
        rd_data <= ring[rd_addr];

    (* ASYNC_REG = "TRUE" *) reg [DEPTH_LG2-1:0] wrptr_meta, wrptr_sync;
    (* ASYNC_REG = "TRUE" *) reg [15:0]          filt_meta,  filt_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0]           flags_meta, flags_sync;
    always @(posedge rd_clk) begin
        if (rd_rst) begin
            wrptr_meta <= {DEPTH_LG2{1'b0}};
            wrptr_sync <= {DEPTH_LG2{1'b0}};
            filt_meta  <= 16'd0;
            filt_sync  <= 16'd0;
            flags_meta <= 2'b00;
            flags_sync <= 2'b00;
        end else begin
            wrptr_meta <= wr_ptr_gray;
            wrptr_sync <= wrptr_meta;
            filt_meta  <= filtered_gray;
            filt_sync  <= filt_meta;
            flags_meta <= {frozen, wrapped};
            flags_sync <= flags_meta;
        end
    end

    wire [31:0] wrptr_bin = gray2bin32({{(32-DEPTH_LG2){1'b0}}, wrptr_sync});
    wire [31:0] filt_bin  = gray2bin32({16'd0, filt_sync});

    assign rd_wrptr   = wrptr_bin[DEPTH_LG2-1:0];
    assign rd_filtered = filt_bin[15:0];
    assign rd_wrapped = flags_sync[0];
    assign rd_frozen  = flags_sync[1];

endmodule

`default_nettype wire
