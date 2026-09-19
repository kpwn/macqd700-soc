// rtl/soc/scsi_trace_ring.v — non-destructive 53C96 register-access trace ring
//
// PURPOSE
// -------
// Record the exact sequence of register accesses the Mac OS SCSI Manager
// makes to the on-board NCR 53C96, so it can be diffed against the same
// sequence captured from MAME (tools/mame_scsi96_capture.lua, which taps
// the 0x50f0f000 TurboSCSI aperture).  Both sides emit the same
// (rw, reg, data) event stream, so `tools/scsi96_trace_diff.py` can find
// the first divergence.
//
// WHY THIS EXISTS AT ALL
// ----------------------
// Reading the live 53C96 over JTAG is DESTRUCTIVE: a read of register 2
// pops the FIFO and a read of register 5 clears the pending interrupt.
// You cannot poll the register file on hardware without changing the
// thing you are measuring.  This ring is a pure observer -- it snoops the
// peripheral_bus <-> scsi.v port and never issues an access of its own --
// so it can run continuously through a whole boot without perturbing the
// driver.
//
// WHAT IT CAPTURES
// ----------------
// One entry per COMPLETED peripheral-bus beat (pb_ack), carrying the
// direction, the register index, and the byte.  For reads the byte is the
// value scsi.v actually returned; for writes it is the value the CPU
// wrote.  That is exactly the information the MAME tap provides.
//
// THE POLL FILTER  (this is the load-bearing design decision)
// -----------------------------------------------------------
// A wedged driver spins forever re-reading the status register.  A plain
// last-N-wins ring would be completely overwritten by that poll loop
// before anyone could read it out, destroying precisely the history we
// want -- the events LEADING INTO the wedge.
//
// So reads are filtered: a read is recorded only if its value DIFFERS
// from the last value read from that same register.  Writes are always
// recorded (they are the commands -- never drop them), and a write to a
// register invalidates that register's cached read value so the next read
// is always recorded.
//
// Consequences, both measured against the MAME capture of a healthy 7.5.3
// boot (275,355 raw events):
//   * During HEALTHY operation the filter is nearly transparent -- 84.9%
//     of events survive, because the registers genuinely change value as
//     a transaction progresses.  History depth is ~78 SCSI transactions
//     in a 4096-entry ring (median filtered transaction = 52 events).
//   * During a WEDGE every polled register returns a constant, so every
//     poll is filtered out and the ring STOPS ADVANCING ON ITS OWN.  The
//     history leading into the wedge is preserved indefinitely with no
//     trigger logic at all.
//
// That self-freezing property is why there is deliberately NO watchdog
// auto-freeze in this module.  A watchdog would need a timeout, and a
// timeout long enough not to misfire on a legitimate idle gap during boot
// is also long enough to be useless -- while a timeout short enough to be
// useful would freeze the ring on a normal pause and silently hand back
// the wrong window.  Instead the host freezes explicitly, and can CONFIRM
// the ring is quiescent first by reading `wr_ptr` twice a second apart
// (see the `scsi-trace` command in tools/jtag_repl.tcl).  That is a
// measurement, not an assumption.
//
// ENTRY FORMAT (32 bits, LSB-aligned fields chosen so a raw hex dump is
// readable by eye):
//     [31]     rw            1 = CPU write, 0 = CPU read
//     [30]     dma_shim      1 = pseudo-DMA port (pb_addr 0x100/0x101)
//     [29:26]  reg           53C96 register index (pb_addr[3:0])
//     [25:18]  data          the byte transferred
//     [17:0]   tstamp        free-running pb_clk counter bits [24:7]
//                            (~1 us resolution, ~268 ms wrap at 125 MHz)
//
// CLOCK DOMAINS
// -------------
// Capture runs in pb_clk (the Mac MMIO island, the same domain scsi.v
// takes).  Readout runs in the readout clock (core_clk, where the
// AXI-Lite config block lives).  The ring RAM is a simple dual-port
// (write pb_clk / read rd_clk).
//
// The CDC is deliberately trivial because readout is only ever valid
// while the ring is FROZEN: with `freeze` asserted, wr_ptr / wrapped and
// every RAM location are static, so a plain two-flop synchroniser on a
// multi-bit vector is safe (there is no coherency hazard on a value that
// is not changing).  `rd_frozen` tells the host when that precondition
// actually holds -- do not trust wr_ptr until it reads 1.
//
// Latency: an entry appears in the RAM 1 pb_clk after the pb_ack that
// completed the beat.

module scsi_trace_ring #(
    parameter integer DEPTH_LG2 = 12    // 4096 entries
) (
    // ── Capture domain: the peripheral_bus <-> scsi.v port ────────────
    input  wire                  pb_clk,
    input  wire                  pb_rst,
    input  wire [8:0]            pb_addr,
    input  wire [7:0]            pb_wdata,
    input  wire                  pb_wr,
    input  wire                  pb_rd,
    input  wire [7:0]            pb_rdata,
    input  wire                  pb_ack,

    // ── Readout domain ────────────────────────────────────────────────
    input  wire                  rd_clk,
    input  wire                  rd_rst,
    input  wire                  rd_freeze,   // level: stop capturing
    input  wire                  rd_clear,    // level-toggle: clear + re-arm
    input  wire [DEPTH_LG2-1:0]  rd_addr,
    output reg  [31:0]           rd_data,
    output wire [DEPTH_LG2-1:0]  rd_wrptr,
    output wire                  rd_wrapped,
    output wire                  rd_frozen
);

    localparam integer DEPTH = (1 << DEPTH_LG2);

    // ══════════════════════════════════════════════════════════════════
    // Control CDC: readout domain -> capture domain
    // ══════════════════════════════════════════════════════════════════
    // ASYNC_REG clusters each meta/sync pair on one slice for metastability
    // margin.  It does NOT stop the timer from timing the cross-domain edge --
    // synth/fpga_top.xdc carries the matching `set_false_path -to ...\*_meta_reg/D`
    // for these, following the same policy as u_dafb_vbl_cdc.
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
    // rd_clear is a TOGGLE, not a pulse: the readout domain flips it and
    // we edge-detect here.  A pulse generated in a slower/faster domain
    // could be missed entirely; a toggle cannot be.
    wire clear_ev = (clear_sync ^ clear_sync_d);

    // ══════════════════════════════════════════════════════════════════
    // Beat tracker
    // ══════════════════════════════════════════════════════════════════
    // scsi.v answers with `pb_ack <= pb_wr | pb_rd` (registered) and
    // latches `pb_rdata <= pb_rd_mux` on the same edge, so on the cycle
    // pb_ack is high, pb_rdata holds THIS beat's data.  peripheral_bus
    // holds addr/wdata stable for the whole beat, and scsi.v can withhold
    // pb_ack for extra cycles on the DMA shim, so we latch the request on
    // its first cycle and commit when the ack lands.
    reg        pend;
    reg        lat_wr;
    reg        lat_dma;
    reg [3:0]  lat_reg;
    reg [7:0]  lat_wdata;

    wire dma_shim = (pb_addr == 9'h100) || (pb_addr == 9'h101);

    // ── Poll filter state: last value READ from each register ─────────
    // Index is {dma_shim, reg}, so the pseudo-DMA port gets its own
    // 16-entry shadow and cannot alias a real register.
    reg [7:0]  last_val [0:31];
    reg [31:0] last_vld;

    wire [4:0] lat_idx = {lat_dma, lat_reg};
    wire [7:0] beat_data = lat_wr ? lat_wdata : pb_rdata;
    wire       beat_commit = pend && pb_ack;
    // Writes are ALWAYS kept -- they are the commands, and a repeated
    // identical command (the driver re-issuing Transfer Info to pull the
    // next 16-byte chunk) is exactly the signal we are hunting.
    wire       beat_keep = lat_wr ||
                           !last_vld[lat_idx] ||
                           (last_val[lat_idx] != beat_data);

    // ── Free-running timestamp ────────────────────────────────────────
    reg [24:0] tick;

    // ── Ring ──────────────────────────────────────────────────────────
    (* ram_style = "block" *)
    reg [31:0] ring [0:DEPTH-1];
    reg [DEPTH_LG2-1:0] wr_ptr;
    reg                 wrapped;
    reg                 frozen;

    wire [31:0] entry = {lat_wr, lat_dma, lat_reg, beat_data, tick[24:7]};

    integer i;
    always @(posedge pb_clk) begin
        if (pb_rst) begin
            pend      <= 1'b0;
            lat_wr    <= 1'b0;
            lat_dma   <= 1'b0;
            lat_reg   <= 4'd0;
            lat_wdata <= 8'd0;
            wr_ptr    <= {DEPTH_LG2{1'b0}};
            wrapped   <= 1'b0;
            frozen    <= 1'b0;
            last_vld  <= 32'd0;
            tick      <= 25'd0;
        end else begin
            tick <= tick + 25'd1;

            // Explicit freeze from the host.  Sticky until cleared, so a
            // glitch on the way in cannot un-freeze a captured window.
            if (freeze_sync) frozen <= 1'b1;

            if (clear_ev) begin
                wr_ptr   <= {DEPTH_LG2{1'b0}};
                wrapped  <= 1'b0;
                frozen   <= 1'b0;
                last_vld <= 32'd0;
            end

            // Latch a new request on its first cycle.
            if (!pend && (pb_wr || pb_rd)) begin
                pend      <= 1'b1;
                lat_wr    <= pb_wr;
                lat_dma   <= dma_shim;
                lat_reg   <= pb_addr[3:0];
                lat_wdata <= pb_wdata;
            end

            if (beat_commit) begin
                pend <= 1'b0;
                // Maintain the filter shadow even while frozen, so a
                // re-arm after a freeze starts from a truthful baseline.
                if (lat_wr) begin
                    last_vld[lat_idx] <= 1'b0;
                end else begin
                    last_val[lat_idx] <= beat_data;
                    last_vld[lat_idx] <= 1'b1;
                end
                if (beat_keep && !frozen && !freeze_sync) begin
                    ring[wr_ptr] <= entry;
                    wr_ptr <= wr_ptr + {{(DEPTH_LG2-1){1'b0}}, 1'b1};
                    if (&wr_ptr) wrapped <= 1'b1;
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
    (* ASYNC_REG = "TRUE" *) reg [1:0]           flags_meta, flags_sync;
    always @(posedge rd_clk) begin
        if (rd_rst) begin
            wrptr_meta <= {DEPTH_LG2{1'b0}};
            wrptr_sync <= {DEPTH_LG2{1'b0}};
            flags_meta <= 2'b00;
            flags_sync <= 2'b00;
        end else begin
            wrptr_meta <= wr_ptr;
            wrptr_sync <= wrptr_meta;
            flags_meta <= {frozen, wrapped};
            flags_sync <= flags_meta;
        end
    end

    assign rd_wrptr   = wrptr_sync;
    assign rd_wrapped = flags_sync[0];
    assign rd_frozen  = flags_sync[1];

endmodule
