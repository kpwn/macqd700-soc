// ifetch_window_guard.v -- fault instruction fetches that no slave claims.
//
// Why this exists (2026-09-09): the CPU's dedicated fetch master (axi_i,
// native 256b) is bound straight to l2c's f_axi port, NOT through the
// xbar, and l2c forwards f_axi_araddr into the cache controller / DDR with
// no window decode at all.  A fetch to an address no slave owns is
// therefore served from DRAM at (addr mod DRAM size).  Measured on p164:
// the Q700 ROM's 24-bit slot map sends NuBus slot $B addresses to
// physical 0xFB0493AA; the fetch of that address returned the ROM's own
// memory-test filler (0xdb6d b6db ...), which the CPU then EXECUTED --
// stray `addw` writes, an odd supervisor SP, a 10-byte store at RAM 0,
// execution of the vector table, every "mystery crash" of the campaign.
// A real Quadra 700 bus-errors that fetch.  The data side already has
// this policy (glue.v `fault` for unmapped windows); the fetch side had
// none.
//
// Policy: a fetch is IN WINDOW iff it targets the visible RAM window
// (0 .. 2^ram_window_lg2, the same runtime size the xbar honours) or the
// ROM mirror window (decided upstream, passed in as s_ar_is_rom so the
// decode is byte-for-byte the one ifa_araddr_folded already applies).
// Everything else is answered LOCALLY with DECERR (rresp=2'b11), zero
// data, arlen+1 beats and rlast on the last -- a real AXI response, which
// the CPU's I-cache turns into an instruction-access bus error
// (IcachePlugin respErr -> rspFault).  This is a fabric response, not an
// address-based cache-inhibit backstop: cacheability still comes from the
// MMU; the fabric merely refuses to invent data for addresses it does
// not own.
//
// Ordering: a local DECERR burst never interleaves inside, NOR PRECEDES, an
// in-flight downstream fetch; while the guard emits, downstream R is held
// (m_rready=0), which AXI permits.  One faulting request is held at a
// time; s_arready is dropped while it is outstanding.  In-window requests
// already accepted by l2c continue normally.
//
// ⚠ CORRECTED 2026-09-18 (race audit).  That used to read "(m_burst_open
// tracks m_r beats to rlast)", and m_burst_open was written ONLY on
// `m_rvalid && m_rready`.  Between AR acceptance downstream and the FIRST
// returned beat -- the whole l2c miss plus DDR round trip, 40-200 core
// clocks -- it therefore read 0, so the guard believed the R channel was
// free and went straight to G_EMIT.  Its local DECERR burst then PRECEDED
// the in-flight fetch's data and held m_rready low while it did.  The old
// claim was true for INTERLEAVE and false for PRECEDE.
//
// Second failure of the same flag, with two fetch ids live: l2c interleaves
// R across ids (see l2c.v), so m_burst_open dropped to 0 on the FIRST
// burst's rlast while a second was still mid-burst -- and then the DECERR
// beats really were inserted inside it, which AXI4 forbids outright.  The
// core this SoC hosts is multi-outstanding on its I side (l2c_mshr.v,
// l2c.v) and axi_i binds straight through this guard in the shipping
// CPU_M68K040 build, so both were reachable.
//
// The flag is now a POSITIVE RECORD of outstanding-ness -- incremented on
// the downstream AR handshake, decremented on the downstream RLAST -- which
// is the thing the ordering rule actually needs.  Both events are
// unconditional handshakes on the same interface with no abandonment path
// between them, so unlike a ledger kept across two different modules this
// one cannot leak; the counter is nevertheless sized well past any
// plausible fetch-side limit and carries a simulation overflow assertion.
//
// Negative control: tb_ifetch_window_guard.cpp, "a faulting AR while a
// downstream fetch is OUTSTANDING BUT HAS NO BEATS YET must still wait"
// (its downstream model gained an AR->R latency knob for exactly this; the
// zero-latency model is why the hole was invisible).
module ifetch_window_guard #(
    parameter ID_WIDTH   = 4,
    parameter DATA_WIDTH = 256
) (
    input  wire                  clk,
    input  wire                  rst,
    input  wire [5:0]            ram_window_lg2,   // visible RAM size, log2 bytes
    input  wire                  s_ar_is_rom,      // upstream decode: ROM mirror window

    // upstream: the CPU fetch master
    input  wire [ID_WIDTH-1:0]   s_arid,
    input  wire [31:0]           s_araddr,
    input  wire [7:0]            s_arlen,
    input  wire [2:0]            s_arsize,
    input  wire [1:0]            s_arburst,
    input  wire                  s_arvalid,
    output wire                  s_arready,
    output wire [ID_WIDTH-1:0]   s_rid,
    output wire [DATA_WIDTH-1:0] s_rdata,
    output wire [1:0]            s_rresp,
    output wire                  s_rlast,
    output wire                  s_rvalid,
    input  wire                  s_rready,

    // downstream: l2c f_axi port
    output wire [ID_WIDTH-1:0]   m_arid,
    output wire [31:0]           m_araddr,
    output wire [7:0]            m_arlen,
    output wire [2:0]            m_arsize,
    output wire [1:0]            m_arburst,
    output wire                  m_arvalid,
    input  wire                  m_arready,
    input  wire [ID_WIDTH-1:0]   m_rid,
    input  wire [DATA_WIDTH-1:0] m_rdata,
    input  wire [1:0]            m_rresp,
    input  wire                  m_rlast,
    input  wire                  m_rvalid,
    output wire                  m_rready,

    // observability (VIO / debug): first faulting fetch address, sticky
    output wire                  fault_sticky,
    output wire [31:0]           fault_addr,
    output wire [15:0]           fault_count
);
    // Clamp exactly like axi_xbar.v (22 = 4 MiB .. 30 = 1 GiB).
    wire [5:0] lg2 = (ram_window_lg2 < 6'd22) ? 6'd22 :
                     (ram_window_lg2 > 6'd30) ? 6'd30 : ram_window_lg2;
    wire in_ram    = ((s_araddr >> lg2) == 32'd0);
    wire in_window = in_ram || s_ar_is_rom;

    localparam [1:0] G_IDLE = 2'd0, G_PEND = 2'd1, G_EMIT = 2'd2;
    reg  [1:0]          gstate;
    reg  [ID_WIDTH-1:0] g_id;
    reg  [7:0]          g_len, g_beat;

    // Downstream OUTSTANDING-READ tracking, so a DECERR burst can neither
    // split nor overtake one.  See the ordering note in the header for why
    // this is a counter and not the old beat-observation flag.
    localparam OUT_CNT_W = 5;                 // 31 outstanding; see the assert
    reg  [OUT_CNT_W-1:0] m_out_cnt;
    wire m_ar_hs   = m_arvalid && m_arready;
    wire m_rlast_hs= m_rvalid  && m_rready && m_rlast;
    always @(posedge clk) begin
        if (rst) m_out_cnt <= {OUT_CNT_W{1'b0}};
        else if (m_ar_hs && !m_rlast_hs) m_out_cnt <= m_out_cnt + {{(OUT_CNT_W-1){1'b0}}, 1'b1};
        else if (m_rlast_hs && !m_ar_hs) m_out_cnt <= m_out_cnt - {{(OUT_CNT_W-1){1'b0}}, 1'b1};
    end
    wire m_burst_open = (m_out_cnt != {OUT_CNT_W{1'b0}});
    // synthesis translate_off
    always @(posedge clk) begin
        if (!rst && m_ar_hs && !m_rlast_hs &&
            (m_out_cnt == {OUT_CNT_W{1'b1}})) begin
            $display("IFETCH_GUARD ASSERT FAIL [%0t]: downstream outstanding-read counter would wrap (>%0d in flight); widen OUT_CNT_W",
                     $time, (1 << OUT_CNT_W) - 1);
            $finish;
        end
        if (!rst && m_rlast_hs && !m_ar_hs &&
            (m_out_cnt == {OUT_CNT_W{1'b0}})) begin
            $display("IFETCH_GUARD ASSERT FAIL [%0t]: downstream RLAST with nothing outstanding", $time);
            $finish;
        end
    end
    // synthesis translate_on

    wire g_busy = (gstate != G_IDLE);
    wire g_emit = (gstate == G_EMIT);

    // AR: in-window passes through (only while the guard is idle, so a
    // faulting request is never reordered behind a later in-window one in
    // a way that confuses the id-tagged I-cache); out-of-window is
    // accepted by the guard.
    assign m_arid    = s_arid;
    assign m_araddr  = s_araddr;
    assign m_arlen   = s_arlen;
    assign m_arsize  = s_arsize;
    assign m_arburst = s_arburst;
    assign m_arvalid = s_arvalid && in_window && !g_busy;
    // ⚠ The `!rst` term is load-bearing, race audit 2026-09-18.  The 1'b1 arm
    // used to be unconditional while the g_accept capture below lives inside
    // the `else` of `if (rst)`.  Upstream is the CPU axi_i master on
    // `cpu_rst`, a DIFFERENT net from this guard's `core_rst`, so an
    // out-of-window AR really can be presented while the guard is held: it
    // handshook, nothing was recorded, no DECERR was ever emitted, and the
    // I-cache's refill slot for that id never cleared.  Refusing it instead
    // costs nothing -- the master simply retries.
    assign s_arready = (!rst) && (g_busy ? 1'b0 : (in_window ? m_arready : 1'b1));
    wire   g_accept  = (!rst) && s_arvalid && !in_window && !g_busy;

    // R: guard has the channel only while emitting; downstream is held.
    assign s_rvalid  = g_emit ? 1'b1 : m_rvalid;
    assign s_rid     = g_emit ? g_id : m_rid;
    assign s_rdata   = g_emit ? {DATA_WIDTH{1'b0}} : m_rdata;
    assign s_rresp   = g_emit ? 2'b11 : m_rresp;     // DECERR
    assign s_rlast   = g_emit ? (g_beat == g_len) : m_rlast;
    assign m_rready  = g_emit ? 1'b0 : s_rready;

    reg        fault_sticky_r;
    reg [31:0] fault_addr_r;
    reg [15:0] fault_count_r;
    assign fault_sticky = fault_sticky_r;
    assign fault_addr   = fault_addr_r;
    assign fault_count  = fault_count_r;

    always @(posedge clk) begin
        if (rst) begin
            gstate <= G_IDLE; g_id <= {ID_WIDTH{1'b0}}; g_len <= 8'd0; g_beat <= 8'd0;
            fault_sticky_r <= 1'b0; fault_addr_r <= 32'd0; fault_count_r <= 16'd0;
        end else begin
            case (gstate)
                G_IDLE: if (g_accept) begin
                    g_id <= s_arid; g_len <= s_arlen; g_beat <= 8'd0;
                    // A first beat may be presented but stalled, or may
                    // handshake on this very edge. m_burst_open alone
                    // describes only the PREVIOUS accepted beat. Taking
                    // the channel now would change a stalled payload or
                    // split the newly opened downstream burst.
                    gstate <= (m_burst_open || m_rvalid) ? G_PEND : G_EMIT;
                    if (!fault_sticky_r) begin fault_sticky_r <= 1'b1; fault_addr_r <= s_araddr; end
                    fault_count_r <= fault_count_r + 16'd1;
                end
                // Also cover a new first beat appearing during the gap
                // after an older RLAST: never replace a presented beat.
                G_PEND: if (!m_burst_open && !m_rvalid) gstate <= G_EMIT;
                G_EMIT: if (s_rready) begin
                    if (g_beat == g_len) gstate <= G_IDLE;
                    else g_beat <= g_beat + 8'd1;
                end
                default: gstate <= G_IDLE;
            endcase
        end
    end
endmodule
