// dh_write_window_guard.v -- address guard for the WRITE half of the D-side
// hot door (P6W, l2c.v DW_PORT_EN).  The AW/W/B twin of
// ifetch_window_guard.v, which guards dh_axi_ar*/r* (and axi_i).
//
// WHY IT EXISTS.  The hot door has no address decode: l2c serves whatever
// line address it is given, and the DDR side answers (addr mod DRAM size).
// The core routes a write to this door by its MMU cache mode, and "the MMU
// says copyback" is not "this is memory" -- with TC.E=0 every address is
// cacheable, and a copyback page can be mapped over anything.  A write that
// takes this door and is NOT RAM would silently overwrite aliased DRAM, which
// is strictly worse than the read-side failure (stale bytes returned): it
// corrupts state that the correct path will later read.  So every AW is
// classified, exactly once, at the door:
//
//   in the visible RAM window   -> FORWARDED to l2c unchanged
//   ROM mirror (s_aw_is_rom)    -> DROPPED, B = OKAY.  This is the crossbar's
//                                  own policy for a CPU-LSU write to ROM
//                                  (axi_xbar.v, M0 role note): Mac OS stores
//                                  into the ROM mirror during boot and
//                                  expects no fault.  A copyback-cached ROM
//                                  line dirtied that way is written back
//                                  HERE, so dropping it is the only answer
//                                  that matches the cold path.
//   anything else               -> DECERR, data discarded, counted.  Loud.
//
// ORDERING.  AXI requires B in AW order per ID.  A locally answered burst is
// therefore started only when NOTHING forwarded is still outstanding (no B
// owed, no W beat owed), and while it runs nothing new is forwarded.  That
// is conservative -- it serialises across IDs too -- and is free where it
// matters: a local answer is an error path (DECERR) or a boot-time ROM
// store, never streaming traffic.
//
// W ROUTING.  AXI4 has no WID, so the W stream follows AW order.  A W beat is
// forwarded only while some forwarded AW still owes W beats (`w_owed`), and
// consumed locally only in the local burst's own drain state.  A master that
// presents W before its AW simply waits (legal AXI; l2c cannot take a W beat
// before its header either, so this costs nothing).
//
// CPU RESET (`abandon`).  The core's reset clears its writeback buffer, but
// the fabric keeps what it already accepted: an AW whose W beats never came
// would hold l2c's hot-door tracker open forever and the NEXT core's W beats
// would be taken as that burst's data; a B still owed would be handed to a
// writeback buffer that never issued it.  So, exactly like the crossbar's
// abandoned-burst padding on the cold path (axi_xbar.v, m0_pad_w): while
// `abandon` is high, and afterwards until every forwarded burst has fully
// retired, the guard refuses new AWs and W beats, completes owed W beats
// itself with WSTRB=0 filler (a no-op write), finishes a local burst's
// drain, and swallows every B.  The core therefore needs no write-side reset
// absorber on this port.
//
// Verilog-2005, single clock, synchronous active-high reset.
`default_nettype none
module dh_write_window_guard #(
    parameter ID_WIDTH   = 2,
    parameter DATA_WIDTH = 128
) (
    input  wire                    clk,
    input  wire                    rst,
    input  wire [5:0]              ram_window_lg2,   // visible RAM size, log2 bytes
    input  wire                    s_aw_is_rom,      // upstream decode: ROM mirror window
    input  wire                    abandon,          // CPU held in reset (level)

    input  wire [ID_WIDTH-1:0]     s_awid,
    input  wire [31:0]             s_awaddr,
    input  wire [7:0]              s_awlen,
    input  wire [2:0]              s_awsize,
    input  wire [1:0]              s_awburst,
    input  wire                    s_awvalid,
    output wire                    s_awready,
    input  wire [DATA_WIDTH-1:0]   s_wdata,
    input  wire [DATA_WIDTH/8-1:0] s_wstrb,
    input  wire                    s_wlast,
    input  wire                    s_wvalid,
    output wire                    s_wready,
    output wire [ID_WIDTH-1:0]     s_bid,
    output wire [1:0]              s_bresp,
    output wire                    s_bvalid,
    input  wire                    s_bready,

    output wire [ID_WIDTH-1:0]     m_awid,
    output wire [31:0]             m_awaddr,
    output wire [7:0]              m_awlen,
    output wire [2:0]              m_awsize,
    output wire [1:0]              m_awburst,
    output wire                    m_awvalid,
    input  wire                    m_awready,
    output wire [DATA_WIDTH-1:0]   m_wdata,
    output wire [DATA_WIDTH/8-1:0] m_wstrb,
    output wire                    m_wlast,
    output wire                    m_wvalid,
    input  wire                    m_wready,
    input  wire [ID_WIDTH-1:0]     m_bid,
    input  wire [1:0]              m_bresp,
    input  wire                    m_bvalid,
    output wire                    m_bready,

    output wire                    fault_sticky,     // first DECERR latched
    output wire [31:0]             fault_addr,       // ...and its address
    output wire [15:0]             fault_count,      // DECERR bursts
    output wire [15:0]             drop_count        // ROM-mirror bursts dropped (OKAY)
);
    // Same clamp as ifetch_window_guard: the window is 4 MiB .. 1 GiB.
    wire [5:0] lg2 = (ram_window_lg2 < 6'd22) ? 6'd22 :
                     (ram_window_lg2 > 6'd30) ? 6'd30 : ram_window_lg2;
    wire in_ram = ((s_awaddr >> lg2) == 32'd0);

    // ---- forwarded-traffic ledgers -------------------------------------
    // b_owed: forwarded AWs whose B has not come back.  w_owed: forwarded
    // AWs whose last W beat has not been forwarded.  5 bits = 31 in flight,
    // far above l2c's two-header door; overflow is asserted, not assumed.
    localparam CW = 5;
    reg [CW-1:0] b_owed, w_owed;
    reg [7:0]    wf_beat;             // beat index inside the head forwarded burst
    reg [7:0]    wf_len_q [0:3];      // awlen of forwarded bursts, FIFO (W order = AW order)
    reg [1:0]    wf_wp, wf_rp;

    // ---- local (dropped / DECERR) burst ---------------------------------
    localparam [1:0] L_IDLE = 2'd0, L_DRAIN = 2'd1, L_RESP = 2'd2;
    reg [1:0]          lst;
    reg [ID_WIDTH-1:0] l_id;
    reg [7:0]          l_len, l_beat;
    reg                l_rom;
    wire l_busy = (lst != L_IDLE);

    wire fwd_quiet   = (b_owed == {CW{1'b0}}) && (w_owed == {CW{1'b0}});
    // Abandon: `pad_q` holds from the first cycle of `abandon` until
    // everything accepted before it has retired; `quiesce` blocks the
    // upstream side for that whole span.
    reg  pad_q;
    wire quiesce     = abandon || pad_q;
    wire local_start = s_awvalid && !in_ram && !l_busy && fwd_quiet && !rst && !quiesce;

    // AW: forward in-window headers while no local burst is running.  An
    // out-of-window header waits (awready low) until the forwarded traffic
    // ahead of it has fully retired.
    assign m_awid    = s_awid;
    assign m_awaddr  = s_awaddr;
    assign m_awlen   = s_awlen;
    assign m_awsize  = s_awsize;
    assign m_awburst = s_awburst;
    // w_owed must never exceed the length FIFO's 4 entries.
    wire   wf_full   = (w_owed[CW-1:2] != {(CW-2){1'b0}});
    assign m_awvalid = s_awvalid && in_ram && !l_busy && !wf_full && !rst && !quiesce;
    assign s_awready = !rst && !l_busy && !quiesce && (in_ram ? (m_awready && !wf_full) : fwd_quiet);
    wire   m_aw_hs   = m_awvalid && m_awready;

    // W.
    wire   l_drain   = (lst == L_DRAIN);
    wire   wf_live   = (w_owed != {CW{1'b0}});
    wire   wf_last_c = (wf_beat == wf_len_q[wf_rp]);
    assign m_wdata   = s_wdata;
    assign m_wstrb   = quiesce ? {(DATA_WIDTH/8){1'b0}} : s_wstrb;   // pad = no-op beat
    assign m_wlast   = wf_last_c;          // beat count, not the master's WLAST
    assign m_wvalid  = wf_live && !l_drain && (quiesce || s_wvalid);
    assign s_wready  = !quiesce && (l_drain ? 1'b1 : (wf_live && m_wready));
    wire   m_w_hs    = m_wvalid && m_wready;
    wire   m_wl_hs   = m_w_hs && wf_last_c;

    // B.
    wire   l_resp    = (lst == L_RESP);
    assign s_bvalid  = !quiesce && (l_resp ? 1'b1 : m_bvalid);
    assign s_bid     = l_resp ? l_id : m_bid;
    assign s_bresp   = l_resp ? (l_rom ? 2'b00 : 2'b11) : m_bresp;
    assign m_bready  = quiesce ? !l_resp : (l_resp ? 1'b0 : s_bready);
    wire   m_b_hs    = m_bvalid && m_bready;

    reg        fault_sticky_r;
    reg [31:0] fault_addr_r;
    reg [15:0] fault_count_r, drop_count_r;
    assign fault_sticky = fault_sticky_r;
    assign fault_addr   = fault_addr_r;
    assign fault_count  = fault_count_r;
    assign drop_count   = drop_count_r;

    wire unused_ok = s_wlast;   // beat counts drive every burst boundary

    always @(posedge clk) begin
        if (rst) begin
            b_owed <= {CW{1'b0}}; w_owed <= {CW{1'b0}};
            wf_beat <= 8'd0; wf_wp <= 2'd0; wf_rp <= 2'd0;
            lst <= L_IDLE; l_id <= {ID_WIDTH{1'b0}}; l_len <= 8'd0; l_beat <= 8'd0; l_rom <= 1'b0;
            fault_sticky_r <= 1'b0; fault_addr_r <= 32'd0;
            fault_count_r <= 16'd0; drop_count_r <= 16'd0;
            pad_q <= 1'b0;
        end else begin
            // Sticky from `abandon` until everything accepted before it --
            // forwarded W beats, forwarded Bs, a local burst -- has retired.
            if (abandon) pad_q <= 1'b1;
            else if (pad_q && fwd_quiet && !l_busy) pad_q <= 1'b0;
            // Ledgers.
            if (m_aw_hs && !m_b_hs) b_owed <= b_owed + {{(CW-1){1'b0}}, 1'b1};
            else if (m_b_hs && !m_aw_hs) b_owed <= b_owed - {{(CW-1){1'b0}}, 1'b1};
            if (m_aw_hs && !m_wl_hs) w_owed <= w_owed + {{(CW-1){1'b0}}, 1'b1};
            else if (m_wl_hs && !m_aw_hs) w_owed <= w_owed - {{(CW-1){1'b0}}, 1'b1};
            if (m_aw_hs) begin
                wf_len_q[wf_wp] <= s_awlen;
                wf_wp <= wf_wp + 2'd1;
            end
            if (m_w_hs) begin
                if (wf_last_c) begin wf_beat <= 8'd0; wf_rp <= wf_rp + 2'd1; end
                else wf_beat <= wf_beat + 8'd1;
            end
            // Local burst.
            case (lst)
                L_IDLE: if (local_start) begin
                    lst <= L_DRAIN; l_id <= s_awid; l_len <= s_awlen; l_beat <= 8'd0;
                    l_rom <= s_aw_is_rom;
                    if (s_aw_is_rom) drop_count_r <= drop_count_r + 16'd1;
                    else begin
                        if (!fault_sticky_r) begin fault_sticky_r <= 1'b1; fault_addr_r <= s_awaddr; end
                        fault_count_r <= fault_count_r + 16'd1;
                    end
                end
                L_DRAIN: if (s_wvalid || quiesce) begin
                    if (l_beat == l_len) lst <= L_RESP;
                    else l_beat <= l_beat + 8'd1;
                end
                L_RESP: if (s_bready || quiesce) lst <= L_IDLE;
                default: lst <= L_IDLE;
            endcase
        end
    end

    // synthesis translate_off
    always @(posedge clk) begin
        if (!rst && m_aw_hs && !m_b_hs && (b_owed == {CW{1'b1}})) begin
            $display("DH_WRITE_GUARD ASSERT FAIL [%0t]: outstanding-B ledger would wrap", $time);
            $fatal(1);
        end
        if (!rst && m_b_hs && !m_aw_hs && (b_owed == {CW{1'b0}})) begin
            $display("DH_WRITE_GUARD ASSERT FAIL [%0t]: downstream B with nothing outstanding", $time);
            $fatal(1);
        end
        if (!rst && m_bvalid && l_resp) begin
            $display("DH_WRITE_GUARD ASSERT FAIL [%0t]: downstream B while a local burst answers -- local start was not quiet", $time);
            $fatal(1);
        end
        if (!rst && m_wvalid && !quiesce && (m_wlast !== s_wlast)) begin
            $display("DH_WRITE_GUARD ASSERT FAIL [%0t]: master WLAST=%b disagrees with AWLEN beat count", $time, s_wlast);
            $fatal(1);
        end
    end
    // synthesis translate_on
endmodule
`default_nettype wire
