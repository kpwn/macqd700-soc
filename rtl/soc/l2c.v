// l2c.v -- L2 system cache top level.
//
// Standalone module (integration into fpga_top's xbar S0 <-> DDR path is
// a LATER task -- see docs/l2c_spec.md).  AXI4 slave port (from the
// xbar's S0), AXI4 master port (toward the DDR path).  128-bit data,
// ID_WIDTH=6 by default (matches what axi_xbar's S0 presents today --
// XID_WIDTH = ID_WIDTH(4) + 2, see rtl/soc/fpga_top_xbar.vh).
//
// Instantiates l2c_ctrl (cache-proper: tags/data/mshr/reset/front-door),
// l2c_victim (dirty-eviction writeback buffer) and l2c_bypass
// (never-allocate window pass-through), then arbitrates their AXI-master
// sub-ports onto the single physical m_axi port and their three response
// sources onto s_axi's R/B channels (fixed priority bypass > mshr >
// hit-path -- see l2c_ctrl.v/l2c_bypass.v headers for why bypass wins
// starvation-avoidance ties).
//
// Round-2/3/4 review fixes (see individual comments below for detail):
// - Critical-4 (m_axi arbiters, *_grant_live_c): AR/R and AW/W/B
//   master-port arbiters LOCK their winner the moment ANYTHING is first
//   presented (VALID asserted), not after the handshake -- else a
//   higher-priority source's valid mid-presentation could swap the
//   payload under an already-VALID transfer (AXI stability violation).
// - Critical-5 (response mux, r_lock/b_lock): same fix, response side.
// - Important-A (v2): AW/W/B arbiter holds a reset-grant only while a
//   REAL AW-accept-to-B transaction is open, not for a bare AW
//   presentation -- see the arbiter's own comment.
// - 2026-08-20: the AW/W/B arbiter no longer holds its grant to B.  It
//   locks the write port to ONE OWNER (AXI4 has no WID, so W bursts from
//   two sources may never interleave) but lets that owner run many writes
//   in flight, tracked by count.  l2c_victim.v is correspondingly deeper
//   and pipelined; see its header for the measurement that motivated it.
// - Round-4: s_axi_rlast for bypass reads now echoes byp_rsp_last
//   (per-beat, threaded from l2c_ctrl.v) instead of a hardcoded 1'b1.
//
// L2_BYPASS_ALL (default 0): compile-time bringup escape hatch -- when
// set, s_axi is wired straight through to m_axi and none of the cache
// machinery is even instantiated (docs/l2c_spec.md S8).
//
// Verilog-2005, single clk domain, sync active-high rst.
`default_nettype none
module l2c #(
    parameter ADDR_WIDTH   = 32,
    parameter DATA_WIDTH   = 128,
    parameter ID_WIDTH     = 6,
    parameter L2_BYPASS_ALL = 0, parameter NUM_BYPASS_WINDOWS = 1,
    parameter [NUM_BYPASS_WINDOWS*ADDR_WIDTH-1:0] BYP_WIN_BASE = {ADDR_WIDTH{1'b0}},
    parameter [NUM_BYPASS_WINDOWS*ADDR_WIDTH-1:0] BYP_WIN_MASK = {ADDR_WIDTH{1'b0}},
    parameter [NUM_BYPASS_WINDOWS-1:0]            BYP_WIN_EN   = {NUM_BYPASS_WINDOWS{1'b0}},
    parameter [ADDR_WIDTH-1:0] CACHEABLE_BASE = 32'h0000_0000,
    parameter [ADDR_WIDTH-1:0] CACHEABLE_SIZE = 32'h4100_0000,
    parameter EXTERNAL_WRITE_RESET_RECOVERY = 0,
    // l2c_victim depth = how many dirty writebacks may be outstanding.
    // Power of two >= 2.  8 is the measured knee on the deterministic
    // model in tb_l2c.cpp: it is what turns conflict-eviction traffic from
    // 0.746 x DDR_write_latency per op into roughly
    // DDR_write_latency / VICTIM_SLOTS per op.  Deeper keeps helping while
    // DRAM latency stays above ~8 cycles per writeback, at the cost of one
    // more line-address comparator in l2c_ctrl's hit-resolve cone -- see
    // l2c_victim.v's SLOTS comment.
    parameter VICTIM_SLOTS = 8,
    // l2c_bypass depth = how many bypass-window beats may be outstanding.
    // Power of two, 1..32.  l2c_ctrl decomposes a bypass burst into one
    // request PER 16 B BEAT, so this is what decides whether a 512 B
    // RAM-disk transfer costs 32 serialized DDR round trips or four.
    //
    // 8 IS NOT WHERE THE ISOLATED MODEL SATURATES.  It keeps improving all
    // the way to 32, because the floor for a 32-beat transfer is one whole
    // burst in flight at once (docs/l2c_perf.md S14.3 has the curve).  8 is
    // where the SYSTEM saturates: axi_ddr4_mig_bridge accepts
    // RMAX_OUTSTANDING = 8 reads and WMAX_OUTSTANDING = 4 writes
    // (docs/ddr4_mig_bridge_contract.md), so past 8 the extra slots only
    // move the queue to the other side of that boundary.  Raising this is
    // therefore a PAIRED change with the bridge's own caps, never a
    // standalone one.
    //
    // Area is not the constraint and should not be used as one: both of
    // l2c_bypass's payload arrays are one LUT6 per bit at any depth up to
    // 32, and the engine adds NOTHING to l2c_ctrl's cones at any depth --
    // it is queried by ONE id comparator (byp_active_id == cur_id), which
    // is what the one-id-at-a-time rule buys.  That is the structural
    // difference from VICTIM_SLOTS above, whose per-slot line-address CAM
    // does feed hit-resolve and is why THAT one stopped at 8 for a
    // timing reason.
    parameter BYPASS_SLOTS = 8,
    // Per-MSHR-entry secondary-merge FIFO depth (+ its index width,
    // ceil(log2), minimum 1).  Default 4 = the shipped v1 value, kept so
    // this parameter cannot change any existing build by accident.
    //
    // SIZING (docs/l2c_perf.md S5.5): 2 is measured-correct -- the full
    // tb passes at MSHR_REPLAY_N=2/K=1 under a requester running 24
    // concurrent transactions across 59 distinct AXI IDs -- and would save
    // an estimated ~1.0-1.5 K LUT / ~2.3 K FF.  DO NOT TAKE IT YET.
    //
    // An earlier revision of this comment argued 2 was provably sufficient
    // because every master reaching xbar S0 is one-outstanding with a
    // constant AXI ID, so at most 2 secondaries could ever queue behind
    // one primary.  That argument is WITHDRAWN: the core this SoC exists
    // to host (m68k-core-040-ooo) is multi-outstanding on both its I and D
    // side, so many distinct IDs can target one line at once and the
    // replay FIFO becomes reachable at any depth.  Saturation is a stall
    // (merge_ready deasserts, S_LOOKUP retries), never a wrong answer --
    // but it is a throughput knob now, not free area.  Leave it at 4 until
    // someone measures replay-FIFO occupancy under the new core.
    parameter MSHR_REPLAY_N = 4,
    parameter MSHR_REPLAY_K = 2,
    // Level A (task #269): dedicated fetch (axi_i) read-only port.  ID
    // width matches the CPU socket's native axi_i ID field
    // (CPU_SOCKET_AXI_IW, cpu_socket.vh) -- deliberately NOT ID_WIDTH
    // above, which is l2c's internal/LSU-facing width; the two ID spaces
    // never need to compare against each other outside l2c_ctrl.v's own
    // internal source-qualified compare (see that module).  Sized as the
    // in-flight-quadrant-pair reassembly table's depth (2**F_ID_WIDTH
    // entries) -- see the fetch reassembly block below.
    parameter F_ID_WIDTH = 4
) (
    input  wire clk, rst,
    // AXI4 slave (from xbar S0).
    input  wire [ID_WIDTH-1:0] s_axi_awid, input wire [ADDR_WIDTH-1:0] s_axi_awaddr,
    input  wire [7:0] s_axi_awlen, input wire [2:0] s_axi_awsize, input wire [1:0] s_axi_awburst,
    input  wire s_axi_awvalid, output wire s_axi_awready,
    input  wire [DATA_WIDTH-1:0] s_axi_wdata, input wire [DATA_WIDTH/8-1:0] s_axi_wstrb,
    input  wire s_axi_wlast, input wire s_axi_wvalid, output wire s_axi_wready,
    output wire [ID_WIDTH-1:0] s_axi_bid, output wire [1:0] s_axi_bresp,
    output wire s_axi_bvalid, input wire s_axi_bready,
    input  wire [ID_WIDTH-1:0] s_axi_arid, input wire [ADDR_WIDTH-1:0] s_axi_araddr,
    input  wire [7:0] s_axi_arlen, input wire [2:0] s_axi_arsize, input wire [1:0] s_axi_arburst,
    input  wire s_axi_arvalid, output wire s_axi_arready,
    output wire [ID_WIDTH-1:0] s_axi_rid, output wire [DATA_WIDTH-1:0] s_axi_rdata,
    output wire [1:0] s_axi_rresp, output wire s_axi_rlast, output wire s_axi_rvalid, input wire s_axi_rready,
    // Level A (task #269): dedicated fetch (axi_i) AR/R slave sub-port,
    // 256b, read-only (no AW/W/B -- fetch never writes).  Folds into the
    // SAME single front door as s_axi above as a 3rd arbitration
    // candidate (l2c_ctrl.v), NOT a second independent read pipeline --
    // see l2c_ctrl.v's header / the task's design doc for why that is
    // Level A's whole point (reuses S_WAIT/S_LOOKUP/hit-detect/MSHR
    // as-is, no new hazard class).  A 256b beat is modeled as two
    // sequential 128b quadrant accepts into l2c_ctrl.v (decompose below,
    // mirrors how the AW/W full-line gather already decomposes bursts),
    // reassembled here into one 256b beat (reassembly block below).
    input  wire [F_ID_WIDTH-1:0] f_axi_arid, input wire [ADDR_WIDTH-1:0] f_axi_araddr,
    input  wire [7:0] f_axi_arlen, input wire [2:0] f_axi_arsize, input wire [1:0] f_axi_arburst,
    input  wire f_axi_arvalid, output wire f_axi_arready,
    output wire [F_ID_WIDTH-1:0] f_axi_rid, output wire [255:0] f_axi_rdata,
    output wire [1:0] f_axi_rresp, output wire f_axi_rlast, output wire f_axi_rvalid, input wire f_axi_rready,
    // AXI4 master (toward DDR path).
    output wire [ID_WIDTH-1:0] m_axi_awid, output wire [ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [7:0] m_axi_awlen, output wire [2:0] m_axi_awsize, output wire [1:0] m_axi_awburst,
    output wire m_axi_awvalid, input wire m_axi_awready,
    output wire [DATA_WIDTH-1:0] m_axi_wdata, output wire [DATA_WIDTH/8-1:0] m_axi_wstrb,
    output wire m_axi_wlast, output wire m_axi_wvalid, input wire m_axi_wready,
    input  wire [ID_WIDTH-1:0] m_axi_bid, input wire [1:0] m_axi_bresp,
    input  wire m_axi_bvalid, output wire m_axi_bready,
    output wire [ID_WIDTH-1:0] m_axi_arid, output wire [ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0] m_axi_arlen, output wire [2:0] m_axi_arsize, output wire [1:0] m_axi_arburst,
    output wire m_axi_arvalid, input wire m_axi_arready,
    input  wire [ID_WIDTH-1:0] m_axi_rid, input wire [DATA_WIDTH-1:0] m_axi_rdata,
    input  wire [1:0] m_axi_rresp, input wire m_axi_rlast, input wire m_axi_rvalid, output wire m_axi_rready,
    // 2026-07-24 L2C/MIG-cal-race debug taps — see dbg_l2c_master_snap
    // comment below. dbg_ctrl_write_snap pass-through's l2c_ctrl.v's own
    // front-door state (its dbg_write_snap output).
    output wire [10:0] dbg_ctrl_write_snap,
    output wire [10:0] dbg_master_snap,
    // 2026-07-25 hit/miss/occupancy counters -- pass-through of
    // l2c_ctrl.v's own dbg_hit_count/dbg_miss_count/dbg_mshr_occupancy,
    // see that module for the exact counted events.
    output wire [31:0] dbg_hit_count,
    output wire [31:0] dbg_miss_count,
    output wire [3:0]  dbg_mshr_occupancy,
    // 2026-08-30 boot investigation round 8 (docs/
    // BUG_calibration_word_misplaced_0d00.md Part 54's precise closing
    // recommendation): fetch-specific L2C internal state.  Neither
    // dbg_ctrl_write_snap (S_AXI AW/W only) nor dbg_master_snap (DRAM-side
    // master port, no fetch-vs-LSU source tag) carries this -- see the
    // assign site below for the exact bit layout.  Answers whether an
    // axi_i (fetch) request that reached the L2C front door (f_axi_ar*)
    // ever gets accepted, resolved by hit or MSHR-fill, and drained back
    // out through f_axi_r* to the CPU.
    output wire [15:0] dbg_fetch_snap,
    // 2026-08-31 boot investigation round 10 (docs/
    // BUG_calibration_word_misplaced_0d00.md Part 56's precise closing
    // recommendation): a direct tap on which AXI ID the fetch-reassembly
    // output skid register (`f_rid_q`) is CURRENTLY parked on, alongside
    // the ID of whatever fetch-tagged hit response is live at the front
    // door THIS cycle (`hit_rsp_id`/`fetch_q_id`) -- see the assign site
    // below for the exact bit layout. Cross-referenced against cpu040's
    // new per-slot generation-counter taps (`icMshrGen`/`icMshrArSentGen`)
    // to settle whether the permanently-parked, undrained response
    // belongs to an ID L2C is STILL trying to serve a fresh request for
    // (would point at an L2C-side response-tracking defect) or whether
    // L2C's front door has gone completely quiet on that ID (consistent
    // with cpu040 having moved on to a new generation without draining
    // the old one -- an L2C-external, cpu040-side trigger).
    output wire [15:0] dbg_fetch_id_snap
);
generate
if (L2_BYPASS_ALL != 0) begin : g_bypass_all
    // Cache machinery isn't instantiated in this branch at all, so clk/rst
    // are structurally unused here -- give them a harmless sink.
    /* verilator lint_off UNUSEDSIGNAL */
    wire unused_clk_rst = clk ^ rst;
    /* verilator lint_on UNUSEDSIGNAL */
    assign s_axi_awready = m_axi_awready; assign m_axi_awid = s_axi_awid; assign m_axi_awaddr = s_axi_awaddr;
    assign m_axi_awlen = s_axi_awlen; assign m_axi_awsize = s_axi_awsize; assign m_axi_awburst = s_axi_awburst;
    assign m_axi_awvalid = s_axi_awvalid;
    assign s_axi_wready = m_axi_wready; assign m_axi_wdata = s_axi_wdata; assign m_axi_wstrb = s_axi_wstrb;
    assign m_axi_wlast = s_axi_wlast; assign m_axi_wvalid = s_axi_wvalid;
    assign m_axi_bready = s_axi_bready; assign s_axi_bid = m_axi_bid; assign s_axi_bresp = m_axi_bresp;
    assign s_axi_bvalid = m_axi_bvalid; assign s_axi_arready = m_axi_arready; assign m_axi_arid = s_axi_arid;
    assign m_axi_araddr = s_axi_araddr; assign m_axi_arlen = s_axi_arlen; assign m_axi_arsize = s_axi_arsize;
    assign m_axi_arburst = s_axi_arburst; assign m_axi_arvalid = s_axi_arvalid;
    assign m_axi_rready = s_axi_rready; assign s_axi_rid = m_axi_rid; assign s_axi_rdata = m_axi_rdata;
    assign s_axi_rresp = m_axi_rresp; assign s_axi_rlast = m_axi_rlast; assign s_axi_rvalid = m_axi_rvalid;
    // L2_BYPASS_ALL has no tag pipeline to fold a 3rd source into and no
    // m_axi widening story for a 256b passthrough -- the fetch port is
    // simply held inert here (never accepts).  L2_BYPASS_ALL is a
    // bringup escape hatch, not a shipping configuration this port needs
    // to serve.
    assign f_axi_arready = 1'b0;
    assign f_axi_rvalid  = 1'b0;
    assign f_axi_rid     = {F_ID_WIDTH{1'b0}};
    assign f_axi_rdata   = 256'd0;
    assign f_axi_rresp   = 2'b00;
    assign f_axi_rlast   = 1'b0;
    assign dbg_ctrl_write_snap = 11'd0;
    assign dbg_master_snap     = 11'd0;
    assign dbg_hit_count       = 32'd0;
    assign dbg_miss_count      = 32'd0;
    assign dbg_mshr_occupancy  = 4'd0;
    assign dbg_fetch_snap      = 16'd0;
    assign dbg_fetch_id_snap   = 16'd0;
end else begin : g_active
    // -- l2c_ctrl (cache proper) --------------------------------------------
    wire hit_rsp_valid, hit_rsp_is_write, hit_rsp_is_fetch, hit_rsp_last, hit_rsp_ready; wire [1:0] hit_rsp_resp;
    wire [ID_WIDTH-1:0] hit_rsp_id; wire [127:0] hit_rsp_rdata;
    wire mshr_rsp_valid, mshr_rsp_is_write, mshr_rsp_is_fetch, mshr_rsp_last, mshr_rsp_ready; wire [1:0] mshr_rsp_resp;
    wire [ID_WIDTH-1:0] mshr_rsp_id; wire [127:0] mshr_rsp_rdata;
    // Level A (task #269): l2c_ctrl.v's fetch-side AR sub-port.  Decompose
    // logic below translates f_axi_ar* (256b, this module's external
    // port) into this 128b-quadrant-granular descriptor: arlen doubles
    // (+1 for the extra low bit, i.e. (f_axi_arlen+1)*2 - 1 quadrants),
    // arsize is pinned to 4 (16B/128b) regardless of what f_axi_arsize
    // says -- axi_i is defined (cpu_socket.vh) to always present native
    // 256b beats here, so the caller's arsize is not re-validated, only
    // reinterpreted.  ID is zero-extended from F_ID_WIDTH up to ID_WIDTH
    // (l2c_ctrl.v mirrors s_ar*'s ID_WIDTH shape exactly); the extra high
    // bits are always 0 so no numeric collision with a real LSU ID is
    // possible at that port -- the actual source-disambiguation happens
    // inside l2c_ctrl.v's own MSHR id-qualification, not here.
    wire [ID_WIDTH-1:0] fctrl_arid   = {{(ID_WIDTH-F_ID_WIDTH){1'b0}}, f_axi_arid};
    wire [ADDR_WIDTH-1:0] fctrl_araddr = f_axi_araddr;
    // fctrl_arlen = 2*(f_axi_arlen+1) - 1 = 2*f_axi_arlen + 1, the AXI4
    // ARLEN encoding of "twice the quadrant count, minus 1".  ARLEN is
    // architecturally 8 bits on BOTH sides of this translation (mirrors
    // s_arlen's shape exactly), so the arithmetic only fits losslessly
    // for f_axi_arlen <= 127 (256 quadrants, translated arlen=255) --
    // only f_axi_arlen's low 7 bits feed the shift; a set bit [7] would
    // silently truncate, so it is asserted against below rather than
    // trusted.  Real axi_i traffic uses arlen=0/1 (a half- or whole-line
    // fetch burst) per the CPU-socket contract, far under this bound.
    wire [7:0] fctrl_arlen  = {f_axi_arlen[6:0], 1'b1};
    wire [2:0] fctrl_arsize = 3'd4;
    wire [1:0] fctrl_arburst = f_axi_arburst;
    wire fctrl_arvalid = f_axi_arvalid;
    wire fctrl_arready;
    assign f_axi_arready = fctrl_arready;
    // synthesis translate_off
    always @(posedge clk) begin
        if (!rst && f_axi_arvalid && f_axi_arready && f_axi_arlen[7]) begin
            $display("L2C ASSERT: fetch AR arlen=%0d exceeds the 128-bit-quadrant decompose's representable range (max 127)", f_axi_arlen);
            $fatal(1);
        end
    end
    // synthesis translate_on
    wire victim_push_valid, victim_push_ready, victim_query_hit;
    wire [ADDR_WIDTH-1:0] victim_push_addr, victim_query_addr; wire [511:0] victim_push_data;
    wire [3:0] victim_push_dsec;
    wire byp_match_hit; wire [ADDR_WIDTH-1:0] byp_match_addr;
    wire byp_match2_hit; wire [ADDR_WIDTH-1:0] byp_match2_addr;
    wire byp_req_valid, byp_req_is_write, byp_req_need_resp, byp_req_last, byp_req_ready;
    wire [ADDR_WIDTH-1:0] byp_req_addr; wire [ID_WIDTH-1:0] byp_req_id;
    wire [127:0] byp_req_wdata; wire [15:0] byp_req_wstrb;
    wire byp_active_valid; wire [ID_WIDTH-1:0] byp_active_id;
    wire [ID_WIDTH-1:0] mf_arid; wire [ADDR_WIDTH-1:0] mf_araddr; wire [7:0] mf_arlen;
    wire [2:0] mf_arsize; wire [1:0] mf_arburst; wire mf_arvalid, mf_arready;
    wire [ID_WIDTH-1:0] mf_rid; wire [127:0] mf_rdata; wire [1:0] mf_rresp;
    wire mf_rlast, mf_rvalid, mf_rready;
    l2c_ctrl #(.SET_BITS(12), .TAG_BITS(14), .WAY_BITS(3), .ADDR_WIDTH(ADDR_WIDTH),
               .ID_WIDTH(ID_WIDTH), .LINE_BITS(512),
               .MSHR_REPLAY_N(MSHR_REPLAY_N), .MSHR_REPLAY_K(MSHR_REPLAY_K)) u_ctrl (
        .clk(clk), .rst(rst),
        .s_awid(s_axi_awid), .s_awaddr(s_axi_awaddr), .s_awlen(s_axi_awlen), .s_awsize(s_axi_awsize),
        .s_awburst(s_axi_awburst), .s_awvalid(s_axi_awvalid), .s_awready(s_axi_awready),
        .s_wdata(s_axi_wdata), .s_wstrb(s_axi_wstrb), .s_wlast(s_axi_wlast), .s_wvalid(s_axi_wvalid), .s_wready(s_axi_wready),
        .s_arid(s_axi_arid), .s_araddr(s_axi_araddr), .s_arlen(s_axi_arlen), .s_arsize(s_axi_arsize),
        .s_arburst(s_axi_arburst), .s_arvalid(s_axi_arvalid), .s_arready(s_axi_arready),
        .f_arid(fctrl_arid), .f_araddr(fctrl_araddr), .f_arlen(fctrl_arlen), .f_arsize(fctrl_arsize),
        .f_arburst(fctrl_arburst), .f_arvalid(fctrl_arvalid), .f_arready(fctrl_arready),
        .hit_rsp_valid(hit_rsp_valid), .hit_rsp_is_write(hit_rsp_is_write), .hit_rsp_is_fetch(hit_rsp_is_fetch), .hit_rsp_id(hit_rsp_id),
        .hit_rsp_rdata(hit_rsp_rdata), .hit_rsp_last(hit_rsp_last), .hit_rsp_resp(hit_rsp_resp),
        .hit_rsp_ready(hit_rsp_ready),
        .mshr_rsp_valid(mshr_rsp_valid), .mshr_rsp_is_write(mshr_rsp_is_write), .mshr_rsp_is_fetch(mshr_rsp_is_fetch), .mshr_rsp_id(mshr_rsp_id),
        .mshr_rsp_rdata(mshr_rsp_rdata), .mshr_rsp_last(mshr_rsp_last), .mshr_rsp_resp(mshr_rsp_resp),
        .mshr_rsp_ready(mshr_rsp_ready), .byp_active_valid(byp_active_valid), .byp_active_id(byp_active_id),
        .victim_push_valid(victim_push_valid), .victim_push_addr(victim_push_addr),
        .victim_push_data(victim_push_data), .victim_push_dsec(victim_push_dsec), .victim_push_ready(victim_push_ready),
        .victim_query_addr(victim_query_addr), .victim_query_hit(victim_query_hit),
        .byp_match_addr(byp_match_addr), .byp_match_hit(byp_match_hit),
        .byp_match2_addr(byp_match2_addr), .byp_match2_hit(byp_match2_hit),
        .byp_req_valid(byp_req_valid), .byp_req_is_write(byp_req_is_write), .byp_req_addr(byp_req_addr),
        .byp_req_id(byp_req_id), .byp_req_wdata(byp_req_wdata), .byp_req_wstrb(byp_req_wstrb),
        .byp_req_need_resp(byp_req_need_resp), .byp_req_last(byp_req_last), .byp_req_ready(byp_req_ready),
        .mf_arid(mf_arid), .mf_araddr(mf_araddr), .mf_arlen(mf_arlen), .mf_arsize(mf_arsize),
        .mf_arburst(mf_arburst), .mf_arvalid(mf_arvalid), .mf_arready(mf_arready),
        .mf_rid(mf_rid), .mf_rdata(mf_rdata), .mf_rresp(mf_rresp), .mf_rlast(mf_rlast),
        .mf_rvalid(mf_rvalid), .mf_rready(mf_rready),
        .dbg_write_snap(dbg_ctrl_write_snap),
        .dbg_hit_count(dbg_hit_count), .dbg_miss_count(dbg_miss_count),
        .dbg_mshr_occupancy(dbg_mshr_occupancy)
    );
    // -- l2c_victim (dirty-eviction writeback) -------------------------------
    wire [ID_WIDTH-1:0] vw_awid; wire [ADDR_WIDTH-1:0] vw_awaddr; wire [7:0] vw_awlen;
    wire [2:0] vw_awsize; wire [1:0] vw_awburst; wire vw_awvalid, vw_awready;
    wire [127:0] vw_wdata; wire [15:0] vw_wstrb; wire vw_wlast, vw_wvalid, vw_wready;
    wire [ID_WIDTH-1:0] vw_bid; wire [1:0] vw_bresp; wire vw_bvalid, vw_bready;
    l2c_victim #(.ADDR_WIDTH(ADDR_WIDTH), .LINE_BITS(512), .ID_WIDTH(ID_WIDTH),
                 .SLOTS(VICTIM_SLOTS),
                 .EXTERNAL_RESET_RECOVERY(EXTERNAL_WRITE_RESET_RECOVERY)) u_victim (
        .clk(clk), .rst(rst),
        .push_valid(victim_push_valid), .push_ready(victim_push_ready),
        .push_addr(victim_push_addr), .push_data(victim_push_data), .push_dsec(victim_push_dsec),
        .query_addr(victim_query_addr), .query_hit(victim_query_hit),
        .m_awid(vw_awid), .m_awaddr(vw_awaddr), .m_awlen(vw_awlen), .m_awsize(vw_awsize),
        .m_awburst(vw_awburst), .m_awvalid(vw_awvalid), .m_awready(vw_awready),
        .m_wdata(vw_wdata), .m_wstrb(vw_wstrb), .m_wlast(vw_wlast), .m_wvalid(vw_wvalid), .m_wready(vw_wready),
        .m_bid(vw_bid), .m_bresp(vw_bresp), .m_bvalid(vw_bvalid), .m_bready(vw_bready)
    );
    // -- l2c_bypass -----------------------------------------------------------
    wire [ID_WIDTH-1:0] by_awid; wire [ADDR_WIDTH-1:0] by_awaddr; wire [7:0] by_awlen;
    wire [2:0] by_awsize; wire [1:0] by_awburst; wire by_awvalid, by_awready;
    wire [127:0] by_wdata; wire [15:0] by_wstrb; wire by_wlast, by_wvalid, by_wready;
    wire [ID_WIDTH-1:0] by_bid; wire [1:0] by_bresp; wire by_bvalid, by_bready;
    wire [ID_WIDTH-1:0] by_arid; wire [ADDR_WIDTH-1:0] by_araddr; wire [7:0] by_arlen;
    wire [2:0] by_arsize; wire [1:0] by_arburst; wire by_arvalid, by_arready;
    wire [ID_WIDTH-1:0] by_rid; wire [127:0] by_rdata; wire [1:0] by_rresp;
    wire by_rlast, by_rvalid, by_rready;
    wire byp_rsp_valid, byp_rsp_is_write, byp_rsp_ready, byp_rsp_last; wire [ID_WIDTH-1:0] byp_rsp_id;
    wire [127:0] byp_rsp_rdata; wire [1:0] byp_rsp_resp;
    l2c_bypass #(.ADDR_WIDTH(ADDR_WIDTH), .ID_WIDTH(ID_WIDTH), .NUM_WINDOWS(NUM_BYPASS_WINDOWS),
                 .WIN_BASE(BYP_WIN_BASE), .WIN_MASK(BYP_WIN_MASK), .WIN_EN(BYP_WIN_EN),
                 .CACHEABLE_BASE(CACHEABLE_BASE), .CACHEABLE_SIZE(CACHEABLE_SIZE),
                 .SLOTS(BYPASS_SLOTS),
                 .EXTERNAL_RESET_RECOVERY(EXTERNAL_WRITE_RESET_RECOVERY)) u_bypass (
        .clk(clk), .rst(rst),
        .match_addr(byp_match_addr), .match_hit(byp_match_hit),
        .match_addr2(byp_match2_addr), .match_hit2(byp_match2_hit),
        .req_valid(byp_req_valid), .req_ready(byp_req_ready), .req_is_write(byp_req_is_write),
        .req_addr(byp_req_addr), .req_id(byp_req_id), .req_wdata(byp_req_wdata), .req_wstrb(byp_req_wstrb),
        .req_need_resp(byp_req_need_resp), .req_last(byp_req_last),
        .rsp_valid(byp_rsp_valid), .rsp_ready(byp_rsp_ready), .rsp_is_write(byp_rsp_is_write),
        .rsp_id(byp_rsp_id), .rsp_rdata(byp_rsp_rdata), .rsp_resp(byp_rsp_resp), .rsp_last(byp_rsp_last),
        .active_valid(byp_active_valid), .active_id(byp_active_id),
        .m_awid(by_awid), .m_awaddr(by_awaddr), .m_awlen(by_awlen), .m_awsize(by_awsize),
        .m_awburst(by_awburst), .m_awvalid(by_awvalid), .m_awready(by_awready),
        .m_wdata(by_wdata), .m_wstrb(by_wstrb), .m_wlast(by_wlast), .m_wvalid(by_wvalid), .m_wready(by_wready),
        .m_bid(by_bid), .m_bresp(by_bresp), .m_bvalid(by_bvalid), .m_bready(by_bready),
        .m_arid(by_arid), .m_araddr(by_araddr), .m_arlen(by_arlen), .m_arsize(by_arsize),
        .m_arburst(by_arburst), .m_arvalid(by_arvalid), .m_arready(by_arready),
        .m_rid(by_rid), .m_rdata(by_rdata), .m_rresp(by_rresp), .m_rlast(by_rlast),
        .m_rvalid(by_rvalid), .m_rready(by_rready)
    );
    // -- Pipelined AR/R merge: bypass > MSHR fill at each AR grant ----------
    // The downstream DDR path returns bursts in accepted-AR order. Keep a
    // source bit per accepted transaction, allowing many fill/bypass reads
    // in flight while routing every R burst back to its owner. The
    // one-entry presentation hold is solely an AXI payload-stability skid;
    // it does not serialize accepted transactions through RLAST.
    //
    // DEPTH (2026-08-20).  This queue is the SHARED cap on read
    // concurrency, so it has to cover both sources at once or it silently
    // becomes the limiter instead of either engine: the MSHR fill path can
    // have up to 8 fills in flight (one per entry) and l2c_bypass can now
    // have BYPASS_SLOTS beats in flight.  It was a flat 8 back when bypass
    // was single-outstanding; sizing it to the sum keeps the two
    // independent, and costs 1 bit of LUTRAM per entry and nothing else.
    localparam RDQ_W     = (BYPASS_SLOTS <= 8)  ? 4 :
                           (BYPASS_SLOTS <= 24) ? 5 : 6;
    localparam RDQ_DEPTH = (1 << RDQ_W);
    reg rdq_src [0:RDQ_DEPTH-1]; // 0 = MSHR fill, 1 = bypass
    reg [RDQ_W-1:0] rdq_wptr, rdq_rptr;
    reg [RDQ_W:0]   rdq_count;
    reg ar_hold_valid, ar_hold_src;
    localparam [RDQ_W:0]   RDQ_DEPTH_V = RDQ_DEPTH;
    localparam [RDQ_W:0]   RDQ_ZERO    = {(RDQ_W+1){1'b0}};
    localparam [RDQ_W:0]   RDQ_ONE     = {{RDQ_W{1'b0}}, 1'b1};
    localparam [RDQ_W-1:0] RDQ_PONE    = {{(RDQ_W-1){1'b0}}, 1'b1};
    wire rdq_full  = (rdq_count == RDQ_DEPTH_V);
    wire rdq_empty = (rdq_count == RDQ_ZERO);
    wire ar_pick_c = by_arvalid;
    wire ar_src_c  = ar_hold_valid ? ar_hold_src : ar_pick_c;
    wire ar_src_valid_c = ar_src_c ? by_arvalid : mf_arvalid;

    // ================= MASTER AR REGISTER SLICE (2026-09-15) ==============
    //
    // The AR channel used to leave this module combinationally, and that
    // made the ONE remaining violated path whose SOURCE is inside u_l2c:
    //
    //   u_l2c/.../u_mshr/m_v_reg[1]/C -> issue_rr_ptr -> issue_vec_rot_c
    //   -> m_arid[1] (fo=69, 0.947 ns) -> <this mux> -> u_vram_lane_mux
    //   -> u_ddr/u_core_to_mig_ui/ar_fifo/mem_reg_0_3_0_13/RAMB/I
    //
    //   -0.655 ns, 5.382 ns of data path, 82.5% of it ROUTE.  The MSHR's
    //   round-robin issue arbiter picks an entry and that pick has to reach
    //   the memory controller's AR FIFO -- across two intervening modules
    //   and most of the die -- inside the same cycle.  2.3 ns of it is
    //   spent after the address has already left this mux.
    //
    // The slice cuts it in two: arbiter -> slice input (~4.2 ns, inside
    // u_l2c) and slice output -> u_ddr (~1.4 ns, pure inter-module route
    // with no logic in it).
    //
    // COST: one cycle on the issue of a fill/bypass READ address.  A miss
    // already costs tens of cycles of DRAM latency, and throughput is
    // unaffected (registered-output 2-deep slice, one beat per cycle).
    //
    // ORDERING.  The slice is AFTER the bypass-vs-fill mux, so both sources
    // are delayed identically and their relative order is preserved; and
    // `rdq`, the queue that routes each read RESPONSE back to the source
    // that asked for it, now pushes on the slice's INPUT handshake rather
    // than on m_axi's.  That is the same event one cycle earlier, and the
    // slice cannot reorder what it only delays, so the queue still lines up
    // with the AR stream the memory controller sees.
    //
    // Only AR is sliced, deliberately.  Delaying a READ relative to a
    // WRITE is the safe direction for the one same-address ordering this
    // cache depends on -- a dirty line's writeback (AW/W) must land before
    // anything re-fetches that address (AR) -- so this makes that ordering
    // strictly wider, never narrower.  Slicing AW would narrow it.
    wire iar_valid = ar_src_valid_c && !rdq_full;
    wire iar_ready;
    l2c_rsp_skid #(.W(ID_WIDTH + ADDR_WIDTH + 8 + 3 + 2)) u_ar_skid (
        .clk(clk), .rst(rst),
        .s_valid(iar_valid), .s_ready(iar_ready),
        .s_data({ar_src_c ? by_arid    : mf_arid,
                 ar_src_c ? by_araddr  : mf_araddr,
                 ar_src_c ? by_arlen   : mf_arlen,
                 ar_src_c ? by_arsize  : mf_arsize,
                 ar_src_c ? by_arburst : mf_arburst}),
        .m_valid(m_axi_arvalid), .m_ready(m_axi_arready),
        .m_data({m_axi_arid, m_axi_araddr, m_axi_arlen, m_axi_arsize, m_axi_arburst})
    );
    assign mf_arready = !ar_src_c && !rdq_full && iar_ready;
    assign by_arready =  ar_src_c && !rdq_full && iar_ready;
    wire rdq_push = iar_valid && iar_ready;

    wire rdq_head_src = rdq_src[rdq_rptr];
    // With the route queue empty, feed any late pre-reset response into the
    // MSHR drain state.  A core reset clears this queue while the downstream
    // stale sink may still owe completions; without this fallback RREADY
    // stays low and a directly-connected test slave (or a delayed boundary
    // response) can wedge permanently.
    wire rdq_route_mf = rdq_empty ? mf_rready : !rdq_head_src;
    assign mf_rvalid = rdq_route_mf && m_axi_rvalid;
    assign mf_rid = m_axi_rid; assign mf_rdata = m_axi_rdata;
    assign mf_rresp = m_axi_rresp; assign mf_rlast = m_axi_rlast;
    assign by_rvalid = !rdq_empty && rdq_head_src && m_axi_rvalid;
    assign by_rid = m_axi_rid; assign by_rdata = m_axi_rdata;
    assign by_rresp = m_axi_rresp; assign by_rlast = m_axi_rlast;
    assign m_axi_rready = rdq_empty ? mf_rready :
                          (rdq_head_src ? by_rready : mf_rready);
    wire rdq_pop = !rdq_empty && m_axi_rvalid && m_axi_rready && m_axi_rlast;

    always @(posedge clk) begin
        if (rst) begin
            rdq_wptr <= {RDQ_W{1'b0}}; rdq_rptr <= {RDQ_W{1'b0}}; rdq_count <= RDQ_ZERO;
            ar_hold_valid <= 1'b0; ar_hold_src <= 1'b0;
        end else begin
            // 2026-09-15: the handshake observed here is the AR register
            // slice's input, not the m_axi pins -- same stream, one cycle
            // earlier, and it is the one this arbiter actually drives.
            if (!ar_hold_valid && !rdq_full && ar_src_valid_c && !iar_ready) begin
                ar_hold_valid <= 1'b1;
                ar_hold_src <= ar_pick_c;
            end else if (rdq_push) begin
                ar_hold_valid <= 1'b0;
            end
            if (rdq_push) begin
                rdq_src[rdq_wptr] <= ar_src_c;
                rdq_wptr <= rdq_wptr + RDQ_PONE;
            end
            if (rdq_pop) rdq_rptr <= rdq_rptr + RDQ_PONE;
            case ({rdq_push, rdq_pop})
                2'b10: rdq_count <= rdq_count + RDQ_ONE;
                2'b01: rdq_count <= rdq_count - RDQ_ONE;
                default: rdq_count <= rdq_count;
            endcase
        end
    end
    // -- AW/W/B master-port arbiter: bypass > victim-writeback ---------------
    // 2026-08-20: this arbiter used to hold its grant from AW-PRESENTATION
    // all the way to B, which serialized the write port to ONE transaction
    // at a time and made l2c_victim's second slot decorative.  Measured cost
    // on conflict-eviction traffic (54919fe): cycles/op = 3.0 + 0.746 x
    // DDR_write_latency, i.e. 32.85 at DDR-40 and 152.22 at DDR-200 against
    // a 3.75 floor.  It now tracks completions BY COUNT instead.
    //
    // What still has to be exclusive, and why:
    //
    //   * AXI4 has no WID.  W bursts must reach the slave in AW-acceptance
    //     order, so two sources may never have write transactions in flight
    //     at the same time -- one of them would have to interleave its W
    //     beats into the middle of the other's burst.  `aw_out` (AW-accepted
    //     minus B-accepted for the CURRENT owner) is therefore part of the
    //     lock, not just the presentation.  A W burst always has its own AW
    //     accepted and its B not yet returned, so aw_out != 0 covers the
    //     whole burst without needing a separate mid-burst flag.
    //   * Critical-4: the payload mux must not swap under an already-VALID
    //     AW.  `aw_lock` is the one-cycle presentation lock that does that,
    //     exactly as `aw_busy` used to.
    //
    // What is now allowed: the OWNER may present its next AW as soon as the
    // previous one's W burst is done, without waiting for any B.  For the
    // victim engine that is the entire win.
    //
    // STARVATION.  Bypass keeps priority, but it can no longer simply take
    // the port -- a saturated victim engine might never reach aw_out == 0 on
    // its own.  `vw_present_c` withholds NEW victim AWs the moment bypass
    // asserts awvalid (never mid-presentation: `aw_lock` wins that term), so
    // aw_out drains and bypass acquires the port.  The wait is bounded by
    // the outstanding writebacks' completion, which are pipelined and so
    // complete within roughly one DRAM round trip of each other -- about
    // what bypass waited for under the old B-held grant anyway.
    reg aw_grant;             // current owner: 1 = bypass, 0 = victim
    reg aw_lock;              // an AW is presented and not yet accepted
    // aw_out must hold the LARGEST number of writes either owner can have
    // in flight, and the two owners are independently parameterized.  It
    // was a flat 5 bits back when the deepest was l2c_victim's 8; a
    // BYPASS_SLOTS=32 build then silently wrapped 5'd32 to zero at DDR
    // latencies high enough to actually fill the queue, which dropped
    // owner_locked_c mid-flight and deadlocked the arbiter (found by the
    // depth sweep, not by reasoning).  Sized from the parameters instead:
    // ceil(log2(max))+1 bits, which is 4 at the shipped 8/8 -- one bit
    // NARROWER than the hardcoded value it replaces.
    localparam AW_MAX   = (VICTIM_SLOTS > BYPASS_SLOTS) ? VICTIM_SLOTS : BYPASS_SLOTS;
    localparam AW_OUT_W = (AW_MAX <=  2) ? 2 : (AW_MAX <=  4) ? 3 :
                          (AW_MAX <=  8) ? 4 : (AW_MAX <= 16) ? 5 :
                          (AW_MAX <= 32) ? 6 : 7;
    localparam [AW_OUT_W-1:0] AW_OUT_ZERO = {AW_OUT_W{1'b0}};
    localparam [AW_OUT_W-1:0] AW_OUT_ONE  = {{(AW_OUT_W-1){1'b0}}, 1'b1};
    reg [AW_OUT_W-1:0] aw_out;  // AW-accepted minus B-accepted, current owner
    wire aw_out_nz_c = (aw_out != AW_OUT_ZERO);
    wire owner_locked_c = aw_lock || aw_out_nz_c;
    wire aw_grant_live_c = owner_locked_c ? aw_grant : by_awvalid;
    // The victim may hold a presentation it already started (aw_lock), but
    // may not START a new one while bypass wants the port.
    wire vw_present_c = vw_awvalid && (aw_lock || !by_awvalid);
    always @(posedge clk) begin
        // IMPORTANT-A (v2, generalised): hold a VICTIM-locked grant across
        // reset only while real AW-ACCEPTED-to-B transactions are open
        // (`aw_out != 0`) -- NOT for a mere presentation (`aw_lock`, the
        // Critical-4 lock).  Reset while the victim sits in S_AW unaccepted
        // (routine backpressure) sets aw_lock but DRAM never committed to
        // anything; l2c_victim.v resets clean to idle there (no B ever
        // coming), so holding on the presentation alone waited forever --
        // HOL deadlock (probe-confirmed).  Truly-open transactions ARE
        // drained by S_RSTDRAINW/S_RSTSINKB and DO produce the real Bs this
        // block's release condition consumes -- now possibly several of
        // them, which is why aw_out is a count and is carried across reset
        // rather than cleared.
        //
        // 2026-08-20: this release condition used to be
        // `!(aw_out_nz_c && !aw_grant)` -- i.e. it held the grant across
        // reset ONLY for the victim.  That asymmetry was correct while
        // l2c_bypass reset straight to idle with no sink of its own:
        // holding a BYPASS-locked grant would then have waited forever for
        // Bs nobody was going to accept.  l2c_bypass now carries the same
        // bout_cnt/bsink_cnt drain-and-sink machinery as l2c_victim (it
        // has to -- it can have several writes outstanding now), so the
        // condition is symmetric.  It has to be: with the old form, a
        // reset landing on outstanding BYPASS writes flipped aw_grant to
        // victim, and the strays DRAM still owed were then routed to
        // l2c_victim, whose own out_cnt gate would eventually accept one
        // as if it were a writeback's completion and retire a slot whose
        // dirty line had not reached DRAM.
        if (rst) begin
            aw_lock <= 1'b0;
            if ((EXTERNAL_WRITE_RESET_RECOVERY != 0) || !aw_out_nz_c) begin
                aw_grant <= 1'b0; aw_out <= AW_OUT_ZERO;
            end
        end else begin
            if (!owner_locked_c) begin
                if (m_axi_awvalid) begin
                    aw_grant <= aw_grant_live_c;
                    aw_lock  <= !m_axi_awready;
                end
            end else begin
                if (aw_lock) begin
                    if (m_axi_awready) aw_lock <= 1'b0;
                end else if (m_axi_awvalid && !m_axi_awready) begin
                    aw_lock <= 1'b1;
                end
            end
            case ({m_axi_awvalid && m_axi_awready, m_axi_bvalid && m_axi_bready})
                2'b10:   aw_out <= aw_out + AW_OUT_ONE;
                2'b01:   aw_out <= aw_out - AW_OUT_ONE;
                default: aw_out <= aw_out;   // 00, or 11 = net zero
            endcase
        end
    end
    assign m_axi_awid = aw_grant_live_c ? by_awid : vw_awid;
    assign m_axi_awaddr = aw_grant_live_c ? by_awaddr : vw_awaddr;
    assign m_axi_awlen = aw_grant_live_c ? by_awlen : vw_awlen;
    assign m_axi_awsize = aw_grant_live_c ? by_awsize : vw_awsize;
    assign m_axi_awburst = aw_grant_live_c ? by_awburst : vw_awburst;
    assign m_axi_awvalid = aw_grant_live_c ? by_awvalid : vw_present_c;
    assign vw_awready = !aw_grant_live_c && vw_present_c && m_axi_awready;
    assign by_awready  = aw_grant_live_c && m_axi_awready;
    assign m_axi_wdata = aw_grant_live_c ? by_wdata : vw_wdata;
    assign m_axi_wstrb = aw_grant_live_c ? by_wstrb : vw_wstrb;
    assign m_axi_wlast = aw_grant_live_c ? by_wlast : vw_wlast;
    assign m_axi_wvalid = aw_grant_live_c ? by_wvalid : vw_wvalid;
    assign vw_wready = !aw_grant_live_c && m_axi_wready;
    assign by_wready  = aw_grant_live_c && m_axi_wready;

    // 2026-07-24: real-HW investigation companion to l2c_ctrl.v's
    // dbg_l2c_write_snap. Leading theory: boot_fsm's first write is a
    // cold write-miss (write granularity 16B < LINE_BITS=512 = 64B), so
    // it needs a miss-fill READ from DRAM (mf_ar*/mf_r* below) before
    // the line can be merged/marked dirty -- and L2C has no dependency
    // on ddr_cal_done anywhere, so that fill-read can be issued before
    // MIG has actually finished calibrating, hanging forever waiting
    // for m_axi_arready/m_axi_rvalid that never come.
    assign dbg_master_snap = {
        m_axi_awvalid,  // [10]
        m_axi_awready,  // [9]
        m_axi_wvalid,   // [8]
        m_axi_wready,   // [7]
        m_axi_bvalid,   // [6]
        aw_grant,       // [5]
        aw_lock,        // [4] -- was aw_busy (AW presentation lock)
        aw_out_nz_c,    // [3] -- was aw_open (>=1 write open to B)
        mf_arvalid,     // [2]
        mf_arready,     // [1]
        mf_rvalid       // [0]
    };
    assign vw_bvalid = !aw_grant_live_c && m_axi_bvalid; assign vw_bid = m_axi_bid; assign vw_bresp = m_axi_bresp;
    assign by_bvalid = aw_grant_live_c && m_axi_bvalid; assign by_bid = m_axi_bid; assign by_bresp = m_axi_bresp;
    assign m_axi_bready = aw_grant_live_c ? by_bready : vw_bready;
    // -- Response arbiter: bypass > mshr > hit-path, LOCKED (Critical-5) -----
    // Bypass first for the same starvation-avoidance reason as the two
    // master-port arbiters above.  r_pick_c/b_pick_c are the sole
    // combinational decisions (fresh pick while idle, else held lock),
    // used both for this cycle's mux and what gets registered --
    // prevents a higher-priority response swapping s_axi_r*/b_* payload
    // out from under an already-VALID lower-priority one.
    localparam RSP_NONE=2'd0, RSP_BYP=2'd1, RSP_MSHR=2'd2, RSP_HIT=2'd3;
    reg [1:0] r_lock, b_lock;
    // Level A (task #269): hit_rsp_valid/mshr_rsp_valid are SHARED
    // signals now -- a fetch-sourced completion can occupy the same skid
    // register / MSHR-rsp channel an LSU one would.  Excluding
    // *_is_fetch here (and routing those instead through the new fetch
    // response path below) is what keeps a fetch-tagged completion from
    // being presented on s_axi_r*, the LSU-facing bus.
    wire [1:0] r_pick_c = (r_lock != RSP_NONE) ? r_lock :
                          (byp_rsp_valid && !byp_rsp_is_write)  ? RSP_BYP  :
                          (mshr_rsp_valid && !mshr_rsp_is_write && !mshr_rsp_is_fetch) ? RSP_MSHR :
                          (hit_rsp_valid && !hit_rsp_is_write && !hit_rsp_is_fetch)   ? RSP_HIT  : RSP_NONE;
    wire [1:0] b_pick_c = (b_lock != RSP_NONE) ? b_lock :
                          (byp_rsp_valid && byp_rsp_is_write)  ? RSP_BYP  :
                          (mshr_rsp_valid && mshr_rsp_is_write) ? RSP_MSHR :
                          (hit_rsp_valid && hit_rsp_is_write)   ? RSP_HIT  : RSP_NONE;
    // The accept-clears-the-lock check MUST have priority over (not
    // "else" alongside) the lock-in check, on the SAME cycle a fresh
    // pick is made: s_axi_rready is often held high, so a freshly-picked
    // response can be accepted that cycle -- else the NEXT cycle would
    // read a stale lock after the source's own *_rsp_valid dropped,
    // presenting a phantom duplicate R/B (corrupts a later same-ID op).
    always @(posedge clk) begin
        if (rst) begin r_lock <= RSP_NONE; b_lock <= RSP_NONE; end
        else begin
            // 2026-09-15: the handshake this observes is the INTERNAL one
            // (ir_*/ib_*), i.e. the arbiter's own output into the response
            // register slice, not the s_axi pins.  Same stream, one cycle
            // earlier; the lock must track where the payload mux is, and
            // the payload mux is upstream of the slice.
            if (ir_valid && ir_ready) r_lock <= RSP_NONE;
            else if (r_lock == RSP_NONE && r_pick_c != RSP_NONE) r_lock <= r_pick_c;
            if (ib_valid && ib_ready) b_lock <= RSP_NONE;
            else if (b_lock == RSP_NONE && b_pick_c != RSP_NONE) b_lock <= b_pick_c;
        end
    end
    wire r_grant_byp = (r_pick_c == RSP_BYP), r_grant_mshr = (r_pick_c == RSP_MSHR), r_grant_hit = (r_pick_c == RSP_HIT);
    wire b_grant_byp = (b_pick_c == RSP_BYP), b_grant_mshr = (b_pick_c == RSP_MSHR), b_grant_hit = (b_pick_c == RSP_HIT);
    // ================== SLAVE RESPONSE REGISTER SLICE ====================
    //
    // 2026-09-15 (FMax).  The R and B channels no longer leave this module
    // combinationally.  `ir_*`/`ib_*` are the INTERNAL response streams the
    // arbiter above drives; two registered-output skids (l2c_rsp_skid,
    // bottom of this file) carry them to the s_axi_r*/s_axi_b* pins.
    //
    // WHAT WAS MEASURED, and why this is not cosmetic.  On the routed
    // 200 MHz eth netlist (synth/timing_reports/timing_20260915_164315.rpt)
    // FIFTY-TWO of the fifty-five violated setup paths that touch u_l2c
    // enter it at `s_axi_rready` and leave it at a URAM288 ADDR_A pin.
    // They are not L2C paths at all for their first half: they start at a
    // distributed-RAM cell in u_pb_s1_cdc's read-data FIFO (or at
    // u_vram_lane_mux's read-queue pointer), cross u_xbar's response
    // router as `m3_rlast` -> `m3_rready`, and only then arrive here.  The
    // worst is -0.648 ns over 16 logic levels; the representative
    // pb_s1_cdc one is -0.377 ns over 14.
    //
    // What made them L2C's problem is that `s_axi_rready` was a TERM OF
    // THE RESOLVE DECISION.  The measured tail inside u_l2c was
    //
    //   s_axi_rready -> hit_rsp_ready -> hit_rsp_free_c -> hit_rsp_block_c
    //                -> s_lookup_hit_go -> p2_done_c -> do_accept_c
    //                -> <array address> -> u_data/.../ADDR_A[n]
    //
    // -- five logic levels and 1.16 ns of it inside this module, bolted
    // onto the end of a four-level crossbar path in another clock domain.
    // No amount of work on the cache pipeline can fix that shape, because
    // the cache is not where the path starts.
    //
    // The slice cuts it at the port.  `ir_ready`/`ib_ready` are the skids'
    // own "not full" bits, which are REGISTERS, so every input to the
    // resolve cone is now a register inside u_l2c and the 52 paths cease
    // to exist as timing paths: what remains of the external ready is the
    // skid's own pop, one LUT level into a flop.
    //
    // COST: one cycle of RESPONSE latency on s_axi_r and s_axi_b.  NOT a
    // throughput cost -- the skid is two deep with a registered output, so
    // it accepts a beat every cycle for as long as the consumer accepts
    // one every cycle (the only bubble is the single cycle after the skid
    // has actually filled, i.e. after the consumer has already stalled).
    //
    // NO HANDSHAKE CONTRACT CHANGES.  The skid is a plain AXI register
    // slice: payload is stable from the cycle VALID rises until READY
    // takes it, VALID never withdraws, and beats leave in the order the
    // arbiter above granted them.  Response ORDERING is untouched for the
    // same reason -- the skid is strictly downstream of r_pick_c/r_lock,
    // it is a single FIFO per channel, and it cannot reorder a stream it
    // only delays.  (This is why the fix is a slice at the port and NOT a
    // deeper hit_rsp_* skid: a second hit-response slot would let a beat
    // be accepted, allocated into the MSHR and answered from the fill path
    // while an older same-ID hit response was still queued behind the
    // head, and r_lock -- which is what keeps same-ID responses ordered
    // today -- only ever locks the head.)
    wire ir_valid, ir_ready, ir_last;
    wire [ID_WIDTH-1:0] ir_id; wire [DATA_WIDTH-1:0] ir_data; wire [1:0] ir_resp;
    wire ib_valid, ib_ready;
    wire [ID_WIDTH-1:0] ib_id; wire [1:0] ib_resp;
    assign ir_valid = r_grant_byp || r_grant_mshr || r_grant_hit;
    assign ir_id    = r_grant_byp ? byp_rsp_id : (r_grant_mshr ? mshr_rsp_id : hit_rsp_id);
    assign ir_data  = r_grant_byp ? byp_rsp_rdata : (r_grant_mshr ? mshr_rsp_rdata : hit_rsp_rdata);
    assign ir_resp  = r_grant_byp ? byp_rsp_resp : (r_grant_mshr ? mshr_rsp_resp : hit_rsp_resp);
    // Round-4 fix: was hardcoded 1'b1 for bypass -- forced RLAST on every
    // beat of a multi-beat bypass read, closing the requester's burst
    // tracking after beat 0 (byp_rsp_last echoes the real per-beat flag).
    assign ir_last  = r_grant_byp ? byp_rsp_last : (r_grant_mshr ? mshr_rsp_last : hit_rsp_last);
    assign ib_valid = b_grant_byp || b_grant_mshr || b_grant_hit;
    assign ib_id    = b_grant_byp ? byp_rsp_id : (b_grant_mshr ? mshr_rsp_id : hit_rsp_id);
    assign ib_resp  = b_grant_byp ? byp_rsp_resp : (b_grant_mshr ? mshr_rsp_resp : hit_rsp_resp);
    l2c_rsp_skid #(.W(ID_WIDTH + DATA_WIDTH + 2 + 1)) u_r_skid (
        .clk(clk), .rst(rst),
        .s_valid(ir_valid), .s_ready(ir_ready), .s_data({ir_id, ir_data, ir_resp, ir_last}),
        .m_valid(s_axi_rvalid), .m_ready(s_axi_rready),
        .m_data({s_axi_rid, s_axi_rdata, s_axi_rresp, s_axi_rlast})
    );
    l2c_rsp_skid #(.W(ID_WIDTH + 2)) u_b_skid (
        .clk(clk), .rst(rst),
        .s_valid(ib_valid), .s_ready(ib_ready), .s_data({ib_id, ib_resp}),
        .m_valid(s_axi_bvalid), .m_ready(s_axi_bready),
        .m_data({s_axi_bid, s_axi_bresp})
    );
    // Level A (task #269): a fetch-tagged hit_rsp/mshr_rsp never sets
    // r_grant_mshr/r_grant_hit (r_pick_c excludes it above), so its
    // *_rsp_ready must come from the fetch reassembly path instead --
    // otherwise it would never drain (r_grant_* stuck 0 forever) and the
    // shared skid register/MSHR entry would wedge the WHOLE front door,
    // LSU traffic included.  (Declared here, driven by the fetch
    // reassembly block below -- default_nettype none needs the explicit
    // wire before use.)
    wire mshr_rsp_ready_fetch_c, hit_rsp_ready_fetch_c;
    assign mshr_rsp_ready = mshr_rsp_is_fetch
                           ? mshr_rsp_ready_fetch_c
                           : ((r_grant_mshr && ir_ready) || (b_grant_mshr && ib_ready));
    assign hit_rsp_ready  = hit_rsp_is_fetch
                           ? hit_rsp_ready_fetch_c
                           : ((r_grant_hit  && ir_ready) || (b_grant_hit  && ib_ready));
    assign byp_rsp_ready  = (r_grant_byp  && ir_ready) || (b_grant_byp  && ib_ready);

    // -- Fetch response path: pick HIT vs MSHR (task #269) ------------------
    // No RSP_BYP leg here: l2c_bypass.v carries no source tag (explicitly
    // out of scope for Level A -- see the report), so a fetch address
    // that happens to land in a declared bypass window is a known,
    // documented residual gap (its response would misroute onto s_axi's
    // B/R above, tagged with byp_rsp_id, which is never fetch-qualified).
    // Not expected in practice: instruction fetch only ever targets
    // cacheable code, never a bypass (non-cacheable MMIO/DAFB/ramdisk)
    // window, and the do_accept_c/byp_match_hit tag-array-corruption
    // assertion in l2c_ctrl.v (which DOES stay fully protective,
    // unmodified) guarantees such an address is at least kept out of the
    // tag/data arrays either way.
    //
    // Priority MSHR > HIT mirrors the existing bypass>mshr>hit rationale
    // (the slower path should not be starved by faster path); the two
    // are mutually exclusive in steady state given the single serialized
    // front door, this is defensive ordering for a same-cycle coincidence.
    wire fetch_pick_mshr = mshr_rsp_valid && mshr_rsp_is_fetch;
    wire fetch_pick_hit  = !fetch_pick_mshr && hit_rsp_valid && hit_rsp_is_fetch;
    wire fetch_q_avail   = fetch_pick_mshr || fetch_pick_hit;
    wire [127:0]         fetch_q_rdata = fetch_pick_mshr ? mshr_rsp_rdata : hit_rsp_rdata;
    wire [1:0]           fetch_q_resp  = fetch_pick_mshr ? mshr_rsp_resp  : hit_rsp_resp;
    wire [ID_WIDTH-1:0]  fetch_q_id    = fetch_pick_mshr ? mshr_rsp_id    : hit_rsp_id;
    wire                 fetch_q_last  = fetch_pick_mshr ? mshr_rsp_last  : hit_rsp_last;

    // 2-quadrant reassembly, indexed by the requester's OWN native
    // F_ID_WIDTH-bit ID (not the zero-extended ID_WIDTH one) -- axi_i can
    // have several distinct IDs' fetch bursts captured-but-not-yet-fully-
    // resolved concurrently (fetch_ar_have in l2c_ctrl.v clears on
    // DISPATCH of a burst's last quadrant, not on its RESOLUTION, and a
    // different-ID burst can then be captured while an earlier one's
    // final quadrant is still draining), so this cannot assume only one
    // pair is ever in flight -- a small per-ID table is used instead of
    // a single skid.  WITHIN one ID, the two quadrants of a beat can
    // never race each other: they share one AXI ID, so l2c_ctrl.v's
    // same-ID response ordering serializes them and the first quadrant's
    // response is always fully drained before the second is dispatched.
    //
    // ⚠ CORRECTED 2026-09-18 (race audit): this used to credit
    // "Critical-3 id_busy_c".  `id_busy_c` NO LONGER GATES CACHE-PATH
    // ACCEPTS AT ALL -- `do_accept_c` (l2c_ctrl.v) does not include it;
    // it survives only on the bypass-dispatch path.  The property still
    // holds, but it is `ord_now_block_c` / `ord_merge_block_c` at S2 that
    // now hold it up, using l2c_mshr's per-entry id-match vector.  Same
    // disease as the defects this campaign found: a recorded reason that
    // is no longer the real reason.
    localparam FQ_DEPTH = (1 << F_ID_WIDTH);
    reg                 fq_have  [0:FQ_DEPTH-1];
    reg [127:0]         fq_data0 [0:FQ_DEPTH-1];
    reg [1:0]           fq_resp0 [0:FQ_DEPTH-1];
    wire [F_ID_WIDTH-1:0] fq_idx = fetch_q_id[F_ID_WIDTH-1:0];
    wire                   fq_pair_have = fq_have[fq_idx];

    // Output side: a locked 1-deep skid, same AXI-payload-stability
    // pattern as r_lock/b_lock above -- once f_axi_rvalid is asserted its
    // payload must not change until f_axi_rready accepts it.
    reg                    f_rvalid_q;
    reg [F_ID_WIDTH-1:0]   f_rid_q;
    reg [255:0]            f_rdata_q;
    reg [1:0]              f_rresp_q;
    reg                    f_rlast_q;
    // 2026-09-15 (FMax).  This USED to be `!f_rvalid_q || f_axi_rready`.
    // The `|| f_axi_rready` term put the FETCH master's external ready pin
    // into l2c_ctrl.v's resolve cone by exactly the route the s_axi one
    // took (fetch_out_free_c -> fetch_consume_c -> hit_rsp_ready_fetch_c
    // -> hit_rsp_free_c -> hit_rsp_block_c -> p2_done_c -> do_accept_c ->
    // the array address pins), so the cache's closure depended on where
    // the CPU's instruction-fetch return logic happened to be placed.
    // Dropping the term leaves every input to the cone a register.
    //
    // It costs NOTHING in the steady state, and that is a property of the
    // 2-quadrant reassembly rather than luck: a 256 b fetch beat is built
    // from TWO quadrant responses, so a burst cannot complete faster than
    // one per two cycles, while the output register drains in one.  The
    // second-of-pair therefore finds `f_rvalid_q` already clear on the
    // cycle it wants it.  Measured: fetch door 1.992 / 3.977 / 1.992
    // cyc/burst, unchanged (the 2- and 4-quadrant structural floors).
    // What it does cost is one extra cycle per event in which the fetch
    // master actually stalls -- a back-pressure case, not a rate case.
    //
    // The s_axi side needed a register slice instead of the same one-term
    // deletion because its R channel has no such natural spacing: the LSU
    // read-hit rate IS one beat per cycle, so `!r_valid` alone would have
    // halved it.  See the SLAVE RESPONSE REGISTER SLICE block above.
    wire fetch_out_free_c = !f_rvalid_q;
    // First-of-pair always drains into the table (1 free slot per ID,
    // freed the instant its partner arrives); second-of-pair only drains
    // once the output register has room -- this is what backpressures
    // f_axi_rready all the way back through hit_rsp_block_c/mshr's own
    // rsp_ready contract into l2c_ctrl.v's S_LOOKUP retry, exactly like
    // LSU's existing r_grant_hit && s_axi_rready gating does today.
    wire fetch_consume_c = fetch_q_avail && (fq_pair_have ? fetch_out_free_c : 1'b1);
    assign mshr_rsp_ready_fetch_c = fetch_pick_mshr && fetch_consume_c;
    assign hit_rsp_ready_fetch_c  = fetch_pick_hit  && fetch_consume_c;

    integer fq_i;
    always @(posedge clk) begin
        if (rst) begin
            f_rvalid_q <= 1'b0;
            for (fq_i = 0; fq_i < FQ_DEPTH; fq_i = fq_i + 1) fq_have[fq_i] <= 1'b0;
        end else begin
            if (f_rvalid_q && f_axi_rready) f_rvalid_q <= 1'b0;
            if (fetch_consume_c) begin
                if (!fq_pair_have) begin
                    fq_have[fq_idx]  <= 1'b1;
                    fq_data0[fq_idx] <= fetch_q_rdata;
                    fq_resp0[fq_idx] <= fetch_q_resp;
                end else begin
                    fq_have[fq_idx] <= 1'b0;
                    f_rvalid_q <= 1'b1;
                    f_rid_q    <= fq_idx;
                    // Quadrant 0 (lower address, dispatched first) is the
                    // low half; quadrant 1 (higher address, dispatched
                    // second) is the high half -- matches l2c_ctrl.v's
                    // own req_qoff*128 slice convention (low addr = low
                    // bits).
                    f_rdata_q  <= {fetch_q_rdata, fq_data0[fq_idx]};
                    f_rresp_q  <= fq_resp0[fq_idx] | fetch_q_resp;
                    f_rlast_q  <= fetch_q_last;
                end
            end
        end
    end
    assign f_axi_rvalid = f_rvalid_q;
    assign f_axi_rid    = f_rid_q;
    assign f_axi_rdata  = f_rdata_q;
    assign f_axi_rresp  = f_rresp_q;
    assign f_axi_rlast  = f_rlast_q;

    // 2026-08-30 boot investigation round 8 -- see the port declaration's
    // comment above for why this exists. Directly shows, for the fetch
    // (axi_i) path specifically: whether a request is even presented at
    // the front door and accepted (bits 0/1), which of the two response
    // sources (hit vs. MSHR-fill) is claiming a fetch-tagged completion
    // and whether that source is VALID at all (bits 2-5), whether the
    // 256b quadrant-pair reassembly has a completion ready to drain
    // (bits 6/7), whether the output skid register is occupied and
    // whether the CPU side is accepting it (bits 8/9), and whether a
    // quadrant-pair is straddling (first-of-pair captured, second not
    // yet arrived) (bit 10).
    assign dbg_fetch_snap = {
        5'd0,                // [15:11] reserved
        fq_pair_have,        // [10] a quadrant-pair is straddling for fq_idx
        f_axi_rready,        // [9]  CPU-side ready for the reassembled beat
        f_rvalid_q,          // [8]  reassembled beat is presented, awaiting drain
        fetch_consume_c,     // [7]  a fetch completion is being consumed this cycle
        fetch_q_avail,       // [6]  a fetch-tagged hit/MSHR completion is available
        mshr_rsp_is_fetch,   // [5]  the live MSHR response is fetch-sourced
        mshr_rsp_valid,      // [4]  MSHR response channel valid
        hit_rsp_is_fetch,    // [3]  the live hit response is fetch-sourced
        hit_rsp_valid,       // [2]  hit response channel valid
        f_axi_arready,       // [1]  front door accepted the fetch AR
        f_axi_arvalid        // [0]  fetch AR presented at the front door
    };

    // 2026-08-31 boot investigation round 10 -- see the port declaration's
    // comment above for why this exists. `f_rid_q` is the AXI ID of
    // whatever response is CURRENTLY parked in the one-deep output skid
    // (only meaningful while f_rvalid_q, bit [15], reads 1); `fq_idx` is
    // the ID a fresh fetch-tagged hit/MSHR completion is targeting THIS
    // cycle (only meaningful while fetch_pick_hit, bit [14], reads 1);
    // `hit_rsp_id` is the raw ID_WIDTH-bit tag L2C's own hit pipeline is
    // presenting this cycle, truncated to its low 6 bits (same value
    // fq_idx is derived from via fetch_q_id's low F_ID_WIDTH bits when
    // fetch_pick_hit is live).
    assign dbg_fetch_id_snap = {
        f_rvalid_q,                    // [15] parked skid register occupied
        fetch_pick_hit,                // [14] a fresh fetch-tagged hit is live this cycle
        hit_rsp_id[5:0],               // [13:8] raw hit_rsp_id (ID_WIDTH=6)
        fq_idx,                        // [7:4]  ID a fresh completion targets this cycle
        f_rid_q                        // [3:0]  ID currently parked in the output skid
    };
end
endgenerate
endmodule

// ============================================================================
// l2c_rsp_skid -- AXI response register slice (2 deep, registered output).
//
// Added 2026-09-15 to take the crossbar's `rready`/`bready` out of l2c_ctrl's
// resolve cone; see the SLAVE RESPONSE REGISTER SLICE block in l2c.v for the
// measured paths that motivated it.
//
// The contract, in full:
//
//   * `s_ready` is `!v1` -- a REGISTER, with no combinational dependence on
//     `m_ready` whatsoever.  That is the entire point: it is what makes the
//     upstream's accept decision independent of the downstream's.
//   * `m_valid`/`m_data` come straight out of registers, so the downstream
//     sees no combinational path from this module's inputs either.
//   * ORDER IS PRESERVED.  v1 can only be occupied while v0 is, and v1 always
//     drains into v0, so this is a 2-entry FIFO, not a bypass.
//   * PAYLOAD STABILITY.  d0 is only ever rewritten on a cycle in which it was
//     either empty or accepted (`m_valid && m_ready`), which is exactly AXI's
//     rule that a payload may not change under a VALID that has not been
//     taken.
//   * THROUGHPUT.  With `m_ready` held high the slice accepts a beat every
//     cycle indefinitely: `v1` is only ever set on a cycle the consumer
//     refused, so in an unstalled stream the second slot is never used.  The
//     one cycle of upstream refusal after the slice has genuinely filled is
//     the standard and unavoidable cost of a registered `s_ready`.
//
// Verilog-2005, no reset on the payload registers (they are only read behind
// their own valid bit).
// ============================================================================
`default_nettype none
module l2c_rsp_skid #(
    parameter W = 1
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         s_valid,
    output wire         s_ready,
    input  wire [W-1:0] s_data,
    output wire         m_valid,
    input  wire         m_ready,
    output wire [W-1:0] m_data
);
    reg         v0, v1;
    reg [W-1:0] d0, d1;

    assign m_valid = v0;
    assign m_data  = d0;
    assign s_ready = !v1;

    wire pop_c  = v0 && m_ready;
    wire push_c = s_valid && !v1;

    always @(posedge clk) begin
        if (rst) begin
            v0 <= 1'b0; v1 <= 1'b0;
        end else begin
            // Head.  Refilled from the tail first (FIFO order), then from a
            // fresh beat, then emptied.
            if (pop_c) begin
                if (v1)           begin v0 <= 1'b1; d0 <= d1; end
                else if (push_c)  begin v0 <= 1'b1; d0 <= s_data; end
                else                    v0 <= 1'b0;
            end else if (push_c && !v0) begin
                v0 <= 1'b1; d0 <= s_data;
            end
            // Tail.  Only ever holds a beat the consumer refused.
            if (pop_c && v1) begin
                if (push_c) begin v1 <= 1'b1; d1 <= s_data; end
                else              v1 <= 1'b0;
            end else if (push_c && v0 && !pop_c) begin
                v1 <= 1'b1; d1 <= s_data;
            end
        end
    end

    // INVARIANT: the tail is never occupied without the head.  Everything
    // this module promises rests on it -- `s_ready = !v1` is only a correct
    // "not full" if v1 implies v0, and order is only preserved if the tail
    // can never be the sole occupant and be read out of turn.
    // synthesis translate_off
    always @(posedge clk) begin
        if (!rst && v1 && !v0) begin
            $display("L2C_RSP_SKID ASSERT: tail occupied with an empty head -- s_ready (!v1) no longer means 'not full'");
            $fatal(1);
        end
    end
    // synthesis translate_on
endmodule
`default_nettype wire

