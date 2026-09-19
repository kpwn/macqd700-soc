// if_to_axi.v — Instruction-fetch (req/valid, 128-bit line) to AXI4 master.
//
// The CPU core exposes an extremely thin I-fetch interface:
//
//   if_addr  [31:0]   — byte address of the requested 16-byte line
//   if_req            — req valid (combinational; deassertion is fine)
//   if_rdata [127:0]  — 128-bit line response
//   if_rvalid         — single-cycle response strobe
//   if_fault          — valid with if_rvalid when AXI RRESP != OKAY
//
// On the xbar side we need a full AXI4 read master that does a single-beat,
// 16-byte-wide burst (arlen=0, arsize=4 → 16 bytes).  This adapter turns
// each `if_req` rising-edge (while idle) into an AR transaction and
// delivers the resulting R beat back as the `if_rvalid` pulse.
//
// One outstanding transaction at a time.  The core's if_stage already
// self-serialises its fetches (next if_req is gated on the previous
// response), so there is no queue here.
//
// Latency: IF_ADDR→AXI AR is one cycle (registered).  AXI R → IF_RDATA
// is one cycle (registered).  Internal bandwidth matches the xbar port.

`default_nettype none

module if_to_axi #(
    parameter ID_WIDTH   = 4,
    parameter DATA_WIDTH = 128,
    parameter [ID_WIDTH-1:0] ID_TAG = {ID_WIDTH{1'b0}}
) (
    input  wire                   clk,
    input  wire                   rst,

    // ── CPU-side fetch interface ─────────────────────────────────────
    input  wire [31:0]            if_addr,
    input  wire                   if_req,
    output wire [DATA_WIDTH-1:0]  if_rdata,
    output wire                   if_rvalid,
    output wire                   if_fault,

    // ── AXI4 master (read-only) ───────────────────────────────────────
    output wire [ID_WIDTH-1:0]    m_arid,
    output wire [31:0]            m_araddr,
    output wire [7:0]             m_arlen,
    output wire [2:0]             m_arsize,
    output wire [1:0]             m_arburst,
    output wire                   m_arvalid,
    input  wire                   m_arready,

    input  wire [ID_WIDTH-1:0]    m_rid,
    input  wire [DATA_WIDTH-1:0]  m_rdata,
    input  wire [1:0]             m_rresp,
    input  wire                   m_rlast,
    input  wire                   m_rvalid,
    output wire                   m_rready
);

    // ── State: one outstanding read at a time ─────────────────────────
    localparam S_IDLE = 2'd0;
    localparam S_AR   = 2'd1;   // AR launched, waiting for R
    reg [1:0] state;
    reg [31:0] addr_q;

    always @(posedge clk) begin
        if (rst) begin
            state  <= S_IDLE;
            addr_q <= 32'h0;
        end else begin
            case (state)
            S_IDLE: begin
                if (if_req) begin
                    // Align to 16-byte boundary (I-cache lines).
                    addr_q <= {if_addr[31:4], 4'b0000};
                    state  <= S_AR;
                end
            end
            S_AR: begin
                if (m_rvalid && m_rready) begin
                    state <= S_IDLE;
                end
            end
            default: state <= S_IDLE;
            endcase
        end
    end

    // ── AR channel ────────────────────────────────────────────────────
    // Hold ARVALID high while in S_AR until accepted.  Simple
    // one-shot: ARREADY terminates the handshake.
    reg ar_launched;
    always @(posedge clk) begin
        if (rst || state == S_IDLE) begin
            ar_launched <= 1'b0;
        end else if (m_arvalid && m_arready) begin
            ar_launched <= 1'b1;
        end
    end

    assign m_arid    = ID_TAG;
    assign m_araddr  = addr_q;
    assign m_arlen   = 8'd0;           // 1 beat
    assign m_arsize  = 3'd4;           // 2^4 = 16 bytes (= DATA_WIDTH)
    assign m_arburst = 2'b01;          // INCR
    assign m_arvalid = (state == S_AR) && !ar_launched;

    // ── R channel ─────────────────────────────────────────────────────
    // Always ready (one outstanding → we can drain on the next cycle).
    assign m_rready  = (state == S_AR);

    // Forward response to the core.  Fault is metadata carried with the
    // line; the core realizes it precisely only if the fetched instruction
    // reaches the ROB head.
    //
    // 128-bit line word-reversal — BE/LE reconciliation for HW fetch path.
    //
    // The CPU is BE: narrow-side byte at file offset 0 of a 32-bit narrow
    // word rides wdata[31:24] (MSB lane).  `axi_narrow_to_wide` places
    // each narrow word at an addr[3:2]-selected 32-bit lane of the 128-bit
    // beat (A=0→[31:0], A=4→[63:32], A=8→[95:64], A=12→[127:96]).  DDR
    // (`ddr_ctrl`) stores bit-for-bit in LE-lane convention:
    // `mem_bN = wdata[N*8 +: 8]`.  Net for a fetched line: byte at file
    // offset K lands at `mem_b[(K/4)*4 + (3 - K%4)]`, i.e. each 32-bit
    // word preserves its internal BE byte order, but the four 4-byte
    // words are in ascending-address order across
    // `m_rdata[31:0]..[127:96]`.
    //
    // `if_stage`/`predecode` expect `if_rdata` in the CPU's BE convention
    // — byte at file offset 0 at `if_rdata[127:120]`, byte at offset 15 at
    // `if_rdata[7:0]`.  Algebraically this is a 4-word reversal of
    // `m_rdata` (inter-word swap; internal byte order inside each 32-bit
    // word is already correct).  NOT a full 16-byte byte-reverse, NOT a
    // within-word swap.
    //
    // Background: without this swap HW fetched the 16-byte line with the
    // four 4-byte words in reverse order; every opword the CPU saw was
    // garbage, so `dbg_committed` counted a random walk through
    // non-trapping opwords while PC wedged in the low-vector region.  See
    // memory entry `project_hw_firstlight_investigation_20260423.md`.
    //
    // Sim is unaffected: `mac_top.v` (sim top) bypasses `if_to_axi.v` and
    // drives `if_rdata` directly in BE-lane via `tb_top.cpp`.  This
    // module is HW-only (used only from `fpga_top.v`).
    assign if_rdata  = { m_rdata[ 31:  0],
                         m_rdata[ 63: 32],
                         m_rdata[ 95: 64],
                         m_rdata[127: 96] };
    assign if_rvalid = m_rvalid && m_rready;
    assign if_fault  = if_rvalid && (m_rresp != 2'b00);

endmodule

`default_nettype wire
