// ddr_ctrl.v — DDR4 MIG wrapper + behavioural sim model
//
// Purpose
//   Presents a single AXI4 slave port matching the shape of `axi_xbar.v`'s
//   S0 output (128-bit data, 6-bit XID, 32-bit address, full AW/W/B/AR/R).
//   In SIM_MODEL mode the slave is backed by a small BRAM-style array so
//   the Verilator tb has a real DDR substitute.  In real-hardware
//   mode (ifndef SIM_MODEL) the module bridges the repo's 128-bit DDR AXI
//   contract into the pcie_test MIG's 256-bit UI AXI contract.  The physical
//   MIG instance lives at fpga_top so Vivado sees its DDR pins as top-level
//   memory-interface ports.
//
// Key interfaces
//   AXI4 slave (from xbar perspective these are outputs of the xbar):
//     awid[5:0], awaddr[31:0], awlen[7:0], awsize[2:0], awburst[1:0],
//     awvalid/awready
//     wdata[127:0], wstrb[15:0], wlast, wvalid/wready
//     bid[5:0], bresp[1:0], bvalid/bready
//     arid[5:0], araddr[31:0], arlen[7:0], arsize[2:0], arburst[1:0],
//     arvalid/arready
//     rid[5:0], rdata[127:0], rresp[1:0], rlast, rvalid/rready
//
//   Status / clocking:
//     ddr_cal_done — MIG init_calib_complete.  In SIM_MODEL rises N
//       cycles after reset release (SIM_CAL_CYCLES, default 16).
//     ddr_clk      — MIG user-clock output.  In SIM_MODEL this is clk
//       (pass-through); on real hw this would be c0_ddr4_ui_clk.
//
//   Debug counters:
//     dbg_aw_cnt / dbg_ar_cnt / dbg_b_cnt / dbg_r_cnt — 32-bit perf
//     counters incremented on AXI handshakes (for IPC / BW stats).
//
// SIM_MODEL storage
//   16-bit addressed into a `BRAM_BEATS`-deep array of 128-bit words.
//   Address is taken from the low bits of the DDR-flattened 32-bit
//   address after shifting off 4 LSBs (one 128-bit beat = 16 bytes).
//   AXI sizes up to the native 16-byte beat are accepted when aligned to
//   their requested size; byte strobes select the active lanes.
//
// MIG IP (real hw)
//   Configuration expected by this wrapper:
//     - DDR4-2400, 1200 MHz memory clock, 1200 MHz DRAM clock (half-bus)
//     - Matches the known-good ~/FPGA/pcie_test DDR4 IP shell:
//       MT40A512M16LY-075 components, parity/alert disabled, 32-bit bus
//     - 17-bit address, 2-bit BA, 1-bit BG, 1-bit CS/ODT/CKE
//     - MIG UI clock: 333.25 MHz in the working project
//     - MIG AXI port in that project: 256-bit data, 31-bit address, 1-bit ID
//       (this repo still uses a 128-bit, 32-bit-address, 6-bit-ID xbar until
//       the real MIG bridge integrates; see axi_ddr4_mig_bridge.v for the
//       current first-light contract shim)
//   Vivado reads the matching black-box stub from rtl/vendor and stitches
//   the OOC DCP from PCIE_TEST_DIR into fpga_top during real-MIG synth/impl.
//
// Verilog-2005, synchronous active-high rst, 4-space indent, no latches.

`default_nettype none

/* verilator lint_off UNUSEDPARAM */
module ddr_ctrl #(
    parameter DATA_WIDTH      = 128,
    parameter STRB_WIDTH      = DATA_WIDTH/8,
    parameter XID_WIDTH       = 6,
    parameter ADDR_WIDTH      = 32,
    // SIM_MODEL storage — default tiny (64 KB) so it fits on a KU5P
    // alongside the CPU core for the first hardware impl round.  The
    // behavioural AXI slave was originally 8 MB (BRAM_LOG2_BEATS=19) which
    // infers 524K RAM256X1D LUT-RAMs — infeasible on a 216K-LUT part.
    // 2^12 beats × 16B = 64 KB maps to ~2K LUT-RAMs or ~16 BRAM36 — fits
    // comfortably with the rest of the design.  Tests that need a bigger
    // sim window override BRAM_LOG2_BEATS at instantiation (see
    // tb/tb_ddr_model.v overrides to 11 for unit-tb).  The real MIG IP
    // path (ifndef SIM_MODEL, pending follow-up) is the production path.
    parameter BRAM_LOG2_BEATS = 12,     // 2^12 * 16B = 64 KB
    parameter SIM_CAL_CYCLES  = 16,     // cycles after reset release → ddr_cal_done
    // 2026-05-22 — SIM_DDR_READ_DELAY: inject artificial read latency
    // before rvalid asserts.  Default 0 = current zero-latency behaviour.
    // Used to mimic real-MIG variable read latency (~30-80 cycles) so
    // sim can exercise pipeline races invisible to zero-latency reads.
    // Only affects the SIM_MODEL read path.
    parameter SIM_DDR_READ_DELAY = 0
) (
    // Clock / reset
    input  wire                       clk,
    input  wire                       rst,

    // MIG "user" clock exposed to the rest of the SoC.  In SIM_MODEL this
    // is just `clk` routed through; in real hw it is c0_ddr4_ui_clk.
    output wire                       ddr_clk,
    // MIG init_calib_complete — goes high once DDR init + training done.
    output wire                       ddr_cal_done,

    // ── AXI4 slave — shape matches axi_xbar.v S0 ─────────────────────────
    // AW
    input  wire [XID_WIDTH-1:0]       awid,
    input  wire [ADDR_WIDTH-1:0]      awaddr,
    input  wire [7:0]                 awlen,
    input  wire [2:0]                 awsize,
    input  wire [1:0]                 awburst,
    input  wire                       awvalid,
    output wire                       awready,
    // W
    input  wire [DATA_WIDTH-1:0]      wdata,
    input  wire [STRB_WIDTH-1:0]      wstrb,
    input  wire                       wlast,
    input  wire                       wvalid,
    output wire                       wready,
    // B
    output wire [XID_WIDTH-1:0]       bid,
    output wire [1:0]                 bresp,
    output wire                       bvalid,
    input  wire                       bready,
    // AR
    input  wire [XID_WIDTH-1:0]       arid,
    input  wire [ADDR_WIDTH-1:0]      araddr,
    input  wire [7:0]                 arlen,
    input  wire [2:0]                 arsize,
    input  wire [1:0]                 arburst,
    input  wire                       arvalid,
    output wire                       arready,
    // R
    output wire [XID_WIDTH-1:0]       rid,
    output wire [DATA_WIDTH-1:0]      rdata,
    output wire [1:0]                 rresp,
    output wire                       rlast,
    output wire                       rvalid,
    input  wire                       rready,

    // ── Debug / perf counters ────────────────────────────────────────────
    output wire [31:0]                dbg_aw_cnt,
    output wire [31:0]                dbg_w_cnt,
    output wire [31:0]                dbg_b_cnt,
    output wire [31:0]                dbg_ar_cnt,
    output wire [31:0]                dbg_r_cnt

`ifndef SIM_MODEL
    ,
    // ── Real-hw only: MIG UI clock/status and 256-bit AXI pins ───────────
    // fpga_top owns the physical DDR4 MIG instance; this module owns only the
    // core-clock to MIG-UI-clock bridge and 128-bit to 256-bit contract shim.
    input  wire                       mig_ui_clk,
    input  wire                       mig_ui_rst,
    input  wire                       mig_cal_done,

    output wire [0:0]                 mig_awid,
    output wire [30:0]                mig_awaddr,
    output wire [7:0]                 mig_awlen,
    output wire [2:0]                 mig_awsize,
    output wire [1:0]                 mig_awburst,
    output wire                       mig_awvalid,
    input  wire                       mig_awready,

    output wire [255:0]               mig_wdata,
    output wire [31:0]                mig_wstrb,
    output wire                       mig_wlast,
    output wire                       mig_wvalid,
    input  wire                       mig_wready,

    input  wire [0:0]                 mig_bid,
    input  wire [1:0]                 mig_bresp,
    input  wire                       mig_bvalid,
    output wire                       mig_bready,

    output wire [0:0]                 mig_arid,
    output wire [30:0]                mig_araddr,
    output wire [7:0]                 mig_arlen,
    output wire [2:0]                 mig_arsize,
    output wire [1:0]                 mig_arburst,
    output wire                       mig_arvalid,
    input  wire                       mig_arready,

    output wire                       mig_rready,
    input  wire [0:0]                 mig_rid,
    input  wire [255:0]               mig_rdata,
    input  wire [1:0]                 mig_rresp,
    input  wire                       mig_rlast,
    input  wire                       mig_rvalid
`endif
);

// ══════════════════════════════════════════════════════════════════════════
// Performance counters (shared across both modes)
// ══════════════════════════════════════════════════════════════════════════
reg [31:0] aw_cnt, w_cnt, b_cnt, ar_cnt, r_cnt;
localparam [1:0] RESP_OKAY   = 2'b00;
localparam [1:0] RESP_SLVERR = 2'b10;

assign dbg_aw_cnt = aw_cnt;
assign dbg_w_cnt  = w_cnt;
assign dbg_b_cnt  = b_cnt;
assign dbg_ar_cnt = ar_cnt;
assign dbg_r_cnt  = r_cnt;

wire aw_fire = awvalid & awready;
wire w_fire  = wvalid  & wready;
wire b_fire  = bvalid  & bready;
wire ar_fire = arvalid & arready;
wire r_fire  = rvalid  & rready;

always @(posedge clk) begin
    if (rst) begin
        aw_cnt <= 32'h0;
        w_cnt  <= 32'h0;
        b_cnt  <= 32'h0;
        ar_cnt <= 32'h0;
        r_cnt  <= 32'h0;
    end else begin
        if (aw_fire) aw_cnt <= aw_cnt + 32'd1;
        if (w_fire)  w_cnt  <= w_cnt  + 32'd1;
        if (b_fire)  b_cnt  <= b_cnt  + 32'd1;
        if (ar_fire) ar_cnt <= ar_cnt + 32'd1;
        if (r_fire)  r_cnt  <= r_cnt  + 32'd1;
    end
end

`ifdef SIM_MODEL
`ifdef SIM_MIG_BRIDGE
// ══════════════════════════════════════════════════════════════════════════
// SIM_MIG_BRIDGE — Sim path with the production axi_ddr4_mig_bridge in
// front of a behavioural 256-bit MIG backend (sim_mig_backend.v).
//
// This is the SAME stack the bitstream uses (xbar 128 → bridge → MIG 256),
// minus only the real DDR4 IP itself.  Closes the coverage gap where the
// regular SIM_MODEL elides the bridge entirely.  Skips axi_async_bridge
// (CDC) since sim is single-clock; the bridge's combinational packing /
// wstrb routing is identical to the HW path.
// ══════════════════════════════════════════════════════════════════════════

assign ddr_clk = clk;

wire        mig_sim_cal_done;
wire [0:0]  mig_sim_awid;
wire [30:0] mig_sim_awaddr;
wire [7:0]  mig_sim_awlen;
wire [2:0]  mig_sim_awsize;
wire [1:0]  mig_sim_awburst;
wire        mig_sim_awvalid;
wire        mig_sim_awready;

wire [255:0] mig_sim_wdata;
wire [31:0]  mig_sim_wstrb;
wire         mig_sim_wlast;
wire         mig_sim_wvalid;
wire         mig_sim_wready;

wire [0:0]   mig_sim_bid;
wire [1:0]   mig_sim_bresp;
wire         mig_sim_bvalid;
wire         mig_sim_bready;

wire [0:0]   mig_sim_arid;
wire [30:0]  mig_sim_araddr;
wire [7:0]   mig_sim_arlen;
wire [2:0]   mig_sim_arsize;
wire [1:0]   mig_sim_arburst;
wire         mig_sim_arvalid;
wire         mig_sim_arready;

wire [0:0]   mig_sim_rid;
wire [255:0] mig_sim_rdata;
wire [1:0]   mig_sim_rresp;
wire         mig_sim_rlast;
wire         mig_sim_rvalid;
wire         mig_sim_rready;

assign ddr_cal_done = mig_sim_cal_done;

axi_ddr4_mig_bridge u_bridge_sim (
    .clk(clk),
    .rst(rst || !mig_sim_cal_done),
    .s_awid(awid), .s_awaddr(awaddr), .s_awlen(awlen),
    .s_awsize(awsize), .s_awburst(awburst),
    .s_awvalid(awvalid), .s_awready(awready),
    .s_wdata(wdata), .s_wstrb(wstrb), .s_wlast(wlast),
    .s_wvalid(wvalid), .s_wready(wready),
    .s_bid(bid), .s_bresp(bresp), .s_bvalid(bvalid),
    .s_bready(bready),
    .s_arid(arid), .s_araddr(araddr), .s_arlen(arlen),
    .s_arsize(arsize), .s_arburst(arburst),
    .s_arvalid(arvalid), .s_arready(arready),
    .s_rid(rid), .s_rdata(rdata), .s_rresp(rresp),
    .s_rlast(rlast), .s_rvalid(rvalid), .s_rready(rready),

    .m_awid(mig_sim_awid), .m_awaddr(mig_sim_awaddr),
    .m_awlen(mig_sim_awlen), .m_awsize(mig_sim_awsize),
    .m_awburst(mig_sim_awburst),
    .m_awvalid(mig_sim_awvalid), .m_awready(mig_sim_awready),
    .m_wdata(mig_sim_wdata), .m_wstrb(mig_sim_wstrb),
    .m_wlast(mig_sim_wlast),
    .m_wvalid(mig_sim_wvalid), .m_wready(mig_sim_wready),
    .m_bid(mig_sim_bid), .m_bresp(mig_sim_bresp),
    .m_bvalid(mig_sim_bvalid), .m_bready(mig_sim_bready),
    .m_arid(mig_sim_arid), .m_araddr(mig_sim_araddr),
    .m_arlen(mig_sim_arlen), .m_arsize(mig_sim_arsize),
    .m_arburst(mig_sim_arburst),
    .m_arvalid(mig_sim_arvalid), .m_arready(mig_sim_arready),
    .m_rid(mig_sim_rid), .m_rdata(mig_sim_rdata),
    .m_rresp(mig_sim_rresp), .m_rlast(mig_sim_rlast),
    .m_rvalid(mig_sim_rvalid), .m_rready(mig_sim_rready)
);

sim_mig_backend #(
    .BEATS_LOG2(22)     // 2^22 × 32 B = 128 MiB — covers RAM (64 MiB) + ROM (4 MiB folded)
) u_mig_sim (
    .clk(clk),
    .rst(rst),
    .cal_done(mig_sim_cal_done),
    .awid(mig_sim_awid), .awaddr(mig_sim_awaddr),
    .awlen(mig_sim_awlen), .awsize(mig_sim_awsize),
    .awburst(mig_sim_awburst),
    .awvalid(mig_sim_awvalid), .awready(mig_sim_awready),
    .wdata(mig_sim_wdata), .wstrb(mig_sim_wstrb),
    .wlast(mig_sim_wlast),
    .wvalid(mig_sim_wvalid), .wready(mig_sim_wready),
    .bid(mig_sim_bid), .bresp(mig_sim_bresp),
    .bvalid(mig_sim_bvalid), .bready(mig_sim_bready),
    .arid(mig_sim_arid), .araddr(mig_sim_araddr),
    .arlen(mig_sim_arlen), .arsize(mig_sim_arsize),
    .arburst(mig_sim_arburst),
    .arvalid(mig_sim_arvalid), .arready(mig_sim_arready),
    .rid(mig_sim_rid), .rdata(mig_sim_rdata),
    .rresp(mig_sim_rresp), .rlast(mig_sim_rlast),
    .rvalid(mig_sim_rvalid), .rready(mig_sim_rready)
);

`else  // SIM_MIG_BRIDGE not defined — original SIM_MODEL behavioural BRAM
// ══════════════════════════════════════════════════════════════════════════
// SIM_MODEL — Behavioural AXI4 slave backed by a BRAM array.
//
// State machines:
//   Write channel:  AW_IDLE → AW_W (capture AW) → AW_B (issue BRESP)
//   Read channel:   AR_IDLE → AR_R (stream beats, issue RLAST on last)
//
// One outstanding write and one outstanding read; AXI allows interleaving
// the two directions freely.  Bursts honour awlen/arlen; only INCR makes
// sense here (FIXED/WRAP not exercised by the xbar).
// ══════════════════════════════════════════════════════════════════════════

// MIG UI clock = sim clk
assign ddr_clk = clk;

// Cal-done: count N cycles after reset deassertion, then latch high.
localparam CAL_CW = 6;
reg [CAL_CW-1:0] cal_ctr;
reg              cal_done_r;
assign ddr_cal_done = cal_done_r;

always @(posedge clk) begin
    if (rst) begin
        cal_ctr    <= {CAL_CW{1'b0}};
        cal_done_r <= 1'b0;
    end else if (!cal_done_r) begin
        if (cal_ctr >= SIM_CAL_CYCLES[CAL_CW-1:0]) begin
            cal_done_r <= 1'b1;
        end else begin
            cal_ctr <= cal_ctr + {{(CAL_CW-1){1'b0}}, 1'b1};
        end
    end
end

// ── Storage: per-byte split for BRAM inference with byte-enables ─────────
// Split into STRB_WIDTH byte-wide memories so Vivado can infer block RAM
// with native byte-write-enables (BRAM36 BYTE_WRITE mode).  Single 128-bit
// "reg [127:0] mem[]" with combinational merge_beat RMW does not infer
// as BRAM/URAM and falls back to distributed RAM (LUT RAM) — at 2 MB that
// was 131K LUT-RAMs, over half the KU5P LUTs.  At 64 KB (default) this
// is small either way; the byte-split still helps Vivado pick BRAM on
// more generous sizes.
//
// Sizing: 2^12 beats × 16 bytes = 64 KB default.  Byte-lane is 4096 × 8
// bits = 32 Kb; one BRAM36 per lane suffices → 16 BRAMs total.
`ifdef FPGA_ROM_SIM
// Full fpga_top ROM simulation needs the xbar-flattened ROM window
// (0x4000_0000..) to stay distinct from low RAM.  Keep this behind an
// explicit simulation define so the regular small SIM_MODEL stays tiny.
localparam SIM_RAM_BYTES  = 32'h0400_0000;
localparam SIM_ROM_BYTES  = 32'h0040_0000;
localparam SIM_RAM_BEATS  = SIM_RAM_BYTES / STRB_WIDTH;
localparam SIM_ROM_BEATS  = SIM_ROM_BYTES / STRB_WIDTH;
localparam BEATS          = SIM_RAM_BEATS + SIM_ROM_BEATS;
localparam BEAT_IDX_WIDTH = 23;
`else
localparam BEATS = (1 << BRAM_LOG2_BEATS);
localparam BEAT_IDX_WIDTH = BRAM_LOG2_BEATS;
`endif

(* ram_style = "block" *)
reg [7:0] mem_b0  [0:BEATS-1];
(* ram_style = "block" *)
reg [7:0] mem_b1  [0:BEATS-1];
(* ram_style = "block" *)
reg [7:0] mem_b2  [0:BEATS-1];
(* ram_style = "block" *)
reg [7:0] mem_b3  [0:BEATS-1];
(* ram_style = "block" *)
reg [7:0] mem_b4  [0:BEATS-1];
(* ram_style = "block" *)
reg [7:0] mem_b5  [0:BEATS-1];
(* ram_style = "block" *)
reg [7:0] mem_b6  [0:BEATS-1];
(* ram_style = "block" *)
reg [7:0] mem_b7  [0:BEATS-1];
(* ram_style = "block" *)
reg [7:0] mem_b8  [0:BEATS-1];
(* ram_style = "block" *)
reg [7:0] mem_b9  [0:BEATS-1];
(* ram_style = "block" *)
reg [7:0] mem_b10 [0:BEATS-1];
(* ram_style = "block" *)
reg [7:0] mem_b11 [0:BEATS-1];
(* ram_style = "block" *)
reg [7:0] mem_b12 [0:BEATS-1];
(* ram_style = "block" *)
reg [7:0] mem_b13 [0:BEATS-1];
(* ram_style = "block" *)
reg [7:0] mem_b14 [0:BEATS-1];
(* ram_style = "block" *)
reg [7:0] mem_b15 [0:BEATS-1];

// Address → beat-index.  We take the low (BRAM_LOG2_BEATS+4) bits of the
// byte address and drop the bottom 4 (128-bit alignment).  Wraps above
// storage — fine because simulation uses only a small window.  The
// upper/lower address bits are intentionally unused here.
//
// 2026-08-20: the FPGA_ROM_SIM ROM-window special-case below used to
// match ONLY the primary [0x4000_0000, 0x4040_0000) 4 MiB reservation.
// boot_fsm.v now also writes a real ROM-mirror alias at MIRROR_WRAP_ADDR
// (0x4080_0000, see boot_fsm.v's own parameter comment) whenever
// MIRROR_LOW_RAM=1 -- the shipping CPU=m68k040 default. Address
// 0x4080_0000 differs from 0x4000_0000 ONLY in bit 23 (0x0080_0000),
// which sits OUTSIDE the a[22:4] slice this function uses as the ROM
// beat offset -- so an unwidened range check sent it down the `else`
// branch instead, where `a[25:4]` ALIASES it onto the SAME beats as real
// low-RAM address 0x0080_0000 (8 MiB into RAM): a real AXI write to the
// new mirror address would have silently corrupted live low RAM in any
// full fpga_top FPGA_ROM_SIM run (tb-fpga-top-rom-realboot or similar),
// even though the write is completely correct per real Q700 hardware
// semantics -- this is a simulation-behavioural-model gap, not an RTL
// bug on the write side. Fixed by masking bit 23 out of the comparison
// (not out of the a[22:4] beat-offset extraction itself, which already
// doesn't see that bit) so both the 0x4000_0000 and 0x4080_0000 windows
// fold onto the identical ROM beats -- matching the real Q700 ROM chip's
// own decode, which does not distinguish these aliases either.
/* verilator lint_off UNUSEDSIGNAL */
function [BEAT_IDX_WIDTH-1:0] addr_to_idx;
    input [ADDR_WIDTH-1:0] a;
    begin
`ifdef FPGA_ROM_SIM
        if ((a & ~32'h0080_0000) >= 32'h4000_0000 &&
            (a & ~32'h0080_0000) <  32'h4040_0000)
            addr_to_idx = SIM_RAM_BEATS[BEAT_IDX_WIDTH-1:0] +
                          a[22:4];
        else
            addr_to_idx = a[25:4];
`else
        addr_to_idx = a[BRAM_LOG2_BEATS+4-1:4];
`endif
    end
endfunction
/* verilator lint_on UNUSEDSIGNAL */

// ── Write FSM ────────────────────────────────────────────────────────────
localparam AW_IDLE = 2'd0;
localparam AW_W    = 2'd1;
localparam AW_B    = 2'd2;

reg [1:0]              aw_state;
reg [XID_WIDTH-1:0]    aw_id_r;
reg [ADDR_WIDTH-1:0]   aw_addr_r;      // running byte address
reg [7:0]              aw_beats_left;  // beats remaining incl current
reg [2:0]              aw_size_r;
reg [1:0]              aw_burst_r;
reg                    aw_err_r;

function align_error;
    input [2:0] size;
    input [ADDR_WIDTH-1:0] addr;
    begin
        case (size)
            3'd0: align_error = 1'b0;
            3'd1: align_error = addr[0];
            3'd2: align_error = |addr[1:0];
            3'd3: align_error = |addr[2:0];
            3'd4: align_error = |addr[3:0];
            default: align_error = 1'b1;
        endcase
    end
endfunction

assign awready = (aw_state == AW_IDLE) & cal_done_r;
assign wready  = (aw_state == AW_W)    & cal_done_r;

reg                    bvalid_r;
reg [XID_WIDTH-1:0]    bid_r;
assign bvalid = bvalid_r;
assign bid    = bid_r;
assign bresp  = aw_err_r ? RESP_SLVERR : RESP_OKAY;

// Write-index decode — computed once per AW_W cycle.
wire [BEAT_IDX_WIDTH-1:0] w_idx = addr_to_idx(aw_addr_r);
wire                       w_fire_int = wvalid & wready;

always @(posedge clk) begin
    if (rst) begin
        aw_state      <= AW_IDLE;
        aw_id_r       <= {XID_WIDTH{1'b0}};
        aw_addr_r     <= {ADDR_WIDTH{1'b0}};
        aw_beats_left <= 8'd0;
        aw_size_r     <= 3'd0;
        aw_burst_r    <= 2'd0;
        aw_err_r      <= 1'b0;
        bvalid_r      <= 1'b0;
        bid_r         <= {XID_WIDTH{1'b0}};
    end else begin
        case (aw_state)
            AW_IDLE: begin
                if (awvalid && awready) begin
                    aw_id_r       <= awid;
                    aw_addr_r     <= awaddr;
                    aw_beats_left <= awlen + 8'd1;
                    aw_size_r     <= awsize;
                    aw_burst_r    <= awburst;
                    aw_err_r      <= (awburst != 2'b01) ||
                                     (awsize  > 3'd4) ||
                                     align_error(awsize, awaddr);
                    aw_state      <= AW_W;
                end
            end

            AW_W: begin
                if (w_fire_int) begin
                    if (wlast != (aw_beats_left == 8'd1)) begin
                        aw_err_r <= 1'b1;
                    end
                    // INCR: advance by 2^awsize bytes.
                    if (aw_burst_r == 2'b01) begin
                        aw_addr_r <= aw_addr_r + (32'd1 << aw_size_r);
                    end
                    aw_beats_left <= aw_beats_left - 8'd1;
                    if (wlast || aw_beats_left == 8'd1) begin
                        aw_state <= AW_B;
                        bvalid_r <= 1'b1;
                        bid_r    <= aw_id_r;
                    end
                end
            end

            AW_B: begin
                if (bvalid && bready) begin
                    bvalid_r <= 1'b0;
                    aw_err_r <= 1'b0;
                    aw_state <= AW_IDLE;
                end
            end

            default: aw_state <= AW_IDLE;
        endcase
    end
end

// Per-byte-lane write — conditional on wstrb[i].  Vivado infers this as
// a BRAM with byte-write-enables (native on BRAM36 in BYTE_WRITE_ENABLE
// mode).  Each byte lane is a simple write-only port on its BRAM;
// reads happen through the separate read FSM on the same BRAM's other
// port.
always @(posedge clk) if (w_fire_int && !aw_err_r && wstrb[ 0]) mem_b0 [w_idx] <= wdata[  7:  0];
always @(posedge clk) if (w_fire_int && !aw_err_r && wstrb[ 1]) mem_b1 [w_idx] <= wdata[ 15:  8];
always @(posedge clk) if (w_fire_int && !aw_err_r && wstrb[ 2]) mem_b2 [w_idx] <= wdata[ 23: 16];
always @(posedge clk) if (w_fire_int && !aw_err_r && wstrb[ 3]) mem_b3 [w_idx] <= wdata[ 31: 24];
always @(posedge clk) if (w_fire_int && !aw_err_r && wstrb[ 4]) mem_b4 [w_idx] <= wdata[ 39: 32];
always @(posedge clk) if (w_fire_int && !aw_err_r && wstrb[ 5]) mem_b5 [w_idx] <= wdata[ 47: 40];
always @(posedge clk) if (w_fire_int && !aw_err_r && wstrb[ 6]) mem_b6 [w_idx] <= wdata[ 55: 48];
always @(posedge clk) if (w_fire_int && !aw_err_r && wstrb[ 7]) mem_b7 [w_idx] <= wdata[ 63: 56];
always @(posedge clk) if (w_fire_int && !aw_err_r && wstrb[ 8]) mem_b8 [w_idx] <= wdata[ 71: 64];
always @(posedge clk) if (w_fire_int && !aw_err_r && wstrb[ 9]) mem_b9 [w_idx] <= wdata[ 79: 72];
always @(posedge clk) if (w_fire_int && !aw_err_r && wstrb[10]) mem_b10[w_idx] <= wdata[ 87: 80];
always @(posedge clk) if (w_fire_int && !aw_err_r && wstrb[11]) mem_b11[w_idx] <= wdata[ 95: 88];
always @(posedge clk) if (w_fire_int && !aw_err_r && wstrb[12]) mem_b12[w_idx] <= wdata[103: 96];
always @(posedge clk) if (w_fire_int && !aw_err_r && wstrb[13]) mem_b13[w_idx] <= wdata[111:104];
always @(posedge clk) if (w_fire_int && !aw_err_r && wstrb[14]) mem_b14[w_idx] <= wdata[119:112];
always @(posedge clk) if (w_fire_int && !aw_err_r && wstrb[15]) mem_b15[w_idx] <= wdata[127:120];

// ── Read FSM ─────────────────────────────────────────────────────────────
localparam AR_IDLE  = 2'd0;
localparam AR_DELAY = 2'd1;  // SIM_DDR_READ_DELAY wait state
localparam AR_R     = 2'd2;

reg [1:0]              ar_state;
reg [15:0]             ar_delay_ctr;  // SIM_DDR_READ_DELAY counter
reg [ADDR_WIDTH-1:0]   ar_addr_r;
reg [7:0]              ar_beats_left;
reg [2:0]              ar_size_r;
reg [1:0]              ar_burst_r;
reg                    ar_err_r;

assign arready = (ar_state == AR_IDLE) & cal_done_r;

reg                    rvalid_r;
reg                    rlast_r;
reg [XID_WIDTH-1:0]    rid_r;
reg [DATA_WIDTH-1:0]   rdata_r;
assign rvalid = rvalid_r;
assign rlast  = rlast_r;
assign rid    = rid_r;
assign rdata  = rdata_r;
assign rresp  = ar_err_r ? RESP_SLVERR : RESP_OKAY;

// Helper — combinational assembly of 128-bit beat from byte lanes at a
// given index.  Vivado infers this as one synchronous read on each of
// the 16 BRAMs when the function result is registered into rdata_r.
function [DATA_WIDTH-1:0] read_beat;
    input [BEAT_IDX_WIDTH-1:0] idx;
    begin
        read_beat = {mem_b15[idx], mem_b14[idx], mem_b13[idx], mem_b12[idx],
                     mem_b11[idx], mem_b10[idx], mem_b9 [idx], mem_b8 [idx],
                     mem_b7 [idx], mem_b6 [idx], mem_b5 [idx], mem_b4 [idx],
                     mem_b3 [idx], mem_b2 [idx], mem_b1 [idx], mem_b0 [idx]};
    end
endfunction

always @(posedge clk) begin
    if (rst) begin
        ar_state      <= AR_IDLE;
        ar_addr_r     <= {ADDR_WIDTH{1'b0}};
        ar_beats_left <= 8'd0;
        ar_size_r     <= 3'd0;
        ar_burst_r    <= 2'd0;
        ar_err_r      <= 1'b0;
        ar_delay_ctr  <= 16'd0;
        rvalid_r      <= 1'b0;
        rlast_r       <= 1'b0;
        rid_r         <= {XID_WIDTH{1'b0}};
        rdata_r       <= {DATA_WIDTH{1'b0}};
    end else begin
        case (ar_state)
            AR_IDLE: begin
                if (arvalid && arready) begin
                    ar_addr_r     <= araddr;
                    ar_beats_left <= arlen + 8'd1;
                    ar_size_r     <= arsize;
                    ar_burst_r    <= arburst;
                    ar_err_r      <= (arburst != 2'b01) ||
                                     (arsize  > 3'd4) ||
                                     align_error(arsize, araddr);
                    rid_r         <= arid;
                    rdata_r       <= ((arburst != 2'b01) ||
                                      (arsize  > 3'd4) ||
                                      align_error(arsize, araddr))
                                     ? {DATA_WIDTH{1'b0}}
                                     : read_beat(addr_to_idx(araddr));
                    rlast_r       <= (arlen == 8'd0);
                    // SIM_DDR_READ_DELAY: if > 0, transit through AR_DELAY
                    // to mimic real-MIG variable read latency.  Default 0
                    // keeps the original zero-latency behaviour.
                    if (SIM_DDR_READ_DELAY > 0) begin
                        ar_state     <= AR_DELAY;
                        ar_delay_ctr <= SIM_DDR_READ_DELAY[15:0];
                    end else begin
                        ar_state <= AR_R;
                        rvalid_r <= 1'b1;
                    end
                end
            end

            AR_DELAY: begin
                if (ar_delay_ctr != 16'd0) begin
                    ar_delay_ctr <= ar_delay_ctr - 16'd1;
                end else begin
                    rvalid_r <= 1'b1;
                    ar_state <= AR_R;
                end
            end

            AR_R: begin
                if (rvalid && rready) begin
                    if (rlast_r) begin
                        rvalid_r <= 1'b0;
                        rlast_r  <= 1'b0;
                        ar_err_r <= 1'b0;
                        ar_state <= AR_IDLE;
                    end else begin
                        // Advance to next beat.
                        if (ar_burst_r == 2'b01) begin
                            ar_addr_r <= ar_addr_r + (32'd1 << ar_size_r);
                            rdata_r   <= ar_err_r ? {DATA_WIDTH{1'b0}} :
                                         read_beat(addr_to_idx(ar_addr_r + (32'd1 << ar_size_r)));
                        end else begin
                            rdata_r   <= ar_err_r ? {DATA_WIDTH{1'b0}} :
                                         read_beat(addr_to_idx(ar_addr_r));
                        end
                        ar_beats_left <= ar_beats_left - 8'd1;
                        rlast_r       <= (ar_beats_left == 8'd2);
                    end
                end
            end

            default: ar_state <= AR_IDLE;
        endcase
    end
end

// ── Synthesisable sim helpers — zero memory on reset (sim-only) ──────
// The sim tool zeroes `reg` arrays via --x-initial fast; real synth
// ignores this block because SIM_MODEL is only defined during sim
// builds.  Avoid `initial` to stay lint-clean under --x-initial.

`endif  // SIM_MIG_BRIDGE (else branch above)
`else  // !SIM_MODEL — real-hw build
// ══════════════════════════════════════════════════════════════════════════
// Real-hw build: pcie_test DDR4 MIG + repo-contract bridge
// ══════════════════════════════════════════════════════════════════════════

assign ddr_clk      = mig_ui_clk;
assign ddr_cal_done = mig_cal_done;

// ── UI-domain reset: pipelined + replicated (333 MHz timing) ──────────
// Both mig-UI consumers used to take `mig_ui_rst || !mig_cal_done`
// COMBINATIONALLY.  That put the MIG's own div_clk_rst_r1_reg (plus the
// cal_done OR) directly in front of every reset endpoint inside
// axi_core_to_mig_ui AND axi_ddr4_mig_bridge, at a 3.0 ns period.  It
// measured as 48 of the worst violated paths post-route:
//   u_mig_ddr4/inst/div_clk_rst_r1_reg/C -> u_repo_to_pcie_mig  (26)
//                                        -> u_core_to_mig_ui    (22)
// Reset is not a hot path, so re-registering it locally is free: one
// pipeline stage, then a SEPARATE replica per consumer so each drives
// its own fanout cone instead of sharing one high-fanout net.
//
// Timing of the two edges, deliberately asymmetric:
//   ASSERT  is 1 cycle late.  Safe: while the MIG is in reset its AXI
//           slave holds awready/wready/arready LOW, so neither consumer
//           can fire a new beat during that cycle -- they stall, they do
//           not push transactions into a resetting MIG.  (This project
//           has a documented history of mid-transaction reset
//           corruption, so that is the specific hazard being ruled out
//           here, not hand-waved.)
//   RELEASE is 2 cycles late, which only holds the consumers in reset
//           slightly longer -- always the safe direction.
wire mig_ui_rst_raw = mig_ui_rst || !mig_cal_done;

(* max_fanout = 64 *) reg mig_ui_rst_p1      = 1'b1;
(* max_fanout = 64 *) reg mig_ui_rst_bridge  = 1'b1;
(* max_fanout = 64 *) reg mig_ui_rst_coremig = 1'b1;

always @(posedge mig_ui_clk) begin
    if (mig_ui_rst_raw) begin
        mig_ui_rst_p1      <= 1'b1;
        mig_ui_rst_bridge  <= 1'b1;
        mig_ui_rst_coremig <= 1'b1;
    end else begin
        mig_ui_rst_p1      <= 1'b0;
        mig_ui_rst_bridge  <= mig_ui_rst_p1;
        mig_ui_rst_coremig <= mig_ui_rst_p1;
    end
end

wire [XID_WIDTH-1:0] ui_awid;
wire [ADDR_WIDTH-1:0] ui_awaddr;
wire [7:0] ui_awlen;
wire [2:0] ui_awsize;
wire [1:0] ui_awburst;
wire ui_awlock;
wire [3:0] ui_awcache;
wire [2:0] ui_awprot;
wire [3:0] ui_awqos;
wire ui_awuser;
wire ui_awvalid;
wire ui_awready;

wire [DATA_WIDTH-1:0] ui_wdata;
wire [STRB_WIDTH-1:0] ui_wstrb;
wire ui_wlast;
wire ui_wuser;
wire ui_wvalid;
wire ui_wready;

wire [XID_WIDTH-1:0] ui_bid;
wire [1:0] ui_bresp;
wire ui_bvalid;
wire ui_bready;

wire [XID_WIDTH-1:0] ui_arid;
wire [ADDR_WIDTH-1:0] ui_araddr;
wire [7:0] ui_arlen;
wire [2:0] ui_arsize;
wire [1:0] ui_arburst;
wire ui_arlock;
wire [3:0] ui_arcache;
wire [2:0] ui_arprot;
wire [3:0] ui_arqos;
wire ui_aruser;
wire ui_arvalid;
wire ui_arready;

wire [XID_WIDTH-1:0] ui_rid;
wire [DATA_WIDTH-1:0] ui_rdata;
wire [1:0] ui_rresp;
wire ui_rlast;
wire ui_rvalid;
wire ui_rready;

axi_async_bridge #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .ID_WIDTH  (XID_WIDTH),
    .USER_WIDTH(1)
) u_core_to_mig_ui (
    .s_clk(clk),
    .s_rst(rst),
    .s_awid(awid), .s_awaddr(awaddr), .s_awlen(awlen),
    .s_awsize(awsize), .s_awburst(awburst), .s_awlock(1'b0),
    .s_awcache(4'b0011), .s_awprot(3'b000), .s_awqos(4'b0000),
    .s_awuser(1'b0), .s_awvalid(awvalid), .s_awready(awready),
    .s_wdata(wdata), .s_wstrb(wstrb), .s_wlast(wlast),
    .s_wuser(1'b0), .s_wvalid(wvalid), .s_wready(wready),
    .s_bid(bid), .s_bresp(bresp), .s_buser(), .s_bvalid(bvalid),
    .s_bready(bready),
    .s_arid(arid), .s_araddr(araddr), .s_arlen(arlen),
    .s_arsize(arsize), .s_arburst(arburst), .s_arlock(1'b0),
    .s_arcache(4'b0011), .s_arprot(3'b000), .s_arqos(4'b0000),
    .s_aruser(1'b0), .s_arvalid(arvalid), .s_arready(arready),
    .s_rid(rid), .s_rdata(rdata), .s_rresp(rresp), .s_rlast(rlast),
    .s_ruser(), .s_rvalid(rvalid), .s_rready(rready),

    .m_clk(mig_ui_clk),
    .m_rst(mig_ui_rst_coremig),
    .m_awid(ui_awid), .m_awaddr(ui_awaddr), .m_awlen(ui_awlen),
    .m_awsize(ui_awsize), .m_awburst(ui_awburst), .m_awlock(ui_awlock),
    .m_awcache(ui_awcache), .m_awprot(ui_awprot), .m_awqos(ui_awqos),
    .m_awuser(ui_awuser), .m_awvalid(ui_awvalid), .m_awready(ui_awready),
    .m_wdata(ui_wdata), .m_wstrb(ui_wstrb), .m_wlast(ui_wlast),
    .m_wuser(ui_wuser), .m_wvalid(ui_wvalid), .m_wready(ui_wready),
    .m_bid(ui_bid), .m_bresp(ui_bresp), .m_buser(1'b0),
    .m_bvalid(ui_bvalid), .m_bready(ui_bready),
    .m_arid(ui_arid), .m_araddr(ui_araddr), .m_arlen(ui_arlen),
    .m_arsize(ui_arsize), .m_arburst(ui_arburst), .m_arlock(ui_arlock),
    .m_arcache(ui_arcache), .m_arprot(ui_arprot), .m_arqos(ui_arqos),
    .m_aruser(ui_aruser), .m_arvalid(ui_arvalid), .m_arready(ui_arready),
    .m_rid(ui_rid), .m_rdata(ui_rdata), .m_rresp(ui_rresp),
    .m_rlast(ui_rlast), .m_ruser(1'b0), .m_rvalid(ui_rvalid),
    .m_rready(ui_rready)
);

axi_ddr4_mig_bridge u_repo_to_pcie_mig (
    .clk(mig_ui_clk),
    .rst(mig_ui_rst_bridge),
    .s_awid(ui_awid), .s_awaddr(ui_awaddr), .s_awlen(ui_awlen),
    .s_awsize(ui_awsize), .s_awburst(ui_awburst),
    .s_awvalid(ui_awvalid), .s_awready(ui_awready),
    .s_wdata(ui_wdata), .s_wstrb(ui_wstrb), .s_wlast(ui_wlast),
    .s_wvalid(ui_wvalid), .s_wready(ui_wready),
    .s_bid(ui_bid), .s_bresp(ui_bresp), .s_bvalid(ui_bvalid),
    .s_bready(ui_bready),
    .s_arid(ui_arid), .s_araddr(ui_araddr), .s_arlen(ui_arlen),
    .s_arsize(ui_arsize), .s_arburst(ui_arburst),
    .s_arvalid(ui_arvalid), .s_arready(ui_arready),
    .s_rid(ui_rid), .s_rdata(ui_rdata), .s_rresp(ui_rresp),
    .s_rlast(ui_rlast), .s_rvalid(ui_rvalid), .s_rready(ui_rready),

    .m_awid(mig_awid), .m_awaddr(mig_awaddr), .m_awlen(mig_awlen),
    .m_awsize(mig_awsize), .m_awburst(mig_awburst),
    .m_awvalid(mig_awvalid), .m_awready(mig_awready),
    .m_wdata(mig_wdata), .m_wstrb(mig_wstrb), .m_wlast(mig_wlast),
    .m_wvalid(mig_wvalid), .m_wready(mig_wready),
    .m_bid(mig_bid), .m_bresp(mig_bresp), .m_bvalid(mig_bvalid),
    .m_bready(mig_bready),
    .m_arid(mig_arid), .m_araddr(mig_araddr), .m_arlen(mig_arlen),
    .m_arsize(mig_arsize), .m_arburst(mig_arburst),
    .m_arvalid(mig_arvalid), .m_arready(mig_arready),
    .m_rid(mig_rid), .m_rdata(mig_rdata), .m_rresp(mig_rresp),
    .m_rlast(mig_rlast), .m_rvalid(mig_rvalid), .m_rready(mig_rready)
);

// synthesis translate_off
wire _unused_real = &{1'b0, ui_awlock, ui_awcache, ui_awprot, ui_awqos,
                      ui_awuser, ui_wuser, ui_arlock, ui_arcache,
                      ui_arprot, ui_arqos, ui_aruser, 1'b0};
// synthesis translate_on

`endif  // SIM_MODEL

endmodule
/* verilator lint_on UNUSEDPARAM */

`default_nettype wire
