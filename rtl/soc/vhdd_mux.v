// vhdd_mux.v — 2:1 router for the virtual-HDD contract (rtl/vhdd.vh)
//
// Role
// ════
// One SCSI master (rtl/mac/scsi.v) now answers to two target IDs, and
// each of those IDs is backed by a DIFFERENT volume provider:
//
//                        ┌──► a_*  vhdd_sd  (SD-card volume)
//   scsi.v ──m_*──► mux ─┤
//                        └──► b_*  provider B (SCSI ID 1; vhdd_net under
//                                   ENABLE_NET_VHDD.  The DDR-backed RAM
//                                   disk held this port until 2026-09-10)
//
// rtl/vhdd.vh says a wrapper that only forwards wires does not earn its
// place, and that the *selection* between providers is the thing that
// does.  This is that module and nothing more: `dev_sel` comes from the
// master's `vh_dev_sel`, latched when a SCSI selection succeeded, so it
// is already stable for the whole transaction it qualifies.  The mux
// adds no state and no arbitration — only one transaction is live on a
// SCSI bus at a time, so there is nothing to arbitrate.
//
// Latency
// ═══════
// Combinational — no clock, no state, no port for one.  Same reason as
// vhdd_sd: the contract requires req_lba to be valid on the same cycle
// req_go is high, and a register here would break that for both
// providers at once.
//
// Block size is VHDD_BLOCK_BYTES on every face; the mux never looks at
// a byte count, an LBA, or a block, so it is size-agnostic by
// construction.

`default_nettype none

`include "vhdd.vh"

module vhdd_mux (
    // 0 = route to provider A, 1 = route to provider B.
    input  wire        dev_sel,

    // ── master face (toward scsi.v) ───────────────────────────────────
    output wire [31:0] m_num_lbas,

    input  wire [31:0] m_chk_lba,
    input  wire [23:0] m_chk_blocks,
    output wire        m_chk_ok,

    input  wire        m_req_write,
    input  wire        m_req_multi,
    input  wire [31:0] m_req_lba,
    input  wire [15:0] m_req_block_count,
    input  wire        m_req_go,

    output wire        m_busy,
    output wire        m_done,
    output wire        m_error,

    output wire        m_rd_valid,
    output wire [7:0]  m_rd_data,
    input  wire        m_rd_ready,

    output wire        m_wr_ready,
    input  wire        m_wr_valid,
    input  wire [7:0]  m_wr_data,
    input  wire        m_wr_avail,

    // ── provider face A ───────────────────────────────────────────────
    input  wire [31:0] a_num_lbas,

    output wire [31:0] a_chk_lba,
    output wire [23:0] a_chk_blocks,
    input  wire        a_chk_ok,

    output wire        a_req_write,
    output wire        a_req_multi,
    output wire [31:0] a_req_lba,
    output wire [15:0] a_req_block_count,
    output wire        a_req_go,

    input  wire        a_busy,
    input  wire        a_done,
    input  wire        a_error,

    input  wire        a_rd_valid,
    input  wire [7:0]  a_rd_data,
    output wire        a_rd_ready,

    input  wire        a_wr_ready,
    output wire        a_wr_valid,
    output wire [7:0]  a_wr_data,
    output wire        a_wr_avail,

    // ── provider face B ───────────────────────────────────────────────
    input  wire [31:0] b_num_lbas,

    output wire [31:0] b_chk_lba,
    output wire [23:0] b_chk_blocks,
    input  wire        b_chk_ok,

    output wire        b_req_write,
    output wire        b_req_multi,
    output wire [31:0] b_req_lba,
    output wire [15:0] b_req_block_count,
    output wire        b_req_go,

    input  wire        b_busy,
    input  wire        b_done,
    input  wire        b_error,

    input  wire        b_rd_valid,
    input  wire [7:0]  b_rd_data,
    output wire        b_rd_ready,

    input  wire        b_wr_ready,
    output wire        b_wr_valid,
    output wire [7:0]  b_wr_data,
    output wire        b_wr_avail
);

    // ── master → providers: broadcast ─────────────────────────────────
    // Everything the master DRIVES goes to both providers unconditionally.
    // The extent probe in particular has no side effect (rtl/vhdd.vh), so
    // letting the idle provider evaluate it costs nothing and keeps the
    // path purely combinational.
    assign a_chk_lba         = m_chk_lba;
    assign b_chk_lba         = m_chk_lba;
    assign a_chk_blocks      = m_chk_blocks;
    assign b_chk_blocks      = m_chk_blocks;

    assign a_req_write       = m_req_write;
    assign b_req_write       = m_req_write;
    assign a_req_multi       = m_req_multi;
    assign b_req_multi       = m_req_multi;
    assign a_req_lba         = m_req_lba;
    assign b_req_lba         = m_req_lba;
    assign a_req_block_count = m_req_block_count;
    assign b_req_block_count = m_req_block_count;

    assign a_wr_valid        = m_wr_valid;
    assign b_wr_valid        = m_wr_valid;
    assign a_wr_data         = m_wr_data;
    assign b_wr_data         = m_wr_data;

    // ── req_go: the only master→provider signal that is GATED ─────────
    // req_go is what actually starts work.  Broadcasting it would start
    // BOTH providers on every request; gating it here is the whole point
    // of the module.
    assign a_req_go = m_req_go & ~dev_sel;
    assign b_req_go = m_req_go &  dev_sel;

    // ── rd_ready: selected gets the real value, unselected gets 1 ─────
    // rtl/vhdd.vh: "an abandoned stream must be allowed to drain into the
    // void rather than park `busy` forever."  An unselected provider may
    // still be finishing a previous request (a multi-block read whose
    // tail outlived its SCSI transaction), and nothing is listening to
    // it.  Feeding it a constant 1 lets it run its stream out and reach
    // `done`; feeding it the master's real rd_ready — which the master
    // drives from the ring state of the OTHER provider's transfer —
    // could hold it low indefinitely and wedge it in `busy`, which is
    // exactly the unbounded-provider failure that header forbids.
    assign a_rd_ready = dev_sel ? 1'b1 : m_rd_ready;
    assign b_rd_ready = dev_sel ? m_rd_ready : 1'b1;

    // ── wr_avail: same treatment, same reason ─────────────────────────
    // The write-side twin of rd_ready.  The unselected provider gets a
    // constant 1 so an abandoned write stream can drain to `done`
    // instead of parking in `busy` on a level the master is driving
    // from the OTHER provider's ring.
    assign a_wr_avail = dev_sel ? 1'b1 : m_wr_avail;
    assign b_wr_avail = dev_sel ? m_wr_avail : 1'b1;

    // ── providers → master: select ────────────────────────────────────
    assign m_chk_ok   = dev_sel ? b_chk_ok   : a_chk_ok;
    assign m_busy     = dev_sel ? b_busy     : a_busy;
    assign m_done     = dev_sel ? b_done     : a_done;
    assign m_error    = dev_sel ? b_error    : a_error;
    assign m_rd_valid = dev_sel ? b_rd_valid : a_rd_valid;
    assign m_rd_data  = dev_sel ? b_rd_data  : a_rd_data;
    assign m_wr_ready = dev_sel ? b_wr_ready : a_wr_ready;
    assign m_num_lbas = dev_sel ? b_num_lbas : a_num_lbas;

endmodule

`default_nettype wire
