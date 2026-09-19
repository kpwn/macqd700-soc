// rtl/soc/cpu_stub.v — standalone CPU socket tie-off (Phase 3)
//
// Presents the REAL dual-AXI CPU↔SoC socket defined in
// rtl/soc/cpu_socket.vh:
//
//   * axi_i_*  — instruction read master, held idle (arvalid=0,
//                rready=1 to drain any stray beat).
//   * axi_d_*  — data read/write master, held idle (awvalid/wvalid/
//                arvalid=0, bready/rready=1 to drain).
//   * dbg_axi_*— debug/control AXI(-Lite) SLAVE.  Terminates EVERY
//                transaction with OKAY and rdata=0 so the SoC's
//                JTAG-AXI master can poke the debug window without
//                wedging when no real CPU is present.
//   * ipl_ack  = 0; cpu_cold_reset_* / cpu_ram_window_lg2 = idle.
//
// This is the STANDALONE placeholder.  The real CPU arrives via the
// m68k submodule + m68k_axi_wrapper in Phase 4/5, presenting EXACTLY
// this same socket surface, so binding it only swaps the instantiated
// module name in fpga_top_cpu.vh — no SoC rewiring.
//
// Pure combinational tie-off — no clocked logic, no functional
// behaviour.  Parameters mirror the future wrapper for instantiation
// compatibility; they have no effect on the stub.

`include "cpu_socket.vh"

module cpu_stub #(
    parameter [31:0] RESET_PC            = 32'h4080_0000,
    parameter        DCACHE_REFILL_BURST = 1'b0,
    parameter        FETCH_RESET_VECTORS = 1'b0,
    // Socket-surface parity with m68k_axi_wrapper: the real CPU threads
    // BUILD_ID to its debug_ctrl (OFF_BUILD_ID readback); the stub just
    // accepts and ignores it so the shared instantiation binds cleanly.
    parameter [31:0] BUILD_ID            = 32'h0000_0000,
    parameter integer AXI_I_DW = `CPU_SOCKET_AXI_I_DW,
    parameter integer AXI_D_DW = `CPU_SOCKET_AXI_D_DW,
    parameter integer AXI_AW = `CPU_SOCKET_AXI_AW,
    parameter integer AXI_IW = `CPU_SOCKET_AXI_IW,
    parameter integer DBG_AW = `CPU_SOCKET_DBG_AW,
    parameter integer DBG_DW = `CPU_SOCKET_DBG_DW
) (
    input  wire                 clk,
    input  wire                 rst,

    // ── Instruction read master (AR/R only) — AXI_I_DW (256) ─────────
    output wire [AXI_IW-1:0]    axi_i_arid,
    output wire [AXI_AW-1:0]    axi_i_araddr,
    output wire [7:0]           axi_i_arlen,
    output wire [2:0]           axi_i_arsize,
    output wire [1:0]           axi_i_arburst,
    output wire                 axi_i_arvalid,
    input  wire                 axi_i_arready,
    input  wire [AXI_IW-1:0]    axi_i_rid,
    input  wire [AXI_I_DW-1:0]  axi_i_rdata,
    input  wire [1:0]           axi_i_rresp,
    input  wire                 axi_i_rlast,
    input  wire                 axi_i_rvalid,
    output wire                 axi_i_rready,

    // ── Data read/write master (AW/W/B/AR/R) — AXI_D_DW (128) ────────
    output wire [AXI_IW-1:0]    axi_d_awid,
    output wire [AXI_AW-1:0]    axi_d_awaddr,
    output wire [7:0]           axi_d_awlen,
    output wire [2:0]           axi_d_awsize,
    output wire [1:0]           axi_d_awburst,
    output wire                 axi_d_awvalid,
    input  wire                 axi_d_awready,
    output wire [AXI_D_DW-1:0]  axi_d_wdata,
    output wire [AXI_D_DW/8-1:0] axi_d_wstrb,
    output wire                 axi_d_wlast,
    output wire                 axi_d_wvalid,
    input  wire                 axi_d_wready,
    input  wire [AXI_IW-1:0]    axi_d_bid,
    input  wire [1:0]           axi_d_bresp,
    input  wire                 axi_d_bvalid,
    output wire                 axi_d_bready,
    output wire [AXI_IW-1:0]    axi_d_arid,
    output wire [AXI_AW-1:0]    axi_d_araddr,
    output wire [7:0]           axi_d_arlen,
    output wire [2:0]           axi_d_arsize,
    output wire [1:0]           axi_d_arburst,
    output wire                 axi_d_arvalid,
    input  wire                 axi_d_arready,
    input  wire [AXI_IW-1:0]    axi_d_rid,
    input  wire [AXI_D_DW-1:0]  axi_d_rdata,
    input  wire [1:0]           axi_d_rresp,
    input  wire                 axi_d_rlast,
    input  wire                 axi_d_rvalid,
    output wire                 axi_d_rready,

    // ── Debug/control AXI(-Lite) SLAVE ────────────────────────────────
    input  wire [DBG_AW-1:0]    dbg_axi_awaddr,
    input  wire                 dbg_axi_awvalid,
    output wire                 dbg_axi_awready,
    input  wire [DBG_DW-1:0]    dbg_axi_wdata,
    input  wire [DBG_DW/8-1:0]  dbg_axi_wstrb,
    input  wire                 dbg_axi_wvalid,
    output wire                 dbg_axi_wready,
    output wire [1:0]           dbg_axi_bresp,
    output wire                 dbg_axi_bvalid,
    input  wire                 dbg_axi_bready,
    input  wire [DBG_AW-1:0]    dbg_axi_araddr,
    input  wire                 dbg_axi_arvalid,
    output wire                 dbg_axi_arready,
    output wire [DBG_DW-1:0]    dbg_axi_rdata,
    output wire [1:0]           dbg_axi_rresp,
    output wire                 dbg_axi_rvalid,
    input  wire                 dbg_axi_rready,

    // ── Interrupt seam ────────────────────────────────────────────────
    input  wire [2:0]           cpu_ipl,
    output wire                 ipl_ack,

    // ── SoC-fabric control group (CPU → SoC) ──────────────────────────
    output wire                 cpu_cold_reset_pulse,
    output wire                 cpu_cold_reset_hold,
    output wire                 cpu_peripheral_reset,
    output wire [5:0]           cpu_ram_window_lg2,
    output wire [6:0]           cpu_mon_sense,
    input  wire                 init_done_seen
`ifdef ILA_ENABLE
    ,
    // ── ILA-only debug-export group — socket parity with
    // m68k_axi_wrapper.v's ILA_ENABLE port group (see that file for the
    // full rationale).  The stub has no CPU internals to source these
    // from, so they are all tied to constant 0 below; they exist purely
    // so the shared fpga_top instantiation (CPU=stub vs CPU=m68k) binds
    // with one identical port-connection list regardless of which
    // module is selected.
    output wire [25:0] dbg_ila_commit_bundle_w,
    output wire [4:0]  dbg_ila_rob_arch_dst_w,
    output wire [6:0]  dbg_ila_rob_phys_dst_w,
    output wire [6:0]  dbg_ila_rob_phys_old_w,
    output wire        dbg_ila_rob_pop_w,
    output wire        dbg_ila_rob_is_last_uop_w,
    output wire        dbg_ila_flush_en_w,
    output wire [31:0] dbg_ila_rob_pc_w,
    output wire [31:0] dbg_ila_arch_a7_w,
    output wire        dbg_ila_dc_aw_is_evict_w,
    output wire        dbg_ila_cdb0_en_w,
    output wire        dbg_ila_cdb0_has_dst_w,
    output wire [6:0]  dbg_ila_cdb0_phys_w,
    output wire [31:0] dbg_ila_cdb0_data_w,
    output wire        dbg_ila_cdb1_en_w,
    output wire        dbg_ila_cdb1_has_dst_w,
    output wire [6:0]  dbg_ila_cdb1_phys_w,
    output wire [31:0] dbg_ila_cdb1_data_w,
    output wire        dbg_ila_cdb2_en_w,
    output wire        dbg_ila_cdb2_has_dst_w,
    output wire [6:0]  dbg_ila_cdb2_phys_w,
    output wire        dbg_ila_cdb_alu_hi_en_w,
    output wire [6:0]  dbg_ila_cdb_alu_hi_phys_w,
    output wire        dbg_ila_a7_writeback_en_w,
    output wire [6:0]  dbg_ila_a7_writeback_phys_w,
    output wire [31:0] dbg_ila_a7_writeback_val_w,
    output wire        dbg_ila_vec_ssp_valid_w,
    output wire        dbg_ila_sp_slot_write_en_w,
    output wire [1:0]  dbg_ila_sp_slot_write_sel_w,
    output wire [6:0]  dbg_ila_committed_a7_phys_w,
    output wire [31:0] dbg_ila_real_a7_val_w,
    output wire [31:0] dbg_ila_prf01_w,
    output wire [6:0]  dbg_ila_snap_prf_idx_w,
    output wire        dbg_ila_effective_halt_w,
    output wire [31:0] dbg_ila_if_pc_w,
    output wire [31:0] dbg_ila_pred_pc_w,
    output wire        dbg_ila_supervisor_mode_w,
    output wire [6:0]  dbg_ila_exc_held_a7_phys_w,
    output wire        dbg_ila_take_rte_finalize_w,
    output wire [31:0] dbg_ila_exc_held_fault_pc_w,
    output wire [31:0] dbg_ila_rob_brtgt_w,
    output wire [31:0] dbg_ila_dc_rdata_w,
    output wire        dbg_ila_take_finalize_w,
    output wire        dbg_ila_rob_brt_w,
    output wire [5:0]  dbg_ila_snap_tag_w,
    output wire [31:0] dbg_ila_snap_ea_w,
    // ── Raw (pre-widen) data-master AXI W-channel snapshot — socket
    // parity with m68k_axi_wrapper.v's raw_daxi_* group (no real CPU
    // bus to source from, so all tied to 0/idle below).
    output wire [31:0] raw_daxi_awaddr,
    output wire        raw_daxi_awvalid,
    output wire        raw_daxi_awready,
    output wire        raw_daxi_wvalid,
    output wire        raw_daxi_wready,
    output wire        raw_daxi_bvalid,
    output wire        raw_daxi_bready,
    output wire        raw_daxi_wlast
`endif
);

    // ── Instruction master: never request a fetch ────────────────────
    assign axi_i_arid    = {AXI_IW{1'b0}};
    assign axi_i_araddr  = {AXI_AW{1'b0}};
    assign axi_i_arlen   = 8'b0;
    assign axi_i_arsize  = 3'b0;
    assign axi_i_arburst = 2'b0;
    assign axi_i_arvalid = 1'b0;
    assign axi_i_rready  = 1'b1;   // drain any stray R beat

    // ── Data master: never request a read or write ───────────────────
    assign axi_d_awid    = {AXI_IW{1'b0}};
    assign axi_d_awaddr  = {AXI_AW{1'b0}};
    assign axi_d_awlen   = 8'b0;
    assign axi_d_awsize  = 3'b0;
    assign axi_d_awburst = 2'b0;
    assign axi_d_awvalid = 1'b0;
    assign axi_d_wdata   = {AXI_D_DW{1'b0}};
    assign axi_d_wstrb   = {(AXI_D_DW/8){1'b0}};
    assign axi_d_wlast   = 1'b0;
    assign axi_d_wvalid  = 1'b0;
    assign axi_d_bready  = 1'b1;   // drain any stray B response
    assign axi_d_arid    = {AXI_IW{1'b0}};
    assign axi_d_araddr  = {AXI_AW{1'b0}};
    assign axi_d_arlen   = 8'b0;
    assign axi_d_arsize  = 3'b0;
    assign axi_d_arburst = 2'b0;
    assign axi_d_arvalid = 1'b0;
    assign axi_d_rready  = 1'b1;   // drain any stray R response

    // ── Debug slave: terminate every transaction OKAY, rdata=0 ───────
    // Single-cycle handshakes — accept addr+data together and respond
    // the same cycle.  This never wedges the JTAG-AXI master: a write
    // gets B/OKAY as soon as both AW and W are valid; a read returns
    // R/OKAY with rdata=0 as soon as AR is valid.
    assign dbg_axi_awready = dbg_axi_awvalid && dbg_axi_wvalid;
    assign dbg_axi_wready  = dbg_axi_awvalid && dbg_axi_wvalid;
    assign dbg_axi_bvalid  = dbg_axi_awvalid && dbg_axi_wvalid;
    assign dbg_axi_bresp   = 2'b00;   // OKAY
    assign dbg_axi_arready = dbg_axi_arvalid;
    assign dbg_axi_rvalid  = dbg_axi_arvalid;
    assign dbg_axi_rdata   = {DBG_DW{1'b0}};
    assign dbg_axi_rresp   = 2'b00;   // OKAY

    // ── IRQ seam ──────────────────────────────────────────────────────
    assign ipl_ack = 1'b0;

    // ── SoC-fabric control group: idle ───────────────────────────────
    assign cpu_cold_reset_pulse = 1'b0;
    assign cpu_cold_reset_hold  = 1'b0;
    assign cpu_peripheral_reset = 1'b0;
    assign cpu_ram_window_lg2   = 6'd0;
    // NOT idle-zero: 0 is a legal monitor-sense code (Mac 21" Color
    // Display), so tying it low would silently change what a standalone
    // SoC build reports on the DAFB sense pins.  7'h06 is the historical
    // video.v MONITOR_TYPE parameter default the real CPU's CSR also
    // powers up to, so CPU=stub and CPU=m68k agree out of reset.
    assign cpu_mon_sense        = 7'h06;

`ifdef ILA_ENABLE
    // ── ILA-only debug-export group: idle (no real CPU to source from).
    assign dbg_ila_commit_bundle_w    = 26'd0;
    assign dbg_ila_rob_arch_dst_w     = 5'd0;
    assign dbg_ila_rob_phys_dst_w     = 7'd0;
    assign dbg_ila_rob_phys_old_w     = 7'd0;
    assign dbg_ila_rob_pop_w          = 1'b0;
    assign dbg_ila_rob_is_last_uop_w  = 1'b0;
    assign dbg_ila_flush_en_w         = 1'b0;
    assign dbg_ila_rob_pc_w           = 32'd0;
    assign dbg_ila_arch_a7_w          = 32'd0;
    assign dbg_ila_dc_aw_is_evict_w   = 1'b0;
    assign dbg_ila_cdb0_en_w          = 1'b0;
    assign dbg_ila_cdb0_has_dst_w     = 1'b0;
    assign dbg_ila_cdb0_phys_w        = 7'd0;
    assign dbg_ila_cdb0_data_w        = 32'd0;
    assign dbg_ila_cdb1_en_w          = 1'b0;
    assign dbg_ila_cdb1_has_dst_w     = 1'b0;
    assign dbg_ila_cdb1_phys_w        = 7'd0;
    assign dbg_ila_cdb1_data_w        = 32'd0;
    assign dbg_ila_cdb2_en_w          = 1'b0;
    assign dbg_ila_cdb2_has_dst_w     = 1'b0;
    assign dbg_ila_cdb2_phys_w        = 7'd0;
    assign dbg_ila_cdb_alu_hi_en_w    = 1'b0;
    assign dbg_ila_cdb_alu_hi_phys_w  = 7'd0;
    assign dbg_ila_a7_writeback_en_w  = 1'b0;
    assign dbg_ila_a7_writeback_phys_w= 7'd0;
    assign dbg_ila_a7_writeback_val_w = 32'd0;
    assign dbg_ila_vec_ssp_valid_w    = 1'b0;
    assign dbg_ila_sp_slot_write_en_w = 1'b0;
    assign dbg_ila_sp_slot_write_sel_w= 2'd0;
    assign dbg_ila_committed_a7_phys_w= 7'd0;
    assign dbg_ila_real_a7_val_w      = 32'd0;
    assign dbg_ila_prf01_w            = 32'd0;
    assign dbg_ila_snap_prf_idx_w     = 7'd0;
    assign dbg_ila_effective_halt_w   = 1'b0;
    assign dbg_ila_if_pc_w            = 32'd0;
    assign dbg_ila_pred_pc_w          = 32'd0;
    assign dbg_ila_supervisor_mode_w  = 1'b0;
    assign dbg_ila_exc_held_a7_phys_w = 7'd0;
    assign dbg_ila_take_rte_finalize_w= 1'b0;
    assign dbg_ila_exc_held_fault_pc_w= 32'd0;
    assign dbg_ila_rob_brtgt_w        = 32'd0;
    assign dbg_ila_dc_rdata_w         = 32'd0;
    assign dbg_ila_take_finalize_w    = 1'b0;
    assign dbg_ila_rob_brt_w          = 1'b0;
    assign dbg_ila_snap_tag_w         = 6'd0;
    assign dbg_ila_snap_ea_w          = 32'd0;
    assign raw_daxi_awaddr            = 32'd0;
    assign raw_daxi_awvalid           = 1'b0;
    assign raw_daxi_awready            = 1'b0;
    assign raw_daxi_wvalid            = 1'b0;
    assign raw_daxi_wready            = 1'b0;
    assign raw_daxi_bvalid            = 1'b0;
    assign raw_daxi_bready            = 1'b0;
    assign raw_daxi_wlast             = 1'b0;
`endif

    // ── Anti-lint: sink all unused inputs ─────────────────────────────
    wire _unused_stub_inputs = &{1'b0,
        clk, rst,
        axi_i_arready, axi_i_rid, axi_i_rdata, axi_i_rresp,
        axi_i_rlast, axi_i_rvalid,
        axi_d_awready, axi_d_wready, axi_d_bid, axi_d_bresp,
        axi_d_bvalid, axi_d_arready, axi_d_rid, axi_d_rdata,
        axi_d_rresp, axi_d_rlast, axi_d_rvalid,
        dbg_axi_awaddr, dbg_axi_wdata, dbg_axi_wstrb, dbg_axi_bready,
        dbg_axi_araddr, dbg_axi_rready,
        cpu_ipl, init_done_seen};

endmodule
