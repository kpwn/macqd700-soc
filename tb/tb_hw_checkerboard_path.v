// tb_hw_checkerboard_path.v -- CPU program -> AXI -> xbar -> VRAM smoke.
//
// Simulation-only integration wrapper for the first-hardware checkerboard
// program. The instruction side is driven by the C++ harness so the test
// stays fast and deterministic; the data side follows the hardware write path:
//
//   mac_top CPU D-AXI -> axi_narrow_to_wide -> axi_xbar S3 -> vram -> scanner
//
// The xbar/VRAM block is the existing tb_vram_xbar_e2e wrapper, reused here
// rather than duplicating its sink-slave plumbing.  This is not an SD-card
// model; the harness feeds the same flat program image through the instruction
// fetch port so this test stays focused on CPU data writes reaching VRAM.

`default_nettype none

module tb_hw_checkerboard_path (
    input  wire        clk,
    input  wire        rst,

    // Instruction fetch, driven by the C++ harness.
    output wire [31:0] if_addr,
    output wire        if_req,
    input  wire [127:0] if_rdata,
    input  wire        if_rvalid,
    input  wire        if_fault,

    // VRAM scanner readback.
    input  wire [12:0] rd_addr,
    input  wire        rd_en,
    // 4-byte group starting at rd_addr (vram.v RD_DATA_W=32): [31:24] is the
    // byte at rd_addr (valid at any alignment), [23:0] the bytes at +1/+2/+3
    // (valid only when rd_addr[1:0]==0).
    output wire [31:0] rd_data,
    output wire        rd_valid,

    output wire [31:0] dbg_pc,
    output wire [31:0] dbg_committed,
    output wire [31:0] dbg_macros,
    output wire [31:0] dbg_last_pc,
    output reg  [31:0] vram_store_count
);

    // ── mac_top CPU wrapper ──────────────────────────────────────────
    wire [31:0] core_daxi_awaddr;
    wire [2:0]  core_daxi_awprot;
    wire        core_daxi_awvalid;
    wire        core_daxi_awready;
    wire [31:0] core_daxi_wdata;
    wire [3:0]  core_daxi_wstrb;
    wire        core_daxi_wlast;
    wire        core_daxi_wvalid;
    wire        core_daxi_wready;
    wire [1:0]  core_daxi_bresp;
    wire        core_daxi_bvalid;
    wire        core_daxi_bready;
    wire [31:0] core_daxi_araddr;
    wire [2:0]  core_daxi_arprot;
    wire        core_daxi_arvalid;
    wire        core_daxi_arready;
    wire [31:0] core_daxi_rdata;
    wire [1:0]  core_daxi_rresp;
    wire        core_daxi_rlast;
    wire        core_daxi_rvalid;
    wire        core_daxi_rready;
    wire [4:0]  dbg_ccr_unused;
    wire        dbg_dcache_flush_done_unused;

    mac_top #(
        .RESET_PC(32'h4080_0000),
        .DCACHE_REFILL_BURST(1'b0)
    ) u_mac (
        .clk(clk),
        .rst(rst),
        .if_addr(if_addr),
        .if_req(if_req),
        .if_rdata(if_rdata),
        .if_rvalid(if_rvalid),
        .if_fault(if_fault),
        .daxi_awaddr(core_daxi_awaddr),
        .daxi_awprot(core_daxi_awprot),
        .daxi_awvalid(core_daxi_awvalid),
        .daxi_awready(core_daxi_awready),
        .daxi_wdata(core_daxi_wdata),
        .daxi_wstrb(core_daxi_wstrb),
        .daxi_wlast(core_daxi_wlast),
        .daxi_wvalid(core_daxi_wvalid),
        .daxi_wready(core_daxi_wready),
        .daxi_bresp(core_daxi_bresp),
        .daxi_bvalid(core_daxi_bvalid),
        .daxi_bready(core_daxi_bready),
        .daxi_araddr(core_daxi_araddr),
        .daxi_arprot(core_daxi_arprot),
        .daxi_arvalid(core_daxi_arvalid),
        .daxi_arready(core_daxi_arready),
        .daxi_rdata(core_daxi_rdata),
        .daxi_rresp(core_daxi_rresp),
        .daxi_rlast(core_daxi_rlast),
        .daxi_rvalid(core_daxi_rvalid),
        .daxi_rready(core_daxi_rready),
        .dbg_pc(dbg_pc),
        .dbg_ccr(dbg_ccr_unused),
        .dbg_committed(dbg_committed),
        .dbg_macros(dbg_macros),
        .dbg_last_pc(dbg_last_pc),
        .dbg_dcache_flush_req(1'b0),
        .dbg_dcache_flush_done(dbg_dcache_flush_done_unused),
        .dbg_precise_stop_req(1'b0),
        .dbg_precise_stop_keep_tag(5'd0),
        .dbg_break_pc_array_in(128'd0),
        .dbg_break_pc_enable_array_in(4'd0),
        .dbg_break_pc_skip_once_array_in(4'd0),
        .dbg_break_pc_skip_consume_array_out(),
        .dbg_break_uop_fire_out(),
        .dbg_break_pc_hit_slot_out(),
        .dbg_core_halt(1'b0),
        .dbg_arch_apply_en(1'b0),
        .dbg_arch_reg_load_en(1'b0),
        .dbg_arch_reg_load_idx(5'd0),
        .dbg_arch_reg_load_val(32'd0),
        .dbg_arch_ccr_load_en(1'b0),
        .dbg_arch_ccr_load_val(5'd0),
        .dbg_arch_ctrl_load_en(1'b0),
        .dbg_arch_ctrl_load_sel(4'd0),
        .dbg_arch_ctrl_load_val(32'd0),
        .dbg_arch_mmu_load_en(1'b0),
        .dbg_arch_mmu_load_sel(4'd0),
        .dbg_arch_mmu_load_val(32'd0),
        .dbg_arch_pc_load_en(1'b0),
        .dbg_arch_pc_load_val(32'd0)
    );

    // ── 32-bit CPU D-AXI to 128-bit xbar M0 ──────────────────────────
    wire [3:0]   m0_awid;
    wire [31:0]  m0_awaddr;
    wire [7:0]   m0_awlen;
    wire [2:0]   m0_awsize;
    wire [1:0]   m0_awburst;
    wire         m0_awvalid;
    wire         m0_awready;
    wire [127:0] m0_wdata;
    wire [15:0]  m0_wstrb;
    wire         m0_wlast;
    wire         m0_wvalid;
    wire         m0_wready;
    wire [3:0]   m0_bid;
    wire [1:0]   m0_bresp;
    wire         m0_bvalid;
    wire         m0_bready;
    wire [3:0]   m0_arid;
    wire [31:0]  m0_araddr;
    wire [7:0]   m0_arlen;
    wire [2:0]   m0_arsize;
    wire [1:0]   m0_arburst;
    wire         m0_arvalid;
    wire         m0_arready;
    wire [3:0]   m0_rid;
    wire [127:0] m0_rdata;
    wire [1:0]   m0_rresp;
    wire         m0_rlast;
    wire         m0_rvalid;
    wire         m0_rready;

    axi_narrow_to_wide #(
        .ID_WIDTH(4),
        .ID_TAG  (4'd0)
    ) u_cpu_n2w (
        .clk(clk),
        .rst(rst),
        .n_awaddr(core_daxi_awaddr),
        .n_awprot(core_daxi_awprot),
        .n_awvalid(core_daxi_awvalid),
        .n_awready(core_daxi_awready),
        .n_wdata(core_daxi_wdata),
        .n_wstrb(core_daxi_wstrb),
        .n_wlast(core_daxi_wlast),
        .n_wvalid(core_daxi_wvalid),
        .n_wready(core_daxi_wready),
        .n_bresp(core_daxi_bresp),
        .n_bvalid(core_daxi_bvalid),
        .n_bready(core_daxi_bready),
        .n_araddr(core_daxi_araddr),
        .n_arprot(core_daxi_arprot),
        .n_arvalid(core_daxi_arvalid),
        .n_arready(core_daxi_arready),
        .n_rdata(core_daxi_rdata),
        .n_rresp(core_daxi_rresp),
        .n_rlast(core_daxi_rlast),
        .n_rvalid(core_daxi_rvalid),
        .n_rready(core_daxi_rready),
        .w_awid(m0_awid),
        .w_awaddr(m0_awaddr),
        .w_awlen(m0_awlen),
        .w_awsize(m0_awsize),
        .w_awburst(m0_awburst),
        .w_awvalid(m0_awvalid),
        .w_awready(m0_awready),
        .w_wdata(m0_wdata),
        .w_wstrb(m0_wstrb),
        .w_wlast(m0_wlast),
        .w_wvalid(m0_wvalid),
        .w_wready(m0_wready),
        .w_bid(m0_bid),
        .w_bresp(m0_bresp),
        .w_bvalid(m0_bvalid),
        .w_bready(m0_bready),
        .w_arid(m0_arid),
        .w_araddr(m0_araddr),
        .w_arlen(m0_arlen),
        .w_arsize(m0_arsize),
        .w_arburst(m0_arburst),
        .w_arvalid(m0_arvalid),
        .w_arready(m0_arready),
        .w_rid(m0_rid),
        .w_rdata(m0_rdata),
        .w_rresp(m0_rresp),
        .w_rlast(m0_rlast),
        .w_rvalid(m0_rvalid),
        .w_rready(m0_rready)
    );

    // Existing xbar->VRAM E2E block.  Its M0 is driven by the CPU path above.
    tb_vram_xbar_e2e u_bus (
        .clk(clk),
        .rst(rst),
        .m0_awid(m0_awid),
        .m0_awaddr(m0_awaddr),
        .m0_awlen(m0_awlen),
        .m0_awsize(m0_awsize),
        .m0_awburst(m0_awburst),
        .m0_awvalid(m0_awvalid),
        .m0_awready(m0_awready),
        .m0_wdata(m0_wdata),
        .m0_wstrb(m0_wstrb),
        .m0_wlast(m0_wlast),
        .m0_wvalid(m0_wvalid),
        .m0_wready(m0_wready),
        .m0_bid(m0_bid),
        .m0_bresp(m0_bresp),
        .m0_bvalid(m0_bvalid),
        .m0_bready(m0_bready),
        .m0_arid(m0_arid),
        .m0_araddr(m0_araddr),
        .m0_arlen(m0_arlen),
        .m0_arsize(m0_arsize),
        .m0_arburst(m0_arburst),
        .m0_arvalid(m0_arvalid),
        .m0_arready(m0_arready),
        .m0_rid(m0_rid),
        .m0_rdata(m0_rdata),
        .m0_rresp(m0_rresp),
        .m0_rlast(m0_rlast),
        .m0_rvalid(m0_rvalid),
        .m0_rready(m0_rready),
        .rd_addr(rd_addr),
        .rd_en(rd_en),
        .rd_data(rd_data),
        .rd_valid(rd_valid)
    );

    reg pending_vram_store;
    wire aw_fire = core_daxi_awvalid && core_daxi_awready;
    wire b_fire  = core_daxi_bvalid && core_daxi_bready;
    wire aw_is_vram = (core_daxi_awaddr[31:20] == 12'hF90);

    always @(posedge clk) begin
        if (rst) begin
            vram_store_count  <= 32'd0;
            pending_vram_store <= 1'b0;
        end else begin
            if (aw_fire)
                pending_vram_store <= aw_is_vram;
            if (b_fire && pending_vram_store) begin
                vram_store_count  <= vram_store_count + 32'd1;
                pending_vram_store <= 1'b0;
            end
        end
    end

    wire _unused = &{1'b0, dbg_ccr_unused, dbg_dcache_flush_done_unused,
                     m0_bid, m0_bresp, m0_arready, m0_rid, m0_rdata,
                     m0_rresp, m0_rlast, m0_rvalid, 1'b0};

endmodule

`default_nettype wire
