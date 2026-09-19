// rtl/fpga_top_debug_host.vh — included from rtl/fpga_top.v
//
// Host debug master (xbar M1): PCIe XDMA with CDC bridge (PCIE_XDMA_ENABLE) or Xilinx JTAG-to-AXI (JTAG_AXI_ENABLE), else stubbed off.
//
// Generated mechanically from rtl/fpga_top.v during the fpga_top split
// (agent/p3-fpga-top-split). No functional change: this file contains
// the original lines verbatim, inside the fpga_top module scope via
// `include. Do not add module/endmodule wrappers or nettype directives.

    // ═══════════════════════════════════════════════════════════════════
    // Host debug master (M1) — XDMA bypass or Xilinx JTAG-to-AXI when enabled
    // ═══════════════════════════════════════════════════════════════════
    wire [3:0]   xdma_awid;
    wire [31:0]  xdma_awaddr;
    wire [7:0]   xdma_awlen;
    wire [2:0]   xdma_awsize;
    wire [1:0]   xdma_awburst;
    wire         xdma_awvalid;
    wire         xdma_awready;
    wire [127:0] xdma_wdata;
    wire [15:0]  xdma_wstrb;
    wire         xdma_wlast;
    wire         xdma_wvalid;
    wire         xdma_wready;
    wire [3:0]   xdma_bid;
    wire [1:0]   xdma_bresp;
    wire         xdma_bvalid;
    wire         xdma_bready;
    wire [3:0]   xdma_arid;
    wire [31:0]  xdma_araddr;
    wire [7:0]   xdma_arlen;
    wire [2:0]   xdma_arsize;
    wire [1:0]   xdma_arburst;
    wire         xdma_arvalid;
    wire         xdma_arready;
    wire [3:0]   xdma_rid;
    wire [127:0] xdma_rdata;
    wire [1:0]   xdma_rresp;
    wire         xdma_rlast;
    wire         xdma_rvalid;
    wire         xdma_rready;

`ifdef PCIE_XDMA_ENABLE
    // XDMA owns its own 250 MHz axi_aclk.  Keep the system xbar on core_clk
    // and cross from M_AXI_BYPASS through the reusable AXI CDC bridge.  The
    // regular descriptor DMA M_AXI port is deliberately not connected yet;
    // use the bypass BAR for host debug access until the DDR burst path is
    // validated end to end.
    wire         pcie_axi_aclk;
    wire         pcie_axi_aresetn;
    wire         pcie_user_lnk_up;
    // GT power-good, exported by the IP as the CE it would hand a
    // dedicated external sysclk buffer (ext_sys_clk_bufg=true).  We feed
    // sys_clk from the free-running video_ref_clk instead, so this is
    // observability only (VIO probe_in1[1] in PCIe builds).
    wire         pcie_sysclk_ce;

    wire [63:0]  pcie_byp_awaddr;
    wire [7:0]   pcie_byp_awlen;
    wire [2:0]   pcie_byp_awsize;
    wire [1:0]   pcie_byp_awburst;
    wire [2:0]   pcie_byp_awprot;
    wire         pcie_byp_awvalid;
    wire         pcie_byp_awready;
    wire         pcie_byp_awlock;
    wire [3:0]   pcie_byp_awcache;
    wire [3:0]   pcie_byp_awid;
    wire [127:0] pcie_byp_wdata;
    wire [15:0]  pcie_byp_wstrb;
    wire         pcie_byp_wlast;
    wire         pcie_byp_wvalid;
    wire         pcie_byp_wready;
    wire [3:0]   pcie_byp_bid;
    wire [1:0]   pcie_byp_bresp;
    wire         pcie_byp_bvalid;
    wire         pcie_byp_bready;
    wire [63:0]  pcie_byp_araddr;
    wire [7:0]   pcie_byp_arlen;
    wire [2:0]   pcie_byp_arsize;
    wire [1:0]   pcie_byp_arburst;
    wire [2:0]   pcie_byp_arprot;
    wire         pcie_byp_arvalid;
    wire         pcie_byp_arready;
    wire         pcie_byp_arlock;
    wire [3:0]   pcie_byp_arcache;
    wire [3:0]   pcie_byp_arid;
    wire [3:0]   pcie_byp_rid;
    wire [127:0] pcie_byp_rdata;
    wire [1:0]   pcie_byp_rresp;
    wire         pcie_byp_rlast;
    wire         pcie_byp_rvalid;
    wire         pcie_byp_rready;

    // ext_sys_clk_bufg=true: the IP has no internal BUFG_GT on sys_clk;
    // feed it the already-buffered DIV=0 100 MHz fabric clock instead of
    // raw fabric_clk_odiv2 (see fpga_top_clocks.vh PCIe/XDMA note).
    // sys_clk_ce_out is the CE the IP would hand to a dedicated external
    // buffer; video_ref_clk is free-running, so it is intentionally
    // unconnected.
    design_1_xdma_0_0 u_pcie_xdma (
        .sys_clk        (video_ref_clk),
        .sys_clk_gt     (fabric_clk_gt),
        .sys_clk_ce_out (pcie_sysclk_ce),
        // T19 = slot PERST# = board cpu_resetn: same physical pin, one
        // port.  Host-driven PERST# resets both the XDMA and the SoC.
        .sys_rst_n      (cpu_resetn),
        .user_lnk_up (pcie_user_lnk_up),
        .pci_exp_txp (pcie_txp),
        .pci_exp_txn (pcie_txn),
        .pci_exp_rxp (pcie_rxp),
        .pci_exp_rxn (pcie_rxn),
        .axi_aclk    (pcie_axi_aclk),
        .axi_aresetn (pcie_axi_aresetn),

        .usr_irq_req      (9'b0),
        .usr_irq_ack      (),
        .msi_enable       (),
        .msix_enable      (),
        .msi_vector_width (),

        // Descriptor DMA M_AXI: intentionally parked for this skeleton.
        .m_axi_awready(1'b0),
        .m_axi_wready (1'b0),
        .m_axi_bid    (4'b0),
        .m_axi_bresp  (`AXI_RESP_DECERR),
        .m_axi_bvalid (1'b0),
        .m_axi_arready(1'b0),
        .m_axi_rid    (4'b0),
        .m_axi_rdata  (128'b0),
        .m_axi_rresp  (`AXI_RESP_DECERR),
        .m_axi_rlast  (1'b0),
        .m_axi_rvalid (1'b0),
        .m_axi_awid   (),
        .m_axi_awaddr (),
        .m_axi_awlen  (),
        .m_axi_awsize (),
        .m_axi_awburst(),
        .m_axi_awprot (),
        .m_axi_awvalid(),
        .m_axi_awlock (),
        .m_axi_awcache(),
        .m_axi_wdata  (),
        .m_axi_wstrb  (),
        .m_axi_wlast  (),
        .m_axi_wvalid (),
        .m_axi_bready (),
        .m_axi_arid   (),
        .m_axi_araddr (),
        .m_axi_arlen  (),
        .m_axi_arsize (),
        .m_axi_arburst(),
        .m_axi_arprot (),
        .m_axi_arvalid(),
        .m_axi_arlock (),
        .m_axi_arcache(),
        .m_axi_rready (),

        .cfg_mgmt_addr           (19'b0),
        .cfg_mgmt_write          (1'b0),
        .cfg_mgmt_write_data     (32'b0),
        .cfg_mgmt_byte_enable    (4'b0),
        .cfg_mgmt_read           (1'b0),
        .cfg_mgmt_read_data      (),
        .cfg_mgmt_read_write_done(),

        .m_axib_awid    (pcie_byp_awid),
        .m_axib_awaddr  (pcie_byp_awaddr),
        .m_axib_awlen   (pcie_byp_awlen),
        .m_axib_awsize  (pcie_byp_awsize),
        .m_axib_awburst (pcie_byp_awburst),
        .m_axib_awprot  (pcie_byp_awprot),
        .m_axib_awvalid (pcie_byp_awvalid),
        .m_axib_awready (pcie_byp_awready),
        .m_axib_awlock  (pcie_byp_awlock),
        .m_axib_awcache (pcie_byp_awcache),
        .m_axib_wdata   (pcie_byp_wdata),
        .m_axib_wstrb   (pcie_byp_wstrb),
        .m_axib_wlast   (pcie_byp_wlast),
        .m_axib_wvalid  (pcie_byp_wvalid),
        .m_axib_wready  (pcie_byp_wready),
        .m_axib_bid     (pcie_byp_bid),
        .m_axib_bresp   (pcie_byp_bresp),
        .m_axib_bvalid  (pcie_byp_bvalid),
        .m_axib_bready  (pcie_byp_bready),
        .m_axib_arid    (pcie_byp_arid),
        .m_axib_araddr  (pcie_byp_araddr),
        .m_axib_arlen   (pcie_byp_arlen),
        .m_axib_arsize  (pcie_byp_arsize),
        .m_axib_arburst (pcie_byp_arburst),
        .m_axib_arprot  (pcie_byp_arprot),
        .m_axib_arvalid (pcie_byp_arvalid),
        .m_axib_arready (pcie_byp_arready),
        .m_axib_arlock  (pcie_byp_arlock),
        .m_axib_arcache (pcie_byp_arcache),
        .m_axib_rid     (pcie_byp_rid),
        .m_axib_rdata   (pcie_byp_rdata),
        .m_axib_rresp   (pcie_byp_rresp),
        .m_axib_rlast   (pcie_byp_rlast),
        .m_axib_rvalid  (pcie_byp_rvalid),
        .m_axib_rready  (pcie_byp_rready)
    );

    localparam [31:0] PCIE_BYP_DECERR_ADDR = 32'h4100_0000;

    wire [31:0] pcie_byp_awaddr_lo =
        (|pcie_byp_awaddr[63:32]) ? PCIE_BYP_DECERR_ADDR : pcie_byp_awaddr[31:0];
    wire [31:0] pcie_byp_araddr_lo =
        (|pcie_byp_araddr[63:32]) ? PCIE_BYP_DECERR_ADDR : pcie_byp_araddr[31:0];

    wire       pcie_m_awlock;
    wire [3:0] pcie_m_awcache;
    wire [2:0] pcie_m_awprot;
    wire [3:0] pcie_m_awqos;
    wire       pcie_m_awuser;
    wire       pcie_m_wuser;
    wire       pcie_m_buser = 1'b0;
    wire       pcie_m_arlock;
    wire [3:0] pcie_m_arcache;
    wire [2:0] pcie_m_arprot;
    wire [3:0] pcie_m_arqos;
    wire       pcie_m_aruser;
    wire       pcie_m_ruser = 1'b0;

    axi_async_bridge #(
        .DATA_WIDTH(128),
        .ADDR_WIDTH(32),
        .ID_WIDTH  (4)
    ) u_pcie_bypass_cdc (
        .s_clk(pcie_axi_aclk),
        .s_rst(~pcie_axi_aresetn),
        .s_awid(pcie_byp_awid),
        .s_awaddr(pcie_byp_awaddr_lo),
        .s_awlen(pcie_byp_awlen),
        .s_awsize(pcie_byp_awsize),
        .s_awburst(pcie_byp_awburst),
        .s_awlock(pcie_byp_awlock),
        .s_awcache(pcie_byp_awcache),
        .s_awprot(pcie_byp_awprot),
        .s_awqos(4'b0),
        .s_awuser(1'b0),
        .s_awvalid(pcie_byp_awvalid),
        .s_awready(pcie_byp_awready),
        .s_wdata(pcie_byp_wdata),
        .s_wstrb(pcie_byp_wstrb),
        .s_wlast(pcie_byp_wlast),
        .s_wuser(1'b0),
        .s_wvalid(pcie_byp_wvalid),
        .s_wready(pcie_byp_wready),
        .s_bid(pcie_byp_bid),
        .s_bresp(pcie_byp_bresp),
        .s_buser(),
        .s_bvalid(pcie_byp_bvalid),
        .s_bready(pcie_byp_bready),
        .s_arid(pcie_byp_arid),
        .s_araddr(pcie_byp_araddr_lo),
        .s_arlen(pcie_byp_arlen),
        .s_arsize(pcie_byp_arsize),
        .s_arburst(pcie_byp_arburst),
        .s_arlock(pcie_byp_arlock),
        .s_arcache(pcie_byp_arcache),
        .s_arprot(pcie_byp_arprot),
        .s_arqos(4'b0),
        .s_aruser(1'b0),
        .s_arvalid(pcie_byp_arvalid),
        .s_arready(pcie_byp_arready),
        .s_rid(pcie_byp_rid),
        .s_rdata(pcie_byp_rdata),
        .s_rresp(pcie_byp_rresp),
        .s_rlast(pcie_byp_rlast),
        .s_ruser(),
        .s_rvalid(pcie_byp_rvalid),
        .s_rready(pcie_byp_rready),

        .m_clk(core_clk),
        .m_rst(core_rst),
        .m_awid(xdma_awid),
        .m_awaddr(xdma_awaddr),
        .m_awlen(xdma_awlen),
        .m_awsize(xdma_awsize),
        .m_awburst(xdma_awburst),
        .m_awlock(pcie_m_awlock),
        .m_awcache(pcie_m_awcache),
        .m_awprot(pcie_m_awprot),
        .m_awqos(pcie_m_awqos),
        .m_awuser(pcie_m_awuser),
        .m_awvalid(xdma_awvalid),
        .m_awready(xdma_awready),
        .m_wdata(xdma_wdata),
        .m_wstrb(xdma_wstrb),
        .m_wlast(xdma_wlast),
        .m_wuser(pcie_m_wuser),
        .m_wvalid(xdma_wvalid),
        .m_wready(xdma_wready),
        .m_bid(xdma_bid),
        .m_bresp(xdma_bresp),
        .m_buser(pcie_m_buser),
        .m_bvalid(xdma_bvalid),
        .m_bready(xdma_bready),
        .m_arid(xdma_arid),
        .m_araddr(xdma_araddr),
        .m_arlen(xdma_arlen),
        .m_arsize(xdma_arsize),
        .m_arburst(xdma_arburst),
        .m_arlock(pcie_m_arlock),
        .m_arcache(pcie_m_arcache),
        .m_arprot(pcie_m_arprot),
        .m_arqos(pcie_m_arqos),
        .m_aruser(pcie_m_aruser),
        .m_arvalid(xdma_arvalid),
        .m_arready(xdma_arready),
        .m_rid(xdma_rid),
        .m_rdata(xdma_rdata),
        .m_rresp(xdma_rresp),
        .m_rlast(xdma_rlast),
        .m_ruser(pcie_m_ruser),
        .m_rvalid(xdma_rvalid),
        .m_rready(xdma_rready)
    );

    /* verilator lint_off UNUSED */
    wire _unused_pcie_xdma = &{1'b0, pcie_user_lnk_up,
                               pcie_m_awlock, pcie_m_awcache,
                               pcie_m_awprot, pcie_m_awqos,
                               pcie_m_awuser, pcie_m_wuser,
                               pcie_m_arlock, pcie_m_arcache,
                               pcie_m_arprot, pcie_m_arqos,
                               pcie_m_aruser, 1'b0};
    /* verilator lint_on UNUSED */
`elsif JTAG_AXI_ENABLE
    wire [0:0]  jtag_axi_awid;
    wire [31:0] jtag_axi_awaddr;
    wire [7:0]  jtag_axi_awlen;
    wire [2:0]  jtag_axi_awsize;
    wire [1:0]  jtag_axi_awburst;
    wire        jtag_axi_awlock;
    wire [3:0]  jtag_axi_awcache;
    wire [2:0]  jtag_axi_awprot;
    wire [3:0]  jtag_axi_awqos;
    wire        jtag_axi_awvalid;
    wire        jtag_axi_awready;
    wire [31:0] jtag_axi_wdata;
    wire [3:0]  jtag_axi_wstrb;
    wire        jtag_axi_wlast;
    wire        jtag_axi_wvalid;
    wire        jtag_axi_wready;
    wire [0:0]  jtag_axi_bid = 1'b0;
    wire [1:0]  jtag_axi_bresp;
    wire        jtag_axi_bvalid;
    wire        jtag_axi_bready;
    wire [0:0]  jtag_axi_arid;
    wire [31:0] jtag_axi_araddr;
    wire [7:0]  jtag_axi_arlen;
    wire [2:0]  jtag_axi_arsize;
    wire [1:0]  jtag_axi_arburst;
    wire        jtag_axi_arlock;
    wire [3:0]  jtag_axi_arcache;
    wire [2:0]  jtag_axi_arprot;
    wire [3:0]  jtag_axi_arqos;
    wire        jtag_axi_arvalid;
    wire        jtag_axi_arready;
    wire [0:0]  jtag_axi_rid = 1'b0;
    wire [31:0] jtag_axi_rdata;
    wire [1:0]  jtag_axi_rresp;
    wire        jtag_axi_rlast;
    wire        jtag_axi_rvalid;
    wire        jtag_axi_rready;

    debug_jtag_axi u_debug_jtag_axi (
        .aclk(core_clk),
        .aresetn(~core_rst),
        .m_axi_awid(jtag_axi_awid),
        .m_axi_awaddr(jtag_axi_awaddr),
        .m_axi_awlen(jtag_axi_awlen),
        .m_axi_awsize(jtag_axi_awsize),
        .m_axi_awburst(jtag_axi_awburst),
        .m_axi_awlock(jtag_axi_awlock),
        .m_axi_awcache(jtag_axi_awcache),
        .m_axi_awprot(jtag_axi_awprot),
        .m_axi_awqos(jtag_axi_awqos),
        .m_axi_awvalid(jtag_axi_awvalid),
        .m_axi_awready(jtag_axi_awready),
        .m_axi_wdata(jtag_axi_wdata),
        .m_axi_wstrb(jtag_axi_wstrb),
        .m_axi_wlast(jtag_axi_wlast),
        .m_axi_wvalid(jtag_axi_wvalid),
        .m_axi_wready(jtag_axi_wready),
        .m_axi_bid(jtag_axi_bid),
        .m_axi_bresp(jtag_axi_bresp),
        .m_axi_bvalid(jtag_axi_bvalid),
        .m_axi_bready(jtag_axi_bready),
        .m_axi_arid(jtag_axi_arid),
        .m_axi_araddr(jtag_axi_araddr),
        .m_axi_arlen(jtag_axi_arlen),
        .m_axi_arsize(jtag_axi_arsize),
        .m_axi_arburst(jtag_axi_arburst),
        .m_axi_arlock(jtag_axi_arlock),
        .m_axi_arcache(jtag_axi_arcache),
        .m_axi_arprot(jtag_axi_arprot),
        .m_axi_arqos(jtag_axi_arqos),
        .m_axi_arvalid(jtag_axi_arvalid),
        .m_axi_arready(jtag_axi_arready),
        .m_axi_rid(jtag_axi_rid),
        .m_axi_rdata(jtag_axi_rdata),
        .m_axi_rresp(jtag_axi_rresp),
        .m_axi_rlast(jtag_axi_rlast),
        .m_axi_rvalid(jtag_axi_rvalid),
        .m_axi_rready(jtag_axi_rready)
    );

    // ══════════════════════════════════════════════════════════════════
    // THE DEBUG BUS (work item 1+2, docs/bus_debug_split_plan.md)
    // ══════════════════════════════════════════════════════════════════
    // The single JTAG-AXI bridge above lands on `axi_dbg_bus`, not on the
    // SoC crossbar.  The debug bus serves the 0x5090_0000 window LOCALLY
    // (jtagdbg_* -> CPU dbg_axi / eth debug regs, see
    // fpga_top_peripherals.vh) and MASTERS the SoC bus for everything else
    // (jtagsys_* -> u_jtag_n2w -> xbar M1, unchanged).
    //
    // There is still exactly ONE bridge and ONE hw_axi core; the host
    // address space is unchanged.  This is not the dual-BSCAN arrangement
    // reverted in 330ad925 -- that revert removed a SECOND debug_jtag_axi
    // IP instance, which this does not add.
    //
    // Two measured defects motivate it; both are in axi_dbg_bus.v's header
    // with their evidence.  The short form:
    //   * p150 (0xE934ECC7): a wedged peripheral killed EVERY JTAG read,
    //     because the debug window ran through the shared fabric and
    //     through peripheral_bus itself.
    //   * 2026-09-12 vio_boot_diag: a JTAG `reset` IS a write to
    //     DBG_CONTROL, i.e. a write through S1.  The reset swallowed its
    //     own BRESP and stranded the S1 slot (xbar_s1_slot_busy 0 -> 1)
    //     with the poison latches never firing (slv_poisoned = 0x00 in
    //     both states).  The reset request must not travel the fabric it
    //     resets.
    //
    // `rst` is core_rst on purpose -- see axi_dbg_bus.v "RESET DOMAIN".
    wire [31:0] jtagsys_awaddr, jtagsys_araddr;
    wire [7:0]  jtagsys_awlen,  jtagsys_arlen;
    wire        jtagsys_awvalid, jtagsys_awready;
    wire [31:0] jtagsys_wdata;   wire [3:0] jtagsys_wstrb;
    wire        jtagsys_wlast,  jtagsys_wvalid, jtagsys_wready;
    wire [1:0]  jtagsys_bresp;   wire jtagsys_bvalid, jtagsys_bready;
    wire        jtagsys_arvalid, jtagsys_arready;
    wire [31:0] jtagsys_rdata;   wire [1:0] jtagsys_rresp;
    wire        jtagsys_rlast,  jtagsys_rvalid, jtagsys_rready;

    wire [31:0] jtagdbg_awaddr, jtagdbg_araddr;
    wire [7:0]  jtagdbg_awlen,  jtagdbg_arlen;
    wire        jtagdbg_awvalid, jtagdbg_awready;
    wire [31:0] jtagdbg_wdata;   wire [3:0] jtagdbg_wstrb;
    wire        jtagdbg_wlast,  jtagdbg_wvalid, jtagdbg_wready;
    wire [1:0]  jtagdbg_bresp;   wire jtagdbg_bvalid, jtagdbg_bready;
    wire        jtagdbg_arvalid, jtagdbg_arready;
    wire [31:0] jtagdbg_rdata;   wire [1:0] jtagdbg_rresp;
    wire        jtagdbg_rlast,  jtagdbg_rvalid, jtagdbg_rready;
    wire        jtagdbg_wr_active, jtagdbg_rd_active;

    axi_dbg_bus #(
        .ID_WIDTH(4), .ADDR_WIDTH(32), .DATA_WIDTH(32),
        .DBG_BASE(32'h5090_0000), .DBG_MASK(32'hFFF0_0000)
    ) u_dbg_bus (
        .clk(core_clk), .rst(core_rst),

        .s_awid(4'd0), .s_awaddr(jtag_axi_awaddr), .s_awlen(jtag_axi_awlen),
        .s_awsize(3'd2), .s_awburst(2'b01),
        .s_awvalid(jtag_axi_awvalid), .s_awready(jtag_axi_awready),
        .s_wdata(jtag_axi_wdata), .s_wstrb(jtag_axi_wstrb),
        .s_wlast(jtag_axi_wlast), .s_wvalid(jtag_axi_wvalid),
        .s_wready(jtag_axi_wready),
        .s_bid(), .s_bresp(jtag_axi_bresp), .s_bvalid(jtag_axi_bvalid),
        .s_bready(jtag_axi_bready),
        .s_arid(4'd0), .s_araddr(jtag_axi_araddr), .s_arlen(jtag_axi_arlen),
        .s_arsize(3'd2), .s_arburst(2'b01),
        .s_arvalid(jtag_axi_arvalid), .s_arready(jtag_axi_arready),
        .s_rid(), .s_rdata(jtag_axi_rdata), .s_rresp(jtag_axi_rresp),
        .s_rlast(jtag_axi_rlast), .s_rvalid(jtag_axi_rvalid),
        .s_rready(jtag_axi_rready),

        // ── Master port onto the SoC bus ──────────────────────────────
        .m_awid(), .m_awaddr(jtagsys_awaddr), .m_awlen(jtagsys_awlen),
        .m_awsize(), .m_awburst(),
        .m_awvalid(jtagsys_awvalid), .m_awready(jtagsys_awready),
        .m_wdata(jtagsys_wdata), .m_wstrb(jtagsys_wstrb),
        .m_wlast(jtagsys_wlast), .m_wvalid(jtagsys_wvalid),
        .m_wready(jtagsys_wready),
        .m_bid(4'd0), .m_bresp(jtagsys_bresp), .m_bvalid(jtagsys_bvalid),
        .m_bready(jtagsys_bready),
        .m_arid(), .m_araddr(jtagsys_araddr), .m_arlen(jtagsys_arlen),
        .m_arsize(), .m_arburst(),
        .m_arvalid(jtagsys_arvalid), .m_arready(jtagsys_arready),
        .m_rid(4'd0), .m_rdata(jtagsys_rdata), .m_rresp(jtagsys_rresp),
        .m_rlast(jtagsys_rlast), .m_rvalid(jtagsys_rvalid),
        .m_rready(jtagsys_rready),

        // ── Debug-bus local slave port ────────────────────────────────
        .d_awid(), .d_awaddr(jtagdbg_awaddr), .d_awlen(jtagdbg_awlen),
        .d_awsize(), .d_awburst(),
        .d_awvalid(jtagdbg_awvalid), .d_awready(jtagdbg_awready),
        .d_wdata(jtagdbg_wdata), .d_wstrb(jtagdbg_wstrb),
        .d_wlast(jtagdbg_wlast), .d_wvalid(jtagdbg_wvalid),
        .d_wready(jtagdbg_wready),
        .d_bid(4'd0), .d_bresp(jtagdbg_bresp), .d_bvalid(jtagdbg_bvalid),
        .d_bready(jtagdbg_bready),
        .d_arid(), .d_araddr(jtagdbg_araddr), .d_arlen(jtagdbg_arlen),
        .d_arsize(), .d_arburst(),
        .d_arvalid(jtagdbg_arvalid), .d_arready(jtagdbg_arready),
        .d_rid(4'd0), .d_rdata(jtagdbg_rdata), .d_rresp(jtagdbg_rresp),
        .d_rlast(jtagdbg_rlast), .d_rvalid(jtagdbg_rvalid),
        .d_rready(jtagdbg_rready),

        .dbg_wr_active(jtagdbg_wr_active),
        .dbg_rd_active(jtagdbg_rd_active)
    );

    /* verilator lint_off UNUSED */
    wire _unused_dbg_bus = &{1'b0, jtagdbg_awlen, jtagdbg_arlen,
                             jtagdbg_wlast, jtagdbg_wr_active,
                             jtagdbg_rd_active, 1'b0};
    /* verilator lint_on UNUSED */

    // The first-light DDR bridge accepts single 32-bit word transactions
    // today.  Use Vivado hw_axi single-word writes for ROM patch/load; full
    // JTAG burst loading waits on the broader DDR burst-support task.
    axi_narrow_to_wide #(
        .ID_WIDTH(4),
        .ID_TAG  (4'd1)
    ) u_jtag_n2w (
        .clk(core_clk), .rst(core_rst),

        .n_awaddr (jtagsys_awaddr ),
        .n_awprot (jtag_axi_awprot ),
        // Connect the JTAG-AXI master's REAL length outputs rather than
        // tying them off: the host does carry awlen/arlen, and a constant
        // here would silently truncate any multi-beat access it issues.
        // Explicitly connected because production lint runs
        // -Wno-PINMISSING and would otherwise leave them undriven mute.
        .n_awlen  (jtagsys_awlen  ),
        .n_arlen  (jtagsys_arlen  ),
        .n_awvalid(jtagsys_awvalid),
        .n_awready(jtagsys_awready),
        .n_wdata  (jtagsys_wdata  ),
        .n_wstrb  (jtagsys_wstrb  ),
        .n_wlast  (jtagsys_wlast  ),
        .n_wvalid (jtagsys_wvalid ),
        .n_wready (jtagsys_wready ),
        .n_bresp  (jtagsys_bresp  ),
        .n_bvalid (jtagsys_bvalid ),
        .n_bready (jtagsys_bready ),
        .n_araddr (jtagsys_araddr ),
        .n_arprot (jtag_axi_arprot ),
        .n_arvalid(jtagsys_arvalid),
        .n_arready(jtagsys_arready),
        .n_rdata  (jtagsys_rdata  ),
        .n_rresp  (jtagsys_rresp  ),
        .n_rlast  (jtagsys_rlast  ),
        .n_rvalid (jtagsys_rvalid ),
        .n_rready (jtagsys_rready ),

        .w_awid   (xdma_awid   ), .w_awaddr (xdma_awaddr ),
        .w_awlen  (xdma_awlen  ), .w_awsize (xdma_awsize ),
        .w_awburst(xdma_awburst), .w_awvalid(xdma_awvalid),
        .w_awready(xdma_awready),
        .w_wdata  (xdma_wdata  ), .w_wstrb  (xdma_wstrb  ),
        .w_wlast  (xdma_wlast  ), .w_wvalid (xdma_wvalid ),
        .w_wready (xdma_wready ),
        .w_bid    (xdma_bid    ), .w_bresp  (xdma_bresp  ),
        .w_bvalid (xdma_bvalid ), .w_bready (xdma_bready ),
        .w_arid   (xdma_arid   ), .w_araddr (xdma_araddr ),
        .w_arlen  (xdma_arlen  ), .w_arsize (xdma_arsize ),
        .w_arburst(xdma_arburst), .w_arvalid(xdma_arvalid),
        .w_arready(xdma_arready),
        .w_rid    (xdma_rid    ), .w_rdata  (xdma_rdata  ),
        .w_rresp  (xdma_rresp  ), .w_rlast  (xdma_rlast  ),
        .w_rvalid (xdma_rvalid ), .w_rready (xdma_rready )
    );

    /* verilator lint_off UNUSED */
    wire _unused_jtag_axi = &{1'b0, jtag_axi_awid, jtag_axi_awlen,
                              jtag_axi_awsize, jtag_axi_awburst,
                              jtag_axi_awlock, jtag_axi_awcache,
                              jtag_axi_awqos,
                              jtag_axi_arid, jtag_axi_arlen,
                              jtag_axi_arsize, jtag_axi_arburst,
                              jtag_axi_arlock, jtag_axi_arcache,
                              jtag_axi_arqos,
                              jtag_axi_rlast, 1'b0};
    /* verilator lint_on UNUSED */
`else
    assign xdma_awid    = 4'b0;
    assign xdma_awaddr  = 32'b0;
    assign xdma_awlen   = 8'b0;
    assign xdma_awsize  = 3'b0;
    assign xdma_awburst = 2'b0;
    assign xdma_awvalid = 1'b0;
    assign xdma_wdata   = 128'b0;
    assign xdma_wstrb   = 16'b0;
    assign xdma_wlast   = 1'b0;
    assign xdma_wvalid  = 1'b0;
    assign xdma_bready  = 1'b1;
    assign xdma_arid    = 4'b0;
    assign xdma_araddr  = 32'b0;
    assign xdma_arlen   = 8'b0;
    assign xdma_arsize  = 3'b0;
    assign xdma_arburst = 2'b0;
    assign xdma_arvalid = 1'b0;
    assign xdma_rready  = 1'b1;
`endif

