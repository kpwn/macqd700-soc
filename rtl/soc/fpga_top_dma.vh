// rtl/soc/fpga_top_dma.vh — included from rtl/soc/fpga_top.v
//
// Shared DMA engine and the legacy xbar S2 control-window terminator.  The
// peripheral-neutral dma_engine occupies M3 in normal builds; the S2 window
// remains owned by vhdd_ctrl and is not the DMA client's programming model.
//
// History
// ───────
// 2026-07-16 master-count reduction: `dma_ctrl`'s 64-bit AXI4 master had
// zero live consumers (nothing in the design ever triggered a transfer),
// so its xbar M4 seat and the 64->128 `axi_n64_to_wide` widening bridge
// beneath it were removed and the master-side response inputs were tied
// to a permanently-idle AXI slave.  That left `dma_ctrl` instantiated
// purely for its config-register file: a register bank whose registers
// could be written and read back but could never cause a byte to move.
//
// 2026-08-01 area reduction: that residual instance is now gone too.  On
// the last CPU-bearing routed build (`build/vivado/reports/
// utilization_route.rpt`, 2026-07-31) `u_dma_ctrl` measured 2,024 LUTs /
// 814 FFs of pure dead weight in a design whose binding constraint is
// LUTs.  It is replaced below by `axil_null_slave` (rtl/soc/
// axil_null_slave.v, a handful of LUTs) which accepts every write
// (BRESP=OKAY) and returns 0 on every read (RRESP=OKAY), so an access to
// the DMA window still *completes* rather than wedging the fabric.
//
// 2026-08-03 vHDD control block: the terminator is itself replaced by
// `vhdd_ctrl` (rtl/soc/vhdd_ctrl.v), a small register file for the two
// SCSI volumes (SD-backed and DDR-backed RAM disk).  It PRESERVES the
// terminator behaviour for every offset it does not decode — reads 0,
// writes accepted with OKAY — so the "a stray access still COMPLETES"
// safety claim above still holds; it just also answers eight real
// registers at the bottom of the window.  Reusing this dead window costs
// zero changes to the crossbar's slave count, slot numbering or decode.
//
// `rtl/soc/dma_ctrl.v` itself is deliberately NOT deleted — it is
// earmarked for reuse as the NVMe-as-SCSI transfer engine (see
// `docs/dma_ctrl.md`) and is still fully unit-tested standalone by
// `make tb-dma-ctrl`.  Re-instantiating it here (plus the widening
// bridge, see git history) is the path back.
//
// Behavioural delta versus the previous build, for the record: a CPU or
// JTAG write to 0x5010_xxxx used to land in a dma_ctrl descriptor
// register and read back the stored value; it now reads back 0.  Nothing
// in the ROM, in Mac OS, in `tools/`, or in any testbench targets that
// window (the only references are `tb/tb_axi_xbar.cpp`, which drives its
// own BFM slave on S2 and never sees this file, and the address-map
// docs).  The DMA IRQ line into `irq_agg` (`dma_irq_w`, level 6) is now
// tied low, which is what it already was in practice.
//
// Do not add module/endmodule wrappers or nettype directives.

    // ═══════════════════════════════════════════════════════════════════
    // DMA config window (xbar S2) — AXI-Lite null responder
    // ═══════════════════════════════════════════════════════════════════
    // Driven by `u_dma_s2_bridge` in fpga_top_xbar.vh, which converts the
    // 128-bit AXI4 S2 port down to these AXI-Lite signals.  That bridge
    // presents AW and W independently (it tracks `wr_aw_done` /
    // `wr_w_done` separately), so this responder must too — it cannot
    // require the two handshakes in the same cycle.
    wire [19:0]  dma_cfg_awaddr;
    wire         dma_cfg_awvalid;
    wire         dma_cfg_awready;
    wire [31:0]  dma_cfg_wdata;
    wire [3:0]   dma_cfg_wstrb;
    wire         dma_cfg_wvalid;
    wire         dma_cfg_wready;
    wire [1:0]   dma_cfg_bresp;
    wire         dma_cfg_bvalid;
    wire         dma_cfg_bready;
    wire [19:0]  dma_cfg_araddr;
    wire         dma_cfg_arvalid;
    wire         dma_cfg_arready;
    wire [31:0]  dma_cfg_rdata;
    wire [1:0]   dma_cfg_rresp;
    wire         dma_cfg_rvalid;
    wire         dma_cfg_rready;

    // Same reset bank the retired dma_ctrl used (bank bit [5], DMA / SD
    // region) so a JTAG/btn cold-boot still clears any parked response.
    // The terminator is a real module rather than inline logic so it can
    // be unit-tested — `make tb-axil-null-slave`.  That matters here: the
    // whole safety claim of removing dma_ctrl is "a stray access still
    // COMPLETES", and this SoC has a documented history of wild accesses
    // landing in the 0x5xxx_xxxx I/O region
    // (docs/diag-bus-fault-51001c00.md).
    // ── vHDD control/status registers (also the window terminator) ────
    // Cross-file seam: these wires are DECLARED here because this file is
    // `included before fpga_top_peripherals.vh, and driven/consumed there.
    // Continuous assignments do not care about textual order; a reg
    // referenced before its declaration would not compile.
    wire [1:0]  vhdd_dev_en_core;      // out: [0]=SD live, [1]=RAM disk live
    wire        vhdd_sd_wprot_core;    // out: CTRL[2], SD volume locked
    wire [31:0] vhdd_rd_blocks_core;   // out: RAM-disk size in 512 B blocks
    wire [31:0] vhdd_sd_blocks_core;   // in : SD volume size (peripherals.vh)
    wire        vhdd_rd_busy_core;     // in : synchronised from pb_clk
    wire        vhdd_rd_error_core;    // in : synchronised from pb_clk
    wire [3:0]  vhdd_rd_state_core;    // in : synchronised from pb_clk
    wire [15:0] vhdd_rd_wdog_core;     // in : synchronised from pb_clk

    wire [47:0] net_vhdd_our_mac, net_vhdd_dst_mac;
    wire [31:0] net_vhdd_our_ip,  net_vhdd_dst_ip;
    wire [15:0] net_vhdd_our_port, net_vhdd_dst_port;
    // Explicit sink, not a dangling wire: the endpoint CSR ships before the
    // provider that consumes it, and an unread output is exactly the shape
    // that hid the core_done_pint break.  Delete this when vhdd_net is
    // instantiated and these feed it instead.
`ifndef ENABLE_NET_VHDD
    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused_net_vhdd_cfg = &{1'b0, net_vhdd_our_mac, net_vhdd_dst_mac,
                                  net_vhdd_our_ip, net_vhdd_dst_ip,
                                  net_vhdd_our_port, net_vhdd_dst_port};
    /* verilator lint_on UNUSEDSIGNAL */
`endif

    vhdd_ctrl #(
        .ADDR_WIDTH     (20),
        // RD_APERTURE / RD_MAX_BLOCKS keep the module's own defaults.  The
        // DDR-backed RAM-disk volume they described was removed on
        // 2026-09-10 (owner directive, "vhdd ddr should be killed") together
        // with the 0x7000_0000 aperture and its xbar decode, so there is no
        // longer a system constant to bind them to; the RD_* CSR fields are
        // retained only so the provider-B slot stays describable for a future
        // tenant (see docs/net_vhdd_design.md).
        .RD_BLOCKS_RESET(32'd65536),      // 32 MiB default volume
        // SD ONLY at power-on.  Was 2'b11 ("both volumes live"), which put an
        // unformatted 32 MB target at SCSI ID 1 in front of the ROM's boot
        // scan on every cold boot -- a dual-target configuration with NO full
        // ROM-boot sim behind it.  Provider B is a deliberately-invoked
        // tool, not a device that should exist unless asked for.
        //
        // Enable it from JTAG with `vhdd-enable ram 1`.  That now STICKS
        // across a JTAG reboot: ctrl_q runs on cfg_rst (core_rst_bank[5]),
        // not on this module's soc_full_rst, so the enable survives the
        // debug-full-reset used to reboot the Mac and is cleared only by a
        // power cycle or a real platform reset.  See the ctrl_q block
        // comment in vhdd_ctrl.v.
        // bit0 = SD volume, bit1 = RAM disk, bit2 = SD write-protect
        .CTRL_RESET     (3'b001)
    ) u_vhdd_ctrl (
        .clk    (core_clk), .rst    (soc_full_rst_bank[5]),
        .cfg_rst(core_rst_bank[5]),
        .awaddr (dma_cfg_awaddr ), .awvalid(dma_cfg_awvalid),
        .awready(dma_cfg_awready),
        .wdata  (dma_cfg_wdata  ), .wstrb  (dma_cfg_wstrb  ),
        .wvalid (dma_cfg_wvalid ), .wready (dma_cfg_wready ),
        .bresp  (dma_cfg_bresp  ), .bvalid (dma_cfg_bvalid ),
        .bready (dma_cfg_bready ),
        .araddr (dma_cfg_araddr ), .arvalid(dma_cfg_arvalid),
        .arready(dma_cfg_arready),
        .rdata  (dma_cfg_rdata  ), .rresp  (dma_cfg_rresp  ),
        .rvalid (dma_cfg_rvalid ), .rready (dma_cfg_rready ),
        // net-vHDD endpoint.  Declared and carried now so the CSR is
        // configurable ahead of the provider; vhdd_net is not instantiated
        // yet, so these currently terminate in the explicit sink below
        // rather than dangling.
        .net_our_mac  (net_vhdd_our_mac),
        .net_our_ip   (net_vhdd_our_ip),
        .net_our_port (net_vhdd_our_port),
        .net_dst_mac  (net_vhdd_dst_mac),
        .net_dst_ip   (net_vhdd_dst_ip),
        .net_dst_port (net_vhdd_dst_port),
        .dev_en       (vhdd_dev_en_core),
        .sd_wprot     (vhdd_sd_wprot_core),
        .rd_num_lbas  (vhdd_rd_blocks_core),
        .sd_num_lbas  (vhdd_sd_blocks_core),
        .rd_busy      (vhdd_rd_busy_core),
        .rd_error     (vhdd_rd_error_core),
        .rd_state     (vhdd_rd_state_core),
        .rd_wdog_fires(vhdd_rd_wdog_core),
        .trace_rd_addr(scsi_trace_rd_addr),
        .trace_rd_data(scsi_trace_rd_data),
        .trace_wrptr  (scsi_trace_wrptr),
        .trace_wrapped(scsi_trace_wrapped),
        .trace_frozen (scsi_trace_frozen),
        .trace_freeze (scsi_trace_freeze),
        .trace_clear  (scsi_trace_clear)
    );

    // ── 53C96 trace-ring cross-file wires ─────────────────────────────
    // The ring itself is instantiated in fpga_top_peripherals.vh, next to
    // the scsi.v it snoops (that is where the pb_scsi_* port wires live).
    // Declared here per this file set's convention: cross-file wires are
    // declared in the earlier include that first uses them, and
    // fpga_top_dma.vh is included before fpga_top_peripherals.vh.
    wire [11:0] scsi_trace_rd_addr;
    wire [31:0] scsi_trace_rd_data;
    wire [11:0] scsi_trace_wrptr;
    wire        scsi_trace_wrapped;
    wire        scsi_trace_frozen;
    wire        scsi_trace_freeze;
    wire        scsi_trace_clear;

    // ═══════════════════════════════════════════════════════════════════
    // xbar M3 — the shared DMA engine's AXI master
    // ═══════════════════════════════════════════════════════════════════
    // Declared here (before fpga_top_xbar.vh's u_xbar instance) and driven
    // by u_dma_engine below.  M3 used to be shared with the DDR-backed
    // RAM-disk volume under a compile-time switch; that volume was removed
    // on 2026-09-10 (owner directive), so the seat is now unconditionally
    // the DMA engine's.
    wire [3:0]   m3_awid;
    wire [31:0]  m3_awaddr;
    wire [7:0]   m3_awlen;
    wire [2:0]   m3_awsize;
    wire [1:0]   m3_awburst;
    wire         m3_awvalid;
    wire         m3_awready;
    wire [127:0] m3_wdata;
    wire [15:0]  m3_wstrb;
    wire         m3_wlast;
    wire         m3_wvalid;
    wire         m3_wready;
    wire [3:0]   m3_bid;
    wire [1:0]   m3_bresp;
    wire         m3_bvalid;
    wire         m3_bready;
    wire [3:0]   m3_arid;
    wire [31:0]  m3_araddr;
    wire [7:0]   m3_arlen;
    wire [2:0]   m3_arsize;
    wire [1:0]   m3_arburst;
    wire         m3_arvalid;
    wire         m3_arready;
    wire [3:0]   m3_rid;
    wire [127:0] m3_rdata;
    wire [1:0]   m3_rresp;
    wire         m3_rlast;
    wire         m3_rvalid;
    wire         m3_rready;

    // ── Generic shared DMA engine on xbar M3 ──────────────────────────
    // Four packed client lanes accept byte-granular read/write requests.
    // Lane 0 is SONIC when its real packet endpoint is selected; the remaining
    // lanes stay available for storage and other peripherals.
    wire [127:0] dma_client_req_addr;
    wire [27:0] dma_client_req_len;
    wire [2047:0] dma_client_req_wdata;
    wire [31:0] dma_client_req_tag;
    wire [3:0] dma_client_req_write, dma_client_req_valid;
    wire [3:0] dma_client_req_ready;
    wire [2047:0] dma_client_rsp_rdata;
    wire [31:0] dma_client_rsp_tag;
    wire [27:0] dma_client_rsp_len;
    wire [7:0] dma_client_rsp_status;
    wire [3:0] dma_client_rsp_write;
    wire [3:0] dma_client_rsp_valid;

    dma_engine #(
        .N_CLIENTS (4),
        .ADDR_WIDTH(32),
        .TAG_WIDTH (8),
        .DATA_WIDTH(128),
        .ID_WIDTH  (4),
        // Back to 16.  This was raised to 32 as a WORKAROUND for a
        // 1088-byte (17*64) receive ceiling, on the reasoning that 17 is
        // QUEUE_DEPTH+1-in-flight and so the first write to meet
        // back-pressure.  That was the right pointer but the wrong repair:
        // the real defect was in q700_sonic_rx, whose BRAM read pipeline
        // free-ran during a stall and skipped a chunk.  With that fixed the
        // engine is stall-safe at any depth, so the extra BRAM buys nothing.
        .QUEUE_DEPTH(16),
        // Mac RAM lanes carry the lowest-address byte in bits [31:24] of each
        // 32-bit group (see rtl/soc/vram_cpu_byteswap.v).  Permute so SONIC
        // descriptors and frame payload reach the engine's clients in guest
        // byte order.
        .BYTE_SWAP32(1)
    ) u_dma_engine (
        .clk(core_clk), .rst(core_rst_bank[5]),
        .req_addr(dma_client_req_addr), .req_len(dma_client_req_len),
        .req_wdata(dma_client_req_wdata), .req_tag(dma_client_req_tag),
        .req_write(dma_client_req_write), .req_valid(dma_client_req_valid),
        .req_ready(dma_client_req_ready),
        .rsp_rdata(dma_client_rsp_rdata), .rsp_tag(dma_client_rsp_tag),
        .rsp_len(dma_client_rsp_len), .rsp_status(dma_client_rsp_status),
        .rsp_write(dma_client_rsp_write), .rsp_valid(dma_client_rsp_valid),
        .rsp_ready(4'hf),
        .m_awid(m3_awid), .m_awaddr(m3_awaddr), .m_awlen(m3_awlen),
        .m_awsize(m3_awsize), .m_awburst(m3_awburst),
        .m_awvalid(m3_awvalid), .m_awready(m3_awready),
        .m_wdata(m3_wdata), .m_wstrb(m3_wstrb), .m_wlast(m3_wlast),
        .m_wvalid(m3_wvalid), .m_wready(m3_wready),
        .m_bid(m3_bid), .m_bresp(m3_bresp), .m_bvalid(m3_bvalid),
        .m_bready(m3_bready),
        .m_arid(m3_arid), .m_araddr(m3_araddr), .m_arlen(m3_arlen),
        .m_arsize(m3_arsize), .m_arburst(m3_arburst),
        .m_arvalid(m3_arvalid), .m_arready(m3_arready),
        .m_rid(m3_rid), .m_rdata(m3_rdata), .m_rresp(m3_rresp),
        .m_rlast(m3_rlast), .m_rvalid(m3_rvalid), .m_rready(m3_rready)
    );

`ifdef ETH_ENABLE
    generate if (ETH_ICMP_RESPONDER == 0) begin : g_sonic_dma_client
        wire core_cmd_valid, core_cmd_ready;
        wire [15:0] core_cmd_dcr, core_cmd_utda, core_cmd_ctda;
        wire core_done_valid, core_done_ready, core_done_error;
        wire [15:0] core_done_ctda, core_done_tcr, core_done_tps, core_done_tfc;
        wire [31:0] lane_addr;
        wire [6:0] lane_len;
        wire [511:0] lane_wdata;
        wire [7:0] lane_tag;
        wire lane_write, lane_valid;
        wire rx_enable_core,rx_cfg_valid_core,rx_cfg_ready_core;
        wire [2:0] rx_cfg_op_core;
        wire [15:0] rx_dcr,rx_rcr,rx_urda,rx_crda,rx_urra,rx_rsa,rx_rea,rx_rrp,rx_rwp,rx_eobc,rx_rsc,rx_llfa,rx_cdp,rx_cdc;
        wire rx_done_valid_core,rx_done_ready_core,rx_done_error_core;
        wire [15:0] rx_done_rcr,rx_done_crda,rx_done_crba0,rx_done_crba1,rx_done_rbwc0,rx_done_rbwc1;
        wire [15:0] rx_done_rrp,rx_done_rsc,rx_done_llfa,rx_done_trba0,rx_done_trba1,rx_done_tbwc0,rx_done_tbwc1,rx_done_isr;
        wire [15:0] rx_done_cdp,rx_done_cdc,rx_done_ce;
        wire [31:0] rx_lane_addr; wire [6:0] rx_lane_len; wire [511:0] rx_lane_wdata;
        wire [7:0] rx_lane_tag; wire rx_lane_write,rx_lane_valid;
        // core_done_pint and the live RCR/CAM taps used to be declared inside
        // `ifdef ETH_DEBUG_ENABLE, but they are consumed unconditionally: the
        // q700_sonic_cdc/q700_sonic_tx instances below wire core_done_pint, and
        // sonic_loopback / sonic_rx_drain -- the hardware-proven RCR_LB loopback
        // and receiver-disabled drain -- are derived from the live RCR here.  A
        // permitted ETH_ENABLE=1 ETH_ICMP_RESPONDER=0 ETH_DEBUG_ENABLE=0 build
        // therefore failed elaboration on core_done_pint under
        // `default_nettype none, and would have left loopback and drain with no
        // driver at all.  Telemetry is optional; these are not.
        wire        core_done_pint;
        wire        sonic_tx_halt;   // CR_HTX on core_clk, out of u_q700_sonic_cdc
        wire [15:0] rx_dbg_rcr, rx_dbg_cam_enable;
        wire [47:0] rx_dbg_cam_entry;
        wire [3:0]  eth_cam_index;
`ifndef ETH_DEBUG_ENABLE
        // eth_debug_regs drives this, and it only exists under
        // ETH_DEBUG_ENABLE -- but u_q700_sonic_rx consumes it unconditionally.
        // Tie it off here rather than leaving it floating: that asymmetry
        // between declaration and driver is the same shape as the
        // core_done_pint break noted above.
        assign eth_cam_index = 4'd0;
`endif
        // RCR_LB = 0x0600 (dp83932c.h).  Non-zero selects loopback: the
        // transmitted frame must come back through the receive path and must
        // NOT reach the wire.  Every Mac driver runs this as an init
        // self-test, which is why EtherTalk/MacTCP failed while EtherTrace
        // (which skips the test) worked.
        assign sonic_loopback = (rx_dbg_rcr & 16'h0600) != 16'h0000;
        // Discard wire traffic while the receiver is disabled, so the first
        // CR_RXEN does not inherit a FIFO full of frames buffered since boot.
        assign sonic_rx_drain = !rx_enable_core;
        wire eth_promisc_enable;
`ifdef ETH_DEBUG_ENABLE
        wire [3:0] tx_dbg_state;
        wire [4:0] rx_dbg_state;
        wire [31:0] tx_dbg_descriptor_addr, rx_dbg_descriptor_addr;
        wire [15:0] rx_dbg_frame_len;
`else
        assign eth_promisc_enable = 1'b0;
`endif

        q700_sonic_cdc u_q700_sonic_cdc (
            .pb_halt(sonic_tx_halt_pb), .core_halt(sonic_tx_halt),
            .pb_clk(pb_clk), .pb_rst(pb_full_rst_bank[3]),
            .pb_cmd_valid(sonic_tx_cmd_valid_pb), .pb_cmd_ready(sonic_tx_cmd_ready_pb),
            .pb_cmd_dcr(sonic_tx_cmd_dcr_pb), .pb_cmd_utda(sonic_tx_cmd_utda_pb),
            .pb_cmd_ctda(sonic_tx_cmd_ctda_pb),
            .pb_done_valid(sonic_tx_done_valid_pb), .pb_done_ready(sonic_tx_done_ready_pb),
            .pb_done_error(sonic_tx_done_error_pb), .pb_done_pint(sonic_tx_done_pint_pb),
            .pb_done_ctda(sonic_tx_done_ctda_pb),
            .pb_done_tcr(sonic_tx_done_tcr_pb), .pb_done_tps(sonic_tx_done_tps_pb),
            .pb_done_tfc(sonic_tx_done_tfc_pb),
            .core_clk(core_clk), .core_rst(core_rst_bank[5]),
            .core_cmd_valid(core_cmd_valid), .core_cmd_ready(core_cmd_ready),
            .core_cmd_dcr(core_cmd_dcr), .core_cmd_utda(core_cmd_utda),
            .core_cmd_ctda(core_cmd_ctda),
            .core_done_valid(core_done_valid), .core_done_ready(core_done_ready),
            .core_done_error(core_done_error), .core_done_pint(core_done_pint),
            .core_done_ctda(core_done_ctda),
            .core_done_tcr(core_done_tcr), .core_done_tps(core_done_tps),
            .core_done_tfc(core_done_tfc)
        );
        q700_sonic_tx u_q700_sonic_tx (
            .halt(sonic_tx_halt),
            .clk(core_clk), .rst(core_rst_bank[5]),
            .start_valid(core_cmd_valid), .start_ready(core_cmd_ready),
            .start_dcr(core_cmd_dcr), .start_utda(core_cmd_utda), .start_ctda(core_cmd_ctda),
            .dma_req_addr(lane_addr), .dma_req_len(lane_len), .dma_req_wdata(lane_wdata),
            .dma_req_tag(lane_tag), .dma_req_write(lane_write), .dma_req_valid(lane_valid),
            .dma_req_ready(dma_client_req_ready[0]),
            .dma_rsp_rdata(dma_client_rsp_rdata[511:0]), .dma_rsp_tag(dma_client_rsp_tag[7:0]),
            .dma_rsp_len(dma_client_rsp_len[6:0]), .dma_rsp_status(dma_client_rsp_status[1:0]),
            .dma_rsp_write(dma_client_rsp_write[0]), .dma_rsp_valid(dma_client_rsp_valid[0]),
            .dma_rsp_ready(),
            .tx_axis_tdata(sonic_mac_tx_tdata), .tx_axis_tvalid(sonic_mac_tx_tvalid),
            .tx_axis_tready(sonic_mac_tx_tready), .tx_axis_tlast(sonic_mac_tx_tlast),
            .tx_cpl_valid(sonic_mac_tx_cpl_valid), .tx_cpl_ready(sonic_mac_tx_cpl_ready),
            .done_valid(core_done_valid), .done_ready(core_done_ready),
            .done_error(core_done_error), .done_pint(core_done_pint),
            .done_ctda(core_done_ctda),
            .done_tcr(core_done_tcr), .done_tps(core_done_tps), .done_tfc(core_done_tfc),
`ifdef ETH_DEBUG_ENABLE
            .dbg_state(tx_dbg_state), .dbg_descriptor_addr(tx_dbg_descriptor_addr)
`else
            .dbg_state(), .dbg_descriptor_addr()
`endif
        );
        q700_sonic_rx_cdc u_q700_sonic_rx_cdc (
            .pb_clk(pb_clk),.pb_rst(pb_full_rst_bank[3]),.pb_enable(sonic_rx_enabled_pb),
            .pb_cfg_valid(sonic_rx_cfg_valid_pb),.pb_cfg_ready(sonic_rx_cfg_ready_pb),
            .pb_cfg_op(sonic_rx_cfg_op_pb),
            .pb_cfg_dcr(sonic_rx_cfg_dcr_pb),.pb_cfg_rcr(sonic_rx_cfg_rcr_pb),.pb_cfg_urda(sonic_rx_cfg_urda_pb),.pb_cfg_crda(sonic_rx_cfg_crda_pb),
            .pb_cfg_urra(sonic_rx_cfg_urra_pb),.pb_cfg_rsa(sonic_rx_cfg_rsa_pb),.pb_cfg_rea(sonic_rx_cfg_rea_pb),.pb_cfg_rrp(sonic_rx_cfg_rrp_pb),
            .pb_cfg_rwp(sonic_rx_cfg_rwp_pb),.pb_cfg_eobc(sonic_rx_cfg_eobc_pb),.pb_cfg_rsc(sonic_rx_cfg_rsc_pb),.pb_cfg_llfa(sonic_rx_cfg_llfa_pb),
            .pb_cfg_cdp(sonic_rx_cfg_cdp_pb),.pb_cfg_cdc(sonic_rx_cfg_cdc_pb),
            .pb_done_valid(sonic_rx_done_valid_pb),.pb_done_ready(sonic_rx_done_ready_pb),.pb_done_error(sonic_rx_done_error_pb),
            .pb_done_rcr(sonic_rx_done_rcr_pb),.pb_done_crda(sonic_rx_done_crda_pb),.pb_done_crba0(sonic_rx_done_crba0_pb),.pb_done_crba1(sonic_rx_done_crba1_pb),
            .pb_done_rbwc0(sonic_rx_done_rbwc0_pb),.pb_done_rbwc1(sonic_rx_done_rbwc1_pb),.pb_done_rrp(sonic_rx_done_rrp_pb),.pb_done_rsc(sonic_rx_done_rsc_pb),
            .pb_done_llfa(sonic_rx_done_llfa_pb),.pb_done_trba0(sonic_rx_done_trba0_pb),.pb_done_trba1(sonic_rx_done_trba1_pb),
            .pb_done_tbwc0(sonic_rx_done_tbwc0_pb),.pb_done_tbwc1(sonic_rx_done_tbwc1_pb),.pb_done_isr_set(sonic_rx_done_isr_set_pb),
            .pb_done_cdp(sonic_rx_done_cdp_pb),.pb_done_cdc(sonic_rx_done_cdc_pb),.pb_done_ce(sonic_rx_done_ce_pb),
            .core_clk(core_clk),.core_rst(core_rst_bank[5]),.core_enable(rx_enable_core),
            .core_cfg_valid(rx_cfg_valid_core),.core_cfg_ready(rx_cfg_ready_core),
            .core_cfg_op(rx_cfg_op_core),
            .core_cfg_dcr(rx_dcr),.core_cfg_rcr(rx_rcr),.core_cfg_urda(rx_urda),.core_cfg_crda(rx_crda),.core_cfg_urra(rx_urra),
            .core_cfg_rsa(rx_rsa),.core_cfg_rea(rx_rea),.core_cfg_rrp(rx_rrp),.core_cfg_rwp(rx_rwp),.core_cfg_eobc(rx_eobc),.core_cfg_rsc(rx_rsc),.core_cfg_llfa(rx_llfa),
            .core_cfg_cdp(rx_cdp),.core_cfg_cdc(rx_cdc),
            .core_done_valid(rx_done_valid_core),.core_done_ready(rx_done_ready_core),.core_done_error(rx_done_error_core),
            .core_done_rcr(rx_done_rcr),.core_done_crda(rx_done_crda),.core_done_crba0(rx_done_crba0),.core_done_crba1(rx_done_crba1),
            .core_done_rbwc0(rx_done_rbwc0),.core_done_rbwc1(rx_done_rbwc1),.core_done_rrp(rx_done_rrp),.core_done_rsc(rx_done_rsc),
            .core_done_llfa(rx_done_llfa),.core_done_trba0(rx_done_trba0),.core_done_trba1(rx_done_trba1),
            .core_done_tbwc0(rx_done_tbwc0),.core_done_tbwc1(rx_done_tbwc1),.core_done_isr_set(rx_done_isr),
            .core_done_cdp(rx_done_cdp),.core_done_cdc(rx_done_cdc),.core_done_ce(rx_done_ce)
        );
        q700_sonic_rx #(.ACCEPT_ALL(1'b0)) u_q700_sonic_rx (
            .clk(core_clk),.rst(core_rst_bank[5]),.rx_enable(rx_enable_core),
            .promisc_enable(eth_promisc_enable),
            .cfg_valid(rx_cfg_valid_core),.cfg_ready(rx_cfg_ready_core),.cfg_op(rx_cfg_op_core),.cfg_dcr(rx_dcr),.cfg_rcr(rx_rcr),.cfg_urda(rx_urda),.cfg_crda(rx_crda),
            .cfg_urra(rx_urra),.cfg_rsa(rx_rsa),.cfg_rea(rx_rea),.cfg_rrp(rx_rrp),.cfg_rwp(rx_rwp),.cfg_eobc(rx_eobc),.cfg_rsc(rx_rsc),.cfg_llfa(rx_llfa),
            .cfg_cdp(rx_cdp),.cfg_cdc(rx_cdc),
            .rx_axis_tdata(sonic_mac_rx_tdata),.rx_axis_tvalid(sonic_mac_rx_tvalid),.rx_axis_tready(sonic_mac_rx_tready),
            .rx_axis_tlast(sonic_mac_rx_tlast),.rx_axis_tuser(sonic_mac_rx_tuser),
            .dma_req_addr(rx_lane_addr),.dma_req_len(rx_lane_len),.dma_req_wdata(rx_lane_wdata),.dma_req_tag(rx_lane_tag),
            .dma_req_write(rx_lane_write),.dma_req_valid(rx_lane_valid),.dma_req_ready(dma_client_req_ready[1]),
            .dma_rsp_rdata(dma_client_rsp_rdata[1023:512]),.dma_rsp_tag(dma_client_rsp_tag[15:8]),.dma_rsp_len(dma_client_rsp_len[13:7]),
            .dma_rsp_status(dma_client_rsp_status[3:2]),.dma_rsp_write(dma_client_rsp_write[1]),.dma_rsp_valid(dma_client_rsp_valid[1]),.dma_rsp_ready(),
            .done_valid(rx_done_valid_core),.done_ready(rx_done_ready_core),.done_error(rx_done_error_core),
            .done_rcr(rx_done_rcr),.done_crda(rx_done_crda),.done_crba0(rx_done_crba0),.done_crba1(rx_done_crba1),
            .done_rbwc0(rx_done_rbwc0),.done_rbwc1(rx_done_rbwc1),.done_rrp(rx_done_rrp),.done_rsc(rx_done_rsc),.done_llfa(rx_done_llfa),
            .done_trba0(rx_done_trba0),.done_trba1(rx_done_trba1),.done_tbwc0(rx_done_tbwc0),.done_tbwc1(rx_done_tbwc1),.done_isr_set(rx_done_isr),
            .done_cdp(rx_done_cdp),.done_cdc(rx_done_cdc),.done_ce(rx_done_ce),
`ifdef ETH_DEBUG_ENABLE
            .dbg_state(rx_dbg_state),.dbg_descriptor_addr(rx_dbg_descriptor_addr),
            .dbg_frame_len(rx_dbg_frame_len),
`else
            .dbg_state(),.dbg_descriptor_addr(),.dbg_frame_len(),
`endif
            // Not telemetry: sonic_loopback and the CAM taps feed real logic.
            .dbg_rcr(rx_dbg_rcr), .dbg_cam_enable(rx_dbg_cam_enable),
            .dbg_cam_index(eth_cam_index),
            .dbg_cam_entry(rx_dbg_cam_entry)
        );
`ifdef ETH_DEBUG_ENABLE
        // ══════════════════════════════════════════════════════════════
        // SONIC register-access trace ring (debug observer)
        // ══════════════════════════════════════════════════════════════
        // Snoops the SAME pb port u_q700_eth_sonic is attached to and never
        // drives it -- every port below is an input to the ring.  It exists
        // because the remaining Ethernet questions are all sequence
        // questions ("did the driver ever write IMR, or write it and have it
        // cleared?", "does CR_TXP stick set after ~8 transmits?") that a
        // post-mortem register snapshot structurally cannot answer.  Readout
        // is through u_eth_debug_regs at ETH_DEBUG_BASE+0x80..0x8c.  See
        // rtl/soc/sonic_trace_ring.v for the entry format and for the poll
        // filter that is what preserves the pre-wedge window.
        //
        // Declared and instantiated ENTIRELY inside this `ifdef, alongside
        // its only consumer, so an ETH_DEBUG_ENABLE=0 build has neither the
        // logic nor a dangling wire.  (Do not lift these declarations out of
        // the `ifdef and leave the consumers inside it, or the reverse --
        // that asymmetry is exactly what broke the ETH_DEBUG_ENABLE=0 build
        // for core_done_pint above.)
        //
        // Same reset as u_q700_eth_sonic (pb_full_rst_bank[3]) so the ring
        // rewinds exactly when the register file it is recording does -- a
        // ring that survived a reset the recorded device did not would
        // splice two different driver sessions into one trace.
        wire [11:0] sonic_trace_rd_addr;
        wire [31:0] sonic_trace_rd_data;
        wire [11:0] sonic_trace_wrptr;
        wire [15:0] sonic_trace_filtered;
        wire        sonic_trace_wrapped, sonic_trace_frozen;
        wire        sonic_trace_freeze, sonic_trace_clear;

        sonic_trace_ring #(.DEPTH_LG2(12)) u_sonic_trace_ring (
            .pb_clk  (pb_clk),
            .pb_rst  (pb_full_rst_bank[3]),
            .pb_addr (pb_sonic_addr),
            .pb_wdata(pb_sonic_wdata),
            .pb_wstrb(pb_sonic_wstrb),
            .pb_wr   (pb_sonic_wr),
            .pb_rd   (pb_sonic_rd),
            .pb_rdata(pb_sonic_rdata),
            .pb_ack  (pb_sonic_ack),
            .rd_clk    (core_clk),
            .rd_rst    (core_rst_bank[5]),
            .rd_freeze (sonic_trace_freeze),
            .rd_clear  (sonic_trace_clear),
            .rd_addr   (sonic_trace_rd_addr),
            .rd_data   (sonic_trace_rd_data),
            .rd_wrptr  (sonic_trace_wrptr),
            .rd_filtered(sonic_trace_filtered),
            .rd_wrapped(sonic_trace_wrapped),
            .rd_frozen (sonic_trace_frozen)
        );

        eth_debug_regs u_eth_debug_regs (
            .clk(core_clk), .rst(core_rst_bank[5]),
            .awaddr(eth_dbg_awaddr), .awvalid(eth_dbg_awvalid), .awready(eth_dbg_awready),
            .wdata(eth_dbg_wdata), .wstrb(eth_dbg_wstrb), .wvalid(eth_dbg_wvalid),
            .wready(eth_dbg_wready), .bresp(eth_dbg_bresp), .bvalid(eth_dbg_bvalid),
            .bready(eth_dbg_bready), .araddr(eth_dbg_araddr),
            .arvalid(eth_dbg_arvalid), .arready(eth_dbg_arready),
            .rdata(eth_dbg_rdata), .rresp(eth_dbg_rresp), .rvalid(eth_dbg_rvalid),
            .rready(eth_dbg_rready),
            .link_speed_async(eth_link_speed),
            .mac_event_toggle_async(eth_mac_debug_event_toggle),
            .sonic_irq_async(sonic_irq_pb), .sonic_rx_enable_async(sonic_rx_enabled_pb),
            .sonic_cr_async(sonic_dbg_cr_pb), .sonic_dcr_async(sonic_dbg_dcr_pb),
            .sonic_imr_async(sonic_dbg_imr_pb),
            .sonic_isr_async(sonic_dbg_isr_pb),
            .tx_state(tx_dbg_state), .rx_state(rx_dbg_state),
            .tx_descriptor_addr(tx_dbg_descriptor_addr),
            .rx_descriptor_addr(rx_dbg_descriptor_addr), .rx_frame_len(rx_dbg_frame_len),
            .rx_rcr(rx_dbg_rcr), .rx_cam_enable(rx_dbg_cam_enable),
            .rx_cam_entry(rx_dbg_cam_entry),
            .cam_index(eth_cam_index),
            .tx_cmd_fire(core_cmd_valid && core_cmd_ready),
            .tx_done_fire(core_done_valid && core_done_ready),
            .tx_done_error(core_done_error),
            .rx_done_fire(rx_done_valid_core && rx_done_ready_core),
            .rx_done_error(rx_done_error_core),
            .tx_axis_fire(sonic_mac_tx_tvalid && sonic_mac_tx_tready),
            .tx_axis_last(sonic_mac_tx_tlast),
            .rx_axis_fire(sonic_mac_rx_tvalid && sonic_mac_rx_tready),
            .rx_axis_last(sonic_mac_rx_tlast), .rx_axis_user(sonic_mac_rx_tuser),
            .dma_req_fire({rx_lane_valid && dma_client_req_ready[1],
                           lane_valid && dma_client_req_ready[0]}),
            .dma_req_addr({rx_lane_addr,lane_addr}), .dma_req_len({rx_lane_len,lane_len}),
            .dma_req_tag({rx_lane_tag,lane_tag}), .dma_req_write({rx_lane_write,lane_write}),
            .dma_rsp_fire(dma_client_rsp_valid[1:0]),
            .dma_rsp_status(dma_client_rsp_status[3:0]),
            .dma_rsp_tag(dma_client_rsp_tag[15:0]),
            .dma_rsp_write(dma_client_rsp_write[1:0]),
            .promisc_enable(eth_promisc_enable),
            .trace_rd_addr(sonic_trace_rd_addr),
            .trace_rd_data(sonic_trace_rd_data),
            .trace_wrptr(sonic_trace_wrptr),
            .trace_filtered(sonic_trace_filtered),
            .trace_wrapped(sonic_trace_wrapped),
            .trace_frozen(sonic_trace_frozen),
            .trace_freeze(sonic_trace_freeze),
            .trace_clear(sonic_trace_clear)
        );
`endif
        assign dma_client_req_addr = {64'd0,rx_lane_addr,lane_addr};
        assign dma_client_req_len = {14'd0,rx_lane_len,lane_len};
        assign dma_client_req_wdata = {1024'd0,rx_lane_wdata,lane_wdata};
        assign dma_client_req_tag = {16'd0,rx_lane_tag,lane_tag};
        assign dma_client_req_write = {2'd0,rx_lane_write,lane_write};
        assign dma_client_req_valid = {2'd0,rx_lane_valid,lane_valid};
    end else begin : g_sonic_dma_inactive
        assign dma_client_req_addr = 128'd0;
        assign dma_client_req_len = 28'd0;
        assign dma_client_req_wdata = 2048'd0;
        assign dma_client_req_tag = 32'd0;
        assign dma_client_req_write = 4'd0;
        assign dma_client_req_valid = 4'd0;
        assign sonic_tx_cmd_ready_pb = 1'b0;
        assign sonic_tx_done_valid_pb = 1'b0;
        assign sonic_tx_done_error_pb = 1'b0;
        assign sonic_tx_done_pint_pb = 1'b0;
        assign sonic_tx_done_ctda_pb = 16'd0;
        assign sonic_tx_done_tcr_pb = 16'd0;
        assign sonic_tx_done_tps_pb = 16'd0;
        assign sonic_tx_done_tfc_pb = 16'd0;
        assign sonic_mac_tx_tdata = 8'd0;
        assign sonic_mac_tx_tvalid = 1'b0;
        assign sonic_mac_tx_tlast = 1'b0;
        assign sonic_mac_tx_cpl_ready = 1'b0;
        assign sonic_mac_rx_tready = 1'b1;
        assign sonic_rx_cfg_ready_pb=0; assign sonic_rx_done_valid_pb=0; assign sonic_rx_done_error_pb=0;
        assign sonic_rx_done_rcr_pb=0; assign sonic_rx_done_crda_pb=0; assign sonic_rx_done_crba0_pb=0; assign sonic_rx_done_crba1_pb=0;
        assign sonic_rx_done_rbwc0_pb=0; assign sonic_rx_done_rbwc1_pb=0; assign sonic_rx_done_rrp_pb=0; assign sonic_rx_done_rsc_pb=0;
        assign sonic_rx_done_llfa_pb=0; assign sonic_rx_done_trba0_pb=0; assign sonic_rx_done_trba1_pb=0;
        assign sonic_rx_done_tbwc0_pb=0; assign sonic_rx_done_tbwc1_pb=0; assign sonic_rx_done_isr_set_pb=0;
        assign sonic_rx_done_cdp_pb=0; assign sonic_rx_done_cdc_pb=0; assign sonic_rx_done_ce_pb=0;
    end endgenerate
`else
    assign dma_client_req_addr = 128'd0;
    assign dma_client_req_len = 28'd0;
    assign dma_client_req_wdata = 2048'd0;
    assign dma_client_req_tag = 32'd0;
    assign dma_client_req_write = 4'd0;
    assign dma_client_req_valid = 4'd0;
    assign sonic_tx_cmd_ready_pb = 1'b0;
    assign sonic_tx_done_valid_pb = 1'b0;
    assign sonic_tx_done_error_pb = 1'b0;
    assign sonic_tx_done_ctda_pb = 16'd0;
    assign sonic_tx_done_tcr_pb = 16'd0;
    assign sonic_tx_done_tps_pb = 16'd0;
    assign sonic_tx_done_tfc_pb = 16'd0;
    assign sonic_mac_tx_tdata = 8'd0;
    assign sonic_mac_tx_tvalid = 1'b0;
    assign sonic_mac_tx_tlast = 1'b0;
    assign sonic_mac_tx_cpl_ready = 1'b0;
    assign sonic_mac_rx_tready = 1'b1;
    assign sonic_rx_cfg_ready_pb=0; assign sonic_rx_done_valid_pb=0; assign sonic_rx_done_error_pb=0;
    assign sonic_rx_done_rcr_pb=0; assign sonic_rx_done_crda_pb=0; assign sonic_rx_done_crba0_pb=0; assign sonic_rx_done_crba1_pb=0;
    assign sonic_rx_done_rbwc0_pb=0; assign sonic_rx_done_rbwc1_pb=0; assign sonic_rx_done_rrp_pb=0; assign sonic_rx_done_rsc_pb=0;
    assign sonic_rx_done_llfa_pb=0; assign sonic_rx_done_trba0_pb=0; assign sonic_rx_done_trba1_pb=0;
    assign sonic_rx_done_tbwc0_pb=0; assign sonic_rx_done_tbwc1_pb=0; assign sonic_rx_done_isr_set_pb=0;
    assign sonic_rx_done_cdp_pb=0; assign sonic_rx_done_cdc_pb=0; assign sonic_rx_done_ce_pb=0;
`endif

    // The shared engine reports completion per client; it has no global IRQ.
    // Peripheral adapters raise their own device-specific interrupt after
    // consuming a completion.
    assign dma_irq_w = 1'b0;
