// Ethernet register/packet-engine seams are declared unconditionally because
// the SONIC register block exists in every build.  Non-Ethernet builds tie
// them off next to the register instance in fpga_top_peripherals.vh.
    wire sonic_tx_cmd_valid_pb, sonic_tx_cmd_ready_pb;
    wire [15:0] sonic_tx_cmd_dcr_pb, sonic_tx_cmd_utda_pb, sonic_tx_cmd_ctda_pb;
    wire sonic_tx_done_valid_pb, sonic_tx_done_ready_pb, sonic_tx_done_error_pb;
    wire sonic_tx_done_pint_pb;
    wire [15:0] sonic_tx_done_ctda_pb, sonic_tx_done_tcr_pb;
    wire [15:0] sonic_tx_done_tps_pb, sonic_tx_done_tfc_pb;
    wire sonic_rx_enabled_pb,sonic_rx_cfg_valid_pb,sonic_rx_cfg_ready_pb;
    wire sonic_tx_halt_pb;   // CR_HTX, pb domain; crossed in u_q700_sonic_cdc
    wire [2:0] sonic_rx_cfg_op_pb;
    wire [15:0] sonic_rx_cfg_dcr_pb,sonic_rx_cfg_rcr_pb,sonic_rx_cfg_urda_pb,sonic_rx_cfg_crda_pb;
    wire [15:0] sonic_rx_cfg_urra_pb,sonic_rx_cfg_rsa_pb,sonic_rx_cfg_rea_pb,sonic_rx_cfg_rrp_pb;
    wire [15:0] sonic_rx_cfg_rwp_pb,sonic_rx_cfg_eobc_pb,sonic_rx_cfg_rsc_pb,sonic_rx_cfg_llfa_pb;
    wire [15:0] sonic_rx_cfg_cdp_pb,sonic_rx_cfg_cdc_pb;
    wire sonic_rx_done_valid_pb,sonic_rx_done_ready_pb,sonic_rx_done_error_pb;
    wire [15:0] sonic_rx_done_rcr_pb,sonic_rx_done_crda_pb,sonic_rx_done_crba0_pb,sonic_rx_done_crba1_pb;
    wire [15:0] sonic_rx_done_rbwc0_pb,sonic_rx_done_rbwc1_pb,sonic_rx_done_rrp_pb,sonic_rx_done_rsc_pb;
    wire [15:0] sonic_rx_done_llfa_pb,sonic_rx_done_trba0_pb,sonic_rx_done_trba1_pb;
    wire [15:0] sonic_rx_done_tbwc0_pb,sonic_rx_done_tbwc1_pb,sonic_rx_done_isr_set_pb;
    wire [15:0] sonic_rx_done_cdp_pb,sonic_rx_done_cdc_pb,sonic_rx_done_ce_pb;
    wire [7:0] sonic_mac_tx_tdata, sonic_mac_rx_tdata;
    // RCR[10:9] (RCR_LB) -- any non-zero value means loopback.  MAME treats
    // the field as one boolean (set_loopback(RCR & RCR_LB)) rather than
    // distinguishing MAC/ENDEC/transceiver, so we do the same.  Driven from
    // the RX engine's LIVE rcr, which is already core_clk and already
    // refreshed on an RCR write, so no extra CDC is needed.
    wire sonic_loopback;
    wire sonic_rx_drain;
    wire sonic_mac_tx_tvalid, sonic_mac_tx_tready, sonic_mac_tx_tlast;
    wire sonic_mac_tx_cpl_valid, sonic_mac_tx_cpl_ready;
    wire sonic_mac_rx_tvalid, sonic_mac_rx_tready, sonic_mac_rx_tlast, sonic_mac_rx_tuser;
    wire [15:0] sonic_dbg_cr_pb, sonic_dbg_imr_pb, sonic_dbg_isr_pb;
    wire [15:0] sonic_dbg_dcr_pb;
    wire [7:0] eth_mac_debug_event_toggle;

    // Second client on the shared Taxi MAC: the Ethernet-backed block service
    // (docs/net_vhdd_design.md).  The engine does not exist yet, so the seam is
    // strapped absent: an all-zero blk_mac tells q700_eth_link no client is
    // attached, which keeps RX routing -- broadcast included -- exactly as it
    // was before the MAC was shared.  Wire these to the block engine when it
    // lands; the link itself now carries the clock crossing, so the engine
    // attaches on pb_clk and nothing else in the link needs to change.
`ifdef ENABLE_NET_VHDD
    // Driven by u_net_block_framer in fpga_top_peripherals.vh, which sits on
    // pb_clk beside vhdd_sd; q700_eth_link crosses these to core_clk itself.
    // blk_mac comes straight off the CSR with no synchroniser because the
    // demux that consumes it is already core_clk, and it is quasi-static --
    // written once over JTAG, and written LAST so the client stays absent
    // until the rest of the endpoint is in place.
    wire [47:0] net_blk_mac = net_vhdd_our_mac;
    wire [7:0]  net_blk_tx_tdata;
    wire        net_blk_tx_tvalid;
    wire        net_blk_tx_tready;
    wire        net_blk_tx_tlast;
    wire [7:0]  net_blk_rx_tdata;
    wire        net_blk_rx_tvalid;
    wire        net_blk_rx_tready;
    wire        net_blk_rx_tlast;
`else
    wire [47:0] net_blk_mac = 48'h00_00_00_00_00_00;
    wire [7:0]  net_blk_tx_tdata = 8'd0;
    wire        net_blk_tx_tvalid = 1'b0;
    wire        net_blk_tx_tready;
    wire        net_blk_tx_tlast = 1'b0;
    wire [7:0]  net_blk_rx_tdata;
    wire        net_blk_rx_tvalid;
    // An absent client must never backpressure the shared wire stream.
    wire        net_blk_rx_tready = 1'b1;
    wire        net_blk_rx_tlast;
`endif // ENABLE_NET_VHDD

`ifdef ETH_DEBUG_ENABLE
    // AXI-Lite slave face for the optional telemetry page at 0x5098_0000.
    wire [19:0] eth_dbg_awaddr, eth_dbg_araddr;
    wire eth_dbg_awvalid, eth_dbg_awready, eth_dbg_wvalid, eth_dbg_wready;
    wire [31:0] eth_dbg_wdata; wire [3:0] eth_dbg_wstrb;
    wire [1:0] eth_dbg_bresp; wire eth_dbg_bvalid, eth_dbg_bready;
    wire eth_dbg_arvalid, eth_dbg_arready;
    wire [31:0] eth_dbg_rdata; wire [1:0] eth_dbg_rresp;
    wire eth_dbg_rvalid, eth_dbg_rready;
`endif

`ifdef ETH_ENABLE
    wire [1:0] eth_link_speed;
    wire eth_rx_activity, eth_tx_activity;
    q700_eth_link #(.ICMP_RESPONDER(ETH_ICMP_RESPONDER),
`ifdef SIM_MODEL
        .REF_CLK_MHZ(200)
`else
        .REF_CLK_MHZ(100)
`endif
    ) u_q700_eth_link (
        .clk_ref(video_ref_clk),
        .phy_rx_clk(phy_rx_clk), .phy_rxd(phy_rxd), .phy_rx_ctl(phy_rx_ctl),
        .phy_tx_clk(phy_tx_clk), .phy_txd(phy_txd), .phy_tx_ctl(phy_tx_ctl),
        .core_clk(core_clk), .core_rst(core_rst_bank[5]),
        .loopback(sonic_loopback),
        .rx_drain(sonic_rx_drain),
        .sonic_tx_tdata(sonic_mac_tx_tdata), .sonic_tx_tvalid(sonic_mac_tx_tvalid),
        .sonic_tx_tready(sonic_mac_tx_tready), .sonic_tx_tlast(sonic_mac_tx_tlast),
        .sonic_tx_cpl_valid(sonic_mac_tx_cpl_valid), .sonic_tx_cpl_ready(sonic_mac_tx_cpl_ready),
        .sonic_rx_tdata(sonic_mac_rx_tdata), .sonic_rx_tvalid(sonic_mac_rx_tvalid),
        .sonic_rx_tready(sonic_mac_rx_tready), .sonic_rx_tlast(sonic_mac_rx_tlast),
        .sonic_rx_tuser(sonic_mac_rx_tuser),
        .blk_mac(net_blk_mac),
        // The block client lives in the vhdd/SCSI domain, beside vhdd_sd on
        // the pb_clk vhdd contract -- q700_eth_link crosses it to core_clk
        // internally with a pair of FRAME fifos.  Same reset bank as u_scsi
        // so a debug full reset rewinds the target and its transport together.
        .blk_clk(pb_clk), .blk_rst(pb_full_rst_bank[2]),
        .blk_tx_tdata(net_blk_tx_tdata), .blk_tx_tvalid(net_blk_tx_tvalid),
        .blk_tx_tready(net_blk_tx_tready), .blk_tx_tlast(net_blk_tx_tlast),
        .blk_rx_tdata(net_blk_rx_tdata), .blk_rx_tvalid(net_blk_rx_tvalid),
        .blk_rx_tready(net_blk_rx_tready), .blk_rx_tlast(net_blk_rx_tlast),
        .link_speed(eth_link_speed), .rx_activity(eth_rx_activity), .tx_activity(eth_tx_activity),
        .debug_event_toggle(eth_mac_debug_event_toggle)
    );
`else
    assign sonic_mac_tx_tready = 1'b0;
    assign sonic_mac_tx_cpl_valid = 1'b0;
    assign sonic_mac_rx_tdata = 8'd0;
    assign sonic_mac_rx_tvalid = 1'b0;
    assign sonic_mac_rx_tlast = 1'b0;
    assign sonic_mac_rx_tuser = 1'b0;
    assign eth_mac_debug_event_toggle = 8'd0;
    assign net_blk_tx_tready = 1'b0;
    assign net_blk_rx_tdata = 8'd0;
    assign net_blk_rx_tvalid = 1'b0;
    assign net_blk_rx_tlast = 1'b0;
`endif
