// tb_scsi_trace_pb.v — integration wrapper that reproduces the SHIPPING
// wiring of the 53C96 trace ring: real peripheral_bus.v -> real scsi.v,
// with rtl/soc/scsi_trace_ring.v snooping the pb port and
// rtl/soc/vhdd_ctrl.v presenting the ring to the host over AXI4-Lite.
//
// WHY A SECOND SCSI TOP.  tb/tb_scsi_trace_ring.cpp already unit-tests
// the ring against hand-driven pb beats.  That proves the ring's own
// logic and nothing about the INTEGRATION, which is where this feature
// has actually failed before: the module has existed, linted and passed
// its unit tb for a month while being absent from every bitstream, and
// the three things a user of `scsi-trace` depends on
//
//   1. that the tap sees the beats scsi.v really answers, including the
//      pseudo-DMA aperture beats whose ack peripheral_bus/scsi.v can
//      withhold for thousands of cycles,
//   2. that the vhdd_ctrl register map matches what tools/jtag_repl.tcl
//      reads (0x020 TRACE_CTRL / 0x024 TRACE_ADDR / 0x028 TRACE_DATA,
//      and the {frozen, wrapped, wr_ptr} packing inside TRACE_CTRL),
//   3. that freeze/re-arm reach the capture domain across the pb_clk ->
//      core_clk CDC,
//
// are all properties of the WIRING, not of the ring in isolation.  None
// of them is observable from either existing unit tb.
//
// STRUCTURE.  tb/tb_pb_scsi.v is reused verbatim as the SCSI subsystem
// (peripheral_bus + scsi + vhdd_sd + the peripheral tie-offs), through
// the observation-only snoop_* outputs added to it for this purpose, so
// there is exactly one description of that subsystem in the tree.  This
// file adds only what fpga_top does: the ring, the CSR block, and the
// two clock domains they live in.
//
// TWO CLOCKS, ON PURPOSE.  `clk` is the pb_clk island (peripheral_bus,
// scsi.v, ring capture); `rd_clk` is core_clk (vhdd_ctrl, ring readout).
// The C++ harness runs them at different rates so the ring's control
// CDC (rd_freeze level, rd_clear toggle) and its readout synchronisers
// are genuinely crossed, as they are on hardware.  Running them locked
// together would test a design that does not ship.
//
// POSITIVE CONTROL.  `cap_en` gates the capture taps -- and ONLY the
// taps; the SCSI subsystem is untouched, so the traffic still happens,
// it just is not recorded.  With cap_en=0 every capture assertion in
// tb_scsi_trace_pb.cpp must fail.  This is fault injection in the
// harness, never in the DUT: the ring instance below is the same
// instantiation fpga_top_peripherals.vh has.

`default_nettype none

module tb_scsi_trace_pb #(
    parameter ID_WIDTH = 6,
    parameter PB_WATCHDOG_LOG2 = 12,
    parameter ENABLE_ACK_WATCHDOG = 1
) (
    // ── pb_clk island ─────────────────────────────────────────────────
    input  wire                 clk,
    input  wire                 rst,
    // ── core_clk island (host/JTAG readout) ───────────────────────────
    input  wire                 rd_clk,
    input  wire                 rd_rst,

    // Harness-only capture gate (see POSITIVE CONTROL above).
    input  wire                 cap_en,

    input  wire [31:0]          disk_num_lbas,

    // ── AXI4 slave: the Mac's MMIO path into peripheral_bus ───────────
    input  wire [ID_WIDTH-1:0]  s_awid,
    input  wire [31:0]          s_awaddr,
    input  wire [7:0]           s_awlen,
    input  wire [2:0]           s_awsize,
    input  wire [1:0]           s_awburst,
    input  wire                 s_awvalid,
    output wire                 s_awready,

    input  wire [127:0]         s_wdata,
    input  wire [15:0]          s_wstrb,
    input  wire                 s_wlast,
    input  wire                 s_wvalid,
    output wire                 s_wready,

    output wire [ID_WIDTH-1:0]  s_bid,
    output wire [1:0]           s_bresp,
    output wire                 s_bvalid,
    input  wire                 s_bready,

    input  wire [ID_WIDTH-1:0]  s_arid,
    input  wire [31:0]          s_araddr,
    input  wire [7:0]           s_arlen,
    input  wire [2:0]           s_arsize,
    input  wire [1:0]           s_arburst,
    input  wire                 s_arvalid,
    output wire                 s_arready,

    output wire [ID_WIDTH-1:0]  s_rid,
    output wire [127:0]         s_rdata,
    output wire [1:0]           s_rresp,
    output wire                 s_rlast,
    output wire                 s_rvalid,
    input  wire                 s_rready,

    // ── AXI4-Lite: the host's path into vhdd_ctrl (xbar S2) ───────────
    // Byte offsets inside the 1 MB window, i.e. exactly what
    // tools/jtag_repl.tcl computes as VHDD_BASE + off.
    input  wire [19:0]          cfg_awaddr,
    input  wire                 cfg_awvalid,
    output wire                 cfg_awready,
    input  wire [31:0]          cfg_wdata,
    input  wire [3:0]           cfg_wstrb,
    input  wire                 cfg_wvalid,
    output wire                 cfg_wready,
    output wire [1:0]           cfg_bresp,
    output wire                 cfg_bvalid,
    input  wire                 cfg_bready,
    input  wire [19:0]          cfg_araddr,
    input  wire                 cfg_arvalid,
    output wire                 cfg_arready,
    output wire [31:0]          cfg_rdata,
    output wire [1:0]           cfg_rresp,
    output wire                 cfg_rvalid,
    input  wire                 cfg_rready,

    // ── SCSI-side control / observation, straight through ─────────────
    input  wire [8:0]           scsi_ctrl_in,
    output wire                 scsi_irq,
    output wire                 scsi_drq,

    output wire [2:0]           sd_cmd_type,
    output wire [31:0]          sd_lba,
    output wire [15:0]          sd_block_count,
    output wire                 sd_go,
    input  wire                 sd_busy,
    input  wire                 sd_done,
    input  wire                 sd_error,
    input  wire                 sd_rd_valid,
    input  wire [7:0]           sd_rd_data,
    output wire                 sd_rd_ready,
    input  wire                 sd_wr_ready,
    output wire                 sd_wr_valid,
    output wire                 sd_wr_avail,
    output wire [7:0]           sd_wr_data,

    // ── The snooped bus, re-exported so the C++ harness can build its
    //    OWN golden model of what the ring should hold.  The model must
    //    come from the bus, not from the test's expectations: a model
    //    written from expectations would agree with a ring that is
    //    recording the wrong wires.
    output wire [8:0]           snoop_scsi_addr,
    output wire [7:0]           snoop_scsi_wdata,
    output wire                 snoop_scsi_wr,
    output wire                 snoop_scsi_rd,
    output wire [7:0]           snoop_scsi_rdata,
    output wire                 snoop_scsi_ack
);

    tb_pb_scsi #(
        .ID_WIDTH           (ID_WIDTH),
        .PB_WATCHDOG_LOG2   (PB_WATCHDOG_LOG2),
        .ENABLE_ACK_WATCHDOG(ENABLE_ACK_WATCHDOG)
    ) u_sys (
        .clk(clk), .rst(rst),
        .periph_rst(1'b0), // This wrapper exercises only the common reset.
        .disk_num_lbas(disk_num_lbas),

        .s_awid(s_awid), .s_awaddr(s_awaddr), .s_awlen(s_awlen),
        .s_awsize(s_awsize), .s_awburst(s_awburst), .s_awvalid(s_awvalid),
        .s_awready(s_awready),
        .s_wdata(s_wdata), .s_wstrb(s_wstrb), .s_wlast(s_wlast),
        .s_wvalid(s_wvalid), .s_wready(s_wready),
        .s_bid(s_bid), .s_bresp(s_bresp), .s_bvalid(s_bvalid), .s_bready(s_bready),
        .s_arid(s_arid), .s_araddr(s_araddr), .s_arlen(s_arlen),
        .s_arsize(s_arsize), .s_arburst(s_arburst), .s_arvalid(s_arvalid),
        .s_arready(s_arready),
        .s_rid(s_rid), .s_rdata(s_rdata), .s_rresp(s_rresp),
        .s_rlast(s_rlast), .s_rvalid(s_rvalid), .s_rready(s_rready),

        .scsi_ctrl_in(scsi_ctrl_in),
        .scsi_irq(scsi_irq), .scsi_drq(scsi_drq),

        .sd_cmd_type(sd_cmd_type), .sd_lba(sd_lba),
        .sd_block_count(sd_block_count), .sd_go(sd_go),
        .sd_busy(sd_busy), .sd_done(sd_done), .sd_error(sd_error),
        .sd_rd_valid(sd_rd_valid), .sd_rd_data(sd_rd_data),
        .sd_rd_ready(sd_rd_ready),
        .sd_wr_ready(sd_wr_ready), .sd_wr_valid(sd_wr_valid),
        .sd_wr_avail(sd_wr_avail), .sd_wr_data(sd_wr_data),

        .snoop_scsi_addr (snoop_scsi_addr),
        .snoop_scsi_wdata(snoop_scsi_wdata),
        .snoop_scsi_wr   (snoop_scsi_wr),
        .snoop_scsi_rd   (snoop_scsi_rd),
        .snoop_scsi_rdata(snoop_scsi_rdata),
        .snoop_scsi_ack  (snoop_scsi_ack)
    );

    // ── The ring, wired exactly as fpga_top_peripherals.vh wires it ───
    // Depth is FIXED at 4096, not a parameter: vhdd_ctrl's
    // trace_rd_addr/trace_wrptr ports are 12 bits wide and
    // tools/jtag_repl.tcl hardcodes VHDD_TRACE_DEPTH 4096, so a
    // "convenient" smaller depth here would test a geometry that cannot
    // ship and would silently truncate the address on the way in.
    localparam integer TRACE_DEPTH_LG2 = 12;
    wire [TRACE_DEPTH_LG2-1:0] trace_rd_addr;
    wire [31:0]                trace_rd_data;
    wire [TRACE_DEPTH_LG2-1:0] trace_wrptr;
    wire                       trace_wrapped;
    wire                       trace_frozen;
    wire                       trace_freeze;
    wire                       trace_clear;

    scsi_trace_ring #(
        .DEPTH_LG2 (TRACE_DEPTH_LG2)
    ) u_scsi_trace_ring (
        .pb_clk    (clk),
        .pb_rst    (rst),
        .pb_addr   (snoop_scsi_addr),
        .pb_wdata  (snoop_scsi_wdata),
        // cap_en is the harness's fault-injection gate and exists ONLY
        // here; the shipping instantiation ties these straight to the
        // bus.  Gating the REQUEST strobes (not the ack) means the ring
        // never even latches a beat, which is what "capture disabled"
        // has to mean for the positive control to be honest.
        .pb_wr     (snoop_scsi_wr & cap_en),
        .pb_rd     (snoop_scsi_rd & cap_en),
        .pb_rdata  (snoop_scsi_rdata),
        .pb_ack    (snoop_scsi_ack),
        .rd_clk    (rd_clk),
        .rd_rst    (rd_rst),
        .rd_freeze (trace_freeze),
        .rd_clear  (trace_clear),
        .rd_addr   (trace_rd_addr),
        .rd_data   (trace_rd_data),
        .rd_wrptr  (trace_wrptr),
        .rd_wrapped(trace_wrapped),
        .rd_frozen (trace_frozen)
    );

    // ── The CSR block the host actually talks to ──────────────────────
    // Same parameters as fpga_top_dma.vh's u_vhdd_ctrl for the fields
    // that matter here; the RAM-disk status inputs are tied off because
    // this harness has no RAM disk and never reads those offsets.
    vhdd_ctrl #(
        .ADDR_WIDTH(20)
    ) u_vhdd_ctrl (
        .clk    (rd_clk), .rst(rd_rst), .cfg_rst(rd_rst),
        .awaddr (cfg_awaddr ), .awvalid(cfg_awvalid), .awready(cfg_awready),
        .wdata  (cfg_wdata  ), .wstrb  (cfg_wstrb  ),
        .wvalid (cfg_wvalid ), .wready (cfg_wready ),
        .bresp  (cfg_bresp  ), .bvalid (cfg_bvalid ), .bready (cfg_bready),
        .araddr (cfg_araddr ), .arvalid(cfg_arvalid), .arready(cfg_arready),
        .rdata  (cfg_rdata  ), .rresp  (cfg_rresp  ),
        .rvalid (cfg_rvalid ), .rready (cfg_rready ),
        .dev_en       (),
        // Ports vhdd_ctrl grew after this harness was written (network trace
        // export + the SD write-protect status bit). All are OUTPUTS and this
        // harness consumes none of them, so name them explicitly rather than
        // leaving them implicit -- Verilator treats PINMISSING as an error here,
        // which had left this whole testbench UNBUILDABLE on main.
        .net_our_mac  (),
        .net_our_ip   (),
        .net_our_port (),
        .net_dst_mac  (),
        .net_dst_ip   (),
        .net_dst_port (),
        .sd_wprot     (),
        .rd_num_lbas  (),
        .sd_num_lbas  (32'd0),
        .rd_busy      (1'b0),
        .rd_error     (1'b0),
        .rd_state     (4'd0),
        .rd_wdog_fires(16'd0),
        .trace_rd_addr(trace_rd_addr),
        .trace_rd_data(trace_rd_data),
        .trace_wrptr  (trace_wrptr),
        .trace_wrapped(trace_wrapped),
        .trace_frozen (trace_frozen),
        .trace_freeze (trace_freeze),
        .trace_clear  (trace_clear)
    );

endmodule

`default_nettype wire
