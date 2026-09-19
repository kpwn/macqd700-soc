// tb_pb_scsi.v — Verilog wrapper instantiating peripheral_bus + scsi
// together (real RTL, not a BFM stand-in for scsi) for the SCSI
// DMA-shim handshake reconciliation (T1b: scsi-pb-reconcile).
//
// Root problem this integration tb exists to catch: tb_turboscsi.cpp and
// tb_scsi_c96_read6.cpp drive scsi.v's pb_addr/pb_wdata/pb_wr/pb_rd ports
// directly, bypassing peripheral_bus.v entirely — so no pre-existing unit
// tb can exercise the DMA-shim handshake end-to-end through the real
// peripheral_bus.v FSM (pulse-vs-hold sequencing, ack-timeout watchdog,
// concurrent-R+W address-mux arbitration) driving a REAL scsi.v.  This
// wrapper instantiates the REAL peripheral_bus.v driving a REAL scsi.v
// (TURBOSCSI_C96=1) so the C++ harness can drive genuine AXI4
// transactions through the fabric and observe the handshake end-to-end.
//
// Other peripheral_bus faces are tied off with lightweight stand-ins:
//   - VIA1: a tiny 16-byte register file with a registered ack (matches
//     the general "pb_ack one cycle after pb_wr/pb_rd" contract every
//     real pb_* peripheral in this design follows) and a distinguishable
//     reset pattern (reg[i] = 0xA0+i) — used by the "the bus stays alive
//     for other peripherals after a SCSI timeout" scenario.
//   - debug_ctrl / DAFB AXI4-Lite faces: always-ready, immediate-ack
//     stubs (never exercised by these tests, but must present a valid
//     AXI4-Lite response so the module doesn't hang if mis-routed).
//   - VIA2 / ENET / SONIC / ORWELL / SCC / ASC / IWM / ADBINJ: trivial
//     combinational-ack stubs (rdata=0), likewise unexercised.
//
// SCSI runs in TURBOSCSI_C96=1 mode with scsi_ctrl_in exposed directly
// to the top (DAFB's real wiring path — NOT part of peripheral_bus'
// pb_* contract) so the C++ harness can toggle the DRQ-check bits.  The
// sd_* backing-store interface is also exposed so the harness can drive
// (or withhold) SD data, controlling when drq_c96 rises.
//
// scsi.v's dma_rd_ready / dma_wr_ready outputs (added 2026-07-21,
// commits f22b26d / eba8aa2) feed peripheral_bus's scsi_dma_rd_ready /
// scsi_dma_wr_ready inputs — the DRQ-check pulse-gate mechanism this
// base branch uses (as opposed to a level-held request).
//
// PB_WATCHDOG_LOG2 overrides peripheral_bus's ack-timeout watchdog
// bound (default 12 -> 2^12 = 4096 pb-clk cycles) so the "DRQ never
// rises -> SLVERR" scenario stays fast in sim.  Production
// (fpga_top_peripherals.vh) instantiates peripheral_bus with the
// module's real default (24, ~335 ms @ 50 MHz) — no override needed
// there.

// 2026-08-19 (scsi-fuzz widening): TARGET_ID / SD_LBA_BIAS parameters and
// a `disk_num_lbas` port were added so tools/fuzz/scsi_fuzz.py can reuse
// THIS top as its RTL executor (tb/tb_scsi_fuzz.cpp built with
// -DSCSI_FUZZ_PB).  That is what puts peripheral_bus.v — and therefore the
// word-splitting rd_scsi_phase_q / wr_scsi_strb_q serializers and the
// scsi_dma16_lo_beat grant carry — INSIDE the differential fuzzer's DUT.
// Before that the fuzz top was scsi.v + vhdd_sd only, so no seed could
// reach the FSM that produced the 2026-08-19 Sad Mac 0F02.  Defaults are
// the historical tb_pb_scsi.cpp shape, so that harness is unchanged.
module tb_pb_scsi #(
    parameter ID_WIDTH = 6,
    parameter PB_WATCHDOG_LOG2 = 12,
    // peripheral_bus's ack watchdog is OFF by default in production
    // (owner decision 2026-09-06: fix forward-progress bugs, do not
    // bound them).  This tb still exercises the "DRQ never rises ->
    // SLVERR" scenario, so it turns the watchdog back ON explicitly.
    parameter ENABLE_ACK_WATCHDOG = 1,
    parameter [2:0]  TARGET_ID   = 3'd0,
    parameter [31:0] SD_LBA_BIAS = 32'd8192
) (
    input  wire                 clk,
    input  wire                 rst,
    // Peripheral-only reset (2026-09-18).  In the SoC the pb peripherals
    // and peripheral_bus sit on DIFFERENT reset nets: pb_full_rst (board
    // cold | JTAG debug-full | warm_peripheral_reset, i.e. a 68040 RESET
    // instruction) resets u_scsi, while peripheral_bus keeps running on
    // pb_soc_full_rst so the CPU can keep using it.  This harness used to
    // have only the shared `rst` and could not express that window at all.
    // Hold this high (with `rst` low) to model "the 53C96 is in reset
    // while the bus is live" -- the shape that used to strand a one-shot
    // pb_* strobe and kill the whole S1 slave.  Defaults to 0, so every
    // existing harness is unchanged.
    input  wire                 periph_rst,

    // Backing-store size presented to scsi.v / vhdd_sd.  tb_pb_scsi.cpp
    // drives 32'd1048576 (the historical hardcoded value); the fuzz
    // harness drives its deterministic-disk size.
    input  wire [31:0]          disk_num_lbas,

    // ── AXI4 slave (driven by the C++ harness) ─────────────────────────
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

    // ── SCSI TurboSCSI DRQ-check control (DAFB-direct, not pb_*) ───────
    input  wire [8:0]           scsi_ctrl_in,

    // ── SCSI informational outs ──────────────────────────────────────
    output wire                 scsi_irq,
    output wire                 scsi_drq,

    // ── SCSI SD backing-store interface (driven by the C++ harness) ────
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

    // ── Snoop taps on the peripheral_bus <-> scsi.v port ───────────────
    // OBSERVATION ONLY.  Added 2026-09-09 so tb/tb_scsi_trace_pb.v can
    // hang rtl/soc/scsi_trace_ring.v off exactly the wires the ring is
    // wired to in fpga_top_peripherals.vh, and so the C++ harness can
    // build an INDEPENDENT golden model of what the ring should have
    // captured from the real bus rather than from its own expectations.
    // Nothing inside this module reads them back, so they cannot change
    // the behaviour of any existing test built on this top.
    output wire [8:0]           snoop_scsi_addr,
    output wire [7:0]           snoop_scsi_wdata,
    output wire                 snoop_scsi_wr,
    output wire                 snoop_scsi_rd,
    output wire [7:0]           snoop_scsi_rdata,
    output wire                 snoop_scsi_ack
);

    // ── SCSI pb_* face ──────────────────────────────────────────────────
    wire [8:0] scsi_addr;
    wire [7:0] scsi_wdata;
    wire       scsi_wr;
    wire       scsi_rd;
    wire [7:0] scsi_rdata;
    wire       scsi_ack;
    wire       scsi_dma_rd_ready;
    wire       scsi_dma_wr_ready;
    wire       scsi_dma16_lo_beat;

    assign snoop_scsi_addr  = scsi_addr;
    assign snoop_scsi_wdata = scsi_wdata;
    assign snoop_scsi_wr    = scsi_wr;
    assign snoop_scsi_rd    = scsi_rd;
    assign snoop_scsi_rdata = scsi_rdata;
    assign snoop_scsi_ack   = scsi_ack;

    // scsi.v now speaks the vhdd contract (rtl/vhdd.vh); vhdd_sd is the
    // provider that turns it back into the sd_ctrl-shaped interface this
    // harness mocks.  The two are instantiated side by side rather than
    // through tb/tb_scsi_vhdd_sd.v so that `u_scsi` stays the scsi
    // instance — tb_pb_scsi.cpp reaches into u_scsi's C96 state by
    // hierarchical name and an extra level of hierarchy would break it.
    wire [31:0] vh_chk_lba;
    wire [23:0] vh_chk_blocks;
    wire        vh_chk_ok;
    wire        vh_req_write;
    wire        vh_req_multi;
    wire [31:0] vh_req_lba;
    wire [15:0] vh_req_block_count;
    wire        vh_req_go;
    wire        vh_busy;
    wire        vh_done;
    wire        vh_error;
    wire        vh_rd_valid;
    wire [7:0]  vh_rd_data;
    wire        vh_rd_ready;
    wire        vh_wr_ready;
    wire        vh_wr_valid;
    wire        vh_wr_avail;
    wire [7:0]  vh_wr_data;

    scsi #(
        .TARGET_ID     (TARGET_ID),
        .TURBOSCSI_C96 (1'b1)
    ) u_scsi (
        // Write-protect is a vhdd_ctrl runtime setting (CTRL bit 2), not a
        // property of the SCSI target; these harnesses predate it and test
        // the writable behaviour, so tie it off.
        .wprot(1'b0),
        // Debug observability outputs (purely informational; this
        // harness does not exercise them).  Listed explicitly because
        // PINMISSING is fatal in this repo's Verilator config.
        .dbg_chk_ok            (),
        .dbg_xfer_blocks       (),
        .dbg_xfer_lba          (),
        .dbg_medium_not_present(),
        .dbg_sense_key         (),
        .dbg_sense_asc         (),
        .dbg_check_cond_count  (),
        .dbg_c96_state(),
        .vh_num_lbas   (disk_num_lbas),
        .clk           (clk),
        .rst           (rst || periph_rst),
        // Single-target build (TARGET_B_EN defaults to 0).
        .dev_en        (2'b01),

        .pb_addr       (scsi_addr),
        .pb_dma16_lo_beat(scsi_dma16_lo_beat),
        .pb_wdata      (scsi_wdata),
        .pb_wr         (scsi_wr),
        .pb_rd         (scsi_rd),
        .pb_rdata      (scsi_rdata),
        .pb_ack        (scsi_ack),

        .irq           (scsi_irq),
        .drq           (scsi_drq),

        .vh_chk_lba        (vh_chk_lba),
        .vh_chk_blocks     (vh_chk_blocks),
        .vh_chk_ok         (vh_chk_ok),
        .vh_req_write      (vh_req_write),
        .vh_req_multi      (vh_req_multi),
        .vh_req_lba        (vh_req_lba),
        .vh_req_block_count(vh_req_block_count),
        .vh_req_go         (vh_req_go),
        .vh_busy           (vh_busy),
        .vh_done           (vh_done),
        .vh_error          (vh_error),
        .vh_rd_valid       (vh_rd_valid),
        .vh_rd_data        (vh_rd_data),
        .vh_rd_ready       (vh_rd_ready),
        .vh_wr_ready       (vh_wr_ready),
        .vh_wr_valid       (vh_wr_valid),
        .vh_wr_avail       (vh_wr_avail),
        .vh_wr_data        (vh_wr_data),
        .vh_dev_sel        (),

        .scsi_ctrl_in   (scsi_ctrl_in),
        .dma_rd_ready   (scsi_dma_rd_ready),
        .dma_wr_ready   (scsi_dma_wr_ready)
    );

    vhdd_sd #(
        .RESERVED_LBAS(SD_LBA_BIAS)
    ) u_vhdd_sd (
        .num_lbas       (disk_num_lbas),
        .chk_lba        (vh_chk_lba),
        .chk_blocks     (vh_chk_blocks),
        .chk_ok         (vh_chk_ok),
        .req_write      (vh_req_write),
        .req_multi      (vh_req_multi),
        .req_lba        (vh_req_lba),
        .req_block_count(vh_req_block_count),
        .req_go         (vh_req_go),
        .busy           (vh_busy),
        .done           (vh_done),
        .error          (vh_error),
        .rd_valid       (vh_rd_valid),
        .rd_data        (vh_rd_data),
        .rd_ready       (vh_rd_ready),
        .wr_ready       (vh_wr_ready),
        .wr_valid       (vh_wr_valid),
        .wr_avail       (vh_wr_avail),
        .wr_data        (vh_wr_data),

        .sd_cmd_type    (sd_cmd_type),
        .sd_lba         (sd_lba),
        .sd_block_count (sd_block_count),
        .sd_go          (sd_go),
        .sd_busy        (sd_busy),
        .sd_done        (sd_done),
        .sd_error       (sd_error),
        .sd_rd_valid    (sd_rd_valid),
        .sd_rd_data     (sd_rd_data),
        .sd_rd_ready    (sd_rd_ready),
        .sd_wr_ready    (sd_wr_ready),
        .sd_wr_valid    (sd_wr_valid),
        .sd_wr_avail    (sd_wr_avail),
        .sd_wr_data     (sd_wr_data)
    );

    // ── VIA1 stand-in: tiny registered-ack register file ────────────────
    wire [3:0] via1_addr;
    wire [7:0] via1_wdata;
    wire       via1_wr;
    wire       via1_rd;
    reg  [7:0] via1_rdata_r;
    reg        via1_ack_r;
    reg  [7:0] via1_mem [0:15];
    integer    vi;
    always @(posedge clk) begin
        if (rst) begin
            for (vi = 0; vi < 16; vi = vi + 1) via1_mem[vi] <= 8'hA0 + vi[7:0];
            via1_rdata_r <= 8'h00;
            via1_ack_r   <= 1'b0;
        end else begin
            if (via1_wr) via1_mem[via1_addr] <= via1_wdata;
            if (via1_rd) via1_rdata_r <= via1_mem[via1_addr];
            via1_ack_r <= via1_wr | via1_rd;
        end
    end
    wire [7:0] via1_rdata = via1_rdata_r;
    wire       via1_ack   = via1_ack_r;

    // ── Everything else: trivial combinational-ack stubs ────────────────
    wire [3:0] via2_addr;  wire [7:0] via2_wdata;  wire via2_wr, via2_rd;
    wire [7:0] via2_rdata = 8'h00;
    wire       via2_ack   = via2_wr | via2_rd;

    wire [2:0] enet_addr;  wire [7:0] enet_wdata;  wire enet_wr, enet_rd;
    wire [7:0] enet_rdata = 8'h00;
    wire       enet_ack   = enet_wr | enet_rd;

    wire [5:0] sonic_addr; wire [15:0] sonic_wdata; wire [1:0] sonic_wstrb; wire sonic_wr, sonic_rd;
    wire [15:0] sonic_rdata = 16'h0000;
    wire        sonic_ack  = sonic_wr | sonic_rd;

    wire [7:0] orwell_addr; wire [7:0] orwell_wdata; wire orwell_wr, orwell_rd;
    wire [7:0] orwell_rdata = 8'h00;
    wire       orwell_ack   = orwell_wr | orwell_rd;

    wire [3:0] scc_addr;   wire [7:0] scc_wdata;   wire scc_wr, scc_rd;
    wire [7:0] scc_rdata  = 8'h00;
    wire       scc_ack    = scc_wr | scc_rd;

    wire [11:0] asc_addr;  wire [7:0] asc_wdata;   wire asc_wr, asc_rd;
    wire [7:0]  asc_rdata = 8'h00;
    wire        asc_ack   = asc_wr | asc_rd;

    wire [3:0] iwm_addr;   wire [7:0] iwm_wdata;   wire iwm_wr, iwm_rd;
    wire [7:0] iwm_rdata  = 8'h00;
    wire       iwm_ack    = iwm_wr | iwm_rd;

    wire [7:0] adbinj_addr; wire [7:0] adbinj_wdata; wire adbinj_wr, adbinj_rd;
    wire [7:0] adbinj_rdata = 8'h00;
    wire       adbinj_ack   = adbinj_wr | adbinj_rd;

    // ── debug_ctrl / DAFB AXI4-Lite: always-ready immediate-ack stubs ──
    wire dbg_awvalid, dbg_wvalid, dbg_bready, dbg_arvalid, dbg_rready;
    /* verilator lint_off UNUSEDSIGNAL */
    wire [19:0] dbg_awaddr, dbg_araddr;
    wire [31:0] dbg_wdata;
    wire [3:0]  dbg_wstrb;
    /* verilator lint_on UNUSEDSIGNAL */

    wire dafb_awvalid, dafb_wvalid, dafb_bready, dafb_arvalid, dafb_rready;
    /* verilator lint_off UNUSEDSIGNAL */
    wire [31:0] dafb_awaddr, dafb_araddr, dafb_wdata;
    wire [3:0]  dafb_wstrb;
    /* verilator lint_on UNUSEDSIGNAL */

    peripheral_bus #(
        .ID_WIDTH         (ID_WIDTH),
        .DATA_WIDTH       (128),
        .STRB_WIDTH       (16),
        .PB_WATCHDOG_LOG2 (PB_WATCHDOG_LOG2),
        .ENABLE_ACK_WATCHDOG (ENABLE_ACK_WATCHDOG)
    ) u_pb (
        .clk (clk),
        .rst (rst),
        // The bus is told about the peripherals' reset; it does NOT take it.
        .periph_rst (periph_rst),

        .s_awid    (s_awid),    .s_awaddr  (s_awaddr),  .s_awlen  (s_awlen),
        .s_awsize  (s_awsize),  .s_awburst (s_awburst), .s_awvalid(s_awvalid),
        .s_awready (s_awready),

        .s_wdata (s_wdata), .s_wstrb (s_wstrb), .s_wlast (s_wlast),
        .s_wvalid(s_wvalid), .s_wready(s_wready),

        .s_bid (s_bid), .s_bresp (s_bresp), .s_bvalid (s_bvalid), .s_bready (s_bready),

        .s_arid    (s_arid),    .s_araddr  (s_araddr),  .s_arlen  (s_arlen),
        .s_arsize  (s_arsize),  .s_arburst (s_arburst), .s_arvalid(s_arvalid),
        .s_arready (s_arready),

        .s_rid (s_rid), .s_rdata (s_rdata), .s_rresp (s_rresp),
        .s_rlast (s_rlast), .s_rvalid (s_rvalid), .s_rready (s_rready),

        .dbg_awaddr(dbg_awaddr), .dbg_awvalid(dbg_awvalid), .dbg_awready(1'b1),
        .dbg_wdata(dbg_wdata), .dbg_wstrb(dbg_wstrb), .dbg_wvalid(dbg_wvalid), .dbg_wready(1'b1),
        .dbg_bresp(2'b00), .dbg_bvalid(dbg_awvalid | dbg_wvalid), .dbg_bready(dbg_bready),
        .dbg_araddr(dbg_araddr), .dbg_arvalid(dbg_arvalid), .dbg_arready(1'b1),
        .dbg_rdata(32'h0), .dbg_rresp(2'b00), .dbg_rvalid(dbg_arvalid), .dbg_rready(dbg_rready),

        .dafb_awaddr(dafb_awaddr), .dafb_awvalid(dafb_awvalid), .dafb_awready(1'b1),
        .dafb_wdata(dafb_wdata), .dafb_wstrb(dafb_wstrb), .dafb_wvalid(dafb_wvalid), .dafb_wready(1'b1),
        .dafb_bresp(2'b00), .dafb_bvalid(dafb_awvalid | dafb_wvalid), .dafb_bready(dafb_bready),
        .dafb_araddr(dafb_araddr), .dafb_arvalid(dafb_arvalid), .dafb_arready(1'b1),
        .dafb_rdata(32'h0), .dafb_rresp(2'b00), .dafb_rvalid(dafb_arvalid), .dafb_rready(dafb_rready),

        .via1_addr(via1_addr), .via1_wdata(via1_wdata), .via1_wr(via1_wr), .via1_rd(via1_rd),
        .via1_rdata(via1_rdata), .via1_ack(via1_ack),

        .via2_addr(via2_addr), .via2_wdata(via2_wdata), .via2_wr(via2_wr), .via2_rd(via2_rd),
        .via2_rdata(via2_rdata), .via2_ack(via2_ack),

        .enet_addr(enet_addr), .enet_wdata(enet_wdata), .enet_wr(enet_wr), .enet_rd(enet_rd),
        .enet_rdata(enet_rdata), .enet_ack(enet_ack),

        .sonic_addr(sonic_addr), .sonic_wdata(sonic_wdata), .sonic_wr(sonic_wr), .sonic_rd(sonic_rd),
        .sonic_wstrb(sonic_wstrb),
        .sonic_rdata(sonic_rdata), .sonic_ack(sonic_ack),

        .orwell_addr(orwell_addr), .orwell_wdata(orwell_wdata), .orwell_wr(orwell_wr), .orwell_rd(orwell_rd),
        .orwell_rdata(orwell_rdata), .orwell_ack(orwell_ack),

        .scc_addr(scc_addr), .scc_wdata(scc_wdata), .scc_wr(scc_wr), .scc_rd(scc_rd),
        .scc_rdata(scc_rdata), .scc_ack(scc_ack),

        .scsi_addr(scsi_addr), .scsi_wdata(scsi_wdata), .scsi_wr(scsi_wr), .scsi_rd(scsi_rd),
        .scsi_rdata(scsi_rdata), .scsi_ack(scsi_ack),
        .scsi_dma_rd_ready(scsi_dma_rd_ready), .scsi_dma_wr_ready(scsi_dma_wr_ready),
        .scsi_dma16_lo_beat(scsi_dma16_lo_beat),

        .asc_addr(asc_addr), .asc_wdata(asc_wdata), .asc_wr(asc_wr), .asc_rd(asc_rd),
        .asc_rdata(asc_rdata), .asc_ack(asc_ack),

        .iwm_addr(iwm_addr), .iwm_wdata(iwm_wdata), .iwm_wr(iwm_wr), .iwm_rd(iwm_rd),
        .iwm_rdata(iwm_rdata), .iwm_ack(iwm_ack),

        .adbinj_addr(adbinj_addr), .adbinj_wdata(adbinj_wdata), .adbinj_wr(adbinj_wr), .adbinj_rd(adbinj_rd),
        .adbinj_rdata(adbinj_rdata), .adbinj_ack(adbinj_ack)
    );

endmodule
