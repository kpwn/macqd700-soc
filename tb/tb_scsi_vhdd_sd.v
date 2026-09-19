// tb_scsi_vhdd_sd.v — SCSI target + SD virtual-HDD provider, composed.
//
// Why this wrapper exists
// ═══════════════════════
// scsi.v used to expose the sd_ctrl-shaped backing-store interface on its
// own ports and to instantiate sd_scsi_lba_mapper internally.  All of the
// SCSI unit testbenches (tb_scsi.cpp, tb_turboscsi.cpp, the tb_scsi_c96_*
// family, tools/mame_scsi_bridge.cpp) mock an SD card against exactly that
// interface: they watch sd_go/sd_cmd_type/sd_lba, check that a READ(6) at
// SCSI LBA 0 arrives as SD sector 8192, and feed bytes back.
//
// The vhdd refactor moved the SD encoding and the reserved-window bias out
// of scsi.v and down into vhdd_sd.v, so that pair is now what presents the
// SD-shaped interface.  This wrapper composes them and re-exposes exactly
// the port names the pre-refactor `scsi` module had — which is what lets
// every one of those testbenches keep its assertions verbatim, so a green
// run is real evidence that the refactor changed nothing observable at the
// SD seam.
//
// It is a testbench harness, not RTL: fpga_top instantiates scsi and
// vhdd_sd directly (rtl/soc/fpga_top_peripherals.vh).

`default_nettype none

module tb_scsi_vhdd_sd #(
    parameter [2:0]  TARGET_ID      = 3'd0,
    parameter [31:0] SD_LBA_BIAS    = 32'd8192,
    parameter        TURBOSCSI_C96  = 1'b0,
    // ── Read-ahead cache (rtl/soc/vhdd_readahead.v) ──────────────────
    // DEFAULT 0, and that is load-bearing.  Every SCSI harness in the
    // tree asserts on the SD-side traffic -- "a READ(6) at SCSI LBA 0
    // arrives as SD sector 8192, as a CMD17, for one block".  Read-ahead
    // deliberately changes that traffic (one CMD18 for a run), so turning
    // it on by default would rewrite the expectations of a dozen
    // testbenches at once and destroy their value as a regression fence.
    //
    // At 0 the module elaborates its ENABLE=0 arm, which is pure wires --
    // so every existing harness still runs THROUGH it and proves the
    // bypass really is transparent, without changing a single assertion.
    parameter integer READAHEAD     = 0,
    parameter integer RA_BLOCKS     = 8
) (
    input  wire        clk,
    // vhdd CTRL[2] equivalent: lock the volume so WRITE commands are
    // refused with CHECK CONDITION / DATA PROTECT.
    input  wire        wprot,
    input  wire        rst,
    input  wire [31:0] disk_num_lbas,
    // ── Peripheral-bus slave ─────────────────────────────────────────
    input  wire [8:0]  pb_addr,
    // Second half of a host 16-bit pseudo-DMA access, replayed as two
    // byte beats.  In the SoC peripheral_bus.v drives this; here the C++
    // harness does, so tb_scsi_fuzz.cpp's DR16/DW16 ops can exercise
    // scsi.v's c96_dma16_hi_granted DRQ-grant carry WITHOUT dragging the
    // whole fabric in.  Every other harness leaves it at 0 (Verilator
    // zero-initialises top-level inputs), which is the historical shape.
    input  wire        pb_dma16_lo_beat,
    input  wire [7:0]  pb_wdata,
    input  wire        pb_wr,
    input  wire        pb_rd,
    output wire [7:0]  pb_rdata,
    output wire        pb_ack,
    // ── IRQ / DRQ lines ──────────────────────────────────────────────
    output wire        irq,
    output wire        drq,
    // ── SD backing store (sd_ctrl-shaped, below vhdd_sd) ─────────────
    output wire [2:0]  sd_cmd_type,
    output wire [31:0] sd_lba,
    output wire [15:0] sd_block_count,
    output wire        sd_go,
    input  wire        sd_busy,
    input  wire        sd_done,
    input  wire        sd_error,
    input  wire        sd_rd_valid,
    input  wire [7:0]  sd_rd_data,
    output wire        sd_rd_ready,
    input  wire        sd_wr_ready,
    output wire        sd_wr_valid,
    output wire [7:0]  sd_wr_data,
    output wire        sd_wr_avail,
    // ── TurboSCSI shim ───────────────────────────────────────────────
    input  wire [8:0]  scsi_ctrl_in,
    output wire        dma_rd_ready,
    output wire        dma_wr_ready
);

    // ══ Build-time shape selection ════════════════════════════════════
    //
    // DEFAULT (no defines): the single-target shape this harness has
    // always had — TARGET_B_EN=0, dev_en=2'b01, scsi.v wired STRAIGHT to
    // vhdd_sd with no vhdd_mux.  Every existing tb that instantiates this
    // wrapper (tb_scsi, tb_turboscsi, the tb_scsi_c96_* family,
    // tb_scsi_sd_e2e, tools/mame_scsi_bridge) keeps that shape verbatim.
    //
    // `SCSI_TB_DUAL_TARGET selects the shape rtl/soc/fpga_top_peripherals.vh
    // ACTUALLY BUILDS: TARGET_B_EN=1 (two target IDs off one FSM) with
    // vhdd_mux inserted on the vhdd seam.  That configuration had ZERO
    // testbench coverage — every SCSI tb in the tree exercised the
    // single-target shape, which is not what ships — so a regression in
    // the dual-target path could not have been caught by `make tb-all`.
    //
    // `SCSI_TB_DEV_EN overrides the device-enable mask.  It exists so a
    // build can turn the SD target OFF (2'b00) and prove the harness
    // actually FAILS when the target is absent — the positive control
    // without which a green dual-target run means nothing.
`ifdef SCSI_TB_DUAL_TARGET
    localparam       TGT_B_EN = 1'b1;
    localparam       USE_MUX  = 1'b1;
`else
    localparam       TGT_B_EN = 1'b0;
    localparam       USE_MUX  = 1'b0;
`endif
    localparam [2:0] TGT_ID_B = 3'd1;
`ifdef SCSI_TB_DEV_EN
    localparam [1:0] DEV_EN = `SCSI_TB_DEV_EN;
`else
    localparam [1:0] DEV_EN = 2'b01;
`endif

    // ── vhdd seam: master face (scsi.v side) ──────────────────────────
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
    wire [7:0]  vh_wr_data;
    wire        vh_wr_avail;
    wire        vh_dev_sel;
    wire [31:0] vh_num_lbas;

    // ── vhdd seam: provider-A face (vhdd_sd side) ─────────────────────
    // Identical to the master face when USE_MUX=0 (wired straight
    // through below); routed by vhdd_mux when USE_MUX=1.
    wire [31:0] va_chk_lba;
    wire [23:0] va_chk_blocks;
    wire        va_chk_ok;
    wire        va_req_write;
    wire        va_req_multi;
    wire [31:0] va_req_lba;
    wire [15:0] va_req_block_count;
    wire        va_req_go;
    wire        va_busy;
    wire        va_done;
    wire        va_error;
    wire        va_rd_valid;
    wire [7:0]  va_rd_data;
    wire        va_rd_ready;
    wire        va_wr_ready;
    wire        va_wr_valid;
    wire [7:0]  va_wr_data;
    wire        va_wr_avail;

    scsi #(
        .TARGET_ID     (TARGET_ID),
        .TARGET_ID_B   (TGT_ID_B),
        .TARGET_B_EN   (TGT_B_EN),
        .TURBOSCSI_C96 (TURBOSCSI_C96)
    ) u_scsi (
        .wprot(wprot),
        // debug-only observability (rtl/mac/scsi.v); unconnected here
        .dbg_chk_ok(), .dbg_xfer_blocks(), .dbg_xfer_lba(),
        .dbg_medium_not_present(), .dbg_sense_key(),
        .dbg_sense_asc(), .dbg_check_cond_count(),
        .dbg_c96_state(),
        .clk           (clk),
        .rst           (rst),
        // Default build: only target A is live and TARGET_B_EN=0, so this
        // harness is byte-identical to the pre-dual-target scsi.v.  Under
        // `SCSI_TB_DUAL_TARGET it is fpga_top's shape instead.
        .dev_en        (DEV_EN),
        .vh_num_lbas   (vh_num_lbas),
        .pb_addr       (pb_addr),
        // No peripheral_bus in this harness: the C++ side supplies the
        // split-word flag itself (0 for every standalone byte beat, so
        // each performs its own DRQ check — the historical shape).
        .pb_dma16_lo_beat(pb_dma16_lo_beat),
        .pb_wdata      (pb_wdata),
        .pb_wr         (pb_wr),
        .pb_rd         (pb_rd),
        .pb_rdata      (pb_rdata),
        .pb_ack        (pb_ack),
        .irq           (irq),
        .drq           (drq),
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
        .vh_wr_data        (vh_wr_data),
        .vh_wr_avail       (vh_wr_avail),
        .vh_dev_sel        (vh_dev_sel),
        .scsi_ctrl_in  (scsi_ctrl_in),
        .dma_rd_ready  (dma_rd_ready),
        .dma_wr_ready  (dma_wr_ready)
    );

    // ══ Provider routing ══════════════════════════════════════════════
    generate
    if (USE_MUX) begin : g_vhdd_mux
        // fpga_top's shape.  Provider B (SCSI ID 1 in the real SoC) is
        // tied off here: this harness has no AXI fabric to hang it on,
        // and with dev_en[1]=0 the master can never select it, so an
        // absent B is exactly what the shipping power-on state models.
        // Anything that made B's tie-off observable at the A face would
        // therefore be a routing bug in vhdd_mux — which is the point.
        vhdd_mux u_vhdd_mux (
            .dev_sel          (vh_dev_sel),
            .m_num_lbas       (vh_num_lbas),
            .m_chk_lba        (vh_chk_lba),
            .m_chk_blocks     (vh_chk_blocks),
            .m_chk_ok         (vh_chk_ok),
            .m_req_write      (vh_req_write),
            .m_req_multi      (vh_req_multi),
            .m_req_lba        (vh_req_lba),
            .m_req_block_count(vh_req_block_count),
            .m_req_go         (vh_req_go),
            .m_busy           (vh_busy),
            .m_done           (vh_done),
            .m_error          (vh_error),
            .m_rd_valid       (vh_rd_valid),
            .m_rd_data        (vh_rd_data),
            .m_rd_ready       (vh_rd_ready),
            .m_wr_ready       (vh_wr_ready),
            .m_wr_valid       (vh_wr_valid),
            .m_wr_data        (vh_wr_data),
            .m_wr_avail       (vh_wr_avail),

            .a_num_lbas       (disk_num_lbas),
            .a_chk_lba        (va_chk_lba),
            .a_chk_blocks     (va_chk_blocks),
            .a_chk_ok         (va_chk_ok),
            .a_req_write      (va_req_write),
            .a_req_multi      (va_req_multi),
            .a_req_lba        (va_req_lba),
            .a_req_block_count(va_req_block_count),
            .a_req_go         (va_req_go),
            .a_busy           (va_busy),
            .a_done           (va_done),
            .a_error          (va_error),
            .a_rd_valid       (va_rd_valid),
            .a_rd_data        (va_rd_data),
            .a_rd_ready       (va_rd_ready),
            .a_wr_ready       (va_wr_ready),
            .a_wr_valid       (va_wr_valid),
            .a_wr_data        (va_wr_data),
            .a_wr_avail       (va_wr_avail),

            .b_num_lbas       (32'd0),
            /* verilator lint_off PINCONNECTEMPTY */
            .b_chk_lba        (),
            .b_chk_blocks     (),
            .b_req_write      (),
            .b_req_multi      (),
            .b_req_lba        (),
            .b_req_block_count(),
            .b_req_go         (),
            .b_rd_ready       (),
            .b_wr_valid       (),
            .b_wr_data        (),
            .b_wr_avail       (),
            /* verilator lint_on PINCONNECTEMPTY */
            .b_chk_ok         (1'b0),
            .b_busy           (1'b0),
            .b_done           (1'b0),
            .b_error          (1'b0),
            .b_rd_valid       (1'b0),
            .b_rd_data        (8'h00),
            .b_wr_ready       (1'b0)
        );
    end else begin : g_vhdd_direct
        // Pre-dual-target shape: scsi.v straight onto vhdd_sd.
        assign vh_num_lbas  = disk_num_lbas;
        assign va_chk_lba         = vh_chk_lba;
        assign va_chk_blocks      = vh_chk_blocks;
        assign vh_chk_ok          = va_chk_ok;
        assign va_req_write       = vh_req_write;
        assign va_req_multi       = vh_req_multi;
        assign va_req_lba         = vh_req_lba;
        assign va_req_block_count = vh_req_block_count;
        assign va_req_go          = vh_req_go;
        assign vh_busy            = va_busy;
        assign vh_done            = va_done;
        assign vh_error           = va_error;
        assign vh_rd_valid        = va_rd_valid;
        assign vh_rd_data         = va_rd_data;
        assign va_rd_ready        = vh_rd_ready;
        assign vh_wr_ready        = va_wr_ready;
        assign va_wr_valid        = vh_wr_valid;
        assign va_wr_data         = vh_wr_data;
        assign va_wr_avail        = vh_wr_avail;
    end
    endgenerate

    // ── Read-ahead cache, exactly where fpga_top puts it ──────────────
    // Between the mux's A port and vhdd_sd.  See the READAHEAD parameter
    // comment for why it defaults to built-out.
    wire [31:0] vc_chk_lba;
    wire [23:0] vc_chk_blocks;
    wire        vc_chk_ok;
    wire        vc_req_write, vc_req_multi, vc_req_go;
    wire [31:0] vc_req_lba;
    wire [15:0] vc_req_block_count;
    wire        vc_busy, vc_done, vc_error;
    wire        vc_rd_valid;
    wire [7:0]  vc_rd_data;
    wire        vc_rd_ready;
    wire        vc_wr_ready, vc_wr_valid;
    wire [7:0]  vc_wr_data;
    wire        vc_wr_avail;

    vhdd_readahead #(
        .BLOCKS_PER_WAY(RA_BLOCKS),
        .ENABLE        (READAHEAD)
    ) u_vhdd_readahead (
        .clk              (clk),
        .rst              (rst),
        .inval            (1'b0),
        .num_lbas         (disk_num_lbas),
        .chk_lba          (va_chk_lba),
        .chk_blocks       (va_chk_blocks),
        .chk_ok           (va_chk_ok),
        .req_write        (va_req_write),
        .req_multi        (va_req_multi),
        .req_lba          (va_req_lba),
        .req_block_count  (va_req_block_count),
        .req_go           (va_req_go),
        .busy             (va_busy),
        .done             (va_done),
        .error            (va_error),
        .rd_valid         (va_rd_valid),
        .rd_data          (va_rd_data),
        .rd_ready         (va_rd_ready),
        .wr_ready         (va_wr_ready),
        .wr_valid         (va_wr_valid),
        .wr_data          (va_wr_data),
        .wr_avail         (va_wr_avail),
        .p_chk_lba        (vc_chk_lba),
        .p_chk_blocks     (vc_chk_blocks),
        .p_chk_ok         (vc_chk_ok),
        .p_req_write      (vc_req_write),
        .p_req_multi      (vc_req_multi),
        .p_req_lba        (vc_req_lba),
        .p_req_block_count(vc_req_block_count),
        .p_req_go         (vc_req_go),
        .p_busy           (vc_busy),
        .p_done           (vc_done),
        .p_error          (vc_error),
        .p_rd_valid       (vc_rd_valid),
        .p_rd_data        (vc_rd_data),
        .p_rd_ready       (vc_rd_ready),
        .p_wr_ready       (vc_wr_ready),
        .p_wr_valid       (vc_wr_valid),
        .p_wr_data        (vc_wr_data),
        .p_wr_avail       (vc_wr_avail)
    );

    vhdd_sd #(
        .RESERVED_LBAS(SD_LBA_BIAS)
    ) u_vhdd_sd (
        .num_lbas       (disk_num_lbas),
        .chk_lba        (vc_chk_lba),
        .chk_blocks     (vc_chk_blocks),
        .chk_ok         (vc_chk_ok),
        .req_write      (vc_req_write),
        .req_multi      (vc_req_multi),
        .req_lba        (vc_req_lba),
        .req_block_count(vc_req_block_count),
        .req_go         (vc_req_go),
        .busy           (vc_busy),
        .done           (vc_done),
        .error          (vc_error),
        .rd_valid       (vc_rd_valid),
        .rd_data        (vc_rd_data),
        .rd_ready       (vc_rd_ready),
        .wr_ready       (vc_wr_ready),
        .wr_valid       (vc_wr_valid),
        .wr_data        (vc_wr_data),
        .wr_avail       (vc_wr_avail),
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
        .sd_wr_data     (sd_wr_data),
        .sd_wr_avail    (sd_wr_avail)
    );

endmodule

`default_nettype wire
