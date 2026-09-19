// vhdd_ctrl.v — AXI4-Lite control/status registers for the vHDD volumes
//
// Purpose
// ═══════
// The host-visible knobs for the two SCSI volumes behind the vhdd seam
// (rtl/vhdd.vh): which of them is present on the bus, and how big the
// DDR-backed RAM disk is.  Driven from JTAG-AXI by `vhdd-status`,
// `vhdd-enable`, `ramdisk-size` and friends in tools/jtag_repl.tcl.
//
// WHERE IT LIVES, AND WHY THERE
// ═════════════════════════════
// It replaces the `axil_null_slave` terminator on xbar S2, the 1 MB
// "DMA config" window at 0x5010_0000.  That window has had NO consumer
// since dma_ctrl was removed (rtl/soc/fpga_top_dma.vh, 2026-08-01): every
// access landed in a terminator that read 0.  Reusing it costs zero
// changes to the crossbar's slave count, slot numbering, decode, or to
// any master's view of the address map — all of which are load-bearing
// and none of which is worth disturbing for eight registers.
//
// The terminator behaviour is PRESERVED, not replaced: any offset this
// module does not decode still reads 0 and still accepts writes with
// BRESP=OKAY.  That property is the whole reason the null slave existed
// (this SoC has a documented history of wild accesses landing in
// 0x5xxx_xxxx — docs/diag-bus-fault-51001c00.md) and losing it would turn
// one of those into an xbar wedge.
//
// Register map (byte offsets from the window base, 0x5010_0000)
// ═════════════════════════════════════════════════════════════
//   0x000  IDENT          RO  0x5D0D0001 — presence/version probe.  Host
//                             tooling MUST check this before trusting any
//                             other offset: on a bitstream without this
//                             module the same address reads 0.
//   0x004  CTRL           RW  [0] SD volume enabled
//                             [1] RAM-disk volume enabled
//                             [2] SD volume WRITE-PROTECTED.  The SCSI
//                             target answers WRITE(6)/WRITE(10) with
//                             CHECK CONDITION / DATA PROTECT (sense key
//                             7, ASC 0x27) instead of touching the card,
//                             so the guest sees a locked volume and the
//                             image cannot be modified.  Reset 0 (RW).
//                             reset CTRL_RESET.  Other bits RAZ/WI.
//                             NOTE: the module default is 0x3 (both live)
//                             but fpga_top OVERRIDES it to 0x1 — SD only
//                             at power-on (fpga_top_dma.vh:105-117).  A
//                             JTAG `vhdd-enable` does NOT survive reset.
//   0x008  RD_BLOCKS      RW  RAM-disk size, in 512-byte blocks.
//                             reset RD_BLOCKS_RESET (32 MB).  Writes are
//                             CLAMPED to RD_MAX_BLOCKS here, so the value
//                             read back is always the value in force.
//   0x00C  SD_BLOCKS      RO  the SD volume's block count as the platform
//                             computed it (card sectors - reserved window,
//                             clamped) — visibility only.
//   0x010  STATUS         RO  [0] SD enabled          (mirrors CTRL[0])
//                             [1] RAM-disk enabled    (mirrors CTRL[1])
//                             [2] RAM-disk busy
//                             [3] RAM-disk last-request error
//                             [7:4] RAM-disk FSM state
//                             [8] SD volume write-protected (mirrors CTRL[2])
//                             [31:16] RAM-disk watchdog fires (saturating)
//   0x014  RD_MAX_BLOCKS  RO  compile-time ceiling (aperture / 512)
//   0x018  RD_APERTURE    RO  base address of the RAM-disk AXI aperture.
//                             Exposed so host tooling reads it instead of
//                             hardcoding it — a future rebase of the
//                             aperture would otherwise silently corrupt
//                             whatever the host uploaded.
//   0x01C  RD_WDOG_FIRES  RO  same counter as STATUS[31:16], full width,
//                             for tooling that wants it unpacked.
//
// Clock domain
// ════════════
// core_clk only.  `dev_en` and `rd_num_lbas` are consumed in the pb_clk
// domain (scsi.v and whatever provider holds SCSI ID 1) and the status
// inputs are produced there;
// ALL of that CDC lives in rtl/soc/fpga_top_peripherals.vh alongside the
// synchronisers that file already has for dafb_scsi0_ctrl / scsi_irq /
// scsi_drq, rather than being smuggled into this register file.  Keeping
// this module single-clock is what makes it unit-testable.
//
// Latency: one outstanding write and one outstanding read; BVALID rises
// the cycle after the second of (AW, W) lands, RVALID the cycle after AR.
// AW and W are tracked INDEPENDENTLY — axi_wide_to_axilite drives them
// from separate flags and can present them in either order.

`default_nettype none

module vhdd_ctrl #(
    parameter ADDR_WIDTH      = 20,
    parameter [31:0] IDENT_VALUE     = 32'h5D0D_0001,
    // Base of the RAM-disk AXI aperture, reported at offset 0x018.
    parameter [31:0] RD_APERTURE     = 32'h7000_0000,
    // Compile-time ceiling on the RAM disk, in 512-byte blocks.
    // 524288 * 512 = 256 MiB.
    parameter [31:0] RD_MAX_BLOCKS   = 32'd524288,
    // Power-on RAM-disk size, in 512-byte blocks.  65536 * 512 = 32 MiB.
    parameter [31:0] RD_BLOCKS_RESET = 32'd65536,
    // Power-on device-enable mask.  Module default is "both volumes
    // live"; the ONE instantiation that ships (fpga_top_dma.vh:117)
    // overrides it to 2'b01 (SD only) so the unformatted RAM disk is not
    // in the ROM's boot scan.  Read that comment before changing this.
    parameter [2:0]  CTRL_RESET      = 3'b011
) (
    input  wire                     clk,
    input  wire                     rst,
    // Reset for the CTRL (dev_en) register ONLY -- deliberately a
    // different domain from `rst`.  See the ctrl_q always block below.
    input  wire                     cfg_rst,

    // ── AXI4-Lite slave ───────────────────────────────────────────────
    input  wire [ADDR_WIDTH-1:0]    awaddr,
    input  wire                     awvalid,
    output wire                     awready,
    input  wire [31:0]              wdata,
    input  wire [3:0]               wstrb,
    input  wire                     wvalid,
    output wire                     wready,
    output wire [1:0]               bresp,
    output wire                     bvalid,
    input  wire                     bready,
    input  wire [ADDR_WIDTH-1:0]    araddr,
    input  wire                     arvalid,
    output wire                     arready,
    output reg  [31:0]              rdata,
    output wire [1:0]               rresp,
    output wire                     rvalid,
    input  wire                     rready,

    // ── Control outputs (core_clk; CDC'd to pb_clk by the caller) ─────
    // ── net-vHDD endpoint (JTAG-loadable; see docs/net_vhdd_design.md) ─
    output wire [47:0]              net_our_mac,
    output wire [31:0]              net_our_ip,
    output wire [15:0]              net_our_port,
    output wire [47:0]              net_dst_mac,
    output wire [31:0]              net_dst_ip,
    output wire [15:0]              net_dst_port,

    output wire [1:0]               dev_en,        // [0]=SD, [1]=RAM disk
    output wire                     sd_wprot,      // CTRL[2]: SD volume locked
    output wire [31:0]              rd_num_lbas,   // RAM-disk size, blocks

    // ── Status inputs (already synchronised into core_clk) ────────────
    input  wire [31:0]              sd_num_lbas,
    input  wire                     rd_busy,
    input  wire                     rd_error,
    input  wire [3:0]               rd_state,
    input  wire [15:0]              rd_wdog_fires,

    // ── 53C96 trace-ring readout (rtl/soc/scsi_trace_ring.v) ──────────
    // Parked in this register block rather than in a slave of its own
    // because it is already an AXI-Lite terminator on core_clk with 1 MB
    // of address space and only 32 bytes used; adding a slave to the xbar
    // for a debug ring would be a far larger change for no benefit.
    output reg  [11:0]              trace_rd_addr,
    input  wire [31:0]              trace_rd_data,
    input  wire [11:0]              trace_wrptr,
    input  wire                     trace_wrapped,
    input  wire                     trace_frozen,
    output reg                      trace_freeze,
    output reg                      trace_clear      // TOGGLE, not a pulse
);

    // ── Register offsets (word-aligned, low 6 bits of the window) ─────
    localparam [6:0] OFF_IDENT         = 7'h00,
                     OFF_CTRL          = 7'h04,
                     OFF_RD_BLOCKS     = 7'h08,
                     OFF_SD_BLOCKS     = 7'h0C,
                     OFF_STATUS        = 7'h10,
                     OFF_RD_MAX_BLOCKS = 7'h14,
                     OFF_RD_APERTURE   = 7'h18,
                     OFF_RD_WDOG_FIRES = 7'h1C,
                     // ── net-vHDD endpoint (see docs/net_vhdd_design.md) ──
                     // The Ethernet-backed provider has no ARP, so both
                     // endpoints are configured rather than discovered; the
                     // destination is the NEXT HOP, which is exactly what ARP
                     // would have resolved.  They live here rather than in a
                     // new AXI slave because this is already the vhdd
                     // configuration block and already JTAG-reachable.
                     OFF_NET_MAC_LO    = 7'h40,
                     OFF_NET_MAC_HI    = 7'h44,
                     OFF_NET_IP        = 7'h48,
                     OFF_NET_DST_MAC_LO= 7'h4C,
                     OFF_NET_DST_MAC_HI= 7'h50,
                     OFF_NET_DST_IP    = 7'h54,
                     OFF_NET_PORTS     = 7'h58,
    // Trace ring.  TRACE_CTRL bit0 = freeze (level, sticky in the ring
    // until cleared), bit1 = clear+re-arm (writing 1 flips the toggle the
    // ring edge-detects).  Reads give {frozen, wrapped, wr_ptr}.
    // TRACE_ADDR sets the RAM index; TRACE_DATA returns ring[TRACE_ADDR].
    // TRACE_DATA does NOT auto-increment: the RAM read is registered, so
    // an auto-increment would return the PREVIOUS index's word and every
    // dump would be off by one -- a silent, plausible-looking corruption.
    // The host writes the address then reads the data, one pair per entry
    // (tools/jtag_repl.tcl `scsi-trace`).
                     OFF_TRACE_CTRL    = 7'h20,
                     OFF_TRACE_ADDR    = 7'h24,
                     OFF_TRACE_DATA    = 7'h28;

    reg [2:0]  ctrl_q;
    reg [31:0] rd_blocks_q;

    assign dev_en      = ctrl_q[1:0];
    assign sd_wprot    = ctrl_q[2];
    assign rd_num_lbas = rd_blocks_q;

    // ── Write channel ─────────────────────────────────────────────────
    reg                  aw_seen;
    reg                  w_seen;
    reg [ADDR_WIDTH-1:0] aw_addr_q;
    reg [31:0]           w_data_q;
    reg [3:0]            w_strb_q;
    reg                  bvalid_r;
    reg                  rvalid_r;

    assign awready = !aw_seen && !bvalid_r;
    assign wready  = !w_seen  && !bvalid_r;
    assign bvalid  = bvalid_r;
    assign bresp   = 2'b00;                 // always OKAY (terminator role)
    assign arready = !rvalid_r;
    assign rvalid  = rvalid_r;
    assign rresp   = 2'b00;

    // The address/data/strobes being committed.  `wr_commit` can fire on
    // the very cycle the second half lands, in which case the latched
    // copies are still stale — so every commit-time expression must pick
    // the live port value in that case, not just the address.  (Getting
    // this wrong is silent: the write still completes with BRESP=OKAY,
    // it just stores the PREVIOUS write's data.)
    wire [ADDR_WIDTH-1:0] commit_addr = aw_seen ? aw_addr_q : awaddr;
    // Only the bottom 32 bytes of the 1 MB window are registers.
    // EVERYTHING else keeps the axil_null_slave behaviour it replaced:
    // reads 0, writes accepted and dropped with BRESP=OKAY.  It must NOT
    // mirror — a wild write to 0x5010_5004 flipping CTRL would be a much
    // worse failure than the wedge the terminator exists to prevent, and
    // this SoC has a documented history of wild 0x5xxx_xxxx accesses
    // (docs/diag-bus-fault-51001c00.md).
    wire [31:0]           commit_data = w_seen  ? w_data_q  : wdata;
    wire [3:0]            commit_strb = w_seen  ? w_strb_q  : wstrb;
    // Widened from 5 to 6 bits when the trace-ring registers landed at
    // 0x20..0x28.  Everything at or above 0x40 keeps the terminator
    // behaviour described above.
    wire commit_in_regs = (commit_addr[ADDR_WIDTH-1:7] == {(ADDR_WIDTH-7){1'b0}});
    wire ar_in_regs     = (araddr     [ADDR_WIDTH-1:7] == {(ADDR_WIDTH-7){1'b0}});

    // net-vHDD endpoint registers.  our_mac RESETS TO ZERO deliberately: the
    // MAC-sharing demux treats an all-zero block MAC as "client absent" and
    // excludes it from both exact match and broadcast fan-out.  A non-zero
    // default would make an UNCONFIGURED board start claiming frames from the
    // SONIC, and -- because the absent client's tready is strapped -- fanning
    // a broadcast at it would hold the frame forever and starve the wire.
    // Absent-until-configured is the only safe power-on state.
    reg [47:0] net_mac_q, net_dst_mac_q;
    reg [31:0] net_ip_q, net_dst_ip_q;
    reg [15:0] net_port_q, net_dst_port_q;

    assign net_our_mac  = net_mac_q;
    assign net_our_ip   = net_ip_q;
    assign net_our_port = net_port_q;
    assign net_dst_mac  = net_dst_mac_q;
    assign net_dst_ip   = net_dst_ip_q;
    assign net_dst_port = net_dst_port_q;

    // Byte-merge: apply wstrb to the existing value, so a byte-granular
    // write from the AXI-Lite bridge cannot clobber the other bytes.
    // CTRL only has bits [1:0], so only strobe 0 can touch it.
    wire [2:0] merged_ctrl = commit_strb[0] ? commit_data[2:0] : ctrl_q;

    wire [31:0] merged_rd_blocks =
        {commit_strb[3] ? commit_data[31:24] : rd_blocks_q[31:24],
         commit_strb[2] ? commit_data[23:16] : rd_blocks_q[23:16],
         commit_strb[1] ? commit_data[15:8]  : rd_blocks_q[15:8],
         commit_strb[0] ? commit_data[7:0]   : rd_blocks_q[7:0]};

    // Same byte-merge for the net-vHDD endpoint registers.  A function
    // rather than another hand-written concat per register: six of them
    // would be six chances to transpose a strobe index.
    function automatic [31:0] merge32;
        input [31:0] old_value;
        begin
            merge32 = {commit_strb[3] ? commit_data[31:24] : old_value[31:24],
                       commit_strb[2] ? commit_data[23:16] : old_value[23:16],
                       commit_strb[1] ? commit_data[15:8]  : old_value[15:8],
                       commit_strb[0] ? commit_data[7:0]   : old_value[7:0]};
        end
    endfunction

    // Materialise the merged values as wires.  A part-select applied
    // DIRECTLY to a function call -- merge32(...)[15:0] -- violates IEEE
    // 1800 and Vivado rejects it outright (Synth 8-12513), even though our
    // lint accepts it happily.  Only a synthesis run catches this class.
    wire [31:0] merged_net_mac_hi     = merge32({16'd0, net_mac_q[47:32]});
    wire [31:0] merged_net_dst_mac_hi = merge32({16'd0, net_dst_mac_q[47:32]});
    wire [31:0] merged_net_ports      = merge32({net_port_q, net_dst_port_q});

    // Clamp at the register, not at the point of use: whatever the host
    // reads back at 0x008 is then always the value actually in force.
    wire [31:0] clamped_rd_blocks =
        (merged_rd_blocks > RD_MAX_BLOCKS) ? RD_MAX_BLOCKS : merged_rd_blocks;

    // Both halves of the write are in — this cycle or earlier.
    wire wr_commit = !bvalid_r &&
                     (aw_seen || (awvalid && awready)) &&
                     (w_seen  || (wvalid  && wready ));

    // ── CTRL / dev_en: sticky across the resets used to reboot from JTAG ──
    // This register is driven from `cfg_rst`, NOT the module's `rst`.
    //
    // Why: `rst` is soc_full_rst_bank[5], which asserts on the JTAG
    // debug-full-reset (vio_boot_ctrl[3]) and on the debug-CSR reset paths
    // -- i.e. on exactly the resets used to reboot the Mac from JTAG.  With
    // ctrl_q on that reset, `vhdd-enable ram 1` could never be observed by a
    // booting machine: every reset that produced a trustworthy cold boot also
    // restored CTRL_RESET, so the RAM disk could only ever be enabled on an
    // already-running system.  The alternative -- making the RAM disk live at
    // power-on -- puts an unformatted target at SCSI ID 1 in front of the
    // ROM's boot scan on EVERY boot, which is a config with no full ROM-boot
    // sim behind it and not something to impose by default.
    //
    // cfg_rst is core_rst_bank[5]: asserted by power-on and by a real
    // platform reset (button / vio-hard-reset), NOT by the JTAG debug-full
    // reset.  So the enable behaves the way an operator expects -- set it
    // over JTAG, reboot the machine over JTAG, and it is still set; power
    // cycle or press the button, and it is back to CTRL_RESET.  Same
    // precedent as `cold_reset_hold` (docs/reset_story.md S4), which sits on
    // core_rst for the same reason.
    always @(posedge clk) begin
        if (cfg_rst)
            ctrl_q <= CTRL_RESET;
        else if (wr_commit && commit_in_regs &&
                 (commit_addr[6:0] == OFF_CTRL))
            ctrl_q <= merged_ctrl;
    end

    always @(posedge clk) begin
        if (rst) begin
            // Absent-until-configured: see the comment on net_mac_q.
            net_mac_q     <= 48'd0;
            net_dst_mac_q <= 48'd0;
            net_ip_q      <= 32'd0;
            net_dst_ip_q  <= 32'd0;
            net_port_q    <= 16'd0;
            net_dst_port_q<= 16'd0;
            aw_seen     <= 1'b0;
            w_seen      <= 1'b0;
            aw_addr_q   <= {ADDR_WIDTH{1'b0}};
            w_data_q    <= 32'd0;
            w_strb_q    <= 4'd0;
            bvalid_r    <= 1'b0;
            rvalid_r    <= 1'b0;
            rdata       <= 32'd0;
            rd_blocks_q <= (RD_BLOCKS_RESET > RD_MAX_BLOCKS) ? RD_MAX_BLOCKS
                                                             : RD_BLOCKS_RESET;
            trace_rd_addr <= 12'd0;
            trace_freeze  <= 1'b0;
            trace_clear   <= 1'b0;
        end else begin
            if (awvalid && awready) begin
                aw_seen   <= 1'b1;
                aw_addr_q <= awaddr;
            end
            if (wvalid && wready) begin
                w_seen   <= 1'b1;
                w_data_q <= wdata;
                w_strb_q <= wstrb;
            end

            if (wr_commit) begin
                bvalid_r <= 1'b1;
                aw_seen  <= 1'b0;
                w_seen   <= 1'b0;
                if (commit_in_regs) begin
                    case (commit_addr[6:0])
                        OFF_RD_BLOCKS: rd_blocks_q <= clamped_rd_blocks;
                        OFF_NET_MAC_LO:     net_mac_q[31:0]      <= merge32(net_mac_q[31:0]);
                        OFF_NET_MAC_HI:     net_mac_q[47:32]     <= merged_net_mac_hi[15:0];
                        OFF_NET_IP:         net_ip_q             <= merge32(net_ip_q);
                        OFF_NET_DST_MAC_LO: net_dst_mac_q[31:0]  <= merge32(net_dst_mac_q[31:0]);
                        OFF_NET_DST_MAC_HI: net_dst_mac_q[47:32] <= merged_net_dst_mac_hi[15:0];
                        OFF_NET_DST_IP:     net_dst_ip_q         <= merge32(net_dst_ip_q);
                        OFF_NET_PORTS: begin
                            net_port_q     <= merged_net_ports[31:16];
                            net_dst_port_q <= merged_net_ports[15:0];
                        end
                        OFF_TRACE_CTRL: if (commit_strb[0]) begin
                            trace_freeze <= commit_data[0];
                            // bit1 is a "do it" bit, not stored state:
                            // flip the toggle the ring edge-detects.
                            if (commit_data[1]) trace_clear <= ~trace_clear;
                        end
                        OFF_TRACE_ADDR: if (commit_strb[0])
                            trace_rd_addr <= commit_data[11:0];
                        // Every other offset is write-ignored and answers
                        // OKAY — the terminator behaviour this module
                        // inherits from axil_null_slave.
                        default: ;
                    endcase
                end
            end else if (bvalid_r && bready) begin
                bvalid_r <= 1'b0;
            end

            // ── Read channel ──────────────────────────────────────────
            if (!rvalid_r) begin
                if (arvalid && arready) begin
                    rvalid_r <= 1'b1;
                    if (!ar_in_regs) rdata <= 32'd0;
                    else case (araddr[6:0])
                        OFF_IDENT:         rdata <= IDENT_VALUE;
                        OFF_CTRL:          rdata <= {29'd0, ctrl_q};  // ctrl_q is 3 bits since wprot
                        OFF_RD_BLOCKS:     rdata <= rd_blocks_q;
                        OFF_NET_MAC_LO:     rdata <= net_mac_q[31:0];
                        OFF_NET_MAC_HI:     rdata <= {16'd0, net_mac_q[47:32]};
                        OFF_NET_IP:         rdata <= net_ip_q;
                        OFF_NET_DST_MAC_LO: rdata <= net_dst_mac_q[31:0];
                        OFF_NET_DST_MAC_HI: rdata <= {16'd0, net_dst_mac_q[47:32]};
                        OFF_NET_DST_IP:     rdata <= net_dst_ip_q;
                        OFF_NET_PORTS:      rdata <= {net_port_q, net_dst_port_q};
                        OFF_SD_BLOCKS:     rdata <= sd_num_lbas;
                        // ctrl_q[1:0] EXPLICITLY, not ctrl_q: widening CTRL
                        // for the write-protect bit silently shifted every
                        // field above it up by one and pushed this concat to
                        // 33 bits, dropping the top watchdog bit.  STATUS is
                        // a published layout (docs above, jtag_repl.tcl,
                        // tb_vhdd_ctrl), so wprot goes in the reserved
                        // padding instead of displacing anything.
                        OFF_STATUS:        rdata <= {rd_wdog_fires, 7'd0,
                                                     ctrl_q[2],
                                                     rd_state, rd_error,
                                                     rd_busy, ctrl_q[1:0]};
                        OFF_RD_MAX_BLOCKS: rdata <= RD_MAX_BLOCKS;
                        OFF_RD_APERTURE:   rdata <= RD_APERTURE;
                        OFF_RD_WDOG_FIRES: rdata <= {16'd0, rd_wdog_fires};
                        OFF_TRACE_CTRL:    rdata <= {14'd0, trace_frozen,
                                                     trace_wrapped,
                                                     4'd0, trace_wrptr};
                        OFF_TRACE_ADDR:    rdata <= {20'd0, trace_rd_addr};
                        OFF_TRACE_DATA:    rdata <= trace_rd_data;
                        default:           rdata <= 32'd0;
                    endcase
                end
            end else if (rready) begin
                rvalid_r <= 1'b0;
            end
        end
    end

endmodule

`default_nettype wire
