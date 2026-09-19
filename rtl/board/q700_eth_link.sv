// q700_eth_link.sv -- proven RK5 RGMII/Taxi physical-link wrapper.
//
// In SONIC mode the single Taxi MAC is shared with the network-block client.
// RX frames are classified from their six-byte destination address: an exact
// blk_mac match goes only to the block client, all other unicasts go only to
// the SONIC, and broadcast is copied to both.  The broadcast fork remembers
// acceptance independently for each consumer, so a fast consumer never sees
// a byte twice while the other is stalled.  This deliberately lets the slower
// consumer backpressure the wire RX stream: dropping one copy would make the
// two clients observe different broadcast traffic, while the MAC's frame FIFO
// provides the appropriate place to absorb a bounded stall.  The alternative
// -- replicating into a per-client elastic buffer -- costs a full-MTU store per
// client and only converts a stall into a drop once that buffer fills, which
// buys nothing for two engines that both accept at line rate into their own
// packet stores.  The one case that must NOT stall the wire is a client that
// is not there at all, so an all-zero blk_mac means "absent": such a client is
// excluded from both the exact match and the broadcast fan-out, and routing
// collapses to exactly the pre-sharing behaviour.  Without that rule a tied-off
// seam whose blk_rx_tready is strapped low would hold the first broadcast frame
// forever and starve the SONIC of the entire wire.
//
// TX arbitration is frame-granular.  A selected client owns the MAC through
// the accepted tlast beat; only then may the other client be selected.  A grant
// is revocable only until its first accepted beat, which is what stops a
// just-granted-but-never-started SONIC stream from parking the MAC forever.
//
// SONIC loopback remains a SONIC-local diagnostic: SONIC TX is diverted to
// SONIC RX and completed locally, but the block client retains full wire TX/RX
// access.  That is deliberate.  RCR_LB means "do not put MY frames on the
// wire" -- it is a NIC self-test the guest driver runs at init.  The block
// service is a host-side facility the guest cannot see, and it may be backing
// the boot volume; stalling storage I/O because the guest ran a NIC self-test
// would present as a SCSI hang with no causal relation to what the guest did.
// loopback is sampled only BETWEEN SONIC transmit frames, for the same reason
// rx_drain is sampled only between wire frames: a mid-frame flip would strand a
// half-written frame in the MAC's frame FIFO.  Likewise rx_drain disables only
// the SONIC copy, at the existing wire-frame boundary; it never drains traffic
// addressed to the block client.
`default_nettype none

// Byte-stream sharing core, kept separate from the PHY and CDC plumbing so
// frame routing/arbitration can be unit-tested without vendor primitives.
module q700_eth_stream_share (
    input  wire        clk,
    input  wire        rst,
    input  wire        loopback,
    input  wire        rx_drain,
    input  wire [47:0] blk_mac,

    input  wire [7:0]  mac_rx_tdata,
    input  wire        mac_rx_tvalid,
    output wire        mac_rx_tready,
    input  wire        mac_rx_tlast,
    input  wire        mac_rx_tuser,

    output wire [7:0]  mac_tx_tdata,
    output wire        mac_tx_tvalid,
    input  wire        mac_tx_tready,
    output wire        mac_tx_tlast,
    output wire        mac_tx_owner_sonic,

    input  wire [7:0]  sonic_tx_tdata,
    input  wire        sonic_tx_tvalid,
    output wire        sonic_tx_tready,
    input  wire        sonic_tx_tlast,
    output wire        sonic_loopback_cpl,
    output wire [7:0]  sonic_rx_tdata,
    output wire        sonic_rx_tvalid,
    input  wire        sonic_rx_tready,
    output wire        sonic_rx_tlast,
    output wire        sonic_rx_tuser,

    input  wire [7:0]  blk_tx_tdata,
    input  wire        blk_tx_tvalid,
    output wire        blk_tx_tready,
    input  wire        blk_tx_tlast,
    output wire [7:0]  blk_rx_tdata,
    output wire        blk_rx_tvalid,
    input  wire        blk_rx_tready,
    output wire        blk_rx_tlast
);
    localparam [1:0] TX_NONE = 2'd0;
    localparam [1:0] TX_SONIC = 2'd1;
    localparam [1:0] TX_BLOCK = 2'd2;
    localparam [1:0] RX_DEST = 2'd0;
    localparam [1:0] RX_REPLAY = 2'd1;
    localparam [1:0] RX_FORWARD = 2'd2;
    localparam [47:0] BROADCAST_MAC = 48'hff_ff_ff_ff_ff_ff;
    localparam [47:0] NO_BLOCK_MAC = 48'h00_00_00_00_00_00;
    localparam integer DEST_BYTES = 6;
    localparam [2:0] DEST_LAST_INDEX = 3'd5;

    // An all-zero blk_mac means no block client is attached -- that address is
    // never a legal Ethernet destination, so it is free to carry the meaning.
    // The distinction is load-bearing rather than cosmetic: broadcast fan-out
    // waits for EVERY routed consumer, so a tied-off client whose blk_rx_tready
    // is strapped low would hold the first broadcast frame forever and starve
    // the SONIC of the whole wire.  With the client absent, routing collapses
    // to exactly the pre-sharing behaviour.
    wire blk_present = (blk_mac != NO_BLOCK_MAC);

    // Round-robin selection applies only between frames.  Latching the grant
    // before exposing ready also keeps the selected source stable under MAC
    // backpressure.
    reg [1:0] tx_grant;
    reg       tx_last_was_sonic;
    // Set once the granted client has actually put a beat into the MAC.  Until
    // then the grant is revocable: a client may withdraw tvalid, and loopback
    // may engage in the very cycle the grant was taken (the arbiter decides
    // from the pre-latch view of loopback).  Without the release the MAC would
    // stay parked on a source that never presents another wire beat, and the
    // other client would never be granted again.
    reg       tx_frame_started;
    // loopback is re-evaluated only BETWEEN SONIC transmit frames, for the same
    // reason rx_drain is latched at a wire-frame boundary.  A mid-frame flip
    // would strand a half-written frame in the MAC's frame FIFO and -- worse --
    // park the grant on a SONIC stream that has stopped presenting wire beats,
    // permanently starving the block client of the MAC.
    reg  sonic_tx_in_frame;
    reg  tx_loopback;
    wire sonic_tx_fire = sonic_tx_tvalid && sonic_tx_tready;
`ifdef Q700_ETH_SHARE_MUTANT_RAW_LOOPBACK
    // Verification mutant: sampling loopback continuously lets it flip inside a
    // SONIC frame already streaming to the MAC.
    wire tx_loopback_eff = loopback;
`else
    wire tx_loopback_eff = tx_loopback;
`endif
    wire sonic_wire_valid = sonic_tx_tvalid && !tx_loopback_eff;
    wire choose_sonic = sonic_wire_valid &&
        (!blk_tx_tvalid || !tx_last_was_sonic);
    wire tx_fire = mac_tx_tvalid && mac_tx_tready;

    assign mac_tx_tdata = (tx_grant == TX_SONIC) ? sonic_tx_tdata : blk_tx_tdata;
    assign mac_tx_tvalid = ((tx_grant == TX_SONIC) && sonic_wire_valid) ||
                           ((tx_grant == TX_BLOCK) && blk_tx_tvalid);
    assign mac_tx_tlast = (tx_grant == TX_SONIC) ? sonic_tx_tlast : blk_tx_tlast;
    assign mac_tx_owner_sonic = (tx_grant == TX_SONIC);
    assign sonic_tx_tready = tx_loopback_eff ? sonic_rx_tready :
        ((tx_grant == TX_SONIC) && mac_tx_tready);
    assign blk_tx_tready = (tx_grant == TX_BLOCK) && mac_tx_tready;
    assign sonic_loopback_cpl = tx_loopback_eff && sonic_tx_tvalid &&
        sonic_tx_tready && sonic_tx_tlast;

    always @(posedge clk) begin
        if (rst) begin
            tx_frame_started <= 1'b0;
        end else if (tx_grant == TX_NONE) begin
            tx_frame_started <= 1'b0;
        end else if (tx_fire) begin
            tx_frame_started <= !mac_tx_tlast;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            sonic_tx_in_frame <= 1'b0;
            tx_loopback <= 1'b0;
        end else begin
            if (sonic_tx_fire) sonic_tx_in_frame <= !sonic_tx_tlast;
            if (!sonic_tx_in_frame && !(sonic_tx_fire && !sonic_tx_tlast))
                tx_loopback <= loopback;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            tx_grant <= TX_NONE;
            tx_last_was_sonic <= 1'b0;
        end else if (tx_grant == TX_NONE) begin
            if (choose_sonic)
                tx_grant <= TX_SONIC;
            else if (blk_tx_tvalid)
                tx_grant <= TX_BLOCK;
`ifndef Q700_ETH_SHARE_MUTANT_STICKY_GRANT
        end else if (!tx_frame_started && !mac_tx_tvalid) begin
            tx_grant <= TX_NONE;   // nothing was sent: the grant is revocable
`endif
        end else if (tx_fire &&
`ifdef Q700_ETH_SHARE_MUTANT_BEAT_ARBITER
                     1'b1) begin
            // Verification mutant: re-arbitrating every beat interleaves
            // simultaneous frames and must be rejected by the unit test.
            tx_last_was_sonic <= (tx_grant == TX_SONIC);
            tx_grant <= TX_NONE;
`else
                     mac_tx_tlast) begin
            tx_last_was_sonic <= (tx_grant == TX_SONIC);
            tx_grant <= TX_NONE;
`endif
        end
    end

    // Preserve the hardware-proven drain rule: sample rx_drain only between
    // complete wire frames.  Demux replay stalls do not create a boundary.
    reg rx_draining;
    reg rx_in_frame;
    wire mac_rx_fire = mac_rx_tvalid && mac_rx_tready;
    always @(posedge clk) begin
        if (rst) begin
            rx_draining <= 1'b1;
            rx_in_frame <= 1'b0;
        end else begin
            if (mac_rx_fire)
                rx_in_frame <= !mac_rx_tlast;
            if (!rx_in_frame && !(mac_rx_fire && !mac_rx_tlast))
                rx_draining <= rx_drain;
        end
    end

    reg [1:0] rx_state;
    reg [2:0] dest_index;
    reg [2:0] replay_index;
    reg [2:0] replay_count;
    reg [7:0] dest_data [0:DEST_BYTES-1];
    reg       dest_last [0:DEST_BYTES-1];
    reg       dest_user [0:DEST_BYTES-1];
    reg [39:0] captured_dest;   // dest bytes 0..4; byte 5 compares live
    reg route_sonic;
    reg route_block;

    // One elastic output beat carries independent pending bits.  For a
    // broadcast, either client may accept first; that pending bit clears while
    // the byte remains valid for the stalled client.  The next byte replaces
    // this one in the same cycle once every required copy has been accepted.
    reg       out_valid;
    reg [7:0] out_data;
    reg       out_last;
    reg       out_user;
    reg       out_sonic_pending;
    reg       out_block_pending;
    wire sonic_demux_ready = (tx_loopback_eff || rx_draining) ? 1'b1 : sonic_rx_tready;
    wire sonic_copy_fire = out_valid && out_sonic_pending && sonic_demux_ready;
    wire block_copy_fire = out_valid && out_block_pending && blk_rx_tready;
    wire out_done = out_valid &&
        (!out_sonic_pending || sonic_demux_ready) &&
        (!out_block_pending || blk_rx_tready);
    wire out_slot_ready = !out_valid || out_done;

    assign sonic_rx_tdata = tx_loopback_eff ? sonic_tx_tdata : out_data;
    assign sonic_rx_tvalid = tx_loopback_eff ? sonic_tx_tvalid :
        (out_valid && out_sonic_pending && !rx_draining);
    assign sonic_rx_tlast = tx_loopback_eff ? sonic_tx_tlast : out_last;
    assign sonic_rx_tuser = tx_loopback_eff ? 1'b0 : out_user;
    assign blk_rx_tdata = out_data;
    assign blk_rx_tvalid = out_valid && out_block_pending;
    assign blk_rx_tlast = out_last;
    assign mac_rx_tready = (rx_state == RX_DEST) ||
                           ((rx_state == RX_FORWARD) && out_slot_ready);

    integer rx_i;
    always @(posedge clk) begin
        if (rst) begin
            rx_state <= RX_DEST;
            dest_index <= 3'd0;
            replay_index <= 3'd0;
            replay_count <= 3'd0;
            captured_dest <= 40'd0;
            route_sonic <= 1'b1;
            route_block <= 1'b0;
            out_valid <= 1'b0;
            out_data <= 8'd0;
            out_last <= 1'b0;
            out_user <= 1'b0;
            out_sonic_pending <= 1'b0;
            out_block_pending <= 1'b0;
            for (rx_i = 0; rx_i < DEST_BYTES; rx_i = rx_i + 1) begin
                dest_data[rx_i] <= 8'd0;
                dest_last[rx_i] <= 1'b0;
                dest_user[rx_i] <= 1'b0;
            end
        end else begin
            if (sonic_copy_fire)
                out_sonic_pending <= 1'b0;
            if (block_copy_fire)
                out_block_pending <= 1'b0;
            if (out_done)
                out_valid <= 1'b0;

            case (rx_state)
                RX_DEST: if (mac_rx_fire) begin
                    dest_data[dest_index] <= mac_rx_tdata;
                    dest_last[dest_index] <= mac_rx_tlast;
                    dest_user[dest_index] <= mac_rx_tuser;
                    captured_dest <= {captured_dest[31:0], mac_rx_tdata};
                    if (mac_rx_tlast || (dest_index == DEST_LAST_INDEX)) begin
                        replay_index <= 3'd0;
                        replay_count <= dest_index + 1'b1;
                        if ((dest_index == DEST_LAST_INDEX) &&
                            ({captured_dest, mac_rx_tdata} == BROADCAST_MAC)) begin
                            route_sonic <= 1'b1;
`ifdef Q700_ETH_SHARE_MUTANT_BCAST_ABSENT
                            // Verification mutant: fanning broadcast out to a
                            // client that is not attached wedges the wire.
                            route_block <= 1'b1;
`else
                            route_block <= blk_present;
`endif
                        end else if (blk_present &&
                            (dest_index == DEST_LAST_INDEX) &&
                            ({captured_dest, mac_rx_tdata} == blk_mac)) begin
                            route_sonic <= 1'b0;
                            route_block <= 1'b1;
                        end else begin
                            route_sonic <= 1'b1;
                            route_block <= 1'b0;
                        end
                        rx_state <= RX_REPLAY;
                        dest_index <= 3'd0;
                    end else begin
                        dest_index <= dest_index + 1'b1;
                    end
                end

                RX_REPLAY: if (out_slot_ready) begin
                    out_valid <= 1'b1;
                    out_data <= dest_data[replay_index];
                    out_last <= dest_last[replay_index];
                    out_user <= dest_user[replay_index];
                    out_sonic_pending <= route_sonic;
                    out_block_pending <= route_block;
                    if (replay_index + 1'b1 == replay_count) begin
                        replay_index <= 3'd0;
                        rx_state <= dest_last[replay_index] ? RX_DEST : RX_FORWARD;
                    end else begin
                        replay_index <= replay_index + 1'b1;
                    end
                end

                RX_FORWARD: if (mac_rx_fire) begin
                    out_valid <= 1'b1;
                    out_data <= mac_rx_tdata;
                    out_last <= mac_rx_tlast;
                    out_user <= mac_rx_tuser;
                    out_sonic_pending <= route_sonic;
                    out_block_pending <= route_block;
                    if (mac_rx_tlast)
                        rx_state <= RX_DEST;
                end

                default: rx_state <= RX_DEST;
            endcase
        end
    end
endmodule


// ── q700_toggle_rx ──────────────────────────────────────────────────────
// Toggle-to-pulse CDC RECEIVER with reset-skew priming.
//
// The toggle-per-event idiom crosses a clock boundary by flipping a bit in
// the source domain and edge-detecting it in the destination.  It is
// correct only while the two domains' resets are the same event.  Here
// they are not, and cannot be made so:
//
//   * the source (`tx_cpl_toggle`, 125 MHz MAC domain) resets on
//     `rst_125mhz`, which is `taxi_sync_reset(~mmcm_locked)` of an MMCM
//     whose `.RST` is hardwired to 1'b0 -- so NO platform reset, cold
//     reset, debug-full-reset or 68040 RESET instruction ever clears it.
//     After the first lock it only ever flips on real MAC transmits.
//   * the destination synchroniser chain resets on `core_rst`, which
//     forces all three stages to 0.
//
// So on EVERY core reset taken after any TX traffic, the chain refills
// from 0 towards whatever parity the source happens to hold.  If that
// parity is 1 -- half the time -- the refill walks 0,0 -> 1,0 -> 1,1 and
// the `s2 ^ s3` detector manufactures a completion NOBODY GENERATED.
// `tx_cpl_pending` then latches and the FIRST REAL transmit after the
// reset completes instantly: the SONIC writes back TCR_PTX before the
// frame has left the MAC, and the completion stream stays permanently one
// ahead.  The corruption is permanent, not transient -- there is no
// self-correcting event, because a toggle carries no absolute value.
//
// The fix is to suppress edge detection until the chain has FILLED with
// the source's real value.  Three stages take three destination clocks,
// so a 2-bit counter loaded with SYNC_FF at reset and counted down to
// zero covers exactly the refill, and nothing longer: a genuine event in
// those cycles is impossible, because the destination was itself in reset
// and therefore had nothing outstanding for the source to complete.
//
// This is deliberately a SEPARATE, always-compiled module rather than
// inline logic inside `q700_eth_link`: that module is behind `ETH_ENABLE`
// and needs vendor MAC/MMCM/IDELAY primitives, so nothing can unit-test
// it.  Out here it is exercised directly by `tb-q700-toggle-rx`, whose
// `-mut` target rebuilds this file with the priming deleted and requires
// the test to REJECT it.
module q700_toggle_rx #(
    parameter integer W       = 1,
    // Synchroniser depth.  3 is the chain this replaced; the priming
    // window is sized from this, so the two can never drift apart.
    parameter integer SYNC_FF = 3
) (
    input  wire         clk,
    input  wire         rst,
    input  wire [W-1:0] src_toggle,   // asynchronous, source-domain
    output wire [W-1:0] pulse         // one clk-wide, destination-domain
);
    (* ASYNC_REG="TRUE" *) reg [W-1:0] sync_q [0:SYNC_FF-1];
    reg [1:0] prime_q;
    integer   k;
    always @(posedge clk) begin
        if (rst) begin
            for (k = 0; k < SYNC_FF; k = k + 1) sync_q[k] <= {W{1'b0}};
            prime_q <= SYNC_FF[1:0];
        end else begin
            sync_q[0] <= src_toggle;
            for (k = 1; k < SYNC_FF; k = k + 1) sync_q[k] <= sync_q[k-1];
            if (prime_q != 2'd0) prime_q <= prime_q - 2'd1;
        end
    end
`ifdef Q700_TOGGLE_RX_MUTANT_NO_PRIME
    // RED-VERIFY mutant: the pre-fix behaviour -- edge-detect immediately,
    // so a reset taken with the source toggle at 1 manufactures a pulse.
    assign pulse = sync_q[SYNC_FF-2] ^ sync_q[SYNC_FF-1];
`else
    assign pulse = (prime_q == 2'd0)
                   ? (sync_q[SYNC_FF-2] ^ sync_q[SYNC_FF-1])
                   : {W{1'b0}};
`endif
endmodule

`ifdef ETH_ENABLE
module q700_eth_link #(
    parameter bit ICMP_RESPONDER = 1'b1,
    parameter integer REF_CLK_MHZ = 100
) (
    // Already-buffered SoC utility clock.  Real-MIG builds supply the
    // 100 MHz fabric MGT-reference BUFG output; SIM_MODEL supplies 200 MHz.
    // The DDR sys_clk pins belong exclusively to the MIG black box.
    input  wire       clk_ref,
    input  wire       phy_rx_clk,
    input  wire [3:0] phy_rxd,
    input  wire       phy_rx_ctl,
    output wire       phy_tx_clk,
    output wire [3:0] phy_txd,
    output wire       phy_tx_ctl,
    output wire [1:0] link_speed,
    output wire       rx_activity,
    output wire       tx_activity,
    output reg  [7:0] debug_event_toggle,
    input  wire       core_clk,
    input  wire       core_rst,
    // RCR_LB non-zero: loop the transmit stream straight back into the
    // receive stream and keep it off the wire.  Frames handed to us are
    // payload-only (Taxi owns pad/FCS on the way out, and strips FCS on the
    // way in), so feeding TX directly to RX presents exactly the shape the
    // receive adapter already expects -- it reconstructs the FCS itself.
    input  wire       loopback,
    // Assert while the SONIC receiver is disabled.  A real SONIC always
    // accepts from the MAC and discards; it never lets frames accumulate.
    // We used to backpressure instead, so the Taxi async FIFO built up a
    // backlog while RX was off and then dumped it into the descriptor ring
    // the instant the driver enabled reception -- exhausting the ring before
    // the driver could service it (ISR_RDE, CRDA at EOL).
    input  wire       rx_drain,
    input  wire [7:0] sonic_tx_tdata,
    input  wire       sonic_tx_tvalid,
    output wire       sonic_tx_tready,
    input  wire       sonic_tx_tlast,
    output wire       sonic_tx_cpl_valid,
    input  wire       sonic_tx_cpl_ready,
    output wire [7:0] sonic_rx_tdata,
    output wire       sonic_rx_tvalid,
    input  wire       sonic_rx_tready,
    output wire       sonic_rx_tlast,
    output wire       sonic_rx_tuser,
    input  wire [47:0] blk_mac,
    // The block client runs in the vhdd/SCSI domain (pb_clk), not core_clk:
    // its provider sits beside vhdd_sd on the vhdd contract, and that contract
    // is pb_clk throughout.  So the blk_* ports below are blk_clk, and the two
    // FRAME fifos inside do the crossing.  Frame fifos specifically -- the TX
    // arbiter grants tlast-to-tlast, so a byte-granular crossing that let half
    // a frame through would break the very non-interleaving it enforces.
    input  wire       blk_clk,
    input  wire       blk_rst,
    input  wire [7:0] blk_tx_tdata,
    input  wire       blk_tx_tvalid,
    output wire       blk_tx_tready,
    input  wire       blk_tx_tlast,
    output wire [7:0] blk_rx_tdata,
    output wire       blk_rx_tvalid,
    input  wire       blk_rx_tready,
    output wire       blk_rx_tlast
);
    wire clk_125mhz_mmcm, clk90_125mhz_mmcm;
    wire clk_312mhz_mmcm, clk_125mhz, clk90_125mhz, clk_312mhz;
    wire mmcm_clkfb, mmcm_locked, rst_125mhz, rst_312mhz;

    MMCME4_BASE #(
        .CLKIN1_PERIOD((REF_CLK_MHZ == 200) ? 5.0 : 10.0),
        .DIVCLK_DIVIDE((REF_CLK_MHZ == 200) ? 4 : 2), .CLKFBOUT_MULT_F(25),
        .CLKOUT0_DIVIDE_F(10), .CLKOUT1_DIVIDE(10), .CLKOUT1_PHASE(90),
        .CLKOUT2_DIVIDE(4), .BANDWIDTH("OPTIMIZED"), .STARTUP_WAIT("FALSE")
    ) clk_mmcm_inst (
        .CLKIN1(clk_ref), .CLKFBIN(mmcm_clkfb), .CLKFBOUT(mmcm_clkfb),
        .CLKFBOUTB(), .CLKOUT0(clk_125mhz_mmcm), .CLKOUT0B(),
        .CLKOUT1(clk90_125mhz_mmcm), .CLKOUT1B(), .CLKOUT2(clk_312mhz_mmcm),
        .CLKOUT2B(), .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(),
        .CLKOUT6(), .RST(1'b0), .PWRDWN(1'b0), .LOCKED(mmcm_locked)
    );
    BUFG clk_bufg_inst (.I(clk_125mhz_mmcm), .O(clk_125mhz));
    BUFG clk90_bufg_inst (.I(clk90_125mhz_mmcm), .O(clk90_125mhz));
    BUFG clk_312_bufg_inst (.I(clk_312mhz_mmcm), .O(clk_312mhz));
    taxi_sync_reset #(.N(4)) rst_125_inst (.clk(clk_125mhz), .rst(~mmcm_locked), .out(rst_125mhz));
    taxi_sync_reset #(.N(4)) rst_312_inst (.clk(clk_312mhz), .rst(~mmcm_locked), .out(rst_312mhz));

    wire [3:0] phy_rxd_delayed;
    wire phy_rx_ctl_delayed;
    IDELAYCTRL #(.SIM_DEVICE("ULTRASCALE")) idelayctrl_inst (
        .REFCLK(clk_312mhz), .RST(rst_312mhz), .RDY()
    );
    for (genvar n = 0; n < 4; n = n + 1) begin : phy_rxd_idelay_bit
        IDELAYE3 #(.DELAY_SRC("IDATAIN"), .CASCADE("NONE"), .DELAY_TYPE("FIXED"),
            .DELAY_VALUE(0), .REFCLK_FREQUENCY(312.5), .DELAY_FORMAT("TIME"),
            .UPDATE_MODE("SYNC"), .SIM_DEVICE("ULTRASCALE_PLUS")) idelay_inst (
            .CASC_IN(1'b0), .CASC_RETURN(1'b0), .CASC_OUT(), .IDATAIN(phy_rxd[n]),
            .DATAIN(1'b0), .DATAOUT(phy_rxd_delayed[n]), .CLK(1'b0), .EN_VTC(1'b1),
            .CE(1'b0), .INC(1'b0), .LOAD(1'b0), .RST(1'b0), .CNTVALUEIN(9'd0), .CNTVALUEOUT()
        );
    end
    IDELAYE3 #(.DELAY_SRC("IDATAIN"), .CASCADE("NONE"), .DELAY_TYPE("FIXED"),
        .DELAY_VALUE(0), .REFCLK_FREQUENCY(312.5), .DELAY_FORMAT("TIME"),
        .UPDATE_MODE("SYNC"), .SIM_DEVICE("ULTRASCALE_PLUS")) phy_rx_ctl_idelay (
        .CASC_IN(1'b0), .CASC_RETURN(1'b0), .CASC_OUT(), .IDATAIN(phy_rx_ctl),
        .DATAIN(1'b0), .DATAOUT(phy_rx_ctl_delayed), .CLK(1'b0), .EN_VTC(1'b1),
        .CE(1'b0), .INC(1'b0), .LOAD(1'b0), .RST(1'b0), .CNTVALUEIN(9'd0), .CNTVALUEOUT()
    );

    taxi_axis_if #(.DATA_W(8), .ID_W(8), .USER_EN(1), .USER_W(1)) axis_tx();
    taxi_axis_if #(.DATA_W(8), .ID_W(8), .USER_EN(1), .USER_W(1)) axis_rx();
    taxi_axis_if #(.DATA_W(96), .KEEP_W(1), .ID_W(8)) axis_tx_cpl();
    taxi_axis_if #(.DATA_W(16), .KEEP_W(1), .KEEP_EN(0), .LAST_EN(0),
        .USER_EN(1), .USER_W(1), .ID_EN(1), .ID_W(10)) axis_stat();

    wire tx_error_underflow;
    wire tx_fifo_overflow, tx_fifo_bad_frame, tx_fifo_good_frame;
    wire rx_error_bad_frame, rx_error_bad_fcs, rx_fifo_overflow;
    wire rx_fifo_bad_frame, rx_fifo_good_frame;

    taxi_eth_mac_1g_rgmii_fifo #(
        .SIM(1'b0), .VENDOR("XILINX"), .FAMILY("kintexuplus"),
        .USE_CLK90(1'b0), .STAT_EN(1'b0), .TX_FIFO_DEPTH(4096), .TX_FRAME_FIFO(1'b1),
        .RX_FIFO_DEPTH(4096), .RX_FRAME_FIFO(1'b1)
    ) eth_mac_inst (
        .gtx_clk(clk_125mhz), .gtx_clk90(clk90_125mhz), .gtx_rst(rst_125mhz),
        .logic_clk(clk_125mhz), .logic_rst(rst_125mhz), .s_axis_tx(axis_tx),
        .m_axis_tx_cpl(axis_tx_cpl), .m_axis_rx(axis_rx), .rgmii_rx_clk(phy_rx_clk),
        .rgmii_rxd(phy_rxd_delayed), .rgmii_rx_ctl(phy_rx_ctl_delayed),
        .rgmii_tx_clk(phy_tx_clk), .rgmii_txd(phy_txd), .rgmii_tx_ctl(phy_tx_ctl),
        .stat_clk(clk_125mhz), .stat_rst(rst_125mhz), .m_axis_stat(axis_stat),
        .tx_error_underflow(tx_error_underflow), .tx_fifo_overflow(tx_fifo_overflow),
        .tx_fifo_bad_frame(tx_fifo_bad_frame), .tx_fifo_good_frame(tx_fifo_good_frame),
        .rx_error_bad_frame(rx_error_bad_frame), .rx_error_bad_fcs(rx_error_bad_fcs),
        .rx_fifo_overflow(rx_fifo_overflow), .rx_fifo_bad_frame(rx_fifo_bad_frame),
        .rx_fifo_good_frame(rx_fifo_good_frame), .link_speed(link_speed), .cfg_tx_pad_en(1'b1),
        .cfg_tx_min_pkt_len(8'd59), .cfg_tx_max_pkt_len(16'd1517), .cfg_tx_ifg(8'd12),
        .cfg_tx_enable(1'b1), .cfg_rx_max_pkt_len(16'd1517), .cfg_rx_enable(1'b1)
    );
    assign axis_stat.tready = 1'b1;

    // Toggle-per-event CDC seam for the optional core-clock JTAG telemetry
    // block.  A toggle survives a narrow 125 MHz pulse; the destination
    // synchronizes and edge-detects each bit independently.  The entire
    // cone is pruned when ETH_DEBUG_ENABLE is not built.
    wire [7:0] debug_event = {
        rx_fifo_good_frame, rx_fifo_overflow, rx_error_bad_fcs,
        rx_error_bad_frame | rx_fifo_bad_frame,
        tx_fifo_good_frame, tx_fifo_bad_frame, tx_fifo_overflow,
        tx_error_underflow
    };
    always @(posedge clk_125mhz) begin
        if (rst_125mhz)
            debug_event_toggle <= 8'd0;
        else
            debug_event_toggle <= debug_event_toggle ^ debug_event;
    end

    generate if (ICMP_RESPONDER) begin : g_icmp_responder
        wire [31:0] unused_arp_count, unused_icmp_count;
        assign axis_tx.tkeep = 1'b1;
        assign axis_tx.tstrb = 1'b1;
        assign axis_tx.tid = '0;
        assign axis_tx.tdest = '0;
        icmp_echo_responder responder_inst (
            .clk(clk_125mhz), .rst(rst_125mhz), .rx_tdata(axis_rx.tdata),
            .rx_tvalid(axis_rx.tvalid), .rx_tlast(axis_rx.tlast), .rx_tuser(axis_rx.tuser[0]),
            .rx_tready(axis_rx.tready), .tx_tdata(axis_tx.tdata), .tx_tvalid(axis_tx.tvalid),
            .tx_tlast(axis_tx.tlast), .tx_tuser(axis_tx.tuser[0]), .tx_tready(axis_tx.tready),
            .rx_activity(rx_activity), .reply_activity(tx_activity), .arp_reply_count(unused_arp_count),
            .icmp_reply_count(unused_icmp_count)
        );
        assign sonic_tx_tready = 1'b0;
        assign sonic_tx_cpl_valid = 1'b0;
        assign sonic_rx_tdata = 8'd0;
        assign sonic_rx_tvalid = 1'b0;
        assign sonic_rx_tlast = 1'b0;
        assign sonic_rx_tuser = 1'b0;
        assign blk_tx_tready = 1'b0;
        assign blk_rx_tdata = 8'd0;
        assign blk_rx_tvalid = 1'b0;
        assign blk_rx_tlast = 1'b0;
        /* verilator lint_off UNUSEDSIGNAL */
        wire _unused_blk_clk = &{1'b0, blk_clk, blk_rst};
        /* verilator lint_on UNUSEDSIGNAL */
        assign axis_tx_cpl.tready = 1'b1;
    end else begin : g_sonic_packet_adapter
        taxi_axis_if #(.DATA_W(8), .ID_W(8), .USER_EN(1), .USER_W(1)) core_tx_axis();
        taxi_axis_if #(.DATA_W(8), .ID_W(8), .USER_EN(1), .USER_W(1)) core_rx_axis();
        taxi_axis_if #(.DATA_W(8), .ID_W(8), .USER_EN(1), .USER_W(1)) owner_core_axis();
        taxi_axis_if #(.DATA_W(8), .ID_W(8), .USER_EN(1), .USER_W(1)) owner_mac_axis();
        taxi_axis_if #(.DATA_W(8), .ID_W(8), .USER_EN(1), .USER_W(1)) blk_core_tx();
        taxi_axis_if #(.DATA_W(8), .ID_W(8), .USER_EN(1), .USER_W(1)) blk_core_rx();
        taxi_axis_if #(.DATA_W(8), .ID_W(8), .USER_EN(1), .USER_W(1)) blk_pb_tx();
        taxi_axis_if #(.DATA_W(8), .ID_W(8), .USER_EN(1), .USER_W(1)) blk_pb_rx();
        wire [7:0] shared_tx_tdata;
        wire shared_tx_tvalid, shared_tx_tready, shared_tx_tlast;
        wire shared_tx_owner_sonic;
        wire sonic_loopback_cpl;

        q700_eth_stream_share stream_share_inst (
            .clk(core_clk), .rst(core_rst), .loopback(loopback),
            .rx_drain(rx_drain), .blk_mac(blk_mac),
            .mac_rx_tdata(core_rx_axis.tdata),
            .mac_rx_tvalid(core_rx_axis.tvalid),
            .mac_rx_tready(core_rx_axis.tready),
            .mac_rx_tlast(core_rx_axis.tlast),
            .mac_rx_tuser(core_rx_axis.tuser[0]),
            .mac_tx_tdata(shared_tx_tdata),
            .mac_tx_tvalid(shared_tx_tvalid),
            .mac_tx_tready(shared_tx_tready),
            .mac_tx_tlast(shared_tx_tlast),
            .mac_tx_owner_sonic(shared_tx_owner_sonic),
            .sonic_tx_tdata(sonic_tx_tdata),
            .sonic_tx_tvalid(sonic_tx_tvalid),
            .sonic_tx_tready(sonic_tx_tready),
            .sonic_tx_tlast(sonic_tx_tlast),
            .sonic_loopback_cpl(sonic_loopback_cpl),
            .sonic_rx_tdata(sonic_rx_tdata),
            .sonic_rx_tvalid(sonic_rx_tvalid),
            .sonic_rx_tready(sonic_rx_tready),
            .sonic_rx_tlast(sonic_rx_tlast),
            .sonic_rx_tuser(sonic_rx_tuser),
            .blk_tx_tdata(blk_core_tx.tdata), .blk_tx_tvalid(blk_core_tx.tvalid),
            .blk_tx_tready(blk_core_tx.tready), .blk_tx_tlast(blk_core_tx.tlast),
            .blk_rx_tdata(blk_core_rx.tdata), .blk_rx_tvalid(blk_core_rx.tvalid),
            .blk_rx_tready(blk_core_rx.tready), .blk_rx_tlast(blk_core_rx.tlast)
        );

        // ── block client clock crossing (blk_clk <-> core_clk) ─────────
        // Same shape and same depth as the SONIC's MAC-side crossing above.
        // DROP_OVERSIZE_FRAME is deliberate: the protocol caps a frame at two
        // blocks plus headers, so an oversize frame is a bug upstream, and
        // dropping it loses one transaction the transaction layer will retry
        // rather than jamming the FIFO for every transaction after it.
        assign blk_core_tx.tkeep = 1'b1;
        assign blk_core_tx.tstrb = 1'b1;
        assign blk_core_tx.tid   = '0;
        assign blk_core_tx.tdest = '0;
        assign blk_core_tx.tuser = '0;
        assign blk_pb_tx.tdata  = blk_tx_tdata;
        assign blk_pb_tx.tvalid = blk_tx_tvalid;
        assign blk_pb_tx.tlast  = blk_tx_tlast;
        assign blk_pb_tx.tkeep  = 1'b1;
        assign blk_pb_tx.tstrb  = 1'b1;
        assign blk_pb_tx.tid    = '0;
        assign blk_pb_tx.tdest  = '0;
        assign blk_pb_tx.tuser  = '0;
        assign blk_tx_tready    = blk_pb_tx.tready;

        taxi_axis_async_fifo #(.DEPTH(2048), .FRAME_FIFO(1'b1),
            .DROP_OVERSIZE_FRAME(1'b1)) blk_tx_cdc_fifo (
            .s_clk(blk_clk),  .s_rst(blk_rst),  .s_axis(blk_pb_tx),
            .m_clk(core_clk), .m_rst(core_rst), .m_axis(blk_core_tx)
        );
        taxi_axis_async_fifo #(.DEPTH(2048), .FRAME_FIFO(1'b1),
            .DROP_OVERSIZE_FRAME(1'b1)) blk_rx_cdc_fifo (
            .s_clk(core_clk), .s_rst(core_rst), .s_axis(blk_core_rx),
            .m_clk(blk_clk),  .m_rst(blk_rst),  .m_axis(blk_pb_rx)
        );
        assign blk_rx_tdata     = blk_pb_rx.tdata;
        assign blk_rx_tvalid    = blk_pb_rx.tvalid;
        assign blk_rx_tlast     = blk_pb_rx.tlast;
        assign blk_pb_rx.tready = blk_rx_tready;

        // The data and one-beat owner records cross together at each tlast.
        // Pairing Taxi's completion with the queued owner prevents a block
        // frame from falsely completing the SONIC transmit descriptor.
        assign core_tx_axis.tdata = shared_tx_tdata;
        assign core_tx_axis.tkeep = 1'b1;
        assign core_tx_axis.tstrb = 1'b1;
        assign core_tx_axis.tvalid = shared_tx_tvalid &&
            (!shared_tx_tlast || owner_core_axis.tready);
        assign core_tx_axis.tlast = shared_tx_tlast;
        assign core_tx_axis.tid = '0;
        assign core_tx_axis.tdest = '0;
        assign core_tx_axis.tuser = '0;
        assign shared_tx_tready = core_tx_axis.tready &&
            (!shared_tx_tlast || owner_core_axis.tready);
        assign owner_core_axis.tdata = {7'd0, shared_tx_owner_sonic};
        assign owner_core_axis.tkeep = 1'b1;
        assign owner_core_axis.tstrb = 1'b1;
        assign owner_core_axis.tvalid = shared_tx_tvalid && shared_tx_tlast &&
            core_tx_axis.tready;
        assign owner_core_axis.tlast = 1'b1;
        assign owner_core_axis.tid = '0;
        assign owner_core_axis.tdest = '0;
        assign owner_core_axis.tuser = '0;

        taxi_axis_async_fifo #(.DEPTH(2048), .FRAME_FIFO(1'b1),
            .DROP_OVERSIZE_FRAME(1'b1)) tx_cdc_fifo (
            .s_clk(core_clk), .s_rst(core_rst), .s_axis(core_tx_axis),
            .m_clk(clk_125mhz), .m_rst(rst_125mhz), .m_axis(axis_tx)
        );
        taxi_axis_async_fifo #(.DEPTH(2048), .FRAME_FIFO(1'b1),
            .DROP_OVERSIZE_FRAME(1'b1), .DROP_BAD_FRAME(1'b1)) rx_cdc_fifo (
            .s_clk(clk_125mhz), .s_rst(rst_125mhz), .s_axis(axis_rx),
            .m_clk(core_clk), .m_rst(core_rst), .m_axis(core_rx_axis)
        );
        taxi_axis_async_fifo #(.DEPTH(32), .FRAME_FIFO(1'b1)) owner_cdc_fifo (
            .s_clk(core_clk), .s_rst(core_rst), .s_axis(owner_core_axis),
            .m_clk(clk_125mhz), .m_rst(rst_125mhz), .m_axis(owner_mac_axis)
        );

        assign axis_tx_cpl.tready = owner_mac_axis.tvalid;
        assign owner_mac_axis.tready = axis_tx_cpl.tvalid;

        reg tx_cpl_toggle, tx_cpl_pending;
        always @(posedge clk_125mhz) begin
            if (rst_125mhz) tx_cpl_toggle <= 1'b0;
            else if (axis_tx_cpl.tvalid && axis_tx_cpl.tready &&
                     owner_mac_axis.tdata[0])
                tx_cpl_toggle <= ~tx_cpl_toggle;
        end
        // ── Race audit 2026-09-18 ────────────────────────────────────────
        // The synchroniser + edge detector used to be inline here, with the
        // chain reset to 0 by core_rst while tx_cpl_toggle -- whose only
        // reset is rst_125mhz, i.e. ~mmcm_locked of an MMCM with .RST tied
        // to 0 -- kept its parity across every platform reset.  Half of all
        // core resets taken after any TX traffic therefore manufactured a
        // PHANTOM transmit completion as the chain refilled, latching
        // tx_cpl_pending and leaving the completion stream permanently one
        // ahead (the first real transmit afterwards writes back TCR_PTX
        // before its frame has left the MAC).  See q700_toggle_rx above for
        // the full argument and the negative control.
        wire tx_cpl_edge;
        q700_toggle_rx #(.W(1), .SYNC_FF(3)) u_tx_cpl_rx (
            .clk(core_clk), .rst(core_rst),
            .src_toggle(tx_cpl_toggle), .pulse(tx_cpl_edge)
        );
        always @(posedge core_clk) begin
            if (core_rst) begin
                tx_cpl_pending <= 0;
            end else begin
                // In loopback the MAC never transmits, so its completion
                // toggle never arrives.  Raise completion on the looped
                // frame's last accepted beat instead, so the TX engine still
                // writes back TCR_PTX exactly as MAME does (its transmit
                // completion path is not special-cased for loopback).
                if (tx_cpl_edge || sonic_loopback_cpl)
                    tx_cpl_pending <= 1'b1;
                else if (tx_cpl_pending && sonic_tx_cpl_ready) tx_cpl_pending <= 1'b0;
            end
        end
        assign sonic_tx_cpl_valid = tx_cpl_pending;
        assign rx_activity = axis_rx.tvalid && axis_rx.tready;
        assign tx_activity = axis_tx.tvalid && axis_tx.tready;
    end endgenerate
`ifndef SYNTHESIS
    initial if (REF_CLK_MHZ != 100 && REF_CLK_MHZ != 200)
        $error("q700_eth_link REF_CLK_MHZ must be 100 or 200");
`endif
endmodule
`endif

`default_nettype wire
