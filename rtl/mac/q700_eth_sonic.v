// MAME reference/excerpt/adaptation attribution: Copyright Patrick Mackinlay.
// MAME-derived portions retain BSD-3-Clause; original contributions are MIT.
// See THIRD_PARTY_NOTICES.md and LICENSES/MAME-BSD-3-Clause.txt.
// q700_eth_sonic.v - Quadra 700 Ethernet PROM + DP83932C SONIC register RTL.
//
// The Q700 has two ROM-visible network windows:
//   0x5000_8000..0x5000_8007  Ethernet MAC PROM/config bytes
//   0x5000_A000..0x5000_B0FF  National DP83932C SONIC registers
//
// The SONIC is on D15..D0 of the Q700's 32-bit bus.  The Q700 bus adapter
// turns each four-byte CPU slot into one native 16-bit register transaction;
// byte writes are represented by the two byte strobes below.
//
// Descriptor and packet movement are deliberately delegated to a core-clock
// engine.  PACKET_ENGINE enables a held-valid command/completion seam for that
// engine; the default retains the probe-stage immediate completion behavior.
// The register file,
// reset values, writable masks, command/reset behavior, and ISR/IMR interrupt
// generation follow MAME's dp83932c_device closely enough for ROM probing and
// for early driver setup to observe owned hardware state.

`default_nettype none

module q700_eth_sonic #(
    // The Q700 MAC PROM stores bytes with the bit order used by MAME's
    // macquadra700.cpp machine_start().  These defaults correspond to an
    // Apple 00:05:02 OUI with zero low bytes after the PROM bit swizzle.
    parameter [7:0] MAC0 = 8'h00,
    parameter [7:0] MAC1 = 8'hA0,
    parameter [7:0] MAC2 = 8'h40,
    parameter [7:0] MAC3 = 8'h00,
    parameter [7:0] MAC4 = 8'h00,
    parameter [7:0] MAC5 = 8'h00,
    parameter integer PACKET_ENGINE = 0
) (
    input  wire        clk,
    input  wire        rst,

    input  wire        enet_cs,
    input  wire        enet_rd,
    input  wire        enet_wr,
    input  wire [2:0]  enet_addr,
    input  wire [7:0]  enet_wdata,
    output reg  [7:0]  enet_rdata,
    output wire        enet_ack,

    input  wire        sonic_cs,
    input  wire        sonic_rd,
    input  wire        sonic_wr,
    input  wire [5:0]  sonic_addr,
    input  wire [15:0] sonic_wdata,
    input  wire [1:0]  sonic_wstrb,
    output wire [15:0] sonic_rdata,
    output wire        sonic_ack,
    output wire        sonic_irq,
    output wire [15:0] dbg_cr,
    output wire [15:0] dbg_dcr,
    output wire [15:0] dbg_imr,
    output wire [15:0] dbg_isr,

    output reg         tx_cmd_valid,
    input  wire        tx_cmd_ready,
    output reg  [15:0] tx_cmd_dcr,
    output reg  [15:0] tx_cmd_utda,
    output reg  [15:0] tx_cmd_ctda,
    input  wire        tx_done_valid,
    output wire        tx_done_ready,
    input  wire        tx_done_error,
    input  wire        tx_done_pint,
    input  wire [15:0] tx_done_ctda,
    input  wire [15:0] tx_done_tcr,
    input  wire [15:0] tx_done_tps,
    input  wire [15:0] tx_done_tfc,

    output wire        rx_enabled,
    // CR_HTX out to the transmit engine.  The CR bit alone halted nothing --
    // the engine kept walking the descriptor list to end-of-list.
    output wire        tx_halt,
    output reg         rx_cfg_valid,
    input  wire        rx_cfg_ready,
    output reg  [2:0]  rx_cfg_op,
    output reg  [15:0] rx_cfg_dcr, rx_cfg_rcr, rx_cfg_urda, rx_cfg_crda,
    output reg  [15:0] rx_cfg_urra, rx_cfg_rsa, rx_cfg_rea, rx_cfg_rrp,
    output reg  [15:0] rx_cfg_rwp, rx_cfg_eobc, rx_cfg_rsc, rx_cfg_llfa,
    output reg  [15:0] rx_cfg_cdp, rx_cfg_cdc,
    input  wire        rx_done_valid,
    output wire        rx_done_ready,
    input  wire        rx_done_error,
    input  wire [15:0] rx_done_rcr, rx_done_crda, rx_done_crba0, rx_done_crba1,
    input  wire [15:0] rx_done_rbwc0, rx_done_rbwc1, rx_done_rrp, rx_done_rsc,
    input  wire [15:0] rx_done_llfa, rx_done_trba0, rx_done_trba1,
    input  wire [15:0] rx_done_tbwc0, rx_done_tbwc1, rx_done_isr_set,
    input  wire [15:0] rx_done_cdp, rx_done_cdc, rx_done_ce
);

    localparam [5:0]
        REG_CR    = 6'h00,
        REG_DCR   = 6'h01,
        REG_RCR   = 6'h02,
        REG_TCR   = 6'h03,
        REG_IMR   = 6'h04,
        REG_ISR   = 6'h05,
        REG_EOBC  = 6'h13,
        REG_CE    = 6'h25,
        REG_CDP   = 6'h26,
        REG_CDC   = 6'h27,
        REG_SR    = 6'h28,
        REG_RSC   = 6'h2B,
        REG_CRCT  = 6'h2C,
        REG_FAET  = 6'h2D,
        REG_MPT   = 6'h2E,
        REG_DCR2  = 6'h3F;

    localparam [15:0]
        CR_HTX   = 16'h0001,
        CR_TXP   = 16'h0002,
        CR_RXDIS = 16'h0004,
        CR_RXEN  = 16'h0008,
        CR_STP   = 16'h0010,
        CR_ST    = 16'h0020,
        CR_RST   = 16'h0080,
        CR_RRRA  = 16'h0100,
        CR_LCAM  = 16'h0200,
        TCR_PTX  = 16'h0001,
        TCR_NCRS = 16'h0100,
        TCR_PINT = 16'h8000,
        ISR_PINT = 16'h0800,
        ISR_TXDN = 16'h0200,
        // 0x0100, NOT 0x0400.  0x0400 is PKTRX -- and q700_sonic_rx.sv:70
        // already calls it that, so the two files contradicted each other.
        // With the wrong value a transmit ERROR raised "packet received":
        // the driver goes hunting in the RX ring, finds nothing, and never
        // learns the transmit failed, while TXER itself could never set.
        // MAME dp83932c.h:191-193 is authoritative here.
        ISR_TXER = 16'h0100,
        ISR_RBE  = 16'h0020,
        ISR_RDE  = 16'h0040,
        ISR_LCD  = 16'h1000;

    localparam [5:0]
        REG_UTDA = 6'h06,
        REG_CTDA = 6'h07,
        REG_TPS  = 6'h08,
        REG_TFC  = 6'h09,
        REG_TTDA = 6'h20;
    localparam [5:0] REG_URDA=6'h0d,REG_CRDA=6'h0e,REG_CRBA0=6'h0f,
        REG_CRBA1=6'h10,REG_RBWC0=6'h11,REG_RBWC1=6'h12,
        REG_URRA=6'h14,REG_RSA=6'h15,REG_REA=6'h16,REG_RRP=6'h17,
        REG_RWP=6'h18,REG_TRBA0=6'h19,REG_TRBA1=6'h1a,
        REG_TBWC0=6'h1b,REG_TBWC1=6'h1c,REG_LLFA=6'h1f;

    reg [15:0] sonic_reg [0:63];
    reg [1:0] rx_recovery_pending;
    reg rrra_inflight, cam_inflight;
    reg rcr_dirty;
    integer i;

    function [7:0] enet_prom_byte;
        input [2:0] idx;
        reg [7:0] xor_total;
        begin
            xor_total = MAC0 ^ MAC1 ^ MAC2 ^ MAC3 ^ MAC4 ^ MAC5;
            case (idx)
                3'd0: enet_prom_byte = MAC0;
                3'd1: enet_prom_byte = MAC1;
                3'd2: enet_prom_byte = MAC2;
                3'd3: enet_prom_byte = MAC3;
                3'd4: enet_prom_byte = MAC4;
                3'd5: enet_prom_byte = MAC5;
                3'd7: enet_prom_byte = xor_total ^ 8'hFF;
                default: enet_prom_byte = 8'h00;
            endcase
        end
    endfunction

    function [15:0] sonic_reset_value;
        input [5:0] idx;
        begin
            case (idx)
                REG_CR:   sonic_reset_value = CR_RST | CR_STP | CR_RXDIS;
                REG_TCR:  sonic_reset_value = TCR_NCRS | TCR_PTX;
                REG_EOBC: sonic_reset_value = 16'h02F8;
                REG_SR:   sonic_reset_value = 16'h0006;
                default:  sonic_reset_value = 16'h0000;
            endcase
        end
    endfunction

    function [15:0] sonic_reg_mask;
        input [5:0] idx;
        begin
            case (idx)
                6'h00: sonic_reg_mask = 16'h03BF;
                6'h01: sonic_reg_mask = 16'hBFFF;
                6'h02: sonic_reg_mask = 16'hFE00;
                6'h03: sonic_reg_mask = 16'hF000;
                6'h04: sonic_reg_mask = 16'h7FFF;
                6'h05: sonic_reg_mask = 16'h7FFF;
                6'h06: sonic_reg_mask = 16'hFFFF;
                6'h07: sonic_reg_mask = 16'hFFFF;
                6'h08: sonic_reg_mask = 16'hFFFF;
                6'h09: sonic_reg_mask = 16'hFFFF;
                6'h0A: sonic_reg_mask = 16'hFFFF;
                6'h0B: sonic_reg_mask = 16'hFFFF;
                6'h0C: sonic_reg_mask = 16'hFFFF;
                6'h0D: sonic_reg_mask = 16'hFFFF;
                6'h0E: sonic_reg_mask = 16'hFFFF;
                6'h0F: sonic_reg_mask = 16'hFFFF;
                6'h10: sonic_reg_mask = 16'hFFFF;
                6'h11: sonic_reg_mask = 16'hFFFF;
                6'h12: sonic_reg_mask = 16'hFFFF;
                6'h13: sonic_reg_mask = 16'hFFFF;
                6'h14: sonic_reg_mask = 16'hFFFF;
                6'h15: sonic_reg_mask = 16'hFFFE;
                6'h16: sonic_reg_mask = 16'hFFFE;
                6'h17: sonic_reg_mask = 16'hFFFE;
                6'h18: sonic_reg_mask = 16'hFFFE;
                6'h19: sonic_reg_mask = 16'hFFFF;
                6'h1A: sonic_reg_mask = 16'hFFFF;
                6'h1B: sonic_reg_mask = 16'hFFFF;
                6'h1C: sonic_reg_mask = 16'hFFFF;
                6'h1D: sonic_reg_mask = 16'hFFFF;
                6'h1E: sonic_reg_mask = 16'hFFFF;
                6'h1F: sonic_reg_mask = 16'hFFFF;
                6'h20: sonic_reg_mask = 16'hFFFF;
                6'h21: sonic_reg_mask = 16'h000F;
                6'h22: sonic_reg_mask = 16'h0000;
                6'h23: sonic_reg_mask = 16'h0000;
                6'h24: sonic_reg_mask = 16'h0000;
                6'h25: sonic_reg_mask = 16'hFFFF;
                6'h26: sonic_reg_mask = 16'hFFFE;
                6'h27: sonic_reg_mask = 16'h001F;
                6'h28: sonic_reg_mask = 16'h0000;
                6'h29: sonic_reg_mask = 16'hFFFF;
                6'h2A: sonic_reg_mask = 16'hFFFF;
                6'h2B: sonic_reg_mask = 16'hFFFF;
                6'h2C: sonic_reg_mask = 16'hFFFF;
                6'h2D: sonic_reg_mask = 16'hFFFF;
                6'h2E: sonic_reg_mask = 16'hFFFF;
                6'h2F: sonic_reg_mask = 16'h0000;
                6'h3F: sonic_reg_mask = 16'hF017;
                default: sonic_reg_mask = 16'hFFFF;
            endcase
        end
    endfunction

    wire [5:0] sonic_idx = sonic_addr;
    wire [15:0] sonic_word = sonic_reg[sonic_idx];

    assign enet_ack = enet_cs && (enet_rd || enet_wr);
    assign sonic_ack = sonic_cs && (sonic_rd || sonic_wr);
    assign sonic_rdata = (sonic_cs && sonic_rd) ? sonic_word : 16'hFFFF;
    assign sonic_irq = |(sonic_reg[REG_ISR] & sonic_reg[REG_IMR]);
    assign dbg_cr = sonic_reg[REG_CR];
    assign dbg_dcr = sonic_reg[REG_DCR];
    assign dbg_imr = sonic_reg[REG_IMR];
    assign dbg_isr = sonic_reg[REG_ISR];
    assign tx_done_ready = (PACKET_ENGINE != 0);
    assign rx_done_ready = (PACKET_ENGINE != 0);
    // A software reset must stop the receiver, not just the command path.
    // CR_RST deliberately leaves RXEN set (ds:1585 -- the reset mask touches
    // only bits 9,8,1,0 and 7,2), so gating on RXEN alone left the RX engine
    // accepting frames and DMAing them against the CRBA/CRDA the driver had
    // already disowned.  That is the same failure class as the 65535-RX-DMA-op
    // storm at a garbage CRDA seen on hardware: the datasheet is explicit that
    // "a software reset immediately terminates DMA operations" (ds:3459-3462).
    assign tx_halt = (sonic_reg[REG_CR] & CR_HTX) != 16'h0000;
    assign rx_enabled = ((sonic_reg[REG_CR] & CR_RXEN) != 0) &&
                        ((sonic_reg[REG_CR] & CR_RST) == 0);
    wire [15:0] isr_bus_clear =
        (sonic_cs && sonic_wr && sonic_idx == REG_ISR) ?
        (sonic_wdata & {{8{sonic_wstrb[1]}}, {8{sonic_wstrb[0]}}} &
         sonic_reg_mask(REG_ISR)) : 16'h0000;
    // MAME raises the programmable interrupt from the descriptor's TCR_PINT
    // alongside the normal completion bit, not instead of it.
    // PINT is EDGE-triggered across descriptors, not level.  ds:1930-1936:
    // "PINT in the Transmit Control Register must be cleared before it is set
    // again in order to have the interrupt issued for another packet".  A
    // driver that leaves PINT set in consecutive TDAs must get ONE interrupt,
    // not one per packet.  sonic_reg[REG_TCR] still holds the PREVIOUS
    // descriptor's config half at this instant (it is updated in this same
    // cycle by the writeback below), so it is exactly MAME's `u16 const tcr =
    // m_reg[TCR]` snapshot at mame.cpp:368-377.
    //
    // Residual, deliberately not fixed here: the datasheet raises PINT on
    // reading the TDA, whereas this raises it at completion -- one transmit
    // late.  Correcting that needs a separate event path out of the header
    // state; the storm is the driver-visible half and this stops it.
    wire tx_pint_edge = tx_done_pint &&
                        ((sonic_reg[REG_TCR] & TCR_PINT) == 16'h0000);
    wire [15:0] tx_isr_event = tx_done_valid ?
        ((tx_done_error ? ISR_TXER : ISR_TXDN) |
         (tx_pint_edge ? ISR_PINT : 16'h0000)) : 16'h0000;
    wire [15:0] rx_isr_event = rx_done_valid ? rx_done_isr_set : 16'h0000;

    // A CPU write to CR and an engine writeback can land in the same pb_clk
    // cycle.  Both are nonblocking assignments to sonic_reg[REG_CR] in the one
    // always block, so the later statement wins OUTRIGHT and the earlier one
    // vanishes -- and the CPU write is the earlier statement.  Dropping a
    // command that way is not a lost status bit, it is a wedge: a swallowed
    // CR_TXP means the driver waits forever for a TXDN that can never come.
    // So the two are merged instead of raced -- the CPU write carries the
    // engine's clears with it, and the engine defers when the CPU is writing.
    wire        cpu_wr_cr = sonic_cs && sonic_wr && (sonic_idx == REG_CR);
    wire [15:0] cr_engine_clear =
        (tx_done_valid ? CR_TXP : 16'h0000) |
        ((rx_done_valid && (rx_done_isr_set & ISR_LCD) != 0) ? CR_LCAM : 16'h0000) |
        ((rx_done_valid && (rx_done_isr_set & ISR_LCD) == 0 && rrra_inflight)
             ? CR_RRRA : 16'h0000);

    always @(*) begin
        enet_rdata = (enet_cs && enet_rd) ? enet_prom_byte(enet_addr) : 8'hFF;
    end

    /* verilator lint_off BLKSEQ */
    task sonic_enter_reset;
        begin
            sonic_reg[REG_CR]  <= (sonic_reg[REG_CR] &
                                   ~(CR_LCAM | CR_RRRA | CR_TXP | CR_HTX)) |
                                  CR_RST | CR_RXDIS;
            // A SOFTWARE reset touches CR and nothing else.  IMR/ISR/CE/RSC/
            // DCR2 are cleared by POWER-ON reset only -- MAME splits the two
            // (device_reset() clears them; the CR_RST arm of reg_w() does not),
            // and the power-on path here gets them from sonic_reset_value().
            // Clearing IMR on the software path silently disarmed every
            // interrupt the moment a driver reset the chip after programming
            // it, which is why IMR read back 0x0000 on hardware and no
            // delivered frame was ever serviced.
            // A software reset must also cancel receive-side work that is
            // queued or believed in flight.  Leaving these set let the engine
            // keep issuing descriptor DMA against URDA/CRDA the driver had not
            // reprogrammed yet -- observed on hardware as 65535+ RX DMA ops
            // for 129 frames, aimed at a garbage CRDA (0x002D), while CR still
            // read RST|RXDIS and DCR was 0.  S_CLEAR_REQ in that loop is a
            // WRITE, so it scribbles into whatever guest address the stale
            // pointers happen to name.
            rx_recovery_pending <= 2'b00;
            rrra_inflight <= 1'b0;
            cam_inflight <= 1'b0;
            rcr_dirty <= 1'b0;
            // ...and the transmit side, for the same reason.  MAME cancels the
            // whole pending-command timer here.  A queued TX left armed across
            // a reset walks a descriptor list the driver has already disowned
            // and reports its completion into TCR/TPS/CTDA afterwards.
            tx_cmd_valid <= 1'b0;
        end
    endtask

    task sonic_command_low;
        input [7:0] data;
        reg [15:0] cr_next;
        begin
            if ((sonic_reg[REG_CR] & CR_RST) != 16'h0000) begin
                if (!data[7])
                    sonic_reg[REG_CR] <= sonic_reg[REG_CR] & ~CR_RST & ~cr_engine_clear;
            end else if (data[7]) begin
                sonic_enter_reset();
            end else begin
                cr_next = sonic_reg[REG_CR] | ({8'h00, data} & sonic_reg_mask(REG_CR));

                if (data[0])
                    cr_next = cr_next & ~CR_TXP;
                if (data[2])
                    cr_next = cr_next & ~CR_RXEN;
                if (data[3])
                    cr_next = cr_next & ~CR_RXDIS;
                if (data[4])
                    cr_next = cr_next & ~CR_ST;
                if (data[5])
                    cr_next = cr_next & ~CR_STP;

                if (data[1]) begin
                    // MAME's command() clears HTX on TXP and TXP on HTX; we
                    // had only the second direction, leaving HTX stuck set in
                    // the CR readback forever once a driver had used it.
                    cr_next = cr_next & ~CR_HTX;
                    if (PACKET_ENGINE != 0) begin
                        if ((sonic_reg[REG_CR] & CR_TXP) == 0 && !tx_cmd_valid) begin
                            tx_cmd_dcr <= sonic_reg[REG_DCR];
                            tx_cmd_utda <= sonic_reg[REG_UTDA];
                            tx_cmd_ctda <= sonic_reg[REG_CTDA];
                            sonic_reg[REG_TTDA] <= sonic_reg[REG_CTDA];
                            tx_cmd_valid <= 1'b1;
                        end
                    end else begin
                        cr_next = cr_next & ~CR_TXP;
                        sonic_reg[REG_TCR] <= (sonic_reg[REG_TCR] | TCR_PTX);
                        sonic_reg[REG_ISR] <= (sonic_reg[REG_ISR] | ISR_TXDN);
                    end
                end

                sonic_reg[REG_CR] <= cr_next & ~cr_engine_clear;
            end
        end
    endtask

    task sonic_write_selected_byte;
        input [5:0] idx;
        input       low_byte;
        input [7:0] data;
        reg [15:0] byte_mask;
        reg [15:0] data_word;
        reg [15:0] writable;
        reg [15:0] cr_next;
        begin
            byte_mask = low_byte ? 16'h00FF : 16'hFF00;
            data_word = low_byte ? {8'h00, data} : {data, 8'h00};
            writable = sonic_reg_mask(idx) & byte_mask;

            case (idx)
                REG_CR: begin
                    if (low_byte) begin
                        sonic_command_low(data);
                    end else if ((sonic_reg[REG_CR] & CR_RST) != 0) begin
                        // ds:1578-1580: "Before any commands can be issued,
                        // the RST bit must first be reset to 0".  The 16-bit
                        // write path gates on this; the high-BYTE path did
                        // not, so LCAM/RRRA could be armed mid-reset.
                        sonic_reg[REG_CR] <= sonic_reg[REG_CR];
                    end else begin
                        cr_next = sonic_reg[REG_CR] |
                                  ({data, 8'h00} & sonic_reg_mask(REG_CR));
                        if (data[0] && PACKET_ENGINE == 0)
                            cr_next = cr_next & ~CR_RRRA;
                        if (data[1]) begin
                            if (PACKET_ENGINE == 0) begin
                                cr_next = cr_next & ~CR_LCAM;
                                sonic_reg[REG_ISR] <= sonic_reg[REG_ISR] | ISR_LCD;
                            end
                        end
                        sonic_reg[REG_CR] <= cr_next & ~cr_engine_clear;
                    end
                end
                REG_RCR: begin
                    sonic_reg[idx] <= (sonic_reg[idx] & ~writable) |
                                      (data_word & writable);
                    // Only a SOFTWARE write makes the engine's filter stale.
                    // Do not compare register values instead: the engine writes
                    // per-frame receive status back into RCR, so a value
                    // comparison desynchronises after every frame.
                    rcr_dirty <= 1'b1;
                end
                REG_IMR: begin
                    sonic_reg[idx] <= (sonic_reg[idx] & ~writable) |
                                      (data_word & writable);
                end
                REG_ISR: begin
                    // RBE clear fetches the newly replenished resource;
                    // RDE clear reloads the descriptor link at URDA:LLFA.
                    if (PACKET_ENGINE != 0)
                        rx_recovery_pending <= rx_recovery_pending |
                            {|(data_word&writable&sonic_reg[REG_ISR]&ISR_RDE),
                             |(data_word&writable&sonic_reg[REG_ISR]&ISR_RBE)};
                    if (PACKET_ENGINE == 0)
                        sonic_reg[idx] <= sonic_reg[idx] & ~(data_word & writable);
                end
                REG_CRCT,
                REG_FAET,
                REG_MPT: begin
                    sonic_reg[idx] <= (sonic_reg[idx] & ~byte_mask) |
                                      ((~data_word) & byte_mask);
                end
                REG_SR,
                REG_CE,
                REG_RSC,
                REG_DCR2: begin
                    if (writable != 16'h0000)
                        sonic_reg[idx] <= (sonic_reg[idx] & ~writable) |
                                          (data_word & writable);
                end
                default: begin
                    if (writable != 16'h0000)
                        sonic_reg[idx] <= (sonic_reg[idx] & ~writable) |
                                          (data_word & writable);
                end
            endcase
        end
    endtask

    // Apply a native register write atomically.  Single-byte accesses retain
    // the established command semantics; the common two-byte case merges the
    // writable mask once so two nonblocking byte updates cannot overwrite one
    // another.
    task sonic_write_native;
        input [5:0] idx;
        input [15:0] data;
        input [1:0] strb;
        reg [15:0] writable;
        reg [15:0] cr_next;
        begin
            if (strb == 2'b10)
                sonic_write_selected_byte(idx, 1'b0, data[15:8]);
            else if (strb == 2'b01)
                sonic_write_selected_byte(idx, 1'b1, data[7:0]);
            else if (strb == 2'b11) begin
                writable = sonic_reg_mask(idx);
                // RCR has no dedicated arm below -- it lands in `default` --
                // so mark the engine's filter stale here, where every 16-bit
                // write is seen regardless of which arm handles it.
                if (idx == REG_RCR) rcr_dirty <= 1'b1;
                case (idx)
                    REG_CR: begin
                        if ((sonic_reg[REG_CR] & CR_RST) != 0) begin
                            if (!data[7])
                                sonic_reg[REG_CR] <= sonic_reg[REG_CR] & ~CR_RST & ~cr_engine_clear;
                        end else if (data[7]) begin
                            sonic_enter_reset();
                        end else begin
                            cr_next = sonic_reg[REG_CR] | (data & writable);
                            if (data[0]) cr_next = cr_next & ~CR_TXP;
                            if (data[2]) cr_next = cr_next & ~CR_RXEN;
                            if (data[3]) cr_next = cr_next & ~CR_RXDIS;
                            if (data[4]) cr_next = cr_next & ~CR_ST;
                            if (data[5]) cr_next = cr_next & ~CR_STP;
                            if (data[8] && PACKET_ENGINE == 0)
                                cr_next = cr_next & ~CR_RRRA;
                            if (data[9] && PACKET_ENGINE == 0) begin
                                cr_next = cr_next & ~CR_LCAM;
                                sonic_reg[REG_ISR] <= sonic_reg[REG_ISR] | ISR_LCD;
                            end
                            if (data[1]) begin
                                cr_next = cr_next & ~CR_HTX;
                                if (PACKET_ENGINE != 0) begin
                                    if ((sonic_reg[REG_CR] & CR_TXP) == 0 && !tx_cmd_valid) begin
                                        tx_cmd_dcr <= sonic_reg[REG_DCR];
                                        tx_cmd_utda <= sonic_reg[REG_UTDA];
                                        tx_cmd_ctda <= sonic_reg[REG_CTDA];
                                        sonic_reg[REG_TTDA] <= sonic_reg[REG_CTDA];
                                        tx_cmd_valid <= 1'b1;
                                    end
                                end else begin
                                    cr_next = cr_next & ~CR_TXP;
                                    sonic_reg[REG_TCR] <= sonic_reg[REG_TCR] | TCR_PTX;
                                    sonic_reg[REG_ISR] <= sonic_reg[REG_ISR] | ISR_TXDN;
                                end
                            end
                            sonic_reg[REG_CR] <= cr_next & ~cr_engine_clear;
                        end
                    end
                    REG_ISR: begin
                        if (PACKET_ENGINE != 0)
                            rx_recovery_pending <= rx_recovery_pending |
                                {|(data&writable&sonic_reg[REG_ISR]&ISR_RDE),
                                 |(data&writable&sonic_reg[REG_ISR]&ISR_RBE)};
                        if (PACKET_ENGINE == 0)
                            sonic_reg[idx] <= sonic_reg[idx] & ~(data & writable);
                    end
                    REG_CRCT, REG_FAET, REG_MPT:
                        sonic_reg[idx] <= ~data;
                    default: begin
                        if (writable != 16'h0000)
                            sonic_reg[idx] <= (sonic_reg[idx] & ~writable) |
                                              (data & writable);
                    end
                endcase
            end
        end
    endtask
    /* verilator lint_on BLKSEQ */

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 64; i = i + 1)
                sonic_reg[i] <= sonic_reset_value(i[5:0]);
            tx_cmd_valid <= 1'b0;
            tx_cmd_dcr <= 16'd0;
            tx_cmd_utda <= 16'd0;
            tx_cmd_ctda <= 16'd0;
            rx_cfg_valid<=0; rx_cfg_op<=0; rx_cfg_dcr<=0; rx_cfg_rcr<=0; rx_cfg_urda<=0; rx_cfg_crda<=0;
            rx_cfg_urra<=0; rx_cfg_rsa<=0; rx_cfg_rea<=0; rx_cfg_rrp<=0;
            rx_cfg_rwp<=0; rx_cfg_eobc<=0; rx_cfg_rsc<=0; rx_cfg_llfa<=0;
            rx_cfg_cdp<=0; rx_cfg_cdc<=0; rx_recovery_pending<=0;
            rrra_inflight<=0; cam_inflight<=0; rcr_dirty<=0;
        end else begin
            if (sonic_cs && sonic_wr)
                sonic_write_native(sonic_idx, sonic_wdata, sonic_wstrb);
            if (PACKET_ENGINE != 0) begin
                if (tx_cmd_valid && tx_cmd_ready)
                    tx_cmd_valid <= 1'b0;
                if (rx_cfg_valid && rx_cfg_ready) begin
                    rx_cfg_valid<=1'b0;
                end
                // CR command bits and ISR recovery requests form a small
                // pending-command queue.  This avoids losing a command when
                // software writes LCAM/RRRA while the CDC mailbox is busy.
                // CR_RST gates the whole issue path, not just the data
                // path: a SONIC in software reset is not a bus master, and
                // its URDA/CRDA/URRA are whatever the last driver left behind.
                if (!rx_cfg_valid && (sonic_reg[REG_CR] & CR_RST) == 0 &&
                    !(sonic_cs && sonic_wr &&
                      sonic_idx == REG_ISR)) begin
                    if ((sonic_reg[REG_CR] & CR_LCAM) != 0 && !cam_inflight) begin
                        rx_cfg_op<=3'b100; cam_inflight<=1'b1;
                        rx_cfg_valid<=1'b1;
                    end else if ((sonic_reg[REG_CR] & CR_RRRA) != 0 && !rrra_inflight) begin
                        rx_cfg_op<=3'b001; rrra_inflight<=1'b1;
                        rx_cfg_valid<=1'b1;
                    end else if (rx_recovery_pending != 0) begin
                        rx_cfg_op<={1'b0,rx_recovery_pending};
                        rx_recovery_pending<=0;
                        rx_cfg_valid<=1'b1;
                    end else if (rcr_dirty) begin
                        // rx_cfg_rcr holds what the engine was last told, so a
                        // mismatch means its receive filter is stale.  Push a
                        // no-DMA refresh (op 0) rather than let it keep
                        // filtering on a value the driver has since changed.
                        rcr_dirty<=1'b0;
                        rx_cfg_op<=3'b000;
                        rx_cfg_valid<=1'b1;
                    end
                    if (((sonic_reg[REG_CR] & CR_LCAM) != 0 && !cam_inflight) ||
                        ((sonic_reg[REG_CR] & CR_RRRA) != 0 && !rrra_inflight) ||
                        (rx_recovery_pending != 0) ||
                        rcr_dirty) begin
                        rx_cfg_dcr<=sonic_reg[REG_DCR]; rx_cfg_rcr<=sonic_reg[REG_RCR];
                        rx_cfg_urda<=sonic_reg[REG_URDA]; rx_cfg_crda<=sonic_reg[REG_CRDA];
                        rx_cfg_urra<=sonic_reg[REG_URRA]; rx_cfg_rsa<=sonic_reg[REG_RSA];
                        rx_cfg_rea<=sonic_reg[REG_REA]; rx_cfg_rrp<=sonic_reg[REG_RRP];
                        rx_cfg_rwp<=sonic_reg[REG_RWP]; rx_cfg_eobc<=sonic_reg[REG_EOBC];
                        rx_cfg_rsc<=sonic_reg[REG_RSC]; rx_cfg_llfa<=sonic_reg[REG_LLFA];
                        rx_cfg_cdp<=sonic_reg[REG_CDP]; rx_cfg_cdc<=sonic_reg[REG_CDC];
                    end
                end
                if (tx_done_valid) begin
                    if (!cpu_wr_cr)
                        sonic_reg[REG_CR] <= sonic_reg[REG_CR] & ~CR_TXP;
                    sonic_reg[REG_CTDA] <= tx_done_ctda;
                    sonic_reg[REG_TCR] <= tx_done_tcr;
                    sonic_reg[REG_TPS] <= tx_done_tps;
                    sonic_reg[REG_TFC] <= tx_done_tfc;
                end
                if (rx_done_valid) begin
                    if ((rx_done_isr_set & ISR_LCD) != 0) begin
                        sonic_reg[REG_CDP]<=rx_done_cdp;
                        sonic_reg[REG_CDC]<=rx_done_cdc;
                        sonic_reg[REG_CE]<=rx_done_ce;
                        if (!cpu_wr_cr)
                            sonic_reg[REG_CR]<=sonic_reg[REG_CR]&~CR_LCAM&
                                ~(tx_done_valid ? CR_TXP : 16'h0000);
                        cam_inflight<=1'b0;
                    end else begin
                        if (rrra_inflight) begin
                            if (!cpu_wr_cr)
                                sonic_reg[REG_CR]<=sonic_reg[REG_CR]&~CR_RRRA&
                                    ~(tx_done_valid ? CR_TXP : 16'h0000);
                            rrra_inflight<=1'b0;
                        end
                        // RCR is written by the driver at runtime, unlike the
                        // pointer/status registers below, which move only while
                        // the receiver is stopped.  Let the CPU write stand: it
                        // also sets rcr_dirty, so the engine is refreshed with
                        // the merged value on the very next issue slot.
                        if (!(sonic_cs && sonic_wr && sonic_idx == REG_RCR))
                            sonic_reg[REG_RCR]<=rx_done_rcr;
                        sonic_reg[REG_CRDA]<=rx_done_crda;
                        sonic_reg[REG_CRBA0]<=rx_done_crba0; sonic_reg[REG_CRBA1]<=rx_done_crba1;
                        sonic_reg[REG_RBWC0]<=rx_done_rbwc0; sonic_reg[REG_RBWC1]<=rx_done_rbwc1;
                        sonic_reg[REG_RRP]<=rx_done_rrp; sonic_reg[REG_RSC]<=rx_done_rsc;
                        sonic_reg[REG_LLFA]<=rx_done_llfa; sonic_reg[REG_TRBA0]<=rx_done_trba0;
                        sonic_reg[REG_TRBA1]<=rx_done_trba1; sonic_reg[REG_TBWC0]<=rx_done_tbwc0;
                        sonic_reg[REG_TBWC1]<=rx_done_tbwc1;
                    end
                end
                if (isr_bus_clear != 0 || tx_done_valid || rx_done_valid)
                    sonic_reg[REG_ISR] <= (sonic_reg[REG_ISR] & ~isr_bus_clear) |
                        tx_isr_event | rx_isr_event;
            end
        end
    end

    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused_enet_wdata = |enet_wdata;
    /* verilator lint_on UNUSEDSIGNAL */

endmodule

`default_nettype wire
