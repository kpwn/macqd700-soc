// MAME reference/excerpt/adaptation attribution: Copyright Olivier Galibert.
// MAME-derived portions retain BSD-3-Clause; original contributions are MIT.
// See THIRD_PARTY_NOTICES.md and LICENSES/MAME-BSD-3-Clause.txt.
// iwm_stub.v - Probe-safe SWIM/IWM floppy controller stub.
//
// Purpose
// -------
// This is a narrow register stub for the Quadra-class floppy controller
// window.  The platform peripheral bus wires it as the live SWIM/IWM sink;
// the stand-alone Verilator tests lock down the same conservative contract.
//
// Behaviour
// ---------
// - Reset defaults to IWM-compatible idle state with no IRQ/DMA activity.
// - Reset starts in IWM-compatible mode, where offsets update the IWM
//   phase/control latch before returning data.  This is enough for the
//   Q700 ROM's no-media probe loop: write IWM mode through offset F,
//   select status with offsets D/E, and observe the low status bits.
//   The MAME-observed 0x57,0x17,0x57,0x57 offset-F sequence also selects
//   SWIM mode.  Other SWIM-bank writes are ignored until SWIM mode is
//   selected, except offset 7 which is the conservative ROM-safe SWIM entry
//   hook.
// - When SWIM/ISM mode bit 6 is set, register aliases match the SWIM
//   bank layout:
//     * 0/8 data, 1/9 mark, 2/A error, 3/B parameter, 4/C phase, 5/D setup
//     * 6 readback returns the current mode
//     * 7/F handshake/sense return the no-media indication
// - Mode register access uses clear-on-write / set-on-write semantics
//   matching the SWIM register bank.
// - Reads return benign no-media values for the canonical Q700 power-on
//   state (drive attached but no media inserted):
//     * data/mark/parameter paths never expose a media image
//     * IWM status (control[7:6]=2'b01) reports bit 7 high (write-protect
//       / no-disk sense)
//     * SWIM handshake (offset 7 / F) reports bits 2,3 set (= 0x0c) per
//       MAME swim1_device::ism_read line 233:
//         h |= 0x0c when (!m_floppy || m_floppy->wpt_r())
//       — the 0x0c (NOT 0x08) bit pattern is what makes the Q700 ROM
//       enter the canonical floppy-poll loop and show the ?-floppy icon
//       instead of falling into MacsBug.
//     * `drive_present`=1 (the platform default) selects this MAME-faithful
//       behaviour.  `drive_present`=0 retains the legacy "no drive at all"
//       handshake (0x08, bit 3 only) for backwards-compat with older unit
//       tests.
// - Register writes never raise IRQs or DMA requests.
//
// The `irq` and `dma_req` outputs are HARDWIRED to 1'b0
// ------------------------------------------------------
// This is deliberate and it is a LIMITATION, not an oversight — say so
// loudly, because the consumer side reads as if the interrupt were live.
// rtl/soc/fpga_top_peripherals.vh wires `.ca2_in(~iwm_irq_w)` on VIA2
// under a long (and correct) comment explaining that MAME's
// macquadra700.cpp:875 routes the SWIM hint to VIA2 CA2 and that the
// Q700 ROM programs PCR = 0x22 expecting a negative edge there.  All of
// that is true of the WIRING.  The SOURCE is this file, and it is
// constant: `~iwm_irq_w` is therefore a constant 1'b1, Vivado folds it,
// and VIA2 CA2 can never take an edge.
//
// It is constant because this module has nothing to interrupt ABOUT.  It
// is a register stub for the canonical power-on state "drive attached,
// no media": there is no seek, no read/write engine, no error path, and
// no ISM interrupt-enable decode — none of the conditions MAME's
// swim1_device raises ism_hint for exist here.  Wiring a signal out of a
// module that cannot generate it would be worse than the tie-off, not
// better.
//
// Making the floppy interrupt real is a FEATURE: implement ISM hint
// generation (mode-register interrupt enables plus the ERROR / data /
// handshake conditions), then this assign becomes the real expression
// and the fpga_top seam needs no change at all.  `dma_req` is dead for
// the same reason; `mode_o` is driven but consumed nowhere.
//
// Latency
// -------
// Synchronous writes on the next rising clock edge.  Reads are
// combinational.

`default_nettype none

module iwm_stub (
    input  wire       clk,
    input  wire       rst,
    input  wire       cs,
    input  wire       rd,
    input  wire       wr,
    input  wire [3:0] reg_sel,
    input  wire [7:0] wdata,
    input  wire       drive_present,
    output reg  [7:0] rdata,
    output wire       irq,
    output wire       dma_req,
    output wire [7:0] mode_o
);

    reg [7:0] data_reg;
    reg [7:0] mark_reg;
    reg [7:0] error_reg;
    reg [7:0] phase_reg;
    reg [7:0] setup_reg;
    reg [7:0] mode_reg;
    reg [7:0] iwm_data_reg;
    reg [7:0] iwm_mode_reg;
    reg [7:0] iwm_status_reg;
    reg [7:0] iwm_control_reg;
    reg [7:0] iwm_whd_reg;
    reg       iwm_active_reg;
    reg [1:0] iwm_to_swim_ctr;
    reg [3:0] param_idx;
    reg [7:0] param_ram [0:15];
    integer i;

    // Constant by design — see "The `irq` and `dma_req` outputs are
    // HARDWIRED to 1'b0" in the header.  fpga_top wires irq to VIA2 CA2
    // and the comment there describes the wiring fix, NOT a live
    // interrupt: nothing in this stub can ever raise either line.
    assign irq = 1'b0;
    assign dma_req = 1'b0;
    assign mode_o = mode_reg;

    function [7:0] iwm_read_value;
        input [7:0] control_after;
        input [3:0] phases_after;
        begin
            case (control_after[7:6])
                2'b00: iwm_read_value = (control_after[4] ||
                                          (iwm_active_reg && !iwm_mode_reg[2])) ? iwm_data_reg : 8'hFF;
                2'b01: iwm_read_value = ((iwm_status_reg |
                                           (control_after[4] ? 8'h20 : 8'h00)) &
                                          ((iwm_active_reg && !control_after[4] && iwm_mode_reg[2]) ?
                                           8'hDF : 8'hFF) & 8'h7F) |
                                          (((control_after[4] || (iwm_active_reg && !iwm_mode_reg[2])) &&
                                            !control_after[5] && phases_after[0] && phases_after[1]) ? 8'h00 : 8'h80);
                2'b10: iwm_read_value = iwm_whd_reg;
                2'b11: iwm_read_value = 8'hFF;
                default: iwm_read_value = 8'hFF;
            endcase
        end
    endfunction

    function [3:0] iwm_phases_after_access;
        input [3:0] idx;
        begin
            iwm_phases_after_access = phase_reg[3:0];
            if (idx < 4'h8) begin
                if (idx[0])
                    iwm_phases_after_access[idx[2:1]] = 1'b1;
                else
                    iwm_phases_after_access[idx[2:1]] = 1'b0;
            end
        end
    endfunction

    function [7:0] iwm_control_after_access;
        input [3:0] idx;
        begin
            if (idx < 4'h8) begin
                iwm_control_after_access = iwm_control_reg;
            end else if (idx[0]) begin
                iwm_control_after_access = iwm_control_reg | (8'h01 << idx[3:1]);
            end else begin
                iwm_control_after_access = iwm_control_reg & ~(8'h01 << idx[3:1]);
            end
        end
    endfunction

    // SWIM/ISM handshake "no-media" pattern.
    //
    // MAME swim1_device::ism_read(7) returns h |= 0x0c when
    //   (!m_floppy || m_floppy->wpt_r())
    // — bit 2 is read-data idle, bit 3 is write-protect.  The Q700 ROM
    // floppy-poll loop interprets 0x0c as "drive attached, no media
    // inserted, please insert a disk" and stays in the poll loop.  The
    // legacy 0x08 pattern signals "no drive at all" and the ROM bails
    // out to MacsBug because both the floppy bus AND the SCSI bus then
    // report no boot devices.
    //
    // `drive_present` selects between the two:
    //   1 = MAME-faithful drive-attached, no-media   -> 0x0c
    //   0 = legacy "no drive at all" stub baseline   -> 0x08
    localparam [7:0] SWIM_HANDSHAKE_DRIVE_PRESENT = 8'h0c;
    localparam [7:0] SWIM_HANDSHAKE_NO_DRIVE      = 8'h08;
    wire [7:0] swim_handshake_no_media =
        drive_present ? SWIM_HANDSHAKE_DRIVE_PRESENT
                      : SWIM_HANDSHAKE_NO_DRIVE;

    function [7:0] swim_read_value;
        input [3:0] idx;
        begin
            case (idx)
                4'h0: swim_read_value = 8'hFF;
                4'h1: swim_read_value = 8'hFF;
                4'h2: swim_read_value = error_reg;
                4'h3: swim_read_value = param_ram[param_idx];
                4'h4: swim_read_value = phase_reg;
                4'h5: swim_read_value = setup_reg;
                4'h6: swim_read_value = mode_reg;
                4'h7: swim_read_value = swim_handshake_no_media;
                4'h8: swim_read_value = 8'hFF;
                4'h9: swim_read_value = 8'hFF;
                4'hA: swim_read_value = error_reg;
                4'hB: swim_read_value = param_ram[param_idx];
                4'hC: swim_read_value = phase_reg;
                4'hD: swim_read_value = setup_reg;
                4'hE: swim_read_value = {1'b0, mode_reg[6:0]} | 8'h80;
                4'hF: swim_read_value = swim_handshake_no_media;
                default: swim_read_value = 8'hFF;
            endcase
        end
    endfunction

    always @(*) begin
        if (cs && rd) begin
            if (mode_reg[6])
                rdata = swim_read_value(reg_sel);
            else
                rdata = iwm_read_value(iwm_control_after_access(reg_sel),
                                       iwm_phases_after_access(reg_sel));
        end else begin
            rdata = 8'hFF;
        end
    end

    always @(posedge clk) begin : iwm_seq
        reg [7:0] control_next;
        if (rst) begin
            data_reg  <= 8'h00;
            mark_reg  <= 8'h00;
            error_reg <= 8'h00;
            phase_reg <= 8'h00;
            setup_reg <= 8'h00;
            mode_reg  <= 8'h00;
            iwm_data_reg <= 8'h00;
            iwm_mode_reg <= 8'h00;
            iwm_status_reg <= 8'h00;
            iwm_control_reg <= 8'h00;
            iwm_whd_reg <= 8'hBF;
            iwm_active_reg <= 1'b0;
            iwm_to_swim_ctr <= 2'd0;
            param_idx <= 4'h0;
            for (i = 0; i < 16; i = i + 1)
                param_ram[i] <= 8'h00;
        end else if (cs && wr) begin
            if (!mode_reg[6]) begin
                control_next = iwm_control_after_access(reg_sel);
                iwm_to_swim_ctr <= 2'd0;
                if (reg_sel < 4'h8) begin
                    if (reg_sel[0])
                        phase_reg <= phase_reg | (8'h01 << reg_sel[3:1]);
                    else
                        phase_reg <= phase_reg & ~(8'h01 << reg_sel[3:1]);
                end else if (reg_sel[0]) begin
                    iwm_control_reg <= iwm_control_reg | (8'h01 << reg_sel[3:1]);
                end else begin
                    iwm_control_reg <= iwm_control_reg & ~(8'h01 << reg_sel[3:1]);
                end

                if (control_next[4]) begin
                    iwm_active_reg <= 1'b1;
                    iwm_status_reg <= iwm_status_reg | 8'h20;
                    if (control_next[7]) begin
                        iwm_whd_reg <= iwm_whd_reg | 8'h40;
                    end else begin
                        iwm_data_reg <= 8'h00;
                    end
                end else if (iwm_active_reg && iwm_mode_reg[2]) begin
                    iwm_active_reg <= 1'b0;
                    iwm_status_reg <= iwm_status_reg & ~8'h20;
                    iwm_whd_reg <= iwm_whd_reg & ~8'h40;
                end

                if (reg_sel == 4'hF &&
                    (iwm_control_after_access(reg_sel) & 8'hC0) == 8'hC0) begin
                    iwm_mode_reg <= wdata;
                    iwm_status_reg <= (iwm_status_reg & 8'hE0) | (wdata & 8'h1F);
                end

                // MAME swim1_device::iwm_control() recognizes a four-write
                // IWM-to-ISM entry sequence on offset F: bit6 set, clear,
                // set, set.  The Quadra ROM uses 57,17,57,57 before probing
                // the SWIM register bank.
                if (reg_sel == 4'hF) begin
                    case (iwm_to_swim_ctr)
                        2'd0: iwm_to_swim_ctr <= wdata[6] ? 2'd1 : 2'd0;
                        2'd1: iwm_to_swim_ctr <= wdata[6] ? 2'd0 : 2'd2;
                        2'd2: iwm_to_swim_ctr <= wdata[6] ? 2'd3 : 2'd0;
                        2'd3: begin
                            iwm_to_swim_ctr <= 2'd0;
                            if (wdata[6]) begin
                                mode_reg <= mode_reg | 8'h40;
                                param_idx <= 4'h0;
                            end
                        end
                        default: iwm_to_swim_ctr <= 2'd0;
                    endcase
                end else begin
                    iwm_to_swim_ctr <= 2'd0;
                end

                if (reg_sel == 4'h7) begin
                    mode_reg <= mode_reg | wdata;
                    param_idx <= 4'h0;
                end
            end else begin
                case (reg_sel)
                    4'h0: data_reg <= wdata;
                    4'h1: mark_reg <= wdata;
                    4'h2: error_reg <= wdata;
                    4'h3: begin
                        param_ram[param_idx] <= wdata;
                        param_idx <= param_idx + 4'h1;
                    end
                    4'h4: phase_reg <= wdata;
                    4'h5: setup_reg <= wdata;
                    4'h6: begin
                        mode_reg <= mode_reg & ~wdata;
                        param_idx <= 4'h0;
                    end
                    4'h7: begin
                        mode_reg <= mode_reg | wdata;
                        param_idx <= 4'h0;
                    end
                    4'h8: data_reg <= wdata;
                    4'h9: mark_reg <= wdata;
                    4'hA: error_reg <= wdata;
                    4'hB: begin
                        param_ram[param_idx] <= wdata;
                        param_idx <= param_idx + 4'h1;
                    end
                    4'hC: phase_reg <= wdata;
                    4'hD: setup_reg <= wdata;
                    default: ;
                endcase
            end
        end else if (cs && rd) begin
            if (!mode_reg[6]) begin
                control_next = iwm_control_after_access(reg_sel);
                if (reg_sel < 4'h8) begin
                    if (reg_sel[0])
                        phase_reg <= phase_reg | (8'h01 << reg_sel[3:1]);
                    else
                        phase_reg <= phase_reg & ~(8'h01 << reg_sel[3:1]);
                end else if (reg_sel[0]) begin
                    iwm_control_reg <= iwm_control_reg | (8'h01 << reg_sel[3:1]);
                end else begin
                    iwm_control_reg <= iwm_control_reg & ~(8'h01 << reg_sel[3:1]);
                end

                if (control_next[4]) begin
                    iwm_active_reg <= 1'b1;
                    iwm_status_reg <= iwm_status_reg | 8'h20;
                    if (control_next[7]) begin
                        iwm_whd_reg <= iwm_whd_reg | 8'h40;
                    end else begin
                        iwm_data_reg <= 8'h00;
                    end
                end else if (iwm_active_reg && iwm_mode_reg[2]) begin
                    iwm_active_reg <= 1'b0;
                    iwm_status_reg <= iwm_status_reg & ~8'h20;
                    iwm_whd_reg <= iwm_whd_reg & ~8'h40;
                end
            end
            if ((reg_sel == 4'h2) || (reg_sel == 4'hA))
                error_reg <= 8'h00;
            if (mode_reg[6] && ((reg_sel == 4'h3) || (reg_sel == 4'hB)))
                param_idx <= param_idx + 4'h1;
        end
    end

endmodule

`default_nettype wire
