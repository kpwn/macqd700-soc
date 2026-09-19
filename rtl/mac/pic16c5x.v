// MAME reference/excerpt/adaptation attribution: Copyright Tony La Porta.
// MAME-derived portions retain BSD-3-Clause; original contributions are MIT.
// See THIRD_PARTY_NOTICES.md and LICENSES/MAME-BSD-3-Clause.txt.
// pic16c5x.v - small PIC16C5x-family microcontroller core
//
// Implements the 12-bit PIC16C54/PIC1654S subset used by the Macintosh ADB
// PIC firmware: 512-word program ROM, 32-byte register file, W accumulator,
// two-deep call stack, GPIO/TRIS, TMR0, OPTION, SLEEP/CLRWDT.  One retired
// instruction is accepted per cyc_en when ready; two-cycle instructions insert
// one internal bubble so TMR0 observes the extra instruction cycle.

module pic16c5x #(
    parameter PROGHEX = "adb_pic_fw.hex"
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        cyc_en,
    input  wire        rtcc_in,
    input  wire [7:0]  porta_in,
    input  wire [7:0]  portb_in,
    output wire [7:0]  porta_out,
    output wire [7:0]  portb_out,
    output wire [7:0]  porta_dir,
    output wire [7:0]  portb_dir,
    output wire [8:0]  dbg_pc,
    output wire [7:0]  dbg_w,
    output wire        cyc_done
);
    /* verilator lint_off BLKSEQ */
    localparam [4:0] REG_INDF = 5'h00, REG_TMR0 = 5'h01, REG_PCL = 5'h02;
    localparam [4:0] REG_STATUS = 5'h03, REG_FSR = 5'h04;
    localparam [4:0] REG_PORTA = 5'h05, REG_PORTB = 5'h06;
    localparam [7:0] STATUS_TO_PD = 8'h18; localparam [8:0] RESET_PC = 9'h1ff;
    // Fetch on the falling fabric-clock edge; retire on the rising edge.
    // This preserves every cyc_en/branch-bubble/GPIO edge, even for back-to-
    // back cyc_en. Both PC->BRAM and BRAM->execute are HALF-CYCLE paths:
    // do not false-path them. The SoC's ADB instance runs on 50 MHz pb_clk.
    wire [11:0] fetched_instr;
    pic16c5x_program_rom #(.PROGHEX(PROGHEX)) u_program_rom (
        .clk(clk), .addr(pc), .data(fetched_instr)
    );
    reg [7:0]  ram[0:31];
    reg [8:0]  pc, stack0, stack1;
    reg [7:0]  w_reg, status_reg, fsr_reg, porta_latch, portb_latch;
    reg [7:0]  trisa_reg, trisb_reg, option_reg, prescaler;
    reg [1:0]  tmr0_inhibit;
    reg [15:0] wdt_counter;
    reg        bubble, sleep_stall, cyc_done_reg, rtcc_prev;
    reg [11:0] instr;
    reg [8:0]  next_pc, add_val, sub_val;
    reg [4:0]  file_addr;
    reg [7:0]  file_val, result_val;
    reg        tmr0_written;
    reg        pic_trace_en;
    reg        pic_trace_evt;
    reg [7:0]  pb_lat_prev, tb_prev, pa_lat_prev, ta_prev, pa_in_prev;
`ifndef SYNTHESIS
    initial pic_trace_en  = $test$plusargs("pic_trace");
    initial pic_trace_evt = $test$plusargs("pic_trace_evt");
`else
    initial pic_trace_en  = 1'b0;
    initial pic_trace_evt = 1'b0;
`endif
    assign porta_out = porta_latch; assign portb_out = portb_latch;
    assign porta_dir = trisa_reg; assign portb_dir = trisb_reg;
    assign dbg_pc = pc; assign dbg_w = w_reg; assign cyc_done = cyc_done_reg;

    function [7:0] direct_read;
        input [4:0] addr;
        begin
            case (addr)
                REG_INDF:   direct_read = 8'h00;
                REG_TMR0:   direct_read = ram[REG_TMR0];
                // Reading PCL yields the *incremented* PC (address of the
                // next instruction) — the real PIC has already advanced PC
                // by the time the instruction executes.  ADDWF PCL jump
                // tables depend on this; returning pc[7:0] is off by one.
                REG_PCL:    direct_read = next_pc[7:0];
                REG_STATUS: direct_read = status_reg;
                // PIC1654S has a 5-bit register file: FSR bits 7:5 are
                // unimplemented and read as 1.  The ADB firmware's RAM-clear
                // loop (CLRF INDF / INCFSZ FSR / GOTO) relies on this — FSR
                // reads 0xFF at file index 0x1F so INCFSZ wraps and exits.
                // Reading raw 8-bit FSR makes the loop never terminate (and
                // self-clobber FSR when it indirectly hits register 4).
                REG_FSR:    direct_read = fsr_reg | 8'he0;
                // PIC1650/1654/1654S port I/O is open-drain: TRIS is not
                // used.  Writing the port latch directly drives the pin
                // low (latch=0) or tri-states it (latch=1, pulled high
                // externally).  Reads return external_input AND latch —
                // a bit reads as 1 only if the latch is letting the pin
                // float AND the external line is high.  Matches MAME
                // pic16c5x.cpp porta_r / portb_r for picmodel 0x1654:
                //     return read_port(...) & m_port_data[PORT].
                // Reset trisa/trisb stay at 0xff and are unused on this
                // model (the ADB firmware never executes TRIS).
                REG_PORTA:  direct_read = (porta_in & porta_latch) & 8'h0f;
                REG_PORTB:  direct_read = (portb_in & portb_latch);
                default:    direct_read = ram[addr];
            endcase
        end
    endfunction

    function [7:0] file_read;
        input [4:0] addr;
        reg [4:0] real_addr;
        begin
            real_addr = (addr == REG_INDF) ? fsr_reg[4:0] : addr;
            file_read = ((addr == REG_INDF) && (real_addr == REG_INDF)) ? 8'h00 : direct_read(real_addr);
        end
    endfunction

    task write_direct;
        input [4:0] addr;
        input [7:0] data;
        begin
            case (addr)
                REG_INDF: ;
                REG_TMR0: begin
                    ram[REG_TMR0] <= data; prescaler <= 8'h00; tmr0_inhibit <= 2'd2;
                    tmr0_written = 1'b1;
                end
                REG_PCL: begin
                    // 9-bit-PC part: a PCL write forces PC[8]=0 (the
                    // STATUS PA bits land at PC[9+], which don't exist).
                    pc <= {1'b0, data}; bubble <= 1'b1;
                end
                REG_STATUS: status_reg <= data;
                REG_FSR:    fsr_reg <= data;
                REG_PORTA:  porta_latch <= data & 8'h0f;
                REG_PORTB:  portb_latch <= data;
                default:    ram[addr] <= data;
            endcase
        end
    endtask

    task file_write;
        input [4:0] addr;
        input [7:0] data;
        reg [4:0] real_addr;
        begin
            real_addr = (addr == REG_INDF) ? fsr_reg[4:0] : addr;
            if (!((addr == REG_INDF) && (real_addr == REG_INDF))) write_direct(real_addr, data);
        end
    endtask

    task set_z;
        input [7:0] data;
        begin
            status_reg[2] <= (data == 8'h00);
        end
    endtask

    task tick_tmr0;
        begin
            if (tmr0_written) begin
            end else if (tmr0_inhibit != 2'd0) begin
                tmr0_inhibit <= tmr0_inhibit - 2'd1;
            end else if (!option_reg[5]) begin
                if (option_reg[3]) begin
                    ram[REG_TMR0] <= ram[REG_TMR0] + 8'd1;
                end else if (prescaler == ((8'd2 << option_reg[2:0]) - 8'd1)) begin
                    prescaler <= 8'h00; ram[REG_TMR0] <= ram[REG_TMR0] + 8'd1;
                end else begin
                    prescaler <= prescaler + 8'd1;
                end
            end else if ((!option_reg[4] && !rtcc_prev && rtcc_in) ||
                         ( option_reg[4] &&  rtcc_prev && !rtcc_in)) begin
                ram[REG_TMR0] <= ram[REG_TMR0] + 8'd1;
            end
            rtcc_prev <= rtcc_in;
        end
    endtask

    integer i;
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1) begin
                ram[i] <= 8'h00;
            end
            pc <= RESET_PC; w_reg <= 8'h00; status_reg <= STATUS_TO_PD; fsr_reg <= 8'h00;
            porta_latch <= 8'h00; portb_latch <= 8'h00; trisa_reg <= 8'hff; trisb_reg <= 8'hff;
            option_reg <= 8'hff; prescaler <= 8'h00; tmr0_inhibit <= 2'd0; wdt_counter <= 16'h0000;
            rtcc_prev <= rtcc_in;
            stack0 <= 9'h000; stack1 <= 9'h000; bubble <= 1'b0; sleep_stall <= 1'b0; cyc_done_reg <= 1'b0;
        end else begin
            cyc_done_reg <= 1'b0;
            tmr0_written = 1'b0;
            if (cyc_en && !sleep_stall) begin
                if (bubble) begin
                    bubble <= 1'b0;
                    tick_tmr0();
                end else begin
                    instr = fetched_instr; next_pc = pc + 9'd1;
                    file_addr = instr[4:0]; file_val = file_read(file_addr);
                    pc <= next_pc; cyc_done_reg <= 1'b1;
`ifndef SYNTHESIS
                    if (pic_trace_en ||
                        (pic_trace_evt &&
                         ({portb_latch, trisb_reg, porta_latch, trisa_reg, porta_in} !=
                          {pb_lat_prev, tb_prev, pa_lat_prev, ta_prev, pa_in_prev})))
                        $display("[pic] pc=%03x ir=%03x w=%02x st=%02x pa_in=%02x pb_in=%02x pb_lat=%02x trisa=%02x trisb=%02x fsr=%02x r8=%02x r0f=%02x r17=%02x",
                                 pc, instr, w_reg, status_reg, porta_in, portb_in,
                                 portb_latch, trisa_reg, trisb_reg,
                                 fsr_reg, ram[8], ram[15], ram[23]);
                    pb_lat_prev <= portb_latch; tb_prev <= trisb_reg;
                    pa_lat_prev <= porta_latch; ta_prev <= trisa_reg;
                    pa_in_prev  <= porta_in;
                    if ((pic_trace_en || pic_trace_evt) && pc == 9'h022)
                        $display("[pic-jt] PC=022 ADDWF PCL,F  w=%02x -> target_idx=%0d  ram15=%02x ram8=%02x",
                                 w_reg, w_reg & 8'h0f, ram[15], ram[8]);
`endif

                    case (instr[11:8])
                        4'h0, 4'h1, 4'h2, 4'h3: begin
                            case (instr[11:6])
                                6'b000010: begin
                                    sub_val = {1'b0, file_val} - {1'b0, w_reg};
                                    result_val = sub_val[7:0];
                                    if (instr[5]) file_write(file_addr, result_val);
                                    else w_reg <= result_val;
                                    status_reg[0] <= (file_val >= w_reg);
                                    status_reg[1] <= (file_val[3:0] >= w_reg[3:0]);
                                    set_z(result_val);
                                end
                                6'b000011: begin
                                    result_val = file_val - 8'd1;
                                    if (instr[5]) file_write(file_addr, result_val);
                                    else w_reg <= result_val;
                                    set_z(result_val);
                                end
                                6'b000100, 6'b000101, 6'b000110: begin
                                    if (instr[11:6] == 6'b000100) result_val = file_val | w_reg;
                                    else if (instr[11:6] == 6'b000101) result_val = file_val & w_reg;
                                    else result_val = file_val ^ w_reg;
                                    if (instr[5]) file_write(file_addr, result_val);
                                    else w_reg <= result_val;
                                    set_z(result_val);
                                end
                                6'b000111: begin
                                    add_val = {1'b0, file_val} + {1'b0, w_reg};
                                    result_val = add_val[7:0];
                                    if (instr[5]) file_write(file_addr, result_val);
                                    else w_reg <= result_val;
                                    status_reg[0] <= add_val[8];
                                    status_reg[1] <= (({1'b0, file_val[3:0]} + {1'b0, w_reg[3:0]}) > 5'h0f);
                                    set_z(result_val);
                                end
                                6'b001000: begin
                                    result_val = file_val;
                                    if (instr[5]) file_write(file_addr, result_val);
                                    else w_reg <= result_val;
                                    set_z(result_val);
                                end
                                6'b001001, 6'b001010: begin
                                    result_val = (instr[11:6] == 6'b001001) ? ~file_val : file_val + 8'd1;
                                    if (instr[5]) file_write(file_addr, result_val);
                                    else w_reg <= result_val;
                                    set_z(result_val);
                                end
                                6'b001011, 6'b001111: begin
                                    result_val = (instr[11:6] == 6'b001011) ? file_val - 8'd1 : file_val + 8'd1;
                                    if (instr[5]) file_write(file_addr, result_val);
                                    else w_reg <= result_val;
                                    if (result_val == 8'h00) begin pc <= pc + 9'd2; bubble <= 1'b1; end
                                end
                                6'b001100: begin
                                    result_val = {status_reg[0], file_val[7:1]};
                                    if (instr[5]) file_write(file_addr, result_val);
                                    else w_reg <= result_val;
                                    status_reg[0] <= file_val[0];
                                end
                                6'b001101: begin
                                    result_val = {file_val[6:0], status_reg[0]};
                                    if (instr[5]) file_write(file_addr, result_val);
                                    else w_reg <= result_val;
                                    status_reg[0] <= file_val[7];
                                end
                                6'b001110: begin
                                    result_val = {file_val[3:0], file_val[7:4]};
                                    if (instr[5]) file_write(file_addr, result_val);
                                    else w_reg <= result_val;
                                end
                                default: begin
                                    if (instr[11:5] == 7'b0000001) file_write(file_addr, w_reg);
                                    else if (instr[11:5] == 7'b0000011) begin
                                        file_write(file_addr, 8'h00); status_reg[2] <= 1'b1;
                                    end else if (instr == 12'h002) option_reg <= w_reg;
                                    else if (instr == 12'h003) sleep_stall <= 1'b1;
                                    else if (instr == 12'h004) begin
                                        wdt_counter <= 16'h0000; status_reg[4:3] <= 2'b11;
                                    end else if (instr == 12'h005) trisa_reg <= w_reg;
                                    else if (instr == 12'h006) trisb_reg <= w_reg;
                                    else if (instr == 12'h040) begin
                                        w_reg <= 8'h00; status_reg[2] <= 1'b1;
                                    end
                                end
                            endcase
                        end
                        4'h4, 4'h5: begin
                            if (instr[11:8] == 4'h4) result_val = file_val & ~(8'h01 << instr[7:5]);
                            else result_val = file_val | (8'h01 << instr[7:5]);
                            file_write(file_addr, result_val);
                        end
                        4'h6, 4'h7: begin
                            if ((instr[11:8] == 4'h6 && !file_val[instr[7:5]]) ||
                                (instr[11:8] == 4'h7 && file_val[instr[7:5]])) begin
                                pc <= pc + 9'd2; bubble <= 1'b1;
                            end
                        end
                        4'h8: begin
                            w_reg <= instr[7:0]; pc <= stack0; stack0 <= stack1; bubble <= 1'b1;
                        end
                        4'h9: begin
                            // CALL on a 9-bit-PC part: target = {0, k[7:0]} —
                            // bit 8 is forced 0, PA bits land beyond PC width.
                            stack1 <= stack0; stack0 <= next_pc; pc <= {1'b0, instr[7:0]}; bubble <= 1'b1;
                        end
                        4'ha, 4'hb: begin
                            pc <= instr[8:0]; bubble <= 1'b1;
                        end
                        4'hc: w_reg <= instr[7:0];
                        4'hd, 4'he, 4'hf: begin
                            if (instr[11:8] == 4'hd) result_val = w_reg | instr[7:0];
                            else if (instr[11:8] == 4'he) result_val = w_reg & instr[7:0];
                            else result_val = w_reg ^ instr[7:0];
                            w_reg <= result_val;
                            set_z(result_val);
                        end
                    endcase
                    tick_tmr0();
                end
                wdt_counter <= wdt_counter + 16'd1;
            end
        end
    end
    /* verilator lint_on BLKSEQ */
endmodule

// Fixed physical layout for post-build local firmware insertion. One RAMB36
// stores 1024 32-bit containers; words 0..511 hold the 12-bit PIC program.
// Higher data bits, unused words and parity remain zero. The deliberately
// over-wide layout makes the MMI mapping direct and avoids parity packing.
// Synthesis NEVER reads PROGHEX: the distributed base image is blank. Use
// tools/patch_adb_bitstream.py with your own dump before programming a board.
module pic16c5x_program_rom #(
    parameter PROGHEX = "adb_pic_fw.hex"
)(
    input wire clk,
    input wire [8:0] addr,
    output wire [11:0] data
);
`ifdef SYNTHESIS
    wire [31:0] rom_data;
    // Explicit primitive + DONT_TOUCH prevents all-zero initialization from
    // collapsing the memory or its consumers into constants during synthesis.
    (* DONT_TOUCH = "yes" *) RAMB36E2 #(
        .READ_WIDTH_A(36), .READ_WIDTH_B(0),
        .WRITE_WIDTH_A(36), .WRITE_WIDTH_B(0),
        .DOA_REG(0), .DOB_REG(0),
        .IS_CLKARDCLK_INVERTED(1'b1),
        .EN_ECC_READ("FALSE"), .EN_ECC_WRITE("FALSE")
    ) u_firmware_bram (
        .CLKARDCLK(clk), .CLKBWRCLK(clk),
        .ADDRARDADDR({1'b0, addr, 5'b00000}), .ADDRBWRADDR(15'b0),
        .ENARDEN(1'b1), .ENBWREN(1'b0),
        .ADDRENA(1'b1), .ADDRENB(1'b0),
        .WEA(4'b0), .WEBWE(8'b0),
        .DINADIN(32'b0), .DINBDIN(32'b0),
        .DINPADINP(4'b0), .DINPBDINP(4'b0),
        .DOUTADOUT(rom_data), .DOUTBDOUT(),
        .DOUTPADOUTP(), .DOUTPBDOUTP(),
        .REGCEAREGCE(1'b0), .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0), .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0), .RSTREGB(1'b0), .SLEEP(1'b0),
        .CASDINA(32'b0), .CASDINB(32'b0),
        .CASDINPA(4'b0), .CASDINPB(4'b0),
        .CASDIMUXA(1'b0), .CASDIMUXB(1'b0),
        .CASDOMUXA(1'b0), .CASDOMUXB(1'b0),
        .CASDOMUXEN_A(1'b0), .CASDOMUXEN_B(1'b0),
        .CASOREGIMUXA(1'b0), .CASOREGIMUXB(1'b0),
        .CASOREGIMUXEN_A(1'b0), .CASOREGIMUXEN_B(1'b0),
        .CASINDBITERR(1'b0), .CASINSBITERR(1'b0),
        .INJECTDBITERR(1'b0), .INJECTSBITERR(1'b0), .ECCPIPECE(1'b0),
        .CASDOUTA(), .CASDOUTB(), .CASDOUTPA(), .CASDOUTPB(),
        .CASOUTDBITERR(), .CASOUTSBITERR(),
        .DBITERR(), .SBITERR(), .ECCPARITY(), .RDADDRECC()
    );
    assign data = rom_data[11:0];
`else
    reg [11:0] prog [0:511];
    reg [11:0] read_data;
    initial $readmemh(PROGHEX, prog);
    always @(negedge clk) read_data <= prog[addr];
    assign data = read_data;
`endif
endmodule
