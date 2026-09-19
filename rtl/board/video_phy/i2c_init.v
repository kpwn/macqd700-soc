// i2c_init.v -- ROM-driven I2C init sequencer for SiI9134 HDMI transmitter
// ---------------------------------------------------------------------------
// Ported verbatim from ~/sd-hdmi-bringup/rtl/i2c_init.v for the m68k-ooo
// mac-on-FPGA project.  Module body unchanged.
//
// After a hardware reset pulse, plays back a fixed table of {reg, data} writes
// at 7-bit slave address 0x39 (write byte 0x72) over a true open-drain I2C bus.
// ACKs and NAKs are counted for diagnostics -- the sequencer does not retry on
// NAK, so a marginal ACK doesn't stall bring-up.
//
// Timing (clk = 148.5 MHz, I2C_HALF = 744 -> SCL ~ 99.8 kHz):
//   Half period = 5.0 us   (spec: >= 4.7 us START hold-time)
//   Reset low   = 10 ms    then 10 ms post-reset settle
//
// Bus discipline is true open-drain: scl_oe / sda_oe drive low (1) or release
// (0, pull-ups restore the line to 1). sda_i is the pin readback used for ACK
// sampling.
// ---------------------------------------------------------------------------
`default_nettype none

module i2c_init #(
    parameter I2C_HALF     = 10'd372,
    parameter RESET_CYCLES = 24'd742500,
    parameter INIT_WAIT    = 24'd742500
) (
    input  wire       clk,
    input  wire       resetn,
    output reg        scl_oe,
    output reg        sda_oe,
    input  wire       sda_i,
    output reg        chip_rstn,
    output reg        done,
    output reg  [7:0] ack_count,
    output reg  [7:0] nak_count,
    output wire [3:0] dbg_state,
    output wire [3:0] dbg_rom_idx
);

    // SiI9134: 7-bit slave 0x39 -> write byte 0x72.  (Not "TPI" -- this part
    // has no TPI; see the register-table audit below.)
    localparam [7:0] DEV_ADDR = 8'h72;

    // -----------------------------------------------------------------------
    // SiI9134 register init table.
    //
    // ⚠️⚠️ THE DESCRIPTIONS BELOW ARE UNVERIFIED AND AT LEAST TWO ARE PROVABLY
    // WRONG (audited 2026-09-15).  DO NOT reason about the encoder's
    // configuration from them.  The VALUES are unchanged and board-proven --
    // this table brings the part up and it syncs -- but the labels were pasted
    // from a DIFFERENT PART'S register map.
    //
    // What the audit established, with sources:
    //
    //   * THE SiI9134 HAS NO TPI.  Lattice SiI9136-3 data sheet SiI-DS-1084-C,
    //     Table 2.1 "Product Selection Guide", row "Programming Interface":
    //     SiI9134 = No, SiI9136 = Yes.  Every "TPI" label below is wrong on its
    //     face; this part uses the older SiI9034/SiI164-lineage register map.
    //
    //   * 0x0A IS NOT THE CLOCK-EDGE REGISTER, in EITHER map.  In the legacy
    //     lineage 0x0A is the de-skew / CTL register and bit 0 is reserved; in
    //     TPI (which this part does not have) 0x0A is AVI OUTPUT format, where
    //     0x01 would mean 8-bit YCbCr 4:4:4 -- wrong for RGB and still not an
    //     edge control.  So "Input bus format: 24-bit RGB, rising-edge clock"
    //     is wrong about the register, the bus width AND the edge.
    //
    //   * THE EDGE AND BUS WIDTH LIVE IN 0x08.  SiI9022A/9024A TPI Programmer's
    //     Reference SiI-PR-1032, Table 2 maps the legacy scheme's System Control
    //     group to 0x08-0x0D and names edge + bsel as its contents.  The SiI164
    //     data sheet SiI-DS-0021-D gives that group's bit layout field-for-field:
    //         0x08:  [7:6] RSVD  [5] VEN  [4] HEN  [3] DSEL  [2] BSEL  [1] EDGE  [0] PD
    //         EDGE = 0 -> falling-edge latched ;  EDGE = 1 -> rising-edge latched
    //         BSEL = 1 -> 24-bit bus          ;  PD   = 1 -> normal operation
    //     Under that layout 0x35 = 0b0011_0101 decodes as
    //         PD=1, EDGE=0, BSEL=1, DSEL=0, HEN=1, VEN=1
    //     i.e. powered up, 24-bit, single-edge, syncs through, FALLING-edge latch.
    //     0x37 would be the same with EDGE=1 (rising).
    //
    //   * THE SiI9134 DATA SHEET (SiI-DS-0193-F) CONFIRMS THE EDGE CONCEPT AND
    //     THE TIMING, but contains NO register map at all:
    //         TSIDF setup 1.0 ns / THIDF hold 0.8 ns   to IDCK falling (EDGE=0)
    //         TSIDR setup 1.0 ns / THIDR hold 0.5 ns   to IDCK rising  (EDGE=1)
    //     synth/hdmi.xdc now uses those numbers instead of an assumed budget.
    //
    // WHAT IS STILL UNRESOLVED.  The SiI9134 Programmer's Reference -- the one
    // document that would pin 0x08's bit layout to THIS part rather than to its
    // SiI164 ancestor -- could not be found, publicly or on this machine.  And
    // the board behaves as though capture were RISING while 0x35 reads as
    // FALLING under the layout above.  Those two cannot both be right.
    //
    // NOTHING HERE IS CHANGED ON THAT BASIS.  A register bit written on an
    // inference across part families is exactly the guess that must not be made.
    // The interface was instead made INDEPENDENT of the answer: video_top
    // forwards the pixel clock a quarter period out of phase, so both candidate
    // edges clear every data transition: measured on the first build to carry it,
    // the rising edge lands 2.27-4.07 ns into the data eye and the falling edge
    // 9.01-10.81 ns in.  STA confirms all four checks positive (worst +1.600 ns),
    // so the interface is verified on BOTH candidate capture edges.
    //
    // THE EXPERIMENT THAT SETTLES IT, if it ever needs settling: rebuild with
    // HDMI_PCLK_FWD_PHASE (rtl/soc/fpga_top_video.vh) at 0 and at 180 and look at
    // a 1bpp desktop.  0 puts the RISING edge on the data transition, 180 puts the
    // FALLING edge there.  Whichever one displaces is the edge the encoder uses.
    // 720p60 at scale N=1 is required -- at N=2 a one-pixel slip is half-masked,
    // which is how the 2026-09-13 observation came to be ambiguous.
    // A second, independent check: read register 0x08 back over I2C and test bit 1
    // (this sequencer is write-only, so that needs a read path adding).
    //
    // Original (UNVERIFIED, retained only to show what was believed):
    //   0x08  TMDS output swing + receiver sense enable
    //   0x09  Audio interface control
    //   0x0A  Input bus format: 24-bit RGB, rising-edge clock   <-- WRONG, see above
    //   0x0B  AVI DB1: RGB, no bar info
    //   0x0C  AVI DB2: 16:9 aspect
    //   0x0D  AVI DB3
    //   0x0E  AVI DB4: VIC=0 (monitor auto-detect)
    //   0x0F  AVI DB5
    //   0x1A  HDMI/DVI mode: 0x00=DVI, 0x02=HDMI
    //   0x1E  TPI System Control: power on, TMDS active          <-- no TPI on this part
    //   0x3C  AVI InfoFrame enable
    // -----------------------------------------------------------------------
    localparam ROM_DEPTH = 11;
    reg [15:0] rom [0:ROM_DEPTH-1];
    initial begin
        rom[0]  = {8'h08, 8'h35};
        rom[1]  = {8'h09, 8'h00};
        rom[2]  = {8'h0A, 8'h01};
        rom[3]  = {8'h0B, 8'h00};
        rom[4]  = {8'h0C, 8'h28};
        rom[5]  = {8'h0D, 8'h00};
        rom[6]  = {8'h0E, 8'h00};
        rom[7]  = {8'h0F, 8'h00};
        rom[8]  = {8'h1A, 8'h00};
        rom[9]  = {8'h1E, 8'h00};
        rom[10] = {8'h3C, 8'h01};
    end

    localparam [3:0]
        S_CHIP_RESET   = 4'd0,
        S_CHIP_WAIT    = 4'd1,
        S_IDLE         = 4'd2,
        S_START1       = 4'd3,   // SDA low, SCL released-high
        S_START2       = 4'd4,   // SCL low, load first bit
        S_BIT_LO       = 4'd5,   // SCL low, SDA stable
        S_BIT_HI       = 4'd6,   // SCL released-high, slave latches
        S_BIT_LO2      = 4'd7,   // SCL low again
        S_ACK_LO       = 4'd8,   // SCL low, SDA released for slave
        S_ACK_HI       = 4'd9,   // SCL high, sample SDA
        S_ACK_LO2      = 4'd10,  // SCL low, decide next step
        S_STOP_SCL     = 4'd11,  // SCL released-high, SDA still low
        S_STOP_SDA     = 4'd12,  // SDA released-high -> STOP
        S_INTER        = 4'd13,
        S_DONE         = 4'd14;

    reg [3:0]  state;
    reg [23:0] timer;
    reg [3:0]  bit_idx;
    reg [7:0]  shift_reg;
    reg [1:0]  byte_phase;   // 0 = addr, 1 = reg, 2 = data
    reg [3:0]  rom_idx;

    wire timer_done = (timer == 24'd0);

    assign dbg_state   = state;
    assign dbg_rom_idx = rom_idx;

    always @(posedge clk) begin
        if (~resetn) begin
            state      <= S_CHIP_RESET;
            timer      <= RESET_CYCLES;
            scl_oe     <= 1'b0;
            sda_oe     <= 1'b0;
            chip_rstn  <= 1'b0;
            done       <= 1'b0;
            ack_count  <= 8'd0;
            nak_count  <= 8'd0;
            rom_idx    <= 4'd0;
            bit_idx    <= 4'd7;
            byte_phase <= 2'd0;
            shift_reg  <= 8'd0;
        end else begin
            if (!timer_done)
                timer <= timer - 24'd1;

            case (state)

                S_CHIP_RESET: begin
                    chip_rstn <= 1'b0;
                    scl_oe    <= 1'b0;
                    sda_oe    <= 1'b0;
                    if (timer_done) begin
                        chip_rstn <= 1'b1;
                        timer     <= INIT_WAIT;
                        state     <= S_CHIP_WAIT;
                    end
                end

                S_CHIP_WAIT: begin
                    if (timer_done)
                        state <= S_IDLE;
                end

                S_IDLE: begin
                    scl_oe <= 1'b0;
                    sda_oe <= 1'b0;
                    if (rom_idx == ROM_DEPTH) begin
                        state <= S_DONE;
                    end else begin
                        shift_reg  <= DEV_ADDR;
                        byte_phase <= 2'd0;
                        bit_idx    <= 4'd7;
                        timer      <= {14'd0, I2C_HALF};
                        state      <= S_START1;
                    end
                end

                S_START1: begin
                    scl_oe <= 1'b0;
                    sda_oe <= 1'b1;   // SDA falls while SCL high -> START
                    if (timer_done) begin
                        timer <= {14'd0, I2C_HALF};
                        state <= S_START2;
                    end
                end

                S_START2: begin
                    scl_oe <= 1'b1;   // SCL low
                    if (timer_done) begin
                        sda_oe <= ~shift_reg[7];
                        timer  <= {14'd0, I2C_HALF};
                        state  <= S_BIT_LO;
                    end
                end

                S_BIT_LO: begin
                    if (timer_done) begin
                        scl_oe <= 1'b0;
                        timer  <= {14'd0, I2C_HALF};
                        state  <= S_BIT_HI;
                    end
                end

                S_BIT_HI: begin
                    if (timer_done) begin
                        scl_oe <= 1'b1;
                        timer  <= {14'd0, I2C_HALF};
                        state  <= S_BIT_LO2;
                    end
                end

                S_BIT_LO2: begin
                    if (timer_done) begin
                        if (bit_idx == 4'd0) begin
                            sda_oe <= 1'b0;   // release SDA for slave ACK
                            timer  <= {14'd0, I2C_HALF};
                            state  <= S_ACK_LO;
                        end else begin
                            bit_idx <= bit_idx - 4'd1;
                            sda_oe  <= ~shift_reg[bit_idx - 4'd1];
                            timer   <= {14'd0, I2C_HALF};
                            state   <= S_BIT_LO;
                        end
                    end
                end

                S_ACK_LO: begin
                    if (timer_done) begin
                        scl_oe <= 1'b0;
                        timer  <= {14'd0, I2C_HALF};
                        state  <= S_ACK_HI;
                    end
                end

                S_ACK_HI: begin
                    if (timer_done) begin
                        if (sda_i == 1'b0)
                            ack_count <= ack_count + 8'd1;
                        else
                            nak_count <= nak_count + 8'd1;
                        scl_oe <= 1'b1;
                        timer  <= {14'd0, I2C_HALF};
                        state  <= S_ACK_LO2;
                    end
                end

                S_ACK_LO2: begin
                    if (timer_done) begin
                        case (byte_phase)
                            2'd0: begin
                                shift_reg  <= rom[rom_idx][15:8];
                                byte_phase <= 2'd1;
                                bit_idx    <= 4'd7;
                                sda_oe     <= ~rom[rom_idx][15];
                                timer      <= {14'd0, I2C_HALF};
                                state      <= S_BIT_LO;
                            end
                            2'd1: begin
                                shift_reg  <= rom[rom_idx][7:0];
                                byte_phase <= 2'd2;
                                bit_idx    <= 4'd7;
                                sda_oe     <= ~rom[rom_idx][7];
                                timer      <= {14'd0, I2C_HALF};
                                state      <= S_BIT_LO;
                            end
                            2'd2: begin
                                sda_oe <= 1'b1;  // drive SDA low ahead of STOP
                                timer  <= {14'd0, I2C_HALF};
                                state  <= S_STOP_SCL;
                            end
                            default: state <= S_DONE;
                        endcase
                    end
                end

                S_STOP_SCL: begin
                    scl_oe <= 1'b0;
                    if (timer_done) begin
                        timer <= {14'd0, I2C_HALF};
                        state <= S_STOP_SDA;
                    end
                end

                S_STOP_SDA: begin
                    sda_oe <= 1'b0;
                    if (timer_done) begin
                        rom_idx <= rom_idx + 4'd1;
                        timer   <= {14'd0, I2C_HALF};
                        state   <= S_INTER;
                    end
                end

                S_INTER: begin
                    if (timer_done)
                        state <= S_IDLE;
                end

                S_DONE: begin
                    done   <= 1'b1;
                    scl_oe <= 1'b0;
                    sda_oe <= 1'b0;
                end

                default: state <= S_DONE;
            endcase
        end
    end

endmodule
`default_nettype wire
