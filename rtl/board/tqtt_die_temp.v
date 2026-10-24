// Read the KU5P on-chip temperature conversion through SYSMONE1's DRP.
// The sensor runs its continuous sequencer; each temperature EOC triggers
// one read of status register 00h. The fan controller checks freshness.
module tqtt_die_temp (
    input  wire       clk,
    input  wire       locked,
    output reg  [9:0] adc_code = 0,
    output reg        valid = 0
);
    wire [5:0] channel;
    wire eoc, drdy, jtag_locked;
    wire [15:0] drp_data;
    reg den = 0;
    reg pending = 0;

    SYSMONE1 #(
        .INIT_40(16'h9000), // Continuous conversions, normal sensor setup.
        .INIT_41(16'h2ef0), // Continuous sequencer and calibration.
        .INIT_42(16'h0400), // Divide 50 MHz DCLK for the ADC.
        .INIT_48(16'h4701), // Include temperature and supply channels.
        .INIT_4A(16'h4700)  // Average the internal channels.
    ) u_sysmon (
        .DCLK(clk), .RESET(!locked),
        .DADDR(8'h00), .DEN(den), .DI(16'h0000), .DWE(1'b0),
        .DO(drp_data), .DRDY(drdy),
        .CHANNEL(channel), .EOC(eoc), .EOS(), .BUSY(),
        .JTAGBUSY(), .JTAGLOCKED(jtag_locked), .JTAGMODIFIED(),
        .ALM(), .OT(), .MUXADDR(),
        .CONVST(1'b0), .CONVSTCLK(1'b0),
        .VAUXP(16'h0000), .VAUXN(16'h0000), .VP(1'b0), .VN(1'b0),
        .I2C_SCLK(1'b0), .I2C_SDA(1'b0), .I2C_SCLK_TS(), .I2C_SDA_TS()
    );

    always @(posedge clk or negedge locked) begin
        if (!locked) begin
            den <= 0;
            pending <= 0;
            adc_code <= 0;
            valid <= 0;
        end else begin
            den <= 0;
            valid <= 0;
            if (drdy && pending) begin
                adc_code <= drp_data[15:6];
                valid <= 1;
                pending <= 0;
            end else if (!pending && !jtag_locked && eoc && channel == 6'h00) begin
                den <= 1;
                pending <= 1;
            end
        end
    end
endmodule
