// Fail-safe fan curve using SYSMONE1's upper ten temperature ADC bits.
// The KU5P maps SYSMONE1 to SYSMONE4. UG580's SYSMONE4 internal-reference
// transfer T = code*509.3140064/1024 - 280.2308787 maps codes 600 and
// 664 to about 18.2 C and 50.0 C. Their
// 64-code span needs only a shift for 8-bit PWM.
module tqtt_fan_thermal #(
    parameter integer CLK_HZ = 50_000_000
) (
    input  wire       clk,
    input  wire       locked,
    input  wire [9:0] adc_code,
    input  wire       adc_valid,
    input  wire [15:0] rpm,
    output wire [7:0] duty,
    output reg        fault = 0
);
    localparam integer SEC_W = $clog2(CLK_HZ);
    localparam integer SECOND_LAST = CLK_HZ - 1;
    reg [SEC_W-1:0] second_count = 0;
    reg [1:0] age_seconds = 2;
    reg [1:0] warmup_seconds = 0;
    reg [1:0] stalled_seconds = 0;
    reg [9:0] accepted_code = 0;
    wire second_tick = second_count == SECOND_LAST[SEC_W-1:0];
    wire plausible = adc_code >= 10'd540 && adc_code <= 10'd720;
    wire [5:0] offset = accepted_code[5:0] - 6'd24;
    wire [7:0] curve_duty = accepted_code <= 10'd600 ? 8'd0 :
                            accepted_code >= 10'd664 ? 8'hff :
                            {offset, 2'b00};
    wire sensor_stale = age_seconds >= 2;
    assign duty = !locked || fault || sensor_stale ? 8'hff : curve_duty;

    always @(posedge clk or negedge locked) begin
        if (!locked) begin
            second_count <= 0;
            age_seconds <= 2;
            warmup_seconds <= 0;
            stalled_seconds <= 0;
            accepted_code <= 0;
            fault <= 0;
        end else begin
            if (second_tick) second_count <= 0;
            else second_count <= second_count + 1'b1;

            if (adc_valid) begin
                if (plausible) begin
                    accepted_code <= adc_code;
                    age_seconds <= 0;
                end else begin
                    // Calibration can yield a transient invalid first
                    // conversion. Run full speed and recover on a valid one.
                    age_seconds <= 2;
                end
            end else if (second_tick && age_seconds != 2) begin
                age_seconds <= age_seconds + 1'b1;
            end

            if (second_tick) begin
                if (warmup_seconds != 3) warmup_seconds <= warmup_seconds + 1'b1;
                if (warmup_seconds == 3 && !sensor_stale &&
                    curve_duty >= 8'd64 && rpm == 0) begin
                    if (stalled_seconds == 2) fault <= 1;
                    else stalled_seconds <= stalled_seconds + 1'b1;
                end else begin
                    stalled_seconds <= 0;
                end
            end
        end
    end
endmodule
