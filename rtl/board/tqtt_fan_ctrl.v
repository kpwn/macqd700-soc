// TQTT J10 four-wire fan. Q9 pulls the fan's PWM input low when fan_pwm=1.
// The fan's own pull-up makes fan_pwm=0 the full-speed, fail-safe state.
module tqtt_fan_ctrl #(
    parameter integer CLK_HZ = 200_000_000
) (
    input  wire       clk,
    input  wire       locked,
    input  wire       fan_fb,
    input  wire [7:0] duty,
    output wire       fan_pwm,
    output reg  [15:0] rpm
);
    // 256 steps at about 25 kHz (25.2 kHz at 200 MHz; 24.4 kHz at 100 MHz).
    localparam integer TICK_DIV = (CLK_HZ + 3_200_000) / 6_400_000;
    localparam integer TICK_W = $clog2(TICK_DIV);
    localparam integer SEC_W = $clog2(CLK_HZ);
    localparam integer TICK_LAST = TICK_DIV - 1;
    localparam integer SECOND_LAST = CLK_HZ - 1;
    reg [TICK_W-1:0] tick = 0;
    reg [7:0] phase = 0;
    reg [SEC_W-1:0] second_count = 0;
    reg startup_done = 0;
    reg fb_meta = 1, fb_sync = 1, fb_last = 1;
    reg [10:0] tach_pulses = 0;

    // Do not let a debug setting stop the fan while temperature control is
    // absent. 64/256 is the minimum until speed and temperature are proven.
    wire [7:0] safe_duty = duty < 8'd64 ? 8'd64 : duty;
    assign fan_pwm = !locked || !startup_done ||
                     safe_duty == 8'hff || phase < safe_duty ? 1'b0 : 1'b1;

    always @(posedge clk or negedge locked) begin
        if (!locked) begin
            tick <= 0;
            phase <= 0;
            second_count <= 0;
            startup_done <= 0;
            fb_meta <= 1;
            fb_sync <= 1;
            fb_last <= 1;
            tach_pulses <= 0;
            rpm <= 0;
        end else begin
            if (tick == TICK_LAST[TICK_W-1:0]) begin
                tick <= 0;
                phase <= phase + 1'b1;
            end else begin
                tick <= tick + 1'b1;
            end
            fb_meta <= fan_fb;
            fb_sync <= fb_meta;
            fb_last <= fb_sync;
            if (second_count == SECOND_LAST[SEC_W-1:0]) begin
                second_count <= 0;
                startup_done <= 1;
                // Standard four-wire tach outputs two pulses per revolution.
                rpm <= tach_pulses * 16'd30;
                tach_pulses <= 0;
            end else begin
                second_count <= second_count + 1'b1;
                if (fb_last && !fb_sync && tach_pulses != 11'h7ff)
                    tach_pulses <= tach_pulses + 1'b1;
            end
        end
    end
endmodule
