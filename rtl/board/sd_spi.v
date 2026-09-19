// sd_spi.v — SPI master for SD card communication (boot + SCSI path)
//
// SPI mode 0 (CPOL=0, CPHA=0), MSB-first.  One byte per cmd_valid handshake.
//
// Ported near-verbatim from sd-hdmi-bringup/rtl/sd_spi.v.  The only change
// is the parameter defaults, recalibrated for m68k-ooo's 200 MHz core clock
// (was 148.5 MHz pclk in the source project).  The 200 MHz numbers yield:
//
//   SLOW_HALF = 250 → ~400 kHz   (power-on init, ≤400 kHz per SD spec)
//   FAST_HALF =   4 → 25.0 MHz   (default SD SPI rate)
//   HS_HALF   =   2 → 50.0 MHz   (post-CMD6 High-Speed switch)
//
// HS_HALF note: at 200 MHz core clock, HS_HALF=2 yields 200/(2*2) = 50.00
// MHz SPI — the High-Speed mode ceiling.  Upstream sd-hdmi-bringup runs
// at 198 MHz with HS_HALF=2 (= 49.5 MHz), validated on hardware.  The SD
// spec permits 50 MHz in HS mode so this is the intended rate.
//
// The top-level may override these if a different core clock is used; the
// parameter names are kept identical to the source to simplify instantiation.
//
// Ports (three groups):
//   - cmd_data/cmd_valid/cmd_ready : byte-sized producer handshake
//   - rsp_data/rsp_valid           : byte received on last 8 SPI clocks
//   - spi_clk/spi_mosi/spi_miso/spi_cs_n : physical SD card pins
//   - cs_n_in                       : external CS driver (owned by bus mux)
//   - fast_mode / hs_mode           : select among the three divider values
//
// Latency: one byte transfer takes 16 × half_limit core cycles (~40 µs at
// 400 kHz init, ~320 ns at 25 MHz fast).  MISO is double-flop synchronised
// and sampled at the falling edge (end of HI phase).

module sd_spi #(
    parameter SLOW_HALF = 250,
    parameter FAST_HALF = 4,
    parameter HS_HALF   = 2
) (
    input  wire       clk,
    input  wire       rst,

    input  wire       fast_mode,
    input  wire       hs_mode,

    input  wire       cs_n_in,

    input  wire       cmd_valid,
    output reg        cmd_ready,
    input  wire [7:0] cmd_data,

    output reg        rsp_valid,
    output reg  [7:0] rsp_data,

    output reg        spi_clk,
    output reg        spi_mosi,
    input  wire       spi_miso,
    output reg        spi_cs_n
);
    localparam [1:0]
        S_IDLE = 2'd0,
        S_LO   = 2'd1,
        S_HI   = 2'd2,
        S_DONE = 2'd3;

    reg  [1:0] state;
    reg  [7:0] tx_sh;
    reg  [7:0] rx_sh;
    reg  [3:0] bit_cnt;
    reg  [8:0] phase_cnt;

    wire [8:0] half_limit = fast_mode
        ? (hs_mode ? (HS_HALF - 1) : (FAST_HALF - 1))
        : (SLOW_HALF - 1);
    wire       phase_tick = (phase_cnt == half_limit);

    // 2-flop sync of MISO.  Init to 2'b11 because SD MISO idles high
    // (pulled-up); explicit reset keeps the first 1-2 SPI clocks of a
    // post-umbrella-reset boot deterministic instead of inheriting
    // whatever the FFs latched at the previous boot's last edge.
    reg [1:0] miso_sync;
    always @(posedge clk) begin
        if (rst) miso_sync <= 2'b11;
        else     miso_sync <= {miso_sync[0], spi_miso};
    end

    always @(posedge clk)
        if (rst) spi_cs_n <= 1'b1;
        else     spi_cs_n <= cs_n_in;

    always @(posedge clk) begin
        if (rst) begin
            state     <= S_IDLE;
            cmd_ready <= 1'b0;
            rsp_valid <= 1'b0;
            rsp_data  <= 8'h00;
            spi_clk   <= 1'b0;
            spi_mosi  <= 1'b1;
            tx_sh     <= 8'h00;
            rx_sh     <= 8'h00;
            bit_cnt   <= 4'd0;
            phase_cnt <= 9'd0;
        end else begin
            rsp_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    cmd_ready <= 1'b1;
                    spi_clk   <= 1'b0;
                    if (cmd_valid && cmd_ready) begin
                        cmd_ready <= 1'b0;
                        tx_sh     <= cmd_data;
                        spi_mosi  <= cmd_data[7];
                        bit_cnt   <= 4'd0;
                        phase_cnt <= 9'd0;
                        state     <= S_LO;
                    end
                end

                S_LO: begin
                    spi_clk <= 1'b0;
                    if (phase_tick) begin
                        phase_cnt <= 9'd0;
                        spi_clk   <= 1'b1;
                        state     <= S_HI;
                    end else begin
                        phase_cnt <= phase_cnt + 9'd1;
                    end
                end

                S_HI: begin
                    spi_clk <= 1'b1;
                    if (phase_tick) begin
                        phase_cnt <= 9'd0;
                        rx_sh     <= {rx_sh[6:0], miso_sync[1]};
                        spi_clk   <= 1'b0;
                        tx_sh     <= {tx_sh[6:0], 1'b1};
                        spi_mosi  <= tx_sh[6];
                        bit_cnt   <= bit_cnt + 4'd1;
                        if (bit_cnt == 4'd7) state <= S_DONE;
                        else                 state <= S_LO;
                    end else begin
                        phase_cnt <= phase_cnt + 9'd1;
                    end
                end

                S_DONE: begin
                    spi_clk   <= 1'b0;
                    spi_mosi  <= 1'b1;
                    rsp_data  <= rx_sh;
                    rsp_valid <= 1'b1;
                    state     <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
