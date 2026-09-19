// uart_byte_bridge.v - byte-level 8N1 UART adapter for SCC external byte pins.
//
// The SCC RTL exposes byte-complete TX pulses and byte-valid RX pulses.  This
// adapter converts one board UART RX/TX pair to that byte interface.  It is not
// an SCC replacement; modem control, SCC FIFOs, interrupts, and register status
// remain in rtl/mac/scc.v.

`default_nettype none

module uart_byte_bridge #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115200
) (
    input  wire       clk,
    input  wire       rst,

    input  wire       uart_rx,
    output wire       uart_tx,

    output reg        rx_valid,
    output reg [7:0]  rx_data,

    input  wire       tx_valid,
    input  wire [7:0] tx_data,
    output wire       tx_busy
);
    localparam integer BAUD_DIV_INT = (CLK_HZ + (BAUD / 2)) / BAUD;
    localparam integer BAUD_DIV = (BAUD_DIV_INT < 1) ? 1 : BAUD_DIV_INT;
    localparam integer BAUD_CNT_W = (BAUD_DIV <= 2) ? 2 : $clog2(BAUD_DIV + 1);

    (* ASYNC_REG = "TRUE" *) reg uart_rx_meta;
    (* ASYNC_REG = "TRUE" *) reg uart_rx_sync;
    always @(posedge clk) begin
        if (rst) begin
            uart_rx_meta <= 1'b1;
            uart_rx_sync <= 1'b1;
        end else begin
            uart_rx_meta <= uart_rx;
            uart_rx_sync <= uart_rx_meta;
        end
    end

    reg        rx_active;
    reg [3:0]  rx_bit_idx;
    reg [7:0]  rx_shift;
    reg [BAUD_CNT_W-1:0] rx_baud_cnt;

    always @(posedge clk) begin
        if (rst) begin
            rx_active   <= 1'b0;
            rx_bit_idx  <= 4'd0;
            rx_shift    <= 8'h00;
            rx_baud_cnt <= {BAUD_CNT_W{1'b0}};
            rx_valid    <= 1'b0;
            rx_data     <= 8'h00;
        end else begin
            rx_valid <= 1'b0;
            if (!rx_active) begin
                rx_bit_idx <= 4'd0;
                if (!uart_rx_sync) begin
                    rx_active   <= 1'b1;
                    rx_baud_cnt <= (BAUD_DIV / 2);
                end
            end else if (rx_baud_cnt != 0) begin
                rx_baud_cnt <= rx_baud_cnt - 1'b1;
            end else begin
                rx_baud_cnt <= BAUD_DIV - 1;
                if (rx_bit_idx == 4'd0) begin
                    if (uart_rx_sync)
                        rx_active <= 1'b0;
                    rx_bit_idx <= 4'd1;
                end else if (rx_bit_idx <= 4'd8) begin
                    rx_shift   <= {uart_rx_sync, rx_shift[7:1]};
                    rx_bit_idx <= rx_bit_idx + 1'b1;
                end else begin
                    rx_active <= 1'b0;
                    if (uart_rx_sync) begin
                        rx_data  <= rx_shift;
                        rx_valid <= 1'b1;
                    end
                end
            end
        end
    end

    reg        tx_active;
    reg [3:0]  tx_bit_idx;
    reg [9:0]  tx_shift;
    reg [BAUD_CNT_W-1:0] tx_baud_cnt;
    reg        tx_pin;
    reg        tx_pending;
    reg [7:0]  tx_pending_data;

    assign uart_tx = tx_pin;
    assign tx_busy = tx_active | tx_pending;

    always @(posedge clk) begin
        if (rst) begin
            tx_active       <= 1'b0;
            tx_bit_idx      <= 4'd0;
            tx_shift        <= 10'h3ff;
            tx_baud_cnt     <= {BAUD_CNT_W{1'b0}};
            tx_pin          <= 1'b1;
            tx_pending      <= 1'b0;
            tx_pending_data <= 8'h00;
        end else begin
            if (tx_valid) begin
                if (!tx_active && !tx_pending) begin
                    tx_active   <= 1'b1;
                    tx_bit_idx  <= 4'd0;
                    tx_shift    <= {1'b1, tx_data, 1'b0};
                    tx_baud_cnt <= BAUD_DIV - 1;
                    tx_pin      <= 1'b0;
                end else if (!tx_pending) begin
                    tx_pending      <= 1'b1;
                    tx_pending_data <= tx_data;
                end
            end

            if (tx_active) begin
                if (tx_baud_cnt != 0) begin
                    tx_baud_cnt <= tx_baud_cnt - 1'b1;
                end else if (tx_bit_idx == 4'd9) begin
                    if (tx_pending) begin
                        tx_pending  <= 1'b0;
                        tx_bit_idx  <= 4'd0;
                        tx_shift    <= {1'b1, tx_pending_data, 1'b0};
                        tx_baud_cnt <= BAUD_DIV - 1;
                        tx_pin      <= 1'b0;
                    end else begin
                        tx_active <= 1'b0;
                        tx_pin    <= 1'b1;
                    end
                end else begin
                    tx_bit_idx  <= tx_bit_idx + 1'b1;
                    tx_shift    <= {1'b1, tx_shift[9:1]};
                    tx_pin      <= tx_shift[1];
                    tx_baud_cnt <= BAUD_DIV - 1;
                end
            end
        end
    end
endmodule

`default_nettype wire
