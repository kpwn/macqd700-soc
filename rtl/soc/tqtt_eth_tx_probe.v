// TQTT RTL8211EG GMII transmit/receive probe.
// Sends an identifiable broadcast Ethernet frame once per second, including
// preamble, SFD, and Ethernet FCS. JTAG VIO exposes RX and TX counters.
`default_nettype none
module tqtt_eth_tx_probe (
    input  wire       sys_clk_p,
    input  wire       sys_clk_n,
    input  wire       eth_rxclk,
    input  wire       eth_rxdv,
    input  wire       eth_rxer,
    output wire       eth_rst_n,
    output wire       eth_gtx_clk,
    output reg        eth_txen = 0,
    output wire       eth_txer,
    output reg  [7:0] eth_txd = 0
);
    wire sys_clk_unbuf;
    wire sys_clk;
    wire rx_clk;
    IBUFDS u_sys_clk_ibuf (.I(sys_clk_p), .IB(sys_clk_n), .O(sys_clk_unbuf));
    BUFG u_sys_clk_bufg (.I(sys_clk_unbuf), .O(sys_clk));
    BUFG u_rx_clk_bufg (.I(eth_rxclk), .O(rx_clk));

    // GMII needs a 125 MHz MAC-to-PHY clock even while TXEN stays idle.
    // 100 / 2 * 25 / 10 = 125 MHz. Hold the PHY in reset until the MMCM
    // has locked and its forwarded clock has been stable for about 1 ms.
    wire tx_clk_unbuf;
    wire tx_clk;
    wire tx_clkfb;
    wire tx_locked;
    MMCME4_BASE #(
        .CLKIN1_PERIOD(10.0), .DIVCLK_DIVIDE(2),
        .CLKFBOUT_MULT_F(25.0), .CLKOUT0_DIVIDE_F(10.0),
        .BANDWIDTH("OPTIMIZED"), .STARTUP_WAIT("FALSE")
    ) u_tx_mmcm (
        .CLKIN1(sys_clk), .CLKFBIN(tx_clkfb), .CLKFBOUT(tx_clkfb),
        .CLKFBOUTB(), .CLKOUT0(tx_clk_unbuf), .CLKOUT0B(),
        .CLKOUT1(), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(),
        .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
        .RST(1'b0), .PWRDWN(1'b0), .LOCKED(tx_locked)
    );
    BUFG u_tx_clk_bufg (.I(tx_clk_unbuf), .O(tx_clk));
    reg [16:0] phy_reset_delay = 0;
    always @(posedge sys_clk) begin
        if (!tx_locked)
            phy_reset_delay <= 0;
        else if (phy_reset_delay != 17'd100000)
            phy_reset_delay <= phy_reset_delay + 1'b1;
    end
    assign eth_rst_n = (phy_reset_delay == 17'd100000);
    ODDRE1 u_gtx_clock_fwd (
        .C(tx_clk), .D1(1'b1), .D2(1'b0), .SR(!tx_locked), .Q(eth_gtx_clk)
    );
    assign eth_txer = 1'b0;

    function [31:0] crc_byte;
        input [31:0] current;
        input [7:0] value;
        reg [31:0] c;
        integer bitno;
        begin
            c = current;
            for (bitno = 0; bitno < 8; bitno = bitno + 1)
                c = (c[0] ^ value[bitno]) ?
                    ((c >> 1) ^ 32'hedb88320) : (c >> 1);
            crc_byte = c;
        end
    endfunction

    function [7:0] frame_byte;
        input [5:0] position;
        input [31:0] frame_id;
        begin
            case (position)
                0, 1, 2, 3, 4, 5: frame_byte = 8'hff;
                6: frame_byte = 8'h02;
                7: frame_byte = 8'h54;
                8: frame_byte = 8'h51;
                9: frame_byte = 8'h54;
                10: frame_byte = 8'h54;
                11: frame_byte = 8'h01;
                12: frame_byte = 8'h88;
                13: frame_byte = 8'hb5;
                14: frame_byte = frame_id[7:0];
                15: frame_byte = frame_id[15:8];
                16: frame_byte = frame_id[23:16];
                17: frame_byte = frame_id[31:24];
                18: frame_byte = 8'h54; // TQTT
                19: frame_byte = 8'h51;
                20: frame_byte = 8'h54;
                21: frame_byte = 8'h54;
                default: frame_byte = {2'b00, position} ^ 8'ha5;
            endcase
        end
    endfunction

    localparam [2:0] IDLE = 0, PREAMBLE = 1, BODY = 2,
                     FCS = 3, INTERFRAME = 4;
    reg [2:0] tx_state = IDLE;
    reg [26:0] interval = 0;
    reg [5:0] byte_index = 0;
    reg [31:0] crc = 32'hffffffff;
    reg [31:0] fcs = 0;
    reg [31:0] frame_id = 0;
    wire [7:0] body_byte = frame_byte(byte_index, frame_id);
    wire [31:0] crc_next = crc_byte(crc, body_byte);
    (* ASYNC_REG = "TRUE" *) reg [1:0] phy_ready_tx = 0;
    always @(negedge tx_clk)
        phy_ready_tx <= {phy_ready_tx[0], eth_rst_n};

    // Change GMII data on the falling edge; the forwarded GTX clock samples
    // it on the next rising edge at the PHY.
    always @(negedge tx_clk) begin
        if (!phy_ready_tx[1]) begin
            eth_txen <= 0;
            eth_txd <= 0;
            tx_state <= IDLE;
            interval <= 0;
            byte_index <= 0;
            crc <= 32'hffffffff;
            fcs <= 0;
            frame_id <= 0;
        end else begin
            case (tx_state)
                IDLE: begin
                    eth_txen <= 0;
                    if (interval == 27'd124999999) begin
                        interval <= 0;
                        byte_index <= 0;
                        tx_state <= PREAMBLE;
                    end else interval <= interval + 1'b1;
                end
                PREAMBLE: begin
                    eth_txen <= 1;
                    if (byte_index == 6'd7) begin
                        eth_txd <= 8'hd5;
                        byte_index <= 0;
                        crc <= 32'hffffffff;
                        tx_state <= BODY;
                    end else begin
                        eth_txd <= 8'h55;
                        byte_index <= byte_index + 1'b1;
                    end
                end
                BODY: begin
                    eth_txen <= 1;
                    eth_txd <= body_byte;
                    crc <= crc_next;
                    if (byte_index == 6'd59) begin
                        fcs <= ~crc_next;
                        byte_index <= 0;
                        tx_state <= FCS;
                    end else byte_index <= byte_index + 1'b1;
                end
                FCS: begin
                    eth_txen <= 1;
                    eth_txd <= fcs[8*byte_index +: 8];
                    if (byte_index == 6'd3) begin
                        frame_id <= frame_id + 1'b1;
                        byte_index <= 0;
                        tx_state <= INTERFRAME;
                    end else byte_index <= byte_index + 1'b1;
                end
                INTERFRAME: begin
                    eth_txen <= 0;
                    if (byte_index == 6'd11) begin
                        byte_index <= 0;
                        tx_state <= IDLE;
                    end else byte_index <= byte_index + 1'b1;
                end
                default: tx_state <= IDLE;
            endcase
        end
    end

    reg [31:0] edge_count = 0;
    reg [31:0] frame_count = 0;
    reg [31:0] error_count = 0;
    reg        rxdv_prev = 0;
    always @(posedge rx_clk) begin
        edge_count <= edge_count + 1'b1;
        rxdv_prev <= eth_rxdv;
        if (eth_rxdv && !rxdv_prev)
            frame_count <= frame_count + 1'b1;
        if (eth_rxdv && eth_rxer)
            error_count <= error_count + 1'b1;
    end

    wire [31:0] edge_gray = edge_count ^ (edge_count >> 1);
    wire [31:0] frame_gray = frame_count ^ (frame_count >> 1);
    wire [31:0] error_gray = error_count ^ (error_count >> 1);
    wire [31:0] tx_gray = frame_id ^ (frame_id >> 1);
    (* ASYNC_REG = "TRUE" *) reg [31:0] edge_gray_meta = 0;
    (* ASYNC_REG = "TRUE" *) reg [31:0] edge_gray_sync = 0;
    (* ASYNC_REG = "TRUE" *) reg [31:0] frame_gray_meta = 0;
    (* ASYNC_REG = "TRUE" *) reg [31:0] frame_gray_sync = 0;
    (* ASYNC_REG = "TRUE" *) reg [31:0] error_gray_meta = 0;
    (* ASYNC_REG = "TRUE" *) reg [31:0] error_gray_sync = 0;
    (* ASYNC_REG = "TRUE" *) reg [31:0] tx_gray_meta = 0;
    (* ASYNC_REG = "TRUE" *) reg [31:0] tx_gray_sync = 0;
    always @(posedge sys_clk) begin
        edge_gray_meta <= edge_gray;
        edge_gray_sync <= edge_gray_meta;
        frame_gray_meta <= frame_gray;
        frame_gray_sync <= frame_gray_meta;
        error_gray_meta <= error_gray;
        error_gray_sync <= error_gray_meta;
        tx_gray_meta <= tx_gray;
        tx_gray_sync <= tx_gray_meta;
    end

    // VIO reads Gray values so every sampled count is coherent across the
    // asynchronous clock boundary. The host converts Gray to binary.
    (* DONT_TOUCH = "true" *) tqtt_eth_tx_vio u_vio (
        .clk(sys_clk),
        .probe_in0(edge_gray_sync),
        .probe_in1(frame_gray_sync),
        .probe_in2(error_gray_sync),
        .probe_in3(tx_gray_sync)
    );
endmodule
`default_nettype wire
