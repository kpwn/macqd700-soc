// Read-only TQTT RTL8211EG GMII receive probe.
// No packet transmission: TX controls and data are held idle while JTAG VIO
// samples synchronized counters for RX clock, frames, and in-frame RX errors.
`default_nettype none
module tqtt_eth_probe (
    input  wire       sys_clk_p,
    input  wire       sys_clk_n,
    input  wire       eth_rxclk,
    input  wire       eth_rxdv,
    input  wire       eth_rxer,
    output wire       eth_rst_n,
    output wire       eth_gtx_clk,
    output wire       eth_txen,
    output wire       eth_txer,
    output wire [7:0] eth_txd
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
    assign eth_txen = 1'b0;
    assign eth_txer = 1'b0;
    assign eth_txd = 8'd0;

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
    (* ASYNC_REG = "TRUE" *) reg [31:0] edge_gray_meta = 0;
    (* ASYNC_REG = "TRUE" *) reg [31:0] edge_gray_sync = 0;
    (* ASYNC_REG = "TRUE" *) reg [31:0] frame_gray_meta = 0;
    (* ASYNC_REG = "TRUE" *) reg [31:0] frame_gray_sync = 0;
    (* ASYNC_REG = "TRUE" *) reg [31:0] error_gray_meta = 0;
    (* ASYNC_REG = "TRUE" *) reg [31:0] error_gray_sync = 0;
    always @(posedge sys_clk) begin
        edge_gray_meta <= edge_gray;
        edge_gray_sync <= edge_gray_meta;
        frame_gray_meta <= frame_gray;
        frame_gray_sync <= frame_gray_meta;
        error_gray_meta <= error_gray;
        error_gray_sync <= error_gray_meta;
    end

    // VIO reads Gray values so every sampled count is coherent across the
    // asynchronous clock boundary. The host converts Gray to binary.
    (* DONT_TOUCH = "true" *) tqtt_eth_vio u_vio (
        .clk(sys_clk),
        .probe_in0(edge_gray_sync),
        .probe_in1(frame_gray_sync),
        .probe_in2(error_gray_sync)
    );
endmodule
`default_nettype wire
