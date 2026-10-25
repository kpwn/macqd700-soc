// Standalone four-bit SD probe. LiteSDCard runs at the TQTT 100 MHz fabric
// reference; a VIO-driven Wishbone master gives the host direct access to its
// CSRs. The DMA master terminates in a 16 KiB local RAM for data comparison.
// The host must initialize a freshly power-cycled card in native mode.
module tqtt_sd_native_probe (
    input  wire       sys_clk_p,
    input  wire       sys_clk_n,
    output wire       sd_clk,
    inout  wire       sd_cmd,
    inout  wire [3:0] sd_dat
);
    wire clk_ibuf;
    wire clk;
    IBUFDS u_clk_ibuf (.I(sys_clk_p), .IB(sys_clk_n), .O(clk_ibuf));
    BUFG u_clk_bufg (.I(clk_ibuf), .O(clk));

    reg [7:0] por = 8'hff;
    always @(posedge clk)
        if (por != 0) por <= por - 1'b1;

    wire [31:0] vio_addr;
    wire [31:0] vio_wdata;
    wire [3:0]  vio_ctrl;
    wire [11:0] vio_mem_addr;
    wire rst = (por != 0) | vio_ctrl[2];

    reg  [31:0] wb_rdata = 0;
    reg         wb_done_toggle = 0;
    reg         wb_error = 0;
    reg         seen_toggle = 0;
    reg         wb_cyc = 0;
    reg         wb_we = 0;
    reg  [29:0] wb_adr = 0;
    reg  [31:0] wb_wdata = 0;
    wire        wb_ack;
    wire        wb_err;
    wire [31:0] wb_dat_r;
    wire        irq;
    wire        sd_cmd_dir, sd_dat0_dir, sd_dat13_dir;

    always @(posedge clk) begin
        if (rst) begin
            wb_cyc <= 0;
            seen_toggle <= vio_ctrl[0];
            wb_done_toggle <= 0;
            wb_error <= 0;
        end else if (!wb_cyc) begin
            if (vio_ctrl[0] != seen_toggle) begin
                seen_toggle <= vio_ctrl[0];
                wb_adr <= vio_addr[31:2];
                wb_wdata <= vio_wdata;
                wb_we <= vio_ctrl[1];
                wb_cyc <= 1;
            end
        end else if (wb_ack || wb_err) begin
            wb_rdata <= wb_dat_r;
            wb_error <= wb_err;
            wb_done_toggle <= ~wb_done_toggle;
            wb_cyc <= 0;
        end
    end

    wire        dma_ack;
    wire [29:0] dma_adr;
    wire [1:0]  dma_bte;
    wire [2:0]  dma_cti;
    wire        dma_cyc;
    wire [31:0] dma_dat_w;
    wire [3:0]  dma_sel;
    wire        dma_stb;
    wire        dma_we;
    reg  [31:0] dma_dat_r = 0;
    reg         dma_ack_q = 0;
    reg  [23:0] dma_words = 0;
    reg  [31:0] dma_crc = 32'hffffffff;
    reg  [23:0] dma_expected_words = 0;
    reg  [31:0] dma_cycles = 0;
    reg         dma_timer_running = 0;
    reg  [31:0] ram [0:4095];
    reg  [31:0] vio_mem_data = 0;

    function [31:0] crc32_byte;
        input [31:0] old_crc;
        input [7:0] byte_in;
        integer bit_idx;
        reg [31:0] c;
        begin
            c = old_crc ^ {24'b0, byte_in};
            for (bit_idx = 0; bit_idx < 8; bit_idx = bit_idx + 1)
                c = c[0] ? (c >> 1) ^ 32'hedb88320 : c >> 1;
            crc32_byte = c;
        end
    endfunction

    always @(posedge clk) begin
        vio_mem_data <= ram[vio_mem_addr];
        if (rst) begin
            dma_ack_q <= 0;
            dma_words <= 0;
            dma_crc <= 32'hffffffff;
            dma_expected_words <= 0;
            dma_cycles <= 0;
            dma_timer_running <= 0;
        end else begin
            dma_ack_q <= dma_cyc && dma_stb && !dma_ack_q;
            if (wb_cyc && wb_ack && wb_we && wb_adr == (30'h808 >> 2))
                dma_expected_words <= wb_wdata[25:2];
            if (vio_ctrl[3]) begin
                dma_words <= 0;
                dma_crc <= 32'hffffffff;
                dma_cycles <= 0;
                dma_timer_running <= 0;
            end
            if (dma_timer_running) dma_cycles <= dma_cycles + 1'b1;
            if (dma_cyc && dma_stb && !dma_ack_q) begin
                dma_dat_r <= ram[dma_adr[11:0]];
                if (dma_we) begin
                    // The DMA window is local and base zero. Keep all words
                    // for host-side byte comparison after each transfer.
                    ram[dma_adr[11:0]] <= dma_dat_w;
                    dma_words <= dma_words + 1'b1;
                    if (dma_words == 0) dma_timer_running <= 1;
                    if (dma_expected_words != 0 &&
                        dma_words + 1'b1 == dma_expected_words)
                        dma_timer_running <= 0;
                    if (dma_sel == 4'hf)
                        dma_crc <= crc32_byte(crc32_byte(crc32_byte(crc32_byte(
                            dma_crc, dma_dat_w[31:24]), dma_dat_w[23:16]),
                            dma_dat_w[15:8]), dma_dat_w[7:0]);
                end
            end
        end
    end
    assign dma_ack = dma_ack_q;

    litesdcard_core u_sd (
        .clk(clk), .rst(rst), .irq(irq), .sdcard_cd(1'b0),
        .sdcard_clk(sd_clk), .sdcard_cmd(sd_cmd),
        .sdcard_cmd_dir(sd_cmd_dir), .sdcard_dat0_dir(sd_dat0_dir),
        .sdcard_dat13_dir(sd_dat13_dir), .sdcard_data(sd_dat),
        .wb_ctrl_ack(wb_ack), .wb_ctrl_adr(wb_adr),
        .wb_ctrl_bte(2'b00), .wb_ctrl_cti(3'b000),
        .wb_ctrl_cyc(wb_cyc), .wb_ctrl_dat_r(wb_dat_r),
        .wb_ctrl_dat_w(wb_wdata), .wb_ctrl_err(wb_err),
        .wb_ctrl_sel(4'hf), .wb_ctrl_stb(wb_cyc), .wb_ctrl_we(wb_we),
        .wb_dma_ack(dma_ack), .wb_dma_adr(dma_adr),
        .wb_dma_bte(dma_bte), .wb_dma_cti(dma_cti),
        .wb_dma_cyc(dma_cyc), .wb_dma_dat_r(dma_dat_r),
        .wb_dma_dat_w(dma_dat_w), .wb_dma_err(1'b0),
        .wb_dma_sel(dma_sel), .wb_dma_stb(dma_stb), .wb_dma_we(dma_we)
    );

    wire [31:0] vio_status = {dma_words, 4'b0, wb_error,
                              wb_done_toggle, irq, rst};
    wire [31:0] vio_crc = dma_crc ^ 32'hffffffff;
    wire [31:0] vio_dma_cycles = dma_cycles;
    native_sd_vio u_vio (
        .clk(clk),
        .probe_in0(wb_rdata), .probe_in1(vio_status),
        .probe_in2(vio_crc), .probe_in3(vio_mem_data),
        .probe_in4(vio_dma_cycles),
        .probe_out0(vio_addr), .probe_out1(vio_wdata),
        .probe_out2(vio_ctrl), .probe_out3(vio_mem_addr)
    );
endmodule
