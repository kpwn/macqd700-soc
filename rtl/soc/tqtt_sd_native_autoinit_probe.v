// Standalone TQTT native-card initialization probe. The card must have been
// power-cycled out of SPI mode before this bitstream starts. VIO inputs:
//   probe_in0 card sectors, probe_in1 RCA, probe_in2 init phase,
//   probe_in3 {error,ready}, probe_in4 {command timeout,event}.
// VIO probe_out0 bit 0 requests a controller reset for repeatable diagnosis.
module tqtt_sd_native_autoinit_probe (
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
    wire [31:0] vio_control;
    wire rst = (por != 0) || vio_control[0];

    wire ready, init_error;
    wire [31:0] card_num_lbas, rca;
    wire [4:0] init_phase;
    wire [3:0] cmd_event;
    wire cmd_timeout;
    wire [29:0] wb_adr;
    wire [1:0] wb_bte;
    wire [2:0] wb_cti;
    wire wb_cyc;
    wire [31:0] wb_dat_r, wb_dat_w;
    wire wb_ack, wb_err;
    wire [3:0] wb_sel;
    wire wb_stb, wb_we;
    wire irq;
    wire sd_cmd_dir, sd_dat0_dir, sd_dat13_dir;

    sd_native_init_controller u_init (
        .clk(clk), .rst(rst), .ready(ready), .error(init_error),
        .card_num_lbas(card_num_lbas), .rca(rca),
        .debug_phase(init_phase), .debug_cmd_event(cmd_event),
        .debug_cmd_timeout(cmd_timeout),
        .wb_adr(wb_adr), .wb_bte(wb_bte), .wb_cti(wb_cti),
        .wb_cyc(wb_cyc), .wb_dat_r(wb_dat_r), .wb_dat_w(wb_dat_w),
        .wb_ack(wb_ack), .wb_err(wb_err), .wb_sel(wb_sel),
        .wb_stb(wb_stb), .wb_we(wb_we)
    );

    litesdcard_core u_sd (
        .clk(clk), .rst(rst), .irq(irq), .sdcard_cd(1'b0),
        .sdcard_clk(sd_clk), .sdcard_cmd(sd_cmd),
        .sdcard_cmd_dir(sd_cmd_dir), .sdcard_dat0_dir(sd_dat0_dir),
        .sdcard_dat13_dir(sd_dat13_dir), .sdcard_data(sd_dat),
        .wb_ctrl_ack(wb_ack), .wb_ctrl_adr(wb_adr),
        .wb_ctrl_bte(wb_bte), .wb_ctrl_cti(wb_cti),
        .wb_ctrl_cyc(wb_cyc), .wb_ctrl_dat_r(wb_dat_r),
        .wb_ctrl_dat_w(wb_dat_w), .wb_ctrl_err(wb_err),
        .wb_ctrl_sel(wb_sel), .wb_ctrl_stb(wb_stb), .wb_ctrl_we(wb_we),
        .wb_dma_ack(1'b0), .wb_dma_adr(), .wb_dma_bte(),
        .wb_dma_cti(), .wb_dma_cyc(), .wb_dma_dat_r(32'b0),
        .wb_dma_dat_w(), .wb_dma_err(1'b0),
        .wb_dma_sel(), .wb_dma_stb(), .wb_dma_we()
    );

    wire [31:0] vio_phase = {27'b0, init_phase};
    wire [31:0] vio_state = {30'b0, init_error, ready};
    wire [31:0] vio_event = {27'b0, cmd_timeout, cmd_event};
    native_sd_vio u_vio (
        .clk(clk),
        .probe_in0(card_num_lbas), .probe_in1(rca),
        .probe_in2(vio_phase), .probe_in3(vio_state),
        .probe_in4(vio_event),
        .probe_out0(vio_control), .probe_out1(),
        .probe_out2(), .probe_out3()
    );
endmodule
