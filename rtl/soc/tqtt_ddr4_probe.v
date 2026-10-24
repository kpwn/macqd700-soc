// Standalone TQTT DDR4 calibration and full-address-range AXI probe.
// Writes 4096 contiguous 256-bit words and 4096 words 256 KiB apart,
// then reads them all back. The two regions do not overlap.
// Results are visible through tqtt_ddr4_vio on the MIG UI clock.
module tqtt_ddr4_probe (
    input wire sys_rst,
    input wire c0_sys_clk_p,
    input wire c0_sys_clk_n,
    output wire c0_ddr4_act_n,
    output wire [16:0] c0_ddr4_adr,
    output wire [1:0] c0_ddr4_ba,
    output wire [0:0] c0_ddr4_bg,
    output wire [0:0] c0_ddr4_cke,
    output wire [0:0] c0_ddr4_odt,
    output wire [0:0] c0_ddr4_cs_n,
    output wire [0:0] c0_ddr4_ck_t,
    output wire [0:0] c0_ddr4_ck_c,
    output wire c0_ddr4_reset_n,
    inout wire [3:0] c0_ddr4_dm_dbi_n,
    inout wire [31:0] c0_ddr4_dq,
    inout wire [3:0] c0_ddr4_dqs_c,
    inout wire [3:0] c0_ddr4_dqs_t
);
    localparam [3:0] WAIT_CAL = 0, WRITE = 1, WRITE_RESP = 2,
                     READ_ADDR = 3, READ_DATA = 4, PASS = 5, FAIL = 6;
    wire ui_clk, ui_rst, cal_done;
    wire mig_sys_rst;
    IBUF u_reset_ibuf (.I(sys_rst), .O(mig_sys_rst));
    wire [511:0] dbg_bus;
    wire dbg_clk;
    reg [3:0] phase = WAIT_CAL;
    reg [12:0] index = 0;
    reg aw_done = 0, w_done = 0;
    reg [3:0] error_code = 0;
    reg [29:0] error_addr = 0;
    reg [31:0] error_xor = 0;
    reg [13:0] verified = 0;
    wire [29:0] test_addr = index[12] ?
        {index[11:0], 1'b1, 17'b0} : {13'b0, index[11:0], 5'b0};
    wire [255:0] expected = {
        (32'hd15ea5e7 ^ {19'b0, index}),
        (32'h6c9e31a2 + {19'b0, index}),
        (32'hf00dcafe ^ {index, 19'h54321}),
        (32'h0badf00d + {index, 19'h2bcde}),
        (32'h12345678 ^ {index, 19'h76543}),
        (32'h89abcdef + {index, 19'h10203}),
        (32'h55aa33cc ^ {index, 19'h70f0f}),
        (32'ha5a55a5a + {19'b0, index})
    };
    wire awready, wready, bvalid, arready, rvalid, rlast;
    wire [1:0] bresp, rresp;
    wire [255:0] rdata;

    design_1_ddr4_0_1 u_mig (
        .sys_rst(mig_sys_rst), .c0_sys_clk_p(c0_sys_clk_p),
        .c0_sys_clk_n(c0_sys_clk_n),
        .c0_ddr4_act_n(c0_ddr4_act_n), .c0_ddr4_adr(c0_ddr4_adr),
        .c0_ddr4_ba(c0_ddr4_ba), .c0_ddr4_bg(c0_ddr4_bg),
        .c0_ddr4_cke(c0_ddr4_cke), .c0_ddr4_odt(c0_ddr4_odt),
        .c0_ddr4_cs_n(c0_ddr4_cs_n), .c0_ddr4_ck_t(c0_ddr4_ck_t),
        .c0_ddr4_ck_c(c0_ddr4_ck_c), .c0_ddr4_reset_n(c0_ddr4_reset_n),
        .c0_ddr4_dm_dbi_n(c0_ddr4_dm_dbi_n), .c0_ddr4_dq(c0_ddr4_dq),
        .c0_ddr4_dqs_c(c0_ddr4_dqs_c), .c0_ddr4_dqs_t(c0_ddr4_dqs_t),
        .c0_init_calib_complete(cal_done), .c0_ddr4_ui_clk(ui_clk),
        .c0_ddr4_ui_clk_sync_rst(ui_rst), .dbg_clk(dbg_clk),
        .dbg_bus(dbg_bus), .c0_ddr4_aresetn(~ui_rst),
        .c0_ddr4_s_axi_awid(1'b0), .c0_ddr4_s_axi_awaddr(test_addr),
        .c0_ddr4_s_axi_awlen(8'd0), .c0_ddr4_s_axi_awsize(3'd5),
        .c0_ddr4_s_axi_awburst(2'b01), .c0_ddr4_s_axi_awlock(1'b0),
        .c0_ddr4_s_axi_awcache(4'b0011), .c0_ddr4_s_axi_awprot(3'b000),
        .c0_ddr4_s_axi_awqos(4'b0000),
        .c0_ddr4_s_axi_awvalid(phase == WRITE && !aw_done),
        .c0_ddr4_s_axi_awready(awready),
        .c0_ddr4_s_axi_wdata(expected), .c0_ddr4_s_axi_wstrb(32'hffffffff),
        .c0_ddr4_s_axi_wlast(1'b1),
        .c0_ddr4_s_axi_wvalid(phase == WRITE && !w_done),
        .c0_ddr4_s_axi_wready(wready),
        .c0_ddr4_s_axi_bready(1'b1), .c0_ddr4_s_axi_bid(),
        .c0_ddr4_s_axi_bresp(bresp), .c0_ddr4_s_axi_bvalid(bvalid),
        .c0_ddr4_s_axi_arid(1'b0), .c0_ddr4_s_axi_araddr(test_addr),
        .c0_ddr4_s_axi_arlen(8'd0), .c0_ddr4_s_axi_arsize(3'd5),
        .c0_ddr4_s_axi_arburst(2'b01), .c0_ddr4_s_axi_arlock(1'b0),
        .c0_ddr4_s_axi_arcache(4'b0011), .c0_ddr4_s_axi_arprot(3'b000),
        .c0_ddr4_s_axi_arqos(4'b0000),
        .c0_ddr4_s_axi_arvalid(phase == READ_ADDR),
        .c0_ddr4_s_axi_arready(arready),
        .c0_ddr4_s_axi_rready(1'b1), .c0_ddr4_s_axi_rid(),
        .c0_ddr4_s_axi_rdata(rdata), .c0_ddr4_s_axi_rresp(rresp),
        .c0_ddr4_s_axi_rlast(rlast), .c0_ddr4_s_axi_rvalid(rvalid)
    );

    always @(posedge ui_clk) begin
        if (ui_rst || !cal_done) begin
            phase <= WAIT_CAL;
            index <= 0;
            aw_done <= 0;
            w_done <= 0;
            error_code <= 0;
            error_addr <= 0;
            error_xor <= 0;
            verified <= 0;
        end else begin
            case (phase)
                WAIT_CAL: phase <= WRITE;
                WRITE: begin
                    if (awready && !aw_done) aw_done <= 1;
                    if (wready && !w_done) w_done <= 1;
                    if ((aw_done || awready) && (w_done || wready))
                        phase <= WRITE_RESP;
                end
                WRITE_RESP: if (bvalid) begin
                    if (bresp != 0) begin
                        error_code <= 1;
                        error_addr <= test_addr;
                        phase <= FAIL;
                    end else if (index == 13'h1fff) begin
                        index <= 0;
                        phase <= READ_ADDR;
                    end else begin
                        index <= index + 1'b1;
                        aw_done <= 0;
                        w_done <= 0;
                        phase <= WRITE;
                    end
                end
                READ_ADDR: if (arready) phase <= READ_DATA;
                READ_DATA: if (rvalid) begin
                    if (rresp != 0 || !rlast || rdata != expected) begin
                        error_code <= (rresp != 0) ? 2 : (!rlast ? 3 : 4);
                        error_addr <= test_addr;
                        error_xor <= rdata[31:0] ^ expected[31:0];
                        phase <= FAIL;
                    end else begin
                        verified <= verified + 1'b1;
                        if (index == 13'h1fff) phase <= PASS;
                        else begin
                            index <= index + 1'b1;
                            phase <= READ_ADDR;
                        end
                    end
                end
                default: phase <= phase;
            endcase
        end
    end

    tqtt_ddr4_vio u_vio (
        .clk(ui_clk),
        .probe_in0({20'b0, error_code, 2'b0, phase, cal_done, phase == PASS}),
        .probe_in1({18'b0, verified}),
        .probe_in2({2'b0, error_addr}),
        .probe_in3(error_xor)
    );
endmodule
