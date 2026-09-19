// tb_dafb_scanout.v -- Verilator wrapper for live DAFB-state -> scanout proof.
//
// Instantiates the DAFB register shim plus a small scanout geometry.  The
// shim's live framebuffer placement outputs drive the scaler inputs, and the
// C++ testbench writes representative Q700 DAFB registers before checking
// the resulting scanout addresses.  The scanout path now includes the real
// fb_reader CDC bridge between scaler and the byte-addressed memory model so
// the test exercises the same request/response split as the production video
// pipeline.

`default_nettype none

module tb_dafb_scanout (
    input  wire        pclk,
    input  wire        vram_clk,
    input  wire        rst,

    // AXI4-lite host interface into the DAFB shim.
    input  wire [31:0] s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,
    output wire [1:0]  s_axi_bresp,
    output wire        s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [31:0] s_axi_araddr,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,
    output wire [31:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,
    output wire        s_axi_rvalid,
    input  wire        s_axi_rready,

    // Live DAFB state, exposed for the host testbench.
    output wire [31:0] fb_base_px,
    output wire [31:0] fb_stride_px,
    output wire [31:0] fb_bpp_reg,

    // Test-only CPU-style byte writes into the scanout memory.
    input  wire        cpu_vram_wr_en,
    input  wire [19:0] cpu_vram_wr_addr,
    input  wire [7:0]  cpu_vram_wr_data,
    input  wire        vram_rsp_hold,

    // Scan timing / scaler observability.
    input  wire [11:0] hcount,
    input  wire [10:0] vcount,
    input  wire        de_in,
    input  wire        hs_in,
    input  wire        vs_in,
    output wire        fb_rd_en,
    output wire [19:0] fb_rd_addr,
    output wire        fb_rd_ready,
    output wire        vram_rd_en,
    output wire [19:0] vram_rd_addr,
    output wire [23:0] scanout_rgb,
    output wire        scanout_de,
    // hs/vs alongside de/rgb -- exposed so the host tb can lock in the
    // sync-to-pixel phase relationship (see tb_dafb_scanout.cpp's sync
    // phase-lock check) and catch a future pipeline retime that shifts
    // hs/vs relative to de/rgb without anyone noticing.
    output wire        scanout_hs,
    output wire        scanout_vs,
    output wire        fb_underflow_sticky
);

    wire [31:0] shim_fb_base_px;
    wire [31:0] shim_fb_stride_px;
    wire [31:0] shim_fb_bpp_reg;
    wire        shim_clut_we;
    wire [7:0]  shim_clut_waddr;
    wire [23:0] shim_clut_wdata;

    video u_video (
        .clk          (pclk),
        .rst          (rst),
        .s_axi_awaddr (s_axi_awaddr),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_awready(s_axi_awready),
        .s_axi_wdata  (s_axi_wdata),
        .s_axi_wstrb  (s_axi_wstrb),
        .s_axi_wvalid (s_axi_wvalid),
        .s_axi_wready (s_axi_wready),
        .s_axi_bresp  (s_axi_bresp),
        .s_axi_bvalid (s_axi_bvalid),
        .s_axi_bready (s_axi_bready),
        .s_axi_araddr (s_axi_araddr),
        .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready),
        .s_axi_rdata  (s_axi_rdata),
        .s_axi_rresp  (s_axi_rresp),
        .s_axi_rvalid (s_axi_rvalid),
        .s_axi_rready (s_axi_rready),
        .fb_base_px   (shim_fb_base_px),
        .fb_stride_px (shim_fb_stride_px),
        .fb_bpp_reg   (shim_fb_bpp_reg),
        .bpp_shift    (),
        .fb_bytes_per_px (),
        .depth_supported (),
        .hres         (),
        .vres         (),
        .clut_we      (shim_clut_we),
        .clut_waddr   (shim_clut_waddr),
        .clut_wdata   (shim_clut_wdata),
        .irq          (),
        .pll_pixel_clock(),
        .scsi0_ctrl_out(),
        .scsi0_drq_in (1'b0),
        // This tb exercises scanout placement/geometry, not vblank
        // cadence — tie frame_tick low.
        .frame_tick   (1'b0),
        // Monitor sense is irrelevant here; pin the historical default
        // (7'h06) that video.v's old MONITOR_TYPE parameter carried.
        .monitor_sense(7'h06)
    );

    assign fb_base_px   = shim_fb_base_px;
    assign fb_stride_px = shim_fb_stride_px;
    assign fb_bpp_reg   = shim_fb_bpp_reg;

    wire [19:0] scaler_fb_base    = shim_fb_base_px[19:0];
    wire [19:0] scaler_fb_stride  = shim_fb_stride_px[19:0];
    wire [23:0] scaler_rgb;
    wire        scaler_de;
    wire        scaler_hs;
    wire        scaler_vs;
    // Streaming read port returns a 4-BYTE GROUP per request (see
    // vram.v / linebuf_scanout.v headers): [31:24] is the byte at the
    // requested address (valid at any alignment), [23:0] are the next three
    // bytes (architecturally valid only for a 4-byte-aligned request).
    wire [31:0] scaler_fb_rd_data;
    wire        scaler_fb_rd_valid;
    wire [31:0] vram_rd_data;
    wire        vram_rd_valid;
    wire        fb_reader_underflow_sticky;
    wire        line_underflow_sticky;

    assign scanout_rgb = scaler_rgb;
    assign scanout_de  = scaler_de;
    assign scanout_hs  = scaler_hs;
    assign scanout_vs  = scaler_vs;
    assign fb_underflow_sticky = fb_reader_underflow_sticky | line_underflow_sticky;

    localparam SCAN_MEM_ADDR_W = 16;
    localparam SCAN_MEM_DEPTH  = (1 << SCAN_MEM_ADDR_W);
    localparam FB_LATENCY      = 33;
    localparam RESP_FIFO_LOG2  = 8;
    localparam RESP_FIFO_DEPTH = (1 << RESP_FIFO_LOG2);
    localparam [RESP_FIFO_LOG2:0] RESP_FIFO_DEPTH_COUNT = RESP_FIFO_DEPTH;

    reg [7:0] scan_mem [0:SCAN_MEM_DEPTH-1];
    reg [31:0] vram_rd_data_r;
    reg       vram_rd_valid_r;
    reg [19:0] resp_addr_fifo [0:RESP_FIFO_DEPTH-1];
    reg [RESP_FIFO_LOG2-1:0] resp_wr_ptr;
    reg [RESP_FIFO_LOG2-1:0] resp_rd_ptr;
    reg [RESP_FIFO_LOG2:0]   resp_count;

    wire resp_pop  = !vram_rsp_hold && (resp_count != {(RESP_FIFO_LOG2+1){1'b0}});
    wire resp_push = vram_rd_en && ((resp_count != RESP_FIFO_DEPTH_COUNT) || resp_pop);

    // Address of the response being popped, in scan_mem units.  The model
    // returns the true bytes at +0/+1/+2/+3 (wrapping inside scan_mem) for
    // EVERY alignment, which is a strict superset of the contract — the
    // hardware only guarantees the lower three bytes when 4-byte aligned.
    wire [SCAN_MEM_ADDR_W-1:0] resp_head_addr =
        resp_addr_fifo[resp_rd_ptr][SCAN_MEM_ADDR_W-1:0];
    wire [SCAN_MEM_ADDR_W-1:0] resp_head_addr1 =
        resp_head_addr + {{(SCAN_MEM_ADDR_W-2){1'b0}}, 2'd1};
    wire [SCAN_MEM_ADDR_W-1:0] resp_head_addr2 =
        resp_head_addr + {{(SCAN_MEM_ADDR_W-2){1'b0}}, 2'd2};
    wire [SCAN_MEM_ADDR_W-1:0] resp_head_addr3 =
        resp_head_addr + {{(SCAN_MEM_ADDR_W-2){1'b0}}, 2'd3};

    integer mi;
    initial begin
        for (mi = 0; mi < SCAN_MEM_DEPTH; mi = mi + 1)
            scan_mem[mi] = 8'd0;
    end

    always @(posedge vram_clk) begin
        if (rst) begin
            vram_rd_data_r  <= 32'd0;
            vram_rd_valid_r <= 1'b0;
            resp_wr_ptr     <= {RESP_FIFO_LOG2{1'b0}};
            resp_rd_ptr     <= {RESP_FIFO_LOG2{1'b0}};
            resp_count      <= {(RESP_FIFO_LOG2+1){1'b0}};
        end else begin
            if (cpu_vram_wr_en)
                scan_mem[cpu_vram_wr_addr[SCAN_MEM_ADDR_W-1:0]] <= cpu_vram_wr_data;

            vram_rd_valid_r <= resp_pop;
            if (resp_pop) begin
                vram_rd_data_r <= { scan_mem[resp_head_addr],
                                    scan_mem[resp_head_addr1],
                                    scan_mem[resp_head_addr2],
                                    scan_mem[resp_head_addr3] };
                resp_rd_ptr <= resp_rd_ptr + {{(RESP_FIFO_LOG2-1){1'b0}}, 1'b1};
            end else begin
                vram_rd_data_r <= 32'd0;
            end

            if (resp_push) begin
                resp_addr_fifo[resp_wr_ptr] <= vram_rd_addr;
                resp_wr_ptr <= resp_wr_ptr + {{(RESP_FIFO_LOG2-1){1'b0}}, 1'b1};
            end

            case ({resp_push, resp_pop})
                2'b10: resp_count <= resp_count + {{RESP_FIFO_LOG2{1'b0}}, 1'b1};
                2'b01: resp_count <= resp_count - {{RESP_FIFO_LOG2{1'b0}}, 1'b1};
                default: resp_count <= resp_count;
            endcase
        end
    end

`ifdef VERILATOR
    always @(posedge vram_clk) begin
        if (!rst && vram_rd_en &&
            resp_count == RESP_FIFO_DEPTH_COUNT && !resp_pop) begin
            $fatal(1, "tb_dafb_scanout VRAM response queue overflow");
        end
    end
`endif

    assign vram_rd_data  = vram_rd_data_r;
    assign vram_rd_valid = vram_rd_valid_r;

    fb_reader #(
        .ADDR_W        (20),
        .DATA_W        (32)
    ) u_fb_reader (
        .pclk       (pclk),
        .resetn     (~rst),
        .vram_clk   (vram_clk),
        .vram_rst   (rst),
        .s_rd_en    (fb_rd_en),
        .s_rd_addr  (fb_rd_addr),
        .s_rd_ready (fb_rd_ready),
        .s_rd_data  (scaler_fb_rd_data),
        .s_rd_valid (scaler_fb_rd_valid),
        .v_rd_en    (vram_rd_en),
        .v_rd_addr  (vram_rd_addr),
        .v_rd_data  (vram_rd_data),
        .v_rd_valid (vram_rd_valid),
        .underflow_sticky(fb_reader_underflow_sticky),
        .req_count  (),
        .rsp_count  (),
        .miss_count ()
    );

    linebuf_scanout #(
        .SRC_W           (16),
        .SRC_H           (12),
        .FB_MAX_PIXELS   (65536),
        .DST_W           (40),
        .DST_H           (28),
        .FETCH_W         (32),
        .ADDR_W          (20),
        .LINE_COUNT_LOG2 (4)
    ) u_scanout (
        .pclk        (pclk),
        .resetn      (~rst),
        .hcount      (hcount),
        .vcount      (vcount),
        .de_in       (de_in),
        .hs_in       (hs_in),
        .vs_in       (vs_in),
        .fb_base_px  (scaler_fb_base),
        .fb_stride_px(scaler_fb_stride),
        .bpp_shift   (3'd0),
        .bytes_per_px(3'd1),   // 8bpp indexed
        // active 32×24 in DST 40×28 = 16×12 source @ 2× scale
        .hres        (12'd16),
        .vres        (12'd12),
        .scale_n     (3'd2),
        // Existing scanout tbs cover the NORMAL video path, so the boot
        // splash is retired here (dafb_live=1).  tb_scanout_frames.v drives
        // it low to cover the splash itself.
        .dafb_live   (1'b1),
        .clut_wclk   (pclk),
        .clut_we     (shim_clut_we),
        .clut_waddr  (shim_clut_waddr),
        .clut_wdata  (shim_clut_wdata),
        .fb_rd_en    (fb_rd_en),
        .fb_rd_addr  (fb_rd_addr),
        .fb_rd_ready (fb_rd_ready),
        .fb_rd_data  (scaler_fb_rd_data),
        .fb_rd_valid (scaler_fb_rd_valid),
        .rgb         (scaler_rgb),
        .de_out      (scaler_de),
        .hs_out      (scaler_hs),
        .vs_out      (scaler_vs),
        .line_underflow_sticky(line_underflow_sticky)
    );

endmodule

`default_nettype wire
