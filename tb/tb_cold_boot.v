// tb_cold_boot.v — Verilator-only cold-boot wrapper.
//
// Stages exercised here:
//   SD byte model -> boot_fsm -> axi_narrow_to_wide -> axi_xbar -> ddr_ctrl
//   m68k_core -> if_to_axi / axi_narrow_to_wide -> axi_xbar -> DDR / IO / VRAM
//
// This is a focused simulation wrapper, not a board top: no PLLs, no HDMI,
// no host-debug master.  It keeps only the cold-boot fabric needed for the
// ROM copy, first fetch/execute, and directed peripheral pokes.

`default_nettype none
`include "axi_defs.vh"

module tb_axi_fail_slave #(
    parameter ID_WIDTH   = 6,
    parameter DATA_WIDTH = 128,
    parameter STRB_WIDTH = DATA_WIDTH/8
) (
    input  wire                   clk,
    input  wire                   rst,
    input  wire [ID_WIDTH-1:0]    s_awid,
    input  wire [31:0]            s_awaddr,
    input  wire [7:0]             s_awlen,
    input  wire [2:0]             s_awsize,
    input  wire [1:0]             s_awburst,
    input  wire                   s_awvalid,
    output wire                   s_awready,
    input  wire [DATA_WIDTH-1:0]  s_wdata,
    input  wire [STRB_WIDTH-1:0]  s_wstrb,
    input  wire                   s_wlast,
    input  wire                   s_wvalid,
    output wire                   s_wready,
    output reg  [ID_WIDTH-1:0]    s_bid,
    output reg  [1:0]             s_bresp,
    output reg                    s_bvalid,
    input  wire                   s_bready,
    input  wire [ID_WIDTH-1:0]    s_arid,
    input  wire [31:0]            s_araddr,
    input  wire [7:0]             s_arlen,
    input  wire [2:0]             s_arsize,
    input  wire [1:0]             s_arburst,
    input  wire                   s_arvalid,
    output wire                   s_arready,
    output reg  [ID_WIDTH-1:0]    s_rid,
    output reg  [DATA_WIDTH-1:0]  s_rdata,
    output reg  [1:0]             s_rresp,
    output reg                    s_rlast,
    output reg                    s_rvalid,
    input  wire                   s_rready
);
    localparam [1:0] AXI_RESP_DECERR = 2'b11;

    reg aw_pending;
    reg [ID_WIDTH-1:0] awid_q;

    assign s_awready = !aw_pending;
    assign s_wready  = aw_pending && !s_bvalid;
    assign s_arready = !s_rvalid;

    wire _unused_inputs = &{1'b0, s_awaddr, s_awlen, s_awsize, s_awburst,
                            s_wdata[0], s_wstrb[0], s_wlast,
                            s_araddr, s_arlen, s_arsize, s_arburst, 1'b0};

    always @(posedge clk) begin
        if (rst) begin
            aw_pending <= 1'b0;
            awid_q     <= {ID_WIDTH{1'b0}};
            s_bid      <= {ID_WIDTH{1'b0}};
            s_bresp    <= AXI_RESP_DECERR;
            s_bvalid   <= 1'b0;
            s_rid      <= {ID_WIDTH{1'b0}};
            s_rdata    <= {DATA_WIDTH{1'b0}};
            s_rresp    <= AXI_RESP_DECERR;
            s_rlast    <= 1'b1;
            s_rvalid   <= 1'b0;
        end else begin
            if (!aw_pending && s_awvalid && s_awready) begin
                aw_pending <= 1'b1;
                awid_q     <= s_awid;
            end
            if (!s_bvalid && aw_pending && s_wvalid && s_wready) begin
                s_bid    <= awid_q;
                s_bresp  <= AXI_RESP_DECERR;
                s_bvalid <= 1'b1;
                aw_pending <= 1'b0;
            end else if (s_bvalid && s_bready) begin
                s_bvalid <= 1'b0;
            end

            if (!s_rvalid && s_arvalid && s_arready) begin
                s_rid    <= s_arid;
                s_rdata  <= {DATA_WIDTH{1'b0}};
                s_rresp  <= AXI_RESP_DECERR;
                s_rlast  <= 1'b1;
                s_rvalid <= 1'b1;
            end else if (s_rvalid && s_rready) begin
                s_rvalid <= 1'b0;
            end
        end
    end
endmodule

module tb_axil_fail_slave (
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] s_awaddr,
    input  wire        s_awvalid,
    output wire        s_awready,
    input  wire [31:0] s_wdata,
    input  wire [3:0]  s_wstrb,
    input  wire        s_wvalid,
    output wire        s_wready,
    output reg  [1:0]  s_bresp,
    output reg         s_bvalid,
    input  wire        s_bready,
    input  wire [31:0] s_araddr,
    input  wire        s_arvalid,
    output wire        s_arready,
    output reg  [31:0] s_rdata,
    output reg  [1:0]  s_rresp,
    output reg         s_rvalid,
    input  wire        s_rready
);
    localparam [1:0] AXI_RESP_DECERR = 2'b11;

    reg aw_pending;

    assign s_awready = !aw_pending;
    assign s_wready  = aw_pending && !s_bvalid;
    assign s_arready = !s_rvalid;

    wire _unused_inputs = &{1'b0, s_awaddr, s_wdata, s_wstrb, s_araddr, 1'b0};

    always @(posedge clk) begin
        if (rst) begin
            aw_pending <= 1'b0;
            s_bresp    <= AXI_RESP_DECERR;
            s_bvalid   <= 1'b0;
            s_rdata    <= 32'd0;
            s_rresp    <= AXI_RESP_DECERR;
            s_rvalid   <= 1'b0;
        end else begin
            if (!aw_pending && s_awvalid && s_awready)
                aw_pending <= 1'b1;
            if (!s_bvalid && aw_pending && s_wvalid && s_wready) begin
                s_bresp    <= AXI_RESP_DECERR;
                s_bvalid   <= 1'b1;
                aw_pending <= 1'b0;
            end else if (s_bvalid && s_bready) begin
                s_bvalid <= 1'b0;
            end

            if (!s_rvalid && s_arvalid && s_arready) begin
                s_rdata  <= 32'd0;
                s_rresp  <= AXI_RESP_DECERR;
                s_rvalid <= 1'b1;
            end else if (s_rvalid && s_rready) begin
                s_rvalid <= 1'b0;
            end
        end
    end
endmodule

module tb_cold_boot #(
    parameter [31:0] NUM_SECTORS      = 32'd2048,
    parameter [31:0] ROM_BASE_ADDR    = 32'h4000_0000,
    parameter [31:0] RESET_PC         = 32'h4000_002A,
    parameter integer VRAM_WIDTH_PX   = 1024,
    parameter integer VRAM_HEIGHT_PX  = 768,
    parameter integer VRAM_BPP        = 8,
    // vram.v's streaming read-port DATA width (RD_DATA_W).  vram.v requires it
    // to be exactly 4*BPP: the port is pixel-ADDRESSED but returns a 4-LANE
    // group.  Derived (not hardcoded 32) so overriding VRAM_BPP can't silently
    // mis-size the port.  See vram_peek_data below.
    parameter integer VRAM_RD_DATA_W  = 4 * VRAM_BPP,
    // u_vram below does not override VRAM_BYTES, so its PX_ADDR_W is
    // derived from the full 2 MiB VRAM_BYTES aperture (2,097,152 pixels
    // at BPP=8 => 21 bits), not from 1024x768=786,432 pixels (which
    // alone would only need 20 bits) — see vram.v's PX_ADDR_SPAN
    // comment.  Must match u_vram's actual PX_ADDR_W exactly or the
    // vram_peek_addr <-> rd_addr connection width-mismatches.
    parameter integer VRAM_PX_ADDR_W  = 21,
    parameter integer PHI2_DIV        = 8,
    parameter integer VIA_PHI2_HZ     = 783360,
    // When 1 the CPU performs a real 68k cold-boot vector-0 fetch (SSP
    // from addr 0, PC from addr 4) instead of starting at RESET_PC.  The
    // overlay aliases addr 0..ROM_SIZE onto ROM_BASE+0..ROM_SIZE at cold
    // start so the vec-0 read hits the ROM image boot_fsm loaded into
    // DDR.  This mirrors the live fpga_top.v cold-boot path and is the
    // integration check for the HW first-light vec-0 regression.  Kept
    // off by default so existing scenarios using boot_pc_override keep
    // working as before.
    parameter        FETCH_RESET_VECTORS = 1'b0
) (
    input  wire                       clk,
    input  wire                       rst,

    output wire                       spi_cmd_valid,
    input  wire                       spi_cmd_ready,
    output wire [7:0]                 spi_cmd_data,
    input  wire                       spi_rsp_valid,
    input  wire [7:0]                 spi_rsp_data,
    output wire                       spi_cs_n,
    output wire                       spi_fast_mode,
    output wire                       spi_hs_mode,

    input  wire                       boot_pc_override_en,
    input  wire [31:0]                boot_pc_override_val,
    input  wire                       dbg_pc_load_en,
    input  wire [31:0]                dbg_pc_load_val,

    output wire                       rom_loading,
    output wire                       rom_loaded,
    output wire                       boot_error,
    output wire [2:0]                 dbg_err_cause,
    output wire [5:0]                 dbg_st,
    output wire [15:0]                dbg_sector,
    output wire                       ddr_cal_done,
    output wire                       cpu_rst,

    output wire [31:0]                if_addr,
    output wire                       if_req,
    output wire [127:0]               if_rdata,
    output wire                       if_rvalid,
    output wire                       if_fault,

    output wire [31:0]                dbg_pc,
    output wire [31:0]                dbg_committed,
    output wire [31:0]                dbg_macros,
    output wire [31:0]                dbg_last_pc,
    output wire [31:0]                dbg_boundary_seq,
    output wire [1:0]                 dbg_boundary_kind,
    output wire [31:0]                dbg_boundary_pc,
    output wire [31:0]                dbg_boundary_next_pc,
    output wire [7:0]                 dbg_boundary_exc_vec,
    output wire [31:0]                dbg_boundary_fault_pc,
    output wire [31:0]                dbg_boundary_fault_addr,
    output wire [4:0]                 dbg_boundary_keep_tag,

    output reg                        sentinel_valid,
    output reg  [31:0]                sentinel_value,

    output wire                       via1_overlay_bit,
    output wire                       overlay_active,
    output wire [2:0]                 cpu_ipl_ext,
    output wire                       via1_wr_tap,
    output wire [3:0]                 via1_addr_tap,
    output wire [7:0]                 via1_wdata_tap,
    output wire [31:0]                dafb_fb_base_px,
    output wire [31:0]                dafb_fb_stride_px,
    output wire [31:0]                dafb_fb_bpp_reg,

    input  wire [VRAM_PX_ADDR_W-1:0]  vram_peek_addr,
    input  wire                       vram_peek_en,
    // vram.v's streaming read port returns a 4-LANE group starting at
    // vram_peek_addr, the pixel AT that address occupying the TOP BPP bits
    // (bit-identical to the old BPP-wide rd_data) and the pixels at +1/+2/+3
    // below it (valid only for a 4-lane-aligned address).
    output wire [VRAM_RD_DATA_W-1:0]  vram_peek_data,
    output wire                       vram_peek_valid
);
    localparam [1:0] AXI_RESP_OKAY = 2'b00;

    reg [7:0] phi2_div_q;
    always @(posedge clk) begin
        if (rst)
            phi2_div_q <= 8'd0;
        else if (phi2_div_q == PHI2_DIV - 1)
            phi2_div_q <= 8'd0;
        else
            phi2_div_q <= phi2_div_q + 8'd1;
    end
    wire phi2_tick = (phi2_div_q == 8'd0);

    wire [3:0]  bf_awid;
    wire [31:0] bf_awaddr;
    wire [7:0]  bf_awlen;
    wire [2:0]  bf_awsize;
    wire [1:0]  bf_awburst;
    wire        bf_awvalid;
    wire        bf_awready;
    wire [31:0] bf_wdata;
    wire [3:0]  bf_wstrb;
    wire        bf_wlast;
    wire        bf_wvalid;
    wire        bf_wready;
    wire [3:0]  bf_bid;
    wire [1:0]  bf_bresp;
    wire        bf_bvalid;
    wire        bf_bready;

    wire boot_rst = rst | !ddr_cal_done;
    wire base_cpu_rst = rst | !ddr_cal_done | !rom_loaded;
    assign cpu_rst = base_cpu_rst;

    boot_fsm #(
        .NUM_SECTORS  (NUM_SECTORS),
        .ROM_BASE_ADDR(ROM_BASE_ADDR),
        .AXI_ID       (4'd2)
    ) u_boot_fsm (
        // Cold-boot harness: always run the pre-zero pass.
        .zero_en       (1'b1),
        .clk           (clk),
        .rst           (boot_rst),
        .spi_cmd_valid (spi_cmd_valid),
        .spi_cmd_ready (spi_cmd_ready),
        .spi_cmd_data  (spi_cmd_data),
        .spi_rsp_valid (spi_rsp_valid),
        .spi_rsp_data  (spi_rsp_data),
        .spi_cs_n      (spi_cs_n),
        .spi_fast_mode (spi_fast_mode),
        .spi_hs_mode   (spi_hs_mode),
        .m_axi_awid    (bf_awid),
        .m_axi_awaddr  (bf_awaddr),
        .m_axi_awlen   (bf_awlen),
        .m_axi_awsize  (bf_awsize),
        .m_axi_awburst (bf_awburst),
        .m_axi_awvalid (bf_awvalid),
        .m_axi_awready (bf_awready),
        .m_axi_wdata   (bf_wdata),
        .m_axi_wstrb   (bf_wstrb),
        .m_axi_wlast   (bf_wlast),
        .m_axi_wvalid  (bf_wvalid),
        .m_axi_wready  (bf_wready),
        .m_axi_bid     (bf_bid),
        .m_axi_bresp   (bf_bresp),
        .m_axi_bvalid  (bf_bvalid),
        .m_axi_bready  (bf_bready),
        .rom_loading   (rom_loading),
        .rom_loaded    (rom_loaded),
        .error         (boot_error),
        .dbg_st        (dbg_st),
        .dbg_cur_cmd   (),
        .dbg_last_r1   (),
        .dbg_last_rx   (),
        .dbg_acmd41_try(),
        .dbg_rsp_count (),
        .dbg_sector    (dbg_sector),
        .dbg_is_sdhc   (),
        .dbg_ocr0      (),
        .dbg_err_cause (dbg_err_cause)
    );

    wire [3:0]   boot_w_awid;
    wire [31:0]  boot_w_awaddr;
    wire [7:0]   boot_w_awlen;
    wire [2:0]   boot_w_awsize;
    wire [1:0]   boot_w_awburst;
    wire         boot_w_awvalid;
    wire         boot_w_awready;
    wire [127:0] boot_w_wdata;
    wire [15:0]  boot_w_wstrb;
    wire         boot_w_wlast;
    wire         boot_w_wvalid;
    wire         boot_w_wready;
    wire [3:0]   boot_w_bid;
    wire [1:0]   boot_w_bresp;
    wire         boot_w_bvalid;
    wire         boot_w_bready;

    axi_narrow_to_wide #(
        .ID_WIDTH(4),
        .ID_TAG  (4'd2)
    ) u_boot_n2w (
        .clk        (clk),
        .rst        (boot_rst),
        .n_awaddr   (bf_awaddr),
        .n_awprot   (3'b000),
        .n_awvalid  (bf_awvalid),
        .n_awready  (bf_awready),
        .n_wdata    (bf_wdata),
        .n_wstrb    (bf_wstrb),
        .n_wlast    (bf_wlast),
        .n_wvalid   (bf_wvalid),
        .n_wready   (bf_wready),
        .n_bresp    (bf_bresp),
        .n_bvalid   (bf_bvalid),
        .n_bready   (bf_bready),
        .n_araddr   (32'd0),
        .n_arprot   (3'b000),
        .n_arvalid  (1'b0),
        .n_arready  (),
        .n_rdata    (),
        .n_rresp    (),
        .n_rlast    (),
        .n_rvalid   (),
        .n_rready   (1'b0),
        .w_awid     (boot_w_awid),
        .w_awaddr   (boot_w_awaddr),
        .w_awlen    (boot_w_awlen),
        .w_awsize   (boot_w_awsize),
        .w_awburst  (boot_w_awburst),
        .w_awvalid  (boot_w_awvalid),
        .w_awready  (boot_w_awready),
        .w_wdata    (boot_w_wdata),
        .w_wstrb    (boot_w_wstrb),
        .w_wlast    (boot_w_wlast),
        .w_wvalid   (boot_w_wvalid),
        .w_wready   (boot_w_wready),
        .w_bid      (boot_w_bid),
        .w_bresp    (boot_w_bresp),
        .w_bvalid   (boot_w_bvalid),
        .w_bready   (boot_w_bready),
        .w_arid     (),
        .w_araddr   (),
        .w_arlen    (),
        .w_arsize   (),
        .w_arburst  (),
        .w_arvalid  (),
        .w_arready  (1'b0),
        .w_rid      (4'd0),
        .w_rdata    (128'd0),
        .w_rresp    (2'b00),
        .w_rlast    (1'b0),
        .w_rvalid   (1'b0),
        .w_rready   ()
    );
    assign bf_bid = boot_w_bid;

    reg base_cpu_rst_q;
    always @(posedge clk) begin
        if (rst)
            base_cpu_rst_q <= 1'b1;
        else
            base_cpu_rst_q <= base_cpu_rst;
    end
    wire auto_pc_load_en =
        boot_pc_override_en && base_cpu_rst_q && !base_cpu_rst;
    wire cpu_dbg_pc_load_en = dbg_pc_load_en | auto_pc_load_en;
    wire [31:0] cpu_dbg_pc_load_val =
        dbg_pc_load_en ? dbg_pc_load_val : boot_pc_override_val;

    wire [31:0] raw_daxi_awaddr;
    wire [2:0]  raw_daxi_awprot;
    wire        raw_daxi_awvalid;
    wire        raw_daxi_awready;
    wire [31:0] raw_daxi_wdata;
    wire [3:0]  raw_daxi_wstrb;
    wire        raw_daxi_wlast;
    wire        raw_daxi_wvalid;
    wire        raw_daxi_wready;
    wire [1:0]  raw_daxi_bresp;
    wire        raw_daxi_bvalid;
    wire        raw_daxi_bready;
    wire [31:0] raw_daxi_araddr;
    wire [7:0]  raw_daxi_arlen;
    wire [2:0]  raw_daxi_arsize;
    wire [1:0]  raw_daxi_arburst;
    wire [2:0]  raw_daxi_arprot;
    wire        raw_daxi_arvalid;
    wire        raw_daxi_arready;
    wire [31:0] raw_daxi_rdata;
    wire [1:0]  raw_daxi_rresp;
    wire        raw_daxi_rlast;
    wire        raw_daxi_rvalid;
    wire        raw_daxi_rready;

    m68k_core #(
        .RESET_PC(RESET_PC),
        .FETCH_RESET_VECTORS(FETCH_RESET_VECTORS)
    ) u_cpu (
        .clk                  (clk),
        .rst                  (cpu_rst),
        .if_addr              (if_addr),
        .if_req               (if_req),
        .if_rdata             (if_rdata),
        .if_rvalid            (if_rvalid),
        .if_fault             (if_fault),
        .daxi_awaddr          (raw_daxi_awaddr),
        .daxi_awprot          (raw_daxi_awprot),
        .daxi_awvalid         (raw_daxi_awvalid),
        .daxi_awready         (raw_daxi_awready),
        .daxi_wdata           (raw_daxi_wdata),
        .daxi_wstrb           (raw_daxi_wstrb),
        .daxi_wlast           (raw_daxi_wlast),
        .daxi_wvalid          (raw_daxi_wvalid),
        .daxi_wready          (raw_daxi_wready),
        .daxi_bresp           (raw_daxi_bresp),
        .daxi_bvalid          (raw_daxi_bvalid),
        .daxi_bready          (raw_daxi_bready),
        .daxi_araddr          (raw_daxi_araddr),
        .daxi_arlen           (raw_daxi_arlen),
        .daxi_arsize          (raw_daxi_arsize),
        .daxi_arburst         (raw_daxi_arburst),
        .daxi_arprot          (raw_daxi_arprot),
        .daxi_arvalid         (raw_daxi_arvalid),
        .daxi_arready         (raw_daxi_arready),
        .daxi_rdata           (raw_daxi_rdata),
        .daxi_rresp           (raw_daxi_rresp),
        .daxi_rlast           (raw_daxi_rlast),
        .daxi_rvalid          (raw_daxi_rvalid),
        .daxi_rready          (raw_daxi_rready),
        .cpu_ipl_ext          (cpu_ipl_ext),
        .dbg_pc               (dbg_pc),
        .dbg_ccr              (),
        .dbg_committed        (dbg_committed),
        .dbg_macros           (dbg_macros),
        .dbg_last_pc          (dbg_last_pc),
        .dbg_boundary_seq     (dbg_boundary_seq),
        .dbg_boundary_kind    (dbg_boundary_kind),
        .dbg_boundary_pc      (dbg_boundary_pc),
        .dbg_boundary_next_pc (dbg_boundary_next_pc),
        .dbg_boundary_exc_vec (dbg_boundary_exc_vec),
        .dbg_boundary_fault_pc(dbg_boundary_fault_pc),
        .dbg_boundary_fault_addr(dbg_boundary_fault_addr),
        .dbg_boundary_keep_tag(dbg_boundary_keep_tag),
        .dbg_dcache_flush_req (1'b0),
        .dbg_dcache_flush_done(),
        .dbg_dcache_probe_set (5'd0),
        .dbg_dcache_probe_way (2'd0),
        .dbg_dcache_probe_word(3'd0),
        .dbg_dcache_probe_tag (),
        .dbg_dcache_probe_valid(),
        .dbg_dcache_probe_dirty(),
        .dbg_dcache_probe_data(),
        .dbg_dcache_op_req    (1'b0),
        .dbg_dcache_op_kind   (1'b0),
        .dbg_dcache_op_done   (),
        .dbg_icache_op_req    (1'b0),
        .dbg_icache_op_done   (),
        .dbg_precise_stop_req (1'b0),
        .dbg_precise_stop_keep_tag(5'd0),
        .dbg_break_pc_array_in(128'd0),
        .dbg_break_pc_enable_array_in(4'd0),
        .dbg_break_pc_skip_once_array_in(4'd0),
        .dbg_step_macro_arm_in(1'b0),
        .dbg_break_pc_skip_consume_array_out(),
        .dbg_break_uop_fire_out(),
        .dbg_break_pc_hit_slot_out(),
        .dbg_break_pc_hit_is_step_out(),
        .dbg_core_halt        (1'b0),
        .dbg_arch_apply_en    (cpu_dbg_pc_load_en),
        .dbg_arch_reg_load_en (1'b0),
        .dbg_arch_reg_load_idx(5'd0),
        .dbg_arch_reg_load_val(32'd0),
        .dbg_arch_ccr_load_en (1'b0),
        .dbg_arch_ccr_load_val(5'd0),
        .dbg_arch_ctrl_load_en(1'b0),
        .dbg_arch_ctrl_load_sel(4'd0),
        .dbg_arch_ctrl_load_val(32'd0),
        .dbg_arch_mmu_load_en (1'b0),
        .dbg_arch_mmu_load_sel(4'd0),
        .dbg_arch_mmu_load_val(32'd0),
        .dbg_arch_pc_load_en  (cpu_dbg_pc_load_en),
        .dbg_arch_pc_load_val (cpu_dbg_pc_load_val)
    );

    wire [31:0] cpu_daxi_awaddr;
    wire [2:0]  cpu_daxi_awprot;
    wire        cpu_daxi_awvalid;
    wire        cpu_daxi_awready;
    wire [31:0] cpu_daxi_wdata;
    wire [3:0]  cpu_daxi_wstrb;
    wire        cpu_daxi_wlast;
    wire        cpu_daxi_wvalid;
    wire        cpu_daxi_wready;
    wire [1:0]  cpu_daxi_bresp;
    wire        cpu_daxi_bvalid;
    wire        cpu_daxi_bready;
    wire [31:0] cpu_daxi_araddr;
    wire [7:0]  cpu_daxi_arlen;
    wire [2:0]  cpu_daxi_arsize;
    wire [1:0]  cpu_daxi_arburst;
    wire [2:0]  cpu_daxi_arprot;
    wire        cpu_daxi_arvalid;
    wire        cpu_daxi_arready;
    wire [31:0] cpu_daxi_rdata;
    wire [1:0]  cpu_daxi_rresp;
    wire        cpu_daxi_rlast;
    wire        cpu_daxi_rvalid;
    wire        cpu_daxi_rready;

    reg  sent_wr_route_valid;
    reg  sent_wr_route_hit;
    reg  sent_bvalid_q;
    wire raw_aw_to_sentinel = (raw_daxi_awaddr == 32'hFFFF_0000);

    assign cpu_daxi_awaddr  = raw_daxi_awaddr;
    assign cpu_daxi_awprot  = raw_daxi_awprot;
    assign cpu_daxi_awvalid = raw_daxi_awvalid && !sent_wr_route_valid &&
                              !raw_aw_to_sentinel;
    assign raw_daxi_awready = !sent_wr_route_valid &&
                              (raw_aw_to_sentinel ? 1'b1 : cpu_daxi_awready);

    assign cpu_daxi_wdata   = raw_daxi_wdata;
    assign cpu_daxi_wstrb   = raw_daxi_wstrb;
    assign cpu_daxi_wlast   = raw_daxi_wlast;
    assign cpu_daxi_wvalid  = raw_daxi_wvalid && sent_wr_route_valid &&
                              !sent_wr_route_hit;
    assign raw_daxi_wready  = sent_wr_route_valid ?
                              (sent_wr_route_hit ? !sent_bvalid_q
                                                 : cpu_daxi_wready)
                              : 1'b0;

    assign cpu_daxi_bready  = raw_daxi_bready && sent_wr_route_valid &&
                              !sent_wr_route_hit;
    assign raw_daxi_bresp   = (sent_wr_route_valid && sent_wr_route_hit) ?
                              AXI_RESP_OKAY : cpu_daxi_bresp;
    assign raw_daxi_bvalid  = (sent_wr_route_valid && sent_wr_route_hit) ?
                              sent_bvalid_q : cpu_daxi_bvalid;

    assign cpu_daxi_araddr  = raw_daxi_araddr;
    assign cpu_daxi_arlen   = raw_daxi_arlen;
    assign cpu_daxi_arsize  = raw_daxi_arsize;
    assign cpu_daxi_arburst = raw_daxi_arburst;
    assign cpu_daxi_arprot  = raw_daxi_arprot;
    assign cpu_daxi_arvalid = raw_daxi_arvalid;
    assign raw_daxi_arready = cpu_daxi_arready;
    assign raw_daxi_rdata   = cpu_daxi_rdata;
    assign raw_daxi_rresp   = cpu_daxi_rresp;
    assign raw_daxi_rlast   = cpu_daxi_rlast;
    assign raw_daxi_rvalid  = cpu_daxi_rvalid;
    assign cpu_daxi_rready  = raw_daxi_rready;

    always @(posedge clk) begin
        if (rst) begin
            sent_wr_route_valid <= 1'b0;
            sent_wr_route_hit   <= 1'b0;
            sent_bvalid_q       <= 1'b0;
            sentinel_valid      <= 1'b0;
            sentinel_value      <= 32'd0;
        end else begin
            sentinel_valid <= 1'b0;

            if (!sent_wr_route_valid && raw_daxi_awvalid && raw_daxi_awready) begin
                sent_wr_route_valid <= 1'b1;
                sent_wr_route_hit   <= raw_aw_to_sentinel;
            end

            if (sent_wr_route_valid && sent_wr_route_hit &&
                !sent_bvalid_q && raw_daxi_wvalid && raw_daxi_wready) begin
                sent_bvalid_q  <= 1'b1;
                sentinel_valid <= 1'b1;
                sentinel_value <= raw_daxi_wdata;
            end else if (sent_bvalid_q && raw_daxi_bready) begin
                sent_bvalid_q <= 1'b0;
            end

            if (sent_wr_route_valid && sent_wr_route_hit &&
                sent_bvalid_q && raw_daxi_bready) begin
                sent_wr_route_valid <= 1'b0;
                sent_wr_route_hit   <= 1'b0;
            end else if (sent_wr_route_valid && !sent_wr_route_hit &&
                         cpu_daxi_bvalid && cpu_daxi_bready) begin
                sent_wr_route_valid <= 1'b0;
                sent_wr_route_hit   <= 1'b0;
            end
        end
    end

    wire [3:0]   cpu_d_awid;
    wire [31:0]  cpu_d_awaddr;
    wire [7:0]   cpu_d_awlen;
    wire [2:0]   cpu_d_awsize;
    wire [1:0]   cpu_d_awburst;
    wire         cpu_d_awvalid;
    wire         cpu_d_awready;
    wire [127:0] cpu_d_wdata;
    wire [15:0]  cpu_d_wstrb;
    wire         cpu_d_wlast;
    wire         cpu_d_wvalid;
    wire         cpu_d_wready;
    wire [3:0]   cpu_d_bid;
    wire [1:0]   cpu_d_bresp;
    wire         cpu_d_bvalid;
    wire         cpu_d_bready;
    wire [3:0]   cpu_d_arid;
    wire [31:0]  cpu_d_araddr;
    wire [7:0]   cpu_d_arlen;
    wire [2:0]   cpu_d_arsize;
    wire [1:0]   cpu_d_arburst;
    wire         cpu_d_arvalid;
    wire         cpu_d_arready;
    wire [3:0]   cpu_d_rid;
    wire [127:0] cpu_d_rdata;
    wire [1:0]   cpu_d_rresp;
    wire         cpu_d_rlast;
    wire         cpu_d_rvalid;
    wire         cpu_d_rready;

    axi_narrow_to_wide #(
        .ID_WIDTH(4),
        .ID_TAG  (4'd0)
    ) u_cpu_n2w (
        .clk        (clk),
        .rst        (cpu_rst),
        .n_awaddr   (cpu_daxi_awaddr),
        .n_awprot   (cpu_daxi_awprot),
        .n_awvalid  (cpu_daxi_awvalid),
        .n_awready  (cpu_daxi_awready),
        .n_wdata    (cpu_daxi_wdata),
        .n_wstrb    (cpu_daxi_wstrb),
        .n_wlast    (cpu_daxi_wlast),
        .n_wvalid   (cpu_daxi_wvalid),
        .n_wready   (cpu_daxi_wready),
        .n_bresp    (cpu_daxi_bresp),
        .n_bvalid   (cpu_daxi_bvalid),
        .n_bready   (cpu_daxi_bready),
        .n_araddr   (cpu_daxi_araddr),
        .n_arprot   (cpu_daxi_arprot),
        .n_arvalid  (cpu_daxi_arvalid),
        .n_arready  (cpu_daxi_arready),
        .n_rdata    (cpu_daxi_rdata),
        .n_rresp    (cpu_daxi_rresp),
        .n_rlast    (cpu_daxi_rlast),
        .n_rvalid   (cpu_daxi_rvalid),
        .n_rready   (cpu_daxi_rready),
        .w_awid     (cpu_d_awid),
        .w_awaddr   (cpu_d_awaddr),
        .w_awlen    (cpu_d_awlen),
        .w_awsize   (cpu_d_awsize),
        .w_awburst  (cpu_d_awburst),
        .w_awvalid  (cpu_d_awvalid),
        .w_awready  (cpu_d_awready),
        .w_wdata    (cpu_d_wdata),
        .w_wstrb    (cpu_d_wstrb),
        .w_wlast    (cpu_d_wlast),
        .w_wvalid   (cpu_d_wvalid),
        .w_wready   (cpu_d_wready),
        .w_bid      (cpu_d_bid),
        .w_bresp    (cpu_d_bresp),
        .w_bvalid   (cpu_d_bvalid),
        .w_bready   (cpu_d_bready),
        .w_arid     (cpu_d_arid),
        .w_araddr   (cpu_d_araddr),
        .w_arlen    (cpu_d_arlen),
        .w_arsize   (cpu_d_arsize),
        .w_arburst  (cpu_d_arburst),
        .w_arvalid  (cpu_d_arvalid),
        .w_arready  (cpu_d_arready),
        .w_rid      (cpu_d_rid),
        .w_rdata    (cpu_d_rdata),
        .w_rresp    (cpu_d_rresp),
        .w_rlast    (cpu_d_rlast),
        .w_rvalid   (cpu_d_rvalid),
        .w_rready   (cpu_d_rready)
    );

    wire [3:0]   ifa_arid;
    wire [31:0]  ifa_araddr;
    wire [7:0]   ifa_arlen;
    wire [2:0]   ifa_arsize;
    wire [1:0]   ifa_arburst;
    wire         ifa_arvalid;
    wire         ifa_arready;
    wire [3:0]   ifa_rid;
    wire [127:0] ifa_rdata;
    wire [1:0]   ifa_rresp;
    wire         ifa_rlast;
    wire         ifa_rvalid;
    wire         ifa_rready;

    if_to_axi #(
        .ID_WIDTH(4),
        .ID_TAG  (4'd1)
    ) u_if_to_axi (
        .clk        (clk),
        .rst        (cpu_rst),
        .if_addr    (if_addr),
        .if_req     (if_req),
        .if_rdata   (if_rdata),
        .if_rvalid  (if_rvalid),
        .if_fault   (if_fault),
        .m_arid     (ifa_arid),
        .m_araddr   (ifa_araddr),
        .m_arlen    (ifa_arlen),
        .m_arsize   (ifa_arsize),
        .m_arburst  (ifa_arburst),
        .m_arvalid  (ifa_arvalid),
        .m_arready  (ifa_arready),
        .m_rid      (ifa_rid),
        .m_rdata    (ifa_rdata),
        .m_rresp    (ifa_rresp),
        .m_rlast    (ifa_rlast),
        .m_rvalid   (ifa_rvalid),
        .m_rready   (ifa_rready)
    );

    assign overlay_active = via1_overlay_bit;

    wire [3:0]   m0_awid;
    wire [31:0]  m0_awaddr;
    wire [7:0]   m0_awlen;
    wire [2:0]   m0_awsize;
    wire [1:0]   m0_awburst;
    wire         m0_awvalid;
    wire         m0_awready;
    wire [127:0] m0_wdata;
    wire [15:0]  m0_wstrb;
    wire         m0_wlast;
    wire         m0_wvalid;
    wire         m0_wready;
    wire [3:0]   m0_bid;
    wire [1:0]   m0_bresp;
    wire         m0_bvalid;
    wire         m0_bready;
    wire [3:0]   m0_arid;
    wire [31:0]  m0_araddr;
    wire [7:0]   m0_arlen;
    wire [2:0]   m0_arsize;
    wire [1:0]   m0_arburst;
    wire         m0_arvalid;
    wire         m0_arready;
    wire [3:0]   m0_rid;
    wire [127:0] m0_rdata;
    wire [1:0]   m0_rresp;
    wire         m0_rlast;
    wire         m0_rvalid;
    wire         m0_rready;

    assign m0_awid    = cpu_d_awid;
    assign m0_awaddr  = cpu_d_awaddr;
    assign m0_awlen   = cpu_d_awlen;
    assign m0_awsize  = cpu_d_awsize;
    assign m0_awburst = cpu_d_awburst;
    assign m0_awvalid = cpu_d_awvalid;
    assign cpu_d_awready = m0_awready;
    assign m0_wdata   = cpu_d_wdata;
    assign m0_wstrb   = cpu_d_wstrb;
    assign m0_wlast   = cpu_d_wlast;
    assign m0_wvalid  = cpu_d_wvalid;
    assign cpu_d_wready = m0_wready;
    assign cpu_d_bid    = m0_bid;
    assign cpu_d_bresp  = m0_bresp;
    assign cpu_d_bvalid = m0_bvalid;
    assign m0_bready    = cpu_rst ? 1'b1 : cpu_d_bready;

    assign m0_arid    = cpu_d_arid;
    assign m0_araddr  = cpu_d_araddr;
    assign m0_arlen   = cpu_d_arlen;
    assign m0_arsize  = cpu_d_arsize;
    assign m0_arburst = cpu_d_arburst;
    assign m0_arvalid = cpu_d_arvalid;
    assign cpu_d_arready = m0_arready;

    assign cpu_d_rid    = m0_rid;
    assign cpu_d_rdata  = m0_rdata;
    assign cpu_d_rresp  = m0_rresp;
    assign cpu_d_rlast  = m0_rlast;
    assign cpu_d_rvalid = m0_rvalid;
    assign m0_rready    = cpu_rst ? 1'b1 : cpu_d_rready;

    wire [5:0]   s0_awid;
    wire [31:0]  s0_awaddr;
    wire [7:0]   s0_awlen;
    wire [2:0]   s0_awsize;
    wire [1:0]   s0_awburst;
    wire         s0_awvalid;
    wire         s0_awready;
    wire [127:0] s0_wdata;
    wire [15:0]  s0_wstrb;
    wire         s0_wlast;
    wire         s0_wvalid;
    wire         s0_wready;
    wire [5:0]   s0_bid;
    wire [1:0]   s0_bresp;
    wire         s0_bvalid;
    wire         s0_bready;
    wire [5:0]   s0_arid;
    wire [31:0]  s0_araddr;
    wire [7:0]   s0_arlen;
    wire [2:0]   s0_arsize;
    wire [1:0]   s0_arburst;
    wire         s0_arvalid;
    wire         s0_arready;
    wire [5:0]   s0_rid;
    wire [127:0] s0_rdata;
    wire [1:0]   s0_rresp;
    wire         s0_rlast;
    wire         s0_rvalid;
    wire         s0_rready;

    wire [5:0]   s1_awid;
    wire [31:0]  s1_awaddr;
    wire [7:0]   s1_awlen;
    wire [2:0]   s1_awsize;
    wire [1:0]   s1_awburst;
    wire         s1_awvalid;
    wire         s1_awready;
    wire [127:0] s1_wdata;
    wire [15:0]  s1_wstrb;
    wire         s1_wlast;
    wire         s1_wvalid;
    wire         s1_wready;
    wire [5:0]   s1_bid;
    wire [1:0]   s1_bresp;
    wire         s1_bvalid;
    wire         s1_bready;
    wire [5:0]   s1_arid;
    wire [31:0]  s1_araddr;
    wire [7:0]   s1_arlen;
    wire [2:0]   s1_arsize;
    wire [1:0]   s1_arburst;
    wire         s1_arvalid;
    wire         s1_arready;
    wire [5:0]   s1_rid;
    wire [127:0] s1_rdata;
    wire [1:0]   s1_rresp;
    wire         s1_rlast;
    wire         s1_rvalid;
    wire         s1_rready;

    wire [5:0]   s2_awid;
    wire [31:0]  s2_awaddr;
    wire [7:0]   s2_awlen;
    wire [2:0]   s2_awsize;
    wire [1:0]   s2_awburst;
    wire         s2_awvalid;
    wire         s2_awready;
    wire [127:0] s2_wdata;
    wire [15:0]  s2_wstrb;
    wire         s2_wlast;
    wire         s2_wvalid;
    wire         s2_wready;
    wire [5:0]   s2_bid;
    wire [1:0]   s2_bresp;
    wire         s2_bvalid;
    wire         s2_bready;
    wire [5:0]   s2_arid;
    wire [31:0]  s2_araddr;
    wire [7:0]   s2_arlen;
    wire [2:0]   s2_arsize;
    wire [1:0]   s2_arburst;
    wire         s2_arvalid;
    wire         s2_arready;
    wire [5:0]   s2_rid;
    wire [127:0] s2_rdata;
    wire [1:0]   s2_rresp;
    wire         s2_rlast;
    wire         s2_rvalid;
    wire         s2_rready;

    wire [5:0]   s3_awid;
    wire [31:0]  s3_awaddr;
    wire [7:0]   s3_awlen;
    wire [2:0]   s3_awsize;
    wire [1:0]   s3_awburst;
    wire         s3_awvalid;
    wire         s3_awready;
    wire [127:0] s3_wdata;
    wire [15:0]  s3_wstrb;
    wire         s3_wlast;
    wire         s3_wvalid;
    wire         s3_wready;
    wire [5:0]   s3_bid;
    wire [1:0]   s3_bresp;
    wire         s3_bvalid;
    wire         s3_bready;
    wire [5:0]   s3_arid;
    wire [31:0]  s3_araddr;
    wire [7:0]   s3_arlen;
    wire [2:0]   s3_arsize;
    wire [1:0]   s3_arburst;
    wire         s3_arvalid;
    wire         s3_arready;
    wire [5:0]   s3_rid;
    wire [127:0] s3_rdata;
    wire [1:0]   s3_rresp;
    wire         s3_rlast;
    wire         s3_rvalid;
    wire         s3_rready;

    wire [5:0]   s4_awid;
    wire [31:0]  s4_awaddr;
    wire [7:0]   s4_awlen;
    wire [2:0]   s4_awsize;
    wire [1:0]   s4_awburst;
    wire         s4_awvalid;
    wire         s4_awready;
    wire [127:0] s4_wdata;
    wire [15:0]  s4_wstrb;
    wire         s4_wlast;
    wire         s4_wvalid;
    wire         s4_wready;
    wire [5:0]   s4_bid;
    wire [1:0]   s4_bresp;
    wire         s4_bvalid;
    wire         s4_bready;
    wire [5:0]   s4_arid;
    wire [31:0]  s4_araddr;
    wire [7:0]   s4_arlen;
    wire [2:0]   s4_arsize;
    wire [1:0]   s4_arburst;
    wire         s4_arvalid;
    wire         s4_arready;
    wire [5:0]   s4_rid;
    wire [127:0] s4_rdata;
    wire [1:0]   s4_rresp;
    wire         s4_rlast;
    wire         s4_rvalid;
    wire         s4_rready;

    axi_xbar #(
        .DATA_WIDTH(128),
        .ID_WIDTH  (4),
        .XID_WIDTH (6)
    ) u_xbar (
        .clk        (clk),
        .rst        (rst),
        .cpu_overlay_active(via1_overlay_bit),
        .cpu_overlay_reset(rst),
        .dbg_ram_window_lg2(6'd22),
        .m0_awid    (m0_awid),
        .m0_awaddr  (m0_awaddr),
        .m0_awlen   (m0_awlen),
        .m0_awsize  (m0_awsize),
        .m0_awburst (m0_awburst),
        .m0_awvalid (m0_awvalid),
        .m0_awready (m0_awready),
        .m0_wdata   (m0_wdata),
        .m0_wstrb   (m0_wstrb),
        .m0_wlast   (m0_wlast),
        .m0_wvalid  (m0_wvalid),
        .m0_wready  (m0_wready),
        .m0_bid     (m0_bid),
        .m0_bresp   (m0_bresp),
        .m0_bvalid  (m0_bvalid),
        .m0_bready  (m0_bready),
        .m0_arid    (m0_arid),
        .m0_araddr  (m0_araddr),
        .m0_arlen   (m0_arlen),
        .m0_arsize  (m0_arsize),
        .m0_arburst (m0_arburst),
        .m0_arvalid (m0_arvalid),
        .m0_arready (m0_arready),
        .m0_rid     (m0_rid),
        .m0_rdata   (m0_rdata),
        .m0_rresp   (m0_rresp),
        .m0_rlast   (m0_rlast),
        .m0_rvalid  (m0_rvalid),
        .m0_rready  (m0_rready),
        .m1_awid    (4'd0),
        .m1_awaddr  (32'd0),
        .m1_awlen   (8'd0),
        .m1_awsize  (3'd0),
        .m1_awburst (2'd0),
        .m1_awvalid (1'b0),
        .m1_awready (),
        .m1_wdata   (128'd0),
        .m1_wstrb   (16'd0),
        .m1_wlast   (1'b0),
        .m1_wvalid  (1'b0),
        .m1_wready  (),
        .m1_bid     (),
        .m1_bresp   (),
        .m1_bvalid  (),
        .m1_bready  (1'b1),
        .m1_arid    (4'd0),
        .m1_araddr  (32'd0),
        .m1_arlen   (8'd0),
        .m1_arsize  (3'd0),
        .m1_arburst (2'd0),
        .m1_arvalid (1'b0),
        .m1_arready (),
        .m1_rid     (),
        .m1_rdata   (),
        .m1_rresp   (),
        .m1_rlast   (),
        .m1_rvalid  (),
        .m1_rready  (1'b1),
        .m2_awid    (boot_w_awid),
        .m2_awaddr  (boot_w_awaddr),
        .m2_awlen   (boot_w_awlen),
        .m2_awsize  (boot_w_awsize),
        .m2_awburst (boot_w_awburst),
        .m2_awvalid (boot_w_awvalid),
        .m2_awready (boot_w_awready),
        .m2_wdata   (boot_w_wdata),
        .m2_wstrb   (boot_w_wstrb),
        .m2_wlast   (boot_w_wlast),
        .m2_wvalid  (boot_w_wvalid),
        .m2_wready  (boot_w_wready),
        .m2_bid     (boot_w_bid),
        .m2_bresp   (boot_w_bresp),
        .m2_bvalid  (boot_w_bvalid),
        .m2_bready  (boot_w_bready),
        .m3_arid    (ifa_arid),
        .m3_araddr  (ifa_araddr),
        .m3_arlen   (ifa_arlen),
        .m3_arsize  (ifa_arsize),
        .m3_arburst (ifa_arburst),
        .m3_arvalid (ifa_arvalid),
        .m3_arready (ifa_arready),
        .m3_rid     (ifa_rid),
        .m3_rdata   (ifa_rdata),
        .m3_rresp   (ifa_rresp),
        .m3_rlast   (ifa_rlast),
        .m3_rvalid  (ifa_rvalid),
        .m3_rready  (cpu_rst ? 1'b1 : ifa_rready),
        .m4_awid    (4'd0),
        .m4_awaddr  (32'd0),
        .m4_awlen   (8'd0),
        .m4_awsize  (3'd0),
        .m4_awburst (2'd0),
        .m4_awvalid (1'b0),
        .m4_awready (),
        .m4_wdata   (128'd0),
        .m4_wstrb   (16'd0),
        .m4_wlast   (1'b0),
        .m4_wvalid  (1'b0),
        .m4_wready  (),
        .m4_bid     (),
        .m4_bresp   (),
        .m4_bvalid  (),
        .m4_bready  (1'b1),
        .m4_arid    (4'd0),
        .m4_araddr  (32'd0),
        .m4_arlen   (8'd0),
        .m4_arsize  (3'd0),
        .m4_arburst (2'd0),
        .m4_arvalid (1'b0),
        .m4_arready (),
        .m4_rid     (),
        .m4_rdata   (),
        .m4_rresp   (),
        .m4_rlast   (),
        .m4_rvalid  (),
        .m4_rready  (1'b1),
        .s0_awid    (s0_awid),
        .s0_awaddr  (s0_awaddr),
        .s0_awlen   (s0_awlen),
        .s0_awsize  (s0_awsize),
        .s0_awburst (s0_awburst),
        .s0_awvalid (s0_awvalid),
        .s0_awready (s0_awready),
        .s0_wdata   (s0_wdata),
        .s0_wstrb   (s0_wstrb),
        .s0_wlast   (s0_wlast),
        .s0_wvalid  (s0_wvalid),
        .s0_wready  (s0_wready),
        .s0_bid     (s0_bid),
        .s0_bresp   (s0_bresp),
        .s0_bvalid  (s0_bvalid),
        .s0_bready  (s0_bready),
        .s0_arid    (s0_arid),
        .s0_araddr  (s0_araddr),
        .s0_arlen   (s0_arlen),
        .s0_arsize  (s0_arsize),
        .s0_arburst (s0_arburst),
        .s0_arvalid (s0_arvalid),
        .s0_arready (s0_arready),
        .s0_rid     (s0_rid),
        .s0_rdata   (s0_rdata),
        .s0_rresp   (s0_rresp),
        .s0_rlast   (s0_rlast),
        .s0_rvalid  (s0_rvalid),
        .s0_rready  (s0_rready),
        .s1_awid    (s1_awid),
        .s1_awaddr  (s1_awaddr),
        .s1_awlen   (s1_awlen),
        .s1_awsize  (s1_awsize),
        .s1_awburst (s1_awburst),
        .s1_awvalid (s1_awvalid),
        .s1_awready (s1_awready),
        .s1_wdata   (s1_wdata),
        .s1_wstrb   (s1_wstrb),
        .s1_wlast   (s1_wlast),
        .s1_wvalid  (s1_wvalid),
        .s1_wready  (s1_wready),
        .s1_bid     (s1_bid),
        .s1_bresp   (s1_bresp),
        .s1_bvalid  (s1_bvalid),
        .s1_bready  (s1_bready),
        .s1_arid    (s1_arid),
        .s1_araddr  (s1_araddr),
        .s1_arlen   (s1_arlen),
        .s1_arsize  (s1_arsize),
        .s1_arburst (s1_arburst),
        .s1_arvalid (s1_arvalid),
        .s1_arready (s1_arready),
        .s1_rid     (s1_rid),
        .s1_rdata   (s1_rdata),
        .s1_rresp   (s1_rresp),
        .s1_rlast   (s1_rlast),
        .s1_rvalid  (s1_rvalid),
        .s1_rready  (s1_rready),
        .s2_awid    (s2_awid),
        .s2_awaddr  (s2_awaddr),
        .s2_awlen   (s2_awlen),
        .s2_awsize  (s2_awsize),
        .s2_awburst (s2_awburst),
        .s2_awvalid (s2_awvalid),
        .s2_awready (s2_awready),
        .s2_wdata   (s2_wdata),
        .s2_wstrb   (s2_wstrb),
        .s2_wlast   (s2_wlast),
        .s2_wvalid  (s2_wvalid),
        .s2_wready  (s2_wready),
        .s2_bid     (s2_bid),
        .s2_bresp   (s2_bresp),
        .s2_bvalid  (s2_bvalid),
        .s2_bready  (s2_bready),
        .s2_arid    (s2_arid),
        .s2_araddr  (s2_araddr),
        .s2_arlen   (s2_arlen),
        .s2_arsize  (s2_arsize),
        .s2_arburst (s2_arburst),
        .s2_arvalid (s2_arvalid),
        .s2_arready (s2_arready),
        .s2_rid     (s2_rid),
        .s2_rdata   (s2_rdata),
        .s2_rresp   (s2_rresp),
        .s2_rlast   (s2_rlast),
        .s2_rvalid  (s2_rvalid),
        .s2_rready  (s2_rready),
        .s3_awid    (s3_awid),
        .s3_awaddr  (s3_awaddr),
        .s3_awlen   (s3_awlen),
        .s3_awsize  (s3_awsize),
        .s3_awburst (s3_awburst),
        .s3_awvalid (s3_awvalid),
        .s3_awready (s3_awready),
        .s3_wdata   (s3_wdata),
        .s3_wstrb   (s3_wstrb),
        .s3_wlast   (s3_wlast),
        .s3_wvalid  (s3_wvalid),
        .s3_wready  (s3_wready),
        .s3_bid     (s3_bid),
        .s3_bresp   (s3_bresp),
        .s3_bvalid  (s3_bvalid),
        .s3_bready  (s3_bready),
        .s3_arid    (s3_arid),
        .s3_araddr  (s3_araddr),
        .s3_arlen   (s3_arlen),
        .s3_arsize  (s3_arsize),
        .s3_arburst (s3_arburst),
        .s3_arvalid (s3_arvalid),
        .s3_arready (s3_arready),
        .s3_rid     (s3_rid),
        .s3_rdata   (s3_rdata),
        .s3_rresp   (s3_rresp),
        .s3_rlast   (s3_rlast),
        .s3_rvalid  (s3_rvalid),
        .s3_rready  (s3_rready),
        .s4_awid    (s4_awid),
        .s4_awaddr  (s4_awaddr),
        .s4_awlen   (s4_awlen),
        .s4_awsize  (s4_awsize),
        .s4_awburst (s4_awburst),
        .s4_awvalid (s4_awvalid),
        .s4_awready (s4_awready),
        .s4_wdata   (s4_wdata),
        .s4_wstrb   (s4_wstrb),
        .s4_wlast   (s4_wlast),
        .s4_wvalid  (s4_wvalid),
        .s4_wready  (s4_wready),
        .s4_bid     (s4_bid),
        .s4_bresp   (s4_bresp),
        .s4_bvalid  (s4_bvalid),
        .s4_bready  (s4_bready),
        .s4_arid    (s4_arid),
        .s4_araddr  (s4_araddr),
        .s4_arlen   (s4_arlen),
        .s4_arsize  (s4_arsize),
        .s4_arburst (s4_arburst),
        .s4_arvalid (s4_arvalid),
        .s4_arready (s4_arready),
        .s4_rid     (s4_rid),
        .s4_rdata   (s4_rdata),
        .s4_rresp   (s4_rresp),
        .s4_rlast   (s4_rlast),
        .s4_rvalid  (s4_rvalid),
        .s4_rready  (s4_rready)
    );

    wire ddr_clk_unused;
    ddr_ctrl #(
        .DATA_WIDTH      (128),
        .XID_WIDTH       (6),
        .BRAM_LOG2_BEATS (18),
        .SIM_CAL_CYCLES  (8)
    ) u_ddr (
        .clk          (clk),
        .rst          (rst),
        .ddr_clk      (ddr_clk_unused),
        .ddr_cal_done (ddr_cal_done),
        .awid         (s0_awid),
        .awaddr       (s0_awaddr),
        .awlen        (s0_awlen),
        .awsize       (s0_awsize),
        .awburst      (s0_awburst),
        .awvalid      (s0_awvalid),
        .awready      (s0_awready),
        .wdata        (s0_wdata),
        .wstrb        (s0_wstrb),
        .wlast        (s0_wlast),
        .wvalid       (s0_wvalid),
        .wready       (s0_wready),
        .bid          (s0_bid),
        .bresp        (s0_bresp),
        .bvalid       (s0_bvalid),
        .bready       (s0_bready),
        .arid         (s0_arid),
        .araddr       (s0_araddr),
        .arlen        (s0_arlen),
        .arsize       (s0_arsize),
        .arburst      (s0_arburst),
        .arvalid      (s0_arvalid),
        .arready      (s0_arready),
        .rid          (s0_rid),
        .rdata        (s0_rdata),
        .rresp        (s0_rresp),
        .rlast        (s0_rlast),
        .rvalid       (s0_rvalid),
        .rready       (s0_rready),
        .dbg_aw_cnt   (),
        .dbg_w_cnt    (),
        .dbg_b_cnt    (),
        .dbg_ar_cnt   (),
        .dbg_r_cnt    ()
    );

    tb_axi_fail_slave #(
        .ID_WIDTH  (6),
        .DATA_WIDTH(128)
    ) u_s2_fail (
        .clk       (clk),
        .rst       (rst),
        .s_awid    (s2_awid),
        .s_awaddr  (s2_awaddr),
        .s_awlen   (s2_awlen),
        .s_awsize  (s2_awsize),
        .s_awburst (s2_awburst),
        .s_awvalid (s2_awvalid),
        .s_awready (s2_awready),
        .s_wdata   (s2_wdata),
        .s_wstrb   (s2_wstrb),
        .s_wlast   (s2_wlast),
        .s_wvalid  (s2_wvalid),
        .s_wready  (s2_wready),
        .s_bid     (s2_bid),
        .s_bresp   (s2_bresp),
        .s_bvalid  (s2_bvalid),
        .s_bready  (s2_bready),
        .s_arid    (s2_arid),
        .s_araddr  (s2_araddr),
        .s_arlen   (s2_arlen),
        .s_arsize  (s2_arsize),
        .s_arburst (s2_arburst),
        .s_arvalid (s2_arvalid),
        .s_arready (s2_arready),
        .s_rid     (s2_rid),
        .s_rdata   (s2_rdata),
        .s_rresp   (s2_rresp),
        .s_rlast   (s2_rlast),
        .s_rvalid  (s2_rvalid),
        .s_rready  (s2_rready)
    );

    vram #(
        .FB_WIDTH_PX (VRAM_WIDTH_PX),
        .FB_HEIGHT_PX(VRAM_HEIGHT_PX),
        // BPP is the pixel packing / read-port address granularity;
        // RD_DATA_W is the (widened) read-port DATA width.
        .BPP         (VRAM_BPP),
        .RD_DATA_W   (VRAM_RD_DATA_W),
        .DATA_WIDTH  (128),
        .ID_WIDTH    (6)
    ) u_vram (
        .clk      (clk),
        .rst      (rst),
        .s_awid   (s3_awid),
        .s_awaddr (s3_awaddr),
        .s_awlen  (s3_awlen),
        .s_awsize (s3_awsize),
        .s_awburst(s3_awburst),
        .s_awvalid(s3_awvalid),
        .s_awready(s3_awready),
        .s_wdata  (s3_wdata),
        .s_wstrb  (s3_wstrb),
        .s_wlast  (s3_wlast),
        .s_wvalid (s3_wvalid),
        .s_wready (s3_wready),
        .s_bid    (s3_bid),
        .s_bresp  (s3_bresp),
        .s_bvalid (s3_bvalid),
        .s_bready (s3_bready),
        .s_arid   (s3_arid),
        .s_araddr (s3_araddr),
        .s_arlen  (s3_arlen),
        .s_arsize (s3_arsize),
        .s_arburst(s3_arburst),
        .s_arvalid(s3_arvalid),
        .s_arready(s3_arready),
        .s_rid    (s3_rid),
        .s_rdata  (s3_rdata),
        .s_rresp  (s3_rresp),
        .s_rlast  (s3_rlast),
        .s_rvalid (s3_rvalid),
        .s_rready (s3_rready),
        .rd_clk   (clk),
        .rd_rst   (rst),
        .rd_addr  (vram_peek_addr),
        .rd_en    (vram_peek_en),
        .rd_data  (vram_peek_data),
        .rd_valid (vram_peek_valid)
    );

    wire [19:0] dbg_awaddr, dbg_araddr, prov_awaddr, prov_araddr;
    wire dbg_awvalid, dbg_awready, dbg_wvalid, dbg_wready, dbg_bvalid, dbg_bready;
    wire dbg_arvalid, dbg_arready, dbg_rvalid, dbg_rready;
    wire prov_awvalid, prov_awready, prov_wvalid, prov_wready, prov_bvalid, prov_bready;
    wire prov_arvalid, prov_arready, prov_rvalid, prov_rready;
    wire [31:0] dbg_wdata, dbg_rdata, prov_wdata, prov_rdata;
    wire [3:0]  dbg_wstrb, prov_wstrb;
    wire [1:0]  dbg_bresp, dbg_rresp, prov_bresp, prov_rresp;
    wire [31:0] dafb_awaddr, dafb_wdata, dafb_araddr, dafb_rdata;
    wire [3:0]  dafb_wstrb;
    wire        dafb_awvalid, dafb_awready, dafb_wvalid, dafb_wready;
    wire [1:0]  dafb_bresp, dafb_rresp;
    wire        dafb_bvalid, dafb_bready, dafb_arvalid, dafb_arready;
    wire        dafb_rvalid, dafb_rready;
    wire [19:0] dafb_axil_awaddr, dafb_axil_araddr;
    wire [31:0] pb_dafb_awaddr, pb_dafb_wdata, pb_dafb_araddr;
    wire [3:0]  pb_dafb_wstrb;
    wire        pb_dafb_awvalid, pb_dafb_wvalid, pb_dafb_bready;
    wire        pb_dafb_arvalid, pb_dafb_rready;
    wire        dafb_clut_we;
    wire [7:0]  dafb_clut_waddr;
    wire [23:0] dafb_clut_wdata;
    wire        dafb_irq_w;

    wire [3:0]  pb_via1_addr;
    wire [7:0]  pb_via1_wdata;
    wire        pb_via1_wr;
    wire        pb_via1_rd;
    wire [7:0]  pb_via1_rdata;
    wire        pb_via1_ack;
    wire [3:0]  pb_via2_addr;
    wire [7:0]  pb_via2_wdata;
    wire        pb_via2_wr;
    wire        pb_via2_rd;
    wire [7:0]  pb_via2_rdata;
    wire        pb_via2_ack;
    wire [2:0]  pb_enet_addr;
    wire [7:0]  pb_enet_wdata;
    wire        pb_enet_wr;
    wire        pb_enet_rd;
    wire [7:0]  pb_enet_rdata;
    wire        pb_enet_ack;
    wire [5:0]  pb_sonic_addr;
    wire [15:0] pb_sonic_wdata;
    wire [1:0]  pb_sonic_wstrb;
    wire        pb_sonic_wr;
    wire        pb_sonic_rd;
    wire [15:0] pb_sonic_rdata;
    wire        pb_sonic_ack;
    wire [7:0]  pb_orwell_addr;
    wire [7:0]  pb_orwell_wdata;
    wire        pb_orwell_wr;
    wire        pb_orwell_rd;
    wire [7:0]  pb_orwell_rdata;
    wire        pb_orwell_ack;
    wire [3:0]  pb_scc_addr;
    wire [7:0]  pb_scc_wdata;
    wire        pb_scc_wr;
    wire        pb_scc_rd;
    wire [7:0]  pb_scc_rdata;
    wire        pb_scc_ack;
    wire [8:0]  pb_scsi_addr;
    wire [7:0]  pb_scsi_wdata;
    wire        pb_scsi_wr;
    wire        pb_scsi_rd;
    wire [7:0]  pb_scsi_rdata;
    wire        pb_scsi_ack;
    wire        pb_scsi_dma16_lo_beat;
    wire        pb_scsi_dma_rd_ready;
    wire        pb_scsi_dma_wr_ready;
    wire [11:0] pb_asc_addr;
    wire [7:0]  pb_asc_wdata;
    wire        pb_asc_wr;
    wire        pb_asc_rd;
    wire [7:0]  pb_asc_rdata;
    wire        pb_asc_ack;
    wire [3:0]  pb_iwm_addr;
    wire [7:0]  pb_iwm_wdata;
    wire        pb_iwm_wr;
    wire        pb_iwm_rd;
    wire [7:0]  pb_iwm_rdata;
    wire        pb_iwm_ack;

    assign via1_wr_tap    = pb_via1_wr;
    assign via1_addr_tap  = pb_via1_addr;
    assign via1_wdata_tap = pb_via1_wdata;

    peripheral_bus #(
        .ID_WIDTH  (6),
        .DATA_WIDTH(128)
    ) u_pbus (
        .clk        (clk),
        .rst        (rst),
        // Peripherals share `rst` with the bus here -- no reset skew, so
        // the peripheral-reset barrier has nothing to guard.
        .periph_rst (1'b0),
        .s_awid     (s1_awid),
        .s_awaddr   (s1_awaddr),
        .s_awlen    (s1_awlen),
        .s_awsize   (s1_awsize),
        .s_awburst  (s1_awburst),
        .s_awvalid  (s1_awvalid),
        .s_awready  (s1_awready),
        .s_wdata    (s1_wdata),
        .s_wstrb    (s1_wstrb),
        .s_wlast    (s1_wlast),
        .s_wvalid   (s1_wvalid),
        .s_wready   (s1_wready),
        .s_bid      (s1_bid),
        .s_bresp    (s1_bresp),
        .s_bvalid   (s1_bvalid),
        .s_bready   (s1_bready),
        .s_arid     (s1_arid),
        .s_araddr   (s1_araddr),
        .s_arlen    (s1_arlen),
        .s_arsize   (s1_arsize),
        .s_arburst  (s1_arburst),
        .s_arvalid  (s1_arvalid),
        .s_arready  (s1_arready),
        .s_rid      (s1_rid),
        .s_rdata    (s1_rdata),
        .s_rresp    (s1_rresp),
        .s_rlast    (s1_rlast),
        .s_rvalid   (s1_rvalid),
        .s_rready   (s1_rready),
        .dbg_awaddr (dbg_awaddr),
        .dbg_awvalid(dbg_awvalid),
        .dbg_awready(dbg_awready),
        .dbg_wdata  (dbg_wdata),
        .dbg_wstrb  (dbg_wstrb),
        .dbg_wvalid (dbg_wvalid),
        .dbg_wready (dbg_wready),
        .dbg_bresp  (dbg_bresp),
        .dbg_bvalid (dbg_bvalid),
        .dbg_bready (dbg_bready),
        .dbg_araddr (dbg_araddr),
        .dbg_arvalid(dbg_arvalid),
        .dbg_arready(dbg_arready),
        .dbg_rdata  (dbg_rdata),
        .dbg_rresp  (dbg_rresp),
        .dbg_rvalid (dbg_rvalid),
        .dbg_rready (dbg_rready),
        .prov_awaddr (prov_awaddr),
        .prov_awvalid(prov_awvalid),
        .prov_awready(prov_awready),
        .prov_wdata  (prov_wdata),
        .prov_wstrb  (prov_wstrb),
        .prov_wvalid (prov_wvalid),
        .prov_wready (prov_wready),
        .prov_bresp  (prov_bresp),
        .prov_bvalid (prov_bvalid),
        .prov_bready (prov_bready),
        .prov_araddr (prov_araddr),
        .prov_arvalid(prov_arvalid),
        .prov_arready(prov_arready),
        .prov_rdata  (prov_rdata),
        .prov_rresp  (prov_rresp),
        .prov_rvalid (prov_rvalid),
        .prov_rready (prov_rready),
        .dafb_awaddr (pb_dafb_awaddr),
        .dafb_awvalid(pb_dafb_awvalid),
        .dafb_awready(1'b1),
        .dafb_wdata  (pb_dafb_wdata),
        .dafb_wstrb  (pb_dafb_wstrb),
        .dafb_wvalid (pb_dafb_wvalid),
        .dafb_wready (1'b1),
        .dafb_bresp  (2'b11),
        .dafb_bvalid (1'b1),
        .dafb_bready (pb_dafb_bready),
        .dafb_araddr (pb_dafb_araddr),
        .dafb_arvalid(pb_dafb_arvalid),
        .dafb_arready(1'b1),
        .dafb_rdata  (32'd0),
        .dafb_rresp  (2'b11),
        .dafb_rvalid (1'b1),
        .dafb_rready (pb_dafb_rready),
        .via1_addr   (pb_via1_addr),
        .via1_wdata  (pb_via1_wdata),
        .via1_wr     (pb_via1_wr),
        .via1_rd     (pb_via1_rd),
        .via1_rdata  (pb_via1_rdata),
        .via1_ack    (pb_via1_ack),
        .via2_addr   (pb_via2_addr),
        .via2_wdata  (pb_via2_wdata),
        .via2_wr     (pb_via2_wr),
        .via2_rd     (pb_via2_rd),
        .via2_rdata  (pb_via2_rdata),
        .via2_ack    (pb_via2_ack),
        .enet_addr   (pb_enet_addr),
        .enet_wdata  (pb_enet_wdata),
        .enet_wr     (pb_enet_wr),
        .enet_rd     (pb_enet_rd),
        .enet_rdata  (pb_enet_rdata),
        .enet_ack    (pb_enet_ack),
        .sonic_addr  (pb_sonic_addr),
        .sonic_wdata (pb_sonic_wdata),
        .sonic_wstrb (pb_sonic_wstrb),
        .sonic_wr    (pb_sonic_wr),
        .sonic_rd    (pb_sonic_rd),
        .sonic_rdata (pb_sonic_rdata),
        .sonic_ack   (pb_sonic_ack),
        .orwell_addr (pb_orwell_addr),
        .orwell_wdata(pb_orwell_wdata),
        .orwell_wr   (pb_orwell_wr),
        .orwell_rd   (pb_orwell_rd),
        .orwell_rdata(pb_orwell_rdata),
        .orwell_ack  (pb_orwell_ack),
        .scc_addr    (pb_scc_addr),
        .scc_wdata   (pb_scc_wdata),
        .scc_wr      (pb_scc_wr),
        .scc_rd      (pb_scc_rd),
        .scc_rdata   (pb_scc_rdata),
        .scc_ack     (pb_scc_ack),
        .scsi_addr   (pb_scsi_addr),
        .scsi_wdata  (pb_scsi_wdata),
        .scsi_wr     (pb_scsi_wr),
        .scsi_rd     (pb_scsi_rd),
        .scsi_rdata  (pb_scsi_rdata),
        .scsi_ack    (pb_scsi_ack),
        .scsi_dma_rd_ready(pb_scsi_dma_rd_ready),
        .scsi_dma_wr_ready(pb_scsi_dma_wr_ready),
        .scsi_dma16_lo_beat(pb_scsi_dma16_lo_beat),
        .asc_addr    (pb_asc_addr),
        .asc_wdata   (pb_asc_wdata),
        .asc_wr      (pb_asc_wr),
        .asc_rd      (pb_asc_rd),
        .asc_rdata   (pb_asc_rdata),
        .asc_ack     (pb_asc_ack),
        .iwm_addr    (pb_iwm_addr),
        .iwm_wdata   (pb_iwm_wdata),
        .iwm_wr      (pb_iwm_wr),
        .iwm_rd      (pb_iwm_rd),
        .iwm_rdata   (pb_iwm_rdata),
        .iwm_ack     (pb_iwm_ack)
    );

    wire unused_pb_dafb_decode = &{1'b0, pb_dafb_awaddr, pb_dafb_wdata,
        pb_dafb_wstrb, pb_dafb_awvalid, pb_dafb_wvalid, pb_dafb_bready,
        pb_dafb_araddr, pb_dafb_arvalid, pb_dafb_rready};

    tb_axil_fail_slave u_dbg_fail (
        .clk      (clk),
        .rst      (rst),
        .s_awaddr ({12'd0, dbg_awaddr}),
        .s_awvalid(dbg_awvalid),
        .s_awready(dbg_awready),
        .s_wdata  (dbg_wdata),
        .s_wstrb  (dbg_wstrb),
        .s_wvalid (dbg_wvalid),
        .s_wready (dbg_wready),
        .s_bresp  (dbg_bresp),
        .s_bvalid (dbg_bvalid),
        .s_bready (dbg_bready),
        .s_araddr ({12'd0, dbg_araddr}),
        .s_arvalid(dbg_arvalid),
        .s_arready(dbg_arready),
        .s_rdata  (dbg_rdata),
        .s_rresp  (dbg_rresp),
        .s_rvalid (dbg_rvalid),
        .s_rready (dbg_rready)
    );

    tb_axil_fail_slave u_prov_fail (
        .clk      (clk),
        .rst      (rst),
        .s_awaddr ({12'd0, prov_awaddr}),
        .s_awvalid(prov_awvalid),
        .s_awready(prov_awready),
        .s_wdata  (prov_wdata),
        .s_wstrb  (prov_wstrb),
        .s_wvalid (prov_wvalid),
        .s_wready (prov_wready),
        .s_bresp  (prov_bresp),
        .s_bvalid (prov_bvalid),
        .s_bready (prov_bready),
        .s_araddr ({12'd0, prov_araddr}),
        .s_arvalid(prov_arvalid),
        .s_arready(prov_arready),
        .s_rdata  (prov_rdata),
        .s_rresp  (prov_rresp),
        .s_rvalid (prov_rvalid),
        .s_rready (prov_rready)
    );

    axi_wide_to_axilite #(.ID_WIDTH(6), .DATA_WIDTH(128)) u_dafb_s4_bridge (
        .clk(clk),
        .rst(rst),
        .s_awid(s4_awid),
        .s_awaddr(s4_awaddr),
        .s_awlen(s4_awlen),
        .s_awsize(s4_awsize),
        .s_awburst(s4_awburst),
        .s_awvalid(s4_awvalid),
        .s_awready(s4_awready),
        .s_wdata(s4_wdata),
        .s_wstrb(s4_wstrb),
        .s_wlast(s4_wlast),
        .s_wvalid(s4_wvalid),
        .s_wready(s4_wready),
        .s_bid(s4_bid),
        .s_bresp(s4_bresp),
        .s_bvalid(s4_bvalid),
        .s_bready(s4_bready),
        .s_arid(s4_arid),
        .s_araddr(s4_araddr),
        .s_arlen(s4_arlen),
        .s_arsize(s4_arsize),
        .s_arburst(s4_arburst),
        .s_arvalid(s4_arvalid),
        .s_arready(s4_arready),
        .s_rid(s4_rid),
        .s_rdata(s4_rdata),
        .s_rresp(s4_rresp),
        .s_rlast(s4_rlast),
        .s_rvalid(s4_rvalid),
        .s_rready(s4_rready),
        .l_awaddr(dafb_axil_awaddr),
        .l_awvalid(dafb_awvalid),
        .l_awready(dafb_awready),
        .l_wdata(dafb_wdata),
        .l_wstrb(dafb_wstrb),
        .l_wvalid(dafb_wvalid),
        .l_wready(dafb_wready),
        .l_bresp(dafb_bresp),
        .l_bvalid(dafb_bvalid),
        .l_bready(dafb_bready),
        .l_araddr(dafb_axil_araddr),
        .l_arvalid(dafb_arvalid),
        .l_arready(dafb_arready),
        .l_rdata(dafb_rdata),
        .l_rresp(dafb_rresp),
        .l_rvalid(dafb_rvalid),
        .l_rready(dafb_rready)
    );

    assign dafb_awaddr = {12'h000, dafb_axil_awaddr};
    assign dafb_araddr = {12'h000, dafb_axil_araddr};

    video u_dafb (
        .clk          (clk),
        .rst          (rst),
        .s_axi_awaddr (dafb_awaddr),
        .s_axi_awvalid(dafb_awvalid),
        .s_axi_awready(dafb_awready),
        .s_axi_wdata  (dafb_wdata),
        .s_axi_wstrb  (dafb_wstrb),
        .s_axi_wvalid (dafb_wvalid),
        .s_axi_wready (dafb_wready),
        .s_axi_bresp  (dafb_bresp),
        .s_axi_bvalid (dafb_bvalid),
        .s_axi_bready (dafb_bready),
        .s_axi_araddr (dafb_araddr),
        .s_axi_arvalid(dafb_arvalid),
        .s_axi_arready(dafb_arready),
        .s_axi_rdata  (dafb_rdata),
        .s_axi_rresp  (dafb_rresp),
        .s_axi_rvalid (dafb_rvalid),
        .s_axi_rready (dafb_rready),
        .fb_base_px   (dafb_fb_base_px),
        .fb_stride_px (dafb_fb_stride_px),
        .fb_bpp_reg   (dafb_fb_bpp_reg),
        .fb_bytes_per_px (),
        .depth_supported (),
        .clut_we      (dafb_clut_we),
        .clut_waddr   (dafb_clut_waddr),
        .clut_wdata   (dafb_clut_wdata),
        .irq          (dafb_irq_w),
        // Vblank cadence out of scope for this cold-boot smoke tb.
        .frame_tick   (1'b0),
        // Monitor sense: pin the historical MONITOR_TYPE default (7'h06).
        .monitor_sense(7'h06)
    );

    wire sonic_irq_w;
    q700_eth_sonic u_eth_sonic (
        .clk       (clk),
        .rst       (rst),
        .enet_cs   (pb_enet_wr | pb_enet_rd),
        .enet_rd   (pb_enet_rd),
        .enet_wr   (pb_enet_wr),
        .enet_addr (pb_enet_addr),
        .enet_wdata(pb_enet_wdata),
        .enet_rdata(pb_enet_rdata),
        .enet_ack  (pb_enet_ack),
        .sonic_cs   (pb_sonic_wr | pb_sonic_rd),
        .sonic_rd   (pb_sonic_rd),
        .sonic_wr   (pb_sonic_wr),
        .sonic_addr (pb_sonic_addr),
        .sonic_wdata(pb_sonic_wdata),
        .sonic_wstrb(pb_sonic_wstrb),
        .sonic_rdata(pb_sonic_rdata),
        .sonic_ack  (pb_sonic_ack),
        .sonic_irq  (sonic_irq_w)
    );

    orwell_stub u_orwell_stub (
        .cs    (pb_orwell_wr | pb_orwell_rd),
        .rd    (pb_orwell_rd),
        .wr    (pb_orwell_wr),
        .addr  (pb_orwell_addr),
        .wdata (pb_orwell_wdata),
        .rdata (pb_orwell_rdata),
        .ack   (pb_orwell_ack)
    );

    wire [7:0] via1_pa_out, via1_pa_mask, via1_pb_out, via1_pb_mask;
    wire [7:0] via1_pb_in;
    wire       via1_adb_rx_ready;
    wire [7:0] via1_adb_tx_byte;
    wire       via1_adb_tx_valid;
    wire       via1_rtc_enb;
    wire       via1_rtc_clk;
    wire       via1_rtc_data_o;
    wire       via1_rtc_data_oe;
    wire       via1_rtc_data_i;
    wire       via1_rtc_cko;
    wire       via1_irq_w;
    wire [7:0] via2_pa_in = {~scsi_irq_w, ~scsi_drq_w, 5'h1F, ~sonic_irq_w};
    wire [7:0] via2_pb_in = 8'hCF;
    wire [7:0] via2_pa_out, via2_pa_mask, via2_pb_out, via2_pb_mask;
    wire       via2_ca2_out, via2_cb1_out, via2_cb2_out;
    wire       via2_irq_w;
    wire       via2_pb7_ca1 = via2_pb_mask[7] ? via2_pb_out[7] : 1'b1;

    assign via1_pb_in = {4'b0, 3'b100, via1_rtc_data_i};

    via1 #(
        .ENABLE_INTERNAL_VBL(1'b0)
    ) u_via1 (
        .clk         (clk),
        .rst         (rst),
        .phi2_tick   (phi2_tick),
        .pb_addr     (pb_via1_addr),
        .pb_wdata    (pb_via1_wdata),
        .pb_wr       (pb_via1_wr),
        .pb_rd       (pb_via1_rd),
        .pb_rdata    (pb_via1_rdata),
        .pb_ack      (pb_via1_ack),
        .pa_in       (8'hC1),
        .pb_in       (via1_pb_in),
        .pa_out      (via1_pa_out),
        .pa_mask     (via1_pa_mask),
        .pb_out      (via1_pb_out),
        .pb_mask     (via1_pb_mask),
        .overlay_bit (via1_overlay_bit),
        .adb_rx_byte (8'h00),
        .adb_rx_valid(1'b0),
        .adb_rx_ready(via1_adb_rx_ready),
        .adb_tx_byte (via1_adb_tx_byte),
        .adb_tx_valid(via1_adb_tx_valid),
        .rtc_enb     (via1_rtc_enb),
        .rtc_clk     (via1_rtc_clk),
        .rtc_data_o  (via1_rtc_data_o),
        .rtc_data_oe (via1_rtc_data_oe),
        .rtc_data_i  (via1_rtc_data_i),
        .rtc_cko     (via1_rtc_cko),
        .vblank_irq_in(via2_pb7_ca1),
        .irq         (via1_irq_w)
    );

    rtc #(
        .SEC_DIV(VIA_PHI2_HZ)
    ) u_rtc (
        .clk        (clk),
        .rst        (rst),
        .phi2_tick  (phi2_tick),
        .rtc_enb    (via1_rtc_enb),
        .rtc_clk    (via1_rtc_clk),
        .rtc_data_o (via1_rtc_data_o),
        .rtc_data_oe(via1_rtc_data_oe),
        // PRAM is battery-backed (survives rst); no zap source here.
        .pram_clear (1'b0),
        .pram_busy (),
        // PRAM snapshot/restore back door (rtl/soc/pram_sd.v) — unused here.
        .pram_ext_addr(8'h00), .pram_ext_we(1'b0),
        .pram_ext_wdata(8'h00), .pram_ext_rdata(),
        .cko        (via1_rtc_cko),
        .rtc_data_i (via1_rtc_data_i)
    );

    via2 u_via2 (
        .clk      (clk),
        .rst      (rst),
        .phi2_tick(phi2_tick),
        .pb_addr  (pb_via2_addr),
        .pb_wdata (pb_via2_wdata),
        .pb_wr    (pb_via2_wr),
        .pb_rd    (pb_via2_rd),
        .pb_rdata (pb_via2_rdata),
        .pb_ack   (pb_via2_ack),
        .pa_in    (via2_pa_in),
        .pa_out   (via2_pa_out),
        .pa_mask  (via2_pa_mask),
        .pb_in    (via2_pb_in),
        .pb_out   (via2_pb_out),
        .pb_mask  (via2_pb_mask),
        .ca1_in   (1'b1),
        .ca2_in   (1'b1),
        // Q700 board glue routes ASC/EASC and SCSI interrupt lines
        // through VIA2's active-low control pins.
        .cb1_in   (~asc_irq_w),
        .cb2_in   (~scsi_irq_w),
        .ca2_out  (via2_ca2_out),
        .cb1_out  (via2_cb1_out),
        .cb2_out  (via2_cb2_out),
        .irq      (via2_irq_w)
    );

    wire scc_irq_w;
    scc u_scc (
        .clk     (clk),
        .rst     (rst),
        .pb_addr (pb_scc_addr),
        .pb_wdata(pb_scc_wdata),
        .pb_wr   (pb_scc_wr),
        .pb_rd   (pb_scc_rd),
        .pb_rdata(pb_scc_rdata),
        .pb_ack  (pb_scc_ack),
        .rx_a_valid(1'b0),
        .rx_a_data (8'h00),
        .rx_b_valid(1'b0),
        .rx_b_data (8'h00),
        .cts_a_n   (1'b1),
        .dcd_a_n   (1'b1),
        .sync_a_n  (1'b1),
        .cts_b_n   (1'b1),
        .dcd_b_n   (1'b1),
        .sync_b_n  (1'b1),
        .tx_a_valid(),
        .tx_a_data (),
        .tx_b_valid(),
        .tx_b_data (),
        .rts_a_n   (),
        .dtr_a_n   (),
        .rts_b_n   (),
        .dtr_b_n   (),
        .irq     (scc_irq_w)
    );

    wire scsi_irq_w;
    wire scsi_drq_w;
    wire [2:0]  scsi_sd_cmd_type_w;
    wire [31:0] scsi_sd_lba_w;
    wire [15:0] scsi_sd_block_count_w;
    wire        scsi_sd_go_w;
    wire        scsi_sd_rd_ready_w;
    wire        scsi_sd_wr_valid_w;
    wire [7:0]  scsi_sd_wr_data_w;
    wire        scsi_sd_wr_avail_w;

    scsi #(.TURBOSCSI_C96(1'b1)) u_scsi (
        // Write-protect is a vhdd_ctrl runtime setting (CTRL bit 2), not a
        // property of the SCSI target; these harnesses predate it and test
        // the writable behaviour, so tie it off.
        .wprot(1'b0),
        .clk          (clk),
        .rst          (rst),
        .pb_addr      (pb_scsi_addr),
        .pb_dma16_lo_beat(pb_scsi_dma16_lo_beat),
        .pb_wdata     (pb_scsi_wdata),
        .pb_wr        (pb_scsi_wr),
        .pb_rd        (pb_scsi_rd),
        .pb_rdata     (pb_scsi_rdata),
        .pb_ack       (pb_scsi_ack),
        .dma_rd_ready (pb_scsi_dma_rd_ready),
        .dma_wr_ready (pb_scsi_dma_wr_ready),
        .irq          (scsi_irq_w),
        .drq          (scsi_drq_w),
        .sd_cmd_type  (scsi_sd_cmd_type_w),
        .sd_lba       (scsi_sd_lba_w),
        .sd_block_count(scsi_sd_block_count_w),
        .sd_go        (scsi_sd_go_w),
        .sd_busy      (1'b0),
        .sd_done      (1'b0),
        .sd_error     (1'b0),
        .sd_rd_valid  (1'b0),
        .sd_rd_data   (8'h00),
        .sd_rd_ready  (scsi_sd_rd_ready_w),
        .sd_wr_ready  (1'b0),
        .sd_wr_valid  (scsi_sd_wr_valid_w),
        .sd_wr_data   (scsi_sd_wr_data_w),
        .sd_wr_avail  (scsi_sd_wr_avail_w)
    );

    wire [15:0] asc_audio_sample_out_w;
    wire [15:0] asc_audio_pcm_l_w;
    wire [15:0] asc_audio_pcm_r_w;
    wire        asc_audio_sample_valid_w;
    wire        asc_irq_w;
    asc u_asc (
        .clk               (clk),
        .rst               (rst),
        .phi2_tick         (phi2_tick),
        .pb_addr           (pb_asc_addr),
        .pb_wdata          (pb_asc_wdata),
        .pb_wr             (pb_asc_wr),
        .pb_rd             (pb_asc_rd),
        .pb_rdata          (pb_asc_rdata),
        .pb_ack            (pb_asc_ack),
        .audio_sample_out  (asc_audio_sample_out_w),
        .audio_pcm_l       (asc_audio_pcm_l_w),
        .audio_pcm_r       (asc_audio_pcm_r_w),
        .audio_sample_valid(asc_audio_sample_valid_w),
        .irq               (asc_irq_w)
    );

    wire       iwm_irq_w;
    wire       iwm_dma_req_w;
    wire [7:0] iwm_mode_w;
    iwm_stub u_iwm (
        .clk          (clk),
        .rst          (rst),
        .cs           (pb_iwm_wr | pb_iwm_rd),
        .rd           (pb_iwm_rd),
        .wr           (pb_iwm_wr),
        .reg_sel      (pb_iwm_addr),
        .wdata        (pb_iwm_wdata),
        .drive_present(1'b0),
        .rdata        (pb_iwm_rdata),
        .irq          (iwm_irq_w),
        .dma_req      (iwm_dma_req_w),
        .mode_o       (iwm_mode_w)
    );
    assign pb_iwm_ack = pb_iwm_wr | pb_iwm_rd;

    irq_agg u_irq (
        .clk       (clk),
        .rst       (rst),
        .via1_irq  (via1_irq_w),
        .via2_irq  (via2_irq_w),
        .scsi_irq  (1'b0),
        .scc_irq   (scc_irq_w),
        .snd_irq   (1'b0),
        .rsvd_irq6 (1'b0),
        .nmi_edge  (1'b0),
        .ipl_ack   (1'b0),
        .ipl       (cpu_ipl_ext)
    );
endmodule
