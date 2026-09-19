// dma_ctrl.v — Programmable multi-channel AXI-master DMA engine.
//
// Phase-3.5 foundation for virtual peripherals (NVMe-as-SCSI, 1GbE-as-
// NuBus, etc.).  See docs/dma_ctrl.md for the full spec, register
// map, descriptor layout, integration plan, and IRQ wiring.
//
// Summary:
//   - 4 channels (N_CH parameter; room for 8 at H1).
//   - 64-bit AXI4 master (DATA_WIDTH parameter).
//   - INCR bursts up to 16 beats per segment; longer transfers cracked
//     at 4 KB boundaries (AXI4 rule).
//   - Channel-round-robin arbitration.
//   - Descriptor-chained mode for scatter-gather.
//   - AXI RRESP/BRESP != OKAY → sticky per-channel error, level IRQ.
//   - Long-aligned addresses only (P0).
//
// Register map:
//   0x000  DMA_CTRL    R/W   [0] global_enable, [1] irq_enable
//   0x004  DMA_STATUS  R/O   [3:0] busy [11:8] done [19:16] error [24] irq
//   0x008  DMA_IRQ_CLR W1C   write 1<<n to clear CH[n].done/.error
//
//   Per-channel at 0x100 + ch*0x40 (ch in 0..3):
//   0x00  CH_CTRL      R/W   [0] enable [1] start [2] desc_mode [3] irq_en
//                            [7:4] burst_beats_log2 (0..4 → 1/2/4/8/16)
//   0x04  CH_STATUS    R/O   [0] busy [1] done [2] error [3] desc_active
//   0x08  CH_SRC, 0x0C CH_DST, 0x10 CH_LEN        R/W   addresses + length
//   0x14  CH_DESC_PTR  R/W   first descriptor
//   0x18  CH_NEXT_DESC R/O   live next-desc pointer
//
// Descriptor (32 B): {src, dst, len, flags, next_ptr, rsv[3]} (each 32-bit).
//
// For phase-3.5 the module is UNCONNECTED in mac_top.v; it gets wired
// as a new xbar master once task #19's retune lands (follow-up task
// `dma-ctrl-wire`).  Unit tb `tb-dma-ctrl` exercises it standalone.
//
// Verilog-2005, synchronous active-high `rst`, 4-space indent, no latches.

`default_nettype none

module dma_ctrl #(
    parameter N_CH         = 4,          // 4 channels; room for 8
    parameter DATA_WIDTH   = 64,
    parameter STRB_WIDTH   = DATA_WIDTH/8,
    parameter ADDR_WIDTH   = 32,
    parameter ID_WIDTH     = 4,
    parameter DEFAULT_BURST_LOG2 = 2     // 4 beats default
) (
    input  wire                     clk,
    input  wire                     rst,

    // ── AXI4-Lite slave  — config bank (20-bit byte address) ────────────
    input  wire [19:0]              cfg_awaddr,
    input  wire                     cfg_awvalid,
    output wire                     cfg_awready,
    input  wire [31:0]              cfg_wdata,
    input  wire [3:0]               cfg_wstrb,
    input  wire                     cfg_wvalid,
    output wire                     cfg_wready,
    output wire [1:0]               cfg_bresp,
    output wire                     cfg_bvalid,
    input  wire                     cfg_bready,

    input  wire [19:0]              cfg_araddr,
    input  wire                     cfg_arvalid,
    output wire                     cfg_arready,
    output wire [31:0]              cfg_rdata,
    output wire [1:0]               cfg_rresp,
    output wire                     cfg_rvalid,
    input  wire                     cfg_rready,

    // ── AXI4 master  — data-movement port ───────────────────────────────
    output wire [ID_WIDTH-1:0]      m_awid,
    output wire [ADDR_WIDTH-1:0]    m_awaddr,
    output wire [7:0]               m_awlen,
    output wire [2:0]               m_awsize,
    output wire [1:0]               m_awburst,
    output wire                     m_awvalid,
    input  wire                     m_awready,

    output wire [DATA_WIDTH-1:0]    m_wdata,
    output wire [STRB_WIDTH-1:0]    m_wstrb,
    output wire                     m_wlast,
    output wire                     m_wvalid,
    input  wire                     m_wready,

    input  wire [ID_WIDTH-1:0]      m_bid,
    input  wire [1:0]               m_bresp,
    input  wire                     m_bvalid,
    output wire                     m_bready,

    output wire [ID_WIDTH-1:0]      m_arid,
    output wire [ADDR_WIDTH-1:0]    m_araddr,
    output wire [7:0]               m_arlen,
    output wire [2:0]               m_arsize,
    output wire [1:0]               m_arburst,
    output wire                     m_arvalid,
    input  wire                     m_arready,

    input  wire [ID_WIDTH-1:0]      m_rid,
    input  wire [DATA_WIDTH-1:0]    m_rdata,
    input  wire [1:0]               m_rresp,
    input  wire                     m_rlast,
    input  wire                     m_rvalid,
    output wire                     m_rready,

    // ── IRQ line — level, OR of per-channel done/error when irq_en ──────
    output wire                     irq
);

    // ══════════════════════════════════════════════════════════════════════
    //  Globals
    // ══════════════════════════════════════════════════════════════════════
    reg g_enable;
    reg g_irq_en;

    // ══════════════════════════════════════════════════════════════════════
    //  Per-channel config / working state
    // ══════════════════════════════════════════════════════════════════════
    reg [N_CH-1:0]       ch_enable;
    reg [N_CH-1:0]       ch_desc_mode;
    reg [N_CH-1:0]       ch_irq_en;
    reg [3:0]            ch_burst_log2 [0:N_CH-1];

    reg [ADDR_WIDTH-1:0] ch_src        [0:N_CH-1];
    reg [ADDR_WIDTH-1:0] ch_dst        [0:N_CH-1];
    reg [31:0]           ch_len        [0:N_CH-1];
    reg [ADDR_WIDTH-1:0] ch_desc_ptr   [0:N_CH-1];
    reg [ADDR_WIDTH-1:0] ch_next_desc  [0:N_CH-1];

    // FSM states (encoded in 4 bits):
    localparam S_IDLE     = 4'd0;
    localparam S_DESC_AR  = 4'd1;
    localparam S_DESC_R   = 4'd2;
    localparam S_READY    = 4'd3;
    localparam S_RD_AR    = 4'd4;
    localparam S_RD_R     = 4'd5;
    localparam S_WR_AW    = 4'd6;
    localparam S_WR_W     = 4'd7;
    localparam S_WR_B     = 4'd8;
    localparam S_SEG_DONE = 4'd9;
    localparam S_DONE     = 4'd10;
    localparam S_ERROR    = 4'd11;
    localparam S_PAUSED   = 4'd12;

    reg [3:0]            ch_state      [0:N_CH-1];
    reg [N_CH-1:0]       ch_busy;
    reg [N_CH-1:0]       ch_done;      // sticky
    reg [N_CH-1:0]       ch_error;     // sticky
    reg [N_CH-1:0]       ch_desc_active;

    reg [31:0]           ch_seg_bytes  [0:N_CH-1];
    reg [7:0]            ch_seg_beats  [0:N_CH-1];
    reg [DATA_WIDTH-1:0] ch_buf        [0:N_CH-1][0:15];
    reg [4:0]            ch_buf_rd     [0:N_CH-1];
    reg [4:0]            ch_buf_wr     [0:N_CH-1];

    reg [63:0]           ch_desc_buf   [0:N_CH-1][0:3];
    reg [2:0]            ch_desc_idx   [0:N_CH-1];

    // ══════════════════════════════════════════════════════════════════════
    //  AXI-Lite config slave — state
    // ══════════════════════════════════════════════════════════════════════
    localparam CFG_W_IDLE  = 2'd0;
    localparam CFG_W_DATA  = 2'd1;
    localparam CFG_W_APPLY = 2'd2;
    localparam CFG_W_RESP  = 2'd3;

    reg [1:0]   cfg_wr_state;
    reg [19:0]  cfg_wr_addr;
    reg [31:0]  cfg_wr_data;
    reg [3:0]   cfg_wr_strb;
    reg         cfg_bvalid_r;

    reg         cfg_rvalid_r;
    reg [31:0]  cfg_rdata_r;

    assign cfg_awready = (cfg_wr_state == CFG_W_IDLE);
    assign cfg_wready  = (cfg_wr_state == CFG_W_DATA);
    assign cfg_bvalid  = cfg_bvalid_r;
    assign cfg_bresp   = 2'b00;

    assign cfg_arready = !cfg_rvalid_r;
    assign cfg_rvalid  = cfg_rvalid_r;
    assign cfg_rdata   = cfg_rdata_r;
    assign cfg_rresp   = 2'b00;

    // Zero-extend channel bitmaps to 4 bits for DMA_STATUS read-back.
    wire [3:0] ch_busy_x  = ch_busy[3:0];
    wire [3:0] ch_done_x  = ch_done[3:0];
    wire [3:0] ch_error_x = ch_error[3:0];
    wire       global_irq_w;

    // Combinational read mux over config bank.
    reg [31:0] cfg_rdata_next;
    reg [3:0]  rd_ch;
    reg [5:0]  rd_off;
    reg        rd_is_ch;

    always @(*) begin
        cfg_rdata_next = 32'h0;
        rd_ch          = 4'hF;
        rd_off         = 6'h0;
        rd_is_ch       = 1'b0;

        if (cfg_araddr[19:8] == 12'h001) begin
            rd_is_ch = 1'b1;
            rd_ch    = cfg_araddr[7:6];
            rd_off   = cfg_araddr[5:0];
        end

        if (!rd_is_ch) begin
            case (cfg_araddr[11:0])
                12'h000: cfg_rdata_next = {30'h0, g_irq_en, g_enable};
                12'h004: cfg_rdata_next = {7'h0,  global_irq_w,
                                            4'h0, ch_error_x,
                                            4'h0, ch_done_x,
                                            4'h0, ch_busy_x};
                12'h008: cfg_rdata_next = 32'h0;
                default: cfg_rdata_next = 32'h0;
            endcase
        end else if (rd_ch < N_CH) begin
            case (rd_off)
                6'h00: cfg_rdata_next = {24'h0,
                                          ch_burst_log2[rd_ch],
                                          ch_irq_en[rd_ch],
                                          ch_desc_mode[rd_ch],
                                          1'b0,
                                          ch_enable[rd_ch]};
                6'h04: cfg_rdata_next = {28'h0,
                                          ch_desc_active[rd_ch],
                                          ch_error[rd_ch],
                                          ch_done[rd_ch],
                                          ch_busy[rd_ch]};
                6'h08: cfg_rdata_next = ch_src[rd_ch];
                6'h0C: cfg_rdata_next = ch_dst[rd_ch];
                6'h10: cfg_rdata_next = ch_len[rd_ch];
                6'h14: cfg_rdata_next = ch_desc_ptr[rd_ch];
                6'h18: cfg_rdata_next = ch_next_desc[rd_ch];
                default: cfg_rdata_next = 32'h0;
            endcase
        end
    end

    // ══════════════════════════════════════════════════════════════════════
    //  Arbiter — which channel (if any) owns the AXI master right now
    // ══════════════════════════════════════════════════════════════════════
    wire [N_CH-1:0] ch_bus_req;
    genvar gi;
    generate
        for (gi = 0; gi < N_CH; gi = gi + 1) begin : g_busreq
            assign ch_bus_req[gi] = (ch_state[gi] == S_DESC_AR)
                                 || (ch_state[gi] == S_DESC_R)
                                 || (ch_state[gi] == S_RD_AR)
                                 || (ch_state[gi] == S_RD_R)
                                 || (ch_state[gi] == S_WR_AW)
                                 || (ch_state[gi] == S_WR_W)
                                 || (ch_state[gi] == S_WR_B);
        end
    endgenerate

    localparam CH_IDX_W = (N_CH > 1) ? $clog2(N_CH) : 1;
    reg  [CH_IDX_W-1:0] arb_ch;
    reg                 arb_active;
    reg  [CH_IDX_W-1:0] arb_last;

    wire arb_release = arb_active && !ch_bus_req[arb_ch];

    // Compute next round-robin owner (combinational)
    reg  [CH_IDX_W-1:0] rr_next;
    integer             rr_i;
    reg                 rr_hit;
    integer             rr_idx;
    always @(*) begin
        rr_next = arb_last;
        rr_hit  = 1'b0;
        for (rr_i = 1; rr_i <= N_CH; rr_i = rr_i + 1) begin
            rr_idx = (arb_last + rr_i) % N_CH;
            if (!rr_hit && ch_bus_req[rr_idx[CH_IDX_W-1:0]]) begin
                rr_next = rr_idx[CH_IDX_W-1:0];
                rr_hit  = 1'b1;
            end
        end
    end

    // ══════════════════════════════════════════════════════════════════════
    //  AXI master signal generation — combinational from active channel
    // ══════════════════════════════════════════════════════════════════════
    localparam [2:0] AXSIZE_DATA = $clog2(DATA_WIDTH/8);
    localparam [2:0] AXSIZE_DESC = 3'd3;

    wire [3:0] active_burst_log2 = ch_burst_log2[arb_ch];
    wire [7:0] active_burst_beats = (active_burst_log2 > 4'd4) ? 8'd16
                                                               : (8'd1 << active_burst_log2);

    reg [ID_WIDTH-1:0]       m_awid_r;
    reg [ADDR_WIDTH-1:0]     m_awaddr_r;
    reg [7:0]                m_awlen_r;
    reg [2:0]                m_awsize_r;
    reg [1:0]                m_awburst_r;
    reg                      m_awvalid_r;

    reg [DATA_WIDTH-1:0]     m_wdata_r;
    reg [STRB_WIDTH-1:0]     m_wstrb_r;
    reg                      m_wlast_r;
    reg                      m_wvalid_r;

    reg                      m_bready_r;

    reg [ID_WIDTH-1:0]       m_arid_r;
    reg [ADDR_WIDTH-1:0]     m_araddr_r;
    reg [7:0]                m_arlen_r;
    reg [2:0]                m_arsize_r;
    reg [1:0]                m_arburst_r;
    reg                      m_arvalid_r;

    reg                      m_rready_r;

    assign m_awid     = m_awid_r;
    assign m_awaddr   = m_awaddr_r;
    assign m_awlen    = m_awlen_r;
    assign m_awsize   = m_awsize_r;
    assign m_awburst  = m_awburst_r;
    assign m_awvalid  = m_awvalid_r;
    assign m_wdata    = m_wdata_r;
    assign m_wstrb    = m_wstrb_r;
    assign m_wlast    = m_wlast_r;
    assign m_wvalid   = m_wvalid_r;
    assign m_bready   = m_bready_r;
    assign m_arid     = m_arid_r;
    assign m_araddr   = m_araddr_r;
    assign m_arlen    = m_arlen_r;
    assign m_arsize   = m_arsize_r;
    assign m_arburst  = m_arburst_r;
    assign m_arvalid  = m_arvalid_r;
    assign m_rready   = m_rready_r;

    // W-beat index used for wlast calculation.
    wire [4:0] active_buf_wr = ch_buf_wr[arb_ch];
    wire [7:0] active_seg_beats = ch_seg_beats[arb_ch];

    always @(*) begin
        m_awid_r     = {ID_WIDTH{1'b0}};
        m_awaddr_r   = {ADDR_WIDTH{1'b0}};
        m_awlen_r    = 8'h0;
        m_awsize_r   = 3'h0;
        m_awburst_r  = 2'b01;
        m_awvalid_r  = 1'b0;
        m_wdata_r    = {DATA_WIDTH{1'b0}};
        m_wstrb_r    = {STRB_WIDTH{1'b0}};
        m_wlast_r    = 1'b0;
        m_wvalid_r   = 1'b0;
        m_bready_r   = 1'b0;
        m_arid_r     = {ID_WIDTH{1'b0}};
        m_araddr_r   = {ADDR_WIDTH{1'b0}};
        m_arlen_r    = 8'h0;
        m_arsize_r   = 3'h0;
        m_arburst_r  = 2'b01;
        m_arvalid_r  = 1'b0;
        m_rready_r   = 1'b0;

        if (arb_active) begin
            // Tag AXI ID with the channel index (zero-extended).
            m_arid_r = {{(ID_WIDTH-CH_IDX_W){1'b0}}, arb_ch};
            m_awid_r = {{(ID_WIDTH-CH_IDX_W){1'b0}}, arb_ch};
            case (ch_state[arb_ch])
                S_DESC_AR: begin
                    m_arvalid_r = 1'b1;
                    m_araddr_r  = ch_desc_ptr[arb_ch];
                    m_arlen_r   = 8'd3;  // 4 beats
                    m_arsize_r  = AXSIZE_DESC;
                end
                S_DESC_R: begin
                    m_rready_r  = 1'b1;
                end
                S_RD_AR: begin
                    m_arvalid_r = 1'b1;
                    m_araddr_r  = ch_src[arb_ch];
                    m_arlen_r   = ch_seg_beats[arb_ch] - 8'd1;
                    m_arsize_r  = AXSIZE_DATA;
                end
                S_RD_R: begin
                    m_rready_r  = 1'b1;
                end
                S_WR_AW: begin
                    m_awvalid_r = 1'b1;
                    m_awaddr_r  = ch_dst[arb_ch];
                    m_awlen_r   = ch_seg_beats[arb_ch] - 8'd1;
                    m_awsize_r  = AXSIZE_DATA;
                end
                S_WR_W: begin
                    m_wvalid_r = 1'b1;
                    m_wdata_r  = ch_buf[arb_ch][active_buf_wr];
                    m_wstrb_r  = {STRB_WIDTH{1'b1}};
                    m_wlast_r  = ({3'h0, active_buf_wr} + 8'd1 == active_seg_beats);
                end
                S_WR_B: begin
                    m_bready_r = 1'b1;
                end
                default: ;
            endcase
        end
    end

    // ══════════════════════════════════════════════════════════════════════
    //  Config-write decode (pure combinational)
    // ══════════════════════════════════════════════════════════════════════
    wire        is_ch_write = (cfg_wr_addr[19:8] == 12'h001);
    wire [3:0]  wr_ch       = cfg_wr_addr[7:6];
    wire [5:0]  wr_off      = cfg_wr_addr[5:0];
    wire        cfg_apply   = (cfg_wr_state == CFG_W_APPLY);

    // One-cycle pulses — latched in the sequential block so they remain
    // valid for the FSM on the SAME cycle that ch_enable etc. are being
    // written to.  A pending_* register latches on the cfg_apply cycle;
    // the FSM consumes it on the next cycle (when ch_enable has landed)
    // and it self-clears.
    reg  [N_CH-1:0] pending_start;
    reg  [N_CH-1:0] pending_disable;
    reg  [N_CH-1:0] pending_irq_clr;
    wire [N_CH-1:0] do_start_bm    = pending_start;
    wire [N_CH-1:0] do_disable_bm  = pending_disable;
    wire [N_CH-1:0] do_irq_clr_bm  = pending_irq_clr;

    // ══════════════════════════════════════════════════════════════════════
    //  Segment planner (pure combinational — returns {beats, bytes})
    // ══════════════════════════════════════════════════════════════════════
    function [39:0] plan_segment;
        input [31:0] rem_len;
        input [31:0] src_addr;
        input [31:0] dst_addr;
        input [7:0]  max_beats;   // active_burst_beats
        reg   [31:0] bytes;
        reg   [31:0] beats_tmp;
        reg   [7:0]  beats;
        reg   [31:0] bb;
        begin
            beats = max_beats;
            beats_tmp = {24'h0, beats};
            bytes = beats_tmp * (DATA_WIDTH/8);
            if (bytes > rem_len) begin
                bytes = rem_len;
                beats_tmp = (bytes + (DATA_WIDTH/8) - 1) / (DATA_WIDTH/8);
                beats     = beats_tmp[7:0];
                if (beats == 0) beats = 8'd1;
            end
            bb = 32'h1000 - {20'h0, src_addr[11:0]};
            if (bytes > bb) begin
                bytes = bb;
                beats_tmp = bytes / (DATA_WIDTH/8);
                beats     = beats_tmp[7:0];
                if (beats == 0) beats = 8'd1;
            end
            bb = 32'h1000 - {20'h0, dst_addr[11:0]};
            if (bytes > bb) begin
                bytes = bb;
                beats_tmp = bytes / (DATA_WIDTH/8);
                beats     = beats_tmp[7:0];
                if (beats == 0) beats = 8'd1;
            end
            plan_segment = {beats, bytes};
        end
    endfunction

    // ══════════════════════════════════════════════════════════════════════
    //  Per-channel segment-plan wires (computed combinationally from state)
    // ══════════════════════════════════════════════════════════════════════
    wire [7:0]  per_ch_burst_beats [0:N_CH-1];
    wire [39:0] plan_out_per_ch    [0:N_CH-1];
    genvar pgi;
    generate
        for (pgi = 0; pgi < N_CH; pgi = pgi + 1) begin : g_plan
            assign per_ch_burst_beats[pgi] = (ch_burst_log2[pgi] > 4'd4)
                                             ? 8'd16
                                             : (8'd1 << ch_burst_log2[pgi]);
            assign plan_out_per_ch[pgi] = plan_segment(ch_len[pgi], ch_src[pgi],
                                                       ch_dst[pgi],
                                                       per_ch_burst_beats[pgi]);
        end
    endgenerate

    // ══════════════════════════════════════════════════════════════════════
    //  Combined sequential block — ALL reg updates
    // ══════════════════════════════════════════════════════════════════════
    // One always block so the compiler is OK with the multi-writer targets
    // (ch_src / ch_dst / ch_len / ch_desc_ptr / ch_next_desc) that both CPU
    // writes and FSM transitions mutate.
    integer c, bi;
    integer i;

    always @(posedge clk) begin
        if (rst) begin
            g_enable     <= 1'b0;
            g_irq_en     <= 1'b0;
            cfg_wr_state <= CFG_W_IDLE;
            cfg_wr_addr  <= 20'h0;
            cfg_wr_data  <= 32'h0;
            cfg_wr_strb  <= 4'h0;
            cfg_bvalid_r <= 1'b0;
            cfg_rvalid_r <= 1'b0;
            cfg_rdata_r  <= 32'h0;

            ch_enable    <= {N_CH{1'b0}};
            ch_desc_mode <= {N_CH{1'b0}};
            ch_irq_en    <= {N_CH{1'b0}};
            ch_busy      <= {N_CH{1'b0}};
            ch_done      <= {N_CH{1'b0}};
            ch_error     <= {N_CH{1'b0}};
            ch_desc_active <= {N_CH{1'b0}};

            pending_start   <= {N_CH{1'b0}};
            pending_disable <= {N_CH{1'b0}};
            pending_irq_clr <= {N_CH{1'b0}};

            arb_ch       <= {CH_IDX_W{1'b0}};
            arb_active   <= 1'b0;
            arb_last     <= {CH_IDX_W{1'b0}};

            for (i = 0; i < N_CH; i = i + 1) begin
                ch_state[i]        <= S_IDLE;
                ch_burst_log2[i]   <= DEFAULT_BURST_LOG2;
                ch_src[i]          <= {ADDR_WIDTH{1'b0}};
                ch_dst[i]          <= {ADDR_WIDTH{1'b0}};
                ch_len[i]          <= 32'h0;
                ch_desc_ptr[i]     <= {ADDR_WIDTH{1'b0}};
                ch_next_desc[i]    <= {ADDR_WIDTH{1'b0}};
                ch_seg_bytes[i]    <= 32'h0;
                ch_seg_beats[i]    <= 8'h0;
                ch_buf_rd[i]       <= 5'h0;
                ch_buf_wr[i]       <= 5'h0;
                ch_desc_idx[i]     <= 3'h0;
                for (bi = 0; bi < 16; bi = bi + 1)
                    ch_buf[i][bi] <= {DATA_WIDTH{1'b0}};
                for (bi = 0; bi < 4; bi = bi + 1)
                    ch_desc_buf[i][bi] <= 64'h0;
            end
        end else begin
            // Default: pending-pulses self-clear each cycle (they're
            // consumed by the FSM block later in this same always).
            pending_start   <= {N_CH{1'b0}};
            pending_disable <= {N_CH{1'b0}};
            pending_irq_clr <= {N_CH{1'b0}};

            // ── AXI-Lite write FSM ────────────────────────────────────────
            case (cfg_wr_state)
                CFG_W_IDLE: begin
                    if (cfg_awvalid) begin
                        cfg_wr_addr  <= cfg_awaddr;
                        cfg_wr_state <= CFG_W_DATA;
                    end
                end
                CFG_W_DATA: begin
                    if (cfg_wvalid) begin
                        cfg_wr_data  <= cfg_wdata;
                        cfg_wr_strb  <= cfg_wstrb;
                        cfg_wr_state <= CFG_W_APPLY;
                    end
                end
                CFG_W_APPLY: begin
                    if (!is_ch_write) begin
                        case (cfg_wr_addr[11:0])
                            12'h000: begin
                                if (cfg_wr_strb[0]) begin
                                    g_enable <= cfg_wr_data[0];
                                    g_irq_en <= cfg_wr_data[1];
                                end
                            end
                            12'h008: begin
                                if (cfg_wr_strb[0]) begin
                                    for (i = 0; i < N_CH; i = i + 1)
                                        if (cfg_wr_data[i])
                                            pending_irq_clr[i] <= 1'b1;
                                end
                            end
                            default: ;
                        endcase
                    end else if ({28'h0, wr_ch} < N_CH) begin
                        case (wr_off)
                            6'h00: begin
                                if (cfg_wr_strb[0]) begin
                                    ch_enable[wr_ch[CH_IDX_W-1:0]]    <= cfg_wr_data[0];
                                    ch_desc_mode[wr_ch[CH_IDX_W-1:0]] <= cfg_wr_data[2];
                                    ch_irq_en[wr_ch[CH_IDX_W-1:0]]    <= cfg_wr_data[3];
                                    ch_burst_log2[wr_ch[CH_IDX_W-1:0]] <= cfg_wr_data[7:4];
                                    if (cfg_wr_data[1])
                                        pending_start[wr_ch[CH_IDX_W-1:0]]   <= 1'b1;
                                    if (!cfg_wr_data[0])
                                        pending_disable[wr_ch[CH_IDX_W-1:0]] <= 1'b1;
                                end
                            end
                            6'h08: if (|cfg_wr_strb) ch_src[wr_ch[CH_IDX_W-1:0]]      <= apply_wstrb32(ch_src[wr_ch[CH_IDX_W-1:0]],      cfg_wr_data, cfg_wr_strb);
                            6'h0C: if (|cfg_wr_strb) ch_dst[wr_ch[CH_IDX_W-1:0]]      <= apply_wstrb32(ch_dst[wr_ch[CH_IDX_W-1:0]],      cfg_wr_data, cfg_wr_strb);
                            6'h10: if (|cfg_wr_strb) ch_len[wr_ch[CH_IDX_W-1:0]]      <= apply_wstrb32(ch_len[wr_ch[CH_IDX_W-1:0]],      cfg_wr_data, cfg_wr_strb);
                            6'h14: if (|cfg_wr_strb) ch_desc_ptr[wr_ch[CH_IDX_W-1:0]] <= apply_wstrb32(ch_desc_ptr[wr_ch[CH_IDX_W-1:0]], cfg_wr_data, cfg_wr_strb);
                            default: ;
                        endcase
                    end

                    cfg_bvalid_r <= 1'b1;
                    cfg_wr_state <= CFG_W_RESP;
                end
                CFG_W_RESP: begin
                    if (cfg_bready && cfg_bvalid_r) begin
                        cfg_bvalid_r <= 1'b0;
                        cfg_wr_state <= CFG_W_IDLE;
                    end
                end
                default: cfg_wr_state <= CFG_W_IDLE;
            endcase

            // ── AXI-Lite read handshake (1 cycle) ─────────────────────────
            if (!cfg_rvalid_r && cfg_arvalid) begin
                cfg_rdata_r  <= cfg_rdata_next;
                cfg_rvalid_r <= 1'b1;
            end else if (cfg_rvalid_r && cfg_rready) begin
                cfg_rvalid_r <= 1'b0;
            end

            // ── Arbiter update ────────────────────────────────────────────
            if (!arb_active || arb_release) begin
                if (|ch_bus_req) begin
                    arb_ch     <= rr_next;
                    arb_last   <= rr_next;
                    arb_active <= 1'b1;
                end else begin
                    arb_active <= 1'b0;
                end
            end

            // ── Per-channel FSM ───────────────────────────────────────────
            for (c = 0; c < N_CH; c = c + 1) begin
                // IRQ-clear (any state)
                if (do_irq_clr_bm[c]) begin
                    ch_done[c]  <= 1'b0;
                    ch_error[c] <= 1'b0;
                end

                case (ch_state[c])
                    S_IDLE: begin
                        if (do_start_bm[c] && ch_enable[c] && g_enable) begin
                            ch_busy[c]        <= 1'b1;
                            ch_done[c]        <= 1'b0;
                            ch_error[c]       <= 1'b0;
                            ch_buf_rd[c]      <= 5'h0;
                            ch_buf_wr[c]      <= 5'h0;
                            ch_desc_idx[c]    <= 3'h0;
                            if (ch_desc_mode[c]) begin
                                ch_desc_active[c] <= 1'b1;
                                ch_state[c]        <= S_DESC_AR;
                            end else begin
                                ch_desc_active[c] <= 1'b0;
                                ch_state[c]        <= S_READY;
                            end
                        end
                    end

                    S_DESC_AR: begin
                        // AR-accept this cycle wins over pause so we
                        // don't lose the burst to an uncollected R.
                        if (arb_active && arb_ch == c[CH_IDX_W-1:0] && m_arvalid && m_arready) begin
                            ch_state[c] <= S_DESC_R;
                        end else if (!g_enable || do_disable_bm[c]) begin
                            ch_state[c] <= S_PAUSED;
                        end
                    end

                    S_DESC_R: begin
                        if (arb_active && arb_ch == c[CH_IDX_W-1:0] && m_rvalid && m_rready_r) begin
                            ch_desc_buf[c][ch_desc_idx[c][1:0]] <= m_rdata[63:0];
                            if (m_rresp != 2'b00) begin
                                ch_error[c] <= 1'b1;
                                ch_busy[c]  <= 1'b0;
                                ch_state[c] <= S_ERROR;
                            end else if (m_rlast) begin
                                // Unpack all beats — the first three are in
                                // ch_desc_buf[c][0..2] (written on earlier
                                // cycles), the final beat (index 3) arrived
                                // this cycle and is being latched above;
                                // we use m_rdata directly.
                                ch_src[c]       <= ch_desc_buf[c][0][31:0];
                                ch_dst[c]       <= ch_desc_buf[c][0][63:32];
                                ch_len[c]       <= ch_desc_buf[c][1][31:0];
                                ch_next_desc[c] <= ch_desc_buf[c][2][31:0];
                                // beat 3 = reserved, ignored.
                                // If ARLEN were somehow < 3 we'd still land
                                // here; fields past the stored beats remain
                                // at previous values.  Defensive: use
                                // current idx to identify which unpacking
                                // is actually covered (future-proofing).
                                ch_state[c] <= S_READY;
                                ch_desc_idx[c] <= 3'h0;
                            end else begin
                                ch_desc_idx[c] <= ch_desc_idx[c] + 3'd1;
                            end
                        end
                    end

                    S_READY: begin
                        if (!g_enable || do_disable_bm[c]) begin
                            ch_state[c] <= S_PAUSED;
                        end else if (ch_len[c] == 32'h0) begin
                            // empty segment → skip to SEG_DONE so chain can
                            // follow.
                            ch_seg_bytes[c] <= 32'h0;
                            ch_state[c]     <= S_SEG_DONE;
                        end else begin
                            ch_seg_bytes[c] <= plan_out_per_ch[c][31:0];
                            ch_seg_beats[c] <= plan_out_per_ch[c][39:32];
                            ch_buf_rd[c]    <= 5'h0;
                            ch_buf_wr[c]    <= 5'h0;
                            ch_state[c]     <= S_RD_AR;
                        end
                    end

                    S_RD_AR: begin
                        // AR-accept this cycle wins over pause so we
                        // don't leave the slave holding beats we never read.
                        if (arb_active && arb_ch == c[CH_IDX_W-1:0] && m_arvalid && m_arready) begin
                            ch_state[c] <= S_RD_R;
                        end else if (!g_enable || do_disable_bm[c]) begin
                            ch_state[c] <= S_PAUSED;
                        end
                    end

                    S_RD_R: begin
                        if (arb_active && arb_ch == c[CH_IDX_W-1:0] && m_rvalid && m_rready_r) begin
                            ch_buf[c][ch_buf_rd[c]] <= m_rdata;
                            if (m_rresp != 2'b00) begin
                                ch_error[c] <= 1'b1;
                                ch_busy[c]  <= 1'b0;
                                ch_state[c] <= S_ERROR;
                            end else begin
                                ch_buf_rd[c] <= ch_buf_rd[c] + 5'd1;
                                if (m_rlast) begin
                                    ch_state[c] <= S_WR_AW;
                                end
                            end
                        end
                    end

                    S_WR_AW: begin
                        if (arb_active && arb_ch == c[CH_IDX_W-1:0] && m_awvalid && m_awready) begin
                            ch_state[c] <= S_WR_W;
                        end
                    end

                    S_WR_W: begin
                        if (arb_active && arb_ch == c[CH_IDX_W-1:0] && m_wvalid && m_wready) begin
                            ch_buf_wr[c] <= ch_buf_wr[c] + 5'd1;
                            if (m_wlast_r) begin
                                ch_state[c] <= S_WR_B;
                            end
                        end
                    end

                    S_WR_B: begin
                        if (arb_active && arb_ch == c[CH_IDX_W-1:0] && m_bvalid && m_bready_r) begin
                            if (m_bresp != 2'b00) begin
                                ch_error[c] <= 1'b1;
                                ch_busy[c]  <= 1'b0;
                                ch_state[c] <= S_ERROR;
                            end else begin
                                ch_state[c] <= S_SEG_DONE;
                            end
                        end
                    end

                    S_SEG_DONE: begin
                        ch_src[c] <= ch_src[c] + ch_seg_bytes[c];
                        ch_dst[c] <= ch_dst[c] + ch_seg_bytes[c];
                        ch_len[c] <= (ch_len[c] >= ch_seg_bytes[c]) ? (ch_len[c] - ch_seg_bytes[c]) : 32'h0;

                        if (ch_len[c] <= ch_seg_bytes[c]) begin
                            // Descriptor (or direct xfer) complete
                            if (ch_desc_mode[c] && ch_desc_active[c] && ch_next_desc[c] != 32'h0) begin
                                ch_desc_ptr[c] <= ch_next_desc[c];
                                ch_desc_idx[c] <= 3'h0;
                                ch_state[c]    <= S_DESC_AR;
                            end else begin
                                ch_state[c] <= S_DONE;
                            end
                        end else begin
                            ch_state[c] <= S_READY;
                        end
                    end

                    S_DONE: begin
                        ch_busy[c]        <= 1'b0;
                        ch_done[c]        <= 1'b1;
                        ch_desc_active[c] <= 1'b0;
                        ch_state[c]       <= S_IDLE;
                    end

                    S_ERROR: begin
                        ch_busy[c]        <= 1'b0;
                        ch_desc_active[c] <= 1'b0;
                        ch_state[c]       <= S_IDLE;
                    end

                    S_PAUSED: begin
                        // Resume when software re-enables (or global enable
                        // returns AND no disable request outstanding).
                        if (g_enable && ch_enable[c] && !do_disable_bm[c]) begin
                            ch_state[c] <= S_READY;
                        end else if (!ch_enable[c]) begin
                            // Hard-disable finalisation
                            ch_busy[c]        <= 1'b0;
                            ch_desc_active[c] <= 1'b0;
                            ch_state[c]       <= S_IDLE;
                        end
                    end

                    default: ch_state[c] <= S_IDLE;
                endcase
            end
        end
    end

    // ══════════════════════════════════════════════════════════════════════
    //  IRQ aggregation
    // ══════════════════════════════════════════════════════════════════════
    wire [N_CH-1:0] ch_irq_pending = (ch_done | ch_error) & ch_irq_en;
    assign global_irq_w = g_irq_en & (|ch_irq_pending);
    assign irq          = global_irq_w;

    // ══════════════════════════════════════════════════════════════════════
    //  Helpers
    // ══════════════════════════════════════════════════════════════════════
    function [31:0] apply_wstrb32;
        input [31:0] cur;
        input [31:0] new_val;
        input [3:0]  strb;
        begin
            apply_wstrb32 = {
                strb[3] ? new_val[31:24] : cur[31:24],
                strb[2] ? new_val[23:16] : cur[23:16],
                strb[1] ? new_val[15: 8] : cur[15: 8],
                strb[0] ? new_val[ 7: 0] : cur[ 7: 0]
            };
        end
    endfunction

    // ══════════════════════════════════════════════════════════════════════
    //  Lint-avoidance — signals present for parameter scaling headroom.
    // ══════════════════════════════════════════════════════════════════════
    /* verilator lint_off UNUSEDSIGNAL */
    wire [ID_WIDTH-1:0] m_bid_unused = m_bid;
    wire [ID_WIDTH-1:0] m_rid_unused = m_rid;
    /* verilator lint_on UNUSEDSIGNAL */

endmodule

`default_nettype wire
