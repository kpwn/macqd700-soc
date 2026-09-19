// sd_bulk_writer.v — burst-capable AXI4 front-end for bulk SD-card writes.
//
// Purpose
//   Fast-path companion to sd_jtag_writer.v for the DEDICATED provisioning
//   bitstream (sd_provision_top.v).  Where sd_jtag_writer moves one byte
//   per AXI transaction (~515 JTAG round-trips per sector — fine for a
//   few provisioning sectors, hopeless for a 10 MiB disk image), this
//   module accepts full AXI4 INCR write BURSTS (up to 256 x 32-bit beats
//   per transaction, i.e. 1 KiB per JTAG round-trip) into a large
//   STAGE_SECTORS x 512-byte staging BRAM, then streams the whole batch
//   to the card in hardware with a single CMD25 WRITE_MULTIPLE_BLOCK.
//
//   A CMD18 READ_MULTIPLE_BLOCK "CRC verify" op reads the same LBA range
//   back from the card and accumulates a zlib-compatible CRC32 over the
//   byte stream (data itself is discarded), so the host can verify a
//   whole batch end-to-end with two register pokes instead of re-reading
//   128 KiB over JTAG.
//
// Address map (bit 20 of the AXI address selects the window):
//   addr[20] == 0 — register window (byte offsets):
//     0x00 LBA     R/W  target start sector (block address)
//     0x04 CTRL    W    0x5DB00001 = bulk CMD25 write of BLKCNT sectors
//                                    from staging RAM at LBA
//                       0x5DB00002 = bulk CMD18 CRC32 verify of BLKCNT
//                                    sectors starting at LBA
//     0x08 STATUS  R    bit0 busy, bit1 done-sticky, bit2 error,
//                       bits[7:4] err_cause (sd_ctrl codes 0..6,
//                       0xE = bad BLKCNT, 0xF = card not ready),
//                       bit8 card_ready, bit9 init_error
//     0x0C BLKCNT  R/W  sectors per bulk op, 1..STAGE_SECTORS
//     0x10 CRC32   R    zlib-final CRC32 of the last op's byte stream
//                       (write op: bytes streamed to the card; verify
//                       op: bytes read back from the card)
//     0x14 IDENT   R    0x5DB70001
//     0x18 CAPS    R    {version[15:0]=1, STAGE_SECTORS[15:0]}
//   addr[20] == 1 — staging RAM window, STAGE_SECTORS*512 bytes,
//     32-bit little-endian words (byte k of the sector stream lives in
//     word k>>2, bits [8*(k&3) +: 8]).  Burst reads are also served
//     (for spot verification of staged data).
//
// Interfaces / latency
//   One outstanding AXI write + one outstanding AXI read.  Write bursts
//   are consumed at one beat per cycle; read bursts at one beat per
//   three cycles (sync-BRAM issue/capture/present).  SPI streaming
//   reuses sd_ctrl's producer-paced wr_ready pulse exactly like
//   sd_jtag_writer (bytes are prefetched from the staging BRAM two
//   cycles ahead; wr_ready pulses are >= 16 core cycles apart so the
//   prefetch latency is hidden).
//
// The card must already be initialised (boot_fsm handles that in
// sd_provision_core.v); ops are accepted only while card_ready=1.
//
// Verilog-2005, synchronous active-high rst.

`default_nettype none

module sd_bulk_writer #(
    parameter integer STAGE_SECTORS = 256      // 128 KiB staging BRAM
) (
    input  wire        clk,
    input  wire        rst,

    // Card-init status (from boot_fsm in sd_provision_core).
    input  wire        card_ready,
    input  wire        init_error,

    // 32-bit AXI4 slave face (burst-capable; INCR, size=4 bytes).
    input  wire [31:0] s_awaddr,
    input  wire [7:0]  s_awlen,
    input  wire        s_awvalid,
    output wire        s_awready,
    input  wire [31:0] s_wdata,
    input  wire [3:0]  s_wstrb,
    input  wire        s_wlast,
    input  wire        s_wvalid,
    output wire        s_wready,
    output wire [1:0]  s_bresp,
    output wire        s_bvalid,
    input  wire        s_bready,

    input  wire [31:0] s_araddr,
    input  wire [7:0]  s_arlen,
    input  wire        s_arvalid,
    output wire        s_arready,
    output wire [31:0] s_rdata,
    output wire [1:0]  s_rresp,
    output wire        s_rlast,
    output wire        s_rvalid,
    input  wire        s_rready,

    // SPI byte interface to sd_spi_mux prov port.
    output wire        spi_cmd_valid,
    input  wire        spi_cmd_ready,
    output wire [7:0]  spi_cmd_data,
    input  wire        spi_rsp_valid,
    input  wire [7:0]  spi_rsp_data,
    output wire        spi_cs_n_in,
    output wire        spi_fast_mode,
    output wire        spi_hs_mode,

    output wire        writer_busy
);

    localparam [2:0] CT_CMD18 = 3'd2;
    localparam [2:0] CT_CMD25 = 3'd4;

    localparam integer STAGE_WORDS = STAGE_SECTORS * 128;
    localparam integer STAGE_AW    = $clog2(STAGE_WORDS);   // word addr bits
    localparam integer BIDX_W      = STAGE_AW + 3;          // byte idx + carry

    localparam [7:0]
        REG_LBA    = 8'h00,
        REG_CTRL   = 8'h04,
        REG_STATUS = 8'h08,
        REG_BLKCNT = 8'h0C,
        REG_CRC32  = 8'h10,
        REG_IDENT  = 8'h14,
        REG_CAPS   = 8'h18;

    localparam [31:0] CTRL_GO_WRITE  = 32'h5DB0_0001;
    localparam [31:0] CTRL_GO_VERIFY = 32'h5DB0_0002;
    localparam [31:0] IDENT_VALUE    = 32'h5DB7_0001;
    localparam [31:0] STAGE_SECTORS_W = STAGE_SECTORS;
    localparam [31:0] CAPS_VALUE     = {16'd1, STAGE_SECTORS_W[15:0]};

    // ──────────────────────────────────────────────────────────────────
    // Staging RAM — true dual port BRAM.
    //   Port A: AXI window (byte-enable writes + sync reads).
    //   Port B: SPI-stream byte prefetch (sync reads only).
    // No reset initialisation (BRAM contents are host-loaded).
    // ──────────────────────────────────────────────────────────────────
    (* ram_style = "block" *) reg [31:0] stage_ram [0:STAGE_WORDS-1];

    reg                 a_wen;
    reg  [3:0]          a_be;
    reg  [STAGE_AW-1:0] a_addr;
    reg  [31:0]         a_rdata;

    always @(posedge clk) begin
        if (a_wen) begin
            if (a_be[0]) stage_ram[a_addr][7:0]   <= s_wdata[7:0];
            if (a_be[1]) stage_ram[a_addr][15:8]  <= s_wdata[15:8];
            if (a_be[2]) stage_ram[a_addr][23:16] <= s_wdata[23:16];
            if (a_be[3]) stage_ram[a_addr][31:24] <= s_wdata[31:24];
        end
        a_rdata <= stage_ram[a_addr];
    end

    reg  [STAGE_AW-1:0] b_addr;
    reg  [31:0]         b_rdata;
    always @(posedge clk) begin
        b_rdata <= stage_ram[b_addr];
    end

    // ──────────────────────────────────────────────────────────────────
    // CRC32 (reflected, poly 0xEDB88320) — matches zlib crc32: init
    // 0xFFFFFFFF, final xor 0xFFFFFFFF (applied at the read mux).
    // ──────────────────────────────────────────────────────────────────
    // verilator lint_off BLKSEQ
    function [31:0] crc32_byte;
        input [31:0] c;
        input [7:0]  b;
        integer k;
        reg [31:0] x;
        begin
            x = c ^ {24'h000000, b};
            for (k = 0; k < 8; k = k + 1)
                x = (x >> 1) ^ (x[0] ? 32'hEDB8_8320 : 32'h0000_0000);
            crc32_byte = x;
        end
    endfunction
    // verilator lint_on BLKSEQ

    // ──────────────────────────────────────────────────────────────────
    // Registers + bulk-op state
    // ──────────────────────────────────────────────────────────────────
    reg [31:0] lba_q;
    reg [15:0] blkcnt_q;
    reg        done_sticky_q;
    reg        error_q;
    reg [3:0]  err_cause_q;
    reg [31:0] crc_q;
    reg        mode_write_q;        // 1 = CMD25 write, 0 = CMD18 verify

    reg        start_q;
    reg        ctrl_go_q;
    wire       ctrl_busy;
    wire       ctrl_done;
    wire       ctrl_error;
    wire [3:0] ctrl_err_cause;
    wire       ctrl_wr_ready;
    wire       ctrl_wr_valid_unused;
    wire       ctrl_rd_valid;
    wire [7:0] ctrl_rd_data;

    assign writer_busy   = start_q || ctrl_busy;
    assign spi_cs_n_in   = !writer_busy;
    assign spi_fast_mode = 1'b1;
    assign spi_hs_mode   = 1'b1;

    wire idle = !writer_busy;
    wire blkcnt_ok = (blkcnt_q != 16'd0) &&
                     ({16'd0, blkcnt_q} <= STAGE_SECTORS_W);

    // SPI-stream byte prefetch (write op).  Sync-BRAM: the addressed
    // word lands in b_rdata two posedges after b_addr is registered, so
    // the pending flag is a 2-stage shift (bpend0 → bpend1 → capture).
    reg [BIDX_W-1:0] bidx_q;        // current byte index in staging
    reg [1:0]        blane_q;       // byte lane of the prefetched word
    reg              bpend0_q;      // addr registered last posedge
    reg              bpend1_q;      // b_rdata valid — capture this cycle
    reg [7:0]        bdata_q;       // byte currently offered to sd_ctrl

    wire [BIDX_W-1:0] bidx_next = bidx_q + {{(BIDX_W-1){1'b0}}, 1'b1};

    // ──────────────────────────────────────────────────────────────────
    // AXI write channel — one outstanding burst, 1 beat/cycle.
    // ──────────────────────────────────────────────────────────────────
    localparam [1:0] W_IDLE = 2'd0, W_DATA = 2'd1, W_RESP = 2'd2;
    reg [1:0]  wstate_q;
    reg [31:0] waddr_q;

    assign s_awready = (wstate_q == W_IDLE);
    assign s_wready  = (wstate_q == W_DATA);
    assign s_bresp   = 2'b00;
    assign s_bvalid  = (wstate_q == W_RESP);

    wire wr_beat        = (wstate_q == W_DATA) && s_wvalid;
    wire wr_beat_stage  = wr_beat && waddr_q[20];
    wire wr_beat_reg    = wr_beat && !waddr_q[20];
    wire [7:0] wr_reg   = waddr_q[7:0] & 8'hFC;

    // ──────────────────────────────────────────────────────────────────
    // AXI read channel — one outstanding burst, 3 cycles/beat:
    //   R_ISSUE (drive BRAM addr / mux reg value) → R_CAPT (BRAM output
    //   settles; capture into rdata_q) → R_DATA (rvalid).
    // ──────────────────────────────────────────────────────────────────
    localparam [1:0] R_IDLE = 2'd0, R_ISSUE = 2'd1, R_CAPT = 2'd2,
                     R_DATA = 2'd3;
    reg [1:0]  rstate_q;
    reg [31:0] raddr_q;
    reg [7:0]  rleft_q;             // beats remaining after current
    reg [31:0] rdata_q;

    assign s_arready = (rstate_q == R_IDLE);
    assign s_rresp   = 2'b00;
    assign s_rvalid  = (rstate_q == R_DATA);
    assign s_rlast   = (rstate_q == R_DATA) && (rleft_q == 8'd0);
    assign s_rdata   = rdata_q;

    wire [7:0] rd_reg = raddr_q[7:0] & 8'hFC;
    reg [31:0] rd_reg_value;
    always @(*) begin
        case (rd_reg)
            REG_LBA:    rd_reg_value = lba_q;
            REG_STATUS: rd_reg_value = {22'd0, init_error, card_ready,
                                        err_cause_q, 1'b0, error_q,
                                        done_sticky_q, writer_busy};
            REG_BLKCNT: rd_reg_value = {16'd0, blkcnt_q};
            REG_CRC32:  rd_reg_value = ~crc_q;
            REG_IDENT:  rd_reg_value = IDENT_VALUE;
            REG_CAPS:   rd_reg_value = CAPS_VALUE;
            default:    rd_reg_value = 32'h0000_0000;
        endcase
    end

    // Port A ownership: write beats win; the read FSM retries next cycle.
    wire rd_issue_stage = (rstate_q == R_ISSUE) && raddr_q[20] &&
                          !wr_beat_stage;
    always @(*) begin
        a_wen  = wr_beat_stage;
        a_be   = wr_beat_stage ? s_wstrb : 4'h0;
        a_addr = wr_beat_stage ? waddr_q[STAGE_AW+1:2]
                               : raddr_q[STAGE_AW+1:2];
    end

    // ──────────────────────────────────────────────────────────────────
    // Sequential
    // ──────────────────────────────────────────────────────────────────
    always @(posedge clk) begin
        if (rst) begin
            lba_q         <= 32'h0000_0000;
            blkcnt_q      <= 16'd1;
            done_sticky_q <= 1'b0;
            error_q       <= 1'b0;
            err_cause_q   <= 4'd0;
            crc_q         <= 32'hFFFF_FFFF;
            mode_write_q  <= 1'b1;
            start_q       <= 1'b0;
            ctrl_go_q     <= 1'b0;
            bidx_q        <= {BIDX_W{1'b0}};
            blane_q       <= 2'd0;
            bpend0_q      <= 1'b0;
            bpend1_q      <= 1'b0;
            bdata_q       <= 8'hFF;
            b_addr        <= {STAGE_AW{1'b0}};
            wstate_q      <= W_IDLE;
            waddr_q       <= 32'h0000_0000;
            rstate_q      <= R_IDLE;
            raddr_q       <= 32'h0000_0000;
            rleft_q       <= 8'd0;
            rdata_q       <= 32'h0000_0000;
        end else begin
            ctrl_go_q <= 1'b0;
            if (start_q) start_q <= 1'b0;

            // ── AXI write burst FSM ────────────────────────────────────
            case (wstate_q)
                W_IDLE: begin
                    if (s_awvalid) begin
                        waddr_q  <= s_awaddr;
                        wstate_q <= W_DATA;
                    end
                end
                W_DATA: begin
                    if (s_wvalid) begin
                        waddr_q <= waddr_q + 32'd4;
                        if (s_wlast) wstate_q <= W_RESP;
                    end
                end
                W_RESP: begin
                    if (s_bready) wstate_q <= W_IDLE;
                end
                default: wstate_q <= W_IDLE;
            endcase

            // ── Register writes (per accepted beat in the reg window) ──
            if (wr_beat_reg) begin
                case (wr_reg)
                    REG_LBA: begin
                        if (s_wstrb[0]) lba_q[7:0]   <= s_wdata[7:0];
                        if (s_wstrb[1]) lba_q[15:8]  <= s_wdata[15:8];
                        if (s_wstrb[2]) lba_q[23:16] <= s_wdata[23:16];
                        if (s_wstrb[3]) lba_q[31:24] <= s_wdata[31:24];
                    end
                    REG_BLKCNT: begin
                        if (s_wstrb[0]) blkcnt_q[7:0]  <= s_wdata[7:0];
                        if (s_wstrb[1]) blkcnt_q[15:8] <= s_wdata[15:8];
                    end
                    REG_CTRL: begin
                        if (((s_wdata == CTRL_GO_WRITE) ||
                             (s_wdata == CTRL_GO_VERIFY)) && idle) begin
                            if (!card_ready) begin
                                done_sticky_q <= 1'b1;
                                error_q       <= 1'b1;
                                err_cause_q   <= 4'hF;
                            end else if (!blkcnt_ok) begin
                                done_sticky_q <= 1'b1;
                                error_q       <= 1'b1;
                                err_cause_q   <= 4'hE;
                            end else begin
                                done_sticky_q <= 1'b0;
                                error_q       <= 1'b0;
                                err_cause_q   <= 4'd0;
                                crc_q         <= 32'hFFFF_FFFF;
                                mode_write_q  <= (s_wdata == CTRL_GO_WRITE);
                                ctrl_go_q     <= 1'b1;
                                start_q       <= 1'b1;
                                // Prime the byte prefetch at index 0.
                                bidx_q   <= {BIDX_W{1'b0}};
                                b_addr   <= {STAGE_AW{1'b0}};
                                blane_q  <= 2'd0;
                                bpend0_q <= 1'b1;
                            end
                        end
                    end
                    default: ;
                endcase
            end

            // ── AXI read burst FSM ─────────────────────────────────────
            case (rstate_q)
                R_IDLE: begin
                    if (s_arvalid) begin
                        raddr_q  <= s_araddr;
                        rleft_q  <= s_arlen;
                        rstate_q <= R_ISSUE;
                    end
                end
                R_ISSUE: begin
                    if (raddr_q[20]) begin
                        // Staging read: port A addr driven this cycle
                        // (unless a write beat owns it — retry).
                        if (rd_issue_stage) rstate_q <= R_CAPT;
                    end else begin
                        rdata_q  <= rd_reg_value;
                        rstate_q <= R_DATA;
                    end
                end
                R_CAPT: begin
                    // a_rdata now holds stage_ram[raddr]; capture it.
                    rdata_q  <= a_rdata;
                    rstate_q <= R_DATA;
                end
                R_DATA: begin
                    if (s_rready) begin
                        if (rleft_q == 8'd0) begin
                            rstate_q <= R_IDLE;
                        end else begin
                            rleft_q  <= rleft_q - 8'd1;
                            raddr_q  <= raddr_q + 32'd4;
                            rstate_q <= R_ISSUE;
                        end
                    end
                end
                default: rstate_q <= R_IDLE;
            endcase

            // ── SPI-stream byte prefetch + CRC (bulk ops) ──────────────
            bpend1_q <= bpend0_q;
            if (bpend0_q) bpend0_q <= 1'b0;
            if (bpend1_q) bdata_q <= b_rdata[{blane_q, 3'b000} +: 8];
            if (ctrl_wr_ready && mode_write_q) begin
                // sd_ctrl just consumed bdata_q — CRC it, prefetch next.
                crc_q    <= crc32_byte(crc_q, bdata_q);
                bidx_q   <= bidx_next;
                b_addr   <= bidx_next[STAGE_AW+1:2];
                blane_q  <= bidx_next[1:0];
                bpend0_q <= 1'b1;
            end
            if (ctrl_rd_valid && !mode_write_q) begin
                crc_q <= crc32_byte(crc_q, ctrl_rd_data);
            end

            if (ctrl_done) begin
                done_sticky_q <= 1'b1;
                error_q       <= ctrl_error;
                err_cause_q   <= ctrl_err_cause;
            end
        end
    end

    // ──────────────────────────────────────────────────────────────────
    // sd_ctrl — CMD25 (bulk write) / CMD18 (CRC verify) data transport.
    // ──────────────────────────────────────────────────────────────────
    sd_ctrl u_sd_ctrl (
        .clk           (clk),
        .rst           (rst),
        // Provisioning-only path — out of scope for CRC16 checking (its
        // own CMD18-verify op already CRC32-checks the read-back data
        // independently). CMD59 is never issued on this path either.
        .crc_check_en  (1'b0),
        .cmd_type      (mode_write_q ? CT_CMD25 : CT_CMD18),
        .lba           (lba_q),
        .block_count   (blkcnt_q),
        .go            (ctrl_go_q),
        .rd_valid      (ctrl_rd_valid),
        .rd_data       (ctrl_rd_data),
        .rd_ready      (1'b1),
        .wr_ready      (ctrl_wr_ready),
        .wr_valid      (ctrl_wr_valid_unused),
        .wr_data       (bdata_q),
        // wr_avail: this caller always has the byte staged before the
        // engine asks for it, so it takes the legacy unpaced write
        // stream unchanged.  See rtl/vhdd.vh / sd_ctrl.v header.
        .wr_avail      (1'b1),
        .spi_cmd_valid (spi_cmd_valid),
        .spi_cmd_ready (spi_cmd_ready),
        .spi_cmd_data  (spi_cmd_data),
        .spi_rsp_valid (spi_rsp_valid),
        .spi_rsp_data  (spi_rsp_data),
        .busy          (ctrl_busy),
        .done          (ctrl_done),
        .error         (ctrl_error),
        .err_cause     (ctrl_err_cause),
        .dbg_rd_crc_calc(/* unused */),
        .dbg_rd_crc_recv(/* unused */),
        .dbg_last_real_r1(/* unused */),
        .dbg_cur_cmd(/* unused */),
        .dbg_lba_lat(/* unused */),
        .dbg_last_crc7_sent(/* unused */),
        .dbg_last_write_resp(/* unused */),
        .dbg_block_idx(/* unused */),
        .dbg_last_poll_cnt(/* unused */)
    );

    // verilator lint_off UNUSED
    wire _unused = &{1'b0, s_awlen, ctrl_wr_valid_unused,
                     bidx_q[BIDX_W-1:BIDX_W-1],
                     waddr_q[31:21], waddr_q[19:8], waddr_q[1:0],
                     raddr_q[31:21], raddr_q[19:8], raddr_q[1:0], 1'b0};
    // verilator lint_on UNUSED

endmodule

`default_nettype wire
