// tb_scsi_dual.v — two SCSI targets, one FSM, two vhdd providers.
//
// What this harness proves
// ════════════════════════
// scsi.v now answers to TWO target IDs from a single back-end FSM,
// latching `vh_dev_sel` at the instant a selection succeeds.  vhdd_mux.v
// turns that bit into a route between two independent providers.  This
// harness wires the whole chain up with two BEHAVIOURAL vhdd providers
// whose contents differ in every single byte, so a routing bug cannot
// alias into a pass:
//
//   provider A: block N, byte i  ==  8'hA0 ^ (N + i)
//   provider B: block N, byte i  ==  8'hB0 ^ (N + i)
//
// 0xA0 and 0xB0 differ in bit 4 and the XOR preserves that, so EVERY
// byte of A differs from the same byte of B.
//
//   ┌─────────┐   vh_*   ┌──────────┐  a_*  ┌───────────────┐
//   │ scsi.v  ├─────────►│ vhdd_mux ├──────►│ model (SIG A0)│
//   │ ID 0/1  │ dev_sel  │          ├──────►│ model (SIG B0)│
//   └─────────┘          └──────────┘  b_*  └───────────────┘
//
// `dev_en` is a harness INPUT so tb_scsi_dual.cpp can take a target off
// the bus at runtime (dev_en=2'b01 / 2'b10) without a second build.
//
// `probe_dev`/`probe_addr`/`probe_data` expose each model's memory
// directly, so the cross-talk assertion ("writing target B did not touch
// target A's block 3") reads A's storage rather than trusting a second
// SCSI round trip through the same routing logic it is testing.
//
// TURBOSCSI_C96 is 0 here, matching tb_scsi.cpp's build of
// tb_scsi_vhdd_sd: the selection path under test is the bare-5380
// S_BUS_FREE one.  (The two C96 selection sites take the same
// sel_id_hit/sel_id_which pair; they are covered by the existing
// tb-scsi-c96-* family staying green with TARGET_B_EN=0.)

`default_nettype none

`include "vhdd.vh"

// ══════════════════════════════════════════════════════════════════════
// Behavioural vhdd provider — 64 blocks of 512 B, contract-complete.
//
// Implements rtl/vhdd.vh in full: the combinational extent probe, the
// busy/done/error handshake, both byte streams with real rd_ready
// back-pressure, and — per the header's load-bearing clause 2 — its own
// bounded-response watchdog, so a bug upstream turns into `done|error`
// rather than a hung simulation.
// ══════════════════════════════════════════════════════════════════════
module tb_vhdd_model #(
    // High nibble of the data signature.  Two instances with different
    // SIG values differ in every byte of every block.
    parameter [7:0]  SIG        = 8'hA0,
    // Fixed at 64 blocks (32768 bytes) — the memory declaration below
    // is sized to match and is not parameterised.
    parameter [31:0] NBLOCKS    = 32'd64,
    // Per-request watchdog, in cycles.  Never fires in a healthy run;
    // it exists because rtl/vhdd.vh makes bounded response the
    // PROVIDER's obligation, not the master's.
    parameter [31:0] WDOG_LIMIT = 32'd500000,
    // Command turnaround, in cycles, before the byte stream starts.
    parameter [3:0]  TURNAROUND = 4'd2
) (
    input  wire        clk,
    input  wire        rst,

    output wire [31:0] num_lbas,

    input  wire [31:0] chk_lba,
    input  wire [23:0] chk_blocks,
    output wire        chk_ok,

    input  wire        req_write,
    input  wire        req_multi,
    input  wire [31:0] req_lba,
    input  wire [15:0] req_block_count,
    input  wire        req_go,

    output reg         busy,
    output reg         done,
    output reg         error,

    output reg         rd_valid,
    output reg  [7:0]  rd_data,
    input  wire        rd_ready,

    output reg         wr_ready,
    input  wire        wr_valid,
    input  wire [7:0]  wr_data,
    /* verilator lint_off UNUSEDSIGNAL */
    // This mock keeps its whole block buffered, so it never needs to
    // consult the master's wr_avail credit — declared for port parity
    // with the real providers.
    input  wire        wr_avail,
    /* verilator lint_on UNUSEDSIGNAL */

    // Direct storage probe (harness only — not part of the contract).
    input  wire [15:0] probe_addr,
    output wire [7:0]  probe_data
);
    localparam integer MEM_BYTES = 32768;   // 64 blocks * 512 B

    reg [7:0] mem [0:MEM_BYTES-1];

    integer blk, byt;
    reg [7:0] sum8;
    initial begin
        for (blk = 0; blk < 64; blk = blk + 1)
            for (byt = 0; byt < 512; byt = byt + 1) begin
                sum8 = blk[7:0] + byt[7:0];
                mem[blk*512 + byt] = SIG ^ sum8;
            end
    end

    assign num_lbas   = NBLOCKS;
    assign probe_data = mem[probe_addr[14:0]];

    // ── extent probe: combinational, no side effect ────────────────────
    // Fails closed on a zero-length extent, on running off the end of
    // the volume, and on 32-bit wrap of lba+blocks (33-bit sum).
    wire [32:0] chk_end = {1'b0, chk_lba} + {9'd0, chk_blocks};
    assign chk_ok = (chk_blocks != 24'd0) &&
                    (chk_end <= {1'b0, NBLOCKS});

    localparam [1:0] ST_IDLE = 2'd0,
                     ST_RD   = 2'd1,
                     ST_WR   = 2'd2,
                     ST_DONE = 2'd3;

    reg [1:0]  st;
    reg [31:0] cur_lba;
    reg [15:0] blocks_left;
    reg [9:0]  byte_idx;
    reg [3:0]  turn;
    reg [31:0] wdog;

    wire [14:0] cur_addr = {cur_lba[5:0], byte_idx[8:0]};
    wire        last_byte_of_req = (byte_idx == 10'd511) &&
                                   (blocks_left == 16'd1);

    always @(posedge clk) begin
        if (rst) begin
            st          <= ST_IDLE;
            busy        <= 1'b0;
            done        <= 1'b0;
            error       <= 1'b0;
            rd_valid    <= 1'b0;
            rd_data     <= 8'h00;
            wr_ready    <= 1'b0;
            cur_lba     <= 32'd0;
            blocks_left <= 16'd0;
            byte_idx    <= 10'd0;
            turn        <= 4'd0;
            wdog        <= 32'd0;
        end else begin
            done     <= 1'b0;
            rd_valid <= 1'b0;

            // Bounded response: armed at req_go, incremented while the
            // request is live, ungated on any transport progress.
            if (busy) wdog <= wdog + 32'd1;

            if (req_go) begin
                cur_lba     <= req_lba;
                blocks_left <= (req_block_count == 16'd0) ? 16'd1
                                                          : req_block_count;
                byte_idx    <= 10'd0;
                turn        <= TURNAROUND;
                busy        <= 1'b1;
                error       <= 1'b0;
                wdog        <= 32'd0;
                wr_ready    <= 1'b0;
                st          <= req_write ? ST_WR : ST_RD;
            end else if (busy && (wdog >= WDOG_LIMIT)) begin
                // Answer anyway, with error — never park in busy.
                busy     <= 1'b0;
                done     <= 1'b1;
                error    <= 1'b1;
                wr_ready <= 1'b0;
                st       <= ST_IDLE;
            end else begin
                case (st)
                    ST_RD: begin
                        if (turn != 4'd0) begin
                            turn <= turn - 4'd1;
                        end else if (rd_ready) begin
                            // Real back-pressure: no byte leaves the
                            // source while rd_ready is low.
                            rd_valid <= 1'b1;
                            rd_data  <= mem[cur_addr];
                            if (byte_idx == 10'd511) begin
                                byte_idx <= 10'd0;
                                if (blocks_left == 16'd1) begin
                                    st <= ST_DONE;
                                end else begin
                                    blocks_left <= blocks_left - 16'd1;
                                    cur_lba     <= cur_lba + 32'd1;
                                end
                            end else begin
                                byte_idx <= byte_idx + 10'd1;
                            end
                        end
                    end
                    ST_WR: begin
                        if (turn != 4'd0) begin
                            turn <= turn - 4'd1;
                        end else begin
                            // Producer-paced: the master presents
                            // wr_data on the cycle wr_ready is high, so
                            // sample it on the edge that ends that
                            // cycle.  wr_valid is informational only and
                            // is deliberately NOT gated on (vhdd.vh).
                            wr_ready <= 1'b1;
                            if (wr_ready) begin
                                mem[cur_addr] <= wr_data;
                                if (last_byte_of_req) begin
                                    wr_ready <= 1'b0;
                                    byte_idx <= 10'd0;
                                    st       <= ST_DONE;
                                end else if (byte_idx == 10'd511) begin
                                    byte_idx    <= 10'd0;
                                    blocks_left <= blocks_left - 16'd1;
                                    cur_lba     <= cur_lba + 32'd1;
                                end else begin
                                    byte_idx <= byte_idx + 10'd1;
                                end
                            end
                        end
                    end
                    ST_DONE: begin
                        busy     <= 1'b0;
                        done     <= 1'b1;
                        wr_ready <= 1'b0;
                        st       <= ST_IDLE;
                    end
                    default: begin
                        wr_ready <= 1'b0;
                    end
                endcase
            end
        end
    end
endmodule

// ══════════════════════════════════════════════════════════════════════
// Harness top
// ══════════════════════════════════════════════════════════════════════
module tb_scsi_dual (
    input  wire        clk,
    input  wire        rst,
    // Runtime target-enable — bit0 = ID 0 live, bit1 = ID 1 live.
    input  wire [1:0]  dev_en,
    // ── Peripheral-bus slave ──────────────────────────────────────────
    input  wire [8:0]  pb_addr,
    input  wire [7:0]  pb_wdata,
    input  wire        pb_wr,
    input  wire        pb_rd,
    output wire [7:0]  pb_rdata,
    output wire        pb_ack,
    output wire        irq,
    output wire        drq,
    // ── Direct provider-storage probe ─────────────────────────────────
    input  wire        probe_dev,          // 0 = provider A, 1 = provider B
    input  wire [15:0] probe_addr,
    output wire [7:0]  probe_data,
    // ── Observability ─────────────────────────────────────────────────
    output wire        dev_sel
);
    // ── master (scsi.v) face of the vhdd seam ─────────────────────────
    wire [31:0] m_num_lbas;
    wire [31:0] m_chk_lba;
    wire [23:0] m_chk_blocks;
    wire        m_chk_ok;
    wire        m_req_write;
    wire        m_req_multi;
    wire [31:0] m_req_lba;
    wire [15:0] m_req_block_count;
    wire        m_req_go;
    wire        m_busy;
    wire        m_done;
    wire        m_error;
    wire        m_rd_valid;
    wire [7:0]  m_rd_data;
    wire        m_rd_ready;
    wire        m_wr_ready;
    wire        m_wr_valid;
    wire        m_wr_avail;
    wire [7:0]  m_wr_data;
    wire        vh_dev_sel;

    assign dev_sel = vh_dev_sel;

    scsi #(
        .TARGET_ID     (3'd0),
        .TARGET_ID_B   (3'd1),
        .TARGET_B_EN   (1'b1),
        .TURBOSCSI_C96 (1'b0)
    ) u_scsi (
        // Write-protect is a vhdd_ctrl runtime setting (CTRL bit 2), not a
        // property of the SCSI target; these harnesses predate it and test
        // the writable behaviour, so tie it off.
        .wprot(1'b0),
        // debug-only observability (rtl/mac/scsi.v); unconnected here
        .dbg_chk_ok(), .dbg_xfer_blocks(), .dbg_xfer_lba(),
        .dbg_medium_not_present(), .dbg_sense_key(),
        .dbg_sense_asc(), .dbg_check_cond_count(),
        .dbg_c96_state(),
        .clk           (clk),
        .rst           (rst),
        .dev_en        (dev_en),
        .vh_num_lbas   (m_num_lbas),
        .pb_addr       (pb_addr),
        // No peripheral_bus in this harness, so nothing splits a host
        // 16-bit aperture access into two pb beats — every beat here is
        // a standalone access and performs its own DRQ check.
        .pb_dma16_lo_beat(1'b0),
        .pb_wdata      (pb_wdata),
        .pb_wr         (pb_wr),
        .pb_rd         (pb_rd),
        .pb_rdata      (pb_rdata),
        .pb_ack        (pb_ack),
        .irq           (irq),
        .drq           (drq),
        .vh_chk_lba        (m_chk_lba),
        .vh_chk_blocks     (m_chk_blocks),
        .vh_chk_ok         (m_chk_ok),
        .vh_req_write      (m_req_write),
        .vh_req_multi      (m_req_multi),
        .vh_req_lba        (m_req_lba),
        .vh_req_block_count(m_req_block_count),
        .vh_req_go         (m_req_go),
        .vh_busy           (m_busy),
        .vh_done           (m_done),
        .vh_error          (m_error),
        .vh_rd_valid       (m_rd_valid),
        .vh_rd_data        (m_rd_data),
        .vh_rd_ready       (m_rd_ready),
        .vh_wr_ready       (m_wr_ready),
        .vh_wr_valid       (m_wr_valid),
        .vh_wr_data        (m_wr_data),
        .vh_wr_avail       (m_wr_avail),
        .vh_dev_sel        (vh_dev_sel),
        .scsi_ctrl_in  (9'd0),
        .dma_rd_ready  (),
        .dma_wr_ready  ()
    );

    // ── provider A face ───────────────────────────────────────────────
    wire [31:0] a_num_lbas;
    wire [31:0] a_chk_lba;
    wire [23:0] a_chk_blocks;
    wire        a_chk_ok;
    wire        a_req_write;
    wire        a_req_multi;
    wire [31:0] a_req_lba;
    wire [15:0] a_req_block_count;
    wire        a_req_go;
    wire        a_busy;
    wire        a_done;
    wire        a_error;
    wire        a_rd_valid;
    wire [7:0]  a_rd_data;
    wire        a_rd_ready;
    wire        a_wr_ready;
    wire        a_wr_valid;
    wire        a_wr_avail;
    wire [7:0]  a_wr_data;

    // ── provider B face ───────────────────────────────────────────────
    wire [31:0] b_num_lbas;
    wire [31:0] b_chk_lba;
    wire [23:0] b_chk_blocks;
    wire        b_chk_ok;
    wire        b_req_write;
    wire        b_req_multi;
    wire [31:0] b_req_lba;
    wire [15:0] b_req_block_count;
    wire        b_req_go;
    wire        b_busy;
    wire        b_done;
    wire        b_error;
    wire        b_rd_valid;
    wire [7:0]  b_rd_data;
    wire        b_rd_ready;
    wire        b_wr_ready;
    wire        b_wr_valid;
    wire        b_wr_avail;
    wire [7:0]  b_wr_data;

    vhdd_mux u_vhdd_mux (
        .dev_sel           (vh_dev_sel),
        .m_num_lbas        (m_num_lbas),
        .m_chk_lba         (m_chk_lba),
        .m_chk_blocks      (m_chk_blocks),
        .m_chk_ok          (m_chk_ok),
        .m_req_write       (m_req_write),
        .m_req_multi       (m_req_multi),
        .m_req_lba         (m_req_lba),
        .m_req_block_count (m_req_block_count),
        .m_req_go          (m_req_go),
        .m_busy            (m_busy),
        .m_done            (m_done),
        .m_error           (m_error),
        .m_rd_valid        (m_rd_valid),
        .m_rd_data         (m_rd_data),
        .m_rd_ready        (m_rd_ready),
        .m_wr_ready        (m_wr_ready),
        .m_wr_valid        (m_wr_valid),
        .m_wr_data         (m_wr_data),
        .m_wr_avail        (m_wr_avail),

        .a_num_lbas        (a_num_lbas),
        .a_chk_lba         (a_chk_lba),
        .a_chk_blocks      (a_chk_blocks),
        .a_chk_ok          (a_chk_ok),
        .a_req_write       (a_req_write),
        .a_req_multi       (a_req_multi),
        .a_req_lba         (a_req_lba),
        .a_req_block_count (a_req_block_count),
        .a_req_go          (a_req_go),
        .a_busy            (a_busy),
        .a_done            (a_done),
        .a_error           (a_error),
        .a_rd_valid        (a_rd_valid),
        .a_rd_data         (a_rd_data),
        .a_rd_ready        (a_rd_ready),
        .a_wr_ready        (a_wr_ready),
        .a_wr_valid        (a_wr_valid),
        .a_wr_data         (a_wr_data),
        .a_wr_avail        (a_wr_avail),

        .b_num_lbas        (b_num_lbas),
        .b_chk_lba         (b_chk_lba),
        .b_chk_blocks      (b_chk_blocks),
        .b_chk_ok          (b_chk_ok),
        .b_req_write       (b_req_write),
        .b_req_multi       (b_req_multi),
        .b_req_lba         (b_req_lba),
        .b_req_block_count (b_req_block_count),
        .b_req_go          (b_req_go),
        .b_busy            (b_busy),
        .b_done            (b_done),
        .b_error           (b_error),
        .b_rd_valid        (b_rd_valid),
        .b_rd_data         (b_rd_data),
        .b_rd_ready        (b_rd_ready),
        .b_wr_ready        (b_wr_ready),
        .b_wr_valid        (b_wr_valid),
        .b_wr_data         (b_wr_data),
        .b_wr_avail        (b_wr_avail)
    );

    wire [7:0] probe_data_a;
    wire [7:0] probe_data_b;
    assign probe_data = probe_dev ? probe_data_b : probe_data_a;

    tb_vhdd_model #(.SIG(8'hA0)) u_model_a (
        .clk             (clk),
        .rst             (rst),
        .num_lbas        (a_num_lbas),
        .chk_lba         (a_chk_lba),
        .chk_blocks      (a_chk_blocks),
        .chk_ok          (a_chk_ok),
        .req_write       (a_req_write),
        .req_multi       (a_req_multi),
        .req_lba         (a_req_lba),
        .req_block_count (a_req_block_count),
        .req_go          (a_req_go),
        .busy            (a_busy),
        .done            (a_done),
        .error           (a_error),
        .rd_valid        (a_rd_valid),
        .rd_data         (a_rd_data),
        .rd_ready        (a_rd_ready),
        .wr_ready        (a_wr_ready),
        .wr_valid        (a_wr_valid),
        .wr_data         (a_wr_data),
        .wr_avail        (a_wr_avail),
        .probe_addr      (probe_addr),
        .probe_data      (probe_data_a)
    );

    // Provider B is deliberately SMALLER than A (16 vs A's default 64) so a
    // test can read an LBA that is valid for A and OUT OF RANGE for B.  That
    // is what catches the range check consulting the WRONG volume's capacity:
    // vhdd_mux.v:178 routes m_num_lbas by dev_sel, and vh_dev_sel is latched
    // at selection and HELD until the next successful selection.
    // NOTE: shrink B rather than grow A -- tb_vhdd_model's backing store is a
    // FIXED 32768 bytes (64 blocks), so raising NBLOCKS past 64 makes reads
    // WRAP and produce wrong data rather than exercising the range check.
    // Every existing ID-1 scenario uses LBA <= 6, so B=16 leaves them intact.
    tb_vhdd_model #(.SIG(8'hB0), .NBLOCKS(32'd16)) u_model_b (
        .clk             (clk),
        .rst             (rst),
        .num_lbas        (b_num_lbas),
        .chk_lba         (b_chk_lba),
        .chk_blocks      (b_chk_blocks),
        .chk_ok          (b_chk_ok),
        .req_write       (b_req_write),
        .req_multi       (b_req_multi),
        .req_lba         (b_req_lba),
        .req_block_count (b_req_block_count),
        .req_go          (b_req_go),
        .busy            (b_busy),
        .done            (b_done),
        .error           (b_error),
        .rd_valid        (b_rd_valid),
        .rd_data         (b_rd_data),
        .rd_ready        (b_rd_ready),
        .wr_ready        (b_wr_ready),
        .wr_valid        (b_wr_valid),
        .wr_data         (b_wr_data),
        .wr_avail        (b_wr_avail),
        .probe_addr      (probe_addr),
        .probe_data      (probe_data_b)
    );

endmodule

`default_nettype wire
