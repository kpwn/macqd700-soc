// sim_mig_backend.v — Minimal 256-bit AXI4 BRAM slave for simulation.
//
// PURPOSE
//   Behavioural stand-in for the Xilinx MC4 MIG IP that exposes a 256-bit
//   AXI4 slave at the MIG UI clock.  Used in the SIM_MIG_BRIDGE sim variant
//   so the full CPU → axi_xbar → ddr_ctrl → axi_async_bridge →
//   axi_ddr4_mig_bridge → THIS chain is exercised end-to-end in Verilator.
//   This module replaces the real `design_1_ddr4_0_1` MIG instance for sim
//   purposes, providing byte-accurate writes via wstrb.
//
//   The production sim path (SIM_MODEL without SIM_MIG_BRIDGE) bypasses
//   the bridge entirely and uses ddr_ctrl's 128-bit BRAM directly.  That
//   leaves an entire class of bugs (MIG-bridge byte-lane packing, narrow→
//   wide adapter interactions with the bridge, MIG-side wstrb routing)
//   completely uncovered.  SIM_MIG_BRIDGE closes that gap.
//
// CONTRACT
//   - 256-bit data, 32-bit wstrb (one bit per byte)
//   - 31-bit address (matches MIG IP)
//   - 1-bit ID (matches MIG IP)
//   - INCR bursts, awsize=5 (32 B/beat) only — matches what the bridge emits
//   - Single outstanding write; up to READ_QUEUE_DEPTH outstanding reads
//   - Read commands return as non-interleaved bursts in accepted order,
//     matching the production MIG configuration's single AXI ID
//   - cal_done held high after a short post-reset settle (sim-only stub)
//
// STORAGE
//   16 byte-wide BRAMs × 2 (32 byte lanes total).  Address takes addr[N:5]
//   (one MIG beat = 32 B → 5 LSBs are byte-within-beat, indexed by wstrb).
//   Storage sized for 64 MiB (RAM_BYTES from tb_fpga_top_rom).  Verilator
//   handles sparse fills as 0 by default.
//
// LIMITATIONS
//   - Does NOT model refresh cycles or bank/row state. Configurable first-
//     response latency models externally visible queuing delay, not a
//     physical DDR scheduler.
//   - Does NOT model the c0_ddr4_aresetn negative-reset polarity or the
//     dbg_bus, both of which are MIG-side-only signals.

`default_nettype none

/* verilator lint_off UNUSEDPARAM */
module sim_mig_backend #(
    parameter DATA_WIDTH = 256,
    parameter STRB_WIDTH = DATA_WIDTH/8,
    parameter ADDR_WIDTH = 31,
    parameter ID_WIDTH   = 1,
    parameter BEATS_LOG2 = 16,      // 2^16 × 32 B = 2 MiB default
    parameter CAL_CYCLES = 4,
    // Optional random command-side backpressure injection (T11: "extend
    // sim_mig_backend with configurable stall injection").  Off by
    // default -- STALL_ENABLE=0 is IDENTICAL behaviour to before this
    // parameter existed (awready/wready/arready reduce to the plain
    // state-machine conditions, zero extra logic in the default path).
    // When enabled, a free-running LFSR gates awready/wready/arready
    // roughly 1-in-4 cycles, modelling MIG UI command-side backpressure
    // for integration-level tests (e.g. tb-fpga-top-rom-mig) that want
    // more realism than the golden always-ready path below.  Does not
    // gate rvalid/bvalid -- accepted reads instead wait for the configured
    // latency and then return in order at one beat per cycle.
    parameter STALL_ENABLE      = 0,
    parameter [31:0] STALL_SEED = 32'hACE1_1234,
    parameter READ_LATENCY_BASE   = 0,
    parameter READ_LATENCY_JITTER = 0,
    parameter READ_QUEUE_DEPTH    = 8
) (
    input  wire                       clk,
    input  wire                       rst,
    output wire                       cal_done,

    input  wire [ID_WIDTH-1:0]        awid,
    input  wire [ADDR_WIDTH-1:0]      awaddr,
    input  wire [7:0]                 awlen,
    input  wire [2:0]                 awsize,
    input  wire [1:0]                 awburst,
    input  wire                       awvalid,
    output wire                       awready,

    input  wire [DATA_WIDTH-1:0]      wdata,
    input  wire [STRB_WIDTH-1:0]      wstrb,
    input  wire                       wlast,
    input  wire                       wvalid,
    output wire                       wready,

    output reg  [ID_WIDTH-1:0]        bid,
    output reg  [1:0]                 bresp,
    output reg                        bvalid,
    input  wire                       bready,

    input  wire [ID_WIDTH-1:0]        arid,
    input  wire [ADDR_WIDTH-1:0]      araddr,
    input  wire [7:0]                 arlen,
    input  wire [2:0]                 arsize,
    input  wire [1:0]                 arburst,
    input  wire                       arvalid,
    output wire                       arready,

    output reg  [ID_WIDTH-1:0]        rid,
    output reg  [DATA_WIDTH-1:0]      rdata,
    output reg  [1:0]                 rresp,
    output reg                        rlast,
    output reg                        rvalid,
    input  wire                       rready
);

    // ── Optional random stall injection (backpressure realism) ──────────
    // See STALL_ENABLE header note above.  Free-running 32-bit LFSR;
    // stall_now gates the three command-accept ready signals below.
    // Fully synthesises away (constant 0) when STALL_ENABLE=0, since
    // stall_lfsr then has no effect on any output.
    reg [31:0] stall_lfsr;
    wire       stall_lfsr_fb = stall_lfsr[31] ^ stall_lfsr[21] ^ stall_lfsr[1] ^ stall_lfsr[0];
    always @(posedge clk) begin
        if (rst) stall_lfsr <= (STALL_SEED == 32'b0) ? 32'hACE1_1234 : STALL_SEED;
        else     stall_lfsr <= {stall_lfsr[30:0], stall_lfsr_fb};
    end
    wire stall_now = (STALL_ENABLE != 0) && (stall_lfsr[1:0] == 2'b00);

    // ── Calibration stub: hold cal_done low for a few cycles after reset ──
    reg [3:0] cal_ctr;
    reg       cal_done_r;
    assign cal_done = cal_done_r;
    always @(posedge clk) begin
        if (rst) begin
            cal_ctr    <= 4'd0;
            cal_done_r <= 1'b0;
        end else if (!cal_done_r) begin
            if (cal_ctr >= CAL_CYCLES[3:0]) cal_done_r <= 1'b1;
            else                            cal_ctr    <= cal_ctr + 4'd1;
        end
    end

    // ── Per-byte BRAM lanes ────────────────────────────────────────────
    // 32 byte lanes, each 2^BEATS_LOG2 deep.  Inferred as BRAM by Verilator
    // (and by Vivado if this ever became a synth target — we only mean it
    // for sim).
    localparam DEPTH = 1 << BEATS_LOG2;
    reg [7:0] mem_lane [0:31][0:DEPTH-1];

    // Address-to-index — match ddr_ctrl's FPGA_ROM_SIM scheme.
    // Map:
    //   0x00000000..0x03FFFFFF → beats 0..0x1FFFFF (RAM, 64 MiB folded)
    //   0x40000000..0x404FFFFF → beats SIM_RAM_BEATS..+0x27FFF (ROM, 5 MiB)
    //   0x40500000..0x4FFFFFFF → folded into a[25:5] (legacy mirror)
    //   else: a[BEATS_LOG2+4:5]
    localparam SIM_RAM_BEATS = (32'h04000000) >> 5;  // 2 M beats (64 MiB)
    function automatic [BEATS_LOG2-1:0] addr_to_idx;
        input [31:0] a;
        begin
            if (a[31:24] == 8'h40 && a[23:20] < 4'h5)
                addr_to_idx = SIM_RAM_BEATS[BEATS_LOG2-1:0] + a[22:5];
            else
                addr_to_idx = a[BEATS_LOG2+4:5];
        end
    endfunction
    wire [BEATS_LOG2-1:0] aw_idx0 = addr_to_idx({1'b0, awaddr});
    wire [BEATS_LOG2-1:0] ar_idx0 = addr_to_idx({1'b0, araddr});

    // ── Write channel state machine ────────────────────────────────────
    localparam W_IDLE = 2'd0,
               W_BURST = 2'd1,
               W_RESP  = 2'd2;

    reg [1:0]              w_state;
    reg [ADDR_WIDTH-1:0]   w_addr;
    reg [7:0]              w_beats_left;
    reg [ID_WIDTH-1:0]     w_id;

    assign awready = (w_state == W_IDLE) && !stall_now;
    assign wready  = (w_state == W_BURST) && !stall_now;

    wire [BEATS_LOG2-1:0] w_idx = addr_to_idx({1'b0, w_addr});

    integer wi;
    always @(posedge clk) begin
        if (rst) begin
            w_state      <= W_IDLE;
            w_addr       <= {ADDR_WIDTH{1'b0}};
            w_beats_left <= 8'd0;
            w_id         <= {ID_WIDTH{1'b0}};
            bvalid       <= 1'b0;
            bresp        <= 2'b00;
            bid          <= {ID_WIDTH{1'b0}};
        end else begin
            case (w_state)
                W_IDLE: begin
                    if (awvalid && awready) begin
                        w_addr       <= awaddr;
                        w_beats_left <= awlen;
                        w_id         <= awid;
                        w_state      <= W_BURST;
                    end
                    if (bvalid && bready) bvalid <= 1'b0;
                end
                W_BURST: begin
                    if (wvalid && wready) begin
                        // Commit each byte.
                        //
                        // wstrb-drop hypothesis testing modes (mutually exclusive):
                        //
                        //   SIM_MIG_IGNORE_WSTRB        — drop ALL wstrb (write every byte)
                        //   SIM_MIG_DROP_HI_PER_NIBBLE  — drop wstrb bits 2,3,6,7,10,11,…
                        //                                  i.e. drop the high 2 bits of each
                        //                                  4-bit-aligned nibble group
                        //   SIM_MIG_DROP_UPPER_HALF     — drop mig_wstrb[31:16]
                        //   SIM_MIG_DROP_BIT3_PER_NIBBLE — drop only bit 3 of each nibble
                        //
                        // Default (no define): honour wstrb correctly — golden behaviour.
                        for (wi = 0; wi < 32; wi = wi + 1) begin
`ifdef SIM_MIG_IGNORE_WSTRB
                            mem_lane[wi][w_idx] <= wdata[wi*8 +: 8];
`elsif SIM_MIG_DROP_HI_PER_NIBBLE
                            // Drop bits where (wi & 2) != 0 — i.e. bits 2,3,6,7,...
                            if (wstrb[wi] || (wi[1] == 1'b1)) begin
                                mem_lane[wi][w_idx] <= wdata[wi*8 +: 8];
                            end
`elsif SIM_MIG_DROP_UPPER_HALF
                            if (wstrb[wi] || (wi >= 16)) begin
                                mem_lane[wi][w_idx] <= wdata[wi*8 +: 8];
                            end
`elsif SIM_MIG_DROP_BIT3_PER_NIBBLE
                            // Drop bits where (wi & 3) == 3 — i.e. bits 3,7,11,...
                            if (wstrb[wi] || (wi[1:0] == 2'b11)) begin
                                mem_lane[wi][w_idx] <= wdata[wi*8 +: 8];
                            end
`else
                            if (wstrb[wi]) begin
                                mem_lane[wi][w_idx] <= wdata[wi*8 +: 8];
                            end
`endif
                        end
                        if (wlast || w_beats_left == 8'd0) begin
                            // Burst complete — issue B response.
                            bvalid  <= 1'b1;
                            bresp   <= 2'b00;
                            bid     <= w_id;
                            w_state <= W_RESP;
                        end else begin
                            // Advance address by one beat (32 B for awsize=5).
                            w_addr       <= w_addr + (32'd1 << awsize);
                            w_beats_left <= w_beats_left - 8'd1;
                        end
                    end
                end
                W_RESP: begin
                    if (bvalid && bready) begin
                        bvalid  <= 1'b0;
                        w_state <= W_IDLE;
                    end
                end
                default: w_state <= W_IDLE;
            endcase
        end
    end

    // ── Pipelined read-command queue ─────────────────────────────────
    localparam READ_Q_PTR_W = 3; // READ_QUEUE_DEPTH is constrained to 8.
    reg [ADDR_WIDTH-1:0] r_q_addr  [0:READ_QUEUE_DEPTH-1];
    reg [7:0]            r_q_left  [0:READ_QUEUE_DEPTH-1];
    reg [2:0]            r_q_size  [0:READ_QUEUE_DEPTH-1];
    reg [ID_WIDTH-1:0]   r_q_id    [0:READ_QUEUE_DEPTH-1];
    reg [15:0]           r_q_delay [0:READ_QUEUE_DEPTH-1];
    reg                  r_q_valid [0:READ_QUEUE_DEPTH-1];
    reg [READ_Q_PTR_W-1:0] r_q_wptr, r_q_rptr;
    reg [READ_Q_PTR_W:0]   r_q_count;

    wire r_q_full  = (r_q_count == READ_QUEUE_DEPTH);
    wire r_q_empty = (r_q_count == 0);
    assign arready = !r_q_full && !stall_now;
    wire ar_fire = arvalid && arready;
    wire r_pop = rvalid && rready && rlast;
    wire [BEATS_LOG2-1:0] r_head_idx =
        addr_to_idx({1'b0, r_q_addr[r_q_rptr]});
    wire [15:0] r_new_delay = READ_LATENCY_BASE[15:0] +
        ((READ_LATENCY_JITTER > 0) ?
         (stall_lfsr[15:0] % READ_LATENCY_JITTER) : 16'd0);

    // synthesis translate_off
    initial begin
        if (READ_QUEUE_DEPTH != 8) begin
            $display("sim_mig_backend: READ_QUEUE_DEPTH must be 8");
            $fatal(1);
        end
        if (READ_LATENCY_BASE > 65535 || READ_LATENCY_JITTER > 65535) begin
            $display("sim_mig_backend: read latency exceeds the model's 16-bit range");
            $fatal(1);
        end
    end
    // synthesis translate_on

    integer ri, rq;
    always @(posedge clk) begin
        if (rst) begin
            r_q_wptr  <= {READ_Q_PTR_W{1'b0}};
            r_q_rptr  <= {READ_Q_PTR_W{1'b0}};
            r_q_count <= {(READ_Q_PTR_W+1){1'b0}};
            for (rq = 0; rq < READ_QUEUE_DEPTH; rq = rq + 1) begin
                r_q_valid[rq] <= 1'b0;
                r_q_delay[rq] <= 16'd0;
            end
            rvalid       <= 1'b0;
            rdata        <= {DATA_WIDTH{1'b0}};
            rresp        <= 2'b00;
            rlast        <= 1'b0;
            rid          <= {ID_WIDTH{1'b0}};
        end else begin
            for (rq = 0; rq < READ_QUEUE_DEPTH; rq = rq + 1) begin
                if (r_q_valid[rq] && r_q_delay[rq] != 16'd0)
                    r_q_delay[rq] <= r_q_delay[rq] - 16'd1;
            end

            if (ar_fire) begin
                r_q_addr[r_q_wptr]  <= araddr;
                r_q_left[r_q_wptr]  <= arlen;
                r_q_size[r_q_wptr]  <= arsize;
                r_q_id[r_q_wptr]    <= arid;
                r_q_delay[r_q_wptr] <= r_new_delay;
                r_q_valid[r_q_wptr] <= 1'b1;
                r_q_wptr            <= r_q_wptr + 1'b1;
            end

            // Once a burst starts, sustain one beat per cycle. r_q_addr
            // always points at the next beat to load into the output register.
            if (rvalid && rready) begin
                if (rlast) begin
                    rvalid <= 1'b0;
                    rlast  <= 1'b0;
                    r_q_valid[r_q_rptr] <= 1'b0;
                    r_q_rptr <= r_q_rptr + 1'b1;
                end else begin
                    for (ri = 0; ri < 32; ri = ri + 1)
                        rdata[ri*8 +: 8] <= mem_lane[ri][r_head_idx];
                    rid   <= r_q_id[r_q_rptr];
                    rresp <= 2'b00;
                    rlast <= (r_q_left[r_q_rptr] == 8'd0);
                    if (r_q_left[r_q_rptr] != 8'd0) begin
                        r_q_addr[r_q_rptr] <= r_q_addr[r_q_rptr] +
                                             ({{(ADDR_WIDTH-1){1'b0}},1'b1} << r_q_size[r_q_rptr]);
                        r_q_left[r_q_rptr] <= r_q_left[r_q_rptr] - 8'd1;
                    end
                end
            end else if (!rvalid && !r_q_empty && r_q_valid[r_q_rptr] &&
                         r_q_delay[r_q_rptr] == 16'd0) begin
                for (ri = 0; ri < 32; ri = ri + 1)
                    rdata[ri*8 +: 8] <= mem_lane[ri][r_head_idx];
                rid    <= r_q_id[r_q_rptr];
                rresp  <= 2'b00;
                rlast  <= (r_q_left[r_q_rptr] == 8'd0);
                rvalid <= 1'b1;
                if (r_q_left[r_q_rptr] != 8'd0) begin
                    r_q_addr[r_q_rptr] <= r_q_addr[r_q_rptr] +
                                         ({{(ADDR_WIDTH-1){1'b0}},1'b1} << r_q_size[r_q_rptr]);
                    r_q_left[r_q_rptr] <= r_q_left[r_q_rptr] - 8'd1;
                end
            end

            case ({ar_fire, r_pop})
                2'b10: r_q_count <= r_q_count + 1'b1;
                2'b01: r_q_count <= r_q_count - 1'b1;
                default: r_q_count <= r_q_count;
            endcase
        end
    end

endmodule
/* verilator lint_on UNUSEDPARAM */

`default_nettype wire
