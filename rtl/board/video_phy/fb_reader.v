// fb_reader.v -- Framebuffer read CDC bridge from scaler to VRAM.
// ---------------------------------------------------------------------------
// Sits between the HDMI pclk domain and the VRAM streaming read port
// (core/VRAM clock domain).  The scanout side presents an ordered stream of
// source-pixel addresses.  fb_reader crosses those requests into the VRAM
// clock domain, issues pipelined VRAM reads, then crosses the returned pixels
// back into pclk through a bounded response FIFO.
//
// CDC contract:
//   * s_rd_en/s_rd_addr are pclk-domain request valid/data signals.
//   * v_rd_en/v_rd_addr are vram_clk-domain signals and must feed a VRAM port
//     that returns v_rd_data/v_rd_valid in the same vram_clk domain.
//   * s_rd_ready is a pclk-domain backpressure hint for the request FIFO.
//     The scanout side may hold s_rd_en high while s_rd_ready is low, but
//     must keep s_rd_addr stable until the request is accepted.
//   * s_rd_data/s_rd_valid are a registered ordered pclk-domain response
//     stream.  Responses are not tied to a fixed pclk slot; the line-buffered
//     scanout writer consumes them as they arrive.
//   * HDMI timing never stalls.  Backpressure stalls are counted for
//     observability; visible underflow is detected by the line-buffered
//     scanout stage that knows whether the requested display line is ready.
//
// The local FIFO below mirrors the Gray-pointer async-FIFO pattern used by
// rtl/sys/async_fifo.v, but keeps the standalone video tb self-contained
// without changing the shared sys primitive or Makefile ownership.
// ---------------------------------------------------------------------------
`default_nettype none

module fb_reader #(
    parameter ADDR_W               = 20,   // 1024*768 = 786432 pixels < 2^20
    parameter DATA_W               = 24,   // RGB888 / 8bpp zero-padded upper bits
    parameter RETURN_LATENCY       = 31,
    // 256 entries (DEPTH_LOG2=8): plenty given the credit throttle below
    // caps outstanding requests to what the 64-line buffer + in-flight
    // XPM_READ_LATENCY window actually needs.  Was 2048 (DEPTH_LOG2=11)
    // — that size was never reachable in practice (the credit throttle
    // already bounds occupancy far below it) and, combined with the
    // asynchronous read below, forced Vivado into huge distributed
    // LUTRAM instead of block RAM (see fb_reader_cdc_fifo's mem[] read
    // comment).
    parameter REQ_FIFO_DEPTH_LOG2  = 8,
    parameter RSP_FIFO_DEPTH_LOG2  = 8,
    // ── Outstanding-request cap (the downstream port's ONLY protection) ──
    //
    // MAX_INFLIGHT bounds how many reads may be issued to the VRAM port but
    // not yet returned.  It is a HARD CORRECTNESS limit, not a tuning knob:
    // scanout_ddr_reader.v's request queue (2**QDEPTH_LOG2 = 64 entries)
    // has NO backpressure -- its own header states "every rd_en is accepted
    // (no backpressure, matching vram.v)" and names this module's credit
    // machinery as what keeps the caller inside the contract.  Overrun that
    // queue and its tail laps its head: pushes still land, older entries are
    // silently overwritten, and the ordered response stream desynchronises
    // from the request stream in multiples of the queue depth.  Downstream
    // of that, linebuf_scanout's response walk writes each row's bytes at
    // the wrong offsets -- a whole-frame source-origin DISPLACEMENT on a
    // completely static framebuffer, which is what it was measured doing
    // (tb-scanout-ddr-frames; hardware, 2026-08-01).
    //
    // Before this parameter existed the cap did not exist either.  The
    // rsp_future_full term below reads like one but is not: it compares
    // (rsp fill + in flight) against RSP_FIFO_DEPTH = 256, and in_flight_cnt
    // could not even COUNT that high, so the term was unreachable and the
    // number of requests in the downstream queue was bounded by nothing at
    // all.  Measured peak occupancy at the production geometry, with the
    // production parameters, was 1082 entries against a 64-entry queue.
    //
    // 48 leaves 16 entries of margin under that 64.  in_flight_cnt runs
    // about two ahead of the downstream queue's own count (the issue and
    // response pipes), so 48 in flight is <= 48 queued.
    //
    // It costs no bandwidth.  The downstream fetch engine prefetches from
    // the QUEUE HEAD (scanout_line_fetch.v), speculatively, up to
    // PREFETCH_DEPTH lines AHEAD of anything this module has even asked
    // for -- so its pipelining depends on the head advancing, not on how
    // many requests are queued behind it, and a DEEPER queue would not buy
    // it a single extra outstanding AXI burst.  (That was true when the
    // fetch engine was single-outstanding and is MORE true now that it is
    // not: the lookahead window, not the request backlog, is what keeps its
    // AXI pipeline full.)  48 still holds more than a full 128 B ring
    // line's worth of 4-byte-group requests (32).  What the cap changes is
    // only WHERE the backlog waits: in this module's 256-deep request FIFO
    // (which does backpressure, via s_rd_ready) instead of on the floor.
    //
    // WHY A CREDIT COUNTER CANNOT WEDGE HERE (this is a wiring requirement,
    // not an assumption -- check it if you re-wire the resets).  A cap on a
    // counter deadlocks if the counter can OVER-count, because then no
    // request is issued, so no response can arrive to decrement it.  There
    // are exactly two loss paths, and production closes both:
    //   * RESET.  fb_reader's vram_rst and scanout_ddr_reader's rst are the
    //     SAME net (fpga_top_video.vh's vram_rd_rst drives both), so a reset
    //     clears the downstream queue and this counter together.
    //   * rsp_full.  The response FIFO is 256 deep and drained every pclk;
    //     with the cap at 48 it cannot fill from in-flight traffic, and the
    //     only stall in that drain is p_domain_rst -- which forces
    //     vram_domain_rst, i.e. case 1.
    // The zero-clamp on the decrement below covers the opposite direction.
    parameter MAX_INFLIGHT         = 48,
    // Width of in_flight_cnt.  Must be wide enough to hold MAX_INFLIGHT --
    // a too-narrow counter WRAPS, and a wrapped credit counter reads as
    // "nothing outstanding" exactly when the most is.
    parameter INFLIGHT_WIDTH       = 6
) (
    input  wire              pclk,
    input  wire              resetn,

    input  wire              vram_clk,
    input  wire              vram_rst,

    // ── From scaler (pclk) ───────────────────────────────────────────
    input  wire              s_rd_en,
    input  wire [ADDR_W-1:0] s_rd_addr,
    output wire              s_rd_ready,
    output reg  [DATA_W-1:0] s_rd_data,
    output reg               s_rd_valid,

    // ── To VRAM module (vram_clk/core clock) ─────────────────────────
    output reg               v_rd_en,
    output reg  [ADDR_W-1:0] v_rd_addr,
    input  wire [DATA_W-1:0] v_rd_data,
    input  wire              v_rd_valid,
    output wire              underflow_sticky,
    output wire [15:0]       req_count,
    output wire [15:0]       rsp_count,
    output wire [15:0]       miss_count
);

    wire p_rst = ~resetn;
    (* ASYNC_REG = "TRUE" *) reg [1:0] p_rst_vram_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] vram_rst_pclk_sync;
    // 3-stage synchronizer for the overflow toggle.  XOR-ing a 2-stage
    // sync's [1]^[0] compares the settled stage against the STILL-
    // METASTABLE first stage — that first stage can glitch mid-settle
    // and produce a spurious toggle-edge detection (or miss a real one).
    // A 3rd stage gives stage [1] a full cycle to resolve before it is
    // compared against the (by-then-settled) stage [2].
    (* ASYNC_REG = "TRUE" *) reg [2:0] rsp_overflow_toggle_sync;
    reg                           rsp_overflow_toggle_vram;
    wire p_rst_vram = p_rst_vram_sync[1];
    wire vram_rst_pclk = vram_rst_pclk_sync[1];
    wire p_domain_rst = p_rst || vram_rst_pclk;
    wire vram_domain_rst = vram_rst || p_rst_vram;
    wire rsp_overflow_event = rsp_overflow_toggle_sync[2] ^
                              rsp_overflow_toggle_sync[1];

    // Keep both clock domains in reset coherently whenever either local
    // reset source fires.  This avoids one-sided FIFO pointer resets when
    // MMCM lock drops on the pclk side.
    always @(posedge vram_clk) begin
        if (vram_rst) begin
            p_rst_vram_sync <= 2'b11;
        end else begin
            p_rst_vram_sync <= {p_rst_vram_sync[0], p_rst};
        end
    end

    always @(posedge pclk) begin
        if (p_rst) begin
            vram_rst_pclk_sync <= 2'b11;
            rsp_overflow_toggle_sync <= 3'b000;
        end else begin
            vram_rst_pclk_sync <= {vram_rst_pclk_sync[0], vram_rst};
            rsp_overflow_toggle_sync <= {rsp_overflow_toggle_sync[1:0],
                                         rsp_overflow_toggle_vram};
        end
    end

    reg               rsp_wr_en;
    reg  [DATA_W-1:0] rsp_wr_data;
    wire              rsp_full;
    wire              rsp_empty;
    wire [DATA_W-1:0] rsp_rd_data;

    // ── pclk -> vram_clk request FIFO ────────────────────────────────
    wire              req_full;
    wire              req_empty;
    wire [ADDR_W-1:0] req_rd_addr;
    assign s_rd_ready = !req_full && !p_domain_rst;
    wire              req_wr_en = s_rd_en && s_rd_ready;
    reg               req_pipe_valid;
    reg  [ADDR_W-1:0] req_pipe_addr;
    // Credit-based throttle: issue v_rd_en only when (current rsp fill +
    // outstanding in-flight reads) will still fit in rsp once they all
    // land.  Without this, any stall in the pclk-side drain lets rsp
    // fill up and the 6 in-flight XPM reads overflow when they return.
    //
    // rsp_fill_vram is rebuilt in the vram_clk domain from rsp's gray
    // pointers (wr_bin is native wclk; rd_bin_wclk is gray-synced from
    // pclk).  in_flight_cnt counts v_rd_en asserted but not yet seen
    // via v_rd_valid.
    localparam integer RSP_FIFO_DEPTH  = (1 << RSP_FIFO_DEPTH_LOG2);
    // A cap the counter cannot represent is not a cap.
    generate
        if (MAX_INFLIGHT >= (1 << INFLIGHT_WIDTH)) begin : gen_inflight_bad
            initial begin
                $error("[fb_reader] MAX_INFLIGHT=%0d does not fit INFLIGHT_WIDTH=%0d bits -- in_flight_cnt would wrap and the cap would never fire.",
                       MAX_INFLIGHT, INFLIGHT_WIDTH);
            end
        end
    endgenerate
    wire [RSP_FIFO_DEPTH_LOG2:0] rsp_wr_bin;
    wire [RSP_FIFO_DEPTH_LOG2:0] rsp_rd_bin_vram;
    wire [RSP_FIFO_DEPTH_LOG2:0] rsp_fill_vram =
        rsp_wr_bin - rsp_rd_bin_vram;
    reg  [INFLIGHT_WIDTH-1:0]    in_flight_cnt;
    wire [RSP_FIFO_DEPTH_LOG2+1:0] rsp_future_fill =
        {1'b0, rsp_fill_vram} + {{(RSP_FIFO_DEPTH_LOG2-INFLIGHT_WIDTH+2){1'b0}}, in_flight_cnt};
    wire rsp_future_full = (rsp_future_fill >= RSP_FIFO_DEPTH);
    // The real cap (see MAX_INFLIGHT's declaration).  `>=`, not `==`: an
    // equality test only holds a counter that lands exactly on the bound.
    wire inflight_full = (in_flight_cnt >= MAX_INFLIGHT[INFLIGHT_WIDTH-1:0]);
    wire              req_pipe_issue = req_pipe_valid && !rsp_future_full
                                     && !inflight_full && !vram_domain_rst;
    wire              req_rd_en = !vram_domain_rst && !req_empty
                                && (!req_pipe_valid || req_pipe_issue);

    fb_reader_cdc_fifo #(
        .WIDTH      (ADDR_W),
        .DEPTH_LOG2 (REQ_FIFO_DEPTH_LOG2)
    ) u_req_fifo (
        .wclk     (pclk),
        .wrst     (p_domain_rst),
        .wr_en    (req_wr_en),
        .wr_data  (s_rd_addr),
        .wr_full  (req_full),
        .rclk     (vram_clk),
        .rrst     (vram_domain_rst),
        .rd_en    (req_rd_en),
        .rd_data  (req_rd_addr),
        .rd_empty (req_empty),
        .wr_bin     (),
        .rd_bin_wclk()
    );

    // ── vram_clk -> pclk response FIFO ───────────────────────────────
    reg [15:0]             req_count_r;
    reg [15:0]             rsp_count_r;
    reg [15:0]             miss_count_r;
    reg                    underflow_sticky_r;
    wire rsp_rd_en = !rsp_empty && !p_domain_rst;
    assign underflow_sticky = underflow_sticky_r;
    assign req_count = req_count_r;
    assign rsp_count = rsp_count_r;
    assign miss_count = miss_count_r;

    fb_reader_cdc_fifo #(
        .WIDTH      (DATA_W),
        .DEPTH_LOG2 (RSP_FIFO_DEPTH_LOG2)
    ) u_rsp_fifo (
        .wclk     (vram_clk),
        .wrst     (vram_domain_rst),
        .wr_en    (rsp_wr_en && !rsp_full),
        .wr_data  (rsp_wr_data),
        .wr_full  (rsp_full),
        .rclk     (pclk),
        .rrst     (p_domain_rst),
        .rd_en    (rsp_rd_en),
        .rd_data  (rsp_rd_data),
        .rd_empty (rsp_empty),
        .wr_bin     (rsp_wr_bin),
        .rd_bin_wclk(rsp_rd_bin_vram)
    );

    // ── VRAM-side read issuer ────────────────────────────────────────
    // Stage the request FIFO output before driving the VRAM read port.  This
    // keeps the async FIFO RAM output off the outward v_rd_addr/v_rd_en path
    // while preserving one request per vram_clk once the stage is primed.
    always @(posedge vram_clk) begin
        if (vram_domain_rst) begin
            v_rd_en        <= 1'b0;
            v_rd_addr      <= {ADDR_W{1'b0}};
            req_pipe_valid <= 1'b0;
            req_pipe_addr  <= {ADDR_W{1'b0}};
            rsp_wr_en      <= 1'b0;
            rsp_wr_data    <= {DATA_W{1'b0}};
            rsp_overflow_toggle_vram <= 1'b0;
            in_flight_cnt  <= {INFLIGHT_WIDTH{1'b0}};
        end else begin
            v_rd_en   <= req_pipe_issue;
            rsp_wr_en <= 1'b0;

            if (req_pipe_issue) begin
                v_rd_addr <= req_pipe_addr;
            end

            if (req_rd_en) begin
                req_pipe_valid <= 1'b1;
                req_pipe_addr  <= req_rd_addr;
            end else if (req_pipe_issue) begin
                req_pipe_valid <= 1'b0;
            end

            if (v_rd_valid) begin
                if (!rsp_full) begin
                    rsp_wr_en   <= 1'b1;
                    rsp_wr_data <= v_rd_data;
                end else begin
                    rsp_overflow_toggle_vram <= ~rsp_overflow_toggle_vram;
                end
            end

            // in_flight_cnt tracks v_rd_en asserted but not yet returned
            // via v_rd_valid.  Increment on issue, decrement on return.
            case ({req_pipe_issue, v_rd_valid})
                2'b10: in_flight_cnt <= in_flight_cnt + {{(INFLIGHT_WIDTH-1){1'b0}}, 1'b1};
                // CLAMPED AT ZERO, for the same reason scanout_fetch.v clamps
                // its own `outstanding`: an unmatched response would wrap the
                // counter to its maximum, and a credit counter pinned at its
                // maximum reads as "no credit" forever.
                2'b01: if (in_flight_cnt != {INFLIGHT_WIDTH{1'b0}})
                           in_flight_cnt <= in_flight_cnt - {{(INFLIGHT_WIDTH-1){1'b0}}, 1'b1};
                default: in_flight_cnt <= in_flight_cnt;
            endcase
        end
    end

    // ── pclk-side ordered response consumer ──────────────────────────
    always @(posedge pclk) begin
        if (p_domain_rst) begin
            s_rd_data    <= {DATA_W{1'b0}};
            s_rd_valid   <= 1'b0;
            req_count_r  <= 16'd0;
            rsp_count_r  <= 16'd0;
            miss_count_r <= 16'd0;
            underflow_sticky_r <= 1'b0;
        end else begin
            if (req_wr_en)
                req_count_r <= req_count_r + 16'd1;
            if (rsp_rd_en)
                rsp_count_r <= rsp_count_r + 16'd1;
            if (rsp_overflow_event)
                underflow_sticky_r <= 1'b1;

            if (rsp_rd_en) begin
                s_rd_data  <= rsp_rd_data;
                s_rd_valid <= 1'b1;
            end else begin
                s_rd_data  <= {DATA_W{1'b0}};
                s_rd_valid <= 1'b0;
            end

            if (s_rd_en && !s_rd_ready) begin
                miss_count_r <= miss_count_r + 16'd1;
            end
        end
    end

`ifdef VERILATOR
    reg              stall_active;
    reg [ADDR_W-1:0] stall_addr;

    always @(posedge pclk) begin
        if (!resetn) begin
            stall_active <= 1'b0;
            stall_addr   <= {ADDR_W{1'b0}};
        end else if (s_rd_en && !s_rd_ready) begin
            if (stall_active && (s_rd_addr != stall_addr)) begin
                $fatal(1, "fb_reader request changed while stalled");
            end
            stall_active <= 1'b1;
            stall_addr   <= s_rd_addr;
        end else begin
            stall_active <= 1'b0;
        end
    end

    always @(posedge vram_clk) begin
        if (!vram_rst && v_rd_valid && rsp_full) begin
            $fatal(1, "fb_reader response FIFO overflow");
        end
    end

`endif

endmodule

// Small dual-clock FIFO for fb_reader request/response streams.
module fb_reader_cdc_fifo #(
    parameter WIDTH      = 24,
    parameter DEPTH_LOG2 = 11
) (
    input  wire             wclk,
    input  wire             wrst,
    input  wire             wr_en,
    input  wire [WIDTH-1:0] wr_data,
    output wire             wr_full,

    input  wire             rclk,
    input  wire             rrst,
    input  wire             rd_en,
    output wire [WIDTH-1:0] rd_data,
    output wire             rd_empty,

    // Observability ports: gray-to-binary decoded pointers.  wr_bin is
    // the write-count (wclk-domain).  rd_bin_wclk is the read-count
    // reconstructed in the wclk domain via gray-sync, for computing a
    // conservative fill estimate on the write side.  These outputs are
    // optional — pin-connect-empty tolerated at callers that don't need
    // fill observability.
    output wire [DEPTH_LOG2:0] wr_bin,
    output wire [DEPTH_LOG2:0] rd_bin_wclk
);

    localparam DEPTH = (1 << DEPTH_LOG2);
    localparam PW    = DEPTH_LOG2 + 1;

    // NOTE: the `ram_style = "block"` hint below does NOT actually win
    // BRAM here — `rd_data` (further down) is an ASYNCHRONOUS/
    // combinational read off `mem[rbin[...]]`, and Xilinx BRAM primitives
    // only support a REGISTERED (synchronous) read port.  Vivado silently
    // falls back to distributed LUTRAM for this array regardless of the
    // attribute.  At the old DEPTH_LOG2=11 (2048 x 24/20-bit entries)
    // that was a large, unnecessary LUTRAM footprint on the pclk timing
    // path; reducing DEPTH_LOG2 to 8 (256 entries — this module's credit
    // throttle + the 64-line scan-out buffer never approach that
    // occupancy) shrinks it to something reasonable without needing to
    // restructure the read into a synchronous one (a separate,
    // structural fix — see rtl/board/async_fifo.v, owned elsewhere).
    (* ram_style = "block" *) reg [WIDTH-1:0] mem [0:DEPTH-1];

    reg [PW-1:0] wbin;
    reg [PW-1:0] wgray;
    reg [PW-1:0] rbin;
    reg [PW-1:0] rgray;

    (* ASYNC_REG = "TRUE" *) reg [PW-1:0] rgray_w1;
    (* ASYNC_REG = "TRUE" *) reg [PW-1:0] rgray_w2;
    (* ASYNC_REG = "TRUE" *) reg [PW-1:0] wgray_r1;
    (* ASYNC_REG = "TRUE" *) reg [PW-1:0] wgray_r2;

    function [PW-1:0] bin2gray;
        input [PW-1:0] b;
        begin
            bin2gray = b ^ (b >> 1);
        end
    endfunction

    wire [PW-1:0] full_rgray = {~rgray_w2[PW-1:PW-2], rgray_w2[PW-3:0]};
    assign wr_full  = wrst || (wgray == full_rgray);
    assign rd_empty = rrst || (rgray == wgray_r2);

    wire wr_accept = wr_en && !wr_full;
    wire rd_accept = rd_en && !rd_empty;

    always @(posedge wclk) begin
        if (wrst) begin
            wbin     <= {PW{1'b0}};
            wgray    <= {PW{1'b0}};
            rgray_w1 <= {PW{1'b0}};
            rgray_w2 <= {PW{1'b0}};
        end else begin
            rgray_w1 <= rgray;
            rgray_w2 <= rgray_w1;
            if (wr_accept) begin
                mem[wbin[DEPTH_LOG2-1:0]] <= wr_data;
                wbin  <= wbin + {{(PW-1){1'b0}}, 1'b1};
                wgray <= bin2gray(wbin + {{(PW-1){1'b0}}, 1'b1});
            end
        end
    end

    always @(posedge rclk) begin
        if (rrst) begin
            rbin     <= {PW{1'b0}};
            rgray    <= {PW{1'b0}};
            wgray_r1 <= {PW{1'b0}};
            wgray_r2 <= {PW{1'b0}};
        end else begin
            wgray_r1 <= wgray;
            wgray_r2 <= wgray_r1;
            if (rd_accept) begin
                rbin  <= rbin + {{(PW-1){1'b0}}, 1'b1};
                rgray <= bin2gray(rbin + {{(PW-1){1'b0}}, 1'b1});
            end
        end
    end

    assign rd_data = mem[rbin[DEPTH_LOG2-1:0]];

    // Gray-to-binary conversion: for a gray-coded counter, bit i of the
    // binary value is the XOR of gray bits [MSB..i].  We use this to
    // reconstruct the read-side count in the write clock domain.
    function [PW-1:0] gray2bin;
        input [PW-1:0] g;
        integer i;
        begin
            gray2bin[PW-1] = g[PW-1];
            for (i = PW-2; i >= 0; i = i - 1)
                gray2bin[i] = gray2bin[i+1] ^ g[i];
        end
    endfunction

    assign wr_bin      = wbin;
    assign rd_bin_wclk = gray2bin(rgray_w2);

endmodule

`default_nettype wire
