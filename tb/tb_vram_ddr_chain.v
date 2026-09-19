// tb_vram_ddr_chain.v -- Verilator-top wrapper for the VRAM-in-DDR
// integration chain (T14, reshaped by T16 "decode-level VRAM lane"),
// mirroring tb_l2c_chain.v's structure/spirit but one seam further out:
//
//   CPU BFM (M0) -> axi_xbar (VRAM_IN_DDR) -> S3 (genuine slave again,
//        T16 revert of T14's S3->S0 fold) -> [addr-translate] ->
//        axi_vram_priority_mux3 (3-way: l2c-path <-> S3-lane <-> scan)
//        <- scanout_ddr_reader <- scan_rd_* BFM
//        -> axi_async_bridge -> axi_ddr4_mig_bridge -> sim_mig_backend
//
//   (l2c-path, when CHAIN_L2C_ENABLE: RAM/ROM/FB traffic only -- xbar S0
//    -> l2c -> mux's l2c_* port. VRAM-aperture CPU traffic NEVER reaches
//    l2c at all under this topology -- that is the whole point of T16's
//    reshape, "enforced at the decode level instead": see
//    axi_vram_priority_mux3.v's header and docs/l2c_spec.md's
//    invariants section.)
//
// This is the REAL production chain for CPU writes/reads to the VRAM
// pixel aperture under VRAM_IN_DDR: unlike a hand-rolled address-
// translation stand-in, the CPU-side BFM here drives axi_xbar's M0 port
// with the UNTRANSLATED 0xF900_0000-based aperture address, so
// axi_xbar.v's real (now unconditional, regardless of VRAM_IN_DDR) S3
// decode/zero-basing/byte-swap are exercised for real, not assumed.
//
// `CHAIN_L2C_ENABLE` (default 1) mirrors tb_l2c_chain.v's own parameter:
// selects whether l2c sits in the CPU path at all for RAM/ROM/FB traffic
// (1 = real l2c; 0 = xbar S0 -> mux's l2c_* port directly, mirroring
// fpga_top_ddr.vh's L2C_ENABLE-undefined wiring). VRAM-aperture traffic
// is unaffected by this parameter either way -- it always takes the S3
// lane, never l2c.
//
// `STALL_ENABLE`/`STALL_SEED` forward to sim_mig_backend's command-side
// backpressure injection, matching tb_l2c_chain.v.
//
// `CHAIN_VIDEO_SMOKE` (default 0): adds a real `vram_smoke` instance on
// the S3 lane, muxed in by the production `axi_vram_smoke_mux` exactly as
// fpga_top_ddr.vh wires it -- the CPU-less video-test rig
// (VIDEO_SMOKE=1 + VRAM_IN_DDR).  Driven by tb_video_smoke_ddr.cpp; see
// that file's header for what it proves and for the deliberately-broken
// `CHAIN_SMOKE_BROKEN_SWAP` negative control.  At the default 0 there is
// no smoke instance and the mux degenerates to the pure S3 pass-through
// + carveout translate that the inline s3lane_awaddr/araddr wires used to
// be, so tb_vram_ddr_chain.cpp's scenarios are unaffected.
//
// `slv_flush` (T16, new port): forwarded to axi_xbar's own `slv_flush`
// input (tied 0 previously) so a directed scenario can exercise the
// xbar's existing per-slave flush-domain machinery on S3 (S3 has always
// been a member of `is_flush_domain_slv()` -- unaffected by T16's revert
// of the S3->S0 decode fold, which only touched decode_slv()/
// ddr_flatten(), not the flush-domain membership function) while an S3
// op is in flight through the VRAM lane, proving the xbar's existing
// poison/unroutability machinery drops the late backend response cleanly
// with no wedge -- see the `s3_flush_abort_no_wedge` scenario.
//
// Only M0 (CPU LSU) is driven by the top-level BFM; M1 (host debug), the
// boot-FSM fan-in (m0b_*), and M2 (CPU IF) are tied permanently inactive
// -- this tb is scoped to the VRAM-aperture CPU-write/scanout-read
// concurrency question, not general xbar arbitration (already covered by
// tb-axi-xbar). S1/S2/S4/S5 (peripheral/DMA-cfg/DAFB/SD-JTAG) are tied
// off as inert always-ready sinks -- the BFM traffic generator never
// targets those windows.
//
// Verilog-2005, two independent clk domains (core_clk / mig_clk), sync
// active-high resets in each domain, matching tb_l2c_chain.v's contract.

`default_nettype none

`ifdef AXI_VRAM_MUX3_DIRECT_TB

module tb_axi_vram_priority_mux3;
    localparam ID_WIDTH = 2;
    localparam DATA_WIDTH = 32;
    localparam MAX_SCAN_AHEAD = 4;
    // Deliberately DIFFERENT numbers from the production defaults (4/8) so
    // the two caps cannot be confused for one another in a failure message,
    // and so the adaptive scenario below proves the mechanism rather than a
    // coincidence of equal values.
    localparam MAX_BULK_ACTIVE = 2;
    localparam MAX_BULK_QUIET  = 4;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = !clk;

    reg [ID_WIDTH-1:0] l2c_awid;
    reg [31:0] l2c_awaddr;
    reg l2c_awvalid;
    wire l2c_awready;
    reg [DATA_WIDTH-1:0] l2c_wdata;
    reg l2c_wvalid;
    wire l2c_wready;
    wire [ID_WIDTH-1:0] l2c_bid;
    wire l2c_bvalid;
    reg l2c_bready;

    reg [ID_WIDTH-1:0] s3_awid;
    reg [31:0] s3_awaddr;
    reg s3_awvalid;
    wire s3_awready;
    reg [DATA_WIDTH-1:0] s3_wdata;
    reg s3_wvalid;
    wire s3_wready;
    wire [ID_WIDTH-1:0] s3_bid;
    wire s3_bvalid;
    reg s3_bready;

    reg [ID_WIDTH-1:0] l2c_arid;
    reg [31:0] l2c_araddr;
    reg l2c_arvalid;
    wire l2c_arready;
    wire [ID_WIDTH-1:0] l2c_rid;
    wire [DATA_WIDTH-1:0] l2c_rdata;
    wire l2c_rlast, l2c_rvalid;
    reg l2c_rready;

    reg [ID_WIDTH-1:0] s3_arid;
    reg [31:0] s3_araddr;
    reg s3_arvalid;
    wire s3_arready;
    wire [ID_WIDTH-1:0] s3_rid;
    wire [DATA_WIDTH-1:0] s3_rdata;
    wire s3_rlast, s3_rvalid;
    reg s3_rready;

    reg [ID_WIDTH-1:0] scan_arid;
    reg [31:0] scan_araddr;
    reg scan_arvalid;
    wire scan_arready;
    wire [ID_WIDTH-1:0] scan_rid;
    wire [DATA_WIDTH-1:0] scan_rdata;
    wire scan_rlast, scan_rvalid;
    reg scan_rready;

    wire [ID_WIDTH-1:0] m_awid;
    wire [31:0] m_awaddr;
    wire m_awvalid;
    reg m_awready;
    wire [DATA_WIDTH-1:0] m_wdata;
    wire m_wvalid;
    reg m_wready;
    reg [ID_WIDTH-1:0] m_bid;
    reg [1:0] m_bresp;
    reg m_bvalid;
    wire m_bready;

    wire [ID_WIDTH-1:0] m_arid;
    wire [31:0] m_araddr;
    wire m_arvalid;
    reg m_arready;
    reg [ID_WIDTH-1:0] m_rid;
    reg [DATA_WIDTH-1:0] m_rdata;
    reg [1:0] m_rresp;
    reg m_rlast, m_rvalid;
    wire m_rready;

    axi_vram_priority_mux3 #(
        .ID_WIDTH(ID_WIDTH), .ADDR_WIDTH(32), .DATA_WIDTH(DATA_WIDTH),
        .MAX_BULK_AHEAD(MAX_BULK_ACTIVE),
        .MAX_BULK_AHEAD_QUIET(MAX_BULK_QUIET),
        .SCAN_RESERVE(MAX_SCAN_AHEAD),
        .MAX_SCAN_AHEAD(MAX_SCAN_AHEAD)
    ) dut (
        .clk(clk), .rst(rst),
        .l2c_awid(l2c_awid), .l2c_awaddr(l2c_awaddr), .l2c_awlen(8'd0),
        .l2c_awsize(3'd2), .l2c_awburst(2'd1),
        .l2c_awvalid(l2c_awvalid), .l2c_awready(l2c_awready),
        .l2c_wdata(l2c_wdata), .l2c_wstrb(4'hf), .l2c_wlast(1'b1),
        .l2c_wvalid(l2c_wvalid), .l2c_wready(l2c_wready),
        .l2c_bid(l2c_bid), .l2c_bresp(), .l2c_bvalid(l2c_bvalid), .l2c_bready(l2c_bready),
        .l2c_arid(l2c_arid), .l2c_araddr(l2c_araddr), .l2c_arlen(8'd0),
        .l2c_arsize(3'd2), .l2c_arburst(2'd1),
        .l2c_arvalid(l2c_arvalid), .l2c_arready(l2c_arready),
        .l2c_rid(l2c_rid), .l2c_rdata(l2c_rdata), .l2c_rresp(),
        .l2c_rlast(l2c_rlast), .l2c_rvalid(l2c_rvalid), .l2c_rready(l2c_rready),
        .s3_awid(s3_awid), .s3_awaddr(s3_awaddr), .s3_awlen(8'd0),
        .s3_awsize(3'd2), .s3_awburst(2'd1),
        .s3_awvalid(s3_awvalid), .s3_awready(s3_awready),
        .s3_wdata(s3_wdata), .s3_wstrb(4'hf), .s3_wlast(1'b1),
        .s3_wvalid(s3_wvalid), .s3_wready(s3_wready),
        .s3_bid(s3_bid), .s3_bresp(), .s3_bvalid(s3_bvalid), .s3_bready(s3_bready),
        .s3_arid(s3_arid), .s3_araddr(s3_araddr), .s3_arlen(8'd0),
        .s3_arsize(3'd2), .s3_arburst(2'd1),
        .s3_arvalid(s3_arvalid), .s3_arready(s3_arready),
        .s3_rid(s3_rid), .s3_rdata(s3_rdata), .s3_rresp(),
        .s3_rlast(s3_rlast), .s3_rvalid(s3_rvalid), .s3_rready(s3_rready),
        .scan_arid(scan_arid), .scan_araddr(scan_araddr), .scan_arlen(8'd0),
        .scan_arsize(3'd2), .scan_arburst(2'd1),
        .scan_arvalid(scan_arvalid), .scan_arready(scan_arready),
        .scan_rid(scan_rid), .scan_rdata(scan_rdata), .scan_rresp(),
        .scan_rlast(scan_rlast), .scan_rvalid(scan_rvalid), .scan_rready(scan_rready),
        .m_awid(m_awid), .m_awaddr(m_awaddr), .m_awlen(), .m_awsize(),
        .m_awburst(), .m_awvalid(m_awvalid), .m_awready(m_awready),
        .m_wdata(m_wdata), .m_wstrb(), .m_wlast(), .m_wvalid(m_wvalid), .m_wready(m_wready),
        .m_bid(m_bid), .m_bresp(m_bresp), .m_bvalid(m_bvalid), .m_bready(m_bready),
        .m_arid(m_arid), .m_araddr(m_araddr), .m_arlen(), .m_arsize(),
        .m_arburst(), .m_arvalid(m_arvalid), .m_arready(m_arready),
        .m_rid(m_rid), .m_rdata(m_rdata), .m_rresp(m_rresp),
        .m_rlast(m_rlast), .m_rvalid(m_rvalid), .m_rready(m_rready)
    );

    integer failures;
    integer cycle_count;
    integer q_wr, q_rd;
    integer q_due [0:4095];
    integer q_accept_cycle [0:4095];
    reg [1:0] q_src [0:4095];
    reg [ID_WIDTH-1:0] q_id [0:4095];
    reg [31:0] q_addr [0:4095];
    integer accepted_l2c, accepted_s3, accepted_scan;
    integer returned_l2c, returned_s3, returned_scan;
    integer scan_streak, max_scan_streak;
    integer bulk_streak, max_bulk_streak;
    integer max_bulk_count;
    integer min_response_latency, max_response_latency;
    integer response_route_errors;
    reg read_backend_enable;

    task automatic check(input bit condition, input string name);
        begin
            if (!condition) begin
                $display("  FAIL %s at cycle %0d", name, cycle_count);
                failures = failures + 1;
            end
        end
    endtask

    task automatic step;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    task automatic clear_sources;
        begin
            l2c_awid = 0; l2c_awaddr = 0; l2c_awvalid = 0;
            l2c_wdata = 0; l2c_wvalid = 0; l2c_bready = 1;
            s3_awid = 0; s3_awaddr = 0; s3_awvalid = 0;
            s3_wdata = 0; s3_wvalid = 0; s3_bready = 1;
            l2c_arid = 0; l2c_araddr = 32'h1000; l2c_arvalid = 0; l2c_rready = 1;
            s3_arid = 1; s3_araddr = 32'h2000; s3_arvalid = 0; s3_rready = 1;
            scan_arid = 2; scan_araddr = 32'h3000; scan_arvalid = 0; scan_rready = 1;
            m_awready = 1; m_wready = 1;
            m_bid = 0; m_bresp = 0; m_bvalid = 0;
        end
    endtask

    task automatic apply_reset;
        begin
            rst = 1;
            repeat (3) step();
            rst = 0;
            step();
            check(dut.aw_rr === 1'b0, "aw_rr reset value");
            check(dut.ar_rr === 1'b0, "ar_rr reset value");
            check(dut.aw_busy === 1'b0 && dut.aw_open === 1'b0,
                  "write ownership reset state");
            check(dut.rdq_count == 0 && dut.bulk_count == 0 &&
                  dut.scan_ahead_count == 0, "read arbitration reset state");
        end
    endtask

    // In-order DDR stand-in with independent randomized AR acceptance and
    // response latency.  It records the accepted source so every returned
    // beat checks the mux's route FIFO, ID, and data ownership.
    always @(posedge clk) begin
        if (rst || !read_backend_enable) begin
            cycle_count <= 0;
            q_wr <= 0; q_rd <= 0;
            m_arready <= 0; m_rvalid <= 0; m_rlast <= 1;
            m_rid <= 0; m_rdata <= 0; m_rresp <= 0;
            accepted_l2c <= 0; accepted_s3 <= 0; accepted_scan <= 0;
            returned_l2c <= 0; returned_s3 <= 0; returned_scan <= 0;
            scan_streak <= 0; max_scan_streak <= 0;
            bulk_streak <= 0; max_bulk_streak <= 0;
            max_bulk_count <= 0;
            min_response_latency <= 32'h7fff_ffff;
            max_response_latency <= 0;
            response_route_errors <= 0;
        end else begin
            cycle_count <= cycle_count + 1;
            m_arready <= ($urandom_range(0, 3) != 0);
            if (dut.bulk_count > max_bulk_count)
                max_bulk_count <= dut.bulk_count;

            if (m_arvalid && m_arready) begin
                q_id[q_wr] <= m_arid;
                q_addr[q_wr] <= m_araddr;
                // Approximate a loaded DDR path: 200-cycle base latency
                // plus a uniformly distributed 0..63-cycle component.
                q_due[q_wr] <= cycle_count + 200 + $urandom_range(0, 63);
                q_accept_cycle[q_wr] <= cycle_count;
                if (scan_arready) begin
                    q_src[q_wr] <= 2;
                    accepted_scan <= accepted_scan + 1;
                    scan_streak <= scan_streak + 1;
                    bulk_streak <= 0;
                    if (scan_streak + 1 > max_scan_streak)
                        max_scan_streak <= scan_streak + 1;
                end else if (s3_arready) begin
                    q_src[q_wr] <= 1;
                    accepted_s3 <= accepted_s3 + 1;
                    scan_streak <= 0;
                    bulk_streak <= bulk_streak + 1;
                    if (bulk_streak + 1 > max_bulk_streak)
                        max_bulk_streak <= bulk_streak + 1;
                end else begin
                    q_src[q_wr] <= 0;
                    accepted_l2c <= accepted_l2c + 1;
                    scan_streak <= 0;
                    bulk_streak <= bulk_streak + 1;
                    if (bulk_streak + 1 > max_bulk_streak)
                        max_bulk_streak <= bulk_streak + 1;
                end
                q_wr <= q_wr + 1;
            end

            if (!m_rvalid && q_rd < q_wr && q_due[q_rd] <= cycle_count) begin
                m_rvalid <= 1;
                m_rlast <= 1;
                m_rid <= q_id[q_rd];
                m_rdata <= 32'hc000_0000 ^ q_addr[q_rd];
                m_rresp <= 0;
            end

            if (m_rvalid) begin
                case (q_src[q_rd])
                    0: if (!l2c_rvalid || s3_rvalid || scan_rvalid ||
                           l2c_rid != q_id[q_rd] ||
                           l2c_rdata != (32'hc000_0000 ^ q_addr[q_rd]))
                           response_route_errors <= response_route_errors + 1;
                    1: if (l2c_rvalid || !s3_rvalid || scan_rvalid ||
                           s3_rid != q_id[q_rd] ||
                           s3_rdata != (32'hc000_0000 ^ q_addr[q_rd]))
                           response_route_errors <= response_route_errors + 1;
                    2: if (l2c_rvalid || s3_rvalid || !scan_rvalid ||
                           scan_rid != q_id[q_rd] ||
                           scan_rdata != (32'hc000_0000 ^ q_addr[q_rd]))
                           response_route_errors <= response_route_errors + 1;
                endcase
            end

            if (m_rvalid && m_rready) begin
                if (cycle_count - q_accept_cycle[q_rd] < min_response_latency)
                    min_response_latency <= cycle_count - q_accept_cycle[q_rd];
                if (cycle_count - q_accept_cycle[q_rd] > max_response_latency)
                    max_response_latency <= cycle_count - q_accept_cycle[q_rd];
                case (q_src[q_rd])
                    0: returned_l2c <= returned_l2c + 1;
                    1: returned_s3 <= returned_s3 + 1;
                    2: returned_scan <= returned_scan + 1;
                endcase
                q_rd <= q_rd + 1;
                m_rvalid <= 0;
            end
        end
    end

    task automatic finish_write_response(input bit owner_s3, input [ID_WIDTH-1:0] id);
        begin
            m_bid = id;
            m_bvalid = 1;
            #1;
            check(owner_s3 ? (s3_bvalid && !l2c_bvalid) :
                             (l2c_bvalid && !s3_bvalid),
                  "B response routed only to AW owner");
            check(owner_s3 ? (s3_bid == id) : (l2c_bid == id),
                  "B response ID preserved");
            step();
            m_bvalid = 0;
            #1;
        end
    endtask

    task automatic test_write_skew_and_ownership;
        begin
            $display("[TEST] AW/W skew, simultaneous writers, and B ownership");
            clear_sources(); read_backend_enable = 0; apply_reset();

            // W may lead AW by an arbitrary interval, but must see no READY
            // and must never become visible downstream.
            l2c_wdata = 32'h1111_aaaa;
            l2c_wvalid = 1;
            repeat (6) begin
                #1;
                check(!m_wvalid && !l2c_wready && !s3_wready,
                      "early W blocked before any AW owner");
                step();
            end

            // Establish an S3 owner while the unrelated early L2 W remains.
            s3_awid = 2; s3_awaddr = 32'h2000_0040; s3_awvalid = 1;
            s3_wdata = 32'h3333_bbbb; s3_wvalid = 1;
            #1;
            check(m_awvalid && m_awaddr == s3_awaddr && !m_wvalid,
                  "AW selected before either W is routed");
            step();
            s3_awvalid = 0;
            #1;
            check(m_wvalid && m_wdata == s3_wdata && s3_wready && !l2c_wready,
                  "W follows established S3 AW owner");
            step();
            s3_wvalid = 0; l2c_wvalid = 0;
            finish_write_response(1, 2);

            // Clean reset makes L2 the first RR winner. Both W channels are
            // asserted with both AW channels to stress simultaneous arrival.
            clear_sources(); apply_reset();
            m_awready = 0;
            l2c_awid = 1; l2c_awaddr = 32'h1000_0010; l2c_awvalid = 1;
            s3_awid = 2; s3_awaddr = 32'h2000_0020; s3_awvalid = 1;
            l2c_wdata = 32'haaaa_0001; l2c_wvalid = 1;
            s3_wdata = 32'hbbbb_0002; s3_wvalid = 1;
            #1;
            check(m_awvalid && m_awaddr == l2c_awaddr &&
                  !l2c_awready && !s3_awready,
                  "simultaneous AW uses reset RR state");
            check(!m_wvalid && !l2c_wready && !s3_wready,
                  "simultaneous W waits for AW handshake");
            repeat (4) begin
                step();
                check(m_awvalid && m_awaddr == l2c_awaddr &&
                      !l2c_awready && !s3_awready,
                      "AW owner and payload locked through backpressure");
                check(!m_wvalid, "W remains blocked while selected AW is stalled");
            end
            m_awready = 1;
            step();
            l2c_awvalid = 0;
            m_wready = 0;
            #1;
            check(m_wvalid && m_wdata == l2c_wdata && !l2c_wready && !s3_wready,
                  "L2 W ownership retained through W backpressure");
            repeat (3) step();
            check(m_wvalid && m_wdata == l2c_wdata,
                  "W payload stable through backpressure");
            m_wready = 1;
            step(); l2c_wvalid = 0;

            // Hold B until the owner is ready; no second AW may leak through.
            l2c_bready = 0; m_bid = 1; m_bvalid = 1;
            #1;
            check(l2c_bvalid && !s3_bvalid && !m_bready,
                  "B backpressure remains with L2 owner");
            check(!s3_awready, "second AW blocked until first B completes");
            repeat (3) step();
            l2c_bready = 1; step(); m_bvalid = 0;
            #1;
            check(m_awvalid && m_awaddr == s3_awaddr && s3_awready,
                  "RR selects waiting S3 AW after L2 response");
            step(); s3_awvalid = 0;
            #1;
            check(m_wvalid && m_wdata == s3_wdata && s3_wready && !l2c_wready,
                  "S3 W owns second simultaneous transaction");
            step(); s3_wvalid = 0;
            finish_write_response(1, 2);

            // AW may also lead W by a long interval.
            l2c_awid = 3; l2c_awaddr = 32'h1000_0080; l2c_awvalid = 1;
            #1; check(m_awvalid, "delayed-W AW presented");
            step(); l2c_awvalid = 0;
            repeat (9) begin
                #1; check(!m_wvalid, "no fabricated W while owner waits");
                step();
            end
            l2c_wdata = 32'hdead_beef; l2c_wvalid = 1;
            #1; check(m_wvalid && m_wdata == 32'hdead_beef,
                     "late W routed to retained owner");
            step(); l2c_wvalid = 0;
            finish_write_response(0, 3);
        end
    endtask

    task automatic test_write_reset;
        begin
            $display("[TEST] reset with open write ownership");
            clear_sources(); read_backend_enable = 0; apply_reset();
            s3_awid = 2; s3_awaddr = 32'h2000_0100; s3_awvalid = 1;
            step(); s3_awvalid = 0;
            check(dut.aw_open && dut.aw_grant, "S3 AW owner open before reset");
            rst = 1;
            repeat (3) step();
            check(dut.aw_open && dut.aw_grant && dut.aw_rr == 0,
                  "open owner retained while arbitration RR resets");
            rst = 0; step();
            s3_wdata = 32'h55aa_33cc; s3_wvalid = 1;
            #1; check(m_wvalid && s3_wready && !l2c_wready,
                     "post-reset W returns to retained S3 owner");
            step(); s3_wvalid = 0;
            finish_write_response(1, 2);
            check(!dut.aw_busy && !dut.aw_open, "late B releases reset-held owner");

            // Also reset after W, before a deliberately delayed B.
            l2c_awid = 1; l2c_awaddr = 32'h1000_0200; l2c_awvalid = 1;
            step(); l2c_awvalid = 0;
            l2c_wdata = 32'h1234_5678; l2c_wvalid = 1;
            step(); l2c_wvalid = 0;
            rst = 1; repeat (2) step(); rst = 0; repeat (5) step();
            m_bid = 1; m_bvalid = 1;
            #1; check(l2c_bvalid && !s3_bvalid,
                     "B after reset remains with pre-reset L2 owner");
            step(); m_bvalid = 0;
            check(!dut.aw_busy && !dut.aw_open, "post-W reset-held owner released");
        end
    endtask

    task automatic drain_reads(input integer bulk_bound);
        integer guard;
        begin
            l2c_arvalid = 0; s3_arvalid = 0; scan_arvalid = 0;
            guard = 10000;
            while ((q_rd != q_wr || m_rvalid || dut.rdq_count != 0) && guard > 0) begin
                step();
                guard = guard - 1;
            end
            check(guard > 0, "random-latency read queue drained");
            check(response_route_errors == 0, "all read responses routed to accepted owner");
            check(returned_l2c == accepted_l2c && returned_s3 == accepted_s3 &&
                  returned_scan == accepted_scan, "all accepted reads received one response");
            check(max_bulk_count <= bulk_bound,
                  "bulk admission bound remains enforced");
            check(max_response_latency > min_response_latency && min_response_latency >= 200,
                  "response model exercised randomized latency");
        end
    endtask

    task automatic test_read_contention(input integer mode, input string name);
        integer guard;
        begin
            $display("[TEST] %s", name);
            clear_sources(); read_backend_enable = 1; apply_reset();
            scan_arvalid = 1;
            l2c_arvalid = (mode == 0 || mode == 2);
            s3_arvalid = (mode == 1 || mode == 2);
            guard = 20000;
            while ((((mode == 0) && accepted_l2c < 24) ||
                    ((mode == 1) && accepted_s3 < 24) ||
                    ((mode == 2) && (accepted_l2c < 16 || accepted_s3 < 16))) &&
                   guard > 0) begin
                step();
                guard = guard - 1;
            end
            check(guard > 0, "continuous scan contention made bounded bulk progress");
            check(accepted_scan > 24, "continuous scan made bounded progress");
            check(max_scan_streak <= MAX_SCAN_AHEAD,
                  "scan quota bounds delay of pending bulk reader");
            check(max_bulk_streak <= 1,
                  "forced bulk service does not break scanout guarantee");
            if (mode == 2) begin
                check(accepted_l2c > 0 && accepted_s3 > 0,
                      "simultaneous L2 and S3 readers both served");
                check((accepted_l2c - accepted_s3 <= 1) &&
                      (accepted_s3 - accepted_l2c <= 1),
                      "simultaneous bulk readers remain round-robin");
            end
            drain_reads(MAX_BULK_ACTIVE);
        end
    endtask

    // ── Adaptive bulk cap ────────────────────────────────────────────
    // The three test_read_contention scenarios hold `scan_arvalid` high for
    // their whole duration, so the arbiter is NEVER in its quiet state in
    // any of them -- the wider window is invisible to them by construction.
    // This scenario is the one that exercises it, and the one that pins the
    // two properties that make it safe:
    //   (a) while scanout is quiet the CPU may fill the window to
    //       MAX_BULK_AHEAD_QUIET (that is the whole point -- scanout's
    //       reservation should not be charged to the CPU during the long
    //       gaps between scanout's refill excursions);
    //   (b) the instant scanout has anything pending, NO FURTHER bulk is
    //       admitted past MAX_BULK_AHEAD.  The convoy already accepted is
    //       non-preemptible (AXI), so it drains, but it cannot grow.
    // (b) is checked on EVERY cycle of the excursion, not sampled at the
    // end: a cap that leaks for a few cycles is exactly the kind of defect
    // that shows up on hardware as an occasional torn frame and never in a
    // steady-state test.
    task automatic test_adaptive_bulk_cap;
        integer guard;
        integer scan_at_start;
        begin
            $display("[TEST] adaptive bulk cap (quiet CPU window vs. scan reservation)");
            clear_sources(); read_backend_enable = 1; apply_reset();
            l2c_arvalid = 1; scan_arvalid = 0;
            guard = 6000;
            while (dut.bulk_count < MAX_BULK_QUIET && guard > 0) begin
                step(); guard = guard - 1;
            end
            check(guard > 0, "bulk reaches the quiet-state window while scanout is idle");
            check(dut.bulk_count == MAX_BULK_QUIET,
                  "quiet-state window admits exactly MAX_BULK_AHEAD_QUIET bursts");

            scan_at_start = accepted_scan;
            scan_arvalid = 1;
            guard = 6000;
            while ((accepted_scan - scan_at_start) < 4 && guard > 0) begin
                step();
                check(!(dut.bulk_admit && (dut.bulk_count >= MAX_BULK_ACTIVE)),
                      "no bulk admitted past MAX_BULK_AHEAD while scanout is active");
                guard = guard - 1;
            end
            check(guard > 0, "scanout is served despite the pre-existing bulk convoy");

            // ...and the wider window comes back when the excursion ends.
            scan_arvalid = 0;
            guard = 6000;
            while (dut.bulk_count < MAX_BULK_QUIET && guard > 0) begin
                step(); guard = guard - 1;
            end
            check(guard > 0, "quiet-state window is restored after the excursion ends");
            drain_reads(MAX_BULK_QUIET);
        end
    endtask

    initial begin
        failures = 0;
        read_backend_enable = 0;
        clear_sources();
        test_write_skew_and_ownership();
        test_write_reset();
        test_adaptive_bulk_cap();
        test_read_contention(0, "continuous scan versus L2 reader");
        test_read_contention(1, "continuous scan versus S3 VRAM reader");
        test_read_contention(2, "continuous scan versus simultaneous L2/S3 readers");
        if (failures == 0)
            $display("tb_axi_vram_priority_mux3: PASS");
        else
            $fatal(1, "tb_axi_vram_priority_mux3: FAIL (%0d checks)", failures);
        $finish;
    end
endmodule

`else

module tb_vram_ddr_chain #(
    parameter CHAIN_L2C_ENABLE = 1,
    parameter STALL_ENABLE     = 0,
    parameter [31:0] STALL_SEED = 32'hACE1_1234,
    parameter DDR_READ_LATENCY_BASE = 200,
    parameter DDR_READ_LATENCY_JITTER = 64,
    // ── CPU-less video-smoke rig (fpga_top VIDEO_SMOKE=1 under
    //    VRAM_IN_DDR) ────────────────────────────────────────────────
    // 0 (default) = no vram_smoke instance at all; axi_vram_smoke_mux
    // degenerates to the pure S3 pass-through + carveout translate the
    // inline s3lane_awaddr/araddr wires used to be, so the existing
    // tb-vram-ddr-chain / -nol2c scenarios are unaffected.
    parameter CHAIN_VIDEO_SMOKE = 0,
    // Smoke frame geometry.  Deliberately tiny so a whole frame paints in
    // a few thousand cycles.  MUST be a power of two (vram_smoke.v's
    // synthesis note) -- and the checker's golden model assumes it.
    parameter CHAIN_SMOKE_W     = 64,
    parameter CHAIN_SMOKE_H     = 24,
    parameter CHAIN_SMOKE_ROW_BAND_LOG2 = 0,
    // NEGATIVE CONTROL (see tb_video_smoke_ddr.cpp).  1 = byte-swap the
    // smoke writer's wdata within each 32-bit group before it reaches the
    // mux, i.e. deliberately land smoke on the WRONG side of axi_xbar.v's
    // S3 vram_swap_words.  The checker MUST fail when this is set; that is
    // what proves it can detect a wrong pattern and not merely a missing
    // one.  Never set in a production build -- this parameter exists only
    // in the testbench wrapper, not in fpga_top.
    parameter CHAIN_SMOKE_BROKEN_SWAP = 0
) (
    input  wire core_clk, core_rst,
    input  wire mig_clk,  mig_rst,
    input  wire slv_flush,
    output wire smoke_done,

    // ── CPU BFM -- axi_xbar M0 (full AXI4, untranslated system address) ──
    input  wire [3:0]   cpu_awid, input wire [31:0] cpu_awaddr,
    input  wire [7:0]   cpu_awlen, input wire [2:0] cpu_awsize, input wire [1:0] cpu_awburst,
    input  wire cpu_awvalid, output wire cpu_awready,
    input  wire [127:0] cpu_wdata, input wire [15:0] cpu_wstrb,
    input  wire cpu_wlast, input wire cpu_wvalid, output wire cpu_wready,
    output wire [3:0] cpu_bid, output wire [1:0] cpu_bresp,
    output wire cpu_bvalid, input wire cpu_bready,
    input  wire [3:0] cpu_arid, input wire [31:0] cpu_araddr,
    input  wire [7:0] cpu_arlen, input wire [2:0] cpu_arsize, input wire [1:0] cpu_arburst,
    input  wire cpu_arvalid, output wire cpu_arready,
    output wire [3:0] cpu_rid, output wire [127:0] cpu_rdata,
    output wire [1:0] cpu_rresp, output wire cpu_rlast, output wire cpu_rvalid, input wire cpu_rready,

    // ── scanout BFM -- scanout_ddr_reader's streaming pixel port ─────────
    input  wire        scan_rd_en,
    input  wire [20:0] scan_rd_addr,
    // 4-byte group starting at scan_rd_addr: [31:24] is the byte at
    // scan_rd_addr (valid at any alignment), [23:0] the bytes at +1/+2/+3
    // (architecturally valid only when scan_rd_addr[1:0]==0).
    output wire [31:0] scan_rd_data,
    output wire        scan_rd_valid,

    // Test-only direct bulk-read source at mux3's l2c ingress.  This bypasses
    // the single-master xbar serialization so the arbiter's two-deep bulk
    // admission contract can be saturated deterministically.
    input  wire        test_bulk_enable,
    input  wire        test_bulk_arvalid,
    input  wire [31:0] test_bulk_araddr,
    output wire        test_bulk_arready,
    output wire        test_bulk_rvalid,
    output wire        test_bulk_rlast,
    input  wire        test_bulk_rready,

    output wire cal_done,

    // ── Debug-only observability (T14 bring-up; hierarchical refs into
    //    u_scan are fine for a testbench-only wire) ──────────────────────
    output wire [5:0] dbg_q_head, dbg_q_tail,
    output wire [6:0] dbg_q_count,
    output wire [1:0] dbg_f_state,
    output wire [3:0] dbg_mux_bulk_count,
    output wire [3:0] dbg_mux_rdq_count,
    // The mux's CONFIGURED non-scan admission bound, exported so the C++
    // scenarios assert the invariant ("no more than the configured number
    // of bulk bursts may be queued ahead of a scan request") instead of a
    // literal 2 that silently stops testing anything the moment the
    // parameter is retuned.
    output wire [3:0] dbg_mux_max_bulk_ahead,
    // ...and the QUIET cap, which is the bound that actually applies to a
    // convoy already accepted before scanout asked for anything.
    output wire [3:0] dbg_mux_max_bulk_quiet,
    output wire       dbg_scan_ar_fire,
    output wire [31:0] dbg_l2_miss_count,
    output wire [3:0]  dbg_l2_mshr_occupancy,
    // "the L2 victim engine is mid-W-burst".  Deliberately SEMANTIC rather
    // than a raw copy of l2c_victim.v's `st`: the state encoding moved when
    // that FSM was pipelined (2026-08-20, S_B deleted and S_AW became the
    // idle state), and a testbench comparing against a hard-coded 3'd2 goes
    // quietly false rather than failing loudly.
    output wire        dbg_l2_victim_wburst,
    output wire [1:0]  dbg_l2_victim_beat
);
    assign dbg_q_head  = u_scan.q_head;
    assign dbg_q_tail  = u_scan.q_tail;
    assign dbg_q_count = u_scan.q_count;
    assign dbg_f_state = u_scan.u_fetch.f_state;
    assign dbg_mux_max_bulk_ahead = CHAIN_MAX_BULK_AHEAD[3:0];
    assign dbg_mux_max_bulk_quiet = CHAIN_MAX_BULK_QUIET[3:0];
    assign dbg_mux_bulk_count = u_mux.bulk_count;
    assign dbg_mux_rdq_count = u_mux.rdq_count;
    assign dbg_scan_ar_fire = scan_arvalid && scan_arready;

    // ── axi_xbar instance (VRAM_IN_DDR must be defined at build time) ──
    wire [5:0] s0_awid; wire [31:0] s0_awaddr; wire [7:0] s0_awlen;
    wire [2:0] s0_awsize; wire [1:0] s0_awburst; wire s0_awvalid, s0_awready;
    wire [127:0] s0_wdata; wire [15:0] s0_wstrb; wire s0_wlast, s0_wvalid, s0_wready;
    wire [5:0] s0_bid; wire [1:0] s0_bresp; wire s0_bvalid, s0_bready;
    wire [5:0] s0_arid; wire [31:0] s0_araddr; wire [7:0] s0_arlen;
    wire [2:0] s0_arsize; wire [1:0] s0_arburst; wire s0_arvalid, s0_arready;
    wire [5:0] s0_rid; wire [127:0] s0_rdata; wire [1:0] s0_rresp;
    wire s0_rlast, s0_rvalid, s0_rready;

    // S3 -- VRAM aperture, now a genuine slave port again (T16 revert of
    // T14's decode fold): feeds the VRAM lane (address-translate below +
    // axi_vram_priority_mux3's s3_* port).
    wire [5:0] s3_awid; wire [31:0] s3_awaddr; wire [7:0] s3_awlen;
    wire [2:0] s3_awsize; wire [1:0] s3_awburst; wire s3_awvalid, s3_awready;
    wire [127:0] s3_wdata; wire [15:0] s3_wstrb; wire s3_wlast, s3_wvalid, s3_wready;
    wire [5:0] s3_bid; wire [1:0] s3_bresp; wire s3_bvalid, s3_bready;
    wire [5:0] s3_arid; wire [31:0] s3_araddr; wire [7:0] s3_arlen;
    wire [2:0] s3_arsize; wire [1:0] s3_arburst; wire s3_arvalid, s3_arready;
    wire [5:0] s3_rid; wire [127:0] s3_rdata; wire [1:0] s3_rresp;
    wire s3_rlast, s3_rvalid, s3_rready;

    // Inert slave sinks (S1/S2/S4/S5) -- never addressed by this tb.
    wire s1_awready = 1'b1, s1_wready = 1'b1, s1_arready = 1'b1;
    wire s2_awready = 1'b1, s2_wready = 1'b1, s2_arready = 1'b1;
    wire s4_awready = 1'b1, s4_wready = 1'b1, s4_arready = 1'b1;
    wire s5_awready = 1'b1, s5_wready = 1'b1, s5_arready = 1'b1;

    axi_xbar #(.S3_BACKEND_SURVIVES_FLUSH(1)) u_xbar (
        .clk(core_clk), .rst(core_rst),
        .cpu_overlay_active(1'b0), .cpu_overlay_reset(1'b0),
        .dbg_ram_window_lg2(6'd26),
        .cpu_held_in_reset(1'b0),
        .slv_flush(slv_flush),

        // M0 -- CPU LSU
        .m0_awid(cpu_awid), .m0_awaddr(cpu_awaddr), .m0_awlen(cpu_awlen),
        .m0_awsize(cpu_awsize), .m0_awburst(cpu_awburst), .m0_awvalid(cpu_awvalid),
        .m0_awready(cpu_awready),
        .m0_wdata(cpu_wdata), .m0_wstrb(cpu_wstrb), .m0_wlast(cpu_wlast),
        .m0_wvalid(cpu_wvalid), .m0_wready(cpu_wready),
        .m0_bid(cpu_bid), .m0_bresp(cpu_bresp), .m0_bvalid(cpu_bvalid), .m0_bready(cpu_bready),
        .m0_arid(cpu_arid), .m0_araddr(cpu_araddr), .m0_arlen(cpu_arlen),
        .m0_arsize(cpu_arsize), .m0_arburst(cpu_arburst), .m0_arvalid(cpu_arvalid),
        .m0_arready(cpu_arready),
        .m0_rid(cpu_rid), .m0_rdata(cpu_rdata), .m0_rresp(cpu_rresp),
        .m0_rlast(cpu_rlast), .m0_rvalid(cpu_rvalid), .m0_rready(cpu_rready),

        // M0b -- boot FSM (write-only), M1 -- host debug, M2 -- CPU IF:
        // all permanently inactive in this tb.
        .m0b_awid(4'd0), .m0b_awaddr(32'd0), .m0b_awlen(8'd0), .m0b_awsize(3'd0),
        .m0b_awburst(2'd0), .m0b_awvalid(1'b0), .m0b_awready(),
        .m0b_wdata(128'd0), .m0b_wstrb(16'd0), .m0b_wlast(1'b0), .m0b_wvalid(1'b0),
        .m0b_wready(),
        .m0b_bid(), .m0b_bresp(), .m0b_bvalid(), .m0b_bready(1'b1),

        .m1_awid(4'd0), .m1_awaddr(32'd0), .m1_awlen(8'd0), .m1_awsize(3'd0),
        .m1_awburst(2'd0), .m1_awvalid(1'b0), .m1_awready(),
        .m1_wdata(128'd0), .m1_wstrb(16'd0), .m1_wlast(1'b0), .m1_wvalid(1'b0), .m1_wready(),
        .m1_bid(), .m1_bresp(), .m1_bvalid(), .m1_bready(1'b1),
        .m1_arid(4'd0), .m1_araddr(32'd0), .m1_arlen(8'd0), .m1_arsize(3'd0),
        .m1_arburst(2'd0), .m1_arvalid(1'b0), .m1_arready(),
        .m1_rid(), .m1_rdata(), .m1_rresp(), .m1_rlast(), .m1_rvalid(), .m1_rready(1'b1),

        .m2_arid(4'd0), .m2_araddr(32'd0), .m2_arlen(8'd0), .m2_arsize(3'd0),
        .m2_arburst(2'd0), .m2_arvalid(1'b0), .m2_arready(),
        .m2_rid(), .m2_rdata(), .m2_rresp(), .m2_rlast(), .m2_rvalid(), .m2_rready(1'b1),

        // S0 -- DDR (feeds the l2c/mux/bridge chain below)
        .s0_awid(s0_awid), .s0_awaddr(s0_awaddr), .s0_awlen(s0_awlen),
        .s0_awsize(s0_awsize), .s0_awburst(s0_awburst), .s0_awvalid(s0_awvalid),
        .s0_awready(s0_awready),
        .s0_wdata(s0_wdata), .s0_wstrb(s0_wstrb), .s0_wlast(s0_wlast),
        .s0_wvalid(s0_wvalid), .s0_wready(s0_wready),
        .s0_bid(s0_bid), .s0_bresp(s0_bresp), .s0_bvalid(s0_bvalid), .s0_bready(s0_bready),
        .s0_arid(s0_arid), .s0_araddr(s0_araddr), .s0_arlen(s0_arlen),
        .s0_arsize(s0_arsize), .s0_arburst(s0_arburst), .s0_arvalid(s0_arvalid),
        .s0_arready(s0_arready),
        .s0_rid(s0_rid), .s0_rdata(s0_rdata), .s0_rresp(s0_rresp),
        .s0_rlast(s0_rlast), .s0_rvalid(s0_rvalid), .s0_rready(s0_rready),

        // S1/S2/S4/S5 -- inert sinks, never targeted by this tb's traffic.
        .s1_awid(), .s1_awaddr(), .s1_awlen(), .s1_awsize(), .s1_awburst(),
        .s1_awvalid(), .s1_awready(s1_awready),
        .s1_wdata(), .s1_wstrb(), .s1_wlast(), .s1_wvalid(), .s1_wready(s1_wready),
        .s1_bid(6'd0), .s1_bresp(2'd0), .s1_bvalid(1'b0), .s1_bready(),
        .s1_arid(), .s1_araddr(), .s1_arlen(), .s1_arsize(), .s1_arburst(),
        .s1_arvalid(), .s1_arready(s1_arready),
        .s1_rid(6'd0), .s1_rdata(128'd0), .s1_rresp(2'd0), .s1_rlast(1'b1),
        .s1_rvalid(1'b0), .s1_rready(),

        .s2_awid(), .s2_awaddr(), .s2_awlen(), .s2_awsize(), .s2_awburst(),
        .s2_awvalid(), .s2_awready(s2_awready),
        .s2_wdata(), .s2_wstrb(), .s2_wlast(), .s2_wvalid(), .s2_wready(s2_wready),
        .s2_bid(6'd0), .s2_bresp(2'd0), .s2_bvalid(1'b0), .s2_bready(),
        .s2_arid(), .s2_araddr(), .s2_arlen(), .s2_arsize(), .s2_arburst(),
        .s2_arvalid(), .s2_arready(s2_arready),
        .s2_rid(6'd0), .s2_rdata(128'd0), .s2_rresp(2'd0), .s2_rlast(1'b1),
        .s2_rvalid(1'b0), .s2_rready(),

        // S3 -- VRAM aperture: genuine slave port again (T16). Feeds the
        // VRAM lane arbiter below (via the address-translate wires).
        .s3_awid(s3_awid), .s3_awaddr(s3_awaddr), .s3_awlen(s3_awlen),
        .s3_awsize(s3_awsize), .s3_awburst(s3_awburst),
        .s3_awvalid(s3_awvalid), .s3_awready(s3_awready),
        .s3_wdata(s3_wdata), .s3_wstrb(s3_wstrb), .s3_wlast(s3_wlast),
        .s3_wvalid(s3_wvalid), .s3_wready(s3_wready),
        .s3_bid(s3_bid), .s3_bresp(s3_bresp), .s3_bvalid(s3_bvalid), .s3_bready(s3_bready),
        .s3_arid(s3_arid), .s3_araddr(s3_araddr), .s3_arlen(s3_arlen),
        .s3_arsize(s3_arsize), .s3_arburst(s3_arburst),
        .s3_arvalid(s3_arvalid), .s3_arready(s3_arready),
        .s3_rid(s3_rid), .s3_rdata(s3_rdata), .s3_rresp(s3_rresp), .s3_rlast(s3_rlast),
        .s3_rvalid(s3_rvalid), .s3_rready(s3_rready),

        .s4_awid(), .s4_awaddr(), .s4_awlen(), .s4_awsize(), .s4_awburst(),
        .s4_awvalid(), .s4_awready(s4_awready),
        .s4_wdata(), .s4_wstrb(), .s4_wlast(), .s4_wvalid(), .s4_wready(s4_wready),
        .s4_bid(6'd0), .s4_bresp(2'd0), .s4_bvalid(1'b0), .s4_bready(),
        .s4_arid(), .s4_araddr(), .s4_arlen(), .s4_arsize(), .s4_arburst(),
        .s4_arvalid(), .s4_arready(s4_arready),
        .s4_rid(6'd0), .s4_rdata(128'd0), .s4_rresp(2'd0), .s4_rlast(1'b1),
        .s4_rvalid(1'b0), .s4_rready(),

        .s5_awid(), .s5_awaddr(), .s5_awlen(), .s5_awsize(), .s5_awburst(),
        .s5_awvalid(), .s5_awready(s5_awready),
        .s5_wdata(), .s5_wstrb(), .s5_wlast(), .s5_wvalid(), .s5_wready(s5_wready),
        .s5_bid(6'd0), .s5_bresp(2'd0), .s5_bvalid(1'b0), .s5_bready(),
        .s5_arid(), .s5_araddr(), .s5_arlen(), .s5_arsize(), .s5_arburst(),
        .s5_arvalid(), .s5_arready(s5_arready),
        .s5_rid(6'd0), .s5_rdata(128'd0), .s5_rresp(2'd0), .s5_rlast(1'b1),
        .s5_rvalid(1'b0), .s5_rready()
    );

    // -- CPU-path side of the T13/T14 seam: either l2c's master port
    //    (CHAIN_L2C_ENABLE=1) or a direct pass-through of s0_* -------------
    wire [5:0]   c_awid;   wire [31:0]  c_awaddr;  wire [7:0] c_awlen;
    wire [2:0]   c_awsize; wire [1:0]   c_awburst; wire c_awvalid, c_awready;
    wire [127:0] c_wdata;  wire [15:0]  c_wstrb;   wire c_wlast, c_wvalid, c_wready;
    wire [5:0]   c_bid;    wire [1:0]   c_bresp;   wire c_bvalid, c_bready;
    wire [5:0]   c_arid;   wire [31:0]  c_araddr;  wire [7:0] c_arlen;
    wire [2:0]   c_arsize; wire [1:0]   c_arburst; wire c_arvalid, c_arready;
    wire [5:0]   c_rid;    wire [127:0] c_rdata;   wire [1:0] c_rresp;
    wire c_rlast, c_rvalid, c_rready;
    wire [31:0] l2_dbg_miss_count;
    wire [3:0]  l2_dbg_mshr_occupancy;
    wire         test_l2_arready;
    wire         test_l2_rvalid;
    wire         test_l2_rlast;
    assign dbg_l2_miss_count = l2_dbg_miss_count;
    assign dbg_l2_mshr_occupancy = l2_dbg_mshr_occupancy;

    generate
    if (CHAIN_L2C_ENABLE != 0) begin : g_l2c
        wire         l2s_arready;
        wire [5:0]   l2s_rid;
        wire [127:0] l2s_rdata;
        wire [1:0]   l2s_rresp;
        wire         l2s_rlast;
        wire         l2s_rvalid;

        assign s0_arready = !test_bulk_enable && l2s_arready;
        assign s0_rid = l2s_rid;
        assign s0_rdata = l2s_rdata;
        assign s0_rresp = l2s_rresp;
        assign s0_rlast = l2s_rlast;
        assign s0_rvalid = !test_bulk_enable && l2s_rvalid;
        assign test_l2_arready = test_bulk_enable && l2s_arready;
        assign test_l2_rvalid = test_bulk_enable && l2s_rvalid;
        assign test_l2_rlast = l2s_rlast;
        assign dbg_l2_victim_wburst = (u_l2c.g_active.u_victim.st ==
                                        u_l2c.g_active.u_victim.S_W);
        assign dbg_l2_victim_beat = u_l2c.g_active.u_victim.beat;

        // T16: bypass-window params back to l2c.v's own defaults
        // (disabled) -- VRAM-aperture traffic never reaches this
        // instance's slave port at all any more (it takes the S3 lane,
        // never xbar S0), so there is nothing left to bypass. See
        // docs/l2c_spec.md's "Integration update (T16)" note.
        l2c #(
            .ADDR_WIDTH(32), .DATA_WIDTH(128), .ID_WIDTH(6),
            .L2_BYPASS_ALL(0),
            .EXTERNAL_WRITE_RESET_RECOVERY(1),
            .CACHEABLE_BASE(32'h0000_0000), .CACHEABLE_SIZE(32'h4100_0000)
        ) u_l2c (
            .clk(core_clk), .rst(core_rst),
            .s_axi_awid(s0_awid), .s_axi_awaddr(s0_awaddr), .s_axi_awlen(s0_awlen),
            .s_axi_awsize(s0_awsize), .s_axi_awburst(s0_awburst), .s_axi_awvalid(s0_awvalid),
            .s_axi_awready(s0_awready),
            .s_axi_wdata(s0_wdata), .s_axi_wstrb(s0_wstrb), .s_axi_wlast(s0_wlast),
            .s_axi_wvalid(s0_wvalid), .s_axi_wready(s0_wready),
            .s_axi_bid(s0_bid), .s_axi_bresp(s0_bresp), .s_axi_bvalid(s0_bvalid), .s_axi_bready(s0_bready),
            .s_axi_arid(test_bulk_enable ? {3'b111, test_bulk_araddr[8:6]} : s0_arid),
            .s_axi_araddr(test_bulk_enable ? test_bulk_araddr : s0_araddr),
            .s_axi_arlen(test_bulk_enable ? 8'd0 : s0_arlen),
            .s_axi_arsize(test_bulk_enable ? 3'd4 : s0_arsize),
            .s_axi_arburst(test_bulk_enable ? 2'd1 : s0_arburst),
            .s_axi_arvalid(test_bulk_enable ? test_bulk_arvalid : s0_arvalid),
            .s_axi_arready(l2s_arready),
            .s_axi_rid(l2s_rid), .s_axi_rdata(l2s_rdata), .s_axi_rresp(l2s_rresp),
            .s_axi_rlast(l2s_rlast), .s_axi_rvalid(l2s_rvalid),
            .s_axi_rready(test_bulk_enable ? test_bulk_rready : s0_rready),
            .m_axi_awid(c_awid), .m_axi_awaddr(c_awaddr), .m_axi_awlen(c_awlen),
            .m_axi_awsize(c_awsize), .m_axi_awburst(c_awburst), .m_axi_awvalid(c_awvalid),
            .m_axi_awready(c_awready),
            .m_axi_wdata(c_wdata), .m_axi_wstrb(c_wstrb), .m_axi_wlast(c_wlast),
            .m_axi_wvalid(c_wvalid), .m_axi_wready(c_wready),
            .m_axi_bid(c_bid), .m_axi_bresp(c_bresp), .m_axi_bvalid(c_bvalid), .m_axi_bready(c_bready),
            .m_axi_arid(c_arid), .m_axi_araddr(c_araddr), .m_axi_arlen(c_arlen),
            .m_axi_arsize(c_arsize), .m_axi_arburst(c_arburst), .m_axi_arvalid(c_arvalid),
            .m_axi_arready(c_arready),
            .m_axi_rid(c_rid), .m_axi_rdata(c_rdata), .m_axi_rresp(c_rresp),
            .m_axi_rlast(c_rlast), .m_axi_rvalid(c_rvalid), .m_axi_rready(c_rready),
            .dbg_hit_count(), .dbg_miss_count(l2_dbg_miss_count),
            .dbg_mshr_occupancy(l2_dbg_mshr_occupancy)
        );
    end else begin : g_no_l2c
        assign l2_dbg_miss_count = 32'd0;
        assign l2_dbg_mshr_occupancy = 4'd0;
        assign test_l2_arready = 1'b0;
        assign test_l2_rvalid = 1'b0;
        assign test_l2_rlast = 1'b0;
        assign dbg_l2_victim_wburst = 1'b0;
        assign dbg_l2_victim_beat = 2'd0;
        assign c_awid = s0_awid; assign c_awaddr = s0_awaddr; assign c_awlen = s0_awlen;
        assign c_awsize = s0_awsize; assign c_awburst = s0_awburst; assign c_awvalid = s0_awvalid;
        assign s0_awready = c_awready;
        assign c_wdata = s0_wdata; assign c_wstrb = s0_wstrb; assign c_wlast = s0_wlast;
        assign c_wvalid = s0_wvalid;
        assign s0_wready = c_wready;
        assign s0_bid = c_bid; assign s0_bresp = c_bresp; assign s0_bvalid = c_bvalid;
        assign c_bready = s0_bready;
        assign c_arid = s0_arid; assign c_araddr = s0_araddr; assign c_arlen = s0_arlen;
        assign c_arsize = s0_arsize; assign c_arburst = s0_arburst; assign c_arvalid = s0_arvalid;
        assign s0_arready = c_arready;
        assign s0_rid = c_rid; assign s0_rdata = c_rdata; assign s0_rresp = c_rresp;
        assign s0_rlast = c_rlast; assign s0_rvalid = c_rvalid;
        assign c_rready = s0_rready;
    end
    endgenerate

    // -- scanout_ddr_reader, driven by the top-level scan_rd_* BFM ports --
    wire [5:0]   scan_arid;   wire [31:0]  scan_araddr; wire [7:0] scan_arlen;
    wire [2:0]   scan_arsize; wire [1:0]   scan_arburst; wire scan_arvalid, scan_arready;
    wire [5:0]   scan_rid;    wire [127:0] scan_rdata;   wire [1:0] scan_rresp;
    wire scan_rlast, scan_rvalid, scan_rready;
    wire bulkmux_arready;
    wire [5:0] bulkmux_rid;
    wire [127:0] bulkmux_rdata;
    wire [1:0] bulkmux_rresp;
    wire bulkmux_rlast, bulkmux_rvalid, bulkmux_rready;
    generate
    if (CHAIN_L2C_ENABLE != 0) begin : g_l2_test_route
        assign c_arready = bulkmux_arready;
        assign c_rvalid = bulkmux_rvalid;
        assign bulkmux_rready = c_rready;
        assign test_bulk_arready = test_l2_arready;
        assign test_bulk_rvalid = test_l2_rvalid;
        assign test_bulk_rlast = test_l2_rlast;
    end else begin : g_raw_test_route
        assign c_arready = !test_bulk_enable && bulkmux_arready;
        assign c_rvalid = !test_bulk_enable && bulkmux_rvalid;
        assign bulkmux_rready = test_bulk_enable ? test_bulk_rready : c_rready;
        assign test_bulk_arready = test_bulk_enable && bulkmux_arready;
        assign test_bulk_rvalid = test_bulk_enable && bulkmux_rvalid;
        assign test_bulk_rlast = bulkmux_rlast;
    end
    endgenerate
    assign c_rid = bulkmux_rid;
    assign c_rdata = bulkmux_rdata;
    assign c_rresp = bulkmux_rresp;
    assign c_rlast = bulkmux_rlast;

    scanout_ddr_reader #(
        // BPP=8 is the read-port ADDRESS granularity (1 byte per index);
        // RD_DATA_W=32 is the (widened) read-port DATA width.
        .ADDR_W(21), .BPP(8), .RD_DATA_W(32),
        .DATA_WIDTH(128), .ID_WIDTH(6), .AXI_ID(6'h00),
        .CARVEOUT_BASE(`AXI_VRAM_DDR_CARVEOUT_BASE),
        .CARVEOUT_SIZE(`AXI_VRAM_DDR_CARVEOUT_SIZE)
    ) u_scan (
        .clk(core_clk), .rst(core_rst),
        .rd_clk(core_clk), .rd_rst(core_rst),
        .rd_addr(scan_rd_addr), .rd_en(scan_rd_en),
        .rd_data(scan_rd_data), .rd_valid(scan_rd_valid),
        .m_arid(scan_arid), .m_araddr(scan_araddr), .m_arlen(scan_arlen),
        .m_arsize(scan_arsize), .m_arburst(scan_arburst),
        .m_arvalid(scan_arvalid), .m_arready(scan_arready),
        .m_rid(scan_rid), .m_rdata(scan_rdata), .m_rresp(scan_rresp),
        .m_rlast(scan_rlast), .m_rvalid(scan_rvalid), .m_rready(scan_rready)
    );

    // -- S3 backend, part 1: source select + address-translate, both in
    //    axi_vram_smoke_mux -- the SAME module fpga_top_ddr.vh
    //    instantiates, wired the same way.  (Part 2, byte-swap, is
    //    already done unconditionally inside axi_xbar.v on the S3
    //    boundary -- see that file's vram_swap_words/vram_swap_strb.) --
    wire [5:0]   smk_awid;   wire [31:0]  smk_awaddr; wire [7:0] smk_awlen;
    wire [2:0]   smk_awsize; wire [1:0]   smk_awburst; wire smk_awvalid, smk_awready;
    wire [127:0] smk_wdata;  wire [15:0]  smk_wstrb;   wire smk_wlast, smk_wvalid, smk_wready;
    wire [1:0]   smk_bresp;  wire smk_bvalid, smk_bready;
    wire         smoke_active = (CHAIN_VIDEO_SMOKE != 0) && !smoke_done;

    generate
    if (CHAIN_VIDEO_SMOKE != 0) begin : g_smoke
        wire [127:0] smk_wdata_raw;
        // The negative control: reverse the four bytes of every 32-bit
        // group, which is exactly axi_xbar.v's vram_swap_word32.  Applying
        // it here reproduces the "smoke landed on the wrong side of the S3
        // byte-swap" mistake bit-for-bit.
        genvar swi;
        for (swi = 0; swi < 4; swi = swi + 1) begin : g_swap
            assign smk_wdata[swi*32 +: 32] =
                (CHAIN_SMOKE_BROKEN_SWAP != 0)
                    ? { smk_wdata_raw[swi*32 +:  8],
                        smk_wdata_raw[swi*32+8  +: 8],
                        smk_wdata_raw[swi*32+16 +: 8],
                        smk_wdata_raw[swi*32+24 +: 8] }
                    : smk_wdata_raw[swi*32 +: 32];
        end

        vram_smoke #(
            .FB_WIDTH_PX  (CHAIN_SMOKE_W),
            .FB_HEIGHT_PX (CHAIN_SMOKE_H),
            .BPP          (8),
            .DATA_WIDTH   (128),
            .ID_WIDTH     (6),
            .ROW_BAND_LOG2(CHAIN_SMOKE_ROW_BAND_LOG2)
        ) u_smoke (
            .clk(core_clk), .rst(core_rst),
            .done(smoke_done),
            .m_awid(smk_awid), .m_awaddr(smk_awaddr), .m_awlen(smk_awlen),
            .m_awsize(smk_awsize), .m_awburst(smk_awburst),
            .m_awvalid(smk_awvalid), .m_awready(smk_awready),
            .m_wdata(smk_wdata_raw), .m_wstrb(smk_wstrb), .m_wlast(smk_wlast),
            .m_wvalid(smk_wvalid), .m_wready(smk_wready),
            .m_bid(6'd0), .m_bresp(smk_bresp), .m_bvalid(smk_bvalid),
            .m_bready(smk_bready)
        );
    end else begin : g_no_smoke
        assign smk_awid    = 6'd0;
        assign smk_awaddr  = 32'd0;
        assign smk_awlen   = 8'd0;
        assign smk_awsize  = 3'd0;
        assign smk_awburst = 2'd0;
        assign smk_awvalid = 1'b0;
        assign smk_wdata   = 128'd0;
        assign smk_wstrb   = 16'd0;
        assign smk_wlast   = 1'b0;
        assign smk_wvalid  = 1'b0;
        assign smk_bready  = 1'b1;
        assign smoke_done  = 1'b1;
    end
    endgenerate

    wire [5:0]   s3lane_awid;   wire [31:0]  s3lane_awaddr; wire [7:0] s3lane_awlen;
    wire [2:0]   s3lane_awsize; wire [1:0]   s3lane_awburst;
    wire s3lane_awvalid, s3lane_awready;
    wire [127:0] s3lane_wdata;  wire [15:0]  s3lane_wstrb;
    wire s3lane_wlast, s3lane_wvalid, s3lane_wready;
    wire [5:0]   s3lane_bid;    wire [1:0]   s3lane_bresp;
    wire s3lane_bvalid, s3lane_bready;
    wire [5:0]   s3lane_arid;   wire [31:0]  s3lane_araddr; wire [7:0] s3lane_arlen;
    wire [2:0]   s3lane_arsize; wire [1:0]   s3lane_arburst;
    wire s3lane_arvalid, s3lane_arready;
    wire [5:0]   s3lane_rid;    wire [127:0] s3lane_rdata;  wire [1:0] s3lane_rresp;
    wire s3lane_rlast, s3lane_rvalid, s3lane_rready;

    axi_vram_smoke_mux #(
        .ID_WIDTH(6), .ADDR_WIDTH(32), .DATA_WIDTH(128),
        .CARVEOUT_BASE(`AXI_VRAM_DDR_CARVEOUT_BASE)
    ) u_vram_smoke_mux (
        .smoke_active(smoke_active),
        .smk_awid(smk_awid), .smk_awaddr(smk_awaddr), .smk_awlen(smk_awlen),
        .smk_awsize(smk_awsize), .smk_awburst(smk_awburst),
        .smk_awvalid(smk_awvalid), .smk_awready(smk_awready),
        .smk_wdata(smk_wdata), .smk_wstrb(smk_wstrb), .smk_wlast(smk_wlast),
        .smk_wvalid(smk_wvalid), .smk_wready(smk_wready),
        .smk_bresp(smk_bresp), .smk_bvalid(smk_bvalid), .smk_bready(smk_bready),

        .s3_awid(s3_awid), .s3_awaddr(s3_awaddr), .s3_awlen(s3_awlen),
        .s3_awsize(s3_awsize), .s3_awburst(s3_awburst),
        .s3_awvalid(s3_awvalid), .s3_awready(s3_awready),
        .s3_wdata(s3_wdata), .s3_wstrb(s3_wstrb), .s3_wlast(s3_wlast),
        .s3_wvalid(s3_wvalid), .s3_wready(s3_wready),
        .s3_bid(s3_bid), .s3_bresp(s3_bresp), .s3_bvalid(s3_bvalid), .s3_bready(s3_bready),
        .s3_arid(s3_arid), .s3_araddr(s3_araddr), .s3_arlen(s3_arlen),
        .s3_arsize(s3_arsize), .s3_arburst(s3_arburst),
        .s3_arvalid(s3_arvalid), .s3_arready(s3_arready),
        .s3_rid(s3_rid), .s3_rdata(s3_rdata), .s3_rresp(s3_rresp),
        .s3_rlast(s3_rlast), .s3_rvalid(s3_rvalid), .s3_rready(s3_rready),

        .m_awid(s3lane_awid), .m_awaddr(s3lane_awaddr), .m_awlen(s3lane_awlen),
        .m_awsize(s3lane_awsize), .m_awburst(s3lane_awburst),
        .m_awvalid(s3lane_awvalid), .m_awready(s3lane_awready),
        .m_wdata(s3lane_wdata), .m_wstrb(s3lane_wstrb), .m_wlast(s3lane_wlast),
        .m_wvalid(s3lane_wvalid), .m_wready(s3lane_wready),
        .m_bid(s3lane_bid), .m_bresp(s3lane_bresp), .m_bvalid(s3lane_bvalid),
        .m_bready(s3lane_bready),
        .m_arid(s3lane_arid), .m_araddr(s3lane_araddr), .m_arlen(s3lane_arlen),
        .m_arsize(s3lane_arsize), .m_arburst(s3lane_arburst),
        .m_arvalid(s3lane_arvalid), .m_arready(s3lane_arready),
        .m_rid(s3lane_rid), .m_rdata(s3lane_rdata), .m_rresp(s3lane_rresp),
        .m_rlast(s3lane_rlast), .m_rvalid(s3lane_rvalid), .m_rready(s3lane_rready)
    );

    // -- axi_vram_priority_mux3: 3-way full R/W merge of the l2c-path
    //    (c_*), the S3/VRAM-aperture CPU lane (s3_*, address-translated
    //    above), and scanout_ddr_reader (scan_*, read-only, strict top
    //    priority). See that module's header for the priority contract. --
    wire [5:0]   m_awid;   wire [31:0]  m_awaddr; wire [7:0] m_awlen;
    wire [2:0]   m_awsize; wire [1:0]   m_awburst; wire m_awvalid, m_awready;
    wire [127:0] m_wdata;  wire [15:0]  m_wstrb;   wire m_wlast, m_wvalid, m_wready;
    wire [5:0]   m_bid;    wire [1:0]   m_bresp;   wire m_bvalid, m_bready;
    wire [5:0]   m_arid;   wire [31:0]  m_araddr; wire [7:0] m_arlen;
    wire [2:0]   m_arsize; wire [1:0]   m_arburst; wire m_arvalid, m_arready;
    wire [5:0]   m_rid;    wire [127:0] m_rdata;   wire [1:0] m_rresp;
    wire m_rlast, m_rvalid, m_rready;

    // Kept in step with axi_vram_priority_mux3.v's own default ON PURPOSE:
    // this chain is meant to test what SHIPS.  Exported on
    // dbg_mux_max_bulk_ahead so the C++ scenarios check the invariant
    // against the configured bound rather than a hard-coded 2.
    localparam CHAIN_MAX_BULK_AHEAD = 4;
    localparam CHAIN_MAX_BULK_QUIET = 8;

    axi_vram_priority_mux3 #(
        .ID_WIDTH(6), .ADDR_WIDTH(32), .DATA_WIDTH(128),
        .MAX_BULK_AHEAD(CHAIN_MAX_BULK_AHEAD),
        .MAX_BULK_AHEAD_QUIET(CHAIN_MAX_BULK_QUIET),
        .EXTERNAL_WRITE_RESET_RECOVERY(1)
    ) u_mux (
        .clk(core_clk), .rst(core_rst),
        .l2c_awid(c_awid), .l2c_awaddr(c_awaddr), .l2c_awlen(c_awlen),
        .l2c_awsize(c_awsize), .l2c_awburst(c_awburst),
        .l2c_awvalid(c_awvalid), .l2c_awready(c_awready),
        .l2c_wdata(c_wdata), .l2c_wstrb(c_wstrb), .l2c_wlast(c_wlast),
        .l2c_wvalid(c_wvalid), .l2c_wready(c_wready),
        .l2c_bid(c_bid), .l2c_bresp(c_bresp), .l2c_bvalid(c_bvalid), .l2c_bready(c_bready),
        .l2c_arid((CHAIN_L2C_ENABLE == 0 && test_bulk_enable) ? 6'h3f : c_arid),
        .l2c_araddr((CHAIN_L2C_ENABLE == 0 && test_bulk_enable) ? test_bulk_araddr : c_araddr),
        .l2c_arlen((CHAIN_L2C_ENABLE == 0 && test_bulk_enable) ? 8'd3 : c_arlen),
        .l2c_arsize((CHAIN_L2C_ENABLE == 0 && test_bulk_enable) ? 3'd4 : c_arsize),
        .l2c_arburst((CHAIN_L2C_ENABLE == 0 && test_bulk_enable) ? 2'd1 : c_arburst),
        .l2c_arvalid((CHAIN_L2C_ENABLE == 0 && test_bulk_enable) ? test_bulk_arvalid : c_arvalid),
        .l2c_arready(bulkmux_arready),
        .l2c_rid(bulkmux_rid), .l2c_rdata(bulkmux_rdata), .l2c_rresp(bulkmux_rresp),
        .l2c_rlast(bulkmux_rlast), .l2c_rvalid(bulkmux_rvalid), .l2c_rready(bulkmux_rready),
        .s3_awid(s3lane_awid), .s3_awaddr(s3lane_awaddr), .s3_awlen(s3lane_awlen),
        .s3_awsize(s3lane_awsize), .s3_awburst(s3lane_awburst),
        .s3_awvalid(s3lane_awvalid), .s3_awready(s3lane_awready),
        .s3_wdata(s3lane_wdata), .s3_wstrb(s3lane_wstrb), .s3_wlast(s3lane_wlast),
        .s3_wvalid(s3lane_wvalid), .s3_wready(s3lane_wready),
        .s3_bid(s3lane_bid), .s3_bresp(s3lane_bresp), .s3_bvalid(s3lane_bvalid),
        .s3_bready(s3lane_bready),
        .s3_arid(s3lane_arid), .s3_araddr(s3lane_araddr), .s3_arlen(s3lane_arlen),
        .s3_arsize(s3lane_arsize), .s3_arburst(s3lane_arburst),
        .s3_arvalid(s3lane_arvalid), .s3_arready(s3lane_arready),
        .s3_rid(s3lane_rid), .s3_rdata(s3lane_rdata), .s3_rresp(s3lane_rresp),
        .s3_rlast(s3lane_rlast), .s3_rvalid(s3lane_rvalid), .s3_rready(s3lane_rready),
        .scan_arid(scan_arid), .scan_araddr(scan_araddr), .scan_arlen(scan_arlen),
        .scan_arsize(scan_arsize), .scan_arburst(scan_arburst),
        .scan_arvalid(scan_arvalid), .scan_arready(scan_arready),
        .scan_rid(scan_rid), .scan_rdata(scan_rdata), .scan_rresp(scan_rresp),
        .scan_rlast(scan_rlast), .scan_rvalid(scan_rvalid), .scan_rready(scan_rready),
        .m_awid(m_awid), .m_awaddr(m_awaddr), .m_awlen(m_awlen),
        .m_awsize(m_awsize), .m_awburst(m_awburst),
        .m_awvalid(m_awvalid), .m_awready(m_awready),
        .m_wdata(m_wdata), .m_wstrb(m_wstrb), .m_wlast(m_wlast),
        .m_wvalid(m_wvalid), .m_wready(m_wready),
        .m_bid(m_bid), .m_bresp(m_bresp), .m_bvalid(m_bvalid), .m_bready(m_bready),
        .m_arid(m_arid), .m_araddr(m_araddr), .m_arlen(m_arlen),
        .m_arsize(m_arsize), .m_arburst(m_arburst),
        .m_arvalid(m_arvalid), .m_arready(m_arready),
        .m_rid(m_rid), .m_rdata(m_rdata), .m_rresp(m_rresp),
        .m_rlast(m_rlast), .m_rvalid(m_rvalid), .m_rready(m_rready)
    );

    // -- CDC: core_clk -> mig_clk (same instance shape as ddr_ctrl.v's
    //    real-hw u_core_to_mig_ui / tb_l2c_chain.v's u_core_to_mig_ui). --
    wire [5:0]   ui_awid;   wire [31:0]  ui_awaddr;  wire [7:0] ui_awlen;
    wire [2:0]   ui_awsize; wire [1:0]   ui_awburst; wire ui_awvalid, ui_awready;
    wire [127:0] ui_wdata;  wire [15:0]  ui_wstrb;   wire ui_wlast, ui_wvalid, ui_wready;
    wire [5:0]   ui_bid;    wire [1:0]   ui_bresp;   wire ui_bvalid, ui_bready;
    wire [5:0]   ui_arid;   wire [31:0]  ui_araddr;  wire [7:0] ui_arlen;
    wire [2:0]   ui_arsize; wire [1:0]   ui_arburst; wire ui_arvalid, ui_arready;
    wire [5:0]   ui_rid;    wire [127:0] ui_rdata;   wire [1:0] ui_rresp;
    wire ui_rlast, ui_rvalid, ui_rready;

    axi_async_bridge #(
        .DATA_WIDTH(128), .ADDR_WIDTH(32), .ID_WIDTH(6), .USER_WIDTH(1)
    ) u_core_to_mig_ui (
        .s_clk(core_clk), .s_rst(core_rst),
        // AW/W/B come from the merged VRAM-lane arbiter output now (T16)
        // -- both l2c-path AND S3/VRAM-aperture CPU writes are
        // arbitrated onto this one physical port (see
        // axi_vram_priority_mux3.v's header).
        .s_awid(m_awid), .s_awaddr(m_awaddr), .s_awlen(m_awlen),
        .s_awsize(m_awsize), .s_awburst(m_awburst), .s_awlock(1'b0),
        .s_awcache(4'b0011), .s_awprot(3'b000), .s_awqos(4'b0000),
        .s_awuser(1'b0), .s_awvalid(m_awvalid), .s_awready(m_awready),
        .s_wdata(m_wdata), .s_wstrb(m_wstrb), .s_wlast(m_wlast),
        .s_wuser(1'b0), .s_wvalid(m_wvalid), .s_wready(m_wready),
        .s_bid(m_bid), .s_bresp(m_bresp), .s_buser(), .s_bvalid(m_bvalid),
        .s_bready(m_bready),
        // AR/R come from the merged mux output.
        .s_arid(m_arid), .s_araddr(m_araddr), .s_arlen(m_arlen),
        .s_arsize(m_arsize), .s_arburst(m_arburst), .s_arlock(1'b0),
        .s_arcache(4'b0011), .s_arprot(3'b000), .s_arqos(4'b0000),
        .s_aruser(1'b0), .s_arvalid(m_arvalid), .s_arready(m_arready),
        .s_rid(m_rid), .s_rdata(m_rdata), .s_rresp(m_rresp), .s_rlast(m_rlast),
        .s_ruser(), .s_rvalid(m_rvalid), .s_rready(m_rready),

        .m_clk(mig_clk), .m_rst(mig_rst || !cal_done),
        .m_awid(ui_awid), .m_awaddr(ui_awaddr), .m_awlen(ui_awlen),
        .m_awsize(ui_awsize), .m_awburst(ui_awburst), .m_awlock(),
        .m_awcache(), .m_awprot(), .m_awqos(),
        .m_awuser(), .m_awvalid(ui_awvalid), .m_awready(ui_awready),
        .m_wdata(ui_wdata), .m_wstrb(ui_wstrb), .m_wlast(ui_wlast),
        .m_wuser(), .m_wvalid(ui_wvalid), .m_wready(ui_wready),
        .m_bid(ui_bid), .m_bresp(ui_bresp), .m_buser(1'b0),
        .m_bvalid(ui_bvalid), .m_bready(ui_bready),
        .m_arid(ui_arid), .m_araddr(ui_araddr), .m_arlen(ui_arlen),
        .m_arsize(ui_arsize), .m_arburst(ui_arburst), .m_arlock(),
        .m_arcache(), .m_arprot(), .m_arqos(),
        .m_aruser(), .m_arvalid(ui_arvalid), .m_arready(ui_arready),
        .m_rid(ui_rid), .m_rdata(ui_rdata), .m_rresp(ui_rresp),
        .m_rlast(ui_rlast), .m_ruser(1'b0), .m_rvalid(ui_rvalid),
        .m_rready(ui_rready)
    );

    // -- repo DDR contract -> pcie_test MIG contract shim --
    wire [0:0]   mig_awid;  wire [30:0] mig_awaddr; wire [7:0] mig_awlen;
    wire [2:0]   mig_awsize; wire [1:0] mig_awburst; wire mig_awvalid, mig_awready;
    wire [255:0] mig_wdata; wire [31:0] mig_wstrb;   wire mig_wlast, mig_wvalid, mig_wready;
    wire [0:0]   mig_bid;   wire [1:0]  mig_bresp;   wire mig_bvalid, mig_bready;
    wire [0:0]   mig_arid;  wire [30:0] mig_araddr;  wire [7:0] mig_arlen;
    wire [2:0]   mig_arsize; wire [1:0] mig_arburst; wire mig_arvalid, mig_arready;
    wire [0:0]   mig_rid;   wire [255:0] mig_rdata;  wire [1:0] mig_rresp;
    wire mig_rlast, mig_rvalid, mig_rready;

    axi_ddr4_mig_bridge u_repo_to_pcie_mig (
        .clk(mig_clk), .rst(mig_rst || !cal_done),
        .s_awid(ui_awid), .s_awaddr(ui_awaddr), .s_awlen(ui_awlen),
        .s_awsize(ui_awsize), .s_awburst(ui_awburst),
        .s_awvalid(ui_awvalid), .s_awready(ui_awready),
        .s_wdata(ui_wdata), .s_wstrb(ui_wstrb), .s_wlast(ui_wlast),
        .s_wvalid(ui_wvalid), .s_wready(ui_wready),
        .s_bid(ui_bid), .s_bresp(ui_bresp), .s_bvalid(ui_bvalid),
        .s_bready(ui_bready),
        .s_arid(ui_arid), .s_araddr(ui_araddr), .s_arlen(ui_arlen),
        .s_arsize(ui_arsize), .s_arburst(ui_arburst),
        .s_arvalid(ui_arvalid), .s_arready(ui_arready),
        .s_rid(ui_rid), .s_rdata(ui_rdata), .s_rresp(ui_rresp),
        .s_rlast(ui_rlast), .s_rvalid(ui_rvalid), .s_rready(ui_rready),

        .m_awid(mig_awid), .m_awaddr(mig_awaddr), .m_awlen(mig_awlen),
        .m_awsize(mig_awsize), .m_awburst(mig_awburst),
        .m_awvalid(mig_awvalid), .m_awready(mig_awready),
        .m_wdata(mig_wdata), .m_wstrb(mig_wstrb), .m_wlast(mig_wlast),
        .m_wvalid(mig_wvalid), .m_wready(mig_wready),
        .m_bid(mig_bid), .m_bresp(mig_bresp), .m_bvalid(mig_bvalid),
        .m_bready(mig_bready),
        .m_arid(mig_arid), .m_araddr(mig_araddr), .m_arlen(mig_arlen),
        .m_arsize(mig_arsize), .m_arburst(mig_arburst),
        .m_arvalid(mig_arvalid), .m_arready(mig_arready),
        .m_rid(mig_rid), .m_rdata(mig_rdata), .m_rresp(mig_rresp),
        .m_rlast(mig_rlast), .m_rvalid(mig_rvalid), .m_rready(mig_rready)
    );

    // -- behavioural MIG stand-in (sim only) --
    sim_mig_backend #(
        .BEATS_LOG2(20),  // 2^20 x 32B = 32 MiB -- matches tb_l2c_chain.v;
                           // ample for the carveout's working set (this tb
                           // only ever drives carveout-range addresses).
        .STALL_ENABLE(STALL_ENABLE),
        .STALL_SEED(STALL_SEED),
        .READ_LATENCY_BASE(DDR_READ_LATENCY_BASE),
        .READ_LATENCY_JITTER(DDR_READ_LATENCY_JITTER)
    ) u_mig_sim (
        .clk(mig_clk), .rst(mig_rst),
        .cal_done(cal_done),
        .awid(mig_awid), .awaddr(mig_awaddr), .awlen(mig_awlen),
        .awsize(mig_awsize), .awburst(mig_awburst),
        .awvalid(mig_awvalid), .awready(mig_awready),
        .wdata(mig_wdata), .wstrb(mig_wstrb), .wlast(mig_wlast),
        .wvalid(mig_wvalid), .wready(mig_wready),
        .bid(mig_bid), .bresp(mig_bresp), .bvalid(mig_bvalid), .bready(mig_bready),
        .arid(mig_arid), .araddr(mig_araddr), .arlen(mig_arlen),
        .arsize(mig_arsize), .arburst(mig_arburst),
        .arvalid(mig_arvalid), .arready(mig_arready),
        .rid(mig_rid), .rdata(mig_rdata), .rresp(mig_rresp),
        .rlast(mig_rlast), .rvalid(mig_rvalid), .rready(mig_rready)
    );

endmodule

`endif

`default_nettype wire
