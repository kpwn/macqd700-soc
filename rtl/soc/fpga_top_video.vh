// rtl/fpga_top_video.vh — included from rtl/fpga_top.v
//
// VRAM (URAM) with optional VIDEO_SMOKE writer, xbar-S3/smoke AXI mux, vram write counters, and the video_top HDMI scan-out instance.
//
// Generated mechanically from rtl/fpga_top.v during the fpga_top split
// (agent/p3-fpga-top-split). No functional change: this file contains
// the original lines verbatim, inside the fpga_top module scope via
// `include. Do not add module/endmodule wrappers or nettype directives.

    // ═══════════════════════════════════════════════════════════════════
    // VRAM (URAM) — AXI slave tied off this round
    // ═══════════════════════════════════════════════════════════════════
    // ══ Scanner upper-bound geometry ═════════════════════════════════
    // The actual visible window comes from DAFB (m_hres, m_vres) at
    // runtime; FB_W/FB_H only bound what the scanner can hold.  A mode
    // wider than FB_W is CROPPED (scanout_fetch.v `row_px_last_in` clamps
    // to SRC_W-1, scanout_display.v's x_src clamps to SRC_W-1), and one
    // taller than FB_H likewise -- silently, and still rendering, which is
    // why the old 1024x768 bound looked fine for so long.
    //
    // 1024x768 was too small for the Q700's own monitor list.  MAME's
    // macqd700 `monitor_config` (src/mame/apple/dafb.cpp:205) offers TWO
    // 1152x870 displays -- monitor code 0 ("Mac 21\" Color") and code 3
    // ("Mac Two-Page, B&W 21\"") -- and a 640x870 Portrait.  At the old
    // bound all three rendered a 1024x768 crop of the desktop.
    //
    //   FB_W  1152  covers every horizontal mode in that list
    //               (1152, 832, 800, 768, 640, 512).
    //   FB_H  1024  covers every vertical mode (870, 624, 600, 576, 480,
    //               384) with the next power of two, which costs nothing:
    //               FB_H only sets $clog2 widths and the row bound, not
    //               memory.
    //
    // COST, and it is the LINE BUFFER only.  scanout_display.v holds
    // LINE_COUNT(64) x FB_W bytes in each of three byte planes.  At
    // Vivado's 4096-deep 8-bit RAMB36 mode that is
    //     1024 -> 3 x ceil(65536/4096) = 3 x 16 = 48 RAMB36
    //     1152 -> 3 x ceil(73728/4096) = 3 x 18 = 54 RAMB36   (+6)
    // Everything else in the scanout path is already derived: SRC_X_W /
    // SRC_Y_W / LINE_MEM_AW / FRAME_ADDR_W are all $clog2 of these two.
    //
    // FUTURE 1920-WIDE PATH -- NOT taken here, deliberately.  The user's
    // stated direction is eventually 1920x1080x24bpp, reached through an
    // EXTERNAL video path (a NuBus-style card with its own declaration
    // ROM), not by extending DAFB; DAFB stays MAME-faithful and 1080p is
    // not in this matrix.  Sizing THIS buffer for 1920 would cost
    //     1920 -> 3 x ceil(122880/4096) = 3 x 30 = 90 RAMB36   (+42)
    // i.e. ~27% of the KU5P's ~153 RAMB36 budget, spent on a path that is
    // not the one 1080p will use.  The cheap way to get there when the
    // time comes is to trade LINE_COUNT against FB_W (LINE_COUNT_LOG2=5 at
    // FB_W=1920 costs 45 RAMB36, i.e. LESS than today) -- the ring only has
    // to cover the fetch round trip, and at 1:1 output scale 64 resident
    // source rows is far more than that.  That is a deliberate design call,
    // so it is left for whoever builds the external path.
    //
    // Overridable so a bring-up build can shrink or grow it without
    // editing this file.
`ifdef SCANOUT_SRC_W
    localparam FB_W   = `SCANOUT_SRC_W;
`else
    localparam FB_W   = 1152;
`endif
`ifdef SCANOUT_SRC_H
    localparam FB_H   = `SCANOUT_SRC_H;
`else
    localparam FB_H   = 1024;
`endif
    // FB_BPP is the width of the streaming READ PORT, not the pixel depth.
    // That port is byte-addressed and byte-wide at every depth: the runtime
    // depth travels on dafb_fb_bytes_per_px and the multi-byte-per-pixel
    // assembly happens inside linebuf_scanout.v.  vram.v's URAM array at
    // BPP=8 fits comfortably in ~24 URAM288 of KU5P's 64-URAM budget; its
    // BPP>16 elab guard is therefore never armed and never needs to be.
    //
    // FB_MAX_PIXELS is the SCANNER'S addressable byte range (one byte per
    // index at the internal FB_BPP=8), NOT the active scanout window
    // FB_W×FB_H.  Match it to the VRAM aperture so MAME-canonical 1bpp
    // configs whose raw byte addresses span the full source-row range
    // (base + stride*(SRC_H-1) + …) at the scanner's SRC_H=768 do not get
    // falsely rejected by the scanout placement gate.
    //
    // ── Aperture size: 2 MB on URAM, 4 MB on DDR ──────────────────────
    // 2 MB is Q700 silicon (MAME `map(0xf9000000, 0xf91fffff)`) and is what
    // the URAM build keeps: widening it would cost URAM the KU5P does not
    // have (2 MB already needs 64 URAM288, the whole budget).
    //
    // Under VRAM_IN_DDR the array is a DDR carveout instead, so capacity is
    // free, and 24bpp needs it: DAFB 24bpp is 4 B/px, so 1024×768×24bpp is
    // 3,145,728 bytes and simply does not fit 2 MB.  4 MB is the smallest
    // power of two that holds it, stays naturally aligned for the xbar's
    // `(a & ~(VRAM_SIZE-1))` decode, and still leaves the DAFB register
    // window at 0xF980_0000 clear (the aperture ends at 0xF93F_FFFF).
    //
    // COMPATIBILITY NOTE: the Q700 ROM sizes VRAM by probing the aperture,
    // and offers depths from that size + monitor sense.  A 4 MB aperture is
    // therefore visible to Mac OS as 4 MB of VRAM, which is not what Q700
    // silicon has.  That is a deliberate deviation, confined to the
    // VRAM_IN_DDR build, in exchange for 1024×768 direct colour.
`ifdef VRAM_IN_DDR
    localparam FB_MAX_PIXELS = 32'h0040_0000;
`else
    localparam FB_MAX_PIXELS = 32'h0020_0000;
`endif
    localparam FB_BPP = 8;
    // Width of the VRAM streaming read-port DATA bus.  The port stays
    // byte-ADDRESSED (FB_BPP=8, one index per byte -- FB_ADDR_W below is
    // still in those units) but returns the 4-BYTE GROUP starting at the
    // requested address.  Both backends implement that contract identically
    // (vram.v RD_DATA_W / scanout_ddr_reader.v RD_DATA_W), so they remain
    // drop-in interchangeable.  See linebuf_scanout.v's header for why the
    // group exists: it is what makes 24bpp scanout at 1024x768 sustainable.
    localparam FB_FETCH_W = 32;
    // FB_ADDR_W must cover the FULL aperture in read-port units (1 byte per
    // index at FB_BPP=8), not just one video mode's active pixel count
    // (1024x768 = 786,432 needs only 20 bits).  Under-widening it silently
    // truncates/aliases any DAFB base register programmed above the
    // representable range onto the low part of the aperture — see
    // rtl/board/vram.v's PX_ADDR_SPAN comment for the matching fix on the
    // vram.v side.  21 bits spans 0..0x1FFFFF (2 MB); 22 bits spans
    // 0..0x3FFFFF (4 MB).
`ifdef VRAM_IN_DDR
    localparam FB_ADDR_W = 22;
`else
    localparam FB_ADDR_W = 21;
`endif
    localparam [FB_ADDR_W-1:0] FB_INVALID_SCANOUT_ADDR = {FB_ADDR_W{1'b1}};

    wire                   vram_rd_en;
    wire [FB_ADDR_W-1:0]   vram_rd_addr;
    wire [FB_FETCH_W-1:0]  vram_rd_data;
    wire                   vram_rd_valid;

    wire pclk;
    wire hdmi_mmcm_locked;
    wire hdmi_i2c_done;
    wire fb_underflow_sticky;
    wire [11:0] video_debug_hcount;
    wire [10:0] video_debug_vcount;
    wire [23:0] video_debug_rgb;
    wire        video_debug_de;
    wire        video_debug_hs;
    wire        video_debug_vs;
    // Coherent one-pclk-edge scan-out snapshot + committed placement
    // addresses (task #243).  See video_top.v's port comment for why
    // these are single wide buses rather than more individual probes.
    wire [95:0]  video_dbg_snap;
    // 160 bits: the committed placement in the low 64 (unchanged bit
    // positions, so an older jtag_repl decode still reads it correctly) plus
    // the REJECTED tuple + reason + sticky bit above it.  See video_top.v's
    // dbg_video_place assignment for the field map, and
    // docs/video_path_review.md S4.2 for why a rejection has to be loud.
    wire [159:0] video_dbg_place;
    // pclk-domain frame-start tick used to drive VIA1 CA1 via pulse_cdc.
    // The receiving net is declared in fpga_top_cpu.vh so that
    // fpga_top_peripherals.vh (included before this file) can drive its
    // pulse_cdc input from the same wire.  Task #145.
    // VRAM synthesis uses common-clock URAM, so the streaming read port
    // is clocked/reset in core_clk.  video_top's fb_reader CDC bridge
    // explicitly receives the same clock/reset below, so the VRAM scan-
    // out path remains valid when CORE_CLK_DIVIDE is 2/4/8.
    // soc_full_rst here so a JTAG/btn debug-full-reset clears the read
    // FIFO pointers and forces the next scan-out frame to start fresh.
    // The URAM cell content survives reset (it's content-addressed) —
    // only the controller state cycles.
    // Use bank-bit [0] of the q6 broadcast bus so the video reset cone
    // (~1660 sinks under u_video alone) doesn't pull on the same FF as
    // the peripheral / xbar / commit consumers.  Sim-neutral.
    wire vram_rd_rst = soc_full_rst_bank[0];
    // ── Coupled scan-out reset (task #194) ────────────────────────────
    // The VRAM streaming READ port must reset on the union of the core-
    // side reset and the video pclk-domain reset, not on `vram_rd_rst`
    // alone.  video_top exports exactly that term (see its port comment):
    // `vram_rd_rst` OR the pclk reset synchronised into core_clk.
    //
    // WHY IT IS NOT OPTIONAL.  fb_reader's credit accounting and the read
    // port's request queue are the two halves of ONE in-order
    // request/response contract.  A pclk-only reset -- an HDMI MMCM
    // relock, which happens on every JTAG bitstream reload -- clears
    // fb_reader's half and leaves the port's queue loaded; the port then
    // answers stale requests to a caller that has forgotten them and the
    // response stream is permanently offset from the request stream.  With
    // more than ~16 entries queued at that instant the offset also pushes
    // the 64-entry queue past its (unbackpressured) depth, at which point
    // its tail laps its head and the offset becomes unrecoverable.
    // Reproduced in tb-fb-reader-ddr-chain (pclk_only_reset_midstream).
    wire video_vram_side_rst;
    wire vram_rd_rst_coupled = video_vram_side_rst;

`ifndef VRAM_IN_DDR
    // VRAM AXI slave wiring.  Two sources may drive the slave:
    //   - VIDEO_SMOKE=1: a reset-time `vram_smoke` writer paints SMPTE
    //     bars once; while it's active it owns the slave.  After it
    //     finishes (smoke_done=1) it goes quiescent.
    //   - The xbar S3 port (0xF900_0000..0xF90F_FFFF, task #147): CPU
    //     writes to the VRAM pixel aperture.
    //
    // We mux the vram slave inputs by `smoke_done`:
    //   smoke_done=0 → smoke drives AW/W, xbar S3 stalled (awready=0).
    //   smoke_done=1 → xbar S3 drives AW/W, smoke is quiescent (its
    //                  awvalid/wvalid went low when it finished).
    //
    // When VIDEO_SMOKE=0 the smoke writer is NOT instantiated at all
    // and the xbar S3 port owns the slave from reset.
    //
    // The vram slave uses ID_WIDTH=6 to match the xbar's XID.  Smoke
    // zero-extends its 4-bit ID into bits [5:4] so there's no IDW clash.
    localparam VRAM_S_IDW = 6;
    wire [VRAM_S_IDW-1:0] vram_s_awid;
    wire [31:0]           vram_s_awaddr;
    wire [7:0]            vram_s_awlen;
    wire [2:0]            vram_s_awsize;
    wire [1:0]            vram_s_awburst;
    wire                  vram_s_awvalid;
    wire                  vram_s_awready;
    wire [127:0]          vram_s_wdata;
    wire [15:0]           vram_s_wstrb;
    wire                  vram_s_wlast;
    wire                  vram_s_wvalid;
    wire                  vram_s_wready;
    wire [VRAM_S_IDW-1:0] vram_s_bid;
    wire [1:0]            vram_s_bresp;
    wire                  vram_s_bvalid;
    wire                  vram_s_bready;
    wire [VRAM_S_IDW-1:0] vram_s_arid;
    wire [31:0]           vram_s_araddr;
    wire [7:0]            vram_s_arlen;
    wire [2:0]            vram_s_arsize;
    wire [1:0]            vram_s_arburst;
    wire                  vram_s_arvalid;
    wire                  vram_s_arready;
    wire [VRAM_S_IDW-1:0] vram_s_rid;
    wire [127:0]          vram_s_rdata;
    wire [1:0]            vram_s_rresp;
    wire                  vram_s_rlast;
    wire                  vram_s_rvalid;
    wire                  vram_s_rready;
    wire                  smoke_done;
    wire                  smoke_active;
    wire [15:0]           vram_write_count;
    wire [15:0]           dafb_write_count;

    // Smoke-side AXI master wires — driven when VIDEO_SMOKE=1, tied off
    // to zero otherwise.  Smoke doesn't read the slave so AR/R are
    // unused.
    wire [3:0]            smk_awid;
    wire [31:0]           smk_awaddr;
    wire [7:0]            smk_awlen;
    wire [2:0]            smk_awsize;
    wire [1:0]            smk_awburst;
    wire                  smk_awvalid;
    wire                  smk_awready;
    wire [127:0]          smk_wdata;
    wire [15:0]           smk_wstrb;
    wire                  smk_wlast;
    wire                  smk_wvalid;
    wire                  smk_wready;
    wire [1:0]            smk_bresp;
    wire                  smk_bvalid;
    wire                  smk_bready;

    generate
        if (VIDEO_SMOKE != 0) begin : gen_smoke
            vram_smoke #(
                .FB_WIDTH_PX (FB_W),
                .FB_HEIGHT_PX(FB_H),
                .BPP         (FB_BPP),
                .DATA_WIDTH  (128),
                .ID_WIDTH    (4)
            ) u_smoke (
                .clk       (core_clk),
                // soc_full_rst — a debug-full-reset re-arms the SMPTE-bar
                // smoke writer so first-light visibly restarts.  Bank
                // bit [0] (video region) so the smoke-writer reset shares
                // a placement zone with the rest of the video cone.
                .rst       (soc_full_rst_bank[0]),
                .done      (smoke_done),
                .m_awid    (smk_awid),
                .m_awaddr  (smk_awaddr),
                .m_awlen   (smk_awlen),
                .m_awsize  (smk_awsize),
                .m_awburst (smk_awburst),
                .m_awvalid (smk_awvalid),
                .m_awready (smk_awready),
                .m_wdata   (smk_wdata),
                .m_wstrb   (smk_wstrb),
                .m_wlast   (smk_wlast),
                .m_wvalid  (smk_wvalid),
                .m_wready  (smk_wready),
                .m_bid     (/* unused — smoke ignores bid */),
                .m_bresp   (smk_bresp),
                .m_bvalid  (smk_bvalid),
                .m_bready  (smk_bready)
            );
        end else begin : gen_no_smoke
            // No smoke instance — tie all master-side outputs to 0 and
            // absorb ready/valid handshakes.  smoke_done starts high so
            // the mux forwards xbar S3 straight through from reset.
            assign smk_awid    = 4'd0;
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

    // Smoke is still painting while (VIDEO_SMOKE=1 && !smoke_done).
    // Once smoke_done latches high the mux hands the slave to xbar S3.
    assign smoke_active = (VIDEO_SMOKE != 0) && !smoke_done;

    // ── CPU-side byte-swap (now centralised in axi_xbar S3 boundary) ─
    // The 68k LSU is big-endian — byte 0 (lowest address) arrives at
    // wdata[31:24] of each 32-bit lane — but vram.v's scanout uses
    // little-endian lanes within a word (pixel 0 at bits [7:0]).
    //
    // The byte-lane swap on the CPU↔VRAM hop is applied INSIDE
    // `axi_xbar.v` on the S3 master ports (`vram_swap_words` /
    // `vram_swap_strb` — see axi_xbar.v lines ~933 / ~1526).  This
    // file used to inline a duplicate copy of the same swap on the
    // S3↔vram leg, producing a double-swap that cancelled out and
    // re-created the original P1 pixel-mirror bug for any CPU write
    // through the xbar.  Removed in the endianness audit (2026-04-28).
    //
    // The `vram_smoke` path bypasses both swaps because it already
    // authors data in vram-native LE convention.  The standalone
    // `rtl/soc/vram_cpu_byteswap.v` shim module (and its sole consumer,
    // `tb/tb_framebuffer_pixel`, which reproduced the same BE->LE
    // permutation outside the xbar) were removed as dead production code
    // — the swap lives only in axi_xbar.v S3 now (build-truth-hygiene
    // cleanup).

    // ── AW channel mux ──────────────────────────────────────────────────
    assign vram_s_awid    = smoke_active ? {2'b00, smk_awid} : s3_awid;
    assign vram_s_awaddr  = smoke_active ? smk_awaddr  : s3_awaddr;
    assign vram_s_awlen   = smoke_active ? smk_awlen   : s3_awlen;
    assign vram_s_awsize  = smoke_active ? smk_awsize  : s3_awsize;
    assign vram_s_awburst = smoke_active ? smk_awburst : s3_awburst;
    assign vram_s_awvalid = smoke_active ? smk_awvalid : s3_awvalid;
    assign smk_awready    = smoke_active ? vram_s_awready : 1'b0;
    assign s3_awready     = smoke_active ? 1'b0           : vram_s_awready;

    // ── W channel mux (CPU-side swap done by axi_xbar S3) ──────────────
    assign vram_s_wdata  = smoke_active ? smk_wdata  : s3_wdata;
    assign vram_s_wstrb  = smoke_active ? smk_wstrb  : s3_wstrb;
    assign vram_s_wlast  = smoke_active ? smk_wlast  : s3_wlast;
    assign vram_s_wvalid = smoke_active ? smk_wvalid : s3_wvalid;
    assign smk_wready    = smoke_active ? vram_s_wready : 1'b0;
    assign s3_wready     = smoke_active ? 1'b0           : vram_s_wready;

    // ── B channel mux ───────────────────────────────────────────────────
    // Only one master has outstanding AW at a time (serialised by
    // smoke_active flip), so we route bvalid to the owner and mask the
    // other side.
    assign smk_bresp   = vram_s_bresp;
    assign smk_bvalid  = smoke_active ? vram_s_bvalid : 1'b0;
    assign s3_bid      = vram_s_bid;
    assign s3_bresp    = vram_s_bresp;
    assign s3_bvalid   = smoke_active ? 1'b0         : vram_s_bvalid;
    assign vram_s_bready = smoke_active ? smk_bready : s3_bready;

    // ── AR / R channels: smoke does not read; xbar S3 owns them ────────
    // While smoke is active we stall S3 reads too — but the CPU can
    // still issue them; the xbar holds the burst until arready.
    assign vram_s_arid    = smoke_active ? {VRAM_S_IDW{1'b0}} : s3_arid;
    assign vram_s_araddr  = smoke_active ? 32'd0              : s3_araddr;
    assign vram_s_arlen   = smoke_active ? 8'd0               : s3_arlen;
    assign vram_s_arsize  = smoke_active ? 3'd0               : s3_arsize;
    assign vram_s_arburst = smoke_active ? 2'd0               : s3_arburst;
    assign vram_s_arvalid = smoke_active ? 1'b0               : s3_arvalid;
    assign s3_arready     = smoke_active ? 1'b0               : vram_s_arready;

    assign s3_rid    = vram_s_rid;
    assign s3_rdata  = vram_s_rdata;  // CPU-side swap done by axi_xbar S3
    assign s3_rresp  = vram_s_rresp;
    assign s3_rlast  = vram_s_rlast;
    assign s3_rvalid = smoke_active ? 1'b0 : vram_s_rvalid;
    assign vram_s_rready = smoke_active ? 1'b1 : s3_rready;

    reg [15:0] vram_write_count_r;
    reg [15:0] dafb_write_count_r;
    assign vram_write_count = vram_write_count_r;
    assign dafb_write_count = dafb_write_count_r;
    always @(posedge core_clk) begin
        // Bank bit [0] — these counters live alongside the video cone so
        // their reset shares a placement region.
        if (soc_full_rst_bank[0]) begin
            vram_write_count_r <= 16'd0;
            dafb_write_count_r <= 16'd0;
        end else begin
            if (vram_s_wvalid && vram_s_wready && vram_s_wlast)
                vram_write_count_r <= vram_write_count_r + 16'd1;
            if (dafb_wvalid && dafb_wready)
                dafb_write_count_r <= dafb_write_count_r + 16'd1;
        end
    end

    vram #(
        .FB_WIDTH_PX (FB_W),
        .FB_HEIGHT_PX(FB_H),
        .BPP         (FB_BPP),
        .RD_DATA_W   (FB_FETCH_W),
        .DATA_WIDTH  (128),
        .ID_WIDTH    (VRAM_S_IDW)
    ) u_vram (
        // soc_full_rst — drives the controller AXI handshake reset AND
        // (since the wipe-on-reset FSM landed) walks all URAM words
        // writing zero so the framebuffer starts each cold/warm reset
        // clean.  At 100 MHz on a 2 MiB / 128-bit URAM the wipe takes
        // ~131k cycles ≈ 1.3 ms; AXI accept signals stay low until it
        // completes.  Bank bit [0] (video region).
        .clk(core_clk), .rst(soc_full_rst_bank[0]),
        // clear_req = soc_full_rst — every platform reset triggers the
        // VRAM wipe FSM so the framebuffer starts clean instead of
        // showing stale pixels from the previous run.
        .clear_req(soc_full_rst_bank[0]),

        // AXI slave — driven either by vram_smoke (VIDEO_SMOKE=1) or
        // by the xbar S3 port (VIDEO_SMOKE=0) per the mux above.
        .s_awid   (vram_s_awid),
        .s_awaddr (vram_s_awaddr),
        .s_awlen  (vram_s_awlen),
        .s_awsize (vram_s_awsize),
        .s_awburst(vram_s_awburst),
        .s_awvalid(vram_s_awvalid),
        .s_awready(vram_s_awready),
        .s_wdata  (vram_s_wdata),
        .s_wstrb  (vram_s_wstrb),
        .s_wlast  (vram_s_wlast),
        .s_wvalid (vram_s_wvalid),
        .s_wready (vram_s_wready),
        .s_bid    (vram_s_bid),
        .s_bresp  (vram_s_bresp),
        .s_bvalid (vram_s_bvalid),
        .s_bready (vram_s_bready),
        .s_arid   (vram_s_arid), .s_araddr (vram_s_araddr),
        .s_arlen  (vram_s_arlen), .s_arsize (vram_s_arsize),
        .s_arburst(vram_s_arburst), .s_arvalid(vram_s_arvalid),
        .s_arready(vram_s_arready),
        .s_rid    (vram_s_rid), .s_rdata (vram_s_rdata),
        .s_rresp  (vram_s_rresp), .s_rlast (vram_s_rlast),
        .s_rvalid (vram_s_rvalid), .s_rready(vram_s_rready),

        // Streaming read port — VRAM now uses common_clock URAM, so
        // rd_clk = core_clk.  CDC to pclk happens DOWNSTREAM (inside
        // video_top / hdmi TMDS TX FIFO), not inside VRAM.
        .rd_clk  (core_clk    ),
        // Coupled scan-out reset -- see vram_rd_rst_coupled above (#194).
        .rd_rst  (vram_rd_rst_coupled),
        .rd_addr (vram_rd_addr),
        .rd_en   (vram_rd_en  ),
        .rd_data (vram_rd_data),
        .rd_valid(vram_rd_valid)
    );
`else
    // ═══════════════════════════════════════════════════════════════════
    // T16 (decode-vram-lane reshape of T14) VRAM_IN_DDR: vram.v's URAM
    // array is still NOT instantiated here -- that half of the migration
    // is unchanged from T14 (frees the ~57 URAM288 array so L2C_ENABLE's
    // own ~64 URAM288 budget fits on the same KU5P device, docs/
    // l2c_spec.md S3 "URAM budget note").
    //
    // UNLIKE T14: the xbar's S3 port is NOT dead here any more. T14 made
    // axi_xbar.v's decode_slv() route the VRAM aperture onto S0 instead
    // of S3 under this same gate (a decode-level fold) so CPU writes
    // could reach l2c's configured never-allocate bypass window. T16
    // reverts that fold -- decode_slv() is now UNCONDITIONAL, VRAM
    // aperture always -> S3, regardless of VRAM_IN_DDR -- and enforces
    // the "no L2 copy of the VRAM window" invariant structurally instead:
    // S3's traffic physically cannot reach l2c's slave port at all, so
    // there's nothing to bypass. S3's own zero-basing (vram_flatten())
    // and byte-swap (vram_swap_words/vram_swap_strb) are therefore
    // unconditional inside axi_xbar.v too now -- unchanged T14 placement/
    // semantics, just no longer duplicated as a second conditional copy
    // on S0. S3 is consumed by the VRAM-lane arbiter
    // (axi_vram_priority_mux3, instantiated in fpga_top_ddr.vh -- the
    // T13/T14 seam owns that wiring, including the address-translate
    // step from S3's zero-based offset to the DDR carveout's real
    // address; see that file's `s3lane_awaddr`/`s3lane_araddr`), so this
    // file does NOT tie s3_* off any more -- fpga_top_ddr.vh drives it.
    //
    // ── VIDEO_SMOKE under VRAM_IN_DDR (the carveout-write follow-up) ──
    // The former KNOWN LIMITATION here -- "VIDEO_SMOKE is NOT supported in
    // combination with VRAM_IN_DDR", enforced by an elaboration-time
    // `$fatal` -- is resolved.  vram_smoke is a plain AXI4 write master
    // with nothing URAM-specific about it, so instead of a dedicated VRAM
    // slave port it now takes the SAME lane a CPU write to the VRAM
    // aperture takes: axi_vram_smoke_mux (fpga_top_ddr.vh) selects it onto
    // xbar S3's backend, UPSTREAM of that lane's `+ CARVEOUT_BASE`
    // translate, so smoke's aperture-relative addresses are translated by
    // exactly the path CPU writes use.
    //
    // The smk_* wires and `smoke_done` / `smoke_active` are declared in
    // fpga_top_ddr.vh (that file is the VRAM-lane seam and is included
    // BEFORE this one) -- same split this file already uses for
    // scanout_ddr_reader's scan_* master port below.
    //
    // BYTE ORDER: no swap on this path.  axi_xbar.v applies
    // vram_swap_words/vram_swap_strb on its S3 MASTER ports, so the s3_*
    // signals the mux sees are already in plain AXI byte-lane order, which
    // is the order vram_smoke authors natively (lane 0 = lowest-address
    // pixel) and the order scanout_line_fetch.v fills its ring with.  See
    // axi_vram_smoke_mux.v's header, and tb_vram_ddr_chain's
    // `smoke_matches_cpu_path_byte_order` scenario for the executable
    // proof.
    //
    // GEOMETRY: FB_W must stay a POWER OF TWO for vram_smoke to synthesise
    // sanely (its per-lane `pix % FB_WIDTH_PX` / `pix / FB_WIDTH_PX` fold
    // to bit-slices only then -- see that file's synthesis note).  A
    // narrower visible window is expressed by programming the DAFB stride
    // to FB_W and its hres to the narrower value; the scanner then shows
    // the left hres columns of the pattern.
    //
    // CAVEAT for VIDEO_SMOKE=1 *with a real CPU*: while smoke owns the
    // lane the mux holds s3_awready low, so a CPU write to the VRAM
    // aperture stalls for the whole paint (1024x768x8bpp = 49152 single-
    // beat writes, ~200k core cycles ~= 2 ms at 100 MHz).  axi_xbar.v's
    // per-slave watchdog default is 2^WD_LOG2 = 2^18 = 262144 cycles, so
    // that is inside the bound but not by much.  The rig this exists for
    // builds CPU=stub, where nothing else touches the aperture; a
    // CPU=m68k + VIDEO_SMOKE=1 build should shrink FB_H (or raise
    // WD_LOG2) rather than assume the margin holds.
    //
    // ROW_BAND_LOG2=6 advances the bar phase once every 64 source rows,
    // which is exactly linebuf_scanout.v's LINE_COUNT.  That makes
    // "the fetcher filled the ring and parked at source row 63" visually
    // distinguishable from "the scanner is walking the frame" -- vertical
    // bars alone cannot tell those apart, which is the whole reason this
    // rig exists.
    generate
        if (VIDEO_SMOKE != 0) begin : gen_smoke_ddr
            vram_smoke #(
                .FB_WIDTH_PX  (FB_W),
                .FB_HEIGHT_PX (FB_H),
                .BPP          (FB_BPP),
                .DATA_WIDTH   (128),
                .ID_WIDTH     (6),
                .ROW_BAND_LOG2(6)
            ) u_smoke (
                .clk       (core_clk),
                // soc_full_rst bank bit [0] (video region) -- a debug
                // full-reset re-arms the writer so first-light visibly
                // restarts, same as the URAM branch above.
                .rst       (soc_full_rst_bank[0]),
                .done      (smoke_done),
                .m_awid    (smk_awid),
                .m_awaddr  (smk_awaddr),
                .m_awlen   (smk_awlen),
                .m_awsize  (smk_awsize),
                .m_awburst (smk_awburst),
                .m_awvalid (smk_awvalid),
                .m_awready (smk_awready),
                .m_wdata   (smk_wdata),
                .m_wstrb   (smk_wstrb),
                .m_wlast   (smk_wlast),
                .m_wvalid  (smk_wvalid),
                .m_wready  (smk_wready),
                .m_bid     (6'd0),
                .m_bresp   (smk_bresp),
                .m_bvalid  (smk_bvalid),
                .m_bready  (smk_bready)
            );
        end else begin : gen_no_smoke_ddr
            // No smoke instance -- tie the master side off and hold
            // smoke_done high so axi_vram_smoke_mux is a pure S3
            // pass-through from reset.
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

    // VRAM-aperture write counter. There is no VRAM AXI *slave* under
    // VRAM_IN_DDR, but there IS a single physical point every aperture
    // write crosses: axi_vram_smoke_mux's merged output (fpga_top_ddr.vh),
    // which carries both CPU/S3 writes and, when VIDEO_SMOKE=1, the
    // SMPTE-bar writer's. Counting there restores the same observable the
    // URAM branch exports (it used to be hard-tied 0 here, from T14 when
    // no such merge point existed) -- and it is what tells you over JTAG
    // whether the smoke writer is actually painting.
    // Branch-local declarations: the off-path (`ifndef`) branch declares
    // these at its own top; under VRAM_IN_DDR they must be declared here
    // (caught by the first gated `lint-fpga-top` elaboration, T16 Minor-4).
    wire [15:0] vram_write_count;
    wire [15:0] dafb_write_count;
    reg  [15:0] vram_write_count_r;
    assign vram_write_count = vram_write_count_r;
    always @(posedge core_clk) begin
        if (soc_full_rst_bank[0])
            vram_write_count_r <= 16'd0;
        else if (s3lane_wvalid && s3lane_wready && s3lane_wlast)
            vram_write_count_r <= vram_write_count_r + 16'd1;
    end

    scanout_ddr_reader #(
        .ADDR_W       (FB_ADDR_W),
        .BPP          (FB_BPP),
        .RD_DATA_W    (FB_FETCH_W),
        .DATA_WIDTH   (128),
        .ID_WIDTH     (6),
        .AXI_ID       (6'h00),
        .CARVEOUT_BASE(`AXI_VRAM_DDR_CARVEOUT_BASE),
        .CARVEOUT_SIZE(`AXI_VRAM_DDR_CARVEOUT_SIZE)
    ) u_scanout_reader (
        .clk    (core_clk   ),
        // Coupled scan-out reset -- see vram_rd_rst_coupled above (#194).
        // THIS is the fix for the reset-domain skew: this module's request
        // queue and fb_reader's credit counter now clear on one event.
        .rst    (vram_rd_rst_coupled),

        .rd_clk  (core_clk    ),
        .rd_rst  (vram_rd_rst_coupled),
        .rd_addr (vram_rd_addr),
        .rd_en   (vram_rd_en  ),
        .rd_data (vram_rd_data),
        .rd_valid(vram_rd_valid),

        // AXI4 master -- wires declared in fpga_top_ddr.vh (the T13/T14
        // seam), merged into the l2c-path/S3-lane traffic there by
        // axi_vram_priority_mux3 before reaching u_ddr.
        .m_arid   (scan_arid   ), .m_araddr (scan_araddr ),
        .m_arlen  (scan_arlen  ), .m_arsize (scan_arsize ),
        .m_arburst(scan_arburst), .m_arvalid(scan_arvalid),
        .m_arready(scan_arready),
        .m_rid    (scan_rid    ), .m_rdata  (scan_rdata  ),
        .m_rresp  (scan_rresp  ), .m_rlast  (scan_rlast  ),
        .m_rvalid (scan_rvalid ), .m_rready (scan_rready )
    );

    // dafb_write_count counting -- unrelated to VRAM_IN_DDR (DAFB
    // register-shim writes are unaffected by this migration), duplicated
    // here (rather than hoisted out of the ifndef/else) so the `ifndef`
    // branch above stays byte-for-byte identical to the pre-T14 file
    // (preprocessor-diff-provable off-path requirement).
    reg [15:0] dafb_write_count_r;
    assign dafb_write_count = dafb_write_count_r;
    always @(posedge core_clk) begin
        if (soc_full_rst_bank[0])
            dafb_write_count_r <= 16'd0;
        else if (dafb_wvalid && dafb_wready)
            dafb_write_count_r <= dafb_write_count_r + 16'd1;
    end
`endif

    // ═══════════════════════════════════════════════════════════════════
    // Video / HDMI — video_top owns its own MMCM off video_ref_clk
    // ═══════════════════════════════════════════════════════════════════
// ── 720p60 (2026-09-13) ────────────────────────────────────────────────
// Moved from 1080p60 (148.5 MHz) to 74.25 MHz. TWO independent
// reasons, both measured:
//
//  1. HDIO BANK OVER-SPEED. synth/hdmi.xdc documents an 8 ns min-period on
//     this part's HDIOLOGIC FFs -- 125 MHz -- which is why `IOB FALSE` is
//     forced on the RGB bus and why ODDR clock forwarding is unavailable, so
//     `al9134_clk` is a BUFG driving an OBUF directly. At 148.5 MHz that ran
//     the bank ~19% ABOVE its rating, with the data bus launched from fabric
//     flops whose skew is placement-dependent. Symptoms on a real display:
//     slipped lines, intermittent loss of clock, black-then-resync. 74.25 MHz
//     is comfortably inside the rating.
//
//  2. SCANOUT REQUEST-FIFO PRESSURE. fb_reader's `miss` counter (cycles where
//     the scanout had `s_rd_en` high while `s_rd_ready` was low, i.e. the
//     request FIFO was FULL) wraps its 16-bit range continuously on the
//     150 MHz core build. The 5-line prefetch still absorbs it -- the visible
//     failure flag `line_underflow_sticky` is 0 -- but the margin is being
//     consumed, and it is worse at 150 MHz than 100 because the CPU competes
//     for DDR. Halving the pixel rate halves the demand and restores margin.
//
// WHY 1080p30 AND NOT A SMALLER MODE. Two earlier attempts were rejected:
//   * 720p60 would silently HALVE the picture -- place_plan picks the largest
//     INTEGER scale that fits, and 720 active lines cannot hold 480*2 = 960.
//   * 1280x960@60 fits x2 and clocks at 99 MHz, but it is a VESA-style mode and
//     the target display REFUSED it. 1080p60 was already known to work there.
// 1080p30 is CEA-861 VIC 34: the SAME 1920x1080 format the display already
// accepts, same porches, 1080 >= 960 so integer x2 still fits -- only the refresh
// halves. That makes it the one CEA mode under the 125 MHz HDIO rating that keeps
// the picture at its current size.
//
// It also needs NO geometry override: 2200 x 1125 x 30 = 74,250,000 exactly, so
// video_top's 1080p defaults are already correct and only the divide moves.
// Cost: 30 Hz refresh, so pointer motion is choppier.
// ── VCO RETUNED TO 1485 MHz ON THE BOARD PATH (2026-09-15) ────────────────
// The board path used DIVCLK 4 / MULT 37.125 -> VCO 928.125 MHz and reached
// 74.25 MHz through the FRACTIONAL divide CLKOUT0_DIVIDE_F = 12.5.
//
// That works for CLKOUT0 and ONLY CLKOUT0: it is the one MMCME4 output with a
// fractional divide.  The forwarded pixel clock now needs a SECOND output at
// the same frequency but phase-shifted (see video_top's ODDRE1 note), and
// CLKOUT1_DIVIDE is integer-only -- so the VCO must be an integer multiple of
// 74.25 MHz.  928.125 is not (928.125 / 74.25 = 12.5).
//
// 1485 MHz is, at 20x.  From 100 MHz: DIVCLK 5 -> PFD 20 MHz, MULT_F 74.250
// (a legal 0.125 step, max 128) -> VCO 1485 MHz -> /20 -> 74.25 MHz EXACTLY.
//
// ⚠️ pclk itself is BIT-IDENTICAL in frequency and phase: still 74.25 MHz,
// still CLKOUT0_PHASE 0.  The raster, the VTG, the scanout, the VSYNC-to-first-
// active-pixel count and every line/frame strobe are untouched.  Only the VCO
// arithmetic and the new CLKOUT1 change.
//
// Side effects to watch on the first board bring-up: PFD drops 25 -> 20 MHz
// (slightly worse MMCM phase noise) while the VCO nearly doubles (better period
// jitter).  Expected net-neutral, but a sync-stability regression would show up
// here, so check that the display still locks before blaming the phase shift.
// Both paths now share ONE VCO and ONE set of divides, which they did not before.
`ifdef SIM_MODEL
    localparam real HDMI_CLKIN1_PERIOD_NS  = 5.000;   // 200 MHz SIM_MODEL fabric
    localparam integer HDMI_DIVCLK_DIVIDE  = 5;       // PFD = 40 MHz
    localparam real HDMI_CLKFBOUT_MULT_F   = 37.125;  // VCO = 1485 MHz
`else
    localparam real HDMI_CLKIN1_PERIOD_NS  = 10.000;  // 100 MHz AB7/AB6 fabric
    localparam integer HDMI_DIVCLK_DIVIDE  = 5;       // PFD = 20 MHz
    localparam real HDMI_CLKFBOUT_MULT_F   = 74.250;  // VCO = 1485 MHz
`endif
    // Same on both paths now.  ⚠️ _F and _I must stay equal -- _I feeds the
    // integer-only CLKOUT1 that forwards the clock off-chip; a mismatch sends
    // the encoder a clock at the wrong FREQUENCY (no sync at all).
    localparam real    HDMI_PCLK_DIVIDE_F = 20.000;   // 1485 / 20 = 74.25 MHz (720p60)
    localparam integer HDMI_PCLK_DIVIDE_I = 20;
    // Phase of the clock forwarded to the encoder, relative to the RGB launch.
    // 90 = a quarter period = both encoder-clock edges clear of every data
    // transition.  See the measured pad delays in video_top's ODDRE1 note.
    // This is the single knob for the capture-edge experiment: 0 reproduces the
    // broken build, 180 is the old inverted forward, 90/270 clear both edges.
    localparam real    HDMI_PCLK_FWD_PHASE = 90.000;

    // ── HDMI MMCM reset broadcast ──────────────────────────────────────
    // Previously `u_video.ext_resetn` was wired directly to `cpu_resetn`
    // (the raw board push-button pin).  That bypassed the SoC reset
    // broadcast, so a JTAG `debug_full_reset` would reset every core_clk
    // consumer via `soc_full_rst` but leave the video MMCM running.
    // pclk-domain state then survived the warm reset and the CDC FIFOs
    // inside `fb_reader` could deadlock with stale read-request pointers.
    //
    // Fix: drive the MMCM resetn (active-low, async-assert) from the OR
    // of the debounced board reset (platform_resetn), the SoC-wide
    // soc_full_rst broadcast (bank bit [0] — same slot as the rest of
    // the video reset cone), and `~platform_init_done` so the MMCM does
    // not try to lock against an unstable input clock during cold boot.
    // All three sources assert asynchronously; deassertion is taken
    // from the MMCM's internal reset synchroniser (the LOCKED output
    // already gates downstream pclk-domain logic — see video_top.v
    // rst_pipe).
    wire video_mmcm_resetn = platform_resetn &&
                             !soc_full_rst_bank[0] &&
                             platform_init_done;

    video_top #(
        .SRC_W          (FB_W),
        .SRC_H          (FB_H),
        .FETCH_W        (FB_FETCH_W),
        .ADDR_W         (FB_ADDR_W),
        .FB_MAX_PIXELS  (FB_MAX_PIXELS),
        .PCLK_DIVIDE_F  (HDMI_PCLK_DIVIDE_F),
        .PCLK_DIVIDE_I  (HDMI_PCLK_DIVIDE_I),
        .PCLK_FWD_PHASE_DEG (HDMI_PCLK_FWD_PHASE),
        // ── 720p60 (CEA-861 VIC 4) ────────────────────────────────────────
        // H total 1650 = 1280+110+40+220 ; V total 750 = 720+5+5+20
        // 1650 * 750 * 60 = 74,250,000 -> exactly PCLK, exactly 60.000 Hz.
        //
        // ⭐ SAME 74.25 MHz PIXEL CLOCK AS 1080p30, so this mode change is
        // TIMING-NEUTRAL: PCLK_DIVIDE_F, the MMCM parameters, the I2C divisors
        // below and synth/hdmi.xdc are all unchanged. Only the geometry moves.
        // video_top's DEFAULTS are 1080p, so these MUST be overridden here or the
        // VTG free-runs 1920x1080 against a 74.25 MHz clock and never locks.
        //
        // WHY 720p60 over the 1080p30 it replaces: the HDMI capture card is a
        // MacroSilicon MS2109, whose INPUT timing table is 1080p 60/59/50,
        // 720p 60/59/50, 480p/576p and 4K30 -- 1080p30 is ABSENT, so it never
        // locked and emitted a synthetic black frame (proved: frames byte-identical
        // at uniform Y=7 even at the card's own native output mode). 720p60 is its
        // NATIVE first detailed timing. The monitor takes both.
        //
        // COST: place_plan.v picks the largest INTEGER scale that fits, and
        // 480*2 = 960 > 720, so 640x480 renders at x1 -- a 640x480 image centred in
        // a 1280x720 frame. 512x384 is x1 too (768 > 720). The capture pipeline
        // crops to the active region to put the pixels back on screen.
        .DST_W          (1280), .DST_H   (720),
        .H_FP           (110),  .H_SYNC  (40),  .H_BP (220),
        .V_FP           (5),    .V_SYNC  (5),   .V_BP (20),
        // I2C timing derives from PCLK and video_top's defaults assume 148.5 MHz;
        // halve them so SCL stays ~100 kHz and the SiI9134 reset / post-reset waits
        // keep their intended 10 ms of REAL time. UNCHANGED from 1080p30 -- same PCLK.
        .I2C_HALF        (10'd372),
        .I2C_RESET_CYCLES(24'd742500),
        .I2C_INIT_WAIT   (24'd742500),
        .MMCM_CLKIN1_PERIOD(HDMI_CLKIN1_PERIOD_NS),
        .MMCM_DIVCLK_DIVIDE(HDMI_DIVCLK_DIVIDE),
        .MMCM_CLKFBOUT_MULT_F(HDMI_CLKFBOUT_MULT_F),
        .TEST_PATTERN   (0),  // never enabled — VRAM is the only scan source
        // Feed video_top the already-buffered single-ended sys_clk so it
        // skips its internal IBUFDS — Vivado errors out with two IBUFDS
        // drivers on one differential pair.
        .EXTERNAL_IBUFDS(1)
    ) u_video (
        .clk_ref_p    (video_ref_clk), // already-buffered SE fabric clock
        .clk_ref_n    (1'b0        ),  // unused when EXTERNAL_IBUFDS=1
        .ext_resetn   (video_mmcm_resetn),
        .vram_clk     (core_clk    ),
        .vram_rst     (vram_rd_rst ),
        .vram_side_rst(video_vram_side_rst),
        .pclk_out     (pclk        ),
        .mmcm_locked  (hdmi_mmcm_locked),
        .hdmi_i2c_done(hdmi_i2c_done),
        .fb_underflow_sticky(fb_underflow_sticky),
        .fb_reader_req_count(fb_reader_req_count),
        .fb_reader_rsp_count(fb_reader_rsp_count),
        .fb_reader_miss_count(fb_reader_miss_count),
        .debug_hcount (video_debug_hcount),
        .debug_vcount (video_debug_vcount),
        .debug_rgb    (video_debug_rgb),
        .debug_de     (video_debug_de),
        .debug_hs     (video_debug_hs),
        .debug_vs     (video_debug_vs),
        .dbg_video_snap (video_dbg_snap),
        .dbg_video_place(video_dbg_place),
        // VBL → VIA1 CA1 source (declared in fpga_top_cpu.vh).
        .vbl_pulse_pclk(video_vbl_pulse_pclk),

        .al9134_scl    (al9134_scl    ),
        .al9134_sda    (al9134_sda    ),
        .al9134_d      (al9134_d      ),
        .al9134_hs     (al9134_hs     ),
        .al9134_vs     (al9134_vs     ),
        .al9134_de     (al9134_de     ),
        .al9134_clk    (al9134_clk    ),
        .al9134_resetn (al9134_resetn ),
        .al9134_int    (al9134_int    ),

        .scanout_fb_base_px  (dafb_scanout_fb_base_px),
        .scanout_fb_stride_px(dafb_scanout_fb_stride_px),
        .scanout_bpp_shift   (dafb_bpp_shift),
        .scanout_bytes_per_px(dafb_fb_bytes_per_px),
        .scanout_depth_supported (dafb_depth_supported),
        .scanout_hres        (dafb_hres),
        .scanout_vres        (dafb_vres),
        .scanout_clut_we     (dafb_clut_we),
        .scanout_clut_waddr  (dafb_clut_waddr),
        .scanout_clut_wdata  (dafb_clut_wdata),

        .vram_rd_en  (vram_rd_en  ),
        .vram_rd_addr(vram_rd_addr),
        .vram_rd_data(vram_rd_data),
        .vram_rd_valid(vram_rd_valid)
    );

