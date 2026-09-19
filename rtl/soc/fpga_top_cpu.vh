// rtl/fpga_top_cpu.vh — included from rtl/fpga_top.v
//
// CPU socket wire declarations: the dual-AXI master pair (axi_i_* → xbar
// M2 instr at AXI_I_DW, axi_d_* → xbar M0 data [merged with boot FSM via a
// cpu_rst-selected mux inside axi_xbar.v — see its header] at AXI_D_DW),
// the CPU-exported debug/control AXI(-Lite) SLAVE (dbg_axi_*), and the
// SoC-fabric control group (reset/window).  The cpu_stub instance itself
// binds in fpga_top_debug_ctrl.vh (after the peripheral bus declares the
// debug-window slave wires).  Reset overlay address policy lives in the
// xbar decoder.  DAFB register-shim AXI wires (platform, not CPU) stay
// here because later includes consume them.
//
// task #266 / SOC-1: AXI_I_DW (axi_i, 256) and AXI_D_DW (axi_d, 128) are
// now independent per-master widths (rtl/soc/cpu_socket.vh).  axi_d
// still binds straight to the xbar's M0 port with no width change.
// axi_i does NOT — the xbar's M2 port (`ifa_*` below) is still natively
// 128-bit (a single `DATA_WIDTH` parameter shared by every xbar master
// and slave, `axi_xbar.v` / `fpga_top_xbar.vh`'s `u_xbar` instance), so
// `ifa_rdata` genuinely widens across that binding today with NO real
// narrowing logic behind it — see the flag at that connection site in
// fpga_top_xbar.vh.  This is SOC-1's still-open second sub-item (see
// cpu040's 2026-08-18-axi-socket-adapter-design.md §5/§11): either widen
// the xbar's CPU-I path to 256b end-to-end, or add a dedicated 256<->128
// narrowing shim ahead of the M2 port.  Neither is implemented here —
// it is a real fabric-architecture decision, not a wire-width bump, and
// is inert today because no CPU is bound to this socket that ever
// drives axi_i_arvalid (cpu_stub ties it low; CPU=m68k is untouched by
// this task).
//
// Do not add module/endmodule wrappers or nettype directives.

`include "cpu_socket.vh"

    // ═══════════════════════════════════════════════════════════════════
    // Forward-declared peripheral IRQ lines + aggregated ipl[2:0]
    // ═══════════════════════════════════════════════════════════════════
    // Wired up further down when the peripherals are instantiated.
    wire        via1_irq_w;
    wire        via2_irq_w;
    wire        scsi_irq_w;
    wire        scc_irq_w;
    wire        asc_irq_w;
    wire        dma_irq_w;    // level 6 — dma_ctrl, task #116
    wire [2:0]  cpu_ipl_periph_w;
    wire [2:0]  cpu_ipl_ext_w;
    // 1-cycle pulse from commit.v when the CPU dispatches an IRQ
    // exception.  Feeds the EXTERNAL irq_agg (in fpga_top_peripherals.vh)
    // so its NMI rising-edge latch clears once the NMI is serviced.
    wire        cpu_ipl_ack_w;

    // ═══════════════════════════════════════════════════════════════════
    // CPU SOCKET: cpu_stub standalone; real CPU via submodule wrapper (CPU=m68k)
    // ═══════════════════════════════════════════════════════════════════
    // The CPU now attaches through the dual-AXI socket defined in
    // rtl/soc/cpu_socket.vh: an instruction-read master (axi_i_*), a
    // data read/write master (axi_d_*), a CPU-exported debug/control
    // AXI(-Lite) SLAVE (dbg_axi_*), the IRQ pair, and a small SoC-fabric
    // control group (reset/window).  No m68k-specific signal crosses
    // this boundary — the legacy if_*/daxi_* surface, the if_to_axi /
    // axi_narrow_to_wide width shims, and debug_ctrl + its ~130 taps all
    // relocate CPU-side (Phase 4).
    //
    // axi_d binds straight to xbar M0 (cpu_d_*) at its native AXI_D_DW
    // (=128), NO SoC-side widen.  axi_i (ifa_* → xbar M2) is AXI_I_DW
    // (=256) at the socket but the xbar's M2 port is still 128b today —
    // see the task #266 / SOC-1 note above and the flag on `ifa_rdata`'s
    // binding in fpga_top_xbar.vh.  The cpu_stub instance itself is
    // bound later in fpga_top_debug_ctrl.vh (after the peripheral bus
    // declares the dbg_core_* debug-window slave wires it terminates and
    // after cpu_ipl_ext_w is resolved); this include only declares the
    // socket wires the xbar (loaded next) and reset tree consume.

    localparam integer CPU_AXI_I_DW = `CPU_SOCKET_AXI_I_DW;
    localparam integer CPU_AXI_D_DW = `CPU_SOCKET_AXI_D_DW;
    localparam integer CPU_AXI_AW = `CPU_SOCKET_AXI_AW;
    localparam integer CPU_AXI_IW = `CPU_SOCKET_AXI_IW;
    localparam integer CPU_DBG_AW = `CPU_SOCKET_DBG_AW;
    localparam integer CPU_DBG_DW = `CPU_SOCKET_DBG_DW;

    // task #270 / SOC-3: CPU=m68k040 (M68kSocketTop, cpu040/) fetches
    // natively at AXI_I_DW=256 with NO CPU-side 256->128 downconverter
    // (cpu040 design doc 2026-08-18-axi-socket-adapter-design.md D11) --
    // it can ONLY run correctly when ifa_* binds to l2c's dedicated 256b
    // fetch port (task #269's `L2C_ENABLE` path, fpga_top_ddr.vh:201).
    // The `L2C_ENABLE`-undefined fallback rebinds ifa_* straight onto the
    // xbar's 128b-native M2 port (fpga_top_xbar.vh:371-386) with an
    // implicit zero-extend/truncate -- silent I-fetch data corruption for
    // this CPU, not merely a missing-cache slowdown the way it is for
    // CPU=stub (inert) or CPU=m68k (v1 presents 128b at the socket, see
    // cpu_socket.vh's AXI_SOCKET_AXI_I_DW note).  The Makefile's
    // CPU=m68k040 branch always forwards `+define+L2C_ENABLE
    // +define+VRAM_IN_DDR` (see "CPU build select"), so this should be
    // unreachable via the normal build path; this is the elaboration-time
    // backstop for anyone who hand-invokes verilator/vivado with
    // CPU_M68K040 defined but L2C_ENABLE undefined.
`ifdef CPU_M68K040
  `ifndef L2C_ENABLE
    `error "CPU=m68k040 requires L2C_ENABLE: axi_i is native 256b and the no-L2C xbar M2 fallback is 128b-only, which would silently truncate every instruction fetch. Build with L2C_ENABLE=1 (the default) or add a real 256->128 downconverter before using L2C_ENABLE=0 with this CPU."
  `endif
`endif

    // ── axi_i_* : CPU instruction read master (ifa_* names) ───────────
    // task #269 (Level A): ifa_* now binds DIRECTLY to l2c's dedicated
    // 256b fetch port (rtl/soc/l2c.v's f_axi_*, wired in fpga_top_ddr.vh)
    // when `L2C_ENABLE` is defined -- which is the shipping default
    // (Makefile `L2C_ENABLE ?= 1`) -- CLOSING the task #266/SOC-1 open
    // gap that used to widen-mismatch this onto the xbar's 128b M2 port.
    // ifa_rdata stays AXI_I_DW (256) end-to-end in that config; no
    // implicit zero-extend anywhere on the read-data path.
    //
    // When `L2C_ENABLE` is UNDEFINED (an explicit opt-out build, e.g.
    // `make impl L2C_ENABLE=0`, or the bare `lint-configs` "default" row)
    // there is no l2c instance to bind to at all, so ifa_* falls back to
    // the xbar's M2 port exactly as before task #269 -- see the ifdef in
    // fpga_top_xbar.vh.  That fallback path is STILL width-mismatched
    // (128b M2 vs. 256b ifa_rdata, implicit zero-extend) exactly as SOC-1
    // originally flagged; task #269 did not touch it, since fixing the
    // no-L2C xbar path would mean widening axi_xbar.v itself (see the
    // task #269 report for why that was out of scope here).
    //
    // Inert under CPU=stub either way (axi_i_arvalid is tied low, so no
    // real fetch ever transfers).
    wire [CPU_AXI_IW-1:0]  ifa_arid;
    wire [CPU_AXI_AW-1:0]  ifa_araddr;
    wire [7:0]             ifa_arlen;
    wire [2:0]             ifa_arsize;
    wire [1:0]             ifa_arburst;
    wire                   ifa_arvalid;
    wire                   ifa_arready;
    wire [CPU_AXI_IW-1:0]  ifa_rid;
    wire [CPU_AXI_I_DW-1:0] ifa_rdata;
    wire [1:0]             ifa_rresp;
    wire                   ifa_rlast;
    wire                   ifa_rvalid;
    wire                   ifa_rready;
    // JTAG CPU reset must not stall the shared fabric (xbar/DDR OR the
    // direct l2c fetch port, whichever ifa_* is bound to below): drain
    // any already-accepted I-read response while the CPU is held in
    // reset.  Was named ifa_rready_xbar; renamed since task #269 gave it
    // a second consumer.
    wire                   ifa_rready_gated = cpu_rst ? 1'b1 : ifa_rready;

    // ── axi_d_* : CPU data read/write master → xbar M0 (cpu_d_* names) ─
    wire [CPU_AXI_IW-1:0]  cpu_d_awid;
    wire [CPU_AXI_AW-1:0]  cpu_d_awaddr;
    wire [7:0]             cpu_d_awlen;
    wire [2:0]             cpu_d_awsize;
    wire [1:0]             cpu_d_awburst;
    wire                   cpu_d_awvalid;
    wire                   cpu_d_awready;
    wire [CPU_AXI_D_DW-1:0]  cpu_d_wdata;
    wire [CPU_AXI_D_DW/8-1:0] cpu_d_wstrb;
    wire                   cpu_d_wlast;
    wire                   cpu_d_wvalid;
    wire                   cpu_d_wready;
    wire [CPU_AXI_IW-1:0]  cpu_d_bid;
    wire [1:0]             cpu_d_bresp;
    wire                   cpu_d_bvalid;
    wire                   cpu_d_bready;
    wire [CPU_AXI_IW-1:0]  cpu_d_arid;
    wire [CPU_AXI_AW-1:0]  cpu_d_araddr;
    wire [7:0]             cpu_d_arlen;
    wire [2:0]             cpu_d_arsize;
    wire [1:0]             cpu_d_arburst;
    wire                   cpu_d_arvalid;
    wire                   cpu_d_arready;
    wire [CPU_AXI_IW-1:0]  cpu_d_rid;
    wire [CPU_AXI_D_DW-1:0]  cpu_d_rdata;
    wire [1:0]             cpu_d_rresp;
    wire                   cpu_d_rlast;
    wire                   cpu_d_rvalid;
    wire                   cpu_d_rready;
    // Same JTAG-CPU-reset drain policy on the data B/R channels.
    wire                   cpu_d_bready_xbar = cpu_rst ? 1'b1 : cpu_d_bready;
    wire                   cpu_d_rready_xbar = cpu_rst ? 1'b1 : cpu_d_rready;

    // ── dbg_axi_* : CPU-exported debug/control AXI(-Lite) SLAVE ───────
    // Bound to the SoC's JTAG-AXI debug window (dbg_core_*) in
    // fpga_top_debug_ctrl.vh.  Declared here so the bind site (loaded
    // after the peripheral bus) and this include agree on widths.
    wire [CPU_DBG_AW-1:0]  cpu_dbg_axi_awaddr;
    wire                   cpu_dbg_axi_awvalid;
    wire                   cpu_dbg_axi_awready;
    wire [CPU_DBG_DW-1:0]  cpu_dbg_axi_wdata;
    wire [CPU_DBG_DW/8-1:0] cpu_dbg_axi_wstrb;
    wire                   cpu_dbg_axi_wvalid;
    wire                   cpu_dbg_axi_wready;
    wire [1:0]             cpu_dbg_axi_bresp;
    wire                   cpu_dbg_axi_bvalid;
    wire                   cpu_dbg_axi_bready;
    wire [CPU_DBG_AW-1:0]  cpu_dbg_axi_araddr;
    wire                   cpu_dbg_axi_arvalid;
    wire                   cpu_dbg_axi_arready;
    wire [CPU_DBG_DW-1:0]  cpu_dbg_axi_rdata;
    wire [1:0]             cpu_dbg_axi_rresp;
    wire                   cpu_dbg_axi_rvalid;
    wire                   cpu_dbg_axi_rready;

    // ── SoC-fabric control group (CPU → SoC) ──────────────────────────
    // The CPU's debug-CSR (reached via dbg_axi) owns these; the SoC reset
    // tree + xbar consume them.  They REPLACE the discrete debug_ctrl
    // outputs (dbg_cold_reset_pulse / dbg_cold_reset_hold / dbg_ram_
    // window_lg2).  cpu_stub drives them idle (except cpu_mon_sense, whose
    // idle value would be a DIFFERENT monitor — see below); the real CPU
    // drives them from its debug-CSR block over dbg_axi.
    wire                   cpu_cold_reset_pulse;
    wire                   cpu_cold_reset_hold;
    wire                   cpu_peripheral_reset;
    // Driven by peripheral_reset_sequencer in fpga_top_sd.vh after the
    // storage-busy signal is available.  Earlier include slices consume it.
    wire                   warm_peripheral_reset;
    wire                   warm_storage_reset;
    wire [5:0]             cpu_ram_window_lg2;
    // DAFB monitor-sense code (CPU debug-CSR OFF_MON_SENSE 0x0005C).  Was
    // video.v's compile-time MONITOR_TYPE parameter; runtime-settable over
    // JTAG now, and it survives a CPU reset because the CSR lives in the
    // CPU's POR-only debug reset domain.  Bit 6 = "extended monitor" flag,
    // bits [5:0] = code — all seven bits go to u_dafb, which interprets
    // them in sense_response().  cpu_stub drives 7'h06 (the historical
    // parameter default) so a standalone SoC build is unchanged.
    wire [6:0]             cpu_mon_sense;

    // Map socket control outputs onto the SoC reset-tree / xbar wires the
    // fabric already references (declared in fpga_top_clocks.vh /
    // fpga_top_debug_ctrl.vh).  dbg_soft_rst is the deprecated legacy
    // alias (no live SoC consumer); tie it low.
    assign dbg_cold_reset_pulse = cpu_cold_reset_pulse;
    assign dbg_cold_reset_hold  = cpu_cold_reset_hold;
    assign dbg_soft_rst         = 1'b0;
    assign dbg_ram_window_lg2   = cpu_ram_window_lg2;


    // ═══════════════════════════════════════════════════════════════════
    // DAFB register shim AXI wiring
    // ═══════════════════════════════════════════════════════════════════
    // The live DAFB register window is decoded by the system AXI fabric
    // (xbar S4 -> AXI-Lite bridge), not intercepted on the CPU-only path.
    // That makes CPU and host/JTAG/XDMA writes to 0xF980_0000 share the
    // same live base/stride/BPP/CLUT state.
    wire [31:0] dafb_awaddr;
    wire        dafb_awvalid;
    wire        dafb_awready;
    wire [31:0] dafb_wdata;
    wire [3:0]  dafb_wstrb;
    wire        dafb_wvalid;
    wire        dafb_wready;
    wire [1:0]  dafb_bresp;
    wire        dafb_bvalid;
    wire        dafb_bready;
    wire [31:0] dafb_araddr;
    wire        dafb_arvalid;
    wire        dafb_arready;
    wire [31:0] dafb_rdata;
    wire [1:0]  dafb_rresp;
    wire        dafb_rvalid;
    wire        dafb_rready;
    wire [31:0] dafb_fb_base_px;
    wire [31:0] dafb_fb_stride_px;
    wire [31:0] dafb_fb_bpp_reg;
    wire [2:0]  dafb_bpp_shift;
    wire [2:0]  dafb_fb_bytes_per_px;
    wire        dafb_depth_supported;
    wire [11:0] dafb_hres;
    wire [11:0] dafb_vres;
    // 256-entry RAMDAC CLUT write export (core_clk domain) -- see
    // video.v / linebuf_scanout.v headers.  Replaces the old 384-bit
    // flat 16-entry clut_rgb bus.
    wire        dafb_clut_we;
    wire [7:0]  dafb_clut_waddr;
    wire [23:0] dafb_clut_wdata;
    wire [15:0]  fb_reader_req_count;
    wire [15:0]  fb_reader_rsp_count;
    wire [15:0]  fb_reader_miss_count;
    wire        dafb_irq_w;
    // pclk-domain VBL pulse from u_video_top, fed to the pulse_cdc that
    // drives VIA1 CA1 in fpga_top_peripherals.vh.  Declared here so the
    // peripherals include (loaded before fpga_top_video.vh) can reference
    // it.  Task #145.
    wire        video_vbl_pulse_pclk;
    wire [FB_ADDR_W-1:0] dafb_scanout_fb_base_px =
        (|dafb_fb_base_px[31:FB_ADDR_W]) ? FB_INVALID_SCANOUT_ADDR
                                         : dafb_fb_base_px[FB_ADDR_W-1:0];
    wire [FB_ADDR_W-1:0] dafb_scanout_fb_stride_px =
        (|dafb_fb_stride_px[31:FB_ADDR_W]) ? FB_INVALID_SCANOUT_ADDR
                                           : dafb_fb_stride_px[FB_ADDR_W-1:0];
