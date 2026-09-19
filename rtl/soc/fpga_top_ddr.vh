// rtl/fpga_top_ddr.vh — included from rtl/fpga_top.v
//
// DDR controller: SIM_MODEL path uses behavioural ddr_ctrl; real hardware path instantiates the pcie_test DDR4 MIG black box and bridges to ddr_ctrl.
//
// Generated mechanically from rtl/fpga_top.v during the fpga_top split
// (agent/p3-fpga-top-split). No functional change: this file contains
// the original lines verbatim, inside the fpga_top module scope via
// `include. Do not add module/endmodule wrappers or nettype directives.

    // ═══════════════════════════════════════════════════════════════════
    // DDR controller — behavioural for SIM_MODEL; real-hardware DDR4/MIG
    // path when SIM_MODEL is absent.
    // ═══════════════════════════════════════════════════════════════════
    wire ddr_clk_unused;
    wire [31:0] ddr_dbg_aw_cnt, ddr_dbg_w_cnt, ddr_dbg_b_cnt;
    wire [31:0] ddr_dbg_ar_cnt, ddr_dbg_r_cnt;

    // ═══════════════════════════════════════════════════════════════════
    // L2 system cache (T13) — xbar S0 -> l2c -> axi_async_bridge (inside
    // ddr_ctrl below) -> MIG.  Single core_clk domain; no CDC inside l2c
    // itself (docs/l2c_spec.md, fpga_top_ddr.vh integration design #1).
    //
    // Build-gated behind `L2C_ENABLE`, OFF by default: l2c's ~64 URAM288
    // (2 MB, 8-way, docs/l2c_spec.md S3 URAM-budget note) plus the
    // existing 2 MB Q700-silicon VRAM array's ~57 URAM288
    // (rtl/mac/video.v) exceed KU5P's 80 URAM288 budget together — L2C
    // must NOT land in a synth/bitstream build until the VRAM-in-DDR
    // migration frees VRAM's URAM allocation.  synth/vivado.tcl does NOT
    // define L2C_ENABLE (see the commented hook there).  Until then this
    // whole block compiles out and the DDR path below is wired to xbar's
    // s0_* signals exactly as before -- byte-identical preprocessed
    // output with L2C_ENABLE undefined (verified in the T13 report).
    //
    // Reset: core_rst_bank[4] -- the SAME reset bit the xbar instance
    // itself uses for its S0 port (fpga_top_xbar.vh: "AXI xbar instances
    // + DDR arbiter reset", .rst(core_rst_bank[4])).  core_rst_bank[*]
    // are documented sim-neutral copies of the single-wire core_rst
    // (fpga_top_clocks.vh: "every bit holds the identical Q value as the
    // single-wire core_rst"), so this is value-identical to u_ddr's own
    // .rst(core_rst) below and introduces no new reset domain.
    //
    // Reset-walk-vs-watchdog: l2c's reset FSM walks all 4096 sets
    // (~4096 core_clk cycles, docs/l2c_spec.md S7) holding
    // s_axi_awready/arready low the whole time.  The xbar's per-slot
    // watchdog (axi_xbar.v WD_LOG2=18, i.e. 2^18=262144 cycles) only
    // starts counting once a transaction is ACCEPTED (awvalid&&awready);
    // since l2c and the xbar share the identical reset, the xbar cannot
    // have accepted anything into S0 during l2c's post-reset walk (it's
    // still in its own reset/re-arm window too), so the walk never
    // interacts with the watchdog at all. Once accepted, worst-case
    // steady-state front-door stall is ~12 cycles (docs/l2c_spec.md S9
    // Critical-7 retry bound) -- 262144/12 >= 20000x margin.
    //
    // ROM window: axi_defs.vh's ddr_flatten() folds RAM/ROM-mirror/FB
    // uniformly onto the same S0 port (docs/l2c_spec.md S3), so ROM
    // reads DO flow through l2c here -- cached as ordinary clean lines,
    // no special handling needed (matches the spec's S3 finding; ROM
    // writes are already dropped upstream by axi_xbar.v before reaching
    // S0). No parameter override needed for this: l2c's default
    // CACHEABLE_BASE/CACHEABLE_SIZE (0x0000_0000 + 0x4100_0000) already
    // spans AXI_DDR_RAM_OFFSET(0)+AXI_RAM_SIZE(1G) through
    // AXI_DDR_FB_OFFSET(0x4040_0000)+AXI_FB_SIZE(8M) (axi_defs.vh).
    // This paragraph is true for S0 only -- it does NOT cover cpu040's
    // Level-A fetch port, which binds directly to l2c's dedicated
    // f_axi_* port below and never touches S0 or ddr_flatten() at all.
    // See the ifa_araddr_folded comment right before the u_l2c
    // instantiation for that port's own, separately-applied ROM-mirror
    // fold.
    //
    // Bypass windows: left at l2c.v's own defaults (NUM_BYPASS_WINDOWS=1,
    // BYP_WIN_EN=0) -- i.e. disabled, per integration design #4 AND
    // per T16's "decode-level VRAM lane" reshape: T14 briefly wired a
    // real base/mask into this window for the VRAM-in-DDR carveout so
    // l2c would never-allocate CPU writes to VRAM; T16 moves that
    // invariant down to axi_xbar.v's own S3 decode instead (the VRAM
    // aperture routes to a genuine S3 slave again and never reaches
    // l2c's slave port at all -- see axi_vram_priority_mux3.v's header
    // and docs/l2c_spec.md's "Enforcement moved to decode" note), so
    // 2026-09-10: the ONE consumer that window ever had -- the DDR-backed
    // RAM-disk SCSI volume at aperture 0x7000_0000, flattened to DDR
    // 0x5000_0000 -- was REMOVED (owner directive, "vhdd ddr should be
    // killed").  Its aperture, its xbar decode/flatten arms and the
    // {base, mask} pair that carved the flattened range out of the
    // cacheable span went with it, so the window is back to l2c.v's own
    // disabled defaults with nothing left to describe.
    //
    // l2c_bypass.v $fatal-asserts window disjointness from
    // CACHEABLE_BASE/SIZE at elaboration; with EN=0 there is no window to
    // check.
`ifdef L2C_ENABLE
    wire [5:0]   l2c_m_awid;
    wire [31:0]  l2c_m_awaddr;
    wire [7:0]   l2c_m_awlen;
    wire [2:0]   l2c_m_awsize;
    wire [1:0]   l2c_m_awburst;
    wire         l2c_m_awvalid;
    wire         l2c_m_awready;
    wire [127:0] l2c_m_wdata;
    wire [15:0]  l2c_m_wstrb;
    wire         l2c_m_wlast;
    wire         l2c_m_wvalid;
    wire         l2c_m_wready;
    wire [5:0]   l2c_m_bid;
    wire [1:0]   l2c_m_bresp;
    wire         l2c_m_bvalid;
    wire         l2c_m_bready;
    wire [5:0]   l2c_m_arid;
    wire [31:0]  l2c_m_araddr;
    wire [7:0]   l2c_m_arlen;
    wire [2:0]   l2c_m_arsize;
    wire [1:0]   l2c_m_arburst;
    wire         l2c_m_arvalid;
    wire         l2c_m_arready;
    wire [5:0]   l2c_m_rid;
    wire [127:0] l2c_m_rdata;
    wire [1:0]   l2c_m_rresp;
    wire         l2c_m_rlast;
    wire         l2c_m_rvalid;
    wire         l2c_m_rready;
    // 2026-07-24 L2C/MIG-cal-race debug taps (probe41/42 in
    // fpga_top_debug_vio.vh's ENABLE_ILA block) — see l2c_ctrl.v's
    // dbg_write_snap / l2c.v's dbg_master_snap port comments.
    wire [10:0]  dbg_l2c_write_snap;
    wire [10:0]  dbg_l2c_master_snap;
    // 2026-08-30 boot investigation round 8 (docs/
    // BUG_calibration_word_misplaced_0d00.md Part 54's closing
    // recommendation) -- see l2c.v's dbg_fetch_snap port comment.
    wire [15:0]  dbg_l2c_fetch_snap;
    // 2026-08-31 boot investigation round 10 (docs/
    // BUG_calibration_word_misplaced_0d00.md Part 56's closing
    // recommendation) -- see l2c.v's dbg_fetch_id_snap port comment.
    wire [15:0]  dbg_l2c_fetch_id_snap;
    // 2026-07-25 hit/miss/occupancy counters -- see l2c_ctrl.v's own
    // dbg_hit_count/dbg_miss_count/dbg_mshr_occupancy port comments.
    wire [31:0]  dbg_l2c_hit_count;
    wire [31:0]  dbg_l2c_miss_count;
    wire [3:0]   dbg_l2c_mshr_occupancy;

    // ── ROM-mirror decode fold for the direct fetch port ───────────────
    // cpu040's Level-A fetch port (ifa_*) binds STRAIGHT to l2c's
    // dedicated f_axi_* port below, never through axi_xbar.v -- and
    // axi_xbar.v's ddr_flatten() is exactly where this SoC's real ROM-
    // mirror decode lives today (its `is_rom_addr()` branch: any address
    // in the 256 MiB `AXI_ROM_MIRROR_BASE`.. window folds down to its
    // offset within one `AXI_ROM_IMAGE_SIZE` (1 MiB) period, matching the
    // real Q700 ROM chip's own address decode, which WRAPS at that period
    // across the whole 0x4xxx_xxxx span -- MAME:
    // `map(0x40000000, 0x400fffff).mirror(0x0ff00000)`,
    // macquadra700.cpp:557).  A master that never sees ddr_flatten()
    // never sees that fold either, so without this, any fetch at a ROM-
    // mirror alias other than the exact 0x4000_0000 base -- e.g. real Mac
    // OS ROM code legitimately touching 0x4080_0000 (axi_xbar.v:90-92's
    // `MOVE.B D0,(0x80,A3)` example) -- resolves through l2c's tag/set
    // math as a FULLY DISTINCT physical DDR line from the one address
    // boot_fsm.v actually streamed ROM content into at reset, reading
    // uninitialised DDR instead of ROM.
    //
    // Fold it here, once, on the address feeding l2c's fetch port --
    // same arithmetic as ddr_flatten()'s ROM branch (rtl/soc/axi_xbar.v),
    // so the two decodes are trivially diffable against each other if
    // either ever changes. `AXI_DDR_ROM_OFFSET` == `AXI_ROM_MIRROR_BASE`
    // (both 0x4000_0000) on this fetch path -- there is no separate
    // "flattened DDR window" here the way the xbar has one, since l2c's
    // own CACHEABLE_BASE/SIZE (0x0000_0000 + 0x4100_0000, see the ROM
    // window comment on u_l2c below) already addresses RAM and ROM with
    // plain system addresses -- but the `+ offset` term is kept anyway,
    // spelled the same way as ddr_flatten(), rather than special-cased
    // away as a no-op.
    //
    // Scoped strictly to the ROM-mirror window: every other address
    // (RAM, the 0x5800_0000 RAM-probe alias, IO, etc.) passes through
    // ifa_araddr unchanged, exactly as before this fold existed. This
    // wire feeds ONLY l2c's f_axi_araddr port below -- the debug-ctrl
    // JTAG/ILA taps on ifa_araddr (fpga_top_debug_ctrl.vh) and the
    // L2C_ENABLE-undefined xbar M2 fallback (fpga_top_xbar.vh, which
    // already runs the real ddr_flatten() when it applies) both still see
    // the raw, unfolded address, which is what each of those wants.
    wire ifa_araddr_is_rom_mirror =
        ((ifa_araddr & ~(`AXI_ROM_MIRROR_SIZE - 1)) == `AXI_ROM_MIRROR_BASE);
    wire [31:0] ifa_araddr_folded = ifa_araddr_is_rom_mirror ?
        (((ifa_araddr - `AXI_ROM_MIRROR_BASE) & (`AXI_ROM_IMAGE_SIZE - 1)) +
         `AXI_DDR_ROM_OFFSET) :
        ifa_araddr;

    // ── Instruction-fetch window guard (2026-09-09) ────────────────────
    // The fetch master is bound straight to l2c (not the xbar) and l2c has
    // no window decode, so a fetch no slave owns was served from DRAM at
    // (addr mod DRAM size).  Root cause of the p163/p164 "mystery crash"
    // family: the ROM's 24-bit slot map sent a NuBus slot-$B fetch to
    // physical 0xFB0493AA, the L2C returned memory-test filler, the CPU
    // executed it.  The guard answers such fetches with a REAL AXI DECERR
    // (the I-cache raises an instruction-access bus error), exactly what a
    // Quadra 700 does.  Window = visible RAM (cpu_ram_window_lg2, the same
    // runtime size the xbar honours) or the ROM mirror (the identical
    // decode ifa_araddr_folded already uses).  Cacheability still comes
    // from the MMU -- this is a fabric response, not an address backstop.
    wire [CPU_AXI_IW-1:0]  ifg_arid;
    wire [31:0]            ifg_araddr;
    wire [7:0]             ifg_arlen;
    wire [2:0]             ifg_arsize;
    wire [1:0]             ifg_arburst;
    wire                   ifg_arvalid, ifg_arready;
    wire [CPU_AXI_IW-1:0]  ifg_rid;
    wire [255:0]           ifg_rdata;
    wire [1:0]             ifg_rresp;
    wire                   ifg_rlast, ifg_rvalid, ifg_rready;
    wire                   ifg_fault_sticky;
    wire [31:0]            ifg_fault_addr;
    wire [15:0]            ifg_fault_count;
    ifetch_window_guard #(
        .ID_WIDTH(CPU_AXI_IW), .DATA_WIDTH(256)
    ) u_ifetch_guard (
        .clk(core_clk), .rst(core_rst_bank[4]),
        .ram_window_lg2(cpu_ram_window_lg2),
        .s_ar_is_rom(ifa_araddr_is_rom_mirror),
        .s_arid(ifa_arid), .s_araddr(ifa_araddr_folded), .s_arlen(ifa_arlen),
        .s_arsize(ifa_arsize), .s_arburst(ifa_arburst),
        .s_arvalid(ifa_arvalid), .s_arready(ifa_arready),
        .s_rid(ifa_rid), .s_rdata(ifa_rdata), .s_rresp(ifa_rresp),
        .s_rlast(ifa_rlast), .s_rvalid(ifa_rvalid), .s_rready(ifa_rready_gated),
        .m_arid(ifg_arid), .m_araddr(ifg_araddr), .m_arlen(ifg_arlen),
        .m_arsize(ifg_arsize), .m_arburst(ifg_arburst),
        .m_arvalid(ifg_arvalid), .m_arready(ifg_arready),
        .m_rid(ifg_rid), .m_rdata(ifg_rdata), .m_rresp(ifg_rresp),
        .m_rlast(ifg_rlast), .m_rvalid(ifg_rvalid), .m_rready(ifg_rready),
        .fault_sticky(ifg_fault_sticky), .fault_addr(ifg_fault_addr),
        .fault_count(ifg_fault_count)
    );

    l2c #(
        // Bypass windows: left at l2c.v's own defaults
        // (NUM_BYPASS_WINDOWS=1, BYP_WIN_BASE/MASK=0, BYP_WIN_EN=0).
        //
        // The window existed for exactly one consumer, the DDR-backed
        // RAM-disk volume, which was compiled out on 2026-08-19 and
        // DELETED on 2026-09-10 (owner directive).  With the 0x7000_0000
        // aperture and its xbar decode gone there is no flattened range to
        // carve out of the cacheable span any more, so the base/mask
        // overrides that used to sit here are gone too -- there is nothing
        // for them to name.
        //
        // If a future raw-DDR client wants an uncached carve-out, it needs
        // BOTH an xbar decode arm and a matching window here; re-add them
        // together or neither, and note that l2c_bypass's MASK POLARITY IS
        // INVERTED (a 1 bit is a fixed base bit, a 0 bit is in-window
        // offset).
        .EXTERNAL_WRITE_RESET_RECOVERY(1),
        // task #269 (Level A): F_ID_WIDTH matches the CPU socket's native
        // axi_i ID field (CPU_SOCKET_AXI_IW, cpu_socket.vh via
        // fpga_top_cpu.vh's CPU_AXI_IW) so ifa_arid/ifa_rid connect
        // directly below with no truncation/extension at THIS boundary
        // (l2c.v internally zero-extends up to its own ID_WIDTH=6 for
        // l2c_ctrl.v's mirrored-s_ar*-shape fetch sub-port).
        .F_ID_WIDTH(CPU_AXI_IW)
    ) u_l2c (
        .clk(core_clk), .rst(core_rst_bank[4]),
        // AXI4 slave -- from xbar S0 (unchanged shape: 128b data, 6b ID,
        // 32b addr -- matches l2c.v's own defaults, no overrides needed).
        .s_axi_awid(s0_awid), .s_axi_awaddr(s0_awaddr),
        .s_axi_awlen(s0_awlen), .s_axi_awsize(s0_awsize),
        .s_axi_awburst(s0_awburst), .s_axi_awvalid(s0_awvalid),
        .s_axi_awready(s0_awready),
        .s_axi_wdata(s0_wdata), .s_axi_wstrb(s0_wstrb),
        .s_axi_wlast(s0_wlast), .s_axi_wvalid(s0_wvalid),
        .s_axi_wready(s0_wready),
        .s_axi_bid(s0_bid), .s_axi_bresp(s0_bresp),
        .s_axi_bvalid(s0_bvalid), .s_axi_bready(s0_bready),
        .s_axi_arid(s0_arid), .s_axi_araddr(s0_araddr),
        .s_axi_arlen(s0_arlen), .s_axi_arsize(s0_arsize),
        .s_axi_arburst(s0_arburst), .s_axi_arvalid(s0_arvalid),
        .s_axi_arready(s0_arready),
        .s_axi_rid(s0_rid), .s_axi_rdata(s0_rdata),
        .s_axi_rresp(s0_rresp), .s_axi_rlast(s0_rlast),
        .s_axi_rvalid(s0_rvalid), .s_axi_rready(s0_rready),
        // task #269 (Level A): dedicated fetch (axi_i) port, bound
        // directly to the CPU socket's ifa_* wires (fpga_top_cpu.vh) --
        // NOT through the xbar (see the ifdef'd M2 tie-off in
        // fpga_top_xbar.vh).  ifa_rready_gated applies the same
        // JTAG-CPU-reset drain policy the xbar M0/M2 R channels already
        // use (fpga_top_cpu.vh). araddr goes through ifa_araddr_folded,
        // not the raw ifa_araddr, so this port sees the same ROM-mirror
        // decode fold axi_xbar.v's ddr_flatten() applies for xbar-routed
        // masters -- see the ifa_araddr_folded comment above.
        // 2026-09-09: the fetch port now goes through ifetch_window_guard
        // (ifg_* wires, declared above u_l2c) so a fetch outside the visible
        // RAM window / ROM mirror gets a local DECERR instead of DRAM
        // filler -- see u_ifetch_guard and rtl/soc/ifetch_window_guard.v.
        .f_axi_arid(ifg_arid), .f_axi_araddr(ifg_araddr),
        .f_axi_arlen(ifg_arlen), .f_axi_arsize(ifg_arsize),
        .f_axi_arburst(ifg_arburst), .f_axi_arvalid(ifg_arvalid),
        .f_axi_arready(ifg_arready),
        .f_axi_rid(ifg_rid), .f_axi_rdata(ifg_rdata),
        .f_axi_rresp(ifg_rresp), .f_axi_rlast(ifg_rlast),
        .f_axi_rvalid(ifg_rvalid), .f_axi_rready(ifg_rready),
        // AXI4 master -- toward u_ddr below (still core_clk; u_ddr owns
        // the core_clk -> mig_ui_clk CDC internally via axi_async_bridge
        // on the real-hw path, or SIM_MIG_BRIDGE's own chain in sim).
        .m_axi_awid(l2c_m_awid), .m_axi_awaddr(l2c_m_awaddr),
        .m_axi_awlen(l2c_m_awlen), .m_axi_awsize(l2c_m_awsize),
        .m_axi_awburst(l2c_m_awburst), .m_axi_awvalid(l2c_m_awvalid),
        .m_axi_awready(l2c_m_awready),
        .m_axi_wdata(l2c_m_wdata), .m_axi_wstrb(l2c_m_wstrb),
        .m_axi_wlast(l2c_m_wlast), .m_axi_wvalid(l2c_m_wvalid),
        .m_axi_wready(l2c_m_wready),
        .m_axi_bid(l2c_m_bid), .m_axi_bresp(l2c_m_bresp),
        .m_axi_bvalid(l2c_m_bvalid), .m_axi_bready(l2c_m_bready),
        .m_axi_arid(l2c_m_arid), .m_axi_araddr(l2c_m_araddr),
        .m_axi_arlen(l2c_m_arlen), .m_axi_arsize(l2c_m_arsize),
        .m_axi_arburst(l2c_m_arburst), .m_axi_arvalid(l2c_m_arvalid),
        .m_axi_arready(l2c_m_arready),
        .m_axi_rid(l2c_m_rid), .m_axi_rdata(l2c_m_rdata),
        .m_axi_rresp(l2c_m_rresp), .m_axi_rlast(l2c_m_rlast),
        .m_axi_rvalid(l2c_m_rvalid), .m_axi_rready(l2c_m_rready),
        .dbg_ctrl_write_snap(dbg_l2c_write_snap),
        .dbg_master_snap(dbg_l2c_master_snap),
        .dbg_hit_count(dbg_l2c_hit_count),
        .dbg_miss_count(dbg_l2c_miss_count),
        .dbg_mshr_occupancy(dbg_l2c_mshr_occupancy),
        .dbg_fetch_snap(dbg_l2c_fetch_snap),
        .dbg_fetch_id_snap(dbg_l2c_fetch_id_snap)
    );
`endif

`ifndef SIM_MODEL
    wire mig_ui_clk;
    wire mig_ui_rst;
    // 2026-07-24: wired to debug_ila probe43 (fpga_top_debug_vio.vh) for
    // the L2C/MIG-cal-race investigation — correlate against
    // dbg_l2c_write_snap/dbg_l2c_master_snap in the same capture.
    wire mig_cal_done;
    wire mig_dbg_clk;
    wire [511:0] mig_dbg_bus;

    wire [0:0]  mig_awid;
    wire [30:0] mig_awaddr;
    wire [7:0]  mig_awlen;
    wire [2:0]  mig_awsize;
    wire [1:0]  mig_awburst;
    wire        mig_awvalid;
    wire        mig_awready;

    wire [255:0] mig_wdata;
    wire [31:0]  mig_wstrb;
    wire         mig_wlast;
    wire         mig_wvalid;
    wire         mig_wready;

    wire [0:0] mig_bid;
    wire [1:0] mig_bresp;
    wire       mig_bvalid;
    wire       mig_bready;

    wire [0:0]  mig_arid;
    wire [30:0] mig_araddr;
    wire [7:0]  mig_arlen;
    wire [2:0]  mig_arsize;
    wire [1:0]  mig_arburst;
    wire        mig_arvalid;
    wire        mig_arready;

    wire        mig_rready;
    wire [0:0]  mig_rid;
    wire [255:0] mig_rdata;
    wire [1:0]  mig_rresp;
    wire        mig_rlast;
    wire        mig_rvalid;

    design_1_ddr4_0_1 u_mig_ddr4 (
        // sys_rst used to be `~btn[0]` and NOTHING ELSE: the memory
        // controller was the one block in the design reachable only by a
        // physical button press, which made "a VIO reset for the whole
        // design" impossible.  `soc_hard_rst_req` (fpga_top_clocks.vh)
        // adds the board cold-reset pin, btn[3] and the VIO probe; btn[0]
        // is kept as the DDR-only reset it has always been.  The SoC holds
        // itself in reset until re-calibration finishes on its own, via
        // c0_init_calib_complete -> ddr_cal_done -> platform_init_done ->
        // clk_rst's rst_req; core_clk does not come from mig_ui_clk, so
        // the debug hub survives the re-cal.  See that comment block.
        .sys_rst(~btn[0] | soc_hard_rst_req),
        .c0_sys_clk_p(sys_clk_p),
        .c0_sys_clk_n(sys_clk_n),

        .c0_ddr4_act_n(ddr4_act_n),
        .c0_ddr4_adr(ddr4_adr),
        .c0_ddr4_ba(ddr4_ba),
        .c0_ddr4_bg(ddr4_bg),
        .c0_ddr4_cke(ddr4_cke),
        .c0_ddr4_odt(ddr4_odt),
        .c0_ddr4_cs_n(ddr4_cs_n),
        .c0_ddr4_ck_t(ddr4_ck_t),
        .c0_ddr4_ck_c(ddr4_ck_c),
        .c0_ddr4_reset_n(ddr4_reset_n),
        .c0_ddr4_dm_dbi_n(ddr4_dm_dbi_n),
        .c0_ddr4_dq(ddr4_dq),
        .c0_ddr4_dqs_c(ddr4_dqs_c),
        .c0_ddr4_dqs_t(ddr4_dqs_t),

        .c0_init_calib_complete(mig_cal_done),
        .c0_ddr4_ui_clk(mig_ui_clk),
        .c0_ddr4_ui_clk_sync_rst(mig_ui_rst),
        .dbg_clk(mig_dbg_clk),
        .c0_ddr4_aresetn(!mig_ui_rst),

        .c0_ddr4_s_axi_awid(mig_awid),
        .c0_ddr4_s_axi_awaddr(mig_awaddr),
        .c0_ddr4_s_axi_awlen(mig_awlen),
        .c0_ddr4_s_axi_awsize(mig_awsize),
        .c0_ddr4_s_axi_awburst(mig_awburst),
        .c0_ddr4_s_axi_awlock(1'b0),
        .c0_ddr4_s_axi_awcache(4'b0011),
        .c0_ddr4_s_axi_awprot(3'b000),
        .c0_ddr4_s_axi_awqos(4'b0000),
        .c0_ddr4_s_axi_awvalid(mig_awvalid),
        .c0_ddr4_s_axi_awready(mig_awready),

        .c0_ddr4_s_axi_wdata(mig_wdata),
        .c0_ddr4_s_axi_wstrb(mig_wstrb),
        .c0_ddr4_s_axi_wlast(mig_wlast),
        .c0_ddr4_s_axi_wvalid(mig_wvalid),
        .c0_ddr4_s_axi_wready(mig_wready),

        .c0_ddr4_s_axi_bready(mig_bready),
        .c0_ddr4_s_axi_bid(mig_bid),
        .c0_ddr4_s_axi_bresp(mig_bresp),
        .c0_ddr4_s_axi_bvalid(mig_bvalid),

        .c0_ddr4_s_axi_arid(mig_arid),
        .c0_ddr4_s_axi_araddr(mig_araddr),
        .c0_ddr4_s_axi_arlen(mig_arlen),
        .c0_ddr4_s_axi_arsize(mig_arsize),
        .c0_ddr4_s_axi_arburst(mig_arburst),
        .c0_ddr4_s_axi_arlock(1'b0),
        .c0_ddr4_s_axi_arcache(4'b0011),
        .c0_ddr4_s_axi_arprot(3'b000),
        .c0_ddr4_s_axi_arqos(4'b0000),
        .c0_ddr4_s_axi_arvalid(mig_arvalid),
        .c0_ddr4_s_axi_arready(mig_arready),

        .c0_ddr4_s_axi_rready(mig_rready),
        .c0_ddr4_s_axi_rid(mig_rid),
        .c0_ddr4_s_axi_rdata(mig_rdata),
        .c0_ddr4_s_axi_rresp(mig_rresp),
        .c0_ddr4_s_axi_rlast(mig_rlast),
        .c0_ddr4_s_axi_rvalid(mig_rvalid),
        .dbg_bus(mig_dbg_bus)
    );
`endif

`ifdef VRAM_IN_DDR
    // ═══════════════════════════════════════════════════════════════════
    // T16 (decode-vram-lane reshape of T14): the merge point at this same
    // T13/T14 seam, now a full R/W 3-way arbiter (axi_vram_priority_mux3)
    // instead of T14's read-only 2-way mux -- see that module's header
    // for the full priority-contract writeup and which idioms are reused
    // verbatim vs. new. Three sources:
    //   - l2c's master port (or xbar S0 passthrough when L2C_ENABLE is
    //     undefined) -- RAM/ROM/legacy-FB traffic, l2c_* below.
    //   - the VRAM-aperture CPU lane -- xbar's S3 slave port (a genuine,
    //     separate slave again under VRAM_IN_DDR now that axi_xbar.v's
    //     T14 S3->S0 decode fold is reverted), address-translated from
    //     its xbar-zero-based offset to the real DDR carveout address
    //     right here (s3lane_awaddr/araddr below) -- the byte-swap
    //     itself is already done, unconditionally, at the xbar S3
    //     boundary (unchanged T14 placement/semantics; xbar treats S3
    //     identically regardless of VRAM_IN_DDR now).
    //   - scanout_ddr_reader's read-only real-time HDMI-scanout port
    //     (scan_* below, instantiated in fpga_top_video.vh -- its AXI
    //     master ports are declared here since this IS the seam).
    // ═══════════════════════════════════════════════════════════════════
    wire [5:0]   vmux_m_awid;   wire [31:0]  vmux_m_awaddr; wire [7:0] vmux_m_awlen;
    wire [2:0]   vmux_m_awsize; wire [1:0]   vmux_m_awburst; wire vmux_m_awvalid, vmux_m_awready;
    wire [127:0] vmux_m_wdata;  wire [15:0]  vmux_m_wstrb;   wire vmux_m_wlast, vmux_m_wvalid, vmux_m_wready;
    wire [5:0]   vmux_m_bid;    wire [1:0]   vmux_m_bresp;   wire vmux_m_bvalid, vmux_m_bready;
    wire [5:0]   vmux_m_arid;   wire [31:0]  vmux_m_araddr; wire [7:0] vmux_m_arlen;
    wire [2:0]   vmux_m_arsize; wire [1:0]   vmux_m_arburst; wire vmux_m_arvalid, vmux_m_arready;
    wire [5:0]   vmux_m_rid;    wire [127:0] vmux_m_rdata;   wire [1:0] vmux_m_rresp;
    wire vmux_m_rlast, vmux_m_rvalid, vmux_m_rready;

    // scanout_ddr_reader's AXI master port (instantiated in
    // fpga_top_video.vh; declared here so this seam owns the wiring).
    wire [5:0]   scan_arid;   wire [31:0]  scan_araddr; wire [7:0] scan_arlen;
    wire [2:0]   scan_arsize; wire [1:0]   scan_arburst; wire scan_arvalid, scan_arready;
    wire [5:0]   scan_rid;    wire [127:0] scan_rdata;   wire [1:0] scan_rresp;
    wire scan_rlast, scan_rvalid, scan_rready;

    // S3's backend, part 1: source select + address-translate, both owned
    // by axi_vram_smoke_mux (see that module's header).
    //
    // s3_awaddr/s3_araddr (declared in fpga_top_xbar.vh, driven by
    // axi_xbar.v's S3 port) arrive zero-based (vram_flatten() peeled off
    // AXI_VRAM_BASE, same as the URAM slave always saw) -- the mux adds
    // the DDR carveout base to land on the real physical address the VRAM
    // lane needs. Part 2 (byte-swap) already happened inside axi_xbar.v on
    // this same S3 boundary, unconditionally -- nothing to do here for
    // that half, and nothing to do for the smoke arm either, which authors
    // its words in the same post-swap (plain AXI byte-lane) order.
    //
    // The mux ALSO folds in the VIDEO_SMOKE reset-time SMPTE-bar writer,
    // whose instance lives in fpga_top_video.vh (included after this file)
    // and drives the smk_* wires declared just below -- same
    // declared-here / instantiated-there split this seam already uses for
    // scanout_ddr_reader's scan_* master port. Under VRAM_IN_DDR there is
    // no vram.v URAM slave for smoke to write, so routing it through the
    // CPU-path carveout write is the ONLY way it can paint; that was the
    // documented follow-up the old `$fatal` guard in fpga_top_video.vh
    // stood in for.
    wire [5:0]   smk_awid;   wire [31:0]  smk_awaddr; wire [7:0] smk_awlen;
    wire [2:0]   smk_awsize; wire [1:0]   smk_awburst; wire smk_awvalid, smk_awready;
    wire [127:0] smk_wdata;  wire [15:0]  smk_wstrb;   wire smk_wlast, smk_wvalid, smk_wready;
    wire [1:0]   smk_bresp;  wire smk_bvalid, smk_bready;
    // Driven by fpga_top_video.vh's smoke generate block; consumed by
    // fpga_top_debug_vio.vh's smoke-pending indicator.
    wire         smoke_done;
    wire         smoke_active = (VIDEO_SMOKE != 0) && !smoke_done;

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

    axi_vram_priority_mux3 #(
        .ID_WIDTH(6), .ADDR_WIDTH(32), .DATA_WIDTH(128),
        .EXTERNAL_WRITE_RESET_RECOVERY(1)
    ) u_vram_lane_mux (
        .clk(core_clk), .rst(core_rst_bank[4]),
`ifdef L2C_ENABLE
        .l2c_awid(l2c_m_awid), .l2c_awaddr(l2c_m_awaddr), .l2c_awlen(l2c_m_awlen),
        .l2c_awsize(l2c_m_awsize), .l2c_awburst(l2c_m_awburst),
        .l2c_awvalid(l2c_m_awvalid), .l2c_awready(l2c_m_awready),
        .l2c_wdata(l2c_m_wdata), .l2c_wstrb(l2c_m_wstrb), .l2c_wlast(l2c_m_wlast),
        .l2c_wvalid(l2c_m_wvalid), .l2c_wready(l2c_m_wready),
        .l2c_bid(l2c_m_bid), .l2c_bresp(l2c_m_bresp), .l2c_bvalid(l2c_m_bvalid), .l2c_bready(l2c_m_bready),
        .l2c_arid(l2c_m_arid), .l2c_araddr(l2c_m_araddr), .l2c_arlen(l2c_m_arlen),
        .l2c_arsize(l2c_m_arsize), .l2c_arburst(l2c_m_arburst),
        .l2c_arvalid(l2c_m_arvalid), .l2c_arready(l2c_m_arready),
        .l2c_rid(l2c_m_rid), .l2c_rdata(l2c_m_rdata), .l2c_rresp(l2c_m_rresp),
        .l2c_rlast(l2c_m_rlast), .l2c_rvalid(l2c_m_rvalid), .l2c_rready(l2c_m_rready),
`else
        .l2c_awid(s0_awid), .l2c_awaddr(s0_awaddr), .l2c_awlen(s0_awlen),
        .l2c_awsize(s0_awsize), .l2c_awburst(s0_awburst),
        .l2c_awvalid(s0_awvalid), .l2c_awready(s0_awready),
        .l2c_wdata(s0_wdata), .l2c_wstrb(s0_wstrb), .l2c_wlast(s0_wlast),
        .l2c_wvalid(s0_wvalid), .l2c_wready(s0_wready),
        .l2c_bid(s0_bid), .l2c_bresp(s0_bresp), .l2c_bvalid(s0_bvalid), .l2c_bready(s0_bready),
        .l2c_arid(s0_arid), .l2c_araddr(s0_araddr), .l2c_arlen(s0_arlen),
        .l2c_arsize(s0_arsize), .l2c_arburst(s0_arburst),
        .l2c_arvalid(s0_arvalid), .l2c_arready(s0_arready),
        .l2c_rid(s0_rid), .l2c_rdata(s0_rdata), .l2c_rresp(s0_rresp),
        .l2c_rlast(s0_rlast), .l2c_rvalid(s0_rvalid), .l2c_rready(s0_rready),
`endif
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
        .m_awid(vmux_m_awid), .m_awaddr(vmux_m_awaddr), .m_awlen(vmux_m_awlen),
        .m_awsize(vmux_m_awsize), .m_awburst(vmux_m_awburst),
        .m_awvalid(vmux_m_awvalid), .m_awready(vmux_m_awready),
        .m_wdata(vmux_m_wdata), .m_wstrb(vmux_m_wstrb), .m_wlast(vmux_m_wlast),
        .m_wvalid(vmux_m_wvalid), .m_wready(vmux_m_wready),
        .m_bid(vmux_m_bid), .m_bresp(vmux_m_bresp), .m_bvalid(vmux_m_bvalid), .m_bready(vmux_m_bready),
        .m_arid(vmux_m_arid), .m_araddr(vmux_m_araddr), .m_arlen(vmux_m_arlen),
        .m_arsize(vmux_m_arsize), .m_arburst(vmux_m_arburst),
        .m_arvalid(vmux_m_arvalid), .m_arready(vmux_m_arready),
        .m_rid(vmux_m_rid), .m_rdata(vmux_m_rdata), .m_rresp(vmux_m_rresp),
        .m_rlast(vmux_m_rlast), .m_rvalid(vmux_m_rvalid), .m_rready(vmux_m_rready)
    );
`endif

    ddr_ctrl #(
        .DATA_WIDTH (128),
        .XID_WIDTH  (6),
        // 2026-05-22 — Inject SIM_DDR_READ_DELAY cycles of artificial
        // read latency in SIM_MODEL builds to mimic real-MIG variable
        // DDR4 read timing (~30-80 cycles).  Lets sim exercise pipeline
        // races invisible to the zero-latency BRAM model.
`ifdef SIM_DDR_READ_DELAY
        .SIM_DDR_READ_DELAY (`SIM_DDR_READ_DELAY)
`else
        .SIM_DDR_READ_DELAY (0)
`endif
    ) u_ddr (
        .clk(core_clk), .rst(core_rst),

        .ddr_clk     (ddr_clk_unused),
        .ddr_cal_done(ddr_cal_done  ),

`ifdef VRAM_IN_DDR
        // T16: u_ddr's slave port faces the VRAM-lane arbiter's merged
        // master port on ALL FOUR channels now (T14 only re-routed AR/R
        // this way, since its mux was read-only and AW/W/B went straight
        // to l2c/S0 -- see axi_vram_priority_mux3.v's header for why
        // AW/W/B needs the same arbitration now that S3-CPU writes also
        // land on this shared port instead of going around it).
        .awid   (vmux_m_awid   ), .awaddr (vmux_m_awaddr ),
        .awlen  (vmux_m_awlen  ), .awsize (vmux_m_awsize ),
        .awburst(vmux_m_awburst), .awvalid(vmux_m_awvalid),
        .awready(vmux_m_awready),
        .wdata  (vmux_m_wdata  ), .wstrb  (vmux_m_wstrb  ),
        .wlast  (vmux_m_wlast  ), .wvalid (vmux_m_wvalid ),
        .wready (vmux_m_wready ),
        .bid    (vmux_m_bid    ), .bresp  (vmux_m_bresp  ),
        .bvalid (vmux_m_bvalid ), .bready (vmux_m_bready ),
        .arid   (vmux_m_arid   ), .araddr (vmux_m_araddr ),
        .arlen  (vmux_m_arlen  ), .arsize (vmux_m_arsize ),
        .arburst(vmux_m_arburst), .arvalid(vmux_m_arvalid),
        .arready(vmux_m_arready),
        .rid    (vmux_m_rid    ), .rdata  (vmux_m_rdata  ),
        .rresp  (vmux_m_rresp  ), .rlast  (vmux_m_rlast  ),
        .rvalid (vmux_m_rvalid ), .rready (vmux_m_rready ),
`else
`ifdef L2C_ENABLE
        // L2C_ENABLE: u_ddr's slave port faces l2c's master port instead
        // of xbar's S0 directly -- see the u_l2c instance above.  Off-
        // path (VRAM_IN_DDR undefined): byte-identical to pre-T14.
        .awid   (l2c_m_awid   ), .awaddr (l2c_m_awaddr ),
        .awlen  (l2c_m_awlen  ), .awsize (l2c_m_awsize ),
        .awburst(l2c_m_awburst), .awvalid(l2c_m_awvalid),
        .awready(l2c_m_awready),
        .wdata  (l2c_m_wdata  ), .wstrb  (l2c_m_wstrb  ),
        .wlast  (l2c_m_wlast  ), .wvalid (l2c_m_wvalid ),
        .wready (l2c_m_wready ),
        .bid    (l2c_m_bid    ), .bresp  (l2c_m_bresp  ),
        .bvalid (l2c_m_bvalid ), .bready (l2c_m_bready ),
        .arid   (l2c_m_arid   ), .araddr (l2c_m_araddr ),
        .arlen  (l2c_m_arlen  ), .arsize (l2c_m_arsize ),
        .arburst(l2c_m_arburst), .arvalid(l2c_m_arvalid),
        .arready(l2c_m_arready),
        .rid    (l2c_m_rid    ), .rdata  (l2c_m_rdata  ),
        .rresp  (l2c_m_rresp  ), .rlast  (l2c_m_rlast  ),
        .rvalid (l2c_m_rvalid ), .rready (l2c_m_rready ),
`else
        .awid   (s0_awid   ), .awaddr (s0_awaddr ),
        .awlen  (s0_awlen  ), .awsize (s0_awsize ),
        .awburst(s0_awburst), .awvalid(s0_awvalid),
        .awready(s0_awready),
        .wdata  (s0_wdata  ), .wstrb  (s0_wstrb  ),
        .wlast  (s0_wlast  ), .wvalid (s0_wvalid ),
        .wready (s0_wready ),
        .bid    (s0_bid    ), .bresp  (s0_bresp  ),
        .bvalid (s0_bvalid ), .bready (s0_bready ),
        .arid   (s0_arid   ), .araddr (s0_araddr ),
        .arlen  (s0_arlen  ), .arsize (s0_arsize ),
        .arburst(s0_arburst), .arvalid(s0_arvalid),
        .arready(s0_arready),
        .rid    (s0_rid    ), .rdata  (s0_rdata  ),
        .rresp  (s0_rresp  ), .rlast  (s0_rlast  ),
        .rvalid (s0_rvalid ), .rready (s0_rready ),
`endif
`endif

        .dbg_aw_cnt(ddr_dbg_aw_cnt), .dbg_w_cnt(ddr_dbg_w_cnt),
        .dbg_b_cnt (ddr_dbg_b_cnt ), .dbg_ar_cnt(ddr_dbg_ar_cnt),
        .dbg_r_cnt (ddr_dbg_r_cnt )

`ifndef SIM_MODEL
        ,
        .mig_ui_clk(mig_ui_clk),
        .mig_ui_rst(mig_ui_rst),
        .mig_cal_done(mig_cal_done),

        .mig_awid(mig_awid), .mig_awaddr(mig_awaddr),
        .mig_awlen(mig_awlen), .mig_awsize(mig_awsize),
        .mig_awburst(mig_awburst), .mig_awvalid(mig_awvalid),
        .mig_awready(mig_awready),
        .mig_wdata(mig_wdata), .mig_wstrb(mig_wstrb),
        .mig_wlast(mig_wlast), .mig_wvalid(mig_wvalid),
        .mig_wready(mig_wready),
        .mig_bid(mig_bid), .mig_bresp(mig_bresp),
        .mig_bvalid(mig_bvalid), .mig_bready(mig_bready),
        .mig_arid(mig_arid), .mig_araddr(mig_araddr),
        .mig_arlen(mig_arlen), .mig_arsize(mig_arsize),
        .mig_arburst(mig_arburst), .mig_arvalid(mig_arvalid),
        .mig_arready(mig_arready),
        .mig_rready(mig_rready), .mig_rid(mig_rid),
        .mig_rdata(mig_rdata), .mig_rresp(mig_rresp),
        .mig_rlast(mig_rlast), .mig_rvalid(mig_rvalid)
`endif
    );

`ifndef SIM_MODEL
    // synthesis translate_off
    wire _unused_mig_dbg = &{1'b0, mig_dbg_clk, mig_dbg_bus, 1'b0};
    // synthesis translate_on
`endif
