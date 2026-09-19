// axi_xbar.v — 3-master × 6-slave AXI4 system crossbar
//
// 2026-07-16 master-count reduction (task: "reduce AXI system crossbar
// master count").  Was 5 masters (M0 CPU LSU, M1 host debug, M2 boot FSM,
// M3 CPU IF, M4 DMA).  Per the maintainer: CPU LSU and CPU IF MUST stay
// independent masters (non-negotiable); host debug and DMA MUST stay
// independent from each other; boot FSM and CPU LSU are the one pair that
// is safe and worth merging, because their active windows are provably
// disjoint (see "M0/boot merge" below).  DMA currently has zero live
// consumers (nothing in the design triggers a dma_ctrl transfer today —
// confirmed by a feasibility pass this session), so rather than give it a
// permanent xbar seat it never uses, its AXI4 *master* port is stubbed out
// of the crossbar entirely.  `rtl/soc/dma_ctrl.v` itself is untouched and
// still fully unit-tested (`tb-dma-ctrl`); only the wiring from its
// `m_axi_*` port into this crossbar is removed (see `fpga_top_dma.vh`).
// Its AXI4-Lite config slave port (S2, below) is kept live — CPU/debug can
// still program DMA descriptors; they just won't go anywhere until a real
// consumer re-wires M4 in a future session.
//
// Topology
// ────────
//     ┌─ M0  CPU LSU  ──┐                      ┌─► S0  DDR  (RAM+ROM+FB)
//     │  (+ boot FSM,   │                      │
//     │   reset-hold    │──► [ AXI4 XBAR ] ────┼─► S1  IO   (GLUE → pb_*)
//     │   mux — below)  │                      │
//     ├─ M1  host debug ┤                      ├─► S2  DMA  (dma_ctrl AXI-Lite cfg)
//     │                 │                      │      — config slave only;
//     └─ M2  CPU IF  ───┘                      │        M4 master stubbed, see above
//        (read-only)                           ├─► S3  VRAM (URAM pixel aperture)
//                                               │
//                                               └─► S4  DAFB register shim
//                                               └─► S5  SD JTAG writer
//
// M0/boot merge — reset-hold mux, NOT an arbiter
//   Boot FSM (`rtl/soc/boot_fsm.v`) is WRITE-ONLY and streams SD sectors +
//   a RAM zero-pass into DDR while the CPU is held in reset
//   (`cpu_rst`/`cpu_held_in_reset` asserted).  It reaches its terminal
//   ST_DONE state — and asserts `rom_loaded`, the signal that ungates
//   `cpu_rst` — only after its last write's BRESP has been observed and
//   no AW/W is outstanding (`boot_fsm.v` ST_AXI_B_WAIT / ST_ZERO_B gates).
//   `cpu_rst` itself then stays asserted for a further settle window
//   (`CPU_RST_SETTLE_CYCLES` + an 8-cycle minimum-pulse stretch, see
//   `fpga_top_clocks.vh`) after boot_rom_loaded fires, so there is a
//   comfortable dead-time gap, not a knife-edge race.  Symmetrically, any
//   `soc_full_rst` (cold/debug-full-reset) that re-arms the CPU's reset
//   also re-arms `boot_fsm_rst`, so the two windows are re-synchronised on
//   every reset event, not just the very first one.  Net effect: boot FSM
//   traffic and CPU LSU traffic are NEVER simultaneously live on the
//   shared physical port — this is provable from the reset sequencing,
//   not an assumption papered over with arbitration.  Because of that, the
//   merge at slot 0's fan-in below is a plain 2:1 mux, not a round-robin
//   arbiter: whichever side is deselected is guaranteed idle (awvalid/wvalid
//   low) by construction, so there is no starvation/fairness concern to
//   arbitrate away.
//
//   The mux select is `m0_wsel_q`, a REGISTER that tracks
//   `cpu_held_in_reset` while slot 0 is idle and freezes for the life of a
//   slot-0 write — NOT `cpu_held_in_reset` itself.  The reset sequencing
//   above bounds when each side is *live*, but it does not bound when
//   `cpu_held_in_reset` may *change*: JTAG DBG_CONTROL bit 4 raises it at
//   an arbitrary cycle with this module's own `rst` low.  A combinational
//   select therefore re-selected mid-transaction and handed an in-flight
//   burst's remaining W beats (and its B) to the other side.  See the long
//   note at the `mw_*[0]` assigns for the full mechanism.
//   `mw_midx[0]` (the "who is this write" tag consumed by the ROM-write
//   policy below) tracks the same mux select, so the existing "boot may
//   write ROM, CPU LSU may not" policy transfers unchanged onto the
//   shared slot — see `is_rom_loader_w` and the ROM-write comment in M0
//   below.
//
// Internal-arbiter note: the write/read fan-in arrays (NW=4, NR=4 local
// params) and the round-robin picker (`rr_pick_w`/`rr_pick_r`, hardcoded
// to 4 named inputs) are UNCHANGED by this master-count reduction — only
// 2 of 4 write slots and 3 of 4 read slots have a live top-level port now;
// the rest are permanently tied to "never valid".  This was a deliberate
// choice to avoid resizing load-bearing, hand-written round-robin
// arbitration logic that is not parameterized over width; the actual
// synthesis/integration win from this task (fewer external AXI4 master
// interfaces, fewer register slices/adapters in `fpga_top.v`) does not
// require shrinking the internal arbiter.
//
// Master roles
//   M0: CPU LSU + boot FSM, time-multiplexed by `cpu_held_in_reset` (see
//                  above).  Reads always come from the CPU LSU side only
//                  (boot FSM is write-only).  Writes to ROM: allowed
//                  (land in DDR, ROM image staging) while presenting as
//                  boot FSM; silently dropped with BRESP=OKAY while
//                  presenting as CPU LSU — matches MAME-canonical Q700
//                  behaviour where Mac OS probes/touches ROM mirror
//                  during boot and expects no fault (e.g. ROM 0x40899ABA
//                  `MOVE.B D0,(0x80,A3)` with A3=0x40800000 writes 0x47 to
//                  0x40800080).
//   M1: Host debug AXI peer (XDMA or JTAG-to-AXI).  Reads + writes
//                  everywhere, including the ROM window for first-board
//                  ROM patch/load iteration while the CPU is held in reset.
//                  Kept independent from DMA (M4, stubbed) per the
//                  maintainer — debug/JTAG access and bulk data-movement
//                  DMA are different enough in purpose/timing that merging
//                  them isn't worth the complexity, especially for a DMA
//                  path with no live consumer yet.
//   M2: CPU instruction fetch.  READ ONLY — independent from CPU LSU
//                  (M0).  This separation is non-negotiable per the
//                  maintainer regardless of any other consolidation.  Its
//                  write ports are tied off; reads anywhere are allowed
//                  through the same decode policy as the LSU master.
//   M3: dma_engine — the shared DMA engine (rtl/soc/dma_engine.v,
//                  instantiated in fpga_top_dma.vh).  Reads and writes
//                  wherever its clients point it.  The seat was originally
//                  taken by vhdd_ddr, the DDR-backed RAM-disk SCSI volume;
//                  that volume was deleted on 2026-09-10 (owner directive)
//                  and the DMA engine now holds M3 unconditionally.
//                  2026-08-03: this reoccupies the write/read fan-in slot
//                  index 3 that the 2026-07-16 master-count reduction left
//                  permanently tied off when dma_ctrl's M4 was stubbed.
//                  Nothing about the round-robin arbiter changed — the
//                  slot was already there and already arbitrated, it just
//                  had no top-level port.  Its fan-in tag stays
//                  `XBAR_M_DMA`, which means `is_rom_loader_w` is false
//                  for it, i.e. a stray write to the ROM mirror from this
//                  master is silently dropped with OKAY like the CPU's —
//                  the conservative default, and unreachable in practice.
//
// Slave roles
//   S0: DDR port.  Single physical DDR4 MIG channel.  Address-flattened:
//       RAM, the Q700 0x5800_0000 RAM probe alias, ROM mirrors, and FB
//       regions map to contiguous DDR space.
//   S1: Peripheral bus (GLUE front door).  AXI4-Lite-ish.
//   S2: DMA config (1 MB AXI4-Lite window, bridged via axi_wide_to_axilite).
//       Live regardless of the M4 master-port stub above — any master can
//       still program dma_ctrl's descriptor registers through here; they
//       just won't produce bus traffic until a real consumer re-wires M4.
//   S3: VRAM pixel aperture (0xF900_0000..0xF90F_FFFF).  The xbar peels
//       off AXI_VRAM_BASE before forwarding so the vram module sees a
//       zero-based byte offset — matching rtl/sys/vram.v's expectation
//       documented in its header ("FB_BASE is peeled off BY THE XBAR").
//       Additionally, S3 byte-lane ordering is little-endian within each
//       32-bit word, while the CPU/system bus presents normal 68k
//       big-endian byte lanes.  The xbar therefore byte-swaps each 32-bit
//       word and reverses each 4-bit WSTRB nibble on the S3 boundary so
//       byte-addressed CPU reads/writes still land on the intended pixel.
//       Added in task #147 (xbar-vram-wire) — before that the VRAM AXI
//       slave was driven only by the reset-time VIDEO_SMOKE writer, and
//       CPU writes to the pixel aperture landed in DECERR.
//   S4: DAFB register shim (0xF980_0000..0xF980_03FF).  The xbar peels
//       off AXI_DAFB_BASE before forwarding so the shim sees local
//       register offsets.
//   S5: SD JTAG writer (0x50A0_0000..0x50A0_FFFF).  The xbar peels
//       off AXI_SD_JTAG_BASE before forwarding so the writer sees local
//       register offsets.
//
// Address map — see rtl/soc/axi_defs.vh.  DMA config (S2) is a hole inside
// IO: decode checks DMA first, then IO.  VRAM lives in its own 0xF9
// window and does not overlap any other slave.
//
// Burst legalization + slave watchdog/poison (task: xbar-burst-guard)
// ─────────────────────────────────────────────────────────────────
//   S1 (peripheral bus / GLUE), S2 (DMA cfg, axi_wide_to_axilite bridge),
//   S4 (DAFB shim) and S5 (SD JTAG writer) are single-beat-only —
//   `axi_wide_to_axilite` accepts exactly one W beat and peripheral_bus
//   always drives RLAST=1.  S0 (DDR) and S3 (VRAM) are burst-capable.
//   `is_lite_only_slv()` below is the single source of truth for that
//   split.  A burst (awlen/arlen > 0) decoded onto a lite-only slave is
//   NOT forwarded: writes are routed through the existing local-drain
//   machinery (WS_DRAIN_W consumes every W beat through WLAST, then
//   BRESP=SLVERR); reads are answered by the existing local-R machinery
//   (RS_SEND_RLOCAL, which already sends len+1 beats via `rs_bcnt`) with
//   every beat RRESP=SLVERR and RLAST on the last one.
//
//   Separately, each side keeps a per-outstanding-transaction cycle
//   counter (`ws_wd_cnt` / `rs_wd_cnt`) covering AW/AR-accepted-by-xbar
//   through B/RLAST.  If a slave never completes within WD_MAX_CNT+1
//   (2^16) cycles, the xbar aborts: it synthesizes the terminal
//   response itself (B=SLVERR, or the remaining R beats as SLVERR with
//   RLAST), releases the write-side `sw_owned` lock, and latches that
//   slave "poisoned" — every subsequent read AND write to that slave
//   (from any master) is answered locally with SLVERR until the next
//   `rst`, never forwarded again.  The poison flag is the mechanism that
//   makes a late response from the wedged slave safe to ignore: once
//   poisoned, `sw_owned`/`rs_state` for that slave are never re-armed
//   against the real slave port, so a stale S*_bvalid/S*_rvalid can
//   never be mis-routed onto a new transaction (see `slv_poisoned`
//   comment below for the full argument).  Kept as two single-driver
//   regs (`slv_w_poisoned` written only by the write always-block,
//   `slv_r_poisoned` only by the read one) OR'd together at use sites,
//   rather than one reg driven from both always blocks.
//
// Abandoned-B master walk-away (task #219, 2026-08-02)
// ────────────────────────────────────────────────────
//   A THIRD failure, distinct from both of the above: the slave answers
//   normally, but the OWNING MASTER never asserts BREADY again.  That is
//   what `axi_narrow_to_wide.v` produced whenever its own abandonment
//   watchdog force-cleared `aw_valid_q` (its `w_bready` is
//   `(aw_valid_q && n_bready) || aw_drain_q`, so it goes low and stays
//   low), and the JTAG-AXI host sits behind exactly such an instance.
//   The MINOR-4 suppression below correctly keeps the POISONING
//   watchdog off once a slave has answered — but nothing else covered
//   this case, so `sw_owned[slv]` was pinned at 1 FOREVER and that slave
//   was dead to every master until the next `rst`.  Observed on hardware
//   as `run_hw_axi` blocking with no output, recoverable only by reload.
//   A B that has sat unaccepted for `B_HOLD_LOG2` cycles AND blown the
//   ordinary watchdog deadline now takes the `ws_b_abandon` path, which
//   retires the dangling B at the slave, releases the lock and idles it —
//   WITHOUT poisoning (the slave is healthy) and without synthesizing a
//   local B (nobody is listening, and a stale one would only alias onto
//   that master's next transaction).  The read side never had this bug:
//   `rs_wd_fire` suppresses on an actual RVALID&&RREADY handshake.
//
// Abandoned-burst W padding (task: xbar-burst-gaps)
// ─────────────────────────────────────────────────
//   The watchdog above is for a WEDGED SLAVE.  A write burst whose MASTER
//   goes away mid-burst is a different failure and must NOT be left to it:
//   slot 0 is the CPU-LSU / boot-FSM shared port, so a JTAG `reset hold`
//   landing mid-writeback strands the slot in WS_FWD_W, and 2^WD_LOG2
//   cycles later the watchdog poisons that slave — S0, i.e. the whole DDR
//   path — for every master until the next `rst`.  Suppressing the poison
//   would be worse, not better: the slave really is parked mid-burst, so
//   the next write's beats would be consumed as the stale burst's
//   continuation and commit to the OLD address.  Instead `WS_PAD_W` +
//   `ws_wbeats` complete the burst here with wstrb=0 filler beats and a
//   correct WLAST, then swallow the resulting B (`m0_abandon` forces
//   BREADY and masks both masters' BVALID) and release `sw_owned` cleanly.
//   Same idiom as axi_bridge_w_pad.v (one hop further out),
//   l2c_victim.v's S_RSTDRAINW and axi_ddr4_mig_bridge.v's wr_pad_active.
//
// Abandonment always produces a response — audit, 2026-08-02
// ─────────────────────────────────────────────────────────
//   Companion audit to the cpu-side change that made
//   `axi_narrow_to_wide.v` answer ITS abandoned transactions with a
//   well-formed SLVERR instead of nothing at all (it used to gate
//   n_bvalid on aw_valid_q and n_rvalid on ar_valid_q, both of which its
//   watchdog force-cleared, so an abandoned transaction was a permanent
//   hang rather than a recovery).  Every abandonment path in THIS module
//   was re-checked against the same contract — "no lock is released, and
//   no transaction is dropped, without the waiting master receiving a
//   terminating response".  Result, path by path:
//
//     ws_wd_fire   -> ws_release_slot -> WS_DRAIN_W (consumes the
//                     master's remaining W beats through WLAST) then
//                     WS_SEND_BLOCAL, or WS_SEND_BLOCAL directly.
//                     One B, BRESP=SLVERR.  CONFORMS.
//     rs_wd_fire   -> rs_release_slot -> RS_SEND_RLOCAL seeded with
//                     rs_wd_remaining = (rs_len+1) - rs_beats_done,
//                     computed in 9 bits and clamped to >= 1, so beats
//                     already delivered are never re-sent and the burst
//                     always terminates with an RLAST.  Every
//                     synthesized beat carries RRESP=SLVERR.  CONFORMS.
//     ws_flush_abort / rs_flush_abort
//                  -> the same two tasks with poison=0.  CONFORMS.
//
//     ws_b_abandon -> deliberately synthesizes NOTHING, and this is
//                     CORRECT — do not "fix" it to match the others.
//                     The B it retires is the SLAVE's real B for a
//                     transaction the master has provably walked away
//                     from; that master is an axi_narrow_to_wide whose
//                     own watchdog already answered its narrow side.
//                     A second B here would be taken as the response to
//                     that master's NEXT write.  The cpu-side change
//                     makes this path rarer still, because the adapter
//                     now holds w_bready high through its drain window
//                     and consumes the dangling B itself.
//     ws_release_slot's slot-0 `m0_abandon || (m0_wsel_q !=
//     cpu_held_in_reset)` discard, and rs_release_slot's slot-0
//     `cpu_held_in_reset` discard
//                  -> also deliberately silent: the owning side is in
//                     reset and cannot consume a response; a local one
//                     would alias onto whichever side is live next.
//
//   Coverage for the conforming paths already exists in tb-axi-xbar
//   (scenarios 28/29 burst-reject, 30 write watchdog, 32 read watchdog,
//   33 mid-burst W-wedge, 34 mid-burst R-wedge "exactly the missing 5
//   beats, all SLVERR, RLAST on the last", plus a 256-beat case), and
//   for the deliberate silence in scenario 29b, which has its own
//   positive control.  No RTL change was required in this module.
//
// Verilog-2005, synchronous active-high rst, 4-space indent, no latches.

`include "axi_defs.vh"

module axi_xbar #(
    parameter DATA_WIDTH  = 128,
    parameter STRB_WIDTH  = DATA_WIDTH/8,
    parameter ID_WIDTH    = 4,
    parameter XID_WIDTH   = ID_WIDTH + 2,
    // NOTE: N_MASTERS / N_SLAVES are documentation-only — nothing inside
    // this module indexes off them (the write/read fan-in arrays are
    // fixed-width local params NW/NR/NS below, sized for the internal
    // round-robin arbiter's hardcoded 4-way pickers; see the master-count
    // reduction header note above).  Keep them accurate for readers.
    parameter N_MASTERS   = 3,
    parameter N_SLAVES    = 6,
    parameter FB_BASE     = `AXI_FB_BASE,
    parameter FB_SIZE     = `AXI_FB_SIZE,
    parameter RAM_BASE    = `AXI_RAM_BASE,
    parameter RAM_SIZE    = `AXI_RAM_SIZE,
    parameter RAM_ALIAS_BASE = `AXI_RAM_ALIAS_BASE,
    parameter RAM_ALIAS_SIZE = `AXI_RAM_ALIAS_SIZE,
    parameter ROM_BASE    = `AXI_ROM_BASE,
    parameter ROM_SIZE    = `AXI_ROM_SIZE,
    // Mirror PERIOD (1 MiB), distinct from the DDR reservation above.
    parameter ROM_IMAGE_SIZE = `AXI_ROM_IMAGE_SIZE,
    parameter ROM_MIRROR_BASE = `AXI_ROM_MIRROR_BASE,
    parameter ROM_MIRROR_SIZE = `AXI_ROM_MIRROR_SIZE,
    parameter IO_BASE     = `AXI_IO_BASE,
    parameter IO_SIZE     = `AXI_IO_SIZE,
    parameter DMA_BASE    = `AXI_DMA_BASE,
    parameter DMA_SIZE    = `AXI_DMA_SIZE,
    parameter SD_JTAG_BASE = `AXI_SD_JTAG_BASE,
    parameter SD_JTAG_SIZE = `AXI_SD_JTAG_SIZE,
    parameter VRAM_BASE   = `AXI_VRAM_BASE,
    parameter VRAM_SIZE   = `AXI_VRAM_SIZE,
    parameter DAFB_BASE   = `AXI_DAFB_BASE,
    parameter DAFB_SIZE   = `AXI_DAFB_SIZE,
    // RAM_ALIAS_MODE controls the response to RAM-range accesses past the
    // visible window (`ram_window_lg2`, default 4 MiB to match the
    // Q700-default SIMM-empty config).
    //   1 — silently mask the upper address bits and re-route into the
    //       window (every out-of-window access succeeds, observed at a
    //       wrap-around DDR offset).  Useful for tests that assume a
    //       large RAM but only want a small physical buffer in DDR.
    //   0 — out-of-window accesses fall through `decode_slv` to NONE,
    //       which responds with **OKAY + 0x00000000** (open bus,
    //       MAME-canonical Q700 semantics).  Reads return all-zeros,
    //       writes are silently dropped.  This matches MAME's actual
    //       default `set_unmap_value` (= 0), which is what the Q700
    //       driver inherits since `macquadra700.cpp` never overrides
    //       it.  Earlier revisions in this tree returned 0xFFFFFFFF
    //       on the (incorrect) theory that MAME's default unmap
    //       value was all-ones; that observation has been corrected
    //       against MAME's source.  Even earlier revisions returned
    //       local DECERR; that also diverged from MAME and broke
    //       the SIMM-detect loop in a different way.
    parameter RAM_ALIAS_MODE = 0,
    // Per-slot watchdog timeout, expressed as log2(cycles) (review round
    // 2, IMPORTANT).  Default 18 (2^18 = 262144 cycles, ~2.6 ms @
    // 100 MHz): chosen to sit ABOVE VRAM's own post-reset URAM-wipe
    // latency (~131k cycles — vram.v holds off AXI acceptance during the
    // wipe, which would otherwise poison a perfectly healthy, merely-
    // busy S3 at the old 2^16 bound) and BELOW the peripheral-bus-side
    // watchdog order of magnitude (2^24) so this remains the FIRST
    // watchdog to fire, not a redundant second one racing a slower
    // outer timeout.  Override to something small (e.g. 8-10) with a
    // -G parameter override on the Verilator build command line for
    // fast unit-tb coverage of the timeout paths — see
    // tb/tb_axi_xbar.cpp and the tb-axi-xbar Makefile rule.
    parameter WD_LOG2 = 18,
    // Per-slave override for S1 (peripheral_bus / IO): S1 carries its
    // OWN bounded ack watchdog (PB_WATCHDOG_LOG2 = 24 @ 50 MHz pb_clk
    // ~= 335 ms), sized for legitimate SD-card latency behind a
    // DRQ-checked SCSI beat (worst case > 100 ms).  The xbar's OUTER
    // timeout for S1 must therefore exceed that INNER guarantee or a
    // legitimately slow disk access would be SLVERR'd and S1 poisoned
    // mid-transfer (wave-1 integration finding, T1 x T5).  2^27 core
    // cycles = ~1.34 s @ 100 MHz / ~671 ms @ 200 MHz > 335 ms.  All
    // other slaves keep the tighter WD_LOG2 bound.
    parameter WD_LOG2_S1 = 27,
    // ── SLAVE-STALL WATCHDOG — DISABLED BY DEFAULT (owner, 2026-09-06) ──
    // Set to 1 to restore.  At 0 both fire terms are constant-false, so
    // synthesis strips the counters and the SLVERR-synthesis arms.
    // Same rationale as peripheral_bus.v's ENABLE_ACK_WATCHDOG (read the
    // long comment there): a bounded watchdog LAUNDERS a forward-progress
    // bug into a fatal bus error far from its cause.  This file's own
    // header records the layering hazard first-hand -- the FATAL
    // abandonment path pre-empted the GRACEFUL retryable one by 3.75x,
    // so recovery could never run.  Policy: fix what stops progress; do
    // not bound it.  A wedged slave now holds its port until reset, which
    // is DELIBERATE -- a hang that stops where it broke is debuggable.
    parameter ENABLE_WD = 0,
    // ── S1 RESET-DEASSERT TAIL (backlog item 3) ────────────────────────
    // How many core_clk cycles after `slv_flush` (or `rst`) deasserts S1
    // stays closed to new traffic.  S1 is the ONLY slave whose far side
    // leaves reset in a DIFFERENT clock domain, so it is the only one that
    // needs this -- see the s1_rst_tail_busy block for the full argument.
    //
    // Sizing.  Both domains release through xpm_cdc_async_rst with
    // DEST_SYNC_FF = 4 (rtl/board/clk_rst.v), each counted in its OWN
    // clock.  pb_clk is always exactly 50 MHz while core_clk is up to
    // 200 MHz (fpga_top_clocks.vh's one-VCO table), so the pb side's 4
    // destination clocks are up to 16 core_clk, on top of which the S1
    // CDC's own FIFO pointers have to resync on both sides.  2^8 = 256
    // core_clk (1.28 us at 200 MHz, 2.56 us at 100 MHz) is an order of
    // magnitude of margin over that and costs nothing: the only event that
    // arms it also holds the CPU in reset for the whole boot-ROM reload,
    // which is orders of magnitude longer again.  Overridable (the unit tb
    // builds with a small value) so the hold is observable in-sim.
    parameter S1_RST_TAIL_LOG2 = 8,
    // S3 normally terminates in resettable local VRAM, so slv_flush means
    // its backend forgot all accepted work.  With VRAM_IN_DDR, however, S3
    // is a route into the always-live DDR mux/MIG path.  In that topology
    // late B/R responses must be quarantined before S3 can be reused.
    parameter S3_BACKEND_SURVIVES_FLUSH = 0,
    // Grace window, as a log2 cycle count, for the MINOR-4 "slave
    // answered right at the deadline" write-watchdog suppression.  See
    // the B_HOLD_MAX localparam below for the full rationale; must not
    // exceed B_HOLD_CNT_W = 16.
    parameter B_HOLD_LOG2 = 10
) (
    input  wire                       clk,
    input  wire                       rst,
    input  wire                       cpu_overlay_active,
    input  wire                       cpu_overlay_reset,
    // Observability for the ROM-overlay state (2026-09-12).  The debug
    // `reset` wedge could not be decided on hardware because neither half
    // of `effective_cpu_overlay_active` was reachable from JTAG -- they
    // only drove LEDs.  Brought out so a VIO probe can answer "did the
    // overlay re-arm across this reset?" in one command.  Pure observation:
    // no decode or datapath behaviour depends on these.
    output wire                       cpu_overlay_disabled,
    output wire                       cpu_overlay_effective,
    // Reset-wedge instrumentation (2026-09-12). The sticky poison latches and the
    // per-slot busy state are the ONE part of the debug path not visible from JTAG,
    // and two speculative fixes have already been spent guessing at them. Observation
    // only; nothing downstream consumes these.
    output wire [5:0]                 dbg_slv_poisoned,
    output wire                       dbg_s1_slot_busy,
    // Runtime RAM window size selector (log2(bytes)).
    // 22=4 MiB .. 30=1 GiB, default latched value is 26 (64 MiB).
    // Addresses in the 1 GiB RAM decode outside this visible size return
    // local DECERR (not wrapped/aliased).
    input  wire [5:0]                 dbg_ram_window_lg2,
    // Selects the M0 physical port's fan-in source: 1 = boot FSM
    // (m0b_*), 0 = CPU LSU (m0_*).  Drive with the SoC's `cpu_rst` (the
    // CPU-held-in-reset signal) — see the M0/boot merge header note
    // above for why a plain mux (not an arbiter) is correct here.
    input  wire                       cpu_held_in_reset,
    // slv_flush — CRITICAL fix (task review round 1, poison-on-reset-
    // transient).  Several slave-side bridges live on a DIFFERENT reset
    // domain than this xbar instance itself: e.g. in fpga_top_xbar.vh,
    // the xbar is `.rst(core_rst_bank[4])` but the S2/S4/S5
    // `axi_wide_to_axilite` bridges are `.rst(soc_full_rst_bank[4])`,
    // and local-URAM S3 (VRAM) resets on `soc_full_rst_bank[0]`.  In the
    // VRAM_IN_DDR topology, S3 instead terminates in the live DDR fabric;
    // S3_BACKEND_SURVIVES_FLUSH enables the response quarantine required
    // for that exception.  A `soc_full_rst` while a
    // transaction is in flight (JTAG `reset hold` is the reliable
    // repro) makes the bridge forget the transaction out from under the
    // xbar; without this input the watchdog would fire post-release and
    // sticky-poison a perfectly healthy slave on every routine unified
    // reset with traffic in-flight.  Drive with the SAME signal the
    // S2/S4/S5 bridges reset on (`soc_full_rst_bank[4]`) — see
    // fpga_top_xbar.vh.
    //
    // DOMAIN MEMBERSHIP (this is NOT the same set as
    // is_lite_only_slv()): see `is_flush_domain_slv()` below for the
    // full rationale, and for why S1 joined the set on 2026-09-12.
    //   - Flush-domain (aborted/rejected by slv_flush): S1, S2, S3, S4, S5.
    //   - NOT flush-domain: S0 (DDR).  The MIG, and l2c above it, stay on
    //     core_rst to skip the ~100 ms DDR re-calibration, so that backend
    //     really does survive a soc_full_rst and really will answer.
    //     Aborting S0 here would need a late-response quarantine for the
    //     DDR path first (the S3_BACKEND_SURVIVES_FLUSH idiom); that is
    //     the remaining piece of "the SoC bus resets as one unit" and is
    //     NOT done.  See docs/bus_debug_split_plan.md.
    // Contract, while asserted:
    //   (i)   all per-slot watchdog counters freeze (no increment, no
    //         fire), for EVERY slave including S0 — see
    //         ws_wd_cnt/rs_wd_cnt update loops below.
    //   (ii)  every write/read slot with an outstanding transaction
    //         against a flush-domain slave (is_flush_domain_slv(), NOT
    //         is_lite_only_slv()) aborts through the SAME local
    //         drain/RLOCAL machinery the watchdog uses (drain remaining
    //         W beats, BRESP=SLVERR; remaining R beats SLVERR with
    //         RLAST) but does NOT set the sticky poison latch — this is
    //         a "the far side just forgot everything", not "the far
    //         side is actually wedged".  S0 transactions are left
    //         alone entirely (frozen watchdog only).
    //   (iii) new AW/AR decoding onto a flush-domain slave answers local
    //         SLVERR immediately (non-sticky) instead of being
    //         forwarded, since the bridge behind it is itself in reset.
    //         S0 decode is unaffected.
    //   (iv)  on the ASSERTION edge (rising edge) of slv_flush, every
    //         sticky poison latch (slv_w_poisoned/slv_r_poisoned, ALL
    //         six slaves) clears — a unified reset is a legitimate
    //         "fresh start", so a slave poisoned before the reset
    //         window should not still read as poisoned after it.
    // Net policy: "poisoned" means a genuine runtime wedge observed
    // with NO reset in progress; a reset-window drop is "flushed", not
    // "poisoned" — flushed transactions get exactly one clean SLVERR
    // and the slave is trusted again on the next real access.
    input  wire                       slv_flush,

    // ── s1_far_reset (race audit, 2026-09-18) ─────────────────────────
    // "S1's far side is being reset by an event this crossbar does not
    // otherwise see."  `slv_flush` is soc_full_rst_bank[4] and the S1 tail
    // below already covers everything that reaches it -- but the pb island
    // has a THIRD reset source that does not:
    //
    //   pb_full_rst = pb_rst || debug_full_reset_pb_sync || warm_peripheral_reset
    //
    // and `warm_peripheral_reset` -- the 68040 RESET instruction, i.e. an
    // ordinary guest boot -- is in NEITHER core_rst nor soc_full_rst.  It
    // resets every Mac peripheral for ~128 core_clk (plus 4 pb_clk of
    // release sync) while `peripheral_bus` itself, which is deliberately on
    // pb_soc_full_rst_bank[3] so the still-running CPU keeps a front door,
    // stays fully live.
    //
    // A transaction admitted in that window is handed to peripheral_bus and
    // then fired at a peripheral that is in reset.  For most slots that
    // self-heals -- the strobes are LEVELS held until ack and every
    // peripheral re-acks on release -- but FOUR are one-cycle PULSES
    // latched behind a "kicked" flag that only an ack can clear: the SCSI
    // DMA-shim read and write, and the ASC and ORWELL multi-byte write
    // serialisers.  The pulse lands on a reset chip, the flag latches, the
    // pulse can never re-fire, and rd_busy/wr_busy stick -- taking
    // s_arready/s_awready to 0, i.e. S1 is dead TO EVERY MASTER until
    // core_rst.  peripheral_bus's own ack-timeout watchdog does not save it:
    // ENABLE_ACK_WATCHDOG defaults to 0 and the production instantiation
    // never overrides it, so that logic is compiled out (the comment beside
    // it describing a live 2^24-cycle watchdog is stale).
    //
    // This is the same defect as backlog item 3, one layer further out, and
    // the same remedy answers it: hold S1 closed, do not reject.  Sized by
    // the same tail, which is an order of magnitude past both the pulse and
    // the pb-side release sync -- and past the extra 2 pb_clk the pb-side
    // copy of the DAFB SCSI control register needs to reload, which matters
    // because that register is zeroed by the SAME net that resets the SCSI
    // chip and its [7] bit is what would otherwise have withheld the pulse.
    //
    // ⚠ This covers transactions admitted DURING and just after the window.
    // A transaction already in flight when the far reset ASSERTS is a
    // separate hole that must be fixed inside peripheral_bus by re-arming
    // its one-shot latches; it is NOT closed here.  Negative control for
    // what IS closed: scenario 54 in tb_axi_xbar.cpp.
    input  wire                       s1_far_reset,

    // ── m0b_master_reset (race audit, 2026-09-18) ─────────────────────
    // "The boot-FSM side of the shared slot-0 port has been reset."
    //
    // m0_abandon's whole job is "the owner of this in-flight slot-0 write
    // has gone away", and its predicate is a MISMATCH test
    // (m0_wsel_q != cpu_held_in_reset) that is structurally blind whenever
    // BOTH sides are held.  d85deb57 covered one such case by ORing in
    // slv_flush.  There is a THIRD arming event it still misses:
    //
    //   boot_fsm_rst = soc_full_rst || jtag_boot_bypass || dbg_cold_reset_hold
    //
    // `dbg_cold_reset_hold` is DBG_CONTROL[4], a sticky level the JTAG host
    // raises at an arbitrary cycle.  It resets boot_fsm -- which forgets an
    // in-flight ROM-copy burst -- while cpu_held_in_reset STAYS 1 (the hold
    // is itself a cpu_rst term) and soc_full_rst never asserts, so neither
    // the mismatch nor slv_flush fires.  The slot sits in WS_FWD_W with
    // S0/DDR parked mid-burst, or in WS_WAIT_B holding a B nobody will take;
    // sw_owned[0] stays set and DDR is dead to every master.  `rst` here is
    // core_rst_bank[4], which a debug reset does not assert, so only a
    // reconfigure clears it.
    //
    // Nothing new is needed downstream of the predicate: the WS_FWD_W
    // m0_abandon branch already completes the slave's burst with wstrb=0
    // filler and a correct WLAST (WS_PAD_W), and the WS_WAIT_B branch
    // already swallows the orphaned B.  The machinery was right; it was
    // simply never told about this event.
    //
    // ORed in unconditionally, like slv_flush and for the same reason: if
    // the CPU side owns the slot instead, cpu_held_in_reset rises with the
    // hold and the mismatch term reaches the identical conclusion a cycle
    // later, so an early uniform abandon is strictly the safer of the two.
    //
    // ⚠ PAIR THIS WITH RESETTING THE BOOT-SIDE ADAPTER.  This frees the
    // CROSSBAR.  boot_fsm's axi_narrow_to_wide (u_boot_n2w) must reset on
    // the same event as boot_fsm itself, or its aw_valid_q stays latched on
    // a burst that will never get its wide WLAST and n_awready is 0 forever
    // -- the restarted boot_fsm's first AW is refused and rom_loaded never
    // rises.  Fixing only one half moves the wedge; see fpga_top_boot_master.vh.
    //
    // Negative control: scenario 55 in tb_axi_xbar.cpp.
    input  wire                       m0b_master_reset,

    // Master 0 — CPU LSU (full AXI4)
    input  wire [ID_WIDTH-1:0]        m0_awid,
    input  wire [31:0]                m0_awaddr,
    input  wire [7:0]                 m0_awlen,
    input  wire [2:0]                 m0_awsize,
    input  wire [1:0]                 m0_awburst,
    input  wire                       m0_awvalid,
    output wire                       m0_awready,
    input  wire [DATA_WIDTH-1:0]      m0_wdata,
    input  wire [STRB_WIDTH-1:0]      m0_wstrb,
    input  wire                       m0_wlast,
    input  wire                       m0_wvalid,
    output wire                       m0_wready,
    output wire [ID_WIDTH-1:0]        m0_bid,
    output wire [1:0]                 m0_bresp,
    output wire                       m0_bvalid,
    input  wire                       m0_bready,
    input  wire [ID_WIDTH-1:0]        m0_arid,
    input  wire [31:0]                m0_araddr,
    input  wire [7:0]                 m0_arlen,
    input  wire [2:0]                 m0_arsize,
    input  wire [1:0]                 m0_arburst,
    input  wire                       m0_arvalid,
    output wire                       m0_arready,
    output wire [ID_WIDTH-1:0]        m0_rid,
    output wire [DATA_WIDTH-1:0]      m0_rdata,
    output wire [1:0]                 m0_rresp,
    output wire                       m0_rlast,
    output wire                       m0_rvalid,
    input  wire                       m0_rready,

    // Master 0b — boot FSM (write-only).  Time-multiplexed onto the M0
    // physical port by `cpu_held_in_reset` — see the M0/boot merge header
    // note above.  No AR/R channel: boot FSM never reads.
    input  wire [ID_WIDTH-1:0]        m0b_awid,
    input  wire [31:0]                m0b_awaddr,
    input  wire [7:0]                 m0b_awlen,
    input  wire [2:0]                 m0b_awsize,
    input  wire [1:0]                 m0b_awburst,
    input  wire                       m0b_awvalid,
    output wire                       m0b_awready,
    input  wire [DATA_WIDTH-1:0]      m0b_wdata,
    input  wire [STRB_WIDTH-1:0]      m0b_wstrb,
    input  wire                       m0b_wlast,
    input  wire                       m0b_wvalid,
    output wire                       m0b_wready,
    output wire [ID_WIDTH-1:0]        m0b_bid,
    output wire [1:0]                 m0b_bresp,
    output wire                       m0b_bvalid,
    input  wire                       m0b_bready,

    // Master 1 — host debug (XDMA / JTAG-to-AXI)
    input  wire [ID_WIDTH-1:0]        m1_awid,
    input  wire [31:0]                m1_awaddr,
    input  wire [7:0]                 m1_awlen,
    input  wire [2:0]                 m1_awsize,
    input  wire [1:0]                 m1_awburst,
    input  wire                       m1_awvalid,
    output wire                       m1_awready,
    input  wire [DATA_WIDTH-1:0]      m1_wdata,
    input  wire [STRB_WIDTH-1:0]      m1_wstrb,
    input  wire                       m1_wlast,
    input  wire                       m1_wvalid,
    output wire                       m1_wready,
    output wire [ID_WIDTH-1:0]        m1_bid,
    output wire [1:0]                 m1_bresp,
    output wire                       m1_bvalid,
    input  wire                       m1_bready,
    input  wire [ID_WIDTH-1:0]        m1_arid,
    input  wire [31:0]                m1_araddr,
    input  wire [7:0]                 m1_arlen,
    input  wire [2:0]                 m1_arsize,
    input  wire [1:0]                 m1_arburst,
    input  wire                       m1_arvalid,
    output wire                       m1_arready,
    output wire [ID_WIDTH-1:0]        m1_rid,
    output wire [DATA_WIDTH-1:0]      m1_rdata,
    output wire [1:0]                 m1_rresp,
    output wire                       m1_rlast,
    output wire                       m1_rvalid,
    input  wire                       m1_rready,

    // Master 2 — CPU instruction fetch (read-only).  Independent from CPU
    // LSU (M0) by maintainer mandate — never merge these two.
    input  wire [ID_WIDTH-1:0]        m2_arid,
    input  wire [31:0]                m2_araddr,
    input  wire [7:0]                 m2_arlen,
    input  wire [2:0]                 m2_arsize,
    input  wire [1:0]                 m2_arburst,
    input  wire                       m2_arvalid,
    output wire                       m2_arready,
    output wire [ID_WIDTH-1:0]        m2_rid,
    output wire [DATA_WIDTH-1:0]      m2_rdata,
    output wire [1:0]                 m2_rresp,
    output wire                       m2_rlast,
    output wire                       m2_rvalid,
    input  wire                       m2_rready,

    // Master 3 — the shared DMA engine (fpga_top_dma.vh).  Occupies the
    // write/read fan-in slot (index 3) that dma_ctrl's stubbed-out M4
    // vacated in the 2026-07-16 master-count reduction; the arbiter was
    // never resized, so this is a port re-attach, not an arbiter change.
    input  wire [ID_WIDTH-1:0]        m3_awid,
    input  wire [31:0]                m3_awaddr,
    input  wire [7:0]                 m3_awlen,
    input  wire [2:0]                 m3_awsize,
    input  wire [1:0]                 m3_awburst,
    input  wire                       m3_awvalid,
    output wire                       m3_awready,
    input  wire [DATA_WIDTH-1:0]      m3_wdata,
    input  wire [STRB_WIDTH-1:0]      m3_wstrb,
    input  wire                       m3_wlast,
    input  wire                       m3_wvalid,
    output wire                       m3_wready,
    output wire [ID_WIDTH-1:0]        m3_bid,
    output wire [1:0]                 m3_bresp,
    output wire                       m3_bvalid,
    input  wire                       m3_bready,
    input  wire [ID_WIDTH-1:0]        m3_arid,
    input  wire [31:0]                m3_araddr,
    input  wire [7:0]                 m3_arlen,
    input  wire [2:0]                 m3_arsize,
    input  wire [1:0]                 m3_arburst,
    input  wire                       m3_arvalid,
    output wire                       m3_arready,
    output wire [ID_WIDTH-1:0]        m3_rid,
    output wire [DATA_WIDTH-1:0]      m3_rdata,
    output wire [1:0]                 m3_rresp,
    output wire                       m3_rlast,
    output wire                       m3_rvalid,
    input  wire                       m3_rready,

    // M4 (DMA controller) intentionally has NO top-level port anymore —
    // stubbed out 2026-07-16 (see header note above).  Its fan-in slot
    // (index 3) is now occupied by M3 above.

    // Slave 0 — DDR
    output wire [XID_WIDTH-1:0]       s0_awid,
    output wire [31:0]                s0_awaddr,
    output wire [7:0]                 s0_awlen,
    output wire [2:0]                 s0_awsize,
    output wire [1:0]                 s0_awburst,
    output wire                       s0_awvalid,
    input  wire                       s0_awready,
    output wire [DATA_WIDTH-1:0]      s0_wdata,
    output wire [STRB_WIDTH-1:0]      s0_wstrb,
    output wire                       s0_wlast,
    output wire                       s0_wvalid,
    input  wire                       s0_wready,
    input  wire [XID_WIDTH-1:0]       s0_bid,
    input  wire [1:0]                 s0_bresp,
    input  wire                       s0_bvalid,
    output wire                       s0_bready,
    output wire [XID_WIDTH-1:0]       s0_arid,
    output wire [31:0]                s0_araddr,
    output wire [7:0]                 s0_arlen,
    output wire [2:0]                 s0_arsize,
    output wire [1:0]                 s0_arburst,
    output wire                       s0_arvalid,
    input  wire                       s0_arready,
    input  wire [XID_WIDTH-1:0]       s0_rid,
    input  wire [DATA_WIDTH-1:0]      s0_rdata,
    input  wire [1:0]                 s0_rresp,
    input  wire                       s0_rlast,
    input  wire                       s0_rvalid,
    output wire                       s0_rready,

    // Slave 1 — peripheral bus
    output wire [XID_WIDTH-1:0]       s1_awid,
    output wire [31:0]                s1_awaddr,
    output wire [7:0]                 s1_awlen,
    output wire [2:0]                 s1_awsize,
    output wire [1:0]                 s1_awburst,
    output wire                       s1_awvalid,
    input  wire                       s1_awready,
    output wire [DATA_WIDTH-1:0]      s1_wdata,
    output wire [STRB_WIDTH-1:0]      s1_wstrb,
    output wire                       s1_wlast,
    output wire                       s1_wvalid,
    input  wire                       s1_wready,
    input  wire [XID_WIDTH-1:0]       s1_bid,
    input  wire [1:0]                 s1_bresp,
    input  wire                       s1_bvalid,
    output wire                       s1_bready,
    output wire [XID_WIDTH-1:0]       s1_arid,
    output wire [31:0]                s1_araddr,
    output wire [7:0]                 s1_arlen,
    output wire [2:0]                 s1_arsize,
    output wire [1:0]                 s1_arburst,
    output wire                       s1_arvalid,
    input  wire                       s1_arready,
    input  wire [XID_WIDTH-1:0]       s1_rid,
    input  wire [DATA_WIDTH-1:0]      s1_rdata,
    input  wire [1:0]                 s1_rresp,
    input  wire                       s1_rlast,
    input  wire                       s1_rvalid,
    output wire                       s1_rready,

    // Slave 2 — DMA config
    output wire [XID_WIDTH-1:0]       s2_awid,
    output wire [31:0]                s2_awaddr,
    output wire [7:0]                 s2_awlen,
    output wire [2:0]                 s2_awsize,
    output wire [1:0]                 s2_awburst,
    output wire                       s2_awvalid,
    input  wire                       s2_awready,
    output wire [DATA_WIDTH-1:0]      s2_wdata,
    output wire [STRB_WIDTH-1:0]      s2_wstrb,
    output wire                       s2_wlast,
    output wire                       s2_wvalid,
    input  wire                       s2_wready,
    input  wire [XID_WIDTH-1:0]       s2_bid,
    input  wire [1:0]                 s2_bresp,
    input  wire                       s2_bvalid,
    output wire                       s2_bready,
    output wire [XID_WIDTH-1:0]       s2_arid,
    output wire [31:0]                s2_araddr,
    output wire [7:0]                 s2_arlen,
    output wire [2:0]                 s2_arsize,
    output wire [1:0]                 s2_arburst,
    output wire                       s2_arvalid,
    input  wire                       s2_arready,
    input  wire [XID_WIDTH-1:0]       s2_rid,
    input  wire [DATA_WIDTH-1:0]      s2_rdata,
    input  wire [1:0]                 s2_rresp,
    input  wire                       s2_rlast,
    input  wire                       s2_rvalid,
    output wire                       s2_rready,

    // Slave 3 — VRAM pixel aperture (URAM-backed)
    // The xbar peels off VRAM_BASE before forwarding: the vram slave
    // sees s_awaddr / s_araddr as a zero-based byte offset into its
    // framebuffer.  See rtl/sys/vram.v header §"Port-A muxing".  Added
    // by task #147 (xbar-vram-wire) — P0 blocker for mac-logo on HDMI.
    output wire [XID_WIDTH-1:0]       s3_awid,
    output wire [31:0]                s3_awaddr,
    output wire [7:0]                 s3_awlen,
    output wire [2:0]                 s3_awsize,
    output wire [1:0]                 s3_awburst,
    output wire                       s3_awvalid,
    input  wire                       s3_awready,
    output wire [DATA_WIDTH-1:0]      s3_wdata,
    output wire [STRB_WIDTH-1:0]      s3_wstrb,
    output wire                       s3_wlast,
    output wire                       s3_wvalid,
    input  wire                       s3_wready,
    input  wire [XID_WIDTH-1:0]       s3_bid,
    input  wire [1:0]                 s3_bresp,
    input  wire                       s3_bvalid,
    output wire                       s3_bready,
    output wire [XID_WIDTH-1:0]       s3_arid,
    output wire [31:0]                s3_araddr,
    output wire [7:0]                 s3_arlen,
    output wire [2:0]                 s3_arsize,
    output wire [1:0]                 s3_arburst,
    output wire                       s3_arvalid,
    input  wire                       s3_arready,
    input  wire [XID_WIDTH-1:0]       s3_rid,
    input  wire [DATA_WIDTH-1:0]      s3_rdata,
    input  wire [1:0]                 s3_rresp,
    input  wire                       s3_rlast,
    input  wire                       s3_rvalid,
    output wire                       s3_rready,

    // Slave 4 — DAFB register shim
    output wire [XID_WIDTH-1:0]       s4_awid,
    output wire [31:0]                s4_awaddr,
    output wire [7:0]                 s4_awlen,
    output wire [2:0]                 s4_awsize,
    output wire [1:0]                 s4_awburst,
    output wire                       s4_awvalid,
    input  wire                       s4_awready,
    output wire [DATA_WIDTH-1:0]      s4_wdata,
    output wire [STRB_WIDTH-1:0]      s4_wstrb,
    output wire                       s4_wlast,
    output wire                       s4_wvalid,
    input  wire                       s4_wready,
    input  wire [XID_WIDTH-1:0]       s4_bid,
    input  wire [1:0]                 s4_bresp,
    input  wire                       s4_bvalid,
    output wire                       s4_bready,
    output wire [XID_WIDTH-1:0]       s4_arid,
    output wire [31:0]                s4_araddr,
    output wire [7:0]                 s4_arlen,
    output wire [2:0]                 s4_arsize,
    output wire [1:0]                 s4_arburst,
    output wire                       s4_arvalid,
    input  wire                       s4_arready,
    input  wire [XID_WIDTH-1:0]       s4_rid,
    input  wire [DATA_WIDTH-1:0]      s4_rdata,
    input  wire [1:0]                 s4_rresp,
    input  wire                       s4_rlast,
    input  wire                       s4_rvalid,
    output wire                       s4_rready,

    // Slave 5 — SD JTAG writer
    output wire [XID_WIDTH-1:0]       s5_awid,
    output wire [31:0]                s5_awaddr,
    output wire [7:0]                 s5_awlen,
    output wire [2:0]                 s5_awsize,
    output wire [1:0]                 s5_awburst,
    output wire                       s5_awvalid,
    input  wire                       s5_awready,
    output wire [DATA_WIDTH-1:0]      s5_wdata,
    output wire [STRB_WIDTH-1:0]      s5_wstrb,
    output wire                       s5_wlast,
    output wire                       s5_wvalid,
    input  wire                       s5_wready,
    input  wire [XID_WIDTH-1:0]       s5_bid,
    input  wire [1:0]                 s5_bresp,
    input  wire                       s5_bvalid,
    output wire                       s5_bready,
    output wire [XID_WIDTH-1:0]       s5_arid,
    output wire [31:0]                s5_araddr,
    output wire [7:0]                 s5_arlen,
    output wire [2:0]                 s5_arsize,
    output wire [1:0]                 s5_arburst,
    output wire                       s5_arvalid,
    input  wire                       s5_arready,
    input  wire [XID_WIDTH-1:0]       s5_rid,
    input  wire [DATA_WIDTH-1:0]      s5_rdata,
    input  wire [1:0]                 s5_rresp,
    input  wire                       s5_rlast,
    input  wire                       s5_rvalid,
    output wire                       s5_rready
);

    localparam NW = 4;
    localparam NR = 4;
    localparam NS = 6;
    localparam SLOT_W = 2;
    // Slave-select field width — must cover all valid slave IDs and the
    // NONE sentinel.  Widened from 2 to 3 bits when VRAM (S3) landed.
    localparam SSEL_W = 3;
    localparam [5:0] RAM_WINDOW_LG2_MIN = 6'd22; // 4 MiB
    localparam [5:0] RAM_WINDOW_LG2_MAX = 6'd30; // 1 GiB
    localparam [5:0] RAM_WINDOW_LG2_DFLT = 6'd22; // 4 MiB (Q700-default)

    // Write-side bundling.
    wire [ID_WIDTH-1:0]    mw_awid    [0:NW-1];
    wire [31:0]            mw_awaddr  [0:NW-1];
    wire [7:0]             mw_awlen   [0:NW-1];
    wire [2:0]             mw_awsize  [0:NW-1];
    wire [1:0]             mw_awburst [0:NW-1];
    wire                   mw_awvalid [0:NW-1];
    wire                   mw_awready [0:NW-1];
    wire [DATA_WIDTH-1:0]  mw_wdata   [0:NW-1];
    wire [STRB_WIDTH-1:0]  mw_wstrb   [0:NW-1];
    wire                   mw_wlast   [0:NW-1];
    wire                   mw_wvalid  [0:NW-1];
    wire                   mw_wready  [0:NW-1];
    wire [ID_WIDTH-1:0]    mw_bid     [0:NW-1];
    wire [1:0]             mw_bresp   [0:NW-1];
    wire                   mw_bvalid  [0:NW-1];
    wire                   mw_bready  [0:NW-1];
    wire [2:0]             mw_midx    [0:NW-1];

    // Slot 0 — CPU LSU / boot FSM merge.  2:1 mux, selected by
    // `m0_wsel_q`; see the M0/boot merge header note for why a mux (not an
    // arbiter) is correct here.  `mw_midx[0]` tracks the same select so
    // the ROM-write policy (`is_rom_loader_w` below) applies to whichever
    // side is actually driving the slot.
    //
    // WHY THE SELECT IS A REGISTER AND NOT `cpu_held_in_reset` DIRECTLY
    // ────────────────────────────────────────────────────────────────
    // It used to be `cpu_held_in_reset` combinationally, which means it
    // re-selected MID-TRANSACTION.  A slot-0 write already streaming in
    // WS_FWD_W keeps driving its owned slave's W channel (`ws?_drive[0]`
    // does not care about the mux), so after the select flips the
    // remaining beats are sourced from the OTHER side of the mux and the
    // B is delivered to the other side as well.  Concretely: boot_fsm's
    // W beat is consumed as the CPU burst's next beat — boot's data lands
    // at the CPU's address in DDR — and `m0b_bvalid` then reports a
    // completion, carrying the CPU's AWID, for a write whose AW the
    // crossbar never accepted from boot.  Silent cross-master corruption,
    // the same failure class as task #162 (`axi_bridge_w_pad.v`), one hop
    // further in.  Symmetric on the falling edge.
    //
    // This is reachable on hardware: `cpu_held_in_reset` is driven by
    // `cpu_rst` (fpga_top_xbar.vh), which ORs in `dbg_cold_reset_hold` —
    // DBG_CONTROL bit 4, the JTAG `reset hold` / `reset-and-halt-after`
    // path (fpga_top_clocks.vh) — and that bit raises `cpu_rst` at an
    // arbitrary cycle with THIS module's `rst` (core_rst_bank[4]) LOW and
    // `slv_flush` deasserted.  With single-beat writes the exposure window
    // was the slave's WREADY latency, a cycle or two; once the CPU's
    // D-cache burst refill is live the window is the whole multi-beat
    // burst, on every writeback.
    //
    // `m0_wsel_q` therefore TRACKS `cpu_held_in_reset` while slot 0 is
    // idle and FREEZES for the life of a slot-0 write.  It is a plain
    // register output feeding the mux selects, so it is strictly SHORTER
    // than the path it replaces (`cpu_held_in_reset` arrives from an OR
    // tree plus an 8-deep reset stretcher in another file, through a
    // top-level port) — no combinational depth is added anywhere on an
    // AXI ready/valid path.
    //
    // Reads are untouched: boot FSM is write-only, so the M0 read channels
    // have never been muxed at all.
    reg m0_wsel_q;
    reg m0_wsel_busy;   // a slot-0 write is in flight (updated near ws_state's
                        // declaration below, where ws_state is in scope)

    // GAP 1 companions of the select above (see the "Abandoned-burst W
    // padding" block near ws_state's declaration for the full argument).
    //
    // m0_abandon — sticky "the side that owns the in-flight slot-0 write is
    //   gone".  Latched sticky because `cpu_held_in_reset` may flip BACK
    //   before the abandoned burst has been padded out, and the decision
    //   must not un-make itself half way: a reset assert/deassert pair means
    //   the owner restarted, so it is not waiting for that B either way.
    //   Drives mw_bready[0] high and masks m0_bvalid/m0b_bvalid, so the
    //   slave's B is swallowed by the crossbar instead of being offered to a
    //   master that cannot take it (which would otherwise wedge WS_WAIT_B
    //   forever — ws_wd_fire is suppressed while mw_bvalid_slv is high).
    //
    // m0_pad_w — "slot 0 is emitting filler beats".  A registered companion
    //   of (ws_state[0] == WS_PAD_W), maintained in the same clocked
    //   assignments so the two are always coincident.  The duplication is
    //   deliberate and purely for timing: the slot-0 fan-in mux overrides
    //   below must be selected by a plain register, because a 3-bit state
    //   compare there would push mw_wlast[0] from one LUT level to two.
    reg m0_abandon;
    reg m0_pad_w;

    assign mw_midx[0]    = m0_wsel_q ? `XBAR_M_BOOT : `XBAR_M_CPU;
    assign mw_midx[1]    = `XBAR_M_XDMA;
    // Slots 2/3 are permanently idle — see the master-count-reduction
    // header note (boot FSM merged into slot 0 above; DMA/M4 stubbed).
    // Left as `XBAR_M_BOOT`/`XBAR_M_DMA` purely for readability; the
    // tags are dead since mw_awvalid[2]/[3] never assert.
    assign mw_midx[2]    = `XBAR_M_BOOT;
    assign mw_midx[3]    = `XBAR_M_DMA;

    assign mw_awid   [0] = m0_wsel_q ? m0b_awid    : m0_awid;
    assign mw_awaddr [0] = m0_wsel_q ? m0b_awaddr  : m0_awaddr;
    assign mw_awlen  [0] = m0_wsel_q ? m0b_awlen   : m0_awlen;
    assign mw_awsize [0] = m0_wsel_q ? m0b_awsize  : m0_awsize;
    assign mw_awburst[0] = m0_wsel_q ? m0b_awburst : m0_awburst;
    assign mw_awvalid[0] = m0_wsel_q ? m0b_awvalid : m0_awvalid;
    // Route the grant back to whichever side is actually selected; the
    // deselected side is guaranteed idle by construction (see header
    // note) so tying its awready to 0 is defensive, not load-bearing.
    assign m0_awready    = !m0_wsel_q && mw_awready[0];
    assign m0b_awready   =  m0_wsel_q && mw_awready[0];
    // GAP 1: while slot 0 is padding out an abandoned burst (m0_pad_w), the
    // crossbar drives the W channel itself — WVALID forced high, WSTRB all
    // zero so the filler beats commit nothing, and WLAST taken from the
    // registered beat counter so the slave's burst terminates on exactly the
    // beat AWLEN promised.  WDATA is left on the mux output: with WSTRB=0 it
    // is architecturally irrelevant, and overriding it would add a 128-bit
    // mux for no benefit.
    //
    // Every one of these overrides is selected by a REGISTER (m0_pad_w /
    // ws_wlast_pend[0]) with register-sourced data, so each stays a single
    // LUT level exactly as before: mw_wvalid[0] goes from a 3-input mux to
    // a 4-input one, mw_wlast[0] from 3 to 5, mw_wstrb[0] from 3 to 4 — all
    // still one LUT6. No AXI ready/valid path gets deeper.
    assign mw_wdata  [0] = m0_wsel_q ? m0b_wdata  : m0_wdata;
    assign mw_wstrb  [0] = m0_pad_w ? {STRB_WIDTH{1'b0}}
                                    : (m0_wsel_q ? m0b_wstrb  : m0_wstrb);
    assign mw_wlast  [0] = m0_pad_w ? ws_wlast_pend[0]
                                    : (m0_wsel_q ? m0b_wlast  : m0_wlast);
    assign mw_wvalid [0] = m0_pad_w | (m0_wsel_q ? m0b_wvalid : m0_wvalid);
    // Neither side may see WREADY while the crossbar owns the W channel —
    // otherwise a master that happens to be live (the OTHER side of the
    // mux) would book a beat as accepted while a filler beat went to the
    // slave in its place.
    assign m0_wready      = !m0_wsel_q && !m0_pad_w && mw_wready[0];
    assign m0b_wready     =  m0_wsel_q && !m0_pad_w && mw_wready[0];
    // B response fans out to both top-level ports but is only ever
    // "valid" on the side that issued the matching AW — the deselected
    // side's bvalid is forced low so it never sees a spurious response.
    // Because the select is frozen for the life of the write, "the side
    // that issued the matching AW" is now literally true rather than
    // true-only-if-nothing-changed-meanwhile.
    //
    // GAP 1: once the owning side is gone (m0_abandon) the response has
    // nowhere legitimate to go, so the crossbar consumes it itself —
    // BREADY forced high, both masters' BVALID masked.  Without this the
    // slot parks in WS_WAIT_B offering a B nobody will take, and because
    // ws_wd_fire is deliberately suppressed while mw_bvalid_slv is high
    // (MINOR 4, AXI4 BRESP stability), not even the watchdog frees it:
    // sw_owned stays set and the slave is dead to every master.
    assign m0_bid         = mw_bid  [0];
    assign m0_bresp       = mw_bresp[0];
    assign m0_bvalid      = !m0_wsel_q && !m0_abandon && mw_bvalid[0];
    assign m0b_bid         = mw_bid  [0];
    assign m0b_bresp       = mw_bresp[0];
    assign m0b_bvalid      =  m0_wsel_q && !m0_abandon && mw_bvalid[0];
    assign mw_bready [0]  = m0_abandon | (m0_wsel_q ? m0b_bready : m0_bready);

    assign mw_awid   [1] = m1_awid;
    assign mw_awaddr [1] = m1_awaddr;
    assign mw_awlen  [1] = m1_awlen;
    assign mw_awsize [1] = m1_awsize;
    assign mw_awburst[1] = m1_awburst;
    assign mw_awvalid[1] = m1_awvalid;
    assign m1_awready    = mw_awready[1];
    assign mw_wdata  [1] = m1_wdata;
    assign mw_wstrb  [1] = m1_wstrb;
    assign mw_wlast  [1] = m1_wlast;
    assign mw_wvalid [1] = m1_wvalid;
    assign m1_wready     = mw_wready[1];
    assign m1_bid        = mw_bid   [1];
    assign m1_bresp      = mw_bresp [1];
    assign m1_bvalid     = mw_bvalid[1];
    assign mw_bready [1] = m1_bready;

    // Slot 2 — permanently idle (formerly the boot FSM's dedicated
    // port; boot traffic now enters through slot 0's mux above).  No
    // top-level port drives this slot anymore.
    assign mw_awid   [2] = {ID_WIDTH{1'b0}};
    assign mw_awaddr [2] = 32'h0;
    assign mw_awlen  [2] = 8'h0;
    assign mw_awsize [2] = 3'h0;
    assign mw_awburst[2] = 2'h0;
    assign mw_awvalid[2] = 1'b0;
    assign mw_wdata  [2] = {DATA_WIDTH{1'b0}};
    assign mw_wstrb  [2] = {STRB_WIDTH{1'b0}};
    assign mw_wlast  [2] = 1'b0;
    assign mw_wvalid [2] = 1'b0;
    assign mw_bready [2] = 1'b1;

    // Slot 3 — the shared DMA engine.  Reoccupies the slot the stubbed-out
    // DMA M4 vacated; the arbiter already covered index 3.
    assign mw_awid   [3] = m3_awid;
    assign mw_awaddr [3] = m3_awaddr;
    assign mw_awlen  [3] = m3_awlen;
    assign mw_awsize [3] = m3_awsize;
    assign mw_awburst[3] = m3_awburst;
    assign mw_awvalid[3] = m3_awvalid;
    assign m3_awready    = mw_awready[3];
    assign mw_wdata  [3] = m3_wdata;
    assign mw_wstrb  [3] = m3_wstrb;
    assign mw_wlast  [3] = m3_wlast;
    assign mw_wvalid [3] = m3_wvalid;
    assign m3_wready     = mw_wready[3];
    assign m3_bid        = mw_bid  [3];
    assign m3_bresp      = mw_bresp[3];
    assign m3_bvalid     = mw_bvalid[3];
    assign mw_bready [3] = m3_bready;

    // Read-side bundling.
    wire [ID_WIDTH-1:0]    mr_arid    [0:NR-1];
    wire [31:0]            mr_araddr  [0:NR-1];
    wire [7:0]             mr_arlen   [0:NR-1];
    wire [2:0]             mr_arsize  [0:NR-1];
    wire [1:0]             mr_arburst [0:NR-1];
    wire                   mr_arvalid [0:NR-1];
    wire                   mr_arready [0:NR-1];
    wire [ID_WIDTH-1:0]    mr_rid     [0:NR-1];
    wire [DATA_WIDTH-1:0]  mr_rdata   [0:NR-1];
    wire [1:0]             mr_rresp   [0:NR-1];
    wire                   mr_rlast   [0:NR-1];
    wire                   mr_rvalid  [0:NR-1];
    wire                   mr_rready  [0:NR-1];
    wire [2:0]             mr_midx    [0:NR-1];

    assign mr_midx[0]    = `XBAR_M_CPU;
    assign mr_midx[1]    = `XBAR_M_XDMA;
    assign mr_midx[2]    = `XBAR_M_CPUI;
    // Slot 3 is permanently idle (formerly DMA/M4 read, stubbed — see
    // the master-count-reduction header note).  Tag left in place purely
    // for readability; dead since mr_arvalid[3] never asserts.
    assign mr_midx[3]    = `XBAR_M_DMA;

    assign mr_arid   [0] = m0_arid;
    assign mr_araddr [0] = m0_araddr;
    assign mr_arlen  [0] = m0_arlen;
    assign mr_arsize [0] = m0_arsize;
    assign mr_arburst[0] = m0_arburst;
    assign mr_arvalid[0] = m0_arvalid;
    assign m0_arready    = mr_arready[0];
    assign m0_rid        = mr_rid   [0];
    assign m0_rdata      = mr_rdata [0];
    assign m0_rresp      = mr_rresp [0];
    assign m0_rlast      = mr_rlast [0];
    assign m0_rvalid     = mr_rvalid[0];
    assign mr_rready [0] = m0_rready;

    assign mr_arid   [1] = m1_arid;
    assign mr_araddr [1] = m1_araddr;
    assign mr_arlen  [1] = m1_arlen;
    assign mr_arsize [1] = m1_arsize;
    assign mr_arburst[1] = m1_arburst;
    assign mr_arvalid[1] = m1_arvalid;
    assign m1_arready    = mr_arready[1];
    assign m1_rid        = mr_rid   [1];
    assign m1_rdata      = mr_rdata [1];
    assign m1_rresp      = mr_rresp [1];
    assign m1_rlast      = mr_rlast [1];
    assign m1_rvalid     = mr_rvalid[1];
    assign mr_rready [1] = m1_rready;

    assign mr_arid   [2] = m2_arid;
    assign mr_araddr [2] = m2_araddr;
    assign mr_arlen  [2] = m2_arlen;
    assign mr_arsize [2] = m2_arsize;
    assign mr_arburst[2] = m2_arburst;
    assign mr_arvalid[2] = m2_arvalid;
    assign m2_arready    = mr_arready[2];
    assign m2_rid         = mr_rid   [2];
    assign m2_rdata       = mr_rdata [2];
    assign m2_rresp       = mr_rresp [2];
    assign m2_rlast       = mr_rlast [2];
    assign m2_rvalid      = mr_rvalid[2];
    assign mr_rready [2] = m2_rready;

    // Slot 3 — the shared DMA engine, read side.
    assign mr_arid   [3] = m3_arid;
    assign mr_araddr [3] = m3_araddr;
    assign mr_arlen  [3] = m3_arlen;
    assign mr_arsize [3] = m3_arsize;
    assign mr_arburst[3] = m3_arburst;
    assign mr_arvalid[3] = m3_arvalid;
    assign m3_arready    = mr_arready[3];
    assign m3_rid        = mr_rid   [3];
    assign m3_rdata      = mr_rdata [3];
    assign m3_rresp      = mr_rresp [3];
    assign m3_rlast      = mr_rlast [3];
    assign m3_rvalid     = mr_rvalid[3];
    assign mr_rready [3] = m3_rready;

    // Latch + clamp the host-programmed RAM window selector so RAM
    // alias/visibility checks stay timing-clean and deterministic.
    reg [5:0] ram_window_lg2_q;
    always @(posedge clk) begin
        if (rst) begin
            ram_window_lg2_q <= RAM_WINDOW_LG2_DFLT;
        end else if (dbg_ram_window_lg2 < RAM_WINDOW_LG2_MIN) begin
            ram_window_lg2_q <= RAM_WINDOW_LG2_MIN;
        end else if (dbg_ram_window_lg2 > RAM_WINDOW_LG2_MAX) begin
            ram_window_lg2_q <= RAM_WINDOW_LG2_MAX;
        end else begin
            ram_window_lg2_q <= dbg_ram_window_lg2;
        end
    end

    function [31:0] ram_window_mask_from_lg2;
        input [5:0] lg2;
        begin
            case (lg2)
                6'd22: ram_window_mask_from_lg2 = 32'h003f_ffff; // 4 MiB - 1
                6'd23: ram_window_mask_from_lg2 = 32'h007f_ffff; // 8 MiB - 1
                6'd24: ram_window_mask_from_lg2 = 32'h00ff_ffff; // 16 MiB - 1
                6'd25: ram_window_mask_from_lg2 = 32'h01ff_ffff; // 32 MiB - 1
                6'd26: ram_window_mask_from_lg2 = 32'h03ff_ffff; // 64 MiB - 1
                6'd27: ram_window_mask_from_lg2 = 32'h07ff_ffff; // 128 MiB - 1
                6'd28: ram_window_mask_from_lg2 = 32'h0fff_ffff; // 256 MiB - 1
                6'd29: ram_window_mask_from_lg2 = 32'h1fff_ffff; // 512 MiB - 1
                6'd30: ram_window_mask_from_lg2 = 32'h3fff_ffff; // 1 GiB - 1
                default: ram_window_mask_from_lg2 = 32'h03ff_ffff;
            endcase
        end
    endfunction

    wire [31:0] ram_window_mask = ram_window_mask_from_lg2(ram_window_lg2_q);

    function is_rom_addr;
        input [31:0] a;
        begin
            is_rom_addr = ((a & ~(ROM_MIRROR_SIZE-1)) == ROM_MIRROR_BASE);
        end
    endfunction

    // ── Q700 ROM-switch overlay auto-disable ──────────────────────────
    // Models MAME's `quadrax00_state::rom_switch_r` (apple/macquadra700.cpp
    // L514): the very first CPU read from the ROM mirror range
    // (0x40000000..0x4FFFFFFF) clears the overlay sticky and exposes RAM
    // at low addresses.  VIA1 PB3 is NOT consulted on Q700 — the discrete
    // overlay flop on this generation is wired directly to the ROM-CS
    // address-decode side-effect, NOT to a 6522 port pin.
    //
    // We sample mr_araddr (the CPU-view address, BEFORE apply_cpu_overlay)
    // for masters M_CPU and M_CPUI — exactly the same predicate
    // apply_cpu_overlay uses below to decide which masters get the
    // overlay redirect.  Per the master fan-in, mr_midx[0]=M_CPU
    // (data) and mr_midx[2]=M_CPUI (instruction); indices 1 (XDMA) and
    // 3 (DMA) are NOT CPU masters and must be excluded — their reads
    // can land in ROM space too (XDMA loads ROM via the host loader
    // path), and counting those would disable the overlay at random
    // points in early boot, before the CPU has actually dereferenced
    // its reset vectors out of overlay-mapped low memory.  This bug
    // bit us once: index 3 is actually DMA, and at reset its arvalid
    // is uninitialized, so a glitch-high paired with any address that
    // happened to satisfy is_rom_addr fired the latch instantly,
    // making the SSP/PC vector reads at 0x0/0x4 go to uninit DDR
    // instead of the ROM mirror — the CPU jumped to a garbage PC and
    // F-line stormed.  Since the 2026-07-16 master-count reduction,
    // index 3's mr_arvalid is a hardwired 1'b0 constant (M4/DMA read is
    // permanently stubbed — see header note), so this particular glitch
    // class is now structurally impossible rather than merely unlikely.
    //
    // Clear on the CPU request, not on the downstream ready handshake.
    // The overlay side effect belongs to the CPU-visible ROM decode, and
    // tying it to S0 acceptance left a fragile timing hole where high-ROM
    // execution could begin while the low-memory overlay stayed active.
    //
    // Looking at the post-overlay address (mr_araddr_eff) would also
    // mis-detect: when overlay is on, every low-mem read shows up as
    // a 0x40000000+offset address, and that's normal overlay traffic,
    // NOT the trigger.  The trigger is a CPU access whose ORIGINAL
    // target is already in ROM space — typically the very first
    // instruction fetch after the reset-vector PC dereference (e.g.
    // 0x40004052 on Q700).
    //
    // Once asserted, the latch is sticky until either the xbar fabric reset
    // or the CPU cold-reset overlay reset.  `rst` may deliberately stay low
    // across a JTAG-initiated unified reset so the debug/service fabric can
    // accept the write that releases DBG_CONTROL[4]; the separate
    // cpu_overlay_reset input re-arms only this CPU-visible ROM alias latch.
    // cpu_overlay_active (driven by VIA1) is AND-ed in for older Mac
    // compatibility; on Q700 that input always reads "active" so the latch
    // is the dominant disable signal.
    reg cpu_overlay_disabled_q;
    function is_cpu_master_idx;
        input [SSEL_W-1:0] m;
        begin
            is_cpu_master_idx = (m == `XBAR_M_CPU) || (m == `XBAR_M_CPUI);
        end
    endfunction
    wire cpu_rom_read_seen =
        (mr_arvalid[0] && is_rom_addr(mr_araddr[0]) && is_cpu_master_idx(mr_midx[0])) ||
        (mr_arvalid[2] && is_rom_addr(mr_araddr[2]) && is_cpu_master_idx(mr_midx[2]));

    // ── DEFER the disarm until every outstanding CPU read has drained ──────────
    //
    // ROOT CAUSE OF THE 200 MHz RESET WEDGE (measured 2026-09-14). ResetVectorFsm
    // issues the 68040 reset-vector read at PHYSICAL 0, which only has a responder
    // while this overlay aliases it into the ROM mirror. `cpu_rom_read_seen` fires on
    // an I-fetch REQUEST -- including a SPECULATIVE one -- so a wrong-path fetch could
    // disarm the overlay WHILE that address-0 read was still in flight. The read then
    // had no target, no R was ever produced, AxiDMerge held the RESETVEC read grant,
    // and 10 s later the watchdog halted the core with ARBITER_WEDGE.
    // Confirmed by OFF_STALL_ARB = 0x8000010f at a live wedge: rd.busy=1, rd.arTaken=1,
    // rd.owner=3 (RESETVEC), rd.req=0.
    //
    // It is CLOCK-DEPENDENT because a faster core issues more speculative fetches per
    // unit WALL-CLOCK before the address-0 read completes: ~60% of cold boots wedge at
    // 200 MHz, and it is rare at 100 MHz -- which is why it read as "timing" for a long
    // time and was chased with placement and phys_opt instead.
    //
    // Fix: keep the disarm REQUEST sticky, but only commit it once no CPU read is
    // outstanding. Deferral is harmless (microseconds, and the requesting fetch itself
    // is one of the outstanding reads), while disarming early is fatal.
    //
    // ⚠ CORRECTED 2026-09-18 (race audit).  "No CPU read is outstanding" used to be a
    // 4-bit LEDGER: `cpu_ar_fire` (an AR handshake at M0/M2) incremented it,
    // `cpu_r_done` (an RLAST handshake) decremented it, and the disarm committed at
    // zero.  A ledger is only as good as its two events, and BOTH were wrong:
    //
    //   1. IT COMMITTED ONE CYCLE TOO EARLY.  `cpu_ar_fire` needs `mr_arready`, which
    //      this module asserts as a REGISTERED pulse -- so the handshake is visible one
    //      cycle AFTER the arb decided to accept.  The commit test read the
    //      PRE-increment value in that very cycle, which is exactly the window the fix
    //      above claims to have closed: the comment's own "the requesting fetch itself
    //      is one of the outstanding reads" was FALSE for the cycle in which that
    //      fetch's AR is accepted.  Pinned by scenario 53.
    //
    //   2. IT LEAKED, PERMANENTLY.  Every path that retires a read WITHOUT an RLAST
    //      handshake broke the ledger, and one exists: rs_r_abandon (backlog item 5)
    //      WITHDRAWS an unaccepted local R tail and idles the slot, deliberately,
    //      because the master is gone.  M2 is the CPU instruction fetch -- precisely a
    //      master that can be reset out from under an in-flight read.  One abandon and
    //      the count never returns to zero, so the hardware disarm never fires again
    //      and the overlay falls back to VIA1 software control alone: the pre-fix
    //      posture the 200 MHz wedge investigation concluded was not safe to rely on,
    //      reached by another route with nothing to report it.  Pinned by scenario 52.
    //      (The ledger had two further latent leaks -- a saturating 4-bit count, and
    //      only ONE decrement when both CPU slots return RLAST in the same cycle.)
    //
    // The slot FSM already IS the outstanding-read record, exactly and without a
    // ledger to leak: slot 0's reads are always the CPU LSU (boot_fsm is write-only,
    // see the M0/boot merge note) and slot 2 is the CPU instruction fetch.  rs_state
    // and mr_arready_r are written on the SAME edge, so rs_state is non-IDLE in the
    // same cycle the master observes arready -- one cycle EARLIER than the ledger's
    // increment, which is what closes (1) -- and it returns to RS_IDLE on every
    // release path including the abandons, which closes (2).  Costs 4 fewer flops.
    reg       cpu_overlay_disarm_pending;
    // Declared here, DRIVEN below the read-path state declarations (same idiom as the
    // b_abandon_s* pulses).  See gen_cpu_rd_busy.
    wire      cpu_rd_busy;

    always @(posedge clk) begin
        if (rst || cpu_overlay_reset) begin
            cpu_overlay_disabled_q     <= 1'b0;
            cpu_overlay_disarm_pending <= 1'b0;
        end else begin
            if (cpu_rom_read_seen)
                cpu_overlay_disarm_pending <= 1'b1;

            // commit only when nothing the overlay might be aliasing is still in flight
            if (cpu_overlay_disarm_pending && !cpu_rd_busy)
                cpu_overlay_disabled_q <= 1'b1;
        end
    end

    wire effective_cpu_overlay_active =
        cpu_overlay_active && !cpu_overlay_disabled_q;

    // Debug taps (see the port declarations above).
    assign cpu_overlay_disabled  = cpu_overlay_disabled_q;
    assign cpu_overlay_effective = effective_cpu_overlay_active;

    // Reset-wedge taps (see port declarations). slv_poisoned[] is an unpacked array,
    // so pack it; dbg_s1_slot_busy is "any write or read slot still outstanding to S1
    // (XBAR_SLV_IO)", which is the slot a swallowed BRESP would strand.
    genvar dpi;
    generate
        for (dpi = 0; dpi < 6; dpi = dpi + 1) begin : gen_dbg_poison
            assign dbg_slv_poisoned[dpi] = slv_poisoned[dpi];
        end
    endgenerate
    reg dbg_s1_busy_r;
    integer dbi;
    always @(*) begin
        dbg_s1_busy_r = 1'b0;
        for (dbi = 0; dbi < NW; dbi = dbi + 1)
            if (ws_active[dbi] && (ws_slv[dbi] == `XBAR_SLV_IO)) dbg_s1_busy_r = 1'b1;
        for (dbi = 0; dbi < NR; dbi = dbi + 1)
            if (rs_active[dbi] && (rs_slv[dbi] == `XBAR_SLV_IO)) dbg_s1_busy_r = 1'b1;
    end
    assign dbg_s1_slot_busy = dbg_s1_busy_r;

    // CPU reset overlay.  The core always presents raw CPU-visible
    // physical addresses; this fabric-level policy aliases low ROM-size
    // CPU data and instruction-fetch accesses to the ROM mirror while the
    // overlay is "active" (per Q700: until first ROM-mirror CPU read).
    // Host/debug, boot, and DMA masters keep raw access.
    function [31:0] apply_cpu_overlay;
        input [31:0] a;
        begin
            if (effective_cpu_overlay_active && (a < ROM_SIZE))
                apply_cpu_overlay = ROM_BASE | (a & (ROM_SIZE - 1));
            else
                apply_cpu_overlay = a;
        end
    endfunction

    function is_visible_low_ram_addr;
        input [31:0] a;
        begin
            is_visible_low_ram_addr =
                ((a & ~(RAM_SIZE-1)) == RAM_BASE) &&
                (RAM_ALIAS_MODE ||
                 ((((a - RAM_BASE) & ~ram_window_mask)) == 32'h0));
        end
    endfunction

    function is_visible_ram_alias_addr;
        input [31:0] a;
        begin
            is_visible_ram_alias_addr =
                ((a & ~(RAM_ALIAS_SIZE-1)) == RAM_ALIAS_BASE) &&
                (RAM_ALIAS_MODE ||
                 ((((a - RAM_ALIAS_BASE) & ~ram_window_mask)) == 32'h0));
        end
    endfunction

    wire [31:0] mw_awaddr_eff [0:NW-1];
    wire [31:0] mr_araddr_eff [0:NR-1];
    genvar ai;
    generate
        for (ai = 0; ai < NW; ai = ai + 1) begin : gen_aw_overlay
            // The reset ROM overlay is a read/fetch view.  CPU writes to
            // low memory must still initialize/test real RAM.
            assign mw_awaddr_eff[ai] = mw_awaddr[ai];
        end
        for (ai = 0; ai < NR; ai = ai + 1) begin : gen_ar_overlay
            assign mr_araddr_eff[ai] =
                ((mr_midx[ai] == `XBAR_M_CPU) ||
                 (mr_midx[ai] == `XBAR_M_CPUI)) ? apply_cpu_overlay(mr_araddr[ai])
                                                : mr_araddr[ai];
        end
    endgenerate

    // Address decode.  DMA checked before IO (hole-in-IO).  VRAM and
    // DAFB sit in 0xF9 windows and do not overlap each other.
    function [SSEL_W-1:0] decode_slv;
        input [31:0] a;
        begin
            if (is_visible_low_ram_addr(a) || is_visible_ram_alias_addr(a))
                decode_slv = `XBAR_SLV_DDR;
            else if (is_rom_addr(a))
                decode_slv = `XBAR_SLV_DDR;
            else if ((a & ~(FB_SIZE -1)) == FB_BASE)
                decode_slv = `XBAR_SLV_DDR;
            else if ((a & ~(DMA_SIZE-1)) == DMA_BASE)
                decode_slv = `XBAR_SLV_DMA;
            else if ((a & ~(SD_JTAG_SIZE-1)) == SD_JTAG_BASE)
                decode_slv = `XBAR_SLV_SDJTAG;
            else if ((a & ~(IO_SIZE -1)) == IO_BASE)
                decode_slv = `XBAR_SLV_IO;
            else if ((a & ~(VRAM_SIZE-1)) == VRAM_BASE)
                decode_slv = `XBAR_SLV_VRAM;
            else if ((a & ~(DAFB_SIZE-1)) == DAFB_BASE)
                decode_slv = `XBAR_SLV_DAFB;
            else
                decode_slv = `XBAR_SLV_NONE;
        end
    endfunction

    function [31:0] ddr_flatten;
        input [31:0] a;
        begin
            if ((a & ~(RAM_SIZE-1)) == RAM_BASE)
                ddr_flatten = ((a - RAM_BASE) &
                               (RAM_ALIAS_MODE ? ram_window_mask : 32'hffff_ffff)) +
                              `AXI_DDR_RAM_OFFSET;
            else if ((a & ~(RAM_ALIAS_SIZE-1)) == RAM_ALIAS_BASE)
                ddr_flatten = ((a - RAM_ALIAS_BASE) &
                               (RAM_ALIAS_MODE ? ram_window_mask : 32'hffff_ffff)) +
                              `AXI_DDR_RAM_OFFSET;
            else if (is_rom_addr(a))
                // ROM_IMAGE_SIZE, not ROM_SIZE: the mirror repeats every
                // 1 MiB like the real Q700 (and MAME).  Using the 4 MiB
                // DDR reservation here made 3 of every 4 MiB return
                // uninitialised DRAM.  Task #245.
                ddr_flatten = ((a - ROM_MIRROR_BASE) & (ROM_IMAGE_SIZE - 1)) +
                              `AXI_DDR_ROM_OFFSET;
            else if ((a & ~(FB_SIZE -1)) == FB_BASE)
                ddr_flatten = (a - FB_BASE)  + `AXI_DDR_FB_OFFSET;
            else
                ddr_flatten = a;
        end
    endfunction

    // Zero-base the VRAM aperture address before forwarding to the
    // `vram` slave — the slave expects a byte offset within its own
    // framebuffer, not the system-level physical address.
    function [31:0] vram_flatten;
        input [31:0] a;
        begin
            vram_flatten = a - VRAM_BASE;
        end
    endfunction

    // Zero-base the DAFB register aperture before forwarding to the
    // register shim.  Unlike VRAM, no byte swap is applied at this
    // boundary; the 32-bit AXI-Lite shim consumes normal system lanes.
    function [31:0] dafb_flatten;
        input [31:0] a;
        begin
            dafb_flatten = a - DAFB_BASE;
        end
    endfunction

    function [31:0] sd_jtag_flatten;
        input [31:0] a;
        begin
            sd_jtag_flatten = a - SD_JTAG_BASE;
        end
    endfunction

    function is_rom_write;
        input [31:0] a;
        begin
            is_rom_write = is_rom_addr(a);
        end
    endfunction

    // Burst-legalization (Fix 1): slaves that only accept single-beat
    // (len==0) AXI transactions.  S0 (DDR) and S3 (VRAM/URAM) are
    // burst-capable and deliberately excluded — see the module-header
    // note above.  Expressed as an explicit-compare function (not an
    // array indexed by the runtime decode value) so the XBAR_SLV_NONE
    // (3'd7) sentinel can never read out-of-range/X — it simply doesn't
    // match any of the four compares and the function returns 0.
    function is_lite_only_slv;
        input [SSEL_W-1:0] slv;
        begin
            is_lite_only_slv = (slv == `XBAR_SLV_IO)   ||
                                (slv == `XBAR_SLV_DMA)  ||
                                (slv == `XBAR_SLV_DAFB) ||
                                (slv == `XBAR_SLV_SDJTAG);
        end
    endfunction

    // slv_flush domain membership.  This is DELIBERATELY a DIFFERENT set
    // than is_lite_only_slv() above — burst-legalization asks "can this
    // slave accept a burst" (protocol shape); slv_flush asks "does this
    // slave's downstream actually reset on soc_full_rst_bank[4]" (reset
    // topology).
    //   - S3 (VRAM) is burst-capable (excluded from is_lite_only_slv) but
    //     is in the flush set in both topologies.  Local URAM resets
    //     and forgets the transaction.  VRAM_IN_DDR survives the reset, so
    //     the S3 cleanup contexts below drain its accepted write/read before
    //     allowing a fresh transaction to use that route.
    //   - S0 (DDR) is NOT in the set, and the backend genuinely DOES survive
    //     and will answer for real.  Putting S0 in the flush set would need
    //     the S3_BACKEND_SURVIVES_FLUSH style late-response quarantine for
    //     the DDR path first; that is NOT done here.
    //
    //     ⚠ CORRECTED 2026-09-18 -- THE REASON RECORDED HERE WAS WRONG, and
    //     it was being used to justify decisions.  This used to read "the
    //     MIG and (with L2C_ENABLE) l2c both stay on core_rst_bank[4] and
    //     deliberately skip the ~100 ms DDR re-calibration a soc_full_rst
    //     would otherwise force".  Two errors:
    //
    //       1. THE MIG IS NOT ON core_rst_bank[4] AT ALL.  Its reset is its
    //          own net -- `.sys_rst(~btn[0] | soc_hard_rst_req)`,
    //          fpga_top_ddr.vh -- so what core_rst_bank[4] does or does not
    //          do cannot cause a re-calibration either way.  Re-cal is NOT
    //          the cost of moving l2c, ddr_ctrl or this crossbar.
    //       2. The blocks actually on core_rst_bank[4] are u_l2c,
    //          u_ifetch_guard and u_vram_lane_mux (u_ddr is on plain
    //          core_rst).  The real reason NOT to reset u_l2c is much
    //          stronger than re-cal and is not written down anywhere:
    //          l2c is a WRITEBACK cache and its reset is l2c_reset.v, a
    //          walking tag-clear that INVALIDATES every set with no
    //          writeback.  Resetting it discards every dirty line still in
    //          the victim buffer -- silent RAM corruption on every JTAG
    //          reset, which is far worse than the wedge that motivates
    //          resetting it.
    //
    //     And the reason not to reset THIS crossbar with the CPU is a third
    //     thing again: M1 (host debug / JTAG-AXI) is on `core_rst`
    //     (fpga_top_debug_host.vh) and deliberately SURVIVES a
    //     debug-full-reset -- it is the debug path, so it must.  A crossbar
    //     reset under a live M1 orphans M1's in-flight transaction, which is
    //     the same stranding asymmetry, aimed at the one master that cannot
    //     be reset to recover.
    //
    //     Keeping the conclusion, correcting the argument: the three facts
    //     above are what a future "reset the whole SoC bus as one unit"
    //     proposal has to answer.  See docs/bus_debug_split_plan.md.
    //
    // S1 JOINED THIS SET on 2026-09-12 (work item 3).  It used to be
    // excluded, and the exclusion used to be right, for a reason that no
    // longer exists:
    //
    //   OLD: S1's CDC bridge + peripheral_bus ran on core_rst /
    //   pb_core_rst banks and were kept ALIVE through soc_full_rst, so
    //   that the host could still reach debug_ctrl — the 0x5090_0000
    //   debug window was behind S1.  Flush-aborting S1 would then have
    //   orphaned a real B still in flight inside a LIVE bridge (which
    //   aliases onto S1's next write; nothing downstream checks BID) and
    //   returned spurious SLVERR from a slave that was about to answer
    //   for real.  The contract instead was "in-flight S1 transactions
    //   ride out the reset with the watchdog frozen".
    //
    //   WHY THAT BROKE.  That contract has no release path when the
    //   watchdog is the only one and ENABLE_WD = 0 (owner directive,
    //   2026-09-06).  MEASURED on hardware 2026-09-12: after a JTAG
    //   `reset`, dbg_s1_slot_busy reads 1 (0 while running) with
    //   dbg_slv_poisoned = 0x00 in BOTH states.  The slot is simply
    //   STRANDED — not poisoned, not timed out, just never released.  S1
    //   is then write-dead for every master until core_rst.
    //
    //   WHAT CHANGED.  The debug window no longer lives behind S1: the
    //   single JTAG-AXI bridge now lands on axi_dbg_bus, which serves
    //   0x5090_0000 from local core_clk slaves and masters this crossbar
    //   only for everything else.  Nothing needs S1 alive across a
    //   soc_full_rst any more, so fpga_top_peripherals.vh moves the S1
    //   CDC and peripheral_bus onto soc_full_rst_bank[1] /
    //   pb_soc_full_rst_bank[3] — the same event this input carries.
    //   With the bridge genuinely in reset there IS no live bridge to
    //   orphan a B in, and SLVERR during the window is the TRUTHFUL
    //   answer rather than a spurious one.
    //
    //   Note the pb-side reset is pb_soc_full_rst_bank, NOT
    //   pb_full_rst_bank: the latter also carries warm_peripheral_reset
    //   (a 68040 RESET instruction), which does NOT assert this input and
    //   does NOT reset the CPU.  Resetting the S1 front door under a
    //   still-running CPU would create a brand-new stranding class.
    //
    // Net membership: S1, S2, S3, S4, S5.  NOT S0.
    function is_flush_domain_slv;
        input [SSEL_W-1:0] slv;
        begin
            is_flush_domain_slv = (slv == `XBAR_SLV_IO)   ||
                                   (slv == `XBAR_SLV_DMA)  ||
                                   (slv == `XBAR_SLV_VRAM) ||
                                   (slv == `XBAR_SLV_DAFB) ||
                                   (slv == `XBAR_SLV_SDJTAG);
        end
    endfunction

    function [31:0] vram_swap_word32;
        input [31:0] w;
        begin
            vram_swap_word32 = {w[7:0], w[15:8], w[23:16], w[31:24]};
        end
    endfunction

    function [DATA_WIDTH-1:0] vram_swap_words;
        input [DATA_WIDTH-1:0] d;
        integer wi;
        reg [DATA_WIDTH-1:0] tmp;
        begin
            tmp = {DATA_WIDTH{1'b0}};
            for (wi = 0; wi < DATA_WIDTH/32; wi = wi + 1)
                tmp[wi*32 +: 32] = vram_swap_word32(d[wi*32 +: 32]);
            vram_swap_words = tmp;
        end
    endfunction

    function [STRB_WIDTH-1:0] vram_swap_strb;
        input [STRB_WIDTH-1:0] s;
        integer si;
        reg [STRB_WIDTH-1:0] tmp;
        reg [3:0] nib;
        begin
            tmp = {STRB_WIDTH{1'b0}};
            for (si = 0; si < STRB_WIDTH/4; si = si + 1) begin
                nib = s[si*4 +: 4];
                tmp[si*4 +: 4] = {nib[0], nib[1], nib[2], nib[3]};
            end
            vram_swap_strb = tmp;
        end
    endfunction

    // ── Fix 2: per-slave outstanding-transaction watchdog + poison ─────
    // See the module-header note.  WD_MAX_CNT is the last cycle-count
    // value reached before firing: a transaction becomes "active" (its
    // watchdog counter starts at 0) the cycle after the xbar accepts its
    // AW/AR, so firing at WD_MAX_CNT means it timed out after exactly
    // 2^WD_LOG2 active cycles without B/RLAST.  WD_CNT_W is sized for
    // the WORST-CASE default (WD_LOG2_S1 = 27) regardless of what a given
    // build overrides it to — a smaller override just leaves the extra
    // high bits of the counter always 0, which is harmless (the
    // "==WD_MAX_CNT" compare still only matches the smaller value).
    //
    // Policy (review MINOR 5): the counter measures TOTAL transaction
    // duration end-to-end — AW/AR-accepted-by-xbar through B/RLAST —
    // including any stall on the MASTER side (e.g. a slow BREADY/RREADY,
    // or a master that hasn't finished streaming W yet).  This is
    // deliberate, not an oversight: the watchdog's job is "did this
    // transaction ever complete", not "is the SLAVE specifically slow";
    // a master wedged on its own ready/valid is an equally real fabric
    // hang, and the default WD_LOG2=18 bound is generous enough
    // (~2.6 ms @ 100MHz, see the WD_LOG2 parameter comment for why that
    // specific value) that a healthy master/slave pair is never at risk
    // of a false trip.
    localparam WD_CNT_W = 27;
    // All-ones right-shifted by (WD_CNT_W-WD_LOG2) leaves exactly
    // WD_LOG2 ones in the low bits and zeros above — i.e. 2^WD_LOG2-1 —
    // for any WD_LOG2 in [0, WD_CNT_W] including the WD_LOG2==WD_CNT_W
    // default (shift-by-0, all ones).  WD_LOG2 / WD_LOG2_S1 must not exceed
    // WD_CNT_W=27.
    localparam [WD_CNT_W-1:0] WD_MAX_CNT =
        {WD_CNT_W{1'b1}} >> (WD_CNT_W - WD_LOG2);
    // S1's larger bound (see WD_LOG2_S1 parameter comment).
    localparam [WD_CNT_W-1:0] WD_MAX_CNT_S1 =
        {WD_CNT_W{1'b1}} >> (WD_CNT_W - WD_LOG2_S1);

    // ── B-hold grace bound (task #219, 2026-08-02) ─────────────────────
    // MINOR 4 (see gen_ws_wd below) suppresses the write watchdog while
    // the real slave's BVALID is asserted, because that watchdog POISONS
    // and the slave has demonstrably answered.  Correct — but it left NO
    // escape at all for a master that walks away between AW and B, which
    // is exactly what axi_narrow_to_wide.v produces when its own
    // abandonment watchdog force-clears aw_valid_q, taking its BREADY
    // low forever.  sw_owned[slv] was then pinned at 1 permanently and
    // the slave was dead to EVERY master until the next rst.  (The read
    // side never had this shape: rs_wd_fire suppresses on an actual
    // RVALID&&RREADY HANDSHAKE, not on RVALID alone — see gen_rs_wd.)
    //
    // B_HOLD_LOG2 is how long BVALID may sit unaccepted before the
    // master is treated as gone rather than merely slow.  1024 cycles
    // (~10 us @ 100 MHz) is orders of magnitude more than any live
    // master needs to raise BREADY.  Overridable (tb-axi-xbar builds
    // with -GB_HOLD_LOG2=6) so the recovery path is testable in-sim.
    localparam B_HOLD_CNT_W = 16;
    localparam [B_HOLD_CNT_W-1:0] B_HOLD_MAX =
        {B_HOLD_CNT_W{1'b1}} >> (B_HOLD_CNT_W - B_HOLD_LOG2);

    // slv_poisoned[s] (s = XBAR_SLV_* index 0..5, i.e. a plain NS-sized
    // array indexed only by compile-time-constant literals below — never
    // by a runtime decode value) is the OR of the write-side and
    // read-side poison latches.  Two single-driver regs instead of one
    // reg written from both always blocks (illegal/undefined multi-
    // driver reg in standard Verilog).
    reg  slv_w_poisoned [0:NS-1];
    reg  slv_r_poisoned [0:NS-1];
    wire slv_poisoned   [0:NS-1];
    genvar gi;
    generate
        for (gi = 0; gi < NS; gi = gi + 1) begin : gen_poison_or
            assign slv_poisoned[gi] = slv_w_poisoned[gi] | slv_r_poisoned[gi];
        end
    endgenerate

    // slv_flush rising-edge detector (CRITICAL fix).  A single small
    // always block, independent of the write/read always blocks below,
    // so both of them can consume `slv_flush_rise` as a plain
    // combinational condition without a multi-driver hazard.
    reg slv_flush_q;
    always @(posedge clk) begin
        if (rst) slv_flush_q <= 1'b0;
        else     slv_flush_q <= slv_flush;
    end
    wire slv_flush_rise = slv_flush && !slv_flush_q;

    // ── S1 RESET-DEASSERT RACE (backlog item 3, 2026-09-17) ─────────────
    //
    // slv_flush is soc_full_rst_bank[4], a core_clk net.  S1's far side --
    // the axi_pb_s1_cdc bridge and peripheral_bus behind it -- resets on
    // pb_soc_full_rst_bank[3], which is the SAME source event released
    // through a SEPARATE xpm_cdc_async_rst in the pb_clk domain
    // (fpga_top_clocks.vh's u_pb_clk_rst).  Both use DEST_SYNC_FF = 4, but
    // four pb_clk at 50 MHz is up to SIXTEEN core_clk at 200 MHz, and the
    // CDC's own FIFO pointers still have to resync after that.
    //
    // So slv_flush falls FIRST, and for a window afterwards the crossbar
    // believes S1 is available while the bridge behind it is still in
    // reset.  A transaction admitted in that window is accepted by the
    // crossbar, handed to a bridge that forgets it, and then nothing ever
    // completes it:
    //
    //   * slv_flush is already low, so ws_flush_abort / rs_flush_abort
    //     cannot fire for it;
    //   * ENABLE_WD defaults to 0 (owner directive, 2026-09-06), so
    //     ws_wd_fire / rs_wd_fire are constant-false;
    //   * ws_b_abandon needs ws_wd_cnt == WD_MAX_CNT_S1, i.e. 2^27 cycles,
    //     and in any case only fires when the SLAVE produced a B;
    //   * `rst` here is core_rst_bank[4], which a debug-full-reset does
    //     not assert.
    //
    // The slot is stranded and S1 is dead to every master until core_rst
    // -- the exact end state that was MEASURED on hardware after a JTAG
    // `reset` (dbg_s1_slot_busy = 1, dbg_slv_poisoned = 0x00) and that
    // moving S1 into the flush domain was supposed to have retired.  It
    // fixed the transaction in flight WHEN the flush lands; it did not fix
    // the one admitted just after the flush lifts.
    //
    // Fix: S1 stays closed for S1_RST_TAIL_LOG2 more cycles.  The request
    // is HELD, not rejected -- no AW/AR is accepted, so the master simply
    // retries and sees a bounded stall.  Deliberately NOT a local SLVERR
    // like the in-flush rejection: during the flush an error is the
    // truthful answer (the slave really is gone), whereas during the tail
    // the slave is merely not ready yet, and manufacturing a bus error for
    // that would be inventing a fault.  It also matches this module's
    // stated policy on bounded watchdogs -- fix what stops progress, do
    // not launder it into a fatal bus error.
    //
    // S1-ONLY, deliberately.  S2/S4/S5's axi_wide_to_axilite bridges sit on
    // soc_full_rst_bank[4] -- the very net slv_flush carries -- so they
    // release in lockstep with it and have no race.  S3's local VRAM is
    // core-domain too.  S1 is the only slave whose reset release is
    // counted in another clock.
    //
    // Armed by THREE events, not two: `rst`, `slv_flush`, and `s1_far_reset`
    // -- see that port's comment for the pb island's third reset source
    // (warm_peripheral_reset, the 68040 RESET instruction), which reaches
    // neither of the first two.
    //
    // TIMING: the consumer term is a single plain REGISTER
    // (s1_rst_tail_busy), added as one more AND input to req_io[]/
    // req_rd_io[], cones that are already ~10 inputs wide and therefore
    // already two LUT levels.  Nothing gets deeper.  The `!= 0` reduce
    // that would have cost a level is kept OUT of that path by carrying
    // the busy flag as its own flop.
    localparam S1_TAIL_CNT_W = 16;
    localparam [S1_TAIL_CNT_W-1:0] S1_TAIL_MAX =
        {S1_TAIL_CNT_W{1'b1}} >> (S1_TAIL_CNT_W - S1_RST_TAIL_LOG2);
    reg [S1_TAIL_CNT_W-1:0] s1_rst_tail_cnt;
    reg                     s1_rst_tail_busy;
    always @(posedge clk) begin
        // s1_far_reset joins rst/slv_flush here, and ONLY here: it says
        // nothing about a transaction already accepted (see its port
        // comment), so it must not arm rs_flush_abort / ws_flush_abort.
        if (rst || slv_flush || s1_far_reset) begin
            s1_rst_tail_cnt  <= S1_TAIL_MAX;
            s1_rst_tail_busy <= 1'b1;
        end else if (s1_rst_tail_cnt != {S1_TAIL_CNT_W{1'b0}}) begin
            s1_rst_tail_cnt  <= s1_rst_tail_cnt -
                                {{(S1_TAIL_CNT_W-1){1'b0}}, 1'b1};
            s1_rst_tail_busy <= (s1_rst_tail_cnt !=
                                 {{(S1_TAIL_CNT_W-1){1'b0}}, 1'b1});
        end
    end

    // AXI4 burst-type encoding (review MINOR 7).  Local to this module —
    // axi_defs.vh is out of scope for this task.
    localparam [1:0] AXI_BURST_FIXED = 2'b00;
    localparam [1:0] AXI_BURST_INCR  = 2'b01;
    localparam [1:0] AXI_BURST_WRAP  = 2'b10;

    // ── Write path ────────────────────────────────────────────────────
    localparam [2:0] WS_IDLE         = 3'd0;
    localparam [2:0] WS_WAIT_SLV_AW  = 3'd1;
    localparam [2:0] WS_FWD_W        = 3'd2;
    localparam [2:0] WS_WAIT_B       = 3'd3;
    localparam [2:0] WS_DRAIN_W      = 3'd4;
    localparam [2:0] WS_SEND_BLOCAL  = 3'd5;
    // WS_PAD_W — the crossbar itself finishes a write burst whose MASTER
    // went away mid-burst, by pushing wstrb=0 filler beats (with WLAST on
    // the burst's real last beat) into the slave.  See the "Abandoned-burst
    // W padding" block below for why this exists and why the alternatives
    // are worse.
    localparam [2:0] WS_PAD_W        = 3'd6;

    reg  [2:0]            ws_state    [0:NW-1];
    reg  [SSEL_W-1:0]     ws_slv      [0:NW-1];
    reg  [ID_WIDTH-1:0]   ws_mid      [0:NW-1];
    // W burst fully forwarded (wlast beat accepted by the slave) while
    // still in WS_WAIT_SLV_AW — lets the AW-accept transition skip
    // WS_FWD_W and go straight to WS_WAIT_B.  See the early-W note at
    // the ws*_drive assigns below.
    reg                   ws_wdone    [0:NW-1];
    reg  [31:0]           ws_addr_flat[0:NW-1];
    reg  [7:0]            ws_len      [0:NW-1];
    reg  [2:0]            ws_size     [0:NW-1];
    reg  [1:0]            ws_burst    [0:NW-1];
    reg  [1:0]            ws_local_rsp[0:NW-1];
    reg  [7:0]            ws_bcnt     [0:NW-1];
    reg  [WD_CNT_W-1:0]   ws_wd_cnt   [0:NW-1]; // Fix 2 watchdog
    // Task #219: how long the real slave's B has been asserted without
    // the owning master taking it.  Bounds the MINOR-4 suppression.
    reg  [B_HOLD_CNT_W-1:0] ws_bhold_cnt [0:NW-1];
    // ── Race audit 2026-09-18: the LOCAL-completion hold counter ────────
    // ws_bhold_cnt above watches a SLAVE-sourced B (mw_bvalid_slv).  The
    // two states where the CROSSBAR itself is the other party had no such
    // counter and no abandonment path at all -- the exact hole backlog
    // item 5 closed on the read side (rs_r_abandon / RS_SEND_RLOCAL),
    // never closed on its write-side twin:
    //
    //   WS_DRAIN_W      exits ONLY on `mw_wvalid && mw_wready_raw`, and
    //                   its WREADY is a hard 1'b1 (gen_wready).  A master
    //                   that stops sending W pins the slot forever -- and
    //                   because WREADY stays HIGH, that master's NEXT
    //                   write's beats are then eaten as this dead burst's
    //                   continuation (the same W-channel misalignment
    //                   IMPORTANT 2a names, reached from the other side).
    //   WS_SEND_BLOCAL  exits ONLY on `mw_bvalid_loc && mw_bready`.  A
    //                   master that never takes the local B pins the slot
    //                   forever, and the stale B is then mis-attributed to
    //                   whatever that master does next.
    //
    // Neither state is in `ws_active`, so ws_wd_cnt is held at zero for
    // both and ws_wd_fire can never fire there; ws_flush_abort requires
    // ws_active too.  This file already knew: ws_release_slot's GAP 1 note
    // says a misrouted local B would "wedge the slot in the unwatchdogged
    // WS_SEND_BLOCAL" -- and worked around one ENTRY into the state
    // instead of protecting the state.
    //
    // ONE counter serves both because a slot is in at most one of the two
    // at a time.  Deadline, freeze-under-flush and saturation are
    // rs_r_abandon's, verbatim -- that is the closest analogue (a tail the
    // crossbar itself produces, with no slave involved), not ws_b_abandon,
    // which additionally gates on the full watchdog because it is stealing
    // a REAL slave's B.
    // Negative controls: scenarios 50 and 51 in tb_axi_xbar.cpp.
    reg  [B_HOLD_CNT_W-1:0] ws_lhold_cnt [0:NW-1];

    // ── Abandoned-burst W padding (task: xbar-burst-gaps, GAP 1) ────────
    //
    // THE HAZARD.  Slot 0 is the CPU-LSU / boot-FSM SHARED physical port
    // (see the M0/boot merge header note).  `m0_wsel_q` freezes for the
    // life of a slot-0 write, so if `cpu_held_in_reset` flips MID-BURST the
    // side that issued the write has been reset away: it will send no more
    // W beats and consume no B.  The slot then sits in WS_FWD_W with the
    // slave parked mid-burst until the per-slave watchdog fires 2^WD_LOG2
    // cycles later — with poison=1, which SLVERRs that slave for every
    // master until the next `rst`.  On S0 that kills the whole DDR path
    // because one JTAG `reset hold` landed mid-writeback.  With single-beat
    // writes the exposure window was the slave's WREADY latency; with the
    // CPU's D-cache burst refill it is the whole burst, on every writeback.
    //
    // WHY NOT JUST SUPPRESS THE POISON.  Because the slave genuinely IS
    // left mid-burst.  Releasing it un-poisoned lets the next write's AW be
    // admitted while the slave is still counting down the OLD burst, so
    // that write's W beats are consumed as the stale burst's continuation
    // and physically commit to the OLD address — and from then on the AW/W
    // pairing is permanently shifted.  Silent memory corruption in place of
    // a loud death is strictly worse, and it is the exact failure
    // axi_bridge_w_pad.v (task #162) exists to prevent one hop further out.
    //
    // THE FIX.  Complete the abandoned burst here, with wstrb=0 no-op
    // filler beats and a correct WLAST, so the slave's own write FSM
    // finishes and returns to idle BEFORE any fresh write is admitted; then
    // swallow the B it produces (the owner is gone and must not be handed a
    // response) and release `sw_owned`.  Same idiom, same shape, as
    // axi_bridge_w_pad.v's head_beats/head_is_last, l2c_victim.v's
    // S_RSTDRAINW and axi_ddr4_mig_bridge.v's wr_pad_active pusher.
    //
    // ws_wbeats[i] — W beats the SLAVE still owes this burst, including the
    // one currently being presented (AWLEN+1 at accept, decremented on
    // every accepted beat, real or filler).  Burst boundaries come from
    // AWLEN, not from the master's WLAST, because AWLEN is what the slave
    // itself counts — axi_bridge_w_pad.v's reasoning verbatim.
    reg  [8:0]            ws_wbeats     [0:NW-1];
    // Registered (ws_wbeats[i] == 1): "the beat now being presented is this
    // burst's last".  REGISTERED, not compared combinationally, because it
    // drives WLAST at the slot-0 fan-in mux — a 9-bit compare there would
    // add a LUT level to an AXI payload path in a design that closes at
    // ~+0.019 ns WNS.
    reg                   ws_wlast_pend [0:NW-1];

    // S3 can be backed by the always-live DDR path.  A slv_flush may retire
    // its upstream slot after AW was accepted without resetting that
    // downstream transaction.  This single-entry cleanup context pads any
    // remaining W beats with zero strobes and consumes the stale B before a
    // fresh S3 AW is admitted.
    reg                   s3_flush_active;
    reg  [8:0]            s3_flush_wbeats;
    reg  [WD_CNT_W-1:0]   s3_flush_wd_cnt;

    // ── Slot-0 fan-in select register (see the big note at the mw_*[0]
    //    assigns above for why the select must not be combinational) ────
    // m0_wsel_busy marks "a slot-0 write is in flight".  It is set on the
    // slot-0 AW handshake and cleared when slot 0 returns to WS_IDLE —
    // deliberately keyed on the state, not on the B handshake, because one
    // exit path (the slot-0 owner-retired discard in ws_release_slot) goes
    // straight back to WS_IDLE with no B at all and would otherwise leave
    // the select frozen forever.
    //
    // m0_wsel_q tracks cpu_held_in_reset while idle and holds while busy.
    // No combinational loop: both are registers, and mw_awvalid[0] /
    // mw_awready[0] depend on m0_wsel_q, never the reverse.  Note that
    // mw_awready[0] can only assert while ws_state[0] == WS_IDLE (the
    // arbiter gates every req_* on it), so the set and clear terms cannot
    // fight over a single transaction.
    always @(posedge clk) begin
        if (rst)
            m0_wsel_busy <= 1'b0;
        else if (mw_awvalid[0] && mw_awready[0])
            m0_wsel_busy <= 1'b1;
        else if (ws_state[0] == WS_IDLE)
            m0_wsel_busy <= 1'b0;

        // Tracks cpu_held_in_reset while slot 0 is idle, freezes while a
        // slot-0 write is in flight.  `rst` is folded into the tracking
        // condition rather than given its own constant reset value so the
        // select comes OUT of reset already matching cpu_held_in_reset:
        // at core_rst release the CPU is still held (boot_rom_ready low),
        // and a constant 0 here would have deselected boot_fsm for one
        // cycle.  Harmless (boot would simply retry its AW) but it made
        // the select differ from the old combinational behaviour for
        // single-beat traffic, which this change must not do.
        if (rst || !m0_wsel_busy)
            m0_wsel_q <= cpu_held_in_reset;

        // GAP 1: latch "the owner of this in-flight slot-0 write is gone".
        // Lives here rather than in the big write always-block because it is
        // a companion of the select, not of the slot FSM — and keeping it a
        // single-driver reg in this block lets the write block read it
        // freely.  Cleared the moment slot 0 is idle again (the state read
        // here is the OLD value, so a same-cycle re-arm of the slot by the
        // arbiter still clears correctly).
        //
        // GAP 1b (2026-09-17) — THE MISMATCH TEST IS STRUCTURALLY BLIND WHEN
        // BOTH SIDES ARE HELD.  `m0_wsel_q != cpu_held_in_reset` can only
        // ever detect an owner that went away by *handing the port to the
        // other side*.  It cannot see the case where the owner is reset and
        // the select does not move:
        //
        //   boot_fsm owns the slot (m0_wsel_q = cpu_held_in_reset = 1) with a
        //   write in flight to S0/DDR.  A JTAG debug-full-reset raises
        //   soc_full_rst.  boot_fsm_rst = soc_full_rst || ... (see
        //   fpga_top_clocks.vh) so boot_fsm restarts and forgets the write,
        //   but cpu_held_in_reset STAYS 1 — soc_full_rst re-clears
        //   boot_rom_loaded, so the CPU is still held.  1 != 1 is false, so
        //   m0_abandon never sets.
        //
        // and the slave side does not rescue it either, because the one
        // slave boot_fsm actually writes — S0/DDR — is deliberately NOT in
        // the flush domain (is_flush_domain_slv(): the MIG/l2c stay on
        // core_rst to skip re-calibration), so ws_flush_abort[0] cannot fire
        // for it.  The slot parks in WS_FWD_W or WS_WAIT_B offering a B that
        // nobody will ever take; ws_wd_fire is suppressed while
        // mw_bvalid_slv is high (MINOR 4), so not even the watchdog frees
        // it.  sw_owned[0] stays set and DDR is dead to every master until
        // the FPGA is reconfigured — `rst` here is core_rst_bank[4], which a
        // debug-full-reset does not assert.
        //
        // The fix is to name the event directly.  `slv_flush` IS
        // soc_full_rst_bank[4], i.e. exactly the event that resets BOTH
        // slot-0 masters (boot_fsm via boot_fsm_rst; the CPU via
        // boot_rom_loaded -> cpu_rst).  Its slave-side meaning is a
        // coincidence of the reset topology, not a restriction: for the
        // master-abandonment question it is the correct predicate no matter
        // which slave the burst is aimed at.  Ored in unconditionally rather
        // than only for the boot side — the CPU side reaches the same
        // conclusion one cycle later through the mismatch term anyway, and
        // an early, uniform abandon is strictly the safer of the two.
        //
        // Cost: one extra input on this register's D (m0_wsel_busy,
        // m0_wsel_q, cpu_held_in_reset, slv_flush, plus the clear terms) —
        // the cone was already >6 inputs and stays at two LUT levels.  No
        // AXI ready/valid path is touched; m0_abandon is consumed only by
        // register-selected muxes that already read it.
        if (rst || (ws_state[0] == WS_IDLE))
            m0_abandon <= 1'b0;
        else if (m0_wsel_busy && ((m0_wsel_q != cpu_held_in_reset) || slv_flush ||
                                  m0b_master_reset))
            m0_abandon <= 1'b1;
    end

    // sw_owned locks a slave's whole AW→B sequence to one master: the
    // next AW (from ANY master) is not even arb-latched until the
    // previous write's B has been consumed.  Combined with the single
    // per-master ws_state FSM this means 1 outstanding write per master
    // AND per slave.
    //
    // 2026-07-21 investigation — write-side AW pipelining (accept a 2nd
    // AW behind an in-flight B) was analyzed and deliberately NOT
    // implemented:
    //  • Cross-master overlap on one slave (unlock at wlast instead of
    //    B-consumed) would need B routing switched from sw_owner-based
    //    (s?_bready / mw_bvalid_slv below) to bid-slot-based like the
    //    read side's s?_rtgt.  Mechanical, but today every live slave
    //    (ddr_ctrl, peripheral_bus, axi_wide_to_axilite×3, vram) is
    //    itself single-outstanding on writes — the unlock would only
    //    present AW2 one handshake earlier, a marginal win.
    //  • Same-master pipelining (the case that would actually help the
    //    CPU store stream) needs the per-master single-transaction FSM
    //    to become a 2-entry queue with W-burst boundary tracking, and
    //    — if the two AWs decode to DIFFERENT slaves — B responses can
    //    complete out of order, which violates AXI same-ID B ordering
    //    for a master reusing one AWID (the LSU does) unless the xbar
    //    reorders B.  That is a substantial, risk-bearing redesign.
    // Revisit only with a dedicated design pass + multi-outstanding BFM
    // coverage; do not bolt onto the current owner scheme.
    reg  [SLOT_W-1:0]     sw_owner    [0:NS-1];
    reg                   sw_owned    [0:NS-1];
    reg  [SLOT_W-1:0]     sw_rr_ptr   [0:NS-1];

    reg  [XID_WIDTH-1:0]  s_awid_r    [0:NS-1];
    reg  [31:0]           s_awaddr_r  [0:NS-1];
    reg  [7:0]            s_awlen_r   [0:NS-1];
    reg  [2:0]            s_awsize_r  [0:NS-1];
    reg  [1:0]            s_awburst_r [0:NS-1];
    reg                   s_awvalid_r [0:NS-1];

    assign s0_awid    = s_awid_r   [0];
    assign s0_awaddr  = s_awaddr_r [0];
    assign s0_awlen   = s_awlen_r  [0];
    assign s0_awsize  = s_awsize_r [0];
    assign s0_awburst = s_awburst_r[0];
    assign s0_awvalid = s_awvalid_r[0];

    assign s1_awid    = s_awid_r   [1];
    assign s1_awaddr  = s_awaddr_r [1];
    assign s1_awlen   = s_awlen_r  [1];
    assign s1_awsize  = s_awsize_r [1];
    assign s1_awburst = s_awburst_r[1];
    assign s1_awvalid = s_awvalid_r[1];

    assign s2_awid    = s_awid_r   [2];
    assign s2_awaddr  = s_awaddr_r [2];
    assign s2_awlen   = s_awlen_r  [2];
    assign s2_awsize  = s_awsize_r [2];
    assign s2_awburst = s_awburst_r[2];
    assign s2_awvalid = s_awvalid_r[2];

    assign s3_awid    = s_awid_r   [3];
    assign s3_awaddr  = s_awaddr_r [3];
    assign s3_awlen   = s_awlen_r  [3];
    assign s3_awsize  = s_awsize_r [3];
    assign s3_awburst = s_awburst_r[3];
    assign s3_awvalid = s_awvalid_r[3];

    assign s4_awid    = s_awid_r   [4];
    assign s4_awaddr  = s_awaddr_r [4];
    assign s4_awlen   = s_awlen_r  [4];
    assign s4_awsize  = s_awsize_r [4];
    assign s4_awburst = s_awburst_r[4];
    assign s4_awvalid = s_awvalid_r[4];
    assign s5_awid    = s_awid_r   [5];
    assign s5_awaddr  = s_awaddr_r [5];
    assign s5_awlen   = s_awlen_r  [5];
    assign s5_awsize  = s_awsize_r [5];
    assign s5_awburst = s_awburst_r[5];
    assign s5_awvalid = s_awvalid_r[5];

    wire ws0_drive [0:NW-1];
    wire ws1_drive [0:NW-1];
    wire ws2_drive [0:NW-1];
    wire ws3_drive [0:NW-1];
    wire ws4_drive [0:NW-1];
    wire ws5_drive [0:NW-1];
    genvar wi;
    // WS_PAD_W is in the drive set for the same reason WS_FWD_W is: the
    // filler beats are real W traffic to the owned slave, they just come
    // from the crossbar rather than the master (GAP 1).  Adding the state
    // term costs no logic depth — the condition's input set (ws_state,
    // ws_wdone, ws_slv) is unchanged, only the function over it.
    //
    // Early-W forwarding (2026-07-21 dead-cycle removal): W beats are
    // driven to the slave during WS_WAIT_SLV_AW as well as WS_FWD_W.
    // Ownership of the slave's W channel is already exclusive — sw_owner/
    // sw_owned are latched at the same arb edge that enters
    // WS_WAIT_SLV_AW — so no other master's beats can interleave, and
    // AXI4 imposes no AW-before-W ordering (a slave that needs AW first
    // simply holds WREADY low; all six live slaves either do exactly
    // that or FIFO W independently, per the 2026-07-21 audit:
    // ddr_ctrl/vram/axi_wide_to_axilite gate wready on AW-accept,
    // axi_async_bridge accepts into a FIFO).  This removes the one-cycle
    // WS_WAIT_SLV_AW→WS_FWD_W gap on every write.  The !ws_wdone guard
    // stops the drive once the burst's wlast has been accepted so a
    // master's early next-burst WVALID can never leak beats into the
    // still-pending transaction.
    // NOTE: needs a timing-validated synth pass before being trusted on
    // real HW (adds one state term to the W-mux/wready selects) — sim-
    // validated only at time of landing.
    generate
        for (wi = 0; wi < NW; wi = wi + 1) begin : gen_wdrive
            assign ws0_drive[wi] = ((ws_state[wi] == WS_FWD_W) ||
                                    (ws_state[wi] == WS_PAD_W) ||
                                    ((ws_state[wi] == WS_WAIT_SLV_AW) && !ws_wdone[wi]))
                                   && (ws_slv[wi] == `XBAR_SLV_DDR);
            assign ws1_drive[wi] = ((ws_state[wi] == WS_FWD_W) ||
                                    (ws_state[wi] == WS_PAD_W) ||
                                    ((ws_state[wi] == WS_WAIT_SLV_AW) && !ws_wdone[wi]))
                                   && (ws_slv[wi] == `XBAR_SLV_IO);
            assign ws2_drive[wi] = ((ws_state[wi] == WS_FWD_W) ||
                                    (ws_state[wi] == WS_PAD_W) ||
                                    ((ws_state[wi] == WS_WAIT_SLV_AW) && !ws_wdone[wi]))
                                   && (ws_slv[wi] == `XBAR_SLV_DMA);
            assign ws3_drive[wi] = ((ws_state[wi] == WS_FWD_W) ||
                                    (ws_state[wi] == WS_PAD_W) ||
                                    ((ws_state[wi] == WS_WAIT_SLV_AW) && !ws_wdone[wi]))
                                   && (ws_slv[wi] == `XBAR_SLV_VRAM);
            assign ws4_drive[wi] = ((ws_state[wi] == WS_FWD_W) ||
                                    (ws_state[wi] == WS_PAD_W) ||
                                    ((ws_state[wi] == WS_WAIT_SLV_AW) && !ws_wdone[wi]))
                                   && (ws_slv[wi] == `XBAR_SLV_DAFB);
            assign ws5_drive[wi] = ((ws_state[wi] == WS_FWD_W) ||
                                    (ws_state[wi] == WS_PAD_W) ||
                                    ((ws_state[wi] == WS_WAIT_SLV_AW) && !ws_wdone[wi]))
                                   && (ws_slv[wi] == `XBAR_SLV_SDJTAG);
        end
    endgenerate

    wire [SLOT_W-1:0] s0_wowner = ws0_drive[0] ? 2'd0 :
                                  ws0_drive[1] ? 2'd1 :
                                  ws0_drive[2] ? 2'd2 : 2'd3;
    wire              s0_wowner_vld = ws0_drive[0] | ws0_drive[1] | ws0_drive[2] | ws0_drive[3];
    wire [SLOT_W-1:0] s1_wowner = ws1_drive[0] ? 2'd0 :
                                  ws1_drive[1] ? 2'd1 :
                                  ws1_drive[2] ? 2'd2 : 2'd3;
    wire              s1_wowner_vld = ws1_drive[0] | ws1_drive[1] | ws1_drive[2] | ws1_drive[3];
    wire [SLOT_W-1:0] s2_wowner = ws2_drive[0] ? 2'd0 :
                                  ws2_drive[1] ? 2'd1 :
                                  ws2_drive[2] ? 2'd2 : 2'd3;
    wire              s2_wowner_vld = ws2_drive[0] | ws2_drive[1] | ws2_drive[2] | ws2_drive[3];
    wire [SLOT_W-1:0] s3_wowner = ws3_drive[0] ? 2'd0 :
                                  ws3_drive[1] ? 2'd1 :
                                  ws3_drive[2] ? 2'd2 : 2'd3;
    wire              s3_wowner_vld = ws3_drive[0] | ws3_drive[1] | ws3_drive[2] | ws3_drive[3];
    wire [SLOT_W-1:0] s4_wowner = ws4_drive[0] ? 2'd0 :
                                  ws4_drive[1] ? 2'd1 :
                                  ws4_drive[2] ? 2'd2 : 2'd3;
    wire              s4_wowner_vld = ws4_drive[0] | ws4_drive[1] | ws4_drive[2] | ws4_drive[3];
    wire [SLOT_W-1:0] s5_wowner = ws5_drive[0] ? 2'd0 :
                                  ws5_drive[1] ? 2'd1 :
                                  ws5_drive[2] ? 2'd2 : 2'd3;
    wire              s5_wowner_vld = ws5_drive[0] | ws5_drive[1] | ws5_drive[2] | ws5_drive[3];

    assign s0_wdata  = (s0_wowner == 2'd0) ? mw_wdata [0] :
                       (s0_wowner == 2'd1) ? mw_wdata [1] :
                       (s0_wowner == 2'd2) ? mw_wdata [2] : mw_wdata [3];
    assign s0_wstrb  = (s0_wowner == 2'd0) ? mw_wstrb [0] :
                       (s0_wowner == 2'd1) ? mw_wstrb [1] :
                       (s0_wowner == 2'd2) ? mw_wstrb [2] : mw_wstrb [3];
    assign s0_wlast  = (s0_wowner == 2'd0) ? mw_wlast [0] :
                       (s0_wowner == 2'd1) ? mw_wlast [1] :
                       (s0_wowner == 2'd2) ? mw_wlast [2] : mw_wlast [3];
    assign s0_wvalid = s0_wowner_vld &
                       ((s0_wowner == 2'd0) ? mw_wvalid[0] :
                        (s0_wowner == 2'd1) ? mw_wvalid[1] :
                        (s0_wowner == 2'd2) ? mw_wvalid[2] : mw_wvalid[3]);

    assign s1_wdata  = (s1_wowner == 2'd0) ? mw_wdata [0] :
                       (s1_wowner == 2'd1) ? mw_wdata [1] :
                       (s1_wowner == 2'd2) ? mw_wdata [2] : mw_wdata [3];
    assign s1_wstrb  = (s1_wowner == 2'd0) ? mw_wstrb [0] :
                       (s1_wowner == 2'd1) ? mw_wstrb [1] :
                       (s1_wowner == 2'd2) ? mw_wstrb [2] : mw_wstrb [3];
    assign s1_wlast  = (s1_wowner == 2'd0) ? mw_wlast [0] :
                       (s1_wowner == 2'd1) ? mw_wlast [1] :
                       (s1_wowner == 2'd2) ? mw_wlast [2] : mw_wlast [3];
    assign s1_wvalid = s1_wowner_vld &
                       ((s1_wowner == 2'd0) ? mw_wvalid[0] :
                        (s1_wowner == 2'd1) ? mw_wvalid[1] :
                        (s1_wowner == 2'd2) ? mw_wvalid[2] : mw_wvalid[3]);

    assign s2_wdata  = (s2_wowner == 2'd0) ? mw_wdata [0] :
                       (s2_wowner == 2'd1) ? mw_wdata [1] :
                       (s2_wowner == 2'd2) ? mw_wdata [2] : mw_wdata [3];
    assign s2_wstrb  = (s2_wowner == 2'd0) ? mw_wstrb [0] :
                       (s2_wowner == 2'd1) ? mw_wstrb [1] :
                       (s2_wowner == 2'd2) ? mw_wstrb [2] : mw_wstrb [3];
    assign s2_wlast  = (s2_wowner == 2'd0) ? mw_wlast [0] :
                       (s2_wowner == 2'd1) ? mw_wlast [1] :
                       (s2_wowner == 2'd2) ? mw_wlast [2] : mw_wlast [3];
    assign s2_wvalid = s2_wowner_vld &
                       ((s2_wowner == 2'd0) ? mw_wvalid[0] :
                        (s2_wowner == 2'd1) ? mw_wvalid[1] :
                        (s2_wowner == 2'd2) ? mw_wvalid[2] : mw_wvalid[3]);

    wire [DATA_WIDTH-1:0] s3_wdata_bus =
        (s3_wowner == 2'd0) ? mw_wdata [0] :
        (s3_wowner == 2'd1) ? mw_wdata [1] :
        (s3_wowner == 2'd2) ? mw_wdata [2] : mw_wdata [3];
    wire [STRB_WIDTH-1:0] s3_wstrb_bus =
        (s3_wowner == 2'd0) ? mw_wstrb [0] :
        (s3_wowner == 2'd1) ? mw_wstrb [1] :
        (s3_wowner == 2'd2) ? mw_wstrb [2] : mw_wstrb [3];
    assign s3_wdata  = s3_flush_active ? {DATA_WIDTH{1'b0}} :
                       vram_swap_words(s3_wdata_bus);
    assign s3_wstrb  = s3_flush_active ? {STRB_WIDTH{1'b0}} :
                       vram_swap_strb(s3_wstrb_bus);
    assign s3_wlast  = s3_flush_active ? (s3_flush_wbeats == 9'd1) :
                       (s3_wowner == 2'd0) ? mw_wlast [0] :
                       (s3_wowner == 2'd1) ? mw_wlast [1] :
                       (s3_wowner == 2'd2) ? mw_wlast [2] : mw_wlast [3];
    assign s3_wvalid = s3_flush_active ? (s3_flush_wbeats != 9'd0) :
                       s3_wowner_vld &
                       ((s3_wowner == 2'd0) ? mw_wvalid[0] :
                        (s3_wowner == 2'd1) ? mw_wvalid[1] :
                        (s3_wowner == 2'd2) ? mw_wvalid[2] : mw_wvalid[3]);

    assign s4_wdata  = (s4_wowner == 2'd0) ? mw_wdata [0] :
                       (s4_wowner == 2'd1) ? mw_wdata [1] :
                       (s4_wowner == 2'd2) ? mw_wdata [2] : mw_wdata [3];
    assign s4_wstrb  = (s4_wowner == 2'd0) ? mw_wstrb [0] :
                       (s4_wowner == 2'd1) ? mw_wstrb [1] :
                       (s4_wowner == 2'd2) ? mw_wstrb [2] : mw_wstrb [3];
    assign s4_wlast  = (s4_wowner == 2'd0) ? mw_wlast [0] :
                       (s4_wowner == 2'd1) ? mw_wlast [1] :
                       (s4_wowner == 2'd2) ? mw_wlast [2] : mw_wlast [3];
    assign s4_wvalid = s4_wowner_vld &
                       ((s4_wowner == 2'd0) ? mw_wvalid[0] :
                        (s4_wowner == 2'd1) ? mw_wvalid[1] :
                        (s4_wowner == 2'd2) ? mw_wvalid[2] : mw_wvalid[3]);

    assign s5_wdata  = (s5_wowner == 2'd0) ? mw_wdata [0] :
                       (s5_wowner == 2'd1) ? mw_wdata [1] :
                       (s5_wowner == 2'd2) ? mw_wdata [2] : mw_wdata [3];
    assign s5_wstrb  = (s5_wowner == 2'd0) ? mw_wstrb [0] :
                       (s5_wowner == 2'd1) ? mw_wstrb [1] :
                       (s5_wowner == 2'd2) ? mw_wstrb [2] : mw_wstrb [3];
    assign s5_wlast  = (s5_wowner == 2'd0) ? mw_wlast [0] :
                       (s5_wowner == 2'd1) ? mw_wlast [1] :
                       (s5_wowner == 2'd2) ? mw_wlast [2] : mw_wlast [3];
    assign s5_wvalid = s5_wowner_vld &
                       ((s5_wowner == 2'd0) ? mw_wvalid[0] :
                        (s5_wowner == 2'd1) ? mw_wvalid[1] :
                        (s5_wowner == 2'd2) ? mw_wvalid[2] : mw_wvalid[3]);

    wire mw_wready_raw [0:NW-1];
    genvar mi;
    generate
        for (mi = 0; mi < NW; mi = mi + 1) begin : gen_wready
            // Early-W: wready is forwarded during WS_WAIT_SLV_AW too
            // (see the ws*_drive comment above); !ws_wdone stops
            // accepting once the burst's wlast has gone through.
            // WS_PAD_W: the slot must still SEE the slave's WREADY so the
            // filler-beat handshake registers (m0_wready/m0b_wready are
            // separately gated off — see the slot-0 fan-in mux).
            assign mw_wready_raw[mi] =
                ((ws_state[mi] == WS_FWD_W) ||
                 (ws_state[mi] == WS_PAD_W) ||
                 ((ws_state[mi] == WS_WAIT_SLV_AW) && !ws_wdone[mi]))
                                             ? ((ws_slv[mi] == `XBAR_SLV_DDR) ? s0_wready :
                                                (ws_slv[mi] == `XBAR_SLV_IO ) ? s1_wready :
                                                (ws_slv[mi] == `XBAR_SLV_DMA) ? s2_wready :
                                                (ws_slv[mi] == `XBAR_SLV_VRAM) ? s3_wready :
                                                (ws_slv[mi] == `XBAR_SLV_DAFB) ? s4_wready :
                                                (ws_slv[mi] == `XBAR_SLV_SDJTAG) ? s5_wready : 1'b0) :
                (ws_state[mi] == WS_DRAIN_W) ? 1'b1 : 1'b0;
        end
    endgenerate
    assign mw_wready[0] = mw_wready_raw[0];
    assign mw_wready[1] = mw_wready_raw[1];
    assign mw_wready[2] = mw_wready_raw[2];
    assign mw_wready[3] = mw_wready_raw[3];

    // Task #219: per-slave "retire the dangling B this cycle" pulse.
    // Declared here, DRIVEN below the gen_ws_wd generate (which is where
    // ws_b_abandon[] is computed).  ws_b_abandon[mi] already implies
    // slot mi is in WS_WAIT_B and owns the slave named by ws_slv[mi]
    // (mw_bvalid_slv[] carries the sw_owned/sw_owner match), so decoding
    // on ws_slv[mi] alone is sufficient and cannot steal a BREADY meant
    // for another slave.  Forced for exactly the one cycle ws_b_abandon
    // is high, in which the write FSM also releases the lock and idles
    // the slot — so the slave's B is consumed, never left dangling to
    // alias onto whichever master owns that slave next.
    wire b_abandon_s0, b_abandon_s1, b_abandon_s2;
    wire b_abandon_s3, b_abandon_s4, b_abandon_s5;

    assign s0_bready = b_abandon_s0 |
                       sw_owned[0] &
                       ((sw_owner[0] == 2'd0) ? mw_bready[0] :
                        (sw_owner[0] == 2'd1) ? mw_bready[1] :
                        (sw_owner[0] == 2'd2) ? mw_bready[2] : mw_bready[3]) &
                       (ws_state[sw_owner[0]] == WS_WAIT_B);
    assign s1_bready = b_abandon_s1 |
                       sw_owned[1] &
                       ((sw_owner[1] == 2'd0) ? mw_bready[0] :
                        (sw_owner[1] == 2'd1) ? mw_bready[1] :
                        (sw_owner[1] == 2'd2) ? mw_bready[2] : mw_bready[3]) &
                       (ws_state[sw_owner[1]] == WS_WAIT_B);
    assign s2_bready = b_abandon_s2 |
                       sw_owned[2] &
                       ((sw_owner[2] == 2'd0) ? mw_bready[0] :
                        (sw_owner[2] == 2'd1) ? mw_bready[1] :
                        (sw_owner[2] == 2'd2) ? mw_bready[2] : mw_bready[3]) &
                       (ws_state[sw_owner[2]] == WS_WAIT_B);
    assign s3_bready = (s3_flush_active && (s3_flush_wbeats == 9'd0)) |
                       b_abandon_s3 |
                       sw_owned[3] &
                       ((sw_owner[3] == 2'd0) ? mw_bready[0] :
                        (sw_owner[3] == 2'd1) ? mw_bready[1] :
                        (sw_owner[3] == 2'd2) ? mw_bready[2] : mw_bready[3]) &
                       (ws_state[sw_owner[3]] == WS_WAIT_B);
    assign s4_bready = b_abandon_s4 |
                       sw_owned[4] &
                       ((sw_owner[4] == 2'd0) ? mw_bready[0] :
                        (sw_owner[4] == 2'd1) ? mw_bready[1] :
                        (sw_owner[4] == 2'd2) ? mw_bready[2] : mw_bready[3]) &
                       (ws_state[sw_owner[4]] == WS_WAIT_B);
    assign s5_bready = b_abandon_s5 |
                       sw_owned[5] &
                       ((sw_owner[5] == 2'd0) ? mw_bready[0] :
                        (sw_owner[5] == 2'd1) ? mw_bready[1] :
                        (sw_owner[5] == 2'd2) ? mw_bready[2] : mw_bready[3]) &
                       (ws_state[sw_owner[5]] == WS_WAIT_B);

    wire                 mw_bvalid_loc [0:NW-1];
    wire [1:0]           mw_bresp_loc  [0:NW-1];
    wire [ID_WIDTH-1:0]  mw_bid_loc    [0:NW-1];
    wire                 mw_bvalid_slv [0:NW-1];
    wire [1:0]           mw_bresp_slv  [0:NW-1];
    wire [ID_WIDTH-1:0]  mw_bid_slv    [0:NW-1];
    generate
        for (mi = 0; mi < NW; mi = mi + 1) begin : gen_bmux
            assign mw_bvalid_loc[mi] = (ws_state[mi] == WS_SEND_BLOCAL);
            assign mw_bresp_loc [mi] = ws_local_rsp[mi];
            assign mw_bid_loc   [mi] = ws_mid[mi];
            assign mw_bvalid_slv[mi] =
                (ws_state[mi] == WS_WAIT_B) &&
                (((ws_slv[mi] == `XBAR_SLV_DDR ) && sw_owned[0] && (sw_owner[0] == mi[SLOT_W-1:0]) && s0_bvalid) ||
                 ((ws_slv[mi] == `XBAR_SLV_IO  ) && sw_owned[1] && (sw_owner[1] == mi[SLOT_W-1:0]) && s1_bvalid) ||
                 ((ws_slv[mi] == `XBAR_SLV_DMA ) && sw_owned[2] && (sw_owner[2] == mi[SLOT_W-1:0]) && s2_bvalid) ||
                 ((ws_slv[mi] == `XBAR_SLV_VRAM) && sw_owned[3] && (sw_owner[3] == mi[SLOT_W-1:0]) && s3_bvalid) ||
                 ((ws_slv[mi] == `XBAR_SLV_DAFB) && sw_owned[4] && (sw_owner[4] == mi[SLOT_W-1:0]) && s4_bvalid) ||
                 ((ws_slv[mi] == `XBAR_SLV_SDJTAG) && sw_owned[5] && (sw_owner[5] == mi[SLOT_W-1:0]) && s5_bvalid));
            assign mw_bresp_slv [mi] = (ws_slv[mi] == `XBAR_SLV_DDR ) ? s0_bresp :
                                        (ws_slv[mi] == `XBAR_SLV_IO  ) ? s1_bresp :
                                        (ws_slv[mi] == `XBAR_SLV_DMA ) ? s2_bresp :
                                        (ws_slv[mi] == `XBAR_SLV_VRAM) ? s3_bresp :
                                        (ws_slv[mi] == `XBAR_SLV_DAFB) ? s4_bresp : s5_bresp;
            assign mw_bid_slv   [mi] = ws_mid[mi];
        end
    endgenerate
    assign mw_bvalid[0] = mw_bvalid_loc[0] | mw_bvalid_slv[0];
    assign mw_bvalid[1] = mw_bvalid_loc[1] | mw_bvalid_slv[1];
    assign mw_bvalid[2] = mw_bvalid_loc[2] | mw_bvalid_slv[2];
    assign mw_bvalid[3] = mw_bvalid_loc[3] | mw_bvalid_slv[3];
    assign mw_bresp [0] = mw_bvalid_loc[0] ? mw_bresp_loc[0] : mw_bresp_slv[0];
    assign mw_bresp [1] = mw_bvalid_loc[1] ? mw_bresp_loc[1] : mw_bresp_slv[1];
    assign mw_bresp [2] = mw_bvalid_loc[2] ? mw_bresp_loc[2] : mw_bresp_slv[2];
    assign mw_bresp [3] = mw_bvalid_loc[3] ? mw_bresp_loc[3] : mw_bresp_slv[3];
    assign mw_bid   [0] = mw_bid_loc  [0];
    assign mw_bid   [1] = mw_bid_loc  [1];
    assign mw_bid   [2] = mw_bid_loc  [2];
    assign mw_bid   [3] = mw_bid_loc  [3];

    reg  mw_awready_r [0:NW-1];
    assign mw_awready[0] = mw_awready_r[0];
    assign mw_awready[1] = mw_awready_r[1];
    assign mw_awready[2] = mw_awready_r[2];
    assign mw_awready[3] = mw_awready_r[3];

    integer i;

    function [SLOT_W-1:0] rr_pick_w;
        input [SLOT_W-1:0] rr_base;
        input req0, req1, req2, req3;
        reg [SLOT_W-1:0] c [0:3];
        reg r [0:3];
        integer k;
        reg done;
        begin
            c[0] = rr_base;
            c[1] = (rr_base + 2'd1);
            c[2] = (rr_base + 2'd2);
            c[3] = (rr_base + 2'd3);
            rr_pick_w = rr_base;
            done = 1'b0;
            for (k = 0; k < 4; k = k + 1) begin
                r[k] = (c[k] == 2'd0 && req0) || (c[k] == 2'd1 && req1) ||
                       (c[k] == 2'd2 && req2) || (c[k] == 2'd3 && req3);
                if (r[k] && !done) begin
                    rr_pick_w = c[k];
                    done = 1'b1;
                end
            end
        end
    endfunction

    wire [NW-1:0] req_ddr;
    wire [NW-1:0] req_io;
    wire [NW-1:0] req_dma;
    wire [NW-1:0] req_vram;
    wire [NW-1:0] req_dafb;
    wire [NW-1:0] req_sd_jtag;
    wire [NW-1:0] req_local;
    wire [SSEL_W-1:0] dec_slv_w [0:NW-1];
    wire          is_rom_w   [0:NW-1];
    wire          is_rom_loader_w [0:NW-1];
    // Fix 1: awlen>0 decoded onto a single-beat-only slave -> local
    // SLVERR instead of forwarding (see module-header note).  Review
    // MINOR 6: this is now the ONE place the "lite-only-slave burst"
    // predicate is computed; every req_* gate below (including DDR/VRAM,
    // where it is structurally always false) derives its "block
    // forwarding" condition from this wire instead of re-deriving its
    // own (awlen==0) check, so a slave added to is_lite_only_slv() in
    // the future without updating every req_* call site can't produce a
    // simultaneous grant + local-reject double-accept on the same slot.
    wire          is_burst_reject_w [0:NW-1];
    // CRITICAL fix (slv_flush, part iii): a Lite-only-slave decode while
    // the bridge behind it is in reset answers local SLVERR immediately,
    // same shape as a burst-reject, but non-sticky — no poison latch
    // (see local_rsp_is_err_w / req_local below and the slv_flush port
    // comment for the full contract).
    wire          is_flush_reject_w [0:NW-1];
    // Fix 2: decode landed on an already-poisoned slave -> local SLVERR
    // regardless of burst length (DDR/VRAM included).
    wire          is_poisoned_dec_w [0:NW-1];
    wire          local_rsp_is_err_w[0:NW-1];

    generate
        for (mi = 0; mi < NW; mi = mi + 1) begin : gen_wdec
            assign dec_slv_w[mi]  = decode_slv(mw_awaddr_eff[mi]);
            assign is_rom_w [mi]  = is_rom_write(mw_awaddr_eff[mi]);
            assign is_rom_loader_w[mi] =
                (mw_midx[mi] == `XBAR_M_BOOT) ||
                (mw_midx[mi] == `XBAR_M_XDMA);
            assign is_burst_reject_w[mi] =
                is_lite_only_slv(dec_slv_w[mi]) && (mw_awlen[mi] != 8'd0);
            // Flush-reject uses is_flush_domain_slv(), NOT
            // is_lite_only_slv().  The two sets still differ: S3 is in
            // the flush domain though it is not lite-only, and S0 is
            // neither.  (S1 used to be the other difference -- lite-only
            // but NOT flush-domain -- until 2026-09-12; see that
            // function's comment for what changed and why.)
            assign is_flush_reject_w[mi] =
                slv_flush && is_flush_domain_slv(dec_slv_w[mi]);
            assign is_poisoned_dec_w[mi] =
                ((dec_slv_w[mi] == `XBAR_SLV_DDR)    && slv_poisoned[0]) ||
                ((dec_slv_w[mi] == `XBAR_SLV_IO)     && slv_poisoned[1]) ||
                ((dec_slv_w[mi] == `XBAR_SLV_DMA)    && slv_poisoned[2]) ||
                ((dec_slv_w[mi] == `XBAR_SLV_VRAM)   && slv_poisoned[3]) ||
                ((dec_slv_w[mi] == `XBAR_SLV_DAFB)   && slv_poisoned[4]) ||
                ((dec_slv_w[mi] == `XBAR_SLV_SDJTAG) && slv_poisoned[5]);
            assign local_rsp_is_err_w[mi] =
                is_burst_reject_w[mi] || is_poisoned_dec_w[mi] || is_flush_reject_w[mi];

            assign req_ddr  [mi] = (ws_state[mi] == WS_IDLE) && mw_awvalid[mi] &&
                                   (dec_slv_w[mi] == `XBAR_SLV_DDR) &&
                                   (!is_rom_w[mi] || is_rom_loader_w[mi]) &&
                                   !is_burst_reject_w[mi] && !is_flush_reject_w[mi] &&
                                   !slv_poisoned[0];
            assign req_io   [mi] = (ws_state[mi] == WS_IDLE) && mw_awvalid[mi] &&
                                   (dec_slv_w[mi] == `XBAR_SLV_IO) &&
                                   !is_burst_reject_w[mi] && !is_flush_reject_w[mi] &&
                                   !s1_rst_tail_busy &&   // backlog item 3
                                   !slv_poisoned[1];
            assign req_dma  [mi] = (ws_state[mi] == WS_IDLE) && mw_awvalid[mi] &&
                                   (dec_slv_w[mi] == `XBAR_SLV_DMA) &&
                                   !is_burst_reject_w[mi] && !is_flush_reject_w[mi] &&
                                   !slv_poisoned[2];
            assign req_vram [mi] = (ws_state[mi] == WS_IDLE) && mw_awvalid[mi] &&
                                   (dec_slv_w[mi] == `XBAR_SLV_VRAM) &&
                                   !is_burst_reject_w[mi] && !is_flush_reject_w[mi] &&
                                   !slv_poisoned[3];
            assign req_dafb [mi] = (ws_state[mi] == WS_IDLE) && mw_awvalid[mi] &&
                                   (dec_slv_w[mi] == `XBAR_SLV_DAFB) &&
                                   !is_burst_reject_w[mi] && !is_flush_reject_w[mi] &&
                                   !slv_poisoned[4];
            assign req_sd_jtag[mi] = (ws_state[mi] == WS_IDLE) && mw_awvalid[mi] &&
                                     (dec_slv_w[mi] == `XBAR_SLV_SDJTAG) &&
                                     !is_burst_reject_w[mi] && !is_flush_reject_w[mi] &&
                                     !slv_poisoned[5];
            assign req_local[mi] = (ws_state[mi] == WS_IDLE) && mw_awvalid[mi] &&
                                   ((dec_slv_w[mi] == `XBAR_SLV_NONE) ||
                                    (is_rom_w[mi] && !is_rom_loader_w[mi]) ||
                                    is_burst_reject_w[mi] ||
                                    is_poisoned_dec_w[mi] ||
                                    is_flush_reject_w[mi]);
        end
    endgenerate

    // Fix 2 watchdog activity/fire wires — a write slot is "active" (the
    // watchdog counts) while it has an outstanding transaction actually
    // engaged with a real slave (AW pending, W forwarding, or waiting on
    // B).  WS_DRAIN_W/WS_SEND_BLOCAL are already local-only completions
    // and are not watched (a stalled master, not slave, is out of scope
    // here — see the WD_MAX_CNT policy comment above).
    //
    // ws_wd_fire is additionally suppressed:
    //   - while slv_flush is asserted (CRITICAL fix, part i: counters
    //     freeze — the flush-abort path below handles this slot
    //     instead, without poisoning);
    //   - MINOR 4: while the real slave's B is CURRENTLY asserted
    //     (mw_bvalid_slv[mi] high, independent of mw_bready[mi]) — the
    //     slave answered right at the deadline; AXI4 requires BRESP
    //     stay stable once BVALID is asserted, so the watchdog must not
    //     switch the B source out from under an in-flight response.
    //     The real B then completes normally via the WS_WAIT_B
    //     "if (mw_bvalid_slv[i] && mw_bready[i])" branch, whenever
    //     BREADY eventually samples it.
    //
    //     This suppression is CORRECT and is deliberately unchanged: the
    //     ordinary watchdog POISONS, and poisoning a slave that has
    //     already answered would be worse than the hang.  What was
    //     missing (task #219) is that nothing else covered the case
    //     where BREADY never comes back at all — see ws_b_abandon.
    wire ws_active [0:NW-1];
    wire ws_wd_fire[0:NW-1];
    // ws_b_abandon (task #219): the slave produced a B, but the owning
    // master will not take it and, on the evidence of ws_bhold_cnt, is
    // never going to — a MASTER abandonment, not a slave fault.  Until
    // this existed there was NO escape: MINOR 4 above (correctly) keeps
    // the poisoning watchdog off, so sw_owned[slv] stayed pinned at 1
    // forever and the slave was dead to every master until the next rst.
    // Handled by retiring the dangling B at the slave port (s*_bready is
    // forced for that one cycle below) and returning the slot to WS_IDLE
    // with the lock released: no poison (the slave is healthy) and no
    // locally synthesized B (the master is gone; a stale local B would
    // only be mis-attributed to its next transaction — same policy as
    // the m0_abandon discard path in ws_release_slot).
    wire ws_b_abandon [0:NW-1];
    // Race audit 2026-09-18: the WS_DRAIN_W / WS_SEND_BLOCAL escapes.
    // See the ws_lhold_cnt declaration above for the full argument.
    wire ws_lhold_run   [0:NW-1];
    wire ws_w_abandon   [0:NW-1];
    wire ws_bloc_abandon[0:NW-1];
    // Fix 2 flush-abort trigger (part ii): a write slot actively engaged
    // with a flush-DOMAIN slave (is_flush_domain_slv(), NOT
    // is_lite_only_slv() — the sets differ, see that function's comment)
    // while slv_flush is asserted aborts through the SAME
    // drain/local-response machinery as a watchdog timeout, but never
    // sets the sticky poison latch.
    wire ws_flush_abort[0:NW-1];
    // GAP 2: whether an abort out of WS_WAIT_SLV_AW still owes W beats.
    //
    // `need_drain` used to be an unconditional 1'b1 at all six
    // WS_WAIT_SLV_AW abort sites.  But the W burst may ALREADY be fully
    // accepted by the slave in that state — ws_wdone, or the WLAST
    // handshake this very cycle — which is exactly what a slave that FIFOs
    // W ahead of AW produces, and axi_async_bridge (behind S1) does that.
    // Aborting such a slot with need_drain=1 parks it in WS_DRAIN_W waiting
    // for beats that will never arrive; WS_DRAIN_W is excluded from
    // ws_active, so no watchdog rescues it and the MASTER hangs instead of
    // getting its SLVERR.  Pre-existing, unrelated to bursts.
    //
    // This is the negation of the predicate the AW-accept transition
    // already uses to decide whether to skip WS_FWD_W (see WS_WAIT_SLV_AW
    // below): "no beats left to move" is the same question in both places,
    // so it must be the same expression.  Note it deliberately ORs in the
    // SAME-CYCLE wlast handshake, because the registered ws_wdone read in
    // this cycle still shows the old value.
    wire ws_needs_drain[0:NW-1];
    wire [8:0] ws_wbeats_next[0:NW-1];
    generate
        for (mi = 0; mi < NW; mi = mi + 1) begin : gen_ws_wd
            // WS_PAD_W is watched too: if a slave will not even take
            // wstrb=0 filler, it is genuinely wedged and must still get the
            // existing loud treatment (release + poison) rather than hang
            // the crossbar in the pad state forever.
            assign ws_active[mi] = (ws_state[mi] == WS_WAIT_SLV_AW) ||
                                    (ws_state[mi] == WS_FWD_W) ||
                                    (ws_state[mi] == WS_PAD_W) ||
                                    (ws_state[mi] == WS_WAIT_B);
            assign ws_needs_drain[mi] =
                !(ws_wdone[mi] ||
                  (mw_wvalid[mi] && mw_wready_raw[mi] && mw_wlast[mi]));
            // Post-update owed-beat count, for the same reason
            // axi_bridge_w_pad.v snapshots occ_next rather than the stale
            // pre-cycle value: a beat accepted in the very cycle the
            // abandonment is noticed must be reflected, or the pad would
            // emit one beat too many.
            assign ws_wbeats_next[mi] =
                (mw_wvalid[mi] && mw_wready_raw[mi] && (ws_wbeats[mi] != 9'd0))
                    ? (ws_wbeats[mi] - 9'd1) : ws_wbeats[mi];
            // NOTE: the MINOR-4 `!mw_bvalid_slv[mi]` suppression below is
            // deliberately left exactly as it was.  It is the reason the
            // ordinary watchdog can never fire in WS_WAIT_B once the
            // slave has answered — which is correct, because firing there
            // would POISON a healthy slave.  The escape from a master
            // that never takes that B is ws_b_abandon, a separate and
            // separately-bounded path, NOT a relaxation of this one.
            assign ws_wd_fire[mi] = ENABLE_WD && ws_active[mi] &&
                                     (ws_wd_cnt[mi] == ((ws_slv[mi] == `XBAR_SLV_IO) ? WD_MAX_CNT_S1
                                                                                     : WD_MAX_CNT)) &&
                                     !slv_flush && !mw_bvalid_slv[mi];
            // Deliberately gated on the FULL watchdog as well as the
            // B-hold grace: stealing a B from a master that is merely
            // slow would be worse than the park this rescues, so the
            // rescue only runs once the transaction has ALSO blown its
            // ordinary end-to-end deadline.  Recovery latency is
            // therefore bounded by 2^WD_LOG2(_S1), not by B_HOLD_LOG2.
            //
            // TIMING: every term here is a REGISTER (ws_state,
            // ws_bhold_cnt, ws_wd_cnt, ws_slv) except the slv_flush
            // input — deliberately.  This wire feeds s*_bready, and
            // writing it as `ws_wd_fire[mi] && mw_bvalid_slv[mi]` (which
            // is logically identical, since ws_bhold_cnt can only reach
            // B_HOLD_MAX after 2^B_HOLD_LOG2 cycles of mw_bvalid_slv
            // being high) would create a new combinational
            // s*_bvalid -> s*_bready path at the slave port.  That is
            // AXI-legal but is a needless critical-path addition on a
            // module that already closes tightly.
            assign ws_b_abandon[mi] = (ws_state[mi] == WS_WAIT_B) &&
                                       (ws_bhold_cnt[mi] == B_HOLD_MAX) &&
                                       (ws_wd_cnt[mi] == ((ws_slv[mi] == `XBAR_SLV_IO) ? WD_MAX_CNT_S1
                                                                                       : WD_MAX_CNT)) &&
                                       !slv_flush;
            // Race audit 2026-09-18 -- the two local-completion escapes.
            // `ws_lhold_run` is the enable for the shared counter: it runs
            // only while THIS crossbar is waiting on the MASTER in a state
            // no watchdog watches.
            //
            // TIMING: both predicates are all-register except slv_flush,
            // exactly like rs_r_abandon.  ws_bloc_abandon reaches
            // mw_bvalid_loc only through ws_state (a register), so no
            // combinational mw_bready -> mw_bvalid path is created; WREADY
            // in WS_DRAIN_W is a constant, so nothing is created there
            // either.
            assign ws_lhold_run[mi] =
                (((ws_state[mi] == WS_DRAIN_W)     && !mw_wvalid[mi]) ||
                 ((ws_state[mi] == WS_SEND_BLOCAL) && !mw_bready[mi]));
            // The master owes beats it is never going to send.  Hand off to
            // the local B rather than idling: nothing has been OFFERED to
            // this master yet, and this module's own abandonment audit is
            // that no transaction is dropped without a response (the same
            // call RS_DRAIN_R's normal exit makes, for the same reason -- a
            // dropped write response strands whatever is counting it).  If
            // the master really is gone it will not take the B either, and
            // ws_bloc_abandon below retires it: the two compose.
            assign ws_w_abandon[mi] = (ws_state[mi] == WS_DRAIN_W) &&
                                      (ws_lhold_cnt[mi] == B_HOLD_MAX) &&
                                      !slv_flush;
            // The master HAS been offered its B and has refused it for the
            // whole window, so offering it again is pointless: idle the
            // slot, withdrawing the unaccepted BVALID.  Same deliberate
            // protocol deviation, and the same "no poison -- no slave was
            // ever involved in a local response" policy, as rs_r_abandon.
            assign ws_bloc_abandon[mi] = (ws_state[mi] == WS_SEND_BLOCAL) &&
                                         (ws_lhold_cnt[mi] == B_HOLD_MAX) &&
                                         !slv_flush;
            // S1 owner-gone abort (2026-09-12).  S1 is deliberately OUTSIDE the flush
            // domain, so the line above can never fire for it -- which left ws_wd_fire
            // as S1's only release path.  Measured on hardware: after a JTAG `reset`,
            // xbar_s1_slot_busy = 1 (0 while running) with xbar_slv_poisoned = 0x00 --
            // the slot-0 S1 write is stranded because the reset swallowed its BRESP and
            // nothing can complete it.  The JTAG `reset` IS a dbg_wr to DBG_CONTROL
            // through this very window, so it strands its own write, every time.
            //
            // Release it the way the module already releases an owner-less slot-0 write:
            // via ws_release_slot with poison=0, whose slot-0 branch discards SILENTLY
            // (no drain, no local B) precisely because the owner cannot consume one.
            // The predicate is ws_release_slot's own discard predicate, so a slot that
            // aborts here always takes that branch.
            //
            // NARROW BY CONSTRUCTION -- slot 0 AND S1 only.  Slot-0 writes to S0 (DDR)
            // must NOT come through here: a DDR slave parked mid-burst needs the
            // WS_PAD_W wstrb=0 filler + WLAST path (see the "Abandoned-burst W padding"
            // header note), and discarding instead would let the next write's beats be
            // consumed as this burst's continuation and commit to the OLD address.
            assign ws_flush_abort[mi] = ws_active[mi] &&
                                        ((slv_flush && is_flush_domain_slv(ws_slv[mi])) ||
                                         ((mi == 0) && (ws_slv[mi] == `XBAR_SLV_IO) &&
                                          (m0_abandon || (m0_wsel_q != cpu_held_in_reset))));
        end
    endgenerate

    // Task #219: drive the forward-declared per-slave forced-BREADY
    // pulses (see their declaration above the s*_bready assigns).
    assign b_abandon_s0 = (ws_b_abandon[0] && (ws_slv[0] == `XBAR_SLV_DDR)) ||
                          (ws_b_abandon[1] && (ws_slv[1] == `XBAR_SLV_DDR)) ||
                          (ws_b_abandon[2] && (ws_slv[2] == `XBAR_SLV_DDR)) ||
                          (ws_b_abandon[3] && (ws_slv[3] == `XBAR_SLV_DDR));
    assign b_abandon_s1 = (ws_b_abandon[0] && (ws_slv[0] == `XBAR_SLV_IO)) ||
                          (ws_b_abandon[1] && (ws_slv[1] == `XBAR_SLV_IO)) ||
                          (ws_b_abandon[2] && (ws_slv[2] == `XBAR_SLV_IO)) ||
                          (ws_b_abandon[3] && (ws_slv[3] == `XBAR_SLV_IO));
    assign b_abandon_s2 = (ws_b_abandon[0] && (ws_slv[0] == `XBAR_SLV_DMA)) ||
                          (ws_b_abandon[1] && (ws_slv[1] == `XBAR_SLV_DMA)) ||
                          (ws_b_abandon[2] && (ws_slv[2] == `XBAR_SLV_DMA)) ||
                          (ws_b_abandon[3] && (ws_slv[3] == `XBAR_SLV_DMA));
    assign b_abandon_s3 = (ws_b_abandon[0] && (ws_slv[0] == `XBAR_SLV_VRAM)) ||
                          (ws_b_abandon[1] && (ws_slv[1] == `XBAR_SLV_VRAM)) ||
                          (ws_b_abandon[2] && (ws_slv[2] == `XBAR_SLV_VRAM)) ||
                          (ws_b_abandon[3] && (ws_slv[3] == `XBAR_SLV_VRAM));
    assign b_abandon_s4 = (ws_b_abandon[0] && (ws_slv[0] == `XBAR_SLV_DAFB)) ||
                          (ws_b_abandon[1] && (ws_slv[1] == `XBAR_SLV_DAFB)) ||
                          (ws_b_abandon[2] && (ws_slv[2] == `XBAR_SLV_DAFB)) ||
                          (ws_b_abandon[3] && (ws_slv[3] == `XBAR_SLV_DAFB));
    assign b_abandon_s5 = (ws_b_abandon[0] && (ws_slv[0] == `XBAR_SLV_SDJTAG)) ||
                          (ws_b_abandon[1] && (ws_slv[1] == `XBAR_SLV_SDJTAG)) ||
                          (ws_b_abandon[2] && (ws_slv[2] == `XBAR_SLV_SDJTAG)) ||
                          (ws_b_abandon[3] && (ws_slv[3] == `XBAR_SLV_SDJTAG));

    // Shared "abort this write slot and answer locally" body for BOTH
    // the per-slave watchdog (poison=1) and the slv_flush abort path
    // (poison=0, CRITICAL fix part ii) — factored into one task so the
    // six-way slave dispatch (which sw_owned[]/slv_w_poisoned[] index to
    // touch) exists exactly once instead of being re-typed at every one
    // of the three abort call sites (WS_WAIT_SLV_AW / WS_FWD_W /
    // WS_WAIT_B), which is what let the two abort paths drift out of
    // sync in the first place.  Plain (non-automatic) task: called only
    // from the single posedge-clk write always block below, never
    // reentrantly, so static task storage is correct and this is fully
    // synthesizable Verilog-2005 (no SystemVerilog `automatic`/`logic`).
    task ws_release_slot;
        input [SLOT_W-1:0] slot;
        input [SSEL_W-1:0] slv_sel;
        input               need_drain; // 1 = still owes W beats (AW/W path); 0 = WAIT_B path, straight to local B
        input               poison;     // 1 = real watchdog timeout (sticky); 0 = slv_flush abort (non-sticky)
        begin
            case (slv_sel)
                `XBAR_SLV_DDR: begin
                    s_awvalid_r[0] <= 1'b0;
                    sw_owned[0]    <= 1'b0;
                    if (poison) slv_w_poisoned[0] <= 1'b1;
                end
                `XBAR_SLV_IO: begin
                    s_awvalid_r[1] <= 1'b0;
                    sw_owned[1]    <= 1'b0;
                    if (poison) slv_w_poisoned[1] <= 1'b1;
                end
                `XBAR_SLV_DMA: begin
                    s_awvalid_r[2] <= 1'b0;
                    sw_owned[2]    <= 1'b0;
                    if (poison) slv_w_poisoned[2] <= 1'b1;
                end
                `XBAR_SLV_VRAM: begin
                    s_awvalid_r[3] <= 1'b0;
                    sw_owned[3]    <= 1'b0;
                    if (poison) slv_w_poisoned[3] <= 1'b1;
                end
                `XBAR_SLV_DAFB: begin
                    s_awvalid_r[4] <= 1'b0;
                    sw_owned[4]    <= 1'b0;
                    if (poison) slv_w_poisoned[4] <= 1'b1;
                end
                default: begin // XBAR_SLV_SDJTAG
                    s_awvalid_r[5] <= 1'b0;
                    sw_owned[5]    <= 1'b0;
                    if (poison) slv_w_poisoned[5] <= 1'b1;
                end
            endcase
            if ((slot == {SLOT_W{1'b0}}) &&
                (m0_abandon || (m0_wsel_q != cpu_held_in_reset))) begin
                // IMPORTANT 3 (review round 2): slot 0 is the CPU-LSU /
                // boot-FSM SHARED physical port (see the M0/boot merge
                // header note).  If the side that OWNS this in-flight
                // write (m0_wsel_q) is no longer the side
                // cpu_held_in_reset selects, that owner has been reset or
                // retired away and cannot consume a response — and
                // draining here would risk consuming the OTHER side's
                // real W beats.  Discard silently: no drain, no local
                // response, straight back to idle so the slot is
                // available to whichever side is live.
                //
                // The predicate is a MISMATCH, not plain
                // `cpu_held_in_reset`.  With the select latched
                // (m0_wsel_q, above) a local B is delivered to the
                // transaction's actual owner, so a boot-FSM write that
                // aborts while cpu_held_in_reset is still asserted CAN
                // and MUST be answered — the old unconditional
                // `cpu_held_in_reset` test discarded it, leaving boot_fsm
                // waiting for a BRESP that never came.
                //
                // GAP 1: `m0_abandon` is ORed in because it is the STICKY
                // record of that same mismatch.  Once the owner has been
                // seen to go away, a later abort of this transaction must
                // still discard — even if cpu_held_in_reset has flipped
                // back to match m0_wsel_q in the meantime, which would
                // otherwise route a local B to a master that never asked
                // for it (and, since nobody would take it, wedge the slot
                // in the unwatchdogged WS_SEND_BLOCAL).
                ws_state[slot] <= WS_IDLE;
            end else if (need_drain) begin
                ws_local_rsp[slot] <= `AXI_RESP_SLVERR;
                ws_state[slot] <= WS_DRAIN_W;
                // IMPORTANT 2a fix: len+1, NOT a fixed 8'hff.  8'hff
                // left the final WLAST beat of a 256-beat (len==255)
                // burst still pending after WS_DRAIN_W's own
                // "bcnt==1 || WLAST" termination consumed only 255 of
                // them, misaligning that master's W channel into its
                // NEXT write.  len+1 wraps to 8'd0 for len==255, which
                // is exactly the pre-existing convention the local
                // NONE-decode path already relies on (see
                // WS_DRAIN_W/req_local below) — 0 decrements through
                // 255..1 before hitting the "==1" terminal check, i.e.
                // 256 beats, the correct count.
                ws_bcnt[slot] <= ws_len[slot] + 8'd1;
            end else begin
                ws_local_rsp[slot] <= `AXI_RESP_SLVERR;
                ws_state[slot] <= WS_SEND_BLOCAL;
            end
        end
    endtask

    wire [SLOT_W-1:0] aw_win_s0 = rr_pick_w(sw_rr_ptr[0], req_ddr [0], req_ddr [1], req_ddr [2], req_ddr [3]);
    wire [SLOT_W-1:0] aw_win_s1 = rr_pick_w(sw_rr_ptr[1], req_io  [0], req_io  [1], req_io  [2], req_io  [3]);
    wire [SLOT_W-1:0] aw_win_s2 = rr_pick_w(sw_rr_ptr[2], req_dma [0], req_dma [1], req_dma [2], req_dma [3]);
    wire [SLOT_W-1:0] aw_win_s3 = rr_pick_w(sw_rr_ptr[3], req_vram[0], req_vram[1], req_vram[2], req_vram[3]);
    wire [SLOT_W-1:0] aw_win_s4 = rr_pick_w(sw_rr_ptr[4], req_dafb[0], req_dafb[1], req_dafb[2], req_dafb[3]);
    wire [SLOT_W-1:0] aw_win_s5 = rr_pick_w(sw_rr_ptr[5], req_sd_jtag[0], req_sd_jtag[1], req_sd_jtag[2], req_sd_jtag[3]);
    wire              any_req_s0 = |req_ddr;
    wire              any_req_s1 = |req_io;
    wire              any_req_s2 = |req_dma;
    wire              any_req_s3 = |req_vram;
    wire              any_req_s4 = |req_dafb;
    wire              any_req_s5 = |req_sd_jtag;

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < NW; i = i + 1) begin
                ws_state    [i] <= WS_IDLE;
                ws_slv      [i] <= `XBAR_SLV_NONE;
                ws_mid      [i] <= {ID_WIDTH{1'b0}};
                ws_addr_flat[i] <= 32'h0;
                ws_len      [i] <= 8'h0;
                ws_size     [i] <= 3'h0;
                ws_burst    [i] <= 2'h0;
                ws_local_rsp[i] <= `AXI_RESP_OKAY;
                ws_bcnt     [i] <= 8'h0;
                ws_wdone    [i] <= 1'b0;
                ws_wd_cnt   [i] <= {WD_CNT_W{1'b0}};
                ws_bhold_cnt[i] <= {B_HOLD_CNT_W{1'b0}};
                ws_lhold_cnt[i] <= {B_HOLD_CNT_W{1'b0}};
                ws_wbeats   [i] <= 9'd0;
                ws_wlast_pend[i]<= 1'b0;
                mw_awready_r[i] <= 1'b0;
            end
            m0_pad_w <= 1'b0;
            s3_flush_active <= 1'b0;
            s3_flush_wbeats <= 9'd0;
            s3_flush_wd_cnt <= {WD_CNT_W{1'b0}};
            for (i = 0; i < NS; i = i + 1) begin
                sw_owner  [i] <= {SLOT_W{1'b0}};
                sw_owned  [i] <= 1'b0;
                sw_rr_ptr [i] <= {SLOT_W{1'b0}};
                s_awid_r  [i] <= {XID_WIDTH{1'b0}};
                s_awaddr_r[i] <= 32'h0;
                s_awlen_r [i] <= 8'h0;
                s_awsize_r[i] <= 3'h0;
                s_awburst_r[i]<= 2'h0;
                s_awvalid_r[i]<= 1'b0;
                slv_w_poisoned[i] <= 1'b0;
            end
        end else begin
            for (i = 0; i < NW; i = i + 1) mw_awready_r[i] <= 1'b0;

            // The aborted upstream slot is free to drain and return its
            // local response while this context independently closes S3's
            // already accepted transaction.  No response can alias because
            // S3 admission below remains blocked until the stale B is sunk.
            if (s3_flush_active) begin
                if ((s3_flush_wbeats == 9'd0) && s3_bvalid) begin
                    s3_flush_active <= 1'b0;
                    s3_flush_wd_cnt <= {WD_CNT_W{1'b0}};
                end else if (s3_flush_wd_cnt == WD_MAX_CNT) begin
                    s3_flush_active <= 1'b0;
                    s3_flush_wbeats <= 9'd0;
                    s3_flush_wd_cnt <= {WD_CNT_W{1'b0}};
                    slv_w_poisoned[3] <= 1'b1;
                    // synthesis translate_off
                    $display("AXI_XBAR [%0t]: S3 flush cleanup timed out; poisoning VRAM slave", $time);
                    // synthesis translate_on
                end else begin
                    if ((s3_flush_wbeats != 9'd0) && s3_wready)
                        s3_flush_wbeats <= s3_flush_wbeats - 9'd1;
                    s3_flush_wd_cnt <= s3_flush_wd_cnt + {{(WD_CNT_W-1){1'b0}}, 1'b1};
                end
            end else begin
                s3_flush_wd_cnt <= {WD_CNT_W{1'b0}};
            end

            // Fix 2: advance/hold each write slot's watchdog counter.
            // Held (not incremented) the cycle it fires — the case
            // statement below moves ws_state[i] out of the active set
            // that same cycle, so next cycle's "!ws_active" branch zeroes
            // it, ready for the slot's next transaction.  CRITICAL fix
            // part i: frozen (no increment at all) while slv_flush is
            // asserted.  Saturates at WD_MAX_CNT rather than wrapping
            // when ws_wd_fire is suppressed for a reason OTHER than
            // "already fired this cycle" (MINOR 4's mw_bvalid_slv
            // suppression, or slv_flush itself) — without the
            // saturation a suppressed-but-still-active slot would wrap
            // ws_wd_cnt back through 0, silently resetting the whole
            // 2^WD_LOG2-cycle window.
            // RESET-WEDGE, corrected 2026-09-12.  An earlier revision froze the S1
            // watchdog while `cpu_held_in_reset`, believing the watchdog fired and
            // STICKY-POISONED S1.  Hardware refuted that: on the instrumentation
            // bitstream `xbar_slv_poisoned` reads 0x00 BOTH while running and after a
            // wedging `reset` -- S1 is never poisoned.  The freeze also removed the only
            // release path S1 has (ws_flush_abort/rs_flush_abort cannot fire for S1: it
            // is deliberately outside the flush domain), stranding the slot ACTIVE
            // forever -- measured as xbar_s1_slot_busy = 1 after reset, 0 while running.
            // The freeze is gone; the real release path is the slot-0 owner-gone term on
            // ws_flush_abort/rs_flush_abort.
            for (i = 0; i < NW; i = i + 1) begin
                if (!ws_active[i])
                    ws_wd_cnt[i] <= {WD_CNT_W{1'b0}};
                else if (!slv_flush &&
                         (ws_wd_cnt[i] != ((ws_slv[i] == `XBAR_SLV_IO) ? WD_MAX_CNT_S1
                                                                       : WD_MAX_CNT)))
                    ws_wd_cnt[i] <= ws_wd_cnt[i] + {{(WD_CNT_W-1){1'b0}}, 1'b1};
            end

            // Task #219: B-hold grace counter.  Runs only while the real
            // slave's B is asserted at this slot and the owning master
            // has NOT taken it; cleared the moment either goes away (so
            // an ordinary one-or-two-cycle BREADY latency never gets
            // anywhere near B_HOLD_MAX), frozen under slv_flush like
            // ws_wd_cnt, and saturating rather than wrapping for the
            // same reason ws_wd_cnt saturates.
            for (i = 0; i < NW; i = i + 1) begin
                if (!mw_bvalid_slv[i] || mw_bready[i])
                    ws_bhold_cnt[i] <= {B_HOLD_CNT_W{1'b0}};
                else if (!slv_flush && (ws_bhold_cnt[i] != B_HOLD_MAX))
                    ws_bhold_cnt[i] <= ws_bhold_cnt[i] +
                                       {{(B_HOLD_CNT_W-1){1'b0}}, 1'b1};
                // Race audit 2026-09-18: the shared LOCAL-completion hold
                // counter (see its declaration).  Cleared the moment the
                // master does anything -- sends a W beat in WS_DRAIN_W, or
                // raises BREADY in WS_SEND_BLOCAL -- so a slow-but-live
                // master working through a 256-beat drain, which re-arms on
                // every beat, never gets near B_HOLD_MAX.  Frozen under
                // slv_flush and saturating, like every other counter here.
                if (!ws_lhold_run[i])
                    ws_lhold_cnt[i] <= {B_HOLD_CNT_W{1'b0}};
                else if (!slv_flush && (ws_lhold_cnt[i] != B_HOLD_MAX))
                    ws_lhold_cnt[i] <= ws_lhold_cnt[i] +
                                       {{(B_HOLD_CNT_W-1){1'b0}}, 1'b1};
            end

            // GAP 1 bookkeeping: track how many W beats the slave still owes
            // the burst each slot is engaged in.  Counts REAL and FILLER
            // beats alike — both consume one beat of the slave's burst
            // (axi_bridge_w_pad.v's `w_fire`).  Reloaded from AWLEN at every
            // AW-arb / local-drain admission below, which is later in this
            // same always block and therefore wins over this decrement on a
            // same-cycle collision.
            for (i = 0; i < NW; i = i + 1) begin
                if (mw_wvalid[i] && mw_wready_raw[i] && (ws_wbeats[i] != 9'd0)) begin
                    ws_wbeats    [i] <= ws_wbeats[i] - 9'd1;
                    ws_wlast_pend[i] <= (ws_wbeats[i] == 9'd2);
                end
            end

            // slv_w_poisoned cleared on slv_flush's rising edge (CRITICAL
            // fix, part iv) — a unified reset is a legitimate fresh
            // start; a slave poisoned before the reset window should not
            // still read poisoned after it.
            if (slv_flush_rise) begin
                for (i = 0; i < NS; i = i + 1) slv_w_poisoned[i] <= 1'b0;
            end

            for (i = 0; i < NW; i = i + 1) begin
                case (ws_state[i])
                    WS_WAIT_SLV_AW: begin
                        // Early-W (see ws*_drive comment): W beats flow
                        // during this state; record wlast acceptance so
                        // the AW-accept transition can skip WS_FWD_W.
                        // The ws_wdone[i] read below sees the OLD
                        // registered value, so each transition ORs in
                        // the same-cycle wlast-handshake term — an
                        // AW-accept coincident with the wlast beat also
                        // goes straight to WS_WAIT_B.
                        if (mw_wvalid[i] && mw_wready_raw[i] && mw_wlast[i])
                            ws_wdone[i] <= 1'b1;
                        if (ws_slv[i] == `XBAR_SLV_DDR) begin
                            if (s0_awvalid && s0_awready && sw_owned[0] && (sw_owner[0] == i[SLOT_W-1:0])) begin
                                s_awvalid_r[0] <= 1'b0;
                                ws_state[i]    <= (ws_wdone[i] ||
                                                   (mw_wvalid[i] && mw_wready_raw[i] && mw_wlast[i]))
                                                  ? WS_WAIT_B : WS_FWD_W;
                            end else if (ws_flush_abort[i]) begin
                                ws_release_slot(i[SLOT_W-1:0], ws_slv[i], ws_needs_drain[i], 1'b0);
                            end else if (ws_wd_fire[i]) begin
                                ws_release_slot(i[SLOT_W-1:0], ws_slv[i], ws_needs_drain[i], 1'b1);
                            end
                        end else if (ws_slv[i] == `XBAR_SLV_IO) begin
                            if (s1_awvalid && s1_awready && sw_owned[1] && (sw_owner[1] == i[SLOT_W-1:0])) begin
                                s_awvalid_r[1] <= 1'b0;
                                ws_state[i]    <= (ws_wdone[i] ||
                                                   (mw_wvalid[i] && mw_wready_raw[i] && mw_wlast[i]))
                                                  ? WS_WAIT_B : WS_FWD_W;
                            end else if (ws_flush_abort[i]) begin
                                ws_release_slot(i[SLOT_W-1:0], ws_slv[i], ws_needs_drain[i], 1'b0);
                            end else if (ws_wd_fire[i]) begin
                                ws_release_slot(i[SLOT_W-1:0], ws_slv[i], ws_needs_drain[i], 1'b1);
                            end
                        end else if (ws_slv[i] == `XBAR_SLV_DMA) begin
                            if (s2_awvalid && s2_awready && sw_owned[2] && (sw_owner[2] == i[SLOT_W-1:0])) begin
                                s_awvalid_r[2] <= 1'b0;
                                ws_state[i]    <= (ws_wdone[i] ||
                                                   (mw_wvalid[i] && mw_wready_raw[i] && mw_wlast[i]))
                                                  ? WS_WAIT_B : WS_FWD_W;
                            end else if (ws_flush_abort[i]) begin
                                ws_release_slot(i[SLOT_W-1:0], ws_slv[i], ws_needs_drain[i], 1'b0);
                            end else if (ws_wd_fire[i]) begin
                                ws_release_slot(i[SLOT_W-1:0], ws_slv[i], ws_needs_drain[i], 1'b1);
                            end
                        end else if (ws_slv[i] == `XBAR_SLV_VRAM) begin
                            if (s3_awvalid && s3_awready && sw_owned[3] && (sw_owner[3] == i[SLOT_W-1:0])) begin
                                if (ws_flush_abort[i]) begin
                                    if (S3_BACKEND_SURVIVES_FLUSH) begin
                                        s3_flush_active <= 1'b1;
                                        s3_flush_wbeats <= ws_wbeats_next[i];
                                        s3_flush_wd_cnt <= {WD_CNT_W{1'b0}};
                                    end
                                    ws_release_slot(i[SLOT_W-1:0], ws_slv[i],
                                                    ws_needs_drain[i], 1'b0);
                                end else begin
                                    s_awvalid_r[3] <= 1'b0;
                                    ws_state[i]    <= (ws_wdone[i] ||
                                                       (mw_wvalid[i] && mw_wready_raw[i] && mw_wlast[i]))
                                                      ? WS_WAIT_B : WS_FWD_W;
                                end
                            end else if (ws_flush_abort[i]) begin
                                ws_release_slot(i[SLOT_W-1:0], ws_slv[i], ws_needs_drain[i], 1'b0);
                            end else if (ws_wd_fire[i]) begin
                                ws_release_slot(i[SLOT_W-1:0], ws_slv[i], ws_needs_drain[i], 1'b1);
                            end
                        end else if (ws_slv[i] == `XBAR_SLV_DAFB) begin
                            if (s4_awvalid && s4_awready && sw_owned[4] && (sw_owner[4] == i[SLOT_W-1:0])) begin
                                s_awvalid_r[4] <= 1'b0;
                                ws_state[i]    <= (ws_wdone[i] ||
                                                   (mw_wvalid[i] && mw_wready_raw[i] && mw_wlast[i]))
                                                  ? WS_WAIT_B : WS_FWD_W;
                            end else if (ws_flush_abort[i]) begin
                                ws_release_slot(i[SLOT_W-1:0], ws_slv[i], ws_needs_drain[i], 1'b0);
                            end else if (ws_wd_fire[i]) begin
                                ws_release_slot(i[SLOT_W-1:0], ws_slv[i], ws_needs_drain[i], 1'b1);
                            end
                        end else if (ws_slv[i] == `XBAR_SLV_SDJTAG) begin
                            if (s5_awvalid && s5_awready && sw_owned[5] && (sw_owner[5] == i[SLOT_W-1:0])) begin
                                s_awvalid_r[5] <= 1'b0;
                                ws_state[i]    <= (ws_wdone[i] ||
                                                   (mw_wvalid[i] && mw_wready_raw[i] && mw_wlast[i]))
                                                  ? WS_WAIT_B : WS_FWD_W;
                            end else if (ws_flush_abort[i]) begin
                                ws_release_slot(i[SLOT_W-1:0], ws_slv[i], ws_needs_drain[i], 1'b0);
                            end else if (ws_wd_fire[i]) begin
                                ws_release_slot(i[SLOT_W-1:0], ws_slv[i], ws_needs_drain[i], 1'b1);
                            end
                        end
                    end

                    WS_FWD_W: begin
                        if (mw_wvalid[i] && mw_wready_raw[i] && mw_wlast[i]) begin
                            if (ws_flush_abort[i] &&
                                (ws_slv[i] == `XBAR_SLV_VRAM)) begin
                                if (S3_BACKEND_SURVIVES_FLUSH) begin
                                    s3_flush_active <= 1'b1;
                                    s3_flush_wbeats <= 9'd0;
                                    s3_flush_wd_cnt <= {WD_CNT_W{1'b0}};
                                end
                                ws_release_slot(i[SLOT_W-1:0], ws_slv[i], 1'b0, 1'b0);
                            end else begin
                                ws_state[i] <= WS_WAIT_B;
                            end
                        end else if (ws_flush_abort[i]) begin
                            // Bridge behind this slot is in reset — drop
                            // it, drain any still-in-flight W beats via
                            // WS_DRAIN_W (self-driven wready=1), answer
                            // SLVERR, do NOT poison (see slv_flush
                            // contract / ws_release_slot).
                            //
                            // Ordered AHEAD of the GAP-1 pad below: under
                            // slv_flush the SLAVE itself is being reset, so
                            // padding its burst out is pointless (and could
                            // stall against a WREADY the flushed bridge is
                            // holding low).  This keeps the well-exercised
                            // flush path behaviourally untouched.
                            if (S3_BACKEND_SURVIVES_FLUSH &&
                                (ws_slv[i] == `XBAR_SLV_VRAM)) begin
                                s3_flush_active <= 1'b1;
                                s3_flush_wbeats <= ws_wbeats_next[i];
                                s3_flush_wd_cnt <= {WD_CNT_W{1'b0}};
                            end
                            ws_release_slot(i[SLOT_W-1:0], ws_slv[i], 1'b1, 1'b0);
                        end else if (m0_abandon && (i == 0)) begin
                            // GAP 1: the master owning this burst is gone
                            // and will never send the remaining beats.
                            // Finish the burst here with wstrb=0 filler so
                            // the slave's FSM returns to idle BEFORE any
                            // fresh write is admitted; the B it then emits
                            // is swallowed in WS_WAIT_B (mw_bready[0] is
                            // forced by m0_abandon).
                            //
                            // Ordered ahead of ws_wd_fire because that is
                            // the entire point: letting the watchdog reach
                            // this slot poisons the slave for every master
                            // until the next rst.
                            if (ws_wbeats_next[i] == 9'd0) begin
                                // Nothing owed once this cycle's beat is
                                // counted — the slave's burst is already
                                // complete, so go collect (and swallow) its
                                // B without padding anything.
                                ws_state[i] <= WS_WAIT_B;
                            end else begin
                                ws_state[i] <= WS_PAD_W;
                                m0_pad_w    <= 1'b1;
                                // synthesis translate_off
                                $display("AXI_XBAR [%0t]: slot-0 write burst abandoned by its master mid-W (slv=%0d, %0d beat(s) owed); completing with wstrb=0 filler beats",
                                         $time, ws_slv[i], ws_wbeats_next[i]);
                                // synthesis translate_on
                            end
                        end else if (ws_wd_fire[i]) begin
                            // Slave's W-channel (wready) never advanced to
                            // WLAST — release the slot, poison the slave,
                            // and switch to WS_DRAIN_W (which self-drives
                            // wready=1) so the master's still-in-flight W
                            // beats get sunk rather than left stalled.
                            ws_release_slot(i[SLOT_W-1:0], ws_slv[i], 1'b1, 1'b1);
                        end
                    end

                    WS_WAIT_B: begin
                        // Task #219: ws_b_abandon[i] joins the normal
                        // completion branch rather than getting its own.
                        // Both do exactly the same thing to the fabric —
                        // release sw_owned, advance the round-robin
                        // pointer, idle the slot — and in both cases the
                        // slave's B IS consumed this cycle (by the
                        // master's own BREADY, or by the forced
                        // b_abandon_s* pulse ORed into s*_bready above).
                        // The only difference is that nobody upstream
                        // hears about it, which is correct: the owning
                        // master has provably walked away, and this path
                        // deliberately does NOT poison the slave, which
                        // is healthy.
                        if ((mw_bvalid_slv[i] && mw_bready[i]) || ws_b_abandon[i]) begin
                            // synthesis translate_off
                            if (ws_b_abandon[i])
                                $display("AXI_XBAR [%0t]: slot-%0d write B abandoned by its master (slv=%0d); retiring B and releasing the slave lock, no poison",
                                         $time, i, ws_slv[i]);
                            // synthesis translate_on
                            ws_state[i] <= WS_IDLE;
                            if (ws_slv[i] == `XBAR_SLV_DDR) begin
                                sw_owned [0] <= 1'b0;
                                sw_rr_ptr[0] <= i[SLOT_W-1:0] + 2'd1;
                            end else if (ws_slv[i] == `XBAR_SLV_IO) begin
                                sw_owned [1] <= 1'b0;
                                sw_rr_ptr[1] <= i[SLOT_W-1:0] + 2'd1;
                            end else if (ws_slv[i] == `XBAR_SLV_DMA) begin
                                sw_owned [2] <= 1'b0;
                                sw_rr_ptr[2] <= i[SLOT_W-1:0] + 2'd1;
                            end else if (ws_slv[i] == `XBAR_SLV_VRAM) begin
                                sw_owned [3] <= 1'b0;
                                sw_rr_ptr[3] <= i[SLOT_W-1:0] + 2'd1;
                            end else if (ws_slv[i] == `XBAR_SLV_DAFB) begin
                                sw_owned [4] <= 1'b0;
                                sw_rr_ptr[4] <= i[SLOT_W-1:0] + 2'd1;
                            end else begin
                                sw_owned [5] <= 1'b0;
                                sw_rr_ptr[5] <= i[SLOT_W-1:0] + 2'd1;
                            end
                        end else if (ws_flush_abort[i]) begin
                            // AW/W already completed with the slave — it
                            // just never returned B before the bridge
                            // reset it out from under us.  No W left to
                            // drain: straight to local B, no poison.
                            if (S3_BACKEND_SURVIVES_FLUSH &&
                                (ws_slv[i] == `XBAR_SLV_VRAM)) begin
                                s3_flush_active <= 1'b1;
                                s3_flush_wbeats <= 9'd0;
                                s3_flush_wd_cnt <= {WD_CNT_W{1'b0}};
                            end
                            ws_release_slot(i[SLOT_W-1:0], ws_slv[i], 1'b0, 1'b0);
                        end else if (ws_wd_fire[i]) begin
                            // AW/W already completed with the slave — it
                            // just never returned B.  Release + poison,
                            // synthesize the terminal B locally.  Any
                            // late s0..s5_bvalid for this transaction is
                            // ignored: sw_owned[slv] is now 0 and, since
                            // the slave is poisoned, is never re-armed
                            // against the real slave again, so
                            // mw_bvalid_slv[] can never re-match it.
                            ws_release_slot(i[SLOT_W-1:0], ws_slv[i], 1'b0, 1'b1);
                        end
                    end

                    // GAP 1: the crossbar is driving filler beats into the
                    // owned slave to complete a burst its master abandoned.
                    // m0_pad_w forces WVALID=1 / WSTRB=0 / WLAST=
                    // ws_wlast_pend at the slot-0 fan-in mux, so the
                    // ordinary "wlast beat accepted" test is exactly the
                    // right terminal condition here — it fires on the
                    // filler beat that carries the burst's WLAST.
                    WS_PAD_W: begin
                        if (mw_wvalid[i] && mw_wready_raw[i] && mw_wlast[i]) begin
                            ws_state[i] <= WS_WAIT_B;
                            m0_pad_w    <= 1'b0;
                        end else if (ws_flush_abort[i]) begin
                            // Slave is being reset underneath us; stop
                            // padding and take the existing non-poisoning
                            // flush exit.  need_drain=0: the master is gone,
                            // there are no master beats to sink.
                            m0_pad_w <= 1'b0;
                            ws_release_slot(i[SLOT_W-1:0], ws_slv[i], 1'b0, 1'b0);
                        end else if (ws_wd_fire[i]) begin
                            // The slave will not even accept wstrb=0 filler:
                            // genuinely wedged, not merely abandoned.  Fall
                            // back to the existing loud failure (release +
                            // poison) rather than padding here forever.
                            m0_pad_w <= 1'b0;
                            ws_release_slot(i[SLOT_W-1:0], ws_slv[i], 1'b0, 1'b1);
                        end
                    end

                    WS_DRAIN_W: begin
                        if (mw_wvalid[i] && mw_wready_raw[i]) begin
                            if (ws_bcnt[i] == 8'd1 || mw_wlast[i]) begin
                                ws_state[i] <= WS_SEND_BLOCAL;
                            end else begin
                                ws_bcnt[i] <= ws_bcnt[i] - 8'd1;
                            end
                        end else if (ws_w_abandon[i]) begin
                            // Race audit 2026-09-18: the master is never
                            // going to send the rest.  Nothing watches this
                            // state (it is outside ws_active), so without
                            // this the slot is pinned forever WITH WREADY
                            // HELD HIGH, and the master's next write's beats
                            // are consumed as this dead burst's
                            // continuation.  Offer the local B; if the
                            // master is really gone, ws_bloc_abandon below
                            // retires that too.
                            ws_state[i] <= WS_SEND_BLOCAL;
                            // synthesis translate_off
                            $display("AXI_XBAR [%0t]: slot-%0d W drain abandoned by its master (%0d beat(s) still owed); answering locally",
                                     $time, i, ws_bcnt[i]);
                            // synthesis translate_on
                        end
                    end

                    WS_SEND_BLOCAL: begin
                        if (mw_bvalid_loc[i] && mw_bready[i]) begin
                            ws_state[i] <= WS_IDLE;
                        end else if (ws_bloc_abandon[i]) begin
                            // Race audit 2026-09-18: the write-side twin of
                            // rs_r_abandon (backlog item 5).  The master
                            // walked away from a tail the CROSSBAR
                            // synthesized; no slave was ever involved, so
                            // nothing downstream is left inconsistent and no
                            // poison is warranted.  Withdrawing the
                            // unaccepted BVALID is the same deliberate
                            // deviation ws_b_abandon and rs_r_abandon
                            // already make, and it is REQUIRED rather than
                            // merely convenient: leaving it asserted lets it
                            // be mis-attributed to this master's next write.
                            ws_state[i] <= WS_IDLE;
                            // synthesis translate_off
                            $display("AXI_XBAR [%0t]: slot-%0d local B tail abandoned by its master; releasing the write slot, no poison",
                                     $time, i);
                            // synthesis translate_on
                        end
                    end

                    default: ;
                endcase
            end

            if (!sw_owned[0] && any_req_s0 && !s_awvalid_r[0]) begin
                sw_owner[0] <= aw_win_s0;
                sw_owned[0] <= 1'b1;
                s_awid_r   [0] <= {aw_win_s0, mw_awid[aw_win_s0]};
                s_awaddr_r [0] <= ddr_flatten(mw_awaddr_eff[aw_win_s0]);
                s_awlen_r  [0] <= mw_awlen  [aw_win_s0];
                s_awsize_r [0] <= mw_awsize [aw_win_s0];
                s_awburst_r[0] <= mw_awburst[aw_win_s0];
                s_awvalid_r[0] <= 1'b1;
                ws_state    [aw_win_s0] <= WS_WAIT_SLV_AW;
                ws_wdone    [aw_win_s0] <= 1'b0;
                ws_slv      [aw_win_s0] <= `XBAR_SLV_DDR;
                ws_mid      [aw_win_s0] <= mw_awid  [aw_win_s0];
                ws_addr_flat[aw_win_s0] <= ddr_flatten(mw_awaddr_eff[aw_win_s0]);
                ws_len      [aw_win_s0] <= mw_awlen [aw_win_s0];
                // GAP 1: AWLEN+1 beats now owed to this slave.
                ws_wbeats   [aw_win_s0] <= {1'b0, mw_awlen[aw_win_s0]} + 9'd1;
                ws_wlast_pend[aw_win_s0] <= (mw_awlen[aw_win_s0] == 8'd0);
                ws_size     [aw_win_s0] <= mw_awsize[aw_win_s0];
                ws_burst    [aw_win_s0] <= mw_awburst[aw_win_s0];
                mw_awready_r[aw_win_s0] <= 1'b1;
            end

            if (!sw_owned[1] && any_req_s1 && !s_awvalid_r[1]) begin
                sw_owner[1] <= aw_win_s1;
                sw_owned[1] <= 1'b1;
                s_awid_r   [1] <= {aw_win_s1, mw_awid[aw_win_s1]};
                s_awaddr_r [1] <= mw_awaddr_eff[aw_win_s1];
                s_awlen_r  [1] <= mw_awlen  [aw_win_s1];
                s_awsize_r [1] <= mw_awsize [aw_win_s1];
                s_awburst_r[1] <= mw_awburst[aw_win_s1];
                s_awvalid_r[1] <= 1'b1;
                ws_state    [aw_win_s1] <= WS_WAIT_SLV_AW;
                ws_wdone    [aw_win_s1] <= 1'b0;
                ws_slv      [aw_win_s1] <= `XBAR_SLV_IO;
                ws_mid      [aw_win_s1] <= mw_awid  [aw_win_s1];
                ws_addr_flat[aw_win_s1] <= mw_awaddr_eff[aw_win_s1];
                ws_len      [aw_win_s1] <= mw_awlen [aw_win_s1];
                // GAP 1: AWLEN+1 beats now owed to this slave.
                ws_wbeats   [aw_win_s1] <= {1'b0, mw_awlen[aw_win_s1]} + 9'd1;
                ws_wlast_pend[aw_win_s1] <= (mw_awlen[aw_win_s1] == 8'd0);
                ws_size     [aw_win_s1] <= mw_awsize[aw_win_s1];
                ws_burst    [aw_win_s1] <= mw_awburst[aw_win_s1];
                mw_awready_r[aw_win_s1] <= 1'b1;
            end

            if (!sw_owned[2] && any_req_s2 && !s_awvalid_r[2]) begin
                sw_owner[2] <= aw_win_s2;
                sw_owned[2] <= 1'b1;
                s_awid_r   [2] <= {aw_win_s2, mw_awid[aw_win_s2]};
                s_awaddr_r [2] <= mw_awaddr_eff[aw_win_s2];
                s_awlen_r  [2] <= mw_awlen  [aw_win_s2];
                s_awsize_r [2] <= mw_awsize [aw_win_s2];
                s_awburst_r[2] <= mw_awburst[aw_win_s2];
                s_awvalid_r[2] <= 1'b1;
                ws_state    [aw_win_s2] <= WS_WAIT_SLV_AW;
                ws_wdone    [aw_win_s2] <= 1'b0;
                ws_slv      [aw_win_s2] <= `XBAR_SLV_DMA;
                ws_mid      [aw_win_s2] <= mw_awid  [aw_win_s2];
                ws_addr_flat[aw_win_s2] <= mw_awaddr_eff[aw_win_s2];
                ws_len      [aw_win_s2] <= mw_awlen [aw_win_s2];
                // GAP 1: AWLEN+1 beats now owed to this slave.
                ws_wbeats   [aw_win_s2] <= {1'b0, mw_awlen[aw_win_s2]} + 9'd1;
                ws_wlast_pend[aw_win_s2] <= (mw_awlen[aw_win_s2] == 8'd0);
                ws_size     [aw_win_s2] <= mw_awsize[aw_win_s2];
                ws_burst    [aw_win_s2] <= mw_awburst[aw_win_s2];
                mw_awready_r[aw_win_s2] <= 1'b1;
            end

            if (!sw_owned[3] && !s3_flush_active && any_req_s3 && !s_awvalid_r[3]) begin
                sw_owner[3] <= aw_win_s3;
                sw_owned[3] <= 1'b1;
                s_awid_r   [3] <= {aw_win_s3, mw_awid[aw_win_s3]};
                // VRAM aperture: forward a zero-based byte offset.
                s_awaddr_r [3] <= vram_flatten(mw_awaddr_eff[aw_win_s3]);
                s_awlen_r  [3] <= mw_awlen  [aw_win_s3];
                s_awsize_r [3] <= mw_awsize [aw_win_s3];
                s_awburst_r[3] <= mw_awburst[aw_win_s3];
                s_awvalid_r[3] <= 1'b1;
                ws_state    [aw_win_s3] <= WS_WAIT_SLV_AW;
                ws_wdone    [aw_win_s3] <= 1'b0;
                ws_slv      [aw_win_s3] <= `XBAR_SLV_VRAM;
                ws_mid      [aw_win_s3] <= mw_awid  [aw_win_s3];
                ws_addr_flat[aw_win_s3] <= vram_flatten(mw_awaddr_eff[aw_win_s3]);
                ws_len      [aw_win_s3] <= mw_awlen [aw_win_s3];
                // GAP 1: AWLEN+1 beats now owed to this slave.
                ws_wbeats   [aw_win_s3] <= {1'b0, mw_awlen[aw_win_s3]} + 9'd1;
                ws_wlast_pend[aw_win_s3] <= (mw_awlen[aw_win_s3] == 8'd0);
                ws_size     [aw_win_s3] <= mw_awsize[aw_win_s3];
                ws_burst    [aw_win_s3] <= mw_awburst[aw_win_s3];
                mw_awready_r[aw_win_s3] <= 1'b1;
            end

            if (!sw_owned[4] && any_req_s4 && !s_awvalid_r[4]) begin
                sw_owner[4] <= aw_win_s4;
                sw_owned[4] <= 1'b1;
                s_awid_r   [4] <= {aw_win_s4, mw_awid[aw_win_s4]};
                // DAFB register aperture: forward a zero-based byte offset.
                s_awaddr_r [4] <= dafb_flatten(mw_awaddr_eff[aw_win_s4]);
                s_awlen_r  [4] <= mw_awlen  [aw_win_s4];
                s_awsize_r [4] <= mw_awsize [aw_win_s4];
                s_awburst_r[4] <= mw_awburst[aw_win_s4];
                s_awvalid_r[4] <= 1'b1;
                ws_state    [aw_win_s4] <= WS_WAIT_SLV_AW;
                ws_wdone    [aw_win_s4] <= 1'b0;
                ws_slv      [aw_win_s4] <= `XBAR_SLV_DAFB;
                ws_mid      [aw_win_s4] <= mw_awid  [aw_win_s4];
                ws_addr_flat[aw_win_s4] <= dafb_flatten(mw_awaddr_eff[aw_win_s4]);
                ws_len      [aw_win_s4] <= mw_awlen [aw_win_s4];
                // GAP 1: AWLEN+1 beats now owed to this slave.
                ws_wbeats   [aw_win_s4] <= {1'b0, mw_awlen[aw_win_s4]} + 9'd1;
                ws_wlast_pend[aw_win_s4] <= (mw_awlen[aw_win_s4] == 8'd0);
                ws_size     [aw_win_s4] <= mw_awsize[aw_win_s4];
                ws_burst    [aw_win_s4] <= mw_awburst[aw_win_s4];
                mw_awready_r[aw_win_s4] <= 1'b1;
            end

            if (!sw_owned[5] && any_req_s5 && !s_awvalid_r[5]) begin
                sw_owner[5] <= aw_win_s5;
                sw_owned[5] <= 1'b1;
                s_awid_r   [5] <= {aw_win_s5, mw_awid[aw_win_s5]};
                s_awaddr_r [5] <= sd_jtag_flatten(mw_awaddr_eff[aw_win_s5]);
                s_awlen_r  [5] <= mw_awlen  [aw_win_s5];
                s_awsize_r [5] <= mw_awsize [aw_win_s5];
                s_awburst_r[5] <= mw_awburst[aw_win_s5];
                s_awvalid_r[5] <= 1'b1;
                ws_state    [aw_win_s5] <= WS_WAIT_SLV_AW;
                ws_wdone    [aw_win_s5] <= 1'b0;
                ws_slv      [aw_win_s5] <= `XBAR_SLV_SDJTAG;
                ws_mid      [aw_win_s5] <= mw_awid  [aw_win_s5];
                ws_addr_flat[aw_win_s5] <= sd_jtag_flatten(mw_awaddr_eff[aw_win_s5]);
                ws_len      [aw_win_s5] <= mw_awlen [aw_win_s5];
                // GAP 1: AWLEN+1 beats now owed to this slave.
                ws_wbeats   [aw_win_s5] <= {1'b0, mw_awlen[aw_win_s5]} + 9'd1;
                ws_wlast_pend[aw_win_s5] <= (mw_awlen[aw_win_s5] == 8'd0);
                ws_size     [aw_win_s5] <= mw_awsize[aw_win_s5];
                ws_burst    [aw_win_s5] <= mw_awburst[aw_win_s5];
                mw_awready_r[aw_win_s5] <= 1'b1;
            end

            // Local-response policy (uniform MAME-canonical Q700):
            //   NONE-decoded  → OKAY (writes silently dropped, reads return
            //                   0x00000000).  Matches MAME's default
            //                   unmap-value (= 0/OKAY).
            //   ROM-write     → OKAY (writes silently dropped).  Matches
            //                   MAME-canonical Q700 where Mac OS probes /
            //                   touches the ROM-mirror during boot and the
            //                   write is silently absorbed (no SLVERR fault).
            //                   Previously SLVERR; that diverged from MAME
            //                   and triggered the post-IMMU Sad Mac on the
            //                   `MOVE.B D0,(0x80,A3)` at ROM 0x40899ABA
            //                   when A3=0x40800000 (legitimate Mac-OS path).
            if (req_local[0] && (ws_state[0] == WS_IDLE) && !mw_awready_r[0]) begin
                ws_state    [0] <= WS_DRAIN_W;
                ws_slv      [0] <= `XBAR_SLV_NONE;
                ws_mid      [0] <= mw_awid [0];
                ws_len      [0] <= mw_awlen[0];
                ws_bcnt     [0] <= mw_awlen[0] + 8'd1;
                ws_wbeats   [0] <= {1'b0, mw_awlen[0]} + 9'd1;
                ws_wlast_pend[0] <= (mw_awlen[0] == 8'd0);
                // Fix 1/2: burst-onto-lite-only-slave and poisoned-slave
                // rejections answer SLVERR; the pre-existing NONE/ROM-
                // write-probe policy above is unchanged (OKAY).
                ws_local_rsp[0] <= local_rsp_is_err_w[0] ? `AXI_RESP_SLVERR : `AXI_RESP_OKAY;
                mw_awready_r[0] <= 1'b1;
            end else if (req_local[1] && (ws_state[1] == WS_IDLE) && !mw_awready_r[1]) begin
                ws_state    [1] <= WS_DRAIN_W;
                ws_slv      [1] <= `XBAR_SLV_NONE;
                ws_mid      [1] <= mw_awid [1];
                ws_len      [1] <= mw_awlen[1];
                ws_bcnt     [1] <= mw_awlen[1] + 8'd1;
                ws_wbeats   [1] <= {1'b0, mw_awlen[1]} + 9'd1;
                ws_wlast_pend[1] <= (mw_awlen[1] == 8'd0);
                ws_local_rsp[1] <= local_rsp_is_err_w[1] ? `AXI_RESP_SLVERR : `AXI_RESP_OKAY;
                mw_awready_r[1] <= 1'b1;
            end else if (req_local[2] && (ws_state[2] == WS_IDLE) && !mw_awready_r[2]) begin
                ws_state    [2] <= WS_DRAIN_W;
                ws_slv      [2] <= `XBAR_SLV_NONE;
                ws_mid      [2] <= mw_awid [2];
                ws_len      [2] <= mw_awlen[2];
                ws_bcnt     [2] <= mw_awlen[2] + 8'd1;
                ws_wbeats   [2] <= {1'b0, mw_awlen[2]} + 9'd1;
                ws_wlast_pend[2] <= (mw_awlen[2] == 8'd0);
                ws_local_rsp[2] <= local_rsp_is_err_w[2] ? `AXI_RESP_SLVERR : `AXI_RESP_OKAY;
                mw_awready_r[2] <= 1'b1;
            end else if (req_local[3] && (ws_state[3] == WS_IDLE) && !mw_awready_r[3]) begin
                ws_state    [3] <= WS_DRAIN_W;
                ws_slv      [3] <= `XBAR_SLV_NONE;
                ws_mid      [3] <= mw_awid [3];
                ws_len      [3] <= mw_awlen[3];
                ws_bcnt     [3] <= mw_awlen[3] + 8'd1;
                ws_wbeats   [3] <= {1'b0, mw_awlen[3]} + 9'd1;
                ws_wlast_pend[3] <= (mw_awlen[3] == 8'd0);
                ws_local_rsp[3] <= local_rsp_is_err_w[3] ? `AXI_RESP_SLVERR : `AXI_RESP_OKAY;
                mw_awready_r[3] <= 1'b1;
            end
        end
    end

    // synthesis translate_off
    // Fix 3: decode is start-address-only.  A burst whose end address
    // decodes to a DIFFERENT slave than its start address would silently
    // stay routed on the start slave's decode result for its whole
    // length.  Flag it loudly in sim the cycle the xbar accepts the
    // AW/AR (mw_awready[]/mr_arready[] pulse) — sim-only, no hardware
    // effect once stripped by translate_off/_on (matches the existing
    // convention in peripheral_bus.v / vram.v / axi_ddr4_mig_bridge.v).
    // $finish (not $fatal — Verilog-2005 has no $fatal) mirrors vram.v's
    // existing sim-assert style.  MINOR 7: gated on AWBURST/ARBURST ==
    // INCR — FIXED bursts re-present the same address every beat (can
    // never straddle a region edge) and WRAP isn't used on this fabric;
    // evaluating the end-address formula against a FIXED/WRAP burst's
    // awlen/awsize would produce a false positive.
    generate
        for (mi = 0; mi < NW; mi = mi + 1) begin : gen_aw_straddle_chk
            always @(posedge clk) begin
                if (!rst && mw_awvalid[mi] && mw_awready[mi] &&
                    (mw_awburst[mi] == AXI_BURST_INCR) &&
                    (decode_slv(mw_awaddr_eff[mi] +
                        ((({24'b0, mw_awlen[mi]} + 32'd1) << mw_awsize[mi]) - 32'd1))
                     != decode_slv(mw_awaddr_eff[mi]))) begin
                    $display("AXI_XBAR ASSERT FAIL [%0t]: AW burst straddles a slave decode region: master_slot=%0d start_addr=%08h awlen=%0d awsize=%0d start_slv=%0d end_slv=%0d",
                        $time, mi, mw_awaddr_eff[mi], mw_awlen[mi], mw_awsize[mi],
                        decode_slv(mw_awaddr_eff[mi]),
                        decode_slv(mw_awaddr_eff[mi] +
                            ((({24'b0, mw_awlen[mi]} + 32'd1) << mw_awsize[mi]) - 32'd1)));
                    $finish;
                end
            end
        end
        for (mi = 0; mi < NR; mi = mi + 1) begin : gen_ar_straddle_chk
            always @(posedge clk) begin
                if (!rst && mr_arvalid[mi] && mr_arready[mi] &&
                    (mr_arburst[mi] == AXI_BURST_INCR) &&
                    (decode_slv(mr_araddr_eff[mi] +
                        ((({24'b0, mr_arlen[mi]} + 32'd1) << mr_arsize[mi]) - 32'd1))
                     != decode_slv(mr_araddr_eff[mi]))) begin
                    $display("AXI_XBAR ASSERT FAIL [%0t]: AR burst straddles a slave decode region: master_slot=%0d start_addr=%08h arlen=%0d arsize=%0d start_slv=%0d end_slv=%0d",
                        $time, mi, mr_araddr_eff[mi], mr_arlen[mi], mr_arsize[mi],
                        decode_slv(mr_araddr_eff[mi]),
                        decode_slv(mr_araddr_eff[mi] +
                            ((({24'b0, mr_arlen[mi]} + 32'd1) << mr_arsize[mi]) - 32'd1)));
                    $finish;
                end
            end
        end
        // W-beat accounting check.  The crossbar never counts W beats —
        // WS_FWD_W leaves for WS_WAIT_B purely on mw_wlast[] — so it
        // trusts each master's WLAST placement completely.  A master that
        // asserts WLAST on the wrong beat therefore makes the crossbar
        // release slave ownership early or late, and the NEXT write's
        // beats get appended to the slave's still-open burst: silent
        // corruption of the exact #162 shape, with no elaboration-time or
        // runtime symptom anywhere.  Until 2026-07-30 nothing on this
        // fabric emitted a multi-beat W burst at all (the CPU's socket
        // widener hardcoded awlen to 0, boot_fsm drives awlen=0/wlast=1),
        // so this could not be got wrong; the CPU's D-cache burst refill
        // makes it a live risk on every writeback.  Count the beats and
        // flag a mismatch loudly in sim.  Sim-only, no hardware effect —
        // same translate_off convention as the straddle checks above.
        for (mi = 0; mi < NW; mi = mi + 1) begin : gen_w_beat_chk
            reg [8:0] w_beats_seen;
            // Only beats FORWARDED to a real slave matter here.  WS_DRAIN_W
            // deliberately terminates on `bcnt==1 || mw_wlast` and forwards
            // nothing, so WLAST placement is irrelevant there — and the
            // abort scenarios in tb_axi_xbar.cpp legitimately park a
            // WLAST-asserted beat against a wedged slave to be swallowed by
            // the drain.  Checking during the drain would flag those, which
            // are testbench intent, not a fabric hazard.
            // WS_PAD_W IS included (GAP 1): the crossbar's own filler beats
            // are forwarded to a real slave, so their WLAST placement is
            // exactly as load-bearing as a master's — this assertion is what
            // makes "the pad terminates the slave's burst on the right beat"
            // machine-checked rather than argued.
            wire w_fwd_state = (ws_state[mi] == WS_FWD_W) ||
                               (ws_state[mi] == WS_PAD_W) ||
                               ((ws_state[mi] == WS_WAIT_SLV_AW) && !ws_wdone[mi]);
            always @(posedge clk) begin
                // Per-transaction counter: cleared whenever the slot is
                // idle, so a burst abandoned without WLAST (its master was
                // reset away) cannot leak a stale count into the next one.
                if (rst || (ws_state[mi] == WS_IDLE)) begin
                    w_beats_seen <= 9'd0;
                end else if (mw_wvalid[mi] && mw_wready_raw[mi]) begin
                    if (mw_wlast[mi]) w_beats_seen <= 9'd0;
                    else              w_beats_seen <= w_beats_seen + 9'd1;

                    if (w_fwd_state && mw_wlast[mi] &&
                        (w_beats_seen != {1'b0, ws_len[mi]})) begin
                        $display("AXI_XBAR ASSERT FAIL [%0t]: WLAST on the wrong beat: master_slot=%0d awlen=%0d but WLAST arrived on beat %0d (0-based)",
                            $time, mi, ws_len[mi], w_beats_seen);
                        $finish;
                    end
                    if (w_fwd_state && !mw_wlast[mi] &&
                        (w_beats_seen == {1'b0, ws_len[mi]})) begin
                        $display("AXI_XBAR ASSERT FAIL [%0t]: more W beats than AWLEN+1: master_slot=%0d awlen=%0d, beat %0d had WLAST low",
                            $time, mi, ws_len[mi], w_beats_seen);
                        $finish;
                    end
                end
            end
        end
    endgenerate
    // synthesis translate_on

    // ── Read path ────────────────────────────────────────────────────
    localparam [2:0] RS_IDLE         = 3'd0;
    localparam [2:0] RS_WAIT_SLV_AR  = 3'd1;
    localparam [2:0] RS_WAIT_R       = 3'd2;
    localparam [2:0] RS_SEND_RLOCAL  = 3'd3;
    // ── RS_DRAIN_R (backlog item 4, 2026-09-17) ─────────────────────────
    // The read-side mirror of WS_DRAIN_W.  While a slot sits in RS_WAIT_R
    // the ONLY thing that drives the slave's RREADY for it is that slot's
    // own master (s?_rready_per[] = ... && mr_rready[mi]).  If the master
    // stops -- which is exactly what axi_narrow_to_wide does when its
    // abandonment watchdog force-clears ar_valid_q, taking its w_rready low
    // and keeping it low, and the JTAG-AXI / XDMA host on M1 sits behind
    // one of those -- then the slave's RVALID is never acknowledged and
    // THAT SLAVE'S READ CHANNEL IS BLOCKED FOR EVERY MASTER.
    //
    // Nothing recovered it.  With ENABLE_WD = 0 (the shipping default)
    // rs_wd_fire is constant-false, so the slot is simply never released.
    // And with ENABLE_WD = 1 the watchdog is worse than useless here: it
    // releases the slot with poison=1 -- SLVERRing that slave for every
    // master until the next rst -- while STILL never draining the stale
    // burst, so the read channel stays blocked on top of the poison.
    // MEASURED, in the unit tb, at ENABLE_WD=1: an abandoned 4-beat S0
    // read leaves the slave owing beats forever, makes DDR unreadable for
    // every other master, and poisons S0.
    //
    // This is the read-side twin of ws_b_abandon (task #219), one channel
    // over: the B channel got this treatment, the R channel did not, and
    // the module header's claim that "the read side never had this shape"
    // is about rs_wd_fire's handshake suppression, which is a statement
    // about the WATCHDOG, not about who drives RREADY.
    //
    // In RS_DRAIN_R the crossbar drives the slave's RREADY itself and
    // consumes the rest of the burst through RLAST, then idles the slot.
    // No poison: the slave is healthy, it is the master that left.  The
    // master's own RVALID is withdrawn on entry (the state gates
    // mr_rvalid_slv), which is the same deliberate protocol deviation
    // ws_b_abandon already makes on BVALID in this module and is REQUIRED
    // here rather than merely convenient: holding RVALID at a master while
    // the payload changes every cycle underneath it would be a far worse
    // violation than withdrawing it.
    //
    // Like WS_DRAIN_W, this state carries no watchdog of its own (it is
    // excluded from rs_active).  That is sound in a way WS_DRAIN_W's
    // exclusion was not: WS_DRAIN_W waits for beats from a MASTER that may
    // never send them (which is what GAP 2 fixed), whereas this state is
    // only ever entered BECAUSE the slave is actively producing beats.  A
    // slv_flush during the drain is handled explicitly below -- the slave
    // resets and stops, so there is nothing left to drain.
    localparam [2:0] RS_DRAIN_R      = 3'd4;

    reg  [2:0]            rs_state    [0:NR-1];
    reg  [SSEL_W-1:0]     rs_slv      [0:NR-1];

    reg  [ID_WIDTH-1:0]   rs_mid      [0:NR-1];
    reg  [7:0]            rs_len      [0:NR-1];
    reg  [7:0]            rs_bcnt     [0:NR-1];
    reg  [WD_CNT_W-1:0]   rs_wd_cnt   [0:NR-1]; // Fix 2 watchdog
    // Beats already delivered to the master this transaction, tracked so
    // a mid-burst watchdog timeout (Fix 2) can synthesize exactly the
    // remaining beats rather than re-sending the whole burst.
    reg  [7:0]            rs_beats_done[0:NR-1];
    reg  [1:0]            rs_local_rsp[0:NR-1];
    // Read-side twin of ws_bhold_cnt (task #219).  See rs_r_abandon below.
    reg  [B_HOLD_CNT_W-1:0] rs_rhold_cnt [0:NR-1];
    // Backlog item 4: the same grace counter for a SLAVE-sourced beat that
    // the owning master is not taking.  See RS_DRAIN_R above.
    reg  [B_HOLD_CNT_W-1:0] rs_rslv_hold_cnt [0:NR-1];
    // Live-DDR S3 cleanup: after a flush-aborted AR, consume the old burst
    // through RLAST before another S3 AR can be admitted.
    //
    // 2026-09-17 (backlog item 2) — THIS USED TO BE A SINGLE `active` BIT
    // WITH NO RECORD OF *WHICH* BURST IT WAS DRAINING, and that is not
    // enough, because the S3 read path is ID-routed and multi-outstanding:
    // nothing on the read side corresponds to the write side's sw_owned
    // lock, so several slots can have S3 reads in flight at once and their
    // bursts may come back in any order.  Two independent failures followed:
    //
    //   (a) UNDER-DRAIN.  One slv_flush aborts EVERY slot that is on S3, but
    //       all of them set the same single bit.  The quarantine then cleared
    //       on the FIRST RLAST it saw, and every further stale burst arrived
    //       with nobody consuming it.  With VRAM_IN_DDR that burst sits on
    //       the DDR port S3 shares with S0, so it blocks EVERY DDR read --
    //       and it survives every reset the xbar has (the state is not on
    //       core_rst), so only a reconfigure clears it.
    //
    //   (b) DATA LOSS.  The `active` bit forced s3_rready unconditionally.
    //       A beat belonging to a slot that was NOT aborted -- still in
    //       RS_WAIT_R, master possibly not ready -- was handshaken away by
    //       the quarantine and never delivered.
    //
    // Both are fixed by making the quarantine a per-slot VECTOR: a bit per
    // read slot tag, set on abort, cleared only by the RLAST of a burst
    // whose own RID tag is quarantined, and forcing RREADY only for such a
    // beat.  The RREADY force is folded into the existing per-slot
    // s3_rready_per[] terms rather than ORed on at the top, so the final
    // OR4 and the per-slot cone both keep their present depth (see
    // gen_rready).  Cost: 3 extra flops and a handful of LUTs.
    //
    // The AR-admission gate stays GLOBAL (`!s3_flush_read_active`, now
    // |s3_rq_v) so the externally visible behaviour is unchanged: no new
    // S3 read is admitted while any stale burst is still owed.
    reg  [NR-1:0]         s3_rq_v;      // per read-slot stale-burst quarantine
    wire                  s3_flush_read_active = |s3_rq_v;
    reg  [WD_CNT_W-1:0]   s3_flush_read_wd_cnt;
    // Per-master "open-bus value" hint for unmapped local-response reads.
    // MAME-canonical Q700 returns 0x00000000 for ALL unmapped accesses
    // (RAM-OOR, ROM-OOR, I/O gap) — that's the default value of the
    // address space's unmap_value, and `macquadra700.cpp` never
    // overrides it.  This signal is therefore always 1 today; kept as
    // a per-master reg so a future per-region policy change is a
    // one-line edit at the AR-latch site below.
    reg                   rs_open_bus_zero[0:NR-1];

    // ROM-overlay disarm gate (see the long note at cpu_overlay_disabled_q).
    // Slot 0's reads are always the CPU LSU -- boot_fsm is write-only, so the
    // shared M0 port never carries a boot read -- and slot 2 is the CPU
    // instruction fetch.  Slot 1 (host debug) and slot 3 (DMA) are correctly
    // excluded: the overlay only aliases what the CPU sees.
    assign cpu_rd_busy = (rs_state[0] != RS_IDLE) || (rs_state[2] != RS_IDLE);

    reg  [SLOT_W-1:0]     sr_rr_ptr   [0:NS-1];
    reg  [XID_WIDTH-1:0]  s_arid_r    [0:NS-1];
    reg  [31:0]           s_araddr_r  [0:NS-1];
    reg  [7:0]            s_arlen_r   [0:NS-1];
    reg  [2:0]            s_arsize_r  [0:NS-1];
    reg  [1:0]            s_arburst_r [0:NS-1];
    reg                   s_arvalid_r [0:NS-1];

    assign s0_arid    = s_arid_r   [0];
    assign s0_araddr  = s_araddr_r [0];
    assign s0_arlen   = s_arlen_r  [0];
    assign s0_arsize  = s_arsize_r [0];
    assign s0_arburst = s_arburst_r[0];
    assign s0_arvalid = s_arvalid_r[0];
    assign s1_arid    = s_arid_r   [1];
    assign s1_araddr  = s_araddr_r [1];
    assign s1_arlen   = s_arlen_r  [1];
    assign s1_arsize  = s_arsize_r [1];
    assign s1_arburst = s_arburst_r[1];
    assign s1_arvalid = s_arvalid_r[1];
    assign s2_arid    = s_arid_r   [2];
    assign s2_araddr  = s_araddr_r [2];
    assign s2_arlen   = s_arlen_r  [2];
    assign s2_arsize  = s_arsize_r [2];
    assign s2_arburst = s_arburst_r[2];
    assign s2_arvalid = s_arvalid_r[2];
    assign s3_arid    = s_arid_r   [3];
    assign s3_araddr  = s_araddr_r [3];
    assign s3_arlen   = s_arlen_r  [3];
    assign s3_arsize  = s_arsize_r [3];
    assign s3_arburst = s_arburst_r[3];
    assign s3_arvalid = s_arvalid_r[3];
    assign s4_arid    = s_arid_r   [4];
    assign s4_araddr  = s_araddr_r [4];
    assign s4_arlen   = s_arlen_r  [4];
    assign s4_arsize  = s_arsize_r [4];
    assign s4_arburst = s_arburst_r[4];
    assign s4_arvalid = s_arvalid_r[4];
    assign s5_arid    = s_arid_r   [5];
    assign s5_araddr  = s_araddr_r [5];
    assign s5_arlen   = s_arlen_r  [5];
    assign s5_arsize  = s_arsize_r [5];
    assign s5_arburst = s_arburst_r[5];
    assign s5_arvalid = s_arvalid_r[5];

    wire [SSEL_W-1:0] dec_slv_r   [0:NR-1];
    // Fix 1 / Fix 2 — see the write-side gen_wdec block above; identical
    // policy applied on the read/AR side.  MINOR 6: every req_rd_* gate
    // (including DDR/VRAM) derives its "block forwarding" condition from
    // is_burst_reject_r[mi] / is_flush_reject_r[mi] instead of an
    // independently hardcoded (arlen==0) check — see the write-side
    // comment for the double-accept hazard this avoids.
    wire         is_burst_reject_r [0:NR-1];
    // CRITICAL fix (slv_flush, part iii) — read-side twin of
    // is_flush_reject_w.
    wire         is_flush_reject_r [0:NR-1];
    wire         is_poisoned_dec_r [0:NR-1];
    wire         local_rsp_is_err_r[0:NR-1];
    wire         req_rd_ddr  [0:NR-1];
    wire         req_rd_io   [0:NR-1];
    wire         req_rd_dma  [0:NR-1];
    wire         req_rd_vram [0:NR-1];
    wire         req_rd_dafb [0:NR-1];
    wire         req_rd_sd_jtag [0:NR-1];
    wire         req_rd_local[0:NR-1];
    generate
        for (mi = 0; mi < NR; mi = mi + 1) begin : gen_rdec
            assign dec_slv_r   [mi] = decode_slv(mr_araddr_eff[mi]);
            assign is_burst_reject_r[mi] =
                is_lite_only_slv(dec_slv_r[mi]) && (mr_arlen[mi] != 8'd0);
            // Review round 2 CRITICAL fix — read-side twin of the
            // write-side is_flush_reject_w comment above.
            assign is_flush_reject_r[mi] =
                slv_flush && is_flush_domain_slv(dec_slv_r[mi]);
            assign is_poisoned_dec_r[mi] =
                ((dec_slv_r[mi] == `XBAR_SLV_DDR)    && slv_poisoned[0]) ||
                ((dec_slv_r[mi] == `XBAR_SLV_IO)     && slv_poisoned[1]) ||
                ((dec_slv_r[mi] == `XBAR_SLV_DMA)    && slv_poisoned[2]) ||
                ((dec_slv_r[mi] == `XBAR_SLV_VRAM)   && slv_poisoned[3]) ||
                ((dec_slv_r[mi] == `XBAR_SLV_DAFB)   && slv_poisoned[4]) ||
                ((dec_slv_r[mi] == `XBAR_SLV_SDJTAG) && slv_poisoned[5]);
            assign local_rsp_is_err_r[mi] =
                is_burst_reject_r[mi] || is_poisoned_dec_r[mi] || is_flush_reject_r[mi];

            assign req_rd_ddr  [mi] = (rs_state[mi] == RS_IDLE) && mr_arvalid[mi] &&
                                      (dec_slv_r[mi] == `XBAR_SLV_DDR) &&
                                      !is_burst_reject_r[mi] && !is_flush_reject_r[mi] &&
                                      !slv_poisoned[0];
            assign req_rd_io   [mi] = (rs_state[mi] == RS_IDLE) && mr_arvalid[mi] &&
                                      (dec_slv_r[mi] == `XBAR_SLV_IO) &&
                                      !is_burst_reject_r[mi] && !is_flush_reject_r[mi] &&
                                      !s1_rst_tail_busy &&   // backlog item 3
                                      !slv_poisoned[1];
            assign req_rd_dma  [mi] = (rs_state[mi] == RS_IDLE) && mr_arvalid[mi] &&
                                      (dec_slv_r[mi] == `XBAR_SLV_DMA) &&
                                      !is_burst_reject_r[mi] && !is_flush_reject_r[mi] &&
                                      !slv_poisoned[2];
            assign req_rd_vram [mi] = (rs_state[mi] == RS_IDLE) && mr_arvalid[mi] &&
                                      (dec_slv_r[mi] == `XBAR_SLV_VRAM) &&
                                      !is_burst_reject_r[mi] && !is_flush_reject_r[mi] &&
                                      !slv_poisoned[3];
            assign req_rd_dafb [mi] = (rs_state[mi] == RS_IDLE) && mr_arvalid[mi] &&
                                      (dec_slv_r[mi] == `XBAR_SLV_DAFB) &&
                                      !is_burst_reject_r[mi] && !is_flush_reject_r[mi] &&
                                      !slv_poisoned[4];
            assign req_rd_sd_jtag[mi] = (rs_state[mi] == RS_IDLE) && mr_arvalid[mi] &&
                                        (dec_slv_r[mi] == `XBAR_SLV_SDJTAG) &&
                                        !is_burst_reject_r[mi] && !is_flush_reject_r[mi] &&
                                        !slv_poisoned[5];
            assign req_rd_local[mi] = (rs_state[mi] == RS_IDLE) && mr_arvalid[mi] &&
                                      ((dec_slv_r[mi] == `XBAR_SLV_NONE) ||
                                       is_burst_reject_r[mi] || is_poisoned_dec_r[mi] ||
                                       is_flush_reject_r[mi]);
        end
    endgenerate

    // Fix 2 watchdog activity/fire wires (read side).  A read slot is
    // "active" while it has an AR pending on the real slave or is
    // waiting on R data/RLAST.  RS_SEND_RLOCAL is already local-only.
    //
    // rs_wd_fire is additionally suppressed:
    //   - while slv_flush is asserted (CRITICAL fix part i: frozen —
    //     the flush-abort path below handles the slot instead, without
    //     poisoning);
    //   - IMPORTANT 3: on any cycle with an active R handshake for this
    //     slot (mr_rvalid_slv[mi] && mr_rready[mi]).  Without this, a
    //     non-RLAST beat handshaking on the exact fire cycle increments
    //     rs_beats_done (see the counter-update loop below) via a
    //     SEPARATE non-blocking write that hasn't taken effect yet when
    //     rs_wd_fire's "remaining beats" arithmetic reads rs_beats_done
    //     THIS cycle — the fire branch would then double-count that
    //     beat (once for real, once synthesized locally), delivering
    //     len+2 total beats to the master.  Suppressing fire on any
    //     handshake cycle (RLAST included, though that case is already
    //     handled by the normal-completion branch's priority) sidesteps
    //     the race entirely; firing is simply delayed to the next
    //     non-handshake cycle.
    wire rs_active [0:NR-1];
    wire rs_wd_fire[0:NR-1];
    // Fix 2 flush-abort trigger (CRITICAL fix part ii), read-side twin
    // of ws_flush_abort — uses is_flush_domain_slv(), NOT
    // is_lite_only_slv() (see that function's comment).
    wire rs_flush_abort[0:NR-1];
    wire rs_r_abandon [0:NR-1];
    wire rs_r_slv_abandon [0:NR-1];
    generate
        for (mi = 0; mi < NR; mi = mi + 1) begin : gen_rs_wd
            assign rs_active[mi] = (rs_state[mi] == RS_WAIT_SLV_AR) || (rs_state[mi] == RS_WAIT_R);
            assign rs_wd_fire[mi] = ENABLE_WD && rs_active[mi] &&
                                     (rs_wd_cnt[mi] == ((rs_slv[mi] == `XBAR_SLV_IO) ? WD_MAX_CNT_S1
                                                                                     : WD_MAX_CNT)) &&
                                     !slv_flush &&
                                     !(mr_rvalid_slv[mi] && mr_rready[mi]);
            // Read-side twin of the S1 owner-gone abort -- see the ws_flush_abort note.
            // Slot 0 reads are always the CPU LSU (the boot FSM never reads), so
            // `cpu_held_in_reset` alone is the owner-gone condition, matching
            // rs_release_slot's existing slot-0 silent-discard predicate exactly.
            assign rs_flush_abort[mi] = rs_active[mi] &&
                                        ((slv_flush && is_flush_domain_slv(rs_slv[mi])) ||
                                         ((mi == 0) && (rs_slv[mi] == `XBAR_SLV_IO) &&
                                          cpu_held_in_reset));
            // ── rs_r_abandon: the READ-side twin of ws_b_abandon ────────
            //
            // 2026-09-17 (backlog item 5).  The header's abandonment audit
            // claims "the read side never had this shape: rs_wd_fire
            // suppresses on an actual RVALID&&RREADY handshake".  That is
            // true of RS_WAIT_R, where the SLAVE produces the data.  It is
            // NOT true of RS_SEND_RLOCAL, where the CROSSBAR produces it
            // and the MASTER is the one that has to take it:
            //
            //   * RS_SEND_RLOCAL is deliberately excluded from rs_active
            //     ("already local-only"), so rs_wd_cnt is held at zero and
            //     rs_wd_fire can never fire there;
            //   * mr_rvalid_loc[] is asserted unconditionally from the
            //     state, so the tail is offered forever;
            //   * and nothing else ever leaves the state.
            //
            // So a master that walks away from a local R tail pins its read
            // slot at RS_SEND_RLOCAL permanently and can never read again.
            // This is exactly the failure task #219 fixed on the B channel,
            // produced by exactly the same master: axi_narrow_to_wide's own
            // abandonment watchdog force-clears ar_valid_q, which takes its
            // w_rready low and keeps it low -- and the JTAG-AXI / XDMA host
            // on M1 (read slot 1) sits behind one of those.  Slot 2 (CPU
            // instruction fetch) is equally exposed: it is on cpu_rst.
            //
            // The deadline and the trade are the B channel's, verbatim: a
            // tail unaccepted for 2^B_HOLD_LOG2 cycles (1024 by default,
            // ~10 us at 100 MHz -- orders of magnitude more than any live
            // master needs to raise RREADY) means gone, not slow, and the
            // slot is returned to RS_IDLE with NO poison (the slave is
            // healthy and was never even involved).  Withdrawing an
            // unaccepted RVALID is the same deliberate protocol deviation
            // ws_b_abandon already makes on BVALID in this module, for the
            // same reason and under the same evidence.
            //
            // TIMING: every term is a REGISTER, like ws_b_abandon's, so no
            // new combinational mr_rready -> mr_rvalid path is created;
            // RVALID falls the cycle AFTER the deadline, never in response
            // to RREADY within a cycle.
            assign rs_r_abandon[mi] = (rs_state[mi] == RS_SEND_RLOCAL) &&
                                      (rs_rhold_cnt[mi] == B_HOLD_MAX) &&
                                      !slv_flush;
            // Backlog item 4: the SLAVE-sourced counterpart.  Same evidence
            // and same deadline as ws_b_abandon, applied to a beat the
            // slave is presenting at this slot that the owning master has
            // not taken for 2^B_HOLD_LOG2 cycles.  Every term is a REGISTER
            // except slv_flush, deliberately and for exactly the reason
            // ws_b_abandon states: this wire reaches s*_rready, and writing
            // it in terms of mr_rready would create a new combinational
            // s*_rvalid -> s*_rready path at the slave port.
            assign rs_r_slv_abandon[mi] = (rs_state[mi] == RS_WAIT_R) &&
                                          (rs_rslv_hold_cnt[mi] == B_HOLD_MAX) &&
                                          !slv_flush;
        end
    endgenerate

    // IMPORTANT 2b fix: remaining-beats arithmetic in 9 bits.  8-bit
    // (rs_len+1)-rs_beats_done wraps len==255,beats_done==0 to 0 — the
    // SAME encoding the file already (deliberately) uses to mean "256"
    // via the WS/RS beat-counter's "==1" terminal check, but my
    // "clamp-to-1-if-computed-zero" safety net (RS_WAIT_R fire below,
    // for the "slave delivered everything but never asserted RLAST"
    // case) can't tell that apart from a GENUINE zero-remaining in only
    // 8 bits.  9 bits make 256 a distinct, non-wrapped value so the
    // zero-check is unambiguous; the STORED value (rs_bcnt, 8 bits) is
    // still the low 8 bits of this, preserving the existing
    // wraps-to-0-means-256 convention.
    wire [8:0] rs_wd_remaining [0:NR-1];
    wire [8:0] rs_release_remaining [0:NR-1];
    generate
        for (mi = 0; mi < NR; mi = mi + 1) begin : gen_rs_wd_remaining
            assign rs_wd_remaining[mi] =
                ({1'b0, rs_len[mi]} + 9'd1) - {1'b0, rs_beats_done[mi]};
            // A flush may coincide with a non-RLAST handshake.  Account for
            // that just-accepted real beat before constructing the local
            // SLVERR tail; watchdog release is handshake-suppressed, so this
            // expression is also identical to rs_wd_remaining there.
            assign rs_release_remaining[mi] = rs_wd_remaining[mi] -
                ((mr_rvalid_slv[mi] && mr_rready[mi]) ? 9'd1 : 9'd0);
        end
    endgenerate

    // Shared "abort this read slot and answer locally" body, read-side
    // twin of ws_release_slot — see its comment for the rationale
    // (single source of truth for the six-way slave dispatch, shared by
    // the watchdog and slv_flush abort paths).  `clear_arvalid` is 1
    // only for the RS_WAIT_SLV_AR call site (AR may still be
    // outstanding on the real slave); by RS_WAIT_R the AR has already
    // been accepted so there is nothing left to clear.
    task rs_release_slot;
        input [SLOT_W-1:0] slot;
        input [SSEL_W-1:0] slv_sel;
        input               clear_arvalid;
        input               poison;
        begin
            case (slv_sel)
                `XBAR_SLV_DDR: begin
                    if (clear_arvalid) s_arvalid_r[0] <= 1'b0;
                    if (poison)        slv_r_poisoned[0] <= 1'b1;
                end
                `XBAR_SLV_IO: begin
                    if (clear_arvalid) s_arvalid_r[1] <= 1'b0;
                    if (poison)        slv_r_poisoned[1] <= 1'b1;
                end
                `XBAR_SLV_DMA: begin
                    if (clear_arvalid) s_arvalid_r[2] <= 1'b0;
                    if (poison)        slv_r_poisoned[2] <= 1'b1;
                end
                `XBAR_SLV_VRAM: begin
                    if (clear_arvalid) s_arvalid_r[3] <= 1'b0;
                    if (poison)        slv_r_poisoned[3] <= 1'b1;
                end
                `XBAR_SLV_DAFB: begin
                    if (clear_arvalid) s_arvalid_r[4] <= 1'b0;
                    if (poison)        slv_r_poisoned[4] <= 1'b1;
                end
                default: begin // XBAR_SLV_SDJTAG
                    if (clear_arvalid) s_arvalid_r[5] <= 1'b0;
                    if (poison)        slv_r_poisoned[5] <= 1'b1;
                end
            endcase
            // ⛔ REMOVED 2026-09-17 — the slot-0 SILENT DISCARD.  It used to read:
            //
            //     if ((slot == 0) && cpu_held_in_reset) rs_state[slot] <= RS_IDLE;
            //
            // justified as "slot 0 reads are always CPU LSU; while cpu_held_in_reset
            // that requester no longer exists, so discard silently rather than present
            // a local R nobody is waiting for."
            //
            // THAT PREMISE IS NO LONGER TRUE, and the consequence was a board that
            // needed a re-flash.  The CPU socket gained `AxiReadResetAbsorber`
            // (cpu040 src/main/scala/m68k040/socket/) specifically to be the thing
            // that waits: it counts reads the SoC has accepted, it lives in a
            // `resetKind = BOOT` domain so that `cpu_rst` CANNOT clear it (that is the
            // whole point — the reset is what destroys the core's own trackers), and
            // it refuses to issue any new AR until the count returns to zero.
            //
            // So a read discarded here leaves that counter stuck forever.  The core
            // then never issues another AR — not even the reset-vector fetch —
            // `AxiDMerge` sits on a read grant with no progress, and ~20 s later D20's
            // bounded-grant watchdog halts the core with ARBITER_WEDGE.  Nothing
            // recovers it: NO reset in this system clears a BOOT-domain register, so
            // neither the JTAG pulse nor `vio-hard-reset` nor the board cold reset
            // helps.  Only re-configuring the FPGA does.  That is the measured
            // "a REPL `reset` kills the board and only reprogramming brings it back",
            // including the detail that `halt-kind` reads 0 on the FIRST probe after
            // the reset and 0x4 only on a later one — the D20 bound has to expire.
            //
            // Same wiring-level shape as the sd_scsi_bridge/sd_ctrl reset-pairing bug:
            // two modules each correct in isolation, each assuming something about the
            // other's reset behaviour, and no unit testbench able to see it.
            //
            // The fix is simply to stop making an exception: EVERY abandoned read now
            // takes the local SLVERR tail below, which restores this module's own
            // documented conformance claim (header: "a well-formed SLVERR instead of
            // nothing at all") that this branch was the last violation of.
            //
            // Safe in both reset postures, which is why no guard replaces it:
            //   * while `cpu_held_in_reset` is asserted, `cpu_d_rready_xbar` is forced
            //     high (fpga_top_cpu.vh), so the tail is consumed immediately;
            //   * after it deasserts, the absorber is by construction still armed for
            //     this very transaction, and `axi_d.rready := dm.r.ready || absorbD`
            //     is therefore high.
            // The slot cannot be re-armed underneath the tail either: it stays out of
            // RS_IDLE until the tail's RLAST goes out, so no later read can consume it.
            begin
                rs_state[slot]         <= RS_SEND_RLOCAL;
                rs_local_rsp[slot]     <= `AXI_RESP_SLVERR;
                rs_open_bus_zero[slot] <= 1'b1;
                // IMPORTANT 2b fix: 9-bit-computed remaining beats (see
                // rs_wd_remaining above), clamped to a minimum of 1 so
                // the master always observes a terminating RLAST even
                // in the RS_WAIT_R "every data beat already arrived but
                // the slave never asserted RLAST" case.  At
                // RS_WAIT_SLV_AR, rs_beats_done[slot] is always 0
                // (reset whenever rs_state!=RS_WAIT_R), so this
                // naturally reduces to the full rs_len[slot]+1.
                rs_bcnt[slot] <= (rs_release_remaining[slot] == 9'd0)
                                 ? 8'd1 : rs_release_remaining[slot][7:0];
            end
        end
    endtask

    function [SLOT_W-1:0] rr_pick_r;
        input [SLOT_W-1:0] rr_base;
        input req0, req1, req2, req3;
        reg [SLOT_W-1:0] c [0:3];
        reg r [0:3];
        integer k;
        reg done;
        begin
            c[0] = rr_base;
            c[1] = (rr_base + 2'd1);
            c[2] = (rr_base + 2'd2);
            c[3] = (rr_base + 2'd3);
            rr_pick_r = rr_base;
            done = 1'b0;
            for (k = 0; k < 4; k = k + 1) begin
                r[k] = (c[k] == 2'd0 && req0) || (c[k] == 2'd1 && req1) ||
                       (c[k] == 2'd2 && req2) || (c[k] == 2'd3 && req3);
                if (r[k] && !done) begin
                    rr_pick_r = c[k];
                    done = 1'b1;
                end
            end
        end
    endfunction

    wire [SLOT_W-1:0] ar_win_s0 = rr_pick_r(sr_rr_ptr[0], req_rd_ddr [0], req_rd_ddr [1], req_rd_ddr [2], req_rd_ddr [3]);
    wire [SLOT_W-1:0] ar_win_s1 = rr_pick_r(sr_rr_ptr[1], req_rd_io  [0], req_rd_io  [1], req_rd_io  [2], req_rd_io  [3]);
    wire [SLOT_W-1:0] ar_win_s2 = rr_pick_r(sr_rr_ptr[2], req_rd_dma [0], req_rd_dma [1], req_rd_dma [2], req_rd_dma [3]);
    wire [SLOT_W-1:0] ar_win_s3 = rr_pick_r(sr_rr_ptr[3], req_rd_vram[0], req_rd_vram[1], req_rd_vram[2], req_rd_vram[3]);
    wire [SLOT_W-1:0] ar_win_s4 = rr_pick_r(sr_rr_ptr[4], req_rd_dafb[0], req_rd_dafb[1], req_rd_dafb[2], req_rd_dafb[3]);
    wire [SLOT_W-1:0] ar_win_s5 = rr_pick_r(sr_rr_ptr[5], req_rd_sd_jtag[0], req_rd_sd_jtag[1], req_rd_sd_jtag[2], req_rd_sd_jtag[3]);
    wire              any_req_rd_s0 = req_rd_ddr [0] | req_rd_ddr [1] | req_rd_ddr [2] | req_rd_ddr [3];
    wire              any_req_rd_s1 = req_rd_io  [0] | req_rd_io  [1] | req_rd_io  [2] | req_rd_io  [3];
    wire              any_req_rd_s2 = req_rd_dma [0] | req_rd_dma [1] | req_rd_dma [2] | req_rd_dma [3];
    wire              any_req_rd_s3 = req_rd_vram[0] | req_rd_vram[1] | req_rd_vram[2] | req_rd_vram[3];
    wire              any_req_rd_s4 = req_rd_dafb[0] | req_rd_dafb[1] | req_rd_dafb[2] | req_rd_dafb[3];
    wire              any_req_rd_s5 = req_rd_sd_jtag[0] | req_rd_sd_jtag[1] | req_rd_sd_jtag[2] | req_rd_sd_jtag[3];

    wire [SLOT_W-1:0] s0_rtgt = s0_rid[XID_WIDTH-1 -: SLOT_W];
    wire [SLOT_W-1:0] s1_rtgt = s1_rid[XID_WIDTH-1 -: SLOT_W];
    wire [SLOT_W-1:0] s2_rtgt = s2_rid[XID_WIDTH-1 -: SLOT_W];
    wire [SLOT_W-1:0] s3_rtgt = s3_rid[XID_WIDTH-1 -: SLOT_W];
    wire [SLOT_W-1:0] s4_rtgt = s4_rid[XID_WIDTH-1 -: SLOT_W];
    wire [SLOT_W-1:0] s5_rtgt = s5_rid[XID_WIDTH-1 -: SLOT_W];
    wire       s0_r_valid_slot [0:NR-1];
    wire       s1_r_valid_slot [0:NR-1];
    wire       s2_r_valid_slot [0:NR-1];
    wire       s3_r_valid_slot [0:NR-1];
    wire       s4_r_valid_slot [0:NR-1];
    wire       s5_r_valid_slot [0:NR-1];
    generate
        for (mi = 0; mi < NR; mi = mi + 1) begin : gen_rroute
            assign s0_r_valid_slot[mi] = s0_rvalid && (s0_rtgt == mi[SLOT_W-1:0]);
            assign s1_r_valid_slot[mi] = s1_rvalid && (s1_rtgt == mi[SLOT_W-1:0]);
            assign s2_r_valid_slot[mi] = s2_rvalid && (s2_rtgt == mi[SLOT_W-1:0]);
            assign s3_r_valid_slot[mi] = s3_rvalid && (s3_rtgt == mi[SLOT_W-1:0]);
            assign s4_r_valid_slot[mi] = s4_rvalid && (s4_rtgt == mi[SLOT_W-1:0]);
            assign s5_r_valid_slot[mi] = s5_rvalid && (s5_rtgt == mi[SLOT_W-1:0]);
        end
    endgenerate

    reg  mr_arready_r [0:NR-1];
    assign mr_arready[0] = mr_arready_r[0];
    assign mr_arready[1] = mr_arready_r[1];
    assign mr_arready[2] = mr_arready_r[2];
    assign mr_arready[3] = mr_arready_r[3];

    wire                 mr_rvalid_slv [0:NR-1];
    wire                 mr_r_here     [0:NR-1];
    wire [DATA_WIDTH-1:0] mr_rdata_slv [0:NR-1];
    wire [1:0]           mr_rresp_slv  [0:NR-1];
    wire                 mr_rlast_slv  [0:NR-1];
    wire                 mr_rvalid_loc [0:NR-1];
    wire [1:0]           mr_rresp_loc  [0:NR-1];
    wire                 mr_rlast_loc  [0:NR-1];
    generate
        for (mi = 0; mi < NR; mi = mi + 1) begin : gen_rmux
            // mr_r_here[mi] -- "this slot's slave is presenting a beat tagged
            // for this slot", independent of the slot's state.  Factored out
            // of mr_rvalid_slv (zero logic change: mr_rvalid_slv is still
            // exactly the RS_WAIT_R-qualified version) so RS_DRAIN_R can use
            // the same decode to spot its RLAST -- see backlog item 4.
            assign mr_r_here[mi] =
                (((rs_slv[mi] == `XBAR_SLV_DDR ) && s0_r_valid_slot[mi]) ||
                 ((rs_slv[mi] == `XBAR_SLV_IO  ) && s1_r_valid_slot[mi]) ||
                 ((rs_slv[mi] == `XBAR_SLV_DMA ) && s2_r_valid_slot[mi]) ||
                 ((rs_slv[mi] == `XBAR_SLV_VRAM) && s3_r_valid_slot[mi]) ||
                 ((rs_slv[mi] == `XBAR_SLV_DAFB) && s4_r_valid_slot[mi]) ||
                 ((rs_slv[mi] == `XBAR_SLV_SDJTAG) && s5_r_valid_slot[mi]));
            assign mr_rvalid_slv[mi] = (rs_state[mi] == RS_WAIT_R) && mr_r_here[mi];
            assign mr_rdata_slv [mi] = (rs_slv[mi] == `XBAR_SLV_DDR ) ? s0_rdata :
                                        (rs_slv[mi] == `XBAR_SLV_IO  ) ? s1_rdata :
                                        (rs_slv[mi] == `XBAR_SLV_DMA ) ? s2_rdata :
                                        (rs_slv[mi] == `XBAR_SLV_VRAM) ? vram_swap_words(s3_rdata) :
                                        (rs_slv[mi] == `XBAR_SLV_DAFB) ? s4_rdata : s5_rdata;
            assign mr_rresp_slv [mi] = (rs_slv[mi] == `XBAR_SLV_DDR ) ? s0_rresp :
                                        (rs_slv[mi] == `XBAR_SLV_IO  ) ? s1_rresp :
                                        (rs_slv[mi] == `XBAR_SLV_DMA ) ? s2_rresp :
                                        (rs_slv[mi] == `XBAR_SLV_VRAM) ? s3_rresp :
                                        (rs_slv[mi] == `XBAR_SLV_DAFB) ? s4_rresp : s5_rresp;
            assign mr_rlast_slv [mi] = (rs_slv[mi] == `XBAR_SLV_DDR ) ? s0_rlast :
                                        (rs_slv[mi] == `XBAR_SLV_IO  ) ? s1_rlast :
                                        (rs_slv[mi] == `XBAR_SLV_DMA ) ? s2_rlast :
                                        (rs_slv[mi] == `XBAR_SLV_VRAM) ? s3_rlast :
                                        (rs_slv[mi] == `XBAR_SLV_DAFB) ? s4_rlast : s5_rlast;
            assign mr_rvalid_loc[mi] = (rs_state[mi] == RS_SEND_RLOCAL);
            assign mr_rresp_loc [mi] = rs_local_rsp[mi];
            assign mr_rlast_loc [mi] = (rs_bcnt[mi] == 8'd1);
        end
    endgenerate

    assign mr_rvalid[0] = mr_rvalid_slv[0] | mr_rvalid_loc[0];
    assign mr_rvalid[1] = mr_rvalid_slv[1] | mr_rvalid_loc[1];
    assign mr_rvalid[2] = mr_rvalid_slv[2] | mr_rvalid_loc[2];
    assign mr_rvalid[3] = mr_rvalid_slv[3] | mr_rvalid_loc[3];
    // Unmapped/local-response reads: open-bus value is uniform 0x00000000
    // for ALL apertures (RAM-OOR, ROM-OOR, I/O gap), matching MAME's
    // default `set_unmap_value = 0` which `macquadra700.cpp` never
    // overrides.  The `rs_open_bus_zero[mi]` flag is therefore always
    // 1 today; kept as a per-master reg so a future per-region policy
    // change is a one-line edit at the AR-latch site.
    assign mr_rdata [0] = mr_rvalid_slv[0] ? mr_rdata_slv[0] :
                          (rs_open_bus_zero[0] ? {DATA_WIDTH{1'b0}}
                                               : {DATA_WIDTH{1'b1}});
    assign mr_rdata [1] = mr_rvalid_slv[1] ? mr_rdata_slv[1] :
                          (rs_open_bus_zero[1] ? {DATA_WIDTH{1'b0}}
                                               : {DATA_WIDTH{1'b1}});
    assign mr_rdata [2] = mr_rvalid_slv[2] ? mr_rdata_slv[2] :
                          (rs_open_bus_zero[2] ? {DATA_WIDTH{1'b0}}
                                               : {DATA_WIDTH{1'b1}});
    assign mr_rdata [3] = mr_rvalid_slv[3] ? mr_rdata_slv[3] :
                          (rs_open_bus_zero[3] ? {DATA_WIDTH{1'b0}}
                                               : {DATA_WIDTH{1'b1}});
    assign mr_rresp [0] = mr_rvalid_slv[0] ? mr_rresp_slv[0] : mr_rresp_loc[0];
    assign mr_rresp [1] = mr_rvalid_slv[1] ? mr_rresp_slv[1] : mr_rresp_loc[1];
    assign mr_rresp [2] = mr_rvalid_slv[2] ? mr_rresp_slv[2] : mr_rresp_loc[2];
    assign mr_rresp [3] = mr_rvalid_slv[3] ? mr_rresp_slv[3] : mr_rresp_loc[3];
    assign mr_rlast [0] = mr_rvalid_slv[0] ? mr_rlast_slv[0] : mr_rlast_loc[0];
    assign mr_rlast [1] = mr_rvalid_slv[1] ? mr_rlast_slv[1] : mr_rlast_loc[1];
    assign mr_rlast [2] = mr_rvalid_slv[2] ? mr_rlast_slv[2] : mr_rlast_loc[2];
    assign mr_rlast [3] = mr_rvalid_slv[3] ? mr_rlast_slv[3] : mr_rlast_loc[3];
    assign mr_rid   [0] = rs_mid[0];
    assign mr_rid   [1] = rs_mid[1];
    assign mr_rid   [2] = rs_mid[2];
    assign mr_rid   [3] = rs_mid[3];

    wire s0_rready_per [0:NR-1];
    wire s1_rready_per [0:NR-1];
    wire s2_rready_per [0:NR-1];
    wire s3_rready_per [0:NR-1];
    wire s4_rready_per [0:NR-1];
    wire s5_rready_per [0:NR-1];
    generate
        for (mi = 0; mi < NR; mi = mi + 1) begin : gen_rready
            assign s0_rready_per[mi] = (rs_slv[mi] == `XBAR_SLV_DDR) && s0_r_valid_slot[mi] &&
                                        (((rs_state[mi] == RS_WAIT_R) && mr_rready[mi]) ||
                                         (rs_state[mi] == RS_DRAIN_R));
            assign s1_rready_per[mi] = (rs_slv[mi] == `XBAR_SLV_IO) && s1_r_valid_slot[mi] &&
                                        (((rs_state[mi] == RS_WAIT_R) && mr_rready[mi]) ||
                                         (rs_state[mi] == RS_DRAIN_R));
            assign s2_rready_per[mi] = (rs_slv[mi] == `XBAR_SLV_DMA) && s2_r_valid_slot[mi] &&
                                        (((rs_state[mi] == RS_WAIT_R) && mr_rready[mi]) ||
                                         (rs_state[mi] == RS_DRAIN_R));
            // Quarantine term folded in here (backlog item 2): RREADY is
            // forced ONLY for a beat whose RID tag is quarantined, never
            // for a live slot's beat.  One extra OR input inside a cone
            // that was already >6 inputs wide; the OR4 below is untouched.
            assign s3_rready_per[mi] = s3_r_valid_slot[mi] &&
                                       (s3_rq_v[mi] ||
                                        ((rs_slv[mi] == `XBAR_SLV_VRAM) &&
                                         (((rs_state[mi] == RS_WAIT_R) && mr_rready[mi]) ||
                                          (rs_state[mi] == RS_DRAIN_R))));
            assign s4_rready_per[mi] = (rs_slv[mi] == `XBAR_SLV_DAFB) && s4_r_valid_slot[mi] &&
                                        (((rs_state[mi] == RS_WAIT_R) && mr_rready[mi]) ||
                                         (rs_state[mi] == RS_DRAIN_R));
            assign s5_rready_per[mi] = (rs_slv[mi] == `XBAR_SLV_SDJTAG) && s5_r_valid_slot[mi] &&
                                        (((rs_state[mi] == RS_WAIT_R) && mr_rready[mi]) ||
                                         (rs_state[mi] == RS_DRAIN_R));
        end
    endgenerate
    assign s0_rready = s0_rready_per[0] | s0_rready_per[1] | s0_rready_per[2] | s0_rready_per[3];
    assign s1_rready = s1_rready_per[0] | s1_rready_per[1] | s1_rready_per[2] | s1_rready_per[3];
    assign s2_rready = s2_rready_per[0] | s2_rready_per[1] | s2_rready_per[2] | s2_rready_per[3];
    assign s3_rready = s3_rready_per[0] | s3_rready_per[1] |
                       s3_rready_per[2] | s3_rready_per[3];
    assign s4_rready = s4_rready_per[0] | s4_rready_per[1] | s4_rready_per[2] | s4_rready_per[3];
    assign s5_rready = s5_rready_per[0] | s5_rready_per[1] | s5_rready_per[2] | s5_rready_per[3];

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < NR; i = i + 1) begin
                rs_state    [i] <= RS_IDLE;
                rs_slv      [i] <= `XBAR_SLV_NONE;
                rs_mid      [i] <= {ID_WIDTH{1'b0}};
                rs_len      [i] <= 8'h0;
                rs_bcnt     [i] <= 8'h0;
                rs_wd_cnt   [i] <= {WD_CNT_W{1'b0}};
                rs_beats_done[i] <= 8'h0;
                rs_local_rsp[i] <= `AXI_RESP_OKAY;
                rs_open_bus_zero[i] <= 1'b0;
                rs_rhold_cnt[i] <= {B_HOLD_CNT_W{1'b0}};
                rs_rslv_hold_cnt[i] <= {B_HOLD_CNT_W{1'b0}};
                mr_arready_r[i] <= 1'b0;
            end
            for (i = 0; i < NS; i = i + 1) begin
                sr_rr_ptr  [i] <= {SLOT_W{1'b0}};
                s_arid_r   [i] <= {XID_WIDTH{1'b0}};
                s_araddr_r [i] <= 32'h0;
                s_arlen_r  [i] <= 8'h0;
                s_arsize_r [i] <= 3'h0;
                s_arburst_r[i] <= 2'h0;
                s_arvalid_r[i] <= 1'b0;
                slv_r_poisoned[i] <= 1'b0;
            end
            s3_rq_v              <= {NR{1'b0}};
            s3_flush_read_wd_cnt <= {WD_CNT_W{1'b0}};
        end else begin
            for (i = 0; i < NR; i = i + 1) mr_arready_r[i] <= 1'b0;

            // Retire a quarantined burst on ITS OWN RLAST (backlog item 2).
            // Guarded by the OLD s3_rq_v[i], which is also what forced
            // RREADY for this beat, so the handshake is guaranteed -- and a
            // slot being quarantined for the first time THIS cycle (old bit
            // 0) cannot be cleared by a beat the xbar did not consume.
            // Placed ahead of the per-slot FSM loop below so that a set in
            // the same cycle takes precedence over this clear.
            for (i = 0; i < NR; i = i + 1) begin
                if (s3_rq_v[i] && s3_r_valid_slot[i] && s3_rlast)
                    s3_rq_v[i] <= 1'b0;
            end

            if (s3_flush_read_active) begin
                // One shared watchdog, re-armed whenever ANY quarantined
                // burst retires -- so N stale bursts get N windows, not one.
                if (s3_rvalid && s3_rlast && s3_rq_v[s3_rtgt]) begin
                    s3_flush_read_wd_cnt <= {WD_CNT_W{1'b0}};
                end else if (s3_flush_read_wd_cnt == WD_MAX_CNT) begin
                    s3_rq_v              <= {NR{1'b0}};
                    s3_flush_read_wd_cnt <= {WD_CNT_W{1'b0}};
                    slv_r_poisoned[3] <= 1'b1;
                    // synthesis translate_off
                    $display("AXI_XBAR [%0t]: S3 flush read cleanup timed out; poisoning VRAM slave", $time);
                    // synthesis translate_on
                end else begin
                    s3_flush_read_wd_cnt <= s3_flush_read_wd_cnt +
                                            {{(WD_CNT_W-1){1'b0}}, 1'b1};
                end
            end else begin
                s3_flush_read_wd_cnt <= {WD_CNT_W{1'b0}};
            end

            // Fix 2: advance/hold each read slot's watchdog counter, and
            // track beats already delivered so a mid-burst timeout can
            // synthesize exactly the remaining beats (see RS_WAIT_R
            // watchdog-fire branch below).  CRITICAL fix part i: frozen
            // while slv_flush is asserted.  Saturates at WD_MAX_CNT
            // (does not wrap) when rs_wd_fire is suppressed for a
            // reason OTHER than "already fired this cycle" (IMPORTANT 3's
            // same-cycle-handshake suppression, or slv_flush) — see the
            // write-side ws_wd_cnt comment for why wrapping would be
            // wrong (silently resets the whole window).
            // RESET-WEDGE, corrected 2026-09-12: read-side twin -- see the write-side
            // ws_wd_cnt note above for the measurement that refuted the freeze.
            for (i = 0; i < NR; i = i + 1) begin
                if (!rs_active[i])
                    rs_wd_cnt[i] <= {WD_CNT_W{1'b0}};
                else if (!slv_flush &&
                         (rs_wd_cnt[i] != ((rs_slv[i] == `XBAR_SLV_IO) ? WD_MAX_CNT_S1
                                                                       : WD_MAX_CNT)))
                    rs_wd_cnt[i] <= rs_wd_cnt[i] + {{(WD_CNT_W-1){1'b0}}, 1'b1};
                if (rs_state[i] != RS_WAIT_R)
                    rs_beats_done[i] <= 8'h0;
                else if (mr_rvalid_slv[i] && mr_rready[i])
                    rs_beats_done[i] <= rs_beats_done[i] + 8'd1;
                // Backlog item 5: R-hold grace counter, the exact twin of
                // ws_bhold_cnt.  Runs only while a LOCAL R beat is offered
                // at this slot and the master has NOT taken it; cleared the
                // moment either goes away, so an ordinary one-or-two-cycle
                // RREADY latency -- and a slow-but-live master working
                // through a 256-beat local burst, which re-arms on every
                // beat -- never gets anywhere near B_HOLD_MAX.  Frozen
                // under slv_flush and saturating rather than wrapping, for
                // the same reasons ws_bhold_cnt is.
                if (!mr_rvalid_loc[i] || mr_rready[i])
                    rs_rhold_cnt[i] <= {B_HOLD_CNT_W{1'b0}};
                else if (!slv_flush && (rs_rhold_cnt[i] != B_HOLD_MAX))
                    rs_rhold_cnt[i] <= rs_rhold_cnt[i] +
                                       {{(B_HOLD_CNT_W-1){1'b0}}, 1'b1};
                // Backlog item 4: the slave-sourced twin.  Runs only while
                // this slot's slave is presenting a beat FOR THIS SLOT in
                // RS_WAIT_R and the owning master has not taken it; cleared
                // the moment either goes away, so ordinary backpressure --
                // and a slow-but-live master working through a 256-beat
                // burst, which re-arms on every beat -- never gets near
                // B_HOLD_MAX.  Frozen under slv_flush and saturating, like
                // every other counter in this file.
                if (!mr_rvalid_slv[i] || mr_rready[i])
                    rs_rslv_hold_cnt[i] <= {B_HOLD_CNT_W{1'b0}};
                else if (!slv_flush && (rs_rslv_hold_cnt[i] != B_HOLD_MAX))
                    rs_rslv_hold_cnt[i] <= rs_rslv_hold_cnt[i] +
                                           {{(B_HOLD_CNT_W-1){1'b0}}, 1'b1};
            end

            // slv_r_poisoned cleared on slv_flush's rising edge (CRITICAL
            // fix part iv) — see the matching write-side comment.
            if (slv_flush_rise) begin
                for (i = 0; i < NS; i = i + 1) slv_r_poisoned[i] <= 1'b0;
            end

            if (s_arvalid_r[0] && s0_arready) s_arvalid_r[0] <= 1'b0;
            if (s_arvalid_r[1] && s1_arready) s_arvalid_r[1] <= 1'b0;
            if (s_arvalid_r[2] && s2_arready) s_arvalid_r[2] <= 1'b0;
            if (s_arvalid_r[3] && s3_arready) s_arvalid_r[3] <= 1'b0;
            if (s_arvalid_r[4] && s4_arready) s_arvalid_r[4] <= 1'b0;
            if (s_arvalid_r[5] && s5_arready) s_arvalid_r[5] <= 1'b0;

            for (i = 0; i < NR; i = i + 1) begin
                case (rs_state[i])
                    RS_WAIT_SLV_AR: begin
                        // Advance on the slave AR handshake ITSELF
                        // (s_arvalid_r && arready) rather than observing
                        // the registered clear (!s_arvalid_r) one cycle
                        // later — entering RS_WAIT_R a cycle earlier so a
                        // fast slave's first R beat is accepted without a
                        // dead cycle.  Entering this state always sets
                        // s_arvalid_r=1 at the same arb edge, so the
                        // handshake is the unique exit event; ID-tagged R
                        // routing (s?_rtgt) is unaffected.
                        // NOTE: needs a timing-validated synth pass
                        // before being trusted on real HW (sN_arready now
                        // feeds rs_state next-state logic; it already fed
                        // the s_arvalid_r clear term).
                        if (rs_slv[i] == `XBAR_SLV_DDR) begin
                            if ((s_arid_r[0][XID_WIDTH-1 -: SLOT_W] == i[SLOT_W-1:0]) && s_arvalid_r[0] && s0_arready)
                                rs_state[i] <= RS_WAIT_R;
                            else if (rs_flush_abort[i]) begin
                                rs_release_slot(i[SLOT_W-1:0], rs_slv[i], 1'b1, 1'b0);
                            end else if (rs_wd_fire[i]) begin
                                rs_release_slot(i[SLOT_W-1:0], rs_slv[i], 1'b1, 1'b1);
                            end
                        end else if (rs_slv[i] == `XBAR_SLV_IO) begin
                            if ((s_arid_r[1][XID_WIDTH-1 -: SLOT_W] == i[SLOT_W-1:0]) && s_arvalid_r[1] && s1_arready)
                                rs_state[i] <= RS_WAIT_R;
                            else if (rs_flush_abort[i]) begin
                                rs_release_slot(i[SLOT_W-1:0], rs_slv[i], 1'b1, 1'b0);
                            end else if (rs_wd_fire[i]) begin
                                rs_release_slot(i[SLOT_W-1:0], rs_slv[i], 1'b1, 1'b1);
                            end
                        end else if (rs_slv[i] == `XBAR_SLV_DMA) begin
                            if ((s_arid_r[2][XID_WIDTH-1 -: SLOT_W] == i[SLOT_W-1:0]) && s_arvalid_r[2] && s2_arready)
                                rs_state[i] <= RS_WAIT_R;
                            else if (rs_flush_abort[i]) begin
                                rs_release_slot(i[SLOT_W-1:0], rs_slv[i], 1'b1, 1'b0);
                            end else if (rs_wd_fire[i]) begin
                                rs_release_slot(i[SLOT_W-1:0], rs_slv[i], 1'b1, 1'b1);
                            end
                        end else if (rs_slv[i] == `XBAR_SLV_VRAM) begin
                            if ((s_arid_r[3][XID_WIDTH-1 -: SLOT_W] == i[SLOT_W-1:0]) && s_arvalid_r[3] && s3_arready) begin
                                if (S3_BACKEND_SURVIVES_FLUSH && rs_flush_abort[i]) begin
                                    s3_rq_v[i]           <= 1'b1;
                                    s3_flush_read_wd_cnt <= {WD_CNT_W{1'b0}};
                                    rs_release_slot(i[SLOT_W-1:0], rs_slv[i], 1'b0, 1'b0);
                                end else begin
                                    rs_state[i] <= RS_WAIT_R;
                                end
                            end
                            else if (rs_flush_abort[i]) begin
                                rs_release_slot(i[SLOT_W-1:0], rs_slv[i], 1'b1, 1'b0);
                            end else if (rs_wd_fire[i]) begin
                                rs_release_slot(i[SLOT_W-1:0], rs_slv[i], 1'b1, 1'b1);
                            end
                        end else if (rs_slv[i] == `XBAR_SLV_DAFB) begin
                            if ((s_arid_r[4][XID_WIDTH-1 -: SLOT_W] == i[SLOT_W-1:0]) && s_arvalid_r[4] && s4_arready)
                                rs_state[i] <= RS_WAIT_R;
                            else if (rs_flush_abort[i]) begin
                                rs_release_slot(i[SLOT_W-1:0], rs_slv[i], 1'b1, 1'b0);
                            end else if (rs_wd_fire[i]) begin
                                rs_release_slot(i[SLOT_W-1:0], rs_slv[i], 1'b1, 1'b1);
                            end
                        end else if (rs_slv[i] == `XBAR_SLV_SDJTAG) begin
                            if ((s_arid_r[5][XID_WIDTH-1 -: SLOT_W] == i[SLOT_W-1:0]) && s_arvalid_r[5] && s5_arready)
                                rs_state[i] <= RS_WAIT_R;
                            else if (rs_flush_abort[i]) begin
                                rs_release_slot(i[SLOT_W-1:0], rs_slv[i], 1'b1, 1'b0);
                            end else if (rs_wd_fire[i]) begin
                                rs_release_slot(i[SLOT_W-1:0], rs_slv[i], 1'b1, 1'b1);
                            end
                        end
                    end

                    RS_WAIT_R: begin
                        if (mr_rvalid_slv[i] && mr_rready[i] && mr_rlast_slv[i]) begin
                            rs_state[i] <= RS_IDLE;
                            if (rs_slv[i] == `XBAR_SLV_DDR)
                                sr_rr_ptr[0] <= i[SLOT_W-1:0] + 2'd1;
                            else if (rs_slv[i] == `XBAR_SLV_IO)
                                sr_rr_ptr[1] <= i[SLOT_W-1:0] + 2'd1;
                            else if (rs_slv[i] == `XBAR_SLV_DMA)
                                sr_rr_ptr[2] <= i[SLOT_W-1:0] + 2'd1;
                            else if (rs_slv[i] == `XBAR_SLV_VRAM)
                                sr_rr_ptr[3] <= i[SLOT_W-1:0] + 2'd1;
                            else if (rs_slv[i] == `XBAR_SLV_DAFB)
                                sr_rr_ptr[4] <= i[SLOT_W-1:0] + 2'd1;
                            else
                                sr_rr_ptr[5] <= i[SLOT_W-1:0] + 2'd1;
                        end else if (rs_flush_abort[i]) begin
                            // AR already accepted — nothing to clear on
                            // the AR channel.  No poison: the bridge is
                            // in reset, not wedged.
                            if (S3_BACKEND_SURVIVES_FLUSH &&
                                (rs_slv[i] == `XBAR_SLV_VRAM)) begin
                                s3_rq_v[i]           <= 1'b1;
                                s3_flush_read_wd_cnt <= {WD_CNT_W{1'b0}};
                            end
                            rs_release_slot(i[SLOT_W-1:0], rs_slv[i], 1'b0, 1'b0);
                        end else if (rs_r_slv_abandon[i]) begin
                            // Backlog item 4: the slave is producing and the
                            // MASTER has stopped taking.  Ordered ahead of
                            // rs_wd_fire because that path is for a wedged
                            // SLAVE and is the wrong answer here -- it would
                            // poison a healthy slave and still leave the
                            // stale burst blocking its read channel.
                            rs_state[i] <= RS_DRAIN_R;
                            // The terminating tail's beat count MUST be
                            // captured here, on the way in: `rs_beats_done` is
                            // cleared the moment rs_state leaves RS_WAIT_R, so
                            // by the time RS_DRAIN_R finishes there is nothing
                            // left to compute it from.  Same expression, and
                            // the same >=1 clamp, that rs_release_slot uses.
                            rs_bcnt[i] <= (rs_release_remaining[i] == 9'd0)
                                          ? 8'd1 : rs_release_remaining[i][7:0];
                            rs_local_rsp[i]     <= `AXI_RESP_SLVERR;
                            rs_open_bus_zero[i] <= 1'b1;
                            // synthesis translate_off
                            $display("AXI_XBAR [%0t]: slot-%0d read abandoned mid-burst by its master (slv=%0d); draining the rest of the burst at the slave, no poison",
                                     $time, i, rs_slv[i]);
                            // synthesis translate_on
                        end else if (rs_wd_fire[i]) begin
                            // AR already accepted by the slave — it's
                            // either stalled mid-burst or never asserted
                            // RLAST.  Poison + synthesize exactly the
                            // remaining beats.  Any late s0..s5_rvalid
                            // for this transaction is ignored: rs_state[i]
                            // is no longer RS_WAIT_R, and since the slave
                            // is poisoned no future AR ever re-arms
                            // rs_slv[i] to it, so mr_rvalid_slv[] can
                            // never re-match.
                            rs_release_slot(i[SLOT_W-1:0], rs_slv[i], 1'b0, 1'b1);
                        end
                    end

                    // Backlog item 4: consume the abandoned burst at the
                    // slave.  s*_rready_per[] drives the slave's RREADY from
                    // this state without consulting mr_rready, so every beat
                    // handshakes and the RLAST below always arrives.
                    RS_DRAIN_R: begin
                        if (slv_flush && is_flush_domain_slv(rs_slv[i])) begin
                            // The slave is being reset under us.  No poison
                            // -- same contract as rs_flush_abort.
                            //
                            // ⚠ CORRECTED 2026-09-18 (race audit).  This used
                            // to read `rs_state[i] <= RS_IDLE;`, justified as
                            // "it has forgotten the burst, so there is nothing
                            // left to drain".  BOTH halves of that were wrong,
                            // and they are the two defects this campaign is
                            // about, one state apart:
                            //
                            //  1. THE MASTER IS STILL OWED A RESPONSE.  This
                            //     was the ONLY exit in the read machine that
                            //     released a slot without one -- exactly what
                            //     the normal RS_DRAIN_R exit below was fixed
                            //     NOT to do, for exactly the reason written
                            //     there: the core's AxiReadResetAbsorber
                            //     decrements only on a bus-side RLAST, lives
                            //     in a BOOT domain no reset clears, and a
                            //     dropped response latches `absorbing`
                            //     forever -> ARBITER_WEDGE, reconfigure only.
                            //     Handing off to RS_SEND_RLOCAL costs nothing:
                            //     rs_bcnt / rs_local_rsp / rs_open_bus_zero
                            //     were all latched on the way INTO this state,
                            //     and rs_bcnt is already the right length
                            //     (beats consumed by the drain are not beats
                            //     delivered to the master, so rs_beats_done --
                            //     which is what rs_bcnt was computed from --
                            //     correctly did not count them).  If the master
                            //     really has walked away it will not take the
                            //     tail either, and rs_r_abandon retires it.
                            //
                            //  2. "FORGOTTEN THE BURST" IS FALSE FOR S3 UNDER
                            //     S3_BACKEND_SURVIVES_FLUSH.  In the shipping
                            //     VRAM_IN_DDR topology S3 is a route into the
                            //     always-live DDR mux/MIG path, which does NOT
                            //     reset on slv_flush -- that parameter exists
                            //     to say so.  Without the quarantine, the
                            //     remaining beats come back with s3_rready
                            //     driven by nothing (it is sourced only from
                            //     RS_WAIT_R, RS_DRAIN_R and s3_rq_v, and the
                            //     slot is now in none of them), blocking the
                            //     read channel S3 SHARES WITH S0 for every
                            //     master -- backlog item 2's failure mode (a),
                            //     reintroduced one state over, and it outlives
                            //     every reset the design has.  The RS_WAIT_R
                            //     flush-abort branch above already does this
                            //     correctly; this branch simply never was
                            //     brought into line with it.
                            //
                            // Negative control: scenario 49 in tb_axi_xbar.cpp
                            // (all three checks go red if either half of this
                            // is reverted).
                            if (S3_BACKEND_SURVIVES_FLUSH &&
                                (rs_slv[i] == `XBAR_SLV_VRAM)) begin
                                s3_rq_v[i]           <= 1'b1;
                                s3_flush_read_wd_cnt <= {WD_CNT_W{1'b0}};
                            end
                            rs_state[i] <= RS_SEND_RLOCAL;
                        end else if (mr_r_here[i] && mr_rlast_slv[i]) begin
                            // ⚠ NOT RS_IDLE.  Draining the slave frees its read
                            // channel, but the MASTER is still owed a
                            // terminating response, and this module's own
                            // "Abandonment always produces a response" audit is
                            // explicit that no transaction may be dropped
                            // without one.  Going straight to idle here was a
                            // real defect, and precisely the wedge this whole
                            // campaign is about:
                            //
                            //   ws_b_abandon may synthesize NOTHING because its
                            //   master is an axi_narrow_to_wide whose own
                            //   watchdog already answered its narrow side.  M0
                            //   (CPU data) and M2 (CPU fetch) have no such
                            //   adapter -- what waits on them is the core's
                            //   AxiReadResetAbsorber, and that decrements ONLY
                            //   on a bus-side RLAST.  Drop the response and its
                            //   count never returns to zero, `absorbing` stays
                            //   latched, no AR can ever issue, AxiDMerge holds a
                            //   grant and D20 halts with ARBITER_WEDGE -- which
                            //   no reset clears, because the absorber lives in a
                            //   BOOT domain on purpose.
                            //
                            // So the drain hands off to the ordinary local tail.
                            // If the master really has walked away it will not
                            // take that either, and rs_r_abandon retires it
                            // after its own window -- the two compose, and the
                            // slot is released either way.
                            rs_state[i] <= RS_SEND_RLOCAL;
                            if (rs_slv[i] == `XBAR_SLV_DDR)
                                sr_rr_ptr[0] <= i[SLOT_W-1:0] + 2'd1;
                            else if (rs_slv[i] == `XBAR_SLV_IO)
                                sr_rr_ptr[1] <= i[SLOT_W-1:0] + 2'd1;
                            else if (rs_slv[i] == `XBAR_SLV_DMA)
                                sr_rr_ptr[2] <= i[SLOT_W-1:0] + 2'd1;
                            else if (rs_slv[i] == `XBAR_SLV_VRAM)
                                sr_rr_ptr[3] <= i[SLOT_W-1:0] + 2'd1;
                            else if (rs_slv[i] == `XBAR_SLV_DAFB)
                                sr_rr_ptr[4] <= i[SLOT_W-1:0] + 2'd1;
                            else
                                sr_rr_ptr[5] <= i[SLOT_W-1:0] + 2'd1;
                        end
                    end

                    RS_SEND_RLOCAL: begin
                        if (mr_rvalid_loc[i] && mr_rready[i]) begin
                            if (rs_bcnt[i] == 8'd1) begin
                                rs_state[i] <= RS_IDLE;
                            end else begin
                                rs_bcnt[i] <= rs_bcnt[i] - 8'd1;
                            end
                        end else if (rs_r_abandon[i]) begin
                            // Backlog item 5: the master walked away from a
                            // tail the crossbar itself synthesized.  Nothing
                            // downstream is left inconsistent -- no slave was
                            // ever involved in a local response -- so the slot
                            // simply idles.  No poison: this says nothing
                            // about any slave's health.
                            rs_state[i] <= RS_IDLE;
                            // synthesis translate_off
                            $display("AXI_XBAR [%0t]: slot-%0d local R tail abandoned by its master (%0d beat(s) still owed); releasing the read slot, no poison",
                                     $time, i, rs_bcnt[i]);
                            // synthesis translate_on
                        end
                    end

                    default: ;
                endcase
            end

            if (any_req_rd_s0 && !s_arvalid_r[0]) begin
                s_arid_r   [0] <= {ar_win_s0, mr_arid[ar_win_s0]};
                s_araddr_r [0] <= ddr_flatten(mr_araddr_eff[ar_win_s0]);
                s_arlen_r  [0] <= mr_arlen  [ar_win_s0];
                s_arsize_r [0] <= mr_arsize [ar_win_s0];
                s_arburst_r[0] <= mr_arburst[ar_win_s0];
                s_arvalid_r[0] <= 1'b1;
                rs_state   [ar_win_s0] <= RS_WAIT_SLV_AR;
                rs_slv     [ar_win_s0] <= `XBAR_SLV_DDR;
                rs_mid     [ar_win_s0] <= mr_arid[ar_win_s0];
                rs_len     [ar_win_s0] <= mr_arlen[ar_win_s0];
                mr_arready_r[ar_win_s0] <= 1'b1;
            end

            if (any_req_rd_s1 && !s_arvalid_r[1]) begin
                s_arid_r   [1] <= {ar_win_s1, mr_arid[ar_win_s1]};
                s_araddr_r [1] <= mr_araddr_eff[ar_win_s1];
                s_arlen_r  [1] <= mr_arlen  [ar_win_s1];
                s_arsize_r [1] <= mr_arsize [ar_win_s1];
                s_arburst_r[1] <= mr_arburst[ar_win_s1];
                s_arvalid_r[1] <= 1'b1;
                rs_state   [ar_win_s1] <= RS_WAIT_SLV_AR;
                rs_slv     [ar_win_s1] <= `XBAR_SLV_IO;
                rs_mid     [ar_win_s1] <= mr_arid[ar_win_s1];
                rs_len     [ar_win_s1] <= mr_arlen[ar_win_s1];
                mr_arready_r[ar_win_s1] <= 1'b1;
            end

            if (any_req_rd_s2 && !s_arvalid_r[2]) begin
                s_arid_r   [2] <= {ar_win_s2, mr_arid[ar_win_s2]};
                s_araddr_r [2] <= mr_araddr_eff[ar_win_s2];
                s_arlen_r  [2] <= mr_arlen  [ar_win_s2];
                s_arsize_r [2] <= mr_arsize [ar_win_s2];
                s_arburst_r[2] <= mr_arburst[ar_win_s2];
                s_arvalid_r[2] <= 1'b1;
                rs_state   [ar_win_s2] <= RS_WAIT_SLV_AR;
                rs_slv     [ar_win_s2] <= `XBAR_SLV_DMA;
                rs_mid     [ar_win_s2] <= mr_arid[ar_win_s2];
                rs_len     [ar_win_s2] <= mr_arlen[ar_win_s2];
                mr_arready_r[ar_win_s2] <= 1'b1;
            end

            if (any_req_rd_s3 && !s3_flush_read_active && !s_arvalid_r[3]) begin
                s_arid_r   [3] <= {ar_win_s3, mr_arid[ar_win_s3]};
                // VRAM aperture: forward a zero-based byte offset.
                s_araddr_r [3] <= vram_flatten(mr_araddr_eff[ar_win_s3]);
                s_arlen_r  [3] <= mr_arlen  [ar_win_s3];
                s_arsize_r [3] <= mr_arsize [ar_win_s3];
                s_arburst_r[3] <= mr_arburst[ar_win_s3];
                s_arvalid_r[3] <= 1'b1;
                rs_state   [ar_win_s3] <= RS_WAIT_SLV_AR;
                rs_slv     [ar_win_s3] <= `XBAR_SLV_VRAM;
                rs_mid     [ar_win_s3] <= mr_arid[ar_win_s3];
                rs_len     [ar_win_s3] <= mr_arlen[ar_win_s3];
                mr_arready_r[ar_win_s3] <= 1'b1;
            end

            if (any_req_rd_s4 && !s_arvalid_r[4]) begin
                s_arid_r   [4] <= {ar_win_s4, mr_arid[ar_win_s4]};
                // DAFB register aperture: forward a zero-based byte offset.
                s_araddr_r [4] <= dafb_flatten(mr_araddr_eff[ar_win_s4]);
                s_arlen_r  [4] <= mr_arlen  [ar_win_s4];
                s_arsize_r [4] <= mr_arsize [ar_win_s4];
                s_arburst_r[4] <= mr_arburst[ar_win_s4];
                s_arvalid_r[4] <= 1'b1;
                rs_state   [ar_win_s4] <= RS_WAIT_SLV_AR;
                rs_slv     [ar_win_s4] <= `XBAR_SLV_DAFB;
                rs_mid     [ar_win_s4] <= mr_arid[ar_win_s4];
                rs_len     [ar_win_s4] <= mr_arlen[ar_win_s4];
                mr_arready_r[ar_win_s4] <= 1'b1;
            end

            if (any_req_rd_s5 && !s_arvalid_r[5]) begin
                s_arid_r   [5] <= {ar_win_s5, mr_arid[ar_win_s5]};
                s_araddr_r [5] <= sd_jtag_flatten(mr_araddr_eff[ar_win_s5]);
                s_arlen_r  [5] <= mr_arlen  [ar_win_s5];
                s_arsize_r [5] <= mr_arsize [ar_win_s5];
                s_arburst_r[5] <= mr_arburst[ar_win_s5];
                s_arvalid_r[5] <= 1'b1;
                rs_state   [ar_win_s5] <= RS_WAIT_SLV_AR;
                rs_slv     [ar_win_s5] <= `XBAR_SLV_SDJTAG;
                rs_mid     [ar_win_s5] <= mr_arid[ar_win_s5];
                rs_len     [ar_win_s5] <= mr_arlen[ar_win_s5];
                mr_arready_r[ar_win_s5] <= 1'b1;
            end

            // Unmapped reads: OKAY + open-bus rdata = 0x00000000
            // (MAME-canonical, see `rs_open_bus_zero` comment above).
            // No DECERR — bus errors are not raised on unmapped Q700
            // addresses; the Q700 ROM's RAM-sizing routine treats the
            // 0-read past the SIMM boundary as "device absent" and
            // sizes RAM correctly.  Earlier revisions returned
            // 0xFFFFFFFF here on an incorrect inference about MAME's
            // default unmap value — fixed 2026-05-03 after the live
            // FPGA halt-bisect at N=10M showed VBR being corrupted to
            // 0x0000fece by a RAM probe consuming an all-ones read.
            // Fix 1/2: burst-onto-lite-only-slave and poisoned-slave
            // rejections answer SLVERR; the pre-existing NONE (unmapped)
            // policy above is unchanged (OKAY).
            if (req_rd_local[0] && (rs_state[0] == RS_IDLE) && !mr_arready_r[0]) begin
                rs_state    [0] <= RS_SEND_RLOCAL;
                rs_slv      [0] <= `XBAR_SLV_NONE;
                rs_mid      [0] <= mr_arid [0];
                rs_len      [0] <= mr_arlen[0];
                rs_bcnt     [0] <= mr_arlen[0] + 8'd1;
                rs_local_rsp[0] <= local_rsp_is_err_r[0] ? `AXI_RESP_SLVERR : `AXI_RESP_OKAY;
                rs_open_bus_zero[0] <= 1'b1;
                mr_arready_r[0] <= 1'b1;
            end else if (req_rd_local[1] && (rs_state[1] == RS_IDLE) && !mr_arready_r[1]) begin
                rs_state    [1] <= RS_SEND_RLOCAL;
                rs_slv      [1] <= `XBAR_SLV_NONE;
                rs_mid      [1] <= mr_arid [1];
                rs_len      [1] <= mr_arlen[1];
                rs_bcnt     [1] <= mr_arlen[1] + 8'd1;
                rs_local_rsp[1] <= local_rsp_is_err_r[1] ? `AXI_RESP_SLVERR : `AXI_RESP_OKAY;
                rs_open_bus_zero[1] <= 1'b1;
                mr_arready_r[1] <= 1'b1;
            end else if (req_rd_local[2] && (rs_state[2] == RS_IDLE) && !mr_arready_r[2]) begin
                rs_state    [2] <= RS_SEND_RLOCAL;
                rs_slv      [2] <= `XBAR_SLV_NONE;
                rs_mid      [2] <= mr_arid [2];
                rs_len      [2] <= mr_arlen[2];
                rs_bcnt     [2] <= mr_arlen[2] + 8'd1;
                rs_local_rsp[2] <= local_rsp_is_err_r[2] ? `AXI_RESP_SLVERR : `AXI_RESP_OKAY;
                rs_open_bus_zero[2] <= 1'b1;
                mr_arready_r[2] <= 1'b1;
            end else if (req_rd_local[3] && (rs_state[3] == RS_IDLE) && !mr_arready_r[3]) begin
                rs_state    [3] <= RS_SEND_RLOCAL;
                rs_slv      [3] <= `XBAR_SLV_NONE;
                rs_mid      [3] <= mr_arid [3];
                rs_len      [3] <= mr_arlen[3];
                rs_bcnt     [3] <= mr_arlen[3] + 8'd1;
                rs_local_rsp[3] <= local_rsp_is_err_r[3] ? `AXI_RESP_SLVERR : `AXI_RESP_OKAY;
                rs_open_bus_zero[3] <= 1'b1;
                mr_arready_r[3] <= 1'b1;
            end
        end
    end

endmodule
