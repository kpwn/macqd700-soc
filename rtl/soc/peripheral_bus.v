// peripheral_bus.v — Thin AXI4 slave → multi-peripheral fan-out.
//
// The system xbar's S1 slave exposes a 16 MB I/O window at 0x5000_0000.
// This module demultiplexes that window onto:
//
//   ──────────────────────────────────────────────────────────────────
//   Offset range          | Peripheral     | Local port shape
//   ──────────────────────────────────────────────────────────────────
//   0x000_0000..0x000_1FFF | VIA1           | pb_addr[3:0], 8-bit data
//   0x000_2000..0x000_3FFF | VIA2           | pb_addr[3:0], 8-bit data
//   0x000_8000..0x000_8007 | Ethernet ID    | pb_addr[2:0], 8-bit data
//   0x000_A000..0x000_B0FF | SONIC          | reg[5:0], native 16-bit data
//   0x000_C000..0x000_DFFF | SCC            | pb_addr[3:0], 8-bit data
//   0x000_E000..0x000_E0FF | Orwell controls| pb_addr[7:0], 8-bit data
//   0x000_F000..0x000_F0FF | SCSI/TurboSCSI | pb_addr[3:0] = addr[7:4]
//   0x000_F100..0x000_F101 | SCSI DMA shim  | pb_addr[8:0], 8-bit data
//   0x001_1000..0x001_1FFF | ADB injection  | pb_addr[7:0], 8-bit data
//   0x001_4000..0x001_5FFF | ASC (8 KB)     | pb_addr[11:0] BYTE-GRAN, 8-bit data
//   0x001_E000..0x001_FFFF | SWIM/IWM       | pb_addr[3:0], 8-bit data
//   0x0xx_xxxx             | Q700 mirror    | addr[23:18] ignored for Mac devices
//   0x0F1_E000..0x0F1_FFFF | SWIM/IWM       | NO SCC alt-base.  This window is
//                                              an ordinary Q700 mirror copy of
//                                              0x001_E000 and belongs to
//                                              SWIM/IWM.  A carve-out routing
//                                              it to SCC was DELETED in task
//                                              #246 (settled 2026-08-05) — see
//                                              the "NO SCC ALTERNATE BASE"
//                                              block in decode_slot().
//   0x080_0000..0x08F_FFFF | (REMOVED)      | was sd_provision — now VOID
//   0x090_0000..0x09F_FFFF | debug_ctrl     | 20-bit AXI4-Lite slave
//   0x010_0000..0x01F_FFFF | DMA config     | CARVED OUT by xbar → S2
//   0xF980_0000..0xF980_03FF | DAFB regs    | CARVED OUT by xbar → S4
//   (all other)            | VOID (OKAY + 0x00000000) | matches MAME Q700 I/O open-bus
//   ──────────────────────────────────────────────────────────────────
//
// The DMA config window at offset 0x010_0000 (system 0x5010_0000) is a
// hole inside the IO region — xbar routes it to S2 directly so it never
// lands here.  Any stray CPU mis-address within that range now returns
// OKAY + 0x00000000 rather than DECERR or aliasing into a random
// peripheral.  Task #116 (dma-ctrl-wire).
//
// Mac peripherals (VIA1/VIA2/ENET/SONIC/SCC/ORWELL/SCSI/ASC/IWM) speak the
// simpler `pb_*` handshake (pulse `pb_wr`/`pb_rd` for one cycle;
// peripheral presents `pb_rdata` + `pb_ack` the next cycle).
// debug_ctrl keeps an AXI4-Lite slave interface.  The DAFB AXI4-Lite shim
// is still present on this module's port list for unit-test compatibility,
// but production `fpga_top` ties that legacy decode off and routes DAFB
// through xbar S4.  sd_provision was removed; its 0x080_0000..0x08F_FFFF
// window is now caught by the SLOT_FAULT (a.k.a. VOID) path.
//
// ── Open-bus / VOID policy (MAME-canonical) ───────────────────────────
// Q700 I/O-space open-bus reads return 0x00 + AXI OKAY, matching MAME's
// observed behaviour for the Quadra 700 I/O aperture.  Writes are
// silently dropped with OKAY.
//
// History: an earlier revision returned 0xFFFFFFFF as a guess at the
// "pull-up" silicon convention.  Cross-validation against MAME using
// `tools/mame_axi_capture.lua` (see docs/axi_lockstep.md) showed every
// I/O-space open-bus read returning 0x00 there; matching that observed
// behaviour in our fabric closes the lockstep divergence at seq=0.
// RAM-space and ROM-space open-bus reads also return 0x00 per
// `axi_defs.vh` — that's MAME's actual default `set_unmap_value`,
// which `macquadra700.cpp` never overrides.  See the axi_xbar.v
// `rs_open_bus_zero` comment for the cross-aperture policy.
//
// Diagnostic loss: an unmapped write/read no longer raises an explicit
// DECERR — agents debugging "this peripheral isn't getting hit" should
// observe at the peripheral side instead of relying on the master
// seeing a fault.
//
// Byte-lane logic: the xbar's 128-bit data bus carries the 32-bit
// word in one of four 32-bit lanes selected by addr[3:2].  For
// 8-bit Mac peripherals we further pick one byte out of the selected
// 32-bit word using addr[1:0] on big-endian byte lane.  For writes we
// replicate the write data onto the target byte.  For reads we broadcast
// the peripheral's 8-bit rdata into all 4 byte lanes of the selected
// 32-bit word and zero the other lanes.
//
// One outstanding read and one outstanding write at a time.
// Peripheral-bus traffic is rare and low-bandwidth — this simplicity is
// fine.
//
// ── Same-slot read/write interlock ───────────────────────────────────
// The read and write FSMs are independent (one outstanding each), but
// every pb device has ONE shared addr bus (the <dev>_addr muxes below
// give the WRITE side priority) and ONE shared ack wire consumed by
// BOTH FSMs.  If a read and a write to the same peripheral were ever
// in flight simultaneously, the read's pulse would go out under the
// write's address, and each FSM could consume the other's ack —
// misrouted register access / phantom completion.  Unreachable from
// the CPU alone (the LSU serializes non-cacheable MMIO), but reachable
// if the host-debug/JTAG AXI master touches a peripheral the CPU is
// also touching — and the window is widest on SCSI, where a DMA-shim
// transaction can legitimately stall for a DRQ wait (see the
// wr_scsi_word_active / rd_scsi_dma_shim_active serializers).
//
// CLOSED by the acceptance-point interlock below (`aw_same_slot_hold` /
// `ar_same_slot_hold`): a new AR is not accepted while a write to the
// same pb slot is in flight, and vice versa; on simultaneous same-cycle
// arrival to the same idle slot the WRITE wins (matching the addr mux's
// write priority) and the read waits one transaction.  The interlock
// only guards the shared pb_* faces — SLOT_DBG / SLOT_DAFB have fully
// independent AXI-Lite read/write channels and SLOT_FAULT touches no
// device, so concurrent rd+wr there remains allowed (JTAG debug_ctrl
// polling keeps full throughput).  Blocking is acceptance-only: an
// in-flight transaction never waits on the other FSM, so no deadlock.
// Regression lock: tb_peripheral_bus.cpp
// test_same_slot_write_read_interlock (address-routing proof) and
// test_same_slot_scsi_stalled_write_blocks_read (DRQ-stall window).

`default_nettype none

module peripheral_bus #(
    parameter ID_WIDTH   = 6,
    parameter DATA_WIDTH = 128,
    parameter STRB_WIDTH = DATA_WIDTH/8,
    // Ack-timeout watchdog bound, as a log2 cycle count.  A tight bound
    // (e.g. 2^16 pb-clk cycles, ~1.3 ms @ 50 MHz) would be WRONG here: a
    // legitimate DRQ-checked SCSI DMA-shim beat can wait on real SD-card
    // latency, which is >100 ms worst-case in hardware, and would be
    // falsely SLVERR'd.  Default 24 -> 2^24 cycles (~335 ms @ 50 MHz):
    // still a hard bound against a genuinely wedged peripheral, generous
    // enough for any real SD-backed transfer.  Test harnesses override
    // this down (tb_peripheral_bus builds with -GPB_WATCHDOG_LOG2=10) so
    // the timeout scenarios stay fast in sim; production instantiation
    // sites use the default (fpga_top_peripherals.vh pins it explicitly).
    parameter PB_WATCHDOG_LOG2 = 24,
    // ── ACK WATCHDOG — DISABLED BY DEFAULT (owner decision 2026-09-06) ──
    //
    // Set to 1 to restore.  At 0 both fire conditions are constant-false,
    // so synthesis strips the two 32-bit counters and the SLVERR arms.
    //
    // WHY IT IS OFF.  A real Mac has no such timer: a pseudo-DMA read that
    // is not ready simply withholds DTACK and the CPU waits.  This watchdog
    // instead LAUNDERS a forward-progress bug into a fatal, misleading
    // vector-2 bus error ~335 ms later, at an address that says nothing
    // about the real fault.  Measured cost, 2026-09-06 (p149, System 7.0.1):
    // a stalled 53C96 pseudo-DMA beat SLVERR'd here, the ROM's vec-2
    // handler ran its 1-retry give-up path, and that path `rts`-es over an
    // un-popped SwapMMUMode word -> SP-2 -> vec-3/vec-4 -> Sad Mac.  The
    // watchdog turned a debuggable hang into a flaky crash three layers
    // removed from its cause.
    //
    // This file's own header already documents the same class of harm:
    // raising sd_ctrl's watchdog re-inverted the layering so the FATAL AXI
    // abandonment pre-empted the GRACEFUL retryable CHECK CONDITION by
    // 3.75x, making recovery unreachable.  axi_narrow_to_wide's
    // abandonment watchdog was disabled by default for this reason on
    // 2026-08-20; this is the same call, one layer down.
    //
    // THE TRADE, stated plainly: with this off, a genuinely wedged
    // peripheral holds wr_busy/rd_busy — and the fabric and LSU behind it —
    // until reset.  That is DELIBERATE.  A hang that stops where it broke
    // is debuggable; a bus error 335 ms downstream is not.  Fix the
    // forward-progress bug, do not bound it.
    parameter ENABLE_ACK_WATCHDOG = 0,
    // ── PERIPHERAL-RESET BARRIER TAIL ────────────────────────────────
    // Extra clk cycles the pb-face barrier (`pb_quiesce`) is held after
    // `periph_rst` deasserts.  It must cover every synchroniser that is
    // reset WITH the peripherals and feeds a decision this module makes
    // while the peripherals are coming back.  The binding one today is
    // fpga_top_peripherals.vh's `dafb_scsi0_ctrl_sync_q` (2 FFs, reset by
    // the same pb_full_rst broadcast as u_scsi): until it has reloaded,
    // scsi.v's `scsi_ctrl_in[8:7]` read 0, so `dma_rd_ready`/`dma_wr_ready`
    // read 1 ("DRQ check inactive") no matter what the DAFB actually
    // programmed -- i.e. the shim's pulse gate is lying for 2 cycles past
    // the release.  4 gives 2 cycles of margin over that.
    parameter PERIPH_RST_TAIL = 4
) (
    input  wire                   clk,
    input  wire                   rst,

    // ── Peripheral-fabric reset indication ───────────────────────────
    // 1 while the pb_* peripherals downstream of this module are held in
    // reset.  In the SoC this is the `pb_full_rst` broadcast (board cold
    // reset | JTAG debug-full-reset | warm_peripheral_reset, i.e. the
    // 68040 RESET instruction) -- a DIFFERENT net from this module's own
    // `rst` (pb_soc_full_rst), deliberately so: a 68040 RESET must not
    // reset the S1 front door under a still-running CPU.
    //
    // WHY THIS PORT EXISTS.  Every pb_* strobe this module emits is a
    // ONE-SHOT: `pb_rd_active` (generic read), `pb_wr_active` (generic
    // write), `rd_scsi_dma_shim_active` / `wr_scsi_word_active` (the SCSI
    // pseudo-DMA serializers) and `wr_ser_pulse` (the ASC/ORWELL/SONIC
    // byte walk) each fire for exactly one cycle and are then latched off
    // (`rd_ar_done`, `wr_w_done`, `*_beat_kicked_q`, `wr_ser_phase_q`)
    // until the peripheral's `pb_ack` comes back.  A peripheral held in
    // reset cannot sample the strobe and will never ack it, so the pulse
    // is LOST and the FSM waits forever: `wr_busy`/`rd_busy` stay set and
    // `s_awready`/`s_arready` stay low -- the whole S1 slave dead to every
    // master, permanently, on a plain 68040 RESET instruction.
    //
    // Worse, the shim's own guard does not see the reset either: the
    // `scsi_dma_{rd,wr}_ready` mirrors are combinational from
    // `scsi_ctrl_in`, which is reset by the same net as u_scsi, so they
    // read 1 ("go ahead") for the entire window.  The pulse ALWAYS fires
    // into the reset chip.
    //
    // With this port wired, `pb_quiesce` below closes the front door to
    // pb-slot traffic, suppresses every pb_* strobe, and rolls any
    // already-kicked transaction back to its pre-strobe state so it
    // re-issues when the peripherals are back.  Tie to 1'b0 only in a
    // harness where the peripherals share this module's `rst`.
    input  wire                   periph_rst,

    // ── AXI4 slave (from xbar S1) ─────────────────────────────────────
    // AXI4 requires the full signal set; this module uses only the
    // single-beat subset (len/size/burst ignored).  Live xbar wiring
    // range-checks S1 but forwards the full system address; unit tests
    // may also drive zero-based 24-bit offsets.
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [ID_WIDTH-1:0]    s_awid,
    input  wire [31:0]            s_awaddr,
    input  wire [7:0]             s_awlen,
    input  wire [2:0]             s_awsize,
    input  wire [1:0]             s_awburst,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire                   s_awvalid,
    output wire                   s_awready,

    input  wire [DATA_WIDTH-1:0]  s_wdata,
    input  wire [STRB_WIDTH-1:0]  s_wstrb,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire                   s_wlast,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire                   s_wvalid,
    output wire                   s_wready,

    output wire [ID_WIDTH-1:0]    s_bid,
    output wire [1:0]             s_bresp,
    output wire                   s_bvalid,
    input  wire                   s_bready,

    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [ID_WIDTH-1:0]    s_arid,
    input  wire [31:0]            s_araddr,
    input  wire [7:0]             s_arlen,
    input  wire [2:0]             s_arsize,
    input  wire [1:0]             s_arburst,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire                   s_arvalid,
    output wire                   s_arready,

    output wire [ID_WIDTH-1:0]    s_rid,
    output wire [DATA_WIDTH-1:0]  s_rdata,
    output wire [1:0]             s_rresp,
    output wire                   s_rlast,
    output wire                   s_rvalid,
    input  wire                   s_rready,

    // ── debug_ctrl AXI4-Lite master face ──────────────────────────────
    output wire [19:0]            dbg_awaddr,
    output wire                   dbg_awvalid,
    input  wire                   dbg_awready,
    output wire [31:0]            dbg_wdata,
    output wire [3:0]             dbg_wstrb,
    output wire                   dbg_wvalid,
    input  wire                   dbg_wready,
    input  wire [1:0]             dbg_bresp,
    input  wire                   dbg_bvalid,
    output wire                   dbg_bready,
    output wire [19:0]            dbg_araddr,
    output wire                   dbg_arvalid,
    input  wire                   dbg_arready,
    input  wire [31:0]            dbg_rdata,
    input  wire [1:0]             dbg_rresp,
    input  wire                   dbg_rvalid,
    output wire                   dbg_rready,

    // ── DAFB register-shim AXI4-Lite master face ────────────────────
    output wire [31:0]            dafb_awaddr,
    output wire                   dafb_awvalid,
    input  wire                   dafb_awready,
    output wire [31:0]            dafb_wdata,
    output wire [3:0]             dafb_wstrb,
    output wire                   dafb_wvalid,
    input  wire                   dafb_wready,
    input  wire [1:0]             dafb_bresp,
    input  wire                   dafb_bvalid,
    output wire                   dafb_bready,
    output wire [31:0]            dafb_araddr,
    output wire                   dafb_arvalid,
    input  wire                   dafb_arready,
    input  wire [31:0]            dafb_rdata,
    input  wire [1:0]             dafb_rresp,
    input  wire                   dafb_rvalid,
    output wire                   dafb_rready,

    // ── VIA1 pb_* master face ─────────────────────────────────────────
    output wire [3:0]             via1_addr,
    output wire [7:0]             via1_wdata,
    output wire                   via1_wr,
    output wire                   via1_rd,
    input  wire [7:0]             via1_rdata,
    input  wire                   via1_ack,

    // ── VIA2 pb_* master face ─────────────────────────────────────────
    output wire [3:0]             via2_addr,
    output wire [7:0]             via2_wdata,
    output wire                   via2_wr,
    output wire                   via2_rd,
    input  wire [7:0]             via2_rdata,
    input  wire                   via2_ack,

    // ── Ethernet ID / SONIC pb_* master faces ────────────────────────
    output wire [2:0]             enet_addr,
    output wire [7:0]             enet_wdata,
    output wire                   enet_wr,
    output wire                   enet_rd,
    input  wire [7:0]             enet_rdata,
    input  wire                   enet_ack,

    output wire [5:0]             sonic_addr,
    output wire [15:0]            sonic_wdata,
    output wire [1:0]             sonic_wstrb,
    output wire                   sonic_wr,
    output wire                   sonic_rd,
    input  wire [15:0]            sonic_rdata,
    input  wire                   sonic_ack,

    // ── Orwell controls pb_* master face ─────────────────────────────
    output wire [7:0]             orwell_addr,
    output wire [7:0]             orwell_wdata,
    output wire                   orwell_wr,
    output wire                   orwell_rd,
    input  wire [7:0]             orwell_rdata,
    input  wire                   orwell_ack,

    // ── SCC pb_* master face ─────────────────────────────────────────
    output wire [3:0]             scc_addr,
    output wire [7:0]             scc_wdata,
    output wire                   scc_wr,
    output wire                   scc_rd,
    input  wire [7:0]             scc_rdata,
    input  wire                   scc_ack,

    // ── SCSI pb_* master face ─────────────────────────────────────────
    output wire [8:0]             scsi_addr,
    output wire [7:0]             scsi_wdata,
    output wire                   scsi_wr,
    output wire                   scsi_rd,
    input  wire [7:0]             scsi_rdata,
    input  wire                   scsi_ack,
    input  wire                   scsi_dma_rd_ready,
    input  wire                   scsi_dma_wr_ready,
    // 1 while the SCSI beat we are driving is the LOW half of a HOST
    // 16-bit pseudo-DMA aperture access that we split into two byte
    // beats.  scsi.v needs this stated explicitly: address parity alone
    // cannot tell a split word's low half from a genuine standalone
    // byte access to the odd aperture byte.  See scsi_dma16_lo_beat's
    // driver below and c96_dma16_hi_granted in rtl/mac/scsi.v.
    output wire                   scsi_dma16_lo_beat,

    // ── ASC pb_* master face ─────────────────────────────────────────
    output wire [11:0]            asc_addr,
    output wire [7:0]             asc_wdata,
    output wire                   asc_wr,
    output wire                   asc_rd,
    input  wire [7:0]             asc_rdata,
    input  wire                   asc_ack,

    // ── SWIM/IWM pb_* master face ────────────────────────────────────
    output wire [3:0]             iwm_addr,
    output wire [7:0]             iwm_wdata,
    output wire                   iwm_wr,
    output wire                   iwm_rd,
    input  wire [7:0]             iwm_rdata,
    input  wire                   iwm_ack,

    // ── ADB injection MMIO pb_* master face ──────────────────────────
    // Window 0x0F0_11000..0x0F0_11FFF (mac-canonical 0x011000); register
    // map documented in fpga_top_peripherals.vh.
    output wire [7:0]             adbinj_addr,
    output wire [7:0]             adbinj_wdata,
    output wire                   adbinj_wr,
    output wire                   adbinj_rd,
    input  wire [7:0]             adbinj_rdata,
    input  wire                   adbinj_ack
);

    // ── Peripheral slot encoding ──────────────────────────────────────
    // 4-bit slot index identifies the destination of each pending
    // transaction.  The destination is derived combinationally from
    // address bits and latched with the transaction.
    localparam [3:0]
        SLOT_DBG  = 4'd0,   // default fallback → debug_ctrl
        SLOT_ENET = 4'd2,
        SLOT_SONIC= 4'd3,
        SLOT_SCC  = 4'd4,
        SLOT_SCSI = 4'd5,
        SLOT_ASC  = 4'd6,
        SLOT_VIA1 = 4'd7,
        SLOT_VIA2 = 4'd8,
        SLOT_DAFB = 4'd9,
        SLOT_ORWELL = 4'd10,
        SLOT_IWM  = 4'd11,
        SLOT_ADBINJ = 4'd13, // ADB injection MMIO @ 0x001_1000..0x001_1FFF
        SLOT_FAULT = 4'd12;  // unmapped Q700/service gap → VOID (OKAY+0,
                             // mimics open-bus on real Q700 silicon).
                             // Name retained for routing semantics; the
                             // BEHAVIOUR is now a quiet acknowledge, not
                             // a fault.  See header block for rationale.
        // SLOT 4'd1 (was SLOT_PROV/sd_provision) removed; the 0x080_0000
        // window is now caught by the catch-all FAULT (VOID) branch.

    // ── Bounded ack watchdog ─────────────────────────────────────────
    // Generic, peripheral-agnostic safety net: if a transaction's
    // downstream ack / B-response never arrives (a wedged or misbehaving
    // peripheral, or any future ack-loss bug in the same family as the
    // 2026-07-2x SCSI DMA-shim races), the write/read FSMs below would
    // hold wr_busy/rd_busy — and, through the xbar, the whole AXI fabric
    // and the LSU behind it — hostage forever.  After PB_ACK_TIMEOUT
    // cycles without completion the transaction is aborted with SLVERR,
    // freeing the bus for recovery.  This sits ALONGSIDE the targeted
    // fixes already in place (scsi_dma_{rd,wr}_ready pulse gating, the
    // same-slot rd/wr interlock): those prevent known ack-loss
    // mechanisms; this bounds the damage of any unknown one.  Ported in
    // concept from fix/scsi-pb-handshake's watchdog (re-implemented
    // against this FSM shape).  Fixed 32-bit counter width regardless of
    // the parameter value (LOG2 is never expected to exceed ~30) keeps
    // the width static across instantiations.
    localparam [31:0] PB_ACK_TIMEOUT = (32'd1 << PB_WATCHDOG_LOG2);

    // Q700 Mac-device mirror: MAME/local ROM traces canonicalize device
    // addresses by clearing addr[23:18] inside 0x5000_0000..0x50FF_FFFF.
    // Example: 0x50F0_C000 aliases to canonical SCC base 0x5000_C000.
    // The local debug_ctrl window (and the now-removed sd_provision
    // window's DECERR catch) is decoded from the raw offset before this
    // mirror is applied.
    localparam [23:0] Q700_IO_MIRROR_MASK = 24'hFC0000;

    // ── SCC alternate-base alias: 0x50F1_E000..0x50F1_FFFF ────────────
    // The Q700 ROM's MacsBug "machine UNKNOWN" dispatcher (gated on
    // D0[17] / D7[17], reached via the macsbug-feature-bit17 ROM patch)
    // routes the SCC base via *(A0+0x44) = 0x50F1_E020.  Under MAME's
    // canonical Q700 mirror semantics that physical address aliases to
    // mac_off 0x01_E020 — i.e. SWIM/IWM, NOT SCC — because the bit-17
    // path was authored for a *different* Mac variant (post-Q700 with a
    // different SCC controller location, the eclipse-state Q900-class
    // chipset).
    //
    // We deliberately deviate from MAME for this single mirror copy: when
    // the raw IO offset matches 0x0F1_E000..0x0F1_FFFF (i.e. the bit-17
    // dispatcher's hardcoded base + a small address window for the chan-A
    // control/data port pair at +2/+6), we route the access to SCC.  All
    // OTHER mirror copies of mac_off 0x01_E000 still alias to SWIM, so
    // the canonical Q700 floppy-poll path (which uses the unmodified
    // 0x5001_E000 family) is unaffected.
    //
    // This makes the macsbug-feature-bit17 diagnostic patch produce
    // host-visible TX traffic (cpu_tx_bytes growing past the boot 0x80
    // self-test byte) without ROM-image hacks or testbench-side patches.
    // See docs/BUG_macsbug_repl_unreached.md for the investigation
    // history and the alternative options that were considered.

    // Combinational slot decode from either a zero-based 24-bit I/O offset
    // (unit-tb style) or a full 0x50xx_xxxx system address (xbar style).
    // Full addresses outside 0x50xx_xxxx are not Mac I/O and fault rather
    // than accidentally aliasing through addr[23:0].
    /* verilator lint_off UNUSEDSIGNAL */
    function [3:0] decode_slot;
        input [31:0] addr;
        reg [23:0] off_raw;
        reg [23:0] mac_off;
        begin
            off_raw = addr[23:0];
            mac_off = off_raw & ~Q700_IO_MIRROR_MASK;

            // Legacy/unit-test DAFB register window.  Production xbar
            // decodes this full physical address to S4 before S1.
            if ((addr >= 32'hF980_0000) && (addr < 32'hF980_0400)) begin
                decode_slot = SLOT_DAFB;
            end
            // sd_provision was REMOVED; the 0x080_0000..0x08F_FFFF window
            // now returns DECERR.  Catch it BEFORE the Q700 mirror so it
            // can't alias back to VIA1 through addr[23:18].
            else if ((addr[31:24] == 8'h00 || addr[31:24] == 8'h50) &&
                (off_raw[23:20] == 4'h8)) begin
                decode_slot = SLOT_FAULT;
            end
            // DMA config is carved out by the xbar.  If a misrouted
            // transaction reaches S1 anyway, fault it before the Q700
            // mirror can alias 0x5010_0000 back to VIA1.
            else if ((addr[31:24] == 8'h00 || addr[31:24] == 8'h50) &&
                     (off_raw[23:20] == 4'h1)) begin
                decode_slot = SLOT_FAULT;
            end
            // debug_ctrl service BAR at raw 0x090_0000..0x09F_FFFF.
            // Decode before the Q700 mirror so service traffic cannot be
            // mistaken for VIA1 through the addr[23:18] mask.
            else if ((addr[31:24] == 8'h00 || addr[31:24] == 8'h50) &&
                     (off_raw[23:20] == 4'h9)) begin
                decode_slot = SLOT_DBG;
            end
            // NO SCC ALTERNATE BASE.  The whole 0x50F1_Exxx window belongs
            // to SWIM/IWM through the Q700 addr[23:18] mirror, exactly as
            // the canonical 0x5001_Exxx copy does.
            //
            // History (task #246, settled 2026-08-05): a carve-out here
            // used to route raw 0xF1_E000..0xF1_E03F (later narrowed to
            // 0xF1_E020..0xF1_E03F) to SLOT_SCC, on the theory that a
            // MacsBug "bit-17 dispatcher" reached the SCC via an alternate
            // base at 0x50F1_E020.  Nobody had confirmed that; the original
            // window also swallowed SWIM/IWM register 0 -- the exact base
            // the .Sony driver polls -- answering every floppy poll with SCC
            // channel-B control (RR0 = 0x54 when idle) AND resetting the
            // SCC's register pointer, which is shared across both channels.
            // 7.5.3 brings AppleTalk up by default, so that pointer clobber
            // corrupted live serial register sequences.
            //
            // MEASURED against MAME (macqd700, 25 s of a real 7.5.3 boot,
            // read taps on both windows):
            //   0x50F1_E000..E1FF :     46 reads, ALL at 0x50F1_E000 (reg 0)
            //                           ZERO reads anywhere in E020..E03F
            //   0x50F0_C000..C0FF : 284,055 reads (C020 x282,085, C024 x1,970)
            // A one-shot probe also shows the disputed window reading a flat
            // 0xFF across E000..E040, with no discontinuity at E020, while
            // the canonical SCC at 0x50F0_C000 reads 0x44.  The SCC that the
            // OS actually talks to lives at offset 0x20 inside the CANONICAL
            // window (0x50F0_C020) -- almost certainly the source of the
            // confusion, since it shares the low 0x20 offset with the
            // imagined alt base.  There is no SCC at 0x50F1_E020.
            //
            // So the alias is deleted rather than narrowed again: narrowing
            // could never help, because every byte of it sits inside SWIM
            // register 0's 0x200 stride.  Canonical SCC decode below still
            // covers 0x000_C000..0x000_DFFF, which is where the real traffic
            // goes.
            // Reject accidental full-address calls outside the 16 MB Q700
            // I/O mirror.  The xbar already enforces this in live fabric;
            // this guard keeps the unit-level decode honest too.
            else if (!(addr[31:24] == 8'h00 || addr[31:24] == 8'h50)) begin
                decode_slot = SLOT_FAULT;
            end
            else if (mac_off < 24'h002000) begin
                decode_slot = SLOT_VIA1;
            end
            else if ((mac_off >= 24'h002000) && (mac_off < 24'h004000)) begin
                decode_slot = SLOT_VIA2;
            end
            else if ((mac_off >= 24'h008000) && (mac_off < 24'h008008)) begin
                decode_slot = SLOT_ENET;
            end
            else if ((mac_off >= 24'h00A000) && (mac_off < 24'h00B100)) begin
                decode_slot = SLOT_SONIC;
            end
            else if ((mac_off >= 24'h00C000) && (mac_off < 24'h00E000)) begin
                decode_slot = SLOT_SCC;
            end
            else if ((mac_off >= 24'h00E000) && (mac_off < 24'h00E100)) begin
                decode_slot = SLOT_ORWELL;
            end
            else if (((mac_off >= 24'h00F000) && (mac_off < 24'h00F100)) ||
                     ((mac_off >= 24'h00F100) && (mac_off < 24'h00F102))) begin
                decode_slot = SLOT_SCSI;
            end
            else if ((mac_off >= 24'h011000) && (mac_off < 24'h012000)) begin
                // ADB injection MMIO — host-side write-only enqueue
                // window for the keyboard/mouse models.  Byte-granular
                // pb_* shape (8-bit data, 8-bit register address).  Reads
                // expose status flags.  See fpga_top_peripherals.vh §ADB.
                decode_slot = SLOT_ADBINJ;
            end
            else if ((mac_off >= 24'h014000) && (mac_off < 24'h016000)) begin
                decode_slot = SLOT_ASC;
            end
            else if ((mac_off >= 24'h01E000) && (mac_off < 24'h020000)) begin
                decode_slot = SLOT_IWM;
            end
            // Everything else: deterministic bus fault.  Q700 gaps must
            // not silently fall into debug_ctrl or a neighboring device.
            else begin
                decode_slot = SLOT_FAULT;
            end
        end
    endfunction
    /* verilator lint_on UNUSEDSIGNAL */

    wire [3:0] s_aw_slot = decode_slot(s_awaddr);
    wire [3:0] s_ar_slot = decode_slot(s_araddr);

    // ── Same-slot read/write interlock (see header block) ─────────────
    // True for slots whose downstream face shares ONE addr bus and ONE
    // ack wire between the rd/wr FSMs.  DBG/DAFB are AXI-Lite masters
    // with independent read/write channels; FAULT touches no device.
    function slot_shares_pb_face;
        input [3:0] slot;
        begin
            slot_shares_pb_face = (slot != SLOT_DBG) &&
                                  (slot != SLOT_DAFB) &&
                                  (slot != SLOT_FAULT);
        end
    endfunction

    // ── Peripheral-reset barrier (`pb_quiesce`) ──────────────────────
    // See the `periph_rst` port comment for the defect this closes.
    //
    // The barrier is `periph_rst` widened by PERIPH_RST_TAIL cycles.  The
    // tail is not cosmetic: the SCSI shim's pulse gate
    // (scsi_dma_{rd,wr}_ready) is combinational from a synchroniser that
    // is reset WITH the peripherals, so for PERIPH_RST_TAIL-ish cycles
    // after release it still reports "ready" from its reset state.
    //
    // It is deliberately a plain down-counter with no reset-edge detect:
    // re-loading on every cycle `periph_rst` is high means an arbitrarily
    // long (or re-triggered) peripheral reset simply holds the barrier,
    // and there is no bounded-time assumption anywhere in it.
    //
    // Loaded on this module's own `rst` too.  Costs nothing (we are idle
    // there) and means a harness that ties periph_rst low still starts
    // from a defined barrier state.
    reg [7:0] pq_tail_q;
    always @(posedge clk) begin
        if (rst || periph_rst)   pq_tail_q <= PERIPH_RST_TAIL[7:0];
        else if (pq_tail_q != 8'd0) pq_tail_q <= pq_tail_q - 8'd1;
    end
    wire pb_quiesce = periph_rst || (pq_tail_q != 8'd0);

    // Front-door hold: a request whose slot drives a pb_* face is not
    // ACCEPTED while the barrier is up, so it can never be admitted into
    // a window in which its strobe would be lost.  DBG / DAFB / FAULT are
    // untouched -- they have independent AXI-Lite channels (or touch no
    // device at all), so JTAG debug_ctrl polling keeps full throughput
    // right through a peripheral reset, which is exactly when it is most
    // wanted.  `s_aw_slot` / `s_ar_slot` are already in the ready path via
    // the same-slot interlock, so this adds a term, not a level.
    wire aw_quiesce_hold = pb_quiesce && slot_shares_pb_face(s_aw_slot);
    wire ar_quiesce_hold = pb_quiesce && slot_shares_pb_face(s_ar_slot);

    // Hold off accepting a new AW while a READ to the same pb slot is
    // in flight.  (wr_busy / rd_busy / wr_slot_q / rd_slot_q are the
    // FSM registers declared below; forward references are fine at
    // module scope.)
    wire aw_same_slot_hold = rd_busy && slot_shares_pb_face(s_aw_slot) &&
                             (s_aw_slot == rd_slot_q);
    // Hold off accepting a new AR while a WRITE to the same pb slot is
    // in flight — and on simultaneous same-cycle arrival to the same
    // idle slot, let the write win (matches the addr mux's write
    // priority) so the two FSMs can never latch the same slot together.
    wire ar_same_slot_hold =
        wr_busy ? (slot_shares_pb_face(s_ar_slot) && (s_ar_slot == wr_slot_q))
                : (s_awvalid && slot_shares_pb_face(s_ar_slot) &&
                   (s_ar_slot == s_aw_slot));

    // ── Write transaction ─────────────────────────────────────────────
    // Two-phase FSM: accept AW+W from the xbar, dispatch to one of the
    // peripheral faces, wait for B/ack, then return B upstream.
    reg        wr_busy;
    reg [31:0] wr_timeout_cnt;  // bounded ack watchdog (see PB_ACK_TIMEOUT)
    reg [ID_WIDTH-1:0] wr_id_q;
    // Lower bits feed the peripheral-local address.  Keep the full
    // system address so AXI-Lite targets such as DAFB can preserve
    // recognizable addresses in debug prints and traces.
    /* verilator lint_off UNUSEDSIGNAL */
    reg [31:0] wr_addr_q;
    /* verilator lint_on UNUSEDSIGNAL */
    reg [1:0]  wr_lane_q;     // which 32-bit slice of the 128-bit beat
    reg [3:0]  wr_slot_q;
    reg [2:0]  wr_size_q;
    reg        wr_aw_done;
    reg        wr_w_done;
    reg        wr_b_done;
    reg [1:0]  wr_resp_q;
    // ── SCSI DMA-shim write serializer state ─────────────────────────
    // Latched copy of the W beat (data + remaining strobes) for writes
    // to the TurboSCSI pseudo-DMA window (SLOT_SCSI, wr_addr_q[8]).
    // The AXI W handshake completes immediately at capture; the
    // serializer below then replays the beat into scsi.v one byte per
    // pb_wr pulse, each pulse gated on scsi_dma_wr_ready and re-armed
    // only after its pb_ack — see wr_scsi_word_active's comment for the
    // full rationale.
    reg [31:0] wr_scsi_data_q;      // latched 32-bit lane data
    reg [3:0]  wr_scsi_strb_q;      // remaining hot strobe bits
    reg        wr_scsi_beat_kicked_q; // pulse fired, ack still in flight
    // "The next pulse is the FIRST byte of this AXI beat."  Set at
    // capture, cleared by the first pulse.  Its complement is what makes
    // the second (and any later) byte of a split 16-bit aperture write
    // identifiable to scsi.v as the LOW half of one atomic host access —
    // see scsi_dma16_lo_beat's assign below.
    reg        wr_scsi_first_q;

    // ── Byte-granular multi-byte write serializer state ───────────────
    // (Historically ASC-only — `wr_asc_*`.  Generalized to the whole
    //  byte-granular slot family; see the slot_serializes() note below.)
    //
    // The Q700 ROM EASC startup-chime path writes to volume registers
    // R_FIFOA_VOLUME_LR (0xF06) and R_FIFOB_VOLUME_LR (0xF26) using a
    // single 16-bit MOVE.W (e.g. `move.w #$7F00, asc_base+0xF06`).  The
    // LSU emits this as one AXI beat with TWO strobe bits hot in the
    // active 32-bit lane.  We serialize the multi-byte beat into N
    // back-to-back single-byte pb_* transactions on the asc face.
    //
    // Strobe-walk direction is HIGH→LOW.  m68k_mem_strb / m68k_mem_wdata
    // (rtl/core/mem/m68k_mem_lane.vh) place the FIRST big-endian byte
    // (going to address AW) at the HIGHEST hot strobe bit:
    //   WORD off=2: strb=4'b0011, wdata={16'd0, hi, lo}
    //              bit 1 guards wdata[15:8] = high byte → addr AW (2)
    //              bit 0 guards wdata[7:0]  = low  byte → addr AW+1 (3)
    // Walking HIGH→LOW therefore emits AW, AW+1, AW+2, ... in big-endian
    // memory order.  See the byte-lane note at the top of this file for
    // the broader Option A semantic ("AW carries the actual byte start
    // address; strb tells where in the 32-bit lane each byte lives") and
    // the multi-byte canary block below for the historical context.
    //
    // FSM phases (wr_ser_phase_q):
    //   2'd0  pulse pb_wr for current strobe bit, wait one cycle.
    //   2'd1  await the slot's pb_ack; on ack, clear current strobe bit
    //         and either advance to next-lower hot bit (back to phase 0)
    //         or finish.
    // wr_ser_multi_q latches "multi-byte mode active" so the participating
    // slot's combinational drives are sourced from FSM state instead of
    // the single-byte fast path.
    //
    // WHICH SLOTS PARTICIPATE — and why it is not "all of them".
    //
    // Necessary condition: AWADDR[1:0] must actually reach the slot's
    // device-local address, i.e. consecutive byte addresses must be
    // distinct device locations.  That holds for ASC (addr[11:0]), ORWELL
    // (addr[7:0]), SONIC (addr[12:0]), and also ENET (addr[2:0]) and
    // ADBINJ (addr[7:0]) — see the per-slot address muxes at the bottom
    // of this file.  It does NOT hold for VIA1/VIA2/IWM (addr[12:9],
    // 0x200 stride) or the SCSI *register* page ({5'b0, addr[7:4]},
    // 0x10 stride): there AWADDR[1:0] is a don't-care, so "increment the
    // byte address and pulse again" resolves to pulsing the SAME stateful
    // register N times (VIA IFR/IER/SR/timer latches, SWIM phase latches)
    // — strictly worse than today's single pulse.  Those slots need a
    // different fix (deliver the byte AWADDR names, single pulse) which
    // is deliberately NOT done here.  SCC is neither family (its decode
    // uses addr[2:1] but not addr[0]) and double-pulsing it would push
    // the Z85C30's shared cross-channel register pointer twice — also
    // excluded.
    //
    // Not sufficient, though — and this is the part that cost a
    // regression to learn.  A multi-hot beat is only unambiguously an
    // N-byte transfer if the MASTER's convention says so.  The m68k LSU's
    // convention (byte-lane note at the top of this file) does: AWADDR is
    // the byte start address and each hot strobe is a real byte.  But
    // this fabric also carries a second, documented convention
    // (see the wr_byte comment): host/debug masters — notably the
    // JTAG-AXI master — put ONE meaningful byte in the strobed lane with
    // a WORD-ALIGNED AWADDR and all four strobes hot.  Serializing that
    // shape scatters the payload over four device registers.
    //
    // So membership is restricted to slots with a MEASURED m68k
    // multi-hot producer and no host/debug producer:
    //   ASC     — ROM chime MOVE.W to 0xF06/0xF26 + OS Sound Manager
    //             MOVE.L to the FIFO ports (the original case).
    //   ORWELL  — the ROM's `move.l D4,(A2)+` init loop over 0x00..0xB8.
    //   SONIC   — the System 7 Ethernet driver's MOVE.L reset sequence.
    // Deliberately EXCLUDED despite being byte-granular:
    //   ADBINJ  — its ONLY producer is the JTAG-AXI host path, which uses
    //             the word-aligned/full-strobe convention above; tb
    //             test_adbinj_decode locks that behaviour.  No Mac code
    //             can reach this window at all, so there is no m68k
    //             multi-hot producer to serve.
    //   ENET    — the Ethernet address PROM window; zero writes of any
    //             width were observed under a full System 7 boot, so
    //             there is nothing to serve and the same host-convention
    //             ambiguity would apply.  Latent, left alone.
    // Evidence for every claim above: docs/mame_periph_multihot_reachability.md
    // (MAME macquadra700, System 7.0.1 + 7.5.3, two independent
    //  instruments, 1.69 M classified peripheral writes).
    reg        wr_ser_multi_q;
    reg [31:0] wr_ser_data_q;       // latched 32-bit lane data
    reg [3:0]  wr_ser_strb_q;       // remaining hot strobe bits
    reg [1:0]  wr_ser_phase_q;      // 0 = pulse, 1 = await ack
    reg [1:0]  wr_ser_addr_base_q;  // AW[1:0] at start of multi-byte burst
    reg [1:0]  wr_ser_addr_off_q;   // byte offset advanced after each ack
    // Ack captured during the PULSE cycle.  ASC registers its pb_ack, so
    // the historical two-phase walk (pulse in phase 0, ack in phase 1)
    // worked as written.  ORWELL and SONIC ack COMBINATIONALLY —
    // `ack = cs && (rd || wr)` — so their ack is visible only in the
    // pulse cycle and is already gone by phase 1; without this latch the
    // walk would stall until the PB_ACK_TIMEOUT watchdog and deliver a
    // single byte plus an SLVERR.  This is the one place the ASC pattern
    // did not generalize unchanged.  Both device styles consume the byte
    // at the pulse cycle's posedge, so latching the ack (rather than
    // holding pb_wr until it arrives) is also the shape that keeps pb_wr
    // a strict one-cycle pulse with a low cycle between bytes.
    reg        wr_ser_ack_seen_q;

    // Lane derived from addr[3:2] — same mechanic as before, the xbar
    // has stripped the lower 4 bits into a single 32-bit word placement.
    wire [1:0] s_aw_lane = s_awaddr[3:2];

    // 32-bit slice of the incoming beat.
    wire [31:0] wr_lane_data =
        (wr_lane_q == 2'd0) ? s_wdata[31:0]   :
        (wr_lane_q == 2'd1) ? s_wdata[63:32]  :
        (wr_lane_q == 2'd2) ? s_wdata[95:64]  :
                              s_wdata[127:96];
    wire [3:0]  wr_lane_strb =
        (wr_lane_q == 2'd0) ? s_wstrb[3:0]    :
        (wr_lane_q == 2'd1) ? s_wstrb[7:4]    :
        (wr_lane_q == 2'd2) ? s_wstrb[11:8]   :
                              s_wstrb[15:12];

    // For 8-bit Mac peripherals: pick one byte out of the selected 32-bit
    // lane.  WSTRB IS THE AUTHORITY, and the only authority.
    //
    // In AXI4 the write strobes determine which byte lanes are written; the
    // address determines only where the transfer starts.  A compliant slave
    // therefore uses WSTRB, and this bus must be correct from the perspective
    // of a driver running on the CPU without assuming anything about which
    // master is upstream.
    //
    // WHAT WAS HERE BEFORE, and why it went (2026-09-05).  This used to be
    //     wr_byte = onehot ? (wr_addr_byte | wr_strb_byte) : wr_strb_byte
    // bitwise-ORing an ADDRESS-selected byte with the STROBE-selected byte, to
    // "accept either convention", justified by "well-formed writes have only
    // one non-zero copy".  That is a property of the MASTER, not of this bus,
    // and it is FALSE for cpu040: its store path presents a full 16-byte cache
    // LINE IMAGE on WDATA, so the non-strobed lanes carry live data, not zero.
    // The OR was therefore not merely redundant there -- it was the one master
    // for which a divergence would have delivered a corrupt OR of two real
    // bytes to a register.  Every 8-bit peripheral is on this wire (VIA1, VIA2,
    // SCC, SCSI, IWM, ENET, ASC, ORWELL, ADB), so that failure would have been
    // broad and silent.
    //
    // The two selectors also used OPPOSITE conventions -- the address one was
    // big-endian (addr[1:0]==0 -> [31:24]), the strobe one little-endian
    // (strb[0] -> [7:0]).  They agreed only because both real CPUs happen to
    // land the byte at index 3-addr[1:0].  A correctness argument that depends
    // on a coincidence between two masters is one that breaks the next time a
    // master changes.
    //
    // Deleting it is behaviour-preserving for every master in this tree,
    // established by characterising each rather than by assuming:
    //   v1 m68k LSU  AWADDR = the byte address; WSTRB = 4'b1000 >> addr[1:0]
    //                (m68k_mem_lane.vh:19); byte at [8*(3-addr[1:0]) +: 8]
    //                (m68k_mem_lane.vh:38-45); other lanes ZERO.
    //   cpu040       same two indices via SocketByteOrder's per-lane swap;
    //                other lanes are a live line image (see above).
    //   boot FSM     WSTRB = 4'b1111 (boot_fsm.v:1553) -- never one-hot, so it
    //                never reached the OR at all.
    //   JTAG-AXI     WSTRB = 4'b1111; the Vivado JTAG-to-AXI IP has no strobe
    //                argument and jtag_repl.tcl requires word alignment -- also
    //                never one-hot.  Deleting the OR cannot affect the REPL.
    //   MAME bridge  AXI-standard little-endian (mame_axi_periph_bridge.cpp:
    //                557-571).  For THIS producer removing the address selector
    //                is not merely safe, it is REQUIRED to stay correct.
    // The PCIe XDMA host path's convention lives in host software outside this
    // repo and is unknown; a compliant host is HELPED by this change and could
    // only ever have been corrupted by the OR.
    //
    // `wr_addr_q` itself is still needed -- for the byte-granular slot
    // addresses (ASC/ORWELL/SONIC pb_addr) and for wr_sonic_word_path below.
    // Only the byte SELECTOR is gone.
    wire [7:0] wr_byte =
        wr_lane_strb[0] ? wr_lane_data[ 7: 0] :
        wr_lane_strb[1] ? wr_lane_data[15: 8] :
        wr_lane_strb[2] ? wr_lane_data[23:16] :
                          wr_lane_data[31:24];

    // ── Byte-granular slot family (multi-byte write serializer) ───────
    // See the wr_ser_* declaration block above for why membership is
    // exactly this set and not "every pb_* slot".
    function slot_serializes;
        input [3:0] slot;
        begin
            slot_serializes = (slot == SLOT_ASC)    ||
                              (slot == SLOT_ORWELL);
        end
    endfunction

    // SONIC is a native 16-bit endpoint.  It never enters the byte-walk
    // serializers; one AXI access produces one register transaction.
    wire wr_ser_slot = slot_serializes(wr_slot_q);

    // Per-slot ack / write-strobe taps for the serializer branch — same
    // shape the wr_b_done case below already uses for the generic slots.
    wire wr_ser_ack = (wr_slot_q == SLOT_ASC)    ? asc_ack    :
                      (wr_slot_q == SLOT_ORWELL) ? orwell_ack : 1'b0;
    wire wr_ser_wr  = (wr_slot_q == SLOT_ASC)    ? asc_wr     :
                      (wr_slot_q == SLOT_ORWELL) ? orwell_wr  : 1'b0;

    // ── Is the in-flight write's payload recoverable after a cancelled
    //    pulse?  (peripheral-reset barrier -- see `periph_rst`.)
    // TRUE exactly for the two serializers, which latch the W beat into
    // their own registers and clear a strobe bit only on that byte's
    // pb_ack.  FALSE for the generic one-pulse path, whose data is a
    // combinational tap on s_wdata that is gone the cycle after capture.
    // Note wr_ser_multi_q is the right term (not wr_ser_slot): a
    // SINGLE-byte write to a byte-granular slot takes the generic
    // pb_wr_active pulse, not the byte walk.
    wire wr_replayable = (wr_ser_slot && wr_ser_multi_q) ||
                         ((wr_slot_q == SLOT_SCSI) && wr_addr_q[8]);

    always @(posedge clk) begin
        if (rst) begin
            wr_busy     <= 1'b0;
            wr_timeout_cnt <= 32'd0;
            wr_id_q     <= {ID_WIDTH{1'b0}};
            wr_addr_q   <= 32'b0;
            wr_lane_q   <= 2'b0;
            wr_slot_q   <= SLOT_DBG;
            wr_size_q   <= 3'b0;
            wr_aw_done  <= 1'b0;
            wr_w_done   <= 1'b0;
            wr_b_done   <= 1'b0;
            wr_resp_q   <= 2'b0;
            wr_scsi_data_q <= 32'b0;
            wr_scsi_strb_q <= 4'b0;
            wr_scsi_beat_kicked_q <= 1'b0;
            wr_scsi_first_q <= 1'b0;
            wr_ser_multi_q <= 1'b0;
            wr_ser_data_q  <= 32'b0;
            wr_ser_strb_q  <= 4'b0;
            wr_ser_phase_q <= 2'd0;
            wr_ser_addr_base_q <= 2'd0;
            wr_ser_addr_off_q  <= 2'd0;
            wr_ser_ack_seen_q  <= 1'b0;
        end else begin
            if (!wr_busy) begin
                if (s_awvalid && !aw_same_slot_hold && !aw_quiesce_hold) begin
                    wr_busy     <= 1'b1;
                    wr_timeout_cnt <= 32'd0;
                    wr_id_q     <= s_awid;
                    wr_addr_q   <= s_awaddr;
                    wr_lane_q   <= s_aw_lane;
                    wr_slot_q   <= s_aw_slot;
                    wr_size_q   <= s_awsize;
                    wr_aw_done  <= 1'b0;
                    wr_w_done   <= 1'b0;
                    wr_b_done   <= 1'b0;
                    wr_resp_q   <= 2'b0;
                    wr_scsi_strb_q <= 4'b0;
                    wr_scsi_beat_kicked_q <= 1'b0;
                    wr_scsi_first_q <= 1'b0;
                    wr_ser_multi_q <= 1'b0;
                    wr_ser_phase_q <= 2'd0;
                    wr_ser_addr_off_q <= 2'd0;
                    wr_ser_ack_seen_q <= 1'b0;
                end
            end else begin
                // Bounded ack watchdog (see PB_ACK_TIMEOUT): if the
                // timeout has already expired (from previous cycles'
                // increments), short-circuit BEFORE the per-slot case so
                // we never race a legitimate same-cycle completion — the
                // two paths are mutually exclusive (if/else-if), never
                // both scheduling an NBA write to wr_b_done/wr_resp_q.
                // Once wr_b_done is set (either way) the per-slot case is
                // skipped; every arm's actions are gated on !wr_b_done /
                // pre-B done-flags, so this is behaviour-neutral for a
                // normal completion.
                //
                // Corner case (documented, carried over from the original
                // fix/scsi-pb-handshake design, not fixed here): the
                // counter starts on AW acceptance, not on the W beat's
                // arrival — so a master that completes the AW handshake
                // but then stalls WVALID for longer than PB_ACK_TIMEOUT
                // would also trip this watchdog, even though no
                // PERIPHERAL is wedged.  Not reachable today: this fabric
                // has exactly one in-order AXI master behind the xbar,
                // which always follows an accepted AW with WVALID
                // promptly.  Revisit if a second master (e.g. a DMA
                // engine) ever shares this slave port.
                //
                // ── PERIPHERAL-RESET BARRIER, WRITE SIDE ─────────────
                // Checked BEFORE the watchdog, and before the per-slot
                // case, for the same mutual-exclusion reason the watchdog
                // is: exactly one of these arms may schedule an NBA write
                // to wr_b_done / wr_resp_q in a cycle.
                //
                // Two consequences of taking this arm:
                //   * wr_timeout_cnt does NOT advance, so a peripheral
                //     reset can never be mistaken for a wedged peripheral
                //     by the (production-disabled) ack watchdog.
                //   * every REPLAYABLE one-shot is rolled back to its
                //     pre-pulse state, so it re-emits when the barrier
                //     lifts.  See the `periph_rst` port comment.
                if (pb_quiesce && !wr_b_done && slot_shares_pb_face(wr_slot_q)) begin
                    // The SCSI DMA shim and the byte-granular byte walk
                    // both LATCH their payload (wr_scsi_data_q/_strb_q,
                    // wr_ser_data_q/_strb_q), and only clear a strobe bit
                    // on that byte's pb_ack -- so rolling the "pulse is in
                    // flight" state back re-emits exactly the bytes that
                    // were never acknowledged.  Lossless.
                    wr_scsi_beat_kicked_q <= 1'b0;
                    // ...and the re-emitted byte must re-run scsi.v's DRQ
                    // grant rather than inherit the pre-reset one, which
                    // the reset cancelled.
                    wr_scsi_first_q       <= 1'b1;
                    wr_ser_phase_q        <= 2'd0;
                    // A pb_ack latched from a pulse the reset then
                    // cancelled must not satisfy the phase-1 wait.
                    wr_ser_ack_seen_q     <= 1'b0;
                    // The GENERIC pb write path is the one that cannot be
                    // replayed: pb_wr_active drives <slot>_wdata straight
                    // off s_wdata and wr_w_done latches from s_wvalid
                    // alone, so once the W beat is consumed the payload is
                    // gone.  Its exposure is exactly one cycle wide (the
                    // pulse and the W capture are the same cycle, and
                    // s_wready is barred for every later one), and in that
                    // one cycle the peripheral DID sample the byte -- the
                    // reset then discarded it, which is the reset's
                    // semantics, not a dropped write.  Answer the master
                    // with OKAY so the S1 slave comes back, instead of
                    // waiting forever on an ack that was cancelled.
                    if (wr_w_done && !wr_replayable) begin
                        wr_b_done <= 1'b1;
                        wr_resp_q <= 2'b00;
                    end
                end else if (ENABLE_ACK_WATCHDOG && wr_timeout_cnt >= PB_ACK_TIMEOUT && !wr_b_done) begin
                    wr_b_done <= 1'b1;
                    wr_resp_q <= 2'b10; // SLVERR — peripheral never acked
                end else if (!wr_b_done) begin
                    wr_timeout_cnt <= wr_timeout_cnt + 32'd1;
                case (wr_slot_q)
                    SLOT_DBG: begin
                        if (!wr_aw_done && dbg_awready) wr_aw_done <= 1'b1;
                        if (!wr_w_done  && dbg_wready)  wr_w_done  <= 1'b1;
                        if (!wr_b_done  && dbg_bvalid) begin
                            wr_b_done <= 1'b1;
                            wr_resp_q <= dbg_bresp;
                        end
                    end
                    SLOT_DAFB: begin
                        if (!wr_aw_done && dafb_awready) wr_aw_done <= 1'b1;
                        if (!wr_w_done  && dafb_wready)  wr_w_done  <= 1'b1;
                        if (!wr_b_done  && dafb_bvalid) begin
                            wr_b_done <= 1'b1;
                            wr_resp_q <= dafb_bresp;
                        end
                    end
                    SLOT_FAULT: begin
                        // Quiet open-bus write: accept AW + W beats and
                        // acknowledge with OKAY.  Data is dropped on the
                        // floor — no peripheral is hit, and no fault is
                        // raised.  Mimics real Q700 silicon's response to
                        // probes against unmapped I/O.
                        if (!wr_aw_done) wr_aw_done <= 1'b1;
                        if (!wr_w_done && s_wvalid) wr_w_done <= 1'b1;
                        if (!wr_b_done && wr_aw_done && wr_w_done) begin
                            wr_b_done <= 1'b1;
                            wr_resp_q <= 2'b00; // OKAY (VOID)
                        end
                    end
                    default: begin
                        // Mac-peripheral pb_* slot.  pb_wr pulses for one
                        // cycle while s_wvalid is high; the peripheral's
                        // pb_ack is registered (arrives the cycle after
                        // the pb_wr pulse) — so the B-phase ack check
                        // waits on `wr_w_done && <slot>_ack`, which is
                        // naturally aligned because wr_w_done latches the
                        // cycle after the W beat is captured.  Task #125:
                        // the old check was `ack && wr` same-cycle, which
                        // never fired for registered-ack peripherals.
                        if (wr_ser_slot) begin
                            // ── Byte-granular slot: optional multi-byte
                            //    iteration FSM (ASC / ORWELL).
                            // Capture W beat (always — same edge as wr_w_done
                            // would latch on the generic path).  Detect
                            // multi-byte at capture time and enter the
                            // iteration FSM if more than one strobe is hot.
                            // Single-byte writes still go through the simple
                            // ack-driven termination below, whose condition is
                            // deliberately IDENTICAL to the generic default
                            // arm's, so one-hot writes to these slots keep
                            // bit- and cycle-identical behaviour (ORWELL
                            // acks combinationally and completes in the
                            // W-capture cycle; ASC registers its ack and
                            // completes a cycle later).
                            if (!wr_aw_done && s_wvalid) wr_aw_done <= 1'b1;
                            if (!wr_w_done && s_wvalid) begin
                                wr_w_done <= 1'b1;
                                wr_ser_data_q      <= wr_lane_data;
                                wr_ser_strb_q      <= wr_lane_strb;
                                wr_ser_addr_base_q <= wr_addr_q[1:0];
                                // Multi-byte iff >1 hot strobe bit.
                                if (wr_lane_strb_count > 3'd1) begin
                                    wr_ser_multi_q <= 1'b1;
                                    wr_ser_phase_q <= 2'd0;
                                end
                            end
                            if (wr_ser_multi_q) begin
                                if (wr_w_done && !wr_b_done) begin
                                    // ── Multi-byte iteration ────────────
                                    // Phase 0: pb_wr is asserted combinationally
                                    //          for the highest remaining hot
                                    //          strobe bit.  Move to phase 1.
                                    //          A combinational-ack slave
                                    //          (ORWELL/SONIC) acks in this
                                    //          same cycle; latch it.
                                    // Phase 1: wait for the slot's pb_ack —
                                    //          either latched from phase 0 or
                                    //          arriving now from a registered-
                                    //          ack slave (ASC).  On ack, clear
                                    //          that strobe bit; if any remain,
                                    //          return to phase 0; else
                                    //          complete the AXI beat.
                                    if (wr_ser_phase_q == 2'd0) begin
                                        wr_ser_phase_q    <= 2'd1;
                                        wr_ser_ack_seen_q <= wr_ser_ack;
                                    end else if (wr_ser_phase_q == 2'd1 &&
                                                 (wr_ser_ack || wr_ser_ack_seen_q)) begin
                                        wr_ser_ack_seen_q <= 1'b0;
                                        // Clear the just-acknowledged bit.
                                        // The <slot>_addr / <slot>_wdata
                                        // combinational drives prioritise the
                                        // HIGHEST hot bit; clearing it here
                                        // hands the next iteration to the
                                        // next-lower hot bit.
                                        if (wr_ser_strb_q[3])
                                            wr_ser_strb_q[3] <= 1'b0;
                                        else if (wr_ser_strb_q[2])
                                            wr_ser_strb_q[2] <= 1'b0;
                                        else if (wr_ser_strb_q[1])
                                            wr_ser_strb_q[1] <= 1'b0;
                                        else
                                            wr_ser_strb_q[0] <= 1'b0;
                                        // Advance the byte offset: each
                                        // strobe consumed bumps the AW by 1.
                                        wr_ser_addr_off_q <= wr_ser_addr_off_q + 2'd1;
                                        // If exactly one bit was hot before
                                        // this clear, the burst is finished.
                                        if (({2'b00, wr_ser_strb_q[0]}
                                           + {2'b00, wr_ser_strb_q[1]}
                                           + {2'b00, wr_ser_strb_q[2]}
                                           + {2'b00, wr_ser_strb_q[3]})
                                            == 3'd1) begin
                                            wr_b_done <= 1'b1;
                                            wr_ser_multi_q <= 1'b0;
                                        end else begin
                                            wr_ser_phase_q <= 2'd0;
                                        end
                                        wr_resp_q <= 2'b00;
                                    end
                                end
                            end else if (!wr_b_done) begin
                                // Single-byte: terminate on the first ack,
                                // either same-cycle (combinational-ack slave
                                // during the pulse) or once wr_w_done has
                                // latched (registered-ack slave).  Same
                                // condition as the generic arm below.
                                if ((wr_ser_ack && wr_ser_wr) ||
                                    (wr_w_done && wr_ser_ack)) begin
                                    wr_b_done <= 1'b1;
                                    wr_resp_q <= 2'b00;
                                end
                            end
                        end else if (wr_slot_q == SLOT_SCSI && wr_addr_q[8]) begin
                            // ── SCSI DMA-shim write serializer ──────
                            // See wr_scsi_word_active's comment below
                            // for the full mechanism/rationale.  Capture
                            // the W beat (data + strobes) and complete
                            // the AXI W handshake immediately; the
                            // pulse/ack sequencing below then delivers
                            // one pb_wr per hot strobe bit, HIGH→LOW
                            // (big-endian byte order, matching the LSU's
                            // m68k_mem_strb layout and MAME's
                            // dma16_swap_w high-byte-first order), each
                            // pulse gated on scsi_dma_wr_ready and
                            // re-armed only after its pb_ack arrives.
                            if (!wr_aw_done && s_wvalid) wr_aw_done <= 1'b1;
                            if (!wr_w_done && s_wvalid) begin
                                wr_w_done       <= 1'b1;
                                wr_scsi_data_q  <= wr_lane_data;
                                wr_scsi_strb_q  <= wr_lane_strb;
                                wr_scsi_first_q <= 1'b1;
                            end
                            // A pulse fired this cycle → block re-pulse
                            // until its ack has been consumed.  Mutually
                            // exclusive with the ack branch below
                            // (wr_scsi_word_active requires
                            // !wr_scsi_beat_kicked_q).
                            if (wr_scsi_word_active) begin
                                wr_scsi_beat_kicked_q <= 1'b1;
                                // Everything after the first pulse of
                                // this AXI beat is a LOW half — see
                                // wr_scsi_dma16_lo below.
                                wr_scsi_first_q       <= 1'b0;
                            end
                            if (wr_scsi_beat_kicked_q && scsi_ack) begin
                                wr_scsi_beat_kicked_q <= 1'b0;
                                // Retire the just-acknowledged byte:
                                // clear the highest remaining hot bit.
                                if (wr_scsi_strb_q[3])
                                    wr_scsi_strb_q[3] <= 1'b0;
                                else if (wr_scsi_strb_q[2])
                                    wr_scsi_strb_q[2] <= 1'b0;
                                else if (wr_scsi_strb_q[1])
                                    wr_scsi_strb_q[1] <= 1'b0;
                                else
                                    wr_scsi_strb_q[0] <= 1'b0;
                            end
                            // All strobes retired (or a degenerate
                            // zero-strobe write: complete without ever
                            // pulsing — a spurious pulse would consume a
                            // pseudo-DMA byte) → return B.
                            if (wr_w_done && !wr_b_done &&
                                !wr_scsi_beat_kicked_q &&
                                (wr_scsi_strb_q == 4'b0000)) begin
                                wr_b_done <= 1'b1;
                                wr_resp_q <= 2'b00;
                            end
                        end else begin
                            if (!wr_aw_done && s_wvalid) wr_aw_done <= 1'b1;
                            if (!wr_w_done  && s_wvalid) wr_w_done  <= 1'b1;
                            // NOTE: the SONIC/ORWELL/ASC arms of the case
                            // below are no longer reachable — those slots
                            // are handled by the wr_ser_ branch
                            // above (whose single-byte termination uses this
                            // exact condition).  They are kept verbatim as a
                            // safety default in case slot_serializes() is
                            // ever narrowed again.
                            // Accept ack either (a) the cycle the pb_wr
                            // pulse is high and a combinational-ack slave
                            // raises ack same-cycle, or (b) a later cycle
                            // once wr_w_done has latched and a registered
                            // ack arrives — covers both timing styles.
                            if (!wr_b_done) begin
                                case (wr_slot_q)
                                    SLOT_ENET: if ((enet_ack  && enet_wr ) || (wr_w_done && enet_ack )) wr_b_done <= 1'b1;
                                    SLOT_SONIC: if ((sonic_ack && sonic_wr) || (wr_w_done && sonic_ack)) wr_b_done <= 1'b1;
                                    SLOT_ORWELL: if ((orwell_ack && orwell_wr) || (wr_w_done && orwell_ack)) wr_b_done <= 1'b1;
                                    SLOT_SCC:  if ((scc_ack  && scc_wr ) || (wr_w_done && scc_ack )) wr_b_done <= 1'b1;
                                    SLOT_SCSI: if ((scsi_ack && scsi_wr) || (wr_w_done && scsi_ack)) wr_b_done <= 1'b1;
                                    SLOT_ASC:  if ((asc_ack  && asc_wr ) || (wr_w_done && asc_ack )) wr_b_done <= 1'b1;
                                    SLOT_VIA1: if ((via1_ack && via1_wr) || (wr_w_done && via1_ack)) wr_b_done <= 1'b1;
                                    SLOT_VIA2: if ((via2_ack && via2_wr) || (wr_w_done && via2_ack)) wr_b_done <= 1'b1;
                                    SLOT_IWM:  if ((iwm_ack  && iwm_wr ) || (wr_w_done && iwm_ack )) wr_b_done <= 1'b1;
                                    SLOT_ADBINJ: if ((adbinj_ack && adbinj_wr) || (wr_w_done && adbinj_ack)) wr_b_done <= 1'b1;
                                    default:   wr_b_done <= 1'b1; // safety
                                endcase
                                wr_resp_q <= 2'b00;
                            end
                        end
                    end
                endcase
                end
                // Drop wr_busy once we have returned B to the xbar
                if (wr_b_done && s_bready) begin
                    wr_busy <= 1'b0;
                end
            end
        end
    end

    // ── AXI slave back-pressure / handshake ──────────────────────────
    // aw_same_slot_hold: same-slot rd/wr interlock (see header block).
    assign s_awready = !wr_busy && !aw_same_slot_hold && !aw_quiesce_hold;

    // s_wready is high while we still need a W beat and the selected
    // downstream accepts it.
    wire wr_slot_is_dbg  = (wr_slot_q == SLOT_DBG);
    wire wr_slot_is_dafb = (wr_slot_q == SLOT_DAFB);
    wire wr_slot_is_fault = (wr_slot_q == SLOT_FAULT);
    wire wr_slot_is_pb   = !wr_slot_is_dbg && !wr_slot_is_dafb
                         && !wr_slot_is_fault;
    assign s_wready = wr_busy && !wr_w_done &&
                      ((wr_slot_is_dbg  && dbg_wready) ||
                       (wr_slot_is_dafb && dafb_wready) ||
                       (wr_slot_is_fault) ||
                       // pb_* slots always accept -- EXCEPT while the
                       // peripheral-reset barrier is up.  Deferring the W
                       // beat (rather than consuming it and losing the
                       // pulse) is the LOSSLESS half of the fix: WVALID is
                       // still asserted when the barrier lifts, so the byte
                       // is delivered late instead of dropped.
                       (wr_slot_is_pb && !pb_quiesce));
    assign s_bid     = wr_id_q;
    assign s_bresp   = wr_resp_q;
    assign s_bvalid  = wr_busy && wr_b_done;

    // debug_ctrl write channels
    assign dbg_awaddr  = wr_addr_q[19:0];
    assign dbg_awvalid = wr_busy && wr_slot_is_dbg && !wr_aw_done;
    assign dbg_wdata   = wr_lane_data;
    assign dbg_wstrb   = wr_lane_strb;
    assign dbg_wvalid  = wr_busy && wr_slot_is_dbg && !wr_w_done && s_wvalid;
    assign dbg_bready  = wr_busy && wr_slot_is_dbg && !wr_b_done;

    // DAFB register-shim write channels.
    assign dafb_awaddr  = wr_addr_q;
    assign dafb_awvalid = wr_busy && wr_slot_is_dafb && !wr_aw_done;
    assign dafb_wdata   = wr_lane_data;
    assign dafb_wstrb   = wr_lane_strb;
    assign dafb_wvalid  = wr_busy && wr_slot_is_dafb && !wr_w_done && s_wvalid;
    assign dafb_bready  = wr_busy && wr_slot_is_dafb && !wr_b_done;

    // pb_* write strobes — hold pb_wr high for exactly one cycle when
    // the W beat is captured AND the B has not yet been signalled.
    // Multi-byte writes to the byte-granular slot family (ASC, ORWELL) are serialized by
    // the wr_ser_* FSM (one pb_wr pulse per hot strobe bit) — the generic
    // single-cycle pulse below is suppressed in the W-capture cycle when
    // we detect >1 hot strobe, so the FSM (starting on the cycle after
    // capture) emits ALL bytes, including the first.
    wire [2:0] wr_lane_strb_count = {2'b00, wr_lane_strb[0]}
                                  + {2'b00, wr_lane_strb[1]}
                                  + {2'b00, wr_lane_strb[2]}
                                  + {2'b00, wr_lane_strb[3]};
    wire wr_ser_multi_now = wr_busy && wr_ser_slot && s_wvalid &&
                            !wr_w_done && (wr_lane_strb_count > 3'd1);
    wire wr_ser_multi_active = wr_busy && wr_ser_slot &&
                              wr_w_done && !wr_b_done && wr_ser_multi_q;
    wire pb_wr_active = wr_busy && wr_slot_is_pb && !wr_b_done && s_wvalid &&
                        // Peripheral-reset barrier: never strobe a face
                        // that cannot sample it.  s_wready is gated by the
                        // same term, so the W beat waits with us.
                        !pb_quiesce &&
                        !wr_ser_multi_now &&
                        !(wr_ser_slot && wr_ser_multi_q) &&
                        // SCSI DMA-shim writes are serialized by the
                        // wr_scsi_* FSM below instead (drq-gated pulse
                        // per strobe byte) — suppress the generic
                        // W-capture-cycle pulse entirely for them.
                        !(wr_slot_q == SLOT_SCSI && wr_addr_q[8]);

    // ── Byte-granular multi-byte iteration drive (combinational from
    //    latched state).  Walk strobe bits HIGH→LOW; emit one pb_wr pulse
    //    per visit during phase 0; phase 1 holds pb_wr low while we wait
    //    for the slot's pb_ack.
    function [1:0] wr_ser_hi_bit_idx;
        input [3:0] strb;
        begin
            wr_ser_hi_bit_idx = strb[3] ? 2'd3 :
                                strb[2] ? 2'd2 :
                                strb[1] ? 2'd1 : 2'd0;
        end
    endfunction
    wire [1:0] wr_ser_hi_idx  = wr_ser_hi_bit_idx(wr_ser_strb_q);
    wire [7:0] wr_ser_byte    = (wr_ser_hi_idx == 2'd0) ? wr_ser_data_q[ 7: 0] :
                                (wr_ser_hi_idx == 2'd1) ? wr_ser_data_q[15: 8] :
                                (wr_ser_hi_idx == 2'd2) ? wr_ser_data_q[23:16] :
                                                          wr_ser_data_q[31:24];
    wire [1:0] wr_ser_addr_lsb = wr_ser_addr_base_q + wr_ser_addr_off_q;
    wire       wr_ser_pulse    = wr_ser_multi_active && (wr_ser_phase_q == 2'd0) &&
                                 !pb_quiesce;
    // Per-slot qualified pulse — one term per participating slot's *_wr.
    wire wr_ser_pulse_asc    = wr_ser_pulse && (wr_slot_q == SLOT_ASC);
    wire wr_ser_pulse_orwell = wr_ser_pulse && (wr_slot_q == SLOT_ORWELL);
    // Multi-byte mode is live for a given slot (address/wdata muxes take
    // their value from the FSM instead of the single-byte fast path).
    wire wr_ser_on_asc    = wr_ser_multi_active && (wr_slot_q == SLOT_ASC);
    wire wr_ser_on_orwell = wr_ser_multi_active && (wr_slot_q == SLOT_ORWELL);

    // ── SCSI DMA-shim write serializer drive ─────────────────────────
    // Write-side twin of the rd_scsi_dma_shim_active fix (see that wire's
    // comment for the underlying drq_c96 ack-withhold race), but the
    // mechanism is deliberately DIFFERENT from the read side's simple
    // pulse gate, because the write path's pulse is tied to the AXI
    // W-channel handshake rather than to an SCSI-local kicked latch:
    //
    //   • pb_wr_active fires on the cycle s_wvalid is high, and
    //     wr_w_done latches from s_wvalid ALONE (`if (!wr_w_done &&
    //     s_wvalid) wr_w_done <= 1'b1;`), independent of whether the
    //     peripheral pulse actually fired.  Naively AND-ing a
    //     scsi_dma_wr_ready term into scsi_wr's assignment would
    //     therefore let the AXI master complete its W beat (s_wready is
    //     unconditional for pb_* slots, so WVALID drops the next cycle)
    //     while the suppressed pulse never reaches scsi.v — wr_b_done
    //     then waits forever on a pb_ack that can never come, trading
    //     the read-side lost-ack wedge for an unresolvable-B-response
    //     wedge.  Do NOT "simplify" this back to the read-side pattern.
    //
    //   • Instead, DMA-shim writes (SLOT_SCSI, wr_addr_q[8]) get their
    //     own serializer: the W beat's data + strobes are LATCHED at
    //     capture (so the AXI handshake completes normally and the
    //     source data survives WVALID dropping), and the latched beat
    //     is replayed into scsi.v one byte per pb_wr pulse.  Each pulse
    //     fires only on a cycle where scsi_dma_wr_ready reads 1 — the
    //     exported mirror of scsi.v's own `scsi_ctrl_in[8] && !drq_c96`
    //     pb_ack withhold-gate — so pulse-decision and ack-decision
    //     consult the identical drq_c96 value on the identical cycle,
    //     and every fired pulse is guaranteed its ack.
    //
    // The per-strobe-bit walk (HIGH→LOW, one pulse per hot bit) also
    // closes a second latent gap in the same family as the 2026-07-15
    // read-side word/byte bug: a `move.w` store to the shim arrives as
    // ONE AXI beat with TWO hot strobe bits, and per MAME's 53C94
    // (ncr53c94_device::dma16_w / dma16_swap_w) must push TWO sequential
    // bytes (high byte first on the m68k bus) and count the transfer
    // down by 2 — the old single-pulse generic path would have silently
    // dropped the low byte and under-counted the pseudo-DMA transfer.
    // The read side already splits word reads the same way
    // (rd_scsi_phase_q); this is its write-side complement.
    wire wr_scsi_word_active = wr_busy && (wr_slot_q == SLOT_SCSI) &&
                               !pb_quiesce &&
                               wr_addr_q[8] && wr_w_done && !wr_b_done &&
                               (wr_scsi_strb_q != 4'b0000) &&
                               !wr_scsi_beat_kicked_q &&
                               scsi_dma_wr_ready;
    // Byte for the current pulse: highest remaining hot strobe bit
    // (HIGH→LOW walk = big-endian order; the LSU places the first
    // big-endian byte at the highest hot bit — see the byte-lane note
    // above the ASC serializer).
    // ── Write-side "low half of a split 16-bit aperture access" ──────
    // The exact twin of rd_scsi_dma16_lo below, and the reason the old
    // "Writes never take the split path ... read-only by construction"
    // claim at scsi_dma16_lo_beat was wrong: the per-strobe-bit walk
    // above IS a split path, it just splits by strobe bit instead of by
    // rd_scsi_phase_q.  MAME's host access is atomic — dafb.cpp's
    // turboscsi_dma_w consults m_drq ONCE and then hands the whole word
    // to ncr53c94_device::dma16_swap_w, which pushes both bytes with no
    // second DRQ consultation — so every byte after the first must
    // INHERIT the first byte's grant rather than re-running the check.
    // Without this, a DRQ that falls between the two beats (fifo_pos
    // reaching 15, or tcounter reaching 0 on a tcount==1 chunk) strands
    // the second beat forever: wr_scsi_word_active never re-asserts,
    // wr_scsi_strb_q never empties, and the AXI B response only arrives
    // via the PB_ACK_TIMEOUT watchdog as a bus error.
    //
    // Built from registers only (never from wr_scsi_word_active, which
    // consumes scsi_dma_wr_ready) so the inherit term this feeds inside
    // scsi.v cannot close a combinational loop through the DRQ gate.
    wire wr_scsi_dma16_lo = wr_busy && (wr_slot_q == SLOT_SCSI) &&
                            wr_addr_q[8] && wr_w_done && !wr_b_done &&
                            (wr_scsi_strb_q != 4'b0000) &&
                            !wr_scsi_beat_kicked_q &&
                            !wr_scsi_first_q;
    wire [1:0] wr_scsi_hi_idx = wr_ser_hi_bit_idx(wr_scsi_strb_q);
    wire [7:0] wr_scsi_byte =
        (wr_scsi_hi_idx == 2'd3) ? wr_scsi_data_q[31:24] :
        (wr_scsi_hi_idx == 2'd2) ? wr_scsi_data_q[23:16] :
        (wr_scsi_hi_idx == 2'd1) ? wr_scsi_data_q[15: 8] :
                                   wr_scsi_data_q[ 7: 0];

    // ── Read transaction ──────────────────────────────────────────────
    reg        rd_busy;
    reg [31:0] rd_timeout_cnt;  // bounded ack watchdog (see PB_ACK_TIMEOUT)
    reg [ID_WIDTH-1:0] rd_id_q;
    /* verilator lint_off UNUSEDSIGNAL */
    reg [31:0] rd_addr_q;
    /* verilator lint_on UNUSEDSIGNAL */
    reg [1:0]  rd_lane_q;
    reg [3:0]  rd_slot_q;
    reg [2:0]  rd_size_q;
    reg        rd_ar_done;
    reg        rd_r_done;
    reg [31:0] rd_data_q;
    reg [1:0]  rd_resp_q;
    reg [7:0]  rd_scsi_hi_q;
    reg [1:0]  rd_scsi_phase_q;
    reg        rd_scsi_beat_kicked_q;

    wire [1:0] s_ar_lane = s_araddr[3:2];
    wire rd_slot_is_dbg  = (rd_slot_q == SLOT_DBG);
    wire rd_slot_is_dafb = (rd_slot_q == SLOT_DAFB);
    wire rd_slot_is_fault = (rd_slot_q == SLOT_FAULT);
    wire rd_slot_is_pb   = !rd_slot_is_dbg && !rd_slot_is_dafb
                         && !rd_slot_is_fault;

    // Byte → word placement for pb_* reads.  The 8-bit peripheral
    // rdata is broadcast into all four byte lanes of the 32-bit word,
    // so regardless of which byte the CPU's LSU samples, it sees the
    // peripheral's answer.  (Mac byte reads will pick one byte; long-
    // word reads see the byte replicated four times — the CPU-side
    // masking will handle the discard.)
    function [7:0] pb_rd_byte;
        input [3:0] slot;
        begin
            case (slot)
                SLOT_ENET: pb_rd_byte = enet_rdata;
                SLOT_SONIC: pb_rd_byte = sonic_rdata[7:0];
                SLOT_ORWELL: pb_rd_byte = orwell_rdata;
                SLOT_SCC:  pb_rd_byte = scc_rdata;
                SLOT_SCSI: pb_rd_byte = scsi_rdata;
                SLOT_ASC:  pb_rd_byte = asc_rdata;
                SLOT_VIA1: pb_rd_byte = via1_rdata;
                SLOT_VIA2: pb_rd_byte = via2_rdata;
                SLOT_IWM:  pb_rd_byte = iwm_rdata;
                SLOT_ADBINJ: pb_rd_byte = adbinj_rdata;
                default:   pb_rd_byte = 8'h00;
            endcase
        end
    endfunction

    function pb_rd_ack;
        input [3:0] slot;
        begin
            case (slot)
                SLOT_ENET: pb_rd_ack = enet_ack;
                SLOT_SONIC: pb_rd_ack = sonic_ack;
                SLOT_ORWELL: pb_rd_ack = orwell_ack;
                SLOT_SCC:  pb_rd_ack = scc_ack;
                SLOT_SCSI: pb_rd_ack = scsi_ack;
                SLOT_ASC:  pb_rd_ack = asc_ack;
                SLOT_VIA1: pb_rd_ack = via1_ack;
                SLOT_VIA2: pb_rd_ack = via2_ack;
                SLOT_IWM:  pb_rd_ack = iwm_ack;
                SLOT_ADBINJ: pb_rd_ack = adbinj_ack;
                default:   pb_rd_ack = 1'b1;
            endcase
        end
    endfunction

    function pb_rd_same_cycle_ok;
        input [3:0] slot;
        begin
            case (slot)
                SLOT_ENET,
                SLOT_SONIC,
                SLOT_ORWELL,
                SLOT_IWM:  pb_rd_same_cycle_ok = 1'b1;
                default:   pb_rd_same_cycle_ok = 1'b0;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            rd_busy     <= 1'b0;
            rd_timeout_cnt <= 32'd0;
            rd_id_q     <= {ID_WIDTH{1'b0}};
            rd_addr_q   <= 32'b0;
            rd_lane_q   <= 2'b0;
            rd_slot_q   <= SLOT_DBG;
            rd_size_q   <= 3'b0;
            rd_ar_done  <= 1'b0;
            rd_r_done   <= 1'b0;
            rd_data_q   <= 32'b0;
            rd_resp_q   <= 2'b0;
            rd_scsi_hi_q  <= 8'b0;
            rd_scsi_phase_q <= 2'd0;
            rd_scsi_beat_kicked_q <= 1'b0;
        end else begin
            if (!rd_busy) begin
                if (s_arvalid && !ar_same_slot_hold && !ar_quiesce_hold) begin
                    rd_busy     <= 1'b1;
                    rd_timeout_cnt <= 32'd0;
                    rd_id_q     <= s_arid;
                    rd_addr_q   <= s_araddr;
                    rd_lane_q   <= s_ar_lane;
                    rd_slot_q   <= s_ar_slot;
                    rd_size_q   <= s_arsize;
                    rd_ar_done  <= 1'b0;
                    rd_r_done   <= 1'b0;
                    rd_scsi_phase_q  <= 2'd0;
                    rd_scsi_beat_kicked_q <= 1'b0;
                end
            end else begin
                // Bounded ack watchdog — see the write-side comment above
                // for the mutual-exclusion rationale (short-circuit before
                // the per-slot case, never alongside it) and the
                // documented AW/AR-acceptance-start corner case.  A
                // timed-out read returns SLVERR with zeroed data.
                // ── PERIPHERAL-RESET BARRIER, READ SIDE ──────────────
                // Same placement rule as the write side above.  Reads
                // carry no payload downstream, so every read one-shot is
                // replayable unconditionally: re-strobing after the
                // barrier lifts returns the peripheral's post-reset value,
                // which is the truthful answer for a read the reset
                // cancelled.  Nor can the replay disturb a read-sensitive
                // register (VIA IFR, the SCC's shared register pointer,
                // the 53C96 FIFO): the same reset just wiped all of them.
                if (pb_quiesce && !rd_r_done && slot_shares_pb_face(rd_slot_q)) begin
                    rd_ar_done            <= 1'b0;  // re-arm pb_rd_active
                    rd_scsi_beat_kicked_q <= 1'b0;  // re-arm the shim pulse
                    rd_scsi_phase_q       <= 2'd0;  // restart a split word read
                end else if (ENABLE_ACK_WATCHDOG && rd_timeout_cnt >= PB_ACK_TIMEOUT && !rd_r_done) begin
                    rd_r_done <= 1'b1;
                    rd_data_q <= 32'h00000000;
                    rd_resp_q <= 2'b10; // SLVERR — peripheral never acked
                end else if (!rd_r_done) begin
                    rd_timeout_cnt <= rd_timeout_cnt + 32'd1;
                case (rd_slot_q)
                    SLOT_DBG: begin
                        if (!rd_ar_done && dbg_arready) rd_ar_done <= 1'b1;
                        if (!rd_r_done  && dbg_rvalid) begin
                            rd_r_done <= 1'b1;
                            rd_data_q <= dbg_rdata;
                            rd_resp_q <= dbg_rresp;
                        end
                    end
                    SLOT_DAFB: begin
                        if (!rd_ar_done && dafb_arready) rd_ar_done <= 1'b1;
                        if (!rd_r_done  && dafb_rvalid) begin
                            rd_r_done <= 1'b1;
                            rd_data_q <= dafb_rdata;
                            rd_resp_q <= dafb_rresp;
                        end
                    end
                    SLOT_FAULT: begin
                        // Quiet open-bus read: return OKAY + data 0x00000000
                        // with no peripheral access.  This matches MAME's
                        // observed behaviour on Q700 I/O space (e.g.
                        // 0x51001c00 reads return 0x00, not 0xFF) — the
                        // canonical Q700 I/O-side open-bus convention.
                        // Note: RAM-space SIMM-detect probes at PC
                        // 0x408046aa go through the AXI xbar's RAM-out-
                        // of-range path, handled separately in
                        // axi_xbar.v's local-response block (also OKAY +
                        // 0x00000000 per `axi_defs.vh`'s uniform
                        // MAME-canonical unmap policy — an earlier
                        // revision of this note claimed 0xFFFFFFFF,
                        // which was the pre-2026-05-03 behaviour and is
                        // long gone; see the rs_open_bus_zero comment in
                        // axi_xbar.v).
                        if (!rd_ar_done) rd_ar_done <= 1'b1;
                        if (!rd_r_done && rd_ar_done) begin
                            rd_r_done <= 1'b1;
                            rd_data_q <= 32'h00000000;
                            rd_resp_q <= 2'b00; // OKAY (VOID)
                        end
                    end
                    default: begin
                        // Mac-peripheral: pb_rd pulses for one cycle on
                        // entry.  Real RTL devices register pb_rdata/pb_ack;
                        // only the explicitly combinational placeholder
                        // endpoints may complete in that same cycle.
                        if (rd_slot_q == SLOT_SONIC) begin
                            if (!rd_ar_done)
                                rd_ar_done <= 1'b1;
                            if (!rd_r_done && sonic_ack) begin
                                rd_r_done <= 1'b1;
                                rd_resp_q <= 2'b00;
                                if (rd_size_q == 3'd2)
                                    rd_data_q <= {16'hFFFF, sonic_rdata};
                                else if (rd_size_q == 3'd1)
                                    rd_data_q <= rd_addr_q[1] ?
                                                 {sonic_rdata, sonic_rdata} :
                                                 32'hFFFFFFFF;
                                else if (rd_addr_q[1:0] == 2'd2)
                                    rd_data_q <= {4{sonic_rdata[15:8]}};
                                else if (rd_addr_q[1:0] == 2'd3)
                                    rd_data_q <= {4{sonic_rdata[7:0]}};
                                else
                                    rd_data_q <= 32'hFFFFFFFF;
                            end
                        end else if (rd_slot_q == SLOT_SCSI && rd_addr_q[8]) begin
                            // SCSI DMA-shim read (byte OR word) — see
                            // rd_scsi_dma_shim_active's comment above for
                            // the root-cause writeup.  Pulse/ack/re-arm
                            // sequencing (NOT level-held like SONIC,
                            // because scsi.v's ack is a same-shape
                            // passthrough of pb_rd rather than a one-shot
                            // handshake): rd_scsi_dma_shim_active pulses
                            // scsi_rd for exactly one cycle per phase;
                            // this block latches rd_scsi_beat_kicked_q
                            // the cycle the pulse fires (blocking a
                            // second beat while ack is in flight) and
                            // clears it once that phase's scsi_ack has
                            // been consumed, arming the next phase's
                            // pulse.  Byte accesses (rd_size_q != 3'd1)
                            // complete in a single phase; word (movew)
                            // accesses split into two sequential
                            // byte-granular phases, unchanged from
                            // before.  Reconciliation note (T1b,
                            // 2026-07-23): this branch used to require
                            // rd_size_q == 3'd1, so a plain BYTE access
                            // to the DMA-shim window fell through to the
                            // generic pb_rd_active one-shot-pulse path
                            // below — which has zero visibility into
                            // scsi.v's drq_c96 DRQ-check and can fire its
                            // single pulse on a cycle where the ack is
                            // withheld, losing the beat forever (the same
                            // family of bug the word-read fix above
                            // closed, just for the byte-size case).
                            // Widened to cover both sizes so every
                            // DMA-shim read is guaranteed to eventually
                            // ack once DRQ rises instead of stalling to
                            // the ack-timeout watchdog.
                            if (!rd_ar_done) rd_ar_done <= 1'b1;
                            if (rd_scsi_dma_shim_active)
                                rd_scsi_beat_kicked_q <= 1'b1;
                            if (!rd_r_done && scsi_ack) begin
                                if (rd_size_q == 3'd1) begin
                                    if (rd_scsi_phase_q == 2'd0) begin
                                        rd_scsi_hi_q          <= scsi_rdata;
                                        rd_scsi_phase_q       <= 2'd1;
                                        rd_scsi_beat_kicked_q <= 1'b0;
                                    end else if (rd_scsi_phase_q == 2'd1) begin
                                        rd_r_done        <= 1'b1;
                                        rd_scsi_phase_q  <= 2'd2;
                                        rd_data_q <= {rd_scsi_hi_q, scsi_rdata,
                                                      rd_scsi_hi_q, scsi_rdata};
                                        rd_resp_q <= 2'b00;
                                    end
                                end else begin
                                    rd_r_done        <= 1'b1;
                                    rd_scsi_beat_kicked_q <= 1'b0;
                                    rd_data_q <= {4{scsi_rdata}};
                                    rd_resp_q <= 2'b00;
                                end
                            end
                        end else begin
                            if (!rd_ar_done) rd_ar_done <= 1'b1;  // AR captured by us
                            if (!rd_r_done && pb_rd_ack(rd_slot_q) &&
                                (rd_ar_done ||
                                 (pb_rd_active && pb_rd_same_cycle_ok(rd_slot_q)))) begin
                                rd_r_done <= 1'b1;
                                // Replicate the byte across all four lanes
                                // so ANY byte-address read succeeds.
                                rd_data_q <= {4{pb_rd_byte(rd_slot_q)}};
                                rd_resp_q <= 2'b00;
                            end
                        end
                    end
                endcase
                end
                if (rd_r_done && s_rready) begin
                    rd_busy <= 1'b0;
                end
            end
        end
    end

    // SCSI DMA-shim (pb_addr 0x100/0x101) word reads: the 2026-07-15
    // HW post-mortem root cause.  The generic single-shot pb_* path
    // (below) issues exactly ONE scsi_rd strobe per AXI beat and
    // replicates that ONE byte across all four response lanes — correct
    // for ordinary byte-wide register reads, but WRONG for the
    // pseudo-DMA port: the Q700 ROM/driver drains it with `movew`
    // (16-bit) reads, and per MAME's real 53C94 model
    // (ncr53c94_device::dma16_r(), ncr53c90.cpp:1326) a 16-bit access
    // pops TWO SEQUENTIAL bytes and decrements the transfer counter by
    // 2, not 1.  Treating it as one byte read twice: (a) returns the
    // SAME byte in both halves of the word (silent data corruption)
    // and (b) only drains ONE byte's worth of c96_xfr_left/tcounter
    // bookkeeping per movew instead of two — for the ROM's 8-word
    // (16-byte) blind burst this under-counts by exactly half,
    // c96_xfr_left never reaches zero, and the chunk-completion I_BUS
    // interrupt never fires — a deterministic wedge (not a timing
    // race), matching the observed HW hang (frozen at the post-burst
    // INTR poll, TC0 stays set, tcounter reads 0 forever) exactly.
    // Never caught by any prior sim testbench because they all drive
    // scsi.v's Vscsi ports directly, bypassing this module (and its
    // word-vs-byte splitting) entirely.  Fixed the same way SONIC's
    // 16-bit register reads already are, two lines below: two
    // sequential byte-granular slave beats, assembled big-endian
    // (first-fetched byte = high byte, matching
    // dma16_swap_r()'s effective byte order on the m68k bus).
    // scsi.v's pb_ack is a direct combinational passthrough of pb_rd
    // registered one cycle later — holding scsi_rd asserted across
    // multiple cycles would cause a new beat EVERY cycle it's high,
    // not one beat per logical phase.  So this is pulsed (one cycle
    // per phase, gated by rd_scsi_beat_kicked_q) and re-armed only
    // once that phase's scsi_ack has been consumed — see the
    // rd_scsi_beat_kicked_q sequencing in the read-completion block
    // below for the cycle-by-cycle contract.
    //
    // scsi_dma_rd_ready gates a SECOND, independent race found 2026-07-
    // 20/21: this wire fires scsi_rd with zero visibility into scsi.v's
    // own drq_c96 DRQ-check, which withholds pb_ack on the exact same
    // cycle whenever TurboSCSI's DAFB-controlled DRQ-check is active and
    // drq_c96 is momentarily low.  Because scsi_rd is a single-cycle
    // pulse and rd_scsi_beat_kicked_q blocks any retry, a pulse that
    // lands on such a cycle loses that beat's ack forever, regardless of
    // drq_c96 going true again immediately after — a permanent LSU wedge
    // on the SCSI DMA-shim port, confirmed via live ILA capture
    // (ILA_SD) showing every drq_c96 precondition healthy while the
    // CPU's read stayed stuck.  Holding off the pulse until
    // scsi_dma_rd_ready reads 1 makes the pulse-decision and the
    // ack-decision consult the exact same drq_c96 value on the exact
    // same cycle, closing the race.
    //
    // Reconciliation note (T1b, 2026-07-23): renamed from
    // rd_scsi_word_active to rd_scsi_dma_shim_active and widened to
    // cover byte-size DMA-shim reads too (dropped the rd_size_q == 3'd1
    // term that used to gate this wire).  A plain BYTE access to
    // pb_addr 0x100/0x101 does NOT hit the movew word-split path above,
    // and used to fall through unconditionally to the generic
    // pb_rd_active one-shot pulse below — which has no scsi_dma_rd_ready
    // gating at all, so a byte read issued while drq_c96 happened to be
    // momentarily low lost its ack forever, exactly like the pre-fix
    // word case, just for byte-size accesses.  Confirmed empirically:
    // tb-pb-scsi's drq_checked_read_rises_later scenario (byte-size AXI
    // reads) failed against the unwidened gate, timing out to the
    // ack-timeout watchdog instead of completing once DRQ rose.  The
    // (rd_scsi_phase_q < 2'd2) term still gates correctly for byte
    // accesses: rd_scsi_phase_q resets to 0 on AR acceptance and is
    // never advanced on the byte-size completion path (see the
    // read-completion block above), so it stays < 2'd2 until rd_r_done
    // latches.
    wire rd_scsi_dma_shim_active = rd_busy && (rd_slot_q == SLOT_SCSI) &&
                               !pb_quiesce &&
                               rd_addr_q[8] &&
                               !rd_r_done && (rd_scsi_phase_q < 2'd2) &&
                               !rd_scsi_beat_kicked_q &&
                               scsi_dma_rd_ready;

    // pb_rd pulse: we hold it high for one cycle — the cycle after we
    // enter the busy state — so the peripheral gets exactly one pb_rd
    // strobe.  Detect that by holding pb_rd while !rd_ar_done.  SONIC
    // 16-bit reads and the SCSI DMA-shim (any size, see
    // rd_scsi_dma_shim_active above) use their own serialized/gated
    // paths above/below instead.
    wire pb_rd_active = rd_busy && rd_slot_is_pb && !rd_ar_done &&
                        !pb_quiesce &&
                        !(rd_slot_q == SLOT_SCSI && rd_addr_q[8]);

    // ar_same_slot_hold: same-slot rd/wr interlock (see header block).
    assign s_arready = !rd_busy && !ar_same_slot_hold && !ar_quiesce_hold;
    assign s_rid     = rd_id_q;
    assign s_rdata   = (rd_lane_q == 2'd0) ? {96'b0, rd_data_q} :
                       (rd_lane_q == 2'd1) ? {64'b0, rd_data_q, 32'b0} :
                       (rd_lane_q == 2'd2) ? {32'b0, rd_data_q, 64'b0} :
                                             {rd_data_q, 96'b0};
    assign s_rresp   = rd_resp_q;
    assign s_rlast   = 1'b1;
    assign s_rvalid  = rd_busy && rd_r_done;

    assign dbg_araddr  = rd_addr_q[19:0];
    assign dbg_arvalid = rd_busy && rd_slot_is_dbg && !rd_ar_done;
    assign dbg_rready  = rd_busy && rd_slot_is_dbg && !rd_r_done;

    assign dafb_araddr  = rd_addr_q;
    assign dafb_arvalid = rd_busy && rd_slot_is_dafb && !rd_ar_done;
    assign dafb_rready  = rd_busy && rd_slot_is_dafb && !rd_r_done;

    // ── pb_* peripheral drives ───────────────────────────────────────
    // VIA1/VIA2 use the Q700 16-register layout at 0x200 stride.
    // addr[12:9] selects the 6522 register inside the 8 KB slot window:
    //   0x0000, 0x0200, ... 0x1E00 within VIA1
    //   0x2000, 0x2200, ... 0x3E00 within VIA2
    // The byte lane is handled separately by wr_byte / rd_data_q.
    assign via1_addr  = wr_busy && wr_slot_q == SLOT_VIA1 ? wr_addr_q[12:9] :
                        rd_busy && rd_slot_q == SLOT_VIA1 ? rd_addr_q[12:9] :
                                                             4'd0;
    assign via1_wdata = wr_byte;
    assign via1_wr    = pb_wr_active && (wr_slot_q == SLOT_VIA1);
    assign via1_rd    = pb_rd_active && (rd_slot_q == SLOT_VIA1);

    assign via2_addr  = wr_busy && wr_slot_q == SLOT_VIA2 ? wr_addr_q[12:9] :
                        rd_busy && rd_slot_q == SLOT_VIA2 ? rd_addr_q[12:9] :
                                                             4'd0;
    assign via2_wdata = wr_byte;
    assign via2_wr    = pb_wr_active && (wr_slot_q == SLOT_VIA2);
    assign via2_rd    = pb_rd_active && (rd_slot_q == SLOT_VIA2);

    // Ethernet ID: 8-byte probe window, byte-granular within the window.
    // Byte-granular decode, but deliberately NOT in slot_serializes() —
    // see that function's comment (no observed producer of any width).
    assign enet_addr  = wr_busy && wr_slot_q == SLOT_ENET ? wr_addr_q[2:0] :
                        rd_busy && rd_slot_q == SLOT_ENET ? rd_addr_q[2:0] :
                                                            3'd0;
    assign enet_wdata = wr_byte;
    assign enet_wr    = pb_wr_active && (wr_slot_q == SLOT_ENET);
    assign enet_rd    = pb_rd_active && (rd_slot_q == SLOT_ENET);

    // SONIC: one native 16-bit register operation per CPU access.  The chip
    // occupies D15..D0 (physical bytes +2/+3) in each 32-bit Q700 slot;
    // physical bytes +0/+1 are open bus and writes to them carry no strobes.
    assign sonic_addr = wr_busy && wr_slot_q == SLOT_SONIC ? wr_addr_q[7:2] :
                        rd_busy && rd_slot_q == SLOT_SONIC ? rd_addr_q[7:2] :
                                                            6'd0;
    // Byte-lane convention (see "Byte-lane logic" above and wr_addr_byte):
    // the lowest-address byte of a 32-bit slot sits in wr_lane_data[31:24], so
    // the SONIC's connected halfword -- physical bytes +2/+3 -- is always
    // wr_lane_data[15:0], for LONG and WORD writes alike.  A WORD write to the
    // open-bus half (+0) carries no strobes, so its data selection is moot.
    assign sonic_wdata = (wr_size_q == 3'd1) ? wr_lane_data[15:0] :
                         (wr_size_q == 3'd0 && wr_addr_q[1:0] == 2'd2) ?
                         {wr_byte, 8'h00} :
                         (wr_size_q == 3'd0 && wr_addr_q[1:0] == 2'd3) ?
                         {8'h00, wr_byte} : wr_lane_data[15:0];
    assign sonic_wstrb = (wr_size_q == 3'd2) ?
                         {wr_lane_strb[1], wr_lane_strb[0]} :
                         (wr_size_q == 3'd1) ?
                         (wr_addr_q[1] ? 2'b11 : 2'b00) :
                         (wr_addr_q[1:0] == 2'd2) ? 2'b10 :
                         (wr_addr_q[1:0] == 2'd3) ? 2'b01 : 2'b00;
    assign sonic_wr = pb_wr_active && (wr_slot_q == SLOT_SONIC);
    assign sonic_rd = pb_rd_active && (rd_slot_q == SLOT_SONIC);

    // Orwell controls: 256-byte probe-safe reset-value stub window.
    // Byte-granular (addr[7:0]) — the Q700 ROM's Orwell init loop issues
    // `move.l D4, (A2)+` over this page, i.e. genuinely multi-hot beats,
    // so it takes the wr_ser_* byte walk.
    assign orwell_addr  = wr_ser_on_orwell ? {wr_addr_q[7:2], wr_ser_addr_lsb} :
                          wr_busy && wr_slot_q == SLOT_ORWELL ? wr_addr_q[7:0] :
                          rd_busy && rd_slot_q == SLOT_ORWELL ? rd_addr_q[7:0] :
                                                                8'd0;
    assign orwell_wdata = wr_ser_on_orwell ? wr_ser_byte : wr_byte;
    assign orwell_wr    = (pb_wr_active && (wr_slot_q == SLOT_ORWELL)) ||
                          wr_ser_pulse_orwell;
    assign orwell_rd    = pb_rd_active && (rd_slot_q == SLOT_ORWELL);

    // SCC: Z85C30 Universal Bus dc_ab layout as used by the Q700 MAME
    // model.  Byte offset bit 1 selects channel A/B and bit 2 selects
    // control/data.  Keep higher local alias bits for probe logging while
    // feeding scc.v the same low two semantic bits MAME passes to dc_ab_r/w.
    assign scc_addr  = wr_busy && wr_slot_q == SLOT_SCC ? {wr_addr_q[5:4], wr_addr_q[1], wr_addr_q[2]} :
                       rd_busy && rd_slot_q == SLOT_SCC ? {rd_addr_q[5:4], rd_addr_q[1], rd_addr_q[2]} :
                                                           4'd0;
    assign scc_wdata = wr_byte;
    assign scc_wr    = pb_wr_active && (wr_slot_q == SLOT_SCC);
    assign scc_rd    = pb_rd_active && (rd_slot_q == SLOT_SCC);

    // SCSI/TurboSCSI: Q700 DAFB maps the NCR register aperture on 16-byte
    // strides (`offset >> 4` in MAME's dafb_device::turboscsi_r/w).  Keep
    // the DMA shim byte-granular at 0x100/0x101, but collapse 0x000..0x0ff
    // to controller register numbers so 0xF040 reaches status register 4.
    function [8:0] scsi_local_addr;
        input [31:0] addr;
        begin
            if (addr[8])
                scsi_local_addr = addr[8:0];
            else
                scsi_local_addr = {5'b00000, addr[7:4]};
        end
    endfunction

    // 16-bit DMA-port pairing.  The DAFB TurboSCSI pseudo-DMA aperture is
    // a SIXTEEN-BIT port (macquadra700.cpp:565 maps 0x5000f100..0x5000f101
    // to dafb_device::turboscsi_dma_r, which DRQ-checks ONCE per host
    // access and then pops both bytes atomically through
    // ncr53c94_device::dma16_swap_r).  We replay one word access as two
    // byte beats, so the SECOND beat must address the aperture's ODD byte
    // — that is what lets scsi.v inherit the first beat's DRQ grant
    // instead of re-running the check mid-word (see c96_dma16_hi_granted
    // in rtl/mac/scsi.v; re-checking there stranded the last word of every
    // 16-byte chunk under LBTM and turned the Q700 ROM's drain loop into a
    // bus error).  Byte-size accesses never advance rd_scsi_phase_q, so
    // they keep their literal address unchanged.
    // The single source of truth for "this beat is the low half of a
    // split 16-bit aperture access" — it drives BOTH the odd address
    // and the explicit scsi_dma16_lo_beat flag, so the two can never
    // disagree.  Byte-size accesses never advance rd_scsi_phase_q, so
    // they keep their literal address and leave the flag low.
    wire rd_scsi_dma16_lo = rd_busy && (rd_slot_q == SLOT_SCSI) &&
                            rd_addr_q[8] && (rd_size_q == 3'd1) &&
                            (rd_scsi_phase_q == 2'd1);
    wire [8:0] scsi_rd_addr_split = rd_scsi_dma16_lo
                                    ? 9'h101
                                    : scsi_local_addr(rd_addr_q);
    // 2026-08-19: writes DO take a split path — the per-strobe-bit
    // serializer above — and they need the identical grant inherit, so
    // this flag now carries both directions.  A SCSI write in flight
    // owns the flag (it also owns scsi_addr / scsi_wdata / the pb_wr
    // pulse), and a read only sees it while no SCSI write is busy —
    // the same arbitration the old form encoded.
    assign scsi_dma16_lo_beat = (wr_busy && (wr_slot_q == SLOT_SCSI))
                                ? wr_scsi_dma16_lo
                                : rd_scsi_dma16_lo;
    assign scsi_addr  = wr_busy && wr_slot_q == SLOT_SCSI ? scsi_local_addr(wr_addr_q) :
                        rd_busy && rd_slot_q == SLOT_SCSI ? scsi_rd_addr_split :
                                                             9'd0;
    // DMA-shim writes source their byte from the serializer's latched
    // copy (s_wdata is gone once the W handshake completed); register
    // writes keep the historical combinational fast path.
    assign scsi_wdata = wr_scsi_word_active ? wr_scsi_byte : wr_byte;
    assign scsi_wr    = wr_scsi_word_active ||
                        (pb_wr_active && (wr_slot_q == SLOT_SCSI));
    assign scsi_rd    = rd_scsi_dma_shim_active || (pb_rd_active && (rd_slot_q == SLOT_SCSI));

    // ASC: 12-bit BYTE-granular addr (4 KB register file).  Task #123:
    // SONORA control registers at +0x800..+0x80A are byte-addressable —
    // ROM writes `move.b d0, 0x50014801` (mode) vs `move.b d0, 0x50014802`
    // (chan_ctl) must land at pb_addr 0x801 and 0x802, not both collapsed
    // to the same word-aligned 0x800.
    //
    // wr_addr_q[1:0] is the byte-within-word offset from the CPU's original
    // byte-level awaddr (narrow_to_wide passes it through unmodified).  The
    // corresponding byte VALUE is already extracted by `wr_byte` above via
    // the wstrb-indicated lane, so the {addr, value} pair is self-consistent
    // for single-byte stores (which is how the ROM accesses these regs).
    //
    // Word-sized stores to ASC registers are undefined in the SONORA spec
    // and would collapse to the first byte under this scheme — matches
    // pre-existing behaviour for other pb_* peripherals.
    //
    // For reads, rd_addr_q[1:0] comes directly from AR — also byte-granular.
    //
    // The low 2 bits of the peripheral-local address replace the hard-
    // coded `00` that the word-aligned {addr[13:2]} form implied.  Upper
    // bits still mask down to 12 (ASC's addressable range) — we pass
    // addr[11:0] so the 16 KB mapped window aliases every 4 KB inside
    // ASC.  (SONORA only implements 4 KB.)
    // ASC address: multi-byte iteration walks AW[1:0] from base+0..base+N-1
    // (HIGH→LOW strobe walk → ascending byte addresses for big-endian
    // m68k stores).  Single-byte path keeps the historical wr_addr_q[11:0]
    // direct mapping.
    assign asc_addr  = wr_ser_on_asc
                            ? {wr_addr_q[11:2], wr_ser_addr_lsb}
                       : (wr_busy && wr_slot_q == SLOT_ASC) ? wr_addr_q[11:0]
                       : (rd_busy && rd_slot_q == SLOT_ASC) ? rd_addr_q[11:0]
                       :                                       12'd0;
    assign asc_wdata = wr_ser_on_asc ? wr_ser_byte : wr_byte;
    // pb_wr_active suppresses the W-capture-cycle pulse for ASC multi-byte
    // (so we don't double-pulse).  wr_ser_pulse_asc drives the per-iteration
    // pulse from the FSM during phase 0.
    assign asc_wr    = (pb_wr_active && (wr_slot_q == SLOT_ASC)) ||
                       wr_ser_pulse_asc;
    assign asc_rd    = pb_rd_active && (rd_slot_q == SLOT_ASC);

    // Multi-byte-write trace (informational).
    //
    // The master's contract for byte-granular slots is documented in the
    // byte-lane note at the top of this file: AWADDR is byte-granular, and
    // strobe bits identify which bytes of the active 32-bit lane carry
    // the data.  Multi-byte writes ARE supported via the wr_ser_* FSM
    // above (HIGH→LOW strobe walk, AW++ per byte; matches the LSU's
    // big-endian m68k_mem_strb / m68k_mem_wdata layout).
    //
    // Known live multi-hot producers, measured on MAME macquadra700 under
    // System 7.0.1 and 7.5.3 (docs/mame_periph_multihot_reachability.md):
    //   ASC     — MOVE.W to R_FIFOA/B_VOLUME_LR (0xF06/0xF26) from the ROM
    //             chime path, and MOVE.L to the FIFO ports from the OS
    //             Sound Manager;
    //   ORWELL  — MOVE.L init loop in the ROM (~91 LONG writes per boot);
    //   SONIC   — MOVE.L from the System 7 Ethernet driver's reset path.
    // ENET and ADBINJ are byte-granular but deliberately excluded — see
    // slot_serializes()'s comment for the reason (no m68k producer, and
    // ADBINJ's real producer uses the opposite host/debug convention).
    // Single-byte writes (the common case for VIA/SCC/SCSI/etc and most
    // ASC traffic) bypass the FSM via the single-byte fast path.
    //
    // Earlier revisions trapped multi-byte writes with $finish to flag
    // an unimplemented case.  Now that the FSM handles them correctly,
    // the trap is downgraded to an informational $display so post-mortem
    // logs still annotate when a multi-byte serialization fires.
    // synthesis translate_off
    always @(posedge clk) begin
        if (!rst && !pb_quiesce && wr_busy && wr_ser_slot && s_wvalid &&
            !wr_w_done && (wr_lane_strb_count > 3'd1)) begin
            $display("[%0t] peripheral_bus: multi-byte write serialized — slot=%0d wr_addr_q=0x%08x wr_lane_strb=0x%01x wr_lane_data=0x%08x count=%0d (HIGH->LOW strobe walk, AW++).",
                     $time, wr_slot_q, wr_addr_q, wr_lane_strb, wr_lane_data,
                     wr_lane_strb_count);
        end
        // Multi-hot beat on a slot that is NOT serialized (strided/aliased
        // decode, or SCC): only ONE byte can reach the device.  Loud in
        // simulation so a future regression naming such a producer is
        // found rather than silently truncated.  Informational, not fatal
        // — the same downgrade rationale as the block above.
        if (!rst && !pb_quiesce && wr_busy && wr_slot_is_pb && !wr_ser_slot &&
            (wr_slot_q != SLOT_SONIC) &&
            !(wr_slot_q == SLOT_SCSI && wr_addr_q[8]) &&
            s_wvalid && !wr_w_done && (wr_lane_strb_count > 3'd1)) begin
            $display("[%0t] peripheral_bus: NOTE multi-byte write to a non-serialized slot — slot=%0d wr_addr_q=0x%08x wr_lane_strb=0x%01x count=%0d; only one byte reaches the device.",
                     $time, wr_slot_q, wr_addr_q, wr_lane_strb,
                     wr_lane_strb_count);
        end
    end
    // synthesis translate_on

    // SWIM/IWM: Q700 16-register layout at 0x200 stride.
    assign iwm_addr  = wr_busy && wr_slot_q == SLOT_IWM ? wr_addr_q[12:9] :
                       rd_busy && rd_slot_q == SLOT_IWM ? rd_addr_q[12:9] :
                                                           4'd0;
    assign iwm_wdata = wr_byte;
    assign iwm_wr    = pb_wr_active && (wr_slot_q == SLOT_IWM);
    assign iwm_rd    = pb_rd_active && (rd_slot_q == SLOT_IWM);

    // ADB injection: 8-bit byte-granular within the 4 KB window.  The
    // sub-register is selected by addr[7:0] (low 8 bits of mac_off).
    // See fpga_top_peripherals.vh for the byte map (kbd enqueue, mouse
    // dx/dy/btn, status).
    // Byte-granular decode, but deliberately NOT serialized: the JTAG-AXI
    // host master writes this port with ONE meaningful byte in the
    // strobed lane at a WORD-ALIGNED AWADDR and all four strobes hot.
    // Serializing that would scatter the injected byte across four
    // sub-registers.  See slot_serializes()'s comment.
    assign adbinj_addr  = wr_busy && wr_slot_q == SLOT_ADBINJ ? wr_addr_q[7:0] :
                          rd_busy && rd_slot_q == SLOT_ADBINJ ? rd_addr_q[7:0] :
                                                                 8'd0;
    assign adbinj_wdata = wr_byte;
    assign adbinj_wr    = pb_wr_active && (wr_slot_q == SLOT_ADBINJ);
    assign adbinj_rd    = pb_rd_active && (rd_slot_q == SLOT_ADBINJ);

endmodule

`default_nettype wire
