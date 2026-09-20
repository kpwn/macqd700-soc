// MAME reference/excerpt/adaptation attribution: Copyright R. Belmont.
// MAME-derived portions retain BSD-3-Clause; original contributions are MIT.
// See THIRD_PARTY_NOTICES.md and LICENSES/MAME-BSD-3-Clause.txt.
// video.v — DAFB register-shim (Quadra 700 display controller front-door)
//
// Role
// ═══════════════════════════════════════════════════════════════════════
// The Quadra 700 ROM pokes 28+ distinct DAFB registers across the
// 0xF9800000..0xF98003FF window before it dares to draw anything.  Our
// scanner RTL is still intentionally small, but the ROM's framebuffer
// placement writes must be observable by scanout.  If we leave the
// register window dead-ended, the ROM wedges on the first DAFB access.
//
// This module is the **shim** that unblocks the ROM: a 256-word
// register file (0x00..0x3FF @ 4-byte stride) that ACKs every write,
// returns last-written on every read, hard-codes a couple of read-back
// values on the handful of registers the ROM treats as status / sense,
// and exports the ROM-programmed framebuffer base / stride / BPP latches
// for scanout.  The status register has a frame-tick-driven vblank latch
// (see the `frame_tick` port) so ROM polling observes a real once-per-frame
// event source instead of register aliasing or a free-running counter; the
// same enable-gated pending level is exported as `irq` for the VIA VBL
// route.
//
// What the ROM actually touches (per docs/dafb_audit.md):
//   +0x00..0x24  core control + monitor sense + IRQ + "first write" reg
//   +0x2C  DAFB test/version register
//   +0x100..0x168  PLL + H/V timing
//   +0x200/0x210/0x220  AC842 RAMDAC address / palette-data / PCBR byte regs
//   +0x300..0x3F0  DP8531 pixel-clock PLL register file (stride 0x10)
//
// AC842 RAMDAC tri-byte CLUT protocol (MAME dafb.cpp lines 710-789):
//   +0x200 W: m_pal_address := data[7:0]; m_pal_idx := 0
//   +0x200 R: m_pal_idx := 0; returns m_pal_address (low byte)
//   +0x210 W: byte writes R/G/B sub-component of CLUT[m_pal_address]
//             based on m_pal_idx; on idx==3 wrap m_pal_idx:=0,
//             m_pal_address++
//   +0x210 R: byte reads R/G/B sub-component of CLUT[m_pal_address]
//             based on m_pal_idx; on idx==3 wrap m_pal_idx:=0,
//             m_pal_address++
// We store the full 256-entry x 24-bit CLUT to be MAME-faithful since
// the address pointer is 8-bit and software may probe any entry.  MAME's
// dafb_base::screen_update() indexes ALL pixel depths (1/2/4/8bpp) through
// this SAME 256-entry palette (pens[value], value = 0/1, 0-3, 0-15, or
// 0-255 depending on mode) -- there is no separate low-depth CLUT.  The
// scanout side (linebuf_scanout.v) owns a 256x24 dual-clock BRAM CLUT fed
// directly from this module's RAMDAC write protocol via the
// clut_we/clut_waddr/clut_wdata export below -- see linebuf_scanout.v and
// video_top.v.  The former 16-entry `clut_rgb[383:0]` flat-bus export (a
// quasi-static-CDC "16-color legacy" shim) is retired.
//
// Interface
// ═══════════════════════════════════════════════════════════════════════
// Simple AXI4-lite slave, 32-bit addr + data, 4-byte WSTRB.  Ports
// follow the pattern of `debug_ctrl.v` and `dma_ctrl.v`.  The external
// decoder selects the 4 KB DAFB window on 0xF98xxxxx; internally we only
// implement the first 1 KB of real register storage.  Higher offsets
// ACK deterministically and read back as zero instead of aliasing into
// the live register file.
//
// Read latency: 1 cycle (registered RDATA off a 1-deep queue).
// Write latency: 1 cycle (BRESP returns OKAY the cycle after WVALID).
//
// Reset values
// ═══════════════════════════════════════════════════════════════════════
// All named control/status registers reset to 0.  The two big
// dynamically-indexed arrays (256x32 raw register storage + the 256x8x3
// AC842 RAMDAC CLUT) are LUTRAM-inferred and intentionally do NOT reset
// on a synchronous `rst` — see the comment at the reset block below.
// They come up zeroed at power-on (bitstream INIT / Verilator zero-init)
// but may hold stale last-written values across a debug full-reset.
// Reads of status-ish offsets bypass the storage where the real device
// returns live status.
//
// Notes
// ═══════════════════════════════════════════════════════════════════════
// * Writes are "last-value wins" across the full 32-bit word — WSTRB
//   lanes that are masked off leave the corresponding byte unchanged.
// * The low-depth CLUT registers at +0x300+n*0x10 are exposed as
//   RGB888 entries.  The ROM's early values are small 4-bit colour
//   codes, so writes update the live palette by decoding write_word[3:0].
// * The IRQ status register exposes sticky vblank in bit 0 and an
//   IRQ-enable-gated pending indication in bit 1.  Framebuffer config
//   arms the frame_tick-driven vblank generator, a rising edge on the
//   frame_tick input latches status bit 0, and a byte-lane-valid
//   write-1-to-clear at +0x20 acknowledges it.
// * Synchronous reset, active-high, per project convention.

`default_nettype none

module video #(
    // PLL reference frequency in Hz.  MAME ctor (dafb.cpp:83) uses
    //   m_pixel_clock(31334400)
    // as the reset value, before the CPU programs DP8531 dividers.  The
    // VCO update on reg 15 write follows
    //   vco = (PLL_REF_HZ / R) * N
    // with R / N derived from the 16 4-bit dp8531 register nibbles.
    parameter [31:0] PLL_REF_HZ = 32'd20_000_000,
    // Reset value for m_pixel_clock (MAME dafb.cpp:83).
    parameter [31:0] PLL_RESET_HZ = 32'd31_334_400
) (
    input  wire        clk,
    input  wire        rst,

    // ── AXI4-lite slave ─────────────────────────────────────────────
    input  wire [31:0] s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,

    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,

    output reg  [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,

    input  wire [31:0] s_axi_araddr,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,

    output reg  [31:0] s_axi_rdata,
    output reg  [1:0]  s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,

    // ── Live DAFB scanout state ────────────────────────────────────
    // These registers mirror the ROM-visible framebuffer placement
    // knobs the scanner path consumes.  Encoding follows MAME canonical
    // (`src/mame/apple/dafb.cpp`):
    //   fb_base_px   = byte offset into VRAM
    //                = (r_base_hi[11:0] << 9) | (r_base_lo[3:0] << 5)
    //   fb_stride_px = byte stride per source row
    //                = r_config[3] ? 1024 : (r_stride_raw << 2)
    //   fb_bpp_reg   = log2(BPP), decoded from AC842 PCBR @ +0x220 bits[4:2]:
    //                  0x00→1bpp, 0x08→2bpp, 0x10→4bpp, 0x18→8bpp, 0x1c→24bpp
    output wire [31:0] fb_base_px,
    output wire [31:0] fb_stride_px,
    output wire [31:0] fb_bpp_reg,
    // Source-pixels-per-byte log2: 1bpp→3 (8 px/byte), 2bpp→2, 4bpp→1, 8bpp→0.
    // Drives the scanner's line_rd_addr right-shift so each VRAM byte
    // emits 1<<bpp_shift output pixels at the appropriate sub-byte slice.
    //
    // NOTE: bpp_shift can only express depths of 8bpp OR LESS -- it encodes
    // pixels *per byte*.  It is meaningless for the multi-byte-per-pixel
    // depths (24bpp = 4 B/px of VRAM, see fb_bytes_per_px).  Consumers must
    // therefore qualify bpp_shift with `depth_supported` below and must NOT
    // infer bytes-per-pixel from it.  `fb_bytes_per_px` carries the honest
    // multi-byte term.
    output wire [2:0]  bpp_shift,
    // VRAM bytes consumed per source pixel, for the depths where that is >= 1:
    //   1/2/4bpp → 0 (sub-byte; use bpp_shift instead)
    //   8bpp     → 1
    //   24bpp    → 4   (NOT 3 -- see decode_bytes_per_px below)
    // Exported so the scanout datapath can size its per-row byte span
    // correctly.  Previously the only depth term was bpp_shift, which
    // reported 3'd0 ("1 byte per pixel") for 24bpp -- an outright lie that
    // made a 24bpp frame scan 1/4 of each row and push raw colour bytes
    // through the CLUT as if they were palette indices.
    output wire [2:0]  fb_bytes_per_px,
    // 1 iff the *scanout datapath* can actually render the currently
    // programmed depth.  This is deliberately distinct from "the ROM has
    // programmed a depth at all" (see r_pcbr_set / fb_ready below) and from
    // "the depth is a legal AC842 code" (see decode_bpp).  The scanner now
    // supports 1/2/4/8bpp (CLUT-indexed, sub-byte slicing) AND 24bpp
    // (direct colour, 4 B/px VRAM footprint, byte-phase fetch) -- so this is
    // high for all five mapped AC842 codes and low only for the three
    // genuinely unmapped ones (0x04/0x0C/0x14) and for "PCBR never written".
    // Downstream (scanout_placement_sync) gates placement commits on this
    // bit, so an unmapped code retains the last good placement instead of
    // scanning garbage -- the same fail-safe already applied to an
    // out-of-range base/stride.
    output wire        depth_supported,
    // Visible source dimensions decoded from MAME-canonical Swatch timing
    // params (HAL/HFP for hres, VAL/VFP for vres at +0x140..+0x160).  The
    // scanner uses these as runtime bounds + drives the integer-scale
    // selection (2× if both fit in 1920×1080, else 1×).  Both 12-bit
    // wide to cover up to 4095×4095 (MAME m_hres/m_vres are 12-bit fields).
    output wire [11:0] hres,
    output wire [11:0] vres,
    // ── 256-entry RAMDAC CLUT write export (core_clk domain) ────────
    // One-cycle pulse per completed RGB triple (fires when the blue
    // sub-component write lands, i.e. ramdac_pal_idx wraps 2->0).
    // clut_wdata is {R,G,B}; clut_waddr is the entry that was just
    // completed (ramdac_pal_address BEFORE the post-write autoincrement).
    // Consumed by linebuf_scanout.v's dual-clock 256x24 BRAM CLUT --
    // no CDC here by design (mid-frame writes land immediately, matching
    // real RAMDAC behaviour; see linebuf_scanout.v header).
    output reg          clut_we,
    output reg  [7:0]   clut_waddr,
    output reg  [23:0]  clut_wdata,
    output wire        irq,

    // ── Pixel-clock PLL output (DP8531) ─────────────────────────────
    // Computed pixel-clock frequency in Hz from the DP8531 register
    // nibble file at +0x300+n*0x10.  The VCO equation matches MAME
    // dafb.cpp:894-907 verbatim.  Reset value is PLL_RESET_HZ; the
    // register updates on every write to reg 15 (offset 0x3F0+3 in
    // MAME, 32-bit word at +0x3F0 for us — see comment block).  Wire
    // is read-back-only by default (no scanout coupling); see the
    // pll_pixel_clock note above the always-block.
    output wire [31:0] pll_pixel_clock,

    // ── TurboSCSI shim glue ─────────────────────────────────────────
    // DAFB owns m_scsi_ctrl[0/1] storage + the +0x24/+0x28 read-back
    // mirrors that fold the live DRQ from each NCR 53C9x bus into bit
    // 9 (MAME dafb.cpp:418-422).  The CPU polls these to know when to
    // issue a pseudo-DMA cycle to 0x5000_F100/0x5000_F101.
    //
    // scsi0_ctrl_out is the bus-1 control word last written to +0x24,
    // exposed to the TurboSCSI shim inside scsi.v so it can implement
    // the DRQ-check / DTACK-hold semantics on the DMA window.
    // scsi0_drq_in is the live DRQ from the NCR 53C9x bus.
    output wire [8:0]  scsi0_ctrl_out,
    input  wire        scsi0_drq_in,

    // ── Frame-tick vblank source ─────────────────────────────────────
    // Real per-frame tick, already synchronised into THIS module's clk
    // domain by the caller (see the instantiation site for the CDC).
    // `vblank_pending` latches on frame_tick's rising edge, replacing a
    // former free-running 1024-cycle counter (~10us @100MHz) that made
    // "vblank" fire ~1600x faster than a real 60Hz frame and had nothing
    // to do with the actual displayed frame boundary.
    // Safe to leave unconnected: an unconnected input floats (reads X/Z
    // in sim, is simply never driven true), so the rising-edge check
    // below never fires and vblank_pending stays at its reset value —
    // identical in effect to tying this pin to 0.  Used by tb targets
    // that don't care about vblank cadence (tb-asc-adjacent PIC/ADB tbs
    // never touch this module at all; tb-dafb drives it directly).
    input  wire        frame_tick,

    // ── Apple Display Sense code for the "plugged-in" monitor ────────
    // Was a compile-time `parameter [6:0] MONITOR_TYPE`; now an input so
    // the operator can change the emulated display over JTAG instead of
    // paying a ~50-minute bitstream per candidate code.  In fpga_top it
    // comes from the CPU debug-CSR (OFF_MON_SENSE 0x0005C) via the socket
    // signal cpu_mon_sense — which lives in the CPU's POR-only debug
    // reset domain, so a written value survives a CPU reset (and Mac OS
    // only re-probes sense at DAFB init after a reset, so that survival
    // is what makes the knob usable at all).
    //
    // This drives the +0x1C read-side passthrough when the CPU has not
    // asserted the extended-sense drive (bit 6 of monitor_sense).
    // Standard codes (MAME dafb.cpp:202-216):
    //   0  Mac 21" Color Display
    //   1  Mac Portrait B&W 15"
    //   2  Mac RGB 12" 512x384 (Rubik)
    //   3  Mac Two-Page B&W 21"
    //   6  Mac Hi-Res 12-14" 640x480 (the historical parameter default,
    //      and the POR default of the CSR that now drives this pin)
    //   7  no monitor
    // Extended codes set bit 0x40 plus bc/ac/ab nibbles per MAME's
    // ext(bc,ac,ab) macro (dafb.cpp:197-200).  The monitor sense state
    // machine in this module returns res^7 to match the inverse-sense
    // wiring (MAME dafb.cpp:414).
    //
    // Purely combinational into the +0x1C read path — nothing latches it,
    // so a change is visible on the very next read.  There is no reset
    // value to get wrong here; whoever drives the pin owns the default.
    input  wire [6:0]  monitor_sense
);

    // ── Address window ──────────────────────────────────────────────
    // 256 long-words = 1 KB.  Offset = addr[9:2].
    localparam ADDR_BITS = 8;
    localparam REG_WORDS = (1 << ADDR_BITS);
    localparam REG_BITS  = REG_WORDS * 32;

    // ── Register file (256x32) ──────────────────────────────────────
    // Two asynchronous read addresses (AXI readback and write-side merge)
    // plus byte write enables fit distributed RAM. Constant-address live
    // taps must stay outside this array: they previously caused the entire
    // 8192-bit array to become FFs despite the array-shaped declaration.
    (* ram_style = "distributed" *) reg [31:0] regs [0:REG_WORDS-1];
    // Only these words need continuously available, independent read ports.
    // Keep small mirrors rather than turning the entire shadow RAM into FFs.
    // Like the shadow RAM, they deliberately survive a warm reset.
    reg [2:0] shadow_swatch_ctrl;
    reg [31:0] shadow_cursor_line;
    reg [8:0] shadow_scsi_ctrl;

    // ── Named register offsets (long-word index) ────────────────────
    // Offsets match Q700 ROM trace from docs/dafb_audit.md §2.
    // Only the ones the ROM actively polls for a sensible read-back
    // get hard-coded responses below.
    // Register layout matches MAME canonical (src/mame/apple/dafb.cpp):
    //   +0x00 BASE_HI    (m_base bits 20-9)
    //   +0x04 BASE_LO    (m_base bits 8-5)
    //   +0x08 STRIDE     (stored value << 2 → byte stride)
    //   +0x0C TIMING_CTRL
    //   +0x10 CONFIG     (bit[3] = convolution → forces stride=1024)
    //   +0x14 BLOCK_CTRL
    //   +0x1C MONITOR_ID drive (write) / inverse-sense (read)
    //   +0x24 SCSI bus 1 ctrl mirror
    //   +0x28 SCSI bus 2 ctrl mirror
    //   +0x2C TEST/VERSION (test[8:0] | dafb_version<<9; Q700 dafb_version=1)
    //   +0x220 AC842 PCBR (RAMDAC) — bits[4:2] select pixel mode
    localparam [ADDR_BITS-1:0] REG_BASE_HI    = 8'h00; // +0x00
    localparam [ADDR_BITS-1:0] REG_BASE_LO    = 8'h01; // +0x04
    localparam [ADDR_BITS-1:0] REG_STRIDE     = 8'h02; // +0x08
    localparam [ADDR_BITS-1:0] REG_TIMING_CTRL= 8'h03; // +0x0C
    localparam [ADDR_BITS-1:0] REG_CONFIG     = 8'h04; // +0x10
    localparam [ADDR_BITS-1:0] REG_IRQ_ENABLE = 8'h07; // +0x1C write: local IRQ enable
    localparam [ADDR_BITS-1:0] REG_MON_SENSE  = 8'h07; // +0x1C read: DAFB monitor sense
    localparam [ADDR_BITS-1:0] REG_IRQ_STATUS = 8'h08; // +0x20 (ROM polls this)
    localparam [ADDR_BITS-1:0] REG_FIRST_HIT  = 8'h09; // +0x24 (ROM's first write)
    localparam [ADDR_BITS-1:0] REG_DAFB_TEST  = 8'h0B; // +0x2C, test[8:0] | version[10:9]
    localparam [ADDR_BITS-1:0] REG_SWATCH_BASE = 8'h40; // +0x100
    localparam [ADDR_BITS-1:0] REG_SWATCH_CTRL = 8'h41; // +0x104
    localparam [ADDR_BITS-1:0] REG_SWATCH_CURSOR_ACK  = 8'h43; // +0x10C
    localparam [ADDR_BITS-1:0] REG_SWATCH_VBLANK_ACK  = 8'h45; // +0x114
    localparam [ADDR_BITS-1:0] REG_SWATCH_CURSOR_LINE = 8'h46; // +0x118
    // ── Swatch programmable timing registers (MAME dafb.cpp:683-706) ──
    // All 12-bit wide on real silicon; reset = 0 per MAME ctor (dafb.cpp:94-95
    // `std::fill(...m_horizontal_params/m_vertical_params, 0)`).  Word-index
    // values below are byte-offset / 4.  MAME line refs are for /tmp/mame_src/dafb.cpp.
    //   Horizontal params (m_horizontal_params[0..9], MAME dafb.cpp:683-693)
    localparam [ADDR_BITS-1:0] REG_SWATCH_HSERR = 8'h49; // +0x124 MAME 0x24
    localparam [ADDR_BITS-1:0] REG_SWATCH_HLFLN = 8'h4A; // +0x128 MAME 0x28
    localparam [ADDR_BITS-1:0] REG_SWATCH_HEQ   = 8'h4B; // +0x12C MAME 0x2c
    localparam [ADDR_BITS-1:0] REG_SWATCH_HSP   = 8'h4C; // +0x130 MAME 0x30
    localparam [ADDR_BITS-1:0] REG_SWATCH_HBWAY = 8'h4D; // +0x134 MAME 0x34
    localparam [ADDR_BITS-1:0] REG_SWATCH_HBRST = 8'h4E; // +0x138 MAME 0x38
    localparam [ADDR_BITS-1:0] REG_SWATCH_HBP   = 8'h4F; // +0x13C MAME 0x3c
    localparam [ADDR_BITS-1:0] REG_SWATCH_HAL   = 8'h50; // +0x140 MAME 0x40 (m_hres = HFP - HAL)
    localparam [ADDR_BITS-1:0] REG_SWATCH_HFP   = 8'h51; // +0x144 MAME 0x44
    localparam [ADDR_BITS-1:0] REG_SWATCH_HPIX  = 8'h52; // +0x148 MAME 0x48
    //   Vertical params (m_vertical_params[0..6], MAME dafb.cpp:697-705)
    localparam [ADDR_BITS-1:0] REG_SWATCH_VHLINE = 8'h53; // +0x14C MAME 0x4c
    localparam [ADDR_BITS-1:0] REG_SWATCH_VSYNC  = 8'h54; // +0x150 MAME 0x50
    localparam [ADDR_BITS-1:0] REG_SWATCH_VBPEQ  = 8'h55; // +0x154 MAME 0x54
    localparam [ADDR_BITS-1:0] REG_SWATCH_VBP    = 8'h56; // +0x158 MAME 0x58
    localparam [ADDR_BITS-1:0] REG_SWATCH_VAL    = 8'h57; // +0x15C MAME 0x5c (m_vres = (VFP-VAL)>>1)
    localparam [ADDR_BITS-1:0] REG_SWATCH_VFP    = 8'h58; // +0x160 MAME 0x60
    localparam [ADDR_BITS-1:0] REG_SWATCH_VFPEQ  = 8'h59; // +0x164 MAME 0x64
    localparam [ADDR_BITS-1:0] REG_SWATCH_END  = 8'h7F; // +0x1FC
    localparam [ADDR_BITS-1:0] REG_RAMDAC_ADDR   = 8'h80; // +0x200
    localparam [ADDR_BITS-1:0] REG_RAMDAC_DATA   = 8'h84; // +0x210
    localparam [ADDR_BITS-1:0] REG_RAMDAC_PBCTRL = 8'h88; // +0x220
    localparam [ADDR_BITS-1:0] REG_CLUT_BASE   = 8'hC0; // +0x300, stride 0x10
    // The +0x300-+0x3FF window is mapped to the DP8531 pixel-clock PLL in
    // MAME (dafb.cpp:76 + 877-910) -- a 16-entry nibble register file, n =
    // aw_idx[5:2] (= byte-offset / 0x10), captured into m_dp8531_regs[n].
    // This matches MAME dafb.cpp:889 "m_dp8531_regs[offset>>4] = data & 0xf".
    // (The former "16-entry low-depth CLUT" overlay on this same window
    // is retired -- the CLUT is now exclusively the 256-entry AC842
    // RAMDAC table at +0x200/+0x210, see the header comment.)
    localparam [31:0] DAFB_VERSION_BITS = 32'h0000_0200; // original discrete DAFB

    // The AC842 RAMDAC window is byte-wide behind the 32-bit DAFB bus.
    // MAME's Q700 model returns the low byte for the palette address
    // register at +0x200 and pixel-bus-control register at +0x220.

    // ── AW channel ──────────────────────────────────────────────────
    // Simple 1-deep handshake: we always accept AW and W in the same
    // cycle (or either earlier).  awready rises combinationally.
    reg        aw_pending;
    reg [9:0]  aw_word_idx;
    reg        aw_in_range;
    reg [31:0] aw_addr;
    reg        w_pending;
    reg [31:0] w_data;
    reg [3:0]  w_strb;
    reg        vblank_pending;
    reg        frame_tick_q;
    reg        swatch_cursor_pending;
    reg [31:0] swatch_cursor_countdown;

    // ── AC842 RAMDAC palette state (MAME dafb.cpp lines 710-789) ────
    // 256-entry x 24-bit CLUT, byte-wide address pointer, 2-bit
    // R/G/B sub-byte index.  Written/read one byte at a time via the
    // tri-byte protocol at +0x210; m_pal_address auto-advances on
    // wrap so software can stream a contiguous block of entries.
    reg [7:0]  ramdac_pal_address;
    reg [1:0]  ramdac_pal_idx;
    // 2026-05-22 — converted from flat 2048-bit vectors to indexable
    // 256x8 arrays.  As flat vectors with dynamic bit-select, Vivado
    // synthesised this as ~6144 flip-flops PLUS huge 256:1 byte-select
    // muxes for both read and write (multi-K LUT cost).  As proper
    // arrays, the synth tool infers compact LUTRAM (≈32 LUTs per
    // channel = ~96 LUTs total) and no read mux is required.  Net
    // saving expected: several thousand LUTs in the DAFB shim.
    reg [7:0]  ramdac_clut_r [0:255];
    reg [7:0]  ramdac_clut_g [0:255];
    reg [7:0]  ramdac_clut_b [0:255];

    // ── Swatch programmable timing register storage ─────────────────
    // 12-bit wide each, mirroring MAME dafb.cpp m_horizontal_params /
    // m_vertical_params arrays.  Holding these as named regs (instead of
    // dropping them into the regs_flat dead-letter store) lets ROM
    // diagnostics + mode-probe code read back the values they wrote and
    // gives downstream consumers (interlace, geometry decode) explicit
    // hooks.  See MAME dafb.cpp:683-706 (write decode) and 611-616 (read
    // decode); reset values are 0 per MAME dafb.cpp:94-95.
    reg [11:0] r_hserr;     // MAME dafb.cpp:683 HSERR
    reg [11:0] r_hlfln;     // MAME dafb.cpp:684 HLFLN
    reg [11:0] r_heq;       // MAME dafb.cpp:685 HEQ
    reg [11:0] r_hsp;       // MAME dafb.cpp:686 HSP
    reg [11:0] r_hbway;     // MAME dafb.cpp:687 HBWAY
    reg [11:0] r_hbrst;     // MAME dafb.cpp:688 HBRST
    reg [11:0] r_hbp;       // MAME dafb.cpp:689 HBP
    reg [11:0] r_hal;       // MAME dafb.cpp:690 HAL
    reg [11:0] r_hfp;       // MAME dafb.cpp:691 HFP
    reg [11:0] r_hpix;      // MAME dafb.cpp:692 HPIX
    reg [11:0] r_vhline;    // MAME dafb.cpp:697 VHLINE
    reg [11:0] r_vsync;     // MAME dafb.cpp:698 VSYNC
    reg [11:0] r_vbpeq;     // MAME dafb.cpp:699 VBPEQ
    reg [11:0] r_vbp;       // MAME dafb.cpp:700 VBP
    reg [11:0] r_val;       // MAME dafb.cpp:701 VAL
    reg [11:0] r_vfp;       // MAME dafb.cpp:702 VFP
    reg [11:0] r_vfpeq;     // MAME dafb.cpp:703 VFPEQ

    // ── Monitor sense (MAME dafb.cpp:387-415, 469-471) ───────────────
    // m_monitor_id is the 3-bit drive pattern the CPU latches into
    // every +0x1C write: m_monitor_id = (data & 0x7) ^ 7.  Reset = 0
    // (matches MAME dafb.cpp:89).  Whether the read-side response
    // engages the extended convolution is determined by bit 6 of
    // `monitor_sense` (the "user plugged in this display" input —
    // equivalent of MAME's m_monitor_config->read()),
    // NOT by anything in the data word the CPU writes (MAME does
    // not capture data&0x40 anywhere in dafb_w; the protocol
    // assumes bit 0x40 of mon is fixed by the user).
    reg [2:0]  r_monitor_id;

    // ── DP8531 pixel-clock PLL (MAME dafb.cpp:877-910) ───────────────
    // 16 register nibbles @ 0x300+n*0x10.  Reg layout (per dafb.cpp):
    //   regs[0..3]  → n_modulus (lo..hi nibbles of 16-bit field)
    //   regs[4..6]  → R divider (lo..hi nibbles of 12-bit field)
    //   regs[9]     → P divider exponent (P = 1 << regs[9])
    // Other slots (7,8,10..14) are stored but unused by the equation.
    // VCO recompute fires on a write to reg 15 (MAME dafb.cpp:892).
    reg [3:0]  dp8531_regs [0:15];
    reg [31:0] r_pixel_clock;
    integer di;

    assign pll_pixel_clock = r_pixel_clock;

    // ── Monitor sense response (MAME dafb.cpp:387-415) ───────────────
    // The CPU drives 3 sense bits on the connector; the EDID-pre-cursor
    // monitor differentially returns bits per its type.  When the CPU has
    // NOT asserted the extended drive (write side stored bit 6 = 0), the
    // standard 3-bit code from monitor_sense[2:0] passes through.  When
    // the CPU asserts extended-drive AND the user plugged in an extended
    // monitor (monitor_sense bit 6 = 1), the response is computed from
    // m_monitor_id selecting a window into monitor_sense[5:0].
    // Final read returns res^7 to match the inverse-sense wiring
    // (MAME dafb.cpp:414).
    function [2:0] sense_response;
        input [2:0] monitor_id;
        input [6:0] mon;
        reg   [2:0] res;
        begin
            if (mon[6]) begin
                res = 3'b111;
                if (monitor_id == 3'h4)
                    // bc field = mon[5:4]; AND res with (4 | (m5 << 1) | m4).
                    res = res & {1'b1, mon[5], mon[4]};
                if (monitor_id == 3'h2)
                    // ac field = mon[3:2]; AND res with ((m3 << 2) | 2 | m2).
                    res = res & {mon[3], 1'b1, mon[2]};
                if (monitor_id == 3'h1)
                    // ab field = mon[1:0]; AND res with ((m1 << 2) | (m0 << 1) | 1).
                    res = res & {mon[1], mon[0], 1'b1};
            end else begin
                res = mon[2:0];
            end
            sense_response = res;
        end
    endfunction

    wire [2:0] mon_sense_inv = sense_response(r_monitor_id, monitor_sense) ^ 3'b111;

    function [31:0] merge_wstrb;
        input [31:0] prev;
        input [31:0] data;
        input [3:0]  strb;
        integer bi;
        begin
            merge_wstrb = prev;
            for (bi = 0; bi < 4; bi = bi + 1) begin
                if (strb[bi])
                    merge_wstrb[bi*8 +: 8] = data[bi*8 +: 8];
            end
        end
    endfunction

    wire [31:0] write_prev = aw_in_range ? regs[aw_word_idx[7:0]]
                                         : 32'd0;
    wire [31:0] write_word = merge_wstrb(write_prev, w_data, w_strb);
    wire [7:0]  aw_idx = aw_word_idx[7:0];
    wire [7:0]  ar_idx = ar_word_idx[7:0];
    wire [31:0] shadow_read = regs[ar_idx];
    integer shadow_byte;
    always @(posedge clk) begin
        if (!rst && do_write && aw_in_range) begin
            for (shadow_byte = 0; shadow_byte < 4; shadow_byte = shadow_byte + 1)
                if (w_strb[shadow_byte])
                    regs[aw_idx][shadow_byte*8 +: 8] <= w_data[shadow_byte*8 +: 8];
            if (aw_idx == REG_SWATCH_CTRL) shadow_swatch_ctrl <= write_word[2:0];
            if (aw_idx == REG_SWATCH_CURSOR_LINE) shadow_cursor_line <= write_word;
            if (aw_idx == REG_FIRST_HIT) shadow_scsi_ctrl <= write_word[8:0];
        end
    end
    // PLL: same 0x300 window, but use the same 16-entry stride (n =
    // (offset & 0xFF) >> 4 = aw_idx[5:2]).  Only the low nibble of
    // write_word becomes the dp8531 register value, matching MAME
    // dafb.cpp:889 "data & 0xf".  We capture on every aligned 32-bit
    // write (wstrb[0] valid) — software always writes byte 3 of the
    // four-byte word, which lands in wdata[7:0] under our big-endian
    // wstrb mapping (wstrb[0] gates wdata[7:0]).
    wire        write_is_pll = aw_in_range
                            && (aw_idx[7:6] == REG_CLUT_BASE[7:6])
                            && (aw_idx[1:0] == 2'b00)
                            && w_strb[0];
    wire [3:0]  write_pll_idx = aw_idx[5:2];
    wire        write_base_hi   = aw_in_range && (aw_idx == REG_BASE_HI);
    wire        write_base_lo   = aw_in_range && (aw_idx == REG_BASE_LO);
    wire        write_stride    = aw_in_range && (aw_idx == REG_STRIDE);
    wire        write_config    = aw_in_range && (aw_idx == REG_CONFIG);
    wire        write_pcbr      = aw_in_range && (aw_idx == REG_RAMDAC_PBCTRL);
    wire        write_irq_ack   = aw_in_range && (aw_idx == REG_IRQ_STATUS);
    wire        write_irq_w1c   = write_irq_ack && w_strb[0] && w_data[0];
    wire        write_swatch_ctrl = aw_in_range && (aw_idx == REG_SWATCH_CTRL);
    wire        write_swatch_cursor_ack = aw_in_range && (aw_idx == REG_SWATCH_CURSOR_ACK);
    wire        write_swatch_vblank_ack = aw_in_range && (aw_idx == REG_SWATCH_VBLANK_ACK);
    // RAMDAC: writes hit only when low byte is being driven (wstrb[0] valid).
    // MAME treats the AC842 register window as byte-wide behind the 32-bit
    // bus; the address/data register is the low byte of the 32-bit word.
    wire        write_ramdac_addr = aw_in_range && (aw_idx == REG_RAMDAC_ADDR) && w_strb[0];
    wire        write_ramdac_data = aw_in_range && (aw_idx == REG_RAMDAC_DATA) && w_strb[0];
    // The two monochrome displays in MAME's monitor list (dafb.cpp:205,207):
    //   1 = Mac Portrait Display (B&W 15" 640x870)
    //   3 = Mac Two-Page Display (B&W 21" 1152x870)
    // `ramdac_w` special-cases exactly these two codes (dafb.cpp:760) --
    // they are driven from the blue channel alone.  Compared against the
    // whole 7-bit sense value, matching MAME's `m_monitor_config->read()`.
    wire        mono_monitor = (monitor_sense == 7'd1) || (monitor_sense == 7'd3);
    // Swatch timing-register window: word-index 0x49..0x59 (= byte 0x124..0x164).
    // Per MAME dafb.cpp:683-706 these are the 17 H/V programmable timing regs.
    wire        write_swatch_timing = aw_in_range
                                    && (aw_idx >= REG_SWATCH_HSERR)
                                    && (aw_idx <= REG_SWATCH_VFPEQ);
    // MAME `dafb_base::recalc_ints()` asserts the DAFB slot IRQ while ANY
    // int_status bit is set: bit0 = VBL, bit2 = cursor scanline.  There is NO
    // separate IRQ-enable register -- whether a source may fire is decided by
    // the SWATCH_CTRL (+0x104) timer-enable bits (bit0 VBL, bit2 cursor), and
    // each source is acked through its own address (+0x114 VBL, +0x10C
    // cursor); see dafb.cpp swatch_w cases 0x4 / 0xc / 0x14.
    //
    // This previously read `vblank_pending && regs[REG_IRQ_ENABLE][0]`, which
    // was wrong twice over: it gated on an INVENTED enable at +0x1C (MAME has
    // no such register -- dafb +0x1C reads back monitor sense), and it ignored
    // `swatch_cursor_pending` completely.  Mac OS drives the built-in video
    // slot interrupt from the CURSOR SCANLINE source and acks it by writing
    // +0x10C -- ROM slot-$F handler at 0x00007574 does `clr.l (0x10C,a0)` with
    // a0 = 0xF9800000.  Asserting from vblank_pending while the OS acked the
    // cursor source meant the slot line could NEVER be released: VIA2 PA6
    // stayed low and the ROM's slot dispatcher at 0x40806ECA re-dispatched
    // forever, starving every other interrupt.  HW-observed 2026-07-25 on
    // build 0xDC7E299E: VIA2 PA = 0xBF, exc_count frozen, ADB never polled.
    wire        irq_observable  = vblank_pending || swatch_cursor_pending;

    // ── DAFB scanout state (MAME-canonical decode) ──────────────────
    // Stored register values (MAME masks all dafb_w writes with 0xfff
    // for the core control / Swatch / clockgen blocks; we keep the
    // full word here and let downstream consumers slice as needed).
    reg [11:0] r_base_hi;       // +0x00 dafb_w: m_base bits[20:9]
    reg [3:0]  r_base_lo;       // +0x04 dafb_w: m_base bits[8:5]
    reg [29:0] r_stride_raw;    // +0x08 dafb_w: m_stride = stored<<2
    // +0x10 dafb_w.  MAME's m_config is consumed at exactly TWO bit
    // positions (dafb.cpp:266 / :844 / :861) and nowhere else:
    //     bit[3] convolution -- stride forced to 1024, hres /= clockdiv, -23
    //     bit[2] interlace   -- vres <<= 1
    // Every other bit is stored and read back at +0x10 with NO behavioural
    // effect, in MAME and here.  In particular bit[1], which the Q700
    // Monitors control panel flips when selecting Millions (0x30 -> 0x32),
    // is NOT a depth select: the depth comes solely from AC842 PCBR bits
    // [4:2] (see decode_bpp below).  Recorded here because a live-board
    // register diff makes bit[1] look load-bearing and it is not.
    reg [11:0] r_config;
    reg [7:0]  r_pcbr;          // +0x220 AC842 PCBR (bits[4:2] = mode)

    // Decoded outputs.  The ARITHMETIC lives in mode_decode (stage 2,
    // instantiated below); this file owns the register file and the WHEN.
    wire [31:0] fb_base_programmed =
        {11'd0, r_base_hi, 9'd0} | {23'd0, r_base_lo, 5'd0};
    // MAME's Q700 512x384 fixup WRITES m_base (dafb.cpp:835) -- it is a
    // STORED override that a later +0x00/+0x04 base write replaces, not a
    // live predicate on the current Swatch pair.  Set when recalc_mode()
    // samples a 512-wide raw geometry; cleared by any base write.
    reg         r_base_override_512;
    assign fb_base_px   = r_base_override_512 ? 32'h0000_1000
                                              : md_fb_base_bytes;
    assign fb_stride_px = md_bytes_per_row;
    // BPP code from AC842 PCBR bits[4:2].  Map to actual bits-per-pixel.
    // Drives 0 until the ROM writes the PCBR register so fb_ready stays
    // low through the early DAFB init writes — VBL IRQ should not fire
    // before the framebuffer mode has been chosen.
    reg        r_pcbr_set;

    // ── Stage 2: mode_decode (rtl/mac/mode_decode.v) ─────────────────
    // ALL DAFB mode arithmetic now lives in one pure, stateless module --
    // geometry, clockdiv, convolution, the Q700 512x384 fixup and the AC842
    // depth table.  This file keeps the Mac-facing register file and the bus
    // side effects and does NO geometry maths.  See
    // docs/video_path_review.md §4.1 stage 2 for why, and the mode_decode
    // header for the MAME line-by-line derivation of every term.
    wire [11:0] md_src_w_px;
    wire [11:0] md_src_h_px;
    wire [2:0]  md_bpp_shift;
    wire [31:0] md_bytes_per_row;
    wire [31:0] md_fb_base_bytes;
    wire [31:0] md_lut_depth;
    wire [2:0]  md_bytes_per_px;
    wire        md_depth_supported;
    wire        md_base_override_512;

    mode_decode u_mode_decode (
        .reg_hal           (r_hal),
        .reg_hfp           (r_hfp),
        .reg_val           (r_val),
        .reg_vfp           (r_vfp),
        .reg_config        (r_config),
        .reg_pcbr          (r_pcbr),
        .reg_pcbr_set      (r_pcbr_set),
        .reg_base_bytes    (fb_base_programmed),
        .reg_stride_words  (r_stride_raw),
        .src_w_px          (md_src_w_px),
        .src_h_px          (md_src_h_px),
        .bpp_shift         (md_bpp_shift),
        .bytes_per_row     (md_bytes_per_row),
        .fb_base_bytes     (md_fb_base_bytes),
        .lut_depth         (md_lut_depth),
        .bytes_per_px      (md_bytes_per_px),
        .depth_supported   (md_depth_supported),
        .base_override_512 (md_base_override_512)
    );

    // ── WHEN the descriptor is sampled ───────────────────────────────
    // MAME's `recalc_mode()` is reachable from EXACTLY ONE place in
    // dafb.cpp: the AC842 PCBR write, `ramdac_w` case 0x20 (dafb.cpp:816).
    // Enumerated over the whole file the call sites are :816 (dafb_base),
    // :1164 (q950), :1291 (memc), :1433 (memcjr) -- all four inside a
    // `ramdac_w`.  `swatch_w` (HAL/HFP/VAL/VFP) and `dafb_w` (base / stride /
    // config) NEVER recompute.  So m_hres/m_vres are a SNAPSHOT taken when
    // the driver writes the PCBR, not a live function of the Swatch pair.
    //
    // THIS IS THE `hres` BUG, and it is a WHEN bug, not a formula bug.  The
    // clockdiv that scales hres lives in the same register that selects the
    // depth, while HFP/HAL live in the Swatch block, and a mode set is many
    // separate CPU writes.  Evaluated live, `hres` is formed from whatever
    // mixture of old and new registers happens to be latched at that instant,
    // and every mixture is a wrong resolution.  Captured from MAME 0.285
    // macqd700 boots, one per monitor-sense code:
    //     sense 0x6D  832x624 : HFP-HAL = 0x22b-0x08b = 416, PCBR 0xa0 -> cd 2
    //     sense 0x00  1152x870: HFP-HAL = 0x160-0x040 = 288, PCBR 0xc0 -> cd 4
    // The board reported hres=1664 for an 832-wide mode.  1664 is 416 << 2:
    // the 832x624 Swatch pair scaled by the 1152x870 clockdiv.  The misread
    // input is the clockdiv, read from the wrong mode epoch.  Sampled the way
    // MAME samples it, that mixture is not representable at all.
    //
    // The sample is taken one cycle AFTER the PCBR write commits, so r_pcbr
    // already carries the new value -- MAME assigns m_ac842_pbctrl and then
    // calls recalc_mode() within the same case.  A cycle of latency here is
    // free: the video path's budget is ~67 output lines
    // (docs/video_path_review.md §4.1).
    reg        recalc_pulse;
    reg [11:0] r_hres_latched;
    reg [11:0] r_vres_latched;
    assign hres = r_hres_latched;
    assign vres = r_vres_latched;

    // The depth outputs stay combinational.  They are pure functions of
    // r_pcbr alone, and r_pcbr changes ONLY on the same write that triggers
    // recalc_mode() -- MAME sets m_mode and calls recalc_mode() in the same
    // `ramdac_w` case (dafb.cpp:791-816) -- so latching them would be a
    // no-op with an extra cycle of skew against nothing.
    assign fb_bpp_reg      = md_lut_depth;
    assign bpp_shift       = md_bpp_shift;
    assign fb_bytes_per_px = md_bytes_per_px;
    // Before the ROM has written PCBR at all there is no depth to render, so
    // depth_supported stays low and the scanner holds its reset placement.
    assign depth_supported = md_depth_supported;

    // ── fb_ready: "has the ROM programmed the framebuffer yet" ────────
    // This gate exists ONLY to hold the VBL interrupt off through early DAFB
    // init (see the comment above r_pcbr_set).  It must NOT encode "is this
    // depth renderable".
    //
    // It used to read `fb_bpp_reg != 32'd0`, which conflated the two: every
    // PCBR code that decode_bpp leaves unmapped (0x04, 0x0C, 0x14) decodes to
    // 0, which drove fb_ready low, which killed `vblank_tick` (below), which
    // meant `vblank_pending` never set and the DAFB VBL interrupt went DEAD.
    // On this platform that is catastrophic rather than cosmetic: the $0160
    // bit-6 VBL guard gates ALL deferred tasks, so a dead DAFB VBL freezes
    // the cursor and starves level-2 slot dispatch.
    //
    // Those three codes are reachable in practice, not theoretical.  A driver
    // probing for AC842a-style 15bpp support writes a PCBR with bits 2:1 set
    // (see the 16bpp derivation note above decode_bpp) -- e.g. 0x06 masks to
    // 0x04, 0x16 masks to 0x14.  On our AC842 that is an unmapped code, and
    // under the old gate merely *probing* for a depth we do not have would
    // have taken the VBL down permanently.
    //
    // r_pcbr_set is exactly the intended condition and already exists.
    // fb_bpp_reg stays an honest 0 for unmapped codes; the VBL survives.
    wire        fb_ready        = (fb_base_px   != 32'd0)
                               && (fb_stride_px != 32'd0)
                               && r_pcbr_set;
    // Rising edge of the (already clk-domain-synchronised) frame_tick
    // input.  Gated on fb_ready so the shim doesn't manufacture vblank
    // events before the ROM has finished programming the framebuffer
    // placement/depth registers, matching the previous counter's gating.
    wire        vblank_tick     = fb_ready && frame_tick && !frame_tick_q;
    assign irq = irq_observable;

    function [31:0] swatch_cursor_delay_cycles;
        input [31:0] cursor_line;
        begin
            // MAME schedules Swatch cursor IRQ by screen position.  The
            // bridge advances this shim in CPU-cycle units, so use the
            // Q700 mode's observed CPU cycles per scanline plus the fixed
            // bridge/screen phase from the enable write to the next
            // matching cursor scanline event.
            //
            // Recalibrated 2026-07-06: original constants (1494/106861/
            // 108355) were fit against MAME's DEFAULT/fallback screen
            // timing (htotal=896, vtotal=525 @ 31.3344MHz pixel clock).
            // Live HW readback of the Q700 mode actually configured by
            // the ROM (REG_SWATCH_HPIX=862, REG_SWATCH_VFPEQ>>1=523)
            // gives real cycles/scanline @ this build's 100MHz core
            // clock of ~2751, not 1494 (ratio 1.8414x). Scaled all three
            // constants by that ratio. This is still a phase-BLIND linear
            // approximation (no real per-scanline raster tracking in
            // this shim), just recalibrated to the right scale for the
            // mode actually in use.
            if (cursor_line[11:0] == 12'd0)
                swatch_cursor_delay_cycles = 32'd199521;
            else
                swatch_cursor_delay_cycles = ({20'd0, cursor_line[11:0]} * 32'd2751) + 32'd196770;
        end
    endfunction

    wire aw_fire = s_axi_awvalid && !aw_pending;
    wire w_fire  = s_axi_wvalid  && !w_pending;
    assign s_axi_awready = !aw_pending;
    assign s_axi_wready  = !w_pending;

    // A write commits when both AW and W have landed AND the previous
    // B response (if any) has been acknowledged.
    wire do_write = aw_pending && w_pending && !s_axi_bvalid;

    always @(posedge clk) begin
        if (rst) begin
            aw_pending   <= 1'b0;
            w_pending    <= 1'b0;
            aw_word_idx  <= 10'd0;
            aw_in_range  <= 1'b0;
            aw_addr      <= 32'd0;
            w_data       <= 32'd0;
            w_strb       <= 4'd0;
            s_axi_bvalid <= 1'b0;
            s_axi_bresp  <= 2'b00;
            r_base_hi    <= 12'd0;
            r_base_lo    <= 4'd0;
            r_stride_raw <= 30'd0;
            r_config     <= 12'd0;
            r_pcbr       <= 8'd0;
            r_pcbr_set   <= 1'b0;
            recalc_pulse        <= 1'b0;
            r_hres_latched      <= 12'd0;
            r_vres_latched      <= 12'd0;
            r_base_override_512 <= 1'b0;
            vblank_pending <= 1'b0;
            frame_tick_q   <= 1'b0;
            swatch_cursor_pending <= 1'b0;
            swatch_cursor_countdown <= 32'd0;
            // Swatch timing regs reset to 0 per MAME dafb.cpp:94-95.
            r_hserr  <= 12'd0;
            r_hlfln  <= 12'd0;
            r_heq    <= 12'd0;
            r_hsp    <= 12'd0;
            r_hbway  <= 12'd0;
            r_hbrst  <= 12'd0;
            r_hbp    <= 12'd0;
            r_hal    <= 12'd0;
            r_hfp    <= 12'd0;
            r_hpix   <= 12'd0;
            r_vhline <= 12'd0;
            r_vsync  <= 12'd0;
            r_vbpeq  <= 12'd0;
            r_vbp    <= 12'd0;
            r_val    <= 12'd0;
            r_vfp    <= 12'd0;
            r_vfpeq  <= 12'd0;
            r_monitor_id       <= 3'd0;
            r_pixel_clock      <= PLL_RESET_HZ;
            for (di = 0; di < 16; di = di + 1)
                dp8531_regs[di] <= 4'd0;
            clut_we    <= 1'b0;
            clut_waddr <= 8'd0;
            clut_wdata <= 24'd0;
            // NOTE: `regs` (256x32) and `ramdac_clut_r/g/b` (256x8 each) are
            // intentionally NOT reset here.  A synchronous reset loop over a
            // dynamically-indexed array forces per-entry reset muxing, which
            // defeats LUTRAM inference. Cold-boot init is
            // covered by bitstream INIT values on real hardware and by
            // the simulator's zero-init in sim; the ROM reprograms every
            // register (including the three below) before relying on any
            // of them, so this is invisible to a normal cold boot.
            //
            // This is NOT just "stale CLUT readback" — three LIVE,
            // control mirrors below do NOT clear on a debug-only full-reset
            // (JTAG/VIO, no power-cycle):
            //   - shadow_scsi_ctrl = regs[REG_FIRST_HIT][8:0]
            //     TurboSCSI bus-1 ctrl word into scsi.v's DMA gating.
            //   - shadow_cursor_line = regs[REG_SWATCH_CURSOR_LINE]
            //     supplies the cursor re-arm delay.
            //   - shadow_swatch_ctrl = regs[REG_SWATCH_CTRL][2:0]
            //     cursor auto-arm gate / VBL auto-arm gate.
            // See docs/uarch_decisions.md #1 for the full writeup. A debug
            // full-reset is therefore NOT equivalent to a ROM cold boot for
            // DAFB state — a debug flow that needs a guaranteed-clean DAFB
            // slate must explicitly re-program +0x24/+0x1C/+0x104 (or any
            // other regs[]-backed offset) after the reset.
            ramdac_pal_address <= 8'd0;
            ramdac_pal_idx     <= 2'd0;
        end else begin
            frame_tick_q <= frame_tick;
            // MAME arms the VBL timer only while SWATCH_CTRL (+0x104) bit0
            // (VBL enable) is set; with that bit clear the VBL source cannot
            // raise int_status bit0 at all (dafb.cpp swatch_w case 0x4).
            if (vblank_tick && shadow_swatch_ctrl[0]) begin
                vblank_pending <= 1'b1;
            end

            // clut_we is a single-cycle pulse -- default low each cycle,
            // set below when a RAMDAC blue-component write completes a
            // full RGB triple.
            clut_we <= 1'b0;

            if (shadow_swatch_ctrl[2] && !swatch_cursor_pending) begin
                if (swatch_cursor_countdown == 32'd0)
                    swatch_cursor_pending <= 1'b1;
                else
                    swatch_cursor_countdown <= swatch_cursor_countdown - 32'd1;
            end

            // ── recalc_mode() sample ──────────────────────────────
            // One cycle after a PCBR write commits, so r_pcbr (and therefore
            // every mode_decode output) already carries the new value.
            // Placed BEFORE the do_write block below so that a base write
            // landing in this same cycle wins the r_base_override_512 race --
            // MAME's fixup writes m_base, and a later dafb_w base write
            // replaces it.
            recalc_pulse <= do_write && aw_in_range && write_pcbr;
            if (recalc_pulse) begin
                r_hres_latched      <= md_src_w_px;
                r_vres_latched      <= md_src_h_px;
                r_base_override_512 <= md_base_override_512;
            end

            // Latch AW
            if (aw_fire) begin
                aw_pending <= 1'b1;
                aw_word_idx <= s_axi_awaddr[11:2];
                aw_in_range <= (s_axi_awaddr[11:10] == 2'b00);
                aw_addr     <= s_axi_awaddr;
            end
            // Latch W
            if (w_fire) begin
                w_pending <= 1'b1;
                w_data    <= s_axi_wdata;
                w_strb    <= s_axi_wstrb;
            end
            // Commit write + raise B
            if (do_write) begin
                if (aw_in_range) begin
                    // Byte-lane merge.  WSTRB bit 0 = bits [7:0], etc.
                    // (matches the "little-endian WSTRB over big-endian
                    // data" convention used elsewhere — each byte in
                    // wdata pairs with wstrb[i] = (1<<i).)
                    // MAME's dafb_w masks data to 0xfff before storing.
                    // We mirror that for the base/stride/config slots so
                    // the decode below matches MAME m_base / m_stride /
                    // m_config exactly.  AC842 PCBR is byte-wide.
                    if (write_base_hi) begin
                        r_base_hi <= write_word[11:0];
                        r_base_override_512 <= 1'b0;
                    end
                    else if (write_base_lo) begin
                        r_base_lo <= write_word[3:0];
                        r_base_override_512 <= 1'b0;
                    end
                    else if (write_stride)
                        r_stride_raw <= write_word[29:0];
                    else if (write_config)
                        r_config <= write_word[11:0];
                    else if (write_pcbr) begin
                        r_pcbr     <= write_word[7:0];
                        r_pcbr_set <= 1'b1;
                    end
                    else if (write_irq_w1c || write_swatch_vblank_ack)
                        vblank_pending <= 1'b0;
                    else if (write_swatch_cursor_ack) begin
                        if (swatch_cursor_pending)
                            swatch_cursor_countdown <= swatch_cursor_delay_cycles(
                                shadow_cursor_line);
                        swatch_cursor_pending <= 1'b0;
                    end
                    else if (write_swatch_ctrl) begin
                        // MAME swatch_w case 0x4: clearing the VBL enable bit
                        // also clears int_status bit0 outright.
                        if (!write_word[0])
                            vblank_pending <= 1'b0;
                        if (write_word[2]) begin
                            swatch_cursor_pending <= 1'b0;
                            swatch_cursor_countdown <= swatch_cursor_delay_cycles(
                                shadow_cursor_line);
                        end else begin
                            swatch_cursor_pending <= 1'b0;
                            swatch_cursor_countdown <= 32'd0;
                        end
                    end
                    // ── Monitor sense drive write (MAME dafb.cpp:469-471) ──
                    // The CPU writes +0x1C to drive the connector sense
                    // pins.  m_monitor_id = (data & 0x7) ^ 7 — the low
                    // three bits inverted, since "0=drive, 1=tri-state"
                    // on each pin.  MAME does not capture data&0x40
                    // anywhere; the read-side convolution is gated on
                    // (monitor_sense & 0x40), a host-supplied input.
                    if (aw_in_range && (aw_idx == REG_IRQ_ENABLE) && w_strb[0])
                        r_monitor_id <= write_word[2:0] ^ 3'b111;
                    // ── DP8531 PLL register file (MAME dafb.cpp:882-910) ──
                    // The +0x300-+0x3FF window holds 16 4-bit PLL
                    // register nibbles.  MAME stores on byte-3 writes
                    // (offset & 3 == 3); under our big-endian wstrb
                    // mapping this corresponds to wstrb[0] valid for a
                    // 32-bit word write at +0x300+n*0x10.  The CLUT
                    // decode above reads the same low nibble for
                    // backwards-compat with the 16-entry colour map.
                    if (write_is_pll) begin
                        dp8531_regs[write_pll_idx] <= write_word[3:0];
                        // VCO recompute fires when reg 15 is written
                        // (MAME dafb.cpp:892 "if ((offset>>4) == 15)").
                        // See pll_recompute below for the equation.
                        if (write_pll_idx == 4'hF) begin : pll_recompute
                            // R = regs[6]<<8 | regs[5]<<4 | regs[4]
                            //   (MAME dafb.cpp:894).  12-bit field.
                            reg [11:0] r;
                            // P = 1 << regs[9]   (MAME dafb.cpp:895).
                            reg [3:0]  p_shift;
                            reg [15:0] p;
                            // n_modulus = regs[3..0] in nibble order
                            //   (MAME dafb.cpp:897).
                            reg [15:0] n_modulus;
                            // a = (n_modulus & 0x1f) ^ 0x1f
                            //   (MAME dafb.cpp:898).
                            // b = (n_modulus & 0xffe0) >> 5
                            //   (MAME dafb.cpp:899).
                            reg [10:0] a_pre;
                            reg [10:0] b_pre;
                            reg [10:0] a;
                            reg [10:0] b;
                            // N = 32*(B-A) + 31*(1+A)
                            //   (MAME dafb.cpp:905).
                            reg [31:0] n;
                            // VCO = (PLL_REF / R) * N
                            //   (MAME dafb.cpp:906).
                            reg [31:0] vco;
                            r         = {dp8531_regs[6], dp8531_regs[5], dp8531_regs[4]};
                            p_shift   = dp8531_regs[9];
                            p         = 16'd1 << p_shift;
                            n_modulus = {dp8531_regs[3], dp8531_regs[2],
                                         dp8531_regs[1], dp8531_regs[0]};
                            a_pre     = {6'd0, n_modulus[4:0]} ^ 11'h01F;
                            b_pre     = n_modulus[15:5];
                            // a = min(a_pre, b_pre); b = max(b_pre, 2)
                            //   (MAME dafb.cpp:901-902).
                            a = (a_pre < b_pre) ? a_pre : b_pre;
                            b = (b_pre > 11'd2) ? b_pre : 11'd2;
                            n = (32 * ({21'd0, b} - {21'd0, a}))
                              + (31 * (32'd1 + {21'd0, a}));
                            // Avoid divide-by-zero on R == 0 / P == 0:
                            // MAME would still divide and produce a NaN
                            // double; we hold the previous pixel-clock.
                            if ((r != 12'd0) && (p != 16'd0)) begin
                                vco = (PLL_REF_HZ / {20'd0, r}) * n;
                                r_pixel_clock <= vco >> p_shift;
                            end
                        end
                    end
                    // ── AC842 RAMDAC tri-byte protocol (MAME dafb.cpp:748-789) ──
                    // +0x200 W: latch palette index, reset sub-byte counter.
                    // +0x210 W: write current sub-byte (R/G/B) to selected
                    //           entry; on idx==2->3 wrap, reset idx and
                    //           advance m_pal_address (8-bit, naturally wraps).
                    if (write_ramdac_addr) begin
                        ramdac_pal_address <= w_data[7:0];
                        ramdac_pal_idx     <= 2'd0;
                    end
                    if (write_ramdac_data) begin
                        if (mono_monitor) begin
                            // MAME dafb.cpp:760-767.  The two B&W displays
                            // (monitor codes 1 = 15" Portrait, 3 = 21"
                            // Two-Page) carry intensity on the BLUE channel
                            // only: the R and G sub-byte writes are dropped
                            // on the floor, and the blue byte is replicated
                            // across all three components.  Dropping (rather
                            // than storing) the R/G bytes is what makes the
                            // per-component READBACK at +0x210 match MAME,
                            // whose ramdac_r returns palette pen components
                            // that those writes never touched.
                            if (ramdac_pal_idx == 2'd2) begin
                                ramdac_clut_r[ramdac_pal_address] <= w_data[7:0];
                                ramdac_clut_g[ramdac_pal_address] <= w_data[7:0];
                                ramdac_clut_b[ramdac_pal_address] <= w_data[7:0];
                            end
                        end else begin
                            case (ramdac_pal_idx)
                                2'd0: ramdac_clut_r[ramdac_pal_address] <= w_data[7:0];
                                2'd1: ramdac_clut_g[ramdac_pal_address] <= w_data[7:0];
                                2'd2: ramdac_clut_b[ramdac_pal_address] <= w_data[7:0];
                                default: ; // 2'd3 unreachable (we wrap before reaching it)
                            endcase
                        end
                        if (ramdac_pal_idx == 2'd2) begin
                            ramdac_pal_idx     <= 2'd0;
                            ramdac_pal_address <= ramdac_pal_address + 8'd1;
                            // Blue sub-component just landed -- the RGB
                            // triple for ramdac_pal_address is now complete
                            // (R/G were latched by the two earlier writes of
                            // this same tri-byte sequence).  Pulse the
                            // scanout CLUT write export.  No CDC: consumer
                            // (linebuf_scanout.v) samples this on its own
                            // write-clock port of the dual-clock BRAM.
                            //
                            // Deliberate divergence from MAME (brief-
                            // sanctioned, see the T9 task brief's "RGB
                            // assembled when the third component of an
                            // entry lands" wording): MAME's dafb_w calls
                            // set_pen_{red,green,blue}_level() per
                            // sub-byte write (dafb.cpp:763-765), so a
                            // freshly-written R sub-component is visible
                            // to m_palette->pens() immediately, ahead of
                            // the G/B writes that complete the triple --
                            // harmless there because MAME redraws the
                            // whole frame from scratch once per frame
                            // (screen_update), not continuously.  Our
                            // scanout reads the CLUT BRAM live, pixel by
                            // pixel, while a frame is in flight, so
                            // exposing a partial triple (new R paired
                            // with a stale G/B from whatever entry used
                            // to occupy this slot) would flash a
                            // momentarily-wrong colour instead of the
                            // authentic "one entry updates mid-frame"
                            // sparkle the brief calls for.  Deferring the
                            // scanout-visible write to triple completion
                            // avoids that torn-entry artifact while still
                            // updating within the same frame (no frame-
                            // atomic buffering).  Per-component READBACK
                            // at +0x210 (ramdac_data_byte() below) still
                            // matches MAME exactly -- only the scanout-
                            // side visibility timing differs.
                            clut_we    <= 1'b1;
                            clut_waddr <= ramdac_pal_address;
                            // On a B&W monitor the completed entry is the
                            // blue byte replicated (MAME dafb.cpp:762-766),
                            // NOT the stale R/G still sitting in the arrays.
                            clut_wdata <= mono_monitor
                                        ? {w_data[7:0], w_data[7:0], w_data[7:0]}
                                        : {ramdac_clut_r[ramdac_pal_address],
                                           ramdac_clut_g[ramdac_pal_address],
                                           w_data[7:0]};
                        end else begin
                            ramdac_pal_idx <= ramdac_pal_idx + 2'd1;
                        end
                    end
                    // ── Swatch programmable timing registers (MAME dafb.cpp:683-706) ──
                    // 12-bit-wide hardware regs ("data &= 0xfff;" applied at the
                    // top of swatch_w on dafb.cpp:625, before the case dispatch).
                    if (write_swatch_timing) begin
                        case (aw_idx)
                            REG_SWATCH_HSERR : r_hserr  <= write_word[11:0];
                            REG_SWATCH_HLFLN : r_hlfln  <= write_word[11:0];
                            REG_SWATCH_HEQ   : r_heq    <= write_word[11:0];
                            REG_SWATCH_HSP   : r_hsp    <= write_word[11:0];
                            REG_SWATCH_HBWAY : r_hbway  <= write_word[11:0];
                            REG_SWATCH_HBRST : r_hbrst  <= write_word[11:0];
                            REG_SWATCH_HBP   : r_hbp    <= write_word[11:0];
                            REG_SWATCH_HAL   : r_hal    <= write_word[11:0];
                            REG_SWATCH_HFP   : r_hfp    <= write_word[11:0];
                            REG_SWATCH_HPIX  : r_hpix   <= write_word[11:0];
                            REG_SWATCH_VHLINE: r_vhline <= write_word[11:0];
                            REG_SWATCH_VSYNC : r_vsync  <= write_word[11:0];
                            REG_SWATCH_VBPEQ : r_vbpeq  <= write_word[11:0];
                            REG_SWATCH_VBP   : r_vbp    <= write_word[11:0];
                            REG_SWATCH_VAL   : r_val    <= write_word[11:0];
                            REG_SWATCH_VFP   : r_vfp    <= write_word[11:0];
                            REG_SWATCH_VFPEQ : r_vfpeq  <= write_word[11:0];
                            default          : ;
                        endcase
                    end
`ifdef VERILATOR
                    if (aw_idx == REG_FIRST_HIT) begin
                        $display("[video.dafb] first-write addr=0x%08x data=0x%08x merged=0x%08x strb=0x%x semantic=rom_first_dafb_write",
                                 aw_addr, w_data, write_word, w_strb);
                    end
`endif
                end
                aw_pending   <= 1'b0;
                w_pending    <= 1'b0;
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00;  // OKAY
            end
            if (s_axi_bvalid && s_axi_bready)
                s_axi_bvalid <= 1'b0;

            // ── AC842 RAMDAC read side-effects (MAME dafb.cpp:710-738) ──
            // Co-located in the write-side block so ramdac_pal_{idx,address}
            // have a single driver (Vivado DRC MDRV-1 safety).  Semantics
            // mirror the original read-block update verbatim — both blocks
            // are clocked by the same posedge clk and sample the same
            // pre-cycle state, so moving the update is a no-op for timing.
            if (ar_pending && !s_axi_rvalid && ar_in_range) begin
                if (ar_word_idx[7:0] == REG_RAMDAC_ADDR) begin
                    ramdac_pal_idx <= 2'd0;
                end else if (ar_word_idx[7:0] == REG_RAMDAC_DATA) begin
                    if (ramdac_pal_idx == 2'd2) begin
                        ramdac_pal_idx     <= 2'd0;
                        ramdac_pal_address <= ramdac_pal_address + 8'd1;
                    end else begin
                        ramdac_pal_idx <= ramdac_pal_idx + 2'd1;
                    end
                end
            end
        end
    end

    // ── AR channel ──────────────────────────────────────────────────
    reg        ar_pending;
    reg [9:0]  ar_word_idx;
    reg        ar_in_range;

    wire ar_fire = s_axi_arvalid && !ar_pending && !s_axi_rvalid;
    assign s_axi_arready = !ar_pending && !s_axi_rvalid;

    // Read-data mux: named-override for the handful of ROM-polled
    // status/RAMDAC-byte offsets, else last-written from shadow storage.
    // The write side still persists every byte lane in `regs`, so the
    // special reads stay deterministic without turning the window dead.
    function [31:0] swatch_read_reg;
        input [7:0] idx;
        begin
            case (idx)
                // +0x108 IRQ/VBL status (MAME dafb.cpp:589 case 0x8)
                {REG_SWATCH_BASE[7:4], 4'h2}:
                    swatch_read_reg = {29'd0,
                                       swatch_cursor_pending,
                                       1'b0,
                                       vblank_pending && shadow_swatch_ctrl[0]};
                // +0x120 driver-stash (MAME dafb.cpp:608 case 0x20: m_swatch_test)
                {REG_SWATCH_BASE[7:4], 4'h8}:
                    swatch_read_reg = {20'd0, shadow_read[11:0]};
                // +0x124..0x148 horizontal params (MAME dafb.cpp:611-613)
                REG_SWATCH_HSERR : swatch_read_reg = {20'd0, r_hserr};
                REG_SWATCH_HLFLN : swatch_read_reg = {20'd0, r_hlfln};
                REG_SWATCH_HEQ   : swatch_read_reg = {20'd0, r_heq};
                REG_SWATCH_HSP   : swatch_read_reg = {20'd0, r_hsp};
                REG_SWATCH_HBWAY : swatch_read_reg = {20'd0, r_hbway};
                REG_SWATCH_HBRST : swatch_read_reg = {20'd0, r_hbrst};
                REG_SWATCH_HBP   : swatch_read_reg = {20'd0, r_hbp};
                REG_SWATCH_HAL   : swatch_read_reg = {20'd0, r_hal};
                REG_SWATCH_HFP   : swatch_read_reg = {20'd0, r_hfp};
                REG_SWATCH_HPIX  : swatch_read_reg = {20'd0, r_hpix};
                // +0x14C..0x164 vertical params (MAME dafb.cpp:615-616)
                REG_SWATCH_VHLINE: swatch_read_reg = {20'd0, r_vhline};
                REG_SWATCH_VSYNC : swatch_read_reg = {20'd0, r_vsync};
                REG_SWATCH_VBPEQ : swatch_read_reg = {20'd0, r_vbpeq};
                REG_SWATCH_VBP   : swatch_read_reg = {20'd0, r_vbp};
                REG_SWATCH_VAL   : swatch_read_reg = {20'd0, r_val};
                REG_SWATCH_VFP   : swatch_read_reg = {20'd0, r_vfp};
                REG_SWATCH_VFPEQ : swatch_read_reg = {20'd0, r_vfpeq};
                default:
                    swatch_read_reg = 32'd0;
            endcase
        end
    endfunction

    // Helper: extract one R/G/B sub-byte of CLUT[ramdac_pal_address] for
    // the current ramdac_pal_idx, matching MAME dafb.cpp:721-738.
    function [7:0] ramdac_data_byte;
        input [1:0] idx;
        input [7:0] entry;
        begin
            case (idx)
                2'd0: ramdac_data_byte = ramdac_clut_r[entry];
                2'd1: ramdac_data_byte = ramdac_clut_g[entry];
                2'd2: ramdac_data_byte = ramdac_clut_b[entry];
                default: ramdac_data_byte = 8'd0;
            endcase
        end
    endfunction

    // TurboSCSI bus 1 ctrl word (low 9 bits live at +0x24).  Exposed
    // to scsi.v so the C96 path can implement DRQ-check on its DMA
    // window per dafb.cpp:1001-1011 + 1040-1047.
    assign scsi0_ctrl_out = shadow_scsi_ctrl;

    function [31:0] read_reg;
        input [9:0] idx;
        input       in_range;
        begin
            if (!in_range) begin
                read_reg = 32'd0;
            end else begin
                case (idx[7:0])
                    // +0x1C read = inverse of monitor sense bits (MAME
                    // dafb.cpp:387-415).  Live response computed by
                    // sense_response() above; MAME returns res^7.
                    REG_MON_SENSE  : read_reg = {29'd0, mon_sense_inv};
                    REG_IRQ_STATUS : read_reg = {30'd0, irq_observable, vblank_pending};
                    // +0x24 SCSI bus 1 status (dafb.cpp:418-419):
                    //   m_scsi_ctrl[0] | (m_drq[0] << 9)
                    REG_FIRST_HIT  : read_reg = (shadow_read & 32'h0000_01ff)
                                              | (scsi0_drq_in ? 32'h0000_0200 : 32'h0);
                    REG_DAFB_TEST  : read_reg = (shadow_read & 32'h0000_01ff)
                                                  | DAFB_VERSION_BITS;
                    REG_RAMDAC_ADDR   : read_reg = {24'd0, ramdac_pal_address};
                    REG_RAMDAC_DATA   : read_reg = {24'd0, ramdac_data_byte(ramdac_pal_idx, ramdac_pal_address)};
                    REG_RAMDAC_PBCTRL : read_reg = {24'd0, shadow_read[7:0]};
                    default        : begin
                        if ((idx[7:0] >= REG_SWATCH_BASE) && (idx[7:0] <= REG_SWATCH_END))
                            read_reg = swatch_read_reg(idx[7:0]);
                        else
                            read_reg = shadow_read;
                    end
                endcase
            end
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            ar_pending   <= 1'b0;
            ar_word_idx  <= 10'd0;
            ar_in_range  <= 1'b0;
            s_axi_rvalid <= 1'b0;
            s_axi_rdata  <= 32'd0;
            s_axi_rresp  <= 2'b00;
        end else begin
            if (ar_fire) begin
                ar_pending <= 1'b1;
                ar_word_idx <= s_axi_araddr[11:2];
                ar_in_range <= (s_axi_araddr[11:10] == 2'b00);
            end
            if (ar_pending && !s_axi_rvalid) begin
                s_axi_rdata  <= read_reg(ar_word_idx, ar_in_range);
                s_axi_rresp  <= 2'b00;
                s_axi_rvalid <= 1'b1;
                ar_pending   <= 1'b0;
                // (RAMDAC read side-effects on ramdac_pal_{idx,address}
                //  are owned by the write-side always block at line ~539;
                //  see "AC842 RAMDAC read side-effects" co-located there.
                //  Single-driver-required for Vivado DRC MDRV-1.)
            end
            if (s_axi_rvalid && s_axi_rready)
                s_axi_rvalid <= 1'b0;
        end
    end

endmodule

`default_nettype wire
