// glue.v — Quadra 700 address decoder + ROM-overlay alias
//
// Role in the SoC
// ═══════════════
// The CPU's AXI master emits a 32-bit physical address.  `glue` is the
// top-level address decoder that classifies it into one of
//
//   RAM   : DDR4 backing store, read/write (cacheable)
//   ROM   : ROM image (read-only, cacheable)
//   IO    : 16 MB I/O window at 0x5000_0000 (VIA1/VIA2/SCC/SCSI/ASC/SWIM)
//   VIDEO : DAFB framebuffer at 0xF900_0000
//   UNMAP : anything else — asserts `fault` so the LSU can raise a bus
//           error.
//
// Additionally, `glue` implements the classic Mac **ROM overlay**:
// while VIA1 ORB[3] reads as 1 (the reset state) any access to the
// low 16 MB (phys 0x0000_0000..0x00FF_FFFF) is transparently aliased
// to the ROM image.  Once the ROM clears the overlay (writing 0 into
// VIA1 ORB[3] with DDRB[3]=1), the low window decodes to RAM again
// and ROM is only accessible through its normal 0x4000_0000 mirror.
//
// Overlay semantics (Q700 + earlier pre-MEMCjr Macs):
//   overlay = 1 (reset)         low 16 MB → ROM (phys 0x0000_0000..0x00FF_FFFF)
//   overlay = 0 (after init)    low window → RAM, ROM only at 0x4000_0000
//
// Address map (as instantiated in Q700 era)
// ═════════════════════════════════════════════════════════════════════
//   0x0000_0000..0x0FFF_FFFF  RAM window (overlay=0) / ROM alias (overlay=1)
//   0x4000_0000..0x4FFF_FFFF  ROM image (always)
//   0x5000_0000..0x5000_1FFF  VIA1     (overlay flop + ADB + RTC + timers)
//   0x5000_2000..0x5000_3FFF  VIA2     (NuBus slot IRQs)
//   0x5000_4000..0x5000_7FFF  empty    (MAME leaves 0x50F04000 unmapped)
//   0x5000_8000..0x5000_8007  Ethernet ID/config
//   0x5000_A000..0x5000_B0FF  SONIC registers
//   0x5000_C000..0x5000_DFFF  SCC      (Z85C30 serial)
//   0x5000_E000..0x5000_E0FF  Orwell controls (probe-safe reset stub)
//   0x5000_F000..0x5000_F0FF  DAFB TurboSCSI/NCR53C96 registers
//   0x5000_F100..0x5000_F101  DAFB TurboSCSI DMA handshake
//   0x5001_4000..0x5001_5FFF  ASC/EASC (SONORA)
//   0x5001_E000..0x5001_FFFF  SWIM/IWM (floppy — stub, cs asserted only)
//   0x50xx_xxxx               mirrors of the canonical map above with
//                             addr[23:18] cleared (`0x00FC_0000` mask)
//   0x5100_0000..             outside the Q700 I/O mirror — unmapped
//   0x5800_0000..0x5BFF_FFFF  Q700 RAM probe alias (maps to low RAM)
//   0xF900_0000..0xF91F_FFFF  VRAM pixel aperture (2 MB, routed to xbar S3
//                                     in fpga_top, task #147 — not a
//                                     glue concern but still CLASS_VIDEO)
//   0xF980_0000..0xF980_03FF  DAFB register window (1 KB)  — consumed by
//                                     the DAFB shim (task #143).  This
//                                     is the ONLY window where cs_video
//                                     fires now; the 2 MB pixel aperture
//                                     at 0xF900_0000 is owned by the
//                                     xbar's S3 (vram) port.  The rest
//                                     of 0xF9xx_xxxx remains CLASS_VIDEO
//                                     so the existing observational
//                                     decode stays consistent.
//   (all other)               unmapped — `fault` asserted
//
// This decoder matches the I/O-slot layout of
// `rtl/sys/peripheral_bus.v` so the same chip-select hierarchy applies
// at both levels — `glue` tells the fabric "is this an I/O access?" /
// "which peripheral is targeted?"; `peripheral_bus` then generates the
// per-peripheral pb_* handshake for transactions that reach the I/O
// window.
//
// Interface
// ═════════
// Pure combinational — `req_addr` arrives from the LSU/I-cache side,
// `out_addr` is the aliased physical address (with overlay applied)
// that the memory fabric should actually use.  Chip-select lines are
// one-hot per peripheral so the TB (and downstream arbitration) can
// assert on decode correctness without plumbing the full AXI fan-out.
// The `class` output is a small enum flagging whether this request
// should be routed to RAM, ROM, I/O, VIDEO, or UNMAP.
//
// The `fc` input is the 68040 function code (from CPU supervisor
// state; 3'b111 == CPU space).  `priv_violate` is asserted if an
// **I/O** access is made from user-mode (FC[2]=0).  Real hardware
// leaves this to the MMU; the glue publishes it for unit-tb
// convenience and for the `exception.v` bus-error path to OR in.
//
// Lint: pure combinational, no sequential state.  Reset is unused
// because glue.v has no flops of its own — the overlay flop lives
// inside `via1.v`.  `clk` / `rst` inputs kept for port-shape
// stability with peripheral-module convention.

`default_nettype none

module glue #(
    parameter ROM_BASE      = 32'h4000_0000,
    parameter ROM_SIZE_LOG2 = 8'd24,         // 16 MB ROM window
    parameter RAM_SIZE_LOG2 = 8'd28,         // 256 MB RAM window at phys 0
    parameter RAM_ALIAS_BASE = 32'h5800_0000,
    parameter RAM_ALIAS_SIZE = 32'h0400_0000 // 64 MB Q700 RAM probe alias
) (
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire        clk,
    input  wire        rst,
    /* verilator lint_on UNUSEDSIGNAL */

    // ── Decode request ─────────────────────────────────────────────
    input  wire [31:0] req_addr,    // CPU-visible physical address
    input  wire        req_valid,   // qualifies the decode outputs
    input  wire        req_rw,      // 1 = write, 0 = read
    input  wire [1:0]  req_size,    // 00=byte, 01=word, 10=long (observed)
    input  wire [2:0]  req_fc,      // 68040 function code
    input  wire        overlay_in,  // VIA1 ORB[3] live value

    // ── Classified outputs ─────────────────────────────────────────
    output wire [31:0] out_addr,    // post-overlay physical address
    output wire [2:0]  out_class,   // CLASS_* enum (see below)

    // ── Per-slave chip-selects (one-hot inside the I/O window) ──────
    output wire        cs_ram,
    output wire        cs_rom,
    output wire        cs_via1,
    output wire        cs_via2,
    output wire        cs_scc,
    output wire        cs_scsi,
    output wire        cs_asc,
    output wire        cs_iwm,
    output wire        cs_enet,
    output wire        cs_sonic,
    output wire        cs_orwell,
    output wire        cs_video,

    // ── Fault flags ────────────────────────────────────────────────
    output wire        fault,        // unmapped access (bus error)
    output wire        priv_violate  // I/O from user mode
);

    // ── Class enum ─────────────────────────────────────────────────
    localparam [2:0] CLASS_UNMAP = 3'd0;
    localparam [2:0] CLASS_RAM   = 3'd1;
    localparam [2:0] CLASS_ROM   = 3'd2;
    localparam [2:0] CLASS_IO    = 3'd3;
    localparam [2:0] CLASS_VIDEO = 3'd4;

    /* verilator lint_off UNUSEDSIGNAL */
    wire [1:0]  _unused_sz = req_size;
    wire        _unused_rw = req_rw;
    /* verilator lint_on UNUSEDSIGNAL */

    // ── Top-quarter classification ─────────────────────────────────
    // bit field: addr[31:28]  -> broad class
    //   0x0..0x0      RAM (or ROM alias if overlay=1)
    //   0x4           ROM
    //   0x50          I/O (16 MB Q700 mirror)
    //   0xF (specifically 0xF900_0000..0xF9FF_FFFF) → VIDEO/DAFB
    wire [3:0] a_hi = req_addr[31:28];

    // RAM reach: 256 MB window at phys 0 (RAM_SIZE_LOG2 = 28 default)
    /* verilator lint_off CMPCONST */
    wire in_low_window = (req_addr < (32'd1 << RAM_SIZE_LOG2));
    /* verilator lint_on CMPCONST */

    wire in_rom_window = (a_hi == 4'h4) &&
                         (req_addr[27:ROM_SIZE_LOG2] == {(28-ROM_SIZE_LOG2){1'b0}});
    wire in_io_window  = (req_addr[31:24] == 8'h50);
    wire in_ram_alias_window =
        ((req_addr & ~(RAM_ALIAS_SIZE-1)) == RAM_ALIAS_BASE);
    wire in_vid_window = (req_addr[31:24] == 8'hF9);

    // Sub-split the 0xF9 window: the 2 MB pixel aperture at 0xF900_0000
    // is the VRAM port (xbar S3) — matches Q700 silicon (2 MB VRAM,
    // AXI_VRAM_SIZE in axi_defs.vh); the 4 KB DAFB register window at
    // 0xF980_0000 is the register shim's territory (task #143); the
    // rest of 0xF9xxxxxx stays observational CLASS_VIDEO but faults so
    // probes cannot be silently dropped with no owning sink.
    localparam [31:0] VRAM_APERTURE_BASE = 32'hF900_0000;
    localparam [31:0] VRAM_APERTURE_END  = 32'hF920_0000;
    localparam [31:0] DAFB_REG_BASE      = 32'hF980_0000;
    localparam [31:0] DAFB_REG_END       = 32'hF980_0400;

    wire in_vram_pixel_win = (req_addr >= VRAM_APERTURE_BASE) &&
                             (req_addr <  VRAM_APERTURE_END);
    wire in_dafb_reg_win   = (req_addr >= DAFB_REG_BASE) &&
                             (req_addr <  DAFB_REG_END);

    // ── Overlay alias ──────────────────────────────────────────────
    // When overlay=1 AND the access is in the low RAM window, the
    // low 24 bits fall through to the ROM image.  Masked to the ROM
    // size so any wrap folds back (Q700 hardware also wraps — a 1 MB
    // ROM mirrored 16× over 16 MB).  We don't know the actual ROM
    // size at elab time; mask to ROM_SIZE_LOG2 bits.
    wire overlay_active = overlay_in && in_low_window;
    wire [31:0] alias_addr =
        ROM_BASE |
        { {(32-ROM_SIZE_LOG2){1'b0}}, req_addr[ROM_SIZE_LOG2-1:0] };

    wire [31:0] ram_alias_addr = req_addr - RAM_ALIAS_BASE;

    // out_addr: pass through unchanged unless overlay or the Q700 RAM
    // probe alias remaps us.
    assign out_addr = overlay_active ? alias_addr :
                      in_ram_alias_window ? ram_alias_addr :
                      req_addr;

    // ── I/O sub-region decode ──────────────────────────────────────
    // MAME's Q700 map mirrors the 0x5000_0000 device block through the
    // 16 MB 0x5000_0000..0x50FF_FFFF I/O window by ignoring addr[23:18].
    // The ROM harness documents this as the 0x00FC_0000 mirror mask:
    // 0x50F0_C000 and 0x5000_C000 are the same SCC registers, while
    // 0x5100_xxxx is outside the mirror and must not decode here.
    //
    // Only live RTL sinks are selected below.  Ethernet ID/SONIC has a
    // synthesized reset/config register block; Orwell controls and SWIM/IWM
    // still have probe-safe stubs.  Do not alias any remaining gaps to a
    // nearby device just to satisfy probes.
    localparam [23:0] Q700_IO_MIRROR_MASK = 24'hFC0000;
    wire [23:0] io_off = req_addr[23:0] & ~Q700_IO_MIRROR_MASK;

    wire io_via1_hit = in_io_window && (io_off < 24'h002000);
    wire io_via2_hit = in_io_window && (io_off >= 24'h002000) && (io_off < 24'h004000);
    wire io_enet_hit = in_io_window && (io_off >= 24'h008000) && (io_off < 24'h008008);
    wire io_sonic_hit = in_io_window && (io_off >= 24'h00A000) && (io_off < 24'h00B100);
    wire io_scc_hit  = in_io_window && (io_off >= 24'h00C000) && (io_off < 24'h00E000);
    wire io_orwell_hit = in_io_window && (io_off >= 24'h00E000) && (io_off < 24'h00E100);
    // MAME macqd700 maps DAFB TurboSCSI as a 0x100-byte NCR53C96 register
    // window plus a two-byte DMA handshake at +0x100/+0x101, all mirrored
    // through the Q700 addr[23:18] mask.  Keep the rest of the 4 KB page
    // unmapped so accidental 0x50f0f4xx/0x50f0ffxx probes are visible.
    wire io_scsi_regs = (io_off >= 24'h00F000) && (io_off < 24'h00F100);
    wire io_scsi_dma  = (io_off >= 24'h00F100) && (io_off < 24'h00F102);
    wire io_scsi_hit  = in_io_window && (io_scsi_regs || io_scsi_dma);
    wire io_asc_hit  = in_io_window && (io_off >= 24'h014000) && (io_off < 24'h016000);
    wire io_iwm      = in_io_window && (io_off >= 24'h01E000) && (io_off < 24'h020000);

    // ── Primary class select ───────────────────────────────────────
    // Priority: overlay always wins in the low window; ROM image
    // wins on 0x4xxx; I/O wins on 0x50xx; video wins on 0xF9; else
    // RAM (if in the RAM window) or UNMAP.
    reg [2:0] class_r;
    always @(*) begin
        if (!req_valid)              class_r = CLASS_UNMAP;
        else if (overlay_active)     class_r = CLASS_ROM;
        else if (in_rom_window)      class_r = CLASS_ROM;
        else if (in_io_window)       class_r = CLASS_IO;
        else if (in_ram_alias_window) class_r = CLASS_RAM;
        else if (in_vid_window)      class_r = CLASS_VIDEO;
        else if (in_low_window)      class_r = CLASS_RAM;
        else                         class_r = CLASS_UNMAP;
    end
    assign out_class = class_r;

    // ── Chip-select outputs ────────────────────────────────────────
    // Qualified by req_valid so quiescent outputs are 0 (prevents
    // spurious peripheral pulses in the TB).
    assign cs_ram   = req_valid && (class_r == CLASS_RAM);
    assign cs_rom   = req_valid && (class_r == CLASS_ROM);
    assign cs_via1  = req_valid && io_via1_hit;
    assign cs_via2  = req_valid && io_via2_hit;
    assign cs_enet  = req_valid && io_enet_hit;
    assign cs_sonic = req_valid && io_sonic_hit;
    assign cs_scc   = req_valid && io_scc_hit;
    assign cs_orwell = req_valid && io_orwell_hit;
    assign cs_scsi  = req_valid && io_scsi_hit;
    assign cs_asc   = req_valid && io_asc_hit;
    assign cs_iwm   = req_valid && io_iwm;
    // cs_video is scoped to the DAFB register window only — task #147
    // split off the 0xF900_0000..0xF91F_FFFF (2 MB) pixel aperture to the
    // xbar's VRAM slave (S3) so a write there does NOT need a downstream
    // CLASS_VIDEO handler.  The DAFB register shim (task #143) keys off
    // `cs_video` and thus sees ONLY its 1 KB 0xF980_0000..0xF980_03FF
    // window, which matches the real Q700 DAFB pinout.  The observational
    // CLASS_VIDEO class still includes the whole 0xF9xx window so
    // downstream tooling / debug decoders are unchanged.
    assign cs_video = req_valid && in_dafb_reg_win;

    // ── Fault detection ───────────────────────────────────────────
    // A request is UNMAP if (1) it's in the I/O window but no chip-
    // select resolved it, (2) it lands in an unowned 0xF9 video gap,
    // or (3) it falls outside all four top-level windows.  We don't
    // raise fault on plain top-level UNMAPs when req_valid is low.
    wire io_hit_any = io_scc_hit | io_scsi_hit | io_asc_hit
                    | io_iwm
                    | io_via1_hit | io_via2_hit
                    | io_enet_hit | io_sonic_hit
                    | io_orwell_hit;
    wire io_unmapped = in_io_window && !io_hit_any;
    wire video_hit_any = in_vram_pixel_win | in_dafb_reg_win;
    wire video_unmapped = in_vid_window && !video_hit_any;
    assign fault = req_valid && (
                      io_unmapped ||
                      video_unmapped ||
                      (class_r == CLASS_UNMAP)
                  );

    // ── Privilege check ───────────────────────────────────────────
    // 68040 FC[2] is set for supervisor accesses.  I/O space is only
    // reachable from supervisor mode on a real Mac (the ROM / OS own
    // it); a user-mode hit here raises a bus-fault-style violation so
    // the CPU's exception.v path can emit vector 8 / 9 as appropriate.
    // This is ADVISORY — the MMU's ITT/DTT+TT walk is still the
    // authoritative protection check; glue_fault/priv_violate are
    // handy TB hooks and a defense-in-depth signal.
    assign priv_violate = req_valid && in_io_window && !req_fc[2];

endmodule

`default_nettype wire
