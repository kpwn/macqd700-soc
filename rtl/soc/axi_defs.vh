// axi_defs.vh — system address map and AXI-xbar parameters
//
// Single authoritative place for the AXI system-bus address map used by
// `axi_xbar.v` and any future integrator.  The xbar is parameterised; the
// constants below match the map documented in docs/peripheral_arch.md:
//
//   0x0000_0000 .. 0x0xxx_xxxx  RAM        (DDR4, sized by `dbg_ram_window_lg2`,
//                                           default 4 MiB Q700-silicon-spec)
//   0x0xxx_xxxx .. 0x3FFF_FFFF  RAM-OOR    (open bus — OKAY + 0x00000000;
//                                           MAME-canonical Q700 unmap default)
//   0x4000_0000 .. 0x4FFF_FFFF  ROM mirror (DDR4-resident, R/O from CPU)
//   0x5000_0000 .. 0x50FF_FFFF  I/O        (peripheral bus via GLUE, 16 MB)
//   0x5010_0000 .. 0x501F_FFFF  DMA cfg    (DMA controller AXI-Lite, 1 MB)
//   0x50A0_0000 .. 0x50A0_FFFF  SD JTAG writer (AXI-Lite, 64 KB)
//   0x5800_0000 .. 0x5BFF_FFFF  RAM alias  (Q700 ROM probe mirror)
//   0x6000_0000 .. 0x607F_FFFF  VIDEO FB   (DDR4-alias, 8 MB default — legacy)
//   0xF900_0000 .. 0xF91F_FFFF  VRAM pixel aperture (URAM-backed, 2 MB silicon)
//   0xF980_0000 .. 0xF980_03FF  DAFB regs   (live video shim via S4)
//
// Unmapped-everywhere policy (MAME-canonical):  every unmapped read
// returns 0x00000000 + AXI OKAY, writes are silently dropped with OKAY.
// No SLVERR / DECERR raised on unmapped addresses.  Implemented in the
// xbar's local-response path (NONE-decoded → OKAY) and mirrored by
// `peripheral_bus.v`'s SLOT_VOID path for I/O gaps.  ROM-mirror writes
// from CPU (M0) get the same OKAY-drop treatment (mapped-but-RO, write
// silently absorbed) — NOT SLVERR — matching MAME-canonical Q700, where
// Mac OS legitimately probes/touches the ROM mirror during boot; SLVERR
// there diverged from MAME and caused a post-IMMU Sad Mac (see
// axi_xbar.v's local-response policy comment, ~line 1577).
//
// The DMA cfg window sits INSIDE the I/O address range; the xbar's
// decoder checks DMA first (narrower match) then IO so 0x5010_0000
// lands on S2 and the rest of 0x5xxx_xxxx lands on S1.
//
// The VRAM pixel aperture at 0xF900_0000 routes to the URAM-backed
// `vram` slave (S3).  The DAFB register window at 0xF980_0000 is not
// part of that pixel aperture; it is decoded onto its own S4 xbar slave
// so CPU and host AXI masters share the same live DAFB register shim
// without routing display MMIO through the slower Mac peripheral island.

`ifndef AXI_DEFS_VH
`define AXI_DEFS_VH

// ── System address map ────────────────────────────────────────────────────
`define AXI_RAM_BASE      32'h0000_0000
`define AXI_RAM_SIZE      32'h4000_0000   // 1 GiB decode window
`define AXI_RAM_ALIAS_BASE 32'h5800_0000
`define AXI_RAM_ALIAS_SIZE 32'h0400_0000  // 64 MiB Q700 RAM probe alias

`define AXI_ROM_BASE      32'h4000_0000
`define AXI_ROM_SIZE      32'h0040_0000   // 4 MiB backing window (DDR reservation)
// The Q700's ROM is 1 MB and its address decode WRAPS at 1 MB.  MAME:
//   map(0x40000000, 0x400fffff).mirror(0x0ff00000)   macquadra700.cpp:557
// This is the MIRROR PERIOD and is a property of the hardware -- it is NOT the
// same thing as AXI_ROM_SIZE, which is only how much DDR we reserve.  Conflating
// the two made the mirror repeat every 4 MiB instead of 1 MiB, so 3 of every 4
// MiB of the 0x4xxx_xxxx mirror read DDR that NOTHING ever writes (the loader
// only fills 1 MiB, and the zero pass stops at 0x003F_FFFF).  See task #245.
`define AXI_ROM_IMAGE_SIZE 32'h0010_0000  // 1 MiB — Q700 ROM mirror period
`define AXI_ROM_MIRROR_BASE 32'h4000_0000
`define AXI_ROM_MIRROR_SIZE 32'h1000_0000 // Q700 0x4xxx_xxxx ROM mirror

`define AXI_IO_BASE       32'h5000_0000
`define AXI_IO_SIZE       32'h0100_0000   // 16 MB peripheral window

`define AXI_DMA_BASE      32'h5010_0000
`define AXI_DMA_SIZE      32'h0010_0000   // 1 MB DMA config hole inside IO window.

`define AXI_SD_JTAG_BASE  32'h50A0_0000
`define AXI_SD_JTAG_SIZE  32'h0001_0000   // 64 KB SD-card JTAG writer window.

`define AXI_FB_BASE       32'h6000_0000
`define AXI_FB_SIZE       32'h0080_0000   // 8 MB (default; grows for 1080p@32bpp)

// ── RAM-disk aperture: REMOVED 2026-09-10 ─────────────────────────────────
// AXI_RAMDISK_BASE (0x7000_0000) / AXI_RAMDISK_SIZE (256 MB) described the
// DDR-backed second SCSI volume served by rtl/soc/vhdd_ddr.v.  That volume
// was deleted on owner directive ("vhdd ddr should be killed") along with
// its xbar decode/flatten arms and the l2c bypass window that kept the
// flattened range uncached.
//
// 0x7xxx_xxxx is therefore unmapped again and reads back open bus
// (0x00000000 + OKAY, MAME-canonical) exactly as it did before the volume
// existed -- which is what tb_axi_xbar.cpp scenarios 7 and 10 assert.

// Quadra 700 VRAM pixel aperture — routed to URAM-backed `vram` slave (S3).
// **2 MB is the Q700 silicon VRAM size** (matches MAME `map(0xf9000000,
// 0xf91fffff)`).  The size is fixed-2MB regardless of framebuffer
// resolution: future resolution changes (e.g. raise BPP from 8→16,
// 1024×768 → 1152×870) reuse the same 2 MB aperture.  See
// rtl/sys/vram.v `VRAM_BYTES` parameter.  The xbar's S3 port peels off
// `AXI_VRAM_BASE` before forwarding, so the VRAM slave sees a
// zero-based byte offset.  The DAFB *register* window is separate
// and routes through S4 to the live video shim.
`define AXI_VRAM_BASE     32'hF900_0000
// Aperture size: 2 MB (Q700 silicon) on the URAM build, 4 MB under
// VRAM_IN_DDR.  DAFB 24bpp stores 4 B/px (MAME dafb.cpp:340-350), so
// 1024x768 direct colour is 3,145,728 bytes and does NOT fit 2 MB; 4 MB is
// the smallest power of two that holds it, which matters because axi_xbar.v
// decodes this window as `(a & ~(VRAM_SIZE-1)) == VRAM_BASE` and so requires
// a power-of-two, naturally-aligned size.  0xF900_0000 + 4 MB ends at
// 0xF93F_FFFF, well clear of the DAFB register window at 0xF980_0000.
//
// Widening is gated on VRAM_IN_DDR because on the URAM build the array IS
// the capacity limit (2 MB already consumes all 64 KU5P URAM288 blocks),
// and because a 4 MB aperture is visible to the ROM's VRAM-size probe --
// i.e. it makes the machine claim more VRAM than Q700 silicon has.  See the
// COMPATIBILITY NOTE in rtl/soc/fpga_top_video.vh.
`ifdef VRAM_IN_DDR
`define AXI_VRAM_SIZE     32'h0040_0000   // 4 MB pixel aperture (24bpp @ 1024x768)
`else
`define AXI_VRAM_SIZE     32'h0020_0000   // 2 MB pixel aperture (Q700 silicon)
`endif

`define AXI_DAFB_BASE     32'hF980_0000
`define AXI_DAFB_SIZE     32'h0000_0400   // 1 KB live DAFB register shim

// ── VRAM-in-DDR carveout (T14, `VRAM_IN_DDR` build gate) ──────────────────
// Reserves the TOP of the flattened DDR address space (see "DDR
// flattening policy" below) for the 2 MB VRAM pixel aperture once it
// moves off URAM onto the DDR4 carveout.  Only consumed when
// `VRAM_IN_DDR` is defined.  Unconditionally `define`d here (like every
// other window in this file) so the value exists regardless of build
// config -- it is simply unused RTL-wise unless `VRAM_IN_DDR` is also
// defined at the consumer.
//
// T16 ("decode-vram-lane" reshape): consumers changed.  T14 had
// axi_xbar.v's `ddr_flatten()` apply this base directly to VRAM-
// aperture addresses re-routed onto S0, and set it as l2c's bypass-
// window base/mask (rtl/soc/fpga_top_ddr.vh).  T16 reverts both of
// those -- axi_xbar.v no longer references this macro at all (S3 is a
// genuine slave again, unconditionally), and l2c's bypass window is
// back to its disabled default.  The macro is now consumed by
// rtl/soc/fpga_top_ddr.vh's S3-backend address-translate
// (`s3lane_awaddr`/`s3lane_araddr`, `+ AXI_VRAM_DDR_CARVEOUT_BASE` on
// xbar S3's already-zero-based address) and by
// rtl/soc/scanout_ddr_reader.v (unchanged from T14).  Same physical DDR
// window, enforcement of "never reaches l2c" moved from a runtime
// bypass-window check to the address decode itself -- see
// docs/l2c_spec.md's invariants section.
//
// AXI_DDR_FLAT_SIZE is the total flattened-DDR *window* this design's
// address map reaches today, NOT the physical DDR4 chip capacity (the
// AN9134 SODIMM is 4 GB, docs/hardware_feasibility.md) -- it only needs
// to cover the existing RAM(1 GiB decode window)/ROM(4 MiB)/FB(8 MiB)
// span (folded below 0x40C0_0000 by ddr_flatten() below) plus headroom
// for the carveout, with room to spare below l2c's default
// CACHEABLE_BASE/CACHEABLE_SIZE (0x0000_0000 + 0x4100_0000,
// docs/l2c_spec.md S3) so the carveout and the cacheable span stay
// disjoint per l2c's invariant 1 precondition (docs/l2c_spec.md S1) --
// l2c_bypass.v's own `$fatal` assert enforces this at elaboration time
// once NUM_BYPASS_WINDOWS>0/WIN_EN=1 for this window.
// 2026-08-03: widened from 0x4800_0000 to cover the RAM-disk carveout at
// 0x5000_0000.  That carveout was removed on 2026-09-10 with the RAM disk
// itself (see the note below the flattening offsets); the value is LEFT AS
// IS -- it is a ceiling, not a claim, it is still far inside the 2 GiB
// physical DDR4 on the AN9134 board, and shrinking it back buys nothing.
`define AXI_DDR_FLAT_SIZE           32'h6000_0000   // 1.5 GiB flattened window
`define AXI_VRAM_DDR_CARVEOUT_SIZE  32'h0200_0000   // 32 MiB carveout (2 MB VRAM aperture + headroom for future direct-color modes)
`define AXI_VRAM_DDR_CARVEOUT_BASE  32'h4600_0000   // AXI_DDR_FLAT_SIZE - AXI_VRAM_DDR_CARVEOUT_SIZE

// ── DDR flattening policy (implemented in axi_xbar.v) ─────────────────────
// The MIG presents a single contiguous address space.  The xbar maps:
//   RAM   [0x0000_0000 .. 0x3FFF_FFFF]  → DDR [0x0000_0000 .. 0x3FFF_FFFF]
//         default Q700 sizing mode aliases this through the runtime
//         RAM-window mask (64 MiB default), so 0x0400_0000,
//         0x0800_0000, ... mirror low RAM.  axi_xbar can be parameterized
//         back to strict mode, where accesses above the visible window
//         return DECERR locally.
//   ROM   [0x4000_0000 .. 0x4FFF_FFFF]  → DDR [0x4000_0000 .. 0x403F_FFFF],
//         mirrored by AXI_ROM_SIZE.
//   FB    [0x6000_0000 .. 0x607F_FFFF]  → DDR [0x4040_0000 .. 0x40BF_FFFF]
//
// The mapping is a compile-time constant (small LUT), not a shift — real
// hardware DDR is bigger than RAM alone so ROM/FB just live above RAM.
`define AXI_DDR_RAM_OFFSET 32'h0000_0000
`define AXI_DDR_ROM_OFFSET 32'h4000_0000
`define AXI_DDR_FB_OFFSET  32'h4040_0000

// DDR 0x5000_0000 .. 0x5FFF_FFFF was the RAM disk's flattened carve-out
// (AXI_DDR_RAMDISK_OFFSET + AXI_RAMDISK_L2_BYP_BASE/MASK).  Removed with
// the volume on 2026-09-10; nothing flattens there any more, so the range
// is simply unclaimed DDR.  A future raw-DDR client that wants it uncached
// needs BOTH an axi_xbar decode arm and a matching l2c bypass window --
// re-add them together or neither (fpga_top_ddr.vh's l2c instantiation).

// ── AXI response codes ────────────────────────────────────────────────────
`define AXI_RESP_OKAY     2'b00
`define AXI_RESP_EXOKAY   2'b01
`define AXI_RESP_SLVERR   2'b10
`define AXI_RESP_DECERR   2'b11

// ── Slave select encoding (internal to xbar) ──────────────────────────────
// NOTE: widened to 3 bits when VRAM was added as S3 (task #147).
// The NONE sentinel still sits at the top of the encoded space so the
// decoder's default path continues to mean "DECERR — handled locally".
`define XBAR_SLV_DDR      3'd0  // S0
`define XBAR_SLV_IO       3'd1  // S1
`define XBAR_SLV_DMA      3'd2  // S2 — DMA config (AXI-Lite bridged)
`define XBAR_SLV_VRAM     3'd3  // S3 — URAM-backed VRAM pixel aperture
`define XBAR_SLV_DAFB     3'd4  // S4 — DAFB register shim (AXI-Lite bridged)
`define XBAR_SLV_SDJTAG   3'd5  // S5 — SD JTAG writer (AXI-Lite bridged)
`define XBAR_SLV_NONE     3'd7  // DECERR — handled locally, no slave issued
`define XBAR_SLV_W        3    // width of the slave-select field

// ── Master indexing (internal to xbar write/read fan-in tags) ─────────────
// NOTE (2026-07-16 master-count reduction): these tags no longer map 1:1
// onto top-level port numbers "M<n>" — the xbar now exposes only 3 live
// top-level masters (M0 = CPU LSU / boot FSM merge, M1 = host debug,
// M2 = CPU instruction fetch).  XBAR_M_BOOT is now the *time-multiplexed*
// identity `mw_midx[0]` takes while the M0 physical port is presenting
// boot-FSM traffic (see axi_xbar.v's M0/boot merge header note) — it is
// NOT a separate physical master anymore.  XBAR_M_DMA is retained purely
// for readability on the now-permanently-idle write/read fan-in slot 3
// (dma_ctrl's AXI4 master port is stubbed out of the crossbar — see
// axi_xbar.v header — until a real consumer needs it).
`define XBAR_M_CPU        3'd0  // xbar M0 fan-in identity: CPU LSU
`define XBAR_M_XDMA       3'd1  // xbar M1: host debug (XDMA / JTAG-to-AXI)
`define XBAR_M_BOOT       3'd2  // xbar M0 fan-in identity: boot FSM (time-
                                 // multiplexed onto M0 via cpu_held_in_reset)
`define XBAR_M_CPUI       3'd3  // xbar M2: CPU instruction fetch (read-only)
`define XBAR_M_DMA        3'd4  // formerly M4 — DMA controller; stubbed,
                                 // fan-in slot permanently idle

`endif // AXI_DEFS_VH
