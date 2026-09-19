// MAME reference/excerpt/adaptation attribution: Copyright R. Belmont.
// MAME-derived portions retain BSD-3-Clause; original contributions are MIT.
// See THIRD_PARTY_NOTICES.md and LICENSES/MAME-BSD-3-Clause.txt.
// mode_decode.v — DAFB stage 2: raw register snapshot -> canonical mode
// descriptor.  Pure combinational function; no clock, no state.
//
// PURPOSE.  This is THE ONLY PLACE THE DAFB MODE ARITHMETIC LIVES
// (docs/video_path_review.md §4.1, stage 2).  `video.v` keeps the Mac-facing
// register file and the bus side-effects; every derivation of a resolution, a
// depth, a row pitch or a framebuffer base happens here and is CARRIED
// downstream, never recomputed.  Nothing below stage 4 knows what a
// "resolution" is.
//
// INTERFACE.  Inputs are the raw DAFB/Swatch/AC842 register values, named for
// the register they come from.  Outputs are the canonical descriptor
//
//     {src_w_px, src_h_px, bpp_shift, bytes_per_row, fb_base_bytes, lut_depth}
//
// plus the two depth-table derivatives `bytes_per_px` and `depth_supported`
// (same AC842 field, so they belong to the same owner) and the two control
// outputs the caller needs to sample this function MAME's way,
// `recalc_valid` and `base_override_512`.
//
// EVERY OUTPUT NAME CARRIES ITS UNIT (`_px` / `_bytes`).  This is not style.
// The in-tree `FB_MAX_PIXELS` -> `FB_MAX_BYTES` rename happened because a
// pixel-named parameter was compared against byte counts, and the live
// `hres` 2x bug is the same class of error.  A signal whose unit is not in
// its name will eventually be compared against the wrong thing.
//
// LATENCY: zero (combinational).
//
// ── GOLDEN REFERENCE ────────────────────────────────────────────────────
// MAME `dafb_base::recalc_mode()`, src/mame/apple/dafb.cpp:821-868, and
// `dafb_base::ramdac_w` case 0x20, dafb.cpp:748-816.  Reproduced here so the
// arithmetic can be checked line by line:
//
//     m_htotal = m_horizontal_params[HPIX];
//     m_vtotal = m_vertical_params[VFPEQ] >> 1;
//     if ((m_htotal > 0) && (m_vtotal > 0)) {
//         m_hres = m_horizontal_params[HFP] - m_horizontal_params[HAL];
//         m_vres = (m_vertical_params[VFP] >> 1)
//                - (m_vertical_params[VAL] >> 1);   // half-line units
//         if ((m_hres == 512) && (m_dafb_version == 1)) {
//             m_base = 0x1000;  m_vres = 384;       // Quadra 700 fixup
//         }
//         const int clockdiv = 1 << ((m_ac842_pbctrl & 0x60) >> 5);
//         if (BIT(m_config, 3)) {  m_hres /= clockdiv;  m_stride /= clockdiv;
//                                  m_hres -= 23; }
//         else                  {  m_hres *= clockdiv;  m_htotal *= clockdiv; }
//         if (BIT(m_config, 2))    m_vres <<= 1;
//     }
//
// ── WHY `recalc_valid` EXISTS, AND WHY IT IS THE WHOLE POINT ────────────
// `recalc_mode()` is reachable from EXACTLY ONE place in dafb.cpp: the AC842
// PCBR write, `ramdac_w` case 0x20 (dafb.cpp:816).  It is NOT called from
// `swatch_w` (HAL/HFP/VAL/VFP) and NOT from `dafb_w` (base/stride/config).
// Enumerated over the whole file, the call sites are dafb.cpp:816 (base),
// :1164 (q950), :1291 (memc), :1433 (memcjr) — all four inside a `ramdac_w`.
//
// So in MAME `m_hres`/`m_vres` are a SNAPSHOT taken when the driver writes
// the PCBR, not a live function of the Swatch registers.  The clockdiv field
// that scales `hres` lives in the SAME register that selects the depth, and a
// mode set is many separate CPU writes, so a live evaluation forms `hres`
// from whatever mixture of old and new registers happens to be latched.
//
// That mixture is the shipped `hres` bug.  Captured from MAME 0.285 macqd700
// boots, one per monitor-sense code (see tb/tb_mode_decode.cpp for the full
// table):
//     sense 0x6D  832x624 : HAL=0x08b HFP=0x22b -> raw 416, PCBR=0xa0 -> cd 2
//     sense 0x00  1152x870: HAL=0x040 HFP=0x160 -> raw 288, PCBR=0xc0 -> cd 4
// The board reported `hres=1664` for an 832-wide mode.  1664 is 416 << 2 —
// the 832x624 Swatch pair scaled by the 1152x870 clockdiv.  The misread input
// is the clockdiv, read from the wrong mode epoch.
//
// The caller therefore samples this function on a PCBR write and only when
// `recalc_valid` is high, exactly mirroring `recalc_mode()`'s own guard.
// Held that way, no mixed-epoch geometry is representable at all.

`default_nettype none

module mode_decode (
    // ── Stage 1 -> 2: raw DAFB register snapshot ─────────────────────
    input  wire [11:0] reg_hal,           // Swatch +0x140  (MAME HAL)
    input  wire [11:0] reg_hfp,           // Swatch +0x144  (MAME HFP)
    input  wire [11:0] reg_val,           // Swatch +0x15C  (MAME VAL)
    input  wire [11:0] reg_vfp,           // Swatch +0x160  (MAME VFP)
    input  wire [11:0] reg_config,        // DAFB   +0x010  (MAME m_config)
    input  wire [7:0]  reg_pcbr,          // AC842  +0x220  (m_ac842_pbctrl)
    input  wire        reg_pcbr_set,      // has the PCBR ever been written
    input  wire [31:0] reg_base_bytes,    // DAFB   +0x000/+0x004, assembled
    input  wire [29:0] reg_stride_words,  // DAFB   +0x008 as written

    // ── Stage 2 -> 3: canonical mode descriptor ──────────────────────
    output wire [11:0] src_w_px,          // visible width  in PIXELS
    output wire [11:0] src_h_px,          // visible height in PIXELS (lines)
    output wire [2:0]  bpp_shift,         // log2(pixels per byte), sub-byte
    output wire [31:0] bytes_per_row,     // row pitch in BYTES
    output wire [31:0] fb_base_bytes,     // framebuffer origin in BYTES
    output wire [31:0] lut_depth,         // bits per pixel: 1/2/4/8/24, 0 = n/a
    output wire [2:0]  bytes_per_px,      // BYTES per pixel for >=1 B/px depths
    output wire        depth_supported,   // scanner can render this depth

    // ── Sampling control ─────────────────────────────────────────────
    output wire        base_override_512  // recalc_mode's `m_base = 0x1000`
);

    // ── Geometry, in the order recalc_mode() evaluates it ─────────────
    // Swatch horizontal counters run in units of `clockdiv` PIXELS.
    wire [11:0] raw_w_px = reg_hfp - reg_hal;
    // Half-line units, so each param is halved BEFORE the subtraction --
    // dafb.cpp:829 does exactly that, and it is not the same as
    // (VFP - VAL) >> 1 when the low bits differ.
    wire [11:0] raw_h_lines = (reg_vfp >> 1) - (reg_val >> 1);

    // ── MAME's `(m_htotal > 0) && (m_vtotal > 0)` guard: NOT reproduced ──
    // recalc_mode() wraps its whole body in that test, where m_htotal = HPIX
    // and m_vtotal = VFPEQ >> 1.  It exists to keep `m_screen->configure()`
    // from dividing the pixel clock by a zero total (dafb.cpp:864-867); its
    // only effect on the GEOMETRY is to hold the previous value.
    //
    // This shim configures no screen, and no scanout consumer reads HPIX or
    // VFPEQ at all.  Reproducing the guard would therefore make a mode's
    // resolution depend on two registers nothing else in the design uses --
    // and the one case where the guard bites in MAME (a PCBR write before
    // the Swatch block is programmed, i.e. the AC842a probe at the head of
    // every captured boot trace) computes 0x0 here, which is exactly the
    // value the caller's reset state already holds.  Deliberate, documented
    // divergence.  HPIX (+0x148) and VFPEQ (+0x164) are therefore not inputs
    // to this stage at all; video.v stores them for register readback only.

    // ── The Quadra 700 512x384 fixup (dafb.cpp:832-838) ──────────────
    // MAME:
    //     // Quadra 700 programs the wrong base for the 512x384 mode and is
    //     // off-by-1 on the vertical res.
    //     if ((m_hres == 512) && (m_dafb_version == 1))
    //     { m_base = 0x1000; m_vres = 384; }
    //
    // It is a BASE OVERRIDE, not cosmetic: without it the scanner reads the
    // framebuffer 512 bytes below where Mac OS draws it, which at 1bpp lands
    // the visible span inside untouched row padding -- index 0, CLUT[0],
    // white -- i.e. "512x384 renders only a white square".
    //
    // MEASURED (MAME 0.285 write tap over a real macqd700 ROM boot at monitor
    // code 2): the ROM writes `W f9800000 = 00000007`, m_base = 7<<9 = 0x0E00,
    // versus the 0x1000 MAME substitutes.  A post-boot READBACK returns
    // 0x00000008 because recalc_mode() has already replaced m_base -- which is
    // why this looked correct for as long as anyone only read the register.
    //
    // Tested against the width BEFORE the clockdiv term and applied to the
    // height BEFORE the interlace doubling, matching MAME's ordering.  The
    // DAFB version is a compile-time property of this shim (it reports
    // version 1 at +0x2C, exactly the `m_dafb_version == 1` MAME tests), so
    // the version half of the condition is structurally true here.
    //
    // raw 512 is unique to this mode across the whole Q700 monitor list (the
    // others decode to raw 640/416/288/320/1326/1582 -- see the captured
    // table in tb/tb_mode_decode.cpp), and MAME carries the identical
    // collision risk.
    assign base_override_512 = (raw_w_px == 12'd512);
    wire [11:0] fixed_h_lines = base_override_512 ? 12'd384 : raw_h_lines;

    // ── clockdiv, and the convolution branch ─────────────────────────
    // clockdiv = 1 << ((m_ac842_pbctrl & 0x60) >> 5), i.e. the log2 IS
    // reg_pcbr[6:5].  Same register whose bits[4:2] select the depth below.
    // Captured values across the Q700 monitor list: 0x80 -> cd 1 (640x480,
    // 512x384), 0xa0 -> cd 2 (832x624, 640x870), 0xc0 -> cd 4 (1152x870),
    // 0x21/0xa1 -> cd 2 (PAL/NTSC encoder, convolution on).
    wire [1:0] clockdiv_log2 = reg_pcbr[6:5];
    wire       convolution   = reg_config[3];
    wire       interlace     = reg_config[2];

    // All modes with convolution enabled on the Q700 overstate the horizontal
    // resolution by 23 (dafb.cpp:849-852; MAME notes the documentation does
    // not show this and suspects an early-revision chip bug).
    assign src_w_px = convolution ? ((raw_w_px >> clockdiv_log2) - 12'd23)
                                  : (raw_w_px << clockdiv_log2);
    assign src_h_px = interlace ? (fixed_h_lines << 1) : fixed_h_lines;

    // ── Row pitch ─────────────────────────────────────────────────────
    // MAME sets m_stride = data << 2 in dafb_w case 8, then recalc_mode()
    // does `m_stride /= clockdiv` on the convolution branch.  That division
    // is IN PLACE on a stored member, so MAME re-divides on every subsequent
    // PCBR write -- an artefact rather than a model.  This shim instead pins
    // the convolution pitch at the 1024 bytes the encoder modes actually run
    // at, which is what every scanout tb has been validated against.  Noted
    // as a deliberate, documented divergence; see the report for §4.3.
    assign bytes_per_row = convolution ? 32'd1024
                                       : {reg_stride_words, 2'd0};

    // ── Framebuffer origin ────────────────────────────────────────────
    // Carried through unchanged.  The 512x384 base override is NOT applied
    // here: MAME's fixup WRITES m_base, so a later base write replaces it.
    // The caller owns that stored value and applies `base_override_512` to it
    // when it samples this function.
    assign fb_base_bytes = reg_base_bytes;

    // ── AC842 depth encoding ──────────────────────────────────────────
    // The five cases below are the COMPLETE AC842 mode set, verbatim from
    // MAME `dafb_base::ramdac_w` case 0x20 (dafb.cpp:791-816).  Codes 0x04,
    // 0x0C and 0x14 are genuinely unmapped in silicon -- MAME's switch has no
    // case for them and leaves m_mode untouched -- so `default` here is
    // faithful, not a gap.
    //
    // There is NO 16bpp code in this table, and that is correct for a Quadra
    // 700.  15/16bpp (x555) exists only on the AC842a CODEC, in the
    // dafb_q950 / dafb_memc / dafb_memcjr overrides (dafb.cpp:1120-1174,
    // :1260-1290, :1402-1430), and it is a TWO-register condition, not a
    // spare code in this field (dafb.cpp:1133-1140):
    //       if (((m_pcbr1 & 0xc0) == 0xc0) && ((pcbr0 & 0x06) == 0x06))
    //           m_mode = 5;   // 16bpp x555
    // where PCBR1 is a SECOND register multiplexed onto this same +0x220
    // offset.  Mac OS gates the depths it offers on the DAFB version at
    // +0x2C; the Q700 is version 1 (dafb.cpp:82) and DAFB II is version 3
    // (dafb.cpp:1097), so a version-1 DAFB is never offered Thousands and a
    // PCBR1 implementation alone would be dead code.
    //
    // NOTE the non-monotonic look of this table (0x18 -> 8bpp but 0x1c ->
    // 24bpp) invites a "bit 2 is a direct-colour flag, so 0x14 must be 16bpp"
    // reading.  That inference is WRONG.  Do not "fill in" 0x04/0x0C/0x14.
    wire [7:0] depth_code = reg_pcbr & 8'h1c;

    reg [31:0] lut_depth_r;
    always @(*) begin
        case (depth_code)
            8'h00:   lut_depth_r = 32'd1;
            8'h08:   lut_depth_r = 32'd2;
            8'h10:   lut_depth_r = 32'd4;
            8'h18:   lut_depth_r = 32'd8;
            8'h1c:   lut_depth_r = 32'd24;
            default: lut_depth_r = 32'd0;
        endcase
    end

    // bpp_shift = log2(pixels per byte) for the sub-byte depths.  0x1c
    // (24bpp) and the three unmapped codes fall to 0, which is NOT a
    // meaningful answer for 24bpp -- it is the safe "don't shift" value;
    // 24bpp consumers key off bytes_per_px == 4 instead.
    reg [2:0] bpp_shift_r;
    always @(*) begin
        case (depth_code)
            8'h00:   bpp_shift_r = 3'd3;   // 1bpp -> 8 px/byte
            8'h08:   bpp_shift_r = 3'd2;   // 2bpp -> 4 px/byte
            8'h10:   bpp_shift_r = 3'd1;   // 4bpp -> 2 px/byte
            8'h18:   bpp_shift_r = 3'd0;   // 8bpp -> 1 px/byte
            default: bpp_shift_r = 3'd0;
        endcase
    end

    // VRAM BYTES per pixel for the >=1-byte depths; 0 for sub-byte depths and
    // for the unmapped codes.
    //
    // 24bpp is 4 B/px, NOT 3.  This is load-bearing and was wrong in the
    // first cut of the arithmetic.  MAME dafb.cpp:340-350, `case 4: // 24 bpp`:
    //     u32 const *base = &m_vram[(y * (stride/4)) + (m_base/4)];
    //     for (int x = 0; x < m_hres; x++)  *scanline++ = *base++;
    // i.e. the framebuffer is walked as an array of 32-bit words, one word
    // per pixel, and the word IS the RGB value (no palette lookup).  Every
    // other mode in that switch reads through `vram8` (the big-endian byte
    // cast) -- only 24bpp reads u32s.  A packed-3-bytes reading of this mode
    // would shear every row.
    reg [2:0] bytes_per_px_r;
    always @(*) begin
        case (depth_code)
            8'h18:   bytes_per_px_r = 3'd1;   // 8bpp
            8'h1c:   bytes_per_px_r = 3'd4;   // 24bpp: xRGB, 4 B/px
            default: bytes_per_px_r = 3'd0;   // 1/2/4bpp: sub-byte
        endcase
    end

    // Depths the scanout datapath can actually render: the CLUT-indexed
    // sub-byte/byte depths (1/2/4/8bpp) plus direct-colour 24bpp.  The three
    // unmapped AC842 codes stay low.
    reg depth_ok_r;
    always @(*) begin
        case (depth_code)
            8'h00, 8'h08, 8'h10, 8'h18, 8'h1c: depth_ok_r = 1'b1;
            default:                           depth_ok_r = 1'b0;
        endcase
    end

    // Before the ROM has written the PCBR at all there is no depth to render.
    assign lut_depth       = reg_pcbr_set ? lut_depth_r    : 32'd0;
    assign bpp_shift       = reg_pcbr_set ? bpp_shift_r    : 3'd0;
    assign bytes_per_px    = reg_pcbr_set ? bytes_per_px_r : 3'd0;
    assign depth_supported = reg_pcbr_set && depth_ok_r;

endmodule

`default_nettype wire
