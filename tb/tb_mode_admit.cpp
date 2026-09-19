// tb_mode_admit.cpp -- the scan-out admission contract, as a table.
// ---------------------------------------------------------------------------
// mode_admit.v (docs/video_path_review.md S4.1 stage 3) is the ONLY place in
// the video path where an admission inequality lives.  This tb exists because
// the review's rule for a pure stage is that it should be "trivially
// unit-testable with a table, no testbench harness, no clock" -- and because
// the gates it owns are the ones that, when wrong, produce a silent black
// screen rather than a failure anyone can see.
//
// >>> THE MODEL BELOW IS WRITTEN FROM THE CONTRACT, NOT FROM THE RTL.
// >>>
// >>> That is the whole point.  `row_span_bytes` is computed here as a
// >>> PIXEL-DOMAIN ceiling division, ceil(row_px / 2^bpp_shift), because that
// >>> is what "how many bytes does a row of row_px pixels occupy" means.  The
// >>> RTL computes ((row_px-1) >> s) + 1, a byte-domain "highest index plus
// >>> one".  They are equal, and a transcription of the RTL into C++ would
// >>> prove nothing about whether either is right.
//
// The operator derivation this file pins down (see mode_admit.v's header):
//
//     stride gate   fb_stride_bytes >= row_span_bytes         COUNT vs COUNT
//     memory gate   frame_last_addr_bytes < FB_MAX_BYTES      ADDR  vs COUNT
//
// Scenario 3 is the load-bearing one: an EXACTLY packed framebuffer, where
// stride == span with no pad byte at all, must be ADMITTED.  That is the
// normal case for every real Q700 mode, and a gate written as `>` over the
// count would reject all of them -- black screen everywhere.  One byte less
// must be REFUSED, because the scanner would read into the next row.
// ---------------------------------------------------------------------------
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <random>
#include <string>
#include <vector>
#include "Vtb_mode_admit.h"

static Vtb_mode_admit* dut = nullptr;
static int passes = 0, fails = 0;

static void check(const std::string& what, uint64_t got, uint64_t want) {
    if (got == want) { passes++; return; }
    fails++;
    std::printf("  [FAIL] %s: got %llu want %llu\n", what.c_str(),
                (unsigned long long)got, (unsigned long long)want);
}

// ── Reject codes.  Wire protocol; mirrored from mode_admit.v's REJ_*. ──
enum { REJ_NONE = 0, REJ_DEPTH_UNSUP = 1, REJ_STRIDE_SHORT = 2,
       REJ_FRAME_OOM = 3, REJ_GEOMETRY_ZERO = 4 };

struct Cfg { uint32_t src_w, src_h; uint64_t fb_max_bytes; };
static const Cfg CFG_A{1152, 1024, 0x200000};   // production Q700 bound
static const Cfg CFG_B{1024,  512, 0x010000};   // tiny aperture
static const Cfg CFG_C{ 640,  480, 0x200000};   // narrow bound

struct Verdict {
    uint32_t row_px, row_count, last_row;
    uint32_t span, last_off, elem;
    uint64_t last_addr;
    bool direct, stride_fits, in_range, addr_adm, renderable, geom_zero;
    int reason;
};

// THE CONTRACT, in C++.  Not a transcription of the RTL -- see the header.
static Verdict model(const Cfg& c, uint32_t hres, uint32_t vres,
                     uint32_t bpp_shift, uint32_t bytes_per_px,
                     bool depth_ok, uint64_t base, uint64_t stride) {
    Verdict v{};
    v.direct = (bytes_per_px == 4);

    // A programmed geometry of 0, or one at/beyond the build-time scanner
    // window, is what the scanner clamps to the window itself.
    v.row_px    = (hres != 0 && hres < c.src_w) ? hres : c.src_w;
    v.row_count = (vres != 0 && vres < c.src_h) ? vres : c.src_h;
    v.last_row  = v.row_count - 1;

    // Bytes one visible row occupies.  Direct colour is 4 bytes per pixel.
    // An indexed depth packs 2^bpp_shift pixels into each byte, so a row of
    // row_px pixels needs ceil(row_px / 2^bpp_shift) bytes -- a partly-filled
    // final byte still costs a whole byte.
    const uint32_t px_per_byte = 1u << bpp_shift;
    v.span = v.direct ? v.row_px * 4u
                      : (v.row_px + px_per_byte - 1u) / px_per_byte;
    v.last_off = v.span - 1u;

    // The fetch port's addressing granule: a pixel at direct colour, a byte
    // when indexed.  Index of the last one in a row.
    v.elem = v.direct ? (v.row_px - 1u) : ((v.row_px - 1u) / px_per_byte);

    v.last_addr = base + stride * (uint64_t)v.last_row + v.last_off;

    // A row must fit inside the pitch between rows.  Equality is a fit.
    v.stride_fits = (stride >= v.span);
    // The last byte read must be a legal address.  FB_MAX_BYTES is a
    // capacity, so the highest legal address is FB_MAX_BYTES-1.
    v.in_range = (v.last_addr < c.fb_max_bytes);

    v.addr_adm   = v.stride_fits && v.in_range;
    v.renderable = depth_ok && v.addr_adm;
    v.geom_zero  = (hres == 0) || (vres == 0);

    v.reason = !depth_ok      ? REJ_DEPTH_UNSUP  :
               !v.stride_fits ? REJ_STRIDE_SHORT :
               !v.in_range    ? REJ_FRAME_OOM    :
               v.geom_zero    ? REJ_GEOMETRY_ZERO
                              : REJ_NONE;
    return v;
}

static void drive(uint32_t hres, uint32_t vres, uint32_t bpp_shift,
                  uint32_t bytes_per_px, bool depth_ok,
                  uint64_t base, uint64_t stride) {
    dut->hres_px         = hres;
    dut->vres_px         = vres;
    dut->bpp_shift       = bpp_shift;
    dut->bytes_per_px    = bytes_per_px;
    dut->depth_supported = depth_ok ? 1 : 0;
    dut->fb_base_bytes   = (uint32_t)base;
    dut->fb_stride_bytes = (uint32_t)stride;
    dut->eval();
}

// Compare every observable of configuration A.
static void check_a(const std::string& tag, uint32_t hres, uint32_t vres,
                    uint32_t s, uint32_t bpx, bool depth_ok,
                    uint64_t base, uint64_t stride) {
    drive(hres, vres, s, bpx, depth_ok, base, stride);
    Verdict m = model(CFG_A, hres, vres, s, bpx, depth_ok, base, stride);
    check(tag + ": renderable",      dut->a_renderable,            m.renderable);
    check(tag + ": addr_admissible", dut->a_addr_admissible,       m.addr_adm);
    check(tag + ": stride_fits",     dut->a_stride_fits,           m.stride_fits);
    check(tag + ": frame_in_range",  dut->a_frame_in_range,        m.in_range);
    check(tag + ": reject_reason",   dut->a_reject_reason,         (uint64_t)m.reason);
    check(tag + ": row_px",          dut->a_row_px,                m.row_px);
    check(tag + ": row_span_bytes",  dut->a_row_span_bytes,        m.span);
    check(tag + ": row_last_off",    dut->a_row_last_off_bytes,    m.last_off);
    check(tag + ": row_last_elem",   dut->a_row_last_elem_idx,     m.elem);
    check(tag + ": row_count",       dut->a_row_count,             m.row_count);
    check(tag + ": last_row",        dut->a_last_row,              m.last_row);
    check(tag + ": frame_last_addr", dut->a_frame_last_addr_bytes, m.last_addr);
    check(tag + ": direct_colour",   dut->a_direct_colour,         m.direct ? 1u : 0u);
    check(tag + ": geometry_zero",   dut->a_geometry_zero,         m.geom_zero ? 1u : 0u);
}

// The Q700 monitor-sense mode list (docs/video_path_review.md S3's table).
struct Mode { uint32_t w, h; const char* name; };
static const Mode Q700_MODES[] = {
    { 512, 384, "512x384"  },
    { 640, 480, "640x480"  },
    { 640, 870, "640x870"  },
    { 832, 624, "832x624"  },
    { 800, 600, "800x600"  },
    {1024, 768, "1024x768" },
    {1152, 870, "1152x870" },
};

// (bpp_shift, bytes_per_px, label) for every depth the scanner can present.
struct Depth { uint32_t s, bpx; const char* name; };
static const Depth DEPTHS[] = {
    { 3, 0, "1bpp"   },
    { 2, 0, "2bpp"   },
    { 1, 0, "4bpp"   },
    { 0, 1, "8bpp"   },
    { 0, 4, "24bpp"  },
};

int main() {
    dut = new Vtb_mode_admit;
    std::printf("-- tb_mode_admit: the scan-out admission contract --\n");

    // ── 1. The constant export ───────────────────────────────────────
    drive(640, 480, 0, 1, true, 0, 640);
    check("REJ_NONE export is 0", dut->a_reason_none_code, REJ_NONE);

    // ── 2. Every Q700 mode at every depth, tightly packed ────────────
    // Tight packing (stride == span) is the normal Mac case, and it is the
    // one that exercises the >= boundary on every single vector.
    std::printf("\n-- Q700 mode list x depth, stride == span (tight pack) --\n");
    for (const Mode& md : Q700_MODES) {
        for (const Depth& d : DEPTHS) {
            Verdict m = model(CFG_A, md.w, md.h, d.s, d.bpx, true, 0, 0);
            std::string tag = std::string(md.name) + "/" + d.name + " tight";
            check_a(tag, md.w, md.h, d.s, d.bpx, true, 0, m.span);
            // The tight pack must actually be ADMITTED, not merely modelled.
            // This is the assertion that a `>`-over-count gate would fail.
            drive(md.w, md.h, d.s, d.bpx, true, 0, m.span);
            check(tag + ": ADMITTED at stride == span", dut->a_stride_fits, 1);
        }
    }

    // ── 3. The stride boundary, both sides ───────────────────────────
    // 832x624 is the mode the review flagged as passing "with one byte of
    // margin".  It is an exact fit, and an exact fit must be admitted.
    std::printf("\n-- the stride boundary --\n");
    {
        // 832x624 @ 24bpp direct: span 3328, last_off 3327.
        Verdict m = model(CFG_A, 832, 624, 0, 4, true, 0, 0);
        check("832x624x24bpp span is 3328",     m.span,     3328u);
        check("832x624x24bpp last_off is 3327", m.last_off, 3327u);
        check_a("832x624x24 stride=3328 (exact)", 832, 624, 0, 4, true, 0, 3328);
        drive(832, 624, 0, 4, true, 0, 3328);
        check("832x624x24 exact fit is ADMITTED", dut->a_stride_fits, 1);
        drive(832, 624, 0, 4, true, 0, 3327);
        check("832x624x24 one byte short is REFUSED", dut->a_stride_fits, 0);
        check("832x624x24 one byte short says STRIDE_SHORT",
              dut->a_reject_reason, REJ_STRIDE_SHORT);
        drive(832, 624, 0, 4, true, 0, 3329);
        check("832x624x24 one pad byte is ADMITTED", dut->a_stride_fits, 1);

        // 832x624 @ 8bpp: span 832, last_off 831 -- the original 832 > 831.
        Verdict m8 = model(CFG_A, 832, 624, 0, 1, true, 0, 0);
        check("832x624x8bpp span is 832",     m8.span,     832u);
        check("832x624x8bpp last_off is 831", m8.last_off, 831u);
        drive(832, 624, 0, 1, true, 0, 832);
        check("832x624x8 stride 832 ADMITTED", dut->a_stride_fits, 1);
        drive(832, 624, 0, 1, true, 0, 831);
        check("832x624x8 stride 831 REFUSED",  dut->a_stride_fits, 0);
    }

    // ── 4. The memory boundary, both sides ───────────────────────────
    // Configuration B has a 64 KiB aperture, so the last byte can be placed
    // exactly on and exactly past the limit with ordinary numbers.
    std::printf("\n-- the frame-out-of-memory boundary --\n");
    {
        // 8bpp, 256x256 inside a 1024x512 window: span 256, last_off 255.
        // last_addr = base + stride*255 + 255.  Choose stride 256:
        //   base + 65280 + 255 = base + 65535.  base 0 -> 65535 == LIMIT-1.
        drive(256, 256, 0, 1, true, 0, 256);
        check("last byte == LIMIT-1 is IN RANGE", dut->b_frame_in_range, 1);
        check("last byte == LIMIT-1 last_addr",   dut->b_frame_last_addr_bytes,
              0xFFFFu);
        drive(256, 256, 0, 1, true, 1, 256);
        check("last byte == LIMIT is OUT OF RANGE", dut->b_frame_in_range, 0);
        check("last byte == LIMIT says FRAME_OOM",  dut->b_reject_reason,
              REJ_FRAME_OOM);
        check("last byte == LIMIT last_addr",       dut->b_frame_last_addr_bytes,
              0x10000u);
    }

    // ── 5. Reason priority ───────────────────────────────────────────
    // The ladder reports the FIRST failing gate, so a tuple that fails
    // several must name the earliest.  A bitmask-order slip here is exactly
    // the "names the wrong gate confidently" failure the reason channel was
    // added to prevent.
    std::printf("\n-- reject-reason priority --\n");
    {
        // depth unsupported AND stride short AND out of memory.
        drive(256, 256, 0, 1, false, 0xF00000, 1);
        check("depth beats stride and range", dut->a_reject_reason, REJ_DEPTH_UNSUP);
        // stride short AND out of memory, depth fine.
        drive(256, 256, 0, 1, true, 0xF00000, 1);
        check("stride beats range", dut->a_reject_reason, REJ_STRIDE_SHORT);
        // only out of memory.
        drive(1152, 1024, 0, 1, true, 0x1F0000, 1152);
        check("range alone reports FRAME_OOM", dut->a_reject_reason, REJ_FRAME_OOM);
        // everything fine but the window is empty.
        drive(0, 480, 0, 1, true, 0, 4096);
        check("geometry zero is ADMITTED",     dut->a_addr_admissible, 1);
        check("geometry zero is still NAMED",  dut->a_reject_reason,
              REJ_GEOMETRY_ZERO);
        // vres 0 clamps to SRC_H (1024 rows), so the stride has to be small
        // enough that 1024 rows still fit the 2 MiB aperture -- otherwise the
        // tuple is genuinely FRAME_OOM and that outranks GEOMETRY_ZERO, which
        // is what this vector originally (wrongly) expected.
        // 2048*1023 + 639 = 2,095,743 < 0x200000.
        drive(640, 0, 0, 1, true, 0, 2048);
        check("vres zero is IN RANGE at stride 2048", dut->a_addr_admissible, 1);
        check("vres zero is also NAMED", dut->a_reject_reason, REJ_GEOMETRY_ZERO);
    }

    // ── 6. The clamp ─────────────────────────────────────────────────
    // Configuration C is 640x480.  Every wider Q700 mode must clamp to it,
    // and a zero geometry must clamp too -- both are what the scanner
    // actually reads.
    std::printf("\n-- clamping to the build-time scanner window --\n");
    for (const Mode& md : Q700_MODES) {
        drive(md.w, md.h, 0, 1, true, 0, 4096);
        Verdict m = model(CFG_C, md.w, md.h, 0, 1, true, 0, 4096);
        check(std::string(md.name) + " clamps row_px",    dut->c_row_px,    m.row_px);
        check(std::string(md.name) + " clamps row_count", dut->c_row_count, m.row_count);
    }
    drive(0, 0, 0, 1, true, 0, 4096);
    check("hres 0 clamps to SRC_W", dut->c_row_px,    640u);
    check("vres 0 clamps to SRC_H", dut->c_row_count, 480u);
    drive(640, 480, 0, 1, true, 0, 4096);
    check("hres == SRC_W clamps to SRC_W", dut->c_row_px, 640u);

    // ── 7. Sub-byte packing on a NON-POWER-OF-TWO window ─────────────
    // Configuration A's SRC_W is 1152.  This is the case mode_admit.v warns
    // about: a "simplified" span of row_px>>s is right for a power of two and
    // wrong here.  Widths that are not a multiple of the packing factor are
    // the ones that separate ceil from floor.
    std::printf("\n-- sub-byte packing, non-power-of-two widths --\n");
    {
        const uint32_t widths[] = {1, 2, 3, 5, 7, 9, 15, 17, 31, 33, 63, 65,
                                   127, 129, 255, 257, 511, 513, 639, 641,
                                   831, 833, 1151};
        for (uint32_t w : widths) {
            for (uint32_t s = 0; s <= 3; s++) {
                char tag[96];
                std::snprintf(tag, sizeof tag, "w=%u shift=%u", w, s);
                check_a(tag, w, 600, s, (s == 0) ? 1 : 0, true, 0, 4096);
            }
        }
        // Spot-check the ceiling explicitly: 1151 pixels at 8 px/byte is
        // 143.875 bytes, i.e. 144, not 143.
        drive(1151, 600, 3, 0, true, 0, 4096);
        check("1151 px at 1bpp needs 144 bytes", dut->a_row_span_bytes, 144u);
        check("1151 px at 1bpp last byte is 143", dut->a_row_last_off_bytes, 143u);
        // ...and that a floor would have said 143.
        check("floor(1151/8) would be 143 -- proving ceil is load-bearing",
              1151u / 8u, 143u);
    }

    // ── 8. Randomised sweep ──────────────────────────────────────────
    std::printf("\n-- randomised sweep vs the contract model --\n");
    {
        std::mt19937 rng(0xAD3117);
        int mismatches = 0;
        for (int i = 0; i < 200000; i++) {
            uint32_t hres = rng() % 1400;
            uint32_t vres = rng() % 1200;
            const Depth& d = DEPTHS[rng() % 5];
            bool depth_ok = (rng() & 7) != 0;
            uint64_t base   = (uint64_t)(rng() % 0x300000);
            uint64_t stride = (uint64_t)(rng() % 8192);
            drive(hres, vres, d.s, d.bpx, depth_ok, base, stride);
            Verdict m = model(CFG_A, hres, vres, d.s, d.bpx, depth_ok,
                              base, stride);
            if (dut->a_renderable != (m.renderable ? 1u : 0u) ||
                dut->a_addr_admissible != (m.addr_adm ? 1u : 0u) ||
                dut->a_stride_fits != (m.stride_fits ? 1u : 0u) ||
                dut->a_frame_in_range != (m.in_range ? 1u : 0u) ||
                dut->a_reject_reason != (uint32_t)m.reason ||
                dut->a_row_span_bytes != m.span ||
                dut->a_row_last_off_bytes != m.last_off ||
                dut->a_row_last_elem_idx != m.elem ||
                dut->a_frame_last_addr_bytes != m.last_addr) {
                if (mismatches < 8) {
                    std::printf("  [FAIL] random hres=%u vres=%u %s depth=%d "
                                "base=%llu stride=%llu: rtl{ren=%u adm=%u sf=%u "
                                "ir=%u rr=%u span=%u lo=%u el=%u la=%llu} "
                                "model{ren=%d adm=%d sf=%d ir=%d rr=%d span=%u "
                                "lo=%u el=%u la=%llu}\n",
                        hres, vres, d.name, depth_ok ? 1 : 0,
                        (unsigned long long)base, (unsigned long long)stride,
                        dut->a_renderable, dut->a_addr_admissible,
                        dut->a_stride_fits, dut->a_frame_in_range,
                        dut->a_reject_reason, (uint32_t)dut->a_row_span_bytes,
                        (uint32_t)dut->a_row_last_off_bytes,
                        (uint32_t)dut->a_row_last_elem_idx,
                        (unsigned long long)dut->a_frame_last_addr_bytes,
                        m.renderable, m.addr_adm, m.stride_fits, m.in_range,
                        m.reason, m.span, m.last_off, m.elem,
                        (unsigned long long)m.last_addr);
                }
                mismatches++;
            }
        }
        check("200000 random tuples match the contract", mismatches, 0);
    }

    // ── 9. All three configurations agree where they must ────────────
    // A and B differ ONLY in window and aperture, so for a tuple inside both
    // the span/offset arithmetic must be identical.  This catches a change
    // that accidentally makes the derivation depend on the aperture.
    std::printf("\n-- configuration independence --\n");
    for (const Depth& d : DEPTHS) {
        drive(512, 384, d.s, d.bpx, true, 0, 4096);
        check(std::string("A/B agree on span at ") + d.name,
              dut->a_row_span_bytes, dut->b_row_span_bytes);
        check(std::string("A/B agree on last_off at ") + d.name,
              dut->a_row_last_off_bytes, dut->b_row_last_off_bytes);
    }

    std::printf("\n-- tb_mode_admit: %d PASS / %d FAIL --\n", passes, fails);
    delete dut;
    return fails ? 1 : 0;
}
