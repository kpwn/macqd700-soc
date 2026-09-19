// tb_mode_decode.cpp -- the Quadra 700 monitor-sense table, in lockstep
// against MAME.
//
// WHAT THIS EXISTS FOR.  The DAFB mode arithmetic decides the resolution,
// depth and row pitch of every frame the machine ever draws, and until this
// file landed NOTHING tested it against the golden model.  A 2x error in
// `hres` shipped to hardware and presented as a black screen
// (docs/video_path_review.md §1, §4.3).  The table it got wrong is small,
// closed and enumerable: eleven Apple Display Sense codes, five AC842 depths.
//
// THE GOLDEN MODEL is MAME 0.285 `dafb_base` (src/mame/apple/dafb.cpp),
// reimplemented below in `MameDafb` directly from that source -- including,
// critically, the fact that `recalc_mode()` is reachable from EXACTLY ONE
// call site: the AC842 PCBR write in `ramdac_w` case 0x20 (dafb.cpp:816).
// Enumerated over the whole file the call sites are :816 (dafb_base), :1164
// (q950), :1291 (memc), :1433 (memcjr) -- all four inside a `ramdac_w`.
// `swatch_w` (HAL/HFP/VAL/VFP) and `dafb_w` (base/stride/config) never
// recompute.  So MAME's m_hres/m_vres are a SNAPSHOT taken when the driver
// finishes a mode set, not a live function of the Swatch registers.
//
// THE STIMULUS is not invented either: tb_mode_decode_traces.h holds every
// DAFB register write a real macqd700 ROM boot performs, captured with a MAME
// write tap, once per monitor code.  The test replays those writes into the
// real video.v over real AXI, in the real order.
//
// PASSES
//   1  steady state    -- replay each sense trace, check the settled
//                         descriptor against MAME and against the resolution
//                         MAME's own monitor list names.
//   2  write lockstep  -- check the descriptor after EVERY write of every
//                         trace.  This is the pass the pre-fix RTL fails:
//                         evaluated live, `hres` mixes the Swatch pair of one
//                         mode with the clockdiv of another.
//   3  depth sweep     -- every sense code x every AC842 depth code, driven
//                         through a real PCBR write as Mac OS does it.
//   4  mode transition -- every ordered pair of sense codes, replayed back to
//                         back with NO reset, in lockstep.
//   5  runtime reprogram -- the SHIPPED failure.  Every boot trace opens by
//                         programming 640x480 (PCBR 0x80, clockdiv 1) before
//                         it reads the sense line, so a full trace replay
//                         always resets the clockdiv on the way past and pass
//                         4 never forms the pairing the board hit.  A driver
//                         changing modes on a LIVE machine does not re-run
//                         that init: it writes the mode-set block only, and
//                         the PCBR carrying the new clockdiv is its LAST
//                         write (true in all 11 captures).  So between the
//                         new HFP landing and the new PCBR landing, the new
//                         Swatch pair sits against the OLD clockdiv.  From
//                         1152x870 (PCBR 0xc0 -> clockdiv 4) to 832x624
//                         (HFP-HAL = 416) that window reads
//                             416 << 2 = 1664
//                         which is exactly what the board reported for an
//                         832-wide mode, with the correct stride (832) and
//                         the correct vres (624) beside it.
//
// NEGATIVE CONTROLS.  `MD_SELFTEST=<n>` perturbs the MODEL so a green run
// proves the comparison is live rather than vacuous.  RTL-side mutants are
// driven from outside (see the §4.3 handoff); the point of MD_SELFTEST is
// that a harness bug -- reading the wrong signal, never ticking, comparing a
// value against itself -- cannot hide behind a green table.

#include <verilated.h>
#include "Vtb_mode_decode.h"
#include "tb_mode_decode_traces.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

static Vtb_mode_decode* dut = nullptr;
static vluint64_t       main_time = 0;
static int              passes = 0, failures = 0;
static int              g_selftest = 0;
static bool             g_verbose = false;
// MD_FULL=1 keeps replaying a trace past its first divergence.  Off by
// default only because one broken epoch cascades into every later write
// and buries the log; on, when you want the whole divergence profile.
static bool             g_full = false;

static void tick() {
    dut->clk = 0; dut->eval(); main_time++;
    dut->clk = 1; dut->eval(); main_time++;
}

static void check(bool ok, const char* fmt, ...) {
    va_list ap; char buf[512];
    va_start(ap, fmt); vsnprintf(buf, sizeof buf, fmt, ap); va_end(ap);
    if (ok) { passes++; if (g_verbose) printf("  PASS %s\n", buf); }
    else    { failures++; printf("  FAIL %s\n", buf); }
}

// ── AXI4-Lite write BFM ──────────────────────────────────────────────
static void axi_write(uint32_t off, uint32_t data) {
    dut->s_axi_awaddr  = off;
    dut->s_axi_awvalid = 1;
    dut->s_axi_wdata   = data;
    dut->s_axi_wstrb   = 0xF;
    dut->s_axi_wvalid  = 1;
    dut->s_axi_bready  = 1;
    for (int i = 0; i < 64; i++) {
        tick();
        if (dut->s_axi_awready) dut->s_axi_awvalid = 0;
        if (dut->s_axi_wready)  dut->s_axi_wvalid  = 0;
        if (dut->s_axi_bvalid)  break;
    }
    tick();
    dut->s_axi_awvalid = 0;
    dut->s_axi_wvalid  = 0;
    dut->s_axi_bready  = 0;
    // The descriptor is sampled one cycle after the write commits (video.v's
    // recalc_mode() equivalent).  Settle well past that.
    for (int i = 0; i < 8; i++) tick();
}

static void reset_dut(int sense) {
    dut->monitor_sense = sense;
    dut->rst = 1;
    for (int i = 0; i < 32; i++) tick();
    dut->rst = 0;
    for (int i = 0; i < 32; i++) tick();
}

// ══════════════════════════════════════════════════════════════════════
// MAME dafb_base, reimplemented from src/mame/apple/dafb.cpp
// ══════════════════════════════════════════════════════════════════════
struct MameDafb {
    // Register file, exactly MAME's members.
    uint32_t m_base = 0, m_stride = 0, m_config = 0;
    uint32_t m_horizontal_params[16] = {0};
    uint32_t m_vertical_params[16]   = {0};
    uint8_t  m_ac842_pbctrl = 0;
    int      m_mode = -1;          // -1 = never set (MAME leaves it at 0)
    bool     pbctrl_written = false;
    // recalc_mode() outputs.
    int      m_hres = 0, m_vres = 0;

    // dafb.cpp horizontal/vertical param indices: the write handler stores at
    // `offset - (0x24/4)` and `offset - (0x4c/4)`, i.e. HAL is byte 0x40 ->
    // index 7, HFP 0x44 -> 8, HPIX 0x48 -> 9; VAL 0x5c -> 4, VFP 0x60 -> 5,
    // VFPEQ 0x64 -> 6.
    enum { HAL = 7, HFP = 8, HPIX = 9 };
    enum { VAL = 4, VFP = 5, VFPEQ = 6 };

    void write(uint32_t off, uint32_t data) {
        if (off < 0x100) {                                  // dafb_w
            switch (off) {
                case 0x00: m_base = (m_base & ~0x1FFE00u) | ((data & 0xFFFu) << 9); break;
                case 0x04: m_base = (m_base & ~0x1E0u)    | ((data & 0xFu)   << 5); break;
                case 0x08: m_stride = data << 2; break;     // "stride in 32-bit words"
                case 0x10: m_config = data; break;
                default: break;
            }
        } else if (off < 0x200) {                           // swatch_w
            uint32_t o = off - 0x100;
            if (o >= 0x24 && o <= 0x48)      m_horizontal_params[(o - 0x24) / 4] = data;
            else if (o >= 0x4c && o <= 0x64) m_vertical_params[(o - 0x4c) / 4]   = data;
            // NOTE: no recalc_mode() here.  That absence is the whole point.
        } else if (off < 0x300) {                           // ramdac_w
            if ((off - 0x200) == 0x20) {
                m_ac842_pbctrl = (uint8_t)data;
                pbctrl_written = true;
                switch (data & 0x1c) {                      // dafb.cpp:791-816
                    case 0x00: m_mode = 0; break;           // 1bpp
                    case 0x08: m_mode = 1; break;           // 2bpp
                    case 0x10: m_mode = 2; break;           // 4bpp
                    case 0x18: m_mode = 3; break;           // 8bpp
                    case 0x1c: m_mode = 4; break;           // 24bpp
                    default:   break;                       // unmapped: retained
                }
                recalc_mode();
            }
        }
    }

    void recalc_mode() {                                    // dafb.cpp:821-868
        int htotal = (int)m_horizontal_params[HPIX];
        int vtotal = (int)(m_vertical_params[VFPEQ] >> 1);
        // MAME guards the whole body on both totals being non-zero.  Our shim
        // deliberately omits that guard (rtl/mac/mode_decode.v documents why:
        // the guard protects m_screen->configure()'s refresh division, and no
        // scanout consumer reads HPIX or VFPEQ).  The one case where it bites
        // in MAME -- a PCBR write before the Swatch block is programmed --
        // computes 0x0 either way, so the models still agree.  Reproduced
        // here as a comment rather than as code so the divergence is visible.
        (void)htotal; (void)vtotal;

        int hres = (int)m_horizontal_params[HFP] - (int)m_horizontal_params[HAL];
        int vres = (int)(m_vertical_params[VFP] >> 1)
                 - (int)(m_vertical_params[VAL] >> 1);
        if (hres == 512) { m_base = 0x1000; vres = 384; }   // Q700 fixup
        const int clockdiv = 1 << ((m_ac842_pbctrl & 0x60) >> 5);
        if ((m_config >> 3) & 1) { hres /= clockdiv; hres -= 23; }
        else                     { hres *= clockdiv; }
        // MAME also does `m_stride /= clockdiv` on the convolution branch.
        // It mutates a stored member, so it re-divides on every later PCBR
        // write -- an artefact, not a model.  The shim pins the convolution
        // pitch at 1024 instead; that divergence is asserted explicitly in
        // check_descriptor() rather than absorbed here.
        if ((m_config >> 2) & 1) vres <<= 1;
        m_hres = hres & 0xFFF;                              // shim regs are 12-bit
        m_vres = vres & 0xFFF;
    }

    bool convolution() const { return (m_config >> 3) & 1; }

    // The shim's row pitch: `config[3] ? 1024 : stride_reg << 2`.
    uint32_t shim_bytes_per_row() const {
        return convolution() ? 1024u : m_stride;
    }

    // AC842 depth table, as the shim reports it.  MAME retains m_mode for the
    // three unmapped codes (0x04/0x0C/0x14); the shim decodes them to "no
    // depth".  Returns -1 for the unmapped codes so the caller can assert the
    // divergence instead of hiding it.
    int shim_lut_depth() const {
        if (!pbctrl_written) return 0;
        switch (m_ac842_pbctrl & 0x1c) {
            case 0x00: return 1;
            case 0x08: return 2;
            case 0x10: return 4;
            case 0x18: return 8;
            case 0x1c: return 24;
            default:   return -1;
        }
    }
    int shim_bpp_shift() const {
        if (!pbctrl_written) return 0;
        switch (m_ac842_pbctrl & 0x1c) {
            case 0x00: return 3; case 0x08: return 2;
            case 0x10: return 1; default:   return 0;
        }
    }
    int shim_bytes_per_px() const {
        if (!pbctrl_written) return 0;
        switch (m_ac842_pbctrl & 0x1c) {
            case 0x18: return 1; case 0x1c: return 4; default: return 0;
        }
    }
    bool shim_depth_supported() const { return shim_lut_depth() > 0; }
};

// ── Compare the DUT's descriptor against the model ───────────────────
// `where` names the exact stimulus point so a failure is self-locating.
static int check_descriptor(const MameDafb& m, const std::string& where) {
    int before = failures;

    int want_w = m.m_hres, want_h = m.m_vres;
    uint32_t want_base = m.m_base;
    if (g_selftest == 1) want_w  += 1;
    if (g_selftest == 2) want_h  += 1;
    if (g_selftest == 3) want_base ^= 0x20u;

    check((int)dut->src_w_px == want_w,
          "%s: src_w_px=%u want %d  [MAME recalc_mode(): (HFP 0x%03x - HAL 0x%03x) "
          "%s clockdiv %d, config 0x%02x]",
          where.c_str(), (unsigned)dut->src_w_px, want_w,
          m.m_horizontal_params[MameDafb::HFP], m.m_horizontal_params[MameDafb::HAL],
          m.convolution() ? "/" : "*",
          1 << ((m.m_ac842_pbctrl & 0x60) >> 5), m.m_config);
    check((int)dut->src_h_px == want_h,
          "%s: src_h_px=%u want %d  [(VFP 0x%03x >> 1) - (VAL 0x%03x >> 1)]",
          where.c_str(), (unsigned)dut->src_h_px, want_h,
          m.m_vertical_params[MameDafb::VFP], m.m_vertical_params[MameDafb::VAL]);
    check(dut->fb_base_bytes == want_base,
          "%s: fb_base_bytes=0x%06x want 0x%06x",
          where.c_str(), (unsigned)dut->fb_base_bytes, want_base);

    uint32_t want_row = m.shim_bytes_per_row();
    if (g_selftest == 4) want_row += 4;
    check(dut->bytes_per_row == want_row,
          "%s: bytes_per_row=%u want %u%s",
          where.c_str(), (unsigned)dut->bytes_per_row, want_row,
          m.convolution()
              ? "  [convolution: shim pins 1024, MAME divides the stride -- "
                "documented divergence, mode_decode.v]" : "");

    int want_depth = m.shim_lut_depth();
    if (want_depth < 0) {
        // Unmapped AC842 code.  MAME retains m_mode; the shim reports "no
        // depth".  Assert the shim's documented behaviour explicitly.
        check(dut->lut_depth == 0 && dut->depth_supported == 0,
              "%s: unmapped AC842 code 0x%02x -> shim reports no depth "
              "(lut_depth=%u supported=%u); MAME retains m_mode=%d",
              where.c_str(), m.m_ac842_pbctrl & 0x1c,
              (unsigned)dut->lut_depth, (unsigned)dut->depth_supported, m.m_mode);
    } else {
        if (g_selftest == 5) want_depth = want_depth ? 0 : 1;
        check((int)dut->lut_depth == want_depth,
              "%s: lut_depth=%u want %d (PCBR 0x%02x & 0x1c = 0x%02x)",
              where.c_str(), (unsigned)dut->lut_depth, want_depth,
              m.m_ac842_pbctrl, m.m_ac842_pbctrl & 0x1c);
        int want_shift = m.shim_bpp_shift();
        if (g_selftest == 6) want_shift ^= 1;
        check((int)dut->bpp_shift == want_shift,
              "%s: bpp_shift=%u want %d", where.c_str(),
              (unsigned)dut->bpp_shift, want_shift);
        check((int)dut->bytes_per_px == m.shim_bytes_per_px(),
              "%s: bytes_per_px=%u want %d", where.c_str(),
              (unsigned)dut->bytes_per_px, m.shim_bytes_per_px());
        check((int)dut->depth_supported == (m.shim_depth_supported() ? 1 : 0),
              "%s: depth_supported=%u want %d", where.c_str(),
              (unsigned)dut->depth_supported, m.shim_depth_supported() ? 1 : 0);
    }
    return failures - before;
}

// Replay a trace into both DUT and model.  `lockstep` checks after EVERY
// write; otherwise only the settled state is checked by the caller.
static void replay(const SenseTrace& t, MameDafb& m, bool lockstep,
                   const char* tag, int stop_after_first_fail = 0) {
    for (int i = 0; i < t.n; i++) {
        axi_write(t.w[i].off, t.w[i].data);
        m.write(t.w[i].off, t.w[i].data);
        if (lockstep) {
            char buf[192];
            snprintf(buf, sizeof buf, "%s w%d/+0x%03x=0x%x", tag, i, t.w[i].off,
                     t.w[i].data);
            int nf = check_descriptor(m, buf);
            if (nf && stop_after_first_fail && !g_full) return;  // readable log
        }
    }
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_mode_decode;
    if (const char* e = getenv("MD_SELFTEST")) g_selftest = atoi(e);
    g_verbose = getenv("MD_VERBOSE") && atoi(getenv("MD_VERBOSE"));
    g_full    = getenv("MD_FULL")    && atoi(getenv("MD_FULL"));

    if (g_selftest)
        printf("[negative control] MD_SELFTEST=%d -- the MODEL is perturbed; "
               "every affected check MUST go red\n", g_selftest);

    int total_writes = 0;
    for (int i = 0; i < N_SENSE_TRACES; i++) total_writes += SENSE_TRACES[i].n;
    printf("Q700 DAFB monitor-sense table, in lockstep against MAME 0.285 "
           "dafb_base (%d sense codes, %d captured ROM writes)\n",
           N_SENSE_TRACES, total_writes);

    // ══ PASS 1: settled descriptor per monitor-sense code ═════════════
    printf("\n---- PASS 1: settled descriptor per sense code ----\n");
    for (int i = 0; i < N_SENSE_TRACES; i++) {
        const SenseTrace& t = SENSE_TRACES[i];
        MameDafb m;
        reset_dut(t.name_sense);
        replay(t, m, false, t.name);
        printf("  sense 0x%02X %-13s %-38s -> %4ux%-4u bpp=%-2d pitch=%-5u "
               "base=0x%06x\n",
               t.name_sense, t.name, t.monitor,
               (unsigned)dut->src_w_px, (unsigned)dut->src_h_px,
               (int)dut->lut_depth, (unsigned)dut->bytes_per_row,
               (unsigned)dut->fb_base_bytes);
        check_descriptor(m, std::string("settled/") + t.name);
        // ...and against the resolution MAME's own monitor list NAMES, so a
        // model that merely agrees with a broken DUT still gets caught.
        if (t.want_w_px) {
            int want_w = t.want_w_px;
            if (g_selftest == 7) want_w *= 2;
            check((int)dut->src_w_px == want_w,
                  "settled/%s: src_w_px=%u, MAME's monitor list names %d",
                  t.name, (unsigned)dut->src_w_px, want_w);
            check((int)dut->src_h_px == t.want_h_px,
                  "settled/%s: src_h_px=%u, MAME's monitor list names %d",
                  t.name, (unsigned)dut->src_h_px, t.want_h_px);
        }
    }

    // ══ PASS 2: write-by-write lockstep ═══════════════════════════════
    // Checks the descriptor after every single register write of every boot
    // trace, not just the settled state.  A live-evaluated `hres` diverges
    // from MAME the moment the Swatch pair and the PCBR clockdiv belong to
    // different mode epochs, which is most of a mode set.
    printf("\n---- PASS 2: write-by-write lockstep over every boot trace ----\n");
    for (int i = 0; i < N_SENSE_TRACES; i++) {
        const SenseTrace& t = SENSE_TRACES[i];
        MameDafb m;
        reset_dut(t.name_sense);
        int before = failures;
        replay(t, m, true, t.name, 1);
        printf("  sense 0x%02X %-13s %d writes, %s\n", t.name_sense, t.name, t.n,
               failures == before ? "in lockstep" : "DIVERGED");
    }

    // ══ PASS 3: every sense code x every AC842 depth ══════════════════
    // Mac OS changes depth by rewriting the PCBR on a live scanner, keeping
    // the clockdiv field.  Both DUT and MAME recompute on that write, so this
    // is a direct comparison with no caveats.
    printf("\n---- PASS 3: sense code x AC842 depth ----\n");
    static const uint32_t DEPTH_CODE[] = {0x00, 0x04, 0x08, 0x0c, 0x10,
                                          0x14, 0x18, 0x1c};
    for (int i = 0; i < N_SENSE_TRACES; i++) {
        const SenseTrace& t = SENSE_TRACES[i];
        MameDafb m;
        reset_dut(t.name_sense);
        replay(t, m, false, t.name);
        const uint8_t keep = (uint8_t)(m.m_ac842_pbctrl & ~0x1c);
        for (uint32_t dc : DEPTH_CODE) {
            uint32_t pcbr = keep | dc;
            axi_write(0x220, pcbr);
            m.write(0x220, pcbr);
            char buf[160];
            snprintf(buf, sizeof buf, "depth/%s/PCBR=0x%02x", t.name, pcbr);
            check_descriptor(m, buf);
        }
        printf("  sense 0x%02X %-13s 8 AC842 codes (5 mapped + 3 unmapped) "
               "on PCBR base 0x%02x\n", t.name_sense, t.name, keep);
    }

    // ══ PASS 4: mode -> mode transition, NO reset ═════════════════════
    // The pass that reproduces the SHIPPED number.  Nothing in the tree has
    // ever driven one Q700 monitor geometry on top of another on a live
    // register file: tb-dafb-mode-matrix resets between modes, and its depth
    // transitions keep the geometry fixed.  So the state that put 1664 on the
    // board -- one mode's Swatch pair against another mode's clockdiv -- was
    // structurally unreachable by every existing test.
    printf("\n---- PASS 4: mode -> mode transition, no reset, in lockstep ----\n");
    for (int a = 0; a < N_SENSE_TRACES; a++) {
        for (int b = 0; b < N_SENSE_TRACES; b++) {
            if (a == b) continue;
            const SenseTrace& ta = SENSE_TRACES[a];
            const SenseTrace& tb = SENSE_TRACES[b];
            MameDafb m;
            reset_dut(ta.name_sense);
            replay(ta, m, false, ta.name);            // settle into `from`
            dut->monitor_sense = tb.name_sense;
            char tag[96];
            snprintf(tag, sizeof tag, "%s->%s", ta.name, tb.name);
            int before = failures;
            replay(tb, m, true, tag, 1);
            if (failures != before)
                printf("  %-30s DIVERGED\n", tag);
        }
    }
    printf("  %d ordered mode pairs replayed\n",
           N_SENSE_TRACES * (N_SENSE_TRACES - 1));

    // ══ PASS 5: runtime mode reprogram (the shipped failure) ══════════
    // Replay only the MODE-SET BLOCK of `to` onto a settled `from`, with no
    // reset and no re-init.  In every capture that block begins at the last
    // write to +0x000 (base high) and ends with the AC842 PCBR at +0x220, so
    // it is taken from the trace rather than composed here.
    printf("\n---- PASS 5: runtime mode reprogram onto a live mode ----\n");
    int worst_w = 0; std::string worst_where;
    for (int a = 0; a < N_SENSE_TRACES; a++) {
        for (int b = 0; b < N_SENSE_TRACES; b++) {
            if (a == b) continue;
            const SenseTrace& ta = SENSE_TRACES[a];
            const SenseTrace& tb = SENSE_TRACES[b];
            int start = -1;
            for (int i = 0; i < tb.n; i++) if (tb.w[i].off == 0x000) start = i;
            if (start < 0) continue;

            MameDafb m;
            reset_dut(ta.name_sense);
            replay(ta, m, false, ta.name);
            dut->monitor_sense = tb.name_sense;

            for (int i = start; i < tb.n; i++) {
                axi_write(tb.w[i].off, tb.w[i].data);
                m.write(tb.w[i].off, tb.w[i].data);
                char buf[192];
                snprintf(buf, sizeof buf, "reprogram %s->%s w%d/+0x%03x=0x%x",
                         ta.name, tb.name, i, tb.w[i].off, tb.w[i].data);
                if ((int)dut->src_w_px > worst_w) {
                    worst_w = (int)dut->src_w_px;
                    worst_where = buf;
                }
                if (check_descriptor(m, buf) && !g_full) break;
            }
            // The destination must be fully in force once the block ends.
            if (tb.want_w_px) {
                check((int)dut->src_w_px == tb.want_w_px &&
                      (int)dut->src_h_px == tb.want_h_px,
                      "reprogram %s->%s settled at %ux%u, want %dx%d",
                      ta.name, tb.name, (unsigned)dut->src_w_px,
                      (unsigned)dut->src_h_px, tb.want_w_px, tb.want_h_px);
            }
        }
    }
    printf("  widest src_w_px seen anywhere in pass 5: %d  (%s)\n",
           worst_w, worst_where.c_str());

    printf("\n==== tb-mode-decode: pass=%d fail=%d ====\n", passes, failures);
    if (g_selftest && failures == 0) {
        printf("!! MD_SELFTEST=%d perturbed the model and NOTHING went red -- "
               "this table is vacuous\n", g_selftest);
        delete dut;
        return 1;
    }
    if (g_selftest) {
        printf("(negative control: %d failures as required)\n", failures);
        delete dut;
        return 0;
    }
    delete dut;
    return failures ? 1 : 0;
}
