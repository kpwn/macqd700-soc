// tb_dafb_scanout.cpp -- Live DAFB-state -> scaler address-generation tb.
//
// Writes representative Quadra 700 DAFB framebuffer registers into the
// shim, checks the exported live state, then drives a small scanout frame
// through the real line-buffer/fb_reader CDC bridge and verifies the source
// prefetch addresses track the programmed base/stride.  The wrapper also
// exposes a CPU-style byte write port into a tiny scanout memory, so this
// test proves programmed VRAM bytes can become visible through 8bpp output.

#include <cstdio>
#include <cstdint>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <verilated.h>
#include "Vtb_dafb_scanout.h"

static Vtb_dafb_scanout* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0;
static int n_fail = 0;

#define CHECK(cond, fmt, ...) do { \
    if (cond) { \
        n_pass++; \
        std::printf("  [PASS] " fmt "\n", ##__VA_ARGS__); \
    } else { \
        n_fail++; \
        std::printf("  [FAIL] " fmt "\n", ##__VA_ARGS__); \
    } \
} while (0)

static constexpr int SRC_W = 16;
static constexpr int SRC_H = 12;
static constexpr int DST_W = 40;
static constexpr int DST_H = 28;
static constexpr int ACTIVE_W = 32;
static constexpr int ACTIVE_H = 24;

// Golden sync-phase-lock offsets (in tick_scan_step() pclk-domain steps)
// -- see the sync phase-lock check in main() and run_frame_sync_phase()
// below.  run_frame_sync_phase() pulses hs_in/vs_in/de_in all starting on
// the very first scanned step (hs_in at every row's h==0, vs_in at the
// frame's v==0,h==0, de_in held high throughout), so with hs_out/vs_out/
// de_out all advancing through the identical 4-stage output pipeline
// (hs_pipe1..4 / vs_pipe1..4 / de_pipe1..4 in linebuf_scanout.v, one
// stage per clock) they reach the output side at the SAME step -- offset
// 0.  Captured by direct simulation against the current (T9) pipeline
// depth; a future retime that adds/removes a stage for only one of
// de_out/hs_out/vs_out (instead of all three together) will produce a
// nonzero offset here and fail loudly instead of only showing up as a
// visibly torn HDMI frame.
static constexpr int SYNC_HS_TO_DE_STEPS = 0;
static constexpr int SYNC_VS_TO_DE_STEPS = 0;

static void tick_pclk() {
    dut->pclk = 0;
    dut->eval();
    dut->pclk = 1;
    dut->eval();
    sim_time++;
}

static void tick_vram() {
    dut->vram_clk = 0;
    dut->eval();
    dut->vram_clk = 1;
    dut->eval();
    sim_time++;
}

static void tick_scan_step(int step) {
    int pre_ticks = 1;
    if ((step % 4) == 0) pre_ticks++;
    if ((step % 11) == 0) pre_ticks++;

    for (int i = 0; i < pre_ticks; i++)
        tick_vram();
    tick_pclk();
    if ((step % 7) == 0)
        tick_vram();
}

static void idle_inputs() {
    dut->s_axi_awaddr  = 0;
    dut->s_axi_awvalid = 0;
    dut->s_axi_wdata   = 0;
    dut->s_axi_wstrb   = 0;
    dut->s_axi_wvalid  = 0;
    dut->s_axi_bready  = 0;
    dut->s_axi_araddr  = 0;
    dut->s_axi_arvalid = 0;
    dut->s_axi_rready  = 0;
    dut->cpu_vram_wr_en = 0;
    dut->cpu_vram_wr_addr = 0;
    dut->cpu_vram_wr_data = 0;
    dut->vram_rsp_hold = 0;
    dut->hcount        = 0;
    dut->vcount        = 0;
    dut->de_in         = 0;
    dut->hs_in         = 0;
    dut->vs_in         = 0;
    dut->pclk         = 0;
    dut->vram_clk     = 0;
}

static void reset() {
    idle_inputs();
    dut->rst = 1;
    for (int i = 0; i < 4; i++) {
        tick_pclk();
        tick_vram();
    }
    dut->rst = 0;
    tick_pclk();
    tick_vram();
    dut->hcount = 1;
    dut->vcount = 0;
}

static int axil_write(uint32_t addr, uint32_t data, uint32_t strb = 0xF) {
    dut->s_axi_awaddr  = addr;
    dut->s_axi_awvalid = 1;
    dut->s_axi_wdata   = data;
    dut->s_axi_wstrb   = strb;
    dut->s_axi_wvalid  = 1;
    dut->s_axi_bready  = 1;

    bool aw_done = false, w_done = false;
    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (!aw_done && dut->s_axi_awready) aw_done = true;
        if (!w_done  && dut->s_axi_wready)  w_done  = true;
        tick_pclk();
        if (aw_done) dut->s_axi_awvalid = 0;
        if (w_done)  dut->s_axi_wvalid  = 0;
        if (aw_done && w_done) break;
    }
    if (!aw_done || !w_done) return 1;

    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (dut->s_axi_bvalid) {
            if (dut->s_axi_bresp != 0) return 2;
            tick_pclk();
            dut->s_axi_bready = 0;
            return 0;
        }
        tick_pclk();
    }
    return 3;
}

// T9: deterministic RGB formula for a FULL 8-bit CLUT index (0-255),
// programmed through the real AC842 RAMDAC tri-byte protocol (+0x200
// latch / +0x210 x3 R,G,B) rather than the retired 16-entry low-depth
// CLUT overlay that used to live at +0x300.  "inverse" keeps this tb's
// original naming/spirit (code = 255-idx) while covering the whole byte
// range that pattern_byte()/rgb_from_idx() actually exercise below --
// pattern_byte() is NOT restricted to a 4-bit range, so a 16-entry
// palette masked with idx&0x0F (the pre-T9 nibble-hack shape) silently
// discarded the upper nibble of every pixel it was asked to render.
static uint32_t clut_rgb_for_index(uint8_t idx) {
    uint8_t code = static_cast<uint8_t>(255 - idx);
    uint8_t r = code;
    uint8_t g = static_cast<uint8_t>(code ^ 0x55u);
    uint8_t b = static_cast<uint8_t>(code ^ 0xAAu);
    return (static_cast<uint32_t>(r) << 16)
         | (static_cast<uint32_t>(g) << 8)
         | static_cast<uint32_t>(b);
}

static int program_inverse_clut() {
    for (int i = 0; i < 256; i++) {
        uint32_t rgb = clut_rgb_for_index(static_cast<uint8_t>(i));
        int rc = axil_write(0x200, static_cast<uint32_t>(i));
        if (rc != 0) return rc;
        rc = axil_write(0x210, (rgb >> 16) & 0xFFu);
        if (rc != 0) return rc;
        rc = axil_write(0x210, (rgb >> 8) & 0xFFu);
        if (rc != 0) return rc;
        rc = axil_write(0x210, rgb & 0xFFu);
        if (rc != 0) return rc;
    }
    return 0;
}

static void cpu_vram_write_byte(uint32_t addr, uint8_t data) {
    dut->cpu_vram_wr_addr = addr;
    dut->cpu_vram_wr_data = data;
    dut->cpu_vram_wr_en = 1;
    tick_vram();
    dut->cpu_vram_wr_en = 0;
    dut->cpu_vram_wr_addr = 0;
    dut->cpu_vram_wr_data = 0;
    tick_vram();
}

static uint32_t pixel_addr(uint32_t fb_base_px,
                           uint32_t fb_stride_px,
                           int x,
                           int y) {
    return fb_base_px + (uint32_t)y * fb_stride_px + (uint32_t)x;
}

static uint32_t rgb_from_idx(uint8_t idx) {
    return clut_rgb_for_index(idx);
}

static uint8_t pattern_byte(int x, int y, uint8_t seed) {
    uint8_t v = static_cast<uint8_t>(seed
                                   + static_cast<uint8_t>(x * 13)
                                   + static_cast<uint8_t>(y * 37));
    return static_cast<uint8_t>(v | 0x01u);
}

static void write_pattern_framebuffer(uint32_t fb_base_px,
                                      uint32_t fb_stride_px,
                                      uint8_t seed) {
    const uint8_t guard = static_cast<uint8_t>(0xE0u | (seed & 0x0Fu));

    for (int y = 0; y < SRC_H; y++) {
        for (int x = 0; x < SRC_W; x++) {
            cpu_vram_write_byte(fb_base_px + (uint32_t)y * SRC_W + x, guard);
        }
    }

    for (int y = 0; y < SRC_H; y++) {
        for (int x = 0; x < SRC_W; x++) {
            cpu_vram_write_byte(pixel_addr(fb_base_px, fb_stride_px, x, y),
                                pattern_byte(x, y, seed));
        }

        cpu_vram_write_byte(pixel_addr(fb_base_px, fb_stride_px, SRC_W, y), guard);
        cpu_vram_write_byte(pixel_addr(fb_base_px, fb_stride_px, SRC_W + 1, y), guard);
    }
}

static uint32_t expected_addr(int read_idx,
                              uint32_t fb_base_px,
                              uint32_t fb_stride_px) {
    int src_y = read_idx / SRC_W;
    int src_x = read_idx % SRC_W;
    return fb_base_px + (src_y * fb_stride_px) + src_x;
}

struct FrameCapture {
    std::vector<uint32_t> addrs;
    std::vector<uint32_t> vram_addrs;
    std::vector<uint32_t> rgbs;
    std::vector<int> runs;
    int out_of_window_reads = 0;
    int out_of_range_reads = 0;
    int vram_reads = 0;
};

static FrameCapture run_frame(uint32_t fb_base_px,
                              uint32_t fb_stride_px,
                              int hold_start = -1,
                              int hold_len = 0) {
    FrameCapture cap;
    uint32_t limit = fb_base_px + ((SRC_H - 1) * fb_stride_px) + (SRC_W - 1);
    int run_len = 0;
    int pix_idx = 0;
    int step = 0;

    for (int v = 0; v < DST_H; v++) {
        for (int h = 0; h < DST_W; h++) {
            dut->vram_rsp_hold = (hold_start >= 0 &&
                                  pix_idx >= hold_start &&
                                  pix_idx < hold_start + hold_len) ? 1 : 0;
            dut->hcount = h;
            dut->vcount = v;
            dut->de_in = 1;
            dut->hs_in = 0;
            dut->vs_in = 0;
            tick_scan_step(step++);

            if (dut->fb_rd_en && dut->fb_rd_ready) {
                if (dut->fb_rd_addr > limit)
                    cap.out_of_range_reads++;
                cap.addrs.push_back(dut->fb_rd_addr);
                run_len++;
            } else if (run_len != 0) {
                cap.runs.push_back(run_len);
                run_len = 0;
            }

            if (dut->scanout_de)
                cap.rgbs.push_back(dut->scanout_rgb & 0xFFFFFFu);

            if (dut->vram_rd_en) {
                cap.vram_reads++;
                cap.vram_addrs.push_back(dut->vram_rd_addr);
            }
            pix_idx++;
        }
    }

    dut->vram_rsp_hold = 0;
    for (int flush = 0; flush < 64; flush++) {
        dut->de_in = 0;
        dut->hcount = 0;
        dut->vcount = 0;
        tick_scan_step(step++);
        if (dut->scanout_de)
            cap.rgbs.push_back(dut->scanout_rgb & 0xFFFFFFu);
        if (dut->vram_rd_en) {
            cap.vram_reads++;
            cap.vram_addrs.push_back(dut->vram_rd_addr);
        }
    }

    if (run_len != 0) cap.runs.push_back(run_len);
    return cap;
}

// ────────────────────────────────────────────────────────────────────
// Sync phase-lock check.  hs_out/vs_out advance through the exact same
// output pipeline depth as de_out/rgb (hs_pipe1..4 / vs_pipe1..4 /
// de_pipe1..4 in linebuf_scanout.v all update together, one stage per
// clock, terminating in hs_out/vs_out/de_out/rgb registered together).
// A real hsync/vsync pulse should therefore land on scanout_hs/
// scanout_vs at a FIXED step offset relative to when the first active
// pixel reaches scanout_de -- this drives a real (not tied-low) hs_in/
// vs_in stimulus and locks that offset in against golden constants, so
// a future retime that changes one signal's pipe depth without matching
// the others (easy mistake: bump de/rgb's stage count but forget hs/vs,
// or vice versa) trips this test immediately instead of silently
// desyncing sync from pixels only visible on real HDMI hardware.
// ────────────────────────────────────────────────────────────────────
struct SyncPhase {
    int de_first_step = -1;
    int hs_first_step = -1;
    int vs_first_step = -1;
};

static SyncPhase run_frame_sync_phase() {
    SyncPhase ph;
    int step = 0;
    for (int v = 0; v < DST_H; v++) {
        for (int h = 0; h < DST_W; h++) {
            dut->vram_rsp_hold = 0;
            dut->hcount = h;
            dut->vcount = v;
            dut->de_in  = 1;
            // Single-cycle hsync pulse at the start of every row, vsync
            // pulse at the start of the frame -- typical VTG convention
            // (matches vtg.v's own hcount==0/vcount==0 pulse shape).
            dut->hs_in  = (h == 0) ? 1 : 0;
            dut->vs_in  = (v == 0 && h == 0) ? 1 : 0;
            tick_scan_step(step);
            if (ph.de_first_step < 0 && dut->scanout_de) ph.de_first_step = step;
            if (ph.hs_first_step < 0 && dut->scanout_hs) ph.hs_first_step = step;
            if (ph.vs_first_step < 0 && dut->scanout_vs) ph.vs_first_step = step;
            step++;
        }
    }
    dut->hs_in = 0;
    dut->vs_in = 0;
    for (int flush = 0; flush < 64; flush++) {
        dut->de_in  = 0;
        dut->hcount = 0;
        dut->vcount = 0;
        tick_scan_step(step);
        if (ph.de_first_step < 0 && dut->scanout_de) ph.de_first_step = step;
        if (ph.hs_first_step < 0 && dut->scanout_hs) ph.hs_first_step = step;
        if (ph.vs_first_step < 0 && dut->scanout_vs) ph.vs_first_step = step;
        step++;
    }
    return ph;
}

static bool check_frame_geometry(const FrameCapture& cap,
                                 uint32_t fb_base_px,
                                 uint32_t fb_stride_px) {
    bool ok = true;
    const int expected_reads = SRC_W * SRC_H;
    if ((int)cap.addrs.size() != expected_reads) {
        std::printf("    read count got %zu expected %d\n",
                    cap.addrs.size(), expected_reads);
        ok = false;
    }
    if ((int)cap.runs.size() != 1) {
        std::printf("    prefetch run count got %zu expected 1\n",
                    cap.runs.size());
        ok = false;
    }
    for (size_t i = 0; i < cap.runs.size(); i++) {
        if (cap.runs[i] != expected_reads) {
            std::printf("    run[%zu] got %d expected %d\n",
                        i, cap.runs[i], expected_reads);
            ok = false;
            break;
        }
    }
    for (size_t i = 0; i < cap.addrs.size() && i < (size_t)expected_reads; i++) {
        uint32_t want = expected_addr((int)i, fb_base_px, fb_stride_px);
        if (cap.addrs[i] != want) {
            std::printf("    addr[%zu] got 0x%05x expected 0x%05x\n",
                        i, cap.addrs[i], want);
            ok = false;
            break;
        }
    }
    if (cap.out_of_range_reads != 0) {
        std::printf("    issued %d out-of-range reads\n",
                    cap.out_of_range_reads);
        ok = false;
    }
    return ok;
}

static uint32_t expected_frame_rgb(int out_x, int out_y, uint8_t seed) {
    bool in_x = (out_x >= 4) && (out_x < 36);
    bool in_y = (out_y >= 2) && (out_y < 26);
    if (!in_x || !in_y)
        return 0;

    int src_x = (out_x - 4) / 2;
    int src_y = (out_y - 2) / 2;
    return rgb_from_idx(pattern_byte(src_x, src_y, seed));
}

static void mkdirp(const std::string& path) {
    if (!path.empty())
        ::mkdir(path.c_str(), 0755);
}

static void write_ppm(const std::string& path,
                      const FrameCapture& cap,
                      int w,
                      int h) {
    FILE* fp = std::fopen(path.c_str(), "wb");
    if (!fp) {
        std::printf("  [WARN] could not write %s (%s)\n",
                    path.c_str(), std::strerror(errno));
        return;
    }

    std::fprintf(fp, "P6\n%d %d\n255\n", w, h);
    const int expected = w * h;
    for (int i = 0; i < expected; i++) {
        uint32_t rgb = (i < (int)cap.rgbs.size()) ? cap.rgbs[i] : 0;
        std::fputc((rgb >> 16) & 0xff, fp);
        std::fputc((rgb >> 8) & 0xff, fp);
        std::fputc(rgb & 0xff, fp);
    }
    std::fclose(fp);
    std::printf("  [INFO] DAFB scanout frame dumped to %s\n", path.c_str());
}

static void maybe_dump_scanout_frame(const char* suffix,
                                     const FrameCapture& cap) {
    const char* single_path = std::getenv("DAFB_SCANOUT_PPM");
    const char* dir_path = std::getenv("DAFB_SCANOUT_PPM_DIR");
    if (single_path && single_path[0] && std::strcmp(suffix, "frame0") == 0) {
        write_ppm(single_path, cap, DST_W, DST_H);
        return;
    }
    if (dir_path && dir_path[0]) {
        mkdirp(dir_path);
        write_ppm(std::string(dir_path) + "/" + suffix + ".ppm", cap, DST_W, DST_H);
    }
}

static bool check_pattern_exact(const FrameCapture& cap,
                                const char* label,
                                uint8_t seed) {
    bool ok = true;
    const int expected_pixels = DST_W * DST_H;
    if ((int)cap.rgbs.size() != expected_pixels) {
        std::printf("    %s: output pixel count got %zu expected %d\n",
                    label, cap.rgbs.size(), expected_pixels);
        ok = false;
    }

    int n = ((int)cap.rgbs.size() < expected_pixels)
        ? (int)cap.rgbs.size() : expected_pixels;
    for (int i = 0; i < n; i++) {
        int out_x = i % DST_W;
        int out_y = i / DST_W;
        uint32_t want = expected_frame_rgb(out_x, out_y, seed);
        if (cap.rgbs[i] != want) {
            std::printf("    %s: rgb[%d,%d] got 0x%06x expected 0x%06x\n",
                        label, out_x, out_y, cap.rgbs[i], want);
            ok = false;
            break;
        }
    }
    return ok;
}

static bool check_no_stale_pixels(const FrameCapture& cap,
                                  const char* label,
                                  uint8_t seed) {
    bool ok = true;
    const int expected_pixels = DST_W * DST_H;
    if ((int)cap.rgbs.size() != expected_pixels) {
        std::printf("    %s: output pixel count got %zu expected %d\n",
                    label, cap.rgbs.size(), expected_pixels);
        ok = false;
    }

    int n = ((int)cap.rgbs.size() < expected_pixels)
        ? (int)cap.rgbs.size() : expected_pixels;

    for (int i = 0; i < n; i++) {
        int out_x = i % DST_W;
        int out_y = i / DST_W;
        uint32_t want = expected_frame_rgb(out_x, out_y, seed);
        uint32_t got = cap.rgbs[i];

        if (want == 0) {
            if (got != 0) {
                std::printf("    %s: border rgb[%d,%d] got stale 0x%06x\n",
                            label, out_x, out_y, got);
                ok = false;
                break;
            }
        } else if (got != want && got != 0) {
            std::printf("    %s: rgb[%d,%d] got stale 0x%06x expected 0x%06x or blank\n",
                        label, out_x, out_y, got, want);
            ok = false;
            break;
        }
    }

    return ok;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_dafb_scanout;

    reset();

    CHECK(dut->fb_base_px == 0 && dut->fb_stride_px == 0 && dut->fb_bpp_reg == 0,
          "live DAFB outputs reset to zero");

    // MAME-canonical DAFB writes: base = (BASE_HI<<9) | (BASE_LO<<5),
    // stride = STRIDE<<2, bpp derived from AC842 PCBR @ +0x220.
    // Choose BASE_HI=0, BASE_LO=8 → m_base = 0x100; STRIDE=0x80 → m_stride
    // = 0x200; PCBR=0x18 (bits[4:2]=110) → 8bpp.
    CHECK(axil_write(0x00, 0x00000000) == 0, "write BASE_HI +0x00 <- 0");
    CHECK(axil_write(0x04, 0x00000008) == 0, "write BASE_LO +0x04 <- 0x8");
    CHECK(axil_write(0x08, 0x00000080) == 0, "write STRIDE +0x08 <- 0x80");
    CHECK(axil_write(0x220, 0x00000018) == 0, "write PCBR +0x220 <- 0x18 (8bpp)");
    CHECK(dut->fb_base_px == 0x00000100u, "live FB base output updates to 0x%08x",
          dut->fb_base_px);
    CHECK(dut->fb_stride_px == 0x00000200u, "live FB stride output updates to 0x%08x",
          dut->fb_stride_px);
    CHECK(dut->fb_bpp_reg == 0x00000008u, "live FB BPP output updates to 0x%08x",
          dut->fb_bpp_reg);
    CHECK(program_inverse_clut() == 0, "program inverse DAFB CLUT for scanout");

    const uint8_t seed0 = 0x31;
    write_pattern_framebuffer(dut->fb_base_px, dut->fb_stride_px, seed0);

    FrameCapture frame0 = run_frame(dut->fb_base_px, dut->fb_stride_px);
    maybe_dump_scanout_frame("frame0", frame0);
    CHECK(check_frame_geometry(frame0, dut->fb_base_px, dut->fb_stride_px),
          "DAFB-programmed base/stride steer scanout addresses");
    CHECK(check_pattern_exact(frame0, "rom-stride-pattern", seed0),
          "deterministic strided framebuffer pattern reaches scanout exactly");
    CHECK(dut->fb_underflow_sticky == 0,
          "fb_reader underflow stays clear during normal DAFB scanout");

    const int first_active = 2 * DST_W + 4;
    FrameCapture held = run_frame(dut->fb_base_px, dut->fb_stride_px,
                                  first_active, 48);
    CHECK(check_no_stale_pixels(held, "cdc-held-response", seed0),
          "delayed VRAM responses blank instead of shifting stale pixels");
    CHECK(dut->fb_underflow_sticky == 0,
          "held VRAM responses do not create false request underflow");

    // Sync phase-lock: drive real hs_in/vs_in pulses (tied low everywhere
    // else in this tb) and confirm scanout_hs/scanout_vs land at the
    // captured golden offset relative to the first scanout_de pixel.
    SyncPhase phase0 = run_frame_sync_phase();
    CHECK(phase0.de_first_step >= 0 && phase0.hs_first_step >= 0 &&
          phase0.vs_first_step >= 0,
          "sync phase-lock: de/hs/vs all observed within the captured frame "
          "(de=%d hs=%d vs=%d)",
          phase0.de_first_step, phase0.hs_first_step, phase0.vs_first_step);
    if (phase0.de_first_step >= 0 && phase0.hs_first_step >= 0 &&
        phase0.vs_first_step >= 0) {
        int hs_to_de = phase0.de_first_step - phase0.hs_first_step;
        int vs_to_de = phase0.de_first_step - phase0.vs_first_step;
        CHECK(hs_to_de == SYNC_HS_TO_DE_STEPS,
              "sync phase-lock: hs->first-de offset is %d steps (golden %d)",
              hs_to_de, SYNC_HS_TO_DE_STEPS);
        CHECK(vs_to_de == SYNC_VS_TO_DE_STEPS,
              "sync phase-lock: vs->first-de offset is %d steps (golden %d)",
              vs_to_de, SYNC_VS_TO_DE_STEPS);
    }

    reset();
    // BASE_HI=1 → m_base = 0x200 (bit 9); BASE_LO unused; STRIDE=0x100
    // → m_stride = 0x400; PCBR=0x10 (bits[4:2]=100) → 4bpp.
    CHECK(axil_write(0x00, 0x00000001) == 0, "rewrite BASE_HI +0x00 <- 1");
    CHECK(axil_write(0x04, 0x00000000) == 0, "rewrite BASE_LO +0x04 <- 0");
    CHECK(axil_write(0x08, 0x00000100) == 0, "rewrite STRIDE +0x08 <- 0x100");
    CHECK(axil_write(0x220, 0x00000010) == 0, "rewrite PCBR +0x220 <- 0x10 (4bpp)");
    CHECK(dut->fb_base_px == 0x00000200u,
          "live FB base updates on second write: expected 0x200 got 0x%08x",
          dut->fb_base_px);
    CHECK(dut->fb_stride_px == 0x00000400u, "live FB stride updates on second write");
    CHECK(dut->fb_bpp_reg == 0x00000004u, "live FB BPP updates on second write");
    CHECK(program_inverse_clut() == 0, "reprogram inverse DAFB CLUT after reset");

    const uint8_t seed1 = 0x55;
    write_pattern_framebuffer(dut->fb_base_px, dut->fb_stride_px, seed1);
    FrameCapture frame1 = run_frame(dut->fb_base_px, dut->fb_stride_px);
    maybe_dump_scanout_frame("frame1", frame1);
    CHECK(check_frame_geometry(frame1, dut->fb_base_px, dut->fb_stride_px),
          "updated DAFB placement changes scanout addresses");
    CHECK(check_pattern_exact(frame1, "rewritten-dafb-pattern", seed1),
          "rewritten DAFB placement selects the new CPU-written framebuffer");
    CHECK(dut->fb_underflow_sticky == 0,
          "fb_reader underflow clears after reset and stays clear after rewrite");

    std::printf("--------------------------------\n");
    std::printf("  pass=%d fail=%d cycles=%llu\n",
                n_pass, n_fail, (unsigned long long)sim_time);

    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
