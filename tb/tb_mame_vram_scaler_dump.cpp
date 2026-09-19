// tb_mame_vram_scaler_dump.cpp -- render a MAME VRAM dump through scanout RTL.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <verilated.h>
#include "Vtb_mame_vram_scaler_dump.h"

static constexpr int W = 640;
static constexpr int H = 480;

static Vtb_mame_vram_scaler_dump* dut = nullptr;
static uint64_t sim_time = 0;
static std::vector<uint8_t> vram;
static uint32_t vram_base = 0;
static uint32_t vram_stride = 1024;
static int vram_bpp = 1;
// rtl_decode=true: harness serves RAW VRAM bytes (no SW unpacking);
// the RTL's BPP-aware scanner does the bit/nibble extraction.  This is
// the path that validates the new `bpp_shift` plumbing end-to-end.
static bool rtl_decode = false;
static bool rsp_valid = false;
// 4-byte response group: byte at the requested address in [31:24], the next
// three bytes in [23:16]/[15:8]/[7:0].  See tb_mame_vram_scaler_dump.v's
// v_rd_data comment for the contract this models.
static uint32_t rsp_data = 0;

static bool parse_plusarg(int argc, char** argv, const char* key, std::string& out) {
    const std::string prefix = std::string("+") + key + "=";
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a.rfind(prefix, 0) == 0) {
            out = a.substr(prefix.size());
            return true;
        }
    }
    return false;
}

static uint32_t parse_u32(const std::string& s, uint32_t fallback) {
    char* end = nullptr;
    unsigned long v = std::strtoul(s.c_str(), &end, 0);
    return (end && *end == '\0') ? static_cast<uint32_t>(v) : fallback;
}

static uint8_t read_raw_byte(uint32_t off) {
    return off < vram.size() ? vram[off] : 0;
}

static uint8_t pixel_at(uint32_t addr) {
    const uint32_t x = addr % W;
    const uint32_t y = addr / W;
    if (y >= H)
        return 0;

    switch (vram_bpp) {
    case 1: {
        const uint8_t b = read_raw_byte(vram_base + y * vram_stride + (x >> 3));
        return static_cast<uint8_t>((b >> (7 - (x & 7))) & 0x1);
    }
    case 2: {
        const uint8_t b = read_raw_byte(vram_base + y * vram_stride + (x >> 2));
        return static_cast<uint8_t>((b >> (6 - 2 * (x & 3))) & 0x3);
    }
    case 4: {
        const uint8_t b = read_raw_byte(vram_base + y * vram_stride + (x >> 1));
        return static_cast<uint8_t>((x & 1) ? (b & 0x0f) : (b >> 4));
    }
    case 8:
        return read_raw_byte(vram_base + y * vram_stride + x);
    default:
        return 0;
    }
}

static void tick_pclk() {
    dut->pclk = 0;
    dut->eval();
    dut->pclk = 1;
    dut->eval();
    sim_time++;
}

static void tick_vram() {
    dut->v_rd_valid = rsp_valid ? 1 : 0;
    dut->v_rd_data = rsp_data;
    dut->vram_clk = 0;
    dut->eval();
    dut->vram_clk = 1;
    dut->eval();

    const bool req = dut->v_rd_en != 0;
    const uint32_t addr = dut->v_rd_addr;
    rsp_valid = req;
    if (req) {
        // rtl_decode mode: RTL's BPP-aware scanner expects raw VRAM
        // bytes at byte-address `addr` (already includes base+stride
        // baked into fb_rd_addr by linebuf_scanout's BPP-aware shift).
        // Legacy SW mode: pixel_at unpacks bits/nibbles from VRAM and
        // returns one CLUT-index byte per output pixel.
        //
        // The port returns a 4-byte GROUP.  Model all four bytes (the true
        // bytes at +0..+3, a strict superset of the contract, which only
        // guarantees the lower three for a 4-byte-aligned request); this tb
        // runs bytes_per_px=1 indexed, so the scanner consumes [31:24] only.
        rsp_data = 0;
        for (uint32_t k = 0; k < 4; k++) {
            const uint32_t b = rtl_decode ? read_raw_byte(addr + k)
                                          : pixel_at(addr + k);
            rsp_data |= b << (24 - 8 * k);
        }
    } else {
        rsp_data = 0;
    }
    sim_time++;
}

static void tick_both(int n) {
    for (int i = 0; i < n; i++) {
        tick_vram();
        tick_pclk();
    }
}

static void reset_dut() {
    rsp_valid = false;
    rsp_data = 0;
    dut->rst = 1;
    dut->pclk = 0;
    dut->vram_clk = 0;
    dut->hcount = 1;
    dut->vcount = 0;
    dut->de_in = 0;
    dut->hs_in = 0;
    dut->vs_in = 1;
    dut->v_rd_data = 0;
    dut->v_rd_valid = 0;
    // Plumb DAFB-style scanout config to the RTL.  In legacy SW mode the
    // RTL operates 8bpp on pre-unpacked bytes (shift=0, base=0,
    // stride=W); in rtl_decode mode the RTL needs the real Mac config so
    // its bpp_shift drives correct line_rd_addr shifting.
    if (rtl_decode) {
        dut->bpp_shift    = (vram_bpp == 1) ? 3
                          : (vram_bpp == 2) ? 2
                          : (vram_bpp == 4) ? 1
                          :                   0;
        dut->fb_base_px   = vram_base & 0xFFFFFu;
        dut->fb_stride_px = vram_stride & 0xFFFFFu;
    } else {
        dut->bpp_shift    = 0;
        dut->fb_base_px   = 0;
        dut->fb_stride_px = static_cast<uint32_t>(W);
    }
    tick_both(8);
    dut->rst = 0;
    tick_both(4);
}

static void dump_ppm(const std::string& path, const std::vector<uint32_t>& rgb) {
    std::filesystem::path p(path);
    if (!p.parent_path().empty())
        std::filesystem::create_directories(p.parent_path());

    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
        std::perror(path.c_str());
        return;
    }
    std::fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (uint32_t px : rgb) {
        const uint8_t bytes[3] = {
            static_cast<uint8_t>((px >> 16) & 0xff),
            static_cast<uint8_t>((px >> 8) & 0xff),
            static_cast<uint8_t>(px & 0xff),
        };
        std::fwrite(bytes, 1, 3, f);
    }
    std::fclose(f);
}

static bool load_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::fprintf(stderr, "cannot open VRAM dump: %s\n", path.c_str());
        return false;
    }
    vram.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
}

static void scanout_frame(std::vector<uint32_t>& rgb) {
    rgb.assign(W * H, 0xffffff);

    dut->vs_in = 1;
    dut->de_in = 0;
    dut->hcount = 0;
    dut->vcount = 0;
    tick_both(4);
    dut->vs_in = 0;

    // Frame-start kicks the line prefetcher.  Give it enough blanking-like
    // cycles to fetch the first source line before active pixels begin.
    dut->hcount = 0;
    dut->vcount = 0;
    dut->de_in = 0;
    tick_pclk();
    for (int i = 0; i < 1600; i++) {
        tick_vram();
        tick_pclk();
    }

    int pix_idx = 0;
    bool first_pulse_seen = false;
    for (int y = 0; y < H; y++) {
        dut->vcount = static_cast<uint16_t>(y);
        for (int x = 0; x < W; x++) {
            dut->hcount = static_cast<uint16_t>(x);
            dut->de_in = 1;
            tick_vram();
            tick_vram();
            tick_pclk();
            if (dut->scanout_de) {
                if (!first_pulse_seen) {
                    first_pulse_seen = true;
                } else {
                    const int cx = pix_idx % W;
                    const int cy = pix_idx / W;
                    if (cy < H)
                        rgb[cy * W + cx] = dut->scanout_rgb & 0xffffffu;
                    pix_idx++;
                }
            }
        }
    }

    for (int i = 0; i < 2; i++) {
        dut->hcount = W - 1;
        dut->vcount = H - 1;
        dut->de_in = 1;
        tick_vram();
        tick_vram();
        tick_pclk();
        if (dut->scanout_de) {
            if (!first_pulse_seen) {
                first_pulse_seen = true;
            } else {
                const int cx = pix_idx % W;
                const int cy = pix_idx / W;
                if (cy < H)
                    rgb[cy * W + cx] = dut->scanout_rgb & 0xffffffu;
                pix_idx++;
            }
        }
    }

    dut->de_in = 0;
    for (int i = 0; i < 16; i++) {
        dut->hcount = W + i;
        tick_vram();
        tick_vram();
        tick_pclk();
        if (dut->scanout_de) {
            if (!first_pulse_seen) {
                first_pulse_seen = true;
            } else {
                const int cx = pix_idx % W;
                const int cy = pix_idx / W;
                if (cy < H)
                    rgb[cy * W + cx] = dut->scanout_rgb & 0xffffffu;
                pix_idx++;
            }
        }
    }
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

    std::string in_path;
    std::string out_path = "build/mame_runs/mame_vram_scaler.ppm";
    std::string value;
    parse_plusarg(argc, argv, "ppm", out_path);
    if (parse_plusarg(argc, argv, "vram_base", value))
        vram_base = parse_u32(value, vram_base);
    if (parse_plusarg(argc, argv, "vram_stride", value))
        vram_stride = parse_u32(value, vram_stride);
    if (parse_plusarg(argc, argv, "vram_bpp", value))
        vram_bpp = static_cast<int>(parse_u32(value, vram_bpp));
    if (parse_plusarg(argc, argv, "rtl_decode", value))
        rtl_decode = (parse_u32(value, 0) != 0);

    if (!parse_plusarg(argc, argv, "vram_image", in_path)) {
        std::fprintf(stderr,
                     "usage: %s +vram_image=PATH [+ppm=PATH] "
                     "[+vram_bpp=1|2|4|8] [+vram_stride=N] [+vram_base=N]\n",
                     argv[0]);
        return 2;
    }
    if (vram_bpp != 1 && vram_bpp != 2 && vram_bpp != 4 && vram_bpp != 8) {
        std::fprintf(stderr, "unsupported +vram_bpp=%d\n", vram_bpp);
        return 2;
    }
    if (!load_file(in_path))
        return 1;

    dut = new Vtb_mame_vram_scaler_dump;
    reset_dut();

    std::vector<uint32_t> rgb;
    scanout_frame(rgb);
    dump_ppm(out_path, rgb);

    uint64_t white = 0;
    uint64_t black = 0;
    uint64_t other = 0;
    for (uint32_t px : rgb) {
        if ((px & 0xffffffu) == 0xffffffu)
            white++;
        else if ((px & 0xffffffu) == 0)
            black++;
        else
            other++;
    }

    std::printf("mame-vram-scaler-dump: %s -> %s\n",
                in_path.c_str(), out_path.c_str());
    std::printf("  geometry=%dx%d bpp=%d raw_base=0x%x raw_stride=%u "
                "white=%llu black=%llu other=%llu underflow=%u\n",
                W, H, vram_bpp, vram_base, vram_stride,
                static_cast<unsigned long long>(white),
                static_cast<unsigned long long>(black),
                static_cast<unsigned long long>(other),
                static_cast<unsigned>(dut->fb_underflow_sticky));

    const bool underflow = dut->fb_underflow_sticky != 0;
    dut->final();
    delete dut;
    return underflow ? 1 : 0;
}
