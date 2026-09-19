// tb_vram_scaler_firstlight.cpp -- checkerboard/stride/CDC first-light test.

#include <cstdint>
#include <cstdio>
#include <deque>
#include <vector>
#include <verilated.h>
#include "Vtb_vram_scaler_firstlight.h"

static Vtb_vram_scaler_firstlight* dut = nullptr;
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
static constexpr int BORDER_X = 4;
static constexpr int BORDER_Y = 2;
static constexpr uint32_t DEFAULT_BASE = 0;
static constexpr uint32_t DEFAULT_STRIDE = SRC_W;

struct FrameCapture {
    std::vector<uint32_t> fb_addrs;
    std::vector<uint32_t> vram_addrs;
    std::vector<uint32_t> rgbs;
    std::vector<int> runs;
    int out_of_window_reads = 0;
    int out_of_range_reads = 0;
};

static FrameCapture* active_capture = nullptr;
static bool capture_prefetch_filter = false;
static bool capture_prefetch_vram_started = false;
static uint32_t capture_prefetch_base = 0;
static int capture_prefetch_expected = 0;

struct VramActivity {
    bool valid = false;
    uint64_t cycle = 0;
    uint32_t addr = 0;
    uint8_t data = 0;
};

static VramActivity first_vram_write;
static VramActivity first_vram_scanout_read;

struct PendingByteWrite {
    uint32_t pixel_addr;
    uint8_t data;
};

class BackgroundVramWriter {
public:
    void enqueue(uint32_t pixel_addr, uint8_t data) {
        writes_.push_back({pixel_addr, data});
    }

    bool done() const {
        return writes_.empty() && state_ == State::Idle;
    }

    int completed() const {
        return completed_;
    }

    int failed() const {
        return failed_;
    }

    void drive(Vtb_vram_scaler_firstlight* d) {
        if (state_ == State::Idle && !writes_.empty()) {
            const PendingByteWrite& wr = writes_.front();
            uint32_t lane = wr.pixel_addr & 0xFu;
            VlWide<4> wdata;
            for (int i = 0; i < 4; i++) wdata[i] = 0;
            wdata[lane / 4] = uint32_t(wr.data) << ((lane % 4) * 8);

            d->vram_awid = 3;
            d->vram_awaddr = wr.pixel_addr & ~0xFu;
            d->vram_awlen = 0;
            d->vram_awsize = 4;
            d->vram_awburst = 1;
            d->vram_awvalid = 1;
            d->vram_wdata = wdata;
            d->vram_wstrb = uint16_t(1u << lane);
            d->vram_wlast = 1;
            d->vram_wvalid = 1;
            d->vram_bready = 1;
            aw_done_ = false;
            w_done_ = false;
            state_ = State::AddrData;
        }
    }

    void observe_before_edge(Vtb_vram_scaler_firstlight* d) {
        aw_fire_ = (state_ == State::AddrData) && d->vram_awvalid && d->vram_awready;
        w_fire_ = (state_ == State::AddrData) && d->vram_wvalid && d->vram_wready;
        b_fire_ = (state_ == State::Resp) && d->vram_bvalid && d->vram_bready;
        if (b_fire_ && d->vram_bresp != 0)
            failed_++;
    }

    void update_after_edge(Vtb_vram_scaler_firstlight* d) {
        if (aw_fire_) {
            aw_done_ = true;
            d->vram_awvalid = 0;
        }
        if (w_fire_) {
            w_done_ = true;
            d->vram_wvalid = 0;
        }
        if (state_ == State::AddrData && aw_done_ && w_done_) {
            state_ = State::Resp;
        }
        if (b_fire_) {
            const PendingByteWrite& wr = writes_.front();
            if (!first_vram_write.valid) {
                first_vram_write.valid = true;
                first_vram_write.cycle = sim_time;
                first_vram_write.addr = wr.pixel_addr;
                first_vram_write.data = wr.data;
            }
            d->vram_bready = 0;
            d->vram_wlast = 0;
            writes_.pop_front();
            completed_++;
            state_ = State::Idle;
        }
        aw_fire_ = false;
        w_fire_ = false;
        b_fire_ = false;
    }

private:
    enum class State {
        Idle,
        AddrData,
        Resp,
    };

    std::deque<PendingByteWrite> writes_;
    State state_ = State::Idle;
    bool aw_done_ = false;
    bool w_done_ = false;
    bool aw_fire_ = false;
    bool w_fire_ = false;
    bool b_fire_ = false;
    int completed_ = 0;
    int failed_ = 0;
};

static BackgroundVramWriter* active_writer = nullptr;

static void sample_vram_edge() {
    if (dut->vram_rd_en) {
        if (!first_vram_scanout_read.valid) {
            first_vram_scanout_read.valid = true;
            first_vram_scanout_read.cycle = sim_time;
            first_vram_scanout_read.addr = dut->vram_rd_addr;
            first_vram_scanout_read.data = 0;
        }
    }
    if (active_capture && dut->vram_rd_en) {
        if (capture_prefetch_filter) {
            if (!capture_prefetch_vram_started &&
                dut->vram_rd_addr == capture_prefetch_base) {
                capture_prefetch_vram_started = true;
            }
            if (capture_prefetch_vram_started &&
                int(active_capture->vram_addrs.size()) < capture_prefetch_expected) {
                active_capture->vram_addrs.push_back(dut->vram_rd_addr);
            }
        } else {
            active_capture->vram_addrs.push_back(dut->vram_rd_addr);
        }
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
    if (active_writer)
        active_writer->drive(dut);
    dut->vram_clk = 0;
    dut->eval();
    if (active_writer)
        active_writer->observe_before_edge(dut);
    dut->vram_clk = 1;
    dut->eval();
    sample_vram_edge();
    if (active_writer)
        active_writer->update_after_edge(dut);
    sim_time++;
}

static void tick_scan_step(int step) {
    int pre_ticks = 1;
    if ((step % 4) == 0) pre_ticks++;
    if ((step % 11) == 0) pre_ticks++;

    for (int i = 0; i < pre_ticks; i++) tick_vram();
    tick_pclk();
    if ((step % 7) == 0) tick_vram();
}

static void tick_scan_step_slow_vram(int step) {
    tick_pclk();
    if ((step % 3) != 2)
        tick_vram();
}

static void tick_scan_step_mode(int step, bool slow_vram) {
    if (slow_vram)
        tick_scan_step_slow_vram(step);
    else
        tick_scan_step(step);
}

static void idle_inputs() {
    dut->pclk = 0;
    dut->vram_clk = 0;

    dut->dafb_awaddr = 0;
    dut->dafb_awvalid = 0;
    dut->dafb_wdata = 0;
    dut->dafb_wstrb = 0;
    dut->dafb_wvalid = 0;
    dut->dafb_bready = 0;
    dut->dafb_araddr = 0;
    dut->dafb_arvalid = 0;
    dut->dafb_rready = 0;

    dut->vram_awid = 0;
    dut->vram_awaddr = 0;
    dut->vram_awlen = 0;
    dut->vram_awsize = 0;
    dut->vram_awburst = 0;
    dut->vram_awvalid = 0;
    for (int i = 0; i < 4; i++) dut->vram_wdata[i] = 0;
    dut->vram_wstrb = 0;
    dut->vram_wlast = 0;
    dut->vram_wvalid = 0;
    dut->vram_bready = 0;
    dut->vram_arid = 0;
    dut->vram_araddr = 0;
    dut->vram_arlen = 0;
    dut->vram_arsize = 0;
    dut->vram_arburst = 0;
    dut->vram_arvalid = 0;
    dut->vram_rready = 1;

    dut->hcount = 0;
    dut->vcount = 0;
    dut->de_in = 0;
    dut->hs_in = 0;
    dut->vs_in = 0;
}

static void reset_dut() {
    active_capture = nullptr;
    capture_prefetch_filter = false;
    capture_prefetch_vram_started = false;
    first_vram_write = VramActivity{};
    first_vram_scanout_read = VramActivity{};
    idle_inputs();
    dut->rst = 1;
    // Hold hcount off-origin during reset+deassert so the frame_start
    // pulse does not auto-trigger start_pending before the test has
    // programmed the DAFB base/stride.  With the "auto-kick on first
    // sof" fix in linebuf_scanout, any hc=0,vc=0 pclk after rst=0 would
    // otherwise launch a prefetch with default placement values.
    dut->hcount = 1;
    dut->vcount = 0;
    for (int i = 0; i < 8; i++) {
        tick_pclk();
        tick_vram();
    }
    dut->rst = 0;
    for (int i = 0; i < 4; i++) {
        tick_pclk();
        tick_vram();
    }
    dut->hcount = 1;
    dut->vcount = 0;
}

static int dafb_write(uint32_t addr, uint32_t data) {
    dut->dafb_awaddr = addr;
    dut->dafb_awvalid = 1;
    dut->dafb_wdata = data;
    dut->dafb_wstrb = 0xF;
    dut->dafb_wvalid = 1;
    dut->dafb_bready = 1;

    bool aw_done = false;
    bool w_done = false;
    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (!aw_done && dut->dafb_awready) aw_done = true;
        if (!w_done && dut->dafb_wready) w_done = true;
        tick_vram();
        if (aw_done) dut->dafb_awvalid = 0;
        if (w_done) dut->dafb_wvalid = 0;
        if (aw_done && w_done) break;
    }
    if (!aw_done || !w_done) return 1;

    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (dut->dafb_bvalid) {
            int resp = dut->dafb_bresp;
            tick_vram();
            dut->dafb_bready = 0;
            return resp == 0 ? 0 : 2;
        }
        tick_vram();
    }
    return 3;
}

static int vram_write_word(uint32_t byte_addr, uint32_t lane,
                           uint8_t data, uint16_t wstrb) {
    uint32_t word_addr = byte_addr & ~0xFu;
    VlWide<4> wdata;
    for (int i = 0; i < 4; i++) wdata[i] = 0;
    wdata[lane / 4] = uint32_t(data) << ((lane % 4) * 8);

    dut->vram_awid = 3;
    dut->vram_awaddr = word_addr;
    dut->vram_awlen = 0;
    dut->vram_awsize = 4;
    dut->vram_awburst = 1;
    dut->vram_awvalid = 1;
    dut->vram_wdata = wdata;
    dut->vram_wstrb = wstrb;
    dut->vram_wlast = 1;
    dut->vram_wvalid = 1;
    dut->vram_bready = 1;

    bool aw_done = false;
    bool w_done = false;
    for (int i = 0; i < 128; i++) {
        dut->eval();
        if (!aw_done && dut->vram_awready) aw_done = true;
        if (!w_done && dut->vram_wready) w_done = true;
        tick_vram();
        if (aw_done) dut->vram_awvalid = 0;
        if (w_done) dut->vram_wvalid = 0;
        if (aw_done && w_done) break;
    }
    if (!aw_done || !w_done) return 1;

    for (int i = 0; i < 128; i++) {
        dut->eval();
        if (dut->vram_bvalid) {
            int resp = dut->vram_bresp;
            tick_vram();
            dut->vram_bready = 0;
            dut->vram_wlast = 0;
            if (resp == 0 && !first_vram_write.valid) {
                first_vram_write.valid = true;
                first_vram_write.cycle = sim_time;
                first_vram_write.addr = byte_addr;
                first_vram_write.data = data;
            }
            return resp == 0 ? 0 : 2;
        }
        tick_vram();
    }
    return 3;
}

static int vram_write_byte(uint32_t pixel_addr, uint8_t data) {
    uint32_t lane = pixel_addr & 0xFu;
    return vram_write_word(pixel_addr, lane, data, uint16_t(1u << lane));
}

static uint32_t pixel_addr(uint32_t fb_base_px,
                           uint32_t fb_stride_px,
                           int x,
                           int y) {
    return fb_base_px + uint32_t(y) * fb_stride_px + uint32_t(x);
}

static uint8_t checker_byte(int x, int y, uint8_t seed) {
    uint8_t a = static_cast<uint8_t>(0x21u + (seed & 0x0Fu));
    uint8_t b = static_cast<uint8_t>(0xD0u - (seed & 0x0Fu));
    uint8_t v = (((x >> 1) ^ y) & 1) ? b : a;
    return static_cast<uint8_t>(v | 0x01u);
}

// T9: deterministic RGB formula for a FULL 8-bit CLUT index (0-255),
// programmed through the real AC842 RAMDAC tri-byte protocol (+0x200
// latch / +0x210 x3 R,G,B) instead of relying on the retired 16-entry
// low-depth CLUT's hardcoded reset palette.  checker_byte() below is NOT
// restricted to a 4-bit range (a/b are derived from 0x21+seed and
// 0xD0-seed), so an idx&0x0F mask silently discarded the upper nibble of
// every checkerboard pixel under the old scheme -- program_clut() below
// populates all 256 entries so the full byte range renders correctly.
static uint32_t palette_rgb(uint8_t idx) {
    uint8_t code = static_cast<uint8_t>(255 - idx);
    uint8_t r = code;
    uint8_t g = static_cast<uint8_t>(code ^ 0x55u);
    uint8_t b = static_cast<uint8_t>(code ^ 0xAAu);
    return (static_cast<uint32_t>(r) << 16)
         | (static_cast<uint32_t>(g) << 8)
         | static_cast<uint32_t>(b);
}

// Programs all 256 CLUT entries via the real AC842 RAMDAC write protocol
// so palette_rgb()'s expectations above match the live scanout CLUT.
static int program_clut() {
    for (int i = 0; i < 256; i++) {
        uint32_t rgb = palette_rgb(static_cast<uint8_t>(i));
        int rc = dafb_write(0x200, static_cast<uint32_t>(i));
        if (rc != 0) return rc;
        rc = dafb_write(0x210, (rgb >> 16) & 0xFFu);
        if (rc != 0) return rc;
        rc = dafb_write(0x210, (rgb >> 8) & 0xFFu);
        if (rc != 0) return rc;
        rc = dafb_write(0x210, rgb & 0xFFu);
        if (rc != 0) return rc;
    }
    return 0;
}

static int write_checkerboard(uint32_t fb_base_px,
                              uint32_t fb_stride_px,
                              uint8_t seed) {
    uint8_t guard = static_cast<uint8_t>(0x70u | (seed & 0x0Fu));
    for (int y = 0; y < SRC_H; y++) {
        for (int x = 0; x < SRC_W; x++) {
            int rc = vram_write_byte(fb_base_px + uint32_t(y) * SRC_W + x,
                                     guard);
            if (rc != 0) return rc;
        }
    }

    for (int y = 0; y < SRC_H; y++) {
        for (int x = 0; x < SRC_W; x++) {
            int rc = vram_write_byte(pixel_addr(fb_base_px, fb_stride_px, x, y),
                                     checker_byte(x, y, seed));
            if (rc != 0) return rc;
        }
        int rc0 = vram_write_byte(pixel_addr(fb_base_px, fb_stride_px, SRC_W, y),
                                  guard);
        int rc1 = vram_write_byte(pixel_addr(fb_base_px, fb_stride_px,
                                             SRC_W + 1, y), guard);
        if (rc0 != 0) return rc0;
        if (rc1 != 0) return rc1;
    }
    return 0;
}

static void enqueue_checkerboard(BackgroundVramWriter& writer,
                                 uint32_t fb_base_px,
                                 uint32_t fb_stride_px,
                                 uint8_t seed) {
    uint8_t guard = static_cast<uint8_t>(0x70u | (seed & 0x0Fu));
    for (int y = 0; y < SRC_H; y++) {
        for (int x = 0; x < SRC_W; x++) {
            writer.enqueue(fb_base_px + uint32_t(y) * SRC_W + x, guard);
        }
    }

    for (int y = 0; y < SRC_H; y++) {
        for (int x = 0; x < SRC_W; x++) {
            writer.enqueue(pixel_addr(fb_base_px, fb_stride_px, x, y),
                           checker_byte(x, y, seed));
        }
        writer.enqueue(pixel_addr(fb_base_px, fb_stride_px, SRC_W, y), guard);
        writer.enqueue(pixel_addr(fb_base_px, fb_stride_px, SRC_W + 1, y), guard);
    }
}

static uint32_t expected_addr(int read_idx,
                              uint32_t fb_base_px,
                              uint32_t fb_stride_px) {
    int src_y = read_idx / SRC_W;
    int src_x = read_idx % SRC_W;
    return fb_base_px + uint32_t(src_y) * fb_stride_px + uint32_t(src_x);
}

static uint32_t expected_rgb_at(int out_x, int out_y, uint8_t seed) {
    bool in_x = out_x >= BORDER_X && out_x < BORDER_X + ACTIVE_W;
    bool in_y = out_y >= BORDER_Y && out_y < BORDER_Y + ACTIVE_H;
    if (!in_x || !in_y) return 0;

    int src_x = (out_x - BORDER_X) / 2;
    int src_y = (out_y - BORDER_Y) / 2;
    return palette_rgb(checker_byte(src_x, src_y, seed));
}

static FrameCapture run_frame(uint32_t fb_base_px, uint32_t fb_stride_px) {
    FrameCapture cap;
    active_capture = &cap;

    uint32_t limit = fb_base_px + uint32_t(SRC_H - 1) * fb_stride_px
                   + uint32_t(SRC_W - 1);
    int run_len = 0;
    int step = 0;

    for (int v = 0; v < DST_H; v++) {
        for (int h = 0; h < DST_W; h++) {
            dut->hcount = h;
            dut->vcount = v;
            dut->de_in = 1;
            dut->hs_in = 0;
            dut->vs_in = 0;
            tick_scan_step(step++);

            if (dut->fb_rd_en && dut->fb_rd_ready) {
                bool in_x = h >= BORDER_X && h < BORDER_X + ACTIVE_W;
                bool in_y = v >= BORDER_Y && v < BORDER_Y + ACTIVE_H;
                if (!in_x || !in_y) cap.out_of_window_reads++;
                if (dut->fb_rd_addr > limit) cap.out_of_range_reads++;
                cap.fb_addrs.push_back(dut->fb_rd_addr);
                run_len++;
            } else if (run_len != 0) {
                cap.runs.push_back(run_len);
                run_len = 0;
            }

            if (dut->scanout_de) {
                cap.rgbs.push_back(dut->scanout_rgb & 0xFFFFFFu);
            }
        }
    }

    dut->de_in = 0;
    dut->hcount = 0;
    dut->vcount = 0;
    for (int flush = 0; flush < 96; flush++) {
        tick_scan_step(step++);
        if (dut->scanout_de) {
            cap.rgbs.push_back(dut->scanout_rgb & 0xFFFFFFu);
        }
    }
    dut->hcount = 1;
    dut->vcount = 0;

    if (run_len != 0) cap.runs.push_back(run_len);
    active_capture = nullptr;
    return cap;
}

static FrameCapture run_frame_with_background_writer(uint32_t fb_base_px,
                                                     uint32_t fb_stride_px,
                                                     BackgroundVramWriter& writer) {
    active_writer = &writer;
    FrameCapture cap = run_frame(fb_base_px, fb_stride_px);
    active_writer = nullptr;
    return cap;
}

static FrameCapture prime_scanout_prefetch(uint32_t fb_base_px,
                                           uint32_t fb_stride_px,
                                           int cycles = 2048,
                                           bool slow_vram = false) {
    FrameCapture cap;
    active_capture = &cap;
    capture_prefetch_filter = true;
    capture_prefetch_vram_started = false;
    capture_prefetch_base = fb_base_px;
    capture_prefetch_expected = SRC_W * SRC_H;
    int step = 0;
    int run_len = 0;
    bool fb_started = false;
    uint32_t limit = fb_base_px + uint32_t(SRC_H - 1) * fb_stride_px
                   + uint32_t(SRC_W - 1);

    dut->hcount = 0;
    dut->vcount = 0;
    dut->de_in = 1;
    dut->hs_in = 0;
    dut->vs_in = 0;
    tick_scan_step_mode(step++, slow_vram);
    if (dut->fb_rd_en && dut->fb_rd_ready) {
        if (!fb_started && dut->fb_rd_addr == fb_base_px)
            fb_started = true;
        if (fb_started && int(cap.fb_addrs.size()) < capture_prefetch_expected) {
            cap.fb_addrs.push_back(dut->fb_rd_addr);
            if (dut->fb_rd_addr > limit) cap.out_of_range_reads++;
            run_len++;
        }
    }

    dut->hcount = 1;
    dut->vcount = 0;
    dut->de_in = 0;
    for (int i = 0; i < cycles; i++) {
        tick_scan_step_mode(step++, slow_vram);
        if (dut->fb_rd_en && dut->fb_rd_ready) {
            if (!fb_started && dut->fb_rd_addr == fb_base_px)
                fb_started = true;
            if (fb_started && int(cap.fb_addrs.size()) < capture_prefetch_expected) {
                cap.fb_addrs.push_back(dut->fb_rd_addr);
                if (dut->fb_rd_addr > limit) cap.out_of_range_reads++;
                run_len++;
            }
        } else if (run_len != 0) {
            cap.runs.push_back(run_len);
            run_len = 0;
        }
        if (int(cap.fb_addrs.size()) == capture_prefetch_expected &&
            int(cap.vram_addrs.size()) == capture_prefetch_expected) {
            break;
        }
    }
    if (run_len != 0) cap.runs.push_back(run_len);

    active_capture = nullptr;
    capture_prefetch_filter = false;
    capture_prefetch_vram_started = false;
    return cap;
}

static bool check_geometry(const FrameCapture& cap,
                           uint32_t fb_base_px,
                           uint32_t fb_stride_px,
                           bool allow_fragmented_runs = false) {
    bool ok = true;
    int expected_reads = SRC_W * SRC_H;
    if (int(cap.fb_addrs.size()) != expected_reads) {
        std::printf("    fb read count got %zu expected %d\n",
                    cap.fb_addrs.size(), expected_reads);
        ok = false;
    }
    if (int(cap.vram_addrs.size()) != expected_reads) {
        std::printf("    vram read count got %zu expected %d\n",
                    cap.vram_addrs.size(), expected_reads);
        ok = false;
    }
    if (allow_fragmented_runs) {
        if (int(cap.runs.size()) < 2) {
            std::printf("    throttled prefetch run count got %zu expected >=2\n",
                        cap.runs.size());
            ok = false;
        }
    } else {
        if (int(cap.runs.size()) != 1) {
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
    }
    for (size_t i = 0; i < cap.fb_addrs.size() && i < size_t(expected_reads); i++) {
        uint32_t want = expected_addr(int(i), fb_base_px, fb_stride_px);
        if (cap.fb_addrs[i] != want) {
            std::printf("    fb_addr[%zu] got 0x%05x expected 0x%05x\n",
                        i, cap.fb_addrs[i], want);
            ok = false;
            break;
        }
    }
    for (size_t i = 0; i < cap.vram_addrs.size() && i < size_t(expected_reads); i++) {
        uint32_t want = expected_addr(int(i), fb_base_px, fb_stride_px);
        if (cap.vram_addrs[i] != want) {
            std::printf("    vram_addr[%zu] got 0x%05x expected 0x%05x\n",
                        i, cap.vram_addrs[i], want);
            ok = false;
            break;
        }
    }
    if (cap.out_of_range_reads != 0) {
        std::printf("    reads beyond programmed frame: %d\n",
                    cap.out_of_range_reads);
        ok = false;
    }
    return ok;
}

static bool check_pixels(const FrameCapture& cap,
                         const char* label,
                         uint8_t seed) {
    bool ok = true;
    int expected_pixels = DST_W * DST_H;
    if (int(cap.rgbs.size()) != expected_pixels) {
        std::printf("    %s: pixel count got %zu expected %d\n",
                    label, cap.rgbs.size(), expected_pixels);
        ok = false;
    }

    int n = int(cap.rgbs.size()) < expected_pixels
        ? int(cap.rgbs.size()) : expected_pixels;
    for (int i = 0; i < n; i++) {
        int x = i % DST_W;
        int y = i / DST_W;
        uint32_t want = expected_rgb_at(x, y, seed);
        if (cap.rgbs[i] != want) {
            std::printf("    %s: rgb[%d,%d] got 0x%06x expected 0x%06x\n",
                        label, x, y, cap.rgbs[i], want);
            ok = false;
            break;
        }
    }
    return ok;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_vram_scaler_firstlight;

    reset_dut();
    CHECK(dut->fb_base_px == DEFAULT_BASE &&
          dut->fb_stride_px == DEFAULT_STRIDE &&
          dut->fb_bpp_reg == 0,
          "effective scanout state resets to safe defaults");

    // MAME-canonical writes: m_base = (BASE_HI<<9)|(BASE_LO<<5);
    // m_stride = STRIDE<<2; BPP from PCBR @ +0x220 bits[4:2].
    // base0 = 0x100 → BASE_HI=0, BASE_LO=8.  stride0 = 0x400 → STRIDE=0x100.
    // PCBR=0x18 → 8bpp.
    const uint32_t base0 = 0x00000100u;
    const uint32_t stride0 = 0x00000400u;
    CHECK(dafb_write(0x00, 0) == 0,    "program DAFB BASE_HI +0x00 <- 0");
    CHECK(dafb_write(0x04, 8) == 0,    "program DAFB BASE_LO +0x04 <- 8");
    CHECK(dafb_write(0x08, 0x100) == 0, "program DAFB STRIDE +0x08 <- 0x100");
    CHECK(dafb_write(0x220, 0x18) == 0, "program DAFB PCBR +0x220 <- 0x18 (8bpp)");
    CHECK(dut->fb_base_px == DEFAULT_BASE, "scanout base holds default before frame boundary");
    CHECK(dut->fb_stride_px == DEFAULT_STRIDE, "scanout stride holds default before frame boundary");
    CHECK(dut->fb_bpp_reg == 0x00000008u, "DAFB bpp latch is 0x%08x",
          dut->fb_bpp_reg);

    for (int i = 0; i < 6; i++) tick_pclk();
    CHECK(dut->fb_base_px == DEFAULT_BASE, "stable DAFB base still waits for frame_start");
    CHECK(dut->fb_stride_px == DEFAULT_STRIDE, "stable DAFB stride still waits for frame_start");

    CHECK(program_clut() == 0, "program 256-entry RAMDAC CLUT for scanout");

    uint8_t seed0 = 0x03;
    CHECK(write_checkerboard(base0, stride0, seed0) == 0,
          "CPU-style AXI byte writes fill strided checkerboard");
    CHECK(first_vram_write.valid && first_vram_write.addr == base0,
          "first VRAM AXI activity is framebuffer byte write addr=0x%05x data=0x%02x cycle=%llu",
          first_vram_write.addr,
          first_vram_write.data,
          (unsigned long long)first_vram_write.cycle);

    first_vram_scanout_read = VramActivity{};
    FrameCapture prefetch0 = prime_scanout_prefetch(base0, stride0, 4096, true);
    CHECK(dut->fb_base_px == base0, "prefetch boundary commits DAFB base to 0x%08x",
          dut->fb_base_px);
    CHECK(dut->fb_stride_px == stride0, "prefetch boundary commits DAFB stride to 0x%08x",
          dut->fb_stride_px);
    CHECK(check_geometry(prefetch0, base0, stride0, true),
          "line buffer prefetches the DAFB-programmed source image across slow VRAM backpressure");

    BackgroundVramWriter writer;
    // relocated_base = 0x200 → BASE_HI=1, BASE_LO=0.
    // relocated_stride = 0x400 → STRIDE=0x100.  Same depth (PCBR sticky).
    const uint32_t relocated_base = 0x00000200u;
    const uint32_t relocated_stride = 0x00000400u;
    uint8_t seed1 = 0x0B;
    enqueue_checkerboard(writer, relocated_base, relocated_stride, seed1);

    FrameCapture frame0 = run_frame_with_background_writer(base0, stride0,
                                                           writer);
    CHECK(first_vram_scanout_read.valid && first_vram_scanout_read.addr == base0,
          "first scanout VRAM read observes programmed framebuffer base addr=0x%05x cycle=%llu",
          first_vram_scanout_read.addr,
          (unsigned long long)first_vram_scanout_read.cycle);
    CHECK(check_pixels(frame0, "q700-stride-checkerboard", seed0),
          "current framebuffer scans out exactly while CPU writes offscreen VRAM");
    CHECK(dut->fb_underflow_sticky == 0,
          "fb_reader underflow stays clear during current-frame scanout");
    CHECK(writer.done() && writer.failed() == 0,
          "background AXI writes completed during bounded scanout window (%d beats)",
          writer.completed());

    CHECK(dafb_write(0x00, 1) == 0,    "reprogram DAFB BASE_HI +0x00 <- 1");
    CHECK(dafb_write(0x04, 0) == 0,    "reprogram DAFB BASE_LO +0x04 <- 0");
    CHECK(dafb_write(0x08, 0x100) == 0, "reprogram DAFB STRIDE +0x08 <- 0x100");
    // PCBR is sticky from earlier write; no need to re-poke.
    CHECK(dut->fb_base_px == base0 && dut->fb_stride_px == stride0,
          "DAFB relocation remains pending until the next frame boundary");

    for (int i = 0; i < 6; i++) tick_pclk();
    FrameCapture prefetch1 = prime_scanout_prefetch(relocated_base, relocated_stride);
    CHECK(check_geometry(prefetch1, relocated_base, relocated_stride),
          "line buffer prefetches relocated source image once");
    FrameCapture frame1 = run_frame(relocated_base, relocated_stride);
    CHECK(dut->fb_base_px == relocated_base,
          "next frame commits relocated DAFB base to 0x%08x", dut->fb_base_px);
    CHECK(dut->fb_stride_px == relocated_stride,
          "next frame commits relocated DAFB stride to 0x%08x", dut->fb_stride_px);
    CHECK(frame1.fb_addrs.empty(),
          "relocated visible frame reuses prefetched line buffers without pclk read requests");
    CHECK(check_pixels(frame1, "relocated-checkerboard", seed1),
          "relocated real VRAM checkerboard scans out exactly");
    CHECK(dut->fb_underflow_sticky == 0,
          "fb_reader underflow stays clear after DAFB base/stride relocation");

    std::printf("  first_vram_write addr=0x%05x data=0x%02x cycle=%llu\n",
                first_vram_write.addr,
                first_vram_write.data,
                (unsigned long long)first_vram_write.cycle);
    std::printf("  first_vram_scanout_read addr=0x%05x cycle=%llu\n",
                first_vram_scanout_read.addr,
                (unsigned long long)first_vram_scanout_read.cycle);
    std::printf("--------------------------------\n");
    std::printf("  pass=%d fail=%d cycles=%llu\n",
                n_pass, n_fail, (unsigned long long)sim_time);

    dut->final();
    delete dut;
    return n_fail == 0 ? 0 : 1;
}
