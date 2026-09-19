// tb_vram_cpu_write.cpp — CPU→VRAM→scanner E2E unit testbench.
//
// Rationale
// ─────────
// `VIDEO_SMOKE=1` today proves scanner + HDMI alone: a reset-time AXI
// writer (`vram_smoke.v`) preloads VRAM with SMPTE bars.  What we DON'T
// have is an explicit test that proves an AXI write burst emitted
// with CPU-LSU-shaped traffic is byte-visible on the scanner-side
// read port.  When the ROM eventually writes the Mac boot logo to
// VRAM, any dropped or reordered AXI beat invalidates the entire
// mac-logo bring-up.  This tb is the last-mile gate for that path.
//
// Ground truth from scouting
// ──────────────────────────
// The axi_xbar today decodes FB_BASE (0x6000_0000) to the DDR slave
// (see rtl/sys/axi_xbar.v decode_slv() + ddr_flatten() — FB lands in
// contiguous DDR at AXI_DDR_FB_OFFSET).  The `vram` module's AXI
// slave is wired at rtl/fpga_top.v ONLY to the `vram_smoke` writer
// when VIDEO_SMOKE=1, and is tied-off otherwise.  There is no CPU→
// VRAM-AXI path wired through the xbar yet — that lands with the
// axi-xbar-vram retune ticket (3M×3S).  Therefore this tb exercises
// the vram slave directly with CPU-LSU-shaped traffic: multi-beat
// INCR bursts, varied offsets, varied WSTRB masks, across both legal
// BPP modes (8, 16 — BPP>16 is compile-refused by vram.v).
//
// Scenarios
// ─────────
//   A. BPP=8,  small logo-like block (32×16 pixels) via 2-beat INCR
//      bursts at successive word offsets — verify every pixel via
//      the streaming read port.
//   B. BPP=16, same logo-like block, same bursts.
//   C. BPP=8,  4-beat INCR burst at non-zero offset, verifies
//      multi-beat address increment.
//   D. BPP=8,  WSTRB-masked overwrite of an already-populated region
//      — verifies that the scanner sees the merged value (prove no
//      read-modify-write hazard between CPU writes and scanner reads).
//
// Build: make tb-vram-cpu-write
// Pass:  stdout shows "All N scenarios PASSED." and exit(0).

#include <array>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <verilated.h>
#include "Vtb_vram_cpu_write.h"

static Vtb_vram_cpu_write* dut = nullptr;
static int n_pass = 0, n_fail = 0;
static constexpr int VRAM_READ_LATENCY = 6;  // matches vram.v XPM_READ_LATENCY

// ── Clocking ────────────────────────────────────────────────────────────
// 200 MHz system clock — same period convention as tb_vram.cpp.  Both the
// AXI side and scanner read side share `clk` (matches production common-
// clock URAM constraint after task #132).

static uint64_t sim_step = 0;

static void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    sim_step++;
}

// ── Word128 helper (128-bit AXI beat packed as 4×uint32) ────────────────
using Word128 = std::array<uint32_t, 4>;

template <typename Port>
static void set_wdata(Port& p, const Word128& v) {
    for (int i = 0; i < 4; i++) p[i] = v[i];
}

static Word128 w128_zero() { return {0,0,0,0}; }

// Pack 16 bytes (lane 0 = byte 0 = bits [7:0]).
static Word128 w128_from_bytes(const uint8_t b[16]) {
    Word128 v{};
    for (int i = 0; i < 16; i++) {
        v[i / 4] |= (uint32_t(b[i]) << ((i % 4) * 8));
    }
    return v;
}
static uint8_t w128_get_byte(const Word128& w, int lane) {
    return uint8_t((w[lane / 4] >> ((lane % 4) * 8)) & 0xFF);
}

// ── Per-instance AXI views ──────────────────────────────────────────────
// The wrapper exposes two identically-shaped sets of AXI ports, prefixed
// i0_ (BPP=8) and i1_ (BPP=16).  We use a trait struct so a single set
// of helpers can drive either instance.
enum InstSel { INST0 = 0, INST1 = 1 };

template<InstSel I>
struct AxiPorts;

template<>
struct AxiPorts<INST0> {
    static void set_awid   (uint32_t v) { dut->i0_s_awid    = v; }
    static void set_awaddr (uint32_t v) { dut->i0_s_awaddr  = v; }
    static void set_awlen  (uint32_t v) { dut->i0_s_awlen   = v; }
    static void set_awsize (uint32_t v) { dut->i0_s_awsize  = v; }
    static void set_awburst(uint32_t v) { dut->i0_s_awburst = v; }
    static void set_awvalid(uint32_t v) { dut->i0_s_awvalid = v; }
    static uint32_t get_awready()       { return dut->i0_s_awready; }

    static void set_wstrb(uint32_t v)   { dut->i0_s_wstrb   = v; }
    static void set_wlast(uint32_t v)   { dut->i0_s_wlast   = v; }
    static void set_wvalid(uint32_t v)  { dut->i0_s_wvalid  = v; }
    static uint32_t get_wready()        { return dut->i0_s_wready; }
    static void set_wdata(const Word128& w) {
        for (int i = 0; i < 4; i++) dut->i0_s_wdata[i] = w[i];
    }

    static void set_bready(uint32_t v)  { dut->i0_s_bready  = v; }
    static uint32_t get_bvalid()        { return dut->i0_s_bvalid; }
    static uint32_t get_bresp()         { return dut->i0_s_bresp; }

    static void set_rd_addr(uint32_t v) { dut->i0_rd_addr   = v; }
    static void set_rd_en(uint32_t v)   { dut->i0_rd_en     = v; }
    // The streaming port returns a 32-bit GROUP of RD_DATA_W/BPP consecutive
    // pixels; the pixel AT rd_addr is the TOP BPP bits.  CONSUMER, so shift
    // it down -- this is bit-identical to the pre-widening BPP-wide rd_data.
    static uint32_t get_rd_data()       { return (dut->i0_rd_data >> (32 - 8)) & 0xFFu; }
    static uint32_t get_rd_valid()      { return dut->i0_rd_valid; }

    static void idle_axi() {
        dut->i0_s_awid = 0; dut->i0_s_awaddr = 0; dut->i0_s_awlen = 0;
        dut->i0_s_awsize = 0; dut->i0_s_awburst = 0; dut->i0_s_awvalid = 0;
        ::set_wdata(dut->i0_s_wdata, w128_zero());
        dut->i0_s_wstrb = 0; dut->i0_s_wlast = 0; dut->i0_s_wvalid = 0;
        dut->i0_s_bready = 1;
        dut->i0_s_arid = 0; dut->i0_s_araddr = 0; dut->i0_s_arlen = 0;
        dut->i0_s_arsize = 0; dut->i0_s_arburst = 0; dut->i0_s_arvalid = 0;
        dut->i0_s_rready = 1;
        dut->i0_rd_addr = 0; dut->i0_rd_en = 0;
    }
    static constexpr int BPP = 8;
    static const char* name() { return "BPP=8"; }
};

template<>
struct AxiPorts<INST1> {
    static void set_awid   (uint32_t v) { dut->i1_s_awid    = v; }
    static void set_awaddr (uint32_t v) { dut->i1_s_awaddr  = v; }
    static void set_awlen  (uint32_t v) { dut->i1_s_awlen   = v; }
    static void set_awsize (uint32_t v) { dut->i1_s_awsize  = v; }
    static void set_awburst(uint32_t v) { dut->i1_s_awburst = v; }
    static void set_awvalid(uint32_t v) { dut->i1_s_awvalid = v; }
    static uint32_t get_awready()       { return dut->i1_s_awready; }

    static void set_wstrb(uint32_t v)   { dut->i1_s_wstrb   = v; }
    static void set_wlast(uint32_t v)   { dut->i1_s_wlast   = v; }
    static void set_wvalid(uint32_t v)  { dut->i1_s_wvalid  = v; }
    static uint32_t get_wready()        { return dut->i1_s_wready; }
    static void set_wdata(const Word128& w) {
        for (int i = 0; i < 4; i++) dut->i1_s_wdata[i] = w[i];
    }

    static void set_bready(uint32_t v)  { dut->i1_s_bready  = v; }
    static uint32_t get_bvalid()        { return dut->i1_s_bvalid; }
    static uint32_t get_bresp()         { return dut->i1_s_bresp; }

    static void set_rd_addr(uint32_t v) { dut->i1_rd_addr   = v; }
    static void set_rd_en(uint32_t v)   { dut->i1_rd_en     = v; }
    // See AxiPorts<INST0>::get_rd_data() -- same contract, BPP=16 here, so the
    // pixel at rd_addr is rd_data[31:16].
    // This instance is BPP=16, so vram.v's RD_DATA_W defaults to 4*BPP = 64
    // and the 4-lane group puts the pixel AT rd_addr in the TOP lane [63:48].
    static uint32_t get_rd_data()       { return uint32_t((dut->i1_rd_data >> (64 - 16)) & 0xFFFFu); }
    static uint32_t get_rd_valid()      { return dut->i1_rd_valid; }

    static void idle_axi() {
        dut->i1_s_awid = 0; dut->i1_s_awaddr = 0; dut->i1_s_awlen = 0;
        dut->i1_s_awsize = 0; dut->i1_s_awburst = 0; dut->i1_s_awvalid = 0;
        ::set_wdata(dut->i1_s_wdata, w128_zero());
        dut->i1_s_wstrb = 0; dut->i1_s_wlast = 0; dut->i1_s_wvalid = 0;
        dut->i1_s_bready = 1;
        dut->i1_s_arid = 0; dut->i1_s_araddr = 0; dut->i1_s_arlen = 0;
        dut->i1_s_arsize = 0; dut->i1_s_arburst = 0; dut->i1_s_arvalid = 0;
        dut->i1_s_rready = 1;
        dut->i1_rd_addr = 0; dut->i1_rd_en = 0;
    }
    static constexpr int BPP = 16;
    static const char* name() { return "BPP=16"; }
};

// ── AXI write burst helper ──────────────────────────────────────────────
// Drives AW + W beats + waits for B.  Returns 0 on OKAY BRESP, non-zero
// on SLVERR/timeout.  Templated on InstSel so it can target either
// vram instance.
template<InstSel I>
static uint32_t axi_write_burst(uint32_t addr,
                                const std::vector<Word128>& beats,
                                const std::vector<uint16_t>& strbs) {
    using P = AxiPorts<I>;
    if (beats.empty() || beats.size() != strbs.size() || beats.size() > 256) {
        std::printf("  [ERROR] axi_write_burst: bad burst size\n");
        return 2;
    }
    P::set_awid   (1);
    P::set_awaddr (addr);
    P::set_awlen  (uint8_t(beats.size() - 1));
    P::set_awsize (4);  // 16 bytes per beat
    P::set_awburst(1);  // INCR
    P::set_awvalid(1);

    for (int t = 0; t < 32; t++) {
        dut->eval();
        if (P::get_awready()) break;
        tick();
    }
    if (!P::get_awready()) {
        std::printf("  [ERROR] AWREADY timeout\n");
        P::set_awvalid(0);
        return 2;
    }
    tick();
    P::set_awvalid(0);

    for (size_t i = 0; i < beats.size(); i++) {
        P::set_wdata(beats[i]);
        P::set_wstrb (strbs[i]);
        P::set_wlast ((i + 1 == beats.size()) ? 1 : 0);
        P::set_wvalid(1);
        for (int t = 0; t < 32; t++) {
            dut->eval();
            if (P::get_wready()) break;
            tick();
        }
        if (!P::get_wready()) {
            std::printf("  [ERROR] WREADY timeout beat %zu\n", i);
            P::set_wvalid(0);
            return 2;
        }
        tick();
    }
    P::set_wvalid(0);
    P::set_wlast (0);
    P::set_wstrb (0);
    P::set_wdata (w128_zero());

    P::set_bready(1);
    for (int t = 0; t < 32; t++) {
        dut->eval();
        if (P::get_bvalid()) break;
        tick();
    }
    if (!P::get_bvalid()) {
        std::printf("  [ERROR] BVALID timeout\n");
        return 2;
    }
    uint32_t resp = P::get_bresp();
    tick();
    return resp;
}

// ── Scanner-side pixel read ─────────────────────────────────────────────
// Asserts rd_en + rd_addr for one cycle, then samples rd_data after the
// vram streaming port's internal XPM read latency.
template<InstSel I>
static uint32_t rd_pixel(uint32_t px) {
    using P = AxiPorts<I>;
    P::set_rd_addr(px);
    P::set_rd_en(1);
    tick();
    P::set_rd_en(0);
    P::set_rd_addr(0);
    for (int i = 1; i < VRAM_READ_LATENCY; i++) tick();

    uint32_t data = P::get_rd_data();
    bool valid    = P::get_rd_valid();
    if (!valid) {
        std::printf("  [ERROR] rd_valid low %d cycles after rd_en (px=0x%x)\n",
                    VRAM_READ_LATENCY, px);
        n_fail++;
    }
    return data;
}

// ── Reset ───────────────────────────────────────────────────────────────
static void reset() {
    AxiPorts<INST0>::idle_axi();
    AxiPorts<INST1>::idle_axi();
    dut->rst = 1;
    for (int i = 0; i < 4; i++) {
        dut->clk = 0; dut->eval();
        dut->clk = 1; dut->eval();
        sim_step++;
    }
    dut->rst = 0;
    tick();
}

// ── Logo-like pattern helper ────────────────────────────────────────────
// The "logo block" is a 32 (wide) × 16 (tall) region starting at pixel
// (0, 0) in the 128×48 frame.  At BPP=8 each pixel = 1 byte, so the
// block is 32 bytes/row × 16 rows = 512 bytes = 4 × 128-bit words per
// row (no — each row is 32 B = 2 words; 16 rows × 2 words = 32 words).
//
// At BPP=16 each row is 64 B = 4 words; 16 rows × 4 = 64 words.
//
// Pattern: byte value = (y & 0xF) << 4 | (x & 0xF).  Distinct,
// monotone, easy to visually recognise in stdout dump.
static uint32_t logo_byte(int x, int y) {
    return uint32_t(((y & 0xF) << 4) | (x & 0xF));
}
// BPP=16: 2 bytes per pixel = 0xRRGG where RR = (x * 7) & 0xFF, GG = (y * 11) & 0xFF.
static uint32_t logo_pix16(int x, int y) {
    uint32_t hi = uint32_t((x * 7) & 0xFF);
    uint32_t lo = uint32_t((y * 11) & 0xFF);
    return (hi << 8) | lo;
}

// ── Scenario A: BPP=8 logo write via 2-beat bursts ──────────────────────
// Frame is 128×48, BPP=8 → row stride = 128 B = 8 words.  Logo block
// sits at (0,0)..(31,15).  Each row of the block occupies bytes [0..31]
// of the row, i.e. words 0..1 of the row (since 128/16=8 words/row).
// We write each row as a 2-beat INCR burst at the row's base word addr.
static void scenario_A_bpp8_logo() {
    std::printf("[A] BPP=8 32×16 logo via 2-beat INCR bursts\n");
    constexpr int FB_W = 128;   // bytes per row
    constexpr int LOGO_W = 32;  // pixels (= bytes at BPP=8)
    constexpr int LOGO_H = 16;

    bool ok = true;

    for (int row = 0; row < LOGO_H; row++) {
        // Two beats of 16 bytes each cover the 32-byte logo row.
        uint8_t b0[16], b1[16];
        for (int i = 0; i < 16; i++) {
            b0[i] = uint8_t(logo_byte(i,      row));
            b1[i] = uint8_t(logo_byte(i + 16, row));
        }
        std::vector<Word128> beats{ w128_from_bytes(b0), w128_from_bytes(b1) };
        std::vector<uint16_t> strbs{ 0xFFFF, 0xFFFF };
        uint32_t row_base_byte = row * FB_W;   // byte address into VRAM
        uint32_t resp = axi_write_burst<INST0>(row_base_byte, beats, strbs);
        if (resp != 0) {
            std::printf("  FAIL: row %d BRESP=%u\n", row, resp);
            n_fail++;
            return;
        }
    }

    // Allow writes to commit.
    for (int i = 0; i < 4; i++) tick();

    // Read every logo pixel via the scanner port, compare.
    int first_bad = -1;
    uint32_t first_got = 0, first_want = 0;
    for (int y = 0; y < LOGO_H && ok; y++) {
        for (int x = 0; x < LOGO_W; x++) {
            uint32_t px_idx = uint32_t(y * FB_W + x);   // BPP=8: byte idx == pixel idx
            uint32_t got  = rd_pixel<INST0>(px_idx);
            uint32_t want = logo_byte(x, y);
            if (got != want) {
                if (first_bad < 0) {
                    first_bad = int(px_idx);
                    first_got = got;
                    first_want = want;
                }
                ok = false;
            }
        }
    }

    if (ok) {
        std::printf("  PASS (512 pixels verified byte-exact)\n");
        n_pass++;
    } else {
        std::printf("  FAIL: first mismatch px=%d got=0x%02x want=0x%02x\n",
                    first_bad, first_got, first_want);
        n_fail++;
    }
}

// ── Scenario B: BPP=16 logo write via 4-beat bursts ─────────────────────
// Frame is 128×48, BPP=16 → row stride = 256 B = 16 words.  Logo block
// sits at (0,0)..(31,15); row width = 32 px × 2 B = 64 B = 4 words.
static void scenario_B_bpp16_logo() {
    std::printf("[B] BPP=16 32×16 logo via 4-beat INCR bursts\n");
    constexpr int FB_W = 128;   // pixels per row
    constexpr int ROW_STRIDE_B = FB_W * 2;  // 256 bytes per row at 16bpp
    constexpr int LOGO_W = 32;
    constexpr int LOGO_H = 16;

    bool ok = true;

    for (int row = 0; row < LOGO_H; row++) {
        // 4 beats of 16 bytes = 64 bytes = 32 pixels × 2 B/pixel.
        // Lane mapping: byte 2*i = low byte of pixel i, byte 2*i+1 = high.
        // vram.v lane convention: pixel_i_within_word = wdata[i*BPP +: BPP].
        // At BPP=16, DATA_WIDTH=128 → PX_PER_WORD = 8.
        uint8_t bytes[4][16];
        for (int beat = 0; beat < 4; beat++) {
            for (int lane = 0; lane < 8; lane++) {   // 8 pixels per 128b word
                int px_x = beat * 8 + lane;
                uint32_t v = logo_pix16(px_x, row);
                bytes[beat][lane*2 + 0] = uint8_t(v & 0xFF);        // lo
                bytes[beat][lane*2 + 1] = uint8_t((v >> 8) & 0xFF); // hi
            }
        }
        std::vector<Word128> beats{
            w128_from_bytes(bytes[0]),
            w128_from_bytes(bytes[1]),
            w128_from_bytes(bytes[2]),
            w128_from_bytes(bytes[3])
        };
        std::vector<uint16_t> strbs{ 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF };
        uint32_t row_base_byte = row * ROW_STRIDE_B;
        uint32_t resp = axi_write_burst<INST1>(row_base_byte, beats, strbs);
        if (resp != 0) {
            std::printf("  FAIL: row %d BRESP=%u\n", row, resp);
            n_fail++;
            return;
        }
    }

    for (int i = 0; i < 4; i++) tick();

    int first_bad = -1;
    uint32_t first_got = 0, first_want = 0;
    for (int y = 0; y < LOGO_H && ok; y++) {
        for (int x = 0; x < LOGO_W; x++) {
            uint32_t px_idx = uint32_t(y * FB_W + x);
            uint32_t got  = rd_pixel<INST1>(px_idx);
            uint32_t want = logo_pix16(x, y);
            if (got != want) {
                if (first_bad < 0) {
                    first_bad = int(px_idx);
                    first_got = got;
                    first_want = want;
                }
                ok = false;
            }
        }
    }

    if (ok) {
        std::printf("  PASS (512 pixels verified half-word-exact)\n");
        n_pass++;
    } else {
        std::printf("  FAIL: first mismatch px=%d got=0x%04x want=0x%04x\n",
                    first_bad, first_got, first_want);
        n_fail++;
    }
}

// ── Scenario C: BPP=8 4-beat INCR burst at non-zero offset ──────────────
// Tests multi-beat address increment in vram.v's write FSM (ws_waddr +1).
// Writes 64 B at byte offset 0x100 (row 2 of 128-wide frame).  Reads
// back each byte as a scanner pixel.
static void scenario_C_bpp8_burst_len4() {
    std::printf("[C] BPP=8 4-beat INCR burst @ offset 0x100\n");
    constexpr uint32_t OFFSET_B = 0x100;

    uint8_t bytes[4][16];
    for (int beat = 0; beat < 4; beat++) {
        for (int i = 0; i < 16; i++) {
            bytes[beat][i] = uint8_t(0xC0 | ((beat * 16 + i) & 0x3F));
        }
    }
    std::vector<Word128> beats{
        w128_from_bytes(bytes[0]),
        w128_from_bytes(bytes[1]),
        w128_from_bytes(bytes[2]),
        w128_from_bytes(bytes[3])
    };
    std::vector<uint16_t> strbs{ 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF };
    uint32_t resp = axi_write_burst<INST0>(OFFSET_B, beats, strbs);
    if (resp != 0) {
        std::printf("  FAIL: BRESP=%u\n", resp);
        n_fail++;
        return;
    }

    for (int i = 0; i < 4; i++) tick();

    bool ok = true;
    int first_bad = -1;
    uint32_t first_got = 0, first_want = 0;
    for (int i = 0; i < 64; i++) {
        uint32_t px_idx = OFFSET_B + i;   // BPP=8: byte idx == pixel idx
        uint32_t got  = rd_pixel<INST0>(px_idx);
        uint32_t want = uint32_t(0xC0 | (i & 0x3F));
        if (got != want) {
            if (first_bad < 0) {
                first_bad = int(i);
                first_got = got;
                first_want = want;
            }
            ok = false;
        }
    }

    if (ok) {
        std::printf("  PASS (64 bytes across 4 beats verified)\n");
        n_pass++;
    } else {
        std::printf("  FAIL: first mismatch beat-offset %d got=0x%02x want=0x%02x\n",
                    first_bad, first_got, first_want);
        n_fail++;
    }
}

// ── Scenario D: BPP=8 WSTRB-masked overwrite ────────────────────────────
// Pre-fill word at offset 0x80 with 0xAA on all 16 lanes.  Then issue a
// single-beat burst with data=0xFF, WSTRB=0x5555 (even lanes only).
// Read each of the 16 pixels via the scanner: even = 0xFF, odd = 0xAA.
// This proves the vram slave's per-byte WSTRB merge is visible to the
// scanner (i.e. no read-modify-write hazard between the CPU write and
// the scanner read path).
static void scenario_D_bpp8_wstrb_mask() {
    std::printf("[D] BPP=8 WSTRB-masked overwrite visible on scanner\n");
    constexpr uint32_t OFFSET_B = 0x80;   // word-aligned

    // Pre-fill 0xAA
    uint8_t pre[16];
    for (int i = 0; i < 16; i++) pre[i] = 0xAA;
    {
        std::vector<Word128> b{ w128_from_bytes(pre) };
        std::vector<uint16_t> s{ 0xFFFF };
        if (axi_write_burst<INST0>(OFFSET_B, b, s) != 0) {
            std::printf("  FAIL: prefill BRESP nonzero\n");
            n_fail++;
            return;
        }
    }
    // Partial overwrite: 0xFF with even-lane strobes.
    uint8_t over[16];
    for (int i = 0; i < 16; i++) over[i] = 0xFF;
    {
        std::vector<Word128> b{ w128_from_bytes(over) };
        std::vector<uint16_t> s{ 0x5555 };
        if (axi_write_burst<INST0>(OFFSET_B, b, s) != 0) {
            std::printf("  FAIL: masked BRESP nonzero\n");
            n_fail++;
            return;
        }
    }

    for (int i = 0; i < 4; i++) tick();

    bool ok = true;
    for (int lane = 0; lane < 16; lane++) {
        uint32_t got  = rd_pixel<INST0>(OFFSET_B + lane);
        uint32_t want = (lane & 1) ? 0xAA : 0xFF;
        if (got != want) {
            std::printf("  FAIL: lane %d got=0x%02x want=0x%02x\n", lane, got, want);
            ok = false;
        }
    }

    if (ok) {
        std::printf("  PASS (WSTRB merge visible on scanner)\n");
        n_pass++;
    } else {
        n_fail++;
    }
}

// ─────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_vram_cpu_write;

    reset();
    scenario_A_bpp8_logo();
    reset();
    scenario_B_bpp16_logo();
    reset();
    scenario_C_bpp8_burst_len4();
    reset();
    scenario_D_bpp8_wstrb_mask();

    std::printf("────────────────────────────────\n");
    if (n_fail == 0) {
        std::printf("All %d scenarios PASSED.\n", n_pass);
    } else {
        std::printf("%d PASSED, %d FAILED.\n", n_pass, n_fail);
    }

    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
