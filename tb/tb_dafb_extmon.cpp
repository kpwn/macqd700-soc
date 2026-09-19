// tb_dafb_extmon.cpp — extended-monitor sense unit tb for video.v.
//
// The default tb-dafb instantiates `video` with monitor_sense = 6 (12-14"
// RGB).  That code has bit 0x40 = 0, so the read-side passthrough always
// returns mon[2:0] regardless of m_monitor_id.  This tb instantiates the
// shim with monitor_sense = 0x5D (bit 6 + bc=1/ac=3/ab=1) which engages the
// m_monitor_id × monitor_sense convolution per MAME dafb.cpp:387-415, and
// exercises the three monitor_id branches (0x4 / 0x2 / 0x1) plus the
// "no drive" baseline.

#include <cstdio>
#include <cstdint>
#include <verilated.h>
#include "Vtb_dafb_extmon.h"

static Vtb_dafb_extmon* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0;
static int n_fail = 0;

#define CHECK(cond, fmt, ...) do { \
    if (cond) { n_pass++; std::printf("  [PASS] " fmt "\n", ##__VA_ARGS__); } \
    else      { n_fail++; std::printf("  [FAIL] " fmt "\n", ##__VA_ARGS__); } \
} while (0)

static void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    sim_time++;
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
}

static void reset() {
    idle_inputs();
    dut->rst = 1;
    for (int i = 0; i < 4; i++) tick();
    dut->rst = 0;
    tick();
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
        tick();
        if (aw_done) dut->s_axi_awvalid = 0;
        if (w_done)  dut->s_axi_wvalid  = 0;
        if (aw_done && w_done) break;
    }
    if (!aw_done || !w_done) return 1;
    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (dut->s_axi_bvalid) {
            tick();
            dut->s_axi_bready = 0;
            return (dut->s_axi_bresp == 0) ? 0 : 2;
        }
        tick();
    }
    return 3;
}

static int axil_read(uint32_t addr, uint32_t* out) {
    dut->s_axi_araddr  = addr;
    dut->s_axi_arvalid = 1;
    dut->s_axi_rready  = 1;
    bool ar_done = false;
    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (dut->s_axi_arready) { ar_done = true; tick(); break; }
        tick();
    }
    dut->s_axi_arvalid = 0;
    if (!ar_done) return 1;
    for (int i = 0; i < 64; i++) {
        dut->eval();
        if (dut->s_axi_rvalid) {
            *out = dut->s_axi_rdata;
            tick();
            dut->s_axi_rready = 0;
            return (dut->s_axi_rresp == 0) ? 0 : 2;
        }
        tick();
    }
    return 3;
}

// Reference computation matching MAME dafb.cpp:387-415 verbatim.
//   Inputs: m_monitor_id (3-bit drive pattern), mon (7-bit monitor_sense).
//   Returns res^7 (the inverse-sense response read at +0x1C).
static uint32_t mame_sense_response(uint8_t monitor_id, uint8_t mon) {
    uint8_t res;
    if (mon & 0x40) {
        res = 7;
        if (monitor_id == 0x4)
            res &= 4 | ((mon >> 5) & 1) << 1 | ((mon >> 4) & 1);
        if (monitor_id == 0x2)
            res &= ((mon >> 3) & 1) << 2 | 2 | ((mon >> 2) & 1);
        if (monitor_id == 0x1)
            res &= ((mon >> 1) & 1) << 2 | ((mon >> 0) & 1) << 1 | 1;
    } else {
        res = mon & 0x7f;
    }
    return (uint32_t)((res ^ 7) & 0x7);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_dafb_extmon;
    std::printf("── tb_dafb_extmon: DAFB monitor sense (ext) tb ──\n");
    reset();

    // The sense code is an INPUT PIN now (video.v's `monitor_sense`,
    // promoted onto tb_dafb_extmon.v's port list), not the compile-time
    // `.MONITOR_TYPE(7'h5D)` parameter override it used to be.  On
    // hardware the pin comes from the CPU debug-CSR at OFF_MON_SENSE
    // (0x0005C) via the socket signal cpu_mon_sense, so an operator can
    // sweep codes over JTAG instead of rebuilding a bitstream per code.
    //
    // 0x5D = bit 6 set with bc = mon[5:4] = 1, ac = mon[3:2] = 3,
    // ab = mon[1:0] = 1.  (An earlier comment here called this
    // "ext(2, 3, 1)"; that label is wrong — 0x40|(2<<4)|(3<<2)|1 = 0x6D.
    // The tb body always used the bit fields, so only the label was off.)
    // The m_monitor_id latched from a +0x1C write determines which 3-bit
    // window of mon[5:0] is selected:
    //   monitor_id == 4 → bc field (mon[5:4])
    //   monitor_id == 2 → ac field (mon[3:2])
    //   monitor_id == 1 → ab field (mon[1:0])
    uint8_t mon_type = 0x5D;
    dut->monitor_sense = mon_type;
    // Combinational into the read path; settle one cycle before reading.
    tick();

    uint32_t v;

    // Baseline — no extended drive (bit 6 of write data is 0).  The
    // chip drives mon[2:0] and we read back the inverse.  m_monitor_id
    // = (data & 0x7) ^ 7.  Write 0x07 → m_monitor_id = 0; bit 6 = 0
    // means standard passthrough → res = mon[2:0] = 5; read = 5^7 = 2.
    CHECK(axil_write(0x1C, 0x00000007) == 0, "ext: write 0x07 (standard, no drive)");
    CHECK(axil_read(0x1C, &v) == 0,           "ext: read +0x1C standard");
    {
        uint32_t want = mame_sense_response(0, mon_type);  // monitor_id=0, drive=0 → mon[2:0]=5 → 5^7=2
        CHECK(v == want, "ext: standard sense = 0x%x (want 0x%x = mon[2:0]^7)", v, want);
    }

    // Extended drive, monitor_id == 4 (data&7 = 3, ^7 = 4): bc field of
    // 0x5D = (mon[5:4]) = (1,0) = 1.  res = 7 & (4 | 1<<1 | 0) = 7 & 6 = 6.
    // Read returns 6^7 = 1.
    CHECK(axil_write(0x1C, 0x00000043) == 0, "ext: write 0x43 (drive ext, m_id=4)");
    CHECK(axil_read(0x1C, &v) == 0,           "ext: read +0x1C ext m_id=4");
    {
        uint32_t want = mame_sense_response(4, mon_type);
        CHECK(v == want, "ext: m_id=4 sense = 0x%x (want 0x%x)", v, want);
    }

    // Extended drive, monitor_id == 2 (data&7 = 5, ^7 = 2): ac field of
    // 0x5D = (mon[3:2]) = (1,1) = 3.  res = 7 & (1<<2 | 2 | 1) = 7 & 7 = 7.
    // Read returns 7^7 = 0.
    CHECK(axil_write(0x1C, 0x00000045) == 0, "ext: write 0x45 (drive ext, m_id=2)");
    CHECK(axil_read(0x1C, &v) == 0,           "ext: read +0x1C ext m_id=2");
    {
        uint32_t want = mame_sense_response(2, mon_type);
        CHECK(v == want, "ext: m_id=2 sense = 0x%x (want 0x%x)", v, want);
    }

    // Extended drive, monitor_id == 1 (data&7 = 6, ^7 = 1): ab field of
    // 0x5D = (mon[1:0]) = (0,1) = 1.  res = 7 & (0<<2 | 1<<1 | 1) = 7 & 3 = 3.
    // Read returns 3^7 = 4.
    CHECK(axil_write(0x1C, 0x00000046) == 0, "ext: write 0x46 (drive ext, m_id=1)");
    CHECK(axil_read(0x1C, &v) == 0,           "ext: read +0x1C ext m_id=1");
    {
        uint32_t want = mame_sense_response(1, mon_type);
        CHECK(v == want, "ext: m_id=1 sense = 0x%x (want 0x%x)", v, want);
    }

    // Drop drive bit while m_monitor_id stays latched: per MAME
    // dafb.cpp:469-470 the data&0x40 of the write is NOT captured;
    // the read-side convolution is gated on (monitor_sense & 0x40),
    // which is fixed at 1 for our 0x5D config.  Writing 0x06 just
    // updates m_monitor_id to (6&7)^7 = 1.  Response stays in the
    // ab-field branch: matches the m_id=1 case above (= 4).
    CHECK(axil_write(0x1C, 0x00000006) == 0, "ext: write 0x06 (data&0x40=0, m_id=1)");
    CHECK(axil_read(0x1C, &v) == 0,           "ext: read +0x1C after data&0x40=0");
    {
        uint32_t want = mame_sense_response(1, mon_type);
        CHECK(v == want, "ext: response 0x%x (want 0x%x — convolution still engaged via monitor_sense bit6)", v, want);
    }

    // Sweep all 8 m_monitor_id values with extended drive engaged, and
    // verify the RTL response matches MAME's reference computation.
    for (uint8_t mid = 0; mid < 8; mid++) {
        uint8_t write_data = 0x40 | ((mid ^ 0x7) & 0x7);
        CHECK(axil_write(0x1C, write_data) == 0,
              "ext sweep: write 0x%02x → m_id=%u", write_data, mid);
        CHECK(axil_read(0x1C, &v) == 0,
              "ext sweep: read +0x1C m_id=%u", mid);
        uint32_t want = mame_sense_response(mid, mon_type);
        CHECK(v == want, "ext sweep: m_id=%u → sense 0x%x (want 0x%x)",
              mid, v, want);
    }

    // ── The sense code is RUNTIME-SETTABLE ──────────────────────────────
    // This is the property the debug CSR exists to provide, and this tb is
    // the one that exercises the extended-monitor path, so prove it here:
    // walk the extended codes MAME knows (dafb.cpp:210-216) plus the full
    // bc/ac/ab space, changing the pin with no reset and no rebuild, and
    // require every (m_monitor_id × mon) response to keep matching MAME's
    // reference computation.  A datapath that only wired mon[2:0] through
    // (dropping bit 6 and bits [5:3]) would pass the fixed-0x5D sweep
    // above and fail here.
    static const uint8_t ext_codes[] = {
        // ext(bc, ac, ab) = 0x40 | (bc << 4) | (ac << 2) | ab, matching the
        // bit positions video.v's sense_response() reads (mon[5:4] = bc,
        // mon[3:2] = ac, mon[1:0] = ab).
        0x40 | (0 << 4) | (0 << 2) | 0,   // 0x40 = ext(0,0,0)
        0x40 | (1 << 4) | (2 << 2) | 3,   // 0x5B = ext(1,2,3)
        0x40 | (1 << 4) | (3 << 2) | 1,   // 0x5D — this tb's baseline code
        0x40 | (2 << 4) | (3 << 2) | 1,   // 0x6D = ext(2,3,1)
        0x40 | (3 << 4) | (3 << 2) | 3,   // 0x7F = ext(3,3,3), all fields open
        0x06,                             // NON-extended: 12-14" 640x480
        0x00,                             // NON-extended code 0 (21" Color)
        0x07,                             // NON-extended: "no monitor"
    };
    for (unsigned ci = 0; ci < sizeof(ext_codes) / sizeof(ext_codes[0]); ci++) {
        mon_type = ext_codes[ci];
        dut->monitor_sense = mon_type;
        tick();
        for (uint8_t mid = 0; mid < 8; mid++) {
            uint8_t write_data = 0x40 | ((mid ^ 0x7) & 0x7);
            CHECK(axil_write(0x1C, write_data) == 0,
                  "runtime sense 0x%02x: write 0x%02x → m_id=%u",
                  mon_type, write_data, mid);
            CHECK(axil_read(0x1C, &v) == 0,
                  "runtime sense 0x%02x: read +0x1C m_id=%u", mon_type, mid);
            uint32_t want = mame_sense_response(mid, mon_type);
            CHECK(v == want,
                  "runtime sense 0x%02x, m_id=%u → 0x%x (want 0x%x)",
                  mon_type, mid, v, want);
        }
    }
    // A change must also be visible WITHOUT an intervening +0x1C write —
    // nothing in video.v latches the pin, so the very next read sees it.
    // (On hardware Mac OS only re-probes sense at DAFB init, i.e. after a
    // CPU reset; that is an OS-side sampling limitation, not an RTL one.)
    // Latch m_monitor_id = 4 once, then change ONLY the pin between two
    // reads and require the answer to move.
    dut->monitor_sense = 0x5C;               // bc = mon[5:4] = 0b01
    tick();
    CHECK(axil_write(0x1C, 0x00000043) == 0, "no-write change: latch m_id=4");
    CHECK(axil_read(0x1C, &v) == 0,          "no-write change: baseline read");
    CHECK(v == mame_sense_response(4, 0x5C),
          "no-write change: baseline 0x5C → 0x%x (want 0x%x)",
          v, mame_sense_response(4, 0x5C));
    dut->monitor_sense = 0x7C;               // bc = mon[5:4] = 0b11
    tick();
    CHECK(axil_read(0x1C, &v) == 0,          "no-write change: read after pin change");
    CHECK(v == mame_sense_response(4, 0x7C),
          "no-write change: 0x7C → 0x%x (want 0x%x, no +0x1C write in between)",
          v, mame_sense_response(4, 0x7C));
    CHECK(mame_sense_response(4, 0x5C) != mame_sense_response(4, 0x7C),
          "no-write change: the two codes really do differ (test is discriminating)");

    std::printf("── result: pass=%d fail=%d\n", n_pass, n_fail);
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
