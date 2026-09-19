#include <cstdint>
#include <cstdio>
#include <map>
#include <verilated.h>
#include "Vdcache.h"

static Vdcache* dut = nullptr;
static uint64_t sim_time = 0;

struct Mem {
    std::map<uint32_t, uint8_t> b;
    uint32_t read32(uint32_t a) const {
        uint32_t v = 0;
        for (int i = 0; i < 4; i++) {
            auto it = b.find((a & ~0x3u) + (uint32_t)i);
            uint8_t x = (it == b.end()) ? 0 : it->second;
            v |= (uint32_t)x << ((3 - i) * 8);
        }
        return v;
    }
} mem;

struct Axi {
    bool ar_active = false;
    uint32_t ar_addr = 0;
    uint8_t ar_len = 0;
    uint8_t ar_size = 2;
    uint8_t ar_burst = 1;
    uint8_t ar_beat = 0;
    int ar_delay = 0;
    bool r_pending = false;
    uint64_t ar_hs = 0;
    uint64_t r_hs = 0;
    uint8_t first_len = 0;
    uint8_t first_size = 0;
    uint8_t first_burst = 0;
    bool first_seen = false;
} ax;

static void drive_slave() {
    dut->ar_ready = (!ax.ar_active && !ax.r_pending) ? 1 : 0;
    dut->r_valid = ax.r_pending ? 1 : 0;
    dut->r_resp = 0;
    if (ax.r_pending) {
        uint32_t a = ax.ar_addr;
        if (ax.ar_burst == 1) a += ((uint32_t)ax.ar_beat << ax.ar_size);
        dut->r_data = mem.read32(a);
    } else {
        dut->r_data = 0;
    }

    // Keep write channel idle (this test is read-only).
    dut->aw_ready = 1;
    dut->w_ready = 1;
    dut->b_valid = 0;
    dut->b_resp = 0;
}

static void tick() {
    drive_slave();
    uint8_t pre_ar_v = dut->ar_valid;
    uint8_t pre_ar_r = dut->ar_ready;
    uint32_t pre_ar_a = dut->ar_addr;
    uint8_t pre_ar_len = dut->ar_len;
    uint8_t pre_ar_size = dut->ar_size;
    uint8_t pre_ar_burst = dut->ar_burst;
    uint8_t pre_r_v = dut->r_valid;
    uint8_t pre_r_r = dut->r_ready;

    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();

    if (pre_ar_v && pre_ar_r) {
        ax.ar_active = true;
        ax.ar_addr = pre_ar_a;
        ax.ar_len = pre_ar_len;
        ax.ar_size = pre_ar_size;
        ax.ar_burst = pre_ar_burst;
        ax.ar_beat = 0;
        ax.ar_delay = 1;
        ax.ar_hs++;
        if (!ax.first_seen) {
            ax.first_seen = true;
            ax.first_len = pre_ar_len;
            ax.first_size = pre_ar_size;
            ax.first_burst = pre_ar_burst;
        }
    }

    if (ax.ar_active && !ax.r_pending) {
        if (ax.ar_delay > 0) ax.ar_delay--;
        else ax.r_pending = true;
    }

    if (pre_r_v && pre_r_r) {
        ax.r_hs++;
        if (ax.ar_beat == ax.ar_len) {
            ax.r_pending = false;
            ax.ar_active = false;
        } else {
            ax.ar_beat++;
            ax.r_pending = true;
        }
    }

    sim_time++;
}

static void reset() {
    dut->rst = 1;
    dut->req = 0;
    dut->is_write = 0;
    dut->addr = 0;
    dut->wdata = 0;
    dut->wstrb = 0;
    dut->cache_enable = 1;
    dut->cache_inh = 0;
    dut->maint_req = 0;
    dut->maint_is_inv = 0;
    dut->maint_scope = 0;
    dut->maint_addr = 0;
    dut->flush_all_req = 0;
    dut->flush_all_inval = 0;
    for (int i = 0; i < 8; i++) tick();
    dut->rst = 0;
    tick();
}

static uint32_t do_load(uint32_t addr) {
    dut->req = 1;
    dut->is_write = 0;
    dut->addr = addr;
    for (int i = 0; i < 2000; i++) {
        tick();
        if (dut->rvalid) {
            uint32_t d = dut->rdata;
            dut->req = 0;
            tick();
            return d;
        }
    }
    std::fprintf(stderr, "timeout waiting for rvalid at addr=0x%08x\n", addr);
    std::exit(2);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vdcache;

    const uint32_t line_base = 0x00001000u;
    for (int w = 0; w < 8; w++) {
        uint32_t v = 0xA0000000u | (uint32_t)w;
        uint32_t a = line_base + (uint32_t)(w * 4);
        mem.b[a + 0] = (uint8_t)((v >> 24) & 0xFF);
        mem.b[a + 1] = (uint8_t)((v >> 16) & 0xFF);
        mem.b[a + 2] = (uint8_t)((v >> 8) & 0xFF);
        mem.b[a + 3] = (uint8_t)(v & 0xFF);
    }

    reset();

    const uint32_t d0 = do_load(line_base + 12);
    if (d0 != 0xA0000003u) {
        std::fprintf(stderr, "FAIL: first load data mismatch got=0x%08x\n", d0);
        return 1;
    }
    if (ax.ar_hs != 1) {
        std::fprintf(stderr, "FAIL: expected 1 AR handshake, got %llu\n",
                     (unsigned long long)ax.ar_hs);
        return 1;
    }
    if (ax.r_hs != 8) {
        std::fprintf(stderr, "FAIL: expected 8 R beats, got %llu\n",
                     (unsigned long long)ax.r_hs);
        return 1;
    }
    if (ax.first_len != 7 || ax.first_size != 2 || ax.first_burst != 1) {
        std::fprintf(stderr,
                     "FAIL: first AR metadata len=%u size=%u burst=%u (want 7/2/1)\n",
                     (unsigned)ax.first_len, (unsigned)ax.first_size,
                     (unsigned)ax.first_burst);
        return 1;
    }

    const uint32_t ar_before = (uint32_t)ax.ar_hs;
    const uint32_t d1 = do_load(line_base + 20);
    if (d1 != 0xA0000005u) {
        std::fprintf(stderr, "FAIL: second load data mismatch got=0x%08x\n", d1);
        return 1;
    }
    if (ax.ar_hs != ar_before) {
        std::fprintf(stderr, "FAIL: hit path unexpectedly issued AR\n");
        return 1;
    }

    std::printf("PASS: dcache burst refill (ARLEN=7) smoke\n");
    return 0;
}
