// tb_sd_jtag_writer.cpp — Focused unit test for rtl/sys/sd_jtag_writer.v
//
// Drives the writer's AXI-Lite register aperture, then observes the sd_ctrl
// byte stream to verify a well-formed CMD24 sector write.
//
// Build: make tb-sd-jtag-writer

#include <cstdio>
#include <cstdint>
#include <vector>
#include "Vsd_jtag_writer.h"

static Vsd_jtag_writer* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0;
static int n_fail = 0;

static const uint32_t REG_LBA     = 0x00;
static const uint32_t REG_CTRL    = 0x04;
static const uint32_t REG_STATUS  = 0x08;
static const uint32_t REG_BUFPTR  = 0x0C;
static const uint32_t REG_BUFDATA = 0x10;
static const uint32_t REG_IDENT   = 0x14;
static const uint32_t CTRL_GO     = 0x5D000001;
static const uint32_t IDENT       = 0x5D7A0001;

struct SdByteModel {
    enum State {
        S_CMD,
        S_R1,
        S_GAP,
        S_TOKEN,
        S_DATA,
        S_CRC,
        S_RESP,
        S_BUSY
    };

    State state = S_CMD;
    std::vector<uint8_t> frame;
    std::vector<uint8_t> data;
    uint32_t lba = 0;
    int crc_count = 0;
    int busy_count = 4;
    bool cmd24_seen = false;
    bool token_seen = false;
    bool done = false;
    bool bad = false;

    uint8_t accept(uint8_t tx) {
        switch (state) {
            case S_CMD:
                // SPI-mode framing: the host idles the bus at 0xFF and a command
                // always starts with bit7=0, bit6=1 (0x40 | cmd). This model had
                // NO idle skip -- it took the first six bytes on the wire as a
                // command frame, so the writer's leading idle bytes were parsed
                // as command 0x3F, `bad` was set immediately, and every state
                // after that was shifted. Skip idle until a real command opens.
                if (frame.empty() && (tx & 0xC0) != 0x40) return 0xFF;
                frame.push_back(tx);
                if (frame.size() == 6) {
                    uint8_t cmd = frame[0] & 0x3F;
                    lba = ((uint32_t)frame[1] << 24) |
                          ((uint32_t)frame[2] << 16) |
                          ((uint32_t)frame[3] << 8) |
                          (uint32_t)frame[4];
                    if (cmd != 24) bad = true;
                    cmd24_seen = true;
                    state = S_R1;
                }
                return 0xFF;
            case S_R1:
                state = S_GAP;
                return 0x00;
            case S_GAP:
                // SD spec: after the R1 response the host must wait at least one
                // byte (Nwr >= 1) before sending the data token, and MAY wait
                // longer. This model used to accept EXACTLY one 0xFF and then
                // demand 0xFE, so a host that sends a second idle byte -- which
                // sd_jtag_writer legally does -- desynced it by one: the extra
                // 0xFF scored as a bad token, the REAL 0xFE became data[0], the
                // 512-byte count completed one byte early, and the response
                // phase never lined up. The writer then never saw its completion
                // and the tb failed on `done sticky` with busy stuck high.
                // Idle here until the token actually arrives.
                if (tx == 0xFF) return 0xFF;
                if (tx == 0xFE) { token_seen = true; state = S_DATA; return 0xFF; }
                bad = true;              // neither idle nor token: protocol error
                state = S_DATA;
                return 0xFF;
            case S_TOKEN:
                // Unreachable now (S_GAP absorbs the token) but kept so an
                // explicit state transition elsewhere still behaves.
                if (tx != 0xFE) bad = true;
                token_seen = true;
                state = S_DATA;
                return 0xFF;
            case S_DATA:
                data.push_back(tx);
                if (data.size() == 512) {
                    state = S_CRC;
                    crc_count = 0;
                }
                return 0xFF;
            case S_CRC:
                crc_count++;
                if (crc_count == 2) state = S_RESP;
                return 0xFF;
            case S_RESP:
                state = S_BUSY;
                busy_count = 4;
                return 0xE5;
            case S_BUSY:
                if (busy_count > 0) {
                    busy_count--;
                    return 0x00;
                }
                done = true;
                return 0xFF;
            default:
                return 0xFF;
        }
    }
} sd;

static bool rsp_pending = false;
static uint8_t rsp_byte = 0xFF;

static void tick() {
    dut->spi_cmd_ready = 1;
    dut->spi_rsp_valid = rsp_pending ? 1 : 0;
    dut->spi_rsp_data = rsp_byte;

    dut->clk = 0;
    dut->eval();
    bool cmd_fire = dut->spi_cmd_valid && dut->spi_cmd_ready;
    uint8_t cmd_byte = (uint8_t)dut->spi_cmd_data;
    dut->clk = 1;
    dut->eval();

    if (rsp_pending) {
        rsp_pending = false;
    }
    if (cmd_fire) {
        rsp_byte = sd.accept(cmd_byte);
        rsp_pending = true;
    }
    sim_time++;
}

static void idle_inputs() {
    dut->boot_done = 1;
    dut->s_awaddr = 0;
    dut->s_awvalid = 0;
    dut->s_wdata = 0;
    dut->s_wstrb = 0;
    dut->s_wvalid = 0;
    dut->s_bready = 0;
    dut->s_araddr = 0;
    dut->s_arvalid = 0;
    dut->s_rready = 0;
    dut->spi_cmd_ready = 1;
    dut->spi_rsp_valid = 0;
    dut->spi_rsp_data = 0xFF;
}

static void reset() {
    idle_inputs();
    sd = SdByteModel();
    rsp_pending = false;
    rsp_byte = 0xFF;
    dut->rst = 1;
    for (int i = 0; i < 8; i++) tick();
    dut->rst = 0;
    tick();
}

static void axil_write(uint32_t addr, uint32_t data, uint8_t strb = 0xF) {
    dut->s_awaddr = addr;
    dut->s_wdata = data;
    dut->s_wstrb = strb;
    dut->s_awvalid = 1;
    dut->s_wvalid = 1;
    dut->s_bready = 0;
    int guard = 0;
    while (!(dut->s_awready && dut->s_wready)) {
        tick();
        if (++guard > 1000) {
            printf("  AXI write ready timeout addr=0x%08x awready=%d wready=%d bvalid=%d\n",
                   addr, dut->s_awready, dut->s_wready, dut->s_bvalid);
            break;
        }
    }
    tick();
    dut->s_awvalid = 0;
    dut->s_wvalid = 0;
    guard = 0;
    while (!dut->s_bvalid) {
        tick();
        if (++guard > 1000) {
            printf("  AXI write response timeout addr=0x%08x\n", addr);
            break;
        }
    }
    dut->s_bready = 1;
    tick();
    dut->s_bready = 0;
}

static uint32_t axil_read(uint32_t addr) {
    dut->s_araddr = addr;
    dut->s_arvalid = 1;
    dut->s_rready = 0;
    int guard = 0;
    while (!dut->s_arready) {
        tick();
        if (++guard > 1000) {
            printf("  AXI read address timeout addr=0x%08x arready=%d\n",
                   addr, dut->s_arready);
            return 0xDEADBEEF;
        }
    }
    tick();
    dut->s_arvalid = 0;
    guard = 0;
    while (!dut->s_rvalid) {
        tick();
        if (++guard > 1000) {
            printf("  AXI read data timeout addr=0x%08x\n", addr);
            return 0xDEADBEEF;
        }
    }
    uint32_t data = dut->s_rdata;
    dut->s_rready = 1;
    tick();
    dut->s_rready = 0;
    return data;
}

static void tick_axi() {
    tick();
}

#define CHECK_TRUE(label, cond) do { \
    if (!(cond)) { printf("  FAIL %s\n", (label)); return false; } \
} while (0)

#define CHECK_EQ(label, got, exp) do { \
    uint32_t g = (uint32_t)(got); \
    uint32_t e = (uint32_t)(exp); \
    if (g != e) { \
        printf("  FAIL %s: got 0x%x expected 0x%x\n", (label), g, e); \
        return false; \
    } \
} while (0)

static bool test_ident_and_buffer_readback() {
    reset();
    CHECK_EQ("ident", axil_read(REG_IDENT), IDENT);

    axil_write(REG_BUFPTR, 0);
    for (int i = 0; i < 8; i++) axil_write(REG_BUFDATA, 0xA0 + i, 0x1);
    axil_write(REG_BUFPTR, 0);
    for (int i = 0; i < 8; i++) {
        CHECK_EQ("buffer readback", axil_read(REG_BUFDATA) & 0xFF, 0xA0 + i);
    }
    return true;
}

static bool test_cmd24_sector_write() {
    reset();
    const uint32_t LBA = 0x12345678;
    std::vector<uint8_t> pattern(512);
    for (int i = 0; i < 512; i++) pattern[i] = (uint8_t)((i * 13 + 7) & 0xFF);

    axil_write(REG_BUFPTR, 0);
    for (int i = 0; i < 512; i++) axil_write(REG_BUFDATA, pattern[i], 0x1);
    axil_write(REG_LBA, LBA);
    axil_write(REG_CTRL, CTRL_GO);

    uint64_t start = sim_time;
    while ((sim_time - start) < 3000000) {
        uint32_t st = axil_read(REG_STATUS);
        if ((st & 1) == 0 && (st & 2) != 0) break;
        tick_axi();
    }

    uint32_t status = axil_read(REG_STATUS);
    if ((status & 0x2) == 0) {
        printf("  status=0x%08x sd_state=%d cmd24=%d token=%d data=%zu bad=%d done=%d\n",
               status, (int)sd.state, sd.cmd24_seen, sd.token_seen,
               sd.data.size(), sd.bad, sd.done);
    }
    CHECK_TRUE("done sticky", (status & 0x2) != 0);
    CHECK_TRUE("no error", (status & 0x4) == 0);
    CHECK_TRUE("CMD24 seen", sd.cmd24_seen);
    CHECK_TRUE("data token seen", sd.token_seen);
    CHECK_TRUE("SD model complete", sd.done);
    CHECK_TRUE("model clean", !sd.bad);
    CHECK_EQ("LBA", sd.lba, LBA);
    CHECK_EQ("data length", sd.data.size(), 512u);
    for (int i = 0; i < 512; i++) {
        if (sd.data[i] != pattern[i]) {
            printf("  FAIL data[%d]: got 0x%02x expected 0x%02x\n",
                   i, sd.data[i], pattern[i]);
            return false;
        }
    }
    return true;
}

static void run_test(const char* name, bool (*fn)()) {
    printf("Running %s...\n", name);
    if (fn()) {
        printf("  PASS %s\n", name);
        n_pass++;
    } else {
        n_fail++;
    }
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vsd_jtag_writer;

    run_test("ident_and_buffer_readback", test_ident_and_buffer_readback);
    run_test("cmd24_sector_write", test_cmd24_sector_write);

    printf("sd_jtag_writer: %d passed, %d failed\n", n_pass, n_fail);
    delete dut;
    return n_fail ? 1 : 0;
}
