// mame_axi_periph_bridge.cpp - UNIX-socket MAME bridge for AXI peripheral_bus.

#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>
#include <verilated.h>

#include "Vmame_axi_periph_top.h"

namespace {

constexpr uint8_t OP_READ = 1;
constexpr uint8_t OP_WRITE = 2;
constexpr uint8_t RESP_OKAY = 0;
constexpr uint8_t RESP_SLVERR = 2;
constexpr uint8_t RESP_DECERR = 3;
constexpr uint8_t RESP_TIMEOUT = 4;
constexpr uint32_t MAX_WAIT_CYCLES = 256;
constexpr uint32_t VRAM_BASE = 0xf9000000u;
constexpr uint32_t VRAM_SIZE = 0x00200000u;

struct Request {
    uint8_t version = 0;
    uint8_t op = 0;
    uint8_t size = 0;
    uint8_t wstrb = 0;
    uint32_t addr = 0;
    uint32_t data = 0;
    uint32_t pc = 0;
    uint32_t cpu_cycles = 0;
    bool has_cpu_cycles = false;
};

struct Response {
    uint8_t resp = RESP_DECERR;
    uint32_t data = 0;
    uint32_t cycles = 0;
    uint8_t irq_bitmap = 0;
};

Vmame_axi_periph_top* dut = nullptr;
uint64_t sim_time = 0;
uint64_t trace_every = 0;
uint32_t cpu_cycle_scale = 1;
uint32_t max_cpu_advance = 4096;
bool trace_pc_change = false;
bool have_last_cpu_cycles = false;
uint32_t last_cpu_cycles = 0;
bool trace_scc_serial = false;
uint32_t active_pc = 0;
std::string vram_shm_path;
int vram_fd = -1;
uint8_t* vram_map = nullptr;
std::vector<uint8_t> vram_fallback;
std::vector<uint8_t> scc_rx_a_bytes;
std::vector<uint8_t> scc_rx_b_bytes;
size_t scc_rx_a_pos = 0;
size_t scc_rx_b_pos = 0;
volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int) {
    stop_requested = 1;
}

struct TraceStats {
    uint64_t total = 0;
    uint64_t reads = 0;
    uint64_t writes = 0;
    uint64_t via1 = 0;
    uint64_t via2 = 0;
    uint64_t enet = 0;
    uint64_t sonic = 0;
    uint64_t scc = 0;
    uint64_t scsi = 0;
    uint64_t asc = 0;
    uint64_t swim = 0;
    uint64_t vram = 0;
    uint64_t dafb = 0;
    uint64_t other = 0;
    uint32_t last_addr = 0;
    uint32_t last_pc = 0;
    uint32_t last_data = 0;
    uint8_t last_op = 0;
    uint8_t last_size = 0;
    uint8_t last_resp = 0;
    uint32_t last_trace_pc = 0xffffffffu;
    uint32_t last_trace_window = 0xffffffffu;
    uint64_t scc_tx_a = 0;
    uint64_t scc_tx_b = 0;
    uint64_t scc_rx_a = 0;
    uint64_t scc_rx_b = 0;
};

TraceStats stats;

enum class Window : uint32_t {
    VIA1,
    VIA2,
    ENET,
    SONIC,
    SCC,
    SCSI,
    ASC,
    SWIM,
    VRAM,
    DAFB,
    OTHER,
};

const char* window_name(Window w) {
    switch (w) {
    case Window::VIA1: return "via1";
    case Window::VIA2: return "via2";
    case Window::ENET: return "enet";
    case Window::SONIC: return "sonic";
    case Window::SCC: return "scc";
    case Window::SCSI: return "scsi";
    case Window::ASC: return "asc";
    case Window::SWIM: return "swim";
    case Window::VRAM: return "vram";
    case Window::DAFB: return "dafb";
    default: return "other";
    }
}

bool q700_io_offset(uint32_t addr, uint32_t& off) {
    if ((addr >> 24) != 0x50u)
        return false;

    const uint32_t raw = addr & 0x00ffffffu;
    const uint32_t service_nibble = raw >> 20;
    if (service_nibble == 0x1u || service_nibble == 0x8u ||
        service_nibble == 0x9u) {
        return false;
    }

    off = raw & ~0x00fc0000u;
    return true;
}

Window decode_window(uint32_t addr) {
    if (addr >= VRAM_BASE && addr < VRAM_BASE + VRAM_SIZE) return Window::VRAM;
    if (addr >= 0xf9800000u && addr <= 0xf98003ffu) return Window::DAFB;

    uint32_t off = 0;
    if (!q700_io_offset(addr, off))
        return Window::OTHER;

    if (off < 0x002000u) return Window::VIA1;
    if (off >= 0x002000u && off < 0x004000u) return Window::VIA2;
    if (off >= 0x008000u && off < 0x008008u) return Window::ENET;
    if (off >= 0x00a000u && off < 0x00b100u) return Window::SONIC;
    if (off >= 0x00c000u && off < 0x00e000u) return Window::SCC;
    if (off >= 0x00f000u && off < 0x00f102u) return Window::SCSI;
    if (off >= 0x014000u && off < 0x016000u) return Window::ASC;
    if (off >= 0x01e000u && off < 0x020000u) return Window::SWIM;
    return Window::OTHER;
}

uint8_t irq_bitmap() {
    uint8_t bits = 0;
    if (dut->via1_irq) bits |= 0x01;
    if (dut->via2_irq) bits |= 0x02;
    if (dut->scc_irq)  bits |= 0x04;
    if (dut->scsi_irq) bits |= 0x08;
    if (dut->asc_irq)  bits |= 0x10;
    if (dut->iwm_irq)  bits |= 0x20;
    return bits;
}

uint32_t be32(const uint8_t* p) {
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) |
           (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

void put_be32(uint8_t* p, uint32_t v) {
    p[0] = uint8_t(v >> 24);
    p[1] = uint8_t(v >> 16);
    p[2] = uint8_t(v >> 8);
    p[3] = uint8_t(v);
}

uint8_t axi_resp_to_bridge(uint8_t resp) {
    if (resp == 0)
        return RESP_OKAY;
    if (resp == 3)
        return RESP_DECERR;
    return RESP_SLVERR;
}

uint8_t default_wstrb(uint32_t addr, uint8_t size) {
    const uint32_t lane = addr & 0x3u;
    if (size == 1)
        return uint8_t(1u << lane);
    if (size == 2)
        return uint8_t(0x3u << (lane & 0x2u));
    return 0xfu;
}

uint32_t byte_shift(uint32_t addr) {
    return (addr & 0x3u) * 8u;
}

uint32_t read_data_word(uint32_t addr) {
    const uint32_t lane = (addr >> 2) & 0x3u;
    switch (lane) {
    case 0: return dut->s_rdata[0];
    case 1: return dut->s_rdata[1];
    case 2: return dut->s_rdata[2];
    default: return dut->s_rdata[3];
    }
}

uint32_t extract_sized(uint32_t word, uint32_t addr, uint8_t size) {
    if (size == 1)
        return (word >> byte_shift(addr)) & 0xffu;
    if (size == 2)
        return (word >> ((addr & 0x2u) * 8u)) & 0xffffu;
    return word;
}

uint32_t lane_replicate(uint32_t value, uint8_t size) {
    if (size == 1)
        return (value & 0xffu) * 0x01010101u;
    if (size == 2)
        return ((value & 0xffffu) << 16) | (value & 0xffffu);
    return value;
}

uint32_t protocol_write_value(const Request& req);

uint8_t* vram_bytes() {
    return vram_map ? vram_map : vram_fallback.data();
}

bool init_vram_backing() {
    if (vram_shm_path.empty()) {
        vram_fallback.assign(VRAM_SIZE, 0);
        return true;
    }

    vram_fd = ::open(vram_shm_path.c_str(), O_RDWR | O_CREAT, 0666);
    if (vram_fd < 0) {
        std::perror("open vram shm");
        return false;
    }
    if (::ftruncate(vram_fd, VRAM_SIZE) < 0) {
        std::perror("ftruncate vram shm");
        ::close(vram_fd);
        vram_fd = -1;
        return false;
    }
    void* p = ::mmap(nullptr, VRAM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, vram_fd, 0);
    if (p == MAP_FAILED) {
        std::perror("mmap vram shm");
        ::close(vram_fd);
        vram_fd = -1;
        return false;
    }
    vram_map = static_cast<uint8_t*>(p);
    std::memset(vram_map, 0, VRAM_SIZE);
    return true;
}

void close_vram_backing() {
    if (vram_map) {
        ::msync(vram_map, VRAM_SIZE, MS_SYNC);
        ::munmap(vram_map, VRAM_SIZE);
        vram_map = nullptr;
    }
    if (vram_fd >= 0) {
        ::close(vram_fd);
        vram_fd = -1;
    }
}

Response vram_transact(const Request& req) {
    const uint32_t off = req.addr - VRAM_BASE;
    if (off + req.size > VRAM_SIZE)
        return Response{RESP_DECERR, 0, 0};

    uint8_t* mem = vram_bytes();
    if (!mem)
        return Response{RESP_SLVERR, 0, 0};

    if (req.op == OP_WRITE) {
        const uint32_t value = protocol_write_value(req);
        if (req.size == 1) {
            mem[off] = uint8_t(value);
        } else if (req.size == 2) {
            mem[off] = uint8_t(value >> 8);
            mem[off + 1] = uint8_t(value);
        } else {
            mem[off] = uint8_t(value >> 24);
            mem[off + 1] = uint8_t(value >> 16);
            mem[off + 2] = uint8_t(value >> 8);
            mem[off + 3] = uint8_t(value);
        }
        return Response{RESP_OKAY, 0, 1};
    }

    uint32_t value = 0;
    if (req.size == 1) {
        value = mem[off];
    } else if (req.size == 2) {
        value = (uint32_t(mem[off]) << 8) | uint32_t(mem[off + 1]);
    } else {
        value = (uint32_t(mem[off]) << 24) | (uint32_t(mem[off + 1]) << 16) |
                (uint32_t(mem[off + 2]) << 8) | uint32_t(mem[off + 3]);
    }
    return Response{RESP_OKAY, lane_replicate(value, req.size), 1};
}

uint32_t protocol_write_value(const Request& req) {
    if (req.size == 1)
        return (req.data >> ((3u - (req.addr & 0x3u)) * 8u)) & 0xffu;
    if (req.size == 2)
        return (req.data >> (((req.addr & 0x2u) == 0) ? 16u : 0u)) & 0xffffu;
    return req.data;
}

bool parse_hex_bytes(const char* text, std::vector<uint8_t>& out) {
    const char* p = text;
    while (*p != '\0') {
        while (*p == ',' || *p == ':' || *p == ' ' || *p == '\t')
            p++;
        if (*p == '\0')
            break;
        char* end = nullptr;
        unsigned long v = std::strtoul(p, &end, 16);
        if (end == p || v > 0xfful)
            return false;
        out.push_back(uint8_t(v));
        p = end;
    }
    return true;
}

void clear_inputs() {
    dut->s_awid = 0;
    dut->s_awaddr = 0;
    dut->s_awlen = 0;
    dut->s_awsize = 0;
    dut->s_awburst = 1;
    dut->s_awvalid = 0;
    for (int i = 0; i < 4; i++)
        dut->s_wdata[i] = 0;
    dut->s_wstrb = 0;
    dut->s_wlast = 1;
    dut->s_wvalid = 0;
    dut->s_bready = 1;
    dut->s_arid = 0;
    dut->s_araddr = 0;
    dut->s_arlen = 0;
    dut->s_arsize = 0;
    dut->s_arburst = 1;
    dut->s_arvalid = 0;
    dut->s_rready = 1;
    dut->scc_rx_a_valid = 0;
    dut->scc_rx_a_data = 0;
    dut->scc_rx_b_valid = 0;
    dut->scc_rx_b_data = 0;
    dut->scc_cts_a_n = 1;
    dut->scc_dcd_a_n = 1;
    dut->scc_sync_a_n = 1;
    dut->scc_cts_b_n = 1;
    dut->scc_dcd_b_n = 1;
    dut->scc_sync_b_n = 1;
}

void tick() {
    dut->clk = 0;
    dut->eval();
    dut->clk = 1;
    dut->eval();
    sim_time++;
    if (dut->scc_tx_a_valid) {
        stats.scc_tx_a++;
        if (trace_scc_serial) {
            std::printf("mame_axi_periph_bridge scc-tx ch=A data=0x%02x pc=0x%08x sim=%llu\n",
                        unsigned(dut->scc_tx_a_data), active_pc,
                        (unsigned long long)sim_time);
            std::fflush(stdout);
        }
    }
    if (dut->scc_tx_b_valid) {
        stats.scc_tx_b++;
        if (trace_scc_serial) {
            std::printf("mame_axi_periph_bridge scc-tx ch=B data=0x%02x pc=0x%08x sim=%llu\n",
                        unsigned(dut->scc_tx_b_data), active_pc,
                        (unsigned long long)sim_time);
            std::fflush(stdout);
        }
    }
}

void reset_dut() {
    clear_inputs();
    dut->rst = 1;
    for (int i = 0; i < 8; i++)
        tick();
    dut->rst = 0;
    tick();
}

void inject_scc_rx(char ch, uint8_t data) {
    if (ch == 'A') {
        dut->scc_rx_a_data = data;
        dut->scc_rx_a_valid = 1;
        stats.scc_rx_a++;
    } else {
        dut->scc_rx_b_data = data;
        dut->scc_rx_b_valid = 1;
        stats.scc_rx_b++;
    }
    if (trace_scc_serial) {
        std::printf("mame_axi_periph_bridge scc-rx ch=%c data=0x%02x pc=0x%08x sim=%llu\n",
                    ch, unsigned(data), active_pc, (unsigned long long)sim_time);
        std::fflush(stdout);
    }
    tick();
    dut->scc_rx_a_valid = 0;
    dut->scc_rx_b_valid = 0;
}

void maybe_inject_scc_rx_after_status_read(const Request& req) {
    if (req.op != OP_READ || decode_window(req.addr) != Window::SCC)
        return;
    uint32_t q700_off = 0;
    if (!q700_io_offset(req.addr, q700_off))
        return;
    const uint32_t off = q700_off & 0xfffu;
    if (off == 0x022u && scc_rx_a_pos < scc_rx_a_bytes.size()) {
        inject_scc_rx('A', scc_rx_a_bytes[scc_rx_a_pos++]);
    } else if (off == 0x020u && scc_rx_b_pos < scc_rx_b_bytes.size()) {
        inject_scc_rx('B', scc_rx_b_bytes[scc_rx_b_pos++]);
    }
}

bool read_exact(int fd, uint8_t* buf, size_t len) {
    size_t got = 0;
    while (got < len) {
        if (stop_requested)
            return false;
        ssize_t n = ::read(fd, buf + got, len - got);
        if (n == 0)
            return false;
        if (n < 0) {
            if (errno == EINTR)
                return false;
            return false;
        }
        got += size_t(n);
    }
    return true;
}

bool write_exact(int fd, const uint8_t* buf, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        if (stop_requested)
            return false;
        ssize_t n = ::write(fd, buf + sent, len - sent);
        if (n < 0) {
            if (errno == EINTR)
                return false;
            return false;
        }
        sent += size_t(n);
    }
    return true;
}

bool decode_request(const uint8_t raw[20], Request& req) {
    if (std::memcmp(raw, "MRTB", 4) != 0)
        return false;
    req.version = raw[4];
    req.op = raw[5];
    req.size = raw[6];
    req.wstrb = raw[7];
    req.addr = be32(raw + 8);
    req.data = be32(raw + 12);
    req.pc = be32(raw + 16);
    if (req.version != 1 && req.version != 2)
        return false;
    if (req.op != OP_READ && req.op != OP_WRITE)
        return false;
    if (req.size != 1 && req.size != 2 && req.size != 4)
        return false;
    return true;
}

void decode_request_v2_tail(const uint8_t raw_tail[4], Request& req) {
    req.cpu_cycles = be32(raw_tail);
    req.has_cpu_cycles = true;
}

void encode_response(const Response& resp, uint8_t raw[16]) {
    std::memcpy(raw, "MRTB", 4);
    raw[4] = 1;
    raw[5] = resp.resp;
    raw[6] = resp.irq_bitmap;
    raw[7] = 0;
    put_be32(raw + 8, resp.data);
    put_be32(raw + 12, resp.cycles);
}

Response axi_read(const Request& req) {
    clear_inputs();
    dut->s_arid = 1;
    dut->s_araddr = req.addr;
    dut->s_arsize = (req.size == 1) ? 0 : (req.size == 2) ? 1 : 2;
    dut->s_arvalid = 1;
    dut->s_rready = 1;

    bool ar_done = false;
    for (uint32_t i = 0; i < MAX_WAIT_CYCLES; i++) {
        dut->eval();
        const bool ar_hs = dut->s_arvalid && dut->s_arready;
        const bool r_hs = dut->s_rvalid && dut->s_rready;
        uint32_t value = 0;
        uint8_t resp = RESP_TIMEOUT;
        if (r_hs) {
            const uint32_t word = read_data_word(req.addr);
            value = extract_sized(word, req.addr, req.size);
            resp = axi_resp_to_bridge(dut->s_rresp);
        }
        tick();
        if (!ar_done && ar_hs) {
            dut->s_arvalid = 0;
            ar_done = true;
        }
        if (r_hs) {
            tick();
            clear_inputs();
            return Response{resp, lane_replicate(value, req.size), i + 1, irq_bitmap()};
        }
    }
    clear_inputs();
    return Response{RESP_TIMEOUT, 0, MAX_WAIT_CYCLES, irq_bitmap()};
}

Response axi_write(const Request& req) {
    clear_inputs();
    const uint8_t wstrb4 = default_wstrb(req.addr, req.size);
    const uint32_t lane = (req.addr >> 2) & 0x3u;
    const uint32_t data32 = protocol_write_value(req) << byte_shift(req.addr);

    dut->s_awid = 1;
    dut->s_awaddr = req.addr;
    dut->s_awsize = (req.size == 1) ? 0 : (req.size == 2) ? 1 : 2;
    dut->s_awvalid = 1;
    dut->s_wlast = 1;
    dut->s_wvalid = 1;
    dut->s_bready = 1;
    for (int i = 0; i < 4; i++)
        dut->s_wdata[i] = 0;
    dut->s_wdata[lane] = data32;
    dut->s_wstrb = uint16_t(wstrb4) << (lane * 4u);

    bool aw_done = false;
    bool w_done = false;
    for (uint32_t i = 0; i < MAX_WAIT_CYCLES; i++) {
        dut->eval();
        const bool aw_hs = dut->s_awvalid && dut->s_awready;
        const bool w_hs = dut->s_wvalid && dut->s_wready;
        const bool b_hs = dut->s_bvalid && dut->s_bready;
        const uint8_t resp = b_hs ? axi_resp_to_bridge(dut->s_bresp) : RESP_TIMEOUT;
        tick();
        if (!aw_done && aw_hs) {
            dut->s_awvalid = 0;
            aw_done = true;
        }
        if (!w_done && w_hs) {
            dut->s_wvalid = 0;
            w_done = true;
        }
        if (b_hs) {
            tick();
            clear_inputs();
            return Response{resp, 0, i + 1, irq_bitmap()};
        }
    }
    clear_inputs();
    return Response{RESP_TIMEOUT, 0, MAX_WAIT_CYCLES, irq_bitmap()};
}

Response transact(const Request& req) {
    if (decode_window(req.addr) == Window::VRAM)
        return vram_transact(req);
    if (req.op == OP_READ)
        return axi_read(req);
    return axi_write(req);
}

Response bridge_read8(uint32_t addr) {
    Request req{};
    req.version = 1;
    req.op = OP_READ;
    req.size = 1;
    req.addr = addr;
    return transact(req);
}

Response bridge_write8(uint32_t addr, uint8_t data) {
    Request req{};
    req.version = 1;
    req.op = OP_WRITE;
    req.size = 1;
    req.addr = addr;
    req.data = lane_replicate(data, 1);
    return transact(req);
}

Response bridge_read16(uint32_t addr) {
    Request req{};
    req.version = 1;
    req.op = OP_READ;
    req.size = 2;
    req.addr = addr;
    return transact(req);
}

Response bridge_write16(uint32_t addr, uint16_t data) {
    Request req{};
    req.version = 1;
    req.op = OP_WRITE;
    req.size = 2;
    req.addr = addr;
    req.data = (addr & 2u) ? uint32_t(data) : (uint32_t(data) << 16);
    return transact(req);
}

Response bridge_read32(uint32_t addr) {
    Request req{};
    req.version = 1;
    req.op = OP_READ;
    req.size = 4;
    req.addr = addr;
    return transact(req);
}

Response bridge_write32(uint32_t addr, uint32_t data) {
    Request req{};
    req.version = 1;
    req.op = OP_WRITE;
    req.size = 4;
    req.addr = addr;
    req.data = data;
    return transact(req);
}

bool selftest_write8(uint32_t addr, uint8_t data) {
    Response r = bridge_write8(addr, data);
    for (int i = 0; i < 4; i++)
        tick();
    return r.resp == RESP_OKAY;
}

bool selftest_read8(uint32_t addr, uint8_t& data) {
    Response r = bridge_read8(addr);
    data = uint8_t(r.data & 0xffu);
    return r.resp == RESP_OKAY;
}

void selftest_advance(uint32_t cycles) {
    for (uint32_t i = 0; i < cycles; i++)
        tick();
}

bool selftest_asc_via2_irq_route(uint8_t& via2_ifr, uint8_t& irq_bits) {
    constexpr uint32_t VIA2_BASE = 0x50002000u;
    constexpr uint32_t VIA_REG_STRIDE = 0x200u;
    constexpr uint32_t VIA2_IFR = VIA2_BASE + 13u * VIA_REG_STRIDE;
    constexpr uint32_t VIA2_IER = VIA2_BASE + 14u * VIA_REG_STRIDE;
    constexpr uint32_t ASC_BASE = 0x50014000u;

    via2_ifr = 0;
    irq_bits = 0;

    // VIA2 CB1 is the Q700 ASC IRQ input.  PCR reset selects negative
    // edge, matching the active-low board pin driven by ~asc_irq.
    if (!selftest_write8(VIA2_IFR, 0x10u) ||
        !selftest_write8(VIA2_IER, 0x90u) ||
        !selftest_write8(ASC_BASE + 0x801u, 0x01u) ||
        !selftest_write8(ASC_BASE + 0x808u, 0x01u) ||
        !selftest_write8(ASC_BASE + 0x802u, 0x02u) ||
        !selftest_write8(ASC_BASE + 0x000u, 0x7fu) ||
        !selftest_write8(ASC_BASE + 0x000u, 0x80u) ||
        !selftest_write8(ASC_BASE + 0x400u, 0x80u) ||
        !selftest_write8(ASC_BASE + 0x400u, 0x7fu))
        return false;

    selftest_advance(2048);
    Response r_ifr = bridge_read8(VIA2_IFR);
    via2_ifr = uint8_t(r_ifr.data & 0xffu);
    irq_bits = r_ifr.irq_bitmap;
    return r_ifr.resp == RESP_OKAY &&
           ((via2_ifr & 0x90u) == 0x90u) &&
           ((irq_bits & 0x12u) == 0x12u);
}

bool selftest_rtc_shift_bit_in(uint8_t bit) {
    const uint8_t base = uint8_t(0xf8u | (bit & 1u));
    return selftest_write8(0x50000000u, uint8_t(base | 0x02u)) &&
           selftest_write8(0x50000000u, base) &&
           selftest_write8(0x50000000u, uint8_t(base | 0x02u));
}

bool selftest_rtc_shift_byte_in(uint8_t data) {
    for (int bit = 7; bit >= 0; bit--) {
        if (!selftest_rtc_shift_bit_in(uint8_t((data >> bit) & 1u)))
            return false;
    }
    return true;
}

bool selftest_rtc_shift_bit_out(uint8_t& bit) {
    uint8_t sample = 0;
    if (!selftest_write8(0x50000000u, 0xfau) ||
        !selftest_write8(0x50000000u, 0xf8u) ||
        !selftest_read8(0x50000000u, sample) ||
        !selftest_write8(0x50000000u, 0xfau))
        return false;
    bit = sample & 1u;
    return true;
}

bool selftest_rtc_xpram_read(uint8_t addr, uint8_t& data) {
    const uint8_t cmd = uint8_t(0x80u | 0x38u | ((addr >> 5) & 0x07u));
    const uint8_t addr_byte = uint8_t(((addr & 0x1fu) << 2) | 0x01u);
    data = 0;
    if (!selftest_write8(0x50000400u, 0xf7u) ||
        !selftest_write8(0x50000000u, 0xfeu) ||
        !selftest_write8(0x50000000u, 0xfau) ||
        !selftest_rtc_shift_byte_in(cmd) ||
        !selftest_rtc_shift_byte_in(addr_byte) ||
        !selftest_write8(0x50000400u, 0xf6u))
        return false;
    for (int bit = 7; bit >= 0; bit--) {
        uint8_t in_bit = 0;
        if (!selftest_rtc_shift_bit_out(in_bit))
            return false;
        data = uint8_t((data << 1) | in_bit);
    }
    return selftest_write8(0x50000400u, 0xf7u) &&
           selftest_write8(0x50000000u, 0xfeu);
}

void advance_cpu_delta(const Request& req) {
    if (!req.has_cpu_cycles) {
        have_last_cpu_cycles = false;
        return;
    }
    if (!have_last_cpu_cycles) {
        last_cpu_cycles = req.cpu_cycles;
        have_last_cpu_cycles = true;
        return;
    }

    uint32_t delta = req.cpu_cycles - last_cpu_cycles;
    last_cpu_cycles = req.cpu_cycles;
    if (cpu_cycle_scale > 1)
        delta /= cpu_cycle_scale;
    if (delta > max_cpu_advance)
        delta = max_cpu_advance;
    for (uint32_t i = 0; i < delta; i++)
        tick();
}

void update_trace_stats(const Request& req, const Response& resp) {
    stats.total++;
    if (req.op == OP_READ)
        stats.reads++;
    else
        stats.writes++;

    const Window window = decode_window(req.addr);
    switch (window) {
    case Window::VIA1: stats.via1++; break;
    case Window::VIA2: stats.via2++; break;
    case Window::ENET: stats.enet++; break;
    case Window::SONIC: stats.sonic++; break;
    case Window::SCC: stats.scc++; break;
    case Window::SCSI: stats.scsi++; break;
    case Window::ASC: stats.asc++; break;
    case Window::SWIM: stats.swim++; break;
    case Window::VRAM: stats.vram++; break;
    case Window::DAFB: stats.dafb++; break;
    default: stats.other++; break;
    }

    stats.last_addr = req.addr;
    stats.last_pc = req.pc;
    stats.last_data = (req.op == OP_READ) ? resp.data : req.data;
    stats.last_op = req.op;
    stats.last_size = req.size;
    stats.last_resp = resp.resp;

    if (trace_every != 0 && (stats.total % trace_every) == 0) {
        std::printf(
            "mame_axi_periph_bridge tx=%llu r=%llu w=%llu via1=%llu via2=%llu "
            "enet=%llu sonic=%llu scc=%llu scsi=%llu asc=%llu swim=%llu "
            "vram=%llu dafb=%llu other=%llu "
            "scc_tx_a=%llu scc_tx_b=%llu scc_rx_a=%llu scc_rx_b=%llu "
            "last=%s%u addr=0x%08x data=0x%08x "
            "pc=0x%08x resp=%u sim=%llu\n",
            (unsigned long long)stats.total,
            (unsigned long long)stats.reads,
            (unsigned long long)stats.writes,
            (unsigned long long)stats.via1,
            (unsigned long long)stats.via2,
            (unsigned long long)stats.enet,
            (unsigned long long)stats.sonic,
            (unsigned long long)stats.scc,
            (unsigned long long)stats.scsi,
            (unsigned long long)stats.asc,
            (unsigned long long)stats.swim,
            (unsigned long long)stats.vram,
            (unsigned long long)stats.dafb,
            (unsigned long long)stats.other,
            (unsigned long long)stats.scc_tx_a,
            (unsigned long long)stats.scc_tx_b,
            (unsigned long long)stats.scc_rx_a,
            (unsigned long long)stats.scc_rx_b,
            req.op == OP_READ ? "r" : "w",
            req.size,
            req.addr,
            stats.last_data,
            req.pc,
            resp.resp,
            (unsigned long long)sim_time);
        std::fflush(stdout);
    }
    if (trace_pc_change &&
        (req.pc != stats.last_trace_pc || uint32_t(window) != stats.last_trace_window)) {
        stats.last_trace_pc = req.pc;
        stats.last_trace_window = uint32_t(window);
        std::printf(
            "mame_axi_periph_bridge progress tx=%llu window=%s last=%s%u "
            "addr=0x%08x data=0x%08x pc=0x%08x resp=%u sim=%llu\n",
            (unsigned long long)stats.total,
            window_name(window),
            req.op == OP_READ ? "r" : "w",
            req.size,
            req.addr,
            stats.last_data,
            req.pc,
            resp.resp,
            (unsigned long long)sim_time);
        std::fflush(stdout);
    }
}

int listen_socket(const std::string& path) {
    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        std::perror("socket");
        return -1;
    }
    ::unlink(path.c_str());
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (path.size() >= sizeof(addr.sun_path)) {
        std::fprintf(stderr, "socket path too long: %s\n", path.c_str());
        ::close(fd);
        return -1;
    }
    std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("bind");
        ::close(fd);
        return -1;
    }
    if (::listen(fd, 1) < 0) {
        std::perror("listen");
        ::close(fd);
        return -1;
    }
    return fd;
}

void serve(const std::string& path) {
    int srv = listen_socket(path);
    if (srv < 0)
        std::exit(2);
    std::printf("mame AXI peripheral RTL bridge listening on %s\n", path.c_str());
    std::fflush(stdout);

    while (!stop_requested) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(srv, &rfds);
        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 250000;
        int ready = ::select(srv + 1, &rfds, nullptr, nullptr, &tv);
        if (ready < 0) {
            if (errno == EINTR)
                continue;
            std::perror("select");
            break;
        }
        if (ready == 0)
            continue;

        int conn = ::accept(srv, nullptr, nullptr);
        if (conn < 0) {
            if (errno == EINTR && !stop_requested)
                continue;
            std::perror("accept");
            break;
        }
        uint8_t req_raw[20];
        while (!stop_requested && read_exact(conn, req_raw, sizeof(req_raw))) {
            Request req;
            bool req_ok = decode_request(req_raw, req);
            if (req_ok && req.version == 2) {
                uint8_t req_tail[4];
                req_ok = read_exact(conn, req_tail, sizeof(req_tail));
                if (req_ok)
                    decode_request_v2_tail(req_tail, req);
            }
            if (req_ok)
                active_pc = req.pc;
            if (req_ok)
                advance_cpu_delta(req);
            Response resp = req_ok ? transact(req) : Response{RESP_DECERR, 0, 0};
            if (req.version == 1 || req.version == 2)
                update_trace_stats(req, resp);
            if (req_ok)
                maybe_inject_scc_rx_after_status_read(req);
            uint8_t resp_raw[16];
            encode_response(resp, resp_raw);
            if (!write_exact(conn, resp_raw, sizeof(resp_raw)))
                break;
        }
        ::close(conn);
    }
    ::close(srv);
    ::unlink(path.c_str());
}

} // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    std::string socket_path = "/tmp/mame-axi-periph-bridge.sock";
    bool selftest = false;
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--socket") == 0 && i + 1 < argc) {
            socket_path = argv[++i];
        } else if (std::strcmp(argv[i], "--selftest") == 0) {
            selftest = true;
        } else if (std::strcmp(argv[i], "--trace-every") == 0 && i + 1 < argc) {
            trace_every = std::strtoull(argv[++i], nullptr, 0);
        } else if (std::strcmp(argv[i], "--trace-pc-change") == 0) {
            trace_pc_change = true;
        } else if (std::strcmp(argv[i], "--trace-scc-serial") == 0) {
            trace_scc_serial = true;
        } else if (std::strcmp(argv[i], "--scc-rx-a-hex") == 0 && i + 1 < argc) {
            if (!parse_hex_bytes(argv[++i], scc_rx_a_bytes)) {
                std::fprintf(stderr, "invalid --scc-rx-a-hex byte list\n");
                return 2;
            }
        } else if (std::strcmp(argv[i], "--scc-rx-b-hex") == 0 && i + 1 < argc) {
            if (!parse_hex_bytes(argv[++i], scc_rx_b_bytes)) {
                std::fprintf(stderr, "invalid --scc-rx-b-hex byte list\n");
                return 2;
            }
        } else if (std::strcmp(argv[i], "--cpu-cycle-scale") == 0 && i + 1 < argc) {
            cpu_cycle_scale = uint32_t(std::strtoul(argv[++i], nullptr, 0));
            if (cpu_cycle_scale == 0)
                cpu_cycle_scale = 1;
        } else if (std::strcmp(argv[i], "--max-cpu-advance") == 0 && i + 1 < argc) {
            max_cpu_advance = uint32_t(std::strtoul(argv[++i], nullptr, 0));
        } else if (std::strcmp(argv[i], "--vram-shm") == 0 && i + 1 < argc) {
            vram_shm_path = argv[++i];
        } else if (argv[i][0] == '+') {
            // Verilated::commandArgs consumed simulation plusargs already.
            // Keep them legal here so RTL-local diagnostics such as
            // +rtc_trace can be enabled on the bridge executable.
        } else {
            std::fprintf(stderr, "usage: %s [--socket PATH] [--selftest] [--trace-every N] [--trace-pc-change] [--trace-scc-serial] [--scc-rx-a-hex HEX[,HEX...]] [--scc-rx-b-hex HEX[,HEX...]] [--cpu-cycle-scale N] [--max-cpu-advance N] [--vram-shm PATH] [+verilator_plusarg]\n", argv[0]);
            return 2;
        }
    }

    if (!init_vram_backing())
        return 2;
    dut = new Vmame_axi_periph_top;
    reset_dut();

    if (selftest) {
        Response r_via = bridge_read8(0x50000000u);
        Response r_enet = bridge_read8(0x50008001u);
        Response r_sonic_cr = bridge_read16(0x5000a000u);
        Response r_sonic_tcr = bridge_read16(0x5000a006u);
        Response w_sonic_exit = bridge_write16(0x5000a000u, 0x0000u);
        Response r_sonic_cr_run = bridge_read16(0x5000a000u);
        Response w_sonic_imr = bridge_write16(0x5000a008u, 0x0200u);
        Response r_sonic_imr = bridge_read16(0x5000a008u);
        Response r_scsi_status = bridge_read8(0x5000f040u);
        Response r_scsi_status_mirror = bridge_read8(0x50f0f040u);
        Response r_scc_status_mirror = bridge_read8(0x50f0c020u);
        const bool mirror_decode_ok =
            decode_window(0x50f0c020u) == Window::SCC &&
            decode_window(0x50f14834u) == Window::ASC &&
            decode_window(0x50f04000u) == Window::OTHER &&
            decode_window(0x50100000u) == Window::OTHER;
        Response w_vram = bridge_write32(0xf9000120u, 0x11223344u);
        Response r_vram = bridge_read32(0xf9000120u);
        Response w_vram_b = bridge_write8(0xf9000122u, 0xa5u);
        Response r_vram_b = bridge_read32(0xf9000120u);
        Response w_dafb_base = bridge_write32(0xf9800008u, 0x00000100u);
        Response r_dafb_base = bridge_read32(0xf9800008u);
        Response w_dafb_stride = bridge_write32(0xf980000cu, 0x0000061eu);
        Response r_dafb_stride = bridge_read32(0xf980000cu);
        Response r_dafb_sense = bridge_read32(0xf9800200u);
        Request gap{};
        gap.version = 1;
        gap.op = OP_READ;
        gap.size = 1;
        gap.addr = 0x50004000u;
        Response r_gap = transact(gap);
        uint8_t xpram78 = 0;
        uint8_t xpramf9 = 0;
        uint8_t xpram47 = 0;
        uint8_t xpramf8 = 0;
        uint8_t xpramfa = 0;
        uint8_t xpramfb = 0;
        uint8_t xpram77 = 0;
        uint8_t xpram7b = 0;
        uint8_t via2_asc_ifr = 0;
        uint8_t via2_asc_irq = 0;
        const bool rtc_ok = selftest_rtc_xpram_read(0x47u, xpram47) &&
                            selftest_rtc_xpram_read(0xf8u, xpramf8) &&
                            selftest_rtc_xpram_read(0xf9u, xpramf9) &&
                            selftest_rtc_xpram_read(0xfau, xpramfa) &&
                            selftest_rtc_xpram_read(0xfbu, xpramfb) &&
                            selftest_rtc_xpram_read(0x77u, xpram77) &&
                            selftest_rtc_xpram_read(0x78u, xpram78) &&
                            selftest_rtc_xpram_read(0x7bu, xpram7b);
        const bool asc_via2_irq_ok = selftest_asc_via2_irq_route(via2_asc_ifr, via2_asc_irq);
        if (r_via.resp != RESP_OKAY || r_gap.resp != RESP_DECERR ||
            r_enet.resp != RESP_OKAY || (r_enet.data & 0xffu) != 0xa0u ||
            r_sonic_cr.resp != RESP_OKAY || (r_sonic_cr.data & 0xffffu) != 0x0094u ||
            r_sonic_tcr.resp != RESP_OKAY || (r_sonic_tcr.data & 0xffffu) != 0x0101u ||
            w_sonic_exit.resp != RESP_OKAY ||
            r_sonic_cr_run.resp != RESP_OKAY || (r_sonic_cr_run.data & 0xffffu) != 0x0014u ||
            w_sonic_imr.resp != RESP_OKAY ||
            r_sonic_imr.resp != RESP_OKAY || (r_sonic_imr.data & 0xffffu) != 0x0200u ||
            r_scsi_status.resp != RESP_OKAY || r_scsi_status.data != 0x00000000u ||
            r_scsi_status_mirror.resp != RESP_OKAY ||
            r_scsi_status_mirror.data != 0x00000000u ||
            r_scc_status_mirror.resp != RESP_OKAY || !mirror_decode_ok ||
            w_vram.resp != RESP_OKAY || r_vram.resp != RESP_OKAY ||
            r_vram.data != 0x11223344u ||
            w_vram_b.resp != RESP_OKAY || r_vram_b.resp != RESP_OKAY ||
            r_vram_b.data != 0x1122a544u ||
            w_dafb_base.resp != RESP_OKAY || r_dafb_base.resp != RESP_OKAY ||
            r_dafb_base.data != 0x00000100u ||
            w_dafb_stride.resp != RESP_OKAY || r_dafb_stride.resp != RESP_OKAY ||
            r_dafb_stride.data != 0x0000061eu ||
            r_dafb_sense.resp != RESP_OKAY || r_dafb_sense.data != 0x00000007u ||
            !rtc_ok || xpram47 != 0x33u || xpramf8 != 0x00u ||
            xpramf9 != 0x01u || xpramfa != 0x00u || xpramfb != 0x00u ||
            xpram77 != 0x01u || xpram78 != 0xffu || xpram7b != 0xdfu ||
            !asc_via2_irq_ok) {
            std::fprintf(stderr,
                         "selftest failed via_resp=%u gap_resp=%u rtc_ok=%u "
                         "enet_resp=%u/0x%08x sonic_cr=%u/0x%08x "
                         "sonic_tcr=%u/0x%08x sonic_run=%u/0x%08x "
                         "sonic_imr=%u/0x%08x scsi_status=%u/0x%08x "
                         "scsi_mirror=%u/0x%08x scc_mirror=%u/0x%08x mirror_decode=%u "
                         "vram_resp=%u/0x%08x vram_byte_resp=%u/0x%08x "
                         "dafb_base_resp=%u/0x%08x dafb_stride_resp=%u/0x%08x "
                         "dafb_sense_resp=%u/0x%08x "
                         "xpram47=0x%02x xpramf8=0x%02x xpram78=0x%02x "
                         "xpramf9=0x%02x xpramfa=0x%02x xpramfb=0x%02x "
                         "xpram77=0x%02x xpram7b=0x%02x "
                         "asc_via2_irq_ok=%u via2_ifr=0x%02x irq=0x%02x\n",
                         r_via.resp, r_gap.resp, rtc_ok ? 1u : 0u,
                         r_enet.resp, r_enet.data,
                         r_sonic_cr.resp, r_sonic_cr.data,
                         r_sonic_tcr.resp, r_sonic_tcr.data,
                         r_sonic_cr_run.resp, r_sonic_cr_run.data,
                         r_sonic_imr.resp, r_sonic_imr.data,
                         r_scsi_status.resp, r_scsi_status.data,
                         r_scsi_status_mirror.resp, r_scsi_status_mirror.data,
                         r_scc_status_mirror.resp, r_scc_status_mirror.data,
                         mirror_decode_ok ? 1u : 0u,
                         r_vram.resp, r_vram.data,
                         r_vram_b.resp, r_vram_b.data,
                         r_dafb_base.resp, r_dafb_base.data,
                         r_dafb_stride.resp, r_dafb_stride.data,
                         r_dafb_sense.resp, r_dafb_sense.data,
                         xpram47, xpramf8, xpram78, xpramf9, xpramfa,
                         xpramfb, xpram77, xpram7b,
                         asc_via2_irq_ok ? 1u : 0u, via2_asc_ifr, via2_asc_irq);
            delete dut;
            close_vram_backing();
            return 1;
        }
        std::printf("mame_axi_periph_bridge selftest passed via=0x%08x enet=0x%02x sonic_cr=0x%04x sonic_tcr=0x%04x sonic_imr=0x%04x scsi=0x%08x scsi_mirror=0x%08x scc_mirror=0x%08x gap_resp=%u vram=0x%08x vram_byte=0x%08x dafb_base=0x%08x dafb_stride=0x%08x dafb_sense=0x%08x xpram47=0x%02x xpramf8=0x%02x xpram78=0x%02x xpramf9=0x%02x xpramfa=0x%02x xpramfb=0x%02x xpram77=0x%02x xpram7b=0x%02x asc_via2_ifr=0x%02x asc_via2_irq=0x%02x\n",
                    r_via.data,
                    unsigned(r_enet.data & 0xffu),
                    unsigned(r_sonic_cr.data & 0xffffu),
                    unsigned(r_sonic_tcr.data & 0xffffu),
                    unsigned(r_sonic_imr.data & 0xffffu),
                    r_scsi_status.data, r_scsi_status_mirror.data,
                    r_scc_status_mirror.data,
                    r_gap.resp, r_vram.data, r_vram_b.data, r_dafb_base.data,
                    r_dafb_stride.data, r_dafb_sense.data,
                    xpram47, xpramf8, xpram78,
                    xpramf9, xpramfa, xpramfb, xpram77, xpram7b,
                    via2_asc_ifr, via2_asc_irq);
        delete dut;
        close_vram_backing();
        return 0;
    }

    std::signal(SIGPIPE, SIG_IGN);
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    serve(socket_path);
    delete dut;
    close_vram_backing();
    return 0;
}
