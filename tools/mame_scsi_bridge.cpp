// mame_scsi_bridge.cpp - UNIX-socket MAME bridge endpoint for rtl/mac/scsi.v.
//
// This is the first Verilator-backed endpoint for the MAME RTL bridge
// protocol.  It handles only the Q700 TurboSCSI window:
//   0x5000_f000..0x5000_f0ff NCR register aperture
//   0x5000_f100..0x5000_f101 pseudo-DMA shim
// plus the normal 0x50xx_xxxx Q700 mirror mask.
//
// Protocol format matches tools/mame_rtl_bridge_protocol.py:
//   request  >4sBBBBIII  magic/version/op/size/wstrb/addr/data/pc
//   response >4sBBHII    magic/version/resp/reserved/data/cycles

#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <verilated.h>

#include "Vtb_scsi_vhdd_sd.h"

namespace {

constexpr uint8_t OP_READ = 1;
constexpr uint8_t OP_WRITE = 2;
constexpr uint8_t RESP_OKAY = 0;
constexpr uint8_t RESP_DECERR = 3;
constexpr uint8_t RESP_TIMEOUT = 4;
constexpr uint32_t SCSI_BASE = 0x5000F000u;
constexpr uint32_t SCSI_END = 0x5000F102u;
constexpr uint32_t Q700_IO_MIRROR_MASK = 0x00FC0000u;
constexpr uint32_t MAX_WAIT_CYCLES = 64;

struct Request {
    uint8_t version = 0;
    uint8_t op = 0;
    uint8_t size = 0;
    uint8_t wstrb = 0;
    uint32_t addr = 0;
    uint32_t data = 0;
    uint32_t pc = 0;
};

struct Response {
    uint8_t resp = RESP_DECERR;
    uint32_t data = 0;
    uint32_t cycles = 0;
};

Vtb_scsi_vhdd_sd* dut = nullptr;
uint64_t sim_time = 0;

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

uint32_t canonical_addr(uint32_t addr) {
    if ((addr & 0xFF000000u) == 0x50000000u)
        return addr & ~Q700_IO_MIRROR_MASK;
    return addr;
}

bool scsi_addr_to_pb(uint32_t addr, uint16_t& pb_addr) {
    const uint32_t ca = canonical_addr(addr);
    if (ca < SCSI_BASE || ca >= SCSI_END)
        return false;
    const uint32_t off = ca - SCSI_BASE;
    if (off < 0x100u) {
        pb_addr = uint16_t((off >> 4) & 0x0fu);
        return true;
    }
    if (off == 0x100u || off == 0x101u) {
        pb_addr = uint16_t(off);
        return true;
    }
    return false;
}

bool read_exact(int fd, uint8_t* buf, size_t len) {
    size_t got = 0;
    while (got < len) {
        ssize_t n = ::read(fd, buf + got, len - got);
        if (n == 0)
            return false;
        if (n < 0) {
            if (errno == EINTR)
                continue;
            return false;
        }
        got += size_t(n);
    }
    return true;
}

bool write_exact(int fd, const uint8_t* buf, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = ::write(fd, buf + sent, len - sent);
        if (n < 0) {
            if (errno == EINTR)
                continue;
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
    if (req.version != 1)
        return false;
    if (req.op != OP_READ && req.op != OP_WRITE)
        return false;
    if (req.size != 1 && req.size != 2 && req.size != 4)
        return false;
    return true;
}

void encode_response(const Response& resp, uint8_t raw[16]) {
    std::memcpy(raw, "MRTB", 4);
    raw[4] = 1;
    raw[5] = resp.resp;
    raw[6] = 0;
    raw[7] = 0;
    put_be32(raw + 8, resp.data);
    put_be32(raw + 12, resp.cycles);
}

void tick() {
    dut->clk = 0;
    dut->eval();
    dut->clk = 1;
    dut->eval();
    sim_time++;
}

void reset_dut() {
    dut->rst = 1;
    dut->pb_addr = 0;
    dut->pb_wdata = 0;
    dut->pb_wr = 0;
    dut->pb_rd = 0;
    dut->sd_busy = 0;
    dut->sd_done = 0;
    dut->sd_error = 0;
    dut->sd_rd_valid = 0;
    dut->sd_rd_data = 0;
    dut->sd_wr_ready = 0;
    for (int i = 0; i < 4; i++)
        tick();
    dut->rst = 0;
    tick();
}

uint32_t lane_replicate(uint8_t byte, uint8_t size) {
    if (size == 1)
        return uint32_t(byte) * 0x01010101u;
    if (size == 2)
        return (uint32_t(byte) << 24) | (uint32_t(byte) << 16) |
               (uint32_t(byte) << 8) | uint32_t(byte);
    return uint32_t(byte) * 0x01010101u;
}

uint8_t write_byte_from_request(const Request& req) {
    const uint32_t lane = req.addr & 0x3u;
    const unsigned shift = (3u - lane) * 8u;
    return uint8_t(req.data >> shift);
}

Response transact_scsi(const Request& req) {
    uint16_t pb = 0;
    if (!scsi_addr_to_pb(req.addr, pb))
        return Response{RESP_DECERR, 0, 0};

    if (req.op == OP_WRITE) {
        dut->pb_addr = pb & 0x1ffu;
        dut->pb_wdata = write_byte_from_request(req);
        dut->pb_wr = 1;
        dut->pb_rd = 0;
        for (uint32_t i = 0; i < MAX_WAIT_CYCLES; i++) {
            tick();
            if (dut->pb_ack) {
                dut->pb_wr = 0;
                tick();
                return Response{RESP_OKAY, 0, i + 1};
            }
        }
        dut->pb_wr = 0;
        return Response{RESP_TIMEOUT, 0, MAX_WAIT_CYCLES};
    }

    dut->pb_addr = pb & 0x1ffu;
    dut->pb_wdata = 0;
    dut->pb_wr = 0;
    dut->pb_rd = 1;
    for (uint32_t i = 0; i < MAX_WAIT_CYCLES; i++) {
        tick();
        if (dut->pb_ack) {
            const uint8_t value = dut->pb_rdata & 0xffu;
            dut->pb_rd = 0;
            tick();
            return Response{RESP_OKAY, lane_replicate(value, req.size), i + 1};
        }
    }
    dut->pb_rd = 0;
    return Response{RESP_TIMEOUT, 0, MAX_WAIT_CYCLES};
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

    std::printf("mame SCSI RTL bridge listening on %s\n", path.c_str());
    std::fflush(stdout);

    while (true) {
        int conn = ::accept(srv, nullptr, nullptr);
        if (conn < 0) {
            if (errno == EINTR)
                continue;
            std::perror("accept");
            break;
        }

        uint8_t req_raw[20];
        while (read_exact(conn, req_raw, sizeof(req_raw))) {
            Request req;
            Response resp;
            if (!decode_request(req_raw, req)) {
                resp = Response{RESP_DECERR, 0, 0};
            } else {
                resp = transact_scsi(req);
            }
            uint8_t resp_raw[16];
            encode_response(resp, resp_raw);
            if (!write_exact(conn, resp_raw, sizeof(resp_raw)))
                break;
        }
        ::close(conn);
    }

    ::close(srv);
}

} // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    std::string socket_path = "/tmp/mame-scsi-bridge.sock";
    bool selftest = false;
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--socket") == 0 && i + 1 < argc) {
            socket_path = argv[++i];
        } else if (std::strcmp(argv[i], "--selftest") == 0) {
            selftest = true;
        } else {
            std::fprintf(stderr, "usage: %s [--socket PATH] [--selftest]\n", argv[0]);
            return 2;
        }
    }

    dut = new Vtb_scsi_vhdd_sd;
    reset_dut();

    if (selftest) {
        Request req{};
        req.version = 1;
        req.op = OP_READ;
        req.size = 1;
        req.addr = 0x50F0F040u;
        Response resp = transact_scsi(req);
        if (resp.resp != RESP_OKAY || resp.data != 0x00000000u) {
            std::fprintf(stderr,
                         "selftest failed resp=%u data=0x%08x\n",
                         resp.resp, resp.data);
            delete dut;
            return 1;
        }
        std::printf("mame_scsi_bridge selftest passed data=0x%08x cycles=%u\n",
                    resp.data, resp.cycles);
        delete dut;
        return 0;
    }

    std::signal(SIGPIPE, SIG_IGN);
    serve(socket_path);
    delete dut;
    return 0;
}
