#ifndef TB_COLD_BOOT_SUPPORT_H
#define TB_COLD_BOOT_SUPPORT_H

struct SdSource {
    std::vector<uint8_t> rsp_fifo;
    std::vector<uint8_t> mosi_frame;
    bool in_frame = false;
    bool cmd18_active = false;
    uint32_t cmd18_next_lba = 0;
    std::vector<std::vector<uint8_t>> sector_data;
    uint32_t eof_after_sector = 0xFFFFFFFFu;

    void reset(std::vector<std::vector<uint8_t>> sd, uint32_t eof = 0xFFFFFFFFu) {
        rsp_fifo.clear();
        mosi_frame.clear();
        in_frame = false;
        cmd18_active = false;
        cmd18_next_lba = 0;
        sector_data = std::move(sd);
        eof_after_sector = eof;
    }

    void push(const std::vector<uint8_t>& v) {
        for (uint8_t b : v) rsp_fifo.push_back(b);
    }

    void queue_block(uint32_t sec) {
        push({0xFF, 0xFE});
        for (int i = 0; i < 512; i++) rsp_fifo.push_back(sector_data.at(sec)[i]);
        push({0x00, 0x00});
    }

    uint8_t consume_one_byte() {
        if (rsp_fifo.empty() && cmd18_active) {
            if (cmd18_next_lba >= eof_after_sector ||
                cmd18_next_lba >= sector_data.size()) return 0xFF;
            queue_block(cmd18_next_lba++);
        }
        if (rsp_fifo.empty()) return 0xFF;
        uint8_t b = rsp_fifo.front();
        rsp_fifo.erase(rsp_fifo.begin());
        return b;
    }

    void handle_frame() {
        uint8_t cmd = mosi_frame[0] & 0x3F;
        switch (cmd) {
            case 0:  push({0xFF, 0x01}); break;
            case 8:  push({0xFF, 0x01, 0x00, 0x00, 0x01, 0xAA}); break;
            case 55: push({0xFF, 0x01}); break;
            case 41: push({0xFF, 0x00}); break;
            case 58: push({0xFF, 0x00, 0xC0, 0xFF, 0x80, 0x00}); break;
            case 6:  push({0xFF, 0x04}); break;
            case 18: {
                uint32_t sec = ((uint32_t)mosi_frame[1] << 24) |
                               ((uint32_t)mosi_frame[2] << 16) |
                               ((uint32_t)mosi_frame[3] <<  8) |
                               ((uint32_t)mosi_frame[4]);
                push({0xFF, 0x00});
                cmd18_active = true;
                cmd18_next_lba = sec;
                break;
            }
            case 12:
                cmd18_active = false;
                rsp_fifo.clear();
                push({0xFF, 0x00});
                break;
            default:
                push({0xFF, 0x04});
                break;
        }
    }

    void observe_mosi(uint8_t b) {
        if (!in_frame) {
            if ((b & 0xC0) == 0x40) {
                mosi_frame.clear();
                mosi_frame.push_back(b);
                in_frame = true;
            }
        } else {
            mosi_frame.push_back(b);
            if (mosi_frame.size() == 6) {
                handle_frame();
                in_frame = false;
            }
        }
    }
} sd;

struct SpiByteMux {
    bool have_rsp = false;
    uint8_t pending_rsp = 0xFF;
} mux;

struct BoundaryEvent {
    uint32_t seq = 0;
    uint8_t  kind = 0;
    uint8_t  vec = 0;
    uint32_t pc = 0;
    uint32_t next_pc = 0;
    uint32_t fault_pc = 0;
    uint32_t fault_addr = 0;
};

struct SentinelEvent {
    uint32_t value = 0;
    uint64_t cycle = 0;
};

static uint32_t last_boundary_seq = 0;

static void spi_tick() {
    dut->spi_cmd_ready = 1;
    if (mux.have_rsp) {
        dut->spi_rsp_valid = 1;
        dut->spi_rsp_data = mux.pending_rsp;
        mux.have_rsp = false;
    } else {
        dut->spi_rsp_valid = 0;
        dut->spi_rsp_data = 0;
    }
    if (dut->spi_cmd_valid && dut->spi_cmd_ready) {
        sd.observe_mosi(dut->spi_cmd_data);
        mux.pending_rsp = sd.consume_one_byte();
        mux.have_rsp = true;
    }
}

static void tick() {
    dut->clk = 0;
    dut->eval();
    dut->clk = 1;
    dut->eval();
    spi_tick();
    dut->eval();
    sim_time++;
}

static void reset(bool boot_pc_override = false, uint32_t override_pc = 0) {
    dut->rst = 1;
    dut->boot_pc_override_en = boot_pc_override;
    dut->boot_pc_override_val = override_pc;
    dut->dbg_pc_load_en = 0;
    dut->dbg_pc_load_val = 0;
    dut->vram_peek_en = 0;
    dut->vram_peek_addr = 0;
    dut->spi_cmd_ready = 0;
    dut->spi_rsp_valid = 0;
    dut->spi_rsp_data = 0;
    mux.have_rsp = false;
    mux.pending_rsp = 0xFF;
    for (int i = 0; i < 8; i++) tick();
    dut->rst = 0;
    tick();
    last_boundary_seq = dut->dbg_boundary_seq;
}

static uint8_t line_byte(const uint32_t* line, int idx) {
    const int word = idx >> 2;
    const int shift = 24 - 8 * (idx & 3);
    return (uint8_t)((line[word] >> shift) & 0xFFu);
}

static bool poll_boundary(BoundaryEvent& ev) {
    uint32_t seq = dut->dbg_boundary_seq;
    if (seq == last_boundary_seq) return false;
    last_boundary_seq = seq;
    ev.seq = seq;
    ev.kind = (uint8_t)dut->dbg_boundary_kind;
    ev.vec = (uint8_t)dut->dbg_boundary_exc_vec;
    ev.pc = dut->dbg_boundary_pc;
    ev.next_pc = dut->dbg_boundary_next_pc;
    ev.fault_pc = dut->dbg_boundary_fault_pc;
    ev.fault_addr = dut->dbg_boundary_fault_addr;
    return true;
}

#endif
