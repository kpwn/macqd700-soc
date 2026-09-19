// Descriptor-level test for q700_sonic_tx.
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <vector>
#include <verilated.h>
#include "Vq700_sonic_tx.h"

struct Pending {
    uint32_t addr;
    uint8_t len;
    uint8_t tag;
    bool write;
    uint32_t delay;
};

static Vq700_sonic_tx *dut;
static std::vector<uint8_t> memory(1u << 16);
static std::vector<Pending> pending;
static std::vector<uint8_t> frame;
static std::vector<uint64_t> payload_issue_cycle;
static uint64_t cycles;

static void clear_wide(WData *v) {
    for (int i = 0; i < 16; ++i) v[i] = 0;
}

static void put_word(uint32_t addr, uint16_t value, bool wide) {
    if (wide) {
        memory.at(addr + 0) = 0;
        memory.at(addr + 1) = 0;
        memory.at(addr + 2) = uint8_t(value >> 8);
        memory.at(addr + 3) = uint8_t(value);
    } else {
        memory.at(addr + 0) = uint8_t(value >> 8);
        memory.at(addr + 1) = uint8_t(value);
    }
}

static bool status_is_zero(uint32_t addr, bool wide) {
    const unsigned n = wide ? 4 : 2;
    for (unsigned i = 0; i < n; ++i)
        if (memory.at(addr + i) != 0) return false;
    return true;
}

static void drive_response() {
    dut->dma_rsp_valid = 0;
    dut->dma_rsp_write = 0;
    dut->dma_rsp_status = 0;
    dut->dma_rsp_tag = 0;
    dut->dma_rsp_len = 0;
    clear_wide(dut->dma_rsp_rdata);
    for (auto &p : pending) if (p.delay) --p.delay;
    int chosen = -1;
    // Highest ready tag first deliberately returns payload out of order.
    for (int i = 0; i < int(pending.size()); ++i)
        if (!pending[i].delay &&
            (chosen < 0 || pending[i].tag > pending[chosen].tag)) chosen = i;
    if (chosen < 0) return;
    const Pending &p = pending[chosen];
    dut->dma_rsp_valid = 1;
    dut->dma_rsp_write = p.write;
    dut->dma_rsp_tag = p.tag;
    dut->dma_rsp_len = p.len;
    if (!p.write) {
        for (unsigned i = 0; i < p.len; ++i)
            dut->dma_rsp_rdata[i / 4] |= uint32_t(memory.at(p.addr + i)) << ((i % 4) * 8);
    }
}

static void accept_request() {
    if (!(dut->dma_req_valid && dut->dma_req_ready)) return;
    Pending p{uint32_t(dut->dma_req_addr), uint8_t(dut->dma_req_len),
              uint8_t(dut->dma_req_tag), bool(dut->dma_req_write),
              dut->dma_req_tag < 32 ? uint32_t(8 + (dut->dma_req_tag % 3)) : 2u};
    if (!p.len || p.len > 64) {
        std::printf("FAIL: illegal DMA length %u\n", p.len);
        std::exit(2);
    }
    if (p.tag < 32) payload_issue_cycle.push_back(cycles);
    if (p.write) {
        for (unsigned i = 0; i < p.len; ++i)
            memory.at(p.addr + i) = uint8_t(dut->dma_req_wdata[i / 4] >> ((i % 4) * 8));
    }
    pending.push_back(p);
}

static void cycle() {
    dut->clk = 0;
    dut->dma_req_ready = 1;
    dut->tx_axis_tready = (cycles % 5) != 2;
    drive_response();
    dut->eval();
    accept_request();
    if (dut->tx_axis_tvalid && dut->tx_axis_tready)
        frame.push_back(uint8_t(dut->tx_axis_tdata));
    const bool consumed_rsp = dut->dma_rsp_valid && dut->dma_rsp_ready;
    uint8_t consumed_tag = dut->dma_rsp_tag;
    dut->clk = 1;
    dut->eval();
    if (consumed_rsp) {
        auto it = std::find_if(pending.begin(), pending.end(),
            [consumed_tag](const Pending &p) { return p.tag == consumed_tag && p.delay == 0; });
        if (it != pending.end()) pending.erase(it);
    }
    ++cycles;
}

static bool run_case(bool wide) {
    std::fill(memory.begin(), memory.end(), 0);
    pending.clear(); frame.clear(); payload_issue_cycle.clear();
    const uint32_t tda = wide ? 0x1400 : 0x1000;
    const uint32_t next_tda = tda + 0x100;
    const unsigned wb = wide ? 4 : 2;
    const uint32_t frag0 = 0x2003;
    const uint32_t frag1 = 0x3101;
    put_word(tda + 0*wb, 0, wide);
    put_word(tda + 1*wb, 0x2000, wide);  // TCR descriptor control (CRCI)
    put_word(tda + 2*wb, 0x1234, wide);  // TPS
    put_word(tda + 3*wb, 2, wide);       // TFC: two fragments
    put_word(tda + 4*wb, uint16_t(frag0), wide);
    put_word(tda + 5*wb, 0, wide);
    put_word(tda + 6*wb, 70, wide);
    put_word(tda + 7*wb, uint16_t(frag1), wide);
    put_word(tda + 8*wb, 0, wide);
    put_word(tda + 9*wb, 5, wide);
    put_word(tda + 10*wb, uint16_t(next_tda) | 1, wide); // next + EOL
    std::vector<uint8_t> expected;
    for (unsigned i = 0; i < 70; ++i) {
        uint8_t b = uint8_t(0x20 + i);
        memory.at(frag0 + i) = b; expected.push_back(b);
    }
    for (unsigned i = 0; i < 5; ++i) {
        uint8_t b = uint8_t(0xd0 + i);
        memory.at(frag1 + i) = b; expected.push_back(b);
    }

    dut->rst = 1; dut->start_valid = 0; dut->done_ready = 0;
    dut->tx_cpl_valid = 0;
    for (int i = 0; i < 3; ++i) cycle();
    dut->rst = 0;
    dut->start_dcr = wide ? 0x20 : 0;
    dut->start_utda = uint16_t(tda >> 16);
    dut->start_ctda = uint16_t(tda);
    dut->start_valid = 1;
    while (!dut->start_ready) cycle();
    cycle();
    dut->start_valid = 0;

    unsigned timeout = 3000;
    while (frame.size() != expected.size() && timeout--) cycle();
    if (frame != expected) {
        std::printf("FAIL %s-bit descriptor: frame size/data mismatch (%zu/%zu)\n",
                    wide ? "32" : "16", frame.size(), expected.size());
        return false;
    }
    if (!status_is_zero(tda, wide)) {
        std::printf("FAIL: descriptor completed before MAC completion\n");
        return false;
    }
    if (payload_issue_cycle.size() != 3 ||
        payload_issue_cycle[1] != payload_issue_cycle[0] + 1 ||
        payload_issue_cycle[2] != payload_issue_cycle[1] + 1) {
        std::printf("FAIL: payload DMA requests were not accepted on consecutive cycles\n");
        return false;
    }
    for (int i = 0; i < 5; ++i) cycle();
    if (!status_is_zero(tda, wide)) return false;
    dut->tx_cpl_valid = 1;
    while (!dut->tx_cpl_ready) cycle();
    cycle(); dut->tx_cpl_valid = 0;
    timeout = 1000;
    while (!dut->done_valid && timeout--) cycle();
    // done_tcr is 0x2001, not 0x0001: the descriptor's config half (CRCI,
    // 0x2000, written at the TDA above) is PRESERVED into TCR and PTX is ORed
    // in.  This assertion previously demanded 0x0001, which encoded the bug --
    // `& 16'h07ff` was erasing the config half, so TCR read back 0x0001
    // forever and PINT/POWC/CRCI/EXDIS never appeared.  ds:1899-1900 loads
    // TXpkt.config bits 15-12 into TCR; mame.cpp:369+427 keeps them.
    if (!dut->done_valid || dut->done_error || dut->done_tcr != 0x2001 ||
        dut->done_ctda != (uint16_t(next_tda) | 1) ||
        dut->done_tps != 0x1234 || dut->done_tfc != 2) {
        std::printf("FAIL: bad completion status\n");
        return false;
    }
    const unsigned status_pos = wide ? 2 : 0;
    if (memory[tda + status_pos] != 0 || memory[tda + status_pos + 1] != 1) {
        std::printf("FAIL: descriptor PTX writeback missing\n");
        return false;
    }
    dut->done_ready = 1; cycle(); dut->done_ready = 0;

    // A driver appends a packet by clearing the predecessor's EOL bit and
    // issuing TXP again.  CTDA still contains the link value it loaded at
    // completion, including bit zero; the engine must mask that flag when it
    // forms the next descriptor address.
    const uint32_t frag2 = 0x3605;
    const uint16_t after_next = uint16_t(next_tda + 0x100);
    put_word(tda + 10*wb, uint16_t(next_tda), wide);
    put_word(next_tda + 0*wb, 0, wide);
    put_word(next_tda + 1*wb, 0, wide);
    put_word(next_tda + 2*wb, 9, wide);
    put_word(next_tda + 3*wb, 1, wide);
    put_word(next_tda + 4*wb, uint16_t(frag2), wide);
    put_word(next_tda + 5*wb, 0, wide);
    put_word(next_tda + 6*wb, 9, wide);
    put_word(next_tda + 7*wb, after_next | 1, wide);
    frame.clear(); payload_issue_cycle.clear();
    std::vector<uint8_t> expected2;
    for (unsigned i = 0; i < 9; ++i) {
        uint8_t b = uint8_t(0xa8 + i);
        memory.at(frag2 + i) = b; expected2.push_back(b);
    }
    dut->start_ctda = uint16_t(next_tda) | 1;
    dut->start_valid = 1;
    while (!dut->start_ready) cycle();
    cycle(); dut->start_valid = 0;
    timeout = 1000;
    while (frame.size() != expected2.size() && timeout--) cycle();
    if (frame != expected2) {
        std::printf("FAIL: EOL-tagged CTDA did not address the next descriptor "
                    "(frame=%zu pending=%zu timeout=%u)\n",
                    frame.size(), pending.size(), timeout);
        return false;
    }
    dut->tx_cpl_valid = 1;
    while (!dut->tx_cpl_ready) cycle();
    cycle(); dut->tx_cpl_valid = 0;
    timeout = 1000;
    while (!dut->done_valid && timeout--) cycle();
    if (!dut->done_valid || dut->done_error ||
        dut->done_ctda != (after_next | 1)) {
        std::printf("FAIL: linked CTDA value was not preserved at EOL\n");
        return false;
    }
    dut->done_ready = 1; cycle(); dut->done_ready = 0;

    // ── HTX must actually halt the engine ─────────────────────────────
    // ds:1656-1659: HTX "halts the transmit command after the current
    // transmission has completed... sampled after writing to the TXpkt.status
    // field".  ds:1224-1227: on halt "the CTDA register is NOT loaded".
    // Before this, HTX cleared the CR bit and nothing else -- the engine kept
    // walking the list to end-of-list.  Point the descriptor at a live link
    // (no EOL) so a NON-halting engine would visibly advance CTDA to it.
    put_word(tda + 10*wb, uint16_t(next_tda), wide);   // link, EOL clear
    dut->halt = 1;
    dut->start_ctda = uint16_t(tda);
    dut->start_valid = 1;
    while (!dut->start_ready) cycle();
    cycle();
    dut->start_valid = 0;
    // The engine is completion-gated: it will not write TXpkt.status (and so
    // will not reach the halt sample point) until the MAC acknowledges the
    // frame, exactly as in the transmits above.
    timeout = 20000;
    while (!dut->tx_cpl_ready && timeout--) cycle();
    dut->tx_cpl_valid = 1;
    while (!dut->tx_cpl_ready) cycle();
    cycle(); dut->tx_cpl_valid = 0;
    timeout = 20000;
    while (!dut->done_valid && timeout--) cycle();
    if (!dut->done_valid) {
        std::printf("FAIL: halted transmit never completed\n");
        return false;
    }
    if (dut->done_ctda != uint16_t(tda)) {
        std::printf("FAIL: HTX still advanced CTDA to 0x%04x (want 0x%04x)\n",
                    dut->done_ctda, uint16_t(tda));
        return false;
    }
    dut->done_ready = 1; cycle(); dut->done_ready = 0;
    dut->halt = 0;

    std::printf("PASS q700_sonic_tx %s-bit descriptors: unaligned 70+5 byte fragments, "
                "64/6/5 byte pipelined DMA, ordered MAC frame, completion-gated writeback, "
                "EOL-tagged CTDA restart\n",
                wide ? "32" : "16");
    return true;
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vq700_sonic_tx;
    bool ok = run_case(false) && run_case(true);
    delete dut;
    return ok ? 0 : 1;
}
