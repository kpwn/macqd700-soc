// tb_q700_eth_sonic.cpp - Verilator unit testbench for q700_eth_sonic.v.
//
// Exercises the Q700 Ethernet PROM bytes plus the reset/config-visible
// DP83932C SONIC register block used by the FPGA and MAME AXI bridge tops.

#include <cstdint>
#include <cstdio>
#include <verilated.h>
#include "Vq700_eth_sonic.h"

static Vq700_eth_sonic* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0;
static int n_fail = 0;
static bool helper_ok = true;

static void tick() {
    dut->clk = 0;
    dut->eval();
    dut->clk = 1;
    dut->eval();
    sim_time++;
}

static void reset() {
    dut->rst = 1;
    dut->enet_cs = 0;
    dut->enet_rd = 0;
    dut->enet_wr = 0;
    dut->enet_addr = 0;
    dut->enet_wdata = 0;
    dut->sonic_cs = 0;
    dut->sonic_rd = 0;
    dut->sonic_wr = 0;
    dut->sonic_addr = 0;
    dut->sonic_wdata = 0;
    dut->sonic_wstrb = 0;
    dut->tx_cmd_ready = 0;
    dut->tx_done_valid = 0;
    dut->tx_done_error = 0;
    dut->tx_done_ctda = 0; dut->tx_done_pint = 0;
    dut->tx_done_tcr = 0;
    dut->tx_done_tps = 0;
    dut->tx_done_tfc = 0;
    dut->rx_cfg_ready = 0;
    dut->rx_done_valid = 0; dut->rx_done_error = 0;
    dut->rx_done_rcr = 0; dut->rx_done_crda = 0;
    dut->rx_done_crba0 = 0; dut->rx_done_crba1 = 0;
    dut->rx_done_rbwc0 = 0; dut->rx_done_rbwc1 = 0;
    dut->rx_done_rrp = 0; dut->rx_done_rsc = 0; dut->rx_done_llfa = 0;
    dut->rx_done_trba0 = 0; dut->rx_done_trba1 = 0;
    dut->rx_done_tbwc0 = 0; dut->rx_done_tbwc1 = 0; dut->rx_done_isr_set = 0;
    dut->rx_done_cdp = 0; dut->rx_done_cdc = 0; dut->rx_done_ce = 0;
    helper_ok = true;
    tick();
    tick();
    dut->rst = 0;
    tick();
}

#define CHECK_EQ(name, got, exp) do { \
    if ((uint64_t)(got) != (uint64_t)(exp)) { \
        std::printf("  FAIL %s: got 0x%llx expected 0x%llx\n", \
                    name, (unsigned long long)(got), \
                    (unsigned long long)(exp)); \
        return false; \
    } \
} while (0)

#define CHECK_TRUE(name, cond) do { \
    if (!(cond)) { \
        std::printf("  FAIL %s: condition false\n", name); \
        return false; \
    } \
} while (0)

static uint8_t read_enet(uint8_t addr) {
    dut->enet_cs = 1;
    dut->enet_rd = 1;
    dut->enet_wr = 0;
    dut->enet_addr = addr & 7;
    dut->eval();
    const uint8_t value = dut->enet_rdata;
    if (!dut->enet_ack) {
        std::printf("  FAIL ENET ack missing\n");
        helper_ok = false;
    }
    tick();
    dut->enet_cs = 0;
    dut->enet_rd = 0;
    dut->eval();
    return value;
}

static void write_enet(uint8_t addr, uint8_t value) {
    dut->enet_cs = 1;
    dut->enet_rd = 0;
    dut->enet_wr = 1;
    dut->enet_addr = addr & 7;
    dut->enet_wdata = value;
    dut->eval();
    tick();
    dut->enet_cs = 0;
    dut->enet_wr = 0;
    dut->eval();
}

static uint16_t sonic_physical_addr(uint16_t packed_addr) {
    return uint16_t(((packed_addr & ~uint16_t(1)) << 1) | 2 |
                    (packed_addr & 1));
}

static uint8_t read_sonic_physical_byte(uint16_t addr) {
    if ((addr & 2u) == 0)
        return 0xff;
    dut->sonic_cs = 1;
    dut->sonic_rd = 1;
    dut->sonic_wr = 0;
    dut->sonic_addr = (addr >> 2) & 0x3f;
    dut->eval();
    const uint8_t value = (addr & 1u) ? uint8_t(dut->sonic_rdata) :
                                       uint8_t(dut->sonic_rdata >> 8);
    if (!dut->sonic_ack) {
        std::printf("  FAIL SONIC read ack missing\n");
        helper_ok = false;
    }
    tick();
    dut->sonic_cs = 0;
    dut->sonic_rd = 0;
    dut->eval();
    return value;
}

static uint8_t read_sonic_byte(uint16_t addr) {
    return read_sonic_physical_byte(sonic_physical_addr(addr));
}

static uint16_t read_sonic_word(uint16_t addr) {
    return (uint16_t(read_sonic_byte(addr)) << 8) |
           uint16_t(read_sonic_byte(addr + 1));
}

static void write_sonic_physical_byte(uint16_t addr, uint8_t value) {
    dut->sonic_cs = 1;
    dut->sonic_rd = 0;
    dut->sonic_wr = 1;
    dut->sonic_addr = (addr >> 2) & 0x3f;
    dut->sonic_wdata = (addr & 1u) ? uint16_t(value) : uint16_t(value) << 8;
    dut->sonic_wstrb = ((addr & 2u) == 0) ? 0 : ((addr & 1u) ? 1 : 2);
    dut->eval();
    if (!dut->sonic_ack) {
        std::printf("  FAIL SONIC write ack missing\n");
        helper_ok = false;
    }
    tick();
    dut->sonic_cs = 0;
    dut->sonic_wr = 0;
    dut->sonic_wstrb = 0;
    dut->eval();
}

static void write_sonic_byte(uint16_t addr, uint8_t value) {
    write_sonic_physical_byte(sonic_physical_addr(addr), value);
}

static void write_sonic_word(uint16_t addr, uint16_t value) {
    write_sonic_byte(addr, uint8_t(value >> 8));
    write_sonic_byte(addr + 1, uint8_t(value));
}

// A true native 16-bit register write (both byte strobes hot), as opposed to
// write_sonic_word() which issues two separate byte writes.  The two take
// different arms inside the RTL, so a driver using MOVE.W exercises a path the
// byte-pair helper never reaches.
static void write_sonic_native_word(uint8_t reg_index, uint16_t value) {
    dut->sonic_cs = 1;
    dut->sonic_rd = 0;
    dut->sonic_wr = 1;
    dut->sonic_addr = reg_index & 0x3f;
    dut->sonic_wdata = value;
    dut->sonic_wstrb = 3;
    dut->eval();
    if (!dut->sonic_ack) {
        std::printf("  FAIL SONIC native write ack missing\n");
        helper_ok = false;
    }
    tick();
    dut->sonic_cs = 0;
    dut->sonic_wr = 0;
    dut->sonic_wstrb = 0;
    tick();
}

static bool test_enet_prom() {
    reset();
    CHECK_EQ("deselected ENET", dut->enet_rdata, 0xff);
    CHECK_EQ("deselected SONIC", dut->sonic_rdata, 0xffff);
    CHECK_EQ("no reset IRQ", dut->sonic_irq, 0);

    const uint8_t expected[8] = {0x00, 0xa0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x1f};
    for (uint8_t i = 0; i < 8; i++) {
        CHECK_EQ("ENET PROM byte", read_enet(i), expected[i]);
    }
    write_enet(0, 0xff);
    CHECK_EQ("ENET PROM ignores writes", read_enet(0), 0x00);
    CHECK_TRUE("ENET helpers saw acks", helper_ok);
    return true;
}

static bool test_sonic_reset_values() {
    reset();
    CHECK_EQ("CR reset", read_sonic_word(0x00), 0x0094);
    CHECK_EQ("TCR reset", read_sonic_word(0x06), 0x0101);
    CHECK_EQ("IMR reset", read_sonic_word(0x08), 0x0000);
    CHECK_EQ("ISR reset", read_sonic_word(0x0a), 0x0000);
    CHECK_EQ("EOBC reset", read_sonic_word(0x26), 0x02f8);
    CHECK_EQ("SR reset", read_sonic_word(0x50), 0x0006);
    CHECK_TRUE("SONIC reset helpers saw acks", helper_ok);
    return true;
}

static bool test_mame_bus_layout() {
    reset();

    CHECK_EQ("CR upper lane byte 0 is unconnected",
             read_sonic_physical_byte(0x00), 0xff);
    CHECK_EQ("CR upper lane byte 1 is unconnected",
             read_sonic_physical_byte(0x01), 0xff);
    CHECK_EQ("CR high byte at +2", read_sonic_physical_byte(0x02), 0x00);
    CHECK_EQ("CR low byte at +3", read_sonic_physical_byte(0x03), 0x94);
    CHECK_EQ("SR high byte at register*4+2",
             read_sonic_physical_byte(0xa2), 0x00);
    CHECK_EQ("SR low byte at register*4+3",
             read_sonic_physical_byte(0xa3), 0x06);

    write_sonic_physical_byte(0x00, 0xff);
    write_sonic_physical_byte(0x01, 0xff);
    CHECK_EQ("upper-lane writes ignored", read_sonic_word(0x00), 0x0094);

    // Exact byte sequence emitted for System 7's MOVE.L #4,(SONIC_CR).
    write_sonic_physical_byte(0x00, 0x00);
    write_sonic_physical_byte(0x01, 0x00);
    write_sonic_physical_byte(0x02, 0x00);
    write_sonic_physical_byte(0x03, 0x04);
    CHECK_EQ("long write reaches CR low halfword", read_sonic_word(0x00), 0x0014);
    CHECK_EQ("long write does not spill into DCR", read_sonic_word(0x02), 0x0000);

    write_sonic_physical_byte(0x96, 0x12);
    write_sonic_physical_byte(0x97, 0x34);
    CHECK_EQ("CE resides at register*4+2", read_sonic_word(0x4a), 0x1234);
    CHECK_TRUE("physical-layout helpers saw acks", helper_ok);
    return true;
}

static bool test_sonic_masks_and_commands() {
    reset();

    write_sonic_byte(0x01, 0x00);
    CHECK_EQ("CR exits reset", read_sonic_word(0x00), 0x0014);

    write_sonic_word(0x08, 0x0200);
    CHECK_EQ("IMR high-byte write", read_sonic_word(0x08), 0x0200);

    write_sonic_byte(0x01, 0x02);
    CHECK_EQ("TX completion ISR", read_sonic_word(0x0a), 0x0200);
    CHECK_EQ("TX interrupt output", dut->sonic_irq, 1);

    write_sonic_byte(0x0a, 0x02);
    CHECK_EQ("ISR write-one-clear", read_sonic_word(0x0a), 0x0000);
    CHECK_EQ("IRQ clears with ISR", dut->sonic_irq, 0);

    write_sonic_byte(0x05, 0xff);
    CHECK_EQ("RCR low byte masked", read_sonic_word(0x04), 0x0000);
    write_sonic_byte(0x04, 0xfe);
    CHECK_EQ("RCR high byte writable", read_sonic_word(0x04), 0xfe00);

    write_sonic_byte(0x00, 0x02);
    CHECK_EQ("LCAM command sets LCD", read_sonic_word(0x0a), 0x1000);

    // A SOFTWARE reset (CR_RST) touches CR and nothing else.  These two
    // assertions previously demanded the opposite, and in doing so encoded the
    // live hardware bug as intended behaviour: IMR is programmed at line ~261
    // and a software reset silently zeroed it, so no interrupt could ever
    // assert again and delivered frames were never serviced.  MAME splits the
    // two resets -- device_reset() clears IMR/ISR/CE/RSC/DCR2, while the
    // CR_RST arm of reg_w() does `m_reg[CR] &= ~(CR_LCAM|CR_RRRA|CR_TXP|
    // CR_HTX); m_reg[CR] |= CR_RST|CR_RXDIS;` and touches no other register.
    write_sonic_byte(0x01, 0x80);
    CHECK_EQ("CR enters reset", read_sonic_word(0x00), 0x0094);
    CHECK_EQ("software reset preserves IMR", read_sonic_word(0x08), 0x0200);
    CHECK_EQ("software reset preserves ISR", read_sonic_word(0x0a), 0x1000);

    // ...and a POWER-ON reset still does clear them, so the split is real
    // rather than the clears having been dropped altogether.
    reset();
    CHECK_EQ("power-on reset clears IMR", read_sonic_word(0x08), 0x0000);
    CHECK_EQ("power-on reset clears ISR", read_sonic_word(0x0a), 0x0000);

    // MAME's command(): TXP clears HTX and HTX clears TXP.  Only the second
    // direction existed here, so once a driver issued HTX to abort a transmit,
    // bit 0 stayed set in every subsequent CR readback.
    write_sonic_byte(0x01, 0x00);           // leave reset
    write_sonic_byte(0x01, 0x01);           // HTX
    CHECK_EQ("HTX sets, TXP clear", read_sonic_word(0x00) & 0x0003, 0x0001);
    // With PACKET_ENGINE=0 the stub transmit completes in the same write, so
    // TXP self-clears and only the HTX side is observable here.  The opposite
    // direction (HTX clearing a TXP that is genuinely still in flight) needs
    // the real engine and is checked in test_packet_engine_handshake().
    write_sonic_byte(0x01, 0x02);           // TXP
    CHECK_EQ("TXP clears HTX", read_sonic_word(0x00) & 0x0001, 0x0000);

    // ── a software reset must stop the RECEIVER, not just commands ────
    // CR_RST deliberately leaves RXEN set, so gating the RX datapath on RXEN
    // alone left the engine accepting frames and DMAing them against pointers
    // the driver had already disowned -- the same failure class as the
    // 65535-RX-DMA-op storm at a garbage CRDA seen on hardware.
    write_sonic_byte(0x01, 0x00);            // leave reset
    write_sonic_byte(0x01, 0x08);            // CR_RXEN
    CHECK_EQ("receiver enabled", dut->rx_enabled, 1);
    write_sonic_byte(0x01, 0x80);            // CR_RST
    CHECK_EQ("RXEN survives a software reset", read_sonic_word(0x00) & 0x0008, 0x0008);
    CHECK_EQ("but the receiver is GATED during reset", dut->rx_enabled, 0);
    write_sonic_byte(0x01, 0x00);            // leave reset
    CHECK_EQ("receiver resumes when reset is released", dut->rx_enabled, 1);

    // ── the CR high-BYTE path must honour the RST gate too ────────────
    // ds:1578-1580: no command may be issued while RST is set.  The 16-bit
    // path checked this; the high-byte path did not, so LCAM/RRRA could be
    // armed mid-reset through a byte write.
    write_sonic_byte(0x01, 0x80);            // enter reset
    {
        const uint16_t before = read_sonic_word(0x00);
        write_sonic_byte(0x00, 0x02);        // LCAM via the HIGH byte
        CHECK_EQ("high-byte CR write is ignored during reset",
                 read_sonic_word(0x00), before);
    }
    write_sonic_byte(0x01, 0x00);

    CHECK_TRUE("SONIC command helpers saw acks", helper_ok);
    return true;
}

#ifdef PACKET_ENGINE_TEST
static bool test_packet_engine_handshake() {
    reset();
    write_sonic_byte(0x01, 0x00); // leave reset
    write_sonic_word(0x02, 0x0020); // DCR: 32-bit descriptors
    write_sonic_word(0x0c, 0x1234); // UTDA
    write_sonic_word(0x0e, 0x5678); // CTDA
    write_sonic_byte(0x01, 0x02);   // TXP
    CHECK_EQ("TX command held valid", dut->tx_cmd_valid, 1);
    CHECK_EQ("TX command DCR", dut->tx_cmd_dcr, 0x0020);
    CHECK_EQ("TX command UTDA", dut->tx_cmd_utda, 0x1234);
    CHECK_EQ("TX command CTDA", dut->tx_cmd_ctda, 0x5678);
    CHECK_EQ("TXP remains active", read_sonic_word(0x00), 0x0016);
    CHECK_EQ("no premature TXDN", read_sonic_word(0x0a), 0x0000);
    dut->tx_cmd_ready = 1; tick(); dut->tx_cmd_ready = 0;
    CHECK_EQ("TX command handshake", dut->tx_cmd_valid, 0);

    dut->tx_done_ctda = 0x5678;
    dut->tx_done_tcr = 0x0001;
    dut->tx_done_tps = 0x0102;
    dut->tx_done_tfc = 0x004b;
    dut->tx_done_error = 0;
    dut->tx_done_valid = 1;
    tick();
    dut->tx_done_valid = 0;
    CHECK_EQ("TXP clears on real completion", read_sonic_word(0x00), 0x0014);
    CHECK_EQ("TXDN follows real completion", read_sonic_word(0x0a), 0x0200);
    CHECK_EQ("TCR completion", read_sonic_word(0x06), 0x0001);
    CHECK_EQ("TPS completion", read_sonic_word(0x10), 0x0102);

    // TCR_PINT (0x8000) in a descriptor asks for an interrupt on THAT frame.
    // MAME raises ISR_PINT alongside the normal completion bit, not instead of
    // it.  The request cannot ride in done_tcr: descriptor_tcr keeps only the
    // config half (& 0xF000) and done_tcr is masked to the status half
    // (& 0x07FF), so it travels as its own flag.
    CHECK_EQ("ISR holds TXDN before PINT", read_sonic_word(0x0a), 0x0200);
    dut->tx_done_pint = 1;
    dut->tx_done_valid = 1;
    tick();
    dut->tx_done_valid = 0;
    dut->tx_done_pint = 0;
    CHECK_EQ("PINT raises ISR_PINT with TXDN", read_sonic_word(0x0a), 0x0A00);
    // Clear ONLY PINT, so the ISR is left exactly as the surrounding flow
    // expects (TXDN still pending for the later LCD check).
    write_sonic_word(0x0a, 0x0800);
    CHECK_EQ("PINT clears write-one, TXDN survives", read_sonic_word(0x0a), 0x0200);
    CHECK_EQ("TFC completion", read_sonic_word(0x12), 0x004b);

    // TXER is 0x0100.  It was encoded 0x0400 -- which is PKTRX, the bit
    // q700_sonic_rx.sv:70 already owns by that name -- so a transmit ERROR
    // raised "packet received": the driver would go hunting in the RX ring,
    // find nothing, and never learn the transmit failed, while TXER itself
    // could never set at all.  Pin BOTH halves: that the error bit is
    // 0x0100, and that it is NOT the bit the receive path owns.
    write_sonic_word(0x0a, 0x0200);              // clear TXDN
    dut->tx_done_error = 1; dut->tx_done_valid = 1;
    tick();
    dut->tx_done_valid = 0; dut->tx_done_error = 0;
    CHECK_EQ("TX error raises TXER (0x0100)", read_sonic_word(0x0a) & 0x0100, 0x0100);
    CHECK_EQ("TX error does NOT raise PKTRX (0x0400)",
             read_sonic_word(0x0a) & 0x0400, 0x0000);
    write_sonic_word(0x0a, 0x0100);              // clear TXER
    CHECK_EQ("TXER clears write-one", read_sonic_word(0x0a) & 0x0100, 0x0000);
    // Restore the ISR the following flow expects: TXDN pending, nothing else.
    dut->tx_done_valid = 1; tick(); dut->tx_done_valid = 0;
    CHECK_EQ("ISR restored to TXDN for the RRRA flow",
             read_sonic_word(0x0a), 0x0200);

    // ── PINT is EDGE-triggered across descriptors, not level ──────────
    // ds:1930-1936: PINT "must be cleared before it is set again in order to
    // have the interrupt issued for another packet".  A driver that leaves
    // PINT set in consecutive TDAs must get ONE interrupt, not one per packet.
    // This only works because TCR now retains its config half between
    // transmits -- with it erased, no edge could ever be detected and PINT
    // fired on every descriptor.
    write_sonic_word(0x0a, 0x0200);            // clear TXDN
    dut->tx_done_tcr = 0x8000;                 // this descriptor requests PINT
    dut->tx_done_pint = 1;
    dut->tx_done_valid = 1; tick(); dut->tx_done_valid = 0;
    CHECK_EQ("first PINT descriptor raises ISR_PINT",
             read_sonic_word(0x0a) & 0x0800, 0x0800);
    CHECK_EQ("TCR retains its config half", read_sonic_word(0x06) & 0x8000, 0x8000);
    write_sonic_word(0x0a, 0x0A00);            // clear PINT and TXDN
    dut->tx_done_valid = 1; tick(); dut->tx_done_valid = 0;
    CHECK_EQ("a SECOND consecutive PINT descriptor does NOT re-raise it",
             read_sonic_word(0x0a) & 0x0800, 0x0000);
    // ...and once TCR's PINT is cleared, the next one arms again.
    dut->tx_done_tcr = 0x0000; dut->tx_done_pint = 0;
    dut->tx_done_valid = 1; tick(); dut->tx_done_valid = 0;
    dut->tx_done_tcr = 0x8000; dut->tx_done_pint = 1;
    dut->tx_done_valid = 1; tick(); dut->tx_done_valid = 0;
    CHECK_EQ("PINT re-arms after TCR.PINT goes low",
             read_sonic_word(0x0a) & 0x0800, 0x0800);
    dut->tx_done_pint = 0; dut->tx_done_tcr = 0x0001;
    write_sonic_word(0x0a, 0x0A00);
    dut->tx_done_valid = 1; tick(); dut->tx_done_valid = 0;
    CHECK_EQ("ISR restored to TXDN for the RRRA flow (2)",
             read_sonic_word(0x0a), 0x0200);

    write_sonic_word(0x1a, 0x0000); // URDA
    write_sonic_word(0x1c, 0x3000); // CRDA
    write_sonic_word(0x28, 0x0000); // URRA
    write_sonic_word(0x2a, 0x1000); // RSA
    write_sonic_word(0x2e, 0x1000); // RRP
    write_sonic_word(0x30, 0x100c); // RWP
    write_sonic_byte(0x00, 0x01);   // RRRA command (CR high byte)
    tick();
    CHECK_EQ("RX config held valid", dut->rx_cfg_valid, 1);
    CHECK_EQ("RX config CRDA", dut->rx_cfg_crda, 0x3000);
    CHECK_EQ("RX config operation is RRA fetch", dut->rx_cfg_op, 1);
    CHECK_EQ("RRRA remains active", read_sonic_word(0x00), 0x0114);
    dut->rx_cfg_ready = 1; tick(); dut->rx_cfg_ready = 0;
    CHECK_EQ("RX config handshake", dut->rx_cfg_valid, 0);
    CHECK_EQ("RRRA remains active during DMA", read_sonic_word(0x00), 0x0114);
    dut->rx_done_crda=0x3000;dut->rx_done_crba0=0x2003;dut->rx_done_crba1=0;
    dut->rx_done_rbwc0=0x0400;dut->rx_done_rbwc1=0;dut->rx_done_rrp=0x1008;
    dut->rx_done_rsc=7;dut->rx_done_llfa=0;dut->rx_done_isr_set=0;
    dut->rx_done_valid=1;tick();dut->rx_done_valid=0;
    CHECK_EQ("RRRA clears after DMA completion", read_sonic_word(0x00), 0x0014);
    CHECK_EQ("RRA completion updates CRBA",read_sonic_word(0x1e),0x2003);

    write_sonic_word(0x4c, 0x0800); // CDP
    write_sonic_word(0x4e, 0x0001); // CDC
    write_sonic_byte(0x00, 0x02);   // LCAM
    tick();
    CHECK_EQ("LCAM config held valid", dut->rx_cfg_valid, 1);
    CHECK_EQ("LCAM operation", dut->rx_cfg_op, 4);
    CHECK_EQ("LCAM CDP snapshot", dut->rx_cfg_cdp, 0x0800);
    CHECK_EQ("LCAM CDC snapshot", dut->rx_cfg_cdc, 1);
    CHECK_EQ("LCAM remains active", read_sonic_word(0x00), 0x0214);
    dut->rx_cfg_ready=1;tick();dut->rx_cfg_ready=0;
    dut->rx_done_cdp=0x0808;dut->rx_done_cdc=0;dut->rx_done_ce=0x0008;
    dut->rx_done_isr_set=0x1000;dut->rx_done_valid=1;tick();dut->rx_done_valid=0;
    CHECK_EQ("LCAM clears on DMA completion",read_sonic_word(0x00),0x0014);
    CHECK_EQ("LCD follows DMA completion",read_sonic_word(0x0a),0x1200);
    CHECK_EQ("CAM enable completion",read_sonic_word(0x4a),0x0008);
    CHECK_EQ("CAM pointer completion",read_sonic_word(0x4c),0x0808);
    write_sonic_byte(0x0a,0x10); // clear LCD before RX completion checks

    dut->rx_done_rcr=0x3001; dut->rx_done_crda=1;
    dut->rx_done_crba0=0x206b; dut->rx_done_crba1=0;
    dut->rx_done_rbwc0=0x03cc; dut->rx_done_rbwc1=0;
    dut->rx_done_rrp=0x1008; dut->rx_done_rsc=8; dut->rx_done_llfa=0x300a;
    dut->rx_done_isr_set=0x0440; dut->rx_done_valid=1; tick(); dut->rx_done_valid=0;
    CHECK_EQ("RX PKTRX/RDE completion", read_sonic_word(0x0a), 0x0640);
    CHECK_EQ("RX CRDA completion", read_sonic_word(0x1c), 0x0001);
    CHECK_EQ("RX CRBA completion", read_sonic_word(0x1e), 0x206b);

    write_sonic_byte(0x0b, 0x40); // clear RDE
    tick();
    CHECK_EQ("RDE clear queues RX recovery", dut->rx_cfg_valid, 1);
    CHECK_EQ("RDE recovery operation", dut->rx_cfg_op, 2);
    CHECK_EQ("RDE recovery LLFA snapshot", dut->rx_cfg_llfa, 0x300a);
    dut->rx_cfg_ready=1;tick();dut->rx_cfg_ready=0;

    dut->rx_done_isr_set=0x0020; dut->rx_done_valid=1;tick();dut->rx_done_valid=0;
    write_sonic_word(0x30,0x1014); // driver advances RWP before clearing RBE
    write_sonic_byte(0x0b,0x20);
    tick();
    CHECK_EQ("RBE clear queues resource fetch",dut->rx_cfg_valid,1);
    CHECK_EQ("RBE recovery operation",dut->rx_cfg_op,1);
    CHECK_EQ("RBE recovery sees new RWP",dut->rx_cfg_rwp,0x1014);

    dut->rx_cfg_ready=1;tick();dut->rx_cfg_ready=0;
    write_sonic_word(0x0a,0x7fff);
    dut->sonic_cs=1;dut->sonic_wr=1;
    dut->sonic_addr=(sonic_physical_addr(0x0a)>>2);dut->sonic_wdata=0x0002;dut->sonic_wstrb=1;
    dut->tx_done_error=0;dut->tx_done_valid=1;
    dut->rx_done_isr_set=0x0400;dut->rx_done_valid=1;
    tick();
    dut->sonic_cs=0;dut->sonic_wr=0;dut->sonic_wstrb=0;dut->tx_done_valid=0;dut->rx_done_valid=0;
    CHECK_EQ("simultaneous TX/RX ISR events are merged",read_sonic_word(0x0a),0x0600);
    return true;
}
#endif

static void run(const char* name, bool (*fn)()) {
    std::printf("Running %s...\n", name);
    if (fn()) {
        std::printf("  PASS %s\n", name);
        n_pass++;
    } else {
        n_fail++;
    }
}

// A SONIC held in software reset is not a bus master.  Hardware showed the
// engine issuing 65535+ receive DMA operations for 129 frames while CR read
// RST|RXDIS and DCR was 0, aimed at a garbage CRDA -- and the recovery path's
// in-use clear is a WRITE, so that loop scribbles into guest RAM.  Nothing may
// be queued to the engine while CR_RST is asserted, and a reset must cancel
// work that is already queued or believed in flight.
static bool test_reset_issues_no_receive_dma() {
    reset();
    // CR's reset value is RST|STP|RXDIS (0x0094): the chip powers up held in
    // software reset.  That is the state the board sat in -- CR=0x0094, DCR=0,
    // rx_enable=0 -- while the engine issued 65535+ receive DMA ops for 129
    // frames against a garbage CRDA.  The recovery path's in-use clear is a
    // WRITE, so that loop scribbles into whatever guest address the stale
    // URDA/LLFA name.  A device in reset must issue nothing at all.
    write_sonic_byte(0x01, 0x00);
    CHECK_EQ("out of reset", read_sonic_word(0x00) & 0x0080, 0x0000);
    write_sonic_word(0x02, 0x0020); // DCR: 32-bit descriptors
    write_sonic_word(0x1a, 0x0030); // URDA
    write_sonic_word(0x1c, 0x1a30); // CRDA
    write_sonic_word(0x28, 0x0000); // URRA
    write_sonic_word(0x2e, 0x1000); // RRP

    // Occupy the mailbox with an RRA fetch and leave it unacknowledged.
    write_sonic_byte(0x00, 0x01);
    tick();
    CHECK_EQ("RRA fetch in flight", dut->rx_cfg_valid, 1);

    // Queue receive recovery behind it, the way it really happens: the engine
    // reports RDE and the driver clears it.
    dut->rx_done_isr_set = 0x0040; dut->rx_done_valid = 1; tick();
    dut->rx_done_valid = 0; dut->rx_done_isr_set = 0;
    write_sonic_byte(0x0b, 0x40);
    tick();

    // Reset while that queued work is still waiting for the mailbox.
    write_sonic_byte(0x01, 0x80);
    tick();
    CHECK_EQ("reset asserted", read_sonic_word(0x00) & 0x0080, 0x0080);

    // Retire the in-flight op.  Nothing new may follow it: the queued recovery
    // was cancelled by the reset, and a device in reset issues no work.
    dut->rx_cfg_ready = 1; tick(); dut->rx_cfg_ready = 0;
    for (int i = 0; i < 12; i++) {
        tick();
        CHECK_EQ("no config op while in reset", dut->rx_cfg_valid, 0);
    }

    // Coming out of reset must not replay the cancelled work either.
    write_sonic_byte(0x01, 0x00);
    for (int i = 0; i < 12; i++) {
        tick();
        CHECK_EQ("cancelled work is not replayed", dut->rx_cfg_valid, 0);
    }

    // Second half: work RAISED while the device is held in reset.  Cancelling
    // on the reset edge cannot help here -- only gating the issue path on
    // CR_RST keeps the engine off the bus, which is the state the board was
    // actually stuck in.
    write_sonic_byte(0x01, 0x80);
    tick();
    CHECK_EQ("reset re-asserted", read_sonic_word(0x00) & 0x0080, 0x0080);
    dut->rx_done_isr_set = 0x0040; dut->rx_done_valid = 1; tick();
    dut->rx_done_valid = 0; dut->rx_done_isr_set = 0;
    write_sonic_byte(0x0b, 0x40);          // clear RDE -> requests recovery
    for (int i = 0; i < 12; i++) {
        tick();
        CHECK_EQ("reset blocks newly raised work", dut->rx_cfg_valid, 0);
    }
    return true;
}

// RCR is a live receive-filter input: real silicon reads it per frame.  This
// engine only saw it in the snapshot taken at a config handshake, and drivers
// program RCR *after* their last RRA/CAM command -- so on hardware the engine
// held RCR=0x0000 while CR read RXEN and DCR was correctly 0x8023, and every
// validated frame was dropped before any DMA (accepts=NONE-all-frames-dropped).
// A later RCR write must reach the engine on its own.
static bool test_rcr_change_refreshes_the_filter() {
    reset();
    write_sonic_byte(0x01, 0x00);          // out of reset
    write_sonic_word(0x02, 0x0020);        // DCR: 32-bit descriptors
    for (int i = 0; i < 4; i++) tick();
    CHECK_EQ("no refresh queued for an untouched RCR", dut->rx_cfg_valid, 0);

    // The driver enables broadcast reception after everything else is set up.
    write_sonic_word(0x04, 0x2000);        // RCR = BRD
    tick();
    CHECK_EQ("RCR change queues a refresh", dut->rx_cfg_valid, 1);
    CHECK_EQ("refresh carries no DMA op", dut->rx_cfg_op, 0);
    CHECK_EQ("refresh carries the new RCR", dut->rx_cfg_rcr, 0x2000);
    CHECK_EQ("refresh keeps DCR", dut->rx_cfg_dcr, 0x0020);
    dut->rx_cfg_ready = 1; tick(); dut->rx_cfg_ready = 0;

    // Once delivered it must settle -- no repeat storm on an unchanged RCR.
    for (int i = 0; i < 12; i++) {
        tick();
        CHECK_EQ("refresh does not repeat", dut->rx_cfg_valid, 0);
    }

    // A further change is picked up too -- via a NATIVE 16-bit write, the
    // shape a driver's MOVE.W takes.  RCR has no dedicated arm in that path,
    // so it is handled by `default` and needs its own staleness marking.
    write_sonic_native_word(0x02, 0x2800);  // RCR = BRD | AMC
    tick();
    CHECK_EQ("native RCR write refreshes", dut->rx_cfg_valid, 1);
    CHECK_EQ("native refresh RCR", dut->rx_cfg_rcr, 0x2800);
    dut->rx_cfg_ready = 1; tick(); dut->rx_cfg_ready = 0;

    // And a device held in reset still issues nothing.
    write_sonic_byte(0x01, 0x80);
    write_sonic_word(0x04, 0x3000);
    for (int i = 0; i < 12; i++) {
        tick();
        CHECK_EQ("reset still blocks the refresh", dut->rx_cfg_valid, 0);
    }
    return true;
}


// Drive a CR low-byte write and an engine completion pulse in the SAME clock
// edge.  Both are nonblocking assignments to CR inside one always block, so
// before the merge the engine's statement -- being later in the block -- won
// outright and the CPU's command simply evaporated.
static void write_cr_low_with_tx_done(uint8_t value) {
    dut->sonic_cs = 1;
    dut->sonic_rd = 0;
    dut->sonic_wr = 1;
    dut->sonic_addr = 0;                 // CR
    dut->sonic_wdata = value;            // low byte
    dut->sonic_wstrb = 1;
    dut->tx_done_valid = 1;
    dut->eval();
    tick();
    dut->sonic_cs = 0;
    dut->sonic_wr = 0;
    dut->sonic_wstrb = 0;
    dut->tx_done_valid = 0;
    dut->eval();
}

static bool test_cr_write_survives_engine_writeback() {
    reset();
    write_sonic_byte(0x01, 0x00);        // leave reset
    write_sonic_word(0x02, 0x0020);      // DCR: 32-bit descriptors
    write_sonic_byte(0x01, 0x02);        // TXP -> transmit in flight
    dut->tx_cmd_ready = 1; tick(); dut->tx_cmd_ready = 0;
    CHECK_EQ("TXP in flight", read_sonic_word(0x00) & 0x0002, 0x0002);

    // HTX while the transmit is genuinely still running: the direction the
    // stub engine cannot show.
    write_sonic_byte(0x01, 0x01);
    CHECK_EQ("HTX clears an in-flight TXP", read_sonic_word(0x00) & 0x0003, 0x0001);

    // Re-arm a transmit, then collide a RXEN command with its completion.
    write_sonic_byte(0x01, 0x02);
    dut->tx_cmd_ready = 1; tick(); dut->tx_cmd_ready = 0;
    dut->tx_done_ctda = 0x1111;
    dut->tx_done_tcr  = 0x0001;
    dut->tx_done_tps  = 0x0022;
    dut->tx_done_tfc  = 0x0033;
    dut->tx_done_error = 0;
    write_cr_low_with_tx_done(0x08);     // CR_RXEN, same edge as completion

    uint16_t cr = read_sonic_word(0x00);
    CHECK_EQ("colliding RXEN command is not swallowed", cr & 0x0008, 0x0008);
    CHECK_EQ("engine's TXP clear still applied", cr & 0x0002, 0x0000);
    CHECK_EQ("RXEN cleared RXDIS", cr & 0x0004, 0x0000);
    CHECK_EQ("completion status still landed", read_sonic_word(0x10), 0x0022);
    CHECK_EQ("TXDN still raised", read_sonic_word(0x0a) & 0x0200, 0x0200);
    CHECK_TRUE("helpers saw acks", helper_ok);
    return true;
}

static bool test_software_reset_cancels_pending_tx() {
    reset();
    write_sonic_byte(0x01, 0x00);        // leave reset
    write_sonic_word(0x02, 0x0020);      // DCR
    write_sonic_word(0x0c, 0x1234);      // UTDA
    write_sonic_word(0x0e, 0x5678);      // CTDA
    write_sonic_byte(0x01, 0x02);        // TXP, never handshaked
    CHECK_EQ("TX command queued", dut->tx_cmd_valid, 1);

    // The RX side was already cancelled here; the TX side was not, so a queued
    // transmit stayed armed across the reset and would walk a descriptor list
    // the driver had already disowned, reporting into TCR/TPS/CTDA afterwards.
    write_sonic_byte(0x01, 0x80);        // CR_RST
    CHECK_EQ("software reset cancels the queued TX", dut->tx_cmd_valid, 0);
    for (int i = 0; i < 8; i++) {
        tick();
        CHECK_EQ("and it does not re-arm", dut->tx_cmd_valid, 0);
    }
    CHECK_TRUE("helpers saw acks", helper_ok);
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vq700_eth_sonic;
    run("enet_prom", test_enet_prom);
    run("sonic_reset_values", test_sonic_reset_values);
    run("mame_bus_layout", test_mame_bus_layout);
#ifdef PACKET_ENGINE_TEST
    run("packet_engine_handshake", test_packet_engine_handshake);
    run("reset_issues_no_receive_dma", test_reset_issues_no_receive_dma);
    run("rcr_change_refreshes_the_filter", test_rcr_change_refreshes_the_filter);
    run("cr_write_survives_engine_writeback", test_cr_write_survives_engine_writeback);
    run("software_reset_cancels_pending_tx", test_software_reset_cancels_pending_tx);
#else
    run("sonic_masks_and_commands", test_sonic_masks_and_commands);
#endif
    delete dut;
    std::printf("q700_eth_sonic: %d passed, %d failed at t=%llu\n",
                n_pass, n_fail, (unsigned long long)sim_time);
    return n_fail == 0 ? 0 : 1;
}
