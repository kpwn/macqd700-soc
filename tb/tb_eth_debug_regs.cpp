#include <cstdint>
#include <cstdio>
#include <verilated.h>
#include "Veth_debug_regs.h"

static Veth_debug_regs d;
static int failures;

static void tick() {
    d.clk = 0; d.eval();
    d.clk = 1; d.eval();
}

static void idle() {
    d.awvalid=0; d.wvalid=0; d.bready=0; d.arvalid=0; d.rready=0;
    d.tx_cmd_fire=0; d.tx_done_fire=0; d.tx_done_error=0;
    d.rx_done_fire=0; d.rx_done_error=0;
    d.tx_axis_fire=0; d.tx_axis_last=0;
    d.rx_axis_fire=0; d.rx_axis_last=0; d.rx_axis_user=0;
    d.dma_req_fire=0; d.dma_rsp_fire=0;
}

static void expect(const char *name, uint32_t got, uint32_t want) {
    if (got != want) {
        std::printf("FAIL %-28s got=%08x want=%08x\n", name, got, want);
        failures++;
    }
}

static uint32_t read_reg(uint32_t addr) {
    d.araddr=addr; d.arvalid=1; d.rready=1;
    for (int n=0; n<16; n++) {
        d.eval();
        if (d.rvalid) {
            uint32_t v=d.rdata; tick(); idle(); return v;
        }
        tick();
    }
    failures++; std::printf("FAIL read timeout @%x\n", addr); idle(); return 0;
}

static void write_reg(uint32_t addr, uint32_t data, int skew) {
    bool aw=false, w=false;
    for (int n=0; n<24; n++) {
        d.awaddr=addr; d.wdata=data; d.wstrb=0xf;
        d.awvalid=!aw && n >= (skew < 0 ? -skew : 0);
        d.wvalid =!w  && n >= (skew > 0 ?  skew : 0);
        d.bready=1; d.eval();
        bool af=d.awvalid && d.awready, wf=d.wvalid && d.wready;
        if (d.bvalid) { tick(); idle(); return; }
        tick(); aw |= af; w |= wf;
    }
    failures++; std::printf("FAIL write timeout @%x\n", addr); idle();
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    d.link_speed_async=0; d.mac_event_toggle_async=0;
    d.sonic_irq_async=0; d.sonic_rx_enable_async=0;
    d.sonic_cr_async=0; d.sonic_dcr_async=0;
    d.sonic_imr_async=0; d.sonic_isr_async=0;
    d.tx_state=0; d.rx_state=0; d.tx_descriptor_addr=0;
    d.rx_descriptor_addr=0; d.rx_frame_len=0;
    d.rx_rcr=0; d.rx_cam_enable=0; d.rx_cam_entry=0;
    d.dma_req_addr=0; d.dma_req_len=0; d.dma_req_tag=0; d.dma_req_write=0;
    d.dma_rsp_status=0; d.dma_rsp_tag=0; d.dma_rsp_write=0;
    idle(); d.rst=1; for (int i=0;i<4;i++) tick(); d.rst=0; tick();

    expect("IDENT", read_reg(0x00), 0x45544801);

    d.link_speed_async=2; d.sonic_irq_async=1; d.sonic_rx_enable_async=1;
    d.sonic_cr_async=0x1234; d.sonic_dcr_async=0x0020;
    d.sonic_imr_async=0x5678; d.sonic_isr_async=0x9abc;
    d.tx_state=5; d.rx_state=17; d.tx_descriptor_addr=0x11112222;
    d.rx_descriptor_addr=0x33334444; d.rx_frame_len=37;
    for (int i=0;i<4;i++) tick();
    expect("CR/IMR snapshot", read_reg(0x0c), 0x12345678);
    expect("ISR snapshot", read_reg(0x10) >> 16, 0x9abc);
    // DCR is the only place the descriptor width (bit 5, DW) is observable
    // from JTAG; caps bit 5 tells the host the register exists at all.
    expect("DCR snapshot", read_reg(0x3c), 0x00000020);
    expect("caps advertises DCR", read_reg(0x04) & 0x20, 0x20);
    d.sonic_dcr_async=0x0000; for (int i=0;i<4;i++) tick();
    expect("DCR tracks a 16-bit reprogram", read_reg(0x3c), 0x00000000);
    // Receive-admission state: without one of these terms every validated
    // frame is dropped before any DMA, which is invisible in the counters.
    d.rx_rcr=0x2000; d.rx_cam_enable=0x0001;
    d.rx_cam_entry=0x00A040001122ULL;
    for (int i=0;i<4;i++) tick();
    expect("RCR/CAM-enable snapshot", read_reg(0x74), 0x20000001);
    expect("CAM entry low", read_reg(0x78), 0x40001122);
    expect("CAM entry high", read_reg(0x7c), 0x000000A0);

    // 0x5c selects which CAM entry 0x78/0x7c report.  It has its own offset
    // so that picking an entry cannot disturb promiscuous mode (0x70) by
    // read-modify-write; assert both directions of that independence.
    expect("CAM index defaults to 0", read_reg(0x5c), 0x00000000);
    write_reg(0x70, 0x1, 0);
    write_reg(0x5c, 0xF, 0);
    expect("CAM index reads back", read_reg(0x5c), 0x0000000F);
    expect("CAM select left promisc alone", read_reg(0x70) & 1u, 0x1u);
    write_reg(0x70, 0x0, 0);
    expect("promisc clear left CAM index alone", read_reg(0x5c), 0x0000000F);
    expect("CAM index is 4 bits", (read_reg(0x5c) >> 4), 0x00000000);
    expect("caps advertises RX filter state", read_reg(0x04) & 0x40, 0x40);

    d.tx_cmd_fire=1; tick(); idle();
    for (int i=0;i<3;i++) { d.tx_axis_fire=1; d.tx_axis_last=(i==2); tick(); }
    idle(); d.tx_done_fire=1; d.tx_done_error=1; tick(); idle();
    for (int i=0;i<2;i++) { d.rx_axis_fire=1; d.rx_axis_last=(i==1); d.rx_axis_user=(i==1); tick(); }
    idle(); d.rx_done_fire=1; tick(); idle();
    expect("TX cmd/frame counters", read_reg(0x40), 0x00010001);
    expect("TX done/error counters", read_reg(0x44), 0x00010001);
    expect("RX frame/axis-error", read_reg(0x48), 0x00010001);
    expect("last frame lengths", read_reg(0x24), 0x00030002);
    expect("first error kind/address", read_reg(0x30), 0x01000000);
    expect("first error descriptor", read_reg(0x34), 0x11112222);

    d.dma_req_addr=0xdeadbeef; d.dma_req_len=9; d.dma_req_tag=0x42;
    d.dma_req_write=1; d.dma_req_fire=1; tick(); idle();
    d.dma_rsp_fire=1; d.dma_rsp_status=2; d.dma_rsp_tag=0x42;
    d.dma_rsp_write=1; tick(); idle();
    expect("DMA req counters", read_reg(0x50), 0x00010000);
    expect("DMA rsp counters", read_reg(0x54), 0x00010000);
    expect("DMA error counters", read_reg(0x58), 0x00010000);

    // Bit 3 is a GOOD TX frame and bit 7 a GOOD RX frame; neither may arm the
    // sticky first-error snapshot.  The mask used to include bit 3, so every
    // healthy transmit reported first_error=1 with info 0x10000008.
    write_reg(0x08, 1, +2);
    d.mac_event_toggle_async=0x08; for (int i=0;i<4;i++) tick();
    expect("good TX frame is not an error", read_reg(0x30), 0);
    d.mac_event_toggle_async=0x88; for (int i=0;i<4;i++) tick();
    expect("good RX frame is not an error", read_reg(0x30), 0);
    // Bit 4 IS an error (RX bad frame) and used to be masked out entirely.
    d.mac_event_toggle_async=0x98; for (int i=0;i<4;i++) tick();
    expect("RX bad frame arms the snapshot", read_reg(0x30) >> 24, 0x10);
    write_reg(0x08, 1, +2);
    d.mac_event_toggle_async=0x00; for (int i=0;i<4;i++) tick();
    write_reg(0x08, 1, +2);

    d.mac_event_toggle_async=0x81; for (int i=0;i<4;i++) tick();
    expect("MAC event pair 0/1", read_reg(0x60), 0x00010000);
    expect("MAC event pair 6/7", read_reg(0x6c), 0x00000001);

    write_reg(0x08, 1, +2); // AW before W
    expect("clear counters", read_reg(0x40), 0);
    expect("clear sticky error", read_reg(0x30), 0);
    write_reg(0x08, 1, -2); // W before AW must also complete

    expect("promisc reset", read_reg(0x70), 0);
    write_reg(0x70, 1, +1);
    expect("promisc enable", read_reg(0x70), 1);
    write_reg(0x70, 0, -1);
    expect("promisc disable", read_reg(0x70), 0);

    if (failures) { std::printf("eth_debug_regs: %d failure(s)\n", failures); return 1; }
    std::puts("eth_debug_regs: PASS");
    return 0;
}
