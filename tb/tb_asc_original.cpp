#include <cstdint>
#include <cstdio>
#include <verilated.h>
#include "Vasc.h"

static Vasc dut;
static void tick() { dut.clk=0; dut.eval(); dut.clk=1; dut.eval(); }
static void reset() {
    dut.rst=1; dut.phi2_tick=0; dut.pb_wr=0; dut.pb_rd=0;
    dut.pb_addr=0; dut.pb_wdata=0; tick(); tick();
    dut.rst=0; tick();
}
static void write8(uint16_t a,uint8_t v) {
    dut.pb_addr=a; dut.pb_wdata=v; dut.pb_wr=1; tick(); dut.pb_wr=0;
}
static uint8_t read8(uint16_t a) {
    dut.pb_addr=a; dut.pb_rd=1; tick(); dut.pb_rd=0; dut.eval();
    return dut.pb_rdata;
}
static void samples(int n) {
    for (int i=0;i<n*35;i++) {
        dut.phi2_tick=1; tick(); dut.phi2_tick=0; tick();
    }
}
static bool check(const char *name,int got,int want) {
    if (got==want) return true;
    std::printf("FAIL %s: got %d, wanted %d\n",name,got,want);
    return false;
}
int main() {
    bool ok=true;
    reset();
    ok &= check("original version",read8(0x800),0);
    ok &= check("original reset mode",read8(0x801),0);
    ok &= check("original reset status",read8(0x804),0);
    write8(0x807,3);
    ok &= check("clock register readback",read8(0x807),3);
    write8(0x012,0x5a); // direct-addressed FIFO RAM in mode 0
    ok &= check("silent-mode RAM write",read8(0x012),0x5a);
    write8(0x801,1);
    for(int i=0;i<600;i++) write8(0x000,uint8_t(i));
    samples(89);
    ok &= check("no early half-empty IRQ",dut.irq,0);
    samples(1);
    ok &= check("exact cap=511 half-empty IRQ",dut.irq,1);
    ok &= check("half-empty status before acknowledge",read8(0x804)&1,1);
    ok &= check("804 read clears IRQ",dut.irq,0);
    ok &= check("804 read clears status",read8(0x804)&1,0);
    ok &= check("mono B pointer reset",read8(0xF23),0);
    for(int i=0;i<20;i++) write8(0x400,uint8_t(i));
    samples(20);
    ok &= check("mono ignores B FIFO",read8(0xF23),0);

    reset();
    write8(0x802,2); // original R_CONTROL stereo bit
    ok &= check("control register readback",read8(0x802),2);
    write8(0x801,1);
    for(int i=0;i<600;i++) write8(0x400,uint8_t(i));
    samples(90);
    ok &= check("stereo B half-empty IRQ",dut.irq,1);
    ok &= check("stereo B status",read8(0x804)&4,4);
    ok &= check("stereo IRQ acknowledgement",dut.irq,0);

    reset();
    write8(0x801,2);
    ok &= check("ROM wavetable mode retained",read8(0x801),2);
    std::printf("Original ASC FIFO contract: %s\n",ok?"PASS":"FAIL");
    return ok?0:1;
}
