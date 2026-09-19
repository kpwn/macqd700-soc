// tb_scsi_c96_stuck_supply.cpp — deterministic repro of the boot-#2
// hardware stall: the CPU pins forever inside the ROM's blind MOVE.W
// pseudo-DMA burst because the 53C96 never receives the next byte of a
// multi-block SCSI READ.
//
// MECHANISM (established by RTL reading, scsi.v):
//   A multi-block READ enters S_DATA_IN after the FIRST buffered byte
//   (scsi.v:4385) and streams the remaining blocks concurrently.  If the
//   SD provider STALLS mid-stream with `vh_busy` HIGH and never asserts
//   `vh_done`/`vh_error`, then:
//     * `c96_avail_bytes` (= vh_buf_count) drains to 0 and never recovers;
//     * `c96_accept_ev` cannot fire (avail==0), the FIFO stays empty, and
//       f8ad8c33's `c96_shim_rd_starved` back-pressure withholds the beat
//       FOREVER — the CPU pins in the MOVE.W burst;
//     * the S_DATA_IN supply-exhaustion backstop (scsi.v:5492) CANNOT
//       fire: it requires BOTH `!vh_busy` (false — provider stuck busy)
//       AND `!t_req` (false — t_req is held HIGH for the whole block in
//       the C96 pseudo-DMA path; it is only lowered at block boundaries);
//     * the mid-stream error path (scsi.v:4685) needs `vh_done && vh_error`
//       — the provider never raises them.
//   Net: no completion path at all, INT never rises, unrecoverable pin.
//   This is the hardware boot-#2 signature (pinned at ROM 0x40899664).
//
// This tb models the STUCK provider directly (sd_busy stays high, no
// done/error) so the wedge is deterministic and does not depend on the
// real SD/SPI timing that made it invisible in every prior sim.
//
// f8ad8c33's harness lesson is honoured: every pseudo-DMA beat honours
// dma_rd_ready before pulsing pb_rd, exactly as peripheral_bus.v does —
// a beat withheld past a large budget is reported as the CPU-pin, not
// hung silently.
//
// Build via:  make tb-scsi-c96-stuck-supply

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <verilated.h>
#include "Vtb_scsi_vhdd_sd.h"
#include "Vtb_scsi_vhdd_sd___024root.h"

static Vtb_scsi_vhdd_sd* dut = nullptr;
static int n_pass = 0, n_fail = 0;
static std::string cur_scn = "?";
static std::string g_fail_kind = "";
static void note_fail(const char* k){ if (g_fail_kind.empty()) g_fail_kind=k; }

// ─── SD mock with a mid-stream STUCK-BUSY knob ───────────────────────
struct SdMock {
    std::vector<uint8_t> read_sector;
    uint32_t last_lba = 0;
    uint8_t  last_cmd_type = 0;
    int      gap = 0;
    // Stall knobs:
    int  stall_at = -1;     // byte index after which to stop delivering
    bool stall_busy = true; // true: keep sd_busy HIGH (stuck); false: drop
                            //       busy without done (silent close)
    bool err_on_stall = false; // true: assert done+error at the stall
    int skid_max = 2, skid = 2;   // consumer back-pressure skid (see mame-chunk)
    enum class State { Idle, Reading, Done, StuckBusy, SilentIdle } st = State::Idle;
    int cnt = 0, total = 512, delay = 0, gap_ctr = 0;
} sd_mock;

static void sd_mock_tick() {
    dut->sd_busy = 0; dut->sd_done = 0; dut->sd_error = 0;
    dut->sd_rd_valid = 0; dut->sd_rd_data = 0; dut->sd_wr_ready = 0;

    if (sd_mock.st == SdMock::State::StuckBusy) { dut->sd_busy = 1; return; }
    if (sd_mock.st == SdMock::State::SilentIdle) { return; } // busy low, nothing

    if (sd_mock.st == SdMock::State::Idle && dut->sd_go) {
        sd_mock.last_lba = dut->sd_lba;
        sd_mock.last_cmd_type = dut->sd_cmd_type;
        sd_mock.cnt = 0; sd_mock.delay = 2; sd_mock.gap_ctr = 0;
        if (dut->sd_cmd_type == 1) { sd_mock.st = SdMock::State::Reading; sd_mock.total = 512; }
        else if (dut->sd_cmd_type == 2) { sd_mock.st = SdMock::State::Reading; sd_mock.total = 512*(int)dut->sd_block_count; }
        else { sd_mock.st = SdMock::State::Done; }
        dut->sd_busy = 1; return;
    }
    if (sd_mock.st == SdMock::State::Reading) {
        dut->sd_busy = 1;
        if (sd_mock.delay > 0) { --sd_mock.delay; return; }
        if (sd_mock.gap_ctr > 0) { --sd_mock.gap_ctr; return; }
        // Mid-stream stall trigger.
        if (sd_mock.stall_at >= 0 && sd_mock.cnt >= sd_mock.stall_at) {
            if (sd_mock.err_on_stall) {
                sd_mock.st = SdMock::State::Done;   // will assert done+error
                dut->sd_done = 1; dut->sd_error = 1; return;
            }
            sd_mock.st = sd_mock.stall_busy ? SdMock::State::StuckBusy
                                            : SdMock::State::SilentIdle;
            return;
        }
        // Honour the consumer's back-pressure exactly as the SoC's
        // peripheral_bus / scsi.v ring pacing requires (the f8ad8c33
        // harness lesson): a provider that ignores sd_rd_ready models a
        // host this SoC cannot build and would overrun the 512-byte ring.
        if (dut->sd_rd_ready) { sd_mock.skid = sd_mock.skid_max; }
        else if (sd_mock.skid > 0) { --sd_mock.skid; }
        else { return; }                   // paused: ring full downstream
        if (sd_mock.cnt < sd_mock.total) {
            dut->sd_rd_valid = 1;
            uint8_t b = sd_mock.read_sector.empty() ? 0
                        : sd_mock.read_sector[sd_mock.cnt % sd_mock.read_sector.size()];
            dut->sd_rd_data = b; ++sd_mock.cnt; sd_mock.gap_ctr = sd_mock.gap;
            if (sd_mock.cnt == sd_mock.total) sd_mock.st = SdMock::State::Done;
        }
    } else if (sd_mock.st == SdMock::State::Done) {
        dut->sd_busy = 0; dut->sd_done = 1; sd_mock.st = SdMock::State::Idle;
    }
}

static void tick(){ sd_mock_tick(); dut->clk=1; dut->eval(); dut->clk=0; dut->eval(); }
static void tick_n(int n){ for(int i=0;i<n;++i) tick(); }

static void reset() {
    dut->rst=1; dut->disk_num_lbas=1048576u;
    dut->pb_addr=0; dut->pb_wdata=0; dut->pb_wr=0; dut->pb_rd=0; dut->pb_dma16_lo_beat=0;
    int g=sd_mock.gap, sa=sd_mock.stall_at; bool sb=sd_mock.stall_busy, eo=sd_mock.err_on_stall;
    sd_mock = SdMock(); sd_mock.gap=g; sd_mock.stall_at=sa; sd_mock.stall_busy=sb; sd_mock.err_on_stall=eo;
    for(int i=0;i<4;++i) tick();
    dut->rst=0; tick();
}

static uint8_t reg_r(uint8_t off){ dut->pb_addr=off&0xf; dut->pb_rd=1; dut->pb_wr=0; tick(); dut->pb_rd=0; dut->eval(); return dut->pb_rdata&0xff; }
static void reg_w(uint8_t off,uint8_t v){ dut->pb_addr=off&0xf; dut->pb_wdata=v; dut->pb_wr=1; dut->pb_rd=0; tick(); dut->pb_wr=0; dut->pb_wdata=0; }

static const int BEAT_BUDGET = 200000;
static bool shim_beat_r(uint16_t addr,bool lo,uint8_t* out){
    dut->pb_addr=addr; dut->pb_dma16_lo_beat=lo?1:0; dut->pb_wr=0; dut->eval();
    int i=0; for(;i<BEAT_BUDGET && !dut->dma_rd_ready; ++i) tick();
    if(!dut->dma_rd_ready){ dut->pb_dma16_lo_beat=0; return false; }
    dut->pb_rd=1; tick(); dut->pb_rd=0; dut->pb_dma16_lo_beat=0; dut->eval();
    *out=dut->pb_rdata&0xff; return true;
}
static bool shim_r16(uint16_t* out){ uint8_t hi=0,lo=0; if(!shim_beat_r(0x100,false,&hi))return false; if(!shim_beat_r(0x101,true,&lo))return false; *out=((uint16_t)hi<<8)|lo; return true; }
static bool shim_w8(uint8_t v){ dut->pb_addr=0x100; dut->pb_dma16_lo_beat=0; dut->pb_wdata=v; dut->eval(); int i=0; for(;i<BEAT_BUDGET && !dut->dma_wr_ready; ++i) tick(); if(!dut->dma_wr_ready) return false; dut->pb_wr=1; tick(); dut->pb_wr=0; dut->pb_wdata=0; dut->eval(); return true; }

#define U_SCSI(sig) (dut->rootp->tb_scsi_vhdd_sd__DOT__u_scsi__DOT__##sig)

static void dump(const char* why){
    std::printf("  [%s] DUMP(%s): phase=%u tcounter=0x%04x fifo=%u drq=%d "
                "vh_buf_count=%u xfer_bytes_left=%u xfer_blocks=%u t_req=%u "
                "status=0x%02x sd_st=%d\n", cur_scn.c_str(), why,
                (unsigned)U_SCSI(phase), (unsigned)U_SCSI(c96_tcounter),
                (unsigned)U_SCSI(c96_fifo_pos), (int)dut->drq,
                (unsigned)U_SCSI(vh_buf_count), (unsigned)U_SCSI(xfer_bytes_left),
                (unsigned)U_SCSI(xfer_blocks), (unsigned)U_SCSI(t_req),
                reg_r(0x4), (int)sd_mock.st);
}

#define CHECK_TRUE(k,name,cond) do{ if(!(cond)){ std::printf("  [%s] FAIL %s\n",cur_scn.c_str(),name); note_fail(k); dump(name); return false; } }while(0)
#define CHECK_EQ(k,name,got,exp) do{ uint32_t _g=(uint32_t)(got),_e=(uint32_t)(exp); if(_g!=_e){ std::printf("  [%s] FAIL %s: got 0x%x exp 0x%x\n",cur_scn.c_str(),name,_g,_e); note_fail(k); dump(name); return false; } }while(0)
#define RUN(n,e) do{ cur_scn=n; std::printf("[RUN ] %s\n",n); bool ok=(e); if(ok){++n_pass;std::printf("[PASS] %s\n",n);} else {++n_fail;std::printf("[FAIL] %s\n",n);} }while(0)

static constexpr uint8_t CI_COMPLETE=0x11, CI_MSG_ACCEPT=0x12;
static constexpr uint8_t S_INTR=0x80, S_TC0=0x10, I_FUNCTION=0x08, I_BUS=0x10, I_DISCONNECT=0x20;
static constexpr uint8_t TARGET_ID=6;

// ROM-form DMA select for a READ(6) of `blocks` blocks.
static bool rom_select_read6(uint32_t lba,int blocks){
    reg_w(0x4,TARGET_ID); reg_w(0x3,0x01); reg_w(0x1,0x00); reg_w(0x0,0x01); reg_w(0x3,0xC1);
    uint8_t v=0; for(int i=0;i<64;++i){ v=reg_r(0x6); if(v&7)break; }
    CHECK_TRUE("select","seq!=0",(v&7)!=0);
    for(int i=0;i<64;++i){ v=reg_r(0x4); if(v&7)break; }
    CHECK_EQ("select","phase=COMMAND",v&7,0x2);
    reg_w(0x2,0x08); reg_w(0x2,(lba>>16)&0x1f); reg_w(0x2,(lba>>8)&0xff); reg_w(0x2,lba&0xff); reg_w(0x2,blocks&0xff);
    bool drq=false; for(int i=0;i<20000;++i){ if(dut->drq){drq=true;break;} tick(); }
    CHECK_TRUE("select","DRQ for tail",drq);
    CHECK_TRUE("select","tail accepted",shim_w8(0x00));
    for(int i=0;i<64000;++i){ v=reg_r(0x4); if(v&0x80)break; }
    CHECK_EQ("select","post-sel status",v,0x91);
    CHECK_EQ("select","istatus",reg_r(0x5),I_FUNCTION|I_BUS);
    return true;
}

// Drain the transfer as 16-byte word chunks until the chip leaves
// DATA_IN (natural end OR a stuck-watchdog CHECK CONDITION) or wedges.
// Sets *wedged only for the unrecoverable CPU-pin: a pseudo-DMA beat
// withheld past a large budget while the chip is STILL in DATA_IN.
// Returns the number of payload bytes drained before the chip left
// DATA_IN.  A graceful short read and a full read both return !*wedged.
static int drain_until_exit(int max_bytes,bool* wedged){
    *wedged=false;
    int chunk_bytes=16; int nchunks=max_bytes/chunk_bytes; int got=0;
    for(int chunk=0; chunk<nchunks; ++chunk){
        // Chip already off DATA_IN (previous chunk completed the xfer)?
        if((reg_r(0x4)&0x07)!=0x01) return got;
        reg_w(0x0,chunk_bytes&0xff); reg_w(0x1,0); reg_w(0x3,0x90);
        for(int w=0; w<chunk_bytes/2; ++w){
            uint16_t v=0;
            if(!shim_r16(&v)){
                uint8_t s=reg_r(0x4);
                if((s&0x07)==0x01 && !(s&S_INTR)){
                    *wedged=true;
                    std::printf("  [%s] WEDGE: chunk %d word %d beat withheld "
                                "forever (status=0x%02x, still DATA_IN) — "
                                "CPU-pin signature\n",
                                cur_scn.c_str(),chunk,w,s);
                    dump("beat withheld forever");
                }
                return got;             // left DATA_IN => graceful exit
            }
            got += 2;
        }
        // Wait for this chunk's completion INT, OR the chip leaving
        // DATA_IN (stuck watchdog CHECK CONDITION mid-transfer).
        bool advanced=false;
        for(int i=0;i<300000;++i){
            uint8_t s=reg_r(0x4);
            if(s&S_INTR){ advanced=true; break; }
            if((s&0x07)!=0x01){ return got; }   // completed/short, off DATA_IN
        }
        if(!advanced){
            if((reg_r(0x4)&0x07)!=0x01) return got;
            *wedged=true;
            std::printf("  [%s] WEDGE: chunk %d post-drain INT never fires "
                        "(still DATA_IN)\n",cur_scn.c_str(),chunk);
            dump("post-drain INT never");
            return got;
        }
        (void)reg_r(0x5);
    }
    return got;
}

static void fill(){ sd_mock.read_sector.resize(512); for(int i=0;i<512;++i) sd_mock.read_sector[i]=(uint8_t)(0x5A^(i&0xff)^(i>>2)); }

// Healthy multi-block read: no wedge, drains the full payload.
static bool scn_healthy_multiblock(int blocks){
    sd_mock.gap=0; sd_mock.stall_at=-1; reset(); fill();
    if(!rom_select_read6(0,blocks)) return false;
    bool wedged=false;
    int got=drain_until_exit(512*blocks,&wedged);
    CHECK_TRUE("ctrl","healthy multi-block read does not wedge", !wedged);
    // The chip leaves DATA_IN only after the whole extent is delivered.
    CHECK_TRUE("ctrl","healthy read drains the full payload", got >= 512*blocks - 16);
    return true;
}

// THE REPRO: provider stalls mid-stream with sd_busy HIGH, never
// done/error.  Pre-fix: unrecoverable CPU-pin (wedge).  Post-fix: the
// busy-independent stuck watchdog completes it as CHECK CONDITION — no
// wedge — after draining only the bytes that actually arrived.
static bool scn_stuck_busy_midstream(bool expect_wedge){
    sd_mock.gap=0; sd_mock.stall_at=600; sd_mock.stall_busy=true; sd_mock.err_on_stall=false;
    reset(); fill();
    if(!rom_select_read6(0,4)) return false;
    bool wedged=false;
    int got=drain_until_exit(512*4,&wedged);
    if(expect_wedge){
        CHECK_TRUE("repro","stuck-busy provider WEDGES (the bug)", wedged);
        std::printf("  [%s] reproduced: stuck-busy mid-stream => unrecoverable "
                    "CPU-pin, no completion path (drained %d bytes)\n",
                    cur_scn.c_str(),got);
        return true;
    }
    CHECK_TRUE("repro","post-fix: no unrecoverable CPU-pin", !wedged);
    CHECK_TRUE("repro","post-fix: short read, not the full extent",
               got < 512*4);
    std::printf("  [%s] recovered: stuck-busy => bounded CHECK CONDITION "
                "after %d bytes (no pin)\n",cur_scn.c_str(),got);
    return true;
}

// Silent-close variant: provider drops busy without done.  Mid-block
// t_req is HIGH in the C96 path, so the existing backstop (needs !t_req
// AND !vh_busy) also cannot fire — proving the t_req inhibitor
// independently of vh_busy.  Same watchdog recovery post-fix.
static bool scn_silent_close_midblock(bool expect_wedge){
    sd_mock.gap=0; sd_mock.stall_at=600; sd_mock.stall_busy=false; sd_mock.err_on_stall=false;
    reset(); fill();
    if(!rom_select_read6(0,4)) return false;
    bool wedged=false;
    int got=drain_until_exit(512*4,&wedged);
    if(expect_wedge){
        CHECK_TRUE("repro2","silent-close mid-block WEDGES (t_req inhibitor)", wedged);
        return true;
    }
    CHECK_TRUE("repro2","post-fix: no unrecoverable CPU-pin", !wedged);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// STARVATION SITE MEASUREMENT (owner directive 2026-09-07: eliminate
// the starvation, and first establish empirically WHERE it occurs).
//
// Realistic relative pacing on hardware:
//   SD supply : SPI @25 MHz = ~320 ns/byte = 16 pb-cycles/byte @50 MHz
//   CPU drain : one blind MOVE.W ≈ 400-600 ns AXI round trip
//               ≈ 26 pb-cycles/word (≈13 cycles/byte) — FASTER than SD.
// The mock reproduces that ratio (sd gap=15 → 16 cyc/byte;
// cpu_word_gap=26).  Every pseudo-DMA word beat records how many cycles
// dma_rd_ready withheld it plus its offset, classified as:
//   START   — first 32 bytes of the transfer
//   BLKEDGE — within 16 bytes after a 512-byte block boundary
//   MID     — anywhere else
// A healthy design shows zero withholds while the ring headroom lasts;
// pre-stage fixes should push the first withhold out by the headroom
// bytes (or eliminate it for transfers shorter than the deficit bound).
// ─────────────────────────────────────────────────────────────────────
static int g_cpu_word_gap = 0;

static bool shim_r16_meas(uint16_t* out,int* waited){
    *waited=0;
    uint8_t hi=0,lo=0;
    dut->pb_addr=0x100; dut->pb_dma16_lo_beat=0; dut->pb_wr=0; dut->eval();
    int i=0; for(;i<BEAT_BUDGET && !dut->dma_rd_ready; ++i) tick();
    if(!dut->dma_rd_ready){ dut->pb_dma16_lo_beat=0; return false; }
    *waited+=i;
    dut->pb_rd=1; tick(); dut->pb_rd=0; dut->eval(); hi=dut->pb_rdata&0xff;
    dut->pb_addr=0x101; dut->pb_dma16_lo_beat=1; dut->eval();
    i=0; for(;i<BEAT_BUDGET && !dut->dma_rd_ready; ++i) tick();
    if(!dut->dma_rd_ready){ dut->pb_dma16_lo_beat=0; return false; }
    *waited+=i;
    dut->pb_rd=1; tick(); dut->pb_rd=0; dut->pb_dma16_lo_beat=0; dut->eval();
    lo=dut->pb_rdata&0xff;
    *out=((uint16_t)hi<<8)|lo;
    return true;
}

static bool scn_measure_starvation(int blocks,int chunk_bytes){
    sd_mock.gap=15; sd_mock.stall_at=-1; reset(); fill();
    g_cpu_word_gap=26;
    if(!rom_select_read6(0,blocks)) return false;

    long n_withheld=0, cyc_withheld=0, worst=0;
    long cls_start=0, cls_blkedge=0, cls_mid=0;
    int first_offset=-1;
    int total=512*blocks; int off=0;
    int nchunks=total/chunk_bytes;
    for(int chunk=0; chunk<nchunks; ++chunk){
        if((reg_r(0x4)&0x07)!=0x01) break;
        reg_w(0x0,chunk_bytes&0xff); reg_w(0x1,(chunk_bytes>>8)&0xff);
        reg_w(0x3,0x90);
        for(int w=0; w<chunk_bytes/2; ++w){
            uint16_t v=0; int waited=0;
            if(!shim_r16_meas(&v,&waited)){
                std::printf("  [%s] beat withheld past budget at offset %d\n",
                            cur_scn.c_str(),off);
                note_fail("meas"); return false;
            }
            if(waited>0){
                ++n_withheld; cyc_withheld+=waited;
                if(waited>worst) worst=waited;
                if(first_offset<0) first_offset=off;
                if(off<32) ++cls_start;
                else if((off%512)<16) ++cls_blkedge;
                else ++cls_mid;
            }
            off+=2;
            tick_n(g_cpu_word_gap);
        }
        bool intr=false;
        for(int i=0;i<400000;++i){ uint8_t s=reg_r(0x4); if(s&S_INTR){intr=true;break;}
                                   if((s&0x07)!=0x01){ break; } }
        if(!intr){ std::printf("  [%s] chunk %d INT never fired\n",cur_scn.c_str(),chunk);
                   dump("measure INT timeout");
                   note_fail("meas"); return false; }
        (void)reg_r(0x5);
    }
    std::printf("  [%s] MEASURE blocks=%d chunk=%d: withheld_beats=%ld "
                "total_wait_cyc=%ld worst=%ld first_at_byte=%d | "
                "START=%ld BLKEDGE=%ld MID=%ld (of %d words)\n",
                cur_scn.c_str(),blocks,chunk_bytes,n_withheld,cyc_withheld,
                worst,first_offset,cls_start,cls_blkedge,cls_mid,total/2);
    g_cpu_word_gap=0;
    return true;   // measurement always "passes"; the numbers are the result
}

int main(int argc,char** argv){
    Verilated::commandArgs(argc,argv);
    bool prefix_mode=false, measure=false;
    for(int i=1;i<argc;++i){
        if(!std::strcmp(argv[i],"--expect-wedge")) prefix_mode=true;
        if(!std::strcmp(argv[i],"--measure")) measure=true;
    }
    dut=new Vtb_scsi_vhdd_sd;

    if(measure){
        std::printf("=== 53C96 starvation-site measurement (realistic pacing) ===\n");
        RUN("measure_8blk_chunk16",   scn_measure_starvation(8,16));
        RUN("measure_8blk_chunk512",  scn_measure_starvation(8,512));
        RUN("measure_8blk_chunk2048", scn_measure_starvation(8,2048));
        std::printf("\n%d/%d scenarios passed.\n",n_pass,n_pass+n_fail);
        delete dut;
        return (n_fail==0)?0:1;
    }

    std::printf("=== 53C96 stuck-supply repro (%s) ===\n",
                prefix_mode? "PRE-FIX: wedge expected" : "POST-FIX: recovery expected");

    RUN("healthy_multiblock_4", scn_healthy_multiblock(4));
    RUN("stuck_busy_midstream", scn_stuck_busy_midstream(prefix_mode));
    RUN("silent_close_midblock", scn_silent_close_midblock(prefix_mode));

    std::printf("\n%d/%d scenarios passed.\n",n_pass,n_pass+n_fail);
    delete dut;
    return (n_fail==0)?0:1;
}
