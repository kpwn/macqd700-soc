// tb_scsi_fuzz.cpp — scriptable NCR 53C96 harness for the differential
// fuzzer (tools/fuzz/scsi_fuzz.py).
//
// Executes a register-access script (shared, byte-for-byte, with the MAME
// side driver tools/mame_scsi96_fuzz.lua) and emits a result log in the
// exact same format the MAME side emits, so tools/fuzz/scsi_fuzz.py can
// diff the two.
//
// ── Two DUT shapes, one script interpreter ───────────────────────────
// Built twice from this one file:
//
//   default             top = tb_scsi_vhdd_sd  (scsi.v + vhdd_sd)
//   -DSCSI_FUZZ_PB      top = tb_pb_scsi       (peripheral_bus.v + scsi.v
//                                               + vhdd_sd, driven by AXI4)
//
// The PB shape is the DEFAULT for `make fuzz-scsi` since 2026-08-19.
// Rationale (docs/scsi_fuzz.md "Known blind spots" #3): the DAFB pseudo-
// DMA aperture the Quadra 700 ROM actually drives is a SIXTEEN-BIT port,
// and it is peripheral_bus.v — not scsi.v — that splits one host word
// into two byte beats and carries the DRQ grant across the pair
// (scsi_dma16_lo_beat).  With scsi.v alone as the DUT that FSM is not
// instantiated, so no seed could reach the code that produced the
// 2026-08-19 Sad Mac 0F02 bus error at ROM PC 0x4089931c.  The direct
// shape is kept because it ISOLATES scsi.v: when a divergence shows up on
// the PB build, re-running it on the direct build says whether the fabric
// or the chip model owns the bug.
//
// Script ops (one per line, args in hex unless noted):
//   W <reg> <val>     53C96 register write
//   RC <reg>          register read, value COMPARED
//   RU <reg>          register read, value logged but NOT compared
//   CTRL <val>        set the DAFB TurboSCSI control word (9 bits).
//                     bit 7 = DRQ-check reads, bit 8 = DRQ-check writes
//                     (dafb.cpp:487 / :1001 / :1040).  The MAME side
//                     pokes the real DAFB register at 0xf9800024.
//   DR <n>            DRQ-paced pseudo-DMA read of n BYTES (compared)
//   DRU <n>           same, payload logged but NOT compared
//   DW <n> <b0>..     DRQ-paced pseudo-DMA write of n BYTES
//   DR16 <n>          DRQ-paced pseudo-DMA read of n WORDS (compared)
//   DRU16 <n>         same, payload NOT compared
//   DW16 <n> <w0>..   DRQ-paced pseudo-DMA write of n WORDS (4 hex each)
//   DRB/DRUB/DWB      BLIND (non-DRQ-paced) byte forms of the above
//   DRB16/DRUB16/DWB16  BLIND word forms
//   GAP <cycles>      RTL-only delay (decimal); the MAME side ignores it
//   SDGAP <cycles>    RTL-only: SD mock inter-byte pacing (decimal)
//   SETTLE            quiesce (bounded; early-exits once irq is stable)
//   SYNC <id>         emit a sync-point record (see docs/scsi_fuzz.md)
//   END               end of script
//
// ── Pacing / hold-off contract (both executors implement it verbatim) ─
// PACED ops wait for the chip's DRQ (RTL: the drq port; MAME: save item
// "0/drq") with a bounded budget and give up if it never rises — "how
// many bytes moved" is itself compared.
//
// BLIND ops (DRB…) do not wait.  They model the ROM's real 16-byte chunk
// drain, which issues eight back-to-back `move.w` with no DRQ poll at
// all; under LBTM (config3 bit 2) that drain reaches fifo_pos == 0
// precisely because its 16-bit pops skip the odd occupancy a paced byte
// drain stalls at.
//
// For BOTH families, a beat whose direction has its DAFB DRQ-check bit
// set while DRQ is low is HELD OFF: the host access is not issued at all
// and the op stops there.  That is what both sides really do — MAME
// rewinds the instruction and returns 0xffff without touching the chip
// (dafb.cpp:1003-1009); the SoC withholds the pb pulse until the
// fabric's ack watchdog fires a bus error.  Modelling it as "stop, and
// log how far we got" keeps the two sides comparable while making the
// DRQ decision a PER-BEAT compared quantity instead of something only
// visible at SYNC granularity.
//
// If an issued access fails to complete (PB build: peripheral_bus's ack
// watchdog answers SLVERR; direct build: no pb_ack), the record carries
// to=1.  MAME can never produce that, so it is always a divergence — by
// design: an aperture beat that never terminates IS the Sad Mac.
//
// Multiple scripts run sequentially in ONE process against a live chip —
// deliberately matching the MAME side, where scripts run back-to-back in
// one emulator boot.  Each script starts with a normalization preamble
// (emitted by the generator) instead of a power-on reset.
//
// Build via:   make tb-scsi-fuzz-harness         (PB shape, the default)
//              make tb-scsi-fuzz-harness-direct  (scsi.v in isolation)

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <map>
#include <array>
#include <algorithm>
#include <dirent.h>
#include <verilated.h>

#ifdef SCSI_FUZZ_PB
#include "Vtb_pb_scsi.h"
#include "Vtb_pb_scsi___024root.h"
typedef Vtb_pb_scsi Dut;
#define U_SCSI(field) (dut->rootp->tb_pb_scsi__DOT__u_scsi__DOT__##field)
#define DUT_IRQ  (dut->scsi_irq)
#define DUT_DRQ  (dut->scsi_drq)
#else
#include "Vtb_scsi_vhdd_sd.h"
#include "Vtb_scsi_vhdd_sd___024root.h"
typedef Vtb_scsi_vhdd_sd Dut;
#define U_SCSI(field) (dut->rootp->tb_scsi_vhdd_sd__DOT__u_scsi__DOT__##field)
#define DUT_IRQ  (dut->irq)
#define DUT_DRQ  (dut->drq)
#endif

static Dut* dut = nullptr;

// ─── Deterministic disk (matches the CHD the fuzzer builds for MAME) ──
// Default backing is the procedural pattern
//   p(s,i) = (s*197 + i*13 + ((s>>8)*59) + 7) & 0xff
// With --img <file>, sectors are served from that image instead (the
// fuzzer builds MAME's CHD from the same file, so payload comparison
// covers real disk content).  Writes land in an in-memory overlay on
// this side and in MAME's diff file on the other — the image file
// itself is never modified by either side.
// The SD aperture is biased: SD sector = disk LBA + SD_LBA_BIAS (8192).
static constexpr uint32_t SD_BIAS = 8192;
static uint32_t g_disk_lbas = 32768;
static FILE*    g_img       = nullptr;

static std::map<uint32_t, std::vector<uint8_t>> disk_overlay; // written sectors

static uint8_t disk_byte(uint32_t lba, uint32_t i) {
    auto it = disk_overlay.find(lba);
    if (it != disk_overlay.end()) return it->second[i];
    if (g_img) {
        static uint32_t cached_lba = ~0u;
        static uint8_t  cache[512];
        if (lba != cached_lba) {
            if (fseek(g_img, (long)lba * 512, SEEK_SET) != 0 ||
                fread(cache, 1, 512, g_img) != 512)
                memset(cache, 0, sizeof cache);
            cached_lba = lba;
        }
        return cache[i];
    }
    return (uint8_t)((lba * 197u + i * 13u + ((lba >> 8) * 59u) + 7u) & 0xffu);
}
#define DISK_LBAS g_disk_lbas

// ─── SD mock (pattern-backed, write overlay), from tb_scsi_c96_read6 ──
struct SdMock {
    uint32_t cur_lba = 0;
    uint8_t  cmd = 0;
    int      cnt = 0, total = 0, delay = 0, gap = 0, gap_ctr = 0;
    int      skid = 2, skid_max = 2;
    bool     oor = false;
    std::vector<uint8_t> wr_buf;
    enum class St { Idle, Reading, Writing, Done } st = St::Idle;
} sd;

static void sd_mock_tick() {
    dut->sd_busy     = (sd.st != SdMock::St::Idle) ? 1 : 0;
    dut->sd_done     = 0;
    dut->sd_error    = 0;
    dut->sd_rd_valid = 0;
    dut->sd_rd_data  = 0;
    dut->sd_wr_ready = 0;

    if (sd.st == SdMock::St::Idle && dut->sd_go) {
        sd.cur_lba = dut->sd_lba;
        sd.cmd     = dut->sd_cmd_type;
        sd.cnt     = 0;
        sd.delay   = 2;
        sd.gap_ctr = 0;
        sd.wr_buf.clear();
        sd.oor = (sd.cur_lba < SD_BIAS) || (sd.cur_lba >= SD_BIAS + DISK_LBAS);
        if (sd.cmd == 1) { sd.st = SdMock::St::Reading; sd.total = 512; }
        else if (sd.cmd == 2) { sd.st = SdMock::St::Reading;
                                sd.total = 512 * (int)dut->sd_block_count; }
        else if (sd.cmd == 3 || sd.cmd == 4) {
            sd.st = SdMock::St::Writing;
            sd.total = 512 * (dut->sd_block_count ? (int)dut->sd_block_count : 1);
        } else sd.st = SdMock::St::Done;
        dut->sd_busy = 1;
        return;
    }
    if (sd.st == SdMock::St::Reading) {
        dut->sd_busy = 1;
        if (sd.oor) { sd.st = SdMock::St::Done; dut->sd_done = 1; dut->sd_error = 1; return; }
        if (sd.delay > 0)   { --sd.delay; return; }
        if (sd.gap_ctr > 0) { --sd.gap_ctr; return; }
        if (dut->sd_rd_ready)      sd.skid = sd.skid_max;
        else if (sd.skid > 0)      --sd.skid;
        else                       return;
        if (sd.cnt < sd.total) {
            uint32_t lba = sd.cur_lba + (uint32_t)(sd.cnt / 512) - SD_BIAS;
            dut->sd_rd_valid = 1;
            dut->sd_rd_data  = (lba < DISK_LBAS) ? disk_byte(lba, sd.cnt % 512) : 0;
            ++sd.cnt;
            sd.gap_ctr = sd.gap;
            if (sd.cnt == sd.total) { sd.st = SdMock::St::Done; dut->sd_done = 1; }
        }
        return;
    }
    if (sd.st == SdMock::St::Writing) {
        dut->sd_busy = 1;
        if (sd.oor) { sd.st = SdMock::St::Done; dut->sd_done = 1; dut->sd_error = 1; return; }
        if (sd.delay > 0)   { --sd.delay; return; }
        if (sd.gap_ctr > 0) { --sd.gap_ctr; return; }
        if (dut->sd_wr_avail) {
            dut->sd_wr_ready = 1;
            sd.wr_buf.push_back(dut->sd_wr_data);
            ++sd.cnt;
            sd.gap_ctr = sd.gap;
            if ((sd.cnt % 512) == 0) {
                uint32_t lba = sd.cur_lba + (uint32_t)(sd.cnt / 512) - 1 - SD_BIAS;
                if (lba < DISK_LBAS)
                    disk_overlay[lba] = std::vector<uint8_t>(
                        sd.wr_buf.end() - 512, sd.wr_buf.end());
            }
            if (sd.cnt == sd.total) { sd.st = SdMock::St::Done; dut->sd_done = 1; }
        }
        return;
    }
    if (sd.st == SdMock::St::Done) {
        // sd_done already pulsed the cycle we entered Done
        sd.st = SdMock::St::Idle;
        return;
    }
}

static void tick() {
    sd_mock_tick();
    dut->clk = 1; dut->eval();
    dut->clk = 0; dut->eval();
}
static void tick_n(int n) { for (int i = 0; i < n; ++i) tick(); }

// ─── DAFB TurboSCSI control word (harness-side mirror) ────────────────
// Both executors keep the value the script last set; the RTL drives it on
// scsi_ctrl_in, the MAME side pokes DAFB register 0x24.  Bit 7 gates
// pseudo-DMA READS on DRQ, bit 8 gates pseudo-DMA WRITES.
static uint16_t g_ctrl = 0;
static bool rd_check_on() { return (g_ctrl & 0x080) != 0; }
static bool wr_check_on() { return (g_ctrl & 0x100) != 0; }

// ═══════════════════════════════════════════════════════════════════════
// Access layer — the ONLY part that differs between the two DUT shapes
// ═══════════════════════════════════════════════════════════════════════
//
// Common contract:
//   reg_w / reg_r     53C96 register file, 16-byte stride (DAFB decode)
//   ap_rd8  / ap_rd16 pseudo-DMA aperture read,  returns {value, ok}
//   ap_wr8  / ap_wr16 pseudo-DMA aperture write, returns ok
// ok == false means the access was issued but never terminated.

struct ApRes { uint16_t val; bool ok; };

#ifdef SCSI_FUZZ_PB
// ── AXI4 master BFM (lifted from tb_pb_scsi.cpp) ─────────────────────
using Word128 = std::array<uint32_t, 4>;
template <typename Port> static void set128(Port& p, const Word128& v) {
    for (int i = 0; i < 4; i++) p[i] = v[i];
}
template <typename Port> static Word128 get128(Port& p) {
    Word128 v{}; for (int i = 0; i < 4; i++) v[i] = p[i]; return v;
}

static constexpr uint32_t SCSI_BASE     = 0x000F000u;
static constexpr uint32_t SCSI_DMA_SHIM = 0x000F100u;
// peripheral_bus's ack watchdog is 2^13 pb clocks in this build; the BFM
// budget is comfortably larger so a withheld beat surfaces as a clean
// SLVERR rather than as a BFM give-up.
static constexpr int AXI_BUDGET     = 2000;
static constexpr int AXI_DMA_BUDGET = 40000;

struct AxiRd { uint32_t data; uint32_t resp; bool completed; };

static AxiRd axi_read_full(uint32_t addr, uint8_t arsize, int max_cycles) {
    dut->s_arid = 0x3; dut->s_araddr = addr; dut->s_arlen = 0;
    dut->s_arsize = arsize; dut->s_arburst = 1; dut->s_arvalid = 1;
    dut->s_rready = 1;
    uint32_t lane = (addr >> 2) & 0x3;
    uint32_t got = 0, resp = 0;
    bool ar_done = false, r_done = false;
    for (int i = 0; i < max_cycles && !r_done; i++) {
        dut->eval();
        bool ar_hs = dut->s_arvalid && dut->s_arready;
        bool r_hs  = dut->s_rvalid  && dut->s_rready;
        if (r_hs) { Word128 v = get128(dut->s_rdata); got = v[lane]; resp = dut->s_rresp; }
        tick();
        if (ar_hs && !ar_done) { dut->s_arvalid = 0; ar_done = true; }
        if (r_hs) r_done = true;
    }
    dut->s_arvalid = 0;
    tick(); tick();
    return {got, resp, r_done};
}

// nbytes = 1 or 2.  Byte order follows the SoC's big-endian lane layout:
// peripheral_bus retires hot strobes HIGH bit first (see
// wr_scsi_word_active in peripheral_bus.v), so for a two-byte aperture
// write the first byte delivered to scsi.v is the one in the upper
// strobe of the pair — i.e. the byte at the LOWER m68k address, which is
// what MAME's dma16_swap_w pushes first.
static bool axi_write_bytes(uint32_t addr, const uint8_t* bytes, int nbytes,
                            int max_cycles) {
    dut->s_awid = 0xC; dut->s_awaddr = addr; dut->s_awlen = 0;
    dut->s_awsize = (nbytes == 2) ? 1 : 0; dut->s_awburst = 1;
    dut->s_awvalid = 1;
    uint32_t lane = (addr >> 2) & 0x3;
    Word128 w{0, 0, 0, 0};
    uint16_t strb = 0;
    if (nbytes == 1) {
        w[lane] = bytes[0];
        strb = (uint16_t)(1u << (lane * 4 + 0));
    } else {
        // strobes {3,2} of the lane: bit 3 retires first -> bytes[0].
        w[lane] = ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16);
        strb = (uint16_t)(0x3u << (lane * 4 + 2));
    }
    set128(dut->s_wdata, w);
    dut->s_wstrb = strb; dut->s_wlast = 1; dut->s_wvalid = 1; dut->s_bready = 1;

    bool aw_done = false, w_done = false, b_done = false;
    uint32_t resp = 0;
    for (int i = 0; i < max_cycles && !b_done; i++) {
        dut->eval();
        bool aw_hs = dut->s_awvalid && dut->s_awready;
        bool w_hs  = dut->s_wvalid  && dut->s_wready;
        bool b_hs  = dut->s_bvalid  && dut->s_bready;
        if (b_hs) resp = dut->s_bresp;
        tick();
        if (aw_hs && !aw_done) { dut->s_awvalid = 0; aw_done = true; }
        if (w_hs  && !w_done)  { dut->s_wvalid = 0; dut->s_wlast = 0; w_done = true; }
        if (b_hs) b_done = true;
    }
    dut->s_awvalid = 0; dut->s_wvalid = 0; dut->s_wlast = 0;
    tick(); tick();
    return b_done && resp == 0;
}

static void reg_w(uint8_t off, uint8_t v) {
    uint8_t b = v;
    (void)axi_write_bytes(SCSI_BASE | ((uint32_t)(off & 0xf) << 4), &b, 1,
                          AXI_BUDGET);
}
static uint8_t reg_r(uint8_t off) {
    AxiRd r = axi_read_full(SCSI_BASE | ((uint32_t)(off & 0xf) << 4), 2,
                            AXI_BUDGET);
    return (uint8_t)(r.data & 0xff);
}
static ApRes ap_rd8() {
    AxiRd r = axi_read_full(SCSI_DMA_SHIM, /*byte*/0, AXI_DMA_BUDGET);
    return {(uint16_t)(r.data & 0xff), r.completed && r.resp == 0};
}
static ApRes ap_rd16() {
    AxiRd r = axi_read_full(SCSI_DMA_SHIM, /*word*/1, AXI_DMA_BUDGET);
    return {(uint16_t)(r.data & 0xffff), r.completed && r.resp == 0};
}
static bool ap_wr8(uint8_t v) {
    return axi_write_bytes(SCSI_DMA_SHIM, &v, 1, AXI_DMA_BUDGET);
}
static bool ap_wr16(uint16_t v) {
    uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)(v & 0xff) };
    return axi_write_bytes(SCSI_DMA_SHIM, b, 2, AXI_DMA_BUDGET);
}
static void set_ctrl(uint16_t v) {
    g_ctrl = v & 0x1ff;
    dut->scsi_ctrl_in = g_ctrl;
    dut->eval();
}
static void bus_idle_init() {
    dut->s_awid = 0; dut->s_awaddr = 0; dut->s_awlen = 0; dut->s_awsize = 0;
    dut->s_awburst = 0; dut->s_awvalid = 0;
    set128(dut->s_wdata, Word128{0, 0, 0, 0});
    dut->s_wstrb = 0; dut->s_wlast = 0; dut->s_wvalid = 0; dut->s_bready = 1;
    dut->s_arid = 0; dut->s_araddr = 0; dut->s_arlen = 0; dut->s_arsize = 0;
    dut->s_arburst = 0; dut->s_arvalid = 0; dut->s_rready = 1;
}

#else  // ── direct pb face on scsi.v ────────────────────────────────────

static uint8_t pb_access(uint16_t addr, bool wr, uint8_t wdata, bool lo_beat,
                         bool* acked) {
    dut->pb_addr = addr;
    dut->pb_dma16_lo_beat = lo_beat ? 1 : 0;
    dut->pb_wdata = wr ? wdata : 0;
    dut->pb_wr = wr ? 1 : 0;
    dut->pb_rd = wr ? 0 : 1;
    tick();
    dut->pb_wr = 0; dut->pb_rd = 0; dut->pb_wdata = 0;
    dut->pb_dma16_lo_beat = 0;
    dut->eval();
    if (acked) *acked = dut->pb_ack != 0;
    return dut->pb_rdata & 0xff;
}

static void reg_w(uint8_t off, uint8_t v) {
    (void)pb_access(off & 0xf, true, v, false, nullptr);
}
static uint8_t reg_r(uint8_t off) {
    return pb_access(off & 0xf, false, 0, false, nullptr);
}
static ApRes ap_rd8() {
    bool ok = false;
    uint8_t v = pb_access(0x100, false, 0, false, &ok);
    return {v, ok};
}
// A host 16-bit aperture access is replayed by peripheral_bus.v as two
// byte beats: high half at 0x100 with the flag LOW, low half at 0x101
// with pb_dma16_lo_beat HIGH so scsi.v's c96_dma16_hi_granted carries the
// DRQ grant across the pair (rtl/mac/scsi.v:413-448).  Reproduced here
// verbatim so the direct build and the PB build drive scsi.v identically.
static ApRes ap_rd16() {
    bool ok0 = false, ok1 = false;
    uint8_t hi = pb_access(0x100, false, 0, false, &ok0);
    uint8_t lo = pb_access(0x101, false, 0, true,  &ok1);
    return {(uint16_t)(((uint16_t)hi << 8) | lo), ok0 && ok1};
}
static bool ap_wr8(uint8_t v) {
    bool ok = false;
    (void)pb_access(0x100, true, v, false, &ok);
    return ok;
}
// NOTE the asymmetry, and that it is deliberate: peripheral_bus.v drives
// scsi_dma16_lo_beat from rd_scsi_dma16_lo ONLY (peripheral_bus.v:1722),
// so the WRITE half of a split word gets NO grant carry and re-checks DRQ
// on its second beat.  Mirrored here rather than "fixed", because the
// point of the fuzzer is to report that, not to paper over it.
static bool ap_wr16(uint16_t v) {
    bool ok0 = false, ok1 = false;
    (void)pb_access(0x100, true, (uint8_t)(v >> 8),   false, &ok0);
    (void)pb_access(0x101, true, (uint8_t)(v & 0xff), false, &ok1);
    return ok0 && ok1;
}
static void set_ctrl(uint16_t v) {
    g_ctrl = v & 0x1ff;
    dut->scsi_ctrl_in = g_ctrl;
    dut->eval();
}
static void bus_idle_init() {
    dut->pb_addr = 0; dut->pb_wdata = 0; dut->pb_wr = 0; dut->pb_rd = 0;
    dut->pb_dma16_lo_beat = 0;
}
#endif

// ─── Script execution ─────────────────────────────────────────────────
static uint8_t g_sel_timeout = 0;   // tracked reg-5 writes, bounds SETTLE

static long settle_budget() {
    // Worst-case select timeout in pb clocks: (1019 + t*8192) chip clocks
    // times cc_eff (max 8 given the generator's constraints) plus slack,
    // times C96_CLK_DIV (2 pb clocks per chip clock).  See scsi.v:401.
    long chip = (1019L + (long)g_sel_timeout * 8192L) * 8L + 4L;
    return chip * 2L + 30000L;
}

static void do_settle() {
    long budget = settle_budget();
    long stable = 0;
    for (long i = 0; i < budget; ++i) {
        tick();
        // Early exit: interrupt posted and back-end idle for a while.
        if (DUT_IRQ && sd.st == SdMock::St::Idle) {
            if (++stable >= 4096) return;
        } else stable = 0;
    }
}

// Wait for chip DRQ with a bounded budget; false = gave up.
static bool wait_drq(long budget) {
    for (long i = 0; i < budget; ++i) {
        if (DUT_DRQ) return true;
        tick();
    }
    return false;
}

static const long DMA_BEAT_BUDGET = 200000;

// One aperture beat's gating decision, shared by every DMA op.
//   paced : wait for DRQ first (bounded)
//   then  : the DAFB check for this direction, if enabled, must see DRQ
// Returns false when the beat must NOT be issued (the op stops there).
static bool beat_allowed(bool paced, bool is_write) {
    if (paced && !wait_drq(DMA_BEAT_BUDGET)) return false;
    bool check = is_write ? wr_check_on() : rd_check_on();
    if (check && !DUT_DRQ) return false;
    return true;
}

// Internal-state peek (mirrors the MAME side's save-state item reads).
static void emit_sync(FILE* out, const char* id) {
    // Internal snapshot FIRST (register reads below mutate state).
    char fifobuf[64];
    int fp = (int)(U_SCSI(c96_fifo_pos) & 0x1f);
    int n = snprintf(fifobuf, sizeof fifobuf, "%d:", fp);
    for (int i = 0; i < fp && i < 16; ++i)
        n += snprintf(fifobuf + n, sizeof fifobuf - n, "%02x",
                      (unsigned)U_SCSI(c96_fifo)[i]);
    unsigned irq = DUT_IRQ & 1, drq = DUT_DRQ & 1;
    unsigned busid = U_SCSI(c96_bus_id) & 7;
    unsigned selto = U_SCSI(c96_select_timeout) & 0xff;
    unsigned synper = U_SCSI(c96_sync_period) & 0x1f;
    unsigned synoff = U_SCSI(c96_sync_offset) & 0xf;
    unsigned clkcnv = U_SCSI(c96_clock_conv) & 7;
    unsigned cfg1 = U_SCSI(c96_config1) & 0xff;
    unsigned cfg2 = U_SCSI(c96_config2) & 0xff;
    unsigned cfg3 = U_SCSI(c96_config3) & 0xff;
    unsigned cmd0 = U_SCSI(c96_command_q) & 0xff;
    unsigned cpos = U_SCSI(c96_command_pos) & 3;
    unsigned tcnt = U_SCSI(c96_tcount) & 0xffff;
    // Bus reads in fixed order; istatus (destructive) last.
    unsigned st  = reg_r(4);
    unsigned sq  = reg_r(6);
    unsigned fl  = reg_r(7);
    unsigned tlo = reg_r(0);
    unsigned thi = reg_r(1);
    unsigned is  = reg_r(5);
    fprintf(out,
        "SYNC %s fifo=%s irq=%u drq=%u cfg=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x"
        " cmd=%02x:%u tcount=%04x stat=%02x seq=%02x flags=%02x tclo=%02x tchi=%02x istat=%02x\n",
        id, fifobuf, irq, drq,
        busid, selto, synper, synoff, clkcnv, cfg1, cfg2, cfg3,
        cmd0, cpos, tcnt, st, sq, fl, tlo, thi, is);
}

// ─── DMA op implementations ───────────────────────────────────────────
static void do_dma_read(FILE* out, const char* op, unsigned n,
                        bool word, bool paced) {
    std::vector<uint16_t> data;
    bool timeout = false;
    for (unsigned i = 0; i < n; ++i) {
        if (!beat_allowed(paced, /*is_write=*/false)) break;
        ApRes r = word ? ap_rd16() : ap_rd8();
        data.push_back(r.val);
        if (!r.ok) { timeout = true; break; }
    }
    fprintf(out, "%s %x got=%zx data=", op, n, data.size());
    for (uint16_t v : data)
        fprintf(out, word ? "%04x" : "%02x", (unsigned)v);
    fprintf(out, " to=%d\n", timeout ? 1 : 0);
}

static void do_dma_write(FILE* out, const char* op, unsigned n,
                         const std::vector<uint16_t>& vals,
                         bool word, bool paced) {
    unsigned put = 0;
    bool timeout = false;
    for (unsigned i = 0; i < n && i < vals.size(); ++i) {
        if (!beat_allowed(paced, /*is_write=*/true)) break;
        bool ok = word ? ap_wr16(vals[i]) : ap_wr8((uint8_t)vals[i]);
        ++put;
        if (!ok) { timeout = true; break; }
    }
    fprintf(out, "%s %x put=%x to=%d\n", op, n, put, timeout ? 1 : 0);
}

static bool run_script(const char* path, const char* outpath) {
    FILE* f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); return false; }
    FILE* out = fopen(outpath, "w");
    if (!out) { fprintf(stderr, "cannot open %s\n", outpath); fclose(f); return false; }

    char line[16384];
    while (fgets(line, sizeof line, f)) {
        char* p = line;
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == '#' || *p == '\n' || *p == 0) continue;
        char op[16] = {0};
        int consumed = 0;
        if (sscanf(p, "%15s%n", op, &consumed) != 1) continue;
        p += consumed;
        if (!strcmp(op, "W")) {
            unsigned r, v; sscanf(p, "%x %x", &r, &v);
            if ((r & 0xf) == 5) g_sel_timeout = (uint8_t)v;
            reg_w((uint8_t)r, (uint8_t)v);
        } else if (!strcmp(op, "RC") || !strcmp(op, "RU")) {
            unsigned r; sscanf(p, "%x", &r);
            uint8_t v = reg_r((uint8_t)r);
            fprintf(out, "%s %x=%02x\n", op, r, v);
        } else if (!strcmp(op, "CTRL")) {
            unsigned v = 0; sscanf(p, "%x", &v);
            set_ctrl((uint16_t)v);
        } else if (!strcmp(op, "DR")    || !strcmp(op, "DRU")   ||
                   !strcmp(op, "DRB")   || !strcmp(op, "DRUB")  ||
                   !strcmp(op, "DR16")  || !strcmp(op, "DRU16") ||
                   !strcmp(op, "DRB16") || !strcmp(op, "DRUB16")) {
            unsigned n = 0; sscanf(p, "%x", &n);
            bool word  = strstr(op, "16") != nullptr;
            bool blind = (op[2] == 'B') || (op[2] == 'U' && op[3] == 'B');
            do_dma_read(out, op, n, word, /*paced=*/!blind);
        } else if (!strcmp(op, "DW")   || !strcmp(op, "DWB") ||
                   !strcmp(op, "DW16") || !strcmp(op, "DWB16")) {
            unsigned n = 0; sscanf(p, "%x%n", &n, &consumed); p += consumed;
            bool word  = strstr(op, "16") != nullptr;
            bool blind = (op[2] == 'B');
            std::vector<uint16_t> vals;
            for (unsigned i = 0; i < n; ++i) {
                unsigned b;
                if (sscanf(p, "%x%n", &b, &consumed) != 1) break;
                p += consumed;
                vals.push_back((uint16_t)b);
            }
            do_dma_write(out, op, n, vals, word, /*paced=*/!blind);
        } else if (!strcmp(op, "GAP")) {
            int n = 0; sscanf(p, "%d", &n);
            tick_n(n);
        } else if (!strcmp(op, "SDGAP")) {
            int n = 0; sscanf(p, "%d", &n);
            sd.gap = n;
        } else if (!strcmp(op, "SETTLE")) {
            do_settle();
        } else if (!strcmp(op, "SYNC")) {
            char id[32] = {0}; sscanf(p, "%31s", id);
            emit_sync(out, id);
        } else if (!strcmp(op, "END")) {
            break;
        } else {
            fprintf(stderr, "unknown op '%s' in %s\n", op, path);
        }
    }
    fclose(f);
    fclose(out);
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const char* dir = nullptr;
    const char* img = nullptr;
    for (int i = 1; i < argc - 1; ++i) {
        if (!strcmp(argv[i], "--dir")) dir = argv[i + 1];
        if (!strcmp(argv[i], "--img")) img = argv[i + 1];
    }
    if (!dir) {
        fprintf(stderr, "usage: %s --dir <scriptdir> [--img <disk image>]\n",
                argv[0]);
        return 2;
    }
    if (img) {
        g_img = fopen(img, "rb");
        if (!g_img) { fprintf(stderr, "cannot open image %s\n", img); return 2; }
        fseek(g_img, 0, SEEK_END);
        g_disk_lbas = (uint32_t)(ftell(g_img) / 512);
        fseek(g_img, 0, SEEK_SET);
    }

    dut = new Dut;
    // Power-on (once; scripts then run sequentially, like the MAME side).
    dut->rst = 1;
#ifdef SCSI_FUZZ_PB
    dut->periph_rst = 0;   // peripheral-only reset idle (see tb_pb_scsi.v)
#endif
    dut->disk_num_lbas = DISK_LBAS;
    dut->scsi_ctrl_in  = 0;          // DAFB reset default: blind DMA aperture
    g_ctrl = 0;
    bus_idle_init();
    for (int i = 0; i < 8; ++i) tick();
    dut->rst = 0;
    tick_n(4);

    // Collect <name>.txt scripts, sorted.
    std::vector<std::string> scripts;
    if (DIR* d = opendir(dir)) {
        while (dirent* e = readdir(d)) {
            std::string n(e->d_name);
            if (n.size() > 4 && n.substr(n.size() - 4) == ".txt")
                scripts.push_back(n);
        }
        closedir(d);
    }
    std::sort(scripts.begin(), scripts.end());
    for (const auto& s : scripts) {
        std::string in  = std::string(dir) + "/" + s;
        std::string outp = std::string(dir) + "/" +
                           s.substr(0, s.size() - 4) + ".rtl.log";
        if (!run_script(in.c_str(), outp.c_str())) return 1;
    }
    printf("tb_scsi_fuzz: %zu scripts executed\n", scripts.size());
    delete dut;
    return 0;
}
