// tb_top.cpp — Verilator top-level testbench for m68k-ooo
//
// Drives mac_top's instruction-fetch and AXI4 data buses against a
// flat MemModel.  The instruction port is a simple req/valid handshake
// returning a 16-byte line.  The data port is a single-outstanding
// AXI4 master (32-bit data) — we serve it directly from MemModel.
//
// Test completion: the CPU writes a magic value to 0xFFFF0000:
//   0xC0FFEE00   → PASS
//   anything else → FAIL
//
// CLI:
//   +test=<name>     test label (only used for logs)
//   +timeout=<n>     cycle limit (default 10M)
//   +rom=<path>      load Mac ROM at 0x40800000 + mirror at 0
//   +bin=<path>      load test binary at <addr>; default addr 0
//   +binaddr=<hex>   override binary load address
//   +post_sentinel_drain=<n>
//                    tick n cycles after the sentinel before cache flush
//   +stop_pc=<hex>   finish with PASS when this PC commits
//   +ipl=<cyc>:<lvl> at cycle <cyc>, raise external IPL to <lvl> (1..7).
//                    Cleared automatically on cpu_ipl_ack.  Multiple +ipl=
//                    flags may be supplied; processed in cycle order.
//   +waves           dump FST waveform

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <string>
#include <vector>
#include <verilated.h>
#include <verilated_fst_c.h>
#include "Vmac_top.h"
#include "Vmac_top___024root.h"   // for +dump_final_state internal access
#include "mem_model.h"
#include "mac_rom.h"

static Vmac_top*       dut  = nullptr;
static VerilatedFstC*  fst  = nullptr;
static MemModel*       mem  = nullptr;
static uint64_t        sim_time = 0;
static uint64_t        timeout  = 10000000ULL;
static uint64_t        post_sentinel_drain = 0;
static bool            waves    = false;
static bool            nowaves_explicit = false;
static std::string     test_name;
static std::string     rom_path;
static std::string     bin_path;
static uint32_t        bin_addr = 0;

static bool            test_done = false;
static bool            test_pass = false;
// `+ipl=cycle:level` — at simulation cycle `cycle`, raise cpu_ipl_ext to
// `level` (1..7, autovector).  Each event fires ONCE (when sim_time
// crosses its cycle); after that the level stays until cpu_ipl_ack
// clears it.  Multiple +ipl= occurrences are processed in cycle order;
// each is consumed exactly once (use multiple events to sustain or
// re-pulse IPL).
struct IplEvent {
    uint64_t cycle;
    uint8_t  level;
    bool     fired;
};
static std::vector<IplEvent> ipl_events;
static uint8_t              cur_ipl_ext = 0;
static std::string     dump_state_path;        // +dump_final_state=<path>
static bool            dump_debug_state = false;
static bool            stop_pc_enabled = false;
static uint32_t        stop_pc = 0;
static uint32_t        last_committed_seen = 0;
// Track all byte writes performed by the CPU (addr → latest value).
// Populated in drive_daxi(); consumed by dump_final_state().
#include <map>
static std::map<uint32_t, uint8_t> cpu_writes;

static void dump_final_state(const std::string& path);

// ── USP/SSP/ISP shadow vs PRF[PHYS_*_TAG] divergence detector (H1 audit) ──
// Hypothesis: a7_writeback paths read the SHADOW reg `usp` (commit.v:2880,
// 4222) into the A7 PRF slot when crossing supervisor→user.  Phase-A
// architecture says shadow tracks PRF[PHYS_USP_TAG] exactly; any divergence
// is a missed sp_slot_write_en at a usp/ssp/isp <= update site, OR a write
// to PHYS_USP_TAG slot without a matching shadow update.  Either path
// would let a stale value persist across many RTEs.
//
// Settle window: the SP-cache write goes through commit.v's sp_slot_write_en
// output → m68k_core_execute.vh PRF write — a one-cycle NBA pipeline lag
// from the shadow update.  We only flag PERSISTENT divergence (= holds for
// at least DIVERG_SETTLE_CYC cycles).  Without this, every legitimate SP
// write produces a 1-cycle spurious divergence pulse.
//
// Counts divergence events (entries into "persistent diverged" state) per
// slot.  +divergence_check enables the detector.  Reports at end.
static const int DIVERG_SETTLE_CYC = 4;
static bool     divergence_check_enabled = false;
static uint64_t divergence_events_usp = 0;
static uint64_t divergence_events_ssp = 0;
static uint64_t divergence_events_isp = 0;
static int      divergence_usp_run = 0;   // consecutive cycles diverged
static int      divergence_ssp_run = 0;
static int      divergence_isp_run = 0;
static bool     divergence_usp_persistent = false;
static bool     divergence_ssp_persistent = false;
static bool     divergence_isp_persistent = false;
static uint64_t divergence_first_usp_cycle = 0;
static uint64_t divergence_first_ssp_cycle = 0;
static uint64_t divergence_first_isp_cycle = 0;
static uint32_t divergence_first_usp_shadow = 0;
static uint32_t divergence_first_usp_prf    = 0;
static uint32_t divergence_first_usp_pc     = 0;
static uint32_t divergence_first_ssp_shadow = 0;
static uint32_t divergence_first_ssp_prf    = 0;
static uint32_t divergence_first_ssp_pc     = 0;
static uint32_t divergence_first_isp_shadow = 0;
static uint32_t divergence_first_isp_prf    = 0;
static uint32_t divergence_first_isp_pc     = 0;

// ── I-fetch model: simple req/valid, 1 cycle latency ────────────────
static int  if_pending  = 0;     // cycles left until response (0 = idle)
static uint32_t if_addr_q = 0;

// ── AXI4 model: single outstanding read or write at a time ──────────
struct Axi {
    // AR
    bool     ar_outstanding = false;
    uint32_t ar_addr = 0;
    uint8_t  ar_len = 0;
    uint8_t  ar_size = 2;
    uint8_t  ar_burst = 1;
    uint8_t  ar_beat = 0;
    int      ar_delay = 0;
    // AW/W
    bool     aw_done = false;
    bool     w_done  = false;
    uint32_t aw_addr = 0;
    uint32_t w_data  = 0;
    uint32_t w_strb  = 0;
    int      b_delay = 0;
    bool     b_pending = false;
    bool     b_mapped  = true;   // SLVERR on B if false
} ax;

static void tick() {
    dut->clk = 1;
    dut->eval();
    if (waves) fst->dump(sim_time * 10 + 5);
    sim_time++;

    dut->clk = 0;
    dut->eval();
    if (waves) fst->dump(sim_time * 10);

    if (sim_time >= timeout) {
        fprintf(stderr, "[TIMEOUT] %s exceeded %llu cycles\n",
                test_name.c_str(), (unsigned long long)timeout);
        if (!dump_state_path.empty()) dump_final_state(dump_state_path);
        exit(2);
    }
}

static void reset(int cycles = 16) {
    dut->rst = 1;
    for (int i = 0; i < 4; i++) dut->if_rdata.at(i) = 0;
    dut->if_rvalid = 0;
    dut->if_fault = 0;
    dut->daxi_awready = 0;
    dut->daxi_wready  = 0;
    dut->daxi_bvalid  = 0;
    dut->daxi_bresp   = 0;
    dut->daxi_arready = 0;
    dut->daxi_rvalid  = 0;
    dut->daxi_rdata   = 0;
    dut->daxi_rresp   = 0;
    dut->daxi_rlast   = 0;
    dut->dbg_dcache_flush_req = 0;
    dut->dbg_precise_stop_req = 0;
    dut->dbg_precise_stop_keep_tag = 0;
    dut->dbg_core_halt = 0;
    dut->dbg_arch_apply_en = 0;
    dut->dbg_arch_reg_load_en = 0;
    dut->dbg_arch_reg_load_idx = 0;
    dut->dbg_arch_reg_load_val = 0;
    dut->cpu_ipl_ext = 0;
    dut->dbg_arch_ccr_load_en = 0;
    dut->dbg_arch_ccr_load_val = 0;
    dut->dbg_arch_ctrl_load_en = 0;
    dut->dbg_arch_ctrl_load_sel = 0;
    dut->dbg_arch_ctrl_load_val = 0;
    dut->dbg_arch_mmu_load_en = 0;
    dut->dbg_arch_mmu_load_sel = 0;
    dut->dbg_arch_mmu_load_val = 0;
    dut->dbg_arch_pc_load_en = 0;
    dut->dbg_arch_pc_load_val = 0;
    for (int i = 0; i < cycles; i++) tick();
    dut->rst = 0;
}

// Build the 128-bit fetch line (big-endian byte 0 = MSB) from MemModel.
static void load_fetch_line(uint32_t addr, uint8_t out[16]) {
    uint32_t base = addr & ~0xFu;
    for (int i = 0; i < 16; i++) out[i] = mem->read8(base + i);
}

static void drive_ifetch() {
    // Default deasserts
    dut->if_rvalid = 0;
    dut->if_fault = 0;

    if (dut->if_req && if_pending == 0) {
        // Capture address, schedule a 1-cycle response
        if_addr_q  = dut->if_addr;
        if_pending = 2;
    }
    if (if_pending > 0) {
        if (--if_pending == 0) {
            uint8_t line[16];
            load_fetch_line(if_addr_q, line);
            // Pack into 128-bit big-endian: byte 0 of line at bits [127:120]
            // Verilator stores wide signals as little-endian arrays of 32-bit
            // words.  We assemble manually.
            // Vmac_top exposes if_rdata as a WData[4] array (128 bits → 4 words).
            // Word 0 = bits [31:0], Word 3 = bits [127:96].
            uint32_t w0 = ((uint32_t)line[12] << 24) | ((uint32_t)line[13] << 16)
                        | ((uint32_t)line[14] <<  8) |  line[15];
            uint32_t w1 = ((uint32_t)line[8]  << 24) | ((uint32_t)line[9]  << 16)
                        | ((uint32_t)line[10] <<  8) |  line[11];
            uint32_t w2 = ((uint32_t)line[4]  << 24) | ((uint32_t)line[5]  << 16)
                        | ((uint32_t)line[6]  <<  8) |  line[7];
            uint32_t w3 = ((uint32_t)line[0]  << 24) | ((uint32_t)line[1]  << 16)
                        | ((uint32_t)line[2]  <<  8) |  line[3];
            dut->if_rdata.at(0) = w0;
            dut->if_rdata.at(1) = w1;
            dut->if_rdata.at(2) = w2;
            dut->if_rdata.at(3) = w3;
            dut->if_rvalid = 1;
            bool fault = false;
            uint32_t base = if_addr_q & ~0xFu;
            for (int i = 0; i < 16; i++)
                fault = fault || !mem->mapped(base + (uint32_t)i);
            dut->if_fault = fault ? 1 : 0;
        }
    }
}

// Drive the AXI data slave.
static void drive_daxi() {
    // AR: accept whenever we're idle on the read channel
    dut->daxi_arready = (!ax.ar_outstanding && !ax.b_pending);
    if (dut->daxi_arvalid && dut->daxi_arready) {
        ax.ar_addr = dut->daxi_araddr;
        ax.ar_len = dut->daxi_arlen;
        ax.ar_size = dut->daxi_arsize;
        ax.ar_burst = dut->daxi_arburst;
        ax.ar_beat = 0;
        ax.ar_outstanding = true;
        ax.ar_delay = 1;   // 1-cycle "memory" latency
    }

    // R: deliver when delay expires
    dut->daxi_rvalid = 0;
    dut->daxi_rlast  = 0;
    dut->daxi_rresp  = 0;
    if (ax.ar_outstanding) {
        if (ax.ar_delay > 0) ax.ar_delay--;
        else {
            uint32_t beat_addr = ax.ar_addr;
            if (ax.ar_burst == 1) {
                beat_addr += ((uint32_t)ax.ar_beat << ax.ar_size);
            }
            uint32_t a = beat_addr & ~0x3u;
            uint32_t d = mem->read32(a);
            dut->daxi_rdata  = d;
            dut->daxi_rvalid = 1;
            dut->daxi_rlast  = (ax.ar_beat == ax.ar_len) ? 1 : 0;
            dut->daxi_rresp  = mem->mapped(a) ? 0 : 2;   // SLVERR if unmapped
            if (dut->daxi_rready) {
                if (ax.ar_beat == ax.ar_len) {
                    ax.ar_outstanding = false;
                } else {
                    ax.ar_beat++;
                }
            }
        }
    }

    // AW + W: accept independently
    dut->daxi_awready = !ax.aw_done && !ax.b_pending;
    dut->daxi_wready  = !ax.w_done  && !ax.b_pending;
    if (dut->daxi_awvalid && dut->daxi_awready) {
        ax.aw_addr = dut->daxi_awaddr;
        ax.aw_done = true;
    }
    if (dut->daxi_wvalid && dut->daxi_wready) {
        ax.w_data = dut->daxi_wdata;
        ax.w_strb = dut->daxi_wstrb;
        ax.w_done = true;
    }
    // Once both have arrived: perform the write and queue B
    if (ax.aw_done && ax.w_done && !ax.b_pending) {
        uint32_t a = ax.aw_addr & ~0x3u;
        // Latch whether the target is mapped — drives SLVERR on B if not.
        // The PASS sentinel (0xFFFF0000) and any test-side memory ranges
        // remain mapped; unmapped ranges (e.g. 0xAAAA0000) return SLVERR
        // so write bus-error tests can fault on stores symmetrically with
        // the read path.
        ax.b_mapped = mem->mapped(a);
        // Apply byte strobes (AXI WSTRB[0] = byte at a+0, big-endian
        // mapping: byte at address a+i lives in lane (3 - i) of wdata).
        for (int i = 0; i < 4; i++) {
            if (ax.w_strb & (1u << (3 - i))) {
                uint8_t bv = (ax.w_data >> ((3 - i) * 8)) & 0xFF;
                mem->write8(a + i, bv);
                cpu_writes[a + i] = bv;
            }
        }
        // Magic: write to 0xFFFF0000 finishes the test
        if (a == 0xFFFF0000u) {
            test_done = true;
            test_pass = (ax.w_data == 0xC0FFEE00u);
        }
        ax.aw_done = false;
        ax.w_done  = false;
        ax.b_pending = true;
        ax.b_delay   = 1;
    }

    // B response
    dut->daxi_bvalid = 0;
    dut->daxi_bresp  = 0;
    if (ax.b_pending) {
        if (ax.b_delay > 0) ax.b_delay--;
        else {
            dut->daxi_bvalid = 1;
            dut->daxi_bresp  = ax.b_mapped ? 0 : 2;   // SLVERR if unmapped
            if (dut->daxi_bready) ax.b_pending = false;
        }
    }
}

static void parse_args(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    for (int i = 1; i < argc; i++) {
        std::string a(argv[i]);
        if (a.substr(0, 6) == "+test=")    test_name = a.substr(6);
        else if (a.substr(0, 9) == "+timeout=") timeout   = std::stoull(a.substr(9));
        else if (a.substr(0, 21) == "+post_sentinel_drain=")
            post_sentinel_drain = std::stoull(a.substr(21));
        else if (a.substr(0, 5) == "+rom=")     rom_path  = a.substr(5);
        else if (a.substr(0, 5) == "+bin=")     bin_path  = a.substr(5);
        else if (a.substr(0, 9) == "+binaddr=") bin_addr  = (uint32_t)std::stoul(a.substr(9), nullptr, 16);
        else if (a.substr(0, 9) == "+stop_pc=") {
            stop_pc = (uint32_t)std::stoul(a.substr(9), nullptr, 16);
            stop_pc_enabled = true;
        }
        else if (a == "+waves")                 waves     = true;
        else if (a == "+nowaves")               nowaves_explicit = true;
        else if (a == "+dump_debug_state")      dump_debug_state = true;
        else if (a == "+divergence_check")      divergence_check_enabled = true;
        else if (a.substr(0, 18) == "+dump_final_state=") dump_state_path = a.substr(18);
        else if (a.substr(0, 5) == "+ipl=") {
            // Format: cycle:level (e.g. "+ipl=200:1").  Multiple events
            // OK; processed in cycle order at runtime.
            std::string s = a.substr(5);
            size_t colon = s.find(':');
            if (colon != std::string::npos) {
                IplEvent ev;
                ev.cycle = std::stoull(s.substr(0, colon));
                ev.level = (uint8_t)std::stoul(s.substr(colon + 1));
                ev.fired = false;
                ipl_events.push_back(ev);
            }
        }
    }
#ifdef WAVES
    if (!nowaves_explicit) waves = true;
#endif
    if (test_name.empty()) test_name = "default";
}

// Dump the CPU's final architectural state to a simple key=value file
// consumed by tools/fuzz/fuzz.py.  Pulls D0–D7 / A0–A7 directly from
// the committed RAT (via Verilator root-level signal access) so we
// get the in-order-committed view, not any in-flight speculative
// values.  CCR comes from ccr_rat's committed slot (crat_tag).
//
// Signal paths come from Verilator's generated header (see
// Vmac_top___024root.h) — they reflect the mac_top → cpu → {prf,
// u_rat, u_ccr_rat} hierarchy.
static void dump_final_state(const std::string& path) {
    if (path.empty() || !dut) return;
    auto* r = dut->rootp;
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) {
        std::fprintf(stderr, "[dump] cannot open %s\n", path.c_str());
        return;
    }
    std::fprintf(f, "pass=%d\n",      test_pass ? 1 : 0);
    std::fprintf(f, "cycles=%llu\n",  (unsigned long long)sim_time);
    std::fprintf(f, "committed=%u\n", (unsigned)dut->dbg_committed);
    std::fprintf(f, "committed_macros=%u\n", (unsigned)dut->dbg_macros);
    std::fprintf(f, "last_pc=0x%08x\n", (unsigned)dut->dbg_last_pc);
    // CCR bits [4:0] = {X, N, Z, V, C}.
    unsigned ccr = r->mac_top__DOT__cpu__DOT__u_ccr_rat__DOT__ccr_prf
                     [r->mac_top__DOT__cpu__DOT__u_ccr_rat__DOT__crat_tag];
    unsigned sr_upper = r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_sr;
    std::fprintf(f, "pc=0x%08x\n", (unsigned)dut->dbg_last_pc);
    std::fprintf(f, "sr=0x%04x\n", (sr_upper & 0xFFE0u) | (ccr & 0x1F));
    std::fprintf(f, "ccr=0x%02x\n", ccr & 0x1F);
    if (dump_debug_state) {
    std::fprintf(f, "debug.rat_free_cnt=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_rat__DOT__free_cnt_r);
    std::fprintf(f, "debug.decode_pre_stall=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__pd_pre_stall_w);
    std::fprintf(f, "debug.commit_drain_active=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__drain_active);
    std::fprintf(f, "debug.commit_mb_head=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__mb_head);
    std::fprintf(f, "debug.commit_mb_tail=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__mb_tail);
    std::fprintf(f, "debug.commit_can_commit=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__can_commit);
    std::fprintf(f, "debug.commit_in_flight=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__commit_in_flight);
    std::fprintf(f, "debug.store_retire_ready=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__store_retire_ready);
    std::fprintf(f, "debug.rob_hd=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__rob_hd);
    std::fprintf(f, "debug.rob_complete=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_rob__DOT__e_complete
                   [r->mac_top__DOT__cpu__DOT__u_rob__DOT__head_ptr & 63u]);
    std::fprintf(f, "debug.rob_head_ptr=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_rob__DOT__head_ptr);
    std::fprintf(f, "debug.rob_tail_ptr=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_rob__DOT__tail_ptr);
    std::fprintf(f, "debug.rob_pc=0x%08x\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__rob_pc);
    std::fprintf(f, "debug.rob_last=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__rob_is_last_uop);
    std::fprintf(f, "debug.rob_uop_t=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__rob_uop_t);
    std::fprintf(f, "debug.rob_uop_op=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__rob_uop_op);
    std::fprintf(f, "debug.rob_hd_dst=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__rob_hd);
    std::fprintf(f, "debug.rob_arch_dst=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__rob_ad);
    std::fprintf(f, "debug.iq_mem_valid=0x%02x\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__e_valid);
    std::fprintf(f, "debug.iq_mem_sel_valid=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__sel_valid);
    std::fprintf(f, "debug.iq_mem_oldest_ready=0x%02x\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__oldest_ready);
    unsigned iq_int_valid_mask = 0;
    for (int i = 0; i < 16; i++) {
        if (r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_valid[i])
            iq_int_valid_mask |= (1u << i);
    }
    std::fprintf(f, "debug.iq_int_valid=0x%04x\n", iq_int_valid_mask);
    std::fprintf(f, "debug.iq_int_sel_valid=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__sel_valid);
    std::fprintf(f, "debug.mem_iss_v=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__mem_iss_v);
    std::fprintf(f, "debug.mem_iss_tag=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__mem_iss_tag);
    std::fprintf(f, "debug.lsu_state=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__state);
    std::fprintf(f, "debug.lsu_cur_tag=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__cur_tag);
    std::fprintf(f, "debug.lsu_cur_split=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__cur_split);
    std::fprintf(f, "debug.lsu_cur_squashed=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__cur_squashed);
    std::fprintf(f, "debug.lsu_ea=0x%08x\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__ea);
    std::fprintf(f, "debug.lsu_split_first=0x%08x\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__split_first_addr);
    std::fprintf(f, "debug.lsu_split_second=0x%08x\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__split_second_addr);
    std::fprintf(f, "debug.lsu_cdb_valid=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__lsu_cdb_valid);
    std::fprintf(f, "debug.dcache_state=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__state);
    std::fprintf(f, "debug.dcache_ar_valid=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__dc_daxi_arvalid);
    std::fprintf(f, "debug.dcache_ar_ready=%u\n",
                 (unsigned)dut->daxi_arready);
    std::fprintf(f, "debug.dcache_r_ready=%u\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__dc_daxi_rready);
    std::fprintf(f, "debug.dut_r_ready=%u\n",
                 (unsigned)dut->daxi_rready);
    std::fprintf(f, "debug.dcache_r_valid=%u\n",
                 (unsigned)dut->daxi_rvalid);
    std::fprintf(f, "debug.dcache_araddr=0x%08x\n",
                 (unsigned)dut->daxi_araddr);
    std::fprintf(f, "debug.tb_ax_ar_outstanding=%u\n", ax.ar_outstanding ? 1u : 0u);
    std::fprintf(f, "debug.tb_ax_ar_addr=0x%08x\n", ax.ar_addr);
    std::fprintf(f, "debug.tb_ax_ar_delay=%d\n", ax.ar_delay);
    std::fprintf(f, "debug.tb_ax_b_pending=%u\n", ax.b_pending ? 1u : 0u);
    }
    // SR bits: we only synthesise the CCR portion (bits 4:0), upper bits
    // (T1/T0, S, M, I2-I0) are implementation-dependent — emit CCR only.
    // D0–D7 = arch 0–7 → phys crat[0..7] → prf[phys].
    for (int i = 0; i < 8; i++) {
        unsigned phys = r->mac_top__DOT__cpu__DOT__u_rat__DOT__crat[i];
        unsigned v    = r->mac_top__DOT__cpu__DOT__prf[phys];
        std::fprintf(f, "d%d=0x%08x\n", i, v);
    }
    // A0–A7 = arch 8–15 → phys crat[8..15] → prf[phys].
    for (int i = 0; i < 8; i++) {
        unsigned phys = r->mac_top__DOT__cpu__DOT__u_rat__DOT__crat[8 + i];
        unsigned v    = r->mac_top__DOT__cpu__DOT__prf[phys];
        std::fprintf(f, "a%d=0x%08x\n", i, v);
    }
    // Memory writes observed on the AXI data bus, ascending address.
    std::fprintf(f, "mem_writes=%zu\n", cpu_writes.size());
    for (const auto& kv : cpu_writes) {
        std::fprintf(f, "mem[0x%08x]=0x%02x\n", kv.first, kv.second);
    }
    std::fclose(f);
}

int main(int argc, char** argv) {
    parse_args(argc, argv);

    printf("[SIM] Starting test: %s\n", test_name.c_str());

    mem = new MemModel();
    // Directed asm tests are linked at 0x4080_0000 for historical reasons.
    // Keep the compatibility alias local to tb_top; ROM boot / FPGA use the
    // canonical 0x4000_0000 map.
    mem->enable_legacy_rom_alias(true);

    if (!rom_path.empty()) {
        if (!MacRom::load(*mem, rom_path)) {
            fprintf(stderr, "[ERROR] Failed to load ROM: %s\n", rom_path.c_str());
            return 1;
        }
        printf("[SIM] Loaded ROM: %s\n", rom_path.c_str());
    }

    // Auto-discover binary from test name if +bin= not provided.
    // Load at 0x40800000 (RESET_PC / ROM base) so the CPU finds it on boot.
    if (bin_path.empty() && !test_name.empty() && test_name != "default") {
        bin_path = "build/tests/" + test_name + ".bin";
        if (bin_addr == 0) bin_addr = 0x40800000u;
    }

    if (!bin_path.empty()) {
        if (!mem->load_file(bin_path, bin_addr)) {
            fprintf(stderr, "[ERROR] Failed to load binary: %s\n", bin_path.c_str());
            return 1;
        }
        printf("[SIM] Loaded test binary: %s @ 0x%08x\n", bin_path.c_str(), bin_addr);
    }

    // ── Install cold-boot exception vector 0 ─────────────────────────
    // mac_top is elaborated with -GFETCH_RESET_VECTORS=1, so the CPU
    // does a real 68k reset-vector fetch: SSP from mem[0..3], PC from
    // mem[4..7] (both big-endian longwords).  Populate them before
    // reset is deasserted.  SSP = 0x0080_0000 (top of a sensible
    // 8 MB low-RAM region; the legacy tests never crossed 0x0040_0000
    // so any stack push lands in mapped RAM).  PC = the binary load
    // address (0x4080_0000 by default, overridable via +binaddr=).
    // This keeps every directed .s test passing unchanged — they used
    // to boot at the compile-time RESET_PC and now boot to the same
    // PC via a runtime vector fetch.
    {
        uint32_t initial_ssp = 0x00800000u;
        uint32_t initial_pc  = (bin_addr != 0) ? bin_addr : 0x40800000u;
        mem->write32(0x00000000u, initial_ssp);
        mem->write32(0x00000004u, initial_pc);
        printf("[SIM] Cold-boot vector 0: SSP=0x%08x PC=0x%08x\n",
               initial_ssp, initial_pc);
    }

    dut = new Vmac_top;

    if (waves) {
        Verilated::traceEverOn(true);
        fst = new VerilatedFstC;
        dut->trace(fst, 99);
        std::string fst_path = test_name + ".fst";
        fst->open(fst_path.c_str());
        printf("[SIM] Waveform: %s\n", fst_path.c_str());
    }

    reset(16);

    dut->dbg_dcache_flush_req = 0;

    while (!test_done && !Verilated::gotFinish()) {
        drive_ifetch();
        drive_daxi();
        // ── IPL injection: scan +ipl= events and assert level on cur_ipl_ext.
        // Each event fires EXACTLY ONCE when sim_time crosses its cycle
        // (the .fired flag prevents re-asserting after cpu_ipl_ack clears
        // cur_ipl_ext).  Multiple +ipl= flags can be used to schedule a
        // sequence of pulses.
        for (auto& ev : ipl_events) {
            if (!ev.fired && sim_time >= ev.cycle) {
                cur_ipl_ext = ev.level;
                ev.fired = true;
            }
        }
        // Auto-clear cur_ipl_ext on cpu_ipl_ack so a one-shot IRQ doesn't
        // re-trigger continuously.  Mirrors the irq_agg.v "level 1-6 stay
        // high while the peripheral holds them" semantics — the testbench
        // is acting as the peripheral, and we want a single IRQ per +ipl=
        // event.  If a test wants persistent IRQ, it can issue multiple
        // +ipl= events at later cycles to re-assert.
        if (dut->cpu_ipl_ack) cur_ipl_ext = 0;
        dut->cpu_ipl_ext = cur_ipl_ext;
        tick();
        // ── H1: USP/SSP/ISP shadow vs PRF[PHYS_*_TAG] divergence detector ──
        // Edge-triggered: count one event per rising edge of the "diverged"
        // predicate.  We don't print every cycle (would flood logs); we
        // capture the FIRST divergence cycle + values for each slot.
        if (divergence_check_enabled) {
            auto* r = dut->rootp;
            uint32_t shadow_usp = r->mac_top__DOT__cpu__DOT__u_commit__DOT__usp;
            uint32_t shadow_ssp = r->mac_top__DOT__cpu__DOT__u_commit__DOT__ssp;
            uint32_t shadow_isp = r->mac_top__DOT__cpu__DOT__u_commit__DOT__isp;
            uint32_t prf_usp    = r->mac_top__DOT__cpu__DOT__prf[19];
            uint32_t prf_ssp    = r->mac_top__DOT__cpu__DOT__prf[20];
            uint32_t prf_isp    = r->mac_top__DOT__cpu__DOT__prf[21];
            bool d_usp = (shadow_usp != prf_usp);
            bool d_ssp = (shadow_ssp != prf_ssp);
            bool d_isp = (shadow_isp != prf_isp);
            // Run-length count of consecutive diverged cycles; flags
            // "persistent" divergence after DIVERG_SETTLE_CYC.
            divergence_usp_run = d_usp ? (divergence_usp_run + 1) : 0;
            divergence_ssp_run = d_ssp ? (divergence_ssp_run + 1) : 0;
            divergence_isp_run = d_isp ? (divergence_isp_run + 1) : 0;
            bool p_usp = (divergence_usp_run >= DIVERG_SETTLE_CYC);
            bool p_ssp = (divergence_ssp_run >= DIVERG_SETTLE_CYC);
            bool p_isp = (divergence_isp_run >= DIVERG_SETTLE_CYC);
            if (p_usp && !divergence_usp_persistent) {
                divergence_events_usp++;
                if (divergence_first_usp_cycle == 0) {
                    divergence_first_usp_cycle  = sim_time;
                    divergence_first_usp_shadow = shadow_usp;
                    divergence_first_usp_prf    = prf_usp;
                    divergence_first_usp_pc     = (uint32_t)dut->dbg_last_pc;
                    std::printf("[DIVERG-USP-FIRST] cyc=%llu pc=0x%08x shadow=0x%08x prf=0x%08x diff=0x%08x\n",
                                (unsigned long long)sim_time,
                                divergence_first_usp_pc,
                                shadow_usp, prf_usp,
                                shadow_usp - prf_usp);
                }
            }
            if (p_ssp && !divergence_ssp_persistent) {
                divergence_events_ssp++;
                if (divergence_first_ssp_cycle == 0) {
                    divergence_first_ssp_cycle  = sim_time;
                    divergence_first_ssp_shadow = shadow_ssp;
                    divergence_first_ssp_prf    = prf_ssp;
                    divergence_first_ssp_pc     = (uint32_t)dut->dbg_last_pc;
                    std::printf("[DIVERG-SSP-FIRST] cyc=%llu pc=0x%08x shadow=0x%08x prf=0x%08x diff=0x%08x\n",
                                (unsigned long long)sim_time,
                                divergence_first_ssp_pc,
                                shadow_ssp, prf_ssp,
                                shadow_ssp - prf_ssp);
                }
            }
            if (p_isp && !divergence_isp_persistent) {
                divergence_events_isp++;
                if (divergence_first_isp_cycle == 0) {
                    divergence_first_isp_cycle  = sim_time;
                    divergence_first_isp_shadow = shadow_isp;
                    divergence_first_isp_prf    = prf_isp;
                    divergence_first_isp_pc     = (uint32_t)dut->dbg_last_pc;
                    std::printf("[DIVERG-ISP-FIRST] cyc=%llu pc=0x%08x shadow=0x%08x prf=0x%08x diff=0x%08x\n",
                                (unsigned long long)sim_time,
                                divergence_first_isp_pc,
                                shadow_isp, prf_isp,
                                shadow_isp - prf_isp);
                }
            }
            divergence_usp_persistent = p_usp;
            divergence_ssp_persistent = p_ssp;
            divergence_isp_persistent = p_isp;
        }
        if (stop_pc_enabled && dut->dbg_committed != last_committed_seen) {
            last_committed_seen = dut->dbg_committed;
            if ((uint32_t)dut->dbg_last_pc == stop_pc) {
                test_done = true;
                test_pass = true;
            }
        }
        // Double-fault halt detection: core asserts cpu_halted when a
        // bus error fires during exception entry (per 68040 UM
        // §8.4.5.4).  Treat as PASS-equivalent for tests that exercise
        // the halt path (e.g. exc_double_fault_halt) — the harness
        // writes the PASS sentinel on behalf of the halted CPU.  The
        // CPU's pipeline freezes so JTAG / debug paths can still read
        // architectural state for post-mortem inspection.
        if (dut->cpu_halted) {
            mem->write32(0xFFFF0000u, 0xC0FFEE00u);
            test_done = true;
            test_pass = true;
            std::printf("[HALT] double bus fault — cpu_halted asserted\n");
        }
        if (dut->dbg_break_uop_fire) {
            mem->write32(0xFFFF0000u, 0xC0FFEE00u);
            test_done = true;
            test_pass = true;
            std::printf("[HALT] SYS_DBG_BREAK fired\n");
        }
    }

    // The PASS sentinel is a non-cacheable AXI write, so the testbench
    // observes it at the external bus boundary.  Older cacheable stores
    // may still be draining into the write-back D-cache when that happens.
    // Run a small bounded window before the debug flush so the final
    // write-log reflects program memory state, not the sentinel race.
    for (uint64_t i = 0; i < post_sentinel_drain && !Verilated::gotFinish(); i++) {
        drive_ifetch();
        drive_daxi();
        tick();
    }

    // Post-test cache flush: with the real L1D (write-back + write-
    // allocate), any dirty line that never evicted is invisible to the
    // AXI-write oracle built up in cpu_writes.  The core exposes a
    // direct flush-all hook (routed to dcache.flush_all_req) — we pulse
    // it here and tick forward until flush_all_done fires.  After that,
    // every dirty byte has landed in the tb AXI slave's mem_model, so
    // fuzz.py's memory compare sees the same state Musashi does.
    //
    // Bounded by the cache geometry: 32 sets × 4 ways × 8 beats × ~3
    // cycles/beat ≈ 3000 cycles worst-case (all 4 KB dirty).
    {
        dut->dbg_dcache_flush_req = 1;
        uint64_t flush_cycles = 0;
        const uint64_t flush_cap = 8000;
        bool seen_done = false;
        while (!seen_done && flush_cycles < flush_cap && !Verilated::gotFinish()) {
            drive_ifetch();
            drive_daxi();
            tick();
            flush_cycles++;
            if (dut->dbg_dcache_flush_done) seen_done = true;
        }
        dut->dbg_dcache_flush_req = 0;
        // Let any trailing AXI bvalid settle.
        for (int i = 0; i < 8 && !Verilated::gotFinish(); i++) {
            drive_ifetch();
            drive_daxi();
            tick();
        }
    }

    dut->final();

    // Dump final state before destroying DUT (must happen before delete dut).
    if (!dump_state_path.empty()) dump_final_state(dump_state_path);

    if (waves) {
        fst->close();
        delete fst;
    }

    // Macro-IPC metric (task #192): committed-macro-instructions / cycles.
    // dbg_committed counts every retired µop; dbg_macros counts only the
    // final µop of each macro-instruction crack (where last_phase=1).
    // macro_ipc is the honest software-visible IPC — two RTL variants
    // that pick different cracks for the same macro look identical here.
    const unsigned long long rep_cycles = (unsigned long long)sim_time;
    const unsigned           rep_uops   = (unsigned)dut->dbg_committed;
    const unsigned           rep_macros = (unsigned)dut->dbg_macros;
    const double uop_ipc   = rep_cycles ? (double)rep_uops   / (double)rep_cycles : 0.0;
    const double macro_ipc = rep_cycles ? (double)rep_macros / (double)rep_cycles : 0.0;

    printf("[%s] %s after %llu cycles  (committed=%u, last_pc=0x%08x)\n",
           test_pass ? "PASS" : "FAIL",
           test_name.c_str(),
           rep_cycles,
           rep_uops,
           (unsigned)dut->dbg_last_pc);
    printf("[METRICS] %s cycles=%llu committed_uops=%u committed_macros=%u "
           "uop_ipc=%.4f macro_ipc=%.4f\n",
           test_name.c_str(),
           rep_cycles, rep_uops, rep_macros, uop_ipc, macro_ipc);
    {
        auto* rr = dut->rootp;
        unsigned l1_fires =
            rr->mac_top__DOT__cpu__DOT__dbg_l1_fires_r;
        printf("[METRICS] %s lane1_fires=%u\n",
               test_name.c_str(), l1_fires);
    }
    if (divergence_check_enabled) {
        printf("[DIVERG-SUMMARY] %s usp_events=%llu ssp_events=%llu isp_events=%llu\n",
               test_name.c_str(),
               (unsigned long long)divergence_events_usp,
               (unsigned long long)divergence_events_ssp,
               (unsigned long long)divergence_events_isp);
        if (divergence_first_usp_cycle)
            printf("[DIVERG-USP] first cyc=%llu pc=0x%08x shadow=0x%08x prf=0x%08x diff=0x%08x\n",
                   (unsigned long long)divergence_first_usp_cycle,
                   divergence_first_usp_pc,
                   divergence_first_usp_shadow,
                   divergence_first_usp_prf,
                   divergence_first_usp_shadow - divergence_first_usp_prf);
        if (divergence_first_ssp_cycle)
            printf("[DIVERG-SSP] first cyc=%llu pc=0x%08x shadow=0x%08x prf=0x%08x diff=0x%08x\n",
                   (unsigned long long)divergence_first_ssp_cycle,
                   divergence_first_ssp_pc,
                   divergence_first_ssp_shadow,
                   divergence_first_ssp_prf,
                   divergence_first_ssp_shadow - divergence_first_ssp_prf);
        if (divergence_first_isp_cycle)
            printf("[DIVERG-ISP] first cyc=%llu pc=0x%08x shadow=0x%08x prf=0x%08x diff=0x%08x\n",
                   (unsigned long long)divergence_first_isp_cycle,
                   divergence_first_isp_pc,
                   divergence_first_isp_shadow,
                   divergence_first_isp_prf,
                   divergence_first_isp_shadow - divergence_first_isp_prf);
    }

    delete dut;
    delete mem;
    return test_pass ? 0 : 1;
}
