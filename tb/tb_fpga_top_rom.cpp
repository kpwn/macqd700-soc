#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <climits>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include <verilated.h>
#include <verilated_save.h>
#if VM_TRACE_FST
#include <verilated_fst_c.h>
#endif
#include "Vfpga_top.h"
#include "Vfpga_top___024root.h"
#include "models/rom_patch_sets.h"
#include "models/sd_card_spi.h"

// ─────────────────────────────────────────────────────────────────────
// CPU-hierarchy member accessors (repo-split socketization, Phase 5.3;
// widened to a 3rd CPU value, task #272).
//
// fpga_top's CPU socket `u_cpu` is now a socket-bound module.  Under the
// socketed build (CPU=m68k, -DCPU_M68K), u_cpu binds m68k_axi_wrapper,
// which instantiates m68k_core as its own `u_cpu` and keeps the debug
// taps (dbg_committed / dbg_pc / dbg_boundary_*) as wrapper-internal
// wires.  So:
//   * CPU pipeline internals (prf, crat, rob_*, u_commit.*, u_*mmu.*,
//     u_dcache.*) moved from fpga_top.u_cpu.*  ->  fpga_top.u_cpu.u_cpu.*
//   * debug taps moved from   fpga_top.dbg_*   ->  fpga_top.u_cpu.dbg_*
//
// CPUI_REF(r, member) / DBGT_REF(r, member) are the call-site form (used
// as e.g. `CPUI_REF(r, prf)[15]`, replacing the old `r->CPUI(prf)[15]`)
// so the non-CPU_M68K arm below can route through a real C++ object
// instead of a `->`-glued Verilated member name that may not exist.
//
// Historically the `#else` arm here just reproduced the *legacy*
// (pre-socketization) non-socket build's flat member names
// (`fpga_top.u_cpu.*` / `fpga_top.dbg_*` directly) on the assumption
// that was still what CPU=stub built. It never was, post-socketization
// — cpu_stub.v has none of the v1-only prf/crat/u_commit/* hierarchy —
// and CPU=m68k040's M68kSocketTop (flat, SpinalHDL-generated, no such
// hierarchy either) makes the same gap concrete: `tb-fpga-top-rom
// CPU=m68k040` doesn't even compile against the stale `#else` arm (task
// #272). None of this file's deep CPU-internal/commit-boundary
// diagnostics (PC-boundary tracing, arch-register dumps, exception/
// MacsBug/dual-commit detectors, state-replay arch pokes) have an
// equivalent on either of those two CPU values today — that would need
// real observability plumbed through the socket (e.g. the `dbg_axi`
// debug-CSR slave), which is future testbench work, not this task's
// scope (task #272 only needs reset + first-fetch + no-wedge). So for
// CPU_M68K undefined, every CPUI_REF/DBGT_REF read returns a constant 0
// (a false/never-fires condition for every one of those detectors) and
// every write is silently discarded via `DeadSink` below — this makes
// the whole (currently CPU_M68K-only-correct) diagnostic surface COMPILE
// cleanly and behave as inert/absent, rather than reading stale garbage
// off the wrong netlist location.
#ifdef CPU_M68K
#define CPUI_REF(r, member) ((r)->fpga_top__DOT__u_cpu__DOT__u_cpu__DOT__##member)
#define DBGT_REF(r, member) ((r)->fpga_top__DOT__u_cpu__DOT__dbg_##member)
#else
struct DeadSink {
    DeadSink& operator[](size_t) { return *this; }
    template <class T> DeadSink& operator=(T) { return *this; }
    operator uint64_t() const { return 0; }
};
static DeadSink g_dead_sink;
#define CPUI_REF(r, member) (g_dead_sink)
#define DBGT_REF(r, member) (g_dead_sink)
#endif

static Vfpga_top* dut = nullptr;
static uint64_t sim_time = 0;

// Waveform — gated to a retire-count window so the FST stays small.
//   +wave_path=<file>           output FST path (default fpga_top.fst)
//   +wave_start_retired=<n>     open + start dumping when retired >= n
//   +wave_end_retired=<m>       close + stop dumping when retired >= m
#if VM_TRACE_FST
static VerilatedFstC* g_fst = nullptr;
static std::string    g_wave_path;
static uint64_t       g_wave_start_retired = 0xFFFFFFFFFFFFFFFFull;
static uint64_t       g_wave_end_retired   = 0xFFFFFFFFFFFFFFFFull;
static bool           g_wave_open          = false;
#endif
static uint64_t       g_vbl_log_max        = 0;
static uint64_t       g_vbl_log_count      = 0;
static uint8_t        g_prev_dafb_vbl_level = 0;
// task #272: always-on axi_i (instruction-fetch AR channel) rise
// counter/last-address, independent of the CPU-internal dbg_committed/
// boundary_* machinery -- see the tick()-resident probe and the
// end-of-run summary line for how these get used.
static uint64_t        g_ifetch_arvalid_count = 0;
static bool            g_ifetch_prev_arvalid  = false;
static uint32_t        g_ifetch_last_addr     = 0;
static uint64_t        g_ifetch_rsp_count     = 0;
#ifdef CPU_M68K040
// The legacy fpga_top debug counters are tied off for the socketized 040 core.
// Keep the ROM harness's generic +max_insts contract working from the core's
// real retirement pulse until the top-level diagnostics are fully unified.
static uint64_t        g_cpu040_retired_count = 0;
static uint32_t        g_cpu040_last_pc       = 0;
// +pc_dump_path= writer state, hoisted to file scope so the CPU_M68K040
// per-cycle diagnostic block inside tick() (a free function, no access to
// main()'s locals) can drive it directly off RobPlugin_logic_traceVec_0/1
// -- the only two macro-retire-order signals that are actually simPublic
// and live for the whole run on this core.  This replaces the
// debugMacroRetirePc_0/1-driven +pc_dump path below, which was found
// (this investigation) to silently stop firing after roughly the first
// 20 retirements of any run: its own valid signals only pulse during an
// initial startup burst, not the whole run, on a CPU_M68K040 build. That
// left EVERY prior +pc_dump_path capture on this build silently truncated
// to a few dozen lines with no error -- a real, previously undetected gap
// in this exact testbench feature, not a one-off. traceVec_1 (the second,
// dual-commit lane) was ALSO previously dropped from the old capped
// "[retire-pc]" stderr diagnostic (only traceVec_0 was printed), which
// would have silently skipped every head+1 macro in a dual-retire cycle;
// fixed here too.
static FILE*            g_pc_dump_fp    = nullptr;
static uint64_t         g_pc_dump_count = 0;
static uint64_t         g_pc_dump_max   = 0;
#endif
// Part 73 IPC investigation: per-cycle store-queue/D-cache probe. Prints
// one line per cycle, for the first `g_sq_trace_cycles` cycles of THIS
// process's run (i.e. right after a +load_checkpoint resume, if used),
// gated by +sq_trace_cycles=<n>. Deliberately separate from +pc_dump_path
// (which only fires on an actual retire) -- this fires EVERY cycle so
// stalls between retirements are visible, not just the retire events
// themselves. Declared unconditionally (not CPU_M68K040-gated) since the
// generic plusarg-parsing loop in main() below isn't itself CPU-gated.
static uint64_t         g_sq_trace_cycles  = 0;
static uint64_t         g_sq_trace_emitted = 0;

// Part 75 pipeline-stall investigation: broader per-cycle whitebox probe
// than Part 73's store-queue-only +sq_trace_cycles -- adds ROB head/tail/
// count/flush, IQ occupancy+per-scoreboard busy vectors, SQ head/tail
// (in addition to Part 73's alloc/full/empty/drain fields), I-cache MSHR/
// hit state, and D-cache outstanding-store/serial-store/barrier state, all
// pre-existing simPublic() signals (RobPlugin/IssueQueuePlugin/StoreQueue/
// IcachePlugin/DcachePlugin), no new RTL taps. Gated by
// +pipe_trace_cycles=<n> (how many cycles to emit) and
// +pipe_trace_path=<path> (dedicated output file so this doesn't interleave
// with other stderr diagnostics); prints nothing if the path isn't given.
static uint64_t         g_pipe_trace_cycles  = 0;
static uint64_t         g_pipe_trace_emitted = 0;
static FILE*            g_pipe_trace_fp      = nullptr;

// 2026-09-02 follow-up session (BUG_calibration_word_misplaced_0d00.md
// Part 8/9 root-cause pursuit): dedicated per-cycle IRQ-recognition probe.
// Neither +sq_trace_cycles nor +pipe_trace_cycles carries RobPlugin's own
// interrupt-recognition internals (interruptPending/normalIrqGate/
// iplActive/excIdle/flushing/branchRedirect/p0.first/p0.pc) or VIA1's live
// IFR/IER state -- exactly the signal set Part 9's ILA captures used, but
// those only ever caught two clean-negative windows (never live-captured
// the actual "pending the whole loop, never recognized" moment Part 8
// found on real hardware). Gated by +irq_trace_cycles=<n>/
// +irq_trace_path=<path>, same convention as +pipe_trace_*; prints
// nothing if the path isn't given.
static uint64_t         g_irq_trace_cycles  = 0;
static uint64_t         g_irq_trace_emitted = 0;
static FILE*            g_irq_trace_fp      = nullptr;

// ROM-mirror decode-fold check (SIM_L2C_ENABLE only -- see
// ifa_araddr_folded's comment in rtl/soc/fpga_top_ddr.vh). Every real
// fetch AR handshake's raw ifa_araddr is cross-checked in C++, using the
// exact same arithmetic as that RTL wire, against the wire's own live
// value (fpga_top__DOT__ifa_araddr_folded) -- proving the fold that
// actually feeds l2c's dedicated fetch port on cpu040's Level-A path is
// live and correct, without needing to decode ifa_rdata's byte/lane
// layout at all. g_rom_fold_mirror_hits counts handshakes that landed
// outside the primary [AXI_ROM_MIRROR_BASE, +AXI_ROM_IMAGE_SIZE) alias
// -- i.e. genuinely exercised the wraparound fold, not just its identity
// case -- so a summary of 0 here would mean the check never got real
// coverage even though it ran clean.
#ifdef SIM_L2C_ENABLE
static uint64_t g_rom_fold_checks       = 0;
static uint64_t g_rom_fold_mismatches   = 0;
static uint64_t g_rom_fold_mirror_hits  = 0;
#endif
static uint8_t        g_prev_via1_ca1_in    = 0;
static uint8_t        g_prev_via1_irq_pb    = 0;

// realboot verification (task background: real boot_fsm SD-streaming +
// mirror-to-0x0 path, no DDR-backdoor preload).  Forward-declared here,
// defined after ddr_read_phys32() below, and called from tick() -- one
// stderr line the moment boot_rom_ready rises, dumping the DDR model's
// content at both the mirror (0x0) and native (AXI_ROM_BASE=0x40000000)
// windows.  This is a host-side READ of the model for verification only
// (never a write) -- it does not influence CPU/boot_fsm behaviour, and
// is the cheapest way to confirm on the record that the bytes the CPU is
// about to fetch came from boot_fsm's real mirror write, not stale/zero
// DRAM, without needing a full waveform dump.
static void boot_rom_ready_probe();
static bool g_prev_boot_rom_ready = false;

// Cycle-precise IRQ injection (task #41).  Globals + an irq_cycle_tick()
// driven from tick() so the arm/inject runs EVERY sys_clk — including
// the state-replay boot-tick phase that executes before main()'s loop.
// The +irq_at_inst / +irq_at_pc paths (main-loop-resident) cannot reach
// a window like the A-line dispatcher prologue, which runs entirely in
// the boot-tick phase.  irq_cycle_tick() can.  g_irqc_at is in sim_time
// units (tick() advances sim_time by 2 per sys_clk).
static uint64_t       g_irqc_at        = 0;
static bool           g_irqc_set       = false;
static uint8_t        g_irqc_lvl       = 0;
static uint64_t       g_irqc_total     = 1;
static uint64_t       g_irqc_gap       = 1;
static bool           g_irqc_armed     = false;
static uint64_t       g_irqc_remaining = 0;
static uint64_t       g_irqc_next_ret  = 0;

// ASC audio + bus-event capture (set up by main() from +audio_dump=
// and +asc_event_dump= args).  Globals because tick() is the natural
// hook point for per-clock sampling.
static FILE*    g_audio_dump_fp        = nullptr;
static uint64_t g_audio_dump_max       = 0;        // 0 = unlimited
static uint64_t g_audio_dump_count     = 0;
static FILE*    g_asc_event_dump_fp    = nullptr;
static uint64_t g_asc_event_dump_max   = 0;
static uint64_t g_asc_event_dump_count = 0;
// Edge-detect snapshots for audio_sample_valid + ASC bus accesses.
static uint8_t  g_prev_audio_sample_valid = 0;
static uint8_t  g_prev_pb_asc_wr          = 0;
static uint8_t  g_prev_pb_asc_rd          = 0;
static uint64_t g_retired_count           = 0;     // updated by main loop

// ── SD card model ────────────────────────────────────────────────────
// Functional SPI-mode SD card backed by a disk image (tb/models/
// sd_card_spi.h).  Off unless +sd_image= / +sd_card_image= is given, in
// which case sd_miso stays tied high exactly as before.  Driven from
// tick() so it is live during the pre-reset idle ticks and the 1000-tick
// boot phase as well as the main loop.
static SdCardSpi g_sd;
static bool      g_sd_enabled = false;

// SD-provider (sd_ctrl_scsi) request tracing.  Enabled by +sd_trace: one
// line per request start and per completion, with sd_ctrl.v's err_cause.
// This is the layer BETWEEN scsi.v's virtual HDD and the SPI pins, and it
// is where a "READ error sense=3 asc=11" from scsi.v actually originates.
static uint64_t g_sdreq_log_max = 0;
static uint64_t g_sdreq_log_n   = 0;
static uint8_t  g_prev_scsi_go   = 0;

static void sd_req_tick() {
    if (g_sdreq_log_n >= g_sdreq_log_max) return;
    auto* r = dut->rootp;
    const uint8_t go = r->fpga_top__DOT__core_scsi_go;
    if (go && !g_prev_scsi_go) {
        g_sdreq_log_n++;
        std::fprintf(stderr,
            "[sdreq] t=%llu GO cmd_type=%u lba=0x%08x blocks=%u\n",
            (unsigned long long)sim_time,
            (unsigned)r->fpga_top__DOT__core_scsi_cmd_type,
            (unsigned)r->fpga_top__DOT__core_scsi_lba,
            (unsigned)r->fpga_top__DOT__core_scsi_block_count);
    }
    g_prev_scsi_go = go;
    if (r->fpga_top__DOT__core_scsi_done) {
        g_sdreq_log_n++;
        std::fprintf(stderr,
            "[sdreq] t=%llu DONE error=%u err_cause=%u cur_cmd=%u "
            "block_idx=%u poll_cnt=%u\n",
            (unsigned long long)sim_time,
            (unsigned)r->fpga_top__DOT__core_scsi_error,
            (unsigned)r->fpga_top__DOT__core_scsi_err_cause,
            (unsigned)r->fpga_top__DOT__core_scsi_dbg_cur_cmd,
            (unsigned)r->fpga_top__DOT__core_scsi_dbg_block_idx,
            (unsigned)r->fpga_top__DOT__core_scsi_dbg_last_poll_cnt);
        std::fflush(stderr);
    }
}

// 53C96 pseudo-DMA drain probe.  +c96_probe_at_pc=<pc> prints, on every
// retire of that PC, the state that decides whether the TurboSCSI DMA
// aperture grants DRQ: FIFO occupancy, config3 (LBTM), TC0, transfer
// residue, the live DRQ, and the DAFB bus-1 control word that carries
// scsi_ctrl_in.  This is the exact tuple the 0x4089931c chunk-drain bus
// error is decided by.
static uint32_t g_c96_probe_pc  = 0xFFFFFFFFu;
static uint64_t g_c96_probe_max = 0;
static uint64_t g_c96_probe_n   = 0;

static void c96_probe(uint32_t pc) {
    if (pc != g_c96_probe_pc || g_c96_probe_n >= g_c96_probe_max) return;
    auto* r = dut->rootp;
    g_c96_probe_n++;
    std::fprintf(stderr,
        "[c96] pc=0x%08x fifo_pos=%u config3=0x%02x tc0=%u tcounter=%u "
        "xfr_left=%u drq=%u dafb_ctrl=0x%03x\n",
        (unsigned)pc,
        (unsigned)r->fpga_top__DOT__u_scsi__DOT__c96_fifo_pos,
        (unsigned)r->fpga_top__DOT__u_scsi__DOT__c96_config3,
        (unsigned)r->fpga_top__DOT__u_scsi__DOT__c96_tc0_set,
        (unsigned)r->fpga_top__DOT__u_scsi__DOT__c96_tcounter,
        (unsigned)r->fpga_top__DOT__u_scsi__DOT__c96_xfr_left,
        (unsigned)r->fpga_top__DOT__u_scsi__DOT__drq_c96,
        (unsigned)r->fpga_top__DOT__dafb_scsi0_ctrl_pb);
    std::fflush(stderr);
}

static void sd_summary() {
    if (!g_sd_enabled) return;
    std::fprintf(stderr,
        "[sd] commands=%llu read_blocks=%llu write_blocks=%llu "
        "spi_bytes=%llu\n",
        (unsigned long long)g_sd.cmd_count,
        (unsigned long long)g_sd.read_blocks,
        (unsigned long long)g_sd.write_blocks,
        (unsigned long long)g_sd.bytes_clocked);
    std::fflush(stderr);
}

static constexpr uint32_t ROM_BYTES = 0x00400000u;
static constexpr uint32_t RAM_BYTES = 0x04000000u;
static constexpr uint32_t ROM_BEAT_BASE = RAM_BYTES / 16u;
static constexpr uint32_t FLOPPY_LOOP_START = 0x408009ceu;
static constexpr uint32_t FLOPPY_LOOP_END   = 0x408009feu;
// Q700 ROM no-boot-disk poll loop (canonical "?-floppy" idle).  PC
// matches docs/rom_boot_bringup.md §"ROM outcome stops" / DISK_PROMPT_PC
// in tb_rom_boot.cpp.  Hitting and STAYING in this loop is the
// success criterion for the iwm-handshake fix: the ROM concludes
// "no boot device, please insert disk" instead of falling into
// MacsBug at 0x40849b00..0x4084b800.
static constexpr uint32_t DISK_PROMPT_PC = 0x40898e3eu;
static constexpr int FRAME_W = 640;
static constexpr int FRAME_H = 480;

struct RomPatch {
    uint32_t off;
    uint8_t value;
    const char* name;
};

static uint32_t read_arch_reg(int idx) {
    auto* r = dut->rootp;
    const unsigned phys = CPUI_REF(r, u_rat__DOT__crat)[idx];
    return CPUI_REF(r, prf)[phys];
}

static bool parse_u32(const std::string& s, uint32_t& out) {
    errno = 0;
    char* end = nullptr;
    unsigned long v = std::strtoul(s.c_str(), &end, 0);
    if (errno || end == s.c_str() || *end != '\0' || v > UINT32_MAX)
        return false;
    out = static_cast<uint32_t>(v);
    return true;
}

static bool patch_sets_include(const std::string& patch_sets,
                               const std::string& needle) {
    size_t start = 0;
    while (start <= patch_sets.size()) {
        const size_t comma = patch_sets.find(',', start);
        const std::string one = patch_sets.substr(
            start, comma == std::string::npos ? std::string::npos : comma - start);
        if (one == needle) return true;
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return false;
}

static bool patch_sets_need_ramtest_fastfill(const std::string& patch_sets) {
    return patch_sets_include(patch_sets, "ramtest-mame-state") ||
           patch_sets_include(patch_sets, "mame-ramtest-fast") ||
           patch_sets_include(patch_sets, "mame-fastdiag") ||
           patch_sets_include(patch_sets, "mame-q700-fastdiag") ||
           patch_sets_include(patch_sets, "mame-firstlight") ||
           patch_sets_include(patch_sets, "mame-q700-firstlight");
}

static const char* probe_pc_name(uint32_t pc) {
    switch (pc) {
    case 0x40802f36u: return "desc_accept_common";
    case 0x40802f96u: return "q700_desc_accept";
    case 0x40802f98u: return "post_desc_feature";
    case 0x40802fbeu: return "feature_bit11_done";
    case 0x40802fdeu: return "feature_bit16_probe";
    case 0x40802ffcu: return "feature_bit17_probe";
    case 0x4080301eu: return "feature_bit15_probe";
    case 0x40803044u: return "feature_done";
    case 0x40846d20u: return "scsi_desc_base";
    case 0x40846d2eu: return "scsi_status_read";
    case 0x40846d36u: return "scsi_status_cmp";
    case 0x40846d3au: return "scsi_status_branch";
    case 0x40846d5au: return "scsi_accept_set_d7";
    case 0x40846b88u: return "diag_fail_set_25";
    case 0x40846b90u: return "diag_fail_bset_25";
    case 0x40846bf0u: return "diag_fail_set_24";
    case 0x40846bf8u: return "diag_fail_bset_24";
    case 0x40846c02u: return "diag_fail_to_monitor";
    case 0x40846e5cu: return "asc_load_a3";
    case 0x40846e64u: return "asc_via1_or";
    case 0x40846e70u: return "asc_chime_delay_arg";
    case 0x40846e76u: return "asc_chime_lea_a0";
    case 0x40846e7cu: return "asc_chime_jmp";
    case 0x40846e80u: return "asc_cleanup";
    case 0x40846ed0u: return "ram_list_next";
    case 0x40846ed2u: return "ram_list_end_cmp";
    case 0x40846ef2u: return "ram_helper_return";
    case 0x40846ef4u: return "ram_helper_tst_d6";
    case 0x40846ef6u: return "ram_helper_fail";
    case 0x40846efau: return "ram_list_advance";
    case 0x40846efeu: return "ram_list_done";
    case 0x4084712eu: return "frame_setup_entry";
    case 0x40847138u: return "frame_suba_alloc";
    case 0x40847142u: return "frame_copy_sig";
    case 0x40847188u: return "frame_dispatch";
    case 0x40847280u: return "ram_helper_entry";
    case 0x4084728eu: return "ram_helper_loop";
    case 0x408472aeu: return "ram_helper_check";
    case 0x4084737cu: return "ram_helper_exit";
    case 0x40849b08u: return "monitor_gate_d7";
    case 0x40849b24u: return "monitor_branch";
    case 0x4084a7d4u: return "rom_monitor_entry";
    case 0x4084a7e8u: return "rom_monitor_after_blink";
    case 0x4084a7f0u: return "rom_monitor_hwdesc";
    case 0x4084a802u: return "rom_monitor_diag_check";
    case 0x4084a812u: return "rom_monitor_scc_check";
    case 0x4084a82eu: return "rom_monitor_body";
    case 0x4084a830u: return "rom_monitor_prepare";
    case 0x4084a838u: return "rom_monitor_after_desc";
    case 0x4084a840u: return "rom_monitor_poll";
    case 0x4084a848u: return "rom_monitor_rx_result";
    case 0x4084aa08u: return "rom_monitor_loop";
    case 0x4084ae28u: return "rom_monitor_desc";
    case 0x4084ae34u: return "rom_monitor_desc_ret";
    case 0x4084ae54u: return "rom_monitor_banner";
    case 0x4084ae94u: return "rom_monitor_banner_ret";
    case 0x4084af9cu: return "rom_scc_rx_poll";
    case 0x4084afa6u: return "rom_scc_rr0_test";
    case 0x4084afcau: return "rom_scc_rx_return";
    case 0x408be180u: return "asc_chime_ext_entry";
    case 0x408be182u: return "asc_chime_ext_probe";
    case 0x4084bb74u: return "memsize_entry";
    case 0x4084bc38u: return "memsize_success";
    default: return nullptr;
    }
}

static void maybe_log_probe(uint32_t pc, bool enabled) {
    if (!enabled) return;
    const char* name = probe_pc_name(pc);
    if (!name) return;

    static uint32_t seen_pc[64] = {};
    static uint8_t seen_count[64] = {};
    for (unsigned i = 0; i < 64; i++) {
        if (seen_pc[i] == pc) {
            if (seen_count[i] >= 8) return;
            seen_count[i]++;
            break;
        }
        if (seen_pc[i] == 0) {
            seen_pc[i] = pc;
            seen_count[i] = 1;
            break;
        }
    }

    auto* r = dut->rootp;
    std::fprintf(stderr,
        "[probe] t=%llu retired=%u pc=0x%08x %-22s "
        "d0=%08x d1=%08x d2=%08x d6=%08x d7=%08x "
        "a0=%08x a1=%08x a2=%08x a3=%08x a4=%08x a5=%08x fp=%08x "
        "sr=%04x ccr=%02x "
        "scc rd=%u wr=%u addr=%x r=%02x w=%02x "
        "via1 rd=%u wr=%u addr=%x r=%02x w=%02x "
        "via2 rd=%u wr=%u addr=%x r=%02x w=%02x "
        "scsi rd=%u wr=%u addr=%03x r=%02x w=%02x c96_istatus=%02x\n",
        (unsigned long long)sim_time,
        (unsigned)DBGT_REF(r, committed),
        (unsigned)pc, name,
        read_arch_reg(0), read_arch_reg(1), read_arch_reg(2),
        read_arch_reg(6), read_arch_reg(7),
        read_arch_reg(8), read_arch_reg(9), read_arch_reg(10),
        read_arch_reg(11), read_arch_reg(12), read_arch_reg(13),
        read_arch_reg(14),
        (unsigned)CPUI_REF(r, u_commit__DOT__arch_sr),
        (unsigned)CPUI_REF(r, u_ccr_rat__DOT__ccr_prf)[
            CPUI_REF(r, u_ccr_rat__DOT__crat_tag)],
        (unsigned)r->fpga_top__DOT__pb_scc_rd,
        (unsigned)r->fpga_top__DOT__pb_scc_wr,
        (unsigned)r->fpga_top__DOT__pb_scc_addr,
        (unsigned)r->fpga_top__DOT__pb_scc_rdata,
        (unsigned)r->fpga_top__DOT__pb_scc_wdata,
        (unsigned)r->fpga_top__DOT__pb_via1_rd,
        (unsigned)r->fpga_top__DOT__pb_via1_wr,
        (unsigned)r->fpga_top__DOT__pb_via1_addr,
        (unsigned)r->fpga_top__DOT__pb_via1_rdata,
        (unsigned)r->fpga_top__DOT__pb_via1_wdata,
        (unsigned)r->fpga_top__DOT__pb_via2_rd,
        (unsigned)r->fpga_top__DOT__pb_via2_wr,
        (unsigned)r->fpga_top__DOT__pb_via2_addr,
        (unsigned)r->fpga_top__DOT__pb_via2_rdata,
        (unsigned)r->fpga_top__DOT__pb_via2_wdata,
        (unsigned)r->fpga_top__DOT__pb_scsi_rd,
        (unsigned)r->fpga_top__DOT__pb_scsi_wr,
        (unsigned)r->fpga_top__DOT__pb_scsi_addr,
        (unsigned)r->fpga_top__DOT__pb_scsi_rdata,
        (unsigned)r->fpga_top__DOT__pb_scsi_wdata,
        (unsigned)r->fpga_top__DOT__u_scsi__DOT__c96_istatus);
}

static void maybe_log_exception(bool enabled) {
    if (!enabled) return;
    auto* r = dut->rootp;
    if (!DBGT_REF(r, boundary_event) ||
        DBGT_REF(r, boundary_kind) != 2)
        return;

    std::fprintf(stderr,
        "[probe-exc] t=%llu retired=%u vec=%u pc=0x%08x fault_pc=0x%08x "
        "fault_addr=0x%08x d0=%08x d6=%08x d7=%08x a0=%08x a2=%08x "
        "a3=%08x a5=%08x fp=%08x\n",
        (unsigned long long)sim_time,
        (unsigned)DBGT_REF(r, committed),
        (unsigned)DBGT_REF(r, boundary_exc_vec),
        (unsigned)DBGT_REF(r, boundary_pc),
        (unsigned)DBGT_REF(r, boundary_fault_pc),
        (unsigned)DBGT_REF(r, boundary_fault_addr),
        read_arch_reg(0), read_arch_reg(6), read_arch_reg(7),
        read_arch_reg(8), read_arch_reg(10), read_arch_reg(11),
        read_arch_reg(13), read_arch_reg(14));
}

static void maybe_log_dual_commit(bool enabled) {
#ifdef CPU_M68K
    // Deep ROB/commit (lb_*_w, rob_*) introspection relocates to the CPU
    // repo's tb under socketization (Phase 6 follow-up).  No-op here.
    (void)enabled;
#else
    if (!enabled) return;
    auto* r = dut->rootp;
    if (!CPUI_REF(r, u_commit__DOT__lb_commit_en))
        return;

    const uint32_t retired = DBGT_REF(r, committed);
    const uint32_t rob_pc = CPUI_REF(r, rob_pc);
    const uint32_t lb_pc = CPUI_REF(r, lb_pc_w);
    const bool near_chime = (rob_pc >= 0x40846e40u && rob_pc <= 0x40846e90u) ||
                            (lb_pc >= 0x40846e40u && lb_pc <= 0x40846e90u) ||
                            (retired >= 184900u && retired <= 185350u);
    if (!near_chime) return;

    std::fprintf(stderr,
        "[dual] t=%llu retired=%u "
        "h pc=%08x hd=%u ad=%u pd=%u po=%u last=%u ccr=%u "
        "b pc=%08x hd=%u ad=%u pd=%u po=%u last=%u ccr=%u "
        "crat a0=%u a3=%u a5=%u vals a0=%08x a3=%08x a5=%08x\n",
        (unsigned long long)sim_time,
        retired,
        rob_pc,
        (unsigned)CPUI_REF(r, rob_hd),
        (unsigned)CPUI_REF(r, rob_ad),
        (unsigned)CPUI_REF(r, rob_pd),
        (unsigned)CPUI_REF(r, rob_po),
        (unsigned)CPUI_REF(r, rob_is_last_uop),
        (unsigned)CPUI_REF(r, rob_ccr_hd),
        lb_pc,
        (unsigned)CPUI_REF(r, lb_has_dst_w),
        (unsigned)CPUI_REF(r, lb_arch_dst_w),
        (unsigned)CPUI_REF(r, lb_phys_dst_w),
        (unsigned)CPUI_REF(r, lb_phys_old_w),
        (unsigned)CPUI_REF(r, lb_is_last_uop_w),
        (unsigned)CPUI_REF(r, lb_ccr_has_dst_w),
        (unsigned)CPUI_REF(r, u_rat__DOT__crat)[8],
        (unsigned)CPUI_REF(r, u_rat__DOT__crat)[11],
        (unsigned)CPUI_REF(r, u_rat__DOT__crat)[13],
        read_arch_reg(8), read_arch_reg(11), read_arch_reg(13));
#endif  // CPU_M68K
}

static void record_pc_history(uint32_t pc, bool dump) {
    static uint32_t hist[256] = {};
    static unsigned pos = 0;
    static bool dumped_late_monitor = false;
    hist[pos++ & 255u] = pc;
    if (pc == 0x4084a7d4u || pc == 0x4084a82eu || pc == 0x4084aa08u) {
        if (!dumped_late_monitor) dump = true;
        dumped_late_monitor = true;
    }
    // TEMP INSTRUMENTATION (session goal: find the caller of the
    // 0x4086AB9C PEA/A-line-storm glue routine — reached via an
    // indirect jump table per ROM static analysis, so static
    // disassembly can't identify the calling instruction).  Dump the
    // 256-deep ring EVERY time we enter this routine so we can see
    // both the calling PC and how A7 varies across invocations.
    static unsigned glue_hit_count = 0;
    if (pc == 0x4086ab9cu) {
        glue_hit_count++;
        if (glue_hit_count <= 5) dump = true;
    }
    if (!dump) return;
    std::fprintf(stderr, "[probe] pc history before monitor entry (pc=0x%08x hit#%u):\n",
                 pc, glue_hit_count);
    for (unsigned i = 0; i < 256; i++) {
        const unsigned idx = (pos + i) & 255u;
        if (hist[idx] != 0)
            std::fprintf(stderr, "[probe]   pc[%02u]=0x%08x\n", i, hist[idx]);
    }
}

static void asc_capture_sample() {
    auto* r = dut->rootp;
    uint8_t sv = (uint8_t)r->fpga_top__DOT__asc_audio_sample_valid_w;
    if (g_audio_dump_fp && sv && !g_prev_audio_sample_valid) {
        if (g_audio_dump_max == 0 || g_audio_dump_count < g_audio_dump_max) {
            int16_t pcm_l = (int16_t)r->fpga_top__DOT__asc_audio_pcm_l_w;
            int16_t pcm_r = (int16_t)r->fpga_top__DOT__asc_audio_pcm_r_w;
            std::fprintf(g_audio_dump_fp,
                         "%llu\t%llu\t%d\t%d\n",
                         (unsigned long long)g_retired_count,
                         (unsigned long long)sim_time,
                         (int)pcm_l, (int)pcm_r);
            g_audio_dump_count++;
        }
    }
    g_prev_audio_sample_valid = sv;
}

static void asc_capture_event() {
    auto* r = dut->rootp;
    uint8_t wr = (uint8_t)r->fpga_top__DOT__pb_asc_wr;
    uint8_t rd = (uint8_t)r->fpga_top__DOT__pb_asc_rd;
    bool wr_edge = (wr && !g_prev_pb_asc_wr);
    bool rd_edge = (rd && !g_prev_pb_asc_rd);
    if (g_asc_event_dump_fp && (wr_edge || rd_edge)) {
        if (g_asc_event_dump_max == 0
            || g_asc_event_dump_count < g_asc_event_dump_max) {
            uint16_t addr = (uint16_t)r->fpga_top__DOT__pb_asc_addr;
            uint8_t wdata = (uint8_t)r->fpga_top__DOT__pb_asc_wdata;
            uint8_t rdata = (uint8_t)r->fpga_top__DOT__pb_asc_rdata;
            std::fprintf(g_asc_event_dump_fp,
                         "%llu\t%llu\t%c\t0x%03x\t0x%02x\n",
                         (unsigned long long)g_retired_count,
                         (unsigned long long)sim_time,
                         wr_edge ? 'W' : 'R',
                         (unsigned)addr,
                         (unsigned)(wr_edge ? wdata : rdata));
            g_asc_event_dump_count++;
        }
    }
    g_prev_pb_asc_wr = wr;
    g_prev_pb_asc_rd = rd;
}

// Cycle-precise IRQ injection — runs every sys_clk via tick(), so it
// is active during the state-replay boot-tick phase too.  Arms when
// sim_time first reaches g_irqc_at, then asserts the autovector IPL
// (held by the RTL until cpu_ipl_ack_w).
static void irq_cycle_tick() {
    if (!g_irqc_set) return;
    const uint64_t retired = g_retired_count;
    if (!g_irqc_armed && sim_time >= g_irqc_at) {
        g_irqc_armed     = true;
        g_irqc_remaining = g_irqc_total;
        g_irqc_next_ret  = retired;
        std::fprintf(stderr,
            "[irqc] t=%llu armed IPL%u count=%llu gap=%llu "
            "(cycle threshold=%llu retired=%llu)\n",
            (unsigned long long)sim_time, (unsigned)g_irqc_lvl,
            (unsigned long long)g_irqc_total,
            (unsigned long long)g_irqc_gap,
            (unsigned long long)g_irqc_at,
            (unsigned long long)retired);
        std::fflush(stderr);
    }
    if (g_irqc_armed && g_irqc_remaining != 0 &&
        retired >= g_irqc_next_ret &&
        DBGT_REF(dut->rootp, irq_inject_pending_lvl) == 0) {
        DBGT_REF(dut->rootp, irq_inject_pending_lvl) = g_irqc_lvl;
        g_irqc_remaining--;
        g_irqc_next_ret = retired + g_irqc_gap;
        std::fprintf(stderr,
            "[irqc] t=%llu injected IPL%u at retired=%llu remaining=%llu\n",
            (unsigned long long)sim_time, (unsigned)g_irqc_lvl,
            (unsigned long long)retired,
            (unsigned long long)g_irqc_remaining);
        std::fflush(stderr);
    }
}

static void tick() {
    dut->sys_clk_p = 0;
    dut->sys_clk_n = 1;
    dut->eval();
#if VM_TRACE_FST
    if (g_wave_open && g_fst) g_fst->dump(sim_time);
#endif
    sim_time++;
    dut->sys_clk_p = 1;
    dut->sys_clk_n = 0;
    dut->eval();
#if VM_TRACE_FST
    if (g_wave_open && g_fst) g_fst->dump(sim_time);
#endif
    sim_time++;
#ifdef CPU_M68K040
    g_retired_count = g_cpu040_retired_count;
#else
    g_retired_count = (uint64_t)DBGT_REF(dut->rootp, committed);
#endif
    // task #272 (SOC-3 behavioral smoke): unconditional, always-on
    // first-fetch probe.  `ifa_ar*` is the SoC xbar's axi_i AR channel
    // (the wide instruction-fetch master port) -- a real top-level
    // fpga_top net for every CPU value, unlike the CPU-internal
    // dbg_committed/boundary_* taps above (only meaningful for
    // CPU=m68k; see the CPUI_REF/DBGT_REF comment near the top of this
    // file, which reads back a constant 0 for CPU=m68k040/stub). Logs
    // the first handful of AR handshakes so a clean build can be
    // independently verified to have actually reset, fetched, and kept
    // fetching, without depending on any of that CPU-internal
    // machinery.
    {
        auto* r = dut->rootp;
        const bool arvalid = r->fpga_top__DOT__ifa_arvalid;
        const bool arready = r->fpga_top__DOT__ifa_arready;
        if (arvalid && !g_ifetch_prev_arvalid) {
            g_ifetch_last_addr = r->fpga_top__DOT__ifa_araddr;
            // Print every rise for the first 40 (enough to see reset ->
            // first-fetch -> the initial fall-through/speculative-fetch
            // burst past an unresolved backward branch settle down), then
            // just every 2000th so a long run doesn't flood stderr but we
            // can still see whether the fetch address trend ever comes
            // back around to the idle loop (0x40000008..0x4000000e) or
            // its ROM-overlay alias (0x00000008..0x0000000e) instead of
            // free-running forever.
            if (g_ifetch_arvalid_count < 40 ||
                (g_ifetch_arvalid_count % 2000) == 0) {
                std::fprintf(stderr,
                    "[ifetch-probe] t=%llu ifa_arvalid RISE #%llu addr=0x%08x arready=%u\n",
                    (unsigned long long)sim_time,
                    (unsigned long long)g_ifetch_arvalid_count,
                    g_ifetch_last_addr, (unsigned)arready);
            }
            g_ifetch_arvalid_count++;
        }
        g_ifetch_prev_arvalid = arvalid;
        if (r->fpga_top__DOT__ifa_rvalid && r->fpga_top__DOT__ifa_rready) {
            if (g_ifetch_rsp_count < 8) {
                std::fprintf(stderr,
                    "[ifetch-rsp] t=%llu rsp#%llu id=%u resp=%u last=%u "
                    "data=%08x_%08x_%08x_%08x_%08x_%08x_%08x_%08x\n",
                    (unsigned long long)sim_time,
                    (unsigned long long)g_ifetch_rsp_count,
                    (unsigned)r->fpga_top__DOT__ifa_rid,
                    (unsigned)r->fpga_top__DOT__ifa_rresp,
                    (unsigned)r->fpga_top__DOT__ifa_rlast,
                    r->fpga_top__DOT__ifa_rdata[7], r->fpga_top__DOT__ifa_rdata[6],
                    r->fpga_top__DOT__ifa_rdata[5], r->fpga_top__DOT__ifa_rdata[4],
                    r->fpga_top__DOT__ifa_rdata[3], r->fpga_top__DOT__ifa_rdata[2],
                    r->fpga_top__DOT__ifa_rdata[1], r->fpga_top__DOT__ifa_rdata[0]);
            }
            g_ifetch_rsp_count++;
        }
#ifdef SIM_L2C_ENABLE
        // ROM-mirror decode-fold check -- keyed on the actual AR
        // handshake (arvalid && arready), not the rise above, so a
        // stalled AR (held valid over several cycles while !arready)
        // gets checked exactly once, the cycle it is actually accepted.
        // Mirrors rtl/soc/fpga_top_ddr.vh's ifa_araddr_folded arithmetic
        // exactly (AXI_ROM_MIRROR_BASE=0x4000_0000,
        // AXI_ROM_MIRROR_SIZE=0x1000_0000, AXI_ROM_IMAGE_SIZE=0x0010_0000,
        // AXI_DDR_ROM_OFFSET=0x4000_0000 -- rtl/soc/axi_defs.vh)
        // hardcoded here the same way kAxiRomBase is above (host-side
        // C++, not RTL, kept in sync by inspection).
        if (arvalid && arready) {
            static constexpr uint32_t kRomMirrorBase = 0x40000000u;
            static constexpr uint32_t kRomMirrorSize = 0x10000000u;
            static constexpr uint32_t kRomImageSize  = 0x00100000u;
            static constexpr uint32_t kDdrRomOffset  = 0x40000000u;
            const uint32_t addr = r->fpga_top__DOT__ifa_araddr;
            const bool in_mirror =
                ((addr & ~(kRomMirrorSize - 1u)) == kRomMirrorBase);
            const uint32_t expected = in_mirror ?
                (((addr - kRomMirrorBase) & (kRomImageSize - 1u)) +
                 kDdrRomOffset) : addr;
            const uint32_t actual = r->fpga_top__DOT__ifa_araddr_folded;
            g_rom_fold_checks++;
            if (in_mirror && addr != expected) g_rom_fold_mirror_hits++;
            if (actual != expected) {
                g_rom_fold_mismatches++;
                std::fprintf(stderr,
                    "[rom-fold-probe] MISMATCH t=%llu raw=0x%08x "
                    "expected_fold=0x%08x actual_fold=0x%08x "
                    "(check #%llu, mismatch #%llu)\n",
                    (unsigned long long)sim_time, addr, expected, actual,
                    (unsigned long long)g_rom_fold_checks,
                    (unsigned long long)g_rom_fold_mismatches);
            }
        }
#endif  // SIM_L2C_ENABLE
    }
    boot_rom_ready_probe();
#ifdef CPU_M68K040
    // task #272, throwaway diagnostic (NOT a permanent tap -- see the
    // CPUI_REF/DBGT_REF comment; the real fix is a dbg_axi-CSR-based
    // observability path, not a raw signal name that will drift with
    // every cpu040 regen). RobPlugin_logic_retiredThisCycle is a real,
    // currently-flat internal signal in cpu040/generated/M68kSocketTop.v
    // (grep confirmed, not part of any documented socket contract) that
    // sums how many uops the ROB retired this cycle. Used once here only
    // to settle whether the ifetch-probe's monotonically-climbing
    // fetch address (never returning to the idle loop) means the
    // backend is genuinely stalled, or just deep wrong-path speculative
    // fetch racing ahead of a not-yet-resolved first-ever branch while
    // retirement proceeds normally underneath (dbg_committed can't
    // answer this -- it's tied to constant 0 at the SoC top level for
    // every CPU value, see fpga_top_debug_ctrl.vh).
    {
        static uint64_t s_last_print_at = 0;
        static uint64_t s_trace_count = 0;
        static uint64_t s_feed_count = 0;
        static uint64_t s_ic_rsp_count = 0;
        static bool s_saw_reset_redirect = false;
        auto* core = dut->rootp;
        if (core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__FetchAlignPlugin_logic_resetRedirect_valid &&
            !s_saw_reset_redirect) {
            std::fprintf(stderr,
                "[reset-vector] t=%llu ssp=0x%08x pc=0x%08x\n",
                (unsigned long long)sim_time,
                core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__ResetVectorPlugin_logic_sspData,
                core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__FetchAlignPlugin_logic_resetRedirect_payload);
            s_saw_reset_redirect = true;
        }
        if (core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__FetchAlignPlugin_logic_feed_valid &&
            core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__FetchAlignPlugin_logic_feed_payload_0_valid &&
            s_feed_count < 16) {
            std::fprintf(stderr,
                "[decode-feed] t=%llu feed#%llu pc=0x%08x words=%04x %04x %04x %04x\n",
                (unsigned long long)sim_time,
                (unsigned long long)s_feed_count,
                core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__FetchAlignPlugin_logic_feed_payload_0_pc,
                core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__FetchAlignPlugin_logic_feed_payload_0_words_0,
                core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__FetchAlignPlugin_logic_feed_payload_0_words_1,
                core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__FetchAlignPlugin_logic_feed_payload_0_words_2,
                core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__FetchAlignPlugin_logic_feed_payload_0_words_3);
            s_feed_count++;
        }
        if (core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IcachePlugin_logic_rspPort_valid &&
            s_ic_rsp_count < 8) {
            std::fprintf(stderr,
                "[icache-rsp] t=%llu rsp#%llu pc=0x%08x data=%016llx fault=%u\n",
                (unsigned long long)sim_time,
                (unsigned long long)s_ic_rsp_count,
                core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IcachePlugin_logic_rspPort_payload_pc,
                (unsigned long long)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IcachePlugin_logic_rspPort_payload_data,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IcachePlugin_logic_rspPort_payload_fault);
            s_ic_rsp_count++;
        }
        const uint32_t inc =
            dut->rootp->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_retiredThisCycle;
        g_cpu040_retired_count += inc;
        if (sim_time - s_last_print_at >= 200000 || (inc && g_cpu040_retired_count <= 8)) {
            std::fprintf(stderr,
                "[retire-probe] t=%llu retired_sum=%llu (this-cycle inc=%u)\n",
                (unsigned long long)sim_time,
                (unsigned long long)g_cpu040_retired_count, inc);
            s_last_print_at = sim_time;
        }
        // Part 73 IPC investigation: unconditional per-cycle store-queue/
        // D-cache probe, gated by +sq_trace_cycles=<n> (prints for the
        // first n cycles of this process's run, then goes silent).
        if (g_sq_trace_emitted < g_sq_trace_cycles) {
            std::fprintf(stderr,
                "[sq-trace] t=%llu retThis=%u mmuEn=%u fastSt=%u sqAlloc=%u sqFull=%u sqEmpty=%u "
                "sqDrainV=%u sqDrainR=%u sqCompl=%u dcSt1=%u dcSt2=%u dcSt3=%u "
                "dcBusy=%u stPortV=%u stPortR=%u stPortFire=%u\n",
                (unsigned long long)sim_time,
                inc,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__MmuControlPlugin_logic_mmuEnable,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__LsEuPlugin_logic_fastStore,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__LsEuPlugin_logic_sq_io_alloc_valid,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__LsEuPlugin_logic_sq_io_full,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__LsEuPlugin_logic_sq_io_empty,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__LsEuPlugin_logic_sq_io_drain_valid,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__LsEuPlugin_logic_sq_io_drain_ready,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__LsEuPlugin_logic_sq_io_sqCompletion_valid,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__DcachePlugin_logic_stS1Valid,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__DcachePlugin_logic_stS2Valid,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__DcachePlugin_logic_stS3Valid,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__DcachePlugin_logic_busy,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__DcachePlugin_logic_storePort_valid,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__DcachePlugin_logic_storePort_ready,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__DcachePlugin_logic_storePort_fire);
            g_sq_trace_emitted++;
        }
        // Part 75: broader per-cycle pipeline-stall probe (ROB/IQ/SQ
        // occupancy + I$/D$ outstanding state), gated by
        // +pipe_trace_cycles=<n>/+pipe_trace_path=<path>.
        if (g_pipe_trace_fp && g_pipe_trace_emitted < g_pipe_trace_cycles) {
            std::fprintf(g_pipe_trace_fp,
                "t=%llu retThis=%u "
                "robHead=%u robTail=%u robCnt=%u robFlush=%u robFlushPc=0x%08x "
                "brVal=%u brMispred=%u "
                "iqCnt=%u iqInt=%016llx iqNzvc=%04x iqX=%04x iqLs=%014llx iqLsNzvc=%u iqCplx=%014llx iqCplxNzvc=%u iqCplxFp=%u iqCplxFpcc=%u "
                "mmuEn=%u fastSt=%u "
                "sqHead=%u sqTail=%u sqAlloc=%u sqFull=%u sqEmpty=%u sqDrainV=%u sqCompl=%u "
                "icMshr=%u%u%u%u%u icS1Hit=%u icMissPc=0x%08x "
                "dcStOut=%u dcSerSt=%u dcStBar=%u dcLdBar=%u dcLdS1V=%u dcLdS1Hit=%u dcStS1V=%u dcMaintBusy=%u "
                "flIntCnt=%u flNzvcCnt=%u flXCnt=%u\n",
                (unsigned long long)sim_time,
                inc,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_head,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_tail,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_count,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_doFlushReg,
                core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_flushPcReg,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_branchCompletion_valid,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_branchCompletion_payload_mispredict,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IssueQueuePlugin_logic_count,
                (unsigned long long)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IssueQueuePlugin_logic_sbInt_busy,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IssueQueuePlugin_logic_sbNzvc_busy,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IssueQueuePlugin_logic_sbX_busy,
                (unsigned long long)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IssueQueuePlugin_logic_lsBusy,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IssueQueuePlugin_logic_lsNzvcBusy,
                (unsigned long long)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IssueQueuePlugin_logic_cplxBusy,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IssueQueuePlugin_logic_cplxNzvcBusy,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IssueQueuePlugin_logic_cplxFpBusy,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IssueQueuePlugin_logic_cplxFpccBusy,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__MmuControlPlugin_logic_mmuEnable,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__LsEuPlugin_logic_fastStore,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__LsEuPlugin_logic_sq__DOT__head,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__LsEuPlugin_logic_sq__DOT__tail,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__LsEuPlugin_logic_sq_io_alloc_valid,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__LsEuPlugin_logic_sq_io_full,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__LsEuPlugin_logic_sq_io_empty,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__LsEuPlugin_logic_sq_io_drain_valid,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__LsEuPlugin_logic_sq_io_sqCompletion_valid,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IcachePlugin_logic_mshrValid_0,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IcachePlugin_logic_mshrValid_1,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IcachePlugin_logic_mshrValid_2,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IcachePlugin_logic_mshrValid_3,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IcachePlugin_logic_mshrValid_4,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IcachePlugin_logic_s1Hit,
                core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__IcachePlugin_logic_missPC,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__DcachePlugin_logic_storeOutstanding,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__DcachePlugin_logic_serialStoreInFlight,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__DcachePlugin_logic_storeMissBarrier,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__DcachePlugin_logic_loadMissStoreBarrier,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__DcachePlugin_logic_ldS1Valid,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__DcachePlugin_logic_ldS1Hit,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__DcachePlugin_logic_stS1Valid,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__DcachePlugin_logic_maintBusyReg,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RenameStage_logic_intFree__DOT__count,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RenameStage_logic_nzvcFree__DOT__count,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RenameStage_logic_xFree__DOT__count);
            g_pipe_trace_emitted++;
        }
        // 2026-09-02 follow-up session: IRQ-recognition probe (see the
        // g_irq_trace_* declarations above for rationale). All fields are
        // pre-existing simPublic() RobPlugin/InterruptControlPlugin/via1
        // signals -- no new RTL taps.
        if (g_irq_trace_fp && g_irq_trace_emitted < g_irq_trace_cycles) {
            std::fprintf(g_irq_trace_fp,
                "t=%llu retThis=%u robCnt=%u robHead=%u p0First=%u p0Pc=0x%08x stopped=%u "
                "excIdle=%u flushing=%u doFlushReg=%u branchRedirect=%u "
                "iplIn=%u iplActive=%u normalIrqGate=%u interruptPending=%u "
                "via1IfrAny=%u via1Ifr=0x%02x via1IfrRd=0x%02x via1Ier=0x%02x\n",
                (unsigned long long)sim_time,
                inc,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_count,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_head,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_p0_first,
                core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_p0_pc,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_stopped,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_excIdle,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_flushing,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_doFlushReg,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_branchRedirect,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_iplIn,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_iplActive,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_normalIrqGate,
                (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_interruptPending,
                (unsigned)core->fpga_top__DOT__u_via1__DOT__ifr_any,
                (unsigned)core->fpga_top__DOT__u_via1__DOT__ifr,
                (unsigned)core->fpga_top__DOT__u_via1__DOT__ifr_rd,
                (unsigned)core->fpga_top__DOT__u_via1__DOT__ier);
            g_irq_trace_emitted++;
        }
        if (dut->rootp->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_traceVec_0_fire) {
            g_cpu040_last_pc = core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_traceVec_0_pc;
            if (s_trace_count < 16) {
                std::fprintf(stderr, "[retire-pc] t=%llu retire#%llu pc=0x%08x op=%04x exc=%u vec=%u\n",
                    (unsigned long long)sim_time,
                    (unsigned long long)s_trace_count,
                    core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_traceVec_0_pc,
                    core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_traceVec_0_opword,
                    (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_traceVec_0_excTaken,
                    (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_traceVec_0_excVector);
                s_trace_count++;
            }
            if (g_pc_dump_fp && g_pc_dump_count < g_pc_dump_max) {
                std::fprintf(g_pc_dump_fp, "%llu %llu 0x%08x\n",
                    (unsigned long long)g_cpu040_retired_count,
                    (unsigned long long)sim_time,
                    (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_traceVec_0_pc);
                g_pc_dump_count++;
            }
        }
        if (dut->rootp->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_traceVec_1_fire) {
            g_cpu040_last_pc = core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_traceVec_1_pc;
            if (g_pc_dump_fp && g_pc_dump_count < g_pc_dump_max) {
                std::fprintf(g_pc_dump_fp, "%llu %llu 0x%08x\n",
                    (unsigned long long)g_cpu040_retired_count,
                    (unsigned long long)sim_time,
                    (unsigned)core->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_traceVec_1_pc);
                g_pc_dump_count++;
            }
        }
    }
#endif
    // SD card: sample the SPI pins for this posedge and present the MISO
    // level the card would drive for the next one.  sd_spi.v samples its
    // 2-flop-synchronised MISO at the end of the SCK HI phase, so a value
    // launched here (one core_clk after the falling edge) is stable well
    // before the sampling edge even at HS_HALF=2.
    if (g_sd_enabled)
        dut->sd_miso = g_sd.tick(dut->sd_clk, dut->sd_mosi, dut->sd_cs_n);
    sd_req_tick();
    irq_cycle_tick();
    if (g_vbl_log_count < g_vbl_log_max) {
        auto* r = dut->rootp;
        const uint8_t pclk_pulse = r->fpga_top__DOT__video_vbl_pulse_pclk;
        const uint8_t pb_pulse = r->fpga_top__DOT__dafb_vbl_pulse_pb;
        const uint8_t level = r->fpga_top__DOT__dafb_vbl_level;
        const uint8_t ca1 = r->fpga_top__DOT__via1_ca1_in;
        const uint8_t irq = r->fpga_top__DOT__via1_irq_pb;
        if (pclk_pulse || pb_pulse || level != g_prev_dafb_vbl_level) {
            std::fprintf(stderr,
                "[vbl] t=%llu retired=%llu pclk_pulse=%u pb_pulse=%u "
                "level=%u ca1=%u via1_irq=%u vcount=%u vs=%u\n",
                (unsigned long long)sim_time,
                (unsigned long long)g_retired_count,
                (unsigned)pclk_pulse, (unsigned)pb_pulse,
                (unsigned)level, (unsigned)ca1, (unsigned)irq,
                (unsigned)r->fpga_top__DOT__video_debug_vcount,
                (unsigned)r->fpga_top__DOT__video_debug_vs);
            g_vbl_log_count++;
        }
        g_prev_dafb_vbl_level = level;
        g_prev_via1_ca1_in = ca1;
        g_prev_via1_irq_pb = irq;
    }
    asc_capture_sample();
    asc_capture_event();
#if VM_TRACE_FST
    // Open the FST when retired enters the window; close on exit.
    if (!g_wave_open && g_retired_count >= g_wave_start_retired &&
        g_retired_count < g_wave_end_retired && !g_wave_path.empty()) {
        Verilated::traceEverOn(true);
        g_fst = new VerilatedFstC;
        dut->trace(g_fst, 99);
        g_fst->open(g_wave_path.c_str());
        g_wave_open = true;
        std::fprintf(stderr,
            "[wave] OPEN retired=%llu path=%s end_at=%llu\n",
            (unsigned long long)g_retired_count, g_wave_path.c_str(),
            (unsigned long long)g_wave_end_retired);
    }
    if (g_wave_open && g_retired_count >= g_wave_end_retired) {
        g_fst->close();
        delete g_fst;
        g_fst = nullptr;
        g_wave_open = false;
        std::fprintf(stderr, "[wave] CLOSE retired=%llu\n",
            (unsigned long long)g_retired_count);
    }
#endif
}

static bool load_file(const std::string& path, std::vector<uint8_t>& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::fprintf(stderr, "cannot open ROM: %s\n", path.c_str());
        return false;
    }
    out.assign(std::istreambuf_iterator<char>(in),
               std::istreambuf_iterator<char>());
    return true;
}

#ifdef SIM_MIG_BRIDGE
// SIM_MIG_BRIDGE: DDR backing lives in u_ddr.u_mig_sim.mem_lane[byte][beat],
// where beat is 32-byte-indexed (phys>>5) and `byte` is the AXI byte position
// within the 32-byte beat.  The big-endian m68k byte-invariant swap maps
// m68k phys → AXI byte as: word_in_beat = (phys>>2)&7; byte_in_word = phys&3;
// axi_byte = (word_in_beat<<2) | (3 - byte_in_word).
static inline uint32_t mig_axi_byte_index(uint32_t phys) {
    const uint32_t byte_in_beat = phys & 0x1Fu;
    const uint32_t word_in_beat = byte_in_beat >> 2;
    const uint32_t byte_in_word = byte_in_beat & 3u;
    return (word_in_beat << 2) | (3u - byte_in_word);
}
static void ddr_write_phys_byte(uint32_t phys, uint8_t v) {
    const uint32_t beat = phys >> 5;
    const uint32_t lane = mig_axi_byte_index(phys);
    dut->rootp->fpga_top__DOT__u_ddr__DOT__u_mig_sim__DOT__mem_lane[lane][beat] = v;
}
static uint8_t ddr_read_phys_byte(uint32_t phys) {
    const uint32_t beat = phys >> 5;
    const uint32_t lane = mig_axi_byte_index(phys);
    return dut->rootp->fpga_top__DOT__u_ddr__DOT__u_mig_sim__DOT__mem_lane[lane][beat];
}
#else  // Regular SIM_MODEL: 16 byte-lanes in u_ddr.mem_bN, 16-byte beats.
static void ddr_write_phys_byte(uint32_t phys, uint8_t v) {
    const uint32_t beat = phys >> 4;
    const uint32_t byte_in_beat = phys & 0xfu;
    const uint32_t lane = (byte_in_beat & ~3u) | (3u - (byte_in_beat & 3u));
    switch (lane) {
    case 0:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b0 [beat] = v; break;
    case 1:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b1 [beat] = v; break;
    case 2:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b2 [beat] = v; break;
    case 3:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b3 [beat] = v; break;
    case 4:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b4 [beat] = v; break;
    case 5:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b5 [beat] = v; break;
    case 6:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b6 [beat] = v; break;
    case 7:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b7 [beat] = v; break;
    case 8:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b8 [beat] = v; break;
    case 9:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b9 [beat] = v; break;
    case 10: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b10[beat] = v; break;
    case 11: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b11[beat] = v; break;
    case 12: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b12[beat] = v; break;
    case 13: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b13[beat] = v; break;
    case 14: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b14[beat] = v; break;
    default: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b15[beat] = v; break;
    }
}

static uint8_t ddr_read_phys_byte(uint32_t phys) {
    const uint32_t beat = phys >> 4;
    const uint32_t byte_in_beat = phys & 0xfu;
    const uint32_t lane = (byte_in_beat & ~3u) | (3u - (byte_in_beat & 3u));
    switch (lane) {
    case 0:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b0 [beat];
    case 1:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b1 [beat];
    case 2:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b2 [beat];
    case 3:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b3 [beat];
    case 4:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b4 [beat];
    case 5:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b5 [beat];
    case 6:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b6 [beat];
    case 7:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b7 [beat];
    case 8:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b8 [beat];
    case 9:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b9 [beat];
    case 10: return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b10[beat];
    case 11: return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b11[beat];
    case 12: return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b12[beat];
    case 13: return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b13[beat];
    case 14: return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b14[beat];
    default: return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b15[beat];
    }
}
#endif  // SIM_MIG_BRIDGE

static void ddr_write_phys32(uint32_t phys, uint32_t v) {
    ddr_write_phys_byte(phys + 0, static_cast<uint8_t>(v >> 24));
    ddr_write_phys_byte(phys + 1, static_cast<uint8_t>(v >> 16));
    ddr_write_phys_byte(phys + 2, static_cast<uint8_t>(v >> 8));
    ddr_write_phys_byte(phys + 3, static_cast<uint8_t>(v));
}

static uint32_t ddr_read_phys32(uint32_t phys) {
    return (static_cast<uint32_t>(ddr_read_phys_byte(phys + 0)) << 24) |
           (static_cast<uint32_t>(ddr_read_phys_byte(phys + 1)) << 16) |
           (static_cast<uint32_t>(ddr_read_phys_byte(phys + 2)) << 8) |
           static_cast<uint32_t>(ddr_read_phys_byte(phys + 3));
}

static void boot_rom_ready_probe() {
    // AXI_ROM_BASE (rtl/soc/axi_defs.vh) -- native ROM window's REAL
    // system address, for the log line's label only.  Hardcoded here
    // rather than pulled from a Verilog `define since this is host-side
    // C++, not RTL; kept in sync by inspection (the task background/this
    // probe's own comment both name the same constant, and axi_defs.vh
    // documents it as architecturally fixed).
    static constexpr uint32_t kAxiRomBase = 0x40000000u;
    // BUG FOUND while adding the 0x4080_0000 ROM-mirror-decode-fold work
    // (rtl/soc/fpga_top_ddr.vh's ifa_araddr_folded): this probe used to
    // pass kAxiRomBase (the REAL 0x4000_0000 system address) straight
    // into ddr_read_phys32(), but ddr_read_phys_byte()/ddr_write_rom_byte()
    // (above) do NOT address the model in real system-address space --
    // they use a COMPACT model layout where ROM content lands right
    // after the RAM_BYTES-sized RAM region, at model-offset
    // `ROM_BEAT_BASE << 4` (== RAM_BYTES, since ROM_BEAT_BASE =
    // RAM_BYTES/16u), not at 0x4000_0000. Passing kAxiRomBase computed a
    // beat index (kAxiRomBase>>4) ~16x past where ddr_write_rom_byte()
    // actually wrote the preloaded ROM -- and, worse, past the sized
    // model array entirely, so any run that ever got boot_rom_ready to
    // rise (real SD-streamed realboot, or here where FPGA_ROM_SIM's
    // backdoor preload makes it rise almost immediately) SEGFAULTED the
    // instant this probe fired, out-of-bounds mem_bN[]/mem_lane[]
    // access. This is very likely the same root cause behind this
    // session's prior tb-fpga-top-rom-realboot segfault "right after the
    // SD stream finished" (exactly when boot_rom_ready first rises for
    // real). Fixed by reading back at the model's own ROM offset
    // (RAM_BYTES) instead of the real address.
    auto* r = dut->rootp;
    const bool ready = r->fpga_top__DOT__boot_rom_ready;
    if (ready && !g_prev_boot_rom_ready) {
        std::fprintf(stderr,
            "[realboot] t=%llu boot_rom_ready RISE -- boot_fsm's real "
            "SD-streaming+mirror pass reports done. DDR content: "
            "mirror[0x0..0x7]=%08x %08x  native[0x%08x..+7]=%08x %08x "
            "(real ROM first 8 bytes are 420dbff3 0000002a)\n",
            (unsigned long long)sim_time,
            ddr_read_phys32(0x0u), ddr_read_phys32(0x4u),
            (unsigned)kAxiRomBase,
            ddr_read_phys32(RAM_BYTES), ddr_read_phys32(RAM_BYTES + 0x4u));
    }
    g_prev_boot_rom_ready = ready;
}

static void invalidate_dcache_for_host_ram_patch() {
#ifndef CPU_M68K
    auto* r = dut->rootp;
    for (unsigned set = 0; set < 32; set++) {
        for (unsigned way = 0; way < 4; way++) {
            CPUI_REF(r, u_dcache__DOT__valid)[set][way] = 0;
        }
    }
#endif  // !CPU_M68K — deep D-cache backdoor relocates to CPU repo tb
}

#ifdef SIM_MIG_BRIDGE
// SIM_MIG_BRIDGE: ROM image lands at the same DDR addresses (ROM_BEAT_BASE
// is the 16-byte-beat-indexed offset; convert to a byte phys address and
// reuse ddr_write_phys_byte which already knows the 32-byte beat layout).
static void ddr_write_rom_byte(uint32_t byte_off, uint8_t v) {
    const uint32_t phys = (ROM_BEAT_BASE << 4) + byte_off;
    ddr_write_phys_byte(phys, v);
}
#else
static void ddr_write_rom_byte(uint32_t byte_off, uint8_t v) {
    const uint32_t beat = ROM_BEAT_BASE + (byte_off >> 4);
    const uint32_t byte_in_beat = byte_off & 0xfu;
    const uint32_t lane = (byte_in_beat & ~3u) | (3u - (byte_in_beat & 3u));
    switch (lane) {
    case 0:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b0 [beat] = v; break;
    case 1:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b1 [beat] = v; break;
    case 2:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b2 [beat] = v; break;
    case 3:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b3 [beat] = v; break;
    case 4:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b4 [beat] = v; break;
    case 5:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b5 [beat] = v; break;
    case 6:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b6 [beat] = v; break;
    case 7:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b7 [beat] = v; break;
    case 8:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b8 [beat] = v; break;
    case 9:  dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b9 [beat] = v; break;
    case 10: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b10[beat] = v; break;
    case 11: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b11[beat] = v; break;
    case 12: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b12[beat] = v; break;
    case 13: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b13[beat] = v; break;
    case 14: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b14[beat] = v; break;
    default: dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b15[beat] = v; break;
    }
}
#endif  // SIM_MIG_BRIDGE

// FIXME(loose_ends.md 2026-04-27 monitor_gate_d7): host-side fast-fill
// writes the MAME-observed RAM-test end state directly into the DDR model
// through the verilator backdoor and brute-invalidates the entire L1D.
// This bypasses LSU / MMU / AXI — exactly the paths a real cache-coherence,
// MMU-walker, or store-buffer-ordering bug would live on.  The lockstep
// audit (2026-04-27) flags this as critical weakness #2; the D7[26] /
// monitor_gate_d7 task entry in docs/loose_ends.md names it as the leading
// hypothesis for why the smoke trips that fail-PC.  The real fix is to
// either (a) make the host-fill genuinely transparent to LSU/MMU/D-cache,
// or (b) replace it with an in-ROM `move.l #...,(a0)+` patch so the same
// path that the verify pass uses is the one that filled.  Until that
// lands, this helper exists as a frontier-iteration aid, not a
// hardware-valid path.
static void maybe_apply_ramtest_mame_state_fastfill(uint32_t retired_pc,
                                                    bool enabled) {
    if (!enabled) return;

    static uint64_t hits = 0;
    static uint64_t skips = 0;
    static uint64_t bytes = 0;
    static uint64_t sentinels = 0;
    static bool rtc_pram_state_prefilled = false;
    uint32_t start = 0;
    uint32_t end = 0;
    uint32_t list_sentinel = 0;
    if (retired_pc == 0x4084721cu && !rtc_pram_state_prefilled) {
        // The rtc-pram-mame-state patch installs this exact MAME-observed
        // state before branching to the RAM helper.  Apply the skipped RAM
        // side effects here, before younger speculative loads can consume the
        // descriptor-list sentinel.
        start = 0x00000000u;
        end = 0x003fffd4u;
        list_sentinel = 0x003fffdcu;
        rtc_pram_state_prefilled = true;
    } else if (retired_pc == 0x40847280u) {
        start = read_arch_reg(8);       // A0, helper start pointer.
        end = read_arch_reg(9);         // A1, helper top pointer.
        list_sentinel = read_arch_reg(12); // A4 at helper entry.
    } else {
        return;
    }

    if (hits != 0 && start == end) {
        skips++;
        if (skips <= 8 || (skips & (skips - 1u)) == 0) {
            std::fprintf(stderr,
                "[fpga-rom] ramtest-mame-state skipped empty range "
                "0x%08x..0x%08x (skip=%llu)\n",
                start, end, (unsigned long long)skips);
        }
        return;
    }
    if (start >= end || end > RAM_BYTES) {
        std::fprintf(stderr,
            "[fpga-rom] ramtest-mame-state bad fill range: "
            "start=0x%08x end=0x%08x ram=0x%08x\n",
            start, end, RAM_BYTES);
        return;
    }

    static const uint8_t pattern[3] = {0x6du, 0xb6u, 0xdbu};
    const uint32_t len = end - start;
    for (uint32_t i = 0; i < len; i++)
        ddr_write_phys_byte(start + i, pattern[i % 3u]);

    if (list_sentinel <= RAM_BYTES - 4u) {
        ddr_write_phys32(list_sentinel, 0xffffffffu);
        const uint32_t readback = ddr_read_phys32(list_sentinel);
        sentinels++;
        std::fprintf(stderr,
            "[fpga-rom] ramtest-mame-state wrote MAME list sentinel "
            "0x%08x <- 0xffffffff readback=0x%08x (sentinel=%llu)\n",
            list_sentinel, readback, (unsigned long long)sentinels);
    }

    invalidate_dcache_for_host_ram_patch();
    hits++;
    bytes += len;
    std::fprintf(stderr,
        "[fpga-rom] ramtest-mame-state host-filled RAM "
        "0x%08x..0x%08x (%u bytes, hit=%llu, total_bytes=%llu)\n",
        start, end, len, (unsigned long long)hits,
        (unsigned long long)bytes);
}

static bool apply_rom_patch(std::vector<uint8_t>& rom, const RomPatch& patch) {
    if (patch.off >= rom.size()) {
        std::fprintf(stderr,
            "[fpga-rom] patch %s offset 0x%05x outside ROM size %zu\n",
            patch.name, patch.off, rom.size());
        return false;
    }
    const uint8_t old = rom[patch.off];
    rom[patch.off] = patch.value;
    std::fprintf(stderr,
        "[fpga-rom] patch %s off=0x%05x old=0x%02x new=0x%02x\n",
        patch.name, patch.off, old, patch.value);
    return true;
}

static bool apply_rom_patches(std::vector<uint8_t>& rom,
                              const RomPatch* patches,
                              size_t count) {
    for (size_t i = 0; i < count; i++) {
        if (!apply_rom_patch(rom, patches[i])) return false;
    }
    return true;
}

static bool apply_one_rom_patch_set(std::vector<uint8_t>& rom,
                                    const std::string& patch_set) {
    if (patch_set.empty()) return true;
    // chime-skip (and every other set) flows through the shared
    // add_rom_patch_set definition in rom_patch_sets.h — that is the
    // single source of truth, including chime-skip's word-sum checksum
    // compensation.  Do NOT re-add a special case here.
    std::vector<RomPatchByte> patches;
    if (!add_rom_patch_set(patches, patch_set, "[fpga-rom]"))
        return false;
    for (const RomPatchByte& patch : patches) {
        const RomPatch one = {patch.off, patch.value, patch.set};
        if (!apply_rom_patch(rom, one)) return false;
    }
    return true;
}

static bool apply_rom_patch_set(std::vector<uint8_t>& rom,
                                const std::string& patch_sets) {
    size_t start = 0;
    while (start <= patch_sets.size()) {
        const size_t comma = patch_sets.find(',', start);
        const std::string one = patch_sets.substr(
            start, comma == std::string::npos ? std::string::npos : comma - start);
        if (!apply_one_rom_patch_set(rom, one)) return false;
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return true;
}

static void patch_rom_jmp_to(std::vector<uint8_t>& rom, uint32_t target_pc,
                             uint32_t target_ssp);

static bool preload_rom(const std::string& path, const std::string& patch_set,
                        bool patch_reset_vec, uint32_t target_pc,
                        uint32_t target_ssp) {
    std::vector<uint8_t> rom;
    if (!load_file(path, rom)) return false;
    if (!apply_rom_patch_set(rom, patch_set)) return false;
    if (patch_reset_vec) patch_rom_jmp_to(rom, target_pc, target_ssp);
    if (rom.size() > ROM_BYTES) {
        std::fprintf(stderr, "ROM too large: %zu > %u\n", rom.size(), ROM_BYTES);
        return false;
    }
#ifdef SIM_MIG_BRIDGE
    // SIM_MIG_BRIDGE backend is sized via BEATS_LOG2=21 (64 MiB).
    // BEATS_LOG2 isn't directly exposed; just trust the build-time sizing.
#else
    const uint32_t required_beats = ROM_BEAT_BASE + (ROM_BYTES / 16u);
    const uint32_t ddr_beats = dut->rootp->fpga_top__DOT__u_ddr__DOT__BEATS;
    if (ddr_beats < required_beats) {
        std::fprintf(stderr,
            "[fpga-rom] DDR model too small for ROM preload: "
            "BEATS=%u required=%u; rebuild fpga_top harness with FPGA_ROM_SIM\n",
            ddr_beats, required_beats);
        return false;
    }
#endif
    for (uint32_t i = 0; i < ROM_BYTES; i++) {
        const uint8_t byte = i < rom.size() ? rom[i] : 0;
        ddr_write_rom_byte(i, byte);
        // FPGA_ROM_SIM bypasses boot_fsm, so reproduce boot_fsm's completed
        // DDR state, not just its native-ROM copy.  The real boot path writes
        // every byte to both AXI_DDR_ROM_OFFSET and low RAM before releasing
        // cpu_rst; cpu040 consumes the architectural reset vectors at +0/+4
        // from that low mirror.  Leaving it zero made the full-SoC harness
        // retire a stream of ORI.B #0 instructions from address 0x2a instead
        // of executing the ROM, while still claiming boot_rom_ready.
        ddr_write_phys_byte(i, byte);
    }
    std::fprintf(stderr,
                 "[fpga-rom] preloaded %zu bytes into DDR ROM window and "
                 "low-RAM boot mirror\n",
                 rom.size());
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// MAME state-replay snapshot loader.  Format produced by
// tools/mame_state_dump.py:
//   line 0: "# state_replay_v1\n"
//   lines 1..N: "KEY=HEX_VALUE\n" pairs (PC, SR, VBR, USP, SP, D0..D7,
//                                        A0..A7)
//   line N+1: literal "DRAM\n"
//   bytes: 4 MiB of DRAM contents starting at phys 0x00000000
// ─────────────────────────────────────────────────────────────────────
struct StateReplay {
    bool     present = false;
    uint32_t pc = 0;
    uint32_t sr = 0x2700;
    uint32_t vbr = 0;
    uint32_t usp = 0;
    uint32_t sp  = 0;        // active SP at snapshot time
    uint32_t d[8] = {0};
    uint32_t a[8] = {0};
    // MMU CSRs (populated by mame_state_dump.py STATE_MMU line).
    uint32_t tc = 0;
    uint32_t srp = 0;
    uint32_t urp = 0;
    uint32_t dtt0 = 0;
    uint32_t dtt1 = 0;
    uint32_t itt0 = 0;
    uint32_t itt1 = 0;
    uint32_t cacr = 0;
    std::vector<uint8_t> dram;
};

static bool load_state_replay(const std::string& path, StateReplay& out) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) {
        std::fprintf(stderr, "[state-replay] cannot open %s: %s\n",
                     path.c_str(), std::strerror(errno));
        return false;
    }
    auto read_line = [&](std::string& s) -> bool {
        s.clear();
        int c;
        while ((c = std::fgetc(f)) != EOF) {
            if (c == '\n') return true;
            s.push_back(static_cast<char>(c));
        }
        return !s.empty();
    };

    std::string line;
    if (!read_line(line) || line != "# state_replay_v1") {
        std::fprintf(stderr, "[state-replay] bad magic in %s: '%s'\n",
                     path.c_str(), line.c_str());
        std::fclose(f);
        return false;
    }
    auto set_kv = [&](const std::string& key, uint32_t val) -> bool {
        if (key == "PC") out.pc = val;
        else if (key == "SR") out.sr = val;
        else if (key == "VBR") out.vbr = val;
        else if (key == "USP") out.usp = val;
        else if (key == "SP")  out.sp  = val;
        else if (key == "TC")  out.tc = val;
        else if (key == "SRP") out.srp = val;
        else if (key == "URP") out.urp = val;
        else if (key == "DTT0") out.dtt0 = val;
        else if (key == "DTT1") out.dtt1 = val;
        else if (key == "ITT0") out.itt0 = val;
        else if (key == "ITT1") out.itt1 = val;
        else if (key == "CACR") out.cacr = val;
        else if (key.size() == 2 && key[0] == 'D' &&
                 key[1] >= '0' && key[1] <= '7')
            out.d[key[1] - '0'] = val;
        else if (key.size() == 2 && key[0] == 'A' &&
                 key[1] >= '0' && key[1] <= '7')
            out.a[key[1] - '0'] = val;
        else {
            std::fprintf(stderr, "[state-replay] unknown key '%s'\n",
                         key.c_str());
            return false;
        }
        return true;
    };
    while (read_line(line)) {
        if (line == "DRAM") break;
        const size_t eq = line.find('=');
        if (eq == std::string::npos) {
            std::fprintf(stderr, "[state-replay] bad line: '%s'\n",
                         line.c_str());
            std::fclose(f);
            return false;
        }
        const std::string key = line.substr(0, eq);
        const std::string vs  = line.substr(eq + 1);
        const uint32_t val = static_cast<uint32_t>(
            std::strtoul(vs.c_str(), nullptr, 16));
        if (!set_kv(key, val)) {
            std::fclose(f);
            return false;
        }
    }
    // DRAM window size is dynamic: read to EOF rather than a hardcoded
    // 0x400000 (4 MiB).  A snapshot taken after the MMU is enabled can have
    // SRP/URP root a page table well above 4 MiB (e.g. near the top of an
    // 8 MiB -ramsize config); a fixed 4 MiB window silently truncates the
    // page tables the walker needs, producing a spurious ITLB/DTLB miss on
    // the very first fetch at the snapshot PC.  tools/mame_state_dump.py /
    // scratch capture scripts must dump a window at least as large as the
    // MAME -ramsize used, and this loader now accepts whatever size was
    // actually written (minimum 4 MiB, kept for old-format compatibility;
    // no hard maximum -- bounded only by the DDR backing model's own size).
    const long here = std::ftell(f);
    if (here < 0 || std::fseek(f, 0, SEEK_END) != 0) {
        std::fprintf(stderr, "[state-replay] cannot seek %s to measure DRAM size\n",
                     path.c_str());
        std::fclose(f);
        return false;
    }
    const long end = std::ftell(f);
    if (end < 0 || end < here || std::fseek(f, here, SEEK_SET) != 0) {
        std::fprintf(stderr, "[state-replay] bad DRAM size in %s\n", path.c_str());
        std::fclose(f);
        return false;
    }
    size_t dram_size = static_cast<size_t>(end - here);
    if (dram_size < 0x400000) dram_size = 0x400000;  // legacy minimum
    out.dram.assign(dram_size, 0);
    const size_t n = std::fread(out.dram.data(), 1, out.dram.size(), f);
    std::fclose(f);
    if (n != out.dram.size()) {
        std::fprintf(stderr,
            "[state-replay] short DRAM read: got %zu, want %zu\n",
            n, out.dram.size());
        return false;
    }
    out.present = true;
    std::fprintf(stderr,
        "[state-replay] loaded snapshot: PC=0x%08x SR=0x%04x VBR=0x%08x "
        "USP=0x%08x SP=0x%08x DRAM=%zu bytes\n",
        out.pc, out.sr, out.vbr, out.usp, out.sp, out.dram.size());
    for (int i = 0; i < 8; i++) {
        std::fprintf(stderr,
            "[state-replay]   D%d=0x%08x  A%d=0x%08x\n",
            i, out.d[i], i, out.a[i]);
    }
    std::fprintf(stderr,
        "[state-replay]   TC=0x%08x SRP=0x%08x URP=0x%08x CACR=0x%08x\n",
        out.tc, out.srp, out.urp, out.cacr);
    std::fprintf(stderr,
        "[state-replay]   DTT0=0x%08x DTT1=0x%08x ITT0=0x%08x ITT1=0x%08x\n",
        out.dtt0, out.dtt1, out.itt0, out.itt1);
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// FULL-DESIGN checkpoint / resume (Part 71, from-reset full-SoC boot
// campaign).  Unlike +state_replay= (CPU regs + DRAM only, see
// StateReplay above — the exact gap Part 69/70 hit at a VIA-touching
// PC), this captures EVERYTHING the Verilated model owns: every RTL
// register/array in the whole `fpga_top` hierarchy (CPU core incl. ROB/
// PRF/caches, L2C, AXI fabric, the DDR/ROM behavioural-BRAM backing
// array u_ddr.mem_b0..b15 — see the ddr_write_phys32/ddr_read_phys32
// backdoor helpers above, which write into that SAME array — and every
// peripheral: via1/via2/scc/rtc/adb/scsi/dafb/asc/irq_agg/...) via
// Verilator's own `--savable` serialization (VerilatedSave/
// VerilatedRestore, see verilated_save.h).  This is a real Verilator
// built-in, not a custom serializer: `os << *dut` walks the whole
// design's `___024root` state (confirmed end-to-end with a standalone
// smoke test before wiring this in).  What it does NOT cover (and
// doesn't need to, for this harness's purposes): host-side C++ objects
// that live outside the Verilated model, e.g. the SD-card SPI behavioural
// model (tb/models/sd_card_spi.h) and open log-file handles/offsets —
// none of those matter for a run that boots via the DDR-backdoor ROM
// preload path (the default, non-+no_preload path this campaign uses),
// since the modelled SD card is never touched on that path.
//
// A tiny text sidecar (`<base>.meta`) carries the handful of testbench-
// side scalars VerilatedSave doesn't own: `sim_time` (this file's own
// free-running edge counter — NOT part of the Verilated model, tick()
// increments it manually) plus the CPU_M68K040 retire-count/last-PC
// globals used for progress reporting.  Two files together
// (`<base>.vltsav` + `<base>.meta`) are one checkpoint.
//
// Naming/indexing scheme: `ckpt_<cycle>.vltsav` / `.meta`, where <cycle>
// is the zero-padded (20 digits, so directory listings sort
// lexicographically = chronologically) clock-cycle count (sim_time/2 —
// one clock edge pair = one `tick()` call = one clock cycle).
// `<dir>/latest.txt` always names the most recently completed
// checkpoint's base path, so a resuming run doesn't need to know the
// exact cycle count.  Writes go to a `.tmp` suffix first and are
// rename()'d into place after both files are fully written, so a run
// killed mid-checkpoint never leaves a partial file at the "real" name.
static bool checkpoint_save(const std::string& dir, uint64_t cycle) {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    char namebuf[48];
    std::snprintf(namebuf, sizeof(namebuf), "ckpt_%020llu",
                  (unsigned long long)cycle);
    const std::string base      = dir + "/" + namebuf;
    const std::string vlt_path  = base + ".vltsav";
    const std::string meta_path = base + ".meta";
    const std::string tmp_vlt   = vlt_path + ".tmp";
    const std::string tmp_meta  = meta_path + ".tmp";

    {
        VerilatedSave os;
        os.open(tmp_vlt.c_str());
        if (!os.isOpen()) {
            std::fprintf(stderr, "[checkpoint] FAILED to open %s for write\n",
                         tmp_vlt.c_str());
            return false;
        }
        os << *dut;
        os.close();
    }
#ifdef CPU_M68K040
    const unsigned long long retired_now = (unsigned long long)g_cpu040_retired_count;
    const uint32_t last_pc_now = g_cpu040_last_pc;
#else
    const unsigned long long retired_now = (unsigned long long)g_retired_count;
    const uint32_t last_pc_now = 0;
#endif
    {
        std::FILE* f = std::fopen(tmp_meta.c_str(), "w");
        if (!f) {
            std::fprintf(stderr, "[checkpoint] FAILED to open %s for write\n",
                         tmp_meta.c_str());
            return false;
        }
        std::fprintf(f, "# checkpoint_v1\n");
        std::fprintf(f, "CYCLE=%llu\n", (unsigned long long)cycle);
        std::fprintf(f, "SIM_TIME=%llu\n", (unsigned long long)sim_time);
        std::fprintf(f, "RETIRED=%llu\n", retired_now);
        std::fprintf(f, "LAST_PC=0x%08x\n", last_pc_now);
        std::fclose(f);
    }
    std::rename(tmp_vlt.c_str(), vlt_path.c_str());
    std::rename(tmp_meta.c_str(), meta_path.c_str());
    {
        const std::string latest_path = dir + "/latest.txt";
        std::FILE* lf = std::fopen(latest_path.c_str(), "w");
        if (lf) {
            std::fprintf(lf, "%s\n", base.c_str());
            std::fclose(lf);
        }
    }
    std::fprintf(stderr,
        "[checkpoint] SAVED cycle=%llu sim_time=%llu retired=%llu "
        "last_pc=0x%08x -> %s{.vltsav,.meta}\n",
        (unsigned long long)cycle, (unsigned long long)sim_time,
        retired_now, last_pc_now, base.c_str());
    std::fflush(stderr);
    return true;
}

// Resume: constructs `dut` fresh, restores its ENTIRE state from
// `<base>.vltsav`, and restores the testbench-side sidecar scalars from
// `<base>.meta`.  Caller must NOT run any of the normal reset/preload
// sequence afterward — the restored state already reflects a
// fully-booted-to-that-point design, mid-run.
static bool checkpoint_load(const std::string& base) {
    const std::string vlt_path  = base + ".vltsav";
    const std::string meta_path = base + ".meta";
    dut = new Vfpga_top;
    {
        VerilatedRestore is;
        is.open(vlt_path.c_str());
        if (!is.isOpen()) {
            std::fprintf(stderr, "[checkpoint] FAILED to open %s for read\n",
                         vlt_path.c_str());
            return false;
        }
        is >> *dut;
        is.close();
    }
    unsigned long long loaded_sim_time = 0, loaded_retired = 0;
    unsigned loaded_last_pc = 0;
    std::FILE* f = std::fopen(meta_path.c_str(), "r");
    if (!f) {
        std::fprintf(stderr, "[checkpoint] FAILED to open %s for read\n",
                     meta_path.c_str());
        return false;
    }
    char line[256];
    while (std::fgets(line, sizeof(line), f)) {
        unsigned long long v;
        unsigned pv;
        if (std::sscanf(line, "SIM_TIME=%llu", &v) == 1) { loaded_sim_time = v; continue; }
        if (std::sscanf(line, "RETIRED=%llu", &v) == 1) { loaded_retired = v; continue; }
        if (std::sscanf(line, "LAST_PC=0x%x", &pv) == 1) { loaded_last_pc = pv; continue; }
    }
    std::fclose(f);
    sim_time = loaded_sim_time;
    g_retired_count = loaded_retired;
#ifdef CPU_M68K040
    g_cpu040_retired_count = loaded_retired;
    g_cpu040_last_pc = static_cast<uint32_t>(loaded_last_pc);
#endif
    std::fprintf(stderr,
        "[checkpoint] RESTORED from %s: sim_time=%llu retired=%llu "
        "last_pc=0x%08x\n",
        base.c_str(), (unsigned long long)sim_time,
        (unsigned long long)loaded_retired, loaded_last_pc);
    std::fflush(stderr);
    return true;
}

// Patch ROM reset vectors at offset 0x0..0x7 so the FETCH_RESET_VECTORS
// boot fetch lands on the snapshot's SSP and PC directly — no JMP
// needed.  m68k_core's if_stage with FETCH_RESET_VECTORS=1 reads
// initial SSP from mem[0..3] and initial PC from mem[4..7] before
// retiring any user instruction, and writes those into A7/PC.  If we
// don't patch, the boot fetch reads ROM[0..3]=0x420DBFF3 (the ROM
// stored-checksum) into PRF[committed_a7_phys], blowing away our
// state-replay SSP poke.
static void patch_rom_jmp_to(std::vector<uint8_t>& rom, uint32_t target_pc,
                             uint32_t target_ssp) {
    if (rom.size() < 8) return;
    rom[0x0] = static_cast<uint8_t>((target_ssp >> 24) & 0xff);
    rom[0x1] = static_cast<uint8_t>((target_ssp >> 16) & 0xff);
    rom[0x2] = static_cast<uint8_t>((target_ssp >> 8) & 0xff);
    rom[0x3] = static_cast<uint8_t>(target_ssp & 0xff);
    rom[0x4] = static_cast<uint8_t>((target_pc >> 24) & 0xff);
    rom[0x5] = static_cast<uint8_t>((target_pc >> 16) & 0xff);
    rom[0x6] = static_cast<uint8_t>((target_pc >> 8) & 0xff);
    rom[0x7] = static_cast<uint8_t>(target_pc & 0xff);
    std::fprintf(stderr,
        "[state-replay] patched ROM reset vector: SSP=0x%08x PC=0x%08x "
        "(boot fetch pulls these via FETCH_RESET_VECTORS)\n",
        target_ssp, target_pc);
}

// Pre-populate the DDR backing model with snapshot DRAM bytes.
static void apply_state_dram(const StateReplay& s) {
    if (!s.present) return;
    for (size_t i = 0; i < s.dram.size(); i++)
        ddr_write_phys_byte(static_cast<uint32_t>(i), s.dram[i]);
    std::fprintf(stderr,
        "[state-replay] preloaded %zu bytes of DRAM (0x00000000..0x%08zx)\n",
        s.dram.size(), s.dram.size() - 1);
}

// Poke arch state into the running CPU: PRF[0..15] = D0..D7/A0..A6, A7,
// PRF[19/20/21] = USP/SSP/ISP shadows, and the dedicated CSR regs in
// u_commit (arch_vbr / arch_sr / usp / ssp / isp).  CRAT defaults to
// identity at reset (rat.v:618), so D0→phys 0, D1→phys 1, ..., A7→phys 15
// — no CRAT poke required.  Call this AFTER cpu_rst has deasserted but
// BEFORE the `JMP <snap_pc>.l` retires; the JMP itself doesn't read or
// write any of these regs, so they stay stable until the snapshot code
// starts executing.
static void apply_state_arch(const StateReplay& s) {
    // 2026-07-02: re-enabled under CPU_M68K.  The CPUI() macro above
    // already resolves the extra `u_cpu.u_cpu.` wrapper hop introduced
    // by socketization (m68k_axi_wrapper wraps m68k_core, both named
    // u_cpu), and the CPU-internal signal names below (prf, u_commit.*,
    // u_ccr_rat.*, u_immu/u_dmmu/u_mmu.r_*) are all
    // still direct children of m68k_core in the current cpu/ submodule
    // (verified against rtl/core/{m68k_core_execute,m68k_core_commit,
    // m68k_core_issue,m68k_core_memory}.vh).  So the deep poke works
    // unmodified for both the socketed (CPU_M68K) and legacy builds —
    // the previous "Phase 6 follow-up" no-op was stale.
    if (!s.present) return;
    auto* r = dut->rootp;

    // PRF[0..7] = D0..D7
    for (int i = 0; i < 8; i++)
        CPUI_REF(r, prf)[i] = s.d[i];
    // PRF[8..15] = A0..A7 (live, active bank)
    for (int i = 0; i < 8; i++)
        CPUI_REF(r, prf)[8 + i] = s.a[i];
    // A7 (live) = SP value MAME reported.  MAME's `a7` register is
    // already the live SP, so s.a[7] == s.sp; the snapshot's "SP" key
    // is redundant but kept for sanity.

    // PRF[19] = USP shadow, PRF[20] = SSP shadow, PRF[21] = ISP shadow.
    // SR.S=1 + SR.M=0 → ISP active (= `sp`); SR.S=1 + SR.M=1 → MSP
    // active (we treat MSP as SSP since the pipeline lacks a separate
    // MSP shadow).  SR.S=0 → USP active.
    const bool s_bit = (s.sr >> 13) & 1;
    const bool m_bit = (s.sr >> 12) & 1;
    CPUI_REF(r, prf)[19] = s.usp;            // USP
    if (!s_bit) {
        // user mode: live A7=USP, ISP/SSP unknown — leave as snapshot
        // SP value to keep them sane defaults.
        CPUI_REF(r, prf)[20] = s.sp;
        CPUI_REF(r, prf)[21] = s.sp;
    } else if (m_bit) {
        // supervisor M=1 → MSP active.  Stuff into SSP slot.
        CPUI_REF(r, prf)[20] = s.sp;
        CPUI_REF(r, prf)[21] = s.sp;
    } else {
        // supervisor M=0 → ISP active.  Stuff into ISP slot; SSP
        // unknown, mirror SP.
        CPUI_REF(r, prf)[20] = s.sp;
        CPUI_REF(r, prf)[21] = s.sp;
    }

    // Dedicated CSR shadows in u_commit.
    const uint8_t ccr_val = static_cast<uint8_t>(s.sr & 0x1fu);
    CPUI_REF(r, u_commit__DOT__arch_vbr) = s.vbr;
    CPUI_REF(r, u_commit__DOT__arch_sr)  =
        static_cast<uint16_t>(s.sr);
    CPUI_REF(r, u_ccr_rat__DOT__ccr_prf)[
        CPUI_REF(r, u_ccr_rat__DOT__crat_tag)] = ccr_val;
    CPUI_REF(r, u_commit__DOT__usp) = s.usp;
    CPUI_REF(r, u_commit__DOT__ssp) = s.sp;
    CPUI_REF(r, u_commit__DOT__isp) = s.sp;

    // MMU CSRs.  Three MMU instances (u_immu, u_dmmu,
    // u_mmu) each carry their own CSR copy and stay
    // synchronized via the shared movec-broadcast bus; we poke all
    // three so they boot to the same MAME-equivalent state.
    CPUI_REF(r, u_immu__DOT__r_tc)   = s.tc;
    CPUI_REF(r, u_immu__DOT__r_srp)  = s.srp;
    CPUI_REF(r, u_immu__DOT__r_urp)  = s.urp;
    CPUI_REF(r, u_immu__DOT__r_dtt0) = s.dtt0;
    CPUI_REF(r, u_immu__DOT__r_dtt1) = s.dtt1;
    CPUI_REF(r, u_immu__DOT__r_itt0) = s.itt0;
    CPUI_REF(r, u_immu__DOT__r_itt1) = s.itt1;
    CPUI_REF(r, u_dmmu__DOT__r_tc)   = s.tc;
    CPUI_REF(r, u_dmmu__DOT__r_srp)  = s.srp;
    CPUI_REF(r, u_dmmu__DOT__r_urp)  = s.urp;
    CPUI_REF(r, u_dmmu__DOT__r_dtt0) = s.dtt0;
    CPUI_REF(r, u_dmmu__DOT__r_dtt1) = s.dtt1;
    CPUI_REF(r, u_dmmu__DOT__r_itt0) = s.itt0;
    CPUI_REF(r, u_dmmu__DOT__r_itt1) = s.itt1;
    CPUI_REF(r, u_mmu__DOT__r_tc)   = s.tc;
    CPUI_REF(r, u_mmu__DOT__r_srp)  = s.srp;
    CPUI_REF(r, u_mmu__DOT__r_urp)  = s.urp;
    CPUI_REF(r, u_mmu__DOT__r_dtt0) = s.dtt0;
    CPUI_REF(r, u_mmu__DOT__r_dtt1) = s.dtt1;
    CPUI_REF(r, u_mmu__DOT__r_itt0) = s.itt0;
    CPUI_REF(r, u_mmu__DOT__r_itt1) = s.itt1;

    // i-mmu-walker (2026-05-06): clear the cold-boot ROM overlay so
    // post-boot DDR reads at PA 0..0x3FFFFF return DRAM (not aliased
    // ROM bytes).  At the snapshot's PC the OS has long since cleared
    // the overlay; the RTL boots with overlay active by default
    // (ddrb=0 → overlay_live=1) because the OS poke happens via VIA1
    // register writes the snapshot skips.  Without this, the I-side
    // MMU walker reading PT entries from SRP-rooted DRAM gets garbage
    // (ROM[0x003fdc80] = whatever, not the snapshot's PT entry),
    // causing every walk to fault on the first PTE.  Set ddrb[3]=1
    // and orb[3]=0 to drive overlay_live=0.  Also mirror the patched
    // SSP/PC into DRAM[0..7] so the cold-boot vector fetch (now
    // routed to DRAM, not aliased ROM) reads the snapshot's intended
    // resume PC.
    {
        uint8_t ddrb = r->fpga_top__DOT__u_via1__DOT__ddrb;
        uint8_t orb  = r->fpga_top__DOT__u_via1__DOT__orb;
        ddrb |= 0x08u;
        orb  &= ~0x08u;
        r->fpga_top__DOT__u_via1__DOT__ddrb = ddrb;
        r->fpga_top__DOT__u_via1__DOT__orb  = orb;
        ddr_write_phys32(0x00000000u, s.sp);
        ddr_write_phys32(0x00000004u, s.pc);
        std::fprintf(stderr,
            "[state-replay] cleared ROM overlay: VIA1 ddrb=0x%02x orb=0x%02x; "
            "DRAM[0..7] set to SSP=0x%08x PC=0x%08x\n",
            ddrb, orb, s.sp, s.pc);
    }
    std::fprintf(stderr,
        "[state-replay] arch state poked: PC=0x%08x SR=0x%04x CCR=0x%02x "
        "VBR=0x%08x (S=%d M=%d) SP=0x%08x USP=0x%08x\n",
        s.pc, s.sr, ccr_val, s.vbr, s_bit, m_bit, s.sp, s.usp);
    // Readback validation: did the pokes stick?
    for (int i = 0; i < 8; i++) {
        std::fprintf(stderr,
            "[state-replay] readback PRF[%d]=0x%08x  PRF[%d]=0x%08x\n",
            i, (unsigned)CPUI_REF(r, prf)[i],
            8 + i, (unsigned)CPUI_REF(r, prf)[8 + i]);
    }
    {
        const unsigned ccr_phys =
            CPUI_REF(r, u_ccr_rat__DOT__crat_tag);
        const unsigned arch_ccr =
            CPUI_REF(r, u_ccr_rat__DOT__ccr_prf)[ccr_phys];
        const unsigned live_sr =
            (CPUI_REF(r, u_commit__DOT__arch_sr) & 0xffe0u) |
            (arch_ccr & 0x1fu);
        std::fprintf(stderr,
            "[state-replay] readback arch_vbr=0x%08x arch_sr=0x%04x "
            "arch_ccr=0x%02x seeded_ccr=0x%02x live_sr=0x%04x "
            "usp=0x%08x ssp=0x%08x isp=0x%08x\n",
            (unsigned)CPUI_REF(r, u_commit__DOT__arch_vbr),
            (unsigned)CPUI_REF(r, u_commit__DOT__arch_sr),
            arch_ccr,
            ccr_val,
            live_sr,
            (unsigned)CPUI_REF(r, u_commit__DOT__usp),
            (unsigned)CPUI_REF(r, u_commit__DOT__ssp),
            (unsigned)CPUI_REF(r, u_commit__DOT__isp));
    }
}

#ifdef CPU_M68K040
// Restore a v2 architectural snapshot through the core's real halted-only
// ARCH_APPLY machinery.  CPUI_REF is intentionally inert for CPU_M68K040;
// poking it used to make +state_replay silently continue with zero registers.
// Driving the apply sequencer here exercises the same committed-map/system
// register write paths as JTAG and gives the frontend a clean PC redirect.
static bool apply_state_arch_cpu040(const StateReplay& s) {
    if (!s.present) return true;
    auto* r = dut->rootp;

#define CPU040_CORE(member) \
    (r->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__##member)

    // Request a precise macro-boundary halt.  The request register is a
    // one-cycle pulse, so one tick is sufficient to hand it to the ROB.
    CPU040_CORE(DebugCtrlPlugin_logic_csr_debugStopRequest) = 1;
    tick();
    unsigned halt_wait = 0;
    while (!CPU040_CORE(RobPlugin_logic_debugHalted) && halt_wait < 100000) {
        tick();
        ++halt_wait;
    }
    if (!CPU040_CORE(RobPlugin_logic_debugHalted)) {
        std::fprintf(stderr,
            "[state-replay] ERROR: cpu040 precise halt timed out\n");
        return false;
    }

    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_0)  = s.d[0];
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_1)  = s.d[1];
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_2)  = s.d[2];
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_3)  = s.d[3];
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_4)  = s.d[4];
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_5)  = s.d[5];
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_6)  = s.d[6];
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_7)  = s.d[7];
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_8)  = s.a[0];
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_9)  = s.a[1];
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_10) = s.a[2];
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_11) = s.a[3];
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_12) = s.a[4];
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_13) = s.a[5];
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_14) = s.a[6];
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_15) = s.a[7];
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_16) = s.usp;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_17) = s.sp;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_18) = s.sp;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_19) = s.sr;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_20) = s.vbr;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_21) = s.cacr;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_22) = s.tc;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_23) = s.itt0;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_24) = s.itt1;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_25) = s.dtt0;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_26) = s.dtt1;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_27) = s.urp;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_28) = s.srp;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_29) = s.pc;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_30) = 0;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyShadow_31) = 0;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyDirty) = 0xffffffffu;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyRewrite) = 0;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyIndex) = 0;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyPhase) = 0;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyDone) = 0;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyRejected) = 0;
    CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyBusy) = 1;

    unsigned apply_wait = 0;
    while (CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyBusy) &&
           apply_wait < 100000) {
        tick();
        ++apply_wait;
    }
    if (CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyBusy) ||
        CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyRejected) ||
        !CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyDone)) {
        std::fprintf(stderr,
            "[state-replay] ERROR: cpu040 ARCH_APPLY failed "
            "busy=%u done=%u rejected=%u phase=%u\n",
            (unsigned)CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyBusy),
            (unsigned)CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyDone),
            (unsigned)CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyRejected),
            (unsigned)CPU040_CORE(DebugCtrlPlugin_logic_csr_archApplyPhase));
        return false;
    }

    // A post-MMU snapshot assumes the reset ROM overlay has already been
    // removed.  Otherwise the freshly restored page-table walker reads ROM
    // bytes at low physical addresses instead of the snapshot's DRAM PTEs.
    {
        uint8_t ddrb = r->fpga_top__DOT__u_via1__DOT__ddrb;
        uint8_t orb  = r->fpga_top__DOT__u_via1__DOT__orb;
        ddrb |= 0x08u;
        orb  &= ~0x08u;
        r->fpga_top__DOT__u_via1__DOT__ddrb = ddrb;
        r->fpga_top__DOT__u_via1__DOT__orb  = orb;
        ddr_write_phys32(0x00000000u, s.sp);
        ddr_write_phys32(0x00000004u, s.pc);
        std::fprintf(stderr,
            "[state-replay] cpu040 cleared ROM overlay: "
            "VIA1 ddrb=0x%02x orb=0x%02x\n", ddrb, orb);
    }

    CPU040_CORE(DebugCtrlPlugin_logic_csr_debugResumeRequest) = 1;
    tick();
    unsigned resume_wait = 0;
    while (CPU040_CORE(RobPlugin_logic_debugHalted) && resume_wait < 100000) {
        tick();
        ++resume_wait;
    }
    if (CPU040_CORE(RobPlugin_logic_debugHalted)) {
        std::fprintf(stderr,
            "[state-replay] ERROR: cpu040 resume timed out\n");
        return false;
    }
    std::fprintf(stderr,
        "[state-replay] cpu040 ARCH_APPLY complete: halt_wait=%u "
        "apply_wait=%u resume_wait=%u PC=0x%08x A1=0x%08x SR=0x%04x\n",
        halt_wait, apply_wait, resume_wait, s.pc, s.a[1], s.sr);
#undef CPU040_CORE
    return true;
}
#endif

static void dump_ppm(const std::string& path,
                     const std::vector<uint32_t>& rgb) {
    std::filesystem::path p(path);
    if (!p.parent_path().empty())
        std::filesystem::create_directories(p.parent_path());
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
        std::perror(path.c_str());
        return;
    }
    std::fprintf(f, "P6\n%d %d\n255\n", FRAME_W, FRAME_H);
    for (uint32_t px : rgb) {
        const uint8_t b[3] = {
            static_cast<uint8_t>((px >> 16) & 0xff),
            static_cast<uint8_t>((px >> 8) & 0xff),
            static_cast<uint8_t>(px & 0xff),
        };
        std::fwrite(b, 1, 3, f);
    }
    std::fclose(f);
}

// Decode a host-supplied byte sequence string with simple C-style escapes.
// Supports \r \n \t \0 \\ \xHH and bare characters.  Used by +scc_rx_inject=
// so a user can spell out e.g. "G\r" or "\x47\x0d" interchangeably.
static bool decode_byte_string(const std::string& in, std::vector<uint8_t>& out) {
    out.clear();
    for (size_t i = 0; i < in.size(); ) {
        char c = in[i];
        if (c != '\\') {
            out.push_back(static_cast<uint8_t>(c));
            i++;
            continue;
        }
        if (i + 1 >= in.size()) return false;
        char n = in[i + 1];
        switch (n) {
        case 'r': out.push_back('\r'); i += 2; break;
        case 'n': out.push_back('\n'); i += 2; break;
        case 't': out.push_back('\t'); i += 2; break;
        case '0': out.push_back(0);    i += 2; break;
        case '\\': out.push_back('\\'); i += 2; break;
        case 'x': {
            if (i + 3 >= in.size()) return false;
            char hi = in[i + 2], lo = in[i + 3];
            auto hex = [](char x) -> int {
                if (x >= '0' && x <= '9') return x - '0';
                if (x >= 'a' && x <= 'f') return 10 + (x - 'a');
                if (x >= 'A' && x <= 'F') return 10 + (x - 'A');
                return -1;
            };
            int h = hex(hi), l = hex(lo);
            if (h < 0 || l < 0) return false;
            out.push_back(static_cast<uint8_t>((h << 4) | l));
            i += 4;
            break;
        }
        default:
            return false;
        }
    }
    return true;
}

static const char* macsbug_pc_kind(uint32_t pc) {
    if (pc >= 0x4084af9cu && pc <= 0x4084afceu) return "rom_scc_rx_*";
    if (pc >= 0x40849b00u && pc <  0x4084b000u) return "macsbug_range";
    if (pc >= 0x4084b000u && pc <  0x4084b800u) return "macsbug_extended";
    if (pc >= 0x408be100u && pc <  0x408be400u) return "macsbug_scc_tx_poll";
    return nullptr;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    std::string rom_path = "files/420dbff3.rom";
    std::string rom_patch_set;
    std::string ppm_path = "build/fpga_top_rom_100m/scaler_scanout.ppm";
    uint64_t max_retired = 100000000ULL;
    uint64_t timeout_cycles = 2000000000ULL;
    // Full-design periodic checkpointing (Part 71).
    std::string checkpoint_dir;                        // empty = disabled
    uint64_t checkpoint_interval_cycles = 10000000ULL;  // ~10M clock cycles
    std::string load_checkpoint_base;                   // empty = cold boot
    bool stop_floppy_loop = false;
    bool stop_monitor = false;
    bool probe = false;
    // +exc_log[=<max>] — one stderr line per taken exception (vector,
    // faulting PC, fault address).  Cheap enough to leave on for a whole
    // boot, unlike +probe's per-retire firehose.
    uint64_t exc_log_max = 0;
    uint64_t exc_log_count = 0;

    // ── SD card image ────────────────────────────────────────────────
    // +sd_image=<path>       raw Mac HDD image; file byte 0 is SCSI LBA 0,
    //                        i.e. it is mounted at SD LBA 8192 (the
    //                        RESERVED_LBAS bias that sd_scsi_lba_mapper.v /
    //                        sd_ctrl.v's RAW_BASE_LBA apply).  Same
    //                        convention as tb_scsi_fuzz.cpp's --img.
    // +sd_card_image=<path>  whole-card image; file byte 0 is SD LBA 0
    //                        (what `m68kctl sd make-image` produces).
    // +sd_trace              log every SD command to stderr.
    // +sd_writeback          let writes reach the image file.  OFF by
    //                        default: writes land in an in-memory overlay
    //                        so a sim can never corrupt the user's disk.
    // Without either image arg the card is absent and sd_miso stays tied
    // high, exactly as this harness behaved before the model existed.
    // +scsi_config3=<byte> — seed rtl/mac/scsi.v's 53C96 config3 register.
    //
    // Why this exists: +state_replay restores the CPU (regs + MMU CSRs) and
    // 4 MiB of DRAM, but NOT peripheral register state.  The Q700 ROM
    // programs config3 = 0x04 (LBTM) exactly once, at ROM PC 0x4089911a,
    // during SCSI Manager init — roughly 11 million instructions before the
    // boot-blocks read.  A snapshot taken at the SCSISelect that precedes
    // that read therefore replays with config3 = 0x00, and LBTM-dependent
    // DRQ behaviour (the whole point of the 0x4089931c chunk-drain bug)
    // silently cannot happen.  Seeding it here restores the machine state
    // the snapshot implies.  Ignored without +state_replay-style setups;
    // the register is written normally on a full boot.
    int scsi_config3 = -1;
    // +dafb_scsi_ctrl=<9-bit> — seed the DAFB bus-1 control word
    // (rtl/mac/video.v regs[REG_FIRST_HIT][8:0], MMIO 0xF980_0024), which
    // is what reaches scsi.v as `scsi_ctrl_in`.  Bit 7 = "DRQ-check this
    // read through the TurboSCSI pseudo-DMA aperture", bit 8 = the write
    // equivalent.  Same rationale as +scsi_config3: the ROM sets this up
    // long before a +state_replay snapshot point, and with it at 0 every
    // DRQ gate in the pseudo-DMA path is inert — so the drain cannot fail
    // the way it does on hardware.
    int dafb_scsi_ctrl = -1;
    // +no_preload — skip preload_rom()'s DRAM-backdoor write entirely.
    // Task: realboot verification (docs background: "boot_fsm mirror
    // ROM into low RAM"). preload_rom() writes straight into the
    // verilator DDR model's internal arrays at a legacy/toy offset,
    // completely bypassing boot_fsm/SD -- fine for the FPGA_ROM_SIM
    // fast-boot harness (which also force-releases cpu_rst via
    // vio_boot_ctrl and never runs boot_fsm at all), but it contaminates
    // any attempt to observe boot_fsm's REAL SD-streaming-and-mirror
    // path, which is the whole point of a realboot run: with this flag
    // set, the CPU can only ever see ROM content that boot_fsm itself
    // streamed in from the attached +sd_card_image, through the real
    // AXI mirror-to-0x0 + native-0x40000000 writes (rtl/soc/boot_fsm.v
    // MIRROR_LOW_RAM), never through this host-side backdoor.
    bool no_preload = false;
    std::string sd_image_path;
    std::string sd_card_image_path;
    bool sd_trace = false;
    bool sd_writeback = false;
    bool fail_monitor = false;
    // +halt_on_vec=<n> (repeatable): when an exception with this vec
    // fires, log the full arch state and STOP the sim immediately.
    // Used to catch the post-DBF wild-jump's illegal-instruction
    // (vec=4) entry point cleanly.  Multiple --halt_on_vec flags allow
    // halting on more than one vector.
    // Entries are (vec, fault_pc); fault_pc == 0xFFFFFFFF means "any PC".
    // "+halt_on_vec=2:0x4089931c" halts only on a vector-2 whose faulting
    // instruction is that PC — the ROM takes benign vector-2 probes during
    // boot, so an unqualified vec-2 halt stops on the wrong one.
    std::vector<std::pair<uint32_t, uint32_t> > halt_on_vecs;
    // +expect_floppy_poll: success when the CPU eventually reaches the
    // canonical disk-prompt poll loop (PC == DISK_PROMPT_PC) at least
    // +expect_floppy_poll_min_hits times before the timeout.  Failure =
    // timeout reached without hitting that count.  We do NOT use any
    // MacsBug-range violation here: MAME's macqd700 reaches and dwells
    // inside MacsBug-range PCs (e.g. 0x4084b0a4) as part of normal boot
    // before proceeding to the floppy prompt — so MacsBug-range entry
    // is not a reliable trap signal.
    bool expect_floppy_poll = false;
    uint64_t expect_floppy_poll_min_hits = 100000ULL;
    uint64_t disk_prompt_hit_count = 0;
    bool expect_floppy_poll_satisfied = false;
    bool expect_q700_feature = false;
    uint32_t expected_q700_feature = 0;
    std::string pc_dump_path;
    uint64_t pc_dump_max = 10000;
    uint64_t pc_dump_every = 1;

    // +audio_dump=<path> — capture every ASC audio_sample_valid edge
    // with the L/R PCM samples + retired count + sim_time.  Empty path
    // disables capture (default).  Format: one line per sample,
    // tab-separated `<retired>\t<sim_time>\t<pcm_l_int16>\t<pcm_r_int16>`.
    std::string audio_dump_path;
    uint64_t audio_dump_max = 0;       // 0 = unlimited
    uint64_t audio_dump_count = 0;
    FILE* audio_dump_fp = nullptr;

    // +asc_event_dump=<path> — capture every ASC peripheral-bus access
    // (read or write to 0x800-0xFFF or 0x000-0x7FF FIFO regions).
    // Format: `<retired>\t<sim_time>\t<R/W>\t<addr_hex>\t<data_hex>\t<pc_hex>`.
    std::string asc_event_dump_path;
    uint64_t asc_event_dump_max = 0;
    uint64_t asc_event_dump_count = 0;
    FILE* asc_event_dump_fp = nullptr;

    // +state_replay=<path> — load a MAME-dumped snapshot and seed
    // RTL with arch state + DRAM contents, then patch reset entry
    // (ROM offset 0x2A) with `JMP <snap_pc>.l` so the CPU lands at
    // the snapshot PC after coming out of reset.  See
    // tools/mame_state_dump.py for the snapshot format.
    std::string state_replay_path;

    // +watch_pa=<hex> — poll DDR backing at a 4-byte-aligned PA each
    // cycle and log every change with the retired count + last
    // committed PC.  Repeatable.  Use to find which instruction wrote
    // a corrupted page-table entry, etc.
    std::vector<uint32_t> watch_pas;
    // +dump_word_at=<hex addr> (2026-09-03): read a live WORD out of the
    // ddr_ctrl SIM_MODEL backing array (mem_b0/mem_b1, byte-lane split,
    // addr_to_idx()'s a[25:4]/a[3:0] decode -- see rtl/board/ddr_ctrl.v)
    // and print it, right after a +load_checkpoint= restore, no cycles
    // run. Repeatable. Backdoor DDR read, same coherency caveat as
    // Part 76's own poke -- not I-cache/D-cache coherent, only useful for
    // addresses nothing has cached (low-memory globals like the
    // calibration word are the intended use case).
    std::vector<uint32_t> dump_word_at_addrs;

    // +arch_dump_at_pc=<hex> +arch_dump_max=<n>
    // On every retire whose boundary_pc matches one of the requested
    // PCs, dump committed D0-D7, A0-A7, CCR to stderr.  Repeatable;
    // shared `arch_dump_count` cap.  Output format mirrors MAME's
    // bpset+tracelog so the lines diff cleanly side-by-side against
    // golden state captured via `m68k-mame-debugger`.
    std::vector<uint32_t> arch_dump_pcs;
    uint64_t arch_dump_max = 16;
    uint64_t arch_dump_count = 0;

    // +poke_word_at_pc=<pc_hex>:<addr_hex>:<value_hex> -- on every retire
    // whose boundary_pc matches <pc_hex>, write <value_hex> (16 bits) to
    // physical DDR address <addr_hex>. Repeatable. Fires unconditionally on
    // every hit (idempotent for a fixed target address), no CPU-specific
    // register tap needed -- this pokes DDR directly, same class of
    // operation as maybe_apply_ramtest_mame_state_fastfill above. Built to
    // test the docs/BUG_calibration_word_misplaced_0d00.md Part 7 fix
    // hypothesis: does injecting the MAME-reference-measured calibration
    // value at 0x00000D00 unblock the DIVU-by-zero boot fault, without any
    // RTL change at all.
    struct PokeWordAtPc { uint32_t pc; uint32_t addr; uint16_t value; };
    std::vector<PokeWordAtPc> poke_word_at_pc;
    uint64_t poke_word_hits = 0;

    // SCC end-to-end RX/TX validation knobs.
    //
    // +scc_rx_inject=<bytes>   Drive the canonical input sequence at MacsBug
    //                          entry.  Bytes accept C-style escapes:
    //                            +scc_rx_inject="G\r"  ==  +scc_rx_inject="\x47\x0d"
    //                          Bytes are queued on the board UART RX line
    //                          (uart_rtl_0_rxd) at the configured baud rate
    //                          (default 115200) one by one once the CPU
    //                          first lands inside the MacsBug RX poll
    //                          (rom_scc_rx_*).  This exercises the full
    //                          uart_byte_bridge → SCC RX FIFO → RR0[0] →
    //                          RR8 read path.
    // +scc_rx_inject_fast=1    Bypass the UART-bit serialisation and pulse
    //                          the SCC's external rx_valid/rx_data byte
    //                          interface directly.  Drops latency by ~10×;
    //                          the byte is observed inside the SCC FIFO
    //                          one pb_clk cycle after MacsBug entry.
    //                          Default = bit-serial (true to HW path).
    // +scc_rx_inject_at=<pc>   Override the MacsBug entry trigger PC.  Default
    //                          fires the queue once any rom_scc_rx_* / MacsBug
    //                          range PC retires.
    // +scc_tx_log=<path>       Capture every CPU-side TX byte (snapshot from
    //                          scc_uart_tx_valid + data, fires once per
    //                          completed SCC byte time).  Each line:
    //                            <retired> <sim_time> <byte_hex> '<ascii>'
    // +scc_tx_uart_log=<path>  Capture every UART-pin TX byte (post serial
    //                          framer, observed by sniffing uart_rtl_0_txd
    //                          at the same baud).  Same format as above.
    // +scc_tx_max=<n>          Stop after capturing N bytes EACH on tx_log
    //                          and tx_uart_log (default 64).
    //
    // NMI / programmer's-switch fire knobs.  The Q700 ROM's MacsBug is
    // dormant code in the ROM image — it has no auto-entry on a clean
    // boot.  On real hardware MacsBug is invoked by the programmer's
    // switch (level-7 NMI on real Q700 silicon, board btn[1] on this
    // KU5P bitstream).  This testbench needs an equivalent — without
    // it, the ROM never enters MacsBug, never polls the SCC RX FIFO,
    // and the +scc_rx_inject path injects bytes into a peripheral
    // nobody is reading.  See docs/scc_hw_runbook.md and the BUG note
    // at docs/BUG_macsbug_repl_unreached.md.
    //
    // +nmi_at_inst=<N>         Fire one NMI pulse once `dbg_committed`
    //                          first reaches N retired uops.  Drives
    //                          u_btn1_db.out_n=0 for nmi_pulse_cycles
    //                          sys_clk ticks then back to 1, producing
    //                          a clean rising edge into irq_agg.
    // +nmi_at_pc=<hex>         Fire NMI when a retiring boundary_pc
    //                          first equals the given PC (hex, with
    //                          or without 0x prefix).
    // +nmi_pulse_cycles=<N>    Hold the press for N sys_clk cycles
    //                          (default 4 — enough for the irq_agg
    //                          rising-edge detector to latch).
    // +irq_at_inst=<N>:<L>[:C[:G]]
    //                          Inject autovector IRQ level L (1..7) when
    //                          retired count first reaches N.  Optional C
    //                          repeats the injection C times; optional G is
    //                          the retired-instruction gap between repeats.
    // +irq_at_pc=<hex>:<L>[:C[:G]]
    //                          Same, but arms when a retiring boundary_pc
    //                          first equals PC.
    // +irq_at_cycle=<N>:<L>[:C[:G]]
    //                          Same, but arms when the sim-cycle counter
    //                          first reaches N.  Cycle-precise — places
    //                          the IRQ at an exact cycle, reaching short
    //                          windows that retire-count / PC-match
    //                          injection cannot (e.g. a dispatcher
    //                          prologue).  Sweep N to find a timing race.
    //                          The RTL holds the injected IPL until
    //                          cpu_ipl_ack_w, matching JTAG injection.
    bool scc_rx_inject = false;
    std::string scc_rx_inject_str;
    bool scc_rx_inject_fast = false;
    uint32_t scc_rx_inject_at = 0;
    bool scc_rx_inject_at_set = false;
    std::string scc_tx_log_path;
    std::string scc_tx_uart_log_path;
    uint64_t scc_tx_max = 64;
    uint64_t nmi_at_inst = 0;
    bool     nmi_at_inst_set = false;
    uint32_t nmi_at_pc = 0;
    bool     nmi_at_pc_set = false;
    uint64_t nmi_pulse_cycles = 4;
    bool     nmi_fired = false;
    bool     nmi_fired_pc = false;
    uint64_t nmi_press_active_until = 0;
    uint64_t irq_at_inst = 0;
    bool     irq_at_inst_set = false;
    uint32_t irq_at_pc = 0;
    bool     irq_at_pc_set = false;
    // +irq_at_cycle=<N>:<L>[:C[:G]] — cycle-precise IRQ injection.
    // Drives g_irqc_* globals consumed by irq_cycle_tick() (called from
    // tick()), so it is active during the state-replay boot-tick phase
    // too — reaching short windows like the A-line dispatcher prologue
    // that the main-loop-resident +irq_at_inst / +irq_at_pc cannot.
    uint8_t  irq_inject_level = 0;
    uint64_t irq_inject_total = 1;
    uint64_t irq_inject_gap = 1;
    bool     irq_triggered = false;
    uint64_t irq_remaining = 0;
    uint64_t irq_next_retired = 0;

    // +axi_lockstep_log=<path> — capture every CPU-side AXI transaction
    // outside the DDR (0x0000_0000..0x3FFF_FFFF) and ROM
    // (0x4000_0000..0x40FF_FFFF) windows.  Output format matches
    // tools/mame_axi_capture.lua (CSV: <seq>,<rw>,<addr>,<size>,<data>) so
    // tools/axi_lockstep_diff.py can byte-diff the two streams.  Used by
    // `make tb-axi-lockstep` to find the first peripheral / VRAM / DAFB
    // access where the CPU's bus traffic diverges from MAME.  See
    // docs/axi_lockstep.md.
    std::string axi_log_path;
    uint64_t axi_log_max = 5000;

    // +via1_lockstep_log=<path> — capture every VIA1 register access at
    // the peripheral_bus master face (pb_via1_*).  Output format matches
    // tools/mame_via1_capture.lua (CSV: <seq>,<rw>,<reg>,<byte>) so
    // tools/via1_lockstep_diff.py can byte-diff the two streams.  Used by
    // `make tb-via1-lockstep` to verify byte parity of the VIA1 register
    // file against MAME's via6522_device.  See docs/via1_lockstep.md.
    std::string via1_log_path;
    uint64_t via1_log_max = 4096;

    // +dafb_lockstep_log=<path> — capture every CPU-side DAFB-related AXI
    // transaction (DAFB register aperture 0xF980_0000..0xF980_03FF +
    // TurboSCSI register window 0x5000_F000..0x5000_F0FF + TurboSCSI DMA
    // handshake 0x5000_F100..0x5000_F101 + optional VRAM window).  Output
    // format matches tools/mame_dafb_capture.lua so
    // tools/dafb_lockstep_diff.py can byte-diff the two streams.  See
    // docs/dafb_lockstep.md.
    std::string dafb_log_path;
    uint64_t dafb_log_max = 8000;
    bool dafb_log_include_vram = false;

    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a.rfind("+rom=", 0) == 0) rom_path = a.substr(5);
        else if (a.rfind("+rom_patch=", 0) == 0) rom_patch_set = a.substr(11);
        else if (a.rfind("+ppm=", 0) == 0) ppm_path = a.substr(5);
        else if (a.rfind("+max_insts=", 0) == 0)
            max_retired = std::stoull(a.substr(11));
        else if (a.rfind("+timeout=", 0) == 0)
            timeout_cycles = std::stoull(a.substr(9));
        else if (a == "+stop_floppy_loop")
            stop_floppy_loop = true;
        else if (a == "+expect_floppy_poll")
            expect_floppy_poll = true;
        else if (a.rfind("+expect_floppy_poll_min_hits=", 0) == 0)
            expect_floppy_poll_min_hits = std::stoull(a.substr(29));
        else if (a == "+stop_monitor")
            stop_monitor = true;
        else if (a == "+probe")
            probe = true;
        else if (a.rfind("+sd_image=", 0) == 0)
            sd_image_path = a.substr(10);
        else if (a.rfind("+sd_card_image=", 0) == 0)
            sd_card_image_path = a.substr(15);
        else if (a.rfind("+c96_probe_at_pc=", 0) == 0) {
            g_c96_probe_pc = std::stoul(a.substr(17), nullptr, 0);
            if (g_c96_probe_max == 0) g_c96_probe_max = 64;
        }
        else if (a.rfind("+c96_probe_max=", 0) == 0)
            g_c96_probe_max = std::stoull(a.substr(15));
        else if (a.rfind("+scsi_config3=", 0) == 0)
            scsi_config3 = (int)std::stoul(a.substr(14), nullptr, 0);
        else if (a.rfind("+dafb_scsi_ctrl=", 0) == 0)
            dafb_scsi_ctrl = (int)std::stoul(a.substr(16), nullptr, 0);
        else if (a == "+no_preload")
            no_preload = true;
        else if (a == "+sd_trace")
            sd_trace = true;
        else if (a == "+sd_writeback")
            sd_writeback = true;
        else if (a.rfind("+halt_on_vec=", 0) == 0) {
            std::string spec = a.substr(13);
            uint32_t want_pc = 0xFFFFFFFFu;
            const size_t colon = spec.find(':');
            if (colon != std::string::npos) {
                want_pc = std::stoul(spec.substr(colon + 1), nullptr, 0);
                spec = spec.substr(0, colon);
            }
            uint32_t v = std::stoul(spec, nullptr, 0);
            halt_on_vecs.push_back(std::make_pair(v, want_pc));
        }
        else if (a == "+exc_log")
            exc_log_max = 1000;
        else if (a.rfind("+exc_log=", 0) == 0)
            exc_log_max = std::stoull(a.substr(9));
        else if (a == "+fail_monitor")
            fail_monitor = true;
        else if (a.rfind("+pc_dump_path=", 0) == 0)
            pc_dump_path = a.substr(14);
        else if (a.rfind("+pc_dump_max=", 0) == 0)
            pc_dump_max = std::stoull(a.substr(13));
        else if (a.rfind("+pc_dump_every=", 0) == 0)
            pc_dump_every = std::stoull(a.substr(15));
        else if (a.rfind("+sq_trace_cycles=", 0) == 0)
            g_sq_trace_cycles = std::stoull(a.substr(17));
        else if (a.rfind("+pipe_trace_cycles=", 0) == 0)
            g_pipe_trace_cycles = std::stoull(a.substr(19));
        else if (a.rfind("+irq_trace_cycles=", 0) == 0)
            g_irq_trace_cycles = std::stoull(a.substr(18));
        else if (a.rfind("+irq_trace_path=", 0) == 0) {
            g_irq_trace_fp = std::fopen(a.substr(16).c_str(), "w");
            if (!g_irq_trace_fp)
                std::fprintf(stderr, "[fpga-rom] cannot open irq_trace_path=%s\n", a.substr(16).c_str());
        }
        else if (a.rfind("+pipe_trace_path=", 0) == 0) {
            g_pipe_trace_fp = std::fopen(a.substr(17).c_str(), "w");
            if (!g_pipe_trace_fp) {
                std::fprintf(stderr, "[fpga-rom] cannot open pipe_trace_path=%s\n",
                             a.substr(17).c_str());
                return 1;
            }
        }
        else if (a.rfind("+audio_dump=", 0) == 0)
            audio_dump_path = a.substr(12);
        else if (a.rfind("+audio_dump_max=", 0) == 0)
            audio_dump_max = std::stoull(a.substr(16));
        else if (a.rfind("+asc_event_dump=", 0) == 0)
            asc_event_dump_path = a.substr(16);
        else if (a.rfind("+asc_event_dump_max=", 0) == 0)
            asc_event_dump_max = std::stoull(a.substr(20));
        else if (a.rfind("+state_replay=", 0) == 0)
            state_replay_path = a.substr(14);
        // Full-design periodic checkpointing (Part 71).  See the
        // checkpoint_save()/checkpoint_load() comment block above.
        else if (a.rfind("+checkpoint_dir=", 0) == 0)
            checkpoint_dir = a.substr(16);
        else if (a.rfind("+checkpoint_interval_cycles=", 0) == 0)
            checkpoint_interval_cycles = std::stoull(a.substr(28));
        else if (a.rfind("+load_checkpoint=", 0) == 0)
            load_checkpoint_base = a.substr(17);
#if VM_TRACE_FST
        else if (a.rfind("+wave_path=", 0) == 0)
            g_wave_path = a.substr(11);
        else if (a.rfind("+wave_start_retired=", 0) == 0)
            g_wave_start_retired = std::stoull(a.substr(20));
        else if (a.rfind("+wave_end_retired=", 0) == 0)
            g_wave_end_retired = std::stoull(a.substr(18));
#endif
        else if (a.rfind("+vbl_log_max=", 0) == 0)
            g_vbl_log_max = std::stoull(a.substr(13));
        else if (a.rfind("+watch_pa=", 0) == 0) {
            uint32_t v;
            if (!parse_u32(a.substr(10), v)) {
                std::fprintf(stderr, "bad +watch_pa value: %s\n",
                             a.c_str() + 10);
                return 1;
            }
            watch_pas.push_back(v & ~3u);  // 4-byte align
        }
        else if (a.rfind("+arch_dump_at_pc=", 0) == 0) {
            uint32_t v;
            if (!parse_u32(a.substr(17), v)) {
                std::fprintf(stderr, "bad +arch_dump_at_pc value: %s\n",
                             a.c_str() + 17);
                return 1;
            }
            arch_dump_pcs.push_back(v);
        }
        else if (a.rfind("+arch_dump_max=", 0) == 0)
            arch_dump_max = std::stoull(a.substr(15));
        else if (a.rfind("+dump_word_at=", 0) == 0) {
            uint32_t v;
            if (!parse_u32(a.substr(14), v)) {
                std::fprintf(stderr, "bad +dump_word_at value: %s\n",
                             a.c_str() + 14);
                return 1;
            }
            dump_word_at_addrs.push_back(v);
        }
        else if (a.rfind("+poke_word_at_pc=", 0) == 0) {
            const std::string spec = a.substr(17);
            const size_t c1 = spec.find(':');
            const size_t c2 = (c1 == std::string::npos) ? std::string::npos
                                                          : spec.find(':', c1 + 1);
            uint32_t pc = 0, addr = 0;
            if (c1 == std::string::npos || c2 == std::string::npos ||
                !parse_u32(spec.substr(0, c1), pc) ||
                !parse_u32(spec.substr(c1 + 1, c2 - c1 - 1), addr)) {
                std::fprintf(stderr, "bad +poke_word_at_pc spec: %s\n",
                             spec.c_str());
                return 1;
            }
            const uint16_t value = static_cast<uint16_t>(
                std::stoul(spec.substr(c2 + 1), nullptr, 16));
            poke_word_at_pc.push_back({pc, addr, value});
        }
        else if (a.rfind("+expect_q700_feature=", 0) == 0) {
            if (!parse_u32(a.substr(21), expected_q700_feature)) {
                std::fprintf(stderr, "bad +expect_q700_feature value: %s\n",
                             a.c_str() + 21);
                return 1;
            }
            expect_q700_feature = true;
        }
        else if (a.rfind("+scc_rx_inject=", 0) == 0) {
            scc_rx_inject = true;
            scc_rx_inject_str = a.substr(15);
        }
        else if (a == "+scc_rx_inject_fast" ||
                 a == "+scc_rx_inject_fast=1") {
            scc_rx_inject_fast = true;
        }
        else if (a.rfind("+scc_rx_inject_at=", 0) == 0) {
            if (!parse_u32(a.substr(18), scc_rx_inject_at)) {
                std::fprintf(stderr, "bad +scc_rx_inject_at value: %s\n",
                             a.c_str() + 18);
                return 1;
            }
            scc_rx_inject_at_set = true;
        }
        else if (a.rfind("+scc_tx_log=", 0) == 0) {
            scc_tx_log_path = a.substr(12);
        }
        else if (a.rfind("+scc_tx_uart_log=", 0) == 0) {
            scc_tx_uart_log_path = a.substr(17);
        }
        else if (a.rfind("+scc_tx_max=", 0) == 0) {
            scc_tx_max = std::stoull(a.substr(12));
        }
        else if (a.rfind("+nmi_at_inst=", 0) == 0) {
            nmi_at_inst = std::stoull(a.substr(13));
            nmi_at_inst_set = true;
        }
        else if (a.rfind("+nmi_at_pc=", 0) == 0) {
            if (!parse_u32(a.substr(11), nmi_at_pc)) {
                std::fprintf(stderr, "bad +nmi_at_pc value: %s\n",
                             a.c_str() + 11);
                return 1;
            }
            nmi_at_pc_set = true;
        }
        else if (a.rfind("+nmi_pulse_cycles=", 0) == 0) {
            nmi_pulse_cycles = std::stoull(a.substr(18));
            if (nmi_pulse_cycles == 0) nmi_pulse_cycles = 1;
        }
        else if (a.rfind("+irq_at_inst=", 0) == 0) {
            const std::string spec = a.substr(13);
            const size_t colon = spec.find(':');
            if (colon == std::string::npos) {
                std::fprintf(stderr, "bad +irq_at_inst spec: %s\n",
                             spec.c_str());
                return 1;
            }
            irq_at_inst = std::stoull(spec.substr(0, colon));
            const std::string rest = spec.substr(colon + 1);
            const size_t colon2 = rest.find(':');
            const std::string lvl_s = rest.substr(0, colon2);
            unsigned lvl = std::stoul(lvl_s, nullptr, 0);
            if (lvl < 1 || lvl > 7) {
                std::fprintf(stderr, "bad +irq_at_inst level: %u\n", lvl);
                return 1;
            }
            if (colon2 != std::string::npos) {
                const std::string rest2 = rest.substr(colon2 + 1);
                const size_t colon3 = rest2.find(':');
                irq_inject_total = std::stoull(rest2.substr(0, colon3));
                if (colon3 != std::string::npos)
                    irq_inject_gap = std::stoull(rest2.substr(colon3 + 1));
                if (irq_inject_total == 0) irq_inject_total = 1;
                if (irq_inject_gap == 0) irq_inject_gap = 1;
            }
            irq_inject_level = (uint8_t)lvl;
            irq_at_inst_set = true;
        }
        else if (a.rfind("+irq_at_pc=", 0) == 0) {
            const std::string spec = a.substr(11);
            const size_t colon = spec.find(':');
            if (colon == std::string::npos ||
                !parse_u32(spec.substr(0, colon), irq_at_pc)) {
                std::fprintf(stderr, "bad +irq_at_pc spec: %s\n",
                             spec.c_str());
                return 1;
            }
            const std::string rest = spec.substr(colon + 1);
            const size_t colon2 = rest.find(':');
            const std::string lvl_s = rest.substr(0, colon2);
            unsigned lvl = std::stoul(lvl_s, nullptr, 0);
            if (lvl < 1 || lvl > 7) {
                std::fprintf(stderr, "bad +irq_at_pc level: %u\n", lvl);
                return 1;
            }
            if (colon2 != std::string::npos) {
                const std::string rest2 = rest.substr(colon2 + 1);
                const size_t colon3 = rest2.find(':');
                irq_inject_total = std::stoull(rest2.substr(0, colon3));
                if (colon3 != std::string::npos)
                    irq_inject_gap = std::stoull(rest2.substr(colon3 + 1));
                if (irq_inject_total == 0) irq_inject_total = 1;
                if (irq_inject_gap == 0) irq_inject_gap = 1;
            }
            irq_inject_level = (uint8_t)lvl;
            irq_at_pc_set = true;
        }
        else if (a.rfind("+irq_at_cycle=", 0) == 0) {
            const std::string spec = a.substr(14);
            const size_t colon = spec.find(':');
            if (colon == std::string::npos) {
                std::fprintf(stderr, "bad +irq_at_cycle spec: %s\n",
                             spec.c_str());
                return 1;
            }
            g_irqc_at = std::stoull(spec.substr(0, colon));
            const std::string rest = spec.substr(colon + 1);
            const size_t colon2 = rest.find(':');
            const std::string lvl_s = rest.substr(0, colon2);
            unsigned lvl = std::stoul(lvl_s, nullptr, 0);
            if (lvl < 1 || lvl > 7) {
                std::fprintf(stderr, "bad +irq_at_cycle level: %u\n", lvl);
                return 1;
            }
            if (colon2 != std::string::npos) {
                const std::string rest2 = rest.substr(colon2 + 1);
                const size_t colon3 = rest2.find(':');
                g_irqc_total = std::stoull(rest2.substr(0, colon3));
                if (colon3 != std::string::npos)
                    g_irqc_gap = std::stoull(rest2.substr(colon3 + 1));
                if (g_irqc_total == 0) g_irqc_total = 1;
                if (g_irqc_gap == 0) g_irqc_gap = 1;
            }
            g_irqc_lvl = (uint8_t)lvl;
            g_irqc_set = true;
        }
        else if (a.rfind("+axi_lockstep_log=", 0) == 0) {
            axi_log_path = a.substr(18);
        }
        else if (a.rfind("+axi_lockstep_max=", 0) == 0) {
            axi_log_max = std::stoull(a.substr(18));
        }
        else if (a.rfind("+via1_lockstep_log=", 0) == 0) {
            via1_log_path = a.substr(19);
        }
        else if (a.rfind("+via1_lockstep_max=", 0) == 0) {
            via1_log_max = std::stoull(a.substr(19));
        }
        else if (a.rfind("+dafb_lockstep_log=", 0) == 0) {
            dafb_log_path = a.substr(19);
        }
        else if (a.rfind("+dafb_lockstep_max=", 0) == 0) {
            dafb_log_max = std::stoull(a.substr(19));
        }
        else if (a == "+dafb_lockstep_include_vram" ||
                 a == "+dafb_lockstep_include_vram=1") {
            dafb_log_include_vram = true;
        }
    }

    // Decode the inject string up-front so a syntax error fails fast before
    // we spend several minutes reaching MacsBug.
    std::vector<uint8_t> scc_rx_bytes;
    if (scc_rx_inject) {
        if (!decode_byte_string(scc_rx_inject_str, scc_rx_bytes)) {
            std::fprintf(stderr,
                "[fpga-rom] bad +scc_rx_inject= byte string '%s' "
                "(supports \\r \\n \\t \\0 \\\\ \\xHH)\n",
                scc_rx_inject_str.c_str());
            return 1;
        }
        std::fprintf(stderr,
            "[fpga-rom] scc_rx_inject queued %zu byte(s):", scc_rx_bytes.size());
        for (uint8_t b : scc_rx_bytes)
            std::fprintf(stderr, " 0x%02x", b);
        std::fprintf(stderr, " (mode=%s)\n",
                     scc_rx_inject_fast ? "byte-fast" : "uart-bit-serial");
    }

    FILE* scc_tx_fp = nullptr;
    FILE* scc_tx_uart_fp = nullptr;
    if (!scc_tx_log_path.empty()) {
        scc_tx_fp = std::fopen(scc_tx_log_path.c_str(), "w");
        if (!scc_tx_fp) {
            std::fprintf(stderr, "[fpga-rom] cannot open scc_tx_log=%s\n",
                         scc_tx_log_path.c_str());
            return 1;
        }
    }
    if (!scc_tx_uart_log_path.empty()) {
        scc_tx_uart_fp = std::fopen(scc_tx_uart_log_path.c_str(), "w");
        if (!scc_tx_uart_fp) {
            std::fprintf(stderr,
                         "[fpga-rom] cannot open scc_tx_uart_log=%s\n",
                         scc_tx_uart_log_path.c_str());
            return 1;
        }
    }
    // PC trace dump: write per-changed-boundary_pc lines to a file.  Used to
    // bisect "where did the CPU jump to a wild PC?" — emits up to N entries
    // (default 10000) of `<retired> <sim_time> <pc>` text.  Stop sampling
    // when the limit is hit; the rest of the run continues normally.
    FILE* pc_dump_fp = nullptr;
    uint64_t pc_dump_count = 0;
    uint32_t pc_dump_prev = 0;
    uint64_t pc_dump_next_at = 0;
    // Dual-commit detection: dbg_boundary_seq tracks last_uop retirements
    // (single-commit cycle: +1, dual-commit cycle with both lanes last_uop:
    // +2).  When delta==2, dump head+0's PC (dbg_boundary_a_pc) before
    // head+1's (dbg_boundary_pc) so the trace records both retired macros.
    // Without this, line 1460's NBA in commit.v would silently suppress
    // head+0 from the trace (overrides dbg_boundary_pc with lb_pc).
    uint32_t pc_dump_seq_prev = 0;
    if (!pc_dump_path.empty()) {
        pc_dump_fp = std::fopen(pc_dump_path.c_str(), "w");
        if (!pc_dump_fp) {
            std::fprintf(stderr, "[fpga-rom] cannot open pc_dump_path=%s\n",
                         pc_dump_path.c_str());
            return 1;
        }
#ifdef CPU_M68K040
        // See the g_pc_dump_fp declaration comment: CPU_M68K040 drives the
        // dump from tick()'s traceVec_0/1-based tap, not the
        // debugMacroRetirePc_0/1 path below (which silently stops firing
        // after roughly the first 20 retirements on this core).
        g_pc_dump_fp = pc_dump_fp;
        g_pc_dump_max = pc_dump_max;
#endif
    }

    // Wire +audio_dump / +asc_event_dump into the globals tick() reads.
    if (!audio_dump_path.empty()) {
        audio_dump_fp = std::fopen(audio_dump_path.c_str(), "w");
        if (!audio_dump_fp) {
            std::fprintf(stderr, "[fpga-rom] cannot open audio_dump=%s\n",
                         audio_dump_path.c_str());
            return 1;
        }
        std::fprintf(audio_dump_fp,
                     "# retired\tsim_time\tpcm_l\tpcm_r\n");
        g_audio_dump_fp  = audio_dump_fp;
        g_audio_dump_max = audio_dump_max;
    }
    if (!asc_event_dump_path.empty()) {
        asc_event_dump_fp = std::fopen(asc_event_dump_path.c_str(), "w");
        if (!asc_event_dump_fp) {
            std::fprintf(stderr,
                         "[fpga-rom] cannot open asc_event_dump=%s\n",
                         asc_event_dump_path.c_str());
            return 1;
        }
        std::fprintf(asc_event_dump_fp,
                     "# retired\tsim_time\tkind\taddr\tdata\n");
        g_asc_event_dump_fp  = asc_event_dump_fp;
        g_asc_event_dump_max = asc_event_dump_max;
    }

    // AXI lockstep capture state.  One CSV line per CPU-side AXI
    // transaction (outside DDR / ROM) — paired with MAME's
    // tools/mame_axi_capture.lua output for byte-diff in
    // tools/axi_lockstep_diff.py.  See docs/axi_lockstep.md.
    FILE* axi_log_fp = nullptr;
    uint64_t axi_log_seq = 0;
    if (!axi_log_path.empty()) {
        axi_log_fp = std::fopen(axi_log_path.c_str(), "w");
        if (!axi_log_fp) {
            std::fprintf(stderr, "[fpga-rom] cannot open axi_lockstep_log=%s\n",
                         axi_log_path.c_str());
            return 1;
        }
        std::fprintf(axi_log_fp,
            "# tb_fpga_top_rom AXI lockstep capture v1 — seq,rw,addr,size,data\n");
        std::fprintf(axi_log_fp, "# limit=%llu\n",
                     (unsigned long long)axi_log_max);
        std::fflush(axi_log_fp);
    }

    // VIA1 lockstep capture state.  One CSV line per peripheral_bus VIA1
    // master-face access (pb_via1_rd / pb_via1_wr pulses), to byte-diff
    // against MAME's tools/mame_via1_capture.lua output.  See
    // docs/via1_lockstep.md.
    FILE* via1_log_fp = nullptr;
    uint64_t via1_log_seq = 0;
    if (!via1_log_path.empty()) {
        via1_log_fp = std::fopen(via1_log_path.c_str(), "w");
        if (!via1_log_fp) {
            std::fprintf(stderr, "[fpga-rom] cannot open via1_lockstep_log=%s\n",
                         via1_log_path.c_str());
            return 1;
        }
        std::fprintf(via1_log_fp,
            "# tb_fpga_top_rom VIA1 lockstep capture v1 — seq,rw,reg,byte\n");
        std::fprintf(via1_log_fp, "# limit=%llu\n",
                     (unsigned long long)via1_log_max);
        std::fflush(via1_log_fp);
    }

    // Filter: drop DDR (0..0x3FFFFFFF) and ROM (0x40000000..0x40FFFFFF).
    // Keep peripheral aperture (0x50000000..0x5FFFFFFF), VRAM
    // (0xF9000000..0xF91FFFFF), DAFB (0xF9800000..0xF98003FF), and any
    // out-of-decode probe addresses above 0x40FFFFFF.  Mirrors the
    // filter in tools/mame_axi_capture.lua.
    auto axi_in_scope = [](uint32_t addr) -> bool {
        if (addr < 0x40000000u) return false;
        if (addr < 0x50000000u) return false;
        return true;
    };

    // DAFB lockstep capture state.  CSV format identical to AXI lockstep
    // (seq,rw,addr,size,data) so the same diff tooling works; the
    // address filter is narrower — DAFB register window + TurboSCSI
    // window + (optional) VRAM aperture.  The TurboSCSI mirror bits
    // (0x00FC_0000) get stripped by the diff tool, not here, so the
    // RTL trace records what the bus actually saw.
    FILE* dafb_log_fp = nullptr;
    uint64_t dafb_log_seq = 0;
    if (!dafb_log_path.empty()) {
        dafb_log_fp = std::fopen(dafb_log_path.c_str(), "w");
        if (!dafb_log_fp) {
            std::fprintf(stderr, "[fpga-rom] cannot open dafb_lockstep_log=%s\n",
                         dafb_log_path.c_str());
            return 1;
        }
        std::fprintf(dafb_log_fp,
            "# tb_fpga_top_rom DAFB lockstep capture v1 — seq,rw,addr,size,data\n");
        std::fprintf(dafb_log_fp, "# limit=%llu include_vram=%d\n",
                     (unsigned long long)dafb_log_max,
                     (int)dafb_log_include_vram);
        std::fflush(dafb_log_fp);
    }
    auto dafb_in_scope = [&](uint32_t addr) -> bool {
        // DAFB register aperture.
        if (addr >= 0xF9800000u && addr <= 0xF98003FFu) return true;
        // VRAM aperture (off by default).
        if (dafb_log_include_vram &&
            addr >= 0xF9000000u && addr <= 0xF91FFFFFu) return true;
        // TurboSCSI register window + DMA shim, with 0x00FC_0000 mirror.
        // Strip the mirror bits before checking, mirroring what the diff
        // tool does on the MAME side.
        if (addr >= 0x50000000u && addr <= 0x50FFFFFFu) {
            uint32_t sub = addr & 0x000FFFFFu;
            if (sub >= 0x0F000u && sub <= 0x0F0FFu) return true;
            if (sub >= 0x0F100u && sub <= 0x0F101u) return true;
        }
        return false;
    };

    // Decode a narrow-side 4-bit wstrb into (architectural CPU byte addr,
    // size, value) for emission.  m68k is BIG-ENDIAN: the CPU's
    // architectural byte address A maps to AXI lane (3 - (A & 3)) — so
    // wstrb 0x8 (lane 3) corresponds to CPU byte_off 0, wstrb 0x1 (lane 0)
    // to CPU byte_off 3.  This is the inverse of the little-endian
    // mapping you'd expect from the AXI spec.
    //
    // Lane → architectural byte_off:
    //   wstrb 0x8 → byte_off 0    (CPU writes to (base & ~3) + 0)
    //   wstrb 0x4 → byte_off 1
    //   wstrb 0x2 → byte_off 2
    //   wstrb 0x1 → byte_off 3
    //
    // For half-word (size=2) the two contiguous wstrb patterns are:
    //   wstrb 0xC → byte_off 0 (lanes 3+2 = high half of AXI word = CPU
    //                            bytes 0,1 = aligned half-word at base & ~3)
    //   wstrb 0x3 → byte_off 2 (lanes 1+0 = low half of AXI word = CPU
    //                            bytes 2,3 = aligned half-word at base+2)
    // For size=4: wstrb 0xF (all lanes) at base.
    //
    // Sparse / unexpected patterns are emitted as size=4 at the base
    // address with full wdata, so a divergence still surfaces.
    auto decode_narrow_w = [](uint32_t base_addr,
                              uint32_t wdata,
                              uint8_t wstrb,
                              uint32_t& out_addr,
                              uint32_t& out_size,
                              uint32_t& out_data) -> void {
        unsigned size = 0;
        unsigned byte_off = 0;       // architectural CPU byte offset
        unsigned axi_lane_lsb = 0;   // bit position in wdata
        switch (wstrb & 0xF) {
        case 0x8: size = 1; byte_off = 0; axi_lane_lsb = 24; break;
        case 0x4: size = 1; byte_off = 1; axi_lane_lsb = 16; break;
        case 0x2: size = 1; byte_off = 2; axi_lane_lsb =  8; break;
        case 0x1: size = 1; byte_off = 3; axi_lane_lsb =  0; break;
        case 0xC: size = 2; byte_off = 0; axi_lane_lsb = 16; break;
        case 0x3: size = 2; byte_off = 2; axi_lane_lsb =  0; break;
        case 0xF: size = 4; byte_off = 0; axi_lane_lsb =  0; break;
        default:  size = 4; byte_off = 0; axi_lane_lsb =  0; break;
        }
        out_addr = (base_addr & ~0x3u) + byte_off;
        out_size = size;
        if (size == 1) out_data = (wdata >> axi_lane_lsb) & 0xFFu;
        else if (size == 2) out_data = (wdata >> axi_lane_lsb) & 0xFFFFu;
        else out_data = wdata;
    };

    // M0 (CPU LSU narrow data) AR/AW capture state.  We snapshot at the
    // narrow side (core_daxi_*) — byte-granular addresses, simple wstrb,
    // matches MAME's CPU-visible semantics.  Up to one outstanding read
    // and one outstanding write at this layer.
    bool m0_ar_pending = false;
    uint32_t m0_ar_addr = 0;
    uint32_t m0_ar_size = 0;     // arsize as logged at AR handshake (0/1/2)
    bool m0_aw_pending = false;
    uint32_t m0_aw_addr = 0;
    bool m0_aw_w_done  = false;
    uint32_t m0_aw_w_addr = 0;
    uint32_t m0_aw_w_size = 0;
    uint32_t m0_aw_w_data = 0;
    // M3 (CPU instruction fetch wide) AR capture state.  IF accesses are
    // wide-side directly (no narrow adapter); record at the lane width.
    bool m3_ar_pending = false;
    uint32_t m3_ar_addr = 0;

    auto axi_emit = [&](const char* rw, uint32_t addr,
                        uint32_t size, uint32_t data) {
        const char* fmt = (size == 1) ? "%llu,%s,%08x,%u,%02x\n"
                        : (size == 2) ? "%llu,%s,%08x,%u,%04x\n"
                                      : "%llu,%s,%08x,%u,%08x\n";
        if (axi_log_fp && axi_log_seq < axi_log_max && axi_in_scope(addr)) {
            std::fprintf(axi_log_fp, fmt,
                         (unsigned long long)axi_log_seq, rw,
                         (unsigned)addr, (unsigned)size, (unsigned)data);
            axi_log_seq++;
            if (axi_log_seq >= axi_log_max) {
                std::fflush(axi_log_fp);
                std::fprintf(stderr,
                    "[axi-lockstep] hit limit=%llu, capture closed\n",
                    (unsigned long long)axi_log_max);
            }
        }
        if (dafb_log_fp && dafb_log_seq < dafb_log_max && dafb_in_scope(addr)) {
            std::fprintf(dafb_log_fp, fmt,
                         (unsigned long long)dafb_log_seq, rw,
                         (unsigned)addr, (unsigned)size, (unsigned)data);
            dafb_log_seq++;
            if (dafb_log_seq >= dafb_log_max) {
                std::fflush(dafb_log_fp);
                std::fprintf(stderr,
                    "[dafb-lockstep] hit limit=%llu, capture closed\n",
                    (unsigned long long)dafb_log_max);
            }
        }
    };

    // VIA1 lockstep emitter — same shape as axi_emit but for the VIA1
    // master face on peripheral_bus.  Captures (rw, reg, byte) per access.
    // 5th CSV column = last-retired macro PC at the time of the access.
    // tools/via1_lockstep_diff.py only reads columns 0..3, so the PC is
    // an informational annotation — it does not affect the diff but lets
    // a control-flow fork be attributed to a ROM PC at a glance.
    uint32_t via1_last_pc = 0;
    auto via1_emit = [&](const char* rw, uint32_t reg, uint32_t byte) {
        if (!via1_log_fp) return;
        if (via1_log_seq >= via1_log_max) return;
        std::fprintf(via1_log_fp, "%llu,%s,%x,%02x,%08x\n",
                     (unsigned long long)via1_log_seq, rw,
                     (unsigned)(reg & 0xFu), (unsigned)(byte & 0xFFu),
                     (unsigned)via1_last_pc);
        via1_log_seq++;
        if (via1_log_seq >= via1_log_max) {
            std::fflush(via1_log_fp);
            std::fprintf(stderr,
                "[via1-lockstep] hit limit=%llu, capture closed\n",
                (unsigned long long)via1_log_max);
        }
    };

    // VIA1 read-pending state: pb_rd pulses one cycle, pb_ack/pb_rdata
    // are registered (visible the cycle after).  Latch the register on
    // the pulse, emit when ack arrives.
    bool     via1_rd_pending = false;
    uint32_t via1_rd_reg = 0;
    uint8_t  prev_via1_rd = 0;
    uint8_t  prev_via1_wr = 0;

    if (!sd_image_path.empty() && !sd_card_image_path.empty()) {
        std::fprintf(stderr,
            "error: +sd_image and +sd_card_image are mutually exclusive\n");
        return 1;
    }
    if (!sd_image_path.empty() || !sd_card_image_path.empty()) {
        g_sd.trace = sd_trace;
        if (sd_trace) g_sdreq_log_max = 400;
        g_sd.writeback = sd_writeback;
        const bool ok = sd_card_image_path.empty()
            ? g_sd.attach_hdd(sd_image_path)
            : g_sd.attach_raw(sd_card_image_path);
        if (!ok) return 1;
        g_sd_enabled = true;
    }

    // Full-design checkpoint resume (Part 71): skip the ENTIRE cold-boot
    // reset/preload/1000-cycle-settle sequence below when +load_checkpoint=
    // is given -- checkpoint_load() constructs `dut` itself and restores
    // its complete state (CPU+caches+L2C+AXI fabric+every peripheral+DRAM),
    // which already reflects a design mid-run past all of that.
    const bool ramtest_fastfill =
        patch_sets_need_ramtest_fastfill(rom_patch_set);
    if (load_checkpoint_base.empty()) {
    dut = new Vfpga_top;
    dut->cpu_resetn = 0;
    // btn[3:0] are active-low on the board (released = HIGH, pressed = GND).
    // 0xF = all released — anything else holds platform_resetn low and pins
    // the SoC in reset forever.  See commit 4a5968d.
    dut->btn = 0xF;
    dut->uart_rtl_0_rxd = 1;
    dut->sd_miso = 1;
    dut->al9134_int = 0;
    for (int i = 0; i < 64; i++) tick();
    // Sim-only: force the btn[1] (NMI) and btn[2] (debug-full-reset)
    // debounce filters to "released" (out_n=1) so the SoC does not storm a
    // level-7 NMI on the very first instruction.
    //
    // CORRECTION (2026-07-17): this comment used to claim the underlying
    // bug "does NOT happen on real HW" — that claim was WRONG and has
    // been disproven by a live-HW investigation (release/no-vipt,
    // build_id 0x8b18323b): the board took exactly one spurious level-7
    // (NMI) exception on 100% of observed cold boots, landing the CPU in
    // a dead ROM SCC-poll loop it never returns from
    // (pc=0x40847abe -> drifts to ~0x4084a840, exc_count stuck at 1).
    // The root cause IS the mechanism described below; the "(a)/(b)"
    // timing-cushion argument that follows does not reliably hold on
    // real HW (bitstream configuration and the debounce module's own
    // free-running clock start well before whatever later releases
    // cpu_resetn/soc_full_rst, but that gap is not actually guaranteed
    // to exceed the 2M-cycle/10ms debounce window on every boot path —
    // e.g. JTAG SRAM-loaded bitstreams, as opposed to a power-on with an
    // external supervisor holding cpu_resetn low for a guaranteed
    // multi-ms window).
    //
    // THE REAL FIX now lives at the RTL source: reset_debounce.v gained
    // an `IDLE_OUT_N` parameter (default 0, preserving today's
    // cpu_resetn/btn[3] "hold reset at power-up" behaviour), and
    // fpga_top_clocks.vh's u_btn1_db/u_btn2_db instances now pass
    // IDLE_OUT_N=1 so `out_n` — and the sync chain feeding it — power up
    // already agreeing with "not pressed", instead of racing a 10 ms
    // debounce window before disagreeing with reality.  See
    // reset_debounce.v's header comment and
    // tb/tb_reset_debounce.cpp's "Scenario 4" (built via
    // `make tb-reset-debounce-idle-high`) for the unit-level regression
    // proof.
    //
    // The poke below is now REDUNDANT with that RTL fix (out_n already
    // reads 1 from cycle 0) but is kept as a harmless no-op / extra
    // safety net — it saves a little sim time regardless by skipping
    // past the 3-FF sync-chain settle, and protects this harness if a
    // future change ever regresses the RTL-level fix.
    //
    // Original mechanism notes (still accurate as a description of the
    // bug, not as a "sim-only" claim): reset_debounce sits in sys_clk
    // and is NOT held by any reset.  Pre-fix, its `initial` block set
    // out_n=0 ("button pressed" — cpu_resetn pin semantics) regardless
    // of the caller's true idle polarity, and it requires
    // NMI_DEBOUNCE_CYCLES (2_000_000) consecutive raw_in_n=1 samples to
    // flip out_n to 1 (= "released").  In sim we tick far fewer than 2M
    // cycles before releasing cpu_rst -> btn1's out_n is still 0 ->
    // btn1_sync = ~0 = 1 -> nmi_btn_pulse = 1 -> irq_agg sees a rising
    // edge of nmi_edge on the first cycle out of soc_full_rst, latches
    // nmi_pending=1 forever, and the CPU takes a level-7 autovector
    // exception (vec=31) on every macro before it can retire (PC sticks
    // at the handler-fetched 0x000000ac, dbg_committed never advances).
    //
    // The two cpu/btn3 *reset* debounces (RST_DEBOUNCE_CYCLES) already
    // have a SIM_MODEL gate (8 cycles) so they're fine; the NMI
    // debounces don't need one anymore now that IDLE_OUT_N fixes the
    // real bug at the source.
    //
    // We poke AFTER the first 64 ticks so the module's `initial` block has
    // already run and our pokes overwrite the post-init `out_n=0` state.
    // raw_in_n=btn[N]=1 ensures the debounce stays latched at out_n=1
    // forever (sync_qq==out_n on every cycle, counter never advances).
    {
        auto* r = dut->rootp;
        r->fpga_top__DOT__u_btn1_db__DOT__out_n     = 1;
        r->fpga_top__DOT__u_btn1_db__DOT__sync_meta = 1;
        r->fpga_top__DOT__u_btn1_db__DOT__sync_q    = 1;
        r->fpga_top__DOT__u_btn1_db__DOT__sync_qq   = 1;
        r->fpga_top__DOT__u_btn1_db__DOT__cnt       = 0;
        r->fpga_top__DOT__u_btn2_db__DOT__out_n     = 1;
        r->fpga_top__DOT__u_btn2_db__DOT__sync_meta = 1;
        r->fpga_top__DOT__u_btn2_db__DOT__sync_q    = 1;
        r->fpga_top__DOT__u_btn2_db__DOT__sync_qq   = 1;
        r->fpga_top__DOT__u_btn2_db__DOT__cnt       = 0;
    }
    StateReplay state_snap;
    if (!state_replay_path.empty()) {
        if (!load_state_replay(state_replay_path, state_snap)) return 1;
    }
    if (no_preload) {
        std::fprintf(stderr,
            "[fpga-rom] +no_preload: skipping the DDR-backdoor ROM "
            "preload -- relying entirely on boot_fsm's real "
            "SD-streaming + mirror-to-0x0 path for ROM content.\n");
    } else if (!preload_rom(rom_path, rom_patch_set,
                     state_snap.present, state_snap.pc,
                     state_snap.sp)) return 1;
    if (state_snap.present) apply_state_dram(state_snap);
    dut->cpu_resetn = 1;
    // Diagnostic: tick a few hundred cycles + dump reset gate state every 100c
    // so we can see what's holding the CPU in reset if `retired` stays 0.
    bool state_arch_applied = !state_snap.present;
    for (int i = 0; i < 1000; i++) {
        tick();
        // Poke arch state on the FIRST cycle cpu_rst deasserts so PRF
        // values land before any retire updates them.  The patched
        // JMP at ROM offset 0x2A doesn't touch any arch reg, so the
        // poked values stay live until the snapshot PC starts running.
        if (!state_arch_applied &&
            !dut->rootp->fpga_top__DOT__cpu_rst) {
#ifdef CPU_M68K040
            if (!apply_state_arch_cpu040(state_snap)) return 1;
#else
            apply_state_arch(state_snap);
#endif
            state_arch_applied = true;
            if (dafb_scsi_ctrl >= 0) {
                uint32_t& reg =
                    dut->rootp->fpga_top__DOT__u_dafb__DOT__regs[9];
                reg = (reg & ~0x1FFu) | ((uint32_t)dafb_scsi_ctrl & 0x1FFu);
                std::fprintf(stderr,
                    "[scsi-seed] dafb regs[REG_FIRST_HIT][8:0] = 0x%03x\n",
                    (unsigned)(dafb_scsi_ctrl & 0x1FF));
            }
            if (scsi_config3 >= 0) {
                dut->rootp->fpga_top__DOT__u_scsi__DOT__c96_config3 =
                    (uint8_t)scsi_config3;
                std::fprintf(stderr,
                    "[scsi-seed] c96_config3 = 0x%02x\n",
                    (unsigned)(scsi_config3 & 0xFF));
            }
        }
        if ((i % 100) == 0) {
            std::fprintf(stderr,
                "[reset-dbg] i=%d cpu_resetn=1 ddr_cal_done=%u boot_rom_loaded=%u "
                "boot_rom_ready=%u soc_full_rst=%u cpu_rst=%u "
                "cpu_rst_settle_done=%u dbg_committed=%u\n",
                i,
                (unsigned)dut->rootp->fpga_top__DOT__ddr_cal_done,
                (unsigned)dut->rootp->fpga_top__DOT__boot_rom_loaded,
                (unsigned)dut->rootp->fpga_top__DOT__boot_rom_ready,
                (unsigned)dut->rootp->fpga_top__DOT__soc_full_rst,
                (unsigned)dut->rootp->fpga_top__DOT__cpu_rst,
                (unsigned)dut->rootp->fpga_top__DOT__cpu_rst_settle_done,
                (unsigned)DBGT_REF(dut->rootp, committed));
        }
    }
    } else {
        // ── Resume from a full-design checkpoint (Part 71) ──
        if (!checkpoint_load(load_checkpoint_base)) return 1;
        std::fprintf(stderr,
            "[fpga-rom] resumed from checkpoint %s -- skipping cold-boot "
            "reset/preload entirely, entering the main loop directly.\n",
            load_checkpoint_base.c_str());
    }
    for (uint32_t addr : dump_word_at_addrs) {
        // RAM-only decode (addr_to_idx()'s non-ROM-window branch,
        // rtl/board/ddr_ctrl.v): idx = a[25:4], byte lane = a[3:0].
        // read_beat() packs {mem_b15,...,mem_b0} MSB-first, i.e. lane N =
        // bits [8N+7:8N] of the 128-bit beat -- but this SoC's AXI byte
        // addressing runs the opposite direction (byte offset 0 within a
        // beat is the HIGH lane, matching 68k big-endian expectations at
        // the CPU), empirically confirmed against the known reset-vector
        // bytes at 0x0 (2026-09-03): lane = 15 - (addr & 0xF), not
        // addr & 0xF.
        // Mirror ddr_ctrl.v's addr_to_idx() exactly (FPGA_ROM_SIM branch):
        // ROM window (masking bit 23, the 0x40800000/0x40000000 alias) ->
        // SIM_RAM_BEATS + a[22:4]; everything else -> a[25:4].
        auto addr_to_idx = [](uint32_t a) -> uint32_t {
            const uint32_t SIM_RAM_BEATS = 0x04000000u / 16u;  // 0x400000
            const uint32_t masked = a & ~0x00800000u;
            if (masked >= 0x40000000u && masked < 0x40400000u)
                return SIM_RAM_BEATS + ((a >> 4) & 0x7FFFFu);  // a[22:4]
            return (a >> 4) & 0x3FFFFFu;                        // a[25:4]
        };
        const uint32_t idx = addr_to_idx(addr);
        const uint32_t idx1 = addr_to_idx(addr + 1);
        // Empirically confirmed against known ROM bytes at 0x40800000
        // (2026-09-03): the byte lanes are swapped WITHIN each 32-bit
        // word (offset XOR 3), not reversed across the full 16-byte
        // beat -- a standard endian byte-swap at 32-bit granularity.
        const uint32_t lane0 = (addr & 0xFu) ^ 3u;
        const uint32_t lane1 = ((addr + 1) & 0xFu) ^ 3u;
        auto lane_byte = [&](uint32_t lane, uint32_t i) -> uint8_t {
            switch (lane) {
                case 0:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b0[i];
                case 1:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b1[i];
                case 2:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b2[i];
                case 3:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b3[i];
                case 4:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b4[i];
                case 5:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b5[i];
                case 6:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b6[i];
                case 7:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b7[i];
                case 8:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b8[i];
                case 9:  return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b9[i];
                case 10: return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b10[i];
                case 11: return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b11[i];
                case 12: return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b12[i];
                case 13: return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b13[i];
                case 14: return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b14[i];
                default: return dut->rootp->fpga_top__DOT__u_ddr__DOT__mem_b15[i];
        }};
        const uint8_t b0 = lane_byte(lane0, idx);
        const uint8_t b1 = lane_byte(lane1, idx1);
        const uint16_t word = ((uint16_t)b0 << 8) | b1;
        std::fprintf(stderr,
            "[dump_word_at] 0x%08x = 0x%04x (idx=%u lane0=%u b0=0x%02x b1=0x%02x)\n",
            addr, word, idx, lane0, b0, b1);
    }
    // 2026-09-03: the coordinator's cpuBootThrottleEn sanity-check peek
    // (was here) was removed along with the throttle plumbing itself
    // (SoC repo commit 58726ee) -- confirmed inert (en_w=0/window=0/
    // core_throttleAllow=1 bracketing round 1's real execution, Part 90)
    // before the plumbing was deleted, so nothing to re-check post-removal.

    std::vector<uint32_t> frame(FRAME_W * FRAME_H, 0);
    int x = 0, y = 0;
    uint8_t prev_vs = 0;
    uint64_t last_report = 0;
    uint64_t retired = 0;
    bool hit_floppy_loop = false;
    bool hit_monitor = false;
    bool hit_q700_feature = false;
    bool bad_q700_feature = false;
    uint32_t q700_feature = 0;
    uint32_t q700_entry = 0;
    uint32_t hit_pc = 0;

    // ── SCC RX/TX validation state ──────────────────────────────────────
    // The driver waits for MacsBug entry (any rom_scc_rx_* / MacsBug-range
    // PC, or the explicit override pc) and then either:
    //   (a) bit-bangs uart_rtl_0_rxd at PB_BAUD baud (default 115200) to
    //       traverse the full uart_byte_bridge → SCC FIFO → CPU read path, or
    //   (b) pulses the SCC's external rx_valid/rx_data byte interface
    //       directly via the public-flat-rw hook (skips the bridge serial
    //       path; ~10× faster).
    bool macsbug_seen = false;
    uint64_t macsbug_seen_time = 0;
    uint64_t scc_rx_inject_done_time = 0;
    bool scc_rx_inject_done = false;
    size_t scc_rx_byte_idx = 0;
    int scc_rx_bit_state = 0;        // 0=idle, 1..10=start/data/stop bits
    int scc_rx_bit_phase = 0;        // 0..BAUD_DIV countdown per bit
    // PB clock = 50 MHz (matches PB_CLK_HZ default in fpga_top.v).  pb_clk
    // ticks once per 4 sys_clk edges in SIM_MODEL.  One sys_clk = one tick()
    // = 2 sim_time units, so one pb_clk = 8 sim_time units.  At 115200 baud,
    // BAUD_DIV ≈ 50e6 / 115200 ≈ 434 pb_clk = 3472 sim_time per UART bit.
    // For sim economy default to a higher "sim baud" (1 Mbaud) when running
    // through the bit-serial path; the bridge is asynchronous so any
    // baud the bridge BAUD parameter can sample correctly is fine.
    // A real-HW capture should set the on-host serial port to 115200.
    const int SCC_PB_CLK_PER_TICK = 4;          // sys_clk → pb_clk ratio
    const int SCC_TICKS_PER_BIT = 50;           // ~1 Mbaud sim baud
                                                // (50 pb_clk per bit @ 50MHz)
    static_cast<void>(SCC_PB_CLK_PER_TICK);

    // CPU-side TX byte capture (scc_uart_tx_valid pulses one cycle per
    // completed SCC byte, on whichever channel scc_uart_sel routes — default
    // chan A on sim).
    uint64_t scc_tx_count = 0;
    uint64_t scc_tx_uart_count = 0;
    uint64_t scc_first_tx_time = 0;
    uint64_t scc_first_tx_uart_time = 0;
    uint8_t  scc_first_tx_byte = 0;
    uint8_t  scc_first_tx_uart_byte = 0;
    std::vector<uint8_t> scc_tx_bytes;
    std::vector<uint8_t> scc_tx_uart_bytes;

    // UART-pin TX byte sniffer: track uart_rtl_0_txd 1→0 starts, sample
    // mid-bit at the same baud the bridge transmits at (using BAUD that the
    // bridge instantiates with — 115200 over 50 MHz pb_clk).
    int  uart_tx_state = 0;     // 0=idle (line high), 1=in-frame
    int  uart_tx_bit_phase = 0; // sim_time countdown per bit
    int  uart_tx_bit_idx = 0;   // 0..9 (start, 8 data, stop)
    uint8_t uart_tx_shift = 0;
    uint8_t uart_rxd_pin = 1;

    // Helper: drive the inject byte line one tick.  Returns true once the
    // entire queue has been transmitted.
    auto step_rx_inject = [&]() -> bool {
        if (!scc_rx_inject) return true;
        if (scc_rx_byte_idx >= scc_rx_bytes.size()) return true;
        if (scc_rx_inject_fast) {
            // Bridge-rx_valid pulse path: assert rx_a_data + rx_a_valid
            // and let scc.v's own always-block do the FIFO push (so all
            // NBAs on rxf_a*/rxlvl_a/rx_ip_a happen in the right order,
            // no Vdly/poke fight).
            //
            // The SCC pushes one byte per pb_clk that rx_a_valid &&
            // wra[3][0] is high.  To push exactly one byte per call we
            // need a single-pb_clk-wide pulse — but our tick() is
            // sys_clk granularity (one tick = ¼ pb_clk in SIM_MODEL).
            // So we use a small state machine: ARM the data bus +
            // assert rx_valid, then HOLD it long enough to span ≥1
            // pb_clk edge but ≤2 (so we don't push twice).  Empirically
            // 4-6 ticks is the safe window.  After the hold drops, we
            // wait for the rxlvl_a counter to bump as the handshake
            // before queuing the next byte.
            const uint8_t b = scc_rx_bytes[scc_rx_byte_idx];
            auto* r = dut->rootp;
            const uint8_t cur_lvl =
                r->fpga_top__DOT__u_scc__DOT__rxlvl_a;
            static uint8_t prev_inject_lvl = 0;
            static size_t  prev_inject_idx = ~size_t(0);
            static int     inject_phase    = 0; // 0=arm, 1..N=hold, N+1=wait
            static const int HOLD_TICKS    = 4; // ≈ 1 pb_clk
            if (prev_inject_idx != scc_rx_byte_idx) {
                // New byte to inject — arm the data + valid lines.
                prev_inject_lvl = cur_lvl;
                prev_inject_idx = scc_rx_byte_idx;
                inject_phase    = 0;
                std::fprintf(stderr,
                    "[scc-rx] t=%llu fast-inject byte[%zu]=0x%02x ('%c') "
                    "armed (pre-lvl=%u)\n",
                    (unsigned long long)sim_time, scc_rx_byte_idx,
                    b, (b >= 0x20 && b < 0x7f) ? b : '.', cur_lvl);
            }
            if (inject_phase < HOLD_TICKS) {
                r->fpga_top__DOT__u_scc_board_uart__DOT__rx_data = b;
                r->fpga_top__DOT__u_scc_board_uart__DOT__rx_valid = 1;
                inject_phase++;
                return false;
            }
            // Hold expired — release rx_valid (the de-pulse block above
            // does this every tick anyway, but be explicit).  Now wait
            // for the SCC to acknowledge by bumping rxlvl_a.
            r->fpga_top__DOT__u_scc_board_uart__DOT__rx_valid = 0;
            if (cur_lvl > prev_inject_lvl ||
                (cur_lvl == 3 && prev_inject_lvl == 3)) {
                std::fprintf(stderr,
                    "[scc-rx] t=%llu fast-inject byte[%zu] consumed "
                    "(post-lvl=%u)\n",
                    (unsigned long long)sim_time, scc_rx_byte_idx,
                    cur_lvl);
                scc_rx_byte_idx++;
                return scc_rx_byte_idx >= scc_rx_bytes.size();
            }
            // Still waiting — possibly RX disabled (WR3[0]=0) or FIFO
            // full and overrun-latch held — keep polling.
            return false;
        }
        // Bit-serial path: drive uart_rtl_0_rxd through the byte one bit
        // at a time.  Idle state = line high.
        if (scc_rx_bit_state == 0) {
            // Start a new byte: drive start bit (line low).
            if (scc_rx_byte_idx >= scc_rx_bytes.size()) return true;
            const uint8_t b = scc_rx_bytes[scc_rx_byte_idx];
            std::fprintf(stderr,
                "[scc-rx] t=%llu bit-serial start byte[%zu]=0x%02x ('%c')\n",
                (unsigned long long)sim_time, scc_rx_byte_idx,
                b, (b >= 0x20 && b < 0x7f) ? b : '.');
            uart_rxd_pin = 0;
            scc_rx_bit_state = 1;
            scc_rx_bit_phase = SCC_TICKS_PER_BIT;
        }
        else {
            scc_rx_bit_phase--;
            if (scc_rx_bit_phase <= 0) {
                if (scc_rx_bit_state >= 1 && scc_rx_bit_state <= 8) {
                    // data bit (LSB-first 8N1)
                    const uint8_t b = scc_rx_bytes[scc_rx_byte_idx];
                    uart_rxd_pin = (b >> (scc_rx_bit_state - 1)) & 1u;
                    scc_rx_bit_state++;
                    scc_rx_bit_phase = SCC_TICKS_PER_BIT;
                }
                else if (scc_rx_bit_state == 9) {
                    // stop bit (line high)
                    uart_rxd_pin = 1;
                    scc_rx_bit_state = 10;
                    scc_rx_bit_phase = SCC_TICKS_PER_BIT;
                }
                else {
                    // end of byte; advance index, return to idle
                    scc_rx_byte_idx++;
                    scc_rx_bit_state = 0;
                    uart_rxd_pin = 1;
                }
            }
        }
        dut->uart_rtl_0_rxd = uart_rxd_pin;
        return scc_rx_byte_idx >= scc_rx_bytes.size() && scc_rx_bit_state == 0;
    };

    // Watchpoint state: previous value per watched PA.
    std::vector<uint32_t> watch_prev(watch_pas.size(), 0);
    std::vector<bool> watch_init(watch_pas.size(), false);
    uint64_t watch_last_pc = 0;

    // Full-design periodic checkpointing (Part 71): fires every
    // checkpoint_interval_cycles clock cycles (sim_time increments 2 per
    // tick()/cycle).  next_checkpoint_cycle starts at the resumed cycle
    // (if any) + one interval so a resumed run doesn't immediately
    // re-save at cycle 0.
    uint64_t next_checkpoint_cycle =
        (sim_time / 2) + checkpoint_interval_cycles;
    while (!Verilated::gotFinish() && sim_time < timeout_cycles) {
        tick();

        if (!checkpoint_dir.empty() && checkpoint_interval_cycles > 0) {
            const uint64_t cur_cycle = sim_time / 2;
            if (cur_cycle >= next_checkpoint_cycle) {
                checkpoint_save(checkpoint_dir, cur_cycle);
                next_checkpoint_cycle = cur_cycle + checkpoint_interval_cycles;
            }
        }

        // ── DDR PA watchpoints ──
        // Poll each watched PA every cycle; on change, log retired count,
        // last committed PC, old/new values.  Used to find which
        // instruction overwrote a PT entry, etc.
        if (!watch_pas.empty()) {
            for (size_t i = 0; i < watch_pas.size(); i++) {
                const uint32_t cur = ddr_read_phys32(watch_pas[i]);
                if (!watch_init[i]) {
                    watch_prev[i] = cur;
                    watch_init[i] = true;
                    std::fprintf(stderr,
                        "[watch] PA 0x%08x init=0x%08x (t=%llu retired=%llu)\n",
                        watch_pas[i], cur,
                        (unsigned long long)sim_time,
                        (unsigned long long)
                            DBGT_REF(dut->rootp, committed));
                } else if (cur != watch_prev[i]) {
                    std::fprintf(stderr,
                        "[watch] PA 0x%08x: 0x%08x -> 0x%08x "
                        "(t=%llu retired=%llu last_pc=0x%08x)\n",
                        watch_pas[i], watch_prev[i], cur,
                        (unsigned long long)sim_time,
                        (unsigned long long)
                            DBGT_REF(dut->rootp, committed),
                        (unsigned)watch_last_pc);
                    watch_prev[i] = cur;
                }
            }
            // Track the most recent retired PC so we can attribute the
            // write to the instruction that just retired.  Read on every
            // cycle since boundary_event/kind=1 fires only on retire.
            if (DBGT_REF(dut->rootp, boundary_event) &&
                DBGT_REF(dut->rootp, boundary_kind) == 1) {
                watch_last_pc = DBGT_REF(dut->rootp, boundary_pc);
            }
        }

        // ── AXI / DAFB lockstep capture ──
        // Sniff the M0 narrow CPU data path (core_daxi_*) and the M3 wide
        // instruction-fetch path (ifa_*) at the xbar boundary.  Snapshot
        // AR/AW handshakes, emit a CSV line on R/B completion (or W beat
        // for writes).  Address filtering is applied in axi_emit() — the
        // same emit function feeds both the AXI lockstep CSV (broad
        // peripheral filter) and the DAFB lockstep CSV (narrow DAFB +
        // TurboSCSI filter).  See docs/axi_lockstep.md and
        // docs/dafb_lockstep.md.
        if (axi_log_fp || dafb_log_fp) {
            auto* r = dut->rootp;
            // ── M0 read path (CPU LSU narrow side) ──
            // NOTE (repo-split socketization): the narrow CPU data-master
            // nets `core_daxi_*` were fpga_top-internal in the legacy
            // direct-instantiation build.  Under the CPU socket they live
            // inside u_cpu (the wrapper presents axi_d_*, fpga_top wires
            // them as cpu_d_*), so the old core_daxi_* taps no longer
            // exist at fpga_top scope.  This M0 lockstep capture relocates
            // to the CPU repo tb (Phase 6 follow-up); compiled out here.
            // The M3 instruction-fetch path below uses ifa_* (SoC xbar
            // nets) and stays live.
            //
            // task #272: the guard used to read `#ifndef CPU_M68K`, i.e.
            // the OPPOSITE of what this comment says — `core_daxi_*`
            // doesn't exist at fpga_top scope for ANY current CPU value
            // (this block predates socketization entirely), so the old
            // polarity meant it was dead code for CPU=m68k (where the
            // comment says it belongs) and a hard compile error for
            // anything else, which is exactly what surfaced building
            // `tb-fpga-top-rom CPU=m68k040` for the first time. `#if 0`
            // until the real Phase-6 cpu_d_*-based relocation happens.
#if 0
            // AR handshake: capture address + size.  arsize is the AXI
            // log2(bytes) field (0=byte, 1=half, 2=long); the CPU's
            // architectural access size matches.  Latching this here is
            // what gives the lockstep CSV byte-granularity that aligns
            // with MAME's handler-level mask decode.
            if (!m0_ar_pending &&
                r->fpga_top__DOT__core_daxi_arvalid &&
                r->fpga_top__DOT__core_daxi_arready) {
                m0_ar_pending = true;
                m0_ar_addr = r->fpga_top__DOT__core_daxi_araddr;
                m0_ar_size = r->fpga_top__DOT__core_daxi_arsize;
            }
            // R handshake: emit and clear.  Slice the size_b bytes the
            // CPU asked for out of the 32-bit narrow data response.  68k
            // narrow rdata is delivered with byte 0 at bits [7:0]; for
            // peripherals the rdata byte is broadcast across all four
            // lanes (peripheral_bus.v), so the LSB-first slice still
            // produces the right value.  For VOID/open-bus the data is
            // 0x00000000 (MAME-canonical default unmap_value) and any
            // size mask still matches MAME.
            if (m0_ar_pending &&
                r->fpga_top__DOT__core_daxi_rvalid &&
                r->fpga_top__DOT__core_daxi_rready) {
                const uint32_t data = r->fpga_top__DOT__core_daxi_rdata;
                const uint32_t size_b = (m0_ar_size <= 2u)
                                         ? (1u << m0_ar_size) : 4u;
                uint32_t value = data;
                if (size_b == 1) value = data & 0xFFu;
                else if (size_b == 2) value = data & 0xFFFFu;
                axi_emit("R", m0_ar_addr, size_b, value);
                m0_ar_pending = false;
            }
            // ── M0 write path ──
            // AW handshake: capture address.
            if (!m0_aw_pending &&
                r->fpga_top__DOT__core_daxi_awvalid &&
                r->fpga_top__DOT__core_daxi_awready) {
                m0_aw_pending = true;
                m0_aw_addr = r->fpga_top__DOT__core_daxi_awaddr;
                m0_aw_w_done = false;
            }
            // W handshake: capture data + decode size from wstrb.
            if (m0_aw_pending && !m0_aw_w_done &&
                r->fpga_top__DOT__core_daxi_wvalid &&
                r->fpga_top__DOT__core_daxi_wready) {
                const uint32_t wd = r->fpga_top__DOT__core_daxi_wdata;
                const uint8_t  ws = r->fpga_top__DOT__core_daxi_wstrb;
                decode_narrow_w(m0_aw_addr, wd, ws,
                                m0_aw_w_addr, m0_aw_w_size, m0_aw_w_data);
                m0_aw_w_done = true;
            }
            // B handshake: emit and clear.
            if (m0_aw_pending && m0_aw_w_done &&
                r->fpga_top__DOT__core_daxi_bvalid &&
                r->fpga_top__DOT__core_daxi_bready) {
                axi_emit("W", m0_aw_w_addr, m0_aw_w_size, m0_aw_w_data);
                // ROM-write watchpoint: any write into 0x40000000..0x40FFFFFF
                // corrupts the ROM image (mem_model is flat, no
                // write-protection).  Yell loudly so 0xCAB-style sum drifts
                // can be traced back to a stray store.  Cap output at 64 to
                // avoid log flooding if a runaway store storm hits.
                static uint32_t rom_write_count = 0;
                if (m0_aw_w_addr >= 0x40000000u &&
                    m0_aw_w_addr <  0x41000000u &&
                    rom_write_count < 64) {
                    const uint32_t pc =
                        DBGT_REF(r, boundary_pc);
                    std::fprintf(stderr,
                        "[rom-write] retired=%llu PC=0x%08x addr=0x%08x size=%u data=0x%08x\n",
                        (unsigned long long)retired, pc,
                        m0_aw_w_addr, m0_aw_w_size, m0_aw_w_data);
                    std::fflush(stderr);
                    rom_write_count++;
                }
                m0_aw_pending = false;
                m0_aw_w_done = false;
            }
#endif  // !CPU_M68K (M0 narrow CPU-data lockstep capture)
            // ── M3 read path (CPU instruction fetch, wide side) ──
            // The IF master is wide-only.  Most accesses are inside the
            // ROM filter window; capture them only when the CPU runs
            // wild and fetches from outside RAM/ROM (which surfaces as
            // a divergence we want to see).
            if (!m3_ar_pending &&
                r->fpga_top__DOT__ifa_arvalid &&
                r->fpga_top__DOT__ifa_arready) {
                m3_ar_pending = true;
                m3_ar_addr = r->fpga_top__DOT__ifa_araddr;
            }
            if (m3_ar_pending &&
                r->fpga_top__DOT__ifa_rvalid &&
                r->fpga_top__DOT__ifa_rready) {
                // Slice the lane the IF wanted: addr[3:2].
                const unsigned lane = (m3_ar_addr >> 2) & 3u;
                uint32_t word = 0;
                switch (lane) {
                case 0: word = r->fpga_top__DOT__ifa_rdata[0]; break;
                case 1: word = r->fpga_top__DOT__ifa_rdata[1]; break;
                case 2: word = r->fpga_top__DOT__ifa_rdata[2]; break;
                case 3: word = r->fpga_top__DOT__ifa_rdata[3]; break;
                }
                axi_emit("R", m3_ar_addr & ~0x3u, 4, word);
                m3_ar_pending = false;
            }
        }

        // ── VIA1 lockstep capture ──
        // Sniff the peripheral_bus VIA1 master face (pb_via1_rd / pb_via1_wr
        // / pb_via1_addr / pb_via1_wdata / pb_via1_rdata).  The bus runs on
        // pb_clk (slower than sys_clk); pb_via1_rd / pb_via1_wr are 1-cycle
        // pulses on the pb_clk and we want EXACTLY one event per pulse.
        // Detect the rising edge via the saved-prev technique.  Reads need
        // to wait one pb_clk cycle for pb_rdata (registered in via1.v) — we
        // capture the addr on the pulse and emit when via1.v drops pb_ack
        // back into the captured rdata.
        if (via1_log_fp) {
            auto* r = dut->rootp;
            // Track the most recent retired macro PC so each VIA1 event
            // can be attributed to the ROM instruction that issued it.
            if (DBGT_REF(r, boundary_event) &&
                DBGT_REF(r, boundary_kind) == 1) {
                via1_last_pc = DBGT_REF(r, boundary_pc);
            }
            const uint8_t  via1_wr = r->fpga_top__DOT__pb_via1_wr;
            const uint8_t  via1_rd = r->fpga_top__DOT__pb_via1_rd;
            const uint32_t via1_addr  = r->fpga_top__DOT__pb_via1_addr;
            const uint32_t via1_wdata = r->fpga_top__DOT__pb_via1_wdata;
            const uint32_t via1_rdata = r->fpga_top__DOT__pb_via1_rdata;

            // Write: emit on rising edge; pb_via1_wdata is valid same cycle.
            if (via1_wr && !prev_via1_wr) {
                via1_emit("W", via1_addr, via1_wdata);
            }
            // Read: latch the addr on rising edge; emit one cycle later
            // when the registered pb_rdata is valid (next pb_clk).
            if (via1_rd && !prev_via1_rd) {
                via1_rd_pending = true;
                via1_rd_reg = via1_addr & 0xFu;
            } else if (via1_rd_pending && !via1_rd) {
                // Falling edge of pb_rd — pb_rdata is now registered with
                // the response.  via1.v's pb_rdata combinational mux runs
                // off pb_addr (which holds while pb_rd is high), so reading
                // pb_rdata on the falling edge captures the response that
                // the peripheral_bus will return upstream.
                via1_emit("R", via1_rd_reg, via1_rdata);
                via1_rd_pending = false;
            }
            prev_via1_wr = via1_wr;
            prev_via1_rd = via1_rd;
            // Lockstep capture is the only thing a +via1_lockstep_log run
            // is for — once the VIA1 event cap is reached, stop rather
            // than burning the whole +timeout window booting past it.
            if (via1_log_seq >= via1_log_max) {
                std::fprintf(stderr,
                    "[via1-lockstep] capture complete (%llu events) "
                    "— stopping sim\n",
                    (unsigned long long)via1_log_seq);
                break;
            }
        }

        // Targeted SCC register-bus visibility for frontier work.  +probe
        // is explicit because the later SCC polling loops are very chatty.
        if (probe) {
            auto* r = dut->rootp;
            static bool prev_scc_rd = false;
            static bool prev_scc_wr = false;
            const bool scc_rd = r->fpga_top__DOT__pb_scc_rd;
            const bool scc_wr = r->fpga_top__DOT__pb_scc_wr;
            if (scc_wr && !prev_scc_wr) {
                std::fprintf(stderr,
                    "[scc-bus] retired=%llu W reg=0x%x data=0x%02x\n",
                    (unsigned long long)retired,
                    (unsigned)r->fpga_top__DOT__pb_scc_addr,
                    (unsigned)r->fpga_top__DOT__pb_scc_wdata & 0xffu);
            }
            if (scc_rd && !prev_scc_rd) {
                std::fprintf(stderr,
                    "[scc-bus] retired=%llu R reg=0x%x start\n",
                    (unsigned long long)retired,
                    (unsigned)r->fpga_top__DOT__pb_scc_addr);
            }
            if (!scc_rd && prev_scc_rd) {
                std::fprintf(stderr,
                    "[scc-bus] retired=%llu R done data=0x%02x\n",
                    (unsigned long long)retired,
                    (unsigned)r->fpga_top__DOT__pb_scc_rdata & 0xffu);
            }
            if ((scc_rd && !prev_scc_rd) || (scc_wr && !prev_scc_wr) ||
                (!scc_rd && prev_scc_rd))
                std::fflush(stderr);
            prev_scc_rd = scc_rd;
            prev_scc_wr = scc_wr;
        }

        // ── SCC byte-level fast-inject de-pulse ──
        //
        // Direct-fast-inject: bypass the bridge entirely and push the byte
        // into the SCC's chan A RX FIFO + bump rxlvl_a + raise rx_ip_a.
        // The bridge's rx_valid pulse path is unreliable in sim — the
        // testbench drives rx_valid for ~1 sys_clk (= ~0.25 pb_clk) which
        // the bridge sampling at pb_clk often misses, and the bit-serial
        // path's SCC_TICKS_PER_BIT=50 sys_clk = ~12 pb_clk is ~35× too
        // fast for the bridge's BAUD=115200 / 434-pb_clk-per-bit clock,
        // so the bridge's start-bit detector misframes.  The
        // direct-write path skips both and feeds the SCC the byte the
        // way scc.v's RX FIFO writes it on a real bridge byte arrival.
        // This is what "fast" used to mean — restoring it correctly.
        if (scc_rx_inject_fast) {
            dut->rootp->fpga_top__DOT__u_scc_board_uart__DOT__rx_valid = 0;
        }

        // ── NMI / programmer's-switch fire driver ──
        // We model a press of board btn[1] (NMI) by overriding the
        // debounce module's outputs to "pressed" (out_n=0) for
        // nmi_pulse_cycles sys_clk cycles, then back to "released"
        // (out_n=1).  The single rising edge of btn1_sync feeds the
        // sys_clk-domain nmi_btn_pulse; fpga_top_peripherals.vh syncs
        // it into pb_clk and irq_agg.v's `nmi_rise` detector latches
        // exactly once per press.
        //
        // Why we override the debounce internals: at boot the testbench
        // already pokes u_btn1_db.{out_n,sync_meta,sync_q,sync_qq}=1
        // to suppress the post-init NMI storm (see comment around the
        // tick() loop start ~line 1170).  Driving the dut->btn pin alone
        // would not flip btn1_sync because the 2_000_000-cycle debounce
        // would never settle in sim wall-time.
        if (!nmi_fired) {
            bool inst_match = nmi_at_inst_set && retired >= nmi_at_inst;
            // PC-match check uses the boundary_pc that was last seen by
            // the macsbug detector below.  For boot-time NMI tests (e.g.
            // before any boundary fires) the inst-count form is more
            // reliable.
            if (inst_match) {
                std::fprintf(stderr,
                    "[nmi] t=%llu firing NMI at retired=%llu "
                    "(trigger=inst_count threshold=%llu)\n",
                    (unsigned long long)sim_time,
                    (unsigned long long)retired,
                    (unsigned long long)nmi_at_inst);
                auto* r = dut->rootp;
                // Override debounce to "pressed".  The downstream
                // nmi_btn_pulse = btn1_sync = ~out_n becomes 1.  We hold
                // for nmi_pulse_cycles, then restore release.
                r->fpga_top__DOT__u_btn1_db__DOT__out_n     = 0;
                r->fpga_top__DOT__u_btn1_db__DOT__sync_meta = 0;
                r->fpga_top__DOT__u_btn1_db__DOT__sync_q    = 0;
                r->fpga_top__DOT__u_btn1_db__DOT__sync_qq   = 0;
                nmi_press_active_until = sim_time + 2 * nmi_pulse_cycles;
                nmi_fired = true;
            }
        }
        if (nmi_press_active_until != 0 && sim_time >= nmi_press_active_until) {
            // Restore "released" state so the next NMI press could rise
            // again, and so the debounce stays latched at out_n=1 forever
            // (matching the boot-time poke).
            auto* r = dut->rootp;
            r->fpga_top__DOT__u_btn1_db__DOT__out_n     = 1;
            r->fpga_top__DOT__u_btn1_db__DOT__sync_meta = 1;
            r->fpga_top__DOT__u_btn1_db__DOT__sync_q    = 1;
            r->fpga_top__DOT__u_btn1_db__DOT__sync_qq   = 1;
            r->fpga_top__DOT__u_btn1_db__DOT__cnt       = 0;
            std::fprintf(stderr,
                "[nmi] t=%llu releasing NMI button\n",
                (unsigned long long)sim_time);
            nmi_press_active_until = 0;
        }

        // ── Drive RX once MacsBug has been entered ──
        // Wait for MacsBug's own SCC re-init to finish before we inject.
        // The Q700 ROM's MacsBug entry path does a hardware reset of
        // chan-A (WR9=0xc0), reprograms WR5, etc.  If we inject too
        // soon, our byte gets wiped.  We use WR5=0xea as the "init
        // complete" signature (the final WR5 value MacsBug writes for
        // active TX/RX).
        if (scc_rx_inject && macsbug_seen && !scc_rx_inject_done) {
            const uint8_t wr5 =
                dut->rootp->fpga_top__DOT__u_scc__DOT__wra[5];
            const bool init_done = (wr5 == 0xea);
            // Wait at least 128 sim_time after MacsBug entry AND for the
            // SCC re-init signature.  If init_done never arrives we
            // fall back to the time-only gate after 1M sim_time.
            const bool time_fallback =
                sim_time - macsbug_seen_time >= 1000000;
            if (sim_time - macsbug_seen_time >= 128 &&
                (init_done || time_fallback)) {
                if (step_rx_inject()) {
                    if (scc_rx_byte_idx >= scc_rx_bytes.size() &&
                        !scc_rx_inject_done) {
                        scc_rx_inject_done = true;
                        scc_rx_inject_done_time = sim_time;
                        std::fprintf(stderr,
                            "[scc-rx] t=%llu inject queue drained "
                            "(%zu bytes)\n",
                            (unsigned long long)sim_time,
                            scc_rx_bytes.size());
                    }
                }
            }
        }

        // ── Sniff CPU-side TX bytes (post-SCC, pre-UART-bridge) ──
        // scc_uart_tx_valid is asserted for one pb_clk cycle when the SCC
        // serialiser completes a byte time.  Sample on the rising edge.
        {
            static uint8_t last_scc_tx_valid = 0;
            const uint8_t v =
                dut->rootp->fpga_top__DOT__scc_uart_tx_valid;
            if (v && !last_scc_tx_valid && scc_tx_count < scc_tx_max) {
                const uint8_t b =
                    dut->rootp->fpga_top__DOT__scc_uart_tx_data;
                if (scc_tx_count == 0) {
                    scc_first_tx_byte = b;
                    scc_first_tx_time = sim_time;
                }
                scc_tx_bytes.push_back(b);
                scc_tx_count++;
                if (scc_tx_fp) {
                    std::fprintf(scc_tx_fp,
                        "%llu %llu 0x%02x '%c'\n",
                        (unsigned long long)retired,
                        (unsigned long long)sim_time, b,
                        (b >= 0x20 && b < 0x7f) ? b : '.');
                    std::fflush(scc_tx_fp);
                }
                std::fprintf(stderr,
                    "[scc-tx]      t=%llu byte[%llu]=0x%02x ('%c')\n",
                    (unsigned long long)sim_time,
                    (unsigned long long)(scc_tx_count - 1), b,
                    (b >= 0x20 && b < 0x7f) ? b : '.');
            }
            last_scc_tx_valid = v;
        }

        // ── Sniff UART-pin TX bytes (post bridge serialiser) ──
        // The bridge instantiation uses BAUD=115200 and CLK_HZ=PB_CLK_HZ.
        // PB_CLK_HZ defaults to 50 MHz in SIM_MODEL, so BAUD_DIV ≈ 434
        // pb_clk per UART bit.  pb_clk : sys_clk = 1:4 in SIM_MODEL, so
        // 434 pb_clk = 1736 sys_clk = 3472 sim_time per UART bit.  We
        // detect the start bit (line goes low while idle) and sample at
        // 1.5 bit-times to land in the middle of bit 0.
        {
            const int UART_TX_BIT_SIM_TIME = 3472; // 50 MHz / 115200
            const uint8_t pin = dut->uart_rtl_0_txd & 1u;
            if (uart_tx_state == 0) {
                if (pin == 0) {
                    uart_tx_state = 1;
                    uart_tx_bit_idx = 0;
                    uart_tx_shift = 0;
                    // Sample first data bit 1.5 bit-times from now.
                    uart_tx_bit_phase = UART_TX_BIT_SIM_TIME +
                                         UART_TX_BIT_SIM_TIME / 2;
                }
            } else {
                uart_tx_bit_phase -= 2; // sim_time advances 2 per tick()
                if (uart_tx_bit_phase <= 0) {
                    if (uart_tx_bit_idx < 8) {
                        // LSB-first 8N1.
                        if (pin) uart_tx_shift |= (1u << uart_tx_bit_idx);
                        uart_tx_bit_idx++;
                        uart_tx_bit_phase = UART_TX_BIT_SIM_TIME;
                    } else {
                        // Stop bit consumed; emit the byte if not exceeding cap.
                        if (scc_tx_uart_count < scc_tx_max) {
                            const uint8_t b = uart_tx_shift;
                            if (scc_tx_uart_count == 0) {
                                scc_first_tx_uart_byte = b;
                                scc_first_tx_uart_time = sim_time;
                            }
                            scc_tx_uart_bytes.push_back(b);
                            scc_tx_uart_count++;
                            if (scc_tx_uart_fp) {
                                std::fprintf(scc_tx_uart_fp,
                                    "%llu %llu 0x%02x '%c'\n",
                                    (unsigned long long)retired,
                                    (unsigned long long)sim_time, b,
                                    (b >= 0x20 && b < 0x7f) ? b : '.');
                                std::fflush(scc_tx_uart_fp);
                            }
                            std::fprintf(stderr,
                                "[scc-tx-uart] t=%llu byte[%llu]=0x%02x ('%c')\n",
                                (unsigned long long)sim_time,
                                (unsigned long long)(scc_tx_uart_count - 1), b,
                                (b >= 0x20 && b < 0x7f) ? b : '.');
                        }
                        uart_tx_state = 0;
                    }
                }
            }
        }

#ifdef CPU_M68K040
        retired = g_cpu040_retired_count;
#else
        retired = DBGT_REF(dut->rootp, committed);
#endif
        if (irq_at_inst_set && !irq_triggered && retired >= irq_at_inst) {
            irq_triggered = true;
            irq_remaining = irq_inject_total;
            irq_next_retired = retired;
            std::fprintf(stderr,
                "[irq] t=%llu armed IPL%u count=%llu gap=%llu "
                "(trigger=inst threshold=%llu retired=%llu)\n",
                (unsigned long long)sim_time, (unsigned)irq_inject_level,
                (unsigned long long)irq_inject_total,
                (unsigned long long)irq_inject_gap,
                (unsigned long long)irq_at_inst,
                (unsigned long long)retired);
            std::fflush(stderr);
        }
        // Cycle-precise injection (+irq_at_cycle) is handled by
        // irq_cycle_tick() inside tick() — see task #41.
        if (irq_triggered && irq_remaining != 0 &&
            retired >= irq_next_retired &&
            DBGT_REF(dut->rootp, irq_inject_pending_lvl) == 0) {
            DBGT_REF(dut->rootp, irq_inject_pending_lvl) =
                irq_inject_level;
            irq_remaining--;
            irq_next_retired = retired + irq_inject_gap;
            std::fprintf(stderr,
                "[irq] t=%llu injected IPL%u at retired=%llu "
                "remaining=%llu next_retired=%llu\n",
                (unsigned long long)sim_time, (unsigned)irq_inject_level,
                (unsigned long long)retired,
                (unsigned long long)irq_remaining,
                (unsigned long long)irq_next_retired);
            std::fflush(stderr);
        }
        maybe_log_dual_commit(probe);
        maybe_log_exception(probe);
        if (exc_log_count < exc_log_max &&
            DBGT_REF(dut->rootp, boundary_event) &&
            DBGT_REF(dut->rootp, boundary_kind) == 2 &&
            // Skip the exceptions that ARE normal Mac operation and would
            // otherwise bury everything else: line-A/line-F emulator traps
            // (10/11), TRAP #n (32..47) and autovector interrupts (24..31).
            !(DBGT_REF(dut->rootp, boundary_exc_vec) == 10 ||
              DBGT_REF(dut->rootp, boundary_exc_vec) == 11 ||
              (DBGT_REF(dut->rootp, boundary_exc_vec) >= 24 &&
               DBGT_REF(dut->rootp, boundary_exc_vec) <= 47))) {
            exc_log_count++;
            std::fprintf(stderr,
                "[exc] #%llu retired=%llu vec=%u fault_pc=0x%08x "
                "fault_addr=0x%08x last_pc=0x%08x\n",
                (unsigned long long)exc_log_count,
                (unsigned long long)retired,
                (unsigned)DBGT_REF(dut->rootp, boundary_exc_vec),
                (unsigned)DBGT_REF(dut->rootp, boundary_fault_pc),
                (unsigned)DBGT_REF(dut->rootp, boundary_fault_addr),
                (unsigned)DBGT_REF(dut->rootp, boundary_pc));
            std::fflush(stderr);
        }
        // +halt_on_vec: stop the sim cleanly on a specific exception
        // vector.  Designed for catching the post-DBF wild-jump's
        // illegal-instruction (vec=4) entry without firehose-tracing
        // the full subsequent cascade.
        if (!halt_on_vecs.empty() &&
            DBGT_REF(dut->rootp, boundary_event) &&
            DBGT_REF(dut->rootp, boundary_kind) == 2) {
            uint32_t vec_now =
                DBGT_REF(dut->rootp, boundary_exc_vec);
            for (const auto& want : halt_on_vecs) {
                if (vec_now == want.first &&
                    (want.second == 0xFFFFFFFFu ||
                     want.second ==
                        (uint32_t)DBGT_REF(dut->rootp, boundary_fault_pc))) {
                    std::fprintf(stderr,
                        "[halt-on-vec] vec=%u retired=%llu fault_pc=0x%08x "
                        "fault_addr=0x%08x last_pc=0x%08x A7=0x%08x SR=0x%04x\n",
                        (unsigned)vec_now,
                        (unsigned long long)retired,
                        (unsigned)DBGT_REF(dut->rootp, boundary_fault_pc),
                        (unsigned)DBGT_REF(dut->rootp, boundary_fault_addr),
                        (unsigned)DBGT_REF(dut->rootp, boundary_pc),
                        read_arch_reg(15),
                        (unsigned)CPUI_REF(dut->rootp, u_commit__DOT__arch_sr));
                    std::fprintf(stderr,
                        "[halt-on-vec] D0=%08x D1=%08x D2=%08x D3=%08x "
                        "D4=%08x D5=%08x D6=%08x D7=%08x\n",
                        read_arch_reg(0), read_arch_reg(1),
                        read_arch_reg(2), read_arch_reg(3),
                        read_arch_reg(4), read_arch_reg(5),
                        read_arch_reg(6), read_arch_reg(7));
                    std::fprintf(stderr,
                        "[halt-on-vec] A0=%08x A1=%08x A2=%08x A3=%08x "
                        "A4=%08x A5=%08x A6=%08x A7=%08x\n",
                        read_arch_reg(8), read_arch_reg(9),
                        read_arch_reg(10), read_arch_reg(11),
                        read_arch_reg(12), read_arch_reg(13),
                        read_arch_reg(14), read_arch_reg(15));
                    std::fflush(stderr);
                    sd_summary();
                    return 0;
                }
            }
        }
#ifdef CPU_M68K040
        // Stage 5 has no legacy dbg_boundary_* socket taps.  Its two
        // program-ordered macro-last retire flows are simPublic, however, so
        // use those to make +pc_dump useful for full-SoC CPU040 runs.
        {
            auto* r = dut->rootp;
            const bool v0 =
                r->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_debugMacroRetirePc_0_valid;
            const bool v1 =
                r->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_debugMacroRetirePc_1_valid;
            const uint32_t pc0 =
                r->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_debugMacroRetirePc_0_payload;
            const uint32_t pc1 =
                r->fpga_top__DOT__u_cpu__DOT__socket_core__DOT__RobPlugin_logic_debugMacroRetirePc_1_payload;
            // The fastdiag host fill is keyed by retired macro PC.  It used
            // to be hidden inside the legacy boundary-tap block below, so
            // CPU040 silently applied the ROM patch without its required
            // RAM side effect and then spent millions of macros verifying
            // state that had never been installed.
            if (v0) maybe_apply_ramtest_mame_state_fastfill(pc0,
                                                            ramtest_fastfill);
            if (v1) maybe_apply_ramtest_mame_state_fastfill(pc1,
                                                            ramtest_fastfill);
            // pc_dump writing DISABLED here (superseded by the
            // g_pc_dump_fp tap in tick(), see its declaration comment):
            // v0/v1 (debugMacroRetirePc_0/1_valid) were found this
            // investigation to only pulse during an initial startup burst
            // on a CPU_M68K040 build, silently truncating every prior
            // +pc_dump_path capture to a few dozen lines.  The
            // maybe_apply_ramtest_mame_state_fastfill() calls above this
            // block are UNRELATED and still real/needed -- only the dump
            // write itself is dead-code-disabled here, not the whole tap.
            if (false && pc_dump_fp && pc_dump_count < pc_dump_max &&
                retired >= pc_dump_next_at && (v0 || v1)) {
                auto emit_pc = [&](uint32_t pc) {
                    if (pc_dump_count >= pc_dump_max) return;
                    std::fprintf(pc_dump_fp, "%llu %llu 0x%08x\n",
                        (unsigned long long)retired,
                        (unsigned long long)sim_time, (unsigned)pc);
                    pc_dump_count++;
                    pc_dump_prev = pc;
                };
                if (v0) emit_pc(pc0);
                if (v1) emit_pc(pc1);
                pc_dump_next_at = retired + pc_dump_every;
                if (pc_dump_count >= pc_dump_max) std::fflush(pc_dump_fp);
            }
        }
#endif
        if (DBGT_REF(dut->rootp, boundary_event) &&
            DBGT_REF(dut->rootp, boundary_kind) == 1) {
            const uint32_t boundary_pc =
                DBGT_REF(dut->rootp, boundary_pc);
            const uint32_t boundary_a_pc =
                DBGT_REF(dut->rootp, boundary_a_pc);
            const uint32_t boundary_seq =
                DBGT_REF(dut->rootp, boundary_seq);
            const uint32_t seq_delta = boundary_seq - pc_dump_seq_prev;
            // Per-instruction PC trace dump.  Emit one line per retired
            // macro (last_uop boundary).  In dual-commit cycles where
            // both head+0 and head+1 retire as last_uop, dbg_boundary_seq
            // increments by 2; we emit dbg_boundary_a_pc (head+0) first,
            // then dbg_boundary_pc (head+1).  Without this, head+0's PC
            // would be silently dropped from the trace (commit.v's line
            // 1460 NBA overrides dbg_boundary_pc with lb_pc on dual).
            if (pc_dump_fp && pc_dump_count < pc_dump_max &&
                retired >= pc_dump_next_at) {
                if (seq_delta == 2 && boundary_a_pc != pc_dump_prev) {
                    std::fprintf(pc_dump_fp,
                        "%llu %llu 0x%08x\n",
                        (unsigned long long)retired,
                        (unsigned long long)sim_time,
                        (unsigned)boundary_a_pc);
                    pc_dump_count++;
                    pc_dump_prev = boundary_a_pc;
                    pc_dump_next_at = retired + pc_dump_every;
                }
                if (pc_dump_count < pc_dump_max &&
                    boundary_pc != pc_dump_prev) {
                    std::fprintf(pc_dump_fp,
                        "%llu %llu 0x%08x\n",
                        (unsigned long long)retired,
                        (unsigned long long)sim_time,
                        (unsigned)boundary_pc);
                    pc_dump_count++;
                    pc_dump_prev = boundary_pc;
                    pc_dump_next_at = retired + pc_dump_every;
                }
                if (pc_dump_count >= pc_dump_max) {
                    std::fflush(pc_dump_fp);
                }
            }
            pc_dump_seq_prev = boundary_seq;
            // +poke_word_at_pc=<pc>:<addr>:<value>: see declaration above.
            for (const auto& p : poke_word_at_pc) {
                if (p.pc != boundary_pc) continue;
                ddr_write_phys_byte(p.addr + 0,
                    static_cast<uint8_t>(p.value >> 8));
                ddr_write_phys_byte(p.addr + 1,
                    static_cast<uint8_t>(p.value));
                poke_word_hits++;
                std::fprintf(stderr,
                    "[poke-word] hit pc=0x%08x wrote 0x%04x to 0x%08x "
                    "(hit #%llu)\n",
                    p.pc, p.value, p.addr,
                    (unsigned long long)poke_word_hits);
            }
            // +arch_dump_at_pc=<hex>: dump committed arch state on every
            // hit, capped at +arch_dump_max=<n> (default 16) total emits
            // across all requested PCs.  Format mirrors MAME's bpset
            // tracelog so the lines diff side-by-side with golden state.
            if (!arch_dump_pcs.empty() &&
                arch_dump_count < arch_dump_max) {
                bool match = false;
                for (uint32_t want : arch_dump_pcs) {
                    if (want == boundary_pc) { match = true; break; }
                }
                if (match) {
                    auto* r2 = dut->rootp;
                    const unsigned ccr_phys =
                        CPUI_REF(r2, u_ccr_rat__DOT__crat_tag);
                    const unsigned ccr =
                        CPUI_REF(r2, u_ccr_rat__DOT__ccr_prf)[ccr_phys];
                    // A7 is not a real architectural register — it's a
                    // banked alias that selects USP / SSP (=MSP) / ISP
                    // based on SR.S and SR.M.  Expose the three stack
                    // pointers separately from u_commit's CSR shadows.
                    const uint32_t usp =
                        CPUI_REF(r2, u_commit__DOT__usp);
                    const uint32_t ssp =
                        CPUI_REF(r2, u_commit__DOT__ssp);
                    const uint32_t isp =
                        CPUI_REF(r2, u_commit__DOT__isp);
#ifndef CPU_M68K
                    // Diagnostic: A7 banking — print crat[15],
                    // committed_a7_phys, PRF[15], PRF[crat[15]] so we
                    // can see if the live-A7 path tracks ISP correctly.
                    // committed_a7_phys is a deep commit-internal tap;
                    // this fine-grained A7-banking dump relocates to the
                    // CPU repo tb (Phase 6 follow-up).
                    const unsigned crat15 =
                        CPUI_REF(r2, u_rat__DOT__crat)[15];
                    const unsigned com_a7_phys =
                        CPUI_REF(r2, u_commit__DOT__committed_a7_phys);
                    std::fprintf(stderr,
                        "[arch-a7-dbg] crat[15]=%u committed_a7_phys=%u "
                        "prf[15]=0x%08x prf[crat15]=0x%08x prf[19=USP]=0x%08x "
                        "prf[20=SSP]=0x%08x prf[21=ISP]=0x%08x\n",
                        crat15, com_a7_phys,
                        (unsigned)CPUI_REF(r2, prf)[15],
                        (unsigned)CPUI_REF(r2, prf)[crat15],
                        (unsigned)CPUI_REF(r2, prf)[19],
                        (unsigned)CPUI_REF(r2, prf)[20],
                        (unsigned)CPUI_REF(r2, prf)[21]);
#endif  // !CPU_M68K
                    const uint32_t vbr =
                        CPUI_REF(r2, u_commit__DOT__arch_vbr);
                    const uint16_t sr =
                        CPUI_REF(r2, u_commit__DOT__arch_sr);
                    std::fprintf(stderr,
                        "[arch] PC=%08X retired=%llu "
                        "D0=%08x D1=%08x D2=%08x D3=%08x "
                        "D4=%08x D5=%08x D6=%08x D7=%08x "
                        "A0=%08x A1=%08x A2=%08x A3=%08x "
                        "A4=%08x A5=%08x A6=%08x A7=%08x "
                        "USP=%08x ISP=%08x MSP=%08x "
                        "VBR=%08x SR=%04x CCR=%02x\n",
                        boundary_pc, (unsigned long long)retired,
                        read_arch_reg(0), read_arch_reg(1),
                        read_arch_reg(2), read_arch_reg(3),
                        read_arch_reg(4), read_arch_reg(5),
                        read_arch_reg(6), read_arch_reg(7),
                        read_arch_reg(8), read_arch_reg(9),
                        read_arch_reg(10), read_arch_reg(11),
                        read_arch_reg(12), read_arch_reg(13),
                        read_arch_reg(14), read_arch_reg(15),
                        usp, isp, ssp,
                        vbr, sr,
                        ccr & 0x1f);
                    std::fflush(stderr);
                    arch_dump_count++;
                }
            }
            record_pc_history(boundary_pc,
                probe && (boundary_pc == 0x4084a7d4u ||
                          boundary_pc == 0x4084a802u));
            maybe_log_probe(boundary_pc, probe);
            c96_probe(boundary_pc);
            maybe_apply_ramtest_mame_state_fastfill(boundary_pc,
                                                    ramtest_fastfill);
            if (!hit_q700_feature && boundary_pc == 0x40802f96u) {
                q700_feature = read_arch_reg(1);
                q700_entry = read_arch_reg(9);
                hit_q700_feature = true;
                std::fprintf(stderr,
                    "[fpga-rom] q700_descriptor_selected=1 entry=0x%08x feature=0x%08x\n",
                    q700_entry, q700_feature);
                if (expect_q700_feature &&
                    q700_feature != expected_q700_feature) {
                    bad_q700_feature = true;
                    hit_pc = boundary_pc;
                    break;
                }
            }
            // MacsBug entry detector — first-touch + range-based.  We want
            // sim to exit immediately the moment the CPU enters the MacsBug
            // monitor code.  The previous exact-PC list missed the actual
            // entry PCs we observed on HW (0x4084a840 rom_monitor_poll,
            // 0x4084afa0 rom_scc_rx_poll, 0x4084a966, 0x4084afca
            // rom_scc_rx_return).  MacsBug code on the Q700 ROM occupies
            //   * 0x4084a000..0x4084b000   monitor_*, rom_monitor_*,
            //                              rom_scc_rx_*  (real MacsBug)
            //   * 0x40846c02 (single)      diag_fail_to_monitor outlier
            //
            // 2026-05-03 narrowed the lower bound from 0x40849b00 to
            // 0x4084a000.  The 0x40849xxx region (e.g. 0x40849c24) is
            // normal ROM code that MAME also reaches during the boot
            // path — flagging it as MacsBug entry produced false
            // positives in tb-fpga-top-rom-monitor-guard.  Real
            // MacsBug entry PCs all live at 0x4084a000+.
            const bool in_macsbug_range =
                (boundary_pc >= 0x4084a000u && boundary_pc < 0x4084b000u);
            const bool in_diag_fail = (boundary_pc == 0x40846c02u);
            // Q700 ROM lands MacsBug entry at 0x4084b0e4 on the
            // checksum-fast,chime-skip,meminit-fast path before falling into
            // the rom_scc_rx_* poll.  Recognise that extended range plus the
            // SCC TX-empty poll at 0x408be2c2..0x408be2ca so the RX driver
            // knows to start once the CPU is sitting in MacsBug code.
            const bool in_macsbug_ext_range =
                (boundary_pc >= 0x4084b000u && boundary_pc < 0x4084b800u);
            const bool in_macsbug_scc_tx_range =
                (boundary_pc >= 0x408be100u && boundary_pc < 0x408be400u);
            if (!macsbug_seen) {
                const bool override_pc =
                    scc_rx_inject_at_set && boundary_pc == scc_rx_inject_at;
                // Require that a non-trivial number of instructions have
                // already retired before considering this a real MacsBug
                // entry — otherwise the dbg_boundary_pc latch may surface
                // stale ROB values during the pre-reset / patch-set
                // bootstrap window.  The Q700 ROM cannot reach MacsBug in
                // fewer than ~50K retired uops on the fastdiag path.
                const bool past_bootstrap = retired >= 50000u;
                if (past_bootstrap &&
                    (override_pc || in_macsbug_range || in_diag_fail ||
                     in_macsbug_ext_range || in_macsbug_scc_tx_range)) {
                    macsbug_seen = true;
                    macsbug_seen_time = sim_time;
                    std::fprintf(stderr,
                        "[scc-rx] t=%llu MacsBug entry detected at pc=0x%08x (%s) retired=%llu\n",
                        (unsigned long long)sim_time, boundary_pc,
                        macsbug_pc_kind(boundary_pc) ? macsbug_pc_kind(boundary_pc) : "override",
                        (unsigned long long)retired);
                }
            }
            if ((fail_monitor || stop_monitor) &&
                (in_macsbug_range || in_diag_fail)) {
                hit_monitor = true;
                hit_pc = boundary_pc;
                break;
            }

            // ── +nmi_at_pc fire trigger ──
            // Latched inside the boundary_pc handler so we observe the
            // PC the same way the rest of the testbench does.  The fire
            // logic itself runs above (in the per-tick driver block) —
            // we just set a flag that gets picked up next tick.
            if (nmi_at_pc_set && !nmi_fired_pc &&
                boundary_pc == nmi_at_pc) {
                std::fprintf(stderr,
                    "[nmi] PC-trigger hit at boundary_pc=0x%08x "
                    "retired=%llu — arming inst-count fire on next tick\n",
                    boundary_pc, (unsigned long long)retired);
                // Use the inst-count form to actually fire on next tick,
                // by lowering nmi_at_inst to the current retired value.
                nmi_at_inst = retired;
                nmi_at_inst_set = true;
                nmi_fired_pc = true;
            }
            if (irq_at_pc_set && !irq_triggered && boundary_pc == irq_at_pc) {
                irq_triggered = true;
                irq_remaining = irq_inject_total;
                irq_next_retired = retired;
                std::fprintf(stderr,
                    "[irq] t=%llu armed IPL%u count=%llu gap=%llu "
                    "(trigger=pc boundary_pc=0x%08x retired=%llu)\n",
                    (unsigned long long)sim_time, (unsigned)irq_inject_level,
                    (unsigned long long)irq_inject_total,
                    (unsigned long long)irq_inject_gap,
                    boundary_pc, (unsigned long long)retired);
                std::fflush(stderr);
            }

            // +expect_floppy_poll: success = CPU reaches DISK_PROMPT_PC
            // and the polling-loop hit count crosses the threshold.  No
            // MacsBug-range guard here: MAME also dwells in MacsBug-range
            // PCs during normal boot before reaching the floppy prompt,
            // so we let the run continue and let timeout serve as the
            // failure mode.
            if (expect_floppy_poll && boundary_pc == DISK_PROMPT_PC) {
                disk_prompt_hit_count++;
                if (!expect_floppy_poll_satisfied &&
                    disk_prompt_hit_count >= expect_floppy_poll_min_hits) {
                    expect_floppy_poll_satisfied = true;
                    std::fprintf(stderr,
                        "[fpga-rom] +expect_floppy_poll satisfied "
                        "pc=0x%08x hits=%llu retired=%llu\n",
                        boundary_pc,
                        (unsigned long long)disk_prompt_hit_count,
                        (unsigned long long)retired);
                    break;
                }
            }
            // One-shot arch-state dump at the Q700 ROM I/O-discovery routine
            // entry (PC 0x408046aa). The routine reads through
            // `tstb (0,A2,D2:l)` walking Mac I/O slots looking for device
            // descriptors; A2 = address being probed, observed at runtime
            // as 0x50F01C00 (an unmapped Mac I/O slot, returns open-bus
            // 0x00000000/OKAY per MAME-canonical default unmap_value).  This
            // is NOT a bus fault — predecessor names "[busfault]" /
            // "[simm-probe]" were misleading.  Print ONCE per sim run.
            static bool dumped_io_probe = false;
            if (!dumped_io_probe && boundary_pc == 0x408046aau) {
                dumped_io_probe = true;
                std::fprintf(stderr,
                    "[io-discover] PC=0x%08x A0=0x%08x A1=0x%08x A2=0x%08x A3=0x%08x A4=0x%08x A5=0x%08x A6=0x%08x A7=0x%08x\n",
                    boundary_pc,
                    read_arch_reg(8),  read_arch_reg(9),  read_arch_reg(10), read_arch_reg(11),
                    read_arch_reg(12), read_arch_reg(13), read_arch_reg(14), read_arch_reg(15));
                std::fprintf(stderr,
                    "[io-discover]          D0=0x%08x D1=0x%08x D2=0x%08x D3=0x%08x D4=0x%08x D5=0x%08x D6=0x%08x D7=0x%08x\n",
                    read_arch_reg(0), read_arch_reg(1), read_arch_reg(2), read_arch_reg(3),
                    read_arch_reg(4), read_arch_reg(5), read_arch_reg(6), read_arch_reg(7));
                std::fflush(stderr);
            }
        }
        if (retired >= max_retired) break;
        if (stop_floppy_loop) {
            const uint32_t boundary_pc =
                DBGT_REF(dut->rootp, boundary_pc);
            if (boundary_pc >= FLOPPY_LOOP_START &&
                boundary_pc <= FLOPPY_LOOP_END) {
                hit_floppy_loop = true;
                hit_pc = boundary_pc;
                break;
            }
        }

        const uint8_t vs = dut->al9134_vs;
        if (vs && !prev_vs) {
            x = 0;
            y = 0;
        }
        prev_vs = vs;
        if (dut->al9134_de) {
            if (x < FRAME_W && y < FRAME_H)
                frame[y * FRAME_W + x] = dut->al9134_d & 0x00ffffffu;
            x++;
        } else if (x != 0) {
            x = 0;
            if (y < FRAME_H) y++;
        }

        if (sim_time - last_report >= 2000000ULL) {
            last_report = sim_time;
            std::fprintf(stderr,
                "[fpga-rom] t=%llu retired=%llu pc=0x%08x de=%u y=%d\n",
                (unsigned long long)sim_time,
                (unsigned long long)retired,
                (unsigned)DBGT_REF(dut->rootp, pc),
                (unsigned)dut->al9134_de, y);
        }
        // Early exit once we have captured the requested number of TX
        // bytes from BOTH viewpoints (CPU-side + UART pin) — useful for
        // CI smoke runs that just want to assert "MacsBug responded".
        if (scc_rx_inject &&
            scc_tx_count >= scc_tx_max &&
            (scc_tx_uart_log_path.empty() ||
             scc_tx_uart_count >= scc_tx_max)) {
            std::fprintf(stderr,
                "[scc-rx] reached scc_tx_max=%llu on both viewpoints; stopping early\n",
                (unsigned long long)scc_tx_max);
            break;
        }
    }

    // Final checkpoint on the way out (any exit reason: max_insts,
    // timeout, a break-on-condition, or gotFinish()) so the very end of
    // a run is always resumable, not just the last periodic interval.
    if (!checkpoint_dir.empty())
        checkpoint_save(checkpoint_dir, sim_time / 2);

    sd_summary();
    dump_ppm(ppm_path, frame);
    if (hit_floppy_loop) {
        std::fprintf(stderr,
            "[fpga-rom] stop reason=floppy_loop pc=0x%08x range=0x%08x..0x%08x\n",
            hit_pc, FLOPPY_LOOP_START, FLOPPY_LOOP_END);
    }
    if (hit_monitor) {
        std::fprintf(stderr,
            "[fpga-rom] stop reason=rom_monitor pc=0x%08x\n",
            hit_pc);
    }
    if (bad_q700_feature) {
        std::fprintf(stderr,
            "[fpga-rom] stop reason=q700_feature_mismatch got=0x%08x expected=0x%08x entry=0x%08x\n",
            q700_feature, expected_q700_feature, q700_entry);
    } else if (expect_q700_feature && !hit_q700_feature) {
        std::fprintf(stderr,
            "[fpga-rom] stop reason=q700_feature_not_seen expected=0x%08x\n",
            expected_q700_feature);
    }
    std::fprintf(stderr,
        "[fpga-rom] stop t=%llu retired=%llu pc=0x%08x ppm=%s\n",
        (unsigned long long)sim_time,
        (unsigned long long)retired,
#ifdef CPU_M68K040
        (unsigned)g_cpu040_last_pc,
#else
        (unsigned)DBGT_REF(dut->rootp, pc),
#endif
        ppm_path.c_str());
    std::fprintf(stderr,
        "[ifetch-probe] summary: total ifa_arvalid rises=%llu "
        "last_addr=0x%08x\n",
        (unsigned long long)g_ifetch_arvalid_count, g_ifetch_last_addr);

    if (scc_rx_inject || scc_tx_count > 0 || scc_tx_uart_count > 0) {
        std::fprintf(stderr,
            "[scc] macsbug_seen_t=%llu rx_inject_done_t=%llu "
            "rx_bytes=%zu cpu_tx_bytes=%llu uart_tx_bytes=%llu\n",
            (unsigned long long)macsbug_seen_time,
            (unsigned long long)scc_rx_inject_done_time,
            scc_rx_bytes.size(),
            (unsigned long long)scc_tx_count,
            (unsigned long long)scc_tx_uart_count);
        if (scc_tx_count > 0) {
            // Latency from RX inject completion to first CPU-side TX byte.
            const uint64_t rx_done = scc_rx_inject_done_time
                                         ? scc_rx_inject_done_time
                                         : macsbug_seen_time;
            // Convert sim_time → cycles: 2 sim_time = 1 sys_clk = 1 cycle.
            const uint64_t lat_cyc =
                (scc_first_tx_time > rx_done)
                    ? (scc_first_tx_time - rx_done) / 2
                    : 0;
            std::fprintf(stderr,
                "[scc] first_cpu_tx byte=0x%02x ('%c') t=%llu "
                "latency_cycles=%llu\n",
                scc_first_tx_byte,
                (scc_first_tx_byte >= 0x20 && scc_first_tx_byte < 0x7f)
                    ? scc_first_tx_byte : '.',
                (unsigned long long)scc_first_tx_time,
                (unsigned long long)lat_cyc);
        }
        if (scc_tx_uart_count > 0) {
            std::fprintf(stderr,
                "[scc] first_uart_tx byte=0x%02x ('%c') t=%llu\n",
                scc_first_tx_uart_byte,
                (scc_first_tx_uart_byte >= 0x20 &&
                 scc_first_tx_uart_byte < 0x7f)
                    ? scc_first_tx_uart_byte : '.',
                (unsigned long long)scc_first_tx_uart_time);
        }
        // Pretty-print the captured bytes for direct copy/paste into docs.
        auto hexdump = [](const char* label,
                          const std::vector<uint8_t>& v) {
            std::fprintf(stderr, "[scc] %s hex:", label);
            for (uint8_t b : v) std::fprintf(stderr, " %02x", b);
            std::fprintf(stderr, "\n[scc] %s ascii: \"", label);
            for (uint8_t b : v) {
                if (b == '\r') std::fprintf(stderr, "\\r");
                else if (b == '\n') std::fprintf(stderr, "\\n");
                else if (b == '\t') std::fprintf(stderr, "\\t");
                else if (b >= 0x20 && b < 0x7f)
                    std::fputc(static_cast<int>(b), stderr);
                else std::fprintf(stderr, "\\x%02x", b);
            }
            std::fprintf(stderr, "\"\n");
        };
        if (!scc_tx_bytes.empty()) hexdump("cpu_tx", scc_tx_bytes);
        if (!scc_tx_uart_bytes.empty()) hexdump("uart_tx", scc_tx_uart_bytes);
    }
    int rc;
    if (bad_q700_feature)                                       rc = 4;
    else if (expect_q700_feature && !hit_q700_feature)          rc = 5;
    else if (expect_floppy_poll && !expect_floppy_poll_satisfied) rc = 7;
    else if (hit_monitor && fail_monitor)                       rc = 3;
    else if (expect_floppy_poll && expect_floppy_poll_satisfied) rc = 0;
    else if (retired >= max_retired ||
             hit_floppy_loop ||
             (hit_monitor && stop_monitor))                     rc = 0;
    else                                                        rc = 2;
#ifdef SIM_L2C_ENABLE
    std::fprintf(stderr,
        "[rom-fold-probe] summary: checks=%llu mismatches=%llu "
        "mirror_hits=%llu (mirror_hits>0 means real, non-identity "
        "ROM-wraparound coverage was exercised, not just the "
        "addr==expected identity case)\n",
        (unsigned long long)g_rom_fold_checks,
        (unsigned long long)g_rom_fold_mismatches,
        (unsigned long long)g_rom_fold_mirror_hits);
    if (g_rom_fold_mismatches > 0 && rc == 0) rc = 6;
#endif
    if (g_pipe_trace_fp) {
        std::fflush(g_pipe_trace_fp);
        std::fclose(g_pipe_trace_fp);
        std::fprintf(stderr, "[fpga-rom] pipe_trace wrote %llu lines\n",
                     (unsigned long long)g_pipe_trace_emitted);
        g_pipe_trace_fp = nullptr;
    }
    if (g_irq_trace_fp) {
        std::fflush(g_irq_trace_fp);
        std::fclose(g_irq_trace_fp);
        std::fprintf(stderr, "[fpga-rom] irq_trace wrote %llu lines\n",
                     (unsigned long long)g_irq_trace_emitted);
        g_irq_trace_fp = nullptr;
    }
    if (pc_dump_fp) {
        std::fflush(pc_dump_fp);
        std::fclose(pc_dump_fp);
#ifdef CPU_M68K040
        g_pc_dump_fp = nullptr;  // fp now closed; don't let a later tick() write through it
        pc_dump_count = g_pc_dump_count;
#endif
        std::fprintf(stderr, "[fpga-rom] pc_dump wrote %llu entries to %s\n",
                     (unsigned long long)pc_dump_count, pc_dump_path.c_str());
    }
    if (audio_dump_fp) {
        std::fflush(audio_dump_fp);
        std::fclose(audio_dump_fp);
        std::fprintf(stderr,
                     "[fpga-rom] audio_dump wrote %llu sample(s) to %s\n",
                     (unsigned long long)g_audio_dump_count,
                     audio_dump_path.c_str());
    }
    if (asc_event_dump_fp) {
        std::fflush(asc_event_dump_fp);
        std::fclose(asc_event_dump_fp);
        std::fprintf(stderr,
                     "[fpga-rom] asc_event_dump wrote %llu event(s) to %s\n",
                     (unsigned long long)g_asc_event_dump_count,
                     asc_event_dump_path.c_str());
    }
    if (scc_tx_fp) {
        std::fflush(scc_tx_fp);
        std::fclose(scc_tx_fp);
        std::fprintf(stderr, "[fpga-rom] scc_tx_log wrote %llu byte(s) to %s\n",
                     (unsigned long long)scc_tx_count, scc_tx_log_path.c_str());
    }
    if (scc_tx_uart_fp) {
        std::fflush(scc_tx_uart_fp);
        std::fclose(scc_tx_uart_fp);
        std::fprintf(stderr,
                     "[fpga-rom] scc_tx_uart_log wrote %llu byte(s) to %s\n",
                     (unsigned long long)scc_tx_uart_count,
                     scc_tx_uart_log_path.c_str());
    }
    if (axi_log_fp) {
        std::fflush(axi_log_fp);
        std::fclose(axi_log_fp);
        std::fprintf(stderr,
                     "[fpga-rom] axi_lockstep_log wrote %llu event(s) to %s\n",
                     (unsigned long long)axi_log_seq,
                     axi_log_path.c_str());
    }
    if (via1_log_fp) {
        std::fflush(via1_log_fp);
        std::fclose(via1_log_fp);
        std::fprintf(stderr,
                     "[fpga-rom] via1_lockstep_log wrote %llu event(s) to %s\n",
                     (unsigned long long)via1_log_seq,
                     via1_log_path.c_str());
    }
    if (dafb_log_fp) {
        std::fflush(dafb_log_fp);
        std::fclose(dafb_log_fp);
        std::fprintf(stderr,
                     "[fpga-rom] dafb_lockstep_log wrote %llu event(s) to %s\n",
                     (unsigned long long)dafb_log_seq,
                     dafb_log_path.c_str());
    }
    dut->final();
    delete dut;
    return rc;
}
