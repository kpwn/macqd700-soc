// tb_rom_boot.cpp — Quadra 700 ROM cold-boot bring-up harness (task #101)
//
// Standalone harness for running the real Q700 ROM in Verilator against
// mac_top.  Complementary to tb_top.cpp (which owns the asm-test flow);
// this one focuses on real-ROM execution with peripheral stubs good
// enough to not hang the ROM on its early probe loops.
//
// Scope:
//   * Load files/420dbff3.rom (1 MB, Q700 Universal ROM).
//   * Place ROM image at 0x40000000 (real Q700 ROM base) AND mirror
//     the first 256 KB at 0x00000000 while the overlay is asserted
//     (classic Mac reset-time aliasing — VIA1 ORB[3] = 1 on reset,
//     cleared by ROM once init is done).
//   * Reset PC is driven directly into the Q700 ROM's reset vector at
//     0x4000_002A — the Makefile builds this harness with a Verilator
//     `-GRESET_PC=0x4000002A` parameter override on mac_top, so the CPU
//     fetches the real `JMP %pc@(0x4000008c)` ROM prologue on the very
//     first cycle.  No synthetic trampoline is needed; the ROM's reset
//     prologue is 100% PC-relative (task #139).  This matches the FPGA
//     boot path (fpga_top.v + boot_fsm.v with ROM_BASE_ADDR=0x4000_0000).
//   * Stub VIA1 / VIA2 / SCC / SCSI / ASC / RTC over AXI with
//     deterministic responses: most reads return 0x00; VIA1 models
//     enough 6522 port/IFR/IER behavior for the ROM's hardware probe
//     and ADB polling loops to terminate without ROM byte patches.
//   * Match MAME's Quadra overlay exit: the first instruction fetch from
//     the high ROM window lifts the reset overlay, so later low addresses
//     decode as RAM.  VIA1 ORB/DDRB writes may also clear the overlay
//     before that point, but the overlay is one-way after reset.
//   * Log every committed instruction (PC, IR, CCR) to
//     build/sim/rom_boot_trace.log.  Intended to diff against a MAME
//     macqd700 trace via tools/rom_trace_diff.py.
//   * Termination: bra. loop (stuck PC), illegal/exception landing,
//     inst-budget or cycle-budget exhaustion, configured stop/end PC, or
//     sentinel write to 0xFFFF_0000 (same convention as tb_top.cpp).
//
// Forecast per task spec: we do NOT expect a full boot.  The goal is
// to capture the first divergence-from-MAME with enough context to
// file surgical follow-up tasks.  500..5000 committed instructions
// is the aspirational window.
//
// CLI:
//   +rom=<path>         override ROM path  (default: files/420dbff3.rom)
//   +trace=<path>       trace output file  (default: build/sim/rom_boot_trace.log)
//   +periph_log=<path>   capped peripheral access log (off by default)
//   +periph_log_limit=<n>
//                       max detailed peripheral log lines (default: 128)
//   +periph_event_log=<path>
//                       capped peripheral model event log (off by default)
//   +periph_event_log_limit=<n>
//                       max detailed event lines (default: 256, 0=summary only)
//   +periph_event_filter=A,B
//                       only record/watch selected model event categories
//                       (e.g. VIA1,ADB,VBL,RTC,SCSI)
//   +periph_event_summary
//                       print end-of-run model event summary without detail log
//   +probe_log=<path>   capped ROM frontier register/memory probe log
//                       (off by default)
//   +probe_log_limit=<n>
//                       max probe log lines (default: 1024)
//   +data_watch=<spec>  watch comma-separated data addresses/ranges after
//                       translation; token forms: addr, start-end,
//                       start..end, start+len.  Each hit also reports the
//                       LSU effective-address context when available.
//   +data_watch_log=<path>
//                       write capped data watch log (stderr if omitted)
//   +data_watch_limit=<n>
//                       max watch log lines (default: 1024, 0=summary only)
//   +cache_event_log=<path>
//                       write capped cache-maint/snoop event log
//   +cache_event_log_limit=<n>
//                       max cache-event detail lines (default: 256, 0=summary only)
//   +lastn_trace=<n>    dump last n per-cycle state samples at termination
//   +lastn_trace_path=<path>
//                       write last-N trace to path instead of stderr
//   +timeout=<n>        cycle budget       (default: 5_000_000)
//   +max_insts=<n>      committed-insn cap (default: 5_000)
//   +q700_ram=<bytes|nK|nM>
//                       installed low RAM visible to the ROM sizing probes
//                       (default: MemModel::RAM_WINDOW_DEFAULT; use 4M to match MAME)
//   +save_state=<path>  save exact model+harness state at stop
//   +restore_state=<p>  restore exact model+harness state before run
//   +arch_checkpoint=<path>
//                       write portable committed architectural checkpoint
//                       plus ROM/RAM image data at stop
//   +arch_replay=<path>
//                       cold-reset the model, load a portable architectural
//                       checkpoint, restore the saved CPU/RAM state, restore
//                       only the explicitly recorded subset of IO/peripheral
//                       state, and resume at the checkpoint's saved next PC
//   +arch_sample_every=<n>
//                       write committed architectural state every n commits
//   +arch_sample_log=<path>
//                       destination for +arch_sample_every state samples
//   +checkpoint_prefix=<p> +checkpoint_every=<n>
//                       save periodic checkpoints as <p>.<committed>.vlt
//   +checkpoint_points=a,b,c
//                       save checkpoints at exact committed-uop depths
//   +checkpoint_cycle_points=a,b,c
//                       save checkpoints at exact absolute sim cycles
//   +stop_cycle=<n>     absolute sim-cycle stop, useful after restore
//   +stop_pc=a,b,c      stop on a macro-instruction retire boundary when
//                       the boundary PC matches any listed PC
//   +stop_pc_hit=<n>    stop on the Nth hit for a configured stop PC (default: 1)
//   +stop_on_sad_mac    stop when the ROM reaches the Sad Mac diagnostic entry
//   +stop_on_disk_prompt
//                       stop when the ROM reaches the no-boot-disk polling loop
//   +disk_prompt_hit=<n>
//                       stop on the Nth no-disk polling-loop hit (default: 32)
//   +stop_on_rom_outcome
//                       shorthand for +stop_on_sad_mac +stop_on_disk_prompt
//   +end_pc=a,b,c       end run on a macro-instruction retire boundary when
//                       the boundary PC matches any listed PC
//   +end_pc_hit=<n>     end on the Nth hit for a configured end PC (default: 1)
//   +stop_on_exc=a,b,c  stop on a precise exception-entry boundary when the
//                       completed handler-entry vector matches any listed
//                       exception vector (decimal or 0x-prefixed); known
//                       ROM fault vectors are named in the stop summary
//   +stop_on_exc_after_committed=<n>
//                       ignore +stop_on_exc hits until committed >= n
//   +stop_on_illegal    shorthand for +stop_on_exc=4
//   +stop_on_rom_faults shorthand for +stop_on_exc=2,4,11
//   +stop_on_ifetch_berr
//                       stop when instruction fetch asks for an unmapped line
//   +stuck_pc_threshold=<n>
//                       stop after n same-PC repeat commits (default: 8192;
//                       0 disables this watchdog)
//   +no_progress_cycles=<n>
//                       stop after n cycles with no internal progress
//                       (default: 500000; 0 disables this watchdog)
//   +end_on_exc=a,b,c   like +stop_on_exc, but classify the reason as an
//                       intentional terminal ender for scripted frontiers
//   +end_on_exc_after_committed=<n>
//                       ignore +end_on_exc hits until committed >= n
//   +end_on_illegal     shorthand for +end_on_exc=4
//   +end_on_rom_faults  shorthand for +end_on_exc=2,4,11
//   +end_on_ifetch_berr
//                       classify an unmapped instruction fetch as end-ifetch-berr
//   +end_on_stuck_pc    classify the stuck-PC watchdog as end-stuck-pc
//   +end_on_no_progress classify no-progress watchdog as end-no-progress
//   +fault_dump_dir=<path>
//                       opt-in bounded code/RAM windows at termination
//   +fault_dump_bytes=<n>
//                       bytes per fault window (default: 128, max: 4096)
//   +rom_patch=<name[,name]>
//                       apply named harness-local ROM patches after load/restore
//                       (supported: checksum-fast, meminit-fast,
//                        ramtest-mame-state, rtc-pram-mame-state,
//                        adb-init-wait, mame-firstlight,
//                        clean-fastdiag/diag-loops, chime-delay,
//                        timer-delay, scc-delay; legacy unsafe:
//                        checksum-unsafe, diag-loops-unsafe)
//   +stop_on_low_pc     stop if post-overlay fetch PC drops below 0x10000000
//   +strict_overlay     apply low-memory ROM overlay to data-side accesses too
//                       and require VIA1 overlay-clear writes (no high-ROM
//                       fetch auto-clear shortcut)
//   +strict_axi_resp    return decode/slave errors for invalid DAXI accesses
//                       instead of unconditional OKAY open-bus responses
//   +via1_timer_div=<n> host cycles per VIA1 Timer1 tick
//                       (default: 128 ~= 100 MHz / 783.36 kHz)
//   +via1_fast_timing   shorthand for +via1_timer_div=1 (legacy accelerated)
//   +via1_t1_selftest   run a directed VIA1 Timer1 stub selftest and exit
//   +harness_strictness_selftest
//                       run directed strict-overlay/AXI-response checks and exit
//   +rtc_sidechannel_selftest
//                       run a directed VIA1 PB0/PB1/PB2 RTC selftest and exit
//   +adb_shadow_selftest
//                       run a directed VIA1 SR/ADB shadow selftest and exit
//   +scsi_window_selftest
//                       run a directed TurboSCSI window event-log selftest and exit
//   +display_watch_selftest
//                       run a directed DAFB register/VRAM watch selftest and exit
//   +stop_on_exc_selftest
//                       run a directed boundary-based stop_on_exc consumer
//                       selftest and exit
//   +watchdog_selftest  run directed terminal-watchdog helper selftest and exit
//   +waves              dump FST waveform  (waveform path: rom_boot.fst)
//   +no_waves           disable waveform dumping even if compiled with WAVES
//   +verbose            noisy AXI / VIA decode prints
//
// Build: make tb-rom-boot  — see Makefile.

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <cassert>
#include <cctype>
#include <cerrno>
#include <exception>
#include <map>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <utility>
#include <vector>
#include <verilated.h>
#include <verilated_fst_c.h>
#include <verilated_save.h>
#include "Vmac_top.h"
#include "Vmac_top___024root.h"
#include "mem_model.h"
#include "periph_event_log.h"
#include "rom_patch_sets.h"

// ─── Globals ──────────────────────────────────────────────────────────
static Vmac_top*        dut   = nullptr;
static VerilatedFstC*   fst   = nullptr;
static MemModel*        mem   = nullptr;
static uint64_t         sim_time = 0;
static uint64_t         timeout_cycles = 5000000ULL;
static uint64_t         stop_cycle = 0;
static uint64_t         absolute_stop_cycle = 0;
static uint64_t         max_insts = 5000;
static uint32_t         visible_ram_size = MemModel::RAM_WINDOW_DEFAULT;
static bool             waves = false;
static bool             no_waves = false;
static bool             verbose = false;
static bool             stop_on_low_pc = false;
static bool             strict_overlay = false;
static bool             strict_axi_resp = false;
static bool             stop_on_ifetch_berr = false;
static bool             end_on_ifetch_berr = false;
static bool             stop_on_sad_mac = false;
static bool             stop_on_disk_prompt = false;
static bool             end_on_stuck_pc = false;
static bool             end_on_no_progress = false;
static std::vector<uint8_t> stop_exc_vecs;
static std::vector<uint8_t> end_exc_vecs;
static uint64_t         stop_on_exc_after_committed = 0;
static uint64_t         end_on_exc_after_committed = 0;
static std::string      rom_path   = "files/420dbff3.rom";
static std::string      trace_path = "build/sim/rom_boot_trace.log";
static std::string      periph_log_path;
static std::string      periph_event_log_path;
static std::string      lastn_trace_path;
static std::string      probe_log_path;
static std::string      data_watch_log_path;
static std::string      cache_event_log_path;
static std::string      fault_dump_dir;
static std::string      save_state_path;
static std::string      restore_state_path;
static std::string      arch_checkpoint_path;
static std::string      arch_replay_path;
static std::string      arch_sample_log_path;
static std::string      checkpoint_prefix;
static std::vector<std::string> requested_rom_patch_sets;
static bool             ramtest_mame_state_fastfill = false;
static uint64_t         ramtest_mame_state_fastfill_hits = 0;
static uint64_t         ramtest_mame_state_fastfill_skips = 0;
static uint64_t         ramtest_mame_state_fastfill_bytes = 0;
static uint64_t         ramtest_mame_state_fastfill_sentinels = 0;
static std::vector<std::string> periph_event_filter;
static uint64_t         lastn_trace_cycles = 0;
static uint64_t         periph_log_limit = 128;
static uint64_t         periph_event_log_limit = 256;
static uint64_t         probe_log_limit = 1024;
static uint64_t         probe_log_emitted = 0;
static uint64_t         data_watch_limit = 1024;
static uint64_t         cache_event_log_limit = 256;
static uint64_t         daxi_ar_reqs = 0;
static uint64_t         daxi_ar_burst_reqs = 0;
static uint64_t         daxi_ar_burst_beats = 0;
static uint64_t         disk_prompt_hit_target = 32;
static uint64_t         disk_prompt_hits = 0;
static uint64_t         fault_dump_bytes = 128;
static uint64_t         arch_checkpoint_flush_timeout = 20000;
static uint64_t         arch_sample_every = 0;
static uint64_t         next_arch_sample_commit = 0;
static bool             arch_sample_pending = false;
static uint64_t         arch_sample_pending_commit = 0;
static std::map<uint32_t, uint64_t> probe_pc_hits;
static uint64_t         checkpoint_every = 0;
static uint64_t         next_checkpoint_commit = 0;
static std::vector<uint64_t> checkpoint_points;
static std::vector<uint64_t> checkpoint_cycle_points;
static size_t           next_checkpoint_point = 0;
static size_t           next_checkpoint_cycle_point = 0;
static FILE*            trace_fp   = nullptr;
static FILE*            periph_fp  = nullptr;
static FILE*            probe_fp   = nullptr;
static FILE*            arch_sample_fp = nullptr;
static FILE*            data_watch_fp = nullptr;
static FILE*            cache_event_fp = nullptr;
static PeriphEventLog   periph_events;
static bool             arch_next_pc_valid = false;
static uint32_t         arch_next_pc = 0;
static uint32_t         arch_next_pc_commit_pc = 0;
static uint32_t         arch_next_pc_committed = 0;
static bool             init_periph_event_logger();
static bool             init_cache_event_logger();
static uint32_t         io_read32(uint32_t a);
static void             io_write32(uint32_t a, uint32_t data, uint32_t strb);
static uint32_t         daxi_read32(uint32_t a);
static void             daxi_write32(uint32_t a, uint32_t data, uint32_t strb);
static void             via1_write_reg(uint8_t idx, uint8_t v);
static uint8_t          via1_read_reg(uint8_t idx);
static bool             via1_overlay_active(void);
static void             drive_daxi();
static void             tick();
static bool             check_stuck_pc_commit(uint32_t pc);
static bool             check_no_progress_watchdog(uint64_t no_progress_cycles);
static bool             is_dafb_reg_addr(uint32_t addr);
static bool             is_dafb_vram_addr(uint32_t addr);
static void             maybe_clear_overlay_from_high_rom_fetch(uint32_t addr);

// Overlay flop: ROM shadowed at low addresses when overlay=1.
// Reset value matches real Q700 (VIA1 ORB[3] reads 1 at reset).
static bool             overlay = true;

// In-TB ROM buffer — keep ROM bytes separate from MemModel so the harness
// can enforce Q700-specific mirror/overlay fetch behavior explicitly.
// (MemModel has a 4 MiB ROM window at 0x40000000, but ROM boot uses a
// private 1 MiB image with overlay semantics tracked in this harness.)
static std::vector<uint8_t> rom_img;

// Q700 ROM size + base addresses.
static constexpr uint32_t Q700_ROM_BASE   = 0x40000000u;
static constexpr uint32_t Q700_ROM_SIZE   = 0x00100000u;  // 1 MB
static constexpr uint32_t Q700_ROM_ENTRY  = 0x0000002Au;  // ROM[4..7]
// Q700_RESET_PC is the absolute address the CPU resets to — it equals
// Q700_ROM_BASE + Q700_ROM_ENTRY so the CPU fetches the real ROM
// prologue on cycle 0.  Matches mac_top's -GRESET_PC elaboration
// override in the Makefile (task #139).
static constexpr uint32_t Q700_RESET_PC   = Q700_ROM_BASE + Q700_ROM_ENTRY;
static constexpr uint32_t OVERLAY_MIRROR  = 0x00000000u;
static constexpr uint32_t OVERLAY_BYTES   = 0x00040000u;  // 256 KB mirror
static constexpr uint64_t VIA1_TIMER_DIV_REALISTIC = 128; // ~= 100 MHz / 783.36 kHz
static constexpr uint64_t VIA1_TIMER_DIV_FAST      = 1;

static constexpr uint32_t AXI_RESP_OKAY   = 0u;
static constexpr uint32_t AXI_RESP_SLVERR = 2u;
static constexpr uint32_t AXI_RESP_DECERR = 3u;

// Mac Universal ROM header layout: bytes [0..3] = stored checksum
// (big-endian; also the image's MAME dump name on disk), [4..7] =
// reset-vector entry offset.  For the Q700 Universal ROM this is
// 0x420dbff3 / 0x0000002a.  tb_rom_boot is tuned for Q700 — a
// different chipset ROM won't boot against our peripheral stubs,
// so we warn loudly at load time rather than silently fail in the
// middle of an incompatible ROM.
static constexpr uint32_t Q700_ROM_CHECKSUM = 0x420dbff3u;
static constexpr uint32_t Q700_RAM_DECODE_LIMIT = MemModel::RAM_DECODE_SIZE; // 1 GiB low decode

// Peripheral address space (Q700 layout).  Source of truth: MAME
// src/mame/apple/macquadra700.cpp.  Q700 devices are decoded at
// 0x5000_xxxx with a 0x00fc_0000 mirror mask, so the ROM-visible
// 0x50f0_xxxx aliases are the same devices.  Example:
//   0x50f0_0000 -> 0x5000_0000 VIA1
//   0x50f0_c020 -> 0x5000_c020 SCC
//   0x50f8_1c00 -> 0x5000_1c00 VIA1 IER mirror
//
// Crucially, 0x50f0_4000 canonicalizes to 0x5000_4000, which MAME's
// Q700 map does NOT assign to SCC.  Leaving that address unmapped is
// intentional; if the ROM is polling it, the harness selected the wrong
// ROM hardware descriptor rather than discovering a missing Q700 SCC
// alias.
static constexpr uint32_t IO_BASE          = 0x50000000u;
static constexpr uint32_t IO_SIZE          = 0x01000000u; // 0x5000_0000..0x50ff_ffff mirrors
static constexpr uint32_t IO_MIRROR_MASK   = 0x00FC0000u;
static constexpr uint32_t VIA1_BASE        = 0x50000000u;
static constexpr uint32_t VIA1_SIZE        = 0x00002000u; // 16 regs × 0x200 stride
static constexpr uint32_t VIA2_BASE        = 0x50002000u;
static constexpr uint32_t VIA2_SIZE        = 0x00002000u;
static constexpr uint32_t ENET_BASE        = 0x50008000u;
static constexpr uint32_t ENET_SIZE        = 0x00000008u;
static constexpr uint32_t SONIC_BASE       = 0x5000A000u;
static constexpr uint32_t SONIC_SIZE       = 0x00001100u;
static constexpr uint32_t SCC_BASE         = 0x5000C000u;
static constexpr uint32_t SCC_SIZE         = 0x00002000u;
static constexpr uint32_t ORWELL_BASE      = 0x5000E000u;
static constexpr uint32_t ORWELL_SIZE      = 0x00000100u;
static constexpr uint32_t TURBOSCSI_BASE   = 0x5000F000u;
static constexpr uint32_t TURBOSCSI_SIZE   = 0x00000102u;
static constexpr uint32_t ASC_BASE         = 0x50014000u;
static constexpr uint32_t ASC_SIZE         = 0x00002000u;
static constexpr uint32_t SWIM_BASE        = 0x5001E000u;
static constexpr uint32_t SWIM_SIZE        = 0x00002000u;
static constexpr uint32_t SWIM_REG_SHIFT   = 9;
static constexpr uint32_t SWIM_REG_MASK    = 0x0Fu;

static inline uint32_t q700_io_canonical(uint32_t a) {
    return a & ~IO_MIRROR_MASK;
}

static inline bool in_range(uint32_t a, uint32_t base, uint32_t size) {
    return a >= base && a < (base + size);
}

static inline bool overlay_rom_for_ifetch(uint32_t a) {
    return overlay && a < OVERLAY_BYTES;
}

static inline bool overlay_rom_for_data(uint32_t a) {
    return strict_overlay && overlay && a < OVERLAY_BYTES;
}

static inline bool io_addr_backed(uint32_t a) {
    const uint32_t ca = q700_io_canonical(a);
    return in_range(ca, VIA1_BASE, VIA1_SIZE) ||
           in_range(ca, VIA2_BASE, VIA2_SIZE) ||
           in_range(ca, ENET_BASE, ENET_SIZE) ||
           in_range(ca, SONIC_BASE, SONIC_SIZE) ||
           in_range(ca, SCC_BASE, SCC_SIZE) ||
           in_range(ca, ORWELL_BASE, ORWELL_SIZE) ||
           in_range(ca, TURBOSCSI_BASE, TURBOSCSI_SIZE) ||
           in_range(ca, ASC_BASE, ASC_SIZE) ||
           in_range(ca, SWIM_BASE, SWIM_SIZE);
}

static inline uint32_t daxi_resp_for_access(uint32_t a, bool write) {
    const uint32_t aa = a & ~0x3u;
    if (!strict_axi_resp) return AXI_RESP_OKAY;
    if (overlay_rom_for_data(aa)) return write ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
    if (aa < visible_ram_size) return AXI_RESP_OKAY;
    if (aa >= Q700_ROM_BASE && aa < 0x50000000u)
        return write ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
    if (in_range(aa, IO_BASE, IO_SIZE))
        return io_addr_backed(aa) ? AXI_RESP_OKAY : AXI_RESP_DECERR;
    if (is_dafb_vram_addr(aa)) return AXI_RESP_OKAY;
    if (is_dafb_reg_addr(aa)) return AXI_RESP_DECERR;
    return AXI_RESP_DECERR;
}

static inline uint32_t daxi_rresp_for_addr(uint32_t a) {
    return daxi_resp_for_access(a, false);
}

static inline uint32_t daxi_bresp_for_addr(uint32_t a) {
    return daxi_resp_for_access(a, true);
}

// VIA1 minimal state — enough so that ROM probes don't hang.  VIA1 has
// 16 byte registers at 512-byte stride on Q700 (A9..A12 select reg).
// We key on (addr >> 9) & 0xF.  Reset values come from peripheral_arch.md
// §"VIA1 minimum viable" + the macquadra700 ROM bring-up observations.
//
// Task #152: added an "empty-bus" ADB shadow responder.  When the ROM
// bit-bangs an ADB command through SR (reg 10), we simulate no devices
// present by: (a) latching the shifted byte, (b) scheduling an SR
// interrupt-flag assertion in IFR.SR (bit 2) a few hundred cycles later,
// (c) returning 0xFF on every SR read during the response window.  ROM
// reads 0xFF as "no device at this address" and walks the ADB bus; the
// outer ADB-enum state machine eventually gives up and we escape into
// post-ADB code.  This replaces the earlier three ROM-byte NOP patches
// from task #149 (which forced the scan-loop exits directly).
struct Via1 {
    uint8_t  orb   = 0x80;   // output latch
    uint8_t  ora   = 0x00;
    uint8_t  ddrb  = 0x00;
    uint8_t  ddra  = 0x00;
    uint8_t  t1cl  = 0x00;
    uint8_t  t1ch  = 0x00;
    uint8_t  t1ll  = 0x00;
    uint8_t  t1lh  = 0x00;
    uint8_t  t2cl  = 0x00;
    uint8_t  t2ch  = 0x00;
    uint8_t  sr    = 0xFF;   // empty-bus idle: SR reads as all-1s
    uint8_t  acr   = 0x00;
    uint8_t  pcr   = 0x00;
    uint8_t  ifr   = 0x00;   // no interrupts pending — bit 7 summary auto
    uint8_t  ier   = 0x80;   // bit 7 read-back convention (write 1xxx = set)

    // Timer 1 minimal state (VBL poll).  The ROM eventually programs T1 and
    // polls IFR bit 6 for wraparound.  In the ROM-boot harness we model a
    // simple countdown so the poll can terminate, without trying to match
    // exact Q700 cycle timing.
    bool     t1_running = false;
    uint16_t t1_latch   = 0;
    uint16_t t1_counter = 0;
    bool     t2_running = false;
    uint16_t t2_latch   = 0;
    uint16_t t2_counter = 0;

    // ADB "empty-bus" shadow state machine.  The Mac ADB driver writes
    // the command byte into SR, configures PCR for output-shift, and
    // polls IFR.SR (bit 2) for "shift complete."  We fire the flag a
    // fixed delay after the SR write so real-silicon timing polls see
    // plausible latency rather than an instant ack (the ROM sometimes
    // does `tstb; beq .` style tight poll loops that want ≥ few-cycle
    // latency before seeing the flag).  On the response phase the ROM
    // switches PCR to input-shift and reads SR; we return 0xFF which
    // means "no device acked this address."
    bool     adb_shift_in_progress   = false;
    uint64_t adb_shift_complete_cyc  = 0;
    uint8_t  adb_last_byte           = 0;
    uint32_t adb_transaction_id      = 0;
} via1;

// Host cycle counter — ticked by tick().  Used by the ADB shadow to
// schedule deferred IFR.SR assertion.
static uint64_t host_cycle = 0;
// Delay (in host sim cycles) between an SR write and IFR.SR-bit
// assertion.  200 cycles is plenty to pass any real-silicon "give the
// shift register time to clock out" poll loop the ROM might have,
// without wasting cycles.  (At 200 MHz this is 1 µs — ADB bit clock
// is ~100 µs, so this is fast but still non-zero.)
static constexpr uint64_t ADB_SHIFT_DELAY_CYCLES = 200;
static constexpr uint32_t SCSI_RAW_SD_LBA_BIAS = 8192u;
static constexpr uint32_t SCSI_RAW_SD_RESERVED_BYTES =
    SCSI_RAW_SD_LBA_BIAS * 512u;
static constexpr uint32_t DAFB_REG_BASE = 0xF9800000u;
static constexpr uint32_t DAFB_REG_SIZE = 0x00001000u;
static constexpr uint32_t DAFB_VRAM_BASE = MemModel::VRAM_BASE;
static constexpr uint32_t DAFB_VRAM_SIZE = MemModel::VRAM_SIZE;

static bool is_dafb_reg_addr(uint32_t addr) {
    return addr >= DAFB_REG_BASE && addr < (DAFB_REG_BASE + DAFB_REG_SIZE);
}

static bool is_dafb_vram_addr(uint32_t addr) {
    return addr >= DAFB_VRAM_BASE && addr < (DAFB_VRAM_BASE + DAFB_VRAM_SIZE);
}

// Host cycles per VIA timer tick.  Default tracks the current 100 MHz FPGA
// bring-up target against the Q700's ~783.36 kHz phi2 domain; opt into the
// old accelerated model with +via1_fast_timing or +via1_timer_div=1.
static uint64_t via1_timer_div = VIA1_TIMER_DIV_REALISTIC;
static uint64_t via1_timer_div_ctr = 0;
static bool     via1_t1_selftest = false;
static bool     harness_strictness_selftest = false;
static bool     rtc_sidechannel_selftest = false;
static bool     adb_shadow_selftest = false;
static bool     scc_asc_selftest = false;
static bool     scsi_window_selftest = false;
static bool     display_watch_selftest = false;
static bool     stop_on_exc_selftest = false;
static bool     watchdog_selftest = false;
static bool     rom_outcome_selftest = false;

struct TurboScsiTrace {
    std::array<uint8_t, 16> cdb{};
    std::array<uint8_t, 0x100> regs{};
    uint8_t cdb_idx = 0;
    uint8_t cdb_len = 0;
    uint8_t last_command = 0;
    bool irq_pending = false;
    bool drq_pending = false;
    bool end_dma_pending = false;
    bool busy_error_pending = false;
    bool selected = false;
    bool command_active = false;
    uint8_t current_data = 0x00;
    uint8_t data_phase = 0x00;

    void reset() {
        cdb.fill(0);
        regs.fill(0);
        cdb_idx = 0;
        cdb_len = 0;
        last_command = 0;
        irq_pending = false;
        drq_pending = false;
        end_dma_pending = false;
        busy_error_pending = false;
        selected = false;
        command_active = false;
        current_data = 0x00;
        data_phase = 0x00;
    }
} turboscsi_trace;

struct Via2 {
    // Q700 VIA2 — most reads return 0 so the slot-IRQ scan says "no IRQ".
    // ORB reads: bit0 clear = NO slot IRQ pending (Q700 ROM polls this).
    uint8_t regs[16] = {};
} via2;

struct AscStub {
    // The standalone RTL ASC reports the SONORA ID, but the ROM-boot
    // harness follows the MAME Q700 trace: the Universal ROM tests
    // +0x800 at 0x408be186 and takes the classic zero-ID init path.
    // Returning 0x0B here diverts into an old descriptor/probe loop.
    static constexpr uint8_t VERSION_ROM_BOOT = 0x00;
    static constexpr uint8_t RATE_DEFAULT   = 45;

    uint8_t mode       = 0x00;
    uint8_t channel    = 0x00;
    uint8_t fifo_ctl   = 0x00;
    uint8_t irq_status = 0x00;
    uint8_t rate       = RATE_DEFAULT;
    uint8_t volume     = 0xFF;
    uint8_t clock      = 0x00;
    uint8_t fifo_a_irq_ctl = 0x00;
    uint8_t fifo_b_irq_ctl = 0x00;
    uint8_t fifo_a_status = 0x00;
    uint8_t fifo_b_status = 0x00;
    uint32_t fifo_a_writes = 0;
    uint32_t fifo_b_writes = 0;
    uint8_t wavetable[16] = {};

    uint8_t read(uint32_t off) {
        off &= 0xFFFu;
        if (off < 0x800u)
            return 0x00; // FIFOs are CPU-write / chip-read.

        switch (off) {
            case 0x800: return VERSION_ROM_BOOT;
            case 0x801: return mode;
            case 0x802: return channel;
            case 0x803: return fifo_ctl;
            case 0x804: {
                uint8_t v = irq_status;
                irq_status = 0x00;
                fifo_a_status &= 0x7F;
                fifo_b_status &= 0x7F;
                return v;
            }
            case 0x806: return volume;
            case 0x807: return clock;
            case 0x808: return rate;
            case 0x80A: return volume;
            case 0x810: {
                uint8_t v = fifo_a_status;
                fifo_a_status &= 0x7F;
                irq_status &= 0x7F;
                return v;
            }
            case 0x811: {
                uint8_t v = fifo_b_status;
                fifo_b_status &= 0x7F;
                irq_status &= 0xBF;
                return v;
            }
            default:
                if (off == 0xF09u)
                    return fifo_a_irq_ctl;
                if (off == 0xF29u)
                    return fifo_b_irq_ctl;
                if (off >= 0x830u && off <= 0x83Fu)
                    return wavetable[off & 0x0Fu];
                return 0x00;
        }
    }

    void write(uint32_t off, uint8_t v) {
        off &= 0xFFFu;
        if (off < 0x400u) {
            fifo_a_writes++;
            return; // FIFO data writes are accepted and drained silently.
        }
        if (off < 0x800u) {
            fifo_b_writes++;
            return;
        }

        switch (off) {
            case 0x800:
                break; // Version is read-only.
            case 0x801:
                mode = v;
                break;
            case 0x802:
                channel = v;
                break;
            case 0x803:
                fifo_ctl = v;
                if (v & 0x80) {
                    irq_status = 0x00;
                    fifo_a_status = 0x00;
                    fifo_b_status = 0x00;
                    fifo_a_writes = 0;
                    fifo_b_writes = 0;
                }
                break;
            case 0x804:
                irq_status = 0x00;
                fifo_a_status &= 0x7F;
                fifo_b_status &= 0x7F;
                break;
            case 0x806:
                volume = v;
                break;
            case 0x807:
                clock = v;
                rate = ((v & 0x03u) == 0x03u) ? 23 : RATE_DEFAULT;
                break;
            case 0x808:
                rate = v ? v : 1;
                break;
            case 0x80A:
                volume = v;
                break;
            case 0x810:
                fifo_a_status &= 0x7F;
                irq_status &= 0x7F;
                break;
            case 0x811:
                fifo_b_status &= 0x7F;
                irq_status &= 0xBF;
                break;
            default:
                if (off == 0xF09u) {
                    fifo_a_irq_ctl = v & 0x01u;
                    break;
                }
                if (off == 0xF29u) {
                    fifo_b_irq_ctl = v & 0x01u;
                    break;
                }
                if (off >= 0x830u && off <= 0x83Fu)
                    wavetable[off & 0x0Fu] = v;
                break;
        }
    }
} asc;

struct SccStub {
    static constexpr uint8_t RR0_IDLE = 0x6C; // TX empty, DCD/CTS high, no RX byte.

    uint8_t ptr[2] = {0x00, 0x00}; // 0=B, 1=A
    uint8_t wr[2][16] = {};
    uint8_t last_tx[2] = {0x00, 0x00};

    static uint8_t channel_index(uint32_t off) {
        return (uint8_t)(((off >> 3) & 0x1u) ? 1u : 0u);
    }

    static bool is_data_port(uint32_t off) {
        return ((off >> 2) & 0x1u) != 0;
    }

    uint8_t rr(uint8_t ch, uint8_t reg) const {
        switch (reg & 0x0Fu) {
            case 0: return RR0_IDLE;
            case 1: return 0x00;
            case 2: return wr[0][2]; // Shared interrupt vector.
            case 3: return ch ? 0x00 : 0x00;
            case 4: return wr[ch][4];
            case 5: return wr[ch][5];
            case 8: return 0x00;     // No attached serial source.
            case 9: return wr[0][9]; // Shared master interrupt register.
            case 11: return wr[ch][11];
            case 12: return wr[ch][12];
            case 13: return wr[ch][13];
            case 15: return wr[ch][15];
            default: return 0x00;
        }
    }

    uint8_t read(uint32_t off) {
        uint8_t ch = channel_index(off);
        if (is_data_port(off))
            return 0x00;
        uint8_t reg = ptr[ch] & 0x0Fu;
        uint8_t value = rr(ch, reg);
        if (reg != 0)
            ptr[ch] = 0;
        return value;
    }

    void write(uint32_t off, uint8_t value) {
        uint8_t ch = channel_index(off);
        if (is_data_port(off)) {
            wr[ch][8] = value;
            last_tx[ch] = value;
            return;
        }

        uint8_t reg = ptr[ch] & 0x0Fu;
        if (reg == 0) {
            wr[ch][0] = value;
            if (value == 0) {
                ptr[ch] = 0;
            } else if (((value >> 3) & 0x07u) == 0x01u) {
                ptr[ch] = 0x08u | (value & 0x07u);
            } else if (((value >> 3) & 0x07u) == 0x00u) {
                ptr[ch] = value & 0x07u;
            }
            return;
        }

        wr[ch][reg] = value;
        if (reg == 2) {
            wr[0][2] = value;
            wr[1][2] = value;
        } else if (reg == 8) {
            last_tx[ch] = value;
        } else if (reg == 9) {
            wr[0][9] = value & 0x3Fu;
            wr[1][9] = value & 0x3Fu;
            if ((value & 0xC0u) != 0) {
                ptr[0] = 0;
                ptr[1] = 0;
            }
        }
        ptr[ch] = 0;
    }
} scc;

// SWIM/IWM probe-safe stub state.  This keeps the ROM out of floppy
// dead-ends without claiming real media transfer, IRQ, or DMA behavior.
struct SwimStub {
    uint8_t data   = 0x00;
    uint8_t mark   = 0x00;
    uint8_t error  = 0x00;
    uint8_t phase  = 0x00;
    uint8_t setup  = 0x00;
    uint8_t mode   = 0x00;
    uint8_t iwm_mode = 0x00;
    uint8_t iwm_status = 0x00;
    uint8_t iwm_control = 0x00;
    uint8_t iwm_whd = 0xbf;
    uint8_t iwm_to_swim_ctr = 0;
    uint8_t param[16] = {};
    uint8_t param_idx = 0;

    void iwm_control_access(uint8_t reg) {
        // MAME swim1_device::iwm_control(): offsets below 8 drive phase
        // lines; offsets 8..15 clear/set control bits by even/odd alias.
        if ((reg & SWIM_REG_MASK) < 8) {
            uint8_t bit = 1u << ((reg & SWIM_REG_MASK) >> 1);
            if (reg & 1) phase |= bit;
            else         phase &= (uint8_t)~bit;
        } else {
            uint8_t bit = 1u << ((reg & SWIM_REG_MASK) >> 1);
            if (reg & 1) iwm_control |= bit;
            else         iwm_control &= (uint8_t)~bit;
        }
    }

    uint8_t iwm_read(uint8_t reg) {
        iwm_control_access(reg);
        iwm_to_swim_ctr = 0;
        switch (iwm_control & 0xc0) {
            case 0x00: return 0xff;                  // idle read data
            case 0x40: return (iwm_status & 0x7f) | 0x80; // no floppy/write-protect
            case 0x80: return iwm_whd;
            case 0xc0: return 0xff;
        }
        return 0xff;
    }

    uint8_t read(uint8_t reg) {
        if ((mode & 0x40) == 0)
            return iwm_read(reg);

        switch (reg & SWIM_REG_MASK) {
            case 0x0: return 0xff;                // IWM read-all-ones
            case 0x1: return 0xff;                // benign no-media data
            case 0x2: {
                uint8_t v = error;
                error = 0;
                return v;
            }
            case 0x3: {
                uint8_t v = param[param_idx];
                param_idx = (param_idx + 1) & 0x0f;
                return v;
            }
            case 0x4: return phase;
            case 0x5: return setup;
            case 0x6: return mode;
            case 0x7: return 0x08;                 // no-media sense
            case 0x8: return 0xff;
            case 0x9: return 0xff;
            case 0xA: {
                uint8_t v = error;
                error = 0;
                return v;
            }
            case 0xB: {
                uint8_t v = param[param_idx];
                param_idx = (param_idx + 1) & 0x0f;
                return v;
            }
            case 0xC: return phase;
            case 0xD: return setup;
            case 0xE: return (mode & 0x7f) | 0x80;    // no-media status
            case 0xF: return 0x08;                 // no-media sense
        }
        return 0xff;
    }

    void write(uint8_t reg, uint8_t v) {
        if ((mode & 0x40) == 0) {
            iwm_control_access(reg);
            // MAME only treats offset 0xf as an IWM mode write after the
            // control select bits are both set.  The Q700 ROM writes 0x17
            // through this path, then expects the status low bits to echo it.
            if ((iwm_control & 0xc0) == 0xc0 && (reg & 1)) {
                iwm_mode = v;
                iwm_status = (iwm_status & 0xe0) | (v & 0x1f);
            }
            if ((reg & SWIM_REG_MASK) == 0x0f) {
                switch (iwm_to_swim_ctr) {
                    case 0: iwm_to_swim_ctr = (v & 0x40) ? 1 : 0; break;
                    case 1: iwm_to_swim_ctr = (v & 0x40) ? 0 : 2; break;
                    case 2: iwm_to_swim_ctr = (v & 0x40) ? 3 : 0; break;
                    case 3:
                        if (v & 0x40) {
                            mode |= 0x40;
                            param_idx = 0;
                        }
                        iwm_to_swim_ctr = 0;
                        break;
                    default:
                        iwm_to_swim_ctr = 0;
                        break;
                }
            } else {
                iwm_to_swim_ctr = 0;
            }
            if ((reg & SWIM_REG_MASK) != 7)
                return;
            // Also keep the simplified SWIM-mode entry used by the unit stub.
        }

        switch (reg & SWIM_REG_MASK) {
            case 0x0: data = v; break;
            case 0x1: mark = v; break;
            case 0x2: error = v; break;
            case 0x3: param[param_idx] = v;
                      param_idx = (param_idx + 1) & 0x0f;
                      break;
            case 0x4: phase = v; break;
            case 0x5: setup = v; break;
            case 0x6: mode &= ~v;
                      param_idx = 0;
                      break;
            case 0x7: mode |= v;
                      param_idx = 0;
                      break;
            case 0x8: data = v; break;
            case 0x9: mark = v; break;
            case 0xA: error = v; break;
            case 0xB: param[param_idx] = v;
                      param_idx = (param_idx + 1) & 0x0f;
                      break;
            case 0xC: phase = v; break;
            case 0xD: setup = v; break;
            default: break;
        }
    }
} swim;

// Scoreboard of observed but unhandled writes — helps diagnose "ROM hung
// on peripheral X because we never ack'd/returned Y" after the run.
static std::map<uint32_t, uint64_t> unmapped_write_count;
static std::map<uint32_t, uint64_t> unmapped_read_count;

enum PeriphCat {
    PC_VIA1,
    PC_VIA2,
    PC_SCC,
    PC_SCSI,
    PC_ASC,
    PC_DAFB_REG,
    PC_VRAM,
    PC_ROM,
    PC_RAM,
    PC_UNMAPPED,
    PC_COUNT
};

struct PeriphAccessSnapshot {
    bool     valid = false;
    bool     write = false;
    uint64_t cycle = 0;
    uint64_t committed = 0;
    uint32_t pc = 0;
    uint32_t addr = 0;
    uint32_t value = 0;
};

struct PeriphStats {
    uint64_t reads = 0;
    uint64_t writes = 0;
    PeriphAccessSnapshot first;
    PeriphAccessSnapshot last;
};

static std::array<PeriphStats, PC_COUNT> periph_stats;
static uint64_t periph_detail_emitted = 0;
static bool     periph_detail_suppressed = false;

static PeriphEventContext periph_event_ctx() {
    PeriphEventContext ctx;
    ctx.cycle = sim_time;
    ctx.committed = dut ? (uint32_t)dut->dbg_committed : 0;
    ctx.pc = dut ? (uint32_t)dut->dbg_pc : 0;
    return ctx;
}

static void periph_event(const char* category,
                         const char* event,
                         uint32_t addr,
                         uint32_t value,
                         const std::string& detail = std::string()) {
    if (!periph_events.enabled()) return;
    periph_events.record(periph_event_ctx(), category, event, addr, value, detail);
}

static void periph_event_c(const char* category,
                           const char* event,
                           uint32_t addr,
                           uint32_t value,
                           const char* detail = "") {
    if (!periph_events.enabled()) return;
    periph_events.record(periph_event_ctx(), category, event, addr, value,
                         detail ? std::string(detail) : std::string());
}

static std::string fmt_kv_u32(const char* key, uint32_t value) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%s=0x%08x", key, value);
    return std::string(buf);
}

static const char* via_reg_name(uint8_t idx) {
    static const char* names[16] = {
        "ORB", "ORA", "DDRB", "DDRA", "T1CL", "T1CH", "T1LL", "T1LH",
        "T2CL", "T2CH", "SR", "ACR", "PCR", "IFR", "IER", "ORA_NH"
    };
    return names[idx & 0x0F];
}

static std::string detail_reg_value(const char* reg, uint32_t value) {
    char detail[96];
    std::snprintf(detail, sizeof(detail), "reg=%s,value=0x%08x", reg, value);
    return std::string(detail);
}

static std::string via1_ifr_clear_detail(uint8_t idx, uint8_t before, uint8_t after) {
    const uint8_t cleared = (uint8_t)((before & ~after) & 0x7f);
    char detail[160];
    std::snprintf(detail, sizeof(detail),
                  "reg=%s,cleared=0x%02x,ifr_before=0x%02x,ifr_after=0x%02x",
                  via_reg_name(idx), cleared, before, after);
    return std::string(detail);
}

static const char* asc_reg_name(uint32_t off) {
    if (off < 0x400) return "FIFO_A";
    if (off < 0x800) return "FIFO_B";
    switch (off) {
        case 0x800: return "VERSION";
        case 0x801: return "MODE";
        case 0x802: return "CHANNEL";
        case 0x803: return "FIFO_CONTROL";
        case 0x804: return "IRQ_STATUS";
        case 0x805: return "WAVETABLE_CONTROL";
        case 0x806: return "VOLUME_ALIAS";
        case 0x807: return "CLOCK_SELECT";
        case 0x808: return "RATE";
        case 0x80A: return "VOLUME";
        case 0x810: return "FIFO_A_STATUS";
        case 0x811: return "FIFO_B_STATUS";
        case 0xF09: return "FIFO_A_IRQ_CONTROL";
        case 0xF29: return "FIFO_B_IRQ_CONTROL";
        default:
            if (off >= 0x830 && off <= 0x83F) return "WAVETABLE";
            return "UNKNOWN";
    }
}

static std::string asc_detail(uint32_t ca) {
    uint32_t off = ca - ASC_BASE;
    char detail[160];
    std::snprintf(detail, sizeof(detail),
                  "off=0x%03x,reg=%s,fifo_a_writes=%u,fifo_b_writes=%u",
                  off & 0xFFFu, asc_reg_name(off & 0xFFFu),
                  asc.fifo_a_writes, asc.fifo_b_writes);
    return std::string(detail);
}

static std::string offset_detail(uint32_t ca, uint32_t base) {
    char detail[64];
    std::snprintf(detail, sizeof(detail), "off=0x%04x", ca - base);
    return std::string(detail);
}

static const char* scc_reg_name(uint8_t reg) {
    switch (reg & 0x0F) {
        case 0: return "RR0_WR0_STATUS_COMMAND";
        case 1: return "RR1_WR1_INT_DATA";
        case 2: return "RR2_WR2_VECTOR";
        case 3: return "RR3_WR3_RX_CONTROL";
        case 4: return "WR4_TX_RX_PARAMS";
        case 5: return "WR5_TX_CONTROL";
        case 8: return "RR8_WR8_DATA";
        case 9: return "WR9_MASTER_INT_RESET";
        case 11: return "WR11_CLOCK_MODE";
        case 12: return "WR12_BRG_LOW";
        case 13: return "WR13_BRG_HIGH";
        case 14: return "WR14_MISC_CONTROL";
        case 15: return "WR15_EXT_STATUS";
        default: return "UNMODELED";
    }
}

static std::string scc_detail(uint32_t ca) {
    uint32_t off = ca - SCC_BASE;
    uint8_t ch = SccStub::channel_index(off);
    bool data = SccStub::is_data_port(off);
    uint8_t reg = data ? 8u : (scc.ptr[ch] & 0x0Fu);
    char detail[192];
    std::snprintf(detail, sizeof(detail),
                  "off=0x%04x,pb=0x%x,channel=%c,port=%s,ptr=%u,reg=%s,rr0=0x%02x,last_tx=0x%02x",
                  off & 0x1FFFu, (off >> 2) & 0x0Fu, ch ? 'A' : 'B',
                  data ? "data" : "control", scc.ptr[ch],
                  scc_reg_name(reg), scc.rr(ch, 0), scc.last_tx[ch]);
    return std::string(detail);
}

static const char* turboscsi_region_name(uint32_t off) {
    if (off < 0x100u) return "ncr53c96_regs";
    if (off < 0x102u) return "dma_handshake";
    return "gap";
}

static const char* turboscsi_reg_name(uint32_t off) {
    switch (off & 0x0Fu) {
        case 0x0: return "TC_LOW";
        case 0x1: return "TC_MID";
        case 0x2: return "FIFO";
        case 0x3: return "COMMAND";
        case 0x4: return "STATUS";
        case 0x5: return "INTERRUPT";
        case 0x6: return "SEQ_STEP";
        case 0x7: return "FLAGS";
        case 0x8: return "CONFIG1";
        case 0x9: return "CLOCK_CONV";
        case 0xA: return "TEST";
        case 0xB: return "CONFIG2";
        case 0xC: return "CONFIG3";
        case 0xD: return "CONFIG4";
        default:  return "RESERVED";
    }
}

static const char* turboscsi_event_name(bool write, uint32_t off) {
    if (off < 0x100u) return write ? "reg_write" : "reg_read";
    if (off < 0x102u) return write ? "dma_write" : "dma_read";
    return write ? "gap_write" : "gap_read";
}

static const char* turboscsi_command_name(uint8_t cmd) {
    switch (cmd & 0x7Fu) {
        case 0x00: return "NOP";
        case 0x01: return "FLUSH_FIFO";
        case 0x02: return "RESET_CHIP";
        case 0x03: return "RESET_SCSI_BUS";
        case 0x10: return "TRANSFER_INFORMATION";
        case 0x11: return "INITIATOR_COMMAND_COMPLETE";
        case 0x12: return "MESSAGE_ACCEPTED";
        case 0x18: return "TRANSFER_PAD";
        case 0x20: return "SET_ATN";
        case 0x21: return "RESET_ATN";
        case 0x40: return "SELECT_WITHOUT_ATN";
        case 0x41: return "SELECT_WITH_ATN";
        case 0x42: return "SELECT_WITH_ATN_STOP";
        case 0x43: return "ENABLE_SELECTION_RESELECTION";
        case 0x44: return "DISABLE_SELECTION_RESELECTION";
        default:   return "UNKNOWN";
    }
}

static std::string turboscsi_detail(uint32_t ca) {
    uint32_t off = ca - TURBOSCSI_BASE;
    char detail[128];
    if (off < 0x100u) {
        std::snprintf(detail, sizeof(detail), "off=0x%03x,region=%s,reg=%s",
                      off, turboscsi_region_name(off), turboscsi_reg_name(off));
    } else {
        std::snprintf(detail, sizeof(detail), "off=0x%03x,region=%s",
                      off, turboscsi_region_name(off));
    }
    return std::string(detail);
}

static const char* turboscsi_phase_name(uint8_t phase) {
    switch (phase & 0x07u) {
        case 0x00: return "BUS_FREE";
        case 0x01: return "SELECT";
        case 0x02: return "COMMAND";
        case 0x03: return "DATA_IN";
        case 0x04: return "DATA_OUT";
        case 0x05: return "STATUS";
        case 0x06: return "MSG_IN";
        case 0x07: return "DISCONNECT";
        default:   return "UNKNOWN";
    }
}

static uint8_t turboscsi_phase_bits(uint8_t phase) {
    switch (phase & 0x07u) {
        case 0x02: return 0x02;
        case 0x03: return 0x01;
        case 0x04: return 0x00;
        case 0x05: return 0x03;
        case 0x06: return 0x07;
        default:   return 0x00;
    }
}

static uint8_t turboscsi_status_value() {
    if (turboscsi_trace.command_active) {
        switch (turboscsi_trace.data_phase & 0x07u) {
            case 0x01:
                return 0x40 | 0x02;
            case 0x02:
                return 0x40 | 0x08 | 0x20;
            case 0x03:
                return 0x40 | 0x04 | 0x20;
            case 0x04:
                return 0x40 | 0x20;
            case 0x05:
                return 0x40 | 0x08 | 0x04 | 0x20;
            case 0x06:
                return 0x40 | 0x10 | 0x08 | 0x04 | 0x20;
            default:
                break;
        }
    }
    if (turboscsi_trace.selected)
        return 0x40 | 0x02;
    return 0x00;
}

static uint8_t turboscsi_bus_and_stat_value() {
    uint8_t value = 0x00;
    if (turboscsi_phase_bits(turboscsi_trace.data_phase) ==
        (turboscsi_trace.regs[3] & 0x07u)) {
        value |= 0x08;
    }
    if (turboscsi_trace.end_dma_pending)
        value |= 0x80;
    if (turboscsi_trace.drq_pending)
        value |= 0x40;
    if (turboscsi_trace.irq_pending || turboscsi_trace.data_phase == 0x05 ||
        turboscsi_trace.data_phase == 0x06) {
        value |= 0x10;
    }
    if (turboscsi_trace.busy_error_pending)
        value |= 0x04;
    if (turboscsi_trace.regs[1] & 0x02u)
        value |= 0x02;
    if (turboscsi_trace.regs[1] & 0x10u)
        value |= 0x01;
    return value;
}

static void turboscsi_set_phase(uint8_t phase) {
    turboscsi_trace.data_phase = phase & 0x07u;
    switch (turboscsi_trace.data_phase) {
        case 0x00:  // BUS_FREE
            turboscsi_trace.selected = false;
            turboscsi_trace.command_active = false;
            turboscsi_trace.drq_pending = false;
            turboscsi_trace.irq_pending = false;
            turboscsi_trace.end_dma_pending = false;
            break;
        case 0x01:  // SELECT
        case 0x02:  // COMMAND
            turboscsi_trace.selected = true;
            turboscsi_trace.command_active = true;
            turboscsi_trace.drq_pending = false;
            turboscsi_trace.irq_pending = false;
            turboscsi_trace.end_dma_pending = false;
            break;
        case 0x03:  // DATA_IN
        case 0x04:  // DATA_OUT
            turboscsi_trace.selected = true;
            turboscsi_trace.command_active = true;
            turboscsi_trace.drq_pending = true;
            turboscsi_trace.irq_pending = false;
            turboscsi_trace.end_dma_pending = false;
            break;
        case 0x05:  // STATUS
        case 0x06:  // MSG_IN
            turboscsi_trace.selected = true;
            turboscsi_trace.command_active = true;
            turboscsi_trace.drq_pending = false;
            turboscsi_trace.irq_pending = true;
            turboscsi_trace.end_dma_pending = true;
            break;
        case 0x07:  // DISCONNECT
            turboscsi_trace.selected = false;
            turboscsi_trace.command_active = false;
            turboscsi_trace.drq_pending = false;
            break;
        default:
            break;
    }
}

static void turboscsi_observe_read(uint32_t ca, uint8_t value) {
    const uint32_t off = ca - TURBOSCSI_BASE;
    if (!periph_events.enabled() || off >= 0x100u) return;

    char detail[160];
    if ((off & 0x0Fu) == 0x04u) {
        std::snprintf(detail, sizeof(detail),
                      "reg=STATUS,value=0x%02x,phase=%s,last_command=0x%02x,last_command_name=%s,selected=%u,drq=%u,irq=%u,end_dma=%u",
                      value,
                      turboscsi_phase_name(turboscsi_trace.data_phase),
                      turboscsi_trace.last_command,
                      turboscsi_command_name(turboscsi_trace.last_command),
                      turboscsi_trace.selected ? 1u : 0u,
                      turboscsi_trace.drq_pending ? 1u : 0u,
                      turboscsi_trace.irq_pending ? 1u : 0u,
                      turboscsi_trace.end_dma_pending ? 1u : 0u);
        periph_event("SCSI", "status_phase", ca, value, detail);
    } else if ((off & 0x0Fu) == 0x05u) {
        std::snprintf(detail, sizeof(detail),
                      "reg=INTERRUPT,value=0x%02x,phase=%s,last_command=0x%02x,last_command_name=%s,selected=%u,drq=%u,irq=%u,end_dma=%u",
                      value,
                      turboscsi_phase_name(turboscsi_trace.data_phase),
                      turboscsi_trace.last_command,
                      turboscsi_command_name(turboscsi_trace.last_command),
                      turboscsi_trace.selected ? 1u : 0u,
                      turboscsi_trace.drq_pending ? 1u : 0u,
                      turboscsi_trace.irq_pending ? 1u : 0u,
                      turboscsi_trace.end_dma_pending ? 1u : 0u);
        periph_event("SCSI", "interrupt_phase", ca, value, detail);
    } else if ((off & 0x0Fu) == 0x06u) {
        std::snprintf(detail, sizeof(detail),
                      "reg=SEQ_STEP,value=0x%02x,phase=%s,last_command=0x%02x,last_command_name=%s",
                      value,
                      turboscsi_phase_name(turboscsi_trace.data_phase),
                      turboscsi_trace.last_command,
                      turboscsi_command_name(turboscsi_trace.last_command));
        periph_event("SCSI", "sequence_phase", ca, value, detail);
    }
}

static const char* scsi_cdb_name(uint8_t opcode) {
    switch (opcode) {
        case 0x00: return "TEST_UNIT_READY";
        case 0x03: return "REQUEST_SENSE";
        case 0x08: return "READ_6";
        case 0x0A: return "WRITE_6";
        case 0x12: return "INQUIRY";
        case 0x15: return "MODE_SELECT_6";
        case 0x1A: return "MODE_SENSE_6";
        case 0x1B: return "START_STOP_UNIT";
        case 0x25: return "READ_CAPACITY_10";
        case 0x28: return "READ_10";
        case 0x2A: return "WRITE_10";
        default:   return "UNKNOWN";
    }
}

static uint8_t scsi_cdb_len(uint8_t opcode) {
    if ((opcode & 0xE0u) == 0x20u || (opcode & 0xE0u) == 0x40u) return 10;
    return 6;
}

static bool scsi_cdb_opcode_observable(uint8_t opcode) {
    switch (opcode) {
        case 0x00: case 0x03: case 0x08: case 0x0A:
        case 0x12: case 0x15: case 0x1A: case 0x1B:
        case 0x25: case 0x28: case 0x2A:
            return true;
        default:
            return false;
    }
}

static void turboscsi_emit_cdb_summary(uint32_t ca) {
    if (!periph_events.enabled() || turboscsi_trace.cdb_len == 0) return;

    char bytes[96];
    size_t pos = 0;
    for (uint8_t i = 0; i < turboscsi_trace.cdb_len && pos < sizeof(bytes); i++) {
        int n = std::snprintf(bytes + pos, sizeof(bytes) - pos,
                              "%s%02x", (i == 0) ? "" : " ",
                              turboscsi_trace.cdb[i]);
        if (n < 0) break;
        pos += (size_t)n;
    }
    bytes[sizeof(bytes) - 1] = '\0';

    char detail[192];
    const uint8_t opcode = turboscsi_trace.cdb[0];
    std::snprintf(detail, sizeof(detail),
                  "opcode=0x%02x,name=%s,len=%u,cdb=%s,raw_base_lba=%u,raw_reserved_bytes=%u",
                  opcode, scsi_cdb_name(opcode),
                  (unsigned)turboscsi_trace.cdb_len, bytes,
                  SCSI_RAW_SD_LBA_BIAS, SCSI_RAW_SD_RESERVED_BYTES);
    periph_event("SCSI", "cdb", ca, opcode, detail);

    if (opcode == 0x08 || opcode == 0x0A) {
        const uint32_t lba = (((uint32_t)turboscsi_trace.cdb[1] & 0x1Fu) << 16) |
                             ((uint32_t)turboscsi_trace.cdb[2] << 8) |
                             (uint32_t)turboscsi_trace.cdb[3];
        const uint32_t blocks = (turboscsi_trace.cdb[4] == 0)
                              ? 256u : (uint32_t)turboscsi_trace.cdb[4];
        std::snprintf(detail, sizeof(detail),
                      "opcode=0x%02x,name=%s,lba=0x%06x,blocks=%u,sd_lba=0x%08x,sd_byte_offset=0x%08x,drq=%u,irq=%u",
                      opcode, scsi_cdb_name(opcode), lba, blocks,
                      SCSI_RAW_SD_LBA_BIAS + lba,
                      (SCSI_RAW_SD_LBA_BIAS + lba) * 512u,
                      turboscsi_trace.drq_pending ? 1u : 0u,
                      turboscsi_trace.irq_pending ? 1u : 0u);
        periph_event("SCSI", opcode == 0x08 ? "raw_block_read" : "raw_block_write",
                     ca, lba, detail);
    } else if (opcode == 0x28 || opcode == 0x2A) {
        const uint32_t lba = ((uint32_t)turboscsi_trace.cdb[2] << 24) |
                             ((uint32_t)turboscsi_trace.cdb[3] << 16) |
                             ((uint32_t)turboscsi_trace.cdb[4] << 8) |
                             (uint32_t)turboscsi_trace.cdb[5];
        const uint32_t blocks = ((uint32_t)turboscsi_trace.cdb[7] << 8) |
                                (uint32_t)turboscsi_trace.cdb[8];
        std::snprintf(detail, sizeof(detail),
                      "opcode=0x%02x,name=%s,lba=0x%08x,blocks=%u,sd_lba=0x%08x,sd_byte_offset=0x%08x,drq=%u,irq=%u",
                      opcode, scsi_cdb_name(opcode), lba, blocks,
                      SCSI_RAW_SD_LBA_BIAS + lba,
                      (SCSI_RAW_SD_LBA_BIAS + lba) * 512u,
                      turboscsi_trace.drq_pending ? 1u : 0u,
                      turboscsi_trace.irq_pending ? 1u : 0u);
        periph_event("SCSI", opcode == 0x28 ? "raw_block_read" : "raw_block_write",
                     ca, lba, detail);
    }
}

static void turboscsi_observe_write(uint32_t ca, uint8_t value) {
    const uint32_t off = ca - TURBOSCSI_BASE;

    if (off == 0x02u) {
        turboscsi_trace.regs[off] = value;
        if (turboscsi_trace.cdb_idx == 0) {
            if (!scsi_cdb_opcode_observable(value)) {
                if (periph_events.enabled())
                    periph_event("SCSI", "fifo_write", ca, value, "reg=FIFO,role=unknown");
                return;
            }
            turboscsi_trace.cdb_len = scsi_cdb_len(value);
            turboscsi_trace.command_active = true;
            turboscsi_set_phase(0x02);
        }

        if (turboscsi_trace.cdb_idx < turboscsi_trace.cdb.size()) {
            turboscsi_trace.cdb[turboscsi_trace.cdb_idx++] = value;
            if (periph_events.enabled()) {
                char detail[96];
                std::snprintf(detail, sizeof(detail),
                              "reg=FIFO,cdb_index=%u,cdb_len=%u,phase=%s",
                              (unsigned)(turboscsi_trace.cdb_idx - 1),
                              (unsigned)turboscsi_trace.cdb_len,
                              turboscsi_phase_name(turboscsi_trace.data_phase));
                periph_event("SCSI", "cdb_byte", ca, value, detail);
            }
            if (turboscsi_trace.cdb_idx >= turboscsi_trace.cdb_len) {
                turboscsi_emit_cdb_summary(ca);
                const uint8_t opcode = turboscsi_trace.cdb[0];
                if (opcode == 0x08 || opcode == 0x28) {
                    turboscsi_set_phase(0x03);
                } else if (opcode == 0x0A || opcode == 0x2A) {
                    turboscsi_set_phase(0x04);
                } else {
                    turboscsi_trace.current_data = 0x00;
                    turboscsi_set_phase(0x05);
                }
                turboscsi_trace.cdb_idx = 0;
                turboscsi_trace.cdb_len = 0;
            }
        } else {
            turboscsi_trace.reset();
        }
    } else if (off == 0x03u) {
        turboscsi_trace.last_command = value;
        if ((value & 0x7Fu) == 0x02u) {
            turboscsi_trace.reset();
            if (periph_events.enabled())
                periph_event("SCSI", "fifo_reset", ca, value, "reg=COMMAND");
        } else {
            if ((value & 0x7Cu) == 0x40u) {
                turboscsi_trace.regs[off] = 0x01;
            } else if ((value & 0x7Fu) == 0x10u) {
                turboscsi_trace.regs[off] = 0x02;
            } else if ((value & 0x7Fu) == 0x11u) {
                turboscsi_trace.regs[off] = 0x05;
            } else if ((value & 0x7Fu) == 0x12u) {
                turboscsi_trace.regs[off] = 0x06;
            } else {
                turboscsi_trace.regs[off] = (uint8_t)(value & 0x07u);
            }
            char detail[96];
            std::snprintf(detail, sizeof(detail),
                          "reg=COMMAND,cmd=0x%02x,name=%s,phase=%s,selected=%u,drq=%u,irq=%u",
                          value, turboscsi_command_name(value),
                          turboscsi_phase_name(turboscsi_trace.data_phase),
                          turboscsi_trace.selected ? 1u : 0u,
                          turboscsi_trace.drq_pending ? 1u : 0u,
                          turboscsi_trace.irq_pending ? 1u : 0u);
            if (periph_events.enabled())
                periph_event("SCSI", "command_write", ca, value, detail);
            if ((value & 0x7Cu) == 0x40u) {
                turboscsi_set_phase(0x01);
                if (periph_events.enabled())
                    periph_event("SCSI", "selection_phase", ca, value, detail);
            } else if ((value & 0x7Fu) == 0x10u) {
                turboscsi_set_phase(0x02);
                if (periph_events.enabled())
                    periph_event("SCSI", "command_phase", ca, value, detail);
            } else if ((value & 0x7Fu) == 0x11u) {
                turboscsi_set_phase(0x05);
                if (periph_events.enabled())
                    periph_event("SCSI", "status_phase", ca, value, detail);
            } else if ((value & 0x7Fu) == 0x12u) {
                turboscsi_set_phase(0x06);
                if (periph_events.enabled())
                    periph_event("SCSI", "interrupt_phase", ca, value, detail);
            }
        }
    } else if (off >= 0x100u && off < 0x102u) {
        if (turboscsi_trace.drq_pending) {
            turboscsi_trace.current_data = value;
        }
        if (periph_events.enabled()) {
            periph_event("SCSI", "dma_handshake", ca, value,
                         turboscsi_detail(ca));
        }
    } else if (off < 0x100u) {
        turboscsi_trace.regs[off] = value;
        if (off == 0x00u)
            turboscsi_trace.current_data = value;
        if (off == 0x01u)
            turboscsi_trace.regs[1] = value;
        if (off == 0x04u) {
            turboscsi_trace.selected = (value & 0x02u) != 0;
        }
        if (off == 0x07u && value == 0x00u) {
            turboscsi_trace.irq_pending = false;
            turboscsi_trace.end_dma_pending = false;
            turboscsi_trace.busy_error_pending = false;
        }
        if (periph_events.enabled()) {
            char detail[96];
            std::snprintf(detail, sizeof(detail), "reg=%s,phase=%s",
                          turboscsi_reg_name(off),
                          turboscsi_phase_name(turboscsi_trace.data_phase));
            periph_event("SCSI", "reg_write", ca, value, detail);
        }
    }
}

static const char* dafb_reg_name(uint32_t addr) {
    switch (addr & 0xFFFu) {
        case 0x000: return "CTRL";
        case 0x008: return "FB_BASE";
        case 0x00C: return "FB_STRIDE";
        case 0x010: return "FB_BPP";
        case 0x01C: return "IRQ_ENABLE";
        case 0x020: return "IRQ_STATUS";
        case 0x024: return "FIRST_HIT";
        case 0x200: return "SENSE_DRIVE";
        case 0x220: return "SENSE_DATA";
        default:
            if ((addr & 0x3C0u) == 0x300u && (addr & 0x0Fu) == 0)
                return "CLUT";
            return "UNKNOWN";
    }
}

static const char* dafb_reg_semantics(uint32_t addr) {
    switch (addr & 0xFFFu) {
        case 0x008: return "semantic=scanout_fb_base_px,unit=pixels";
        case 0x00C: return "semantic=scanout_fb_stride_px,unit=pixels";
        case 0x010: return "semantic=raw_depth_selector,firstlight_pix_bpp=8";
        case 0x01C: return "semantic=irq_enable_latch";
        case 0x020: return "semantic=sticky_vblank_status_w1c";
        case 0x024: return "semantic=rom_first_dafb_write";
        case 0x200: return "semantic=monitor_sense_drive";
        case 0x220: return "semantic=monitor_sense_data";
        default:
            if ((addr & 0x3C0u) == 0x300u && (addr & 0x0Fu) == 0)
                return "semantic=low_depth_clut_entry";
            return "semantic=stored_echo";
    }
}

static std::string dafb_detail(uint32_t addr) {
    char detail[160];
    std::snprintf(detail, sizeof(detail),
                  "window=0x%08x..0x%08x,off=0x%03x,reg=%s,%s",
                  DAFB_REG_BASE,
                  DAFB_REG_BASE + DAFB_REG_SIZE - 1,
                  addr & 0xFFFu,
                  dafb_reg_name(addr),
                  dafb_reg_semantics(addr));
    return std::string(detail);
}

static std::string vram_access_detail(bool write, uint32_t addr) {
    const bool first_activity = !periph_stats[(size_t)PC_VRAM].first.valid;
    char detail[128];
    std::snprintf(detail, sizeof(detail),
                  "pixel-vram-aperture,off=0x%05x,op=%s,first_activity=%u",
                  addr - DAFB_VRAM_BASE,
                  write ? "write" : "read",
                  first_activity ? 1u : 0u);
    return std::string(detail);
}

struct RtcSidechannelModel {
    static constexpr uint64_t CYCLES_PER_SECOND = 50000;

    bool initialized = false;
    bool prev_enb = true;
    bool prev_clk = false;
    bool active = false;
    bool is_read = false;
    uint8_t cmd = 0;
    uint8_t data = 0;
    uint8_t cmd_bits = 0;
    uint8_t data_bits = 0;
    uint8_t read_shift = 0;
    bool data_line = true;
    bool is_extended = false;
    bool xp_addr_done = false;
    uint8_t xp_addr = 0;

    uint32_t seconds_base = 0;
    uint64_t seconds_base_cycle = 0;
    bool write_protect = false;
    bool test_mode = false;
    std::array<uint8_t, 256> pram{};

    static uint8_t pram_reset_byte(uint8_t addr) {
        switch (addr) {
            case 0x02: return 0x4F;
            case 0x03: return 0x48;
            case 0x08: return 0x13;
            case 0x09: return 0x88;
            case 0x0B: return 0x4C;
            case 0x0C: return 0x4E;
            case 0x0D: return 0x75;
            case 0x0E: return 0x4D;
            case 0x0F: return 0x63;
            case 0x10: return 0xA8;
            default: return 0x00;
        }
    }

    static bool is_extended_cmd(uint8_t c) {
        return (c & 0x78) == 0x38;
    }

    static bool is_pram_reg(uint8_t reg) {
        return (reg >= 8 && reg <= 11) || reg >= 16;
    }

    static uint8_t xpram_addr_for(uint8_t c, uint8_t addr_byte) {
        return (uint8_t)(((c & 0x07) << 5) | ((addr_byte >> 2) & 0x1f));
    }

    void reset(uint64_t now = 0) {
        initialized = false;
        prev_enb = true;
        prev_clk = false;
        active = false;
        is_read = false;
        cmd = 0;
        data = 0;
        cmd_bits = 0;
        data_bits = 0;
        read_shift = 0;
        data_line = true;
        is_extended = false;
        xp_addr_done = false;
        xp_addr = 0;
        seconds_base = 0;
        seconds_base_cycle = now;
        write_protect = false;
        test_mode = false;
        for (size_t i = 0; i < pram.size(); i++)
            pram[i] = pram_reset_byte((uint8_t)i);
    }

    uint32_t seconds_now(uint64_t now) const {
        const uint64_t elapsed = now - seconds_base_cycle;
        return seconds_base + (uint32_t)(elapsed / CYCLES_PER_SECOND);
    }

    uint8_t read_byte_for_cmd(uint8_t c, uint64_t now) const {
        const uint32_t sec = seconds_now(now);
        switch ((c >> 2) & 0x1f) {
            case 0: case 4: return (uint8_t)(sec >> 0);
            case 1: case 5: return (uint8_t)(sec >> 8);
            case 2: case 6: return (uint8_t)(sec >> 16);
            case 3: case 7: return (uint8_t)(sec >> 24);
            case 12: return test_mode ? 0x80 : 0x00;
            case 13: return write_protect ? 0x80 : 0x00;
            default: {
                const uint8_t reg = (c >> 2) & 0x1F;
                return is_pram_reg(reg) ? pram[reg] : 0x00;
            }
        }
    }

    void write_byte_for_cmd(uint8_t c, uint8_t v, uint64_t now, uint32_t addr) {
        const uint8_t reg = (c >> 2) & 0x1F;
        char detail[192];
        if (reg == 13) {
            write_protect = (v & 0x80) != 0;
            std::snprintf(detail, sizeof(detail),
                          "cmd=0x%02x,reg=%u,data=0x%02x,wp=%u",
                          c, (unsigned)reg, v, write_protect ? 1u : 0u);
            periph_event("PRAM", "write_protect", addr, v, detail);
        } else if (write_protect) {
            std::snprintf(detail, sizeof(detail),
                          "cmd=0x%02x,reg=%u,data=0x%02x",
                          c, (unsigned)reg, v);
            periph_event("PRAM", "write_blocked", addr, v, detail);
        } else if (reg == 12) {
            // The test bit is a latched control flag only.  Keep the
            // deterministic seconds model running so the ROM harness
            // matches the RTL/macrtc behavior and the MAME reference.
            test_mode = (v & 0x80) != 0;
            std::snprintf(detail, sizeof(detail),
                          "cmd=0x%02x,reg=%u,data=0x%02x,test=%u",
                          c, (unsigned)reg, v, test_mode ? 1u : 0u);
            periph_event("PRAM", "test_mode", addr, v, detail);
        } else if (reg < 8) {
            uint32_t sec = seconds_now(now);
            const uint8_t idx = (c >> 2) & 0x03;
            const uint32_t mask = 0xFFu << (idx * 8);
            sec = (sec & ~mask) | ((uint32_t)v << (idx * 8));
            seconds_base = sec;
            seconds_base_cycle = now;
        } else if (is_pram_reg(reg)) {
            pram[reg] = v;
        }
    }

    void write_xpram(uint8_t v, uint64_t, uint32_t addr) {
        char detail[160];
        if (write_protect) {
            std::snprintf(detail, sizeof(detail),
                          "cmd=0x%02x,xpaddr=0x%02x,data=0x%02x",
                          cmd, xp_addr, v);
            periph_event("PRAM", "xpram_write_blocked", addr, v, detail);
            return;
        }
        pram[xp_addr] = v;
    }

    void sample(bool enb,
                bool clk,
                bool cpu_data_bit,
                bool data_oe,
                uint32_t addr,
                uint8_t orb,
                uint8_t ddrb) {
        if (!initialized) {
            initialized = true;
            prev_enb = enb;
            prev_clk = clk;
        }

        char detail[160];
        if (!enb && prev_enb) {
            active = true;
            is_read = false;
            cmd = 0;
            data = 0;
            cmd_bits = 0;
            data_bits = 0;
            read_shift = 0;
            data_line = true;
            is_extended = false;
            xp_addr_done = false;
            xp_addr = 0;
            std::snprintf(detail, sizeof(detail),
                          "orb=0x%02x,ddrb=0x%02x", orb, ddrb);
            periph_event("RTC", "select", addr, orb, detail);
        }

        // MAME macrtc.cpp shifts command/write data on rTCClk high-to-low.
        // ROM code often represents a host-side '1' by releasing PB0 and
        // letting the external pull-up drive the line high.
        bool clk_fall = !clk && prev_clk;
        if (active && !enb && clk_fall) {
            const bool host_bit = data_oe ? cpu_data_bit : true;
            const bool line_bit = data_oe ? cpu_data_bit : data_line;
            std::snprintf(detail, sizeof(detail),
                          "data_oe=%u,host_bit=%u",
                          data_oe ? 1u : 0u, host_bit ? 1u : 0u);
            periph_event("RTC", "clk_fall", addr, line_bit ? 1u : 0u, detail);
            if (cmd_bits < 8) {
                if (!data_oe) data_line = true;
                cmd = (uint8_t)((cmd << 1) | (host_bit ? 1 : 0));
                cmd_bits++;
                if (cmd_bits == 8) {
                    is_read = (cmd & 0x80) != 0;
                    is_extended = is_extended_cmd(cmd);
                    const char* event = is_extended
                                      ? (is_read ? "cmd_xpram_read" : "cmd_xpram_write")
                                      : (is_read ? "cmd_read" : "cmd_write");
                    std::snprintf(detail, sizeof(detail),
                                  "cmd=0x%02x,reg=%u,rw=%s,extended=%u",
                                  cmd, (unsigned)((cmd >> 2) & 0x1F),
                                  is_read ? "read" : "write",
                                  is_extended ? 1u : 0u);
                    periph_event("RTC", event, addr, cmd, detail);
                    if (is_read && !is_extended) {
                        read_shift = read_byte_for_cmd(cmd, host_cycle);
                        const uint8_t reg = (cmd >> 2) & 0x1F;
                        if (reg < 8) {
                            periph_event("PRAM", "seconds_read", addr, cmd, detail);
                        } else if (is_pram_reg(reg)) {
                            periph_event("PRAM", "pram_read", addr, cmd, detail);
                        } else if (reg == 12 || reg == 13) {
                            periph_event("PRAM", "control_read", addr, cmd, detail);
                        }
                    }
                }
            } else if (is_extended && !xp_addr_done && data_bits < 8) {
                if (!data_oe) data_line = true;
                data = (uint8_t)((data << 1) | (host_bit ? 1 : 0));
                data_bits++;
                if (data_bits == 8) {
                    xp_addr = xpram_addr_for(cmd, data);
                    xp_addr_done = true;
                    std::snprintf(detail, sizeof(detail),
                                  "cmd=0x%02x,addr_byte=0x%02x,xpaddr=0x%02x,rw=%s",
                                  cmd, data, xp_addr, is_read ? "read" : "write");
                    periph_event("RTC", "xpram_addr", addr, xp_addr, detail);
                    if (is_read) {
                        read_shift = pram[xp_addr];
                        periph_event("PRAM", "xpram_read", addr, xp_addr, detail);
                    }
                    data = 0;
                    data_bits = 0;
                }
            } else if (is_read && !data_oe && data_bits < 8) {
                data_line = (read_shift & 0x80) != 0;
                read_shift = (uint8_t)(read_shift << 1);
                data_bits++;
            } else if (!is_read && data_bits < 8) {
                if (!data_oe) data_line = true;
                data = (uint8_t)((data << 1) | (host_bit ? 1 : 0));
                data_bits++;
                if (data_bits == 8) {
                    std::snprintf(detail, sizeof(detail),
                                  "cmd=0x%02x,reg=%u,data=0x%02x,extended=%u",
                                  cmd, (unsigned)((cmd >> 2) & 0x1F), data,
                                  is_extended ? 1u : 0u);
                    periph_event("RTC", "write_data", addr, data, detail);
                    if (is_extended)
                        periph_event("PRAM", "xpram_write", addr, data, detail);
                    else if (((cmd >> 2) & 0x1F) < 8)
                        periph_event("PRAM", "seconds_write", addr, data, detail);
                    else if (is_pram_reg((cmd >> 2) & 0x1F))
                        periph_event("PRAM", "pram_write", addr, data, detail);
                    else if (((cmd >> 2) & 0x1F) == 12 || ((cmd >> 2) & 0x1F) == 13)
                        periph_event("PRAM", "control_write", addr, data, detail);
                }
            }
        }

        if (enb && !prev_enb && active) {
            std::snprintf(detail, sizeof(detail),
                          "cmd=0x%02x,cmd_bits=%u,data_bits=%u",
                          cmd, (unsigned)cmd_bits, (unsigned)data_bits);
            periph_event("RTC", "deselect", addr, cmd, detail);
            if (!is_read && cmd_bits == 8 && data_bits == 8) {
                if (is_extended && xp_addr_done)
                    write_xpram(data, host_cycle, addr);
                else if (!is_extended)
                    write_byte_for_cmd(cmd, data, host_cycle, addr);
            }
            active = false;
            data_line = true;
        }

        prev_enb = enb;
        prev_clk = clk;
    }
} rtc_sidechannel;

static void via1_update_rtc_pins(uint32_t addr) {
    // The ROM sees the RTC only through VIA1 PB0/PB1/PB2.  Model the
    // off-chip 343S0042 side channel here so ORB reads can sample PB0
    // after command/read clock edges, while retaining the event log.
    bool enb = (via1.ddrb & 0x04) ? ((via1.orb & 0x04) != 0) : true;
    bool clk = (via1.ddrb & 0x02) ? ((via1.orb & 0x02) != 0) : false;
    bool data_oe = (via1.ddrb & 0x01) != 0;
    bool data_bit = (via1.orb & 0x01) != 0;
    rtc_sidechannel.sample(enb, clk, data_bit, data_oe, addr, via1.orb, via1.ddrb);
}

static bool     q700_descriptor_selected = false;
static uint32_t q700_descriptor_entry    = 0;
static uint32_t q700_feature_word        = 0;

// Optional fast-iteration trace.  When +lastn_trace is absent the only
// per-cycle cost is one branch in the main loop.
struct LastNTraceSample {
    uint64_t sim_time = 0;
    uint32_t committed = 0;
    uint32_t dbg_last_pc = 0;
    uint32_t dbg_pc = 0;
    uint8_t  overlay_active = 0;
    uint8_t  via1_overlay_live = 0;

    uint8_t  rob_valid = 0;
    uint8_t  rob_complete = 0;
    uint8_t  rob_vec = 0;
    uint32_t rob_pc = 0;

    uint8_t  commit_exc_wait = 0;
    uint8_t  commit_take_exc = 0;

    uint8_t  exc_state = 0;
    uint8_t  exc_vec = 0;
    uint32_t exc_fault_pc = 0;
    uint32_t exc_a7 = 0;

    uint16_t arch_sr = 0;
    uint8_t  arch_ccr = 0;
    uint32_t arch_vbr = 0;
    uint32_t arch_usp = 0;
    uint32_t arch_ssp = 0;
    uint32_t arch_isp = 0;
    uint32_t arch_cacr = 0;
    uint32_t arch_sfc = 0;
    uint32_t arch_dfc = 0;

    uint8_t  exc_gate_pending_exc = 0;
    uint8_t  exc_gate_pending_rte = 0;
    uint8_t  exc_flush_req = 0;
    uint8_t  exc_flush_done = 0;

    uint8_t  fq0_valid = 0;
    uint8_t  fq0_src = 0;
    uint8_t  fq0_inval = 0;
    uint8_t  fq1_valid = 0;
    uint8_t  fq1_src = 0;
    uint8_t  fq1_inval = 0;

    uint8_t  dcache_state = 0;
    uint8_t  dcache_fa_set = 0;
    uint8_t  dcache_fa_way = 0;
    uint8_t  dcache_beat = 0;

    uint8_t  mem_iss_valid = 0;
    uint8_t  mem_iss_is_store = 0;
    uint8_t  mem_iss_is_rts = 0;
    uint8_t  mem_iss_pbase = 0;
    uint32_t mem_iss_base = 0;
    uint32_t mem_iss_disp = 0;
    uint8_t  mem_iss_pdst = 0;
    uint8_t  mem_iss_tag = 0;
    uint8_t  mem_iss_imm_is_data = 0;
    uint32_t mem_iss_imm_data = 0;

    uint8_t  f1_mem_valid = 0;
    uint8_t  f1_mem_is_rts = 0;
    uint8_t  f1_mem_pbase = 0;
    uint32_t f1_mem_base = 0;
    uint32_t f1_mem_disp = 0;
    uint8_t  f1_mem_pdst = 0;
    uint8_t  f1_mem_tag = 0;

    uint8_t  lsu_cur_is_rts = 0;
    uint8_t  lsu_state = 0;
    uint8_t  lsu_cur_is_store = 0;
    uint8_t  lsu_cur_split = 0;
    uint8_t  lsu_cur_pdst = 0;
    uint8_t  lsu_cur_tag = 0;
    uint32_t lsu_ea = 0;
    uint32_t lsu_cur_data = 0;
    uint32_t lsu_split_first_rdata = 0;
    uint32_t lsu_split_second_addr = 0;
    uint32_t lsu_split_second_wdata = 0;
    uint8_t  lsu_split_second_wstrb = 0;
    uint8_t  commit_store_en = 0;
    uint8_t  flush_en = 0;
    uint8_t  core_dc_req = 0;
    uint8_t  core_dc_is_write = 0;
    uint32_t core_dc_addr = 0;
    uint32_t core_dc_wdata = 0;
    uint8_t  core_dc_wstrb = 0;
    uint32_t core_dc_rdata = 0;
    uint8_t  core_dc_rvalid = 0;
    uint8_t  core_dc_bvalid = 0;

    uint8_t  dmmu_req_valid = 0;
    uint8_t  dmmu_resp_ready = 0;
    uint8_t  dmmu_lsu_ready = 0;
    uint8_t  dmmu_need_walk = 0;
    uint8_t  dmmu_w_busy = 0;
    uint8_t  dmmu_req_pending = 0;
    uint8_t  dmmu_req_wait_drop = 0;
    uint8_t  dmmu_ttr_wp_fault = 0;
    uint8_t  dmmu_atc_wp_fault = 0;
    uint8_t  dmmu_atc_sup_fault = 0;
    uint8_t  dmmu_fault = 0;
    uint8_t  dmmu_fault_code = 0;
    uint8_t  dmmu_mmu_flush = 0;
    uint8_t  dmmu_flush_wake = 0;
    uint32_t dmmu_va = 0;
    uint32_t dmmu_pa = 0;
    uint32_t dmmu_tc = 0;
    uint32_t dmmu_itt0 = 0;
    uint32_t dmmu_itt1 = 0;
    uint32_t dmmu_dtt0 = 0;
    uint32_t dmmu_dtt1 = 0;
    uint32_t dmmu_srp = 0;
    uint32_t dmmu_urp = 0;

    uint8_t  walk_state = 0;
    uint8_t  walk_is_write = 0;
    uint8_t  walk_sup = 0;
    uint8_t  walk_psz8k = 0;
    uint8_t  walk_three = 0;
    uint8_t  walk_tia = 0;
    uint8_t  walk_tib = 0;
    uint8_t  walk_acc_wp = 0;
    uint8_t  walk_l1_need_u = 0;
    uint8_t  walk_l2_need_u = 0;
    uint8_t  walk_leaf_need_u = 0;
    uint8_t  walk_leaf_need_m = 0;
    uint8_t  walk_up_phase = 0;
    uint8_t  walk_last_fault = 0;
    uint8_t  walk_last_ok = 0;
    uint8_t  walk_last_fc = 0;
    uint8_t  walk_out_fc = 0;
    uint32_t walk_lat_va = 0;
    uint32_t walk_lat_root = 0;
    uint32_t walk_l1_addr = 0;
    uint32_t walk_l1_pte = 0;
    uint32_t walk_l2_addr = 0;
    uint32_t walk_l2_pte = 0;
    uint32_t walk_leaf_addr = 0;
    uint32_t walk_leaf_pte = 0;
    uint32_t walk_leaf_entry_addr = 0;
    uint32_t walk_last_pa = 0;
    uint32_t walk_last_fa = 0;

    uint8_t  cdb1_en = 0;
    uint8_t  cdb1_has_dst = 0;
    uint8_t  cdb1_phys = 0;
    uint32_t cdb1_data = 0;

    uint8_t  cmpl1_en = 0;
    uint8_t  cmpl1_tag = 0;
    uint8_t  cmpl1_br_taken = 0;
    uint32_t cmpl1_br_target = 0;

    uint8_t  rob_is_branch = 0;
    uint8_t  rob_branch_taken = 0;
    uint32_t rob_branch_target = 0;
    uint32_t commit_actual_next = 0;

    uint8_t  daxi_arvalid = 0;
    uint8_t  daxi_arready = 0;
    uint32_t daxi_araddr = 0;
    uint8_t  daxi_rvalid = 0;
    uint8_t  daxi_rready = 0;
    uint8_t  daxi_rresp = 0;
    uint8_t  daxi_awvalid = 0;
    uint8_t  daxi_awready = 0;
    uint32_t daxi_awaddr = 0;
    uint8_t  daxi_wvalid = 0;
    uint8_t  daxi_wready = 0;
    uint8_t  daxi_bvalid = 0;
    uint8_t  daxi_bready = 0;
    uint8_t  daxi_bresp = 0;

    uint8_t  if_req = 0;
    uint32_t if_addr = 0;
    uint8_t  if_rvalid = 0;
    uint8_t  if_pending = 0;
};

static std::vector<LastNTraceSample> lastn_trace_ring;
static size_t lastn_trace_next = 0;
static size_t lastn_trace_count = 0;

// ─── Small utilities ──────────────────────────────────────────────────
// Forward-decl for log_trace (defined later, after cpu_read8).
static uint8_t cpu_read8(uint32_t a);
static uint32_t peek_data32(uint32_t a);
static void log_trace(uint32_t pc, uint8_t ccr) {
    // Peek instruction word at PC through cpu_read8 so we get the
    // bootstrap override, overlay aliasing, and ROM mirror correctly.
    uint16_t ir = ((uint16_t)cpu_read8(pc) << 8) | cpu_read8(pc + 1);
    if (trace_fp) {
        std::fprintf(trace_fp, "%08x %04x %02x\n",
                     pc, ir, ccr & 0x1Fu);
    }
}

static uint32_t read_arch_reg(int idx) {
    auto* r = dut->rootp;
    unsigned phys = r->mac_top__DOT__cpu__DOT__u_rat__DOT__crat[idx];
    return r->mac_top__DOT__cpu__DOT__prf[phys];
}

static uint32_t peek_cpu32(uint32_t a) {
    return ((uint32_t)cpu_read8(a)     << 24)
         | ((uint32_t)cpu_read8(a + 1) << 16)
         | ((uint32_t)cpu_read8(a + 2) <<  8)
         |  (uint32_t)cpu_read8(a + 3);
}

static const char* rom_probe_pc_name(uint32_t pc) {
    switch (pc) {
        case 0x40846f1cu: return "frame_probe_entry";
        case 0x40846f1eu: return "frame_sig_cmp";
        case 0x40846f28u: return "frame_sig_miss";
        case 0x40846f2cu: return "frame_status_store";
        case 0x40846f30u: return "frame_load_a2";
        case 0x40846f34u: return "frame_load_d0";
        case 0x40846f38u: return "frame_load_a1";
        case 0x40846f3cu: return "frame_load_a0";
        case 0x40846a80u: return "buserr_entry";
        case 0x4084712eu: return "frame_setup_entry";
        case 0x40847138u: return "frame_suba_alloc";
        case 0x40847142u: return "frame_copy_sig";
        case 0x40847148u: return "frame_save_d0";
        case 0x4084714cu: return "frame_save_a1";
        case 0x40847150u: return "frame_save_a0";
        case 0x40847154u: return "frame_clear_0c";
        case 0x4084715au: return "frame_clear_08";
        case 0x40847160u: return "frame_clear_04";
        case 0x40847166u: return "frame_save_d3";
        case 0x40847170u: return "frame_pick_table";
        case 0x40847176u: return "frame_save_worker";
        case 0x40847188u: return "frame_dispatch";
        case 0x4084a7d4u: return "rom_monitor_entry";
        case 0x4084a7e2u: return "rom_monitor_clear";
        case 0x4084a7f0u: return "rom_monitor_hwdesc";
        case 0x4084a7f8u: return "rom_monitor_stm_check";
        case 0x4084a802u: return "rom_monitor_diag_check";
        case 0x4084a812u: return "rom_monitor_scc_check";
        case 0x4084a82eu: return "rom_monitor_banner";
        case 0x4084a838u: return "rom_monitor_cte_banner";
        case 0x4084a840u: return "rom_monitor_poll";
        case 0x4084a848u: return "rom_monitor_rx_result";
        case 0x4084a966u: return "rom_monitor_idle";
        case 0x4084aa08u: return "rom_monitor_loop";
        case 0x4084af9cu: return "rom_scc_rx_poll";
        case 0x4084afa6u: return "rom_scc_rr0_test";
        case 0x4084afcau: return "rom_scc_rx_return";
        case 0x4080b1bcu: return "xpram_byte_load";
        case 0x4080b1c0u: return "xpram_byte_store";
        case 0x4080b1c2u: return "xpram_loop_state";
        case 0x4080b1d8u: return "xpram_read_done";
        case 0x4084bb74u: return "memsize_entry";
        case 0x4084bb8eu: return "pop_a0";
        case 0x4084bb96u: return "pop_a1";
        case 0x4084bb9cu: return "lane_probe_jump";
        case 0x4084bba0u: return "lane_probe_return";
        case 0x4084bbb4u: return "alias_probe_jump";
        case 0x4084bbb8u: return "alias_probe_return";
        case 0x4084bbbcu: return "final_stack_delta";
        case 0x4084bbc6u: return "final_d6_test";
        case 0x4084bbcau: return "sad_path_jump";
        case 0x4084bbccu: return "alias_probe_entry";
        case 0x4084bbd0u: return "alias_probe_read";
        case 0x4084bbd8u: return "alias_probe_step";
        case 0x4084bc02u: return "alias_probe_exit";
        case 0x4084bc04u: return "lane_probe_read";
        case 0x4084bc36u: return "lane_probe_exit";
        case 0x4084bc38u: return "memsize_success";
        default:          return nullptr;
    }
}

static uint64_t rom_probe_pc_hit_limit(uint32_t pc) {
    if ((pc >= 0x40846f1cu && pc <= 0x40846f3cu) ||
        (pc >= 0x4084712eu && pc <= 0x40847188u)) return 32;
    if (pc >= 0x4084a7d4u && pc <= 0x4084aa08u) return 8;
    if (pc >= 0x4084af9cu && pc <= 0x4084afcau) return 16;
    if (pc >= 0x4080b1bcu && pc <= 0x4080b1d8u) return 512;
    return 0;
}

static bool rom_probe_pc_is_frame(uint32_t pc) {
    return (pc >= 0x40846f1cu && pc <= 0x40846f3cu) ||
           (pc >= 0x4084712eu && pc <= 0x40847188u);
}

static void maybe_log_rom_probe(uint32_t pc) {
    if (!probe_fp || probe_log_emitted >= probe_log_limit) return;
    const char* name = rom_probe_pc_name(pc);
    if (!name) return;
    uint64_t& pc_hits = probe_pc_hits[pc];
    uint64_t pc_hit_limit = rom_probe_pc_hit_limit(pc);
    if (pc_hit_limit != 0 && pc_hits >= pc_hit_limit) return;
    pc_hits++;

    uint32_t d0 = read_arch_reg(0);
    uint32_t d1 = read_arch_reg(1);
    uint32_t d2 = read_arch_reg(2);
    uint32_t d3 = read_arch_reg(3);
    uint32_t d4 = read_arch_reg(4);
    uint32_t d5 = read_arch_reg(5);
    uint32_t d6 = read_arch_reg(6);
    uint32_t d7 = read_arch_reg(7);
    uint32_t a0 = read_arch_reg(8);
    uint32_t a1 = read_arch_reg(9);
    uint32_t a2 = read_arch_reg(10);
    uint32_t a3 = read_arch_reg(11);
    uint32_t a4 = read_arch_reg(12);
    uint32_t a5 = read_arch_reg(13);
    uint32_t a6 = read_arch_reg(14);
    uint32_t sp = read_arch_reg(15);

    const char* expected_touch = "-";
    uint32_t touch = 0;
    if (pc == 0x4084bb9cu || pc == 0x4084bc04u) {
        expected_touch = "a0";
        touch = a0;
    } else if (pc == 0x4084bbb4u || pc == 0x4084bbccu ||
               pc == 0x4084bbd0u || pc == 0x4084bbd8u) {
        expected_touch = "a1";
        touch = a1;
    } else if (pc >= 0x4080b1bcu && pc <= 0x4080b1d8u) {
        expected_touch = "a1";
        touch = a1;
    } else if (pc == 0x4084bb8eu || pc == 0x4084bb96u ||
               pc == 0x4084bb74u) {
        expected_touch = "sp";
        touch = sp;
    } else if (pc == 0x40846a80u) {
        expected_touch = "fault";
        touch = dut->rootp->mac_top__DOT__cpu__DOT__u_exception__DOT__cur_fault_addr;
    }

    std::fprintf(probe_fp,
        "[probe] cyc=%llu committed=%u pc=0x%08x %-22s hit=%llu "
        "d0=0x%08x d1=0x%08x d2=0x%08x d3=0x%08x d4=0x%08x "
        "d5=0x%08x d6=0x%08x d7=0x%08x "
        "a0=0x%08x a1=0x%08x a2=0x%08x a3=0x%08x a4=0x%08x "
        "a5=0x%08x a6=0x%08x sp=0x%08x "
        "touch=%s:0x%08x resp=%u "
        "mem[a0]=0x%08x mem[a1]=0x%08x mem[a5]=0x%08x mem[a5+4]=0x%08x "
        "mem[sp]=0x%08x mem[sp+4]=0x%08x "
        "exc_vec=%u exc_fault_pc=0x%08x exc_fault_addr=0x%08x\n",
        (unsigned long long)sim_time,
        (unsigned)dut->dbg_committed,
        pc,
        name,
        (unsigned long long)pc_hits,
        d0, d1, d2, d3, d4, d5, d6, d7,
        a0, a1, a2, a3, a4, a5, a6, sp,
        expected_touch, touch, (unsigned)daxi_rresp_for_addr(touch),
        peek_data32(a0), peek_data32(a1), peek_data32(a5), peek_data32(a5 + 4),
        peek_data32(sp), peek_data32(sp + 4),
        (unsigned)dut->rootp->mac_top__DOT__cpu__DOT__u_exception__DOT__cur_vec,
        (unsigned)dut->rootp->mac_top__DOT__cpu__DOT__u_exception__DOT__cur_fault_pc,
        (unsigned)dut->rootp->mac_top__DOT__cpu__DOT__u_exception__DOT__cur_fault_addr);

    if (rom_probe_pc_is_frame(pc)) {
        std::fprintf(probe_fp,
            "[frame] cyc=%llu committed=%u pc=0x%08x %-22s "
            "a2[-4]=0x%08x a4[0]=0x%08x a5[0]=0x%08x "
            "a5[4]=0x%08x a5[8]=0x%08x a5[0c]=0x%08x "
            "a5[12]=0x%08x a5[18]=0x%08x a5[1e]=0x%08x "
            "a5[22]=0x%08x a5[26]=0x%08x a5[2a]=0x%08x "
            "a5[2e]=0x%08x sp[0]=0x%08x sp[4]=0x%08x\n",
            (unsigned long long)sim_time,
            (unsigned)dut->dbg_committed,
            pc,
            name,
            peek_data32(a2 - 4),
            peek_data32(a4),
            peek_data32(a5),
            peek_data32(a5 + 4),
            peek_data32(a5 + 8),
            peek_data32(a5 + 12),
            peek_data32(a5 + 18),
            peek_data32(a5 + 24),
            peek_data32(a5 + 30),
            peek_data32(a5 + 34),
            peek_data32(a5 + 38),
            peek_data32(a5 + 42),
            peek_data32(a5 + 46),
            peek_data32(sp),
            peek_data32(sp + 4));
    }

    probe_log_emitted++;
    if (probe_log_emitted == probe_log_limit) {
        std::fprintf(probe_fp,
            "# probe log limit reached at %llu events; suppressing further detail\n",
            (unsigned long long)probe_log_limit);
    }
}

// Classify a data-side address into {RAM, ROM, IO, UNMAPPED}.  Legacy ROM
// runs keep low-memory data accesses pointed at RAM while overlay is asserted;
// +strict_overlay instead matches the RTL glue path and aliases those data
// accesses to ROM until VIA1 explicitly clears overlay.
enum AddrClass { AC_RAM, AC_ROM, AC_IO, AC_UNMAPPED };
static AddrClass classify(uint32_t a) {
    if (overlay_rom_for_data(a))      return AC_ROM;
    if (a < visible_ram_size)         return AC_RAM;
    // Low-memory decode remains reserved out to 1 GiB.  Above the visible
    // RAM window we model open-bus behavior (AC_UNMAPPED) so size probes
    // see absent memory as faults/unmapped.
    if (a < Q700_RAM_DECODE_LIMIT)    return AC_UNMAPPED;
    if (a >= Q700_ROM_BASE && a < (Q700_ROM_BASE + Q700_ROM_SIZE)) return AC_ROM;
    // Entire 0x4xxx_xxxx decode is ROM on Q700 (address bits [23:0]
    // mask into the 1 MB ROM image).  We'll handle the mask in the
    // read path.
    if (a >= 0x40000000u && a < 0x50000000u) return AC_ROM;
    if (in_range(a, IO_BASE, IO_SIZE)) return AC_IO;
    // DAFB register window 0xF9800000..0xF9800FFF is served by the
    // in-RTL shim (rtl/mac/video.v) via mac_top's daxi intercept
    // (task #143).  No tb-side override needed for register reads.
    // MAME macqd700 maps the DAFB VRAM aperture at 0xF9000000..0xF91FFFFF.
    // Keep only that range RAM-like so ROM framebuffer writes persist in the
    // harness; the rest of 0xF9xxxxxx remains open bus unless RTL intercepts it.
    if (is_dafb_reg_addr(a)) return AC_UNMAPPED;
    if (is_dafb_vram_addr(a)) return AC_RAM;
    if (a >= 0xFFFF0000u && a < 0xFFFF0010u) return AC_RAM; // sim sentinel
    return AC_UNMAPPED;
}

static inline bool ifetch_overlay_rom(uint32_t a) {
    return overlay_rom_for_ifetch(a);
}

// Compute ROM-image offset from a CPU address in the ROM class.
static inline uint32_t rom_offset_of(uint32_t a) {
    if (ifetch_overlay_rom(a))
        return a & (Q700_ROM_SIZE - 1);
    // All of 0x4xxx_xxxx is a ROM mirror per the Q700 decoder.  Mask
    // down to the 1 MB image.
    return (a - Q700_ROM_BASE) & (Q700_ROM_SIZE - 1);
}

// Named ROM patches are opt-in and harness-local: they mutate only this
// process's rom_img buffer after loading files/420dbff3.rom.  Keep them out
// of default runs so parity/debug work still sees unmodified ROM behavior.
//
// Do not revive descriptor-selection patches here.  The harness now models
// enough Q700 probe behavior that the ROM must select the Q700 descriptor
// naturally.  These patches are only for skipping known-deep diagnostic loops
// while iterating on later boot frontiers.
static std::vector<RomPatchByte> active_rom_patches;

static bool rom_patch_set_enables_ramtest_mame_state_fastfill(
        const std::string& set) {
    return set == "ramtest-mame-state" || set == "mame-ramtest-fast" ||
           set == "mame-fastdiag" || set == "mame-q700-fastdiag" ||
           set == "mame-firstlight" || set == "mame-q700-firstlight";
}

static void add_active_rom_patch_byte(const RomPatchByte& patch) {
    for (auto& p : active_rom_patches) {
        if (p.off == patch.off) {
            if (p.value != patch.value) {
                std::fprintf(stderr,
                    "[rom-boot] conflicting ROM patch at 0x%05x: "
                    "0x%02x from %s vs 0x%02x from %s\n",
                    patch.off, p.value, p.set, patch.value, patch.set);
                std::exit(2);
            }
            return;
        }
    }
    active_rom_patches.push_back(patch);
}

static void configure_rom_patches() {
    for (const auto& set : requested_rom_patch_sets) {
        std::vector<RomPatchByte> new_patches;
        if (!add_rom_patch_set(new_patches, set, "[rom-boot]")) {
            std::exit(2);
        }
        if (rom_patch_set_enables_ramtest_mame_state_fastfill(set))
            ramtest_mame_state_fastfill = true;
        for (const auto& patch : new_patches)
            add_active_rom_patch_byte(patch);
    }
}

static bool apply_active_rom_patches() {
    if (active_rom_patches.empty()) return true;

    std::sort(active_rom_patches.begin(), active_rom_patches.end(),
              [](const RomPatchByte& a, const RomPatchByte& b) {
                  return a.off < b.off;
              });
    std::fprintf(stderr, "[rom-boot] applying %zu harness ROM patch bytes\n",
                 active_rom_patches.size());
    for (const auto& p : active_rom_patches) {
        if (p.off >= rom_img.size()) {
            std::fprintf(stderr,
                "[rom-boot] ROM patch out of range: %s offset=0x%05x size=%zu\n",
                p.set, p.off, rom_img.size());
            return false;
        }
        uint8_t old = rom_img[p.off];
        rom_img[p.off] = p.value;
        std::fprintf(stderr,
            "[rom-boot]   +rom_patch=%s off=0x%05x old=0x%02x new=0x%02x %s\n",
            p.set, p.off, old, p.value, p.description);
    }
    return true;
}

static bool apply_rom_patches() {
    configure_rom_patches();
    return apply_active_rom_patches();
}

static inline uint8_t rom_read8(uint32_t off) {
    if (off < rom_img.size()) return rom_img[off];
    return 0xFF;
}

// Read a byte from memory at CPU address `a`, honoring overlay + ROM
// aliasing.  Unmapped reads return 0xFF (matches mem_model).
// Task #139: the synthetic bootstrap trampoline that used to live at
// the core's hard-wired RESET_PC (0x4080_0000) is gone — we now
// elaborate mac_top with RESET_PC=0x4000_002A so the CPU fetches the
// real ROM prologue on cycle zero, matching the FPGA boot path.
static uint8_t cpu_read8(uint32_t a) {
    if (ifetch_overlay_rom(a)) return rom_read8(rom_offset_of(a));
    AddrClass c = classify(a);
    if (c == AC_ROM) return rom_read8(rom_offset_of(a));
    if (c == AC_RAM) return mem->read8(a);
    return 0xFF;
}

static uint8_t peek_data8(uint32_t a) {
    AddrClass c = classify(a);
    if (c == AC_ROM) return rom_read8(rom_offset_of(a));
    if (c == AC_RAM) return mem->read8(a);
    return 0xFF;
}

static uint32_t peek_data32(uint32_t a) {
    return ((uint32_t)peek_data8(a)     << 24)
         | ((uint32_t)peek_data8(a + 1) << 16)
         | ((uint32_t)peek_data8(a + 2) <<  8)
         |  (uint32_t)peek_data8(a + 3);
}

struct DataWatchRange {
    uint32_t start = 0;
    uint32_t end = 0;  // inclusive
};

static std::vector<DataWatchRange> data_watch_ranges;
static uint64_t data_watch_hits = 0;
static uint64_t data_watch_emitted = 0;
static bool data_watch_suppressed = false;

static bool data_watch_enabled() {
    return !data_watch_ranges.empty();
}

static void data_watch_log_access(bool write,
                                  uint32_t addr,
                                  uint32_t strb,
                                  uint32_t data,
                                  uint32_t resp,
                                  const char* source,
                                  const char* bus);

struct CacheEventStats {
    uint64_t requests = 0;
    uint64_t dones = 0;
    uint64_t snoops = 0;
    uint64_t lowmem_writes = 0;
};

static CacheEventStats cache_event_stats;
static uint64_t cache_event_emitted = 0;
static bool cache_event_suppressed = false;
static bool cache_event_prev_req = false;
static bool cache_event_prev_wait = false;
static bool cache_event_prev_flush_done = false;
static bool cache_event_prev_snoop = false;

static inline bool cache_event_lowmem_addr(uint32_t a) {
    return a <= 0x000005ffu;
}

static const char* cache_scope_name(uint8_t scope) {
    switch (scope & 0x3u) {
        case 1: return "LINE";
        case 2: return "PAGE";
        case 3: return "ALL";
        default: return "NONE";
    }
}

static const char* cache_target_name(uint8_t caches) {
    switch (caches & 0x3u) {
        case 1: return "DC";
        case 2: return "IC";
        case 3: return "BC";
        default: return "NONE";
    }
}

static void write_cache_event(FILE* out,
                              const char* event,
                              uint32_t addr,
                              uint32_t value,
                              const std::string& detail) {
    if (!out) return;
    std::fprintf(out,
        "cycle=%llu committed=%llu pc=0x%08x last_pc=0x%08x event=%s addr=0x%08x value=0x%08x",
        (unsigned long long)sim_time,
        (unsigned long long)(dut ? dut->dbg_committed : 0),
        dut ? (unsigned)dut->dbg_pc : 0,
        dut ? (unsigned)dut->dbg_last_pc : 0,
        event ? event : "unknown",
        addr,
        value);
    if (!detail.empty())
        std::fprintf(out, " detail=%s", detail.c_str());
    std::fprintf(out, "\n");
}

static void cache_event_record(const char* event,
                               uint32_t addr,
                               uint32_t value,
                               const std::string& detail,
                               bool force = false) {
    if (!cache_event_fp) return;
    if (cache_event_emitted < cache_event_log_limit) {
        write_cache_event(cache_event_fp, event, addr, value, detail);
        cache_event_emitted++;
        return;
    }
    if (!cache_event_suppressed) {
        std::fprintf(cache_event_fp,
            "# cache event log limit reached at %llu events; continuing summary only\n",
            (unsigned long long)cache_event_log_limit);
        cache_event_suppressed = true;
    }
    if (force)
        write_cache_event(cache_event_fp, event, addr, value, detail);
}

static void cache_event_log_lowmem_table(FILE* out) {
    if (!out) return;
    std::fprintf(out,
        "# low-memory snapshot 0x00000000..0x000005ff (words, big-endian)\n");
    std::fprintf(out,
        "# columns: addr w0 w1 w2 w3\n");
    for (uint32_t a = 0x00000000u; a <= 0x000005ffu; a += 16) {
        std::fprintf(out, "lowmem 0x%08x", a);
        for (int i = 0; i < 4; i++)
            std::fprintf(out, " 0x%08x", peek_data32(a + (uint32_t)(i * 4)));
        std::fprintf(out, "\n");
    }
}

static void dump_cache_event_summary() {
    if (!cache_event_fp) return;
    if (cache_event_fp != stderr) {
        std::fprintf(stderr,
            "[rom-boot] cache-event log at %s (limit=%llu)\n",
            cache_event_log_path.c_str(),
            (unsigned long long)cache_event_log_limit);
    }
    std::fprintf(cache_event_fp,
        "\n-------- rom-boot cache frontier activity --------\n");
    std::fprintf(cache_event_fp,
        "[rom-boot] cache-event summary: requests=%llu dones=%llu snoops=%llu lowmem_writes=%llu detail=%s limit=%llu\n",
        (unsigned long long)cache_event_stats.requests,
        (unsigned long long)cache_event_stats.dones,
        (unsigned long long)cache_event_stats.snoops,
        (unsigned long long)cache_event_stats.lowmem_writes,
        cache_event_fp ? "enabled" : "disabled",
        (unsigned long long)cache_event_log_limit);
    if (cache_event_suppressed) {
        std::fprintf(cache_event_fp,
            "[rom-boot] cache event log was capped at %llu events\n",
            (unsigned long long)cache_event_log_limit);
    }
    cache_event_log_lowmem_table(cache_event_fp);
    std::fprintf(cache_event_fp,
        "------------------------------------------------\n");
    std::fflush(cache_event_fp);
}

static void sample_cache_frontier_activity() {
    if (!dut || !dut->rootp) return;
    if (!cache_event_fp && !data_watch_enabled()) return;
    auto* r = dut->rootp;

    const bool req = r->mac_top__DOT__cpu__DOT__cache_maint_req_w;
    const bool req_q = r->mac_top__DOT__cpu__DOT__cache_maint_req_q;
    const bool wait = r->mac_top__DOT__cpu__DOT__u_commit__DOT__cache_maint_wait;
    const bool flush_done = r->mac_top__DOT__cpu__DOT__dcache_flush_done_raw;
    const bool dcache_req = r->mac_top__DOT__cpu__DOT__dcache_maint_req;
    const bool snoop = r->mac_top__DOT__cpu__DOT__dc_snoop_valid;
    const uint32_t snoop_addr = (uint32_t)r->mac_top__DOT__cpu__DOT__dc_snoop_addr;
    const uint32_t req_addr = (uint32_t)r->mac_top__DOT__cpu__DOT__cache_maint_addr_w;
    const uint8_t req_scope = (uint8_t)r->mac_top__DOT__cpu__DOT__cache_maint_scope_w;
    const uint8_t req_caches = (uint8_t)r->mac_top__DOT__cpu__DOT__cache_maint_caches_w;
    const bool req_is_inv = r->mac_top__DOT__cpu__DOT__cache_maint_is_inv_w;

    if (req && !cache_event_prev_req) {
        cache_event_stats.requests++;
        std::string detail;
        detail.reserve(192);
        detail = "req=1";
        detail += req_is_inv ? " inv=1" : " inv=0";
        detail += " scope=";
        detail += cache_scope_name(req_scope);
        detail += " caches=";
        detail += cache_target_name(req_caches);
        detail += " req_q=";
        detail += req_q ? "1" : "0";
        detail += " dcache_req=";
        detail += dcache_req ? "1" : "0";
        detail += " wait=";
        detail += wait ? "1" : "0";
        detail += " flush_done=";
        detail += flush_done ? "1" : "0";
        detail += " lowmem=";
        detail += cache_event_lowmem_addr(req_addr) ? "1" : "0";
        cache_event_record("cache-maint-req", req_addr, req_caches, detail);
    }
    if (wait != cache_event_prev_wait) {
        cache_event_stats.dones++;
        std::string detail;
        detail = "wait=";
        detail += wait ? "1" : "0";
        detail += " dcache_req=";
        detail += dcache_req ? "1" : "0";
        detail += " flush_done=";
        detail += flush_done ? "1" : "0";
        cache_event_record(wait ? "cache-maint-wait" : "cache-maint-done",
                           req_addr, req_caches, detail);
    } else if (flush_done && !cache_event_prev_flush_done) {
        cache_event_stats.dones++;
        cache_event_record("cache-maint-flush-done", req_addr, req_caches,
                           "flush_done=1");
    }
    if (snoop) {
        cache_event_stats.snoops++;
        const bool lowmem = cache_event_lowmem_addr(snoop_addr);
        char dcache_detail[384];
        const uint32_t lat_addr =
            (uint32_t)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__lat_addr;
        const uint32_t lat_wdata =
            (uint32_t)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__lat_wdata;
        const uint32_t lat_wstrb =
            (uint32_t)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__lat_wstrb;
        const uint32_t lsu_ea =
            (uint32_t)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__ea;
        const uint32_t lsu_dc_addr =
            (uint32_t)r->mac_top__DOT__cpu__DOT__dc_addr;
        if (lowmem)
            cache_event_stats.lowmem_writes++;
        data_watch_log_access(true, lat_addr & ~0x3u, lat_wstrb, lat_wdata,
                              0, "dcache", "dcache-snoop");
        std::snprintf(dcache_detail, sizeof(dcache_detail),
            "snoop=1 lowmem=%u dcache_state=%u lat_is_write=%u "
            "lat_addr=0x%08x lat_line=0x%08x lat_wdata=0x%08x "
            "lat_wstrb=0x%x lsu_state=%u lsu_ea=0x%08x "
            "lsu_dc_addr=0x%08x rob_pc=0x%08x",
            lowmem ? 1u : 0u,
            (unsigned)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__state,
            (unsigned)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__lat_is_write,
            lat_addr,
            lat_addr & ~0x1fu,
            lat_wdata,
            lat_wstrb,
            (unsigned)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__state,
            lsu_ea,
            lsu_dc_addr,
            (unsigned)r->mac_top__DOT__cpu__DOT__rob_pc);
        std::string detail = dcache_detail;
        cache_event_record("dc-snoop", snoop_addr, 0, detail, lowmem);
    }

    cache_event_prev_req = req;
    cache_event_prev_wait = wait;
    cache_event_prev_flush_done = flush_done;
    cache_event_prev_snoop = snoop;
}

static std::string trim_token(const std::string& s) {
    const size_t first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return std::string();
    const size_t last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

static uint32_t parse_watch_u32(const std::string& tok,
                                const char* arg_name) {
    size_t consumed = 0;
    uint64_t value = 0;
    try {
        value = std::stoull(tok, &consumed, 0);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[rom-boot] bad %s value '%s': %s\n",
                     arg_name, tok.c_str(), e.what());
        std::exit(2);
    }
    if (consumed != tok.size() || value > 0xffffffffULL) {
        std::fprintf(stderr, "[rom-boot] bad %s value '%s'\n",
                     arg_name, tok.c_str());
        std::exit(2);
    }
    return (uint32_t)value;
}

static void add_data_watch_range(uint32_t start, uint32_t end) {
    if (end < start) {
        std::fprintf(stderr,
            "[rom-boot] +data_watch range end before start: 0x%08x..0x%08x\n",
            start, end);
        std::exit(2);
    }
    data_watch_ranges.push_back({start, end});
}

static void parse_data_watch_list(const std::string& spec) {
    std::stringstream ss(spec);
    std::string raw;
    while (std::getline(ss, raw, ',')) {
        std::string tok = trim_token(raw);
        if (tok.empty()) continue;

        size_t sep = tok.find("..");
        if (sep != std::string::npos) {
            uint32_t start = parse_watch_u32(trim_token(tok.substr(0, sep)),
                                             "+data_watch");
            uint32_t end = parse_watch_u32(trim_token(tok.substr(sep + 2)),
                                           "+data_watch");
            add_data_watch_range(start, end);
            continue;
        }

        sep = tok.find('+');
        if (sep != std::string::npos) {
            uint32_t start = parse_watch_u32(trim_token(tok.substr(0, sep)),
                                             "+data_watch");
            uint32_t len = parse_watch_u32(trim_token(tok.substr(sep + 1)),
                                           "+data_watch");
            if (len == 0 || start > 0xffffffffu - (len - 1)) {
                std::fprintf(stderr,
                    "[rom-boot] bad +data_watch length in '%s'\n",
                    tok.c_str());
                std::exit(2);
            }
            add_data_watch_range(start, start + len - 1);
            continue;
        }

        sep = tok.find('-', 1);
        if (sep != std::string::npos) {
            uint32_t start = parse_watch_u32(trim_token(tok.substr(0, sep)),
                                             "+data_watch");
            uint32_t end = parse_watch_u32(trim_token(tok.substr(sep + 1)),
                                           "+data_watch");
            add_data_watch_range(start, end);
            continue;
        }

        uint32_t addr = parse_watch_u32(tok, "+data_watch");
        add_data_watch_range(addr, addr);
    }

    std::sort(data_watch_ranges.begin(), data_watch_ranges.end(),
              [](const DataWatchRange& a, const DataWatchRange& b) {
                  if (a.start != b.start) return a.start < b.start;
                  return a.end < b.end;
              });
    data_watch_ranges.erase(
        std::unique(data_watch_ranges.begin(), data_watch_ranges.end(),
                    [](const DataWatchRange& a, const DataWatchRange& b) {
                        return a.start == b.start && a.end == b.end;
                    }),
        data_watch_ranges.end());
}

static void parse_periph_event_filter(const std::string& spec) {
    std::stringstream ss(spec);
    std::string raw;
    while (std::getline(ss, raw, ',')) {
        std::string tok = trim_token(raw);
        if (!tok.empty())
            periph_event_filter.push_back(tok);
    }
}

static const char* periph_cat_name(PeriphCat cat) {
    switch (cat) {
        case PC_VIA1:     return "via1";
        case PC_VIA2:     return "via2";
        case PC_SCC:      return "scc";
        case PC_SCSI:     return "scsi";
        case PC_ASC:      return "asc";
        case PC_DAFB_REG: return "dafb-reg";
        case PC_VRAM:     return "dafb-vram";
        case PC_ROM:      return "rom";
        case PC_RAM:      return "ram";
        case PC_UNMAPPED: return "unmapped";
        case PC_COUNT:    break;
    }
    return "unknown";
}

static const char* via1_reg_name(uint8_t idx) {
    switch (idx & 0x0F) {
        case 0:  return "ORB";
        case 1:  return "ORA";
        case 2:  return "DDRB";
        case 3:  return "DDRA";
        case 4:  return "T1CL";
        case 5:  return "T1CH";
        case 6:  return "T1LL";
        case 7:  return "T1LH";
        case 8:  return "T2CL";
        case 9:  return "T2CH";
        case 10: return "SR(ADB)";
        case 11: return "ACR";
        case 12: return "PCR";
        case 13: return "IFR";
        case 14: return "IER";
        case 15: return "ORA-NH";
    }
    return "VIA1?";
}

static bool periph_detail_sink_active() {
    return periph_fp != nullptr && periph_log_limit != 0;
}

static void periph_detail_log(const char* kind,
                              bool write,
                              uint32_t addr,
                              uint32_t value,
                              uint32_t pc,
                              const char* note) {
    if (!periph_detail_sink_active()) return;
    if (periph_detail_emitted >= periph_log_limit) {
        if (!periph_detail_suppressed) {
            std::fprintf(periph_fp,
                "# periph log limit reached at %llu events; suppressing further detail\n",
                (unsigned long long)periph_log_limit);
            periph_detail_suppressed = true;
        }
        return;
    }
    std::fprintf(periph_fp,
        "[periph] cyc=%llu committed=%u %s %s pc=0x%08x addr=0x%08x value=0x%08x",
        (unsigned long long)sim_time,
        dut ? (unsigned)dut->dbg_committed : 0,
        kind,
        write ? "wr" : "rd",
        pc,
        addr,
        value);
    if (note != nullptr && note[0] != '\0') {
        std::fprintf(periph_fp, " note=%s", note);
    }
    std::fprintf(periph_fp, "\n");
    periph_detail_emitted++;
}

static void periph_record_access(PeriphCat cat,
                                 bool write,
                                 uint32_t addr,
                                 uint32_t value,
                                 uint32_t pc,
                                 const char* note = nullptr) {
    if (cat < 0 || cat >= PC_COUNT) return;
    PeriphStats& s = periph_stats[(size_t)cat];
    if (write) s.writes++;
    else       s.reads++;
    PeriphAccessSnapshot snap;
    snap.valid = true;
    snap.write = write;
    snap.cycle = sim_time;
    snap.committed = dut ? (uint64_t)dut->dbg_committed : 0;
    snap.pc = pc;
    snap.addr = addr;
    snap.value = value;
    if (!s.first.valid) s.first = snap;
    s.last = snap;
    periph_detail_log(periph_cat_name(cat), write, addr, value, pc, note ? note : "");
}

static const char* display_range_note(PeriphCat cat) {
    switch (cat) {
        case PC_DAFB_REG: return "DAFB registers 0xf9800000..0xf9800fff";
        case PC_VRAM:     return "DAFB pixel/VRAM aperture 0xf9000000..0xf91fffff";
        default:          return "";
    }
}

static const char* data_watch_source_name(uint32_t addr, AddrClass c) {
    if (is_dafb_reg_addr(addr)) return "dafb-reg";
    if (is_dafb_vram_addr(addr)) return "dafb-vram";
    if (c == AC_RAM) {
        return "ram";
    }
    if (c == AC_ROM) return "rom";
    if (c == AC_UNMAPPED) return "unmapped";

    uint32_t ca = q700_io_canonical(addr);
    if (in_range(ca, VIA1_BASE, VIA1_SIZE)) return "via1";
    if (in_range(ca, VIA2_BASE, VIA2_SIZE)) return "via2";
    if (in_range(ca, SCC_BASE, SCC_SIZE)) return "scc";
    if (in_range(ca, TURBOSCSI_BASE, TURBOSCSI_SIZE)) return "scsi";
    if (in_range(ca, ASC_BASE, ASC_SIZE)) return "asc";
    if (in_range(ca, SWIM_BASE, SWIM_SIZE)) return "swim";
    if (in_range(ca, ENET_BASE, ENET_SIZE)) return "enet";
    if (in_range(ca, SONIC_BASE, SONIC_SIZE)) return "sonic";
    if (in_range(ca, ORWELL_BASE, ORWELL_SIZE)) return "orwell";
    return "io";
}

static const char* data_watch_bus_owner() {
    if (!dut || !dut->rootp) return "unknown";
    auto* r = dut->rootp;
    if (r->mac_top__DOT__cpu__DOT__exc_active) return "exception";
    if (r->mac_top__DOT__cpu__DOT__dc_daxi_arvalid ||
        r->mac_top__DOT__cpu__DOT__dc_daxi_rready ||
        r->mac_top__DOT__cpu__DOT__dc_daxi_awvalid ||
        r->mac_top__DOT__cpu__DOT__dc_daxi_wvalid ||
        r->mac_top__DOT__cpu__DOT__dc_daxi_bready) return "dcache";
    return "cpu";
}

static unsigned data_watch_strb_popcount(uint32_t strb) {
    unsigned n = 0;
    for (unsigned i = 0; i < 4; i++) {
        if (strb & (1u << i)) n++;
    }
    return n;
}

static bool data_watch_access_matches(uint32_t addr,
                                      uint32_t strb,
                                      bool write,
                                      DataWatchRange* matched) {
    if (!data_watch_enabled()) return false;
    uint32_t first = 0xffffffffu;
    uint32_t last = 0;
    for (unsigned lane = 0; lane < 4; lane++) {
        uint32_t bit = 1u << (3 - lane);
        if (write && ((strb & bit) == 0)) continue;
        uint32_t byte_addr = addr + lane;
        if (byte_addr < first) first = byte_addr;
        if (byte_addr > last) last = byte_addr;
    }
    if (first == 0xffffffffu) return false;

    for (const auto& r : data_watch_ranges) {
        if (first <= r.end && last >= r.start) {
            if (matched) *matched = r;
            return true;
        }
    }
    return false;
}

static void data_watch_log_access(bool write,
                                  uint32_t addr,
                                  uint32_t strb,
                                  uint32_t data,
                                  uint32_t resp,
                                  const char* source,
                                  const char* bus) {
    DataWatchRange matched;
    if (!data_watch_access_matches(addr, strb, write, &matched)) return;

    data_watch_hits++;
    if (data_watch_limit == 0) return;

    FILE* out = data_watch_fp ? data_watch_fp : stderr;
    if (data_watch_emitted >= data_watch_limit) {
        if (!data_watch_suppressed) {
            std::fprintf(out,
                "# data watch log limit reached at %llu events; suppressing further detail\n",
                (unsigned long long)data_watch_limit);
            data_watch_suppressed = true;
        }
        return;
    }

    const unsigned size = write ? data_watch_strb_popcount(strb) : 4;
    const char* owner = data_watch_bus_owner();
    uint32_t pc = dut ? (uint32_t)dut->dbg_pc : 0;
    uint32_t last_pc = dut ? (uint32_t)dut->dbg_last_pc : 0;
    uint32_t rob_pc = 0;
    uint32_t lsu_ea = 0;
    uint32_t lsu_dc_addr = 0;
    uint32_t dcache_lat_addr = 0;
    if (dut && dut->rootp) {
        rob_pc = (uint32_t)dut->rootp->mac_top__DOT__cpu__DOT__rob_pc;
        lsu_ea = (uint32_t)dut->rootp->mac_top__DOT__cpu__DOT__u_lsu__DOT__ea;
        lsu_dc_addr = (uint32_t)dut->rootp->mac_top__DOT__cpu__DOT__dc_addr;
        dcache_lat_addr =
            (uint32_t)dut->rootp->mac_top__DOT__cpu__DOT__u_dcache__DOT__lat_addr;
    }

    std::fprintf(out,
        "[data-watch] cyc=%llu committed=%u op=%s bus=%s owner=%s "
        "source=%s pc=0x%08x last_pc=0x%08x rob_pc=0x%08x "
        "lsu_ea=0x%08x lsu_dc_addr=0x%08x dcache_lat_addr=0x%08x "
        "addr=0x%08x size=%u strb=0x%x data=0x%08x resp=%u "
        "match=0x%08x",
        (unsigned long long)sim_time,
        dut ? (unsigned)dut->dbg_committed : 0,
        write ? "wr" : "rd",
        bus,
        owner,
        source ? source : "unknown",
        pc,
        last_pc,
        rob_pc,
        lsu_ea,
        lsu_dc_addr,
        dcache_lat_addr,
        addr,
        size,
        strb & 0xFu,
        data,
        resp,
        matched.start);
    if (matched.end != matched.start)
        std::fprintf(out, "-0x%08x", matched.end);
    std::fprintf(out, "\n");
    data_watch_emitted++;
}

static bool init_data_watch_logger() {
    if (!data_watch_enabled()) return true;
    if (!data_watch_log_path.empty()) {
        data_watch_fp = std::fopen(data_watch_log_path.c_str(), "w");
        if (!data_watch_fp) {
            std::fprintf(stderr,
                "[rom-boot] cannot open data watch log file %s; using stderr\n",
                data_watch_log_path.c_str());
        }
    }

    FILE* out = data_watch_fp ? data_watch_fp : stderr;
    std::fprintf(out, "# rom-boot data watch log limit=%llu\n",
                 (unsigned long long)data_watch_limit);
    std::fprintf(out,
        "# columns: cyc committed op bus owner source pc last_pc rob_pc lsu_ea lsu_dc_addr dcache_lat_addr addr size strb data resp match\n");
    for (const auto& r : data_watch_ranges) {
        std::fprintf(out, "# watch 0x%08x", r.start);
        if (r.end != r.start) std::fprintf(out, "-0x%08x", r.end);
        std::fprintf(out, "\n");
    }
    std::fflush(out);

    std::fprintf(stderr, "[rom-boot] data watch:");
    for (const auto& r : data_watch_ranges) {
        std::fprintf(stderr, " 0x%08x", r.start);
        if (r.end != r.start) std::fprintf(stderr, "-0x%08x", r.end);
    }
    std::fprintf(stderr, " log=%s limit=%llu\n",
                 data_watch_fp ? data_watch_log_path.c_str() : "stderr",
                 (unsigned long long)data_watch_limit);
    return true;
}

static bool init_cache_event_logger() {
    if (cache_event_log_path.empty()) return true;
    cache_event_fp = std::fopen(cache_event_log_path.c_str(), "w");
    if (!cache_event_fp) {
        std::fprintf(stderr,
            "[rom-boot] cannot open cache event log file %s; using stderr\n",
            cache_event_log_path.c_str());
        cache_event_fp = stderr;
    }
    std::fprintf(cache_event_fp, "# rom-boot cache frontier event log limit=%llu\n",
                 (unsigned long long)cache_event_log_limit);
    std::fprintf(cache_event_fp,
        "# columns: cycle committed pc last_pc event addr value detail\n");
    std::fflush(cache_event_fp);
    std::fprintf(stderr, "[rom-boot] cache frontier log: %s (limit=%llu)\n",
                 cache_event_fp == stderr ? "stderr" : cache_event_log_path.c_str(),
                 (unsigned long long)cache_event_log_limit);
    return true;
}

static void dump_data_watch_summary() {
    if (!data_watch_enabled()) return;
    std::fprintf(stderr,
        "[rom-boot] data watch summary: ranges=%zu hits=%llu emitted=%llu log=%s\n",
        data_watch_ranges.size(),
        (unsigned long long)data_watch_hits,
        (unsigned long long)data_watch_emitted,
        data_watch_fp ? data_watch_log_path.c_str() : "stderr");
    if (data_watch_suppressed) {
        std::fprintf(stderr,
            "[rom-boot] data watch log was capped at %llu events\n",
            (unsigned long long)data_watch_limit);
    }
    if (data_watch_fp) std::fflush(data_watch_fp);
}

static void dump_periph_summary_line(PeriphCat cat) {
    const PeriphStats& s = periph_stats[(size_t)cat];
    uint64_t total = s.reads + s.writes;
    double cyc_rate = sim_time ? (1000.0 * (double)total / (double)sim_time) : 0.0;
    double inst_rate = (dut && dut->dbg_committed)
        ? (1000.0 * (double)total / (double)dut->dbg_committed) : 0.0;
    std::fprintf(stderr,
        "[rom-boot]   %-9s r=%llu w=%llu rate=%6.2f/kcyc %6.2f/kinst",
        periph_cat_name(cat),
        (unsigned long long)s.reads,
        (unsigned long long)s.writes,
        cyc_rate,
        inst_rate);
    if (s.first.valid) {
        std::fprintf(stderr,
            " first=%c pc=0x%08x addr=0x%08x val=0x%08x",
            s.first.write ? 'w' : 'r',
            s.first.pc,
            s.first.addr,
            s.first.value);
    }
    if (s.last.valid) {
        std::fprintf(stderr,
            " last=%c pc=0x%08x addr=0x%08x val=0x%08x",
            s.last.write ? 'w' : 'r',
            s.last.pc,
            s.last.addr,
            s.last.value);
    }
    std::fprintf(stderr, "\n");
}

static void dump_display_summary_line(PeriphCat cat) {
    const PeriphStats& s = periph_stats[(size_t)cat];
    std::fprintf(stderr,
        "[rom-boot] display %-9s %s r=%llu w=%llu",
        periph_cat_name(cat),
        display_range_note(cat),
        (unsigned long long)s.reads,
        (unsigned long long)s.writes);
    if (s.first.valid) {
        std::fprintf(stderr,
            " first=%c cycle=%llu committed=%llu pc=0x%08x addr=0x%08x value=0x%08x",
            s.first.write ? 'w' : 'r',
            (unsigned long long)s.first.cycle,
            (unsigned long long)s.first.committed,
            s.first.pc,
            s.first.addr,
            s.first.value);
    } else {
        std::fprintf(stderr, " first=none");
    }
    if (s.last.valid) {
        std::fprintf(stderr,
            " last=%c cycle=%llu committed=%llu pc=0x%08x addr=0x%08x value=0x%08x",
            s.last.write ? 'w' : 'r',
            (unsigned long long)s.last.cycle,
            (unsigned long long)s.last.committed,
            s.last.pc,
            s.last.addr,
            s.last.value);
    } else {
        std::fprintf(stderr, " last=none");
    }
    std::fprintf(stderr, "\n");
}

static void dump_display_summary() {
    std::fprintf(stderr, "\n-------- rom-boot display activity --------\n");
    dump_display_summary_line(PC_DAFB_REG);
    dump_display_summary_line(PC_VRAM);
    std::fprintf(stderr,
        "[rom-boot] display note: DAFB register traffic is observed separately from "
        "pixel/VRAM aperture traffic so early framebuffer writes do not hide register setup.\n");
    std::fprintf(stderr, "-------------------------------------------\n");
}

static void dump_periph_summary() {
    if (!dut) return;
    uint64_t total = 0;
    for (size_t i = 0; i < periph_stats.size(); i++) {
        total += periph_stats[i].reads + periph_stats[i].writes;
    }
    if (total == 0) return;

    std::fprintf(stderr, "\n──────── rom-boot peripheral activity ────────\n");
    std::fprintf(stderr,
        "[rom-boot] cycles=%llu committed=%u detail=%s limit=%llu\n",
        (unsigned long long)sim_time,
        (unsigned)dut->dbg_committed,
        periph_detail_sink_active() ? periph_log_path.c_str() : "disabled",
        (unsigned long long)periph_log_limit);
    dump_periph_summary_line(PC_VIA1);
    dump_periph_summary_line(PC_VIA2);
    dump_periph_summary_line(PC_SCC);
    dump_periph_summary_line(PC_SCSI);
    dump_periph_summary_line(PC_ASC);
    dump_periph_summary_line(PC_DAFB_REG);
    dump_periph_summary_line(PC_VRAM);
    dump_periph_summary_line(PC_ROM);
    dump_periph_summary_line(PC_RAM);
    dump_periph_summary_line(PC_UNMAPPED);
    std::fprintf(stderr,
        "[rom-boot] daxi-read-reqs total=%llu burst=%llu beats=%llu\n",
        (unsigned long long)daxi_ar_reqs,
        (unsigned long long)daxi_ar_burst_reqs,
        (unsigned long long)daxi_ar_burst_beats);
    std::fprintf(stderr,
        "[rom-boot] note: VIA1 reg10=SR(ADB serial), reg13=IFR, reg14=IER; "
        "RTC is only visible through VIA1 PB0/PB1/PB2 in this harness, not as a separate device.\n");
    std::fprintf(stderr,
        "[rom-boot] note: VBL visibility is reported through the VBL event category for VIA1 CA1/IER/IFR and Timer1 traffic.\n");
    if (periph_fp && periph_fp != stderr) {
        std::fflush(periph_fp);
        std::fprintf(stderr, "[rom-boot] peripheral detail log at %s\n",
                     periph_log_path.c_str());
    }
    if (periph_detail_suppressed) {
        std::fprintf(stderr,
            "[rom-boot] peripheral detail log was capped at %llu events\n",
            (unsigned long long)periph_log_limit);
    }
    std::fprintf(stderr, "───────────────────────────────────────────────\n");
}

// ─── Peripheral-stub AXI responders ───────────────────────────────────
// VIA1 registers are byte-wide but the ROM accesses them as 32-bit
// AXI reads with WSTRB selecting the byte lane.  Register stride on
// Q700 is 512 bytes: reg_index = (addr >> 9) & 0xF.

// ADB "empty-bus" shadow tick — called every host sim cycle via tick().
// If a shift was scheduled and its completion cycle has arrived, set
// IFR bit 2 (SR interrupt flag).  Cheap: one compare + one store.
static void via1_adb_tick(void) {
    if (via1.adb_shift_in_progress &&
        host_cycle >= via1.adb_shift_complete_cyc) {
        via1.ifr |= 0x04;                   // SR shift-complete flag
        via1.adb_shift_in_progress = false;
        if (periph_events.enabled()) {
            char detail[160];
            std::snprintf(detail, sizeof(detail),
                          "txn=%u,byte=0x%02x,ifr=0x%02x,acr=0x%02x,pcr=0x%02x",
                          via1.adb_transaction_id,
                          via1.adb_last_byte, via1.ifr, via1.acr, via1.pcr);
            periph_event_c("ADB", "shift_complete",
                           VIA1_BASE + (10u << 9), via1.adb_last_byte, detail);
            periph_event("ADB", "empty_bus",
                         VIA1_BASE + (10u << 9), 0xFFu,
                         std::string("idle_byte=0xff,source=shadow,txn=") +
                         std::to_string(via1.adb_transaction_id));
        }
    }
}

static void via1_timer_tick(void) {
    if (!via1.t1_running && !via1.t2_running) return;
    if (via1_timer_div == 0) via1_timer_div = 1;
    via1_timer_div_ctr++;
    if ((via1_timer_div_ctr % via1_timer_div) != 0) return;

    if (via1.t1_running) {
        if (via1.t1_counter == 0) via1.t1_counter = 1;
        via1.t1_counter--;
        via1.t1cl = (uint8_t)(via1.t1_counter & 0xFF);
        via1.t1ch = (uint8_t)((via1.t1_counter >> 8) & 0xFF);

        if (via1.t1_counter == 0) {
            // Timer 1 wrap: set IFR.T1 (bit 6).  If ACR says free-run,
            // reload from latch; otherwise stop.
            via1.ifr |= 0x40;
            if (periph_events.enabled()) {
                const bool freerun = (via1.acr & 0x40) != 0;
                char detail[160];
                std::snprintf(detail, sizeof(detail),
                              "ifr=0x%02x,ier=0x%02x,acr=0x%02x,mode=%s,latch=0x%04x,div=%llu",
                              via1.ifr, via1.ier, via1.acr,
                              freerun ? "freerun" : "oneshot",
                              (unsigned)via1.t1_latch,
                              (unsigned long long)via1_timer_div);
                periph_event_c("VBL", "t1_wrap", VIA1_BASE + (13u << 9), via1.ifr, detail);
            }

            if (via1.acr & 0x40) {
                uint16_t reload = via1.t1_latch ? via1.t1_latch : 1;
                via1.t1_counter = reload;
                via1.t1cl = (uint8_t)(via1.t1_counter & 0xFF);
                via1.t1ch = (uint8_t)((via1.t1_counter >> 8) & 0xFF);
            } else {
                via1.t1_running = false;
            }
        }
    }

    if (via1.t2_running && ((via1.acr & 0x20) == 0)) {
        if (via1.t2_counter == 0) via1.t2_counter = 1;
        via1.t2_counter--;
        via1.t2cl = (uint8_t)(via1.t2_counter & 0xFF);
        via1.t2ch = (uint8_t)((via1.t2_counter >> 8) & 0xFF);

        if (via1.t2_counter == 0) {
            via1.ifr |= 0x20;
            via1.t2_running = false;
            if (periph_events.enabled()) {
                char detail[160];
                std::snprintf(detail, sizeof(detail),
                              "ifr=0x%02x,ier=0x%02x,acr=0x%02x,mode=oneshot,latch=0x%04x,div=%llu",
                              via1.ifr, via1.ier, via1.acr,
                              (unsigned)via1.t2_latch,
                              (unsigned long long)via1_timer_div);
                periph_event_c("VIA1", "timer2_wrap", VIA1_BASE + (13u << 9), via1.ifr, detail);
            }
        }
    }
}

static bool run_via1_t1_selftest() {
    // This is a directed smoke for the ROM-harness VIA1 Timer1 stub, not a
    // ROM-boot progression check.  It should stay deterministic and fast.
    (void)init_periph_event_logger();

    // Reset just the fields this stub cares about.
    via1 = Via1{};
    overlay = true;
    rtc_sidechannel.reset(host_cycle);
    bool ok_orb_mux_reset = (via1_read_reg(0) & 0xE8u) == 0x08u;
    bool ok_overlay_reset = via1_overlay_active();
    via1_write_reg(0, 0x00);
    bool ok_overlay_ddr_gated = via1_overlay_active();
    via1_write_reg(2, 0x08);
    bool ok_overlay_clear = !overlay && !via1_overlay_active();

    via1.ifr = 0x00;
    via1.ier = 0x80;
    via1.acr = 0x00;
    via1.t1_latch = 0;
    via1.t1_counter = 0;
    via1.t1_running = false;
    via1.t2_latch = 0;
    via1.t2_counter = 0;
    via1.t2_running = false;
    via1_timer_div = 1;
    via1_timer_div_ctr = 0;

    // One-shot: program 4 ticks, expect IFR.T1 set.
    via1_write_reg(4, 0x04);  // T1CL
    via1_write_reg(5, 0x00);  // T1CH (start)
    {
        char detail[160];
        std::snprintf(detail, sizeof(detail),
                      "t1_latch=0x%04x,t1_counter=0x%04x,acr=0x%02x,div=%llu",
                      (unsigned)via1.t1_latch,
                      (unsigned)via1.t1_counter,
                      via1.acr,
                      (unsigned long long)via1_timer_div);
        periph_event_c("VIA1", "timer1_start", VIA1_BASE + (5u << 9), via1.t1ch, detail);
        periph_event_c("VBL",  "t1_start",     VIA1_BASE + (5u << 9), via1.t1ch, detail);
    }

    for (int i = 0; i < 4; i++) via1_timer_tick();
    bool ok_oneshot = (via1.ifr & 0x40) != 0;
    (void)via1_read_reg(4);          // T1CL read acknowledges IFR.T1
    bool ok_t1cl_ack = (via1.ifr & 0x40) == 0;

    via1.ifr |= 0x02;                // VBL/CA1 pending
    (void)via1_read_reg(1);          // ORA read acknowledges CA-side flags
    bool ok_ora_ack = (via1.ifr & 0x02) == 0;

    // Free-run: clear IFR.T1, start a 2-tick period, and expect the timer to
    // keep running with a reload after wrap.
    via1.ifr &= ~0x40;
    via1.acr |= 0x40;
    via1_write_reg(4, 0x02);
    via1_write_reg(5, 0x00);
    for (int i = 0; i < 2; i++) via1_timer_tick();
    bool ok_freerun = ((via1.ifr & 0x40) != 0) && via1.t1_running && (via1.t1_counter != 0);

    // Timer 2 timed one-shot: reg 8 latches low, reg 9 starts and clears
    // IFR.T2; T2CL read acknowledges the pending flag.
    via1.ifr &= (uint8_t)~0x20;
    via1.acr &= (uint8_t)~0x20;
    via1_write_reg(8, 0x03);
    via1_write_reg(9, 0x00);
    for (int i = 0; i < 3; i++) via1_timer_tick();
    bool ok_t2 = ((via1.ifr & 0x20) != 0) && !via1.t2_running;
    (void)via1_read_reg(8);
    bool ok_t2cl_ack = (via1.ifr & 0x20) == 0;

    // IFR summary bit should only appear when the pending flag is enabled.
    via1.ifr |= 0x20;
    via1.ier = 0x80;
    const bool ok_ifr_summary_masked = (via1_read_reg(13) & 0x80) == 0;
    via1_write_reg(14, 0xA0);
    const bool ok_ifr_summary_enabled = (via1_read_reg(13) & 0x80) != 0;

    periph_events.close();
    std::fprintf(stderr,
                 "[rom-boot] via1_t1_selftest: orb_mux=%s overlay=%s/%s/%s oneshot=%s t1cl_ack=%s ora_ack=%s freerun=%s t2=%s t2cl_ack=%s ifr_summary=%s/%s\n",
                 ok_orb_mux_reset ? "PASS" : "FAIL",
                 ok_overlay_reset ? "PASS" : "FAIL",
                 ok_overlay_ddr_gated ? "PASS" : "FAIL",
                 ok_overlay_clear ? "PASS" : "FAIL",
                 ok_oneshot ? "PASS" : "FAIL",
                 ok_t1cl_ack ? "PASS" : "FAIL",
                 ok_ora_ack ? "PASS" : "FAIL",
                 ok_freerun ? "PASS" : "FAIL",
                 ok_t2 ? "PASS" : "FAIL",
                 ok_t2cl_ack ? "PASS" : "FAIL",
                 ok_ifr_summary_masked ? "PASS" : "FAIL",
                 ok_ifr_summary_enabled ? "PASS" : "FAIL");
    return ok_orb_mux_reset && ok_overlay_reset && ok_overlay_ddr_gated &&
           ok_overlay_clear &&
           ok_oneshot && ok_t1cl_ack && ok_ora_ack && ok_freerun &&
           ok_t2 && ok_t2cl_ack && ok_ifr_summary_masked && ok_ifr_summary_enabled;
}

static bool run_harness_strictness_selftest() {
    MemModel local_mem;
    Vmac_top local_dut;
    mem = &local_mem;
    dut = &local_dut;
    dut->dbg_pc = Q700_RESET_PC;

    rom_img.assign(16, 0xFF);
    rom_img[0] = 0xDE;
    rom_img[1] = 0xAD;
    rom_img[2] = 0xBE;
    rom_img[3] = 0xEF;

    mem->write8(0x00000000u, 0x11);
    mem->write8(0x00000001u, 0x22);
    mem->write8(0x00000002u, 0x33);
    mem->write8(0x00000003u, 0x44);

    overlay = true;
    strict_overlay = false;
    strict_axi_resp = false;
    const uint32_t legacy_low_read = daxi_read32(0x00000000u);
    const uint32_t legacy_unmapped_rresp = daxi_rresp_for_addr(0x60000000u);
    maybe_clear_overlay_from_high_rom_fetch(Q700_ROM_BASE);
    const bool legacy_auto_cleared = !overlay;

    overlay = true;
    strict_overlay = true;
    strict_axi_resp = true;
    const uint32_t strict_low_read = daxi_read32(0x00000000u);
    const uint32_t strict_unmapped_rresp = daxi_rresp_for_addr(0x60000000u);
    const uint32_t strict_io_gap_rresp = daxi_rresp_for_addr(0x50004000u);
    const uint32_t strict_rom_bresp = daxi_bresp_for_addr(Q700_ROM_BASE);
    const uint32_t strict_overlay_bresp = daxi_bresp_for_addr(0x00000000u);
    maybe_clear_overlay_from_high_rom_fetch(Q700_ROM_BASE);
    const bool strict_overlay_retained = overlay;

    const bool ok =
        legacy_low_read == 0x11223344u &&
        legacy_unmapped_rresp == AXI_RESP_OKAY &&
        legacy_auto_cleared &&
        strict_low_read == 0xDEADBEEFu &&
        strict_unmapped_rresp == AXI_RESP_DECERR &&
        strict_io_gap_rresp == AXI_RESP_DECERR &&
        strict_rom_bresp == AXI_RESP_SLVERR &&
        strict_overlay_bresp == AXI_RESP_SLVERR &&
        strict_overlay_retained;

    std::fprintf(stderr,
        "[rom-boot] harness_strictness_selftest: legacy_low=0x%08x "
        "legacy_unmapped_rresp=%u legacy_auto_clear=%s strict_low=0x%08x "
        "strict_unmapped_rresp=%u strict_io_gap_rresp=%u strict_rom_bresp=%u "
        "strict_overlay_bresp=%u strict_overlay_retained=%s %s\n",
        legacy_low_read,
        (unsigned)legacy_unmapped_rresp,
        legacy_auto_cleared ? "PASS" : "FAIL",
        strict_low_read,
        (unsigned)strict_unmapped_rresp,
        (unsigned)strict_io_gap_rresp,
        (unsigned)strict_rom_bresp,
        (unsigned)strict_overlay_bresp,
        strict_overlay_retained ? "PASS" : "FAIL",
        ok ? "PASS" : "FAIL");
    return ok;
}

static bool run_adb_shadow_selftest() {
    (void)init_periph_event_logger();

    Vmac_top local_dut;
    dut = &local_dut;
    dut->dbg_pc = 0x4000002au;
    dut->dbg_committed = 1;

    via1 = Via1{};
    overlay = true;
    sim_time = 0;
    host_cycle = 0;

    const uint32_t sr_addr = VIA1_BASE + (10u << 9);
    io_write32(sr_addr, 0xA5000000u, 0x8u);
    const bool saw_write = (via1.adb_last_byte == 0xA5u) &&
                           via1.adb_shift_in_progress;

    sim_time += ADB_SHIFT_DELAY_CYCLES;
    host_cycle = sim_time;
    via1_adb_tick();
    const bool saw_complete = (via1.ifr & 0x04u) != 0;

    const uint32_t read_back = io_read32(sr_addr);
    const bool saw_read = (read_back == 0xFFFFFFFFu);
    const bool saw_read_clear = (via1.ifr & 0x04u) == 0;

    periph_events.close();
    dut = nullptr;

    std::fprintf(stderr,
        "[rom-boot] adb_shadow_selftest: write=%s complete=%s read=%s clear=%s\n",
        saw_write ? "PASS" : "FAIL",
        saw_complete ? "PASS" : "FAIL",
        saw_read ? "PASS" : "FAIL",
        saw_read_clear ? "PASS" : "FAIL");
    return saw_write && saw_complete && saw_read && saw_read_clear;
}

static bool run_scc_asc_selftest() {
    (void)init_periph_event_logger();

    Vmac_top local_dut;
    dut = &local_dut;
    dut->dbg_pc = 0x4084af9cu;
    dut->dbg_committed = 1;

    sim_time = 0;
    host_cycle = 0;
    scc = SccStub{};
    asc = AscStub{};

    const uint32_t scc_b_ctrl = SCC_BASE + 0x00u;
    const uint32_t scc_b_data = SCC_BASE + 0x04u;
    const uint32_t scc_a_ctrl = SCC_BASE + 0x08u;

    const uint32_t rr0 = io_read32(scc_a_ctrl);
    io_write32(scc_a_ctrl, 0x02000000u, 0x8u); // point next access at WR2.
    io_write32(scc_a_ctrl, 0x5A000000u, 0x8u); // shared vector.
    io_write32(scc_b_ctrl, 0x02000000u, 0x8u);
    const uint32_t rr2_b = io_read32(scc_b_ctrl);
    io_write32(scc_b_data, 0xA5000000u, 0x8u);
    const uint32_t idle_data = io_read32(scc_b_data);

    const uint32_t asc_version = io_read32(ASC_BASE + 0x800u);
    io_write32(ASC_BASE + 0x806u, 0x40000000u, 0x8u);
    io_write32(ASC_BASE + 0x807u, 0x03000000u, 0x8u);
    io_write32(ASC_BASE + 0x000u, 0x80000000u, 0x8u);
    io_write32(ASC_BASE + 0x400u, 0x40000000u, 0x8u);
    io_write32(ASC_BASE + 0xF09u, 0x01000000u, 0x8u);
    const uint32_t asc_volume = io_read32(ASC_BASE + 0x80Au);
    const uint32_t asc_rate = io_read32(ASC_BASE + 0x808u);
    const uint32_t asc_irq_ctl = io_read32(ASC_BASE + 0xF09u);

    periph_events.close();
    dut = nullptr;

    const bool ok_scc_rr0 = rr0 == 0x6C6C6C6Cu;
    const bool ok_scc_rr2 = rr2_b == 0x5A5A5A5Au;
    const bool ok_scc_idle_data = idle_data == 0x00000000u;
    const bool ok_asc_version = asc_version == 0x00000000u;
    const bool ok_asc_volume = asc_volume == 0x40404040u;
    const bool ok_asc_rate = asc_rate == 0x17171717u;
    const bool ok_asc_irq_ctl = asc_irq_ctl == 0x01010101u;
    const bool ok_asc_fifo_counts =
        asc.fifo_a_writes == 1u && asc.fifo_b_writes == 1u;

    std::fprintf(stderr,
        "[rom-boot] scc_asc_selftest: scc_rr0=%s scc_rr2=%s scc_data=%s asc_version=%s asc_volume=%s asc_rate=%s asc_irq_ctl=%s asc_fifo_counts=%s\n",
        ok_scc_rr0 ? "PASS" : "FAIL",
        ok_scc_rr2 ? "PASS" : "FAIL",
        ok_scc_idle_data ? "PASS" : "FAIL",
        ok_asc_version ? "PASS" : "FAIL",
        ok_asc_volume ? "PASS" : "FAIL",
        ok_asc_rate ? "PASS" : "FAIL",
        ok_asc_irq_ctl ? "PASS" : "FAIL",
        ok_asc_fifo_counts ? "PASS" : "FAIL");
    return ok_scc_rr0 && ok_scc_rr2 && ok_scc_idle_data &&
           ok_asc_version && ok_asc_volume && ok_asc_rate &&
           ok_asc_irq_ctl && ok_asc_fifo_counts;
}

static bool run_scsi_window_selftest() {
    (void)init_periph_event_logger();

    Vmac_top local_dut;
    dut = &local_dut;
    dut->dbg_pc = 0x4000002au;
    dut->dbg_committed = 1;

    sim_time = 0;
    host_cycle = 0;
    turboscsi_trace.reset();

    const uint32_t reg_addr = TURBOSCSI_BASE;
    const uint32_t dma_addr = TURBOSCSI_BASE + 0x100u;
    const uint32_t gap_addr = TURBOSCSI_BASE + 0x102u;
    const uint32_t fifo_addr = TURBOSCSI_BASE + 0x02u;
    const uint32_t cmd_addr = TURBOSCSI_BASE + 0x03u;
    const uint32_t status_addr = TURBOSCSI_BASE + 0x04u;
    const uint32_t intr_addr = TURBOSCSI_BASE + 0x05u;

    io_write32(reg_addr, 0x12000000u, 0x8u);
    const uint32_t reg_read = io_read32(reg_addr);
    io_write32(cmd_addr, 0x41000000u, 0x8u);
    io_write32(cmd_addr, 0x10000000u, 0x8u);
    const uint32_t seq_read = io_read32(TURBOSCSI_BASE + 0x06u);
    const uint32_t status_read = io_read32(status_addr);
    const uint32_t intr_read = io_read32(intr_addr);
    const uint8_t read6_cdb[] = {0x08, 0x00, 0x12, 0x34, 0x02, 0x00};
    for (uint8_t b : read6_cdb)
        io_write32(fifo_addr, ((uint32_t)b) << 24, 0x8u);
    const uint32_t data_status = io_read32(status_addr);
    const uint32_t data_intr = io_read32(intr_addr);
    io_write32(dma_addr, 0x34000000u, 0x8u);
    const uint32_t dma_read = io_read32(dma_addr);
    io_write32(gap_addr, 0x56000000u, 0x8u);
    const uint32_t gap_read = io_read32(gap_addr);

    periph_events.close();
    dut = nullptr;

    const bool ok = (reg_read == 0x12121212u) &&
                    (seq_read == 0x12121212u) &&
                    (status_read == 0x68686868u) &&
                    (intr_read == 0x08080808u) &&
                    (data_status == 0x64646464u) &&
                    (data_intr == 0x40404040u) &&
                    (dma_read == 0x34343434u) &&
                    (gap_read == 0x00000000u);
    std::fprintf(stderr,
        "[rom-boot] scsi_window_selftest: reg=%s seq=%s status=%s intr=%s data_status=%s data_intr=%s dma=%s gap=%s\n",
        reg_read == 0x12121212u ? "PASS" : "FAIL",
        seq_read == 0x12121212u ? "PASS" : "FAIL",
        status_read == 0x68686868u ? "PASS" : "FAIL",
        intr_read == 0x08080808u ? "PASS" : "FAIL",
        data_status == 0x64646464u ? "PASS" : "FAIL",
        data_intr == 0x40404040u ? "PASS" : "FAIL",
        dma_read == 0x34343434u ? "PASS" : "FAIL",
        gap_read == 0x00000000u ? "PASS" : "FAIL");
    return ok;
}

static bool run_display_watch_selftest() {
    (void)init_periph_event_logger();

    Vmac_top local_dut;
    dut = &local_dut;
    dut->dbg_pc = 0x4000002au;
    dut->dbg_committed = 1;

    MemModel local_mem;
    mem = &local_mem;
    sim_time = 0;
    host_cycle = 0;

    // Exercise the ROM-observed first write at +0x24 so the smoke ties
    // directly to the bring-up trace instead of a synthetic tail register.
    const uint32_t reg_addr = DAFB_REG_BASE + 0x24u;
    const uint32_t vram_addr = DAFB_VRAM_BASE + 0x20u;

    daxi_write32(reg_addr, 0x11223344u, 0xFu);

    sim_time++;
    dut->dbg_committed++;
    dut->dbg_pc = 0x4000002eu;
    daxi_write32(vram_addr, 0x55667788u, 0xFu);

    sim_time++;
    dut->dbg_committed++;
    dut->dbg_pc = 0x40000032u;
    const uint32_t vram_read = daxi_read32(vram_addr);

    dump_display_summary();
    periph_events.close();

    const PeriphStats& reg_stats = periph_stats[(size_t)PC_DAFB_REG];
    const PeriphStats& vram_stats = periph_stats[(size_t)PC_VRAM];
    const bool ok_reg = reg_stats.writes == 1 &&
                         reg_stats.first.addr == reg_addr &&
                         reg_stats.first.value == 0x11223344u;
    const bool ok_vram = vram_stats.writes == 1 && vram_stats.reads == 1 &&
                         vram_read == 0x55667788u;

    dut = nullptr;
    mem = nullptr;

    std::fprintf(stderr,
        "[rom-boot] display_watch_selftest: dafb_reg=%s dafb_vram=%s\n",
        ok_reg ? "PASS" : "FAIL",
        ok_vram ? "PASS" : "FAIL");
    return ok_reg && ok_vram;
}

static uint8_t via1_input_a(void) {
    // MAME macqd700 via_in_a(): bits 7:6 identify the Spike/Q700 class;
    // bit 0 is the default "diagnostic mode disabled" config input.
    return 0xC1;
}

static uint8_t via1_input_b(void) {
    // MAME macqd700 via_in_b(): PB3 is high when no ADB IRQ is pending.
    // PB0 is the RTC data line when the ROM releases it (DDRB[0]=0).
    return 0x08 | (rtc_sidechannel.data_line ? 0x01 : 0x00);
}

static uint8_t via1_port_a_read(void) {
    return (via1.ora & via1.ddra) | (via1_input_a() & ~via1.ddra);
}

static uint8_t via1_port_b_read(void) {
    uint8_t raw = (via1.orb & via1.ddrb) | (via1_input_b() & ~via1.ddrb);
    bool overlay_live = (via1.ddrb & 0x08) ? ((via1.orb & 0x08) != 0) : true;
    return (raw & 0xF7u) | (overlay_live ? 0x08u : 0x00u);
}

static bool via1_overlay_active(void) {
    return (via1.ddrb & 0x08) ? ((via1.orb & 0x08) != 0) : true;
}

static bool via1_ca2_independent_irq(void) {
    return (via1.pcr & 0x0a) == 0x02;
}

static bool via1_cb2_independent_irq(void) {
    return (via1.pcr & 0xa0) == 0x20;
}

static void via1_clear_pa_handshake(void) {
    uint8_t mask = 0x02;             // CA1
    if (!via1_ca2_independent_irq()) mask |= 0x01;
    via1.ifr &= (uint8_t)~mask;
}

static void via1_clear_pb_handshake(void) {
    uint8_t mask = 0x10;             // CB1
    if (!via1_cb2_independent_irq()) mask |= 0x08;
    via1.ifr &= (uint8_t)~mask;
}

static void via1_update_overlay(void) {
    // Q700/MAME treats the reset overlay as one-way: once the ROM path has
    // escaped to the high 0x4000_0000 mirror, low memory stays RAM even if
    // VIA1's port-B latch would otherwise read back overlay asserted.
    bool new_overlay = overlay && via1_overlay_active();
    bool old_overlay = overlay;
    overlay = new_overlay;
    if (old_overlay && !new_overlay) {
        std::fprintf(stderr,
            "[rom-boot] overlay cleared at cycle %llu (VIA1 ORB=0x%02x DDRB=0x%02x)\n",
            (unsigned long long)sim_time, via1.orb, via1.ddrb);
        if (periph_events.enabled())
            periph_event("VIA1", "overlay_clear", VIA1_BASE, via1.orb,
                         fmt_kv_u32("ddrb", via1.ddrb));
        if (trace_fp) std::fprintf(trace_fp,
            "# overlay cleared at cycle %llu (ORB=0x%02x DDRB=0x%02x)\n",
            (unsigned long long)sim_time, via1.orb, via1.ddrb);
    } else if (!old_overlay && new_overlay) {
        if (verbose) {
            std::fprintf(stderr,
                "[rom-boot] overlay reasserted at cycle %llu (VIA1 ORB=0x%02x DDRB=0x%02x)\n",
                (unsigned long long)sim_time, via1.orb, via1.ddrb);
        }
        if (periph_events.enabled())
            periph_event("VIA1", "overlay_set", VIA1_BASE, via1.orb,
                         fmt_kv_u32("ddrb", via1.ddrb));
    }
}

static inline bool is_high_rom_fetch(uint32_t a) {
    return a >= Q700_ROM_BASE && a < 0x50000000u;
}

static void maybe_clear_overlay_from_high_rom_fetch(uint32_t addr) {
    if (strict_overlay) return;
    if (!overlay || !is_high_rom_fetch(addr)) return;

    overlay = false;
    std::fprintf(stderr,
        "[rom-boot] overlay auto-cleared at cycle %llu (high ROM fetch addr=0x%08x)\n",
        (unsigned long long)sim_time, addr);
    if (periph_events.enabled())
        periph_event("ROM", "overlay_auto_clear", addr, 0, "high_rom_fetch");
    if (trace_fp) std::fprintf(trace_fp,
        "# overlay auto-cleared at cycle %llu (high ROM fetch addr=0x%08x)\n",
        (unsigned long long)sim_time, addr);
}

static uint8_t via1_read_reg(uint8_t idx) {
    switch (idx) {
        case 0: {
            uint8_t v = via1_port_b_read();
            via1_clear_pb_handshake();
            return v;
        }
        case 1: {
            uint8_t v = via1_port_a_read();
            via1_clear_pa_handshake();
            return v;
        }
        case 2:  return via1.ddrb;
        case 3:  return via1.ddra;
        case 4: {
            uint8_t v = via1.t1cl;
            via1.ifr &= (uint8_t)~0x40;
            return v;
        }
        case 5:  return via1.t1ch;
        case 6:  return via1.t1ll;
        case 7:  return via1.t1lh;
        case 8: {
            uint8_t v = via1.t2cl;
            via1.ifr &= (uint8_t)~0x20;
            return v;
        }
        case 9:  return via1.t2ch;
        // SR (reg 10): empty-bus — always read as 0xFF.  On a real Q700
        // this is the ADB shift-register; with no devices acking, the bus
        // idles high and the VIA clocks 0xFF in.  MAME's 6522 model also
        // clears INT_SR on SR read; mirror that so ADB poll loops can observe
        // a one-shot shift-complete event instead of a sticky stale flag.
        case 10:
            via1.ifr &= ~0x04;
            return 0xFF;
        case 11: return via1.acr;
        case 12: return via1.pcr;
        // IFR (reg 13): bit 7 is the "any IRQ pending" summary flag.
        // Synthesise it live from bits [6:0] & IER[6:0] so reads are
        // consistent after the ADB shadow sets bit 2.
        case 13: {
            uint8_t pending = via1.ifr & via1.ier & 0x7F;
            return (via1.ifr & 0x7F) | (pending ? 0x80 : 0x00);
        }
        case 14: return via1.ier;
        case 15: return via1_port_a_read();   // ORA-NH
    }
    return 0x00;
}
static void via1_write_reg(uint8_t idx, uint8_t v) {
    switch (idx) {
        case 0: {
            // ORB — the critical overlay-clear bit lives at ORB[3].
            // Real 6522 writes update the output latch regardless of DDRB;
            // the pin value is selected by DDRB on read/output.
            via1.orb = v;
            via1_clear_pb_handshake();
            via1_update_overlay();
            break;
        }
        case 1:
            via1.ora = v;
            via1_clear_pa_handshake();
            break;
        case 2:  via1.ddrb = v; via1_update_overlay(); break;
        case 3:  via1.ddra = v; break;
        case 4:
            via1.t1cl = v;
            via1.t1_latch = (uint16_t)((via1.t1_latch & 0xFF00u) | v);
            break;
        case 5: {
            via1.t1ch = v;
            via1.t1_latch = (uint16_t)((((uint16_t)v) << 8) | (via1.t1_latch & 0x00FFu));
            // Writing T1CH starts countdown and clears IFR bit 6.
            via1.ifr &= ~0x40;
            via1.t1_counter = via1.t1_latch ? via1.t1_latch : 1;
            via1.t1_running = true;
            break;
        }
        case 6:
            via1.t1ll = v;
            via1.t1_latch = (uint16_t)((via1.t1_latch & 0xFF00u) | v);
            break;
        case 7:
            via1.t1lh = v;
            via1.t1_latch = (uint16_t)((((uint16_t)v) << 8) | (via1.t1_latch & 0x00FFu));
            via1.ifr &= (uint8_t)~0x40;
            break;
        case 8:
            via1.t2cl = v;
            via1.t2_latch = (uint16_t)((via1.t2_latch & 0xFF00u) | v);
            break;
        case 9:
            via1.t2ch = v;
            via1.t2_latch = (uint16_t)((((uint16_t)v) << 8) | (via1.t2_latch & 0x00FFu));
            via1.ifr &= ~0x20;
            via1.t2_counter = via1.t2_latch ? via1.t2_latch : 1;
            via1.t2_running = true;
            break;
        case 10: {
            // SR write — ADB command byte.  Latch the byte, clear the
            // old SR IFR flag (shifting will regenerate it), and
            // schedule a new shift-complete flag a few hundred cycles
            // later.  Empty-bus behaviour: the byte we wrote has no
            // effect on anyone, and reads after the "complete" delay
            // return 0xFF (handled in via1_read_reg).
            via1.sr = v;
            via1.adb_last_byte = v;
            via1.adb_transaction_id++;
            via1.adb_shift_in_progress  = true;
            via1.adb_shift_complete_cyc = host_cycle + ADB_SHIFT_DELAY_CYCLES;
            via1.ifr &= ~0x04;              // clear stale SR-complete flag
            break;
        }
        case 11: via1.acr  = v; break;
        case 12: via1.pcr  = v; break;
        case 13: {
            // IFR is write-1-to-clear (except bit 7).
            via1.ifr &= ~(v & 0x7F);
            break;
        }
        case 14: {
            // IER: bit 7 = 1 ⇒ set bits in {v[6:0]}, bit 7 = 0 ⇒ clear.
            if (v & 0x80) via1.ier |= (v & 0x7F);
            else          via1.ier &= ~(v & 0x7F);
            via1.ier |= 0x80;  // bit 7 reads back as 1 per 6522 convention
            break;
        }
        case 15: via1.ora = v; break;  // ORA-NH
    }
}

static void rtc_selftest_write_via1(uint8_t idx, uint8_t v) {
    via1_write_reg(idx, v);
    if (idx == 0 || idx == 2)
        via1_update_rtc_pins(VIA1_BASE + ((uint32_t)idx << 9));
}

static void rtc_selftest_set_pins(bool enb, bool clk, bool data, bool data_oe) {
    rtc_selftest_write_via1(2, (uint8_t)(0x06 | (data_oe ? 0x01 : 0x00)));
    rtc_selftest_write_via1(0, (uint8_t)((enb ? 0x04 : 0x00) |
                                         (clk ? 0x02 : 0x00) |
                                         (data ? 0x01 : 0x00)));
}

static void rtc_selftest_shift_in_bit(bool bit) {
    rtc_selftest_set_pins(false, true, bit, true);
    rtc_selftest_set_pins(false, false, bit, true);
}

static void rtc_selftest_shift_in_pullup_one(bool bit) {
    rtc_selftest_set_pins(false, true, false, !bit);
    rtc_selftest_set_pins(false, false, false, !bit);
}

static bool rtc_selftest_shift_out_bit(void) {
    rtc_selftest_set_pins(false, true, false, false);
    rtc_selftest_set_pins(false, false, false, false);
    return (via1_read_reg(0) & 0x01) != 0;
}

static uint8_t rtc_selftest_transaction(uint8_t cmd, uint8_t data_byte) {
    uint8_t out = 0;

    rtc_selftest_set_pins(false, false, false, true);
    for (int i = 7; i >= 0; i--)
        rtc_selftest_shift_in_bit(((cmd >> i) & 1) != 0);

    if (cmd & 0x80) {
        for (int i = 0; i < 8; i++)
            out = (uint8_t)((out << 1) | (rtc_selftest_shift_out_bit() ? 1 : 0));
    } else {
        for (int i = 7; i >= 0; i--)
            rtc_selftest_shift_in_bit(((data_byte >> i) & 1) != 0);
    }

    rtc_selftest_set_pins(true, false, false, false);
    return out;
}

static uint8_t rtc_selftest_xpram_transaction(uint8_t cmd, uint8_t addr_byte, uint8_t data_byte) {
    uint8_t out = 0;

    rtc_selftest_set_pins(false, false, false, true);
    for (int i = 7; i >= 0; i--)
        rtc_selftest_shift_in_bit(((cmd >> i) & 1) != 0);
    for (int i = 7; i >= 0; i--)
        rtc_selftest_shift_in_bit(((addr_byte >> i) & 1) != 0);

    if (cmd & 0x80) {
        for (int i = 0; i < 8; i++)
            out = (uint8_t)((out << 1) | (rtc_selftest_shift_out_bit() ? 1 : 0));
    } else {
        for (int i = 7; i >= 0; i--)
            rtc_selftest_shift_in_bit(((data_byte >> i) & 1) != 0);
    }

    rtc_selftest_set_pins(true, false, false, false);
    return out;
}

static uint8_t rtc_selftest_transaction_pullup_ones(uint8_t cmd, uint8_t data_byte) {
    uint8_t out = 0;

    rtc_selftest_set_pins(false, false, false, true);
    for (int i = 7; i >= 0; i--)
        rtc_selftest_shift_in_pullup_one(((cmd >> i) & 1) != 0);

    if (cmd & 0x80) {
        for (int i = 0; i < 8; i++)
            out = (uint8_t)((out << 1) | (rtc_selftest_shift_out_bit() ? 1 : 0));
    } else {
        for (int i = 7; i >= 0; i--)
            rtc_selftest_shift_in_pullup_one(((data_byte >> i) & 1) != 0);
    }

    rtc_selftest_set_pins(true, false, false, false);
    return out;
}

static uint8_t rtc_selftest_xpram_transaction_pullup_ones(uint8_t cmd,
                                                          uint8_t addr_byte,
                                                          uint8_t data_byte) {
    uint8_t out = 0;

    rtc_selftest_set_pins(false, false, false, true);
    for (int i = 7; i >= 0; i--)
        rtc_selftest_shift_in_pullup_one(((cmd >> i) & 1) != 0);
    for (int i = 7; i >= 0; i--)
        rtc_selftest_shift_in_pullup_one(((addr_byte >> i) & 1) != 0);

    if (cmd & 0x80) {
        for (int i = 0; i < 8; i++)
            out = (uint8_t)((out << 1) | (rtc_selftest_shift_out_bit() ? 1 : 0));
    } else {
        for (int i = 7; i >= 0; i--)
            rtc_selftest_shift_in_pullup_one(((data_byte >> i) & 1) != 0);
    }

    rtc_selftest_set_pins(true, false, false, false);
    return out;
}

static uint8_t rtc_selftest_xpram_cmd(uint8_t addr, bool read) {
    return (uint8_t)((read ? 0x80 : 0x00) | 0x38 | ((addr >> 5) & 0x07));
}

static uint8_t rtc_selftest_xpram_addr_byte(uint8_t addr) {
    return (uint8_t)(((addr & 0x1f) << 2) | 0x01);
}

static uint8_t rtc_selftest_xpram_read(uint8_t addr) {
    return rtc_selftest_xpram_transaction(rtc_selftest_xpram_cmd(addr, true),
                                          rtc_selftest_xpram_addr_byte(addr), 0);
}

static void rtc_selftest_xpram_write(uint8_t addr, uint8_t value) {
    (void)rtc_selftest_xpram_transaction(rtc_selftest_xpram_cmd(addr, false),
                                         rtc_selftest_xpram_addr_byte(addr), value);
}

static uint32_t rtc_selftest_read_seconds(void) {
    const uint8_t b0 = rtc_selftest_transaction(0x81, 0);
    const uint8_t b1 = rtc_selftest_transaction(0x85, 0);
    const uint8_t b2 = rtc_selftest_transaction(0x89, 0);
    const uint8_t b3 = rtc_selftest_transaction(0x8D, 0);
    return ((uint32_t)b3 << 24) | ((uint32_t)b2 << 16) |
           ((uint32_t)b1 << 8) | (uint32_t)b0;
}

static bool run_rtc_sidechannel_selftest() {
    (void)init_periph_event_logger();

    via1 = Via1{};
    overlay = true;
    sim_time = 0;
    host_cycle = 0;
    rtc_sidechannel.reset(host_cycle);

    rtc_selftest_set_pins(true, false, false, false);
    if ((via1_read_reg(0) & 0x01) != 0x01) {
        std::fprintf(stderr, "[rom-boot] rtc_sidechannel_selftest: idle PB0 not high\n");
        periph_events.close();
        return false;
    }

    rtc_selftest_transaction(0x20, 0x5A);     // PRAM reg 8
    const uint8_t pram8 = rtc_selftest_transaction(0xA0, 0);
    const uint8_t pram9_default = rtc_selftest_transaction(0xA4, 0);

    rtc_selftest_xpram_write(0x42, 0xA6);
    const uint8_t xpram42 = rtc_selftest_xpram_read(0x42);

    rtc_selftest_transaction_pullup_ones(0x40, 0x5C);  // PRAM reg 16
    const uint8_t pram16_pullup = rtc_selftest_transaction_pullup_ones(0xC0, 0);
    (void)rtc_selftest_xpram_transaction_pullup_ones(
        rtc_selftest_xpram_cmd(0x44, false),
        rtc_selftest_xpram_addr_byte(0x44),
        0x96);
    const uint8_t xpram44_pullup =
        rtc_selftest_xpram_transaction_pullup_ones(
            rtc_selftest_xpram_cmd(0x44, true),
            rtc_selftest_xpram_addr_byte(0x44),
            0);

    rtc_selftest_transaction(0x34, 0x80);     // write-protect set
    rtc_selftest_xpram_write(0x42, 0x19);
    const uint8_t xpram42_protected = rtc_selftest_xpram_read(0x42);
    rtc_selftest_transaction(0x37, 0x00);     // write-protect clear

    rtc_selftest_transaction(0x01, 0x04);
    rtc_selftest_transaction(0x05, 0x03);
    rtc_selftest_transaction(0x09, 0x02);
    rtc_selftest_transaction(0x0D, 0x01);
    const uint32_t sec0 = rtc_selftest_read_seconds();

    host_cycle += RtcSidechannelModel::CYCLES_PER_SECOND * 2;
    const uint32_t sec2 = rtc_selftest_read_seconds();

    const uint32_t sec_before_test = rtc_selftest_read_seconds();
    rtc_selftest_transaction(0x31, 0x80);     // test register set
    const uint8_t test_mode = rtc_selftest_transaction(0xB1, 0);
    host_cycle += RtcSidechannelModel::CYCLES_PER_SECOND;
    const uint32_t sec_after_test = rtc_selftest_read_seconds();
    rtc_selftest_transaction(0x31, 0x00);     // test register clear

    const bool ok = (pram8 == 0x5A) &&
                    (pram9_default == 0x88) &&
                    (xpram42 == 0xA6) &&
                    (pram16_pullup == 0x5C) &&
                    (xpram44_pullup == 0x96) &&
                    (xpram42_protected == 0xA6) &&
                    (sec0 == 0x01020304u) &&
                    (sec2 == 0x01020306u) &&
                    (test_mode == 0x80u) &&
                    (sec_before_test + 1u == sec_after_test);
    std::fprintf(stderr,
        "[rom-boot] rtc_sidechannel_selftest: pram8=0x%02x pram9=0x%02x xpram42=0x%02x pram16_pullup=0x%02x xpram44_pullup=0x%02x protected=0x%02x seconds0=0x%08x seconds2=0x%08x test=0x%02x seconds_test=0x%08x %s\n",
        pram8, pram9_default, xpram42, pram16_pullup, xpram44_pullup,
        xpram42_protected, sec0, sec2, test_mode, sec_after_test,
        ok ? "PASS" : "FAIL");
    periph_events.close();
    return ok;
}

static uint32_t io_read32(uint32_t a) {
    // Full-word reads on Q700 replicate the byte onto all four lanes
    // (big-endian).  Mac ROM selects a single byte via BMOVE.B or by
    // reading D-lane 0 (addr & 3 == 3 in big-endian numbering).
    uint32_t ca = q700_io_canonical(a);
    uint8_t v = 0;
    if (in_range(ca, VIA1_BASE, VIA1_SIZE)) {
        uint8_t idx = (ca >> 9) & 0xF;
        uint8_t ifr_before = via1.ifr;
        const bool via1_sr_pending_before_read =
            (idx == 10) && ((via1.ifr & 0x04u) != 0);
        v = via1_read_reg(idx);
        periph_record_access(PC_VIA1, false, ca, (uint32_t)v * 0x01010101u,
                             (uint32_t)dut->dbg_pc, via1_reg_name(idx));
        if (periph_events.enabled()) {
            periph_event("VIA1", "read", ca, v,
                         detail_reg_value(via1_reg_name(idx), v));
            if (idx == 10) {
                periph_event_c("ADB", "sr_read", ca, v, "reg=SR");
                if (via1_sr_pending_before_read)
                    periph_event_c("ADB", "sr_irq_clear", ca, v,
                                   "reg=SR,ifrbits=SR");
            } else if (idx == 13) {
                periph_event("VIA1", "ifr_read", ca, v, detail_reg_value("IFR", v));
                if (v & 0x02)
                    periph_event_c("VBL", "ifr_pending_read", ca, v, "ifrbits=CA1");
                if (v & 0x40)
                    periph_event_c("VBL", "ifr_pending_read", ca, v, "ifrbits=T1");
            } else if (idx == 14) {
                periph_event("VIA1", "ier_read", ca, v, detail_reg_value("IER", v));
            }
            uint8_t cleared = (uint8_t)((ifr_before & ~via1.ifr) & 0x7f);
            if (cleared) {
                std::string detail = via1_ifr_clear_detail(idx, ifr_before, via1.ifr);
                periph_event("VIA1", "read_clear", ca, cleared, detail);
                if (cleared & 0x04)
                    periph_event("ADB", "sr_read_clear", ca, cleared, detail);
                if (cleared & 0x42)
                    periph_event("VBL", "ifr_read_clear", ca, cleared, detail);
            }
        }
        if (verbose) std::fprintf(stderr,
            "[io] VIA1 rd reg[%d]=0x%02x addr=0x%08x canon=0x%08x (pc=0x%08x)\n",
            idx, v, a, ca, (unsigned)dut->dbg_pc);
    } else if (in_range(ca, VIA2_BASE, VIA2_SIZE)) {
        uint8_t idx = (ca >> 9) & 0xF;
        v = via2.regs[idx];
        if (idx == 0)  v = 0xC7;   // Q700 VIA2 ORB: NO slot IRQs (bits 3:0 set)
        if (idx == 13) v = 0x00;   // IFR = 0 (no IRQ pending)
        periph_record_access(PC_VIA2, false, ca, (uint32_t)v * 0x01010101u,
                             (uint32_t)dut->dbg_pc, "slot-irq stub");
        if (periph_events.enabled())
            periph_event("VIA2", "read", ca, v,
                         detail_reg_value(via_reg_name(idx), v));
    } else if (in_range(ca, ENET_BASE, ENET_SIZE)) {
        v = 0x00;  // Ethernet ID/config stub: absent/idle
    } else if (in_range(ca, SONIC_BASE, SONIC_SIZE)) {
        v = 0x00;  // SONIC idle
    } else if (in_range(ca, SCC_BASE, SCC_SIZE)) {
        std::string note = scc_detail(ca);
        const bool data_port = SccStub::is_data_port(ca - SCC_BASE);
        v = scc.read(ca - SCC_BASE);
        periph_record_access(PC_SCC, false, ca, (uint32_t)v * 0x01010101u,
                             (uint32_t)dut->dbg_pc, note.c_str());
        if (periph_events.enabled())
            periph_event("SCC", data_port ? "data_read" : "reg_read",
                         ca, v, note);
    } else if (in_range(ca, ORWELL_BASE, ORWELL_SIZE)) {
        v = 0x00;  // Orwell controls: conservative reset value
    } else if (in_range(ca, TURBOSCSI_BASE, TURBOSCSI_SIZE)) {
        // DAFB TurboSCSI/NCR53C96 host window per MAME macquadra700.cpp:
        // +0x000..+0x0ff are controller regs, +0x100..+0x101 is the DMA
        // handshake shim.  The harness keeps a small phase-aware register
        // file so ROM polling sees a deterministic select/command/data/
        // status progression rather than a flat zero response.
        uint32_t off = ca - TURBOSCSI_BASE;
        if (off < 0x100u) {
            switch (off & 0x0Fu) {
                case 0x04:
                    v = turboscsi_status_value();
                    break;
                case 0x05:
                    v = turboscsi_bus_and_stat_value();
                    break;
                case 0x06:
                    v = turboscsi_trace.current_data;
                    break;
                default:
                    v = turboscsi_trace.regs[off & 0xFFu];
                    break;
            }
        } else if (off < 0x102u) {
            v = turboscsi_trace.drq_pending ? turboscsi_trace.current_data : 0x00;
        }
        std::string note = turboscsi_detail(ca);
        periph_record_access(PC_SCSI, false, ca, (uint32_t)v * 0x01010101u,
                             (uint32_t)dut->dbg_pc, note.c_str());
        if (periph_events.enabled())
            periph_event("SCSI", turboscsi_event_name(false, off), ca, v,
                         note);
        turboscsi_observe_read(ca, v);
    } else if (in_range(ca, ASC_BASE, ASC_SIZE)) {
        v = asc.read(ca - ASC_BASE);
        std::string note;
        const char* access_note = "sonora sound chip";
        if (periph_events.enabled() || periph_detail_sink_active()) {
            note = asc_detail(ca);
            access_note = note.c_str();
        }
        periph_record_access(PC_ASC, false, ca, (uint32_t)v * 0x01010101u,
                             (uint32_t)dut->dbg_pc, access_note);
        if (periph_events.enabled())
            periph_event("ASC", "read", ca, v, note);
    } else if (in_range(ca, SWIM_BASE, SWIM_SIZE)) {
        uint8_t reg = (uint8_t)(((ca - SWIM_BASE) >> SWIM_REG_SHIFT) & SWIM_REG_MASK);
        v = swim.read(reg);
    } else {
        unmapped_read_count[a & ~0xFFu]++;
        periph_record_access(PC_UNMAPPED, false, ca, 0xFFFFFFFFu,
                             (uint32_t)dut->dbg_pc, "open-bus probe");
        if (verbose) std::fprintf(stderr,
            "[io] UNMAPPED rd 0x%08x (pc=0x%08x)\n",
            a, (unsigned)dut->dbg_pc);
    }
    uint32_t r32 = (uint32_t)v * 0x01010101u;   // broadcast byte
    return r32;
}

static void io_write32(uint32_t a, uint32_t data, uint32_t strb) {
    // Pull the first asserted byte out (big-endian: strb bit 3 = byte at
    // a+0).  Matches the VIA1 RTL which latches a single byte per write.
    uint8_t v = 0;
    for (int i = 0; i < 4; i++) {
        if (strb & (1u << (3 - i))) {
            v = (data >> ((3 - i) * 8)) & 0xFF;
            uint32_t lane_addr = a + (uint32_t)i;
            uint32_t ca = q700_io_canonical(lane_addr);
            if (in_range(ca, VIA1_BASE, VIA1_SIZE)) {
                uint8_t idx = (ca >> 9) & 0xF;
                uint8_t ifr_before = via1.ifr;
                via1_write_reg(idx, v);
                periph_record_access(PC_VIA1, true, ca, (uint32_t)v * 0x01010101u,
                                     (uint32_t)dut->dbg_pc, via1_reg_name(idx));
                if (periph_events.enabled()) {
                    periph_event("VIA1", "write", ca, v,
                                 detail_reg_value(via1_reg_name(idx), v));
                    if (idx == 5) {
                        char detail[160];
                        std::snprintf(detail, sizeof(detail),
                                      "t1_latch=0x%04x,t1_counter=0x%04x,acr=0x%02x,div=%llu",
                                      (unsigned)via1.t1_latch,
                                      (unsigned)via1.t1_counter,
                                      via1.acr,
                                      (unsigned long long)via1_timer_div);
                        periph_event_c("VIA1", "timer1_start", ca, v, detail);
                        periph_event_c("VBL", "t1_start", ca, v, detail);
                    } else if (idx == 9) {
                        char detail[160];
                        std::snprintf(detail, sizeof(detail),
                                      "t2_latch=0x%04x,t2_counter=0x%04x,acr=0x%02x,mode=%s,div=%llu",
                                      (unsigned)via1.t2_latch,
                                      (unsigned)via1.t2_counter,
                                      via1.acr,
                                      (via1.acr & 0x20) ? "pulse-count" : "oneshot",
                                      (unsigned long long)via1_timer_div);
                        periph_event("VIA1", "timer2_start", ca, v, detail);
                    } else if (idx == 10) {
                        char detail[192];
                        const uint8_t sr_mode = (via1.acr >> 2) & 0x07;
                        std::snprintf(detail, sizeof(detail),
                                      "txn=%u,byte=0x%02x,complete_cycle=%llu,acr=0x%02x,sr_mode=%u,pcr=0x%02x",
                                      via1.adb_transaction_id,
                                      v,
                                      (unsigned long long)via1.adb_shift_complete_cyc,
                                      via1.acr,
                                      (unsigned)sr_mode,
                                      via1.pcr);
                        periph_event_c("ADB", "sr_write", ca, v, detail);
                    } else if (idx == 11) {
                        periph_event("ADB", "acr_write", ca, v,
                                     detail_reg_value("ACR", v));
                    } else if (idx == 12) {
                        periph_event("ADB", "pcr_write", ca, v,
                                     detail_reg_value("PCR", v));
                    } else if (idx == 13) {
                        periph_event("VIA1", "ifr_clear", ca, v,
                                     detail_reg_value("IFR_CLEAR", v));
                        if (v & 0x02)
                            periph_event_c("VBL", "ifr_clear", ca, v, "ifrbits=CA1");
                        if (v & 0x40)
                            periph_event_c("VBL", "ifr_clear", ca, v, "ifrbits=T1");
                    } else if (idx == 14) {
                        const bool set = (v & 0x80) != 0;
                        periph_event("VIA1", set ? "ier_set" : "ier_clear", ca, v,
                                     detail_reg_value("IER", v));
                        if (v & 0x02)
                            periph_event_c("VBL", set ? "ier_set" : "ier_clear", ca, v,
                                           "ifrbits=CA1");
                        if (v & 0x40)
                            periph_event_c("VBL", set ? "ier_set" : "ier_clear", ca, v,
                                           "ifrbits=T1");
                    }
                    uint8_t cleared = (uint8_t)((ifr_before & ~via1.ifr) & 0x7f);
                    if (cleared) {
                        std::string detail = via1_ifr_clear_detail(idx, ifr_before, via1.ifr);
                        periph_event("VIA1", "write_clear", ca, cleared, detail);
                        if (cleared & 0x04)
                            periph_event("ADB", "sr_write_clear", ca, cleared, detail);
                        if (cleared & 0x42)
                            periph_event("VBL", "ifr_write_clear", ca, cleared, detail);
                    }
                }
                if (idx == 0 || idx == 2)
                    via1_update_rtc_pins(ca);
                if (verbose) std::fprintf(stderr,
                    "[io] VIA1 wr reg[%d]=0x%02x addr=0x%08x canon=0x%08x (pc=0x%08x)\n",
                    idx, v, lane_addr, ca, (unsigned)dut->dbg_pc);
            } else if (in_range(ca, VIA2_BASE, VIA2_SIZE)) {
                uint8_t idx = (ca >> 9) & 0xF;
                via2.regs[idx] = v;
                periph_record_access(PC_VIA2, true, ca, (uint32_t)v * 0x01010101u,
                                     (uint32_t)dut->dbg_pc, "slot-irq stub");
                if (periph_events.enabled())
                    periph_event("VIA2", "write", ca, v,
                                 detail_reg_value(via_reg_name(idx), v));
            } else if (in_range(ca, ENET_BASE, ENET_SIZE)) {
                // swallow
            } else if (in_range(ca, SONIC_BASE, SONIC_SIZE)) {
                // swallow
            } else if (in_range(ca, SCC_BASE, SCC_SIZE)) {
                std::string note = scc_detail(ca);
                const bool data_port = SccStub::is_data_port(ca - SCC_BASE);
                scc.write(ca - SCC_BASE, v);
                periph_record_access(PC_SCC, true, ca, (uint32_t)v * 0x01010101u,
                                     (uint32_t)dut->dbg_pc, note.c_str());
                if (periph_events.enabled())
                    periph_event("SCC", data_port ? "data_write" : "reg_write",
                                 ca, v, note);
            } else if (in_range(ca, ORWELL_BASE, ORWELL_SIZE)) {
                // swallow
            } else if (in_range(ca, TURBOSCSI_BASE, TURBOSCSI_SIZE)) {
                // DAFB TurboSCSI/NCR53C96 host window.  Accept writes so ROM
                // probes can proceed, but log whether they hit regs or DMA.
                uint32_t off = ca - TURBOSCSI_BASE;
                std::string note = turboscsi_detail(ca);
                periph_record_access(PC_SCSI, true, ca, (uint32_t)v * 0x01010101u,
                                     (uint32_t)dut->dbg_pc, note.c_str());
                turboscsi_observe_write(ca, v);
                if (periph_events.enabled()) {
                    periph_event("SCSI", turboscsi_event_name(true, off), ca, v,
                                 note);
                }
                if (off < 0x100u) {
                    if (off != 0x02u && off != 0x03u) {
                        turboscsi_trace.regs[off & 0xFFu] = v;
                    }
                    if (off == 0x00u)
                        turboscsi_trace.current_data = v;
                    if (off == 0x07u && v == 0x00u) {
                        turboscsi_trace.irq_pending = false;
                        turboscsi_trace.end_dma_pending = false;
                        turboscsi_trace.busy_error_pending = false;
                    }
                }
            } else if (in_range(ca, ASC_BASE, ASC_SIZE)) {
                asc.write(ca - ASC_BASE, v);
                std::string note;
                const char* access_note = "sonora sound chip";
                if (periph_events.enabled() || periph_detail_sink_active()) {
                    note = asc_detail(ca);
                    access_note = note.c_str();
                }
                periph_record_access(PC_ASC, true, ca, (uint32_t)v * 0x01010101u,
                                     (uint32_t)dut->dbg_pc, access_note);
                if (periph_events.enabled())
                    periph_event("ASC", "write", ca, v, note);
            } else if (in_range(ca, SWIM_BASE, SWIM_SIZE)) {
                uint8_t reg = (uint8_t)(((ca - SWIM_BASE) >> SWIM_REG_SHIFT) & SWIM_REG_MASK);
                swim.write(reg, v);
            } else {
                unmapped_write_count[a & ~0xFFu]++;
                periph_record_access(PC_UNMAPPED, true, ca, (uint32_t)v * 0x01010101u,
                                     (uint32_t)dut->dbg_pc, "open-bus probe");
                if (verbose) std::fprintf(stderr,
                    "[io] UNMAPPED wr 0x%08x = 0x%02x (pc=0x%08x)\n",
                    a + i, v, (unsigned)dut->dbg_pc);
            }
        }
    }
    (void)v;
}

// ─── Fetch + AXI plumbing ─────────────────────────────────────────────
static int       if_pending = 0;
static uint32_t  if_addr_q  = 0;

// Copy of tb_top's Axi struct — single outstanding R/W.
struct Axi {
    bool     ar_outstanding = false;
    uint32_t ar_addr = 0;
    uint8_t  ar_len = 0;
    uint8_t  ar_size = 2;
    uint8_t  ar_burst = 1;
    uint8_t  ar_beat = 0;
    int      ar_delay = 0;
    bool     aw_done = false;
    bool     w_done  = false;
    uint32_t aw_addr = 0;
    uint32_t w_data  = 0;
    uint32_t w_strb  = 0;
    uint32_t b_resp  = 0;
    int      b_delay = 0;
    bool     b_pending = false;
} ax;

static bool test_done = false;
static const char* stop_reason = "??";
static char stop_pc_reason[128];
static char end_pc_reason[128];
static char stop_exc_reason[256];
static char end_exc_reason[256];
static char stop_ifetch_berr_reason[192];
static char end_ifetch_berr_reason[192];
static char stuck_pc_reason[160];
static char end_stuck_pc_reason[160];
static char no_progress_reason[160];
static char rom_outcome_reason[192];
static uint32_t last_ifetch_berr_addr = 0;
static uint32_t last_logged_pc = 0;
static uint32_t last_same_pc_count = 0;
static constexpr uint64_t DEFAULT_STUCK_PC_THRESHOLD = 8192;
static constexpr uint64_t DEFAULT_NO_PROGRESS_CYCLES = 500000;
static constexpr uint32_t SAD_MAC_PC = 0x40849afau;
static constexpr uint32_t DISK_PROMPT_PC = 0x40898e3eu;
static uint64_t stuck_pc_threshold = DEFAULT_STUCK_PC_THRESHOLD;
static uint64_t no_progress_cycle_threshold = DEFAULT_NO_PROGRESS_CYCLES;
static constexpr uint64_t FAULT_DUMP_BYTES_MAX = 4096;
static constexpr uint32_t DBG_BOUNDARY_NONE = 0;
static constexpr uint32_t DBG_BOUNDARY_RETIRE = 1;
static constexpr uint32_t DBG_BOUNDARY_EXC = 2;
static constexpr uint32_t DBG_BOUNDARY_RTE = 3;

static void maybe_apply_ramtest_mame_state_fastfill(uint32_t retired_pc) {
    if (!ramtest_mame_state_fastfill || !mem) return;
    if (retired_pc != 0x40847280u) return;

    const uint32_t start = read_arch_reg(8);  // A0, helper start pointer.
    const uint32_t end = read_arch_reg(9);    // A1, helper top pointer.
    if (ramtest_mame_state_fastfill_hits != 0 && start == end) {
        ramtest_mame_state_fastfill_skips++;
        std::fprintf(stderr,
            "[rom-boot] ramtest-mame-state skipped empty range "
            "0x%08x..0x%08x (skip=%llu)\n",
            start, end,
            (unsigned long long)ramtest_mame_state_fastfill_skips);
        return;
    }
    if (start >= end || end > visible_ram_size ||
        end > MemModel::RAM_BACKING_SIZE) {
        std::fprintf(stderr,
            "[rom-boot] ramtest-mame-state bad fill range: "
            "start=0x%08x end=0x%08x visible_ram=0x%08x\n",
            start, end, (unsigned)visible_ram_size);
        stop_reason = "ramtest-mame-state-bad-range";
        test_done = true;
        return;
    }

    static const uint8_t pattern[3] = {0x6du, 0xb6u, 0xdbu};
    uint8_t* ram = mem->data() + MemModel::RAM_OFF + start;
    const uint32_t len = end - start;
    for (uint32_t i = 0; i < len; i++)
        ram[i] = pattern[i % 3u];

    const uint32_t list_sentinel = read_arch_reg(12); // A4 at MAME helper entry.
    if (visible_ram_size >= 4u && MemModel::RAM_BACKING_SIZE >= 4u &&
        list_sentinel <= visible_ram_size - 4u &&
        list_sentinel <= MemModel::RAM_BACKING_SIZE - 4u) {
        mem->write32(list_sentinel, 0xFFFFFFFFu);
        ramtest_mame_state_fastfill_sentinels++;
        std::fprintf(stderr,
            "[rom-boot] ramtest-mame-state wrote MAME list sentinel "
            "0x%08x <- 0xffffffff (sentinel=%llu)\n",
            list_sentinel,
            (unsigned long long)ramtest_mame_state_fastfill_sentinels);
    }

    ramtest_mame_state_fastfill_hits++;
    ramtest_mame_state_fastfill_bytes += len;
    std::fprintf(stderr,
        "[rom-boot] ramtest-mame-state host-filled RAM "
        "0x%08x..0x%08x (%u bytes, hit=%llu)\n",
        start, end, len,
        (unsigned long long)ramtest_mame_state_fastfill_hits);
}

struct BoundaryEvent {
    uint32_t seq = 0;
    uint32_t kind = DBG_BOUNDARY_NONE;
    uint32_t pc = 0;
    uint32_t next_pc = 0;
    uint8_t vec = 0;
    uint32_t fault_pc = 0;
    uint32_t fault_addr = 0;
    uint32_t keep_tag = 0;
};

struct PcBreakpoint {
    uint32_t pc = 0;
    uint64_t hits = 0;
};

static std::vector<PcBreakpoint> stop_pcs;
static std::vector<PcBreakpoint> end_pcs;
static uint64_t stop_pc_hit_target = 1;
static uint64_t end_pc_hit_target = 1;
static bool precise_boundary_pending = false;
static BoundaryEvent precise_boundary_stop;

static void serialize_map(VerilatedSerialize& os,
                          const std::map<uint32_t, uint64_t>& m) {
    uint32_t n = (uint32_t)m.size();
    os << n;
    for (const auto& kv : m) {
        os << kv.first;
        os << kv.second;
    }
}

static void deserialize_map(VerilatedDeserialize& os,
                            std::map<uint32_t, uint64_t>& m) {
    uint32_t n = 0;
    os >> n;
    m.clear();
    for (uint32_t i = 0; i < n; i++) {
        uint32_t key = 0;
        uint64_t val = 0;
        os >> key;
        os >> val;
        m[key] = val;
    }
}

static void serialize_harness(VerilatedSerialize& os) {
    os << sim_time;
    os << host_cycle;
    os << overlay;

    os << via1.orb << via1.ora << via1.ddrb << via1.ddra;
    os << via1.t1cl << via1.t1ch << via1.t1ll << via1.t1lh;
    os << via1.t2cl << via1.t2ch << via1.sr << via1.acr;
    os << via1.pcr << via1.ifr << via1.ier;
    os << via1.adb_shift_in_progress;
    os << via1.adb_shift_complete_cyc;
    os << via1.adb_last_byte;
    os << via1.t1_running << via1.t1_latch << via1.t1_counter;
    os << via1.t2_running << via1.t2_latch << via1.t2_counter;
    os << via1.adb_transaction_id;
    for (int i = 0; i < 16; i++) os << via2.regs[i];

    serialize_map(os, unmapped_write_count);
    serialize_map(os, unmapped_read_count);

    os << (uint32_t)if_pending;
    os << if_addr_q;

    os << ax.ar_outstanding << ax.ar_addr;
    os << (uint32_t)ax.ar_len << (uint32_t)ax.ar_size;
    os << (uint32_t)ax.ar_burst << (uint32_t)ax.ar_beat;
    os << (uint32_t)ax.ar_delay;
    os << ax.aw_done << ax.w_done << ax.aw_addr << ax.w_data << ax.w_strb;
    os << (uint32_t)ax.b_delay << ax.b_pending;

    os << last_logged_pc;
    os << last_same_pc_count;

    uint32_t mem_size = (uint32_t)mem->size();
    os << mem_size;
    os.write(mem->data(), mem_size);

    os << rtc_sidechannel.initialized;
    os << rtc_sidechannel.prev_enb;
    os << rtc_sidechannel.prev_clk;
    os << rtc_sidechannel.active;
    os << rtc_sidechannel.is_read;
    os << rtc_sidechannel.cmd;
    os << rtc_sidechannel.data;
    os << rtc_sidechannel.cmd_bits;
    os << rtc_sidechannel.data_bits;
    os << rtc_sidechannel.read_shift;
    os << rtc_sidechannel.data_line;
    os << rtc_sidechannel.is_extended;
    os << rtc_sidechannel.xp_addr_done;
    os << rtc_sidechannel.xp_addr;
    os << rtc_sidechannel.write_protect;
    os << rtc_sidechannel.test_mode;
    os << rtc_sidechannel.seconds_base;
    os << rtc_sidechannel.seconds_base_cycle;
    for (uint8_t b : rtc_sidechannel.pram) os << b;
}

static bool deserialize_harness(VerilatedDeserialize& os, uint32_t version) {
    uint32_t tmp32 = 0;

    os >> sim_time;
    os >> host_cycle;
    os >> overlay;

    os >> via1.orb >> via1.ora >> via1.ddrb >> via1.ddra;
    os >> via1.t1cl >> via1.t1ch >> via1.t1ll >> via1.t1lh;
    os >> via1.t2cl >> via1.t2ch >> via1.sr >> via1.acr;
    os >> via1.pcr >> via1.ifr >> via1.ier;
    os >> via1.adb_shift_in_progress;
    os >> via1.adb_shift_complete_cyc;
    os >> via1.adb_last_byte;
    if (version >= 4) {
        os >> via1.t1_running >> via1.t1_latch >> via1.t1_counter;
        os >> via1.t2_running >> via1.t2_latch >> via1.t2_counter;
        os >> via1.adb_transaction_id;
    } else {
        via1.t1_running = false;
        via1.t1_latch = (uint16_t)(((uint16_t)via1.t1lh << 8) | via1.t1ll);
        via1.t1_counter = (uint16_t)(((uint16_t)via1.t1ch << 8) | via1.t1cl);
        via1.t2_running = false;
        via1.t2_latch = (uint16_t)(((uint16_t)via1.t2ch << 8) | via1.t2cl);
        via1.t2_counter = via1.t2_latch;
        via1.adb_transaction_id = 0;
    }
    for (int i = 0; i < 16; i++) os >> via2.regs[i];

    deserialize_map(os, unmapped_write_count);
    deserialize_map(os, unmapped_read_count);

    os >> tmp32;
    if_pending = (int)tmp32;
    os >> if_addr_q;

    os >> ax.ar_outstanding >> ax.ar_addr;
    if (version >= 5) {
        os >> tmp32;
        ax.ar_len = (uint8_t)tmp32;
        os >> tmp32;
        ax.ar_size = (uint8_t)tmp32;
        os >> tmp32;
        ax.ar_burst = (uint8_t)tmp32;
        os >> tmp32;
        ax.ar_beat = (uint8_t)tmp32;
    } else {
        ax.ar_len = 0;
        ax.ar_size = 2;
        ax.ar_burst = 1;
        ax.ar_beat = 0;
    }
    os >> tmp32;
    ax.ar_delay = (int)tmp32;
    os >> ax.aw_done >> ax.w_done >> ax.aw_addr >> ax.w_data >> ax.w_strb;
    os >> tmp32 >> ax.b_pending;
    ax.b_delay = (int)tmp32;
    ax.b_resp = ax.b_pending ? daxi_bresp_for_addr(ax.aw_addr) : 0;

    os >> last_logged_pc;
    os >> last_same_pc_count;

    uint32_t mem_size = 0;
    os >> mem_size;
    if (mem_size != mem->size()) {
        std::fprintf(stderr,
            "[rom-boot] checkpoint memory size %u != harness size %zu\n",
            mem_size, mem->size());
        return false;
    }
    os.read(mem->data(), mem_size);

    if (version >= 2) {
        os >> rtc_sidechannel.initialized;
        os >> rtc_sidechannel.prev_enb;
        os >> rtc_sidechannel.prev_clk;
        os >> rtc_sidechannel.active;
        os >> rtc_sidechannel.is_read;
        os >> rtc_sidechannel.cmd;
        os >> rtc_sidechannel.data;
        os >> rtc_sidechannel.cmd_bits;
        os >> rtc_sidechannel.data_bits;
        os >> rtc_sidechannel.read_shift;
        os >> rtc_sidechannel.data_line;
        if (version >= 3) {
            os >> rtc_sidechannel.is_extended;
            os >> rtc_sidechannel.xp_addr_done;
            os >> rtc_sidechannel.xp_addr;
            os >> rtc_sidechannel.write_protect;
            os >> rtc_sidechannel.test_mode;
            os >> rtc_sidechannel.seconds_base;
            os >> rtc_sidechannel.seconds_base_cycle;
            for (uint8_t& b : rtc_sidechannel.pram) os >> b;
        } else {
            os >> rtc_sidechannel.seconds_base;
            os >> rtc_sidechannel.seconds_base_cycle;
            for (size_t i = 0; i < rtc_sidechannel.pram.size(); i++)
                rtc_sidechannel.pram[i] = RtcSidechannelModel::pram_reset_byte((uint8_t)i);
            for (size_t i = 0; i < 20; i++) {
                uint8_t b = 0;
                os >> b;
                rtc_sidechannel.pram[i] = b;
            }
            rtc_sidechannel.is_extended = false;
            rtc_sidechannel.xp_addr_done = false;
            rtc_sidechannel.xp_addr = 0;
            rtc_sidechannel.write_protect = false;
            rtc_sidechannel.test_mode = false;
        }
    } else {
        rtc_sidechannel.reset(host_cycle);
    }
    return true;
}

static bool save_checkpoint(const std::string& path) {
    if (!dut || !mem) return false;
    VerilatedSave os;
    os.open(path);
    if (!os.isOpen()) {
        std::fprintf(stderr, "[rom-boot] cannot open checkpoint for write: %s\n",
                     path.c_str());
        return false;
    }
    const uint32_t magic = 0x5142434bu;  // "QBCK"
    const uint32_t version = 5;
    os << magic << version;
    serialize_harness(os);
    os << *dut;
    os.close();
    std::fprintf(stderr,
        "[rom-boot] checkpoint saved: %s (committed=%u pc=0x%08x)\n",
        path.c_str(), (unsigned)dut->dbg_committed, (unsigned)dut->dbg_last_pc);
    return true;
}

static bool restore_checkpoint(const std::string& path) {
    if (!dut || !mem) return false;
    VerilatedRestore os;
    os.open(path);
    if (!os.isOpen()) {
        std::fprintf(stderr, "[rom-boot] cannot open checkpoint for read: %s\n",
                     path.c_str());
        return false;
    }
    uint32_t magic = 0;
    uint32_t version = 0;
    os >> magic >> version;
    if (magic != 0x5142434bu || (version < 1 || version > 5)) {
        std::fprintf(stderr,
            "[rom-boot] bad checkpoint header in %s: magic=0x%08x version=%u\n",
            path.c_str(), magic, version);
        return false;
    }
    if (!deserialize_harness(os, version)) return false;
    os >> *dut;
    os.close();
    test_done = false;
    stop_reason = "restored";
    std::fprintf(stderr,
        "[rom-boot] checkpoint restored: %s (committed=%u pc=0x%08x)\n",
        path.c_str(), (unsigned)dut->dbg_committed, (unsigned)dut->dbg_last_pc);
    return true;
}

static std::string checkpoint_path(uint64_t committed) {
    char path[512];
    std::snprintf(path, sizeof(path), "%s.%08llu.vlt",
                  checkpoint_prefix.c_str(),
                  (unsigned long long)committed);
    return std::string(path);
}

static std::string checkpoint_cycle_path(uint64_t cycle) {
    char path[512];
    std::snprintf(path, sizeof(path), "%s.cyc%08llu.vlt",
                  checkpoint_prefix.c_str(),
                  (unsigned long long)cycle);
    return std::string(path);
}

static void maybe_periodic_checkpoint() {
    if (checkpoint_prefix.empty()) return;
    uint64_t committed = (uint64_t)dut->dbg_committed;
    while (next_checkpoint_point < checkpoint_points.size() &&
           committed >= checkpoint_points[next_checkpoint_point]) {
        save_checkpoint(checkpoint_path(checkpoint_points[next_checkpoint_point]));
        next_checkpoint_point++;
    }
    if (checkpoint_every != 0 && committed >= next_checkpoint_commit) {
        save_checkpoint(checkpoint_path(committed));
        next_checkpoint_commit = committed + checkpoint_every;
    }
}

static void maybe_cycle_checkpoint() {
    if (checkpoint_prefix.empty()) return;
    while (next_checkpoint_cycle_point < checkpoint_cycle_points.size() &&
           sim_time >= checkpoint_cycle_points[next_checkpoint_cycle_point]) {
        save_checkpoint(
            checkpoint_cycle_path(
                checkpoint_cycle_points[next_checkpoint_cycle_point]));
        next_checkpoint_cycle_point++;
    }
}

static bool mkdir_parent_dirs(const std::string& path) {
    size_t slash = path.find('/');
    while (slash != std::string::npos) {
        if (slash > 0) {
            std::string dir = path.substr(0, slash);
            if (::mkdir(dir.c_str(), 0777) != 0 && errno != EEXIST) {
                std::fprintf(stderr,
                    "[rom-boot] cannot create checkpoint directory %s: %s\n",
                    dir.c_str(), std::strerror(errno));
                return false;
            }
        }
        slash = path.find('/', slash + 1);
    }
    return true;
}

static void write_arch_sample_header(FILE* fp) {
    std::fprintf(fp,
        "# columns: side committed cycles pc sr ccr "
        "d0 d1 d2 d3 d4 d5 d6 d7 "
        "a0 a1 a2 a3 a4 a5 a6 a7 usp isp sfc dfc vbr\n");
}

static bool init_arch_sample_logger() {
    if (arch_sample_every == 0) return true;
    if (arch_sample_log_path.empty()) {
        std::fprintf(stderr,
            "[rom-boot] +arch_sample_every requires +arch_sample_log=<path>\n");
        return false;
    }
    if (!mkdir_parent_dirs(arch_sample_log_path)) return false;
    arch_sample_fp = std::fopen(arch_sample_log_path.c_str(), "w");
    if (!arch_sample_fp) {
        std::fprintf(stderr,
            "[rom-boot] cannot open arch sample log %s: %s\n",
            arch_sample_log_path.c_str(), std::strerror(errno));
        return false;
    }
    write_arch_sample_header(arch_sample_fp);
    std::fprintf(stderr,
        "[rom-boot] arch sample log: %s every=%llu committed instructions\n",
        arch_sample_log_path.c_str(),
        (unsigned long long)arch_sample_every);
    return true;
}

static void write_arch_sample(uint64_t committed, uint64_t cycles) {
    if (!arch_sample_fp || !dut) return;
    auto* r = dut->rootp;
    const unsigned ccr =
        r->mac_top__DOT__cpu__DOT__u_ccr_rat__DOT__ccr_prf
            [r->mac_top__DOT__cpu__DOT__u_ccr_rat__DOT__crat_tag] & 0x1f;
    const uint16_t sr =
        (uint16_t)((r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_sr & 0xffe0u)
                   | ccr);
    const uint32_t pc = arch_next_pc_valid ? arch_next_pc : (uint32_t)dut->dbg_pc;

    std::fprintf(arch_sample_fp,
                 "rtl %llu %llu 0x%08x 0x%04x 0x%02x",
                 (unsigned long long)committed,
                 (unsigned long long)cycles,
                 pc,
                 sr,
                 ccr);
    for (int i = 0; i < 8; i++)
        std::fprintf(arch_sample_fp, " 0x%08x", read_arch_reg(i));
    for (int i = 0; i < 8; i++)
        std::fprintf(arch_sample_fp, " 0x%08x", read_arch_reg(8 + i));
    std::fprintf(arch_sample_fp, " 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__usp,
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__isp,
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_sfc,
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_dfc,
                 (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_vbr);
}

static void maybe_arch_sample(uint64_t committed) {
    if (arch_sample_every == 0 || !arch_sample_fp) return;
    if (committed >= next_arch_sample_commit) {
        arch_sample_pending = true;
        arch_sample_pending_commit = committed;
        next_arch_sample_commit = committed + arch_sample_every;
    }
}

static void flush_pending_arch_sample() {
    if (!arch_sample_pending || !arch_sample_fp || !dut) return;
    if ((uint64_t)dut->dbg_committed < arch_sample_pending_commit) return;
    write_arch_sample(arch_sample_pending_commit, sim_time);
    arch_sample_pending = false;
}

static uint64_t fnv1a64_bytes(const uint8_t* data, size_t len) {
    uint64_t h = 0xcbf29ce484222325ULL;
    for (size_t i = 0; i < len; i++) {
        h ^= data[i];
        h *= 0x100000001b3ULL;
    }
    return h;
}

static void fprint_hex_bytes(FILE* fp, const uint8_t* data, size_t len) {
    static const char hexdigits[] = "0123456789abcdef";
    for (size_t i = 0; i < len; i++) {
        std::fputc(hexdigits[data[i] >> 4], fp);
        std::fputc(hexdigits[data[i] & 0x0f], fp);
    }
}

static void write_full_segment(FILE* fp,
                               const char* name,
                               uint32_t base,
                               const uint8_t* data,
                               size_t size) {
    constexpr size_t CHUNK = 32;
    std::fprintf(fp,
        "segment name=%s base=0x%08x size=0x%08llx encoding=hex-full "
        "chunk_bytes=%zu checksum_fnv1a64=0x%016llx\n",
        name, base, (unsigned long long)size, CHUNK,
        (unsigned long long)fnv1a64_bytes(data, size));
    for (size_t off = 0; off < size; off += CHUNK) {
        const size_t n = std::min(CHUNK, size - off);
        std::fprintf(fp, "data off=0x%08llx bytes=%zu hex=",
                     (unsigned long long)off, n);
        fprint_hex_bytes(fp, data + off, n);
        std::fputc('\n', fp);
    }
    std::fprintf(fp, "endsegment name=%s\n", name);
}

static void write_sparse_segment(FILE* fp,
                                 const char* name,
                                 uint32_t base,
                                 const uint8_t* data,
                                 size_t size,
                                 uint8_t default_value) {
    constexpr size_t CHUNK = 32;
    uint64_t material_bytes = 0;
    uint64_t chunks = 0;
    for (size_t off = 0; off < size; ) {
        if (data[off] == default_value) {
            off++;
            continue;
        }
        size_t n = 0;
        while (off + n < size && n < CHUNK && data[off + n] != default_value)
            n++;
        material_bytes += n;
        chunks++;
        off += n;
    }

    std::fprintf(fp,
        "segment name=%s base=0x%08x size=0x%08llx encoding=sparse-hex "
        "default=0x%02x chunk_bytes=%zu material_bytes=%llu chunks=%llu "
        "checksum_fnv1a64=0x%016llx\n",
        name, base, (unsigned long long)size, default_value, CHUNK,
        (unsigned long long)material_bytes, (unsigned long long)chunks,
        (unsigned long long)fnv1a64_bytes(data, size));
    for (size_t off = 0; off < size; ) {
        if (data[off] == default_value) {
            off++;
            continue;
        }
        size_t n = 0;
        while (off + n < size && n < CHUNK && data[off + n] != default_value)
            n++;
        std::fprintf(fp, "data off=0x%08llx bytes=%zu hex=",
                     (unsigned long long)off, n);
        fprint_hex_bytes(fp, data + off, n);
        std::fputc('\n', fp);
        off += n;
    }
    std::fprintf(fp, "endsegment name=%s\n", name);
}

struct DcacheFlushReport {
    bool attempted = false;
    bool completed = false;
    uint64_t start_cycle = 0;
    uint64_t end_cycle = 0;
    uint32_t start_committed = 0;
    uint32_t end_committed = 0;
};

static bool arch_checkpoint_quiesced() {
    if (!dut) return false;
    auto* r = dut->rootp;
    const unsigned rob_head =
        r->mac_top__DOT__cpu__DOT__u_rob__DOT__head_ptr & 0x3f;
    const unsigned rob_tail =
        r->mac_top__DOT__cpu__DOT__u_rob__DOT__tail_ptr & 0x3f;
    return (rob_head == rob_tail) &&
           ((r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__cnt_r & 0xf) == 0) &&
           ((r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__cnt_r & 0xf) == 0) &&
           !r->mac_top__DOT__cpu__DOT__lsu_busy &&
           !r->mac_top__DOT__cpu__DOT__exc_active &&
           !r->mac_top__DOT__cpu__DOT__u_commit__DOT__cache_maint_wait &&
           (r->mac_top__DOT__cpu__DOT__u_dcache__DOT__state < 11) &&
           !r->mac_top__DOT__cpu__DOT__fq_valid[0] &&
           !r->mac_top__DOT__cpu__DOT__fq_valid[1];
}

template <typename NoteCommitProgress>
static bool drain_arch_checkpoint_pipeline(uint64_t budget,
                                           NoteCommitProgress note_commit_progress) {
    if (!dut) return false;
    for (uint64_t i = 0; i < budget; i++) {
        if (arch_checkpoint_quiesced()) return true;
        // Stop instruction supply while allowing data-side transactions,
        // retire, exception, and cache-maintenance work to drain.
        dut->if_rvalid = 0;
        dut->if_fault = 0;
        if_pending = 0;
        drive_daxi();
        tick();
        note_commit_progress();
    }
    return arch_checkpoint_quiesced();
}

static bool engage_precise_boundary_halt() {
    if (!dut || !precise_boundary_pending) return false;
    dut->dbg_core_halt = 1;
    dut->dbg_precise_stop_keep_tag = precise_boundary_stop.keep_tag & 0x1f;
    dut->dbg_precise_stop_req = 1;
    dut->if_rvalid = 0;
    dut->if_fault = 0;
    if_pending = 0;
    drive_daxi();
    tick();
    dut->dbg_precise_stop_req = 0;
    dut->eval();
    return true;
}

static void release_precise_boundary_halt() {
    if (!dut) return;
    dut->dbg_precise_stop_req = 0;
    dut->dbg_precise_stop_keep_tag = 0;
    dut->dbg_core_halt = 0;
    dut->eval();
}

static DcacheFlushReport flush_dcache_before_arch_checkpoint() {
    DcacheFlushReport report;
    if (!dut) return report;

    report.attempted = true;
    report.start_cycle = sim_time;
    report.start_committed = (uint32_t)dut->dbg_committed;
    const uint64_t saved_stop_cycle = stop_cycle;
    const bool saved_test_done = test_done;
    const char* saved_stop_reason = stop_reason;
    stop_cycle = 0;

    uint32_t prev_committed = (uint32_t)dut->dbg_committed;
    const bool freeze_arch_next_pc = precise_boundary_pending;
    auto note_commit_progress = [&]() {
        uint32_t c = (uint32_t)dut->dbg_committed;
        if (!freeze_arch_next_pc && c != prev_committed) {
            auto* r = dut->rootp;
            arch_next_pc_valid = true;
            arch_next_pc =
                (uint32_t)r->mac_top__DOT__cpu__DOT__u_commit__DOT__actual_next;
            arch_next_pc_commit_pc = (uint32_t)dut->dbg_last_pc;
            arch_next_pc_committed = c;
            prev_committed = c;
        }
    };

    const bool drained_before_flush =
        drain_arch_checkpoint_pipeline(arch_checkpoint_flush_timeout,
                                       note_commit_progress);
    if (!drained_before_flush) {
        std::fprintf(stderr,
            "[rom-boot] arch checkpoint warning: pipeline did not quiesce "
            "before D-cache flush within %llu cycles\n",
            (unsigned long long)arch_checkpoint_flush_timeout);
    }

    dut->dbg_dcache_flush_req = 1;
    for (uint64_t i = 0; i < arch_checkpoint_flush_timeout; i++) {
        dut->if_rvalid = 0;
        dut->if_fault = 0;
        if_pending = 0;
        drive_daxi();
        tick();
        note_commit_progress();
        if (dut->dbg_dcache_flush_done) {
            report.completed = true;
            break;
        }
    }
    dut->dbg_dcache_flush_req = 0;
    dut->eval();

    const bool drained_after_flush =
        drain_arch_checkpoint_pipeline(arch_checkpoint_flush_timeout,
                                       note_commit_progress);
    if (!drained_after_flush) {
        std::fprintf(stderr,
            "[rom-boot] arch checkpoint warning: pipeline did not quiesce "
            "after D-cache flush within %llu cycles\n",
            (unsigned long long)arch_checkpoint_flush_timeout);
    }

    report.end_cycle = sim_time;
    report.end_committed = (uint32_t)dut->dbg_committed;
    stop_cycle = saved_stop_cycle;
    test_done = saved_test_done;
    stop_reason = saved_stop_reason;
    if (!report.completed) {
        std::fprintf(stderr,
            "[rom-boot] arch checkpoint warning: D-cache debug flush did not "
            "complete within %llu cycles\n",
            (unsigned long long)arch_checkpoint_flush_timeout);
    }
    return report;
}

static bool save_arch_checkpoint(const std::string& path) {
    if (!dut || !mem) return false;
    if (!mkdir_parent_dirs(path)) return false;

    const char* stop_reason_before_flush = stop_reason;
    const bool engaged_precise_halt = engage_precise_boundary_halt();
    const DcacheFlushReport flush_report = flush_dcache_before_arch_checkpoint();
    auto* r = dut->rootp;

    FILE* fp = std::fopen(path.c_str(), "w");
    if (!fp) {
        if (engaged_precise_halt) release_precise_boundary_halt();
        std::fprintf(stderr, "[rom-boot] cannot open arch checkpoint %s\n",
                     path.c_str());
        return false;
    }

    const unsigned ccr =
        r->mac_top__DOT__cpu__DOT__u_ccr_rat__DOT__ccr_prf
            [r->mac_top__DOT__cpu__DOT__u_ccr_rat__DOT__crat_tag] & 0x1f;
    const uint16_t sr =
        (uint16_t)((r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_sr & 0xffe0u)
                   | ccr);

    std::fprintf(fp, "format m68k-ooo-arch-checkpoint-v1\n");
    std::fprintf(fp, "producer tb-rom-boot\n");
    std::fprintf(fp, "endianness big\n");
    std::fprintf(fp, "run cycle=0x%016llx committed=0x%08x "
                     "stop_reason_before_flush=\"%s\" stop_reason=\"%s\" "
                     "overlay=%u visible_ram=0x%08x q700_descriptor_selected=%u\n",
        (unsigned long long)sim_time,
        (unsigned)dut->dbg_committed,
        stop_reason_before_flush,
        stop_reason,
        overlay ? 1u : 0u,
        (unsigned)visible_ram_size,
        q700_descriptor_selected ? 1u : 0u);
    std::fprintf(fp, "flush attempted=%u completed=%u start_cycle=0x%016llx "
                     "end_cycle=0x%016llx start_committed=0x%08x "
                     "end_committed=0x%08x\n",
        flush_report.attempted ? 1u : 0u,
        flush_report.completed ? 1u : 0u,
        (unsigned long long)flush_report.start_cycle,
        (unsigned long long)flush_report.end_cycle,
        (unsigned)flush_report.start_committed,
        (unsigned)flush_report.end_committed);
    const unsigned rob_head =
        r->mac_top__DOT__cpu__DOT__u_rob__DOT__head_ptr & 0x3f;
    const unsigned rob_tail =
        r->mac_top__DOT__cpu__DOT__u_rob__DOT__tail_ptr & 0x3f;
    const bool rob_empty = (rob_head == rob_tail);
    std::fprintf(fp,
        "quiesce rob_empty=%u rob_head=%u rob_tail=%u int_iq_count=%u "
        "mem_iq_count=%u lsu_busy=%u exc_active=%u cache_maint_wait=%u "
        "dcache_flush_busy=%u flush_queue_empty=%u\n",
        rob_empty ? 1u : 0u,
        rob_head,
        rob_tail,
        (unsigned)(r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__cnt_r & 0xf),
        (unsigned)(r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__cnt_r & 0xf),
        (unsigned)r->mac_top__DOT__cpu__DOT__lsu_busy,
        (unsigned)r->mac_top__DOT__cpu__DOT__exc_active,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__cache_maint_wait,
        (unsigned)(r->mac_top__DOT__cpu__DOT__u_dcache__DOT__state >= 11),
        (unsigned)(!r->mac_top__DOT__cpu__DOT__fq_valid[0] &&
                   !r->mac_top__DOT__cpu__DOT__fq_valid[1]));

    std::fprintf(fp, "arch ccr=0x%02x sr=0x%04x "
                     "sr_source=commit_arch_sr_15_5_plus_ccr_rat\n",
                 ccr, sr);
    for (int i = 0; i < 8; i++)
        std::fprintf(fp, "reg d%d=0x%08x\n", i, read_arch_reg(i));
    for (int i = 0; i < 8; i++)
        std::fprintf(fp, "reg a%d=0x%08x\n", i, read_arch_reg(8 + i));

    if (arch_next_pc_valid) {
        std::fprintf(fp,
            "pc next_valid=1 next=0x%08x source=debug_boundary_or_last_observed_retire "
            "commit_pc=0x%08x committed=0x%08x fetch_dbg_pc=0x%08x "
            "dbg_last_pc=0x%08x\n",
            arch_next_pc, arch_next_pc_commit_pc, arch_next_pc_committed,
            (unsigned)dut->dbg_pc, (unsigned)dut->dbg_last_pc);
    } else {
        std::fprintf(fp,
            "pc next_valid=0 next=unknown source=not_observed "
            "fetch_dbg_pc=0x%08x dbg_last_pc=0x%08x\n",
            (unsigned)dut->dbg_pc, (unsigned)dut->dbg_last_pc);
    }

    std::fprintf(fp,
        "control vbr=0x%08x cacr=0x%08x sfc=0x%08x dfc=0x%08x "
        "usp=0x%08x ssp=0x%08x isp=0x%08x\n",
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_vbr,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_cacr,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_sfc,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_dfc,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__usp,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__ssp,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__isp);
    std::fprintf(fp,
        "mmu tc=0x%08x itt0=0x%08x itt1=0x%08x dtt0=0x%08x "
        "dtt1=0x%08x urp=0x%08x srp=0x%08x\n",
        (unsigned)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__r_tc,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__r_itt0,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__r_itt1,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__r_dtt0,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__r_dtt1,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__r_urp,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__r_srp);

    std::fprintf(fp,
        "io via1 orb=0x%02x ora=0x%02x ddrb=0x%02x ddra=0x%02x "
        "t1cl=0x%02x t1ch=0x%02x t1ll=0x%02x t1lh=0x%02x "
        "t2cl=0x%02x t2ch=0x%02x sr=0x%02x acr=0x%02x pcr=0x%02x "
        "ifr=0x%02x ier=0x%02x t1_running=%u t1_latch=0x%04x "
        "t1_counter=0x%04x t2_running=%u t2_latch=0x%04x "
        "t2_counter=0x%04x adb_shift_in_progress=%u "
        "adb_shift_complete_cyc=0x%016llx adb_last_byte=0x%02x "
        "adb_transaction_id=0x%08x rtc_data_line=%u "
        "via1_timer_div=0x%016llx via1_timer_div_ctr=0x%016llx\n",
        via1.orb, via1.ora, via1.ddrb, via1.ddra,
        via1.t1cl, via1.t1ch, via1.t1ll, via1.t1lh,
        via1.t2cl, via1.t2ch, via1.sr, via1.acr, via1.pcr,
        via1.ifr, via1.ier,
        via1.t1_running ? 1u : 0u, via1.t1_latch, via1.t1_counter,
        via1.t2_running ? 1u : 0u, via1.t2_latch, via1.t2_counter,
        via1.adb_shift_in_progress ? 1u : 0u,
        (unsigned long long)via1.adb_shift_complete_cyc,
        via1.adb_last_byte, via1.adb_transaction_id,
        rtc_sidechannel.data_line ? 1u : 0u,
        (unsigned long long)via1_timer_div,
        (unsigned long long)via1_timer_div_ctr);
    std::fprintf(fp,
        "io q700_descriptor selected=%u entry=0x%08x feature=0x%08x\n",
        q700_descriptor_selected ? 1u : 0u,
        q700_descriptor_entry,
        q700_feature_word);
    std::fprintf(fp,
        "replay supported=debug_arch_load_v2 io_state=via1-v1 "
        "io_restore=partial-via1-v1 "
        "caches=reset queues=reset predictors=reset\n"
        "gap restore=debug_arch_load_is_simulation_only_until_fpga_debug_path_exists\n"
        "gap replay_io=only_via1_rtc_line_q700_descriptor_restored_other_io_reset\n"
        "gap peripherals=scc_scsi_asc_swim_dafb_not_part_of_arch_replay\n"
        "gap rtc=only_rtc_data_line_restored_in_arch_checkpoint_v2\n"
        "gap sr=observed_via_commit_internal_until_public_debug_port_exists\n"
        "gap pc=boundary_resume_pc_requires_dbg_boundary_next_pc\n");

    if (!active_rom_patches.empty()) {
        for (const auto& p : active_rom_patches) {
            std::fprintf(fp,
                "rom_patch set=%s off=0x%05x value=0x%02x desc=\"%s\"\n",
                p.set, p.off, p.value, p.description);
        }
    }

    std::fprintf(fp, "memory begin\n");
    if (!rom_img.empty()) {
        write_full_segment(fp, "q700-rom", Q700_ROM_BASE,
                           rom_img.data(), rom_img.size());
    }
    const uint8_t* raw = mem->data();
    write_sparse_segment(fp, "ram", MemModel::RAM_BASE,
                         raw + MemModel::RAM_OFF, MemModel::RAM_BACKING_SIZE,
                         MemModel::DEFAULT_FILL);
    write_sparse_segment(fp, "vram", MemModel::VRAM_BASE,
                         raw + MemModel::VRAM_OFF, MemModel::VRAM_SIZE,
                         MemModel::DEFAULT_FILL);
    write_sparse_segment(fp, "magic", MemModel::MAGIC_BASE,
                         raw + MemModel::MAGIC_OFF, MemModel::MAGIC_SIZE,
                         MemModel::DEFAULT_FILL);
    std::fprintf(fp, "memory end\n");
    std::fprintf(fp, "end format=m68k-ooo-arch-checkpoint-v1\n");

    const bool ok = std::fclose(fp) == 0;
    if (engaged_precise_halt) release_precise_boundary_halt();
    precise_boundary_pending = false;
    if (!ok) {
        std::fprintf(stderr, "[rom-boot] error closing arch checkpoint %s\n",
                     path.c_str());
        return false;
    }
    std::fprintf(stderr,
        "[rom-boot] arch checkpoint saved: %s (committed=%u next_valid=%u "
        "next=0x%08x)\n",
        path.c_str(), (unsigned)dut->dbg_committed,
        arch_next_pc_valid ? 1u : 0u, arch_next_pc);
    return true;
}

static bool token_value(const std::string& line,
                        const char* key,
                        std::string& out) {
    const std::string needle = std::string(key) + "=";
    size_t pos = line.find(needle);
    if (pos == std::string::npos) return false;
    pos += needle.size();
    size_t end = pos;
    if (end < line.size() && line[end] == '"') {
        end++;
        while (end < line.size() && line[end] != '"') end++;
        if (end < line.size()) end++;
    } else {
        while (end < line.size() &&
               !std::isspace((unsigned char)line[end])) end++;
    }
    out = line.substr(pos, end - pos);
    if (out.size() >= 2 && out.front() == '"' && out.back() == '"')
        out = out.substr(1, out.size() - 2);
    return true;
}

static bool parse_u64_field(const std::string& text,
                            uint64_t& out,
                            const char* what) {
    char* end = nullptr;
    errno = 0;
    unsigned long long value = std::strtoull(text.c_str(), &end, 0);
    if (errno != 0 || end == text.c_str() || *end != '\0') {
        std::fprintf(stderr, "[rom-boot] bad %s value '%s'\n",
                     what, text.c_str());
        return false;
    }
    out = (uint64_t)value;
    return true;
}

static bool parse_u32_field(const std::string& text,
                            uint32_t& out,
                            const char* what) {
    uint64_t v = 0;
    if (!parse_u64_field(text, v, what) || v > 0xffffffffULL) return false;
    out = (uint32_t)v;
    return true;
}

static bool parse_hex_byte_pair(char hi, char lo, uint8_t& out) {
    auto nibble = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    int h = nibble(hi);
    int l = nibble(lo);
    if (h < 0 || l < 0) return false;
    out = (uint8_t)((h << 4) | l);
    return true;
}

struct ArchReplayCheckpoint {
    uint64_t cycle = 0;
    uint32_t committed = 0;
    uint32_t overlay = 1;
    uint32_t visible_ram = MemModel::RAM_WINDOW_DEFAULT;
    uint32_t regs[16] = {};
    uint32_t ccr = 0;
    uint32_t sr = 0x2000;
    uint32_t pc = Q700_RESET_PC;
    uint32_t commit_pc = 0;
    uint32_t vbr = 0;
    uint32_t cacr = 0;
    uint32_t sfc = 0;
    uint32_t dfc = 0;
    uint32_t usp = 0;
    uint32_t ssp = 0;
    uint32_t isp = 0;
    uint32_t tc = 0;
    uint32_t itt0 = 0;
    uint32_t itt1 = 0;
    uint32_t dtt0 = 0;
    uint32_t dtt1 = 0;
    uint32_t urp = 0;
    uint32_t srp = 0;
    uint32_t flush_attempted = 0;
    uint32_t flush_completed = 0;
    uint32_t quiesce_rob_empty = 0;
    uint32_t quiesce_int_iq_count = 0;
    uint32_t quiesce_mem_iq_count = 0;
    uint32_t quiesce_lsu_busy = 0;
    uint32_t quiesce_exc_active = 0;
    uint32_t quiesce_cache_maint_wait = 0;
    uint32_t quiesce_dcache_flush_busy = 0;
    uint32_t quiesce_flush_queue_empty = 0;
    std::string replay_supported;
    std::string replay_io_state;
    std::string replay_io_restore;
    std::string replay_caches;
    std::string replay_queues;
    std::string replay_predictors;
    Via1 via1;
    uint64_t via1_timer_div_snapshot = 1;
    uint64_t via1_timer_div_ctr_snapshot = 0;
    uint32_t via1_rtc_data_line = 1;
    uint32_t q700_descriptor_selected_snapshot = 0;
    uint32_t q700_descriptor_entry_snapshot = 0;
    uint32_t q700_feature_word_snapshot = 0;
    bool has_format = false;
    bool has_run = false;
    bool has_overlay = false;
    bool has_visible_ram = false;
    bool has_flush = false;
    bool has_quiesce = false;
    bool has_arch = false;
    bool has_pc = false;
    bool has_control = false;
    bool has_mmu = false;
    bool has_replay = false;
    bool has_io_via1 = false;
    bool has_io_q700_descriptor = false;
    bool has_reg[16] = {};
    bool saw_memory_begin = false;
    bool saw_memory_end = false;
    bool saw_end_format = false;
    bool saw_rom = false;
    bool saw_ram = false;
    bool saw_vram = false;
    bool saw_magic = false;
};

static bool parse_u8_token(const std::string& line,
                           const char* key,
                           uint8_t& out,
                           const char* what) {
    std::string v;
    uint32_t tmp = 0;
    if (!token_value(line, key, v) ||
        !parse_u32_field(v, tmp, what) || tmp > 0xffu)
        return false;
    out = (uint8_t)tmp;
    return true;
}

static bool parse_u16_token(const std::string& line,
                            const char* key,
                            uint16_t& out,
                            const char* what) {
    std::string v;
    uint32_t tmp = 0;
    if (!token_value(line, key, v) ||
        !parse_u32_field(v, tmp, what) || tmp > 0xffffu)
        return false;
    out = (uint16_t)tmp;
    return true;
}

static bool parse_bool_token(const std::string& line,
                             const char* key,
                             bool& out,
                             const char* what) {
    std::string v;
    uint32_t tmp = 0;
    if (!token_value(line, key, v) ||
        !parse_u32_field(v, tmp, what) || tmp > 1u)
        return false;
    out = tmp != 0;
    return true;
}

static bool parse_u32_token(const std::string& line,
                            const char* key,
                            uint32_t& out,
                            const char* what) {
    std::string v;
    return token_value(line, key, v) && parse_u32_field(v, out, what);
}

static bool parse_u64_token(const std::string& line,
                            const char* key,
                            uint64_t& out,
                            const char* what) {
    std::string v;
    return token_value(line, key, v) && parse_u64_field(v, out, what);
}

struct ReplaySegment {
    std::string name;
    uint32_t base = 0;
    uint64_t size = 0;
    std::string encoding;
    uint8_t default_value = 0xff;
    bool active = false;
};

static bool replay_segment_target(const ReplaySegment& seg,
                                  uint8_t*& target,
                                  size_t& target_size) {
    target = nullptr;
    target_size = 0;
    if (seg.name == "q700-rom") {
        if (seg.base != Q700_ROM_BASE || seg.size == 0 ||
            seg.size > Q700_ROM_SIZE) {
            std::fprintf(stderr,
                "[rom-boot] arch replay bad q700-rom segment base/size\n");
            return false;
        }
        rom_img.assign((size_t)seg.size, seg.default_value);
        target = rom_img.data();
        target_size = rom_img.size();
        return true;
    }
    uint8_t* raw = mem ? mem->data() : nullptr;
    if (!raw) return false;
    if (seg.name == "ram") {
        if (seg.base != MemModel::RAM_BASE || seg.size != MemModel::RAM_BACKING_SIZE)
            return false;
        target = raw + MemModel::RAM_OFF;
        target_size = MemModel::RAM_BACKING_SIZE;
    } else if (seg.name == "vram") {
        if (seg.base != MemModel::VRAM_BASE || seg.size != MemModel::VRAM_SIZE)
            return false;
        target = raw + MemModel::VRAM_OFF;
        target_size = MemModel::VRAM_SIZE;
    } else if (seg.name == "magic") {
        if (seg.base != MemModel::MAGIC_BASE ||
            seg.size != MemModel::MAGIC_SIZE)
            return false;
        target = raw + MemModel::MAGIC_OFF;
        target_size = MemModel::MAGIC_SIZE;
    } else {
        std::fprintf(stderr,
            "[rom-boot] arch replay unknown memory segment '%s'\n",
            seg.name.c_str());
        return false;
    }
    std::memset(target, seg.default_value, target_size);
    return true;
}

static bool parse_arch_replay_checkpoint(const std::string& path,
                                         ArchReplayCheckpoint& cp) {
    FILE* fp = std::fopen(path.c_str(), "r");
    if (!fp) {
        std::fprintf(stderr, "[rom-boot] cannot open arch replay %s\n",
                     path.c_str());
        return false;
    }

    char buf[8192];
    ReplaySegment seg;
    uint8_t* seg_target = nullptr;
    size_t seg_target_size = 0;
    uint64_t line_no = 0;

    while (std::fgets(buf, sizeof(buf), fp)) {
        line_no++;
        std::string line(buf);
        while (!line.empty() &&
               (line.back() == '\n' || line.back() == '\r'))
            line.pop_back();
        if (line.empty()) continue;

        if (line == "format m68k-ooo-arch-checkpoint-v1") {
            cp.has_format = true;
            continue;
        }
        if (line == "memory begin") {
            cp.saw_memory_begin = true;
            continue;
        }
        if (line == "memory end") {
            cp.saw_memory_end = true;
            continue;
        }
        if (line == "end format=m68k-ooo-arch-checkpoint-v1") {
            cp.saw_end_format = true;
            continue;
        }
        if (line.rfind("run ", 0) == 0) {
            std::string v;
            if (token_value(line, "cycle", v) &&
                !parse_u64_field(v, cp.cycle, "run.cycle")) return false;
            if (token_value(line, "committed", v) &&
                !parse_u32_field(v, cp.committed, "run.committed")) return false;
            if (token_value(line, "visible_ram", v)) {
                if (!parse_u32_field(v, cp.visible_ram,
                                     "run.visible_ram")) return false;
                if (cp.visible_ram == 0 ||
                    cp.visible_ram > MemModel::RAM_BACKING_SIZE ||
                    cp.visible_ram < MemModel::RAM_WINDOW_MIN ||
                    (cp.visible_ram & 0x3u) != 0 ||
                    (cp.visible_ram & (cp.visible_ram - 1u)) != 0) {
                    std::fprintf(stderr,
                        "[rom-boot] arch replay bad run.visible_ram=0x%08x\n",
                        cp.visible_ram);
                    return false;
                }
                cp.has_visible_ram = true;
            }
            if (token_value(line, "overlay", v)) {
                if (!parse_u32_field(v, cp.overlay, "run.overlay")) return false;
                if (cp.overlay > 1) {
                    std::fprintf(stderr,
                        "[rom-boot] arch replay bad run.overlay=%u\n",
                        cp.overlay);
                    return false;
                }
                cp.has_overlay = true;
            }
            cp.has_run = true;
            continue;
        }
        if (line.rfind("flush ", 0) == 0) {
            std::string v;
            if (!token_value(line, "attempted", v) ||
                !parse_u32_field(v, cp.flush_attempted,
                                 "flush.attempted")) return false;
            if (!token_value(line, "completed", v) ||
                !parse_u32_field(v, cp.flush_completed,
                                 "flush.completed")) return false;
            cp.has_flush = true;
            continue;
        }
        if (line.rfind("quiesce ", 0) == 0) {
            std::string v;
            if (!token_value(line, "rob_empty", v) ||
                !parse_u32_field(v, cp.quiesce_rob_empty,
                                 "quiesce.rob_empty")) return false;
            if (!token_value(line, "int_iq_count", v) ||
                !parse_u32_field(v, cp.quiesce_int_iq_count,
                                 "quiesce.int_iq_count")) return false;
            if (!token_value(line, "mem_iq_count", v) ||
                !parse_u32_field(v, cp.quiesce_mem_iq_count,
                                 "quiesce.mem_iq_count")) return false;
            if (!token_value(line, "lsu_busy", v) ||
                !parse_u32_field(v, cp.quiesce_lsu_busy,
                                 "quiesce.lsu_busy")) return false;
            if (!token_value(line, "exc_active", v) ||
                !parse_u32_field(v, cp.quiesce_exc_active,
                                 "quiesce.exc_active")) return false;
            if (!token_value(line, "cache_maint_wait", v) ||
                !parse_u32_field(v, cp.quiesce_cache_maint_wait,
                                 "quiesce.cache_maint_wait")) return false;
            if (!token_value(line, "dcache_flush_busy", v) ||
                !parse_u32_field(v, cp.quiesce_dcache_flush_busy,
                                 "quiesce.dcache_flush_busy")) return false;
            if (!token_value(line, "flush_queue_empty", v) ||
                !parse_u32_field(v, cp.quiesce_flush_queue_empty,
                                 "quiesce.flush_queue_empty")) return false;
            cp.has_quiesce = true;
            continue;
        }
        if (line.rfind("arch ", 0) == 0) {
            std::string v;
            if (!token_value(line, "ccr", v) ||
                !parse_u32_field(v, cp.ccr, "arch.ccr")) return false;
            if (!token_value(line, "sr", v) ||
                !parse_u32_field(v, cp.sr, "arch.sr")) return false;
            cp.has_arch = true;
            continue;
        }
        if (line.rfind("reg ", 0) == 0) {
            if (line.size() < 8 || line[6] != '=') return false;
            char kind = line[4];
            int n = line[5] - '0';
            if ((kind != 'd' && kind != 'a') || n < 0 || n > 7)
                return false;
            uint32_t value = 0;
            if (!parse_u32_field(line.substr(7), value, "reg")) return false;
            unsigned idx = (kind == 'd') ? (unsigned)n : (8u + (unsigned)n);
            cp.regs[idx] = value;
            cp.has_reg[idx] = true;
            continue;
        }
        if (line.rfind("pc ", 0) == 0) {
            std::string v;
            uint32_t valid = 0;
            if (!token_value(line, "next_valid", v) ||
                !parse_u32_field(v, valid, "pc.next_valid") || valid == 0) {
                std::fprintf(stderr,
                    "[rom-boot] arch replay checkpoint has no valid next PC\n");
                return false;
            }
            if (!token_value(line, "next", v) ||
                !parse_u32_field(v, cp.pc, "pc.next")) return false;
            if (token_value(line, "commit_pc", v))
                (void)parse_u32_field(v, cp.commit_pc, "pc.commit_pc");
            cp.has_pc = true;
            continue;
        }
        if (line.rfind("control ", 0) == 0) {
            std::string v;
            if (!token_value(line, "vbr", v) ||
                !parse_u32_field(v, cp.vbr, "control.vbr")) return false;
            if (!token_value(line, "cacr", v) ||
                !parse_u32_field(v, cp.cacr, "control.cacr")) return false;
            if (!token_value(line, "sfc", v) ||
                !parse_u32_field(v, cp.sfc, "control.sfc")) return false;
            if (!token_value(line, "dfc", v) ||
                !parse_u32_field(v, cp.dfc, "control.dfc")) return false;
            if (!token_value(line, "usp", v) ||
                !parse_u32_field(v, cp.usp, "control.usp")) return false;
            if (!token_value(line, "ssp", v) ||
                !parse_u32_field(v, cp.ssp, "control.ssp")) return false;
            if (!token_value(line, "isp", v) ||
                !parse_u32_field(v, cp.isp, "control.isp")) return false;
            cp.has_control = true;
            continue;
        }
        if (line.rfind("mmu ", 0) == 0) {
            std::string v;
            if (!token_value(line, "tc", v) ||
                !parse_u32_field(v, cp.tc, "mmu.tc")) return false;
            if (!token_value(line, "itt0", v) ||
                !parse_u32_field(v, cp.itt0, "mmu.itt0")) return false;
            if (!token_value(line, "itt1", v) ||
                !parse_u32_field(v, cp.itt1, "mmu.itt1")) return false;
            if (!token_value(line, "dtt0", v) ||
                !parse_u32_field(v, cp.dtt0, "mmu.dtt0")) return false;
            if (!token_value(line, "dtt1", v) ||
                !parse_u32_field(v, cp.dtt1, "mmu.dtt1")) return false;
            if (!token_value(line, "urp", v) ||
                !parse_u32_field(v, cp.urp, "mmu.urp")) return false;
            if (!token_value(line, "srp", v) ||
                !parse_u32_field(v, cp.srp, "mmu.srp")) return false;
            cp.has_mmu = true;
            continue;
        }
        if (line.rfind("replay ", 0) == 0) {
            if (!token_value(line, "supported", cp.replay_supported))
                return false;
            if (!token_value(line, "io_state", cp.replay_io_state))
                return false;
            (void)token_value(line, "io_restore", cp.replay_io_restore);
            if (!token_value(line, "caches", cp.replay_caches))
                return false;
            if (!token_value(line, "queues", cp.replay_queues))
                return false;
            if (!token_value(line, "predictors", cp.replay_predictors))
                return false;
            cp.has_replay = true;
            continue;
        }
        if (line.rfind("io via1 ", 0) == 0) {
            uint32_t rtc_line = 1;
            if (!parse_u8_token(line, "orb", cp.via1.orb, "io.via1.orb") ||
                !parse_u8_token(line, "ora", cp.via1.ora, "io.via1.ora") ||
                !parse_u8_token(line, "ddrb", cp.via1.ddrb, "io.via1.ddrb") ||
                !parse_u8_token(line, "ddra", cp.via1.ddra, "io.via1.ddra") ||
                !parse_u8_token(line, "t1cl", cp.via1.t1cl, "io.via1.t1cl") ||
                !parse_u8_token(line, "t1ch", cp.via1.t1ch, "io.via1.t1ch") ||
                !parse_u8_token(line, "t1ll", cp.via1.t1ll, "io.via1.t1ll") ||
                !parse_u8_token(line, "t1lh", cp.via1.t1lh, "io.via1.t1lh") ||
                !parse_u8_token(line, "t2cl", cp.via1.t2cl, "io.via1.t2cl") ||
                !parse_u8_token(line, "t2ch", cp.via1.t2ch, "io.via1.t2ch") ||
                !parse_u8_token(line, "sr", cp.via1.sr, "io.via1.sr") ||
                !parse_u8_token(line, "acr", cp.via1.acr, "io.via1.acr") ||
                !parse_u8_token(line, "pcr", cp.via1.pcr, "io.via1.pcr") ||
                !parse_u8_token(line, "ifr", cp.via1.ifr, "io.via1.ifr") ||
                !parse_u8_token(line, "ier", cp.via1.ier, "io.via1.ier") ||
                !parse_bool_token(line, "t1_running", cp.via1.t1_running,
                                  "io.via1.t1_running") ||
                !parse_u16_token(line, "t1_latch", cp.via1.t1_latch,
                                  "io.via1.t1_latch") ||
                !parse_u16_token(line, "t1_counter", cp.via1.t1_counter,
                                  "io.via1.t1_counter") ||
                !parse_bool_token(line, "t2_running", cp.via1.t2_running,
                                  "io.via1.t2_running") ||
                !parse_u16_token(line, "t2_latch", cp.via1.t2_latch,
                                  "io.via1.t2_latch") ||
                !parse_u16_token(line, "t2_counter", cp.via1.t2_counter,
                                  "io.via1.t2_counter") ||
                !parse_bool_token(line, "adb_shift_in_progress",
                                  cp.via1.adb_shift_in_progress,
                                  "io.via1.adb_shift_in_progress") ||
                !parse_u64_token(line, "adb_shift_complete_cyc",
                                 cp.via1.adb_shift_complete_cyc,
                                 "io.via1.adb_shift_complete_cyc") ||
                !parse_u8_token(line, "adb_last_byte", cp.via1.adb_last_byte,
                                "io.via1.adb_last_byte") ||
                !parse_u32_token(line, "adb_transaction_id",
                                 cp.via1.adb_transaction_id,
                                 "io.via1.adb_transaction_id") ||
                !parse_u32_token(line, "rtc_data_line", rtc_line,
                                 "io.via1.rtc_data_line") ||
                !parse_u64_token(line, "via1_timer_div",
                                 cp.via1_timer_div_snapshot,
                                 "io.via1.via1_timer_div") ||
                !parse_u64_token(line, "via1_timer_div_ctr",
                                 cp.via1_timer_div_ctr_snapshot,
                                 "io.via1.via1_timer_div_ctr") ||
                rtc_line > 1u) {
                std::fprintf(stderr,
                    "[rom-boot] arch replay bad io via1 line at %llu\n",
                    (unsigned long long)line_no);
                return false;
            }
            cp.via1_rtc_data_line = rtc_line;
            cp.has_io_via1 = true;
            continue;
        }
        if (line.rfind("io q700_descriptor ", 0) == 0) {
            if (!parse_u32_token(line, "selected",
                                 cp.q700_descriptor_selected_snapshot,
                                 "io.q700_descriptor.selected") ||
                cp.q700_descriptor_selected_snapshot > 1u ||
                !parse_u32_token(line, "entry",
                                 cp.q700_descriptor_entry_snapshot,
                                 "io.q700_descriptor.entry") ||
                !parse_u32_token(line, "feature",
                                 cp.q700_feature_word_snapshot,
                                 "io.q700_descriptor.feature")) {
                std::fprintf(stderr,
                    "[rom-boot] arch replay bad io q700_descriptor line at %llu\n",
                    (unsigned long long)line_no);
                return false;
            }
            cp.has_io_q700_descriptor = true;
            continue;
        }
        if (line.rfind("segment ", 0) == 0) {
            std::string v;
            seg = ReplaySegment{};
            if (!token_value(line, "name", seg.name)) return false;
            if (!token_value(line, "base", v) ||
                !parse_u32_field(v, seg.base, "segment.base")) return false;
            if (!token_value(line, "size", v) ||
                !parse_u64_field(v, seg.size, "segment.size")) return false;
            if (!token_value(line, "encoding", seg.encoding)) return false;
            if (token_value(line, "default", v)) {
                uint32_t d = 0;
                if (!parse_u32_field(v, d, "segment.default") || d > 0xff)
                    return false;
                seg.default_value = (uint8_t)d;
            }
            seg.active = true;
            if (!replay_segment_target(seg, seg_target, seg_target_size)) {
                std::fprintf(stderr,
                    "[rom-boot] arch replay bad segment at line %llu\n",
                    (unsigned long long)line_no);
                return false;
            }
            if (seg.name == "q700-rom") cp.saw_rom = true;
            if (seg.name == "ram") cp.saw_ram = true;
            if (seg.name == "vram") cp.saw_vram = true;
            if (seg.name == "magic") cp.saw_magic = true;
            continue;
        }
        if (line.rfind("endsegment ", 0) == 0) {
            seg = ReplaySegment{};
            seg_target = nullptr;
            seg_target_size = 0;
            continue;
        }
        if (line.rfind("data ", 0) == 0) {
            if (!seg.active || !seg_target) return false;
            std::string v;
            uint64_t off = 0;
            uint32_t bytes = 0;
            if (!token_value(line, "off", v) ||
                !parse_u64_field(v, off, "data.off")) return false;
            if (!token_value(line, "bytes", v) ||
                !parse_u32_field(v, bytes, "data.bytes")) return false;
            if (!token_value(line, "hex", v)) return false;
            if (v.size() != (size_t)bytes * 2 || off + bytes > seg_target_size)
                return false;
            for (uint32_t i = 0; i < bytes; i++) {
                uint8_t b = 0;
                if (!parse_hex_byte_pair(v[i * 2], v[i * 2 + 1], b))
                    return false;
                seg_target[(size_t)off + i] = b;
            }
            continue;
        }
    }

    const bool close_ok = std::fclose(fp) == 0;
    if (!close_ok) return false;
    bool regs_ok = true;
    for (bool b : cp.has_reg) regs_ok = regs_ok && b;
    if (!cp.has_format || !cp.has_run || !cp.has_overlay ||
        !cp.has_flush || !cp.has_quiesce || !cp.has_arch || !cp.has_pc ||
        !cp.has_control || !cp.has_mmu || !cp.has_replay ||
        !regs_ok || !cp.saw_memory_begin ||
        !cp.saw_memory_end || !cp.saw_end_format || !cp.saw_rom ||
        !cp.saw_ram || !cp.saw_vram || !cp.saw_magic) {
        std::fprintf(stderr,
            "[rom-boot] arch replay checkpoint missing required fields "
            "(format=%u run=%u overlay=%u flush=%u quiesce=%u "
            "arch=%u pc=%u control=%u mmu=%u replay=%u regs=%u memory_begin=%u "
            "memory_end=%u end_format=%u rom=%u ram=%u vram=%u magic=%u)\n",
            cp.has_format ? 1u : 0u, cp.has_run ? 1u : 0u,
            cp.has_overlay ? 1u : 0u, cp.has_flush ? 1u : 0u,
            cp.has_quiesce ? 1u : 0u, cp.has_arch ? 1u : 0u,
            cp.has_pc ? 1u : 0u, cp.has_control ? 1u : 0u,
            cp.has_mmu ? 1u : 0u, cp.has_replay ? 1u : 0u,
            regs_ok ? 1u : 0u,
            cp.saw_memory_begin ? 1u : 0u,
            cp.saw_memory_end ? 1u : 0u,
            cp.saw_end_format ? 1u : 0u,
            cp.saw_rom ? 1u : 0u, cp.saw_ram ? 1u : 0u,
            cp.saw_vram ? 1u : 0u, cp.saw_magic ? 1u : 0u);
        return false;
    }
    if (cp.flush_attempted != 1 || cp.flush_completed != 1) {
        std::fprintf(stderr,
            "[rom-boot] arch replay unsafe checkpoint: flush attempted=%u "
            "completed=%u expected 1/1\n",
            cp.flush_attempted, cp.flush_completed);
        return false;
    }
    if (cp.quiesce_rob_empty != 1 || cp.quiesce_int_iq_count != 0 ||
        cp.quiesce_mem_iq_count != 0 || cp.quiesce_lsu_busy != 0 ||
        cp.quiesce_exc_active != 0 || cp.quiesce_cache_maint_wait != 0 ||
        cp.quiesce_dcache_flush_busy != 0 ||
        cp.quiesce_flush_queue_empty != 1) {
        std::fprintf(stderr,
            "[rom-boot] arch replay unsafe checkpoint: quiesce "
            "rob_empty=%u int_iq_count=%u mem_iq_count=%u lsu_busy=%u "
            "exc_active=%u cache_maint_wait=%u dcache_flush_busy=%u "
            "flush_queue_empty=%u\n",
            cp.quiesce_rob_empty, cp.quiesce_int_iq_count,
            cp.quiesce_mem_iq_count, cp.quiesce_lsu_busy,
            cp.quiesce_exc_active, cp.quiesce_cache_maint_wait,
            cp.quiesce_dcache_flush_busy, cp.quiesce_flush_queue_empty);
        return false;
    }
    const bool replay_v1_reset =
        cp.replay_supported == "debug_arch_load_v1" &&
        cp.replay_io_state == "reset";
    const bool replay_v2_via1 =
        cp.replay_supported == "debug_arch_load_v2" &&
        cp.replay_io_state == "via1-v1";
    if ((!replay_v1_reset && !replay_v2_via1) ||
        cp.replay_caches != "reset" ||
        cp.replay_queues != "reset" ||
        cp.replay_predictors != "reset") {
        std::fprintf(stderr,
            "[rom-boot] arch replay unsupported checkpoint contract: "
            "supported=%s io_state=%s caches=%s queues=%s predictors=%s\n",
            cp.replay_supported.c_str(), cp.replay_io_state.c_str(),
            cp.replay_caches.c_str(), cp.replay_queues.c_str(),
            cp.replay_predictors.c_str());
        return false;
    }
    if (replay_v2_via1 && !cp.has_io_via1) {
        std::fprintf(stderr,
            "[rom-boot] arch replay checkpoint declares io_state=via1-v1 "
            "but has no io via1 line\n");
        return false;
    }
    if (cp.pc == 0x00000000u || cp.pc == 0xffffffffu) {
        std::fprintf(stderr,
            "[rom-boot] arch replay unsafe checkpoint: pc.next=0x%08x\n",
            cp.pc);
        return false;
    }
    return true;
}

static void clear_arch_debug_load_ports() {
    if (!dut) return;
    dut->dbg_arch_apply_en = 0;
    dut->dbg_arch_reg_load_en = 0;
    dut->dbg_arch_reg_load_idx = 0;
    dut->dbg_arch_reg_load_val = 0;
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
}

static void clear_core_debug_stop_ports() {
    if (!dut) return;
    dut->dbg_precise_stop_req = 0;
    dut->dbg_precise_stop_keep_tag = 0;
    dut->dbg_core_halt = 0;
}

static void pulse_arch_reg_load(uint32_t idx, uint32_t value) {
    clear_arch_debug_load_ports();
    dut->dbg_arch_reg_load_en = 1;
    dut->dbg_arch_reg_load_idx = idx;
    dut->dbg_arch_reg_load_val = value;
    tick();
    clear_arch_debug_load_ports();
    dut->eval();
}

static void pulse_arch_ccr_load(uint32_t value) {
    clear_arch_debug_load_ports();
    dut->dbg_arch_ccr_load_en = 1;
    dut->dbg_arch_ccr_load_val = value & 0x1f;
    tick();
    clear_arch_debug_load_ports();
    dut->eval();
}

static void pulse_arch_ctrl_load(uint32_t sel, uint32_t value) {
    clear_arch_debug_load_ports();
    dut->dbg_arch_ctrl_load_en = 1;
    dut->dbg_arch_ctrl_load_sel = sel;
    dut->dbg_arch_ctrl_load_val = value;
    tick();
    clear_arch_debug_load_ports();
    dut->eval();
}

static void pulse_arch_mmu_load(uint32_t sel, uint32_t value) {
    clear_arch_debug_load_ports();
    dut->dbg_arch_mmu_load_en = 1;
    dut->dbg_arch_mmu_load_sel = sel;
    dut->dbg_arch_mmu_load_val = value;
    tick();
    clear_arch_debug_load_ports();
    dut->eval();
}

static void pulse_arch_pc_load(uint32_t pc) {
    clear_arch_debug_load_ports();
    dut->dbg_arch_pc_load_en = 1;
    dut->dbg_arch_pc_load_val = pc;
    tick();
    clear_arch_debug_load_ports();
    dut->eval();
}

static void reset_replay_io_state() {
    overlay = true;
    via1 = Via1{};
    via2 = Via2{};
    asc = AscStub{};
    swim = SwimStub{};
    turboscsi_trace.reset();
    rtc_sidechannel.reset(0);
    unmapped_write_count.clear();
    unmapped_read_count.clear();
    if_pending = 0;
    if_addr_q = 0;
    ax = Axi{};
    last_logged_pc = 0;
    last_same_pc_count = 0;
    q700_descriptor_selected = false;
    q700_descriptor_entry = 0;
    q700_feature_word = 0;
    arch_next_pc_valid = false;
    arch_next_pc = 0;
    arch_next_pc_commit_pc = 0;
    arch_next_pc_committed = 0;
}

static bool apply_arch_replay(const std::string& path) {
    ArchReplayCheckpoint cp;
    if (!parse_arch_replay_checkpoint(path, cp)) return false;
    if (!active_rom_patches.empty()) {
        std::fprintf(stderr,
            "[rom-boot] reapplying harness ROM patches after arch replay load\n");
        if (!apply_active_rom_patches()) return false;
    }

    reset_replay_io_state();
    if (cp.has_io_via1) {
        via1 = cp.via1;
        via1_timer_div = cp.via1_timer_div_snapshot ?
            cp.via1_timer_div_snapshot : 1;
        via1_timer_div_ctr = cp.via1_timer_div_ctr_snapshot;
        rtc_sidechannel.data_line = cp.via1_rtc_data_line != 0;
    }
    if (cp.has_io_q700_descriptor) {
        q700_descriptor_selected =
            cp.q700_descriptor_selected_snapshot != 0;
        q700_descriptor_entry = cp.q700_descriptor_entry_snapshot;
        q700_feature_word = cp.q700_feature_word_snapshot;
    }
    if (cp.has_visible_ram)
        visible_ram_size = cp.visible_ram;
    overlay = cp.overlay != 0;
    pulse_arch_pc_load(cp.pc);

    for (uint32_t i = 0; i < 16; i++)
        pulse_arch_reg_load(i, cp.regs[i]);
    pulse_arch_reg_load(16, 0);
    pulse_arch_reg_load(17, 0);
    pulse_arch_reg_load(18, 0);
    pulse_arch_ccr_load(cp.ccr);

    pulse_arch_ctrl_load(0, cp.sr);
    pulse_arch_ctrl_load(1, cp.vbr);
    pulse_arch_ctrl_load(2, cp.cacr);
    pulse_arch_ctrl_load(3, cp.sfc);
    pulse_arch_ctrl_load(4, cp.dfc);
    pulse_arch_ctrl_load(5, cp.usp);
    pulse_arch_ctrl_load(6, cp.ssp);
    pulse_arch_ctrl_load(7, cp.isp);
    pulse_arch_ctrl_load(8, cp.committed);
    pulse_arch_ctrl_load(9, cp.commit_pc);

    pulse_arch_mmu_load(0, cp.itt0);
    pulse_arch_mmu_load(1, cp.itt1);
    pulse_arch_mmu_load(2, cp.dtt0);
    pulse_arch_mmu_load(3, cp.dtt1);
    pulse_arch_mmu_load(4, cp.tc);
    pulse_arch_mmu_load(5, cp.urp);
    pulse_arch_mmu_load(6, cp.srp);

    pulse_arch_pc_load(cp.pc);

    sim_time = cp.cycle;
    host_cycle = sim_time;
    test_done = false;
    stop_reason = "arch-replay-restored";
    arch_next_pc_valid = true;
    arch_next_pc = cp.pc;
    arch_next_pc_commit_pc = cp.commit_pc;
    arch_next_pc_committed = cp.committed;

    if (cp.has_io_via1) {
        std::fprintf(stderr,
            "[rom-boot] arch replay restored partial IO state "
            "(VIA1/RTC-line/q700-descriptor); other peripheral state reset "
            "to defaults "
            "(SCC/SCSI/ASC/SWIM/DAFB, AXI handshakes, caches, queues, "
            "predictors)\n");
    } else {
        std::fprintf(stderr,
            "[rom-boot] arch replay IO/peripheral state reset to defaults "
            "(VIA/ADB/RTC/SCSI/SCC/ASC/SWIM/DAFB, AXI handshakes, caches, "
            "queues, predictors)\n");
    }
    std::fprintf(stderr,
        "[rom-boot] arch replay restored: %s (committed=%u pc=0x%08x "
        "io_state=%s io_restore=%s overlay=%u)\n",
        path.c_str(), cp.committed, cp.pc,
        cp.has_io_via1 ? "via1-v1" : "reset",
        cp.replay_io_restore.empty() ? "unspecified" :
            cp.replay_io_restore.c_str(),
        cp.overlay);
    return true;
}

static uint64_t parse_uint64_auto(const std::string& tok,
                                  const char* arg_name) {
    size_t consumed = 0;
    uint64_t value = 0;
    try {
        value = std::stoull(tok, &consumed, 0);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[rom-boot] bad %s value '%s': %s\n",
                     arg_name, tok.c_str(), e.what());
        std::exit(2);
    }
    if (consumed != tok.size()) {
        std::fprintf(stderr, "[rom-boot] bad %s value '%s'\n",
                     arg_name, tok.c_str());
        std::exit(2);
    }
    return value;
}

static uint32_t parse_byte_size_arg(const std::string& tok,
                                    const char* arg_name) {
    if (tok.empty()) {
        std::fprintf(stderr, "[rom-boot] bad %s value: empty\n", arg_name);
        std::exit(2);
    }

    std::string number = tok;
    uint64_t scale = 1;
    const char suffix = number.back();
    if (suffix == 'k' || suffix == 'K' ||
        suffix == 'm' || suffix == 'M') {
        number.pop_back();
        scale = (suffix == 'k' || suffix == 'K') ? 1024ULL
                                                 : 1024ULL * 1024ULL;
    }

    const uint64_t n = parse_uint64_auto(number, arg_name);
    if (n == 0 || n > (UINT64_MAX / scale)) {
        std::fprintf(stderr, "[rom-boot] bad %s value '%s'\n",
                     arg_name, tok.c_str());
        std::exit(2);
    }
    const uint64_t bytes = n * scale;
    if (bytes > MemModel::RAM_BACKING_SIZE || bytes < MemModel::RAM_WINDOW_MIN ||
        (bytes & 0x3u) != 0 || (bytes & (bytes - 1u)) != 0) {
        std::fprintf(stderr,
            "[rom-boot] %s must be power-of-two, 32-bit aligned, in [0x%08x..0x%08x]: '%s'\n",
            arg_name, MemModel::RAM_WINDOW_MIN, MemModel::RAM_BACKING_SIZE,
            tok.c_str());
        std::exit(2);
    }
    return (uint32_t)bytes;
}

static void parse_uint64_list(const std::string& spec,
                              std::vector<uint64_t>& out) {
    std::stringstream ss(spec);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        if (!tok.empty()) out.push_back(std::stoull(tok));
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
}

static void parse_pc_breakpoint_list(const std::string& spec,
                                     const char* arg_name,
                                     std::vector<PcBreakpoint>& out) {
    std::stringstream ss(spec);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        if (tok.empty()) continue;
        uint64_t pc64 = parse_uint64_auto(tok, arg_name);
        if (pc64 > 0xffffffffULL) {
            std::fprintf(stderr,
                "[rom-boot] %s value out of 32-bit range: '%s'\n",
                arg_name, tok.c_str());
            std::exit(2);
        }
        PcBreakpoint bp;
        bp.pc = (uint32_t)pc64;
        out.push_back(bp);
    }
    std::sort(out.begin(), out.end(),
              [](const PcBreakpoint& a, const PcBreakpoint& b) {
                  return a.pc < b.pc;
              });
    out.erase(std::unique(out.begin(), out.end(),
                          [](const PcBreakpoint& a, const PcBreakpoint& b) {
                              return a.pc == b.pc;
                          }),
              out.end());
}

static void parse_stop_pc_list(const std::string& spec) {
    parse_pc_breakpoint_list(spec, "+stop_pc", stop_pcs);
}

static void parse_end_pc_list(const std::string& spec) {
    parse_pc_breakpoint_list(spec, "+end_pc", end_pcs);
}

static void add_exc_vec(std::vector<uint8_t>& vecs, uint8_t vec) {
    if (std::find(vecs.begin(), vecs.end(), vec) == vecs.end()) {
        vecs.push_back(vec);
        std::sort(vecs.begin(), vecs.end());
    }
}

static void parse_exc_list(const std::string& spec,
                           const char* arg_name,
                           std::vector<uint8_t>& out) {
    std::stringstream ss(spec);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        if (tok.empty()) continue;
        uint64_t vec64 = parse_uint64_auto(tok, arg_name);
        if (vec64 == 0 || vec64 > 255) {
            std::fprintf(stderr,
                "[rom-boot] %s vector out of range: '%s' "
                "(valid runtime vectors are 1..255)\n",
                arg_name,
                tok.c_str());
            std::exit(2);
        }
        add_exc_vec(out, (uint8_t)vec64);
    }
}

static void parse_stop_exc_list(const std::string& spec) {
    parse_exc_list(spec, "+stop_on_exc", stop_exc_vecs);
}

static void parse_end_exc_list(const std::string& spec) {
    parse_exc_list(spec, "+end_on_exc", end_exc_vecs);
}

static void enable_rom_fault_vecs(std::vector<uint8_t>& vecs) {
    // Common ROM-frontier dead ends: bus error, illegal instruction, F-line.
    // A-line traps are deliberately not included because classic Mac ROMs use
    // them for OS/toolbox dispatch once boot gets farther.
    add_exc_vec(vecs, 2);
    add_exc_vec(vecs, 4);
    add_exc_vec(vecs, 11);
}

static void enable_rom_fault_stops() {
    enable_rom_fault_vecs(stop_exc_vecs);
}

static void enable_rom_fault_enders() {
    enable_rom_fault_vecs(end_exc_vecs);
}

static void enable_rom_outcome_stops() {
    stop_on_sad_mac = true;
    stop_on_disk_prompt = true;
}

static void enable_illegal_stop() {
    add_exc_vec(stop_exc_vecs, 4);
}

static void enable_illegal_ender() {
    add_exc_vec(end_exc_vecs, 4);
}

static void parse_rom_patch_list(const std::string& spec) {
    std::stringstream ss(spec);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        if (!tok.empty()) requested_rom_patch_sets.push_back(tok);
    }
}

static bool check_pc_breakpoints(uint32_t pc,
                                 std::vector<PcBreakpoint>& breakpoints,
                                 uint64_t hit_target,
                                 const char* reason_prefix,
                                 char* reason_buf,
                                 size_t reason_buf_len) {
    for (auto& bp : breakpoints) {
        if (bp.pc != pc) continue;
        bp.hits++;
        if (bp.hits >= hit_target) {
            std::snprintf(reason_buf, reason_buf_len,
                          "%s pc=0x%08x hit=%llu",
                          reason_prefix, bp.pc,
                          (unsigned long long)bp.hits);
            stop_reason = reason_buf;
            test_done = true;
            return true;
        }
    }
    return false;
}

static bool check_stop_pc(uint32_t pc) {
    return check_pc_breakpoints(pc, stop_pcs, stop_pc_hit_target,
                                "stop-pc", stop_pc_reason,
                                sizeof(stop_pc_reason));
}

static bool check_end_pc(uint32_t pc) {
    return check_pc_breakpoints(pc, end_pcs, end_pc_hit_target,
                                "end-breakpoint", end_pc_reason,
                                sizeof(end_pc_reason));
}

static bool check_rom_outcome_commit(uint32_t pc) {
    if (stop_on_sad_mac && pc == SAD_MAC_PC) {
        const uint32_t d6 = dut ? read_arch_reg(6) : 0;
        const uint32_t d7 = dut ? read_arch_reg(7) : 0;
        std::snprintf(rom_outcome_reason, sizeof(rom_outcome_reason),
                      "sad-mac pc=0x%08x code=0x%08x d7=0x%08x",
                      pc, d6, d7);
        stop_reason = rom_outcome_reason;
        test_done = true;
        return true;
    }

    if (stop_on_disk_prompt && pc == DISK_PROMPT_PC) {
        disk_prompt_hits++;
        if (disk_prompt_hits >= disk_prompt_hit_target) {
            const uint32_t d0 = dut ? read_arch_reg(0) : 0;
            const uint32_t d1 = dut ? read_arch_reg(1) : 0;
            const uint32_t d3 = dut ? read_arch_reg(3) : 0;
            const uint32_t d4 = dut ? read_arch_reg(4) : 0;
            std::snprintf(rom_outcome_reason, sizeof(rom_outcome_reason),
                          "disk-prompt pc=0x%08x hit=%llu d0=0x%08x "
                          "d1=0x%08x d3=0x%08x d4=0x%08x",
                          pc,
                          (unsigned long long)disk_prompt_hits,
                          d0, d1, d3, d4);
            stop_reason = rom_outcome_reason;
            test_done = true;
            return true;
        }
    }

    return false;
}

static bool check_stuck_pc_commit(uint32_t pc) {
    if (pc == last_logged_pc) {
        last_same_pc_count++;
        if (stuck_pc_threshold != 0 &&
            (uint64_t)last_same_pc_count >= stuck_pc_threshold) {
            if (end_on_stuck_pc) {
                std::snprintf(end_stuck_pc_reason, sizeof(end_stuck_pc_reason),
                              "end-stuck-pc pc=0x%08x repeats=%u threshold=%llu",
                              pc,
                              last_same_pc_count,
                              (unsigned long long)stuck_pc_threshold);
                stop_reason = end_stuck_pc_reason;
            } else {
                std::snprintf(stuck_pc_reason, sizeof(stuck_pc_reason),
                              "stuck-pc pc=0x%08x repeats=%u threshold=%llu",
                              pc,
                              last_same_pc_count,
                              (unsigned long long)stuck_pc_threshold);
                stop_reason = stuck_pc_reason;
            }
            test_done = true;
            return true;
        }
    } else {
        last_same_pc_count = 0;
        last_logged_pc = pc;
    }
    return false;
}

static bool check_no_progress_watchdog(uint64_t no_progress_cycles) {
    if (no_progress_cycle_threshold == 0 ||
        no_progress_cycles < no_progress_cycle_threshold) {
        return false;
    }
    std::snprintf(no_progress_reason, sizeof(no_progress_reason),
                  "%s cycles=%llu threshold=%llu",
                  end_on_no_progress ? "end-no-progress" : "no-progress",
                  (unsigned long long)no_progress_cycles,
                  (unsigned long long)no_progress_cycle_threshold);
    stop_reason = no_progress_reason;
    test_done = true;
    return true;
}

static bool run_rom_outcome_selftest() {
    const bool saved_stop_sad = stop_on_sad_mac;
    const bool saved_stop_disk = stop_on_disk_prompt;
    const uint64_t saved_disk_target = disk_prompt_hit_target;
    const uint64_t saved_disk_hits = disk_prompt_hits;
    const bool saved_test_done = test_done;
    const char* saved_stop_reason = stop_reason;

    stop_on_sad_mac = true;
    stop_on_disk_prompt = false;
    disk_prompt_hits = 0;
    test_done = false;
    stop_reason = "rom-outcome-selftest";
    const bool sad_stopped = check_rom_outcome_commit(SAD_MAC_PC);
    const bool sad_reason =
        std::string(stop_reason).find("sad-mac pc=0x40849afa") !=
        std::string::npos;

    stop_on_sad_mac = false;
    stop_on_disk_prompt = true;
    disk_prompt_hit_target = 2;
    disk_prompt_hits = 0;
    test_done = false;
    stop_reason = "rom-outcome-selftest";
    const bool disk_first = !check_rom_outcome_commit(DISK_PROMPT_PC);
    const bool disk_second = check_rom_outcome_commit(DISK_PROMPT_PC);
    const bool disk_reason =
        std::string(stop_reason).find("disk-prompt pc=0x40898e3e hit=2") !=
        std::string::npos;

    const bool ok = sad_stopped && sad_reason && disk_first &&
                    disk_second && disk_reason;
    std::fprintf(stderr,
        "[rom-boot] rom_outcome_selftest: sad=%s disk=%s reason=\"%s\"\n",
        (sad_stopped && sad_reason) ? "PASS" : "FAIL",
        (disk_first && disk_second && disk_reason) ? "PASS" : "FAIL",
        stop_reason);

    stop_on_sad_mac = saved_stop_sad;
    stop_on_disk_prompt = saved_stop_disk;
    disk_prompt_hit_target = saved_disk_target;
    disk_prompt_hits = saved_disk_hits;
    test_done = saved_test_done;
    stop_reason = saved_stop_reason;
    return ok;
}

static bool run_watchdog_selftest() {
    const uint64_t saved_stuck_threshold = stuck_pc_threshold;
    const uint64_t saved_no_progress_threshold = no_progress_cycle_threshold;
    const bool saved_end_stuck = end_on_stuck_pc;
    const bool saved_end_progress = end_on_no_progress;
    const char* saved_stop_reason = stop_reason;
    const uint32_t saved_last_pc = last_logged_pc;
    const uint32_t saved_same_pc_count = last_same_pc_count;
    const bool saved_test_done = test_done;

    stuck_pc_threshold = 2;
    no_progress_cycle_threshold = 3;
    end_on_stuck_pc = false;
    end_on_no_progress = false;
    stop_reason = "watchdog-selftest-start";
    test_done = false;
    last_logged_pc = 0;
    last_same_pc_count = 0;

    const bool pc0 = !check_stuck_pc_commit(0x4000002au);
    const bool pc1 = !check_stuck_pc_commit(0x4000002au);
    const bool pc2 = check_stuck_pc_commit(0x4000002au);
    const bool pc_reason =
        std::string(stop_reason).find("stuck-pc pc=0x4000002a repeats=2 threshold=2") !=
        std::string::npos;

    stop_reason = "watchdog-selftest-progress";
    test_done = false;
    uint64_t no_progress = 0;
    bool np0 = false;
    bool np1 = false;
    bool np2 = false;
    for (int i = 0; i < 3; i++) {
        no_progress++;
        bool stopped = check_no_progress_watchdog(no_progress);
        if (i == 0) np0 = !stopped;
        if (i == 1) np1 = !stopped;
        if (i == 2) np2 = stopped;
    }
    const bool np_reason =
        std::string(stop_reason).find("no-progress cycles=3 threshold=3") !=
        std::string::npos;

    const bool ok = pc0 && pc1 && pc2 && pc_reason && np0 && np1 && np2 &&
                    np_reason;
    std::fprintf(stderr,
        "[rom-boot] watchdog_selftest: stuck_pc=%s no_progress=%s\n",
        (pc0 && pc1 && pc2 && pc_reason) ? "PASS" : "FAIL",
        (np0 && np1 && np2 && np_reason) ? "PASS" : "FAIL");

    stuck_pc_threshold = saved_stuck_threshold;
    no_progress_cycle_threshold = saved_no_progress_threshold;
    end_on_stuck_pc = saved_end_stuck;
    end_on_no_progress = saved_end_progress;
    stop_reason = saved_stop_reason;
    last_logged_pc = saved_last_pc;
    last_same_pc_count = saved_same_pc_count;
    test_done = saved_test_done;
    return ok;
}

static bool exc_vec_configured(const std::vector<uint8_t>& vecs, uint8_t vec) {
    if (vec == 0) return false;
    return std::find(vecs.begin(), vecs.end(), vec) != vecs.end();
}

static const char* rom_exception_class(uint8_t vec) {
    switch (vec) {
        case 2:  return "bus-error";
        case 4:  return "illegal-instruction";
        case 11: return "f-line";
        default: return "exception";
    }
}

static BoundaryEvent read_boundary_event() {
    BoundaryEvent ev;
    if (!dut) return ev;
    ev.seq = (uint32_t)dut->dbg_boundary_seq;
    ev.kind = (uint32_t)dut->dbg_boundary_kind;
    ev.pc = (uint32_t)dut->dbg_boundary_pc;
    ev.next_pc = (uint32_t)dut->dbg_boundary_next_pc;
    ev.vec = (uint8_t)dut->dbg_boundary_exc_vec;
    ev.fault_pc = (uint32_t)dut->dbg_boundary_fault_pc;
    ev.fault_addr = (uint32_t)dut->dbg_boundary_fault_addr;
    ev.keep_tag = (uint32_t)dut->dbg_boundary_keep_tag;
    return ev;
}

static bool poll_new_boundary_event(uint32_t& prev_seq, BoundaryEvent& ev) {
    if (!dut) return false;
    ev = read_boundary_event();
    if (ev.seq == 0 || ev.seq == prev_seq || ev.kind == DBG_BOUNDARY_NONE)
        return false;
    prev_seq = ev.seq;
    return true;
}

static void arm_precise_boundary_stop(const BoundaryEvent& ev) {
    precise_boundary_pending = true;
    precise_boundary_stop = ev;
}

static bool check_exception_boundary(const BoundaryEvent& ev,
                                     const std::vector<uint8_t>& vecs,
                                     uint64_t min_committed,
                                     const char* reason_prefix,
                                     char* reason_buf,
                                     size_t reason_buf_len) {
    if (vecs.empty() || ev.kind != DBG_BOUNDARY_EXC)
        return false;
    if (!exc_vec_configured(vecs, ev.vec))
        return false;
    const uint64_t committed = dut ? (uint64_t)dut->dbg_committed : 0u;
    if (committed < min_committed)
        return false;
    std::snprintf(reason_buf, reason_buf_len,
        "%s class=%s source=boundary vec=%u handler_pc=0x%08x "
        "fault_pc=0x%08x fault_addr=0x%08x committed=%llu boundary_seq=%u",
        reason_prefix,
        rom_exception_class(ev.vec),
        (unsigned)ev.vec,
        (unsigned)ev.next_pc,
        (unsigned)ev.fault_pc,
        (unsigned)ev.fault_addr,
        (unsigned long long)committed,
        ev.seq);
    stop_reason = reason_buf;
    test_done = true;
    arm_precise_boundary_stop(ev);
    return true;
}

static bool check_stop_on_exception_boundary(const BoundaryEvent& ev) {
    return check_exception_boundary(ev, stop_exc_vecs,
                                    stop_on_exc_after_committed, "stop-exc",
                                    stop_exc_reason, sizeof(stop_exc_reason));
}

static bool check_end_on_exception_boundary(const BoundaryEvent& ev) {
    return check_exception_boundary(ev, end_exc_vecs,
                                    end_on_exc_after_committed, "end-exc",
                                    end_exc_reason, sizeof(end_exc_reason));
}

static bool run_stop_on_exc_selftest() {
    const uint64_t saved_stop_after = stop_on_exc_after_committed;
    const std::vector<uint8_t> saved_stop_exc_vecs = stop_exc_vecs;
    test_done = false;
    stop_reason = "stop-on-exc-selftest";
    precise_boundary_pending = false;
    precise_boundary_stop = BoundaryEvent{};
    stop_exc_vecs.clear();
    stop_exc_vecs.push_back(4);
    BoundaryEvent ev;
    ev.seq = 1;
    ev.kind = DBG_BOUNDARY_EXC;
    ev.pc = 0x40000100u;
    ev.next_pc = 0x40001234u;
    ev.vec = 4;
    ev.fault_pc = 0x40005678u;
    ev.fault_addr = 0x50f01c00u;
    ev.keep_tag = 7;

    stop_on_exc_after_committed = 1;
    const bool suppressed = !check_stop_on_exception_boundary(ev) &&
                            !precise_boundary_pending &&
                            !test_done;
    stop_on_exc_after_committed = 0;
    const bool stopped = check_stop_on_exception_boundary(ev);
    const bool armed = precise_boundary_pending &&
                       precise_boundary_stop.keep_tag == ev.keep_tag &&
                       precise_boundary_stop.next_pc == ev.next_pc;
    const bool reason_ok = stop_reason &&
                           std::strstr(stop_reason, "source=boundary") &&
                           std::strstr(stop_reason, "vec=4");
    std::fprintf(stderr,
        "[rom-boot] stop_on_exc_selftest: threshold=%s stopped=%s armed=%s reason=\"%s\"\n",
        suppressed ? "PASS" : "FAIL",
        stopped ? "PASS" : "FAIL",
        armed ? "PASS" : "FAIL",
        stop_reason ? stop_reason : "(null)");
    stop_on_exc_after_committed = saved_stop_after;
    stop_exc_vecs = saved_stop_exc_vecs;
    return suppressed && stopped && armed && reason_ok;
}

static void print_pc_breakpoint_config(const char* label,
                                       const std::vector<PcBreakpoint>& breakpoints,
                                       uint64_t hit_target) {
    if (breakpoints.empty()) return;
    std::fprintf(stderr, "[rom-boot] %s:", label);
    for (const auto& bp : breakpoints) {
        std::fprintf(stderr, " 0x%08x", bp.pc);
    }
    std::fprintf(stderr, " hit=%llu\n",
                 (unsigned long long)hit_target);
}

static void print_stop_pc_config() {
    print_pc_breakpoint_config("stop_pc", stop_pcs, stop_pc_hit_target);
}

static void print_end_pc_config() {
    print_pc_breakpoint_config("end_pc", end_pcs, end_pc_hit_target);
}

static void print_rom_outcome_config() {
    if (!stop_on_sad_mac && !stop_on_disk_prompt) return;
    std::fprintf(stderr, "[rom-boot] rom_outcome:");
    if (stop_on_sad_mac)
        std::fprintf(stderr, " sad-mac=0x%08x", SAD_MAC_PC);
    if (stop_on_disk_prompt)
        std::fprintf(stderr, " disk-prompt=0x%08x hit=%llu",
                     DISK_PROMPT_PC,
                     (unsigned long long)disk_prompt_hit_target);
    std::fprintf(stderr, "\n");
}

static void print_stop_exc_config() {
    if (stop_exc_vecs.empty()) return;
    std::fprintf(stderr, "[rom-boot] stop_on_exc:");
    for (uint8_t vec : stop_exc_vecs) {
        std::fprintf(stderr, " %u", (unsigned)vec);
    }
    std::fprintf(stderr,
                 " (2=%s 4=%s 11=%s)",
                 rom_exception_class(2),
                 rom_exception_class(4),
                 rom_exception_class(11));
    if (stop_on_exc_after_committed != 0) {
        std::fprintf(stderr, " after_committed=%llu",
                     (unsigned long long)stop_on_exc_after_committed);
    }
    std::fprintf(stderr, "\n");
}

static void print_watchdog_config() {
    std::fprintf(stderr,
        "[rom-boot] watchdogs: stuck_pc_threshold=%llu no_progress_cycles=%llu\n",
        (unsigned long long)stuck_pc_threshold,
        (unsigned long long)no_progress_cycle_threshold);
}

static void print_end_exc_config() {
    if (end_exc_vecs.empty()) return;
    std::fprintf(stderr, "[rom-boot] end_on_exc:");
    for (uint8_t vec : end_exc_vecs) {
        std::fprintf(stderr, " %u", (unsigned)vec);
    }
    if (end_on_exc_after_committed != 0) {
        std::fprintf(stderr, " after_committed=%llu",
                     (unsigned long long)end_on_exc_after_committed);
    }
    std::fprintf(stderr, "\n");
}

static void print_fault_observability_config() {
    if (stop_on_ifetch_berr)
        std::fprintf(stderr, "[rom-boot] stop_on_ifetch_berr: enabled\n");
    if (end_on_ifetch_berr)
        std::fprintf(stderr, "[rom-boot] end_on_ifetch_berr: enabled\n");
    if (stuck_pc_threshold != DEFAULT_STUCK_PC_THRESHOLD || end_on_stuck_pc) {
        std::fprintf(stderr, "[rom-boot] stuck_pc_threshold: %llu%s\n",
                     (unsigned long long)stuck_pc_threshold,
                     end_on_stuck_pc ? " end-on-hit" : "");
    }
    if (no_progress_cycle_threshold != DEFAULT_NO_PROGRESS_CYCLES ||
        end_on_no_progress) {
        std::fprintf(stderr, "[rom-boot] no_progress_cycles: %llu%s\n",
                     (unsigned long long)no_progress_cycle_threshold,
                     end_on_no_progress ? " end-on-hit" : "");
    }
    if (!fault_dump_dir.empty())
        std::fprintf(stderr, "[rom-boot] fault_dump_dir: %s bytes=%llu\n",
                     fault_dump_dir.c_str(),
                     (unsigned long long)fault_dump_bytes);
}

static void init_lastn_trace() {
    if (lastn_trace_cycles == 0) return;
    lastn_trace_ring.assign((size_t)lastn_trace_cycles, LastNTraceSample{});
    lastn_trace_next = 0;
    lastn_trace_count = 0;
}

static void sample_lastn_trace() {
    if (lastn_trace_cycles == 0 || lastn_trace_ring.empty() || !dut) return;
    auto* r = dut->rootp;

    LastNTraceSample s;
    s.sim_time = sim_time;
    s.committed = (uint32_t)dut->dbg_committed;
    s.dbg_last_pc = (uint32_t)dut->dbg_last_pc;
    s.dbg_pc = (uint32_t)dut->dbg_pc;
    s.overlay_active = overlay ? 1u : 0u;
    s.via1_overlay_live = via1_overlay_active() ? 1u : 0u;

    s.rob_valid = (uint8_t)r->mac_top__DOT__cpu__DOT__rob_v;
    s.rob_complete = (uint8_t)r->mac_top__DOT__cpu__DOT__rob_c;
    s.rob_vec = (uint8_t)r->mac_top__DOT__cpu__DOT__rob_evec;
    s.rob_pc = (uint32_t)r->mac_top__DOT__cpu__DOT__rob_pc;

    s.commit_exc_wait =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_commit__DOT__exc_wait;
    s.commit_take_exc =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_commit__DOT__take_exc;

    s.exc_state =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_exception__DOT__state;
    s.exc_vec =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_exception__DOT__cur_vec;
    s.exc_fault_pc =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_exception__DOT__cur_fault_pc;
    s.exc_a7 =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_exception__DOT__cur_a7_new;

    const uint8_t arch_ccr =
        (uint8_t)(r->mac_top__DOT__cpu__DOT__u_ccr_rat__DOT__ccr_prf
            [r->mac_top__DOT__cpu__DOT__u_ccr_rat__DOT__crat_tag] & 0x1fu);
    s.arch_ccr = arch_ccr;
    s.arch_sr =
        (uint16_t)((r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_sr & 0xffe0u)
                   | arch_ccr);
    s.arch_vbr = (uint32_t)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_vbr;
    s.arch_usp = (uint32_t)r->mac_top__DOT__cpu__DOT__u_commit__DOT__usp;
    s.arch_ssp = (uint32_t)r->mac_top__DOT__cpu__DOT__u_commit__DOT__ssp;
    s.arch_isp = (uint32_t)r->mac_top__DOT__cpu__DOT__u_commit__DOT__isp;
    s.arch_cacr = (uint32_t)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_cacr;
    s.arch_sfc = (uint32_t)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_sfc;
    s.arch_dfc = (uint32_t)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_dfc;

    s.exc_gate_pending_exc =
        (uint8_t)r->mac_top__DOT__cpu__DOT__exc_gate_pending_exc;
    s.exc_gate_pending_rte =
        (uint8_t)r->mac_top__DOT__cpu__DOT__exc_gate_pending_rte;
    s.exc_flush_req = (uint8_t)r->mac_top__DOT__cpu__DOT__exc_flush_req;
    s.exc_flush_done =
        (uint8_t)(r->mac_top__DOT__cpu__DOT__dcache_flush_done_raw &&
                  r->mac_top__DOT__cpu__DOT__fq_valid[0] &&
                  ((r->mac_top__DOT__cpu__DOT__fq_src[0] & 0x3u) == 0));

    s.fq0_valid = (uint8_t)r->mac_top__DOT__cpu__DOT__fq_valid[0];
    s.fq0_src = (uint8_t)(r->mac_top__DOT__cpu__DOT__fq_src[0] & 0x3u);
    s.fq0_inval = (uint8_t)r->mac_top__DOT__cpu__DOT__fq_inval[0];
    s.fq1_valid = (uint8_t)r->mac_top__DOT__cpu__DOT__fq_valid[1];
    s.fq1_src = (uint8_t)(r->mac_top__DOT__cpu__DOT__fq_src[1] & 0x3u);
    s.fq1_inval = (uint8_t)r->mac_top__DOT__cpu__DOT__fq_inval[1];

    s.dcache_state =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__state;
    s.dcache_fa_set =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__fa_set;
    s.dcache_fa_way =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__fa_way;
    s.dcache_beat =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__beat_cnt;

    s.mem_iss_valid = (uint8_t)r->mac_top__DOT__cpu__DOT__mem_iss_v;
    s.mem_iss_is_store =
        (uint8_t)r->mac_top__DOT__cpu__DOT__mem_iss_is_st;
    s.mem_iss_is_rts =
        (uint8_t)r->mac_top__DOT__cpu__DOT__mem_iss_is_rts;
    // Task #243: mem_iss_pbase / mem_iss_pdst are no longer regs in
    // m68k_core after PRF widening (48->96, task #223) — Verilator
    // collapses the wires and only the iq_mem cell-output regs survive
    // in the public flat list.  Read them through the iq_mem cell port.
    s.mem_iss_pbase =
        (uint8_t)r->mac_top__DOT__cpu__DOT____Vcellout__u_iq_mem__iss_phys_base;
    s.mem_iss_base =
        (uint32_t)r->mac_top__DOT__cpu__DOT__lsu_base_val;
    s.mem_iss_disp =
        (uint32_t)r->mac_top__DOT__cpu__DOT__mem_iss_disp;
    s.mem_iss_pdst =
        (uint8_t)r->mac_top__DOT__cpu__DOT____Vcellout__u_iq_mem__iss_phys_dst;
    s.mem_iss_tag =
        (uint8_t)r->mac_top__DOT__cpu__DOT__mem_iss_tag;
    s.mem_iss_imm_is_data =
        (uint8_t)r->mac_top__DOT__cpu__DOT__mem_iss_imm_is_data;
    s.mem_iss_imm_data =
        (uint32_t)r->mac_top__DOT__cpu__DOT__mem_iss_imm_data;

    s.f1_mem_valid =
        (uint8_t)r->mac_top__DOT__cpu__DOT__f1_mem_valid;
    s.f1_mem_is_rts =
        (uint8_t)r->mac_top__DOT__cpu__DOT__f1_mem_is_rts;
    s.f1_mem_pbase =
        (uint8_t)r->mac_top__DOT__cpu__DOT__f1_mem_pbase;
    s.f1_mem_base =
        (uint32_t)r->mac_top__DOT__cpu__DOT__f1_mem_base_val;
    s.f1_mem_disp =
        (uint32_t)r->mac_top__DOT__cpu__DOT__f1_mem_disp;
    s.f1_mem_pdst =
        (uint8_t)r->mac_top__DOT__cpu__DOT__f1_mem_pdst;
    s.f1_mem_tag =
        (uint8_t)r->mac_top__DOT__cpu__DOT__f1_mem_tag;

    s.lsu_cur_is_rts =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__cur_is_rts;
    s.lsu_state =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__state;
    s.lsu_cur_is_store =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__cur_is_store;
    s.lsu_cur_split =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__cur_split;
    s.lsu_cur_pdst =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__cur_pdst;
    s.lsu_cur_tag =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__cur_tag;
    s.lsu_ea = (uint32_t)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__ea;
    s.lsu_cur_data =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__cur_data;
    s.lsu_split_first_rdata =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__split_first_rdata;
    s.lsu_split_second_addr =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__split_second_addr;
    s.lsu_split_second_wdata =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__split_second_wdata;
    s.lsu_split_second_wstrb =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_lsu__DOT__split_second_wstrb;
    s.commit_store_en =
        (uint8_t)r->mac_top__DOT__cpu__DOT__commit_store_en;
    s.flush_en = (uint8_t)r->mac_top__DOT__cpu__DOT__flush_en;
    s.core_dc_req = (uint8_t)r->mac_top__DOT__cpu__DOT__dc_req;
    s.core_dc_is_write =
        (uint8_t)r->mac_top__DOT__cpu__DOT__dc_is_write;
    s.core_dc_addr = (uint32_t)r->mac_top__DOT__cpu__DOT__dc_addr;
    s.core_dc_wdata = (uint32_t)r->mac_top__DOT__cpu__DOT__dc_wdata;
    s.core_dc_wstrb = (uint8_t)r->mac_top__DOT__cpu__DOT__dc_wstrb;
    s.core_dc_rdata = (uint32_t)r->mac_top__DOT__cpu__DOT__dc_rdata;
    s.core_dc_rvalid = (uint8_t)r->mac_top__DOT__cpu__DOT__dc_rvalid;
    s.core_dc_bvalid = (uint8_t)r->mac_top__DOT__cpu__DOT__dc_bvalid;

    const uint8_t dmmu_resp_ready =
        (uint8_t)(!r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__w_busy &&
                  !r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__req_pending);
    const uint8_t dmmu_fault_code =
        r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__ttr_wp_fault ? 7 :
        r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__atc_hit_sup_fault ? 4 :
        r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__atc_hit_wp_fault ? 3 :
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__last_fc;
    s.dmmu_req_valid =
        (uint8_t)r->mac_top__DOT__cpu__DOT____Vcellinp__u_dmmu__req_valid;
    s.dmmu_resp_ready = dmmu_resp_ready;
    s.dmmu_lsu_ready =
        (uint8_t)(dmmu_resp_ready &&
                  !r->mac_top__DOT__cpu__DOT__mmu_dcache_flush_req &&
                  !r->mac_top__DOT__cpu__DOT__flush_wake);
    s.dmmu_need_walk =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__need_walk;
    s.dmmu_w_busy =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__w_busy;
    s.dmmu_req_pending =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__req_pending;
    s.dmmu_req_wait_drop =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__req_wait_drop;
    s.dmmu_ttr_wp_fault =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__ttr_wp_fault;
    s.dmmu_atc_wp_fault =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__atc_hit_wp_fault;
    s.dmmu_atc_sup_fault =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__atc_hit_sup_fault;
    s.dmmu_fault =
        (uint8_t)(r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__ttr_wp_fault ||
                  r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__any_hit_fault ||
                  r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__last_fault);
    s.dmmu_fault_code = dmmu_fault_code;
    s.dmmu_mmu_flush =
        (uint8_t)r->mac_top__DOT__cpu__DOT__mmu_dcache_flush_req;
    s.dmmu_flush_wake =
        (uint8_t)r->mac_top__DOT__cpu__DOT__flush_wake;
    s.dmmu_va = (uint32_t)r->mac_top__DOT__cpu__DOT__dmmu_va_dbg;
    s.dmmu_pa = (uint32_t)r->mac_top__DOT__cpu__DOT__dmmu_pa_dbg;
    s.dmmu_tc = (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__r_tc;
    s.dmmu_itt0 = (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__r_itt0;
    s.dmmu_itt1 = (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__r_itt1;
    s.dmmu_dtt0 = (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__r_dtt0;
    s.dmmu_dtt1 = (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__r_dtt1;
    s.dmmu_srp = (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__r_srp;
    s.dmmu_urp = (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__r_urp;

    s.walk_state =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__state;
    s.walk_is_write =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__lat_is_write;
    s.walk_sup =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__lat_sup;
    s.walk_psz8k =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__lat_psz8k;
    s.walk_three =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__lat_three;
    s.walk_tia =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__lat_tia;
    s.walk_tib =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__lat_tib;
    s.walk_acc_wp =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__acc_wp;
    s.walk_l1_need_u =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__l1_need_u;
    s.walk_l2_need_u =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__l2_need_u;
    s.walk_leaf_need_u =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__leaf_need_u;
    s.walk_leaf_need_m =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__leaf_need_m;
    s.walk_up_phase =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__up_phase;
    s.walk_last_fault =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__last_fault;
    s.walk_last_ok =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__last_ok;
    s.walk_last_fc =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__last_fc;
    s.walk_out_fc =
        (uint8_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__w_out_fc;
    s.walk_lat_va =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__lat_va;
    s.walk_lat_root =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__lat_root;
    s.walk_l1_addr =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__l1_addr;
    s.walk_l1_pte =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__l1_pte;
    s.walk_l2_addr =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__l2_addr;
    s.walk_l2_pte =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__l2_pte;
    s.walk_leaf_addr =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__leaf_addr;
    s.walk_leaf_pte =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__leaf_pte;
    s.walk_leaf_entry_addr =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__u_walker__DOT__leaf_entry_addr;
    s.walk_last_pa = (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__last_pa;
    s.walk_last_fa = (uint32_t)r->mac_top__DOT__cpu__DOT__u_dmmu__DOT__last_fa;

    s.cdb1_en = (uint8_t)r->mac_top__DOT__cpu__DOT__cdb1_en;
    s.cdb1_has_dst =
        (uint8_t)r->mac_top__DOT__cpu__DOT__cdb1_has_dst;
    s.cdb1_phys = (uint8_t)r->mac_top__DOT__cpu__DOT__cdb1_phys;
    s.cdb1_data = (uint32_t)r->mac_top__DOT__cpu__DOT__cdb1_data;

    s.cmpl1_en = (uint8_t)r->mac_top__DOT__cpu__DOT__cmpl1_en;
    s.cmpl1_tag = (uint8_t)r->mac_top__DOT__cpu__DOT__cmpl1_tag;
    s.cmpl1_br_taken =
        (uint8_t)r->mac_top__DOT__cpu__DOT__cmpl1_br_taken;
    s.cmpl1_br_target =
        (uint32_t)r->mac_top__DOT__cpu__DOT__cmpl1_br_target;

    s.rob_is_branch = (uint8_t)r->mac_top__DOT__cpu__DOT__rob_isb;
    s.rob_branch_taken = (uint8_t)r->mac_top__DOT__cpu__DOT__rob_brt;
    s.rob_branch_target = (uint32_t)r->mac_top__DOT__cpu__DOT__rob_brtgt;
    s.commit_actual_next =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_commit__DOT__actual_next;

    s.daxi_arvalid = (uint8_t)dut->daxi_arvalid;
    s.daxi_arready = (uint8_t)dut->daxi_arready;
    s.daxi_araddr = (uint32_t)dut->daxi_araddr;
    s.daxi_rvalid = (uint8_t)dut->daxi_rvalid;
    s.daxi_rready = (uint8_t)dut->daxi_rready;
    s.daxi_rresp = (uint8_t)dut->daxi_rresp;
    s.daxi_awvalid = (uint8_t)dut->daxi_awvalid;
    s.daxi_awready = (uint8_t)dut->daxi_awready;
    s.daxi_awaddr = (uint32_t)dut->daxi_awaddr;
    s.daxi_wvalid = (uint8_t)dut->daxi_wvalid;
    s.daxi_wready = (uint8_t)dut->daxi_wready;
    s.daxi_bvalid = (uint8_t)dut->daxi_bvalid;
    s.daxi_bready = (uint8_t)dut->daxi_bready;
    s.daxi_bresp = (uint8_t)dut->daxi_bresp;

    s.if_req = (uint8_t)dut->if_req;
    s.if_addr = (uint32_t)dut->if_addr;
    s.if_rvalid = (uint8_t)dut->if_rvalid;
    s.if_pending = (uint8_t)if_pending;

    lastn_trace_ring[lastn_trace_next] = s;
    lastn_trace_next = (lastn_trace_next + 1) % lastn_trace_ring.size();
    if (lastn_trace_count < lastn_trace_ring.size()) lastn_trace_count++;
}

static void dump_lastn_trace() {
    if (lastn_trace_cycles == 0 || lastn_trace_count == 0) return;

    bool close_out = false;
    FILE* out = stderr;
    if (!lastn_trace_path.empty()) {
        out = std::fopen(lastn_trace_path.c_str(), "w");
        if (!out) {
            std::fprintf(stderr,
                "[rom-boot] cannot open last-N trace file %s; dumping to stderr\n",
                lastn_trace_path.c_str());
            out = stderr;
        } else {
            close_out = true;
        }
    }

    std::fprintf(out,
        "# rom-boot last-N-cycle trace capacity=%llu samples=%zu reason=%s\n",
        (unsigned long long)lastn_trace_cycles,
        lastn_trace_count,
        stop_reason);
    std::fprintf(out,
        "# columns: sim committed dbg_last_pc dbg_pc rob_v rob_c rob_pc "
        "rob_vec overlay/via1_overlay commit_exc_wait commit_take_exc exc_state exc_vec "
        "exc_fault_pc exc_a7 arch(sr/vbr/usp/ssp/isp/cacr/sfc/dfc) "
        "exc_gate_pending_exc exc_gate_pending_rte "
        "exc_flush_req exc_flush_done fq0(valid/src/inval) "
        "fq1(valid/src/inval) dcache(state/fa_set/fa_way/beat) "
        "mem_iss(v/st/rts/pbase/base/disp/pdst/tag/imm_data/imm) "
        "f1_mem(v/rts/pbase/base/disp/pdst/tag) "
        "lsu(rts/st/split/state/pdst/tag/ea/data/split_r/split2/commit_store/flush) "
        "dc(req/wr/addr/wdata/wstrb/rvalid/rdata/bvalid) "
        "dmmu(req/ready/lsu_ready/need/wbusy/pending/waitdrop/ttrwp/atcwp/atcsup/"
        "fault/fc/mmuflush/wake/va/pa/tc/itt0/itt1/dtt0/dtt1/srp/urp) "
        "walk(state/isw/sup/psz8k/three/tia/tib/accwp/l1u/l2u/leafu/leafm/"
        "phase/lastfault/lastok/lastfc/outfc/latva/root/l1a/l1p/l2a/l2p/leafa/"
        "leafp/leafentry/lastpa/lastfa) "
        "cdb1(en/hd/phys/data) cmpl1(en/tag/br/target) "
        "robbr(is/taken/target/actual_next) "
        "daxi_ar(v/r/addr) daxi_r(v/r/resp) daxi_aw(v/r/addr) "
        "daxi_w(v/r) daxi_b(v/r/resp) if(req/addr/rvalid/pending)\n");

    const size_t cap = lastn_trace_ring.size();
    const size_t start = (lastn_trace_next + cap - lastn_trace_count) % cap;
    for (size_t i = 0; i < lastn_trace_count; i++) {
        const LastNTraceSample& s = lastn_trace_ring[(start + i) % cap];
        std::fprintf(out,
            "lastn[%05zu] sim=%llu committed=%u dbg_last_pc=0x%08x "
            "dbg_pc=0x%08x rob_v=%u rob_c=%u rob_pc=0x%08x rob_vec=%u "
            "overlay=%u/%u "
            "commit_exc_wait=%u commit_take_exc=%u exc_state=%u exc_vec=%u "
            "exc_fault_pc=0x%08x exc_a7=0x%08x "
            "arch=0x%04x/0x%08x/0x%08x/0x%08x/0x%08x/"
            "0x%08x/0x%08x/0x%08x "
            "exc_gate_pending_exc=%u exc_gate_pending_rte=%u "
            "exc_flush_req=%u exc_flush_done=%u fq0=%u/%u/%u fq1=%u/%u/%u "
            "dcache=%u/%u/%u/%u "
            "mem_iss=%u/%u/%u/%u/0x%08x/0x%08x/%u/%u/%u/0x%08x "
            "f1_mem=%u/%u/%u/0x%08x/0x%08x/%u/%u "
            "lsu=%u/%u/%u/%u/%u/%u/0x%08x/0x%08x/0x%08x/"
            "0x%08x/0x%08x/%u/%u/%u "
            "dc=%u/%u/0x%08x/0x%08x/%u/%u/0x%08x/%u "
            "dmmu=%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/"
            "0x%08x/0x%08x/0x%08x/0x%08x/0x%08x/0x%08x/0x%08x/0x%08x/0x%08x "
            "walk=%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/%u/"
            "0x%08x/0x%08x/0x%08x/0x%08x/0x%08x/0x%08x/0x%08x/0x%08x/"
            "0x%08x/0x%08x/0x%08x "
            "cdb1=%u/%u/%u/0x%08x cmpl1=%u/%u/%u/0x%08x "
            "robbr=%u/%u/0x%08x/0x%08x "
            "daxi_ar=%u/%u/0x%08x "
            "daxi_r=%u/%u/%u daxi_aw=%u/%u/0x%08x daxi_w=%u/%u "
            "daxi_b=%u/%u/%u if=%u/0x%08x/%u/%u\n",
            i,
            (unsigned long long)s.sim_time,
            s.committed,
            s.dbg_last_pc,
            s.dbg_pc,
            s.rob_valid,
            s.rob_complete,
            s.rob_pc,
            s.rob_vec,
            s.overlay_active,
            s.via1_overlay_live,
            s.commit_exc_wait,
            s.commit_take_exc,
            s.exc_state,
            s.exc_vec,
            s.exc_fault_pc,
            s.exc_a7,
            s.arch_sr,
            s.arch_vbr,
            s.arch_usp,
            s.arch_ssp,
            s.arch_isp,
            s.arch_cacr,
            s.arch_sfc,
            s.arch_dfc,
            s.exc_gate_pending_exc,
            s.exc_gate_pending_rte,
            s.exc_flush_req,
            s.exc_flush_done,
            s.fq0_valid,
            s.fq0_src,
            s.fq0_inval,
            s.fq1_valid,
            s.fq1_src,
            s.fq1_inval,
            s.dcache_state,
            s.dcache_fa_set,
            s.dcache_fa_way,
            s.dcache_beat,
            s.mem_iss_valid,
            s.mem_iss_is_store,
            s.mem_iss_is_rts,
            s.mem_iss_pbase,
            s.mem_iss_base,
            s.mem_iss_disp,
            s.mem_iss_pdst,
            s.mem_iss_tag,
            s.mem_iss_imm_is_data,
            s.mem_iss_imm_data,
            s.f1_mem_valid,
            s.f1_mem_is_rts,
            s.f1_mem_pbase,
            s.f1_mem_base,
            s.f1_mem_disp,
            s.f1_mem_pdst,
            s.f1_mem_tag,
            s.lsu_cur_is_rts,
            s.lsu_cur_is_store,
            s.lsu_cur_split,
            s.lsu_state,
            s.lsu_cur_pdst,
            s.lsu_cur_tag,
            s.lsu_ea,
            s.lsu_cur_data,
            s.lsu_split_first_rdata,
            s.lsu_split_second_addr,
            s.lsu_split_second_wdata,
            s.lsu_split_second_wstrb,
            s.commit_store_en,
            s.flush_en,
            s.core_dc_req,
            s.core_dc_is_write,
            s.core_dc_addr,
            s.core_dc_wdata,
            s.core_dc_wstrb,
            s.core_dc_rvalid,
            s.core_dc_rdata,
            s.core_dc_bvalid,
            s.dmmu_req_valid,
            s.dmmu_resp_ready,
            s.dmmu_lsu_ready,
            s.dmmu_need_walk,
            s.dmmu_w_busy,
            s.dmmu_req_pending,
            s.dmmu_req_wait_drop,
            s.dmmu_ttr_wp_fault,
            s.dmmu_atc_wp_fault,
            s.dmmu_atc_sup_fault,
            s.dmmu_fault,
            s.dmmu_fault_code,
            s.dmmu_mmu_flush,
            s.dmmu_flush_wake,
            s.dmmu_va,
            s.dmmu_pa,
            s.dmmu_tc,
            s.dmmu_itt0,
            s.dmmu_itt1,
            s.dmmu_dtt0,
            s.dmmu_dtt1,
            s.dmmu_srp,
            s.dmmu_urp,
            s.walk_state,
            s.walk_is_write,
            s.walk_sup,
            s.walk_psz8k,
            s.walk_three,
            s.walk_tia,
            s.walk_tib,
            s.walk_acc_wp,
            s.walk_l1_need_u,
            s.walk_l2_need_u,
            s.walk_leaf_need_u,
            s.walk_leaf_need_m,
            s.walk_up_phase,
            s.walk_last_fault,
            s.walk_last_ok,
            s.walk_last_fc,
            s.walk_out_fc,
            s.walk_lat_va,
            s.walk_lat_root,
            s.walk_l1_addr,
            s.walk_l1_pte,
            s.walk_l2_addr,
            s.walk_l2_pte,
            s.walk_leaf_addr,
            s.walk_leaf_pte,
            s.walk_leaf_entry_addr,
            s.walk_last_pa,
            s.walk_last_fa,
            s.cdb1_en,
            s.cdb1_has_dst,
            s.cdb1_phys,
            s.cdb1_data,
            s.cmpl1_en,
            s.cmpl1_tag,
            s.cmpl1_br_taken,
            s.cmpl1_br_target,
            s.rob_is_branch,
            s.rob_branch_taken,
            s.rob_branch_target,
            s.commit_actual_next,
            s.daxi_arvalid,
            s.daxi_arready,
            s.daxi_araddr,
            s.daxi_rvalid,
            s.daxi_rready,
            s.daxi_rresp,
            s.daxi_awvalid,
            s.daxi_awready,
            s.daxi_awaddr,
            s.daxi_wvalid,
            s.daxi_wready,
            s.daxi_bvalid,
            s.daxi_bready,
            s.daxi_bresp,
            s.if_req,
            s.if_addr,
            s.if_rvalid,
            s.if_pending);
    }

    if (close_out) {
        std::fclose(out);
        std::fprintf(stderr, "[rom-boot] last-N trace at %s\n",
                     lastn_trace_path.c_str());
    } else {
        std::fflush(out);
    }
}

static const char* addr_class_name_for_dump(uint32_t a, bool fetch_view) {
    if (fetch_view && ifetch_overlay_rom(a)) return "overlay-rom";
    switch (classify(a)) {
        case AC_RAM:      return "ram";
        case AC_ROM:      return "rom";
        case AC_IO:       return "io";
        case AC_UNMAPPED: return "unmapped";
    }
    return "unknown";
}

static uint8_t fault_dump_read8(uint32_t a, bool fetch_view) {
    return fetch_view ? cpu_read8(a) : peek_data8(a);
}

static uint32_t fault_window_start(uint32_t center, uint64_t bytes) {
    uint32_t half = (uint32_t)(bytes / 2);
    uint32_t start = (center > half) ? (center - half) : 0;
    return start & ~0xFu;
}

static bool write_fault_window_file(const std::string& path,
                                    const char* label,
                                    uint32_t center,
                                    bool fetch_view) {
    FILE* fp = std::fopen(path.c_str(), "w");
    if (!fp) {
        std::fprintf(stderr, "[rom-boot] cannot open fault dump %s\n",
                     path.c_str());
        return false;
    }

    const uint64_t bytes = fault_dump_bytes ? fault_dump_bytes : 1;
    const uint32_t start = fault_window_start(center, bytes);
    const uint32_t end = start + (uint32_t)bytes;
    std::fprintf(fp,
        "# rom-boot fault window label=%s center=0x%08x start=0x%08x bytes=%llu view=%s reason=%s\n",
        label,
        center,
        start,
        (unsigned long long)bytes,
        fetch_view ? "fetch" : "data",
        stop_reason);
    for (uint32_t a = start; a < end; a += 16) {
        std::fprintf(fp, "%08x  ", a);
        for (int i = 0; i < 16 && (a + (uint32_t)i) < end; i++) {
            std::fprintf(fp, "%02x", fault_dump_read8(a + (uint32_t)i, fetch_view));
            if (i != 15) std::fprintf(fp, " ");
        }
        std::fprintf(fp, "  # %s\n", addr_class_name_for_dump(a, fetch_view));
    }
    std::fclose(fp);
    return true;
}

static void dump_one_fault_window(FILE* manifest,
                                  const char* label,
                                  uint32_t center,
                                  bool fetch_view) {
    char path[1024];
    std::snprintf(path, sizeof(path), "%s/%s_%08x.hex",
                  fault_dump_dir.c_str(), label, center);
    bool ok = write_fault_window_file(path, label, center, fetch_view);
    std::fprintf(manifest, "%s\t0x%08x\t%s\t%s\n",
                 label, center, fetch_view ? "fetch" : "data", path);
    if (ok) {
        std::fprintf(stderr, "[rom-boot] fault dump: %s\n", path);
    }
}

static void dump_fault_windows() {
    if (fault_dump_dir.empty() || fault_dump_bytes == 0 || !dut || !mem) return;

    if (::mkdir(fault_dump_dir.c_str(), 0777) != 0 && errno != EEXIST) {
        std::fprintf(stderr, "[rom-boot] cannot create fault dump dir %s\n",
                     fault_dump_dir.c_str());
        return;
    }

    auto* r = dut->rootp;
    const uint32_t exc_fault_pc =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_exception__DOT__cur_fault_pc;
    const uint32_t exc_fault_addr =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_exception__DOT__cur_fault_addr;
    const uint32_t rob_pc = (uint32_t)r->mac_top__DOT__cpu__DOT__rob_pc;
    const uint32_t sp = read_arch_reg(15);
    const unsigned ccr =
        r->mac_top__DOT__cpu__DOT__u_ccr_rat__DOT__ccr_prf
            [r->mac_top__DOT__cpu__DOT__u_ccr_rat__DOT__crat_tag] & 0x1fu;
    const uint16_t sr =
        (uint16_t)((r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_sr & 0xffe0u)
                   | ccr);

    char manifest_path[1024];
    std::snprintf(manifest_path, sizeof(manifest_path), "%s/manifest.tsv",
                  fault_dump_dir.c_str());
    FILE* manifest = std::fopen(manifest_path, "w");
    if (!manifest) {
        std::fprintf(stderr, "[rom-boot] cannot open fault dump manifest %s\n",
                     manifest_path);
        return;
    }

    std::fprintf(manifest,
        "# reason=%s cycles=%llu committed=%u last_pc=0x%08x next_fetch=0x%08x\n",
        stop_reason,
        (unsigned long long)sim_time,
        (unsigned)dut->dbg_committed,
        (unsigned)dut->dbg_last_pc,
        (unsigned)dut->dbg_pc);
    std::fprintf(manifest,
        "# exc_vec=%u fault_pc=0x%08x fault_addr=0x%08x rob_pc=0x%08x if_addr_q=0x%08x sp=0x%08x bytes=%llu\n",
        (unsigned)r->mac_top__DOT__cpu__DOT__u_exception__DOT__cur_vec,
        exc_fault_pc,
        exc_fault_addr,
        rob_pc,
        if_addr_q,
        sp,
        (unsigned long long)fault_dump_bytes);
    std::fprintf(manifest,
        "# arch sr=0x%04x vbr=0x%08x usp=0x%08x ssp=0x%08x isp=0x%08x cacr=0x%08x sfc=0x%08x dfc=0x%08x\n",
        sr,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_vbr,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__usp,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__ssp,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__isp,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_cacr,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_sfc,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_dfc);
    std::fprintf(manifest, "# label\tcenter\tview\tpath\n");

    if (exc_fault_pc != 0)
        dump_one_fault_window(manifest, "code_fault_pc", exc_fault_pc, true);
    if (rob_pc != 0)
        dump_one_fault_window(manifest, "code_rob_pc", rob_pc, true);
    if (dut->dbg_last_pc != 0)
        dump_one_fault_window(manifest, "code_last_pc", (uint32_t)dut->dbg_last_pc, true);
    if (dut->dbg_pc != 0)
        dump_one_fault_window(manifest, "code_next_fetch", (uint32_t)dut->dbg_pc, true);
    if (if_addr_q != 0)
        dump_one_fault_window(manifest, "code_if_addr", if_addr_q, true);
    if (last_ifetch_berr_addr != 0)
        dump_one_fault_window(manifest, "code_ifetch_berr", last_ifetch_berr_addr, true);
    if (exc_fault_addr != 0)
        dump_one_fault_window(manifest, "data_fault_addr", exc_fault_addr, false);
    if (sp != 0)
        dump_one_fault_window(manifest, "data_sp", sp, false);

    std::fclose(manifest);
    std::fprintf(stderr, "[rom-boot] fault dump manifest: %s\n", manifest_path);
}

static inline bool ifetch_overlay_rom(uint32_t a);

static void dump_instr_bytes_line(FILE* fp, const char* label, uint32_t addr) {
    if (!fp || !label) return;

    std::fprintf(fp, "[rom-boot] instr-bytes %s 0x%08x:", label, addr);
    bool any_backed = false;
    for (int i = 0; i < 16; ++i) {
        uint32_t a = addr + (uint32_t)i;
        AddrClass cls = classify(a);
        bool backed = ifetch_overlay_rom(a) || cls == AC_RAM || cls == AC_ROM;
        if (backed) {
            std::fprintf(fp, " %02x", cpu_read8(a));
            any_backed = true;
        } else {
            std::fprintf(fp, " ??");
        }
    }
    if (!any_backed) std::fprintf(fp, " <unmapped>");
    std::fprintf(fp, "\n");
}

static void load_fetch_line(uint32_t addr, uint8_t out[16]) {
    uint32_t base = addr & ~0xFu;
    for (int i = 0; i < 16; i++) out[i] = cpu_read8(base + i);
}

static bool ifetch_byte_backed(uint32_t a) {
    if (ifetch_overlay_rom(a)) return true;
    AddrClass c = classify(a);
    return c == AC_RAM || c == AC_ROM;
}

static bool ifetch_line_backed(uint32_t addr) {
    uint32_t base = addr & ~0xFu;
    for (int i = 0; i < 16; i++) {
        if (!ifetch_byte_backed(base + (uint32_t)i)) return false;
    }
    return true;
}

static bool check_stop_on_ifetch_berr(uint32_t addr) {
    if ((!stop_on_ifetch_berr && !end_on_ifetch_berr) ||
        ifetch_line_backed(addr)) {
        return false;
    }
    last_ifetch_berr_addr = addr & ~0xFu;
    if (stop_on_ifetch_berr) {
        std::snprintf(stop_ifetch_berr_reason, sizeof(stop_ifetch_berr_reason),
            "stop-ifetch-berr addr=0x%08x dbg_pc=0x%08x dbg_last_pc=0x%08x committed=%u",
            last_ifetch_berr_addr,
            dut ? (unsigned)dut->dbg_pc : 0,
            dut ? (unsigned)dut->dbg_last_pc : 0,
            dut ? (unsigned)dut->dbg_committed : 0);
        stop_reason = stop_ifetch_berr_reason;
    } else {
        std::snprintf(end_ifetch_berr_reason, sizeof(end_ifetch_berr_reason),
            "end-ifetch-berr addr=0x%08x dbg_pc=0x%08x dbg_last_pc=0x%08x committed=%u",
            last_ifetch_berr_addr,
            dut ? (unsigned)dut->dbg_pc : 0,
            dut ? (unsigned)dut->dbg_last_pc : 0,
            dut ? (unsigned)dut->dbg_committed : 0);
        stop_reason = end_ifetch_berr_reason;
    }
    test_done = true;
    return true;
}

static void drive_ifetch() {
    dut->if_rvalid = 0;
    dut->if_fault = 0;
    if (dut->if_req && if_pending == 0) {
        if (check_stop_on_ifetch_berr((uint32_t)dut->if_addr)) return;
        maybe_clear_overlay_from_high_rom_fetch((uint32_t)dut->if_addr);
        if_addr_q  = dut->if_addr;
        if_pending = 2;
    }
    if (if_pending > 0) {
        if (--if_pending == 0) {
            uint8_t line[16];
            load_fetch_line(if_addr_q, line);
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
            dut->if_fault = ifetch_line_backed(if_addr_q) ? 0 : 1;
        }
    }
}

static uint32_t daxi_read32(uint32_t a) {
    uint32_t aa = a & ~0x3u;
    AddrClass c = classify(aa);
    if (c == AC_IO) {
        uint32_t d = io_read32(aa);
        data_watch_log_access(false, aa, 0xFu, d, daxi_rresp_for_addr(aa),
                              data_watch_source_name(aa, c), "daxi");
        return d;
    }
    if (c == AC_ROM) {
        // Big-endian byte assembly via cpu_read8 so overlay-mirror +
        // ROM-window logic is shared with ifetch.
        uint32_t d = ((uint32_t)cpu_read8(aa)     << 24)
                   | ((uint32_t)cpu_read8(aa + 1) << 16)
                   | ((uint32_t)cpu_read8(aa + 2) <<  8)
                   |  (uint32_t)cpu_read8(aa + 3);
        periph_record_access(PC_ROM, false, aa, d, (uint32_t)dut->dbg_pc, "probe/read");
        data_watch_log_access(false, aa, 0xFu, d, daxi_rresp_for_addr(aa),
                              data_watch_source_name(aa, c), "daxi");
        return d;
    }
    if (c == AC_RAM) {
        uint32_t d = mem->read32(aa);
        if (is_dafb_vram_addr(aa)) {
            const std::string detail = vram_access_detail(false, aa);
            periph_record_access(PC_VRAM, false, aa, d, (uint32_t)dut->dbg_pc,
                                 detail.c_str());
            periph_event("VRAM", "read", aa, d, detail);
        } else {
            periph_record_access(PC_RAM, false, aa, d, (uint32_t)dut->dbg_pc, "probe/read");
        }
        data_watch_log_access(false, aa, 0xFu, d, daxi_rresp_for_addr(aa),
                              data_watch_source_name(aa, c), "daxi");
        return d;
    }
    if (is_dafb_reg_addr(aa)) {
        const std::string detail = dafb_detail(aa) + ",fallthrough=outer-daxi";
        periph_record_access(PC_DAFB_REG, false, aa, 0xFFFFFFFFu,
                             (uint32_t)dut->dbg_pc, detail.c_str());
        periph_event("DAFB", "read", aa, 0xFFFFFFFFu, detail);
        data_watch_log_access(false, aa, 0xFu, 0xFFFFFFFFu,
                              daxi_rresp_for_addr(aa),
                              data_watch_source_name(aa, c), "daxi");
        return 0xFFFFFFFFu;
    }
    unmapped_read_count[aa & ~0xFFu]++;
    periph_record_access(PC_UNMAPPED, false, aa, 0xFFFFFFFFu,
                         (uint32_t)dut->dbg_pc, "probe/read");
    data_watch_log_access(false, aa, 0xFu, 0xFFFFFFFFu, daxi_rresp_for_addr(aa),
                          data_watch_source_name(aa, c), "daxi");
    if (verbose) std::fprintf(stderr,
        "[daxi] read unmapped 0x%08x pc=0x%08x\n",
        a, (unsigned)dut->dbg_pc);
    return 0xFFFFFFFFu;
}

static void daxi_write32(uint32_t a, uint32_t data, uint32_t strb) {
    AddrClass c = classify(a);
    if (c == AC_IO) {
        uint32_t aa = a & ~0x3u;
        io_write32(aa, data, strb);
        data_watch_log_access(true, aa, strb, data, daxi_bresp_for_addr(aa),
                              data_watch_source_name(aa, c), "daxi");
        return;
    }
    if (c == AC_ROM) {
        // Writes to ROM are silently dropped in hardware.  Track for
        // diagnostics only.
        unmapped_write_count[a & ~0xFFu]++;
        periph_record_access(PC_ROM, true, a & ~0x3u, data, (uint32_t)dut->dbg_pc, "dropped");
        data_watch_log_access(true, a & ~0x3u, strb, data,
                              daxi_bresp_for_addr(a),
                              data_watch_source_name(a & ~0x3u, c), "daxi");
        return;
    }
    if (c == AC_RAM) {
        uint32_t aa = a & ~0x3u;
        for (int i = 0; i < 4; i++) {
            if (strb & (1u << (3 - i))) {
                uint8_t bv = (data >> ((3 - i) * 8)) & 0xFF;
                mem->write8(aa + i, bv);
            }
        }
        if (cache_event_fp && cache_event_lowmem_addr(aa)) {
            std::string detail = "source=daxi lowmem=1 strb=0x";
            char buf[8];
            std::snprintf(buf, sizeof(buf), "%x", strb & 0xFu);
            detail += buf;
            cache_event_record("lowmem-write", aa, data, detail, true);
            cache_event_stats.lowmem_writes++;
        }
        // Sentinel word for synthetic tests.  Keep this exact so real
        // ROM exception frames near 0xffff0000 do not look like test
        // pass/fail markers while we are exercising bus-error paths.
        if (aa == 0xFFFF0000u && strb == 0xFu &&
            (data == 0xC0FFEE00u || data == 0xDEADBEEFu)) {
            test_done = true;
            stop_reason = (data == 0xC0FFEE00u) ? "sentinel-PASS" : "sentinel-FAIL";
        }
        if (is_dafb_vram_addr(aa)) {
            const std::string detail = vram_access_detail(true, aa);
            periph_record_access(PC_VRAM, true, aa, data, (uint32_t)dut->dbg_pc,
                                 detail.c_str());
            periph_event("VRAM", "write", aa, data, detail);
        } else {
            periph_record_access(PC_RAM, true, aa, data, (uint32_t)dut->dbg_pc, "write");
        }
        data_watch_log_access(true, aa, strb, data, daxi_bresp_for_addr(aa),
                              data_watch_source_name(aa, c), "daxi");
        return;
    }
    if (is_dafb_reg_addr(a & ~0x3u)) {
        const uint32_t aa = a & ~0x3u;
        const std::string detail = dafb_detail(aa) + ",fallthrough=outer-daxi";
        periph_record_access(PC_DAFB_REG, true, aa, data,
                             (uint32_t)dut->dbg_pc, detail.c_str());
        periph_event("DAFB", "write", aa, data, detail);
        data_watch_log_access(true, aa, strb, data, daxi_bresp_for_addr(a),
                              data_watch_source_name(aa, c), "daxi");
        return;
    }
    unmapped_write_count[a & ~0xFFu]++;
    periph_record_access(PC_UNMAPPED, true, a & ~0x3u, data, (uint32_t)dut->dbg_pc, "probe/write");
    data_watch_log_access(true, a & ~0x3u, strb, data, daxi_bresp_for_addr(a),
                          data_watch_source_name(a & ~0x3u, c), "daxi");
    if (verbose) std::fprintf(stderr,
        "[daxi] UNMAPPED wr 0x%08x = 0x%08x strb=0x%x pc=0x%08x\n",
        a, data, strb, (unsigned)dut->dbg_pc);
}

static void drive_daxi() {
    dut->daxi_arready = (!ax.ar_outstanding && !ax.b_pending);
    if (dut->daxi_arvalid && dut->daxi_arready) {
        ax.ar_addr = dut->daxi_araddr;
        ax.ar_len = dut->daxi_arlen;
        ax.ar_size = dut->daxi_arsize;
        ax.ar_burst = dut->daxi_arburst;
        ax.ar_beat = 0;
        daxi_ar_reqs++;
        if (ax.ar_len != 0) daxi_ar_burst_reqs++;
        daxi_ar_burst_beats += (uint64_t)ax.ar_len + 1ull;
        ax.ar_outstanding = true;
        ax.ar_delay = 1;
    }
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
            uint32_t d = daxi_read32(beat_addr);
            dut->daxi_rdata  = d;
            dut->daxi_rvalid = 1;
            dut->daxi_rlast  = (ax.ar_beat == ax.ar_len) ? 1 : 0;
            // Most unmapped probes, including unbacked low-RAM holes during
            // ROM RAM sizing, behave like open bus in the legacy harness.
            // +strict_axi_resp keeps the data path open-bus-like but upgrades
            // the AXI response to DECERR/SLVERR where the RTL would fault.
            dut->daxi_rresp  = daxi_rresp_for_addr(beat_addr);
            if (dut->daxi_rready) {
                if (ax.ar_beat == ax.ar_len) {
                    ax.ar_outstanding = false;
                } else {
                    ax.ar_beat++;
                }
            }
        }
    }

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
    if (ax.aw_done && ax.w_done && !ax.b_pending) {
        daxi_write32(ax.aw_addr & ~0x3u, ax.w_data, ax.w_strb);
        ax.b_resp = daxi_bresp_for_addr(ax.aw_addr);
        ax.aw_done = false;
        ax.w_done  = false;
        ax.b_pending = true;
        ax.b_delay   = 1;
    }
    dut->daxi_bvalid = 0;
    dut->daxi_bresp  = 0;
    if (ax.b_pending) {
        if (ax.b_delay > 0) ax.b_delay--;
        else {
            dut->daxi_bvalid = 1;
            dut->daxi_bresp  = ax.b_resp;
            if (dut->daxi_bready) ax.b_pending = false;
        }
    }
}

// DAFB register requests intercepted inside mac_top never reach the outer
// DAXI port.  Observe those CPU-side handshakes directly; any register-window
// fallthrough is caught in daxi_read32/daxi_write32 so the 4 KiB display watch
// remains complete without changing RTL decode.
static void sample_internal_dafb_activity() {
    if (!dut || !dut->rootp) return;
    auto* r = dut->rootp;
    uint32_t pc = (uint32_t)dut->dbg_pc;
    bool write_fire = r->mac_top__DOT__wr_route_valid &&
                      r->mac_top__DOT__wr_route_dafb &&
                      r->mac_top__DOT__dafb_bvalid &&
                      r->mac_top__DOT__cpu_daxi_bready;
    bool read_fire = r->mac_top__DOT__rd_route_valid &&
                     r->mac_top__DOT__rd_route_dafb &&
                     r->mac_top__DOT__dafb_rvalid &&
                     r->mac_top__DOT__cpu_daxi_rready;
    static bool prev_write_fire = false;
    static bool prev_read_fire = false;

    if (write_fire && !prev_write_fire) {
        const std::string detail = dafb_detail((uint32_t)r->daxi_awaddr);
        periph_record_access(PC_DAFB_REG, true, (uint32_t)r->daxi_awaddr,
                             (uint32_t)r->daxi_wdata, pc, detail.c_str());
        periph_event_c("DAFB", "write", (uint32_t)r->daxi_awaddr,
                       (uint32_t)r->daxi_wdata, detail.c_str());
        data_watch_log_access(true, (uint32_t)r->daxi_awaddr & ~0x3u, 0xFu,
                              (uint32_t)r->daxi_wdata, 0, "dafb-reg",
                              "mac_top");
    }
    if (read_fire && !prev_read_fire) {
        const std::string detail = dafb_detail((uint32_t)r->daxi_araddr);
        periph_record_access(PC_DAFB_REG, false, (uint32_t)r->daxi_araddr,
                             (uint32_t)r->mac_top__DOT__dafb_rdata, pc,
                             detail.c_str());
        periph_event_c("DAFB", "read", (uint32_t)r->daxi_araddr,
                       (uint32_t)r->mac_top__DOT__dafb_rdata, detail.c_str());
        data_watch_log_access(false, (uint32_t)r->daxi_araddr & ~0x3u, 0xFu,
                              (uint32_t)r->mac_top__DOT__dafb_rdata, 0,
                              "dafb-reg", "mac_top");
    }
    prev_write_fire = write_fire;
    prev_read_fire = read_fire;
}

static void tick() {
    dut->clk = 1;
    dut->eval();
    if (waves) fst->dump(sim_time * 10 + 5);
    sim_time++;
    host_cycle = sim_time;
    via1_adb_tick();                         // ADB shadow deferred-flag tick
    via1_timer_tick();                       // VIA1 Timer 1 countdown (VBL poll)
    dut->clk = 0;
    dut->eval();
    if (waves) fst->dump(sim_time * 10);
    if (stop_cycle != 0 && sim_time >= stop_cycle) {
        test_done = true;
        stop_reason = "cycle-budget-exhausted";
    }
}

static void reset_bus() {
    dut->rst = 1;
    clear_arch_debug_load_ports();
    clear_core_debug_stop_ports();
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
    for (int i = 0; i < 16; i++) tick();
    dut->rst = 0;
}

// ─── CLI ─────────────────────────────────────────────────────────────
static void parse_args(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    for (int i = 1; i < argc; i++) {
        std::string a(argv[i]);
        if      (a.rfind("+rom=",0)       == 0) rom_path   = a.substr(5);
        else if (a.rfind("+trace=",0)     == 0) trace_path = a.substr(7);
        else if (a.rfind("+periph_log=",0) == 0) periph_log_path = a.substr(12);
        else if (a.rfind("+periph_log_limit=",0) == 0) periph_log_limit = std::stoull(a.substr(18));
        else if (a.rfind("+periph_event_log=",0) == 0) periph_event_log_path = a.substr(18);
        else if (a.rfind("+periph_event_log_limit=",0) == 0) periph_event_log_limit = std::stoull(a.substr(24));
        else if (a.rfind("+periph_event_filter=",0) == 0) parse_periph_event_filter(a.substr(21));
        else if (a == "+periph_event_summary")  periph_events.set_summary_enabled(true);
        else if (a.rfind("+cache_event_log=",0) == 0) cache_event_log_path = a.substr(17);
        else if (a.rfind("+cache_event_log_limit=",0) == 0) cache_event_log_limit = std::stoull(a.substr(23));
        else if (a.rfind("+probe_log=",0) == 0) probe_log_path = a.substr(11);
        else if (a.rfind("+probe_log_limit=",0) == 0) probe_log_limit = std::stoull(a.substr(17));
        else if (a.rfind("+data_watch=",0) == 0) parse_data_watch_list(a.substr(12));
        else if (a.rfind("+data_watch_log=",0) == 0) data_watch_log_path = a.substr(16);
        else if (a.rfind("+data_watch_limit=",0) == 0) data_watch_limit = std::stoull(a.substr(18));
        else if (a.rfind("+lastn_trace=",0) == 0) lastn_trace_cycles = std::stoull(a.substr(13));
        else if (a.rfind("+lastn_trace_path=",0) == 0) lastn_trace_path = a.substr(18);
        else if (a.rfind("+fault_dump_dir=",0) == 0) fault_dump_dir = a.substr(16);
        else if (a.rfind("+fault_dump_bytes=",0) == 0) fault_dump_bytes = std::stoull(a.substr(18));
        else if (a.rfind("+timeout=",0)   == 0) timeout_cycles = std::stoull(a.substr(9));
        else if (a.rfind("+max_insts=",0) == 0) max_insts     = std::stoull(a.substr(11));
        else if (a.rfind("+q700_ram=",0)  == 0) visible_ram_size = parse_byte_size_arg(a.substr(10), "+q700_ram");
        else if (a.rfind("+save_state=",0) == 0) save_state_path = a.substr(12);
        else if (a.rfind("+restore_state=",0) == 0) restore_state_path = a.substr(15);
        else if (a.rfind("+arch_checkpoint=",0) == 0) arch_checkpoint_path = a.substr(17);
        else if (a.rfind("+arch_replay=",0) == 0) arch_replay_path = a.substr(13);
        else if (a.rfind("+arch_checkpoint_flush_timeout=",0) == 0) arch_checkpoint_flush_timeout = std::stoull(a.substr(31));
        else if (a.rfind("+arch_sample_every=",0) == 0) arch_sample_every = std::stoull(a.substr(19));
        else if (a.rfind("+arch_sample_log=",0) == 0) arch_sample_log_path = a.substr(17);
        else if (a.rfind("+checkpoint_prefix=",0) == 0) checkpoint_prefix = a.substr(19);
        else if (a.rfind("+checkpoint_every=",0) == 0) checkpoint_every = std::stoull(a.substr(18));
        else if (a.rfind("+checkpoint_points=",0) == 0) parse_uint64_list(a.substr(19), checkpoint_points);
        else if (a.rfind("+checkpoint_cycle_points=",0) == 0) parse_uint64_list(a.substr(25), checkpoint_cycle_points);
        else if (a.rfind("+stop_cycle=",0) == 0) absolute_stop_cycle = std::stoull(a.substr(12));
        else if (a.rfind("+stop_pc=",0) == 0) parse_stop_pc_list(a.substr(9));
        else if (a.rfind("+stop_pc_hit=",0) == 0) stop_pc_hit_target = parse_uint64_auto(a.substr(13), "+stop_pc_hit");
        else if (a == "+stop_on_sad_mac") stop_on_sad_mac = true;
        else if (a == "+stop_on_disk_prompt") stop_on_disk_prompt = true;
        else if (a.rfind("+disk_prompt_hit=",0) == 0)
            disk_prompt_hit_target =
                parse_uint64_auto(a.substr(17), "+disk_prompt_hit");
        else if (a == "+stop_on_rom_outcome") enable_rom_outcome_stops();
        else if (a.rfind("+end_pc=",0) == 0) parse_end_pc_list(a.substr(8));
        else if (a.rfind("+end_pc_hit=",0) == 0) end_pc_hit_target = parse_uint64_auto(a.substr(12), "+end_pc_hit");
        else if (a.rfind("+stop_on_exc=",0) == 0) parse_stop_exc_list(a.substr(13));
        else if (a.rfind("+stop_on_exc_after_committed=",0) == 0)
            stop_on_exc_after_committed =
                parse_uint64_auto(a.substr(29), "+stop_on_exc_after_committed");
        else if (a == "+stop_on_illegal") enable_illegal_stop();
        else if (a == "+stop_on_rom_faults") enable_rom_fault_stops();
        else if (a == "+stop_on_ifetch_berr") stop_on_ifetch_berr = true;
        else if (a.rfind("+end_on_exc=",0) == 0) parse_end_exc_list(a.substr(12));
        else if (a.rfind("+end_on_exc_after_committed=",0) == 0)
            end_on_exc_after_committed =
                parse_uint64_auto(a.substr(28), "+end_on_exc_after_committed");
        else if (a == "+end_on_illegal") enable_illegal_ender();
        else if (a == "+end_on_rom_faults") enable_rom_fault_enders();
        else if (a == "+end_on_ifetch_berr") end_on_ifetch_berr = true;
        else if (a.rfind("+stuck_pc_threshold=",0) == 0)
            stuck_pc_threshold = parse_uint64_auto(a.substr(20), "+stuck_pc_threshold");
        else if (a == "+end_on_stuck_pc") end_on_stuck_pc = true;
        else if (a.rfind("+no_progress_cycles=",0) == 0)
            no_progress_cycle_threshold = parse_uint64_auto(a.substr(20), "+no_progress_cycles");
        else if (a == "+end_on_no_progress") end_on_no_progress = true;
        else if (a.rfind("+rom_patch=",0) == 0) parse_rom_patch_list(a.substr(11));
        else if (a == "+stop_on_low_pc")        stop_on_low_pc = true;
        else if (a == "+strict_overlay")        strict_overlay = true;
        else if (a == "+strict_axi_resp")       strict_axi_resp = true;
        else if (a.rfind("+via1_timer_div=",0) == 0) via1_timer_div = std::stoull(a.substr(16));
        else if (a == "+via1_fast_timing")      via1_timer_div = VIA1_TIMER_DIV_FAST;
        else if (a == "+via1_t1_selftest")      via1_t1_selftest = true;
        else if (a == "+harness_strictness_selftest") harness_strictness_selftest = true;
        else if (a == "+rtc_sidechannel_selftest") rtc_sidechannel_selftest = true;
        else if (a == "+adb_shadow_selftest")   adb_shadow_selftest = true;
        else if (a == "+scc_asc_selftest")      scc_asc_selftest = true;
        else if (a == "+scsi_window_selftest")  scsi_window_selftest = true;
        else if (a == "+display_watch_selftest") display_watch_selftest = true;
        else if (a == "+stop_on_exc_selftest") stop_on_exc_selftest = true;
        else if (a == "+watchdog_selftest")     watchdog_selftest = true;
        else if (a == "+rom_outcome_selftest")  rom_outcome_selftest = true;
        else if (a == "+waves")                 waves = true;
        else if (a == "+no_waves")              no_waves = true;
        else if (a == "+verbose")               verbose = true;
    }
    if (stop_pc_hit_target == 0) {
        std::fprintf(stderr, "[rom-boot] +stop_pc_hit must be >= 1\n");
        std::exit(2);
    }
    if (end_pc_hit_target == 0) {
        std::fprintf(stderr, "[rom-boot] +end_pc_hit must be >= 1\n");
        std::exit(2);
    }
    if (disk_prompt_hit_target == 0) {
        std::fprintf(stderr, "[rom-boot] +disk_prompt_hit must be >= 1\n");
        std::exit(2);
    }
    if (fault_dump_bytes > FAULT_DUMP_BYTES_MAX) {
        std::fprintf(stderr,
            "[rom-boot] +fault_dump_bytes=%llu exceeds max %llu\n",
            (unsigned long long)fault_dump_bytes,
            (unsigned long long)FAULT_DUMP_BYTES_MAX);
        std::exit(2);
    }
    if (!restore_state_path.empty() && !arch_replay_path.empty()) {
        std::fprintf(stderr,
            "[rom-boot] use either +restore_state or +arch_replay, not both\n");
        std::exit(2);
    }
#ifdef WAVES
    waves = true;
#endif
    if (no_waves) waves = false;
}

static bool init_periph_event_logger() {
    periph_events.set_log_path(periph_event_log_path);
    periph_events.set_detail_limit(periph_event_log_limit);
    periph_events.set_category_filter(periph_event_filter);
    periph_events.watch_category("VIA1");
    periph_events.watch_category("VIA2");
    periph_events.watch_category("ADB");
    periph_events.watch_category("VBL");
    periph_events.watch_category("RTC");
    periph_events.watch_category("PRAM");
    periph_events.watch_category("ASC");
    periph_events.watch_category("SCSI");
    periph_events.watch_category("SCC");
    periph_events.watch_category("DAFB");
    periph_events.watch_category("VRAM");

    if (!periph_events.open()) {
        std::fprintf(stderr,
            "[rom-boot] cannot open peripheral event log file %s\n",
            periph_event_log_path.c_str());
        return false;
    }
    if (!periph_event_log_path.empty()) {
        std::fprintf(stderr,
            "[rom-boot] peripheral model event log: %s (limit=%llu)\n",
            periph_event_log_path.c_str(),
            (unsigned long long)periph_event_log_limit);
    }
    if (!periph_event_filter.empty()) {
        std::fprintf(stderr, "[rom-boot] peripheral event filter:");
        for (const auto& category : periph_event_filter)
            std::fprintf(stderr, " %s", category.c_str());
        std::fprintf(stderr, "\n");
    }
    return true;
}

// ─── ROM loader ───────────────────────────────────────────────────────
// Task #139: the old `install_bootstrap()` trampoline lived here and
// planted a 6-byte `JMP #$0000002A.L` at 0x4080_0000 so the CPU's
// hard-wired RESET_PC would land on something that jumped into the
// real ROM.  That compromise is retired — mac_top is now elaborated
// with RESET_PC=0x4000_002A (Makefile `-GRESET_PC=...`), so the very
// first fetch is byte 0x2A of the real ROM image.
//
// Load the Q700 ROM into an in-TB buffer.  Classify()→AC_ROM routes all
// reads (fetch + daxi) through rom_read8/rom_offset_of, which means we
// don't need the ROM in mem_model at all.  This keeps mem_model's own
// RAM/ROM layout untouched (so tb_top.cpp's build remains undisturbed)
// and avoids the 1 M unmapped-write warnings we'd otherwise trigger.
static bool load_rom_and_overlay() {
    FILE* fp = std::fopen(rom_path.c_str(), "rb");
    if (!fp) {
        std::fprintf(stderr, "[rom-boot] cannot open %s\n", rom_path.c_str());
        return false;
    }
    std::fseek(fp, 0, SEEK_END);
    long fsz = std::ftell(fp);
    std::fseek(fp, 0, SEEK_SET);
    if (fsz <= 0 || fsz > (long)Q700_ROM_SIZE + 0x100) {
        std::fprintf(stderr, "[rom-boot] unexpected ROM size: %ld\n", fsz);
        std::fclose(fp);
        return false;
    }
    rom_img.assign((size_t)fsz, 0);
    if (std::fread(rom_img.data(), 1, (size_t)fsz, fp) != (size_t)fsz) {
        std::fprintf(stderr, "[rom-boot] short read on %s\n", rom_path.c_str());
        std::fclose(fp);
        return false;
    }
    std::fclose(fp);
    std::fprintf(stderr, "[rom-boot] loaded ROM %s (%ld bytes)\n",
                 rom_path.c_str(), fsz);

    // Mac Universal ROM header sanity — bytes [0..3] are the stored
    // checksum (MAME uses the first 4 bytes as the dump filename), and
    // bytes [4..7] are the reset-vector entry offset.  Parse them and
    // warn if either (a) the checksum doesn't match the Q700 value the
    // harness is tuned for, or (b) the entry offset doesn't match the
    // hard-coded bootstrap target.  Both are fatal to boot progress but
    // we keep running so sub-agents can still inspect the trace.
    if (rom_img.size() >= 8) {
        uint32_t hdr_cksum  = ((uint32_t)rom_img[0] << 24)
                            | ((uint32_t)rom_img[1] << 16)
                            | ((uint32_t)rom_img[2] <<  8)
                            |  (uint32_t)rom_img[3];
        uint32_t hdr_entry  = ((uint32_t)rom_img[4] << 24)
                            | ((uint32_t)rom_img[5] << 16)
                            | ((uint32_t)rom_img[6] <<  8)
                            |  (uint32_t)rom_img[7];
        std::fprintf(stderr,
            "[rom-boot] ROM header: checksum=0x%08x entry=0x%08x\n",
            hdr_cksum, hdr_entry);
        if (hdr_cksum != Q700_ROM_CHECKSUM) {
            std::fprintf(stderr,
                "[rom-boot] WARN: ROM checksum 0x%08x != Q700 Universal "
                "0x%08x — harness peripheral stubs are Q700-specific; "
                "execution past reset-vector is unlikely to progress.\n",
                hdr_cksum, Q700_ROM_CHECKSUM);
        }
        if (hdr_entry != Q700_ROM_ENTRY) {
            std::fprintf(stderr,
                "[rom-boot] WARN: ROM reset-entry 0x%08x != expected "
                "0x%08x — mac_top was elaborated with RESET_PC=0x%08x, "
                "so the CPU will not land on this ROM's actual entry "
                "and execution will almost certainly diverge.\n",
                hdr_entry, Q700_ROM_ENTRY, Q700_RESET_PC);
        }
    } else {
        std::fprintf(stderr,
            "[rom-boot] WARN: ROM image <8 bytes, no header to parse\n");
    }
    if (!apply_rom_patches()) return false;
    return true;
}

// ─── Stuck-PC watchdog ────────────────────────────────────────────────
// Real-Mac boot code ends its cold-boot self-test with one of
//   * a "bra ." trap (if sad-mac path hit),
//   * a branch into high-addr boot-ROM code (happy path),
//   * an RTE into System ROM proper.
// We detect "stuck" as PC unchanged over N successive commits with
// no new commits seen — classic self-wait loop.  Also detect the same
// PC appearing K times in a row (tight bra. / bra loop).
// ─── Final diagnostics dump ───────────────────────────────────────────
static void dump_final_state() {
    if (!dut) return;
    auto* r = dut->rootp;
    const uint32_t last_pc = (uint32_t)dut->dbg_last_pc;
    const uint32_t next_fetch = (uint32_t)dut->dbg_pc;
    const uint32_t fault_pc =
        (uint32_t)r->mac_top__DOT__cpu__DOT__u_exception__DOT__cur_fault_pc;
    const uint32_t rob_pc = (uint32_t)r->mac_top__DOT__cpu__DOT__rob_pc;
    // Macro-IPC metric (task #192): committed-macro-instructions / cycles.
    // dbg_macros counts only retired µops with last_phase=1 → true macro count.
    const unsigned long long rb_cycles  = (unsigned long long)sim_time;
    const unsigned           rb_uops    = (unsigned)dut->dbg_committed;
    const unsigned           rb_macros  = (unsigned)dut->dbg_macros;
    const double rb_uop_ipc   = rb_cycles ? (double)rb_uops   / (double)rb_cycles : 0.0;
    const double rb_macro_ipc = rb_cycles ? (double)rb_macros / (double)rb_cycles : 0.0;

    std::fprintf(stderr,
        "\n──────── rom-boot final state ────────\n"
        "  reason:     %s\n"
        "  cycles:     %llu\n"
        "  committed:  %u\n"
        "  macros:     %u\n"
        "  uop_ipc:    %.4f\n"
        "  macro_ipc:  %.4f\n"
        "  last_pc:    0x%08x\n"
        "  next_fetch: 0x%08x\n"
        "  overlay:    %s\n",
        stop_reason,
        rb_cycles,
        rb_uops,
        rb_macros,
        rb_uop_ipc,
        rb_macro_ipc,
        (unsigned)last_pc,
        (unsigned)next_fetch,
        overlay ? "1 (still asserted)" : "0 (ROM cleared it)"
    );
    std::fprintf(stderr,
        "[METRICS] rom-boot cycles=%llu committed_uops=%u committed_macros=%u "
        "uop_ipc=%.4f macro_ipc=%.4f\n",
        rb_cycles, rb_uops, rb_macros, rb_uop_ipc, rb_macro_ipc);
    dump_instr_bytes_line(stderr, "last_pc", last_pc);
    if (next_fetch != 0)
        dump_instr_bytes_line(stderr, "next_fetch", next_fetch);
    if (fault_pc != 0 && fault_pc != last_pc && fault_pc != next_fetch)
        dump_instr_bytes_line(stderr, "fault_pc", fault_pc);
    if (rob_pc != 0 && rob_pc != last_pc && rob_pc != next_fetch &&
        rob_pc != fault_pc) {
        dump_instr_bytes_line(stderr, "rob_pc", rob_pc);
    }
    // Arch regs from committed RAT (same trick as tb_top.cpp).
    unsigned ccr = r->mac_top__DOT__cpu__DOT__u_ccr_rat__DOT__ccr_prf
                     [r->mac_top__DOT__cpu__DOT__u_ccr_rat__DOT__crat_tag];
    std::fprintf(stderr, "  ccr:        0x%02x (X=%d N=%d Z=%d V=%d C=%d)\n",
        ccr & 0x1F,
        (ccr >> 4) & 1, (ccr >> 3) & 1, (ccr >> 2) & 1,
        (ccr >> 1) & 1,  ccr        & 1);
    const unsigned arch_sr =
        (unsigned)((r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_sr & 0xffe0u)
                   | (ccr & 0x1fu));
    std::fprintf(stderr,
        "  control:    sr=0x%04x vbr=0x%08x usp=0x%08x ssp=0x%08x "
        "isp=0x%08x cacr=0x%08x sfc=0x%08x dfc=0x%08x\n",
        arch_sr,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_vbr,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__usp,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__ssp,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__isp,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_cacr,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_sfc,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__arch_dfc);
    for (int i = 0; i < 8; i++) {
        unsigned phys = r->mac_top__DOT__cpu__DOT__u_rat__DOT__crat[i];
        unsigned v    = r->mac_top__DOT__cpu__DOT__prf[phys];
        std::fprintf(stderr, "  d%d = 0x%08x\n", i, v);
    }
    for (int i = 0; i < 8; i++) {
        unsigned phys = r->mac_top__DOT__cpu__DOT__u_rat__DOT__crat[8 + i];
        unsigned v    = r->mac_top__DOT__cpu__DOT__prf[phys];
        std::fprintf(stderr, "  a%d = 0x%08x\n", i, v);
    }
    std::fprintf(stderr,
        "  q700_descriptor_selected=%d entry=0x%08x feature=0x%08x\n",
        q700_descriptor_selected ? 1 : 0,
        q700_descriptor_entry,
        q700_feature_word);
    if (ramtest_mame_state_fastfill) {
        std::fprintf(stderr,
            "  ramtest_mame_state_fastfill: hits=%llu skips=%llu bytes=%llu sentinels=%llu\n",
            (unsigned long long)ramtest_mame_state_fastfill_hits,
            (unsigned long long)ramtest_mame_state_fastfill_skips,
            (unsigned long long)ramtest_mame_state_fastfill_bytes,
            (unsigned long long)ramtest_mame_state_fastfill_sentinels);
    }
    // VIA1 snapshot.
    std::fprintf(stderr,
        "  via1.orb=0x%02x ddrb=0x%02x ifr=0x%02x ier=0x%02x acr=0x%02x\n",
        via1.orb, via1.ddrb, via1.ifr, via1.ier, via1.acr);
    std::fprintf(stderr,
        "  watchdogs: stuck_pc_threshold=%llu same_pc_repeats=%u "
        "no_progress_cycles=%llu\n",
        (unsigned long long)stuck_pc_threshold,
        last_same_pc_count,
        (unsigned long long)no_progress_cycle_threshold);
    std::fprintf(stderr,
        "  if: pc=0x%08x if_req=%u if_addr=0x%08x if_pending=%d if_addr_q=0x%08x\n"
        "      if_req_s=%u if_addr_va=0x%08x if_addr_pa=0x%08x if_rvalid_s=%u\n"
        "      pd_valid=%u pd_consumed=%u redirect=%u->0x%08x pred=%u rdir=%u->0x%08x\n"
        "      rn_ready=%u d_valid=%u d_branch=%u d_rts=%u d_op=%u d_npc=0x%08x bpu_hit=%u pred_target=0x%08x\n"
        "      l0=%u@0x%07x l1=%u@0x%07x lv=%u@0x%07x flush_resp=%u req_slot=%u\n",
        (unsigned)r->mac_top__DOT__cpu__DOT__u_if__DOT__pc,
        (unsigned)dut->if_req,
        (unsigned)dut->if_addr,
        if_pending,
        (unsigned)if_addr_q,
        (unsigned)r->mac_top__DOT__cpu__DOT__if_req_s,
        (unsigned)r->mac_top__DOT__cpu__DOT__if_addr_va,
        (unsigned)r->mac_top__DOT__cpu__DOT__if_addr_pa,
        (unsigned)r->mac_top__DOT__cpu__DOT__if_rvalid_s,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_if__DOT__pd_valid_w,
        (unsigned)r->mac_top__DOT__cpu__DOT__pd_consumed,
        (unsigned)r->mac_top__DOT__cpu__DOT__redirect_en,
        (unsigned)r->mac_top__DOT__cpu__DOT__redirect_pc,
        (unsigned)r->mac_top__DOT__cpu__DOT__pred_redirect,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_if__DOT__rdir_any,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_if__DOT__rdir_target,
        (unsigned)r->mac_top__DOT__cpu__DOT__rn_ready,
        (unsigned)r->mac_top__DOT__cpu__DOT__d_uop_valid,
        (unsigned)r->mac_top__DOT__cpu__DOT__d_is_branch,
        (unsigned)r->mac_top__DOT__cpu__DOT__d_is_rts,
        (unsigned)r->mac_top__DOT__cpu__DOT__d_uop_op,
        (unsigned)r->mac_top__DOT__cpu__DOT__d_uop_npc,
        (unsigned)r->mac_top__DOT__cpu__DOT__bpu_hit,
        (unsigned)r->mac_top__DOT__cpu__DOT__pred_target_sel,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_if__DOT__l0_valid,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_if__DOT__l0_addr,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_if__DOT__l1_valid,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_if__DOT__l1_addr,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_if__DOT__lv_valid,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_if__DOT__lv_addr,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_if__DOT__flush_resp,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_if__DOT__req_slot);
    const unsigned rob_head =
        r->mac_top__DOT__cpu__DOT__u_rob__DOT__head_ptr & 0x3f;
    const unsigned rob_tail =
        r->mac_top__DOT__cpu__DOT__u_rob__DOT__tail_ptr & 0x3f;
    const unsigned rob_count = (rob_tail - rob_head) & 0x3f;
    const bool rob_full =
        ((rob_head & 0x1f) == (rob_tail & 0x1f)) &&
        (((rob_head ^ rob_tail) & 0x20) != 0);
    const bool rob_empty = (rob_head == rob_tail);
    const unsigned int_iq_count =
        r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__cnt_r & 0xf;
    const unsigned mem_iq_count =
        r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__cnt_r & 0xf;
    const unsigned rat_free =
        r->mac_top__DOT__cpu__DOT__u_rat__DOT__free_cnt_r & 0x3f;
    const unsigned fq0_valid =
        r->mac_top__DOT__cpu__DOT__fq_valid[0] ? 1u : 0u;
    const unsigned fq1_valid =
        r->mac_top__DOT__cpu__DOT__fq_valid[1] ? 1u : 0u;
    const unsigned fq0_src =
        r->mac_top__DOT__cpu__DOT__fq_src[0] & 0x3u;
    const unsigned fq1_src =
        r->mac_top__DOT__cpu__DOT__fq_src[1] & 0x3u;
    const unsigned fq_empty = (!fq0_valid && !fq1_valid) ? 1u : 0u;
    const unsigned exc_flush_done =
        (r->mac_top__DOT__cpu__DOT__dcache_flush_done_raw &&
         fq0_valid && fq0_src == 0) ? 1u : 0u;
    std::fprintf(stderr,
        "  dispatch gates: rob_full=%u rob_empty=%u rob_count=%u "
        "int_iq_count=%u mem_iq_count=%u flush_en=%u\n"
        "      alloc_ok=%u rat_free=%u ccr_alloc_ok=%u "
        "wants_mem=%u d_type=%u d_has_dst=%u d_flags_wr=0x%02x iq_int_mul_busy=%u\n"
        "      rob_head: valid=%u complete=%u pc=0x%08x pop=%u head=%u tail=%u\n"
        "      commit: can=%u in_flight=%u exc_wait=%u exc_active=%u "
        "cache_wait=%u lsu_busy=%u take_exc=%u take_rte=%u take_maint=%u take_irq=%u\n"
        "      branch: is=%u taken=%u target=0x%08x fall=0x%08x actual=0x%08x\n"
        "      exception: state=%u vec=%u fault_pc=0x%08x fault_addr=0x%08x "
        "a7_new=0x%08x dc_req=%u dc_wr=%u dc_addr=0x%08x dc_wstrb=0x%x "
        "dc_rv=%u dc_bv=%u\n"
        "      exc-gate: pend_exc=%u pend_rte=%u flush_req=%u flush_done=%u "
        "dc_busy=%u dc_done=%u fq_empty=%u fq0=%u/%u/%u fq1=%u/%u/%u\n"
        "      dcache: state=%u fa_set=%u fa_way=%u victim_way=%u beat=%u "
        "aw=%u w=%u bready=%u ar=%u rready=%u\n"
        "      daxi: ar_out=%u ar_addr=0x%08x ar_delay=%d aw_done=%u "
        "w_done=%u aw_addr=0x%08x b_pending=%u b_delay=%d\n"
        "      iq_int: disp_ready=%u occ=%u has_free=%u free_idx=%u "
        "sel_mask=0x%02x sel_valid=%u sel_idx=%u iss_valid=%u\n"
        "      iq_mem: disp_ready=%u occ=%u has_free=%u free_idx=%u "
        "sel_mask=0x%02x sel_valid=%u sel_idx=%u iss_valid=%u "
        "pending=%u/%u blocked_valid=0x%02x older_valid=0x%02x\n",
        rob_full ? 1u : 0u,
        rob_empty ? 1u : 0u,
        rob_count,
        int_iq_count,
        mem_iq_count,
        (unsigned)r->mac_top__DOT__cpu__DOT__flush_en,
        (unsigned)r->mac_top__DOT__cpu__DOT__alloc_ok,
        rat_free,
        (unsigned)r->mac_top__DOT__cpu__DOT__ccr_alloc_ok,
        (unsigned)r->mac_top__DOT__cpu__DOT__wants_mem,
        (unsigned)r->mac_top__DOT__cpu__DOT__d_uop_type,
        (unsigned)r->mac_top__DOT__cpu__DOT__d_has_dst,
        (unsigned)r->mac_top__DOT__cpu__DOT__d_flags_wr,
        (unsigned)r->mac_top__DOT__cpu__DOT__iq_int_mul_busy,
        (unsigned)r->mac_top__DOT__cpu__DOT__rob_v,
        (unsigned)r->mac_top__DOT__cpu__DOT__rob_c,
        (unsigned)r->mac_top__DOT__cpu__DOT__rob_pc,
        (unsigned)r->mac_top__DOT__cpu__DOT__rob_pop,
        rob_head,
        rob_tail,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__can_commit,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__commit_in_flight,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__exc_wait,
        (unsigned)r->mac_top__DOT__cpu__DOT__exc_active,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__cache_maint_wait,
        (unsigned)r->mac_top__DOT__cpu__DOT__lsu_busy,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__take_exc,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__take_rte,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__take_cache_maint,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__take_irq,
        (unsigned)r->mac_top__DOT__cpu__DOT__rob_isb,
        (unsigned)r->mac_top__DOT__cpu__DOT__rob_brt,
        (unsigned)r->mac_top__DOT__cpu__DOT__rob_brtgt,
        (unsigned)r->mac_top__DOT__cpu__DOT__rob_npc_fall,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_commit__DOT__actual_next,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_exception__DOT__state,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_exception__DOT__cur_vec,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_exception__DOT__cur_fault_pc,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_exception__DOT__cur_fault_addr,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_exception__DOT__cur_a7_new,
        (unsigned)r->mac_top__DOT__cpu__DOT__exc_dc_req,
        (unsigned)r->mac_top__DOT__cpu__DOT__exc_dc_is_write,
        (unsigned)r->mac_top__DOT__cpu__DOT__exc_dc_addr,
        (unsigned)r->mac_top__DOT__cpu__DOT__exc_dc_wstrb,
        (unsigned)r->mac_top__DOT__cpu__DOT__dc_rvalid,
        (unsigned)r->mac_top__DOT__cpu__DOT__dc_bvalid,
        (unsigned)r->mac_top__DOT__cpu__DOT__exc_gate_pending_exc,
        (unsigned)r->mac_top__DOT__cpu__DOT__exc_gate_pending_rte,
        (unsigned)r->mac_top__DOT__cpu__DOT__exc_flush_req,
        exc_flush_done,
        (unsigned)(r->mac_top__DOT__cpu__DOT__u_dcache__DOT__state >= 11),
        (unsigned)r->mac_top__DOT__cpu__DOT__dcache_flush_done_raw,
        fq_empty,
        fq0_valid,
        fq0_src,
        (unsigned)r->mac_top__DOT__cpu__DOT__fq_inval[0],
        fq1_valid,
        fq1_src,
        (unsigned)r->mac_top__DOT__cpu__DOT__fq_inval[1],
        (unsigned)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__state,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__fa_set,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__fa_way,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__victim_way,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__beat_cnt,
        (unsigned)r->mac_top__DOT__cpu__DOT__dc_daxi_awvalid,
        (unsigned)r->mac_top__DOT__cpu__DOT__dc_daxi_wvalid,
        (unsigned)r->mac_top__DOT__cpu__DOT__dc_daxi_bready,
        (unsigned)r->mac_top__DOT__cpu__DOT__dc_daxi_arvalid,
        (unsigned)r->mac_top__DOT__cpu__DOT__dc_daxi_rready,
        ax.ar_outstanding ? 1u : 0u,
        ax.ar_addr,
        ax.ar_delay,
        ax.aw_done ? 1u : 0u,
        ax.w_done ? 1u : 0u,
        ax.aw_addr,
        ax.b_pending ? 1u : 0u,
        ax.b_delay,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__disp_ready_reg,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__occ_ctr,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__has_free,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__free_idx,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__sel_mask,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__sel_valid,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__sel_idx,
        (unsigned)r->mac_top__DOT__cpu__DOT__int_iss_v,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__disp_ready_reg,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__occ_ctr,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__has_free,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__free_idx,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__sel_mask,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__sel_valid,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__sel_idx,
        (unsigned)r->mac_top__DOT__cpu__DOT__mem_iss_v,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__pending_insert_valid,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__pending_insert_idx,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__blocked_by_valid,
        (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__older_than_valid);
    std::fprintf(stderr, "  iq_int entries:\n");
    for (int i = 0; i < 8; i++) {
        std::fprintf(stderr,
            "      [%d] v=%u pc=0x%08x op=%u br=%u tag=%u "
            "psa=%u/%u psb=%u/%u ccr=%u/%u frd=0x%02x fwr=0x%02x "
            "pdst=%u hd=%u imm_v=%u imm=0x%08x\n",
            i,
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_valid[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_uop_pc[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_uop_op[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_is_branch[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_rob_tag[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_psa[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_psa_rdy[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_psb[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_psb_rdy[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_ccr_src[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_ccr_rdy[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_frd[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_fwr[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_pdst[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_has_dst[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_imm_v[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_int__DOT__e_imm[i]);
    }
    std::fprintf(stderr, "  iq_mem entries:\n");
    for (int i = 0; i < 8; i++) {
        const unsigned bit = 1u << i;
        std::fprintf(stderr,
            "      [%d] v=%u st=%u tag=%u size=%u "
            "base=%u/%u data=%u/%u pdst=%u hd=%u disp=0x%08x "
            "ccr=%u/%u imm_data=%u rts=%u block=0x%02x older=0x%02x\n",
            i,
            (unsigned)((r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__e_valid >> i) & 1u),
            (unsigned)((r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__e_is_store >> i) & 1u),
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__e_rob_tag[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__e_size[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__e_pbase[i],
            (unsigned)((r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__e_base_rdy & bit) ? 1u : 0u),
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__e_pdata[i],
            (unsigned)((r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__e_data_rdy & bit) ? 1u : 0u),
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__e_pdst[i],
            (unsigned)((r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__e_has_dst >> i) & 1u),
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__e_disp[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__e_ccr_src[i],
            (unsigned)((r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__e_ccr_rdy & bit) ? 1u : 0u),
            (unsigned)((r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__e_imm_is_data >> i) & 1u),
            (unsigned)((r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__e_is_rts >> i) & 1u),
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__blocked_by[i],
            (unsigned)r->mac_top__DOT__cpu__DOT__u_iq_mem__DOT__older_than[i]);
    }
    // Unmapped-access scoreboard (top 10).
    if (!unmapped_read_count.empty() || !unmapped_write_count.empty()) {
        std::fprintf(stderr, "  top-hit unmapped addresses:\n");
        std::vector<std::pair<uint64_t,uint32_t>> rd, wr;
        for (auto& kv : unmapped_read_count)  rd.push_back({kv.second, kv.first});
        for (auto& kv : unmapped_write_count) wr.push_back({kv.second, kv.first});
        auto top = [&](std::vector<std::pair<uint64_t,uint32_t>>& v,
                       const char* tag) {
            std::sort(v.begin(), v.end(),
                [](const auto& a, const auto& b){ return a.first > b.first; });
            for (size_t i = 0; i < v.size() && i < 10; i++)
                std::fprintf(stderr, "    %s 0x%08x x%llu\n",
                             tag, v[i].second,
                             (unsigned long long)v[i].first);
        };
        top(rd, "rd");
        top(wr, "wr");
    }
    std::fprintf(stderr, "──────────────────────────────────────\n");
}

static uint64_t progress_signature() {
    if (!dut) return 0;
    auto* r = dut->rootp;
    uint64_t sig = 0xcbf29ce484222325ULL;
    auto mix = [&](uint64_t v) {
        sig ^= v;
        sig *= 0x100000001b3ULL;
    };

    mix((uint32_t)dut->dbg_committed);
    mix((uint32_t)dut->dbg_pc);
    mix((uint32_t)dut->dbg_last_pc);
    mix((uint32_t)dut->if_addr);
    mix((uint32_t)dut->if_req);
    mix((uint32_t)dut->if_rvalid);
    mix((uint32_t)dut->daxi_awvalid);
    mix((uint32_t)dut->daxi_wvalid);
    mix((uint32_t)dut->daxi_bready);
    mix((uint32_t)dut->daxi_bvalid);
    mix((uint32_t)dut->daxi_arvalid);
    mix((uint32_t)dut->daxi_rready);
    mix((uint32_t)dut->daxi_rvalid);
    mix((uint32_t)ax.ar_outstanding);
    mix(ax.ar_addr);
    mix((uint32_t)ax.ar_delay);
    mix((uint32_t)ax.aw_done);
    mix((uint32_t)ax.w_done);
    mix(ax.aw_addr);
    mix((uint32_t)ax.b_pending);
    mix((uint32_t)ax.b_delay);

    mix((uint32_t)r->mac_top__DOT__cpu__DOT__u_exception__DOT__state);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__u_exception__DOT__cur_vec);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__u_commit__DOT__exc_wait);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__exc_active);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__exc_gate_pending_exc);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__exc_gate_pending_rte);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__exc_flush_req);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__dcache_flush_done_raw);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__fq_valid[0]);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__fq_src[0]);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__fq_inval[0]);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__fq_valid[1]);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__fq_src[1]);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__fq_inval[1]);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__state);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__fa_set);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__fa_way);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__victim_way);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__u_dcache__DOT__beat_cnt);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__dc_daxi_awvalid);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__dc_daxi_wvalid);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__dc_daxi_bready);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__dc_daxi_arvalid);
    mix((uint32_t)r->mac_top__DOT__cpu__DOT__dc_daxi_rready);

    return sig;
}

// ─── main ────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    bool arch_checkpoint_failed = false;
    parse_args(argc, argv);
    if (via1_t1_selftest) {
        bool ok = run_via1_t1_selftest();
        return ok ? 0 : 1;
    }
    if (harness_strictness_selftest) {
        bool ok = run_harness_strictness_selftest();
        return ok ? 0 : 1;
    }
    if (rtc_sidechannel_selftest) {
        bool ok = run_rtc_sidechannel_selftest();
        return ok ? 0 : 1;
    }
    if (adb_shadow_selftest) {
        bool ok = run_adb_shadow_selftest();
        return ok ? 0 : 1;
    }
    if (scc_asc_selftest) {
        bool ok = run_scc_asc_selftest();
        return ok ? 0 : 1;
    }
    if (scsi_window_selftest) {
        bool ok = run_scsi_window_selftest();
        return ok ? 0 : 1;
    }
    if (display_watch_selftest) {
        bool ok = run_display_watch_selftest();
        return ok ? 0 : 1;
    }
    if (stop_on_exc_selftest) {
        bool ok = run_stop_on_exc_selftest();
        return ok ? 0 : 1;
    }
    if (watchdog_selftest) {
        bool ok = run_watchdog_selftest();
        return ok ? 0 : 1;
    }
    if (rom_outcome_selftest) {
        bool ok = run_rom_outcome_selftest();
        return ok ? 0 : 1;
    }
    init_lastn_trace();
    std::fprintf(stderr, "[rom-boot] Q700 cold-boot harness starting\n");
    std::fprintf(stderr,
        "[rom-boot] harness strictness: overlay=%s daxi_resp=%s via1_timer_div=%llu\n",
        strict_overlay ? "strict" : "legacy",
        strict_axi_resp ? "strict" : "legacy",
        (unsigned long long)via1_timer_div);
    print_stop_pc_config();
    print_end_pc_config();
    print_rom_outcome_config();
    print_stop_exc_config();
    print_end_exc_config();
    print_watchdog_config();
    print_fault_observability_config();

    mem = new MemModel();
    // ROM boot follows the canonical map (ROM at 0x4000_0000).
    mem->enable_legacy_rom_alias(false);
    if (!load_rom_and_overlay()) {
        std::fprintf(stderr, "[rom-boot] ROM load failed — aborting\n");
        return 1;
    }
    std::fprintf(stderr,
        "[rom-boot] RESET_PC=0x%08x (ROM@0x%08x + entry 0x%x, native Q700 path)\n"
        "[rom-boot] overlay=1 (low 256 KB aliased to ROM)\n"
        "[rom-boot] visible low RAM: 0x%08x bytes\n",
        Q700_RESET_PC, Q700_ROM_BASE, Q700_ROM_ENTRY,
        (unsigned)visible_ram_size);

    trace_fp = std::fopen(trace_path.c_str(), "w");
    if (!trace_fp) {
        std::fprintf(stderr, "[rom-boot] cannot open trace file %s\n",
                     trace_path.c_str());
    } else {
        std::fprintf(trace_fp, "# <pc> <ir> <ccr>  — Q700 ROM cold boot\n");
        std::fprintf(trace_fp, "# ROM=%s reset_pc=0x%08x entry=0x%08x\n",
                     rom_path.c_str(), Q700_RESET_PC, Q700_ROM_ENTRY);
    }
    if (!periph_log_path.empty()) {
        periph_fp = std::fopen(periph_log_path.c_str(), "w");
        if (!periph_fp) {
            std::fprintf(stderr,
                "[rom-boot] cannot open peripheral log file %s\n",
                periph_log_path.c_str());
        } else {
            std::fprintf(periph_fp,
                "# rom-boot peripheral activity log limit=%llu\n",
                (unsigned long long)periph_log_limit);
            std::fprintf(stderr,
                "[rom-boot] peripheral log: %s (limit=%llu)\n",
                periph_log_path.c_str(),
                (unsigned long long)periph_log_limit);
        }
    }
    if (!probe_log_path.empty()) {
        probe_fp = std::fopen(probe_log_path.c_str(), "w");
        if (!probe_fp) {
            std::fprintf(stderr,
                "[rom-boot] cannot open probe log file %s\n",
                probe_log_path.c_str());
        } else {
            std::fprintf(probe_fp,
                "# rom-boot ROM frontier probe log limit=%llu\n",
                (unsigned long long)probe_log_limit);
            std::fprintf(stderr,
                "[rom-boot] ROM frontier probe log: %s (limit=%llu)\n",
                probe_log_path.c_str(),
                (unsigned long long)probe_log_limit);
        }
    }
    if (!init_data_watch_logger()) return 1;
    if (!init_cache_event_logger()) return 1;
    (void)init_periph_event_logger();
    if (!init_arch_sample_logger()) return 1;

    dut = new Vmac_top;
    if (waves) {
        Verilated::traceEverOn(true);
        fst = new VerilatedFstC;
        dut->trace(fst, 99);
        fst->open("rom_boot.fst");
        std::fprintf(stderr, "[rom-boot] waveform: rom_boot.fst\n");
    }

    if (!restore_state_path.empty()) {
        if (!restore_checkpoint(restore_state_path)) {
            std::fprintf(stderr, "[rom-boot] restore failed — aborting\n");
            return 1;
        }
        if (!active_rom_patches.empty()) {
            std::fprintf(stderr,
                "[rom-boot] reapplying harness ROM patches after checkpoint restore\n");
            if (!apply_active_rom_patches()) return 1;
        }
    } else if (!arch_replay_path.empty()) {
        reset_bus();
        if (!apply_arch_replay(arch_replay_path)) {
            std::fprintf(stderr, "[rom-boot] arch replay failed — aborting\n");
            return 1;
        }
    } else {
        reset_bus();
    }
    if (absolute_stop_cycle != 0) {
        stop_cycle = absolute_stop_cycle;
    } else {
        stop_cycle = sim_time + timeout_cycles;
    }
    if (checkpoint_every != 0) {
        uint64_t committed = (uint64_t)dut->dbg_committed;
        next_checkpoint_commit = committed + checkpoint_every;
    }
    while (next_checkpoint_point < checkpoint_points.size() &&
           checkpoint_points[next_checkpoint_point] <=
               (uint64_t)dut->dbg_committed) {
        next_checkpoint_point++;
    }
    while (next_checkpoint_cycle_point < checkpoint_cycle_points.size() &&
           checkpoint_cycle_points[next_checkpoint_cycle_point] <= sim_time) {
        next_checkpoint_cycle_point++;
    }

    precise_boundary_pending = false;
    precise_boundary_stop = BoundaryEvent{};
    uint32_t prev_committed = 0;
    if (!restore_state_path.empty() || !arch_replay_path.empty())
        prev_committed = (uint32_t)dut->dbg_committed;
    if (arch_sample_every != 0) {
        write_arch_sample((uint64_t)dut->dbg_boundary_seq, sim_time);
        next_arch_sample_commit =
            (uint64_t)dut->dbg_boundary_seq + arch_sample_every;
    }
    uint32_t prev_boundary_seq = dut ? (uint32_t)dut->dbg_boundary_seq : 0;
    uint64_t no_progress_cycles = 0;
    uint64_t prev_progress_sig = progress_signature();

    while (!test_done && !Verilated::gotFinish()) {
        drive_ifetch();
        drive_daxi();
        tick();
        flush_pending_arch_sample();
        sample_internal_dafb_activity();
        sample_cache_frontier_activity();
        sample_lastn_trace();
        BoundaryEvent boundary_ev;
        const bool have_boundary = poll_new_boundary_event(prev_boundary_seq, boundary_ev);
        bool stopped_on_exception = false;
        if (have_boundary) {
            arch_next_pc_valid = true;
            arch_next_pc = boundary_ev.next_pc;
            arch_next_pc_commit_pc = boundary_ev.pc;
            arch_next_pc_committed = (uint32_t)dut->dbg_committed;
            if (boundary_ev.kind == DBG_BOUNDARY_EXC) {
                stopped_on_exception =
                    check_stop_on_exception_boundary(boundary_ev);
                if (!stopped_on_exception) {
                    stopped_on_exception =
                        check_end_on_exception_boundary(boundary_ev);
                }
            }
            if (!test_done && boundary_ev.kind == DBG_BOUNDARY_RETIRE) {
                maybe_arch_sample((uint64_t)boundary_ev.seq);
                if (check_end_pc(boundary_ev.pc) ||
                    check_stop_pc(boundary_ev.pc)) {
                    arm_precise_boundary_stop(boundary_ev);
                }
            }
        }

        uint32_t c = (uint32_t)dut->dbg_committed;
        uint64_t progress_sig = progress_signature();
        if (stopped_on_exception) {
            no_progress_cycles = 0;
        } else if (c != prev_committed) {
            // New commit.  dbg_last_pc is the PC of the just-retired uop.
            uint32_t pc = (uint32_t)dut->dbg_last_pc;
            auto* r = dut->rootp;
            unsigned ccr = r->mac_top__DOT__cpu__DOT__u_ccr_rat__DOT__ccr_prf
                             [r->mac_top__DOT__cpu__DOT__u_ccr_rat__DOT__crat_tag];
            log_trace(pc, (uint8_t)ccr);
            maybe_log_rom_probe(pc);
            maybe_apply_ramtest_mame_state_fastfill(pc);
            if (!q700_descriptor_selected && ((pc & 0x00FFFFFFu) == 0x00002F96u)) {
                uint32_t d1 = read_arch_reg(1);
                uint32_t d2 = read_arch_reg(2);
                uint32_t a1 = read_arch_reg(9);
                uint32_t entry_off = a1 & 0x000FFFFFu;

                // Entries 0x390c and 0x388c both point at the Q700 hardware
                // descriptor.  Reaching 0x2f96 means the ROM's type and
                // feature checks have accepted the current table entry.
                if ((d2 & 0xFFu) == 0x08u &&
                    (entry_off == 0x0000390Cu || entry_off == 0x0000388Cu)) {
                    q700_descriptor_selected = true;
                    q700_descriptor_entry = a1;
                    q700_feature_word = d1;
                    std::fprintf(stderr,
                        "[rom-boot] Q700 descriptor table entry accepted "
                        "(entry=0x%08x feature=0x%08x)\n",
                        q700_descriptor_entry,
                        q700_feature_word);
                    if (trace_fp) {
                        std::fprintf(trace_fp,
                            "# Q700 descriptor accepted entry=0x%08x feature=0x%08x\n",
                            q700_descriptor_entry,
                            q700_feature_word);
                    }
                }
            }
            if (!test_done)
                check_rom_outcome_commit(pc);
            if (!test_done)
                check_stuck_pc_commit(pc);
            prev_committed = c;
            no_progress_cycles = 0;
            maybe_periodic_checkpoint();

            if (!test_done && (uint64_t)c >= max_insts) {
                test_done = true;
                stop_reason = "max-insts-reached";
            }
        } else if (progress_sig != prev_progress_sig) {
            no_progress_cycles = 0;
        } else {
            no_progress_cycles++;
            check_no_progress_watchdog(no_progress_cycles);
        }
        prev_progress_sig = progress_sig;
        if (!test_done && stop_on_low_pc && !overlay &&
            dut->dbg_pc != 0 && dut->dbg_pc < 0x10000000u) {
            test_done = true;
            stop_reason = "low-pc-fetch";
        }
        maybe_cycle_checkpoint();
    }

    dump_lastn_trace();
    dump_fault_windows();
    if (trace_fp) std::fclose(trace_fp);
    if (probe_fp) std::fclose(probe_fp);

    if (!arch_checkpoint_path.empty())
        arch_checkpoint_failed = !save_arch_checkpoint(arch_checkpoint_path);

    dump_display_summary();
    dump_periph_summary();
    dump_data_watch_summary();
    dump_cache_event_summary();
    periph_events.dump_summary(stderr);
    dump_final_state();
    if (!save_state_path.empty()) save_checkpoint(save_state_path);

    if (waves) {
        fst->close();
        delete fst;
    }
    dut->final();
    delete dut;
    delete mem;
    if (periph_fp && periph_fp != stderr) std::fclose(periph_fp);
    if (arch_sample_fp) std::fclose(arch_sample_fp);
    if (data_watch_fp && data_watch_fp != stderr) std::fclose(data_watch_fp);
    if (cache_event_fp && cache_event_fp != stderr) std::fclose(cache_event_fp);
    periph_events.close();
    // Harness returns 0 on clean termination — the purpose is bring-up
    // diagnostics, not pass/fail gating.  Trace file + diagnostics are the
    // deliverable.  Non-zero only on setup/output errors such as a requested
    // checkpoint file that could not be written.
    return arch_checkpoint_failed ? 1 : 0;
}
