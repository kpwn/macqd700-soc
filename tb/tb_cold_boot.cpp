// tb_cold_boot.cpp — cold-boot harness for the real RTL path.
//
// Coverage:
//   * Phase A: synthetic SD image -> boot_fsm -> ddr_ctrl -> first fetch
//   * Phase B: real Q700 ROM copy + bounded first execution window

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <string>
#include <vector>
#include <verilated.h>
#include "Vtb_cold_boot.h"

static Vtb_cold_boot* dut = nullptr;
static uint64_t sim_time = 0;
static int n_pass = 0, n_fail = 0;

static const uint32_t RESET_PC        = 0x4000002Au;
static const uint32_t RESET_JMP_TGT   = 0x4000008Cu;
static const uint32_t FULL_ROM_SECTORS = 2048;
// SIM_MODEL boot via the SD/boot_fsm path completes well under 500K cycles;
// 1M is 2× safety.  Was 7M — that was sized for paranoid first-light bringup
// and made every legitimate-timeout test wait 7s, which adds up to ~30-90s
// agent stalls across the 12 scenarios.  The Q700 ROM run keeps a separate
// post-boot wait loop (see test_q700_rom).
static const uint64_t FULL_BOOT_MAX_CYCLES = 1000000;
static const uint32_t PERIPH_ENTRY_PC = 0x40000000u;
static const uint32_t SENTINEL_PASS   = 0xC0FFEE00u;
static const uint8_t  VIA1_IRQ_VEC    = 25u;

struct TestCase {
    const char* name;
    bool (*fn)();
};

#include "tb_cold_boot_support.h"

static std::vector<std::vector<uint8_t>> blank_sectors(size_t n = FULL_ROM_SECTORS) {
    return std::vector<std::vector<uint8_t>>(n, std::vector<uint8_t>(512, 0));
}

static std::vector<uint8_t> load_file_bytes(const char* path) {
    FILE* f = std::fopen(path, "rb");
    if (!f) {
        std::perror(path);
        std::exit(2);
    }
    std::fseek(f, 0, SEEK_END);
    long sz = std::ftell(f);
    std::rewind(f);
    std::vector<uint8_t> bytes((size_t)sz);
    if (sz > 0 && std::fread(bytes.data(), 1, (size_t)sz, f) != (size_t)sz) {
        std::perror("fread");
        std::exit(2);
    }
    std::fclose(f);
    return bytes;
}

static std::vector<uint8_t> load_periph_bytes(const char* name) {
    std::string path = std::string("build/cold_boot_periph/") + name + ".bin";
    auto bytes = load_file_bytes(path.c_str());
    // Directed cold-boot ROMs sit in the first 4 MiB of the SD image
    // (the ROM-loader window before SCSI raw blocks); a 256-byte cap is
    // generous for the few-instruction directed scenarios while still
    // catching accidental "we accidentally linked a 1 MiB ROM here"
    // mistakes.  Was 64 bytes; bumped because realistic peripheral
    // setup (overlay clear + IRQ vector + timer arm) overshoots the
    // tighter limit and forced the via1_t1_irq scenario to silently
    // truncate.  See task #256.
    if (bytes.size() > 256) {
        std::printf("  FAIL %s size %zu exceeds 256-byte limit\n", name, bytes.size());
        std::exit(2);
    }
    return bytes;
}

static std::vector<std::vector<uint8_t>>
bytes_to_sectors(const std::vector<uint8_t>& bytes, size_t nsec = FULL_ROM_SECTORS) {
    auto sec = blank_sectors(nsec);
    for (size_t i = 0; i < bytes.size() && i < nsec * 512; i++)
        sec[i >> 9][i & 0x1FF] = bytes[i];
    return sec;
}

static bool boot_until_done(uint64_t max_cycles) {
    for (uint64_t i = 0; i < max_cycles && !dut->rom_loaded && !dut->boot_error; i++)
        tick();
    return dut->rom_loaded && !dut->boot_error;
}

static bool wait_fetch(uint64_t max_cycles) {
    for (uint64_t i = 0; i < max_cycles && !dut->if_rvalid; i++) tick();
    return dut->if_rvalid;
}

static bool poll_sentinel(SentinelEvent& ev) {
    if (!dut->sentinel_valid) return false;
    ev.value = dut->sentinel_value;
    ev.cycle = sim_time;
    return true;
}

static bool wait_sentinel(uint64_t max_cycles, SentinelEvent& ev) {
    for (uint64_t i = 0; i < max_cycles; i++) {
        if (poll_sentinel(ev)) return true;
        tick();
    }
    return false;
}

static bool arg_has_prefix(const std::string& arg, const char* prefix) {
    return arg.rfind(prefix, 0) == 0;
}

static bool has_arg(int argc, char** argv, const char* name) {
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == name) return true;
    }
    return false;
}

static uint64_t parse_u64_arg(const std::string& text, uint64_t fallback) {
    char* end = nullptr;
    errno = 0;
    unsigned long long value = std::strtoull(text.c_str(), &end, 0);
    if (errno || end == text.c_str() || (end && *end != '\0')) return fallback;
    return (uint64_t)value;
}

static uint64_t plusarg_u64(int argc, char** argv,
                            const char* prefix, uint64_t fallback) {
    for (int i = 1; i < argc; i++) {
        std::string arg(argv[i]);
        if (arg_has_prefix(arg, prefix))
            return parse_u64_arg(arg.substr(std::strlen(prefix)), fallback);
    }
    return fallback;
}

static std::string plusarg_string(int argc, char** argv,
                                  const char* prefix,
                                  const std::string& fallback) {
    for (int i = 1; i < argc; i++) {
        std::string arg(argv[i]);
        if (arg_has_prefix(arg, prefix))
            return arg.substr(std::strlen(prefix));
    }
    return fallback;
}

static bool dump_state(const std::string& path, const char* reason,
                       uint64_t run_start_cycle,
                       uint64_t requested_cycles,
                       uint64_t boundary_events,
                       uint64_t retire_events,
                       uint64_t exc_events,
                       BoundaryEvent last_boundary,
                       BoundaryEvent first_exc,
                       bool have_first_exc) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) {
        std::printf("  FAIL state dump open %s: %s\n",
                    path.c_str(), std::strerror(errno));
        return false;
    }

    std::fprintf(f, "reason=%s\n", reason);
    std::fprintf(f, "run_start_cycle=%llu\n",
                 (unsigned long long)run_start_cycle);
    std::fprintf(f, "requested_cycles=%llu\n",
                 (unsigned long long)requested_cycles);
    std::fprintf(f, "sim_time=%llu\n", (unsigned long long)sim_time);
    std::fprintf(f, "boot rom_loading=%u rom_loaded=%u boot_error=%u "
                    "err_cause=%u st=%u sector=%u ddr_cal_done=%u cpu_rst=%u\n",
                 (unsigned)dut->rom_loading,
                 (unsigned)dut->rom_loaded,
                 (unsigned)dut->boot_error,
                 (unsigned)dut->dbg_err_cause,
                 (unsigned)dut->dbg_st,
                 (unsigned)dut->dbg_sector,
                 (unsigned)dut->ddr_cal_done,
                 (unsigned)dut->cpu_rst);
    std::fprintf(f, "core pc=0x%08x last_pc=0x%08x committed_uops=%u "
                    "committed_macros=%u\n",
                 (unsigned)dut->dbg_pc,
                 (unsigned)dut->dbg_last_pc,
                 (unsigned)dut->dbg_committed,
                 (unsigned)dut->dbg_macros);
    std::fprintf(f, "if addr=0x%08x req=%u rvalid=%u fault=%u "
                    "line=%08x_%08x_%08x_%08x\n",
                 (unsigned)dut->if_addr,
                 (unsigned)dut->if_req,
                 (unsigned)dut->if_rvalid,
                 (unsigned)dut->if_fault,
                 (unsigned)dut->if_rdata[3],
                 (unsigned)dut->if_rdata[2],
                 (unsigned)dut->if_rdata[1],
                 (unsigned)dut->if_rdata[0]);
    std::fprintf(f, "boundary seq=%u kind=%u pc=0x%08x next_pc=0x%08x "
                    "vec=%u fault_pc=0x%08x fault_addr=0x%08x keep_tag=%u\n",
                 (unsigned)dut->dbg_boundary_seq,
                 (unsigned)dut->dbg_boundary_kind,
                 (unsigned)dut->dbg_boundary_pc,
                 (unsigned)dut->dbg_boundary_next_pc,
                 (unsigned)dut->dbg_boundary_exc_vec,
                 (unsigned)dut->dbg_boundary_fault_pc,
                 (unsigned)dut->dbg_boundary_fault_addr,
                 (unsigned)dut->dbg_boundary_keep_tag);
    std::fprintf(f, "boundary_counts total=%llu retire=%llu exc=%llu\n",
                 (unsigned long long)boundary_events,
                 (unsigned long long)retire_events,
                 (unsigned long long)exc_events);
    std::fprintf(f, "last_boundary seq=%u kind=%u pc=0x%08x next_pc=0x%08x "
                    "vec=%u fault_pc=0x%08x fault_addr=0x%08x\n",
                 (unsigned)last_boundary.seq,
                 (unsigned)last_boundary.kind,
                 (unsigned)last_boundary.pc,
                 (unsigned)last_boundary.next_pc,
                 (unsigned)last_boundary.vec,
                 (unsigned)last_boundary.fault_pc,
                 (unsigned)last_boundary.fault_addr);
    if (have_first_exc) {
        std::fprintf(f, "first_exc seq=%u vec=%u pc=0x%08x "
                        "fault_pc=0x%08x fault_addr=0x%08x\n",
                     (unsigned)first_exc.seq,
                     (unsigned)first_exc.vec,
                     (unsigned)first_exc.pc,
                     (unsigned)first_exc.fault_pc,
                     (unsigned)first_exc.fault_addr);
    } else {
        std::fprintf(f, "first_exc none\n");
    }
    std::fprintf(f, "platform overlay_active=%u via1_overlay_bit=%u "
                    "cpu_ipl_ext=%u via1_wr_tap=%u via1_addr_tap=0x%x "
                    "via1_wdata_tap=0x%02x\n",
                 (unsigned)dut->overlay_active,
                 (unsigned)dut->via1_overlay_bit,
                 (unsigned)dut->cpu_ipl_ext,
                 (unsigned)dut->via1_wr_tap,
                 (unsigned)dut->via1_addr_tap,
                 (unsigned)dut->via1_wdata_tap);
    std::fprintf(f, "dafb fb_base_px=0x%08x fb_stride_px=0x%08x "
                    "fb_bpp_reg=0x%08x\n",
                 (unsigned)dut->dafb_fb_base_px,
                 (unsigned)dut->dafb_fb_stride_px,
                 (unsigned)dut->dafb_fb_bpp_reg);
    std::fprintf(f, "sentinel valid=%u value=0x%08x\n",
                 (unsigned)dut->sentinel_valid,
                 (unsigned)dut->sentinel_value);
    std::fclose(f);
    return true;
}

static bool boot_periph(const char* name) {
    sd.reset(bytes_to_sectors(load_periph_bytes(name)));
    reset(true, PERIPH_ENTRY_PC);
    if (boot_until_done(FULL_BOOT_MAX_CYCLES)) return true;
    std::printf("  FAIL %s boot (loaded=%d err=%d cause=%u st=%u sector=%u)\n",
                name, (int)dut->rom_loaded, (int)dut->boot_error,
                (unsigned)dut->dbg_err_cause, (unsigned)dut->dbg_st,
                (unsigned)dut->dbg_sector);
    return false;
}

static bool peek_vram_byte(uint32_t px_addr, uint8_t& value) {
    dut->vram_peek_addr = px_addr;
    dut->vram_peek_en = 1;
    tick();
    dut->vram_peek_en = 0;
    for (int i = 0; i < 8; i++) {
        if (dut->vram_peek_valid) {
            // CONSUMER: the byte at vram_peek_addr is the always-valid top
            // lane [31:24] of the 4-byte group the streaming port returns.
            value = (uint8_t)((dut->vram_peek_data >> 24) & 0xFFu);
            return true;
        }
        tick();
    }
    return false;
}

static bool test_happy_path() {
    auto s = blank_sectors();
    s[0][0x2A] = 0x4E;
    s[0][0x2B] = 0x71;
    sd.reset(s);
    reset();
    if (!boot_until_done(FULL_BOOT_MAX_CYCLES)) {
        std::printf("  FAIL boot (loaded=%d err=%d cause=%u st=%u)\n",
                    (int)dut->rom_loaded, (int)dut->boot_error,
                    (unsigned)dut->dbg_err_cause, (unsigned)dut->dbg_st);
        return false;
    }
    if (!wait_fetch(256)) {
        std::printf("  FAIL no fetch response after boot\n");
        return false;
    }
    bool found = false;
    for (int i = 0; i < 15; i++) {
        if (line_byte(dut->if_rdata, i) == 0x4E &&
            line_byte(dut->if_rdata, i + 1) == 0x71) {
            found = true;
            break;
        }
    }
    if (!found) {
        std::printf("  FAIL first fetch line does not contain 0x4E71\n");
        return false;
    }
    return true;
}

static bool test_short_rom() {
    auto s = blank_sectors();
    sd.reset(s, 4);
    reset();
    for (uint64_t i = 0; i < 4000000 && !dut->boot_error && !dut->rom_loaded; i++)
        tick();
    if (dut->rom_loaded) {
        std::printf("  FAIL rom_loaded despite truncated image\n");
        return false;
    }
    if (!dut->boot_error) {
        std::printf("  FAIL boot_error not raised (st=%u)\n", (unsigned)dut->dbg_st);
        return false;
    }
    if (dut->dbg_err_cause != 3) {
        std::printf("  FAIL err_cause=%u (want 3)\n", (unsigned)dut->dbg_err_cause);
        return false;
    }
    if (!dut->cpu_rst) {
        std::printf("  FAIL cpu_rst released on boot failure\n");
        return false;
    }
    return true;
}

static bool test_late_release() {
    auto s = blank_sectors();
    sd.reset(s);
    reset();
    bool saw_edge = false;
    bool prev_ld = dut->rom_loaded;
    int drop_lag = -1;
    for (uint64_t i = 0; i < FULL_BOOT_MAX_CYCLES; i++) {
        bool ld = dut->rom_loaded;
        if (!prev_ld && ld) saw_edge = true;
        if (!prev_ld && !ld && !dut->cpu_rst) {
            std::printf("  FAIL cpu_rst dropped before rom_loaded edge\n");
            return false;
        }
        if (saw_edge) {
            if (!dut->cpu_rst) { drop_lag = 0; break; }
            tick();
            if (!dut->cpu_rst) { drop_lag = 1; break; }
            std::printf("  FAIL cpu_rst did not drop within 1 cycle\n");
            return false;
        }
        prev_ld = ld;
        tick();
    }
    if (!saw_edge || drop_lag < 0 || drop_lag > 1) {
        std::printf("  FAIL release edge not observed\n");
        return false;
    }
    return true;
}

static bool test_endian() {
    auto s = blank_sectors();
    for (int i = 0; i < 16; i++) s[0][0x20 + i] = (uint8_t)i;
    sd.reset(s);
    reset();
    if (!boot_until_done(FULL_BOOT_MAX_CYCLES)) {
        std::printf("  FAIL boot bad (loaded=%d err=%d cause=%u)\n",
                    (int)dut->rom_loaded, (int)dut->boot_error,
                    (unsigned)dut->dbg_err_cause);
        return false;
    }
    if (!wait_fetch(256)) {
        std::printf("  FAIL no fetch response\n");
        return false;
    }
    uint8_t b0  = line_byte(dut->if_rdata, 0);
    uint8_t b4  = line_byte(dut->if_rdata, 4);
    uint8_t b8  = line_byte(dut->if_rdata, 8);
    uint8_t b12 = line_byte(dut->if_rdata, 12);
    if (b0 != 0x0C || b4 != 0x08 || b8 != 0x04 || b12 != 0x00) {
        std::printf("  FAIL endian line bytes b0/b4/b8/b12 = %02x/%02x/%02x/%02x (want 0C/08/04/00)\n",
                    (unsigned)b0, (unsigned)b4, (unsigned)b8, (unsigned)b12);
        return false;
    }
    return true;
}

static bool test_q700_rom() {
    auto rom = load_file_bytes("files/420dbff3.rom");
    if (rom.size() != 1024 * 1024) {
        std::printf("  FAIL unexpected ROM size %zu\n", rom.size());
        return false;
    }
    sd.reset(bytes_to_sectors(rom));
    reset();
    if (!boot_until_done(FULL_BOOT_MAX_CYCLES)) {
        std::printf("  FAIL boot (loaded=%d err=%d cause=%u st=%u sector=%u)\n",
                    (int)dut->rom_loaded, (int)dut->boot_error,
                    (unsigned)dut->dbg_err_cause, (unsigned)dut->dbg_st,
                    (unsigned)dut->dbg_sector);
        return false;
    }
    if (!wait_fetch(512)) {
        std::printf("  FAIL no first fetch line after ROM boot\n");
        return false;
    }
    uint8_t b0 = line_byte(dut->if_rdata, 0);
    uint8_t b10 = line_byte(dut->if_rdata, 10);
    bool q700_line_match = (b0 == 0x00 && b10 == 0x4E);
    if (!q700_line_match) {
        std::printf("  INFO reset line bytes b0=0x%02x b10=0x%02x\n",
                    (unsigned)b0, (unsigned)b10);
        std::printf("  INFO if_rdata words w3=%08x w2=%08x w1=%08x w0=%08x\n",
                    dut->if_rdata[3], dut->if_rdata[2],
                    dut->if_rdata[1], dut->if_rdata[0]);
    }

    bool saw_reset_jmp = false;
    bool saw_target = false;
    for (int i = 0; i < 5000; i++) {
        BoundaryEvent ev;
        if (poll_boundary(ev)) {
            if (ev.kind == 2 && (ev.vec == 4 || ev.vec == 11)) {
                std::printf("  FAIL exception vec=%u pc=0x%08x fault_pc=0x%08x\n",
                            (unsigned)ev.vec, (unsigned)ev.pc, (unsigned)ev.fault_pc);
                return false;
            }
            if (ev.kind == 1 && ev.pc == RESET_PC && ev.next_pc == RESET_JMP_TGT)
                saw_reset_jmp = true;
            if (ev.kind == 1 && ev.pc == RESET_JMP_TGT)
                saw_target = true;
        }
        tick();
    }
    if (!saw_reset_jmp && !saw_target && dut->dbg_pc != RESET_JMP_TGT) {
        std::printf("  FAIL PC did not advance through reset JMP (pc=0x%08x)\n",
                    (unsigned)dut->dbg_pc);
        return false;
    }
    if (!q700_line_match) {
        std::printf("  FAIL reset line byte check mismatch despite execution progress\n");
        return false;
    }
    return true;
}

static bool test_periph_via1_overlay() {
    if (!boot_periph("via1_overlay")) return false;

    const uint64_t unset = ~0ull;
    bool saw_ddrb = false;
    uint64_t orb_cycle = unset;
    uint64_t via1_drop_cycle = unset;
    SentinelEvent sent;
    bool saw_sent = false;

    for (uint64_t i = 0; i < 4096 && !saw_sent; i++) {
        if (dut->via1_wr_tap && dut->via1_addr_tap == 2 && dut->via1_wdata_tap == 0x08)
            saw_ddrb = true;
        if (dut->via1_wr_tap && dut->via1_addr_tap == 0 &&
            dut->via1_wdata_tap == 0x00 && orb_cycle == unset)
            orb_cycle = sim_time;
        if (!dut->via1_overlay_bit && via1_drop_cycle == unset)
            via1_drop_cycle = sim_time;
        saw_sent = poll_sentinel(sent);
        if (!saw_sent) tick();
    }

    if (!saw_sent) {
        std::printf("  FAIL via1_overlay no sentinel\n");
        return false;
    }
    if (sent.value != SENTINEL_PASS) {
        std::printf("  FAIL via1_overlay sentinel=0x%08x\n", sent.value);
        return false;
    }
    if (!saw_ddrb || orb_cycle == unset) {
        std::printf("  FAIL via1_overlay writes not observed (ddrb=%d orb=%llu)\n",
                    (int)saw_ddrb, (unsigned long long)orb_cycle);
        return false;
    }
    if (via1_drop_cycle != orb_cycle + 1) {
        std::printf("  FAIL via1_overlay bit dropped at %llu (want %llu)\n",
                    (unsigned long long)via1_drop_cycle,
                    (unsigned long long)(orb_cycle + 1));
        return false;
    }
    return true;
}

static bool test_periph_dafb_regs() {
    if (!boot_periph("dafb_regs")) return false;
    SentinelEvent sent;
    if (!wait_sentinel(4096, sent)) {
        std::printf("  FAIL dafb_regs no sentinel\n");
        return false;
    }
    if (sent.value != SENTINEL_PASS) {
        std::printf("  FAIL dafb_regs sentinel=0x%08x\n", sent.value);
        return false;
    }
    if (dut->dafb_fb_base_px != 0x00000100u ||
        dut->dafb_fb_stride_px != 0x0000061Eu ||
        dut->dafb_fb_bpp_reg != 0x00000030u) {
        std::printf("  FAIL dafb_regs outputs base/stride/bpp = %08x/%08x/%08x\n",
                    dut->dafb_fb_base_px, dut->dafb_fb_stride_px, dut->dafb_fb_bpp_reg);
        return false;
    }
    return true;
}

static bool test_periph_vram_roundtrip() {
    if (!boot_periph("vram_roundtrip")) return false;
    SentinelEvent sent;
    if (!wait_sentinel(4096, sent)) {
        std::printf("  FAIL vram_roundtrip no sentinel\n");
        return false;
    }
    if (sent.value != SENTINEL_PASS) {
        uint8_t px0 = 0, px1 = 0;
        bool got_px0 = peek_vram_byte(0, px0);
        bool got_px1 = peek_vram_byte(1, px1);
        std::printf("  FAIL vram_roundtrip sentinel=0x%08x vram0=%s0x%02x vram1=%s0x%02x pc=0x%08x\n",
                    sent.value,
                    got_px0 ? "" : "timeout/", (unsigned)px0,
                    got_px1 ? "" : "timeout/", (unsigned)px1,
                    (unsigned)dut->dbg_pc);
        return false;
    }
    uint8_t px = 0;
    if (!peek_vram_byte(0, px)) {
        std::printf("  FAIL vram_roundtrip peek timeout\n");
        return false;
    }
    if (px != 0x5A) {
        std::printf("  FAIL vram_roundtrip peek=0x%02x (want 0x5A)\n", (unsigned)px);
        return false;
    }
    return true;
}

static bool test_periph_via1_t1_irq() {
    if (!boot_periph("via1_t1_irq")) return false;

    SentinelEvent sent;
    bool saw_sent = false;
    bool saw_ipl_rise = false;
    bool saw_ipl_drop = false;
    bool saw_irq_vec = false;
    uint32_t committed_at_irq = 0;
    uint8_t prev_ipl = (uint8_t)dut->cpu_ipl_ext;

    for (uint64_t i = 0; i < 20000 && !saw_sent; i++) {
        BoundaryEvent ev;
        if (poll_boundary(ev)) {
            if (ev.kind == 2 && ev.vec == VIA1_IRQ_VEC) {
                saw_irq_vec = true;
                committed_at_irq = dut->dbg_committed;
            } else if (ev.kind == 2) {
                std::printf("  FAIL via1_t1_irq unexpected vec=%u pc=0x%08x\n",
                            (unsigned)ev.vec, (unsigned)ev.pc);
                return false;
            }
        }
        if (!saw_ipl_rise && prev_ipl == 0 && dut->cpu_ipl_ext != 0)
            saw_ipl_rise = true;
        if (saw_ipl_rise && prev_ipl != 0 && dut->cpu_ipl_ext == 0)
            saw_ipl_drop = true;
        prev_ipl = (uint8_t)dut->cpu_ipl_ext;
        saw_sent = poll_sentinel(sent);
        if (!saw_sent) tick();
    }

    if (!saw_sent) {
        std::printf("  FAIL via1_t1_irq no sentinel\n");
        return false;
    }
    if (sent.value != SENTINEL_PASS) {
        std::printf("  FAIL via1_t1_irq sentinel=0x%08x\n", sent.value);
        return false;
    }
    if (!saw_ipl_rise || !saw_irq_vec || !saw_ipl_drop) {
        std::printf("  FAIL via1_t1_irq rise/vec/drop = %d/%d/%d\n",
                    (int)saw_ipl_rise, (int)saw_irq_vec, (int)saw_ipl_drop);
        return false;
    }
    if (dut->dbg_committed <= committed_at_irq) {
        std::printf("  FAIL via1_t1_irq committed did not advance after vec=%u\n",
                    (unsigned)VIA1_IRQ_VEC);
        return false;
    }
    return true;
}

static bool test_periph_scsi_no_device() {
    if (!boot_periph("scsi_no_device")) return false;

    SentinelEvent sent;
    bool saw_sent = false;
    bool any_ipl = false;
    for (uint64_t i = 0; i < 4096; i++) {
        if (dut->cpu_ipl_ext != 0) any_ipl = true;
        saw_sent = poll_sentinel(sent);
        if (saw_sent) break;
        tick();
    }
    if (!saw_sent) {
        std::printf("  FAIL scsi_no_device no sentinel\n");
        return false;
    }
    if (sent.value != SENTINEL_PASS) {
        std::printf("  FAIL scsi_no_device sentinel=0x%08x\n", sent.value);
        return false;
    }
    if (any_ipl) {
        std::printf("  FAIL scsi_no_device raised cpu_ipl_ext=%u\n",
                    (unsigned)dut->cpu_ipl_ext);
        return false;
    }
    return true;
}

static bool test_periph_orwell_probe() {
    if (!boot_periph("orwell_probe")) return false;
    SentinelEvent sent;
    if (!wait_sentinel(4096, sent)) {
        std::printf("  FAIL orwell_probe no sentinel\n");
        return false;
    }
    if (sent.value != SENTINEL_PASS) {
        std::printf("  FAIL orwell_probe sentinel=0x%08x\n", sent.value);
        return false;
    }
    return true;
}

static bool test_periph_iwm_probe() {
    if (!boot_periph("iwm_probe")) return false;
    SentinelEvent sent;
    if (!wait_sentinel(4096, sent)) {
        std::printf("  FAIL iwm_probe no sentinel\n");
        return false;
    }
    if (sent.value != SENTINEL_PASS) {
        std::printf("  FAIL iwm_probe sentinel=0x%08x\n", sent.value);
        return false;
    }
    return true;
}

static bool run_q700_deep(uint64_t cycles, const std::string& dump_path) {
    auto rom = load_file_bytes("files/420dbff3.rom");
    if (rom.size() != 1024 * 1024) {
        std::printf("  FAIL unexpected ROM size %zu\n", rom.size());
        return false;
    }

    std::printf("[cold-boot-deep] loading Q700 ROM, then running %llu cycles\n",
                (unsigned long long)cycles);
    sd.reset(bytes_to_sectors(rom));
    reset();
    if (!boot_until_done(FULL_BOOT_MAX_CYCLES)) {
        std::printf("  FAIL boot (loaded=%d err=%d cause=%u st=%u sector=%u)\n",
                    (int)dut->rom_loaded, (int)dut->boot_error,
                    (unsigned)dut->dbg_err_cause, (unsigned)dut->dbg_st,
                    (unsigned)dut->dbg_sector);
        return false;
    }

    const uint64_t run_start = sim_time;
    uint64_t boundary_events = 0;
    uint64_t retire_events = 0;
    uint64_t exc_events = 0;
    BoundaryEvent last_boundary{};
    BoundaryEvent first_exc{};
    bool have_first_exc = false;

    for (uint64_t i = 0; i < cycles; i++) {
        BoundaryEvent ev;
        if (poll_boundary(ev)) {
            boundary_events++;
            last_boundary = ev;
            if (ev.kind == 1) retire_events++;
            if (ev.kind == 2) {
                exc_events++;
                if (!have_first_exc) {
                    first_exc = ev;
                    have_first_exc = true;
                }
            }
        }
        tick();
        if (((i + 1) % 1000000ull) == 0) {
            std::printf("[cold-boot-deep] progress %llu/%llu cycles "
                        "pc=0x%08x committed=%u macros=%u exc=%llu\n",
                        (unsigned long long)(i + 1),
                        (unsigned long long)cycles,
                        (unsigned)dut->dbg_pc,
                        (unsigned)dut->dbg_committed,
                        (unsigned)dut->dbg_macros,
                        (unsigned long long)exc_events);
            std::fflush(stdout);
        }
    }

    bool dump_ok = dump_state(dump_path, "q700_deep_complete",
                              run_start, cycles,
                              boundary_events, retire_events, exc_events,
                              last_boundary, first_exc, have_first_exc);
    if (dump_ok)
        std::printf("[cold-boot-deep] state dump: %s\n", dump_path.c_str());
    return dump_ok;
}

static bool want_test(int argc, char** argv, const char* name) {
    bool have_filter = false;
    for (int i = 1; i < argc; i++) {
        std::string arg(argv[i]);
        if (arg.empty() || arg[0] == '+' || arg == "vec0") continue;
        have_filter = true;
        if (arg == name) return true;
    }
    return !have_filter;
}

// Vec-0 integration scenario — exercises the real HW cold-boot path
// end-to-end: overlay alias (apply_reset_overlay) routes CPU addr 0..ROM_SIZE
// onto DDR's ROM_BASE window, the if_stage vector-0 FSM reads SSP from
// bytes 0..3 and PC from bytes 4..7 of the loaded Q700 ROM, and the CPU
// then fetches at the loaded PC (0x0000_002A for the Q700 Universal ROM).
// Validates:
//   * CPU commits progress past reset entry (0x2A → 0x8C via JMP)
//   * SSP (= ROM checksum prefix 0x420DBFF3) was published to arch A7 on
//     vec-0 delivery; any MOVE a7,... in the ROM would pick it up
//   * No early vec-4 / vec-11 exception — the word-reversal fix in
//     if_to_axi.v keeps the fetched opwords in the CPU's BE-lane
//     convention, so the ROM's 4E7x/4E7B/46FC opcodes at 0x8C+
//     decode cleanly instead of appearing as 0xXXXX0000 garbage.
// Runs ONLY under the FETCH_RESET_VECTORS=1 build (tb-cold-boot-vec0
// target); the default tb-cold-boot binary pins FETCH_RESET_VECTORS=0
// because existing scenarios rely on boot_pc_override which the live
// vec-0 FSM would otherwise shadow (see if_stage.v:298).
#ifdef COLDBOOT_VEC0_BUILD
static bool test_reset_vector_overlay() {
    auto rom = load_file_bytes("files/420dbff3.rom");
    if (rom.size() != 1024 * 1024) {
        std::printf("  FAIL unexpected ROM size %zu\n", rom.size());
        return false;
    }
    // Expected vec-0 bytes from the Q700 Universal ROM dump:
    //   bytes 0..3 (SSP) = 0x420DBFF3   (checksum prefix — unusual as a
    //                                     stack pointer, but it's what the
    //                                     real ROM publishes and should
    //                                     round-trip through if_to_axi's
    //                                     word-reversal unchanged)
    //   bytes 4..7 (PC)  = 0x0000002A   (reset entry, overlay-relative)
    const uint32_t expected_ssp = ((uint32_t)rom[0] << 24) | ((uint32_t)rom[1] << 16)
                                | ((uint32_t)rom[2] <<  8) |  rom[3];
    const uint32_t expected_pc  = ((uint32_t)rom[4] << 24) | ((uint32_t)rom[5] << 16)
                                | ((uint32_t)rom[6] <<  8) |  rom[7];
    if (expected_ssp != 0x420DBFF3u || expected_pc != 0x0000002Au) {
        std::printf("  FAIL unexpected Q700 vec-0 header: SSP=0x%08x PC=0x%08x\n",
                    expected_ssp, expected_pc);
        return false;
    }

    sd.reset(bytes_to_sectors(rom));
    // No boot_pc_override: let the CPU's vec-0 FSM drive PC itself.
    reset();
    if (!boot_until_done(FULL_BOOT_MAX_CYCLES)) {
        std::printf("  FAIL boot (loaded=%d err=%d cause=%u st=%u sector=%u)\n",
                    (int)dut->rom_loaded, (int)dut->boot_error,
                    (unsigned)dut->dbg_err_cause, (unsigned)dut->dbg_st,
                    (unsigned)dut->dbg_sector);
        return false;
    }
    if (!wait_fetch(512)) {
        std::printf("  FAIL no first fetch line after ROM boot\n");
        return false;
    }

    // First CPU ifetch: under FETCH_RESET_VECTORS=1 this must be
    // addr 0 (the overlay aliases 0..ROM_SIZE onto ROM_BASE so the
    // AXI fabric serves ROM bytes 0..15).
    //
    // Helper: the existing `line_byte(line, idx)` helper indexes bytes
    // in a "32-bit-word-reversed" order — idx=0..3 returns line bytes
    // 12..15, idx=12..15 returns line bytes 0..3.  That matches how
    // `predecode.v` labels `fetch_buf[127-8*idx : 120-8*idx]`.  Define
    // a local "real byte-at-line-offset" helper so the assertion below
    // reads naturally.
    auto line_b = [](const uint32_t* line, int off) {
        // off is the actual byte offset 0..15 within the 16-byte line.
        // if_rdata[127:120] = byte 0; if_rdata[7:0] = byte 15.  Verilator
        // packs word 0 = bits [31:0], word 3 = bits [127:96].
        const int word = 3 - (off >> 2);
        const int shift = 24 - 8 * (off & 3);
        return (uint8_t)((line[word] >> shift) & 0xFFu);
    };
    uint8_t b0 = line_b(dut->if_rdata, 0);
    uint8_t b1 = line_b(dut->if_rdata, 1);
    uint8_t b2 = line_b(dut->if_rdata, 2);
    uint8_t b3 = line_b(dut->if_rdata, 3);
    uint8_t b4 = line_b(dut->if_rdata, 4);
    uint8_t b5 = line_b(dut->if_rdata, 5);
    uint8_t b6 = line_b(dut->if_rdata, 6);
    uint8_t b7 = line_b(dut->if_rdata, 7);
    if (b0 != 0x42 || b1 != 0x0D || b2 != 0xBF || b3 != 0xF3 ||
        b4 != 0x00 || b5 != 0x00 || b6 != 0x00 || b7 != 0x2A) {
        std::printf("  FAIL vec-0 line byte-order: got "
                    "%02x %02x %02x %02x / %02x %02x %02x %02x, "
                    "expected 42 0D BF F3 / 00 00 00 2A\n",
                    b0, b1, b2, b3, b4, b5, b6, b7);
        std::printf("  INFO if_rdata[3..0] = %08x %08x %08x %08x\n",
                    dut->if_rdata[3], dut->if_rdata[2],
                    dut->if_rdata[1], dut->if_rdata[0]);
        return false;
    }

    // After the vec-0 FSM completes the CPU loads PC=0x0000002A and
    // begins normal fetch from the overlay-aliased ROM.  The ROM at
    // 0x2A holds `4E FA 00 60` (JMP (d16,PC)) → 0x2C + 0x60 = 0x8C.
    // Watch the retire-boundary stream until we see PC=0x2A commit
    // with next_pc=0x8C, which proves:
    //   (a) vec-0 PC load worked — CPU is executing at 0x2A, not at
    //       RESET_PC or some garbage value;
    //   (b) the word-reversal in if_to_axi keeps the opword
    //       `4E FA 00 60` intact, so the decoder sees a real JMP.
    const uint32_t VEC0_PC       = 0x0000002Au;
    const uint32_t VEC0_JMP_TGT  = 0x0000008Cu;
    bool saw_reset_jmp = false;
    bool saw_target = false;
    for (int i = 0; i < 20000; i++) {
        BoundaryEvent ev;
        if (poll_boundary(ev)) {
            if (ev.kind == 2 && (ev.vec == 4 || ev.vec == 11)) {
                std::printf("  FAIL exception vec=%u pc=0x%08x fault_pc=0x%08x\n",
                            (unsigned)ev.vec, (unsigned)ev.pc, (unsigned)ev.fault_pc);
                return false;
            }
            if (ev.kind == 1 && ev.pc == VEC0_PC && ev.next_pc == VEC0_JMP_TGT)
                saw_reset_jmp = true;
            if (ev.kind == 1 && ev.pc == VEC0_JMP_TGT)
                saw_target = true;
        }
        tick();
    }
    if (!saw_reset_jmp) {
        std::printf("  FAIL CPU did not retire reset-JMP at PC=0x%08x "
                    "(last_pc=0x%08x, dbg_pc=0x%08x)\n",
                    VEC0_PC, (unsigned)dut->dbg_last_pc,
                    (unsigned)dut->dbg_pc);
        return false;
    }
    if (!saw_target) {
        std::printf("  FAIL CPU did not retire the reset-JMP target "
                    "(expected PC=0x%08x; dbg_pc=0x%08x)\n",
                    VEC0_JMP_TGT, (unsigned)dut->dbg_pc);
        return false;
    }
    return true;
}
#endif

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vtb_cold_boot;
    bool vec0_mode = (argc > 1 && std::string(argv[1]) == "vec0");
    if (has_arg(argc, argv, "+q700_deep")) {
        uint64_t cycles = plusarg_u64(argc, argv, "+cycles=", 10000000ull);
        std::string dump_path = plusarg_string(
            argc, argv, "+state_dump=", "build/cold_boot/q700_deep_state.txt");
        bool ok = run_q700_deep(cycles, dump_path);
        if (ok) {
            n_pass++;
            std::printf("[PASS] q700_deep\n");
        } else {
            n_fail++;
            std::printf("[FAIL] q700_deep\n");
        }
        std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);
        if (dut) {
            std::printf("[METRICS] cold-boot committed_uops=%u committed_macros=%u\n",
                        (unsigned)dut->dbg_committed,
                        (unsigned)dut->dbg_macros);
        }
        dut->final();
        delete dut;
        return n_fail ? 1 : 0;
    }
#ifdef COLDBOOT_VEC0_BUILD
    const TestCase vec0_tests[] = {
        {"test_reset_vector_overlay", test_reset_vector_overlay},
    };
    if (!vec0_mode) {
        std::printf("  FAIL COLDBOOT_VEC0_BUILD binary invoked without \"vec0\" "
                    "argv — FETCH_RESET_VECTORS=1 build shadows boot_pc_override, "
                    "so the default test suite cannot run here\n");
        delete dut;
        return 2;
    }
    for (const auto& test : vec0_tests) {
        bool ok = test.fn();
        if (ok) { std::printf("[PASS] %s\n", test.name); n_pass++; }
        else    { std::printf("[FAIL] %s\n", test.name); n_fail++; }
    }
#else
    if (vec0_mode) {
        std::printf("  FAIL default tb_cold_boot binary invoked with \"vec0\" "
                    "argv — rebuild with -GFETCH_RESET_VECTORS=1 via "
                    "`make tb-cold-boot-vec0`\n");
        delete dut;
        return 2;
    }
    // test_q700_rom executes the full 1 MB Q700 Universal ROM and is the
    // single most expensive scenario in this binary (~2-3M cycles wall).
    // Gate it behind +full so default `make tb-cold-boot` runs only the
    // directed scenarios (~10-15s total).  Pass `+full` (or name the test
    // explicitly on the cmd line) to opt in.
    const bool full_mode = has_arg(argc, argv, "+full") ||
                           has_arg(argc, argv, "test_q700_rom");
    const TestCase tests[] = {
        {"test_happy_path", test_happy_path},
        {"test_short_rom", test_short_rom},
        {"test_late_release", test_late_release},
        {"test_endian", test_endian},
        {"test_q700_rom", test_q700_rom},
        {"test_periph_via1_overlay", test_periph_via1_overlay},
        {"test_periph_dafb_regs", test_periph_dafb_regs},
        {"test_periph_vram_roundtrip", test_periph_vram_roundtrip},
        {"test_periph_via1_t1_irq", test_periph_via1_t1_irq},
        {"test_periph_scsi_no_device", test_periph_scsi_no_device},
        {"test_periph_orwell_probe", test_periph_orwell_probe},
        {"test_periph_iwm_probe", test_periph_iwm_probe},
    };
    for (const auto& test : tests) {
        if (!want_test(argc, argv, test.name)) continue;
        if (std::string(test.name) == "test_q700_rom" && !full_mode) {
            std::printf("[SKIP] %s (gated behind +full; pass +full or name "
                        "the test explicitly to run it)\n", test.name);
            continue;
        }
        bool ok = test.fn();
        if (ok) {
            std::printf("[PASS] %s\n", test.name);
            n_pass++;
        } else {
            std::printf("[FAIL] %s\n", test.name);
            n_fail++;
        }
    }
#endif
    std::printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);

    // Macro-IPC metric (task #192): committed-macro-instructions / cycles.
    // dbg_macros counts only uops with last_phase=1 → retired macros.
    // sim_time is not visible from here; cycle-derivation uses a local
    // approximation via committed/macros ratio only — the tb does not
    // maintain a global tick counter across all sub-tests.  Emit the
    // counts unconditionally so CI can grep them.
    if (dut) {
        const unsigned cb_uops   = (unsigned)dut->dbg_committed;
        const unsigned cb_macros = (unsigned)dut->dbg_macros;
        std::printf("[METRICS] cold-boot committed_uops=%u committed_macros=%u\n",
                    cb_uops, cb_macros);
    }
    dut->final();
    delete dut;
    return n_fail ? 1 : 0;
}
