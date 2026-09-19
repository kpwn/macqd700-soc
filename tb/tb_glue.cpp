// tb_glue.cpp — Verilator unit testbench for rtl/mac/glue.v
//
// Standalone Verilator build — glue.v in isolation.  Exercises the
// Quadra 700 address decoder + ROM overlay alias with ≥10 scenarios.
//
// Build via:   make tb-glue
//
// Coverage matrix
// ═══════════════
//   1. Overlay=1 aliases low 16 MB to ROM image (read from phys 0)
//   2. Overlay=0 low window is plain RAM (no alias)
//   3. ROM @0x4xxx_xxxx is always CLASS_ROM regardless of overlay
//   4. MAME Q700 I/O mirror resolves VIA1 / VIA2 / ENET / SONIC / ORWELL / SCC / SCSI / ASC
//   5. I/O sub-decode for SWIM/IWM @0x5001_E000 and the empty 0x4000 gap
//   6. DAFB framebuffer @0xF900_0000 resolves to CLASS_VIDEO
//   7. Unmapped I/O gap (e.g. 0x5000_4000) raises `fault`
//   8. Unmapped top-level (0x8000_0000) raises `fault`
//   9. FC=0..3 (user) I/O access asserts `priv_violate`; FC=4..7 does not
//  10. Overlay-cleared transition: same address flips class from ROM→RAM
//  11. req_valid=0 keeps all chip-selects/fault low (quiescent bus)
//  12. MAME TurboSCSI windows @0x5000_F000..F0FF and F100..F101
//      resolve to cs_scsi; the rest of the 4 KB page faults.
//  13. Q700 mirror mask reaches VIA +0x1C00 aliases, but not 0x5100xxxx.
//  14. Boot-time map transition keeps I/O live while low vectors switch
//      from ROM overlay to RAM after VIA1 clears ORB[3].
//  15. Unowned 0xF9 video gaps fault instead of dropping with no sink.
//  16. ASC and DAFB decode stop at the MAME/Q700 probe boundaries.
//  17. Q700 0x5800_0000 RAM probe alias maps to low RAM.
//
// Expected output:
//   [PASS] ... for each scenario
//   All N scenarios PASSED.

#include <cstdio>
#include <cstdint>
#include <verilated.h>
#include "Vglue.h"

static Vglue*   dut       = nullptr;
static uint64_t sim_time  = 0;
static int      n_pass    = 0;
static int      n_fail    = 0;

// ── Class enum mirror ──────────────────────────────────────────────────
static constexpr uint8_t CLASS_UNMAP = 0;
static constexpr uint8_t CLASS_RAM   = 1;
static constexpr uint8_t CLASS_ROM   = 2;
static constexpr uint8_t CLASS_IO    = 3;
static constexpr uint8_t CLASS_VIDEO = 4;

static constexpr uint32_t ROM_BASE  = 0x40000000u;
static constexpr uint32_t ROM_MASK  = 0x00FFFFFFu;   // ROM_SIZE_LOG2 = 24

// ── Clock helpers ──────────────────────────────────────────────────────
// glue.v is purely combinational but we still tick so any sequential
// observer that might be added later continues to work; also gives
// Verilator a chance to fire settle callbacks if any.
static void tick() {
    dut->clk = 0; dut->eval(); sim_time++;
    dut->clk = 1; dut->eval(); sim_time++;
}

static void reset() {
    dut->rst        = 1;
    dut->req_addr   = 0;
    dut->req_valid  = 0;
    dut->req_rw     = 0;
    dut->req_size   = 2;
    dut->req_fc     = 0b101;   // supervisor data
    dut->overlay_in = 0;
    tick(); tick();
    dut->rst = 0;
    tick();
}

// ── Assertion macros ───────────────────────────────────────────────────
#define CHECK_EQ(name, got, exp) do { \
    if ((uint64_t)(got) != (uint64_t)(exp)) { \
        printf("  FAIL %s: got 0x%llx, expected 0x%llx\n", \
               name, (unsigned long long)(got), (unsigned long long)(exp)); \
        return false; \
    } \
} while(0)

#define CHECK_TRUE(name, cond) do { \
    if (!(cond)) { \
        printf("  FAIL %s: condition false\n", name); \
        return false; \
    } \
} while(0)

// ── Decode helper ──────────────────────────────────────────────────────
// Drive the inputs and settle — glue is combinational so a bare eval()
// suffices, but we also clock once so any waveform observer captures
// the transition.
struct Decode {
    uint32_t out_addr;
    uint8_t  out_class;
    bool     cs_ram, cs_rom, cs_via1, cs_via2, cs_enet, cs_sonic;
    bool     cs_scc, cs_scsi, cs_asc, cs_iwm, cs_orwell, cs_video;
    bool     fault;
    bool     priv;
};

static Decode decode(uint32_t addr, bool valid = true, bool rw = false,
                     uint8_t fc = 0b101, bool overlay = false) {
    dut->req_addr   = addr;
    dut->req_valid  = valid ? 1 : 0;
    dut->req_rw     = rw ? 1 : 0;
    dut->req_size   = 2;
    dut->req_fc     = fc;
    dut->overlay_in = overlay ? 1 : 0;
    dut->eval();
    tick();       // advance sim-time for wave readability
    Decode d;
    d.out_addr  = dut->out_addr;
    d.out_class = dut->out_class;
    d.cs_ram    = dut->cs_ram;
    d.cs_rom    = dut->cs_rom;
    d.cs_via1   = dut->cs_via1;
    d.cs_via2   = dut->cs_via2;
    d.cs_enet   = dut->cs_enet;
    d.cs_sonic  = dut->cs_sonic;
    d.cs_scc    = dut->cs_scc;
    d.cs_scsi   = dut->cs_scsi;
    d.cs_asc    = dut->cs_asc;
    d.cs_iwm    = dut->cs_iwm;
    d.cs_orwell = dut->cs_orwell;
    d.cs_video  = dut->cs_video;
    d.fault     = dut->fault;
    d.priv      = dut->priv_violate;
    return d;
}

static bool any_io_cs(const Decode& d) {
    return d.cs_via1 || d.cs_via2 || d.cs_enet || d.cs_sonic ||
           d.cs_scc || d.cs_scsi || d.cs_asc || d.cs_iwm ||
           d.cs_orwell || d.cs_video;
}

// ════════════════════════════════════════════════════════════════════════
// 1 — Overlay=1 aliases phys 0x00000000..0x00FFFFFF onto ROM image
// ════════════════════════════════════════════════════════════════════════
static bool test_overlay_on_aliases_rom() {
    reset();
    // Read from a low address with overlay set — should decode to ROM
    // and out_addr should reflect the ROM alias.
    Decode d = decode(0x00001234u, true, false, 0b101, true);
    CHECK_EQ  ("class=ROM",        d.out_class, CLASS_ROM);
    CHECK_EQ  ("out_addr=ROM+1234", d.out_addr, ROM_BASE | 0x00001234u);
    CHECK_TRUE("cs_rom asserted",  d.cs_rom);
    CHECK_TRUE("cs_ram clear",     !d.cs_ram);
    CHECK_TRUE("no fault",         !d.fault);

    // A harder address further into the low window.
    d = decode(0x00FEDEADu, true, false, 0b101, true);
    CHECK_EQ  ("class=ROM (high)",  d.out_class, CLASS_ROM);
    CHECK_EQ  ("out_addr (high)",   d.out_addr, ROM_BASE | 0x00FEDEADu);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// 2 — Overlay=0 low window is RAM (no alias)
// ════════════════════════════════════════════════════════════════════════
static bool test_overlay_off_is_ram() {
    reset();
    Decode d = decode(0x00001234u, true, false, 0b101, false);
    CHECK_EQ  ("class=RAM",        d.out_class, CLASS_RAM);
    CHECK_EQ  ("out_addr pass-thru", d.out_addr, 0x00001234u);
    CHECK_TRUE("cs_ram asserted",  d.cs_ram);
    CHECK_TRUE("cs_rom clear",     !d.cs_rom);
    CHECK_TRUE("no fault",         !d.fault);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// 2b — Q700 late-ROM RAM probe alias at 0x5800_0000
// ════════════════════════════════════════════════════════════════════════
static bool test_q700_ram_probe_alias() {
    reset();
    Decode d = decode(0x58000000u, true, false, 0b101, false);
    CHECK_EQ  ("0x58000000 class=RAM", d.out_class, CLASS_RAM);
    CHECK_EQ  ("0x58000000 maps to low RAM", d.out_addr, 0x00000000u);
    CHECK_TRUE("0x58000000 cs_ram", d.cs_ram);
    CHECK_TRUE("0x58000000 no fault", !d.fault);

    d = decode(0x58002000u, true, true, 0b101, false);
    CHECK_EQ  ("0x58002000 class=RAM", d.out_class, CLASS_RAM);
    CHECK_EQ  ("0x58002000 maps to 0x2000", d.out_addr, 0x00002000u);
    CHECK_TRUE("0x58002000 cs_ram", d.cs_ram);
    CHECK_TRUE("0x58002000 no fault", !d.fault);

    d = decode(0x5C000000u, true, false, 0b101, false);
    CHECK_EQ  ("past 0x5800 alias class=UNMAP", d.out_class, CLASS_UNMAP);
    CHECK_TRUE("past 0x5800 alias faults", d.fault);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// 3 — ROM image @0x4xxx_xxxx is CLASS_ROM regardless of overlay
// ════════════════════════════════════════════════════════════════════════
static bool test_rom_mirror_regardless_of_overlay() {
    reset();
    Decode d1 = decode(0x40800000u, true, false, 0b101, false);
    CHECK_EQ  ("class=ROM (overlay off)",  d1.out_class, CLASS_ROM);
    CHECK_EQ  ("out_addr pass-thru", d1.out_addr, 0x40800000u);
    CHECK_TRUE("cs_rom", d1.cs_rom);

    Decode d2 = decode(0x40800000u, true, false, 0b101, true);
    CHECK_EQ  ("class=ROM (overlay on)",   d2.out_class, CLASS_ROM);
    CHECK_EQ  ("out_addr pass-thru (2)", d2.out_addr, 0x40800000u);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// 4 — I/O window sub-decode for each known slave
// ════════════════════════════════════════════════════════════════════════
static bool test_io_chip_selects() {
    reset();
    struct { uint32_t addr; const char *name; bool Decode::*cs; } cases[] = {
        { 0x50000000u, "VIA1 canonical", &Decode::cs_via1 },
        { 0x50001C00u, "VIA1 IER stride", &Decode::cs_via1 },
        { 0x50F81C00u, "VIA1 mirrored IER", &Decode::cs_via1 },
        { 0x50002000u, "VIA2 canonical", &Decode::cs_via2 },
        { 0x50F03C00u, "VIA2 mirrored IER stride", &Decode::cs_via2 },
        { 0x50008000u, "Ethernet ID canonical", &Decode::cs_enet },
        { 0x50F08000u, "Ethernet ID mirrored", &Decode::cs_enet },
        { 0x5000A000u, "SONIC canonical", &Decode::cs_sonic },
        { 0x50F0A000u, "SONIC mirrored", &Decode::cs_sonic },
        { 0x5000C000u, "SCC canonical", &Decode::cs_scc  },
        { 0x50F0C020u, "SCC mirrored reg", &Decode::cs_scc },
        { 0x5000E000u, "Orwell canonical", &Decode::cs_orwell },
        { 0x50F0E000u, "Orwell mirrored", &Decode::cs_orwell },
        { 0x5000F000u, "TurboSCSI", &Decode::cs_scsi },
        { 0x50014000u, "ASC canonical",  &Decode::cs_asc  },
        { 0x50015FFCu, "ASC top word",   &Decode::cs_asc  },
        { 0x50F14801u, "ASC mirrored mode reg", &Decode::cs_asc },
    };
    for (auto &c : cases) {
        Decode d = decode(c.addr, true, false, 0b101, false);
        char lbl[64];
        snprintf(lbl, sizeof(lbl), "%s @ 0x%08x", c.name, c.addr);
        CHECK_EQ  (lbl, d.out_class, CLASS_IO);
        if (!(d.*(c.cs))) {
            printf("  FAIL %s: chip-select not asserted\n", lbl);
            return false;
        }
        if (d.fault) {
            printf("  FAIL %s: unexpected fault\n", lbl);
            return false;
        }
    }
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// 5 — SWIM/IWM slot + MAME-empty 0x4000 mirror gap
// ════════════════════════════════════════════════════════════════════════
static bool test_swim_and_empty_mirror_gap() {
    reset();
    // SWIM/IWM at canonical 0x5001_E000, mirrored through 0x50F1_E000.
    Decode d = decode(0x5001E000u, true, false, 0b101, false);
    CHECK_EQ  ("SWIM class=IO", d.out_class, CLASS_IO);
    CHECK_TRUE("cs_iwm",        d.cs_iwm);
    CHECK_TRUE("no fault",      !d.fault);

    d = decode(0x50F1E000u, true, false, 0b101, false);
    CHECK_EQ  ("SWIM mirror class=IO", d.out_class, CLASS_IO);
    CHECK_TRUE("cs_iwm mirror", d.cs_iwm);
    CHECK_TRUE("no fault on SWIM mirror", !d.fault);

    // MAME validation says 0x50F04000 canonicalizes to empty 0x50004000;
    // do not alias it into SCC or VIA1.
    d = decode(0x50F04000u, true, false, 0b101, false);
    CHECK_EQ  ("empty 0x4000 gap class=IO", d.out_class, CLASS_IO);
    CHECK_TRUE("fault on 0x4000 gap", d.fault);
    CHECK_TRUE("no VIA1 on 0x4000 gap", !d.cs_via1);
    CHECK_TRUE("no SCC on 0x4000 gap", !d.cs_scc);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// 5b — Q700 VIA windows cover all 16 registers at 512-byte stride
// ════════════════════════════════════════════════════════════════════════
static bool test_q700_via_stride_windows() {
    reset();
    Decode d = decode(0x50F01C00u, true, false, 0b101, false);
    CHECK_EQ  ("VIA1 IER-stride class=IO", d.out_class, CLASS_IO);
    CHECK_TRUE("VIA1 IER-stride cs_via1", d.cs_via1);
    CHECK_TRUE("VIA1 IER-stride no fault", !d.fault);

    d = decode(0x50F03C00u, true, false, 0b101, false);
    CHECK_EQ  ("VIA2 IER-stride class=IO", d.out_class, CLASS_IO);
    CHECK_TRUE("VIA2 IER-stride cs_via2", d.cs_via2);
    CHECK_TRUE("VIA2 IER-stride no fault", !d.fault);

    d = decode(0x50F81C00u, true, false, 0b101, false);
    CHECK_EQ  ("VIA1 mirrored IER class=IO", d.out_class, CLASS_IO);
    CHECK_TRUE("VIA1 mirrored IER cs_via1", d.cs_via1);
    CHECK_TRUE("VIA1 mirrored IER no fault", !d.fault);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// 6 — DAFB video + VRAM pixel aperture
// ════════════════════════════════════════════════════════════════════════
// Task #147 split cs_video out of the 0xF900_0000 pixel aperture — the
// xbar now routes that aperture straight to the URAM-backed `vram`
// slave (S3).  cs_video fires ONLY on the DAFB register window at
// 0xF980_0000..0xF980_0FFF (task #143's territory).  class_r stays
// CLASS_VIDEO across the entire 0xF9xx prefix so observational
// decoders and downstream tooling don't regress.
static bool test_video_framebuffer() {
    reset();
    // VRAM pixel aperture (0xF900_0000..0xF91F_FFFF, 2 MB) — still
    // CLASS_VIDEO at the class level, but cs_video is LOW (xbar handles
    // the write).
    Decode d = decode(0xF9001000u, true, false, 0b101, false);
    CHECK_EQ  ("class=VIDEO (pixel)", d.out_class, CLASS_VIDEO);
    CHECK_TRUE("cs_video NOT asserted on pixel aperture", !d.cs_video);
    CHECK_TRUE("no fault on pixel aperture", !d.fault);
    // DAFB register window — cs_video MUST fire here (DAFB shim target).
    d = decode(0xF9800000u, true, false, 0b101, false);
    CHECK_EQ  ("class=VIDEO (dafb reg)", d.out_class, CLASS_VIDEO);
    CHECK_TRUE("cs_video asserted on DAFB regs", d.cs_video);
    d = decode(0xF98003FCu, true, false, 0b101, false);
    CHECK_TRUE("cs_video asserted at DAFB top", d.cs_video);
    // Just past DAFB reg window — still CLASS_VIDEO (observational) but
    // no cs_video (no live sink in glue).
    d = decode(0xF9800400u, true, false, 0b101, false);
    CHECK_EQ  ("class=VIDEO (post-dafb)", d.out_class, CLASS_VIDEO);
    CHECK_TRUE("cs_video NOT asserted past DAFB", !d.cs_video);
    CHECK_TRUE("fault past DAFB", d.fault);
    // Top of 0xF9 window.
    d = decode(0xF9FFFFFFu, true, false, 0b101, false);
    CHECK_EQ  ("class=VIDEO (end)", d.out_class, CLASS_VIDEO);
    CHECK_TRUE("fault at unowned 0xF9 top", d.fault);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// 7 — Unmapped I/O raises `fault`
// ════════════════════════════════════════════════════════════════════════
static bool test_io_unmapped_fault() {
    reset();
    // MAME leaves the 0x4000 canonical gap empty on Q700.
    Decode d = decode(0x50004000u, true, false, 0b101, false);
    CHECK_EQ  ("class=IO (0x4000 gap)", d.out_class, CLASS_IO);
    CHECK_TRUE("fault asserted",     d.fault);
    CHECK_TRUE("no chip-select lit", !any_io_cs(d));

    // The same gap through the common 0x50F0_xxxx mirror must stay empty.
    d = decode(0x50F04000u, true, false, 0b101, false);
    CHECK_EQ  ("class=IO (mirrored gap)", d.out_class, CLASS_IO);
    CHECK_TRUE("fault asserted (2)", d.fault);

    // Gap after VIA1/VIA2.
    d = decode(0x50F06000u, true, false, 0b101, false);
    CHECK_TRUE("fault on VIA gap", d.fault);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// 8 — Unmapped top-level
// ════════════════════════════════════════════════════════════════════════
static bool test_toplevel_unmapped_fault() {
    reset();
    // 0x8000_0000 is neither RAM/ROM/IO/video.
    Decode d = decode(0x80000000u, true, false, 0b101, false);
    CHECK_EQ  ("class=UNMAP", d.out_class, CLASS_UNMAP);
    CHECK_TRUE("fault",        d.fault);
    CHECK_TRUE("no cs",        !(d.cs_ram||d.cs_rom||d.cs_video));
    // And an address beyond the RAM window but below 0x4 — unmap.
    d = decode(0x20000000u, true, false, 0b101, false);
    CHECK_EQ  ("class=UNMAP (2)", d.out_class, CLASS_UNMAP);
    CHECK_TRUE("fault (2)",        d.fault);
    // The ROM frontier probe is outside the 0x5000_0000..0x50FF_FFFF
    // Q700 I/O mirror and must not decode as a peripheral.
    d = decode(0x51001C00u, true, false, 0b101, false);
    CHECK_EQ  ("0x51001c00 class=UNMAP", d.out_class, CLASS_UNMAP);
    CHECK_TRUE("0x51001c00 fault", d.fault);
    CHECK_TRUE("0x51001c00 no peripheral cs", !any_io_cs(d));
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// 8b — Q700 probe gaps adjacent to implemented peripheral windows
// ════════════════════════════════════════════════════════════════════════
static bool test_adjacent_probe_gaps_do_not_alias() {
    reset();

    // Harness/MAME Q700 map uses an 8 KB EASC window.  The following
    // 0x50016000..0x50017fff gap must not keep selecting ASC.
    Decode d = decode(0x50015FFCu, true, false, 0b101, false);
    CHECK_EQ  ("ASC top class=IO", d.out_class, CLASS_IO);
    CHECK_TRUE("ASC top cs_asc", d.cs_asc);
    CHECK_TRUE("ASC top no fault", !d.fault);

    d = decode(0x50016000u, true, false, 0b101, false);
    CHECK_EQ  ("post-ASC gap class=IO", d.out_class, CLASS_IO);
    CHECK_TRUE("post-ASC gap faults", d.fault);
    CHECK_TRUE("post-ASC gap no cs_asc", !d.cs_asc);

    d = decode(0x50F16000u, true, false, 0b101, false);
    CHECK_EQ  ("mirrored post-ASC gap class=IO", d.out_class, CLASS_IO);
    CHECK_TRUE("mirrored post-ASC gap faults", d.fault);
    CHECK_TRUE("mirrored post-ASC gap no cs_asc", !d.cs_asc);

    // Keep common ROM-control probes deterministic: Ethernet, SONIC, and
    // Orwell are owned stubs in glue while adjacent gaps remain unmapped.
    uint32_t unmapped[] = {
        0x50008008u, 0x50F08008u, // post-Ethernet gap
        0x50009FFCu, 0x50F09FFCu, // pre-SONIC gap
        0x5000B100u, 0x50F0B100u, // post-SONIC gap
        0x5000E100u, 0x50F0E100u  // post-Orwell gap
    };
    for (uint32_t addr : unmapped) {
        d = decode(addr, true, false, 0b101, false);
        char lbl[64];
        snprintf(lbl, sizeof(lbl), "unmapped probe 0x%08x", addr);
        CHECK_EQ  (lbl, d.out_class, CLASS_IO);
        CHECK_TRUE("unmapped probe faults", d.fault);
        CHECK_TRUE("unmapped probe no peripheral cs", !any_io_cs(d));
    }

    // MAME's direct DAFB register map is 1 KB.  The observational
    // 0xF9 class remains, but the next word must fault with no cs_video.
    d = decode(0xF98003FCu, true, false, 0b101, false);
    CHECK_EQ  ("DAFB top class=VIDEO", d.out_class, CLASS_VIDEO);
    CHECK_TRUE("DAFB top cs_video", d.cs_video);
    CHECK_TRUE("DAFB top no fault", !d.fault);

    d = decode(0xF9800400u, true, false, 0b101, false);
    CHECK_EQ  ("post-DAFB gap class=VIDEO", d.out_class, CLASS_VIDEO);
    CHECK_TRUE("post-DAFB gap faults", d.fault);
    CHECK_TRUE("post-DAFB gap no cs_video", !d.cs_video);

    d = decode(0xF9800FFCu, true, false, 0b101, false);
    CHECK_EQ  ("old 4KB DAFB top class=VIDEO", d.out_class, CLASS_VIDEO);
    CHECK_TRUE("old 4KB DAFB top faults", d.fault);
    CHECK_TRUE("old 4KB DAFB top no cs_video", !d.cs_video);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// 9 — User-mode I/O raises priv_violate; supervisor-mode does not
// ════════════════════════════════════════════════════════════════════════
static bool test_priv_violate() {
    reset();
    // User-mode data access (FC=001): priv_violate must fire on I/O.
    Decode d = decode(0x50F00000u, true, false, 0b001, false);
    CHECK_TRUE("user→VIA1: priv_violate", d.priv);

    // Supervisor-mode data access (FC=101): priv_violate clear.
    d = decode(0x50F00000u, true, false, 0b101, false);
    CHECK_TRUE("super→VIA1: !priv", !d.priv);

    // User-mode RAM access: priv_violate should NOT fire (RAM is open).
    d = decode(0x00001000u, true, false, 0b001, false);
    CHECK_TRUE("user→RAM: !priv", !d.priv);

    // User-mode ROM read: priv_violate should NOT fire.
    d = decode(0x40800000u, true, false, 0b010, false);
    CHECK_TRUE("user→ROM: !priv", !d.priv);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// 10 — Overlay-cleared transition: same addr flips ROM→RAM
// ════════════════════════════════════════════════════════════════════════
static bool test_overlay_transition() {
    reset();
    Decode d1 = decode(0x00000100u, true, false, 0b101, true);
    CHECK_EQ("pre-clear class=ROM", d1.out_class, CLASS_ROM);
    // Clear overlay — the same address should now decode as RAM.
    Decode d2 = decode(0x00000100u, true, false, 0b101, false);
    CHECK_EQ  ("post-clear class=RAM", d2.out_class, CLASS_RAM);
    CHECK_EQ  ("post-clear addr=pass-thru", d2.out_addr, 0x00000100u);
    CHECK_TRUE("post-clear cs_ram", d2.cs_ram);
    CHECK_TRUE("post-clear cs_rom clear", !d2.cs_rom);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// 11 — req_valid=0 quiesces the bus: no chip-selects, no fault
// ════════════════════════════════════════════════════════════════════════
static bool test_req_invalid_quiesces() {
    reset();
    // Point at an I/O address that would normally assert cs_via1.
    Decode d = decode(0x50F00000u, /*valid=*/false, false, 0b101, false);
    CHECK_TRUE("no cs_via1", !d.cs_via1);
    CHECK_TRUE("no fault",   !d.fault);
    CHECK_TRUE("no priv_violate", !d.priv);
    CHECK_EQ  ("class=UNMAP (idle)", d.out_class, CLASS_UNMAP);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// 12 — MAME TurboSCSI windows route to cs_scsi and adjacent gaps fault
// ════════════════════════════════════════════════════════════════════════
static bool test_turboscsi_window() {
    reset();
    Decode d = decode(0x5000F000u, true, false, 0b101, false);
    CHECK_EQ  ("TurboSCSI class=IO", d.out_class, CLASS_IO);
    CHECK_TRUE("cs_scsi asserted",  d.cs_scsi);
    CHECK_TRUE("no fault",         !d.fault);
    d = decode(0x5000F0FCu, true, false, 0b101, false);
    CHECK_TRUE("cs_scsi asserted at reg top word", d.cs_scsi);
    CHECK_TRUE("no fault at reg top word", !d.fault);
    d = decode(0x5000F100u, true, false, 0b101, false);
    CHECK_TRUE("cs_scsi asserted at DMA shim", d.cs_scsi);
    CHECK_TRUE("no fault at DMA shim", !d.fault);
    d = decode(0x5000F102u, true, false, 0b101, false);
    CHECK_TRUE("fault just past DMA shim", d.fault);
    CHECK_TRUE("no cs_scsi just past DMA shim", !d.cs_scsi);
    d = decode(0x50F0F000u, true, false, 0b101, false);
    CHECK_EQ  ("TurboSCSI mirror class=IO", d.out_class, CLASS_IO);
    CHECK_TRUE("cs_scsi asserted mirror", d.cs_scsi);
    CHECK_TRUE("no fault mirror", !d.fault);
    d = decode(0x50F0F400u, true, false, 0b101, false);
    CHECK_TRUE("fault on mirrored SCSI page gap", d.fault);
    CHECK_TRUE("no cs_scsi on mirrored SCSI page gap", !d.cs_scsi);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// 13 — Directed first-boot map transition
// ════════════════════════════════════════════════════════════════════════
static bool test_boot_time_map_transition() {
    reset();

    // Reset vectors are fetched through the low ROM overlay.
    Decode d = decode(0x00000000u, true, false, 0b110, true);
    CHECK_EQ  ("reset vector class=ROM", d.out_class, CLASS_ROM);
    CHECK_EQ  ("reset vector aliases ROM base", d.out_addr, 0x40000000u);
    CHECK_TRUE("reset vector cs_rom", d.cs_rom);
    CHECK_TRUE("reset vector no fault", !d.fault);

    d = decode(0x00000004u, true, false, 0b110, true);
    CHECK_EQ  ("reset PC vector class=ROM", d.out_class, CLASS_ROM);
    CHECK_EQ  ("reset PC vector aliases ROM+4", d.out_addr, 0x40000004u);

    // The ROM can still reach VIA1 through the MAME mirror to clear ORB[3].
    d = decode(0x50F00000u, true, true, 0b101, true);
    CHECK_EQ  ("VIA1 clear write class=IO", d.out_class, CLASS_IO);
    CHECK_TRUE("VIA1 clear write cs_via1", d.cs_via1);
    CHECK_TRUE("VIA1 clear write no fault", !d.fault);

    // After VIA1 overlay_in falls, low vectors are RAM, but high ROM remains ROM.
    d = decode(0x00000000u, true, false, 0b101, false);
    CHECK_EQ  ("post-clear low class=RAM", d.out_class, CLASS_RAM);
    CHECK_EQ  ("post-clear low addr=0", d.out_addr, 0x00000000u);
    CHECK_TRUE("post-clear low cs_ram", d.cs_ram);
    CHECK_TRUE("post-clear low cs_rom clear", !d.cs_rom);

    d = decode(0x40000000u, true, false, 0b110, false);
    CHECK_EQ  ("native ROM remains ROM", d.out_class, CLASS_ROM);
    CHECK_EQ  ("native ROM pass-through", d.out_addr, 0x40000000u);
    CHECK_TRUE("native ROM cs_rom", d.cs_rom);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// 14 — Unowned 0xF9 gaps fault while the two live ranges do not
// ════════════════════════════════════════════════════════════════════════
static bool test_video_gap_faults() {
    reset();

    // VRAM pixel aperture is 2 MB (0xF900_0000..0xF91F_FFFF) — matches
    // Q700 silicon (AXI_VRAM_SIZE in axi_defs.vh); glue.v's
    // VRAM_APERTURE_END must track that (build-truth-hygiene fix: glue.v
    // previously bounded this at 1 MB / 0xF910_0000).
    Decode d = decode(0xF91FFFFCu, true, true, 0b101, false);
    CHECK_EQ  ("VRAM top class=VIDEO", d.out_class, CLASS_VIDEO);
    CHECK_TRUE("VRAM top no fault", !d.fault);
    CHECK_TRUE("VRAM top no cs_video", !d.cs_video);

    d = decode(0xF9200000u, true, true, 0b101, false);
    CHECK_EQ  ("after VRAM class=VIDEO", d.out_class, CLASS_VIDEO);
    CHECK_TRUE("after VRAM faults", d.fault);
    CHECK_TRUE("after VRAM no cs_video", !d.cs_video);

    d = decode(0xF98003FCu, true, false, 0b101, false);
    CHECK_EQ  ("DAFB top class=VIDEO", d.out_class, CLASS_VIDEO);
    CHECK_TRUE("DAFB top cs_video", d.cs_video);
    CHECK_TRUE("DAFB top no fault", !d.fault);

    d = decode(0xF9800400u, true, false, 0b101, false);
    CHECK_EQ  ("after DAFB class=VIDEO", d.out_class, CLASS_VIDEO);
    CHECK_TRUE("after DAFB faults", d.fault);
    CHECK_TRUE("after DAFB no cs_video", !d.cs_video);
    return true;
}

// ════════════════════════════════════════════════════════════════════════
// Main
// ════════════════════════════════════════════════════════════════════════
#define RUN(fn) do { \
    bool ok = fn(); \
    if (ok) { printf("[PASS] " #fn "\n"); n_pass++; } \
    else    { printf("[FAIL] " #fn "\n"); n_fail++; } \
} while(0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vglue;

    RUN(test_overlay_on_aliases_rom);
    RUN(test_overlay_off_is_ram);
    RUN(test_q700_ram_probe_alias);
    RUN(test_rom_mirror_regardless_of_overlay);
    RUN(test_io_chip_selects);
    RUN(test_swim_and_empty_mirror_gap);
    RUN(test_q700_via_stride_windows);
    RUN(test_video_framebuffer);
    RUN(test_io_unmapped_fault);
    RUN(test_toplevel_unmapped_fault);
    RUN(test_adjacent_probe_gaps_do_not_alias);
    RUN(test_priv_violate);
    RUN(test_overlay_transition);
    RUN(test_req_invalid_quiesces);
    RUN(test_turboscsi_window);
    RUN(test_boot_time_map_transition);
    RUN(test_video_gap_faults);

    printf("\n%d/%d scenarios passed.\n", n_pass, n_pass + n_fail);

    dut->final();
    delete dut;
    return (n_fail == 0) ? 0 : 1;
}
