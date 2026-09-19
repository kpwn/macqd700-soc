#ifndef M68K_OOO_ROM_PATCH_SETS_H
#define M68K_OOO_ROM_PATCH_SETS_H

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

struct RomPatchByte {
    uint32_t off;
    uint8_t value;
    const char* set;
    const char* description;
};

static inline void rom_patch_byte(std::vector<RomPatchByte>& patches,
                                  uint32_t off,
                                  uint8_t value,
                                  const char* set,
                                  const char* description) {
    patches.push_back({off, value, set, description});
}

static inline void rom_patch_blob(std::vector<RomPatchByte>& patches,
                                  uint32_t off,
                                  const uint8_t* bytes,
                                  std::size_t len,
                                  const char* set,
                                  const char* description) {
    for (std::size_t i = 0; i < len; i++)
        rom_patch_byte(patches, off + (uint32_t)i, bytes[i],
                       set, description);
}

static inline void rom_patch_word(std::vector<RomPatchByte>& patches,
                                  uint32_t off,
                                  uint16_t value,
                                  const char* set,
                                  const char* description) {
    rom_patch_byte(patches, off, (uint8_t)(value >> 8), set, description);
    rom_patch_byte(patches, off + 1, (uint8_t)value, set, description);
}

static inline void rom_patch_long(std::vector<RomPatchByte>& patches,
                                  uint32_t off,
                                  uint32_t value,
                                  const char* set,
                                  const char* description) {
    rom_patch_word(patches, off, (uint16_t)(value >> 16), set, description);
    rom_patch_word(patches, off + 2, (uint16_t)value, set, description);
}

static inline void rom_patch_nop_word(std::vector<RomPatchByte>& patches,
                                      uint32_t off,
                                      const char* set,
                                      const char* description) {
    rom_patch_word(patches, off, 0x4e71u, set, description);
}

static inline void rom_patch_return(std::vector<RomPatchByte>& patches,
                                    uint32_t off,
                                    const char* set,
                                    const char* description) {
    rom_patch_word(patches, off, 0x4ed6u, set, description);
}

static inline void add_rom_checksum_fast_patches(
        std::vector<RomPatchByte>& patches) {
    const char* set = "checksum-fast";

    rom_patch_nop_word(patches, 0x474dcu, set,
        "bound byte-lane checksum loop after first sample");
    rom_patch_word(patches, 0x474deu, 0xb080u, set,
        "force byte-lane checksum lane 0 compare success");
    rom_patch_word(patches, 0x474e6u, 0xb281u, set,
        "force byte-lane checksum lane 1 compare success");
    rom_patch_word(patches, 0x474eeu, 0xb482u, set,
        "force byte-lane checksum lane 2 compare success");
    rom_patch_word(patches, 0x474f6u, 0xb683u, set,
        "force byte-lane checksum lane 3 compare success");

    rom_patch_nop_word(patches, 0x4751cu, set,
        "bound word-sum checksum loop after first sample");
    rom_patch_word(patches, 0x47522u, 0xb281u, set,
        "force word-sum checksum compare success");
}

static inline void add_rom_meminit_fast_patches(
        std::vector<RomPatchByte>& patches) {
    const char* set = "meminit-fast";

    rom_patch_nop_word(patches, 0x472b0u, set,
        "bound forward RAM fill loop");
    rom_patch_nop_word(patches, 0x472c2u, set,
        "bound forward RAM tail-fill loop");
    rom_patch_nop_word(patches, 0x47334u, set,
        "bound forward RAM verify DBF loop");
    rom_patch_nop_word(patches, 0x47336u, set,
        "bound forward RAM verify DBF loop");
    rom_patch_nop_word(patches, 0x4733eu, set,
        "bound forward RAM verify long loop");
    rom_patch_word(patches, 0x47340u, 0x6000u, set,
        "exit forward RAM helper after bounded successful sample");
    rom_patch_word(patches, 0x47342u, 0x003au, set,
        "exit forward RAM helper after bounded successful sample");
    rom_patch_word(patches, 0x47348u, 0xb281u, set,
        "force forward RAM retry compare success");
    rom_patch_word(patches, 0x47366u, 0x8c86u, set,
        "preserve D6 while accepting forward RAM sample");

    rom_patch_nop_word(patches, 0x473bcu, set,
        "bound reverse RAM fill loop");
    rom_patch_nop_word(patches, 0x473cau, set,
        "bound reverse RAM tail-fill loop");
    rom_patch_nop_word(patches, 0x4743eu, set,
        "bound reverse RAM verify DBF loop");
    rom_patch_nop_word(patches, 0x47440u, set,
        "bound reverse RAM verify DBF loop");
    rom_patch_nop_word(patches, 0x47448u, set,
        "bound reverse RAM verify long loop");
    rom_patch_word(patches, 0x4744au, 0x6000u, set,
        "exit reverse RAM helper after bounded successful sample");
    rom_patch_word(patches, 0x4744cu, 0x0036u, set,
        "exit reverse RAM helper after bounded successful sample");
    rom_patch_word(patches, 0x47452u, 0xb281u, set,
        "force reverse RAM retry compare success");
    rom_patch_word(patches, 0x4746eu, 0x8c86u, set,
        "preserve D6 while accepting reverse RAM sample");

    rom_patch_nop_word(patches, 0x4753eu, set,
        "bound walking-pattern zero-fill loop");
    rom_patch_nop_word(patches, 0x4754cu, set,
        "bound walking-pattern forward invert loop");
    rom_patch_nop_word(patches, 0x47556u, set,
        "bound walking-pattern reverse verify loop");
    rom_patch_nop_word(patches, 0x47562u, set,
        "bound walking-pattern forward verify loop");
    rom_patch_nop_word(patches, 0x4756cu, set,
        "bound walking-pattern reverse restore loop");
    rom_patch_nop_word(patches, 0x47578u, set,
        "bound walking-pattern final verify loop");

    rom_patch_nop_word(patches, 0x47604u, set,
        "bound single-address walking-bit loop");
    rom_patch_nop_word(patches, 0x47606u, set,
        "bound single-address walking-bit loop");
}

static inline void add_rom_ramtest_mame_state_patches(
        std::vector<RomPatchByte>& patches) {
    const char* set = "ramtest-mame-state";

    rom_patch_word(patches, 0x47280u, 0x7000u, set,
        "MAME-derived forward RAM helper fast path: D0=0");
    rom_patch_word(patches, 0x47282u, 0x7200u, set,
        "MAME-derived forward RAM helper fast path: D1=0");
    rom_patch_word(patches, 0x47284u, 0x7400u, set,
        "MAME-derived forward RAM helper fast path: D2=0");
    rom_patch_word(patches, 0x47286u, 0x263cu, set,
        "MAME-derived forward RAM helper fast path: D3=0x6db6db6d");
    rom_patch_long(patches, 0x47288u, 0x6db6db6du, set,
        "MAME-derived forward RAM helper fast path: D3=0x6db6db6d");
    rom_patch_word(patches, 0x4728cu, 0x283cu, set,
        "MAME-derived forward RAM helper fast path: D4=0xb6db6db6");
    rom_patch_long(patches, 0x4728eu, 0xb6db6db6u, set,
        "MAME-derived forward RAM helper fast path: D4=0xb6db6db6");
    rom_patch_word(patches, 0x47292u, 0x2a3cu, set,
        "MAME-derived forward RAM helper fast path: D5=0xdb6db6db");
    rom_patch_long(patches, 0x47294u, 0xdb6db6dbu, set,
        "MAME-derived forward RAM helper fast path: D5=0xdb6db6db");
    rom_patch_word(patches, 0x47298u, 0x7c00u, set,
        "MAME-derived forward RAM helper fast path: D6=0, CCR=Z");
    rom_patch_word(patches, 0x4729au, 0x45e9u, set,
        "MAME-derived forward RAM helper fast path: A2=A1-4");
    rom_patch_word(patches, 0x4729cu, 0xfffcu, set,
        "MAME-derived forward RAM helper fast path: A2=A1-4");
    rom_patch_word(patches, 0x4729eu, 0x46fcu, set,
        "MAME-derived forward RAM helper fast path: SR=0x2714");
    rom_patch_word(patches, 0x472a0u, 0x2714u, set,
        "MAME-derived forward RAM helper fast path: SR=0x2714");
    rom_patch_word(patches, 0x472a2u, 0x4ed6u, set,
        "MAME-derived forward RAM helper fast path: return through A6");
}

static inline void add_rom_alias_probe_mame_state_patches(
        std::vector<RomPatchByte>& patches) {
    const char* set = "alias-probe-mame-state";

    // MAME macqd700 4 MB state after the RAM lane/alias probe that starts at
    // 0x4084bb74.  The skipped probe restores low memory, reports D0=4 MB,
    // records one good RAM nibble in D5, leaves D6 clear, and continues at
    // 0x4084bc38.  D7 distinguishes the two observed early calls, so preserve
    // the MAME low-memory restore for both call sites before installing the
    // common post-probe architectural state.
    static const uint8_t code[] = {
        0x0c, 0x07, 0x00, 0x13, // cmpi.b #0x13,d7
        0x67, 0x06,             // beq.s second_restore
        0x76, 0xff,             // moveq #-1,d3
        0x78, 0xff,             // moveq #-1,d4
        0x60, 0x0c,             // bra.s store_lowmem
        0x26, 0x3c, 0x88, 0x88, 0x88, 0x88, // second: d3=0x88888888
        0x28, 0x3c, 0x00, 0x88, 0x88, 0x88, // d4=0x00888888
        0x20, 0x7c, 0x00, 0x00, 0x00, 0x00, // a0=0
        0x20, 0x83,                         // (a0)=d3
        0x21, 0x44, 0x00, 0x04,             // 4(a0)=d4
        0x2a, 0x7c, 0x40, 0x80, 0x3b, 0xb8, // a5=0x40803bb8
        0x3e, 0x7c, 0x00, 0x10,             // sp=0x10
        0x20, 0x3c, 0x00, 0x40, 0x00, 0x00, // d0=0x00400000
        0x22, 0x3c, 0xab, 0x96, 0x91, 0x9e, // d1=0xab96919e
        0x74, 0x08,                         // d2=8
        0x76, 0x0f,                         // d3=15
        0x78, 0x00,                         // d4=0
        0x7a, 0x01,                         // d5=1
        0x7c, 0x00,                         // d6=0
        0x20, 0x7c, 0xff, 0xff, 0xff, 0xff, // a0=-1
        0x22, 0x7c, 0x08, 0x00, 0x00, 0x00, // a1=0x08000000
        0x2c, 0x7c, 0x40, 0x84, 0xbb, 0xa0, // a6=0x4084bba0
        0x46, 0xfc, 0x27, 0x04,             // sr=0x2704
        0x4e, 0xf9, 0x40, 0x84, 0xbc, 0x38, // jmp 0x4084bc38
    };
    rom_patch_blob(patches, 0x4bb74u, code, sizeof(code), set,
        "MAME-derived RAM lane/alias probe fast path");
}

static inline void add_rom_ram_list_sentinel_fast_patches(
        std::vector<RomPatchByte>& patches) {
    const char* set = "ram-list-sentinel-fast";

    // The MAME-state RAM shortcut has already performed the sole helper range
    // before this descriptor-list walk resumes.  Avoid the cached descriptor
    // load in full-fpga-top sims and synthesize the following sentinel entry:
    // D0=-1; A0=-1; D0+1 sets Z; the existing BEQ.S takes the list-end path.
    rom_patch_word(patches, 0x46ed0u, 0x70ffu, set,
        "RAM descriptor sentinel fast path: synthesize D0=-1");
    rom_patch_word(patches, 0x46ed2u, 0x2040u, set,
        "RAM descriptor sentinel fast path: synthesize A0=-1");
    rom_patch_word(patches, 0x46ed4u, 0x5280u, set,
        "RAM descriptor sentinel fast path: set Z for A0=-1");
}

static inline void add_rom_macsbug_feature_bit17_patches(
        std::vector<RomPatchByte>& patches) {
    const char* set = "macsbug-feature-bit17";

    // The Q700 ROM's MacsBug REPL serial-output / serial-input routines are
    // gated on D0[17]:
    //
    //   0x4084ae3a rom_monitor_desc:    BTST #17,%D0; BEQ → skip descriptor
    //   0x4084ae56 rom_monitor_banner:  BTST #17,%D0; BEQ → skip banner
    //   0x4084afa0 rom_scc_rx_poll:     BTST #17,%D7; BEQ → skip RX FIFO read
    //
    // D0[17] is also tested at 0x4084781a / 0x40846c66 / 0x40847f9c during
    // the early SCC self-test path: when D0[17]=1 the code uses
    // *(A0+0x44) as the secondary monitor I/O base, otherwise it uses
    // *(A0+0x0c) and busy-waits on bit 0 of (A3) — which on our sim
    // resolves to 0x50f0c022 (SCC chan-A control) and never fires
    // RX_CHAR_AVAIL because no host byte is en route during boot.
    //
    // The Q700 universal-info descriptor at 0x4080390c stores the platform
    // feature bitmap in a 32-bit field at +0x18 (file offset 0x3924).  In
    // the unmodified ROM that field is 0x05a0_183f — bit 17 is clear,
    // even though the Q700 ROM clearly expects it to be set when the
    // monitor I/O descriptor at A0+0x44 has been wired up.
    //
    // The "feature-detect" path that would dynamically set bit 17 lives at
    // 0x40802fec..0x40803012 (BSET #17 on success of the JSR at 0x4080477a),
    // but that dispatcher only runs when the matched descriptor's D0
    // feature bitmap is 0 — for any *known* machine the ROM trusts the
    // bitmap as-is.  The Q700 descriptor's bitmap omits bit 17, which
    // matches the silicon (which never reaches MacsBug from a normal
    // boot since there's no programmer's switch wired to the IRQ
    // controller in our sim setup) but breaks the post-NMI MacsBug
    // serial path completely.
    //
    // OR bit 17 (0x00020000) into the descriptor's D0 feature long.
    // Byte +0x18 of the descriptor is at file offset 0x3924; the bit
    // we want is the high half of the upper-mid byte (offset 0x3925),
    // which currently reads 0xa0 → 0xa2.
    rom_patch_byte(patches, 0x3925u, 0xa2, set,
        "Q700 desc feature D0: set bit 17 (MacsBug serial I/O present)");
}

static inline void add_rom_via_timer_mame_state_patches(
        std::vector<RomPatchByte>& patches) {
    const char* set = "via-timer-mame-state";

    // MAME macqd700 state for the VIA timer IRQ diagnostic at 0x40847bf6.
    // The harness currently advances the VIA timer model but does not feed
    // that interrupt into the core, so the ROM never reaches its handler at
    // 0x40847d06.  Preserve the ROM's validation path by recreating the
    // MAME-observed loop-exit counters instead of jumping around the checks.
    static const uint8_t first_wait[] = {
        0x76, 0x0a,             // moveq #10,d3
        0x38, 0x3c, 0x00, 0xa5, // move.w #0x00a5,d4
        0x7a, 0x01,             // moveq #1,d5
        0x4e, 0x71,             // nop
    };
    rom_patch_blob(patches, 0x47bf6u, first_wait, sizeof(first_wait), set,
        "MAME-derived VIA timer first wait exit: D3=10,D4=0x00a5,D5=1");

    static const uint8_t second_wait[] = {
        0x76, 0x0a,             // moveq #10,d3
        0x78, 0x01,             // moveq #1,d4
        0x7a, 0x01,             // moveq #1,d5
        0x4e, 0x71,             // nop
        0x4e, 0x71,             // nop
    };
    rom_patch_blob(patches, 0x47caau, second_wait, sizeof(second_wait), set,
        "MAME-derived VIA timer second wait exit: D3=10,D4=1,D5=1");
}

static inline void add_rom_adb_init_wait_patches(
        std::vector<RomPatchByte>& patches) {
    const char* set = "adb-init-wait";

    // MAME macqd700 with the repository PIC still spends long wall time in
    // the ADB manager init wait at 0x4080a8e6:
    //   btst #5,0x015d(a3); bne.s 0x4080a8e6
    // NOP only that busy-wait so first-light scouting can reach later ROM
    // probes without changing the ADB setup code that precedes it.
    rom_patch_nop_word(patches, 0x0a8e6u, set,
        "skip ADB manager init busy-wait status test");
    rom_patch_nop_word(patches, 0x0a8e8u, set,
        "skip ADB manager init busy-wait status test");
    rom_patch_nop_word(patches, 0x0a8eau, set,
        "skip ADB manager init busy-wait status operand");
    rom_patch_nop_word(patches, 0x0a8ecu, set,
        "skip ADB manager init busy-wait branch");
}

static inline void add_rom_video_diag_success_patches(
        std::vector<RomPatchByte>& patches) {
    const char* set = "video-diag-success";

    // The video/VRAM diagnostic returns with D6 carrying the accumulated
    // failure mask.  Preserve the diagnostic's memory side effects, but force
    // the result clear at its return boundary for first-light scouting.
    rom_patch_word(patches, 0x49adeu, 0x4286u, set,
        "force video/VRAM diagnostic success before ROM monitor gate");
}

static inline void add_rom_sad_mac_boot_skip_patches(
        std::vector<RomPatchByte>& patches) {
    const char* set = "sad-mac-boot-skip";

    // In the boot component loop, a component failure with D7 bit 26 set
    // branches into the Sad Mac/ROM monitor path.  For first-light scouting,
    // keep the component side effects and continue scanning the remaining
    // entries instead of handing control to the monitor.
    rom_patch_nop_word(patches, 0x46f96u, set,
        "skip Sad Mac branch after boot component failure");
    rom_patch_nop_word(patches, 0x46f98u, set,
        "skip Sad Mac branch after boot component failure");
    rom_patch_nop_word(patches, 0x46facu, set,
        "skip Sad Mac branch after boot component scan");
    rom_patch_nop_word(patches, 0x46faeu, set,
        "skip Sad Mac branch after boot component scan");
}

static inline void add_rom_rtc_pram_mame_state_patches(
        std::vector<RomPatchByte>& patches) {
    const char* set = "rtc-pram-mame-state";

    // MAME macqd700 4 MB state at 0x40847280 after the RTC/PRAM sizing
    // loop: PC=40847280 SR=2718 D0=00400000 D1=00000900 D2=DC001008
    // D3=00400000 D4=40846EAE D5=FFFFFFFF D6=0 D7=3
    // A0=0 A1=003FFFD4 A2=40846FFA A3=FFFFFFFF A4=003FFFDC
    // A5=003FFFD4 A6=40846EF2 A7=00007FFC.
    rom_patch_word(patches, 0x4721cu, 0x203cu, set,
        "MAME-derived RTC/PRAM fast path: D0=4 MB");
    rom_patch_long(patches, 0x4721eu, 0x00400000u, set,
        "MAME-derived RTC/PRAM fast path: D0=4 MB");
    rom_patch_word(patches, 0x47222u, 0x223cu, set,
        "MAME-derived RTC/PRAM fast path: D1=0x900");
    rom_patch_long(patches, 0x47224u, 0x00000900u, set,
        "MAME-derived RTC/PRAM fast path: D1=0x900");
    rom_patch_word(patches, 0x47228u, 0x243cu, set,
        "MAME-derived RTC/PRAM fast path: D2=0xdc001008");
    rom_patch_long(patches, 0x4722au, 0xdc001008u, set,
        "MAME-derived RTC/PRAM fast path: D2=0xdc001008");
    rom_patch_word(patches, 0x4722eu, 0x263cu, set,
        "MAME-derived RTC/PRAM fast path: D3=4 MB");
    rom_patch_long(patches, 0x47230u, 0x00400000u, set,
        "MAME-derived RTC/PRAM fast path: D3=4 MB");
    rom_patch_word(patches, 0x47234u, 0x283cu, set,
        "MAME-derived RTC/PRAM fast path: D4=0x40846eae");
    rom_patch_long(patches, 0x47236u, 0x40846eaeu, set,
        "MAME-derived RTC/PRAM fast path: D4=0x40846eae");
    rom_patch_word(patches, 0x4723au, 0x2a3cu, set,
        "MAME-derived RTC/PRAM fast path: D5=-1");
    rom_patch_long(patches, 0x4723cu, 0xffffffffu, set,
        "MAME-derived RTC/PRAM fast path: D5=-1");
    rom_patch_word(patches, 0x47240u, 0x7c00u, set,
        "MAME-derived RTC/PRAM fast path: D6=0");
    rom_patch_word(patches, 0x47242u, 0x7e03u, set,
        "MAME-derived RTC/PRAM fast path: D7=3");
    rom_patch_word(patches, 0x47244u, 0x207cu, set,
        "MAME-derived RTC/PRAM fast path: A0=0");
    rom_patch_long(patches, 0x47246u, 0x00000000u, set,
        "MAME-derived RTC/PRAM fast path: A0=0");
    rom_patch_word(patches, 0x4724au, 0x227cu, set,
        "MAME-derived RTC/PRAM fast path: A1=0x003fffd4");
    rom_patch_long(patches, 0x4724cu, 0x003fffd4u, set,
        "MAME-derived RTC/PRAM fast path: A1=0x003fffd4");
    rom_patch_word(patches, 0x47250u, 0x247cu, set,
        "MAME-derived RTC/PRAM fast path: A2=0x40846ffa");
    rom_patch_long(patches, 0x47252u, 0x40846ffau, set,
        "MAME-derived RTC/PRAM fast path: A2=0x40846ffa");
    rom_patch_word(patches, 0x47256u, 0x267cu, set,
        "MAME-derived RTC/PRAM fast path: A3=-1");
    rom_patch_long(patches, 0x47258u, 0xffffffffu, set,
        "MAME-derived RTC/PRAM fast path: A3=-1");
    rom_patch_word(patches, 0x4725cu, 0x287cu, set,
        "MAME-derived RTC/PRAM fast path: A4=0x003fffdc");
    rom_patch_long(patches, 0x4725eu, 0x003fffdcu, set,
        "MAME-derived RTC/PRAM fast path: A4=0x003fffdc");
    rom_patch_word(patches, 0x47262u, 0x2a7cu, set,
        "MAME-derived RTC/PRAM fast path: A5=0x003fffd4");
    rom_patch_long(patches, 0x47264u, 0x003fffd4u, set,
        "MAME-derived RTC/PRAM fast path: A5=0x003fffd4");
    rom_patch_word(patches, 0x47268u, 0x2c7cu, set,
        "MAME-derived RTC/PRAM fast path: A6=0x40846ef2");
    rom_patch_long(patches, 0x4726au, 0x40846ef2u, set,
        "MAME-derived RTC/PRAM fast path: A6=0x40846ef2");
    rom_patch_word(patches, 0x4726eu, 0x2e7cu, set,
        "MAME-derived RTC/PRAM fast path: A7=0x00007ffc");
    rom_patch_long(patches, 0x47270u, 0x00007ffcu, set,
        "MAME-derived RTC/PRAM fast path: A7=0x00007ffc");
    rom_patch_word(patches, 0x47274u, 0x46fcu, set,
        "MAME-derived RTC/PRAM fast path: SR=0x2718");
    rom_patch_word(patches, 0x47276u, 0x2718u, set,
        "MAME-derived RTC/PRAM fast path: SR=0x2718");
    rom_patch_word(patches, 0x47278u, 0x6006u, set,
        "MAME-derived RTC/PRAM fast path: branch to 0x40847280");
}

static inline void add_legacy_diag_loop_patches(
        std::vector<RomPatchByte>& patches) {
    rom_patch_return(patches, 0x4749eu, "checksum-unsafe",
        "legacy whole-helper return from byte-lane ROM checksum helper");
    rom_patch_return(patches, 0x47500u, "checksum-unsafe",
        "legacy whole-helper return from word-sum ROM checksum helper");
    rom_patch_return(patches, 0x47280u, "diag-loops-unsafe",
        "legacy whole-helper return from forward RAM pattern helper");
    rom_patch_return(patches, 0x47398u, "diag-loops-unsafe",
        "legacy whole-helper return from reverse RAM pattern helper");
    rom_patch_return(patches, 0x4752cu, "diag-loops-unsafe",
        "legacy whole-helper return from RAM walking-pattern helper");
    rom_patch_return(patches, 0x475e8u, "diag-loops-unsafe",
        "legacy whole-helper return from single-address walking-bit helper");
}

static inline bool add_rom_patch_set(std::vector<RomPatchByte>& patches,
                                     const std::string& set,
                                     const char* log_prefix) {
    if (set == "checksum-fast") {
        add_rom_checksum_fast_patches(patches);
        return true;
    }
    if (set == "meminit-fast") {
        add_rom_meminit_fast_patches(patches);
        return true;
    }
    if (set == "ramtest-mame-state" || set == "mame-ramtest-fast") {
        add_rom_ramtest_mame_state_patches(patches);
        return true;
    }
    if (set == "rtc-pram-mame-state" || set == "mame-pram-fast") {
        add_rom_rtc_pram_mame_state_patches(patches);
        return true;
    }
    if (set == "alias-probe-mame-state" || set == "mame-alias-fast") {
        add_rom_alias_probe_mame_state_patches(patches);
        return true;
    }
    if (set == "ram-list-sentinel-fast") {
        add_rom_ram_list_sentinel_fast_patches(patches);
        return true;
    }
    if (set == "via-timer-mame-state" || set == "mame-via-timer-fast") {
        add_rom_via_timer_mame_state_patches(patches);
        return true;
    }
    if (set == "adb-init-wait" || set == "mame-adb-init-fast") {
        add_rom_adb_init_wait_patches(patches);
        return true;
    }
    if (set == "video-diag-success") {
        add_rom_video_diag_success_patches(patches);
        return true;
    }
    if (set == "sad-mac-boot-skip" || set == "monitor-skip") {
        add_rom_sad_mac_boot_skip_patches(patches);
        return true;
    }
    if (set == "macsbug-feature-bit17" || set == "monitor-feature-bit17") {
        add_rom_macsbug_feature_bit17_patches(patches);
        return true;
    }
    // Keep rtc-pram-mame-state opt-in: in native MAME it currently enters the
    // ROM serial monitor instead of the normal boot-device probe path.
    if (set == "mame-fastdiag" || set == "mame-q700-fastdiag") {
        add_rom_checksum_fast_patches(patches);
        add_rom_meminit_fast_patches(patches);
        add_rom_ramtest_mame_state_patches(patches);
        add_rom_alias_probe_mame_state_patches(patches);
        add_rom_ram_list_sentinel_fast_patches(patches);
        add_rom_via_timer_mame_state_patches(patches);
        return true;
    }
    if (set == "mame-firstlight" || set == "mame-q700-firstlight") {
        add_rom_checksum_fast_patches(patches);
        add_rom_meminit_fast_patches(patches);
        add_rom_ramtest_mame_state_patches(patches);
        add_rom_alias_probe_mame_state_patches(patches);
        add_rom_ram_list_sentinel_fast_patches(patches);
        add_rom_via_timer_mame_state_patches(patches);
        add_rom_adb_init_wait_patches(patches);
        rom_patch_nop_word(patches, 0x04160u, "scsi-delay",
            "skip SCSI status DBF delay loop at 0x4080415a");
        rom_patch_nop_word(patches, 0x04162u, "scsi-delay",
            "skip SCSI status DBF delay loop at 0x4080415a");
        rom_patch_nop_word(patches, 0x04174u, "scsi-delay",
            "skip SCSI status DBF delay loop at 0x4080416e");
        rom_patch_nop_word(patches, 0x04176u, "scsi-delay",
            "skip SCSI status DBF delay loop at 0x4080416e");
        rom_patch_nop_word(patches, 0x07118u, "chime-delay",
            "skip ASC chime inner DBF delay loop");
        rom_patch_nop_word(patches, 0x0711au, "chime-delay",
            "skip ASC chime inner DBF delay loop");
        rom_patch_nop_word(patches, 0x00888u, "timer-delay",
            "skip VIA timer DBF delay loop at 0x40800888");
        rom_patch_nop_word(patches, 0x0088au, "timer-delay",
            "skip VIA timer DBF delay loop at 0x40800888");
        rom_patch_nop_word(patches, 0x0089eu, "scc-delay",
            "skip SCC RR0 polling DBF delay loop at 0x4080089a");
        rom_patch_nop_word(patches, 0x008a0u, "scc-delay",
            "skip SCC RR0 polling DBF delay loop at 0x4080089a");
        return true;
    }
    if (set == "clean-fastdiag" || set == "diag-loops" ||
        set == "fastdiag") {
        add_rom_checksum_fast_patches(patches);
        add_rom_meminit_fast_patches(patches);
        return true;
    }
    if (set == "diag-loops-unsafe" || set == "fastdiag-unsafe") {
        add_legacy_diag_loop_patches(patches);
        return true;
    }
    if (set == "checksum" || set == "checksum-unsafe") {
        rom_patch_return(patches, 0x4749eu, "checksum-unsafe",
            "legacy whole-helper return from byte-lane ROM checksum helper");
        rom_patch_return(patches, 0x47500u, "checksum-unsafe",
            "legacy whole-helper return from word-sum ROM checksum helper");
        return true;
    }
    if (set == "chime-delay" || set == "no-chime") {
        rom_patch_nop_word(patches, 0x07118u, "chime-delay",
            "skip ASC chime inner DBF delay loop");
        rom_patch_nop_word(patches, 0x0711au, "chime-delay",
            "skip ASC chime inner DBF delay loop");
        // Checksum compensation (same mechanism as chime-skip below —
        // see that block's comment for the full derivation).  The two
        // words patched above sit inside the ROM self-checksum's summed
        // range (offset 4..0x100000), so leaving the stored checksum
        // (the first ROM longword) unpatched makes the word-sum compare
        // at 0x40847524 fail on ANY run that reaches it, diverting the
        // boot onto the checksum-error path — even though nothing is
        // actually wrong.  This was misdiagnosed as a real RTL data-read
        // corruption bug across a multi-week investigation chain (see
        // docs history / project_via1_lockstep_deepdive.md, 2026-07-05
        // session) before being closed-form-proven (pure Python re-sum
        // of the patched ROM bytes exactly reproduces the "wrong"
        // checksum RTL was computing, with zero residual) and then
        // independently confirmed by a clean no-patch RTL run computing
        // the correct 0x420dbff3.  0x420dbff3 (original) with the two
        // 0x07118/0x0711a words replaced by NOP (0x4e71) resums to
        // 0x420d0b0d — patch the golden value to match so chime-delay
        // is safe to use standalone (without checksum-fast /
        // mame-fastdiag) for checksum-integrity-sensitive runs, not just
        // control-flow comparisons.
        rom_patch_long(patches, 0x00000u, 0x420d0b0du, "chime-delay",
            "word-sum checksum compensation for the 0x7118/0x711a patch");
        return true;
    }
    if (set == "chime-skip") {
        rom_patch_return(patches, 0x0706eu, "chime-skip",
            "return from ASC chime routine");
        // Checksum compensation.  The Q700 ROM verifies itself with a
        // 32-bit word-sum (loop at 0x4084751c summing every ROM word
        // from offset 4 to the end; compare at 0x40847524).  The patch
        // above changes the word at offset 0x706e from 0x422b to 0x4ed6
        // — i.e. +0xCAB to the running sum — so without compensation the
        // compare fails, the ROM sets the bad-checksum flag (D6=0xffff
        // at 0x40847526) and the boot diverges onto the checksum-error
        // path.  The stored checksum is the first ROM longword (offsets
        // 0..3): it is read into D4 and is itself NOT part of the sum,
        // so bump it by the same +0xCAB to keep the compare balanced.
        // 0x420dbff3 + 0xCAB = 0x420dcc9e.  This makes chime-skip safe
        // to use on its own (without checksum-fast / mame-fastdiag).
        // The byte-lane checksum helper (0x474dc) is not run by this
        // ROM, so only the word-sum stored value needs adjusting.
        rom_patch_long(patches, 0x00000u, 0x420dcc9eu, "chime-skip",
            "word-sum checksum compensation for the 0x706e patch");
        return true;
    }
    if (set == "timer-delay" || set == "dbf-delay") {
        rom_patch_nop_word(patches, 0x00888u, "timer-delay",
            "skip VIA timer DBF delay loop at 0x40800888");
        rom_patch_nop_word(patches, 0x0088au, "timer-delay",
            "skip VIA timer DBF delay loop at 0x40800888");
        return true;
    }
    if (set == "scc-delay" || set == "serial-delay") {
        rom_patch_nop_word(patches, 0x0089eu, "scc-delay",
            "skip SCC RR0 polling DBF delay loop at 0x4080089a");
        rom_patch_nop_word(patches, 0x008a0u, "scc-delay",
            "skip SCC RR0 polling DBF delay loop at 0x4080089a");
        return true;
    }
    if (set == "scsi-delay") {
        rom_patch_nop_word(patches, 0x04160u, "scsi-delay",
            "skip SCSI status DBF delay loop at 0x4080415a");
        rom_patch_nop_word(patches, 0x04162u, "scsi-delay",
            "skip SCSI status DBF delay loop at 0x4080415a");
        rom_patch_nop_word(patches, 0x04174u, "scsi-delay",
            "skip SCSI status DBF delay loop at 0x4080416e");
        rom_patch_nop_word(patches, 0x04176u, "scsi-delay",
            "skip SCSI status DBF delay loop at 0x4080416e");
        return true;
    }

    std::fprintf(stderr,
        "%s unknown ROM patch set '%s' (supported: checksum-fast, "
        "meminit-fast, ramtest-mame-state, rtc-pram-mame-state, "
        "alias-probe-mame-state, ram-list-sentinel-fast, "
        "via-timer-mame-state, adb-init-wait, video-diag-success, "
        "sad-mac-boot-skip, macsbug-feature-bit17, mame-fastdiag, "
        "mame-firstlight, "
        "clean-fastdiag, diag-loops, "
        "chime-delay, chime-skip, timer-delay, scc-delay, scsi-delay, diag-loops-unsafe, "
        "checksum-unsafe)\n",
        log_prefix, set.c_str());
    return false;
}

#endif
