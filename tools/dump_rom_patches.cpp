// dump_rom_patches.cpp — emit a "<offset_hex> <byte_hex>" text patch list
// for a comma-separated rom_patch_sets.h selector.
//
// Mirrors the patch-application logic in tb_fpga_top_rom.cpp's
// apply_rom_patch_set() so the MAME-side capture can apply the SAME
// byte-level edits to its in-memory `:bootrom` region (via
// $MAME_AXI_PATCH_FILE in mame_axi_capture.lua).  This keeps the two
// captures running on the same logical ROM image — without modifying the
// on-disk ROM file (MAME's checksum check would otherwise reject it).
//
// Usage:
//     dump_rom_patches <patch_set_list> [out_file]
//
// where <patch_set_list> is the same comma-separated string accepted by
// the harness's +rom_patch flag (e.g. "mame-fastdiag,chime-skip").  If
// out_file is omitted output goes to stdout.
//
// Exit codes:
//     0  success
//     1  unknown patch set (matches add_rom_patch_set's contract)
//     2  bad command line / I/O error

#include "rom_patch_sets.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static bool apply_one(std::vector<RomPatchByte>& patches,
                      const std::string& set) {
    if (set.empty()) return true;
    // Every set (chime-skip included) flows through the shared
    // add_rom_patch_set definition in rom_patch_sets.h — single source
    // of truth, so the MAME-side and RTL-side patch streams stay
    // byte-identical, checksum compensation included.
    return add_rom_patch_set(patches, set, "[dump-rom-patches]");
}

static bool apply_list(std::vector<RomPatchByte>& patches,
                       const std::string& list) {
    size_t start = 0;
    while (start <= list.size()) {
        const size_t comma = list.find(',', start);
        const std::string one = list.substr(
            start, comma == std::string::npos ? std::string::npos : comma - start);
        if (!apply_one(patches, one)) return false;
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return true;
}

int main(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        std::fprintf(stderr,
            "usage: dump_rom_patches <patch_set_list> [out_file]\n");
        return 2;
    }
    std::vector<RomPatchByte> patches;
    if (!apply_list(patches, argv[1])) return 1;

    std::FILE* out = stdout;
    if (argc == 3) {
        out = std::fopen(argv[2], "w");
        if (!out) {
            std::perror(argv[2]);
            return 2;
        }
    }
    std::fprintf(out, "# rom-patch dump for set=\"%s\" (%zu byte patches)\n",
                 argv[1], patches.size());
    for (const auto& p : patches) {
        std::fprintf(out, "0x%05x 0x%02x  # %s: %s\n",
                     p.off, p.value,
                     p.set ? p.set : "",
                     p.description ? p.description : "");
    }
    if (out != stdout) std::fclose(out);
    return 0;
}
