// mame_patch_rom.cpp - apply existing ROM patch sets to a copy for MAME runs.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "rom_patch_sets.h"

namespace {

std::vector<uint8_t> read_file(const char* path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::fprintf(stderr, "mame_patch_rom: cannot open input %s\n", path);
        std::exit(2);
    }
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)),
                                std::istreambuf_iterator<char>());
}

void write_file(const char* path, const std::vector<uint8_t>& data) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        std::fprintf(stderr, "mame_patch_rom: cannot open output %s\n", path);
        std::exit(2);
    }
    out.write(reinterpret_cast<const char*>(data.data()), (std::streamsize)data.size());
    if (!out) {
        std::fprintf(stderr, "mame_patch_rom: write failed for %s\n", path);
        std::exit(2);
    }
}

void add_patch_sets(std::vector<RomPatchByte>& patches, const std::string& spec) {
    std::stringstream ss(spec);
    std::string set;
    while (std::getline(ss, set, ',')) {
        if (set.empty())
            continue;
        if (!add_rom_patch_set(patches, set, "[mame-patch-rom]"))
            std::exit(2);
    }
}

} // namespace

int main(int argc, char** argv) {
    const char* in_path = nullptr;
    const char* out_path = nullptr;
    std::string patch_spec = "mame-firstlight";

    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--in") == 0 && i + 1 < argc) {
            in_path = argv[++i];
        } else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            out_path = argv[++i];
        } else if (std::strcmp(argv[i], "--patch") == 0 && i + 1 < argc) {
            patch_spec = argv[++i];
        } else {
            std::fprintf(stderr,
                "usage: %s --in ROM --out ROM [--patch set[,set...]]\n",
                argv[0]);
            return 2;
        }
    }

    if (!in_path || !out_path) {
        std::fprintf(stderr,
            "usage: %s --in ROM --out ROM [--patch set[,set...]]\n",
            argv[0]);
        return 2;
    }

    std::vector<uint8_t> rom = read_file(in_path);
    std::vector<RomPatchByte> patches;
    add_patch_sets(patches, patch_spec);

    for (const RomPatchByte& patch : patches) {
        if (patch.off >= rom.size()) {
            std::fprintf(stderr,
                "mame_patch_rom: patch out of range set=%s off=0x%05x size=%zu\n",
                patch.set, patch.off, rom.size());
            return 2;
        }
        rom[patch.off] = patch.value;
    }

    write_file(out_path, rom);
    std::printf("mame_patch_rom wrote %s from %s patch=%s bytes=%zu patches=%zu\n",
                out_path, in_path, patch_spec.c_str(), rom.size(), patches.size());
    return 0;
}
