#!/usr/bin/env bash
# Part 76: full from-reset, full-SoC cpu040 boot simulation using a
# diagnostic-loop-shortened ROM (cpu submodule branch
# feat/calibration-fix-rom-patch, tb/models/rom_patch_sets.h patch set
# "diagnostic-loop-skip-sim-only"), for simulation-throughput acceleration
# ONLY -- not a hardware-valid ROM image.  See
# docs/BUG_calibration_word_misplaced_0d00.md Part 76 for the full writeup.
#
# This is a straight variant of tools/part72_run_full_boot_sim.sh: same
# binary, same plusarg shape, same 10M-cycle checkpoint cadence -- the ONLY
# difference is +rom= points at the patched ROM image and the output
# directory is separate, so this run is fully independent of (and never
# touches) the original unpatched-ROM run under build/part72_full_boot_sim/.
#
# Usage:
#   tools/part76_run_patched_boot_sim.sh [outdir]
#   tools/part76_run_patched_boot_sim.sh <outdir> --resume
#
# Regenerating the patched ROM this script expects (only needed once, or
# after rom_patch_sets.h changes):
#   cd cpu && g++ -O2 -std=c++17 -Itb/models -o build/mame_patch_rom tools/mame_patch_rom.cpp
#   cd .. && mkdir -p build/roms_patched
#   cpu/build/mame_patch_rom --in files/420dbff3.rom \
#       --out build/roms_patched/420dbff3_diagloopskip.rom \
#       --patch diagnostic-loop-skip-sim-only
#
# +pc_dump_path (2026-09-02, coordinator-requested): defaults to /dev/shm
# (tmpfs), not $OUTDIR, and +pc_dump_every=200 samples instead of logging
# every single retirement. A full multi-hundred-million-instruction boot
# logging EVERY retirement to a real NVMe-backed file (as the original
# Part 72/76 runs did -- ~29 bytes/line, hundreds of MB and climbing) is
# real, continuous write load for a trace that's only ever grepped after
# the fact, not needed for correctness (checkpoints already carry full
# state) -- override PC_DUMP_DIR=/some/other/path or PC_DUMP_EVERY=1 if a
# dense trace is genuinely needed for one investigation round.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

OUTDIR_ARG="${1:-build/part76_patched_boot_sim}"
RESUME="${2:-}"
OUTDIR="$REPO_ROOT/$OUTDIR_ARG"
mkdir -p "$OUTDIR/checkpoints"

BIN="$REPO_ROOT/build/fpga_top_rom/Vfpga_top"
ROM="$REPO_ROOT/build/roms_patched/420dbff3_diagloopskip.rom"
if [ ! -f "$ROM" ]; then
    echo "patched ROM not found at $ROM -- see this script's header for how to regenerate it" >&2
    exit 1
fi
# CPU=m68k040 requires running from cpu040/generated/ so Spinal's
# $readmemb paths (relative to the generated-Verilog dir) resolve --
# see Makefile's CPU_MODEL_RUN_PREFIX.  All other paths below are
# absolute so the cd doesn't break them.
RUNDIR="$REPO_ROOT/cpu040/generated"

PC_DUMP_DIR="${PC_DUMP_DIR:-/dev/shm}"
PC_DUMP_EVERY="${PC_DUMP_EVERY:-200}"
mkdir -p "$PC_DUMP_DIR"

EXTRA_ARGS=(
    +rom="$ROM"
    +checkpoint_dir="$OUTDIR/checkpoints"
    +checkpoint_interval_cycles=10000000
    +pc_dump_path="$PC_DUMP_DIR/$(basename "$OUTDIR")_pc_dump.txt"
    +pc_dump_max=20000000
    +pc_dump_every="$PC_DUMP_EVERY"
    +exc_log=200000
    +max_insts=3000000000
    +timeout=20000000000
    +ppm="$OUTDIR/scaler_scanout.ppm"
)

if [ "$RESUME" = "--resume" ]; then
    LATEST_FILE="$OUTDIR/checkpoints/latest.txt"
    if [ ! -f "$LATEST_FILE" ]; then
        echo "no checkpoint found at $LATEST_FILE" >&2
        exit 1
    fi
    CKPT_BASE="$(cat "$LATEST_FILE")"
    echo "resuming from checkpoint: $CKPT_BASE" >&2
    EXTRA_ARGS+=( +load_checkpoint="$CKPT_BASE" )
fi

cd "$RUNDIR"
exec nice -n 15 "$BIN" "${EXTRA_ARGS[@]}" >> "$OUTDIR/run.log" 2>&1
