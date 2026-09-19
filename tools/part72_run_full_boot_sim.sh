#!/usr/bin/env bash
# Part 72: full from-reset, full-SoC cpu040 boot simulation with periodic
# full-design checkpointing.  See docs/BUG_calibration_word_misplaced_0d00.md
# Part 72 for the full writeup.
#
# Usage:
#   tools/part72_run_full_boot_sim.sh [outdir]
#
# Resuming from the latest checkpoint (after a kill/crash/session end):
#   tools/part72_run_full_boot_sim.sh <outdir> --resume
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

OUTDIR_ARG="${1:-build/part72_full_boot_sim}"
RESUME="${2:-}"
OUTDIR="$REPO_ROOT/$OUTDIR_ARG"
mkdir -p "$OUTDIR/checkpoints"

BIN="$REPO_ROOT/build/fpga_top_rom/Vfpga_top"
ROM="$REPO_ROOT/files/420dbff3.rom"
# CPU=m68k040 requires running from cpu040/generated/ so Spinal's
# $readmemb paths (relative to the generated-Verilog dir) resolve --
# see Makefile's CPU_MODEL_RUN_PREFIX.  All other paths below are
# absolute so the cd doesn't break them.
RUNDIR="$REPO_ROOT/cpu040/generated"

EXTRA_ARGS=(
    +rom="$ROM"
    +checkpoint_dir="$OUTDIR/checkpoints"
    +checkpoint_interval_cycles=10000000
    +pc_dump_path="$OUTDIR/pc_dump.txt"
    +pc_dump_max=20000000
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
