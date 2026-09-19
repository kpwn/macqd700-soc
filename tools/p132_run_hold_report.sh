#!/usr/bin/env bash
# Part 132 launcher: queue behind whatever holds the Vivado mutex, then measure
# the CPU clock domain's real hold slack on an already-routed checkpoint.
#
# Builds nothing, writes no bitstream, never touches the SD card or the board.
# Safe to run unattended; safe to re-run.
#
#   tools/p132_run_hold_report.sh [route.dcp] [outdir]
#
# Defaults point at the p123-reseed routed design -- a bitstream MEASURED to
# wedge at ROM PC 0x4084BECE on silicon, so its hold numbers are the ones that
# matter.
set -u

DCP="${1:-/home/qwertyoruiop/macqd700-soc-worktrees/p123-reseed/build/vivado/checkpoints/route.dcp}"
OUT="${2:-/home/qwertyoruiop/macqd700-soc-worktrees/p132-hold-analysis/build/p132_hold}"
LOCK=/var/tmp/m68k-ooo-vivado.lock
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="$OUT/p132_hold_report.out"

mkdir -p "$OUT"

if [ ! -f "$DCP" ]; then
  echo "P132: no such checkpoint: $DCP" | tee -a "$LOG"
  exit 2
fi

echo "P132: waiting for $LOCK (the shared Vivado mutex) ..." | tee -a "$LOG"
# -w 43200 = wait up to 12h for the sibling synth gate to finish, then run.
exec flock -w 43200 "$LOCK" bash -c "
  echo \"P132: mutex acquired \$(date -Is)\" >> '$LOG'
  cd '$HERE' && \
  vivado -mode batch -nojournal -notrace \
         -log '$OUT/vivado_p132.log' \
         -source tools/p132_hold_report.tcl \
         -tclargs '$DCP' '$OUT' >> '$LOG' 2>&1
  echo \"P132: finished rc=\$? \$(date -Is)\" >> '$LOG'
"
