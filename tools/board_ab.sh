#!/bin/bash
# board_ab.sh -- automated board A/B: load-bit -> Finder -> Speedometer Benchmark Mix
# -> OCR of the result window -> REPL perf -> JSON.  Thin wrapper around board_ab.py.
#
# THE OWNER DECIDES WHEN THE BOARD IS USED.  This reprograms the FPGA and types on
# the Mac; run it only inside an owner-granted window.  Never in a background loop.
#
#   tools/board_ab.sh <bit> <ltx> [label] [--runs 2] [--idle 2] [--outdir DIR] ...
#   tools/board_ab.sh --dry-run <bit> <ltx> [label]     # replay fixtures, touch nothing
#   tools/board_ab.sh compare <ref.json> <new.json>     # per-row deltas + one-line summary
#   tools/board_ab.sh ocr <snapshot.jpg>...             # OCR saved screenshots
#   tools/board_ab.sh selftest                          # OCR/classifier vs labelled fixtures
#
# Launch step (Speedometer must be reachable from the Finder): the default is the
# owner's verified recipe -- type-to-select "S" in the open Speedometer folder,
# Cmd-O, Return x3.  Override with --launch "<steps>" or BOARD_AB_LAUNCH, e.g.
#   --launch "mouse:120,80; dblclick; wait:15; return; wait:3; return; wait:3; return"
# Steps: key:<c|0xNN> type:<text> cmd:<c> return tab[:n] wait:<s> mouse:<x>,<y>
#        (Mac coords; pinned at origin then stepped by 4) click dblclick snap:<name>
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
case "${1:-}" in
    compare|ocr|classify|learn|selftest) exec python3 "$HERE/board_ab.py" "$@" ;;
    ""|-h|--help) exec python3 "$HERE/board_ab.py" run --help ;;
    *) exec python3 "$HERE/board_ab.py" run "$@" ;;
esac
