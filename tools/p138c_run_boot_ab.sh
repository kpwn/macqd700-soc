#!/usr/bin/env bash
# p138c: launch the interleaved p133-vs-p138c boot comparison with the exact artifacts.
#
# Separate from p138c_ab_boot.sh so the ARTIFACT PATHS AND EXPECTED build_ids are recorded
# in git rather than living in a shell history. The p133 artifact is a PRESERVED copy and
# must never be overwritten -- it is the only surviving reference distribution
# (7/12 Happy Mac, 3/12 Sad Mac -> serial monitor, 2/12 bus-error livelock, 0/12 Finder).
#
# usage: [PAIRS=6] [SETTLE=200] tools/p138c_run_boot_ab.sh
set -u
cd "$(dirname "$0")/.." || exit 1

P133=${P133_DIR:-/home/qwertyoruiop/tmp/claude-1000/-home-qwertyoruiop-m68k-core-040-ooo/02fab060-05c0-4ce0-953f-15f1be816a95/scratchpad/p133-artifact}
NEW=${NEW_DIR:-build/vivado}

B_ID=$(sed -n 's/^build_id=//p' "$NEW/fpga_top.buildinfo" | head -1)
A_ID=$(sed -n 's/^build_id=//p' "$P133/fpga_top.buildinfo" | head -1)
[ -n "$B_ID" ] || { echo "FAIL: no build_id in $NEW/fpga_top.buildinfo"; exit 1; }
[ -n "$A_ID" ] || { echo "FAIL: no build_id in $P133/fpga_top.buildinfo"; exit 1; }
for f in "$P133/fpga_top.bit" "$P133/fpga_top.ltx" "$NEW/fpga_top.bit" "$NEW/fpga_top.ltx"; do
  [ -f "$f" ] || { echo "FAIL: missing $f"; exit 1; }
done

echo "A = p133  $A_ID  $P133/fpga_top.bit  (md5 $(md5sum "$P133/fpga_top.bit" | cut -d' ' -f1))"
echo "B = p138c $B_ID  $NEW/fpga_top.bit   (md5 $(md5sum "$NEW/fpga_top.bit" | cut -d' ' -f1))"

PAIRS=${PAIRS:-6} SETTLE=${SETTLE:-200} \
OUTDIR=${OUTDIR:-/tmp/p138c_abboot} \
A_BIT="$P133/fpga_top.bit" A_LTX="$P133/fpga_top.ltx" A_NAME=p133  A_ID="$A_ID" \
B_BIT="$NEW/fpga_top.bit"  B_LTX="$NEW/fpga_top.ltx"  B_NAME=p138c B_ID="$B_ID" \
tools/p138c_ab_boot.sh
echo "P138C_BOOT_AB_DONE"
