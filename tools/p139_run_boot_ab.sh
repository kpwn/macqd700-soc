#!/usr/bin/env bash
# p139: interleaved p133-vs-p139 boot A/B. p139 = cpu040 bb3bca1 + arm B ONLY
# (RTE resyncs RobPlugin.committedCcr); arms A (fetch gate) and C (walker->L1D)
# are both provably absent from the netlist.
#
#   p139 BOOTS  -> arm C (2db5bd3, MMU walks through L1D) is the culprit
#   p139 WEDGES -> arm B (7057e80) OR the page-granule fix -- see the CORRECTION below
#
# ══ CORRECTION TO THE RECORD: p133 IS NOT cpu040 bb3bca1 ═══════════════════════
# Every predecessor document states the booting p133 reference is cpu040 bb3bca1.
# It is not. The preserved artifact is build_id 0x14f3c599, and
#   git ls-tree 14f3c599 cpu040  ->  0eafde59c10d024032faa5777166c881dc2a0718
# which is bb3bca1's PARENT. SoC commit fff084d8 ("build(p133): include the
# CPUSHP/CINVP page-granule fix (cpu040 bb3bca1)") landed at 09:31 UTC, while the
# p133 bitstream's own buildinfo says generated_utc=2026-09-04T10:05:49Z off
# commit 14f3c599 (09:23 UTC) -- the pin bump missed that build by 8 minutes.
#
# So the untested delta between the booting reference and the wedging p138c is
# FOUR arms, not three:
#   arm 0  0eafde59 -> bb3bca1   CPUSHP/CINVP page scope honours TC.P (8 KB pages)
#                                45 lines in DcachePlugin.scala -- NEVER BOOT-TESTED
#   arm A  8887507              speculative-fetch gate            -- EXONERATED (p138c)
#   arm B  7057e80              RTE resyncs committedCcr
#   arm C  2db5bd3              MMU table walks through L1D
#
# This bitstream is arm 0 + arm B. It is therefore DECISIVE in one direction only:
# if it BOOTS, arm C is the culprit and arms 0 and B are both cleared. If it
# WEDGES, the culprit is arm 0 or arm B and one more bitstream (cpu040 = bb3bca1
# alone) is needed to split them.
#
# Same shape as tools/p138c_run_boot_ab.sh -- the p133 artifact is a PRESERVED
# copy and must never be overwritten (12673817 bytes, checked below).
#
# usage: [PAIRS=6] [SETTLE=200] tools/p139_run_boot_ab.sh
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
# The preserved reference must be BYTE-intact, not merely present.
SZ=$(stat -c %s "$P133/fpga_top.bit")
if [ "$SZ" != "12673817" ]; then
  echo "FAIL: preserved p133 bitstream is $SZ bytes, expected 12673817 -- REFUSING to run"
  exit 1
fi

echo "A = p133 $A_ID  $P133/fpga_top.bit (md5 $(md5sum "$P133/fpga_top.bit" | cut -d' ' -f1))"
echo "B = p139 $B_ID  $NEW/fpga_top.bit  (md5 $(md5sum "$NEW/fpga_top.bit" | cut -d' ' -f1))"
echo "B cpu040 = $(git -C cpu040 rev-parse HEAD)"

PAIRS=${PAIRS:-6} SETTLE=${SETTLE:-200} \
OUTDIR=${OUTDIR:-/tmp/p139_abboot} \
A_BIT="$P133/fpga_top.bit" A_LTX="$P133/fpga_top.ltx" A_NAME=p133 A_ID="$A_ID" \
B_BIT="$NEW/fpga_top.bit"  B_LTX="$NEW/fpga_top.ltx"  B_NAME=p139 B_ID="$B_ID" \
tools/p138c_ab_boot.sh
echo "P139_BOOT_AB_DONE"
