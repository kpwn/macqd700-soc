#!/usr/bin/env bash
# p138c: extended-settle re-read of the p138c arm, to rule out a PREMATURE SAMPLE.
#
# WHY. `9086c68c` (partial icon on black) and `f8e8fd03` (fully black) are premature-sample
# signatures, NOT failure classes -- reading them as terminal already cost this campaign a
# published wrong conclusion. So a p138c sample that lands on 9086c68c at SETTLE=200 must be
# re-read with a much longer settle before it is binned.
#
# The screen is the weakest instrument here anyway. This script leans on the two that
# distinguish "stopped" from "still booting" directly, sampled across a LONG window:
#   * retired_macros (OFF_INST_LO/HI at 0x50901008 / 0x5090100C) -- live, free-running,
#     needs no halt. A boot in progress advances it by millions.
#   * exc-ring head -- on a healthy boot the 60 Hz VIA1 IRQ churns vec=0x19/0x1a constantly.
# `inst-count` is deliberately NOT used: it has read 0 on a demonstrably executing CPU.
#
# ARMS NOTHING. Plain `reset` (vio-hard-reset does not restart the CPU;
# reset-and-break-pc hard-hangs the REPL).
#
# usage: BIT=... LTX=... WANT_ID=0x... [SETTLE=600] tools/p138c_confirm_wedge.sh
set -u
cd "$(dirname "$0")/.." || exit 1
BIT=${BIT:?}; LTX=${LTX:?}; WANT_ID=${WANT_ID:?}
SETTLE=${SETTLE:-600}
NAME=${NAME:-p138c-confirm}
SNAP=${SNAP:-http://10.200.0.12:8080/snapshot.jpg}
OUT=${OUT:-/tmp/p138c_confirm}; mkdir -p "$OUT"

jt () { JT_WAIT=${2:-15} JT_TAG="$NAME" tools/jt.sh "$1" 2>&1; }
rd () { jt "r $1" 15 | grep -oiE '0x[0-9a-f]{8}' | tail -1; }

echo "=== $NAME: extended-settle confirmation ($(date -Is)) ==="
jt "load-bit $BIT $LTX" 240 | grep -E "programmed|ERROR" | sed 's/^/  /'
sleep 5
jt "build-id" 30 >/dev/null; sleep 2
live=$(jt "build-id" 30 | grep -oiE 'build_id = 0x[0-9a-f]+' | head -1 | grep -oiE '0x[0-9a-f]+')
echo "  live build_id: ${live:-UNKNOWN} (want $WANT_ID)"
[ "${live,,}" = "${WANT_ID,,}" ] || { echo "  ABORT: wrong bitstream live"; exit 1; }

for c in "break-pc off" "halt-clear" "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off"; do
  jt "$c" 10 >/dev/null 2>&1
done
for lane in 0 1 2 3 4 5 6 7; do jt "halt-exc-mask raw $lane 0" 8 >/dev/null 2>&1; done
st=$(jt "halt-status" 10)
printf '%s' "$st" | grep -q 'enables={ha=0 bp=0 exc=0 pcmis=0}' \
  && echo "  disarm verified: enables all zero" \
  || { echo "  WARNING still armed:"; printf '%s' "$st" | grep -oE 'enables=\{[^}]*\}' | sed 's/^/    /'; }

jt "reset" 40 | grep -E "reset done" | sed 's/^/  /'

# Sample the two live instruments at intervals across the WHOLE window, so a slow boot is
# visible as motion rather than being collapsed into one before/after pair.
prev_mac=""; moved=0
START=$(date +%s)
for t in 120 200 300 450 "$SETTLE"; do
  now=$(date +%s); want=$((START + t)); [ "$now" -lt "$want" ] && sleep $((want - now))
  h=$(jt "exc-ring" 20 | grep -oE 'head=[0-9]+' | head -1 | cut -d= -f2)
  mh=$(rd 0x5090100C); ml=$(rd 0x50901008)
  pc=$(jt "halt-status" 10 | grep -oE 'pc_live=0x[0-9a-f]+' | head -1 | sed 's/pc_live=//')
  echo "  t+${t}s  pc_live=$pc  retired_macros=${mh}:${ml}  exc_head=$h"
  [ -n "$prev_mac" ] && [ "$prev_mac" != "${mh}:${ml}" ] && moved=1
  prev_mac="${mh}:${ml}"
done

curl -s -m 20 -o "$OUT/${NAME}.jpg" "$SNAP" 2>/dev/null
smd5=$(md5sum "$OUT/${NAME}.jpg" 2>/dev/null | cut -d' ' -f1)
case "${smd5:0:8}" in
  2f5de0f4) sig="HAPPY-MAC" ;; eed68f0c) sig="SAD-MAC-0F/03" ;;
  feaec0e2) sig="SAD-MAC-0F/0A" ;; 9086c68c) sig="PARTIAL-ICON (premature-sample signature)" ;;
  f8e8fd03) sig="BLACK (premature-sample signature)" ;; *) sig="unknown" ;;
esac
echo "  screen md5 after ${SETTLE}s: ${smd5:-none}  ($sig)"
echo "  RETIRE MOTION ACROSS THE WHOLE WINDOW: $([ $moved -eq 1 ] && echo ADVANCING || echo FROZEN)"
echo "P138C_CONFIRM_DONE"
