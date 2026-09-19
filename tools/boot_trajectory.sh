#!/bin/bash
# boot_trajectory.sh — watch ONE boot over a long window instead of sampling it once.
#
# WHY: a single late sample cannot tell "hung here" from "still working, slower
# than you expected".  Both of tonight's near-misses were this same error:
#   * at 150 s settle, p133's mid-boot SCSI-probe bus errors read as 5/5 Sad Macs;
#   * at 200 s settle, p137 parked at 0x40806b68 looked like a regression.
# Two of p137's three fixes plausibly SLOW the machine (the speculation gate
# removes I-fetch throughput; walks through L1D add walk latency), so a longer
# boot is a live hypothesis and has to be excluded before claiming a regression.
#
# Records pc_live AND the exception-ring head on a fixed cadence.  A head that
# keeps advancing means the machine is still taking exceptions -- progressing,
# not wedged -- regardless of what the PC looks like at any one instant.
#
# Usage: DUR=600 STEP=30 TAG=p137 tools/boot_trajectory.sh
set -u
cd "$(dirname "$0")/.." || exit 1

DUR=${DUR:-600}
STEP=${STEP:-30}
TAG=${TAG:-traj}
OUTDIR=${OUTDIR:-/tmp/traj-$TAG}
SNAP=${SNAP:-http://10.200.0.12:8080/snapshot.jpg}
mkdir -p "$OUTDIR"

jt () { JT_WAIT=${2:-12} tools/jt.sh "$1" 2>&1; }

echo "=== boot_trajectory TAG=$TAG dur=${DUR}s step=${STEP}s ==="
jt "build-id" 20 | grep -i "build_id ="

for c in "break-pc off" "halt-clear" "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off"; do
    jt "$c" 10 >/dev/null 2>&1
done
for lane in 0 1 2 3 4 5 6 7; do jt "halt-exc-mask raw $lane 0" 8 >/dev/null 2>&1; done
jt "halt-status" 10 | grep -oE 'enables=\{[^}]*\}' | sed 's/^/  disarmed: /'

echo "  resetting at $(date -Is)"
jt "reset" 40 | grep -E "reset done" | sed 's/^/  /'
T0=$SECONDS

printf '%6s  %-12s  %6s  %s\n' "t(s)" "pc_live" "head" "screen-md5"
while [ $((SECONDS - T0)) -lt "$DUR" ]; do
    t=$((SECONDS - T0))
    pc=$(jt "halt-status" 10 | grep -oE 'pc_live=0x[0-9a-f]+' | head -1 | sed 's/pc_live=//')
    head=$(jt "exc-ring" 15 | grep -oE 'head=[0-9]+' | head -1 | cut -d= -f2)
    curl -s -m 15 -o "$OUTDIR/t${t}.jpg" "$SNAP" 2>/dev/null
    md5=$(md5sum "$OUTDIR/t${t}.jpg" 2>/dev/null | cut -c1-8)
    printf '%6s  %-12s  %6s  %s\n' "$t" "${pc:-?}" "${head:-?}" "${md5:-?}"
    sleep "$STEP"
done

echo
echo "=== final exception ring ==="
jt "exc-ring" 25 | head -12
echo "=== boot_trajectory TAG=$TAG COMPLETE ==="
