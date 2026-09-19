#!/bin/bash
# retire_trajectory.sh — how far into a boot, in RETIRED MACROS, does a build get?
#
# THE QUESTION THIS ANSWERS
#   p137 wedges with its retired-macro counter frozen at 0x054202c3 (~88.3 M).  A
#   bare count means nothing on its own: it separates "got as far as a healthy boot
#   and then stopped" from "diverged early and only surfaced later" ONLY if you know
#   where 88.3 M falls in a healthy boot.  So sample a HEALTHY build's counter
#   against wall clock from reset and report when it crosses the wedge value.
#
# THE INSTRUMENT
#   OFF_INST_LO/HI at 0x50901008/0x5090100C — live, free-running, monotonic,
#   readable with no halt.  NOT `inst-count`, which reads a halt-captured register
#   and returns 0 on a demonstrably executing CPU.
#
#   THE LO HALF WRAPS.  At 100 MHz it wraps every ~86 s at 1 macro/cycle and
#   proportionally slower at real IPC, i.e. WELL INSIDE a 200 s boot.  LO alone is
#   therefore useless here.  Every sample uses the HI-LO-HI bracket: read HI, read
#   LO, read HI again, and accept only if HI is unchanged, so a carry landing
#   between the two reads cannot silently produce a value off by 2^32.
#
# ARMS NOTHING.  Plain `reset`.  All eight halt-exception lanes are cleared with
# `halt-exc-mask raw <lane> 0` (NOT `halt-exc-mask 0`, which is the SET form and
# arms vector 0) and the enables line is asserted.
#
# Usage: BIT=… LTX=… WANT_ID=0x… NAME=p133 DUR=210 STEP=6 tools/retire_trajectory.sh
set -u
cd "$(dirname "$0")/.." || exit 1

BIT=${BIT:?}; LTX=${LTX:?}; WANT_ID=${WANT_ID:?}; NAME=${NAME:-arm}
DUR=${DUR:-210}; STEP=${STEP:-6}
TARGET=${TARGET:-0x054202c3}      # p137's frozen value
TARGET_HI=${TARGET_HI:-0x00000000}

jt () { JT_WAIT=${2:-12} JT_TAG="retire-$NAME" tools/jt.sh "$1" 2>&1; }
rd () { jt "r $1" 12 | grep -oiE '= *0x[0-9a-f]{8}' | tail -1 | grep -oiE '[0-9a-f]{8}$'; }

# HI-LO-HI bracket.  Echoes "<hi> <lo>" or nothing if the bracket did not hold.
sample () {
    local h1 lo h2
    h1=$(rd 0x5090100C); lo=$(rd 0x50901008); h2=$(rd 0x5090100C)
    [ -n "$h1" ] && [ -n "$lo" ] && [ "$h1" = "$h2" ] || return 1
    echo "$h1 $lo"
}

echo "=== retire trajectory: $NAME  ($(date -Is)) ==="
jt "load-bit $BIT $LTX" 200 | grep -E "programmed|ERROR" | sed 's/^/  /'
sleep 5
jt "build-id" 30 >/dev/null; sleep 2
live=$(jt "build-id" 30 | grep -oiE 'build_id = 0x[0-9a-f]+' | head -1 | grep -oiE '0x[0-9a-f]+')
echo "  live build_id: ${live:-UNKNOWN} (want $WANT_ID)"
[ "${live,,}" = "${WANT_ID,,}" ] || { echo "  ABORT: wrong bitstream"; exit 1; }

for c in "break-pc off" "halt-clear" "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off"; do
    jt "$c" 10 >/dev/null 2>&1
done
for lane in 0 1 2 3 4 5 6 7; do jt "halt-exc-mask raw $lane 0" 8 >/dev/null 2>&1; done
jt "halt-status" 10 | grep -q 'enables={ha=0 bp=0 exc=0 pcmis=0}' \
  && echo "  disarm verified" || echo "  WARNING: something is still armed"

tgt=$(( (0x${TARGET_HI#0x} << 32) | 0x${TARGET#0x} ))
echo "  target (p137's frozen count) = $tgt macros"

jt "reset" 40 | grep -E "reset done" | sed 's/^/  /'
t0=$(date +%s)
crossed=""
prev=0
while :; do
    now=$(date +%s); el=$(( now - t0 ))
    [ "$el" -ge "$DUR" ] && break
    if s=$(sample); then
        hi=$(echo "$s" | cut -d' ' -f1); lo=$(echo "$s" | cut -d' ' -f2)
        v=$(( (0x$hi << 32) | 0x$lo ))
        pc=$(jt "halt-status" 10 | grep -oE 'pc_live=0x[0-9a-f]+' | head -1 | sed 's/pc_live=//')
        printf "  t=%3ds  macros=%-14d (hi=0x%s lo=0x%s)  pc_live=%s\n" "$el" "$v" "$hi" "$lo" "${pc:-?}"
        if [ -z "$crossed" ] && [ "$v" -ge "$tgt" ]; then
            crossed="$el"
            echo "  >>> CROSSED p137's frozen count ($tgt) at t=${el}s, pc_live=${pc:-?}"
        fi
        if [ "$v" -lt "$prev" ]; then echo "  WARNING: counter went BACKWARDS -- bracket failed, distrust this row"; fi
        prev=$v
    else
        printf "  t=%3ds  sample REJECTED (HI changed across the LO read)\n" "$el"
    fi
    sleep "$STEP"
done

echo
if [ -n "$crossed" ]; then
    echo "  RESULT $NAME: reached p137's frozen retired-macro count at t=${crossed}s of a ${DUR}s boot."
else
    echo "  RESULT $NAME: NEVER reached p137's frozen count ($tgt) within ${DUR}s."
fi
echo "=== retire trajectory $NAME COMPLETE ($(date -Is)) ==="
