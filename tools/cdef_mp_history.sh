#!/bin/bash
# cdef_mp_history.sh — record EVERY write to the OK-button CDEF's master pointer
# after boot init, to settle: was the resource LOADED-then-PURGED, or NEVER
# LOADED at all?
#
# The distinction matters and I had been conflating them:
#   loaded-then-purged : a non-zero write (load) later followed by a zero write
#                        (purge) -> a Memory Manager pressure event is the bug
#   never loaded       : no non-zero write ever -> GetResource handed back an
#                        empty handle and nothing ever filled it; the bug is in
#                        the resource lookup, not in purging
#
# NO VALUE FILTER this time.  A previous run armed `value 0x00000000` and caught
# ROM 0x40884496 -- an early-boot CLEAR LOOP (A1=0x129d0 walking, D2=0x20, tight
# loop at 0x40884490, early-boot A5=0x00400bfc), not a purge at all.  The
# address only becomes the CDEF master pointer later, so arming must also be
# DELAYED past init.
#
# MAME reference: its equivalent master pointer is written ZERO times in 125 s
# and stays resident at 601C4BF0.

cd "$(dirname "$0")/.." || exit 1
SETTLE=${SETTLE:-70}
WAIT=${WAIT:-380}
MAXH=${MAXH:-16}
MP=0x000129DC
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
wait_halt() { local s=$SECONDS
    while [ $((SECONDS-s)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0; done; return 1; }

TRIES=${TRIES:-4}
for attempt in $(seq 1 "$TRIES"); do
echo "=== attempt $attempt/$TRIES: reset, settle ${SETTLE}s past init, log writes to $MP ==="
for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
    jt "$c" >/dev/null; done
jt "vio-hard-reset" 30 >/dev/null
s=$SECONDS; while [ $((SECONDS-s)) -lt "$SETTLE" ]; do jt "halt-status" 8 >/dev/null; done

jt "watch 0 $MP w" 25 | grep -E "armed|ERROR" | sed 's/^/  /'
jt "break-pc 0x40815E6A" 25 | grep -E "slot=" | sed 's/^/  /'
echo "  logging..."

LOADED=0; PURGED=0; REACHED=0
for i in $(seq 1 "$MAXH"); do
    wait_halt "$WAIT" || { echo "  no further halt in ${WAIT}s"; break; }
    PC=$(jt "pc" 15 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
    HIT=$(jt "watch status" 20 | grep -oE 'wp HIT: .*' | head -1)
    if echo "$HIT" | grep -q "none latched"; then
        echo "  [$i] CDEF FAILURE arm reached (PC=$PC) — no more writes before it"
        REACHED=1
        break
    fi
    DATA=$(echo "$HIT" | grep -oE 'data=0x[0-9a-fA-F]{8}' | sed 's/data=//')
    WPC=$(echo "$HIT" | grep -oE 'pc=0x[0-9a-fA-F]{8}' | sed 's/pc=//')
    case "$DATA" in
      0x00000000) KIND="ZERO  (purge / clear)"; PURGED=$((PURGED+1)) ;;
      *)          KIND="NONZERO (a LOAD)";      LOADED=$((LOADED+1)) ;;
    esac
    printf "  [%2d] write %s from PC=%s   %s\n" "$i" "$DATA" "$WPC" "$KIND"
    jt "halt-clear" 20 >/dev/null; jt "cont" 30 >/dev/null
done

echo
echo "=== attempt $attempt result ==="
echo "  reached the CDEF failure : $REACHED"
echo "  non-zero writes (loads)  : $LOADED"
echo "  zero writes (purge/clear): $PURGED"

# The verdict is ONLY meaningful if this boot actually REACHED the failure.
# Otherwise "no writes" just means the dialog was never built and the CDEF was
# never needed -- a null from a run that never entered the relevant state.
# A previous version of this script printed "NEVER LOADED" off exactly such a
# run; do not repeat that.
if [ "$REACHED" != "1" ]; then
    echo "  => INCONCLUSIVE: this boot never reached the failure, so an absence"
    echo "     of writes proves nothing. Retrying."
    continue
fi

if [ "$LOADED" -gt 0 ] && [ "$PURGED" -gt 0 ]; then
    echo "  => LOADED THEN PURGED. A memory-pressure purge is the event to chase,"
    echo "     and MAME never purges this resource at all."
elif [ "$LOADED" -eq 0 ]; then
    echo "  => NEVER LOADED. We reached the failure and the master pointer was"
    echo "     never written non-zero, so nothing purged it: GetResource handed"
    echo "     back an unloaded handle. That moves the bug to the resource"
    echo "     LOOKUP path, not the Memory Manager."
fi
exit 0
done
echo "=== no attempt reached the CDEF failure ==="
exit 1
