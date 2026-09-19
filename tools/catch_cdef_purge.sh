#!/bin/bash
# catch_cdef_purge.sh — catch the WRITE that empties the OK-button CDEF's master
# pointer, i.e. the moment the resource is purged.
#
# WHY THIS IS NOW THE RIGHT TARGET.  Measured 2026-08-18:
#   * MAME never purges this CDEF at all (0 writes to its master pointer in
#     125 emulated seconds; it stays resident at 601C4BF0).
#   * On HW the master pointer is 0 and _LoadResource cannot restore it.
#   * ALL disk I/O finishes by t~49s on HW (SD completions 0->3566 then a 371s
#     plateau, nothing busy), and the CDEF failure happens MUCH later -- so the
#     failing reload issues no disk request, and ResErr=-36 is very likely stale
#     from the earlier I/O era rather than caused by this failure.
# So the fault is not the reload; it is that the resource got purged in the
# first place. This catches the purge itself.
#
# Handle address 0x000129DC was identical on two separate failing boots, so it
# is stable enough to arm on. If it moved this boot, the break-pc on the failure
# arm fires instead and says so -- rather than the run silently proving nothing.
#
# RAM watchpoints are known-good here (the [A5-3692] error-flag watch fired
# correctly). It was the I/O-SPACE watch that never fired, and that is explained
# by the empty-traffic window, not by broken watchpoints.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-430}
TRIES=${TRIES:-4}
MP=0x000129DC
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
wait_halt() { local s=$SECONDS
    while [ $((SECONDS-s)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0; done; return 1; }

for t in $(seq 1 "$TRIES"); do
    echo "=== attempt $t/$TRIES ==="
    for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
        jt "$c" >/dev/null; done
    # slot0: the purge (full-longword store of 0 to the master pointer)
    jt "watch 0 $MP w value 0x00000000 lanes 0xF" 25 | grep -E "armed|ERROR" | sed 's/^/  watch: /'
    # and the failure arm, so a boot that fails with the handle somewhere else
    # still tells us something instead of timing out silently
    jt "break-pc 0x40815E6A" 25 | grep -E "slot=" | sed 's/^/  bp:    /'
    jt "vio-hard-reset" 30 >/dev/null

    wait_halt "$WAIT" || { echo "  no halt in ${WAIT}s; retrying"; continue; }

    PC=$(jt "pc" 15 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
    HIT=$(jt "watch status" 20 | grep -oE 'wp HIT: .*' | head -1)
    echo "  halted PC=$PC"
    echo "  $HIT"
    case "$HIT" in
      *"none latched"*)
        echo "  -> the CDEF FAILURE arm fired, not the purge watch."
        echo "     So the master pointer was NOT written at $MP this boot:"
        echo "     either the handle moved, or it was never loaded to begin with"
        echo "     (an empty handle from GetResource, never purged at all)."
        CR=$(jt "dump-mem 0x0007efac 1" 25 | grep -oE '= 0x[0-9a-fA-F]{8}' | head -1 | sed 's/= //')
        echo "     ControlRecord = ${CR:-?}"
        if [ -n "$CR" ]; then
          DP=$(jt "dump-mem $(printf '0x%08X' $(( $(printf '%d' $CR) + 24 ))) 1" 25 | grep -oE '= 0x[0-9a-fA-F]{8}' | head -1 | sed 's/= //')
          echo "     contrlDefProc = ${DP:-?}  (armed on $MP)"
        fi
        ;;
      *)
        echo "  *** PURGE CAUGHT — this is the code that emptied the CDEF ***"
        jt "live-arch" 40 | grep -E "^> (D0|D1|D2|A0|A1|A2|A5|A6|A7|SR|PC) " | sed 's/^/    /'
        echo "  --- code around the purging PC ---"
        tools/dis_at.sh $(printf "0x%08X" $(( $(printf '%d' "$PC") - 0x30 ))) 20 2>&1 | sed 's/^/    /'
        echo "  --- pc-trace ---"
        jt "pc-trace 32" 40 | grep '^> trace' | tail -20 | sed 's/^/    /'
        ;;
    esac
    exit 0
done
echo "=== no halt in $TRIES attempts ==="; exit 1
