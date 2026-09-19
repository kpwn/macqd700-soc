#!/bin/bash
# catch_syserror88.sh — enumerate every _SysError raised during boot and stop on
# code 88 (dsCDEFnFnd -- Control Definition Function not found).
#
# An UNQUALIFIED atrap on $A9C9 halts on SysError(40) = dsGreeting first, which
# is the perfectly normal "Welcome to Macintosh" splash -- the splash is drawn
# THROUGH SysError.  So iterate: at each halt read D0, log it, and continue
# until the low word is 88 (the code the Finder's $A9C9 patch actually chokes
# on).  The log of codes seen is itself useful: it says which errors this boot
# raises and in what order.
#
# 88 = dsCDEFnFnd.  A CDEF draws and handles CONTROLS (buttons).  Matches the
# symptom exactly: our Shutdown Check dialog paints an empty white box with no
# OK button and Return does nothing, while MAME draws the button and Return
# dismisses it.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-420}
MAXHALTS=${MAXHALTS:-40}
TARGET=${TARGET:-88}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }

wait_halt() { local s=$SECONDS
    while [ $((SECONDS-s)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0; done; return 1; }

echo "=== arming atrap on _SysError (\$A9C9), unqualified ==="
jt "watch 0 off" >/dev/null; jt "watch 1 off" >/dev/null
jt "break-pc off" >/dev/null; jt "atrap 1 off" >/dev/null
jt "halt-clear" >/dev/null
jt "atrap 0 0xA9C9" 25 | grep -E "armed|ERROR" | sed 's/^/  /'
jt "vio-hard-reset" 30 >/dev/null

for i in $(seq 1 "$MAXHALTS"); do
    wait_halt "$WAIT" || { echo "no further SysError within ${WAIT}s (after $((i-1)) halts)"; exit 2; }
    PC=$(jt "pc" 15 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
    ARCH=$(jt "live-arch" 40)
    D0=$(echo "$ARCH" | grep -oE '^> D0 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | head -1)
    CODE=$(( $(printf '%d' "${D0:-0}") & 0xFFFF ))
    printf "  halt %2d: PC=%s  D0=%s  code=%d\n" "$i" "$PC" "$D0" "$CODE"

    if [ "$CODE" = "$TARGET" ]; then
        echo
        echo "*** SysError($TARGET) — this is the fault ***"
        echo "--- registers ---"; echo "$ARCH" | sed 's/^/  /'
        SP=$(echo "$ARCH" | grep -oE '^> A7 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | head -1)
        echo "--- stack (top longword = caller return address) ---"
        jt "dump-mem $(printf '0x%08X' $(( $(printf '%d' "$SP") & ~3 ))) 12" 40 | grep '^> mem' | sed 's/^/  /'
        echo "--- pc-trace ---"
        jt "pc-trace 48" 40 | grep '^> trace' | tail -30 | sed 's/^/  /'
        echo "--- code at the call site ---"
        tools/dis_at.sh $(printf "0x%08X" $(( $(printf '%d' "$PC") - 0x40 ))) 28 2>&1 | sed 's/^/  /'
        exit 0
    fi
    jt "halt-clear" 20 >/dev/null
    jt "cont" 30       >/dev/null
done
echo "=== hit MAXHALTS=$MAXHALTS without seeing code $TARGET ==="
exit 1
