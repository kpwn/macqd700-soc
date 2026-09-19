#!/bin/bash
# catch_reqstatus_write.sh — who writes the status the .ASYC00 driver rejects?
#
# The driver tests the WORD at request+10 and retries while it is non-zero,
# giving up after 16 retries with ioErr. On the captured boots the request block
# is at 0x0007EFF0 (stable across boots), so the field is 0x0007EFFA and the
# observed value is 0xE104.
#
# Byte lanes: 0x0007EFFA & 3 == 2, so the word occupies the LOW two bytes of the
# aligned longword -> value 0x0000E104, lanes 0x3.
# Watchpoints on RAM are known-good on this machine (the [A5-3692] flag watch and
# the 0x00010C6C gamma watch both fired); it was only I/O-space watches that
# never fired.
#
# Slot 1 = the driver's ioErr site as a REACH MARKER, so "no hit" can be told
# apart from "this boot never failed". Identify hits via halt-status `hit=`,
# never the live PC.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-260}
MAXH=${MAXH:-8}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
wait_halt() { local s=$SECONDS
    while [ $((SECONDS-s)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0; done; return 1; }

for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
    jt "$c" >/dev/null; done
jt "watch 0 0x0007EFFA w value 0x0000E104 lanes 0x3" 25 | grep -E "armed|ERROR" | sed 's/^/  /'
jt "break-pc 0x0000CFF6" 25 | grep -E "slot=" | sed 's/^/  marker: /'
jt "vio-hard-reset" 30 >/dev/null
START=$SECONDS

for i in $(seq 1 "$MAXH"); do
    wait_halt "$WAIT" || { echo "  no further halt in ${WAIT}s"; break; }
    T=$((SECONDS-START))
    HS=$(jt "halt-status" 20)
    HIT=$(echo "$HS" | grep -oE 'hit=0x[0-9a-f]+' | head -1 | sed 's/hit=//')
    WP=$(jt "watch status" 20 | grep -oE 'wp HIT: .*' | head -1)
    if echo "$WP" | grep -q "none latched"; then
        echo "  [$i] t=${T}s  MARKER: driver ioErr (hit=$HIT) -- status write not caught yet"
    else
        echo "  [$i] t=${T}s  *** STATUS WRITE CAUGHT ***"
        echo "        $WP"
        WPC=$(echo "$WP" | grep -oE 'pc=0x[0-9a-fA-F]{8}' | sed 's/pc=//')
        echo "        writer PC = $WPC"
        jt "live-arch" 40 | grep -E "^> (D0|D1|D2|A0|A1|A2|A5|A6|A7) " | sed 's/^/        /'
        echo "        --- pc-trace (mask bit 31 of each entry) ---"
        jt "pc-trace 24" 40 | grep '^> trace' | tail -12 | sed 's/^/        /'
        [ -n "$WPC" ] && { echo "        --- code at the writer ---"; \
            tools/dis_at.sh $(printf "0x%08X" $(( $(printf '%d' "$WPC") - 0x20 ))) 16 2>&1 | sed 's/^/        /'; }
        exit 0
    fi
    jt "halt-clear" 20 >/dev/null; jt "cont" 30 >/dev/null
done
