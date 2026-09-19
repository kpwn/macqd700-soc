#!/bin/bash
# catch_reserr_set.sh — catch the INSTANT ResErr is set to ioErr (-36), to find
# which operation actually failed and when.
#
# This tests the user's "it may be a scsi timeout after all" in the form the
# data supports.  Measured: ALL disk I/O finishes by t~49s, and the CDEF
# failure happens much later with the bus completely idle.  So if the CDEF load
# failed, it failed during the EARLY I/O era, leaving an empty handle that the
# later redraw cannot recover.  That makes ResErr=-36 genuine rather than
# stale -- just from an earlier moment than I was looking at.
#
# ResErr is the WORD at lowmem 0x0A60.  ioErr = -36 = 0xFFDC.
# Byte lanes: 0x0A60 & 3 == 0, so the word occupies the TOP two bytes of the
# aligned longword -> value 0xFFDC0000, lanes 0xC.
#
# Watchpoint config SURVIVES CPU RESET, so arm it and then reset to catch a
# one-shot early event -- which is the whole point here.
#
# Watchpoints are known to work on RAM (the [A5-3692] flag watch and the
# 0x000129DC clear-loop watch both fired).  Earlier "watchpoint dead" scares
# were the MACHINE not booting (parked in the ROM serial monitor at 0x4084a8xx),
# not the mechanism.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-240}
TRIES=${TRIES:-3}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
wait_halt() { local s=$SECONDS
    while [ $((SECONDS-s)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0; done; return 1; }

for t in $(seq 1 "$TRIES"); do
    echo "=== attempt $t/$TRIES ==="
    for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
        jt "$c" >/dev/null; done
    jt "watch 0 0x00000A60 w value 0xFFDC0000 lanes 0xC" 25 | grep -E "armed|ERROR" | sed 's/^/  /'
    jt "vio-hard-reset" 30 >/dev/null
    echo "  armed before reset; waiting ${WAIT}s for ResErr <- ioErr(-36)"

    if ! wait_halt "$WAIT"; then
        echo "  ResErr was never set to -36 within ${WAIT}s of boot."
        echo "  (If the CDEF failure still happens later, then -36 is set AFTER"
        echo "   this window, or was never set this boot at all.)"
        continue
    fi
    PC=$(jt "pc" 15 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
    echo "  *** ResErr set to ioErr — caught it ***"
    jt "watch status" 20 | grep -oE 'wp HIT: .*' | sed 's/^/    /'
    echo "    halted PC = $PC"
    jt "live-arch" 40 | grep -E "^> (D0|D1|D2|A0|A1|A2|A5|A6|A7|SR|PC) " | sed 's/^/    /'
    # ROM 0x408146E6 stashes the GENUINE error at lowmem 0x3DE before
    # substituting the generic ioErr. If that shim was on this path, the real
    # fault is readable here.
    E=$(jt "dump-mem 0x000003DC 1" 25 | grep -oE '= 0x[0-9a-fA-F]{8}' | head -1 | sed 's/= 0x//')
    python3 -c "
v=int('${E:-0}',16); w=v & 0xFFFF
s=w-0x10000 if w>=0x8000 else w
names={-35:'nsvErr no such volume',-36:'ioErr',-39:'eofErr',-40:'posErr',
 -43:'fnfErr',-49:'opWrErr',-50:'paramErr',-51:'rfNumErr',-53:'volOffLinErr',
 -54:'permErr',-55:'volOnLinErr',-56:'nsDrvErr',-57:'noMacDskErr',-58:'extFSErr',
 -60:'badMDBErr',-64:'lastDskErr',-65:'noDriveErr',-127:'fsDSIntErr'}
print(f'    real-error stash lowmem 0x3DE = {s}  {names.get(s,\"\")}' +
      ('   (0 = this shim was not on the path)' if s==0 else '   <== THE TRUE FAULT'))"
    echo "    --- SD/SCSI state at that moment ---"
    RAW=$(jt "vio-read scsi" 40 | grep -oE 'vio_scsi_sd = [0-9a-fA-F]+' | awk '{print $3}')
    [ -n "$RAW" ] && python3 -c "
v=int('$RAW',16); g=lambda h,l:(v>>l)&((1<<(h-l+1))-1)
print(f'    completions={g(43,28)} errors={g(27,20)} cause={g(19,16)} sticky={g(3,3)} busy={g(2,2)} irq={g(1,1)} drq={g(0,0)}')"
    echo "    --- pc-trace ---"
    jt "pc-trace 32" 40 | grep '^> trace' | tail -18 | sed 's/^/    /'
    echo "    --- code at the setter ---"
    tools/dis_at.sh $(printf "0x%08X" $(( $(printf '%d' "$PC") - 0x30 ))) 20 2>&1 | sed 's/^/    /'
    exit 0
done
echo "=== ResErr never observed being set to -36 ==="
exit 1
