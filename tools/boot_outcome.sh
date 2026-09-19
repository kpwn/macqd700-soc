#!/bin/bash
# boot_outcome.sh — classify N boots by where the machine ENDS UP, to compare a
# candidate fix against the recorded baseline.
#
# BASELINE (build6, before the scsi.v watchdog fix), all measured 2026-08-18:
#   * ~1 boot in 3 reached the Shutdown dialog (~3 of 12)
#   * boots wedged in the ROM Memory Manager compact/reserve loop,
#     pc_live cycling 0x4080e370..0x4080e3b0 with _ResrvMem ($A040) at
#     0x4080ed3c taken >1.3M times
#   * some ended parked in the ROM serial test monitor at 0x4084a8xx
#
# A fix that works should stop the machine ending in the MM wedge.
# Sampling pc_live several times distinguishes "parked in a tight loop" from
# "running normally somewhere".

cd "$(dirname "$0")/.." || exit 1
N=${N:-3}
SETTLE=${SETTLE:-300}
jt() { JT_WAIT=${2:-15} tools/jt.sh "$1" 2>&1; }

for i in $(seq 1 "$N"); do
    echo "=== boot $i/$N ==="
    for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
        jt "$c" >/dev/null; done
    jt "vio-hard-reset" 30 >/dev/null
    s=$SECONDS
    while [ $((SECONDS-s)) -lt "$SETTLE" ]; do jt "halt-status" 8 >/dev/null; done

    PCS=$(for k in $(seq 1 8); do jt "halt-status" 10 | grep -oE 'pc_live=0x[0-9a-f]+' | head -1; done | sed 's/pc_live=//')
    echo "  pc_live samples:"; echo "$PCS" | sort | uniq -c | sort -rn | sed 's/^/    /'

    MM=$(echo "$PCS" | grep -c '^0x4080e3')
    MON=$(echo "$PCS" | grep -c '^0x4084a')
    ROMMM=$(echo "$PCS" | grep -c '^0x4080e')
    RAW=$(jt "vio-read scsi" 40 | grep -oE 'vio_scsi_sd = [0-9a-fA-F]+' | awk '{print $3}')
    [ -n "$RAW" ] && python3 -c "
v=int('$RAW',16); g=lambda h,l:(v>>l)&((1<<(h-l+1))-1)
print(f'  SD: completions={g(43,28)} errors={g(27,20)} cause={g(19,16)} sticky={g(3,3)}')"
    if [ "$MM" -gt 0 ]; then
        echo "  VERDICT: still wedged in the ROM MM compact/reserve loop (baseline failure)"
    elif [ "$MON" -gt 0 ]; then
        echo "  VERDICT: parked in the ROM serial monitor 0x4084a8xx (early failure)"
    elif [ "$ROMMM" -gt 0 ]; then
        echo "  VERDICT: in ROM MM region but not the known wedge loop"
    else
        echo "  VERDICT: NOT in the known failure loops -- running elsewhere"
    fi
    echo
done
