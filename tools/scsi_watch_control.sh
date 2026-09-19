#!/bin/bash
# scsi_watch_control.sh — POSITIVE CONTROL for the 53C96 command-register
# watchpoint.
#
# A null result from catch_scsi_reset_recovery.sh ("no post-init reset") is only
# meaningful if a watchpoint on that register can fire AT ALL.  This arms the
# same watch on a command the driver issues constantly, so a non-firing control
# means the measurement apparatus is broken, not that the event is absent.
#
# MAME's own command-register histogram over one 7.5.3 boot:
#     0x01 Flush FIFO      x8685
#     0x44 Select w/ATN3   x3149
#     0x11 Init Cmd Compl  x3730
# Any of those must fire within seconds of arming on a live, booting machine.
#
# Same addressing as the real test: reg 3 = 0x5000F030, byte lane 0x8,
# value 0x0N000000.

cd "$(dirname "$0")/.." || exit 1
VAL=${VAL:-0x01}          # Flush FIFO
SETTLE=${SETTLE:-60}
WAIT=${WAIT:-120}
CMDREG=0x5000F030
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }

V32=$(printf "0x%08X" $(( $(printf '%d' "$VAL") << 24 )))
echo "=== control: watch for command-register write of $VAL ($V32 lanes 0x8) ==="
for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
    jt "$c" >/dev/null; done
jt "vio-hard-reset" 30 >/dev/null
s=$SECONDS; while [ $((SECONDS-s)) -lt "$SETTLE" ]; do jt "halt-status" 8 >/dev/null; done

jt "watch 0 $CMDREG w value $V32 lanes 0x8" 25 | grep -E "armed|ERROR" | sed 's/^/  /'
s=$SECONDS; H=0
while [ $((SECONDS-s)) -lt "$WAIT" ]; do
    jt "halt-status" 8 | grep -q 'effective=1' && { H=1; break; }
done

if [ "$H" = 1 ]; then
    echo "  CONTROL FIRED after $((SECONDS-s))s -- the watchpoint mechanism works."
    jt "watch status" 20 | grep -E "wp HIT" | sed 's/^/    /'
    echo "  => a null result for 0x02/0x03 is therefore a REAL absence of resets."
    exit 0
fi
echo "  CONTROL DID NOT FIRE in ${WAIT}s."
echo "  => the apparatus is suspect; do NOT read the reset test's null result"
echo "     as evidence.  Check that the CPU is running and that the driver is"
echo "     still issuing SCSI traffic at this point in the boot."
jt "watch status" 20 | grep -E "wp HIT|wp0" | sed 's/^/    /'
exit 2
