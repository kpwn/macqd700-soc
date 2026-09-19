#!/bin/bash
# catch_scsi_reset_recovery.sh — does the Mac's SCSI driver perform ERROR
# RECOVERY (chip / bus reset) after boot init?  That is the observable signature
# of a SCSI Manager TIMEOUT, which is the one failure shape that explains all
# the evidence so far:
#
#   ResErr = -36 (ioErr)          the resource read failed
#   sd_ctrl errors  = 0           the card never faulted
#   scsi.v CHECK CONDITION = none MAME's whole boot has 0 REQUEST SENSE
#
# i.e. a command that never COMPLETED, so nobody reported a fault -- the driver
# just gave up.
#
# MAME baseline (tools/mame_scsi96_capture.lua, same 7.5.3 image):
#   0x02 "Reset chip"     x3   ALL at 4.50s
#   0x03 "Reset SCSI bus" x1   at 4.83s
#   -> ZERO chip/bus resets after ~4.8s.  So any post-init reset on HW is the
#      driver recovering from something, and MAME never needs to.
#
# 53C96 command register is reg 3.  glue.v:223 decodes the chip at
# io_off 0x00F000..0x00F0FF inside the 0x5000_0000 I/O window, and the register
# index is (offset & 0xff) >> 4, so reg 3 = 0x5000F030.  `watch` compares
# PHYSICAL addresses, so this works without the (de-instantiated) trace ring.
#
# Byte lane: 0x5000F030 & 3 == 0, so a byte write lands in wstrb bit 3 with the
# data in bits 31:24 -> value 0x0N000000, lanes 0x8.
#
# Arming is DELAYED past init on purpose: MAME shows legitimate resets at 4.5s,
# and arming from reset would just halt on those.

cd "$(dirname "$0")/.." || exit 1
SETTLE=${SETTLE:-100}      # let boot get past init before arming
WAIT=${WAIT:-400}
CMDREG=0x5000F030
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }

echo "=== disarm, reset, let boot pass init (${SETTLE}s) ==="
for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
    jt "$c" >/dev/null; done
jt "vio-hard-reset" 30 >/dev/null
s=$SECONDS
while [ $((SECONDS-s)) -lt "$SETTLE" ]; do jt "halt-status" 8 >/dev/null; done

echo "=== arming watches on the 53C96 command register ($CMDREG) ==="
jt "watch 0 $CMDREG w value 0x03000000 lanes 0x8" 25 | grep -E "armed|ERROR" | sed 's/^/  slot0 (Reset SCSI bus): /'
jt "watch 1 $CMDREG w value 0x02000000 lanes 0x8" 25 | grep -E "armed|ERROR" | sed 's/^/  slot1 (Reset chip):     /'
echo "  waiting up to ${WAIT}s for a post-init SCSI reset..."

s=$SECONDS; H=0
while [ $((SECONDS-s)) -lt "$WAIT" ]; do
    jt "halt-status" 8 | grep -q 'effective=1' && { H=1; break; }
done

if [ "$H" != 1 ]; then
    echo
    echo "RESULT: no post-init chip/bus reset in ${WAIT}s."
    echo "  The driver did NOT perform reset-style recovery in this window."
    echo "  That WEAKENS but does not disprove the timeout theory -- the SCSI"
    echo "  Manager can return ioErr on a timeout without resetting the bus."
    jt "watch status" 20 | grep -E "wp HIT|wp0|wp1" | sed 's/^/  /'
    exit 2
fi

echo
echo "*** HALTED — a post-init SCSI reset was issued ***"
jt "watch status" 20 | grep -E "wp HIT|wp0|wp1" | sed 's/^/  /'
echo "  PC = $(jt "pc" 15 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)"
jt "live-arch" 40 | grep -E "^> (D0|D1|A0|A1|A5|A7|SR|PC) " | sed 's/^/  /'
RAW=$(jt "vio-read scsi" 40 | grep -oE 'vio_scsi_sd = [0-9a-fA-F]+' | awk '{print $3}')
[ -n "$RAW" ] && python3 -c "
v=int('$RAW',16); g=lambda h,l:(v>>l)&((1<<(h-l+1))-1)
print(f'  SD: completions={g(43,28)} errors={g(27,20)} cause={g(19,16)} sticky={g(3,3)} busy={g(2,2)} irq={g(1,1)} drq={g(0,0)}')"
RE=$(jt "dump-mem 0x00000A60 1" 25 | grep -oE '= 0x[0-9a-fA-F]{8}' | head -1 | sed 's/= //')
[ -n "$RE" ] && python3 -c "
n=int('$RE',16); x=(n>>16)&0xFFFF
print('  ResErr =', x-0x10000 if x>=0x8000 else x)"
echo
echo "  A reset here means the driver hit an error it had to recover from,"
echo "  and MAME never does this after 4.8s on the same image."
