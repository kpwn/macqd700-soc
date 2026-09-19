#!/bin/bash
# btree_buf_ownership.sh — establish the ORDER of the two claims on 0x00010C60.
#
# On boots that reach the failure (layout verified reproducible twice):
#   * the File Manager records 0x00010C60 as its volume control cache buffer
#     -- it appears in VCB+168 (vcbCtlBuf, VCB=0x00012E40 -> 0x00012EE8) and in
#     BTCB+8 for BOTH the extents (0x00010340) and catalog (0x000103A0) trees
#   * a BlockMove copies a gamma ramp INTO 0x00010C60 during early boot
#
# If the same storage has two owners, that is a Memory Manager bug and the read
# failure is a downstream symptom.  Order decides the story:
#   gamma first, then FM records it  -> the FM was handed already-used memory
#   FM first, then gamma lands on it -> something wrote through a stale pointer
#
# slot0: any write to vcbCtlBuf (0x00012EE8)      -- the FM staking its claim
# slot1: the ramp longword 0x0005090B -> 0x00010C6C -- the gamma copy
# Both armed before reset (watch config survives reset); log which fires, in
# order, then continue.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-300}
MAXH=${MAXH:-10}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
wait_halt() { local s=$SECONDS
    while [ $((SECONDS-s)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0; done; return 1; }

BOOTS=${BOOTS:-6}
for boot in $(seq 1 "$BOOTS"); do
echo "=== boot $boot/$BOOTS ==="
REACHED=0; SAW=0
for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
    jt "$c" >/dev/null; done
# Value-filter slot0 on the BUFFER POINTER itself.  An unfiltered watch here
# just catches the ROM RAM test writing the 6db pattern to this address three
# times in early boot (PCs 0x40847292 / 0x4084730a) and tells you nothing.
jt "watch 0 0x00012EE8 w value 0x00010C60 lanes 0xF" 25 | grep -E "armed|ERROR" | sed 's/^/  slot0 vcbCtlBuf<-0x10C60: /'
jt "watch 1 0x00010C6C w value 0x0005090B lanes 0xF" 25 | grep -E "armed|ERROR" | sed 's/^/  slot1 gamma ramp:         /'
# Marker: proves the boot actually reached the CDEF failure.  Without it, "no
# hits" is indistinguishable from "this boot never got there" -- which is how
# the previous run wasted a boot.
jt "break-pc 0x40815E6A" 25 | grep -E "slot=" | sed 's/^/  marker CDEF-fail:         /'
jt "vio-hard-reset" 30 >/dev/null
START=$SECONDS

for i in $(seq 1 "$MAXH"); do
    wait_halt "$WAIT" || { echo "  no further halt in ${WAIT}s"; break; }
    T=$((SECONDS-START))
    HIT=$(jt "watch status" 20 | grep -oE 'wp HIT: .*' | head -1)
    PC=$(jt "pc" 15 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
    if echo "$HIT" | grep -q "none latched"; then
        case "${PC,,}" in
          0x40815e6a) echo "  [$i] t=${T}s  MARKER: reached the CDEF failure (so this boot DID get there)"; REACHED=1 ;;
          *)          echo "  [$i] t=${T}s halted at $PC, no watch latched -- unrelated halt" ;;
        esac
    else
        SLOT=$(echo "$HIT" | grep -oE 'slot=[0-9]' | head -1)
        case "$SLOT" in
          slot=0) WHO="*** FM records vcbCtlBuf <- 0x00010C60 ***"; SAW=1 ;;
          slot=1) WHO="*** gamma BlockMove writes 0x00010C60 ***"; SAW=1 ;;
          *)      WHO="?" ;;
        esac
        echo "  [$i] t=${T}s  $WHO"
        echo "        $HIT"
        echo "        halted PC=$PC"
    fi
    jt "halt-clear" 20 >/dev/null
    jt "cont" 30 >/dev/null
done
echo "  boot $boot summary: reached_failure=$REACHED watch_events=$SAW"
if [ "$SAW" = "1" ]; then
    echo
    echo "Order of the events above decides whether the File Manager was handed"
    echo "memory that was already in use (a Memory Manager / allocation bug)."
    exit 0
fi
echo "  (no ownership events on this boot; retrying)"
echo
done
echo "=== no boot produced an ownership event in $BOOTS attempts ==="
exit 1
