#!/bin/bash
# catch_ctlcache_free.sh — catch the moment the File Manager's 8672-byte control
# cache block is marked FREE, and identify who does it.
#
# Established: the SysZone chain is CONSISTENT (186 blocks, no break), and at the
# clobber the block at 0x00010C58 is a legitimate 304-byte in-use block. So the
# heap is not corrupt -- the FM's 0x21E0 block was FREED and the space re-carved,
# while the mounted VCB still points at it (vcbCtlBuf=0x00010C60, confirmed in a
# single halt). MAME never frees its equivalent: that block takes 1292 writes
# from the FM cache manager and stays live.
#
# When a block is freed the MM writes its header with tag 0 (free), keeping the
# size -- so watch for 0x000021E0 landing at 0x00010C58.
# If that never fires, fall back to an unfiltered watch and read the data.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-250}
VAL=${VAL:-0x000021E0}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }

for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
    jt "$c" >/dev/null; done
jt "watch 0 0x00010C58 w value $VAL lanes 0xF" 25 | grep -E "armed|ERROR" | sed 's/^/  /'
jt "vio-hard-reset" 30 >/dev/null
echo "  waiting ${WAIT}s for the control-cache block to be marked FREE ($VAL)"

s=$SECONDS; H=0
while [ $((SECONDS-s)) -lt "$WAIT" ]; do
    jt "halt-status" 8 | grep -q 'effective=1' && { H=1; break; }
done
[ "$H" = 1 ] || { echo "  did not fire (the free may write a different header value)"; exit 1; }

echo "  *** CAUGHT the control cache being freed ***"
jt "watch status" 20 | grep -oE 'wp HIT: .*' | sed 's/^/    /'
PC=$(jt "pc" 15 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
echo "    halted PC = $PC"
jt "live-arch" 40 | grep -E "^> (D0|D1|A0|A1|A2|A5|A6|A7|PC) " | sed 's/^/    /'
echo "    --- pc-trace (who called the dispose) ---"
jt "pc-trace 32" 40 | grep '^> trace' | tail -18 | sed 's/^/    /'
echo "    --- stack: return addresses back to the CALLER of _DisposPtr ---"
SP=$(jt "live-arch" 40 | grep -oE "^> A7 = 0x[0-9a-f]+" | grep -oE "0x[0-9a-f]+" | head -1)
if [ -n "$SP" ]; then
    SPA=$(printf "0x%08X" $(( $(printf "%d" "$SP") & ~3 )))
    jt "dump-mem $SPA 24" 60 | grep "^> mem" | sed "s/^/      /"
    echo "      (look for the first RAM address -- ROM 0x4080xxxx entries are the"
    echo "       Memory Manager itself; a 0x000xxxxx address is the real caller)"
fi
echo "    --- code at the freeing PC ---"
tools/dis_at.sh $(printf "0x%08X" $(( $(printf '%d' "$PC") - 0x30 ))) 20 2>&1 | sed 's/^/    /'
