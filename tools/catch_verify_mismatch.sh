#!/bin/bash
# catch_verify_mismatch.sh — the File Manager's buffer routine at 0x40811A00 has
# a VERIFY arm that returns ioErr on a data mismatch:
#     40811a0e  bmis 0x40811a22     ; d0<0 -> verify
#     40811a22  moveq #-36,%d0      ; pre-set ioErr
#     40811a26  cmpmb %a0@+,%a1@+   ; compare
#     40811a2c  bnes ...            ; MISMATCH -> return ioErr
#     40811a2e  bras ...            ; match -> return 0
# A mismatch is memory-to-memory, so it produces ioErr with NO disk I/O and is
# perfectly deterministic -- exactly the failure signature we have been unable
# to explain (same PC, same offset, across two bitstreams, bus idle).
#
# Break at 0x40811A22 (before the compare) and capture the two buffers so the
# DIFFERENCE can be inspected -- that difference is the corruption.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-260}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }

for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
    jt "$c" >/dev/null; done
jt "break-pc 0x40811A22" 25 | grep -E "slot=" | sed 's/^/  /'
jt "vio-hard-reset" 30 >/dev/null
echo "  waiting ${WAIT}s for a verify comparison"

s=$SECONDS; H=0
while [ $((SECONDS-s)) -lt "$WAIT" ]; do
    jt "halt-status" 8 | grep -q 'effective=1' && { H=1; break; }
done
[ "$H" = 1 ] || { echo "  no verify comparison in ${WAIT}s"; exit 1; }

echo "  *** verify arm reached ***"
ARCH=$(jt "live-arch" 40)
echo "$ARCH" | grep -E "^> (D0|D2|D6|A0|A1|A2|A3|A5) " | sed 's/^/    /'
A0=$(echo "$ARCH" | grep -oE '^> A0 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | head -1)
A1=$(echo "$ARCH" | grep -oE '^> A1 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | head -1)
D6=$(echo "$ARCH" | grep -oE '^> D6 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | head -1)
echo "    comparing $D6 bytes: A0=$A0  vs  A1=$A1"
jt "dcache-op push" 30 >/dev/null
for L in "$A0:bufA" "$A1:bufB"; do
    A=${L%%:*}; N=${L##*:}
    AA=$(printf '0x%08X' $(( $(printf '%d' "$A") & ~3 )))
    echo "    --- $N @ $AA ---"
    jt "dump-mem $AA 8" 40 | grep -oE '= 0x[0-9a-fA-F]{8}' | sed 's/= 0x//' | tr -d '\n' | sed 's/^/      /'
    echo
done
echo "    (a difference between these two buffers is the corruption that turns"
echo "     into ioErr with no disk access)"
