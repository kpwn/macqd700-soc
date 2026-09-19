#!/bin/bash
# gamma_vs_vcb.sh — settle whether the "Mac Std Gamma" copy lands on the File
# Manager's LIVE control cache, by reading BOTH facts in the SAME halt.
#
# Every previous attempt compared a vcbCtlBuf read on one boot against a gamma
# destination on another, and the heap layout varies between boots -- which is
# how five contradictory conclusions were reached. This bootstraps ONLY from
# fixed lowmem:
#     VCBQHdr qHead @ 0x358  ->  VCB  ->  vcbCtlBuf at VCB+168
# and compares it to the gamma destination in the very same stop.
#
# Verdict per boot:
#   vcbCtlBuf == 0x00010C60  -> the gamma copy overwrote a LIVE FM structure
#                               (allocator handed out storage inside a live block)
#   vcbCtlBuf != 0x00010C60  -> no collision on this boot; the gamma write is
#                               harmless here and the failure needs another cause

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-250}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
rdl() { jt "dump-mem $1 1" 25 | grep -oE '= 0x[0-9a-fA-F]{8}' | head -1 | sed 's/= 0x//'; }

for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
    jt "$c" >/dev/null; done
jt "watch 0 0x00010C6C w value 0x0005090B lanes 0xF" 25 | grep -E "armed|ERROR" | sed 's/^/  /'
jt "vio-hard-reset" 30 >/dev/null

s=$SECONDS; H=0
while [ $((SECONDS-s)) -lt "$WAIT" ]; do
    jt "halt-status" 8 | grep -q 'effective=1' && { H=1; break; }
done
[ "$H" = 1 ] || { echo "  gamma write did not happen this boot"; exit 1; }

echo "  *** halted at the gamma copy ***"
jt "watch status" 20 | grep -oE 'wp HIT: .*' | sed 's/^/    /'
jt "dcache-op push" 30 >/dev/null

QH=$(rdl 0x00000358)
echo "    VCBQHdr qHead = 0x$QH"
N=$(printf '%d' "0x${QH:-0}" 2>/dev/null || echo 0)
FOUND=0
for k in 1 2 3 4; do
    [ "$N" -gt 4096 ] && [ "$N" -lt 2147483647 ] || break
    CTL=$(rdl $(printf '0x%08X' $(( N + 168 ))))
    MADR=$(rdl $(printf '0x%08X' $(( N + 80 ))))
    NM=$(jt "dump-mem $(printf '0x%08X' $(( (N + 44) & ~3 ))) 6" 40 | grep -oE '= 0x[0-9a-fA-F]{8}' | sed 's/= 0x//' | tr -d '\n')
    printf "    VCB[%d]=0x%08X  vcbCtlBuf=0x%s  vcbMAdr=0x%s\n" "$k" "$N" "${CTL:-?}" "${MADR:-?}"
    python3 -c "
import binascii
h='${NM:-}'
if h:
    b=binascii.unhexlify(h)
    ln=b[3] if len(b)>3 else 0
    if 0<ln<28 and len(b)>=4+ln:
        print('      name=%r' % b[4:4+ln].decode('mac-roman','replace'))"
    if [ "${CTL:-}" = "00010c60" ] || [ "${CTL:-}" = "00010C60" ]; then
        echo "      *** vcbCtlBuf == the gamma destination 0x00010C60 ***"
        echo "      => the gamma copy is overwriting a LIVE File Manager cache."
        FOUND=1
    fi
    N=$(printf '%d' "0x$(rdl $(printf '0x%08X' $N))" 2>/dev/null || echo 0)
done
echo
if [ "$FOUND" = "1" ]; then
    echo "  VERDICT: COLLISION CONFIRMED in a single halt."
else
    echo "  VERDICT: no VCB points at 0x00010C60 on this boot -- the gamma write"
    echo "           did NOT hit a live FM cache here."
fi
