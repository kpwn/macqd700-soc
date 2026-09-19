#!/bin/bash
# reserr_full_chain.sh — halt on the failing _Read and derive the ENTIRE File
# Manager chain LIVE, in that one halt, then compare the extents B-tree node
# against the one in the disk image.
#
# METHOD RULE (violated 3x before this): on this machine only ROM addresses and
# lowmem globals are stable across boots.  Every heap address -- FCB array, VCB,
# BTCB, buffers -- MUST be re-derived from lowmem on the SAME boot it is read
# on.  A previous run walked VCB 0x12E40 / BTCB 0x10340 / buffer 0x10C60 from
# one boot and re-read them on another, where 0x10C60 turned out to be
# untouched RAM (db6db6db...), producing a "gamma ramp clobber" claim that had
# to be retracted.  Everything below is read in a single halt.
#
# Also: `dump-mem` refuses unaligned addresses, and FCB bases are routinely
# 2-mod-4 (refnum 2 -> FCBSPtr+2), so every field read stitches two aligned
# longwords.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-280}
TRIES=${TRIES:-3}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
wait_halt() { local s=$SECONDS
    while [ $((SECONDS-s)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0; done; return 1; }
rdl() { jt "dump-mem $1 1" 25 | grep -oE '= 0x[0-9a-fA-F]{8}' | head -1 | sed 's/= 0x//'; }

# read a longword at ANY alignment by stitching the two covering aligned words
rdu() {
    local a=$(printf '%d' "$1"); local b=$(( a & ~3 )); local off=$(( a - b ))
    local w1=$(rdl $(printf '0x%08X' $b))
    local w2=$(rdl $(printf '0x%08X' $(( b + 4 ))))
    [ -n "$w1" ] && [ -n "$w2" ] || { echo ""; return; }
    python3 -c "
v=(int('$w1',16)<<32)|int('$w2',16)
print('%08X' % ((v >> (32-8*$off)) & 0xFFFFFFFF))"
}
rdw() {  # 16-bit at any alignment
    local l=$(rdu "$1"); [ -n "$l" ] && echo "${l:0:4}" || echo ""
}

for t in $(seq 1 "$TRIES"); do
    echo "=== attempt $t/$TRIES ==="
    for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
        jt "$c" >/dev/null; done
    jt "watch 0 0x00000A60 w value 0xFFDC0000 lanes 0xC" 25 | grep -E "armed|ERROR" | sed 's/^/  /'
    jt "vio-hard-reset" 30 >/dev/null
    wait_halt "$WAIT" || { echo "  no ResErr<-ioErr in ${WAIT}s; retrying"; continue; }

    # GATE: a halt is not proof the WATCH fired.  A previous run halted early in
    # boot for another reason and derived a whole chain from garbage
    # (A6=0xb6db6db6, FCBSPtr=0xB6DB6DB6 -- the RAM init pattern).  Require the
    # watchpoint to have actually latched.
    HIT=$(jt "watch status" 20 | grep -oE 'wp HIT: .*' | head -1)
    if echo "$HIT" | grep -q "none latched"; then
        echo "  halted but the ResErr watch did NOT latch ($HIT) -- not our event; retrying"
        continue
    fi
    echo "  HALTED on the failing _Read: $HIT"

    ARCH=$(jt "live-arch" 40)
    A6=$(echo "$ARCH" | grep -oE '^> A6 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | head -1)
    # sanity-check A6 before deriving anything from it
    A6N=$(printf '%d' "$A6" 2>/dev/null || echo 0)
    if [ "$A6N" -lt 4096 ] || [ "$A6N" -gt 2147483647 ]; then
        echo "  A6=$A6 is not a plausible stack frame -- refusing to derive a chain from it; retrying"
        continue
    fi
    PB=$(printf '0x%08X' $(( A6N - 50 )))
    echo "  A6=$A6 -> IOParam at $PB"
    REFNUM=$(rdw $(printf '0x%08X' $(( $(printf '%d' "$PB") + 24 ))))
    POS=$(rdu $(printf '0x%08X' $(( $(printf '%d' "$PB") + 46 ))))
    CNT=$(rdu $(printf '0x%08X' $(( $(printf '%d' "$PB") + 36 ))))
    echo "  ioRefNum=0x$REFNUM  ioReqCount=0x$CNT  ioPosOffset=0x$POS"

    # FCBSPtr is the long at lowmem 0x34E (2-mod-4)
    FCBS=$(rdu 0x0000034E); echo "  FCBSPtr=0x$FCBS"
    FCB=$(( 0x$FCBS + 0x$REFNUM ))
    printf "  FCB for this refnum = 0x%08X\n" $FCB
    NAMEOFF=$(( FCB + 62 ))
    NM=$(jt "dump-mem $(printf '0x%08X' $(( NAMEOFF & ~3 ))) 3" 30 | grep -oE '= 0x[0-9a-fA-F]{8}' | sed 's/= 0x//' | tr -d '\n')
    echo "  fcbCName raw=$NM"
    VCB=$(rdu $(printf '0x%08X' $(( FCB + 20 ))))
    echo "  fcbVPtr (VCB) = 0x$VCB"
    XTREF=$(rdw $(printf '0x%08X' $(( 0x$VCB + 164 ))))
    echo "  vcbXTRef = 0x$XTREF"
    XTFCB=$(( 0x$FCBS + 0x$XTREF ))
    printf "  extents-file FCB = 0x%08X\n" $XTFCB
    BTCB=$(rdu $(printf '0x%08X' $(( XTFCB + 34 ))))
    echo "  fcbBTCBPtr (BTCB) = 0x$BTCB"
    echo "  --- BTCB dump ---"
    jt "dump-mem $(printf '0x%08X' $(( 0x$BTCB & ~3 ))) 6" 30 | grep '^> mem' | sed 's/^/    /'
    BUF=$(rdu $(printf '0x%08X' $(( 0x$BTCB + 8 ))))
    echo "  BTCB+8 (candidate node buffer) = 0x$BUF"
    if [ -n "$BUF" ] && [ "$BUF" != "00000000" ]; then
        echo "  --- first 32 bytes of that buffer ---"
        jt "dump-mem $(printf '0x%08X' $(( 0x$BUF & ~3 ))) 8" 40 | grep -oE '= 0x[0-9a-fA-F]{8}' | sed 's/= 0x//' | tr -d '\n' | tee /tmp/livebuf.hex
        echo
        python3 - <<'PY'
h=open('/tmp/livebuf.hex').read().strip()
exp=open('/tmp/xt_node_expected.bin','rb').read()
print("    HW  :", h[:64])
print("    IMG :", exp[:32].hex())
if len(h)>=24:
    b=bytes.fromhex(h[:32])
    nd=b[8]; hgt=b[9]; nr=int.from_bytes(b[10:12],'big')
    kind={0xFF:'LEAF',0x00:'INDEX',0x01:'HEADER',0x02:'MAP'}.get(nd)
    print(f"    -> ndType=0x{nd:02X} ({kind}) height={hgt} nrecs={nr}")
    if h[:32].lower()==exp[:16].hex(): print("    MATCHES the image node")
    elif set(h[:32].lower())<= set('db6'): print("    *** uninitialised RAM pattern ***")
    else: print("    differs from the image node")
PY
    fi
    exit 0
done
echo "=== never caught the failing read ==="; exit 1
