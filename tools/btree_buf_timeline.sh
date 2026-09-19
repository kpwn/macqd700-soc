#!/bin/bash
# btree_buf_timeline.sh — does the File Manager's B-tree node buffer EVER hold a
# real HFS node, or is it stale from before it was allocated?
#
# Separates the two readings of the ramp found at 0x00010C60:
#   (a) CLOBBER  -- a valid node appears, then gets overwritten by the ramp
#   (b) STALE    -- a node never appears at all; the ramp predates the buffer
#                   and the FM simply never loads one
#
# An HFS B-tree node descriptor starts:
#   +0 ndFLink(4) +4 ndBLink(4) +8 ndType(1) +9 ndNHeight(1) +10 ndNRecs(2)
# ndType is 0xFF (leaf), 0x00 (index), 0x01 (header) or 0x02 (map), and
# ndNHeight is small.  The expected extents node begins
#   0000000000000000 ff01 0004 ...
# The observed ramp begins
#   0000000000000001 0100 0008 0005090b ...

cd "$(dirname "$0")/.." || exit 1
DUR=${DUR:-330}
ADDR=0x00010C60
jt() { JT_WAIT=${2:-12} tools/jt.sh "$1" 2>&1; }

for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
    jt "$c" >/dev/null; done
echo "=== reset, then sample $ADDR every ~15s for ${DUR}s ==="
jt "vio-hard-reset" 30 >/dev/null
s=$SECONDS
while [ $((SECONDS-s)) -lt "$DUR" ]; do
    HEX=$(jt "dump-mem $ADDR 4" 25 | grep -oE '= 0x[0-9a-fA-F]{8}' | sed 's/= 0x//' | tr -d '\n')
    T=$((SECONDS-s))
    if [ -n "$HEX" ]; then
        python3 -c "
h='$HEX'
if len(h)>=32:
    b=bytes.fromhex(h[:32])
    nd=b[8]; hgt=b[9]; nrecs=int.from_bytes(b[10:12],'big')
    kind={0xFF:'LEAF',0x00:'INDEX',0x01:'HEADER',0x02:'MAP'}.get(nd)
    tag=f'NODE({kind} height={hgt} nrecs={nrecs})' if (kind and hgt<8 and nrecs<200) else 'not-a-node'
    print(f'  t={$T:3d}s  {h[:32]}  {tag}')
"
    fi
    sleep 13
done
echo
echo "If no line ever says NODE(...), the buffer NEVER held an HFS node ->"
echo "reading (b): the ramp is stale, nothing clobbered it, and the open"
echo "question is why the File Manager never loads the node."
