#!/bin/bash
# catch_cdef_loadfail.sh — halt exactly where _LoadResource fails to reload the
# purged CDEF, and read the error globals that say WHY.
#
# Site (fixed ROM address -- no Finder relocation to fight):
#   40815e58  tstl %a0@        ; CDEF handle's master ptr NIL? (purged)
#   40815e5a  bne  40815e70    ; loaded -> call it
#   40815e64  a9a2             ; _LoadResource -- try to reload
#   40815e66  tstl %a0@        ; STILL NIL -> load FAILED
#   40815e6a  movew #$58,%d0   ; <-- BREAK HERE (only reached on failure)
#   40815e6e  a9c9             ; _SysError(88) = dsCDEFnFnd
#
# Breaking at 40815e6a means the load has already failed, so ResErr/MemErr hold
# the reason.  This is the first breakpoint in this investigation on a STABLE
# address: ROM does not move between boots, unlike every Finder address that
# burned earlier attempts.
#
#   ResErr  = lowmem 0x0A60 (word)  -- Resource Manager error
#   MemErr  = lowmem 0x0220 (word)  -- Memory Manager error
# A memFullErr (-108) in MemErr points at heap exhaustion; a resource-level
# error (e.g. resNotFound -192, or an I/O error like -36) points at the
# file/disk path instead.  Those are different bugs, so read them, do not guess.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-420}
BP=0x40815e6a
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }

echo "=== arming break-pc at $BP (the CDEF load-failure arm, in ROM) ==="
jt "watch 0 off" >/dev/null; jt "watch 1 off" >/dev/null
jt "atrap 0 off" >/dev/null; jt "atrap 1 off" >/dev/null
jt "halt-clear" >/dev/null
jt "break-pc $BP" 25 | grep -E "slot=|ERROR" | sed 's/^/  /'
jt "vio-hard-reset" 30 >/dev/null

s=$SECONDS; H=0
while [ $((SECONDS-s)) -lt "$WAIT" ]; do
    jt "halt-status" 8 | grep -q 'effective=1' && { H=1; break; }
done
[ "$H" = 1 ] || { echo "FAIL: never reached the CDEF load-failure arm in ${WAIT}s"; exit 1; }

PC=$(jt "pc" 15 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
echo "  halted at PC=$PC (expected $BP)"
echo
ARCH=$(jt "live-arch" 40); echo "$ARCH" | sed 's/^/  /'
reg() { echo "$ARCH" | grep -oE "^> $1 = 0x[0-9a-f]+" | grep -oE '0x[0-9a-f]+' | head -1; }

echo
echo "=== error globals ==="
RES=$(jt "dump-mem 0x00000A60 1" 25 | grep -oE '= 0x[0-9a-fA-F]{8}' | sed 's/= //')
MEM=$(jt "dump-mem 0x00000220 1" 25 | grep -oE '= 0x[0-9a-fA-F]{8}' | sed 's/= //')
echo "  ResErr longword @0x0A60 = $RES"
echo "  MemErr longword @0x0220 = $MEM"
python3 - "$RES" "$MEM" <<'PY'
import sys
def w(v,hi):
    if not v or not v.startswith('0x'): return None
    n=int(v,16); x=(n>>16)&0xFFFF if hi else n&0xFFFF
    return x-0x10000 if x>=0x8000 else x
names={-108:'memFullErr (heap exhausted)',-192:'resNotFound',-193:'resFNotFound',
       -36:'ioErr (disk I/O)',-49:'opWrErr',-42:'tmfoErr',0:'noErr',-116:'memPurErr',
       -117:'memAdrErr',-111:'memWZErr',-109:'nilHandleErr',-120:'dirNFErr'}
for lbl,v in (("ResErr",sys.argv[1]),("MemErr",sys.argv[2])):
    for hi in (True,False):
        x=w(v,hi)
        if x is None: continue
        half='hi' if hi else 'lo'
        print(f"  {lbl} {half}-word = {x:6d}   {names.get(x,'')}")
PY

echo
echo "=== the CDEF handle ==="
A0=$(reg A0)
echo "  A0 = $A0"
if [ -n "$A0" ]; then
    jt "dump-mem $(printf '0x%08X' $(( $(printf '%d' "$A0") & ~3 ))) 4" 30 | grep '^> mem' | sed 's/^/    /'
fi
echo
echo "=== pc-trace ==="
jt "pc-trace 40" 40 | grep '^> trace' | tail -24 | sed 's/^/  /'
