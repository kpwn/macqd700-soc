#!/bin/bash
# reserr_before_after.sh — is the ioErr (-36) in ResErr actually PRODUCED by the
# failing _LoadResource, or is it stale from some earlier Resource Manager call?
#
# ResErr is sticky: every Resource Manager call overwrites it and nothing clears
# it in between, so reading -36 *after* a failure proves nothing on its own.
# Bracket the call instead -- both are fixed ROM addresses:
#
#   40815e64  a9a2   _LoadResource   <-- BEFORE  (slot 0)
#   40815e6a  movew #$58,%d0         <-- AFTER, failure arm only (slot 1)
#
# Read ResErr/MemErr at each halt.  If ResErr is something other than -36 at
# e64 and -36 at e6a, the load itself produced the I/O error -> the disk/SCSI
# read path is implicated.  If it is ALREADY -36 at e64, the error came from
# earlier and _LoadResource may be failing for an unrelated reason.
#
# Read-only: no memory is written, so nothing is perturbed.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-420}
MAXH=${MAXH:-14}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
wait_halt() { local s=$SECONDS
    while [ $((SECONDS-s)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0; done; return 1; }

word() { # $1 = addr of a 16-bit lowmem global -> signed decimal
    local v=$(jt "dump-mem $1 1" 25 | grep -oE '= 0x[0-9a-fA-F]{8}' | head -1 | sed 's/= //')
    [ -n "$v" ] || { echo "?"; return; }
    python3 -c "
n=int('$v',16); x=(n>>16)&0xFFFF
print(x-0x10000 if x>=0x8000 else x)"
}

echo "=== arming break-pc slot0=0x40815E64 (_LoadResource) slot1=0x40815E6A (fail arm) ==="
jt "watch 0 off" >/dev/null; jt "watch 1 off" >/dev/null
jt "atrap 0 off" >/dev/null; jt "atrap 1 off" >/dev/null
jt "halt-clear" >/dev/null
jt "break-pc 0x40815E64" 25 | grep -E "slot=|ERROR" | sed 's/^/  /'
jt "break-pc 0x40815E6A" 25 | grep -E "slot=|ERROR" | sed 's/^/  /'
jt "vio-hard-reset" 30 >/dev/null

for i in $(seq 1 "$MAXH"); do
    wait_halt "$WAIT" || { echo "  no further halt in ${WAIT}s"; break; }
    PC=$(jt "pc" 15 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
    RE=$(word 0x00000A60); ME=$(word 0x00000220)
    case "$PC" in
      0x40815e64|0x40815E64) TAG="BEFORE _LoadResource" ;;
      0x40815e6a|0x40815E6A) TAG="AFTER  -> FAILED (SysError 88 next)" ;;
      *) TAG="(unexpected)" ;;
    esac
    printf "  %2d  PC=%s  ResErr=%-6s MemErr=%-6s  %s\n" "$i" "$PC" "$RE" "$ME" "$TAG"
    jt "halt-clear" 20 >/dev/null
    jt "cont" 30 >/dev/null
done
echo
echo "Interpretation: find a BEFORE immediately followed by an AFTER."
echo "  ResErr changes to -36 across the pair -> the load produced the I/O error."
echo "  ResErr already -36 at BEFORE          -> stale; the load failed for another reason."
