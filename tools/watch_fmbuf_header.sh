#!/bin/bash
# watch_fmbuf_header.sh — log every write to the block HEADER of the File
# Manager's volume control cache buffer, to settle a question I have now
# answered two different ways from inference alone.
#
# At the clobber, header 0x00010C58 = tag=0x4C size=0x130 zone=0x00002000
# (a well-formed non-relocatable SysZone block) and vcbCtlBuf still = 0x00010C60.
# That is consistent with BOTH:
#   (a) the FM's block was DISPOSED and the storage legitimately reallocated
#       -> the FM is left holding a stale pointer
#   (b) the allocator carved a new block INSIDE storage the FM still owns
#       -> a heap/allocator failure
# The header's write history distinguishes them:
#   free-then-realloc  -> header written 2+ times (tag goes to free, then set)
#   carved in place    -> header written once, while the FM still owns the buffer
#
# slot0: any write to the header longword at 0x00010C58
# slot1: the gamma ramp store, to timestamp the clobber
# break-pc marker: proves the boot reached the CDEF failure (only ~1 in 3 do)

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-300}
MAXH=${MAXH:-14}
BOOTS=${BOOTS:-5}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
wait_halt() { local s=$SECONDS
    while [ $((SECONDS-s)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0; done; return 1; }

for boot in $(seq 1 "$BOOTS"); do
echo "=== boot $boot/$BOOTS ==="
SAW=0; REACHED=0
for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
    jt "$c" >/dev/null; done
# Value-filter on the header the NEW owner's block actually gets
# (observed at the clobber halt: tag=0x4C size=0x130 zone=SysZone).
# An unfiltered watch here halts the CPU three times during the ROM RAM test
# (PCs 0x40847298 / 0x408472fa writing the 6db patterns), and every boot
# instrumented that way then failed to reach the Finder -- the instrumentation
# itself was likely perturbing the boot.
jt "watch 0 0x00010C58 w value 0x4C000130 lanes 0xF" 25 | grep -E "armed|ERROR" | sed 's/^/  slot0 header:  /'
jt "watch 1 0x00010C6C w value 0x0005090B lanes 0xF" 25 | grep -E "armed|ERROR" | sed 's/^/  slot1 gamma:   /'
jt "break-pc 0x40815E6A" 25 | grep -E "slot=" | sed 's/^/  marker:        /'
jt "vio-hard-reset" 30 >/dev/null
START=$SECONDS

for i in $(seq 1 "$MAXH"); do
    wait_halt "$WAIT" || { echo "  no further halt in ${WAIT}s"; break; }
    T=$((SECONDS-START))
    HIT=$(jt "watch status" 20 | grep -oE 'wp HIT: .*' | head -1)
    PC=$(jt "pc" 15 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
    if echo "$HIT" | grep -q "none latched"; then
        case "${PC,,}" in
          0x40815e6a) echo "  [$i] t=${T}s  MARKER: reached the CDEF failure"; REACHED=1 ;;
          *)          echo "  [$i] t=${T}s  unrelated halt at $PC" ;;
        esac
    else
        SLOT=$(echo "$HIT" | grep -oE 'slot=[0-9]' | head -1)
        DATA=$(echo "$HIT" | grep -oE 'data=0x[0-9a-fA-F]{8}' | sed 's/data=//')
        WPC=$(echo "$HIT" | grep -oE 'pc=0x[0-9a-fA-F]{8}' | sed 's/pc=//')
        if [ "$SLOT" = "slot=0" ]; then
            TAG=$(python3 -c "
v=int('${DATA:-0}',16); t=(v>>24)&0xFF; sz=v&0xFFFFFF
kind={0x00:'FREE'}.get(t, 'in-use' if t else 'FREE')
print(f'tag=0x{t:02X} ({kind}) size=0x{sz:06X}')")
            echo "  [$i] t=${T}s  HEADER write: $DATA  $TAG   from PC=$WPC"
        else
            echo "  [$i] t=${T}s  GAMMA clobber write from PC=$WPC"
        fi
        SAW=1
    fi
    jt "halt-clear" 20 >/dev/null; jt "cont" 30 >/dev/null
done
echo "  boot $boot summary: reached_failure=$REACHED events=$SAW"
[ "$REACHED" = "1" ] && { echo; echo "Header write history above decides free-then-realloc vs carved-in-place."; exit 0; }
echo "  (boot did not reach the failure; retrying)"; echo
done
echo "=== no boot reached the failure ==="; exit 1
