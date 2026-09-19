#!/bin/bash
# loadres_read_attempt.sh — does the failing _LoadResource even TRY to read the
# disk?  Brackets the call and compares the SD completion counter across it.
#
# Why this matters: at the failure ResErr=-36 (ioErr) but the SD layer reports
# ZERO errors (3566 completions, sticky=0) on the same boot.  Two readings fit:
#   (a) ioErr is stale, and the load failed for another reason entirely;
#   (b) ioErr is REAL but raised ABOVE the disk -- the Resource Manager never
#       got as far as issuing a read (bad/closed file refnum, damaged map), so
#       our SD layer never saw anything to fail at.
# The SD completion counter separates them:
#   completions UNCHANGED across the call -> no read attempted -> (b)
#   completions INCREASED                  -> a read happened and still failed
#
# Both breakpoints are FIXED ROM addresses:
#   40815e64  a9a2   _LoadResource       (only reached when the CDEF is purged)
#   40815e6a  movew #$58,%d0             (only reached when the reload FAILED)
# Boots are flaky and only some reach the dialog, so retry rather than
# concluding from one quiet run.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-430}
TRIES=${TRIES:-5}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
wait_halt() { local s=$SECONDS
    while [ $((SECONDS-s)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0; done; return 1; }
word() {
    local v=$(jt "dump-mem $1 1" 25 | grep -oE '= 0x[0-9a-fA-F]{8}' | head -1 | sed 's/= //')
    [ -n "$v" ] || { echo "?"; return; }
    python3 -c "n=int('$v',16); x=(n>>16)&0xFFFF; print(x-0x10000 if x>=0x8000 else x)"
}
sdstat() {  # -> "completions errors sticky"
    local raw=$(jt "vio-read scsi" 40 | grep -oE 'vio_scsi_sd = [0-9a-fA-F]+' | awk '{print $3}')
    [ -n "$raw" ] || { echo "? ? ?"; return; }
    python3 -c "
v=int('$raw',16)
g=lambda hi,lo:(v>>lo)&((1<<(hi-lo+1))-1)
print(g(43,28), g(27,20), g(3,3))"
}

for t in $(seq 1 "$TRIES"); do
    echo "=== attempt $t/$TRIES ==="
    for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
        jt "$c" >/dev/null; done
    jt "break-pc 0x40815E64" 25 | grep -E "slot=" | sed 's/^/  /'
    jt "break-pc 0x40815E6A" 25 | grep -E "slot=" | sed 's/^/  /'
    jt "vio-hard-reset" 30 >/dev/null

    PREVPC=""; PREVSD=""; PREVRE=""
    for i in $(seq 1 8); do
        wait_halt "$WAIT" || { echo "  no halt in ${WAIT}s (boot did not reach the dialog)"; break; }
        PC=$(jt "pc" 15 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
        RE=$(word 0x00000A60)
        read C E S <<<"$(sdstat)"
        printf "  %s  ResErr=%-5s  SD: completions=%-6s errors=%-3s sticky=%s\n" "$PC" "$RE" "$C" "$E" "$S"

        if [ "${PC,,}" = "0x40815e6a" ] && [ "${PREVPC,,}" = "0x40815e64" ]; then
            echo
            echo "  *** BEFORE -> AFTER pair captured ***"
            echo "    ResErr      : $PREVRE  ->  $RE"
            echo "    completions : $PREVSD  ->  $C"
            if [ "$PREVSD" = "$C" ]; then
                echo "    => NO disk read was attempted across _LoadResource."
                echo "       The failure is ABOVE the disk (Resource Manager / File"
                echo "       Manager), not in our SCSI/SD RTL."
            else
                echo "    => A disk read DID happen and the load still failed."
            fi
            exit 0
        fi
        PREVPC="$PC"; PREVSD="$C"; PREVRE="$RE"
        jt "halt-clear" 20 >/dev/null; jt "cont" 30 >/dev/null
    done
done
echo "=== never captured a BEFORE->AFTER pair ==="
exit 1
