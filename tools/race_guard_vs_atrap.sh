#!/bin/bash
# race_guard_vs_atrap.sh — decide whether break-pc on the Finder guard can fire
# at all, and if so capture the misbranch.
#
# The guard beqw (0x0075b31e on every boot that has reached the Finder so far)
# NECESSARILY executes before the delete, which is what calls
# _SetHandleSize(h,-24).  So arm BOTH and see which halts first:
#
#   halt PC == guard        -> break-pc works; capture D0/SR, step, classify
#   atrap latched instead   -> break-pc at that address CANNOT fire; the
#                              breakpoint approach is dead and we need another
#                              observation point (watchpoint on the store, or
#                              halt-on-exception)
#   neither                 -> that boot never reached the Finder (~2/3 of boots)
#
# Resuming after the atrap is useless: measured, the Finder is NEVER re-entered
# once the ROM Memory Manager wedges (guard not re-entered in a full 600 s).
# The >2752 SetHandleSize(-24) calls per boot are the ROM's own grow-zone retry
# loop, not the Finder looping.
#
# NOTE: do not kill this with `pkill -f <name>` — the pattern matches the
# caller's own command line and kills the calling shell (exit 143/144).

cd "$(dirname "$0")/.." || exit 1
GUARD=0x0075b31e
ATTEMPTS=${ATTEMPTS:-8}

for a in $(seq 1 "$ATTEMPTS"); do
    echo "=== attempt $a/$ATTEMPTS ==="
    JT_WAIT=20 tools/jt.sh "break-pc off" >/dev/null 2>&1
    JT_WAIT=20 tools/jt.sh "atrap 0 off"  >/dev/null 2>&1
    JT_WAIT=20 tools/jt.sh "break-pc $GUARD" >/dev/null 2>&1
    JT_WAIT=20 tools/jt.sh "atrap 0 0xA024 d0 0xFFFFFFE8" >/dev/null 2>&1
    JT_WAIT=30 tools/jt.sh "vio-hard-reset" >/dev/null 2>&1

    S=0; start=$SECONDS
    while [ $((SECONDS-start)) -lt 420 ]; do
        JT_WAIT=8 tools/jt.sh "halt-status" 2>&1 | grep -q 'effective=1' && { S=1; break; }
    done
    [ $S = 1 ] || { echo "  neither fired (boot never reached the Finder)"; continue; }

    PC=$(JT_WAIT=15 tools/jt.sh "pc" 2>&1 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
    AT=$(JT_WAIT=15 tools/jt.sh "atrap status" 2>&1 | grep -oE 'HIT: (none latched|slot=[0-9]+ opword=0x[0-9A-Fa-f]+ pc=0x[0-9A-Fa-f]+)')
    echo "  halted PC=$PC   atrap $AT"

    if [ "$(printf '%d' "$PC")" = "$(printf '%d' "$GUARD")" ]; then
        A=$(JT_WAIT=30 tools/jt.sh "live-arch" 2>&1)
        D0=$(echo "$A" | grep -oE '^> D0 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+')
        SR=$(echo "$A" | grep -oE '^> SR  = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+')
        Z=$(( (${SR:-0} >> 2) & 1 ))
        JT_WAIT=25 tools/jt.sh "step" >/dev/null 2>&1
        NPC=$(JT_WAIT=15 tools/jt.sh "pc" 2>&1 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
        FALL=$(printf "0x%08X" $(( $(printf '%d' "$GUARD") + 4 )))
        echo "  GUARD HIT: D0=$D0 SR=$SR Z=$Z -> PC=$NPC (fall-through=$FALL)"
        if [ "$(printf '%d' "$NPC")" = "$(printf '%d' "$FALL")" ] && [ "$D0" = "0x00000000" ]; then
            echo "  *** MISBRANCH CAPTURED ***"
            [ "$Z" = "1" ] && echo "      Z=1 -> flags CORRECT; BRANCH RESOLUTION is wrong" \
                           || echo "      Z=0 -> flags ALREADY WRONG upstream"
            exit 0
        fi
        echo "  (this iteration behaved correctly; continuing to next guard hit)"
        # Keep stepping through subsequent guard hits on this same boot.
        for k in $(seq 1 30); do
            JT_WAIT=30 tools/jt.sh "cont" >/dev/null 2>&1
            S2=0; s2=$SECONDS
            while [ $((SECONDS-s2)) -lt 120 ]; do
                JT_WAIT=8 tools/jt.sh "halt-status" 2>&1 | grep -q 'effective=1' && { S2=1; break; }
            done
            [ $S2 = 1 ] || { echo "  no further guard hits"; break; }
            PC=$(JT_WAIT=15 tools/jt.sh "pc" 2>&1 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
            [ "$(printf '%d' "$PC")" = "$(printf '%d' "$GUARD")" ] || { echo "  halted elsewhere: $PC"; break; }
            A=$(JT_WAIT=30 tools/jt.sh "live-arch" 2>&1)
            D0=$(echo "$A" | grep -oE '^> D0 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+')
            SR=$(echo "$A" | grep -oE '^> SR  = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+')
            Z=$(( (${SR:-0} >> 2) & 1 ))
            JT_WAIT=25 tools/jt.sh "step" >/dev/null 2>&1
            NPC=$(JT_WAIT=15 tools/jt.sh "pc" 2>&1 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
            echo "  hit $k: D0=$D0 SR=$SR Z=$Z -> $NPC"
            if [ "$(printf '%d' "$NPC")" = "$(printf '%d' "$FALL")" ] && [ "$D0" = "0x00000000" ]; then
                echo "  *** MISBRANCH CAPTURED ***"
                [ "$Z" = "1" ] && echo "      Z=1 -> flags CORRECT; BRANCH RESOLUTION is wrong" \
                               || echo "      Z=0 -> flags ALREADY WRONG upstream"
                exit 0
            fi
        done
        exit 3
    else
        echo "  *** break-pc did NOT fire at the guard, yet execution passed it ***"
        echo "      (the atrap proves the delete ran, and the guard precedes it)"
        exit 4
    fi
done
echo "=== no boot reached the Finder ==="
exit 1
