#!/bin/bash
# catch_guard_misbranch.sh — capture the 7.5.3 Finder guard branch in the act.
#
# THE INSTRUCTION UNDER TEST (Finder RemoveMenuCommandsMatching):
#   G-10: 4ead 02e2   jsr %a5@(738)        ; FindCommand -> D0 (0 = not found)
#   G-6 : 2d40 fffc   movel %d0,%fp@(-4)   ; store; sets N/Z, clears V/C
#   G-2 : 504f        addqw #8,%sp         ; ADDQ to An MUST NOT touch CCR
#   G   : 6700 xxxx   beqw <ret>           ; <-- THE GUARD
#   G+4 : 2f2e fffc   movel %fp@(-4),%sp@- ; <-- fall-through = the delete
#
# break-pc halts BEFORE the instruction executes, so at each halt D0 and SR are
# exactly what the beqw resolves against.  We record them, single-step, and see
# where PC lands.  Fall-through is always G+4; anything else is branch-taken.
#
#   D0==0 & Z==1 & lands G+4  -> BRANCH RESOLUTION is wrong (the bug)
#   D0==0 & Z==0              -> FLAGS were already wrong upstream
#   D0!=0 & lands G+4         -> correct, ordinary found-a-match iteration
#
# THREE THINGS THAT MAKE THIS HARDER THAN IT LOOKS, all learned the hard way:
#
#  1. The Finder's load address MOVES BETWEEN BOOTS.  Guard signatures have been
#     seen at 0x758ed2/0x759610/0x763d88 on one boot and 0x75abda/0x75b318 on
#     another.  So we arm ALL four break-pc slots with known candidates; a boot
#     whose layout matches none of them simply won't hit.
#  2. Only SOME boots reach the Finder's menu build at all — others wedge in
#     early ROM, in a wild branch, or in an unterminated list walk.  Hence the
#     outer reset loop.
#  3. jt.sh returns when the REPL's output goes quiet (~1-2 s), NOT after
#     JT_WAIT seconds.  A poll COUNT is therefore not a timeout; an earlier
#     version used 40 polls and gave a false "wedged elsewhere" after ~60 s
#     against a boot that needs 4-5 MINUTES.  We use a wall-clock deadline.
#
# Arm before the reset: continuing from the SetHandleSize(-24) trap is too late,
# because the first fatal call wedges the ROM Memory Manager's grow-zone retry
# loop and the Finder is never re-entered.  Verified live: the bp enable
# survives vio-hard-reset (a platform reset), not merely a CPU reset.

cd "$(dirname "$0")/.." || exit 1

CANDIDATES=${CANDIDATES:-"0x0075b31e 0x00759616 0x00758ed8 0x0075abe0"}
ATTEMPTS=${ATTEMPTS:-8}
BOOT_TIMEOUT=${BOOT_TIMEOUT:-420}     # seconds to let one boot reach the guard
MAXHITS=${MAXHITS:-40}

arm_all() {
    JT_WAIT=20 tools/jt.sh "break-pc off" >/dev/null 2>&1
    for a in $CANDIDATES; do
        JT_WAIT=15 tools/jt.sh "break-pc $a" 2>&1 | grep -oE 'slot=[0-9]+ target=0x[0-9A-Fa-f]+'
    done
}

wait_for_halt() {   # $1 = deadline seconds; echoes 1 on halt, 0 on timeout
    local start=$SECONDS
    while [ $((SECONDS - start)) -lt "$1" ]; do
        if JT_WAIT=8 tools/jt.sh "halt-status" 2>&1 | grep -q 'effective=1'; then
            echo 1; return
        fi
    done
    echo 0
}

for a in $(seq 1 "$ATTEMPTS"); do
    echo "=== attempt $a/$ATTEMPTS ==="
    JT_WAIT=20 tools/jt.sh "atrap 0 off" >/dev/null 2>&1
    arm_all
    JT_WAIT=30 tools/jt.sh "vio-hard-reset" >/dev/null 2>&1

    for h in $(seq 1 "$MAXHITS"); do
        if [ "$(wait_for_halt "$BOOT_TIMEOUT")" != "1" ]; then
            echo "  no guard hit within ${BOOT_TIMEOUT}s — boot wedged elsewhere or layout differs"
            break
        fi

        G=$(JT_WAIT=15 tools/jt.sh "pc" 2>&1 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
        A=$(JT_WAIT=30 tools/jt.sh "live-arch" 2>&1)
        D0=$(echo "$A" | grep -oE '^> D0 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+')
        SR=$(echo "$A" | grep -oE '^> SR  = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+')
        Z=$(( (${SR:-0} >> 2) & 1 ))

        JT_WAIT=25 tools/jt.sh "step" >/dev/null 2>&1
        NPC=$(JT_WAIT=15 tools/jt.sh "pc" 2>&1 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
        FALL=$(printf "0x%08X" $(( $(printf '%d' "$G") + 4 )) )

        printf "  hit %2d @ %s: D0=%s SR=%s Z=%d -> PC=%s" "$h" "$G" "$D0" "$SR" "$Z" "$NPC"
        if [ "$(printf '%d' "$NPC")" = "$(printf '%d' "$FALL")" ]; then
            if [ "$D0" = "0x00000000" ]; then
                echo "  *** MISBRANCH CAPTURED ***"
                echo "      D0==0 (not found) but the beq was NOT taken."
                if [ "$Z" = "1" ]; then
                    echo "      Z=1 -> flags were CORRECT; BRANCH RESOLUTION is wrong."
                else
                    echo "      Z=0 -> flags were ALREADY WRONG upstream"
                    echo "             (the flag-setting store, or ADDQ clobbering CCR)."
                fi
                echo ">>> board left halted just past the guard"
                exit 0
            fi
            echo "  (not taken, D0!=0 — correct)"
        else
            echo "  (taken — correct)"
        fi
        JT_WAIT=30 tools/jt.sh "cont" >/dev/null 2>&1
    done
done

echo "=== exhausted $ATTEMPTS attempts with no misbranch captured ==="
exit 1
