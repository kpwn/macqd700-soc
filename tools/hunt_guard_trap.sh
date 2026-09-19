#!/bin/bash
# hunt_guard_trap.sh — cycle platform resets until a boot reaches the Finder's
# menu build and the SetHandleSize(-24) A-trap fires.
#
# WHY: the 7.5.3 failure mode varies boot to boot (observed: ROM Memory Manager
# grow-zone livelock, wild branch into data, early-ROM device-poll stall, and an
# unterminated linked-list walk).  Only some boots reach the Finder's menu
# construction, which is where the guarded delete misbranches.  So we retry.
#
# On a hit the CPU halts BEFORE the trap's side effects, which is the state we
# want: from there the caller can walk the stack, locate the Finder's guard for
# THAT boot (its load address moves every boot), and breakpoint it.
#
# Leaves the board halted on success.  Prints the attempt log either way.

cd "$(dirname "$0")/.." || exit 1

ATTEMPTS=${ATTEMPTS:-6}
POLLS=${POLLS:-30}
POLL_WAIT=${POLL_WAIT:-12}

for a in $(seq 1 "$ATTEMPTS"); do
    echo "=== attempt $a/$ATTEMPTS: reset ==="
    JT_WAIT=30 tools/jt.sh "vio-hard-reset" >/dev/null 2>&1
    # Re-arm defensively.  dbg-caps says the debug reset domain preserves these
    # across a CPU reset, but re-arming is free and removes the assumption.
    JT_WAIT=20 tools/jt.sh "atrap 0 0xA024 d0 0xFFFFFFE8" >/dev/null 2>&1

    for p in $(seq 1 "$POLLS"); do
        S=$(JT_WAIT=$POLL_WAIT tools/jt.sh "halt-status" 2>&1 \
            | grep -oE 'effective=[0-9]|pc_live=0x[0-9a-f]+|exc_count=0x[0-9a-f]+' \
            | tr '\n' ' ')
        echo "  a$a p$p  $S"
        case "$S" in
            *"effective=1"*)
                echo ">>> HALTED on attempt $a poll $p"
                JT_WAIT=25 tools/jt.sh "atrap status" 2>&1 | grep -E 'HIT|atrap0'
                echo ">>> board left halted for forensics"
                exit 0
                ;;
        esac
    done
    echo "  attempt $a: no trap; boot wedged elsewhere, retrying"
done

echo ">>> exhausted $ATTEMPTS attempts with no SetHandleSize(-24) trap"
exit 1
