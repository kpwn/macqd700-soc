#!/bin/bash
# csc21_race_trial.sh -- one cold-boot trial of the $0D24/csCode-21
# hardware-presence-probe race vs. permanent-boot-block correlation
# experiment (docs/BUG_calibration_word_misplaced_0d00.md Part 44/64/65).
#
# Uses csc21_send.sh (marker-delimited, race-free) instead of the ad hoc,
# scratchpad-only sender Part 64 built and found buggy twice. Classifies
# the probe HIT/MISS directly off `reset-and-break-pc`'s own response text
# (no extra decoupled `halt-status` round trip needed -- csc21_send.sh's
# per-call boundary is now unambiguous, so the response IS trustworthy),
# keyed on `pc_live=0x40800bf0` + `effective=1` (case-insensitive, robust
# to the `hit=` field resetting to 0 after `break-pc disabled`).
#
# Usage: csc21_race_trial.sh <trial_num> <results_csv> <bit> <ltx>
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEND="$HERE/csc21_send.sh"
CLASSIFY="$HERE/csc21_race_classify.py"
cd "$HERE/.." || exit 1

N="$1"
CSV="$2"
BIT="${3:-build/vivado_ras_ila_combined/fpga_top.bit}"
LTX="${4:-build/vivado_ras_ila_combined/fpga_top.ltx}"
RAWDIR="${CSC21_RAWDIR:-/tmp/csc21_raw}"
mkdir -p "$RAWDIR"
RAW="$RAWDIR/trial_${N}.log"

log() { echo "$1" >> "$RAW"; }

log "=== TRIAL $N $(date -u +%FT%TZ) ==="

LB=$("$SEND" "load-bit $BIT $LTX" 90)
log "$LB"
if echo "$LB" | grep -q "CSC21_SEND_TIMEOUT"; then
    echo "$N,SEND_TIMEOUT,load-bit,,,,,," >> "$CSV"
    echo "TRIAL $N ABORT (load-bit send timed out)"
    exit 1
fi

RB=$("$SEND" "reset-and-break-pc 0x40800bf0 0 d90000" 130)
log "$RB"
if echo "$RB" | grep -q "CSC21_SEND_TIMEOUT"; then
    echo "$N,SEND_TIMEOUT,reset-and-break-pc,,,,,," >> "$CSV"
    echo "TRIAL $N ABORT (reset-and-break-pc send timed out)"
    exit 1
fi

HIT=0
while IFS= read -r line; do
    if echo "$line" | grep -qi "pc_live=0x40800bf0" && echo "$line" | grep -q "effective=1"; then
        HIT=1
        break
    fi
done <<< "$RB"

if [ "$HIT" != "1" ]; then
    # MISS: never reached the probe within the wait window. Record the
    # resting/live PC cheaply (non-intrusive, no breakpoint held) and move on.
    BPOFF=$("$SEND" "break-pc off" 30)
    log "$BPOFF"
    PCV=$("$SEND" "pc" 20)
    log "miss-pc: $PCV"
    RESTPC=$(echo "$PCV" | grep -oE "0x[0-9A-Fa-f]+" | tail -1)
    echo "$N,MISS,,,,,,${RESTPC:-},single" >> "$CSV"
    echo "TRIAL $N HIT=0"
    exit 0
fi

# HIT: classify the $0D24/csCode-21 race outcome BEFORE releasing.
D24PTR=$("$SEND" "coherent-dump 0xd24 1" 30)
log "$D24PTR"
PTR=$(echo "$D24PTR" | grep -oE "= 0x[0-9A-Fa-f]+" | tail -1 | grep -oE "0x[0-9A-Fa-f]+")

RACE_VERDICT="UNKNOWN"
RACE_DETAIL=""
if [ -n "${PTR:-}" ]; then
    DUMP=$("$SEND" "coherent-dump $PTR 150" 45)
    log "$DUMP"
    CLASS_OUT=$(printf '%s\n' "$DUMP" | python3 "$CLASSIFY" race)
    log "$CLASS_OUT"
    RACE_VERDICT=$(echo "$CLASS_OUT" | grep -oE "RACE_VERDICT=\S+" | cut -d= -f2)
    RACE_DETAIL=$(echo "$CLASS_OUT" | grep "^DETAIL:" | sed 's/^DETAIL: //')
fi

# Release and observe: does this SAME boot attempt escape further or park
# permanently? Non-intrusive pc polling.
BPOFF=$("$SEND" "break-pc off" 30)
log "$BPOFF"
HREL=$("$SEND" "halt-release" 30)
log "$HREL"
log "--- post-release pc polling ---"
POLLTEXT=""
for i in $(seq 1 20); do
    PCV=$("$SEND" "pc" 20)
    log "poll$i: $PCV"
    POLLTEXT="$POLLTEXT
poll$i: $PCV"
    sleep 5
done

REST_OUT=$(printf '%s\n' "$POLLTEXT" | python3 "$CLASSIFY" resting)
log "$REST_OUT"
REST_VERDICT=$(echo "$REST_OUT" | grep -oE "RESTING_VERDICT=\S+" | cut -d= -f2)

D24PTRVAL=$(echo "$D24PTR" | grep -oE "= 0x[0-9A-Fa-f]+" | tail -1 | grep -oE "0x[0-9A-Fa-f]+")
# CSV: trial,hit,race_verdict,d24_ptr,race_detail,resting_verdict,,resting_pc,mode
echo "$N,HIT,$RACE_VERDICT,${D24PTRVAL:-},\"$(echo "$RACE_DETAIL" | tr ',' ';')\",$REST_VERDICT,,,batch" >> "$CSV"
echo "TRIAL $N HIT=1 RACE=$RACE_VERDICT REST=$REST_VERDICT"
