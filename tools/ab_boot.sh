#!/bin/bash
# ab_boot.sh — INTERLEAVED A/B boot comparison between two bitstreams.
#
# SETTLE TIME IS THE DOMINANT MEASUREMENT ARTIFACT — READ THIS FIRST
#
#   At SETTLE=150 this board produced 5/5 "bus-error livelock at 0x0030001x".
#   At SETTLE=180, with nothing else changed, it produced Happy Mac (the SCSI
#   INT-poll hang) instead.  The 150 s samples were almost certainly taken
#   MID-BOOT: the ROM legitimately provokes ~15 bus errors on its SCSI probe
#   path, and those runs' exception rings held 13-14 bus errors at
#   handler=0x00300000 -- i.e. normal boot progress, misread as a terminal
#   failure because the PC happened to be stable inside the probe loop across
#   the sampling window.
#
#   A hang and a boot-in-progress are NOT distinguishable from one PC sample.
#   Settle well past the ~2-2.5 min the ROM needs, and treat any bus-error
#   reading as suspect until the ring stops growing.
#
# WHY INTERLEAVED, AND NOT TWO SEPARATE RUNS
#
#   Even with settle fixed, a p137 sample taken tonight and a p133 table taken
#   on another day differ in more than the bitstream (board temperature, DDR and
#   disk state, settle, tooling).  Alternating the two bitstreams inside ONE
#   session with an identical cycle per arm is what makes the comparison paired,
#   so the build is the only thing that differs between the arms.
#
#   NOTE: an earlier revision of this header asserted that boot outcomes are
#   strongly autocorrelated within a session.  That claim is RETRACTED -- it was
#   inferred from the 5/5 run above, which the settle-time confound explains
#   without any need for autocorrelation.  Interleaving is still the right
#   design; the original justification for it was simply wrong.
#
# Both arms run an IDENTICAL cycle -- same load-bit, same reset, same settle,
# same sampling -- so nothing differs but the bitstream.
#
# ARMS NOTHING.  See boot_distribution.sh for why (the ROM legitimately provokes
# ~15 bus errors on the SCSI probe path; arming halts a healthy boot).
#
# Usage:
#   PAIRS=6 SETTLE=180 OUTDIR=/path \
#   A_BIT=/path/p133/fpga_top.bit A_LTX=/path/p133/fpga_top.ltx A_NAME=p133 \
#   B_BIT=/path/p137/fpga_top.bit B_LTX=/path/p137/fpga_top.ltx B_NAME=p137 \
#   tools/ab_boot.sh
set -u
cd "$(dirname "$0")/.." || exit 1

PAIRS=${PAIRS:-6}
SETTLE=${SETTLE:-240}
OUTDIR=${OUTDIR:-/tmp/abboot}
SNAP=${SNAP:-http://10.200.0.12:8080/snapshot.jpg}
mkdir -p "$OUTDIR"

jt () { JT_WAIT=${2:-15} tools/jt.sh "$1" 2>&1; }

run_arm () {
    local name=$1 bit=$2 ltx=$3 idx=$4
    echo
    echo "=== ARM $name  cycle $idx  ($(date -Is)) ==="

    jt "load-bit $bit $ltx" 200 | grep -E "programmed|ERROR|build_id" | sed 's/^/    /'
    sleep 5

    # Verify which bitstream is LIVE.  A first read after load-bit can return
    # 0x00000000 (read-before-CSR-reset race), so read twice and use the second.
    jt "build-id" 30 >/dev/null
    sleep 2
    local idline; idline=$(jt "build-id" 30)
    local live; live=$(printf '%s' "$idline" | grep -oiE 'build_id = 0x[0-9a-f]+' | head -1 | grep -oiE '0x[0-9a-f]+')
    echo "    live build_id: ${live:-UNKNOWN}"
    if [ -z "${live:-}" ] || [ "${live^^}" = "0X00000000" ]; then
        echo "    ABORT cycle: could not establish a live build_id"
        return 1
    fi

    # Disarm.  NOT `halt-exc-mask 0` -- that is the SET form and arms vector 0.
    for c in "break-pc off" "halt-clear" "watch 0 off" "watch 1 off" \
             "atrap 0 off" "atrap 1 off"; do
        jt "$c" 10 >/dev/null 2>&1
    done
    for lane in 0 1 2 3 4 5 6 7; do
        jt "halt-exc-mask raw $lane 0" 8 >/dev/null 2>&1
    done
    local st; st=$(jt "halt-status" 10)
    printf '%s' "$st" | grep -q 'enables={ha=0 bp=0 exc=0 pcmis=0}' \
        || { echo "    WARNING: something is still armed:"; \
             printf '%s' "$st" | grep -oE 'enables=\{[^}]*\}' | sed 's/^/      /'; }

    jt "reset" 40 | grep -E "reset done" | sed 's/^/    /'
    sleep "$SETTLE"

    # LIVENESS, not just position.  A terminal hang and a boot still in progress
    # are indistinguishable from PC samples alone -- that is exactly how a run of
    # mid-boot samples got read as 5/5 bus-error livelocks earlier today.  Read
    # the exception-ring head before and after the PC sampling window: if it
    # advanced, the machine is still taking exceptions and is NOT wedged.
    local head0 head1
    head0=$(jt "exc-ring" 20 | grep -oE 'head=[0-9]+' | head -1 | cut -d= -f2)

    local pcs=""
    for k in $(seq 1 8); do
        pcs="$pcs$(jt "halt-status" 10 | grep -oE 'pc_live=0x[0-9a-f]+' | head -1 | sed 's/pc_live=//')"$'\n'
    done
    echo "    pc_live samples:"
    printf '%s' "$pcs" | grep -v '^$' | sort | uniq -c | sort -rn | sed 's/^/      /'

    head1=$(jt "exc-ring" 20 | grep -oE 'head=[0-9]+' | head -1 | cut -d= -f2)
    if [ -n "${head0:-}" ] && [ -n "${head1:-}" ] && [ "$head0" != "$head1" ]; then
        echo "    liveness: exc-ring head $head0 -> $head1 (STILL TAKING EXCEPTIONS)"
    else
        echo "    liveness: exc-ring head static at ${head0:-?} (no new exceptions in the window)"
    fi

    curl -s -m 20 -o "$OUTDIR/${name}_c${idx}.jpg" "$SNAP" 2>/dev/null
    echo "    screen md5: $(md5sum "$OUTDIR/${name}_c${idx}.jpg" 2>/dev/null | cut -d' ' -f1)"

    echo "    exc-ring (top 6):"
    jt "exc-ring" 20 2>&1 | head -7 | sed 's/^/      /'

    local zero total buserr scsi mon rom v
    zero=$(printf  '%s' "$pcs" | grep -v '^$' | grep -cE '^0x00000000$')
    total=$(printf '%s' "$pcs" | grep -v '^$' | wc -l)
    buserr=$(printf '%s' "$pcs" | grep -cE '^0x0030001')
    scsi=$(printf   '%s' "$pcs" | grep -cE '^0x408(99|98)')
    mon=$(printf    '%s' "$pcs" | grep -cE '^0x4084a')
    rom=$(printf    '%s' "$pcs" | grep -cE '^0x40')
    if   [ "$total" -eq 0 ];        then v="NO-SAMPLES"
    elif [ "$zero" -eq "$total" ];  then v="CPU-STOPPED"
    elif [ "$buserr" -gt 0 ];       then v="BUSERR-LIVELOCK"
    elif [ "$scsi" -gt 0 ];         then v="SCSI-INT-POLL-HANG"
    elif [ "$mon"  -gt 0 ];         then v="ROM-SERIAL-MONITOR"
    elif [ "$rom"  -gt 0 ];         then v="ELSEWHERE-IN-ROM"
    else v="RAM-EXEC-CANDIDATE-FINDER"
    fi
    echo "    RESULT $name cycle $idx: $v  (live=$live)"
}

echo "=== ab_boot: $PAIRS pairs, settle ${SETTLE}s, outdir $OUTDIR ==="
echo "=== A=$A_NAME  B=$B_NAME ==="
for i in $(seq 1 "$PAIRS"); do
    run_arm "$A_NAME" "$A_BIT" "$A_LTX" "$i"
    run_arm "$B_NAME" "$B_BIT" "$B_LTX" "$i"
done
echo
echo "=== ab_boot COMPLETE ==="
