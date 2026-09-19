#!/usr/bin/env bash
# p141 — INTERLEAVED A/B boot test: preserved p133 artifact vs the MEASUREMENT
# bitstream (cpu040 HEAD: walker routing in, three never-boarded fixes, and the
# four live stall-state CSRs).
#
# This is tools/p138c_ab_boot.sh with ONE instrument added and nothing removed.
# p138c's own addition -- the RETIRED-MACRO counter (OFF_INST_LO/HI at
# 0x50901008 / 0x5090100C), sampled before and after the PC window -- stays
# exactly as it was, and remains THE discriminator: retire ADVANCING vs FROZEN,
# not the screen, because both arms show a mixed screen distribution. It is
# live, free-running and needs no halt, unlike `inst-count`.
#
# What p141 adds is the answer to "frozen ON WHAT". Four more live reads in the
# same window (0x5090101C..0x50901028) carry the D-cache quiesce terms
# INDIVIDUALLY, D-cache port ownership and the grant machine, the ExceptionUnit
# FSM one-hot, and both table-walker FSMs. They are decoded only when retire is
# FROZEN -- see the comment at that block for why decoding a running machine
# would be actively misleading.
#
# No ILA. The wedge is deterministic to the exact retire count, so the stalled
# state is static and a read settles it; and ENABLE_ILA reshapes placement
# design-wide, which is how p128 lost the wedge entirely. This bitstream keeps
# p133/p140's exact ENABLE_ILA=0 configuration so the A/B stays controlled.
#
# Written as a new script rather than editing p138c_ab_boot.sh: that script is
# shared and may be running for someone else.
#
# EVERYTHING ELSE IS DELIBERATELY IDENTICAL TO ab_boot.sh:
#   * interleaved, one cycle per arm per pair, so the bitstream is the only
#     thing that differs between arms within a session;
#   * SETTLE >= 180 (at 150 the board is often still booting and the ROM's ~15
#     LEGITIMATE SCSI-probe bus errors read as a terminal failure);
#   * ARMS NOTHING. Disarm is `halt-exc-mask raw <lane> 0` across all eight
#     lanes -- NOT `halt-exc-mask 0`, which is the SET form and arms vector 0 --
#     and `enables={ha=0 bp=0 exc=0 pcmis=0}` is ASSERTED, not assumed;
#   * plain `reset` (vio-hard-reset does not restart the CPU;
#     reset-and-break-pc hangs the REPL);
#   * live build_id is verified per cycle, reading twice (the first read after
#     load-bit can return 0x00000000).
#
# Usage:
#   PAIRS=6 SETTLE=200 OUTDIR=/path \
#   A_BIT=.../p133/fpga_top.bit A_LTX=.../p133/fpga_top.ltx A_NAME=p133 A_ID=0x14f3c599 \
#   B_BIT=.../p141/fpga_top.bit B_LTX=.../p141/fpga_top.ltx B_NAME=p141 B_ID=0x... \
#   tools/p141_ab_boot.sh
set -u
cd "$(dirname "$0")/.." || exit 1

PAIRS=${PAIRS:-6}
SETTLE=${SETTLE:-200}
OUTDIR=${OUTDIR:-/tmp/p141_abboot}
SNAP=${SNAP:-http://10.200.0.12:8080/snapshot.jpg}
mkdir -p "$OUTDIR"

A_BIT=${A_BIT:?}; A_LTX=${A_LTX:?}; A_NAME=${A_NAME:-A}; A_ID=${A_ID:?set A_ID}
B_BIT=${B_BIT:?}; B_LTX=${B_LTX:?}; B_NAME=${B_NAME:-B}; B_ID=${B_ID:?set B_ID}

jt () { JT_WAIT=${2:-15} JT_TAG="p141-ab" tools/jt.sh "$1" 2>&1; }
rd () { jt "r $1" 15 | grep -oiE '0x[0-9a-f]{8}' | tail -1; }

run_arm () {
    local name=$1 bit=$2 ltx=$3 wantid=$4 idx=$5
    echo
    echo "=== ARM $name  cycle $idx  ($(date -Is)) ==="

    jt "load-bit $bit $ltx" 240 | grep -E "programmed|ERROR" | sed 's/^/    /'
    sleep 5
    jt "build-id" 30 >/dev/null; sleep 2
    local live; live=$(jt "build-id" 30 | grep -oiE 'build_id = 0x[0-9a-f]+' | head -1 | grep -oiE '0x[0-9a-f]+')
    echo "    live build_id: ${live:-UNKNOWN}  (want $wantid)"
    if [ -z "${live:-}" ] || [ "${live,,}" != "${wantid,,}" ]; then
        echo "    ABORT cycle: live build_id does not match this arm"
        echo "    RESULT $name cycle $idx: ABORTED-WRONG-BITSTREAM"
        return 1
    fi

    for c in "break-pc off" "halt-clear" "watch 0 off" "watch 1 off" \
             "atrap 0 off" "atrap 1 off"; do
        jt "$c" 10 >/dev/null 2>&1
    done
    for lane in 0 1 2 3 4 5 6 7; do
        jt "halt-exc-mask raw $lane 0" 8 >/dev/null 2>&1
    done
    local st; st=$(jt "halt-status" 10)
    if printf '%s' "$st" | grep -q 'enables={ha=0 bp=0 exc=0 pcmis=0}'; then
        echo "    disarm verified: enables all zero"
    else
        echo "    WARNING: something is still armed:"
        printf '%s' "$st" | grep -oE 'enables=\{[^}]*\}' | sed 's/^/      /'
    fi

    jt "reset" 40 | grep -E "reset done" | sed 's/^/    /'
    sleep "$SETTLE"

    # ── liveness window: retired macros AND the exception ring, before/after ──
    local head0 head1 mac0 mac1 mach0 mach1
    local dc0 gr0 ex0 wk0 dc1 gr1 ex1 wk1
    head0=$(jt "exc-ring" 20 | grep -oE 'head=[0-9]+' | head -1 | cut -d= -f2)
    mach0=$(rd 0x5090100C); mac0=$(rd 0x50901008)
    # p141: the four live stall-state CSRs, sampled in the SAME window as the
    # retire counter and by the same `rd` path -- no halt, no arming. Sampled
    # twice for the same reason the retire counter is: a value that is identical
    # across the window is STATIC, which is what distinguishes a structural
    # stall from a machine that is merely slow.
    dc0=$(rd 0x5090101C); gr0=$(rd 0x50901020)
    ex0=$(rd 0x50901024); wk0=$(rd 0x50901028)

    local pcs=""
    for k in $(seq 1 8); do
        pcs="$pcs$(jt "halt-status" 10 | grep -oE 'pc_live=0x[0-9a-f]+' | head -1 | sed 's/pc_live=//')"$'\n'
    done

    head1=$(jt "exc-ring" 20 | grep -oE 'head=[0-9]+' | head -1 | cut -d= -f2)
    mach1=$(rd 0x5090100C); mac1=$(rd 0x50901008)
    dc1=$(rd 0x5090101C); gr1=$(rd 0x50901020)
    ex1=$(rd 0x50901024); wk1=$(rd 0x50901028)

    echo "    pc_live samples:"
    printf '%s' "$pcs" | grep -v '^$' | sort | uniq -c | sort -rn | sed 's/^/      /'
    echo "    retired_macros: ${mach0:-?}:${mac0:-?} -> ${mach1:-?}:${mac1:-?}"
    local retlive="FROZEN"
    if [ "${mac0:-x}" != "${mac1:-y}" ] || [ "${mach0:-x}" != "${mach1:-y}" ]; then
        retlive="ADVANCING"
    fi
    echo "    retire liveness: $retlive"

    # ── p141: the measurement ────────────────────────────────────────────────
    # Only decoded when retire is FROZEN. On a booting arm these words are a
    # meaningless instantaneous sample of a running machine; printing a decode
    # there would manufacture exactly the kind of confident-looking noise this
    # campaign has already been misled by twice.
    echo "    stall CSRs: dc=${dc0:-?}->${dc1:-?} grant=${gr0:-?}->${gr1:-?}" \
         "exc=${ex0:-?}->${ex1:-?} walk=${wk0:-?}->${wk1:-?}"
    if [ "$retlive" = FROZEN ] && [ -n "${dc0:-}" ] && [ -n "${dc1:-}" ]; then
        local static="CHANGING"
        if [ "$dc0" = "$dc1" ] && [ "$gr0" = "$gr1" ] && \
           [ "$ex0" = "$ex1" ] && [ "$wk0" = "$wk1" ]; then static="STATIC"; fi
        echo "    stall state across the window: $static"
        echo "    ---- decoded stall state ($name cycle $idx) ----"
        tools/p141_decode_stall.sh "$dc1" "$gr1" "$ex1" "$wk1" 2>&1 | sed 's/^/      /'
        echo "    ---- end decoded stall state ----"
    fi
    if [ -n "${head0:-}" ] && [ -n "${head1:-}" ] && [ "$head0" != "$head1" ]; then
        echo "    exc-ring: head $head0 -> $head1 (STILL TAKING EXCEPTIONS)"
    else
        echo "    exc-ring: head static at ${head0:-?}"
    fi

    curl -s -m 20 -o "$OUTDIR/${name}_c${idx}.jpg" "$SNAP" 2>/dev/null
    local smd5; smd5=$(md5sum "$OUTDIR/${name}_c${idx}.jpg" 2>/dev/null | cut -d' ' -f1)
    local sig="unknown"
    case "${smd5:0:8}" in
        2f5de0f4) sig="HAPPY-MAC" ;;
        eed68f0c) sig="SAD-MAC-0F/03" ;;
        feaec0e2) sig="SAD-MAC-0F/0A" ;;
        9086c68c) sig="P137-WEDGE-PARTIAL-ICON" ;;
        f8e8fd03) sig="FULLY-BLACK" ;;
    esac
    echo "    screen md5: ${smd5:-none}  ($sig)"

    echo "    exc-ring (top 6):"
    jt "exc-ring" 20 2>&1 | head -7 | sed 's/^/      /'

    local zero total buserr scsi mon rom wedge v
    zero=$(printf   '%s' "$pcs" | grep -v '^$' | grep -cE '^0x00000000$')
    total=$(printf  '%s' "$pcs" | grep -v '^$' | wc -l)
    buserr=$(printf '%s' "$pcs" | grep -cE '^0x0030001')
    scsi=$(printf   '%s' "$pcs" | grep -cE '^0x408(99|98)')
    mon=$(printf    '%s' "$pcs" | grep -cE '^0x4084a')
    wedge=$(printf  '%s' "$pcs" | grep -cE '^0x40806b68$')
    rom=$(printf    '%s' "$pcs" | grep -cE '^0x40')
    if   [ "$total" -eq 0 ];        then v="NO-SAMPLES"
    elif [ "$zero" -eq "$total" ];  then v="CPU-STOPPED"
    elif [ "$wedge" -gt 0 ] && [ "$retlive" = FROZEN ]; then v="P138B-WEDGE-0x40806B68"
    elif [ "$buserr" -gt 0 ];       then v="BUSERR-LIVELOCK"
    elif [ "$scsi" -gt 0 ];         then v="SCSI-INT-POLL-HANG"
    elif [ "$mon"  -gt 0 ];         then v="ROM-SERIAL-MONITOR"
    elif [ "$rom"  -gt 0 ];         then v="ELSEWHERE-IN-ROM"
    else v="RAM-EXEC-CANDIDATE-FINDER"
    fi
    echo "    RESULT $name cycle $idx: $v  retire=$retlive  screen=$sig"
}

echo "=== p141 ab_boot: $PAIRS pairs, settle ${SETTLE}s, outdir $OUTDIR ==="
echo "=== A=$A_NAME ($A_ID)  B=$B_NAME ($B_ID) ==="
for i in $(seq 1 "$PAIRS"); do
    run_arm "$A_NAME" "$A_BIT" "$A_LTX" "$A_ID" "$i"
    run_arm "$B_NAME" "$B_BIT" "$B_LTX" "$B_ID" "$i"
done
echo
echo "=== p141 ab_boot COMPLETE ==="
echo "=== SUMMARY ==="
