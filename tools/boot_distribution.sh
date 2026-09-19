#!/bin/bash
# boot_distribution.sh — classify N cold boots by where the machine ends up.
#
# WHY NOT tools/boot_outcome.sh: that script resets with `vio-hard-reset`, which
# does NOT restart the CPU on this board.  Plain `reset` does (~15 s, 20/20).
# `reset-and-break-pc` HANGS THE REPL and needs a reprogram — never used here.
#
# ARMS NOTHING.  Vectors 2, 3, 4, 10 and 11 are deliberately left disarmed: the
# ROM legitimately provokes ~15 bus errors on the SCSI probe path, line-F is FPSP
# dispatch and A-line is Toolbox.  Arming them halts a HEALTHY boot and has
# produced a false failure twice in this campaign.
#
# `exc-ring` is the trustworthy liveness instrument; `exc_count` and `inst-count`
# both lie (inst-count is halt-captured, not live).
#
# Usage:  N=12 TAG=p137 OUTDIR=/path tools/boot_distribution.sh
set -u
cd "$(dirname "$0")/.." || exit 1

N=${N:-12}
SETTLE=${SETTLE:-150}
TAG=${TAG:-run}
OUTDIR=${OUTDIR:-/tmp/bootdist-$TAG}
SNAP=${SNAP:-http://10.200.0.12:8080/snapshot.jpg}
mkdir -p "$OUTDIR"

jt () { JT_WAIT=${2:-15} tools/jt.sh "$1" 2>&1; }

echo "=== boot_distribution TAG=$TAG N=$N settle=${SETTLE}s outdir=$OUTDIR ==="
echo "=== live build_id ==="
jt "build-id" 20

for i in $(seq 1 "$N"); do
    echo
    echo "=== boot $i/$N  ($(date -Is)) ==="

    # Disarm everything.  A leftover arm from an earlier experiment is the
    # single easiest way to manufacture a fake failure.
    #
    # NOT `halt-exc-mask 0` -- that is the SET form (`halt-exc-mask <vec>`), so
    # it ARMS exception vector 0 and the enable-sync then turns halt_exc_enable
    # on, leaving the status line reading exc=1.  Measured here 2026-09-04: it
    # armed a vector on every boot of a run whose whole purpose was to arm
    # nothing.  Clear all eight 32-bit lanes explicitly instead; the REPL
    # disables the enable once every lane reads zero.
    for c in "break-pc off" "halt-clear" "watch 0 off" "watch 1 off" \
             "atrap 0 off" "atrap 1 off"; do
        jt "$c" 10 >/dev/null 2>&1
    done
    for lane in 0 1 2 3 4 5 6 7; do
        jt "halt-exc-mask raw $lane 0" 10 >/dev/null 2>&1
    done

    # Prove the disarm took, rather than assuming it.  If exception halting is
    # still enabled the ROM's ~15 legitimate SCSI-probe bus errors will halt a
    # perfectly healthy boot and the whole run is worthless.
    ST=$(jt "halt-status" 10)
    if ! printf '%s' "$ST" | grep -q 'enables={ha=0 bp=0 exc=0 pcmis=0}'; then
        echo "  WARNING: boot $i starts with something still armed:"
        printf '%s' "$ST" | grep -oE 'enables=\{[^}]*\}' | sed 's/^/    /'
    fi

    jt "reset" 40 | sed 's/^/    reset: /' | head -3

    sleep "$SETTLE"

    # Sample pc_live repeatedly: distinguishes "parked in a tight loop" from
    # "running normally somewhere".
    PCS=""
    for k in $(seq 1 8); do
        p=$(jt "halt-status" 10 | grep -oE 'pc_live=0x[0-9a-f]+' | head -1 | sed 's/pc_live=//')
        PCS="$PCS$p"$'\n'
    done
    echo "  pc_live samples:"
    printf '%s' "$PCS" | grep -v '^$' | sort | uniq -c | sort -rn | sed 's/^/    /'

    UNIQ=$(printf '%s' "$PCS" | grep -v '^$' | sort -u | tr '\n' ' ')

    curl -s -m 20 -o "$OUTDIR/boot_${i}.jpg" "$SNAP" 2>/dev/null
    MD5=$(md5sum "$OUTDIR/boot_${i}.jpg" 2>/dev/null | cut -d' ' -f1)
    echo "  screen: $OUTDIR/boot_${i}.jpg md5=$MD5"

    echo "  exc-ring:"
    jt "exc-ring" 20 2>&1 | head -14 | sed 's/^/    /'

    # Classify from the pc_live distribution.
    # A STOPPED cpu reports pc_live=0x00000000, which is not a ROM address.  An
    # earlier version of this classifier had "no ROM sample" as its last branch
    # and therefore reported a DEAD MACHINE as "candidate: reached the
    # System/Finder".  Baseline boot 2 hit exactly that: pc_live=0 on all eight
    # samples, an exception ring full of bus errors, and a corrupted icon on a
    # black screen.  Test pc_live=0 FIRST, and only ever claim a successful boot
    # from positive evidence (genuine RAM execution at a non-zero PC).
    ZERO=$(printf '%s' "$PCS"   | grep -v '^$' | grep -cE '^0x00000000$')
    TOTAL=$(printf '%s' "$PCS"  | grep -v '^$' | wc -l)
    SCSI=$(printf '%s' "$PCS"   | grep -cE '^0x408(99|98)')
    BUSERR=$(printf '%s' "$PCS" | grep -cE '^0x0030001')
    MON=$(printf '%s' "$PCS"    | grep -cE '^0x4084a')
    ROM=$(printf '%s' "$PCS"    | grep -cE '^0x40')
    if   [ "$TOTAL" -eq 0 ];                       then V="NO SAMPLES (could not read pc_live)"
    elif [ "$ZERO"   -eq "$TOTAL" ];               then V="CPU STOPPED (pc_live=0 every sample) -- NOT a successful boot"
    elif [ "$BUSERR" -gt 0 ]; then V="BUS-ERROR LIVELOCK 0x0030001x (Sad Mac 0F/02)"
    elif [ "$SCSI"   -gt 0 ]; then V="SCSI-INT-POLL-HANG (Happy Mac path)"
    elif [ "$MON"    -gt 0 ]; then V="ROM SERIAL MONITOR (Sad Mac 0F/0A or 0F/03)"
    elif [ "$ROM"    -gt 0 ]; then V="ELSEWHERE IN ROM"
    else V="RAM EXECUTION at non-zero PC -- candidate: reached the System/Finder"
    fi
    echo "  VERDICT $i: $V"
    echo "  UNIQ_PCS $i: $UNIQ"
done

echo
echo "=== boot_distribution TAG=$TAG COMPLETE ==="
