#!/bin/bash
# sd_disk_diff.sh — diff the SD-backed boot volume against a golden image,
# and optionally restore ONLY the sectors that differ.
#
# WHY THIS EXISTS
#   The 7.5.3 heap bug (task #248) can corrupt the boot volume: Mac OS disk
#   cache and driver I/O buffers live in the System heap, so a corrupt heap
#   means the OS flushes garbage buffers to disk reporting success.  After
#   any session parked in that hang the volume is suspect.  Re-imaging the
#   whole 46.6 MB used extent takes ~156 s and DESTROYS the evidence of what
#   changed.  This tool reports the damage first, then repairs surgically.
#
# REQUIRES the PROVISIONING bitstream — sd-verify / sd-write-fast are not in
# the main one (they fail with "bulk writer not present (ident=0x408026F6)"):
#   load-bit build/sd_provision/sd_provision_top.bit
#
# USAGE
#   tools/sd_disk_diff.sh diff    <image> [start_blk] [count]
#   tools/sd_disk_diff.sh restore  <image> <start_blk> <count>
#   tools/sd_disk_diff.sh repair   <image> [start_blk] [count]
#   tools/sd_disk_diff.sh localize <image> <batch_start_blk> [nblocks]
#
#   start_blk/count are in DISK BLOCKS (relative to the disk image), not raw
#   SD LBAs.  The tool adds RAW_BASE_LBA for you.
#
# NOTE RAW_BASE_LBA is a COMPILE-TIME parameter (rtl/board/sd_ctrl.v), so it
# must match the bitstream in use.  Do not "fix" a total mismatch by nudging
# this — a 100% mismatch means wrong mapping, not a corrupt card.
set -u
RAW_BASE_LBA=${RAW_BASE_LBA:-8192}
JT_IN=${JT_IN:-/tmp/jtag_in}
JT_OUT=${JT_OUT:-/tmp/jtag_out}

die () { echo "ERROR: $*" >&2; exit 1; }

# ── REPL command dispatch ────────────────────────────────────────────────────
#
# WHY THIS IS SENTINEL-BASED AND NOT TIMING-BASED  (2026-08-08 post-mortem)
#
#   The previous jt() decided a command had finished when $JT_OUT stopped
#   growing for 5 x 0.2 s = 1.0 s.  `sd-verify` prints a progress line every
#   16 batches = 4096 sectors = 2048 KiB, and MEASURED throughput is
#   1649-1816 KiB/s, i.e. a gap of 1.13-1.24 s between consecutive lines.
#   The detector threshold sat ~0.1 s BELOW the normal inter-line gap, so a
#   26 s scan routinely "completed" after its first line.  Three separate
#   phantom findings came out of that one heuristic:
#
#     * "the diff does not enumerate all damaged batches in one pass"
#       (round 1 reports 1 batch, round 2 reports 2 more) — the capture was
#       truncated at a random point, not the scan.
#     * "a single restore pass silently does not stick" — the restore never
#       ran.  jt() returned while the previous 26 s verify was still
#       streaming, the script exited, its EXIT trap deleted the temp slice,
#       and the REPL then dequeued the command against a file that no longer
#       existed.  The proof is in /tmp/jtag_out:
#           > ERROR sd-write-fast: file not found: /tmp/sdrestore.hFI2pK.bin
#           > ERROR sd-verify:     file not found: /tmp/sdrestore.hFI2pK.bin
#       Those errors landed in the NEXT command's capture window, so the
#       operator never saw them.
#     * `repair` livelocking: every round re-reported the same batches
#       because every round's write was eaten the same way.
#
#   Writing to the FIFO does NOT block while the REPL is busy (the kernel
#   pipe buffer accepts it), so "the command was accepted" says nothing about
#   "the command has run".  The only reliable completion signal is the REPL's
#   own `> READY` prompt.  Count those, never wall-clock quiet.
#
#   Do not reintroduce an output-quiet heuristic here, and do not "tune" the
#   threshold — any threshold is a race against a command whose output is
#   bursty by nature.

jt_ready_count () { grep -ac '^> READY$' "$JT_OUT" 2>/dev/null || echo 0; }

# Wait until the REPL is idle (nothing in flight) before issuing anything.
jt_sync () {
    local wait_s=${1:-1800} r0 r1 n0 n1 i
    for ((i=0; i<wait_s*2; i++)); do
        n0=$(wc -l < "$JT_OUT" 2>/dev/null || echo 0)
        sleep 0.5
        n1=$(wc -l < "$JT_OUT" 2>/dev/null || echo 0)
        # Idle == file stopped growing AND the last line is the prompt.
        # NOT `tail -n1 == "> READY"`: Vivado appends ASYNCHRONOUS lines after
        # the prompt -- "INFO: [Labtools 27-3143] Calibration status change
        # detected, refreshing MIG_1" arrives on its own after any load-bit --
        # so that predicate goes permanently false on a HEALTHY repl and this
        # spins the full timeout. Measured 2026-08-08; it cost a board run.
        # Idle = output stopped moving AND a prompt has been printed.
        if [ "$n0" -eq "$n1" ] && [ "$(jt_ready_count)" -gt 0 ]; then
            return 0
        fi
    done
    die "REPL never went idle after ${wait_s}s — is a previous command wedged?"
}

# Send one REPL command; echo only the output THAT COMMAND produced.
jt () {
    local wait_s=${JT_WAIT:-60} before ready0 i
    jt_sync
    before=$(wc -l < "$JT_OUT" 2>/dev/null || echo 0)
    ready0=$(jt_ready_count)
    timeout 5 bash -c "echo \"\$1\" > $JT_IN" _ "$1" || die "REPL not accepting input (dead?)"
    for ((i=0; i<wait_s*5; i++)); do
        sleep 0.2
        [ "$(jt_ready_count)" -gt "$ready0" ] && {
            tail -n +$((before+1)) "$JT_OUT"
            return 0
        }
    done
    die "REPL did not return to the prompt within ${wait_s}s on: $1"
}

# jt + hard failure if the REPL reported an error for that command.
jt_checked () {
    local out
    out=$(jt "$1") || return 1
    printf '%s\n' "$out"
    if printf '%s\n' "$out" | grep -aq '^> ERROR'; then
        die "REPL reported an error for: $1"
    fi
    return 0
}

# Match only an actual per-range mismatch record.  The completion summary
# always contains "MISMATCHING batches", including the clean case
# "0 MISMATCHING batches", so a loose grep for MISMATCH creates false damage.
sd_verify_has_mismatch () {
    grep -aq '^> sd-verify: MISMATCH lba='
}

pgm_check () {
    # A total mismatch usually means the provisioning bitstream isn't loaded.
    jt "vhdd-status" >/dev/null 2>&1 || true
}

# Sourcing hook for tools/tests/test_jt_dispatch.sh — lets the REPL dispatch
# functions above be unit-tested against a fake REPL without a board.  Keep
# this immediately before argument parsing so the test gets the real code,
# not a copy that can drift out of sync with it.
#
# ORDER IS LOAD-BEARING: this MUST come before the lease block below.  The
# unit test sources this file; if the lease were taken first, running the
# test would acquire the real JTAG cable lease (and block for up to an hour
# behind a real board experiment) to test a fake REPL in a temp directory.
if [ "${JT_LIB_ONLY:-0}" = "1" ]; then return 0 2>/dev/null || exit 0; fi

# ---- MANDATORY JTAG LEASE -------------------------------------------------
# Any REPL traffic must be serialised. If an ancestor already holds the lease
# (JTAG_LEASE_HELD set) we inherit it; otherwise we take one for ourselves and
# release on exit. Without this, an agent that runs this script directly
# bypasses the lease entirely -- which is exactly what happened 2026-08-08:
# two concurrent instances deleted each other's temp files mid-restore
# ("ERROR sd-write-fast: file not found: /tmp/sdrep.XXXX.bin").
LEASE="$(dirname "$0")/jtag_lease.sh"
if [ -z "${JTAG_LEASE_HELD:-}" ] && [ -x "$LEASE" ]; then
    "$LEASE" acquire "sd_disk_diff.$$" 1800 3600 || { echo "FATAL: no JTAG lease"; exit 2; }
    export JTAG_LEASE_HELD="sd_disk_diff.$$"
    trap '"$LEASE" release "sd_disk_diff.$$" >/dev/null 2>&1' EXIT
fi

cmd=${1:-}; img=${2:-}
[ -n "$cmd" ] && [ -n "$img" ] || die "usage: $0 {diff|restore} <image> [start_blk] [count]"
[ -r "$img" ] || die "cannot read image: $img"
img_blocks=$(( $(stat -c %s "$img") / 512 ))

case "$cmd" in
diff)
    start=${3:-0}; count=${4:-$((img_blocks-start))}
    echo "== diff $img blocks $start..$((start+count-1)) (SD LBA $((RAW_BASE_LBA+start))+) =="
    out=$(JT_WAIT=1800 jt "sd-verify $((RAW_BASE_LBA+start)) $img $count")
    printf '%s\n' "$out" | grep -aq 'sd-verify done:' || \
        die "sd-verify produced no 'done' summary — capture is truncated, refusing to guess"
    echo "$out" | grep -aE 'MISMATCH|done' | sed 's/^> /  /'
    # GRANULARITY WARNING.  sd-verify checksums in 256-SECTOR BATCHES, so a
    # single changed byte flags a whole 256-block range and every batch base
    # is a multiple of 256 BY CONSTRUCTION.  "MISMATCH lba=8192.." means
    # "something in disk blocks 0..255 differs" — it does NOT mean block 0
    # changed, and the alignment of the bases carries no information.
    # Reading structure into these numbers has already produced one retracted
    # conclusion (2026-08-08: "a failed boot rewrites disk block 0"; MAME
    # shows the real write is the HFS Master Directory Block at block 98).
    echo "  NOTE: 256-block batch granularity — a base of N means 'something in"
    echo "        blocks N..N+255 differs', NOT that block N itself changed."
    # Translate raw-LBA mismatch ranges back into disk blocks for the operator.
    echo "$out" | grep -aoE 'MISMATCH lba=[0-9]+\.\.[0-9]+' | while read -r m; do
        a=${m#MISMATCH lba=}; lo=${a%%..*}; hi=${a##*..}
        echo "  -> damaged disk blocks $((lo-RAW_BASE_LBA))..$((hi-RAW_BASE_LBA))  (restore: $0 restore $img $((lo-RAW_BASE_LBA)) $((hi-lo+1)))"
    done
    ;;
localize)
    # localize <image> <batch_start_blk> [nblocks=256]
    #
    # WHY THIS EXISTS.  `diff` answers at 256-block granularity, which cannot
    # tell "the OS updated the HFS Master Directory Block" (1 block, normal)
    # from "something scribbled over 200 blocks" (corruption).  Both show up
    # as one MISMATCH line.  Every conclusion about WHICH blocks changed needs
    # this subcommand, not `diff`.
    #
    # THE TRAP THIS EXISTS TO AVOID.  `sd-verify <lba> <file> <n>` compares
    # SD[lba..] against <file> FROM OFFSET 0 — it does NOT seek into the file.
    # Probing a non-zero offset with the whole image therefore ALWAYS reports
    # a mismatch, and the giveaway is that every probe prints the SAME
    # `image=` checksum (block 0's).  A 2026-08-08 session did exactly this
    # across blocks 16..231 and read the uniform `image=0x0F647AAD` results as
    # "200 damaged blocks"; they were an artefact.  This subcommand extracts a
    # correctly-offset slice for every probe, so the comparison is real.
    #
    # Two-level scan: 16-block windows, then per-block inside a dirty window.
    start=${3:?batch_start_blk}; nb=${4:-256}
    echo "== localizing changed blocks in disk blocks $start..$((start+nb-1)) =="
    win=16
    found=""
    for ((w=0; w<nb; w+=win)); do
        b=$((start+w))
        [ "$b" -ge "$img_blocks" ] && break
        tmp=$(mktemp /tmp/sdloc.XXXXXX.bin) || die "mktemp"
        dd if="$img" of="$tmp" bs=512 skip="$b" count="$win" status=none
        out=$(JT_WAIT=120 jt "sd-verify $((RAW_BASE_LBA+b)) $tmp $win")
        rm -f "$tmp"
        printf '%s\n' "$out" | grep -aq 'sd-verify done:' || \
            die "sd-verify produced no 'done' summary at blk $b — truncated capture"
        printf '%s\n' "$out" | sd_verify_has_mismatch || continue
        # Dirty window — narrow to single blocks.
        for ((i=0; i<win; i++)); do
            bb=$((b+i))
            [ "$bb" -ge "$img_blocks" ] && break
            tmp=$(mktemp /tmp/sdloc.XXXXXX.bin) || die "mktemp"
            dd if="$img" of="$tmp" bs=512 skip="$bb" count=1 status=none
            o1=$(JT_WAIT=120 jt "sd-verify $((RAW_BASE_LBA+bb)) $tmp 1")
            rm -f "$tmp"
            printf '%s\n' "$o1" | grep -aq 'sd-verify done:' || \
                die "sd-verify produced no 'done' summary at blk $bb — truncated capture"
            if printf '%s\n' "$o1" | sd_verify_has_mismatch; then
                echo "  CHANGED disk block $bb  (SD LBA $((RAW_BASE_LBA+bb)))"
                found="$found $bb"
            fi
        done
    done
    if [ -z "$found" ]; then
        echo "  no changed blocks in this range"
    else
        echo "== changed disk blocks:$found =="
        echo "   For an HFS volume, cross-check against the partition map: the"
        echo "   Master Directory Block sits at (Apple_HFS pmPyPartStart + 2)."
        echo "   On the OpenRetroSCSI images that is block 98, and an ordinary"
        echo "   mount/unmount touches ONLY it (drLsMod / drWrCnt / drAtrb)."
    fi
    ;;
restore)
    start=${3:?start_blk}; count=${4:?count}
    tmp=$(mktemp /tmp/sdrestore.XXXXXX.bin) || die "mktemp"
    trap 'rm -f "$tmp"' EXIT
    dd if="$img" of="$tmp" bs=512 skip="$start" count="$count" status=none
    got=$(( $(stat -c %s "$tmp") / 512 ))
    [ "$got" -eq "$count" ] || die "short read from image: wanted $count got $got"
    echo "== restoring $count blocks at disk block $start (SD LBA $((RAW_BASE_LBA+start))) =="
    JT_WAIT=1200 jt_checked "sd-write-fast $((RAW_BASE_LBA+start)) $tmp" | grep -aE 'done|ERROR|MISMATCH' | sed 's/^> /  /'
    # NOTE: sd-verify compares SD[lba..] against the file from OFFSET 0 — it does
    # NOT seek into the file.  So re-verify against the extracted slice ($tmp),
    # never against the full image, or every restore at a non-zero start block
    # reports a bogus mismatch (image checksum will be block 0's, a dead giveaway).
    echo "== re-verifying (against the extracted slice, not the whole image) =="
    JT_WAIT=600 jt "sd-verify $((RAW_BASE_LBA+start)) $tmp $count" | grep -aE 'MISMATCH|done' | sed 's/^> /  /'
    ;;
repair)
    # Converging repair: diff -> restore every damaged batch -> re-diff, until clean.
    #
    # RETRACTION (2026-08-08).  This loop used to carry the note "a SINGLE
    # restore pass does NOT reliably stick".  That was NOT true of the card or
    # of sd-write-fast.  It was the old timing-based jt() dropping the write
    # command on the floor — see the long post-mortem above jt().  With the
    # sentinel dispatch a single pass does stick, and every write is now
    # checked for an explicit REPL error instead of being sent to /dev/null.
    # The loop is kept because converging is still the right shape (a verify
    # is cheap, and a genuinely failing sector should be retried, not assumed)
    # — but a second round that finds the SAME batch again is now a real
    # finding, not tooling noise, and it aborts loudly.
    start=${3:-0}; count=${4:-$((img_blocks-start))}
    prev_bad=""
    for round in 1 2 3 4 5; do
        out=$(JT_WAIT=1800 jt "sd-verify $((RAW_BASE_LBA+start)) $img $count")
        # A scan that did not reach its own summary line is a truncated
        # capture, not a clean volume.  Refuse to interpret it.
        printf '%s\n' "$out" | grep -aq 'sd-verify done:' || \
            die "sd-verify produced no 'done' summary — capture is truncated, refusing to guess"
        bad=$(printf '%s\n' "$out" | grep -aoE 'MISMATCH lba=[0-9]+' | sed 's/.*lba=//' | sort -n)
        n=$(printf '%s\n' "$bad" | grep -c '[0-9]' || true)
        echo "== round $round: $n damaged batch(es) =="
        [ "$n" -eq 0 ] && { echo "== CLEAN =="; exit 0; }
        if [ -n "$prev_bad" ] && [ "$bad" = "$prev_bad" ]; then
            echo "== round $round found the IDENTICAL batch set as round $((round-1)) =="
            echo "   The writes are reported as succeeding but the data is not changing."
            echo "   That is a real defect (card, mapping, or write path) — investigate,"
            echo "   do NOT paper over it with more rounds."
            exit 1
        fi
        prev_bad="$bad"
        for lba in $bad; do
            blk=$((lba-RAW_BASE_LBA))
            tmp=$(mktemp /tmp/sdrep.XXXXXX.bin) || die "mktemp"
            dd if="$img" of="$tmp" bs=512 skip="$blk" count=256 status=none
            # jt_checked blocks until the REPL prompts again, so removing the
            # slice here can no longer race the command.  This exact race is
            # what produced "ERROR sd-write-fast: file not found" in the
            # 2026-08-08 logs.
            JT_WAIT=600 jt_checked "sd-write-fast $lba $tmp" >/dev/null
            rm -f "$tmp"
            echo "   restored blk $blk"
        done
    done
    echo "== STILL DIRTY after 5 rounds — investigate, do not ignore =="; exit 1
    ;;
*) die "unknown command: $cmd" ;;
esac
