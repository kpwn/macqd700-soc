#!/bin/bash
# boot_clean.sh — the ONLY sanctioned way to boot the Mac on this board.
#
# WHY THIS EXISTS
#   Boot attempts change the SD boot volume, so a run that starts from an
#   unknown volume state is not a controlled experiment.  This script pins the
#   volume to the golden image before every attempt and refuses to boot if it
#   cannot.
#
# CORRECTED 2026-08-08 — two claims that used to live in this header were
# WRONG, and both are worth stating so they are not re-derived:
#
#   * "a FAILED boot rewrites disk block 0 (CRC 0x8C5908B9 -> 0xC738D859 ->
#     0x19D3E2A3)".  Block 0 was never written.  8192 is the RAW SD LBA of the
#     BASE of the first 256-sector verify batch, i.e. disk blocks 0..255; the
#     tool's granularity, not a location.  MAME running the same ROM and the
#     same image writes exactly ONE block on a boot that finds no bootable
#     System: block 98, the HFS Master Directory Block, updating drAtrb bit 8
#     ("volume unmounted cleanly", 0x0A) and drWrCnt (0x46..0x49).  The ROM
#     mounts a volume in order to decide whether it is bootable, and HFS mount
#     and unmount both write the MDB.  The CRC differing on every attempt is
#     drWrCnt incrementing.  Normal Mac behaviour, not damage.
#
#   * "a single restore pass silently does not stick / the diff does not
#     enumerate every damaged batch in one pass".  Both were artefacts of the
#     old output-quiet jt() below, which declared a command finished after 1.0 s
#     of silence while sd-verify's own progress lines are 1.13-1.24 s apart.
#     Captures were truncated at random points and queued commands ran after
#     their temp files had been deleted.  See the post-mortem in
#     tools/sd_disk_diff.sh.  Both scripts now dispatch on the REPL's `> READY`
#     prompt instead.
#
#   Repair before each attempt is still the right discipline: it makes the
#   starting state known.  It is no longer evidence of a disk-path defect.
#
# THE LEASE GUARDS THE CABLE, NOT THE REPL'S COMMAND QUEUE
#   This script now takes tools/jtag_lease.sh for its whole run.  Be aware of
#   the gap that remains, because it bites silently:
#
#   The lease is released when the holder's process exits or its TTL expires.
#   Neither of those stops work the holder already pushed into /tmp/jtag_in —
#   the Vivado REPL is a separate long-lived process with no notion of the
#   lease, and a queued `sd-verify` runs for ~27 s (a 500 MB one for ~285 s).
#   So `jtag_lease.sh status` can legitimately report FREE while the REPL is
#   still streaming the previous holder's output.  MEASURED 2026-08-08:
#   status said `FREE (stale holder reaped)` while /tmp/jtag_out was growing
#   through another agent's scan (458752 -> 544768 sectors).  A new holder
#   that acquires at that moment and starts issuing commands will have its
#   replies interleaved with the old ones — exactly the failure the lease was
#   introduced to prevent.
#
#   Mitigation, and why jt() below starts with jt_sync(): never issue anything
#   until the REPL is demonstrably idle AT THE PROMPT.  Holding the lease is
#   not sufficient; draining is.  Any other tool that drives /tmp/jtag_in
#   needs the same discipline.  The durable fix would be a `drain` step inside
#   jtag_lease.sh's own acquire path so every holder inherits a quiet REPL.
#
# USAGE
#   tools/boot_clean.sh              # repair, then boot, then report state
#   tools/boot_clean.sh --no-boot    # repair only, leave provisioning loaded
#
# It is safe to run this every single time.  A clean volume repairs in ~27 s.
set -uo pipefail
cd "$(dirname "$0")/.."

IMG=${IMG:-/home/qwertyoruiop/hda75/HD0-OpenRetroSCSI-7.5.3.hda}
PROV=${PROV:-build/sd_provision/sd_provision_top.bit}
MAIN=${MAIN:-/home/qwertyoruiop/macqd700-soc-worktrees/scsi-nondma-out/build/vivado/fpga_top.bit}
LTX=${LTX:-/home/qwertyoruiop/macqd700-soc-worktrees/scsi-nondma-out/build/vivado/fpga_top.ltx}
JT_IN=${JT_IN:-/tmp/jtag_in}
JT_OUT=${JT_OUT:-/tmp/jtag_out}

# Dispatch on the REPL's own `> READY` prompt, never on output quiet.  Writing
# to the FIFO succeeds even while the REPL is busy (kernel pipe buffer), so
# "the command was accepted" says nothing about "the command has run".  The
# full post-mortem for why the old quiet-based version produced three phantom
# findings is in tools/sd_disk_diff.sh above its jt().
#
# This REPLACES a quiet-tick version that counted consecutive 0.2 s ticks of
# silence.  Recording why, because it is the sharpest illustration of why
# output-quiet dispatch is unsound: 5 ticks (1.0 s) was too short for load-bit,
# whose program_hw_devices step takes ~40 s with internal quiet gaps well over
# 1 s.  jt() returned early, `grep programmed` failed on a load that had
# SUCCEEDED, and the script exited BEFORE repairing — reporting FATAL while
# leaving a dirty disk to boot from.  Raising the threshold only moves the
# cliff; dispatching on the prompt removes it.  Callers may still pass a third
# argument (the old quiet_ticks); it is accepted and ignored.
jt_ready_count () { grep -ac '^> READY$' "$JT_OUT" 2>/dev/null || echo 0; }

jt_sync () { # wait until nothing is in flight
    # DO NOT test `tail -n1 == "> READY"`.  MEASURED 2026-08-08: Vivado appends
    # ASYNCHRONOUS lines AFTER the prompt -- e.g.
    #   INFO: [Labtools 27-3143] Calibration status change detected, refreshing MIG_1
    # arrives on its own after any load-bit -- so once MIG calibration chatters,
    # the last line is never "> READY" again and this spun the full 1800 s
    # before declaring a wedge on a perfectly healthy REPL.  It cost a board
    # run mid-experiment.
    #
    # Idle is instead: output has stopped moving AND at least one prompt has
    # been printed.  A prompt that is no longer the last line is still a
    # prompt.  Same lesson as the quiet-window bug: a dispatch predicate that
    # a NORMAL event can invalidate is not a predicate.
    local w=${1:-1800} n0 n1 i
    for ((i=0;i<w*2;i++)); do
        n0=$(wc -l < "$JT_OUT" 2>/dev/null || echo 0)
        sleep 0.5
        n1=$(wc -l < "$JT_OUT" 2>/dev/null || echo 0)
        if [ "$n0" -eq "$n1" ] && [ "$(jt_ready_count)" -gt 0 ]; then
            return 0
        fi
    done
    echo "FATAL: REPL never went idle after ${w}s — previous command wedged?"; exit 2
}

jt () { # jt <wait_s> <cmd>  — prints only THAT command's repl output
    local w=$1; shift
    local before ready0 i
    jt_sync
    before=$(wc -l < "$JT_OUT" 2>/dev/null || echo 0)
    ready0=$(jt_ready_count)
    timeout 5 bash -c "echo \"\$1\" > $JT_IN" _ "$1" || { echo "FATAL: REPL not accepting input"; exit 2; }
    for ((i=0;i<w*5;i++)); do
        sleep 0.2
        if [ "$(jt_ready_count)" -gt "$ready0" ]; then
            tail -n +$((before+1)) "$JT_OUT"
            return 0
        fi
    done
    echo "FATAL: REPL did not return to the prompt within ${w}s on: $1"; exit 2
}

LEASE="$(dirname "$0")/jtag_lease.sh"
if [ -z "${JTAG_LEASE_HELD:-}" ] && [ -x "$LEASE" ]; then
    "$LEASE" acquire "boot_clean.$$" 1800 3600 || { echo "FATAL: no JTAG lease"; exit 2; }
    export JTAG_LEASE_HELD="boot_clean.$$"
    trap '"$LEASE" release "boot_clean.$$" >/dev/null 2>&1' EXIT
fi

pgrep -x vivado >/dev/null || { echo "FATAL: no vivado — the JTAG REPL is dead. Commands would hang silently."; exit 2; }
[ -r "$IMG" ]  || { echo "FATAL: cannot read image $IMG"; exit 2; }
[ -r "$PROV" ] || { echo "FATAL: cannot read provisioning bitstream $PROV"; exit 2; }
IMG=$(realpath "$IMG") || { echo "FATAL: cannot resolve image $IMG"; exit 2; }
PROV=$(realpath "$PROV") || { echo "FATAL: cannot resolve provisioning bitstream $PROV"; exit 2; }
if [ "${1:-}" != "--no-boot" ]; then
    [ -r "$MAIN" ] || { echo "FATAL: cannot read main bitstream $MAIN"; exit 2; }
    [ -r "$LTX" ]  || { echo "FATAL: cannot read main probes file $LTX"; exit 2; }
    MAIN=$(realpath "$MAIN") || { echo "FATAL: cannot resolve main bitstream $MAIN"; exit 2; }
    LTX=$(realpath "$LTX") || { echo "FATAL: cannot resolve main probes file $LTX"; exit 2; }
fi

echo "### 1/3  provisioning bitstream (sd-verify/sd-write-fast live ONLY here)"
jt 180 "load-bit $PROV" 60 | grep -a programmed || { echo "FATAL: provisioning load failed"; exit 2; }

echo "### 2/3  restore the boot volume"
# SEQUENTIAL FULL-EXTENT WRITE, *not* the converging batch repair.
#
# MEASURED 2026-08-08, and this cost nine wasted board attempts before it was
# isolated: a volume prepared with `sd_disk_diff.sh repair` DID NOT BOOT -- nine
# consecutive runs sat at the ROM disk prompt and never reached the OS, which
# was misread each time as "the experiment found nothing". A single sequential
# `restore 0 $EXTENT` of the same image, from the same host file, on the same
# bitstream, BOOTED. That is the whole difference.
#
# The MECHANISM IS STILL OPEN. What is established is only the outcome above.
# The obvious suspect is that the batch diff under-enumerates, so `repair`
# certifies clean while residual damage remains -- but note that the previous
# comment here asserted a "RANGE-DEPENDENT under-enumeration" as fact, and that
# claim was later traced to the quiet-window jt() bug and retracted. Do not
# re-derive a mechanism from this comment; measure it.
#
# So: write, don't diff-and-patch. It costs ~191 s for the 7.5.3 extent versus
# ~27 s for a clean repair, which is a trade worth making every single time --
# an unbootable volume burns an entire experiment, not three minutes.
#
# EXTENT is the USED extent in 512 B blocks (HFS bitmap), not the image size.
# 95411 = 48.9 MB is the 7.5.3 image. It is IMAGE-SPECIFIC: 7.0.1 is ~6.6 MB.
# Override for a different OS image.
EXTENT=${EXTENT:-95411}
IMG_BLOCKS=$(( $(stat -c %s "$IMG") / 512 ))
[ "$EXTENT" -le "$IMG_BLOCKS" ] || { echo "FATAL: EXTENT $EXTENT exceeds image ($IMG_BLOCKS blocks)"; exit 2; }
if ! tools/sd_disk_diff.sh restore "$IMG" 0 "$EXTENT"; then
    echo "FATAL: sequential restore failed. DO NOT BOOT — you would be"
    echo "       diagnosing a broken disk, not the bug."
    exit 1
fi

# Belt and braces: the batch diff over the FULL image, to catch anything the
# sequential write did not cover (i.e. past the used extent). If this reports
# damage INSIDE 0..EXTENT, that is a real finding about the write path itself
# -- those blocks were just written sequentially and verified.
echo "### 2b/3  post-restore diff over the full image (outside-extent check)"
if ! tools/sd_disk_diff.sh repair "$IMG" 0 "$IMG_BLOCKS"; then
    echo "FATAL: volume did NOT converge to clean after a sequential restore."
    echo "       DO NOT BOOT. Investigate — this is stronger than a stale disk."
    exit 1
fi

[ "${1:-}" = "--no-boot" ] && { echo "### done (--no-boot): provisioning still loaded, volume CLEAN"; exit 0; }

echo "### 3/3  main bitstream + boot"
[ -r "$MAIN" ] || { echo "FATAL: cannot read main bitstream $MAIN"; exit 2; }
jt 180 "load-bit $MAIN $LTX" 60 | grep -a programmed || { echo "FATAL: main load failed"; exit 2; }

echo "    waiting for boot..."
for i in $(seq 1 24); do sleep 5; done
PC=$(jt 25 "pc" | grep -aoE '0x[0-9a-f]{8}' | tail -1)
echo "    pc = $PC"
case "$PC" in
    0x40898*) echo "    ^ ROM DISK PROMPT — no bootable System. The volume was clean, so this"
              echo "      is NOT a disk problem; something else went wrong. Do not just retry." ;;
    0x4080e*) echo "    ^ IN THE ResrvMem HANG (expected target state)." ;;
    *)        echo "    ^ elsewhere — sample again; it may still be booting." ;;
esac
echo "### volume matched the golden image at boot, so any difference from here"
echo "    was written by THIS run.  Note that MOST of it is expected: a normal"
echo "    7.5.3 boot legitimately writes 30 blocks across 10 of the 256-block"
echo "    batches (MAME reference, 2026-08-08).  A dirty batch is NOT evidence"
echo "    of corruption — use \`sd_disk_diff.sh localize <image> <batch>\` to see"
echo "    WHICH blocks changed before calling anything damage."
