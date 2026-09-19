#!/bin/bash
# sd_os_swap.sh — swap the boot OS on the SD card in ~25 s (7.0.1) / ~2.5 min
# (7.5.3), instead of the ~33 min a full 500 MB image would take.
#
#   tools/sd_os_swap.sh 701          # write System 7.0.1 and boot it
#   tools/sd_os_swap.sh 753          # write System 7.5.3 and boot it
#   tools/sd_os_swap.sh list         # show the known images and their extents
#   tools/sd_os_swap.sh 753 --no-boot   # leave the provisioning bitstream up
#
# HOW IT IS FAST
#   A 500 MB OpenRetroSCSI image is nearly all free space. The HFS allocation
#   bitmap says which blocks are actually used; writing only up to the highest
#   set bit leaves a fully consistent volume. Validated end-to-end 2026-08-03
#   by round-tripping 7.5.3 -> 7.0.1 -> 7.5.3 -> 7.0.1 with no corruption, the
#   restored 7.0.1 booting to the Finder with correct free-space accounting.
#
# THE PART EVERYONE GETS WRONG
#   You must also write the ALTERNATE MDB, which lives at the second-to-last
#   block of the VOLUME -- not the end of the file. Computing it from the
#   partition map came out two sectors wrong; hfs_used_extent.py scans for the
#   `BD` signature instead. Skip it and the volume mounts but is subtly wrong.
#
# WHY IT NEEDS TWO BITSTREAMS
#   `sd-write-fast` lives ONLY in the provisioning bitstream, so this loads
#   that, writes, then reloads the main one. Each load-bit is ~20 s, which is
#   why the tool does the whole sequence rather than leaving you to remember.
set -uo pipefail
cd "$(dirname "$0")/.."

IMG_701=${IMG_701:-/home/qwertyoruiop/hda75/HD0-OpenRetroSCSI-7.0.1-500M.hda}
IMG_753=${IMG_753:-/home/qwertyoruiop/hda75/HD0-OpenRetroSCSI-7.5.3.hda}
PROV=${PROV:-build/sd_provision/sd_provision_top.bit}
MAIN=${MAIN:-build/vivado/fpga_top.bit}
LTX=${LTX:-build/vivado/fpga_top.ltx}
# SD CARD LAYOUT — LBA 0..8191 IS NOT OURS.
#   LBA 0..8190  the Quadra 700 ROM image
#   LBA 8191     the reserved PRAM sector (pram_sd.v)
#   LBA 8192+    the HDD image  <-- everything this tool writes
# Writing a disk image at LBA 0 destroys the ROM, and the machine then fails
# in ways that look exactly like a CPU bug (vec-4 faults on garbage opcodes).
# That is why every write below is offset by RAW_BASE_LBA and then RE-CHECKED
# against it: arithmetic being right today is not the same as it staying right.
RAW_BASE_LBA=8192          # rtl/board/sd_ctrl.v — COMPILE-TIME, not a register
SD_RESERVED_LBAS=8192      # ROM + PRAM. NEVER write below this.
JT_IN=${JT_IN:-/tmp/jtag_in}
JT_OUT=${JT_OUT:-/tmp/jtag_out}
# Large custom images can legitimately exceed the historical 30-minute
# bound (the stock 7.5.3 used prefix is much smaller).  Keep the safe default
# for normal UI swaps while allowing an operator to raise it deliberately.
SD_WRITE_TIMEOUT=${SD_WRITE_TIMEOUT:-1800}
JTAG_LEASE_TTL=${JTAG_LEASE_TTL:-2400}

_ready () {
    local n
    n=$(grep -ac '^> READY$' "$JT_OUT" 2>/dev/null || true)
    echo "${n:-0}"
}

# Dispatch on the REPL's own prompt, never on a fixed sleep and never on
# `tail -n1 == READY` (Vivado appends async INFO lines after the prompt, which
# makes that predicate permanently false on a healthy REPL).
jt () {
    local cmd=$1 max=${2:-600} before r0 i
    before=$(wc -l < "$JT_OUT"); r0=$(_ready)
    printf '%s\n' "$cmd" > "$JT_IN"
    for ((i=0; i<max*7; i++)); do
        [ "$(_ready)" -gt "$r0" ] && { tail -n +$((before+1)) "$JT_OUT"; return 0; }
        sleep 0.15
    done
    echo "FATAL: REPL timeout after ${max}s on: $cmd" >&2
    return 1
}

jt_checked_summary () {             # <command> <timeout> <success-regex> <what>
    local cmd=$1 max=$2 success=$3 what=$4 out
    out=$(jt "$cmd" "$max") || { echo "FATAL: $what did not complete"; return 1; }
    printf '%s\n' "$out" | grep -aE 'programmed|done|ERROR|MISMATCH' || true
    if printf '%s\n' "$out" | grep -aqE '^> ERROR|MISMATCH'; then
        echo "FATAL: $what reported an error"
        return 1
    fi
    if ! printf '%s\n' "$out" | grep -aqE "$success"; then
        echo "FATAL: $what produced no success marker"
        return 1
    fi
}

describe () {                    # describe <tag> <img>
    local out; out=$(python3 tools/hfs_used_extent.py "$2") || return 1
    local used alt mb
    used=$(sed -n 's/^USED_SECTORS=//p' <<<"$out")
    alt=$(sed -n 's/^ALT_MDB_SECTOR=//p' <<<"$out")
    mb=$(sed -n 's/^USED_MB=//p' <<<"$out")
    printf "  %-5s %-52s used=%s MB (%s sectors)  altMDB=%s\n" \
           "$1" "$(basename "$2")" "$mb" "$used" "$alt"
}

case "${1:-}" in
  list)
    echo "known images:"
    describe 701 "$IMG_701"
    describe 753 "$IMG_753"
    exit 0 ;;
  701) TAG=701; IMG=$IMG_701 ;;
  753) TAG=753; IMG=$IMG_753 ;;
  *)   echo "usage: $0 {701|753|list} [--no-boot]"; exit 2 ;;
esac
NOBOOT=${2:-}

[ -r "$IMG" ]  || { echo "FATAL: cannot read $IMG"; exit 2; }
[ -r "$PROV" ] || { echo "FATAL: cannot read provisioning bitstream $PROV"; exit 2; }

# load-bit runs inside the long-lived Vivado process, whose cwd may be a
# different worktree from this script.  Never send it a repo-relative path.
PROV=$(realpath "$PROV") || { echo "FATAL: cannot resolve provisioning bitstream $PROV"; exit 2; }
if [ "$NOBOOT" != "--no-boot" ]; then
    [ -r "$MAIN" ] || { echo "FATAL: cannot read main bitstream $MAIN"; exit 2; }
    [ -r "$LTX" ]  || { echo "FATAL: cannot read main probes file $LTX"; exit 2; }
    MAIN=$(realpath "$MAIN") || { echo "FATAL: cannot resolve main bitstream $MAIN"; exit 2; }
    LTX=$(realpath "$LTX") || { echo "FATAL: cannot resolve main probes file $LTX"; exit 2; }
fi
pgrep -x vivado >/dev/null || { echo "FATAL: no vivado — the JTAG REPL is dead, commands would hang silently"; exit 2; }

# Orphaned helper scripts driving the REPL will interleave their output with
# ours and corrupt a multi-minute write. Check by exact comm; never pkill -f.
ORPH=$(ps -eo pid,ppid,etime,comm | awk '$4 ~ /^wp[0-9]+\.sh$/')
if [ -n "$ORPH" ]; then
    echo "FATAL: orphaned helper scripts are driving the REPL:"; echo "$ORPH"
    echo "       kill with: pkill -x <name>   (exact comm, NEVER -f)"
    exit 2
fi

LEASE="$(dirname "$0")/jtag_lease.sh"
LEASE_OWNER=
if [ -z "${JTAG_LEASE_HELD:-}" ] && [ -x "$LEASE" ]; then
    LEASE_OWNER="sd_os_swap.$$"
    "$LEASE" acquire "$LEASE_OWNER" "$JTAG_LEASE_TTL" 3600 || { echo "FATAL: no JTAG lease"; exit 2; }
    export JTAG_LEASE_HELD="$LEASE_OWNER"
fi

eval "$(python3 tools/hfs_used_extent.py "$IMG" | grep -aE '^(USED_SECTORS|ALT_MDB_SECTOR|IMG_SECTORS)=')"
[ "${ALT_MDB_SECTOR:--1}" -ge 0 ] || { echo "FATAL: no alternate MDB found — refusing a partial write"; exit 1; }

# Refuse any write that would land in the ROM/PRAM region, whatever the
# arithmetic upstream did.  Belt and braces on purpose: the cost of a false
# refusal is re-running a 25 s tool; the cost of a false accept is a wiped ROM
# that presents as a CPU bug.
guard_lba () {                      # guard_lba <lba> <what>
    if [ "$1" -lt "$SD_RESERVED_LBAS" ]; then
        echo "FATAL: refusing to write $2 at LBA $1 — that is inside the"
        echo "       reserved ROM/PRAM region (0..$((SD_RESERVED_LBAS-1)))."
        echo "       Writing there destroys the ROM and the failure then looks"
        echo "       like a CPU bug, not a disk one."
        exit 1
    fi
}

HEAD=$(mktemp /tmp/os_head.XXXXXX.bin); TAIL=$(mktemp /tmp/os_tail.XXXXXX.bin)
cleanup () {
    rm -f "$HEAD" "$TAIL"
    [ -z "$LEASE_OWNER" ] || "$LEASE" release "$LEASE_OWNER" >/dev/null 2>&1
}
trap cleanup EXIT
dd if="$IMG" of="$HEAD" bs=512 count="$USED_SECTORS"        status=none
dd if="$IMG" of="$TAIL" bs=512 skip="$ALT_MDB_SECTOR"       status=none

echo "### swapping to System ${TAG}: ${USED_SECTORS} sectors + alt MDB @ ${ALT_MDB_SECTOR}"
echo "### 1/4  provisioning bitstream (sd-write-fast lives ONLY here)"
jt_checked_summary "load-bit $PROV" 300 '^> programmed ' "provisioning load" || exit 2

echo "### 2/4  writing the used extent"
guard_lba "$RAW_BASE_LBA" "the used extent"
# Timeout scaled from the actual work, not a guess. The fixed 1800 s here
# killed a 1,142,614-sector image at ~60% and left the volume half-written --
# i.e. the timeout CORRUPTED the disk it was protecting. sd-write-fast runs at
# roughly 260-280 KiB/s over JTAG, so allow 512 sectors/s with a 600 s floor
# and a generous margin. Override with SD_WRITE_TIMEOUT if needed.
SD_WRITE_TIMEOUT=${SD_WRITE_TIMEOUT:-$(( USED_SECTORS / 256 + 600 ))}
echo "### (write timeout: ${SD_WRITE_TIMEOUT}s for $USED_SECTORS sectors)"
jt_checked_summary "sd-write-fast $RAW_BASE_LBA $HEAD" "$SD_WRITE_TIMEOUT" \
    '^> sd-write-fast done:' "used-extent write" || exit 1

echo "### 3/4  writing the alternate MDB (skip this and the volume is subtly wrong)"
guard_lba "$((RAW_BASE_LBA + ALT_MDB_SECTOR))" "the alternate MDB"
jt_checked_summary "sd-write-fast $((RAW_BASE_LBA + ALT_MDB_SECTOR)) $TAIL" 600 \
    '^> sd-write-fast done:' "alternate-MDB write" || exit 1

if [ "$NOBOOT" = "--no-boot" ]; then
    echo "### done (--no-boot): provisioning bitstream still loaded"
    exit 0
fi

echo "### 4/4  main bitstream + boot"
jt_checked_summary "load-bit $MAIN $LTX" 300 '^> programmed ' "main load" || exit 2
echo "### System ${TAG} written. build_id reads 0x00000000 until MIG calibration"
echo "    finishes — re-read it after the boot settles rather than calling the"
echo "    bitstream bad."
