#!/bin/bash
# sd_write_rom.sh — write the Quadra 700 ROM image to the SD card at LBA 0
# and VERIFY it read-back, using the same two-bitstream flow as
# tools/sd_os_swap.sh.
#
#   tools/sd_write_rom.sh                     # write files/420dbff3.rom, reboot
#   tools/sd_write_rom.sh <rom>               # write a specific image
#   tools/sd_write_rom.sh <rom> --no-boot     # leave the provisioning bitstream up
#   tools/sd_write_rom.sh --verify-only [rom] # READ-ONLY compare, writes nothing
#
# WHY THIS IS A SEPARATE SCRIPT AND NOT A FLAG ON sd_os_swap.sh
#   sd_os_swap.sh's entire safety model is one direction: "never write below
#   LBA 8192".  A ROM writer is the exact inverse — "never write at or above
#   LBA 2048".  Folding both into one tool would make that guard conditional
#   on an argument, and a conditional guard is precisely the shape that lets
#   the wrong branch through on the day someone types the wrong subcommand.
#   Two scripts, two hard-coded, opposite, unconditional guards.
#
# SD CARD LAYOUT (rtl/soc/boot_fsm.v, rtl/soc/pram_sd.v, rtl/board/sd_ctrl.v)
#   LBA 0..2047     the 1 MiB Quadra 700 ROM   <-- everything this tool writes
#   LBA 2048..8175  free
#   LBA 8176..8191  reserved system-persistence block; PRAM at 8191
#   LBA 8192+       the HDD image
#
# THE FAILURE MODE THIS GUARDS AGAINST, IN BOTH DIRECTIONS
#   Writing an OS image at LBA 0 has previously destroyed the ROM, and the
#   machine then failed in ways that looked exactly like a CPU bug (vec-4
#   faults on garbage opcodes) — an expensive misdiagnosis.  The mirror-image
#   mistake is just as bad: a ROM image longer than 2048 sectors, or a wrong
#   start LBA, would run off the end of the ROM region and eat the PRAM sector
#   and then the HDD image.  So this script refuses BOTH:
#     * the image must be EXACTLY 2048 sectors (1 MiB) — not "at most";
#       BOOT_ROM_SECTORS (Makefile:2206) is 2048 and boot_fsm reads exactly
#       that many, so a short image is a half-loaded ROM, not a small one.
#     * no sector at or above LBA 2048 is ever written.
#   Both are re-checked from the file size at write time, not inferred from
#   the argument, because arithmetic being right today is not the same as it
#   staying right.
#
# WHY IT NEEDS TWO BITSTREAMS
#   `sd-write-fast` lives ONLY in the provisioning bitstream, so this loads
#   that, writes, verifies, then reloads the main one.  Same as sd_os_swap.sh.
set -uo pipefail
cd "$(dirname "$0")/.."

# ── The immovable numbers.  Do not parameterise these. ────────────────────
ROM_BASE_LBA=0             # rtl/soc/boot_fsm.v — the ROM starts at sector 0
ROM_SECTORS=2048           # Makefile:2206 BOOT_ROM_SECTORS — 1 MiB, EXACT
ROM_LIMIT_LBA=2048         # first LBA this tool must NEVER touch
ROM_BYTES=$((ROM_SECTORS * 512))

ROM_DEFAULT=${ROM_DEFAULT:-files/420dbff3.rom}
PROV=${PROV:-build/sd_provision/sd_provision_top.bit}
MAIN=${MAIN:-build/vivado/fpga_top.bit}
LTX=${LTX:-build/vivado/fpga_top.ltx}
JT_IN=${JT_IN:-/tmp/jtag_in}
JT_OUT=${JT_OUT:-/tmp/jtag_out}

VERIFY_ONLY=0
if [ "${1:-}" = "--verify-only" ]; then VERIFY_ONLY=1; shift; fi
ROM=${1:-$ROM_DEFAULT}
NOBOOT=${2:-}
[ "${1:-}" = "--no-boot" ] && { ROM=$ROM_DEFAULT; NOBOOT=--no-boot; }

# ── Guards.  Everything below runs BEFORE the board is touched. ───────────
[ -r "$ROM" ] || { echo "FATAL: cannot read ROM image $ROM"; exit 2; }
ROM=$(realpath "$ROM") || { echo "FATAL: cannot resolve ROM image $ROM"; exit 2; }

SIZE=$(stat -c %s "$ROM")
if [ "$SIZE" -ne "$ROM_BYTES" ]; then
    echo "FATAL: refusing to write $ROM — it is $SIZE bytes"
    echo "       ($((SIZE / 512)) sectors), not exactly $ROM_BYTES bytes"
    echo "       ($ROM_SECTORS sectors)."
    echo "       boot_fsm reads exactly BOOT_ROM_SECTORS=$ROM_SECTORS sectors from"
    echo "       LBA 0, so a short image is a HALF-LOADED ROM (which presents as"
    echo "       a CPU bug, not a disk one) and a long one runs off the end of"
    echo "       the ROM region into the PRAM sector and the HDD image."
    exit 1
fi

# guard_rom_extent <start_lba> <sectors> <what>
# The inverse of sd_os_swap.sh's guard_lba(): that one refuses anything BELOW
# the reserved region, this one refuses anything AT OR ABOVE the ROM region.
# Same reasoning, opposite direction — see the header.
guard_rom_extent () {
    local start=$1 n=$2 what=$3 end=$(( $1 + $2 ))
    if [ "$start" -lt 0 ] || [ "$end" -gt "$ROM_LIMIT_LBA" ]; then
        echo "FATAL: refusing to write $what at LBA $start..$((end - 1)) —"
        echo "       that leaves the ROM region (0..$((ROM_LIMIT_LBA - 1)))."
        echo "       LBA $ROM_LIMIT_LBA and up is free space, then the reserved"
        echo "       PRAM sector at 8191, then the HDD image at 8192+."
        echo "       Writing there corrupts the disk and/or PRAM."
        exit 1
    fi
}
guard_rom_extent "$ROM_BASE_LBA" "$ROM_SECTORS" "the ROM image"

# ── Environment sanity (same checks sd_os_swap.sh makes, same reasons) ────
pgrep -x vivado >/dev/null || { echo "FATAL: no vivado — the JTAG REPL is dead, commands would hang silently"; exit 2; }

# Orphaned helper scripts driving the REPL will interleave their output with
# ours and corrupt a multi-minute write. Check by exact comm; never pkill -f.
ORPH=$(ps -eo pid,ppid,etime,comm | awk '$4 ~ /^wp[0-9]+\.sh$/')
if [ -n "$ORPH" ]; then
    echo "FATAL: orphaned helper scripts are driving the REPL:"; echo "$ORPH"
    echo "       kill with: pkill -x <name>   (exact comm, NEVER -f)"
    exit 2
fi

_ready () { grep -ac '^> READY$' "$JT_OUT" 2>/dev/null || echo 0; }

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

sd_verify_checked () {              # <command> <timeout>
    local cmd=$1 max=$2 out
    out=$(jt "$cmd" "$max") || { echo "FATAL: sd-verify did not complete"; return 1; }
    printf '%s\n' "$out" | grep -aE 'sd-verify done:|MISMATCH|ERROR' || true
    if printf '%s\n' "$out" | grep -aqE '^> ERROR|^> sd-verify: MISMATCH '; then
        echo "FATAL: the ROM did NOT read back clean — do not boot this card."
        return 1
    fi
    if ! printf '%s\n' "$out" | grep -aqE \
        '^> sd-verify done: .* 0 MISMATCHING batches$'; then
        echo "FATAL: sd-verify did not report a clean completion — treating as failed."
        return 1
    fi
}

# Everything that can fail on the host is checked BEFORE the lease is taken.
# sd_os_swap.sh acquires first and checks $PROV later, so a missing
# provisioning bitstream costs another operator a lease acquire/release cycle
# for a run that was never going to happen.  Cheap to get right here.
[ -r "$PROV" ] || { echo "FATAL: cannot read provisioning bitstream $PROV"; exit 2; }
PROV=$(realpath "$PROV") || { echo "FATAL: cannot resolve provisioning bitstream $PROV"; exit 2; }
if [ "$VERIFY_ONLY" != "1" ] && [ "$NOBOOT" != "--no-boot" ]; then
    [ -r "$MAIN" ] || { echo "FATAL: cannot read main bitstream $MAIN — refusing to"; \
                        echo "       start, since that would leave the board sitting on the"; \
                        echo "       provisioning bitstream with no way back."; exit 2; }
    [ -r "$LTX" ] || { echo "FATAL: cannot read main probes file $LTX"; exit 2; }
    MAIN=$(realpath "$MAIN") || { echo "FATAL: cannot resolve main bitstream $MAIN"; exit 2; }
    LTX=$(realpath "$LTX") || { echo "FATAL: cannot resolve main probes file $LTX"; exit 2; }
fi

LEASE="$(dirname "$0")/jtag_lease.sh"
if [ -z "${JTAG_LEASE_HELD:-}" ] && [ -x "$LEASE" ]; then
    "$LEASE" acquire "sd_write_rom.$$" 1200 2400 || { echo "FATAL: no JTAG lease"; exit 2; }
    export JTAG_LEASE_HELD="sd_write_rom.$$"
    trap '"$LEASE" release "sd_write_rom.'$$'" >/dev/null 2>&1' EXIT
fi

if [ "$VERIFY_ONLY" = "1" ]; then
    echo "### verify-only: $ROM against LBA $ROM_BASE_LBA..$((ROM_LIMIT_LBA - 1))"
    echo "### 1/2  provisioning bitstream (sd-verify lives ONLY here)"
    jt_checked_summary "load-bit $PROV" 300 '^> programmed ' "provisioning load" || exit 2
    echo "### 2/2  sd-verify (READ-ONLY: issues CMD18 read-back only, never a write)"
    sd_verify_checked "sd-verify $ROM_BASE_LBA $ROM $ROM_SECTORS" 900 || exit 1
    exit 0
fi

echo "### writing ROM $(basename "$ROM") ($ROM_SECTORS sectors) to LBA $ROM_BASE_LBA"
echo "### 1/4  provisioning bitstream (sd-write-fast lives ONLY here)"
jt_checked_summary "load-bit $PROV" 300 '^> programmed ' "provisioning load" || exit 2

echo "### 2/4  writing the ROM (sd-write-fast CRC32-verifies each batch inline)"
guard_rom_extent "$ROM_BASE_LBA" "$ROM_SECTORS" "the ROM image"
jt_checked_summary "sd-write-fast $ROM_BASE_LBA $ROM" 1800 \
    '^> sd-write-fast done:' "ROM write" || exit 1

# The inline per-batch check above only proves each batch matched what the
# host streamed for THAT batch.  This is a separate, whole-image, read-only
# pass over the finished card — the thing that catches a batch written to the
# wrong LBA, or a sector clobbered after the fact.
echo "### 3/4  full read-back verify (independent CMD18 pass over all $ROM_SECTORS sectors)"
sd_verify_checked "sd-verify $ROM_BASE_LBA $ROM $ROM_SECTORS" 900 || exit 1

if [ "$NOBOOT" = "--no-boot" ]; then
    echo "### done (--no-boot): provisioning bitstream still loaded"
    exit 0
fi

echo "### 4/4  main bitstream + boot"
jt_checked_summary "load-bit $MAIN $LTX" 300 '^> programmed ' "main load" || exit 2
echo "### ROM written and verified. build_id reads 0x00000000 until MIG"
echo "    calibration finishes — re-read it after the boot settles rather than"
echo "    calling the bitstream bad."
