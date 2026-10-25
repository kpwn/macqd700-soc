#!/usr/bin/env bash
# Compare read-only four-bit SD transfers at 25 and negotiated 50 MHz.
# The native probe must already be loaded, and the card must have been
# power-cycled after the last SPI-mode boot. No card writes are issued.
set -euo pipefail

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
out_dir=${1:-/var/tmp/tqtt_sd_native_matrix_$(date +%Y%m%d_%H%M%S)}
if [[ ${NATIVE_SD_CARD_POWER_CYCLED:-0} != 1 ]]; then
    echo "Set NATIVE_SD_CARD_POWER_CYCLED=1 only after reseating the card with the native probe loaded." >&2
    exit 2
fi
mkdir -p "$out_dir"
out_dir=$(cd "$out_dir" && pwd)

bit="$repo_dir/build/tqtt_sd_native_probe/tqtt_sd_native_probe.bit"
ltx="$repo_dir/build/tqtt_sd_native_probe/tqtt_sd_native_probe.ltx"
[[ -r $bit && -r $ltx ]] || { echo "Native probe bit/LTX missing" >&2; exit 2; }

# Each 32-sector range fits the probe's 16 KiB RAM. Include the partition
# map, ROM-reserved boundary, and a filesystem region. Run matching ranges
# at both speeds; CRCs must agree, even if the mounted volume was modified.
lbas=(0 8192 95410)
summary="$out_dir/summary.tsv"
printf 'lba\tblocks\tmhz\tcrc32\tdma_cycles\tMiB_per_s\tlog\n' > "$summary"

run_one() {
    local lba=$1 speed=$2 log="$out_dir/lba${1}_${2}mhz.log" line crc cycles rate
    if ! (cd "$repo_dir" && NATIVE_SD_NO_PROGRAM=1 vivado -nojournal -nolog \
        -mode batch -source tools/tqtt_sd_native_read_probe.tcl -tclargs \
        "$bit" "$ltx" "$lba" 32 "$speed") > "$log" 2>&1; then
        echo "Native SD probe failed: $log" >&2
        tail -30 "$log" >&2
        return 1
    fi
    line=$(grep -E '^PASS native=' "$log" | tail -1 || true)
    [[ $line == "PASS native=${speed}MHz lba=${lba} blocks=32 "* ]] || {
        echo "Missing or mismatched PASS line: $log" >&2
        return 1
    }
    [[ $line =~ crc32=(0x[0-9a-fA-F]+) ]] || { echo "Missing CRC: $log" >&2; return 1; }
    crc=${BASH_REMATCH[1]}
    [[ $line =~ wire_cycles=([0-9]+) ]] || { echo "Missing DMA cycles: $log" >&2; return 1; }
    cycles=${BASH_REMATCH[1]}
    [[ $line =~ wire_MiBps=([0-9.]+) ]] || { echo "Missing rate: $log" >&2; return 1; }
    rate=${BASH_REMATCH[1]}
    printf '%s\t32\t%s\t%s\t%s\t%s\t%s\n' "$lba" "$speed" "$crc" "$cycles" "$rate" "$log" >> "$summary"
    printf '%s\n' "$crc"
}

run_matrix() {
    local lba crc25 crc50
    for lba in "${lbas[@]}"; do
        crc25=$(run_one "$lba" 25)
        crc50=$(run_one "$lba" 50)
        if [[ ${crc25,,} != "${crc50,,}" ]]; then
            echo "CRC mismatch at LBA $lba: 25 MHz $crc25, 50 MHz $crc50" >&2
            return 1
        fi
        echo "PASS LBA $lba: CRC $crc25 matches at 25 and 50 MHz"
    done
    echo "PASS native SD 25/50 MHz matrix; results: $summary"
}

# One lease covers this single read-only matrix experiment. Release it even
# if a Vivado run or CRC comparison fails.
"$repo_dir/tools/jtag_lease.sh" acquire native_sd_matrix 1200
trap '"$repo_dir/tools/jtag_lease.sh" release native_sd_matrix >/dev/null 2>&1' EXIT
run_matrix
