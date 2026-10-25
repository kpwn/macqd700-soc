#!/usr/bin/env bash
# Repeat read-only native SD block-engine transfers across the card. The
# probe must already be loaded and the card physically power-cycled out of
# SPI mode. Every case reinitializes the controller and compares two CRCs.
set -euo pipefail

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
out_dir=${1:-/var/tmp/tqtt_sd_native_engine_matrix_$(date +%Y%m%d_%H%M%S)}
if [[ ${NATIVE_SD_CARD_POWER_CYCLED:-0} != 1 ]]; then
    echo "Set NATIVE_SD_CARD_POWER_CYCLED=1 only after reseating the card with the native engine probe loaded." >&2
    exit 2
fi
mkdir -p "$out_dir"
out_dir=$(cd "$out_dir" && pwd)

engine_dir=${NATIVE_SD_ENGINE_BUILD_DIR:-$repo_dir/build/tqtt_sd_native_engine_probe}
[[ $engine_dir = /* ]] || engine_dir="$repo_dir/$engine_dir"
bit="$engine_dir/tqtt_sd_native_engine_probe.bit"
ltx="$engine_dir/tqtt_sd_native_engine_probe.ltx"
[[ -r $bit && -r $ltx ]] || { echo "Native engine probe bit/LTX missing" >&2; exit 2; }

# ROM, first HDD sectors, a previously changed filesystem region, and widely
# separated card areas. The runner checks card capacity before every read.
cases=(0:1 0:32 8192:32 90000:32 95410:32 250000:32 500000:32 1000000:32 2000000:32)
summary="$out_dir/summary.tsv"
printf 'lba\tblocks\trepeat\tcrc32\tdma_cycles\tMB_per_s\tlog\n' > "$summary"

run_one() {
    local lba=$1 blocks=$2 repeat=$3 log="$out_dir/lba${1}_${2}blocks_run${3}.log"
    local line crc cycles rate
    if ! (cd "$repo_dir" && NATIVE_SD_NO_PROGRAM=1 vivado -nojournal -nolog \
        -mode batch -source tools/tqtt_sd_native_engine_read.tcl -tclargs \
        "$bit" "$ltx" "$lba" "$blocks") > "$log" 2>&1; then
        echo "Native SD engine read failed: $log" >&2
        tail -30 "$log" >&2
        return 1
    fi
    line=$(grep -E '^PASS LBA=' "$log" | tail -1 || true)
    [[ $line == "PASS LBA=${lba} blocks=${blocks} "* ]] || {
        echo "Missing or mismatched PASS line: $log" >&2
        return 1
    }
    [[ $line =~ CRC32=(0x[0-9a-fA-F]+) ]] || { echo "Missing CRC: $log" >&2; return 1; }
    crc=${BASH_REMATCH[1]}
    [[ $line =~ DMA_cycles=([0-9]+) ]] || { echo "Missing DMA cycles: $log" >&2; return 1; }
    cycles=${BASH_REMATCH[1]}
    [[ $line =~ DMA_MBps=([0-9.]+) ]] || { echo "Missing DMA rate: $log" >&2; return 1; }
    rate=${BASH_REMATCH[1]}
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
        "$lba" "$blocks" "$repeat" "$crc" "$cycles" "$rate" "$log" >> "$summary"
    printf '%s\n' "$crc"
}

run_matrix() {
    local case_spec lba blocks crc_first crc_second
    for case_spec in "${cases[@]}"; do
        IFS=: read -r lba blocks <<< "$case_spec"
        crc_first=$(run_one "$lba" "$blocks" 1)
        crc_second=$(run_one "$lba" "$blocks" 2)
        if [[ ${crc_first,,} != "${crc_second,,}" ]]; then
            echo "CRC mismatch at LBA $lba, $blocks blocks: $crc_first vs $crc_second" >&2
            return 1
        fi
        echo "PASS LBA $lba, $blocks blocks: CRC $crc_first repeated"
    done
    echo "PASS native SD engine read matrix; results: $summary"
}

# One lease covers the complete read-only matrix; it is released on failure.
"$repo_dir/tools/jtag_lease.sh" acquire native_sd_engine_matrix 1200
trap '"$repo_dir/tools/jtag_lease.sh" release native_sd_engine_matrix >/dev/null 2>&1' EXIT
run_matrix
