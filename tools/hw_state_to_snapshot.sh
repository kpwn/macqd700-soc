#!/usr/bin/env bash
# hw_state_to_snapshot.sh — capture HW arch state + selected DRAM regions
# into a state_replay_v1 snapshot.  Skips uninitialized DRAM regions (=
# zero-filled in the snapshot) to dramatically reduce capture time.
#
# Live regions captured (Mac OS Q700 mid-boot):
#   0x000000-0x040000  low RAM (256 KB) — toolbox tables, vectors, heap
#   0x1F0000-0x200000  stack region (64 KB) — supervisor stack
#   0x3FC000-0x400000  MMU page tables (16 KB)
#
# Prereq: HW already halted at the desired state.
#
# Usage:
#   tools/hw_state_to_snapshot.sh /tmp/snap_hw_pre_fault

set -euo pipefail

OUT="${1:-/tmp/snap_hw}"
OUT_DIR="$(dirname "$OUT")"
OUT_BASE="$(basename "$OUT")"
mkdir -p "$OUT_DIR"

OUT_TXT="${OUT}.txt"
OUT_DRAM="${OUT}.dram"
OUT_PACK="$OUT"

echo "=== Capturing HW state to $OUT ==="

# ── Pull arch regs ────────────────────────────────────────────────
echo "Reading arch regs..."
ARCH_OUT=$(/tmp/jcmd.sh "arch" 2>&1)
HALT_OUT=$(/tmp/jcmd.sh "halt-status" 2>&1)
MMU_OUT=$(/tmp/jcmd.sh "live-mmu" 2>&1)

get_reg() {
    echo "$ARCH_OUT" | grep -oE "^> $1\s*=\s*0x[0-9a-fA-F]+" | head -1 | grep -oE "0x[0-9a-fA-F]+$" | sed 's/0x//' | tr 'a-f' 'A-F'
}

PC=$(echo "$HALT_OUT" | grep -oE "pc_live=0x[0-9a-fA-F]+" | head -1 | grep -oE "0x[0-9a-fA-F]+" | sed 's/0x//' | tr 'a-f' 'A-F')
SR=$(get_reg "SR ")
VBR=$(get_reg "VBR")
D0=$(get_reg "D0"); D1=$(get_reg "D1"); D2=$(get_reg "D2"); D3=$(get_reg "D3")
D4=$(get_reg "D4"); D5=$(get_reg "D5"); D6=$(get_reg "D6"); D7=$(get_reg "D7")
A0=$(get_reg "A0"); A1=$(get_reg "A1"); A2=$(get_reg "A2"); A3=$(get_reg "A3")
A4=$(get_reg "A4"); A5=$(get_reg "A5"); A6=$(get_reg "A6"); A7=$(get_reg "A7 " | head -1)

TC=$(echo "$MMU_OUT" | grep -oE "TC=0x[0-9a-fA-F]+" | head -1 | grep -oE "0x[0-9a-fA-F]+" | sed 's/0x//' | tr 'a-f' 'A-F')
SRP=$(echo "$MMU_OUT" | grep -oE "SRP=0x[0-9a-fA-F]+" | head -1 | grep -oE "0x[0-9a-fA-F]+" | sed 's/0x//' | tr 'a-f' 'A-F')
URP=$(echo "$MMU_OUT" | grep -oE "URP=0x[0-9a-fA-F]+" | head -1 | grep -oE "0x[0-9a-fA-F]+" | sed 's/0x//' | tr 'a-f' 'A-F')
DTT0=$(echo "$MMU_OUT" | grep -oE "DTT0=0x[0-9a-fA-F]+" | head -1 | grep -oE "0x[0-9a-fA-F]+" | sed 's/0x//' | tr 'a-f' 'A-F')
DTT1=$(echo "$MMU_OUT" | grep -oE "DTT1=0x[0-9a-fA-F]+" | head -1 | grep -oE "0x[0-9a-fA-F]+" | sed 's/0x//' | tr 'a-f' 'A-F')
ITT0=$(echo "$MMU_OUT" | grep -oE "ITT0=0x[0-9a-fA-F]+" | head -1 | grep -oE "0x[0-9a-fA-F]+" | sed 's/0x//' | tr 'a-f' 'A-F')
ITT1=$(echo "$MMU_OUT" | grep -oE "ITT1=0x[0-9a-fA-F]+" | head -1 | grep -oE "0x[0-9a-fA-F]+" | sed 's/0x//' | tr 'a-f' 'A-F')
CACR=80008000

{
    echo "STATE_HDR PC=$PC SR=$SR VBR=$VBR USP=0 SP=$A7"
    echo "STATE_D D0=$D0 D1=$D1 D2=$D2 D3=$D3 D4=$D4 D5=$D5 D6=$D6 D7=$D7"
    echo "STATE_A A0=$A0 A1=$A1 A2=$A2 A3=$A3 A4=$A4 A5=$A5 A6=$A6 A7=$A7"
    echo "STATE_MMU TC=$TC SRP=$SRP URP=$URP DTT0=$DTT0 DTT1=$DTT1 ITT0=$ITT0 ITT1=$ITT1 CACR=$CACR"
} > "$OUT_TXT"

echo "Wrote arch state:"
cat "$OUT_TXT"

# ── Build 4 MiB DRAM image: zero-fill, then read & splice live regions ──
echo ""
echo "Allocating 4 MiB zero DRAM, then reading live regions..."

# Start with 4 MiB of zeros
dd if=/dev/zero of="$OUT_DRAM" bs=1M count=4 status=none

# Read function: read CHUNK words starting at byte ADDR, splice into DRAM image.
read_region() {
    local addr_hex="$1"
    local words="$2"
    local label="$3"
    # Convert hex addr to decimal (dd doesn't handle 0x prefix in seek)
    local addr_dec=$((addr_hex))
    echo "  Reading $label @ 0x$(printf '%X' $addr_dec) ($((words * 4)) bytes)..."
    /tmp/jcmd.sh "dump-mem $addr_hex $words" 2>&1 | \
      python3 -c "
import sys, re, struct
out = sys.stdout.buffer
for line in sys.stdin:
    m = re.search(r'= 0x([0-9a-fA-F]+)$', line.strip())
    if m:
        out.write(struct.pack('>I', int(m.group(1), 16)))
" > "/tmp/region_${label}.bin"
    local size=$(stat -c %s "/tmp/region_${label}.bin")
    echo "    Got $size bytes at offset $addr_dec"
    dd if="/tmp/region_${label}.bin" of="$OUT_DRAM" bs=1 seek=$addr_dec count=$size conv=notrunc status=none
    rm -f "/tmp/region_${label}.bin"
}

t0=$(date +%s)
# Live regions for mid-boot Mac OS Q700:
read_region 0x000000 65536 "low_ram_0"     # 0x000000-0x040000: vectors + low RAM (256KB)
read_region 0x040000 65536 "low_ram_1"
read_region 0x080000 65536 "low_ram_2"
read_region 0x0C0000 65536 "low_ram_3"
read_region 0x1F0000 16384 "stack"         # 0x1F0000-0x200000: supervisor stack (64KB)
read_region 0x3FC000  4096 "page_tables"   # 0x3FC000-0x400000: MMU tables (16KB)

ELAPSED=$(($(date +%s) - t0))
DRAM_SIZE=$(stat -c %s "$OUT_DRAM")
echo "DRAM image: $DRAM_SIZE bytes (live regions read in ${ELAPSED}s)"

# ── Repack into state_replay_v1 file ─────────────────────────────────
{
    echo "# state_replay_v1"
    grep -E "^STATE_" "$OUT_TXT" | tr ' ' '\n' | grep -E '^[A-Z][A-Z0-9_]*=' || true
    echo "DRAM"
    cat "$OUT_DRAM"
} > "$OUT_PACK"

PACK_SIZE=$(stat -c %s "$OUT_PACK")
echo "Wrote packed snapshot $OUT_PACK ($PACK_SIZE bytes)"
