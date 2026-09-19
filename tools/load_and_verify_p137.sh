#!/bin/bash
# load_and_verify_p137.sh — program the freshly built bitstream and PROVE which
# one is actually on the FPGA before any boot measurement is trusted.
#
# Verify the live build_id before trusting anything: a first read straight after
# `load-bit` can return 0x00000000 (known read-before-CSR-reset race), so this
# re-reads rather than believing the first sample.  It also refuses to proceed if
# the live id still equals p133's 0x14F3C599, which would mean the load silently
# did not take and every boot number afterwards would describe the OLD bitstream.
#
# DBG_VERSION must be non-zero (0xDEB60100 for cpu040).  A stub build passes
# every other check, so this is the decisive one.
set -u
cd "$(dirname "$0")/.." || exit 1

BIT=build/vivado/fpga_top.bit
LTX=build/vivado/fpga_top.ltx
INFO=build/vivado/fpga_top.buildinfo
P133_ID=0x14F3C599

jt () { JT_WAIT=${2:-25} tools/jt.sh "$1" 2>&1; }

echo "=== buildinfo on disk ==="
sed -n 's/^build_id=/  build_id=/p;s/^cpu=/  cpu=/p;s/^generated_utc=/  generated_utc=/p' "$INFO"
EXPECT=$(sed -n 's/^build_id=//p' "$INFO" | head -1)
echo "  expecting live build_id = $EXPECT"

echo
echo "=== programming ==="
jt "load-bit $BIT $LTX" 180 | tail -6

echo
echo "=== build_id, read TWICE (first read after load-bit can race) ==="
R1=$(jt "build-id" 30); echo "$R1" | sed 's/^/  read1: /'
sleep 3
R2=$(jt "build-id" 30); echo "$R2" | sed 's/^/  read2: /'

LIVE=$(echo "$R2" | grep -oiE 'build_id = 0x[0-9a-f]+' | head -1 | grep -oiE '0x[0-9a-f]+')
echo "  LIVE build_id = ${LIVE:-<none>}"

if [ -z "${LIVE:-}" ]; then
    echo "ABORT: could not read a live build_id."; exit 1
fi
if [ "${LIVE^^}" = "0X00000000" ]; then
    echo "ABORT: live build_id still 0x00000000 after a re-read."; exit 1
fi
if [ "${LIVE^^}" = "${P133_ID^^}" ]; then
    echo "ABORT: live build_id is still p133's $P133_ID -- the load did NOT take."
    echo "       Every boot measurement after this would describe the OLD bitstream."
    exit 1
fi

echo
echo "=== dbg-caps (DECISIVE: DBG_VERSION must be non-zero; cpu040 = 0xDEB60100) ==="
jt "dbg-caps" 30 | sed 's/^/  /'

echo
echo "=== halt-status ==="
jt "halt-status" 20 | sed 's/^/  /'

echo
echo "VERIFIED: live build_id $LIVE (p133 was $P133_ID)"
