#!/bin/bash
# p137_wedge_characterise.sh — measure WHAT the p137 wedge is, not just where.
#
# p137 (cpu040 2db5bd3) parks at pc_live=0x40806b68 on every boot with the
# exception ring frozen.  Offline disassembly of files/420dbff3.rom says that
# address holds `a06e` -- a Slot Manager A-line trap, D0=0x2E -- inside the
# routine at 0x40806b50, and `pc_live` is the NEXT pc after the last retired
# macro (RobPlugin.scala:1428-1449, commitPc = predNextPc/nextPc).  So the
# machine's last retired instruction is `moveq #46,%d0` at 0x40806b66 and the
# A-line at 0x40806b68 has NEVER retired.
#
# WHAT THIS SCRIPT ADDS, and why each measurement is the one that discriminates:
#
#   1. RETIRED-MACRO counter (OFF_INST_LO/HI at 0x50901008/0x5090100C).  LIVE,
#      free-running, monotonic, needs no halt -- unlike `inst-count`, which is
#      halt-captured and lies.  A frozen value means the CPU retires NOTHING,
#      which separates a wedged pipeline from a machine spinning in a loop.
#
#      DO NOT ADD THE CYCLE COUNTER HERE.  OFF_CYCLE_LO/HI (0x50901000/0x1004)
#      are DECLARED in DebugRegMap.scala:97-98 and NEVER IMPLEMENTED in
#      DebugCtrlPlugin's read mux: they read 0x00000000 on a perfectly healthy
#      running CPU.  An earlier revision of this script read them and printed
#      "VERDICT clock: STOPPED" for a board whose fabric was demonstrably alive
#      (every other JTAG read in the same run succeeded).  That is a fabricated
#      result, not a measurement.
#
#      The LO half wraps roughly every 86-170 s, so a single frozen LO proves
#      nothing; three samples 30 s apart do, since aliasing would need the wrap
#      period to divide 30 s exactly.  HI is read alongside for the same reason.
#   2. exc-ring head before and after the window -- the trustworthy liveness
#      instrument.  A frozen head means no exception of any kind is taken.
#   3. `halt`, with its landing time.  A halt that lands proves the machine is at
#      a coherent macro boundary with nothing in flight.  A halt that TIMES OUT
#      is itself a result (RobPlugin's `stoppedNext` is gated on
#      `!interruptPending`), so a failure here is captured, not fatal.
#   4. Only if the halt lands: SR (the interrupt mask -- masked, or simply not
#      retiring?), VBR, A7, and the vector-10 (A-line) table entry at VBR+0x28,
#      read with `coherent-dump` because a plain JTAG read bypasses the D-cache.
#
# ARMS NOTHING.  Disarm is `halt-exc-mask raw <lane> 0` across all eight lanes --
# NOT `halt-exc-mask 0`, which is the SET form and arms vector 0 -- and the
# enables line is ASSERTED afterwards rather than assumed.
#
# SETTLE >= 180 s.  At 150 s this board is often still booting and the ROM's ~15
# legitimate SCSI-probe bus errors read as a terminal failure.
#
# Usage:  BIT=/path/fpga_top.bit LTX=/path/fpga_top.ltx WANT_ID=0x9c1fa4b5 \
#           NAME=p137 SETTLE=200 tools/p137_wedge_characterise.sh
set -u
cd "$(dirname "$0")/.." || exit 1

BIT=${BIT:?set BIT}
LTX=${LTX:?set LTX}
WANT_ID=${WANT_ID:?set WANT_ID (the build_id this bitstream must report live)}
NAME=${NAME:-arm}
SETTLE=${SETTLE:-200}
SNAP=${SNAP:-http://10.200.0.12:8080/snapshot.jpg}
OUT=${OUT:-/tmp/wedge_$NAME}
mkdir -p "$OUT"

jt () { JT_WAIT=${2:-15} JT_TAG="wedge-$NAME" tools/jt.sh "$1" 2>&1; }

echo "=== characterise $NAME  ($(date -Is)) ==="
echo "bit: $BIT"

jt "load-bit $BIT $LTX" 200 | grep -E "programmed|ERROR" | sed 's/^/  /'
sleep 5

# A first build-id read after load-bit can return 0x00000000 (documented
# read-before-CSR-reset race).  Read twice and use the second.
jt "build-id" 30 >/dev/null; sleep 2
live=$(jt "build-id" 30 | grep -oiE 'build_id = 0x[0-9a-f]+' | head -1 | grep -oiE '0x[0-9a-f]+')
echo "  live build_id: ${live:-UNKNOWN}  (want $WANT_ID)"
if [ "${live,,}" != "${WANT_ID,,}" ]; then
    echo "  ABORT: live build_id does not match the requested arm."
    exit 1
fi

for c in "break-pc off" "halt-clear" "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off"; do
    jt "$c" 10 >/dev/null 2>&1
done
for lane in 0 1 2 3 4 5 6 7; do jt "halt-exc-mask raw $lane 0" 8 >/dev/null 2>&1; done
st=$(jt "halt-status" 10)
if printf '%s' "$st" | grep -q 'enables={ha=0 bp=0 exc=0 pcmis=0}'; then
    echo "  disarm verified: enables all zero"
else
    echo "  WARNING: something is still armed:"
    printf '%s' "$st" | grep -oE 'enables=\{[^}]*\}' | sed 's/^/    /'
fi

jt "reset" 40 | grep -E "reset done" | sed 's/^/  /'
echo "  settling ${SETTLE}s..."
sleep "$SETTLE"

rd () { jt "r $1" 15 | grep -oiE '0x[0-9a-f]{8}' | tail -1; }

echo
echo "--- liveness window ---"
head0=$(jt "exc-ring" 20 | grep -oE 'head=[0-9]+' | head -1 | cut -d= -f2)
mac0=$(rd 0x50901008); mach0=$(rd 0x5090100C)
pc0=$(jt "halt-status" 10 | grep -oE 'pc_live=0x[0-9a-f]+' | head -1 | sed 's/pc_live=//')
echo "  t0  exc_head=$head0  retired_macros=$mach0:$mac0  pc_live=$pc0"
sleep 30
mac1=$(rd 0x50901008); mach1=$(rd 0x5090100C)
pc1=$(jt "halt-status" 10 | grep -oE 'pc_live=0x[0-9a-f]+' | head -1 | sed 's/pc_live=//')
head1=$(jt "exc-ring" 20 | grep -oE 'head=[0-9]+' | head -1 | cut -d= -f2)
echo "  t30 exc_head=$head1  retired_macros=$mach1:$mac1  pc_live=$pc1"
sleep 30
mac2=$(rd 0x50901008); mach2=$(rd 0x5090100C)
pc2=$(jt "halt-status" 10 | grep -oE 'pc_live=0x[0-9a-f]+' | head -1 | sed 's/pc_live=//')
head2=$(jt "exc-ring" 20 | grep -oE 'head=[0-9]+' | head -1 | cut -d= -f2)
echo "  t60 exc_head=$head2  retired_macros=$mach2:$mac2  pc_live=$pc2"

echo
echo "  VERDICT retire:   macros $mac0 -> $mac2  $( [ "$mac0" != "$mac2" ] && echo ADVANCING || echo FROZEN )"
echo "  VERDICT exc-ring: head   $head0 -> $head2  $( [ "$head0" != "$head2" ] && echo ADVANCING || echo FROZEN )"

curl -s -m 20 -o "$OUT/${NAME}.jpg" "$SNAP" 2>/dev/null
echo "  screen md5: $(md5sum "$OUT/${NAME}.jpg" 2>/dev/null | cut -d' ' -f1)"

echo
echo "--- exc-ring (top 8) ---"
jt "exc-ring" 25 2>&1 | head -9 | sed 's/^/  /'

echo
echo "--- wedge-status ---"
jt "wedge-status" 20 2>&1 | head -6 | sed 's/^/  /'

echo
echo "--- halt (does it land?) ---"
hout=$(jt "halt 3000" 40)
printf '%s\n' "$hout" | head -4 | sed 's/^/  /'
if printf '%s' "$hout" | grep -q "halt landed"; then
    echo "  HALT LANDED -- architectural state is coherent, reading it."
    jt "dcache-op push" 60 2>&1 | head -2 | sed 's/^/  /'
    echo "  --- live-arch ---"
    la=$(jt "live-arch" 40)
    printf '%s\n' "$la" | grep -E '^> *(D[0-7]|A[0-7]|SR|VBR|PC|USP|ISP|MSP)' | sed 's/^/  /'
    vbr=$(printf '%s' "$la" | grep -oiE 'VBR *= *0x[0-9a-f]+' | head -1 | grep -oiE '0x[0-9a-f]+')
    echo "  VBR=$vbr"
    if [ -n "${vbr:-}" ]; then
        v10=$(printf '0x%08X' $(( $(printf '%d' "$vbr") + 0x28 )))
        echo "  --- vector 10 (A-line) table entry at VBR+0x28 = $v10 (coherent) ---"
        jt "coherent-dump $v10 2" 30 2>&1 | sed 's/^/  /'
    fi
    echo "  --- 16 words at the wedge PC (coherent) ---"
    jt "coherent-dump 0x40806B50 16" 30 2>&1 | sed 's/^/  /'
else
    echo "  HALT DID NOT LAND -- that is itself a result (RobPlugin stoppedNext is"
    echo "  gated on !interruptPending; a pending, un-takeable IRQ blocks the stop)."
fi

echo
echo "--- release ---"
jt "halt-release" 20 2>&1 | head -2 | sed 's/^/  /'
echo "=== characterise $NAME COMPLETE ($(date -Is)) ==="
