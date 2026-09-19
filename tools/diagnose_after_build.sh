#!/bin/bash
# diagnose_after_build.sh — end-to-end: program the diagnostic bitstream, halt
# at the .ASYC00 driver's ioErr, and read the 187-bit probe that names WHICH
# scsi.v arm rejects READ(10) LBA 2314 and WHY.
#
# Runs the whole post-build sequence in one go so nothing is re-derived by hand:
#   1. relaunch the JTAG REPL on the new bitstream (it programs on startup)
#   2. verify DBG_VERSION != 0 -- a CPU-stub build passes every other check
#      (see .claude/skills/m68k-build-bitstream), so this is THE gate
#   3. break at 0x0000CFF6 (.ASYC00 ioErr site), identified via halt-status
#      `hit=`, NOT the live PC -- they differ and using `pc` misclassifies hits
#   4. read the probe and print the attribution
#
# Expect one of:
#   key 5 / ASC 0x21 -> scsi.v:2905 range check.  chk_ok/xfer_blocks/xfer_lba/
#                       num_lbas in the same sample say which operand is wrong.
#   key 2 / ASC 0x3A -> scsi.v:3001 or 3044, a backing-store timeout arm. These
#                       are the arms whose vh_wait_ctr accumulation bug was
#                       already fixed WITHOUT curing the boot, so it would be
#                       firing for a different reason.

cd "$(dirname "$0")/.." || exit 1
BIT=${BIT:-build/vivado/fpga_top.bit}
LTX=${LTX:-build/vivado/fpga_top.ltx}

if pgrep -x vivado >/dev/null 2>&1 && ! pgrep -f jtag_repl.tcl >/dev/null 2>&1; then
    echo "a build still holds Vivado -- wait for it to finish first"; exit 75
fi
[ -f "$BIT" ] || { echo "no bitstream at $BIT"; exit 1; }
echo "=== bitstream: $BIT ($(date -r "$BIT" +%H:%M)) ==="
cp -f "$BIT" build/vivado/build8_scsi_probe.bit 2>/dev/null
cp -f "$LTX" build/vivado/build8_scsi_probe.ltx 2>/dev/null

# (1) relaunch the REPL, which programs on startup
pgrep -f "exec 7>/tmp/jtag_in" >/dev/null || \
    nohup setsid bash -c 'exec 7>/tmp/jtag_in; sleep 999999' >/dev/null 2>&1 &
sleep 1
mv -f /tmp/jtag_out /tmp/jtag_out.prev 2>/dev/null
nohup setsid bash -c "/tools/Vivado/2025.2/Vivado/bin/vivado -mode tcl -nojournal -nolog \
  -source tools/jtag_repl.tcl -tclargs $BIT $LTX \
  < /tmp/jtag_in > /tmp/jtag_out 2>&1" >/dev/null 2>&1 &
echo "  programming (waiting up to 240s for the REPL) ..."
s=$SECONDS
while [ $((SECONDS-s)) -lt 240 ]; do
    grep -q "READY" /tmp/jtag_out 2>/dev/null && break
done

# (2) THE gate: a stub build passes everything else
DBG=$(JT_WAIT=30 tools/jt.sh "dbg-caps" 2>&1 | grep -oE 'DBG_VERSION = 0x[0-9A-Fa-f]+' | head -1)
echo "  $DBG"
case "$DBG" in
  *0x00000000|"") echo "  *** DBG_VERSION zero -> CPU STUB build. Rebuild with CPU=m68k. ***"; exit 2 ;;
esac
echo "  ROM: $(JT_WAIT=25 tools/jt.sh 'r 0x40800000' 2>&1 | grep -oE '0x[0-9A-Fa-f]{8}' | tail -1) (expect 0x420dbff3)"

# (3)+(4) halt at the driver ioErr and read the probe
echo
echo "=== catching the .ASYC00 ioErr and reading the sense probe ==="
WAIT=${WAIT:-270} MAXH=${MAXH:-6} tools/catch_driver_ioerr.sh 2>&1 | tail -40
echo
echo "=== probe decode ==="
tools/scsi_sense_probe.sh
