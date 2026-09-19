#!/bin/bash
# Boot acceptance test for the "stable Finder boot" goal.
#
# Classification lessons this encodes, each of which cost real time today:
#  * exception-ring PC DIVERSITY is NOT a health signal -- a machine walking
#    DRAM filler still takes 60 Hz ticks at many distinct PCs and scored CLEAN
#    under the old harness. Classify on pc_live instead.
#  * `inst-count` prints in BOTH decimal and hex, so a [0-9]+ regex silently
#    reads 0 from the hex form and every trial looks frozen.
#  * `arch` needs an effective halt, so A7 reads empty while running: never
#    classify on it.
#  * a machine stuck in the ROM SysError loop still shows a healthy tick ring,
#    so detect a REPEATING pc_live across >= 3 samples.
#  * CurApName at 0x910 reads length 0 even on boots that reach the Finder, so
#    it is not a success test.
#  * JTAG polling visibly glitches scan-out: sample sparsely, never in a loop.
#
# Usage: ACCEPT_BIT=<path.bit> boot_acceptance.sh <cold|warm> <trials>
#
# ⚠️ ACCEPT_BIT IS MANDATORY for cold trials. The relaunch helper used to hardcode
# a bitstream path from an old campaign (/tmp/p170bit), so a "cold acceptance run"
# would reprogram THAT image and report its results as the new build's. It also
# used `pkill -9 -x vivado`, which matches every vivado on the machine and would
# destroy a multi-hour implementation run. Both fixed in tools/relaunch_repl.sh;
# prefer the in-repo copy over the /tmp one, which does not survive a reboot.
MODE=${1:-cold}; N=${2:-10}
RELAUNCH=${RELAUNCH:-$(dirname "$0")/relaunch_repl.sh}
[ -x "$RELAUNCH" ] || RELAUNCH=/tmp/relaunch_repl.sh
export ACCEPT_BIT ACCEPT_LTX ACCEPT_REPO
send() { echo "$1" > /tmp/jtag_in; sleep "${2:-3}"; }
grab() { tail -c +$(($1+1)) /tmp/jtag_out | grep -av "^##"; }
rr() { SZ=$(stat -c%s /tmp/jtag_out); send "r $1" 2; grab $SZ | grep -aoE "= 0x[0-9a-fA-F]{8}" | head -1 | cut -c3-; }
inwin() { local v=$((${1:-0})); [ $v -ge $((0x00900000)) ] && [ $v -le $((0x03FFFFFF)) ]; }
declare -i ok=0 filler=0 syserr=0 bomb=0 stuck=0 novideo=0
[ "$MODE" = "warm" ] && { "$RELAUNCH" > /dev/null 2>&1; for t in $(seq 1 100); do grep -aq "^> READY" /tmp/jtag_out && break; sleep 2; done; }
for k in $(seq 1 $N); do
  if [ "$MODE" = "cold" ]; then
    "$RELAUNCH" > /dev/null 2>&1
    for t in $(seq 1 100); do grep -aq "^> READY" /tmp/jtag_out && break; sleep 2; done
    SZ=$(stat -c%s /tmp/jtag_out); send "vio-read boot" 5
    grab $SZ | grep -aq "boot_error = 1" && { echo "trial $k: SD re-stage failed; recovering"; send "vio-hard-reset" 45; }
  fi
  send "break-pc off" 2; send "a7-odd-halt off" 1; send "pc-range-halt off" 1; send "halt-release" 2
  send "reset" 3; sleep 150
  P1=$(rr 0x50900010); sleep 9; P2=$(rr 0x50900010); sleep 9; P3=$(rr 0x50900010); sleep 9; P4=$(rr 0x50900010)
  # ⚠️ READ halt-kind FIRST. A self-halted core (ARBITER_WEDGE etc.) looks EXACTLY like
  # a stalled ROM loop through pc_live -- pinned PC, no progress -- so classifying on
  # pc_live alone silently reports a FUNCTIONAL halt as a timing stall. That cost a
  # whole session: 200 MHz boot failures were attributed to WNS and chased with
  # placement/passes/cell-removal, when all three failures were halt-kind 0x04.
  HK=$(SZ=$(stat -c%s /tmp/jtag_out); send "halt-kind" 5; grab $SZ | grep -aoE "halt-kind = 0x[0-9a-f]+ -> [A-Z_]+" | head -1)
  V=$(SZ=$(stat -c%s /tmp/jtag_out); send "video-status" 7; grab $SZ | grep -a "video source" | grep -aoE "dafb_live=[01]")
  U=$(printf "%s\n%s\n%s\n%s\n" "$P1" "$P2" "$P3" "$P4" | sort -u | grep -c .)
  SZ=$(stat -c%s /tmp/jtag_out); send "exc-ring" 12; RING=$(grab $SZ)
  VH=$(echo "$RING" | grep -aoE "vec=0x[0-9a-f]{2}" | sort | uniq -c | tr '\n' ' ')
  case "$P1$P2$P3$P4" in *408028??*|*408029??*|*40802a??*) SYS=1;; *) SYS=0;; esac
  if   [ "$SYS" = 1 ] && [ "$U" -le 2 ];               then C="SYSERROR-ALERT"; syserr+=1
  elif echo "$RING" | grep -aq "vec=0x04";             then C="BOMB(vec4)";     bomb+=1
  elif inwin "$P1" || inwin "$P2" || inwin "$P3" || inwin "$P4"; then C="FILLER-EXEC"; filler+=1
  elif [ "$U" -eq 1 ];                                 then C="STUCK";          stuck+=1
  elif [ "$V" != "dafb_live=1" ];                      then C="NO-VIDEO";       novideo+=1
  else C="ALIVE";                                      ok+=1; fi
  echo "$MODE $k: $C  $V distinctPC=$U/4  ${HK:-halt-kind=?}  pc=$P1 $P2 $P3 $P4"
  [ "$C" != "ALIVE" ] && { echo "     ring: $VH"; echo "$RING" | grep -aE "^> exc\[" | grep -avE "vec=0x(19|1a)" | head -3 | sed 's/^/     /' | cut -c1-100; }
done
echo "=== $MODE TALLY over $N: ALIVE=$ok FILLER=$filler SYSERROR=$syserr BOMB=$bomb STUCK=$stuck NO-VIDEO=$novideo ==="
echo "=== goal criterion 1 needs ALIVE=$N ==="
