#!/usr/bin/env bash
# p141 -- (re)start the JTAG REPL under systemd, and wait until it is actually up.
#
#   tools/p141_start_repl.sh <bit> <ltx>
#
# WHY SYSTEMD AND NOT nohup/setsid. tools/diagnose_after_build.sh uses
# `nohup setsid ... &`, which is the documented pattern here and does usually
# work -- but this environment reaps an agent's whole process tree, and that has
# already silently killed long jobs on this host mid-run (a `place_design` died
# leaving "Parent process has died" and no artifact, while the launching shell
# still reported success). A `systemd-run --user` unit is genuinely outside the
# caller's tree, so the REPL survives the session that started it. That matters
# here because an A/B run is ~90 minutes of board time and losing the REPL
# halfway through costs the whole run.
#
# Two units, because the FIFO write end must outlive the starter too: if nothing
# holds /tmp/jtag_in open for writing, the REPL's stdin sees EOF and it exits.
#
#   m68k-jtag-fifo   holds the write end open
#   m68k-jtag-repl   the Vivado tcl REPL itself
#
# NOTE: `vivado -mode tcl` is NOT a build. It does not take, and must not be
# made to take, /var/tmp/m68k-ooo-vivado.lock -- and it must never be killed to
# free that lock, which is a mistake this project's own build script warns
# about. It can run concurrently with a synth/impl build.
set -u
cd "$(dirname "$0")/.." || exit 1

BIT=${1:?usage: $0 <bit> <ltx>}
LTX=${2:?usage: $0 <bit> <ltx>}
[ -f "$BIT" ] || { echo "FAIL: no such bitstream: $BIT"; exit 1; }
[ -f "$LTX" ] || { echo "FAIL: no such probes file: $LTX"; exit 1; }

VIVADO=${VIVADO:-/tools/Vivado/2025.2/Vivado/bin/vivado}

echo "== stopping any previous REPL units =="
systemctl --user stop m68k-jtag-repl 2>/dev/null
systemctl --user stop m68k-jtag-fifo 2>/dev/null
sleep 1

[ -p /tmp/jtag_in ] || { rm -f /tmp/jtag_in; mkfifo /tmp/jtag_in; }
mv -f /tmp/jtag_out /tmp/jtag_out.prev 2>/dev/null
: > /tmp/jtag_out

echo "== holding the FIFO write end =="
systemd-run --user --unit=m68k-jtag-fifo \
    --setenv=PATH="$PATH" --setenv=HOME="$HOME" \
    bash -c 'exec 7>/tmp/jtag_in; sleep 999999' >/dev/null 2>&1
sleep 1

# REATTACH MODE.  jtag_repl.tcl honours JTAG_REPL_NO_PROGRAM=1 (it re-attaches
# to a already-programmed device instead of loading $BIT), but this launcher had
# no way to reach it -- so the only way to recover a died-mid-session REPL was a
# reprogram, which wipes whatever the board was running.  That is exactly the
# wrong move when the REPL dies under a live OS.  Pass it through:
#     JTAG_REPL_NO_PROGRAM=1 tools/p141_start_repl.sh <bit> <ltx>
NOPROG=${JTAG_REPL_NO_PROGRAM:-0}
if [ "$NOPROG" = "1" ]; then
    echo "== starting the REPL (REATTACH: NOT programming, board state preserved) =="
else
    echo "== starting the REPL (programs $BIT on startup) =="
fi
systemd-run --user --unit=m68k-jtag-repl \
    --working-directory="$PWD" \
    --setenv=PATH="$PATH" --setenv=HOME="$HOME" \
    --setenv=JTAG_REPL_NO_PROGRAM="$NOPROG" \
    bash -lc "exec $VIVADO -mode tcl -nojournal -nolog \
        -source tools/jtag_repl.tcl -tclargs '$BIT' '$LTX' \
        < /tmp/jtag_in > /tmp/jtag_out 2>&1" >/dev/null 2>&1

echo "== waiting for the REPL to come up (up to 300s) =="
s=$SECONDS
while [ $((SECONDS - s)) -lt 300 ]; do
    # jtag_repl.tcl prints "axi master" once the debug bridge is usable; some
    # builds print READY. Accept either, and fail loudly on the known errors
    # rather than waiting out the full timeout on a dead link.
    if grep -qE "READY|axi master" /tmp/jtag_out 2>/dev/null; then
        echo "REPL up after $((SECONDS - s))s"
        tail -5 /tmp/jtag_out
        exit 0
    fi
    if grep -qiE "no hardware targets|cannot open|ERROR: \[Labtools" /tmp/jtag_out 2>/dev/null; then
        echo "FAIL: REPL reported a hardware/link error:"
        grep -iE "no hardware targets|cannot open|ERROR" /tmp/jtag_out | head -5
        exit 2
    fi
    sleep 5
done
echo "FAIL: REPL did not come up within 300s; last output:"
tail -20 /tmp/jtag_out
exit 3
