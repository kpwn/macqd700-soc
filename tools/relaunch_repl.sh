#!/bin/bash
# Cold-boot helper for tools/boot_acceptance.sh: reprogram the FPGA and relaunch
# the JTAG REPL.
#
# ⚠️ TWO BUGS FIXED 2026-09-14, both of which had teeth:
#  1. It hardcoded /tmp/p170bit/fpga_top.bit -- a bitstream from an OLD campaign.
#     Every "cold boot acceptance run" would have tested that image, not the one
#     under test, and reported its results as if they were the new build's.
#  2. It used `pkill -9 -x vivado`, which matches EVERY vivado on the machine --
#     including a multi-hour implementation run. A cold acceptance sweep would
#     have silently destroyed any build in progress.
#
# Now: the bitstream is an env var, and only the REPL is killed (matched on
# jtag_repl in the command line, never by bare process name).
BIT=${ACCEPT_BIT:?set ACCEPT_BIT to the .bit under test}
LTX=${ACCEPT_LTX:-${BIT%.bit}.ltx}
REPO=${ACCEPT_REPO:-/home/qwertyoruiop/macqd700-soc-worktrees/eth200}
[ -f "$BIT" ] || { echo "relaunch_repl: no such bitstream: $BIT" >&2; exit 1; }
cd "$REPO" || exit 1
# kill ONLY jtag_repl vivado processes -- never a build
for p in /proc/[0-9]*; do
    pid=${p#/proc/}
    [ "$pid" = "$$" ] && continue
    exe=$(readlink "/proc/$pid/exe" 2>/dev/null) || continue
    cl=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)
    case "$exe:$cl" in
        *vivado*jtag_repl*) kill -9 "$pid" 2>/dev/null ;;
        *cs_server*)        kill -9 "$pid" 2>/dev/null ;;
    esac
done
sleep 3
: > /tmp/jtag_out
[ -p /tmp/jtag_in ] || mkfifo /tmp/jtag_in
setsid nohup /tools/Vivado/2025.2/Vivado/bin/vivado -mode tcl -nojournal -nolog \
    -source tools/jtag_repl.tcl -tclargs "$BIT" "$LTX" \
    < /tmp/jtag_in > /tmp/jtag_out 2>&1 &
echo "relaunched pid $! with $BIT"
