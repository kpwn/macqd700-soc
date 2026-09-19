#!/bin/bash
# Send one command to the JTAG REPL and print the output it produced.
#
# WHY THIS EXISTS RATHER THAN `echo cmd > /tmp/jtag_in`:
#
# The REPL's command FIFO has several independent writers -- a human here, an
# agent at another shell, and the MJPEG control panel at :8080.  A line
# injected into the middle of someone else's long operation does NOT fail
# loudly; the REPL simply consumes it as the next command, corrupting whatever
# was in flight.  That is not hypothetical: one button press on the web panel
# during a 2-minute bulk SD write aborted it with a CMD18 verify timeout.
#
# So every writer takes an flock(2) advisory lock on $JTAG_LOCK for the WHOLE
# duration of the command -- not just the instant of the FIFO write, because
# the REPL keeps executing long after the line has been consumed.  We hold it
# until the output goes quiet.
#
# Unlike the web panel (which refuses immediately, since a click that lands
# 90 s later is worse than one that says "busy"), this waits: a shell caller
# is usually scripting a sequence and wants it to complete.
#
# Every command is also appended to $JTAG_LOG.  The lock is ADVISORY -- a
# writer that ignores it still gets through -- so the log is the backstop that
# makes an interleave visible after the fact instead of silently corrupting a
# result you then spend an hour explaining.
#
# Usage:  JT_WAIT=30 tools/jt.sh "<repl command>"
#   JT_WAIT       seconds to wait for output to go quiet (default 30)
#   JT_LOCK_WAIT  seconds to wait for the lock (default 600; 0 = fail fast)
#   JT_TAG        label recorded in the lock file and log (default $USER-shell)

FIFO=${FPGA_JTAG_FIFO:-/tmp/jtag_in}
OUT=${FPGA_JTAG_OUT:-/tmp/jtag_out}
LOCK=${FPGA_JTAG_LOCK:-/tmp/jtag_in.lock}
LOG=${FPGA_JTAG_LOG:-/tmp/jtag_cmd.log}
W=${JT_WAIT:-30}
LW=${JT_LOCK_WAIT:-600}
TAG=${JT_TAG:-${USER:-unknown}-shell}

if [ ! -p "$FIFO" ]; then
    echo "jt.sh: $FIFO is not a FIFO — is the REPL set up?" >&2
    exit 2
fi

exec 9>>"$LOCK" || { echo "jt.sh: cannot open lock $LOCK" >&2; exit 2; }

if ! flock -w "$LW" 9; then
    echo "jt.sh: JTAG busy — held by $(cat "$LOCK" 2>/dev/null || echo unknown)" >&2
    printf '%s pid=%s origin=%s REFUSED(busy) %s\n' \
        "$(date -Is)" "$$" "$TAG" "$*" >> "$LOG" 2>/dev/null
    exit 75
fi

# Record who holds it, so a refusal elsewhere can name us.
: > "$LOCK"
printf '%s %s' "$$" "$TAG" >> "$LOCK"
printf '%s pid=%s origin=%s %s\n' "$(date -Is)" "$$" "$TAG" "$*" >> "$LOG" 2>/dev/null

S=$(stat -c %s "$OUT" 2>/dev/null || echo 0)
echo "$*" > "$FIFO"

# Wait for the output file to stop growing: the command has gone quiet.
prev=-1
for _ in $(seq 1 "$W"); do
    sleep 1
    cur=$(stat -c %s "$OUT" 2>/dev/null || echo 0)
    [ "$cur" != "$S" ] && [ "$cur" = "$prev" ] && break
    prev=$cur
done

tail -c +$((S + 1)) "$OUT" 2>/dev/null
# Lock releases when fd 9 closes at exit.
