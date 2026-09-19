#!/bin/bash
# csc21_send.sh — hardened single-command sender for tools/jtag_repl.tcl,
# built to close a real, reproduced race that cost Part 64 of
# docs/BUG_calibration_word_misplaced_0d00.md two of its three confirmed
# csCode-21-probe hits' outcome data.
#
# ROOT CAUSE THIS REPLACES (see Part 65 for the full writeup): Part 64's own
# ad hoc sender (`jtag_send.sh`, scratchpad-only, never committed) detected
# "my command's response is complete" by polling a shared, ever-growing log
# file for the FIRST appearance of the generic string "> READY" among the
# lines appended since a snapshot line-count taken right before the send.
# `jtag_repl.tcl` prints "> READY" after EVERY command with no correlation
# ID, so under load (a load-bit's own asynchronous trailing MIG-calibration
# message; a REPL that is simply slower than the caller's timeout expects)
# TWO different failure shapes both silently corrupt classification:
#   1. The naive sender gives up on ITS OWN timeout, but by then it has
#      ALREADY written the next command into the queue (the write to
#      cmds.txt/the FIFO already happened; only the wait for a reply timed
#      out) -- so the REPL is left processing a backlog while the CALLER has
#      already moved on to send yet more commands, and a later call's
#      "lines since my snapshot" window ends up containing a mix of a
#      earlier command's late-arriving output and its own real response.
#   2. Even the field-matching bug Part 64 first found (grepping the wrong
#      case, `hit=0x40800BF0` vs the REPL's actual lowercase
#      `hit=0x40800bf0`) is a symptom of the same underlying design flaw:
#      there is no unambiguous, collision-proof boundary marking "this
#      output belongs to THIS command and nothing else."
#
# FIX: after writing the real command to the FIFO, ALSO write a second,
# uniquely-tagged no-op command (`tcl set _csc21mark <random-tag>`, using
# jtag_repl.tcl's own generic Tcl passthrough -- see its `tcl` case). Poll
# the output log for THAT EXACT, single-use tag's own echo
# (`> tcl: <random-tag>`) rather than a reusable generic string. Because the
# tag is fresh and unique per call, a stale/late arrival from a PRIOR call
# can never satisfy a LATER call's wait, and because `jtag_repl.tcl`
# processes stdin strictly in order (a single `gets stdin line` loop), our
# own tag appearing in the log is proof the REPL has fully finished
# processing everything queued before it, INCLUDING our real command --
# however long that took. This holds even under a real backlog: a caller
# simply waits longer, it does not misattribute someone else's output.
#
# The whole operation (send real command, send marker, wait for marker) is
# done under ONE flock hold on the same lock file tools/jt.sh uses, so no
# other writer (a human at the REPL, another agent, the MJPEG panel) can
# interleave a command into OUR window either.
#
# Usage: csc21_send.sh "<command>" [timeout_s]
#   Prints the real command's own output (READY markers and the trailing
#   `tcl: <tag>` echo stripped) on success (exit 0).
#   On timeout: prints "!!! CSC21_SEND_TIMEOUT waiting for tag <tag> after:
#   <command>" to stderr, prints the literal sentinel line
#   `CSC21_SEND_TIMEOUT` to stdout, and exits 1. Callers MUST treat this
#   sentinel as "unknown / not classified", never as a MISS -- conflating
#   "the send timed out" with "the probe was missed" is exactly the bug
#   class this script exists to eliminate.
set -uo pipefail

FIFO=${FPGA_JTAG_FIFO:-/tmp/jtag_in}
OUT=${FPGA_JTAG_OUT:-/tmp/jtag_out}
LOCK=${FPGA_JTAG_LOCK:-/tmp/jtag_in.lock}

CMD="${1:?usage: csc21_send.sh <command> [timeout_s]}"
TIMEOUT="${2:-60}"

if [ ! -p "$FIFO" ]; then
    echo "csc21_send.sh: $FIFO is not a FIFO -- is the REPL set up? (see tools/diagnose_after_build.sh's startup recipe)" >&2
    exit 2
fi
[ -r "$OUT" ] || { echo "csc21_send.sh: cannot read $OUT" >&2; exit 2; }

TAG="CSC21MARK_$$_${RANDOM}${RANDOM}_$(date +%s%N)"

exec 9>>"$LOCK" || { echo "csc21_send.sh: cannot open lock $LOCK" >&2; exit 2; }
flock -w 600 9 || { echo "csc21_send.sh: JTAG busy (600s wait exceeded)" >&2; exit 75; }

START_BYTES=$(stat -c %s "$OUT" 2>/dev/null || echo 0)

{
    printf '%s\n' "$CMD"
    printf 'tcl set _csc21mark %s\n' "$TAG"
} > "$FIFO"

DEADLINE=$((SECONDS + TIMEOUT))
FOUND=0
while [ $SECONDS -lt $DEADLINE ]; do
    if tail -c +$((START_BYTES + 1)) "$OUT" 2>/dev/null | grep -qF "> tcl: $TAG"; then
        FOUND=1
        break
    fi
    sleep 0.3
done

if [ "$FOUND" -eq 1 ]; then
    # Everything from just after our send-point up to (excluding) our own
    # marker's echo line is unambiguously OUR response, however much async
    # noise or backlog-delay is mixed in with it.
    tail -c +$((START_BYTES + 1)) "$OUT" 2>/dev/null \
        | grep -vF "> tcl: $TAG" \
        | grep -v "^> READY$"
    exit 0
else
    echo "!!! CSC21_SEND_TIMEOUT (${TIMEOUT}s) waiting for tag $TAG after: $CMD" >&2
    echo "CSC21_SEND_TIMEOUT"
    exit 1
fi
