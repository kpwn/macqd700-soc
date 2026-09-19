#!/bin/bash
# test_jt_dispatch.sh — board-free regression test for the JTAG-REPL command
# dispatch used by tools/sd_disk_diff.sh and tools/boot_clean.sh.
#
# WHAT THIS PINS DOWN
#   Those scripts talk to a long-running Vivado JTAG REPL through a FIFO and
#   have to answer one question: "has my command finished?".  They used to
#   answer it with "the output file stopped growing for 1.0 s".  `sd-verify`
#   emits a progress line every 4096 sectors, and at its MEASURED 1649-1816
#   KiB/s that is a gap of 1.13-1.24 s — just above the detector's threshold.
#   The result was captures truncated at random points, which manufactured
#   three separate phantom findings on 2026-08-08:
#
#     * "the diff does not enumerate all damaged batches in one pass"
#     * "a single restore pass silently does not stick"
#     * `repair` livelocking on the same batch set every round
#
#   and, worse, commands that were dequeued by the REPL only AFTER the calling
#   script had exited and its EXIT trap had deleted their temp file, producing
#   `ERROR sd-write-fast: file not found` into the NEXT command's capture
#   window where nobody looked.
#
#   Test 1 below is the RED test for that class: it drives a command whose
#   own output contains a gap LONGER than any quiet-based threshold, and
#   requires the full capture.  It fails against the old implementation and
#   passes against the sentinel one.  Do not replace the sentinel with a
#   quiet-based heuristic; any threshold is a race against bursty output.
#
# NO HARDWARE REQUIRED — a fake REPL stands in for Vivado.

set -uo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)

PASS=0
FAIL=0
ok   () { echo "  PASS  $*"; PASS=$((PASS+1)); }
bad  () { echo "  FAIL  $*"; FAIL=$((FAIL+1)); }

WORK=$(mktemp -d /tmp/jtdispatch.XXXXXX) || { echo "mktemp failed"; exit 2; }
export JT_IN="$WORK/in"
export JT_OUT="$WORK/out"
mkfifo "$JT_IN"
: > "$JT_OUT"

# ── Fake REPL ────────────────────────────────────────────────────────────────
# Mimics tools/jtag_repl.tcl's observable contract: every command ends with a
# "> READY" prompt line, and output between commands can be arbitrarily bursty.
fake_repl () {
    # Hold the FIFO open for writing ourselves so it never sees EOF.
    exec 9<>"$JT_IN"
    echo "> READY" >> "$JT_OUT"
    while IFS= read -r line <&9; do
        case "$line" in
            quit) return 0 ;;
            slow*)
                # Three lines separated by gaps WIDER than any plausible
                # output-quiet threshold, then the summary and the prompt.
                for i in 1 2 3; do
                    sleep 1.6
                    echo "> slow: progress $i" >> "$JT_OUT"
                done
                sleep 1.6
                echo "> slow done: 3 steps" >> "$JT_OUT"
                ;;
            err*)
                echo "> ERROR fake: something went wrong" >> "$JT_OUT"
                ;;
            *)
                echo "> ok: $line" >> "$JT_OUT"
                ;;
        esac
        echo "> READY" >> "$JT_OUT"
    done
}

fake_repl &
REPL_PID=$!
cleanup () { kill "$REPL_PID" 2>/dev/null; rm -rf "$WORK"; }
trap cleanup EXIT

# Wait for the fake REPL's initial prompt.
for _ in $(seq 1 50); do
    grep -q '^> READY$' "$JT_OUT" 2>/dev/null && break
    sleep 0.1
done

# Pull in the REAL dispatch functions (not a copy that could drift).
# JT_LIB is overridable so the RED demonstration can point this same test at
# the pre-fix implementation:  JT_LIB=/tmp/old_jt.sh bash tools/tests/test_jt_dispatch.sh
# shellcheck disable=SC1090
JT_LIB_ONLY=1 . "${JT_LIB:-$ROOT/tools/sd_disk_diff.sh}"

echo "== test_jt_dispatch =="

# ── Test 1 — RED against the old quiet-based dispatch ────────────────────────
# A command whose output has 1.6 s internal gaps must still be captured whole.
out=$(JT_WAIT=60 jt "slowcmd")
if printf '%s\n' "$out" | grep -q 'slow done: 3 steps'; then
    ok "bursty output: captured the command's final summary line"
else
    bad "bursty output: capture truncated before the summary (got: $(printf '%s' "$out" | tr '\n' '|'))"
fi
got=$(printf '%s\n' "$out" | grep -c 'slow: progress')
if [ "$got" -eq 3 ]; then
    ok "bursty output: captured all 3 progress lines"
else
    bad "bursty output: captured $got/3 progress lines"
fi

# ── Test 2 — the capture must not bleed into the next command ────────────────
out2=$(JT_WAIT=60 jt "hello")
if printf '%s\n' "$out2" | grep -q 'ok: hello' && \
   ! printf '%s\n' "$out2" | grep -q 'slow:'; then
    ok "no cross-talk: second command's capture contains only its own output"
else
    bad "no cross-talk: second capture polluted (got: $(printf '%s' "$out2" | tr '\n' '|'))"
fi

# ── Test 3 — a command issued back-to-back after a slow one must still run ───
# This is the failure that deleted temp files out from under queued commands:
# the dispatcher returned early, the caller cleaned up, then the REPL ran the
# command against a file that was already gone.
( JT_WAIT=60 jt "slowcmd" >/dev/null ) &
SLOW_PID=$!
sleep 0.5
out3=$(JT_WAIT=60 jt "second")
wait "$SLOW_PID"
if printf '%s\n' "$out3" | grep -q 'ok: second'; then
    ok "serialisation: a command issued during a slow one still gets its own reply"
else
    bad "serialisation: second command's reply missing (got: $(printf '%s' "$out3" | tr '\n' '|'))"
fi

# ── Test 4 — jt_checked must abort on a REPL-reported error ──────────────────
# Run in a subshell because jt_checked calls die(), which exits.
if ( JT_WAIT=60 jt_checked "errcmd" >/dev/null 2>&1 ); then
    bad "error detection: jt_checked returned success on an > ERROR reply"
else
    ok "error detection: jt_checked failed on an > ERROR reply"
fi

# A clean command must NOT trip the error check.
if ( JT_WAIT=60 jt_checked "fine" >/dev/null 2>&1 ); then
    ok "error detection: jt_checked passes a clean reply"
else
    bad "error detection: jt_checked wrongly failed a clean reply"
fi

# ── Test 5 — clean sd-verify summaries are not mismatch records ─────────────
clean_verify='> sd-verify done: 1 sectors from lba=8192 in 0.0 s — 0 MISMATCHING batches'
dirty_verify='> sd-verify: MISMATCH lba=8192..8192 (card=0x1 image=0x2)'
if printf '%s\n' "$clean_verify" | sd_verify_has_mismatch; then
    bad "mismatch parsing: clean summary reported as damaged"
else
    ok "mismatch parsing: clean summary is not a mismatch record"
fi
if printf '%s\n' "$dirty_verify" | sd_verify_has_mismatch; then
    ok "mismatch parsing: actual mismatch record detected"
else
    bad "mismatch parsing: actual mismatch record missed"
fi

echo "== $PASS passed, $FAIL failed =="
[ "$FAIL" -eq 0 ]
