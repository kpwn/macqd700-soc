#!/bin/bash
# jtag_lease.sh — mandatory mutual exclusion for the JTAG cable, with FIFO fairness.
#
# WHY THIS EXISTS
#   There is ONE JTAG cable and one REPL FIFO (/tmp/jtag_in). Two agents issuing
#   commands concurrently interleave their writes into that FIFO: replies get
#   attributed to the wrong requester, `sd-verify` output lands in someone else's
#   grep, and the board can be left programmed with a bitstream the other agent
#   did not expect. This has already produced wrong measurements and a wedged
#   board in this project. "Please don't touch the board" is not enforcement.
#
#   Ownership is per TEST, not per session: acquire, run ONE experiment, release.
#   Do not hold the lease across long analysis, disassembly, or sim runs.
#
# USAGE
#   tools/jtag_lease.sh acquire <agent-name> [ttl_s] [wait_s]
#   tools/jtag_lease.sh release <agent-name>
#   tools/jtag_lease.sh status
#   tools/jtag_lease.sh run <agent-name> <ttl_s> -- <command...>   # acquire, run, always release
#
#   ttl_s  default 900  — lease auto-expires so a dead agent cannot block forever.
#                         Pick > your experiment, < your patience. A boot+watchpoint
#                         run is ~300 s; a bitstream reload ~40 s.
#   wait_s default 3600 — how long to queue before giving up.
#
# FAIRNESS: strict FIFO by ticket. A waiting agent cannot be starved by one that
# repeatedly re-acquires — re-acquiring puts you at the BACK of the queue.
set -uo pipefail
D=${JTAG_LEASE_DIR:-/var/tmp/jtag_lease}
mkdir -p "$D" 2>/dev/null || true
LOCK="$D/.lock"; HOLDER="$D/holder"; QUEUE="$D/queue"
touch "$LOCK" "$QUEUE" 2>/dev/null || true

now () { date +%s; }

# Wait until the REPL is demonstrably idle AT ITS PROMPT before handing the
# lease over.  THE LEASE GUARDS THE CABLE, NOT THE REPL'S COMMAND QUEUE: a
# previous holder's commands live in /tmp/jtag_in and keep executing after that
# holder's process exits or its TTL expires, so a fresh holder can acquire a
# "free" lease and have its replies interleaved with the old holder's output.
#
# MEASURED 2026-08-08, and it cost a full board run: boot_clean.sh acquired a
# genuinely free lease, and three `ERROR watch: ... OFF_FEATURES bit 13` lines
# from the PREVIOUS holder drained into its sd-write-fast capture window. The
# write had actually SUCCEEDED and self-verified (95411 sectors, verify=on),
# but the error grep saw a foreign ERROR, the follow-on captures came back
# truncated, and the run aborted refusing to boot. A false negative produced
# entirely by someone else's output.
#
# Draining here means every holder inherits a quiet REPL instead of each tool
# having to defend itself. Bounded, and non-fatal: if the REPL never goes idle
# we warn and hand over anyway rather than deny the lease, because a wedged
# REPL is its own problem and the holder may be the one intending to fix it.
drain_repl () {
    local out=${JT_OUT:-/tmp/jtag_out} w=${JTAG_LEASE_DRAIN_S:-120} n0 n1 i
    [ -r "$out" ] || return 0
    for ((i=0;i<w*2;i++)); do
        n0=$(wc -l < "$out" 2>/dev/null || echo 0)
        sleep 0.5
        n1=$(wc -l < "$out" 2>/dev/null || echo 0)
        # NOT `tail -n1 == "> READY"`: Vivado appends async lines AFTER the
        # prompt (MIG "Calibration status change detected" after every
        # load-bit), so that predicate goes permanently false on a healthy
        # REPL. Idle = output stopped moving AND a prompt has been printed.
        [ "$n0" -eq "$n1" ] && grep -aq '^> READY$' "$out" 2>/dev/null && return 0
    done
    echo "WARNING: REPL still busy after ${w}s — handing over a NON-QUIET REPL." >&2
    echo "         Output from the previous holder may interleave with yours." >&2
    return 0
}

# holder file format: <name> <pid> <expiry_epoch>
#
# TTL-ONLY liveness, deliberately -- no `kill -0 "$hp"` check. This bit
# an investigation on 2026-08-28: agentic tool-call harnesses (this
# project's primary caller) spawn a FRESH subprocess per Bash tool call,
# so a standalone `acquire` (not wrapped in `run`) records a $PPID that
# is dead the instant that specific tool call returns -- even though the
# calling agent is still very much alive and using the lease across many
# SUBSEQUENT, independent tool calls. A `kill -0` check here made
# `status` falsely report the lease as expired seconds after a genuine
# acquire, letting a second agent acquire a "free" lease and interleave
# commands on the shared physical board mid-experiment -- exactly the
# failure mode this whole script exists to prevent. TTL expiry alone
# (already the documented "dead agent can't block forever" mechanism)
# is the only reliable staleness signal for this calling pattern; a
# genuinely-dead holder is still reaped, just no sooner than its own TTL.
holder_live () {
    [ -s "$HOLDER" ] || return 1
    read -r hn hp he < "$HOLDER" 2>/dev/null || return 1
    [ "$(now)" -lt "$he" ] 2>/dev/null || return 1   # expired
    return 0
}

cmd=${1:-status}
case "$cmd" in
acquire)
    name=${2:?agent-name}; ttl=${3:-900}; wait_s=${4:-3600}
    # Record the CALLER's pid (PPID), not ours: a standalone `acquire` returns
    # immediately, so $$ would be dead the instant we recorded it and `status`
    # would reap the lease as stale while the caller believed it held one.
    # `run` passes JTAG_LEASE_OWNER_PID so the wrapper's own pid is used instead.
    owner_pid=${JTAG_LEASE_OWNER_PID:-$PPID}
    tk="$(now).$$.$name"
    flock "$LOCK" -c "echo '$tk' >> '$QUEUE'"
    trap 'flock "$LOCK" -c "grep -vFx \"$tk\" \"$QUEUE\" > \"$QUEUE.t\" 2>/dev/null; mv -f \"$QUEUE.t\" \"$QUEUE\"" 2>/dev/null' EXIT
    deadline=$(( $(now) + wait_s ))
    while [ "$(now)" -lt "$deadline" ]; do
        got=$(flock "$LOCK" -c "
            if [ -s '$HOLDER' ]; then
                read -r hn hp he < '$HOLDER'
                if [ \$(date +%s) -lt \"\$he\" ]; then echo BUSY; exit 0; fi
            fi
            head=\$(head -1 '$QUEUE' 2>/dev/null)
            if [ \"\$head\" = '$tk' ]; then
                echo '$name $owner_pid '\$(( \$(date +%s) + $ttl )) > '$HOLDER'
                grep -vFx '$tk' '$QUEUE' > '$QUEUE.t' 2>/dev/null; mv -f '$QUEUE.t' '$QUEUE'
                echo GOT
            else echo WAIT; fi")
        if [ "$got" = GOT ]; then
            trap - EXIT
            # Drain BEFORE announcing: the caller starts issuing the moment we
            # return, so a quiet REPL has to be true at that instant.
            drain_repl
            echo "lease ACQUIRED by $name (owner pid $owner_pid, ttl ${ttl}s)"; exit 0
        fi
        sleep 3
    done
    echo "FATAL: could not acquire JTAG lease within ${wait_s}s"; exit 1 ;;

release)
    name=${2:?agent-name}
    flock "$LOCK" -c "
        if [ -s '$HOLDER' ]; then
            read -r hn hp he < '$HOLDER'
            if [ \"\$hn\" != '$name' ]; then echo \"REFUSED: lease held by \$hn, not $name\" >&2; exit 3; fi
        fi
        : > '$HOLDER'"
    rc=$?; [ $rc -eq 0 ] && echo "lease RELEASED by $name"; exit $rc ;;

status)
    if holder_live; then
        read -r hn hp he < "$HOLDER"
        echo "HELD by $hn (pid $hp), expires in $(( he - $(now) ))s"
    else
        [ -s "$HOLDER" ] && echo "FREE (stale holder reaped)" || echo "FREE"
    fi
    n=$(grep -c . "$QUEUE" 2>/dev/null); n=${n:-0}
    [ "$n" -gt 0 ] && { echo "queue ($n waiting):"; sed 's/^/  /' "$QUEUE"; }
    # ORPHANED-REPL CHECK (2026-09-02): the lease only tracks the HOLDER
    # bookkeeping file, not the actual jtag_repl.tcl process -- a REPL that
    # was never released, or whose /tmp/jtag_in|out got recreated out from
    # under it (a fresh REPL launch elsewhere unlinks-and-recreates those
    # paths; the old REPL keeps its now-dangling fds open), reports as a
    # healthy "FREE" lease while a real, unreachable-via-this-protocol
    # Vivado process is still sitting on the board. Found by hand once
    # (2026-09-02, a day-and-a-half-old orphaned REPL from a stale
    # worktree run, confused two concurrent sessions about board
    # ownership) -- catch it automatically from here on. Signature: a
    # jtag_repl.tcl process whose stdin/stdout fds resolve to "(deleted)"
    # targets, i.e. no longer the live /tmp/jtag_in|out inodes.
    for p in $(pgrep -f 'jtag_repl\.tcl' 2>/dev/null); do
        if ls -l "/proc/$p/fd/0" 2>/dev/null | grep -q '(deleted)'; then
            started=$(ps -o lstart= -p "$p" 2>/dev/null)
            echo "WARNING: pid $p (jtag_repl.tcl, started ${started:-unknown}) holds DELETED /tmp/jtag_in|out fds -- orphaned from the current FIFO, invisible to this lease, and will never receive commands. Verify it's unowned (check launch cwd/args via 'ps -o cmd= -p $p') before killing it." >&2
        fi
    done
    exit 0 ;;

run)
    name=${2:?agent-name}; ttl=${3:-900}; shift 3
    [ "${1:-}" = "--" ] && shift
    JTAG_LEASE_OWNER_PID=$$ "$0" acquire "$name" "$ttl" || exit 1
    # Export JTAG_LEASE_HELD so self-acquiring tools (boot_clean.sh,
    # sd_disk_diff.sh) INHERIT this lease instead of queueing behind it.
    # Without this, `run <name> -- boot_clean.sh` self-deadlocks: the child
    # waits for a lease its own parent holds. Measured 2026-08-08.
    export JTAG_LEASE_HELD="$name"
    trap '"$0" release "$name" >/dev/null 2>&1' EXIT
    "$@"; rc=$?
    "$0" release "$name" >/dev/null 2>&1; trap - EXIT
    exit $rc ;;

*) echo "usage: $0 {acquire|release|status|run} ..." >&2; exit 2 ;;
esac
