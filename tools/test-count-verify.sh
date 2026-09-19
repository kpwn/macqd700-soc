#!/usr/bin/env bash
# test-count-verify.sh — cross-check `make test` accounting against the
# source-of-truth (.s file list + deferred.txt) so silent-skip bugs can't
# creep back in.
#
# Usage:
#   tools/test-count-verify.sh           # run from repo root OR any worktree
#   tools/test-count-verify.sh /abs/path # override repo root
#
# Exits 0 if PASS + DEFER + SKIP + FAIL equals the total .s file count.
# Exits non-zero on discrepancy (including parser failures).
#
# The script assumes `make sim` has already been built — if not, a `make
# test` invocation will try to rebuild and may fail independently. For a
# full cold run from a fresh tree, use:
#
#   rsync -a --delete --exclude='.git' --exclude='build' \
#         --exclude='.claude/worktrees' ./ /tmp/m68k-ooo-build/
#   cd /tmp/m68k-ooo-build && MAKEFLAGS="-j1" make sim
#   tools/test-count-verify.sh

set -u

ROOT="${1:-$(pwd)}"
if [ ! -d "$ROOT/tb/tests/asm" ]; then
    echo "ERROR: $ROOT/tb/tests/asm not found — pass repo root as \$1" >&2
    exit 2
fi

ASM_DIR="$ROOT/tb/tests/asm"
DEF_FILE="$ROOT/tb/tests/deferred.txt"

# --- 1. count .s files
total_s=$(ls -1 "$ASM_DIR"/*.s 2>/dev/null | wc -l)
if [ "$total_s" -eq 0 ]; then
    echo "ERROR: no .s files under $ASM_DIR" >&2
    exit 2
fi

# --- 2. count deferred entries (lines, ignoring blanks + #-comments)
if [ -f "$DEF_FILE" ]; then
    deferred=$(grep -cE '^[[:space:]]*[^#[:space:]]' "$DEF_FILE" 2>/dev/null || echo 0)
else
    deferred=0
fi

# --- 3. run make test, capture summary line
tmplog=$(mktemp)
trap "rm -f $tmplog" EXIT

echo "[verify] repo root:    $ROOT"
echo "[verify] .s file count: $total_s"
echo "[verify] deferred.txt:  $deferred"
echo "[verify] running 'make test' (this can take a few minutes)..."

(cd "$ROOT" && make -j1 test) > "$tmplog" 2>&1
make_exit=$?

summary=$(grep -E '^summary: ' "$tmplog" | tail -1)
if [ -z "$summary" ]; then
    echo "ERROR: no 'summary:' line in make test output" >&2
    echo "--- last 40 lines of make test output ---" >&2
    tail -40 "$tmplog" >&2
    exit 2
fi

# Expected shape: "summary: PASS=N DEFER=M FAIL=K SKIP=J"
pass=$(echo   "$summary" | sed -nE 's/.*PASS=([0-9]+).*/\1/p')
defer=$(echo  "$summary" | sed -nE 's/.*DEFER=([0-9]+).*/\1/p')
fail=$(echo   "$summary" | sed -nE 's/.*FAIL=([0-9]+).*/\1/p')
skip=$(echo   "$summary" | sed -nE 's/.*SKIP=([0-9]+).*/\1/p')

: "${pass:=0}"
: "${defer:=0}"
: "${fail:=0}"
: "${skip:=0}"

echo "[verify] summary line:   $summary"
echo "[verify] make exit code: $make_exit"

total_run=$((pass + defer + fail + skip))

echo "[verify] PASS+DEFER+FAIL+SKIP = $pass + $defer + $fail + $skip = $total_run"
echo "[verify] expected total:       $total_s"

ok=1
if [ "$total_run" -ne "$total_s" ]; then
    echo "ERROR: accounting mismatch — $total_run != $total_s" >&2
    ok=0
fi

# Sanity: deferred count in .txt should be >= DEFER result (a deferred
# test might ALSO skip if it fails to assemble, so it's >=).  And FAIL
# should be 0 on a healthy main.
if [ "$defer" -gt "$deferred" ]; then
    echo "WARN: DEFER=$defer exceeds deferred.txt entries ($deferred) — investigate" >&2
fi

if [ "$ok" -eq 1 ]; then
    echo "[verify] OK"
    exit 0
else
    exit 1
fi
