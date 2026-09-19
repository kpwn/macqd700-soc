#!/bin/bash
# catch_syserror.sh — halt on _SysError ($A9C9) and identify WHO raised it.
#
# Established 2026-08-18: our board raises SysError(88) where MAME (same 7.5.3
# image) never raises it at all.  The Finder's $A9C9 patch then walks its
# registered-A5-world table, legitimately matches the Finder's own A5, sets its
# error flag at [A5-3692] and hits _Debugger ($A9FF).  The SetHandleSize(h,-24)
# grow-zone thrash that wedges the machine is downstream of THAT.
#
# So the A5 table / "duplicate registration" / -24 chain is the error REPORTING
# path, not the fault.  The fault is whatever calls SysError.  This halts at the
# trap itself -- atrap matches the OPCODE WORD, so it is immune to the Finder
# relocating each boot -- and dumps the caller's return address off the stack.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-420}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }

echo "=== arming atrap on _SysError (\$A9C9) ==="
jt "watch 0 off" >/dev/null; jt "watch 1 off" >/dev/null
jt "break-pc off" >/dev/null; jt "atrap 1 off" >/dev/null
jt "halt-clear" >/dev/null
jt "atrap 0 0xA9C9" 25 | grep -E "armed|ERROR" | sed 's/^/  /'
jt "vio-hard-reset" 30 >/dev/null

start=$SECONDS; H=0
while [ $((SECONDS-start)) -lt "$WAIT" ]; do
    jt "halt-status" 8 | grep -q 'effective=1' && { H=1; break; }
done
[ "$H" = 1 ] || { echo "FAIL: no _SysError in ${WAIT}s"; exit 1; }

echo "  $(jt "atrap status" 20 | grep -oE 'HIT: .*' | head -1)"
PC=$(jt "pc" 15 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
echo "  halted at PC=$PC  (the \$A9C9 instruction itself)"
echo
echo "=== registers at the SysError call ==="
ARCH=$(jt "live-arch" 40); echo "$ARCH" | sed 's/^/  /'
SP=$(echo "$ARCH" | grep -oE '^> A7 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | head -1)
D0=$(echo "$ARCH" | grep -oE '^> D0 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | head -1)
echo
echo "  error code D0 = $D0  (low word = $(( $(printf '%d' "${D0:-0}") & 0xFFFF )))"
echo
echo "=== stack at A7=$SP — the top longword is the CALLER's return address ==="
jt "dump-mem $(printf '0x%08X' $(( $(printf '%d' "$SP") & ~3 ))) 16" 40 | grep '^> mem' | sed 's/^/  /'
echo
echo "=== PC trace (how we got here) ==="
jt "pc-trace 48" 40 | grep '^> trace' | tail -28 | sed 's/^/  /'
echo
echo "=== code at the SysError site ==="
tools/dis_at.sh $(printf "0x%08X" $(( $(printf '%d' "$PC") - 0x40 ))) 28 2>&1 | sed 's/^/  /'
