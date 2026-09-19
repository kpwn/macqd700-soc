#!/bin/bash
# dump_setsize_call.sh — halt on _SetHandleSize(h,-24) and dump everything
# needed to explain WHERE the -24 came from, for THIS boot.
#
# The -24 was previously reconstructed as 0 + 4 - 28 from a disassembly taken on
# a DIFFERENT boot, and the Finder relocates every boot, so that reconstruction
# was never verified against the registers that actually produced it.  This
# captures code + registers + the pointed-to memory in ONE halt, so the
# arithmetic can be checked rather than inferred:
#
#     moveal %fp@(8),%a4     ; a4 = the handle argument
#     moveal %a4@,%a0        ; a0 = *handle  (master pointer deref)
#     movel  %a0@(4),%d0     ; d0 = a field IN the table  (observed 0)
#     moveq  #-4,%d1
#     subl   %d1,%d0         ; d0 = 0 + 4
#     addl   %d7,%d0         ; d0 = 4 + d7   -> -24 implies d7 = -28
#
# An A-trap halt is relocation-proof (it matches the OPCODE WORD, not an
# address), which is why this works when break-pc on a Finder address does not.
# `atrap` halts BEFORE the trap's side effects, so the registers here are the
# inputs to _SetHandleSize.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-400}

jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }

echo "=== arming atrap on _SetHandleSize with D0 == -24 ==="
jt "watch 0 off"  >/dev/null; jt "watch 1 off" >/dev/null
jt "break-pc off" >/dev/null; jt "atrap 1 off" >/dev/null
jt "halt-clear"   >/dev/null
jt "atrap 0 0xA024 d0 0xFFFFFFE8" 25 | grep -E "armed|ERROR" | sed 's/^/  /'
jt "vio-hard-reset" 30 >/dev/null

start=$SECONDS; HALTED=0
while [ $((SECONDS - start)) -lt "$WAIT" ]; do
    jt "halt-status" 8 | grep -q 'effective=1' && { HALTED=1; break; }
done
[ "$HALTED" = 1 ] || { echo "FAIL: no _SetHandleSize(-24) in ${WAIT}s"; exit 1; }

PC=$(jt "pc" 15 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
echo "  halted at PC=$PC"
echo
echo "=== registers (inputs to _SetHandleSize) ==="
ARCH=$(jt "live-arch" 40); echo "$ARCH" | sed 's/^/  /'

reg() { echo "$ARCH" | grep -oE "^> $1 = 0x[0-9a-f]+" | grep -oE '0x[0-9a-f]+' | head -1; }

echo
echo "=== code around the trap ==="
tools/dis_at.sh $(printf "0x%08X" $(( $(printf '%d' "$PC") - 0x60 ))) 56 2>&1 | sed 's/^/  /'

echo
echo "=== memory behind the pointer registers ==="
for R in A0 A1 A2 A3 A4 A5 A6; do
    V=$(reg $R); [ -n "$V" ] || continue
    N=$(printf '%d' "$V")
    [ "$N" -gt 4096 ] && [ "$N" -lt 2147483647 ] || continue
    echo "--- $R = $V ---"
    jt "dump-mem $(printf '0x%08X' $(( N & ~3 ))) 8" 30 | grep '^> mem' | sed 's/^/    /'
done

echo
echo "=== the handle chain: a4 = handle, *a4 = master ptr, table at *a4 ==="
A4=$(reg A4)
if [ -n "$A4" ]; then
    MP=$(jt "dump-mem $(printf '0x%08X' $(( $(printf '%d' "$A4") & ~3 ))) 1" 25 | grep -oE '= 0x[0-9a-fA-F]{8}' | head -1 | sed 's/= //')
    echo "  *A4 (master pointer) = ${MP:-?}"
    if [ -n "$MP" ]; then
        MPN=$(printf '%d' "$MP")
        if [ "$MPN" -gt 4096 ] && [ "$MPN" -lt 2147483647 ]; then
            echo "  --- table at $MP (offset 4 is the field read into d0) ---"
            jt "dump-mem $(printf '0x%08X' $(( MPN & ~3 ))) 16" 40 | grep '^> mem' | sed 's/^/    /'
        else
            echo "  (master pointer is not a plausible address -- itself a finding)"
        fi
    fi
fi
echo
echo "=== D7 is the delta added to produce -24 ==="
echo "  D7 = $(reg D7)   D0 = $(reg D0)   A5 = $(reg A5)"
