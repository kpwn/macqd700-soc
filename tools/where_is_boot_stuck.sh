#!/bin/bash
# where_is_boot_stuck.sh — for the ~75% of boots that never reach the Shutdown
# dialog, find out WHERE they actually end up.
#
# Rationale: every test on the CDEF frontier needs a boot that reaches the
# dialog, and only ~1 in 3 (measured: ~3 of 12) does. Rather than keep paying
# ~7.5 min per coin flip, characterise the majority case. If those boots are
# wedged at a specific PC, that is a lead in its own right -- possibly the same
# fault failing earlier.
#
# Reads only non-invasive state while the CPU RUNS (no halting, no breakpoints):
#   pc-trace   ring of taken redirects (fall-through is NOT recorded, so an
#              address missing from it proves nothing)
#   exc-ring   recent (vec, pc, fault_addr) + per-PC tally -- use this, not the
#              single exc_pc latch, to tell one looping PC from many faulting
#   halt-status live PC + exception counters

cd "$(dirname "$0")/.." || exit 1
SETTLE=${SETTLE:-300}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }

echo "=== reset and let it run ${SETTLE}s without any breakpoints ==="
for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
    jt "$c" >/dev/null; done
jt "vio-hard-reset" 30 >/dev/null
s=$SECONDS
while [ $((SECONDS-s)) -lt "$SETTLE" ]; do jt "halt-status" 8 >/dev/null; done

echo
echo "=== live PC sampled 8 times (is it moving, or parked?) ==="
for i in $(seq 1 8); do
    jt "halt-status" 10 | grep -oE 'pc_live=0x[0-9a-f]+' | head -1
done | sort | uniq -c | sort -rn | sed 's/^/  /'

echo
echo "=== halt-status ==="
jt "halt-status" 20 | grep -oE 'pc_live=0x[0-9a-f]+|exc_vec=0x[0-9a-f]+|exc_pc=0x[0-9a-f]+|exc_count=0x[0-9a-f]+|effective=[01]' | sed 's/^/  /'

echo
echo "=== exception ring (many PCs faulting vs one looping) ==="
jt "exc-ring 24" 40 | grep -E '^> ' | tail -28 | sed 's/^/  /'

echo
echo "=== pc-trace (taken redirects only) ==="
jt "pc-trace 40" 40 | grep '^> trace' | tail -24 | sed 's/^/  /'

echo
echo "=== SD/SCSI state ==="
RAW=$(jt "vio-read scsi" 40 | grep -oE 'vio_scsi_sd = [0-9a-fA-F]+' | awk '{print $3}')
[ -n "$RAW" ] && python3 -c "
v=int('$RAW',16); g=lambda h,l:(v>>l)&((1<<(h-l+1))-1)
print(f'  completions={g(43,28)} errors={g(27,20)} cause={g(19,16)} sticky={g(3,3)} busy={g(2,2)} irq={g(1,1)} drq={g(0,0)}')"
