#!/bin/bash
# watch_scsi_stall.sh — poll the SD/SCSI VIO probe through a whole boot and look
# for a STALLED command: completions stop advancing while the backing store is
# still busy, or while SCSI DRQ/IRQ sit asserted.
#
# This tests the leading theory directly and needs no breakpoints, no halting,
# and no trace ring (which is de-instantiated on this bitstream):
#
#   scsi.v waits on the provider with NO timeout while vh_busy is high --
#   "a provider is entitled to take as long as it likes as long as it says it
#   is working".  So a provider that raises busy and never finishes hangs the
#   command silently: no CHECK CONDITION, no sd_ctrl error, nothing reported.
#   Only the Mac's SCSI Manager notices, and it returns ioErr (-36) -- which is
#   exactly what ResErr holds at the CDEF failure.
#
# vio_scsi_sd (probe_in22): [43:28] completions  [27:20] errors  [19:16] cause
#                           [3] sticky  [2] busy  [1] irq  [0] drq
#
# Interpreting the tail: completions plateau at the END of boot is NORMAL (the
# machine simply stops reading once booted).  What matters is a plateau while
# busy=1 -- that is a command in flight that never finishes.

cd "$(dirname "$0")/.." || exit 1
DUR=${DUR:-420}
LOG=${LOG:-/tmp/scsi_stall.log}
jt() { JT_WAIT=${1:-12} tools/jt.sh "$2" 2>&1; }

echo "=== reset and poll the SD/SCSI probe for ${DUR}s ==="
for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
    JT_WAIT=15 tools/jt.sh "$c" >/dev/null 2>&1; done
JT_WAIT=30 tools/jt.sh "vio-hard-reset" >/dev/null 2>&1

: > "$LOG"
s=$SECONDS
while [ $((SECONDS-s)) -lt "$DUR" ]; do
    RAW=$(JT_WAIT=10 tools/jt.sh "vio-read scsi" 2>&1 | grep -oE 'vio_scsi_sd = [0-9a-fA-F]+' | awk '{print $3}')
    [ -n "$RAW" ] && python3 -c "
v=int('$RAW',16); g=lambda h,l:(v>>l)&((1<<(h-l+1))-1)
print(f'$((SECONDS-s)) {g(43,28)} {g(27,20)} {g(19,16)} {g(3,3)} {g(2,2)} {g(1,1)} {g(0,0)}')" >> "$LOG"
done

echo "  samples: $(wc -l < "$LOG")"
echo
echo "=== analysis ==="
python3 - "$LOG" <<'PY'
import sys
rows=[]
for line in open(sys.argv[1]):
    p=line.split()
    if len(p)==8: rows.append(tuple(int(x) for x in p))
if not rows: print("  no samples"); raise SystemExit
print(f"  {len(rows)} samples over {rows[-1][0]}s; completions {rows[0][1]} -> {rows[-1][1]}")
errs=[r for r in rows if r[2]>0 or r[4]>0]
print(f"  samples with sd_ctrl error/sticky set: {len(errs)}")
# Longest run with no completion progress while the SCSI side still looks
# ACTIVE.  busy alone is not enough: vhdd_sd:148 is `assign busy = sd_busy`,
# and sd_ctrl has its own request watchdog that would surface as err_cause=8 +
# CHECK CONDITION -- which we never see.  So a permanently stuck vh_busy is
# unlikely, and the more plausible stall is the INITIATOR-side pseudo-DMA
# handshake (drq/irq) hanging in DATA_IN.  Count any of the three.
best=(0,None); cur=0; start=None
for i in range(1,len(rows)):
    t,c,e,ca,st,busy,irq,drq = rows[i]
    if c==rows[i-1][1] and (busy==1 or irq==1 or drq==1):
        if cur==0: start=rows[i-1][0]
        cur+=t-rows[i-1][0]
        if cur>best[0]: best=(cur,start)
    else:
        cur=0
print(f"  longest BUSY-with-no-progress run: {best[0]}s starting at t={best[1]}s")
# also longest plateau regardless of busy, for context
cur=0;bp=(0,None);start=None
for i in range(1,len(rows)):
    if rows[i][1]==rows[i-1][1]:
        if cur==0: start=rows[i-1][0]
        cur+=rows[i][0]-rows[i-1][0]
        if cur>bp[0]: bp=(cur,start)
    else: cur=0
print(f"  longest completions plateau (any state): {bp[0]}s starting at t={bp[1]}s")
# show the active-state samples around the biggest active plateau
act=[r for r in rows if (r[5] or r[6] or r[7])]
print(f"  samples with busy|irq|drq asserted: {len(act)} of {len(rows)}")
if act[:6]:
    print("  first few active samples (t comp err cause sticky busy irq drq):")
    for r in act[:6]: print("   ", r)
print()
# A Mac SCSI Manager timeout is only a few seconds, and sampling is ~1s over
# JTAG, so the bar has to be low. Report the top plateaus either way so a
# marginal result is visible rather than rounded to 'nothing found'.
if best[0]>=2:
    print("  => STALL FOUND: the SCSI side stayed active (busy/irq/drq) with no")
    print("     completion for several seconds -- a command in flight that never")
    print("     finished. That is the shape that makes the driver time out and")
    print("     return ioErr with nothing reported by scsi.v or sd_ctrl.")
else:
    print("  => no active-with-no-progress stall seen in this window.")
    print("     Note the sampling period is ~1s over JTAG, so a stall shorter")
    print("     than that is invisible here; this rules out a LONG hang, not a")
    print("     brief one.")
PY
