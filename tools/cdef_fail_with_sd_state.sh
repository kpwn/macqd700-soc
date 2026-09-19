#!/bin/bash
# cdef_fail_with_sd_state.sh — halt AT the CDEF load failure and, in that same
# halted state, read the SD/SCSI error latches.  Correlates the OS-level error
# with the hardware-level one on the SAME boot.
#
# Reading the VIO probe on an arbitrary boot proves nothing: a boot that never
# reaches the failure shows 0 errors trivially (measured: 3566 completions,
# 0 errors, on a boot that never hit the failure arm).  The two must be sampled
# together.
#
# vio_scsi_sd (probe_in22, 76 bits, 19 hex nibbles) layout:
#   [75:44] first failing SD LBA   [43:28] completions   [27:20] error count
#   [19:16] sd_ctrl err_cause      [15:12] cmd           [11:4] detail
#   [3] sticky first-error valid   [2] busy  [1] irq  [0] drq
#
# DECIDES:
#   err_count > 0 / sticky=1  -> the disk path really did fail; err_cause + LBA
#                                name the RTL defect.
#   err_count == 0            -> ResErr=-36 did NOT come from our SD layer on
#                                this boot; it is stale or from elsewhere, and
#                                the memFullErr/heap path becomes the lead.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-420}
TRIES=${TRIES:-4}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
wait_halt() { local s=$SECONDS
    while [ $((SECONDS-s)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0; done; return 1; }
word() {
    local v=$(jt "dump-mem $1 1" 25 | grep -oE '= 0x[0-9a-fA-F]{8}' | head -1 | sed 's/= //')
    [ -n "$v" ] || { echo "?"; return; }
    python3 -c "n=int('$v',16); x=(n>>16)&0xFFFF; print(x-0x10000 if x>=0x8000 else x)"
}

for t in $(seq 1 "$TRIES"); do
    echo "=== attempt $t/$TRIES ==="
    jt "watch 0 off" >/dev/null; jt "watch 1 off" >/dev/null
    jt "atrap 0 off" >/dev/null; jt "atrap 1 off" >/dev/null
    jt "break-pc off" >/dev/null; jt "halt-clear" >/dev/null
    jt "break-pc 0x40815E6A" 25 | grep -E "slot=|ERROR" | sed 's/^/  /'
    jt "vio-hard-reset" 30 >/dev/null

    if ! wait_halt "$WAIT"; then
        echo "  this boot never reached the CDEF failure -- SD state would be"
        echo "  meaningless here, so not reporting it. Retrying."
        continue
    fi
    echo "  HALTED at the CDEF failure arm (PC=$(jt "pc" 15 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1))"
    echo "  ResErr=$(word 0x00000A60)  MemErr=$(word 0x00000220)"
    RAW=$(jt "vio-read scsi" 40 | grep -oE 'vio_scsi_sd = [0-9a-fA-F]+' | awk '{print $3}')
    echo "  vio_scsi_sd = $RAW"
    python3 - "$RAW" <<'PY'
import sys
s=sys.argv[1].strip()
if not s: print("  (no probe value)"); raise SystemExit
v=int(s,16); n=len(s)*4
g=lambda hi,lo:(v>>lo)&((1<<(hi-lo+1))-1)
cause={0:"none",1:"CRC bad",2:"R1 poll timeout (card silent)",3:"0xFE data-token timeout (read stall)",
       4:"?",5:"busy timeout (write stuck)",8:"global request watchdog"}
print(f"  bits={n}")
print(f"  first failing LBA = 0x{g(75,44):08X}")
print(f"  completions       = {g(43,28)}")
print(f"  ERROR COUNT       = {g(27,20)}")
print(f"  err_cause         = {g(19,16)}  {cause.get(g(19,16),'')}")
print(f"  err_cmd           = {g(15,12)}   detail = 0x{g(11,4):02X}")
print(f"  sticky={g(3,3)} busy={g(2,2)} irq={g(1,1)} drq={g(0,0)}")
print()
if g(27,20)>0 or g(3,3):
    print("  => THE DISK PATH FAILED. err_cause + LBA name the defect.")
else:
    print("  => NO SD-level error on the boot that failed. The ioErr did NOT")
    print("     originate in our SD layer; follow memFullErr / the heap instead.")
PY
    exit 0
done
echo "=== never caught a failing boot in $TRIES attempts ==="
exit 1
