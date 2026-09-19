#!/bin/bash
# cdef_fail_full_state.sh — halt at the CDEF load failure and capture EVERY
# comparison point in ONE stop.  Each board boot costs ~7 minutes and only some
# boots reach the dialog, so sampling one global per boot is unaffordable.
#
# MAME reference values (same 7.5.3 image, measured 2026-08-18):
#   TopMapHdl=00795F18 SysMapHdl=00002068 SysMap=2 CurMap=1694 ResLoad=511 ResErr=0
#   ApplZone free=44464   SysZone free=55868
#   SCSI: 3597 READ(10), 0 REQUEST SENSE (no CHECK CONDITION all boot)
#   CDEF: master pointer never purged (stays 601C4BF0)
#
# A wrong CurMap/TopMapHdl would make _LoadResource search the wrong resource
# map and fail with no disk access at all -- which fits the measured
# "ioErr with zero SD errors".

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-430}
TRIES=${TRIES:-4}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
wait_halt() { local s=$SECONDS
    while [ $((SECONDS-s)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0; done; return 1; }
rdl() { jt "dump-mem $1 1" 25 | grep -oE '= 0x[0-9a-fA-F]{8}' | head -1 | sed 's/= //'; }

for t in $(seq 1 "$TRIES"); do
    echo "=== attempt $t/$TRIES ==="
    for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
        jt "$c" >/dev/null; done
    jt "break-pc 0x40815E6A" 25 | grep -E "slot=" | sed 's/^/  /'
    jt "vio-hard-reset" 30 >/dev/null
    wait_halt "$WAIT" || { echo "  boot did not reach the failure; retrying"; continue; }
    echo "  HALTED at the CDEF failure arm"
    echo

    A50=$(rdl 0x00000A50); A54=$(rdl 0x00000A54); A58=$(rdl 0x00000A58); A5C=$(rdl 0x00000A5C); A60=$(rdl 0x00000A60)
    python3 - "$A50" "$A54" "$A58" "$A5C" "$A60" <<'PY'
import sys
def L(x):
    try: return int(x,16)
    except: return None
a50,a54,a58,a5c,a60=[L(v) for v in sys.argv[1:6]]
def hi(v): return None if v is None else (v>>16)&0xFFFF
def lo(v): return None if v is None else v&0xFFFF
def sw(v): return None if v is None else (v-0x10000 if v>=0x8000 else v)
print("  === Resource Manager globals (HW  vs  MAME) ===")
print(f"    TopMapHdl @0A50 = {a50 if a50 is None else format(a50,'08X')}      MAME 00795F18")
print(f"    SysMapHdl @0A54 = {a54 if a54 is None else format(a54,'08X')}      MAME 00002068")
print(f"    SysMap(w) @0A58 = {sw(hi(a58))}                MAME 2")
print(f"    CurMap(w) @0A5A = {sw(lo(a58))}                MAME 1694")
print(f"    ResLoad(w)@0A5E = {sw(lo(a5c))}                MAME 511")
print(f"    ResErr(w) @0A60 = {sw(hi(a60))}                MAME 0")
PY
    echo
    echo "  === zones ==="
    P2A4=$(rdl 0x000002A4); P2A8=$(rdl 0x000002A8); P2AC=$(rdl 0x000002AC)
    ZONES=$(python3 -c "
a=int('$P2A4',16); b=int('$P2A8',16); c=int('$P2AC',16)
print('0x%08X 0x%08X' % (((a&0xFFFF)<<16)|((b>>16)&0xFFFF), ((b&0xFFFF)<<16)|((c>>16)&0xFFFF)))")
    SZ=$(echo $ZONES | awk '{print $1}'); AZ=$(echo $ZONES | awk '{print $2}')
    SF=$(rdl $(printf '0x%08X' $(( $(printf '%d' $SZ) + 12 ))))
    AF=$(rdl $(printf '0x%08X' $(( $(printf '%d' $AZ) + 12 ))))
    echo "    SysZone  $SZ free=$((16#${SF#0x}))   MAME 55868"
    echo "    ApplZone $AZ free=$((16#${AF#0x}))   MAME 44464"
    echo
    echo "  === SD/SCSI latches ==="
    RAW=$(jt "vio-read scsi" 40 | grep -oE 'vio_scsi_sd = [0-9a-fA-F]+' | awk '{print $3}')
    python3 -c "
v=int('$RAW',16); g=lambda h,l:(v>>l)&((1<<(h-l+1))-1)
print(f'    completions={g(43,28)} errors={g(27,20)} cause={g(19,16)} sticky={g(3,3)} errLBA=0x{g(75,44):08X}')"
    echo
    echo "  === CDEF handle ==="
    CR=$(rdl 0x0007efac)
    echo "    *0x0007EFAC (ControlRecord) = $CR"
    if [ -n "$CR" ]; then
      DP=$(rdl $(printf '0x%08X' $(( $(printf '%d' $CR) + 24 ))))
      echo "    contrlDefProc = $DP    MAME's equivalent is LOADED (601C4BF0)"
      [ -n "$DP" ] && echo "    *contrlDefProc = $(rdl $DP)"
    fi
    exit 0
done
echo "=== never caught a failing boot ==="; exit 1
