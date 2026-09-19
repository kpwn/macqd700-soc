#!/bin/bash
# catch_real_ioerr.sh — recover the REAL error hidden behind the generic ioErr.
#
# ROM 0x408146E6 is an error-translation shim in the File Manager:
#     tstw  %d0            ; underlying error
#     beqs  ok             ; 0 -> fine
#     cmpiw #-65,%d0       ; noDriveErr tolerated
#     beqs  ok
#     movew %d0,0x3de      ; SAVE the real error at lowmem 0x3DE
#     moveq #-36,%d0       ; ...and return generic ioErr
# So every ioErr we have been chasing is a SUBSTITUTE. Breaking at 0x408146F0
# (the store) catches D0 still holding the genuine error code.
#
# Fixed ROM address => reliable breakpoint, unlike every heap address in this
# investigation. Log each hit; the read that fails is
# _Read("System" rsrc fork, 4B @ 0x00288896).

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-260}
MAXH=${MAXH:-12}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
wait_halt() { local s=$SECONDS
    while [ $((SECONDS-s)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0; done; return 1; }

for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
    jt "$c" >/dev/null; done
jt "break-pc 0x408146F0" 25 | grep -E "slot=" | sed 's/^/  /'
jt "vio-hard-reset" 30 >/dev/null
START=$SECONDS

for i in $(seq 1 "$MAXH"); do
    wait_halt "$WAIT" || { echo "  no further hit in ${WAIT}s"; break; }
    T=$((SECONDS-START))
    D0=$(jt "live-arch" 40 | grep -oE '^> D0 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | head -1)
    python3 -c "
v=int('${D0:-0x0}',16); w=v & 0xFFFF
s=w-0x10000 if w>=0x8000 else w
names={-33:'dirFulErr',-34:'dskFulErr',-35:'nsvErr no such volume',-36:'ioErr',
 -37:'bdNamErr',-38:'fnOpnErr',-39:'eofErr',-40:'posErr',-42:'tmfoErr',
 -43:'fnfErr file not found',-44:'wPrErr',-45:'fLckdErr',-46:'vLckdErr',
 -47:'fBsyErr',-48:'dupFNErr',-49:'opWrErr',-50:'paramErr',-51:'rfNumErr',
 -53:'volOffLinErr',-54:'permErr',-55:'volOnLinErr',-56:'nsDrvErr no such drive',
 -57:'noMacDskErr',-58:'extFSErr',-59:'fsRnErr',-60:'badMDBErr',-61:'wrPermErr',
 -64:'lastDskErr',-65:'noDriveErr',-127:'fsDSIntErr internal'}
print(f'  [$i] t=${T}s  REAL error D0=0x{v:08X} -> {s}  {names.get(s,\"(unknown)\")}')"
    jt "halt-clear" 20 >/dev/null; jt "cont" 30 >/dev/null
done
