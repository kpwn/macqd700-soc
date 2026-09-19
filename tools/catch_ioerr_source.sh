#!/bin/bash
# catch_ioerr_source.sh — which ROM site actually generates the ioErr(-36)?
#
# The whole ROM contains only SIX `moveq #-36,%d0` sites; one (0x408A8678)
# disassembles as data, leaving four real candidates. break-pc has 4 slots, so
# cover them all in a single boot instead of guessing:
#   0x40811A22  block-transfer verify path (pre-sets -36, cleared on success)
#   0x408146F0  File Manager error-translation shim -- stores the REAL error to
#               lowmem 0x3DE, then substitutes -36 (break BEFORE the moveq so D0
#               still holds the genuine code)
#   0x40871124  driver path via lowmem 0x134
#   0x40871140  driver path, sibling of the above
# All are FIXED ROM addresses, so these breakpoints are reliable -- unlike every
# heap address in this investigation.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-260}
MAXH=${MAXH:-12}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
wait_halt() { local s=$SECONDS
    while [ $((SECONDS-s)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0; done; return 1; }

for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
    jt "$c" >/dev/null; done
# 0x40811A22 (the verify arm) is ELIMINATED -- broken on alone for 260s with no
# hit, so that path is never taken. Free that slot for a REACH MARKER at the
# CDEF failure, so "no hits" can be distinguished from "this boot never got
# there" -- which is what made the first run of this test uninterpretable.
for a in 0x408146F0 0x40871124 0x40871140 0x40815E6A; do
    jt "break-pc $a" 25 | grep -E "slot=" | sed 's/^/  /'
done
jt "vio-hard-reset" 30 >/dev/null
START=$SECONDS

for i in $(seq 1 "$MAXH"); do
    wait_halt "$WAIT" || { echo "  no further hit in ${WAIT}s"; break; }
    T=$((SECONDS-START))
    # USE HALT_HIT_PC, NOT the live PC.  halt-status reports `hit=` (the
    # breakpoint that actually latched) separately from `pc_live=` (where the
    # CPU is now); they differ (e.g. hit=0x0079b616 pc_live=0x0079b638), so
    # identifying a breakpoint from `pc` misclassifies real hits as
    # "unexpected" -- which is what made the first run of this test look empty.
    HS=$(jt "halt-status" 20)
    PC=$(echo "$HS" | grep -oE 'hit=0x[0-9a-f]+' | head -1 | sed 's/hit=//')
    LIVE=$(echo "$HS" | grep -oE 'pc_live=0x[0-9a-f]+' | head -1 | sed 's/pc_live=//')
    BPF=$(echo "$HS" | grep -oE 'break_pc=[01]' | head -1)
    D0=$(jt "live-arch" 40 | grep -oE '^> D0 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | head -1)
    case "${PC,,}" in
      0x40815e6a) W="MARKER: reached the CDEF failure (so this boot DID get there)" ;;
      0x408146f0) W="FM error-translation shim (D0 = the REAL error)" ;;
      0x40871124) W="driver path via lowmem 0x134" ;;
      0x40871140) W="driver path (sibling)" ;;
      *)          W="(unexpected)" ;;
    esac
    echo "        (hit=$PC pc_live=$LIVE $BPF)"
    python3 -c "
v=int('${D0:-0x0}',16); w=v & 0xFFFF
s=w-0x10000 if w>=0x8000 else w
names={-35:'nsvErr',-36:'ioErr',-39:'eofErr',-40:'posErr',-43:'fnfErr',
 -49:'opWrErr',-50:'paramErr',-51:'rfNumErr',-53:'volOffLinErr',-54:'permErr',
 -55:'volOnLinErr',-56:'nsDrvErr',-57:'noMacDskErr',-58:'extFSErr',-60:'badMDBErr',
 -64:'lastDskErr',-65:'noDriveErr',-127:'fsDSIntErr'}
print(f'  [$i] t=${T}s  PC=$PC  $W')
print(f'        D0=0x{v:08X} -> {s}  {names.get(s,\"\")}')"
    jt "halt-clear" 20 >/dev/null; jt "cont" 30 >/dev/null
done
