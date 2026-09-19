#!/bin/bash
# catch_driver_ioerr.sh — which site in the .ASYC00 disk driver returns ioErr?
#
# All four ROM `moveq #-36` sites are eliminated (breakpointed on a boot that
# provably REACHED the failure, via the marker, and none fired). The disk driver
# is `.ASYC00` -- the OpenRetroSCSI driver loaded from the image's
# Apple_Driver43 partition, i.e. the code that talks to OUR SCSI RTL -- and it
# contains exactly three -36 sites:
#     0x0000CFF6  0x0000D058  0x0000DC04
# Whichever fires shows the condition the driver rejects on, and that condition
# is a hardware interaction we control.
#
# Slot 4 is the reach marker (0x40815E6A) so a null result is interpretable.
# Identify hits via HALT_HIT_PC ("hit="), NOT the live PC -- they differ, and
# using `pc` misclassified real hits earlier in this investigation.
#
# NOTE these are RAM addresses; if the driver loads elsewhere on a given boot
# the breakpoints simply will not fire, so confirm the driver address first
# (DCE via UTableBase 0x11C, unit 32 -> dCtlDriver).

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-270}
MAXH=${MAXH:-12}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
wait_halt() { local s=$SECONDS
    while [ $((SECONDS-s)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0; done; return 1; }

for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
    jt "$c" >/dev/null; done
for a in 0x0000CFF6 0x0000D058 0x0000DC04 0x40815E6A; do
    jt "break-pc $a" 25 | grep -E "slot=" | sed 's/^/  /'
done
jt "vio-hard-reset" 30 >/dev/null
START=$SECONDS

for i in $(seq 1 "$MAXH"); do
    wait_halt "$WAIT" || { echo "  no further hit in ${WAIT}s"; break; }
    T=$((SECONDS-START))
    HS=$(jt "halt-status" 20)
    HIT=$(echo "$HS" | grep -oE 'hit=0x[0-9a-f]+' | head -1 | sed 's/hit=//')
    LIVE=$(echo "$HS" | grep -oE 'pc_live=0x[0-9a-f]+' | head -1 | sed 's/pc_live=//')
    case "${HIT,,}" in
      0x0000cff6) W="*** .ASYC00 ioErr site #1 (0xCFF6) ***" ;;
      0x0000d058) W="*** .ASYC00 ioErr site #2 (0xD058) ***" ;;
      0x0000dc04) W="*** .ASYC00 ioErr site #3 (0xDC04) ***" ;;
      0x40815e6a) W="MARKER: reached the CDEF failure" ;;
      *)          W="(other halt)" ;;
    esac
    echo "  [$i] t=${T}s hit=$HIT pc_live=$LIVE  $W"
    case "${HIT,,}" in
      0x0000cff6|0x0000d058|0x0000dc04)
        A=$(jt "live-arch" 40)
        echo "$A" | grep -E "^> (D0|D1|D2|D3|A0|A1|A2|A3|A4|A5|A7) " | sed 's/^/      /'
        # The driver retries because a4@(10) is non-zero. That word is the
        # status of the failed operation -- dump the block A4 points at.
        A4=$(echo "$A" | grep -oE '^> A4 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | head -1)
        if [ -n "$A4" ]; then
            AA=$(printf '0x%08X' $(( $(printf '%d' "$A4") & ~3 )))
            echo "      --- param block at A4=$A4 (offset +10 is the status) ---"
            jt "dump-mem $AA 12" 60 | grep '^> mem' | sed 's/^/        /'
        fi
        # Correlate with OUR SCSI RTL at this instant. The driver is retrying,
        # which means it RE-ISSUES commands -- but the completion counter was
        # measured frozen at 3566 from t=49s. If completions do not advance
        # across retries, the commands never complete, and that is scsi.v.
        RAW=$(jt "vio-read scsi" 40 | grep -oE 'vio_scsi_sd = [0-9a-fA-F]+' | awk '{print $3}')
        [ -n "$RAW" ] && python3 -c "
v=int('$RAW',16); g=lambda h,l:(v>>l)&((1<<(h-l+1))-1)
print(f'      SCSI/SD now: completions={g(43,28)} errors={g(27,20)} cause={g(19,16)} sticky={g(3,3)} busy={g(2,2)} irq={g(1,1)} drq={g(0,0)}')"
        # Driver globals: the splitting routine at 0xCE48 stores the request
        # parameters relative to A5 --  a5@(-158)=buffer, a5@(-154)=start,
        # a5@(-150)=remainder, a5@(-126)=retry counter.  These name the actual
        # operation that keeps failing.
        # MUST push the D-cache first: JTAG reads DDR, and freshly written
        # values (e.g. the retry counter) are still dirty in the CPU cache --
        # reading without this returned 1 and 12 for a counter the CPU had at 17.
        jt "dcache-op push" 30 >/dev/null
        A5=$(echo "$A" | grep -oE '^> A5 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | head -1)
        if [ -n "$A5" ]; then
            N5=$(printf '%d' "$A5")
            for off in -158 -154 -150 -146 -126 -122 -102 -98; do
                AD=$(( N5 + off ))
                B=$(( AD & ~3 )); SH=$(( AD - B ))
                W1=$(jt "dump-mem $(printf '0x%08X' $B) 1" 20 | grep -oE '= 0x[0-9a-fA-F]{8}' | head -1 | sed 's/= 0x//')
                W2=$(jt "dump-mem $(printf '0x%08X' $(( B + 4 ))) 1" 20 | grep -oE '= 0x[0-9a-fA-F]{8}' | head -1 | sed 's/= 0x//')
                [ -n "$W1" ] && [ -n "$W2" ] && python3 -c "
v=(int('$W1',16)<<32)|int('$W2',16)
print('        a5@(%d) [0x%08X] = 0x%08X' % ($off, $AD, (v >> (32-8*$SH)) & 0xFFFFFFFF))"
            done
        fi
        # Dump the request block: look for a SCSI CDB (opcode 0x00/0x08/0x12/
        # 0x25/0x28) which names the operation that keeps failing.
        A4d=$(echo "$A" | grep -oE '^> A4 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | head -1)
        if [ -n "$A4d" ]; then
            AA=$(printf '0x%08X' $(( $(printf '%d' "$A4d") & ~3 )))
            echo "      --- request block at $A4d (coherent) ---"
            jt "dump-mem $AA 16" 60 | grep -oE '= 0x[0-9a-fA-F]{8}' | sed 's/= 0x//' | tr -d '\n' > /tmp/reqblk.hex
            python3 - <<'PYEOF'
b=bytes.fromhex(open('/tmp/reqblk.hex').read().strip())
names={0x00:'TEST UNIT READY',0x03:'REQUEST SENSE',0x08:'READ(6)',0x12:'INQUIRY',
       0x15:'MODE SELECT',0x1A:'MODE SENSE',0x1B:'START STOP',0x25:'READ CAPACITY',
       0x28:'READ(10)',0x2A:'WRITE(10)'}
print("        " + b[:32].hex())
for i in range(0,min(len(b),48)):
    if b[i] in names and i+5 < len(b):
        print(f"        +{i:02d}: possible CDB opcode 0x{b[i]:02X} {names[b[i]]}  bytes={b[i:i+10].hex()}")
PYEOF
        fi
        # LIVE volume state. scsi.v rejects a READ before touching the backend
        # when the addressed volume reports vh_num_lbas == 0 (!vh_chk_ok ->
        # ILLEGAL REQUEST / CHECK CONDITION). The failing CDB is READ(10) LBA
        # 2314 x1, which sim proves is fine at default state -- so the volume
        # enables/sizes at THIS instant are the thing to check.
        echo "      --- live vhdd/volume state ---"
        jt "vhdd-status" 60 | grep -E "SD volume|RAM disk|ident|WARNING" | sed 's/^/        /'
        echo "      --- pc-trace (how the driver got here) ---"
        jt "pc-trace 24" 40 | grep '^> trace' | tail -12 | sed 's/^/      /'
        ;;
    esac
    jt "halt-clear" 20 >/dev/null; jt "cont" 30 >/dev/null
done
