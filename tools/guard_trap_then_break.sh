#!/bin/bash
# guard_trap_then_break.sh — resolve the Finder guard's address FOR THIS BOOT,
# then breakpoint it.  Supersedes catch_guard_misbranch.sh, which guessed the
# address from previously-observed layouts and missed 8/8 boots.
#
# WHY GUESSING FAILS: the Finder's code lands at a different address nearly
# every boot (guards seen at 0x758ed2/0x759610/0x763d88 on one boot,
# 0x75abda/0x75b318 on another).  Only 4 break-pc slots exist, so a candidate
# list cannot cover it.  Controls confirm the mechanism itself is fine:
# break-pc fires normally, AND fires after a vio-hard-reset while armed.
#
# STRATEGY
#   1. Arm the A-trap on _SetHandleSize (0xA024) qualified on D0 == -24, and
#      reset.  This halts BEFORE the trap's side effects, deep inside the
#      Finder's delete path, on whatever boot actually gets there.
#   2. The stack then holds the delete-call return address, which lies inside
#      the very routine we want.  Scan a NARROW window around each Finder-range
#      stack value for the guard signature 2d40fffc 504f 6700 (movel d0,fp@(-4)
#      / addqw #8,sp / beqw).  ~320 words per candidate, seconds over JTAG.
#   3. Arm break-pc on guard = sig+6, disarm the A-trap, resume, and WAIT.
#      The Finder's `while (RemoveCommandsMatching(...))` loop re-enters the
#      guard once the Memory Manager finally returns memFullErr, so the guard
#      is reachable again -- an earlier attempt gave up after ~8 polls, which
#      was far too short.
#   4. On the guard hit: D0/SR are what the beqw will resolve against
#      (break-pc halts pre-instruction).  Step once; fall-through is guard+4.
#
#        D0==0 & Z==1 & lands guard+4 -> BRANCH RESOLUTION is wrong
#        D0==0 & Z==0                 -> FLAGS were already wrong upstream
#
# jt.sh returns on output-quiet (~1-2 s), NOT after JT_WAIT, so all timeouts
# here are wall-clock deadlines, never poll counts.

cd "$(dirname "$0")/.." || exit 1

ATTEMPTS=${ATTEMPTS:-6}
BOOT_TIMEOUT=${BOOT_TIMEOUT:-420}
GUARD_TIMEOUT=${GUARD_TIMEOUT:-600}
SIG="2d40fffc504f6700"

wait_halt() {  # $1 = deadline sec
    local start=$SECONDS
    while [ $((SECONDS - start)) -lt "$1" ]; do
        JT_WAIT=8 tools/jt.sh "halt-status" 2>&1 | grep -q 'effective=1' && { echo 1; return; }
    done
    echo 0
}

dump_words() { # $1 addr  $2 count -> hex byte stream on stdout; echoes base addr on fd 3
    # MUST align down to a 4-byte boundary.  dump-mem REFUSES an unaligned
    # address ("the JTAG-AXI master is 32-bit word-addressed and would silently
    # read ... instead") and returns no data.  A 68k A7 is routinely 2-mod-4
    # (0x007bd8e2 here), so passing the raw SP yields an empty scan and a
    # silent "no candidates" -- which is exactly how an earlier run of this
    # script threw away two good SetHandleSize(-24) traps.
    # Callers that need the base for offset math MUST align identically.
    local addr=$(( $1 & ~3 ))
    JT_WAIT=60 tools/jt.sh "dump-mem $(printf '0x%08X' $addr) $2" 2>&1 \
      | grep -oE '^> mem 0x[0-9A-Fa-f]+ = 0x[0-9a-fA-F]{8}' \
      | sed -E 's/.*= 0x//' | tr -d '\n'
}

for a in $(seq 1 "$ATTEMPTS"); do
    echo "=== attempt $a/$ATTEMPTS: trap SetHandleSize(-24) ==="
    JT_WAIT=20 tools/jt.sh "break-pc off"                   >/dev/null 2>&1
    JT_WAIT=20 tools/jt.sh "atrap 0 0xA024 d0 0xFFFFFFE8"   >/dev/null 2>&1
    JT_WAIT=30 tools/jt.sh "vio-hard-reset"                 >/dev/null 2>&1

    [ "$(wait_halt "$BOOT_TIMEOUT")" = "1" ] || { echo "  no trap in ${BOOT_TIMEOUT}s"; continue; }
    echo "  trapped: $(JT_WAIT=15 tools/jt.sh "atrap status" 2>&1 | grep -oE 'HIT: slot=[0-9]+ opword=0x[0-9A-Fa-f]+ pc=0x[0-9A-Fa-f]+')"

    JT_WAIT=25 tools/jt.sh "dcache-op push" >/dev/null 2>&1
    SP=$(JT_WAIT=30 tools/jt.sh "live-arch" 2>&1 | grep -oE '^> A7 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | head -1)
    echo "  A7=$SP — scanning stack for Finder-range return addresses"

    STK=$(dump_words "$SP" 256)
    CANDS=$(python3 - "$STK" <<'EOF'
import sys,re
h=sys.argv[1]
b=bytes.fromhex(h) if len(h)%2==0 else b''
seen=[]
for i in range(0,len(b)-3,2):
    v=int.from_bytes(b[i:i+4],'big')
    if 0x00700000 <= v <= 0x007a0000 and v%2==0:
        seen.append(v)
print(' '.join(hex(v) for v in dict.fromkeys(seen)))
EOF
)
    echo "  candidates: $CANDS"

    GUARD=""
    for c in $CANDS; do
        # Align identically to dump_words, so the signature offset math below
        # is relative to the address actually read.
        BASE=$(printf "0x%08X" $(( (c - 0x400) & ~3 )))
        BLOB=$(dump_words "$BASE" 320)
        IDX=$(python3 - "$BLOB" "$SIG" "$BASE" <<'EOF'
import sys
blob,sig,base=sys.argv[1],sys.argv[2],int(sys.argv[3],16)
i=blob.find(sig)
print(hex(base + i//2) if i>=0 and i%2==0 else "")
EOF
)
        if [ -n "$IDX" ]; then
            GUARD=$(printf "0x%08X" $(( IDX + 6 )))
            echo "  signature at $IDX -> guard beqw at $GUARD (via candidate $c)"
            break
        fi
    done
    [ -n "$GUARD" ] || { echo "  no guard signature found near any candidate"; continue; }

    JT_WAIT=20 tools/jt.sh "atrap 0 off"        >/dev/null 2>&1
    JT_WAIT=20 tools/jt.sh "break-pc $GUARD"    2>&1 | grep -oE 'slot=[0-9]+ target=0x[0-9A-Fa-f]+'
    JT_WAIT=30 tools/jt.sh "cont"               >/dev/null 2>&1

    for h in $(seq 1 40); do
        [ "$(wait_halt "$GUARD_TIMEOUT")" = "1" ] || { echo "  guard not re-entered in ${GUARD_TIMEOUT}s"; break; }
        G=$(JT_WAIT=15 tools/jt.sh "pc" 2>&1 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
        A=$(JT_WAIT=30 tools/jt.sh "live-arch" 2>&1)
        D0=$(echo "$A" | grep -oE '^> D0 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+')
        SR=$(echo "$A" | grep -oE '^> SR  = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+')
        Z=$(( (${SR:-0} >> 2) & 1 ))
        JT_WAIT=25 tools/jt.sh "step" >/dev/null 2>&1
        NPC=$(JT_WAIT=15 tools/jt.sh "pc" 2>&1 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
        FALL=$(printf "0x%08X" $(( $(printf '%d' "$G") + 4 )))
        printf "  hit %2d @ %s: D0=%s SR=%s Z=%d -> %s" "$h" "$G" "$D0" "$SR" "$Z" "$NPC"
        if [ "$(printf '%d' "$NPC")" = "$(printf '%d' "$FALL")" ] && [ "$D0" = "0x00000000" ]; then
            echo "   *** MISBRANCH CAPTURED ***"
            [ "$Z" = "1" ] && echo "      Z=1 -> flags CORRECT; BRANCH RESOLUTION is wrong" \
                           || echo "      Z=0 -> flags ALREADY WRONG upstream (MOVE's CCR write / CCR CDB)"
            exit 0
        fi
        echo "   (correct)"
        JT_WAIT=30 tools/jt.sh "cont" >/dev/null 2>&1
    done
done
echo "=== no misbranch captured ==="
exit 1
