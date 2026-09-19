#!/bin/bash
# catch_btree_clobber.sh — catch the code that overwrites the File Manager's
# B-tree node cache with a gamma/colour ramp.
#
# FINDING (2026-08-18): the extents-file BTCB (0x00010340) and the catalog BTCB
# (0x000103A0) BOTH point at node buffer 0x00010C60, and that buffer contains a
# monotonically increasing byte ramp -- a gamma/colour table -- instead of an
# HFS B-tree node:
#     HW:       0000000000000001 010000080005090b 0e10131517191b1d ...
#     expected: 0000000000000000 ff010004000007ff 0000002e0050025d ...
# The file-46 resource-fork extent record is absent from it.
#
# With a corrupt cached node the extents lookup for allocation block 324 fails
# IN MEMORY, so `_Read` of the System file's resource fork at offset 0x288896
# returns ioErr and issues NO disk request -- which is exactly what was measured
# (deterministic across two bitstreams, SD idle, zero SCSI/SD errors).
#
# 0x00010C6C holds 0x0005090B in the ramp, a distinctive value, so watch for
# THAT exact longword store rather than any write (the buffer is also written
# legitimately whenever a real B-tree node is loaded).
# Watch config survives reset, so arm then reset to catch it whenever it lands.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-300}
TRIES=${TRIES:-3}
ADDR=0x00010C6C
VAL=0x0005090B
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
wait_halt() { local s=$SECONDS
    while [ $((SECONDS-s)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0; done; return 1; }

for t in $(seq 1 "$TRIES"); do
    echo "=== attempt $t/$TRIES ==="
    for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
        jt "$c" >/dev/null; done
    jt "watch 0 $ADDR w value $VAL lanes 0xF" 25 | grep -E "armed|ERROR" | sed 's/^/  /'
    jt "vio-hard-reset" 30 >/dev/null
    echo "  armed before reset; waiting ${WAIT}s for the ramp store"

    if ! wait_halt "$WAIT"; then echo "  no hit in ${WAIT}s; retrying"; continue; fi
    PC=$(jt "pc" 15 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)
    echo "  *** CLOBBER CAUGHT ***"
    jt "watch status" 20 | grep -oE 'wp HIT: .*' | sed 's/^/    /'
    echo "    halted PC = $PC"
    jt "live-arch" 40 | grep -E "^> (D0|D1|D2|D3|A0|A1|A2|A3|A5|A6|A7|SR|PC) " | sed 's/^/    /'
    echo "    --- pc-trace ---"
    jt "pc-trace 40" 40 | grep '^> trace' | tail -22 | sed 's/^/    /'
    # Identify the CALLER *in this same halt*.  Disassembling a caller address
    # taken from an earlier boot's pc-trace is invalid -- these are heap
    # addresses and the code moves between boots (that mistake produced a
    # garbage decode once already).
    ARCH=$(jt "live-arch" 40)
    SP=$(echo "$ARCH" | grep -oE "^> A7 = 0x[0-9a-f]+" | grep -oE "0x[0-9a-f]+" | head -1)
    echo "    --- stack from A7=$SP (return addresses) ---"
    SPA=$(printf "0x%08X" $(( $(printf "%d" "$SP") & ~3 )))
    jt "dump-mem $SPA 20" 60 | grep "^> mem" | sed "s/^/      /"
    echo "    --- BlockMove copy registers ---"
    echo "$ARCH" | grep -E "^> (A0|A1|A2|D0|D1) " | sed "s/^/      /"
    # Identify the OWNER of this allocation, in this halt (the memory is reused
    # afterwards -- scanning it later finds nothing).
    A0=$(echo "$ARCH" | grep -oE "^> A0 = 0x[0-9a-f]+" | grep -oE "0x[0-9a-f]+" | head -1)
    echo "    --- source region around A0=$A0 (what is being copied) ---"
    SRCB=$(printf "0x%08X" $(( ($(printf "%d" "$A0") - 0x60) & ~3 )))
    jt "dump-mem $SRCB 48" 90 | grep -oE "= 0x[0-9a-fA-F]{8}" | sed "s/= 0x//" | tr -d "\n" > /tmp/clob_src.hex
    echo "    --- caller region (driver header / name?) ---"
    jt "dump-mem 0x00010A00 96" 120 | grep -oE "= 0x[0-9a-fA-F]{8}" | sed "s/= 0x//" | tr -d "\n" > /tmp/clob_drv.hex
    python3 - <<'PYEOF'
def scan(path, base, label):
    try: h=open(path).read().strip()
    except Exception: return
    try: b=bytes.fromhex(h)
    except Exception: return
    runs=[]; i=0
    while i < len(b):
        if 32 <= b[i] < 127:
            j=i
            while j < len(b) and 32 <= b[j] < 127: j+=1
            if j-i >= 3: runs.append((base+i, b[i:j].decode("ascii")))
            i=j
        else: i+=1
    print(f"    [{label}] {len(b)} bytes, {len(runs)} ascii runs")
    for a,t in runs[:12]:
        mark = "  <== DRIVER NAME" if t.startswith(".") else ""
        print(f"      0x{a:08X} {t!r}{mark}")
scan("/tmp/clob_src.hex", 0, "source")
scan("/tmp/clob_drv.hex", 0x00010A00, "caller region")
PYEOF
    # Walk the heap block layout around the collision.  Non-relocatable block
    # headers carry the ZONE pointer (SysZone = 0x00002000) at +4, so scanning a
    # bulk dump for that reveals block boundaries cheaply -- far faster than
    # chasing the chain one dump-mem at a time.
    echo "    --- heap block headers around 0x00010C60 (zone ptr 0x00002000 at +4) ---"
    jt "dump-mem 0x00010800 512" 240 | grep -oE "= 0x[0-9a-fA-F]{8}" | sed "s/= 0x//" | tr -d "\n" > /tmp/zone_scan.hex
    python3 - <<'PYEOF'
base=0x00010800
try: b=bytes.fromhex(open('/tmp/zone_scan.hex').read().strip())
except Exception as e: print("      (dump failed)"); raise SystemExit
hits=[]
for i in range(0, len(b)-8, 2):
    zone=int.from_bytes(b[i+4:i+8],'big')
    hdr =int.from_bytes(b[i:i+4],'big')
    tag=(hdr>>24)&0xFF; size=hdr&0xFFFFFF
    if zone==0x00002000 and 0x20<=tag<=0x7F and 8<size<0x40000:
        hits.append((base+i, tag, size))
print(f"      {len(hits)} candidate block headers")
for a,t,sz in hits[:24]:
    data=a+8
    end=data+sz
    mark=""
    if data <= 0x00010C60 < end: mark="   <== CONTAINS 0x00010C60"
    if data == 0x00010C60:       mark="   <== STARTS AT 0x00010C60"
    print(f"      hdr@0x{a:08X} tag=0x{t:02X} size=0x{sz:06X} data=0x{data:08X}..0x{end:08X}{mark}")
print()
print("      Two headers whose data ranges OVERLAP = the allocator carved inside")
print("      a live block. One containing block + one starting at 0x00010C60 is")
print("      exactly that.")
PYEOF
    echo
    echo "    A1 is the copy DESTINATION.  If it equals the File Manager buffer"
    echo "    base exactly, the caller holds a pointer to memory the FM owns."
    echo "    Disassemble the return address above THAT LIES IN RAM CODE to find"
    echo "    the culprit -- do it now, in this halt, not from a later boot."
    exit 0
done
echo "=== never caught the clobber ==="; exit 1
