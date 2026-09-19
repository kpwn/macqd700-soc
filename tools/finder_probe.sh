#!/bin/bash
# finder_probe.sh — signature-driven instrumentation for the 7.5.3 Finder bug.
#
# WHY THIS EXISTS: every routine involved (the Finder's RemoveCommands clones,
# the extension code at 0x79xxxx, the duplicate-registration search) is loaded
# into the heap and lands at a DIFFERENT ADDRESS on every boot AND on every
# bitstream. Arming absolute addresses observed on an earlier boot silently
# lands on unrelated instructions and yields confident nonsense — that happened
# repeatedly (an "archive reproduces it" result was pure artifact; a dup-search
# probe fired once and never again). So: ALWAYS re-locate by byte signature
# INSIDE the boot you are measuring, then arm.
#
# Usage:  tools/finder_probe.sh locate            # scan + print this boot's addresses
#         tools/finder_probe.sh guard             # arm guard fall-throughs
#         tools/finder_probe.sh dup               # arm the duplicate-search FOUND arm
#
# Prereq: the machine must already be booted far enough that the code is
# resident (an A-trap on _SetHandleSize(-24) or 0xA860 is a good way to get
# there). Always `dcache-op push` before dumping — JTAG reads DDR and is not
# coherent with the write-back D-cache.

cd "$(dirname "$0")/.."
SCAN_BASE=${SCAN_BASE:-0x00740000}
SCAN_WORDS=${SCAN_WORDS:-16384}

# guard  : movel %d0,%fp@(-4) ; addqw #8,%sp ; beqw     -> guard=+6 fallthru=+10
SIG_GUARD=2d40fffc504f6700
# dupsrch: linkw %fp,#0 ; moveml %d6-%d7/%a3-%a4,-(sp) ; moveal %fp@(8),%a3
SIG_DUP=4e56000048e70318266e0008

locate() {
  JT_WAIT=25 tools/jt.sh "dcache-op push" >/dev/null 2>&1
  : > /tmp/fp_scan.raw
  JT_WAIT=200 tools/jt.sh "dump-mem $SCAN_BASE $SCAN_WORDS" >/dev/null 2>&1
  # dump-mem streams for a long time; wait for the REPL output to go quiet
  local prev=0 stable=0 st=$SECONDS
  while [ $((SECONDS-st)) -lt 420 ]; do
    local cur=$(wc -c < /tmp/jtag_out 2>/dev/null)
    if [ "$cur" = "$prev" ]; then stable=$((stable+1)); else stable=0; fi
    [ "$stable" -ge 3 ] && break
    prev=$cur; sleep 8 2>/dev/null || true
  done
  python3 - "$SIG_GUARD" "$SIG_DUP" <<'PY'
import re, sys
mem={}
for ln in open('/tmp/jtag_out', errors='ignore'):
    m=re.match(r'> mem (0x[0-9A-Fa-f]+) = 0x([0-9a-fA-F]{8})', ln.strip())
    if m:
        a=int(m.group(1),16); v=int(m.group(2),16)
        for k in range(4): mem[a+k]=(v>>(8*(3-k)))&0xFF
if not mem: raise SystemExit("no memory captured")
lo,hi=min(mem),max(mem)
blob=bytearray(mem.get(a,0) for a in range(lo,hi+1))
def find(sig):
    s=bytes.fromhex(sig); out=[]; i=0
    while True:
        j=blob.find(s,i)
        if j<0: break
        out.append(lo+j); i=j+1
    return out
g=find(sys.argv[1]); d=find(sys.argv[2])
print(f"region {lo:#x}..{hi:#x}")
print("GUARD sites   :", [hex(x) for x in g])
print("  guard beqw  :", [hex(x+6) for x in g])
print("  fallthrough :", [hex(x+10) for x in g])
print("DUPSEARCH     :", [hex(x) for x in d])
print("  found-arm ~ :", [hex(x+0x2c) for x in d], "(verify by disassembly)")
PY
}

case "${1:-locate}" in
  locate) locate ;;
  guard)  locate | sed -n 's/^  fallthrough : //p' ;;
  dup)    locate | sed -n 's/^DUPSEARCH     : //p' ;;
  *) echo "usage: $0 {locate|guard|dup}"; exit 2 ;;
esac
