#!/bin/bash
# catch_sysctlcptr.sh — catch the write that sets SysCtlCPtr (lowmem 0x380) to
# the control-cache buffer, and read that block's TRUE size at that instant.
#
# Polling 0x380 is useless: it is non-zero for only ~1 s (MAME: set at t=7.0s,
# cleared by t=8.0s), so a ~6 s poll reads 0 and proves nothing.
# Value-filter on 0x00010C60 -- the value the ROM was observed copying out of
# 0x380 into vcbCtlBuf at pc=0x4080f7da -- so the ROM RAM test (which writes the
# 6db patterns to this address) does NOT halt the boot three times.
#
# GOAL: compare the control-cache block SIZE against MAME's.
#   MAME: SysCtlCPtr=0x0000D460, hdr tag=0x4C size=0x0021E0 = 8672 bytes
#   HW  : previously observed hdr tag=0x4C size=0x000130 = 304 bytes
# 304 bytes cannot hold one 512-byte B-tree node, which would explain the
# garbage the extents lookup reads.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-260}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
rdl() { jt "dump-mem $1 1" 25 | grep -oE '= 0x[0-9a-fA-F]{8}' | head -1 | sed 's/= 0x//'; }

for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
    jt "$c" >/dev/null; done
jt "watch 0 0x00000380 w value 0x00010C60 lanes 0xF" 25 | grep -E "armed|ERROR" | sed 's/^/  /'
jt "vio-hard-reset" 30 >/dev/null
echo "  waiting ${WAIT}s for SysCtlCPtr <- 0x00010C60"

s=$SECONDS; H=0
while [ $((SECONDS-s)) -lt "$WAIT" ]; do
    jt "halt-status" 8 | grep -q 'effective=1' && { H=1; break; }
done
[ "$H" = 1 ] || { echo "  never fired (SysCtlCPtr took a different value this boot)"; exit 1; }

echo "  *** CAUGHT SysCtlCPtr being set ***"
jt "watch status" 20 | grep -oE 'wp HIT: .*' | sed 's/^/    /'
echo "    halted PC = $(jt "pc" 15 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)"
jt "dcache-op push" 30 >/dev/null      # make the JTAG view coherent
H1=$(rdl 0x00010C58); H2=$(rdl 0x00010C5C)
echo "    control-cache block header at 0x00010C58: $H1 / $H2"
python3 -c "
h=int('${H1:-0}',16); z=int('${H2:-0}',16)
sz=h&0xFFFFFF
print('    tag=0x%02X  size=0x%06X (%d bytes)  zone=%08X' % ((h>>24)&0xFF, sz, sz, z))
print('    MAME: tag=0x4C size=0x0021E0 (8672 bytes)')
print()
if sz and sz < 512:
    print('    => TOO SMALL to hold one 512-byte B-tree node. This is a real,')
    print('       quantitative divergence from MAME and a direct mechanism for')
    print('       the garbage node the extents lookup reads.')
elif sz:
    print('    => size is %d bytes; compare against MAME 8672 before concluding.' % sz)
"
echo "    --- pc-trace ---"
jt "pc-trace 24" 40 | grep '^> trace' | tail -14 | sed 's/^/    /'
