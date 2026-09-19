#!/bin/bash
# hw_syscache_timeline.sh — sample the File Manager's system cache pointers from
# EARLY boot and validate each block header.  These are FIXED lowmem addresses
# (the only kind that is stable across boots on this machine):
#   0x378 SysBMCPtr  -> VCB+80  vcbMAdr
#   0x37C SysVolCPtr
#   0x380 SysCtlCPtr -> VCB+168 vcbCtlBuf
# They are non-zero only briefly during FM init, so sampling late reads 0 and
# tells you nothing (that mistake cost a run).
#
# MAME reference (same image):
#   t=7s  SysBMCPtr=0000CDE0 tag=0x40 size=0x000230
#         SysVolCPtr=0000D010 tag=0x44 size=0x000450
#         SysCtlCPtr=0000D460 tag=0x4C size=0x0021E0   <-- 8672 bytes
#   HW previously showed the control cache block as tag=0x4C size=0x130 = 304.
# 304 bytes cannot hold a 512-byte B-tree node.

cd "$(dirname "$0")/.." || exit 1
DUR=${DUR:-90}
jt() { JT_WAIT=${2:-12} tools/jt.sh "$1" 2>&1; }
rdl() { jt "dump-mem $1 1" 20 | grep -oE '= 0x[0-9a-fA-F]{8}' | head -1 | sed 's/= 0x//'; }

for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
    jt "$c" >/dev/null; done
echo "=== reset, then sample 0x378/0x37C/0x380 for ${DUR}s ==="
jt "vio-hard-reset" 30 >/dev/null
s=$SECONDS; LAST=""
while [ $((SECONDS-s)) -lt "$DUR" ]; do
    B=$(rdl 0x00000378); V=$(rdl 0x0000037C); C=$(rdl 0x00000380)
    CUR="$B/$V/$C"
    if [ "$CUR" != "$LAST" ] && [ -n "$C" ]; then
        LAST="$CUR"
        printf "  t=%3ds  SysBMCPtr=%s SysVolCPtr=%s SysCtlCPtr=%s\n" "$((SECONDS-s))" "$B" "$V" "$C"
        CN=$(printf '%d' "0x$C" 2>/dev/null || echo 0)
        if [ "$CN" -gt 4096 ] && [ "$CN" -lt 2147483647 ]; then
            H=$(rdl $(printf '0x%08X' $(( (CN - 8) & ~3 ))))
            H2=$(rdl $(printf '0x%08X' $(( (CN - 4) & ~3 ))))
            python3 -c "
h=int('${H:-0}',16); z=int('${H2:-0}',16)
print('           ctl-cache hdr=%08X/%08X  tag=0x%02X size=0x%06X (%d bytes)  zone=%08X'
      % (h,z,(h>>24)&0xFF,h&0xFFFFFF,h&0xFFFFFF,z))
print('           MAME had tag=0x4C size=0x0021E0 (8672 bytes)')"
        fi
    fi
done
