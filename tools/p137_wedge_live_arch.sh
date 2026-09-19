#!/bin/bash
# p137_wedge_live_arch.sh — read the WEDGED machine's own architectural state.
#
# WHY THIS EXISTS RATHER THAN `live-arch`
#   `live-arch` requires an acknowledged effective halt, and on the p137 wedge the
#   halt DOES NOT LAND (`ERROR halt timeout ... effective=0`): the stop manager
#   waits for the current macro to commit, and nothing commits.  So `live-arch`,
#   `coherent-dump` and `dcache-op push` are all unavailable on the very machine
#   we need to read.
#
#   But the live readback registers those commands consume are plain debug-block
#   registers at fixed offsets (jtag_repl.tcl:857-876, DBG_BASE=0x50900000), fed
#   from the snap path -> cRAT -> PRF.  A plain `r` reaches them with no halt.
#
# WHY A FORCED READ IS TRUSTWORTHY *HERE* AND NOWHERE ELSE
#   The standard warning is that the snap chain is read register-by-register while
#   the pipeline retires, so forced values are stale, torn, or both.  That premise
#   is FALSE on this machine: the free-running retired-macro counter
#   (0x50901008/0x5090100C) does not advance by a single macro in 60 s.  A pipeline
#   that retires nothing cannot tear a register read.
#
#   That is an argument, not a licence, so it is CHECKED rather than assumed: every
#   register is read TWICE and the pair must agree, and the retired-macro counter is
#   read before and after the whole sweep and must be unchanged.  If either check
#   fails the values are reported as UNRELIABLE.
#
# COHERENCE OF THE MEMORY READS -- STATED PER READ, NOT GLOBALLY
#   `r`/`dump-mem` are JTAG-AXI reads that BYPASS the D-cache, and `dcache-op push`
#   needs a halt we cannot get.  Therefore:
#     - a read of ROM space (0x408xxxxx) is sound: the CPU does not write ROM, so no
#       dirty line can shadow it, and it can be cross-checked against the on-disk
#       image byte-for-byte;
#     - a read of the RAM vector table is NOT coherent and is labelled so.  If the
#       ROM has relocated VBR into RAM and the table is still dirty in L1D, this
#       returns stale DDR.  It is reported as raw evidence with that caveat, never
#       as a verdict.
#
# ARMS NOTHING.  Reads only.  No reset, no load-bit -- this is meant to run against
# an ALREADY-WEDGED board so the reproduction is not disturbed.
set -u
cd "$(dirname "$0")/.." || exit 1

WANT_ID=${WANT_ID:?set WANT_ID}
jt () { JT_WAIT=${2:-15} JT_TAG=wedge-arch tools/jt.sh "$1" 2>&1; }
rd () { jt "r $1" 15 | grep -oiE '= *0x[0-9a-f]{8}' | tail -1 | grep -oiE '0x[0-9a-f]{8}'; }

live=$(jt "build-id" 20 | grep -oiE 'build_id = 0x[0-9a-f]+' | head -1 | grep -oiE '0x[0-9a-f]+')
echo "live build_id: ${live:-UNKNOWN} (want $WANT_ID)"
[ "${live,,}" = "${WANT_ID,,}" ] || { echo "ABORT: wrong bitstream live"; exit 1; }

pc=$(jt "halt-status" 10 | grep -oE 'pc_live=0x[0-9a-f]+' | head -1 | sed 's/pc_live=//')
echo "pc_live: $pc"

m_before=$(rd 0x50901008)
echo "retired-macro counter BEFORE sweep: $m_before"

echo
echo "--- live architectural / control state (each read TWICE, must agree) ---"
declare -A R=(
  [VBR]=0x50902100 [SR]=0x50902104 [A7]=0x50902108 [USP]=0x5090210C
  [PC]=0x50902190
  [MMU_TC]=0x50902160 [MMU_DTT0]=0x50902164 [MMU_DTT1]=0x50902168
  [MMU_ITT0]=0x5090216C [MMU_ITT1]=0x50902170
  [MMU_SRP]=0x50902174 [MMU_URP]=0x50902178 [MMUSR]=0x50902194
  [MSP]=0x5090217C [ISP]=0x50902180 [CACR]=0x50902184
)
VBRV=""
for k in VBR SR PC A7 USP MSP ISP CACR MMU_TC MMU_ITT0 MMU_ITT1 MMU_DTT0 MMU_DTT1 MMU_SRP MMU_URP MMUSR; do
    a=$(rd "${R[$k]}"); b=$(rd "${R[$k]}")
    if [ "$a" = "$b" ] && [ -n "$a" ]; then st="stable"; else st="**TORN/UNRELIABLE** (second read $b)"; fi
    printf "  %-9s = %-12s %s\n" "$k" "${a:-READ-FAILED}" "$st"
    [ "$k" = VBR ] && VBRV=$a
done

m_after=$(rd 0x50901008)
echo
echo "retired-macro counter AFTER sweep:  $m_after"
if [ "$m_before" = "$m_after" ]; then
    echo "  -> UNCHANGED across the sweep: the pipeline retired nothing while these were read,"
    echo "     so the register-by-register tearing caveat does not apply to this sample."
else
    echo "  -> CHANGED ($m_before -> $m_after): the CPU retired during the sweep."
    echo "     TREAT EVERY VALUE ABOVE AS UNRELIABLE."
fi

echo
echo "--- SR decode ---"
if [ -n "${R[SR]:-}" ]; then
    srv=$(rd 0x50902104)
    if [ -n "$srv" ]; then
        v=$(printf '%d' "$srv")
        echo "  SR = $srv   T=$(( (v>>14)&3 ))  S=$(( (v>>13)&1 ))  M=$(( (v>>12)&1 ))  IPL=$(( (v>>8)&7 ))  CCR=0x$(printf '%02X' $((v&0x1f)))"
        echo "  interrupt mask IPL=$(( (v>>8)&7 ))  -> level-1 VIA1 60Hz IRQ is $( [ $(( (v>>8)&7 )) -ge 1 ] && echo MASKED || echo ENABLED )"
    fi
fi

echo
echo "--- vector 10 (A-line) table entry ---"
if [ -n "${VBRV:-}" ]; then
    v10=$(printf '0x%08X' $(( $(printf '%d' "$VBRV") + 0x28 )))
    echo "  VBR+0x28 = $v10"
    case "${VBRV,,}" in
        0x40*) coh="ROM space: no dirty D-cache line can shadow it, and it is cross-checkable against files/420dbff3.rom" ;;
        *)     coh="**NON-COHERENT**: RAM space read over JTAG-AXI, which BYPASSES the D-cache. dcache-op push needs a halt that will not land here." ;;
    esac
    echo "  coherence: $coh"
    jt "dump-mem $v10 4" 25 2>&1 | grep -E '^> mem' | sed 's/^/  /'
    tgt=$(jt "dump-mem $v10 1" 20 2>&1 | grep -E '^> mem' | head -1 | grep -oiE '0x[0-9a-f]{8}$')
    echo "  vector 10 target: ${tgt:-UNREADABLE}"
    if [ -n "${tgt:-}" ]; then
        echo "  --- 16 words at the vector-10 target ---"
        case "${tgt,,}" in
            0x408*) echo "  coherence: ROM space, sound + cross-checkable" ;;
            *)      echo "  coherence: **NON-COHERENT** (see above)" ;;
        esac
        jt "dump-mem $(printf '0x%08X' $(( $(printf '%d' "$tgt") & ~3 ))) 16" 30 2>&1 | grep -E '^> mem' | sed 's/^/  /'
    fi
fi

echo
echo "--- the handler the machine ACTUALLY dispatched to, per its own exception ring ---"
jt "exc-ring" 25 2>&1 | grep -E 'exc\[' | head -4 | sed 's/^/  /'
echo "  (16 words at 0x408099B0 -- ROM space, sound)"
jt "dump-mem 0x408099B0 16" 30 2>&1 | grep -E '^> mem' | sed 's/^/  /'

echo
echo "=== live-arch sweep COMPLETE ($(date -Is)) ==="
