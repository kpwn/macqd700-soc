#!/bin/bash
# catch_dup_flag_write.sh — capture the instant the Finder sets its
# duplicate-registration error flag at [A5-3692].
#
# WHY THIS OBSERVABLE.  Measured 2026-08-18 on the SAME 7.5.3 image:
#     MAME (boots fine) : [A5-3692] == 0x00000000 for the whole run
#     our hardware      : [A5-3692] == 0xff000000
# It is the only differential so far measured on the same observable on both
# machines with a validated harness.  Everything downstream of it (the
# SetTableSize 0+4-28 = -24, the _SetHandleSize(h,-24) grow-zone thrash) was
# measured on HW only.
#
# WHY A WATCHPOINT AND NOT break-pc.  The Finder's code lands at a different
# address nearly every boot, so break-pc on the `st %a5@(-3692)` error arm
# needs an address I cannot know in advance (this burned 8/8 boots earlier).
# The DATA address is stable within an A5 world, and `watch` compares the
# physical data address -- so it does not care where the code moved to.
# `watch` config also SURVIVES CPU RESET (see jtag_repl.tcl), which is what
# makes a one-shot per-boot write catchable at all.
#
# WHAT THE CAPTURE DECIDES.  At the halt we get the PC of the error arm FOR
# THIS BOOT plus full arch state, which splits the two live hypotheses:
#   - the searched table really does contain a duplicate  -> DATA corruption
#     upstream, and the compare was right to match
#   - memory says no match but the code took the equal path -> CPU bug in the
#     search loop (compare/flags, or a stale load letting the walk overrun
#     its terminator)
#
# LESSONS BAKED IN (each of these cost a run earlier in this session):
#   - jt.sh returns on output-quiet, NOT after JT_WAIT.  Every wait below is a
#     wall-clock deadline; poll counts are never timeouts.
#   - dump-mem REFUSES an unaligned address, returning nothing.  Align down.
#   - do NOT `pkill -f` this script's own name; the pattern matches the
#     calling shell and kills it (exit 143/144).
#   - do NOT `dcache-op push` before reading; it destroys the evidence.

cd "$(dirname "$0")/.." || exit 1

A5_OFF=3692                       # `st %a5@(-3692)`
WEDGE_WAIT=${WEDGE_WAIT:-330}     # boot-to-wedge is ~4-5 min
CATCH_WAIT=${CATCH_WAIT:-420}

jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }

get_a5() { jt "live-arch force" 40 | grep -oE '^> A5 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | head -1; }

read_long() {  # $1 = address (aligned down by caller if needed)
    jt "dump-mem $(printf '0x%08X' $1) 1" 30 \
      | grep -oE '= 0x[0-9a-fA-F]{8}' | head -1 | sed 's/= //'
}

wait_halt() {  # $1 = wall-clock deadline in seconds
    local start=$SECONDS
    while [ $((SECONDS - start)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0
    done
    return 1
}

echo "=== phase 1: halt on _SetHandleSize(-24) and read A5 reliably ==="
# WHY AN A-TRAP AND NOT `live-arch force`.  There is no manual CPU-halt command,
# and `live-arch force` on a RUNNING cpu relabels every line
# "UNRELIABLE A5 ?= 0x..." (values stale/torn, by design) -- so a grep for
# "A5 = " harvests nothing.  An A-trap halts on the OPCODE WORD, needs no
# address, and so is immune to the Finder relocating every boot.  The wedged
# machine calls _SetHandleSize(h,-24) thousands of times, so this halts fast
# AND re-confirms the -24 thrash in the same stop.
jt "watch 0 off"      >/dev/null
jt "watch 1 off"      >/dev/null
jt "break-pc off"     >/dev/null
jt "atrap 0 off"      >/dev/null
jt "atrap 1 off"      >/dev/null
jt "halt-clear"       >/dev/null
jt "atrap 0 0xA024 d0 0xFFFFFFE8" 25 | sed 's/^/  /'
jt "vio-hard-reset" 30 >/dev/null
echo "  booting; waiting up to ${WEDGE_WAIT}s for _SetHandleSize(h,-24)"

if ! wait_halt "$WEDGE_WAIT"; then
    echo "FAIL: no _SetHandleSize(-24) in ${WEDGE_WAIT}s."
    echo "      Either this boot did not reach the Finder, or -- note -- the"
    echo "      restored 7.5.3 image now behaves DIFFERENTLY, which would itself"
    echo "      be the finding.  Re-run before concluding anything."
    exit 1
fi
echo "  halted. atrap: $(jt "atrap status" 20 | grep -oE 'HIT: .*' | head -1)"

ARCH=$(jt "live-arch" 40)
A5=$(echo "$ARCH" | grep -oE '^> A5 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | head -1)
if [ -z "$A5" ]; then
    echo "FAIL: no reliable A5 at the halt. Raw live-arch was:"
    echo "$ARCH" | sed 's/^/    /' | head -25
    exit 1
fi
echo "  PC=$(jt "pc" 15 | grep -oE '0x[0-9A-Fa-f]{8}' | head -1)  A5=$A5"

FLAG=$(( $(printf '%d' "$A5") - A5_OFF ))
FLAG_AL=$(( FLAG & ~3 ))
V=$(read_long $FLAG_AL)
printf "  flag byte @ 0x%08X   longword @ 0x%08X = %s\n" "$FLAG" "$FLAG_AL" "$V"
case "$V" in
  0xff*|0xFF*) echo "  -> error flag IS set; the write happened. Arming to catch it." ;;
  "")          echo "  -> could not read the flag; aborting rather than arming on a guess"; exit 1 ;;
  *)           echo "  -> flag NOT set on this boot ($V); nothing to catch. Re-run." ; exit 2 ;;
esac

# Byte lane within the longword (see jtag_repl.tcl VALUE FILTER LANES).
SH=$(( 8 * (3 - (FLAG & 3)) ))
VAL=$(printf "0x%08X" $(( 0xFF << SH )))
LANES=$(printf "0x%X" $(( 1 << (3 - (FLAG & 3)) )))

echo
echo "=== phase 2: arm watchpoint on the flag write, then reset ==="
echo "  watch 0 $(printf '0x%08X' $FLAG) w value $VAL lanes $LANES"
jt "watch 0 $(printf '0x%08X' $FLAG) w value $VAL lanes $LANES" 25
jt "watch status" 20 | sed 's/^/  /'
jt "vio-hard-reset" 30 >/dev/null

echo "  waiting up to ${CATCH_WAIT}s for the flag write..."
if ! wait_halt "$CATCH_WAIT"; then
    echo "FAIL: no halt. Either this boot did not take the error path, or A5"
    echo "      moved (the watch address is A5-relative -- re-run phase 1)."
    exit 3
fi

echo
echo "=== phase 3: capture ==="
echo "--- halt-status ---";      jt "halt-status" 20 | sed 's/^/  /'
echo "--- watch status ---";     jt "watch status" 20 | sed 's/^/  /'
echo "--- pc ---";               jt "pc" 15          | sed 's/^/  /'
echo "--- live-arch ---";        jt "live-arch" 40   | sed 's/^/  /'
echo "--- pc-trace 64 ---";      jt "pc-trace 64" 40 | sed 's/^/  /'

SP=$(jt "live-arch" 40 | grep -oE '^> A7 = 0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | head -1)
if [ -n "$SP" ]; then
    SPA=$(printf "0x%08X" $(( $(printf '%d' "$SP") & ~3 )))   # dump-mem refuses unaligned
    echo "--- stack @ $SPA (A7=$SP) ---"
    jt "dump-mem $SPA 48" 60 | sed 's/^/  /'
fi
echo
echo "The halted PC is the error arm for THIS boot. Disassemble backwards from"
echo "it to find the search loop, then dump the table it walked and check by"
echo "hand whether a duplicate genuinely exists."
