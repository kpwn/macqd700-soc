#!/bin/bash
# dis_at.sh <addr> [words] — read memory over JTAG and disassemble it as m68k.
#
# For reading Finder/OS code off a halted board.  The Finder relocates every
# boot, so a captured PC is only meaningful if you can immediately see the code
# around it; that is what this is for.
#
# Do NOT run this while another script is driving the REPL -- both would
# interleave commands on the same Vivado session and corrupt each other's
# output.
#
# dump-mem REFUSES an unaligned address (it is a 32-bit word-addressed master),
# so the base is aligned down here and --adjust-vma is set to the ALIGNED base,
# otherwise every printed address is silently off by 1..3 bytes.

set -o pipefail
cd "$(dirname "$0")/.." || exit 1

ADDR=${1:?usage: dis_at.sh <addr> [words] [--back N]}
WORDS=${2:-48}
BASE=$(( $(printf '%d' "$ADDR") & ~3 ))

HEX=$(JT_WAIT=${JT_WAIT:-60} tools/jt.sh "dump-mem $(printf '0x%08X' $BASE) $WORDS" 2>&1 \
      | grep -oE '^> mem 0x[0-9A-Fa-f]+ = 0x[0-9a-fA-F]{8}' \
      | sed -E 's/.*= 0x//' | tr -d '\n')

if [ -z "$HEX" ]; then
    echo "dis_at: no data back from dump-mem at $(printf '0x%08X' $BASE)" >&2
    echo "        (board halted? another script holding the REPL?)" >&2
    exit 1
fi

BIN=$(mktemp /tmp/dis_at.XXXXXX.bin)
python3 -c "
import sys,binascii
open('$BIN','wb').write(binascii.unhexlify('$HEX'))
"
m68k-linux-gnu-objdump -b binary -m m68k:68040 -D \
    --adjust-vma=$(printf '0x%08X' $BASE) "$BIN" \
  | sed -n '/^ /p'
rm -f "$BIN"
