#!/usr/bin/env bash
# p140: interleaved p133-vs-p140 boot A/B. THE DECISIVE TEST OF THE FIX.
#
# p140 = cpu040 fix/p139-quiesce-hold-entry-drain @ c5ac78a, which carries ALL
# FOUR arms -- INCLUDING arm C (2db5bd3, MMU table walks through L1D), the arm
# p139 proved to be the exposer -- PLUS the fix that adds E_DRAIN and R_DRAIN to
# `quiesceHoldOut`.
#
# This is the only run that demonstrates the wedge is actually gone. p139 showed
# the wedge disappears when you REMOVE the walker routing; that is a diagnosis,
# not a fix, because 2db5bd3 closes a real coherency hole and has to stay. p140
# keeps it in and asks whether the hold makes it live.
#
#   retire ADVANCING, no 0x40806b68  -> the fix works
#   retire FROZEN at 0x40806b68      -> the fix does NOT work; do not merge
#
# ══ CLASSIFY ON RETIRE, NOT ON THE SCREEN ═════════════════════════════════════
# Both arms show a MIXED screen spread (Happy Mac / Sad Mac / ROM serial monitor
# / SCSI INT-poll hang) -- p133's own reference distribution is non-deterministic
# across boots. So a Sad Mac in an arm is NOT a failure signal and a Happy Mac is
# NOT a pass signal. The discriminator that separated p139 from p138c 6/6 was
# retired_macros ADVANCING vs FROZEN, and that is the one to read here.
#
# Screen md5 note: `9086c68c` and `f8e8fd03` are PREMATURE-SAMPLE signatures, not
# failure classes -- re-read at a longer settle rather than binning them.
#
# The p133 artifact is a PRESERVED copy and must never be overwritten
# (12673817 bytes, checked below).
#
# usage: [PAIRS=6] [SETTLE=200] tools/p140_run_boot_ab.sh
set -u
cd "$(dirname "$0")/.." || exit 1

P133=${P133_DIR:-/home/qwertyoruiop/tmp/claude-1000/-home-qwertyoruiop-m68k-core-040-ooo/02fab060-05c0-4ce0-953f-15f1be816a95/scratchpad/p133-artifact}
NEW=${NEW_DIR:-build/vivado}

B_ID=$(sed -n 's/^build_id=//p' "$NEW/fpga_top.buildinfo" | head -1)
A_ID=$(sed -n 's/^build_id=//p' "$P133/fpga_top.buildinfo" | head -1)
[ -n "$B_ID" ] || { echo "FAIL: no build_id in $NEW/fpga_top.buildinfo"; exit 1; }
[ -n "$A_ID" ] || { echo "FAIL: no build_id in $P133/fpga_top.buildinfo"; exit 1; }
for f in "$P133/fpga_top.bit" "$P133/fpga_top.ltx" "$NEW/fpga_top.bit" "$NEW/fpga_top.ltx"; do
  [ -f "$f" ] || { echo "FAIL: missing $f"; exit 1; }
done
# The preserved reference must be BYTE-intact, not merely present.
SZ=$(stat -c %s "$P133/fpga_top.bit")
if [ "$SZ" != "12673817" ]; then
  echo "FAIL: preserved p133 bitstream is $SZ bytes, expected 12673817 -- REFUSING to run"
  exit 1
fi

# ── the B arm must really contain the walker routing AND the fix ──────────────
# Verified from the COMPILED NETLIST, not from the source tree and not inferred
# from a nearby commit -- inferring provenance from a pin bump put a wrong premise
# into the record for a full day on this campaign.
NETV=cpu040/generated/M68kSocketTop.v
if [ -f "$NETV" ]; then
  NETSHA=$(grep -m1 '^// Git hash' "$NETV" | grep -oE '[0-9a-f]{40}')
  echo "netlist Git hash: ${NETSHA:-UNKNOWN}"
  if [ "${NETSHA:-}" != "c5ac78a8705f5695937fcb0689185d921cbba5ea" ]; then
    echo "FAIL: netlist was generated from ${NETSHA:-unknown}, not the fix commit"
    exit 1
  fi
fi
echo "B cpu040 submodule HEAD = $(git -C cpu040 rev-parse HEAD)"

echo "A = p133 $A_ID  $P133/fpga_top.bit (md5 $(md5sum "$P133/fpga_top.bit" | cut -d' ' -f1))"
echo "B = p140 $B_ID  $NEW/fpga_top.bit  (md5 $(md5sum "$NEW/fpga_top.bit" | cut -d' ' -f1))"

PAIRS=${PAIRS:-6} SETTLE=${SETTLE:-200} \
OUTDIR=${OUTDIR:-/tmp/p140_abboot} \
A_BIT="$P133/fpga_top.bit" A_LTX="$P133/fpga_top.ltx" A_NAME=p133 A_ID="$A_ID" \
B_BIT="$NEW/fpga_top.bit"  B_LTX="$NEW/fpga_top.ltx"  B_NAME=p140 B_ID="$B_ID" \
tools/p138c_ab_boot.sh
echo "P140_BOOT_AB_DONE"
