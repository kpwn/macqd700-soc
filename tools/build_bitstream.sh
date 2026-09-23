#!/bin/bash
# build_bitstream.sh — canonical KU5P bitstream build for the Mac Quadra 700 SoC.
#
# WHY THIS EXISTS
#
#   Historically the Makefile defaulted to `CPU ?= stub`, so a bare
#   `make impl` produced a bitstream with the CPU
#   *stub* instead of a real core.  That artifact is not obviously
#   broken -- it programs cleanly, VIO works, and JTAG-AXI reads DDR/ROM
#   correctly (`r 0x40800000` returns the ROM checksum) -- but there is no
#   CPU and no debug block, so:
#       dbg-caps    -> DBG_VERSION = 0x00000000
#       halt-status -> pc_live = 0x00000000, exc_count = 0
#       screen      -> never changes
#   which reads exactly like a dead/wedged board and costs a full ~50 min
#   rebuild to discover.  This happened on 2026-08-11.
#
#   It is worse than it sounds, for two reasons:
#     1. `make check-cpu-sync` is wrapped in `ifeq ($(CPU),m68k)`, so with the
#        default CPU=stub it prints "Nothing to be done" and validates NOTHING.
#        A green sync check does not mean the CPU sources were checked.
#     2. build/vivado/fpga_top.buildinfo DOES record a cpu= field, but only
#        since commit 44670e2 (2026-08-20, synth/vivado.tcl:2256).  Artifacts
#        older than that have none.  verify_artifacts below therefore reads
#        buildinfo FIRST and treats build/vivado/fpga_top.cpu as a second,
#        independent witness -- disagreement between them is a hard failure,
#        because it means the stamp and the artifact came from different runs.
#
#   Every other knob already defaults correctly (verified 2026-08-11):
#     NO_INCREMENTAL=1  L2C_ENABLE=1  VRAM_IN_DDR=1  REAL_FPGA_BUILD=1
#     USE_REAL_MIG=1    ENABLE_VIO=1  ENABLE_JTAG_AXI=1
#     TARGET_FREQ_MHZ=100  CORE_CLK_HZ=100_000_000  PB_CLK_HZ=50_000_000
#     VIDEO_SMOKE=0     BOOT_ROM_SECTORS=2048
#   They are passed explicitly below anyway so the build is reproducible even
#   if a default drifts.
#
# WHICH CORE
#   Both this script and the Makefile now default to m68k040, the OoO 68040.
#   The retired v1 CPU=m68k path is not supported by this script.
#   CPU=stub is REFUSED outright: a stub build is
#   the exact failure this script was written to prevent, so it must never be
#   reachable by default or by accident.
#
#   Before 2026-09-03 this script hard-pinned CPU=m68k with no override and
#   verified the stamp against the literal string "m68k", so the blessed
#   pipeline could not build the new core at all -- every cpu040 hardware
#   build had to be an ad-hoc manual command line outside this script.
#
# USAGE
#   tools/build_bitstream.sh                # full impl -> bitstream (cpu040)
#   tools/build_bitstream.sh --synth-only   # synth only (fast sanity check)
#   tools/build_bitstream.sh --verify       # verify existing artifacts, no build
#
# AFTER IT FINISHES, the artifact is NOT proven good until you program it and
# see DBG_VERSION != 0.  See the post-build checklist this script prints.

set -u -o pipefail
cd "$(dirname "$0")/.." || exit 1
ROOT=$(pwd)

MODE=impl
case "${1:-}" in
    --synth-only) MODE=synth ;;
    --verify)     MODE=verify ;;
    "")           ;;
    *) echo "usage: $0 [--synth-only|--verify]" >&2; exit 2 ;;
esac

BIT=build/vivado/fpga_top.blank.bit
LTX=build/vivado/fpga_top.ltx
INFO=build/vivado/fpga_top.buildinfo
CPUSTAMP=build/vivado/fpga_top.cpu
LOG=/tmp/build_bitstream_$(date +%Y%m%d_%H%M%S).log

# ── Which core ──────────────────────────────────────────────────────────
# Allowlist mirrors synth/vivado.tcl:97's own, minus stub.  Record whether
# the caller asked for a specific core so --verify can cross-check the
# request against the artifact rather than silently accepting either.
# ${CPU:+1} (not ${CPU+1}) so an exported-but-empty CPU= reads as "no
# request" instead of pinning the default and then demanding it match.
CPU_EXPLICIT=${CPU:+1}
CPU="${CPU:-m68k040}"
case "$CPU" in
    m68k040) ;;
    stub)
        echo "ERROR: CPU=stub builds a bitstream with NO CPU and no debug block." >&2
        echo "       That is the exact artifact this script exists to prevent." >&2
        exit 2 ;;
    *)
        echo "ERROR: CPU must be m68k040; got: $CPU" >&2
        exit 2 ;;
esac
CPU_SUBMODULE=cpu040

# ── The one flag that actually matters, plus the rest pinned explicitly ──
BUILD_ENV=(
    CPU=$CPU                 # Explicitly require the real CPU, regardless of Makefile defaults.
    # Full route by default. Override with NO_INCREMENTAL=0 plus
    # INCREMENTAL_REF_DCP=<routed .dcp> for localized iteration (e.g. adding a
    # debug probe): opt_design then read_checkpoint -incremental reuses the
    # reference placement+routing. Keep a COPY of the reference -- the build
    # overwrites checkpoints/route.dcp.
    NO_INCREMENTAL=${NO_INCREMENTAL:-1}
    L2C_ENABLE=1
    VRAM_IN_DDR=1
    REAL_FPGA_BUILD=1
    USE_REAL_MIG=1
    ENABLE_VIO=1             # required: vio-hard-reset recovery path
    ENABLE_JTAG_AXI=1        # required: the whole debug/REPL surface
    PERF_DETAIL_ENABLE=${PERF_DETAIL_ENABLE:-0}
    ENABLE_IPC_ILA=${ENABLE_IPC_ILA:-0}
    TARGET_FREQ_MHZ=100
    VIDEO_SMOKE=0
    BOOT_ROM_SECTORS=2048
    ENABLE_ILA=0             # pinned, not inherited: vivado.tcl parses this
                             # by VALUE, so an exported ENABLE_ILA=1 in the
                             # caller's shell would silently change the build.
)

verify_artifacts() {
    local rc=0
    [ -f "$BIT" ]  || { echo "FAIL: missing $BIT"; rc=1; }
    [ -f "$LTX" ]  || { echo "FAIL: missing $LTX"; rc=1; }
    [ -f "$INFO" ] || { echo "FAIL: missing $INFO"; rc=1; }
    if [ -f "$INFO" ]; then
        grep -qx 'enable_vio=1'      "$INFO" || { echo "FAIL: buildinfo enable_vio != 1"; rc=1; }
        grep -qx 'host_debug=jtag_axi' "$INFO" || { echo "FAIL: buildinfo host_debug != jtag_axi"; rc=1; }
        grep -qx 'l2c_enable=1'      "$INFO" || echo "WARN: l2c_enable != 1"
        grep -qx 'vram_in_ddr=1'     "$INFO" || echo "WARN: vram_in_ddr != 1"
    fi
    # Which CPU is actually in this artifact?  Two independent witnesses:
    # buildinfo's cpu= field (written by vivado.tcl:2256 since 44670e2) and
    # our own stamp file.  Prefer buildinfo; require agreement when both
    # exist, because a mismatch means the stamp survived from an earlier run
    # and is describing a bitstream that is no longer on disk.
    local info_cpu="" stamp_cpu=""
    [ -f "$INFO" ]     && info_cpu=$(sed -n 's/^cpu=//p' "$INFO" | head -1)
    [ -f "$CPUSTAMP" ] && stamp_cpu=$(cat "$CPUSTAMP")

    if [ -n "$info_cpu" ] && [ -n "$stamp_cpu" ] && [ "$info_cpu" != "$stamp_cpu" ]; then
        echo "FAIL: buildinfo says cpu=$info_cpu but $CPUSTAMP says $stamp_cpu."
        echo "      The stamp is stale -- it does not describe $BIT."
        rc=1
    fi

    local eff_cpu="${info_cpu:-$stamp_cpu}"
    if [ -z "$eff_cpu" ]; then
        echo "WARN: neither $INFO (cpu=) nor $CPUSTAMP -- cannot prove which CPU"
        echo "      is in this bitstream.  (Artifacts predating 44670e2 have no"
        echo "      cpu= field; ones predating this script have no stamp.)"
    else
        case "$eff_cpu" in
            m68k040)
                echo "cpu: $eff_cpu${info_cpu:+ (buildinfo)}${stamp_cpu:+ + stamp}" ;;
            *)
                echo "FAIL: artifact reports cpu=$eff_cpu -- that is not a real core."
                rc=1 ;;
        esac
        # If the caller asked for a specific core, hold the artifact to it.
        # Without this, `CPU=m68k ... --verify` silently blesses a cpu040
        # bitstream, which is how the wrong artifact reaches the board.
        if [ -n "$CPU_EXPLICIT" ] && [ -n "$eff_cpu" ] && [ "$eff_cpu" != "$CPU" ]; then
            echo "FAIL: asked for CPU=$CPU but this artifact is cpu=$eff_cpu."
            rc=1
        fi
    fi
    return $rc
}

if [ "$MODE" = verify ]; then
    verify_artifacts; exit $?
fi

# ── Pre-flight ──────────────────────────────────────────────────────────
echo "== pre-flight =="

# One SYNTHESIS/IMPL Vivado at a time, machine-wide.  This host has ~29 GiB
# (NOT the 62 GiB some docs claim) and a KU5P impl peaks around 12-15 GiB plus
# workers, so two concurrent builds OOM it.
#
# 2026-08-12: this check used to reject ANY vivado process, which was wrong and
# cost a build cycle.  The JTAG REPL runs `vivado -mode tcl`; it takes no build
# mutex (make impl uses flock on /var/tmp/m68k-ooo-vivado.lock, which only
# build flows contend for) and costs ~2-3 GiB.  It does NOT conflict with a
# build and must NOT be killed for one -- doing so needlessly drops the board
# connection and any armed breakpoints/watchpoints.  Only a `-mode batch`
# vivado (synth/impl) is a real conflict.
# P133: this guard used to be `pgrep -af 'vivado.*-mode batch'`, which SELF-MATCHES:
# any process whose command line merely *mentions* the pattern trips it, including
# the very ps/grep checks used to look for competing builds.  It falsely reported
# contention for 8 consecutive attempts against a completely idle host.  Probe the
# REAL mutex instead -- `make impl` takes flock on this same file.
if ! flock -n /var/tmp/m68k-ooo-vivado.lock -c true 2>/dev/null; then
    echo "a synthesis/impl vivado holds /var/tmp/m68k-ooo-vivado.lock:"
    ps -eo pid,etime,args | grep -- '-mode batch' | grep -v grep | head -3
    echo "Two concurrent KU5P builds OOM this host. Wait for it to finish."
    exit 1
fi
if pgrep -af 'vivado.*-mode tcl' >/dev/null; then
    echo "note: a JTAG REPL (vivado -mode tcl) is running -- that is fine,"
    echo "      it takes no build mutex and is left alone."
fi
free -g | sed -n '1,2p'

# Check the submodule that will ACTUALLY be compiled for this CPU -- the
# old version always checked cpu/, which says nothing about a cpu040 build.
if [ -n "$(git -C "$CPU_SUBMODULE" status --porcelain 2>/dev/null)" ]; then
    echo "NOTE: $CPU_SUBMODULE/ submodule has uncommitted changes -- they WILL be built."
    git -C "$CPU_SUBMODULE" status --short | head -10
fi
echo "cpu: $CPU (submodule $CPU_SUBMODULE @ $(git -C "$CPU_SUBMODULE" rev-parse --short HEAD 2>/dev/null || echo '?'))"

# cpu040 regenerates M68kSocketTop.v with sbt INSIDE the vivado process
# (synth/vivado.tcl:112-131), while the build mutex is held.  Failing that
# 40 minutes in is expensive; fail now instead.
if [ "$CPU" = m68k040 ] && ! command -v sbt >/dev/null 2>&1; then
    echo "ERROR: CPU=m68k040 needs sbt on PATH to regenerate M68kSocketTop.v" >&2
    echo "       (synth/vivado.tcl runs it inside the build, holding the mutex)." >&2
    exit 2
fi

echo
echo "== building ($MODE) with =="
printf '   %s\n' "${BUILD_ENV[@]}"
echo "   log: $LOG"
echo

start=$(date +%s)
env "${BUILD_ENV[@]}" make "$MODE" > "$LOG" 2>&1
rc=$?
elapsed=$(( $(date +%s) - start ))
echo "make $MODE exit=$rc  (${elapsed}s)"

if [ $rc -ne 0 ]; then
    if [ $rc -eq 75 ]; then
        echo "ERROR: another Vivado run holds the mutex (/var/tmp/m68k-ooo-vivado.lock)."
    fi
    tail -25 "$LOG"
    exit $rc
fi

# Prove the build really selected $CPU rather than trusting our env.
#
# This used to be `grep -q 'CPU=m68k' "$LOG"`, which is a SUBSTRING match:
# it is satisfied by a CPU=m68k040 build line too, so it could not tell the
# two cores apart and would happily stamp "m68k" onto a cpu040 bitstream.
# Parse vivado.tcl:103's own banner instead, anchored and exact.
built_cpu=$(sed -n 's/^=== CPU SOCKET: CPU=\(.*\) ===$/\1/p' "$LOG" | tail -1)
if [ -z "$built_cpu" ]; then
    echo "FAIL: build log has no '=== CPU SOCKET: CPU=... ===' banner -- refusing to stamp."
    rm -f "$CPUSTAMP"
    exit 1
fi
if [ "$built_cpu" != "$CPU" ]; then
    echo "FAIL: asked for CPU=$CPU but the build reports CPU=$built_cpu -- refusing to stamp."
    rm -f "$CPUSTAMP"
    exit 1
fi
if [ "$MODE" = synth ]; then
    # No bitstream was written, so a stamp would describe an artifact this
    # run did not produce.  Leave any existing stamp alone and say so.
    echo "confirmed CPU=$built_cpu; not stamping (synth-only, no bitstream written)"
else
    echo "$built_cpu" > "$CPUSTAMP"
    echo "confirmed CPU=$built_cpu in the build; stamped $CPUSTAMP"
fi

echo
echo "== timing =="
grep -E 'Setup :|Hold  :' "$LOG" | tail -6
# Timing verdict comes from the REPORT, not the log.
#
# 2026-08-12: an earlier version grepped "Setup :" lines out of the build log.
# Those are PER-CLOCK-PAIR summaries and can every one of them read
# "0 Failing Endpoints" while the DESIGN summary still has failing endpoints in
# another path group.  A build with WNS = -0.057 / 54 failing endpoints was
# nearly accepted that way.  Parse report_timing_summary's Design Timing Summary
# row instead: WNS, TNS, and the failing-endpoint count.
RPT=$(ls -t synth/timing_reports/*.rpt 2>/dev/null | head -1)
if [ -n "$RPT" ]; then
    read -r WNS TNS NFAIL _ < <(grep -A8 'Design Timing Summary' "$RPT" \
        | grep -E '^[[:space:]]*-?[0-9]+\.[0-9]+' | head -1 | awk '{print $1, $2, $3}')
    echo "  report: $RPT"
    echo "  WNS=${WNS}ns TNS=${TNS}ns failing_endpoints=${NFAIL}"
    case "$NFAIL" in
        0|"") : ;;
        *) echo "  *** TIMING FAILED: $NFAIL failing endpoints (WNS=$WNS) --"
           echo "      DO NOT program this bitstream; it is not trustworthy. ***" ;;
    esac
else
    echo "  WARNING: no timing report found -- cannot verify timing."
fi

echo
echo "== artifacts =="
ls -la "$BIT" "$LTX" 2>/dev/null
verify_artifacts || exit 1

case "$CPU" in
    m68k040) DBG_VERSION_EXAMPLE=0xDEB60100 ;;   # cpu040, DebugRegMap.scala VERSION_VALUE
esac

# NOTE: this heredoc is deliberately QUOTED (<<'EOF').  The DBG_VERSION line
# below contains backticks around `r 0x40800000`; with an unquoted heredoc
# bash runs that as a command substitution, prints "r: command not found"
# and silently erases the single most useful diagnostic string here.  Emit
# core-specific text with echo instead of unquoting this block.
cat <<'EOF'

== post-build checklist (the artifact is NOT proven good until these pass) ==
  1. Program it:
       First insert your ADB firmware: see docs/adb_firmware_bitstream.md.
       Do NOT load the blank bitstream directly.
       tools/jt.sh 'load-bit build/vivado/fpga_top.local.bit build/vivado/fpga_top.ltx'
     Select that explicit patched path; do not auto-load an older fpga_top.bit.
  2. THE decisive check -- a stub build passes everything else:
       tools/jt.sh 'dbg-caps'      # DBG_VERSION must be NON-ZERO
       tools/jt.sh 'halt-status'   # pc_live must advance; exc_count must be non-zero
     DBG_VERSION = 0x00000000 with working `r 0x40800000` means you built the
     CPU STUB, not a real core.
EOF
echo "     For this build (CPU=$CPU) expect DBG_VERSION = $DBG_VERSION_EXAMPLE;"
echo "     if you get the stub instead, rebuild with CPU=$CPU."
cat <<'EOF'
  3. Boot takes ~2-2.5 min of wall time before the Mac reaches the desktop.
EOF
