#!/usr/bin/env bash
# program_fpga.sh — safe JTAG programmer for the KU5P bringup board.
#
# Default behaviour is DRY-RUN: prints the vivado invocation that would run.
# Pass --go (or --really) to actually program the device.
#
# Refuses to program a bitstream >24h old unless --allow-stale is passed —
# the PM pattern on this project is to re-synth on a fresh baseline before
# every bringup.  Stale bits almost always means "you forgot to rerun
# synth after landing RTL changes."
#
# Delegates VIO programming/snapshotting to synth/vio_dashboard.tcl when a
# matching probe file is present.  It now fails loudly if debug-mode
# programming was requested without that `.ltx`; pass --no-vio only for an
# intentional non-debug image, which then uses synth/program_bitstream.tcl.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BIT="${PROJ_ROOT}/build/vivado/fpga_top.bit"
LTX="${PROJ_ROOT}/build/vivado/fpga_top.ltx"
BUILDINFO=""
GO=0
ALLOW_STALE=0
USE_VIO=1

while [[ $# -gt 0 ]]; do
    case "$1" in
        --go|--really) GO=1; shift ;;
        --allow-stale) ALLOW_STALE=1; shift ;;
        --no-vio)      USE_VIO=0; shift ;;
        --bit)         BIT="$2"; shift 2 ;;
        --ltx)         LTX="$2"; shift 2 ;;
        -h|--help)
            sed -n '2,14p' "$0"
            exit 0 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

if [[ ! -f "$BIT" ]]; then
    echo "ERR: bitstream not found: $BIT" >&2
    exit 1
fi

BUILDINFO="$(dirname "$BIT")/fpga_top.buildinfo"
if [[ ! -f "$BUILDINFO" ]]; then
    BUILDINFO=""
fi

# Freshness guard — reject bits >24h old unless --allow-stale.
AGE_SEC=$(( $(date +%s) - $(stat -c %Y "$BIT") ))
AGE_H=$(( AGE_SEC / 3600 ))
if (( AGE_H > 24 )) && (( ALLOW_STALE == 0 )); then
    echo "ERR: bitstream is ${AGE_H}h old — likely stale." >&2
    echo "     Re-run 'make synth impl' or pass --allow-stale." >&2
    echo "     File: $BIT" >&2
    exit 1
fi

VIVADO="${VIVADO:-/tools/Xilinx/Vivado/2023.1/bin/vivado}"
if [[ ! -x "$VIVADO" ]]; then
    VIVADO="$(command -v vivado || true)"
    [[ -z "$VIVADO" ]] && { echo "ERR: vivado not on PATH" >&2; exit 1; }
fi

manifest_get() {
    local key="$1"
    local file="$2"
    awk -F= -v key="$key" '$1 == key { print $2; found=1; exit } END { if (!found) exit 1 }' "$file"
}

if (( USE_VIO == 1 )); then
    if [[ ! -f "$LTX" ]]; then
        echo "ERR: probe file not found: $LTX" >&2
        echo "     Debug-capable programming now requires a matching .ltx." >&2
        echo "     Rebuild with 'make fpga-100mhz-jtag-bitstream-dram' or pass --no-vio for an intentional non-debug image." >&2
        exit 1
    fi
    if [[ -n "$BUILDINFO" ]]; then
        manifest_vio="$(manifest_get enable_vio "$BUILDINFO" || true)"
        manifest_host="$(manifest_get host_debug "$BUILDINFO" || true)"
        if [[ "$manifest_vio" != "1" ]]; then
            echo "ERR: build manifest says ENABLE_VIO was off: $BUILDINFO" >&2
            echo "     Pass --no-vio for an intentional non-debug image or rebuild with the canonical debug target." >&2
            exit 1
        fi
        if [[ "$manifest_host" != "jtag_axi" && "$manifest_host" != "pcie_xdma" ]]; then
            echo "ERR: build manifest says no supported host debug path was enabled: $BUILDINFO" >&2
            echo "     Rebuild with ENABLE_JTAG_AXI=1 or ENABLE_PCIE_XDMA=1." >&2
            exit 1
        fi
    fi
    TCL="${SCRIPT_DIR}/vio_dashboard.tcl"
    TCLARGS=("$BIT" "$LTX")
    MODE="program + VIO dashboard"
else
    TCL="${SCRIPT_DIR}/program_bitstream.tcl"
    TCLARGS=("$BIT")
    MODE="program only"
fi

echo "bit:        $BIT  (age ${AGE_H}h)"
echo "ltx:        $LTX $([[ -f "$LTX" ]] || echo '(missing — VIO disabled)')"
if [[ -n "$BUILDINFO" ]]; then
    echo "buildinfo:  $BUILDINFO"
fi
echo "tcl:        $TCL"
echo "vivado:     $VIVADO"
echo "mode:       $MODE"
echo "go:         $([[ $GO == 1 ]] && echo 'YES — programming device' || echo 'NO — dry run (pass --go to program)')"

CMD=("$VIVADO" -nojournal -nolog -mode batch -source "$TCL" -tclargs "${TCLARGS[@]}")

if (( GO == 0 )); then
    echo "-- would run --"
    printf '%q ' "${CMD[@]}"; echo
    exit 0
fi

exec "${CMD[@]}"
