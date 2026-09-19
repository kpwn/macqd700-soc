#!/usr/bin/env python3
"""Small terminal dashboard and command wrapper for JTAG bring-up.

This intentionally stays close to the existing Vivado Tcl helpers.  It does
not keep a persistent hw_server session; each status/read/write operation is a
single batch Vivado invocation, which is slower but robust during first-board
debug.
"""

from __future__ import annotations

import argparse
import os
import re
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Iterable

# Ensure the repository root is importable when this file is executed
# directly as `python tools/jtag_bringup_tui.py`.
_THIS_DIR = Path(__file__).resolve().parent
_REPO_PARENT = _THIS_DIR.parent
if str(_REPO_PARENT) not in sys.path:
    sys.path.insert(0, str(_REPO_PARENT))

from tools.m68kctl import regs
from tools.m68kctl.openocd_transport import (
    openocd_cmd,
    parse_kv_output,
    resolve_openocd,
    resolve_openocd_cfgs as _resolve_openocd_cfgs,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
SYNTH_DIR = REPO_ROOT / "synth"
JTAG_TCL = SYNTH_DIR / "jtag_bringup.tcl"
PROGRAM_SH = SYNTH_DIR / "program_fpga.sh"
DEFAULT_BIT = REPO_ROOT / "build" / "vivado" / "fpga_top.bit"
DEFAULT_LTX = REPO_ROOT / "build" / "vivado" / "fpga_top.ltx"

MUTATING_COMMANDS = {"program", "vio-set", "hold", "release", "sd-boot",
                     "axi-write", "debug-halt-after", "debug-break-pc",
                     "debug-halt-exc", "debug-reset-halt",
                     "debug-run-from-reset-halt-after",
                     "debug-sweep-reset-halt-after",
                     "debug-clear-halt", "debug-step", "debug-arch-write",
                     "debug-arch-apply", "video-poke", "vram-wrap-poke",
                     "rom-load", "debug-full-reset"}

ALIASES = {
    "hdmi_mmcm_locked": "probe_in0",
    "hdmi_mmcm_locked_1": "probe_in0",
    "video_debug_hcount": "probe_in1",
    "video_debug_vcount": "probe_in2",
    "vram_rd_addr": "probe_in3",
    "video_debug_rgb": "probe_in4",
    "dbg_pc": "probe_in5",
    "ddr_dbg_r_cnt": "probe_in6",
    "vio_rst_bundle": "probe_in7",
    "vio_rst_bundle_1": "probe_in7",
    "rst/init bundle": "probe_in7",
    "s0_wready": "probe_in8",
    "s0_wready_1": "probe_in8",
    "dbg_committed": "probe_in9",
    "vio_hdmi_ctrl": "probe_in10",
    "vio_hdmi_ctrl_1": "probe_in10",
    "hdmi_ctrl": "probe_in10",
    "vio_vram_read": "probe_in11",
    "vram_read": "probe_in11",
    "vio_ddr_axi": "probe_in12",
    "ddr_axi": "probe_in12",
    # VIO probe trim (2026-04-26 debug-bloat-cull): the
    # vio_write_counts/vio_vram_write/vio_fb_reader_stats probes (old
    # probe_in13/14/17) were dropped to free LUTs.  The remaining set
    # was renumbered: old probe_in15 boot_video → new probe_in13,
    # old probe_in16 dafb_cfg → new probe_in14, old probe_in18 axi_error
    # → new probe_in15.  Keep the legacy names mapped to the new probe
    # indices so any host helper that still asks for `boot_video` or
    # `axi_error_status` keeps working.
    "vio_boot_video": "probe_in13",
    "boot_video": "probe_in13",
    "vio_dafb_cfg": "probe_in14",
    "dafb_cfg": "probe_in14",
    "vio_axi_error": "probe_in15",
    "axi_error_status": "probe_in15",
    "err_aw_addr": "probe_in16",
    "err_ar_addr": "probe_in17",
    "vio_boot_ctrl": "probe_out0",
}

RST_BITS = [
    ("cpu_resetn", 5),
    ("core_rst", 4),
    ("ddr_cal_done", 3),
    ("boot_rom_ready", 2),
    ("hdmi_i2c_done", 1),
    ("fb_underflow_sticky", 0),
]

DDR_AXI_BITS = [
    ("s0_awvalid", 9),
    ("s0_awready", 8),
    ("s0_wvalid", 7),
    ("s0_wready", 6),
    ("s0_bvalid", 5),
    ("s0_bready", 4),
    ("s0_arvalid", 3),
    ("s0_arready", 2),
    ("s0_rvalid", 1),
    ("s0_rready", 0),
]

BOOT_VIDEO_BITS = [
    ("boot_rom_loading", 7),
    ("boot_error", 6),
    ("hdmi_mmcm_locked", 5),
    ("hdmi_i2c_done", 4),
    ("video_debug_de", 3),
    ("vram_rd_en", 2),
    ("vram_rd_valid", 1),
    ("al9134_int", 0),
]

OPENOCD_SUPPORTED_COMMANDS = {
    "status",
    "dashboard",
    "axi-read",
    "axi-write",
    "debug-halt-status",
    "debug-reset-halt",
    "debug-halt-after",
    "debug-break-pc",
    "debug-halt-exc",
    "debug-clear-halt",
    "debug-step",
}

SCANOUT_STATUS_BITS = [
    ("jtag_debug_full_reset", 0),
    ("scc_uart_sel_b", 1),
    ("jtag_boot_release", 2),
    ("jtag_boot_bypass", 3),
    ("cpu_rst", 4),
    ("boot_fsm_rst", 5),
    ("boot_error", 6),
    ("boot_rom_loading", 7),
    ("video_debug_vs", 8),
    ("video_debug_hs", 9),
    ("video_debug_de", 10),
    ("al9134_resetn", 11),
    ("hdmi_i2c_done", 12),
    ("hdmi_mmcm_locked", 13),
    ("fb_underflow_sticky", 14),
    ("hdmi_test_pattern", 15),
]


def parse_u32(text: str) -> int:
    value = int(text, 0)
    if value < 0 or value > 0xFFFFFFFF:
        raise argparse.ArgumentTypeError(f"out of u32 range: {text}")
    return value


def parse_u64(text: str) -> int:
    value = int(text, 0)
    if value < 0 or value > 0xFFFFFFFFFFFFFFFF:
        raise argparse.ArgumentTypeError(f"out of u64 range: {text}")
    return value


def hex_u32(value: int) -> str:
    return f"0x{value & 0xFFFFFFFF:08X}"


def _parse_probe_value(text: str) -> int | None:
    text = text.strip()
    if text == "<missing>":
        return None
    if text.lower().startswith("0x"):
        text = text[2:]
    if any(ch in text.lower() for ch in "xz"):
        return None
    return int(text, 16)


def parse_snapshot(text: str) -> dict[str, int | None]:
    """Parse either snapshot-machine output or the older human dashboard."""

    probes: dict[str, int | None] = {}
    machine_re = re.compile(r"^([A-Za-z0-9_./\[\]-]+)=(0x[0-9a-fA-FxzXZ]+|<missing>)$")
    human_probe_re = re.compile(r"^probe_in\s*(\d+)\s+0x([0-9a-fA-FxzXZ]+)")
    human_out_re = re.compile(r"probe_out0(?: jtag_boot_ctl :|=) 0x([0-9a-fA-FxzXZ]+)")
    compact_out_re = re.compile(r"^probe_out0=0x([0-9a-fA-FxzXZ]+)")
    named_human_re = re.compile(r"^([A-Za-z0-9_./ -]+?)\s*:\s+0x([0-9a-fA-FxzXZ]+)")

    def store(key: str, value_text: str) -> None:
        value = _parse_probe_value(value_text)
        key = key.strip()
        probes[key] = value
        canonical = ALIASES.get(key)
        if canonical and probes.get(canonical) is None:
            probes[canonical] = value

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        m = machine_re.match(line)
        if m:
            store(m.group(1), m.group(2))
            continue
        m = human_probe_re.match(line)
        if m:
            store(f"probe_in{int(m.group(1))}", m.group(2))
            continue
        m = compact_out_re.match(line) or human_out_re.search(line)
        if m:
            store("probe_out0", m.group(1))
            continue
        m = named_human_re.match(line)
        if m:
            store(m.group(1), m.group(2))

    synthesize_bundles(probes)
    return probes


def synthesize_bundles(probes: dict[str, int | None]) -> None:
    def bundle_missing(name: str) -> bool:
        return name not in probes or probes[name] is None

    def put_bundle(name: str, fields: list[tuple[str, int]]) -> None:
        if not bundle_missing(name):
            return
        if not all(field in probes and probes[field] is not None for field, _ in fields):
            return
        value = 0
        for field, index in fields:
            value |= (int(probes[field] or 0) & 1) << index
        probes[name] = value

    put_bundle("probe_in7", RST_BITS)
    put_bundle("probe_in12", DDR_AXI_BITS)
    if bundle_missing("probe_in14") and all(
        k in probes and probes[k] is not None
        for k in ("dafb_fb_base_px", "dafb_fb_stride_px", "dafb_fb_bpp_reg")
    ):
        probes["probe_in14"] = (
            ((int(probes["dafb_fb_base_px"] or 0) & 0xFFFFFFFF) << 64) |
            ((int(probes["dafb_fb_stride_px"] or 0) & 0xFFFFFFFF) << 32) |
            (int(probes["dafb_fb_bpp_reg"] or 0) & 0xFFFFFFFF)
        )
    if bundle_missing("probe_in15") and all(
        k in probes and probes[k] is not None
        for k in ("axi_error_resp", "axi_error_src",
                  "axi_error_sticky", "axi_error_seen")
    ):
        probes["probe_in15"] = (
            ((int(probes["axi_error_seen"] or 0) & 1) << 7) |
            ((int(probes["axi_error_sticky"] or 0) & 1) << 6) |
            ((int(probes["axi_error_src"] or 0) & 0xF) << 2) |
            (int(probes["axi_error_resp"] or 0) & 0x3)
        )

    # The fb_reader_*_count counters used to fold up into probe_in17;
    # since that probe was dropped (debug-bloat-cull, 2026-04-26) we
    # rebuild the per-counter values from `name[i]` bit-decomposed
    # samples when the host produced them via JTAG-AXI / labtools.
    for prefix in ("fb_reader_req_count", "fb_reader_rsp_count",
                   "fb_reader_miss_count"):
        if prefix not in probes:
            value = 0
            complete = True
            for i in range(16):
                bit_name = f"{prefix}[{i}]"
                if bit_name not in probes or probes[bit_name] is None:
                    complete = False
                    break
                value |= (int(probes[bit_name] or 0) & 1) << i
            if complete:
                probes[prefix] = value

    if "probe_in11" in probes and probes["probe_in11"] is not None:
        vram = int(probes["probe_in11"] or 0)
        probes.setdefault("vram_rd_en", vram & 1)
        probes.setdefault("vram_rd_valid", (vram >> 1) & 1)

    put_bundle("probe_in13", BOOT_VIDEO_BITS)

    if "probe_in0" in probes and probes["probe_in0"] is not None:
        probes.setdefault("hdmi_mmcm_locked", int(probes["probe_in0"] or 0) & 1)

    if "probe_in10" in probes and probes["probe_in10"] is not None:
        hdmi = int(probes["probe_in10"] or 0)
        probes.setdefault("video_debug_hs", hdmi & 1)
        probes.setdefault("video_debug_vs", (hdmi >> 1) & 1)
        probes.setdefault("video_debug_de", (hdmi >> 2) & 1)
        probes.setdefault("hdmi_i2c_done", (hdmi >> 3) & 1)
        probes.setdefault("al9134_resetn", (hdmi >> 4) & 1)

    if "probe_in7" in probes and probes["probe_in7"] is not None:
        rst = int(probes["probe_in7"] or 0)
        for field, index in RST_BITS:
            probes.setdefault(field, (rst >> index) & 1)

    if "probe_in13" in probes and probes["probe_in13"] is not None:
        boot_video = int(probes["probe_in13"] or 0)
        for field, index in BOOT_VIDEO_BITS:
            probes.setdefault(field, (boot_video >> index) & 1)

    if "probe_in15" in probes and probes["probe_in15"] is not None:
        err = int(probes["probe_in15"] or 0)
        probes.setdefault("axi_error_resp", err & 0x3)
        probes.setdefault("axi_error_src", (err >> 2) & 0xF)
        probes.setdefault("axi_error_sticky", (err >> 6) & 0x1)
        probes.setdefault("axi_error_seen", (err >> 7) & 0x1)

    if "probe_out0" in probes and probes["probe_out0"] is not None:
        out = int(probes["probe_out0"] or 0)
        probes.setdefault("jtag_boot_bypass", out & 1)
        probes.setdefault("jtag_boot_release", (out >> 1) & 1)
        # vio_boot_ctrl[2] is `scc_uart_sel_b` in the current RTL
        # (rtl/fpga_top_clocks.vh:311).  Old name `jtag_cpu_hold` was
        # subsumed by the umbrella full-reset (bit 3) two refactors ago;
        # mapped here as `scc_uart_sel_b` to match the RTL contract.
        probes.setdefault("scc_uart_sel_b", (out >> 2) & 1)
        probes.setdefault("jtag_debug_full_reset", (out >> 3) & 1)

    put_bundle("probe_in13", BOOT_VIDEO_BITS)

    if bundle_missing("probe_in11") and all(k in probes and probes[k] is not None
                                            for k in ("vram_rd_valid", "vram_rd_en")):
        probes["probe_in11"] = ((int(probes["vram_rd_valid"] or 0) & 1) << 1) | (
            int(probes["vram_rd_en"] or 0) & 1)


def bit(value: int | None, index: int) -> str:
    if value is None:
        return "?"
    return str((value >> index) & 1)


def value_or_unknown(probes: dict[str, int | None], name: str) -> int | None:
    return probes.get(name)


def extract_labtools_warnings(text: str) -> list[str]:
    return [line.strip() for line in text.splitlines()
            if "Labtools 27-3410" in line or "Calibration Failed" in line]


def flag(value: int | None, index: int) -> bool:
    return value is not None and ((value >> index) & 1) == 1


def format_findings(probes: dict[str, int | None], warnings: list[str]) -> list[str]:
    rst = probes.get("probe_in7")
    boot_video = probes.get("probe_in13")
    hdmi_alive = (
        probes.get("probe_in0") == 1 or flag(boot_video, 5)
    ) and (flag(rst, 1) or flag(boot_video, 4))
    ddr_cal_done = flag(rst, 3)
    boot_rom_ready = flag(rst, 2)
    core_rst = flag(rst, 4)

    findings: list[str] = []
    if warnings:
        findings.append("Vivado reported: " + " | ".join(warnings))
    if hdmi_alive and not ddr_cal_done and not boot_rom_ready:
        detail = "HDMI is alive, but DDR/MIG calibration is not done"
        if core_rst:
            detail += "; reset/init is still blocking CPU release"
        findings.append(detail + ".")
    elif hdmi_alive:
        findings.append("HDMI is alive.")
    if rst is not None and not ddr_cal_done:
        findings.append("ddr_cal_done=0; JTAG AXI DDR accesses and ROM boot may stall/fail.")
    return findings


def format_snapshot(probes: dict[str, int | None], *, source: str = "Vivado",
                    warnings: list[str] | None = None) -> str:
    p = lambda name: value_or_unknown(probes, name)
    out = p("probe_out0")
    rst = p("probe_in7")
    hdmi = p("probe_in10")
    vram_read = p("probe_in11")
    ddr_axi = p("probe_in12")
    boot_video = p("probe_in13")
    dafb_cfg = p("probe_in14")
    axi_err = p("probe_in15")
    # The vio_write_counts (vram/dafb counters), vio_vram_write (smoke
    # handshakes) and vio_fb_reader_stats VIO probes were dropped in
    # the 2026-04-26 debug-bloat-cull to relieve LUT pressure.  Their
    # data is reachable through the JTAG-AXI debug_ctrl block; here we
    # display "?" if the host didn't read it that way.
    writes = None
    vram_write = None
    fb_stats = None

    def fmt(name: str, digits: int | None = 8) -> str:
        value = p(name)
        if value is None:
            return "?"
        if digits is None:
            return f"0x{value:X}"
        return f"0x{value:0{digits}X}"

    def fmt_u32(value: int | None) -> str:
        if value is None:
            return "?"
        return f"0x{value & 0xFFFFFFFF:08X}"

    def bit_or_probe(bundle: int | None, index: int, probe_name: str | None = None) -> str:
        if bundle is not None:
            return bit(bundle, index)
        if probe_name is not None and p(probe_name) in (0, 1):
            return str(p(probe_name))
        return "?"

    def flag_bundle(bundle: int | None, index: int) -> str:
        return bit(bundle, index)

    # writes/vram_write probes were dropped; the host can still publish
    # vram_write_count/dafb_write_count via JTAG-AXI debug_ctrl reads,
    # so keep that fallback alive for dashboards that still ask.
    vram_w_count = (writes >> 16) & 0xFFFF if writes is not None else p("vram_write_count")
    dafb_w_count = writes & 0xFFFF if writes is not None else p("dafb_write_count")
    fb_miss_count = (fb_stats >> 48) & 0xFFFF if fb_stats is not None else p("fb_reader_miss_count")
    fb_rsp_count = (fb_stats >> 32) & 0xFFFF if fb_stats is not None else p("fb_reader_rsp_count")
    fb_req_count = (fb_stats >> 16) & 0xFFFF if fb_stats is not None else p("fb_reader_req_count")
    fb_flags = fb_stats & 0xFFFF if fb_stats is not None else None
    dafb_base = (dafb_cfg >> 64) & 0xFFFFFFFF if dafb_cfg is not None else None
    dafb_stride = (dafb_cfg >> 32) & 0xFFFFFFFF if dafb_cfg is not None else None
    dafb_bpp = dafb_cfg & 0xFFFFFFFF if dafb_cfg is not None else None
    axi_err_resp = axi_err & 0x3 if axi_err is not None else None
    axi_err_src = (axi_err >> 2) & 0xF if axi_err is not None else None
    axi_err_sticky = flag(axi_err, 6)
    axi_err_seen = flag(axi_err, 7)
    axi_err_src_name = {
        0: "none",
        1: "ddr",
        2: "io",
        3: "dma",
        4: "vram",
        5: "boot",
    }.get(axi_err_src, "?")
    axi_err_resp_name = {
        0: "OKAY",
        1: "EXOKAY",
        2: "SLVERR",
        3: "DECERR",
    }.get(axi_err_resp, "?")
    scanout_mode = (
        "test-pattern" if fb_flags is not None and flag(fb_flags, 15)
        else "test-pattern" if p("hdmi_test_pattern") == 1
        else "framebuffer" if p("hdmi_test_pattern") == 0
        else "framebuffer" if fb_flags is not None
        else "?"
    )

    lines = [
        "m68k-ooo JTAG bring-up dashboard",
        f"source: {source}",
    ]

    findings = format_findings(probes, warnings or [])
    if findings:
        lines += ["", "Findings"]
        lines += [f"  {finding}" for finding in findings]

    lines += [
        "",
        "Reset / boot",
        f"  cpu_resetn={bit(rst, 5)} core_rst={bit(rst, 4)} "
        f"ddr_cal_done={bit(rst, 3)} boot_rom_ready={bit(rst, 2)} "
        f"hdmi_i2c_done={bit(rst, 1)} fb_underflow={bit(rst, 0)}",
        f"  boot_video: loading={bit_or_probe(boot_video, 7, 'boot_rom_loading')} "
        f"error={bit_or_probe(boot_video, 6, 'boot_error')} "
        f"mmcm={bit_or_probe(boot_video, 5, 'hdmi_mmcm_locked')} "
        f"i2c={bit_or_probe(boot_video, 4, 'hdmi_i2c_done')} "
        f"de={bit_or_probe(boot_video, 3, 'video_debug_de')} "
        f"rd_en={bit_or_probe(boot_video, 2, 'vram_rd_en')} "
        f"rd_valid={bit_or_probe(boot_video, 1, 'vram_rd_valid')} "
        f"al9134_int={bit_or_probe(boot_video, 0, 'al9134_int')}",
        f"  jtag_ctl: bypass_sd={bit(out, 0)} release_cpu={bit(out, 1)} "
        f"scc_uart_sel_b={bit(out, 2)} full_dbg_rst={bit(out, 3)} "
        f"raw={fmt('probe_out0', 1)}",
        f"  reset_state: boot_fsm_rst={bit_or_probe(fb_flags, 5, 'boot_fsm_rst')} "
        f"cpu_rst={bit_or_probe(fb_flags, 4, 'cpu_rst')} "
        f"boot_bypass={bit(out, 0)} "
        f"jtag_release={bit(out, 1)} "
        f"scc_uart_sel_b={bit(out, 2)} "
        f"jtag_full_dbg_rst={bit(out, 3)}",
        "",
        "Core / DDR",
        f"  pc={fmt('probe_in5')} committed={fmt('probe_in9')} "
        f"ddr_reads_low16={fmt('probe_in6', 4)} s0_wready={fmt('probe_in8', 1)}",
        f"  ddr_axi: awv={bit(ddr_axi, 9)} awr={bit(ddr_axi, 8)} "
        f"wv={bit(ddr_axi, 7)} wr={bit(ddr_axi, 6)} "
        f"bv={bit(ddr_axi, 5)} br={bit(ddr_axi, 4)} "
        f"arv={bit(ddr_axi, 3)} arr={bit(ddr_axi, 2)} "
        f"rv={bit(ddr_axi, 1)} rr={bit(ddr_axi, 0)} raw={fmt('probe_in12', 3)}",
        "",
        "Video / VRAM",
        f"  hdmi: mmcm_locked={fmt('probe_in0', 1)} mode={scanout_mode} "
        f"resetn={bit(hdmi, 4)} i2c_done={bit(hdmi, 3)} "
        f"de={bit(hdmi, 2)} vs={bit(hdmi, 1)} hs={bit(hdmi, 0)} "
        f"rgb={fmt('probe_in4', 6)}",
        f"  vtg: h={fmt('probe_in1', 3)} v={fmt('probe_in2', 3)} "
        f"vram_rd_addr={fmt('probe_in3', 5)} "
        f"rd_valid={bit(vram_read, 1)} rd_en={bit(vram_read, 0)}",
        f"  fb_reader: req={fb_req_count if fb_req_count is not None else '?'} "
        f"rsp={fb_rsp_count if fb_rsp_count is not None else '?'} "
        f"stall={fb_miss_count if fb_miss_count is not None else '?'} "
        f"underflow={bit_or_probe(fb_flags, 14, 'fb_underflow_sticky')}",
        f"  dafb: base={fmt_u32(dafb_base)} stride={fmt_u32(dafb_stride)} "
        f"bpp={fmt_u32(dafb_bpp)} raw={fmt('probe_in14', 24)}",
        f"  writes: vram={vram_w_count if vram_w_count is not None else '?'} "
        f"dafb={dafb_w_count if dafb_w_count is not None else '?'} "
        f"  (smoke + vram-write probes dropped 2026-04-26 to free LUTs)",
        "",
        "Bus / errors",
        f"  axi_error: seen={bit(axi_err, 7)} "
        f"sticky={bit(axi_err, 6)} "
        f"src={axi_err_src_name} resp={axi_err_resp_name} raw={fmt('probe_in15', 4)}",
    ]

    def effectively_present(name: str) -> bool:
        if name in probes and probes[name] is not None:
            return True
        return False

    missing = sorted(name for name in [f"probe_in{i}" for i in range(16)] + ["probe_out0"]
                     if not effectively_present(name))
    if missing:
        lines += ["", "Missing/unknown probes: " + ", ".join(missing)]
    return "\n".join(lines)


def resolve_vivado(cli_value: str | None) -> str:
    if cli_value:
        return cli_value
    if os.environ.get("VIVADO"):
        return os.environ["VIVADO"]
    default = Path("/tools/Vivado/2025.2/Vivado/bin/vivado")
    if default.exists():
        return str(default)
    found = shutil.which("vivado")
    return found or "vivado"


def _require_openocd_cfgs(args: argparse.Namespace) -> list[str]:
    cfgs = resolve_openocd_cfgs_from_args(args)
    if cfgs:
        return cfgs
    raise RuntimeError(
        "OpenOCD backend needs at least one --openocd-cfg or an "
        "OPENOCD_CFG environment variable"
    )


def resolve_openocd_cfgs_from_args(args: argparse.Namespace) -> list[str]:
    return _resolve_openocd_cfgs(args.openocd_cfg)


def command_text(cmd: Iterable[str]) -> str:
    return shlex.join([str(part) for part in cmd])


def jtag_env(args: argparse.Namespace) -> dict[str, str]:
    env = os.environ.copy()
    if args.bit:
        env["BIT_FILE"] = str(args.bit)
    if args.ltx:
        env["LTX_FILE"] = str(args.ltx)
    if args.target_re:
        env["HW_TARGET_RE"] = args.target_re
    if args.device_re:
        env["HW_DEVICE_RE"] = args.device_re
    return env


def run(cmd: list[str], *, env: dict[str, str] | None = None,
        capture: bool = False, dry_run: bool = False) -> subprocess.CompletedProcess[str]:
    if dry_run:
        print(command_text(cmd))
        return subprocess.CompletedProcess(cmd, 0, "", "")
    return subprocess.run(
        cmd,
        cwd=REPO_ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.STDOUT if capture else None,
        check=False,
    )


def vivado_cmd(args: argparse.Namespace, *tclargs: str) -> list[str]:
    return [
        resolve_vivado(args.vivado),
        "-nojournal",
        "-nolog",
        "-mode",
        "batch",
        "-source",
        str(JTAG_TCL),
        "-tclargs",
        *tclargs,
    ]


def run_jtag(args: argparse.Namespace, *tclargs: str, capture: bool = False,
             mutate: bool = False) -> subprocess.CompletedProcess[str]:
    dry = args.dry_run or (mutate and not args.go)
    if mutate and dry:
        print("dry-run: pass --go to perform this JTAG write/control operation")
    return run(vivado_cmd(args, *tclargs), env=jtag_env(args), capture=capture, dry_run=dry)


def run_openocd(args: argparse.Namespace, *script_lines: str, capture: bool = False,
                mutate: bool = False) -> subprocess.CompletedProcess[str]:
    cfgs = _require_openocd_cfgs(args)
    if not cfgs:
        raise RuntimeError(
            "OpenOCD backend needs at least one --openocd-cfg or an "
            "OPENOCD_CFG environment variable"
        )
    dry = args.dry_run or (mutate and not args.go)
    if mutate and dry:
        print("dry-run: pass --go to perform this OpenOCD write/control operation")
    cmd = openocd_cmd(
        resolve_openocd(args.openocd_bin),
        cfgs,
        *script_lines,
        target=args.openocd_target,
    )
    return run(cmd, capture=capture, dry_run=dry)


def _openocd_read_expr(addr: int) -> str:
    return f"[lindex [read_memory {hex_u32(addr)} 32 1 phys] 0]"


def _openocd_status_lines() -> list[str]:
    pairs = [
        ("DBG_VERSION", regs.OFF_DBG_VERSION),
        ("DBG_BUILD_ID", regs.OFF_DBG_BUILD_ID),
        ("DBG_CONTROL", regs.OFF_DBG_CONTROL),
        ("DBG_STATUS", regs.OFF_DBG_STATUS),
        ("DBG_PC", regs.OFF_DBG_PC),
        ("DBG_LAST_PC", regs.OFF_DBG_LAST_PC),
        ("DBG_CYCLE_LO", regs.OFF_DBG_CYCLE_LO),
        ("DBG_CYCLE_HI", regs.OFF_DBG_CYCLE_HI),
        ("DBG_INST_LO", regs.OFF_DBG_INST_LO),
        ("DBG_INST_HI", regs.OFF_DBG_INST_HI),
        ("DBG_MISPRED_COUNT", regs.OFF_DBG_MISPRED_COUNT),
        ("DBG_FLUSH_COUNT", regs.OFF_DBG_FLUSH_COUNT),
        ("DBG_EXC_COUNT", regs.OFF_DBG_EXC_COUNT),
        ("DBG_HALT_CTL", regs.OFF_DBG_HALT_CTL),
        ("DBG_HALT_REASON", regs.OFF_DBG_HALT_REASON),
        ("DBG_HALT_HIT_PC", regs.OFF_DBG_HALT_HIT_PC),
        ("DBG_HALT_HIT_INST_LO", regs.OFF_DBG_HALT_HIT_INST_LO),
        ("DBG_HALT_HIT_INST_HI", regs.OFF_DBG_HALT_HIT_INST_HI),
        ("DBG_HALT_EXC_VEC", regs.OFF_DBG_HALT_EXC_VEC),
    ]
    lines = []
    for name, addr in pairs:
        lines.append(f'puts [format "{name}=0x%08X" {_openocd_read_expr(addr)}]')
    return lines


def _format_openocd_status(values: dict[str, int], source: str) -> str:
    def value(name: str) -> int:
        return values.get(name, 0)

    cycles = ((value("DBG_CYCLE_HI") << 32) | value("DBG_CYCLE_LO"))
    insts = ((value("DBG_INST_HI") << 32) | value("DBG_INST_LO"))
    status = value("DBG_STATUS")
    halt_ctl = value("DBG_HALT_CTL")
    out = [
        f"OpenOCD debug snapshot ({source})",
        f'DBG_VERSION  : 0x{value("DBG_VERSION"):08x}',
        f'DBG_BUILD_ID : 0x{value("DBG_BUILD_ID"):08x}',
        f'DBG_CONTROL  : 0x{value("DBG_CONTROL"):08x}',
        f'DBG_STATUS   : 0x{status:08x}  '
        f'(halted={bool(status & regs.STS_HALTED)} '
        f'running={bool(status & regs.STS_CPU_RUNNING)} '
        f'init_done={bool(status & regs.STS_INIT_DONE_SEEN)})',
        f'DBG_PC       : 0x{value("DBG_PC"):08x}',
        f'DBG_LAST_PC  : 0x{value("DBG_LAST_PC"):08x}',
        f'DBG_CYCLES   : {cycles}',
        f'DBG_INSTS    : {insts}',
        f'IPC          : {(insts / cycles) if cycles else 0.0:.3f}',
        f'DBG_MISPRED  : {value("DBG_MISPRED_COUNT")}',
        f'DBG_FLUSHES  : {value("DBG_FLUSH_COUNT")}',
        f'DBG_EXCEPTS  : {value("DBG_EXC_COUNT")}',
        f'DBG_HALT_CTL : 0x{halt_ctl:08x}  '
        f'(halt_after_en={bool(halt_ctl & regs.HALT_AFTER_ENABLE)} '
        f'break_pc_en={bool(halt_ctl & regs.HALT_BREAK_PC_ENABLE)} '
        f'halt_exc_en={bool(halt_ctl & regs.HALT_EXC_ENABLE)} '
        f'auto_latched={bool(halt_ctl & regs.HALT_AUTO_LATCHED)})',
        f'DBG_HALT_REASON  : 0x{value("DBG_HALT_REASON"):08x}',
        f'DBG_HALT_HIT_PC  : 0x{value("DBG_HALT_HIT_PC"):08x}',
        f'DBG_HALT_HIT_INST: 0x{((value("DBG_HALT_HIT_INST_HI") << 32) | value("DBG_HALT_HIT_INST_LO")):016x}',
        f'DBG_HALT_EXC_VEC : {value("DBG_HALT_EXC_VEC") & 0xFF}',
    ]
    return "\n".join(out)


def do_status(args: argparse.Namespace) -> int:
    if args.mock:
        text = Path(args.mock).read_text()
        print(format_snapshot(parse_snapshot(text), source=str(args.mock),
                              warnings=extract_labtools_warnings(text)))
        return 0
    if args.backend == "openocd":
        result = run_openocd(args, *_openocd_status_lines(), capture=True)
        if result.returncode != 0:
            print(result.stdout or "", end="")
            return result.returncode
        values = parse_kv_output(result.stdout or "")
        print(_format_openocd_status(values, source="OpenOCD"))
        return 0

    result = run_jtag(args, "snapshot-machine", capture=True)
    if result.returncode != 0:
        print(result.stdout or "", end="")
        return result.returncode
    print(format_snapshot(parse_snapshot(result.stdout), source="Vivado hw_server/JTAG",
                          warnings=extract_labtools_warnings(result.stdout)))
    return 0


def do_dashboard(args: argparse.Namespace) -> int:
    interval = args.interval
    remaining = 1 if args.once else args.count
    while True:
        if args.mock:
            text = Path(args.mock).read_text()
            rc = 0
        elif args.backend == "openocd":
            result = run_openocd(args, *_openocd_status_lines(), capture=True)
            text = result.stdout or ""
            rc = result.returncode
        else:
            result = run_jtag(args, "snapshot-machine", capture=True)
            text = result.stdout or ""
            rc = result.returncode

        print("\033[2J\033[H", end="")
        print(time.strftime("%Y-%m-%d %H:%M:%S %Z"))
        if rc == 0:
            if args.backend == "openocd":
                print(_format_openocd_status(parse_kv_output(text), source="OpenOCD"))
            else:
                print(format_snapshot(parse_snapshot(text), source="Vivado hw_server/JTAG",
                                      warnings=extract_labtools_warnings(text)))
        else:
            print(text, end="")
            return rc
        print("")
        print("commands: use subcommands hold/release/sd-boot/vio-set/axi-read/axi-write/debug-halt-after/debug-break-pc/debug-halt-exc/debug-reset-halt/debug-run-from-reset-halt-after/debug-sweep-reset-halt-after/debug-clear-halt/debug-step/debug-arch-write/debug-arch-apply/rom-load/debug-full-reset")
        print("safety: mutating subcommands dry-run unless --go is supplied")

        if remaining == 1:
            return 0
        if remaining > 1:
            remaining -= 1
        time.sleep(interval)


def do_program(args: argparse.Namespace) -> int:
    if args.backend == "openocd":
        print("error: program is Vivado-only; use the default backend",
              file=sys.stderr)
        return 2
    cmd = [str(PROGRAM_SH), "--bit", str(args.bit or DEFAULT_BIT), "--ltx", str(args.ltx or DEFAULT_LTX)]
    if args.go:
        cmd.append("--go")
    if args.allow_stale:
        cmd.append("--allow-stale")
    if args.no_vio:
        cmd.append("--no-vio")
    env = os.environ.copy()
    if args.vivado:
        env["VIVADO"] = args.vivado
    return run(cmd, env=env, dry_run=args.dry_run).returncode


def do_passthrough(args: argparse.Namespace) -> int:
    name = args.command
    if args.backend == "openocd":
        if name not in OPENOCD_SUPPORTED_COMMANDS:
            print(f"error: {name} is not supported by the OpenOCD backend",
                  file=sys.stderr)
            return 2
        return _do_passthrough_openocd(args)
    if name == "vio-get":
        return run_jtag(args, "vio-get").returncode
    if name == "vio-set":
        return run_jtag(args, "vio-set", hex(args.value), mutate=True).returncode
    if name == "hold":
        return run_jtag(args, "vio-set", "0x5", mutate=True).returncode
    if name == "release":
        return run_jtag(args, "vio-set", "0x3", mutate=True).returncode
    if name == "sd-boot":
        return run_jtag(args, "vio-set", "0x0", mutate=True).returncode
    if name == "debug-full-reset":
        tclargs = ["debug-full-reset", str(args.hold_ms),
                   "1" if args.release else "0"]
        return run_jtag(args, *tclargs, mutate=True).returncode
    if name == "axi-read":
        return run_jtag(args, "axi-read", hex_u32(args.addr)).returncode
    if name == "axi-write":
        return run_jtag(args, "axi-write", hex_u32(args.addr), hex_u32(args.data),
                        mutate=True).returncode
    if name == "debug-halt-status":
        return run_jtag(args, "debug-halt-status").returncode
    if name == "debug-halt-after":
        tclargs = ["debug-halt-after", str(args.inst_count)]
        if args.halt_ctl is not None:
            tclargs.append(hex_u32(args.halt_ctl))
        return run_jtag(args, *tclargs, mutate=True).returncode
    if name == "debug-break-pc":
        tclargs = ["debug-break-pc", hex_u32(args.pc)]
        if args.halt_ctl is not None:
            tclargs.append(hex_u32(args.halt_ctl))
        return run_jtag(args, *tclargs, mutate=True).returncode
    if name == "debug-halt-exc":
        tclargs = ["debug-halt-exc", str(args.vec)]
        if args.halt_ctl is not None:
            tclargs.append(hex_u32(args.halt_ctl))
        return run_jtag(args, *tclargs, mutate=True).returncode
    if name == "debug-reset-halt":
        return run_jtag(args, "debug-reset-halt", mutate=True).returncode
    if name == "debug-run-from-reset-halt-after":
        return run_jtag(args, "debug-run-from-reset-halt-after",
                        str(args.inst_count), str(args.wait_ms),
                        mutate=True).returncode
    if name == "debug-sweep-reset-halt-after":
        tclargs = ["debug-sweep-reset-halt-after", str(args.wait_ms)]
        tclargs.extend(str(count) for count in args.inst_counts)
        return run_jtag(args, *tclargs, mutate=True).returncode
    if name == "debug-clear-halt":
        tclargs = ["debug-clear-halt"]
        if args.enable_bits is not None:
            tclargs.append(hex_u32(args.enable_bits))
        return run_jtag(args, *tclargs, mutate=True).returncode
    if name == "debug-step":
        tclargs = ["debug-step"]
        if args.halt_first:
            tclargs.append("--halt-first")
        return run_jtag(args, *tclargs, mutate=True).returncode
    if name == "debug-arch-write":
        return run_jtag(args, "debug-arch-write", args.reg.upper(), hex_u32(args.value),
                        mutate=True).returncode
    if name == "debug-arch-apply":
        return run_jtag(args, "debug-arch-apply", mutate=True).returncode
    if name == "video-poke":
        return run_jtag(args, "video-poke", hex_u32(args.fb_base), hex_u32(args.stride),
                        str(args.rows), str(args.cols_words),
                        "1" if args.hold_cpu else "0", mutate=True).returncode
    if name == "vram-wrap-poke":
        return run_jtag(args, "vram-wrap-poke", hex_u32(args.fb_base),
                        hex_u32(args.stride), str(args.rows),
                        str(args.cols_words), "1" if args.hold_cpu else "0",
                        mutate=True).returncode
    if name == "rom-load":
        return run_jtag(args, "rom-load", str(args.rom), hex_u32(args.base),
                        mutate=True).returncode
    raise AssertionError(name)


def _do_passthrough_openocd(args: argparse.Namespace) -> int:
    name = args.command
    if name == "axi-read":
        result = run_openocd(
            args,
            f'puts [format "0x%08X" {_openocd_read_expr(args.addr)}]',
            capture=True,
        )
        if result.returncode != 0:
            print(result.stdout or "", end="")
            return result.returncode
        for line in (result.stdout or "").splitlines():
            line = line.strip()
            if re.fullmatch(r"0x[0-9A-Fa-f]{8}", line):
                print(line)
                return 0
        print(result.stdout or "", end="")
        return 0
    if name == "axi-write":
        return run_openocd(
            args,
            f'write_memory {hex_u32(args.addr)} 32 [list {hex_u32(args.data)}] phys',
            mutate=True,
        ).returncode
    if name == "debug-halt-status":
        result = run_openocd(args, *_openocd_status_lines(), capture=True)
        if result.returncode != 0:
            print(result.stdout or "", end="")
            return result.returncode
        print(_format_openocd_status(parse_kv_output(result.stdout or ""),
                                     source="OpenOCD"))
        return 0
    if name == "debug-reset-halt":
        return run_openocd(
            args,
            f'write_memory {hex_u32(regs.OFF_DBG_CONTROL)} 32 '
            f'[list {hex_u32(regs.CTL_HALT_REQ | regs.CTL_SOFT_RST)}] phys',
            mutate=True,
        ).returncode
    if name == "debug-halt-after":
        ctl_bits = args.halt_ctl
        if ctl_bits is None:
            ctl_bits = regs.HALT_AFTER_ENABLE | regs.HALT_CLEAR_LATCH
        script = [
            f'write_memory {hex_u32(regs.OFF_DBG_HALT_AFTER_LO)} 32 '
            f'[list {hex_u32(args.inst_count & 0xFFFFFFFF)}] phys',
            f'write_memory {hex_u32(regs.OFF_DBG_HALT_AFTER_HI)} 32 '
            f'[list {hex_u32((args.inst_count >> 32) & 0xFFFFFFFF)}] phys',
            f'write_memory {hex_u32(regs.OFF_DBG_HALT_CTL)} 32 '
            f'[list {hex_u32(ctl_bits)}] phys',
        ]
        return run_openocd(args, *script, mutate=True).returncode
    if name == "debug-break-pc":
        ctl_bits = args.halt_ctl
        if ctl_bits is None:
            ctl_bits = regs.HALT_CLEAR_LATCH
        script = [
            f'write_memory {hex_u32(regs.OFF_DBG_BREAK_PC)} 32 '
            f'[list {hex_u32(args.pc)}] phys',
            f'write_memory {hex_u32(regs.OFF_DBG_BREAK_PC_CTRL)} 32 '
            f'[list {hex_u32(1)}] phys',
            f'write_memory {hex_u32(regs.OFF_DBG_HALT_CTL)} 32 '
            f'[list {hex_u32(ctl_bits)}] phys',
        ]
        return run_openocd(args, *script, mutate=True).returncode
    if name == "debug-halt-exc":
        ctl_bits = args.halt_ctl
        if ctl_bits is None:
            ctl_bits = regs.HALT_CLEAR_LATCH
        lane = (args.vec >> 5) & 7
        mask = 1 << (args.vec & 31)
        script = [
            f'write_memory {hex_u32(regs.OFF_DBG_HALT_EXC_MASK0 + i * 4)} 32 '
            f'[list {hex_u32(mask if i == lane else 0)}] phys'
            for i in range(8)
        ] + [
            f'write_memory {hex_u32(regs.OFF_DBG_HALT_CTL)} 32 '
            f'[list {hex_u32(ctl_bits)}] phys',
        ]
        return run_openocd(args, *script, mutate=True).returncode
    if name == "debug-clear-halt":
        enable_bits = args.enable_bits or 0
        return run_openocd(
            args,
            f'write_memory {hex_u32(regs.OFF_DBG_HALT_CTL)} 32 '
            f'[list {hex_u32(enable_bits | regs.HALT_CLEAR_LATCH)}] phys',
            mutate=True,
        ).returncode
    if name == "debug-step":
        script = []
        if args.halt_first:
            script.append(
                f'write_memory {hex_u32(regs.OFF_DBG_CONTROL)} 32 '
                f'[list {hex_u32(regs.CTL_HALT_REQ)}] phys'
            )
        script.append(
            f'write_memory {hex_u32(regs.OFF_DBG_CONTROL)} 32 '
            f'[list {hex_u32(regs.CTL_HALT_REQ | regs.CTL_STEP_PULSE)}] phys'
        )
        return run_openocd(args, *script, mutate=True).returncode
    if name == "status":
        return do_status(args)
    if name == "dashboard":
        return do_dashboard(args)
    raise AssertionError(name)


def add_common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--backend", choices=["vivado", "openocd"],
                        default="vivado",
                        help="transport backend for debug_ctrl / AXI operations")
    parser.add_argument("--vivado", help="Vivado executable; defaults to VIVADO env or vivado on PATH")
    parser.add_argument("--openocd-bin", help="OpenOCD executable; defaults to OPENOCD env or openocd on PATH")
    parser.add_argument("--openocd-cfg", action="append", default=[],
                        help="OpenOCD config file (interface, target, or board); repeatable")
    parser.add_argument("--openocd-target", help="current target name to select after init")
    parser.add_argument("--bit", type=Path, help="bitstream path for PROGRAM_FPGA/BIT_FILE")
    parser.add_argument("--ltx", type=Path, help="probe file path for LTX_FILE")
    parser.add_argument("--target-re", help="regexp filter for get_hw_targets")
    parser.add_argument("--device-re", help="regexp filter for get_hw_devices")
    parser.add_argument("--dry-run", action="store_true", help="print command instead of executing")
    parser.add_argument("--go", action="store_true", help="allow mutating JTAG/program operations")


def add_safety(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--dry-run", action="store_true", default=argparse.SUPPRESS,
                        help="print command instead of executing")
    parser.add_argument("--go", action="store_true", default=argparse.SUPPRESS,
                        help="allow this mutating operation")


def add_artifacts(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--bit", type=Path, default=argparse.SUPPRESS,
                        help="bitstream path")
    parser.add_argument("--ltx", type=Path, default=argparse.SUPPRESS,
                        help="probe file path")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Near-TUI JTAG bring-up wrapper for Vivado hw_server",
    )
    add_common(parser)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("status", help="one decoded VIO snapshot")
    p.add_argument("--mock", type=Path, help="parse saved Tcl output instead of running Vivado")
    p.set_defaults(func=do_status)

    p = sub.add_parser("snapshot", help="alias for status; attaches LTX without reprogramming")
    p.add_argument("--mock", type=Path, help="parse saved Tcl output instead of running Vivado")
    p.set_defaults(func=do_status)

    p = sub.add_parser("dashboard", help="refresh-loop decoded VIO dashboard")
    p.add_argument("--interval", type=float, default=5.0)
    p.add_argument("--count", type=int, default=0, help="0 means forever")
    p.add_argument("--once", action="store_true")
    p.add_argument("--mock", type=Path, help="parse saved Tcl output instead of running Vivado")
    p.set_defaults(func=do_dashboard)

    p = sub.add_parser("program", help="run synth/program_fpga.sh; requires matching .ltx unless --no-vio")
    add_safety(p)
    add_artifacts(p)
    p.add_argument("--allow-stale", action="store_true")
    p.add_argument("--no-vio", action="store_true")
    p.set_defaults(func=do_program)

    sub.add_parser("vio-get", help="read probe_out0").set_defaults(func=do_passthrough)

    p = sub.add_parser("vio-set", help="set raw probe_out0 control value; dry-run unless --go")
    add_safety(p)
    p.add_argument("value", type=parse_u32)
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser("hold", help="set probe_out0=0x5: bypass SD and hold CPU reset")
    add_safety(p)
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser("release", help="set probe_out0=0x3: bypass SD and release CPU")
    add_safety(p)
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser("sd-boot", help="set probe_out0=0x0: return to SD boot controls")
    add_safety(p)
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser("debug-full-reset",
                       help=("pulse probe_out0[3]=1 (jtag_debug_full_reset): "
                             "re-arm reset overlay + reset VIA1, then release "
                             "for cold-boot CPU resume; dry-run unless --go"))
    add_safety(p)
    p.add_argument("--hold-ms", dest="hold_ms", type=int, default=100,
                   help="how long (ms) to hold the bit asserted (default 100)")
    p.add_argument("--no-release", dest="release", action="store_false",
                   default=True,
                   help="leave bit[3] asserted; caller releases via vio-set 0")
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser("axi-read", help="single 32-bit JTAG AXI read")
    p.add_argument("addr", type=parse_u32)
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser("axi-write", help="single 32-bit JTAG AXI write; dry-run unless --go")
    add_safety(p)
    p.add_argument("addr", type=parse_u32)
    p.add_argument("data", type=parse_u32)
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser("debug-halt-status", help="read debug_ctrl halt/breakpoint registers")
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser("debug-halt-after", help="halt core after instruction-boundary count reaches N; dry-run unless --go")
    add_safety(p)
    p.add_argument("inst_count", type=parse_u64)
    p.add_argument("--halt-ctl", type=parse_u32,
                   help="raw HALT_CTL bits; default enables halt-after and clears latch (0x5)")
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser("debug-break-pc", help="halt before the instruction at PC takes effect; dry-run unless --go")
    add_safety(p)
    p.add_argument("pc", type=parse_u32)
    p.add_argument("--halt-ctl", type=parse_u32,
                   help="raw HALT_CTL bits; default enables break-pc and clears latch (0x6)")
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser("debug-halt-exc", help="halt core after precise exception vector entry; default vector is illegal instruction (4)")
    add_safety(p)
    p.add_argument("vec", type=parse_u32, nargs="?", default=4)
    p.add_argument("--halt-ctl", type=parse_u32,
                   help="raw HALT_CTL bits; default enables halt-exc and clears latch (0x44)")
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser("debug-reset-halt", help="pulse CPU soft reset and keep manual halt asserted; dry-run unless --go")
    add_safety(p)
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser(
        "debug-run-from-reset-halt-after",
        help="reset the CPU, run until halt-after trips, then dump status/snapshot; dry-run unless --go",
    )
    add_safety(p)
    p.add_argument("inst_count", type=parse_u64,
                   help="retired-boundary count to stop at after reset")
    p.add_argument("wait_ms", type=parse_u32, nargs="?", default=50,
                   help="time to wait after releasing reset before sampling")
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser(
        "debug-sweep-reset-halt-after",
        help="repeat reset/run/halt-after for each requested count in one Vivado session; dry-run unless --go",
    )
    add_safety(p)
    p.add_argument("wait_ms", type=parse_u32,
                   help="time to wait after each release before sampling")
    p.add_argument("inst_counts", type=parse_u64, nargs="+",
                   help="retired-boundary counts to sweep")
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser("debug-clear-halt", help="clear latched debug auto-halt; dry-run unless --go")
    add_safety(p)
    p.add_argument("enable_bits", type=parse_u32, nargs="?",
                   help="HALT_CTL enable bits to preserve while clearing")
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser(
        "debug-step",
        help="single-step one retired boundary; requires a halted core unless --halt-first is supplied",
    )
    add_safety(p)
    p.add_argument(
        "--halt-first",
        action="store_true",
        help="request halt first if the CPU is running, then single-step once halted",
    )
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser("debug-arch-write", help="write one halt-time arch shadow register; dry-run unless --go")
    add_safety(p)
    p.add_argument("reg", help="D0-D7, A0-A7, PC, SR, VBR, USP, SSP, ISP, CACR, SFC, DFC, ITT0/1, DTT0/1, TC, URP, SRP")
    p.add_argument("value", type=parse_u32)
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser("debug-arch-apply", help="apply halt-time arch shadow state and resume; dry-run unless --go")
    add_safety(p)
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser("video-poke", help="program DAFB/CLUT and paint a visible VRAM rectangle; dry-run unless --go")
    add_safety(p)
    p.add_argument("fb_base", type=parse_u32, nargs="?", default=0x100,
                   help="DAFB framebuffer base/pixel offset")
    p.add_argument("stride", type=parse_u32, nargs="?", default=0x61E,
                   help="DAFB framebuffer stride in pixels/bytes for 8bpp")
    p.add_argument("rows", type=int, nargs="?", default=48,
                   help="number of rows to paint")
    p.add_argument("cols_words", type=int, nargs="?", default=96,
                   help="32-bit words per row to paint")
    p.add_argument("--hold-cpu", action="store_true",
                   help="assert VIO force_cpu_rst while painting so the ROM cannot overwrite it")
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser(
        "vram-wrap-poke",
        help="paint active scanout VRAM addresses without touching DAFB; dry-run unless --go",
    )
    add_safety(p)
    p.add_argument("fb_base", type=parse_u32, nargs="?", default=0xF0010,
                   help="current scanout framebuffer base/pixel offset")
    p.add_argument("stride", type=parse_u32, nargs="?", default=0x40000,
                   help="current scanout framebuffer stride")
    p.add_argument("rows", type=int, nargs="?", default=4,
                   help="number of source rows/phases to paint")
    p.add_argument("cols_words", type=int, nargs="?", default=256,
                   help="32-bit words per row to paint")
    p.add_argument("--hold-cpu", action="store_true",
                   help="assert VIO force_cpu_rst while painting so the ROM cannot overwrite it")
    p.set_defaults(func=do_passthrough)

    p = sub.add_parser("rom-load", help="load a ROM file through JTAG AXI; dry-run unless --go")
    add_safety(p)
    p.add_argument("rom", type=Path)
    p.add_argument("base", type=parse_u32, nargs="?", default=0x40000000)
    p.set_defaults(func=do_passthrough)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
