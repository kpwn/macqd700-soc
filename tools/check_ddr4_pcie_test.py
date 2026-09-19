#!/usr/bin/env python3
"""Compare this repo's DDR4 shell with known KU5P DDR4 reference projects.

The first hardware run should not rely on memory about the working Vivado
project.  This preflight checks the low-level facts we can compare without
building a netlist: DDR4 pin locations, exposed MIG wrapper ports, and the
AXI geometry exported by the working XCI.

The pcie_test project is the active build contract because synth/vivado.tcl
stitches its OOC MIG DCP into rtl/sys/ddr_ctrl.v.  The factory image_ku5p
project is advisory: it is a second board reference for DDR pins, but its MIG
timing/part/AXI-ID settings differ from pcie_test and should not be silently
mixed into the current DCP-stitch path.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path


PIN_RE = re.compile(
    r"set_property\s+PACKAGE_PIN\s+(\S+)\s+\[get_ports\s+(?:\{([^}]+)\}|([^\]\s]+))\]"
)
DECL_RE = re.compile(
    r"^\s*(input|output|inout)\s+(?:(wire|reg)\s+)?(?:\[(\d+):(\d+)\]\s*)?([A-Za-z0-9_]+)\s*(?:[,;]|$)",
    re.MULTILINE,
)
COMMENT_RE = re.compile(r"/\*.*?\*/|//[^\n]*", re.DOTALL)
TCL_PARAM_RE = re.compile(r"^\s*\{([^{}\s]+)\s+([^{}]+?)\}\s*$")

PCIE_TO_REPO_PORT = {
    "DDR4_DIFF_CLK_clk_p": "sys_clk_p",
    "DDR4_DIFF_CLK_clk_n": "sys_clk_n",
    "ddr4_rtl_0_act_n": "ddr4_act_n",
    "ddr4_rtl_0_reset_n": "ddr4_reset_n",
    "ddr4_rtl_0_odt": "ddr4_odt",
    "ddr4_rtl_0_cs_n": "ddr4_cs_n",
    "ddr4_rtl_0_cke": "ddr4_cke",
    "ddr4_rtl_0_ck_t": "ddr4_ck_t",
    "ddr4_rtl_0_ck_c": "ddr4_ck_c",
    "ddr4_rtl_0_bg": "ddr4_bg",
    "ddr4_rtl_0_ba": "ddr4_ba",
    "ddr4_rtl_0_adr": "ddr4_adr",
    "ddr4_rtl_0_dm_n": "ddr4_dm_dbi_n",
    "ddr4_rtl_0_dqs_t": "ddr4_dqs_t",
    "ddr4_rtl_0_dqs_c": "ddr4_dqs_c",
    "ddr4_rtl_0_dq": "ddr4_dq",
}

MIG_KEYS = {
    "C0.DDR4_MemoryPart": "MT40A512M16LY-075",
    "C0.DDR4_DataWidth": "32",
    "C0.DDR4_DataMask": "DM_NO_DBI",
    "C0.DDR4_EN_PARITY": "false",
    "C0.DDR4_TimePeriod": "750",
    "C0.DDR4_InputClockPeriod": "5000",
    "C0.DDR4_PhyClockRatio": "4:1",
    "C0.DDR4_AxiDataWidth": "256",
    "C0.DDR4_AxiAddressWidth": "31",
    "C0.DDR4_AxiIDWidth": "1",
}

PCIE_BD_NETS = {
    "ddr4_0_c0_ddr4_ui_clk": {
        "ddr4_0/c0_ddr4_ui_clk",
        "rst_ddr4_0_333M/slowest_sync_clk",
        "axi_smc/aclk",
    },
    "ddr4_0_c0_ddr4_ui_clk_sync_rst": {
        "ddr4_0/c0_ddr4_ui_clk_sync_rst",
        "rst_ddr4_0_333M/ext_reset_in",
    },
    "ddr4_0_c0_init_calib_complete": {
        "ddr4_0/c0_init_calib_complete",
        "c0_init_calib_complete_0",
    },
    "reset_rtl_0_1": {
        "ddr4_mig_resetn",
        "util_vector_logic_0/Op1",
    },
    "util_vector_logic_0_Res": {
        "util_vector_logic_0/Res",
        "ddr4_0/sys_rst",
    },
    "rst_ddr4_0_333M_peripheral_aresetn": {
        "rst_ddr4_0_333M/peripheral_aresetn",
        "ddr4_0/c0_ddr4_aresetn",
    },
}

PCIE_INTERFACE_NETS = {
    "diff_clock_rtl_0_1": {
        "DDR4_DIFF_CLK",
        "ddr4_0/C0_SYS_CLK",
    },
    "ddr4_0_C0_DDR4": {
        "ddr4_rtl_0",
        "ddr4_0/C0_DDR4",
    },
}

PCIE_INTERFACES = {
    ("C0_SYS_CLK", "FREQ_HZ"): "200000000",
    ("C0_DDR4_CLOCK", "FREQ_HZ"): "333250000",
    ("C0_DDR4_ARESETN", "POLARITY"): "ACTIVE_LOW",
    ("C0_DDR4_RESET", "POLARITY"): "ACTIVE_HIGH",
}

PCIE_MIG_STUB_PORTS = {
    "sys_rst": ("input", 1),
    "c0_sys_clk_p": ("input", 1),
    "c0_sys_clk_n": ("input", 1),
    "c0_ddr4_act_n": ("output", 1),
    "c0_ddr4_adr": ("output", 17),
    "c0_ddr4_ba": ("output", 2),
    "c0_ddr4_bg": ("output", 1),
    "c0_ddr4_cke": ("output", 1),
    "c0_ddr4_odt": ("output", 1),
    "c0_ddr4_cs_n": ("output", 1),
    "c0_ddr4_ck_t": ("output", 1),
    "c0_ddr4_ck_c": ("output", 1),
    "c0_ddr4_reset_n": ("output", 1),
    "c0_ddr4_dm_dbi_n": ("inout", 4),
    "c0_ddr4_dq": ("inout", 32),
    "c0_ddr4_dqs_c": ("inout", 4),
    "c0_ddr4_dqs_t": ("inout", 4),
    "c0_init_calib_complete": ("output", 1),
    "c0_ddr4_ui_clk": ("output", 1),
    "c0_ddr4_ui_clk_sync_rst": ("output", 1),
    "dbg_clk": ("output", 1),
    "c0_ddr4_aresetn": ("input", 1),
    "c0_ddr4_s_axi_awid": ("input", 1),
    "c0_ddr4_s_axi_awaddr": ("input", 31),
    "c0_ddr4_s_axi_awlen": ("input", 8),
    "c0_ddr4_s_axi_awsize": ("input", 3),
    "c0_ddr4_s_axi_awburst": ("input", 2),
    "c0_ddr4_s_axi_awlock": ("input", 1),
    "c0_ddr4_s_axi_awcache": ("input", 4),
    "c0_ddr4_s_axi_awprot": ("input", 3),
    "c0_ddr4_s_axi_awqos": ("input", 4),
    "c0_ddr4_s_axi_awvalid": ("input", 1),
    "c0_ddr4_s_axi_awready": ("output", 1),
    "c0_ddr4_s_axi_wdata": ("input", 256),
    "c0_ddr4_s_axi_wstrb": ("input", 32),
    "c0_ddr4_s_axi_wlast": ("input", 1),
    "c0_ddr4_s_axi_wvalid": ("input", 1),
    "c0_ddr4_s_axi_wready": ("output", 1),
    "c0_ddr4_s_axi_bready": ("input", 1),
    "c0_ddr4_s_axi_bid": ("output", 1),
    "c0_ddr4_s_axi_bresp": ("output", 2),
    "c0_ddr4_s_axi_bvalid": ("output", 1),
    "c0_ddr4_s_axi_arid": ("input", 1),
    "c0_ddr4_s_axi_araddr": ("input", 31),
    "c0_ddr4_s_axi_arlen": ("input", 8),
    "c0_ddr4_s_axi_arsize": ("input", 3),
    "c0_ddr4_s_axi_arburst": ("input", 2),
    "c0_ddr4_s_axi_arlock": ("input", 1),
    "c0_ddr4_s_axi_arcache": ("input", 4),
    "c0_ddr4_s_axi_arprot": ("input", 3),
    "c0_ddr4_s_axi_arqos": ("input", 4),
    "c0_ddr4_s_axi_arvalid": ("input", 1),
    "c0_ddr4_s_axi_arready": ("output", 1),
    "c0_ddr4_s_axi_rready": ("input", 1),
    "c0_ddr4_s_axi_rid": ("output", 1),
    "c0_ddr4_s_axi_rdata": ("output", 256),
    "c0_ddr4_s_axi_rresp": ("output", 2),
    "c0_ddr4_s_axi_rlast": ("output", 1),
    "c0_ddr4_s_axi_rvalid": ("output", 1),
    "dbg_bus": ("output", 512),
}

REPO_RESET_PINS = {
    "cpu_resetn": "T19",
    "btn[0]": "K9",
}

REPO_FPGA_TOP_REAL_PORTS = {
    "sys_clk_p": ("input", 1),
    "sys_clk_n": ("input", 1),
    "fabric_clk_p": ("input", 1),
    "fabric_clk_n": ("input", 1),
    "ddr4_act_n": ("output", 1),
    "ddr4_adr": ("output", 17),
    "ddr4_ba": ("output", 2),
    "ddr4_bg": ("output", 1),
    "ddr4_cke": ("output", 1),
    "ddr4_odt": ("output", 1),
    "ddr4_cs_n": ("output", 1),
    "ddr4_ck_t": ("output", 1),
    "ddr4_ck_c": ("output", 1),
    "ddr4_reset_n": ("output", 1),
    "ddr4_dm_dbi_n": ("inout", 4),
    "ddr4_dq": ("inout", 32),
    "ddr4_dqs_c": ("inout", 4),
    "ddr4_dqs_t": ("inout", 4),
}

REPO_DDR_CTRL_MIG_UI_PORTS = {
    "mig_ui_clk": ("input", 1),
    "mig_ui_rst": ("input", 1),
    "mig_cal_done": ("input", 1),
    "mig_awid": ("output", 1),
    "mig_awaddr": ("output", 31),
    "mig_awlen": ("output", 8),
    "mig_awsize": ("output", 3),
    "mig_awburst": ("output", 2),
    "mig_awvalid": ("output", 1),
    "mig_awready": ("input", 1),
    "mig_wdata": ("output", 256),
    "mig_wstrb": ("output", 32),
    "mig_wlast": ("output", 1),
    "mig_wvalid": ("output", 1),
    "mig_wready": ("input", 1),
    "mig_bid": ("input", 1),
    "mig_bresp": ("input", 2),
    "mig_bvalid": ("input", 1),
    "mig_bready": ("output", 1),
    "mig_arid": ("output", 1),
    "mig_araddr": ("output", 31),
    "mig_arlen": ("output", 8),
    "mig_arsize": ("output", 3),
    "mig_arburst": ("output", 2),
    "mig_arvalid": ("output", 1),
    "mig_arready": ("input", 1),
    "mig_rready": ("output", 1),
    "mig_rid": ("input", 1),
    "mig_rdata": ("input", 256),
    "mig_rresp": ("input", 2),
    "mig_rlast": ("input", 1),
    "mig_rvalid": ("input", 1),
}

BRIDGE_PORTS = {
    "clk": ("input", 1),
    "rst": ("input", 1),
    "s_awid": ("input", 6),
    "s_awaddr": ("input", 32),
    "s_awlen": ("input", 8),
    "s_awsize": ("input", 3),
    "s_awburst": ("input", 2),
    "s_awvalid": ("input", 1),
    "s_awready": ("output", 1),
    "s_wdata": ("input", 128),
    "s_wstrb": ("input", 16),
    "s_wlast": ("input", 1),
    "s_wvalid": ("input", 1),
    "s_wready": ("output", 1),
    "s_bid": ("output", 6),
    "s_bresp": ("output", 2),
    "s_bvalid": ("output", 1),
    "s_bready": ("input", 1),
    "s_arid": ("input", 6),
    "s_araddr": ("input", 32),
    "s_arlen": ("input", 8),
    "s_arsize": ("input", 3),
    "s_arburst": ("input", 2),
    "s_arvalid": ("input", 1),
    "s_arready": ("output", 1),
    "s_rid": ("output", 6),
    "s_rdata": ("output", 128),
    "s_rresp": ("output", 2),
    "s_rlast": ("output", 1),
    "s_rvalid": ("output", 1),
    "s_rready": ("input", 1),
    "m_awid": ("output", 1),
    "m_awaddr": ("output", 31),
    "m_awlen": ("output", 8),
    "m_awsize": ("output", 3),
    "m_awburst": ("output", 2),
    "m_awvalid": ("output", 1),
    "m_awready": ("input", 1),
    "m_wdata": ("output", 256),
    "m_wstrb": ("output", 32),
    "m_wlast": ("output", 1),
    "m_wvalid": ("output", 1),
    "m_wready": ("input", 1),
    "m_bid": ("input", 1),
    "m_bresp": ("input", 2),
    "m_bvalid": ("input", 1),
    "m_bready": ("output", 1),
    "m_arid": ("output", 1),
    "m_araddr": ("output", 31),
    "m_arlen": ("output", 8),
    "m_arsize": ("output", 3),
    "m_arburst": ("output", 2),
    "m_arvalid": ("output", 1),
    "m_arready": ("input", 1),
    "m_rid": ("input", 1),
    "m_rdata": ("input", 256),
    "m_rresp": ("input", 2),
    "m_rlast": ("input", 1),
    "m_rvalid": ("input", 1),
    "m_rready": ("output", 1),
}


def parse_pin_xdc(path: Path) -> dict[str, str]:
    pins: dict[str, str] = {}
    for match in PIN_RE.finditer(path.read_text()):
        pin, braced_port, bare_port = match.groups()
        port = braced_port or bare_port
        pins[port] = pin
    return pins


def normalize_indexed_pin_map(raw: dict[str, str]) -> dict[str, str]:
    """Map "foo[3]" and scalar "foo" names to the same spelling."""
    return {k.replace("[", "[").replace("]", "]"): v for k, v in raw.items()}


def mapped_repo_name(pcie_port: str) -> str | None:
    base = pcie_port.split("[", 1)[0]
    suffix = ""
    if "[" in pcie_port:
        suffix = "[" + pcie_port.split("[", 1)[1]
    repo_base = PCIE_TO_REPO_PORT.get(base)
    if repo_base is None:
        return None
    return repo_base + suffix


def xci_param(params: dict, key: str) -> str | None:
    value = params.get(key)
    if not value:
        return None
    return str(value[0].get("value"))


def xci_interface_param(interfaces: dict, interface: str, key: str) -> str | None:
    params = interfaces.get(interface, {}).get("parameters", {})
    value = params.get(key)
    if not value:
        return None
    return str(value[0].get("value"))


def bd_component_param(components: dict, component: str, key: str) -> str | None:
    params = components.get(component, {}).get("parameters", {})
    value = params.get(key)
    if isinstance(value, dict):
        return str(value.get("value"))
    return None


def require_net(
    nets: dict,
    name: str,
    expected_ports: set[str],
    errors: list[str],
    kind: str = "net",
) -> None:
    actual_ports = set(nets.get(name, {}).get("ports", []))
    if not actual_ports:
        actual_ports = set(nets.get(name, {}).get("interface_ports", []))
    missing = expected_ports - actual_ports
    if missing:
        errors.append(
            f"pcie_test BD {kind} {name} missing {sorted(missing)}; "
            f"actual ports {sorted(actual_ports)}"
        )


def require_ports(
    ports: dict[str, tuple[str, int]],
    expected_ports: dict[str, tuple[str, int]],
    label: str,
    errors: list[str],
) -> None:
    for name, expected in sorted(expected_ports.items()):
        actual = ports.get(name)
        if actual is None:
            errors.append(f"{label} missing port {name}")
        elif actual != expected:
            errors.append(
                f"{label} port {name} expected direction/width {expected}, got {actual}"
            )


def parse_ports(path: Path) -> dict[str, tuple[str, int]]:
    ports: dict[str, tuple[str, int]] = {}
    text = COMMENT_RE.sub("", path.read_text())
    for direction, _kind, left, right, name in DECL_RE.findall(text):
        width = 1
        if left and right:
            width = abs(int(left) - int(right)) + 1
        ports[name] = (direction, width)
    return ports


def require_file(path: Path, label: str) -> None:
    if not path.exists():
        raise FileNotFoundError(f"{label} not found: {path}")


def compare_mapped_pins(
    repo_pins: dict[str, str],
    ref_pins: dict[str, str],
    ref_label: str,
    errors: list[str],
) -> int:
    checked_pins = 0
    for ref_port, ref_pin in sorted(ref_pins.items()):
        repo_port = mapped_repo_name(ref_port)
        if repo_port is None:
            continue
        repo_pin = repo_pins.get(repo_port)
        if repo_pin is None:
            errors.append(
                f"repo lacks mapped constraint for {ref_label} {ref_port} -> {repo_port}"
            )
            continue
        checked_pins += 1
        if repo_pin != ref_pin:
            errors.append(
                f"{ref_label} pin mismatch {ref_port} ({ref_pin}) -> "
                f"{repo_port} ({repo_pin})"
            )
    return checked_pins


def load_xci_params(path: Path) -> dict:
    with path.open() as f:
        xci = json.load(f)
    return xci["ip_inst"]["parameters"]["component_parameters"]


def load_xci(path: Path) -> dict:
    with path.open() as f:
        return json.load(f)


def load_tcl_param_pairs(path: Path, var_name: str) -> dict[str, str]:
    params: dict[str, str] = {}
    in_list = False
    for line in path.read_text().splitlines():
        stripped = line.strip()
        if not in_list:
            if stripped == f"set {var_name} {{":
                in_list = True
            continue
        if stripped == "}":
            break
        match = TCL_PARAM_RE.match(line)
        if match:
            key, value = match.groups()
            params[key] = value.strip()
    return params


def summarize_mig_params(params: dict, label: str) -> str:
    keys = [
        "C0.DDR4_MemoryPart",
        "C0.DDR4_DataWidth",
        "C0.DDR4_DataMask",
        "C0.DDR4_EN_PARITY",
        "C0.DDR4_TimePeriod",
        "C0.DDR4_InputClockPeriod",
        "C0.DDR4_PhyClockRatio",
        "C0.DDR4_AxiDataWidth",
        "C0.DDR4_AxiAddressWidth",
        "C0.DDR4_AxiIDWidth",
    ]
    facts = ", ".join(f"{key}={xci_param(params, key)}" for key in keys)
    return f"{label} MIG config: {facts}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path.cwd())
    parser.add_argument(
        "--pcie-test",
        type=Path,
        default=Path.home() / "FPGA" / "pcie_test",
        help="Path to the working pcie_test Vivado project",
    )
    parser.add_argument(
        "--factory-image",
        type=Path,
        default=None,
        help=(
            "Optional ALINX/RK-XCKU5P-F factory image_ku5p Vivado project. "
            "When omitted, only the active pcie_test build contract is checked."
        ),
    )
    args = parser.parse_args()

    repo = args.repo.resolve()
    pcie = args.pcie_test.expanduser().resolve()
    factory = args.factory_image.expanduser().resolve() if args.factory_image else None

    repo_ddr_xdc = repo / "synth" / "ddr4.xdc"
    repo_top_xdc = repo / "synth" / "fpga_top.xdc"
    repo_top = repo / "rtl" / "fpga_top.v"
    repo_ddr_ctrl = repo / "rtl" / "sys" / "ddr_ctrl.v"
    repo_bridge = repo / "rtl" / "sys" / "axi_ddr4_mig_bridge.v"
    repo_mig_gen = repo / "synth" / "gen_ddr4_mig.tcl"
    pcie_xdc = pcie / "pcie_test.srcs" / "constrs_1" / "new" / "pcie_test.xdc"
    pcie_xci = (
        pcie
        / "pcie_test.srcs"
        / "sources_1"
        / "bd"
        / "design_1"
        / "ip"
        / "design_1_ddr4_0_1"
        / "design_1_ddr4_0_1.xci"
    )
    pcie_bd = pcie / "pcie_test.srcs" / "sources_1" / "bd" / "design_1" / "design_1.bd"
    pcie_wrapper = (
        pcie
        / "pcie_test.gen"
        / "sources_1"
        / "bd"
        / "design_1"
        / "hdl"
        / "design_1_wrapper.v"
    )
    pcie_mig_stub = (
        pcie
        / "pcie_test.gen"
        / "sources_1"
        / "bd"
        / "design_1"
        / "ip"
        / "design_1_ddr4_0_1"
        / "design_1_ddr4_0_1_stub.v"
    )
    factory_xdc = None
    factory_xci = None
    if factory is not None:
        factory_xdc = factory / "image_ku5p.srcs" / "constrs_1" / "new" / "ddr4.xdc"
        factory_xci = (
            factory
            / "image_ku5p.srcs"
            / "sources_1"
            / "bd"
            / "design_1"
            / "ip"
            / "design_1_ddr4_0_0"
            / "design_1_ddr4_0_0.xci"
        )

    for path, label in [
        (repo_ddr_xdc, "repo DDR4 XDC"),
        (repo_top_xdc, "repo top XDC"),
        (repo_top, "repo fpga_top"),
        (repo_ddr_ctrl, "repo ddr_ctrl"),
        (repo_bridge, "repo DDR4 MIG AXI bridge"),
        (repo_mig_gen, "repo DDR4 MIG generator"),
        (pcie_xdc, "pcie_test XDC"),
        (pcie_xci, "pcie_test DDR4 XCI"),
        (pcie_bd, "pcie_test block design"),
        (pcie_wrapper, "pcie_test wrapper"),
        (pcie_mig_stub, "pcie_test DDR4 MIG stub"),
    ]:
        require_file(path, label)

    repo_pins = normalize_indexed_pin_map(parse_pin_xdc(repo_top_xdc))
    repo_pins.update(normalize_indexed_pin_map(parse_pin_xdc(repo_ddr_xdc)))
    pcie_pins = normalize_indexed_pin_map(parse_pin_xdc(pcie_xdc))

    errors: list[str] = []
    warnings: list[str] = []

    checked_pins = compare_mapped_pins(repo_pins, pcie_pins, "pcie_test", errors)

    pcie_ports = parse_ports(pcie_wrapper)
    repo_ports = parse_ports(repo_top)
    forbidden_repo_ports = ["ddr4_parity", "ddr4_alert_n"]
    for port in forbidden_repo_ports:
        if port in repo_ports:
            errors.append(
                f"repo top still exposes {port}, but pcie_test parity/alert ports are absent"
            )
    for pcie_port in pcie_ports:
        if pcie_port.startswith("ddr4_rtl_0_") or pcie_port.startswith("DDR4_DIFF_CLK_"):
            repo_port = mapped_repo_name(pcie_port)
            if repo_port and repo_port not in repo_ports:
                errors.append(f"repo top lacks mapped DDR4 wrapper port {repo_port}")

    require_ports(
        repo_ports,
        REPO_FPGA_TOP_REAL_PORTS,
        "repo fpga_top real-DDR shell",
        errors,
    )

    repo_ddr_ctrl_ports = parse_ports(repo_ddr_ctrl)
    require_ports(
        repo_ddr_ctrl_ports,
        REPO_DDR_CTRL_MIG_UI_PORTS,
        "repo ddr_ctrl MIG UI shell",
        errors,
    )

    pcie_mig_ports = parse_ports(pcie_mig_stub)
    require_ports(
        pcie_mig_ports,
        PCIE_MIG_STUB_PORTS,
        "pcie_test MIG stub",
        errors,
    )

    xci = load_xci(pcie_xci)
    params = xci["ip_inst"]["parameters"]["component_parameters"]
    for key, expected in MIG_KEYS.items():
        actual = xci_param(params, key)
        if actual != expected:
            errors.append(f"pcie_test XCI {key} expected {expected}, got {actual}")

    generator_params = load_tcl_param_pairs(repo_mig_gen, "ddr4_params")
    for key, expected in MIG_KEYS.items():
        actual = generator_params.get(key)
        if actual != expected:
            errors.append(
                f"repo gen_ddr4_mig.tcl {key} expected {expected}, got {actual}"
            )

    factory_checked_pins = None
    if factory is None:
        pass
    elif factory.exists():
        assert factory_xdc is not None
        assert factory_xci is not None
        if factory_xdc.exists():
            factory_pins = normalize_indexed_pin_map(parse_pin_xdc(factory_xdc))
            factory_checked_pins = compare_mapped_pins(
                repo_pins, factory_pins, "factory image_ku5p", errors
            )
        else:
            warnings.append(f"factory image_ku5p DDR4 XDC not found: {factory_xdc}")

        if factory_xci.exists():
            factory_params = load_xci_params(factory_xci)
            warnings.append(summarize_mig_params(factory_params, "factory image_ku5p"))
            for key in MIG_KEYS:
                factory_value = xci_param(factory_params, key)
                pcie_value = xci_param(params, key)
                if factory_value != pcie_value:
                    warnings.append(
                        "factory image_ku5p differs from pcie_test "
                        f"{key}: factory={factory_value}, pcie_test={pcie_value}"
                    )
        else:
            warnings.append(f"factory image_ku5p DDR4 XCI not found: {factory_xci}")
    else:
        warnings.append(f"factory image_ku5p project not found: {factory}")

    interfaces = xci["ip_inst"]["boundary"]["interfaces"]
    for (interface, key), expected in PCIE_INTERFACES.items():
        actual = xci_interface_param(interfaces, interface, key)
        if actual != expected:
            errors.append(
                f"pcie_test XCI interface {interface}.{key} expected {expected}, got {actual}"
            )

    with pcie_bd.open() as f:
        bd = json.load(f)["design"]
    for name, expected_ports in PCIE_BD_NETS.items():
        require_net(bd["nets"], name, expected_ports, errors)
    for name, expected_ports in PCIE_INTERFACE_NETS.items():
        require_net(bd["interface_nets"], name, expected_ports, errors, "interface net")
    util_op = bd_component_param(bd["components"], "util_vector_logic_0", "C_OPERATION")
    if util_op != "not":
        errors.append(
            "pcie_test BD util_vector_logic_0.C_OPERATION expected not, "
            f"got {util_op}"
        )

    for port, expected_pin in REPO_RESET_PINS.items():
        actual_pin = repo_pins.get(port)
        if actual_pin != expected_pin:
            errors.append(
                f"repo reset contract pin {port} expected {expected_pin}, got {actual_pin}"
            )

    bridge_ports = parse_ports(repo_bridge)
    for port, (expected_dir, expected_width) in BRIDGE_PORTS.items():
        actual = bridge_ports.get(port)
        if actual != (expected_dir, expected_width):
            errors.append(
                f"repo bridge port {port} expected "
                f"{expected_dir}[{expected_width}], got {actual}"
            )

    xbar_note = (
        "known-good MIG AXI: data=256 addr=31 id=1 ui_clk=333250000; "
        "repo xbar/SIM_MODEL path remains data=128 addr=32 id=6; "
        "axi_ddr4_mig_bridge provides the explicit single-beat contract shim"
    )
    reset_note = (
        "known-good reset/clock: ddr4_mig_resetn is active-low and inverted "
        "to MIG sys_rst; MIG c0_ddr4_ui_clk drives proc_sys_reset/AXI at "
        "333250000 Hz; repo keeps cpu_resetn=T19 and btn[0]=K9 available"
    )

    print(f"DDR4 pcie_test comparison: {checked_pins} mapped pin constraints checked")
    if factory_checked_pins is not None:
        print(
            "DDR4 factory image_ku5p advisory comparison: "
            f"{factory_checked_pins} mapped pin constraints checked"
        )
    print(
        "DDR4 pcie_test comparison: "
        f"{len(PCIE_MIG_STUB_PORTS)} MIG stub ports checked"
    )
    print(
        "DDR4 pcie_test comparison: "
        f"{len(REPO_FPGA_TOP_REAL_PORTS)} repo fpga_top real-DDR ports checked"
    )
    print(
        "DDR4 pcie_test comparison: "
        f"{len(REPO_DDR_CTRL_MIG_UI_PORTS)} repo ddr_ctrl MIG UI ports checked"
    )
    print(
        "DDR4 pcie_test comparison: "
        f"{len(BRIDGE_PORTS)} repo bridge ports checked"
    )
    print(
        "DDR4 pcie_test comparison: "
        f"{len(MIG_KEYS)} repo generator MIG parameters checked"
    )
    for warning in warnings:
        print(f"WARN: {warning}")
    print(f"INFO: {xbar_note}")
    print(f"INFO: {reset_note}")

    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1

    print("DDR4 pcie_test comparison: PASS")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except FileNotFoundError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
