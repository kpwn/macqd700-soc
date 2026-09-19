#!/usr/bin/env python3
"""Validate the repo-generated PCIe/XDMA XCI and manifest."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


EXPECTED = {
    "Component_Name": "design_1_xdma_0_0",
    "functional_mode": "DMA",
    "mode_selection": "Advanced",
    "pcie_blk_locn": "X0Y0",
    "pl_link_cap_max_link_width": "X4",
    "pl_link_cap_max_link_speed": "8.0_GT/s",
    "ref_clk_freq": "100_MHz",
    "axi_data_width": "128_bit",
    "axisten_freq": "250",
    "en_axi_master_if": "true",
    "axist_bypass_en": "true",
    "axist_bypass_size": "4",
    "axist_bypass_scale": "Megabytes",
    "xdma_rnum_chnl": "4",
    "xdma_wnum_chnl": "4",
    "xdma_num_usr_irq": "9",
    "select_quad": "GTY_Quad_224",
    "plltype": "QPLL1",
    "ext_sys_clk_bufg": "true",
    "xdma_pcie_64bit_en": "true",
    "axi_bypass_64bit_en": "true",
    "axi_id_width": "4",
    "pf0_bar0_size": "128",
    "pf0_bar0_scale": "Kilobytes",
    "pciebar2axibar_axist_bypass": "0x0000000000000000",
}

MANIFEST_EXPECTED = {
    "ip": "design_1_xdma_0_0",
    "part": "xcku5p-ffvb676-2-i",
    "source_component": "xilinx.com:ip:xdma:4.2 in Vivado 2025.2 (contract from pcie_test xdma:4.1 rev 23 / Vivado 2023.1)",
    "pcie_link": "Gen3_x4",
    "gt_quad": "GTY_Quad_224",
    "axi_aclk_mhz": "250",
    "axi_data_width": "128",
    "bypass_enabled": "true",
    "bypass_bar_size": "4_Megabytes",
}


def _read_manifest(path: Path) -> dict[str, str]:
    data: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise RuntimeError(f"malformed manifest line in {path}: {line!r}")
        key, value = line.split("=", 1)
        data[key] = value
    return data


def _xci_params(path: Path) -> dict[str, str]:
    data = json.loads(path.read_text(encoding="utf-8"))
    ip_inst = data["ip_inst"]
    if ip_inst["component_reference"] != "xilinx.com:ip:xdma:4.2":
        raise RuntimeError(f"unexpected component_reference in {path}")
    if ip_inst["ip_revision"] != "2":
        raise RuntimeError(f"unexpected XDMA ip_revision in {path}")
    params = ip_inst["parameters"]["component_parameters"]
    return {key: str(value[0]["value"]) for key, value in params.items()}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dir", type=Path, required=True,
                    help="generated build/pcie_xdma directory")
    args = ap.parse_args()

    out_dir = args.dir.resolve()
    xci = out_dir / "design_1_xdma_0_0.xci"
    manifest_path = out_dir / "design_1_xdma_0_0_manifest.txt"
    if not xci.exists():
        raise RuntimeError(f"missing generated XCI: {xci}")
    if not manifest_path.exists():
        raise RuntimeError(f"missing generated manifest: {manifest_path}")

    params = _xci_params(xci)
    for key, expected in EXPECTED.items():
        actual = params.get(key)
        if actual != expected:
            raise RuntimeError(
                f"XDMA XCI mismatch for {key}: expected {expected!r}, got {actual!r}"
            )

    manifest = _read_manifest(manifest_path)
    for key, expected in MANIFEST_EXPECTED.items():
        actual = manifest.get(key)
        if actual != expected:
            raise RuntimeError(
                f"XDMA manifest mismatch for {key}: expected {expected!r}, got {actual!r}"
            )
    if Path(manifest.get("xci", "")).resolve() != xci:
        raise RuntimeError("manifest xci path does not match generated XCI")

    print(f"PASS: PCIe/XDMA generated IP manifest validated: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
