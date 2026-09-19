#!/usr/bin/env python3
"""Check high-impact model-vs-RTL map/contract consistency.

This guard is intentionally narrow: it checks the memory-map constants and
RAM-window controls that must stay aligned across RTL and host-side models to
avoid sim-vs-FPGA surprises.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except OSError as exc:
        raise RuntimeError(f"failed reading {path}: {exc}") from exc


def parse_vh_define(text: str, name: str) -> int:
    m = re.search(rf"^\s*`define\s+{re.escape(name)}\s+32'h([0-9a-fA-F_]+)", text, re.MULTILINE)
    if not m:
        raise RuntimeError(f"missing define `{name}`")
    return int(m.group(1).replace("_", ""), 16)


def parse_cpp_u32_const(text: str, name: str) -> int:
    m = re.search(
        rf"\bstatic\s+constexpr\s+uint32_t\s+{re.escape(name)}\s*=\s*(0x[0-9a-fA-F]+)u?\s*;",
        text,
    )
    if not m:
        raise RuntimeError(f"missing C++ constant `{name}`")
    return int(m.group(1), 16)


def parse_v_localparam_dec(text: str, name: str) -> int:
    m = re.search(rf"\b{name}\s*=\s*6'd(\d+)", text)
    if not m:
        raise RuntimeError(f"missing localparam `{name}`")
    return int(m.group(1), 10)


def parse_debug_reset_lg2(text: str) -> int:
    m = re.search(r"ram_window_lg2_r\s*<=\s*6'd(\d+)\s*;", text)
    if not m:
        raise RuntimeError("missing debug_ctrl reset assignment for ram_window_lg2_r")
    return int(m.group(1), 10)


def parse_debug_clamp_lg2(text: str) -> tuple[int, int]:
    m_min = re.search(r"w_data_r\[5:0\]\s*<\s*6'd(\d+)\s*\)\s*[\r\n]+\s*ram_window_lg2_r\s*<=\s*6'd(\d+)", text)
    m_max = re.search(r"w_data_r\[5:0\]\s*>\s*6'd(\d+)\s*\)\s*[\r\n]+\s*ram_window_lg2_r\s*<=\s*6'd(\d+)", text)
    if not m_min or not m_max:
        raise RuntimeError("missing debug_ctrl clamp logic for ram_window_lg2_r")
    cond_min, set_min = int(m_min.group(1), 10), int(m_min.group(2), 10)
    cond_max, set_max = int(m_max.group(1), 10), int(m_max.group(2), 10)
    if cond_min != set_min:
        raise RuntimeError(
            f"debug_ctrl min clamp mismatch: condition {cond_min} != assigned {set_min}"
        )
    if cond_max != set_max:
        raise RuntimeError(
            f"debug_ctrl max clamp mismatch: condition {cond_max} != assigned {set_max}"
        )
    return cond_min, cond_max


def parse_glue_asc_upper(text: str) -> int:
    m = re.search(r"io_asc_hit\s*=.*io_off\s*>=\s*24'h014000\)\s*&&\s*\(io_off\s*<\s*24'h([0-9a-fA-F]+)\)", text)
    if not m:
        raise RuntimeError("missing glue ASC decode range")
    return int(m.group(1), 16)


def parse_periph_bus_asc_upper(text: str) -> int:
    m = re.search(r"mac_off\s*>=\s*24'h014000\)\s*&&\s*\(mac_off\s*<\s*24'h([0-9a-fA-F]+)\)", text)
    if not m:
        raise RuntimeError("missing peripheral_bus ASC decode range")
    return int(m.group(1), 16)


def parse_v_localparam_hex(text: str, name: str) -> int:
    m = re.search(rf"\b{name}\s*=\s*32'h([0-9a-fA-F_]+)", text)
    if not m:
        raise RuntimeError(f"missing localparam `{name}`")
    return int(m.group(1).replace("_", ""), 16)


def check(desc: str, got: int, expect: int, failures: list[str]) -> None:
    if got == expect:
        print(f"[OK]   {desc}: 0x{got:08x}")
    else:
        print(f"[FAIL] {desc}: got 0x{got:08x}, expected 0x{expect:08x}")
        failures.append(desc)


def main() -> int:
    parser = argparse.ArgumentParser(description="Check model-vs-RTL consistency")
    parser.add_argument(
        "--repo",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="Repository root (default: inferred from this script)",
    )
    args = parser.parse_args()
    repo = args.repo.resolve()

    axi_defs = read_text(repo / "rtl/soc/axi_defs.vh")
    mem_model = read_text(repo / "tb/models/mem_model.h")
    xbar = read_text(repo / "rtl/soc/axi_xbar.v")
    dbg = read_text(repo / "rtl/core/debug/debug_ctrl.v")
    glue = read_text(repo / "rtl/mac/glue.v")
    pbus = read_text(repo / "rtl/soc/peripheral_bus.v")
    rbh = read_text(repo / "tb/models/rom_boot_bus.h")

    failures: list[str] = []

    # RTL axi_defs.vh vs host mem_model.h
    check("RAM base", parse_cpp_u32_const(mem_model, "RAM_BASE"),
          parse_vh_define(axi_defs, "AXI_RAM_BASE"), failures)
    check("RAM decode size", parse_cpp_u32_const(mem_model, "RAM_DECODE_SIZE"),
          parse_vh_define(axi_defs, "AXI_RAM_SIZE"), failures)
    check("ROM base", parse_cpp_u32_const(mem_model, "ROM_BASE"),
          parse_vh_define(axi_defs, "AXI_ROM_BASE"), failures)
    check("ROM size", parse_cpp_u32_const(mem_model, "ROM_SIZE"),
          parse_vh_define(axi_defs, "AXI_ROM_SIZE"), failures)
    check("VRAM base", parse_cpp_u32_const(mem_model, "VRAM_BASE"),
          parse_vh_define(axi_defs, "AXI_VRAM_BASE"), failures)
    check("VRAM size", parse_cpp_u32_const(mem_model, "VRAM_SIZE"),
          parse_vh_define(axi_defs, "AXI_VRAM_SIZE"), failures)

    # DDR flatten offsets should chain contiguously.
    axi_ram_size = parse_vh_define(axi_defs, "AXI_RAM_SIZE")
    axi_rom_size = parse_vh_define(axi_defs, "AXI_ROM_SIZE")
    check("DDR ROM offset == RAM decode size", parse_vh_define(axi_defs, "AXI_DDR_ROM_OFFSET"),
          axi_ram_size, failures)
    check("DDR FB offset == ROM offset + ROM size", parse_vh_define(axi_defs, "AXI_DDR_FB_OFFSET"),
          parse_vh_define(axi_defs, "AXI_DDR_ROM_OFFSET") + axi_rom_size, failures)

    # RAM-window lg2 controls: xbar and debug_ctrl must agree exactly.
    xbar_min = parse_v_localparam_dec(xbar, "RAM_WINDOW_LG2_MIN")
    xbar_max = parse_v_localparam_dec(xbar, "RAM_WINDOW_LG2_MAX")
    xbar_dflt = parse_v_localparam_dec(xbar, "RAM_WINDOW_LG2_DFLT")
    dbg_min, dbg_max = parse_debug_clamp_lg2(dbg)
    dbg_dflt = parse_debug_reset_lg2(dbg)

    check("RAM window lg2 min", dbg_min, xbar_min, failures)
    check("RAM window lg2 max", dbg_max, xbar_max, failures)
    check("RAM window lg2 default", dbg_dflt, xbar_dflt, failures)
    check("RAM window min bytes", parse_cpp_u32_const(mem_model, "RAM_WINDOW_MIN"),
          1 << xbar_min, failures)
    check("RAM window default bytes", parse_cpp_u32_const(mem_model, "RAM_WINDOW_DEFAULT"),
          1 << xbar_dflt, failures)
    check("RAM window max/decode bytes", parse_cpp_u32_const(mem_model, "RAM_DECODE_SIZE"),
          1 << xbar_max, failures)

    # Peripheral decode windows: host RomBootBus vs RTL decode.
    glue_asc_upper = parse_glue_asc_upper(glue)
    pbus_asc_upper = parse_periph_bus_asc_upper(pbus)
    check("ASC decode upper (glue vs peripheral_bus)", pbus_asc_upper, glue_asc_upper, failures)
    check("RomBootBus ASC size", parse_cpp_u32_const(rbh, "ASC_SIZE"),
          glue_asc_upper - 0x014000, failures)

    dafb_base = parse_v_localparam_hex(glue, "DAFB_REG_BASE")
    dafb_end = parse_v_localparam_hex(glue, "DAFB_REG_END")
    check("RomBootBus DAFB reg base", parse_cpp_u32_const(rbh, "DAFB_REG_BASE"),
          dafb_base, failures)
    check("RomBootBus DAFB reg size", parse_cpp_u32_const(rbh, "DAFB_REG_SIZE"),
          dafb_end - dafb_base, failures)

    if failures:
        print(f"\nmodel/rtl consistency: FAIL ({len(failures)} checks)")
        return 1
    print("\nmodel/rtl consistency: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
