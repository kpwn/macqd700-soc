#!/usr/bin/env python3
"""Validate and stamp the repo-generated DDR4 MIG cache."""

from __future__ import annotations

import argparse
import hashlib
import sys
from pathlib import Path


IP_NAME = "design_1_ddr4_0_1"
CACHE_METADATA_VERSION = "1"

MANIFEST_EXPECTED = {
    "ip": IP_NAME,
    "part": "xcku5p-ffvb676-2-i",
    "source": "pcie_test design_1_ddr4_0_1.xci",
    "source_component": "xilinx.com:ip:ddr4:2.2 revision 28 in Vivado 2025.2",
    "memory_part": "MT40A512M16LY-075",
    "memory_period_ps": "750",
    "input_clock_period_ps": "5000",
    "phy_clock_ratio": "4:1",
    "ddr4_data_width": "32",
    "ddr4_data_mask": "DM_NO_DBI",
    "ddr4_parity": "false",
    "axi_data_width": "256",
    "axi_addr_width": "31",
    "axi_id_width": "1",
    "ui_clock_hz": "333250000",
    "board_constraints": "synth/ddr4.xdc and synth/fpga_top_real_mig.xdc",
}

CACHE_KEYS = {
    "cache_metadata_version",
    "cache_mode",
    "cache_generator",
    "cache_generator_sha256",
    "cache_xci_sha256",
    "cache_dcp_sha256",
}


class CacheMiss(RuntimeError):
    """Raised when generated DDR4 MIG artifacts are not reusable."""


def artifact_paths(out_dir: Path) -> dict[str, Path]:
    return {
        "xci": out_dir / f"{IP_NAME}.xci",
        "dcp": out_dir / f"{IP_NAME}.dcp",
        "manifest": out_dir / f"{IP_NAME}_manifest.txt",
    }


def hash_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_manifest(path: Path) -> dict[str, str]:
    data: dict[str, str] = {}
    for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if "=" not in stripped:
            raise CacheMiss(f"malformed manifest line {lineno}: {line!r}")
        key, value = stripped.split("=", 1)
        data[key] = value
    return data


def _require_file(path: Path, label: str) -> None:
    if not path.is_file():
        raise CacheMiss(f"missing {label}: {path}")


def _manifest_path_matches(manifest: dict[str, str], key: str, expected: Path) -> None:
    actual = manifest.get(key)
    if actual is None:
        raise CacheMiss(f"manifest missing {key}")
    if Path(actual).resolve() != expected.resolve():
        raise CacheMiss(f"manifest {key} path does not match {expected}")


def _validate_generated(
    out_dir: Path,
    requested_mode: str,
    *,
    allow_synth_for_validate: bool,
) -> tuple[dict[str, str], dict[str, Path], bool]:
    paths = artifact_paths(out_dir)
    _require_file(paths["xci"], "DDR4 MIG XCI")
    _require_file(paths["manifest"], "DDR4 MIG manifest")

    manifest = read_manifest(paths["manifest"])
    manifest_mode = manifest.get("mode")
    if requested_mode == "synth":
        if manifest_mode != "synth":
            raise CacheMiss(f"manifest mode is {manifest_mode!r}, expected 'synth'")
    elif requested_mode == "validate":
        allowed = {"validate"}
        if allow_synth_for_validate:
            allowed.add("synth")
        if manifest_mode not in allowed:
            raise CacheMiss(
                f"manifest mode is {manifest_mode!r}, expected one of {sorted(allowed)}"
            )
    else:
        raise ValueError(f"unsupported DDR4 MIG cache mode: {requested_mode}")

    dcp_required = requested_mode == "synth"
    if dcp_required:
        _require_file(paths["dcp"], "DDR4 MIG synth DCP")

    for key, expected in MANIFEST_EXPECTED.items():
        actual = manifest.get(key)
        if actual != expected:
            raise CacheMiss(
                f"manifest {key} mismatch: expected {expected!r}, got {actual!r}"
            )

    output_dir = manifest.get("output_dir")
    if output_dir is None:
        raise CacheMiss("manifest missing output_dir")
    if Path(output_dir).resolve() != out_dir.resolve():
        raise CacheMiss(f"manifest output_dir does not match {out_dir}")
    _manifest_path_matches(manifest, "xci", paths["xci"])
    _manifest_path_matches(manifest, "dcp", paths["dcp"])

    return manifest, paths, dcp_required


def _validate_cache_metadata(
    manifest: dict[str, str],
    paths: dict[str, Path],
    generator: Path,
    dcp_required: bool,
) -> None:
    if manifest.get("cache_metadata_version") != CACHE_METADATA_VERSION:
        raise CacheMiss("manifest cache metadata is missing or from an unsupported version")
    if manifest.get("cache_mode") != manifest.get("mode"):
        raise CacheMiss("manifest cache mode does not match generated mode")

    expected_generator = generator.resolve()
    actual_generator = manifest.get("cache_generator")
    if actual_generator is None:
        raise CacheMiss("manifest missing cache_generator")
    if Path(actual_generator).resolve() != expected_generator:
        raise CacheMiss("manifest cache_generator path does not match current generator")

    generator_sha = manifest.get("cache_generator_sha256")
    if generator_sha != hash_file(generator):
        raise CacheMiss("generator script hash changed")

    xci_sha = manifest.get("cache_xci_sha256")
    if xci_sha != hash_file(paths["xci"]):
        raise CacheMiss("generated XCI hash changed")

    dcp_sha = manifest.get("cache_dcp_sha256")
    if dcp_required:
        if dcp_sha != hash_file(paths["dcp"]):
            raise CacheMiss("generated DCP hash changed")
    elif dcp_sha is not None and paths["dcp"].is_file():
        if dcp_sha != hash_file(paths["dcp"]):
            raise CacheMiss("generated DCP hash changed")


def check_cache(out_dir: Path, generator: Path, mode: str) -> None:
    _require_file(generator, "DDR4 MIG generator script")
    manifest, paths, dcp_required = _validate_generated(
        out_dir.resolve(), mode, allow_synth_for_validate=True
    )
    _validate_cache_metadata(manifest, paths, generator.resolve(), dcp_required)


def _cache_metadata_lines(
    manifest: dict[str, str],
    paths: dict[str, Path],
    generator: Path,
    dcp_required: bool,
) -> list[str]:
    lines = [
        f"cache_metadata_version={CACHE_METADATA_VERSION}",
        f"cache_mode={manifest['mode']}",
        f"cache_generator={generator.resolve()}",
        f"cache_generator_sha256={hash_file(generator)}",
        f"cache_xci_sha256={hash_file(paths['xci'])}",
    ]
    if dcp_required:
        lines.append(f"cache_dcp_sha256={hash_file(paths['dcp'])}")
    return lines


def stamp_cache(out_dir: Path, generator: Path, mode: str) -> Path:
    _require_file(generator, "DDR4 MIG generator script")
    manifest, paths, dcp_required = _validate_generated(
        out_dir.resolve(), mode, allow_synth_for_validate=False
    )
    manifest_path = paths["manifest"]
    source_lines = manifest_path.read_text(encoding="utf-8").splitlines()
    kept_lines = [
        line for line in source_lines
        if line.strip().split("=", 1)[0] not in CACHE_KEYS
    ]
    kept_lines.extend(_cache_metadata_lines(manifest, paths, generator.resolve(), dcp_required))
    manifest_path.write_text("\n".join(kept_lines) + "\n", encoding="utf-8")
    return manifest_path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="command", required=True)
    for name in ("check", "stamp"):
        sp = sub.add_parser(name)
        sp.add_argument("--mode", choices=("validate", "synth"), required=True)
        sp.add_argument("--dir", type=Path, required=True,
                        help="generated build/ddr4_mig directory")
        sp.add_argument("--generator", type=Path, required=True,
                        help="synth/gen_ddr4_mig.tcl path")
        if name == "check":
            sp.add_argument("--explain-miss", action="store_true",
                            help="print cache miss reason to stderr")
    args = ap.parse_args(argv)

    try:
        if args.command == "check":
            check_cache(args.dir, args.generator, args.mode)
            required = "XCI/manifest/DCP" if args.mode == "synth" else "XCI/manifest"
            print(
                f"DDR4 MIG cache hit ({args.mode}): {args.dir} has valid "
                f"{required}; skipping Vivado"
            )
            return 0

        manifest_path = stamp_cache(args.dir, args.generator, args.mode)
        print(f"DDR4 MIG cache metadata stamped ({args.mode}): {manifest_path}")
        return 0
    except CacheMiss as exc:
        if args.command == "check":
            if getattr(args, "explain_miss", False):
                print(f"DDR4 MIG cache miss ({args.mode}): {exc}", file=sys.stderr)
            return 1
        print(f"ERROR: DDR4 MIG cache stamp failed: {exc}", file=sys.stderr)
        return 2
    except OSError as exc:
        print(f"ERROR: DDR4 MIG cache tool failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
