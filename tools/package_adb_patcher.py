#!/usr/bin/env python3
"""Create the standalone release download; never include firmware or binaries."""
import argparse
from pathlib import Path
import zipfile


def package(output):
    root = Path(__file__).resolve().parents[1]
    members = {
        "patch_adb_bitstream.py": root / "tools/patch_adb_bitstream.py",
        "adb_config.py": root / "tools/adb_config.py",
        "prepare_adb_firmware.py": root / "tools/prepare_adb_firmware.py",
        "README.md": root / "docs/adb_firmware_bitstream.md",
        "LICENSE": root / "LICENSE",
        "THIRD_PARTY_NOTICES.md": root / "THIRD_PARTY_NOTICES.md",
    }
    with zipfile.ZipFile(output, "x", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, path in sorted(members.items()):
            info = zipfile.ZipInfo("adb-patcher/" + name, date_time=(2026, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o100644 << 16
            archive.writestr(info, path.read_bytes())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    package(args.output)
