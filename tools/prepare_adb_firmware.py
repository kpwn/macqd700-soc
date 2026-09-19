#!/usr/bin/env python3
"""Convert a user-supplied 342S0440-B dump to the PIC's readmemh format.

No firmware is bundled. The dump contains 512 little-endian 12-bit words
in 16-bit containers. Do not mask invalid upper bits: that hides wrong input.
"""
import argparse
from pathlib import Path
import struct


def convert(data):
    if len(data) != 1024:
        raise ValueError("expected a 1024-byte 342S0440-B dump")
    words = struct.unpack("<512H", data)
    if any(word > 0xFFF for word in words):
        raise ValueError("dump contains non-12-bit words; check format and byte order")
    return "".join(f"{word:03x}\n" for word in words)


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", type=Path,
                        default=root / "files/342s0440-b.bin")
    parser.add_argument("--output", type=Path,
                        default=root / "rtl/mac/adb_pic_fw.hex")
    args = parser.parse_args()
    try:
        contents = convert(args.input.read_bytes())
        args.output.write_text(contents, encoding="ascii")
    except (OSError, ValueError) as error:
        parser.exit(1, f"ADB firmware: {error}\nSupply your own dump; see files/README.md.\n")
    print(f"Prepared {args.output} (local firmware; do not publish)")


if __name__ == "__main__":
    main()
