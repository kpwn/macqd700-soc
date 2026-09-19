#!/usr/bin/env python3
"""Compare synthetic UpdateMEM output to Vivado's same-placement INIT rewrite.

Only the .bit container header may differ. Compare the complete configuration
packet stream from its synchronization word onward, including frame CRC/ECC.
"""
from pathlib import Path
import sys


def payload(path):
    data = Path(path).read_bytes()
    marker = bytes.fromhex("aa995566")
    position = data.find(marker)
    if position < 0:
        raise ValueError(f"no bitstream sync word: {path}")
    return data[position:]


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: check_adb_roundtrip.py reference.bit patched.bit")
    reference, patched = map(payload, sys.argv[1:])
    if reference != patched:
        mismatch = next((i for i, (a, b) in enumerate(zip(reference, patched)) if a != b), None)
        sys.exit(f"ADB round-trip mismatch: first offset={mismatch}, lengths={len(reference)}/{len(patched)}")
    print("ADB_UPDATEMEM_ROUNDTRIP_PASS: complete configuration packet payload matches Vivado")


if __name__ == "__main__":
    main()
