#!/usr/bin/env python3
"""Compare repeated read-only native SD matrices at 25 and 50 MHz.

Cross-speed CRC equality is a hard integrity gate. An optional host image
comparison is reported separately because a running Mac may legitimately
change sectors after the image was provisioned.
"""

import argparse
import csv
import statistics
import sys
import zlib
from pathlib import Path


def read_matrix(path: Path) -> dict[tuple[int, int], tuple[int, float]]:
    rows: dict[tuple[int, int], dict[int, tuple[int, float]]] = {}
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream, delimiter="\t")
        required = {"lba", "blocks", "repeat", "crc32", "MB_per_s"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"{path}: missing columns {required - set(reader.fieldnames or [])}")
        for row in reader:
            key = (int(row["lba"]), int(row["blocks"]))
            repeat = int(row["repeat"])
            crc = int(row["crc32"], 16)
            rate = float(row["MB_per_s"])
            if key[0] < 0 or not 1 <= key[1] <= 32 or repeat not in (1, 2) or rate <= 0:
                raise ValueError(f"{path}: invalid row {row}")
            samples = rows.setdefault(key, {})
            if repeat in samples:
                raise ValueError(f"{path}: duplicate repeat {repeat} at {key}")
            samples[repeat] = (crc, rate)
    result = {}
    for key, readings in rows.items():
        if readings.keys() != {1, 2} or readings[1][0] != readings[2][0]:
            raise ValueError(f"{path}: missing repeat or repeated CRC mismatch at {key}")
        result[key] = (readings[1][0], statistics.mean(rate for _, rate in readings.values()))
    if not result:
        raise ValueError(f"{path}: empty matrix")
    return result


def host_crc(image: Path, lba: int, blocks: int, base_lba: int) -> int | None:
    offset = (lba - base_lba) * 512
    length = blocks * 512
    if offset < 0 or offset + length > image.stat().st_size:
        return None
    with image.open("rb") as stream:
        stream.seek(offset)
        data = stream.read(length)
    if len(data) != length:
        raise ValueError(f"short host image read at LBA {lba}")
    return zlib.crc32(data)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("default_matrix", type=Path, help="25 MHz summary.tsv")
    parser.add_argument("high_speed_matrix", type=Path, help="50 MHz summary.tsv")
    parser.add_argument("--image", type=Path, help="optional HDD image for advisory CRC comparison")
    parser.add_argument("--image-base-lba", type=int, default=8192)
    args = parser.parse_args()
    normal = read_matrix(args.default_matrix)
    fast = read_matrix(args.high_speed_matrix)
    if normal.keys() != fast.keys():
        raise ValueError(f"matrix ranges differ: 25 MHz only={normal.keys() - fast.keys()}, "
                         f"50 MHz only={fast.keys() - normal.keys()}")
    print("lba\tblocks\tcrc32\t25_MBps\t50_MBps\tratio\thost_image")
    mismatches = []
    ratios = []
    for (lba, blocks) in sorted(normal):
        crc25, rate25 = normal[lba, blocks]
        crc50, rate50 = fast[lba, blocks]
        if crc25 != crc50:
            mismatches.append((lba, blocks, crc25, crc50))
        ratio = rate50 / rate25
        if blocks == 32:
            ratios.append(ratio)
        image_status = "-"
        if args.image:
            expected = host_crc(args.image, lba, blocks, args.image_base_lba)
            if expected is not None:
                image_status = "match" if expected == crc25 else f"diff:0x{expected:08x}"
        print(f"{lba}\t{blocks}\t0x{crc25:08x}\t{rate25:.3f}\t{rate50:.3f}\t"
              f"{ratio:.2f}\t{image_status}")
    if mismatches:
        for lba, blocks, crc25, crc50 in mismatches:
            print(f"ERROR LBA {lba} blocks {blocks}: 25 MHz 0x{crc25:08x} "
                  f"!= 50 MHz 0x{crc50:08x}", file=sys.stderr)
        return 1
    if not ratios:
        raise ValueError("no 32-sector range to compare throughput")
    print(f"PASS {len(normal)} ranges: 25/50 MHz CRCs agree; "
          f"median 32-sector throughput ratio {statistics.median(ratios):.2f}x")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
