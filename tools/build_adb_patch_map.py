#!/usr/bin/env python3
"""Publisher-only map generation and vendor comparison for Python ADB bundles.

Uses synthetic firmware only. Requires UpdateMEM and an uncompressed, blank
bitstream exported from the final routed checkpoint. End users do not run this
tool: the resulting map lets patch_adb_bitstream.py operate with Python alone.
No bit offset or BRAM placement is assumed across builds.
"""
import argparse
import hashlib
import json
from pathlib import Path
import random
import struct
import subprocess
import tempfile

from adb_config import ConfigImage, FIRMWARE_BITS, FORMAT, insert_firmware, validate_map
from patch_adb_bitstream import patch, verify_bundle


def changed_bits(base, other):
    if len(base.words) != len(other.words) or base.packets != other.packets:
        raise ValueError("vendor changed the configuration packet layout")
    crc = set(base.crc_indices)
    result = set()
    for index, (before, after) in enumerate(zip(base.words, other.words)):
        if index in crc:
            continue
        delta = before ^ after
        while delta:
            low = delta & -delta
            result.add(index * 32 + low.bit_length() - 1)
            delta ^= low
    return result


def identify_locations(candidates, patterns):
    """13 address-code patterns uniquely label each of the 6144 physical bits."""
    if len(candidates) != FIRMWARE_BITS or len(patterns) != 13:
        raise ValueError("expected exactly 6144 data bits and 13 address patterns")
    labels = dict.fromkeys(candidates, 0)
    for shift, changed in enumerate(patterns):
        if not changed <= candidates:
            raise ValueError("vendor changed bits outside the firmware data map (including ECC)")
        for location in changed:
            labels[location] |= 1 << shift
    locations = [None] * FIRMWARE_BITS
    for location, label in labels.items():
        if not 1 <= label <= FIRMWARE_BITS or locations[label - 1] is not None:
            raise ValueError("ambiguous/non-permutation firmware mapping")
        locations[label - 1] = location
    return locations


def pattern_firmware(predicate):
    return struct.pack("<512H", *[
        sum(1 << bit for bit in range(12) if predicate(address * 12 + bit))
        for address in range(512)
    ])


def build(bit, mmi, manifest, output, updatemem):
    if output.exists() or output.resolve() in {p.resolve() for p in (bit, mmi, manifest)}:
        raise ValueError("output must be a new map file")
    verify_bundle(bit, mmi, manifest)
    data = bit.read_bytes()
    base = ConfigImage(data)
    base.crc()
    with tempfile.TemporaryDirectory(prefix="adb-map-synthetic-") as directory:
        root = Path(directory)
        references = []

        def reference(name, firmware):
            source, result = root / f"{name}.bin", root / f"{name}.bit"
            source.write_bytes(firmware)
            patch(bit, mmi, manifest, source, result, updatemem)
            image = ConfigImage(result.read_bytes())
            references.append((name, firmware, result))
            return image

        candidates = changed_bits(base, reference("all-ones", pattern_firmware(lambda _: True)))
        changes = []
        for shift in range(13):
            firmware = pattern_firmware(lambda index: (index + 1) & (1 << shift))
            changes.append(changed_bits(base, reference(f"address-{shift:02d}", firmware)))
        mapping = {
            "format": FORMAT,
            "bit_sha256": hashlib.sha256(data).hexdigest(),
            "bit_locations": identify_locations(candidates, changes),
        }
        validate_map(data, mapping)
        # These are independent of the patterns used to discover the mapping.
        reference("zero", bytes(1024))
        reference("alternating", struct.pack("<512H", *([0x555, 0xAAA] * 256)))
        reference("boundaries", pattern_firmware(
            lambda index: index // 12 in (0, 7, 8, 255, 256, 511)))
        for seed in (0, 1, 0xADB040):
            rng = random.Random(seed)
            reference(f"random-{seed}", struct.pack("<512H", *[
                rng.randrange(4096) for _ in range(512)]))
        # Compare EVERY configuration word, including all CRCs, ECC, unused
        # memory, parity, startup commands and non-PIC configuration. Only the
        # variable .bit metadata header is excluded.
        for name, firmware, path in references:
            expected = ConfigImage(path.read_bytes())
            actual = ConfigImage(insert_firmware(data, mapping, firmware))
            if actual.words != expected.words:
                raise ValueError(f"Python/vendor configuration mismatch: {name}")
            print(f"PASS Python/vendor complete payload: {name}", flush=True)
        mapping["validation"] = {
            "vendor_payload_comparisons": len(references),
            "frame_ecc": "unchanged; verified against every vendor payload",
        }
        with output.open("x", encoding="ascii") as out:
            json.dump(mapping, out, indent=2)
            out.write("\n")
    print(f"Created {output}: Python-only map; no firmware included", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for arg in ("bit", "mmi", "manifest", "output"):
        parser.add_argument(f"--{arg}", type=Path, required=True)
    parser.add_argument("--updatemem", default="updatemem")
    args = vars(parser.parse_args())
    try:
        build(**args)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"ADB patch map: {error}\n")


if __name__ == "__main__":
    main()
