#!/usr/bin/env python3
"""Insert the user's ADB PIC dump into a hash-bound, blank BRAM bitstream.

This implementation requires AMD UpdateMEM on PATH (or --updatemem). It does
not synthesize/place/route. It is NOT a standalone UltraScale+ frame patcher.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from prepare_adb_firmware import convert


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_mmi(path):
    root = ET.fromstring(path.read_bytes())
    processors = root.findall("Processor")
    if len(processors) != 1 or processors[0].attrib != {
        "Endianness": "Little", "InstPath": "adb_pic"
    }:
        raise ValueError("expected only the adb_pic little-endian memory map")
    spaces = processors[0].findall("AddressSpace")
    if len(spaces) != 1 or spaces[0].attrib != {
        "Name": "program", "Begin": "0", "End": "4095"
    }:
        raise ValueError("unexpected PIC address space")
    lanes = root.findall(".//BitLane")
    if len(lanes) != 1 or lanes[0].get("MemType") != "RAMB32":
        raise ValueError("expected exactly one RAMB36")
    lane = lanes[0]
    if not re.fullmatch(r"X\d+Y\d+", lane.get("Placement", "")):
        raise ValueError("missing physical BRAM placement")
    for tag, attrs in {
        "DataWidth": {"MSB": "31", "LSB": "0"},
        "AddressRange": {"Begin": "0", "End": "1023"},
        "Parity": {"ON": "false", "NumBits": "0"},
    }.items():
        element = lane.find(tag)
        if element is None or element.attrib != attrs:
            raise ValueError(f"unexpected PIC {tag}")


def memory_text(data):
    convert(data)  # Shared size/12-bit/byte-order validation; never mask input.
    words = list(struct.unpack("<512H", data)) + [0] * 512
    # UpdateMEM consumes each textual token as bytes in file order and applies
    # the MMI's Little endianness. Emit little-endian bytes, NOT numeric hex:
    # PIC 0x123 is token 23010000, yielding physical 32-bit word 0x00000123.
    return "@00000000\n" + "".join(struct.pack("<I", word).hex() + "\n" for word in words)


def seal(bit, mmi, manifest):
    validate_mmi(mmi)
    # Called by the Vivado exporter AFTER it checks that INIT/INITP are zero.
    # Hashes bind the files; they do not independently prove firmware absence.
    contents = {
        "format": "macqd700-adb-bram-v1", "processor": "adb_pic",
        "bit_sha256": digest(bit), "mmi_sha256": digest(mmi),
    }
    with manifest.open("x", encoding="ascii") as out:
        json.dump(contents, out, indent=2)
        out.write("\n")


def verify_bundle(bit, mmi, manifest):
    bundle = json.loads(manifest.read_text(encoding="ascii"))
    if bundle.get("format") != "macqd700-adb-bram-v1" or bundle.get("processor") != "adb_pic":
        raise ValueError("unsupported ADB patch manifest")
    if digest(bit) != bundle.get("bit_sha256") or digest(mmi) != bundle.get("mmi_sha256"):
        raise ValueError("bitstream/MMI hash mismatch; use the matching release bundle")
    validate_mmi(mmi)


def patch(bit, mmi, manifest, firmware, output, updatemem):
    verify_bundle(bit, mmi, manifest)
    contents = memory_text(firmware.read_bytes())
    if output.exists() or output.resolve() in {
        path.resolve() for path in (bit, mmi, manifest, firmware)
    }:
        raise ValueError("output must be a new file, not an input or existing file")
    # Temp directory is private and removed even on error: the converted MEM
    # contains the user's firmware and is not a redistributable build artifact.
    with tempfile.TemporaryDirectory(prefix="macqd700-adb-") as temp:
        work = Path(temp)
        mem = work / "firmware.mem"
        mem.write_text(contents, encoding="ascii")
        result = work / "patched.bit"
        subprocess.run([
            updatemem, "-meminfo", str(mmi.resolve()), "-data", str(mem),
            "-bit", str(bit.resolve()), "-proc", "adb_pic", "-out", str(result),
        ], cwd=work, check=True)
        if not result.is_file() or result.stat().st_size == 0:
            raise ValueError("UpdateMEM did not produce a bitstream")
        with output.open("xb") as out:
            out.write(result.read_bytes())
    print(f"Created {output} (contains your firmware; do not redistribute)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("seal", "patch"):
        command = commands.add_parser(name)
        for arg in ("bit", "mmi", "manifest"):
            command.add_argument(f"--{arg}", type=Path, required=True)
        if name == "patch":
            command.add_argument("--firmware", type=Path, required=True)
            command.add_argument("--output", type=Path, required=True)
            command.add_argument("--updatemem", default="updatemem")
    args = vars(parser.parse_args())
    command = args.pop("command")
    try:
        (seal if command == "seal" else patch)(**args)
    except (OSError, ValueError, ET.ParseError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"ADB bitstream: {error}\n")


if __name__ == "__main__":
    main()
