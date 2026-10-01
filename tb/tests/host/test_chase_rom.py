"""Synthetic ROM image and retired-PC contract checks; no Verilator required."""

import importlib.util
import json
import struct
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "synth_boot" / "chase_rom.py"
spec = importlib.util.spec_from_file_location("chase_rom", SCRIPT)
chase = importlib.util.module_from_spec(spec)
spec.loader.exec_module(chase)


def trace_lines(meta):
    s = meta["symbols"]
    pcs = []
    for hops in (meta["warm_hops"], meta["measured_hops"]):
        pcs.append(s["marker_start"])
        for _ in range(hops // 4):
            pcs.extend(s[f"load_{i}"] for i in range(4))
        pcs.append(s["marker_end"])
    pcs.append(s["pass_pc"])
    return [f"{index} {2 * index} 0x{pc:08x}" for index, pc in enumerate(pcs)]


class ChaseRomTest(unittest.TestCase):
    def test_images_have_one_complete_ring_and_expected_checksum(self):
        with tempfile.TemporaryDirectory() as name:
            for size in chase.SIZES:
                paths = chase.generate(size, Path(name))
                meta = json.loads(paths["json"].read_text())
                image = paths["bin"].read_bytes()
                links = chase.ring(size)
                self.assertEqual(meta["checksum"], chase.checksum(links))
                for index, target in enumerate(links):
                    pointer = struct.unpack_from(">I", image, 0x10000 + index * 64)[0]
                    self.assertEqual(pointer, chase.RING_BASE + target * 64)
                self.assertEqual(struct.unpack_from(">I", image, 4)[0], meta["symbols"]["start"])

    def test_parser_rejects_missing_or_corrupted_window(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            paths = chase.generate(2048, root)
            meta = json.loads(paths["json"].read_text())
            good = trace_lines(meta)
            trace = root / "trace.txt"

            def parse(lines):
                trace.write_text("\n".join(lines) + "\n")
                return chase.parse_trace(meta, trace)

            self.assertEqual(parse(good)["hops"], 128)
            s = meta["symbols"]
            bad = good[:-1]
            with self.assertRaisesRegex(ValueError, "pass"):
                parse(bad)
            bad = good + [f"9999 9999 0x{s['fail_pc']:08x}"]
            with self.assertRaisesRegex(ValueError, "fail"):
                parse(bad)
            bad = [line for line in good if not line.endswith(f"0x{s['marker_end']:08x}")]
            with self.assertRaisesRegex(ValueError, "two start and two end"):
                parse(bad)
            second_start = [i for i, line in enumerate(good) if line.endswith(f"0x{s['marker_start']:08x}")][1]
            bad = good[:second_start + 1] + good[second_start + 2:]
            with self.assertRaisesRegex(ValueError, "load PC counts"):
                parse(bad)
            bad = good.copy()
            end = [i for i, line in enumerate(good) if line.endswith(f"0x{s['marker_end']:08x}")][1]
            fields = bad[end].split()
            fields[1] = good[second_start].split()[1]
            bad[end] = " ".join(fields)
            with self.assertRaisesRegex(ValueError, "order regressed|empty or inverted"):
                parse(bad)


if __name__ == "__main__":
    unittest.main()
