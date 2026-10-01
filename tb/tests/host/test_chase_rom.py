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

    def test_bus_stalls_fifo_histograms_and_rejections(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            paths = chase.generate(65536, root)
            meta = json.loads(paths["json"].read_text())
            trace = root / "retire.txt"
            trace.write_text("\n".join(trace_lines(meta)) + "\n")
            timing = chase.parse_trace(meta, trace)
            warm = timing["warm_start_tick"]
            measured = timing["start_tick"]
            bus_path = root / "bus.txt"

            def parse_bus(lines):
                bus_path.write_text("\n".join(lines) + "\n")
                return chase.parse_bus_trace(bus_path, timing)

            good = [
                f"{warm + 2} AR 1 0x40010000 0 2 1",
                f"{warm + 4} DDR_AR 0x40010000 0",
                f"{warm + 6} RV 1 0 1 1",
                f"{measured + 2} AR 3 0x40010040 1 2 1",
                f"{measured + 4} AR 3 0x40010080 0 2 1",  # same-ID FIFO
                f"{measured + 4} RV 3 0 0 0",             # repeated stall
                f"{measured + 6} RV 3 0 0 0",
                f"{measured + 8} RV 3 0 0 1",
                f"{measured + 10} RV 3 0 1 0",
                f"{measured + 12} RV 3 0 1 1",
                f"{measured + 14} RV 3 0 1 1",
            ]
            result = parse_bus(good)
            self.assertEqual(result["windows"]["measured"]["hot_ar"], 2)
            self.assertEqual(result["windows"]["measured"]["hot_r_transfers"], 3)
            self.assertEqual(result["windows"]["measured"]["hot_r_valid_samples"], 6)
            self.assertEqual(result["measured_ar_to_first_rvalid_cycles"], {1: 1, 5: 1})
            self.assertEqual(result["measured_ar_to_first_transfer_cycles"], {3: 1, 5: 1})
            with self.assertRaisesRegex(ValueError, "pointer request sequence"):
                chase.validate_acceptance(meta, result, "warmL2")
            reordered = [
                f"{measured + 2} AR 1 0x40010000 0 2 1",
                f"{measured + 4} AR 2 0x40010040 0 2 1",
                f"{measured + 6} RV 2 0 1 1",
                f"{measured + 8} RV 1 0 1 1",
            ]
            self.assertEqual(parse_bus(reordered)["windows"]["measured"]["hot_r_transfers"], 2)
            with self.assertRaisesRegex(ValueError, "warmL2 acceptance failed"):
                chase.validate_acceptance({**meta, "size_bytes": 2048}, result, "warmL2")
            with self.assertRaisesRegex(ValueError, "warmL2 acceptance failed"):
                chase.validate_acceptance(meta, {"windows": {**result["windows"],
                    "measured": {**result["windows"]["measured"], "ddr_ar": 1}}}, "warmL2")

            for bad, reason in (
                ([f"{warm + 2} RV 9 0 1 1"], "orphan"),
                (good[:5] + [f"{measured + 4} RV 3 2 0 1"] + good[6:], "RRESP"),
                (good[:5] + [f"{measured + 4} RV 3 0 1 1"] + good[6:], "RLAST"),
                (good[:-1], "incomplete"),
            ):
                with self.assertRaisesRegex(ValueError, reason):
                    parse_bus(bad)

    def test_warm_l2_requires_every_pointer_request_in_order(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            paths = chase.generate(65536, root)
            meta = json.loads(paths["json"].read_text())
            timing = {"warm_start_tick": 0, "warm_end_tick": 20,
                      "start_tick": 100, "end_tick": 100 + 16 * meta["measured_hops"]}
            path = root / "bus.txt"
            links = chase.ring(meta["size_bytes"])
            addresses = []
            node = 0
            for _ in range(meta["measured_hops"]):
                addresses.append(chase.RING_BASE + 64 * node)
                node = links[node]

            def accept(requests, size=4):
                lines = [f"2 AR 0 0x{chase.RING_BASE:x} 0 4 1", "14 RV 0 0 1 1"]
                for i, addr in enumerate(requests):
                    tick = 102 + i * 16
                    lines.extend((f"{tick} AR 0 0x{addr:x} 0 {size} 1",
                                  f"{tick + 12} RV 0 0 1 1"))
                path.write_text("\n".join(lines) + "\n")
                bus = chase.parse_bus_trace(path, timing)
                chase.validate_acceptance(meta, bus, "warmL2")
                return bus

            self.assertEqual(accept(addresses)["measured_ar_to_first_transfer_cycles"],
                             {6: meta["measured_hops"]})
            reordered = addresses.copy()
            reordered[10], reordered[11] = reordered[11], reordered[10]
            # Counts and terminal pointer alone cannot detect reordered accesses
            # or a short cycle repeated to fill the expected measurement window.
            for bad in (reordered, addresses[:32] * (len(addresses) // 32), addresses[:-1]):
                with self.assertRaisesRegex(ValueError, "pointer request sequence"):
                    accept(bad)
            with self.assertRaisesRegex(ValueError, "line-read shape"):
                accept(addresses, size=2)


if __name__ == "__main__":
    unittest.main()
