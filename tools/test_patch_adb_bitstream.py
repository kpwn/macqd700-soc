#!/usr/bin/env python3
"""Synthetic-only tests. No Apple bytes or FPGA tool installation needed."""
import json
from pathlib import Path
import struct
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch as mock_patch

from patch_adb_bitstream import memory_text, patch, seal, validate_mmi, verify_bundle

MMI = '''<MemInfo Version="1" Minor="0">
<Processor Endianness="Little" InstPath="adb_pic">
<AddressSpace Name="program" Begin="0" End="4095"><BusBlock>
<BitLane MemType="RAMB32" Placement="X0Y1">
<DataWidth MSB="31" LSB="0"/><AddressRange Begin="0" End="1023"/>
<Parity ON="false" NumBits="0"/></BitLane>
</BusBlock></AddressSpace></Processor>
<Config><Option Name="Part" Val="xcku5p-ffvb676-2-i"/></Config></MemInfo>'''


class PatchTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.bit, self.mmi, self.manifest, self.firmware, self.output = (
            root / name for name in ("base.bit", "base.mmi", "base.json", "fw.bin", "result.bit")
        )
        self.bit.write_bytes(b"synthetic blank bitstream fixture")
        self.mmi.write_text(MMI)
        self.firmware.write_bytes(struct.pack("<512H", *[(i * 17) & 0xfff for i in range(512)]))
        seal(self.bit, self.mmi, self.manifest)

    def invoke(self):
        patch(self.bit, self.mmi, self.manifest, self.firmware, self.output, "test-updatemem")

    def test_word_layout(self):
        words = memory_text(self.firmware.read_bytes()).splitlines()
        self.assertEqual(words[:4], ["@00000000", "00000000", "11000000", "22000000"])
        self.assertEqual(len(words), 1025)
        self.assertTrue(all(word == "00000000" for word in words[513:]))

    def test_firmware_validation(self):
        for data in (b"", bytes(1023), bytes(1025), struct.pack("<H", 0x1000) + bytes(1022)):
            with self.assertRaises(ValueError):
                memory_text(data)

    def test_bundle_hashes(self):
        verify_bundle(self.bit, self.mmi, self.manifest)
        self.bit.write_bytes(b"another build")
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            self.invoke()

    def test_mmi_hash(self):
        self.mmi.write_text(MMI.replace("X0Y1", "X0Y2"))
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            self.invoke()

    def test_reject_wrong_layout(self):
        for before, after in (("RAMB32", "RAMB18"), ('MSB="31"', 'MSB="15"'),
                              ('End="4095"', 'End="2047"'), ("Little", "Big"),
                              ("X0Y1", "UNPLACED"), ('ON="false"', 'ON="true"')):
            self.mmi.write_text(MMI.replace(before, after))
            with self.subTest(after=after), self.assertRaises(ValueError):
                validate_mmi(self.mmi)

    def test_reject_unknown_manifest(self):
        data = json.loads(self.manifest.read_text())
        data["format"] = "not-our-layout"
        self.manifest.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, "unsupported"):
            self.invoke()

    def test_no_overwrite(self):
        self.output.write_bytes(b"keep me")
        with self.assertRaisesRegex(ValueError, "new file"):
            self.invoke()
        self.assertEqual(self.output.read_bytes(), b"keep me")

    def test_success_and_private_temporary_cleanup(self):
        seen = []
        def updater(command, cwd, check):
            self.assertEqual(command[0], "test-updatemem")
            self.assertTrue(check)
            mem = Path(command[command.index("-data") + 1])
            self.assertEqual(mem.read_text(), memory_text(self.firmware.read_bytes()))
            self.assertEqual(command[command.index("-proc") + 1], "adb_pic")
            seen.append(mem)
            Path(command[command.index("-out") + 1]).write_bytes(b"patched synthetic image")
        with mock_patch("patch_adb_bitstream.subprocess.run", side_effect=updater):
            self.invoke()
        self.assertEqual(self.output.read_bytes(), b"patched synthetic image")
        self.assertFalse(seen[0].exists())

    def test_missing_tool_leaves_no_output(self):
        with mock_patch("patch_adb_bitstream.subprocess.run", side_effect=FileNotFoundError):
            with self.assertRaises(FileNotFoundError):
                self.invoke()
        self.assertFalse(self.output.exists())

    @unittest.skipUnless(shutil.which("tclsh"), "Tcl interpreter unavailable")
    def test_preflight_rejects_old_or_nonblank_checkpoints(self):
        source = Path(__file__).resolve().parents[1] / "synth/adb_firmware_mmi.tcl"
        for fault in ("none", "old_lutrom", "data", "parity", "placement", "width"):
            script = r'''
set fault %s
proc get_cells {args} {
    if {$::fault eq "old_lutrom"} { return {} }
    return u_program_rom/u_firmware_bram
}
proc get_property {name object} {
    if {$name eq "INIT_00" && $::fault eq "data"} { return 256'h1 }
    if {$name eq "INITP_0F" && $::fault eq "parity"} { return 256'h1 }
    if {[string match INIT* $name]} { return 256'h0 }
    if {$name eq "LOC" && $::fault eq "placement"} { return UNPLACED }
    if {$name eq "READ_WIDTH_A" && $::fault eq "width"} { return 18 }
    return [dict get {READ_WIDTH_A 36 DOA_REG 0 EN_ECC_READ FALSE
        EN_ECC_WRITE FALSE IS_CLKARDCLK_INVERTED 1'b1 LOC RAMB36_X0Y1} $name]
}
source {%s}
if {[catch {validate_adb_firmware_bram} reason]} { puts REJECTED } else { puts ACCEPTED }
''' % (fault, source)
            result = subprocess.run(["tclsh"], input=script, text=True, capture_output=True, check=True)
            with self.subTest(fault=fault):
                self.assertEqual(result.stdout.strip(), "ACCEPTED" if fault == "none" else "REJECTED")

    def test_every_production_writer_preflights_before_writing_blank(self):
        root = Path(__file__).resolve().parents[1] / "synth"
        for name in ("vivado.tcl", "impl_from_dcp.tcl", "bitstream_from_route.tcl",
                     "route_from_place.tcl", "close_timing_from_route.tcl",
                     "repair_min_skew_from_route.tcl"):
            contents = (root / name).read_text()
            with self.subTest(flow=name):
                self.assertLess(contents.index("validate_adb_firmware_bram"),
                                contents.index("write_bitstream -force $output_dir/fpga_top.blank.bit"))


if __name__ == "__main__":
    unittest.main()
