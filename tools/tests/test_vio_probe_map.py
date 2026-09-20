"""Keep the compact VIO IP, RTL, lint stub and dashboard in lockstep."""
import re
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class VioProbeMapTest(unittest.TestCase):
    def test_widths_and_ports(self):
        synth = (ROOT / "synth/vivado.tcl").read_text()
        widths = list(map(int, re.search(r"set widths \{([\d ]+)\}", synth)[1].split()))
        self.assertEqual(len(widths), 15)
        self.assertEqual(sum(widths), 843)
        stub = (ROOT / "tb/verilator_xilinx_stubs.v").read_text().split("module debug_vio", 1)[1].split("endmodule", 1)[0]
        ports = re.findall(r"input wire \[(\d+):0\] probe_in(\d+)", stub)
        self.assertEqual([(int(i), int(w) + 1) for w, i in ports], list(enumerate(widths)))
        rtl = (ROOT / "rtl/soc/fpga_top_debug_vio.vh").read_text()
        inputs = re.findall(r"\.probe_in(\d+)\((\w+)\)", rtl)
        self.assertEqual([int(i) for i, _ in inputs], list(range(15)))
        dashboard = (ROOT / "synth/vio_dashboard.tcl").read_text()
        for (_, name), width in zip(inputs, widths):
            self.assertIn(f"{width} bits  {name}", dashboard)
        self.assertIn("output wire [4:0] probe_out0", stub)
        self.assertIn(".probe_out0(vio_boot_ctrl)", rtl)
        self.assertIn(".probe_out1(vio_hard_reset)", rtl)

    def test_no_activity_and_exact_cache_key(self):
        synth = (ROOT / "synth/vivado.tcl").read_text()
        self.assertIn("CONFIG.C_EN_PROBE_IN_ACTIVITY 0", synth)
        self.assertIn("set signature [list probe_map=v27 part=$part config=$config]", synth)
        self.assertIn('[string trim [read $fh]] eq $signature', synth)
        self.assertIn("CONFIG.C_PROBE_IN${n}_WIDTH $width", synth)

    def test_removed_probes_are_not_connected(self):
        rtl = (ROOT / "rtl/soc/fpga_top_debug_vio.vh").read_text()
        instance = rtl.split("debug_vio u_dbg_vio", 1)[1].split(");", 1)[0]
        for name in ("video_debug_hcount", "video_debug_vcount", "video_debug_rgb",
                     "vram_rd_addr", "ddr_dbg_r_cnt", "dbg_committed",
                     "vio_hdmi_ctrl", "vio_vram_read", "vio_dafb_cfg", "vio_adb_dbg"):
            self.assertNotIn(name, instance)

    def test_tcl_syntax(self):
        for name in ("synth/vivado.tcl", "synth/vio_dashboard.tcl"):
            script = 'set f [open {' + str(ROOT / name) + '}]; set s [read $f]; close $f; if {![info complete $s]} {exit 1}'
            subprocess.run(["tclsh"], input=script, text=True, check=True)


if __name__ == "__main__":
    unittest.main()
