#!/usr/bin/env python3
from __future__ import annotations

import contextlib
import io
import tempfile
import subprocess
import unittest
from pathlib import Path
from unittest import mock

from tools import jtag_bringup_tui as tui


SAMPLE_MACHINE = """\
noise before
JTAG_SNAPSHOT_BEGIN
probe_in0=0x1
probe_in1=0x02A
probe_in2=0x00B
probe_in3=0x12345
probe_in4=0xABCDEF
probe_in5=0x4000002A
probe_in6=0x0012
probe_in7=0x2E
probe_in8=0x1
probe_in9=0x00000010
probe_in10=0x1C
probe_in11=0x3
probe_in12=0x3FF
probe_in13=0x3E
probe_in14=0x001122330044556600000030
probe_in15=0x00D6
probe_out0=0x5
hdmi_test_pattern=0x1
vram_write_count=0x4
dafb_write_count=0x2
fb_reader_req_count=0x10
fb_reader_rsp_count=0x8
fb_reader_miss_count=0x2
JTAG_SNAPSHOT_END
"""

SAMPLE_NAMED_HW = """\
WARNING: [Labtools 27-3410] Calibration Failed.
JTAG_SNAPSHOT_BEGIN
hdmi_mmcm_locked=0x1
video_debug_hcount=0x123
video_debug_vcount=0x045
video_debug_rgb=0xffffff
dbg_pc=0x4000002a
ddr_dbg_r_cnt=0x0000
vio_rst_bundle=0x32
s0_awvalid=0x0
s0_awready=0x0
s0_wvalid=0x0
s0_wready=0x0
s0_bvalid=0x0
s0_bready=0x0
s0_arvalid=0x0
s0_arready=0x0
s0_rvalid=0x0
s0_rready=0x0
boot_rom_loading=0x0
boot_error=0x0
probe_in14=0x001122330044556600000030
probe_in15=0x00D6
probe_out0=0x0
hdmi_test_pattern=0x1
JTAG_SNAPSHOT_END
"""

SAMPLE_SPLIT_FB = """\
JTAG_SNAPSHOT_BEGIN
probe_in0=0x1
probe_in7=0x2F
probe_in10=0x1D
probe_in11=0x3
boot_rom_loading=0x0
boot_error=0x0
al9134_int=0x1
boot_fsm_rst=0x0
cpu_rst=0x0
hdmi_test_pattern=0x0
fb_reader_req_count[0]=0x1
fb_reader_req_count[1]=0x1
fb_reader_req_count[2]=0x0
fb_reader_req_count[3]=0x0
fb_reader_req_count[4]=0x0
fb_reader_req_count[5]=0x0
fb_reader_req_count[6]=0x0
fb_reader_req_count[7]=0x0
fb_reader_req_count[8]=0x0
fb_reader_req_count[9]=0x0
fb_reader_req_count[10]=0x0
fb_reader_req_count[11]=0x0
fb_reader_req_count[12]=0x0
fb_reader_req_count[13]=0x0
fb_reader_req_count[14]=0x0
fb_reader_req_count[15]=0x0
fb_reader_rsp_count[0]=0x0
fb_reader_rsp_count[1]=0x1
fb_reader_rsp_count[2]=0x0
fb_reader_rsp_count[3]=0x0
fb_reader_rsp_count[4]=0x0
fb_reader_rsp_count[5]=0x0
fb_reader_rsp_count[6]=0x0
fb_reader_rsp_count[7]=0x0
fb_reader_rsp_count[8]=0x0
fb_reader_rsp_count[9]=0x0
fb_reader_rsp_count[10]=0x0
fb_reader_rsp_count[11]=0x0
fb_reader_rsp_count[12]=0x0
fb_reader_rsp_count[13]=0x0
fb_reader_rsp_count[14]=0x0
fb_reader_rsp_count[15]=0x0
fb_reader_miss_count[0]=0x1
fb_reader_miss_count[1]=0x0
fb_reader_miss_count[2]=0x0
fb_reader_miss_count[3]=0x0
fb_reader_miss_count[4]=0x0
fb_reader_miss_count[5]=0x0
fb_reader_miss_count[6]=0x0
fb_reader_miss_count[7]=0x0
fb_reader_miss_count[8]=0x0
fb_reader_miss_count[9]=0x0
fb_reader_miss_count[10]=0x0
fb_reader_miss_count[11]=0x0
fb_reader_miss_count[12]=0x0
fb_reader_miss_count[13]=0x0
fb_reader_miss_count[14]=0x0
fb_reader_miss_count[15]=0x0
probe_out0=0x0
JTAG_SNAPSHOT_END
"""


class JtagBringupTuiTests(unittest.TestCase):
    def test_parse_machine_snapshot(self) -> None:
        probes = tui.parse_snapshot(SAMPLE_MACHINE)
        self.assertEqual(probes["probe_in5"], 0x4000002A)
        self.assertEqual(probes["probe_in13"], 0x3E)
        self.assertEqual(probes["probe_in14"], 0x001122330044556600000030)
        self.assertEqual(probes["probe_in15"], 0x00D6)
        self.assertEqual(probes["probe_out0"], 0x5)

    def test_parse_legacy_dashboard_snapshot(self) -> None:
        text = """\
=== VIO snapshot ===
probe_in0  0x1
probe_in5  0x4000002A
probe_in13 0x0000003E
probe_out0=0x3 bypass_sd=1 release_cpu=1 force_cpu_rst=0
"""
        probes = tui.parse_snapshot(text)
        self.assertEqual(probes["probe_in0"], 1)
        self.assertEqual(probes["probe_in5"], 0x4000002A)
        self.assertEqual(probes["probe_in13"], 0x3E)
        self.assertEqual(probes["probe_out0"], 3)

    def test_format_decodes_key_fields(self) -> None:
        out = tui.format_snapshot(tui.parse_snapshot(SAMPLE_MACHINE), source="test")
        self.assertIn("ddr_cal_done=1", out)
        self.assertIn("boot_rom_ready=1", out)
        self.assertIn("pc=0x4000002A", out)
        self.assertIn("vram=4 dafb=2", out)
        self.assertIn("mode=test-pattern", out)
        self.assertIn("fb_reader: req=16 rsp=8 stall=2", out)
        self.assertIn("dafb: base=0x00112233 stride=0x00445566 bpp=0x00000030", out)
        self.assertIn("axi_error: seen=1 sticky=1 src=boot resp=SLVERR", out)
        self.assertIn("bypass_sd=1 release_cpu=0 scc_uart_sel_b=1 full_dbg_rst=0", out)

    def test_named_probe_snapshot_highlights_mig_blocking_cpu(self) -> None:
        probes = tui.parse_snapshot(SAMPLE_NAMED_HW)
        self.assertEqual(probes["probe_in0"], 1)
        self.assertEqual(probes["probe_in5"], 0x4000002A)
        self.assertEqual(probes["probe_in7"], 0x32)
        self.assertEqual(probes["probe_in12"], 0x000)
        self.assertEqual(probes["probe_in14"], 0x001122330044556600000030)
        out = tui.format_snapshot(
            probes,
            source="named",
            warnings=tui.extract_labtools_warnings(SAMPLE_NAMED_HW),
        )
        self.assertIn("Vivado reported: WARNING: [Labtools 27-3410] Calibration Failed.", out)
        self.assertIn("HDMI is alive, but DDR/MIG calibration is not done", out)
        self.assertIn("reset/init is still blocking CPU release", out)
        self.assertIn("ddr_cal_done=0", out)
        self.assertIn("mode=test-pattern", out)

    def test_split_fb_reader_bits_are_synthesized(self) -> None:
        probes = tui.parse_snapshot(SAMPLE_SPLIT_FB)
        self.assertEqual(probes["vram_rd_en"], 1)
        self.assertEqual(probes["vram_rd_valid"], 1)
        self.assertEqual(probes["probe_in13"], 0x3F)
        self.assertEqual(probes["fb_reader_req_count"], 3)
        self.assertEqual(probes["fb_reader_rsp_count"], 2)
        self.assertEqual(probes["fb_reader_miss_count"], 1)
        out = tui.format_snapshot(probes, source="split")
        self.assertIn("boot_video: loading=0 error=0 mmcm=1 i2c=1 de=1 rd_en=1 rd_valid=1", out)
        self.assertIn("mode=framebuffer", out)
        self.assertIn("fb_reader: req=3 rsp=2 stall=1", out)

    def test_status_mock_does_not_run_vivado(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "snapshot.txt"
            path.write_text(SAMPLE_MACHINE)
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                rc = tui.main(["status", "--mock", str(path)])
        self.assertEqual(rc, 0)
        self.assertIn("m68k-ooo JTAG bring-up dashboard", buf.getvalue())

    def test_mutating_command_defaults_to_dry_run(self) -> None:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = tui.main(["axi-write", "0x40000000", "0x4ef9002a"])
        self.assertEqual(rc, 0)
        self.assertIn("dry-run: pass --go", buf.getvalue())
        self.assertIn("axi-write 0x40000000 0x4EF9002A", buf.getvalue())

    def test_debug_halt_after_defaults_to_dry_run(self) -> None:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = tui.main(["debug-halt-after", "8192"])
        self.assertEqual(rc, 0)
        self.assertIn("dry-run: pass --go", buf.getvalue())
        self.assertIn("debug-halt-after 8192", buf.getvalue())

    def test_debug_halt_exc_defaults_to_illegal_vector(self) -> None:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = tui.main(["debug-halt-exc"])
        self.assertEqual(rc, 0)
        self.assertIn("dry-run: pass --go", buf.getvalue())
        self.assertIn("debug-halt-exc 4", buf.getvalue())

    def test_openocd_breakpoint_uses_stage5_enable_register(self) -> None:
        captured: dict[str, object] = {}

        def fake_run(cmd: list[str], *, env=None, capture=False, dry_run=False):
            captured["cmd"] = cmd
            return subprocess.CompletedProcess(cmd, 0, "", "")

        with mock.patch.object(tui, "run", side_effect=fake_run):
            rc = tui.main(["--backend", "openocd", "--openocd-cfg", "board.cfg", "debug-break-pc",
                           "0x40001234", "--go"])
        self.assertEqual(rc, 0)
        cmd = captured["cmd"]
        self.assertIn("write_memory 0x00000038 32 [list 0x40001234] phys", cmd)
        self.assertIn("write_memory 0x00000090 32 [list 0x00000001] phys", cmd)
        self.assertIn("write_memory 0x0000003C 32 [list 0x00000004] phys", cmd)

    def test_openocd_exception_uses_stage5_vector_mask(self) -> None:
        captured: dict[str, object] = {}

        def fake_run(cmd: list[str], *, env=None, capture=False, dry_run=False):
            captured["cmd"] = cmd
            return subprocess.CompletedProcess(cmd, 0, "", "")

        with mock.patch.object(tui, "run", side_effect=fake_run):
            rc = tui.main(["--backend", "openocd", "--openocd-cfg", "board.cfg", "debug-halt-exc",
                           "33", "--go"])
        self.assertEqual(rc, 0)
        cmd = captured["cmd"]
        self.assertIn("write_memory 0x00000060 32 [list 0x00000000] phys", cmd)
        self.assertIn("write_memory 0x00000064 32 [list 0x00000002] phys", cmd)
        self.assertIn("write_memory 0x0000007C 32 [list 0x00000000] phys", cmd)
        self.assertNotIn("write_memory 0x00000050", cmd)

    def test_debug_reset_halt_defaults_to_dry_run(self) -> None:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = tui.main(["debug-reset-halt"])
        self.assertEqual(rc, 0)
        self.assertIn("dry-run: pass --go", buf.getvalue())
        self.assertIn("debug-reset-halt", buf.getvalue())

    def test_debug_run_from_reset_halt_after_defaults_to_dry_run(self) -> None:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = tui.main(["debug-run-from-reset-halt-after", "1000", "25"])
        self.assertEqual(rc, 0)
        self.assertIn("dry-run: pass --go", buf.getvalue())
        self.assertIn("debug-run-from-reset-halt-after 1000 25", buf.getvalue())

    def test_debug_step_parses_halt_first_flag(self) -> None:
        args = tui.build_parser().parse_args(["debug-step", "--halt-first"])
        self.assertEqual(args.command, "debug-step")
        self.assertTrue(args.halt_first)

    def test_debug_step_emits_expected_tclargs(self) -> None:
        captured: dict[str, object] = {}

        def fake_run(cmd: list[str], *, env=None, capture=False, dry_run=False):
            captured["cmd"] = cmd
            captured["env"] = env
            captured["capture"] = capture
            captured["dry_run"] = dry_run
            return subprocess.CompletedProcess(cmd, 0, "", "")

        with mock.patch.object(tui, "run", side_effect=fake_run):
            rc = tui.main(["--vivado", "vivado-bin", "debug-step", "--halt-first", "--go"])

        self.assertEqual(rc, 0)
        self.assertEqual(
            captured["cmd"],
            [
                "vivado-bin",
                "-nojournal",
                "-nolog",
                "-mode",
                "batch",
                "-source",
                str(tui.JTAG_TCL),
                "-tclargs",
                "debug-step",
                "--halt-first",
            ],
        )
        self.assertFalse(captured["dry_run"])

    def test_debug_sweep_reset_halt_after_emits_expected_tclargs(self) -> None:
        captured: dict[str, object] = {}

        def fake_run(cmd: list[str], *, env=None, capture=False, dry_run=False):
            captured["cmd"] = cmd
            captured["env"] = env
            captured["capture"] = capture
            captured["dry_run"] = dry_run
            return subprocess.CompletedProcess(cmd, 0, "", "")

        with mock.patch.object(tui, "run", side_effect=fake_run):
            rc = tui.main([
                "--vivado", "vivado-bin",
                "debug-sweep-reset-halt-after", "10", "1", "2", "4", "--go",
            ])

        self.assertEqual(rc, 0)
        self.assertEqual(
            captured["cmd"],
            [
                "vivado-bin",
                "-nojournal",
                "-nolog",
                "-mode",
                "batch",
                "-source",
                str(tui.JTAG_TCL),
                "-tclargs",
                "debug-sweep-reset-halt-after",
                "10",
                "1",
                "2",
                "4",
            ],
        )
        self.assertFalse(captured["dry_run"])

    def test_backend_flag_parses_openocd_config(self) -> None:
        args = tui.build_parser().parse_args([
            "--backend", "openocd",
            "--openocd-bin", "openocd-bin",
            "--openocd-cfg", "board.cfg",
            "debug-halt-status",
        ])
        self.assertEqual(args.backend, "openocd")
        self.assertEqual(args.openocd_bin, "openocd-bin")
        self.assertEqual(args.openocd_cfg, ["board.cfg"])

    def test_openocd_axi_write_emits_openocd_batch(self) -> None:
        captured: dict[str, object] = {}

        def fake_run(cmd: list[str], *, env=None, capture=False, dry_run=False):
            captured["cmd"] = cmd
            captured["capture"] = capture
            captured["dry_run"] = dry_run
            return subprocess.CompletedProcess(cmd, 0, "", "")

        with mock.patch.object(tui, "run", side_effect=fake_run):
            rc = tui.main([
                "--backend", "openocd",
                "--openocd-bin", "openocd-bin",
                "--openocd-cfg", "board.cfg",
                "axi-write", "0x40000000", "0x4ef9002a", "--go",
            ])

        self.assertEqual(rc, 0)
        cmd = captured["cmd"]
        self.assertEqual(cmd[0], "openocd-bin")
        self.assertIn("-f", cmd)
        self.assertIn("board.cfg", cmd)
        self.assertIn("init", cmd)
        self.assertIn("shutdown", cmd)
        self.assertIn("write_memory 0x40000000 32 [list 0x4EF9002A] phys", cmd)
        self.assertFalse(captured["dry_run"])

    def test_openocd_debug_halt_status_parses_machine_output(self) -> None:
        captured: dict[str, object] = {}

        def fake_run(cmd: list[str], *, env=None, capture=False, dry_run=False):
            captured["cmd"] = cmd
            captured["capture"] = capture
            captured["dry_run"] = dry_run
            stdout = "\n".join([
                "OpenOCD startup noise",
                "DBG_VERSION=0xDEB60004",
                "DBG_STATUS=0x0000000C",
                "DBG_PC=0x4000002A",
                "DBG_CYCLE_LO=0x00000010",
                "DBG_CYCLE_HI=0x00000000",
                "DBG_INST_LO=0x00000008",
                "DBG_INST_HI=0x00000000",
                "DBG_HALT_CTL=0x00000005",
                "DBG_HALT_REASON=0x00000000",
                "DBG_HALT_HIT_PC=0x4000002A",
                "DBG_HALT_HIT_INST_LO=0x00000008",
                "DBG_HALT_HIT_INST_HI=0x00000000",
                "DBG_HALT_EXC_VEC=0x00000004",
            ])
            return subprocess.CompletedProcess(cmd, 0, stdout, "")

        buf = io.StringIO()
        with mock.patch.object(tui, "run", side_effect=fake_run):
            with contextlib.redirect_stdout(buf):
                rc = tui.main([
                    "--backend", "openocd",
                    "--openocd-bin", "openocd-bin",
                    "--openocd-cfg", "board.cfg",
                    "debug-halt-status",
                ])

        self.assertEqual(rc, 0)
        self.assertEqual(captured["cmd"][0], "openocd-bin")
        self.assertIn("OpenOCD debug snapshot", buf.getvalue())
        self.assertIn("DBG_STATUS   : 0x0000000c", buf.getvalue())
        self.assertIn("DBG_HALT_CTL : 0x00000005", buf.getvalue())

    def test_openocd_debug_step_halt_first_preserves_halt_on_step_write(self) -> None:
        captured: dict[str, object] = {}

        def fake_run(cmd: list[str], *, env=None, capture=False, dry_run=False):
            captured["cmd"] = cmd
            captured["capture"] = capture
            captured["dry_run"] = dry_run
            return subprocess.CompletedProcess(cmd, 0, "", "")

        with mock.patch.object(tui, "run", side_effect=fake_run):
            rc = tui.main([
                "--backend", "openocd",
                "--openocd-bin", "openocd-bin",
                "--openocd-cfg", "board.cfg",
                "debug-step", "--halt-first", "--go",
            ])

        self.assertEqual(rc, 0)
        cmd = captured["cmd"]
        self.assertIn("write_memory 0x00000008 32 [list 0x00000001] phys", cmd)
        self.assertIn("write_memory 0x00000008 32 [list 0x00000003] phys", cmd)


if __name__ == "__main__":
    unittest.main()
