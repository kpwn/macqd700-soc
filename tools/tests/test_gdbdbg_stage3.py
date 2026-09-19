import unittest

from tools import gdbdbg


class RecordingTransport:
    def __init__(self, fail_apply=False, regs=None):
        self.commands = []
        self.fail_apply = fail_apply
        self.regs = regs

    def execute(self, line, wait_s=0.05):
        self.commands.append(line)
        if line == "arch-apply" and self.fail_apply:
            return ["> ERROR apply rejected"]
        if line == "regs" and self.regs is not None:
            return [f"> {name} = 0x{value:08X}" for name, value in self.regs.items()] + ["> READY"]
        return ["> READY"]


class GdbStage3Test(unittest.TestCase):
    def test_staged_register_then_step_is_exact(self):
        transport = RecordingTransport()
        target = gdbdbg.Target(transport)
        target.stage_reg("CACR", 0x80008000)
        target.stage_reg("PC", 0x12345678)
        target._await_halt = lambda *args, **kwargs: None

        self.assertTrue(target.step())
        self.assertEqual(
            transport.commands,
            [
                "arch-write CACR 0x80008000",
                "arch-write PC 0x12345678",
                "arch-apply",
                "step",
            ],
        )
        self.assertEqual(target.pending_regs(), {})

    def test_failed_apply_keeps_pending_registers(self):
        transport = RecordingTransport(fail_apply=True)
        target = gdbdbg.Target(transport)
        target.stage_reg("D0", 0xDEADBEEF)
        with self.assertRaises(gdbdbg.TargetError):
            target.flush_regs()
        self.assertEqual(target.pending_regs(), {"D0": 0xDEADBEEF})

    def test_complete_stage3_shadow_set_is_writable(self):
        for name in (
            "D7", "A7", "USP", "MSP", "ISP", "SR", "VBR", "CACR", "TC",
            "ITT0", "ITT1", "DTT0", "DTT1", "URP", "SRP", "PC", "SFC", "DFC",
        ):
            self.assertIn(name, gdbdbg.Target.WRITABLE_REGS)

    def test_register_dump_is_single_source_and_honors_mmu_filter(self):
        regs = {**{f"D{i}": i for i in range(8)},
                **{f"A{i}": 0x10 + i for i in range(8)},
                "SR": 0x2700, "VBR": 0, "PC": 0x1000,
                "MSP": 0x2000, "CACR": 0x80000000,
                "TC": 0x8000, "URP": 0x1234, "MMUSR": 0x55}
        transport = RecordingTransport(regs=regs)
        target = gdbdbg.Target(transport)
        target.is_halted = lambda: True
        got = target.read_regs(include_mmu=False)
        self.assertEqual(transport.commands, ["regs"])
        self.assertEqual(got["MSP"], 0x2000)
        self.assertEqual(got["CACR"], 0x80000000)
        self.assertNotIn("TC", got)
        self.assertNotIn("URP", got)
        self.assertNotIn("MMUSR", got)


if __name__ == "__main__":
    unittest.main()
