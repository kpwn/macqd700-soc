#!/usr/bin/env python3
from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "regstate"))

import regstate_compare  # noqa: E402


class RegstateCompareTests(unittest.TestCase):
    def test_generate_cases_includes_new_parity_and_adversarial_slices(self) -> None:
        cases = {case.name: case for case in regstate_compare.generate_cases()}

        self.assertIn("parity_trap_rte_ccr_restore_d4_b00000000_trap0", cases)
        self.assertIn("parity_user_stack_switch_a7_b00000000_trap0", cases)
        self.assertIn("parity_word_odd3_d1_b00000000_odd3", cases)
        self.assertIn("parity_store_load_same_addr_d2_b00000000_repeat", cases)

        xfail_case = cases["parity_movem_an_in_list_a7_b00000000_a7list"]
        self.assertTrue(xfail_case.xfail)
        self.assertIn("A7 in the reglist", xfail_case.xfail_reason)

    def test_list_annotations_show_xfail_reason(self) -> None:
        cases = [case for case in regstate_compare.generate_cases() if case.xfail]
        self.assertEqual(len(cases), 1)
        self.assertTrue(cases[0].xfail_reason)


if __name__ == "__main__":
    unittest.main()
