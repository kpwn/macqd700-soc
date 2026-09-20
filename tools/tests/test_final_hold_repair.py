"""Exercise the final hold-repair rollback with mocked Vivado commands."""
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / 'synth/vivado.tcl').read_text()
START = SOURCE.index('    if {[_pr_whs] < 0} {', SOURCE.index('# Final hold repair can regress'))
STOP = SOURCE.index('    puts [format "=== POST-ROUTE PHYS_OPT done:', START)
BLOCK = SOURCE[START:STOP]


class FinalHoldRepairTest(unittest.TestCase):
    def run_case(self, after, expected, fail=0, checkpoint_fail=0):
        script = '''
set output_dir /unused
set wns -0.026
set whs -0.066
set calls {}
proc _pr_wns {} {return $::wns}
proc _pr_whs {} {return $::whs}
proc write_checkpoint {args} {
    lappend ::calls save
    if {$::checkpoint_fail} {error "checkpoint failed"}
    set ::saved [list $::wns $::whs]
}
proc phys_opt_design {args} {
    lappend ::calls repair
    lassign $::after ::wns ::whs
    if {$::fail} {error "repair failed"}
}
proc close_design {} {lappend ::calls close}
proc open_checkpoint {args} {
    lappend ::calls restore
    lassign $::saved ::wns ::whs
}
'''
        script += f'set after {{{after}}}\nset fail {fail}\nset checkpoint_fail {checkpoint_fail}\n'
        script += 'set rc [catch {\n' + BLOCK + '\n}]\n'
        script += 'puts "RESULT=$rc,$wns,$whs,$calls"\n'
        result = subprocess.run(['tclsh'], input=script, text=True, capture_output=True, check=True)
        self.assertIn(expected, result.stdout)

    def test_setup_regression_restored(self):
        self.run_case('-1.965 -0.066', 'RESULT=0,-0.026,-0.066,save repair close restore')

    def test_hold_regression_restored(self):
        self.run_case('0.020 -0.080', 'RESULT=0,-0.026,-0.066,save repair close restore')

    def test_improvement_kept(self):
        self.run_case('0.020 0.005', 'RESULT=0,0.020,0.005,save repair')

    def test_tool_failure_restored(self):
        self.run_case('0.020 0.005', 'RESULT=0,-0.026,-0.066,save repair close restore', fail=1)

    def test_checkpoint_failure_prevents_mutation(self):
        self.run_case('0.020 0.005', 'RESULT=1,-0.026,-0.066,save', checkpoint_fail=1)


if __name__ == '__main__':
    unittest.main()
