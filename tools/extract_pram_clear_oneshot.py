#!/usr/bin/env python3
"""Lift the PRAM-clear one-shot out of fpga_top_clocks.vh into a standalone DUT.

Why this exists
---------------
`vio_boot_ctrl[4]` (pram_clear) is a LEVEL-driven VIO probe-out, but PRAM is
battery-backed now: rtc.v rewrites all 256 bytes on every clock `pram_clear`
is high.  Feeding the raw level in would mean that for as long as the operator
leaves the bit set -- or forever, if the JTAG link drops with it high -- every
PRAM write Mac OS makes is silently swallowed.  fpga_top_clocks.vh therefore
edge-detects the bit into a fixed-length one-shot.

That property (a stuck-high level produces exactly ONE bounded pulse) is the
thing worth regression-testing.  The repo's existing pattern for this
(tb/tb_dbg_rst_pulse.v) is a hand-written *mirror* of the inline RTL, which
can drift out of sync with the logic it claims to cover and then keep passing.
This script avoids that failure mode entirely by extracting the shipped source
between the PRAM_CLEAR_ONESHOT_BEGIN/END markers verbatim, so the testbench
can only ever exercise the real logic.

Usage: extract_pram_clear_oneshot.py <fpga_top_clocks.vh> <out.v>
"""

import sys

BEGIN = "// PRAM_CLEAR_ONESHOT_BEGIN"
END = "// PRAM_CLEAR_ONESHOT_END"

HEADER = """// GENERATED FILE -- DO NOT EDIT.
// Produced by tools/extract_pram_clear_oneshot.py from
// rtl/soc/fpga_top_clocks.vh (verbatim extraction between the
// PRAM_CLEAR_ONESHOT_BEGIN/END markers).  Edit the RTL, not this file.
//
// PULSE_CYCLES / PCNT_W are overridden by the Makefile so the test can run
// with a short, sim-friendly pulse; the logic itself is untouched.
module pram_clear_oneshot_dut #(
    parameter integer PULSE_CYCLES = 8,
    parameter integer PCNT_W       = 5
) (
    input  wire sys_clk,
    input  wire platform_resetn,
    input  wire jtag_pram_clear,
    output wire pulse_out
);
    localparam integer DBG_RST_PULSE_CYCLES = PULSE_CYCLES;
    localparam integer DBG_RST_PCNT_W       = PCNT_W;

"""

FOOTER = """
    assign pulse_out = pram_clear_pulse_active;
endmodule
"""


def main():
    if len(sys.argv) != 3:
        sys.stderr.write(__doc__)
        return 2
    src_path, out_path = sys.argv[1], sys.argv[2]
    src = open(src_path).read()

    if src.count(BEGIN) != 1 or src.count(END) != 1:
        sys.stderr.write(
            "ERROR: expected exactly one %s and one %s in %s "
            "(found %d / %d).  The markers guard a verbatim extraction -- "
            "if you moved or duplicated the one-shot, fix the markers.\n"
            % (BEGIN, END, src_path, src.count(BEGIN), src.count(END))
        )
        return 1

    body = src.split(BEGIN, 1)[1].split(END, 1)[0]

    # Sanity: the extracted region must actually contain the one-shot. A
    # silently-empty extraction would produce a DUT whose output is constant
    # 0, and every "stuck level does not hold the pulse" assertion would pass
    # vacuously -- the exact class of test that cannot fail.
    for needle in ("pram_clear_rising",
                   "pram_clear_pulse_active",
                   "always @(posedge sys_clk"):
        if needle not in body:
            sys.stderr.write(
                "ERROR: extracted region is missing %r -- refusing to emit a "
                "DUT that would pass vacuously.\n" % needle
            )
            return 1

    open(out_path, "w").write(HEADER + body + FOOTER)
    return 0


if __name__ == "__main__":
    sys.exit(main())
