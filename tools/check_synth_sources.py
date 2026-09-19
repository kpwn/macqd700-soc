#!/usr/bin/env python3
"""Fail if an RTL file is invisible to the Vivado source list.

WHY THIS EXISTS
---------------
The Makefile discovers Verilog with `find`; synth/vivado.tcl uses an
EXPLICIT `read_verilog` list.  So a new module can pass `make lint`, pass
every unit testbench, and still be completely absent from the only flow
that builds a bitstream.

This has now bitten twice on the same day:

  * pram_sd / pram_cdc / axil_split2 -- caught by review before a build.
  * vhdd_ctrl / vhdd_ddr / vhdd_mux  -- NOT caught; synthesis died at
    `ERROR: [Synth 8-439] module 'vhdd_ctrl' not found` after the modules
    had passed lint and all their testbenches, because the two features
    were developed concurrently in separate worktrees and only one author
    knew about the trap.

Both times the module was fully verified and fully invisible.  That is
the same shape as every other instrument failure on this project: a green
result that measured nothing.  A mechanical check is the only reliable
fix, because the failure mode is precisely "a human did not notice a
second list".

ALLOWLIST
---------
Files that are legitimately NOT part of fpga_top: the standalone
provisioning / sdmin bitstreams, and sim-only models.  Adding to this list
is a deliberate act -- if a module is genuinely used by fpga_top it must
go in vivado.tcl instead.
"""
import pathlib
import re
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent

# Legitimately outside fpga_top.  Keep the reason with the entry.
ALLOWLIST = {
    "rtl/board/sd_bulk_writer.v":   "standalone sd_provision_top bitstream",
    "rtl/board/sim_mig_backend.v":  "sim-only DDR model, never synthesised",
    "rtl/soc/fpga_top_sdmin.v":     "standalone fpga-sdmin-bitstream top",
    "rtl/soc/sd_provision_core.v":  "standalone sd_provision_top bitstream",
    "rtl/soc/sd_provision_top.v":   "standalone provisioning bitstream top",
}


# NOTE (2026-08-03): a "non-ASCII inside translate_off" check lived here
# briefly. It was DISPROVEN and removed. The theory was that em-dashes in
# $display strings inside a translate_off block caused
#     ERROR: [Synth 8-2798] unexpected EOF [rtl/soc/vhdd_ddr.v:665]
# Two facts kill it: rtl/soc/peripheral_bus.v:1604 has the identical
# construct and is in the working bitstream, and restoring the em-dashes to
# vhdd_ddr.v still gives RESULT vhdd_ddr OK from Vivado's own read_verilog.
# The real cause of that EOF is still unknown -- do not re-add this rule
# without evidence that survives both of those checks.


def unbalanced_pragmas(path: pathlib.Path):
    """Vivado treats ANY comment containing a bare pragma token as a pragma.

    Vivado accepts both `// synthesis translate_off` and the bare
    `// translate_off`, and requires the closing partner to use the SAME
    keyword style. So a comment that merely *mentions* the bare token opens a
    region that never closes: Vivado skips to end-of-file and reports

        ERROR: [Synth 8-2798] unexpected EOF [<file>:<lastline>]

    naming the last line of the file, not the comment. Verilator does not
    recognise the bare form at all, so the file lints and simulates clean.

    This cost three build attempts on rtl/soc/vhdd_ddr.v, where the comment
    explaining the pragma contained the bare token. The real clue was a
    WARNING Vivado emits just before the error, [Synth 8-11259] "unmatched
    pragma", which names the true line.

    Rule: per file, bare-form opens must equal bare-form closes, and
    synthesis-form opens must equal synthesis-form closes.
    """
    off = on = 0
    bad = []
    # Vivado only treats a comment as a pragma when its CONTENT BEGINS with
    # the keyword.  That is the discriminator, established from three data
    # points rather than assumed:
    #   vhdd_ddr.v:634  "// translate_off is the ..."          -> pragma, FAILED
    #   axi_xbar.v:2912 "// effect once stripped by translate_off/_on" -> builds
    #   axi_xbar.v:2966 "// same translate_off convention ..."         -> builds
    # So prose that merely mentions the token mid-sentence is harmless; a
    # comment that leads with it opens a real region.
    pat = re.compile(r"^//+\s*(?:(?:synthesis|pragma)\s+)?translate_(off|on)\b")
    for i, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        st = raw.strip()
        if not st.startswith("//"):
            continue
        m = pat.match(st)
        if not m:
            continue
        if m.group(1) == "off":
            off += 1
        else:
            on += 1
        bad.append((i, st[:72]))
    if off != on:
        return bad or [(0, f"translate off/on = {off}/{on}")]
    return []


def main() -> int:
    tcl_path = REPO / "synth" / "vivado.tcl"
    # Most legacy RTL is .v, but gated integrations may legitimately use
    # SystemVerilog.  Check both extensions so a new .sv cannot repeat the
    # exact "lint sees it, bitstream does not" failure this tool prevents.
    listed = set(re.findall(r"([A-Za-z0-9_]+)\.(?:v|sv)\b", tcl_path.read_text()))

    missing = []
    for path in sorted([* (REPO / "rtl").rglob("*.v"),
                        * (REPO / "rtl").rglob("*.sv")]):
        rel = path.relative_to(REPO).as_posix()
        if rel in ALLOWLIST:
            continue
        if path.stem not in listed:
            missing.append(rel)

    stale = [rel for rel in ALLOWLIST if not (REPO / rel).exists()]

    pragma = []
    for path in sorted([* (REPO / "rtl").rglob("*.v"),
                        * (REPO / "rtl").rglob("*.sv")]):
        for ln, txt in unbalanced_pragmas(path):
            pragma.append(f"{path.relative_to(REPO).as_posix()}:{ln}  {txt}")

    if not missing and not stale and not pragma:
        print(f"check-synth-sources: OK "
              f"({len(listed)} names in vivado.tcl, {len(ALLOWLIST)} allowlisted)")
        return 0

    if missing:
        print("check-synth-sources: FAIL -- RTL invisible to synth/vivado.tcl:",
              file=sys.stderr)
        for rel in missing:
            print(f"    {rel}", file=sys.stderr)
        print("\n  These pass lint and simulate fine but will NOT be in the "
              "bitstream.\n  Either add a read_verilog line to synth/vivado.tcl, "
              "or add the file\n  to ALLOWLIST in this script WITH a reason.",
              file=sys.stderr)
    if pragma:
        print("\ncheck-synth-sources: FAIL -- comment contains a BARE pragma "
              "token; Vivado reads it as a real pragma:", file=sys.stderr)
        for loc in pragma:
            print(f"    {loc}", file=sys.stderr)
    if stale:
        print("\ncheck-synth-sources: FAIL -- allowlist entries no longer exist:",
              file=sys.stderr)
        for rel in stale:
            print(f"    {rel}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
