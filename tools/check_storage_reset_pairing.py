#!/usr/bin/env python3
"""Structural guard: the SD/SCSI bridge must reset with its handshake peer.

sd_scsi_bridge and sd_ctrl sit on the two ends of one go/done handshake.  The
bridge's only escape from a transfer that dies mid-flight is its peer-reset
rescue, which is armed by a RISING EDGE on core_rst:

    core_rst edge -> core_rst_epoch toggle -> pb_peer_reset -> pb_abort
                  -> pb_busy_internal cleared

If any reset can return sd_ctrl to idle WITHOUT also producing a core_rst edge
on the bridge, that rescue never fires and pb_busy_internal latches forever,
wedging the CPU inside the ROM's blind pseudo-DMA burst.

That is exactly the defect root-caused 2026-09-07: sd_ctrl reset on
warm_storage_reset (the 68040 RESET instruction path) while the bridge reset
only on warm_peripheral_reset.

INVARIANT: reset terms of u_sd_ctrl_scsi.rst  must be a SUBSET of
           reset terms of u_sd_scsi_bridge.core_rst
"""
import re, sys, pathlib

SRC = pathlib.Path(__file__).resolve().parent.parent / "rtl/soc/fpga_top_sd.vh"

def port_expr(text, inst, port):
    """Extract the connection expression for .port(...) inside an instance.

    Anchor on the real INSTANTIATION -- `name (` -- not on any prose mention
    of the name in a comment.  Getting this wrong silently reads ports off a
    neighbouring instance and yields a meaningless PASS.
    """
    hits = [m.start() for m in re.finditer(re.escape(inst) + r"\s*\(", text)]
    if not hits:
        sys.exit(f"FAIL: instantiation of {inst} not found in {SRC}")
    if len(hits) > 1:
        sys.exit(f"FAIL: {inst} instantiated {len(hits)} times; checker needs one")
    i = hits[0]
    # scan forward for .port ( ... ) with paren matching
    m = re.search(r"\.\s*" + re.escape(port) + r"\s*\(", text[i:])
    if not m:
        sys.exit(f"FAIL: port .{port} not found on {inst}")
    start = i + m.end()
    depth, j = 1, start
    while depth:
        if text[j] == "(": depth += 1
        elif text[j] == ")": depth -= 1
        j += 1
    return text[start:j-1]

def terms(expr):
    """Reset-source identifiers OR'd together, comments stripped."""
    expr = re.sub(r"//.*", "", expr)
    expr = re.sub(r"/\*.*?\*/", "", expr, flags=re.S)
    return {t.strip() for t in expr.split("|") if t.strip()}

def main():
    text = SRC.read_text()
    bridge = terms(port_expr(text, "u_sd_scsi_bridge", "core_rst"))
    ctrl   = terms(port_expr(text, "u_sd_ctrl_scsi",   "rst"))
    missing = ctrl - bridge
    if missing:
        print("FAIL: sd_ctrl can be reset without rescuing the bridge.")
        print("      sd_ctrl  rst      terms: " + ", ".join(sorted(ctrl)))
        print("      bridge   core_rst terms: " + ", ".join(sorted(bridge)))
        print("      MISSING from the bridge : " + ", ".join(sorted(missing)))
        print()
        print("  Any of those resets returns sd_ctrl to idle while the bridge")
        print("  keeps pb_busy_internal latched forever -> the CPU wedges in")
        print("  the ROM's blind pseudo-DMA burst.  Add the term to the")
        print("  bridge's .core_rst() so both ends tear down together.")
        return 1
    print("PASS: bridge core_rst covers every sd_ctrl reset source")
    print("      sd_ctrl rst terms : " + ", ".join(sorted(ctrl)))
    print("      bridge  core_rst  : " + ", ".join(sorted(bridge)))
    return 0

if __name__ == "__main__":
    sys.exit(main())
