#!/usr/bin/env python3
"""Structural pre-flight for an ILA_ENABLE + CPU_M68K040 build.

Checks the three things that fail LATE and expensively otherwise:
  1. every port the SoC instantiates exists on the generated M68kSocketTop
     (a missing one is a Verilog elaboration error ~40 min into a build);
  2. every dbg_ila_*/raw_daxi_* wire is driven exactly once (undriven ->
     Chipscope 16-213 hard error at debug-core link, driven twice -> X);
  3. every probeN on the debug_ila instance is connected.
"""
import re, sys, collections

VH   = "rtl/soc/fpga_top_debug_ctrl.vh"
VIO  = "rtl/soc/fpga_top_debug_vio.vh"
NET  = "cpu040/generated/M68kSocketTop.v"
fail = 0

vh  = open(VH).read()
vio = open(VIO).read()
net = open(NET).read()

# ---- 1. instantiated ports vs the generated module -------------------------
m = re.search(r"module M68kSocketTop.*?\n\);", net, re.S)
assert m, "could not find the M68kSocketTop port list"
netports = set(re.findall(r"^\s*(?:input|output|inout)\s+(?:wire\s+)?(?:\[[^\]]+\]\s*)?([A-Za-z_][A-Za-z0-9_]*)",
                          m.group(0), re.M))

inst = re.search(r"M68kSocketTop\s+u_cpu\s*\((.*?)\n    \);", vh, re.S)
assert inst, "could not find the M68kSocketTop u_cpu instantiation"
instports = set(re.findall(r"\.([A-Za-z_][A-Za-z0-9_]*)\s*\(", inst.group(1)))

missing = sorted(instports - netports)
print(f"[1] instantiated ports: {len(instports)}, on generated module: {len(netports)}")
if missing:
    fail = 1
    print(f"    FAIL: {len(missing)} instantiated port(s) do NOT exist on M68kSocketTop:")
    for p in missing:
        print(f"      {p}")
else:
    print("    ok: every instantiated port exists")

# ---- 2. wire drivers -------------------------------------------------------
# Only the shared declaration region matters; both `ifdef arms declare nothing new.
declared = set(re.findall(r"wire\s*(?:\[[^\]]+\]\s*)?(dbg_ila_[A-Za-z0-9_]*_w|raw_daxi_[A-Za-z0-9_]*)\s*[;=]", vh))
# A wire counts as driven if it is assigned, declared-with-initialiser, or bound
# to an output port of u_cpu (whole-wire or any bit-slice).
assigned = set(re.findall(r"assign\s+(dbg_ila_[A-Za-z0-9_]*_w|raw_daxi_[A-Za-z0-9_]*)\s*=", vh))
initd    = set(re.findall(r"wire\s*(?:\[[^\]]+\]\s*)?(dbg_ila_[A-Za-z0-9_]*_w|raw_daxi_[A-Za-z0-9_]*)\s*=", vh))
bound    = set(re.findall(r"\.\w+\s*\(\s*(dbg_ila_[A-Za-z0-9_]*_w|raw_daxi_[A-Za-z0-9_]*)", inst.group(1)))

driven = assigned | initd | bound
undriven = sorted(declared - driven)
print(f"[2] declared {len(declared)} wires; driven {len(declared & driven)}")
if undriven:
    fail = 1
    print(f"    FAIL: {len(undriven)} UNDRIVEN wire(s) -> Chipscope 16-213 at link:")
    for w in undriven:
        print(f"      {w}")
else:
    print("    ok: every declared wire is driven")

both = sorted((assigned | initd) & bound)
if both:
    fail = 1
    print(f"    FAIL: {len(both)} wire(s) driven BOTH by assign and by a port:")
    for w in both:
        print(f"      {w}")
else:
    print("    ok: no wire is double-driven")

# ---- 3. probe connectivity -------------------------------------------------
ila = re.search(r"debug_ila\s+u_dbg_ila\s*\((.*?)\n\s*\);", vio, re.S)
if not ila:
    print("[3] SKIP: debug_ila instantiation not found")
else:
    probes = {int(n) for n in re.findall(r"\.probe(\d+)\s*\(", ila.group(1))}
    gaps = [i for i in range(max(probes) + 1) if i not in probes]
    print(f"[3] probes connected: {len(probes)} (0..{max(probes)})")
    if gaps:
        fail = 1
        print(f"    FAIL: unconnected probe indices: {gaps}")
    else:
        print("    ok: no probe gaps")

print("RESULT:", "FAIL" if fail else "PASS")
sys.exit(fail)
