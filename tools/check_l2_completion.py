#!/usr/bin/env python3
"""Exhaustive two-state check of the actual scalar RTL completion cone.

Bit-parallel truth tables evaluate every combination, including unreachable
ones. This is not a four-state proof or a physical timing prediction.
"""
import re
from pathlib import Path

RTL = Path(__file__).resolve().parents[1] / "rtl/soc/l2c_ctrl.v"
INTERNAL = set("""
s_lookup_active hit_wr_blocked_c s_lookup_hit s_lookup_hit_go
miss_no_hit_c merge_valid_c alloc_valid_c s_lookup_miss_go
ins_ok_c s_lookup_ins_go p2_done_c
lookup_hit_ready_c lookup_merge_ready_c lookup_alloc_ready_c
lookup_install_ready_c lookup_outcome_ready_c
""".split())
LEAVES = set("""
s2_live_c req_illegal skew_hazard_c victim_query_hit any_hit_c
req_is_write mshr_lu_hit mshr_inst_valid hit_rsp_block_c ord_now_block_c
req_full mshr_merge_ready ord_merge_block_c need_fill_c miss_ok_c way_ok_c
""".split())
REFERENCE = """s2_live_c &&
    (req_illegal ? (!hit_rsp_block_c && !ord_now_block_c) :
    (s_lookup_hit_go || merge_valid_c || s_lookup_ins_go || s_lookup_miss_go))"""


def parse(text):
    tokens = re.findall(r"[A-Za-z_]\w*|&&|\|\||[!()?:]", text)
    if "".join(tokens) != re.sub(r"\s", "", text):
        raise ValueError(f"Unsupported syntax: {text}")
    pos = 0

    def take(token):
        nonlocal pos
        if pos < len(tokens) and tokens[pos] == token:
            pos += 1
            return True
        return False

    def primary():
        nonlocal pos
        if take("!"):
            return ("!", primary())
        if take("("):
            node = expr()
            if not take(")"):
                raise ValueError("Missing closing parenthesis")
            return node
        if pos >= len(tokens) or not re.fullmatch(r"[A-Za-z_]\w*", tokens[pos]):
            raise ValueError("Expected scalar identifier")
        node = tokens[pos]
        pos += 1
        return node

    def conjunction():
        node = primary()
        while take("&&"):
            node = ("&", node, primary())
        return node

    def expr():
        node = conjunction()
        while take("||"):
            node = ("|", node, conjunction())
        if take("?"):
            yes = expr()
            if not take(":"):
                raise ValueError("Missing conditional colon")
            node = ("?", node, yes, expr())
        return node

    result = expr()
    if pos != len(tokens):
        raise ValueError("Unconsumed expression tokens")
    return result


def main():
    source = re.sub(r"//[^\n]*|/\*.*?\*/", "", RTL.read_text(), flags=re.S)
    expressions = {}
    for name, value in re.findall(r"\bwire\s+(\w+)\s*=\s*([^;]+);", source):
        if name in INTERNAL:
            if name in expressions:
                raise ValueError(f"Duplicate wire {name}")
            expressions[name] = parse(value)
    cases = 1 << len(LEAVES)
    mask = (1 << cases) - 1
    values = {}
    for bit, name in enumerate(sorted(LEAVES)):
        run = 1 << bit
        # Repeated 00..0011..11 pattern, one bit per input assignment.
        values[name] = (((1 << run) - 1) << run) * (mask // ((1 << (2 * run)) - 1))

    def evaluate(node, active=()):
        if isinstance(node, str):
            if node in values:
                return values[node]
            if node in active or node not in expressions:
                raise ValueError(f"Unknown/cyclic wire {node}")
            return evaluate(expressions[node], active + (node,))
        op, *args = node
        v = [evaluate(arg, active) for arg in args]
        if op == "!":
            return mask ^ v[0]
        if op == "&":
            return v[0] & v[1]
        if op == "|":
            return v[0] | v[1]
        return (v[0] & v[1]) | ((mask ^ v[0]) & v[2])

    actual = evaluate("p2_done_c")
    expected = evaluate(parse(REFERENCE))
    if actual != expected:
        differing = (actual ^ expected) & -(actual ^ expected)
        case = differing.bit_length() - 1
        assignment = {name: (case >> bit) & 1 for bit, name in enumerate(sorted(LEAVES))}
        raise AssertionError(f"Completion mismatch: {assignment}")
    # Sensitivity: forgetting the victim guard must be detected.
    saved = values["victim_query_hit"]
    values["victim_query_hit"] = 0
    mutant = evaluate("p2_done_c")
    values["victim_query_hit"] = saved
    if mutant == expected:
        raise AssertionError("Reference does not detect missing victim guard")
    print(f"L2_COMPLETION_EQ_PASS cases={cases} scalar_inputs={len(LEAVES)} victim_mutant=detected")


if __name__ == "__main__":
    main()
