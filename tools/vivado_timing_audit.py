#!/usr/bin/env python3
"""Summarize Vivado timing reports for FPGA bring-up triage.

This is intentionally a reporting tool.  It does not infer or generate timing
exceptions; paths that are not proven safe must stay visible in Vivado.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


SECTION_RE = re.compile(r"^\|\s+(.+?)\s*$")
CHECK_ITEM_RE = re.compile(r"^\d+\.\s+checking\s+(\S+)\s+\((\d+)\)")
DESIGN_SUMMARY_RE = re.compile(
    r"^\s*(?P<wns>-?\d+(?:\.\d+)?)\s+"
    r"(?P<tns>-?\d+(?:\.\d+)?)\s+"
    r"(?P<fail>\d+)\s+"
    r"(?P<total>\d+)\s+"
    r"(?P<whs>-?\d+(?:\.\d+)?)\s+"
    r"(?P<ths>-?\d+(?:\.\d+)?)\s+"
    r"(?P<hfail>\d+)\s+"
    r"(?P<htotal>\d+)\s+"
    r"(?P<wpws>-?\d+(?:\.\d+)?)\s+"
    r"(?P<tpws>-?\d+(?:\.\d+)?)\s+"
    r"(?P<pfail>\d+)\s+"
    r"(?P<ptotal>\d+)\s*$"
)


def section_name(line: str) -> str | None:
    match = SECTION_RE.match(line)
    if not match:
        return None
    name = match.group(1).strip()
    if name.startswith("-") or not name:
        return None
    return name


def split_row(line: str) -> list[str]:
    return re.split(r"\s{2,}", line.strip())


def is_table_data(line: str) -> bool:
    stripped = line.strip()
    return bool(stripped) and not stripped.startswith("-") and not stripped.startswith("|")


def parse_check_timing(lines: list[str]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for line in lines:
        match = CHECK_ITEM_RE.match(line.strip())
        if match:
            counts[match.group(1)] = int(match.group(2))
    return counts


def parse_design_summary(lines: list[str]) -> dict[str, str] | None:
    in_summary = False
    for line in lines:
        name = section_name(line)
        if name == "Design Timing Summary":
            in_summary = True
            continue
        if in_summary and name and name != "Design Timing Summary":
            return None
        if in_summary:
            match = DESIGN_SUMMARY_RE.match(line)
            if match:
                return match.groupdict()
    return None


def parse_clock_summary(lines: list[str]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    in_table = False
    for line in lines:
        name = section_name(line)
        if name == "Clock Summary":
            in_table = True
            continue
        if in_table and name and name != "Clock Summary":
            break
        if not in_table or not is_table_data(line):
            continue
        parts = split_row(line)
        if len(parts) >= 4 and parts[1].startswith("{") and parts[0] != "Clock":
            rows.append(
                {
                    "clock": parts[0].strip(),
                    "period": parts[2],
                    "frequency": parts[3],
                }
            )
    return rows


def parse_timing_table(lines: list[str], title: str) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    in_table = False
    for line in lines:
        name = section_name(line)
        if name == title:
            in_table = True
            continue
        if in_table and name and name != title:
            break
        if not in_table or not is_table_data(line):
            continue
        parts = split_row(line)
        if title == "Intra Clock Table":
            if len(parts) >= 5 and parts[1] != "WNS(ns)":
                rows.append(
                    {
                        "clock": parts[0],
                        "wns": parts[1],
                        "tns": parts[2],
                        "fail": parts[3],
                        "total": parts[4],
                    }
                )
        elif title == "Inter Clock Table":
            if len(parts) >= 6 and parts[2] != "WNS(ns)":
                rows.append(
                    {
                        "from": parts[0],
                        "to": parts[1],
                        "wns": parts[2],
                        "tns": parts[3],
                        "fail": parts[4],
                        "total": parts[5],
                    }
                )
    return rows


def parse_path_group_table(lines: list[str], title: str) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    in_table = False
    columns: tuple[int, int, int] | None = None
    for line in lines:
        name = section_name(line)
        if name == title:
            in_table = True
            continue
        if in_table and name and name != title:
            break
        if not in_table or not is_table_data(line):
            continue
        if line.strip().startswith("Path Group"):
            group_start = line.index("Path Group")
            from_start = line.index("From Clock")
            to_start = line.index("To Clock")
            columns = (group_start, from_start, to_start)
            continue
        if columns is None:
            continue
        group_start, from_start, to_start = columns
        group = line[group_start:from_start].strip()
        from_clock = line[from_start:to_start].strip()
        to_clock = line[to_start:].strip()
        if group and group != "----------":
            rows.append({"group": group, "from": from_clock or "(blank)", "to": to_clock or "(blank)"})
    return rows


def parse_report(path: Path) -> dict[str, object]:
    lines = path.read_text(errors="replace").splitlines()
    return {
        "check_timing": parse_check_timing(lines),
        "design_summary": parse_design_summary(lines),
        "clocks": parse_clock_summary(lines),
        "intra": parse_timing_table(lines, "Intra Clock Table"),
        "inter": parse_timing_table(lines, "Inter Clock Table"),
        "ignored": parse_path_group_table(lines, "User Ignored Path Table"),
        "unconstrained": parse_path_group_table(lines, "Unconstrained Path Table"),
    }


def negative_float(value: str) -> bool:
    try:
        return float(value) < 0.0
    except ValueError:
        return False


def print_markdown(report_path: Path, parsed: dict[str, object], max_rows: int) -> int:
    check_timing = parsed["check_timing"]
    design = parsed["design_summary"]
    clocks = parsed["clocks"]
    intra = parsed["intra"]
    inter = parsed["inter"]
    ignored = parsed["ignored"]
    unconstrained = parsed["unconstrained"]

    assert isinstance(check_timing, dict)
    assert isinstance(clocks, list)
    assert isinstance(intra, list)
    assert isinstance(inter, list)
    assert isinstance(ignored, list)
    assert isinstance(unconstrained, list)

    exit_code = 0

    print(f"# Vivado timing audit: {report_path}")
    print()

    if isinstance(design, dict):
        if negative_float(design["wns"]) or negative_float(design["whs"]) or negative_float(design["wpws"]):
            exit_code = 2
        print(
            "Design summary: "
            f"WNS {design['wns']} ns, TNS {design['tns']} ns, "
            f"WHS {design['whs']} ns, THS {design['ths']} ns, "
            f"WPWS {design['wpws']} ns, TPWS {design['tpws']} ns"
        )
    else:
        print("Design summary: not found")
        exit_code = 2
    print()

    print("## check_timing")
    for key in (
        "no_clock",
        "unconstrained_internal_endpoints",
        "no_input_delay",
        "no_output_delay",
        "multiple_clock",
        "generated_clocks",
    ):
        count = int(check_timing.get(key, 0))
        if count:
            exit_code = 2
        print(f"- {key}: {count}")
    print()

    print("## clocks")
    for row in clocks[:max_rows]:
        print(f"- {row['clock']}: {row['period']} ns, {row['frequency']} MHz")
    if len(clocks) > max_rows:
        print(f"- ... {len(clocks) - max_rows} more")
    print()

    print("## failing intra-clock groups")
    failing_intra = [row for row in intra if negative_float(row.get("wns", "0"))]
    if failing_intra:
        exit_code = 2
        for row in failing_intra[:max_rows]:
            print(
                f"- {row['clock']}: WNS {row['wns']} ns, TNS {row['tns']} ns, "
                f"failing endpoints {row['fail']}/{row['total']}"
            )
    else:
        print("- none")
    print()

    print("## failing inter-clock groups")
    failing_inter = [row for row in inter if negative_float(row.get("wns", "0"))]
    if failing_inter:
        exit_code = 2
        for row in failing_inter[:max_rows]:
            print(
                f"- {row['from']} -> {row['to']}: WNS {row['wns']} ns, "
                f"TNS {row['tns']} ns, failing endpoints {row['fail']}/{row['total']}"
            )
    else:
        print("- none")
    print()

    print("## ignored and unconstrained path groups")
    if ignored:
        print("Ignored paths:")
        for row in ignored[:max_rows]:
            print(f"- {row['group']}: {row['from']} -> {row['to']}")
    else:
        print("Ignored paths: none")
    if unconstrained:
        exit_code = 2
        print("Unconstrained paths:")
        for row in unconstrained[:max_rows]:
            print(f"- {row['group']}: {row['from']} -> {row['to']}")
    else:
        print("Unconstrained paths: none")

    print()
    print(
        "Audit rule: this tool only reports blockers.  Do not convert any "
        "reported CDC path into a false path unless the RTL boundary is "
        "proven safe by a synchronizer, async FIFO, Xilinx CDC IP, or a "
        "documented static tie-off."
    )
    return exit_code


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path, help="Vivado report_timing_summary output")
    parser.add_argument("--max-rows", type=int, default=20, help="maximum rows per section")
    args = parser.parse_args()

    if not args.report.exists():
        print(f"ERROR: report not found: {args.report}", file=sys.stderr)
        return 1

    parsed = parse_report(args.report)
    return print_markdown(args.report, parsed, args.max_rows)


if __name__ == "__main__":
    raise SystemExit(main())
