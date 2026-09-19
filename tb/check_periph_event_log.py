#!/usr/bin/env python3
"""Validate tb_rom_boot peripheral event summary categories.

The ROM harness event logger intentionally emits watched categories even when
the current boot frontier has not reached them.  This checker lets smoke tests
distinguish "category was watched and saw no live ROM traffic yet" from
"logging silently stopped recording a live request class."
"""

import argparse
import re
import sys


CATEGORY_RE = re.compile(r"^\s*category=([A-Za-z0-9_]+)\s+count=([0-9]+)\b")
EVENT_RE = re.compile(r"^\s+([A-Za-z0-9_]+\.[A-Za-z0-9_]+)\s+x([0-9]+)\b")


def parse_expectation(spec):
    if ">=" in spec:
        name, count = spec.split(">=", 1)
        op = ">="
    elif "=" in spec:
        name, count = spec.split("=", 1)
        op = "="
    else:
        raise ValueError(f"bad expectation '{spec}'; use NAME>=N or NAME=N")

    name = name.strip()
    if not name:
        raise ValueError(f"bad expectation '{spec}'; empty category")
    try:
        count_i = int(count, 0)
    except ValueError as exc:
        raise ValueError(f"bad expectation '{spec}'; invalid count") from exc
    if count_i < 0:
        raise ValueError(f"bad expectation '{spec}'; negative count")
    return name, op, count_i


def load_counts(path):
    category_counts = {}
    event_counts = {}
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            match = CATEGORY_RE.match(line)
            if match:
                category_counts[match.group(1)] = int(match.group(2))
                continue
            match = EVENT_RE.match(line)
            if match:
                event_counts[match.group(1)] = int(match.group(2))
    return category_counts, event_counts


def check_expectations(kind, expected, counts):
    failures = []
    for name, op, want in expected:
        if name not in counts:
            failures.append(f"{kind} {name}: missing summary entry")
            continue
        got = counts[name]
        if op == ">=" and got < want:
            failures.append(f"{kind} {name}: count {got} < {want}")
        elif op == "=" and got != want:
            failures.append(f"{kind} {name}: count {got} != {want}")
    return failures


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log", help="peripheral event log emitted by tb_rom_boot")
    parser.add_argument(
        "--expect",
        action="append",
        default=[],
        metavar="CATEGORY>=N",
        help="required summary category/count check; also accepts CATEGORY=N",
    )
    parser.add_argument(
        "--expect-event",
        action="append",
        default=[],
        metavar="CATEGORY.EVENT>=N",
        help="required event-breakdown count check; also accepts CATEGORY.EVENT=N",
    )
    args = parser.parse_args()

    try:
        expected_categories = [parse_expectation(spec) for spec in args.expect]
        expected_events = [parse_expectation(spec) for spec in args.expect_event]
        category_counts, event_counts = load_counts(args.log)
    except (OSError, ValueError) as exc:
        print(f"check_periph_event_log: {exc}", file=sys.stderr)
        return 2

    failures = check_expectations("category", expected_categories, category_counts)
    failures.extend(check_expectations("event", expected_events, event_counts))

    if failures:
        for failure in failures:
            print(f"check_periph_event_log: {failure}", file=sys.stderr)
        return 1

    rendered = ", ".join(
        f"{name}={category_counts[name]}"
        for name, _, _ in expected_categories
        if name in category_counts
    )
    rendered_events = ", ".join(
        f"{name}={event_counts[name]}"
        for name, _, _ in expected_events
        if name in event_counts
    )
    if rendered_events:
        rendered = f"{rendered}; events: {rendered_events}" if rendered else f"events: {rendered_events}"
    print(f"check_periph_event_log: PASS {rendered}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
