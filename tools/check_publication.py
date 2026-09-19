#!/usr/bin/env python3
"""Check the Git index for accidental release payloads (not a history audit).

Reads indexed blobs, not ignored local firmware or unstaged file contents.
Prints only filenames/reasons, never matched credential values. This is a
guardrail, not a substitute for reviewing licenses, provenance and history.
"""
from pathlib import PurePosixPath
import re
import subprocess
import sys


SECRET = re.compile(
    rb"-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----"
    rb"|ghp_[A-Za-z0-9]{36}|github_pat_[A-Za-z0-9_]{40,}"
    rb"|AKIA[A-Z0-9]{16}"
)
GENERATED = {".bit", ".ltx", ".dcp", ".mcs", ".prm", ".rom", ".log",
             ".jou", ".vcd", ".fst", ".o", ".gch", ".a", ".out"}


def path_problem(name):
    path = PurePosixPath(name)
    if path.parts[0] in {"logs", "build", "build_early_vipt", "captures", ".Xil"}:
        return "generated output directory"
    if name in {"rtl/mac/adb_pic_fw.hex", "files/342s0440-b.bin", "files/pmuv2.bin"}:
        return "user-supplied firmware"
    if path.suffix.lower() in GENERATED:
        return "firmware/build/runtime artifact extension"
    # These small binaries are project-authored CPU regression fixtures.
    if path.suffix.lower() == ".bin" and not name.startswith("tb/fuzz_fails/"):
        return "binary needs explicit provenance review"
    return None


def main():
    entries = subprocess.check_output(["git", "ls-files", "--stage", "-z"]).split(b"\0")
    failures = []
    count = 0
    for entry in filter(None, entries):
        metadata, raw_name = entry.split(b"\t", 1)
        mode, oid, stage = metadata.decode().split()
        name = raw_name.decode()
        count += 1
        if stage != "0":
            failures.append((name, "unresolved merge"))
            continue
        reason = path_problem(name)
        if reason:
            failures.append((name, reason))
        if mode == "160000":
            continue  # independent repository; audit separately
        data = subprocess.check_output(["git", "cat-file", "blob", oid])
        if SECRET.search(data):
            failures.append((name, "credential-shaped content; inspect privately"))
        if mode == "120000":
            target = data.decode()
            depth = len(PurePosixPath(name).parent.parts)
            unsafe = target.startswith("/")
            for part in PurePosixPath(target).parts:
                depth += -1 if part == ".." else (0 if part == "." else 1)
                unsafe |= depth < 0
            if unsafe:
                failures.append((name, "symlink escapes repository"))
    for name, reason in failures:
        print(f"FAIL {name}: {reason}")
    if failures:
        return 1
    print(f"PASS: {count} indexed paths; artifact/firmware/credential-pattern checks.")
    print("History, submodule contents and licensing still require separate review.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
