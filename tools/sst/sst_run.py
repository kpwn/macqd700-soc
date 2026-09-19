#!/usr/bin/env python3
"""sst_run.py — Iterate over .sstpack files and invoke the Verilator
SST harness on each.  Aggregates per-file pass/fail counts and prints
a single summary at the end.

This is a thin wrapper around `build/sst/Vmac_top +sst_pack=<path>`.
Each invocation runs all tests in that one file (or a slice via
--start/--count).  We don't try to multiplex multiple files into a
single Verilator process — each pack gets its own clean Vmac_top.
"""

import argparse
import glob
import os
import re
import subprocess
import sys


def run_one_pack(args, pack_path):
    cmd = [args.bin, f"+sst_pack={pack_path}",
           f"+sst_start={args.start}",
           f"+sst_timeout={args.timeout}",
           f"+sst_warmup={args.warmup}"]
    if args.count is not None:
        cmd.append(f"+sst_count={args.count}")
    if args.verbose:
        cmd.append("+sst_verbose")
    if args.check_pc:
        cmd.append("+sst_check_pc")
    if args.check_sr:
        cmd.append("+sst_check_sr")
    if args.summary_dir:
        base = os.path.splitext(os.path.basename(pack_path))[0]
        cmd.append(f"+sst_summary={args.summary_dir}/{base}.tsv")
    print(f"\n[sst_run] running: {' '.join(cmd)}", flush=True)
    # Drop most of the +verbose stdout; keep PASS/FAIL summary line.
    proc = subprocess.run(cmd, capture_output=True, text=True)
    out = proc.stdout
    err = proc.stderr
    # Forward stderr (load-failed errors etc.)
    if err.strip():
        sys.stderr.write(err)
    # Parse the SUMMARY line.
    m = re.search(
        r"\[sst\] SUMMARY:\s+(\d+)/(\d+)\s+PASS\s+(\d+)\s+FAIL\s+(\d+)\s+TIMEOUT",
        out)
    if m:
        passed = int(m.group(1))
        total  = int(m.group(2))
        failed = int(m.group(3))
        timed  = int(m.group(4))
    else:
        passed = total = failed = timed = 0
        sys.stderr.write(out)
        sys.stderr.write(f"[sst_run] WARN: no SUMMARY in {pack_path} output\n")
    # Forward the per-test PASS/FAIL/TIME lines if verbose.
    if args.verbose:
        sys.stdout.write(out)
    else:
        # In quiet mode, print only failing test names (the diffs are
        # suppressed unless +sst_verbose is set on the harness side).
        for line in out.splitlines():
            if line.startswith("[FAIL]") or line.startswith("[TIME]"):
                print(line)
    return passed, total, failed, timed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True, help="Path to Vmac_top SST harness binary")
    ap.add_argument("--pack-dir", required=True, help="Directory of .sstpack files")
    ap.add_argument("--file", default=None,
                    help="Single pack basename (e.g. ADD.b — matches ADD.b.sstpack)")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--count", type=int, default=None)
    ap.add_argument("--timeout", type=int, default=5000)
    ap.add_argument("--warmup", type=int, default=32)
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--check-pc", action="store_true")
    ap.add_argument("--check-sr", action="store_true")
    ap.add_argument("--summary-dir", default=None,
                    help="If set, each pack gets a .tsv summary file here")
    args = ap.parse_args()

    if args.summary_dir:
        os.makedirs(args.summary_dir, exist_ok=True)

    if args.file:
        # Allow the user to pass either "ADD.b" or "ADD.b.sstpack".
        base = args.file
        if not base.endswith(".sstpack"):
            base += ".sstpack"
        packs = [os.path.join(args.pack_dir, base)]
        if not os.path.exists(packs[0]):
            print(f"[sst_run] error: {packs[0]} not found", file=sys.stderr)
            return 2
    else:
        packs = sorted(glob.glob(os.path.join(args.pack_dir, "*.sstpack")))
        if not packs:
            print(f"[sst_run] error: no .sstpack files in {args.pack_dir}",
                  file=sys.stderr)
            return 2

    total_pass = total_fail = total_timeout = total_total = 0
    per_pack = []
    for p in packs:
        passed, total, failed, timed = run_one_pack(args, p)
        per_pack.append((p, passed, total, failed, timed))
        total_pass += passed
        total_fail += failed
        total_timeout += timed
        total_total += total

    print("\n" + "=" * 72)
    print(f"[sst_run] AGGREGATE: {total_pass}/{total_total} PASS  "
          f"{total_fail} FAIL  {total_timeout} TIMEOUT  "
          f"across {len(per_pack)} packs")
    print("=" * 72)
    for p, passed, total, failed, timed in per_pack:
        name = os.path.basename(p)
        rate = (100.0 * passed / total) if total else 0.0
        print(f"  {name:<32}  {passed:>5}/{total:<5}  ({rate:5.1f}%)  "
              f"fail={failed}  timeout={timed}")
    return 0 if (total_fail + total_timeout) == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
