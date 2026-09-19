#!/usr/bin/env python3
"""Expand a Taxi .f filelist into a deduplicated source list.

synth/vivado.tcl already does this (read_taxi_filelist) so Vivado sees the
right sources for an ETH_ENABLE build.  `make lint-eth-link` needs the same
list to elaborate rtl/board/q700_eth_link.sv against the real MAC instead of
skipping it, so the expansion lives here rather than being duplicated in shell.

Entries are relative to the .f file that names them and may themselves be .f
files.  Deduplication is by BASENAME, not path: the vendored tree ships the
same module (taxi_sync_reset.sv, taxi_sync_signal.sv) under several lib/
mirrors, and feeding two copies to Verilator is a duplicate-module error.
"""
import os
import sys


def expand(entry, seen_files, seen_names, out):
    entry = os.path.normpath(entry)
    if entry in seen_files:
        return
    seen_files.add(entry)
    base = os.path.dirname(entry)
    with open(entry) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            path = os.path.normpath(os.path.join(base, line))
            if path.endswith('.f'):
                expand(path, seen_files, seen_names, out)
            elif os.path.basename(path) not in seen_names:
                seen_names.add(os.path.basename(path))
                out.append(path)


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: taxi_filelist.py <rk5-eth-dir>")
    root = os.path.abspath(sys.argv[1])
    taxi = os.path.join(root, 'third_party', 'taxi', 'src')
    out, seen_files, seen_names = [], set(), set()
    expand(os.path.join(taxi, 'eth/rtl/taxi_eth_mac_1g_rgmii_fifo.f'),
           seen_files, seen_names, out)
    for extra in (os.path.join(taxi, 'sync/rtl/taxi_sync_reset.sv'),
                  os.path.join(root, 'rtl', 'icmp_echo_responder.sv')):
        if os.path.basename(extra) not in seen_names:
            seen_names.add(os.path.basename(extra))
            out.append(extra)
    missing = [p for p in out if not os.path.isfile(p)]
    if missing:
        sys.exit("taxi_filelist.py: missing source(s): " + " ".join(missing))
    print(' '.join(out))


if __name__ == '__main__':
    main()
