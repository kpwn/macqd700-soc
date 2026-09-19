#!/usr/bin/env python3
"""sst_pack.py — Convert SingleStepTests/m68000 JSON files into a flat
binary form (`.sstpack`) that the Verilator harness can mmap and parse
in tens of microseconds per test.

The SingleStepTests repo ships `.json.bin` (custom MAME dump format).
Their `decode.py` produces `.json`.  This script walks `.json` files
and emits `.sstpack` next to them.

Pack format (little-endian, no padding):
    magic       : 4 bytes  "SST\\x01"
    num_tests   : u32
    For each test:
        name_len    : u16
        name        : name_len bytes (UTF-8, no NUL)
        initial_regs: 19 × u32  (d0..d7, a0..a6, usp, ssp, sr, pc)
        init_ram_n  : u32
        init_ram    : init_ram_n × (addr:u32, val:u8)
        final_regs  : 19 × u32
        final_ram_n : u32
        final_ram   : final_ram_n × (addr:u32, val:u8)

Run:
    tools/sst/sst_pack.py third_party/singlesteptests/v1
"""

import argparse
import glob
import json
import os
import struct
import sys

REG_KEYS = ['d0', 'd1', 'd2', 'd3', 'd4', 'd5', 'd6', 'd7',
            'a0', 'a1', 'a2', 'a3', 'a4', 'a5', 'a6',
            'usp', 'ssp', 'sr', 'pc']

MAGIC = b'SST\x01'


def pack_state(state):
    out = bytearray()
    for k in REG_KEYS:
        out += struct.pack('<I', state[k] & 0xFFFFFFFF)
    ram = state['ram']
    out += struct.pack('<I', len(ram))
    for entry in ram:
        addr, val = entry[0], entry[1]
        out += struct.pack('<IB', addr & 0xFFFFFFFF, val & 0xFF)
    return out


def pack_test(t):
    out = bytearray()
    nm = t['name'].encode('utf-8')
    out += struct.pack('<H', len(nm))
    out += nm
    out += pack_state(t['initial'])
    out += pack_state(t['final'])
    return out


def pack_file(json_path, pack_path):
    with open(json_path) as f:
        tests = json.load(f)
    out = bytearray()
    out += MAGIC
    out += struct.pack('<I', len(tests))
    for t in tests:
        out += pack_test(t)
    with open(pack_path, 'wb') as f:
        f.write(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('input_dir', help='Directory of .json files')
    ap.add_argument('-o', '--out_dir', default=None,
                    help='Output directory (default: same as input_dir)')
    ap.add_argument('-f', '--file', default=None,
                    help='Pack only this single .json filename (basename)')
    ap.add_argument('--decode-bin-first', action='store_true',
                    help='Run decode.py on the input dir first if .json '
                         'files are missing')
    args = ap.parse_args()

    in_dir = os.path.abspath(args.input_dir)
    out_dir = os.path.abspath(args.out_dir) if args.out_dir else in_dir
    os.makedirs(out_dir, exist_ok=True)

    if args.decode_bin_first:
        # Run the upstream decode.py on the parent of the v1 dir
        sst_root = os.path.dirname(in_dir)
        decoder = os.path.join(sst_root, 'decode.py')
        if os.path.exists(decoder):
            cmd = f"cd {sst_root} && python3 decode.py"
            print(f"[sst_pack] running upstream decoder: {cmd}")
            os.system(cmd)

    if args.file:
        files = [os.path.join(in_dir, args.file)]
    else:
        files = sorted(glob.glob(os.path.join(in_dir, '*.json')))

    total_tests = 0
    for json_path in files:
        base = os.path.basename(json_path)
        if not base.endswith('.json') or base.endswith('.json.bin'):
            continue
        pack_path = os.path.join(out_dir, base[:-5] + '.sstpack')
        try:
            pack_file(json_path, pack_path)
        except Exception as e:
            print(f"[sst_pack] {base}: ERROR {e}", file=sys.stderr)
            continue
        sz = os.path.getsize(pack_path)
        with open(json_path) as f:
            nt = len(json.load(f))
        total_tests += nt
        print(f"[sst_pack] {base}: {nt} tests -> {pack_path} ({sz} bytes)")
    print(f"[sst_pack] total {total_tests} tests across {len(files)} files")


if __name__ == '__main__':
    main()
