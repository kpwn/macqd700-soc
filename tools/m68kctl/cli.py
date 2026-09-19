"""cli.py — ``m68kctl`` argparse CLI.

Subcommand tree:

    m68kctl info                            — VERSION/BUILD_ID + link status
    m68kctl cpu halt|resume|step|redirect   — control
    m68kctl cpu regs|trace                  — observability
    m68kctl sd  read|write|upload|verify    — SD provisioning
    m68kctl sd  check-layout|make-image      — raw first-light SD images
    m68kctl bus read|write|dump|load        — system-bus access
    m68kctl checkpoint dump                — bulk checkpoint capture

The ``--mock`` flag selects the ``MockDevice`` backend for any subcommand,
so the full tree exercises cleanly on a workstation with no FPGA.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
from pathlib import Path
from typing import List, Optional

from . import regs
from .bus import SystemBus
from .cpu import CpuDebug
from .device import (
    MOCK_STATE_ENV,
    Device,
    MockDevice,
    XdmaDevice,
    default_mock_state_path,
    reset_mock_state,
)
from .provision import (
    DumpRegion,
    dump_region,
    dump_regions,
    load_region,
    upload_rom,
    verify_rom,
    write_dump_manifest,
)
from .sd import SdCard
from .sd_image import format_plan, plan_image, write_image


# ═════════════════════════════════════════════════════════════════════════
# Device factory
# ═════════════════════════════════════════════════════════════════════════
def _resolve_mock_state_path() -> str:
    """Resolve the pickle path used by CLI-mode ``--mock`` invocations.

    Precedence:
      1. ``$M68KCTL_MOCK_STATE`` from the caller's environment.
      2. ``default_mock_state_path()`` (XDG-aware default).

    We intentionally do NOT write the resolved path back to
    ``os.environ`` inside the parent Python process: that would leak
    persistence into library consumers who construct a plain
    ``MockDevice()`` in the same process (e.g. our own unittest
    suite).  Any child processes the CLI spawns will still inherit
    ``M68KCTL_MOCK_STATE`` if the user exported it upstream.
    """
    return os.environ.get(MOCK_STATE_ENV) or default_mock_state_path()


def _make_device(args: argparse.Namespace) -> Device:
    if args.mock:
        return MockDevice(state_path=_resolve_mock_state_path())
    return XdmaDevice(
        bar1_dev=args.bar1_dev,
        h2c_dev=args.h2c_dev,
        c2h_dev=args.c2h_dev,
    )


# ═════════════════════════════════════════════════════════════════════════
# ``info`` — identify the bitstream
# ═════════════════════════════════════════════════════════════════════════
def cmd_info(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        cpu = CpuDebug(dev, verify_magic=not args.no_verify)
        sd = SdCard(dev)
        print(f'DBG_VERSION  : 0x{cpu.version:08x}')
        print(f'DBG_BUILD_ID : 0x{cpu.build_id:08x}')
        print(f'SDP_VERSION  : 0x{sd.version:08x}')
        print(f'STATUS       : 0x{cpu.status:08x}  '
              f'(halted={cpu.halted} running={cpu.running} '
              f'init_done={cpu.init_done_seen})')
        print(f'SD card_ready: {sd.card_ready}')
        print(f'PC           : 0x{cpu.pc:08x}')
        print(f'CYCLES       : {cpu.cycles}')
        print(f'INSTS        : {cpu.insts}')
        print(f'IPC          : {cpu.ipc:.3f}')
        print(f'MISPRED      : {cpu.mispred_count}')
        print(f'FLUSHES      : {cpu.flush_count}')
        print(f'EXCEPTIONS   : {cpu.exc_count}')
        if args.mock:
            print('(mock backend — no real FPGA accessed)')
    return 0


# ═════════════════════════════════════════════════════════════════════════
# ``cpu`` subcommands
# ═════════════════════════════════════════════════════════════════════════
def cmd_cpu_halt(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        CpuDebug(dev, verify_magic=not args.no_verify).halt()
    return 0


def cmd_cpu_resume(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        CpuDebug(dev, verify_magic=not args.no_verify).resume()
    return 0


def cmd_cpu_step(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        CpuDebug(dev, verify_magic=not args.no_verify).step()
    return 0


def cmd_cpu_soft_rst(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        CpuDebug(dev, verify_magic=not args.no_verify).soft_rst()
    return 0


def cmd_cpu_reset_halt(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        CpuDebug(dev, verify_magic=not args.no_verify).reset_halt()
    return 0


def cmd_cpu_redirect(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        CpuDebug(dev, verify_magic=not args.no_verify).redirect(args.pc)
    return 0


def cmd_cpu_regs(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        cpu = CpuDebug(dev, verify_magic=not args.no_verify)
        rg = cpu.regs()
        tier = rg.pop('_tier', None)
        for k, v in rg.items():
            print(f'{k:4s} : 0x{v:08x}')
        if tier is not None:
            print(f'\n(debug_ctrl tier observed: {tier})')
    return 0


def cmd_cpu_trace(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        cpu = CpuDebug(dev, verify_magic=not args.no_verify)
        trace = cpu.pc_trace()
        tail = trace[-args.tail:] if args.tail else trace
        print(f'Last {len(tail)} retired PCs (oldest → newest):')
        for pc in tail:
            print(f'  0x{pc:08x}')
    return 0


def cmd_cpu_halt_status(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        cpu = CpuDebug(dev, verify_magic=not args.no_verify)
        hs = cpu.halt_status()
        print(f'HALT_CTL       : 0x{hs.control:08x}  '
              f'(halt_after_en={hs.halt_after_enabled} '
              f'break_pc_en={hs.break_pc_enabled} '
              f'halt_exc_en={hs.halt_exc_enabled} '
              f'auto_latched={hs.auto_latched})')
        print(f'HALT_REASON    : 0x{hs.reason:08x}')
        print(f'HALT_HIT_PC    : 0x{hs.hit_pc:08x}')
        print(f'HALT_HIT_INST  : {hs.hit_inst}')
        print(f'HALT_EXC_VEC   : {hs.exc_vec}')
    return 0


def cmd_cpu_halt_after(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        cpu = CpuDebug(dev, verify_magic=not args.no_verify)
        cpu.set_halt_after(args.inst_count, enable=not args.disable,
                           clear=not args.no_clear)
        hs = cpu.halt_status()
        state = 'enabled' if hs.halt_after_enabled else 'disabled'
        print(f'halt-after {state}: threshold={args.inst_count}')
    return 0


def cmd_cpu_break_pc(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        cpu = CpuDebug(dev, verify_magic=not args.no_verify)
        cpu.set_breakpoint(args.pc, enable=not args.disable,
                           clear=not args.no_clear, slot=args.slot)
        hs = cpu.halt_status()
        state = 'enabled' if hs.break_pc_enabled else 'disabled'
        print(f'break-pc {state}: slot={args.slot} pc=0x{args.pc & 0xFFFFFFFF:08x}')
    return 0


def cmd_cpu_halt_exc(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        cpu = CpuDebug(dev, verify_magic=not args.no_verify)
        cpu.set_halt_exception(args.vec, enable=not args.disable,
                               clear=not args.no_clear)
        hs = cpu.halt_status()
        state = 'enabled' if hs.halt_exc_enabled else 'disabled'
        print(f'halt-exc {state}: vec={args.vec & 0xFF}')
    return 0


def cmd_cpu_clear_halt(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        CpuDebug(dev, verify_magic=not args.no_verify).clear_auto_halt()
    return 0


def _parse_state_pairs(pairs: List[str]) -> dict[str, int]:
    state: dict[str, int] = {}
    for pair in pairs:
        if '=' not in pair:
            raise argparse.ArgumentTypeError(f'expected KEY=VALUE, got {pair!r}')
        key, value = pair.split('=', 1)
        state[key.upper()] = _int_auto(value)
    return state


def cmd_cpu_load_arch(args: argparse.Namespace) -> int:
    state = _parse_state_pairs(args.state)
    with _make_device(args) as dev:
        cpu = CpuDebug(dev, verify_magic=not args.no_verify)
        status = cpu.load_arch_state(state, apply=not args.no_apply)
        print(f'arch shadow words={len(state)} status=0x{status:08x}')
    return 0


# ═════════════════════════════════════════════════════════════════════════
# ``sd`` subcommands
# ═════════════════════════════════════════════════════════════════════════
def _progress(done: int, total: int) -> None:
    pct = (100.0 * done / total) if total else 0.0
    sys.stdout.write(f'\r  {done:>10d} / {total:<10d}  ({pct:5.1f}%)')
    sys.stdout.flush()
    if done >= total:
        sys.stdout.write('\n')


def cmd_sd_read(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        sd = SdCard(dev)
        data = sd.read_block(args.lba)
    if args.output:
        Path(args.output).write_bytes(data)
        print(f'LBA {args.lba}: 512 B → {args.output}')
    else:
        # 16-byte-per-line hex dump
        for i in range(0, len(data), 16):
            row = data[i:i + 16]
            hex_part = ' '.join(f'{b:02x}' for b in row)
            asc_part = ''.join(chr(b) if 32 <= b < 127 else '.' for b in row)
            print(f'{i:04x}  {hex_part:<48s}  {asc_part}')
    return 0


def cmd_sd_write(args: argparse.Namespace) -> int:
    data = Path(args.input).read_bytes()
    if len(data) > regs.SDP_BLOCK_SIZE:
        print(f'error: {args.input} is {len(data)} B > 512 B; '
              f'use `m68kctl sd upload` for multi-block',
              file=sys.stderr)
        return 2
    # pad to block
    if len(data) < regs.SDP_BLOCK_SIZE:
        data = data + b'\x00' * (regs.SDP_BLOCK_SIZE - len(data))
    with _make_device(args) as dev:
        sd = SdCard(dev)
        sd.write_block(args.lba, data)
    print(f'wrote LBA {args.lba}: 512 B from {args.input}')
    return 0


def cmd_sd_upload(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        sd = SdCard(dev)
        n = upload_rom(sd, args.image, start_lba=args.start_lba,
                       progress_cb=_progress if args.progress else None)
    print(f'uploaded {n} blocks from {args.image}')
    return 0


def cmd_sd_verify(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        sd = SdCard(dev)
        bad = verify_rom(sd, args.image, start_lba=args.start_lba,
                         progress_cb=_progress if args.progress else None)
    if bad:
        print(f'verify FAILED: {len(bad)} block mismatches')
        for lba, _, _ in bad[:16]:
            print(f'  mismatch at LBA {lba}')
        if len(bad) > 16:
            print(f'  ... ({len(bad) - 16} more)')
        return 1
    print(f'verify OK ({args.image})')
    return 0


def cmd_sd_check_layout(args: argparse.Namespace) -> int:
    plan = plan_image(args.rom, args.hdd)
    if args.json:
        print(json.dumps(plan.as_dict(), indent=2, sort_keys=True))
    else:
        print(format_plan(plan))
    return 0


def cmd_sd_make_image(args: argparse.Namespace) -> int:
    plan = plan_image(args.rom, args.hdd)
    size = write_image(plan, args.output, overwrite=args.overwrite)
    if args.json:
        data = plan.as_dict()
        data['output'] = args.output
        data['written_bytes'] = size
        print(json.dumps(data, indent=2, sort_keys=True))
    else:
        print(format_plan(plan))
        print(f'wrote raw SD image: {args.output} ({size} bytes)')
    return 0


# ═════════════════════════════════════════════════════════════════════════
# ``bus`` subcommands
# ═════════════════════════════════════════════════════════════════════════
def cmd_bus_read(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        bus = SystemBus(dev)
        data = bus.read_bytes(args.addr, args.length)
    if args.output:
        Path(args.output).write_bytes(data)
        print(f'0x{args.addr:08x}: {len(data)} B → {args.output}')
    else:
        for i in range(0, len(data), 16):
            row = data[i:i + 16]
            hex_part = ' '.join(f'{b:02x}' for b in row)
            asc_part = ''.join(chr(b) if 32 <= b < 127 else '.' for b in row)
            print(f'{args.addr + i:08x}  {hex_part:<48s}  {asc_part}')
    return 0


def cmd_bus_write(args: argparse.Namespace) -> int:
    # Accept either --input PATH or --bytes HEX-STRING
    if args.input:
        data = Path(args.input).read_bytes()
    elif args.bytes:
        data = bytes.fromhex(args.bytes.replace(' ', '').replace(',', ''))
    else:
        print('error: must specify --input PATH or --bytes HEX', file=sys.stderr)
        return 2
    with _make_device(args) as dev:
        bus = SystemBus(dev)
        bus.write_bytes(args.addr, data)
    print(f'wrote {len(data)} B @ 0x{args.addr:08x}')
    return 0


def cmd_bus_dump(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        bus = SystemBus(dev)
        n = dump_region(bus, args.addr, args.length, args.output,
                        progress_cb=_progress if args.progress else None)
    print(f'dumped {n} B from 0x{args.addr:08x} → {args.output}')
    return 0


def cmd_bus_load(args: argparse.Namespace) -> int:
    with _make_device(args) as dev:
        bus = SystemBus(dev)
        n = load_region(bus, args.addr, args.input,
                        progress_cb=_progress if args.progress else None)
    print(f'loaded {n} B from {args.input} → 0x{args.addr:08x}')
    return 0


def cmd_checkpoint_dump(args: argparse.Namespace) -> int:
    if not args.region:
        print('error: at least one --region NAME ADDR LENGTH is required',
              file=sys.stderr)
        return 2

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    regions = []
    for name, addr, length in args.region:
        addr_i = _int_auto(addr)
        length_i = _int_auto(length)
        regions.append(DumpRegion(
            name=name,
            addr=addr_i,
            length=length_i,
            path=out_dir / f'{name}.bin',
        ))

    with _make_device(args) as dev:
        bus = SystemBus(dev)
        manifest = dump_regions(
            bus,
            regions,
            progress_cb=_progress if args.progress else None,
            chunk_bytes=args.chunk_bytes,
        )

    manifest_path = out_dir / args.manifest_name
    write_dump_manifest(
        manifest_path,
        manifest=manifest,
        source='m68kctl checkpoint dump',
        chunk_bytes=args.chunk_bytes,
        total_bytes=sum(region.length for region in regions),
    )
    print(f'wrote {len(regions)} region(s) to {out_dir}')
    print(f'manifest: {manifest_path}')
    return 0


# ═════════════════════════════════════════════════════════════════════════
# Parser assembly
# ═════════════════════════════════════════════════════════════════════════
def _add_global_flags(p: argparse.ArgumentParser) -> None:
    p.add_argument('--mock', action='store_true',
                   help='use MockDevice instead of real /dev/xdma0_* '
                        '(for host-side smoke testing)')
    p.add_argument('--reset-state', action='store_true',
                   help='with --mock: delete the persisted MockDevice '
                        'pickle snapshot before running the subcommand '
                        '(gives a fresh device each session)')
    p.add_argument('--no-verify', action='store_true',
                   help='skip the DBG_VERSION magic check on open')
    p.add_argument('--bar1-dev', default=XdmaDevice.BAR1_DEV,
                   help=f'BAR1 char device (default: {XdmaDevice.BAR1_DEV})')
    p.add_argument('--h2c-dev', default=XdmaDevice.H2C_DEV,
                   help=f'H2C DMA device (default: {XdmaDevice.H2C_DEV})')
    p.add_argument('--c2h-dev', default=XdmaDevice.C2H_DEV,
                   help=f'C2H DMA device (default: {XdmaDevice.C2H_DEV})')
    p.add_argument('--log-level',
                   default='WARNING',
                   choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                   help='logging level (default: WARNING)')


def _int_auto(x: str) -> int:
    """Accept decimal, 0x-prefixed hex, or underscore-separated digits."""
    return int(x.replace('_', ''), 0)


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog='m68kctl',
        description='m68k-ooo host-side control and debug CLI.')
    _add_global_flags(ap)
    sub = ap.add_subparsers(dest='cmd', required=True)

    # info
    p = sub.add_parser('info', help='print identity + link status')
    p.set_defaults(func=cmd_info)

    # cpu <op>
    pcpu = sub.add_parser('cpu', help='CPU control + observability')
    scpu = pcpu.add_subparsers(dest='op', required=True)

    sp = scpu.add_parser('halt',     help='assert halt_req')
    sp.set_defaults(func=cmd_cpu_halt)
    sp = scpu.add_parser('resume',   help='clear halt_req')
    sp.set_defaults(func=cmd_cpu_resume)
    sp = scpu.add_parser('step',     help='single-step pulse')
    sp.set_defaults(func=cmd_cpu_step)
    sp = scpu.add_parser('soft-rst', help='pulse soft_rst')
    sp.set_defaults(func=cmd_cpu_soft_rst)
    sp = scpu.add_parser('reset-halt',
                         help='pulse CPU soft reset and keep halt_req asserted')
    sp.set_defaults(func=cmd_cpu_reset_halt)
    sp = scpu.add_parser('redirect', help='force PC')
    sp.add_argument('pc', type=_int_auto, help='target PC (hex with 0x prefix)')
    sp.set_defaults(func=cmd_cpu_redirect)
    sp = scpu.add_parser('regs',     help='dump arch register snapshot')
    sp.set_defaults(func=cmd_cpu_regs)
    sp = scpu.add_parser('trace',    help='dump PC trace ring')
    sp.add_argument('--tail', type=int, default=32,
                    help='print only the last N entries (default: 32; 0 = all)')
    sp.set_defaults(func=cmd_cpu_trace)
    sp = scpu.add_parser('halt-status', help='show programmable halt/breakpoint state')
    sp.set_defaults(func=cmd_cpu_halt_status)
    sp = scpu.add_parser('halt-after', help='halt after retired instruction count reaches N')
    sp.add_argument('inst_count', type=_int_auto,
                    help='retired instruction-boundary count threshold')
    sp.add_argument('--disable', action='store_true',
                    help='program threshold but leave halt-after disabled')
    sp.add_argument('--no-clear', action='store_true',
                    help='do not clear an existing latched auto-halt')
    sp.set_defaults(func=cmd_cpu_halt_after)
    sp = scpu.add_parser('break-pc', help='halt before the instruction at PC takes effect')
    sp.add_argument('pc', type=_int_auto, help='retired PC to match')
    sp.add_argument('--slot', type=int, choices=range(4), default=0,
                    help='hardware breakpoint slot (default: 0)')
    sp.add_argument('--disable', action='store_true',
                    help='program PC but leave breakpoint disabled')
    sp.add_argument('--no-clear', action='store_true',
                    help='do not clear an existing latched auto-halt')
    sp.set_defaults(func=cmd_cpu_break_pc)
    sp = scpu.add_parser('halt-exc',
                         help='halt after precise exception vector entry (default: illegal instruction vector 4)')
    sp.add_argument('vec', type=_int_auto, nargs='?', default=4,
                    help='exception vector to match (default: 4)')
    sp.add_argument('--disable', action='store_true',
                    help='program vector but leave halt-exc disabled')
    sp.add_argument('--no-clear', action='store_true',
                    help='do not clear an existing latched auto-halt')
    sp.set_defaults(func=cmd_cpu_halt_exc)
    sp = scpu.add_parser('clear-halt', help='clear latched auto-halt hold')
    sp.set_defaults(func=cmd_cpu_clear_halt)
    sp = scpu.add_parser('load-arch',
                         help='write halt-time arch shadow KEY=VALUE pairs and apply/resume')
    sp.add_argument('--no-apply', action='store_true',
                    help='only write shadow registers; do not apply/resume')
    sp.add_argument('state', nargs='+',
                    help='KEY=VALUE, e.g. PC=0x40800000 D0=1 A7=0x1000 SR=0x2700')
    sp.set_defaults(func=cmd_cpu_load_arch)

    # sd <op>
    psd = sub.add_parser('sd', help='SD card provisioning')
    ssd = psd.add_subparsers(dest='op', required=True)

    sp = ssd.add_parser('read',  help='read single block')
    sp.add_argument('lba', type=_int_auto, help='block address')
    sp.add_argument('-o', '--output', help='write block to this file '
                                            '(default: hex dump to stdout)')
    sp.set_defaults(func=cmd_sd_read)

    sp = ssd.add_parser('write', help='write single block')
    sp.add_argument('lba', type=_int_auto, help='block address')
    sp.add_argument('input', help='block data file (<=512 B; zero-padded)')
    sp.set_defaults(func=cmd_sd_write)

    sp = ssd.add_parser('upload', help='upload image (multi-block)')
    sp.add_argument('image', help='image file to upload')
    sp.add_argument('--start-lba', type=_int_auto, default=0,
                    help='starting block address (default: 0)')
    sp.add_argument('--progress', action='store_true', default=True,
                    help='show progress bar (default: on)')
    sp.set_defaults(func=cmd_sd_upload)

    sp = ssd.add_parser('verify', help='read back and diff against image')
    sp.add_argument('image', help='reference image to diff against')
    sp.add_argument('--start-lba', type=_int_auto, default=0,
                    help='starting block address (default: 0)')
    sp.add_argument('--progress', action='store_true', default=True)
    sp.set_defaults(func=cmd_sd_verify)

    sp = ssd.add_parser('check-layout',
                        help='validate the ROM/raw-HDD SD image split')
    sp.add_argument('--rom', required=True, help='ROM image for SD LBA 0')
    sp.add_argument('--hdd', help='optional raw SCSI disk image')
    sp.add_argument('--json', action='store_true',
                    help='print machine-readable layout data')
    sp.set_defaults(func=cmd_sd_check_layout)

    sp = ssd.add_parser('make-image',
                        help='create a raw SD image with ROM then HDD')
    sp.add_argument('--rom', required=True, help='ROM image for SD LBA 0')
    sp.add_argument('--hdd', help='optional raw SCSI disk image')
    sp.add_argument('-o', '--output', required=True,
                    help='output raw SD-card image path')
    sp.add_argument('--overwrite', action='store_true',
                    help='replace an existing output image')
    sp.add_argument('--json', action='store_true',
                    help='print machine-readable layout data')
    sp.set_defaults(func=cmd_sd_make_image)

    # bus <op>
    pbus = sub.add_parser('bus', help='system-bus (RAM / ROM / FB / I-O) access')
    sbus = pbus.add_subparsers(dest='op', required=True)

    sp = sbus.add_parser('read', help='read bytes from system bus')
    sp.add_argument('addr', type=_int_auto)
    sp.add_argument('length', type=_int_auto)
    sp.add_argument('-o', '--output', help='write to file instead of stdout')
    sp.set_defaults(func=cmd_bus_read)

    sp = sbus.add_parser('write', help='write bytes to system bus')
    sp.add_argument('addr', type=_int_auto)
    g = sp.add_mutually_exclusive_group()
    g.add_argument('--input', help='file with bytes to write')
    g.add_argument('--bytes', help='hex-string bytes (e.g. "DE AD BE EF")')
    sp.set_defaults(func=cmd_bus_write)

    sp = sbus.add_parser('dump', help='dump a region of system bus to a file')
    sp.add_argument('addr', type=_int_auto)
    sp.add_argument('length', type=_int_auto)
    sp.add_argument('-o', '--output', required=True)
    sp.add_argument('--progress', action='store_true', default=True)
    sp.add_argument('--chunk-bytes', type=_int_auto, default=1 << 22,
                    help='DMA chunk size in bytes (default: 4 MiB)')
    sp.set_defaults(func=cmd_bus_dump)

    sp = sbus.add_parser('load', help='load a file into system bus')
    sp.add_argument('addr', type=_int_auto)
    sp.add_argument('--input', required=True)
    sp.add_argument('--progress', action='store_true', default=True)
    sp.add_argument('--chunk-bytes', type=_int_auto, default=1 << 22,
                    help='DMA chunk size in bytes (default: 4 MiB)')
    sp.set_defaults(func=cmd_bus_load)

    # checkpoint <op>
    pchk = sub.add_parser('checkpoint', help='bulk checkpoint capture')
    schk = pchk.add_subparsers(dest='op', required=True)

    sp = schk.add_parser('dump', help='dump multiple RAM/ROM regions')
    sp.add_argument('--output-dir', required=True,
                    help='directory for <name>.bin files and manifest.json')
    sp.add_argument('--manifest-name', default='manifest.json',
                    help='manifest file name inside output-dir')
    sp.add_argument('--region', nargs=3, action='append',
                    metavar=('NAME', 'ADDR', 'LENGTH'),
                    help='repeatable region spec: name base length')
    sp.add_argument('--chunk-bytes', type=_int_auto, default=1 << 22,
                    help='DMA chunk size in bytes (default: 4 MiB)')
    sp.add_argument('--progress', action='store_true', default=True)
    sp.set_defaults(func=cmd_checkpoint_dump)

    return ap


def main(argv: Optional[List[str]] = None) -> int:
    ap = build_parser()
    args = ap.parse_args(argv)
    logging.basicConfig(level=getattr(logging, args.log_level),
                        format='%(levelname)s %(name)s: %(message)s')
    # --reset-state must run BEFORE the subcommand opens the device,
    # otherwise we'd load → wipe file → save blank state at teardown,
    # which is the same as just deleting up front but noisier.  Only
    # meaningful with --mock; warn (but keep going) if misused.
    if args.reset_state:
        if not args.mock:
            print('warning: --reset-state has no effect without --mock',
                  file=sys.stderr)
        else:
            path = _resolve_mock_state_path()
            removed = reset_mock_state(path)
            if removed:
                logging.getLogger(__name__).info(
                    'wiped MockDevice state file %s', path)
    try:
        return args.func(args)
    except FileNotFoundError as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    except PermissionError as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    except RuntimeError as e:
        print(f'error: {e}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
