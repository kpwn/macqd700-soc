#!/usr/bin/env python3
"""Blocking transaction protocol for a MAME-to-RTL Q700 bridge.

This module is shared by early Python scaffolding and the future Verilator
endpoint.  The custom MAME device should issue one request and block for one
response inside each MMIO read/write handler; this preserves synchronous read
return semantics.
"""

from __future__ import annotations

from dataclasses import dataclass
import argparse
import struct
import sys


MAGIC = b"MRTB"
VERSION = 2

OP_READ = 1
OP_WRITE = 2

RESP_OKAY = 0
RESP_SLVERR = 2
RESP_DECERR = 3
RESP_TIMEOUT = 4

REQUEST_STRUCT_V1 = struct.Struct(">4sBBBBIII")
REQUEST_STRUCT = struct.Struct(">4sBBBBIIII")
RESPONSE_STRUCT = struct.Struct(">4sBBHII")


@dataclass(frozen=True)
class Window:
    name: str
    base: int
    size: int
    mirror_mask: int = 0

    def contains(self, addr: int) -> bool:
        ca = canonical_addr(addr)
        return self.base <= ca < self.base + self.size


WINDOWS = (
    Window("via1", 0x50000000, 0x00002000, 0x00FC0000),
    Window("via2", 0x50002000, 0x00002000, 0x00FC0000),
    Window("scc", 0x5000C000, 0x00002000, 0x00FC0000),
    Window("scsi_regs", 0x5000F000, 0x00000100, 0x00FC0000),
    Window("scsi_dma", 0x5000F100, 0x00000002, 0x00FC0000),
    Window("asc", 0x50014000, 0x00002000, 0x00FC0000),
    Window("swim_iwm", 0x5001E000, 0x00002000, 0x00FC0000),
    Window("dafb_regs", 0xF9800000, 0x00000400, 0),
    Window("vram_pixels", 0xF9000000, 0x00100000, 0),
)


@dataclass(frozen=True)
class Request:
    op: int
    size: int
    addr: int
    data: int = 0
    wstrb: int = 0
    pc: int = 0
    cpu_cycles: int = 0

    def pack(self) -> bytes:
        return REQUEST_STRUCT.pack(
            MAGIC,
            VERSION,
            self.op & 0xFF,
            self.size & 0xFF,
            self.wstrb & 0xFF,
            self.addr & 0xFFFFFFFF,
            self.data & 0xFFFFFFFF,
            self.pc & 0xFFFFFFFF,
            self.cpu_cycles & 0xFFFFFFFF,
        )

    @classmethod
    def unpack(cls, payload: bytes) -> "Request":
        if len(payload) == REQUEST_STRUCT_V1.size:
            magic, version, op, size, wstrb, addr, data, pc = REQUEST_STRUCT_V1.unpack(payload)
            cpu_cycles = 0
        else:
            magic, version, op, size, wstrb, addr, data, pc, cpu_cycles = REQUEST_STRUCT.unpack(payload)
        if magic != MAGIC:
            raise ValueError(f"bad magic {magic!r}")
        if version not in (1, VERSION):
            raise ValueError(f"bad version {version}")
        if op not in (OP_READ, OP_WRITE):
            raise ValueError(f"bad op {op}")
        if size not in (1, 2, 4):
            raise ValueError(f"bad size {size}")
        return cls(op=op, size=size, addr=addr, data=data, wstrb=wstrb, pc=pc, cpu_cycles=cpu_cycles)


@dataclass(frozen=True)
class Response:
    resp: int
    data: int = 0
    cycles: int = 0

    def pack(self) -> bytes:
        return RESPONSE_STRUCT.pack(
            MAGIC,
            VERSION,
            self.resp & 0xFF,
            0,
            self.data & 0xFFFFFFFF,
            self.cycles & 0xFFFFFFFF,
        )

    @classmethod
    def unpack(cls, payload: bytes) -> "Response":
        magic, version, resp, _reserved, data, cycles = RESPONSE_STRUCT.unpack(payload)
        if magic != MAGIC:
            raise ValueError(f"bad magic {magic!r}")
        if version not in (1, VERSION):
            raise ValueError(f"bad version {version}")
        return cls(resp=resp, data=data, cycles=cycles)


def canonical_addr(addr: int) -> int:
    addr &= 0xFFFFFFFF
    if (addr & 0xFF000000) == 0x50000000:
        return addr & ~0x00FC0000
    return addr


def window_for_addr(addr: int) -> Window | None:
    for window in WINDOWS:
        if window.contains(addr):
            return window
    return None


def default_wstrb(addr: int, size: int) -> int:
    lane = addr & 0x3
    if size == 1:
        return 1 << (3 - lane)
    if size == 2:
        return 0b1100 >> (lane & 0x2)
    if size == 4:
        return 0b1111
    raise ValueError(f"unsupported size {size}")


def selftest() -> None:
    req = Request(OP_WRITE, 1, 0x50F0F100, data=0xA5, wstrb=default_wstrb(0x50F0F100, 1), pc=0x40001234)
    decoded = Request.unpack(req.pack())
    assert decoded == req
    v1_payload = REQUEST_STRUCT_V1.pack(MAGIC, 1, req.op, req.size, req.wstrb, req.addr, req.data, req.pc)
    assert Request.unpack(v1_payload).cpu_cycles == 0
    assert canonical_addr(decoded.addr) == 0x5000F100
    assert window_for_addr(decoded.addr).name == "scsi_dma"  # type: ignore[union-attr]
    resp = Response(RESP_OKAY, data=0x5A, cycles=17)
    assert Response.unpack(resp.pack()) == resp


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--dump-windows", action="store_true")
    args = ap.parse_args(argv)

    if args.selftest:
        selftest()
        print("mame_rtl_bridge_protocol selftest passed")
    if args.dump_windows:
        for w in WINDOWS:
            print(f"{w.name:10s} 0x{w.base:08x}..0x{w.base + w.size - 1:08x}")
    if not args.selftest and not args.dump_windows:
        ap.print_help(sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
