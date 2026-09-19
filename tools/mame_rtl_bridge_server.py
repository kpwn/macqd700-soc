#!/usr/bin/env python3
"""Prototype endpoint for the MAME RTL bridge protocol.

This server is intentionally a mock responder.  It proves the blocking socket
shape expected by a custom MAME MMIO device while the Verilated endpoint is
being split out.  Unknown windows return DECERR; known read windows return a
deterministic value derived from the canonical address.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import socket

from mame_rtl_bridge_protocol import (
    OP_READ,
    OP_WRITE,
    REQUEST_STRUCT,
    RESP_DECERR,
    RESP_OKAY,
    Request,
    Response,
    canonical_addr,
    window_for_addr,
)


def recv_exact(conn: socket.socket, size: int) -> bytes | None:
    chunks = []
    remaining = size
    while remaining:
        chunk = conn.recv(remaining)
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def handle_request(req: Request) -> Response:
    ca = canonical_addr(req.addr)
    window = window_for_addr(req.addr)
    if window is None:
        return Response(RESP_DECERR, data=0, cycles=0)
    if req.op == OP_WRITE:
        return Response(RESP_OKAY, data=0, cycles=1)
    if req.op == OP_READ:
        byte = ((ca >> 2) ^ (ca >> 11) ^ len(window.name)) & 0xFF
        data = byte * 0x01010101
        return Response(RESP_OKAY, data=data, cycles=1)
    return Response(RESP_DECERR, data=0, cycles=0)


def serve(path: Path) -> None:
    if path.exists():
        path.unlink()
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as srv:
        srv.bind(str(path))
        srv.listen(1)
        print(f"mame RTL bridge mock listening on {path}")
        while True:
            conn, _ = srv.accept()
            with conn:
                while True:
                    payload = recv_exact(conn, REQUEST_STRUCT.size)
                    if payload is None:
                        break
                    req = Request.unpack(payload)
                    conn.sendall(handle_request(req).pack())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--socket", type=Path, default=Path("/tmp/mame-rtl-bridge.sock"))
    args = ap.parse_args()
    serve(args.socket)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
