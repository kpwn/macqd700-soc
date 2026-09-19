#!/usr/bin/env python3
"""Host-side daemon for the Ethernet-backed virtual HDD: serve a disk image over UDP.

The FPGA is always the initiator.  It sends NBHD read/write requests from
``rtl/board/net_block_framer.sv`` and this daemon answers them out of a raw
disk image file.  Nothing here touches the FPGA, JTAG, or any RTL -- it is an
ordinary unprivileged UDP socket.

====================================================================
READ THIS FIRST: THERE IS DELIBERATELY NO ARP ON THE FPGA SIDE
====================================================================

The FPGA implements no ARP at all -- neither requester nor responder.  That is
fine for the FPGA's own transmits, because its next-hop MAC is a hardcoded,
JTAG-loadable CSR.  It is NOT fine for yours.

    The FPGA always initiates, but the HOST KERNEL must still resolve the
    FPGA's IP address before it can build the Ethernet header of a reply.

With no neighbour-table entry, your kernel will ARP for the FPGA's IP, get
silence (nothing on the board answers), and then drop every single reply
before it reaches the wire.  ``tcpdump`` shows the requests arriving and no
replies leaving.  That is indistinguishable from a broken FPGA receive path,
a wrong port, a bad checksum, or a dead MAC -- and it will burn an afternoon.

Before you run this daemon, on the host, once per boot:

    ip neigh replace <fpga-ip> lladdr <fpga-mac> dev <iface> nud permanent

If replies ever stop working after a host reboot, a NIC swap, an interface
rename, or a link flap, check ``ip neigh show`` FIRST.  A stale or evicted
entry has exactly the same symptom.

====================================================================
Wire format
====================================================================

``rtl/board/net_block_framer.sv`` is the authority; this is a transcription.
Ethernet / IPv4 / UDP, our own header inside the UDP payload, every multi-byte
field in network (big-endian) byte order.  Offsets are within the UDP payload:

    Request   0..3   magic        0x4e424844 ("NBHD")
              4      op           0x00 read, 0x01 write
              5..8   lba          uint32
              9..10  block_count  uint16
              11..12 tag          uint16
              13..   write data   block_count * 512 bytes; absent on read

    Reply     0..3   magic        0x4e424844 ("NBHD")
              4..5   tag          uint16, echoed from the request
              6      status       0x00 success, other values daemon-defined
              7..    payload      block_count * 512 bytes on a successful read

Both directions are capped at two 512-byte blocks per datagram by the 1500-byte
IPv4 MTU.  The RTL enforces this on its own transmit path, and its receive path
rejects anything that arrived fragmented, so a larger reply would not be
reassembled -- it would simply vanish.  The transaction layer above the framer
is responsible for windowing big transfers; this daemon refuses anything wider
with a distinct status rather than fragmenting or truncating.

The FPGA transmits its UDP checksum as zero, which IPv4 defines as "not
computed" (RFC 768).  A normal SOCK_DGRAM socket accepts that, so no special
handling is needed here -- but do not "fix" it by adding a checksum check.

The FPGA's own receive path DOES validate the IPv4 header checksum of our
replies and rejects IPv4 options and any fragmentation.  The kernel gets all of
that right for us, provided the outgoing interface MTU is at least 1500; on a
tunnel or a reduced-MTU link the kernel would fragment a 2-block read reply and
the FPGA would drop it.
"""

from __future__ import annotations

import argparse
import logging
import os
import socket
import struct
import time
from dataclasses import dataclass
from typing import BinaryIO, Optional, Sequence


LOGGER = logging.getLogger("net_vhdd")

MAGIC = b"NBHD"                     # 0x4e424844
OP_READ = 0x00
OP_WRITE = 0x01

BLOCK_BYTES = 512
REQUEST_HEADER_BYTES = 13
REPLY_HEADER_BYTES = 7

# Derived from the RTL, not chosen here.  net_block_framer.sv computes
#   MAX_REPLY_PAYLOAD = 1500 - 20 (IPv4) - 8 (UDP) - 7 (reply header) = 1465
#   MAX_WRITE_PAYLOAD = 1500 - 20 (IPv4) - 8 (UDP) - 13 (request header) = 1459
# so both directions fit exactly two 512-byte blocks and no more.
MAX_REPLY_PAYLOAD_BYTES = 1465
MAX_WRITE_PAYLOAD_BYTES = 1459
MAX_BLOCKS_PER_DATAGRAM = min(
    MAX_REPLY_PAYLOAD_BYTES, MAX_WRITE_PAYLOAD_BYTES
) // BLOCK_BYTES

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 4011                 # must equal the FPGA's dst_port CSR
DEFAULT_STATS_INTERVAL = 5.0

# Status byte.  The RTL calls every non-zero value "daemon-defined", so these
# are ours.  Every refusal gets its own code: on the wire a refusal must be
# distinguishable from a lost datagram, and from every other kind of refusal,
# or hardware bring-up degenerates into guesswork.
STATUS_SUCCESS = 0x00
STATUS_BAD_MAGIC = 0x01             # not an NBHD datagram
STATUS_UNKNOWN_OP = 0x02            # op is neither read nor write
STATUS_OUT_OF_RANGE = 0x03          # lba/count runs off the end of the image
STATUS_READ_ONLY = 0x04             # write refused, daemon started --read-only
STATUS_BAD_LENGTH = 0x05            # datagram length disagrees with the header
STATUS_TOO_MANY_BLOCKS = 0x06       # would not fit an unfragmented datagram
STATUS_IO_ERROR = 0x07              # the backing file failed us

STATUS_NAMES = {
    STATUS_SUCCESS: "success",
    STATUS_BAD_MAGIC: "bad-magic",
    STATUS_UNKNOWN_OP: "unknown-op",
    STATUS_OUT_OF_RANGE: "out-of-range",
    STATUS_READ_ONLY: "read-only",
    STATUS_BAD_LENGTH: "bad-length",
    STATUS_TOO_MANY_BLOCKS: "too-many-blocks",
    STATUS_IO_ERROR: "io-error",
}

OP_NAMES = {OP_READ: "read", OP_WRITE: "write"}

_REQUEST_HEADER = struct.Struct("!4sBIHH")   # magic, op, lba, block_count, tag
_REPLY_HEADER = struct.Struct("!4sHB")       # magic, tag, status


@dataclass(frozen=True)
class Outcome:
    """Everything the log line and the reply datagram both need."""

    tag: int
    op: Optional[int]
    lba: int
    block_count: int
    status: int
    payload: bytes = b""

    @property
    def op_name(self) -> str:
        if self.op is None:
            return "?"
        return OP_NAMES.get(self.op, "op=0x%02x" % self.op)

    @property
    def status_name(self) -> str:
        return STATUS_NAMES.get(self.status, "status=0x%02x" % self.status)


def encode_reply(tag: int, status: int, payload: bytes = b"") -> bytes:
    """Build one reply UDP payload."""
    return _REPLY_HEADER.pack(MAGIC, tag & 0xFFFF, status & 0xFF) + payload


def image_size(image: BinaryIO) -> int:
    """Size of the backing file, without disturbing its file position."""
    try:
        return os.fstat(image.fileno()).st_size
    except (AttributeError, OSError):
        position = image.tell()
        try:
            image.seek(0, os.SEEK_END)
            return image.tell()
        finally:
            image.seek(position)


def _read_exact(image: BinaryIO, offset: int, length: int) -> bytes:
    image.seek(offset)
    chunks = []
    remaining = length
    while remaining > 0:
        chunk = image.read(remaining)
        if not chunk:
            break
        chunks.append(chunk)
        remaining -= len(chunk)
    if remaining:
        raise OSError("short read at offset %d (%d of %d bytes)"
                      % (offset, length - remaining, length))
    return b"".join(chunks)


def _write_exact(image: BinaryIO, offset: int, data: bytes) -> None:
    image.seek(offset)
    view = memoryview(data)
    while view:
        written = image.write(view)
        if not written:
            raise OSError("short write at offset %d" % offset)
        view = view[written:]
    image.flush()


def process_request(
    datagram: bytes,
    image: BinaryIO,
    *,
    read_only: bool = False,
) -> Outcome:
    """Validate and execute one request; never raises, never does network I/O.

    Returns an :class:`Outcome` carrying both the reply fields and the values
    the operational log wants.  Malformed requests still produce an Outcome
    with a non-zero status: silence is reserved for genuinely lost packets, so
    that the FPGA can tell "refused" from "dropped".
    """
    if len(datagram) < REQUEST_HEADER_BYTES:
        # Too short to hold a tag, so there is nothing honest to echo.
        return Outcome(0, None, 0, 0, STATUS_BAD_LENGTH)

    magic, op, lba, block_count, tag = _REQUEST_HEADER.unpack_from(datagram)

    if magic != MAGIC:
        # Best-effort tag echo: the bytes are in the right place even though we
        # do not trust the datagram.  Worst case the FPGA sees a tag it is not
        # waiting on and ignores the reply, which is what we want anyway.
        return Outcome(tag, None, lba, block_count, STATUS_BAD_MAGIC)

    if op not in (OP_READ, OP_WRITE):
        return Outcome(tag, None, lba, block_count, STATUS_UNKNOWN_OP)

    # Policy refusal before validation: under --read-only no write will ever be
    # executed, so say so plainly rather than reporting some downstream detail
    # of a request we were never going to run.
    if op == OP_WRITE and read_only:
        return Outcome(tag, op, lba, block_count, STATUS_READ_ONLY)

    if block_count == 0 or block_count > MAX_BLOCKS_PER_DATAGRAM:
        # A read reply is capped by the same MTU as a write request, and the
        # FPGA drops fragments, so an over-wide read is refused too.
        return Outcome(tag, op, lba, block_count, STATUS_TOO_MANY_BLOCKS)

    transfer_bytes = block_count * BLOCK_BYTES
    expected_len = REQUEST_HEADER_BYTES + (transfer_bytes if op == OP_WRITE else 0)
    if len(datagram) != expected_len:
        return Outcome(tag, op, lba, block_count, STATUS_BAD_LENGTH)

    offset = lba * BLOCK_BYTES
    if offset + transfer_bytes > image_size(image):
        return Outcome(tag, op, lba, block_count, STATUS_OUT_OF_RANGE)

    try:
        if op == OP_READ:
            payload = _read_exact(image, offset, transfer_bytes)
            return Outcome(tag, op, lba, block_count, STATUS_SUCCESS, payload)
        _write_exact(image, offset, datagram[REQUEST_HEADER_BYTES:])
        return Outcome(tag, op, lba, block_count, STATUS_SUCCESS)
    except (OSError, ValueError) as exc:
        LOGGER.error("tag=0x%04x %s lba=%d count=%d backing store failed: %s",
                     tag, OP_NAMES.get(op, "?"), lba, block_count, exc)
        return Outcome(tag, op, lba, block_count, STATUS_IO_ERROR)


def handle_request(
    datagram: bytes,
    image: BinaryIO,
    *,
    read_only: bool = False,
) -> bytes:
    """Request UDP payload in, reply UDP payload out.  No sockets involved."""
    outcome = process_request(datagram, image, read_only=read_only)
    return encode_reply(outcome.tag, outcome.status, outcome.payload)


class _Stats:
    """Counters for the periodic summary line."""

    def __init__(self) -> None:
        self.reads = 0
        self.writes = 0
        self.blocks = 0
        self.refused = 0

    def record(self, outcome: Outcome) -> None:
        if outcome.status != STATUS_SUCCESS:
            self.refused += 1
            return
        if outcome.op == OP_WRITE:
            self.writes += 1
        else:
            self.reads += 1
        self.blocks += outcome.block_count

    @property
    def total(self) -> int:
        return self.reads + self.writes + self.refused

    def summary(self) -> str:
        return ("%d read %d write %d refused, %d blocks (%.1f KiB)"
                % (self.reads, self.writes, self.refused, self.blocks,
                   self.blocks * BLOCK_BYTES / 1024.0))


def serve(
    image: BinaryIO,
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    *,
    read_only: bool = False,
    stats_interval: float = DEFAULT_STATS_INTERVAL,
) -> None:
    """Bind a UDP socket and answer requests until interrupted."""
    stats = _Stats()
    last_report = time.monotonic()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((host, port))
        LOGGER.info("serving %d bytes on udp://%s:%d (%s, max %d blocks/datagram)",
                    image_size(image), host, port,
                    "read-only" if read_only else "read-write",
                    MAX_BLOCKS_PER_DATAGRAM)
        LOGGER.info("reminder: the FPGA has no ARP -- "
                    "ip neigh replace <fpga-ip> lladdr <fpga-mac> "
                    "dev <iface> nud permanent")
        while True:
            datagram, peer = sock.recvfrom(65535)
            outcome = process_request(datagram, image, read_only=read_only)
            stats.record(outcome)

            # Per-request lines are DEBUG so a multi-megabyte transfer does not
            # bury the interesting events; refusals are always shown.
            line = ("peer=%s:%d tag=0x%04x op=%s lba=%d count=%d status=%s(0x%02x)"
                    % (peer[0], peer[1], outcome.tag, outcome.op_name,
                       outcome.lba, outcome.block_count,
                       outcome.status_name, outcome.status))
            if outcome.status == STATUS_SUCCESS:
                LOGGER.debug("%s len=%d", line, len(outcome.payload))
            else:
                LOGGER.warning("%s len=%d", line, len(datagram))

            try:
                sock.sendto(encode_reply(outcome.tag, outcome.status,
                                         outcome.payload), peer)
            except OSError as exc:
                # The classic cause is the missing static ARP entry above.
                LOGGER.error("reply for tag=0x%04x to %s:%d not sent: %s "
                             "(missing 'ip neigh replace' entry?)",
                             outcome.tag, peer[0], peer[1], exc)

            now = time.monotonic()
            if stats_interval > 0 and now - last_report >= stats_interval:
                if stats.total:
                    LOGGER.info("%s", stats.summary())
                last_report = now


ARP_EPILOG = """\
THERE IS DELIBERATELY NO ARP ON THE FPGA SIDE.

The FPGA always initiates, but your kernel must still resolve the FPGA's IP
address to build the Ethernet header of every reply.  With no neighbour entry
it will ARP, get silence, and drop every reply before it reaches the wire --
requests visible in tcpdump, replies never leaving.  That looks exactly like a
broken FPGA receive path and will waste an afternoon.

Add the static neighbour entry BEFORE starting a transfer:

    ip neigh replace <fpga-ip> lladdr <fpga-mac> dev <iface> nud permanent

If replies stop after a reboot, NIC change or link flap, check 'ip neigh show'
first: a stale or evicted entry has the identical symptom.

--port must match the FPGA's dst_port CSR.  Both directions are limited to two
512-byte blocks per datagram by the 1500-byte IPv4 MTU; wider requests are
refused with status 0x%02x rather than fragmented.
""" % STATUS_TOO_MANY_BLOCKS


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="net_vhdd_server.py",
        description="Serve a raw disk image to the FPGA's Ethernet virtual HDD "
                    "over UDP (protocol: rtl/board/net_block_framer.sv).",
        epilog=ARP_EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("image", help="path to the raw disk image to serve")
    parser.add_argument("--host", default=DEFAULT_HOST,
                        help="local address to bind (default: %(default)s)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help="local UDP port to bind; must match the FPGA's "
                             "dst_port CSR (default: %(default)s)")
    parser.add_argument("--read-only", action="store_true",
                        help="serve reads, refuse every write with status "
                             "0x%02x" % STATUS_READ_ONLY)
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="log every request, not just refusals")
    parser.add_argument("--stats-interval", type=float,
                        default=DEFAULT_STATS_INTERVAL, metavar="SECONDS",
                        help="periodic summary interval, 0 to disable "
                             "(default: %(default)s)")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_argument_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
    )
    try:
        with open(args.image, "rb" if args.read_only else "r+b") as image:
            serve(image, args.host, args.port,
                  read_only=args.read_only,
                  stats_interval=args.stats_interval)
    except KeyboardInterrupt:
        LOGGER.info("stopped")
    except OSError as exc:
        LOGGER.error("%s", exc)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
